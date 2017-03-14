/*
 * Copyright (C) 2016 by Argonne National Laboratory.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <unistd.h>


#include "rdma/bgq/fi_bgq_agent.h"
#include "rdma/bgq/fi_bgq_compiler.h"


#define FI_BGQ_AGENT_MU_INJFIFO_BYTES	(1024*64)
#define FI_BGQ_AGENT_MU_INJFIFO_COUNT	(12)

#define FI_BGQ_AGENT_MU_RECFIFO_BYTES	(1024*32)
#define FI_BGQ_AGENT_MU_RECFIFO_COUNT	(1)


struct fi_bgq_agent_immediate_payload {
	uint8_t			byte[64];	/* TODO - make this bigger? */
} __attribute__((__aligned__(64)));

struct fi_bgq_agent_internal {
	uint64_t	rfifo_id[FI_BGQ_AGENT_MU_RECFIFO_COUNT];
	MUSPI_RecFifo_t	*rfifo[FI_BGQ_AGENT_MU_RECFIFO_COUNT];
	MUSPI_RecFifoSubGroup_t	rfifo_subgroup;

	struct fi_bgq_spi_injfifo	ififo[FI_BGQ_AGENT_MU_INJFIFO_COUNT];
	MUSPI_InjFifoSubGroup_t		ififo_subgroup[FI_BGQ_AGENT_MU_INJFIFO_COUNT];

	BG_CoordinateMapping_t		coords;
	MUHWI_Destination_t		local;

	uint64_t			zero;
	uint64_t			zero_paddr;

	uint64_t			discard;
	uint64_t			discard_paddr;

	void *				rfifo_mem;
	void *				ififo_mem;

	struct l2atomic_lock		mu_lock;
	struct fi_bgq_node		node;
};


uint32_t __line__;
#define GOTO_ERR	({__line__=__LINE__; goto err;})

MUHWI_Descriptor_t __rget_model__;
MUHWI_Descriptor_t __dput_model__;
MUHWI_Descriptor_t __decr_model__;


int fi_bgq_node_init_no_barrier (struct fi_bgq_node * node);


int fi_bgq_agent_init (struct fi_bgq_agent_internal *agent)
{
	int rc;
	uint32_t i;

	rc = fi_bgq_node_init_no_barrier(&agent->node);
	if (rc) GOTO_ERR;

	rc = fi_bgq_node_mu_lock_init(&agent->node, &agent->mu_lock);
	if (rc) GOTO_ERR;

	l2atomic_lock_acquire(&agent->mu_lock);

	/*
	 * Create mu reception fifo(s).
	 */
	uint8_t * memptr;
	size_t nbytes = FI_BGQ_AGENT_MU_RECFIFO_BYTES * FI_BGQ_AGENT_MU_RECFIFO_COUNT;
	rc = posix_memalign((void**)&memptr, 32, nbytes);
	if (rc) goto err;

	Kernel_MemoryRegion_t mregion;
	rc = Kernel_CreateMemoryRegion(&mregion, (void*)memptr, nbytes);
	if (rc) goto err;

	agent->rfifo_mem = (void*)memptr;
	for (i = 0; i < FI_BGQ_AGENT_MU_RECFIFO_COUNT; ++i)
		agent->rfifo[i] = NULL;

	/* allocate recfifo(s) from 65th subgroup */
	const uint32_t requested_subgroup = 65;

	uint32_t free_fifo_num;
	uint32_t free_fifo_ids[BGQ_MU_NUM_REC_FIFOS_PER_SUBGROUP];
	rc = Kernel_QueryRecFifos(requested_subgroup, &free_fifo_num, free_fifo_ids);
	if (rc) goto err;
	if (free_fifo_num < FI_BGQ_AGENT_MU_RECFIFO_COUNT) goto err;

	Kernel_RecFifoAttributes_t rfifo_attrs[FI_BGQ_AGENT_MU_RECFIFO_COUNT];
	memset((void*)&rfifo_attrs[0], 0, sizeof(Kernel_RecFifoAttributes_t)*FI_BGQ_AGENT_MU_RECFIFO_COUNT);
	rc = Kernel_AllocateRecFifos(requested_subgroup,
		&agent->rfifo_subgroup,
		FI_BGQ_AGENT_MU_RECFIFO_COUNT, free_fifo_ids, rfifo_attrs);
	if (rc) goto err;

	for (i = 0; i < FI_BGQ_AGENT_MU_RECFIFO_COUNT; ++i) {
		rc = Kernel_RecFifoInit(&agent->rfifo_subgroup,
			i,
			&mregion,
			((uint64_t)memptr) - (uint64_t)mregion.BaseVa,
			FI_BGQ_AGENT_MU_RECFIFO_BYTES - 1);
		if (rc) goto err;

		memptr += FI_BGQ_AGENT_MU_RECFIFO_BYTES;
	}

	uint64_t shift = (BGQ_MU_NUM_REC_FIFOS_PER_GROUP-1) -
		((requested_subgroup&3)*BGQ_MU_NUM_FIFO_SUBGROUPS);

	for (i = 0; i < FI_BGQ_AGENT_MU_RECFIFO_COUNT; ++i) {
		rc = Kernel_RecFifoEnable(requested_subgroup>>2, 0x01ULL << (shift-i));
		if (rc) goto err;

		agent->rfifo_id[i] = (requested_subgroup * BGQ_MU_NUM_REC_FIFOS_PER_SUBGROUP) + free_fifo_ids[i];
		assert(260 == agent->rfifo_id[i]);	/* TODO - how is this communicated with the application processes? */

		agent->rfifo[i] = &agent->rfifo_subgroup._recfifos[free_fifo_ids[i]];
	}


	/*
	 * Create mu injection fifo(s).
	 *
	 * Allocate injection fifos from subgroups 64 and 65; it does not matter
	 * which specific injection fifos are allocated.
	 */
	unsigned subgrp_id = 65;
	for (i = 0; i < FI_BGQ_AGENT_MU_INJFIFO_COUNT; ++i) {

		if (1 != fi_bgq_spi_injfifo_subgrp_init(&agent->ififo[i],
				&agent->ififo_subgroup[i],
				1, FI_BGQ_AGENT_MU_INJFIFO_BYTES, 64, 0, subgrp_id)) {
			if (subgrp_id == 65) {
				subgrp_id--;
			} else {
				abort();
			}
		}
	}


	Personality_t personality;
	if (Kernel_GetPersonality(&personality, sizeof(Personality_t))) goto err;

	agent->local.Destination.A_Destination = agent->coords.a = personality.Network_Config.Acoord;
	agent->local.Destination.B_Destination = agent->coords.b = personality.Network_Config.Bcoord;
	agent->local.Destination.C_Destination = agent->coords.c = personality.Network_Config.Ccoord;
	agent->local.Destination.D_Destination = agent->coords.d = personality.Network_Config.Dcoord;
	agent->local.Destination.E_Destination = agent->coords.e = personality.Network_Config.Ecoord;
	agent->local.Destination.Reserved2 = 0;

	agent->zero = 0;
	fi_bgq_cnk_vaddr2paddr((const void *)&agent->zero, sizeof(uint64_t), &agent->zero_paddr);

	agent->discard = 0;
	fi_bgq_cnk_vaddr2paddr((const void *)&agent->discard, sizeof(uint64_t), &agent->discard_paddr);

	/*
	 * Initialize the memory-fifo "rget" descriptor model
	 */
	{
		MUHWI_Descriptor_t * model = &__rget_model__;

		MUSPI_DescriptorZeroOut(model);

		model->Half_Word0.Prefetch_Only = MUHWI_DESCRIPTOR_PRE_FETCH_ONLY_NO;
		model->Half_Word1.Interrupt = MUHWI_DESCRIPTOR_DO_NOT_INTERRUPT_ON_PACKET_ARRIVAL;
		model->PacketHeader.NetworkHeader.pt2pt.Data_Packet_Type = MUHWI_PT2PT_DATA_PACKET_TYPE;
		model->PacketHeader.NetworkHeader.pt2pt.Byte3.Byte3 = MUHWI_PACKET_VIRTUAL_CHANNEL_DETERMINISTIC;
		model->PacketHeader.NetworkHeader.pt2pt.Byte8.Byte8 = MUHWI_PACKET_TYPE_FIFO;
		model->PacketHeader.NetworkHeader.pt2pt.Byte8.Size = 16;
		model->Pa_Payload = 0;
		model->Message_Length = 0;
		model->PacketHeader.messageUnitHeader.Packet_Types.Memory_FIFO.Rec_FIFO_Id = agent->rfifo_id[0];

		struct fi_bgq_agent_packet * const packet =
			(struct fi_bgq_agent_packet * const) &model->PacketHeader;
		fi_bgq_agent_packet_set_kind(packet, FI_BGQ_AGENT_PACKET_KIND_RGET);

		packet->header.rget.origin = agent->local;

		/* specified at injection time */
		model->Torus_FIFO_Map = 0;
		model->PacketHeader.NetworkHeader.pt2pt.Destination.Destination.Destination = -1;
		packet->header.rget.fifo_map = 0;
		packet->header.rget.inj_fifo = 0;
		packet->header.rget.niov = 0;
		packet->header.rget.rec_cntr_batid = 1023;
		packet->header.rget.inj_cntr_paddr_rsh3b = 0;
		packet->header.rget.rec_cntr_offset_rsh3b = 0;
	}

	/*
	 * Initialize the direct-put descriptor model for rget data transfer
	 */
	{
		MUHWI_Descriptor_t * model = &__dput_model__;

		MUSPI_DescriptorZeroOut(model);

		model->Half_Word0.Prefetch_Only = MUHWI_DESCRIPTOR_PRE_FETCH_ONLY_NO;
		model->Half_Word1.Interrupt = MUHWI_DESCRIPTOR_DO_NOT_INTERRUPT_ON_PACKET_ARRIVAL;
		model->PacketHeader.NetworkHeader.pt2pt.Data_Packet_Type = MUHWI_PT2PT_DATA_PACKET_TYPE;
		model->PacketHeader.NetworkHeader.pt2pt.Byte3.Byte3 = MUHWI_PACKET_VIRTUAL_CHANNEL_DETERMINISTIC;
		model->PacketHeader.NetworkHeader.pt2pt.Byte8.Byte8 = MUHWI_PACKET_TYPE_PUT;
		model->PacketHeader.NetworkHeader.pt2pt.Byte8.Size = 16;

		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Pacing = MUHWI_PACKET_DIRECT_PUT_IS_NOT_PACED;

		/* specified at injection time */
		model->Torus_FIFO_Map = 0;
		model->Pa_Payload = 0;
		model->Message_Length = 0;
		model->PacketHeader.NetworkHeader.pt2pt.Destination.Destination.Destination = -1;
		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Rec_Counter_Base_Address_Id = 0;
		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Counter_Offset = 0;
		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Rec_Payload_Base_Address_Id = 0;
		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Put_Offset_MSB = 0;
		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Put_Offset_LSB = 0;
	}


	/*
	 * Initialize the direct-put "decrement send byte count" descriptor model
	 */
	{
		MUHWI_Descriptor_t * model = &__decr_model__;

		MUSPI_DescriptorZeroOut(model);

		model->Half_Word0.Prefetch_Only = MUHWI_DESCRIPTOR_PRE_FETCH_ONLY_NO;
		model->Half_Word1.Interrupt = MUHWI_DESCRIPTOR_DO_NOT_INTERRUPT_ON_PACKET_ARRIVAL;
		model->PacketHeader.NetworkHeader.pt2pt.Data_Packet_Type = MUHWI_PT2PT_DATA_PACKET_TYPE;
		model->PacketHeader.NetworkHeader.pt2pt.Byte3.Byte3 = MUHWI_PACKET_VIRTUAL_CHANNEL_DETERMINISTIC;
		model->PacketHeader.NetworkHeader.pt2pt.Byte8.Byte8 = MUHWI_PACKET_TYPE_PUT;
		model->PacketHeader.NetworkHeader.pt2pt.Byte8.Size = 16;

		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Pacing = MUHWI_PACKET_DIRECT_PUT_IS_NOT_PACED;
		model->Torus_FIFO_Map = MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_LOCAL0 | MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_LOCAL1;
		model->Message_Length = 8;
		model->PacketHeader.NetworkHeader.pt2pt.Destination.Destination.Destination = -1;	/* Destination not used for local */
		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Rec_Counter_Base_Address_Id = FI_BGQ_NODE_BAT_ID_GLOBAL_STOREADD;
		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Counter_Offset = agent->discard_paddr;
		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Rec_Payload_Base_Address_Id = FI_BGQ_NODE_BAT_ID_GLOBAL_STOREADD;

		/* specified at injection time */
		model->Pa_Payload = 0;									/* nbytes */
		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Put_Offset_MSB = 0;	/* inj_cntr_paddr */
		model->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Put_Offset_LSB = 0;	/* inj_cntr_paddr */
	}


	/*
	 * Synchronize all of the application agents.
	 */
	rc = MUSPI_GIBarrierEnterAndWait(&agent->node.gi_barrier);
	if (rc) GOTO_ERR;

	l2atomic_lock_release(&agent->mu_lock);

	/*
	 * Synchonize agents and application processes on the node.
	 *
	 * Since the agents have been synchronized across nodes already, when
	 * the application processes exit this "agent barrier" process-to-agent
	 * communication can be safely initiated.
	 */
	l2atomic_barrier_enter(&agent->node.barrier[FI_BGQ_NODE_BARRIER_KIND_AGENT]);

	return 0;
err:
	fprintf(stderr, "%s:%s():%d Error.\n", __FILE__, __func__, __line__);
	return -1;
}

int fi_bgq_agent_fini (struct fi_bgq_agent_internal *agent) {

	return 0;
}


static inline
void fi_bgq_agent_packet_process_rget (struct fi_bgq_agent_packet const * const packet,
		struct fi_bgq_agent_internal *agent) {

	assert(fi_bgq_agent_packet_get_kind(packet) == FI_BGQ_AGENT_PACKET_KIND_RGET);

	const uint64_t fifo_map = packet->header.rget.fifo_map;
	const MUHWI_Destination_t destination = packet->header.rget.origin;
	const unsigned index = packet->header.rget.inj_fifo;
	const unsigned niov = packet->header.rget.niov;
	const uint16_t rec_cntr_batid = packet->header.rget.rec_cntr_batid;
	const uint64_t rec_cntr_offset = ((uint64_t)packet->header.rget.rec_cntr_offset_rsh3b) << 3;
	const uint64_t inj_cntr_paddr = ((uint64_t)packet->header.rget.inj_cntr_paddr_rsh3b) << 3;

	struct fi_bgq_spi_injfifo * f = &agent->ififo[index];
	int64_t tbytes = 0;

	unsigned i;
	for (i=0; i<niov; ++i) {

		const uint64_t tmp0 = packet->payload.iov[i].raw64[0];
		const uint64_t tmp1 = packet->payload.iov[i].raw64[1];

		const uint64_t rec_data_offset = tmp0 & 0x03FFFFFFFFul;
		const uint64_t rec_data_batid = (tmp0 >> 38) & 0x03FFul;

		const uint64_t src_paddr = tmp1 & 0x03FFFFFFFFul;

		const uint64_t nbytes = (tmp1 >> 48) | ((tmp0 >> 32) & 0x00000000FFFF0000ul);
		tbytes -= (int64_t)nbytes;

		MUHWI_Descriptor_t * desc = fi_bgq_spi_injfifo_tail_wait(f);

		/* copy the descriptor model into the injection fifo */
		qpx_memcpy64((void*)desc, (const void*)&__dput_model__);

		desc->Torus_FIFO_Map = fifo_map;
		desc->Pa_Payload = src_paddr;
		desc->Message_Length = nbytes;
		desc->PacketHeader.NetworkHeader.pt2pt.Destination = destination;
		desc->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Rec_Counter_Base_Address_Id = rec_cntr_batid;
		desc->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Counter_Offset = rec_cntr_offset;

		MUSPI_SetRecPayloadBaseAddressInfo(desc, rec_data_batid, rec_data_offset);
		/* DUMP_DESCRIPTOR(desc); */
		MUSPI_InjFifoAdvanceDesc(f->muspi_injfifo);
	}

	if (inj_cntr_paddr != 0) {

		MUHWI_Descriptor_t * desc = fi_bgq_spi_injfifo_tail_wait(f);

		/* copy the descriptor model into the injection fifo */
		qpx_memcpy64((void*)desc, (const void*)&__decr_model__);

		uint64_t *payload = (uint64_t*) fi_bgq_spi_injfifo_immediate_payload(f, desc, &desc->Pa_Payload);

		*payload = (uint64_t)tbytes;

		MUSPI_SetRecPayloadBaseAddressInfo(desc, FI_BGQ_NODE_BAT_ID_GLOBAL_STOREADD, inj_cntr_paddr);
		/* DUMP_DESCRIPTOR(desc); */
		MUSPI_InjFifoAdvanceDesc(f->muspi_injfifo);
	}
}


void fi_bgq_agent_packet_process_noinline(struct fi_bgq_agent_packet const * const packet,
		struct fi_bgq_agent_internal *agent,
		const enum fi_bgq_agent_packet_kind kind) {

	switch (kind) {
		case FI_BGQ_AGENT_PACKET_KIND_RGET:
			fi_bgq_agent_packet_process_rget(packet, agent);
			break;
		default:
			assert(0);	/* Whoops! */
	}
}


static inline
void fi_bgq_agent_packet_process (MUHWI_PacketHeader_t * hdr, struct fi_bgq_agent_internal *agent) {

	struct fi_bgq_agent_packet const * const packet =
		(struct fi_bgq_agent_packet *)hdr;

	const enum fi_bgq_agent_packet_kind kind = fi_bgq_agent_packet_get_kind(packet);
	if (kind == FI_BGQ_AGENT_PACKET_KIND_RGET) {	/* likely */
		fi_bgq_agent_packet_process_rget(packet, agent);
	} else {
		fi_bgq_agent_packet_process_noinline(packet, agent, kind);
	} 
}


static inline
void fi_bgq_agent_poll (struct fi_bgq_agent_internal *agent, const uint32_t rx) {

	assert(rx < FI_BGQ_AGENT_MU_RECFIFO_COUNT);

	/*
	 * The mu reception fifo is consumed by software at the 'head' and
	 * produced by hardware at the 'tail'.
	 */
	MUSPI_Fifo_t * fifo_ptr = &agent->rfifo[rx]->_fifo;
	assert(fifo_ptr);
	volatile uint64_t pa_tail = MUSPI_getHwTail(fifo_ptr);
	const uintptr_t pa_start = MUSPI_getStartPa(fifo_ptr);
	const uintptr_t offset_tail = pa_tail - pa_start;

	const uintptr_t va_head = (uintptr_t) MUSPI_getHeadVa(fifo_ptr);
	const uintptr_t va_start = (uintptr_t) MUSPI_getStartVa(fifo_ptr);
	const uintptr_t offset_head = va_head - va_start;

	MUHWI_PacketHeader_t * hdr = (MUHWI_PacketHeader_t *) va_head;

	if (offset_head < offset_tail) {			/* likely */

		muspi_dcbt(va_head, 0);
		fi_bgq_compiler_msync(FI_BGQ_COMPILER_MSYNC_KIND_RO);

		const uintptr_t stop = va_head + offset_tail - offset_head;
		int process_rfifo_iter = 0;
		while ((uintptr_t)hdr < stop) {

			process_rfifo_iter++;

			fi_bgq_agent_packet_process(hdr, agent);

			hdr += hdr->NetworkHeader.pt2pt.Byte8.Size + 1;
			muspi_dcbt(hdr, 0);
		}

		MUSPI_setHeadVa(fifo_ptr, (void*)hdr);
		MUSPI_setHwHead(fifo_ptr, (uintptr_t)hdr-va_start);

	} else if (offset_head > offset_tail) {			/* unlikely ? */

		/* check if the head packet wraps */
		const uintptr_t va_end = (uintptr_t) fifo_ptr->va_end;
		if ((va_head + 544) < va_end) {			/* likely */

			/* head packet does not wrap */
			muspi_dcbt(va_head, 0);
			fi_bgq_compiler_msync(FI_BGQ_COMPILER_MSYNC_KIND_RO);

			const uintptr_t stop = va_end - 544;
			int process_rfifo_iter = 0;
			while  ((uintptr_t)hdr < stop) {

				process_rfifo_iter++;

				fi_bgq_agent_packet_process(hdr, agent);

				hdr += hdr->NetworkHeader.pt2pt.Byte8.Size + 1;
				muspi_dcbt(hdr, 0);
			}

			MUSPI_setHeadVa(fifo_ptr, (void*)hdr);
			MUSPI_setHwHead(fifo_ptr, (uintptr_t)hdr-va_start);

		} else {					/* unlikely */

			/* head packet may wrap */
			muspi_dcbt(va_head, 0);
			fi_bgq_compiler_msync(FI_BGQ_COMPILER_MSYNC_KIND_RO);

			uint32_t packet_bytes = ((uint32_t)hdr->NetworkHeader.pt2pt.Byte8.Size + 1) << 5;
			const uintptr_t bytes_before_wrap = va_end - va_head;
			if (packet_bytes < bytes_before_wrap) {
				fi_bgq_agent_packet_process(hdr, agent);

				const uintptr_t new_offset_head = offset_head + packet_bytes;
				MUSPI_setHeadVa(fifo_ptr, (void*)(va_start + new_offset_head));
				MUSPI_setHwHead(fifo_ptr, new_offset_head);

			} else if (packet_bytes == bytes_before_wrap) {
				fi_bgq_agent_packet_process(hdr, agent);

				MUSPI_setHeadVa(fifo_ptr, (void*)(va_start));
				MUSPI_setHwHead(fifo_ptr, 0);

			} else {
				uint8_t tmp_pkt[544] __attribute__((__aligned__(32)));

				memcpy((void*)&tmp_pkt[0], (void*)va_head, bytes_before_wrap);
				const uintptr_t bytes_after_wrap = packet_bytes - bytes_before_wrap;
				memcpy((void*)&tmp_pkt[bytes_before_wrap], (void*)va_start, bytes_after_wrap);

				hdr = (MUHWI_PacketHeader_t *)&tmp_pkt[0];
				fi_bgq_agent_packet_process(hdr, agent);

				MUSPI_setHeadVa(fifo_ptr, (void*)(va_start + bytes_after_wrap));
				MUSPI_setHwHead(fifo_ptr, bytes_after_wrap);
			}
		}
	}

	return;
}

void * fi_bgq_agent_start (void * arg) {

	struct fi_bgq_agent_internal agent;
	if (fi_bgq_agent_init(&agent) == 0) {

		do {
			fi_bgq_agent_poll(&agent, 0);
		} while (1);
	}

	return 0;
}

pthread_t fi_bgq_agent_emulate (void * arg) {

	setbuf(stdout, NULL);
	setbuf(stderr, NULL);

	/* create a pthread, etc. */
	pthread_t thread;

	if (pthread_create(&thread, NULL, &fi_bgq_agent_start, NULL)) {
		abort();
	}

	return thread;
}



void fi_bgq_agent_register (struct fi_bgq_agent * agent) {

	Personality_t personality;
	if (Kernel_GetPersonality(&personality, sizeof(Personality_t))) abort();

	MUHWI_Destination_t local;
	local.Destination.A_Destination = personality.Network_Config.Acoord;
	local.Destination.B_Destination = personality.Network_Config.Bcoord;
	local.Destination.C_Destination = personality.Network_Config.Ccoord;
	local.Destination.D_Destination = personality.Network_Config.Dcoord;
	local.Destination.E_Destination = personality.Network_Config.Ecoord;
	local.Destination.Reserved2 = 0;

	/*
	 * Initialize the memory-fifo "rget" descriptor model
	 */
	{
		MUHWI_Descriptor_t * model = &agent->rget_model;

		MUSPI_DescriptorZeroOut(model);

		model->Half_Word0.Prefetch_Only = MUHWI_DESCRIPTOR_PRE_FETCH_ONLY_NO;
		model->Half_Word1.Interrupt = MUHWI_DESCRIPTOR_DO_NOT_INTERRUPT_ON_PACKET_ARRIVAL;
		model->PacketHeader.NetworkHeader.pt2pt.Data_Packet_Type = MUHWI_PT2PT_DATA_PACKET_TYPE;
		model->PacketHeader.NetworkHeader.pt2pt.Byte3.Byte3 = MUHWI_PACKET_VIRTUAL_CHANNEL_DETERMINISTIC;
		model->PacketHeader.NetworkHeader.pt2pt.Byte8.Byte8 = MUHWI_PACKET_TYPE_FIFO;
		model->PacketHeader.NetworkHeader.pt2pt.Byte8.Size = 16;
		model->Pa_Payload = 0;
		model->Message_Length = 0;
		model->PacketHeader.messageUnitHeader.Packet_Types.Memory_FIFO.Rec_FIFO_Id = 260; /* TODO - how is this communicated with the application processes? */

		struct fi_bgq_agent_packet * const packet =
			(struct fi_bgq_agent_packet * const) &model->PacketHeader;
		fi_bgq_agent_packet_set_kind(packet, FI_BGQ_AGENT_PACKET_KIND_RGET);

		packet->header.rget.origin = local;

		/* specified at injection time */
		model->Torus_FIFO_Map = 0;
		model->PacketHeader.NetworkHeader.pt2pt.Destination.Destination.Destination = -1;
		packet->header.rget.fifo_map = 0;
		packet->header.rget.inj_fifo = 0;
		packet->header.rget.niov = 0;
		packet->header.rget.rec_cntr_batid = 1023;
		packet->header.rget.inj_cntr_paddr_rsh3b = 0;
		packet->header.rget.rec_cntr_offset_rsh3b = 0;
	}
}


void fi_bgq_agent_rget_desc_init (MUHWI_Descriptor_t * desc,
		const struct fi_bgq_agent * const agent,
		const MUHWI_Destination_t destination, const uint16_t fifo_map,
		const uint8_t inj_fifo, const uint64_t rec_cntr_paddr,
		const uint64_t inj_cntr_paddr,
		const uint8_t niov, const uint64_t iov_paddr) {

	/* copy the descriptor model into the injection fifo */
	qpx_memcpy64((void*)desc, (const void*)&agent->rget_model);

	desc->Torus_FIFO_Map = fifo_map;
	desc->Pa_Payload = iov_paddr;
	desc->Message_Length = niov * sizeof(union fi_bgq_agent_rget_iov);
	desc->PacketHeader.NetworkHeader.pt2pt.Destination = destination;

	union fi_bgq_agent_packet_hdr * const header =
		(union fi_bgq_agent_packet_hdr * const) &desc->PacketHeader;
	header->rget.fifo_map = fifo_map;
	header->rget.inj_fifo = inj_fifo;
	header->rget.niov = niov;
	header->rget.rec_cntr_batid = FI_BGQ_NODE_BAT_ID_GLOBAL_STOREADD;
	header->rget.rec_cntr_offset_rsh3b = rec_cntr_paddr >> 3;
	header->rget.inj_cntr_paddr_rsh3b = inj_cntr_paddr >> 3;

	/* DUMP_DESCRIPTOR(desc); */

	return;
}
