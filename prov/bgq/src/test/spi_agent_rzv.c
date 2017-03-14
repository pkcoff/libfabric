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
#include "spi_util.h"


struct address {
	MUHWI_Destination_t	destination;
	uint16_t		rx;
	uint16_t		fifo_map;
} __attribute__((__packed__));


struct rts_hdr {
	uint64_t	reserved_0;
	uint32_t	reserved_1;
	uint16_t	reserved_2	: 10;
	uint16_t	unused_0	:  6;
	uint16_t	unused_1;
	uint16_t	nbytes_msb;
	uint16_t	nbytes_lsb;
	uint32_t	inj_cntr_paddr_rsh3b;
	uint64_t	src_paddr;
} __attribute__((__packed__));


uint8_t rbuf[1024*1024];
uint8_t sbuf[1024*1024];


MUHWI_Descriptor_t rts_model;

static inline
void inject_rts (struct fi_bgq_spi_injfifo * injfifo, struct address * addr, void * src, uint64_t nbytes, uint64_t * cc) {

	uint64_t src_paddr = 0;
	fi_bgq_cnk_vaddr2paddr((const void *)src, nbytes, &src_paddr);

	uint64_t cc_paddr = 0;
	fi_bgq_cnk_vaddr2paddr((const void *)cc, sizeof(uint64_t), &cc_paddr);

	assert((cc_paddr & 0x07u) == 0);

	MUHWI_Descriptor_t * desc = fi_bgq_spi_injfifo_tail_wait(injfifo);

	qpx_memcpy64((void*)desc, (const void*)&rts_model);

	struct rts_hdr * hdr = (struct rts_hdr *) &desc->PacketHeader;

	hdr->nbytes_msb = (uint16_t)(nbytes >> 16);
	hdr->nbytes_lsb = (uint16_t)(nbytes & 0x000000000000FFFFul);
	hdr->inj_cntr_paddr_rsh3b = (uint32_t)(cc_paddr >> 3);
	hdr->src_paddr = src_paddr;

	desc->Torus_FIFO_Map = addr->fifo_map;
	desc->PacketHeader.NetworkHeader.pt2pt.Destination = addr->destination;
	desc->PacketHeader.messageUnitHeader.Packet_Types.Memory_FIFO.Rec_FIFO_Id = addr->rx;

	/* DUMP_DESCRIPTOR(desc); */
	//fprintf(stderr, "%s:%s():%d hdr->nbytes_msb ............ = %hu\n", __FILE__, __func__, __LINE__, hdr->nbytes_msb);
	//fprintf(stderr, "%s:%s():%d hdr->nbytes_lsb ............ = %hu\n", __FILE__, __func__, __LINE__, hdr->nbytes_lsb);
	//fprintf(stderr, "%s:%s():%d hdr->inj_cntr_paddr_rsh3b .. = 0x%08x\n", __FILE__, __func__, __LINE__, hdr->inj_cntr_paddr_rsh3b);
	//fprintf(stderr, "%s:%s():%d hdr->src_paddr ............. = 0x%016lx\n", __FILE__, __func__, __LINE__, hdr->src_paddr);
	MUSPI_InjFifoAdvanceDesc(injfifo->muspi_injfifo);
}

static inline
uint64_t wait_rts (MUSPI_RecFifo_t * recfifo, uint64_t * bytes_to_transfer,
		uint64_t * src_paddr, uint64_t * inj_cntr_paddr) {

	MUSPI_Fifo_t * fifo = (MUSPI_Fifo_t *)recfifo;

	const uintptr_t pa_start = MUSPI_getStartPa(fifo);
	const uintptr_t va_head = (uintptr_t) MUSPI_getHeadVa(fifo);
	const uintptr_t va_start = (uintptr_t) MUSPI_getStartVa(fifo);
	const uintptr_t offset_head = va_head - va_start;

	uintptr_t offset_tail = MUSPI_getHwTail(fifo) - pa_start;

	/*
	 * wait until the head does not equal the tail; this signifies that
	 * a packet has been received
	 */
	while (offset_head == offset_tail) {
		offset_tail =  MUSPI_getHwTail(fifo) - pa_start;
	}


	muspi_dcbt(va_head, 0);
	fi_bgq_compiler_msync(FI_BGQ_COMPILER_MSYNC_KIND_RO);

	struct rts_hdr * hdr = (struct rts_hdr *) va_head;

	//fprintf(stderr, "%s:%s():%d hdr->nbytes_msb ............ = %hu\n", __FILE__, __func__, __LINE__, hdr->nbytes_msb);
	//fprintf(stderr, "%s:%s():%d hdr->nbytes_lsb ............ = %hu\n", __FILE__, __func__, __LINE__, hdr->nbytes_lsb);
	//fprintf(stderr, "%s:%s():%d hdr->inj_cntr_paddr_rsh3b .. = 0x%08x\n", __FILE__, __func__, __LINE__, hdr->inj_cntr_paddr_rsh3b);
	//fprintf(stderr, "%s:%s():%d hdr->src_paddr ............. = 0x%016lx\n", __FILE__, __func__, __LINE__, hdr->src_paddr);

	*src_paddr = hdr->src_paddr;
	*inj_cntr_paddr = ((uint64_t)hdr->inj_cntr_paddr_rsh3b) << 3;
	*bytes_to_transfer = (((uint64_t)hdr->nbytes_msb) << 32) | ((uint64_t)hdr->nbytes_lsb);

	uint64_t bytes_consumed;
	if (offset_head < offset_tail) {

		MUSPI_setHeadVa(fifo, (void*)(va_start + offset_tail));
		MUSPI_setHwHead(fifo, offset_tail);

		bytes_consumed = offset_tail - offset_head;

	} else {

		MUSPI_setHeadVa(fifo, (void*)(va_start));
		MUSPI_setHwHead(fifo, 0);

		const uintptr_t va_end = (uintptr_t) fifo->va_end;
		bytes_consumed = va_end - va_head;
	}

	return bytes_consumed >> 5;	/* each chunk is 32 bytes */
}


int main (int argc, char ** argv) {

	struct address * addr;
	struct fi_bgq_node node;
	struct l2atomic_lock lock;
	struct fi_bgq_agent agent;

	uint32_t rank = Kernel_GetRank();
	uint32_t ppn = Kernel_ProcessCount();
	uint64_t size = 0;	/* one endpoint per process */

	if (fi_bgq_node_init(&node))
		return -1;

	if (!node.agent_is_enabled) {
		if (rank == 0) {
			fprintf(stderr, "\n");
			fprintf(stderr, "Error. Application agent is required for this test.\n");
			fprintf(stderr, "\n");
			fprintf(stderr, "Next time set one of these environment variables:\n");
			fprintf(stderr, "    BG_APPAGENTCOMM\n");
			fprintf(stderr, "    BG_APPAGENT\n");
			fprintf(stderr, "    BG_APPAGENT_EMULATED\n");
			fprintf(stderr, "\n");
		}
		return -1;
	}

	if (fi_bgq_node_mu_lock_init(&node, &lock))
		return -1;

	fi_bgq_agent_register(&agent);

	l2atomic_lock_acquire(&lock);


	/*
	 * set up an "address vector"
	 */
	{
		Personality_t personality;
		int rc;
		rc = Kernel_GetPersonality(&personality, sizeof(Personality_t));
		if (rc) { assert(0); return 1; }

		uint64_t dcr_value = DCRReadUser(ND_500_DCR(CTRL_CUTOFFS));

		const size_t node_count = personality.Network_Config.Anodes *
			personality.Network_Config.Bnodes *
			personality.Network_Config.Cnodes *
			personality.Network_Config.Dnodes *
			personality.Network_Config.Enodes;

		size_t mapsize = node_count * ppn;
		BG_CoordinateMapping_t map[mapsize];

		rc = Kernel_RanksToCoords(sizeof(map), map, &size);
		if (rc) { assert(0); return 1; }

		addr = (struct address *) malloc(sizeof(struct address) * size);

		uint64_t i;
		for (i=0; i<size; ++i) {
			addr[i].destination.Destination.A_Destination = map[i].a;
			addr[i].destination.Destination.B_Destination = map[i].b;
			addr[i].destination.Destination.C_Destination = map[i].c;
			addr[i].destination.Destination.D_Destination = map[i].d;
			addr[i].destination.Destination.E_Destination = map[i].e;
			addr[i].destination.Destination.Reserved2 = 0;

			addr[i].fifo_map = fi_bgq_spi_calculate_fifo_map(map[rank], map[i], &personality, dcr_value);

			addr[i].rx = (((BGQ_MU_NUM_REC_FIFO_GROUPS-1) * BGQ_MU_NUM_REC_FIFOS_PER_GROUP) / ppn) * map[i].t;
		}

		if (size < 2) {
			fprintf (stderr, "Error. Test must be run with >1 process.\n");
			return -1;
		}
	}

	/*
	 * allocate the application reception fifo
	 */
	MUSPI_RecFifoSubGroup_t rfifo_subgroup;
	MUSPI_RecFifo_t * recfifo = spiu_allocate_reception_fifo(&rfifo_subgroup, addr[rank].rx);

	/*
	 * allocate the application injection fifo
	 */
	struct fi_bgq_spi_injfifo injfifo;
	MUSPI_InjFifoSubGroup_t injfifo_subgroup;
	fi_bgq_spi_injfifo_init(&injfifo, &injfifo_subgroup, 1, 1024, 512, 0, 1);

	/*
	 * initialize the memory-fifo "rts" descriptor model
	 */
	MUHWI_Descriptor_t * model = &rts_model;
	MUSPI_DescriptorZeroOut(model);

	model->Half_Word0.Prefetch_Only = MUHWI_DESCRIPTOR_PRE_FETCH_ONLY_NO;
	model->Half_Word1.Interrupt = MUHWI_DESCRIPTOR_DO_NOT_INTERRUPT_ON_PACKET_ARRIVAL;
	model->PacketHeader.NetworkHeader.pt2pt.Data_Packet_Type = MUHWI_PT2PT_DATA_PACKET_TYPE;
	model->PacketHeader.NetworkHeader.pt2pt.Byte3.Byte3 = MUHWI_PACKET_VIRTUAL_CHANNEL_DETERMINISTIC;
	model->PacketHeader.NetworkHeader.pt2pt.Byte8.Byte8 = MUHWI_PACKET_TYPE_FIFO;
	model->PacketHeader.NetworkHeader.pt2pt.Byte8.Size = 16;
	model->Pa_Payload = 0;
	model->Message_Length = 0;

	model->Torus_FIFO_Map = 0;								/* runtime */
	model->PacketHeader.messageUnitHeader.Packet_Types.Memory_FIFO.Rec_FIFO_Id = -1;	/* runtime */
	model->PacketHeader.NetworkHeader.pt2pt.Destination.Destination.Destination = -1;	/* runtime */


	l2atomic_lock_release(&lock);


	if (ppn > 1) l2atomic_barrier_enter(&node.barrier[FI_BGQ_NODE_BARRIER_KIND_USER]);
	if (node.is_leader) MUSPI_GIBarrierEnterAndWait(&node.gi_barrier);
	if (ppn > 1) l2atomic_barrier_enter(&node.barrier[FI_BGQ_NODE_BARRIER_KIND_USER]);

	if (rank == 0) {
		volatile uint64_t * cc = (volatile uint64_t *) malloc(sizeof(uint64_t) * size);

		cc[0] = 0;
		unsigned i;
		for (i=1; i<size; ++i) {
			cc[i] = 2048;
			inject_rts(&injfifo, &addr[i], (void *) sbuf, 2048, (uint64_t *)&cc[i]);
		}

		for (i=1; i<size; ++i) {
			while (cc[i] > 0);
		}

		fprintf (stdout, "Success.\n");

	} else {

		volatile uint64_t bytes_to_transfer __attribute__((__aligned__(64)));
		uint64_t rec_cntr_paddr = 0;
		fi_bgq_cnk_vaddr2paddr((const void *)&bytes_to_transfer, sizeof(uint64_t), &rec_cntr_paddr);

		uint64_t dst_paddr = 0;
		fi_bgq_cnk_vaddr2paddr((const void *)rbuf, sizeof(uint64_t), &dst_paddr);

		uint64_t src_paddr = 0;
		uint64_t inj_cntr_paddr = 0;
		wait_rts(recfifo, (uint64_t *)&bytes_to_transfer, &src_paddr, &inj_cntr_paddr);

		MUHWI_Descriptor_t * desc = fi_bgq_spi_injfifo_tail_wait(&injfifo);
		
		uint64_t iov_paddr = 0;
		union fi_bgq_agent_rget_iov * iov =
			(union fi_bgq_agent_rget_iov *)fi_bgq_spi_injfifo_immediate_payload(&injfifo, desc, &iov_paddr);
		fi_bgq_agent_rget_iov_init(iov, 2048, dst_paddr, src_paddr);

		fi_bgq_agent_rget_desc_init(desc, &agent,
				addr[0].destination, addr[0].fifo_map, 0,
				rec_cntr_paddr, inj_cntr_paddr,
				1, iov_paddr);

		/* DUMP_DESCRIPTOR(desc); */
		MUSPI_InjFifoAdvanceDesc(injfifo.muspi_injfifo);

		while (bytes_to_transfer > 0);

		fprintf (stdout, "Success. (%u)\n", rank);
	}

	if (ppn > 1) l2atomic_barrier_enter(&node.barrier[FI_BGQ_NODE_BARRIER_KIND_USER]);
	if (node.is_leader) MUSPI_GIBarrierEnterAndWait(&node.gi_barrier);
	if (ppn > 1) l2atomic_barrier_enter(&node.barrier[FI_BGQ_NODE_BARRIER_KIND_USER]);


	fi_bgq_spi_injfifo_fini(&injfifo);

	return 0;
}

