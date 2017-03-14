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
#ifndef _FI_PROV_BGQ_AGENT_H_
#define _FI_PROV_BGQ_AGENT_H_

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>

#include "rdma/bgq/fi_bgq_node.h"

/*
 * \brief agent packet header
 *
 * The agent packet header is only used to communicate with the libfabric
 * appliation agent running on the 17th core.
 */
union fi_bgq_agent_packet_hdr {

	/* The torus packet header is 32 bytes. see: hwi/include/bqc/MU_PacketHeader.h */
	MUHWI_PacketHeader_t		muhwi;

	uint64_t			raw64b[4];
	uint32_t			raw32b[8];
	uint16_t			raw16b[16];
	uint8_t				raw8b[32];

	struct {
		/* The point-to-point header occupies bytes 0-11 of the packet header
		 * see: MUHWI_Pt2PtNetworkHeader_t in hwi/include/bqc/MU_Pt2PtNetworkHeader.h */
		uint64_t		reserved_0;
		uint32_t		reserved_1;

		/* The message unit header occupies bytes 12-31 of the packet header
		 * see: MUHWI_MessageUnitHeader_t in hwi/include/bqc/MU_MessageUnitHeader.h */
		uint16_t		reserved_2	: 10;
		uint16_t		unused_0	:  6;
		uint8_t			unused_1[18];
	} __attribute__((__packed__)) raw;

	struct {
		uint64_t		reserved_0;

		uint32_t		reserved_1;
		uint16_t		reserved_2	: 10;
		uint16_t		kind		:  6;
		uint16_t		reserved_3;

		uint64_t		reserved_4;
		uint64_t		reserved_5;
	} __attribute__((__packed__)) common;


	struct {
		uint64_t		reserved_0;

		uint32_t		reserved_1;
		uint16_t		reserved_2;
		uint16_t		fifo_map;		/* only 10 msb are used; 6 lsb are always zero */

		MUHWI_Destination_t	origin;
		uint8_t			inj_fifo;		/* "rget" injection fifo to use */
		uint8_t			niov;
		uint16_t		rec_cntr_batid;

		uint32_t		rec_cntr_offset_rsh3b;	/* must be aligned to 8 bytes; see NOTE_MU_PADDR */
		uint32_t		inj_cntr_paddr_rsh3b;	/* must be aligned to 8 bytes; see NOTE_MU_PADDR */

	} __attribute__((__packed__)) rget;

} __attribute__((__aligned__(32)));


union fi_bgq_agent_rget_iov {

	uint64_t			raw64[2];
	uint32_t			raw32[4];
	uint16_t			raw16[8];

	struct {
		uint64_t		nbytes_msb	: 16;	/* 4 GB maximum data transfer per iov */
		uint64_t		rec_data_batid	: 10;
		uint64_t		unused_0	:  4;
		uint64_t		rec_data_offset	: 34;

		uint64_t		nbytes_lsb	: 16;	/* 4 GB maximum data transfer per iov */
		uint64_t		unused_1	: 14;
		uint64_t		src_paddr	: 34;

	} __attribute__((__packed__));
};


union fi_bgq_agent_packet_payload {

	uint8_t				byte[512];

	union fi_bgq_agent_rget_iov	iov[32];

} __attribute__((__aligned__(32)));


struct fi_bgq_agent_packet {

	union fi_bgq_agent_packet_hdr		header;
	union fi_bgq_agent_packet_payload	payload;

} __attribute__((__packed__));


enum fi_bgq_agent_packet_kind {
	FI_BGQ_AGENT_PACKET_KIND_RGET = 0,
	FI_BGQ_AGENT_PACKET_KIND_COUNT
};


static inline void
fi_bgq_agent_packet_set_kind (struct fi_bgq_agent_packet * const packet, const enum fi_bgq_agent_packet_kind kind) {
	assert(((uint16_t)kind & 0xFFC0u) == 0);
	assert(kind < FI_BGQ_AGENT_PACKET_KIND_COUNT);

	/* clear the old value first, then or the new value */
	const uint16_t tmp = (packet->header.raw16b[6] & 0xFFC0u) | (uint16_t)kind;
	packet->header.raw16b[6] = tmp;
}


static inline enum fi_bgq_agent_packet_kind
fi_bgq_agent_packet_get_kind (struct fi_bgq_agent_packet const * const packet) {
	return (enum fi_bgq_agent_packet_kind) (packet->header.raw16b[6] & 0x03Fu);
}








struct fi_bgq_agent {
	 MUHWI_Descriptor_t	rget_model;
};

void fi_bgq_agent_register (struct fi_bgq_agent * agent);


void fi_bgq_agent_rget_desc_init (MUHWI_Descriptor_t * desc,
		const struct fi_bgq_agent * const agent,
		const MUHWI_Destination_t destination, const uint16_t fifo_map,
		const uint8_t inj_fifo, const uint64_t rec_cntr_paddr,
		const uint64_t inj_cntr_paddr,
		const uint8_t niov, const uint64_t iov_paddr);

static inline void
fi_bgq_agent_rget_iov_init (union fi_bgq_agent_rget_iov * const iov,
		const uint64_t nbytes, const uint64_t dst_paddr, const uint64_t src_paddr) {
	iov->raw64[0] = dst_paddr | ((nbytes << 32) & 0xFFFF000000000000ul) | ((uint64_t)FI_BGQ_NODE_BAT_ID_GLOBAL << 38);
	iov->raw64[1] = src_paddr | (nbytes << 48);
}



#endif /* _FI_PROV_BGQ_AGENT_H_ */
