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
#ifndef _FI_PROV_BGQ_SPI_H_
#define _FI_PROV_BGQ_SPI_H_

/*
 * Certain BGQ SPI files expect the '__LINUX__' macro to be defined to '0'
 * or '1' instead of simply checking if the macro is defined or not.
 * Specifically, the following file needs hacking, although there are probably
 * others.
 *
 *   spi/include/mu/Addressing.h
 */
#ifndef __LINUX__
#define __LINUX__ 0
#endif

/*
 * The bgq system software, specifically 'hwi/include/common/compiler_support.h',
 * will define the __INLINE__ macro if it is not already defined to the following:
 *
 *   #define __INLINE__ extern inline __attribute__((always_inline))
 *
 * This is the non-portable "gnu 89" style which easily results in undefined
 * symbols or multiple defined symbols when used by software coded to a more
 * recent C standard.
 *
 * As a workaround the __INLINE__ macro will be defined to the more appropriate
 * 'static inline' style only for the bgq system software includes and then
 * undefined at the end of this file. This seems to fix the problem without
 * requiring any changes to the installed bgq system software files.
 */
#ifdef __INLINE__
#error __INLINE__ already defined!
#else
#define __INLINE__ static inline
#endif

#include <spi/include/kernel/location.h>
#include <spi/include/kernel/memory.h>
#include <spi/include/kernel/MU.h>
#include <spi/include/l1p/flush.h>

/*
 * Avoid the pervasive "function declared static but never defined" warning for
 * unused kernel spi functions, defined in 'spi/include/kernel/MU.h' by
 * providing an implementation.
 */
int32_t Kernel_GetNDExpectedTokens(uint32_t a, uint32_t b, uint32_t *c) { assert(0); return -1; }
int32_t Kernel_GetNDExpectedTokensDCR(uint32_t a, uint64_t *b) { assert(0); return -1; }

/* Work around buggy SPI code when -DNDEBUG is specified */
#ifdef NDEBUG
#undef NDEBUG
#include <spi/include/l2/atomic.h>
#define NDEBUG
#else
#include <spi/include/l2/atomic.h>
#endif

#include <spi/include/l2/lock.h>
#include <spi/include/mu/Addressing.h>
#include <spi/include/mu/Addressing_inlines.h>
#include <spi/include/mu/Descriptor_inlines.h>
#include <spi/include/mu/GIBarrier.h>
#include <spi/include/mu/InjFifo.h>
#include <spi/include/mu/RecFifo.h>

#undef __INLINE__

#ifndef MIN
#define MIN(a,b) (b^((a^b)&-(a<b)))
#endif
#ifndef MIN3
#define MIN3(a,b,c) (MIN(MIN(a,b),c))
#endif
#ifndef MIN4
#define MIN4(a,b,c,d) (MIN(MIN(a,b),MIN(c,d)))
#endif


struct fi_bgq_spi_injfifo {
	MUSPI_InjFifo_t	*muspi_injfifo;
	uint64_t	*sw_freeSpace;
	MUHWI_InjFifo_t	*hw_injfifo;	/* See: MUSPI_getHwFreeSpace */
	uint64_t	*sw_tailva;	/* See: MUSPI_getTailVa */

	size_t		immediate_payload_sizeof;
	uintptr_t	immediate_payload_base_vaddr;
	uint64_t	immediate_payload_base_paddr;

	uint64_t	node_scoped_fifo_id;
	void		*memory;
	void		*immediate_payload_memory;

	uintptr_t	va_start;
} __attribute((aligned(L2_CACHE_LINE_SIZE)));

int fi_bgq_spi_injfifo_init (struct fi_bgq_spi_injfifo *f,
		MUSPI_InjFifoSubGroup_t *injfifo_subgroup,
		unsigned num_fifos_to_allocate,
		const size_t injfifo_size,
		const unsigned immediate_payload_sizeof,
		const unsigned is_remote_get,
		const unsigned is_top_down);

int fi_bgq_spi_injfifo_subgrp_init (struct fi_bgq_spi_injfifo *f,
		MUSPI_InjFifoSubGroup_t *subgrp,
		unsigned num_fifos_to_allocate,
		const size_t injfifo_size,
		const unsigned immediate_payload_sizeof,
		const unsigned is_remote_get,
		const int subgrp_id);

void fi_bgq_spi_injfifo_clone (struct fi_bgq_spi_injfifo *dst, struct fi_bgq_spi_injfifo *src);


int fi_bgq_spi_injfifo_fini (struct fi_bgq_spi_injfifo *f);


static inline
MUHWI_Descriptor_t * fi_bgq_spi_injfifo_tail_wait (struct fi_bgq_spi_injfifo *f) {

	if (0 == *(f->sw_freeSpace)) {	/* unlikely */
		do {
			/* mmio read from hardware to update shadow state */
			*(f->sw_freeSpace) = f->hw_injfifo->freeSpace;
		} while (0 == *(f->sw_freeSpace));
	}

	return (MUHWI_Descriptor_t *) *f->sw_tailva;	/* updated via MUSPI_InjFifoAdvanceDesc */
}


static inline
void * fi_bgq_spi_injfifo_immediate_payload (struct fi_bgq_spi_injfifo *f,
		MUHWI_Descriptor_t *desc, uint64_t *paddr) {

	assert(f);
	assert(f->immediate_payload_base_vaddr != 0);
	assert(f->immediate_payload_sizeof != 0);
	assert(f->va_start != 0);

	const uint64_t offset =
		(((uintptr_t)desc - f->va_start) >> BGQ_MU_DESCRIPTOR_SIZE_IN_POWER_OF_2) *
		f->immediate_payload_sizeof;

	*paddr = f->immediate_payload_base_paddr + offset;

	return (void*)(f->immediate_payload_base_vaddr + offset);
}


static inline
MUHWI_Destination_t fi_bgq_spi_coordinates_to_destination (BG_CoordinateMapping_t coords) {

	union foo {
		BG_CoordinateMapping_t	coords;
		uint32_t		raw;
	};

	const union foo tmp = {.coords=coords};

	const uint32_t tmp2 = (tmp.raw & 0x3FFFFFC0ul) | (tmp.raw >> 31);
	const MUHWI_Destination_t * const out = (const MUHWI_Destination_t * const)&tmp2;

	return *out;
}

static inline uint64_t fi_bgq_cnk_vaddr2paddr(const void * vaddr, size_t len, uint64_t * paddr)
{
	Kernel_MemoryRegion_t cnk_mr;
	uint32_t cnk_rc;
	cnk_rc = Kernel_CreateMemoryRegion(&cnk_mr, (void *)vaddr, len);
	if (cnk_rc) return cnk_rc;

	*paddr = (uint64_t)cnk_mr.BasePa + ((uint64_t)vaddr - (uint64_t)cnk_mr.BaseVa);
	return 0;
}

/* expensive .. not for critical path! */
static inline
uint32_t fi_bgq_spi_calculate_fifo_map(BG_CoordinateMapping_t local,
		BG_CoordinateMapping_t remote, Personality_t * personality,
		uint64_t dcr_value) {

	/* calculate the signed coordinate difference between the source and
	 * destination torus coordinates
	 */
	ssize_t dA = (ssize_t)remote.a - (ssize_t)local.a;
	ssize_t dB = (ssize_t)remote.b - (ssize_t)local.b;
	ssize_t dC = (ssize_t)remote.c - (ssize_t)local.c;
	ssize_t dD = (ssize_t)remote.d - (ssize_t)local.d;
	ssize_t dE = (ssize_t)remote.e - (ssize_t)local.e;

	/* select the fifo based on the t coordinate only if local */
	if ((dA | dB | dC | dD | dE) == 0) {
		return (remote.t & 0x01) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_LOCAL0 : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_LOCAL1;
	}

	/* select either A- or A+ if communicating only along the A dimension */
	if ((dB | dC | dD | dE) == 0) {
		if (ND_ENABLE_TORUS_DIM_A & personality->Network_Config.NetFlags) {
			uint64_t cutoff;
			if (dA > 0) {
				cutoff = ND_500_DCR__CTRL_CUTOFFS__A_PLUS_get(dcr_value);
				return (remote.a > cutoff) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_AM : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_AP;
			} else {
				cutoff = ND_500_DCR__CTRL_CUTOFFS__A_MINUS_get(dcr_value);
				return (remote.a < cutoff) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_AP : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_AM;
			}
		} else {
			return (dA > 0) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_AP : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_AM;
		}
	}

	/* select either B- or B+ if communicating only along the B dimension */
	if ((dA | dC | dD | dE) == 0) {
		if (ND_ENABLE_TORUS_DIM_B & personality->Network_Config.NetFlags) {
			uint64_t cutoff;
			if (dB > 0) {
				cutoff = ND_500_DCR__CTRL_CUTOFFS__B_PLUS_get(dcr_value);
				return (remote.b > cutoff) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_BM : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_BP;
			} else {
				cutoff = ND_500_DCR__CTRL_CUTOFFS__B_MINUS_get(dcr_value);
				return (remote.b < cutoff) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_BP : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_BM;
			}
		} else {
			return (dB > 0) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_BP : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_BM;
		}
	}

	/* select either C- or C+ if communicating only along the C dimension */
	if ((dA | dB | dD | dE) == 0) {
		if (ND_ENABLE_TORUS_DIM_C & personality->Network_Config.NetFlags) {
			uint64_t cutoff;
			if (dC > 0) {
				cutoff = ND_500_DCR__CTRL_CUTOFFS__C_PLUS_get(dcr_value);
				return (remote.c > cutoff) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_CM : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_CP;
			} else {
				cutoff = ND_500_DCR__CTRL_CUTOFFS__C_MINUS_get(dcr_value);
				return (remote.c < cutoff) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_CP : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_CM;
			}
		} else {
			return (dC > 0) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_CP : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_CM;
		}
	}

	/* select either D- or D+ if communicating only along the D dimension */
	if ((dA | dB | dC | dE) == 0) {
		if (ND_ENABLE_TORUS_DIM_D & personality->Network_Config.NetFlags) {
			uint64_t cutoff;
			if (dD > 0) {
				cutoff = ND_500_DCR__CTRL_CUTOFFS__D_PLUS_get(dcr_value);
				return (remote.d > cutoff) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_DM : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_DP;
			} else {
				cutoff = ND_500_DCR__CTRL_CUTOFFS__D_MINUS_get(dcr_value);
				return (remote.d < cutoff) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_DP : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_DM;
			}
		} else {
			return (dD > 0) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_DP : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_DM;
		}
	}

	/* select either E- or E+ if communicating only along the E dimension */
	if ((dA | dB | dC | dD) == 0) {
		/* the maximum 'e' dimension size is 2 - and is a torus */
		return (remote.t & 0x01) ? MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_EP : MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_EM;
	}

	/* communicating along diagonal */
	/* TODO - OPTIMIZE - round-robin the fifo picking based on destination */
	if (dA > 0) {
		return MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_AP;
	} else if (dA < 0)
		return MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_AM;

	if (dB > 0) {
		return MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_BP;
	} else if (dB < 0)
		return MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_BM;

	if (dC > 0) {
		return MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_CP;
	} else if (dC < 0)
		return MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_CM;

	if (dD > 0) {
		return MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_DP;
	} else if(dD < 0)
		return MUHWI_DESCRIPTOR_TORUS_FIFO_MAP_DM;

	assert(0);
	return 0xFFFFu;
}

static inline
uint32_t fi_bgq_spi_calculate_fifo_map_single (BG_CoordinateMapping_t local, BG_CoordinateMapping_t remote) {

	Personality_t personality;
	int rc = Kernel_GetPersonality(&personality, sizeof(Personality_t));
	if (rc) return 0;	/* error!? */

	uint64_t dcr_value = DCRReadUser(ND_500_DCR(CTRL_CUTOFFS));

	return fi_bgq_spi_calculate_fifo_map(local, remote, &personality, dcr_value);
}


#endif /* _FI_PROV_BGQ_SPI_H_ */
