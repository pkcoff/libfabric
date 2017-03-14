#ifndef __SPI_UTIL_H__
#define __SPI_UTIL_H__


static inline void spiu_init_gi_barrier (MUSPI_GIBarrier_t * GIBarrier)
{
	int rc;
	rc = MUSPI_GIBarrierInit(GIBarrier, 0);
	if (rc) exit(1);
}


static inline void spiu_do_gi_barrier (MUSPI_GIBarrier_t * GIBarrier)
{
	int rc;
	uint64_t gi_timeout = 1600000000;	/* about 1 sec at 16 mhz */
	gi_timeout *= 30;
  
	rc = MUSPI_GIBarrierEnter(GIBarrier);
	if (rc) exit(1);

	rc = MUSPI_GIBarrierPollWithTimeout(GIBarrier, gi_timeout);
	if (rc) exit(1);
}


static inline void spiu_do_gi_barrier_no_timeout (MUSPI_GIBarrier_t * GIBarrier)
{
	int rc;
	rc = MUSPI_GIBarrierEnter(GIBarrier);
	if (rc) exit(1);

	while (MUSPI_GIBarrierPoll(GIBarrier) != 0);
}


static inline MUSPI_RecFifo_t * spiu_allocate_reception_fifo (MUSPI_RecFifoSubGroup_t * rfifo_subgroup, uint32_t rfifo_id)
{

	int rc __attribute__ ((unused));
	uint8_t * memptr;

	size_t nbytes = 8 * 1024 * 1024;
	rc = posix_memalign((void**)&memptr, 32, nbytes);
	assert(0 == rc);

	Kernel_MemoryRegion_t mregion;
	rc = Kernel_CreateMemoryRegion(&mregion, (void*)memptr, nbytes);
	assert(0 == rc);

	uint32_t subgrp_id = rfifo_id / BGQ_MU_NUM_REC_FIFOS_PER_SUBGROUP;
	uint32_t subgrp_rfifo_id = rfifo_id & 0x03u;

	uint32_t free_fifo_num;
	uint32_t free_fifo_ids[BGQ_MU_NUM_REC_FIFOS_PER_SUBGROUP];
	rc = Kernel_QueryRecFifos(subgrp_id, &free_fifo_num, free_fifo_ids);
	assert(0 == rc);
	assert(0 < free_fifo_num);
//	assert(0 == free_fifo_ids[0]);

	uint32_t subgrp_rfifo_idx = (uint32_t)-1;
	unsigned i;
	for (i=0; i<free_fifo_num; ++i) {
		if (free_fifo_ids[i] == subgrp_rfifo_id) {
			subgrp_rfifo_idx = i;
			break;
		}
	}
	assert(subgrp_rfifo_idx != (uint32_t)-1);



	Kernel_RecFifoAttributes_t attr;
	memset(&attr, 0x00, sizeof(attr));
	rc = Kernel_AllocateRecFifos(subgrp_id, rfifo_subgroup, 1, &free_fifo_ids[subgrp_rfifo_idx], &attr);
	assert(0 == rc);

	rc = Kernel_RecFifoInit(rfifo_subgroup, free_fifo_ids[subgrp_rfifo_idx], &mregion,
		((uint64_t)memptr) - (uint64_t)mregion.BaseVa,
		nbytes-1);
	assert(0 == rc);



	uint32_t grp_id = rfifo_id / BGQ_MU_NUM_REC_FIFOS_PER_GROUP;
	uint64_t grp_rfifo_id = rfifo_id & 0x0Fu;
	uint64_t enable_bits = 1 << (15 - grp_rfifo_id);

	//rc = Kernel_RecFifoEnable(0, 0x08000ull);
	rc = Kernel_RecFifoEnable(grp_id, enable_bits);
	assert(0 == rc);

	assert(rfifo_subgroup->_recfifos[0]._fifo.hwfifo);

	return &rfifo_subgroup->_recfifos[0];
}


static inline void spiu_inject (struct fi_bgq_spi_injfifo * ififo, MUHWI_Descriptor_t * model)
{

	MUSPI_InjFifo_t * muspi_injfifo = ififo->muspi_injfifo;
	MUHWI_Descriptor_t * d = fi_bgq_spi_injfifo_tail_wait(ififo);
	*d = *model;

	MUSPI_InjFifoAdvanceDesc(muspi_injfifo);
	return;
}


/* return the number of *chunks* consumed; does not process the packet */
static inline uint64_t spiu_receive (MUSPI_RecFifo_t * recfifo)
{
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




#endif /* __SPI_UTIL_H__ */
