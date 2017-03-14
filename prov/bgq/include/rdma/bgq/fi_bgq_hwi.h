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
#ifndef _FI_PROV_BGQ_HWI_H_
#define _FI_PROV_BGQ_HWI_H_

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

#include <firmware/include/personality.h>
#include <hwi/include/bqc/A2_inlines.h>
#include <hwi/include/bqc/A2_inlines.h>
#include <hwi/include/bqc/MU_Descriptor.h>
#include <hwi/include/bqc/MU_Macros.h>
#include <hwi/include/bqc/MU_PacketHeader.h>
#include <hwi/include/bqc/nd_500_dcr.h>
#include <hwi/include/common/bgq_alignment.h>

#include <stdio.h>

static inline void
dump_descriptor (char * prefix, MUHWI_Descriptor_t * desc) {

	uint32_t * ptr = (uint32_t *)desc;
	fprintf(stderr, "%s [%p]: %08x %08x %08x %08x\n", prefix, ptr, *(ptr), *(ptr+1), *(ptr+2), *(ptr+3)); ptr+=4;
	fprintf(stderr, "%s [%p]: %08x %08x %08x %08x\n", prefix, ptr, *(ptr), *(ptr+1), *(ptr+2), *(ptr+3)); ptr+=4;
	fprintf(stderr, "%s [%p]: %08x %08x %08x %08x\n", prefix, ptr, *(ptr), *(ptr+1), *(ptr+2), *(ptr+3)); ptr+=4;
	fprintf(stderr, "%s [%p]: %08x %08x %08x %08x\n", prefix, ptr, *(ptr), *(ptr+1), *(ptr+2), *(ptr+3)); ptr+=4;

	fprintf(stderr, "%s descriptor dump at %p\n", prefix, (void*)desc);
	fprintf(stderr, "%s   .Half_Word0.Prefetch_Only .................. %d\n", prefix, desc->Half_Word0.Prefetch_Only);
	fprintf(stderr, "%s   .Half_Word1.Interrupt ...................... %d\n", prefix, desc->Half_Word1.Interrupt);
	fprintf(stderr, "%s   .Pa_Payload ................................ 0x%016lx\n", prefix, desc->Pa_Payload);
	fprintf(stderr, "%s   .Message_Length ............................ %lu\n", prefix, desc->Message_Length);
	fprintf(stderr, "%s   .Torus_FIFO_Map ............................ 0x%016lx\n", prefix, desc->Torus_FIFO_Map);
	fprintf(stderr, "%s   .PacketHeader.NetworkHeader.pt2pt\n", prefix);
	fprintf(stderr, "%s     .Data_Packet_Type ........................ 0x%02x\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Data_Packet_Type);
	fprintf(stderr, "%s     .Hints ................................... 0x%02x\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Hints);
	fprintf(stderr, "%s     .Byte2.Hint_E_plus ....................... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte2.Hint_E_plus);
	fprintf(stderr, "%s     .Byte2.Hint_E_minus ...................... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte2.Hint_E_minus);
	fprintf(stderr, "%s     .Byte2.Route_To_IO_Node .................. %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte2.Route_To_IO_Node);
	fprintf(stderr, "%s     .Byte2.Return_From_IO_Node ............... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte2.Return_From_IO_Node);
	fprintf(stderr, "%s     .Byte2.Dynamic ........................... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte2.Dynamic);
	fprintf(stderr, "%s     .Byte2.Deposit ........................... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte2.Deposit);
	fprintf(stderr, "%s     .Byte2.Interrupt ......................... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte2.Interrupt);
	fprintf(stderr, "%s     .Byte3.Virtual_channel ................... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte3.Virtual_channel);
	fprintf(stderr, "%s     .Byte3.Zone_Routing_Id ................... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte3.Zone_Routing_Id);
	fprintf(stderr, "%s     .Byte3.Stay_On_Bubble .................... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte3.Stay_On_Bubble);
	fprintf(stderr, "%s     .Destination.Destination.Reserved2 ....... %u\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Destination.Destination.Reserved2);
	fprintf(stderr, "%s     .Destination.Destination.A_Destination ... %u\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Destination.Destination.A_Destination);
	fprintf(stderr, "%s     .Destination.Destination.B_Destination ... %u\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Destination.Destination.B_Destination);
	fprintf(stderr, "%s     .Destination.Destination.C_Destination ... %u\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Destination.Destination.C_Destination);
	fprintf(stderr, "%s     .Destination.Destination.D_Destination ... %u\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Destination.Destination.D_Destination);
	fprintf(stderr, "%s     .Destination.Destination.E_Destination ... %u\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Destination.Destination.E_Destination);
	fprintf(stderr, "%s     .Byte8.Packet_Type ....................... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte8.Packet_Type);
	fprintf(stderr, "%s     .Byte8.Reserved3 ......................... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte8.Reserved3);
	fprintf(stderr, "%s     .Byte8.Size .............................. %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Byte8.Size);
	fprintf(stderr, "%s     .Injection_Info.Reserved4 ................ %hu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Injection_Info.Reserved4);
	fprintf(stderr, "%s     .Injection_Info.Skip ..................... %hhu\n", prefix, desc->PacketHeader.NetworkHeader.pt2pt.Injection_Info.Skip);
	if (desc->PacketHeader.NetworkHeader.pt2pt.Byte8.Packet_Type == 0) {
		fprintf(stderr, "%s   .PacketHeader.messageUnitHeader.Packet_Types\n", prefix);
		fprintf(stderr, "%s     .Memory_FIFO.Reserved1 ................... %hu\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Memory_FIFO.Reserved1);
		fprintf(stderr, "%s     .Memory_FIFO.Rec_FIFO_Id ................. %hu\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Memory_FIFO.Rec_FIFO_Id);
		fprintf(stderr, "%s     .Memory_FIFO.Unused1 ..................... %hu\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Memory_FIFO.Unused1);
		fprintf(stderr, "%s     .Memory_FIFO.Put_Offset_MSB .............. 0x%08hx\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Memory_FIFO.Put_Offset_MSB);
		fprintf(stderr, "%s     .Memory_FIFO.Put_Offset_LSB .............. 0x%08x\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Memory_FIFO.Put_Offset_LSB);
	} else if (desc->PacketHeader.NetworkHeader.pt2pt.Byte8.Packet_Type == 1) {
		fprintf(stderr, "%s   .PacketHeader.messageUnitHeader.Packet_Types\n", prefix);
		fprintf(stderr, "%s     .Direct_Put.Rec_Payload_Base_Address_Id .. %hu\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Rec_Payload_Base_Address_Id);
		fprintf(stderr, "%s     .Direct_Put.Pacing ....................... %hu\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Pacing);
		fprintf(stderr, "%s     .Direct_Put.Put_Offset_MSB ............... 0x%08hx\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Put_Offset_MSB);
		fprintf(stderr, "%s     .Direct_Put.Put_Offset_LSB ............... 0x%08x\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Put_Offset_LSB);
		fprintf(stderr, "%s     .Direct_Put.Unused1 ...................... %hu\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Unused1);
		fprintf(stderr, "%s     .Direct_Put.Rec_Counter_Base_Address_Id .. %hu\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Rec_Counter_Base_Address_Id);
		fprintf(stderr, "%s     .Direct_Put.Counter_Offset ............... 0x%016lx\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Direct_Put.Counter_Offset);
	} else if (desc->PacketHeader.NetworkHeader.pt2pt.Byte8.Packet_Type == 2) {
		fprintf(stderr, "%s   .PacketHeader.messageUnitHeader.Packet_Types\n", prefix);
		fprintf(stderr, "%s     .Remote_Get.Rget_Inj_FIFO_Id ............. %hu\n", prefix, desc->PacketHeader.messageUnitHeader.Packet_Types.Remote_Get.Rget_Inj_FIFO_Id);
	}
	fflush(stderr);
}

#define DUMP_DESCRIPTOR(desc)							\
({										\
	char prefix[1024];							\
	snprintf(prefix, 1023, "%s:%s():%d", __FILE__, __func__, __LINE__);	\
	dump_descriptor(prefix, (desc));					\
})

#undef __INLINE__

#endif /* _FI_PROV_BGQ_HWI_H_ */
