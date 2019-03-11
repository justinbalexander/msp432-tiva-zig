//*****************************************************************************
//
// hw_udma.h - Macros for use in accessing the UDMA registers.
//
// Copyright (c) 2007-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************


//*****************************************************************************
//
// The following are defines for the Micro Direct Memory Access register
// addresses.
//
//*****************************************************************************
pub const UDMA_STAT = usize(0x400FF000);  // DMA Status
pub const UDMA_CFG = usize(0x400FF004);  // DMA Configuration
pub const UDMA_CTLBASE = usize(0x400FF008);  // DMA Channel Control Base Pointer
pub const UDMA_ALTBASE = usize(0x400FF00C);  // DMA Alternate Channel Control
                                            // Base Pointer
pub const UDMA_WAITSTAT = usize(0x400FF010);  // DMA Channel Wait-on-Request
                                            // Status
pub const UDMA_SWREQ = usize(0x400FF014);  // DMA Channel Software Request
pub const UDMA_USEBURSTSET = usize(0x400FF018);  // DMA Channel Useburst Set
pub const UDMA_USEBURSTCLR = usize(0x400FF01C);  // DMA Channel Useburst Clear
pub const UDMA_REQMASKSET = usize(0x400FF020);  // DMA Channel Request Mask Set
pub const UDMA_REQMASKCLR = usize(0x400FF024);  // DMA Channel Request Mask Clear
pub const UDMA_ENASET = usize(0x400FF028);  // DMA Channel Enable Set
pub const UDMA_ENACLR = usize(0x400FF02C);  // DMA Channel Enable Clear
pub const UDMA_ALTSET = usize(0x400FF030);  // DMA Channel Primary Alternate
                                            // Set
pub const UDMA_ALTCLR = usize(0x400FF034);  // DMA Channel Primary Alternate
                                            // Clear
pub const UDMA_PRIOSET = usize(0x400FF038);  // DMA Channel Priority Set
pub const UDMA_PRIOCLR = usize(0x400FF03C);  // DMA Channel Priority Clear
pub const UDMA_ERRCLR = usize(0x400FF04C);  // DMA Bus Error Clear
pub const UDMA_CHASGN = usize(0x400FF500);  // DMA Channel Assignment
pub const UDMA_CHIS = usize(0x400FF504);  // DMA Channel Interrupt Status
pub const UDMA_CHMAP0 = usize(0x400FF510);  // DMA Channel Map Select 0
pub const UDMA_CHMAP1 = usize(0x400FF514);  // DMA Channel Map Select 1
pub const UDMA_CHMAP2 = usize(0x400FF518);  // DMA Channel Map Select 2
pub const UDMA_CHMAP3 = usize(0x400FF51C);  // DMA Channel Map Select 3

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_STAT register.
//
//*****************************************************************************
pub const UDMA_STAT_DMACHANS_M = usize(0x001F0000);  // Available uDMA Channels Minus 1
pub const UDMA_STAT_STATE_M = usize(0x000000F0);  // Control State Machine Status
pub const UDMA_STAT_STATE_IDLE = usize(0x00000000);  // Idle
pub const UDMA_STAT_STATE_RD_CTRL = usize(0x00000010);  // Reading channel controller data
pub const UDMA_STAT_STATE_RD_SRCENDP = usize(0x00000020);  // Reading source end pointer
pub const UDMA_STAT_STATE_RD_DSTENDP = usize(0x00000030);  // Reading destination end pointer
pub const UDMA_STAT_STATE_RD_SRCDAT = usize(0x00000040);  // Reading source data
pub const UDMA_STAT_STATE_WR_DSTDAT = usize(0x00000050);  // Writing destination data
pub const UDMA_STAT_STATE_WAIT = usize(0x00000060);  // Waiting for uDMA request to
                                            // clear
pub const UDMA_STAT_STATE_WR_CTRL = usize(0x00000070);  // Writing channel controller data
pub const UDMA_STAT_STATE_STALL = usize(0x00000080);  // Stalled
pub const UDMA_STAT_STATE_DONE = usize(0x00000090);  // Done
pub const UDMA_STAT_STATE_UNDEF = usize(0x000000A0);  // Undefined
pub const UDMA_STAT_MASTEN = usize(0x00000001);  // Master Enable Status
pub const UDMA_STAT_DMACHANS_S = usize(16);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CFG register.
//
//*****************************************************************************
pub const UDMA_CFG_MASTEN = usize(0x00000001);  // Controller Master Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CTLBASE register.
//
//*****************************************************************************
pub const UDMA_CTLBASE_ADDR_M = usize(0xFFFFFC00);  // Channel Control Base Address
pub const UDMA_CTLBASE_ADDR_S = usize(10);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTBASE register.
//
//*****************************************************************************
pub const UDMA_ALTBASE_ADDR_M = usize(0xFFFFFFFF);  // Alternate Channel Address
                                            // Pointer
pub const UDMA_ALTBASE_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_WAITSTAT register.
//
//*****************************************************************************
pub const UDMA_WAITSTAT_WAITREQ_M = usize(0xFFFFFFFF);  // Channel [n] Wait Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_SWREQ register.
//
//*****************************************************************************
pub const UDMA_SWREQ_M = usize(0xFFFFFFFF);  // Channel [n] Software Request

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_USEBURSTSET
// register.
//
//*****************************************************************************
pub const UDMA_USEBURSTSET_SET_M = usize(0xFFFFFFFF);  // Channel [n] Useburst Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_USEBURSTCLR
// register.
//
//*****************************************************************************
pub const UDMA_USEBURSTCLR_CLR_M = usize(0xFFFFFFFF);  // Channel [n] Useburst Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_REQMASKSET
// register.
//
//*****************************************************************************
pub const UDMA_REQMASKSET_SET_M = usize(0xFFFFFFFF);  // Channel [n] Request Mask Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_REQMASKCLR
// register.
//
//*****************************************************************************
pub const UDMA_REQMASKCLR_CLR_M = usize(0xFFFFFFFF);  // Channel [n] Request Mask Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ENASET register.
//
//*****************************************************************************
pub const UDMA_ENASET_SET_M = usize(0xFFFFFFFF);  // Channel [n] Enable Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ENACLR register.
//
//*****************************************************************************
pub const UDMA_ENACLR_CLR_M = usize(0xFFFFFFFF);  // Clear Channel [n] Enable Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTSET register.
//
//*****************************************************************************
pub const UDMA_ALTSET_SET_M = usize(0xFFFFFFFF);  // Channel [n] Alternate Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTCLR register.
//
//*****************************************************************************
pub const UDMA_ALTCLR_CLR_M = usize(0xFFFFFFFF);  // Channel [n] Alternate Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_PRIOSET register.
//
//*****************************************************************************
pub const UDMA_PRIOSET_SET_M = usize(0xFFFFFFFF);  // Channel [n] Priority Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_PRIOCLR register.
//
//*****************************************************************************
pub const UDMA_PRIOCLR_CLR_M = usize(0xFFFFFFFF);  // Channel [n] Priority Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ERRCLR register.
//
//*****************************************************************************
pub const UDMA_ERRCLR_ERRCLR = usize(0x00000001);  // uDMA Bus Error Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHASGN register.
//
//*****************************************************************************
pub const UDMA_CHASGN_M = usize(0xFFFFFFFF);  // Channel [n] Assignment Select
pub const UDMA_CHASGN_PRIMARY = usize(0x00000000);  // Use the primary channel
                                            // assignment
pub const UDMA_CHASGN_SECONDARY = usize(0x00000001);  // Use the secondary channel
                                            // assignment

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHIS register.
//
//*****************************************************************************
pub const UDMA_CHIS_M = usize(0xFFFFFFFF);  // Channel [n] Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP0 register.
//
//*****************************************************************************
pub const UDMA_CHMAP0_CH7SEL_M = usize(0xF0000000);  // uDMA Channel 7 Source Select
pub const UDMA_CHMAP0_CH6SEL_M = usize(0x0F000000);  // uDMA Channel 6 Source Select
pub const UDMA_CHMAP0_CH5SEL_M = usize(0x00F00000);  // uDMA Channel 5 Source Select
pub const UDMA_CHMAP0_CH4SEL_M = usize(0x000F0000);  // uDMA Channel 4 Source Select
pub const UDMA_CHMAP0_CH3SEL_M = usize(0x0000F000);  // uDMA Channel 3 Source Select
pub const UDMA_CHMAP0_CH2SEL_M = usize(0x00000F00);  // uDMA Channel 2 Source Select
pub const UDMA_CHMAP0_CH1SEL_M = usize(0x000000F0);  // uDMA Channel 1 Source Select
pub const UDMA_CHMAP0_CH0SEL_M = usize(0x0000000F);  // uDMA Channel 0 Source Select
pub const UDMA_CHMAP0_CH7SEL_S = usize(28);
pub const UDMA_CHMAP0_CH6SEL_S = usize(24);
pub const UDMA_CHMAP0_CH5SEL_S = usize(20);
pub const UDMA_CHMAP0_CH4SEL_S = usize(16);
pub const UDMA_CHMAP0_CH3SEL_S = usize(12);
pub const UDMA_CHMAP0_CH2SEL_S = usize(8);
pub const UDMA_CHMAP0_CH1SEL_S = usize(4);
pub const UDMA_CHMAP0_CH0SEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP1 register.
//
//*****************************************************************************
pub const UDMA_CHMAP1_CH15SEL_M = usize(0xF0000000);  // uDMA Channel 15 Source Select
pub const UDMA_CHMAP1_CH14SEL_M = usize(0x0F000000);  // uDMA Channel 14 Source Select
pub const UDMA_CHMAP1_CH13SEL_M = usize(0x00F00000);  // uDMA Channel 13 Source Select
pub const UDMA_CHMAP1_CH12SEL_M = usize(0x000F0000);  // uDMA Channel 12 Source Select
pub const UDMA_CHMAP1_CH11SEL_M = usize(0x0000F000);  // uDMA Channel 11 Source Select
pub const UDMA_CHMAP1_CH10SEL_M = usize(0x00000F00);  // uDMA Channel 10 Source Select
pub const UDMA_CHMAP1_CH9SEL_M = usize(0x000000F0);  // uDMA Channel 9 Source Select
pub const UDMA_CHMAP1_CH8SEL_M = usize(0x0000000F);  // uDMA Channel 8 Source Select
pub const UDMA_CHMAP1_CH15SEL_S = usize(28);
pub const UDMA_CHMAP1_CH14SEL_S = usize(24);
pub const UDMA_CHMAP1_CH13SEL_S = usize(20);
pub const UDMA_CHMAP1_CH12SEL_S = usize(16);
pub const UDMA_CHMAP1_CH11SEL_S = usize(12);
pub const UDMA_CHMAP1_CH10SEL_S = usize(8);
pub const UDMA_CHMAP1_CH9SEL_S = usize(4);
pub const UDMA_CHMAP1_CH8SEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP2 register.
//
//*****************************************************************************
pub const UDMA_CHMAP2_CH23SEL_M = usize(0xF0000000);  // uDMA Channel 23 Source Select
pub const UDMA_CHMAP2_CH22SEL_M = usize(0x0F000000);  // uDMA Channel 22 Source Select
pub const UDMA_CHMAP2_CH21SEL_M = usize(0x00F00000);  // uDMA Channel 21 Source Select
pub const UDMA_CHMAP2_CH20SEL_M = usize(0x000F0000);  // uDMA Channel 20 Source Select
pub const UDMA_CHMAP2_CH19SEL_M = usize(0x0000F000);  // uDMA Channel 19 Source Select
pub const UDMA_CHMAP2_CH18SEL_M = usize(0x00000F00);  // uDMA Channel 18 Source Select
pub const UDMA_CHMAP2_CH17SEL_M = usize(0x000000F0);  // uDMA Channel 17 Source Select
pub const UDMA_CHMAP2_CH16SEL_M = usize(0x0000000F);  // uDMA Channel 16 Source Select
pub const UDMA_CHMAP2_CH23SEL_S = usize(28);
pub const UDMA_CHMAP2_CH22SEL_S = usize(24);
pub const UDMA_CHMAP2_CH21SEL_S = usize(20);
pub const UDMA_CHMAP2_CH20SEL_S = usize(16);
pub const UDMA_CHMAP2_CH19SEL_S = usize(12);
pub const UDMA_CHMAP2_CH18SEL_S = usize(8);
pub const UDMA_CHMAP2_CH17SEL_S = usize(4);
pub const UDMA_CHMAP2_CH16SEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP3 register.
//
//*****************************************************************************
pub const UDMA_CHMAP3_CH31SEL_M = usize(0xF0000000);  // uDMA Channel 31 Source Select
pub const UDMA_CHMAP3_CH30SEL_M = usize(0x0F000000);  // uDMA Channel 30 Source Select
pub const UDMA_CHMAP3_CH29SEL_M = usize(0x00F00000);  // uDMA Channel 29 Source Select
pub const UDMA_CHMAP3_CH28SEL_M = usize(0x000F0000);  // uDMA Channel 28 Source Select
pub const UDMA_CHMAP3_CH27SEL_M = usize(0x0000F000);  // uDMA Channel 27 Source Select
pub const UDMA_CHMAP3_CH26SEL_M = usize(0x00000F00);  // uDMA Channel 26 Source Select
pub const UDMA_CHMAP3_CH25SEL_M = usize(0x000000F0);  // uDMA Channel 25 Source Select
pub const UDMA_CHMAP3_CH24SEL_M = usize(0x0000000F);  // uDMA Channel 24 Source Select
pub const UDMA_CHMAP3_CH31SEL_S = usize(28);
pub const UDMA_CHMAP3_CH30SEL_S = usize(24);
pub const UDMA_CHMAP3_CH29SEL_S = usize(20);
pub const UDMA_CHMAP3_CH28SEL_S = usize(16);
pub const UDMA_CHMAP3_CH27SEL_S = usize(12);
pub const UDMA_CHMAP3_CH26SEL_S = usize(8);
pub const UDMA_CHMAP3_CH25SEL_S = usize(4);
pub const UDMA_CHMAP3_CH24SEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the Micro Direct Memory Access (uDMA) offsets.
//
//*****************************************************************************
pub const UDMA_O_SRCENDP = usize(0x00000000);  // DMA Channel Source Address End
                                            // Pointer
pub const UDMA_O_DSTENDP = usize(0x00000004);  // DMA Channel Destination Address
                                            // End Pointer
pub const UDMA_O_CHCTL = usize(0x00000008);  // DMA Channel Control Word

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_SRCENDP register.
//
//*****************************************************************************
pub const UDMA_SRCENDP_ADDR_M = usize(0xFFFFFFFF);  // Source Address End Pointer
pub const UDMA_SRCENDP_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_DSTENDP register.
//
//*****************************************************************************
pub const UDMA_DSTENDP_ADDR_M = usize(0xFFFFFFFF);  // Destination Address End Pointer
pub const UDMA_DSTENDP_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_CHCTL register.
//
//*****************************************************************************
pub const UDMA_CHCTL_DSTINC_M = usize(0xC0000000);  // Destination Address Increment
pub const UDMA_CHCTL_DSTINC_8 = usize(0x00000000);  // Byte
pub const UDMA_CHCTL_DSTINC_16 = usize(0x40000000);  // Half-word
pub const UDMA_CHCTL_DSTINC_32 = usize(0x80000000);  // Word
pub const UDMA_CHCTL_DSTINC_NONE = usize(0xC0000000);  // No increment
pub const UDMA_CHCTL_DSTSIZE_M = usize(0x30000000);  // Destination Data Size
pub const UDMA_CHCTL_DSTSIZE_8 = usize(0x00000000);  // Byte
pub const UDMA_CHCTL_DSTSIZE_16 = usize(0x10000000);  // Half-word
pub const UDMA_CHCTL_DSTSIZE_32 = usize(0x20000000);  // Word
pub const UDMA_CHCTL_SRCINC_M = usize(0x0C000000);  // Source Address Increment
pub const UDMA_CHCTL_SRCINC_8 = usize(0x00000000);  // Byte
pub const UDMA_CHCTL_SRCINC_16 = usize(0x04000000);  // Half-word
pub const UDMA_CHCTL_SRCINC_32 = usize(0x08000000);  // Word
pub const UDMA_CHCTL_SRCINC_NONE = usize(0x0C000000);  // No increment
pub const UDMA_CHCTL_SRCSIZE_M = usize(0x03000000);  // Source Data Size
pub const UDMA_CHCTL_SRCSIZE_8 = usize(0x00000000);  // Byte
pub const UDMA_CHCTL_SRCSIZE_16 = usize(0x01000000);  // Half-word
pub const UDMA_CHCTL_SRCSIZE_32 = usize(0x02000000);  // Word
pub const UDMA_CHCTL_DSTPROT0 = usize(0x00200000);  // Destination Privilege Access
pub const UDMA_CHCTL_SRCPROT0 = usize(0x00040000);  // Source Privilege Access
pub const UDMA_CHCTL_ARBSIZE_M = usize(0x0003C000);  // Arbitration Size
pub const UDMA_CHCTL_ARBSIZE_1 = usize(0x00000000);  // 1 Transfer
pub const UDMA_CHCTL_ARBSIZE_2 = usize(0x00004000);  // 2 Transfers
pub const UDMA_CHCTL_ARBSIZE_4 = usize(0x00008000);  // 4 Transfers
pub const UDMA_CHCTL_ARBSIZE_8 = usize(0x0000C000);  // 8 Transfers
pub const UDMA_CHCTL_ARBSIZE_16 = usize(0x00010000);  // 16 Transfers
pub const UDMA_CHCTL_ARBSIZE_32 = usize(0x00014000);  // 32 Transfers
pub const UDMA_CHCTL_ARBSIZE_64 = usize(0x00018000);  // 64 Transfers
pub const UDMA_CHCTL_ARBSIZE_128 = usize(0x0001C000);  // 128 Transfers
pub const UDMA_CHCTL_ARBSIZE_256 = usize(0x00020000);  // 256 Transfers
pub const UDMA_CHCTL_ARBSIZE_512 = usize(0x00024000);  // 512 Transfers
pub const UDMA_CHCTL_ARBSIZE_1024 = usize(0x00028000);  // 1024 Transfers
pub const UDMA_CHCTL_XFERSIZE_M = usize(0x00003FF0);  // Transfer Size (minus 1)
pub const UDMA_CHCTL_NXTUSEBURST = usize(0x00000008);  // Next Useburst
pub const UDMA_CHCTL_XFERMODE_M = usize(0x00000007);  // uDMA Transfer Mode
pub const UDMA_CHCTL_XFERMODE_STOP = usize(0x00000000);  // Stop
pub const UDMA_CHCTL_XFERMODE_BASIC = usize(0x00000001);  // Basic
pub const UDMA_CHCTL_XFERMODE_AUTO = usize(0x00000002);  // Auto-Request
pub const UDMA_CHCTL_XFERMODE_PINGPONG = usize(0x00000003);  // Ping-Pong
pub const UDMA_CHCTL_XFERMODE_MEM_SG = usize(0x00000004);  // Memory Scatter-Gather
pub const UDMA_CHCTL_XFERMODE_MEM_SGA = usize(0x00000005);  // Alternate Memory Scatter-Gather
pub const UDMA_CHCTL_XFERMODE_PER_SG = usize(0x00000006);  // Peripheral Scatter-Gather
pub const UDMA_CHCTL_XFERMODE_PER_SGA = usize(0x00000007);  // Alternate Peripheral
                                            // Scatter-Gather
pub const UDMA_CHCTL_XFERSIZE_S = usize(4);

