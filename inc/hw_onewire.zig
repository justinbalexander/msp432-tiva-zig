//*****************************************************************************
//
// hw_onewire.h - Macros used when accessing the One wire hardware.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
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
// The following are defines for the One wire register offsets.
//
//*****************************************************************************
pub const ONEWIRE_O_CS = usize(0x00000000);  // 1-Wire Control and Status
pub const ONEWIRE_O_TIM = usize(0x00000004);  // 1-Wire Timing Override
pub const ONEWIRE_O_DATW = usize(0x00000008);  // 1-Wire Data Write
pub const ONEWIRE_O_DATR = usize(0x0000000C);  // 1-Wire Data Read
pub const ONEWIRE_O_IM = usize(0x00000100);  // 1-Wire Interrupt Mask
pub const ONEWIRE_O_RIS = usize(0x00000104);  // 1-Wire Raw Interrupt Status
pub const ONEWIRE_O_MIS = usize(0x00000108);  // 1-Wire Masked Interrupt Status
pub const ONEWIRE_O_ICR = usize(0x0000010C);  // 1-Wire Interrupt Clear
pub const ONEWIRE_O_DMA = usize(0x00000120);  // 1-Wire uDMA Control
pub const ONEWIRE_O_PP = usize(0x00000FC0);  // 1-Wire Peripheral Properties

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_CS register.
//
//*****************************************************************************
pub const ONEWIRE_CS_USEALT = usize(0x80000000);  // Two Wire Enable
pub const ONEWIRE_CS_ALTP = usize(0x40000000);  // Alternate Polarity Enable
pub const ONEWIRE_CS_BSIZE_M = usize(0x00070000);  // Last Byte Size
pub const ONEWIRE_CS_BSIZE_8 = usize(0x00000000);  // 8 bits (1 byte)
pub const ONEWIRE_CS_BSIZE_1 = usize(0x00010000);  // 1 bit
pub const ONEWIRE_CS_BSIZE_2 = usize(0x00020000);  // 2 bits
pub const ONEWIRE_CS_BSIZE_3 = usize(0x00030000);  // 3 bits
pub const ONEWIRE_CS_BSIZE_4 = usize(0x00040000);  // 4 bits
pub const ONEWIRE_CS_BSIZE_5 = usize(0x00050000);  // 5 bits
pub const ONEWIRE_CS_BSIZE_6 = usize(0x00060000);  // 6 bits
pub const ONEWIRE_CS_BSIZE_7 = usize(0x00070000);  // 7 bits
pub const ONEWIRE_CS_STUCK = usize(0x00000400);  // STUCK Status
pub const ONEWIRE_CS_NOATR = usize(0x00000200);  // Answer-to-Reset Status
pub const ONEWIRE_CS_BUSY = usize(0x00000100);  // Busy Status
pub const ONEWIRE_CS_SKATR = usize(0x00000080);  // Skip Answer-to-Reset Enable
pub const ONEWIRE_CS_LSAM = usize(0x00000040);  // Late Sample Enable
pub const ONEWIRE_CS_ODRV = usize(0x00000020);  // Overdrive Enable
pub const ONEWIRE_CS_SZ_M = usize(0x00000018);  // Data Operation Size
pub const ONEWIRE_CS_OP_M = usize(0x00000006);  // Operation Request
pub const ONEWIRE_CS_OP_NONE = usize(0x00000000);  // No operation
pub const ONEWIRE_CS_OP_RD = usize(0x00000002);  // Read
pub const ONEWIRE_CS_OP_WR = usize(0x00000004);  // Write
pub const ONEWIRE_CS_OP_WRRD = usize(0x00000006);  // Write/Read
pub const ONEWIRE_CS_RST = usize(0x00000001);  // Reset Request
pub const ONEWIRE_CS_SZ_S = usize(3);

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_TIM register.
//
//*****************************************************************************
pub const ONEWIRE_TIM_W1TIM_M = usize(0xF0000000);  // Value '1' Timing
pub const ONEWIRE_TIM_W0TIM_M = usize(0x0F800000);  // Value '0' Timing
pub const ONEWIRE_TIM_W0REST_M = usize(0x00780000);  // Rest Time
pub const ONEWIRE_TIM_W1SAM_M = usize(0x00078000);  // Sample Time
pub const ONEWIRE_TIM_ATRSAM_M = usize(0x00007800);  // Answer-to-Reset Sample
pub const ONEWIRE_TIM_ATRTIM_M = usize(0x000007C0);  // Answer-to-Reset/Rest Period
pub const ONEWIRE_TIM_RSTTIM_M = usize(0x0000003F);  // Reset Low Time
pub const ONEWIRE_TIM_W1TIM_S = usize(28);
pub const ONEWIRE_TIM_W0TIM_S = usize(23);
pub const ONEWIRE_TIM_W0REST_S = usize(19);
pub const ONEWIRE_TIM_W1SAM_S = usize(15);
pub const ONEWIRE_TIM_ATRSAM_S = usize(11);
pub const ONEWIRE_TIM_ATRTIM_S = usize(6);
pub const ONEWIRE_TIM_RSTTIM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_DATW register.
//
//*****************************************************************************
pub const ONEWIRE_DATW_B3_M = usize(0xFF000000);  // Upper Data Byte
pub const ONEWIRE_DATW_B2_M = usize(0x00FF0000);  // Upper Middle Data Byte
pub const ONEWIRE_DATW_B1_M = usize(0x0000FF00);  // Lower Middle Data Byte
pub const ONEWIRE_DATW_B0_M = usize(0x000000FF);  // Lowest Data Byte
pub const ONEWIRE_DATW_B3_S = usize(24);
pub const ONEWIRE_DATW_B2_S = usize(16);
pub const ONEWIRE_DATW_B1_S = usize(8);
pub const ONEWIRE_DATW_B0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_DATR register.
//
//*****************************************************************************
pub const ONEWIRE_DATR_B3_M = usize(0xFF000000);  // Upper Data Byte
pub const ONEWIRE_DATR_B2_M = usize(0x00FF0000);  // Upper Middle Data Byte
pub const ONEWIRE_DATR_B1_M = usize(0x0000FF00);  // Lower Middle Data Byte
pub const ONEWIRE_DATR_B0_M = usize(0x000000FF);  // Lowest Data Byte
pub const ONEWIRE_DATR_B3_S = usize(24);
pub const ONEWIRE_DATR_B2_S = usize(16);
pub const ONEWIRE_DATR_B1_S = usize(8);
pub const ONEWIRE_DATR_B0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_IM register.
//
//*****************************************************************************
pub const ONEWIRE_IM_DMA = usize(0x00000010);  // DMA Done Interrupt Mask
pub const ONEWIRE_IM_STUCK = usize(0x00000008);  // Stuck Status Interrupt Mask
pub const ONEWIRE_IM_NOATR = usize(0x00000004);  // No Answer-to-Reset Interrupt
                                            // Mask
pub const ONEWIRE_IM_OPC = usize(0x00000002);  // Operation Complete Interrupt
                                            // Mask
pub const ONEWIRE_IM_RST = usize(0x00000001);  // Reset Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_RIS register.
//
//*****************************************************************************
pub const ONEWIRE_RIS_DMA = usize(0x00000010);  // DMA Done Raw Interrupt Status
pub const ONEWIRE_RIS_STUCK = usize(0x00000008);  // Stuck Status Raw Interrupt
                                            // Status
pub const ONEWIRE_RIS_NOATR = usize(0x00000004);  // No Answer-to-Reset Raw Interrupt
                                            // Status
pub const ONEWIRE_RIS_OPC = usize(0x00000002);  // Operation Complete Raw Interrupt
                                            // Status
pub const ONEWIRE_RIS_RST = usize(0x00000001);  // Reset Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_MIS register.
//
//*****************************************************************************
pub const ONEWIRE_MIS_DMA = usize(0x00000010);  // DMA Done Masked Interrupt Status
pub const ONEWIRE_MIS_STUCK = usize(0x00000008);  // Stuck Status Masked Interrupt
                                            // Status
pub const ONEWIRE_MIS_NOATR = usize(0x00000004);  // No Answer-to-Reset Masked
                                            // Interrupt Status
pub const ONEWIRE_MIS_OPC = usize(0x00000002);  // Operation Complete Masked
                                            // Interrupt Status
pub const ONEWIRE_MIS_RST = usize(0x00000001);  // Reset Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_ICR register.
//
//*****************************************************************************
pub const ONEWIRE_ICR_DMA = usize(0x00000010);  // DMA Done Interrupt Clear
pub const ONEWIRE_ICR_STUCK = usize(0x00000008);  // Stuck Status Interrupt Clear
pub const ONEWIRE_ICR_NOATR = usize(0x00000004);  // No Answer-to-Reset Interrupt
                                            // Clear
pub const ONEWIRE_ICR_OPC = usize(0x00000002);  // Operation Complete Interrupt
                                            // Clear
pub const ONEWIRE_ICR_RST = usize(0x00000001);  // Reset Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_DMA register.
//
//*****************************************************************************
pub const ONEWIRE_DMA_SG = usize(0x00000008);  // Scatter-Gather Enable
pub const ONEWIRE_DMA_DMAOP_M = usize(0x00000006);  // uDMA Operation
pub const ONEWIRE_DMA_DMAOP_DIS = usize(0x00000000);  // uDMA disabled
pub const ONEWIRE_DMA_DMAOP_RDSNG = usize(0x00000002);  // uDMA single read: 1-Wire
                                            // requests uDMA to read
                                            // ONEWIREDATR register after each
                                            // read transaction
pub const ONEWIRE_DMA_DMAOP_WRMUL = usize(0x00000004);  // uDMA multiple write: 1-Wire
                                            // requests uDMA to load whenever
                                            // the ONEWIREDATW register is
                                            // empty
pub const ONEWIRE_DMA_DMAOP_RDMUL = usize(0x00000006);  // uDMA multiple read: An initial
                                            // read occurs and subsequent reads
                                            // start after uDMA has read the
                                            // ONEWIREDATR register
pub const ONEWIRE_DMA_RST = usize(0x00000001);  // uDMA Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_PP register.
//
//*****************************************************************************
pub const ONEWIRE_PP_DMAP = usize(0x00000010);  // uDMA Present
pub const ONEWIRE_PP_CNT_M = usize(0x00000003);  // 1-Wire Bus Count
pub const ONEWIRE_PP_CNT_S = usize(0);

