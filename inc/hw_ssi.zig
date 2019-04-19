//*****************************************************************************
//
// hw_ssi.h - Macros used when accessing the SSI hardware.
//
// Copyright (c) 2005-2017 Texas Instruments Incorporated.  All rights reserved.
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
// The following are defines for the SSI register offsets.
//
//*****************************************************************************
pub const SSI_O_CR0 = usize(0x00000000);  // SSI Control 0
pub const SSI_O_CR1 = usize(0x00000004);  // SSI Control 1
pub const SSI_O_DR = usize(0x00000008);  // SSI Data
pub const SSI_O_SR = usize(0x0000000C);  // SSI Status
pub const SSI_O_CPSR = usize(0x00000010);  // SSI Clock Prescale
pub const SSI_O_IM = usize(0x00000014);  // SSI Interrupt Mask
pub const SSI_O_RIS = usize(0x00000018);  // SSI Raw Interrupt Status
pub const SSI_O_MIS = usize(0x0000001C);  // SSI Masked Interrupt Status
pub const SSI_O_ICR = usize(0x00000020);  // SSI Interrupt Clear
pub const SSI_O_DMACTL = usize(0x00000024);  // SSI DMA Control
pub const SSI_O_PP = usize(0x00000FC0);  // SSI Peripheral Properties
pub const SSI_O_CC = usize(0x00000FC8);  // SSI Clock Configuration

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR0 register.
//
//*****************************************************************************
pub const SSI_CR0_SCR_M = usize(0x0000FF00);  // SSI Serial Clock Rate
pub const SSI_CR0_SPH = usize(0x00000080);  // SSI Serial Clock Phase
pub const SSI_CR0_SPO = usize(0x00000040);  // SSI Serial Clock Polarity
pub const SSI_CR0_FRF_M = usize(0x00000030);  // SSI Frame Format Select
pub const SSI_CR0_FRF_MOTO = usize(0x00000000);  // Freescale SPI Frame Format
pub const SSI_CR0_FRF_TI = usize(0x00000010);  // Synchronous Serial Frame Format
pub const SSI_CR0_FRF_NMW = usize(0x00000020);  // MICROWIRE Frame Format
pub const SSI_CR0_DSS_M = usize(0x0000000F);  // SSI Data Size Select
pub const SSI_CR0_DSS_4 = usize(0x00000003);  // 4-bit data
pub const SSI_CR0_DSS_5 = usize(0x00000004);  // 5-bit data
pub const SSI_CR0_DSS_6 = usize(0x00000005);  // 6-bit data
pub const SSI_CR0_DSS_7 = usize(0x00000006);  // 7-bit data
pub const SSI_CR0_DSS_8 = usize(0x00000007);  // 8-bit data
pub const SSI_CR0_DSS_9 = usize(0x00000008);  // 9-bit data
pub const SSI_CR0_DSS_10 = usize(0x00000009);  // 10-bit data
pub const SSI_CR0_DSS_11 = usize(0x0000000A);  // 11-bit data
pub const SSI_CR0_DSS_12 = usize(0x0000000B);  // 12-bit data
pub const SSI_CR0_DSS_13 = usize(0x0000000C);  // 13-bit data
pub const SSI_CR0_DSS_14 = usize(0x0000000D);  // 14-bit data
pub const SSI_CR0_DSS_15 = usize(0x0000000E);  // 15-bit data
pub const SSI_CR0_DSS_16 = usize(0x0000000F);  // 16-bit data
pub const SSI_CR0_SCR_S = usize(8);

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR1 register.
//
//*****************************************************************************
pub const SSI_CR1_EOM = usize(0x00000800);  // Stop Frame (End of Message)
pub const SSI_CR1_FSSHLDFRM = usize(0x00000400);  // FSS Hold Frame
pub const SSI_CR1_HSCLKEN = usize(0x00000200);  // High Speed Clock Enable
pub const SSI_CR1_DIR = usize(0x00000100);  // SSI Direction of Operation
pub const SSI_CR1_MODE_M = usize(0x000000C0);  // SSI Mode
pub const SSI_CR1_MODE_LEGACY = usize(0x00000000);  // Legacy SSI mode
pub const SSI_CR1_MODE_BI = usize(0x00000040);  // Bi-SSI mode
pub const SSI_CR1_MODE_QUAD = usize(0x00000080);  // Quad-SSI Mode
pub const SSI_CR1_MODE_ADVANCED = usize(0x000000C0);  // Advanced SSI Mode with 8-bit
                                            // packet size
pub const SSI_CR1_EOT = usize(0x00000010);  // End of Transmission
pub const SSI_CR1_MS = usize(0x00000004);  // SSI Master/Slave Select
pub const SSI_CR1_SSE = usize(0x00000002);  // SSI Synchronous Serial Port
                                            // Enable
pub const SSI_CR1_LBM = usize(0x00000001);  // SSI Loopback Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DR register.
//
//*****************************************************************************
pub const SSI_DR_DATA_M = usize(0x0000FFFF);  // SSI Receive/Transmit Data
pub const SSI_DR_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_SR register.
//
//*****************************************************************************
pub const SSI_SR_BSY = usize(0x00000010);  // SSI Busy Bit
pub const SSI_SR_RFF = usize(0x00000008);  // SSI Receive FIFO Full
pub const SSI_SR_RNE = usize(0x00000004);  // SSI Receive FIFO Not Empty
pub const SSI_SR_TNF = usize(0x00000002);  // SSI Transmit FIFO Not Full
pub const SSI_SR_TFE = usize(0x00000001);  // SSI Transmit FIFO Empty

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CPSR register.
//
//*****************************************************************************
pub const SSI_CPSR_CPSDVSR_M = usize(0x000000FF);  // SSI Clock Prescale Divisor
pub const SSI_CPSR_CPSDVSR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_IM register.
//
//*****************************************************************************
pub const SSI_IM_EOTIM = usize(0x00000040);  // End of Transmit Interrupt Mask
pub const SSI_IM_DMATXIM = usize(0x00000020);  // SSI Transmit DMA Interrupt Mask
pub const SSI_IM_DMARXIM = usize(0x00000010);  // SSI Receive DMA Interrupt Mask
pub const SSI_IM_TXIM = usize(0x00000008);  // SSI Transmit FIFO Interrupt Mask
pub const SSI_IM_RXIM = usize(0x00000004);  // SSI Receive FIFO Interrupt Mask
pub const SSI_IM_RTIM = usize(0x00000002);  // SSI Receive Time-Out Interrupt
                                            // Mask
pub const SSI_IM_RORIM = usize(0x00000001);  // SSI Receive Overrun Interrupt
                                            // Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_RIS register.
//
//*****************************************************************************
pub const SSI_RIS_EOTRIS = usize(0x00000040);  // End of Transmit Raw Interrupt
                                            // Status
pub const SSI_RIS_DMATXRIS = usize(0x00000020);  // SSI Transmit DMA Raw Interrupt
                                            // Status
pub const SSI_RIS_DMARXRIS = usize(0x00000010);  // SSI Receive DMA Raw Interrupt
                                            // Status
pub const SSI_RIS_TXRIS = usize(0x00000008);  // SSI Transmit FIFO Raw Interrupt
                                            // Status
pub const SSI_RIS_RXRIS = usize(0x00000004);  // SSI Receive FIFO Raw Interrupt
                                            // Status
pub const SSI_RIS_RTRIS = usize(0x00000002);  // SSI Receive Time-Out Raw
                                            // Interrupt Status
pub const SSI_RIS_RORRIS = usize(0x00000001);  // SSI Receive Overrun Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_MIS register.
//
//*****************************************************************************
pub const SSI_MIS_EOTMIS = usize(0x00000040);  // End of Transmit Masked Interrupt
                                            // Status
pub const SSI_MIS_DMATXMIS = usize(0x00000020);  // SSI Transmit DMA Masked
                                            // Interrupt Status
pub const SSI_MIS_DMARXMIS = usize(0x00000010);  // SSI Receive DMA Masked Interrupt
                                            // Status
pub const SSI_MIS_TXMIS = usize(0x00000008);  // SSI Transmit FIFO Masked
                                            // Interrupt Status
pub const SSI_MIS_RXMIS = usize(0x00000004);  // SSI Receive FIFO Masked
                                            // Interrupt Status
pub const SSI_MIS_RTMIS = usize(0x00000002);  // SSI Receive Time-Out Masked
                                            // Interrupt Status
pub const SSI_MIS_RORMIS = usize(0x00000001);  // SSI Receive Overrun Masked
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_ICR register.
//
//*****************************************************************************
pub const SSI_ICR_EOTIC = usize(0x00000040);  // End of Transmit Interrupt Clear
pub const SSI_ICR_DMATXIC = usize(0x00000020);  // SSI Transmit DMA Interrupt Clear
pub const SSI_ICR_DMARXIC = usize(0x00000010);  // SSI Receive DMA Interrupt Clear
pub const SSI_ICR_RTIC = usize(0x00000002);  // SSI Receive Time-Out Interrupt
                                            // Clear
pub const SSI_ICR_RORIC = usize(0x00000001);  // SSI Receive Overrun Interrupt
                                            // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DMACTL register.
//
//*****************************************************************************
pub const SSI_DMACTL_TXDMAE = usize(0x00000002);  // Transmit DMA Enable
pub const SSI_DMACTL_RXDMAE = usize(0x00000001);  // Receive DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_PP register.
//
//*****************************************************************************
pub const SSI_PP_FSSHLDFRM = usize(0x00000008);  // FSS Hold Frame Capability
pub const SSI_PP_MODE_M = usize(0x00000006);  // Mode of Operation
pub const SSI_PP_MODE_LEGACY = usize(0x00000000);  // Legacy SSI mode
pub const SSI_PP_MODE_ADVBI = usize(0x00000002);  // Legacy mode, Advanced SSI mode
                                            // and Bi-SSI mode enabled
pub const SSI_PP_MODE_ADVBIQUAD = usize(0x00000004);  // Legacy mode, Advanced mode,
                                            // Bi-SSI and Quad-SSI mode enabled
pub const SSI_PP_HSCLK = usize(0x00000001);  // High Speed Capability

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CC register.
//
//*****************************************************************************
pub const SSI_CC_CS_M = usize(0x0000000F);  // SSI Baud Clock Source
pub const SSI_CC_CS_SYSPLL = usize(0x00000000);  // System clock (based on clock
                                            // source and divisor factor)
pub const SSI_CC_CS_PIOSC = usize(0x00000005);  // PIOSC

