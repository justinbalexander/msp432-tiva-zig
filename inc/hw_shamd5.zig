//*****************************************************************************
//
// hw_shamd5.h - Macros used when accessing the SHA/MD5 hardware.
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
// The following are defines for the SHA/MD5 register offsets.
//
//*****************************************************************************
pub const SHAMD5_O_ODIGEST_A = usize(0x00000000);  // SHA Outer Digest A
pub const SHAMD5_O_ODIGEST_B = usize(0x00000004);  // SHA Outer Digest B
pub const SHAMD5_O_ODIGEST_C = usize(0x00000008);  // SHA Outer Digest C
pub const SHAMD5_O_ODIGEST_D = usize(0x0000000C);  // SHA Outer Digest D
pub const SHAMD5_O_ODIGEST_E = usize(0x00000010);  // SHA Outer Digest E
pub const SHAMD5_O_ODIGEST_F = usize(0x00000014);  // SHA Outer Digest F
pub const SHAMD5_O_ODIGEST_G = usize(0x00000018);  // SHA Outer Digest G
pub const SHAMD5_O_ODIGEST_H = usize(0x0000001C);  // SHA Outer Digest H
pub const SHAMD5_O_IDIGEST_A = usize(0x00000020);  // SHA Inner Digest A
pub const SHAMD5_O_IDIGEST_B = usize(0x00000024);  // SHA Inner Digest B
pub const SHAMD5_O_IDIGEST_C = usize(0x00000028);  // SHA Inner Digest C
pub const SHAMD5_O_IDIGEST_D = usize(0x0000002C);  // SHA Inner Digest D
pub const SHAMD5_O_IDIGEST_E = usize(0x00000030);  // SHA Inner Digest E
pub const SHAMD5_O_IDIGEST_F = usize(0x00000034);  // SHA Inner Digest F
pub const SHAMD5_O_IDIGEST_G = usize(0x00000038);  // SHA Inner Digest G
pub const SHAMD5_O_IDIGEST_H = usize(0x0000003C);  // SHA Inner Digest H
pub const SHAMD5_O_DIGEST_COUNT = usize(0x00000040);  // SHA Digest Count
pub const SHAMD5_O_MODE = usize(0x00000044);  // SHA Mode
pub const SHAMD5_O_LENGTH = usize(0x00000048);  // SHA Length
pub const SHAMD5_O_DATA_0_IN = usize(0x00000080);  // SHA Data 0 Input
pub const SHAMD5_O_DATA_1_IN = usize(0x00000084);  // SHA Data 1 Input
pub const SHAMD5_O_DATA_2_IN = usize(0x00000088);  // SHA Data 2 Input
pub const SHAMD5_O_DATA_3_IN = usize(0x0000008C);  // SHA Data 3 Input
pub const SHAMD5_O_DATA_4_IN = usize(0x00000090);  // SHA Data 4 Input
pub const SHAMD5_O_DATA_5_IN = usize(0x00000094);  // SHA Data 5 Input
pub const SHAMD5_O_DATA_6_IN = usize(0x00000098);  // SHA Data 6 Input
pub const SHAMD5_O_DATA_7_IN = usize(0x0000009C);  // SHA Data 7 Input
pub const SHAMD5_O_DATA_8_IN = usize(0x000000A0);  // SHA Data 8 Input
pub const SHAMD5_O_DATA_9_IN = usize(0x000000A4);  // SHA Data 9 Input
pub const SHAMD5_O_DATA_10_IN = usize(0x000000A8);  // SHA Data 10 Input
pub const SHAMD5_O_DATA_11_IN = usize(0x000000AC);  // SHA Data 11 Input
pub const SHAMD5_O_DATA_12_IN = usize(0x000000B0);  // SHA Data 12 Input
pub const SHAMD5_O_DATA_13_IN = usize(0x000000B4);  // SHA Data 13 Input
pub const SHAMD5_O_DATA_14_IN = usize(0x000000B8);  // SHA Data 14 Input
pub const SHAMD5_O_DATA_15_IN = usize(0x000000BC);  // SHA Data 15 Input
pub const SHAMD5_O_REVISION = usize(0x00000100);  // SHA Revision
pub const SHAMD5_O_SYSCONFIG = usize(0x00000110);  // SHA System Configuration
pub const SHAMD5_O_SYSSTATUS = usize(0x00000114);  // SHA System Status
pub const SHAMD5_O_IRQSTATUS = usize(0x00000118);  // SHA Interrupt Status
pub const SHAMD5_O_IRQENABLE = usize(0x0000011C);  // SHA Interrupt Enable
pub const SHAMD5_O_DMAIM = usize(0xFFFFC010);  // SHA DMA Interrupt Mask
pub const SHAMD5_O_DMARIS = usize(0xFFFFC014);  // SHA DMA Raw Interrupt Status
pub const SHAMD5_O_DMAMIS = usize(0xFFFFC018);  // SHA DMA Masked Interrupt Status
pub const SHAMD5_O_DMAIC = usize(0xFFFFC01C);  // SHA DMA Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_A
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_A_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_A_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_B
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_B_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_B_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_C
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_C_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_C_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_D
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_D_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_D_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_E
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_E_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_E_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_F
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_F_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_F_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_G
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_G_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_G_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_H
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_H_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_H_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_A
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_A_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_A_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_B
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_B_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_B_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_C
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_C_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_C_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_D
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_D_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_D_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_E
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_E_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_E_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_F
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_F_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_F_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_G
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_G_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_G_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_H
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_H_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_H_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DIGEST_COUNT
// register.
//
//*****************************************************************************
pub const SHAMD5_DIGEST_COUNT_M = usize(0xFFFFFFFF);  // Digest Count
pub const SHAMD5_DIGEST_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_MODE register.
//
//*****************************************************************************
pub const SHAMD5_MODE_HMAC_OUTER_HASH = usize(0x00000080);  // HMAC Outer Hash Processing
                                            // Enable
pub const SHAMD5_MODE_HMAC_KEY_PROC = usize(0x00000020);  // HMAC Key Processing Enable
pub const SHAMD5_MODE_CLOSE_HASH = usize(0x00000010);  // Performs the padding, the
                                            // Hash/HMAC will be 'closed' at
                                            // the end of the block, as per
                                            // MD5/SHA-1/SHA-2 specification
pub const SHAMD5_MODE_ALGO_CONSTANT = usize(0x00000008);  // The initial digest register will
                                            // be overwritten with the
                                            // algorithm constants for the
                                            // selected algorithm when hashing
                                            // and the initial digest count
                                            // register will be reset to 0
pub const SHAMD5_MODE_ALGO_M = usize(0x00000007);  // Hash Algorithm
pub const SHAMD5_MODE_ALGO_MD5 = usize(0x00000000);  // MD5
pub const SHAMD5_MODE_ALGO_SHA1 = usize(0x00000002);  // SHA-1
pub const SHAMD5_MODE_ALGO_SHA224 = usize(0x00000004);  // SHA-224
pub const SHAMD5_MODE_ALGO_SHA256 = usize(0x00000006);  // SHA-256

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_LENGTH
// register.
//
//*****************************************************************************
pub const SHAMD5_LENGTH_M = usize(0xFFFFFFFF);  // Block Length/Remaining Byte
                                            // Count
pub const SHAMD5_LENGTH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_0_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_0_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_0_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_1_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_1_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_1_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_2_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_2_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_2_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_3_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_3_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_3_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_4_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_4_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_4_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_5_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_5_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_5_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_6_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_6_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_6_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_7_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_7_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_7_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_8_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_8_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_8_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_9_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_9_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_9_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_10_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_10_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_10_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_11_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_11_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_11_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_12_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_12_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_12_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_13_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_13_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_13_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_14_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_14_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_14_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_15_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_15_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_15_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_REVISION
// register.
//
//*****************************************************************************
pub const SHAMD5_REVISION_M = usize(0xFFFFFFFF);  // Revision Number
pub const SHAMD5_REVISION_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_SYSCONFIG
// register.
//
//*****************************************************************************
pub const SHAMD5_SYSCONFIG_SADVANCED = usize(0x00000080);  // Advanced Mode Enable
pub const SHAMD5_SYSCONFIG_SIDLE_M = usize(0x00000030);  // Sidle mode
pub const SHAMD5_SYSCONFIG_SIDLE_FORCE = usize(0x00000000);  // Force-idle mode
pub const SHAMD5_SYSCONFIG_DMA_EN = usize(0x00000008);  // uDMA Request Enable
pub const SHAMD5_SYSCONFIG_IT_EN = usize(0x00000004);  // Interrupt Enable
pub const SHAMD5_SYSCONFIG_SOFTRESET = usize(0x00000002);  // Soft reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_SYSSTATUS
// register.
//
//*****************************************************************************
pub const SHAMD5_SYSSTATUS_RESETDONE = usize(0x00000001);  // Reset done status

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IRQSTATUS
// register.
//
//*****************************************************************************
pub const SHAMD5_IRQSTATUS_CONTEXT_READY = usize(0x00000008);  // Context Ready Status
pub const SHAMD5_IRQSTATUS_INPUT_READY = usize(0x00000002);  // Input Ready Status
pub const SHAMD5_IRQSTATUS_OUTPUT_READY = usize(0x00000001);  // Output Ready Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IRQENABLE
// register.
//
//*****************************************************************************
pub const SHAMD5_IRQENABLE_CONTEXT_READY = usize(0x00000008);  // Mask for context ready interrupt
pub const SHAMD5_IRQENABLE_INPUT_READY = usize(0x00000002);  // Mask for input ready interrupt
pub const SHAMD5_IRQENABLE_OUTPUT_READY = usize(0x00000001);  // Mask for output ready interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DMAIM register.
//
//*****************************************************************************
pub const SHAMD5_DMAIM_COUT = usize(0x00000004);  // Context Out DMA Done Interrupt
                                            // Mask
pub const SHAMD5_DMAIM_DIN = usize(0x00000002);  // Data In DMA Done Interrupt Mask
pub const SHAMD5_DMAIM_CIN = usize(0x00000001);  // Context In DMA Done Interrupt
                                            // Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DMARIS
// register.
//
//*****************************************************************************
pub const SHAMD5_DMARIS_COUT = usize(0x00000004);  // Context Out DMA Done Raw
                                            // Interrupt Status
pub const SHAMD5_DMARIS_DIN = usize(0x00000002);  // Data In DMA Done Raw Interrupt
                                            // Status
pub const SHAMD5_DMARIS_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DMAMIS
// register.
//
//*****************************************************************************
pub const SHAMD5_DMAMIS_COUT = usize(0x00000004);  // Context Out DMA Done Masked
                                            // Interrupt Status
pub const SHAMD5_DMAMIS_DIN = usize(0x00000002);  // Data In DMA Done Masked
                                            // Interrupt Status
pub const SHAMD5_DMAMIS_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DMAIC register.
//
//*****************************************************************************
pub const SHAMD5_DMAIC_COUT = usize(0x00000004);  // Context Out DMA Done Masked
                                            // Interrupt Status
pub const SHAMD5_DMAIC_DIN = usize(0x00000002);  // Data In DMA Done Interrupt Clear
pub const SHAMD5_DMAIC_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

