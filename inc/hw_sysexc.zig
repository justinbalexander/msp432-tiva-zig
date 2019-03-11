//*****************************************************************************
//
// hw_sysexc.h - Macros used when accessing the system exception module.
//
// Copyright (c) 2011-2017 Texas Instruments Incorporated.  All rights reserved.
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
// The following are defines for the System Exception Module register
// addresses.
//
//*****************************************************************************
pub const SYSEXC_RIS = usize(0x400F9000);  // System Exception Raw Interrupt
                                            // Status
pub const SYSEXC_IM = usize(0x400F9004);  // System Exception Interrupt Mask
pub const SYSEXC_MIS = usize(0x400F9008);  // System Exception Masked
                                            // Interrupt Status
pub const SYSEXC_IC = usize(0x400F900C);  // System Exception Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_RIS register.
//
//*****************************************************************************
pub const SYSEXC_RIS_FPIXCRIS = usize(0x00000020);  // Floating-Point Inexact Exception
                                            // Raw Interrupt Status
pub const SYSEXC_RIS_FPOFCRIS = usize(0x00000010);  // Floating-Point Overflow
                                            // Exception Raw Interrupt Status
pub const SYSEXC_RIS_FPUFCRIS = usize(0x00000008);  // Floating-Point Underflow
                                            // Exception Raw Interrupt Status
pub const SYSEXC_RIS_FPIOCRIS = usize(0x00000004);  // Floating-Point Invalid Operation
                                            // Raw Interrupt Status
pub const SYSEXC_RIS_FPDZCRIS = usize(0x00000002);  // Floating-Point Divide By 0
                                            // Exception Raw Interrupt Status
pub const SYSEXC_RIS_FPIDCRIS = usize(0x00000001);  // Floating-Point Input Denormal
                                            // Exception Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_IM register.
//
//*****************************************************************************
pub const SYSEXC_IM_FPIXCIM = usize(0x00000020);  // Floating-Point Inexact Exception
                                            // Interrupt Mask
pub const SYSEXC_IM_FPOFCIM = usize(0x00000010);  // Floating-Point Overflow
                                            // Exception Interrupt Mask
pub const SYSEXC_IM_FPUFCIM = usize(0x00000008);  // Floating-Point Underflow
                                            // Exception Interrupt Mask
pub const SYSEXC_IM_FPIOCIM = usize(0x00000004);  // Floating-Point Invalid Operation
                                            // Interrupt Mask
pub const SYSEXC_IM_FPDZCIM = usize(0x00000002);  // Floating-Point Divide By 0
                                            // Exception Interrupt Mask
pub const SYSEXC_IM_FPIDCIM = usize(0x00000001);  // Floating-Point Input Denormal
                                            // Exception Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_MIS register.
//
//*****************************************************************************
pub const SYSEXC_MIS_FPIXCMIS = usize(0x00000020);  // Floating-Point Inexact Exception
                                            // Masked Interrupt Status
pub const SYSEXC_MIS_FPOFCMIS = usize(0x00000010);  // Floating-Point Overflow
                                            // Exception Masked Interrupt
                                            // Status
pub const SYSEXC_MIS_FPUFCMIS = usize(0x00000008);  // Floating-Point Underflow
                                            // Exception Masked Interrupt
                                            // Status
pub const SYSEXC_MIS_FPIOCMIS = usize(0x00000004);  // Floating-Point Invalid Operation
                                            // Masked Interrupt Status
pub const SYSEXC_MIS_FPDZCMIS = usize(0x00000002);  // Floating-Point Divide By 0
                                            // Exception Masked Interrupt
                                            // Status
pub const SYSEXC_MIS_FPIDCMIS = usize(0x00000001);  // Floating-Point Input Denormal
                                            // Exception Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_IC register.
//
//*****************************************************************************
pub const SYSEXC_IC_FPIXCIC = usize(0x00000020);  // Floating-Point Inexact Exception
                                            // Interrupt Clear
pub const SYSEXC_IC_FPOFCIC = usize(0x00000010);  // Floating-Point Overflow
                                            // Exception Interrupt Clear
pub const SYSEXC_IC_FPUFCIC = usize(0x00000008);  // Floating-Point Underflow
                                            // Exception Interrupt Clear
pub const SYSEXC_IC_FPIOCIC = usize(0x00000004);  // Floating-Point Invalid Operation
                                            // Interrupt Clear
pub const SYSEXC_IC_FPDZCIC = usize(0x00000002);  // Floating-Point Divide By 0
                                            // Exception Interrupt Clear
pub const SYSEXC_IC_FPIDCIC = usize(0x00000001);  // Floating-Point Input Denormal
                                            // Exception Interrupt Clear

