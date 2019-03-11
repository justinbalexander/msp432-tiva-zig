//*****************************************************************************
//
// hw_pwm.h - Defines and Macros for Pulse Width Modulation (PWM) ports.
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
// The following are defines for the PWM register offsets.
//
//*****************************************************************************
pub const PWM_O_CTL = usize(0x00000000);  // PWM Master Control
pub const PWM_O_SYNC = usize(0x00000004);  // PWM Time Base Sync
pub const PWM_O_ENABLE = usize(0x00000008);  // PWM Output Enable
pub const PWM_O_INVERT = usize(0x0000000C);  // PWM Output Inversion
pub const PWM_O_FAULT = usize(0x00000010);  // PWM Output Fault
pub const PWM_O_INTEN = usize(0x00000014);  // PWM Interrupt Enable
pub const PWM_O_RIS = usize(0x00000018);  // PWM Raw Interrupt Status
pub const PWM_O_ISC = usize(0x0000001C);  // PWM Interrupt Status and Clear
pub const PWM_O_STATUS = usize(0x00000020);  // PWM Status
pub const PWM_O_FAULTVAL = usize(0x00000024);  // PWM Fault Condition Value
pub const PWM_O_ENUPD = usize(0x00000028);  // PWM Enable Update
pub const PWM_O_0_CTL = usize(0x00000040);  // PWM0 Control
pub const PWM_O_0_INTEN = usize(0x00000044);  // PWM0 Interrupt and Trigger
                                            // Enable
pub const PWM_O_0_RIS = usize(0x00000048);  // PWM0 Raw Interrupt Status
pub const PWM_O_0_ISC = usize(0x0000004C);  // PWM0 Interrupt Status and Clear
pub const PWM_O_0_LOAD = usize(0x00000050);  // PWM0 Load
pub const PWM_O_0_COUNT = usize(0x00000054);  // PWM0 Counter
pub const PWM_O_0_CMPA = usize(0x00000058);  // PWM0 Compare A
pub const PWM_O_0_CMPB = usize(0x0000005C);  // PWM0 Compare B
pub const PWM_O_0_GENA = usize(0x00000060);  // PWM0 Generator A Control
pub const PWM_O_0_GENB = usize(0x00000064);  // PWM0 Generator B Control
pub const PWM_O_0_DBCTL = usize(0x00000068);  // PWM0 Dead-Band Control
pub const PWM_O_0_DBRISE = usize(0x0000006C);  // PWM0 Dead-Band Rising-Edge Delay
pub const PWM_O_0_DBFALL = usize(0x00000070);  // PWM0 Dead-Band
                                            // Falling-Edge-Delay
pub const PWM_O_0_FLTSRC0 = usize(0x00000074);  // PWM0 Fault Source 0
pub const PWM_O_0_FLTSRC1 = usize(0x00000078);  // PWM0 Fault Source 1
pub const PWM_O_0_MINFLTPER = usize(0x0000007C);  // PWM0 Minimum Fault Period
pub const PWM_O_1_CTL = usize(0x00000080);  // PWM1 Control
pub const PWM_O_1_INTEN = usize(0x00000084);  // PWM1 Interrupt and Trigger
                                            // Enable
pub const PWM_O_1_RIS = usize(0x00000088);  // PWM1 Raw Interrupt Status
pub const PWM_O_1_ISC = usize(0x0000008C);  // PWM1 Interrupt Status and Clear
pub const PWM_O_1_LOAD = usize(0x00000090);  // PWM1 Load
pub const PWM_O_1_COUNT = usize(0x00000094);  // PWM1 Counter
pub const PWM_O_1_CMPA = usize(0x00000098);  // PWM1 Compare A
pub const PWM_O_1_CMPB = usize(0x0000009C);  // PWM1 Compare B
pub const PWM_O_1_GENA = usize(0x000000A0);  // PWM1 Generator A Control
pub const PWM_O_1_GENB = usize(0x000000A4);  // PWM1 Generator B Control
pub const PWM_O_1_DBCTL = usize(0x000000A8);  // PWM1 Dead-Band Control
pub const PWM_O_1_DBRISE = usize(0x000000AC);  // PWM1 Dead-Band Rising-Edge Delay
pub const PWM_O_1_DBFALL = usize(0x000000B0);  // PWM1 Dead-Band
                                            // Falling-Edge-Delay
pub const PWM_O_1_FLTSRC0 = usize(0x000000B4);  // PWM1 Fault Source 0
pub const PWM_O_1_FLTSRC1 = usize(0x000000B8);  // PWM1 Fault Source 1
pub const PWM_O_1_MINFLTPER = usize(0x000000BC);  // PWM1 Minimum Fault Period
pub const PWM_O_2_CTL = usize(0x000000C0);  // PWM2 Control
pub const PWM_O_2_INTEN = usize(0x000000C4);  // PWM2 Interrupt and Trigger
                                            // Enable
pub const PWM_O_2_RIS = usize(0x000000C8);  // PWM2 Raw Interrupt Status
pub const PWM_O_2_ISC = usize(0x000000CC);  // PWM2 Interrupt Status and Clear
pub const PWM_O_2_LOAD = usize(0x000000D0);  // PWM2 Load
pub const PWM_O_2_COUNT = usize(0x000000D4);  // PWM2 Counter
pub const PWM_O_2_CMPA = usize(0x000000D8);  // PWM2 Compare A
pub const PWM_O_2_CMPB = usize(0x000000DC);  // PWM2 Compare B
pub const PWM_O_2_GENA = usize(0x000000E0);  // PWM2 Generator A Control
pub const PWM_O_2_GENB = usize(0x000000E4);  // PWM2 Generator B Control
pub const PWM_O_2_DBCTL = usize(0x000000E8);  // PWM2 Dead-Band Control
pub const PWM_O_2_DBRISE = usize(0x000000EC);  // PWM2 Dead-Band Rising-Edge Delay
pub const PWM_O_2_DBFALL = usize(0x000000F0);  // PWM2 Dead-Band
                                            // Falling-Edge-Delay
pub const PWM_O_2_FLTSRC0 = usize(0x000000F4);  // PWM2 Fault Source 0
pub const PWM_O_2_FLTSRC1 = usize(0x000000F8);  // PWM2 Fault Source 1
pub const PWM_O_2_MINFLTPER = usize(0x000000FC);  // PWM2 Minimum Fault Period
pub const PWM_O_3_CTL = usize(0x00000100);  // PWM3 Control
pub const PWM_O_3_INTEN = usize(0x00000104);  // PWM3 Interrupt and Trigger
                                            // Enable
pub const PWM_O_3_RIS = usize(0x00000108);  // PWM3 Raw Interrupt Status
pub const PWM_O_3_ISC = usize(0x0000010C);  // PWM3 Interrupt Status and Clear
pub const PWM_O_3_LOAD = usize(0x00000110);  // PWM3 Load
pub const PWM_O_3_COUNT = usize(0x00000114);  // PWM3 Counter
pub const PWM_O_3_CMPA = usize(0x00000118);  // PWM3 Compare A
pub const PWM_O_3_CMPB = usize(0x0000011C);  // PWM3 Compare B
pub const PWM_O_3_GENA = usize(0x00000120);  // PWM3 Generator A Control
pub const PWM_O_3_GENB = usize(0x00000124);  // PWM3 Generator B Control
pub const PWM_O_3_DBCTL = usize(0x00000128);  // PWM3 Dead-Band Control
pub const PWM_O_3_DBRISE = usize(0x0000012C);  // PWM3 Dead-Band Rising-Edge Delay
pub const PWM_O_3_DBFALL = usize(0x00000130);  // PWM3 Dead-Band
                                            // Falling-Edge-Delay
pub const PWM_O_3_FLTSRC0 = usize(0x00000134);  // PWM3 Fault Source 0
pub const PWM_O_3_FLTSRC1 = usize(0x00000138);  // PWM3 Fault Source 1
pub const PWM_O_3_MINFLTPER = usize(0x0000013C);  // PWM3 Minimum Fault Period
pub const PWM_O_0_FLTSEN = usize(0x00000800);  // PWM0 Fault Pin Logic Sense
pub const PWM_O_0_FLTSTAT0 = usize(0x00000804);  // PWM0 Fault Status 0
pub const PWM_O_0_FLTSTAT1 = usize(0x00000808);  // PWM0 Fault Status 1
pub const PWM_O_1_FLTSEN = usize(0x00000880);  // PWM1 Fault Pin Logic Sense
pub const PWM_O_1_FLTSTAT0 = usize(0x00000884);  // PWM1 Fault Status 0
pub const PWM_O_1_FLTSTAT1 = usize(0x00000888);  // PWM1 Fault Status 1
pub const PWM_O_2_FLTSEN = usize(0x00000900);  // PWM2 Fault Pin Logic Sense
pub const PWM_O_2_FLTSTAT0 = usize(0x00000904);  // PWM2 Fault Status 0
pub const PWM_O_2_FLTSTAT1 = usize(0x00000908);  // PWM2 Fault Status 1
pub const PWM_O_3_FLTSEN = usize(0x00000980);  // PWM3 Fault Pin Logic Sense
pub const PWM_O_3_FLTSTAT0 = usize(0x00000984);  // PWM3 Fault Status 0
pub const PWM_O_3_FLTSTAT1 = usize(0x00000988);  // PWM3 Fault Status 1
pub const PWM_O_PP = usize(0x00000FC0);  // PWM Peripheral Properties
pub const PWM_O_CC = usize(0x00000FC8);  // PWM Clock Configuration

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_CTL register.
//
//*****************************************************************************
pub const PWM_CTL_GLOBALSYNC3 = usize(0x00000008);  // Update PWM Generator 3
pub const PWM_CTL_GLOBALSYNC2 = usize(0x00000004);  // Update PWM Generator 2
pub const PWM_CTL_GLOBALSYNC1 = usize(0x00000002);  // Update PWM Generator 1
pub const PWM_CTL_GLOBALSYNC0 = usize(0x00000001);  // Update PWM Generator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_SYNC register.
//
//*****************************************************************************
pub const PWM_SYNC_SYNC3 = usize(0x00000008);  // Reset Generator 3 Counter
pub const PWM_SYNC_SYNC2 = usize(0x00000004);  // Reset Generator 2 Counter
pub const PWM_SYNC_SYNC1 = usize(0x00000002);  // Reset Generator 1 Counter
pub const PWM_SYNC_SYNC0 = usize(0x00000001);  // Reset Generator 0 Counter

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENABLE register.
//
//*****************************************************************************
pub const PWM_ENABLE_PWM7EN = usize(0x00000080);  // MnPWM7 Output Enable
pub const PWM_ENABLE_PWM6EN = usize(0x00000040);  // MnPWM6 Output Enable
pub const PWM_ENABLE_PWM5EN = usize(0x00000020);  // MnPWM5 Output Enable
pub const PWM_ENABLE_PWM4EN = usize(0x00000010);  // MnPWM4 Output Enable
pub const PWM_ENABLE_PWM3EN = usize(0x00000008);  // MnPWM3 Output Enable
pub const PWM_ENABLE_PWM2EN = usize(0x00000004);  // MnPWM2 Output Enable
pub const PWM_ENABLE_PWM1EN = usize(0x00000002);  // MnPWM1 Output Enable
pub const PWM_ENABLE_PWM0EN = usize(0x00000001);  // MnPWM0 Output Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INVERT register.
//
//*****************************************************************************
pub const PWM_INVERT_PWM7INV = usize(0x00000080);  // Invert MnPWM7 Signal
pub const PWM_INVERT_PWM6INV = usize(0x00000040);  // Invert MnPWM6 Signal
pub const PWM_INVERT_PWM5INV = usize(0x00000020);  // Invert MnPWM5 Signal
pub const PWM_INVERT_PWM4INV = usize(0x00000010);  // Invert MnPWM4 Signal
pub const PWM_INVERT_PWM3INV = usize(0x00000008);  // Invert MnPWM3 Signal
pub const PWM_INVERT_PWM2INV = usize(0x00000004);  // Invert MnPWM2 Signal
pub const PWM_INVERT_PWM1INV = usize(0x00000002);  // Invert MnPWM1 Signal
pub const PWM_INVERT_PWM0INV = usize(0x00000001);  // Invert MnPWM0 Signal

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULT register.
//
//*****************************************************************************
pub const PWM_FAULT_FAULT7 = usize(0x00000080);  // MnPWM7 Fault
pub const PWM_FAULT_FAULT6 = usize(0x00000040);  // MnPWM6 Fault
pub const PWM_FAULT_FAULT5 = usize(0x00000020);  // MnPWM5 Fault
pub const PWM_FAULT_FAULT4 = usize(0x00000010);  // MnPWM4 Fault
pub const PWM_FAULT_FAULT3 = usize(0x00000008);  // MnPWM3 Fault
pub const PWM_FAULT_FAULT2 = usize(0x00000004);  // MnPWM2 Fault
pub const PWM_FAULT_FAULT1 = usize(0x00000002);  // MnPWM1 Fault
pub const PWM_FAULT_FAULT0 = usize(0x00000001);  // MnPWM0 Fault

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INTEN register.
//
//*****************************************************************************
pub const PWM_INTEN_INTFAULT3 = usize(0x00080000);  // Interrupt Fault 3
pub const PWM_INTEN_INTFAULT2 = usize(0x00040000);  // Interrupt Fault 2
pub const PWM_INTEN_INTFAULT1 = usize(0x00020000);  // Interrupt Fault 1
pub const PWM_INTEN_INTFAULT0 = usize(0x00010000);  // Interrupt Fault 0
pub const PWM_INTEN_INTPWM3 = usize(0x00000008);  // PWM3 Interrupt Enable
pub const PWM_INTEN_INTPWM2 = usize(0x00000004);  // PWM2 Interrupt Enable
pub const PWM_INTEN_INTPWM1 = usize(0x00000002);  // PWM1 Interrupt Enable
pub const PWM_INTEN_INTPWM0 = usize(0x00000001);  // PWM0 Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_RIS register.
//
//*****************************************************************************
pub const PWM_RIS_INTFAULT3 = usize(0x00080000);  // Interrupt Fault PWM 3
pub const PWM_RIS_INTFAULT2 = usize(0x00040000);  // Interrupt Fault PWM 2
pub const PWM_RIS_INTFAULT1 = usize(0x00020000);  // Interrupt Fault PWM 1
pub const PWM_RIS_INTFAULT0 = usize(0x00010000);  // Interrupt Fault PWM 0
pub const PWM_RIS_INTPWM3 = usize(0x00000008);  // PWM3 Interrupt Asserted
pub const PWM_RIS_INTPWM2 = usize(0x00000004);  // PWM2 Interrupt Asserted
pub const PWM_RIS_INTPWM1 = usize(0x00000002);  // PWM1 Interrupt Asserted
pub const PWM_RIS_INTPWM0 = usize(0x00000001);  // PWM0 Interrupt Asserted

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ISC register.
//
//*****************************************************************************
pub const PWM_ISC_INTFAULT3 = usize(0x00080000);  // FAULT3 Interrupt Asserted
pub const PWM_ISC_INTFAULT2 = usize(0x00040000);  // FAULT2 Interrupt Asserted
pub const PWM_ISC_INTFAULT1 = usize(0x00020000);  // FAULT1 Interrupt Asserted
pub const PWM_ISC_INTFAULT0 = usize(0x00010000);  // FAULT0 Interrupt Asserted
pub const PWM_ISC_INTPWM3 = usize(0x00000008);  // PWM3 Interrupt Status
pub const PWM_ISC_INTPWM2 = usize(0x00000004);  // PWM2 Interrupt Status
pub const PWM_ISC_INTPWM1 = usize(0x00000002);  // PWM1 Interrupt Status
pub const PWM_ISC_INTPWM0 = usize(0x00000001);  // PWM0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_STATUS register.
//
//*****************************************************************************
pub const PWM_STATUS_FAULT3 = usize(0x00000008);  // Generator 3 Fault Status
pub const PWM_STATUS_FAULT2 = usize(0x00000004);  // Generator 2 Fault Status
pub const PWM_STATUS_FAULT1 = usize(0x00000002);  // Generator 1 Fault Status
pub const PWM_STATUS_FAULT0 = usize(0x00000001);  // Generator 0 Fault Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULTVAL register.
//
//*****************************************************************************
pub const PWM_FAULTVAL_PWM7 = usize(0x00000080);  // MnPWM7 Fault Value
pub const PWM_FAULTVAL_PWM6 = usize(0x00000040);  // MnPWM6 Fault Value
pub const PWM_FAULTVAL_PWM5 = usize(0x00000020);  // MnPWM5 Fault Value
pub const PWM_FAULTVAL_PWM4 = usize(0x00000010);  // MnPWM4 Fault Value
pub const PWM_FAULTVAL_PWM3 = usize(0x00000008);  // MnPWM3 Fault Value
pub const PWM_FAULTVAL_PWM2 = usize(0x00000004);  // MnPWM2 Fault Value
pub const PWM_FAULTVAL_PWM1 = usize(0x00000002);  // MnPWM1 Fault Value
pub const PWM_FAULTVAL_PWM0 = usize(0x00000001);  // MnPWM0 Fault Value

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENUPD register.
//
//*****************************************************************************
pub const PWM_ENUPD_ENUPD7_M = usize(0x0000C000);  // MnPWM7 Enable Update Mode
pub const PWM_ENUPD_ENUPD7_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD7_LSYNC = usize(0x00008000);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD7_GSYNC = usize(0x0000C000);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD6_M = usize(0x00003000);  // MnPWM6 Enable Update Mode
pub const PWM_ENUPD_ENUPD6_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD6_LSYNC = usize(0x00002000);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD6_GSYNC = usize(0x00003000);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD5_M = usize(0x00000C00);  // MnPWM5 Enable Update Mode
pub const PWM_ENUPD_ENUPD5_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD5_LSYNC = usize(0x00000800);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD5_GSYNC = usize(0x00000C00);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD4_M = usize(0x00000300);  // MnPWM4 Enable Update Mode
pub const PWM_ENUPD_ENUPD4_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD4_LSYNC = usize(0x00000200);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD4_GSYNC = usize(0x00000300);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD3_M = usize(0x000000C0);  // MnPWM3 Enable Update Mode
pub const PWM_ENUPD_ENUPD3_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD3_LSYNC = usize(0x00000080);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD3_GSYNC = usize(0x000000C0);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD2_M = usize(0x00000030);  // MnPWM2 Enable Update Mode
pub const PWM_ENUPD_ENUPD2_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD2_LSYNC = usize(0x00000020);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD2_GSYNC = usize(0x00000030);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD1_M = usize(0x0000000C);  // MnPWM1 Enable Update Mode
pub const PWM_ENUPD_ENUPD1_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD1_LSYNC = usize(0x00000008);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD1_GSYNC = usize(0x0000000C);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD0_M = usize(0x00000003);  // MnPWM0 Enable Update Mode
pub const PWM_ENUPD_ENUPD0_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD0_LSYNC = usize(0x00000002);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD0_GSYNC = usize(0x00000003);  // Globally Synchronized

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CTL register.
//
//*****************************************************************************
pub const PWM_0_CTL_LATCH = usize(0x00040000);  // Latch Fault Input
pub const PWM_0_CTL_MINFLTPER = usize(0x00020000);  // Minimum Fault Period
pub const PWM_0_CTL_FLTSRC = usize(0x00010000);  // Fault Condition Source
pub const PWM_0_CTL_DBFALLUPD_M = usize(0x0000C000);  // PWMnDBFALL Update Mode
pub const PWM_0_CTL_DBFALLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_0_CTL_DBFALLUPD_LS = usize(0x00008000);  // Locally Synchronized
pub const PWM_0_CTL_DBFALLUPD_GS = usize(0x0000C000);  // Globally Synchronized
pub const PWM_0_CTL_DBRISEUPD_M = usize(0x00003000);  // PWMnDBRISE Update Mode
pub const PWM_0_CTL_DBRISEUPD_I = usize(0x00000000);  // Immediate
pub const PWM_0_CTL_DBRISEUPD_LS = usize(0x00002000);  // Locally Synchronized
pub const PWM_0_CTL_DBRISEUPD_GS = usize(0x00003000);  // Globally Synchronized
pub const PWM_0_CTL_DBCTLUPD_M = usize(0x00000C00);  // PWMnDBCTL Update Mode
pub const PWM_0_CTL_DBCTLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_0_CTL_DBCTLUPD_LS = usize(0x00000800);  // Locally Synchronized
pub const PWM_0_CTL_DBCTLUPD_GS = usize(0x00000C00);  // Globally Synchronized
pub const PWM_0_CTL_GENBUPD_M = usize(0x00000300);  // PWMnGENB Update Mode
pub const PWM_0_CTL_GENBUPD_I = usize(0x00000000);  // Immediate
pub const PWM_0_CTL_GENBUPD_LS = usize(0x00000200);  // Locally Synchronized
pub const PWM_0_CTL_GENBUPD_GS = usize(0x00000300);  // Globally Synchronized
pub const PWM_0_CTL_GENAUPD_M = usize(0x000000C0);  // PWMnGENA Update Mode
pub const PWM_0_CTL_GENAUPD_I = usize(0x00000000);  // Immediate
pub const PWM_0_CTL_GENAUPD_LS = usize(0x00000080);  // Locally Synchronized
pub const PWM_0_CTL_GENAUPD_GS = usize(0x000000C0);  // Globally Synchronized
pub const PWM_0_CTL_CMPBUPD = usize(0x00000020);  // Comparator B Update Mode
pub const PWM_0_CTL_CMPAUPD = usize(0x00000010);  // Comparator A Update Mode
pub const PWM_0_CTL_LOADUPD = usize(0x00000008);  // Load Register Update Mode
pub const PWM_0_CTL_DEBUG = usize(0x00000004);  // Debug Mode
pub const PWM_0_CTL_MODE = usize(0x00000002);  // Counter Mode
pub const PWM_0_CTL_ENABLE = usize(0x00000001);  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_INTEN register.
//
//*****************************************************************************
pub const PWM_0_INTEN_TRCMPBD = usize(0x00002000);  // Trigger for Counter=PWMnCMPB
                                            // Down
pub const PWM_0_INTEN_TRCMPBU = usize(0x00001000);  // Trigger for Counter=PWMnCMPB Up
pub const PWM_0_INTEN_TRCMPAD = usize(0x00000800);  // Trigger for Counter=PWMnCMPA
                                            // Down
pub const PWM_0_INTEN_TRCMPAU = usize(0x00000400);  // Trigger for Counter=PWMnCMPA Up
pub const PWM_0_INTEN_TRCNTLOAD = usize(0x00000200);  // Trigger for Counter=PWMnLOAD
pub const PWM_0_INTEN_TRCNTZERO = usize(0x00000100);  // Trigger for Counter=0
pub const PWM_0_INTEN_INTCMPBD = usize(0x00000020);  // Interrupt for Counter=PWMnCMPB
                                            // Down
pub const PWM_0_INTEN_INTCMPBU = usize(0x00000010);  // Interrupt for Counter=PWMnCMPB
                                            // Up
pub const PWM_0_INTEN_INTCMPAD = usize(0x00000008);  // Interrupt for Counter=PWMnCMPA
                                            // Down
pub const PWM_0_INTEN_INTCMPAU = usize(0x00000004);  // Interrupt for Counter=PWMnCMPA
                                            // Up
pub const PWM_0_INTEN_INTCNTLOAD = usize(0x00000002);  // Interrupt for Counter=PWMnLOAD
pub const PWM_0_INTEN_INTCNTZERO = usize(0x00000001);  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_RIS register.
//
//*****************************************************************************
pub const PWM_0_RIS_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
                                            // Status
pub const PWM_0_RIS_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt Status
pub const PWM_0_RIS_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
                                            // Status
pub const PWM_0_RIS_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt Status
pub const PWM_0_RIS_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt Status
pub const PWM_0_RIS_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_ISC register.
//
//*****************************************************************************
pub const PWM_0_ISC_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
pub const PWM_0_ISC_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt
pub const PWM_0_ISC_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
pub const PWM_0_ISC_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt
pub const PWM_0_ISC_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt
pub const PWM_0_ISC_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_LOAD register.
//
//*****************************************************************************
pub const PWM_0_LOAD_M = usize(0x0000FFFF);  // Counter Load Value
pub const PWM_0_LOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_COUNT register.
//
//*****************************************************************************
pub const PWM_0_COUNT_M = usize(0x0000FFFF);  // Counter Value
pub const PWM_0_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPA register.
//
//*****************************************************************************
pub const PWM_0_CMPA_M = usize(0x0000FFFF);  // Comparator A Value
pub const PWM_0_CMPA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPB register.
//
//*****************************************************************************
pub const PWM_0_CMPB_M = usize(0x0000FFFF);  // Comparator B Value
pub const PWM_0_CMPB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENA register.
//
//*****************************************************************************
pub const PWM_0_GENA_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_0_GENA_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmA
pub const PWM_0_GENA_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmA Low
pub const PWM_0_GENA_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmA High
pub const PWM_0_GENA_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_0_GENA_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmA
pub const PWM_0_GENA_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmA Low
pub const PWM_0_GENA_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmA High
pub const PWM_0_GENA_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_0_GENA_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmA
pub const PWM_0_GENA_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmA Low
pub const PWM_0_GENA_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmA High
pub const PWM_0_GENA_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_0_GENA_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmA
pub const PWM_0_GENA_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmA Low
pub const PWM_0_GENA_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmA High
pub const PWM_0_GENA_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_0_GENA_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTLOAD_INV = usize(0x00000004);  // Invert pwmA
pub const PWM_0_GENA_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmA Low
pub const PWM_0_GENA_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmA High
pub const PWM_0_GENA_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_0_GENA_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTZERO_INV = usize(0x00000001);  // Invert pwmA
pub const PWM_0_GENA_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmA Low
pub const PWM_0_GENA_ACTZERO_ONE = usize(0x00000003);  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENB register.
//
//*****************************************************************************
pub const PWM_0_GENB_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_0_GENB_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmB
pub const PWM_0_GENB_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmB Low
pub const PWM_0_GENB_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmB High
pub const PWM_0_GENB_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_0_GENB_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmB
pub const PWM_0_GENB_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmB Low
pub const PWM_0_GENB_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmB High
pub const PWM_0_GENB_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_0_GENB_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmB
pub const PWM_0_GENB_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmB Low
pub const PWM_0_GENB_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmB High
pub const PWM_0_GENB_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_0_GENB_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmB
pub const PWM_0_GENB_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmB Low
pub const PWM_0_GENB_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmB High
pub const PWM_0_GENB_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_0_GENB_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTLOAD_INV = usize(0x00000004);  // Invert pwmB
pub const PWM_0_GENB_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmB Low
pub const PWM_0_GENB_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmB High
pub const PWM_0_GENB_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_0_GENB_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTZERO_INV = usize(0x00000001);  // Invert pwmB
pub const PWM_0_GENB_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmB Low
pub const PWM_0_GENB_ACTZERO_ONE = usize(0x00000003);  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBCTL register.
//
//*****************************************************************************
pub const PWM_0_DBCTL_ENABLE = usize(0x00000001);  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBRISE register.
//
//*****************************************************************************
pub const PWM_0_DBRISE_DELAY_M = usize(0x00000FFF);  // Dead-Band Rise Delay
pub const PWM_0_DBRISE_DELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBFALL register.
//
//*****************************************************************************
pub const PWM_0_DBFALL_DELAY_M = usize(0x00000FFF);  // Dead-Band Fall Delay
pub const PWM_0_DBFALL_DELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM_0_FLTSRC0_FAULT3 = usize(0x00000008);  // Fault3 Input
pub const PWM_0_FLTSRC0_FAULT2 = usize(0x00000004);  // Fault2 Input
pub const PWM_0_FLTSRC0_FAULT1 = usize(0x00000002);  // Fault1 Input
pub const PWM_0_FLTSRC0_FAULT0 = usize(0x00000001);  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM_0_FLTSRC1_DCMP7 = usize(0x00000080);  // Digital Comparator 7
pub const PWM_0_FLTSRC1_DCMP6 = usize(0x00000040);  // Digital Comparator 6
pub const PWM_0_FLTSRC1_DCMP5 = usize(0x00000020);  // Digital Comparator 5
pub const PWM_0_FLTSRC1_DCMP4 = usize(0x00000010);  // Digital Comparator 4
pub const PWM_0_FLTSRC1_DCMP3 = usize(0x00000008);  // Digital Comparator 3
pub const PWM_0_FLTSRC1_DCMP2 = usize(0x00000004);  // Digital Comparator 2
pub const PWM_0_FLTSRC1_DCMP1 = usize(0x00000002);  // Digital Comparator 1
pub const PWM_0_FLTSRC1_DCMP0 = usize(0x00000001);  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM_0_MINFLTPER_M = usize(0x0000FFFF);  // Minimum Fault Period
pub const PWM_0_MINFLTPER_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CTL register.
//
//*****************************************************************************
pub const PWM_1_CTL_LATCH = usize(0x00040000);  // Latch Fault Input
pub const PWM_1_CTL_MINFLTPER = usize(0x00020000);  // Minimum Fault Period
pub const PWM_1_CTL_FLTSRC = usize(0x00010000);  // Fault Condition Source
pub const PWM_1_CTL_DBFALLUPD_M = usize(0x0000C000);  // PWMnDBFALL Update Mode
pub const PWM_1_CTL_DBFALLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_1_CTL_DBFALLUPD_LS = usize(0x00008000);  // Locally Synchronized
pub const PWM_1_CTL_DBFALLUPD_GS = usize(0x0000C000);  // Globally Synchronized
pub const PWM_1_CTL_DBRISEUPD_M = usize(0x00003000);  // PWMnDBRISE Update Mode
pub const PWM_1_CTL_DBRISEUPD_I = usize(0x00000000);  // Immediate
pub const PWM_1_CTL_DBRISEUPD_LS = usize(0x00002000);  // Locally Synchronized
pub const PWM_1_CTL_DBRISEUPD_GS = usize(0x00003000);  // Globally Synchronized
pub const PWM_1_CTL_DBCTLUPD_M = usize(0x00000C00);  // PWMnDBCTL Update Mode
pub const PWM_1_CTL_DBCTLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_1_CTL_DBCTLUPD_LS = usize(0x00000800);  // Locally Synchronized
pub const PWM_1_CTL_DBCTLUPD_GS = usize(0x00000C00);  // Globally Synchronized
pub const PWM_1_CTL_GENBUPD_M = usize(0x00000300);  // PWMnGENB Update Mode
pub const PWM_1_CTL_GENBUPD_I = usize(0x00000000);  // Immediate
pub const PWM_1_CTL_GENBUPD_LS = usize(0x00000200);  // Locally Synchronized
pub const PWM_1_CTL_GENBUPD_GS = usize(0x00000300);  // Globally Synchronized
pub const PWM_1_CTL_GENAUPD_M = usize(0x000000C0);  // PWMnGENA Update Mode
pub const PWM_1_CTL_GENAUPD_I = usize(0x00000000);  // Immediate
pub const PWM_1_CTL_GENAUPD_LS = usize(0x00000080);  // Locally Synchronized
pub const PWM_1_CTL_GENAUPD_GS = usize(0x000000C0);  // Globally Synchronized
pub const PWM_1_CTL_CMPBUPD = usize(0x00000020);  // Comparator B Update Mode
pub const PWM_1_CTL_CMPAUPD = usize(0x00000010);  // Comparator A Update Mode
pub const PWM_1_CTL_LOADUPD = usize(0x00000008);  // Load Register Update Mode
pub const PWM_1_CTL_DEBUG = usize(0x00000004);  // Debug Mode
pub const PWM_1_CTL_MODE = usize(0x00000002);  // Counter Mode
pub const PWM_1_CTL_ENABLE = usize(0x00000001);  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_INTEN register.
//
//*****************************************************************************
pub const PWM_1_INTEN_TRCMPBD = usize(0x00002000);  // Trigger for Counter=PWMnCMPB
                                            // Down
pub const PWM_1_INTEN_TRCMPBU = usize(0x00001000);  // Trigger for Counter=PWMnCMPB Up
pub const PWM_1_INTEN_TRCMPAD = usize(0x00000800);  // Trigger for Counter=PWMnCMPA
                                            // Down
pub const PWM_1_INTEN_TRCMPAU = usize(0x00000400);  // Trigger for Counter=PWMnCMPA Up
pub const PWM_1_INTEN_TRCNTLOAD = usize(0x00000200);  // Trigger for Counter=PWMnLOAD
pub const PWM_1_INTEN_TRCNTZERO = usize(0x00000100);  // Trigger for Counter=0
pub const PWM_1_INTEN_INTCMPBD = usize(0x00000020);  // Interrupt for Counter=PWMnCMPB
                                            // Down
pub const PWM_1_INTEN_INTCMPBU = usize(0x00000010);  // Interrupt for Counter=PWMnCMPB
                                            // Up
pub const PWM_1_INTEN_INTCMPAD = usize(0x00000008);  // Interrupt for Counter=PWMnCMPA
                                            // Down
pub const PWM_1_INTEN_INTCMPAU = usize(0x00000004);  // Interrupt for Counter=PWMnCMPA
                                            // Up
pub const PWM_1_INTEN_INTCNTLOAD = usize(0x00000002);  // Interrupt for Counter=PWMnLOAD
pub const PWM_1_INTEN_INTCNTZERO = usize(0x00000001);  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_RIS register.
//
//*****************************************************************************
pub const PWM_1_RIS_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
                                            // Status
pub const PWM_1_RIS_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt Status
pub const PWM_1_RIS_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
                                            // Status
pub const PWM_1_RIS_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt Status
pub const PWM_1_RIS_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt Status
pub const PWM_1_RIS_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_ISC register.
//
//*****************************************************************************
pub const PWM_1_ISC_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
pub const PWM_1_ISC_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt
pub const PWM_1_ISC_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
pub const PWM_1_ISC_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt
pub const PWM_1_ISC_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt
pub const PWM_1_ISC_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_LOAD register.
//
//*****************************************************************************
pub const PWM_1_LOAD_LOAD_M = usize(0x0000FFFF);  // Counter Load Value
pub const PWM_1_LOAD_LOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_COUNT register.
//
//*****************************************************************************
pub const PWM_1_COUNT_COUNT_M = usize(0x0000FFFF);  // Counter Value
pub const PWM_1_COUNT_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CMPA register.
//
//*****************************************************************************
pub const PWM_1_CMPA_COMPA_M = usize(0x0000FFFF);  // Comparator A Value
pub const PWM_1_CMPA_COMPA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CMPB register.
//
//*****************************************************************************
pub const PWM_1_CMPB_COMPB_M = usize(0x0000FFFF);  // Comparator B Value
pub const PWM_1_CMPB_COMPB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_GENA register.
//
//*****************************************************************************
pub const PWM_1_GENA_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_1_GENA_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmA
pub const PWM_1_GENA_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmA Low
pub const PWM_1_GENA_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmA High
pub const PWM_1_GENA_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_1_GENA_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmA
pub const PWM_1_GENA_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmA Low
pub const PWM_1_GENA_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmA High
pub const PWM_1_GENA_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_1_GENA_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmA
pub const PWM_1_GENA_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmA Low
pub const PWM_1_GENA_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmA High
pub const PWM_1_GENA_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_1_GENA_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmA
pub const PWM_1_GENA_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmA Low
pub const PWM_1_GENA_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmA High
pub const PWM_1_GENA_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_1_GENA_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTLOAD_INV = usize(0x00000004);  // Invert pwmA
pub const PWM_1_GENA_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmA Low
pub const PWM_1_GENA_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmA High
pub const PWM_1_GENA_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_1_GENA_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTZERO_INV = usize(0x00000001);  // Invert pwmA
pub const PWM_1_GENA_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmA Low
pub const PWM_1_GENA_ACTZERO_ONE = usize(0x00000003);  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_GENB register.
//
//*****************************************************************************
pub const PWM_1_GENB_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_1_GENB_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmB
pub const PWM_1_GENB_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmB Low
pub const PWM_1_GENB_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmB High
pub const PWM_1_GENB_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_1_GENB_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmB
pub const PWM_1_GENB_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmB Low
pub const PWM_1_GENB_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmB High
pub const PWM_1_GENB_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_1_GENB_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmB
pub const PWM_1_GENB_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmB Low
pub const PWM_1_GENB_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmB High
pub const PWM_1_GENB_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_1_GENB_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmB
pub const PWM_1_GENB_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmB Low
pub const PWM_1_GENB_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmB High
pub const PWM_1_GENB_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_1_GENB_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTLOAD_INV = usize(0x00000004);  // Invert pwmB
pub const PWM_1_GENB_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmB Low
pub const PWM_1_GENB_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmB High
pub const PWM_1_GENB_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_1_GENB_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTZERO_INV = usize(0x00000001);  // Invert pwmB
pub const PWM_1_GENB_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmB Low
pub const PWM_1_GENB_ACTZERO_ONE = usize(0x00000003);  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBCTL register.
//
//*****************************************************************************
pub const PWM_1_DBCTL_ENABLE = usize(0x00000001);  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBRISE register.
//
//*****************************************************************************
pub const PWM_1_DBRISE_RISEDELAY_M = usize(0x00000FFF);  // Dead-Band Rise Delay
pub const PWM_1_DBRISE_RISEDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBFALL register.
//
//*****************************************************************************
pub const PWM_1_DBFALL_FALLDELAY_M = usize(0x00000FFF);  // Dead-Band Fall Delay
pub const PWM_1_DBFALL_FALLDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM_1_FLTSRC0_FAULT3 = usize(0x00000008);  // Fault3 Input
pub const PWM_1_FLTSRC0_FAULT2 = usize(0x00000004);  // Fault2 Input
pub const PWM_1_FLTSRC0_FAULT1 = usize(0x00000002);  // Fault1 Input
pub const PWM_1_FLTSRC0_FAULT0 = usize(0x00000001);  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM_1_FLTSRC1_DCMP7 = usize(0x00000080);  // Digital Comparator 7
pub const PWM_1_FLTSRC1_DCMP6 = usize(0x00000040);  // Digital Comparator 6
pub const PWM_1_FLTSRC1_DCMP5 = usize(0x00000020);  // Digital Comparator 5
pub const PWM_1_FLTSRC1_DCMP4 = usize(0x00000010);  // Digital Comparator 4
pub const PWM_1_FLTSRC1_DCMP3 = usize(0x00000008);  // Digital Comparator 3
pub const PWM_1_FLTSRC1_DCMP2 = usize(0x00000004);  // Digital Comparator 2
pub const PWM_1_FLTSRC1_DCMP1 = usize(0x00000002);  // Digital Comparator 1
pub const PWM_1_FLTSRC1_DCMP0 = usize(0x00000001);  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM_1_MINFLTPER_MFP_M = usize(0x0000FFFF);  // Minimum Fault Period
pub const PWM_1_MINFLTPER_MFP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CTL register.
//
//*****************************************************************************
pub const PWM_2_CTL_LATCH = usize(0x00040000);  // Latch Fault Input
pub const PWM_2_CTL_MINFLTPER = usize(0x00020000);  // Minimum Fault Period
pub const PWM_2_CTL_FLTSRC = usize(0x00010000);  // Fault Condition Source
pub const PWM_2_CTL_DBFALLUPD_M = usize(0x0000C000);  // PWMnDBFALL Update Mode
pub const PWM_2_CTL_DBFALLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_2_CTL_DBFALLUPD_LS = usize(0x00008000);  // Locally Synchronized
pub const PWM_2_CTL_DBFALLUPD_GS = usize(0x0000C000);  // Globally Synchronized
pub const PWM_2_CTL_DBRISEUPD_M = usize(0x00003000);  // PWMnDBRISE Update Mode
pub const PWM_2_CTL_DBRISEUPD_I = usize(0x00000000);  // Immediate
pub const PWM_2_CTL_DBRISEUPD_LS = usize(0x00002000);  // Locally Synchronized
pub const PWM_2_CTL_DBRISEUPD_GS = usize(0x00003000);  // Globally Synchronized
pub const PWM_2_CTL_DBCTLUPD_M = usize(0x00000C00);  // PWMnDBCTL Update Mode
pub const PWM_2_CTL_DBCTLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_2_CTL_DBCTLUPD_LS = usize(0x00000800);  // Locally Synchronized
pub const PWM_2_CTL_DBCTLUPD_GS = usize(0x00000C00);  // Globally Synchronized
pub const PWM_2_CTL_GENBUPD_M = usize(0x00000300);  // PWMnGENB Update Mode
pub const PWM_2_CTL_GENBUPD_I = usize(0x00000000);  // Immediate
pub const PWM_2_CTL_GENBUPD_LS = usize(0x00000200);  // Locally Synchronized
pub const PWM_2_CTL_GENBUPD_GS = usize(0x00000300);  // Globally Synchronized
pub const PWM_2_CTL_GENAUPD_M = usize(0x000000C0);  // PWMnGENA Update Mode
pub const PWM_2_CTL_GENAUPD_I = usize(0x00000000);  // Immediate
pub const PWM_2_CTL_GENAUPD_LS = usize(0x00000080);  // Locally Synchronized
pub const PWM_2_CTL_GENAUPD_GS = usize(0x000000C0);  // Globally Synchronized
pub const PWM_2_CTL_CMPBUPD = usize(0x00000020);  // Comparator B Update Mode
pub const PWM_2_CTL_CMPAUPD = usize(0x00000010);  // Comparator A Update Mode
pub const PWM_2_CTL_LOADUPD = usize(0x00000008);  // Load Register Update Mode
pub const PWM_2_CTL_DEBUG = usize(0x00000004);  // Debug Mode
pub const PWM_2_CTL_MODE = usize(0x00000002);  // Counter Mode
pub const PWM_2_CTL_ENABLE = usize(0x00000001);  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_INTEN register.
//
//*****************************************************************************
pub const PWM_2_INTEN_TRCMPBD = usize(0x00002000);  // Trigger for Counter=PWMnCMPB
                                            // Down
pub const PWM_2_INTEN_TRCMPBU = usize(0x00001000);  // Trigger for Counter=PWMnCMPB Up
pub const PWM_2_INTEN_TRCMPAD = usize(0x00000800);  // Trigger for Counter=PWMnCMPA
                                            // Down
pub const PWM_2_INTEN_TRCMPAU = usize(0x00000400);  // Trigger for Counter=PWMnCMPA Up
pub const PWM_2_INTEN_TRCNTLOAD = usize(0x00000200);  // Trigger for Counter=PWMnLOAD
pub const PWM_2_INTEN_TRCNTZERO = usize(0x00000100);  // Trigger for Counter=0
pub const PWM_2_INTEN_INTCMPBD = usize(0x00000020);  // Interrupt for Counter=PWMnCMPB
                                            // Down
pub const PWM_2_INTEN_INTCMPBU = usize(0x00000010);  // Interrupt for Counter=PWMnCMPB
                                            // Up
pub const PWM_2_INTEN_INTCMPAD = usize(0x00000008);  // Interrupt for Counter=PWMnCMPA
                                            // Down
pub const PWM_2_INTEN_INTCMPAU = usize(0x00000004);  // Interrupt for Counter=PWMnCMPA
                                            // Up
pub const PWM_2_INTEN_INTCNTLOAD = usize(0x00000002);  // Interrupt for Counter=PWMnLOAD
pub const PWM_2_INTEN_INTCNTZERO = usize(0x00000001);  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_RIS register.
//
//*****************************************************************************
pub const PWM_2_RIS_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
                                            // Status
pub const PWM_2_RIS_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt Status
pub const PWM_2_RIS_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
                                            // Status
pub const PWM_2_RIS_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt Status
pub const PWM_2_RIS_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt Status
pub const PWM_2_RIS_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_ISC register.
//
//*****************************************************************************
pub const PWM_2_ISC_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
pub const PWM_2_ISC_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt
pub const PWM_2_ISC_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
pub const PWM_2_ISC_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt
pub const PWM_2_ISC_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt
pub const PWM_2_ISC_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_LOAD register.
//
//*****************************************************************************
pub const PWM_2_LOAD_LOAD_M = usize(0x0000FFFF);  // Counter Load Value
pub const PWM_2_LOAD_LOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_COUNT register.
//
//*****************************************************************************
pub const PWM_2_COUNT_COUNT_M = usize(0x0000FFFF);  // Counter Value
pub const PWM_2_COUNT_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CMPA register.
//
//*****************************************************************************
pub const PWM_2_CMPA_COMPA_M = usize(0x0000FFFF);  // Comparator A Value
pub const PWM_2_CMPA_COMPA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CMPB register.
//
//*****************************************************************************
pub const PWM_2_CMPB_COMPB_M = usize(0x0000FFFF);  // Comparator B Value
pub const PWM_2_CMPB_COMPB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_GENA register.
//
//*****************************************************************************
pub const PWM_2_GENA_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_2_GENA_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmA
pub const PWM_2_GENA_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmA Low
pub const PWM_2_GENA_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmA High
pub const PWM_2_GENA_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_2_GENA_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmA
pub const PWM_2_GENA_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmA Low
pub const PWM_2_GENA_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmA High
pub const PWM_2_GENA_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_2_GENA_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmA
pub const PWM_2_GENA_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmA Low
pub const PWM_2_GENA_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmA High
pub const PWM_2_GENA_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_2_GENA_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmA
pub const PWM_2_GENA_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmA Low
pub const PWM_2_GENA_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmA High
pub const PWM_2_GENA_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_2_GENA_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTLOAD_INV = usize(0x00000004);  // Invert pwmA
pub const PWM_2_GENA_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmA Low
pub const PWM_2_GENA_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmA High
pub const PWM_2_GENA_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_2_GENA_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTZERO_INV = usize(0x00000001);  // Invert pwmA
pub const PWM_2_GENA_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmA Low
pub const PWM_2_GENA_ACTZERO_ONE = usize(0x00000003);  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_GENB register.
//
//*****************************************************************************
pub const PWM_2_GENB_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_2_GENB_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmB
pub const PWM_2_GENB_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmB Low
pub const PWM_2_GENB_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmB High
pub const PWM_2_GENB_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_2_GENB_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmB
pub const PWM_2_GENB_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmB Low
pub const PWM_2_GENB_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmB High
pub const PWM_2_GENB_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_2_GENB_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmB
pub const PWM_2_GENB_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmB Low
pub const PWM_2_GENB_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmB High
pub const PWM_2_GENB_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_2_GENB_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmB
pub const PWM_2_GENB_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmB Low
pub const PWM_2_GENB_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmB High
pub const PWM_2_GENB_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_2_GENB_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTLOAD_INV = usize(0x00000004);  // Invert pwmB
pub const PWM_2_GENB_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmB Low
pub const PWM_2_GENB_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmB High
pub const PWM_2_GENB_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_2_GENB_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTZERO_INV = usize(0x00000001);  // Invert pwmB
pub const PWM_2_GENB_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmB Low
pub const PWM_2_GENB_ACTZERO_ONE = usize(0x00000003);  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBCTL register.
//
//*****************************************************************************
pub const PWM_2_DBCTL_ENABLE = usize(0x00000001);  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBRISE register.
//
//*****************************************************************************
pub const PWM_2_DBRISE_RISEDELAY_M = usize(0x00000FFF);  // Dead-Band Rise Delay
pub const PWM_2_DBRISE_RISEDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBFALL register.
//
//*****************************************************************************
pub const PWM_2_DBFALL_FALLDELAY_M = usize(0x00000FFF);  // Dead-Band Fall Delay
pub const PWM_2_DBFALL_FALLDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM_2_FLTSRC0_FAULT3 = usize(0x00000008);  // Fault3 Input
pub const PWM_2_FLTSRC0_FAULT2 = usize(0x00000004);  // Fault2 Input
pub const PWM_2_FLTSRC0_FAULT1 = usize(0x00000002);  // Fault1 Input
pub const PWM_2_FLTSRC0_FAULT0 = usize(0x00000001);  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM_2_FLTSRC1_DCMP7 = usize(0x00000080);  // Digital Comparator 7
pub const PWM_2_FLTSRC1_DCMP6 = usize(0x00000040);  // Digital Comparator 6
pub const PWM_2_FLTSRC1_DCMP5 = usize(0x00000020);  // Digital Comparator 5
pub const PWM_2_FLTSRC1_DCMP4 = usize(0x00000010);  // Digital Comparator 4
pub const PWM_2_FLTSRC1_DCMP3 = usize(0x00000008);  // Digital Comparator 3
pub const PWM_2_FLTSRC1_DCMP2 = usize(0x00000004);  // Digital Comparator 2
pub const PWM_2_FLTSRC1_DCMP1 = usize(0x00000002);  // Digital Comparator 1
pub const PWM_2_FLTSRC1_DCMP0 = usize(0x00000001);  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM_2_MINFLTPER_MFP_M = usize(0x0000FFFF);  // Minimum Fault Period
pub const PWM_2_MINFLTPER_MFP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CTL register.
//
//*****************************************************************************
pub const PWM_3_CTL_LATCH = usize(0x00040000);  // Latch Fault Input
pub const PWM_3_CTL_MINFLTPER = usize(0x00020000);  // Minimum Fault Period
pub const PWM_3_CTL_FLTSRC = usize(0x00010000);  // Fault Condition Source
pub const PWM_3_CTL_DBFALLUPD_M = usize(0x0000C000);  // PWMnDBFALL Update Mode
pub const PWM_3_CTL_DBFALLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_3_CTL_DBFALLUPD_LS = usize(0x00008000);  // Locally Synchronized
pub const PWM_3_CTL_DBFALLUPD_GS = usize(0x0000C000);  // Globally Synchronized
pub const PWM_3_CTL_DBRISEUPD_M = usize(0x00003000);  // PWMnDBRISE Update Mode
pub const PWM_3_CTL_DBRISEUPD_I = usize(0x00000000);  // Immediate
pub const PWM_3_CTL_DBRISEUPD_LS = usize(0x00002000);  // Locally Synchronized
pub const PWM_3_CTL_DBRISEUPD_GS = usize(0x00003000);  // Globally Synchronized
pub const PWM_3_CTL_DBCTLUPD_M = usize(0x00000C00);  // PWMnDBCTL Update Mode
pub const PWM_3_CTL_DBCTLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_3_CTL_DBCTLUPD_LS = usize(0x00000800);  // Locally Synchronized
pub const PWM_3_CTL_DBCTLUPD_GS = usize(0x00000C00);  // Globally Synchronized
pub const PWM_3_CTL_GENBUPD_M = usize(0x00000300);  // PWMnGENB Update Mode
pub const PWM_3_CTL_GENBUPD_I = usize(0x00000000);  // Immediate
pub const PWM_3_CTL_GENBUPD_LS = usize(0x00000200);  // Locally Synchronized
pub const PWM_3_CTL_GENBUPD_GS = usize(0x00000300);  // Globally Synchronized
pub const PWM_3_CTL_GENAUPD_M = usize(0x000000C0);  // PWMnGENA Update Mode
pub const PWM_3_CTL_GENAUPD_I = usize(0x00000000);  // Immediate
pub const PWM_3_CTL_GENAUPD_LS = usize(0x00000080);  // Locally Synchronized
pub const PWM_3_CTL_GENAUPD_GS = usize(0x000000C0);  // Globally Synchronized
pub const PWM_3_CTL_CMPBUPD = usize(0x00000020);  // Comparator B Update Mode
pub const PWM_3_CTL_CMPAUPD = usize(0x00000010);  // Comparator A Update Mode
pub const PWM_3_CTL_LOADUPD = usize(0x00000008);  // Load Register Update Mode
pub const PWM_3_CTL_DEBUG = usize(0x00000004);  // Debug Mode
pub const PWM_3_CTL_MODE = usize(0x00000002);  // Counter Mode
pub const PWM_3_CTL_ENABLE = usize(0x00000001);  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_INTEN register.
//
//*****************************************************************************
pub const PWM_3_INTEN_TRCMPBD = usize(0x00002000);  // Trigger for Counter=PWMnCMPB
                                            // Down
pub const PWM_3_INTEN_TRCMPBU = usize(0x00001000);  // Trigger for Counter=PWMnCMPB Up
pub const PWM_3_INTEN_TRCMPAD = usize(0x00000800);  // Trigger for Counter=PWMnCMPA
                                            // Down
pub const PWM_3_INTEN_TRCMPAU = usize(0x00000400);  // Trigger for Counter=PWMnCMPA Up
pub const PWM_3_INTEN_TRCNTLOAD = usize(0x00000200);  // Trigger for Counter=PWMnLOAD
pub const PWM_3_INTEN_TRCNTZERO = usize(0x00000100);  // Trigger for Counter=0
pub const PWM_3_INTEN_INTCMPBD = usize(0x00000020);  // Interrupt for Counter=PWMnCMPB
                                            // Down
pub const PWM_3_INTEN_INTCMPBU = usize(0x00000010);  // Interrupt for Counter=PWMnCMPB
                                            // Up
pub const PWM_3_INTEN_INTCMPAD = usize(0x00000008);  // Interrupt for Counter=PWMnCMPA
                                            // Down
pub const PWM_3_INTEN_INTCMPAU = usize(0x00000004);  // Interrupt for Counter=PWMnCMPA
                                            // Up
pub const PWM_3_INTEN_INTCNTLOAD = usize(0x00000002);  // Interrupt for Counter=PWMnLOAD
pub const PWM_3_INTEN_INTCNTZERO = usize(0x00000001);  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_RIS register.
//
//*****************************************************************************
pub const PWM_3_RIS_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
                                            // Status
pub const PWM_3_RIS_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt Status
pub const PWM_3_RIS_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
                                            // Status
pub const PWM_3_RIS_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt Status
pub const PWM_3_RIS_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt Status
pub const PWM_3_RIS_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_ISC register.
//
//*****************************************************************************
pub const PWM_3_ISC_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
pub const PWM_3_ISC_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt
pub const PWM_3_ISC_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
pub const PWM_3_ISC_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt
pub const PWM_3_ISC_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt
pub const PWM_3_ISC_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_LOAD register.
//
//*****************************************************************************
pub const PWM_3_LOAD_LOAD_M = usize(0x0000FFFF);  // Counter Load Value
pub const PWM_3_LOAD_LOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_COUNT register.
//
//*****************************************************************************
pub const PWM_3_COUNT_COUNT_M = usize(0x0000FFFF);  // Counter Value
pub const PWM_3_COUNT_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CMPA register.
//
//*****************************************************************************
pub const PWM_3_CMPA_COMPA_M = usize(0x0000FFFF);  // Comparator A Value
pub const PWM_3_CMPA_COMPA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CMPB register.
//
//*****************************************************************************
pub const PWM_3_CMPB_COMPB_M = usize(0x0000FFFF);  // Comparator B Value
pub const PWM_3_CMPB_COMPB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_GENA register.
//
//*****************************************************************************
pub const PWM_3_GENA_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_3_GENA_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmA
pub const PWM_3_GENA_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmA Low
pub const PWM_3_GENA_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmA High
pub const PWM_3_GENA_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_3_GENA_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmA
pub const PWM_3_GENA_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmA Low
pub const PWM_3_GENA_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmA High
pub const PWM_3_GENA_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_3_GENA_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmA
pub const PWM_3_GENA_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmA Low
pub const PWM_3_GENA_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmA High
pub const PWM_3_GENA_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_3_GENA_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmA
pub const PWM_3_GENA_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmA Low
pub const PWM_3_GENA_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmA High
pub const PWM_3_GENA_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_3_GENA_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTLOAD_INV = usize(0x00000004);  // Invert pwmA
pub const PWM_3_GENA_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmA Low
pub const PWM_3_GENA_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmA High
pub const PWM_3_GENA_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_3_GENA_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTZERO_INV = usize(0x00000001);  // Invert pwmA
pub const PWM_3_GENA_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmA Low
pub const PWM_3_GENA_ACTZERO_ONE = usize(0x00000003);  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_GENB register.
//
//*****************************************************************************
pub const PWM_3_GENB_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_3_GENB_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmB
pub const PWM_3_GENB_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmB Low
pub const PWM_3_GENB_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmB High
pub const PWM_3_GENB_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_3_GENB_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmB
pub const PWM_3_GENB_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmB Low
pub const PWM_3_GENB_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmB High
pub const PWM_3_GENB_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_3_GENB_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmB
pub const PWM_3_GENB_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmB Low
pub const PWM_3_GENB_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmB High
pub const PWM_3_GENB_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_3_GENB_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmB
pub const PWM_3_GENB_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmB Low
pub const PWM_3_GENB_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmB High
pub const PWM_3_GENB_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_3_GENB_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTLOAD_INV = usize(0x00000004);  // Invert pwmB
pub const PWM_3_GENB_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmB Low
pub const PWM_3_GENB_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmB High
pub const PWM_3_GENB_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_3_GENB_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTZERO_INV = usize(0x00000001);  // Invert pwmB
pub const PWM_3_GENB_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmB Low
pub const PWM_3_GENB_ACTZERO_ONE = usize(0x00000003);  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBCTL register.
//
//*****************************************************************************
pub const PWM_3_DBCTL_ENABLE = usize(0x00000001);  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBRISE register.
//
//*****************************************************************************
pub const PWM_3_DBRISE_RISEDELAY_M = usize(0x00000FFF);  // Dead-Band Rise Delay
pub const PWM_3_DBRISE_RISEDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBFALL register.
//
//*****************************************************************************
pub const PWM_3_DBFALL_FALLDELAY_M = usize(0x00000FFF);  // Dead-Band Fall Delay
pub const PWM_3_DBFALL_FALLDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM_3_FLTSRC0_FAULT3 = usize(0x00000008);  // Fault3 Input
pub const PWM_3_FLTSRC0_FAULT2 = usize(0x00000004);  // Fault2 Input
pub const PWM_3_FLTSRC0_FAULT1 = usize(0x00000002);  // Fault1 Input
pub const PWM_3_FLTSRC0_FAULT0 = usize(0x00000001);  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM_3_FLTSRC1_DCMP7 = usize(0x00000080);  // Digital Comparator 7
pub const PWM_3_FLTSRC1_DCMP6 = usize(0x00000040);  // Digital Comparator 6
pub const PWM_3_FLTSRC1_DCMP5 = usize(0x00000020);  // Digital Comparator 5
pub const PWM_3_FLTSRC1_DCMP4 = usize(0x00000010);  // Digital Comparator 4
pub const PWM_3_FLTSRC1_DCMP3 = usize(0x00000008);  // Digital Comparator 3
pub const PWM_3_FLTSRC1_DCMP2 = usize(0x00000004);  // Digital Comparator 2
pub const PWM_3_FLTSRC1_DCMP1 = usize(0x00000002);  // Digital Comparator 1
pub const PWM_3_FLTSRC1_DCMP0 = usize(0x00000001);  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM_3_MINFLTPER_MFP_M = usize(0x0000FFFF);  // Minimum Fault Period
pub const PWM_3_MINFLTPER_MFP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSEN register.
//
//*****************************************************************************
pub const PWM_0_FLTSEN_FAULT3 = usize(0x00000008);  // Fault3 Sense
pub const PWM_0_FLTSEN_FAULT2 = usize(0x00000004);  // Fault2 Sense
pub const PWM_0_FLTSEN_FAULT1 = usize(0x00000002);  // Fault1 Sense
pub const PWM_0_FLTSEN_FAULT0 = usize(0x00000001);  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM_0_FLTSTAT0_FAULT3 = usize(0x00000008);  // Fault Input 3
pub const PWM_0_FLTSTAT0_FAULT2 = usize(0x00000004);  // Fault Input 2
pub const PWM_0_FLTSTAT0_FAULT1 = usize(0x00000002);  // Fault Input 1
pub const PWM_0_FLTSTAT0_FAULT0 = usize(0x00000001);  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM_0_FLTSTAT1_DCMP7 = usize(0x00000080);  // Digital Comparator 7 Trigger
pub const PWM_0_FLTSTAT1_DCMP6 = usize(0x00000040);  // Digital Comparator 6 Trigger
pub const PWM_0_FLTSTAT1_DCMP5 = usize(0x00000020);  // Digital Comparator 5 Trigger
pub const PWM_0_FLTSTAT1_DCMP4 = usize(0x00000010);  // Digital Comparator 4 Trigger
pub const PWM_0_FLTSTAT1_DCMP3 = usize(0x00000008);  // Digital Comparator 3 Trigger
pub const PWM_0_FLTSTAT1_DCMP2 = usize(0x00000004);  // Digital Comparator 2 Trigger
pub const PWM_0_FLTSTAT1_DCMP1 = usize(0x00000002);  // Digital Comparator 1 Trigger
pub const PWM_0_FLTSTAT1_DCMP0 = usize(0x00000001);  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSEN register.
//
//*****************************************************************************
pub const PWM_1_FLTSEN_FAULT3 = usize(0x00000008);  // Fault3 Sense
pub const PWM_1_FLTSEN_FAULT2 = usize(0x00000004);  // Fault2 Sense
pub const PWM_1_FLTSEN_FAULT1 = usize(0x00000002);  // Fault1 Sense
pub const PWM_1_FLTSEN_FAULT0 = usize(0x00000001);  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM_1_FLTSTAT0_FAULT3 = usize(0x00000008);  // Fault Input 3
pub const PWM_1_FLTSTAT0_FAULT2 = usize(0x00000004);  // Fault Input 2
pub const PWM_1_FLTSTAT0_FAULT1 = usize(0x00000002);  // Fault Input 1
pub const PWM_1_FLTSTAT0_FAULT0 = usize(0x00000001);  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM_1_FLTSTAT1_DCMP7 = usize(0x00000080);  // Digital Comparator 7 Trigger
pub const PWM_1_FLTSTAT1_DCMP6 = usize(0x00000040);  // Digital Comparator 6 Trigger
pub const PWM_1_FLTSTAT1_DCMP5 = usize(0x00000020);  // Digital Comparator 5 Trigger
pub const PWM_1_FLTSTAT1_DCMP4 = usize(0x00000010);  // Digital Comparator 4 Trigger
pub const PWM_1_FLTSTAT1_DCMP3 = usize(0x00000008);  // Digital Comparator 3 Trigger
pub const PWM_1_FLTSTAT1_DCMP2 = usize(0x00000004);  // Digital Comparator 2 Trigger
pub const PWM_1_FLTSTAT1_DCMP1 = usize(0x00000002);  // Digital Comparator 1 Trigger
pub const PWM_1_FLTSTAT1_DCMP0 = usize(0x00000001);  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSEN register.
//
//*****************************************************************************
pub const PWM_2_FLTSEN_FAULT3 = usize(0x00000008);  // Fault3 Sense
pub const PWM_2_FLTSEN_FAULT2 = usize(0x00000004);  // Fault2 Sense
pub const PWM_2_FLTSEN_FAULT1 = usize(0x00000002);  // Fault1 Sense
pub const PWM_2_FLTSEN_FAULT0 = usize(0x00000001);  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM_2_FLTSTAT0_FAULT3 = usize(0x00000008);  // Fault Input 3
pub const PWM_2_FLTSTAT0_FAULT2 = usize(0x00000004);  // Fault Input 2
pub const PWM_2_FLTSTAT0_FAULT1 = usize(0x00000002);  // Fault Input 1
pub const PWM_2_FLTSTAT0_FAULT0 = usize(0x00000001);  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM_2_FLTSTAT1_DCMP7 = usize(0x00000080);  // Digital Comparator 7 Trigger
pub const PWM_2_FLTSTAT1_DCMP6 = usize(0x00000040);  // Digital Comparator 6 Trigger
pub const PWM_2_FLTSTAT1_DCMP5 = usize(0x00000020);  // Digital Comparator 5 Trigger
pub const PWM_2_FLTSTAT1_DCMP4 = usize(0x00000010);  // Digital Comparator 4 Trigger
pub const PWM_2_FLTSTAT1_DCMP3 = usize(0x00000008);  // Digital Comparator 3 Trigger
pub const PWM_2_FLTSTAT1_DCMP2 = usize(0x00000004);  // Digital Comparator 2 Trigger
pub const PWM_2_FLTSTAT1_DCMP1 = usize(0x00000002);  // Digital Comparator 1 Trigger
pub const PWM_2_FLTSTAT1_DCMP0 = usize(0x00000001);  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSEN register.
//
//*****************************************************************************
pub const PWM_3_FLTSEN_FAULT3 = usize(0x00000008);  // Fault3 Sense
pub const PWM_3_FLTSEN_FAULT2 = usize(0x00000004);  // Fault2 Sense
pub const PWM_3_FLTSEN_FAULT1 = usize(0x00000002);  // Fault1 Sense
pub const PWM_3_FLTSEN_FAULT0 = usize(0x00000001);  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM_3_FLTSTAT0_FAULT3 = usize(0x00000008);  // Fault Input 3
pub const PWM_3_FLTSTAT0_FAULT2 = usize(0x00000004);  // Fault Input 2
pub const PWM_3_FLTSTAT0_FAULT1 = usize(0x00000002);  // Fault Input 1
pub const PWM_3_FLTSTAT0_FAULT0 = usize(0x00000001);  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM_3_FLTSTAT1_DCMP7 = usize(0x00000080);  // Digital Comparator 7 Trigger
pub const PWM_3_FLTSTAT1_DCMP6 = usize(0x00000040);  // Digital Comparator 6 Trigger
pub const PWM_3_FLTSTAT1_DCMP5 = usize(0x00000020);  // Digital Comparator 5 Trigger
pub const PWM_3_FLTSTAT1_DCMP4 = usize(0x00000010);  // Digital Comparator 4 Trigger
pub const PWM_3_FLTSTAT1_DCMP3 = usize(0x00000008);  // Digital Comparator 3 Trigger
pub const PWM_3_FLTSTAT1_DCMP2 = usize(0x00000004);  // Digital Comparator 2 Trigger
pub const PWM_3_FLTSTAT1_DCMP1 = usize(0x00000002);  // Digital Comparator 1 Trigger
pub const PWM_3_FLTSTAT1_DCMP0 = usize(0x00000001);  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_PP register.
//
//*****************************************************************************
pub const PWM_PP_GCNT_M = usize(0x0000000F);  // Generators
pub const PWM_PP_FCNT_M = usize(0x000000F0);  // Fault Inputs (per PWM unit)
pub const PWM_PP_ESYNC = usize(0x00000100);  // Extended Synchronization
pub const PWM_PP_EFAULT = usize(0x00000200);  // Extended Fault
pub const PWM_PP_ONE = usize(0x00000400);  // One-Shot Mode
pub const PWM_PP_GCNT_S = usize(0);
pub const PWM_PP_FCNT_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_CC register.
//
//*****************************************************************************
pub const PWM_CC_USEPWM = usize(0x00000100);  // Use PWM Clock Divisor
pub const PWM_CC_PWMDIV_M = usize(0x00000007);  // PWM Clock Divider
pub const PWM_CC_PWMDIV_2 = usize(0x00000000);  // /2
pub const PWM_CC_PWMDIV_4 = usize(0x00000001);  // /4
pub const PWM_CC_PWMDIV_8 = usize(0x00000002);  // /8
pub const PWM_CC_PWMDIV_16 = usize(0x00000003);  // /16
pub const PWM_CC_PWMDIV_32 = usize(0x00000004);  // /32
pub const PWM_CC_PWMDIV_64 = usize(0x00000005);  // /64

//*****************************************************************************
//
// The following are defines for the PWM Generator standard offsets.
//
//*****************************************************************************
pub const PWM_O_X_CTL = usize(0x00000000);  // Gen Control Reg
pub const PWM_O_X_INTEN = usize(0x00000004);  // Gen Int/Trig Enable Reg
pub const PWM_O_X_RIS = usize(0x00000008);  // Gen Raw Int Status Reg
pub const PWM_O_X_ISC = usize(0x0000000C);  // Gen Int Status Reg
pub const PWM_O_X_LOAD = usize(0x00000010);  // Gen Load Reg
pub const PWM_O_X_COUNT = usize(0x00000014);  // Gen Counter Reg
pub const PWM_O_X_CMPA = usize(0x00000018);  // Gen Compare A Reg
pub const PWM_O_X_CMPB = usize(0x0000001C);  // Gen Compare B Reg
pub const PWM_O_X_GENA = usize(0x00000020);  // Gen Generator A Ctrl Reg
pub const PWM_O_X_GENB = usize(0x00000024);  // Gen Generator B Ctrl Reg
pub const PWM_O_X_DBCTL = usize(0x00000028);  // Gen Dead Band Ctrl Reg
pub const PWM_O_X_DBRISE = usize(0x0000002C);  // Gen DB Rising Edge Delay Reg
pub const PWM_O_X_DBFALL = usize(0x00000030);  // Gen DB Falling Edge Delay Reg
pub const PWM_O_X_FLTSRC0 = usize(0x00000034);  // Fault pin, comparator condition
pub const PWM_O_X_FLTSRC1 = usize(0x00000038);  // Digital comparator condition
pub const PWM_O_X_MINFLTPER = usize(0x0000003C);  // Fault minimum period extension
pub const PWM_GEN_0_OFFSET = usize(0x00000040);  // PWM0 base
pub const PWM_GEN_1_OFFSET = usize(0x00000080);  // PWM1 base
pub const PWM_GEN_2_OFFSET = usize(0x000000C0);  // PWM2 base
pub const PWM_GEN_3_OFFSET = usize(0x00000100);  // PWM3 base

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_CTL register.
//
//*****************************************************************************
pub const PWM_X_CTL_LATCH = usize(0x00040000);  // Latch Fault Input
pub const PWM_X_CTL_MINFLTPER = usize(0x00020000);  // Minimum Fault Period
pub const PWM_X_CTL_FLTSRC = usize(0x00010000);  // Fault Condition Source
pub const PWM_X_CTL_DBFALLUPD_M = usize(0x0000C000);  // PWMnDBFALL Update Mode
pub const PWM_X_CTL_DBFALLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_X_CTL_DBFALLUPD_LS = usize(0x00008000);  // Locally Synchronized
pub const PWM_X_CTL_DBFALLUPD_GS = usize(0x0000C000);  // Globally Synchronized
pub const PWM_X_CTL_DBRISEUPD_M = usize(0x00003000);  // PWMnDBRISE Update Mode
pub const PWM_X_CTL_DBRISEUPD_I = usize(0x00000000);  // Immediate
pub const PWM_X_CTL_DBRISEUPD_LS = usize(0x00002000);  // Locally Synchronized
pub const PWM_X_CTL_DBRISEUPD_GS = usize(0x00003000);  // Globally Synchronized
pub const PWM_X_CTL_DBCTLUPD_M = usize(0x00000C00);  // PWMnDBCTL Update Mode
pub const PWM_X_CTL_DBCTLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_X_CTL_DBCTLUPD_LS = usize(0x00000800);  // Locally Synchronized
pub const PWM_X_CTL_DBCTLUPD_GS = usize(0x00000C00);  // Globally Synchronized
pub const PWM_X_CTL_GENBUPD_M = usize(0x00000300);  // PWMnGENB Update Mode
pub const PWM_X_CTL_GENBUPD_I = usize(0x00000000);  // Immediate
pub const PWM_X_CTL_GENBUPD_LS = usize(0x00000200);  // Locally Synchronized
pub const PWM_X_CTL_GENBUPD_GS = usize(0x00000300);  // Globally Synchronized
pub const PWM_X_CTL_GENAUPD_M = usize(0x000000C0);  // PWMnGENA Update Mode
pub const PWM_X_CTL_GENAUPD_I = usize(0x00000000);  // Immediate
pub const PWM_X_CTL_GENAUPD_LS = usize(0x00000080);  // Locally Synchronized
pub const PWM_X_CTL_GENAUPD_GS = usize(0x000000C0);  // Globally Synchronized
pub const PWM_X_CTL_CMPBUPD = usize(0x00000020);  // Comparator B Update Mode
pub const PWM_X_CTL_CMPAUPD = usize(0x00000010);  // Comparator A Update Mode
pub const PWM_X_CTL_LOADUPD = usize(0x00000008);  // Load Register Update Mode
pub const PWM_X_CTL_DEBUG = usize(0x00000004);  // Debug Mode
pub const PWM_X_CTL_MODE = usize(0x00000002);  // Counter Mode
pub const PWM_X_CTL_ENABLE = usize(0x00000001);  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_INTEN register.
//
//*****************************************************************************
pub const PWM_X_INTEN_TRCMPBD = usize(0x00002000);  // Trigger for Counter=PWMnCMPB
                                            // Down
pub const PWM_X_INTEN_TRCMPBU = usize(0x00001000);  // Trigger for Counter=PWMnCMPB Up
pub const PWM_X_INTEN_TRCMPAD = usize(0x00000800);  // Trigger for Counter=PWMnCMPA
                                            // Down
pub const PWM_X_INTEN_TRCMPAU = usize(0x00000400);  // Trigger for Counter=PWMnCMPA Up
pub const PWM_X_INTEN_TRCNTLOAD = usize(0x00000200);  // Trigger for Counter=PWMnLOAD
pub const PWM_X_INTEN_TRCNTZERO = usize(0x00000100);  // Trigger for Counter=0
pub const PWM_X_INTEN_INTCMPBD = usize(0x00000020);  // Interrupt for Counter=PWMnCMPB
                                            // Down
pub const PWM_X_INTEN_INTCMPBU = usize(0x00000010);  // Interrupt for Counter=PWMnCMPB
                                            // Up
pub const PWM_X_INTEN_INTCMPAD = usize(0x00000008);  // Interrupt for Counter=PWMnCMPA
                                            // Down
pub const PWM_X_INTEN_INTCMPAU = usize(0x00000004);  // Interrupt for Counter=PWMnCMPA
                                            // Up
pub const PWM_X_INTEN_INTCNTLOAD = usize(0x00000002);  // Interrupt for Counter=PWMnLOAD
pub const PWM_X_INTEN_INTCNTZERO = usize(0x00000001);  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_RIS register.
//
//*****************************************************************************
pub const PWM_X_RIS_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
                                            // Status
pub const PWM_X_RIS_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt Status
pub const PWM_X_RIS_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
                                            // Status
pub const PWM_X_RIS_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt Status
pub const PWM_X_RIS_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt Status
pub const PWM_X_RIS_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_ISC register.
//
//*****************************************************************************
pub const PWM_X_ISC_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
pub const PWM_X_ISC_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt
pub const PWM_X_ISC_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
pub const PWM_X_ISC_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt
pub const PWM_X_ISC_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt
pub const PWM_X_ISC_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_LOAD register.
//
//*****************************************************************************
pub const PWM_X_LOAD_M = usize(0x0000FFFF);  // Counter Load Value
pub const PWM_X_LOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_COUNT register.
//
//*****************************************************************************
pub const PWM_X_COUNT_M = usize(0x0000FFFF);  // Counter Value
pub const PWM_X_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_CMPA register.
//
//*****************************************************************************
pub const PWM_X_CMPA_M = usize(0x0000FFFF);  // Comparator A Value
pub const PWM_X_CMPA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_CMPB register.
//
//*****************************************************************************
pub const PWM_X_CMPB_M = usize(0x0000FFFF);  // Comparator B Value
pub const PWM_X_CMPB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_GENA register.
//
//*****************************************************************************
pub const PWM_X_GENA_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_X_GENA_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENA_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmA
pub const PWM_X_GENA_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmA Low
pub const PWM_X_GENA_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmA High
pub const PWM_X_GENA_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_X_GENA_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENA_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmA
pub const PWM_X_GENA_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmA Low
pub const PWM_X_GENA_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmA High
pub const PWM_X_GENA_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_X_GENA_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENA_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmA
pub const PWM_X_GENA_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmA Low
pub const PWM_X_GENA_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmA High
pub const PWM_X_GENA_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_X_GENA_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENA_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmA
pub const PWM_X_GENA_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmA Low
pub const PWM_X_GENA_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmA High
pub const PWM_X_GENA_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_X_GENA_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENA_ACTLOAD_INV = usize(0x00000004);  // Invert pwmA
pub const PWM_X_GENA_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmA Low
pub const PWM_X_GENA_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmA High
pub const PWM_X_GENA_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_X_GENA_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENA_ACTZERO_INV = usize(0x00000001);  // Invert pwmA
pub const PWM_X_GENA_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmA Low
pub const PWM_X_GENA_ACTZERO_ONE = usize(0x00000003);  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_GENB register.
//
//*****************************************************************************
pub const PWM_X_GENB_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_X_GENB_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENB_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmB
pub const PWM_X_GENB_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmB Low
pub const PWM_X_GENB_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmB High
pub const PWM_X_GENB_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_X_GENB_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENB_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmB
pub const PWM_X_GENB_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmB Low
pub const PWM_X_GENB_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmB High
pub const PWM_X_GENB_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_X_GENB_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENB_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmB
pub const PWM_X_GENB_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmB Low
pub const PWM_X_GENB_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmB High
pub const PWM_X_GENB_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_X_GENB_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENB_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmB
pub const PWM_X_GENB_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmB Low
pub const PWM_X_GENB_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmB High
pub const PWM_X_GENB_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_X_GENB_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENB_ACTLOAD_INV = usize(0x00000004);  // Invert pwmB
pub const PWM_X_GENB_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmB Low
pub const PWM_X_GENB_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmB High
pub const PWM_X_GENB_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_X_GENB_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_X_GENB_ACTZERO_INV = usize(0x00000001);  // Invert pwmB
pub const PWM_X_GENB_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmB Low
pub const PWM_X_GENB_ACTZERO_ONE = usize(0x00000003);  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_DBCTL register.
//
//*****************************************************************************
pub const PWM_X_DBCTL_ENABLE = usize(0x00000001);  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_DBRISE register.
//
//*****************************************************************************
pub const PWM_X_DBRISE_DELAY_M = usize(0x00000FFF);  // Dead-Band Rise Delay
pub const PWM_X_DBRISE_DELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_DBFALL register.
//
//*****************************************************************************
pub const PWM_X_DBFALL_DELAY_M = usize(0x00000FFF);  // Dead-Band Fall Delay
pub const PWM_X_DBFALL_DELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM_X_FLTSRC0_FAULT3 = usize(0x00000008);  // Fault3 Input
pub const PWM_X_FLTSRC0_FAULT2 = usize(0x00000004);  // Fault2 Input
pub const PWM_X_FLTSRC0_FAULT1 = usize(0x00000002);  // Fault1 Input
pub const PWM_X_FLTSRC0_FAULT0 = usize(0x00000001);  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM_X_FLTSRC1_DCMP7 = usize(0x00000080);  // Digital Comparator 7
pub const PWM_X_FLTSRC1_DCMP6 = usize(0x00000040);  // Digital Comparator 6
pub const PWM_X_FLTSRC1_DCMP5 = usize(0x00000020);  // Digital Comparator 5
pub const PWM_X_FLTSRC1_DCMP4 = usize(0x00000010);  // Digital Comparator 4
pub const PWM_X_FLTSRC1_DCMP3 = usize(0x00000008);  // Digital Comparator 3
pub const PWM_X_FLTSRC1_DCMP2 = usize(0x00000004);  // Digital Comparator 2
pub const PWM_X_FLTSRC1_DCMP1 = usize(0x00000002);  // Digital Comparator 1
pub const PWM_X_FLTSRC1_DCMP0 = usize(0x00000001);  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM_X_MINFLTPER_M = usize(0x0000FFFF);  // Minimum Fault Period
pub const PWM_X_MINFLTPER_S = usize(0);

//*****************************************************************************
//
// The following are defines for the PWM Generator extended offsets.
//
//*****************************************************************************
pub const PWM_O_X_FLTSEN = usize(0x00000000);  // Fault logic sense
pub const PWM_O_X_FLTSTAT0 = usize(0x00000004);  // Pin and comparator status
pub const PWM_O_X_FLTSTAT1 = usize(0x00000008);  // Digital comparator status
pub const PWM_EXT_0_OFFSET = usize(0x00000800);  // PWM0 extended base
pub const PWM_EXT_1_OFFSET = usize(0x00000880);  // PWM1 extended base
pub const PWM_EXT_2_OFFSET = usize(0x00000900);  // PWM2 extended base
pub const PWM_EXT_3_OFFSET = usize(0x00000980);  // PWM3 extended base

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_FLTSEN register.
//
//*****************************************************************************
pub const PWM_X_FLTSEN_FAULT3 = usize(0x00000008);  // Fault3 Sense
pub const PWM_X_FLTSEN_FAULT2 = usize(0x00000004);  // Fault2 Sense
pub const PWM_X_FLTSEN_FAULT1 = usize(0x00000002);  // Fault1 Sense
pub const PWM_X_FLTSEN_FAULT0 = usize(0x00000001);  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM_X_FLTSTAT0_FAULT3 = usize(0x00000008);  // Fault Input 3
pub const PWM_X_FLTSTAT0_FAULT2 = usize(0x00000004);  // Fault Input 2
pub const PWM_X_FLTSTAT0_FAULT1 = usize(0x00000002);  // Fault Input 1
pub const PWM_X_FLTSTAT0_FAULT0 = usize(0x00000001);  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_X_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM_X_FLTSTAT1_DCMP7 = usize(0x00000080);  // Digital Comparator 7 Trigger
pub const PWM_X_FLTSTAT1_DCMP6 = usize(0x00000040);  // Digital Comparator 6 Trigger
pub const PWM_X_FLTSTAT1_DCMP5 = usize(0x00000020);  // Digital Comparator 5 Trigger
pub const PWM_X_FLTSTAT1_DCMP4 = usize(0x00000010);  // Digital Comparator 4 Trigger
pub const PWM_X_FLTSTAT1_DCMP3 = usize(0x00000008);  // Digital Comparator 3 Trigger
pub const PWM_X_FLTSTAT1_DCMP2 = usize(0x00000004);  // Digital Comparator 2 Trigger
pub const PWM_X_FLTSTAT1_DCMP1 = usize(0x00000002);  // Digital Comparator 1 Trigger
pub const PWM_X_FLTSTAT1_DCMP0 = usize(0x00000001);  // Digital Comparator 0 Trigger

