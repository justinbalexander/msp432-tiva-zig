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
pub const offsetOf = struct {
    pub const CTL = usize(0x00000000); // PWM Master Control
    pub const SYNC = usize(0x00000004); // PWM Time Base Sync
    pub const ENABLE = usize(0x00000008); // PWM Output Enable
    pub const INVERT = usize(0x0000000C); // PWM Output Inversion
    pub const FAULT = usize(0x00000010); // PWM Output Fault
    pub const INTEN = usize(0x00000014); // PWM Interrupt Enable
    pub const RIS = usize(0x00000018); // PWM Raw Interrupt Status
    pub const ISC = usize(0x0000001C); // PWM Interrupt Status and Clear
    pub const STATUS = usize(0x00000020); // PWM Status
    pub const FAULTVAL = usize(0x00000024); // PWM Fault Condition Value
    pub const ENUPD = usize(0x00000028); // PWM Enable Update
    pub const PWM0_CTL = usize(0x00000040); // PWM0 Control
    pub const PWM0_INTEN = usize(0x00000044); // PWM0 Interrupt and Trigger
    // Enable
    pub const PWM0_RIS = usize(0x00000048); // PWM0 Raw Interrupt Status
    pub const PWM0_ISC = usize(0x0000004C); // PWM0 Interrupt Status and Clear
    pub const PWM0_LOAD = usize(0x00000050); // PWM0 Load
    pub const PWM0_COUNT = usize(0x00000054); // PWM0 Counter
    pub const PWM0_CMPA = usize(0x00000058); // PWM0 Compare A
    pub const PWM0_CMPB = usize(0x0000005C); // PWM0 Compare B
    pub const PWM0_GENA = usize(0x00000060); // PWM0 Generator A Control
    pub const PWM0_GENB = usize(0x00000064); // PWM0 Generator B Control
    pub const PWM0_DBCTL = usize(0x00000068); // PWM0 Dead-Band Control
    pub const PWM0_DBRISE = usize(0x0000006C); // PWM0 Dead-Band Rising-Edge Delay
    pub const PWM0_DBFALL = usize(0x00000070); // PWM0 Dead-Band
    // Falling-Edge-Delay
    pub const PWM0_FLTSRC0 = usize(0x00000074); // PWM0 Fault Source 0
    pub const PWM0_FLTSRC1 = usize(0x00000078); // PWM0 Fault Source 1
    pub const PWM0_MINFLTPER = usize(0x0000007C); // PWM0 Minimum Fault Period
    pub const PWM1_CTL = usize(0x00000080); // PWM1 Control
    pub const PWM1_INTEN = usize(0x00000084); // PWM1 Interrupt and Trigger
    // Enable
    pub const PWM1_RIS = usize(0x00000088); // PWM1 Raw Interrupt Status
    pub const PWM1_ISC = usize(0x0000008C); // PWM1 Interrupt Status and Clear
    pub const PWM1_LOAD = usize(0x00000090); // PWM1 Load
    pub const PWM1_COUNT = usize(0x00000094); // PWM1 Counter
    pub const PWM1_CMPA = usize(0x00000098); // PWM1 Compare A
    pub const PWM1_CMPB = usize(0x0000009C); // PWM1 Compare B
    pub const PWM1_GENA = usize(0x000000A0); // PWM1 Generator A Control
    pub const PWM1_GENB = usize(0x000000A4); // PWM1 Generator B Control
    pub const PWM1_DBCTL = usize(0x000000A8); // PWM1 Dead-Band Control
    pub const PWM1_DBRISE = usize(0x000000AC); // PWM1 Dead-Band Rising-Edge Delay
    pub const PWM1_DBFALL = usize(0x000000B0); // PWM1 Dead-Band
    // Falling-Edge-Delay
    pub const PWM1_FLTSRC0 = usize(0x000000B4); // PWM1 Fault Source 0
    pub const PWM1_FLTSRC1 = usize(0x000000B8); // PWM1 Fault Source 1
    pub const PWM1_MINFLTPER = usize(0x000000BC); // PWM1 Minimum Fault Period
    pub const PWM2_CTL = usize(0x000000C0); // PWM2 Control
    pub const PWM2_INTEN = usize(0x000000C4); // PWM2 Interrupt and Trigger
    // Enable
    pub const PWM2_RIS = usize(0x000000C8); // PWM2 Raw Interrupt Status
    pub const PWM2_ISC = usize(0x000000CC); // PWM2 Interrupt Status and Clear
    pub const PWM2_LOAD = usize(0x000000D0); // PWM2 Load
    pub const PWM2_COUNT = usize(0x000000D4); // PWM2 Counter
    pub const PWM2_CMPA = usize(0x000000D8); // PWM2 Compare A
    pub const PWM2_CMPB = usize(0x000000DC); // PWM2 Compare B
    pub const PWM2_GENA = usize(0x000000E0); // PWM2 Generator A Control
    pub const PWM2_GENB = usize(0x000000E4); // PWM2 Generator B Control
    pub const PWM2_DBCTL = usize(0x000000E8); // PWM2 Dead-Band Control
    pub const PWM2_DBRISE = usize(0x000000EC); // PWM2 Dead-Band Rising-Edge Delay
    pub const PWM2_DBFALL = usize(0x000000F0); // PWM2 Dead-Band
    // Falling-Edge-Delay
    pub const PWM2_FLTSRC0 = usize(0x000000F4); // PWM2 Fault Source 0
    pub const PWM2_FLTSRC1 = usize(0x000000F8); // PWM2 Fault Source 1
    pub const PWM2_MINFLTPER = usize(0x000000FC); // PWM2 Minimum Fault Period
    pub const PWM3_CTL = usize(0x00000100); // PWM3 Control
    pub const PWM3_INTEN = usize(0x00000104); // PWM3 Interrupt and Trigger
    // Enable
    pub const PWM3_RIS = usize(0x00000108); // PWM3 Raw Interrupt Status
    pub const PWM3_ISC = usize(0x0000010C); // PWM3 Interrupt Status and Clear
    pub const PWM3_LOAD = usize(0x00000110); // PWM3 Load
    pub const PWM3_COUNT = usize(0x00000114); // PWM3 Counter
    pub const PWM3_CMPA = usize(0x00000118); // PWM3 Compare A
    pub const PWM3_CMPB = usize(0x0000011C); // PWM3 Compare B
    pub const PWM3_GENA = usize(0x00000120); // PWM3 Generator A Control
    pub const PWM3_GENB = usize(0x00000124); // PWM3 Generator B Control
    pub const PWM3_DBCTL = usize(0x00000128); // PWM3 Dead-Band Control
    pub const PWM3_DBRISE = usize(0x0000012C); // PWM3 Dead-Band Rising-Edge Delay
    pub const PWM3_DBFALL = usize(0x00000130); // PWM3 Dead-Band
    // Falling-Edge-Delay
    pub const PWM3_FLTSRC0 = usize(0x00000134); // PWM3 Fault Source 0
    pub const PWM3_FLTSRC1 = usize(0x00000138); // PWM3 Fault Source 1
    pub const PWM3_MINFLTPER = usize(0x0000013C); // PWM3 Minimum Fault Period
    pub const PWM0_FLTSEN = usize(0x00000800); // PWM0 Fault Pin Logic Sense
    pub const PWM0_FLTSTAT0 = usize(0x00000804); // PWM0 Fault Status 0
    pub const PWM0_FLTSTAT1 = usize(0x00000808); // PWM0 Fault Status 1
    pub const PWM1_FLTSEN = usize(0x00000880); // PWM1 Fault Pin Logic Sense
    pub const PWM1_FLTSTAT0 = usize(0x00000884); // PWM1 Fault Status 0
    pub const PWM1_FLTSTAT1 = usize(0x00000888); // PWM1 Fault Status 1
    pub const PWM2_FLTSEN = usize(0x00000900); // PWM2 Fault Pin Logic Sense
    pub const PWM2_FLTSTAT0 = usize(0x00000904); // PWM2 Fault Status 0
    pub const PWM2_FLTSTAT1 = usize(0x00000908); // PWM2 Fault Status 1
    pub const PWM3_FLTSEN = usize(0x00000980); // PWM3 Fault Pin Logic Sense
    pub const PWM3_FLTSTAT0 = usize(0x00000984); // PWM3 Fault Status 0
    pub const PWM3_FLTSTAT1 = usize(0x00000988); // PWM3 Fault Status 1
    pub const PP = usize(0x00000FC0); // PWM Peripheral Properties
    pub const CC = usize(0x00000FC8); // PWM Clock Configuration
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_CTL register.
//
//*****************************************************************************
pub const CTL = struct {
    pub const GLOBALSYNC3 = usize(0x00000008); // Update PWM Generator 3
    pub const GLOBALSYNC2 = usize(0x00000004); // Update PWM Generator 2
    pub const GLOBALSYNC1 = usize(0x00000002); // Update PWM Generator 1
    pub const GLOBALSYNC0 = usize(0x00000001); // Update PWM Generator 0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_SYNC register.
//
//*****************************************************************************
pub const SYNC = struct {
    pub const SYNC3 = usize(0x00000008); // Reset Generator 3 Counter
    pub const SYNC2 = usize(0x00000004); // Reset Generator 2 Counter
    pub const SYNC1 = usize(0x00000002); // Reset Generator 1 Counter
    pub const SYNC0 = usize(0x00000001); // Reset Generator 0 Counter
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENABLE register.
//
//*****************************************************************************
pub const ENABLE = struct {
    pub const PWM7EN = usize(0x00000080); // MnPWM7 Output Enable
    pub const PWM6EN = usize(0x00000040); // MnPWM6 Output Enable
    pub const PWM5EN = usize(0x00000020); // MnPWM5 Output Enable
    pub const PWM4EN = usize(0x00000010); // MnPWM4 Output Enable
    pub const PWM3EN = usize(0x00000008); // MnPWM3 Output Enable
    pub const PWM2EN = usize(0x00000004); // MnPWM2 Output Enable
    pub const PWM1EN = usize(0x00000002); // MnPWM1 Output Enable
    pub const PWM0EN = usize(0x00000001); // MnPWM0 Output Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INVERT register.
//
//*****************************************************************************
pub const INVERT = struct {
    pub const PWM7INV = usize(0x00000080); // Invert MnPWM7 Signal
    pub const PWM6INV = usize(0x00000040); // Invert MnPWM6 Signal
    pub const PWM5INV = usize(0x00000020); // Invert MnPWM5 Signal
    pub const PWM4INV = usize(0x00000010); // Invert MnPWM4 Signal
    pub const PWM3INV = usize(0x00000008); // Invert MnPWM3 Signal
    pub const PWM2INV = usize(0x00000004); // Invert MnPWM2 Signal
    pub const PWM1INV = usize(0x00000002); // Invert MnPWM1 Signal
    pub const PWM0INV = usize(0x00000001); // Invert MnPWM0 Signal
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULT register.
//
//*****************************************************************************
pub const FAULT = struct {
    pub const FAULT7 = usize(0x00000080); // MnPWM7 Fault
    pub const FAULT6 = usize(0x00000040); // MnPWM6 Fault
    pub const FAULT5 = usize(0x00000020); // MnPWM5 Fault
    pub const FAULT4 = usize(0x00000010); // MnPWM4 Fault
    pub const FAULT3 = usize(0x00000008); // MnPWM3 Fault
    pub const FAULT2 = usize(0x00000004); // MnPWM2 Fault
    pub const FAULT1 = usize(0x00000002); // MnPWM1 Fault
    pub const FAULT0 = usize(0x00000001); // MnPWM0 Fault
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INTEN register.
//
//*****************************************************************************
pub const INTEN = struct {
    pub const INTFAULT3 = usize(0x00080000); // Interrupt Fault 3
    pub const INTFAULT2 = usize(0x00040000); // Interrupt Fault 2
    pub const INTFAULT1 = usize(0x00020000); // Interrupt Fault 1
    pub const INTFAULT0 = usize(0x00010000); // Interrupt Fault 0
    pub const INTPWM3 = usize(0x00000008); // PWM3 Interrupt Enable
    pub const INTPWM2 = usize(0x00000004); // PWM2 Interrupt Enable
    pub const INTPWM1 = usize(0x00000002); // PWM1 Interrupt Enable
    pub const INTPWM0 = usize(0x00000001); // PWM0 Interrupt Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_RIS register.
//
//*****************************************************************************
pub const RIS = struct {
    pub const INTFAULT3 = usize(0x00080000); // Interrupt Fault PWM 3
    pub const INTFAULT2 = usize(0x00040000); // Interrupt Fault PWM 2
    pub const INTFAULT1 = usize(0x00020000); // Interrupt Fault PWM 1
    pub const INTFAULT0 = usize(0x00010000); // Interrupt Fault PWM 0
    pub const INTPWM3 = usize(0x00000008); // PWM3 Interrupt Asserted
    pub const INTPWM2 = usize(0x00000004); // PWM2 Interrupt Asserted
    pub const INTPWM1 = usize(0x00000002); // PWM1 Interrupt Asserted
    pub const INTPWM0 = usize(0x00000001); // PWM0 Interrupt Asserted
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ISC register.
//
//*****************************************************************************
pub const ISC = struct {
    pub const INTFAULT3 = usize(0x00080000); // FAULT3 Interrupt Asserted
    pub const INTFAULT2 = usize(0x00040000); // FAULT2 Interrupt Asserted
    pub const INTFAULT1 = usize(0x00020000); // FAULT1 Interrupt Asserted
    pub const INTFAULT0 = usize(0x00010000); // FAULT0 Interrupt Asserted
    pub const INTPWM3 = usize(0x00000008); // PWM3 Interrupt Status
    pub const INTPWM2 = usize(0x00000004); // PWM2 Interrupt Status
    pub const INTPWM1 = usize(0x00000002); // PWM1 Interrupt Status
    pub const INTPWM0 = usize(0x00000001); // PWM0 Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_STATUS register.
//
//*****************************************************************************
pub const STATUS = struct {
    pub const FAULT3 = usize(0x00000008); // Generator 3 Fault Status
    pub const FAULT2 = usize(0x00000004); // Generator 2 Fault Status
    pub const FAULT1 = usize(0x00000002); // Generator 1 Fault Status
    pub const FAULT0 = usize(0x00000001); // Generator 0 Fault Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULTVAL register.
//
//*****************************************************************************
pub const FAULTVAL = struct {
    pub const PWM7 = usize(0x00000080); // MnPWM7 Fault Value
    pub const PWM6 = usize(0x00000040); // MnPWM6 Fault Value
    pub const PWM5 = usize(0x00000020); // MnPWM5 Fault Value
    pub const PWM4 = usize(0x00000010); // MnPWM4 Fault Value
    pub const PWM3 = usize(0x00000008); // MnPWM3 Fault Value
    pub const PWM2 = usize(0x00000004); // MnPWM2 Fault Value
    pub const PWM1 = usize(0x00000002); // MnPWM1 Fault Value
    pub const PWM0 = usize(0x00000001); // MnPWM0 Fault Value
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENUPD register.
//
//*****************************************************************************
pub const ENUPD = struct {
    pub const ENUPD7_M = usize(0x0000C000); // MnPWM7 Enable Update Mode
    pub const ENUPD7_IMM = usize(0x00000000); // Immediate
    pub const ENUPD7_LSYNC = usize(0x00008000); // Locally Synchronized
    pub const ENUPD7_GSYNC = usize(0x0000C000); // Globally Synchronized
    pub const ENUPD6_M = usize(0x00003000); // MnPWM6 Enable Update Mode
    pub const ENUPD6_IMM = usize(0x00000000); // Immediate
    pub const ENUPD6_LSYNC = usize(0x00002000); // Locally Synchronized
    pub const ENUPD6_GSYNC = usize(0x00003000); // Globally Synchronized
    pub const ENUPD5_M = usize(0x00000C00); // MnPWM5 Enable Update Mode
    pub const ENUPD5_IMM = usize(0x00000000); // Immediate
    pub const ENUPD5_LSYNC = usize(0x00000800); // Locally Synchronized
    pub const ENUPD5_GSYNC = usize(0x00000C00); // Globally Synchronized
    pub const ENUPD4_M = usize(0x00000300); // MnPWM4 Enable Update Mode
    pub const ENUPD4_IMM = usize(0x00000000); // Immediate
    pub const ENUPD4_LSYNC = usize(0x00000200); // Locally Synchronized
    pub const ENUPD4_GSYNC = usize(0x00000300); // Globally Synchronized
    pub const ENUPD3_M = usize(0x000000C0); // MnPWM3 Enable Update Mode
    pub const ENUPD3_IMM = usize(0x00000000); // Immediate
    pub const ENUPD3_LSYNC = usize(0x00000080); // Locally Synchronized
    pub const ENUPD3_GSYNC = usize(0x000000C0); // Globally Synchronized
    pub const ENUPD2_M = usize(0x00000030); // MnPWM2 Enable Update Mode
    pub const ENUPD2_IMM = usize(0x00000000); // Immediate
    pub const ENUPD2_LSYNC = usize(0x00000020); // Locally Synchronized
    pub const ENUPD2_GSYNC = usize(0x00000030); // Globally Synchronized
    pub const ENUPD1_M = usize(0x0000000C); // MnPWM1 Enable Update Mode
    pub const ENUPD1_IMM = usize(0x00000000); // Immediate
    pub const ENUPD1_LSYNC = usize(0x00000008); // Locally Synchronized
    pub const ENUPD1_GSYNC = usize(0x0000000C); // Globally Synchronized
    pub const ENUPD0_M = usize(0x00000003); // MnPWM0 Enable Update Mode
    pub const ENUPD0_IMM = usize(0x00000000); // Immediate
    pub const ENUPD0_LSYNC = usize(0x00000002); // Locally Synchronized
    pub const ENUPD0_GSYNC = usize(0x00000003); // Globally Synchronized
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CTL register.
//
//*****************************************************************************
pub const PWM0_CTL = struct {
    pub const LATCH = usize(0x00040000); // Latch Fault Input
    pub const MINFLTPER = usize(0x00020000); // Minimum Fault Period
    pub const FLTSRC = usize(0x00010000); // Fault Condition Source
    pub const DBFALLUPD_M = usize(0x0000C000); // PWMnDBFALL Update Mode
    pub const DBFALLUPD_I = usize(0x00000000); // Immediate
    pub const DBFALLUPD_LS = usize(0x00008000); // Locally Synchronized
    pub const DBFALLUPD_GS = usize(0x0000C000); // Globally Synchronized
    pub const DBRISEUPD_M = usize(0x00003000); // PWMnDBRISE Update Mode
    pub const DBRISEUPD_I = usize(0x00000000); // Immediate
    pub const DBRISEUPD_LS = usize(0x00002000); // Locally Synchronized
    pub const DBRISEUPD_GS = usize(0x00003000); // Globally Synchronized
    pub const DBCTLUPD_M = usize(0x00000C00); // PWMnDBCTL Update Mode
    pub const DBCTLUPD_I = usize(0x00000000); // Immediate
    pub const DBCTLUPD_LS = usize(0x00000800); // Locally Synchronized
    pub const DBCTLUPD_GS = usize(0x00000C00); // Globally Synchronized
    pub const GENBUPD_M = usize(0x00000300); // PWMnGENB Update Mode
    pub const GENBUPD_I = usize(0x00000000); // Immediate
    pub const GENBUPD_LS = usize(0x00000200); // Locally Synchronized
    pub const GENBUPD_GS = usize(0x00000300); // Globally Synchronized
    pub const GENAUPD_M = usize(0x000000C0); // PWMnGENA Update Mode
    pub const GENAUPD_I = usize(0x00000000); // Immediate
    pub const GENAUPD_LS = usize(0x00000080); // Locally Synchronized
    pub const GENAUPD_GS = usize(0x000000C0); // Globally Synchronized
    pub const CMPBUPD = usize(0x00000020); // Comparator B Update Mode
    pub const CMPAUPD = usize(0x00000010); // Comparator A Update Mode
    pub const LOADUPD = usize(0x00000008); // Load Register Update Mode
    pub const DEBUG = usize(0x00000004); // Debug Mode
    pub const MODE = usize(0x00000002); // Counter Mode
    pub const ENABLE = usize(0x00000001); // PWM Block Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_INTEN register.
//
//*****************************************************************************
pub const PWM0_INTEN = struct {
    pub const TRCMPBD = usize(0x00002000); // Trigger for Counter=PWMnCMPB
    // Down
    pub const TRCMPBU = usize(0x00001000); // Trigger for Counter=PWMnCMPB Up
    pub const TRCMPAD = usize(0x00000800); // Trigger for Counter=PWMnCMPA
    // Down
    pub const TRCMPAU = usize(0x00000400); // Trigger for Counter=PWMnCMPA Up
    pub const TRCNTLOAD = usize(0x00000200); // Trigger for Counter=PWMnLOAD
    pub const TRCNTZERO = usize(0x00000100); // Trigger for Counter=0
    pub const INTCMPBD = usize(0x00000020); // Interrupt for Counter=PWMnCMPB
    // Down
    pub const INTCMPBU = usize(0x00000010); // Interrupt for Counter=PWMnCMPB
    // Up
    pub const INTCMPAD = usize(0x00000008); // Interrupt for Counter=PWMnCMPA
    // Down
    pub const INTCMPAU = usize(0x00000004); // Interrupt for Counter=PWMnCMPA
    // Up
    pub const INTCNTLOAD = usize(0x00000002); // Interrupt for Counter=PWMnLOAD
    pub const INTCNTZERO = usize(0x00000001); // Interrupt for Counter=0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_RIS register.
//
//*****************************************************************************
pub const PWM0_RIS = struct {
    pub const INTCMPBD = usize(0x00000020); // Comparator B Down Interrupt
    // Status
    pub const INTCMPBU = usize(0x00000010); // Comparator B Up Interrupt Status
    pub const INTCMPAD = usize(0x00000008); // Comparator A Down Interrupt
    // Status
    pub const INTCMPAU = usize(0x00000004); // Comparator A Up Interrupt Status
    pub const INTCNTLOAD = usize(0x00000002); // Counter=Load Interrupt Status
    pub const INTCNTZERO = usize(0x00000001); // Counter=0 Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_ISC register.
//
//*****************************************************************************
pub const PWM0_ISC = struct {
    pub const INTCMPBD = usize(0x00000020); // Comparator B Down Interrupt
    pub const INTCMPBU = usize(0x00000010); // Comparator B Up Interrupt
    pub const INTCMPAD = usize(0x00000008); // Comparator A Down Interrupt
    pub const INTCMPAU = usize(0x00000004); // Comparator A Up Interrupt
    pub const INTCNTLOAD = usize(0x00000002); // Counter=Load Interrupt
    pub const INTCNTZERO = usize(0x00000001); // Counter=0 Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_LOAD register.
//
//*****************************************************************************
pub const PWM0_LOAD = struct {
    pub const M = usize(0x0000FFFF); // Counter Load Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_COUNT register.
//
//*****************************************************************************
pub const PWM0_COUNT = struct {
    pub const M = usize(0x0000FFFF); // Counter Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPA register.
//
//*****************************************************************************
pub const PWM0_CMPA = struct {
    pub const M = usize(0x0000FFFF); // Comparator A Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPB register.
//
//*****************************************************************************
pub const PWM0_CMPB = struct {
    pub const M = usize(0x0000FFFF); // Comparator B Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENA register.
//
//*****************************************************************************
pub const PWM0_GENA = struct {
    pub const ACTCMPBD_M = usize(0x00000C00); // Action for Comparator B Down
    pub const ACTCMPBD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBD_INV = usize(0x00000400); // Invert pwmA
    pub const ACTCMPBD_ZERO = usize(0x00000800); // Drive pwmA Low
    pub const ACTCMPBD_ONE = usize(0x00000C00); // Drive pwmA High
    pub const ACTCMPBU_M = usize(0x00000300); // Action for Comparator B Up
    pub const ACTCMPBU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBU_INV = usize(0x00000100); // Invert pwmA
    pub const ACTCMPBU_ZERO = usize(0x00000200); // Drive pwmA Low
    pub const ACTCMPBU_ONE = usize(0x00000300); // Drive pwmA High
    pub const ACTCMPAD_M = usize(0x000000C0); // Action for Comparator A Down
    pub const ACTCMPAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAD_INV = usize(0x00000040); // Invert pwmA
    pub const ACTCMPAD_ZERO = usize(0x00000080); // Drive pwmA Low
    pub const ACTCMPAD_ONE = usize(0x000000C0); // Drive pwmA High
    pub const ACTCMPAU_M = usize(0x00000030); // Action for Comparator A Up
    pub const ACTCMPAU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAU_INV = usize(0x00000010); // Invert pwmA
    pub const ACTCMPAU_ZERO = usize(0x00000020); // Drive pwmA Low
    pub const ACTCMPAU_ONE = usize(0x00000030); // Drive pwmA High
    pub const ACTLOAD_M = usize(0x0000000C); // Action for Counter=LOAD
    pub const ACTLOAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTLOAD_INV = usize(0x00000004); // Invert pwmA
    pub const ACTLOAD_ZERO = usize(0x00000008); // Drive pwmA Low
    pub const ACTLOAD_ONE = usize(0x0000000C); // Drive pwmA High
    pub const ACTZERO_M = usize(0x00000003); // Action for Counter=0
    pub const ACTZERO_NONE = usize(0x00000000); // Do nothing
    pub const ACTZERO_INV = usize(0x00000001); // Invert pwmA
    pub const ACTZERO_ZERO = usize(0x00000002); // Drive pwmA Low
    pub const ACTZERO_ONE = usize(0x00000003); // Drive pwmA High
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENB register.
//
//*****************************************************************************
pub const PWM0_GENB = struct {
    pub const ACTCMPBD_M = usize(0x00000C00); // Action for Comparator B Down
    pub const ACTCMPBD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBD_INV = usize(0x00000400); // Invert pwmB
    pub const ACTCMPBD_ZERO = usize(0x00000800); // Drive pwmB Low
    pub const ACTCMPBD_ONE = usize(0x00000C00); // Drive pwmB High
    pub const ACTCMPBU_M = usize(0x00000300); // Action for Comparator B Up
    pub const ACTCMPBU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBU_INV = usize(0x00000100); // Invert pwmB
    pub const ACTCMPBU_ZERO = usize(0x00000200); // Drive pwmB Low
    pub const ACTCMPBU_ONE = usize(0x00000300); // Drive pwmB High
    pub const ACTCMPAD_M = usize(0x000000C0); // Action for Comparator A Down
    pub const ACTCMPAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAD_INV = usize(0x00000040); // Invert pwmB
    pub const ACTCMPAD_ZERO = usize(0x00000080); // Drive pwmB Low
    pub const ACTCMPAD_ONE = usize(0x000000C0); // Drive pwmB High
    pub const ACTCMPAU_M = usize(0x00000030); // Action for Comparator A Up
    pub const ACTCMPAU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAU_INV = usize(0x00000010); // Invert pwmB
    pub const ACTCMPAU_ZERO = usize(0x00000020); // Drive pwmB Low
    pub const ACTCMPAU_ONE = usize(0x00000030); // Drive pwmB High
    pub const ACTLOAD_M = usize(0x0000000C); // Action for Counter=LOAD
    pub const ACTLOAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTLOAD_INV = usize(0x00000004); // Invert pwmB
    pub const ACTLOAD_ZERO = usize(0x00000008); // Drive pwmB Low
    pub const ACTLOAD_ONE = usize(0x0000000C); // Drive pwmB High
    pub const ACTZERO_M = usize(0x00000003); // Action for Counter=0
    pub const ACTZERO_NONE = usize(0x00000000); // Do nothing
    pub const ACTZERO_INV = usize(0x00000001); // Invert pwmB
    pub const ACTZERO_ZERO = usize(0x00000002); // Drive pwmB Low
    pub const ACTZERO_ONE = usize(0x00000003); // Drive pwmB High
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBCTL register.
//
//*****************************************************************************
pub const PWM0_DBCTL = struct {
    pub const ENABLE = usize(0x00000001); // Dead-Band Generator Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBRISE register.
//
//*****************************************************************************
pub const PWM0_DBRISE = struct {
    pub const DELAY_M = usize(0x00000FFF); // Dead-Band Rise Delay
    pub const DELAY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBFALL register.
//
//*****************************************************************************
pub const PWM0_DBFALL = struct {
    pub const DELAY_M = usize(0x00000FFF); // Dead-Band Fall Delay
    pub const DELAY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM0_FLTSRC0 = struct {
    pub const FAULT3 = usize(0x00000008); // Fault3 Input
    pub const FAULT2 = usize(0x00000004); // Fault2 Input
    pub const FAULT1 = usize(0x00000002); // Fault1 Input
    pub const FAULT0 = usize(0x00000001); // Fault0 Input
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM0_FLTSRC1 = struct {
    pub const DCMP7 = usize(0x00000080); // Digital Comparator 7
    pub const DCMP6 = usize(0x00000040); // Digital Comparator 6
    pub const DCMP5 = usize(0x00000020); // Digital Comparator 5
    pub const DCMP4 = usize(0x00000010); // Digital Comparator 4
    pub const DCMP3 = usize(0x00000008); // Digital Comparator 3
    pub const DCMP2 = usize(0x00000004); // Digital Comparator 2
    pub const DCMP1 = usize(0x00000002); // Digital Comparator 1
    pub const DCMP0 = usize(0x00000001); // Digital Comparator 0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM0_MINFLTPER = struct {
    pub const M = usize(0x0000FFFF); // Minimum Fault Period
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CTL register.
//
//*****************************************************************************
pub const PWM1_CTL = struct {
    pub const LATCH = usize(0x00040000); // Latch Fault Input
    pub const MINFLTPER = usize(0x00020000); // Minimum Fault Period
    pub const FLTSRC = usize(0x00010000); // Fault Condition Source
    pub const DBFALLUPD_M = usize(0x0000C000); // PWMnDBFALL Update Mode
    pub const DBFALLUPD_I = usize(0x00000000); // Immediate
    pub const DBFALLUPD_LS = usize(0x00008000); // Locally Synchronized
    pub const DBFALLUPD_GS = usize(0x0000C000); // Globally Synchronized
    pub const DBRISEUPD_M = usize(0x00003000); // PWMnDBRISE Update Mode
    pub const DBRISEUPD_I = usize(0x00000000); // Immediate
    pub const DBRISEUPD_LS = usize(0x00002000); // Locally Synchronized
    pub const DBRISEUPD_GS = usize(0x00003000); // Globally Synchronized
    pub const DBCTLUPD_M = usize(0x00000C00); // PWMnDBCTL Update Mode
    pub const DBCTLUPD_I = usize(0x00000000); // Immediate
    pub const DBCTLUPD_LS = usize(0x00000800); // Locally Synchronized
    pub const DBCTLUPD_GS = usize(0x00000C00); // Globally Synchronized
    pub const GENBUPD_M = usize(0x00000300); // PWMnGENB Update Mode
    pub const GENBUPD_I = usize(0x00000000); // Immediate
    pub const GENBUPD_LS = usize(0x00000200); // Locally Synchronized
    pub const GENBUPD_GS = usize(0x00000300); // Globally Synchronized
    pub const GENAUPD_M = usize(0x000000C0); // PWMnGENA Update Mode
    pub const GENAUPD_I = usize(0x00000000); // Immediate
    pub const GENAUPD_LS = usize(0x00000080); // Locally Synchronized
    pub const GENAUPD_GS = usize(0x000000C0); // Globally Synchronized
    pub const CMPBUPD = usize(0x00000020); // Comparator B Update Mode
    pub const CMPAUPD = usize(0x00000010); // Comparator A Update Mode
    pub const LOADUPD = usize(0x00000008); // Load Register Update Mode
    pub const DEBUG = usize(0x00000004); // Debug Mode
    pub const MODE = usize(0x00000002); // Counter Mode
    pub const ENABLE = usize(0x00000001); // PWM Block Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_INTEN register.
//
//*****************************************************************************
pub const PWM1_INTEN = struct {
    pub const TRCMPBD = usize(0x00002000); // Trigger for Counter=PWMnCMPB
    // Down
    pub const TRCMPBU = usize(0x00001000); // Trigger for Counter=PWMnCMPB Up
    pub const TRCMPAD = usize(0x00000800); // Trigger for Counter=PWMnCMPA
    // Down
    pub const TRCMPAU = usize(0x00000400); // Trigger for Counter=PWMnCMPA Up
    pub const TRCNTLOAD = usize(0x00000200); // Trigger for Counter=PWMnLOAD
    pub const TRCNTZERO = usize(0x00000100); // Trigger for Counter=0
    pub const INTCMPBD = usize(0x00000020); // Interrupt for Counter=PWMnCMPB
    // Down
    pub const INTCMPBU = usize(0x00000010); // Interrupt for Counter=PWMnCMPB
    // Up
    pub const INTCMPAD = usize(0x00000008); // Interrupt for Counter=PWMnCMPA
    // Down
    pub const INTCMPAU = usize(0x00000004); // Interrupt for Counter=PWMnCMPA
    // Up
    pub const INTCNTLOAD = usize(0x00000002); // Interrupt for Counter=PWMnLOAD
    pub const INTCNTZERO = usize(0x00000001); // Interrupt for Counter=0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_RIS register.
//
//*****************************************************************************
pub const PWM1_RIS = struct {
    pub const INTCMPBD = usize(0x00000020); // Comparator B Down Interrupt
    // Status
    pub const INTCMPBU = usize(0x00000010); // Comparator B Up Interrupt Status
    pub const INTCMPAD = usize(0x00000008); // Comparator A Down Interrupt
    // Status
    pub const INTCMPAU = usize(0x00000004); // Comparator A Up Interrupt Status
    pub const INTCNTLOAD = usize(0x00000002); // Counter=Load Interrupt Status
    pub const INTCNTZERO = usize(0x00000001); // Counter=0 Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_ISC register.
//
//*****************************************************************************
pub const PWM1_ISC = struct {
    pub const INTCMPBD = usize(0x00000020); // Comparator B Down Interrupt
    pub const INTCMPBU = usize(0x00000010); // Comparator B Up Interrupt
    pub const INTCMPAD = usize(0x00000008); // Comparator A Down Interrupt
    pub const INTCMPAU = usize(0x00000004); // Comparator A Up Interrupt
    pub const INTCNTLOAD = usize(0x00000002); // Counter=Load Interrupt
    pub const INTCNTZERO = usize(0x00000001); // Counter=0 Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_LOAD register.
//
//*****************************************************************************
pub const PWM1_LOAD = struct {
    pub const LOAD_M = usize(0x0000FFFF); // Counter Load Value
    pub const LOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_COUNT register.
//
//*****************************************************************************
pub const PWM1_COUNT = struct {
    pub const COUNT_M = usize(0x0000FFFF); // Counter Value
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CMPA register.
//
//*****************************************************************************
pub const PWM1_CMPA = struct {
    pub const COMPA_M = usize(0x0000FFFF); // Comparator A Value
    pub const COMPA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CMPB register.
//
//*****************************************************************************
pub const PWM1_CMPB = struct {
    pub const COMPB_M = usize(0x0000FFFF); // Comparator B Value
    pub const COMPB_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_GENA register.
//
//*****************************************************************************
pub const PWM1_GENA = struct {
    pub const ACTCMPBD_M = usize(0x00000C00); // Action for Comparator B Down
    pub const ACTCMPBD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBD_INV = usize(0x00000400); // Invert pwmA
    pub const ACTCMPBD_ZERO = usize(0x00000800); // Drive pwmA Low
    pub const ACTCMPBD_ONE = usize(0x00000C00); // Drive pwmA High
    pub const ACTCMPBU_M = usize(0x00000300); // Action for Comparator B Up
    pub const ACTCMPBU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBU_INV = usize(0x00000100); // Invert pwmA
    pub const ACTCMPBU_ZERO = usize(0x00000200); // Drive pwmA Low
    pub const ACTCMPBU_ONE = usize(0x00000300); // Drive pwmA High
    pub const ACTCMPAD_M = usize(0x000000C0); // Action for Comparator A Down
    pub const ACTCMPAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAD_INV = usize(0x00000040); // Invert pwmA
    pub const ACTCMPAD_ZERO = usize(0x00000080); // Drive pwmA Low
    pub const ACTCMPAD_ONE = usize(0x000000C0); // Drive pwmA High
    pub const ACTCMPAU_M = usize(0x00000030); // Action for Comparator A Up
    pub const ACTCMPAU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAU_INV = usize(0x00000010); // Invert pwmA
    pub const ACTCMPAU_ZERO = usize(0x00000020); // Drive pwmA Low
    pub const ACTCMPAU_ONE = usize(0x00000030); // Drive pwmA High
    pub const ACTLOAD_M = usize(0x0000000C); // Action for Counter=LOAD
    pub const ACTLOAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTLOAD_INV = usize(0x00000004); // Invert pwmA
    pub const ACTLOAD_ZERO = usize(0x00000008); // Drive pwmA Low
    pub const ACTLOAD_ONE = usize(0x0000000C); // Drive pwmA High
    pub const ACTZERO_M = usize(0x00000003); // Action for Counter=0
    pub const ACTZERO_NONE = usize(0x00000000); // Do nothing
    pub const ACTZERO_INV = usize(0x00000001); // Invert pwmA
    pub const ACTZERO_ZERO = usize(0x00000002); // Drive pwmA Low
    pub const ACTZERO_ONE = usize(0x00000003); // Drive pwmA High
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_GENB register.
//
//*****************************************************************************
pub const PWM1_GENB = struct {
    pub const ACTCMPBD_M = usize(0x00000C00); // Action for Comparator B Down
    pub const ACTCMPBD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBD_INV = usize(0x00000400); // Invert pwmB
    pub const ACTCMPBD_ZERO = usize(0x00000800); // Drive pwmB Low
    pub const ACTCMPBD_ONE = usize(0x00000C00); // Drive pwmB High
    pub const ACTCMPBU_M = usize(0x00000300); // Action for Comparator B Up
    pub const ACTCMPBU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBU_INV = usize(0x00000100); // Invert pwmB
    pub const ACTCMPBU_ZERO = usize(0x00000200); // Drive pwmB Low
    pub const ACTCMPBU_ONE = usize(0x00000300); // Drive pwmB High
    pub const ACTCMPAD_M = usize(0x000000C0); // Action for Comparator A Down
    pub const ACTCMPAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAD_INV = usize(0x00000040); // Invert pwmB
    pub const ACTCMPAD_ZERO = usize(0x00000080); // Drive pwmB Low
    pub const ACTCMPAD_ONE = usize(0x000000C0); // Drive pwmB High
    pub const ACTCMPAU_M = usize(0x00000030); // Action for Comparator A Up
    pub const ACTCMPAU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAU_INV = usize(0x00000010); // Invert pwmB
    pub const ACTCMPAU_ZERO = usize(0x00000020); // Drive pwmB Low
    pub const ACTCMPAU_ONE = usize(0x00000030); // Drive pwmB High
    pub const ACTLOAD_M = usize(0x0000000C); // Action for Counter=LOAD
    pub const ACTLOAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTLOAD_INV = usize(0x00000004); // Invert pwmB
    pub const ACTLOAD_ZERO = usize(0x00000008); // Drive pwmB Low
    pub const ACTLOAD_ONE = usize(0x0000000C); // Drive pwmB High
    pub const ACTZERO_M = usize(0x00000003); // Action for Counter=0
    pub const ACTZERO_NONE = usize(0x00000000); // Do nothing
    pub const ACTZERO_INV = usize(0x00000001); // Invert pwmB
    pub const ACTZERO_ZERO = usize(0x00000002); // Drive pwmB Low
    pub const ACTZERO_ONE = usize(0x00000003); // Drive pwmB High
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBCTL register.
//
//*****************************************************************************
pub const PWM1_DBCTL = struct {
    pub const ENABLE = usize(0x00000001); // Dead-Band Generator Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBRISE register.
//
//*****************************************************************************
pub const PWM1_DBRISE = struct {
    pub const RISEDELAY_M = usize(0x00000FFF); // Dead-Band Rise Delay
    pub const RISEDELAY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBFALL register.
//
//*****************************************************************************
pub const PWM1_DBFALL = struct {
    pub const FALLDELAY_M = usize(0x00000FFF); // Dead-Band Fall Delay
    pub const FALLDELAY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM1_FLTSRC0 = struct {
    pub const FAULT3 = usize(0x00000008); // Fault3 Input
    pub const FAULT2 = usize(0x00000004); // Fault2 Input
    pub const FAULT1 = usize(0x00000002); // Fault1 Input
    pub const FAULT0 = usize(0x00000001); // Fault0 Input
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM1_FLTSRC1 = struct {
    pub const DCMP7 = usize(0x00000080); // Digital Comparator 7
    pub const DCMP6 = usize(0x00000040); // Digital Comparator 6
    pub const DCMP5 = usize(0x00000020); // Digital Comparator 5
    pub const DCMP4 = usize(0x00000010); // Digital Comparator 4
    pub const DCMP3 = usize(0x00000008); // Digital Comparator 3
    pub const DCMP2 = usize(0x00000004); // Digital Comparator 2
    pub const DCMP1 = usize(0x00000002); // Digital Comparator 1
    pub const DCMP0 = usize(0x00000001); // Digital Comparator 0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM1_MINFLTPER = struct {
    pub const MFP_M = usize(0x0000FFFF); // Minimum Fault Period
    pub const MFP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CTL register.
//
//*****************************************************************************
pub const PWM2_CTL = struct {
    pub const LATCH = usize(0x00040000); // Latch Fault Input
    pub const MINFLTPER = usize(0x00020000); // Minimum Fault Period
    pub const FLTSRC = usize(0x00010000); // Fault Condition Source
    pub const DBFALLUPD_M = usize(0x0000C000); // PWMnDBFALL Update Mode
    pub const DBFALLUPD_I = usize(0x00000000); // Immediate
    pub const DBFALLUPD_LS = usize(0x00008000); // Locally Synchronized
    pub const DBFALLUPD_GS = usize(0x0000C000); // Globally Synchronized
    pub const DBRISEUPD_M = usize(0x00003000); // PWMnDBRISE Update Mode
    pub const DBRISEUPD_I = usize(0x00000000); // Immediate
    pub const DBRISEUPD_LS = usize(0x00002000); // Locally Synchronized
    pub const DBRISEUPD_GS = usize(0x00003000); // Globally Synchronized
    pub const DBCTLUPD_M = usize(0x00000C00); // PWMnDBCTL Update Mode
    pub const DBCTLUPD_I = usize(0x00000000); // Immediate
    pub const DBCTLUPD_LS = usize(0x00000800); // Locally Synchronized
    pub const DBCTLUPD_GS = usize(0x00000C00); // Globally Synchronized
    pub const GENBUPD_M = usize(0x00000300); // PWMnGENB Update Mode
    pub const GENBUPD_I = usize(0x00000000); // Immediate
    pub const GENBUPD_LS = usize(0x00000200); // Locally Synchronized
    pub const GENBUPD_GS = usize(0x00000300); // Globally Synchronized
    pub const GENAUPD_M = usize(0x000000C0); // PWMnGENA Update Mode
    pub const GENAUPD_I = usize(0x00000000); // Immediate
    pub const GENAUPD_LS = usize(0x00000080); // Locally Synchronized
    pub const GENAUPD_GS = usize(0x000000C0); // Globally Synchronized
    pub const CMPBUPD = usize(0x00000020); // Comparator B Update Mode
    pub const CMPAUPD = usize(0x00000010); // Comparator A Update Mode
    pub const LOADUPD = usize(0x00000008); // Load Register Update Mode
    pub const DEBUG = usize(0x00000004); // Debug Mode
    pub const MODE = usize(0x00000002); // Counter Mode
    pub const ENABLE = usize(0x00000001); // PWM Block Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_INTEN register.
//
//*****************************************************************************
pub const PWM2_INTEN = struct {
    pub const TRCMPBD = usize(0x00002000); // Trigger for Counter=PWMnCMPB
    // Down
    pub const TRCMPBU = usize(0x00001000); // Trigger for Counter=PWMnCMPB Up
    pub const TRCMPAD = usize(0x00000800); // Trigger for Counter=PWMnCMPA
    // Down
    pub const TRCMPAU = usize(0x00000400); // Trigger for Counter=PWMnCMPA Up
    pub const TRCNTLOAD = usize(0x00000200); // Trigger for Counter=PWMnLOAD
    pub const TRCNTZERO = usize(0x00000100); // Trigger for Counter=0
    pub const INTCMPBD = usize(0x00000020); // Interrupt for Counter=PWMnCMPB
    // Down
    pub const INTCMPBU = usize(0x00000010); // Interrupt for Counter=PWMnCMPB
    // Up
    pub const INTCMPAD = usize(0x00000008); // Interrupt for Counter=PWMnCMPA
    // Down
    pub const INTCMPAU = usize(0x00000004); // Interrupt for Counter=PWMnCMPA
    // Up
    pub const INTCNTLOAD = usize(0x00000002); // Interrupt for Counter=PWMnLOAD
    pub const INTCNTZERO = usize(0x00000001); // Interrupt for Counter=0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_RIS register.
//
//*****************************************************************************
pub const PWM2_RIS = struct {
    pub const INTCMPBD = usize(0x00000020); // Comparator B Down Interrupt
    // Status
    pub const INTCMPBU = usize(0x00000010); // Comparator B Up Interrupt Status
    pub const INTCMPAD = usize(0x00000008); // Comparator A Down Interrupt
    // Status
    pub const INTCMPAU = usize(0x00000004); // Comparator A Up Interrupt Status
    pub const INTCNTLOAD = usize(0x00000002); // Counter=Load Interrupt Status
    pub const INTCNTZERO = usize(0x00000001); // Counter=0 Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_ISC register.
//
//*****************************************************************************
pub const PWM2_ISC = struct {
    pub const INTCMPBD = usize(0x00000020); // Comparator B Down Interrupt
    pub const INTCMPBU = usize(0x00000010); // Comparator B Up Interrupt
    pub const INTCMPAD = usize(0x00000008); // Comparator A Down Interrupt
    pub const INTCMPAU = usize(0x00000004); // Comparator A Up Interrupt
    pub const INTCNTLOAD = usize(0x00000002); // Counter=Load Interrupt
    pub const INTCNTZERO = usize(0x00000001); // Counter=0 Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_LOAD register.
//
//*****************************************************************************
pub const PWM2_LOAD = struct {
    pub const LOAD_M = usize(0x0000FFFF); // Counter Load Value
    pub const LOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_COUNT register.
//
//*****************************************************************************
pub const PWM2_COUNT = struct {
    pub const COUNT_M = usize(0x0000FFFF); // Counter Value
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CMPA register.
//
//*****************************************************************************
pub const PWM2_CMPA = struct {
    pub const COMPA_M = usize(0x0000FFFF); // Comparator A Value
    pub const COMPA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CMPB register.
//
//*****************************************************************************
pub const PWM2_CMPB = struct {
    pub const COMPB_M = usize(0x0000FFFF); // Comparator B Value
    pub const COMPB_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_GENA register.
//
//*****************************************************************************
pub const PWM2_GENA = struct {
    pub const ACTCMPBD_M = usize(0x00000C00); // Action for Comparator B Down
    pub const ACTCMPBD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBD_INV = usize(0x00000400); // Invert pwmA
    pub const ACTCMPBD_ZERO = usize(0x00000800); // Drive pwmA Low
    pub const ACTCMPBD_ONE = usize(0x00000C00); // Drive pwmA High
    pub const ACTCMPBU_M = usize(0x00000300); // Action for Comparator B Up
    pub const ACTCMPBU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBU_INV = usize(0x00000100); // Invert pwmA
    pub const ACTCMPBU_ZERO = usize(0x00000200); // Drive pwmA Low
    pub const ACTCMPBU_ONE = usize(0x00000300); // Drive pwmA High
    pub const ACTCMPAD_M = usize(0x000000C0); // Action for Comparator A Down
    pub const ACTCMPAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAD_INV = usize(0x00000040); // Invert pwmA
    pub const ACTCMPAD_ZERO = usize(0x00000080); // Drive pwmA Low
    pub const ACTCMPAD_ONE = usize(0x000000C0); // Drive pwmA High
    pub const ACTCMPAU_M = usize(0x00000030); // Action for Comparator A Up
    pub const ACTCMPAU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAU_INV = usize(0x00000010); // Invert pwmA
    pub const ACTCMPAU_ZERO = usize(0x00000020); // Drive pwmA Low
    pub const ACTCMPAU_ONE = usize(0x00000030); // Drive pwmA High
    pub const ACTLOAD_M = usize(0x0000000C); // Action for Counter=LOAD
    pub const ACTLOAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTLOAD_INV = usize(0x00000004); // Invert pwmA
    pub const ACTLOAD_ZERO = usize(0x00000008); // Drive pwmA Low
    pub const ACTLOAD_ONE = usize(0x0000000C); // Drive pwmA High
    pub const ACTZERO_M = usize(0x00000003); // Action for Counter=0
    pub const ACTZERO_NONE = usize(0x00000000); // Do nothing
    pub const ACTZERO_INV = usize(0x00000001); // Invert pwmA
    pub const ACTZERO_ZERO = usize(0x00000002); // Drive pwmA Low
    pub const ACTZERO_ONE = usize(0x00000003); // Drive pwmA High
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_GENB register.
//
//*****************************************************************************
pub const PWM2_GENB = struct {
    pub const ACTCMPBD_M = usize(0x00000C00); // Action for Comparator B Down
    pub const ACTCMPBD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBD_INV = usize(0x00000400); // Invert pwmB
    pub const ACTCMPBD_ZERO = usize(0x00000800); // Drive pwmB Low
    pub const ACTCMPBD_ONE = usize(0x00000C00); // Drive pwmB High
    pub const ACTCMPBU_M = usize(0x00000300); // Action for Comparator B Up
    pub const ACTCMPBU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBU_INV = usize(0x00000100); // Invert pwmB
    pub const ACTCMPBU_ZERO = usize(0x00000200); // Drive pwmB Low
    pub const ACTCMPBU_ONE = usize(0x00000300); // Drive pwmB High
    pub const ACTCMPAD_M = usize(0x000000C0); // Action for Comparator A Down
    pub const ACTCMPAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAD_INV = usize(0x00000040); // Invert pwmB
    pub const ACTCMPAD_ZERO = usize(0x00000080); // Drive pwmB Low
    pub const ACTCMPAD_ONE = usize(0x000000C0); // Drive pwmB High
    pub const ACTCMPAU_M = usize(0x00000030); // Action for Comparator A Up
    pub const ACTCMPAU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAU_INV = usize(0x00000010); // Invert pwmB
    pub const ACTCMPAU_ZERO = usize(0x00000020); // Drive pwmB Low
    pub const ACTCMPAU_ONE = usize(0x00000030); // Drive pwmB High
    pub const ACTLOAD_M = usize(0x0000000C); // Action for Counter=LOAD
    pub const ACTLOAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTLOAD_INV = usize(0x00000004); // Invert pwmB
    pub const ACTLOAD_ZERO = usize(0x00000008); // Drive pwmB Low
    pub const ACTLOAD_ONE = usize(0x0000000C); // Drive pwmB High
    pub const ACTZERO_M = usize(0x00000003); // Action for Counter=0
    pub const ACTZERO_NONE = usize(0x00000000); // Do nothing
    pub const ACTZERO_INV = usize(0x00000001); // Invert pwmB
    pub const ACTZERO_ZERO = usize(0x00000002); // Drive pwmB Low
    pub const ACTZERO_ONE = usize(0x00000003); // Drive pwmB High
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBCTL register.
//
//*****************************************************************************
pub const PWM2_DBCTL = struct {
    pub const ENABLE = usize(0x00000001); // Dead-Band Generator Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBRISE register.
//
//*****************************************************************************
pub const PWM2_DBRISE = struct {
    pub const RISEDELAY_M = usize(0x00000FFF); // Dead-Band Rise Delay
    pub const RISEDELAY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBFALL register.
//
//*****************************************************************************
pub const PWM2_DBFALL = struct {
    pub const FALLDELAY_M = usize(0x00000FFF); // Dead-Band Fall Delay
    pub const FALLDELAY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM2_FLTSRC0 = struct {
    pub const FAULT3 = usize(0x00000008); // Fault3 Input
    pub const FAULT2 = usize(0x00000004); // Fault2 Input
    pub const FAULT1 = usize(0x00000002); // Fault1 Input
    pub const FAULT0 = usize(0x00000001); // Fault0 Input
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM2_FLTSRC1 = struct {
    pub const DCMP7 = usize(0x00000080); // Digital Comparator 7
    pub const DCMP6 = usize(0x00000040); // Digital Comparator 6
    pub const DCMP5 = usize(0x00000020); // Digital Comparator 5
    pub const DCMP4 = usize(0x00000010); // Digital Comparator 4
    pub const DCMP3 = usize(0x00000008); // Digital Comparator 3
    pub const DCMP2 = usize(0x00000004); // Digital Comparator 2
    pub const DCMP1 = usize(0x00000002); // Digital Comparator 1
    pub const DCMP0 = usize(0x00000001); // Digital Comparator 0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM2_MINFLTPER = struct {
    pub const MFP_M = usize(0x0000FFFF); // Minimum Fault Period
    pub const MFP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CTL register.
//
//*****************************************************************************
pub const PWM3_CTL = struct {
    pub const LATCH = usize(0x00040000); // Latch Fault Input
    pub const MINFLTPER = usize(0x00020000); // Minimum Fault Period
    pub const FLTSRC = usize(0x00010000); // Fault Condition Source
    pub const DBFALLUPD_M = usize(0x0000C000); // PWMnDBFALL Update Mode
    pub const DBFALLUPD_I = usize(0x00000000); // Immediate
    pub const DBFALLUPD_LS = usize(0x00008000); // Locally Synchronized
    pub const DBFALLUPD_GS = usize(0x0000C000); // Globally Synchronized
    pub const DBRISEUPD_M = usize(0x00003000); // PWMnDBRISE Update Mode
    pub const DBRISEUPD_I = usize(0x00000000); // Immediate
    pub const DBRISEUPD_LS = usize(0x00002000); // Locally Synchronized
    pub const DBRISEUPD_GS = usize(0x00003000); // Globally Synchronized
    pub const DBCTLUPD_M = usize(0x00000C00); // PWMnDBCTL Update Mode
    pub const DBCTLUPD_I = usize(0x00000000); // Immediate
    pub const DBCTLUPD_LS = usize(0x00000800); // Locally Synchronized
    pub const DBCTLUPD_GS = usize(0x00000C00); // Globally Synchronized
    pub const GENBUPD_M = usize(0x00000300); // PWMnGENB Update Mode
    pub const GENBUPD_I = usize(0x00000000); // Immediate
    pub const GENBUPD_LS = usize(0x00000200); // Locally Synchronized
    pub const GENBUPD_GS = usize(0x00000300); // Globally Synchronized
    pub const GENAUPD_M = usize(0x000000C0); // PWMnGENA Update Mode
    pub const GENAUPD_I = usize(0x00000000); // Immediate
    pub const GENAUPD_LS = usize(0x00000080); // Locally Synchronized
    pub const GENAUPD_GS = usize(0x000000C0); // Globally Synchronized
    pub const CMPBUPD = usize(0x00000020); // Comparator B Update Mode
    pub const CMPAUPD = usize(0x00000010); // Comparator A Update Mode
    pub const LOADUPD = usize(0x00000008); // Load Register Update Mode
    pub const DEBUG = usize(0x00000004); // Debug Mode
    pub const MODE = usize(0x00000002); // Counter Mode
    pub const ENABLE = usize(0x00000001); // PWM Block Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_INTEN register.
//
//*****************************************************************************
pub const PWM3_INTEN = struct {
    pub const TRCMPBD = usize(0x00002000); // Trigger for Counter=PWMnCMPB
    // Down
    pub const TRCMPBU = usize(0x00001000); // Trigger for Counter=PWMnCMPB Up
    pub const TRCMPAD = usize(0x00000800); // Trigger for Counter=PWMnCMPA
    // Down
    pub const TRCMPAU = usize(0x00000400); // Trigger for Counter=PWMnCMPA Up
    pub const TRCNTLOAD = usize(0x00000200); // Trigger for Counter=PWMnLOAD
    pub const TRCNTZERO = usize(0x00000100); // Trigger for Counter=0
    pub const INTCMPBD = usize(0x00000020); // Interrupt for Counter=PWMnCMPB
    // Down
    pub const INTCMPBU = usize(0x00000010); // Interrupt for Counter=PWMnCMPB
    // Up
    pub const INTCMPAD = usize(0x00000008); // Interrupt for Counter=PWMnCMPA
    // Down
    pub const INTCMPAU = usize(0x00000004); // Interrupt for Counter=PWMnCMPA
    // Up
    pub const INTCNTLOAD = usize(0x00000002); // Interrupt for Counter=PWMnLOAD
    pub const INTCNTZERO = usize(0x00000001); // Interrupt for Counter=0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_RIS register.
//
//*****************************************************************************
pub const PWM3_RIS = struct {
    pub const INTCMPBD = usize(0x00000020); // Comparator B Down Interrupt
    // Status
    pub const INTCMPBU = usize(0x00000010); // Comparator B Up Interrupt Status
    pub const INTCMPAD = usize(0x00000008); // Comparator A Down Interrupt
    // Status
    pub const INTCMPAU = usize(0x00000004); // Comparator A Up Interrupt Status
    pub const INTCNTLOAD = usize(0x00000002); // Counter=Load Interrupt Status
    pub const INTCNTZERO = usize(0x00000001); // Counter=0 Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_ISC register.
//
//*****************************************************************************
pub const PWM3_ISC = struct {
    pub const INTCMPBD = usize(0x00000020); // Comparator B Down Interrupt
    pub const INTCMPBU = usize(0x00000010); // Comparator B Up Interrupt
    pub const INTCMPAD = usize(0x00000008); // Comparator A Down Interrupt
    pub const INTCMPAU = usize(0x00000004); // Comparator A Up Interrupt
    pub const INTCNTLOAD = usize(0x00000002); // Counter=Load Interrupt
    pub const INTCNTZERO = usize(0x00000001); // Counter=0 Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_LOAD register.
//
//*****************************************************************************
pub const PWM3_LOAD = struct {
    pub const LOAD_M = usize(0x0000FFFF); // Counter Load Value
    pub const LOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_COUNT register.
//
//*****************************************************************************
pub const PWM3_COUNT = struct {
    pub const COUNT_M = usize(0x0000FFFF); // Counter Value
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CMPA register.
//
//*****************************************************************************
pub const PWM3_CMPA = struct {
    pub const COMPA_M = usize(0x0000FFFF); // Comparator A Value
    pub const COMPA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CMPB register.
//
//*****************************************************************************
pub const PWM3_CMPB = struct {
    pub const COMPB_M = usize(0x0000FFFF); // Comparator B Value
    pub const COMPB_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_GENA register.
//
//*****************************************************************************
pub const PWM3_GENA = struct {
    pub const ACTCMPBD_M = usize(0x00000C00); // Action for Comparator B Down
    pub const ACTCMPBD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBD_INV = usize(0x00000400); // Invert pwmA
    pub const ACTCMPBD_ZERO = usize(0x00000800); // Drive pwmA Low
    pub const ACTCMPBD_ONE = usize(0x00000C00); // Drive pwmA High
    pub const ACTCMPBU_M = usize(0x00000300); // Action for Comparator B Up
    pub const ACTCMPBU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBU_INV = usize(0x00000100); // Invert pwmA
    pub const ACTCMPBU_ZERO = usize(0x00000200); // Drive pwmA Low
    pub const ACTCMPBU_ONE = usize(0x00000300); // Drive pwmA High
    pub const ACTCMPAD_M = usize(0x000000C0); // Action for Comparator A Down
    pub const ACTCMPAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAD_INV = usize(0x00000040); // Invert pwmA
    pub const ACTCMPAD_ZERO = usize(0x00000080); // Drive pwmA Low
    pub const ACTCMPAD_ONE = usize(0x000000C0); // Drive pwmA High
    pub const ACTCMPAU_M = usize(0x00000030); // Action for Comparator A Up
    pub const ACTCMPAU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAU_INV = usize(0x00000010); // Invert pwmA
    pub const ACTCMPAU_ZERO = usize(0x00000020); // Drive pwmA Low
    pub const ACTCMPAU_ONE = usize(0x00000030); // Drive pwmA High
    pub const ACTLOAD_M = usize(0x0000000C); // Action for Counter=LOAD
    pub const ACTLOAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTLOAD_INV = usize(0x00000004); // Invert pwmA
    pub const ACTLOAD_ZERO = usize(0x00000008); // Drive pwmA Low
    pub const ACTLOAD_ONE = usize(0x0000000C); // Drive pwmA High
    pub const ACTZERO_M = usize(0x00000003); // Action for Counter=0
    pub const ACTZERO_NONE = usize(0x00000000); // Do nothing
    pub const ACTZERO_INV = usize(0x00000001); // Invert pwmA
    pub const ACTZERO_ZERO = usize(0x00000002); // Drive pwmA Low
    pub const ACTZERO_ONE = usize(0x00000003); // Drive pwmA High
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_GENB register.
//
//*****************************************************************************
pub const PWM3_GENB = struct {
    pub const ACTCMPBD_M = usize(0x00000C00); // Action for Comparator B Down
    pub const ACTCMPBD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBD_INV = usize(0x00000400); // Invert pwmB
    pub const ACTCMPBD_ZERO = usize(0x00000800); // Drive pwmB Low
    pub const ACTCMPBD_ONE = usize(0x00000C00); // Drive pwmB High
    pub const ACTCMPBU_M = usize(0x00000300); // Action for Comparator B Up
    pub const ACTCMPBU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPBU_INV = usize(0x00000100); // Invert pwmB
    pub const ACTCMPBU_ZERO = usize(0x00000200); // Drive pwmB Low
    pub const ACTCMPBU_ONE = usize(0x00000300); // Drive pwmB High
    pub const ACTCMPAD_M = usize(0x000000C0); // Action for Comparator A Down
    pub const ACTCMPAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAD_INV = usize(0x00000040); // Invert pwmB
    pub const ACTCMPAD_ZERO = usize(0x00000080); // Drive pwmB Low
    pub const ACTCMPAD_ONE = usize(0x000000C0); // Drive pwmB High
    pub const ACTCMPAU_M = usize(0x00000030); // Action for Comparator A Up
    pub const ACTCMPAU_NONE = usize(0x00000000); // Do nothing
    pub const ACTCMPAU_INV = usize(0x00000010); // Invert pwmB
    pub const ACTCMPAU_ZERO = usize(0x00000020); // Drive pwmB Low
    pub const ACTCMPAU_ONE = usize(0x00000030); // Drive pwmB High
    pub const ACTLOAD_M = usize(0x0000000C); // Action for Counter=LOAD
    pub const ACTLOAD_NONE = usize(0x00000000); // Do nothing
    pub const ACTLOAD_INV = usize(0x00000004); // Invert pwmB
    pub const ACTLOAD_ZERO = usize(0x00000008); // Drive pwmB Low
    pub const ACTLOAD_ONE = usize(0x0000000C); // Drive pwmB High
    pub const ACTZERO_M = usize(0x00000003); // Action for Counter=0
    pub const ACTZERO_NONE = usize(0x00000000); // Do nothing
    pub const ACTZERO_INV = usize(0x00000001); // Invert pwmB
    pub const ACTZERO_ZERO = usize(0x00000002); // Drive pwmB Low
    pub const ACTZERO_ONE = usize(0x00000003); // Drive pwmB High
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBCTL register.
//
//*****************************************************************************
pub const PWM3_DBCTL = struct {
    pub const ENABLE = usize(0x00000001); // Dead-Band Generator Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBRISE register.
//
//*****************************************************************************
pub const PWM3_DBRISE = struct {
    pub const RISEDELAY_M = usize(0x00000FFF); // Dead-Band Rise Delay
    pub const RISEDELAY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBFALL register.
//
//*****************************************************************************
pub const PWM3_DBFALL = struct {
    pub const FALLDELAY_M = usize(0x00000FFF); // Dead-Band Fall Delay
    pub const FALLDELAY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM3_FLTSRC0 = struct {
    pub const FAULT3 = usize(0x00000008); // Fault3 Input
    pub const FAULT2 = usize(0x00000004); // Fault2 Input
    pub const FAULT1 = usize(0x00000002); // Fault1 Input
    pub const FAULT0 = usize(0x00000001); // Fault0 Input
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM3_FLTSRC1 = struct {
    pub const DCMP7 = usize(0x00000080); // Digital Comparator 7
    pub const DCMP6 = usize(0x00000040); // Digital Comparator 6
    pub const DCMP5 = usize(0x00000020); // Digital Comparator 5
    pub const DCMP4 = usize(0x00000010); // Digital Comparator 4
    pub const DCMP3 = usize(0x00000008); // Digital Comparator 3
    pub const DCMP2 = usize(0x00000004); // Digital Comparator 2
    pub const DCMP1 = usize(0x00000002); // Digital Comparator 1
    pub const DCMP0 = usize(0x00000001); // Digital Comparator 0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM3_MINFLTPER = struct {
    pub const MFP_M = usize(0x0000FFFF); // Minimum Fault Period
    pub const MFP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSEN register.
//
//*****************************************************************************
pub const PWM0_FLTSEN = struct {
    pub const FAULT3 = usize(0x00000008); // Fault3 Sense
    pub const FAULT2 = usize(0x00000004); // Fault2 Sense
    pub const FAULT1 = usize(0x00000002); // Fault1 Sense
    pub const FAULT0 = usize(0x00000001); // Fault0 Sense
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM0_FLTSTAT0 = struct {
    pub const FAULT3 = usize(0x00000008); // Fault Input 3
    pub const FAULT2 = usize(0x00000004); // Fault Input 2
    pub const FAULT1 = usize(0x00000002); // Fault Input 1
    pub const FAULT0 = usize(0x00000001); // Fault Input 0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM0_FLTSTAT1 = struct {
    pub const DCMP7 = usize(0x00000080); // Digital Comparator 7 Trigger
    pub const DCMP6 = usize(0x00000040); // Digital Comparator 6 Trigger
    pub const DCMP5 = usize(0x00000020); // Digital Comparator 5 Trigger
    pub const DCMP4 = usize(0x00000010); // Digital Comparator 4 Trigger
    pub const DCMP3 = usize(0x00000008); // Digital Comparator 3 Trigger
    pub const DCMP2 = usize(0x00000004); // Digital Comparator 2 Trigger
    pub const DCMP1 = usize(0x00000002); // Digital Comparator 1 Trigger
    pub const DCMP0 = usize(0x00000001); // Digital Comparator 0 Trigger
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSEN register.
//
//*****************************************************************************
pub const PWM1_FLTSEN = struct {
    pub const FAULT3 = usize(0x00000008); // Fault3 Sense
    pub const FAULT2 = usize(0x00000004); // Fault2 Sense
    pub const FAULT1 = usize(0x00000002); // Fault1 Sense
    pub const FAULT0 = usize(0x00000001); // Fault0 Sense
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM1_FLTSTAT0 = struct {
    pub const FAULT3 = usize(0x00000008); // Fault Input 3
    pub const FAULT2 = usize(0x00000004); // Fault Input 2
    pub const FAULT1 = usize(0x00000002); // Fault Input 1
    pub const FAULT0 = usize(0x00000001); // Fault Input 0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM1_FLTSTAT1 = struct {
    pub const DCMP7 = usize(0x00000080); // Digital Comparator 7 Trigger
    pub const DCMP6 = usize(0x00000040); // Digital Comparator 6 Trigger
    pub const DCMP5 = usize(0x00000020); // Digital Comparator 5 Trigger
    pub const DCMP4 = usize(0x00000010); // Digital Comparator 4 Trigger
    pub const DCMP3 = usize(0x00000008); // Digital Comparator 3 Trigger
    pub const DCMP2 = usize(0x00000004); // Digital Comparator 2 Trigger
    pub const DCMP1 = usize(0x00000002); // Digital Comparator 1 Trigger
    pub const DCMP0 = usize(0x00000001); // Digital Comparator 0 Trigger
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSEN register.
//
//*****************************************************************************
pub const PWM2_FLTSEN = struct {
    pub const FAULT3 = usize(0x00000008); // Fault3 Sense
    pub const FAULT2 = usize(0x00000004); // Fault2 Sense
    pub const FAULT1 = usize(0x00000002); // Fault1 Sense
    pub const FAULT0 = usize(0x00000001); // Fault0 Sense
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM2_FLTSTAT0 = struct {
    pub const FAULT3 = usize(0x00000008); // Fault Input 3
    pub const FAULT2 = usize(0x00000004); // Fault Input 2
    pub const FAULT1 = usize(0x00000002); // Fault Input 1
    pub const FAULT0 = usize(0x00000001); // Fault Input 0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM2_FLTSTAT1 = struct {
    pub const DCMP7 = usize(0x00000080); // Digital Comparator 7 Trigger
    pub const DCMP6 = usize(0x00000040); // Digital Comparator 6 Trigger
    pub const DCMP5 = usize(0x00000020); // Digital Comparator 5 Trigger
    pub const DCMP4 = usize(0x00000010); // Digital Comparator 4 Trigger
    pub const DCMP3 = usize(0x00000008); // Digital Comparator 3 Trigger
    pub const DCMP2 = usize(0x00000004); // Digital Comparator 2 Trigger
    pub const DCMP1 = usize(0x00000002); // Digital Comparator 1 Trigger
    pub const DCMP0 = usize(0x00000001); // Digital Comparator 0 Trigger
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSEN register.
//
//*****************************************************************************
pub const PWM3_FLTSEN = struct {
    pub const FAULT3 = usize(0x00000008); // Fault3 Sense
    pub const FAULT2 = usize(0x00000004); // Fault2 Sense
    pub const FAULT1 = usize(0x00000002); // Fault1 Sense
    pub const FAULT0 = usize(0x00000001); // Fault0 Sense
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM3_FLTSTAT0 = struct {
    pub const FAULT3 = usize(0x00000008); // Fault Input 3
    pub const FAULT2 = usize(0x00000004); // Fault Input 2
    pub const FAULT1 = usize(0x00000002); // Fault Input 1
    pub const FAULT0 = usize(0x00000001); // Fault Input 0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM3_FLTSTAT1 = struct {
    pub const DCMP7 = usize(0x00000080); // Digital Comparator 7 Trigger
    pub const DCMP6 = usize(0x00000040); // Digital Comparator 6 Trigger
    pub const DCMP5 = usize(0x00000020); // Digital Comparator 5 Trigger
    pub const DCMP4 = usize(0x00000010); // Digital Comparator 4 Trigger
    pub const DCMP3 = usize(0x00000008); // Digital Comparator 3 Trigger
    pub const DCMP2 = usize(0x00000004); // Digital Comparator 2 Trigger
    pub const DCMP1 = usize(0x00000002); // Digital Comparator 1 Trigger
    pub const DCMP0 = usize(0x00000001); // Digital Comparator 0 Trigger
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_PP register.
//
//*****************************************************************************
pub const PP = struct {
    pub const GCNT_M = usize(0x0000000F); // Generators
    pub const FCNT_M = usize(0x000000F0); // Fault Inputs (per PWM unit)
    pub const ESYNC = usize(0x00000100); // Extended Synchronization
    pub const EFAULT = usize(0x00000200); // Extended Fault
    pub const ONE = usize(0x00000400); // One-Shot Mode
    pub const GCNT_S = usize(0);
    pub const FCNT_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_CC register.
//
//*****************************************************************************
pub const CC = struct {
    pub const USEPWM = usize(0x00000100); // Use PWM Clock Divisor
    pub const PWMDIV_M = usize(0x00000007); // PWM Clock Divider
    pub const PWMDIV_2 = usize(0x00000000); // /2
    pub const PWMDIV_4 = usize(0x00000001); // /4
    pub const PWMDIV_8 = usize(0x00000002); // /8
    pub const PWMDIV_16 = usize(0x00000003); // /16
    pub const PWMDIV_32 = usize(0x00000004); // /32
    pub const PWMDIV_64 = usize(0x00000005); // /64
};

pub const PWMX = struct {
    //*****************************************************************************
    //
    // The following are defines for the PWM Generator standard offsets.
    //
    //*****************************************************************************
    pub const offsetOf = struct {
        pub const CTL = usize(0x00000000); // Gen Control Reg
        pub const INTEN = usize(0x00000004); // Gen Int/Trig Enable Reg
        pub const RIS = usize(0x00000008); // Gen Raw Int Status Reg
        pub const ISC = usize(0x0000000C); // Gen Int Status Reg
        pub const LOAD = usize(0x00000010); // Gen Load Reg
        pub const COUNT = usize(0x00000014); // Gen Counter Reg
        pub const CMPA = usize(0x00000018); // Gen Compare A Reg
        pub const CMPB = usize(0x0000001C); // Gen Compare B Reg
        pub const GENA = usize(0x00000020); // Gen Generator A Ctrl Reg
        pub const GENB = usize(0x00000024); // Gen Generator B Ctrl Reg
        pub const DBCTL = usize(0x00000028); // Gen Dead Band Ctrl Reg
        pub const DBRISE = usize(0x0000002C); // Gen DB Rising Edge Delay Reg
        pub const DBFALL = usize(0x00000030); // Gen DB Falling Edge Delay Reg
        pub const FLTSRC0 = usize(0x00000034); // Fault pin, comparator condition
        pub const FLTSRC1 = usize(0x00000038); // Digital comparator condition
        pub const MINFLTPER = usize(0x0000003C); // Fault minimum period extension
        pub const GEN_0_OFFSET = usize(0x00000040); // PWM0 base
        pub const GEN_1_OFFSET = usize(0x00000080); // PWM1 base
        pub const GEN_2_OFFSET = usize(0x000000C0); // PWM2 base
        pub const GEN_3_OFFSET = usize(0x00000100); // PWM3 base

        //*****************************************************************************
        //
        // The following are defines for the PWM Generator extended offsets.
        //
        //*****************************************************************************
        pub const FLTSEN = usize(0x00000000); // Fault logic sense
        pub const FLTSTAT0 = usize(0x00000004); // Pin and comparator status
        pub const FLTSTAT1 = usize(0x00000008); // Digital comparator status
        pub const EXT_0_OFFSET = usize(0x00000800); // PWM0 extended base
        pub const EXT_1_OFFSET = usize(0x00000880); // PWM1 extended base
        pub const EXT_2_OFFSET = usize(0x00000900); // PWM2 extended base
        pub const EXT_3_OFFSET = usize(0x00000980); // PWM3 extended base
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_CTL register.
    //
    //*****************************************************************************
    pub const CTL = struct {
        pub const LATCH = usize(0x00040000); // Latch Fault Input
        pub const MINFLTPER = usize(0x00020000); // Minimum Fault Period
        pub const FLTSRC = usize(0x00010000); // Fault Condition Source
        pub const DBFALLUPD_M = usize(0x0000C000); // PWMnDBFALL Update Mode
        pub const DBFALLUPD_I = usize(0x00000000); // Immediate
        pub const DBFALLUPD_LS = usize(0x00008000); // Locally Synchronized
        pub const DBFALLUPD_GS = usize(0x0000C000); // Globally Synchronized
        pub const DBRISEUPD_M = usize(0x00003000); // PWMnDBRISE Update Mode
        pub const DBRISEUPD_I = usize(0x00000000); // Immediate
        pub const DBRISEUPD_LS = usize(0x00002000); // Locally Synchronized
        pub const DBRISEUPD_GS = usize(0x00003000); // Globally Synchronized
        pub const DBCTLUPD_M = usize(0x00000C00); // PWMnDBCTL Update Mode
        pub const DBCTLUPD_I = usize(0x00000000); // Immediate
        pub const DBCTLUPD_LS = usize(0x00000800); // Locally Synchronized
        pub const DBCTLUPD_GS = usize(0x00000C00); // Globally Synchronized
        pub const GENBUPD_M = usize(0x00000300); // PWMnGENB Update Mode
        pub const GENBUPD_I = usize(0x00000000); // Immediate
        pub const GENBUPD_LS = usize(0x00000200); // Locally Synchronized
        pub const GENBUPD_GS = usize(0x00000300); // Globally Synchronized
        pub const GENAUPD_M = usize(0x000000C0); // PWMnGENA Update Mode
        pub const GENAUPD_I = usize(0x00000000); // Immediate
        pub const GENAUPD_LS = usize(0x00000080); // Locally Synchronized
        pub const GENAUPD_GS = usize(0x000000C0); // Globally Synchronized
        pub const CMPBUPD = usize(0x00000020); // Comparator B Update Mode
        pub const CMPAUPD = usize(0x00000010); // Comparator A Update Mode
        pub const LOADUPD = usize(0x00000008); // Load Register Update Mode
        pub const DEBUG = usize(0x00000004); // Debug Mode
        pub const MODE = usize(0x00000002); // Counter Mode
        pub const ENABLE = usize(0x00000001); // PWM Block Enable
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_INTEN register.
    //
    //*****************************************************************************
    pub const INTEN = struct {
        pub const TRCMPBD = usize(0x00002000); // Trigger for Counter=PWMnCMPB
        // Down
        pub const TRCMPBU = usize(0x00001000); // Trigger for Counter=PWMnCMPB Up
        pub const TRCMPAD = usize(0x00000800); // Trigger for Counter=PWMnCMPA
        // Down
        pub const TRCMPAU = usize(0x00000400); // Trigger for Counter=PWMnCMPA Up
        pub const TRCNTLOAD = usize(0x00000200); // Trigger for Counter=PWMnLOAD
        pub const TRCNTZERO = usize(0x00000100); // Trigger for Counter=0
        pub const INTCMPBD = usize(0x00000020); // Interrupt for Counter=PWMnCMPB
        // Down
        pub const INTCMPBU = usize(0x00000010); // Interrupt for Counter=PWMnCMPB
        // Up
        pub const INTCMPAD = usize(0x00000008); // Interrupt for Counter=PWMnCMPA
        // Down
        pub const INTCMPAU = usize(0x00000004); // Interrupt for Counter=PWMnCMPA
        // Up
        pub const INTCNTLOAD = usize(0x00000002); // Interrupt for Counter=PWMnLOAD
        pub const INTCNTZERO = usize(0x00000001); // Interrupt for Counter=0
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_RIS register.
    //
    //*****************************************************************************
    pub const RIS = struct {
        pub const INTCMPBD = usize(0x00000020); // Comparator B Down Interrupt
        // Status
        pub const INTCMPBU = usize(0x00000010); // Comparator B Up Interrupt Status
        pub const INTCMPAD = usize(0x00000008); // Comparator A Down Interrupt
        // Status
        pub const INTCMPAU = usize(0x00000004); // Comparator A Up Interrupt Status
        pub const INTCNTLOAD = usize(0x00000002); // Counter=Load Interrupt Status
        pub const INTCNTZERO = usize(0x00000001); // Counter=0 Interrupt Status
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_ISC register.
    //
    //*****************************************************************************
    pub const ISC = struct {
        pub const INTCMPBD = usize(0x00000020); // Comparator B Down Interrupt
        pub const INTCMPBU = usize(0x00000010); // Comparator B Up Interrupt
        pub const INTCMPAD = usize(0x00000008); // Comparator A Down Interrupt
        pub const INTCMPAU = usize(0x00000004); // Comparator A Up Interrupt
        pub const INTCNTLOAD = usize(0x00000002); // Counter=Load Interrupt
        pub const INTCNTZERO = usize(0x00000001); // Counter=0 Interrupt
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_LOAD register.
    //
    //*****************************************************************************
    pub const LOAD = struct {
        pub const M = usize(0x0000FFFF); // Counter Load Value
        pub const S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_COUNT register.
    //
    //*****************************************************************************
    pub const COUNT = struct {
        pub const M = usize(0x0000FFFF); // Counter Value
        pub const S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_CMPA register.
    //
    //*****************************************************************************
    pub const CMPA = struct {
        pub const M = usize(0x0000FFFF); // Comparator A Value
        pub const S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_CMPB register.
    //
    //*****************************************************************************
    pub const CMPB = struct {
        pub const M = usize(0x0000FFFF); // Comparator B Value
        pub const S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_GENA register.
    //
    //*****************************************************************************
    pub const GENA = struct {
        pub const ACTCMPBD_M = usize(0x00000C00); // Action for Comparator B Down
        pub const ACTCMPBD_NONE = usize(0x00000000); // Do nothing
        pub const ACTCMPBD_INV = usize(0x00000400); // Invert pwmA
        pub const ACTCMPBD_ZERO = usize(0x00000800); // Drive pwmA Low
        pub const ACTCMPBD_ONE = usize(0x00000C00); // Drive pwmA High
        pub const ACTCMPBU_M = usize(0x00000300); // Action for Comparator B Up
        pub const ACTCMPBU_NONE = usize(0x00000000); // Do nothing
        pub const ACTCMPBU_INV = usize(0x00000100); // Invert pwmA
        pub const ACTCMPBU_ZERO = usize(0x00000200); // Drive pwmA Low
        pub const ACTCMPBU_ONE = usize(0x00000300); // Drive pwmA High
        pub const ACTCMPAD_M = usize(0x000000C0); // Action for Comparator A Down
        pub const ACTCMPAD_NONE = usize(0x00000000); // Do nothing
        pub const ACTCMPAD_INV = usize(0x00000040); // Invert pwmA
        pub const ACTCMPAD_ZERO = usize(0x00000080); // Drive pwmA Low
        pub const ACTCMPAD_ONE = usize(0x000000C0); // Drive pwmA High
        pub const ACTCMPAU_M = usize(0x00000030); // Action for Comparator A Up
        pub const ACTCMPAU_NONE = usize(0x00000000); // Do nothing
        pub const ACTCMPAU_INV = usize(0x00000010); // Invert pwmA
        pub const ACTCMPAU_ZERO = usize(0x00000020); // Drive pwmA Low
        pub const ACTCMPAU_ONE = usize(0x00000030); // Drive pwmA High
        pub const ACTLOAD_M = usize(0x0000000C); // Action for Counter=LOAD
        pub const ACTLOAD_NONE = usize(0x00000000); // Do nothing
        pub const ACTLOAD_INV = usize(0x00000004); // Invert pwmA
        pub const ACTLOAD_ZERO = usize(0x00000008); // Drive pwmA Low
        pub const ACTLOAD_ONE = usize(0x0000000C); // Drive pwmA High
        pub const ACTZERO_M = usize(0x00000003); // Action for Counter=0
        pub const ACTZERO_NONE = usize(0x00000000); // Do nothing
        pub const ACTZERO_INV = usize(0x00000001); // Invert pwmA
        pub const ACTZERO_ZERO = usize(0x00000002); // Drive pwmA Low
        pub const ACTZERO_ONE = usize(0x00000003); // Drive pwmA High
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_GENB register.
    //
    //*****************************************************************************
    pub const GENB = struct {
        pub const ACTCMPBD_M = usize(0x00000C00); // Action for Comparator B Down
        pub const ACTCMPBD_NONE = usize(0x00000000); // Do nothing
        pub const ACTCMPBD_INV = usize(0x00000400); // Invert pwmB
        pub const ACTCMPBD_ZERO = usize(0x00000800); // Drive pwmB Low
        pub const ACTCMPBD_ONE = usize(0x00000C00); // Drive pwmB High
        pub const ACTCMPBU_M = usize(0x00000300); // Action for Comparator B Up
        pub const ACTCMPBU_NONE = usize(0x00000000); // Do nothing
        pub const ACTCMPBU_INV = usize(0x00000100); // Invert pwmB
        pub const ACTCMPBU_ZERO = usize(0x00000200); // Drive pwmB Low
        pub const ACTCMPBU_ONE = usize(0x00000300); // Drive pwmB High
        pub const ACTCMPAD_M = usize(0x000000C0); // Action for Comparator A Down
        pub const ACTCMPAD_NONE = usize(0x00000000); // Do nothing
        pub const ACTCMPAD_INV = usize(0x00000040); // Invert pwmB
        pub const ACTCMPAD_ZERO = usize(0x00000080); // Drive pwmB Low
        pub const ACTCMPAD_ONE = usize(0x000000C0); // Drive pwmB High
        pub const ACTCMPAU_M = usize(0x00000030); // Action for Comparator A Up
        pub const ACTCMPAU_NONE = usize(0x00000000); // Do nothing
        pub const ACTCMPAU_INV = usize(0x00000010); // Invert pwmB
        pub const ACTCMPAU_ZERO = usize(0x00000020); // Drive pwmB Low
        pub const ACTCMPAU_ONE = usize(0x00000030); // Drive pwmB High
        pub const ACTLOAD_M = usize(0x0000000C); // Action for Counter=LOAD
        pub const ACTLOAD_NONE = usize(0x00000000); // Do nothing
        pub const ACTLOAD_INV = usize(0x00000004); // Invert pwmB
        pub const ACTLOAD_ZERO = usize(0x00000008); // Drive pwmB Low
        pub const ACTLOAD_ONE = usize(0x0000000C); // Drive pwmB High
        pub const ACTZERO_M = usize(0x00000003); // Action for Counter=0
        pub const ACTZERO_NONE = usize(0x00000000); // Do nothing
        pub const ACTZERO_INV = usize(0x00000001); // Invert pwmB
        pub const ACTZERO_ZERO = usize(0x00000002); // Drive pwmB Low
        pub const ACTZERO_ONE = usize(0x00000003); // Drive pwmB High
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_DBCTL register.
    //
    //*****************************************************************************
    pub const DBCTL = struct {
        pub const ENABLE = usize(0x00000001); // Dead-Band Generator Enable
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_DBRISE register.
    //
    //*****************************************************************************
    pub const DBRISE = struct {
        pub const DELAY_M = usize(0x00000FFF); // Dead-Band Rise Delay
        pub const DELAY_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_DBFALL register.
    //
    //*****************************************************************************
    pub const DBFALL = struct {
        pub const DELAY_M = usize(0x00000FFF); // Dead-Band Fall Delay
        pub const DELAY_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_FLTSRC0
    // register.
    //
    //*****************************************************************************
    pub const FLTSRC0 = struct {
        pub const FAULT3 = usize(0x00000008); // Fault3 Input
        pub const FAULT2 = usize(0x00000004); // Fault2 Input
        pub const FAULT1 = usize(0x00000002); // Fault1 Input
        pub const FAULT0 = usize(0x00000001); // Fault0 Input
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_FLTSRC1
    // register.
    //
    //*****************************************************************************
    pub const FLTSRC1 = struct {
        pub const DCMP7 = usize(0x00000080); // Digital Comparator 7
        pub const DCMP6 = usize(0x00000040); // Digital Comparator 6
        pub const DCMP5 = usize(0x00000020); // Digital Comparator 5
        pub const DCMP4 = usize(0x00000010); // Digital Comparator 4
        pub const DCMP3 = usize(0x00000008); // Digital Comparator 3
        pub const DCMP2 = usize(0x00000004); // Digital Comparator 2
        pub const DCMP1 = usize(0x00000002); // Digital Comparator 1
        pub const DCMP0 = usize(0x00000001); // Digital Comparator 0
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_MINFLTPER
    // register.
    //
    //*****************************************************************************
    pub const MINFLTPER = struct {
        pub const M = usize(0x0000FFFF); // Minimum Fault Period
        pub const S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_FLTSEN register.
    //
    //*****************************************************************************
    pub const FLTSEN = struct {
        pub const FAULT3 = usize(0x00000008); // Fault3 Sense
        pub const FAULT2 = usize(0x00000004); // Fault2 Sense
        pub const FAULT1 = usize(0x00000002); // Fault1 Sense
        pub const FAULT0 = usize(0x00000001); // Fault0 Sense
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_FLTSTAT0
    // register.
    //
    //*****************************************************************************
    pub const FLTSTAT0 = struct {
        pub const FAULT3 = usize(0x00000008); // Fault Input 3
        pub const FAULT2 = usize(0x00000004); // Fault Input 2
        pub const FAULT1 = usize(0x00000002); // Fault Input 1
        pub const FAULT0 = usize(0x00000001); // Fault Input 0
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the PWM_O_X_FLTSTAT1
    // register.
    //
    //*****************************************************************************
    pub const FLTSTAT1 = struct {
        pub const DCMP7 = usize(0x00000080); // Digital Comparator 7 Trigger
        pub const DCMP6 = usize(0x00000040); // Digital Comparator 6 Trigger
        pub const DCMP5 = usize(0x00000020); // Digital Comparator 5 Trigger
        pub const DCMP4 = usize(0x00000010); // Digital Comparator 4 Trigger
        pub const DCMP3 = usize(0x00000008); // Digital Comparator 3 Trigger
        pub const DCMP2 = usize(0x00000004); // Digital Comparator 2 Trigger
        pub const DCMP1 = usize(0x00000002); // Digital Comparator 1 Trigger
        pub const DCMP0 = usize(0x00000001); // Digital Comparator 0 Trigger
    };
};
