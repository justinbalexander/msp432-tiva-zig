//*****************************************************************************
//
// hw_timer.h - Defines and macros used when accessing the timer.
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
// The following are defines for the Timer register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const CFG = usize(0x00000000); // GPTM Configuration
    pub const TAMR = usize(0x00000004); // GPTM Timer A Mode
    pub const TBMR = usize(0x00000008); // GPTM Timer B Mode
    pub const CTL = usize(0x0000000C); // GPTM Control
    pub const SYNC = usize(0x00000010); // GPTM Synchronize
    pub const IMR = usize(0x00000018); // GPTM Interrupt Mask
    pub const RIS = usize(0x0000001C); // GPTM Raw Interrupt Status
    pub const MIS = usize(0x00000020); // GPTM Masked Interrupt Status
    pub const ICR = usize(0x00000024); // GPTM Interrupt Clear
    pub const TAILR = usize(0x00000028); // GPTM Timer A Interval Load
    pub const TBILR = usize(0x0000002C); // GPTM Timer B Interval Load
    pub const TAMATCHR = usize(0x00000030); // GPTM Timer A Match
    pub const TBMATCHR = usize(0x00000034); // GPTM Timer B Match
    pub const TAPR = usize(0x00000038); // GPTM Timer A Prescale
    pub const TBPR = usize(0x0000003C); // GPTM Timer B Prescale
    pub const TAPMR = usize(0x00000040); // GPTM TimerA Prescale Match
    pub const TBPMR = usize(0x00000044); // GPTM TimerB Prescale Match
    pub const TAR = usize(0x00000048); // GPTM Timer A
    pub const TBR = usize(0x0000004C); // GPTM Timer B
    pub const TAV = usize(0x00000050); // GPTM Timer A Value
    pub const TBV = usize(0x00000054); // GPTM Timer B Value
    pub const RTCPD = usize(0x00000058); // GPTM RTC Predivide
    pub const TAPS = usize(0x0000005C); // GPTM Timer A Prescale Snapshot
    pub const TBPS = usize(0x00000060); // GPTM Timer B Prescale Snapshot
    pub const TAPV = usize(0x00000064); // GPTM Timer A Prescale Value
    pub const TBPV = usize(0x00000068); // GPTM Timer B Prescale Value
    pub const DMAEV = usize(0x0000006C); // GPTM DMA Event
    pub const ADCEV = usize(0x00000070); // GPTM ADC Event
    pub const PP = usize(0x00000FC0); // GPTM Peripheral Properties
    pub const CC = usize(0x00000FC8); // GPTM Clock Configuration
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CFG register.
//
//*****************************************************************************
pub const CFG = struct {
    pub const M = usize(0x00000007); // GPTM Configuration
    pub const BIT_32_TIMER = usize(0x00000000); // For a 16/32-bit timer, this
    // value selects the 32-bit timer
    // configuration
    pub const BIT_32_RTC = usize(0x00000001); // For a 16/32-bit timer, this
    // value selects the 32-bit
    // real-time clock (RTC) counter
    // configuration
    pub const BIT_16 = usize(0x00000004); // For a 16/32-bit timer, this
    // value selects the 16-bit timer
    // configuration
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMR register.
//
//*****************************************************************************
pub const TAMR = struct {
    pub const TCACT_M = usize(0x0000E000); // Timer Compare Action Select
    pub const TCACT_NONE = usize(0x00000000); // Disable compare operations
    pub const TCACT_TOGGLE = usize(0x00002000); // Toggle State on Time-Out
    pub const TCACT_CLRTO = usize(0x00004000); // Clear CCP on Time-Out
    pub const TCACT_SETTO = usize(0x00006000); // Set CCP on Time-Out
    pub const TCACT_SETTOGTO = usize(0x00008000); // Set CCP immediately and toggle
    // on Time-Out
    pub const TCACT_CLRTOGTO = usize(0x0000A000); // Clear CCP immediately and toggle
    // on Time-Out
    pub const TCACT_SETCLRTO = usize(0x0000C000); // Set CCP immediately and clear on
    // Time-Out
    pub const TCACT_CLRSETTO = usize(0x0000E000); // Clear CCP immediately and set on
    // Time-Out
    pub const TACINTD = usize(0x00001000); // One-shot/Periodic Interrupt
    // Disable
    pub const TAPLO = usize(0x00000800); // GPTM Timer A PWM Legacy
    // Operation
    pub const TAMRSU = usize(0x00000400); // GPTM Timer A Match Register
    // Update
    pub const TAPWMIE = usize(0x00000200); // GPTM Timer A PWM Interrupt
    // Enable
    pub const TAILD = usize(0x00000100); // GPTM Timer A Interval Load Write
    pub const TASNAPS = usize(0x00000080); // GPTM Timer A Snap-Shot Mode
    pub const TAWOT = usize(0x00000040); // GPTM Timer A Wait-on-Trigger
    pub const TAMIE = usize(0x00000020); // GPTM Timer A Match Interrupt
    // Enable
    pub const TACDIR = usize(0x00000010); // GPTM Timer A Count Direction
    pub const TAAMS = usize(0x00000008); // GPTM Timer A Alternate Mode
    // Select
    pub const TACMR = usize(0x00000004); // GPTM Timer A Capture Mode
    pub const TAMR_M = usize(0x00000003); // GPTM Timer A Mode
    pub const TAMR_1_SHOT = usize(0x00000001); // One-Shot Timer mode
    pub const TAMR_PERIOD = usize(0x00000002); // Periodic Timer mode
    pub const TAMR_CAP = usize(0x00000003); // Capture mode
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMR register.
//
//*****************************************************************************
pub const TBMR = struct {
    pub const TCACT_M = usize(0x0000E000); // Timer Compare Action Select
    pub const TCACT_NONE = usize(0x00000000); // Disable compare operations
    pub const TCACT_TOGGLE = usize(0x00002000); // Toggle State on Time-Out
    pub const TCACT_CLRTO = usize(0x00004000); // Clear CCP on Time-Out
    pub const TCACT_SETTO = usize(0x00006000); // Set CCP on Time-Out
    pub const TCACT_SETTOGTO = usize(0x00008000); // Set CCP immediately and toggle
    // on Time-Out
    pub const TCACT_CLRTOGTO = usize(0x0000A000); // Clear CCP immediately and toggle
    // on Time-Out
    pub const TCACT_SETCLRTO = usize(0x0000C000); // Set CCP immediately and clear on
    // Time-Out
    pub const TCACT_CLRSETTO = usize(0x0000E000); // Clear CCP immediately and set on
    // Time-Out
    pub const TBCINTD = usize(0x00001000); // One-Shot/Periodic Interrupt
    // Disable
    pub const TBPLO = usize(0x00000800); // GPTM Timer B PWM Legacy
    // Operation
    pub const TBMRSU = usize(0x00000400); // GPTM Timer B Match Register
    // Update
    pub const TBPWMIE = usize(0x00000200); // GPTM Timer B PWM Interrupt
    // Enable
    pub const TBILD = usize(0x00000100); // GPTM Timer B Interval Load Write
    pub const TBSNAPS = usize(0x00000080); // GPTM Timer B Snap-Shot Mode
    pub const TBWOT = usize(0x00000040); // GPTM Timer B Wait-on-Trigger
    pub const TBMIE = usize(0x00000020); // GPTM Timer B Match Interrupt
    // Enable
    pub const TBCDIR = usize(0x00000010); // GPTM Timer B Count Direction
    pub const TBAMS = usize(0x00000008); // GPTM Timer B Alternate Mode
    // Select
    pub const TBCMR = usize(0x00000004); // GPTM Timer B Capture Mode
    pub const TBMR_M = usize(0x00000003); // GPTM Timer B Mode
    pub const TBMR_1_SHOT = usize(0x00000001); // One-Shot Timer mode
    pub const TBMR_PERIOD = usize(0x00000002); // Periodic Timer mode
    pub const TBMR_CAP = usize(0x00000003); // Capture mode
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CTL register.
//
//*****************************************************************************
pub const CTL = struct {
    pub const TBPWML = usize(0x00004000); // GPTM Timer B PWM Output Level
    pub const TBOTE = usize(0x00002000); // GPTM Timer B Output Trigger
    // Enable
    pub const TBEVENT_M = usize(0x00000C00); // GPTM Timer B Event Mode
    pub const TBEVENT_POS = usize(0x00000000); // Positive edge
    pub const TBEVENT_NEG = usize(0x00000400); // Negative edge
    pub const TBEVENT_BOTH = usize(0x00000C00); // Both edges
    pub const TBSTALL = usize(0x00000200); // GPTM Timer B Stall Enable
    pub const TBEN = usize(0x00000100); // GPTM Timer B Enable
    pub const TAPWML = usize(0x00000040); // GPTM Timer A PWM Output Level
    pub const TAOTE = usize(0x00000020); // GPTM Timer A Output Trigger
    // Enable
    pub const RTCEN = usize(0x00000010); // GPTM RTC Stall Enable
    pub const TAEVENT_M = usize(0x0000000C); // GPTM Timer A Event Mode
    pub const TAEVENT_POS = usize(0x00000000); // Positive edge
    pub const TAEVENT_NEG = usize(0x00000004); // Negative edge
    pub const TAEVENT_BOTH = usize(0x0000000C); // Both edges
    pub const TASTALL = usize(0x00000002); // GPTM Timer A Stall Enable
    pub const TAEN = usize(0x00000001); // GPTM Timer A Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_SYNC register.
//
//*****************************************************************************
pub const SYNC = struct {
    pub const SYNCWT5_M = usize(0x00C00000); // Synchronize GPTM 32/64-Bit Timer
    // 5
    pub const SYNCWT5_NONE = usize(0x00000000); // GPTM 32/64-Bit Timer 5 is not
    // affected
    pub const SYNCWT5_TA = usize(0x00400000); // A timeout event for Timer A of
    // GPTM 32/64-Bit Timer 5 is
    // triggered
    pub const SYNCWT5_TB = usize(0x00800000); // A timeout event for Timer B of
    // GPTM 32/64-Bit Timer 5 is
    // triggered
    pub const SYNCWT5_TATB = usize(0x00C00000); // A timeout event for both Timer A
    // and Timer B of GPTM 32/64-Bit
    // Timer 5 is triggered
    pub const SYNCWT4_M = usize(0x00300000); // Synchronize GPTM 32/64-Bit Timer
    // 4
    pub const SYNCWT4_NONE = usize(0x00000000); // GPTM 32/64-Bit Timer 4 is not
    // affected
    pub const SYNCWT4_TA = usize(0x00100000); // A timeout event for Timer A of
    // GPTM 32/64-Bit Timer 4 is
    // triggered
    pub const SYNCWT4_TB = usize(0x00200000); // A timeout event for Timer B of
    // GPTM 32/64-Bit Timer 4 is
    // triggered
    pub const SYNCWT4_TATB = usize(0x00300000); // A timeout event for both Timer A
    // and Timer B of GPTM 32/64-Bit
    // Timer 4 is triggered
    pub const SYNCWT3_M = usize(0x000C0000); // Synchronize GPTM 32/64-Bit Timer
    // 3
    pub const SYNCWT3_NONE = usize(0x00000000); // GPTM 32/64-Bit Timer 3 is not
    // affected
    pub const SYNCWT3_TA = usize(0x00040000); // A timeout event for Timer A of
    // GPTM 32/64-Bit Timer 3 is
    // triggered
    pub const SYNCWT3_TB = usize(0x00080000); // A timeout event for Timer B of
    // GPTM 32/64-Bit Timer 3 is
    // triggered
    pub const SYNCWT3_TATB = usize(0x000C0000); // A timeout event for both Timer A
    // and Timer B of GPTM 32/64-Bit
    // Timer 3 is triggered
    pub const SYNCWT2_M = usize(0x00030000); // Synchronize GPTM 32/64-Bit Timer
    // 2
    pub const SYNCWT2_NONE = usize(0x00000000); // GPTM 32/64-Bit Timer 2 is not
    // affected
    pub const SYNCWT2_TA = usize(0x00010000); // A timeout event for Timer A of
    // GPTM 32/64-Bit Timer 2 is
    // triggered
    pub const SYNCWT2_TB = usize(0x00020000); // A timeout event for Timer B of
    // GPTM 32/64-Bit Timer 2 is
    // triggered
    pub const SYNCWT2_TATB = usize(0x00030000); // A timeout event for both Timer A
    // and Timer B of GPTM 32/64-Bit
    // Timer 2 is triggered
    pub const SYNCT7_M = usize(0x0000C000); // Synchronize GPTM Timer 7
    pub const SYNCT7_NONE = usize(0x00000000); // GPT7 is not affected
    pub const SYNCT7_TA = usize(0x00004000); // A timeout event for Timer A of
    // GPTM7 is triggered
    pub const SYNCT7_TB = usize(0x00008000); // A timeout event for Timer B of
    // GPTM7 is triggered
    pub const SYNCT7_TATB = usize(0x0000C000); // A timeout event for both Timer A
    // and Timer B of GPTM7 is
    // triggered
    pub const SYNCWT1_M = usize(0x0000C000); // Synchronize GPTM 32/64-Bit Timer
    // 1
    pub const SYNCWT1_NONE = usize(0x00000000); // GPTM 32/64-Bit Timer 1 is not
    // affected
    pub const SYNCWT1_TA = usize(0x00004000); // A timeout event for Timer A of
    // GPTM 32/64-Bit Timer 1 is
    // triggered
    pub const SYNCWT1_TB = usize(0x00008000); // A timeout event for Timer B of
    // GPTM 32/64-Bit Timer 1 is
    // triggered
    pub const SYNCWT1_TATB = usize(0x0000C000); // A timeout event for both Timer A
    // and Timer B of GPTM 32/64-Bit
    // Timer 1 is triggered
    pub const SYNCWT0_M = usize(0x00003000); // Synchronize GPTM 32/64-Bit Timer
    // 0
    pub const SYNCWT0_NONE = usize(0x00000000); // GPTM 32/64-Bit Timer 0 is not
    // affected
    pub const SYNCWT0_TA = usize(0x00001000); // A timeout event for Timer A of
    // GPTM 32/64-Bit Timer 0 is
    // triggered
    pub const SYNCWT0_TB = usize(0x00002000); // A timeout event for Timer B of
    // GPTM 32/64-Bit Timer 0 is
    // triggered
    pub const SYNCWT0_TATB = usize(0x00003000); // A timeout event for both Timer A
    // and Timer B of GPTM 32/64-Bit
    // Timer 0 is triggered
    pub const SYNCT6_M = usize(0x00003000); // Synchronize GPTM Timer 6
    pub const SYNCT6_NONE = usize(0x00000000); // GPTM6 is not affected
    pub const SYNCT6_TA = usize(0x00001000); // A timeout event for Timer A of
    // GPTM6 is triggered
    pub const SYNCT6_TB = usize(0x00002000); // A timeout event for Timer B of
    // GPTM6 is triggered
    pub const SYNCT6_TATB = usize(0x00003000); // A timeout event for both Timer A
    // and Timer B of GPTM6 is
    // triggered
    pub const SYNCT5_M = usize(0x00000C00); // Synchronize GPTM Timer 5
    pub const SYNCT5_NONE = usize(0x00000000); // GPTM5 is not affected
    pub const SYNCT5_TA = usize(0x00000400); // A timeout event for Timer A of
    // GPTM5 is triggered
    pub const SYNCT5_TB = usize(0x00000800); // A timeout event for Timer B of
    // GPTM5 is triggered
    pub const SYNCT5_TATB = usize(0x00000C00); // A timeout event for both Timer A
    // and Timer B of GPTM5 is
    // triggered
    pub const SYNCT4_M = usize(0x00000300); // Synchronize GPTM Timer 4
    pub const SYNCT4_NONE = usize(0x00000000); // GPTM4 is not affected
    pub const SYNCT4_TA = usize(0x00000100); // A timeout event for Timer A of
    // GPTM4 is triggered
    pub const SYNCT4_TB = usize(0x00000200); // A timeout event for Timer B of
    // GPTM4 is triggered
    pub const SYNCT4_TATB = usize(0x00000300); // A timeout event for both Timer A
    // and Timer B of GPTM4 is
    // triggered
    pub const SYNCT3_M = usize(0x000000C0); // Synchronize GPTM Timer 3
    pub const SYNCT3_NONE = usize(0x00000000); // GPTM3 is not affected
    pub const SYNCT3_TA = usize(0x00000040); // A timeout event for Timer A of
    // GPTM3 is triggered
    pub const SYNCT3_TB = usize(0x00000080); // A timeout event for Timer B of
    // GPTM3 is triggered
    pub const SYNCT3_TATB = usize(0x000000C0); // A timeout event for both Timer A
    // and Timer B of GPTM3 is
    // triggered
    pub const SYNCT2_M = usize(0x00000030); // Synchronize GPTM Timer 2
    pub const SYNCT2_NONE = usize(0x00000000); // GPTM2 is not affected
    pub const SYNCT2_TA = usize(0x00000010); // A timeout event for Timer A of
    // GPTM2 is triggered
    pub const SYNCT2_TB = usize(0x00000020); // A timeout event for Timer B of
    // GPTM2 is triggered
    pub const SYNCT2_TATB = usize(0x00000030); // A timeout event for both Timer A
    // and Timer B of GPTM2 is
    // triggered
    pub const SYNCT1_M = usize(0x0000000C); // Synchronize GPTM Timer 1
    pub const SYNCT1_NONE = usize(0x00000000); // GPTM1 is not affected
    pub const SYNCT1_TA = usize(0x00000004); // A timeout event for Timer A of
    // GPTM1 is triggered
    pub const SYNCT1_TB = usize(0x00000008); // A timeout event for Timer B of
    // GPTM1 is triggered
    pub const SYNCT1_TATB = usize(0x0000000C); // A timeout event for both Timer A
    // and Timer B of GPTM1 is
    // triggered
    pub const SYNCT0_M = usize(0x00000003); // Synchronize GPTM Timer 0
    pub const SYNCT0_NONE = usize(0x00000000); // GPTM0 is not affected
    pub const SYNCT0_TA = usize(0x00000001); // A timeout event for Timer A of
    // GPTM0 is triggered
    pub const SYNCT0_TB = usize(0x00000002); // A timeout event for Timer B of
    // GPTM0 is triggered
    pub const SYNCT0_TATB = usize(0x00000003); // A timeout event for both Timer A
    // and Timer B of GPTM0 is
    // triggered
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_IMR register.
//
//*****************************************************************************
pub const IMR = struct {
    pub const WUEIM = usize(0x00010000); // 32/64-Bit Wide GPTM Write Update
    // Error Interrupt Mask
    pub const DMABIM = usize(0x00002000); // GPTM Timer B DMA Done Interrupt
    // Mask
    pub const TBMIM = usize(0x00000800); // GPTM Timer B Match Interrupt
    // Mask
    pub const CBEIM = usize(0x00000400); // GPTM Timer B Capture Mode Event
    // Interrupt Mask
    pub const CBMIM = usize(0x00000200); // GPTM Timer B Capture Mode Match
    // Interrupt Mask
    pub const TBTOIM = usize(0x00000100); // GPTM Timer B Time-Out Interrupt
    // Mask
    pub const DMAAIM = usize(0x00000020); // GPTM Timer A DMA Done Interrupt
    // Mask
    pub const TAMIM = usize(0x00000010); // GPTM Timer A Match Interrupt
    // Mask
    pub const RTCIM = usize(0x00000008); // GPTM RTC Interrupt Mask
    pub const CAEIM = usize(0x00000004); // GPTM Timer A Capture Mode Event
    // Interrupt Mask
    pub const CAMIM = usize(0x00000002); // GPTM Timer A Capture Mode Match
    // Interrupt Mask
    pub const TATOIM = usize(0x00000001); // GPTM Timer A Time-Out Interrupt
    // Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_RIS register.
//
//*****************************************************************************
pub const RIS = struct {
    pub const WUERIS = usize(0x00010000); // 32/64-Bit Wide GPTM Write Update
    // Error Raw Interrupt Status
    pub const DMABRIS = usize(0x00002000); // GPTM Timer B DMA Done Raw
    // Interrupt Status
    pub const TBMRIS = usize(0x00000800); // GPTM Timer B Match Raw Interrupt
    pub const CBERIS = usize(0x00000400); // GPTM Timer B Capture Mode Event
    // Raw Interrupt
    pub const CBMRIS = usize(0x00000200); // GPTM Timer B Capture Mode Match
    // Raw Interrupt
    pub const TBTORIS = usize(0x00000100); // GPTM Timer B Time-Out Raw
    // Interrupt
    pub const DMAARIS = usize(0x00000020); // GPTM Timer A DMA Done Raw
    // Interrupt Status
    pub const TAMRIS = usize(0x00000010); // GPTM Timer A Match Raw Interrupt
    pub const RTCRIS = usize(0x00000008); // GPTM RTC Raw Interrupt
    pub const CAERIS = usize(0x00000004); // GPTM Timer A Capture Mode Event
    // Raw Interrupt
    pub const CAMRIS = usize(0x00000002); // GPTM Timer A Capture Mode Match
    // Raw Interrupt
    pub const TATORIS = usize(0x00000001); // GPTM Timer A Time-Out Raw
    // Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_MIS register.
//
//*****************************************************************************
pub const MIS = struct {
    pub const WUEMIS = usize(0x00010000); // 32/64-Bit Wide GPTM Write Update
    // Error Masked Interrupt Status
    pub const DMABMIS = usize(0x00002000); // GPTM Timer B DMA Done Masked
    // Interrupt
    pub const TBMMIS = usize(0x00000800); // GPTM Timer B Match Masked
    // Interrupt
    pub const CBEMIS = usize(0x00000400); // GPTM Timer B Capture Mode Event
    // Masked Interrupt
    pub const CBMMIS = usize(0x00000200); // GPTM Timer B Capture Mode Match
    // Masked Interrupt
    pub const TBTOMIS = usize(0x00000100); // GPTM Timer B Time-Out Masked
    // Interrupt
    pub const DMAAMIS = usize(0x00000020); // GPTM Timer A DMA Done Masked
    // Interrupt
    pub const TAMMIS = usize(0x00000010); // GPTM Timer A Match Masked
    // Interrupt
    pub const RTCMIS = usize(0x00000008); // GPTM RTC Masked Interrupt
    pub const CAEMIS = usize(0x00000004); // GPTM Timer A Capture Mode Event
    // Masked Interrupt
    pub const CAMMIS = usize(0x00000002); // GPTM Timer A Capture Mode Match
    // Masked Interrupt
    pub const TATOMIS = usize(0x00000001); // GPTM Timer A Time-Out Masked
    // Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_ICR register.
//
//*****************************************************************************
pub const ICR = struct {
    pub const WUECINT = usize(0x00010000); // 32/64-Bit Wide GPTM Write Update
    // Error Interrupt Clear
    pub const DMABINT = usize(0x00002000); // GPTM Timer B DMA Done Interrupt
    // Clear
    pub const TBMCINT = usize(0x00000800); // GPTM Timer B Match Interrupt
    // Clear
    pub const CBECINT = usize(0x00000400); // GPTM Timer B Capture Mode Event
    // Interrupt Clear
    pub const CBMCINT = usize(0x00000200); // GPTM Timer B Capture Mode Match
    // Interrupt Clear
    pub const TBTOCINT = usize(0x00000100); // GPTM Timer B Time-Out Interrupt
    // Clear
    pub const DMAAINT = usize(0x00000020); // GPTM Timer A DMA Done Interrupt
    // Clear
    pub const TAMCINT = usize(0x00000010); // GPTM Timer A Match Interrupt
    // Clear
    pub const RTCCINT = usize(0x00000008); // GPTM RTC Interrupt Clear
    pub const CAECINT = usize(0x00000004); // GPTM Timer A Capture Mode Event
    // Interrupt Clear
    pub const CAMCINT = usize(0x00000002); // GPTM Timer A Capture Mode Match
    // Interrupt Clear
    pub const TATOCINT = usize(0x00000001); // GPTM Timer A Time-Out Raw
    // Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAILR register.
//
//*****************************************************************************
pub const TAILR = struct {
    pub const M = usize(0xFFFFFFFF); // GPTM Timer A Interval Load
    // Register
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBILR register.
//
//*****************************************************************************
pub const TBILR = struct {
    pub const M = usize(0xFFFFFFFF); // GPTM Timer B Interval Load
    // Register
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMATCHR
// register.
//
//*****************************************************************************
pub const TAMATCHR = struct {
    pub const TAMR_M = usize(0xFFFFFFFF); // GPTM Timer A Match Register
    pub const TAMR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMATCHR
// register.
//
//*****************************************************************************
pub const TBMATCHR = struct {
    pub const TBMR_M = usize(0xFFFFFFFF); // GPTM Timer B Match Register
    pub const TBMR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPR register.
//
//*****************************************************************************
pub const TAPR = struct {
    pub const TAPSRH_M = usize(0x0000FF00); // GPTM Timer A Prescale High Byte
    pub const TAPSR_M = usize(0x000000FF); // GPTM Timer A Prescale
    pub const TAPSRH_S = usize(8);
    pub const TAPSR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPR register.
//
//*****************************************************************************
pub const TBPR = struct {
    pub const TBPSRH_M = usize(0x0000FF00); // GPTM Timer B Prescale High Byte
    pub const TBPSR_M = usize(0x000000FF); // GPTM Timer B Prescale
    pub const TBPSRH_S = usize(8);
    pub const TBPSR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPMR register.
//
//*****************************************************************************
pub const TAPMR = struct {
    pub const TAPSMRH_M = usize(0x0000FF00); // GPTM Timer A Prescale Match High
    // Byte
    pub const TAPSMR_M = usize(0x000000FF); // GPTM TimerA Prescale Match
    pub const TAPSMRH_S = usize(8);
    pub const TAPSMR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPMR register.
//
//*****************************************************************************
pub const TBPMR = struct {
    pub const TBPSMRH_M = usize(0x0000FF00); // GPTM Timer B Prescale Match High
    // Byte
    pub const TBPSMR_M = usize(0x000000FF); // GPTM TimerB Prescale Match
    pub const TBPSMRH_S = usize(8);
    pub const TBPSMR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAR register.
//
//*****************************************************************************
pub const TAR = struct {
    pub const M = usize(0xFFFFFFFF); // GPTM Timer A Register
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBR register.
//
//*****************************************************************************
pub const TBR = struct {
    pub const M = usize(0xFFFFFFFF); // GPTM Timer B Register
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAV register.
//
//*****************************************************************************
pub const TAV = struct {
    pub const M = usize(0xFFFFFFFF); // GPTM Timer A Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBV register.
//
//*****************************************************************************
pub const TBV = struct {
    pub const M = usize(0xFFFFFFFF); // GPTM Timer B Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_RTCPD register.
//
//*****************************************************************************
pub const RTCPD = struct {
    pub const RTCPD_M = usize(0x0000FFFF); // RTC Predivide Counter Value
    pub const RTCPD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPS register.
//
//*****************************************************************************
pub const TAPS = struct {
    pub const PSS_M = usize(0x0000FFFF); // GPTM Timer A Prescaler Snapshot
    pub const PSS_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPS register.
//
//*****************************************************************************
pub const TBPS = struct {
    pub const PSS_M = usize(0x0000FFFF); // GPTM Timer A Prescaler Value
    pub const PSS_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPV register.
//
//*****************************************************************************
pub const TAPV = struct {
    pub const PSV_M = usize(0x0000FFFF); // GPTM Timer A Prescaler Value
    pub const PSV_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPV register.
//
//*****************************************************************************
pub const TBPV = struct {
    pub const PSV_M = usize(0x0000FFFF); // GPTM Timer B Prescaler Value
    pub const PSV_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_DMAEV register.
//
//*****************************************************************************
pub const DMAEV = struct {
    pub const TBMDMAEN = usize(0x00000800); // GPTM B Mode Match Event DMA
    // Trigger Enable
    pub const CBEDMAEN = usize(0x00000400); // GPTM B Capture Event DMA Trigger
    // Enable
    pub const CBMDMAEN = usize(0x00000200); // GPTM B Capture Match Event DMA
    // Trigger Enable
    pub const TBTODMAEN = usize(0x00000100); // GPTM B Time-Out Event DMA
    // Trigger Enable
    pub const TAMDMAEN = usize(0x00000010); // GPTM A Mode Match Event DMA
    // Trigger Enable
    pub const RTCDMAEN = usize(0x00000008); // GPTM A RTC Match Event DMA
    // Trigger Enable
    pub const CAEDMAEN = usize(0x00000004); // GPTM A Capture Event DMA Trigger
    // Enable
    pub const CAMDMAEN = usize(0x00000002); // GPTM A Capture Match Event DMA
    // Trigger Enable
    pub const TATODMAEN = usize(0x00000001); // GPTM A Time-Out Event DMA
    // Trigger Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_ADCEV register.
//
//*****************************************************************************
pub const ADCEV = struct {
    pub const TBMADCEN = usize(0x00000800); // GPTM B Mode Match Event ADC
    // Trigger Enable
    pub const CBEADCEN = usize(0x00000400); // GPTM B Capture Event ADC Trigger
    // Enable
    pub const CBMADCEN = usize(0x00000200); // GPTM B Capture Match Event ADC
    // Trigger Enable
    pub const TBTOADCEN = usize(0x00000100); // GPTM B Time-Out Event ADC
    // Trigger Enable
    pub const TAMADCEN = usize(0x00000010); // GPTM A Mode Match Event ADC
    // Trigger Enable
    pub const RTCADCEN = usize(0x00000008); // GPTM RTC Match Event ADC Trigger
    // Enable
    pub const CAEADCEN = usize(0x00000004); // GPTM A Capture Event ADC Trigger
    // Enable
    pub const CAMADCEN = usize(0x00000002); // GPTM A Capture Match Event ADC
    // Trigger Enable
    pub const TATOADCEN = usize(0x00000001); // GPTM A Time-Out Event ADC
    // Trigger Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_PP register.
//
//*****************************************************************************
pub const PP = struct {
    pub const ALTCLK = usize(0x00000040); // Alternate Clock Source
    pub const SYNCCNT = usize(0x00000020); // Synchronize Start
    pub const CHAIN = usize(0x00000010); // Chain with Other Timers
    pub const SIZE_M = usize(0x0000000F); // Count Size
    pub const SIZE_16 = usize(0x00000000); // Timer A and Timer B counters are
    // 16 bits each with an 8-bit
    // prescale counter
    pub const SIZE_32 = usize(0x00000001); // Timer A and Timer B counters are
    // 32 bits each with a 16-bit
    // prescale counter
};

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CC register.
//
//*****************************************************************************
pub const CC = struct {
    pub const ALTCLK = usize(0x00000001); // Alternate Clock Source
};
