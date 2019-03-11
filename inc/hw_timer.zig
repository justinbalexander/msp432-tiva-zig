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
pub const TIMER_O_CFG = usize(0x00000000);  // GPTM Configuration
pub const TIMER_O_TAMR = usize(0x00000004);  // GPTM Timer A Mode
pub const TIMER_O_TBMR = usize(0x00000008);  // GPTM Timer B Mode
pub const TIMER_O_CTL = usize(0x0000000C);  // GPTM Control
pub const TIMER_O_SYNC = usize(0x00000010);  // GPTM Synchronize
pub const TIMER_O_IMR = usize(0x00000018);  // GPTM Interrupt Mask
pub const TIMER_O_RIS = usize(0x0000001C);  // GPTM Raw Interrupt Status
pub const TIMER_O_MIS = usize(0x00000020);  // GPTM Masked Interrupt Status
pub const TIMER_O_ICR = usize(0x00000024);  // GPTM Interrupt Clear
pub const TIMER_O_TAILR = usize(0x00000028);  // GPTM Timer A Interval Load
pub const TIMER_O_TBILR = usize(0x0000002C);  // GPTM Timer B Interval Load
pub const TIMER_O_TAMATCHR = usize(0x00000030);  // GPTM Timer A Match
pub const TIMER_O_TBMATCHR = usize(0x00000034);  // GPTM Timer B Match
pub const TIMER_O_TAPR = usize(0x00000038);  // GPTM Timer A Prescale
pub const TIMER_O_TBPR = usize(0x0000003C);  // GPTM Timer B Prescale
pub const TIMER_O_TAPMR = usize(0x00000040);  // GPTM TimerA Prescale Match
pub const TIMER_O_TBPMR = usize(0x00000044);  // GPTM TimerB Prescale Match
pub const TIMER_O_TAR = usize(0x00000048);  // GPTM Timer A
pub const TIMER_O_TBR = usize(0x0000004C);  // GPTM Timer B
pub const TIMER_O_TAV = usize(0x00000050);  // GPTM Timer A Value
pub const TIMER_O_TBV = usize(0x00000054);  // GPTM Timer B Value
pub const TIMER_O_RTCPD = usize(0x00000058);  // GPTM RTC Predivide
pub const TIMER_O_TAPS = usize(0x0000005C);  // GPTM Timer A Prescale Snapshot
pub const TIMER_O_TBPS = usize(0x00000060);  // GPTM Timer B Prescale Snapshot
pub const TIMER_O_TAPV = usize(0x00000064);  // GPTM Timer A Prescale Value
pub const TIMER_O_TBPV = usize(0x00000068);  // GPTM Timer B Prescale Value
pub const TIMER_O_DMAEV = usize(0x0000006C);  // GPTM DMA Event
pub const TIMER_O_ADCEV = usize(0x00000070);  // GPTM ADC Event
pub const TIMER_O_PP = usize(0x00000FC0);  // GPTM Peripheral Properties
pub const TIMER_O_CC = usize(0x00000FC8);  // GPTM Clock Configuration

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CFG register.
//
//*****************************************************************************
pub const TIMER_CFG_M = usize(0x00000007);  // GPTM Configuration
pub const TIMER_CFG_32_BIT_TIMER = usize(0x00000000);  // For a 16/32-bit timer, this
                                            // value selects the 32-bit timer
                                            // configuration
pub const TIMER_CFG_32_BIT_RTC = usize(0x00000001);  // For a 16/32-bit timer, this
                                            // value selects the 32-bit
                                            // real-time clock (RTC) counter
                                            // configuration
pub const TIMER_CFG_16_BIT = usize(0x00000004);  // For a 16/32-bit timer, this
                                            // value selects the 16-bit timer
                                            // configuration

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMR register.
//
//*****************************************************************************
pub const TIMER_TAMR_TCACT_M = usize(0x0000E000);  // Timer Compare Action Select
pub const TIMER_TAMR_TCACT_NONE = usize(0x00000000);  // Disable compare operations
pub const TIMER_TAMR_TCACT_TOGGLE = usize(0x00002000);  // Toggle State on Time-Out
pub const TIMER_TAMR_TCACT_CLRTO = usize(0x00004000);  // Clear CCP on Time-Out
pub const TIMER_TAMR_TCACT_SETTO = usize(0x00006000);  // Set CCP on Time-Out
pub const TIMER_TAMR_TCACT_SETTOGTO = usize(0x00008000);  // Set CCP immediately and toggle
                                            // on Time-Out
pub const TIMER_TAMR_TCACT_CLRTOGTO = usize(0x0000A000);  // Clear CCP immediately and toggle
                                            // on Time-Out
pub const TIMER_TAMR_TCACT_SETCLRTO = usize(0x0000C000);  // Set CCP immediately and clear on
                                            // Time-Out
pub const TIMER_TAMR_TCACT_CLRSETTO = usize(0x0000E000);  // Clear CCP immediately and set on
                                            // Time-Out
pub const TIMER_TAMR_TACINTD = usize(0x00001000);  // One-shot/Periodic Interrupt
                                            // Disable
pub const TIMER_TAMR_TAPLO = usize(0x00000800);  // GPTM Timer A PWM Legacy
                                            // Operation
pub const TIMER_TAMR_TAMRSU = usize(0x00000400);  // GPTM Timer A Match Register
                                            // Update
pub const TIMER_TAMR_TAPWMIE = usize(0x00000200);  // GPTM Timer A PWM Interrupt
                                            // Enable
pub const TIMER_TAMR_TAILD = usize(0x00000100);  // GPTM Timer A Interval Load Write
pub const TIMER_TAMR_TASNAPS = usize(0x00000080);  // GPTM Timer A Snap-Shot Mode
pub const TIMER_TAMR_TAWOT = usize(0x00000040);  // GPTM Timer A Wait-on-Trigger
pub const TIMER_TAMR_TAMIE = usize(0x00000020);  // GPTM Timer A Match Interrupt
                                            // Enable
pub const TIMER_TAMR_TACDIR = usize(0x00000010);  // GPTM Timer A Count Direction
pub const TIMER_TAMR_TAAMS = usize(0x00000008);  // GPTM Timer A Alternate Mode
                                            // Select
pub const TIMER_TAMR_TACMR = usize(0x00000004);  // GPTM Timer A Capture Mode
pub const TIMER_TAMR_TAMR_M = usize(0x00000003);  // GPTM Timer A Mode
pub const TIMER_TAMR_TAMR_1_SHOT = usize(0x00000001);  // One-Shot Timer mode
pub const TIMER_TAMR_TAMR_PERIOD = usize(0x00000002);  // Periodic Timer mode
pub const TIMER_TAMR_TAMR_CAP = usize(0x00000003);  // Capture mode

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMR register.
//
//*****************************************************************************
pub const TIMER_TBMR_TCACT_M = usize(0x0000E000);  // Timer Compare Action Select
pub const TIMER_TBMR_TCACT_NONE = usize(0x00000000);  // Disable compare operations
pub const TIMER_TBMR_TCACT_TOGGLE = usize(0x00002000);  // Toggle State on Time-Out
pub const TIMER_TBMR_TCACT_CLRTO = usize(0x00004000);  // Clear CCP on Time-Out
pub const TIMER_TBMR_TCACT_SETTO = usize(0x00006000);  // Set CCP on Time-Out
pub const TIMER_TBMR_TCACT_SETTOGTO = usize(0x00008000);  // Set CCP immediately and toggle
                                            // on Time-Out
pub const TIMER_TBMR_TCACT_CLRTOGTO = usize(0x0000A000);  // Clear CCP immediately and toggle
                                            // on Time-Out
pub const TIMER_TBMR_TCACT_SETCLRTO = usize(0x0000C000);  // Set CCP immediately and clear on
                                            // Time-Out
pub const TIMER_TBMR_TCACT_CLRSETTO = usize(0x0000E000);  // Clear CCP immediately and set on
                                            // Time-Out
pub const TIMER_TBMR_TBCINTD = usize(0x00001000);  // One-Shot/Periodic Interrupt
                                            // Disable
pub const TIMER_TBMR_TBPLO = usize(0x00000800);  // GPTM Timer B PWM Legacy
                                            // Operation
pub const TIMER_TBMR_TBMRSU = usize(0x00000400);  // GPTM Timer B Match Register
                                            // Update
pub const TIMER_TBMR_TBPWMIE = usize(0x00000200);  // GPTM Timer B PWM Interrupt
                                            // Enable
pub const TIMER_TBMR_TBILD = usize(0x00000100);  // GPTM Timer B Interval Load Write
pub const TIMER_TBMR_TBSNAPS = usize(0x00000080);  // GPTM Timer B Snap-Shot Mode
pub const TIMER_TBMR_TBWOT = usize(0x00000040);  // GPTM Timer B Wait-on-Trigger
pub const TIMER_TBMR_TBMIE = usize(0x00000020);  // GPTM Timer B Match Interrupt
                                            // Enable
pub const TIMER_TBMR_TBCDIR = usize(0x00000010);  // GPTM Timer B Count Direction
pub const TIMER_TBMR_TBAMS = usize(0x00000008);  // GPTM Timer B Alternate Mode
                                            // Select
pub const TIMER_TBMR_TBCMR = usize(0x00000004);  // GPTM Timer B Capture Mode
pub const TIMER_TBMR_TBMR_M = usize(0x00000003);  // GPTM Timer B Mode
pub const TIMER_TBMR_TBMR_1_SHOT = usize(0x00000001);  // One-Shot Timer mode
pub const TIMER_TBMR_TBMR_PERIOD = usize(0x00000002);  // Periodic Timer mode
pub const TIMER_TBMR_TBMR_CAP = usize(0x00000003);  // Capture mode

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CTL register.
//
//*****************************************************************************
pub const TIMER_CTL_TBPWML = usize(0x00004000);  // GPTM Timer B PWM Output Level
pub const TIMER_CTL_TBOTE = usize(0x00002000);  // GPTM Timer B Output Trigger
                                            // Enable
pub const TIMER_CTL_TBEVENT_M = usize(0x00000C00);  // GPTM Timer B Event Mode
pub const TIMER_CTL_TBEVENT_POS = usize(0x00000000);  // Positive edge
pub const TIMER_CTL_TBEVENT_NEG = usize(0x00000400);  // Negative edge
pub const TIMER_CTL_TBEVENT_BOTH = usize(0x00000C00);  // Both edges
pub const TIMER_CTL_TBSTALL = usize(0x00000200);  // GPTM Timer B Stall Enable
pub const TIMER_CTL_TBEN = usize(0x00000100);  // GPTM Timer B Enable
pub const TIMER_CTL_TAPWML = usize(0x00000040);  // GPTM Timer A PWM Output Level
pub const TIMER_CTL_TAOTE = usize(0x00000020);  // GPTM Timer A Output Trigger
                                            // Enable
pub const TIMER_CTL_RTCEN = usize(0x00000010);  // GPTM RTC Stall Enable
pub const TIMER_CTL_TAEVENT_M = usize(0x0000000C);  // GPTM Timer A Event Mode
pub const TIMER_CTL_TAEVENT_POS = usize(0x00000000);  // Positive edge
pub const TIMER_CTL_TAEVENT_NEG = usize(0x00000004);  // Negative edge
pub const TIMER_CTL_TAEVENT_BOTH = usize(0x0000000C);  // Both edges
pub const TIMER_CTL_TASTALL = usize(0x00000002);  // GPTM Timer A Stall Enable
pub const TIMER_CTL_TAEN = usize(0x00000001);  // GPTM Timer A Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_SYNC register.
//
//*****************************************************************************
pub const TIMER_SYNC_SYNCWT5_M = usize(0x00C00000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 5
pub const TIMER_SYNC_SYNCWT5_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 5 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT5_TA = usize(0x00400000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 5 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT5_TB = usize(0x00800000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 5 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT5_TATB = usize(0x00C00000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 5 is triggered
pub const TIMER_SYNC_SYNCWT4_M = usize(0x00300000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 4
pub const TIMER_SYNC_SYNCWT4_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 4 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT4_TA = usize(0x00100000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 4 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT4_TB = usize(0x00200000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 4 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT4_TATB = usize(0x00300000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 4 is triggered
pub const TIMER_SYNC_SYNCWT3_M = usize(0x000C0000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 3
pub const TIMER_SYNC_SYNCWT3_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 3 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT3_TA = usize(0x00040000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 3 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT3_TB = usize(0x00080000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 3 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT3_TATB = usize(0x000C0000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 3 is triggered
pub const TIMER_SYNC_SYNCWT2_M = usize(0x00030000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 2
pub const TIMER_SYNC_SYNCWT2_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 2 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT2_TA = usize(0x00010000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 2 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT2_TB = usize(0x00020000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 2 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT2_TATB = usize(0x00030000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 2 is triggered
pub const TIMER_SYNC_SYNCT7_M = usize(0x0000C000);  // Synchronize GPTM Timer 7
pub const TIMER_SYNC_SYNCT7_NONE = usize(0x00000000);  // GPT7 is not affected
pub const TIMER_SYNC_SYNCT7_TA = usize(0x00004000);  // A timeout event for Timer A of
                                            // GPTM7 is triggered
pub const TIMER_SYNC_SYNCT7_TB = usize(0x00008000);  // A timeout event for Timer B of
                                            // GPTM7 is triggered
pub const TIMER_SYNC_SYNCT7_TATB = usize(0x0000C000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM7 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT1_M = usize(0x0000C000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 1
pub const TIMER_SYNC_SYNCWT1_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 1 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT1_TA = usize(0x00004000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 1 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT1_TB = usize(0x00008000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 1 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT1_TATB = usize(0x0000C000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 1 is triggered
pub const TIMER_SYNC_SYNCWT0_M = usize(0x00003000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 0
pub const TIMER_SYNC_SYNCWT0_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 0 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT0_TA = usize(0x00001000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 0 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT0_TB = usize(0x00002000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 0 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT0_TATB = usize(0x00003000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 0 is triggered
pub const TIMER_SYNC_SYNCT6_M = usize(0x00003000);  // Synchronize GPTM Timer 6
pub const TIMER_SYNC_SYNCT6_NONE = usize(0x00000000);  // GPTM6 is not affected
pub const TIMER_SYNC_SYNCT6_TA = usize(0x00001000);  // A timeout event for Timer A of
                                            // GPTM6 is triggered
pub const TIMER_SYNC_SYNCT6_TB = usize(0x00002000);  // A timeout event for Timer B of
                                            // GPTM6 is triggered
pub const TIMER_SYNC_SYNCT6_TATB = usize(0x00003000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM6 is
                                            // triggered
pub const TIMER_SYNC_SYNCT5_M = usize(0x00000C00);  // Synchronize GPTM Timer 5
pub const TIMER_SYNC_SYNCT5_NONE = usize(0x00000000);  // GPTM5 is not affected
pub const TIMER_SYNC_SYNCT5_TA = usize(0x00000400);  // A timeout event for Timer A of
                                            // GPTM5 is triggered
pub const TIMER_SYNC_SYNCT5_TB = usize(0x00000800);  // A timeout event for Timer B of
                                            // GPTM5 is triggered
pub const TIMER_SYNC_SYNCT5_TATB = usize(0x00000C00);  // A timeout event for both Timer A
                                            // and Timer B of GPTM5 is
                                            // triggered
pub const TIMER_SYNC_SYNCT4_M = usize(0x00000300);  // Synchronize GPTM Timer 4
pub const TIMER_SYNC_SYNCT4_NONE = usize(0x00000000);  // GPTM4 is not affected
pub const TIMER_SYNC_SYNCT4_TA = usize(0x00000100);  // A timeout event for Timer A of
                                            // GPTM4 is triggered
pub const TIMER_SYNC_SYNCT4_TB = usize(0x00000200);  // A timeout event for Timer B of
                                            // GPTM4 is triggered
pub const TIMER_SYNC_SYNCT4_TATB = usize(0x00000300);  // A timeout event for both Timer A
                                            // and Timer B of GPTM4 is
                                            // triggered
pub const TIMER_SYNC_SYNCT3_M = usize(0x000000C0);  // Synchronize GPTM Timer 3
pub const TIMER_SYNC_SYNCT3_NONE = usize(0x00000000);  // GPTM3 is not affected
pub const TIMER_SYNC_SYNCT3_TA = usize(0x00000040);  // A timeout event for Timer A of
                                            // GPTM3 is triggered
pub const TIMER_SYNC_SYNCT3_TB = usize(0x00000080);  // A timeout event for Timer B of
                                            // GPTM3 is triggered
pub const TIMER_SYNC_SYNCT3_TATB = usize(0x000000C0);  // A timeout event for both Timer A
                                            // and Timer B of GPTM3 is
                                            // triggered
pub const TIMER_SYNC_SYNCT2_M = usize(0x00000030);  // Synchronize GPTM Timer 2
pub const TIMER_SYNC_SYNCT2_NONE = usize(0x00000000);  // GPTM2 is not affected
pub const TIMER_SYNC_SYNCT2_TA = usize(0x00000010);  // A timeout event for Timer A of
                                            // GPTM2 is triggered
pub const TIMER_SYNC_SYNCT2_TB = usize(0x00000020);  // A timeout event for Timer B of
                                            // GPTM2 is triggered
pub const TIMER_SYNC_SYNCT2_TATB = usize(0x00000030);  // A timeout event for both Timer A
                                            // and Timer B of GPTM2 is
                                            // triggered
pub const TIMER_SYNC_SYNCT1_M = usize(0x0000000C);  // Synchronize GPTM Timer 1
pub const TIMER_SYNC_SYNCT1_NONE = usize(0x00000000);  // GPTM1 is not affected
pub const TIMER_SYNC_SYNCT1_TA = usize(0x00000004);  // A timeout event for Timer A of
                                            // GPTM1 is triggered
pub const TIMER_SYNC_SYNCT1_TB = usize(0x00000008);  // A timeout event for Timer B of
                                            // GPTM1 is triggered
pub const TIMER_SYNC_SYNCT1_TATB = usize(0x0000000C);  // A timeout event for both Timer A
                                            // and Timer B of GPTM1 is
                                            // triggered
pub const TIMER_SYNC_SYNCT0_M = usize(0x00000003);  // Synchronize GPTM Timer 0
pub const TIMER_SYNC_SYNCT0_NONE = usize(0x00000000);  // GPTM0 is not affected
pub const TIMER_SYNC_SYNCT0_TA = usize(0x00000001);  // A timeout event for Timer A of
                                            // GPTM0 is triggered
pub const TIMER_SYNC_SYNCT0_TB = usize(0x00000002);  // A timeout event for Timer B of
                                            // GPTM0 is triggered
pub const TIMER_SYNC_SYNCT0_TATB = usize(0x00000003);  // A timeout event for both Timer A
                                            // and Timer B of GPTM0 is
                                            // triggered

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_IMR register.
//
//*****************************************************************************
pub const TIMER_IMR_WUEIM = usize(0x00010000);  // 32/64-Bit Wide GPTM Write Update
                                            // Error Interrupt Mask
pub const TIMER_IMR_DMABIM = usize(0x00002000);  // GPTM Timer B DMA Done Interrupt
                                            // Mask
pub const TIMER_IMR_TBMIM = usize(0x00000800);  // GPTM Timer B Match Interrupt
                                            // Mask
pub const TIMER_IMR_CBEIM = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Interrupt Mask
pub const TIMER_IMR_CBMIM = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Interrupt Mask
pub const TIMER_IMR_TBTOIM = usize(0x00000100);  // GPTM Timer B Time-Out Interrupt
                                            // Mask
pub const TIMER_IMR_DMAAIM = usize(0x00000020);  // GPTM Timer A DMA Done Interrupt
                                            // Mask
pub const TIMER_IMR_TAMIM = usize(0x00000010);  // GPTM Timer A Match Interrupt
                                            // Mask
pub const TIMER_IMR_RTCIM = usize(0x00000008);  // GPTM RTC Interrupt Mask
pub const TIMER_IMR_CAEIM = usize(0x00000004);  // GPTM Timer A Capture Mode Event
                                            // Interrupt Mask
pub const TIMER_IMR_CAMIM = usize(0x00000002);  // GPTM Timer A Capture Mode Match
                                            // Interrupt Mask
pub const TIMER_IMR_TATOIM = usize(0x00000001);  // GPTM Timer A Time-Out Interrupt
                                            // Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_RIS register.
//
//*****************************************************************************
pub const TIMER_RIS_WUERIS = usize(0x00010000);  // 32/64-Bit Wide GPTM Write Update
                                            // Error Raw Interrupt Status
pub const TIMER_RIS_DMABRIS = usize(0x00002000);  // GPTM Timer B DMA Done Raw
                                            // Interrupt Status
pub const TIMER_RIS_TBMRIS = usize(0x00000800);  // GPTM Timer B Match Raw Interrupt
pub const TIMER_RIS_CBERIS = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Raw Interrupt
pub const TIMER_RIS_CBMRIS = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Raw Interrupt
pub const TIMER_RIS_TBTORIS = usize(0x00000100);  // GPTM Timer B Time-Out Raw
                                            // Interrupt
pub const TIMER_RIS_DMAARIS = usize(0x00000020);  // GPTM Timer A DMA Done Raw
                                            // Interrupt Status
pub const TIMER_RIS_TAMRIS = usize(0x00000010);  // GPTM Timer A Match Raw Interrupt
pub const TIMER_RIS_RTCRIS = usize(0x00000008);  // GPTM RTC Raw Interrupt
pub const TIMER_RIS_CAERIS = usize(0x00000004);  // GPTM Timer A Capture Mode Event
                                            // Raw Interrupt
pub const TIMER_RIS_CAMRIS = usize(0x00000002);  // GPTM Timer A Capture Mode Match
                                            // Raw Interrupt
pub const TIMER_RIS_TATORIS = usize(0x00000001);  // GPTM Timer A Time-Out Raw
                                            // Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_MIS register.
//
//*****************************************************************************
pub const TIMER_MIS_WUEMIS = usize(0x00010000);  // 32/64-Bit Wide GPTM Write Update
                                            // Error Masked Interrupt Status
pub const TIMER_MIS_DMABMIS = usize(0x00002000);  // GPTM Timer B DMA Done Masked
                                            // Interrupt
pub const TIMER_MIS_TBMMIS = usize(0x00000800);  // GPTM Timer B Match Masked
                                            // Interrupt
pub const TIMER_MIS_CBEMIS = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Masked Interrupt
pub const TIMER_MIS_CBMMIS = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Masked Interrupt
pub const TIMER_MIS_TBTOMIS = usize(0x00000100);  // GPTM Timer B Time-Out Masked
                                            // Interrupt
pub const TIMER_MIS_DMAAMIS = usize(0x00000020);  // GPTM Timer A DMA Done Masked
                                            // Interrupt
pub const TIMER_MIS_TAMMIS = usize(0x00000010);  // GPTM Timer A Match Masked
                                            // Interrupt
pub const TIMER_MIS_RTCMIS = usize(0x00000008);  // GPTM RTC Masked Interrupt
pub const TIMER_MIS_CAEMIS = usize(0x00000004);  // GPTM Timer A Capture Mode Event
                                            // Masked Interrupt
pub const TIMER_MIS_CAMMIS = usize(0x00000002);  // GPTM Timer A Capture Mode Match
                                            // Masked Interrupt
pub const TIMER_MIS_TATOMIS = usize(0x00000001);  // GPTM Timer A Time-Out Masked
                                            // Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_ICR register.
//
//*****************************************************************************
pub const TIMER_ICR_WUECINT = usize(0x00010000);  // 32/64-Bit Wide GPTM Write Update
                                            // Error Interrupt Clear
pub const TIMER_ICR_DMABINT = usize(0x00002000);  // GPTM Timer B DMA Done Interrupt
                                            // Clear
pub const TIMER_ICR_TBMCINT = usize(0x00000800);  // GPTM Timer B Match Interrupt
                                            // Clear
pub const TIMER_ICR_CBECINT = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Interrupt Clear
pub const TIMER_ICR_CBMCINT = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Interrupt Clear
pub const TIMER_ICR_TBTOCINT = usize(0x00000100);  // GPTM Timer B Time-Out Interrupt
                                            // Clear
pub const TIMER_ICR_DMAAINT = usize(0x00000020);  // GPTM Timer A DMA Done Interrupt
                                            // Clear
pub const TIMER_ICR_TAMCINT = usize(0x00000010);  // GPTM Timer A Match Interrupt
                                            // Clear
pub const TIMER_ICR_RTCCINT = usize(0x00000008);  // GPTM RTC Interrupt Clear
pub const TIMER_ICR_CAECINT = usize(0x00000004);  // GPTM Timer A Capture Mode Event
                                            // Interrupt Clear
pub const TIMER_ICR_CAMCINT = usize(0x00000002);  // GPTM Timer A Capture Mode Match
                                            // Interrupt Clear
pub const TIMER_ICR_TATOCINT = usize(0x00000001);  // GPTM Timer A Time-Out Raw
                                            // Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAILR register.
//
//*****************************************************************************
pub const TIMER_TAILR_M = usize(0xFFFFFFFF);  // GPTM Timer A Interval Load
                                            // Register
pub const TIMER_TAILR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBILR register.
//
//*****************************************************************************
pub const TIMER_TBILR_M = usize(0xFFFFFFFF);  // GPTM Timer B Interval Load
                                            // Register
pub const TIMER_TBILR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMATCHR
// register.
//
//*****************************************************************************
pub const TIMER_TAMATCHR_TAMR_M = usize(0xFFFFFFFF);  // GPTM Timer A Match Register
pub const TIMER_TAMATCHR_TAMR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMATCHR
// register.
//
//*****************************************************************************
pub const TIMER_TBMATCHR_TBMR_M = usize(0xFFFFFFFF);  // GPTM Timer B Match Register
pub const TIMER_TBMATCHR_TBMR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPR register.
//
//*****************************************************************************
pub const TIMER_TAPR_TAPSRH_M = usize(0x0000FF00);  // GPTM Timer A Prescale High Byte
pub const TIMER_TAPR_TAPSR_M = usize(0x000000FF);  // GPTM Timer A Prescale
pub const TIMER_TAPR_TAPSRH_S = usize(8);
pub const TIMER_TAPR_TAPSR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPR register.
//
//*****************************************************************************
pub const TIMER_TBPR_TBPSRH_M = usize(0x0000FF00);  // GPTM Timer B Prescale High Byte
pub const TIMER_TBPR_TBPSR_M = usize(0x000000FF);  // GPTM Timer B Prescale
pub const TIMER_TBPR_TBPSRH_S = usize(8);
pub const TIMER_TBPR_TBPSR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPMR register.
//
//*****************************************************************************
pub const TIMER_TAPMR_TAPSMRH_M = usize(0x0000FF00);  // GPTM Timer A Prescale Match High
                                            // Byte
pub const TIMER_TAPMR_TAPSMR_M = usize(0x000000FF);  // GPTM TimerA Prescale Match
pub const TIMER_TAPMR_TAPSMRH_S = usize(8);
pub const TIMER_TAPMR_TAPSMR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPMR register.
//
//*****************************************************************************
pub const TIMER_TBPMR_TBPSMRH_M = usize(0x0000FF00);  // GPTM Timer B Prescale Match High
                                            // Byte
pub const TIMER_TBPMR_TBPSMR_M = usize(0x000000FF);  // GPTM TimerB Prescale Match
pub const TIMER_TBPMR_TBPSMRH_S = usize(8);
pub const TIMER_TBPMR_TBPSMR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAR register.
//
//*****************************************************************************
pub const TIMER_TAR_M = usize(0xFFFFFFFF);  // GPTM Timer A Register
pub const TIMER_TAR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBR register.
//
//*****************************************************************************
pub const TIMER_TBR_M = usize(0xFFFFFFFF);  // GPTM Timer B Register
pub const TIMER_TBR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAV register.
//
//*****************************************************************************
pub const TIMER_TAV_M = usize(0xFFFFFFFF);  // GPTM Timer A Value
pub const TIMER_TAV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBV register.
//
//*****************************************************************************
pub const TIMER_TBV_M = usize(0xFFFFFFFF);  // GPTM Timer B Value
pub const TIMER_TBV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_RTCPD register.
//
//*****************************************************************************
pub const TIMER_RTCPD_RTCPD_M = usize(0x0000FFFF);  // RTC Predivide Counter Value
pub const TIMER_RTCPD_RTCPD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPS register.
//
//*****************************************************************************
pub const TIMER_TAPS_PSS_M = usize(0x0000FFFF);  // GPTM Timer A Prescaler Snapshot
pub const TIMER_TAPS_PSS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPS register.
//
//*****************************************************************************
pub const TIMER_TBPS_PSS_M = usize(0x0000FFFF);  // GPTM Timer A Prescaler Value
pub const TIMER_TBPS_PSS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPV register.
//
//*****************************************************************************
pub const TIMER_TAPV_PSV_M = usize(0x0000FFFF);  // GPTM Timer A Prescaler Value
pub const TIMER_TAPV_PSV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPV register.
//
//*****************************************************************************
pub const TIMER_TBPV_PSV_M = usize(0x0000FFFF);  // GPTM Timer B Prescaler Value
pub const TIMER_TBPV_PSV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_DMAEV register.
//
//*****************************************************************************
pub const TIMER_DMAEV_TBMDMAEN = usize(0x00000800);  // GPTM B Mode Match Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_CBEDMAEN = usize(0x00000400);  // GPTM B Capture Event DMA Trigger
                                            // Enable
pub const TIMER_DMAEV_CBMDMAEN = usize(0x00000200);  // GPTM B Capture Match Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_TBTODMAEN = usize(0x00000100);  // GPTM B Time-Out Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_TAMDMAEN = usize(0x00000010);  // GPTM A Mode Match Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_RTCDMAEN = usize(0x00000008);  // GPTM A RTC Match Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_CAEDMAEN = usize(0x00000004);  // GPTM A Capture Event DMA Trigger
                                            // Enable
pub const TIMER_DMAEV_CAMDMAEN = usize(0x00000002);  // GPTM A Capture Match Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_TATODMAEN = usize(0x00000001);  // GPTM A Time-Out Event DMA
                                            // Trigger Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_ADCEV register.
//
//*****************************************************************************
pub const TIMER_ADCEV_TBMADCEN = usize(0x00000800);  // GPTM B Mode Match Event ADC
                                            // Trigger Enable
pub const TIMER_ADCEV_CBEADCEN = usize(0x00000400);  // GPTM B Capture Event ADC Trigger
                                            // Enable
pub const TIMER_ADCEV_CBMADCEN = usize(0x00000200);  // GPTM B Capture Match Event ADC
                                            // Trigger Enable
pub const TIMER_ADCEV_TBTOADCEN = usize(0x00000100);  // GPTM B Time-Out Event ADC
                                            // Trigger Enable
pub const TIMER_ADCEV_TAMADCEN = usize(0x00000010);  // GPTM A Mode Match Event ADC
                                            // Trigger Enable
pub const TIMER_ADCEV_RTCADCEN = usize(0x00000008);  // GPTM RTC Match Event ADC Trigger
                                            // Enable
pub const TIMER_ADCEV_CAEADCEN = usize(0x00000004);  // GPTM A Capture Event ADC Trigger
                                            // Enable
pub const TIMER_ADCEV_CAMADCEN = usize(0x00000002);  // GPTM A Capture Match Event ADC
                                            // Trigger Enable
pub const TIMER_ADCEV_TATOADCEN = usize(0x00000001);  // GPTM A Time-Out Event ADC
                                            // Trigger Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_PP register.
//
//*****************************************************************************
pub const TIMER_PP_ALTCLK = usize(0x00000040);  // Alternate Clock Source
pub const TIMER_PP_SYNCCNT = usize(0x00000020);  // Synchronize Start
pub const TIMER_PP_CHAIN = usize(0x00000010);  // Chain with Other Timers
pub const TIMER_PP_SIZE_M = usize(0x0000000F);  // Count Size
pub const TIMER_PP_SIZE_16 = usize(0x00000000);  // Timer A and Timer B counters are
                                            // 16 bits each with an 8-bit
                                            // prescale counter
pub const TIMER_PP_SIZE_32 = usize(0x00000001);  // Timer A and Timer B counters are
                                            // 32 bits each with a 16-bit
                                            // prescale counter

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CC register.
//
//*****************************************************************************
pub const TIMER_CC_ALTCLK = usize(0x00000001);  // Alternate Clock Source

