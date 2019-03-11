//*****************************************************************************
//
// hw_adc.h - Macros used when accessing the ADC hardware.
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
// The following are defines for the ADC register offsets.
//
//*****************************************************************************
pub const ADC_O_ACTSS = usize(0x00000000);  // ADC Active Sample Sequencer
pub const ADC_O_RIS = usize(0x00000004);  // ADC Raw Interrupt Status
pub const ADC_O_IM = usize(0x00000008);  // ADC Interrupt Mask
pub const ADC_O_ISC = usize(0x0000000C);  // ADC Interrupt Status and Clear
pub const ADC_O_OSTAT = usize(0x00000010);  // ADC Overflow Status
pub const ADC_O_EMUX = usize(0x00000014);  // ADC Event Multiplexer Select
pub const ADC_O_USTAT = usize(0x00000018);  // ADC Underflow Status
pub const ADC_O_TSSEL = usize(0x0000001C);  // ADC Trigger Source Select
pub const ADC_O_SSPRI = usize(0x00000020);  // ADC Sample Sequencer Priority
pub const ADC_O_SPC = usize(0x00000024);  // ADC Sample Phase Control
pub const ADC_O_PSSI = usize(0x00000028);  // ADC Processor Sample Sequence
                                            // Initiate
pub const ADC_O_SAC = usize(0x00000030);  // ADC Sample Averaging Control
pub const ADC_O_DCISC = usize(0x00000034);  // ADC Digital Comparator Interrupt
                                            // Status and Clear
pub const ADC_O_CTL = usize(0x00000038);  // ADC Control
pub const ADC_O_SSMUX0 = usize(0x00000040);  // ADC Sample Sequence Input
                                            // Multiplexer Select 0
pub const ADC_O_SSCTL0 = usize(0x00000044);  // ADC Sample Sequence Control 0
pub const ADC_O_SSFIFO0 = usize(0x00000048);  // ADC Sample Sequence Result FIFO
                                            // 0
pub const ADC_O_SSFSTAT0 = usize(0x0000004C);  // ADC Sample Sequence FIFO 0
                                            // Status
pub const ADC_O_SSOP0 = usize(0x00000050);  // ADC Sample Sequence 0 Operation
pub const ADC_O_SSDC0 = usize(0x00000054);  // ADC Sample Sequence 0 Digital
                                            // Comparator Select
pub const ADC_O_SSEMUX0 = usize(0x00000058);  // ADC Sample Sequence Extended
                                            // Input Multiplexer Select 0
pub const ADC_O_SSTSH0 = usize(0x0000005C);  // ADC Sample Sequence 0 Sample and
                                            // Hold Time
pub const ADC_O_SSMUX1 = usize(0x00000060);  // ADC Sample Sequence Input
                                            // Multiplexer Select 1
pub const ADC_O_SSCTL1 = usize(0x00000064);  // ADC Sample Sequence Control 1
pub const ADC_O_SSFIFO1 = usize(0x00000068);  // ADC Sample Sequence Result FIFO
                                            // 1
pub const ADC_O_SSFSTAT1 = usize(0x0000006C);  // ADC Sample Sequence FIFO 1
                                            // Status
pub const ADC_O_SSOP1 = usize(0x00000070);  // ADC Sample Sequence 1 Operation
pub const ADC_O_SSDC1 = usize(0x00000074);  // ADC Sample Sequence 1 Digital
                                            // Comparator Select
pub const ADC_O_SSEMUX1 = usize(0x00000078);  // ADC Sample Sequence Extended
                                            // Input Multiplexer Select 1
pub const ADC_O_SSTSH1 = usize(0x0000007C);  // ADC Sample Sequence 1 Sample and
                                            // Hold Time
pub const ADC_O_SSMUX2 = usize(0x00000080);  // ADC Sample Sequence Input
                                            // Multiplexer Select 2
pub const ADC_O_SSCTL2 = usize(0x00000084);  // ADC Sample Sequence Control 2
pub const ADC_O_SSFIFO2 = usize(0x00000088);  // ADC Sample Sequence Result FIFO
                                            // 2
pub const ADC_O_SSFSTAT2 = usize(0x0000008C);  // ADC Sample Sequence FIFO 2
                                            // Status
pub const ADC_O_SSOP2 = usize(0x00000090);  // ADC Sample Sequence 2 Operation
pub const ADC_O_SSDC2 = usize(0x00000094);  // ADC Sample Sequence 2 Digital
                                            // Comparator Select
pub const ADC_O_SSEMUX2 = usize(0x00000098);  // ADC Sample Sequence Extended
                                            // Input Multiplexer Select 2
pub const ADC_O_SSTSH2 = usize(0x0000009C);  // ADC Sample Sequence 2 Sample and
                                            // Hold Time
pub const ADC_O_SSMUX3 = usize(0x000000A0);  // ADC Sample Sequence Input
                                            // Multiplexer Select 3
pub const ADC_O_SSCTL3 = usize(0x000000A4);  // ADC Sample Sequence Control 3
pub const ADC_O_SSFIFO3 = usize(0x000000A8);  // ADC Sample Sequence Result FIFO
                                            // 3
pub const ADC_O_SSFSTAT3 = usize(0x000000AC);  // ADC Sample Sequence FIFO 3
                                            // Status
pub const ADC_O_SSOP3 = usize(0x000000B0);  // ADC Sample Sequence 3 Operation
pub const ADC_O_SSDC3 = usize(0x000000B4);  // ADC Sample Sequence 3 Digital
                                            // Comparator Select
pub const ADC_O_SSEMUX3 = usize(0x000000B8);  // ADC Sample Sequence Extended
                                            // Input Multiplexer Select 3
pub const ADC_O_SSTSH3 = usize(0x000000BC);  // ADC Sample Sequence 3 Sample and
                                            // Hold Time
pub const ADC_O_DCRIC = usize(0x00000D00);  // ADC Digital Comparator Reset
                                            // Initial Conditions
pub const ADC_O_DCCTL0 = usize(0x00000E00);  // ADC Digital Comparator Control 0
pub const ADC_O_DCCTL1 = usize(0x00000E04);  // ADC Digital Comparator Control 1
pub const ADC_O_DCCTL2 = usize(0x00000E08);  // ADC Digital Comparator Control 2
pub const ADC_O_DCCTL3 = usize(0x00000E0C);  // ADC Digital Comparator Control 3
pub const ADC_O_DCCTL4 = usize(0x00000E10);  // ADC Digital Comparator Control 4
pub const ADC_O_DCCTL5 = usize(0x00000E14);  // ADC Digital Comparator Control 5
pub const ADC_O_DCCTL6 = usize(0x00000E18);  // ADC Digital Comparator Control 6
pub const ADC_O_DCCTL7 = usize(0x00000E1C);  // ADC Digital Comparator Control 7
pub const ADC_O_DCCMP0 = usize(0x00000E40);  // ADC Digital Comparator Range 0
pub const ADC_O_DCCMP1 = usize(0x00000E44);  // ADC Digital Comparator Range 1
pub const ADC_O_DCCMP2 = usize(0x00000E48);  // ADC Digital Comparator Range 2
pub const ADC_O_DCCMP3 = usize(0x00000E4C);  // ADC Digital Comparator Range 3
pub const ADC_O_DCCMP4 = usize(0x00000E50);  // ADC Digital Comparator Range 4
pub const ADC_O_DCCMP5 = usize(0x00000E54);  // ADC Digital Comparator Range 5
pub const ADC_O_DCCMP6 = usize(0x00000E58);  // ADC Digital Comparator Range 6
pub const ADC_O_DCCMP7 = usize(0x00000E5C);  // ADC Digital Comparator Range 7
pub const ADC_O_PP = usize(0x00000FC0);  // ADC Peripheral Properties
pub const ADC_O_PC = usize(0x00000FC4);  // ADC Peripheral Configuration
pub const ADC_O_CC = usize(0x00000FC8);  // ADC Clock Configuration

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_ACTSS register.
//
//*****************************************************************************
pub const ADC_ACTSS_BUSY = usize(0x00010000);  // ADC Busy
pub const ADC_ACTSS_ADEN3 = usize(0x00000800);  // ADC SS3 DMA Enable
pub const ADC_ACTSS_ADEN2 = usize(0x00000400);  // ADC SS2 DMA Enable
pub const ADC_ACTSS_ADEN1 = usize(0x00000200);  // ADC SS1 DMA Enable
pub const ADC_ACTSS_ADEN0 = usize(0x00000100);  // ADC SS1 DMA Enable
pub const ADC_ACTSS_ASEN3 = usize(0x00000008);  // ADC SS3 Enable
pub const ADC_ACTSS_ASEN2 = usize(0x00000004);  // ADC SS2 Enable
pub const ADC_ACTSS_ASEN1 = usize(0x00000002);  // ADC SS1 Enable
pub const ADC_ACTSS_ASEN0 = usize(0x00000001);  // ADC SS0 Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_RIS register.
//
//*****************************************************************************
pub const ADC_RIS_INRDC = usize(0x00010000);  // Digital Comparator Raw Interrupt
                                            // Status
pub const ADC_RIS_DMAINR3 = usize(0x00000800);  // SS3 DMA Raw Interrupt Status
pub const ADC_RIS_DMAINR2 = usize(0x00000400);  // SS2 DMA Raw Interrupt Status
pub const ADC_RIS_DMAINR1 = usize(0x00000200);  // SS1 DMA Raw Interrupt Status
pub const ADC_RIS_DMAINR0 = usize(0x00000100);  // SS0 DMA Raw Interrupt Status
pub const ADC_RIS_INR3 = usize(0x00000008);  // SS3 Raw Interrupt Status
pub const ADC_RIS_INR2 = usize(0x00000004);  // SS2 Raw Interrupt Status
pub const ADC_RIS_INR1 = usize(0x00000002);  // SS1 Raw Interrupt Status
pub const ADC_RIS_INR0 = usize(0x00000001);  // SS0 Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_IM register.
//
//*****************************************************************************
pub const ADC_IM_DCONSS3 = usize(0x00080000);  // Digital Comparator Interrupt on
                                            // SS3
pub const ADC_IM_DCONSS2 = usize(0x00040000);  // Digital Comparator Interrupt on
                                            // SS2
pub const ADC_IM_DCONSS1 = usize(0x00020000);  // Digital Comparator Interrupt on
                                            // SS1
pub const ADC_IM_DCONSS0 = usize(0x00010000);  // Digital Comparator Interrupt on
                                            // SS0
pub const ADC_IM_DMAMASK3 = usize(0x00000800);  // SS3 DMA Interrupt Mask
pub const ADC_IM_DMAMASK2 = usize(0x00000400);  // SS2 DMA Interrupt Mask
pub const ADC_IM_DMAMASK1 = usize(0x00000200);  // SS1 DMA Interrupt Mask
pub const ADC_IM_DMAMASK0 = usize(0x00000100);  // SS0 DMA Interrupt Mask
pub const ADC_IM_MASK3 = usize(0x00000008);  // SS3 Interrupt Mask
pub const ADC_IM_MASK2 = usize(0x00000004);  // SS2 Interrupt Mask
pub const ADC_IM_MASK1 = usize(0x00000002);  // SS1 Interrupt Mask
pub const ADC_IM_MASK0 = usize(0x00000001);  // SS0 Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_ISC register.
//
//*****************************************************************************
pub const ADC_ISC_DCINSS3 = usize(0x00080000);  // Digital Comparator Interrupt
                                            // Status on SS3
pub const ADC_ISC_DCINSS2 = usize(0x00040000);  // Digital Comparator Interrupt
                                            // Status on SS2
pub const ADC_ISC_DCINSS1 = usize(0x00020000);  // Digital Comparator Interrupt
                                            // Status on SS1
pub const ADC_ISC_DCINSS0 = usize(0x00010000);  // Digital Comparator Interrupt
                                            // Status on SS0
pub const ADC_ISC_DMAIN3 = usize(0x00000800);  // SS3 DMA Interrupt Status and
                                            // Clear
pub const ADC_ISC_DMAIN2 = usize(0x00000400);  // SS2 DMA Interrupt Status and
                                            // Clear
pub const ADC_ISC_DMAIN1 = usize(0x00000200);  // SS1 DMA Interrupt Status and
                                            // Clear
pub const ADC_ISC_DMAIN0 = usize(0x00000100);  // SS0 DMA Interrupt Status and
                                            // Clear
pub const ADC_ISC_IN3 = usize(0x00000008);  // SS3 Interrupt Status and Clear
pub const ADC_ISC_IN2 = usize(0x00000004);  // SS2 Interrupt Status and Clear
pub const ADC_ISC_IN1 = usize(0x00000002);  // SS1 Interrupt Status and Clear
pub const ADC_ISC_IN0 = usize(0x00000001);  // SS0 Interrupt Status and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_OSTAT register.
//
//*****************************************************************************
pub const ADC_OSTAT_OV3 = usize(0x00000008);  // SS3 FIFO Overflow
pub const ADC_OSTAT_OV2 = usize(0x00000004);  // SS2 FIFO Overflow
pub const ADC_OSTAT_OV1 = usize(0x00000002);  // SS1 FIFO Overflow
pub const ADC_OSTAT_OV0 = usize(0x00000001);  // SS0 FIFO Overflow

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_EMUX register.
//
//*****************************************************************************
pub const ADC_EMUX_EM3_M = usize(0x0000F000);  // SS3 Trigger Select
pub const ADC_EMUX_EM3_PROCESSOR = usize(0x00000000);  // Processor (default)
pub const ADC_EMUX_EM3_COMP0 = usize(0x00001000);  // Analog Comparator 0
pub const ADC_EMUX_EM3_COMP1 = usize(0x00002000);  // Analog Comparator 1
pub const ADC_EMUX_EM3_COMP2 = usize(0x00003000);  // Analog Comparator 2
pub const ADC_EMUX_EM3_EXTERNAL = usize(0x00004000);  // External (GPIO Pins)
pub const ADC_EMUX_EM3_TIMER = usize(0x00005000);  // Timer
pub const ADC_EMUX_EM3_PWM0 = usize(0x00006000);  // PWM generator 0
pub const ADC_EMUX_EM3_PWM1 = usize(0x00007000);  // PWM generator 1
pub const ADC_EMUX_EM3_PWM2 = usize(0x00008000);  // PWM generator 2
pub const ADC_EMUX_EM3_PWM3 = usize(0x00009000);  // PWM generator 3
pub const ADC_EMUX_EM3_NEVER = usize(0x0000E000);  // Never Trigger
pub const ADC_EMUX_EM3_ALWAYS = usize(0x0000F000);  // Always (continuously sample)
pub const ADC_EMUX_EM2_M = usize(0x00000F00);  // SS2 Trigger Select
pub const ADC_EMUX_EM2_PROCESSOR = usize(0x00000000);  // Processor (default)
pub const ADC_EMUX_EM2_COMP0 = usize(0x00000100);  // Analog Comparator 0
pub const ADC_EMUX_EM2_COMP1 = usize(0x00000200);  // Analog Comparator 1
pub const ADC_EMUX_EM2_COMP2 = usize(0x00000300);  // Analog Comparator 2
pub const ADC_EMUX_EM2_EXTERNAL = usize(0x00000400);  // External (GPIO Pins)
pub const ADC_EMUX_EM2_TIMER = usize(0x00000500);  // Timer
pub const ADC_EMUX_EM2_PWM0 = usize(0x00000600);  // PWM generator 0
pub const ADC_EMUX_EM2_PWM1 = usize(0x00000700);  // PWM generator 1
pub const ADC_EMUX_EM2_PWM2 = usize(0x00000800);  // PWM generator 2
pub const ADC_EMUX_EM2_PWM3 = usize(0x00000900);  // PWM generator 3
pub const ADC_EMUX_EM2_NEVER = usize(0x00000E00);  // Never Trigger
pub const ADC_EMUX_EM2_ALWAYS = usize(0x00000F00);  // Always (continuously sample)
pub const ADC_EMUX_EM1_M = usize(0x000000F0);  // SS1 Trigger Select
pub const ADC_EMUX_EM1_PROCESSOR = usize(0x00000000);  // Processor (default)
pub const ADC_EMUX_EM1_COMP0 = usize(0x00000010);  // Analog Comparator 0
pub const ADC_EMUX_EM1_COMP1 = usize(0x00000020);  // Analog Comparator 1
pub const ADC_EMUX_EM1_COMP2 = usize(0x00000030);  // Analog Comparator 2
pub const ADC_EMUX_EM1_EXTERNAL = usize(0x00000040);  // External (GPIO Pins)
pub const ADC_EMUX_EM1_TIMER = usize(0x00000050);  // Timer
pub const ADC_EMUX_EM1_PWM0 = usize(0x00000060);  // PWM generator 0
pub const ADC_EMUX_EM1_PWM1 = usize(0x00000070);  // PWM generator 1
pub const ADC_EMUX_EM1_PWM2 = usize(0x00000080);  // PWM generator 2
pub const ADC_EMUX_EM1_PWM3 = usize(0x00000090);  // PWM generator 3
pub const ADC_EMUX_EM1_NEVER = usize(0x000000E0);  // Never Trigger
pub const ADC_EMUX_EM1_ALWAYS = usize(0x000000F0);  // Always (continuously sample)
pub const ADC_EMUX_EM0_M = usize(0x0000000F);  // SS0 Trigger Select
pub const ADC_EMUX_EM0_PROCESSOR = usize(0x00000000);  // Processor (default)
pub const ADC_EMUX_EM0_COMP0 = usize(0x00000001);  // Analog Comparator 0
pub const ADC_EMUX_EM0_COMP1 = usize(0x00000002);  // Analog Comparator 1
pub const ADC_EMUX_EM0_COMP2 = usize(0x00000003);  // Analog Comparator 2
pub const ADC_EMUX_EM0_EXTERNAL = usize(0x00000004);  // External (GPIO Pins)
pub const ADC_EMUX_EM0_TIMER = usize(0x00000005);  // Timer
pub const ADC_EMUX_EM0_PWM0 = usize(0x00000006);  // PWM generator 0
pub const ADC_EMUX_EM0_PWM1 = usize(0x00000007);  // PWM generator 1
pub const ADC_EMUX_EM0_PWM2 = usize(0x00000008);  // PWM generator 2
pub const ADC_EMUX_EM0_PWM3 = usize(0x00000009);  // PWM generator 3
pub const ADC_EMUX_EM0_NEVER = usize(0x0000000E);  // Never Trigger
pub const ADC_EMUX_EM0_ALWAYS = usize(0x0000000F);  // Always (continuously sample)

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_USTAT register.
//
//*****************************************************************************
pub const ADC_USTAT_UV3 = usize(0x00000008);  // SS3 FIFO Underflow
pub const ADC_USTAT_UV2 = usize(0x00000004);  // SS2 FIFO Underflow
pub const ADC_USTAT_UV1 = usize(0x00000002);  // SS1 FIFO Underflow
pub const ADC_USTAT_UV0 = usize(0x00000001);  // SS0 FIFO Underflow

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_TSSEL register.
//
//*****************************************************************************
pub const ADC_TSSEL_PS3_M = usize(0x30000000);  // Generator 3 PWM Module Trigger
                                            // Select
pub const ADC_TSSEL_PS3_0 = usize(0x00000000);  // Use Generator 3 (and its
                                            // trigger) in PWM module 0
pub const ADC_TSSEL_PS3_1 = usize(0x10000000);  // Use Generator 3 (and its
                                            // trigger) in PWM module 1
pub const ADC_TSSEL_PS2_M = usize(0x00300000);  // Generator 2 PWM Module Trigger
                                            // Select
pub const ADC_TSSEL_PS2_0 = usize(0x00000000);  // Use Generator 2 (and its
                                            // trigger) in PWM module 0
pub const ADC_TSSEL_PS2_1 = usize(0x00100000);  // Use Generator 2 (and its
                                            // trigger) in PWM module 1
pub const ADC_TSSEL_PS1_M = usize(0x00003000);  // Generator 1 PWM Module Trigger
                                            // Select
pub const ADC_TSSEL_PS1_0 = usize(0x00000000);  // Use Generator 1 (and its
                                            // trigger) in PWM module 0
pub const ADC_TSSEL_PS1_1 = usize(0x00001000);  // Use Generator 1 (and its
                                            // trigger) in PWM module 1
pub const ADC_TSSEL_PS0_M = usize(0x00000030);  // Generator 0 PWM Module Trigger
                                            // Select
pub const ADC_TSSEL_PS0_0 = usize(0x00000000);  // Use Generator 0 (and its
                                            // trigger) in PWM module 0
pub const ADC_TSSEL_PS0_1 = usize(0x00000010);  // Use Generator 0 (and its
                                            // trigger) in PWM module 1

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSPRI register.
//
//*****************************************************************************
pub const ADC_SSPRI_SS3_M = usize(0x00003000);  // SS3 Priority
pub const ADC_SSPRI_SS2_M = usize(0x00000300);  // SS2 Priority
pub const ADC_SSPRI_SS1_M = usize(0x00000030);  // SS1 Priority
pub const ADC_SSPRI_SS0_M = usize(0x00000003);  // SS0 Priority

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SPC register.
//
//*****************************************************************************
pub const ADC_SPC_PHASE_M = usize(0x0000000F);  // Phase Difference
pub const ADC_SPC_PHASE_0 = usize(0x00000000);  // ADC sample lags by 0.0
pub const ADC_SPC_PHASE_22_5 = usize(0x00000001);  // ADC sample lags by 22.5
pub const ADC_SPC_PHASE_45 = usize(0x00000002);  // ADC sample lags by 45.0
pub const ADC_SPC_PHASE_67_5 = usize(0x00000003);  // ADC sample lags by 67.5
pub const ADC_SPC_PHASE_90 = usize(0x00000004);  // ADC sample lags by 90.0
pub const ADC_SPC_PHASE_112_5 = usize(0x00000005);  // ADC sample lags by 112.5
pub const ADC_SPC_PHASE_135 = usize(0x00000006);  // ADC sample lags by 135.0
pub const ADC_SPC_PHASE_157_5 = usize(0x00000007);  // ADC sample lags by 157.5
pub const ADC_SPC_PHASE_180 = usize(0x00000008);  // ADC sample lags by 180.0
pub const ADC_SPC_PHASE_202_5 = usize(0x00000009);  // ADC sample lags by 202.5
pub const ADC_SPC_PHASE_225 = usize(0x0000000A);  // ADC sample lags by 225.0
pub const ADC_SPC_PHASE_247_5 = usize(0x0000000B);  // ADC sample lags by 247.5
pub const ADC_SPC_PHASE_270 = usize(0x0000000C);  // ADC sample lags by 270.0
pub const ADC_SPC_PHASE_292_5 = usize(0x0000000D);  // ADC sample lags by 292.5
pub const ADC_SPC_PHASE_315 = usize(0x0000000E);  // ADC sample lags by 315.0
pub const ADC_SPC_PHASE_337_5 = usize(0x0000000F);  // ADC sample lags by 337.5

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PSSI register.
//
//*****************************************************************************
pub const ADC_PSSI_GSYNC = usize(0x80000000);  // Global Synchronize
pub const ADC_PSSI_SYNCWAIT = usize(0x08000000);  // Synchronize Wait
pub const ADC_PSSI_SS3 = usize(0x00000008);  // SS3 Initiate
pub const ADC_PSSI_SS2 = usize(0x00000004);  // SS2 Initiate
pub const ADC_PSSI_SS1 = usize(0x00000002);  // SS1 Initiate
pub const ADC_PSSI_SS0 = usize(0x00000001);  // SS0 Initiate

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SAC register.
//
//*****************************************************************************
pub const ADC_SAC_AVG_M = usize(0x00000007);  // Hardware Averaging Control
pub const ADC_SAC_AVG_OFF = usize(0x00000000);  // No hardware oversampling
pub const ADC_SAC_AVG_2X = usize(0x00000001);  // 2x hardware oversampling
pub const ADC_SAC_AVG_4X = usize(0x00000002);  // 4x hardware oversampling
pub const ADC_SAC_AVG_8X = usize(0x00000003);  // 8x hardware oversampling
pub const ADC_SAC_AVG_16X = usize(0x00000004);  // 16x hardware oversampling
pub const ADC_SAC_AVG_32X = usize(0x00000005);  // 32x hardware oversampling
pub const ADC_SAC_AVG_64X = usize(0x00000006);  // 64x hardware oversampling

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCISC register.
//
//*****************************************************************************
pub const ADC_DCISC_DCINT7 = usize(0x00000080);  // Digital Comparator 7 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT6 = usize(0x00000040);  // Digital Comparator 6 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT5 = usize(0x00000020);  // Digital Comparator 5 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT4 = usize(0x00000010);  // Digital Comparator 4 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT3 = usize(0x00000008);  // Digital Comparator 3 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT2 = usize(0x00000004);  // Digital Comparator 2 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT1 = usize(0x00000002);  // Digital Comparator 1 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT0 = usize(0x00000001);  // Digital Comparator 0 Interrupt
                                            // Status and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_CTL register.
//
//*****************************************************************************
pub const ADC_CTL_VREF_M = usize(0x00000003);  // Voltage Reference Select
pub const ADC_CTL_VREF_INTERNAL = usize(0x00000000);  // VDDA and GNDA are the voltage
                                            // references
pub const ADC_CTL_VREF_EXT_3V = usize(0x00000001);  // The external VREFA+ and VREFA-
                                            // inputs are the voltage
                                            // references

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX0 register.
//
//*****************************************************************************
pub const ADC_SSMUX0_MUX7_M = usize(0xF0000000);  // 8th Sample Input Select
pub const ADC_SSMUX0_MUX6_M = usize(0x0F000000);  // 7th Sample Input Select
pub const ADC_SSMUX0_MUX5_M = usize(0x00F00000);  // 6th Sample Input Select
pub const ADC_SSMUX0_MUX4_M = usize(0x000F0000);  // 5th Sample Input Select
pub const ADC_SSMUX0_MUX3_M = usize(0x0000F000);  // 4th Sample Input Select
pub const ADC_SSMUX0_MUX2_M = usize(0x00000F00);  // 3rd Sample Input Select
pub const ADC_SSMUX0_MUX1_M = usize(0x000000F0);  // 2nd Sample Input Select
pub const ADC_SSMUX0_MUX0_M = usize(0x0000000F);  // 1st Sample Input Select
pub const ADC_SSMUX0_MUX7_S = usize(28);
pub const ADC_SSMUX0_MUX6_S = usize(24);
pub const ADC_SSMUX0_MUX5_S = usize(20);
pub const ADC_SSMUX0_MUX4_S = usize(16);
pub const ADC_SSMUX0_MUX3_S = usize(12);
pub const ADC_SSMUX0_MUX2_S = usize(8);
pub const ADC_SSMUX0_MUX1_S = usize(4);
pub const ADC_SSMUX0_MUX0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL0 register.
//
//*****************************************************************************
pub const ADC_SSCTL0_TS7 = usize(0x80000000);  // 8th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE7 = usize(0x40000000);  // 8th Sample Interrupt Enable
pub const ADC_SSCTL0_END7 = usize(0x20000000);  // 8th Sample is End of Sequence
pub const ADC_SSCTL0_D7 = usize(0x10000000);  // 8th Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS6 = usize(0x08000000);  // 7th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE6 = usize(0x04000000);  // 7th Sample Interrupt Enable
pub const ADC_SSCTL0_END6 = usize(0x02000000);  // 7th Sample is End of Sequence
pub const ADC_SSCTL0_D6 = usize(0x01000000);  // 7th Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS5 = usize(0x00800000);  // 6th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE5 = usize(0x00400000);  // 6th Sample Interrupt Enable
pub const ADC_SSCTL0_END5 = usize(0x00200000);  // 6th Sample is End of Sequence
pub const ADC_SSCTL0_D5 = usize(0x00100000);  // 6th Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS4 = usize(0x00080000);  // 5th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE4 = usize(0x00040000);  // 5th Sample Interrupt Enable
pub const ADC_SSCTL0_END4 = usize(0x00020000);  // 5th Sample is End of Sequence
pub const ADC_SSCTL0_D4 = usize(0x00010000);  // 5th Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS3 = usize(0x00008000);  // 4th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE3 = usize(0x00004000);  // 4th Sample Interrupt Enable
pub const ADC_SSCTL0_END3 = usize(0x00002000);  // 4th Sample is End of Sequence
pub const ADC_SSCTL0_D3 = usize(0x00001000);  // 4th Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS2 = usize(0x00000800);  // 3rd Sample Temp Sensor Select
pub const ADC_SSCTL0_IE2 = usize(0x00000400);  // 3rd Sample Interrupt Enable
pub const ADC_SSCTL0_END2 = usize(0x00000200);  // 3rd Sample is End of Sequence
pub const ADC_SSCTL0_D2 = usize(0x00000100);  // 3rd Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS1 = usize(0x00000080);  // 2nd Sample Temp Sensor Select
pub const ADC_SSCTL0_IE1 = usize(0x00000040);  // 2nd Sample Interrupt Enable
pub const ADC_SSCTL0_END1 = usize(0x00000020);  // 2nd Sample is End of Sequence
pub const ADC_SSCTL0_D1 = usize(0x00000010);  // 2nd Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS0 = usize(0x00000008);  // 1st Sample Temp Sensor Select
pub const ADC_SSCTL0_IE0 = usize(0x00000004);  // 1st Sample Interrupt Enable
pub const ADC_SSCTL0_END0 = usize(0x00000002);  // 1st Sample is End of Sequence
pub const ADC_SSCTL0_D0 = usize(0x00000001);  // 1st Sample Differential Input
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO0 register.
//
//*****************************************************************************
pub const ADC_SSFIFO0_DATA_M = usize(0x00000FFF);  // Conversion Result Data
pub const ADC_SSFIFO0_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT0 register.
//
//*****************************************************************************
pub const ADC_SSFSTAT0_FULL = usize(0x00001000);  // FIFO Full
pub const ADC_SSFSTAT0_EMPTY = usize(0x00000100);  // FIFO Empty
pub const ADC_SSFSTAT0_HPTR_M = usize(0x000000F0);  // FIFO Head Pointer
pub const ADC_SSFSTAT0_TPTR_M = usize(0x0000000F);  // FIFO Tail Pointer
pub const ADC_SSFSTAT0_HPTR_S = usize(4);
pub const ADC_SSFSTAT0_TPTR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP0 register.
//
//*****************************************************************************
pub const ADC_SSOP0_S7DCOP = usize(0x10000000);  // Sample 7 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S6DCOP = usize(0x01000000);  // Sample 6 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S5DCOP = usize(0x00100000);  // Sample 5 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S4DCOP = usize(0x00010000);  // Sample 4 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S3DCOP = usize(0x00001000);  // Sample 3 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S2DCOP = usize(0x00000100);  // Sample 2 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S1DCOP = usize(0x00000010);  // Sample 1 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S0DCOP = usize(0x00000001);  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC0 register.
//
//*****************************************************************************
pub const ADC_SSDC0_S7DCSEL_M = usize(0xF0000000);  // Sample 7 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S6DCSEL_M = usize(0x0F000000);  // Sample 6 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S5DCSEL_M = usize(0x00F00000);  // Sample 5 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S4DCSEL_M = usize(0x000F0000);  // Sample 4 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S3DCSEL_M = usize(0x0000F000);  // Sample 3 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S2DCSEL_M = usize(0x00000F00);  // Sample 2 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S1DCSEL_M = usize(0x000000F0);  // Sample 1 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S0DCSEL_M = usize(0x0000000F);  // Sample 0 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S6DCSEL_S = usize(24);
pub const ADC_SSDC0_S5DCSEL_S = usize(20);
pub const ADC_SSDC0_S4DCSEL_S = usize(16);
pub const ADC_SSDC0_S3DCSEL_S = usize(12);
pub const ADC_SSDC0_S2DCSEL_S = usize(8);
pub const ADC_SSDC0_S1DCSEL_S = usize(4);
pub const ADC_SSDC0_S0DCSEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX0 register.
//
//*****************************************************************************
pub const ADC_SSEMUX0_EMUX7 = usize(0x10000000);  // 8th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX6 = usize(0x01000000);  // 7th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX5 = usize(0x00100000);  // 6th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX4 = usize(0x00010000);  // 5th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX3 = usize(0x00001000);  // 4th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX2 = usize(0x00000100);  // 3rd Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX1 = usize(0x00000010);  // 2th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX0 = usize(0x00000001);  // 1st Sample Input Select (Upper
                                            // Bit)

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH0 register.
//
//*****************************************************************************
pub const ADC_SSTSH0_TSH7_M = usize(0xF0000000);  // 8th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH6_M = usize(0x0F000000);  // 7th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH5_M = usize(0x00F00000);  // 6th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH4_M = usize(0x000F0000);  // 5th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH3_M = usize(0x0000F000);  // 4th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH2_M = usize(0x00000F00);  // 3rd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH1_M = usize(0x000000F0);  // 2nd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH0_M = usize(0x0000000F);  // 1st Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH7_S = usize(28);
pub const ADC_SSTSH0_TSH6_S = usize(24);
pub const ADC_SSTSH0_TSH5_S = usize(20);
pub const ADC_SSTSH0_TSH4_S = usize(16);
pub const ADC_SSTSH0_TSH3_S = usize(12);
pub const ADC_SSTSH0_TSH2_S = usize(8);
pub const ADC_SSTSH0_TSH1_S = usize(4);
pub const ADC_SSTSH0_TSH0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX1 register.
//
//*****************************************************************************
pub const ADC_SSMUX1_MUX3_M = usize(0x0000F000);  // 4th Sample Input Select
pub const ADC_SSMUX1_MUX2_M = usize(0x00000F00);  // 3rd Sample Input Select
pub const ADC_SSMUX1_MUX1_M = usize(0x000000F0);  // 2nd Sample Input Select
pub const ADC_SSMUX1_MUX0_M = usize(0x0000000F);  // 1st Sample Input Select
pub const ADC_SSMUX1_MUX3_S = usize(12);
pub const ADC_SSMUX1_MUX2_S = usize(8);
pub const ADC_SSMUX1_MUX1_S = usize(4);
pub const ADC_SSMUX1_MUX0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL1 register.
//
//*****************************************************************************
pub const ADC_SSCTL1_TS3 = usize(0x00008000);  // 4th Sample Temp Sensor Select
pub const ADC_SSCTL1_IE3 = usize(0x00004000);  // 4th Sample Interrupt Enable
pub const ADC_SSCTL1_END3 = usize(0x00002000);  // 4th Sample is End of Sequence
pub const ADC_SSCTL1_D3 = usize(0x00001000);  // 4th Sample Differential Input
                                            // Select
pub const ADC_SSCTL1_TS2 = usize(0x00000800);  // 3rd Sample Temp Sensor Select
pub const ADC_SSCTL1_IE2 = usize(0x00000400);  // 3rd Sample Interrupt Enable
pub const ADC_SSCTL1_END2 = usize(0x00000200);  // 3rd Sample is End of Sequence
pub const ADC_SSCTL1_D2 = usize(0x00000100);  // 3rd Sample Differential Input
                                            // Select
pub const ADC_SSCTL1_TS1 = usize(0x00000080);  // 2nd Sample Temp Sensor Select
pub const ADC_SSCTL1_IE1 = usize(0x00000040);  // 2nd Sample Interrupt Enable
pub const ADC_SSCTL1_END1 = usize(0x00000020);  // 2nd Sample is End of Sequence
pub const ADC_SSCTL1_D1 = usize(0x00000010);  // 2nd Sample Differential Input
                                            // Select
pub const ADC_SSCTL1_TS0 = usize(0x00000008);  // 1st Sample Temp Sensor Select
pub const ADC_SSCTL1_IE0 = usize(0x00000004);  // 1st Sample Interrupt Enable
pub const ADC_SSCTL1_END0 = usize(0x00000002);  // 1st Sample is End of Sequence
pub const ADC_SSCTL1_D0 = usize(0x00000001);  // 1st Sample Differential Input
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO1 register.
//
//*****************************************************************************
pub const ADC_SSFIFO1_DATA_M = usize(0x00000FFF);  // Conversion Result Data
pub const ADC_SSFIFO1_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT1 register.
//
//*****************************************************************************
pub const ADC_SSFSTAT1_FULL = usize(0x00001000);  // FIFO Full
pub const ADC_SSFSTAT1_EMPTY = usize(0x00000100);  // FIFO Empty
pub const ADC_SSFSTAT1_HPTR_M = usize(0x000000F0);  // FIFO Head Pointer
pub const ADC_SSFSTAT1_TPTR_M = usize(0x0000000F);  // FIFO Tail Pointer
pub const ADC_SSFSTAT1_HPTR_S = usize(4);
pub const ADC_SSFSTAT1_TPTR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP1 register.
//
//*****************************************************************************
pub const ADC_SSOP1_S3DCOP = usize(0x00001000);  // Sample 3 Digital Comparator
                                            // Operation
pub const ADC_SSOP1_S2DCOP = usize(0x00000100);  // Sample 2 Digital Comparator
                                            // Operation
pub const ADC_SSOP1_S1DCOP = usize(0x00000010);  // Sample 1 Digital Comparator
                                            // Operation
pub const ADC_SSOP1_S0DCOP = usize(0x00000001);  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC1 register.
//
//*****************************************************************************
pub const ADC_SSDC1_S3DCSEL_M = usize(0x0000F000);  // Sample 3 Digital Comparator
                                            // Select
pub const ADC_SSDC1_S2DCSEL_M = usize(0x00000F00);  // Sample 2 Digital Comparator
                                            // Select
pub const ADC_SSDC1_S1DCSEL_M = usize(0x000000F0);  // Sample 1 Digital Comparator
                                            // Select
pub const ADC_SSDC1_S0DCSEL_M = usize(0x0000000F);  // Sample 0 Digital Comparator
                                            // Select
pub const ADC_SSDC1_S2DCSEL_S = usize(8);
pub const ADC_SSDC1_S1DCSEL_S = usize(4);
pub const ADC_SSDC1_S0DCSEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX1 register.
//
//*****************************************************************************
pub const ADC_SSEMUX1_EMUX3 = usize(0x00001000);  // 4th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX1_EMUX2 = usize(0x00000100);  // 3rd Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX1_EMUX1 = usize(0x00000010);  // 2th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX1_EMUX0 = usize(0x00000001);  // 1st Sample Input Select (Upper
                                            // Bit)

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH1 register.
//
//*****************************************************************************
pub const ADC_SSTSH1_TSH3_M = usize(0x0000F000);  // 4th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH1_TSH2_M = usize(0x00000F00);  // 3rd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH1_TSH1_M = usize(0x000000F0);  // 2nd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH1_TSH0_M = usize(0x0000000F);  // 1st Sample and Hold Period
                                            // Select
pub const ADC_SSTSH1_TSH3_S = usize(12);
pub const ADC_SSTSH1_TSH2_S = usize(8);
pub const ADC_SSTSH1_TSH1_S = usize(4);
pub const ADC_SSTSH1_TSH0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX2 register.
//
//*****************************************************************************
pub const ADC_SSMUX2_MUX3_M = usize(0x0000F000);  // 4th Sample Input Select
pub const ADC_SSMUX2_MUX2_M = usize(0x00000F00);  // 3rd Sample Input Select
pub const ADC_SSMUX2_MUX1_M = usize(0x000000F0);  // 2nd Sample Input Select
pub const ADC_SSMUX2_MUX0_M = usize(0x0000000F);  // 1st Sample Input Select
pub const ADC_SSMUX2_MUX3_S = usize(12);
pub const ADC_SSMUX2_MUX2_S = usize(8);
pub const ADC_SSMUX2_MUX1_S = usize(4);
pub const ADC_SSMUX2_MUX0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL2 register.
//
//*****************************************************************************
pub const ADC_SSCTL2_TS3 = usize(0x00008000);  // 4th Sample Temp Sensor Select
pub const ADC_SSCTL2_IE3 = usize(0x00004000);  // 4th Sample Interrupt Enable
pub const ADC_SSCTL2_END3 = usize(0x00002000);  // 4th Sample is End of Sequence
pub const ADC_SSCTL2_D3 = usize(0x00001000);  // 4th Sample Differential Input
                                            // Select
pub const ADC_SSCTL2_TS2 = usize(0x00000800);  // 3rd Sample Temp Sensor Select
pub const ADC_SSCTL2_IE2 = usize(0x00000400);  // 3rd Sample Interrupt Enable
pub const ADC_SSCTL2_END2 = usize(0x00000200);  // 3rd Sample is End of Sequence
pub const ADC_SSCTL2_D2 = usize(0x00000100);  // 3rd Sample Differential Input
                                            // Select
pub const ADC_SSCTL2_TS1 = usize(0x00000080);  // 2nd Sample Temp Sensor Select
pub const ADC_SSCTL2_IE1 = usize(0x00000040);  // 2nd Sample Interrupt Enable
pub const ADC_SSCTL2_END1 = usize(0x00000020);  // 2nd Sample is End of Sequence
pub const ADC_SSCTL2_D1 = usize(0x00000010);  // 2nd Sample Differential Input
                                            // Select
pub const ADC_SSCTL2_TS0 = usize(0x00000008);  // 1st Sample Temp Sensor Select
pub const ADC_SSCTL2_IE0 = usize(0x00000004);  // 1st Sample Interrupt Enable
pub const ADC_SSCTL2_END0 = usize(0x00000002);  // 1st Sample is End of Sequence
pub const ADC_SSCTL2_D0 = usize(0x00000001);  // 1st Sample Differential Input
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO2 register.
//
//*****************************************************************************
pub const ADC_SSFIFO2_DATA_M = usize(0x00000FFF);  // Conversion Result Data
pub const ADC_SSFIFO2_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT2 register.
//
//*****************************************************************************
pub const ADC_SSFSTAT2_FULL = usize(0x00001000);  // FIFO Full
pub const ADC_SSFSTAT2_EMPTY = usize(0x00000100);  // FIFO Empty
pub const ADC_SSFSTAT2_HPTR_M = usize(0x000000F0);  // FIFO Head Pointer
pub const ADC_SSFSTAT2_TPTR_M = usize(0x0000000F);  // FIFO Tail Pointer
pub const ADC_SSFSTAT2_HPTR_S = usize(4);
pub const ADC_SSFSTAT2_TPTR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP2 register.
//
//*****************************************************************************
pub const ADC_SSOP2_S3DCOP = usize(0x00001000);  // Sample 3 Digital Comparator
                                            // Operation
pub const ADC_SSOP2_S2DCOP = usize(0x00000100);  // Sample 2 Digital Comparator
                                            // Operation
pub const ADC_SSOP2_S1DCOP = usize(0x00000010);  // Sample 1 Digital Comparator
                                            // Operation
pub const ADC_SSOP2_S0DCOP = usize(0x00000001);  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC2 register.
//
//*****************************************************************************
pub const ADC_SSDC2_S3DCSEL_M = usize(0x0000F000);  // Sample 3 Digital Comparator
                                            // Select
pub const ADC_SSDC2_S2DCSEL_M = usize(0x00000F00);  // Sample 2 Digital Comparator
                                            // Select
pub const ADC_SSDC2_S1DCSEL_M = usize(0x000000F0);  // Sample 1 Digital Comparator
                                            // Select
pub const ADC_SSDC2_S0DCSEL_M = usize(0x0000000F);  // Sample 0 Digital Comparator
                                            // Select
pub const ADC_SSDC2_S2DCSEL_S = usize(8);
pub const ADC_SSDC2_S1DCSEL_S = usize(4);
pub const ADC_SSDC2_S0DCSEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX2 register.
//
//*****************************************************************************
pub const ADC_SSEMUX2_EMUX3 = usize(0x00001000);  // 4th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX2_EMUX2 = usize(0x00000100);  // 3rd Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX2_EMUX1 = usize(0x00000010);  // 2th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX2_EMUX0 = usize(0x00000001);  // 1st Sample Input Select (Upper
                                            // Bit)

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH2 register.
//
//*****************************************************************************
pub const ADC_SSTSH2_TSH3_M = usize(0x0000F000);  // 4th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH2_TSH2_M = usize(0x00000F00);  // 3rd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH2_TSH1_M = usize(0x000000F0);  // 2nd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH2_TSH0_M = usize(0x0000000F);  // 1st Sample and Hold Period
                                            // Select
pub const ADC_SSTSH2_TSH3_S = usize(12);
pub const ADC_SSTSH2_TSH2_S = usize(8);
pub const ADC_SSTSH2_TSH1_S = usize(4);
pub const ADC_SSTSH2_TSH0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX3 register.
//
//*****************************************************************************
pub const ADC_SSMUX3_MUX0_M = usize(0x0000000F);  // 1st Sample Input Select
pub const ADC_SSMUX3_MUX0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL3 register.
//
//*****************************************************************************
pub const ADC_SSCTL3_TS0 = usize(0x00000008);  // 1st Sample Temp Sensor Select
pub const ADC_SSCTL3_IE0 = usize(0x00000004);  // Sample Interrupt Enable
pub const ADC_SSCTL3_END0 = usize(0x00000002);  // End of Sequence
pub const ADC_SSCTL3_D0 = usize(0x00000001);  // Sample Differential Input Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO3 register.
//
//*****************************************************************************
pub const ADC_SSFIFO3_DATA_M = usize(0x00000FFF);  // Conversion Result Data
pub const ADC_SSFIFO3_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT3 register.
//
//*****************************************************************************
pub const ADC_SSFSTAT3_FULL = usize(0x00001000);  // FIFO Full
pub const ADC_SSFSTAT3_EMPTY = usize(0x00000100);  // FIFO Empty
pub const ADC_SSFSTAT3_HPTR_M = usize(0x000000F0);  // FIFO Head Pointer
pub const ADC_SSFSTAT3_TPTR_M = usize(0x0000000F);  // FIFO Tail Pointer
pub const ADC_SSFSTAT3_HPTR_S = usize(4);
pub const ADC_SSFSTAT3_TPTR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP3 register.
//
//*****************************************************************************
pub const ADC_SSOP3_S0DCOP = usize(0x00000001);  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC3 register.
//
//*****************************************************************************
pub const ADC_SSDC3_S0DCSEL_M = usize(0x0000000F);  // Sample 0 Digital Comparator
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX3 register.
//
//*****************************************************************************
pub const ADC_SSEMUX3_EMUX0 = usize(0x00000001);  // 1st Sample Input Select (Upper
                                            // Bit)

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH3 register.
//
//*****************************************************************************
pub const ADC_SSTSH3_TSH0_M = usize(0x0000000F);  // 1st Sample and Hold Period
                                            // Select
pub const ADC_SSTSH3_TSH0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCRIC register.
//
//*****************************************************************************
pub const ADC_DCRIC_DCTRIG7 = usize(0x00800000);  // Digital Comparator Trigger 7
pub const ADC_DCRIC_DCTRIG6 = usize(0x00400000);  // Digital Comparator Trigger 6
pub const ADC_DCRIC_DCTRIG5 = usize(0x00200000);  // Digital Comparator Trigger 5
pub const ADC_DCRIC_DCTRIG4 = usize(0x00100000);  // Digital Comparator Trigger 4
pub const ADC_DCRIC_DCTRIG3 = usize(0x00080000);  // Digital Comparator Trigger 3
pub const ADC_DCRIC_DCTRIG2 = usize(0x00040000);  // Digital Comparator Trigger 2
pub const ADC_DCRIC_DCTRIG1 = usize(0x00020000);  // Digital Comparator Trigger 1
pub const ADC_DCRIC_DCTRIG0 = usize(0x00010000);  // Digital Comparator Trigger 0
pub const ADC_DCRIC_DCINT7 = usize(0x00000080);  // Digital Comparator Interrupt 7
pub const ADC_DCRIC_DCINT6 = usize(0x00000040);  // Digital Comparator Interrupt 6
pub const ADC_DCRIC_DCINT5 = usize(0x00000020);  // Digital Comparator Interrupt 5
pub const ADC_DCRIC_DCINT4 = usize(0x00000010);  // Digital Comparator Interrupt 4
pub const ADC_DCRIC_DCINT3 = usize(0x00000008);  // Digital Comparator Interrupt 3
pub const ADC_DCRIC_DCINT2 = usize(0x00000004);  // Digital Comparator Interrupt 2
pub const ADC_DCRIC_DCINT1 = usize(0x00000002);  // Digital Comparator Interrupt 1
pub const ADC_DCRIC_DCINT0 = usize(0x00000001);  // Digital Comparator Interrupt 0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL0 register.
//
//*****************************************************************************
pub const ADC_DCCTL0_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL0_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL0_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL0_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL0_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL0_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL0_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL0_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL0_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL0_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL0_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL0_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL0_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL0_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL0_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL0_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL0_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL0_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL0_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL0_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL1 register.
//
//*****************************************************************************
pub const ADC_DCCTL1_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL1_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL1_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL1_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL1_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL1_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL1_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL1_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL1_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL1_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL1_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL1_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL1_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL1_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL1_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL1_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL1_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL1_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL1_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL1_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL2 register.
//
//*****************************************************************************
pub const ADC_DCCTL2_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL2_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL2_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL2_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL2_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL2_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL2_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL2_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL2_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL2_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL2_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL2_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL2_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL2_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL2_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL2_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL2_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL2_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL2_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL2_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL3 register.
//
//*****************************************************************************
pub const ADC_DCCTL3_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL3_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL3_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL3_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL3_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL3_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL3_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL3_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL3_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL3_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL3_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL3_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL3_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL3_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL3_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL3_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL3_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL3_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL3_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL3_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL4 register.
//
//*****************************************************************************
pub const ADC_DCCTL4_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL4_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL4_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL4_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL4_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL4_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL4_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL4_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL4_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL4_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL4_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL4_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL4_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL4_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL4_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL4_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL4_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL4_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL4_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL4_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL5 register.
//
//*****************************************************************************
pub const ADC_DCCTL5_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL5_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL5_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL5_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL5_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL5_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL5_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL5_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL5_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL5_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL5_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL5_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL5_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL5_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL5_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL5_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL5_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL5_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL5_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL5_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL6 register.
//
//*****************************************************************************
pub const ADC_DCCTL6_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL6_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL6_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL6_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL6_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL6_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL6_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL6_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL6_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL6_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL6_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL6_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL6_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL6_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL6_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL6_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL6_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL6_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL6_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL6_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL7 register.
//
//*****************************************************************************
pub const ADC_DCCTL7_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL7_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL7_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL7_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL7_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL7_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL7_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL7_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL7_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL7_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL7_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL7_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL7_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL7_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL7_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL7_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL7_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL7_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL7_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL7_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP0 register.
//
//*****************************************************************************
pub const ADC_DCCMP0_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP0_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP0_COMP1_S = usize(16);
pub const ADC_DCCMP0_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP1 register.
//
//*****************************************************************************
pub const ADC_DCCMP1_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP1_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP1_COMP1_S = usize(16);
pub const ADC_DCCMP1_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP2 register.
//
//*****************************************************************************
pub const ADC_DCCMP2_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP2_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP2_COMP1_S = usize(16);
pub const ADC_DCCMP2_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP3 register.
//
//*****************************************************************************
pub const ADC_DCCMP3_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP3_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP3_COMP1_S = usize(16);
pub const ADC_DCCMP3_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP4 register.
//
//*****************************************************************************
pub const ADC_DCCMP4_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP4_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP4_COMP1_S = usize(16);
pub const ADC_DCCMP4_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP5 register.
//
//*****************************************************************************
pub const ADC_DCCMP5_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP5_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP5_COMP1_S = usize(16);
pub const ADC_DCCMP5_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP6 register.
//
//*****************************************************************************
pub const ADC_DCCMP6_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP6_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP6_COMP1_S = usize(16);
pub const ADC_DCCMP6_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP7 register.
//
//*****************************************************************************
pub const ADC_DCCMP7_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP7_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP7_COMP1_S = usize(16);
pub const ADC_DCCMP7_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PP register.
//
//*****************************************************************************
pub const ADC_PP_APSHT = usize(0x01000000);  // Application-Programmable
                                            // Sample-and-Hold Time
pub const ADC_PP_TS = usize(0x00800000);  // Temperature Sensor
pub const ADC_PP_RSL_M = usize(0x007C0000);  // Resolution
pub const ADC_PP_TYPE_M = usize(0x00030000);  // ADC Architecture
pub const ADC_PP_TYPE_SAR = usize(0x00000000);  // SAR
pub const ADC_PP_DC_M = usize(0x0000FC00);  // Digital Comparator Count
pub const ADC_PP_CH_M = usize(0x000003F0);  // ADC Channel Count
pub const ADC_PP_MCR_M = usize(0x0000000F);  // Maximum Conversion Rate
pub const ADC_PP_MCR_FULL = usize(0x00000007);  // Full conversion rate (FCONV) as
                                            // defined by TADC and NSH
pub const ADC_PP_MSR_M = usize(0x0000000F);  // Maximum ADC Sample Rate
pub const ADC_PP_MSR_125K = usize(0x00000001);  // 125 ksps
pub const ADC_PP_MSR_250K = usize(0x00000003);  // 250 ksps
pub const ADC_PP_MSR_500K = usize(0x00000005);  // 500 ksps
pub const ADC_PP_MSR_1M = usize(0x00000007);  // 1 Msps
pub const ADC_PP_RSL_S = usize(18);
pub const ADC_PP_DC_S = usize(10);
pub const ADC_PP_CH_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PC register.
//
//*****************************************************************************
pub const ADC_PC_SR_M = usize(0x0000000F);  // ADC Sample Rate
pub const ADC_PC_SR_125K = usize(0x00000001);  // 125 ksps
pub const ADC_PC_SR_250K = usize(0x00000003);  // 250 ksps
pub const ADC_PC_SR_500K = usize(0x00000005);  // 500 ksps
pub const ADC_PC_SR_1M = usize(0x00000007);  // 1 Msps
pub const ADC_PC_MCR_M = usize(0x0000000F);  // Conversion Rate
pub const ADC_PC_MCR_1_8 = usize(0x00000001);  // Eighth conversion rate. After a
                                            // conversion completes, the logic
                                            // pauses for 112 TADC periods
                                            // before starting the next
                                            // conversion
pub const ADC_PC_MCR_1_4 = usize(0x00000003);  // Quarter conversion rate. After a
                                            // conversion completes, the logic
                                            // pauses for 48 TADC periods
                                            // before starting the next
                                            // conversion
pub const ADC_PC_MCR_1_2 = usize(0x00000005);  // Half conversion rate. After a
                                            // conversion completes, the logic
                                            // pauses for 16 TADC periods
                                            // before starting the next
                                            // conversion
pub const ADC_PC_MCR_FULL = usize(0x00000007);  // Full conversion rate (FCONV) as
                                            // defined by TADC and NSH

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_CC register.
//
//*****************************************************************************
pub const ADC_CC_CLKDIV_M = usize(0x000003F0);  // PLL VCO Clock Divisor
pub const ADC_CC_CS_M = usize(0x0000000F);  // ADC Clock Source
pub const ADC_CC_CS_SYSPLL = usize(0x00000000);  // PLL VCO divided by CLKDIV
pub const ADC_CC_CS_PIOSC = usize(0x00000001);  // PIOSC
pub const ADC_CC_CS_MOSC = usize(0x00000002);  // MOSC
pub const ADC_CC_CLKDIV_S = usize(4);

