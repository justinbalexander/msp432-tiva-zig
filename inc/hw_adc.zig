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
pub const offsetOf = struct {
    pub const ACTSS = usize(0x00000000); // ADC Active Sample Sequencer
    pub const RIS = usize(0x00000004); // ADC Raw Interrupt Status
    pub const IM = usize(0x00000008); // ADC Interrupt Mask
    pub const ISC = usize(0x0000000C); // ADC Interrupt Status and Clear
    pub const OSTAT = usize(0x00000010); // ADC Overflow Status
    pub const EMUX = usize(0x00000014); // ADC Event Multiplexer Select
    pub const USTAT = usize(0x00000018); // ADC Underflow Status
    pub const TSSEL = usize(0x0000001C); // ADC Trigger Source Select
    pub const SSPRI = usize(0x00000020); // ADC Sample Sequencer Priority
    pub const SPC = usize(0x00000024); // ADC Sample Phase Control
    pub const PSSI = usize(0x00000028); // ADC Processor Sample Sequence
    // Initiate
    pub const SAC = usize(0x00000030); // ADC Sample Averaging Control
    pub const DCISC = usize(0x00000034); // ADC Digital Comparator Interrupt
    // Status and Clear
    pub const CTL = usize(0x00000038); // ADC Control
    pub const SSMUX0 = usize(0x00000040); // ADC Sample Sequence Input
    // Multiplexer Select 0
    pub const SSCTL0 = usize(0x00000044); // ADC Sample Sequence Control 0
    pub const SSFIFO0 = usize(0x00000048); // ADC Sample Sequence Result FIFO
    // 0
    pub const SSFSTAT0 = usize(0x0000004C); // ADC Sample Sequence FIFO 0
    // Status
    pub const SSOP0 = usize(0x00000050); // ADC Sample Sequence 0 Operation
    pub const SSDC0 = usize(0x00000054); // ADC Sample Sequence 0 Digital
    // Comparator Select
    pub const SSEMUX0 = usize(0x00000058); // ADC Sample Sequence Extended
    // Input Multiplexer Select 0
    pub const SSTSH0 = usize(0x0000005C); // ADC Sample Sequence 0 Sample and
    // Hold Time
    pub const SSMUX1 = usize(0x00000060); // ADC Sample Sequence Input
    // Multiplexer Select 1
    pub const SSCTL1 = usize(0x00000064); // ADC Sample Sequence Control 1
    pub const SSFIFO1 = usize(0x00000068); // ADC Sample Sequence Result FIFO
    // 1
    pub const SSFSTAT1 = usize(0x0000006C); // ADC Sample Sequence FIFO 1
    // Status
    pub const SSOP1 = usize(0x00000070); // ADC Sample Sequence 1 Operation
    pub const SSDC1 = usize(0x00000074); // ADC Sample Sequence 1 Digital
    // Comparator Select
    pub const SSEMUX1 = usize(0x00000078); // ADC Sample Sequence Extended
    // Input Multiplexer Select 1
    pub const SSTSH1 = usize(0x0000007C); // ADC Sample Sequence 1 Sample and
    // Hold Time
    pub const SSMUX2 = usize(0x00000080); // ADC Sample Sequence Input
    // Multiplexer Select 2
    pub const SSCTL2 = usize(0x00000084); // ADC Sample Sequence Control 2
    pub const SSFIFO2 = usize(0x00000088); // ADC Sample Sequence Result FIFO
    // 2
    pub const SSFSTAT2 = usize(0x0000008C); // ADC Sample Sequence FIFO 2
    // Status
    pub const SSOP2 = usize(0x00000090); // ADC Sample Sequence 2 Operation
    pub const SSDC2 = usize(0x00000094); // ADC Sample Sequence 2 Digital
    // Comparator Select
    pub const SSEMUX2 = usize(0x00000098); // ADC Sample Sequence Extended
    // Input Multiplexer Select 2
    pub const SSTSH2 = usize(0x0000009C); // ADC Sample Sequence 2 Sample and
    // Hold Time
    pub const SSMUX3 = usize(0x000000A0); // ADC Sample Sequence Input
    // Multiplexer Select 3
    pub const SSCTL3 = usize(0x000000A4); // ADC Sample Sequence Control 3
    pub const SSFIFO3 = usize(0x000000A8); // ADC Sample Sequence Result FIFO
    // 3
    pub const SSFSTAT3 = usize(0x000000AC); // ADC Sample Sequence FIFO 3
    // Status
    pub const SSOP3 = usize(0x000000B0); // ADC Sample Sequence 3 Operation
    pub const SSDC3 = usize(0x000000B4); // ADC Sample Sequence 3 Digital
    // Comparator Select
    pub const SSEMUX3 = usize(0x000000B8); // ADC Sample Sequence Extended
    // Input Multiplexer Select 3
    pub const SSTSH3 = usize(0x000000BC); // ADC Sample Sequence 3 Sample and
    // Hold Time
    pub const DCRIC = usize(0x00000D00); // ADC Digital Comparator Reset
    // Initial Conditions
    pub const DCCTL0 = usize(0x00000E00); // ADC Digital Comparator Control 0
    pub const DCCTL1 = usize(0x00000E04); // ADC Digital Comparator Control 1
    pub const DCCTL2 = usize(0x00000E08); // ADC Digital Comparator Control 2
    pub const DCCTL3 = usize(0x00000E0C); // ADC Digital Comparator Control 3
    pub const DCCTL4 = usize(0x00000E10); // ADC Digital Comparator Control 4
    pub const DCCTL5 = usize(0x00000E14); // ADC Digital Comparator Control 5
    pub const DCCTL6 = usize(0x00000E18); // ADC Digital Comparator Control 6
    pub const DCCTL7 = usize(0x00000E1C); // ADC Digital Comparator Control 7
    pub const DCCMP0 = usize(0x00000E40); // ADC Digital Comparator Range 0
    pub const DCCMP1 = usize(0x00000E44); // ADC Digital Comparator Range 1
    pub const DCCMP2 = usize(0x00000E48); // ADC Digital Comparator Range 2
    pub const DCCMP3 = usize(0x00000E4C); // ADC Digital Comparator Range 3
    pub const DCCMP4 = usize(0x00000E50); // ADC Digital Comparator Range 4
    pub const DCCMP5 = usize(0x00000E54); // ADC Digital Comparator Range 5
    pub const DCCMP6 = usize(0x00000E58); // ADC Digital Comparator Range 6
    pub const DCCMP7 = usize(0x00000E5C); // ADC Digital Comparator Range 7
    pub const PP = usize(0x00000FC0); // ADC Peripheral Properties
    pub const PC = usize(0x00000FC4); // ADC Peripheral Configuration
    pub const CC = usize(0x00000FC8); // ADC Clock Configuration
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_ACTSS register.
//
//*****************************************************************************
pub const ACTSS = struct {
    pub const BUSY = usize(0x00010000); // ADC Busy
    pub const ADEN3 = usize(0x00000800); // ADC SS3 DMA Enable
    pub const ADEN2 = usize(0x00000400); // ADC SS2 DMA Enable
    pub const ADEN1 = usize(0x00000200); // ADC SS1 DMA Enable
    pub const ADEN0 = usize(0x00000100); // ADC SS1 DMA Enable
    pub const ASEN3 = usize(0x00000008); // ADC SS3 Enable
    pub const ASEN2 = usize(0x00000004); // ADC SS2 Enable
    pub const ASEN1 = usize(0x00000002); // ADC SS1 Enable
    pub const ASEN0 = usize(0x00000001); // ADC SS0 Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_RIS register.
//
//*****************************************************************************
pub const RIS = struct {
    pub const INRDC = usize(0x00010000); // Digital Comparator Raw Interrupt
    // Status
    pub const DMAINR3 = usize(0x00000800); // SS3 DMA Raw Interrupt Status
    pub const DMAINR2 = usize(0x00000400); // SS2 DMA Raw Interrupt Status
    pub const DMAINR1 = usize(0x00000200); // SS1 DMA Raw Interrupt Status
    pub const DMAINR0 = usize(0x00000100); // SS0 DMA Raw Interrupt Status
    pub const INR3 = usize(0x00000008); // SS3 Raw Interrupt Status
    pub const INR2 = usize(0x00000004); // SS2 Raw Interrupt Status
    pub const INR1 = usize(0x00000002); // SS1 Raw Interrupt Status
    pub const INR0 = usize(0x00000001); // SS0 Raw Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_IM register.
//
//*****************************************************************************
pub const IM = struct {
    pub const DCONSS3 = usize(0x00080000); // Digital Comparator Interrupt on
    // SS3
    pub const DCONSS2 = usize(0x00040000); // Digital Comparator Interrupt on
    // SS2
    pub const DCONSS1 = usize(0x00020000); // Digital Comparator Interrupt on
    // SS1
    pub const DCONSS0 = usize(0x00010000); // Digital Comparator Interrupt on
    // SS0
    pub const DMAMASK3 = usize(0x00000800); // SS3 DMA Interrupt Mask
    pub const DMAMASK2 = usize(0x00000400); // SS2 DMA Interrupt Mask
    pub const DMAMASK1 = usize(0x00000200); // SS1 DMA Interrupt Mask
    pub const DMAMASK0 = usize(0x00000100); // SS0 DMA Interrupt Mask
    pub const MASK3 = usize(0x00000008); // SS3 Interrupt Mask
    pub const MASK2 = usize(0x00000004); // SS2 Interrupt Mask
    pub const MASK1 = usize(0x00000002); // SS1 Interrupt Mask
    pub const MASK0 = usize(0x00000001); // SS0 Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_ISC register.
//
//*****************************************************************************
pub const ISC = struct {
    pub const DCINSS3 = usize(0x00080000); // Digital Comparator Interrupt
    // Status on SS3
    pub const DCINSS2 = usize(0x00040000); // Digital Comparator Interrupt
    // Status on SS2
    pub const DCINSS1 = usize(0x00020000); // Digital Comparator Interrupt
    // Status on SS1
    pub const DCINSS0 = usize(0x00010000); // Digital Comparator Interrupt
    // Status on SS0
    pub const DMAIN3 = usize(0x00000800); // SS3 DMA Interrupt Status and
    // Clear
    pub const DMAIN2 = usize(0x00000400); // SS2 DMA Interrupt Status and
    // Clear
    pub const DMAIN1 = usize(0x00000200); // SS1 DMA Interrupt Status and
    // Clear
    pub const DMAIN0 = usize(0x00000100); // SS0 DMA Interrupt Status and
    // Clear
    pub const IN3 = usize(0x00000008); // SS3 Interrupt Status and Clear
    pub const IN2 = usize(0x00000004); // SS2 Interrupt Status and Clear
    pub const IN1 = usize(0x00000002); // SS1 Interrupt Status and Clear
    pub const IN0 = usize(0x00000001); // SS0 Interrupt Status and Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_OSTAT register.
//
//*****************************************************************************
pub const OSTAT = struct {
    pub const OV3 = usize(0x00000008); // SS3 FIFO Overflow
    pub const OV2 = usize(0x00000004); // SS2 FIFO Overflow
    pub const OV1 = usize(0x00000002); // SS1 FIFO Overflow
    pub const OV0 = usize(0x00000001); // SS0 FIFO Overflow
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_EMUX register.
//
//*****************************************************************************
pub const EMUX = struct {
    pub const EM3_M = usize(0x0000F000); // SS3 Trigger Select
    pub const EM3_PROCESSOR = usize(0x00000000); // Processor (default)
    pub const EM3_COMP0 = usize(0x00001000); // Analog Comparator 0
    pub const EM3_COMP1 = usize(0x00002000); // Analog Comparator 1
    pub const EM3_COMP2 = usize(0x00003000); // Analog Comparator 2
    pub const EM3_EXTERNAL = usize(0x00004000); // External (GPIO Pins)
    pub const EM3_TIMER = usize(0x00005000); // Timer
    pub const EM3_PWM0 = usize(0x00006000); // PWM generator 0
    pub const EM3_PWM1 = usize(0x00007000); // PWM generator 1
    pub const EM3_PWM2 = usize(0x00008000); // PWM generator 2
    pub const EM3_PWM3 = usize(0x00009000); // PWM generator 3
    pub const EM3_NEVER = usize(0x0000E000); // Never Trigger
    pub const EM3_ALWAYS = usize(0x0000F000); // Always (continuously sample)
    pub const EM2_M = usize(0x00000F00); // SS2 Trigger Select
    pub const EM2_PROCESSOR = usize(0x00000000); // Processor (default)
    pub const EM2_COMP0 = usize(0x00000100); // Analog Comparator 0
    pub const EM2_COMP1 = usize(0x00000200); // Analog Comparator 1
    pub const EM2_COMP2 = usize(0x00000300); // Analog Comparator 2
    pub const EM2_EXTERNAL = usize(0x00000400); // External (GPIO Pins)
    pub const EM2_TIMER = usize(0x00000500); // Timer
    pub const EM2_PWM0 = usize(0x00000600); // PWM generator 0
    pub const EM2_PWM1 = usize(0x00000700); // PWM generator 1
    pub const EM2_PWM2 = usize(0x00000800); // PWM generator 2
    pub const EM2_PWM3 = usize(0x00000900); // PWM generator 3
    pub const EM2_NEVER = usize(0x00000E00); // Never Trigger
    pub const EM2_ALWAYS = usize(0x00000F00); // Always (continuously sample)
    pub const EM1_M = usize(0x000000F0); // SS1 Trigger Select
    pub const EM1_PROCESSOR = usize(0x00000000); // Processor (default)
    pub const EM1_COMP0 = usize(0x00000010); // Analog Comparator 0
    pub const EM1_COMP1 = usize(0x00000020); // Analog Comparator 1
    pub const EM1_COMP2 = usize(0x00000030); // Analog Comparator 2
    pub const EM1_EXTERNAL = usize(0x00000040); // External (GPIO Pins)
    pub const EM1_TIMER = usize(0x00000050); // Timer
    pub const EM1_PWM0 = usize(0x00000060); // PWM generator 0
    pub const EM1_PWM1 = usize(0x00000070); // PWM generator 1
    pub const EM1_PWM2 = usize(0x00000080); // PWM generator 2
    pub const EM1_PWM3 = usize(0x00000090); // PWM generator 3
    pub const EM1_NEVER = usize(0x000000E0); // Never Trigger
    pub const EM1_ALWAYS = usize(0x000000F0); // Always (continuously sample)
    pub const EM0_M = usize(0x0000000F); // SS0 Trigger Select
    pub const EM0_PROCESSOR = usize(0x00000000); // Processor (default)
    pub const EM0_COMP0 = usize(0x00000001); // Analog Comparator 0
    pub const EM0_COMP1 = usize(0x00000002); // Analog Comparator 1
    pub const EM0_COMP2 = usize(0x00000003); // Analog Comparator 2
    pub const EM0_EXTERNAL = usize(0x00000004); // External (GPIO Pins)
    pub const EM0_TIMER = usize(0x00000005); // Timer
    pub const EM0_PWM0 = usize(0x00000006); // PWM generator 0
    pub const EM0_PWM1 = usize(0x00000007); // PWM generator 1
    pub const EM0_PWM2 = usize(0x00000008); // PWM generator 2
    pub const EM0_PWM3 = usize(0x00000009); // PWM generator 3
    pub const EM0_NEVER = usize(0x0000000E); // Never Trigger
    pub const EM0_ALWAYS = usize(0x0000000F); // Always (continuously sample)
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_USTAT register.
//
//*****************************************************************************
pub const USTAT = struct {
    pub const UV3 = usize(0x00000008); // SS3 FIFO Underflow
    pub const UV2 = usize(0x00000004); // SS2 FIFO Underflow
    pub const UV1 = usize(0x00000002); // SS1 FIFO Underflow
    pub const UV0 = usize(0x00000001); // SS0 FIFO Underflow
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_TSSEL register.
//
//*****************************************************************************
pub const TSSEL = struct {
    pub const PS3_M = usize(0x30000000); // Generator 3 PWM Module Trigger
    // Select
    pub const PS3_0 = usize(0x00000000); // Use Generator 3 (and its
    // trigger) in PWM module 0
    pub const PS3_1 = usize(0x10000000); // Use Generator 3 (and its
    // trigger) in PWM module 1
    pub const PS2_M = usize(0x00300000); // Generator 2 PWM Module Trigger
    // Select
    pub const PS2_0 = usize(0x00000000); // Use Generator 2 (and its
    // trigger) in PWM module 0
    pub const PS2_1 = usize(0x00100000); // Use Generator 2 (and its
    // trigger) in PWM module 1
    pub const PS1_M = usize(0x00003000); // Generator 1 PWM Module Trigger
    // Select
    pub const PS1_0 = usize(0x00000000); // Use Generator 1 (and its
    // trigger) in PWM module 0
    pub const PS1_1 = usize(0x00001000); // Use Generator 1 (and its
    // trigger) in PWM module 1
    pub const PS0_M = usize(0x00000030); // Generator 0 PWM Module Trigger
    // Select
    pub const PS0_0 = usize(0x00000000); // Use Generator 0 (and its
    // trigger) in PWM module 0
    pub const PS0_1 = usize(0x00000010); // Use Generator 0 (and its
    // trigger) in PWM module 1
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSPRI register.
//
//*****************************************************************************
pub const SSPRI = struct {
    pub const SS3_M = usize(0x00003000); // SS3 Priority
    pub const SS2_M = usize(0x00000300); // SS2 Priority
    pub const SS1_M = usize(0x00000030); // SS1 Priority
    pub const SS0_M = usize(0x00000003); // SS0 Priority
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SPC register.
//
//*****************************************************************************
pub const SPC = struct {
    pub const PHASE_M = usize(0x0000000F); // Phase Difference
    pub const PHASE_0 = usize(0x00000000); // ADC sample lags by 0.0
    pub const PHASE_22_5 = usize(0x00000001); // ADC sample lags by 22.5
    pub const PHASE_45 = usize(0x00000002); // ADC sample lags by 45.0
    pub const PHASE_67_5 = usize(0x00000003); // ADC sample lags by 67.5
    pub const PHASE_90 = usize(0x00000004); // ADC sample lags by 90.0
    pub const PHASE_112_5 = usize(0x00000005); // ADC sample lags by 112.5
    pub const PHASE_135 = usize(0x00000006); // ADC sample lags by 135.0
    pub const PHASE_157_5 = usize(0x00000007); // ADC sample lags by 157.5
    pub const PHASE_180 = usize(0x00000008); // ADC sample lags by 180.0
    pub const PHASE_202_5 = usize(0x00000009); // ADC sample lags by 202.5
    pub const PHASE_225 = usize(0x0000000A); // ADC sample lags by 225.0
    pub const PHASE_247_5 = usize(0x0000000B); // ADC sample lags by 247.5
    pub const PHASE_270 = usize(0x0000000C); // ADC sample lags by 270.0
    pub const PHASE_292_5 = usize(0x0000000D); // ADC sample lags by 292.5
    pub const PHASE_315 = usize(0x0000000E); // ADC sample lags by 315.0
    pub const PHASE_337_5 = usize(0x0000000F); // ADC sample lags by 337.5
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PSSI register.
//
//*****************************************************************************
pub const PSSI = struct {
    pub const GSYNC = usize(0x80000000); // Global Synchronize
    pub const SYNCWAIT = usize(0x08000000); // Synchronize Wait
    pub const SS3 = usize(0x00000008); // SS3 Initiate
    pub const SS2 = usize(0x00000004); // SS2 Initiate
    pub const SS1 = usize(0x00000002); // SS1 Initiate
    pub const SS0 = usize(0x00000001); // SS0 Initiate
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SAC register.
//
//*****************************************************************************
pub const SAC = struct {
    pub const AVG_M = usize(0x00000007); // Hardware Averaging Control
    pub const AVG_OFF = usize(0x00000000); // No hardware oversampling
    pub const AVG_2X = usize(0x00000001); // 2x hardware oversampling
    pub const AVG_4X = usize(0x00000002); // 4x hardware oversampling
    pub const AVG_8X = usize(0x00000003); // 8x hardware oversampling
    pub const AVG_16X = usize(0x00000004); // 16x hardware oversampling
    pub const AVG_32X = usize(0x00000005); // 32x hardware oversampling
    pub const AVG_64X = usize(0x00000006); // 64x hardware oversampling
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCISC register.
//
//*****************************************************************************
pub const DCISC = struct {
    pub const DCINT7 = usize(0x00000080); // Digital Comparator 7 Interrupt
    // Status and Clear
    pub const DCINT6 = usize(0x00000040); // Digital Comparator 6 Interrupt
    // Status and Clear
    pub const DCINT5 = usize(0x00000020); // Digital Comparator 5 Interrupt
    // Status and Clear
    pub const DCINT4 = usize(0x00000010); // Digital Comparator 4 Interrupt
    // Status and Clear
    pub const DCINT3 = usize(0x00000008); // Digital Comparator 3 Interrupt
    // Status and Clear
    pub const DCINT2 = usize(0x00000004); // Digital Comparator 2 Interrupt
    // Status and Clear
    pub const DCINT1 = usize(0x00000002); // Digital Comparator 1 Interrupt
    // Status and Clear
    pub const DCINT0 = usize(0x00000001); // Digital Comparator 0 Interrupt
    // Status and Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_CTL register.
//
//*****************************************************************************
pub const CTL = struct {
    pub const VREF_M = usize(0x00000003); // Voltage Reference Select
    pub const VREF_INTERNAL = usize(0x00000000); // VDDA and GNDA are the voltage
    // references
    pub const VREF_EXT_3V = usize(0x00000001); // The external VREFA+ and VREFA-
    // inputs are the voltage
    // references
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX0 register.
//
//*****************************************************************************
pub const SSMUX0 = struct {
    pub const MUX7_M = usize(0xF0000000); // 8th Sample Input Select
    pub const MUX6_M = usize(0x0F000000); // 7th Sample Input Select
    pub const MUX5_M = usize(0x00F00000); // 6th Sample Input Select
    pub const MUX4_M = usize(0x000F0000); // 5th Sample Input Select
    pub const MUX3_M = usize(0x0000F000); // 4th Sample Input Select
    pub const MUX2_M = usize(0x00000F00); // 3rd Sample Input Select
    pub const MUX1_M = usize(0x000000F0); // 2nd Sample Input Select
    pub const MUX0_M = usize(0x0000000F); // 1st Sample Input Select
    pub const MUX7_S = usize(28);
    pub const MUX6_S = usize(24);
    pub const MUX5_S = usize(20);
    pub const MUX4_S = usize(16);
    pub const MUX3_S = usize(12);
    pub const MUX2_S = usize(8);
    pub const MUX1_S = usize(4);
    pub const MUX0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL0 register.
//
//*****************************************************************************
pub const SSCTL0 = struct {
    pub const TS7 = usize(0x80000000); // 8th Sample Temp Sensor Select
    pub const IE7 = usize(0x40000000); // 8th Sample Interrupt Enable
    pub const END7 = usize(0x20000000); // 8th Sample is End of Sequence
    pub const D7 = usize(0x10000000); // 8th Sample Differential Input
    // Select
    pub const TS6 = usize(0x08000000); // 7th Sample Temp Sensor Select
    pub const IE6 = usize(0x04000000); // 7th Sample Interrupt Enable
    pub const END6 = usize(0x02000000); // 7th Sample is End of Sequence
    pub const D6 = usize(0x01000000); // 7th Sample Differential Input
    // Select
    pub const TS5 = usize(0x00800000); // 6th Sample Temp Sensor Select
    pub const IE5 = usize(0x00400000); // 6th Sample Interrupt Enable
    pub const END5 = usize(0x00200000); // 6th Sample is End of Sequence
    pub const D5 = usize(0x00100000); // 6th Sample Differential Input
    // Select
    pub const TS4 = usize(0x00080000); // 5th Sample Temp Sensor Select
    pub const IE4 = usize(0x00040000); // 5th Sample Interrupt Enable
    pub const END4 = usize(0x00020000); // 5th Sample is End of Sequence
    pub const D4 = usize(0x00010000); // 5th Sample Differential Input
    // Select
    pub const TS3 = usize(0x00008000); // 4th Sample Temp Sensor Select
    pub const IE3 = usize(0x00004000); // 4th Sample Interrupt Enable
    pub const END3 = usize(0x00002000); // 4th Sample is End of Sequence
    pub const D3 = usize(0x00001000); // 4th Sample Differential Input
    // Select
    pub const TS2 = usize(0x00000800); // 3rd Sample Temp Sensor Select
    pub const IE2 = usize(0x00000400); // 3rd Sample Interrupt Enable
    pub const END2 = usize(0x00000200); // 3rd Sample is End of Sequence
    pub const D2 = usize(0x00000100); // 3rd Sample Differential Input
    // Select
    pub const TS1 = usize(0x00000080); // 2nd Sample Temp Sensor Select
    pub const IE1 = usize(0x00000040); // 2nd Sample Interrupt Enable
    pub const END1 = usize(0x00000020); // 2nd Sample is End of Sequence
    pub const D1 = usize(0x00000010); // 2nd Sample Differential Input
    // Select
    pub const TS0 = usize(0x00000008); // 1st Sample Temp Sensor Select
    pub const IE0 = usize(0x00000004); // 1st Sample Interrupt Enable
    pub const END0 = usize(0x00000002); // 1st Sample is End of Sequence
    pub const D0 = usize(0x00000001); // 1st Sample Differential Input
    // Select
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO0 register.
//
//*****************************************************************************
pub const SSFIFO0 = struct {
    pub const DATA_M = usize(0x00000FFF); // Conversion Result Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT0 register.
//
//*****************************************************************************
pub const SSFSTAT0 = struct {
    pub const FULL = usize(0x00001000); // FIFO Full
    pub const EMPTY = usize(0x00000100); // FIFO Empty
    pub const HPTR_M = usize(0x000000F0); // FIFO Head Pointer
    pub const TPTR_M = usize(0x0000000F); // FIFO Tail Pointer
    pub const HPTR_S = usize(4);
    pub const TPTR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP0 register.
//
//*****************************************************************************
pub const SSOP0 = struct {
    pub const S7DCOP = usize(0x10000000); // Sample 7 Digital Comparator
    // Operation
    pub const S6DCOP = usize(0x01000000); // Sample 6 Digital Comparator
    // Operation
    pub const S5DCOP = usize(0x00100000); // Sample 5 Digital Comparator
    // Operation
    pub const S4DCOP = usize(0x00010000); // Sample 4 Digital Comparator
    // Operation
    pub const S3DCOP = usize(0x00001000); // Sample 3 Digital Comparator
    // Operation
    pub const S2DCOP = usize(0x00000100); // Sample 2 Digital Comparator
    // Operation
    pub const S1DCOP = usize(0x00000010); // Sample 1 Digital Comparator
    // Operation
    pub const S0DCOP = usize(0x00000001); // Sample 0 Digital Comparator
    // Operation
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC0 register.
//
//*****************************************************************************
pub const SSDC0 = struct {
    pub const S7DCSEL_M = usize(0xF0000000); // Sample 7 Digital Comparator
    // Select
    pub const S6DCSEL_M = usize(0x0F000000); // Sample 6 Digital Comparator
    // Select
    pub const S5DCSEL_M = usize(0x00F00000); // Sample 5 Digital Comparator
    // Select
    pub const S4DCSEL_M = usize(0x000F0000); // Sample 4 Digital Comparator
    // Select
    pub const S3DCSEL_M = usize(0x0000F000); // Sample 3 Digital Comparator
    // Select
    pub const S2DCSEL_M = usize(0x00000F00); // Sample 2 Digital Comparator
    // Select
    pub const S1DCSEL_M = usize(0x000000F0); // Sample 1 Digital Comparator
    // Select
    pub const S0DCSEL_M = usize(0x0000000F); // Sample 0 Digital Comparator
    // Select
    pub const S6DCSEL_S = usize(24);
    pub const S5DCSEL_S = usize(20);
    pub const S4DCSEL_S = usize(16);
    pub const S3DCSEL_S = usize(12);
    pub const S2DCSEL_S = usize(8);
    pub const S1DCSEL_S = usize(4);
    pub const S0DCSEL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX0 register.
//
//*****************************************************************************
pub const SSEMUX0 = struct {
    pub const EMUX7 = usize(0x10000000); // 8th Sample Input Select (Upper
    // Bit)
    pub const EMUX6 = usize(0x01000000); // 7th Sample Input Select (Upper
    // Bit)
    pub const EMUX5 = usize(0x00100000); // 6th Sample Input Select (Upper
    // Bit)
    pub const EMUX4 = usize(0x00010000); // 5th Sample Input Select (Upper
    // Bit)
    pub const EMUX3 = usize(0x00001000); // 4th Sample Input Select (Upper
    // Bit)
    pub const EMUX2 = usize(0x00000100); // 3rd Sample Input Select (Upper
    // Bit)
    pub const EMUX1 = usize(0x00000010); // 2th Sample Input Select (Upper
    // Bit)
    pub const EMUX0 = usize(0x00000001); // 1st Sample Input Select (Upper
    // Bit)
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH0 register.
//
//*****************************************************************************
pub const SSTSH0 = struct {
    pub const TSH7_M = usize(0xF0000000); // 8th Sample and Hold Period
    // Select
    pub const TSH6_M = usize(0x0F000000); // 7th Sample and Hold Period
    // Select
    pub const TSH5_M = usize(0x00F00000); // 6th Sample and Hold Period
    // Select
    pub const TSH4_M = usize(0x000F0000); // 5th Sample and Hold Period
    // Select
    pub const TSH3_M = usize(0x0000F000); // 4th Sample and Hold Period
    // Select
    pub const TSH2_M = usize(0x00000F00); // 3rd Sample and Hold Period
    // Select
    pub const TSH1_M = usize(0x000000F0); // 2nd Sample and Hold Period
    // Select
    pub const TSH0_M = usize(0x0000000F); // 1st Sample and Hold Period
    // Select
    pub const TSH7_S = usize(28);
    pub const TSH6_S = usize(24);
    pub const TSH5_S = usize(20);
    pub const TSH4_S = usize(16);
    pub const TSH3_S = usize(12);
    pub const TSH2_S = usize(8);
    pub const TSH1_S = usize(4);
    pub const TSH0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX1 register.
//
//*****************************************************************************
pub const SSMUX1 = struct {
    pub const MUX3_M = usize(0x0000F000); // 4th Sample Input Select
    pub const MUX2_M = usize(0x00000F00); // 3rd Sample Input Select
    pub const MUX1_M = usize(0x000000F0); // 2nd Sample Input Select
    pub const MUX0_M = usize(0x0000000F); // 1st Sample Input Select
    pub const MUX3_S = usize(12);
    pub const MUX2_S = usize(8);
    pub const MUX1_S = usize(4);
    pub const MUX0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL1 register.
//
//*****************************************************************************
pub const SSCTL1 = struct {
    pub const TS3 = usize(0x00008000); // 4th Sample Temp Sensor Select
    pub const IE3 = usize(0x00004000); // 4th Sample Interrupt Enable
    pub const END3 = usize(0x00002000); // 4th Sample is End of Sequence
    pub const D3 = usize(0x00001000); // 4th Sample Differential Input
    // Select
    pub const TS2 = usize(0x00000800); // 3rd Sample Temp Sensor Select
    pub const IE2 = usize(0x00000400); // 3rd Sample Interrupt Enable
    pub const END2 = usize(0x00000200); // 3rd Sample is End of Sequence
    pub const D2 = usize(0x00000100); // 3rd Sample Differential Input
    // Select
    pub const TS1 = usize(0x00000080); // 2nd Sample Temp Sensor Select
    pub const IE1 = usize(0x00000040); // 2nd Sample Interrupt Enable
    pub const END1 = usize(0x00000020); // 2nd Sample is End of Sequence
    pub const D1 = usize(0x00000010); // 2nd Sample Differential Input
    // Select
    pub const TS0 = usize(0x00000008); // 1st Sample Temp Sensor Select
    pub const IE0 = usize(0x00000004); // 1st Sample Interrupt Enable
    pub const END0 = usize(0x00000002); // 1st Sample is End of Sequence
    pub const D0 = usize(0x00000001); // 1st Sample Differential Input
    // Select
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO1 register.
//
//*****************************************************************************
pub const SSFIFO1 = struct {
    pub const DATA_M = usize(0x00000FFF); // Conversion Result Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT1 register.
//
//*****************************************************************************
pub const SSFSTAT1 = struct {
    pub const FULL = usize(0x00001000); // FIFO Full
    pub const EMPTY = usize(0x00000100); // FIFO Empty
    pub const HPTR_M = usize(0x000000F0); // FIFO Head Pointer
    pub const TPTR_M = usize(0x0000000F); // FIFO Tail Pointer
    pub const HPTR_S = usize(4);
    pub const TPTR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP1 register.
//
//*****************************************************************************
pub const SSOP1 = struct {
    pub const S3DCOP = usize(0x00001000); // Sample 3 Digital Comparator
    // Operation
    pub const S2DCOP = usize(0x00000100); // Sample 2 Digital Comparator
    // Operation
    pub const S1DCOP = usize(0x00000010); // Sample 1 Digital Comparator
    // Operation
    pub const S0DCOP = usize(0x00000001); // Sample 0 Digital Comparator
    // Operation
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC1 register.
//
//*****************************************************************************
pub const SSDC1 = struct {
    pub const S3DCSEL_M = usize(0x0000F000); // Sample 3 Digital Comparator
    // Select
    pub const S2DCSEL_M = usize(0x00000F00); // Sample 2 Digital Comparator
    // Select
    pub const S1DCSEL_M = usize(0x000000F0); // Sample 1 Digital Comparator
    // Select
    pub const S0DCSEL_M = usize(0x0000000F); // Sample 0 Digital Comparator
    // Select
    pub const S2DCSEL_S = usize(8);
    pub const S1DCSEL_S = usize(4);
    pub const S0DCSEL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX1 register.
//
//*****************************************************************************
pub const SSEMUX1 = struct {
    pub const EMUX3 = usize(0x00001000); // 4th Sample Input Select (Upper
    // Bit)
    pub const EMUX2 = usize(0x00000100); // 3rd Sample Input Select (Upper
    // Bit)
    pub const EMUX1 = usize(0x00000010); // 2th Sample Input Select (Upper
    // Bit)
    pub const EMUX0 = usize(0x00000001); // 1st Sample Input Select (Upper
    // Bit)
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH1 register.
//
//*****************************************************************************
pub const SSTSH1 = struct {
    pub const TSH3_M = usize(0x0000F000); // 4th Sample and Hold Period
    // Select
    pub const TSH2_M = usize(0x00000F00); // 3rd Sample and Hold Period
    // Select
    pub const TSH1_M = usize(0x000000F0); // 2nd Sample and Hold Period
    // Select
    pub const TSH0_M = usize(0x0000000F); // 1st Sample and Hold Period
    // Select
    pub const TSH3_S = usize(12);
    pub const TSH2_S = usize(8);
    pub const TSH1_S = usize(4);
    pub const TSH0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX2 register.
//
//*****************************************************************************
pub const SSMUX2 = struct {
    pub const MUX3_M = usize(0x0000F000); // 4th Sample Input Select
    pub const MUX2_M = usize(0x00000F00); // 3rd Sample Input Select
    pub const MUX1_M = usize(0x000000F0); // 2nd Sample Input Select
    pub const MUX0_M = usize(0x0000000F); // 1st Sample Input Select
    pub const MUX3_S = usize(12);
    pub const MUX2_S = usize(8);
    pub const MUX1_S = usize(4);
    pub const MUX0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL2 register.
//
//*****************************************************************************
pub const SSCTL2 = struct {
    pub const TS3 = usize(0x00008000); // 4th Sample Temp Sensor Select
    pub const IE3 = usize(0x00004000); // 4th Sample Interrupt Enable
    pub const END3 = usize(0x00002000); // 4th Sample is End of Sequence
    pub const D3 = usize(0x00001000); // 4th Sample Differential Input
    // Select
    pub const TS2 = usize(0x00000800); // 3rd Sample Temp Sensor Select
    pub const IE2 = usize(0x00000400); // 3rd Sample Interrupt Enable
    pub const END2 = usize(0x00000200); // 3rd Sample is End of Sequence
    pub const D2 = usize(0x00000100); // 3rd Sample Differential Input
    // Select
    pub const TS1 = usize(0x00000080); // 2nd Sample Temp Sensor Select
    pub const IE1 = usize(0x00000040); // 2nd Sample Interrupt Enable
    pub const END1 = usize(0x00000020); // 2nd Sample is End of Sequence
    pub const D1 = usize(0x00000010); // 2nd Sample Differential Input
    // Select
    pub const TS0 = usize(0x00000008); // 1st Sample Temp Sensor Select
    pub const IE0 = usize(0x00000004); // 1st Sample Interrupt Enable
    pub const END0 = usize(0x00000002); // 1st Sample is End of Sequence
    pub const D0 = usize(0x00000001); // 1st Sample Differential Input
    // Select
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO2 register.
//
//*****************************************************************************
pub const SSFIFO2 = struct {
    pub const DATA_M = usize(0x00000FFF); // Conversion Result Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT2 register.
//
//*****************************************************************************
pub const SSFSTAT2 = struct {
    pub const FULL = usize(0x00001000); // FIFO Full
    pub const EMPTY = usize(0x00000100); // FIFO Empty
    pub const HPTR_M = usize(0x000000F0); // FIFO Head Pointer
    pub const TPTR_M = usize(0x0000000F); // FIFO Tail Pointer
    pub const HPTR_S = usize(4);
    pub const TPTR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP2 register.
//
//*****************************************************************************
pub const SSOP2 = struct {
    pub const S3DCOP = usize(0x00001000); // Sample 3 Digital Comparator
    // Operation
    pub const S2DCOP = usize(0x00000100); // Sample 2 Digital Comparator
    // Operation
    pub const S1DCOP = usize(0x00000010); // Sample 1 Digital Comparator
    // Operation
    pub const S0DCOP = usize(0x00000001); // Sample 0 Digital Comparator
    // Operation
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC2 register.
//
//*****************************************************************************
pub const SSDC2 = struct {
    pub const S3DCSEL_M = usize(0x0000F000); // Sample 3 Digital Comparator
    // Select
    pub const S2DCSEL_M = usize(0x00000F00); // Sample 2 Digital Comparator
    // Select
    pub const S1DCSEL_M = usize(0x000000F0); // Sample 1 Digital Comparator
    // Select
    pub const S0DCSEL_M = usize(0x0000000F); // Sample 0 Digital Comparator
    // Select
    pub const S2DCSEL_S = usize(8);
    pub const S1DCSEL_S = usize(4);
    pub const S0DCSEL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX2 register.
//
//*****************************************************************************
pub const SSEMUX2 = struct {
    pub const EMUX3 = usize(0x00001000); // 4th Sample Input Select (Upper
    // Bit)
    pub const EMUX2 = usize(0x00000100); // 3rd Sample Input Select (Upper
    // Bit)
    pub const EMUX1 = usize(0x00000010); // 2th Sample Input Select (Upper
    // Bit)
    pub const EMUX0 = usize(0x00000001); // 1st Sample Input Select (Upper
    // Bit)
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH2 register.
//
//*****************************************************************************
pub const SSTSH2 = struct {
    pub const TSH3_M = usize(0x0000F000); // 4th Sample and Hold Period
    // Select
    pub const TSH2_M = usize(0x00000F00); // 3rd Sample and Hold Period
    // Select
    pub const TSH1_M = usize(0x000000F0); // 2nd Sample and Hold Period
    // Select
    pub const TSH0_M = usize(0x0000000F); // 1st Sample and Hold Period
    // Select
    pub const TSH3_S = usize(12);
    pub const TSH2_S = usize(8);
    pub const TSH1_S = usize(4);
    pub const TSH0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX3 register.
//
//*****************************************************************************
pub const SSMUX3 = struct {
    pub const MUX0_M = usize(0x0000000F); // 1st Sample Input Select
    pub const MUX0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL3 register.
//
//*****************************************************************************
pub const SSCTL3 = struct {
    pub const TS0 = usize(0x00000008); // 1st Sample Temp Sensor Select
    pub const IE0 = usize(0x00000004); // Sample Interrupt Enable
    pub const END0 = usize(0x00000002); // End of Sequence
    pub const D0 = usize(0x00000001); // Sample Differential Input Select
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO3 register.
//
//*****************************************************************************
pub const SSFIFO3 = struct {
    pub const DATA_M = usize(0x00000FFF); // Conversion Result Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT3 register.
//
//*****************************************************************************
pub const SSFSTAT3 = struct {
    pub const FULL = usize(0x00001000); // FIFO Full
    pub const EMPTY = usize(0x00000100); // FIFO Empty
    pub const HPTR_M = usize(0x000000F0); // FIFO Head Pointer
    pub const TPTR_M = usize(0x0000000F); // FIFO Tail Pointer
    pub const HPTR_S = usize(4);
    pub const TPTR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP3 register.
//
//*****************************************************************************
pub const SSOP3 = struct {
    pub const S0DCOP = usize(0x00000001); // Sample 0 Digital Comparator
    // Operation
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC3 register.
//
//*****************************************************************************
pub const SSDC3 = struct {
    pub const S0DCSEL_M = usize(0x0000000F); // Sample 0 Digital Comparator
    // Select
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX3 register.
//
//*****************************************************************************
pub const SSEMUX3 = struct {
    pub const EMUX0 = usize(0x00000001); // 1st Sample Input Select (Upper
    // Bit)
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH3 register.
//
//*****************************************************************************
pub const SSTSH3 = struct {
    pub const TSH0_M = usize(0x0000000F); // 1st Sample and Hold Period
    // Select
    pub const TSH0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCRIC register.
//
//*****************************************************************************
pub const DCRIC = struct {
    pub const DCTRIG7 = usize(0x00800000); // Digital Comparator Trigger 7
    pub const DCTRIG6 = usize(0x00400000); // Digital Comparator Trigger 6
    pub const DCTRIG5 = usize(0x00200000); // Digital Comparator Trigger 5
    pub const DCTRIG4 = usize(0x00100000); // Digital Comparator Trigger 4
    pub const DCTRIG3 = usize(0x00080000); // Digital Comparator Trigger 3
    pub const DCTRIG2 = usize(0x00040000); // Digital Comparator Trigger 2
    pub const DCTRIG1 = usize(0x00020000); // Digital Comparator Trigger 1
    pub const DCTRIG0 = usize(0x00010000); // Digital Comparator Trigger 0
    pub const DCINT7 = usize(0x00000080); // Digital Comparator Interrupt 7
    pub const DCINT6 = usize(0x00000040); // Digital Comparator Interrupt 6
    pub const DCINT5 = usize(0x00000020); // Digital Comparator Interrupt 5
    pub const DCINT4 = usize(0x00000010); // Digital Comparator Interrupt 4
    pub const DCINT3 = usize(0x00000008); // Digital Comparator Interrupt 3
    pub const DCINT2 = usize(0x00000004); // Digital Comparator Interrupt 2
    pub const DCINT1 = usize(0x00000002); // Digital Comparator Interrupt 1
    pub const DCINT0 = usize(0x00000001); // Digital Comparator Interrupt 0
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL0 register.
//
//*****************************************************************************
pub const DCCTL0 = struct {
    pub const CTE = usize(0x00001000); // Comparison Trigger Enable
    pub const CTC_M = usize(0x00000C00); // Comparison Trigger Condition
    pub const CTC_LOW = usize(0x00000000); // Low Band
    pub const CTC_MID = usize(0x00000400); // Mid Band
    pub const CTC_HIGH = usize(0x00000C00); // High Band
    pub const CTM_M = usize(0x00000300); // Comparison Trigger Mode
    pub const CTM_ALWAYS = usize(0x00000000); // Always
    pub const CTM_ONCE = usize(0x00000100); // Once
    pub const CTM_HALWAYS = usize(0x00000200); // Hysteresis Always
    pub const CTM_HONCE = usize(0x00000300); // Hysteresis Once
    pub const CIE = usize(0x00000010); // Comparison Interrupt Enable
    pub const CIC_M = usize(0x0000000C); // Comparison Interrupt Condition
    pub const CIC_LOW = usize(0x00000000); // Low Band
    pub const CIC_MID = usize(0x00000004); // Mid Band
    pub const CIC_HIGH = usize(0x0000000C); // High Band
    pub const CIM_M = usize(0x00000003); // Comparison Interrupt Mode
    pub const CIM_ALWAYS = usize(0x00000000); // Always
    pub const CIM_ONCE = usize(0x00000001); // Once
    pub const CIM_HALWAYS = usize(0x00000002); // Hysteresis Always
    pub const CIM_HONCE = usize(0x00000003); // Hysteresis Once
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL1 register.
//
//*****************************************************************************
pub const DCCTL1 = struct {
    pub const CTE = usize(0x00001000); // Comparison Trigger Enable
    pub const CTC_M = usize(0x00000C00); // Comparison Trigger Condition
    pub const CTC_LOW = usize(0x00000000); // Low Band
    pub const CTC_MID = usize(0x00000400); // Mid Band
    pub const CTC_HIGH = usize(0x00000C00); // High Band
    pub const CTM_M = usize(0x00000300); // Comparison Trigger Mode
    pub const CTM_ALWAYS = usize(0x00000000); // Always
    pub const CTM_ONCE = usize(0x00000100); // Once
    pub const CTM_HALWAYS = usize(0x00000200); // Hysteresis Always
    pub const CTM_HONCE = usize(0x00000300); // Hysteresis Once
    pub const CIE = usize(0x00000010); // Comparison Interrupt Enable
    pub const CIC_M = usize(0x0000000C); // Comparison Interrupt Condition
    pub const CIC_LOW = usize(0x00000000); // Low Band
    pub const CIC_MID = usize(0x00000004); // Mid Band
    pub const CIC_HIGH = usize(0x0000000C); // High Band
    pub const CIM_M = usize(0x00000003); // Comparison Interrupt Mode
    pub const CIM_ALWAYS = usize(0x00000000); // Always
    pub const CIM_ONCE = usize(0x00000001); // Once
    pub const CIM_HALWAYS = usize(0x00000002); // Hysteresis Always
    pub const CIM_HONCE = usize(0x00000003); // Hysteresis Once
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL2 register.
//
//*****************************************************************************
pub const DCCTL2 = struct {
    pub const CTE = usize(0x00001000); // Comparison Trigger Enable
    pub const CTC_M = usize(0x00000C00); // Comparison Trigger Condition
    pub const CTC_LOW = usize(0x00000000); // Low Band
    pub const CTC_MID = usize(0x00000400); // Mid Band
    pub const CTC_HIGH = usize(0x00000C00); // High Band
    pub const CTM_M = usize(0x00000300); // Comparison Trigger Mode
    pub const CTM_ALWAYS = usize(0x00000000); // Always
    pub const CTM_ONCE = usize(0x00000100); // Once
    pub const CTM_HALWAYS = usize(0x00000200); // Hysteresis Always
    pub const CTM_HONCE = usize(0x00000300); // Hysteresis Once
    pub const CIE = usize(0x00000010); // Comparison Interrupt Enable
    pub const CIC_M = usize(0x0000000C); // Comparison Interrupt Condition
    pub const CIC_LOW = usize(0x00000000); // Low Band
    pub const CIC_MID = usize(0x00000004); // Mid Band
    pub const CIC_HIGH = usize(0x0000000C); // High Band
    pub const CIM_M = usize(0x00000003); // Comparison Interrupt Mode
    pub const CIM_ALWAYS = usize(0x00000000); // Always
    pub const CIM_ONCE = usize(0x00000001); // Once
    pub const CIM_HALWAYS = usize(0x00000002); // Hysteresis Always
    pub const CIM_HONCE = usize(0x00000003); // Hysteresis Once
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL3 register.
//
//*****************************************************************************
pub const DCCTL3 = struct {
    pub const CTE = usize(0x00001000); // Comparison Trigger Enable
    pub const CTC_M = usize(0x00000C00); // Comparison Trigger Condition
    pub const CTC_LOW = usize(0x00000000); // Low Band
    pub const CTC_MID = usize(0x00000400); // Mid Band
    pub const CTC_HIGH = usize(0x00000C00); // High Band
    pub const CTM_M = usize(0x00000300); // Comparison Trigger Mode
    pub const CTM_ALWAYS = usize(0x00000000); // Always
    pub const CTM_ONCE = usize(0x00000100); // Once
    pub const CTM_HALWAYS = usize(0x00000200); // Hysteresis Always
    pub const CTM_HONCE = usize(0x00000300); // Hysteresis Once
    pub const CIE = usize(0x00000010); // Comparison Interrupt Enable
    pub const CIC_M = usize(0x0000000C); // Comparison Interrupt Condition
    pub const CIC_LOW = usize(0x00000000); // Low Band
    pub const CIC_MID = usize(0x00000004); // Mid Band
    pub const CIC_HIGH = usize(0x0000000C); // High Band
    pub const CIM_M = usize(0x00000003); // Comparison Interrupt Mode
    pub const CIM_ALWAYS = usize(0x00000000); // Always
    pub const CIM_ONCE = usize(0x00000001); // Once
    pub const CIM_HALWAYS = usize(0x00000002); // Hysteresis Always
    pub const CIM_HONCE = usize(0x00000003); // Hysteresis Once
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL4 register.
//
//*****************************************************************************
pub const DCCTL4 = struct {
    pub const CTE = usize(0x00001000); // Comparison Trigger Enable
    pub const CTC_M = usize(0x00000C00); // Comparison Trigger Condition
    pub const CTC_LOW = usize(0x00000000); // Low Band
    pub const CTC_MID = usize(0x00000400); // Mid Band
    pub const CTC_HIGH = usize(0x00000C00); // High Band
    pub const CTM_M = usize(0x00000300); // Comparison Trigger Mode
    pub const CTM_ALWAYS = usize(0x00000000); // Always
    pub const CTM_ONCE = usize(0x00000100); // Once
    pub const CTM_HALWAYS = usize(0x00000200); // Hysteresis Always
    pub const CTM_HONCE = usize(0x00000300); // Hysteresis Once
    pub const CIE = usize(0x00000010); // Comparison Interrupt Enable
    pub const CIC_M = usize(0x0000000C); // Comparison Interrupt Condition
    pub const CIC_LOW = usize(0x00000000); // Low Band
    pub const CIC_MID = usize(0x00000004); // Mid Band
    pub const CIC_HIGH = usize(0x0000000C); // High Band
    pub const CIM_M = usize(0x00000003); // Comparison Interrupt Mode
    pub const CIM_ALWAYS = usize(0x00000000); // Always
    pub const CIM_ONCE = usize(0x00000001); // Once
    pub const CIM_HALWAYS = usize(0x00000002); // Hysteresis Always
    pub const CIM_HONCE = usize(0x00000003); // Hysteresis Once
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL5 register.
//
//*****************************************************************************
pub const DCCTL5 = struct {
    pub const CTE = usize(0x00001000); // Comparison Trigger Enable
    pub const CTC_M = usize(0x00000C00); // Comparison Trigger Condition
    pub const CTC_LOW = usize(0x00000000); // Low Band
    pub const CTC_MID = usize(0x00000400); // Mid Band
    pub const CTC_HIGH = usize(0x00000C00); // High Band
    pub const CTM_M = usize(0x00000300); // Comparison Trigger Mode
    pub const CTM_ALWAYS = usize(0x00000000); // Always
    pub const CTM_ONCE = usize(0x00000100); // Once
    pub const CTM_HALWAYS = usize(0x00000200); // Hysteresis Always
    pub const CTM_HONCE = usize(0x00000300); // Hysteresis Once
    pub const CIE = usize(0x00000010); // Comparison Interrupt Enable
    pub const CIC_M = usize(0x0000000C); // Comparison Interrupt Condition
    pub const CIC_LOW = usize(0x00000000); // Low Band
    pub const CIC_MID = usize(0x00000004); // Mid Band
    pub const CIC_HIGH = usize(0x0000000C); // High Band
    pub const CIM_M = usize(0x00000003); // Comparison Interrupt Mode
    pub const CIM_ALWAYS = usize(0x00000000); // Always
    pub const CIM_ONCE = usize(0x00000001); // Once
    pub const CIM_HALWAYS = usize(0x00000002); // Hysteresis Always
    pub const CIM_HONCE = usize(0x00000003); // Hysteresis Once
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL6 register.
//
//*****************************************************************************
pub const DCCTL6 = struct {
    pub const CTE = usize(0x00001000); // Comparison Trigger Enable
    pub const CTC_M = usize(0x00000C00); // Comparison Trigger Condition
    pub const CTC_LOW = usize(0x00000000); // Low Band
    pub const CTC_MID = usize(0x00000400); // Mid Band
    pub const CTC_HIGH = usize(0x00000C00); // High Band
    pub const CTM_M = usize(0x00000300); // Comparison Trigger Mode
    pub const CTM_ALWAYS = usize(0x00000000); // Always
    pub const CTM_ONCE = usize(0x00000100); // Once
    pub const CTM_HALWAYS = usize(0x00000200); // Hysteresis Always
    pub const CTM_HONCE = usize(0x00000300); // Hysteresis Once
    pub const CIE = usize(0x00000010); // Comparison Interrupt Enable
    pub const CIC_M = usize(0x0000000C); // Comparison Interrupt Condition
    pub const CIC_LOW = usize(0x00000000); // Low Band
    pub const CIC_MID = usize(0x00000004); // Mid Band
    pub const CIC_HIGH = usize(0x0000000C); // High Band
    pub const CIM_M = usize(0x00000003); // Comparison Interrupt Mode
    pub const CIM_ALWAYS = usize(0x00000000); // Always
    pub const CIM_ONCE = usize(0x00000001); // Once
    pub const CIM_HALWAYS = usize(0x00000002); // Hysteresis Always
    pub const CIM_HONCE = usize(0x00000003); // Hysteresis Once
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL7 register.
//
//*****************************************************************************
pub const DCCTL7 = struct {
    pub const CTE = usize(0x00001000); // Comparison Trigger Enable
    pub const CTC_M = usize(0x00000C00); // Comparison Trigger Condition
    pub const CTC_LOW = usize(0x00000000); // Low Band
    pub const CTC_MID = usize(0x00000400); // Mid Band
    pub const CTC_HIGH = usize(0x00000C00); // High Band
    pub const CTM_M = usize(0x00000300); // Comparison Trigger Mode
    pub const CTM_ALWAYS = usize(0x00000000); // Always
    pub const CTM_ONCE = usize(0x00000100); // Once
    pub const CTM_HALWAYS = usize(0x00000200); // Hysteresis Always
    pub const CTM_HONCE = usize(0x00000300); // Hysteresis Once
    pub const CIE = usize(0x00000010); // Comparison Interrupt Enable
    pub const CIC_M = usize(0x0000000C); // Comparison Interrupt Condition
    pub const CIC_LOW = usize(0x00000000); // Low Band
    pub const CIC_MID = usize(0x00000004); // Mid Band
    pub const CIC_HIGH = usize(0x0000000C); // High Band
    pub const CIM_M = usize(0x00000003); // Comparison Interrupt Mode
    pub const CIM_ALWAYS = usize(0x00000000); // Always
    pub const CIM_ONCE = usize(0x00000001); // Once
    pub const CIM_HALWAYS = usize(0x00000002); // Hysteresis Always
    pub const CIM_HONCE = usize(0x00000003); // Hysteresis Once
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP0 register.
//
//*****************************************************************************
pub const DCCMP0 = struct {
    pub const COMP1_M = usize(0x0FFF0000); // Compare 1
    pub const COMP0_M = usize(0x00000FFF); // Compare 0
    pub const COMP1_S = usize(16);
    pub const COMP0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP1 register.
//
//*****************************************************************************
pub const DCCMP1 = struct {
    pub const COMP1_M = usize(0x0FFF0000); // Compare 1
    pub const COMP0_M = usize(0x00000FFF); // Compare 0
    pub const COMP1_S = usize(16);
    pub const COMP0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP2 register.
//
//*****************************************************************************
pub const DCCMP2 = struct {
    pub const COMP1_M = usize(0x0FFF0000); // Compare 1
    pub const COMP0_M = usize(0x00000FFF); // Compare 0
    pub const COMP1_S = usize(16);
    pub const COMP0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP3 register.
//
//*****************************************************************************
pub const DCCMP3 = struct {
    pub const COMP1_M = usize(0x0FFF0000); // Compare 1
    pub const COMP0_M = usize(0x00000FFF); // Compare 0
    pub const COMP1_S = usize(16);
    pub const COMP0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP4 register.
//
//*****************************************************************************
pub const DCCMP4 = struct {
    pub const COMP1_M = usize(0x0FFF0000); // Compare 1
    pub const COMP0_M = usize(0x00000FFF); // Compare 0
    pub const COMP1_S = usize(16);
    pub const COMP0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP5 register.
//
//*****************************************************************************
pub const DCCMP5 = struct {
    pub const COMP1_M = usize(0x0FFF0000); // Compare 1
    pub const COMP0_M = usize(0x00000FFF); // Compare 0
    pub const COMP1_S = usize(16);
    pub const COMP0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP6 register.
//
//*****************************************************************************
pub const DCCMP6 = struct {
    pub const COMP1_M = usize(0x0FFF0000); // Compare 1
    pub const COMP0_M = usize(0x00000FFF); // Compare 0
    pub const COMP1_S = usize(16);
    pub const COMP0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP7 register.
//
//*****************************************************************************
pub const DCCMP7 = struct {
    pub const COMP1_M = usize(0x0FFF0000); // Compare 1
    pub const COMP0_M = usize(0x00000FFF); // Compare 0
    pub const COMP1_S = usize(16);
    pub const COMP0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PP register.
//
//*****************************************************************************
pub const PP = struct {
    pub const APSHT = usize(0x01000000); // Application-Programmable
    // Sample-and-Hold Time
    pub const TS = usize(0x00800000); // Temperature Sensor
    pub const RSL_M = usize(0x007C0000); // Resolution
    pub const TYPE_M = usize(0x00030000); // ADC Architecture
    pub const TYPE_SAR = usize(0x00000000); // SAR
    pub const DC_M = usize(0x0000FC00); // Digital Comparator Count
    pub const CH_M = usize(0x000003F0); // ADC Channel Count
    pub const MCR_M = usize(0x0000000F); // Maximum Conversion Rate
    pub const MCR_FULL = usize(0x00000007); // Full conversion rate (FCONV) as
    // defined by TADC and NSH
    pub const MSR_M = usize(0x0000000F); // Maximum ADC Sample Rate
    pub const MSR_125K = usize(0x00000001); // 125 ksps
    pub const MSR_250K = usize(0x00000003); // 250 ksps
    pub const MSR_500K = usize(0x00000005); // 500 ksps
    pub const MSR_1M = usize(0x00000007); // 1 Msps
    pub const RSL_S = usize(18);
    pub const DC_S = usize(10);
    pub const CH_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PC register.
//
//*****************************************************************************
pub const PC = struct {
    pub const SR_M = usize(0x0000000F); // ADC Sample Rate
    pub const SR_125K = usize(0x00000001); // 125 ksps
    pub const SR_250K = usize(0x00000003); // 250 ksps
    pub const SR_500K = usize(0x00000005); // 500 ksps
    pub const SR_1M = usize(0x00000007); // 1 Msps
    pub const MCR_M = usize(0x0000000F); // Conversion Rate
    pub const MCR_1_8 = usize(0x00000001); // Eighth conversion rate. After a
    // conversion completes, the logic
    // pauses for 112 TADC periods
    // before starting the next
    // conversion
    pub const MCR_1_4 = usize(0x00000003); // Quarter conversion rate. After a
    // conversion completes, the logic
    // pauses for 48 TADC periods
    // before starting the next
    // conversion
    pub const MCR_1_2 = usize(0x00000005); // Half conversion rate. After a
    // conversion completes, the logic
    // pauses for 16 TADC periods
    // before starting the next
    // conversion
    pub const MCR_FULL = usize(0x00000007); // Full conversion rate (FCONV) as
    // defined by TADC and NSH
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_CC register.
//
//*****************************************************************************
pub const CC = struct {
    pub const CLKDIV_M = usize(0x000003F0); // PLL VCO Clock Divisor
    pub const CS_M = usize(0x0000000F); // ADC Clock Source
    pub const CS_SYSPLL = usize(0x00000000); // PLL VCO divided by CLKDIV
    pub const CS_PIOSC = usize(0x00000001); // PIOSC
    pub const CS_MOSC = usize(0x00000002); // MOSC
    pub const CLKDIV_S = usize(4);
};
