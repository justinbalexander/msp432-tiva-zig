//*****************************************************************************
//
// hw_comp.h - Macros used when accessing the comparator hardware.
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
// The following are defines for the Comparator register offsets.
//
//*****************************************************************************
pub const COMP_O_ACMIS = usize(0x00000000);  // Analog Comparator Masked
                                            // Interrupt Status
pub const COMP_O_ACRIS = usize(0x00000004);  // Analog Comparator Raw Interrupt
                                            // Status
pub const COMP_O_ACINTEN = usize(0x00000008);  // Analog Comparator Interrupt
                                            // Enable
pub const COMP_O_ACREFCTL = usize(0x00000010);  // Analog Comparator Reference
                                            // Voltage Control
pub const COMP_O_ACSTAT0 = usize(0x00000020);  // Analog Comparator Status 0
pub const COMP_O_ACCTL0 = usize(0x00000024);  // Analog Comparator Control 0
pub const COMP_O_ACSTAT1 = usize(0x00000040);  // Analog Comparator Status 1
pub const COMP_O_ACCTL1 = usize(0x00000044);  // Analog Comparator Control 1
pub const COMP_O_ACSTAT2 = usize(0x00000060);  // Analog Comparator Status 2
pub const COMP_O_ACCTL2 = usize(0x00000064);  // Analog Comparator Control 2
pub const COMP_O_PP = usize(0x00000FC0);  // Analog Comparator Peripheral
                                            // Properties

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACMIS register.
//
//*****************************************************************************
pub const COMP_ACMIS_IN2 = usize(0x00000004);  // Comparator 2 Masked Interrupt
                                            // Status
pub const COMP_ACMIS_IN1 = usize(0x00000002);  // Comparator 1 Masked Interrupt
                                            // Status
pub const COMP_ACMIS_IN0 = usize(0x00000001);  // Comparator 0 Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACRIS register.
//
//*****************************************************************************
pub const COMP_ACRIS_IN2 = usize(0x00000004);  // Comparator 2 Interrupt Status
pub const COMP_ACRIS_IN1 = usize(0x00000002);  // Comparator 1 Interrupt Status
pub const COMP_ACRIS_IN0 = usize(0x00000001);  // Comparator 0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACINTEN register.
//
//*****************************************************************************
pub const COMP_ACINTEN_IN2 = usize(0x00000004);  // Comparator 2 Interrupt Enable
pub const COMP_ACINTEN_IN1 = usize(0x00000002);  // Comparator 1 Interrupt Enable
pub const COMP_ACINTEN_IN0 = usize(0x00000001);  // Comparator 0 Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACREFCTL
// register.
//
//*****************************************************************************
pub const COMP_ACREFCTL_EN = usize(0x00000200);  // Resistor Ladder Enable
pub const COMP_ACREFCTL_RNG = usize(0x00000100);  // Resistor Ladder Range
pub const COMP_ACREFCTL_VREF_M = usize(0x0000000F);  // Resistor Ladder Voltage Ref
pub const COMP_ACREFCTL_VREF_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT0 register.
//
//*****************************************************************************
pub const COMP_ACSTAT0_OVAL = usize(0x00000002);  // Comparator Output Value

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL0 register.
//
//*****************************************************************************
pub const COMP_ACCTL0_TOEN = usize(0x00000800);  // Trigger Output Enable
pub const COMP_ACCTL0_ASRCP_M = usize(0x00000600);  // Analog Source Positive
pub const COMP_ACCTL0_ASRCP_PIN = usize(0x00000000);  // Pin value of Cn+
pub const COMP_ACCTL0_ASRCP_PIN0 = usize(0x00000200);  // Pin value of C0+
pub const COMP_ACCTL0_ASRCP_REF = usize(0x00000400);  // Internal voltage reference
pub const COMP_ACCTL0_TSLVAL = usize(0x00000080);  // Trigger Sense Level Value
pub const COMP_ACCTL0_TSEN_M = usize(0x00000060);  // Trigger Sense
pub const COMP_ACCTL0_TSEN_LEVEL = usize(0x00000000);  // Level sense, see TSLVAL
pub const COMP_ACCTL0_TSEN_FALL = usize(0x00000020);  // Falling edge
pub const COMP_ACCTL0_TSEN_RISE = usize(0x00000040);  // Rising edge
pub const COMP_ACCTL0_TSEN_BOTH = usize(0x00000060);  // Either edge
pub const COMP_ACCTL0_ISLVAL = usize(0x00000010);  // Interrupt Sense Level Value
pub const COMP_ACCTL0_ISEN_M = usize(0x0000000C);  // Interrupt Sense
pub const COMP_ACCTL0_ISEN_LEVEL = usize(0x00000000);  // Level sense, see ISLVAL
pub const COMP_ACCTL0_ISEN_FALL = usize(0x00000004);  // Falling edge
pub const COMP_ACCTL0_ISEN_RISE = usize(0x00000008);  // Rising edge
pub const COMP_ACCTL0_ISEN_BOTH = usize(0x0000000C);  // Either edge
pub const COMP_ACCTL0_CINV = usize(0x00000002);  // Comparator Output Invert

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT1 register.
//
//*****************************************************************************
pub const COMP_ACSTAT1_OVAL = usize(0x00000002);  // Comparator Output Value

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL1 register.
//
//*****************************************************************************
pub const COMP_ACCTL1_TOEN = usize(0x00000800);  // Trigger Output Enable
pub const COMP_ACCTL1_ASRCP_M = usize(0x00000600);  // Analog Source Positive
pub const COMP_ACCTL1_ASRCP_PIN = usize(0x00000000);  // Pin value of Cn+
pub const COMP_ACCTL1_ASRCP_PIN0 = usize(0x00000200);  // Pin value of C0+
pub const COMP_ACCTL1_ASRCP_REF = usize(0x00000400);  // Internal voltage reference
pub const COMP_ACCTL1_TSLVAL = usize(0x00000080);  // Trigger Sense Level Value
pub const COMP_ACCTL1_TSEN_M = usize(0x00000060);  // Trigger Sense
pub const COMP_ACCTL1_TSEN_LEVEL = usize(0x00000000);  // Level sense, see TSLVAL
pub const COMP_ACCTL1_TSEN_FALL = usize(0x00000020);  // Falling edge
pub const COMP_ACCTL1_TSEN_RISE = usize(0x00000040);  // Rising edge
pub const COMP_ACCTL1_TSEN_BOTH = usize(0x00000060);  // Either edge
pub const COMP_ACCTL1_ISLVAL = usize(0x00000010);  // Interrupt Sense Level Value
pub const COMP_ACCTL1_ISEN_M = usize(0x0000000C);  // Interrupt Sense
pub const COMP_ACCTL1_ISEN_LEVEL = usize(0x00000000);  // Level sense, see ISLVAL
pub const COMP_ACCTL1_ISEN_FALL = usize(0x00000004);  // Falling edge
pub const COMP_ACCTL1_ISEN_RISE = usize(0x00000008);  // Rising edge
pub const COMP_ACCTL1_ISEN_BOTH = usize(0x0000000C);  // Either edge
pub const COMP_ACCTL1_CINV = usize(0x00000002);  // Comparator Output Invert

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT2 register.
//
//*****************************************************************************
pub const COMP_ACSTAT2_OVAL = usize(0x00000002);  // Comparator Output Value

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL2 register.
//
//*****************************************************************************
pub const COMP_ACCTL2_TOEN = usize(0x00000800);  // Trigger Output Enable
pub const COMP_ACCTL2_ASRCP_M = usize(0x00000600);  // Analog Source Positive
pub const COMP_ACCTL2_ASRCP_PIN = usize(0x00000000);  // Pin value of Cn+
pub const COMP_ACCTL2_ASRCP_PIN0 = usize(0x00000200);  // Pin value of C0+
pub const COMP_ACCTL2_ASRCP_REF = usize(0x00000400);  // Internal voltage reference
pub const COMP_ACCTL2_TSLVAL = usize(0x00000080);  // Trigger Sense Level Value
pub const COMP_ACCTL2_TSEN_M = usize(0x00000060);  // Trigger Sense
pub const COMP_ACCTL2_TSEN_LEVEL = usize(0x00000000);  // Level sense, see TSLVAL
pub const COMP_ACCTL2_TSEN_FALL = usize(0x00000020);  // Falling edge
pub const COMP_ACCTL2_TSEN_RISE = usize(0x00000040);  // Rising edge
pub const COMP_ACCTL2_TSEN_BOTH = usize(0x00000060);  // Either edge
pub const COMP_ACCTL2_ISLVAL = usize(0x00000010);  // Interrupt Sense Level Value
pub const COMP_ACCTL2_ISEN_M = usize(0x0000000C);  // Interrupt Sense
pub const COMP_ACCTL2_ISEN_LEVEL = usize(0x00000000);  // Level sense, see ISLVAL
pub const COMP_ACCTL2_ISEN_FALL = usize(0x00000004);  // Falling edge
pub const COMP_ACCTL2_ISEN_RISE = usize(0x00000008);  // Rising edge
pub const COMP_ACCTL2_ISEN_BOTH = usize(0x0000000C);  // Either edge
pub const COMP_ACCTL2_CINV = usize(0x00000002);  // Comparator Output Invert

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_PP register.
//
//*****************************************************************************
pub const COMP_PP_C2O = usize(0x00040000);  // Comparator Output 2 Present
pub const COMP_PP_C1O = usize(0x00020000);  // Comparator Output 1 Present
pub const COMP_PP_C0O = usize(0x00010000);  // Comparator Output 0 Present
pub const COMP_PP_CMP2 = usize(0x00000004);  // Comparator 2 Present
pub const COMP_PP_CMP1 = usize(0x00000002);  // Comparator 1 Present
pub const COMP_PP_CMP0 = usize(0x00000001);  // Comparator 0 Present

