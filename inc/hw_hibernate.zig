//*****************************************************************************
//
// hw_hibernate.h - Defines and Macros for the Hibernation module.
//
// Copyright (c) 2007-2017 Texas Instruments Incorporated.  All rights reserved.
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
// The following are defines for the Hibernation module register addresses.
//
//*****************************************************************************
pub const HIB_RTCC = usize(0x400FC000);  // Hibernation RTC Counter
pub const HIB_RTCM0 = usize(0x400FC004);  // Hibernation RTC Match 0
pub const HIB_RTCLD = usize(0x400FC00C);  // Hibernation RTC Load
pub const HIB_CTL = usize(0x400FC010);  // Hibernation Control
pub const HIB_IM = usize(0x400FC014);  // Hibernation Interrupt Mask
pub const HIB_RIS = usize(0x400FC018);  // Hibernation Raw Interrupt Status
pub const HIB_MIS = usize(0x400FC01C);  // Hibernation Masked Interrupt
                                            // Status
pub const HIB_IC = usize(0x400FC020);  // Hibernation Interrupt Clear
pub const HIB_RTCT = usize(0x400FC024);  // Hibernation RTC Trim
pub const HIB_RTCSS = usize(0x400FC028);  // Hibernation RTC Sub Seconds
pub const HIB_IO = usize(0x400FC02C);  // Hibernation IO Configuration
pub const HIB_DATA = usize(0x400FC030);  // Hibernation Data
pub const HIB_CALCTL = usize(0x400FC300);  // Hibernation Calendar Control
pub const HIB_CAL0 = usize(0x400FC310);  // Hibernation Calendar 0
pub const HIB_CAL1 = usize(0x400FC314);  // Hibernation Calendar 1
pub const HIB_CALLD0 = usize(0x400FC320);  // Hibernation Calendar Load 0
pub const HIB_CALLD1 = usize(0x400FC324);  // Hibernation Calendar Load
pub const HIB_CALM0 = usize(0x400FC330);  // Hibernation Calendar Match 0
pub const HIB_CALM1 = usize(0x400FC334);  // Hibernation Calendar Match 1
pub const HIB_LOCK = usize(0x400FC360);  // Hibernation Lock
pub const HIB_TPCTL = usize(0x400FC400);  // HIB Tamper Control
pub const HIB_TPSTAT = usize(0x400FC404);  // HIB Tamper Status
pub const HIB_TPIO = usize(0x400FC410);  // HIB Tamper I/O Control
pub const HIB_TPLOG0 = usize(0x400FC4E0);  // HIB Tamper Log 0
pub const HIB_TPLOG1 = usize(0x400FC4E4);  // HIB Tamper Log 1
pub const HIB_TPLOG2 = usize(0x400FC4E8);  // HIB Tamper Log 2
pub const HIB_TPLOG3 = usize(0x400FC4EC);  // HIB Tamper Log 3
pub const HIB_TPLOG4 = usize(0x400FC4F0);  // HIB Tamper Log 4
pub const HIB_TPLOG5 = usize(0x400FC4F4);  // HIB Tamper Log 5
pub const HIB_TPLOG6 = usize(0x400FC4F8);  // HIB Tamper Log 6
pub const HIB_TPLOG7 = usize(0x400FC4FC);  // HIB Tamper Log 7
pub const HIB_PP = usize(0x400FCFC0);  // Hibernation Peripheral
                                            // Properties
pub const HIB_CC = usize(0x400FCFC8);  // Hibernation Clock Control

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCC register.
//
//*****************************************************************************
pub const HIB_RTCC_M = usize(0xFFFFFFFF);  // RTC Counter
pub const HIB_RTCC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCM0 register.
//
//*****************************************************************************
pub const HIB_RTCM0_M = usize(0xFFFFFFFF);  // RTC Match 0
pub const HIB_RTCM0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCLD register.
//
//*****************************************************************************
pub const HIB_RTCLD_M = usize(0xFFFFFFFF);  // RTC Load
pub const HIB_RTCLD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CTL register.
//
//*****************************************************************************
pub const HIB_CTL_WRC = usize(0x80000000);  // Write Complete/Capable
pub const HIB_CTL_RETCLR = usize(0x40000000);  // GPIO Retention/Clear
pub const HIB_CTL_OSCSEL = usize(0x00080000);  // Oscillator Select
pub const HIB_CTL_OSCDRV = usize(0x00020000);  // Oscillator Drive Capability
pub const HIB_CTL_OSCBYP = usize(0x00010000);  // Oscillator Bypass
pub const HIB_CTL_VBATSEL_M = usize(0x00006000);  // Select for Low-Battery
                                            // Comparator
pub const HIB_CTL_VBATSEL_1_9V = usize(0x00000000);  // 1.9 Volts
pub const HIB_CTL_VBATSEL_2_1V = usize(0x00002000);  // 2.1 Volts (default)
pub const HIB_CTL_VBATSEL_2_3V = usize(0x00004000);  // 2.3 Volts
pub const HIB_CTL_VBATSEL_2_5V = usize(0x00006000);  // 2.5 Volts
pub const HIB_CTL_BATCHK = usize(0x00000400);  // Check Battery Status
pub const HIB_CTL_BATWKEN = usize(0x00000200);  // Wake on Low Battery
pub const HIB_CTL_VDD3ON = usize(0x00000100);  // VDD Powered
pub const HIB_CTL_VABORT = usize(0x00000080);  // Power Cut Abort Enable
pub const HIB_CTL_CLK32EN = usize(0x00000040);  // Clocking Enable
pub const HIB_CTL_PINWEN = usize(0x00000010);  // External Wake Pin Enable
pub const HIB_CTL_RTCWEN = usize(0x00000008);  // RTC Wake-up Enable
pub const HIB_CTL_HIBREQ = usize(0x00000002);  // Hibernation Request
pub const HIB_CTL_RTCEN = usize(0x00000001);  // RTC Timer Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_IM register.
//
//*****************************************************************************
pub const HIB_IM_VDDFAIL = usize(0x00000080);  // VDD Fail Interrupt Mask
pub const HIB_IM_RSTWK = usize(0x00000040);  // Reset Pad I/O Wake-Up Interrupt
                                            // Mask
pub const HIB_IM_PADIOWK = usize(0x00000020);  // Pad I/O Wake-Up Interrupt Mask
pub const HIB_IM_WC = usize(0x00000010);  // External Write Complete/Capable
                                            // Interrupt Mask
pub const HIB_IM_EXTW = usize(0x00000008);  // External Wake-Up Interrupt Mask
pub const HIB_IM_LOWBAT = usize(0x00000004);  // Low Battery Voltage Interrupt
                                            // Mask
pub const HIB_IM_RTCALT0 = usize(0x00000001);  // RTC Alert 0 Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RIS register.
//
//*****************************************************************************
pub const HIB_RIS_VDDFAIL = usize(0x00000080);  // VDD Fail Raw Interrupt Status
pub const HIB_RIS_RSTWK = usize(0x00000040);  // Reset Pad I/O Wake-Up Raw
                                            // Interrupt Status
pub const HIB_RIS_PADIOWK = usize(0x00000020);  // Pad I/O Wake-Up Raw Interrupt
                                            // Status
pub const HIB_RIS_WC = usize(0x00000010);  // Write Complete/Capable Raw
                                            // Interrupt Status
pub const HIB_RIS_EXTW = usize(0x00000008);  // External Wake-Up Raw Interrupt
                                            // Status
pub const HIB_RIS_LOWBAT = usize(0x00000004);  // Low Battery Voltage Raw
                                            // Interrupt Status
pub const HIB_RIS_RTCALT0 = usize(0x00000001);  // RTC Alert 0 Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_MIS register.
//
//*****************************************************************************
pub const HIB_MIS_VDDFAIL = usize(0x00000080);  // VDD Fail Interrupt Mask
pub const HIB_MIS_RSTWK = usize(0x00000040);  // Reset Pad I/O Wake-Up Interrupt
                                            // Mask
pub const HIB_MIS_PADIOWK = usize(0x00000020);  // Pad I/O Wake-Up Interrupt Mask
pub const HIB_MIS_WC = usize(0x00000010);  // Write Complete/Capable Masked
                                            // Interrupt Status
pub const HIB_MIS_EXTW = usize(0x00000008);  // External Wake-Up Masked
                                            // Interrupt Status
pub const HIB_MIS_LOWBAT = usize(0x00000004);  // Low Battery Voltage Masked
                                            // Interrupt Status
pub const HIB_MIS_RTCALT0 = usize(0x00000001);  // RTC Alert 0 Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_IC register.
//
//*****************************************************************************
pub const HIB_IC_VDDFAIL = usize(0x00000080);  // VDD Fail Interrupt Clear
pub const HIB_IC_RSTWK = usize(0x00000040);  // Reset Pad I/O Wake-Up Interrupt
                                            // Clear
pub const HIB_IC_PADIOWK = usize(0x00000020);  // Pad I/O Wake-Up Interrupt Clear
pub const HIB_IC_WC = usize(0x00000010);  // Write Complete/Capable Interrupt
                                            // Clear
pub const HIB_IC_EXTW = usize(0x00000008);  // External Wake-Up Interrupt Clear
pub const HIB_IC_LOWBAT = usize(0x00000004);  // Low Battery Voltage Interrupt
                                            // Clear
pub const HIB_IC_RTCALT0 = usize(0x00000001);  // RTC Alert0 Masked Interrupt
                                            // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCT register.
//
//*****************************************************************************
pub const HIB_RTCT_TRIM_M = usize(0x0000FFFF);  // RTC Trim Value
pub const HIB_RTCT_TRIM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCSS register.
//
//*****************************************************************************
pub const HIB_RTCSS_RTCSSM_M = usize(0x7FFF0000);  // RTC Sub Seconds Match
pub const HIB_RTCSS_RTCSSC_M = usize(0x00007FFF);  // RTC Sub Seconds Count
pub const HIB_RTCSS_RTCSSM_S = usize(16);
pub const HIB_RTCSS_RTCSSC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_IO register.
//
//*****************************************************************************
pub const HIB_IO_IOWRC = usize(0x80000000);  // I/O Write Complete
pub const HIB_IO_WURSTEN = usize(0x00000010);  // Reset Wake Source Enable
pub const HIB_IO_WUUNLK = usize(0x00000001);  // I/O Wake Pad Configuration
                                            // Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_DATA register.
//
//*****************************************************************************
pub const HIB_DATA_RTD_M = usize(0xFFFFFFFF);  // Hibernation Module NV Data
pub const HIB_DATA_RTD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALCTL register.
//
//*****************************************************************************
pub const HIB_CALCTL_CAL24 = usize(0x00000004);  // Calendar Mode
pub const HIB_CALCTL_CALEN = usize(0x00000001);  // RTC Calendar/Counter Mode Select

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CAL0 register.
//
//*****************************************************************************
pub const HIB_CAL0_VALID = usize(0x80000000);  // Valid Calendar Load
pub const HIB_CAL0_AMPM = usize(0x00400000);  // AM/PM Designation
pub const HIB_CAL0_HR_M = usize(0x001F0000);  // Hours
pub const HIB_CAL0_MIN_M = usize(0x00003F00);  // Minutes
pub const HIB_CAL0_SEC_M = usize(0x0000003F);  // Seconds
pub const HIB_CAL0_HR_S = usize(16);
pub const HIB_CAL0_MIN_S = usize(8);
pub const HIB_CAL0_SEC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CAL1 register.
//
//*****************************************************************************
pub const HIB_CAL1_VALID = usize(0x80000000);  // Valid Calendar Load
pub const HIB_CAL1_DOW_M = usize(0x07000000);  // Day of Week
pub const HIB_CAL1_YEAR_M = usize(0x007F0000);  // Year Value
pub const HIB_CAL1_MON_M = usize(0x00000F00);  // Month
pub const HIB_CAL1_DOM_M = usize(0x0000001F);  // Day of Month
pub const HIB_CAL1_DOW_S = usize(24);
pub const HIB_CAL1_YEAR_S = usize(16);
pub const HIB_CAL1_MON_S = usize(8);
pub const HIB_CAL1_DOM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALLD0 register.
//
//*****************************************************************************
pub const HIB_CALLD0_AMPM = usize(0x00400000);  // AM/PM Designation
pub const HIB_CALLD0_HR_M = usize(0x001F0000);  // Hours
pub const HIB_CALLD0_MIN_M = usize(0x00003F00);  // Minutes
pub const HIB_CALLD0_SEC_M = usize(0x0000003F);  // Seconds
pub const HIB_CALLD0_HR_S = usize(16);
pub const HIB_CALLD0_MIN_S = usize(8);
pub const HIB_CALLD0_SEC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALLD1 register.
//
//*****************************************************************************
pub const HIB_CALLD1_DOW_M = usize(0x07000000);  // Day of Week
pub const HIB_CALLD1_YEAR_M = usize(0x007F0000);  // Year Value
pub const HIB_CALLD1_MON_M = usize(0x00000F00);  // Month
pub const HIB_CALLD1_DOM_M = usize(0x0000001F);  // Day of Month
pub const HIB_CALLD1_DOW_S = usize(24);
pub const HIB_CALLD1_YEAR_S = usize(16);
pub const HIB_CALLD1_MON_S = usize(8);
pub const HIB_CALLD1_DOM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALM0 register.
//
//*****************************************************************************
pub const HIB_CALM0_AMPM = usize(0x00400000);  // AM/PM Designation
pub const HIB_CALM0_HR_M = usize(0x001F0000);  // Hours
pub const HIB_CALM0_MIN_M = usize(0x00003F00);  // Minutes
pub const HIB_CALM0_SEC_M = usize(0x0000003F);  // Seconds
pub const HIB_CALM0_HR_S = usize(16);
pub const HIB_CALM0_MIN_S = usize(8);
pub const HIB_CALM0_SEC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALM1 register.
//
//*****************************************************************************
pub const HIB_CALM1_DOM_M = usize(0x0000001F);  // Day of Month
pub const HIB_CALM1_DOM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_LOCK register.
//
//*****************************************************************************
pub const HIB_LOCK_HIBLOCK_M = usize(0xFFFFFFFF);  // HIbernate Lock
pub const HIB_LOCK_HIBLOCK_KEY = usize(0xA3359554);  // Hibernate Lock Key
pub const HIB_LOCK_HIBLOCK_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPCTL register.
//
//*****************************************************************************
pub const HIB_TPCTL_WAKE = usize(0x00000800);  // Wake from Hibernate on a Tamper
                                            // Event
pub const HIB_TPCTL_MEMCLR_M = usize(0x00000300);  // HIB Memory Clear on Tamper Event
pub const HIB_TPCTL_MEMCLR_NONE = usize(0x00000000);  // Do not Clear HIB memory on
                                            // tamper event
pub const HIB_TPCTL_MEMCLR_LOW32 = usize(0x00000100);  // Clear Lower 32 Bytes of HIB
                                            // memory on tamper event
pub const HIB_TPCTL_MEMCLR_HIGH32 = usize(0x00000200);  // Clear upper 32 Bytes of HIB
                                            // memory on tamper event
pub const HIB_TPCTL_MEMCLR_ALL = usize(0x00000300);  // Clear all HIB memory on tamper
                                            // event
pub const HIB_TPCTL_TPCLR = usize(0x00000010);  // Tamper Event Clear
pub const HIB_TPCTL_TPEN = usize(0x00000001);  // Tamper Module Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPSTAT register.
//
//*****************************************************************************
pub const HIB_TPSTAT_STATE_M = usize(0x0000000C);  // Tamper Module Status
pub const HIB_TPSTAT_STATE_DISABLED = usize(0x00000000);  // Tamper disabled
pub const HIB_TPSTAT_STATE_CONFIGED = usize(0x00000004);  // Tamper configured
pub const HIB_TPSTAT_STATE_ERROR = usize(0x00000008);  // Tamper pin event occurred
pub const HIB_TPSTAT_XOSCST = usize(0x00000002);  // External Oscillator Status
pub const HIB_TPSTAT_XOSCFAIL = usize(0x00000001);  // External Oscillator Failure

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPIO register.
//
//*****************************************************************************
pub const HIB_TPIO_GFLTR3 = usize(0x08000000);  // TMPR3 Glitch Filtering
pub const HIB_TPIO_PUEN3 = usize(0x04000000);  // TMPR3 Internal Weak Pull-up
                                            // Enable
pub const HIB_TPIO_LEV3 = usize(0x02000000);  // TMPR3 Trigger Level
pub const HIB_TPIO_EN3 = usize(0x01000000);  // TMPR3 Enable
pub const HIB_TPIO_GFLTR2 = usize(0x00080000);  // TMPR2 Glitch Filtering
pub const HIB_TPIO_PUEN2 = usize(0x00040000);  // TMPR2 Internal Weak Pull-up
                                            // Enable
pub const HIB_TPIO_LEV2 = usize(0x00020000);  // TMPR2 Trigger Level
pub const HIB_TPIO_EN2 = usize(0x00010000);  // TMPR2 Enable
pub const HIB_TPIO_GFLTR1 = usize(0x00000800);  // TMPR1 Glitch Filtering
pub const HIB_TPIO_PUEN1 = usize(0x00000400);  // TMPR1 Internal Weak Pull-up
                                            // Enable
pub const HIB_TPIO_LEV1 = usize(0x00000200);  // TMPR1 Trigger Level
pub const HIB_TPIO_EN1 = usize(0x00000100);  // TMPR1Enable
pub const HIB_TPIO_GFLTR0 = usize(0x00000008);  // TMPR0 Glitch Filtering
pub const HIB_TPIO_PUEN0 = usize(0x00000004);  // TMPR0 Internal Weak Pull-up
                                            // Enable
pub const HIB_TPIO_LEV0 = usize(0x00000002);  // TMPR0 Trigger Level
pub const HIB_TPIO_EN0 = usize(0x00000001);  // TMPR0 Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG0 register.
//
//*****************************************************************************
pub const HIB_TPLOG0_TIME_M = usize(0xFFFFFFFF);  // Tamper Log Calendar Information
pub const HIB_TPLOG0_TIME_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG1 register.
//
//*****************************************************************************
pub const HIB_TPLOG1_XOSC = usize(0x00010000);  // Status of external 32
pub const HIB_TPLOG1_TRIG3 = usize(0x00000008);  // Status of TMPR[3] Trigger
pub const HIB_TPLOG1_TRIG2 = usize(0x00000004);  // Status of TMPR[2] Trigger
pub const HIB_TPLOG1_TRIG1 = usize(0x00000002);  // Status of TMPR[1] Trigger
pub const HIB_TPLOG1_TRIG0 = usize(0x00000001);  // Status of TMPR[0] Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG2 register.
//
//*****************************************************************************
pub const HIB_TPLOG2_TIME_M = usize(0xFFFFFFFF);  // Tamper Log Calendar Information
pub const HIB_TPLOG2_TIME_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG3 register.
//
//*****************************************************************************
pub const HIB_TPLOG3_XOSC = usize(0x00010000);  // Status of external 32
pub const HIB_TPLOG3_TRIG3 = usize(0x00000008);  // Status of TMPR[3] Trigger
pub const HIB_TPLOG3_TRIG2 = usize(0x00000004);  // Status of TMPR[2] Trigger
pub const HIB_TPLOG3_TRIG1 = usize(0x00000002);  // Status of TMPR[1] Trigger
pub const HIB_TPLOG3_TRIG0 = usize(0x00000001);  // Status of TMPR[0] Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG4 register.
//
//*****************************************************************************
pub const HIB_TPLOG4_TIME_M = usize(0xFFFFFFFF);  // Tamper Log Calendar Information
pub const HIB_TPLOG4_TIME_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG5 register.
//
//*****************************************************************************
pub const HIB_TPLOG5_XOSC = usize(0x00010000);  // Status of external 32
pub const HIB_TPLOG5_TRIG3 = usize(0x00000008);  // Status of TMPR[3] Trigger
pub const HIB_TPLOG5_TRIG2 = usize(0x00000004);  // Status of TMPR[2] Trigger
pub const HIB_TPLOG5_TRIG1 = usize(0x00000002);  // Status of TMPR[1] Trigger
pub const HIB_TPLOG5_TRIG0 = usize(0x00000001);  // Status of TMPR[0] Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG6 register.
//
//*****************************************************************************
pub const HIB_TPLOG6_TIME_M = usize(0xFFFFFFFF);  // Tamper Log Calendar Information
pub const HIB_TPLOG6_TIME_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG7 register.
//
//*****************************************************************************
pub const HIB_TPLOG7_XOSC = usize(0x00010000);  // Status of external 32
pub const HIB_TPLOG7_TRIG3 = usize(0x00000008);  // Status of TMPR[3] Trigger
pub const HIB_TPLOG7_TRIG2 = usize(0x00000004);  // Status of TMPR[2] Trigger
pub const HIB_TPLOG7_TRIG1 = usize(0x00000002);  // Status of TMPR[1] Trigger
pub const HIB_TPLOG7_TRIG0 = usize(0x00000001);  // Status of TMPR[0] Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_PP register.
//
//*****************************************************************************
pub const HIB_PP_TAMPER = usize(0x00000002);  // Tamper Pin Presence
pub const HIB_PP_WAKENC = usize(0x00000001);  // Wake Pin Presence

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CC register.
//
//*****************************************************************************
pub const HIB_CC_SYSCLKEN = usize(0x00000001);  // RTCOSC to System Clock Enable

