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
pub const addressOf = struct {
    pub const RTCC = usize(0x400FC000); // Hibernation RTC Counter
    pub const RTCM0 = usize(0x400FC004); // Hibernation RTC Match 0
    pub const RTCLD = usize(0x400FC00C); // Hibernation RTC Load
    pub const CTL = usize(0x400FC010); // Hibernation Control
    pub const IM = usize(0x400FC014); // Hibernation Interrupt Mask
    pub const RIS = usize(0x400FC018); // Hibernation Raw Interrupt Status
    pub const MIS = usize(0x400FC01C); // Hibernation Masked Interrupt
    // Status
    pub const IC = usize(0x400FC020); // Hibernation Interrupt Clear
    pub const RTCT = usize(0x400FC024); // Hibernation RTC Trim
    pub const RTCSS = usize(0x400FC028); // Hibernation RTC Sub Seconds
    pub const IO = usize(0x400FC02C); // Hibernation IO Configuration
    pub const DATA = usize(0x400FC030); // Hibernation Data
    pub const CALCTL = usize(0x400FC300); // Hibernation Calendar Control
    pub const CAL0 = usize(0x400FC310); // Hibernation Calendar 0
    pub const CAL1 = usize(0x400FC314); // Hibernation Calendar 1
    pub const CALLD0 = usize(0x400FC320); // Hibernation Calendar Load 0
    pub const CALLD1 = usize(0x400FC324); // Hibernation Calendar Load
    pub const CALM0 = usize(0x400FC330); // Hibernation Calendar Match 0
    pub const CALM1 = usize(0x400FC334); // Hibernation Calendar Match 1
    pub const LOCK = usize(0x400FC360); // Hibernation Lock
    pub const TPCTL = usize(0x400FC400); // HIB Tamper Control
    pub const TPSTAT = usize(0x400FC404); // HIB Tamper Status
    pub const TPIO = usize(0x400FC410); // HIB Tamper I/O Control
    pub const TPLOG0 = usize(0x400FC4E0); // HIB Tamper Log 0
    pub const TPLOG1 = usize(0x400FC4E4); // HIB Tamper Log 1
    pub const TPLOG2 = usize(0x400FC4E8); // HIB Tamper Log 2
    pub const TPLOG3 = usize(0x400FC4EC); // HIB Tamper Log 3
    pub const TPLOG4 = usize(0x400FC4F0); // HIB Tamper Log 4
    pub const TPLOG5 = usize(0x400FC4F4); // HIB Tamper Log 5
    pub const TPLOG6 = usize(0x400FC4F8); // HIB Tamper Log 6
    pub const TPLOG7 = usize(0x400FC4FC); // HIB Tamper Log 7
    pub const PP = usize(0x400FCFC0); // Hibernation Peripheral
    // Properties
    pub const CC = usize(0x400FCFC8); // Hibernation Clock Control
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCC register.
//
//*****************************************************************************
pub const RTCC = struct {
    pub const M = usize(0xFFFFFFFF); // RTC Counter
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCM0 register.
//
//*****************************************************************************
pub const RTCM0 = struct {
    pub const M = usize(0xFFFFFFFF); // RTC Match 0
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCLD register.
//
//*****************************************************************************
pub const RTCLD = struct {
    pub const M = usize(0xFFFFFFFF); // RTC Load
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CTL register.
//
//*****************************************************************************
pub const CTL = struct {
    pub const WRC = usize(0x80000000); // Write Complete/Capable
    pub const RETCLR = usize(0x40000000); // GPIO Retention/Clear
    pub const OSCSEL = usize(0x00080000); // Oscillator Select
    pub const OSCDRV = usize(0x00020000); // Oscillator Drive Capability
    pub const OSCBYP = usize(0x00010000); // Oscillator Bypass
    pub const VBATSEL_M = usize(0x00006000); // Select for Low-Battery
    // Comparator
    pub const VBATSEL_1_9V = usize(0x00000000); // 1.9 Volts
    pub const VBATSEL_2_1V = usize(0x00002000); // 2.1 Volts (default)
    pub const VBATSEL_2_3V = usize(0x00004000); // 2.3 Volts
    pub const VBATSEL_2_5V = usize(0x00006000); // 2.5 Volts
    pub const BATCHK = usize(0x00000400); // Check Battery Status
    pub const BATWKEN = usize(0x00000200); // Wake on Low Battery
    pub const VDD3ON = usize(0x00000100); // VDD Powered
    pub const VABORT = usize(0x00000080); // Power Cut Abort Enable
    pub const CLK32EN = usize(0x00000040); // Clocking Enable
    pub const PINWEN = usize(0x00000010); // External Wake Pin Enable
    pub const RTCWEN = usize(0x00000008); // RTC Wake-up Enable
    pub const HIBREQ = usize(0x00000002); // Hibernation Request
    pub const RTCEN = usize(0x00000001); // RTC Timer Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_IM register.
//
//*****************************************************************************
pub const IM = struct {
    pub const VDDFAIL = usize(0x00000080); // VDD Fail Interrupt Mask
    pub const RSTWK = usize(0x00000040); // Reset Pad I/O Wake-Up Interrupt
    // Mask
    pub const PADIOWK = usize(0x00000020); // Pad I/O Wake-Up Interrupt Mask
    pub const WC = usize(0x00000010); // External Write Complete/Capable
    // Interrupt Mask
    pub const EXTW = usize(0x00000008); // External Wake-Up Interrupt Mask
    pub const LOWBAT = usize(0x00000004); // Low Battery Voltage Interrupt
    // Mask
    pub const RTCALT0 = usize(0x00000001); // RTC Alert 0 Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RIS register.
//
//*****************************************************************************
pub const RIS = struct {
    pub const VDDFAIL = usize(0x00000080); // VDD Fail Raw Interrupt Status
    pub const RSTWK = usize(0x00000040); // Reset Pad I/O Wake-Up Raw
    // Interrupt Status
    pub const PADIOWK = usize(0x00000020); // Pad I/O Wake-Up Raw Interrupt
    // Status
    pub const WC = usize(0x00000010); // Write Complete/Capable Raw
    // Interrupt Status
    pub const EXTW = usize(0x00000008); // External Wake-Up Raw Interrupt
    // Status
    pub const LOWBAT = usize(0x00000004); // Low Battery Voltage Raw
    // Interrupt Status
    pub const RTCALT0 = usize(0x00000001); // RTC Alert 0 Raw Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_MIS register.
//
//*****************************************************************************
pub const MIS = struct {
    pub const VDDFAIL = usize(0x00000080); // VDD Fail Interrupt Mask
    pub const RSTWK = usize(0x00000040); // Reset Pad I/O Wake-Up Interrupt
    // Mask
    pub const PADIOWK = usize(0x00000020); // Pad I/O Wake-Up Interrupt Mask
    pub const WC = usize(0x00000010); // Write Complete/Capable Masked
    // Interrupt Status
    pub const EXTW = usize(0x00000008); // External Wake-Up Masked
    // Interrupt Status
    pub const LOWBAT = usize(0x00000004); // Low Battery Voltage Masked
    // Interrupt Status
    pub const RTCALT0 = usize(0x00000001); // RTC Alert 0 Masked Interrupt
    // Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_IC register.
//
//*****************************************************************************
pub const IC = struct {
    pub const VDDFAIL = usize(0x00000080); // VDD Fail Interrupt Clear
    pub const RSTWK = usize(0x00000040); // Reset Pad I/O Wake-Up Interrupt
    // Clear
    pub const PADIOWK = usize(0x00000020); // Pad I/O Wake-Up Interrupt Clear
    pub const WC = usize(0x00000010); // Write Complete/Capable Interrupt
    // Clear
    pub const EXTW = usize(0x00000008); // External Wake-Up Interrupt Clear
    pub const LOWBAT = usize(0x00000004); // Low Battery Voltage Interrupt
    // Clear
    pub const RTCALT0 = usize(0x00000001); // RTC Alert0 Masked Interrupt
    // Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCT register.
//
//*****************************************************************************
pub const RTCT = struct {
    pub const TRIM_M = usize(0x0000FFFF); // RTC Trim Value
    pub const TRIM_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCSS register.
//
//*****************************************************************************
pub const RTCSS = struct {
    pub const RTCSSM_M = usize(0x7FFF0000); // RTC Sub Seconds Match
    pub const RTCSSC_M = usize(0x00007FFF); // RTC Sub Seconds Count
    pub const RTCSSM_S = usize(16);
    pub const RTCSSC_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_IO register.
//
//*****************************************************************************
pub const IO = struct {
    pub const IOWRC = usize(0x80000000); // I/O Write Complete
    pub const WURSTEN = usize(0x00000010); // Reset Wake Source Enable
    pub const WUUNLK = usize(0x00000001); // I/O Wake Pad Configuration
    // Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_DATA register.
//
//*****************************************************************************
pub const DATA = struct {
    pub const RTD_M = usize(0xFFFFFFFF); // Hibernation Module NV Data
    pub const RTD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALCTL register.
//
//*****************************************************************************
pub const CALCTL = struct {
    pub const CAL24 = usize(0x00000004); // Calendar Mode
    pub const CALEN = usize(0x00000001); // RTC Calendar/Counter Mode Select
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CAL0 register.
//
//*****************************************************************************
pub const CAL0 = struct {
    pub const VALID = usize(0x80000000); // Valid Calendar Load
    pub const AMPM = usize(0x00400000); // AM/PM Designation
    pub const HR_M = usize(0x001F0000); // Hours
    pub const MIN_M = usize(0x00003F00); // Minutes
    pub const SEC_M = usize(0x0000003F); // Seconds
    pub const HR_S = usize(16);
    pub const MIN_S = usize(8);
    pub const SEC_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CAL1 register.
//
//*****************************************************************************
pub const CAL1 = struct {
    pub const VALID = usize(0x80000000); // Valid Calendar Load
    pub const DOW_M = usize(0x07000000); // Day of Week
    pub const YEAR_M = usize(0x007F0000); // Year Value
    pub const MON_M = usize(0x00000F00); // Month
    pub const DOM_M = usize(0x0000001F); // Day of Month
    pub const DOW_S = usize(24);
    pub const YEAR_S = usize(16);
    pub const MON_S = usize(8);
    pub const DOM_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALLD0 register.
//
//*****************************************************************************
pub const CALLD0 = struct {
    pub const AMPM = usize(0x00400000); // AM/PM Designation
    pub const HR_M = usize(0x001F0000); // Hours
    pub const MIN_M = usize(0x00003F00); // Minutes
    pub const SEC_M = usize(0x0000003F); // Seconds
    pub const HR_S = usize(16);
    pub const MIN_S = usize(8);
    pub const SEC_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALLD1 register.
//
//*****************************************************************************
pub const CALLD1 = struct {
    pub const DOW_M = usize(0x07000000); // Day of Week
    pub const YEAR_M = usize(0x007F0000); // Year Value
    pub const MON_M = usize(0x00000F00); // Month
    pub const DOM_M = usize(0x0000001F); // Day of Month
    pub const DOW_S = usize(24);
    pub const YEAR_S = usize(16);
    pub const MON_S = usize(8);
    pub const DOM_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALM0 register.
//
//*****************************************************************************
pub const CALM0 = struct {
    pub const AMPM = usize(0x00400000); // AM/PM Designation
    pub const HR_M = usize(0x001F0000); // Hours
    pub const MIN_M = usize(0x00003F00); // Minutes
    pub const SEC_M = usize(0x0000003F); // Seconds
    pub const HR_S = usize(16);
    pub const MIN_S = usize(8);
    pub const SEC_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALM1 register.
//
//*****************************************************************************
pub const CALM1 = struct {
    pub const DOM_M = usize(0x0000001F); // Day of Month
    pub const DOM_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_LOCK register.
//
//*****************************************************************************
pub const LOCK = struct {
    pub const HIBLOCK_M = usize(0xFFFFFFFF); // HIbernate Lock
    pub const HIBLOCK_KEY = usize(0xA3359554); // Hibernate Lock Key
    pub const HIBLOCK_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPCTL register.
//
//*****************************************************************************
pub const TPCTL = struct {
    pub const WAKE = usize(0x00000800); // Wake from Hibernate on a Tamper
    // Event
    pub const MEMCLR_M = usize(0x00000300); // HIB Memory Clear on Tamper Event
    pub const MEMCLR_NONE = usize(0x00000000); // Do not Clear HIB memory on
    // tamper event
    pub const MEMCLR_LOW32 = usize(0x00000100); // Clear Lower 32 Bytes of HIB
    // memory on tamper event
    pub const MEMCLR_HIGH32 = usize(0x00000200); // Clear upper 32 Bytes of HIB
    // memory on tamper event
    pub const MEMCLR_ALL = usize(0x00000300); // Clear all HIB memory on tamper
    // event
    pub const TPCLR = usize(0x00000010); // Tamper Event Clear
    pub const TPEN = usize(0x00000001); // Tamper Module Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPSTAT register.
//
//*****************************************************************************
pub const TPSTAT = struct {
    pub const STATE_M = usize(0x0000000C); // Tamper Module Status
    pub const STATE_DISABLED = usize(0x00000000); // Tamper disabled
    pub const STATE_CONFIGED = usize(0x00000004); // Tamper configured
    pub const STATE_ERROR = usize(0x00000008); // Tamper pin event occurred
    pub const XOSCST = usize(0x00000002); // External Oscillator Status
    pub const XOSCFAIL = usize(0x00000001); // External Oscillator Failure
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPIO register.
//
//*****************************************************************************
pub const TPIO = struct {
    pub const GFLTR3 = usize(0x08000000); // TMPR3 Glitch Filtering
    pub const PUEN3 = usize(0x04000000); // TMPR3 Internal Weak Pull-up
    // Enable
    pub const LEV3 = usize(0x02000000); // TMPR3 Trigger Level
    pub const EN3 = usize(0x01000000); // TMPR3 Enable
    pub const GFLTR2 = usize(0x00080000); // TMPR2 Glitch Filtering
    pub const PUEN2 = usize(0x00040000); // TMPR2 Internal Weak Pull-up
    // Enable
    pub const LEV2 = usize(0x00020000); // TMPR2 Trigger Level
    pub const EN2 = usize(0x00010000); // TMPR2 Enable
    pub const GFLTR1 = usize(0x00000800); // TMPR1 Glitch Filtering
    pub const PUEN1 = usize(0x00000400); // TMPR1 Internal Weak Pull-up
    // Enable
    pub const LEV1 = usize(0x00000200); // TMPR1 Trigger Level
    pub const EN1 = usize(0x00000100); // TMPR1Enable
    pub const GFLTR0 = usize(0x00000008); // TMPR0 Glitch Filtering
    pub const PUEN0 = usize(0x00000004); // TMPR0 Internal Weak Pull-up
    // Enable
    pub const LEV0 = usize(0x00000002); // TMPR0 Trigger Level
    pub const EN0 = usize(0x00000001); // TMPR0 Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG0 register.
//
//*****************************************************************************
pub const TPLOG0 = struct {
    pub const TIME_M = usize(0xFFFFFFFF); // Tamper Log Calendar Information
    pub const TIME_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG1 register.
//
//*****************************************************************************
pub const TPLOG1 = struct {
    pub const XOSC = usize(0x00010000); // Status of external 32
    pub const TRIG3 = usize(0x00000008); // Status of TMPR[3] Trigger
    pub const TRIG2 = usize(0x00000004); // Status of TMPR[2] Trigger
    pub const TRIG1 = usize(0x00000002); // Status of TMPR[1] Trigger
    pub const TRIG0 = usize(0x00000001); // Status of TMPR[0] Trigger
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG2 register.
//
//*****************************************************************************
pub const TPLOG2 = struct {
    pub const TIME_M = usize(0xFFFFFFFF); // Tamper Log Calendar Information
    pub const TIME_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG3 register.
//
//*****************************************************************************
pub const TPLOG3 = struct {
    pub const XOSC = usize(0x00010000); // Status of external 32
    pub const TRIG3 = usize(0x00000008); // Status of TMPR[3] Trigger
    pub const TRIG2 = usize(0x00000004); // Status of TMPR[2] Trigger
    pub const TRIG1 = usize(0x00000002); // Status of TMPR[1] Trigger
    pub const TRIG0 = usize(0x00000001); // Status of TMPR[0] Trigger
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG4 register.
//
//*****************************************************************************
pub const TPLOG4 = struct {
    pub const TIME_M = usize(0xFFFFFFFF); // Tamper Log Calendar Information
    pub const TIME_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG5 register.
//
//*****************************************************************************
pub const TPLOG5 = struct {
    pub const XOSC = usize(0x00010000); // Status of external 32
    pub const TRIG3 = usize(0x00000008); // Status of TMPR[3] Trigger
    pub const TRIG2 = usize(0x00000004); // Status of TMPR[2] Trigger
    pub const TRIG1 = usize(0x00000002); // Status of TMPR[1] Trigger
    pub const TRIG0 = usize(0x00000001); // Status of TMPR[0] Trigger
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG6 register.
//
//*****************************************************************************
pub const TPLOG6 = struct {
    pub const TIME_M = usize(0xFFFFFFFF); // Tamper Log Calendar Information
    pub const TIME_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG7 register.
//
//*****************************************************************************
pub const TPLOG7 = struct {
    pub const XOSC = usize(0x00010000); // Status of external 32
    pub const TRIG3 = usize(0x00000008); // Status of TMPR[3] Trigger
    pub const TRIG2 = usize(0x00000004); // Status of TMPR[2] Trigger
    pub const TRIG1 = usize(0x00000002); // Status of TMPR[1] Trigger
    pub const TRIG0 = usize(0x00000001); // Status of TMPR[0] Trigger
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_PP register.
//
//*****************************************************************************
pub const PP = struct {
    pub const TAMPER = usize(0x00000002); // Tamper Pin Presence
    pub const WAKENC = usize(0x00000001); // Wake Pin Presence
};

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CC register.
//
//*****************************************************************************
pub const CC = struct {
    pub const SYSCLKEN = usize(0x00000001); // RTCOSC to System Clock Enable
};
