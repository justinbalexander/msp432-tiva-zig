//*****************************************************************************
//
// hw_watchdog.h - Macros used when accessing the Watchdog Timer hardware.
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
// The following are defines for the Watchdog Timer register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const LOAD = usize(0x00000000); // Watchdog Load
    pub const VALUE = usize(0x00000004); // Watchdog Value
    pub const CTL = usize(0x00000008); // Watchdog Control
    pub const ICR = usize(0x0000000C); // Watchdog Interrupt Clear
    pub const RIS = usize(0x00000010); // Watchdog Raw Interrupt Status
    pub const MIS = usize(0x00000014); // Watchdog Masked Interrupt Status
    pub const TEST = usize(0x00000418); // Watchdog Test
    pub const LOCK = usize(0x00000C00); // Watchdog Lock
};

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOAD register.
//
//*****************************************************************************
pub const LOAD = struct {
    pub const M = usize(0xFFFFFFFF); // Watchdog Load Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_VALUE register.
//
//*****************************************************************************
pub const VALUE = struct {
    pub const M = usize(0xFFFFFFFF); // Watchdog Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_CTL register.
//
//*****************************************************************************
pub const CTL = struct {
    pub const WRC = usize(0x80000000); // Write Complete
    pub const INTTYPE = usize(0x00000004); // Watchdog Interrupt Type
    pub const RESEN = usize(0x00000002); // Watchdog Reset Enable
    pub const INTEN = usize(0x00000001); // Watchdog Interrupt Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_ICR register.
//
//*****************************************************************************
pub const ICR = struct {
    pub const M = usize(0xFFFFFFFF); // Watchdog Interrupt Clear
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_RIS register.
//
//*****************************************************************************
pub const RIS = struct {
    pub const WDTRIS = usize(0x00000001); // Watchdog Raw Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_MIS register.
//
//*****************************************************************************
pub const MIS = struct {
    pub const WDTMIS = usize(0x00000001); // Watchdog Masked Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_TEST register.
//
//*****************************************************************************
pub const TEST = struct {
    pub const STALL = usize(0x00000100); // Watchdog Stall Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOCK register.
//
//*****************************************************************************
pub const LOCK = struct {
    pub const M = usize(0xFFFFFFFF); // Watchdog Lock
    pub const UNLOCKED = usize(0x00000000); // Unlocked
    pub const LOCKED = usize(0x00000001); // Locked
    pub const UNLOCK = usize(0x1ACCE551); // Unlocks the watchdog timer
};
