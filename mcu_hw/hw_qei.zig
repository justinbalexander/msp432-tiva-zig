//*****************************************************************************
//
// hw_qei.h - Macros used when accessing the QEI hardware.
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
// The following are defines for the QEI register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const CTL = usize(0x00000000); // QEI Control
    pub const STAT = usize(0x00000004); // QEI Status
    pub const POS = usize(0x00000008); // QEI Position
    pub const MAXPOS = usize(0x0000000C); // QEI Maximum Position
    pub const LOAD = usize(0x00000010); // QEI Timer Load
    pub const TIME = usize(0x00000014); // QEI Timer
    pub const COUNT = usize(0x00000018); // QEI Velocity Counter
    pub const SPEED = usize(0x0000001C); // QEI Velocity
    pub const INTEN = usize(0x00000020); // QEI Interrupt Enable
    pub const RIS = usize(0x00000024); // QEI Raw Interrupt Status
    pub const ISC = usize(0x00000028); // QEI Interrupt Status and Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_CTL register.
//
//*****************************************************************************
pub const CTL = struct {
    pub const FILTCNT_M = usize(0x000F0000); // Input Filter Prescale Count
    pub const FILTEN = usize(0x00002000); // Enable Input Filter
    pub const STALLEN = usize(0x00001000); // Stall QEI
    pub const INVI = usize(0x00000800); // Invert Index Pulse
    pub const INVB = usize(0x00000400); // Invert PhB
    pub const INVA = usize(0x00000200); // Invert PhA
    pub const VELDIV_M = usize(0x000001C0); // Predivide Velocity
    pub const VELDIV_1 = usize(0x00000000); // QEI clock /1
    pub const VELDIV_2 = usize(0x00000040); // QEI clock /2
    pub const VELDIV_4 = usize(0x00000080); // QEI clock /4
    pub const VELDIV_8 = usize(0x000000C0); // QEI clock /8
    pub const VELDIV_16 = usize(0x00000100); // QEI clock /16
    pub const VELDIV_32 = usize(0x00000140); // QEI clock /32
    pub const VELDIV_64 = usize(0x00000180); // QEI clock /64
    pub const VELDIV_128 = usize(0x000001C0); // QEI clock /128
    pub const VELEN = usize(0x00000020); // Capture Velocity
    pub const RESMODE = usize(0x00000010); // Reset Mode
    pub const CAPMODE = usize(0x00000008); // Capture Mode
    pub const SIGMODE = usize(0x00000004); // Signal Mode
    pub const SWAP = usize(0x00000002); // Swap Signals
    pub const ENABLE = usize(0x00000001); // Enable QEI
    pub const FILTCNT_S = usize(16);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_STAT register.
//
//*****************************************************************************
pub const STAT = struct {
    pub const DIRECTION = usize(0x00000002); // Direction of Rotation
    pub const ERROR = usize(0x00000001); // Error Detected
};

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_POS register.
//
//*****************************************************************************
pub const POS = struct {
    pub const M = usize(0xFFFFFFFF); // Current Position Integrator
    // Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_MAXPOS register.
//
//*****************************************************************************
pub const MAXPOS = struct {
    pub const M = usize(0xFFFFFFFF); // Maximum Position Integrator
    // Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_LOAD register.
//
//*****************************************************************************
pub const LOAD = struct {
    pub const M = usize(0xFFFFFFFF); // Velocity Timer Load Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_TIME register.
//
//*****************************************************************************
pub const TIME = struct {
    pub const M = usize(0xFFFFFFFF); // Velocity Timer Current Value
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_COUNT register.
//
//*****************************************************************************
pub const COUNT = struct {
    pub const M = usize(0xFFFFFFFF); // Velocity Pulse Count
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_SPEED register.
//
//*****************************************************************************
pub const SPEED = struct {
    pub const M = usize(0xFFFFFFFF); // Velocity
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_INTEN register.
//
//*****************************************************************************
pub const INTEN = struct {
    pub const ERROR = usize(0x00000008); // Phase Error Interrupt Enable
    pub const DIR = usize(0x00000004); // Direction Change Interrupt
    // Enable
    pub const TIMER = usize(0x00000002); // Timer Expires Interrupt Enable
    pub const INDEX = usize(0x00000001); // Index Pulse Detected Interrupt
    // Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_RIS register.
//
//*****************************************************************************
pub const RIS = struct {
    pub const ERROR = usize(0x00000008); // Phase Error Detected
    pub const DIR = usize(0x00000004); // Direction Change Detected
    pub const TIMER = usize(0x00000002); // Velocity Timer Expired
    pub const INDEX = usize(0x00000001); // Index Pulse Asserted
};

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_ISC register.
//
//*****************************************************************************
pub const ISC = struct {
    pub const ERROR = usize(0x00000008); // Phase Error Interrupt
    pub const DIR = usize(0x00000004); // Direction Change Interrupt
    pub const TIMER = usize(0x00000002); // Velocity Timer Expired Interrupt
    pub const INDEX = usize(0x00000001); // Index Pulse Interrupt
};
