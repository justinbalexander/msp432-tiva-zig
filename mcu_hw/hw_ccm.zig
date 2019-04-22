//*****************************************************************************
//
// hw_ccm.h - Macros used when accessing the CCM hardware.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
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
// The following are defines for the EC register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const CRCCTRL = usize(0x00000400); // CRC Control
    pub const CRCSEED = usize(0x00000410); // CRC SEED/Context
    pub const CRCDIN = usize(0x00000414); // CRC Data Input
    pub const CRCRSLTPP = usize(0x00000418); // CRC Post Processing Result
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CCM_O_CRCCTRL register.
//
//*****************************************************************************
pub const CRCCTRL = struct {
    pub const INIT_M = usize(0x00006000); // CRC Initialization
    pub const INIT_SEED = usize(0x00000000); // Use the CRCSEED register context
    // as the starting value
    pub const INIT_0 = usize(0x00004000); // Initialize to all '0s'
    pub const INIT_1 = usize(0x00006000); // Initialize to all '1s'
    pub const SIZE = usize(0x00001000); // Input Data Size
    pub const RESINV = usize(0x00000200); // Result Inverse Enable
    pub const OBR = usize(0x00000100); // Output Reverse Enable
    pub const BR = usize(0x00000080); // Bit reverse enable
    pub const ENDIAN_M = usize(0x00000030); // Endian Control
    pub const ENDIAN_SBHW = usize(0x00000000); // Configuration unchanged. (B3,
    // B2, B1, B0)
    pub const ENDIAN_SHW = usize(0x00000010); // Bytes are swapped in half-words
    // but half-words are not swapped
    // (B2, B3, B0, B1)
    pub const ENDIAN_SHWNB = usize(0x00000020); // Half-words are swapped but bytes
    // are not swapped in half-word.
    // (B1, B0, B3, B2)
    pub const ENDIAN_SBSW = usize(0x00000030); // Bytes are swapped in half-words
    // and half-words are swapped. (B0,
    // B1, B2, B3)
    pub const TYPE_M = usize(0x0000000F); // Operation Type
    pub const TYPE_P8055 = usize(0x00000000); // Polynomial 0x8005
    pub const TYPE_P1021 = usize(0x00000001); // Polynomial 0x1021
    pub const TYPE_P4C11DB7 = usize(0x00000002); // Polynomial 0x4C11DB7
    pub const TYPE_P1EDC6F41 = usize(0x00000003); // Polynomial 0x1EDC6F41
    pub const TYPE_TCPCHKSUM = usize(0x00000008); // TCP checksum
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CCM_O_CRCSEED register.
//
//*****************************************************************************
pub const CRCSEED = struct {
    pub const SEED_M = usize(0xFFFFFFFF); // SEED/Context Value
    pub const SEED_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CCM_O_CRCDIN register.
//
//*****************************************************************************
pub const CRCDIN = struct {
    pub const DATAIN_M = usize(0xFFFFFFFF); // Data Input
    pub const DATAIN_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CCM_O_CRCRSLTPP
// register.
//
//*****************************************************************************
pub const CRCRSLTPP = struct {
    pub const RSLTPP_M = usize(0xFFFFFFFF); // Post Processing Result
    pub const RSLTPP_S = usize(0);
};
