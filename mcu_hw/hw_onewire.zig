//*****************************************************************************
//
// hw_onewire.h - Macros used when accessing the One wire hardware.
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
// The following are defines for the One wire register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const CS = usize(0x00000000); // 1-Wire Control and Status
    pub const TIM = usize(0x00000004); // 1-Wire Timing Override
    pub const DATW = usize(0x00000008); // 1-Wire Data Write
    pub const DATR = usize(0x0000000C); // 1-Wire Data Read
    pub const IM = usize(0x00000100); // 1-Wire Interrupt Mask
    pub const RIS = usize(0x00000104); // 1-Wire Raw Interrupt Status
    pub const MIS = usize(0x00000108); // 1-Wire Masked Interrupt Status
    pub const ICR = usize(0x0000010C); // 1-Wire Interrupt Clear
    pub const DMA = usize(0x00000120); // 1-Wire uDMA Control
    pub const PP = usize(0x00000FC0); // 1-Wire Peripheral Properties
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_CS register.
//
//*****************************************************************************
pub const CS = struct {
    pub const USEALT = usize(0x80000000); // Two Wire Enable
    pub const ALTP = usize(0x40000000); // Alternate Polarity Enable
    pub const BSIZE_M = usize(0x00070000); // Last Byte Size
    pub const BSIZE_8 = usize(0x00000000); // 8 bits (1 byte)
    pub const BSIZE_1 = usize(0x00010000); // 1 bit
    pub const BSIZE_2 = usize(0x00020000); // 2 bits
    pub const BSIZE_3 = usize(0x00030000); // 3 bits
    pub const BSIZE_4 = usize(0x00040000); // 4 bits
    pub const BSIZE_5 = usize(0x00050000); // 5 bits
    pub const BSIZE_6 = usize(0x00060000); // 6 bits
    pub const BSIZE_7 = usize(0x00070000); // 7 bits
    pub const STUCK = usize(0x00000400); // STUCK Status
    pub const NOATR = usize(0x00000200); // Answer-to-Reset Status
    pub const BUSY = usize(0x00000100); // Busy Status
    pub const SKATR = usize(0x00000080); // Skip Answer-to-Reset Enable
    pub const LSAM = usize(0x00000040); // Late Sample Enable
    pub const ODRV = usize(0x00000020); // Overdrive Enable
    pub const SZ_M = usize(0x00000018); // Data Operation Size
    pub const OP_M = usize(0x00000006); // Operation Request
    pub const OP_NONE = usize(0x00000000); // No operation
    pub const OP_RD = usize(0x00000002); // Read
    pub const OP_WR = usize(0x00000004); // Write
    pub const OP_WRRD = usize(0x00000006); // Write/Read
    pub const RST = usize(0x00000001); // Reset Request
    pub const SZ_S = usize(3);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_TIM register.
//
//*****************************************************************************
pub const TIM = struct {
    pub const W1TIM_M = usize(0xF0000000); // Value '1' Timing
    pub const W0TIM_M = usize(0x0F800000); // Value '0' Timing
    pub const W0REST_M = usize(0x00780000); // Rest Time
    pub const W1SAM_M = usize(0x00078000); // Sample Time
    pub const ATRSAM_M = usize(0x00007800); // Answer-to-Reset Sample
    pub const ATRTIM_M = usize(0x000007C0); // Answer-to-Reset/Rest Period
    pub const RSTTIM_M = usize(0x0000003F); // Reset Low Time
    pub const W1TIM_S = usize(28);
    pub const W0TIM_S = usize(23);
    pub const W0REST_S = usize(19);
    pub const W1SAM_S = usize(15);
    pub const ATRSAM_S = usize(11);
    pub const ATRTIM_S = usize(6);
    pub const RSTTIM_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_DATW register.
//
//*****************************************************************************
pub const DATW = struct {
    pub const B3_M = usize(0xFF000000); // Upper Data Byte
    pub const B2_M = usize(0x00FF0000); // Upper Middle Data Byte
    pub const B1_M = usize(0x0000FF00); // Lower Middle Data Byte
    pub const B0_M = usize(0x000000FF); // Lowest Data Byte
    pub const B3_S = usize(24);
    pub const B2_S = usize(16);
    pub const B1_S = usize(8);
    pub const B0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_DATR register.
//
//*****************************************************************************
pub const DATR = struct {
    pub const B3_M = usize(0xFF000000); // Upper Data Byte
    pub const B2_M = usize(0x00FF0000); // Upper Middle Data Byte
    pub const B1_M = usize(0x0000FF00); // Lower Middle Data Byte
    pub const B0_M = usize(0x000000FF); // Lowest Data Byte
    pub const B3_S = usize(24);
    pub const B2_S = usize(16);
    pub const B1_S = usize(8);
    pub const B0_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_IM register.
//
//*****************************************************************************
pub const IM = struct {
    pub const DMA = usize(0x00000010); // DMA Done Interrupt Mask
    pub const STUCK = usize(0x00000008); // Stuck Status Interrupt Mask
    pub const NOATR = usize(0x00000004); // No Answer-to-Reset Interrupt
    // Mask
    pub const OPC = usize(0x00000002); // Operation Complete Interrupt
    // Mask
    pub const RST = usize(0x00000001); // Reset Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_RIS register.
//
//*****************************************************************************
pub const RIS = struct {
    pub const DMA = usize(0x00000010); // DMA Done Raw Interrupt Status
    pub const STUCK = usize(0x00000008); // Stuck Status Raw Interrupt
    // Status
    pub const NOATR = usize(0x00000004); // No Answer-to-Reset Raw Interrupt
    // Status
    pub const OPC = usize(0x00000002); // Operation Complete Raw Interrupt
    // Status
    pub const RST = usize(0x00000001); // Reset Raw Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_MIS register.
//
//*****************************************************************************
pub const MIS = struct {
    pub const DMA = usize(0x00000010); // DMA Done Masked Interrupt Status
    pub const STUCK = usize(0x00000008); // Stuck Status Masked Interrupt
    // Status
    pub const NOATR = usize(0x00000004); // No Answer-to-Reset Masked
    // Interrupt Status
    pub const OPC = usize(0x00000002); // Operation Complete Masked
    // Interrupt Status
    pub const RST = usize(0x00000001); // Reset Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_ICR register.
//
//*****************************************************************************
pub const ICR = struct {
    pub const DMA = usize(0x00000010); // DMA Done Interrupt Clear
    pub const STUCK = usize(0x00000008); // Stuck Status Interrupt Clear
    pub const NOATR = usize(0x00000004); // No Answer-to-Reset Interrupt
    // Clear
    pub const OPC = usize(0x00000002); // Operation Complete Interrupt
    // Clear
    pub const RST = usize(0x00000001); // Reset Interrupt Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_DMA register.
//
//*****************************************************************************
pub const DMA = struct {
    pub const SG = usize(0x00000008); // Scatter-Gather Enable
    pub const DMAOP_M = usize(0x00000006); // uDMA Operation
    pub const DMAOP_DIS = usize(0x00000000); // uDMA disabled
    pub const DMAOP_RDSNG = usize(0x00000002); // uDMA single read: 1-Wire
    // requests uDMA to read
    // ONEWIREDATR register after each
    // read transaction
    pub const DMAOP_WRMUL = usize(0x00000004); // uDMA multiple write: 1-Wire
    // requests uDMA to load whenever
    // the ONEWIREDATW register is
    // empty
    pub const DMAOP_RDMUL = usize(0x00000006); // uDMA multiple read: An initial
    // read occurs and subsequent reads
    // start after uDMA has read the
    // ONEWIREDATR register
    pub const RST = usize(0x00000001); // uDMA Reset
};

//*****************************************************************************
//
// The following are defines for the bit fields in the ONEWIRE_O_PP register.
//
//*****************************************************************************
pub const PP = struct {
    pub const DMAP = usize(0x00000010); // uDMA Present
    pub const CNT_M = usize(0x00000003); // 1-Wire Bus Count
    pub const CNT_S = usize(0);
};
