//*****************************************************************************
//
// hw_ssi.h - Macros used when accessing the SSI hardware.
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
// The following are defines for the SSI register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const CR0 = usize(0x00000000); // SSI Control 0
    pub const CR1 = usize(0x00000004); // SSI Control 1
    pub const DR = usize(0x00000008); // SSI Data
    pub const SR = usize(0x0000000C); // SSI Status
    pub const CPSR = usize(0x00000010); // SSI Clock Prescale
    pub const IM = usize(0x00000014); // SSI Interrupt Mask
    pub const RIS = usize(0x00000018); // SSI Raw Interrupt Status
    pub const MIS = usize(0x0000001C); // SSI Masked Interrupt Status
    pub const ICR = usize(0x00000020); // SSI Interrupt Clear
    pub const DMACTL = usize(0x00000024); // SSI DMA Control
    pub const PP = usize(0x00000FC0); // SSI Peripheral Properties
    pub const CC = usize(0x00000FC8); // SSI Clock Configuration
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR0 register.
//
//*****************************************************************************
pub const CR0 = struct {
    pub const SCR_M = usize(0x0000FF00); // SSI Serial Clock Rate
    pub const SPH = usize(0x00000080); // SSI Serial Clock Phase
    pub const SPO = usize(0x00000040); // SSI Serial Clock Polarity
    pub const FRF_M = usize(0x00000030); // SSI Frame Format Select
    pub const FRF_MOTO = usize(0x00000000); // Freescale SPI Frame Format
    pub const FRF_TI = usize(0x00000010); // Synchronous Serial Frame Format
    pub const FRF_NMW = usize(0x00000020); // MICROWIRE Frame Format
    pub const DSS_M = usize(0x0000000F); // SSI Data Size Select
    pub const DSS_4 = usize(0x00000003); // 4-bit data
    pub const DSS_5 = usize(0x00000004); // 5-bit data
    pub const DSS_6 = usize(0x00000005); // 6-bit data
    pub const DSS_7 = usize(0x00000006); // 7-bit data
    pub const DSS_8 = usize(0x00000007); // 8-bit data
    pub const DSS_9 = usize(0x00000008); // 9-bit data
    pub const DSS_10 = usize(0x00000009); // 10-bit data
    pub const DSS_11 = usize(0x0000000A); // 11-bit data
    pub const DSS_12 = usize(0x0000000B); // 12-bit data
    pub const DSS_13 = usize(0x0000000C); // 13-bit data
    pub const DSS_14 = usize(0x0000000D); // 14-bit data
    pub const DSS_15 = usize(0x0000000E); // 15-bit data
    pub const DSS_16 = usize(0x0000000F); // 16-bit data
    pub const SCR_S = usize(8);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR1 register.
//
//*****************************************************************************
pub const CR1 = struct {
    pub const EOM = usize(0x00000800); // Stop Frame (End of Message)
    pub const FSSHLDFRM = usize(0x00000400); // FSS Hold Frame
    pub const HSCLKEN = usize(0x00000200); // High Speed Clock Enable
    pub const DIR = usize(0x00000100); // SSI Direction of Operation
    pub const MODE_M = usize(0x000000C0); // SSI Mode
    pub const MODE_LEGACY = usize(0x00000000); // Legacy SSI mode
    pub const MODE_BI = usize(0x00000040); // Bi-SSI mode
    pub const MODE_QUAD = usize(0x00000080); // Quad-SSI Mode
    pub const MODE_ADVANCED = usize(0x000000C0); // Advanced SSI Mode with 8-bit
    // packet size
    pub const EOT = usize(0x00000010); // End of Transmission
    pub const MS = usize(0x00000004); // SSI Master/Slave Select
    pub const SSE = usize(0x00000002); // SSI Synchronous Serial Port
    // Enable
    pub const LBM = usize(0x00000001); // SSI Loopback Mode
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DR register.
//
//*****************************************************************************
pub const DR = struct {
    pub const DATA_M = usize(0x0000FFFF); // SSI Receive/Transmit Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_SR register.
//
//*****************************************************************************
pub const SR = struct {
    pub const BSY = usize(0x00000010); // SSI Busy Bit
    pub const RFF = usize(0x00000008); // SSI Receive FIFO Full
    pub const RNE = usize(0x00000004); // SSI Receive FIFO Not Empty
    pub const TNF = usize(0x00000002); // SSI Transmit FIFO Not Full
    pub const TFE = usize(0x00000001); // SSI Transmit FIFO Empty
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CPSR register.
//
//*****************************************************************************
pub const CPSR = struct {
    pub const CPSDVSR_M = usize(0x000000FF); // SSI Clock Prescale Divisor
    pub const CPSDVSR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_IM register.
//
//*****************************************************************************
pub const IM = struct {
    pub const EOTIM = usize(0x00000040); // End of Transmit Interrupt Mask
    pub const DMATXIM = usize(0x00000020); // SSI Transmit DMA Interrupt Mask
    pub const DMARXIM = usize(0x00000010); // SSI Receive DMA Interrupt Mask
    pub const TXIM = usize(0x00000008); // SSI Transmit FIFO Interrupt Mask
    pub const RXIM = usize(0x00000004); // SSI Receive FIFO Interrupt Mask
    pub const RTIM = usize(0x00000002); // SSI Receive Time-Out Interrupt
    // Mask
    pub const RORIM = usize(0x00000001); // SSI Receive Overrun Interrupt
    // Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_RIS register.
//
//*****************************************************************************
pub const RIS = struct {
    pub const EOTRIS = usize(0x00000040); // End of Transmit Raw Interrupt
    // Status
    pub const DMATXRIS = usize(0x00000020); // SSI Transmit DMA Raw Interrupt
    // Status
    pub const DMARXRIS = usize(0x00000010); // SSI Receive DMA Raw Interrupt
    // Status
    pub const TXRIS = usize(0x00000008); // SSI Transmit FIFO Raw Interrupt
    // Status
    pub const RXRIS = usize(0x00000004); // SSI Receive FIFO Raw Interrupt
    // Status
    pub const RTRIS = usize(0x00000002); // SSI Receive Time-Out Raw
    // Interrupt Status
    pub const RORRIS = usize(0x00000001); // SSI Receive Overrun Raw
    // Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_MIS register.
//
//*****************************************************************************
pub const MIS = struct {
    pub const EOTMIS = usize(0x00000040); // End of Transmit Masked Interrupt
    // Status
    pub const DMATXMIS = usize(0x00000020); // SSI Transmit DMA Masked
    // Interrupt Status
    pub const DMARXMIS = usize(0x00000010); // SSI Receive DMA Masked Interrupt
    // Status
    pub const TXMIS = usize(0x00000008); // SSI Transmit FIFO Masked
    // Interrupt Status
    pub const RXMIS = usize(0x00000004); // SSI Receive FIFO Masked
    // Interrupt Status
    pub const RTMIS = usize(0x00000002); // SSI Receive Time-Out Masked
    // Interrupt Status
    pub const RORMIS = usize(0x00000001); // SSI Receive Overrun Masked
    // Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_ICR register.
//
//*****************************************************************************
pub const ICR = struct {
    pub const EOTIC = usize(0x00000040); // End of Transmit Interrupt Clear
    pub const DMATXIC = usize(0x00000020); // SSI Transmit DMA Interrupt Clear
    pub const DMARXIC = usize(0x00000010); // SSI Receive DMA Interrupt Clear
    pub const RTIC = usize(0x00000002); // SSI Receive Time-Out Interrupt
    // Clear
    pub const RORIC = usize(0x00000001); // SSI Receive Overrun Interrupt
    // Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DMACTL register.
//
//*****************************************************************************
pub const DMACTL = struct {
    pub const TXDMAE = usize(0x00000002); // Transmit DMA Enable
    pub const RXDMAE = usize(0x00000001); // Receive DMA Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_PP register.
//
//*****************************************************************************
pub const PP = struct {
    pub const FSSHLDFRM = usize(0x00000008); // FSS Hold Frame Capability
    pub const MODE_M = usize(0x00000006); // Mode of Operation
    pub const MODE_LEGACY = usize(0x00000000); // Legacy SSI mode
    pub const MODE_ADVBI = usize(0x00000002); // Legacy mode, Advanced SSI mode
    // and Bi-SSI mode enabled
    pub const MODE_ADVBIQUAD = usize(0x00000004); // Legacy mode, Advanced mode,
    // Bi-SSI and Quad-SSI mode enabled
    pub const HSCLK = usize(0x00000001); // High Speed Capability
};

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CC register.
//
//*****************************************************************************
pub const CC = struct {
    pub const CS_M = usize(0x0000000F); // SSI Baud Clock Source
    pub const CS_SYSPLL = usize(0x00000000); // System clock (based on clock
    // source and divisor factor)
    pub const CS_PIOSC = usize(0x00000005); // PIOSC
};
