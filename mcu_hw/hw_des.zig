//*****************************************************************************
//
// hw_des.h - Macros used when accessing the DES hardware.
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
// The following are defines for the DES register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const KEY3_L = usize(0x00000000); // DES Key 3 LSW for 192-Bit Key
    pub const KEY3_H = usize(0x00000004); // DES Key 3 MSW for 192-Bit Key
    pub const KEY2_L = usize(0x00000008); // DES Key 2 LSW for 128-Bit Key
    pub const KEY2_H = usize(0x0000000C); // DES Key 2 MSW for 128-Bit Key
    pub const KEY1_L = usize(0x00000010); // DES Key 1 LSW for 64-Bit Key
    pub const KEY1_H = usize(0x00000014); // DES Key 1 MSW for 64-Bit Key
    pub const IV_L = usize(0x00000018); // DES Initialization Vector
    pub const IV_H = usize(0x0000001C); // DES Initialization Vector
    pub const CTRL = usize(0x00000020); // DES Control
    pub const LENGTH = usize(0x00000024); // DES Cryptographic Data Length
    pub const DATA_L = usize(0x00000028); // DES LSW Data RW
    pub const DATA_H = usize(0x0000002C); // DES MSW Data RW
    pub const REVISION = usize(0x00000030); // DES Revision Number
    pub const SYSCONFIG = usize(0x00000034); // DES System Configuration
    pub const SYSSTATUS = usize(0x00000038); // DES System Status
    pub const IRQSTATUS = usize(0x0000003C); // DES Interrupt Status
    pub const IRQENABLE = usize(0x00000040); // DES Interrupt Enable
    pub const DIRTYBITS = usize(0x00000044); // DES Dirty Bits
    pub const DMAIM = usize(0xFFFF8030); // DES DMA Interrupt Mask
    pub const DMARIS = usize(0xFFFF8034); // DES DMA Raw Interrupt Status
    pub const DMAMIS = usize(0xFFFF8038); // DES DMA Masked Interrupt Status
    pub const DMAIC = usize(0xFFFF803C); // DES DMA Interrupt Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY3_L register.
//
//*****************************************************************************
pub const KEY3_L = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY3_H register.
//
//*****************************************************************************
pub const KEY3_H = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY2_L register.
//
//*****************************************************************************
pub const KEY2_L = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY2_H register.
//
//*****************************************************************************
pub const KEY2_H = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY1_L register.
//
//*****************************************************************************
pub const KEY1_L = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY1_H register.
//
//*****************************************************************************
pub const KEY1_H = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_IV_L register.
//
//*****************************************************************************
pub const IV_L = struct {
    pub const M = usize(0xFFFFFFFF); // Initialization vector for CBC,
    // CFB modes (LSW)
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_IV_H register.
//
//*****************************************************************************
pub const IV_H = struct {
    pub const M = usize(0xFFFFFFFF); // Initialization vector for CBC,
    // CFB modes (MSW)
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_CTRL register.
//
//*****************************************************************************
pub const CTRL = struct {
    pub const CONTEXT = usize(0x80000000); // If 1, this read-only status bit
    // indicates that the context data
    // registers can be overwritten and
    // the host is permitted to write
    // the next context
    pub const MODE_M = usize(0x00000030); // Select CBC, ECB or CFB mode0x0:
    // ECB mode0x1: CBC mode0x2: CFB
    // mode0x3: reserved
    pub const TDES = usize(0x00000008); // Select DES or triple DES
    // encryption/decryption
    pub const DIRECTION = usize(0x00000004); // Select encryption/decryption
    // 0x0: decryption is selected0x1:
    // Encryption is selected
    pub const INPUT_READY = usize(0x00000002); // When 1, ready to encrypt/decrypt
    // data
    pub const OUTPUT_READY = usize(0x00000001); // When 1, Data decrypted/encrypted
    // ready
    pub const MODE_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_LENGTH register.
//
//*****************************************************************************
pub const LENGTH = struct {
    pub const M = usize(0xFFFFFFFF); // Cryptographic data length in
    // bytes for all modes
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DATA_L register.
//
//*****************************************************************************
pub const DATA_L = struct {
    pub const M = usize(0xFFFFFFFF); // Data for encryption/decryption,
    // LSW
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DATA_H register.
//
//*****************************************************************************
pub const DATA_H = struct {
    pub const M = usize(0xFFFFFFFF); // Data for encryption/decryption,
    // MSW
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_REVISION register.
//
//*****************************************************************************
pub const REVISION = struct {
    pub const M = usize(0xFFFFFFFF); // Revision number
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_SYSCONFIG
// register.
//
//*****************************************************************************
pub const SYSCONFIG = struct {
    pub const DMA_REQ_CONTEXT_IN_EN = usize(0x00000080); // DMA Request Context In Enable
    pub const DMA_REQ_DATA_OUT_EN = usize(0x00000040); // DMA Request Data Out Enable
    pub const DMA_REQ_DATA_IN_EN = usize(0x00000020); // DMA Request Data In Enable
    pub const SIDLE_M = usize(0x0000000C); // Sidle mode
    pub const SIDLE_FORCE = usize(0x00000000); // Force-idle mode
    pub const SOFTRESET = usize(0x00000002); // Soft reset
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_SYSSTATUS
// register.
//
//*****************************************************************************
pub const SYSSTATUS = struct {
    pub const RESETDONE = usize(0x00000001); // Reset Done
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_IRQSTATUS
// register.
//
//*****************************************************************************
pub const IRQSTATUS = struct {
    pub const DATA_OUT = usize(0x00000004); // This bit indicates data output
    // interrupt is active and triggers
    // the interrupt output
    pub const DATA_IN = usize(0x00000002); // This bit indicates data input
    // interrupt is active and triggers
    // the interrupt output
    pub const CONTEX_IN = usize(0x00000001); // This bit indicates context
    // interrupt is active and triggers
    // the interrupt output
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_IRQENABLE
// register.
//
//*****************************************************************************
pub const IRQENABLE = struct {
    pub const M_DATA_OUT = usize(0x00000004); // If this bit is set to 1 the data
    // output interrupt is enabled
    pub const M_DATA_IN = usize(0x00000002); // If this bit is set to 1 the data
    // input interrupt is enabled
    pub const M_CONTEX_IN = usize(0x00000001); // If this bit is set to 1 the
    // context interrupt is enabled
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DIRTYBITS
// register.
//
//*****************************************************************************
pub const DIRTYBITS = struct {
    pub const S_DIRTY = usize(0x00000002); // This bit is set to 1 by the
    // module if any of the DES_*
    // registers is written
    pub const S_ACCESS = usize(0x00000001); // This bit is set to 1 by the
    // module if any of the DES_*
    // registers is read
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DMAIM register.
//
//*****************************************************************************
pub const DMAIM = struct {
    pub const DOUT = usize(0x00000004); // Data Out DMA Done Interrupt Mask
    pub const DIN = usize(0x00000002); // Data In DMA Done Interrupt Mask
    pub const CIN = usize(0x00000001); // Context In DMA Done Interrupt
    // Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DMARIS register.
//
//*****************************************************************************
pub const DMARIS = struct {
    pub const DOUT = usize(0x00000004); // Data Out DMA Done Raw Interrupt
    // Status
    pub const DIN = usize(0x00000002); // Data In DMA Done Raw Interrupt
    // Status
    pub const CIN = usize(0x00000001); // Context In DMA Done Raw
    // Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DMAMIS register.
//
//*****************************************************************************
pub const DMAMIS = struct {
    pub const DOUT = usize(0x00000004); // Data Out DMA Done Masked
    // Interrupt Status
    pub const DIN = usize(0x00000002); // Data In DMA Done Masked
    // Interrupt Status
    pub const CIN = usize(0x00000001); // Context In DMA Done Raw
    // Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DMAIC register.
//
//*****************************************************************************
pub const DMAIC = struct {
    pub const DOUT = usize(0x00000004); // Data Out DMA Done Interrupt
    // Clear
    pub const DIN = usize(0x00000002); // Data In DMA Done Interrupt Clear
    pub const CIN = usize(0x00000001); // Context In DMA Done Raw
    // Interrupt Status
};
