//*****************************************************************************
//
// hw_aes.h - Macros used when accessing the AES hardware.
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
// The following are defines for the AES register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const KEY2_6 = usize(0x00000000); // AES Key 2_6
    pub const KEY2_7 = usize(0x00000004); // AES Key 2_7
    pub const KEY2_4 = usize(0x00000008); // AES Key 2_4
    pub const KEY2_5 = usize(0x0000000C); // AES Key 2_5
    pub const KEY2_2 = usize(0x00000010); // AES Key 2_2
    pub const KEY2_3 = usize(0x00000014); // AES Key 2_3
    pub const KEY2_0 = usize(0x00000018); // AES Key 2_0
    pub const KEY2_1 = usize(0x0000001C); // AES Key 2_1
    pub const KEY1_6 = usize(0x00000020); // AES Key 1_6
    pub const KEY1_7 = usize(0x00000024); // AES Key 1_7
    pub const KEY1_4 = usize(0x00000028); // AES Key 1_4
    pub const KEY1_5 = usize(0x0000002C); // AES Key 1_5
    pub const KEY1_2 = usize(0x00000030); // AES Key 1_2
    pub const KEY1_3 = usize(0x00000034); // AES Key 1_3
    pub const KEY1_0 = usize(0x00000038); // AES Key 1_0
    pub const KEY1_1 = usize(0x0000003C); // AES Key 1_1
    pub const IV_IN_0 = usize(0x00000040); // AES Initialization Vector Input
    // 0
    pub const IV_IN_1 = usize(0x00000044); // AES Initialization Vector Input
    // 1
    pub const IV_IN_2 = usize(0x00000048); // AES Initialization Vector Input
    // 2
    pub const IV_IN_3 = usize(0x0000004C); // AES Initialization Vector Input
    // 3
    pub const CTRL = usize(0x00000050); // AES Control
    pub const C_LENGTH_0 = usize(0x00000054); // AES Crypto Data Length 0
    pub const C_LENGTH_1 = usize(0x00000058); // AES Crypto Data Length 1
    pub const AUTH_LENGTH = usize(0x0000005C); // AES Authentication Data Length
    pub const DATA_IN_0 = usize(0x00000060); // AES Data RW Plaintext/Ciphertext
    // 0
    pub const DATA_IN_1 = usize(0x00000064); // AES Data RW Plaintext/Ciphertext
    // 1
    pub const DATA_IN_2 = usize(0x00000068); // AES Data RW Plaintext/Ciphertext
    // 2
    pub const DATA_IN_3 = usize(0x0000006C); // AES Data RW Plaintext/Ciphertext
    // 3
    pub const TAG_OUT_0 = usize(0x00000070); // AES Hash Tag Out 0
    pub const TAG_OUT_1 = usize(0x00000074); // AES Hash Tag Out 1
    pub const TAG_OUT_2 = usize(0x00000078); // AES Hash Tag Out 2
    pub const TAG_OUT_3 = usize(0x0000007C); // AES Hash Tag Out 3
    pub const REVISION = usize(0x00000080); // AES IP Revision Identifier
    pub const SYSCONFIG = usize(0x00000084); // AES System Configuration
    pub const SYSSTATUS = usize(0x00000088); // AES System Status
    pub const IRQSTATUS = usize(0x0000008C); // AES Interrupt Status
    pub const IRQENABLE = usize(0x00000090); // AES Interrupt Enable
    pub const DIRTYBITS = usize(0x00000094); // AES Dirty Bits
    pub const DMAIM = usize(0xFFFFA020); // AES DMA Interrupt Mask
    pub const DMARIS = usize(0xFFFFA024); // AES DMA Raw Interrupt Status
    pub const DMAMIS = usize(0xFFFFA028); // AES DMA Masked Interrupt Status
    pub const DMAIC = usize(0xFFFFA02C); // AES DMA Interrupt Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_6 register.
//
//*****************************************************************************
pub const KEY2_6 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_7 register.
//
//*****************************************************************************
pub const KEY2_7 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_4 register.
//
//*****************************************************************************
pub const KEY2_4 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_5 register.
//
//*****************************************************************************
pub const KEY2_5 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_2 register.
//
//*****************************************************************************
pub const KEY2_2 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_3 register.
//
//*****************************************************************************
pub const KEY2_3 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_0 register.
//
//*****************************************************************************
pub const KEY2_0 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_1 register.
//
//*****************************************************************************
pub const KEY2_1 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_6 register.
//
//*****************************************************************************
pub const KEY1_6 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_7 register.
//
//*****************************************************************************
pub const KEY1_7 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_4 register.
//
//*****************************************************************************
pub const KEY1_4 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_5 register.
//
//*****************************************************************************
pub const KEY1_5 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_2 register.
//
//*****************************************************************************
pub const KEY1_2 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_3 register.
//
//*****************************************************************************
pub const KEY1_3 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_0 register.
//
//*****************************************************************************
pub const KEY1_0 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_1 register.
//
//*****************************************************************************
pub const KEY1_1 = struct {
    pub const KEY_M = usize(0xFFFFFFFF); // Key Data
    pub const KEY_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IV_IN_0 register.
//
//*****************************************************************************
pub const IV_IN_0 = struct {
    pub const DATA_M = usize(0xFFFFFFFF); // Initialization Vector Input
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IV_IN_1 register.
//
//*****************************************************************************
pub const IV_IN_1 = struct {
    pub const DATA_M = usize(0xFFFFFFFF); // Initialization Vector Input
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IV_IN_2 register.
//
//*****************************************************************************
pub const IV_IN_2 = struct {
    pub const DATA_M = usize(0xFFFFFFFF); // Initialization Vector Input
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IV_IN_3 register.
//
//*****************************************************************************
pub const IV_IN_3 = struct {
    pub const DATA_M = usize(0xFFFFFFFF); // Initialization Vector Input
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_CTRL register.
//
//*****************************************************************************
pub const CTRL = struct {
    pub const CTXTRDY = usize(0x80000000); // Context Data Registers Ready
    pub const SVCTXTRDY = usize(0x40000000); // AES TAG/IV Block(s) Ready
    pub const SAVE_CONTEXT = usize(0x20000000); // TAG or Result IV Save
    pub const CCM_M_M = usize(0x01C00000); // Counter with CBC-MAC (CCM)
    pub const CCM_L_M = usize(0x00380000); // L Value
    pub const CCM_L_2 = usize(0x00080000); // width = 2
    pub const CCM_L_4 = usize(0x00180000); // width = 4
    pub const CCM_L_8 = usize(0x00380000); // width = 8
    pub const CCM = usize(0x00040000); // AES-CCM Mode Enable
    pub const GCM_M = usize(0x00030000); // AES-GCM Mode Enable
    pub const GCM_NOP = usize(0x00000000); // No operation
    pub const GCM_HLY0ZERO = usize(0x00010000); // GHASH with H loaded and
    // Y0-encrypted forced to zero
    pub const GCM_HLY0CALC = usize(0x00020000); // GHASH with H loaded and
    // Y0-encrypted calculated
    // internally
    pub const GCM_HY0CALC = usize(0x00030000); // Autonomous GHASH (both H and
    // Y0-encrypted calculated
    // internally)
    pub const CBCMAC = usize(0x00008000); // AES-CBC MAC Enable
    pub const F9 = usize(0x00004000); // AES f9 Mode Enable
    pub const F8 = usize(0x00002000); // AES f8 Mode Enable
    pub const XTS_M = usize(0x00001800); // AES-XTS Operation Enabled
    pub const XTS_NOP = usize(0x00000000); // No operation
    pub const XTS_TWEAKJL = usize(0x00000800); // Previous/intermediate tweak
    // value and j loaded (value is
    // loaded via IV, j is loaded via
    // the AAD length register)
    pub const XTS_K2IJL = usize(0x00001000); // Key2, n and j are loaded (n is
    // loaded via IV, j is loaded via
    // the AAD length register)
    pub const XTS_K2ILJ0 = usize(0x00001800); // Key2 and n are loaded; j=0 (n is
    // loaded via IV)
    pub const CFB = usize(0x00000400); // Full block AES cipher feedback
    // mode (CFB128) Enable
    pub const ICM = usize(0x00000200); // AES Integer Counter Mode (ICM)
    // Enable
    pub const CTR_WIDTH_M = usize(0x00000180); // AES-CTR Mode Counter Width
    pub const CTR_WIDTH_32 = usize(0x00000000); // Counter is 32 bits
    pub const CTR_WIDTH_64 = usize(0x00000080); // Counter is 64 bits
    pub const CTR_WIDTH_96 = usize(0x00000100); // Counter is 96 bits
    pub const CTR_WIDTH_128 = usize(0x00000180); // Counter is 128 bits
    pub const CTR = usize(0x00000040); // Counter Mode
    pub const MODE = usize(0x00000020); // ECB/CBC Mode
    pub const KEY_SIZE_M = usize(0x00000018); // Key Size
    pub const KEY_SIZE_128 = usize(0x00000008); // Key is 128 bits
    pub const KEY_SIZE_192 = usize(0x00000010); // Key is 192 bits
    pub const KEY_SIZE_256 = usize(0x00000018); // Key is 256 bits
    pub const DIRECTION = usize(0x00000004); // Encryption/Decryption Selection
    pub const INPUT_READY = usize(0x00000002); // Input Ready Status
    pub const OUTPUT_READY = usize(0x00000001); // Output Ready Status
    pub const CCM_M_S = usize(22);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_C_LENGTH_0
// register.
//
//*****************************************************************************
pub const C_LENGTH_0 = struct {
    pub const LENGTH_M = usize(0xFFFFFFFF); // Data Length
    pub const LENGTH_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_C_LENGTH_1
// register.
//
//*****************************************************************************
pub const C_LENGTH_1 = struct {
    pub const LENGTH_M = usize(0xFFFFFFFF); // Data Length
    pub const LENGTH_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_AUTH_LENGTH
// register.
//
//*****************************************************************************
pub const AUTH_LENGTH = struct {
    pub const AUTH_M = usize(0xFFFFFFFF); // Authentication Data Length
    pub const AUTH_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DATA_IN_0
// register.
//
//*****************************************************************************
pub const DATA_IN_0 = struct {
    pub const DATA_M = usize(0xFFFFFFFF); // Secure Data RW
    // Plaintext/Ciphertext
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DATA_IN_1
// register.
//
//*****************************************************************************
pub const DATA_IN_1 = struct {
    pub const DATA_M = usize(0xFFFFFFFF); // Secure Data RW
    // Plaintext/Ciphertext
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DATA_IN_2
// register.
//
//*****************************************************************************
pub const DATA_IN_2 = struct {
    pub const DATA_M = usize(0xFFFFFFFF); // Secure Data RW
    // Plaintext/Ciphertext
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DATA_IN_3
// register.
//
//*****************************************************************************
pub const DATA_IN_3 = struct {
    pub const DATA_M = usize(0xFFFFFFFF); // Secure Data RW
    // Plaintext/Ciphertext
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_TAG_OUT_0
// register.
//
//*****************************************************************************
pub const TAG_OUT_0 = struct {
    pub const HASH_M = usize(0xFFFFFFFF); // Hash Result
    pub const HASH_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_TAG_OUT_1
// register.
//
//*****************************************************************************
pub const TAG_OUT_1 = struct {
    pub const HASH_M = usize(0xFFFFFFFF); // Hash Result
    pub const HASH_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_TAG_OUT_2
// register.
//
//*****************************************************************************
pub const TAG_OUT_2 = struct {
    pub const HASH_M = usize(0xFFFFFFFF); // Hash Result
    pub const HASH_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_TAG_OUT_3
// register.
//
//*****************************************************************************
pub const TAG_OUT_3 = struct {
    pub const HASH_M = usize(0xFFFFFFFF); // Hash Result
    pub const HASH_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_REVISION register.
//
//*****************************************************************************
pub const REVISION = struct {
    pub const M = usize(0xFFFFFFFF); // Revision number
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_SYSCONFIG
// register.
//
//*****************************************************************************
pub const SYSCONFIG = struct {
    pub const K3 = usize(0x00001000); // K3 Select
    pub const KEYENC = usize(0x00000800); // Key Encoding
    pub const MAP_CONTEXT_OUT_ON_DATA_OUT = usize(0x00000200); // Map Context Out on Data Out
    // Enable
    pub const DMA_REQ_CONTEXT_OUT_EN = usize(0x00000100); // DMA Request Context Out Enable
    pub const DMA_REQ_CONTEXT_IN_EN = usize(0x00000080); // DMA Request Context In Enable
    pub const DMA_REQ_DATA_OUT_EN = usize(0x00000040); // DMA Request Data Out Enable
    pub const DMA_REQ_DATA_IN_EN = usize(0x00000020); // DMA Request Data In Enable
    pub const SOFTRESET = usize(0x00000002); // Soft reset
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_SYSSTATUS
// register.
//
//*****************************************************************************
pub const SYSSTATUS = struct {
    pub const RESETDONE = usize(0x00000001); // Reset Done
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IRQSTATUS
// register.
//
//*****************************************************************************
pub const IRQSTATUS = struct {
    pub const CONTEXT_OUT = usize(0x00000008); // Context Output Interrupt Status
    pub const DATA_OUT = usize(0x00000004); // Data Out Interrupt Status
    pub const DATA_IN = usize(0x00000002); // Data In Interrupt Status
    pub const CONTEXT_IN = usize(0x00000001); // Context In Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IRQENABLE
// register.
//
//*****************************************************************************
pub const IRQENABLE = struct {
    pub const CONTEXT_OUT = usize(0x00000008); // Context Out Interrupt Enable
    pub const DATA_OUT = usize(0x00000004); // Data Out Interrupt Enable
    pub const DATA_IN = usize(0x00000002); // Data In Interrupt Enable
    pub const CONTEXT_IN = usize(0x00000001); // Context In Interrupt Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DIRTYBITS
// register.
//
//*****************************************************************************
pub const DIRTYBITS = struct {
    pub const S_DIRTY = usize(0x00000002); // AES Dirty Bit
    pub const S_ACCESS = usize(0x00000001); // AES Access Bit
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DMAIM register.
//
//*****************************************************************************
pub const DMAIM = struct {
    pub const DOUT = usize(0x00000008); // Data Out DMA Done Interrupt Mask
    pub const DIN = usize(0x00000004); // Data In DMA Done Interrupt Mask
    pub const COUT = usize(0x00000002); // Context Out DMA Done Interrupt
    // Mask
    pub const CIN = usize(0x00000001); // Context In DMA Done Interrupt
    // Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DMARIS register.
//
//*****************************************************************************
pub const DMARIS = struct {
    pub const DOUT = usize(0x00000008); // Data Out DMA Done Raw Interrupt
    // Status
    pub const DIN = usize(0x00000004); // Data In DMA Done Raw Interrupt
    // Status
    pub const COUT = usize(0x00000002); // Context Out DMA Done Raw
    // Interrupt Status
    pub const CIN = usize(0x00000001); // Context In DMA Done Raw
    // Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DMAMIS register.
//
//*****************************************************************************
pub const DMAMIS = struct {
    pub const DOUT = usize(0x00000008); // Data Out DMA Done Masked
    // Interrupt Status
    pub const DIN = usize(0x00000004); // Data In DMA Done Masked
    // Interrupt Status
    pub const COUT = usize(0x00000002); // Context Out DMA Done Masked
    // Interrupt Status
    pub const CIN = usize(0x00000001); // Context In DMA Done Raw
    // Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DMAIC register.
//
//*****************************************************************************
pub const DMAIC = struct {
    pub const DOUT = usize(0x00000008); // Data Out DMA Done Interrupt
    // Clear
    pub const DIN = usize(0x00000004); // Data In DMA Done Interrupt Clear
    pub const COUT = usize(0x00000002); // Context Out DMA Done Masked
    // Interrupt Status
    pub const CIN = usize(0x00000001); // Context In DMA Done Raw
    // Interrupt Status
};
