//*****************************************************************************
//
// hw_eeprom.h - Macros used when accessing the EEPROM controller.
//
// Copyright (c) 2011-2017 Texas Instruments Incorporated.  All rights reserved.
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
// The following are defines for the EEPROM register offsets.
//
//*****************************************************************************
pub const EEPROM_EESIZE = usize(0x400AF000);  // EEPROM Size Information
pub const EEPROM_EEBLOCK = usize(0x400AF004);  // EEPROM Current Block
pub const EEPROM_EEOFFSET = usize(0x400AF008);  // EEPROM Current Offset
pub const EEPROM_EERDWR = usize(0x400AF010);  // EEPROM Read-Write
pub const EEPROM_EERDWRINC = usize(0x400AF014);  // EEPROM Read-Write with Increment
pub const EEPROM_EEDONE = usize(0x400AF018);  // EEPROM Done Status
pub const EEPROM_EESUPP = usize(0x400AF01C);  // EEPROM Support Control and
                                            // Status
pub const EEPROM_EEUNLOCK = usize(0x400AF020);  // EEPROM Unlock
pub const EEPROM_EEPROT = usize(0x400AF030);  // EEPROM Protection
pub const EEPROM_EEPASS0 = usize(0x400AF034);  // EEPROM Password
pub const EEPROM_EEPASS1 = usize(0x400AF038);  // EEPROM Password
pub const EEPROM_EEPASS2 = usize(0x400AF03C);  // EEPROM Password
pub const EEPROM_EEINT = usize(0x400AF040);  // EEPROM Interrupt
pub const EEPROM_EEHIDE0 = usize(0x400AF050);  // EEPROM Block Hide 0
pub const EEPROM_EEHIDE = usize(0x400AF050);  // EEPROM Block Hide
pub const EEPROM_EEHIDE1 = usize(0x400AF054);  // EEPROM Block Hide 1
pub const EEPROM_EEHIDE2 = usize(0x400AF058);  // EEPROM Block Hide 2
pub const EEPROM_EEDBGME = usize(0x400AF080);  // EEPROM Debug Mass Erase
pub const EEPROM_PP = usize(0x400AFFC0);  // EEPROM Peripheral Properties

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EESIZE register.
//
//*****************************************************************************
pub const EEPROM_EESIZE_WORDCNT_M = usize(0x0000FFFF);  // Number of 32-Bit Words
pub const EEPROM_EESIZE_BLKCNT_M = usize(0x07FF0000);  // Number of 16-Word Blocks
pub const EEPROM_EESIZE_WORDCNT_S = usize(0);
pub const EEPROM_EESIZE_BLKCNT_S = usize(16);

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEBLOCK register.
//
//*****************************************************************************
pub const EEPROM_EEBLOCK_BLOCK_M = usize(0x0000FFFF);  // Current Block
pub const EEPROM_EEBLOCK_BLOCK_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEOFFSET
// register.
//
//*****************************************************************************
pub const EEPROM_EEOFFSET_OFFSET_M = usize(0x0000000F);  // Current Address Offset
pub const EEPROM_EEOFFSET_OFFSET_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EERDWR register.
//
//*****************************************************************************
pub const EEPROM_EERDWR_VALUE_M = usize(0xFFFFFFFF);  // EEPROM Read or Write Data
pub const EEPROM_EERDWR_VALUE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EERDWRINC
// register.
//
//*****************************************************************************
pub const EEPROM_EERDWRINC_VALUE_M = usize(0xFFFFFFFF);  // EEPROM Read or Write Data with
                                            // Increment
pub const EEPROM_EERDWRINC_VALUE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEDONE register.
//
//*****************************************************************************
pub const EEPROM_EEDONE_WORKING = usize(0x00000001);  // EEPROM Working
pub const EEPROM_EEDONE_WKERASE = usize(0x00000004);  // Working on an Erase
pub const EEPROM_EEDONE_WKCOPY = usize(0x00000008);  // Working on a Copy
pub const EEPROM_EEDONE_NOPERM = usize(0x00000010);  // Write Without Permission
pub const EEPROM_EEDONE_WRBUSY = usize(0x00000020);  // Write Busy

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EESUPP register.
//
//*****************************************************************************
pub const EEPROM_EESUPP_ERETRY = usize(0x00000004);  // Erase Must Be Retried
pub const EEPROM_EESUPP_PRETRY = usize(0x00000008);  // Programming Must Be Retried

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEUNLOCK
// register.
//
//*****************************************************************************
pub const EEPROM_EEUNLOCK_UNLOCK_M = usize(0xFFFFFFFF);  // EEPROM Unlock

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPROT register.
//
//*****************************************************************************
pub const EEPROM_EEPROT_PROT_M = usize(0x00000007);  // Protection Control
pub const EEPROM_EEPROT_PROT_RWNPW = usize(0x00000000);  // This setting is the default. If
                                            // there is no password, the block
                                            // is not protected and is readable
                                            // and writable
pub const EEPROM_EEPROT_PROT_RWPW = usize(0x00000001);  // If there is a password, the
                                            // block is readable or writable
                                            // only when unlocked
pub const EEPROM_EEPROT_PROT_RONPW = usize(0x00000002);  // If there is no password, the
                                            // block is readable, not writable
pub const EEPROM_EEPROT_ACC = usize(0x00000008);  // Access Control

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS0 register.
//
//*****************************************************************************
pub const EEPROM_EEPASS0_PASS_M = usize(0xFFFFFFFF);  // Password
pub const EEPROM_EEPASS0_PASS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS1 register.
//
//*****************************************************************************
pub const EEPROM_EEPASS1_PASS_M = usize(0xFFFFFFFF);  // Password
pub const EEPROM_EEPASS1_PASS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS2 register.
//
//*****************************************************************************
pub const EEPROM_EEPASS2_PASS_M = usize(0xFFFFFFFF);  // Password
pub const EEPROM_EEPASS2_PASS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEINT register.
//
//*****************************************************************************
pub const EEPROM_EEINT_INT = usize(0x00000001);  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEHIDE0 register.
//
//*****************************************************************************
pub const EEPROM_EEHIDE0_HN_M = usize(0xFFFFFFFE);  // Hide Block

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEHIDE register.
//
//*****************************************************************************
pub const EEPROM_EEHIDE_HN_M = usize(0xFFFFFFFE);  // Hide Block

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEHIDE1 register.
//
//*****************************************************************************
pub const EEPROM_EEHIDE1_HN_M = usize(0xFFFFFFFF);  // Hide Block

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEHIDE2 register.
//
//*****************************************************************************
pub const EEPROM_EEHIDE2_HN_M = usize(0xFFFFFFFF);  // Hide Block

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEDBGME register.
//
//*****************************************************************************
pub const EEPROM_EEDBGME_ME = usize(0x00000001);  // Mass Erase
pub const EEPROM_EEDBGME_KEY_M = usize(0xFFFF0000);  // Erase Key
pub const EEPROM_EEDBGME_KEY_S = usize(16);

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_PP register.
//
//*****************************************************************************
pub const EEPROM_PP_SIZE_M = usize(0x0000FFFF);  // EEPROM Size
pub const EEPROM_PP_SIZE_64 = usize(0x00000000);  // 64 bytes of EEPROM
pub const EEPROM_PP_SIZE_128 = usize(0x00000001);  // 128 bytes of EEPROM
pub const EEPROM_PP_SIZE_256 = usize(0x00000003);  // 256 bytes of EEPROM
pub const EEPROM_PP_SIZE_512 = usize(0x00000007);  // 512 bytes of EEPROM
pub const EEPROM_PP_SIZE_1K = usize(0x0000000F);  // 1 KB of EEPROM
pub const EEPROM_PP_SIZE_2K = usize(0x0000001F);  // 2 KB of EEPROM
pub const EEPROM_PP_SIZE_3K = usize(0x0000003F);  // 3 KB of EEPROM
pub const EEPROM_PP_SIZE_4K = usize(0x0000007F);  // 4 KB of EEPROM
pub const EEPROM_PP_SIZE_5K = usize(0x000000FF);  // 5 KB of EEPROM
pub const EEPROM_PP_SIZE_6K = usize(0x000001FF);  // 6 KB of EEPROM
pub const EEPROM_PP_SIZE_S = usize(0);

