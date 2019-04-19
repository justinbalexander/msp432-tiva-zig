//*****************************************************************************
//
// hw_flash.h - Macros used when accessing the flash controller.
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
// The following are defines for the FLASH register offsets.
//
//*****************************************************************************
pub const FLASH_FMA = usize(0x400FD000);  // Flash Memory Address
pub const FLASH_FMD = usize(0x400FD004);  // Flash Memory Data
pub const FLASH_FMC = usize(0x400FD008);  // Flash Memory Control
pub const FLASH_FCRIS = usize(0x400FD00C);  // Flash Controller Raw Interrupt
                                            // Status
pub const FLASH_FCIM = usize(0x400FD010);  // Flash Controller Interrupt Mask
pub const FLASH_FCMISC = usize(0x400FD014);  // Flash Controller Masked
                                            // Interrupt Status and Clear
pub const FLASH_FMC2 = usize(0x400FD020);  // Flash Memory Control 2
pub const FLASH_FWBVAL = usize(0x400FD030);  // Flash Write Buffer Valid
pub const FLASH_FLPEKEY = usize(0x400FD03C);  // Flash Program/Erase Key
pub const FLASH_FWBN = usize(0x400FD100);  // Flash Write Buffer n
pub const FLASH_PP = usize(0x400FDFC0);  // Flash Peripheral Properties
pub const FLASH_FSIZE = usize(0x400FDFC0);  // Flash Size
pub const FLASH_SSIZE = usize(0x400FDFC4);  // SRAM Size
pub const FLASH_CONF = usize(0x400FDFC8);  // Flash Configuration Register
pub const FLASH_ROMSWMAP = usize(0x400FDFCC);  // ROM Software Map
pub const FLASH_DMASZ = usize(0x400FDFD0);  // Flash DMA Address Size
pub const FLASH_DMAST = usize(0x400FDFD4);  // Flash DMA Starting Address
pub const FLASH_RVP = usize(0x400FE0D4);  // Reset Vector Pointer
pub const FLASH_RMCTL = usize(0x400FE0F0);  // ROM Control
pub const FLASH_BOOTCFG = usize(0x400FE1D0);  // Boot Configuration
pub const FLASH_USERREG0 = usize(0x400FE1E0);  // User Register 0
pub const FLASH_USERREG1 = usize(0x400FE1E4);  // User Register 1
pub const FLASH_USERREG2 = usize(0x400FE1E8);  // User Register 2
pub const FLASH_USERREG3 = usize(0x400FE1EC);  // User Register 3
pub const FLASH_FMPRE0 = usize(0x400FE200);  // Flash Memory Protection Read
                                            // Enable 0
pub const FLASH_FMPRE1 = usize(0x400FE204);  // Flash Memory Protection Read
                                            // Enable 1
pub const FLASH_FMPRE2 = usize(0x400FE208);  // Flash Memory Protection Read
                                            // Enable 2
pub const FLASH_FMPRE3 = usize(0x400FE20C);  // Flash Memory Protection Read
                                            // Enable 3
pub const FLASH_FMPRE4 = usize(0x400FE210);  // Flash Memory Protection Read
                                            // Enable 4
pub const FLASH_FMPRE5 = usize(0x400FE214);  // Flash Memory Protection Read
                                            // Enable 5
pub const FLASH_FMPRE6 = usize(0x400FE218);  // Flash Memory Protection Read
                                            // Enable 6
pub const FLASH_FMPRE7 = usize(0x400FE21C);  // Flash Memory Protection Read
                                            // Enable 7
pub const FLASH_FMPRE8 = usize(0x400FE220);  // Flash Memory Protection Read
                                            // Enable 8
pub const FLASH_FMPRE9 = usize(0x400FE224);  // Flash Memory Protection Read
                                            // Enable 9
pub const FLASH_FMPRE10 = usize(0x400FE228);  // Flash Memory Protection Read
                                            // Enable 10
pub const FLASH_FMPRE11 = usize(0x400FE22C);  // Flash Memory Protection Read
                                            // Enable 11
pub const FLASH_FMPRE12 = usize(0x400FE230);  // Flash Memory Protection Read
                                            // Enable 12
pub const FLASH_FMPRE13 = usize(0x400FE234);  // Flash Memory Protection Read
                                            // Enable 13
pub const FLASH_FMPRE14 = usize(0x400FE238);  // Flash Memory Protection Read
                                            // Enable 14
pub const FLASH_FMPRE15 = usize(0x400FE23C);  // Flash Memory Protection Read
                                            // Enable 15
pub const FLASH_FMPPE0 = usize(0x400FE400);  // Flash Memory Protection Program
                                            // Enable 0
pub const FLASH_FMPPE1 = usize(0x400FE404);  // Flash Memory Protection Program
                                            // Enable 1
pub const FLASH_FMPPE2 = usize(0x400FE408);  // Flash Memory Protection Program
                                            // Enable 2
pub const FLASH_FMPPE3 = usize(0x400FE40C);  // Flash Memory Protection Program
                                            // Enable 3
pub const FLASH_FMPPE4 = usize(0x400FE410);  // Flash Memory Protection Program
                                            // Enable 4
pub const FLASH_FMPPE5 = usize(0x400FE414);  // Flash Memory Protection Program
                                            // Enable 5
pub const FLASH_FMPPE6 = usize(0x400FE418);  // Flash Memory Protection Program
                                            // Enable 6
pub const FLASH_FMPPE7 = usize(0x400FE41C);  // Flash Memory Protection Program
                                            // Enable 7
pub const FLASH_FMPPE8 = usize(0x400FE420);  // Flash Memory Protection Program
                                            // Enable 8
pub const FLASH_FMPPE9 = usize(0x400FE424);  // Flash Memory Protection Program
                                            // Enable 9
pub const FLASH_FMPPE10 = usize(0x400FE428);  // Flash Memory Protection Program
                                            // Enable 10
pub const FLASH_FMPPE11 = usize(0x400FE42C);  // Flash Memory Protection Program
                                            // Enable 11
pub const FLASH_FMPPE12 = usize(0x400FE430);  // Flash Memory Protection Program
                                            // Enable 12
pub const FLASH_FMPPE13 = usize(0x400FE434);  // Flash Memory Protection Program
                                            // Enable 13
pub const FLASH_FMPPE14 = usize(0x400FE438);  // Flash Memory Protection Program
                                            // Enable 14
pub const FLASH_FMPPE15 = usize(0x400FE43C);  // Flash Memory Protection Program
                                            // Enable 15

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMA register.
//
//*****************************************************************************
pub const FLASH_FMA_OFFSET_M = usize(0x000FFFFF);  // Address Offset
pub const FLASH_FMA_OFFSET_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMD register.
//
//*****************************************************************************
pub const FLASH_FMD_DATA_M = usize(0xFFFFFFFF);  // Data Value
pub const FLASH_FMD_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMC register.
//
//*****************************************************************************
pub const FLASH_FMC_WRKEY = usize(0xA4420000);  // FLASH write key
pub const FLASH_FMC_COMT = usize(0x00000008);  // Commit Register Value
pub const FLASH_FMC_MERASE = usize(0x00000004);  // Mass Erase Flash Memory
pub const FLASH_FMC_ERASE = usize(0x00000002);  // Erase a Page of Flash Memory
pub const FLASH_FMC_WRITE = usize(0x00000001);  // Write a Word into Flash Memory

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FCRIS register.
//
//*****************************************************************************
pub const FLASH_FCRIS_PROGRIS = usize(0x00002000);  // Program Verify Error Raw
                                            // Interrupt Status
pub const FLASH_FCRIS_ERRIS = usize(0x00000800);  // Erase Verify Error Raw Interrupt
                                            // Status
pub const FLASH_FCRIS_INVDRIS = usize(0x00000400);  // Invalid Data Raw Interrupt
                                            // Status
pub const FLASH_FCRIS_VOLTRIS = usize(0x00000200);  // Pump Voltage Raw Interrupt
                                            // Status
pub const FLASH_FCRIS_ERIS = usize(0x00000004);  // EEPROM Raw Interrupt Status
pub const FLASH_FCRIS_PRIS = usize(0x00000002);  // Programming Raw Interrupt Status
pub const FLASH_FCRIS_ARIS = usize(0x00000001);  // Access Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FCIM register.
//
//*****************************************************************************
pub const FLASH_FCIM_PROGMASK = usize(0x00002000);  // PROGVER Interrupt Mask
pub const FLASH_FCIM_ERMASK = usize(0x00000800);  // ERVER Interrupt Mask
pub const FLASH_FCIM_INVDMASK = usize(0x00000400);  // Invalid Data Interrupt Mask
pub const FLASH_FCIM_VOLTMASK = usize(0x00000200);  // VOLT Interrupt Mask
pub const FLASH_FCIM_EMASK = usize(0x00000004);  // EEPROM Interrupt Mask
pub const FLASH_FCIM_PMASK = usize(0x00000002);  // Programming Interrupt Mask
pub const FLASH_FCIM_AMASK = usize(0x00000001);  // Access Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FCMISC register.
//
//*****************************************************************************
pub const FLASH_FCMISC_PROGMISC = usize(0x00002000);  // PROGVER Masked Interrupt Status
                                            // and Clear
pub const FLASH_FCMISC_ERMISC = usize(0x00000800);  // ERVER Masked Interrupt Status
                                            // and Clear
pub const FLASH_FCMISC_INVDMISC = usize(0x00000400);  // Invalid Data Masked Interrupt
                                            // Status and Clear
pub const FLASH_FCMISC_VOLTMISC = usize(0x00000200);  // VOLT Masked Interrupt Status and
                                            // Clear
pub const FLASH_FCMISC_EMISC = usize(0x00000004);  // EEPROM Masked Interrupt Status
                                            // and Clear
pub const FLASH_FCMISC_PMISC = usize(0x00000002);  // Programming Masked Interrupt
                                            // Status and Clear
pub const FLASH_FCMISC_AMISC = usize(0x00000001);  // Access Masked Interrupt Status
                                            // and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMC2 register.
//
//*****************************************************************************
pub const FLASH_FMC2_WRKEY = usize(0xA4420000);  // FLASH write key
pub const FLASH_FMC2_WRBUF = usize(0x00000001);  // Buffered Flash Memory Write

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FWBVAL register.
//
//*****************************************************************************
pub const FLASH_FWBVAL_FWB_M = usize(0xFFFFFFFF);  // Flash Memory Write Buffer

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FLPEKEY register.
//
//*****************************************************************************
pub const FLASH_FLPEKEY_PEKEY_M = usize(0x0000FFFF);  // Key Value
pub const FLASH_FLPEKEY_PEKEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FWBN register.
//
//*****************************************************************************
pub const FLASH_FWBN_DATA_M = usize(0xFFFFFFFF);  // Data

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_PP register.
//
//*****************************************************************************
pub const FLASH_PP_PFC = usize(0x40000000);  // Prefetch Buffer Mode
pub const FLASH_PP_FMM = usize(0x20000000);  // Flash Mirror Mode
pub const FLASH_PP_DFA = usize(0x10000000);  // DMA Flash Access
pub const FLASH_PP_EESS_M = usize(0x00780000);  // EEPROM Sector Size of the
                                            // physical bank
pub const FLASH_PP_EESS_1KB = usize(0x00000000);  // 1 KB
pub const FLASH_PP_EESS_2KB = usize(0x00080000);  // 2 KB
pub const FLASH_PP_EESS_4KB = usize(0x00100000);  // 4 KB
pub const FLASH_PP_EESS_8KB = usize(0x00180000);  // 8 KB
pub const FLASH_PP_MAINSS_M = usize(0x00070000);  // Flash Sector Size of the
                                            // physical bank
pub const FLASH_PP_MAINSS_1KB = usize(0x00000000);  // 1 KB
pub const FLASH_PP_MAINSS_2KB = usize(0x00010000);  // 2 KB
pub const FLASH_PP_MAINSS_4KB = usize(0x00020000);  // 4 KB
pub const FLASH_PP_MAINSS_8KB = usize(0x00030000);  // 8 KB
pub const FLASH_PP_MAINSS_16KB = usize(0x00040000);  // 16 KB
pub const FLASH_PP_SIZE_M = usize(0x0000FFFF);  // Flash Size
pub const FLASH_PP_SIZE_512KB = usize(0x000000FF);  // 512 KB of Flash
pub const FLASH_PP_SIZE_1MB = usize(0x000001FF);  // 1024 KB of Flash

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FSIZE register.
//
//*****************************************************************************
pub const FLASH_FSIZE_SIZE_M = usize(0x0000FFFF);  // Flash Size
pub const FLASH_FSIZE_SIZE_32KB = usize(0x0000000F);  // 32 KB of Flash
pub const FLASH_FSIZE_SIZE_64KB = usize(0x0000001F);  // 64 KB of Flash
pub const FLASH_FSIZE_SIZE_128KB = usize(0x0000003F);  // 128 KB of Flash
pub const FLASH_FSIZE_SIZE_256KB = usize(0x0000007F);  // 256 KB of Flash

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_SSIZE register.
//
//*****************************************************************************
pub const FLASH_SSIZE_SIZE_M = usize(0x0000FFFF);  // SRAM Size
pub const FLASH_SSIZE_SIZE_12KB = usize(0x0000002F);  // 12 KB of SRAM
pub const FLASH_SSIZE_SIZE_24KB = usize(0x0000005F);  // 24 KB of SRAM
pub const FLASH_SSIZE_SIZE_32KB = usize(0x0000007F);  // 32 KB of SRAM
pub const FLASH_SSIZE_SIZE_256KB = usize(0x000003FF);  // 256 KB of SRAM

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_CONF register.
//
//*****************************************************************************
pub const FLASH_CONF_FMME = usize(0x40000000);  // Flash Mirror Mode Enable
pub const FLASH_CONF_SPFE = usize(0x20000000);  // Single Prefetch Mode Enable
pub const FLASH_CONF_CLRTV = usize(0x00100000);  // Clear Valid Tags
pub const FLASH_CONF_FPFON = usize(0x00020000);  // Force Prefetch On
pub const FLASH_CONF_FPFOFF = usize(0x00010000);  // Force Prefetch Off

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_ROMSWMAP register.
//
//*****************************************************************************
pub const FLASH_ROMSWMAP_SAFERTOS = usize(0x00000001);  // SafeRTOS Present
pub const FLASH_ROMSWMAP_SW0EN_M = usize(0x00000003);  // ROM SW Region 0 Availability
pub const FLASH_ROMSWMAP_SW0EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW0EN_CORE = usize(0x00000001);  // Region available to core
pub const FLASH_ROMSWMAP_SW1EN_M = usize(0x0000000C);  // ROM SW Region 1 Availability
pub const FLASH_ROMSWMAP_SW1EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW1EN_CORE = usize(0x00000004);  // Region available to core
pub const FLASH_ROMSWMAP_SW2EN_M = usize(0x00000030);  // ROM SW Region 2 Availability
pub const FLASH_ROMSWMAP_SW2EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW2EN_CORE = usize(0x00000010);  // Region available to core
pub const FLASH_ROMSWMAP_SW3EN_M = usize(0x000000C0);  // ROM SW Region 3 Availability
pub const FLASH_ROMSWMAP_SW3EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW3EN_CORE = usize(0x00000040);  // Region available to core
pub const FLASH_ROMSWMAP_SW4EN_M = usize(0x00000300);  // ROM SW Region 4 Availability
pub const FLASH_ROMSWMAP_SW4EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW4EN_CORE = usize(0x00000100);  // Region available to core
pub const FLASH_ROMSWMAP_SW5EN_M = usize(0x00000C00);  // ROM SW Region 5 Availability
pub const FLASH_ROMSWMAP_SW5EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW5EN_CORE = usize(0x00000400);  // Region available to core
pub const FLASH_ROMSWMAP_SW6EN_M = usize(0x00003000);  // ROM SW Region 6 Availability
pub const FLASH_ROMSWMAP_SW6EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW6EN_CORE = usize(0x00001000);  // Region available to core
pub const FLASH_ROMSWMAP_SW7EN_M = usize(0x0000C000);  // ROM SW Region 7 Availability
pub const FLASH_ROMSWMAP_SW7EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW7EN_CORE = usize(0x00004000);  // Region available to core

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_DMASZ register.
//
//*****************************************************************************
pub const FLASH_DMASZ_SIZE_M = usize(0x0003FFFF);  // uDMA-accessible Memory Size
pub const FLASH_DMASZ_SIZE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_DMAST register.
//
//*****************************************************************************
pub const FLASH_DMAST_ADDR_M = usize(0x1FFFF800);  // Contains the starting address of
                                            // the flash region accessible by
                                            // uDMA if the FLASHPP register DFA
                                            // bit is set
pub const FLASH_DMAST_ADDR_S = usize(11);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_RVP register.
//
//*****************************************************************************
pub const FLASH_RVP_RV_M = usize(0xFFFFFFFF);  // Reset Vector Pointer Address
pub const FLASH_RVP_RV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_RMCTL register.
//
//*****************************************************************************
pub const FLASH_RMCTL_BA = usize(0x00000001);  // Boot Alias

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_BOOTCFG register.
//
//*****************************************************************************
pub const FLASH_BOOTCFG_NW = usize(0x80000000);  // Not Written
pub const FLASH_BOOTCFG_PORT_M = usize(0x0000E000);  // Boot GPIO Port
pub const FLASH_BOOTCFG_PORT_A = usize(0x00000000);  // Port A
pub const FLASH_BOOTCFG_PORT_B = usize(0x00002000);  // Port B
pub const FLASH_BOOTCFG_PORT_C = usize(0x00004000);  // Port C
pub const FLASH_BOOTCFG_PORT_D = usize(0x00006000);  // Port D
pub const FLASH_BOOTCFG_PORT_E = usize(0x00008000);  // Port E
pub const FLASH_BOOTCFG_PORT_F = usize(0x0000A000);  // Port F
pub const FLASH_BOOTCFG_PORT_G = usize(0x0000C000);  // Port G
pub const FLASH_BOOTCFG_PORT_H = usize(0x0000E000);  // Port H
pub const FLASH_BOOTCFG_PIN_M = usize(0x00001C00);  // Boot GPIO Pin
pub const FLASH_BOOTCFG_PIN_0 = usize(0x00000000);  // Pin 0
pub const FLASH_BOOTCFG_PIN_1 = usize(0x00000400);  // Pin 1
pub const FLASH_BOOTCFG_PIN_2 = usize(0x00000800);  // Pin 2
pub const FLASH_BOOTCFG_PIN_3 = usize(0x00000C00);  // Pin 3
pub const FLASH_BOOTCFG_PIN_4 = usize(0x00001000);  // Pin 4
pub const FLASH_BOOTCFG_PIN_5 = usize(0x00001400);  // Pin 5
pub const FLASH_BOOTCFG_PIN_6 = usize(0x00001800);  // Pin 6
pub const FLASH_BOOTCFG_PIN_7 = usize(0x00001C00);  // Pin 7
pub const FLASH_BOOTCFG_POL = usize(0x00000200);  // Boot GPIO Polarity
pub const FLASH_BOOTCFG_EN = usize(0x00000100);  // Boot GPIO Enable
pub const FLASH_BOOTCFG_KEY = usize(0x00000010);  // KEY Select
pub const FLASH_BOOTCFG_DBG1 = usize(0x00000002);  // Debug Control 1
pub const FLASH_BOOTCFG_DBG0 = usize(0x00000001);  // Debug Control 0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG0 register.
//
//*****************************************************************************
pub const FLASH_USERREG0_DATA_M = usize(0xFFFFFFFF);  // User Data
pub const FLASH_USERREG0_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG1 register.
//
//*****************************************************************************
pub const FLASH_USERREG1_DATA_M = usize(0xFFFFFFFF);  // User Data
pub const FLASH_USERREG1_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG2 register.
//
//*****************************************************************************
pub const FLASH_USERREG2_DATA_M = usize(0xFFFFFFFF);  // User Data
pub const FLASH_USERREG2_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG3 register.
//
//*****************************************************************************
pub const FLASH_USERREG3_DATA_M = usize(0xFFFFFFFF);  // User Data
pub const FLASH_USERREG3_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPRE8 register.
//
//*****************************************************************************
pub const FLASH_FMPRE8_READ_ENABLE_M = usize(0xFFFFFFFF);  // Flash Read Enable
pub const FLASH_FMPRE8_READ_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPRE9 register.
//
//*****************************************************************************
pub const FLASH_FMPRE9_READ_ENABLE_M = usize(0xFFFFFFFF);  // Flash Read Enable
pub const FLASH_FMPRE9_READ_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPRE10 register.
//
//*****************************************************************************
pub const FLASH_FMPRE10_READ_ENABLE_M = usize(0xFFFFFFFF);  // Flash Read Enable
pub const FLASH_FMPRE10_READ_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPRE11 register.
//
//*****************************************************************************
pub const FLASH_FMPRE11_READ_ENABLE_M = usize(0xFFFFFFFF);  // Flash Read Enable
pub const FLASH_FMPRE11_READ_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPRE12 register.
//
//*****************************************************************************
pub const FLASH_FMPRE12_READ_ENABLE_M = usize(0xFFFFFFFF);  // Flash Read Enable
pub const FLASH_FMPRE12_READ_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPRE13 register.
//
//*****************************************************************************
pub const FLASH_FMPRE13_READ_ENABLE_M = usize(0xFFFFFFFF);  // Flash Read Enable
pub const FLASH_FMPRE13_READ_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPRE14 register.
//
//*****************************************************************************
pub const FLASH_FMPRE14_READ_ENABLE_M = usize(0xFFFFFFFF);  // Flash Read Enable
pub const FLASH_FMPRE14_READ_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPRE15 register.
//
//*****************************************************************************
pub const FLASH_FMPRE15_READ_ENABLE_M = usize(0xFFFFFFFF);  // Flash Read Enable
pub const FLASH_FMPRE15_READ_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPPE8 register.
//
//*****************************************************************************
pub const FLASH_FMPPE8_PROG_ENABLE_M = usize(0xFFFFFFFF);  // Flash Programming Enable
pub const FLASH_FMPPE8_PROG_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPPE9 register.
//
//*****************************************************************************
pub const FLASH_FMPPE9_PROG_ENABLE_M = usize(0xFFFFFFFF);  // Flash Programming Enable
pub const FLASH_FMPPE9_PROG_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPPE10 register.
//
//*****************************************************************************
pub const FLASH_FMPPE10_PROG_ENABLE_M = usize(0xFFFFFFFF);  // Flash Programming Enable
pub const FLASH_FMPPE10_PROG_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPPE11 register.
//
//*****************************************************************************
pub const FLASH_FMPPE11_PROG_ENABLE_M = usize(0xFFFFFFFF);  // Flash Programming Enable
pub const FLASH_FMPPE11_PROG_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPPE12 register.
//
//*****************************************************************************
pub const FLASH_FMPPE12_PROG_ENABLE_M = usize(0xFFFFFFFF);  // Flash Programming Enable
pub const FLASH_FMPPE12_PROG_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPPE13 register.
//
//*****************************************************************************
pub const FLASH_FMPPE13_PROG_ENABLE_M = usize(0xFFFFFFFF);  // Flash Programming Enable
pub const FLASH_FMPPE13_PROG_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPPE14 register.
//
//*****************************************************************************
pub const FLASH_FMPPE14_PROG_ENABLE_M = usize(0xFFFFFFFF);  // Flash Programming Enable
pub const FLASH_FMPPE14_PROG_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMPPE15 register.
//
//*****************************************************************************
pub const FLASH_FMPPE15_PROG_ENABLE_M = usize(0xFFFFFFFF);  // Flash Programming Enable
pub const FLASH_FMPPE15_PROG_ENABLE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the erase size of the FLASH block that is
// erased by an erase operation, and the protect size is the size of the FLASH
// block that is protected by each protection register.
//
//*****************************************************************************
pub const FLASH_PROTECT_SIZE = usize(0x00000800);
pub const FLASH_ERASE_SIZE = usize(0x00000400);

