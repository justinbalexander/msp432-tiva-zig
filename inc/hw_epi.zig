//*****************************************************************************
//
// hw_epi.h - Macros for use in accessing the EPI registers.
//
// Copyright (c) 2008-2017 Texas Instruments Incorporated.  All rights reserved.
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
// The following are defines for the External Peripheral Interface register
// offsets.
//
//*****************************************************************************
pub const EPI_O_CFG = usize(0x00000000);  // EPI Configuration
pub const EPI_O_BAUD = usize(0x00000004);  // EPI Main Baud Rate
pub const EPI_O_BAUD2 = usize(0x00000008);  // EPI Main Baud Rate
pub const EPI_O_HB16CFG = usize(0x00000010);  // EPI Host-Bus 16 Configuration
pub const EPI_O_GPCFG = usize(0x00000010);  // EPI General-Purpose
                                            // Configuration
pub const EPI_O_SDRAMCFG = usize(0x00000010);  // EPI SDRAM Configuration
pub const EPI_O_HB8CFG = usize(0x00000010);  // EPI Host-Bus 8 Configuration
pub const EPI_O_HB8CFG2 = usize(0x00000014);  // EPI Host-Bus 8 Configuration 2
pub const EPI_O_HB16CFG2 = usize(0x00000014);  // EPI Host-Bus 16 Configuration 2
pub const EPI_O_ADDRMAP = usize(0x0000001C);  // EPI Address Map
pub const EPI_O_RSIZE0 = usize(0x00000020);  // EPI Read Size 0
pub const EPI_O_RADDR0 = usize(0x00000024);  // EPI Read Address 0
pub const EPI_O_RPSTD0 = usize(0x00000028);  // EPI Non-Blocking Read Data 0
pub const EPI_O_RSIZE1 = usize(0x00000030);  // EPI Read Size 1
pub const EPI_O_RADDR1 = usize(0x00000034);  // EPI Read Address 1
pub const EPI_O_RPSTD1 = usize(0x00000038);  // EPI Non-Blocking Read Data 1
pub const EPI_O_STAT = usize(0x00000060);  // EPI Status
pub const EPI_O_RFIFOCNT = usize(0x0000006C);  // EPI Read FIFO Count
pub const EPI_O_READFIFO0 = usize(0x00000070);  // EPI Read FIFO
pub const EPI_O_READFIFO1 = usize(0x00000074);  // EPI Read FIFO Alias 1
pub const EPI_O_READFIFO2 = usize(0x00000078);  // EPI Read FIFO Alias 2
pub const EPI_O_READFIFO3 = usize(0x0000007C);  // EPI Read FIFO Alias 3
pub const EPI_O_READFIFO4 = usize(0x00000080);  // EPI Read FIFO Alias 4
pub const EPI_O_READFIFO5 = usize(0x00000084);  // EPI Read FIFO Alias 5
pub const EPI_O_READFIFO6 = usize(0x00000088);  // EPI Read FIFO Alias 6
pub const EPI_O_READFIFO7 = usize(0x0000008C);  // EPI Read FIFO Alias 7
pub const EPI_O_FIFOLVL = usize(0x00000200);  // EPI FIFO Level Selects
pub const EPI_O_WFIFOCNT = usize(0x00000204);  // EPI Write FIFO Count
pub const EPI_O_DMATXCNT = usize(0x00000208);  // EPI DMA Transmit Count
pub const EPI_O_IM = usize(0x00000210);  // EPI Interrupt Mask
pub const EPI_O_RIS = usize(0x00000214);  // EPI Raw Interrupt Status
pub const EPI_O_MIS = usize(0x00000218);  // EPI Masked Interrupt Status
pub const EPI_O_EISC = usize(0x0000021C);  // EPI Error and Interrupt Status
                                            // and Clear
pub const EPI_O_HB8CFG3 = usize(0x00000308);  // EPI Host-Bus 8 Configuration 3
pub const EPI_O_HB16CFG3 = usize(0x00000308);  // EPI Host-Bus 16 Configuration 3
pub const EPI_O_HB16CFG4 = usize(0x0000030C);  // EPI Host-Bus 16 Configuration 4
pub const EPI_O_HB8CFG4 = usize(0x0000030C);  // EPI Host-Bus 8 Configuration 4
pub const EPI_O_HB8TIME = usize(0x00000310);  // EPI Host-Bus 8 Timing Extension
pub const EPI_O_HB16TIME = usize(0x00000310);  // EPI Host-Bus 16 Timing Extension
pub const EPI_O_HB8TIME2 = usize(0x00000314);  // EPI Host-Bus 8 Timing Extension
pub const EPI_O_HB16TIME2 = usize(0x00000314);  // EPI Host-Bus 16 Timing Extension
pub const EPI_O_HB16TIME3 = usize(0x00000318);  // EPI Host-Bus 16 Timing Extension
pub const EPI_O_HB8TIME3 = usize(0x00000318);  // EPI Host-Bus 8 Timing Extension
pub const EPI_O_HB8TIME4 = usize(0x0000031C);  // EPI Host-Bus 8 Timing Extension
pub const EPI_O_HB16TIME4 = usize(0x0000031C);  // EPI Host-Bus 16 Timing Extension
pub const EPI_O_HBPSRAM = usize(0x00000360);  // EPI Host-Bus PSRAM

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_CFG register.
//
//*****************************************************************************
pub const EPI_CFG_INTDIV = usize(0x00000100);  // Integer Clock Divider Enable
pub const EPI_CFG_BLKEN = usize(0x00000010);  // Block Enable
pub const EPI_CFG_MODE_M = usize(0x0000000F);  // Mode Select
pub const EPI_CFG_MODE_NONE = usize(0x00000000);  // General Purpose
pub const EPI_CFG_MODE_SDRAM = usize(0x00000001);  // SDRAM
pub const EPI_CFG_MODE_HB8 = usize(0x00000002);  // 8-Bit Host-Bus (HB8)
pub const EPI_CFG_MODE_HB16 = usize(0x00000003);  // 16-Bit Host-Bus (HB16)

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_BAUD register.
//
//*****************************************************************************
pub const EPI_BAUD_COUNT1_M = usize(0xFFFF0000);  // Baud Rate Counter 1
pub const EPI_BAUD_COUNT0_M = usize(0x0000FFFF);  // Baud Rate Counter 0
pub const EPI_BAUD_COUNT1_S = usize(16);
pub const EPI_BAUD_COUNT0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_BAUD2 register.
//
//*****************************************************************************
pub const EPI_BAUD2_COUNT1_M = usize(0xFFFF0000);  // CS3n Baud Rate Counter 1
pub const EPI_BAUD2_COUNT0_M = usize(0x0000FFFF);  // CS2n Baud Rate Counter 0
pub const EPI_BAUD2_COUNT1_S = usize(16);
pub const EPI_BAUD2_COUNT0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB16CFG register.
//
//*****************************************************************************
pub const EPI_HB16CFG_CLKGATE = usize(0x80000000);  // Clock Gated
pub const EPI_HB16CFG_CLKGATEI = usize(0x40000000);  // Clock Gated Idle
pub const EPI_HB16CFG_CLKINV = usize(0x20000000);  // Invert Output Clock Enable
pub const EPI_HB16CFG_RDYEN = usize(0x10000000);  // Input Ready Enable
pub const EPI_HB16CFG_IRDYINV = usize(0x08000000);  // Input Ready Invert
pub const EPI_HB16CFG_XFFEN = usize(0x00800000);  // External FIFO FULL Enable
pub const EPI_HB16CFG_XFEEN = usize(0x00400000);  // External FIFO EMPTY Enable
pub const EPI_HB16CFG_WRHIGH = usize(0x00200000);  // WRITE Strobe Polarity
pub const EPI_HB16CFG_RDHIGH = usize(0x00100000);  // READ Strobe Polarity
pub const EPI_HB16CFG_ALEHIGH = usize(0x00080000);  // ALE Strobe Polarity
pub const EPI_HB16CFG_WRCRE = usize(0x00040000);  // PSRAM Configuration Register
                                            // Write
pub const EPI_HB16CFG_RDCRE = usize(0x00020000);  // PSRAM Configuration Register
                                            // Read
pub const EPI_HB16CFG_BURST = usize(0x00010000);  // Burst Mode
pub const EPI_HB16CFG_MAXWAIT_M = usize(0x0000FF00);  // Maximum Wait
pub const EPI_HB16CFG_WRWS_M = usize(0x000000C0);  // Write Wait States
pub const EPI_HB16CFG_WRWS_2 = usize(0x00000000);  // Active WRn is 2 EPI clocks
pub const EPI_HB16CFG_WRWS_4 = usize(0x00000040);  // Active WRn is 4 EPI clocks
pub const EPI_HB16CFG_WRWS_6 = usize(0x00000080);  // Active WRn is 6 EPI clocks
pub const EPI_HB16CFG_WRWS_8 = usize(0x000000C0);  // Active WRn is 8 EPI clocks
pub const EPI_HB16CFG_RDWS_M = usize(0x00000030);  // Read Wait States
pub const EPI_HB16CFG_RDWS_2 = usize(0x00000000);  // Active RDn is 2 EPI clocks
pub const EPI_HB16CFG_RDWS_4 = usize(0x00000010);  // Active RDn is 4 EPI clocks
pub const EPI_HB16CFG_RDWS_6 = usize(0x00000020);  // Active RDn is 6 EPI clocks
pub const EPI_HB16CFG_RDWS_8 = usize(0x00000030);  // Active RDn is 8 EPI clocks
pub const EPI_HB16CFG_BSEL = usize(0x00000004);  // Byte Select Configuration
pub const EPI_HB16CFG_MODE_M = usize(0x00000003);  // Host Bus Sub-Mode
pub const EPI_HB16CFG_MODE_ADMUX = usize(0x00000000);  // ADMUX - AD[15:0]
pub const EPI_HB16CFG_MODE_ADNMUX = usize(0x00000001);  // ADNONMUX - D[15:0]
pub const EPI_HB16CFG_MODE_SRAM = usize(0x00000002);  // Continuous Read - D[15:0]
pub const EPI_HB16CFG_MODE_XFIFO = usize(0x00000003);  // XFIFO - D[15:0]
pub const EPI_HB16CFG_MAXWAIT_S = usize(8);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_GPCFG register.
//
//*****************************************************************************
pub const EPI_GPCFG_CLKPIN = usize(0x80000000);  // Clock Pin
pub const EPI_GPCFG_CLKGATE = usize(0x40000000);  // Clock Gated
pub const EPI_GPCFG_FRM50 = usize(0x04000000);  // 50/50 Frame
pub const EPI_GPCFG_FRMCNT_M = usize(0x03C00000);  // Frame Count
pub const EPI_GPCFG_WR2CYC = usize(0x00080000);  // 2-Cycle Writes
pub const EPI_GPCFG_ASIZE_M = usize(0x00000030);  // Address Bus Size
pub const EPI_GPCFG_ASIZE_NONE = usize(0x00000000);  // No address
pub const EPI_GPCFG_ASIZE_4BIT = usize(0x00000010);  // Up to 4 bits wide
pub const EPI_GPCFG_ASIZE_12BIT = usize(0x00000020);  // Up to 12 bits wide. This size
                                            // cannot be used with 24-bit data
pub const EPI_GPCFG_ASIZE_20BIT = usize(0x00000030);  // Up to 20 bits wide. This size
                                            // cannot be used with data sizes
                                            // other than 8
pub const EPI_GPCFG_DSIZE_M = usize(0x00000003);  // Size of Data Bus
pub const EPI_GPCFG_DSIZE_4BIT = usize(0x00000000);  // 8 Bits Wide (EPI0S0 to EPI0S7)
pub const EPI_GPCFG_DSIZE_16BIT = usize(0x00000001);  // 16 Bits Wide (EPI0S0 to EPI0S15)
pub const EPI_GPCFG_DSIZE_24BIT = usize(0x00000002);  // 24 Bits Wide (EPI0S0 to EPI0S23)
pub const EPI_GPCFG_DSIZE_32BIT = usize(0x00000003);  // 32 Bits Wide (EPI0S0 to EPI0S31)
pub const EPI_GPCFG_FRMCNT_S = usize(22);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_SDRAMCFG register.
//
//*****************************************************************************
pub const EPI_SDRAMCFG_FREQ_M = usize(0xC0000000);  // EPI Frequency Range
pub const EPI_SDRAMCFG_FREQ_NONE = usize(0x00000000);  // 0 - 15 MHz
pub const EPI_SDRAMCFG_FREQ_15MHZ = usize(0x40000000);  // 15 - 30 MHz
pub const EPI_SDRAMCFG_FREQ_30MHZ = usize(0x80000000);  // 30 - 50 MHz
pub const EPI_SDRAMCFG_RFSH_M = usize(0x07FF0000);  // Refresh Counter
pub const EPI_SDRAMCFG_SLEEP = usize(0x00000200);  // Sleep Mode
pub const EPI_SDRAMCFG_SIZE_M = usize(0x00000003);  // Size of SDRAM
pub const EPI_SDRAMCFG_SIZE_8MB = usize(0x00000000);  // 64 megabits (8MB)
pub const EPI_SDRAMCFG_SIZE_16MB = usize(0x00000001);  // 128 megabits (16MB)
pub const EPI_SDRAMCFG_SIZE_32MB = usize(0x00000002);  // 256 megabits (32MB)
pub const EPI_SDRAMCFG_SIZE_64MB = usize(0x00000003);  // 512 megabits (64MB)
pub const EPI_SDRAMCFG_RFSH_S = usize(16);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB8CFG register.
//
//*****************************************************************************
pub const EPI_HB8CFG_CLKGATE = usize(0x80000000);  // Clock Gated
pub const EPI_HB8CFG_CLKGATEI = usize(0x40000000);  // Clock Gated when Idle
pub const EPI_HB8CFG_CLKINV = usize(0x20000000);  // Invert Output Clock Enable
pub const EPI_HB8CFG_RDYEN = usize(0x10000000);  // Input Ready Enable
pub const EPI_HB8CFG_IRDYINV = usize(0x08000000);  // Input Ready Invert
pub const EPI_HB8CFG_XFFEN = usize(0x00800000);  // External FIFO FULL Enable
pub const EPI_HB8CFG_XFEEN = usize(0x00400000);  // External FIFO EMPTY Enable
pub const EPI_HB8CFG_WRHIGH = usize(0x00200000);  // WRITE Strobe Polarity
pub const EPI_HB8CFG_RDHIGH = usize(0x00100000);  // READ Strobe Polarity
pub const EPI_HB8CFG_ALEHIGH = usize(0x00080000);  // ALE Strobe Polarity
pub const EPI_HB8CFG_MAXWAIT_M = usize(0x0000FF00);  // Maximum Wait
pub const EPI_HB8CFG_WRWS_M = usize(0x000000C0);  // Write Wait States
pub const EPI_HB8CFG_WRWS_2 = usize(0x00000000);  // Active WRn is 2 EPI clocks
pub const EPI_HB8CFG_WRWS_4 = usize(0x00000040);  // Active WRn is 4 EPI clocks
pub const EPI_HB8CFG_WRWS_6 = usize(0x00000080);  // Active WRn is 6 EPI clocks
pub const EPI_HB8CFG_WRWS_8 = usize(0x000000C0);  // Active WRn is 8 EPI clocks
pub const EPI_HB8CFG_RDWS_M = usize(0x00000030);  // Read Wait States
pub const EPI_HB8CFG_RDWS_2 = usize(0x00000000);  // Active RDn is 2 EPI clocks
pub const EPI_HB8CFG_RDWS_4 = usize(0x00000010);  // Active RDn is 4 EPI clocks
pub const EPI_HB8CFG_RDWS_6 = usize(0x00000020);  // Active RDn is 6 EPI clocks
pub const EPI_HB8CFG_RDWS_8 = usize(0x00000030);  // Active RDn is 8 EPI clocks
pub const EPI_HB8CFG_MODE_M = usize(0x00000003);  // Host Bus Sub-Mode
pub const EPI_HB8CFG_MODE_MUX = usize(0x00000000);  // ADMUX - AD[7:0]
pub const EPI_HB8CFG_MODE_NMUX = usize(0x00000001);  // ADNONMUX - D[7:0]
pub const EPI_HB8CFG_MODE_SRAM = usize(0x00000002);  // Continuous Read - D[7:0]
pub const EPI_HB8CFG_MODE_FIFO = usize(0x00000003);  // XFIFO - D[7:0]
pub const EPI_HB8CFG_MAXWAIT_S = usize(8);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB8CFG2 register.
//
//*****************************************************************************
pub const EPI_HB8CFG2_CSCFGEXT = usize(0x08000000);  // Chip Select Extended
                                            // Configuration
pub const EPI_HB8CFG2_CSBAUD = usize(0x04000000);  // Chip Select Baud Rate and
                                            // Multiple Sub-Mode Configuration
                                            // enable
pub const EPI_HB8CFG2_CSCFG_M = usize(0x03000000);  // Chip Select Configuration
pub const EPI_HB8CFG2_CSCFG_ALE = usize(0x00000000);  // ALE Configuration
pub const EPI_HB8CFG2_CSCFG_CS = usize(0x01000000);  // CSn Configuration
pub const EPI_HB8CFG2_CSCFG_DCS = usize(0x02000000);  // Dual CSn Configuration
pub const EPI_HB8CFG2_CSCFG_ADCS = usize(0x03000000);  // ALE with Dual CSn Configuration
pub const EPI_HB8CFG2_WRHIGH = usize(0x00200000);  // CS1n WRITE Strobe Polarity
pub const EPI_HB8CFG2_RDHIGH = usize(0x00100000);  // CS1n READ Strobe Polarity
pub const EPI_HB8CFG2_ALEHIGH = usize(0x00080000);  // CS1n ALE Strobe Polarity
pub const EPI_HB8CFG2_WRWS_M = usize(0x000000C0);  // CS1n Write Wait States
pub const EPI_HB8CFG2_WRWS_2 = usize(0x00000000);  // Active WRn is 2 EPI clocks
pub const EPI_HB8CFG2_WRWS_4 = usize(0x00000040);  // Active WRn is 4 EPI clocks
pub const EPI_HB8CFG2_WRWS_6 = usize(0x00000080);  // Active WRn is 6 EPI clocks
pub const EPI_HB8CFG2_WRWS_8 = usize(0x000000C0);  // Active WRn is 8 EPI clocks
pub const EPI_HB8CFG2_RDWS_M = usize(0x00000030);  // CS1n Read Wait States
pub const EPI_HB8CFG2_RDWS_2 = usize(0x00000000);  // Active RDn is 2 EPI clocks
pub const EPI_HB8CFG2_RDWS_4 = usize(0x00000010);  // Active RDn is 4 EPI clocks
pub const EPI_HB8CFG2_RDWS_6 = usize(0x00000020);  // Active RDn is 6 EPI clocks
pub const EPI_HB8CFG2_RDWS_8 = usize(0x00000030);  // Active RDn is 8 EPI clocks
pub const EPI_HB8CFG2_MODE_M = usize(0x00000003);  // CS1n Host Bus Sub-Mode
pub const EPI_HB8CFG2_MODE_ADMUX = usize(0x00000000);  // ADMUX - AD[7:0]
pub const EPI_HB8CFG2_MODE_AD = usize(0x00000001);  // ADNONMUX - D[7:0]

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB16CFG2 register.
//
//*****************************************************************************
pub const EPI_HB16CFG2_CSCFGEXT = usize(0x08000000);  // Chip Select Extended
                                            // Configuration
pub const EPI_HB16CFG2_CSBAUD = usize(0x04000000);  // Chip Select Baud Rate and
                                            // Multiple Sub-Mode Configuration
                                            // enable
pub const EPI_HB16CFG2_CSCFG_M = usize(0x03000000);  // Chip Select Configuration
pub const EPI_HB16CFG2_CSCFG_ALE = usize(0x00000000);  // ALE Configuration
pub const EPI_HB16CFG2_CSCFG_CS = usize(0x01000000);  // CSn Configuration
pub const EPI_HB16CFG2_CSCFG_DCS = usize(0x02000000);  // Dual CSn Configuration
pub const EPI_HB16CFG2_CSCFG_ADCS = usize(0x03000000);  // ALE with Dual CSn Configuration
pub const EPI_HB16CFG2_WRHIGH = usize(0x00200000);  // CS1n WRITE Strobe Polarity
pub const EPI_HB16CFG2_RDHIGH = usize(0x00100000);  // CS1n READ Strobe Polarity
pub const EPI_HB16CFG2_ALEHIGH = usize(0x00080000);  // CS1n ALE Strobe Polarity
pub const EPI_HB16CFG2_WRCRE = usize(0x00040000);  // CS1n PSRAM Configuration
                                            // Register Write
pub const EPI_HB16CFG2_RDCRE = usize(0x00020000);  // CS1n PSRAM Configuration
                                            // Register Read
pub const EPI_HB16CFG2_BURST = usize(0x00010000);  // CS1n Burst Mode
pub const EPI_HB16CFG2_WRWS_M = usize(0x000000C0);  // CS1n Write Wait States
pub const EPI_HB16CFG2_WRWS_2 = usize(0x00000000);  // Active WRn is 2 EPI clocks
pub const EPI_HB16CFG2_WRWS_4 = usize(0x00000040);  // Active WRn is 4 EPI clocks
pub const EPI_HB16CFG2_WRWS_6 = usize(0x00000080);  // Active WRn is 6 EPI clocks
pub const EPI_HB16CFG2_WRWS_8 = usize(0x000000C0);  // Active WRn is 8 EPI clocks
pub const EPI_HB16CFG2_RDWS_M = usize(0x00000030);  // CS1n Read Wait States
pub const EPI_HB16CFG2_RDWS_2 = usize(0x00000000);  // Active RDn is 2 EPI clocks
pub const EPI_HB16CFG2_RDWS_4 = usize(0x00000010);  // Active RDn is 4 EPI clocks
pub const EPI_HB16CFG2_RDWS_6 = usize(0x00000020);  // Active RDn is 6 EPI clocks
pub const EPI_HB16CFG2_RDWS_8 = usize(0x00000030);  // Active RDn is 8 EPI clocks
pub const EPI_HB16CFG2_MODE_M = usize(0x00000003);  // CS1n Host Bus Sub-Mode
pub const EPI_HB16CFG2_MODE_ADMUX = usize(0x00000000);  // ADMUX - AD[15:0]
pub const EPI_HB16CFG2_MODE_AD = usize(0x00000001);  // ADNONMUX - D[15:0]

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_ADDRMAP register.
//
//*****************************************************************************
pub const EPI_ADDRMAP_ECSZ_M = usize(0x00000C00);  // External Code Size
pub const EPI_ADDRMAP_ECSZ_256B = usize(0x00000000);  // 256 bytes; lower address range:
                                            // 0x00 to 0xFF
pub const EPI_ADDRMAP_ECSZ_64KB = usize(0x00000400);  // 64 KB; lower address range:
                                            // 0x0000 to 0xFFFF
pub const EPI_ADDRMAP_ECSZ_16MB = usize(0x00000800);  // 16 MB; lower address range:
                                            // 0x00.0000 to 0xFF.FFFF
pub const EPI_ADDRMAP_ECSZ_256MB = usize(0x00000C00);  // 256MB; lower address range:
                                            // 0x000.0000 to 0x0FFF.FFFF
pub const EPI_ADDRMAP_ECADR_M = usize(0x00000300);  // External Code Address
pub const EPI_ADDRMAP_ECADR_NONE = usize(0x00000000);  // Not mapped
pub const EPI_ADDRMAP_ECADR_1000 = usize(0x00000100);  // At 0x1000.0000
pub const EPI_ADDRMAP_EPSZ_M = usize(0x000000C0);  // External Peripheral Size
pub const EPI_ADDRMAP_EPSZ_256B = usize(0x00000000);  // 256 bytes; lower address range:
                                            // 0x00 to 0xFF
pub const EPI_ADDRMAP_EPSZ_64KB = usize(0x00000040);  // 64 KB; lower address range:
                                            // 0x0000 to 0xFFFF
pub const EPI_ADDRMAP_EPSZ_16MB = usize(0x00000080);  // 16 MB; lower address range:
                                            // 0x00.0000 to 0xFF.FFFF
pub const EPI_ADDRMAP_EPSZ_256MB = usize(0x000000C0);  // 256 MB; lower address range:
                                            // 0x000.0000 to 0xFFF.FFFF
pub const EPI_ADDRMAP_EPADR_M = usize(0x00000030);  // External Peripheral Address
pub const EPI_ADDRMAP_EPADR_NONE = usize(0x00000000);  // Not mapped
pub const EPI_ADDRMAP_EPADR_A000 = usize(0x00000010);  // At 0xA000.0000
pub const EPI_ADDRMAP_EPADR_C000 = usize(0x00000020);  // At 0xC000.0000
pub const EPI_ADDRMAP_EPADR_HBQS = usize(0x00000030);  // Only to be used with Host Bus
                                            // quad chip select. In quad chip
                                            // select mode, CS2n maps to
                                            // 0xA000.0000 and CS3n maps to
                                            // 0xC000.0000
pub const EPI_ADDRMAP_ERSZ_M = usize(0x0000000C);  // External RAM Size
pub const EPI_ADDRMAP_ERSZ_256B = usize(0x00000000);  // 256 bytes; lower address range:
                                            // 0x00 to 0xFF
pub const EPI_ADDRMAP_ERSZ_64KB = usize(0x00000004);  // 64 KB; lower address range:
                                            // 0x0000 to 0xFFFF
pub const EPI_ADDRMAP_ERSZ_16MB = usize(0x00000008);  // 16 MB; lower address range:
                                            // 0x00.0000 to 0xFF.FFFF
pub const EPI_ADDRMAP_ERSZ_256MB = usize(0x0000000C);  // 256 MB; lower address range:
                                            // 0x000.0000 to 0xFFF.FFFF
pub const EPI_ADDRMAP_ERADR_M = usize(0x00000003);  // External RAM Address
pub const EPI_ADDRMAP_ERADR_NONE = usize(0x00000000);  // Not mapped
pub const EPI_ADDRMAP_ERADR_6000 = usize(0x00000001);  // At 0x6000.0000
pub const EPI_ADDRMAP_ERADR_8000 = usize(0x00000002);  // At 0x8000.0000
pub const EPI_ADDRMAP_ERADR_HBQS = usize(0x00000003);  // Only to be used with Host Bus
                                            // quad chip select. In quad chip
                                            // select mode, CS0n maps to
                                            // 0x6000.0000 and CS1n maps to
                                            // 0x8000.0000

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_RSIZE0 register.
//
//*****************************************************************************
pub const EPI_RSIZE0_SIZE_M = usize(0x00000003);  // Current Size
pub const EPI_RSIZE0_SIZE_8BIT = usize(0x00000001);  // Byte (8 bits)
pub const EPI_RSIZE0_SIZE_16BIT = usize(0x00000002);  // Half-word (16 bits)
pub const EPI_RSIZE0_SIZE_32BIT = usize(0x00000003);  // Word (32 bits)

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_RADDR0 register.
//
//*****************************************************************************
pub const EPI_RADDR0_ADDR_M = usize(0xFFFFFFFF);  // Current Address
pub const EPI_RADDR0_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_RPSTD0 register.
//
//*****************************************************************************
pub const EPI_RPSTD0_POSTCNT_M = usize(0x00001FFF);  // Post Count
pub const EPI_RPSTD0_POSTCNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_RSIZE1 register.
//
//*****************************************************************************
pub const EPI_RSIZE1_SIZE_M = usize(0x00000003);  // Current Size
pub const EPI_RSIZE1_SIZE_8BIT = usize(0x00000001);  // Byte (8 bits)
pub const EPI_RSIZE1_SIZE_16BIT = usize(0x00000002);  // Half-word (16 bits)
pub const EPI_RSIZE1_SIZE_32BIT = usize(0x00000003);  // Word (32 bits)

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_RADDR1 register.
//
//*****************************************************************************
pub const EPI_RADDR1_ADDR_M = usize(0xFFFFFFFF);  // Current Address
pub const EPI_RADDR1_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_RPSTD1 register.
//
//*****************************************************************************
pub const EPI_RPSTD1_POSTCNT_M = usize(0x00001FFF);  // Post Count
pub const EPI_RPSTD1_POSTCNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_STAT register.
//
//*****************************************************************************
pub const EPI_STAT_XFFULL = usize(0x00000100);  // External FIFO Full
pub const EPI_STAT_XFEMPTY = usize(0x00000080);  // External FIFO Empty
pub const EPI_STAT_INITSEQ = usize(0x00000040);  // Initialization Sequence
pub const EPI_STAT_WBUSY = usize(0x00000020);  // Write Busy
pub const EPI_STAT_NBRBUSY = usize(0x00000010);  // Non-Blocking Read Busy
pub const EPI_STAT_ACTIVE = usize(0x00000001);  // Register Active

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_RFIFOCNT register.
//
//*****************************************************************************
pub const EPI_RFIFOCNT_COUNT_M = usize(0x0000000F);  // FIFO Count
pub const EPI_RFIFOCNT_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_READFIFO0
// register.
//
//*****************************************************************************
pub const EPI_READFIFO0_DATA_M = usize(0xFFFFFFFF);  // Reads Data
pub const EPI_READFIFO0_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_READFIFO1
// register.
//
//*****************************************************************************
pub const EPI_READFIFO1_DATA_M = usize(0xFFFFFFFF);  // Reads Data
pub const EPI_READFIFO1_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_READFIFO2
// register.
//
//*****************************************************************************
pub const EPI_READFIFO2_DATA_M = usize(0xFFFFFFFF);  // Reads Data
pub const EPI_READFIFO2_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_READFIFO3
// register.
//
//*****************************************************************************
pub const EPI_READFIFO3_DATA_M = usize(0xFFFFFFFF);  // Reads Data
pub const EPI_READFIFO3_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_READFIFO4
// register.
//
//*****************************************************************************
pub const EPI_READFIFO4_DATA_M = usize(0xFFFFFFFF);  // Reads Data
pub const EPI_READFIFO4_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_READFIFO5
// register.
//
//*****************************************************************************
pub const EPI_READFIFO5_DATA_M = usize(0xFFFFFFFF);  // Reads Data
pub const EPI_READFIFO5_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_READFIFO6
// register.
//
//*****************************************************************************
pub const EPI_READFIFO6_DATA_M = usize(0xFFFFFFFF);  // Reads Data
pub const EPI_READFIFO6_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_READFIFO7
// register.
//
//*****************************************************************************
pub const EPI_READFIFO7_DATA_M = usize(0xFFFFFFFF);  // Reads Data
pub const EPI_READFIFO7_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_FIFOLVL register.
//
//*****************************************************************************
pub const EPI_FIFOLVL_WFERR = usize(0x00020000);  // Write Full Error
pub const EPI_FIFOLVL_RSERR = usize(0x00010000);  // Read Stall Error
pub const EPI_FIFOLVL_WRFIFO_M = usize(0x00000070);  // Write FIFO
pub const EPI_FIFOLVL_WRFIFO_EMPT = usize(0x00000000);  // Interrupt is triggered while
                                            // WRFIFO is empty.
pub const EPI_FIFOLVL_WRFIFO_2 = usize(0x00000020);  // Interrupt is triggered until
                                            // there are only two slots
                                            // available. Thus, trigger is
                                            // deasserted when there are two
                                            // WRFIFO entries present. This
                                            // configuration is optimized for
                                            // bursts of 2
pub const EPI_FIFOLVL_WRFIFO_1 = usize(0x00000030);  // Interrupt is triggered until
                                            // there is one WRFIFO entry
                                            // available. This configuration
                                            // expects only single writes
pub const EPI_FIFOLVL_WRFIFO_NFULL = usize(0x00000040);  // Trigger interrupt when WRFIFO is
                                            // not full, meaning trigger will
                                            // continue to assert until there
                                            // are four entries in the WRFIFO
pub const EPI_FIFOLVL_RDFIFO_M = usize(0x00000007);  // Read FIFO
pub const EPI_FIFOLVL_RDFIFO_EMPT = usize(0x00000000);  // Empty
pub const EPI_FIFOLVL_RDFIFO_1 = usize(0x00000001);  // Trigger when there are 1 or more
                                            // entries in the NBRFIFO
pub const EPI_FIFOLVL_RDFIFO_2 = usize(0x00000002);  // Trigger when there are 2 or more
                                            // entries in the NBRFIFO
pub const EPI_FIFOLVL_RDFIFO_4 = usize(0x00000003);  // Trigger when there are 4 or more
                                            // entries in the NBRFIFO
pub const EPI_FIFOLVL_RDFIFO_6 = usize(0x00000004);  // Trigger when there are 6 or more
                                            // entries in the NBRFIFO
pub const EPI_FIFOLVL_RDFIFO_7 = usize(0x00000005);  // Trigger when there are 7 or more
                                            // entries in the NBRFIFO
pub const EPI_FIFOLVL_RDFIFO_8 = usize(0x00000006);  // Trigger when there are 8 entries
                                            // in the NBRFIFO

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_WFIFOCNT register.
//
//*****************************************************************************
pub const EPI_WFIFOCNT_WTAV_M = usize(0x00000007);  // Available Write Transactions
pub const EPI_WFIFOCNT_WTAV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_DMATXCNT register.
//
//*****************************************************************************
pub const EPI_DMATXCNT_TXCNT_M = usize(0x0000FFFF);  // DMA Count
pub const EPI_DMATXCNT_TXCNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_IM register.
//
//*****************************************************************************
pub const EPI_IM_DMAWRIM = usize(0x00000010);  // Write uDMA Interrupt Mask
pub const EPI_IM_DMARDIM = usize(0x00000008);  // Read uDMA Interrupt Mask
pub const EPI_IM_WRIM = usize(0x00000004);  // Write FIFO Empty Interrupt Mask
pub const EPI_IM_RDIM = usize(0x00000002);  // Read FIFO Full Interrupt Mask
pub const EPI_IM_ERRIM = usize(0x00000001);  // Error Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_RIS register.
//
//*****************************************************************************
pub const EPI_RIS_DMAWRRIS = usize(0x00000010);  // Write uDMA Raw Interrupt Status
pub const EPI_RIS_DMARDRIS = usize(0x00000008);  // Read uDMA Raw Interrupt Status
pub const EPI_RIS_WRRIS = usize(0x00000004);  // Write Raw Interrupt Status
pub const EPI_RIS_RDRIS = usize(0x00000002);  // Read Raw Interrupt Status
pub const EPI_RIS_ERRRIS = usize(0x00000001);  // Error Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_MIS register.
//
//*****************************************************************************
pub const EPI_MIS_DMAWRMIS = usize(0x00000010);  // Write uDMA Masked Interrupt
                                            // Status
pub const EPI_MIS_DMARDMIS = usize(0x00000008);  // Read uDMA Masked Interrupt
                                            // Status
pub const EPI_MIS_WRMIS = usize(0x00000004);  // Write Masked Interrupt Status
pub const EPI_MIS_RDMIS = usize(0x00000002);  // Read Masked Interrupt Status
pub const EPI_MIS_ERRMIS = usize(0x00000001);  // Error Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_EISC register.
//
//*****************************************************************************
pub const EPI_EISC_DMAWRIC = usize(0x00000010);  // Write uDMA Interrupt Clear
pub const EPI_EISC_DMARDIC = usize(0x00000008);  // Read uDMA Interrupt Clear
pub const EPI_EISC_WTFULL = usize(0x00000004);  // Write FIFO Full Error
pub const EPI_EISC_RSTALL = usize(0x00000002);  // Read Stalled Error
pub const EPI_EISC_TOUT = usize(0x00000001);  // Timeout Error

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB8CFG3 register.
//
//*****************************************************************************
pub const EPI_HB8CFG3_WRHIGH = usize(0x00200000);  // CS2n WRITE Strobe Polarity
pub const EPI_HB8CFG3_RDHIGH = usize(0x00100000);  // CS2n READ Strobe Polarity
pub const EPI_HB8CFG3_ALEHIGH = usize(0x00080000);  // CS2n ALE Strobe Polarity
pub const EPI_HB8CFG3_WRWS_M = usize(0x000000C0);  // CS2n Write Wait States
pub const EPI_HB8CFG3_WRWS_2 = usize(0x00000000);  // Active WRn is 2 EPI clocks
pub const EPI_HB8CFG3_WRWS_4 = usize(0x00000040);  // Active WRn is 4 EPI clocks
pub const EPI_HB8CFG3_WRWS_6 = usize(0x00000080);  // Active WRn is 6 EPI clocks
pub const EPI_HB8CFG3_WRWS_8 = usize(0x000000C0);  // Active WRn is 8 EPI clocks
pub const EPI_HB8CFG3_RDWS_M = usize(0x00000030);  // CS2n Read Wait States
pub const EPI_HB8CFG3_RDWS_2 = usize(0x00000000);  // Active RDn is 2 EPI clocks
pub const EPI_HB8CFG3_RDWS_4 = usize(0x00000010);  // Active RDn is 4 EPI clocks
pub const EPI_HB8CFG3_RDWS_6 = usize(0x00000020);  // Active RDn is 6 EPI clocks
pub const EPI_HB8CFG3_RDWS_8 = usize(0x00000030);  // Active RDn is 8 EPI clocks
pub const EPI_HB8CFG3_MODE_M = usize(0x00000003);  // CS2n Host Bus Sub-Mode
pub const EPI_HB8CFG3_MODE_ADMUX = usize(0x00000000);  // ADMUX - AD[7:0]
pub const EPI_HB8CFG3_MODE_AD = usize(0x00000001);  // ADNONMUX - D[7:0]

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB16CFG3 register.
//
//*****************************************************************************
pub const EPI_HB16CFG3_WRHIGH = usize(0x00200000);  // CS2n WRITE Strobe Polarity
pub const EPI_HB16CFG3_RDHIGH = usize(0x00100000);  // CS2n READ Strobe Polarity
pub const EPI_HB16CFG3_ALEHIGH = usize(0x00080000);  // CS2n ALE Strobe Polarity
pub const EPI_HB16CFG3_WRCRE = usize(0x00040000);  // CS2n PSRAM Configuration
                                            // Register Write
pub const EPI_HB16CFG3_RDCRE = usize(0x00020000);  // CS2n PSRAM Configuration
                                            // Register Read
pub const EPI_HB16CFG3_BURST = usize(0x00010000);  // CS2n Burst Mode
pub const EPI_HB16CFG3_WRWS_M = usize(0x000000C0);  // CS2n Write Wait States
pub const EPI_HB16CFG3_WRWS_2 = usize(0x00000000);  // Active WRn is 2 EPI clocks
pub const EPI_HB16CFG3_WRWS_4 = usize(0x00000040);  // Active WRn is 4 EPI clocks
pub const EPI_HB16CFG3_WRWS_6 = usize(0x00000080);  // Active WRn is 6 EPI clocks
pub const EPI_HB16CFG3_WRWS_8 = usize(0x000000C0);  // Active WRn is 8 EPI clocks
pub const EPI_HB16CFG3_RDWS_M = usize(0x00000030);  // CS2n Read Wait States
pub const EPI_HB16CFG3_RDWS_2 = usize(0x00000000);  // Active RDn is 2 EPI clocks
pub const EPI_HB16CFG3_RDWS_4 = usize(0x00000010);  // Active RDn is 4 EPI clocks
pub const EPI_HB16CFG3_RDWS_6 = usize(0x00000020);  // Active RDn is 6 EPI clocks
pub const EPI_HB16CFG3_RDWS_8 = usize(0x00000030);  // Active RDn is 8 EPI clocks
pub const EPI_HB16CFG3_MODE_M = usize(0x00000003);  // CS2n Host Bus Sub-Mode
pub const EPI_HB16CFG3_MODE_ADMUX = usize(0x00000000);  // ADMUX - AD[15:0]
pub const EPI_HB16CFG3_MODE_AD = usize(0x00000001);  // ADNONMUX - D[15:0]

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB16CFG4 register.
//
//*****************************************************************************
pub const EPI_HB16CFG4_WRHIGH = usize(0x00200000);  // CS3n WRITE Strobe Polarity
pub const EPI_HB16CFG4_RDHIGH = usize(0x00100000);  // CS3n READ Strobe Polarity
pub const EPI_HB16CFG4_ALEHIGH = usize(0x00080000);  // CS3n ALE Strobe Polarity
pub const EPI_HB16CFG4_WRCRE = usize(0x00040000);  // CS3n PSRAM Configuration
                                            // Register Write
pub const EPI_HB16CFG4_RDCRE = usize(0x00020000);  // CS3n PSRAM Configuration
                                            // Register Read
pub const EPI_HB16CFG4_BURST = usize(0x00010000);  // CS3n Burst Mode
pub const EPI_HB16CFG4_WRWS_M = usize(0x000000C0);  // CS3n Write Wait States
pub const EPI_HB16CFG4_WRWS_2 = usize(0x00000000);  // Active WRn is 2 EPI clocks
pub const EPI_HB16CFG4_WRWS_4 = usize(0x00000040);  // Active WRn is 4 EPI clocks
pub const EPI_HB16CFG4_WRWS_6 = usize(0x00000080);  // Active WRn is 6 EPI clocks
pub const EPI_HB16CFG4_WRWS_8 = usize(0x000000C0);  // Active WRn is 8 EPI clocks
pub const EPI_HB16CFG4_RDWS_M = usize(0x00000030);  // CS3n Read Wait States
pub const EPI_HB16CFG4_RDWS_2 = usize(0x00000000);  // Active RDn is 2 EPI clocks
pub const EPI_HB16CFG4_RDWS_4 = usize(0x00000010);  // Active RDn is 4 EPI clocks
pub const EPI_HB16CFG4_RDWS_6 = usize(0x00000020);  // Active RDn is 6 EPI clocks
pub const EPI_HB16CFG4_RDWS_8 = usize(0x00000030);  // Active RDn is 8 EPI clocks
pub const EPI_HB16CFG4_MODE_M = usize(0x00000003);  // CS3n Host Bus Sub-Mode
pub const EPI_HB16CFG4_MODE_ADMUX = usize(0x00000000);  // ADMUX - AD[15:0]
pub const EPI_HB16CFG4_MODE_AD = usize(0x00000001);  // ADNONMUX - D[15:0]

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB8CFG4 register.
//
//*****************************************************************************
pub const EPI_HB8CFG4_WRHIGH = usize(0x00200000);  // CS3n WRITE Strobe Polarity
pub const EPI_HB8CFG4_RDHIGH = usize(0x00100000);  // CS2n READ Strobe Polarity
pub const EPI_HB8CFG4_ALEHIGH = usize(0x00080000);  // CS3n ALE Strobe Polarity
pub const EPI_HB8CFG4_WRWS_M = usize(0x000000C0);  // CS3n Write Wait States
pub const EPI_HB8CFG4_WRWS_2 = usize(0x00000000);  // Active WRn is 2 EPI clocks
pub const EPI_HB8CFG4_WRWS_4 = usize(0x00000040);  // Active WRn is 4 EPI clocks
pub const EPI_HB8CFG4_WRWS_6 = usize(0x00000080);  // Active WRn is 6 EPI clocks
pub const EPI_HB8CFG4_WRWS_8 = usize(0x000000C0);  // Active WRn is 8 EPI clocks
pub const EPI_HB8CFG4_RDWS_M = usize(0x00000030);  // CS3n Read Wait States
pub const EPI_HB8CFG4_RDWS_2 = usize(0x00000000);  // Active RDn is 2 EPI clocks
pub const EPI_HB8CFG4_RDWS_4 = usize(0x00000010);  // Active RDn is 4 EPI clocks
pub const EPI_HB8CFG4_RDWS_6 = usize(0x00000020);  // Active RDn is 6 EPI clocks
pub const EPI_HB8CFG4_RDWS_8 = usize(0x00000030);  // Active RDn is 8 EPI clocks
pub const EPI_HB8CFG4_MODE_M = usize(0x00000003);  // CS3n Host Bus Sub-Mode
pub const EPI_HB8CFG4_MODE_ADMUX = usize(0x00000000);  // ADMUX - AD[7:0]
pub const EPI_HB8CFG4_MODE_AD = usize(0x00000001);  // ADNONMUX - D[7:0]

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB8TIME register.
//
//*****************************************************************************
pub const EPI_HB8TIME_IRDYDLY_M = usize(0x03000000);  // CS0n Input Ready Delay
pub const EPI_HB8TIME_CAPWIDTH_M = usize(0x00003000);  // CS0n Inter-transfer Capture
                                            // Width
pub const EPI_HB8TIME_WRWSM = usize(0x00000010);  // Write Wait State Minus One
pub const EPI_HB8TIME_RDWSM = usize(0x00000001);  // Read Wait State Minus One
pub const EPI_HB8TIME_IRDYDLY_S = usize(24);
pub const EPI_HB8TIME_CAPWIDTH_S = usize(12);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB16TIME register.
//
//*****************************************************************************
pub const EPI_HB16TIME_IRDYDLY_M = usize(0x03000000);  // CS0n Input Ready Delay
pub const EPI_HB16TIME_PSRAMSZ_M = usize(0x00070000);  // PSRAM Row Size
pub const EPI_HB16TIME_PSRAMSZ_0 = usize(0x00000000);  // No row size limitation
pub const EPI_HB16TIME_PSRAMSZ_128B = usize(0x00010000);  // 128 B
pub const EPI_HB16TIME_PSRAMSZ_256B = usize(0x00020000);  // 256 B
pub const EPI_HB16TIME_PSRAMSZ_512B = usize(0x00030000);  // 512 B
pub const EPI_HB16TIME_PSRAMSZ_1KB = usize(0x00040000);  // 1024 B
pub const EPI_HB16TIME_PSRAMSZ_2KB = usize(0x00050000);  // 2048 B
pub const EPI_HB16TIME_PSRAMSZ_4KB = usize(0x00060000);  // 4096 B
pub const EPI_HB16TIME_PSRAMSZ_8KB = usize(0x00070000);  // 8192 B
pub const EPI_HB16TIME_CAPWIDTH_M = usize(0x00003000);  // CS0n Inter-transfer Capture
                                            // Width
pub const EPI_HB16TIME_WRWSM = usize(0x00000010);  // Write Wait State Minus One
pub const EPI_HB16TIME_RDWSM = usize(0x00000001);  // Read Wait State Minus One
pub const EPI_HB16TIME_IRDYDLY_S = usize(24);
pub const EPI_HB16TIME_CAPWIDTH_S = usize(12);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB8TIME2 register.
//
//*****************************************************************************
pub const EPI_HB8TIME2_IRDYDLY_M = usize(0x03000000);  // CS1n Input Ready Delay
pub const EPI_HB8TIME2_CAPWIDTH_M = usize(0x00003000);  // CS1n Inter-transfer Capture
                                            // Width
pub const EPI_HB8TIME2_WRWSM = usize(0x00000010);  // CS1n Write Wait State Minus One
pub const EPI_HB8TIME2_RDWSM = usize(0x00000001);  // CS1n Read Wait State Minus One
pub const EPI_HB8TIME2_IRDYDLY_S = usize(24);
pub const EPI_HB8TIME2_CAPWIDTH_S = usize(12);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB16TIME2
// register.
//
//*****************************************************************************
pub const EPI_HB16TIME2_IRDYDLY_M = usize(0x03000000);  // CS1n Input Ready Delay
pub const EPI_HB16TIME2_PSRAMSZ_M = usize(0x00070000);  // PSRAM Row Size
pub const EPI_HB16TIME2_PSRAMSZ_0 = usize(0x00000000);  // No row size limitation
pub const EPI_HB16TIME2_PSRAMSZ_128B = usize(0x00010000);  // 128 B
pub const EPI_HB16TIME2_PSRAMSZ_256B = usize(0x00020000);  // 256 B
pub const EPI_HB16TIME2_PSRAMSZ_512B = usize(0x00030000);  // 512 B
pub const EPI_HB16TIME2_PSRAMSZ_1KB = usize(0x00040000);  // 1024 B
pub const EPI_HB16TIME2_PSRAMSZ_2KB = usize(0x00050000);  // 2048 B
pub const EPI_HB16TIME2_PSRAMSZ_4KB = usize(0x00060000);  // 4096 B
pub const EPI_HB16TIME2_PSRAMSZ_8KB = usize(0x00070000);  // 8192 B
pub const EPI_HB16TIME2_CAPWIDTH_M = usize(0x00003000);  // CS1n Inter-transfer Capture
                                            // Width
pub const EPI_HB16TIME2_WRWSM = usize(0x00000010);  // CS1n Write Wait State Minus One
pub const EPI_HB16TIME2_RDWSM = usize(0x00000001);  // CS1n Read Wait State Minus One
pub const EPI_HB16TIME2_IRDYDLY_S = usize(24);
pub const EPI_HB16TIME2_CAPWIDTH_S = usize(12);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB16TIME3
// register.
//
//*****************************************************************************
pub const EPI_HB16TIME3_IRDYDLY_M = usize(0x03000000);  // CS2n Input Ready Delay
pub const EPI_HB16TIME3_PSRAMSZ_M = usize(0x00070000);  // PSRAM Row Size
pub const EPI_HB16TIME3_PSRAMSZ_0 = usize(0x00000000);  // No row size limitation
pub const EPI_HB16TIME3_PSRAMSZ_128B = usize(0x00010000);  // 128 B
pub const EPI_HB16TIME3_PSRAMSZ_256B = usize(0x00020000);  // 256 B
pub const EPI_HB16TIME3_PSRAMSZ_512B = usize(0x00030000);  // 512 B
pub const EPI_HB16TIME3_PSRAMSZ_1KB = usize(0x00040000);  // 1024 B
pub const EPI_HB16TIME3_PSRAMSZ_2KB = usize(0x00050000);  // 2048 B
pub const EPI_HB16TIME3_PSRAMSZ_4KB = usize(0x00060000);  // 4096 B
pub const EPI_HB16TIME3_PSRAMSZ_8KB = usize(0x00070000);  // 8192 B
pub const EPI_HB16TIME3_CAPWIDTH_M = usize(0x00003000);  // CS2n Inter-transfer Capture
                                            // Width
pub const EPI_HB16TIME3_WRWSM = usize(0x00000010);  // CS2n Write Wait State Minus One
pub const EPI_HB16TIME3_RDWSM = usize(0x00000001);  // CS2n Read Wait State Minus One
pub const EPI_HB16TIME3_IRDYDLY_S = usize(24);
pub const EPI_HB16TIME3_CAPWIDTH_S = usize(12);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB8TIME3 register.
//
//*****************************************************************************
pub const EPI_HB8TIME3_IRDYDLY_M = usize(0x03000000);  // CS2n Input Ready Delay
pub const EPI_HB8TIME3_CAPWIDTH_M = usize(0x00003000);  // CS2n Inter-transfer Capture
                                            // Width
pub const EPI_HB8TIME3_WRWSM = usize(0x00000010);  // CS2n Write Wait State Minus One
pub const EPI_HB8TIME3_RDWSM = usize(0x00000001);  // CS2n Read Wait State Minus One
pub const EPI_HB8TIME3_IRDYDLY_S = usize(24);
pub const EPI_HB8TIME3_CAPWIDTH_S = usize(12);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB8TIME4 register.
//
//*****************************************************************************
pub const EPI_HB8TIME4_IRDYDLY_M = usize(0x03000000);  // CS3n Input Ready Delay
pub const EPI_HB8TIME4_CAPWIDTH_M = usize(0x00003000);  // CS3n Inter-transfer Capture
                                            // Width
pub const EPI_HB8TIME4_WRWSM = usize(0x00000010);  // CS3n Write Wait State Minus One
pub const EPI_HB8TIME4_RDWSM = usize(0x00000001);  // CS3n Read Wait State Minus One
pub const EPI_HB8TIME4_IRDYDLY_S = usize(24);
pub const EPI_HB8TIME4_CAPWIDTH_S = usize(12);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HB16TIME4
// register.
//
//*****************************************************************************
pub const EPI_HB16TIME4_IRDYDLY_M = usize(0x03000000);  // CS3n Input Ready Delay
pub const EPI_HB16TIME4_PSRAMSZ_M = usize(0x00070000);  // PSRAM Row Size
pub const EPI_HB16TIME4_PSRAMSZ_0 = usize(0x00000000);  // No row size limitation
pub const EPI_HB16TIME4_PSRAMSZ_128B = usize(0x00010000);  // 128 B
pub const EPI_HB16TIME4_PSRAMSZ_256B = usize(0x00020000);  // 256 B
pub const EPI_HB16TIME4_PSRAMSZ_512B = usize(0x00030000);  // 512 B
pub const EPI_HB16TIME4_PSRAMSZ_1KB = usize(0x00040000);  // 1024 B
pub const EPI_HB16TIME4_PSRAMSZ_2KB = usize(0x00050000);  // 2048 B
pub const EPI_HB16TIME4_PSRAMSZ_4KB = usize(0x00060000);  // 4096 B
pub const EPI_HB16TIME4_PSRAMSZ_8KB = usize(0x00070000);  // 8192 B
pub const EPI_HB16TIME4_CAPWIDTH_M = usize(0x00003000);  // CS3n Inter-transfer Capture
                                            // Width
pub const EPI_HB16TIME4_WRWSM = usize(0x00000010);  // CS3n Write Wait State Minus One
pub const EPI_HB16TIME4_RDWSM = usize(0x00000001);  // CS3n Read Wait State Minus One
pub const EPI_HB16TIME4_IRDYDLY_S = usize(24);
pub const EPI_HB16TIME4_CAPWIDTH_S = usize(12);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPI_O_HBPSRAM register.
//
//*****************************************************************************
pub const EPI_HBPSRAM_CR_M = usize(0x001FFFFF);  // PSRAM Config Register
pub const EPI_HBPSRAM_CR_S = usize(0);

//*****************************************************************************
//
// The following definitions are deprecated.
//
//*****************************************************************************

