//*****************************************************************************
//
// hw_lcd.h - Defines and macros used when accessing the LCD controller.
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
// The following are defines for the LCD register offsets.
//
//*****************************************************************************
pub const LCD_O_PID = usize(0x00000000);  // LCD PID Register Format
pub const LCD_O_CTL = usize(0x00000004);  // LCD Control
pub const LCD_O_LIDDCTL = usize(0x0000000C);  // LCD LIDD Control
pub const LCD_O_LIDDCS0CFG = usize(0x00000010);  // LCD LIDD CS0 Configuration
pub const LCD_O_LIDDCS0ADDR = usize(0x00000014);  // LIDD CS0 Read/Write Address
pub const LCD_O_LIDDCS0DATA = usize(0x00000018);  // LIDD CS0 Data Read/Write
                                            // Initiation
pub const LCD_O_LIDDCS1CFG = usize(0x0000001C);  // LIDD CS1 Configuration
pub const LCD_O_LIDDCS1ADDR = usize(0x00000020);  // LIDD CS1 Address Read/Write
                                            // Initiation
pub const LCD_O_LIDDCS1DATA = usize(0x00000024);  // LIDD CS1 Data Read/Write
                                            // Initiation
pub const LCD_O_RASTRCTL = usize(0x00000028);  // LCD Raster Control
pub const LCD_O_RASTRTIM0 = usize(0x0000002C);  // LCD Raster Timing 0
pub const LCD_O_RASTRTIM1 = usize(0x00000030);  // LCD Raster Timing 1
pub const LCD_O_RASTRTIM2 = usize(0x00000034);  // LCD Raster Timing 2
pub const LCD_O_RASTRSUBP1 = usize(0x00000038);  // LCD Raster Subpanel Display 1
pub const LCD_O_RASTRSUBP2 = usize(0x0000003C);  // LCD Raster Subpanel Display 2
pub const LCD_O_DMACTL = usize(0x00000040);  // LCD DMA Control
pub const LCD_O_DMABAFB0 = usize(0x00000044);  // LCD DMA Frame Buffer 0 Base
                                            // Address
pub const LCD_O_DMACAFB0 = usize(0x00000048);  // LCD DMA Frame Buffer 0 Ceiling
                                            // Address
pub const LCD_O_DMABAFB1 = usize(0x0000004C);  // LCD DMA Frame Buffer 1 Base
                                            // Address
pub const LCD_O_DMACAFB1 = usize(0x00000050);  // LCD DMA Frame Buffer 1 Ceiling
                                            // Address
pub const LCD_O_SYSCFG = usize(0x00000054);  // LCD System Configuration
                                            // Register
pub const LCD_O_RISSET = usize(0x00000058);  // LCD Interrupt Raw Status and Set
                                            // Register
pub const LCD_O_MISCLR = usize(0x0000005C);  // LCD Interrupt Status and Clear
pub const LCD_O_IM = usize(0x00000060);  // LCD Interrupt Mask
pub const LCD_O_IENC = usize(0x00000064);  // LCD Interrupt Enable Clear
pub const LCD_O_CLKEN = usize(0x0000006C);  // LCD Clock Enable
pub const LCD_O_CLKRESET = usize(0x00000070);  // LCD Clock Resets

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_PID register.
//
//*****************************************************************************
pub const LCD_PID_MAJOR_M = usize(0x00000700);  // Major Release Number
pub const LCD_PID_MINOR_M = usize(0x0000003F);  // Minor Release Number
pub const LCD_PID_MAJOR_S = usize(8);
pub const LCD_PID_MINOR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_CTL register.
//
//*****************************************************************************
pub const LCD_CTL_CLKDIV_M = usize(0x0000FF00);  // Clock Divisor
pub const LCD_CTL_UFLOWRST = usize(0x00000002);  // Underflow Restart
pub const LCD_CTL_LCDMODE = usize(0x00000001);  // LCD Mode Select
pub const LCD_CTL_CLKDIV_S = usize(8);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCTL register.
//
//*****************************************************************************
pub const LCD_LIDDCTL_DMACS = usize(0x00000200);  // CS0/CS1 Select for LIDD DMA
                                            // Writes
pub const LCD_LIDDCTL_DMAEN = usize(0x00000100);  // LIDD DMA Enable
pub const LCD_LIDDCTL_CS1E1 = usize(0x00000080);  // Chip Select 1 (CS1)/Enable 1(E1)
                                            // Polarity Control
pub const LCD_LIDDCTL_CS0E0 = usize(0x00000040);  // Chip Select 0 (CS0)/Enable 0
                                            // (E0) Polarity Control
pub const LCD_LIDDCTL_WRDIRINV = usize(0x00000020);  // Write Strobe (WR) /Direction
                                            // (DIR) Polarity Control
pub const LCD_LIDDCTL_RDEN = usize(0x00000010);  // Read Strobe (RD) /Direct Enable
                                            // (EN) Polarity Control
pub const LCD_LIDDCTL_ALE = usize(0x00000008);  // Address Latch Enable (ALE)
                                            // Polarity Control
pub const LCD_LIDDCTL_MODE_M = usize(0x00000007);  // LIDD Mode Select
pub const LCD_LIDDCTL_MODE_SYNCM68 = usize(0x00000000);  // Synchronous Motorola 6800 Mode
pub const LCD_LIDDCTL_MODE_ASYNCM68 = usize(0x00000001);  // Asynchronous Motorola 6800 Mode
pub const LCD_LIDDCTL_MODE_SYNCM80 = usize(0x00000002);  // Synchronous Intel 8080 mode
pub const LCD_LIDDCTL_MODE_ASYNCM80 = usize(0x00000003);  // Asynchronous Intel 8080 mode
pub const LCD_LIDDCTL_MODE_ASYNCHIT = usize(0x00000004);  // Asynchronous Hitachi mode

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS0CFG
// register.
//
//*****************************************************************************
pub const LCD_LIDDCS0CFG_WRSU_M = usize(0xF8000000);  // Write Strobe (WR) Set-Up Cycles
pub const LCD_LIDDCS0CFG_WRDUR_M = usize(0x07E00000);  // Write Strobe (WR) Duration
                                            // Cycles
pub const LCD_LIDDCS0CFG_WRHOLD_M = usize(0x001E0000);  // Write Strobe (WR) Hold cycles
pub const LCD_LIDDCS0CFG_RDSU_M = usize(0x0001F000);  // Read Strobe (RD) Set-Up cycles
pub const LCD_LIDDCS0CFG_RDDUR_M = usize(0x00000FC0);  // Read Strobe (RD) Duration cycles
pub const LCD_LIDDCS0CFG_RDHOLD_M = usize(0x0000003C);  // Read Strobe (RD) Hold cycles
pub const LCD_LIDDCS0CFG_GAP_M = usize(0x00000003);  // Field value defines the number
                                            // of LCDMCLK cycles (GAP +1)
                                            // between the end of one CS0
                                            // (LCDAC) device access and the
                                            // start of another CS0 (LCDAC)
                                            // device access unless the two
                                            // accesses are both reads
pub const LCD_LIDDCS0CFG_WRSU_S = usize(27);
pub const LCD_LIDDCS0CFG_WRDUR_S = usize(21);
pub const LCD_LIDDCS0CFG_WRHOLD_S = usize(17);
pub const LCD_LIDDCS0CFG_RDSU_S = usize(12);
pub const LCD_LIDDCS0CFG_RDDUR_S = usize(6);
pub const LCD_LIDDCS0CFG_RDHOLD_S = usize(2);
pub const LCD_LIDDCS0CFG_GAP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS0ADDR
// register.
//
//*****************************************************************************
pub const LCD_LIDDCS0ADDR_CS0ADDR_M = usize(0x0000FFFF);  // LCD Address
pub const LCD_LIDDCS0ADDR_CS0ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS0DATA
// register.
//
//*****************************************************************************
pub const LCD_LIDDCS0DATA_CS0DATA_M = usize(0x0000FFFF);  // LCD Data Read/Write
pub const LCD_LIDDCS0DATA_CS0DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS1CFG
// register.
//
//*****************************************************************************
pub const LCD_LIDDCS1CFG_WRSU_M = usize(0xF8000000);  // Write Strobe (WR) Set-Up Cycles
pub const LCD_LIDDCS1CFG_WRDUR_M = usize(0x07E00000);  // Write Strobe (WR) Duration
                                            // Cycles
pub const LCD_LIDDCS1CFG_WRHOLD_M = usize(0x001E0000);  // Write Strobe (WR) Hold cycles
pub const LCD_LIDDCS1CFG_RDSU_M = usize(0x0001F000);  // Read Strobe (RD) Set-Up cycles
pub const LCD_LIDDCS1CFG_RDDUR_M = usize(0x00000FC0);  // Read Strobe (RD) Duration cycles
pub const LCD_LIDDCS1CFG_RDHOLD_M = usize(0x0000003C);  // Read Strobe (RD) Hold cycles
pub const LCD_LIDDCS1CFG_GAP_M = usize(0x00000003);  // Field value defines the number
                                            // of LCDMCLK cycles (GAP + 1)
                                            // between the end of one CS1
                                            // (LCDAC) device access and the
                                            // start of another CS0 (LCDAC)
                                            // device access unless the two
                                            // accesses are both reads
pub const LCD_LIDDCS1CFG_WRSU_S = usize(27);
pub const LCD_LIDDCS1CFG_WRDUR_S = usize(21);
pub const LCD_LIDDCS1CFG_WRHOLD_S = usize(17);
pub const LCD_LIDDCS1CFG_RDSU_S = usize(12);
pub const LCD_LIDDCS1CFG_RDDUR_S = usize(6);
pub const LCD_LIDDCS1CFG_RDHOLD_S = usize(2);
pub const LCD_LIDDCS1CFG_GAP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS1ADDR
// register.
//
//*****************************************************************************
pub const LCD_LIDDCS1ADDR_CS1ADDR_M = usize(0x0000FFFF);  // LCD Address Bus
pub const LCD_LIDDCS1ADDR_CS1ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS1DATA
// register.
//
//*****************************************************************************
pub const LCD_LIDDCS1DATA_CS0DATA_M = usize(0x0000FFFF);  // LCD Data Read/Write Initiation
pub const LCD_LIDDCS1DATA_CS0DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRCTL register.
//
//*****************************************************************************
pub const LCD_RASTRCTL_TFT24UPCK = usize(0x04000000);  // 24-bit TFT Mode Packing
pub const LCD_RASTRCTL_TFT24 = usize(0x02000000);  // 24-Bit TFT Mode
pub const LCD_RASTRCTL_FRMBUFSZ = usize(0x01000000);  // Frame Buffer Select
pub const LCD_RASTRCTL_TFTMAP = usize(0x00800000);  // TFT Mode Alternate Signal
                                            // Mapping for Palettized
                                            // Framebuffer
pub const LCD_RASTRCTL_NIBMODE = usize(0x00400000);  // Nibble Mode
pub const LCD_RASTRCTL_PALMODE_M = usize(0x00300000);  // Pallette Loading Mode
pub const LCD_RASTRCTL_PALMODE_PALDAT = usize(0x00000000);  // Palette and data loading, reset
                                            // value
pub const LCD_RASTRCTL_PALMODE_PAL = usize(0x00100000);  // Palette loading only
pub const LCD_RASTRCTL_PALMODE_DAT = usize(0x00200000);  // Data loading only
pub const LCD_RASTRCTL_REQDLY_M = usize(0x000FF000);  // Palette Loading Delay
pub const LCD_RASTRCTL_MONO8B = usize(0x00000200);  // Mono 8-Bit
pub const LCD_RASTRCTL_RDORDER = usize(0x00000100);  // Raster Data Order Select
pub const LCD_RASTRCTL_LCDTFT = usize(0x00000080);  // LCD TFT
pub const LCD_RASTRCTL_LCDBW = usize(0x00000002);  // LCD Monochrome
pub const LCD_RASTRCTL_LCDEN = usize(0x00000001);  // LCD Controller Enable for Raster
                                            // Operations
pub const LCD_RASTRCTL_REQDLY_S = usize(12);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRTIM0
// register.
//
//*****************************************************************************
pub const LCD_RASTRTIM0_HBP_M = usize(0xFF000000);  // Horizontal Back Porch Lowbits
pub const LCD_RASTRTIM0_HFP_M = usize(0x00FF0000);  // Horizontal Front Porch Lowbits
pub const LCD_RASTRTIM0_HSW_M = usize(0x0000FC00);  // Horizontal Sync Pulse Width
                                            // Lowbits
pub const LCD_RASTRTIM0_PPL_M = usize(0x000003F0);  // Pixels-per-line LSB[9:4]
pub const LCD_RASTRTIM0_MSBPPL = usize(0x00000008);  // Pixels-per-line MSB[10]
pub const LCD_RASTRTIM0_HBP_S = usize(24);
pub const LCD_RASTRTIM0_HFP_S = usize(16);
pub const LCD_RASTRTIM0_HSW_S = usize(10);
pub const LCD_RASTRTIM0_PPL_S = usize(4);
pub const LCD_RASTRTIM0_MSBPPL_S = usize(3);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRTIM1
// register.
//
//*****************************************************************************
pub const LCD_RASTRTIM1_VBP_M = usize(0xFF000000);  // Vertical Back Porch
pub const LCD_RASTRTIM1_VFP_M = usize(0x00FF0000);  // Vertical Front Porch
pub const LCD_RASTRTIM1_VSW_M = usize(0x0000FC00);  // Vertical Sync Width Pulse
pub const LCD_RASTRTIM1_LPP_M = usize(0x000003FF);  // Lines Per Panel
pub const LCD_RASTRTIM1_VBP_S = usize(24);
pub const LCD_RASTRTIM1_VFP_S = usize(16);
pub const LCD_RASTRTIM1_VSW_S = usize(10);
pub const LCD_RASTRTIM1_LPP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRTIM2
// register.
//
//*****************************************************************************
pub const LCD_RASTRTIM2_HSW_M = usize(0x78000000);  // Bits 9:6 of the horizontal sync
                                            // width field
pub const LCD_RASTRTIM2_MSBLPP = usize(0x04000000);  // MSB of Lines Per Panel
pub const LCD_RASTRTIM2_PXLCLKCTL = usize(0x02000000);  // Hsync/Vsync Pixel Clock Control
                                            // On/Off
pub const LCD_RASTRTIM2_PSYNCRF = usize(0x01000000);  // Program HSYNC/VSYNC Rise or Fall
pub const LCD_RASTRTIM2_INVOE = usize(0x00800000);  // Invert Output Enable
pub const LCD_RASTRTIM2_INVPXLCLK = usize(0x00400000);  // Invert Pixel Clock
pub const LCD_RASTRTIM2_IHS = usize(0x00200000);  // Invert Hysync
pub const LCD_RASTRTIM2_IVS = usize(0x00100000);  // Invert Vsync
pub const LCD_RASTRTIM2_ACBI_M = usize(0x000F0000);  // AC Bias Pins Transitions per
                                            // Interrupt
pub const LCD_RASTRTIM2_ACBF_M = usize(0x0000FF00);  // AC Bias Pin Frequency
pub const LCD_RASTRTIM2_MSBHBP_M = usize(0x00000030);  // Bits 9:8 of the horizontal back
                                            // porch field
pub const LCD_RASTRTIM2_MSBHFP_M = usize(0x00000003);  // Bits 9:8 of the horizontal front
                                            // porch field
pub const LCD_RASTRTIM2_HSW_S = usize(27);
pub const LCD_RASTRTIM2_MSBLPP_S = usize(26);
pub const LCD_RASTRTIM2_ACBI_S = usize(16);
pub const LCD_RASTRTIM2_ACBF_S = usize(8);
pub const LCD_RASTRTIM2_MSBHBP_S = usize(4);
pub const LCD_RASTRTIM2_MSBHFP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRSUBP1
// register.
//
//*****************************************************************************
pub const LCD_RASTRSUBP1_SPEN = usize(0x80000000);  // Sub Panel Enable
pub const LCD_RASTRSUBP1_HOLS = usize(0x20000000);  // High or Low Signal
pub const LCD_RASTRSUBP1_LPPT_M = usize(0x03FF0000);  // Line Per Panel Threshold
pub const LCD_RASTRSUBP1_DPDLSB_M = usize(0x0000FFFF);  // Default Pixel Data LSB[15:0]
pub const LCD_RASTRSUBP1_LPPT_S = usize(16);
pub const LCD_RASTRSUBP1_DPDLSB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRSUBP2
// register.
//
//*****************************************************************************
pub const LCD_RASTRSUBP2_LPPTMSB = usize(0x00000100);  // Lines Per Panel Threshold Bit 10
pub const LCD_RASTRSUBP2_DPDMSB_M = usize(0x000000FF);  // Default Pixel Data MSB [23:16]
pub const LCD_RASTRSUBP2_DPDMSB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_DMACTL register.
//
//*****************************************************************************
pub const LCD_DMACTL_FIFORDY_M = usize(0x00000700);  // DMA FIFO threshold
pub const LCD_DMACTL_FIFORDY_8 = usize(0x00000000);  // 8 words
pub const LCD_DMACTL_FIFORDY_16 = usize(0x00000100);  // 16 words
pub const LCD_DMACTL_FIFORDY_32 = usize(0x00000200);  // 32 words
pub const LCD_DMACTL_FIFORDY_64 = usize(0x00000300);  // 64 words
pub const LCD_DMACTL_FIFORDY_128 = usize(0x00000400);  // 128 words
pub const LCD_DMACTL_FIFORDY_256 = usize(0x00000500);  // 256 words
pub const LCD_DMACTL_FIFORDY_512 = usize(0x00000600);  // 512 words
pub const LCD_DMACTL_BURSTSZ_M = usize(0x00000070);  // Burst Size setting for DMA
                                            // transfers (all DMA transfers are
                                            // 32 bits wide):
pub const LCD_DMACTL_BURSTSZ_4 = usize(0x00000020);  // burst size of 4
pub const LCD_DMACTL_BURSTSZ_8 = usize(0x00000030);  // burst size of 8
pub const LCD_DMACTL_BURSTSZ_16 = usize(0x00000040);  // burst size of 16
pub const LCD_DMACTL_BYTESWAP = usize(0x00000008);  // This bit controls the bytelane
                                            // ordering of the data on the
                                            // output of the DMA module
pub const LCD_DMACTL_BIGDEND = usize(0x00000002);  // Big Endian Enable
pub const LCD_DMACTL_FMODE = usize(0x00000001);  // Frame Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_DMABAFB0 register.
//
//*****************************************************************************
pub const LCD_DMABAFB0_FB0BA_M = usize(0xFFFFFFFC);  // Frame Buffer 0 Base Address
                                            // pointer
pub const LCD_DMABAFB0_FB0BA_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_DMACAFB0 register.
//
//*****************************************************************************
pub const LCD_DMACAFB0_FB0CA_M = usize(0xFFFFFFFC);  // Frame Buffer 0 Ceiling Address
                                            // pointer
pub const LCD_DMACAFB0_FB0CA_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_DMABAFB1 register.
//
//*****************************************************************************
pub const LCD_DMABAFB1_FB1BA_M = usize(0xFFFFFFFC);  // Frame Buffer 1 Base Address
                                            // pointer
pub const LCD_DMABAFB1_FB1BA_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_DMACAFB1 register.
//
//*****************************************************************************
pub const LCD_DMACAFB1_FB1CA_M = usize(0xFFFFFFFC);  // Frame Buffer 1 Ceiling Address
                                            // pointer
pub const LCD_DMACAFB1_FB1CA_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_SYSCFG register.
//
//*****************************************************************************
pub const LCD_SYSCFG_STDBY_M = usize(0x00000030);  // Standby Mode
pub const LCD_SYSCFG_STDBY_FORCE = usize(0x00000000);  // Force-standby mode: local
                                            // initiator is unconditionally
                                            // placed in standby state. Backup
                                            // mode, for debug only
pub const LCD_SYSCFG_STDBY_NONE = usize(0x00000010);  // No-standby mode: local initiator
                                            // is unconditionally placed out of
                                            // standby state. Backup mode, for
                                            // debug only
pub const LCD_SYSCFG_STDBY_SMART = usize(0x00000020);  // Smart-standby mode: local
                                            // initiator standby status depends
                                            // on local conditions, that is,
                                            // the module's functional
                                            // requirement from the initiator.
                                            // IP module shall not generate
                                            // (initiator-related) wakeup
                                            // events
pub const LCD_SYSCFG_IDLEMODE_M = usize(0x0000000C);  // Idle Mode
pub const LCD_SYSCFG_IDLEMODE_FORCE = usize(0x00000000);  // Force-idle mode: local target's
                                            // idle state follows
                                            // (acknowledges) the system's idle
                                            // requests unconditionally, that
                                            // is, regardless of the IP
                                            // module's internal requirements.
                                            // Backup mode, for debug only
pub const LCD_SYSCFG_IDLEMODE_NONE = usize(0x00000004);  // No-idle mode: local target never
                                            // enters idle state. Backup mode,
                                            // for debug only
pub const LCD_SYSCFG_IDLEMODE_SMART = usize(0x00000008);  // Smart-idle mode: local target's
                                            // idle state eventually follows
                                            // (acknowledges) the system's idle
                                            // requests, depending on the IP
                                            // module's internal requirements.
                                            // IP module shall not generate
                                            // (IRQ- or DMA-requestrelated)
                                            // wakeup events

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RISSET register.
//
//*****************************************************************************
pub const LCD_RISSET_EOF1 = usize(0x00000200);  // DMA End-of-Frame 1 Raw Interrupt
                                            // Status and Set
pub const LCD_RISSET_EOF0 = usize(0x00000100);  // DMA End-of-Frame 0 Raw Interrupt
                                            // Status and Set
pub const LCD_RISSET_PALLOAD = usize(0x00000040);  // DMA Palette Loaded Raw Interrupt
                                            // Status and Set
pub const LCD_RISSET_FIFOU = usize(0x00000020);  // DMA FIFO Underflow Raw Interrupt
                                            // Status and Set
pub const LCD_RISSET_ACBS = usize(0x00000008);  // AC Bias Count Raw Interrupt
                                            // Status and Set
pub const LCD_RISSET_SYNCS = usize(0x00000004);  // Frame Synchronization Lost Raw
                                            // Interrupt Status and Set
pub const LCD_RISSET_RRASTRDONE = usize(0x00000002);  // Raster Mode Frame Done interrupt
pub const LCD_RISSET_DONE = usize(0x00000001);  // Raster or LIDD Frame Done
                                            // (shared, depends on whether
                                            // Raster or LIDD mode enabled) Raw
                                            // Interrupt Status and Set

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_MISCLR register.
//
//*****************************************************************************
pub const LCD_MISCLR_EOF1 = usize(0x00000200);  // DMA End-of-Frame 1 Enabled
                                            // Interrupt and Clear
pub const LCD_MISCLR_EOF0 = usize(0x00000100);  // DMA End-of-Frame 0 Raw Interrupt
                                            // and Clear
pub const LCD_MISCLR_PALLOAD = usize(0x00000040);  // DMA Palette Loaded Enabled
                                            // Interrupt and Clear
pub const LCD_MISCLR_FIFOU = usize(0x00000020);  // DMA FIFO Underflow Enabled
                                            // Interrupt and Clear
pub const LCD_MISCLR_ACBS = usize(0x00000008);  // AC Bias Count Enabled Interrupt
                                            // and Clear
pub const LCD_MISCLR_SYNCS = usize(0x00000004);  // Frame Synchronization Lost
                                            // Enabled Interrupt and Clear
pub const LCD_MISCLR_RRASTRDONE = usize(0x00000002);  // Raster Mode Frame Done interrupt
pub const LCD_MISCLR_DONE = usize(0x00000001);  // Raster or LIDD Frame Done
                                            // (shared, depends on whether
                                            // Raster or LIDD mode enabled)
                                            // Enabled Interrupt and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_IM register.
//
//*****************************************************************************
pub const LCD_IM_EOF1 = usize(0x00000200);  // DMA End-of-Frame 1 Interrupt
                                            // Enable Set
pub const LCD_IM_EOF0 = usize(0x00000100);  // DMA End-of-Frame 0 Interrupt
                                            // Enable Set
pub const LCD_IM_PALLOAD = usize(0x00000040);  // DMA Palette Loaded Interrupt
                                            // Enable Set
pub const LCD_IM_FIFOU = usize(0x00000020);  // DMA FIFO Underflow Interrupt
                                            // Enable Set
pub const LCD_IM_ACBS = usize(0x00000008);  // AC Bias Count Interrupt Enable
                                            // Set
pub const LCD_IM_SYNCS = usize(0x00000004);  // Frame Synchronization Lost
                                            // Interrupt Enable Set
pub const LCD_IM_RRASTRDONE = usize(0x00000002);  // Raster Mode Frame Done Interrupt
                                            // Enable Set
pub const LCD_IM_DONE = usize(0x00000001);  // Raster or LIDD Frame Done
                                            // (shared, depends on whether
                                            // Raster or LIDD mode enabled)
                                            // Interrupt Enable Set

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_IENC register.
//
//*****************************************************************************
pub const LCD_IENC_EOF1 = usize(0x00000200);  // DMA End-of-Frame 1 Interrupt
                                            // Enable Clear
pub const LCD_IENC_EOF0 = usize(0x00000100);  // DMA End-of-Frame 0 Interrupt
                                            // Enable Clear
pub const LCD_IENC_PALLOAD = usize(0x00000040);  // DMA Palette Loaded Interrupt
                                            // Enable Clear
pub const LCD_IENC_FIFOU = usize(0x00000020);  // DMA FIFO Underflow Interrupt
                                            // Enable Clear
pub const LCD_IENC_ACBS = usize(0x00000008);  // AC Bias Count Interrupt Enable
                                            // Clear
pub const LCD_IENC_SYNCS = usize(0x00000004);  // Frame Synchronization Lost
                                            // Interrupt Enable Clear
pub const LCD_IENC_RRASTRDONE = usize(0x00000002);  // Raster Mode Frame Done Interrupt
                                            // Enable Clear
pub const LCD_IENC_DONE = usize(0x00000001);  // Raster or LIDD Frame Done
                                            // (shared, depends on whether
                                            // Raster or LIDD mode enabled)
                                            // Interrupt Enable Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_CLKEN register.
//
//*****************************************************************************
pub const LCD_CLKEN_DMA = usize(0x00000004);  // DMA Clock Enable
pub const LCD_CLKEN_LIDD = usize(0x00000002);  // LIDD Submodule Clock Enable
pub const LCD_CLKEN_CORE = usize(0x00000001);  // LCD Core Clock Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_CLKRESET register.
//
//*****************************************************************************
pub const LCD_CLKRESET_MAIN = usize(0x00000008);  // Software Reset for the entire
                                            // LCD module
pub const LCD_CLKRESET_DMA = usize(0x00000004);  // Software Reset for the DMA
                                            // submodule
pub const LCD_CLKRESET_LIDD = usize(0x00000002);  // Software Reset for the LIDD
                                            // submodule (character displays)
pub const LCD_CLKRESET_CORE = usize(0x00000001);  // Software Reset for the Core,
                                            // which encompasses the Raster
                                            // Active Matrix and Passive Matrix
                                            // logic

