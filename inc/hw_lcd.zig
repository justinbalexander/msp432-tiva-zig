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
pub const offsetOf = struct {
    pub const PID = usize(0x00000000); // LCD PID Register Format
    pub const CTL = usize(0x00000004); // LCD Control
    pub const LIDDCTL = usize(0x0000000C); // LCD LIDD Control
    pub const LIDDCS0CFG = usize(0x00000010); // LCD LIDD CS0 Configuration
    pub const LIDDCS0ADDR = usize(0x00000014); // LIDD CS0 Read/Write Address
    pub const LIDDCS0DATA = usize(0x00000018); // LIDD CS0 Data Read/Write
    // Initiation
    pub const LIDDCS1CFG = usize(0x0000001C); // LIDD CS1 Configuration
    pub const LIDDCS1ADDR = usize(0x00000020); // LIDD CS1 Address Read/Write
    // Initiation
    pub const LIDDCS1DATA = usize(0x00000024); // LIDD CS1 Data Read/Write
    // Initiation
    pub const RASTRCTL = usize(0x00000028); // LCD Raster Control
    pub const RASTRTIM0 = usize(0x0000002C); // LCD Raster Timing 0
    pub const RASTRTIM1 = usize(0x00000030); // LCD Raster Timing 1
    pub const RASTRTIM2 = usize(0x00000034); // LCD Raster Timing 2
    pub const RASTRSUBP1 = usize(0x00000038); // LCD Raster Subpanel Display 1
    pub const RASTRSUBP2 = usize(0x0000003C); // LCD Raster Subpanel Display 2
    pub const DMACTL = usize(0x00000040); // LCD DMA Control
    pub const DMABAFB0 = usize(0x00000044); // LCD DMA Frame Buffer 0 Base
    // Address
    pub const DMACAFB0 = usize(0x00000048); // LCD DMA Frame Buffer 0 Ceiling
    // Address
    pub const DMABAFB1 = usize(0x0000004C); // LCD DMA Frame Buffer 1 Base
    // Address
    pub const DMACAFB1 = usize(0x00000050); // LCD DMA Frame Buffer 1 Ceiling
    // Address
    pub const SYSCFG = usize(0x00000054); // LCD System Configuration
    // Register
    pub const RISSET = usize(0x00000058); // LCD Interrupt Raw Status and Set
    // Register
    pub const MISCLR = usize(0x0000005C); // LCD Interrupt Status and Clear
    pub const IM = usize(0x00000060); // LCD Interrupt Mask
    pub const IENC = usize(0x00000064); // LCD Interrupt Enable Clear
    pub const CLKEN = usize(0x0000006C); // LCD Clock Enable
    pub const CLKRESET = usize(0x00000070); // LCD Clock Resets
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_PID register.
//
//*****************************************************************************
pub const PID = struct {
    pub const MAJOR_M = usize(0x00000700); // Major Release Number
    pub const MINOR_M = usize(0x0000003F); // Minor Release Number
    pub const MAJOR_S = usize(8);
    pub const MINOR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_CTL register.
//
//*****************************************************************************
pub const CTL = struct {
    pub const CLKDIV_M = usize(0x0000FF00); // Clock Divisor
    pub const UFLOWRST = usize(0x00000002); // Underflow Restart
    pub const LCDMODE = usize(0x00000001); // LCD Mode Select
    pub const CLKDIV_S = usize(8);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCTL register.
//
//*****************************************************************************
pub const LIDDCTL = struct {
    pub const DMACS = usize(0x00000200); // CS0/CS1 Select for LIDD DMA
    // Writes
    pub const DMAEN = usize(0x00000100); // LIDD DMA Enable
    pub const CS1E1 = usize(0x00000080); // Chip Select 1 (CS1)/Enable 1(E1)
    // Polarity Control
    pub const CS0E0 = usize(0x00000040); // Chip Select 0 (CS0)/Enable 0
    // (E0) Polarity Control
    pub const WRDIRINV = usize(0x00000020); // Write Strobe (WR) /Direction
    // (DIR) Polarity Control
    pub const RDEN = usize(0x00000010); // Read Strobe (RD) /Direct Enable
    // (EN) Polarity Control
    pub const ALE = usize(0x00000008); // Address Latch Enable (ALE)
    // Polarity Control
    pub const MODE_M = usize(0x00000007); // LIDD Mode Select
    pub const MODE_SYNCM68 = usize(0x00000000); // Synchronous Motorola 6800 Mode
    pub const MODE_ASYNCM68 = usize(0x00000001); // Asynchronous Motorola 6800 Mode
    pub const MODE_SYNCM80 = usize(0x00000002); // Synchronous Intel 8080 mode
    pub const MODE_ASYNCM80 = usize(0x00000003); // Asynchronous Intel 8080 mode
    pub const MODE_ASYNCHIT = usize(0x00000004); // Asynchronous Hitachi mode
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS0CFG
// register.
//
//*****************************************************************************
pub const LIDDCS0CFG = struct {
    pub const WRSU_M = usize(0xF8000000); // Write Strobe (WR) Set-Up Cycles
    pub const WRDUR_M = usize(0x07E00000); // Write Strobe (WR) Duration
    // Cycles
    pub const WRHOLD_M = usize(0x001E0000); // Write Strobe (WR) Hold cycles
    pub const RDSU_M = usize(0x0001F000); // Read Strobe (RD) Set-Up cycles
    pub const RDDUR_M = usize(0x00000FC0); // Read Strobe (RD) Duration cycles
    pub const RDHOLD_M = usize(0x0000003C); // Read Strobe (RD) Hold cycles
    pub const GAP_M = usize(0x00000003); // Field value defines the number
    // of LCDMCLK cycles (GAP +1)
    // between the end of one CS0
    // (LCDAC) device access and the
    // start of another CS0 (LCDAC)
    // device access unless the two
    // accesses are both reads
    pub const WRSU_S = usize(27);
    pub const WRDUR_S = usize(21);
    pub const WRHOLD_S = usize(17);
    pub const RDSU_S = usize(12);
    pub const RDDUR_S = usize(6);
    pub const RDHOLD_S = usize(2);
    pub const GAP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS0ADDR
// register.
//
//*****************************************************************************
pub const LIDDCS0ADDR = struct {
    pub const CS0ADDR_M = usize(0x0000FFFF); // LCD Address
    pub const CS0ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS0DATA
// register.
//
//*****************************************************************************
pub const LIDDCS0DATA = struct {
    pub const CS0DATA_M = usize(0x0000FFFF); // LCD Data Read/Write
    pub const CS0DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS1CFG
// register.
//
//*****************************************************************************
pub const LIDDCS1CFG = struct {
    pub const WRSU_M = usize(0xF8000000); // Write Strobe (WR) Set-Up Cycles
    pub const WRDUR_M = usize(0x07E00000); // Write Strobe (WR) Duration
    // Cycles
    pub const WRHOLD_M = usize(0x001E0000); // Write Strobe (WR) Hold cycles
    pub const RDSU_M = usize(0x0001F000); // Read Strobe (RD) Set-Up cycles
    pub const RDDUR_M = usize(0x00000FC0); // Read Strobe (RD) Duration cycles
    pub const RDHOLD_M = usize(0x0000003C); // Read Strobe (RD) Hold cycles
    pub const GAP_M = usize(0x00000003); // Field value defines the number
    // of LCDMCLK cycles (GAP + 1)
    // between the end of one CS1
    // (LCDAC) device access and the
    // start of another CS0 (LCDAC)
    // device access unless the two
    // accesses are both reads
    pub const WRSU_S = usize(27);
    pub const WRDUR_S = usize(21);
    pub const WRHOLD_S = usize(17);
    pub const RDSU_S = usize(12);
    pub const RDDUR_S = usize(6);
    pub const RDHOLD_S = usize(2);
    pub const GAP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS1ADDR
// register.
//
//*****************************************************************************
pub const LIDDCS1ADDR = struct {
    pub const CS1ADDR_M = usize(0x0000FFFF); // LCD Address Bus
    pub const CS1ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_LIDDCS1DATA
// register.
//
//*****************************************************************************
pub const LIDDCS1DATA = struct {
    pub const CS0DATA_M = usize(0x0000FFFF); // LCD Data Read/Write Initiation
    pub const CS0DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRCTL register.
//
//*****************************************************************************
pub const RASTRCTL = struct {
    pub const TFT24UPCK = usize(0x04000000); // 24-bit TFT Mode Packing
    pub const TFT24 = usize(0x02000000); // 24-Bit TFT Mode
    pub const FRMBUFSZ = usize(0x01000000); // Frame Buffer Select
    pub const TFTMAP = usize(0x00800000); // TFT Mode Alternate Signal
    // Mapping for Palettized
    // Framebuffer
    pub const NIBMODE = usize(0x00400000); // Nibble Mode
    pub const PALMODE_M = usize(0x00300000); // Pallette Loading Mode
    pub const PALMODE_PALDAT = usize(0x00000000); // Palette and data loading, reset
    // value
    pub const PALMODE_PAL = usize(0x00100000); // Palette loading only
    pub const PALMODE_DAT = usize(0x00200000); // Data loading only
    pub const REQDLY_M = usize(0x000FF000); // Palette Loading Delay
    pub const MONO8B = usize(0x00000200); // Mono 8-Bit
    pub const RDORDER = usize(0x00000100); // Raster Data Order Select
    pub const LCDTFT = usize(0x00000080); // LCD TFT
    pub const LCDBW = usize(0x00000002); // LCD Monochrome
    pub const LCDEN = usize(0x00000001); // LCD Controller Enable for Raster
    // Operations
    pub const REQDLY_S = usize(12);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRTIM0
// register.
//
//*****************************************************************************
pub const RASTRTIM0 = struct {
    pub const HBP_M = usize(0xFF000000); // Horizontal Back Porch Lowbits
    pub const HFP_M = usize(0x00FF0000); // Horizontal Front Porch Lowbits
    pub const HSW_M = usize(0x0000FC00); // Horizontal Sync Pulse Width
    // Lowbits
    pub const PPL_M = usize(0x000003F0); // Pixels-per-line LSB[9:4]
    pub const MSBPPL = usize(0x00000008); // Pixels-per-line MSB[10]
    pub const HBP_S = usize(24);
    pub const HFP_S = usize(16);
    pub const HSW_S = usize(10);
    pub const PPL_S = usize(4);
    pub const MSBPPL_S = usize(3);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRTIM1
// register.
//
//*****************************************************************************
pub const RASTRTIM1 = struct {
    pub const VBP_M = usize(0xFF000000); // Vertical Back Porch
    pub const VFP_M = usize(0x00FF0000); // Vertical Front Porch
    pub const VSW_M = usize(0x0000FC00); // Vertical Sync Width Pulse
    pub const LPP_M = usize(0x000003FF); // Lines Per Panel
    pub const VBP_S = usize(24);
    pub const VFP_S = usize(16);
    pub const VSW_S = usize(10);
    pub const LPP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRTIM2
// register.
//
//*****************************************************************************
pub const RASTRTIM2 = struct {
    pub const HSW_M = usize(0x78000000); // Bits 9:6 of the horizontal sync
    // width field
    pub const MSBLPP = usize(0x04000000); // MSB of Lines Per Panel
    pub const PXLCLKCTL = usize(0x02000000); // Hsync/Vsync Pixel Clock Control
    // On/Off
    pub const PSYNCRF = usize(0x01000000); // Program HSYNC/VSYNC Rise or Fall
    pub const INVOE = usize(0x00800000); // Invert Output Enable
    pub const INVPXLCLK = usize(0x00400000); // Invert Pixel Clock
    pub const IHS = usize(0x00200000); // Invert Hysync
    pub const IVS = usize(0x00100000); // Invert Vsync
    pub const ACBI_M = usize(0x000F0000); // AC Bias Pins Transitions per
    // Interrupt
    pub const ACBF_M = usize(0x0000FF00); // AC Bias Pin Frequency
    pub const MSBHBP_M = usize(0x00000030); // Bits 9:8 of the horizontal back
    // porch field
    pub const MSBHFP_M = usize(0x00000003); // Bits 9:8 of the horizontal front
    // porch field
    pub const HSW_S = usize(27);
    pub const MSBLPP_S = usize(26);
    pub const ACBI_S = usize(16);
    pub const ACBF_S = usize(8);
    pub const MSBHBP_S = usize(4);
    pub const MSBHFP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRSUBP1
// register.
//
//*****************************************************************************
pub const RASTRSUBP1 = struct {
    pub const SPEN = usize(0x80000000); // Sub Panel Enable
    pub const HOLS = usize(0x20000000); // High or Low Signal
    pub const LPPT_M = usize(0x03FF0000); // Line Per Panel Threshold
    pub const DPDLSB_M = usize(0x0000FFFF); // Default Pixel Data LSB[15:0]
    pub const LPPT_S = usize(16);
    pub const DPDLSB_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RASTRSUBP2
// register.
//
//*****************************************************************************
pub const RASTRSUBP2 = struct {
    pub const LPPTMSB = usize(0x00000100); // Lines Per Panel Threshold Bit 10
    pub const DPDMSB_M = usize(0x000000FF); // Default Pixel Data MSB [23:16]
    pub const DPDMSB_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_DMACTL register.
//
//*****************************************************************************
pub const DMACTL = struct {
    pub const FIFORDY_M = usize(0x00000700); // DMA FIFO threshold
    pub const FIFORDY_8 = usize(0x00000000); // 8 words
    pub const FIFORDY_16 = usize(0x00000100); // 16 words
    pub const FIFORDY_32 = usize(0x00000200); // 32 words
    pub const FIFORDY_64 = usize(0x00000300); // 64 words
    pub const FIFORDY_128 = usize(0x00000400); // 128 words
    pub const FIFORDY_256 = usize(0x00000500); // 256 words
    pub const FIFORDY_512 = usize(0x00000600); // 512 words
    pub const BURSTSZ_M = usize(0x00000070); // Burst Size setting for DMA
    // transfers (all DMA transfers are
    // 32 bits wide):
    pub const BURSTSZ_4 = usize(0x00000020); // burst size of 4
    pub const BURSTSZ_8 = usize(0x00000030); // burst size of 8
    pub const BURSTSZ_16 = usize(0x00000040); // burst size of 16
    pub const BYTESWAP = usize(0x00000008); // This bit controls the bytelane
    // ordering of the data on the
    // output of the DMA module
    pub const BIGDEND = usize(0x00000002); // Big Endian Enable
    pub const FMODE = usize(0x00000001); // Frame Mode
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_DMABAFB0 register.
//
//*****************************************************************************
pub const DMABAFB0 = struct {
    pub const FB0BA_M = usize(0xFFFFFFFC); // Frame Buffer 0 Base Address
    // pointer
    pub const FB0BA_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_DMACAFB0 register.
//
//*****************************************************************************
pub const DMACAFB0 = struct {
    pub const FB0CA_M = usize(0xFFFFFFFC); // Frame Buffer 0 Ceiling Address
    // pointer
    pub const FB0CA_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_DMABAFB1 register.
//
//*****************************************************************************
pub const DMABAFB1 = struct {
    pub const FB1BA_M = usize(0xFFFFFFFC); // Frame Buffer 1 Base Address
    // pointer
    pub const FB1BA_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_DMACAFB1 register.
//
//*****************************************************************************
pub const DMACAFB1 = struct {
    pub const FB1CA_M = usize(0xFFFFFFFC); // Frame Buffer 1 Ceiling Address
    // pointer
    pub const FB1CA_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_SYSCFG register.
//
//*****************************************************************************
pub const SYSCFG = struct {
    pub const STDBY_M = usize(0x00000030); // Standby Mode
    pub const STDBY_FORCE = usize(0x00000000); // Force-standby mode: local
    // initiator is unconditionally
    // placed in standby state. Backup
    // mode, for debug only
    pub const STDBY_NONE = usize(0x00000010); // No-standby mode: local initiator
    // is unconditionally placed out of
    // standby state. Backup mode, for
    // debug only
    pub const STDBY_SMART = usize(0x00000020); // Smart-standby mode: local
    // initiator standby status depends
    // on local conditions, that is,
    // the module's functional
    // requirement from the initiator.
    // IP module shall not generate
    // (initiator-related) wakeup
    // events
    pub const IDLEMODE_M = usize(0x0000000C); // Idle Mode
    pub const IDLEMODE_FORCE = usize(0x00000000); // Force-idle mode: local target's
    // idle state follows
    // (acknowledges) the system's idle
    // requests unconditionally, that
    // is, regardless of the IP
    // module's internal requirements.
    // Backup mode, for debug only
    pub const IDLEMODE_NONE = usize(0x00000004); // No-idle mode: local target never
    // enters idle state. Backup mode,
    // for debug only
    pub const IDLEMODE_SMART = usize(0x00000008); // Smart-idle mode: local target's
    // idle state eventually follows
    // (acknowledges) the system's idle
    // requests, depending on the IP
    // module's internal requirements.
    // IP module shall not generate
    // (IRQ- or DMA-requestrelated)
    // wakeup events
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_RISSET register.
//
//*****************************************************************************
pub const RISSET = struct {
    pub const EOF1 = usize(0x00000200); // DMA End-of-Frame 1 Raw Interrupt
    // Status and Set
    pub const EOF0 = usize(0x00000100); // DMA End-of-Frame 0 Raw Interrupt
    // Status and Set
    pub const PALLOAD = usize(0x00000040); // DMA Palette Loaded Raw Interrupt
    // Status and Set
    pub const FIFOU = usize(0x00000020); // DMA FIFO Underflow Raw Interrupt
    // Status and Set
    pub const ACBS = usize(0x00000008); // AC Bias Count Raw Interrupt
    // Status and Set
    pub const SYNCS = usize(0x00000004); // Frame Synchronization Lost Raw
    // Interrupt Status and Set
    pub const RRASTRDONE = usize(0x00000002); // Raster Mode Frame Done interrupt
    pub const DONE = usize(0x00000001); // Raster or LIDD Frame Done
    // (shared, depends on whether
    // Raster or LIDD mode enabled) Raw
    // Interrupt Status and Set
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_MISCLR register.
//
//*****************************************************************************
pub const MISCLR = struct {
    pub const EOF1 = usize(0x00000200); // DMA End-of-Frame 1 Enabled
    // Interrupt and Clear
    pub const EOF0 = usize(0x00000100); // DMA End-of-Frame 0 Raw Interrupt
    // and Clear
    pub const PALLOAD = usize(0x00000040); // DMA Palette Loaded Enabled
    // Interrupt and Clear
    pub const FIFOU = usize(0x00000020); // DMA FIFO Underflow Enabled
    // Interrupt and Clear
    pub const ACBS = usize(0x00000008); // AC Bias Count Enabled Interrupt
    // and Clear
    pub const SYNCS = usize(0x00000004); // Frame Synchronization Lost
    // Enabled Interrupt and Clear
    pub const RRASTRDONE = usize(0x00000002); // Raster Mode Frame Done interrupt
    pub const DONE = usize(0x00000001); // Raster or LIDD Frame Done
    // (shared, depends on whether
    // Raster or LIDD mode enabled)
    // Enabled Interrupt and Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_IM register.
//
//*****************************************************************************
pub const IM = struct {
    pub const EOF1 = usize(0x00000200); // DMA End-of-Frame 1 Interrupt
    // Enable Set
    pub const EOF0 = usize(0x00000100); // DMA End-of-Frame 0 Interrupt
    // Enable Set
    pub const PALLOAD = usize(0x00000040); // DMA Palette Loaded Interrupt
    // Enable Set
    pub const FIFOU = usize(0x00000020); // DMA FIFO Underflow Interrupt
    // Enable Set
    pub const ACBS = usize(0x00000008); // AC Bias Count Interrupt Enable
    // Set
    pub const SYNCS = usize(0x00000004); // Frame Synchronization Lost
    // Interrupt Enable Set
    pub const RRASTRDONE = usize(0x00000002); // Raster Mode Frame Done Interrupt
    // Enable Set
    pub const DONE = usize(0x00000001); // Raster or LIDD Frame Done
    // (shared, depends on whether
    // Raster or LIDD mode enabled)
    // Interrupt Enable Set
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_IENC register.
//
//*****************************************************************************
pub const IENC = struct {
    pub const EOF1 = usize(0x00000200); // DMA End-of-Frame 1 Interrupt
    // Enable Clear
    pub const EOF0 = usize(0x00000100); // DMA End-of-Frame 0 Interrupt
    // Enable Clear
    pub const PALLOAD = usize(0x00000040); // DMA Palette Loaded Interrupt
    // Enable Clear
    pub const FIFOU = usize(0x00000020); // DMA FIFO Underflow Interrupt
    // Enable Clear
    pub const ACBS = usize(0x00000008); // AC Bias Count Interrupt Enable
    // Clear
    pub const SYNCS = usize(0x00000004); // Frame Synchronization Lost
    // Interrupt Enable Clear
    pub const RRASTRDONE = usize(0x00000002); // Raster Mode Frame Done Interrupt
    // Enable Clear
    pub const DONE = usize(0x00000001); // Raster or LIDD Frame Done
    // (shared, depends on whether
    // Raster or LIDD mode enabled)
    // Interrupt Enable Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_CLKEN register.
//
//*****************************************************************************
pub const CLKEN = struct {
    pub const DMA = usize(0x00000004); // DMA Clock Enable
    pub const LIDD = usize(0x00000002); // LIDD Submodule Clock Enable
    pub const CORE = usize(0x00000001); // LCD Core Clock Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the LCD_O_CLKRESET register.
//
//*****************************************************************************
pub const CLKRESET = struct {
    pub const MAIN = usize(0x00000008); // Software Reset for the entire
    // LCD module
    pub const DMA = usize(0x00000004); // Software Reset for the DMA
    // submodule
    pub const LIDD = usize(0x00000002); // Software Reset for the LIDD
    // submodule (character displays)
    pub const CORE = usize(0x00000001); // Software Reset for the Core,
    // which encompasses the Raster
    // Active Matrix and Passive Matrix
    // logic
};
