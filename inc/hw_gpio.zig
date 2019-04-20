//*****************************************************************************
//
// hw_gpio.h - Defines and Macros for GPIO hardware.
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
// The following are defines for the GPIO register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const DATA = usize(0x00000000); // GPIO Data
    pub const DIR = usize(0x00000400); // GPIO Direction
    pub const IS = usize(0x00000404); // GPIO Interrupt Sense
    pub const IBE = usize(0x00000408); // GPIO Interrupt Both Edges
    pub const IEV = usize(0x0000040C); // GPIO Interrupt Event
    pub const IM = usize(0x00000410); // GPIO Interrupt Mask
    pub const RIS = usize(0x00000414); // GPIO Raw Interrupt Status
    pub const MIS = usize(0x00000418); // GPIO Masked Interrupt Status
    pub const ICR = usize(0x0000041C); // GPIO Interrupt Clear
    pub const AFSEL = usize(0x00000420); // GPIO Alternate Function Select
    pub const DR2R = usize(0x00000500); // GPIO 2-mA Drive Select
    pub const DR4R = usize(0x00000504); // GPIO 4-mA Drive Select
    pub const DR8R = usize(0x00000508); // GPIO 8-mA Drive Select
    pub const ODR = usize(0x0000050C); // GPIO Open Drain Select
    pub const PUR = usize(0x00000510); // GPIO Pull-Up Select
    pub const PDR = usize(0x00000514); // GPIO Pull-Down Select
    pub const SLR = usize(0x00000518); // GPIO Slew Rate Control Select
    pub const DEN = usize(0x0000051C); // GPIO Digital Enable
    pub const LOCK = usize(0x00000520); // GPIO Lock
    pub const CR = usize(0x00000524); // GPIO Commit
    pub const AMSEL = usize(0x00000528); // GPIO Analog Mode Select
    pub const PCTL = usize(0x0000052C); // GPIO Port Control
    pub const ADCCTL = usize(0x00000530); // GPIO ADC Control
    pub const DMACTL = usize(0x00000534); // GPIO DMA Control
    pub const SI = usize(0x00000538); // GPIO Select Interrupt
    pub const DR12R = usize(0x0000053C); // GPIO 12-mA Drive Select
    pub const WAKEPEN = usize(0x00000540); // GPIO Wake Pin Enable
    pub const WAKELVL = usize(0x00000544); // GPIO Wake Level
    pub const WAKESTAT = usize(0x00000548); // GPIO Wake Status
    pub const PP = usize(0x00000FC0); // GPIO Peripheral Property
    pub const PC = usize(0x00000FC4); // GPIO Peripheral Configuration
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_IM register.
//
//*****************************************************************************
pub const IM = struct {
    pub const DMAIME = usize(0x00000100); // GPIO uDMA Done Interrupt Mask Enable
    pub const GPIO_M = usize(0x000000FF); // GPIO Interrupt Mask Enable
    pub const GPIO_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_RIS register.
//
//*****************************************************************************
pub const RIS = struct {
    pub const DMARIS = usize(0x00000100); // GPIO uDMA Done Interrupt Raw Status
    pub const GPIO_M = usize(0x000000FF); // GPIO Interrupt Raw Status
    pub const GPIO_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_MIS register.
//
//*****************************************************************************
pub const MIS = struct {
    pub const DMAMIS = usize(0x00000100); // GPIO uDMA Done Masked Interrupt Status
    pub const GPIO_M = usize(0x000000FF); // GPIO Masked Interrupt Status
    pub const GPIO_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_ICR register.
//
//*****************************************************************************
pub const ICR = struct {
    pub const DMIC = usize(0x00000100); // GPIO uDMA Interrupt Clear
    pub const GPIO_M = usize(0x000000FF); // GPIO Interrupt Clear
    pub const GPIO_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_LOCK register.
//
//*****************************************************************************
pub const LOCK = struct {
    pub const M = usize(0xFFFFFFFF); // GPIO Lock
    pub const UNLOCKED = usize(0x00000000); // The GPIOCR register is unlocked
    // and may be modified
    pub const LOCKED = usize(0x00000001); // The GPIOCR register is locked
    // and may not be modified
    pub const KEY = usize(0x4C4F434B); // Unlocks the GPIO_CR register
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_SI register.
//
//*****************************************************************************
pub const SI = struct {
    pub const SUM = usize(0x00000001); // Summary Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_DR12R register.
//
//*****************************************************************************
pub const DR12R = struct {
    pub const DRV12_M = usize(0x000000FF); // Output Pad 12-mA Drive Enable
    pub const DRV12_12MA = usize(0x00000001); // The corresponding GPIO pin has
    // 12-mA drive. This encoding is
    // only valid if the GPIOPP EDE bit
    // is set and the appropriate
    // GPIOPC EDM bit field is
    // programmed to 0x3
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_WAKEPEN register.
//
//*****************************************************************************
pub const WAKEPEN = struct {
    pub const WAKEP4 = usize(0x00000010); // P[4] Wake Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_WAKELVL register.
//
//*****************************************************************************
pub const WAKELVL = struct {
    pub const WAKELVL4 = usize(0x00000010); // P[4] Wake Level
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_WAKESTAT
// register.
//
//*****************************************************************************
pub const WAKESTAT = struct {
    pub const STAT4 = usize(0x00000010); // P[4] Wake Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_PP register.
//
//*****************************************************************************
pub const PP = struct {
    pub const EDE = usize(0x00000001); // Extended Drive Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_PC register.
//
//*****************************************************************************
pub const PC = struct {
    pub const EDM7_M = usize(0x0000C000); // Extended Drive Mode Bit 7
    pub const EDM6_M = usize(0x00003000); // Extended Drive Mode Bit 6
    pub const EDM5_M = usize(0x00000C00); // Extended Drive Mode Bit 5
    pub const EDM4_M = usize(0x00000300); // Extended Drive Mode Bit 4
    pub const EDM3_M = usize(0x000000C0); // Extended Drive Mode Bit 3
    pub const EDM2_M = usize(0x00000030); // Extended Drive Mode Bit 2
    pub const EDM1_M = usize(0x0000000C); // Extended Drive Mode Bit 1
    pub const EDM0_M = usize(0x00000003); // Extended Drive Mode Bit 0
    pub const EDM0_DISABLE = usize(0x00000000); // Drive values of 2, 4 and 8 mA
    // are maintained. GPIO n Drive
    // Select (GPIODRnR) registers
    // function as normal
    pub const EDM0_6MA = usize(0x00000001); // An additional 6 mA option is
    // provided
    pub const EDM0_PLUS2MA = usize(0x00000003); // A 2 mA driver is always enabled;
    // setting the corresponding
    // GPIODR4R register bit adds 2 mA
    // and setting the corresponding
    // GPIODR8R of GPIODR12R register
    // bit adds an additional 4 mA
    pub const EDM7_S = usize(14);
    pub const EDM6_S = usize(12);
    pub const EDM5_S = usize(10);
    pub const EDM4_S = usize(8);
    pub const EDM3_S = usize(6);
    pub const EDM2_S = usize(4);
    pub const EDM1_S = usize(2);
};
