//*****************************************************************************
//
// hw_sysctl.h - Macros used when accessing the system control hardware.
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
// The following are defines for the System Control register addresses.
//
//*****************************************************************************
pub const SYSCTL_DID0 = usize(0x400FE000);  // Device Identification 0
pub const SYSCTL_DID1 = usize(0x400FE004);  // Device Identification 1
pub const SYSCTL_DC0 = usize(0x400FE008);  // Device Capabilities 0
pub const SYSCTL_DC1 = usize(0x400FE010);  // Device Capabilities 1
pub const SYSCTL_DC2 = usize(0x400FE014);  // Device Capabilities 2
pub const SYSCTL_DC3 = usize(0x400FE018);  // Device Capabilities 3
pub const SYSCTL_DC4 = usize(0x400FE01C);  // Device Capabilities 4
pub const SYSCTL_DC5 = usize(0x400FE020);  // Device Capabilities 5
pub const SYSCTL_DC6 = usize(0x400FE024);  // Device Capabilities 6
pub const SYSCTL_DC7 = usize(0x400FE028);  // Device Capabilities 7
pub const SYSCTL_DC8 = usize(0x400FE02C);  // Device Capabilities 8
pub const SYSCTL_PBORCTL = usize(0x400FE030);  // Brown-Out Reset Control
pub const SYSCTL_PTBOCTL = usize(0x400FE038);  // Power-Temp Brown Out Control
pub const SYSCTL_SRCR0 = usize(0x400FE040);  // Software Reset Control 0
pub const SYSCTL_SRCR1 = usize(0x400FE044);  // Software Reset Control 1
pub const SYSCTL_SRCR2 = usize(0x400FE048);  // Software Reset Control 2
pub const SYSCTL_RIS = usize(0x400FE050);  // Raw Interrupt Status
pub const SYSCTL_IMC = usize(0x400FE054);  // Interrupt Mask Control
pub const SYSCTL_MISC = usize(0x400FE058);  // Masked Interrupt Status and
                                            // Clear
pub const SYSCTL_RESC = usize(0x400FE05C);  // Reset Cause
pub const SYSCTL_PWRTC = usize(0x400FE060);  // Power-Temperature Cause
pub const SYSCTL_RCC = usize(0x400FE060);  // Run-Mode Clock Configuration
pub const SYSCTL_NMIC = usize(0x400FE064);  // NMI Cause Register
pub const SYSCTL_GPIOHBCTL = usize(0x400FE06C);  // GPIO High-Performance Bus
                                            // Control
pub const SYSCTL_RCC2 = usize(0x400FE070);  // Run-Mode Clock Configuration 2
pub const SYSCTL_MOSCCTL = usize(0x400FE07C);  // Main Oscillator Control
pub const SYSCTL_RSCLKCFG = usize(0x400FE0B0);  // Run and Sleep Mode Configuration
                                            // Register
pub const SYSCTL_MEMTIM0 = usize(0x400FE0C0);  // Memory Timing Parameter Register
                                            // 0 for Main Flash and EEPROM
pub const SYSCTL_RCGC0 = usize(0x400FE100);  // Run Mode Clock Gating Control
                                            // Register 0
pub const SYSCTL_RCGC1 = usize(0x400FE104);  // Run Mode Clock Gating Control
                                            // Register 1
pub const SYSCTL_RCGC2 = usize(0x400FE108);  // Run Mode Clock Gating Control
                                            // Register 2
pub const SYSCTL_SCGC0 = usize(0x400FE110);  // Sleep Mode Clock Gating Control
                                            // Register 0
pub const SYSCTL_SCGC1 = usize(0x400FE114);  // Sleep Mode Clock Gating Control
                                            // Register 1
pub const SYSCTL_SCGC2 = usize(0x400FE118);  // Sleep Mode Clock Gating Control
                                            // Register 2
pub const SYSCTL_DCGC0 = usize(0x400FE120);  // Deep Sleep Mode Clock Gating
                                            // Control Register 0
pub const SYSCTL_DCGC1 = usize(0x400FE124);  // Deep-Sleep Mode Clock Gating
                                            // Control Register 1
pub const SYSCTL_DCGC2 = usize(0x400FE128);  // Deep Sleep Mode Clock Gating
                                            // Control Register 2
pub const SYSCTL_ALTCLKCFG = usize(0x400FE138);  // Alternate Clock Configuration
pub const SYSCTL_DSLPCLKCFG = usize(0x400FE144);  // Deep Sleep Clock Configuration
pub const SYSCTL_DSCLKCFG = usize(0x400FE144);  // Deep Sleep Clock Configuration
                                            // Register
pub const SYSCTL_DIVSCLK = usize(0x400FE148);  // Divisor and Source Clock
                                            // Configuration
pub const SYSCTL_SYSPROP = usize(0x400FE14C);  // System Properties
pub const SYSCTL_PIOSCCAL = usize(0x400FE150);  // Precision Internal Oscillator
                                            // Calibration
pub const SYSCTL_PIOSCSTAT = usize(0x400FE154);  // Precision Internal Oscillator
                                            // Statistics
pub const SYSCTL_PLLFREQ0 = usize(0x400FE160);  // PLL Frequency 0
pub const SYSCTL_PLLFREQ1 = usize(0x400FE164);  // PLL Frequency 1
pub const SYSCTL_PLLSTAT = usize(0x400FE168);  // PLL Status
pub const SYSCTL_SLPPWRCFG = usize(0x400FE188);  // Sleep Power Configuration
pub const SYSCTL_DSLPPWRCFG = usize(0x400FE18C);  // Deep-Sleep Power Configuration
pub const SYSCTL_DC9 = usize(0x400FE190);  // Device Capabilities 9
pub const SYSCTL_NVMSTAT = usize(0x400FE1A0);  // Non-Volatile Memory Information
pub const SYSCTL_LDOSPCTL = usize(0x400FE1B4);  // LDO Sleep Power Control
pub const SYSCTL_LDODPCTL = usize(0x400FE1BC);  // LDO Deep-Sleep Power Control
pub const SYSCTL_RESBEHAVCTL = usize(0x400FE1D8);  // Reset Behavior Control Register
pub const SYSCTL_HSSR = usize(0x400FE1F4);  // Hardware System Service Request
pub const SYSCTL_USBPDS = usize(0x400FE280);  // USB Power Domain Status
pub const SYSCTL_USBMPC = usize(0x400FE284);  // USB Memory Power Control
pub const SYSCTL_EMACPDS = usize(0x400FE288);  // Ethernet MAC Power Domain Status
pub const SYSCTL_EMACMPC = usize(0x400FE28C);  // Ethernet MAC Memory Power
                                            // Control
pub const SYSCTL_LCDMPC = usize(0x400FE294);  // LCD Memory Power Control
pub const SYSCTL_PPWD = usize(0x400FE300);  // Watchdog Timer Peripheral
                                            // Present
pub const SYSCTL_PPTIMER = usize(0x400FE304);  // 16/32-Bit General-Purpose Timer
                                            // Peripheral Present
pub const SYSCTL_PPGPIO = usize(0x400FE308);  // General-Purpose Input/Output
                                            // Peripheral Present
pub const SYSCTL_PPDMA = usize(0x400FE30C);  // Micro Direct Memory Access
                                            // Peripheral Present
pub const SYSCTL_PPEPI = usize(0x400FE310);  // EPI Peripheral Present
pub const SYSCTL_PPHIB = usize(0x400FE314);  // Hibernation Peripheral Present
pub const SYSCTL_PPUART = usize(0x400FE318);  // Universal Asynchronous
                                            // Receiver/Transmitter Peripheral
                                            // Present
pub const SYSCTL_PPSSI = usize(0x400FE31C);  // Synchronous Serial Interface
                                            // Peripheral Present
pub const SYSCTL_PPI2C = usize(0x400FE320);  // Inter-Integrated Circuit
                                            // Peripheral Present
pub const SYSCTL_PPUSB = usize(0x400FE328);  // Universal Serial Bus Peripheral
                                            // Present
pub const SYSCTL_PPEPHY = usize(0x400FE330);  // Ethernet PHY Peripheral Present
pub const SYSCTL_PPCAN = usize(0x400FE334);  // Controller Area Network
                                            // Peripheral Present
pub const SYSCTL_PPADC = usize(0x400FE338);  // Analog-to-Digital Converter
                                            // Peripheral Present
pub const SYSCTL_PPACMP = usize(0x400FE33C);  // Analog Comparator Peripheral
                                            // Present
pub const SYSCTL_PPPWM = usize(0x400FE340);  // Pulse Width Modulator Peripheral
                                            // Present
pub const SYSCTL_PPQEI = usize(0x400FE344);  // Quadrature Encoder Interface
                                            // Peripheral Present
pub const SYSCTL_PPLPC = usize(0x400FE348);  // Low Pin Count Interface
                                            // Peripheral Present
pub const SYSCTL_PPPECI = usize(0x400FE350);  // Platform Environment Control
                                            // Interface Peripheral Present
pub const SYSCTL_PPFAN = usize(0x400FE354);  // Fan Control Peripheral Present
pub const SYSCTL_PPEEPROM = usize(0x400FE358);  // EEPROM Peripheral Present
pub const SYSCTL_PPWTIMER = usize(0x400FE35C);  // 32/64-Bit Wide General-Purpose
                                            // Timer Peripheral Present
pub const SYSCTL_PPRTS = usize(0x400FE370);  // Remote Temperature Sensor
                                            // Peripheral Present
pub const SYSCTL_PPCCM = usize(0x400FE374);  // CRC and Cryptographic Modules
                                            // Peripheral Present
pub const SYSCTL_PPLCD = usize(0x400FE390);  // LCD Peripheral Present
pub const SYSCTL_PPOWIRE = usize(0x400FE398);  // 1-Wire Peripheral Present
pub const SYSCTL_PPEMAC = usize(0x400FE39C);  // Ethernet MAC Peripheral Present
pub const SYSCTL_PPHIM = usize(0x400FE3A4);  // Human Interface Master
                                            // Peripheral Present
pub const SYSCTL_SRWD = usize(0x400FE500);  // Watchdog Timer Software Reset
pub const SYSCTL_SRTIMER = usize(0x400FE504);  // 16/32-Bit General-Purpose Timer
                                            // Software Reset
pub const SYSCTL_SRGPIO = usize(0x400FE508);  // General-Purpose Input/Output
                                            // Software Reset
pub const SYSCTL_SRDMA = usize(0x400FE50C);  // Micro Direct Memory Access
                                            // Software Reset
pub const SYSCTL_SREPI = usize(0x400FE510);  // EPI Software Reset
pub const SYSCTL_SRHIB = usize(0x400FE514);  // Hibernation Software Reset
pub const SYSCTL_SRUART = usize(0x400FE518);  // Universal Asynchronous
                                            // Receiver/Transmitter Software
                                            // Reset
pub const SYSCTL_SRSSI = usize(0x400FE51C);  // Synchronous Serial Interface
                                            // Software Reset
pub const SYSCTL_SRI2C = usize(0x400FE520);  // Inter-Integrated Circuit
                                            // Software Reset
pub const SYSCTL_SRUSB = usize(0x400FE528);  // Universal Serial Bus Software
                                            // Reset
pub const SYSCTL_SREPHY = usize(0x400FE530);  // Ethernet PHY Software Reset
pub const SYSCTL_SRCAN = usize(0x400FE534);  // Controller Area Network Software
                                            // Reset
pub const SYSCTL_SRADC = usize(0x400FE538);  // Analog-to-Digital Converter
                                            // Software Reset
pub const SYSCTL_SRACMP = usize(0x400FE53C);  // Analog Comparator Software Reset
pub const SYSCTL_SRPWM = usize(0x400FE540);  // Pulse Width Modulator Software
                                            // Reset
pub const SYSCTL_SRQEI = usize(0x400FE544);  // Quadrature Encoder Interface
                                            // Software Reset
pub const SYSCTL_SREEPROM = usize(0x400FE558);  // EEPROM Software Reset
pub const SYSCTL_SRWTIMER = usize(0x400FE55C);  // 32/64-Bit Wide General-Purpose
                                            // Timer Software Reset
pub const SYSCTL_SRCCM = usize(0x400FE574);  // CRC and Cryptographic Modules
                                            // Software Reset
pub const SYSCTL_SRLCD = usize(0x400FE590);  // LCD Controller Software Reset
pub const SYSCTL_SROWIRE = usize(0x400FE598);  // 1-Wire Software Reset
pub const SYSCTL_SREMAC = usize(0x400FE59C);  // Ethernet MAC Software Reset
pub const SYSCTL_RCGCWD = usize(0x400FE600);  // Watchdog Timer Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCTIMER = usize(0x400FE604);  // 16/32-Bit General-Purpose Timer
                                            // Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO = usize(0x400FE608);  // General-Purpose Input/Output Run
                                            // Mode Clock Gating Control
pub const SYSCTL_RCGCDMA = usize(0x400FE60C);  // Micro Direct Memory Access Run
                                            // Mode Clock Gating Control
pub const SYSCTL_RCGCEPI = usize(0x400FE610);  // EPI Run Mode Clock Gating
                                            // Control
pub const SYSCTL_RCGCHIB = usize(0x400FE614);  // Hibernation Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCUART = usize(0x400FE618);  // Universal Asynchronous
                                            // Receiver/Transmitter Run Mode
                                            // Clock Gating Control
pub const SYSCTL_RCGCSSI = usize(0x400FE61C);  // Synchronous Serial Interface Run
                                            // Mode Clock Gating Control
pub const SYSCTL_RCGCI2C = usize(0x400FE620);  // Inter-Integrated Circuit Run
                                            // Mode Clock Gating Control
pub const SYSCTL_RCGCUSB = usize(0x400FE628);  // Universal Serial Bus Run Mode
                                            // Clock Gating Control
pub const SYSCTL_RCGCEPHY = usize(0x400FE630);  // Ethernet PHY Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCCAN = usize(0x400FE634);  // Controller Area Network Run Mode
                                            // Clock Gating Control
pub const SYSCTL_RCGCADC = usize(0x400FE638);  // Analog-to-Digital Converter Run
                                            // Mode Clock Gating Control
pub const SYSCTL_RCGCACMP = usize(0x400FE63C);  // Analog Comparator Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCPWM = usize(0x400FE640);  // Pulse Width Modulator Run Mode
                                            // Clock Gating Control
pub const SYSCTL_RCGCQEI = usize(0x400FE644);  // Quadrature Encoder Interface Run
                                            // Mode Clock Gating Control
pub const SYSCTL_RCGCEEPROM = usize(0x400FE658);  // EEPROM Run Mode Clock Gating
                                            // Control
pub const SYSCTL_RCGCWTIMER = usize(0x400FE65C);  // 32/64-Bit Wide General-Purpose
                                            // Timer Run Mode Clock Gating
                                            // Control
pub const SYSCTL_RCGCCCM = usize(0x400FE674);  // CRC and Cryptographic Modules
                                            // Run Mode Clock Gating Control
pub const SYSCTL_RCGCLCD = usize(0x400FE690);  // LCD Controller Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCOWIRE = usize(0x400FE698);  // 1-Wire Run Mode Clock Gating
                                            // Control
pub const SYSCTL_RCGCEMAC = usize(0x400FE69C);  // Ethernet MAC Run Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCWD = usize(0x400FE700);  // Watchdog Timer Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCTIMER = usize(0x400FE704);  // 16/32-Bit General-Purpose Timer
                                            // Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO = usize(0x400FE708);  // General-Purpose Input/Output
                                            // Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCDMA = usize(0x400FE70C);  // Micro Direct Memory Access Sleep
                                            // Mode Clock Gating Control
pub const SYSCTL_SCGCEPI = usize(0x400FE710);  // EPI Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCHIB = usize(0x400FE714);  // Hibernation Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCUART = usize(0x400FE718);  // Universal Asynchronous
                                            // Receiver/Transmitter Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_SCGCSSI = usize(0x400FE71C);  // Synchronous Serial Interface
                                            // Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCI2C = usize(0x400FE720);  // Inter-Integrated Circuit Sleep
                                            // Mode Clock Gating Control
pub const SYSCTL_SCGCUSB = usize(0x400FE728);  // Universal Serial Bus Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_SCGCEPHY = usize(0x400FE730);  // Ethernet PHY Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCCAN = usize(0x400FE734);  // Controller Area Network Sleep
                                            // Mode Clock Gating Control
pub const SYSCTL_SCGCADC = usize(0x400FE738);  // Analog-to-Digital Converter
                                            // Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCACMP = usize(0x400FE73C);  // Analog Comparator Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_SCGCPWM = usize(0x400FE740);  // Pulse Width Modulator Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_SCGCQEI = usize(0x400FE744);  // Quadrature Encoder Interface
                                            // Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCEEPROM = usize(0x400FE758);  // EEPROM Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCWTIMER = usize(0x400FE75C);  // 32/64-Bit Wide General-Purpose
                                            // Timer Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCCCM = usize(0x400FE774);  // CRC and Cryptographic Modules
                                            // Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCLCD = usize(0x400FE790);  // LCD Controller Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCOWIRE = usize(0x400FE798);  // 1-Wire Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCEMAC = usize(0x400FE79C);  // Ethernet MAC Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_DCGCWD = usize(0x400FE800);  // Watchdog Timer Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCTIMER = usize(0x400FE804);  // 16/32-Bit General-Purpose Timer
                                            // Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCGPIO = usize(0x400FE808);  // General-Purpose Input/Output
                                            // Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCDMA = usize(0x400FE80C);  // Micro Direct Memory Access
                                            // Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCEPI = usize(0x400FE810);  // EPI Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCHIB = usize(0x400FE814);  // Hibernation Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCUART = usize(0x400FE818);  // Universal Asynchronous
                                            // Receiver/Transmitter Deep-Sleep
                                            // Mode Clock Gating Control
pub const SYSCTL_DCGCSSI = usize(0x400FE81C);  // Synchronous Serial Interface
                                            // Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCI2C = usize(0x400FE820);  // Inter-Integrated Circuit
                                            // Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCUSB = usize(0x400FE828);  // Universal Serial Bus Deep-Sleep
                                            // Mode Clock Gating Control
pub const SYSCTL_DCGCEPHY = usize(0x400FE830);  // Ethernet PHY Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCCAN = usize(0x400FE834);  // Controller Area Network
                                            // Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCADC = usize(0x400FE838);  // Analog-to-Digital Converter
                                            // Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCACMP = usize(0x400FE83C);  // Analog Comparator Deep-Sleep
                                            // Mode Clock Gating Control
pub const SYSCTL_DCGCPWM = usize(0x400FE840);  // Pulse Width Modulator Deep-Sleep
                                            // Mode Clock Gating Control
pub const SYSCTL_DCGCQEI = usize(0x400FE844);  // Quadrature Encoder Interface
                                            // Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCEEPROM = usize(0x400FE858);  // EEPROM Deep-Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_DCGCWTIMER = usize(0x400FE85C);  // 32/64-Bit Wide General-Purpose
                                            // Timer Deep-Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_DCGCCCM = usize(0x400FE874);  // CRC and Cryptographic Modules
                                            // Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCLCD = usize(0x400FE890);  // LCD Controller Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCOWIRE = usize(0x400FE898);  // 1-Wire Deep-Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_DCGCEMAC = usize(0x400FE89C);  // Ethernet MAC Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_PCWD = usize(0x400FE900);  // Watchdog Timer Power Control
pub const SYSCTL_PCTIMER = usize(0x400FE904);  // 16/32-Bit General-Purpose Timer
                                            // Power Control
pub const SYSCTL_PCGPIO = usize(0x400FE908);  // General-Purpose Input/Output
                                            // Power Control
pub const SYSCTL_PCDMA = usize(0x400FE90C);  // Micro Direct Memory Access Power
                                            // Control
pub const SYSCTL_PCEPI = usize(0x400FE910);  // External Peripheral Interface
                                            // Power Control
pub const SYSCTL_PCHIB = usize(0x400FE914);  // Hibernation Power Control
pub const SYSCTL_PCUART = usize(0x400FE918);  // Universal Asynchronous
                                            // Receiver/Transmitter Power
                                            // Control
pub const SYSCTL_PCSSI = usize(0x400FE91C);  // Synchronous Serial Interface
                                            // Power Control
pub const SYSCTL_PCI2C = usize(0x400FE920);  // Inter-Integrated Circuit Power
                                            // Control
pub const SYSCTL_PCUSB = usize(0x400FE928);  // Universal Serial Bus Power
                                            // Control
pub const SYSCTL_PCEPHY = usize(0x400FE930);  // Ethernet PHY Power Control
pub const SYSCTL_PCCAN = usize(0x400FE934);  // Controller Area Network Power
                                            // Control
pub const SYSCTL_PCADC = usize(0x400FE938);  // Analog-to-Digital Converter
                                            // Power Control
pub const SYSCTL_PCACMP = usize(0x400FE93C);  // Analog Comparator Power Control
pub const SYSCTL_PCPWM = usize(0x400FE940);  // Pulse Width Modulator Power
                                            // Control
pub const SYSCTL_PCQEI = usize(0x400FE944);  // Quadrature Encoder Interface
                                            // Power Control
pub const SYSCTL_PCEEPROM = usize(0x400FE958);  // EEPROM Power Control
pub const SYSCTL_PCCCM = usize(0x400FE974);  // CRC and Cryptographic Modules
                                            // Power Control
pub const SYSCTL_PCLCD = usize(0x400FE990);  // LCD Controller Power Control
pub const SYSCTL_PCOWIRE = usize(0x400FE998);  // 1-Wire Power Control
pub const SYSCTL_PCEMAC = usize(0x400FE99C);  // Ethernet MAC Power Control
pub const SYSCTL_PRWD = usize(0x400FEA00);  // Watchdog Timer Peripheral Ready
pub const SYSCTL_PRTIMER = usize(0x400FEA04);  // 16/32-Bit General-Purpose Timer
                                            // Peripheral Ready
pub const SYSCTL_PRGPIO = usize(0x400FEA08);  // General-Purpose Input/Output
                                            // Peripheral Ready
pub const SYSCTL_PRDMA = usize(0x400FEA0C);  // Micro Direct Memory Access
                                            // Peripheral Ready
pub const SYSCTL_PREPI = usize(0x400FEA10);  // EPI Peripheral Ready
pub const SYSCTL_PRHIB = usize(0x400FEA14);  // Hibernation Peripheral Ready
pub const SYSCTL_PRUART = usize(0x400FEA18);  // Universal Asynchronous
                                            // Receiver/Transmitter Peripheral
                                            // Ready
pub const SYSCTL_PRSSI = usize(0x400FEA1C);  // Synchronous Serial Interface
                                            // Peripheral Ready
pub const SYSCTL_PRI2C = usize(0x400FEA20);  // Inter-Integrated Circuit
                                            // Peripheral Ready
pub const SYSCTL_PRUSB = usize(0x400FEA28);  // Universal Serial Bus Peripheral
                                            // Ready
pub const SYSCTL_PREPHY = usize(0x400FEA30);  // Ethernet PHY Peripheral Ready
pub const SYSCTL_PRCAN = usize(0x400FEA34);  // Controller Area Network
                                            // Peripheral Ready
pub const SYSCTL_PRADC = usize(0x400FEA38);  // Analog-to-Digital Converter
                                            // Peripheral Ready
pub const SYSCTL_PRACMP = usize(0x400FEA3C);  // Analog Comparator Peripheral
                                            // Ready
pub const SYSCTL_PRPWM = usize(0x400FEA40);  // Pulse Width Modulator Peripheral
                                            // Ready
pub const SYSCTL_PRQEI = usize(0x400FEA44);  // Quadrature Encoder Interface
                                            // Peripheral Ready
pub const SYSCTL_PREEPROM = usize(0x400FEA58);  // EEPROM Peripheral Ready
pub const SYSCTL_PRWTIMER = usize(0x400FEA5C);  // 32/64-Bit Wide General-Purpose
                                            // Timer Peripheral Ready
pub const SYSCTL_PRCCM = usize(0x400FEA74);  // CRC and Cryptographic Modules
                                            // Peripheral Ready
pub const SYSCTL_PRLCD = usize(0x400FEA90);  // LCD Controller Peripheral Ready
pub const SYSCTL_PROWIRE = usize(0x400FEA98);  // 1-Wire Peripheral Ready
pub const SYSCTL_PREMAC = usize(0x400FEA9C);  // Ethernet MAC Peripheral Ready
pub const SYSCTL_UNIQUEID0 = usize(0x400FEF20);  // Unique ID 0
pub const SYSCTL_UNIQUEID1 = usize(0x400FEF24);  // Unique ID 1
pub const SYSCTL_UNIQUEID2 = usize(0x400FEF28);  // Unique ID 2
pub const SYSCTL_UNIQUEID3 = usize(0x400FEF2C);  // Unique ID 3
pub const SYSCTL_CCMCGREQ = usize(0x44030204);  // Cryptographic Modules Clock
                                            // Gating Request

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DID0 register.
//
//*****************************************************************************
pub const SYSCTL_DID0_VER_M = usize(0x70000000);  // DID0 Version
pub const SYSCTL_DID0_VER_1 = usize(0x10000000);  // Second version of the DID0
                                            // register format.
pub const SYSCTL_DID0_CLASS_M = usize(0x00FF0000);  // Device Class
pub const SYSCTL_DID0_CLASS_TM4C123 = usize(0x00050000);  // Tiva TM4C123x and TM4E123x
                                            // microcontrollers
pub const SYSCTL_DID0_CLASS_TM4C129 = usize(0x000A0000);  // Tiva(TM) TM4C129-class
                                            // microcontrollers
pub const SYSCTL_DID0_MAJ_M = usize(0x0000FF00);  // Major Revision
pub const SYSCTL_DID0_MAJ_REVA = usize(0x00000000);  // Revision A (initial device)
pub const SYSCTL_DID0_MAJ_REVB = usize(0x00000100);  // Revision B (first base layer
                                            // revision)
pub const SYSCTL_DID0_MAJ_REVC = usize(0x00000200);  // Revision C (second base layer
                                            // revision)
pub const SYSCTL_DID0_MIN_M = usize(0x000000FF);  // Minor Revision
pub const SYSCTL_DID0_MIN_0 = usize(0x00000000);  // Initial device, or a major
                                            // revision update
pub const SYSCTL_DID0_MIN_1 = usize(0x00000001);  // First metal layer change
pub const SYSCTL_DID0_MIN_2 = usize(0x00000002);  // Second metal layer change

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DID1 register.
//
//*****************************************************************************
pub const SYSCTL_DID1_VER_M = usize(0xF0000000);  // DID1 Version
pub const SYSCTL_DID1_VER_1 = usize(0x10000000);  // fury_ib
pub const SYSCTL_DID1_FAM_M = usize(0x0F000000);  // Family
pub const SYSCTL_DID1_FAM_TIVA = usize(0x00000000);  // Tiva family of microcontollers
pub const SYSCTL_DID1_PRTNO_M = usize(0x00FF0000);  // Part Number
pub const SYSCTL_DID1_PRTNO_TM4C1230C3PM = usize(0x00220000);  // TM4C1230C3PM
pub const SYSCTL_DID1_PRTNO_TM4C1230D5PM = usize(0x00230000);  // TM4C1230D5PM
pub const SYSCTL_DID1_PRTNO_TM4C1230E6PM = usize(0x00200000);  // TM4C1230E6PM
pub const SYSCTL_DID1_PRTNO_TM4C1230H6PM = usize(0x00210000);  // TM4C1230H6PM
pub const SYSCTL_DID1_PRTNO_TM4C1231C3PM = usize(0x00180000);  // TM4C1231C3PM
pub const SYSCTL_DID1_PRTNO_TM4C1231D5PM = usize(0x00190000);  // TM4C1231D5PM
pub const SYSCTL_DID1_PRTNO_TM4C1231D5PZ = usize(0x00360000);  // TM4C1231D5PZ
pub const SYSCTL_DID1_PRTNO_TM4C1231E6PM = usize(0x00100000);  // TM4C1231E6PM
pub const SYSCTL_DID1_PRTNO_TM4C1231E6PZ = usize(0x00300000);  // TM4C1231E6PZ
pub const SYSCTL_DID1_PRTNO_TM4C1231H6PGE = usize(0x00350000);  // TM4C1231H6PGE
pub const SYSCTL_DID1_PRTNO_TM4C1231H6PM = usize(0x00110000);  // TM4C1231H6PM
pub const SYSCTL_DID1_PRTNO_TM4C1231H6PZ = usize(0x00310000);  // TM4C1231H6PZ
pub const SYSCTL_DID1_PRTNO_TM4C1232C3PM = usize(0x00080000);  // TM4C1232C3PM
pub const SYSCTL_DID1_PRTNO_TM4C1232D5PM = usize(0x00090000);  // TM4C1232D5PM
pub const SYSCTL_DID1_PRTNO_TM4C1232E6PM = usize(0x000A0000);  // TM4C1232E6PM
pub const SYSCTL_DID1_PRTNO_TM4C1232H6PM = usize(0x000B0000);  // TM4C1232H6PM
pub const SYSCTL_DID1_PRTNO_TM4C1233C3PM = usize(0x00010000);  // TM4C1233C3PM
pub const SYSCTL_DID1_PRTNO_TM4C1233D5PM = usize(0x00020000);  // TM4C1233D5PM
pub const SYSCTL_DID1_PRTNO_TM4C1233D5PZ = usize(0x00D00000);  // TM4C1233D5PZ
pub const SYSCTL_DID1_PRTNO_TM4C1233E6PM = usize(0x00030000);  // TM4C1233E6PM
pub const SYSCTL_DID1_PRTNO_TM4C1233E6PZ = usize(0x00D10000);  // TM4C1233E6PZ
pub const SYSCTL_DID1_PRTNO_TM4C1233H6PGE = usize(0x00D60000);  // TM4C1233H6PGE
pub const SYSCTL_DID1_PRTNO_TM4C1233H6PM = usize(0x00040000);  // TM4C1233H6PM
pub const SYSCTL_DID1_PRTNO_TM4C1233H6PZ = usize(0x00D20000);  // TM4C1233H6PZ
pub const SYSCTL_DID1_PRTNO_TM4C1236D5PM = usize(0x00520000);  // TM4C1236D5PM
pub const SYSCTL_DID1_PRTNO_TM4C1236E6PM = usize(0x00500000);  // TM4C1236E6PM
pub const SYSCTL_DID1_PRTNO_TM4C1236H6PM = usize(0x00510000);  // TM4C1236H6PM
pub const SYSCTL_DID1_PRTNO_TM4C1237D5PM = usize(0x00480000);  // TM4C1237D5PM
pub const SYSCTL_DID1_PRTNO_TM4C1237D5PZ = usize(0x00660000);  // TM4C1237D5PZ
pub const SYSCTL_DID1_PRTNO_TM4C1237E6PM = usize(0x00400000);  // TM4C1237E6PM
pub const SYSCTL_DID1_PRTNO_TM4C1237E6PZ = usize(0x00600000);  // TM4C1237E6PZ
pub const SYSCTL_DID1_PRTNO_TM4C1237H6PGE = usize(0x00650000);  // TM4C1237H6PGE
pub const SYSCTL_DID1_PRTNO_TM4C1237H6PM = usize(0x00410000);  // TM4C1237H6PM
pub const SYSCTL_DID1_PRTNO_TM4C1237H6PZ = usize(0x00610000);  // TM4C1237H6PZ
pub const SYSCTL_DID1_PRTNO_TM4C123AE6PM = usize(0x00800000);  // TM4C123AE6PM
pub const SYSCTL_DID1_PRTNO_TM4C123AH6PM = usize(0x00830000);  // TM4C123AH6PM
pub const SYSCTL_DID1_PRTNO_TM4C123BE6PM = usize(0x00700000);  // TM4C123BE6PM
pub const SYSCTL_DID1_PRTNO_TM4C123BE6PZ = usize(0x00C30000);  // TM4C123BE6PZ
pub const SYSCTL_DID1_PRTNO_TM4C123BH6PGE = usize(0x00C60000);  // TM4C123BH6PGE
pub const SYSCTL_DID1_PRTNO_TM4C123BH6PM = usize(0x00730000);  // TM4C123BH6PM
pub const SYSCTL_DID1_PRTNO_TM4C123BH6PZ = usize(0x00C40000);  // TM4C123BH6PZ
pub const SYSCTL_DID1_PRTNO_TM4C123BH6ZRB = usize(0x00E90000);  // TM4C123BH6ZRB
pub const SYSCTL_DID1_PRTNO_TM4C123FE6PM = usize(0x00B00000);  // TM4C123FE6PM
pub const SYSCTL_DID1_PRTNO_TM4C123FH6PM = usize(0x00B10000);  // TM4C123FH6PM
pub const SYSCTL_DID1_PRTNO_TM4C123GE6PM = usize(0x00A00000);  // TM4C123GE6PM
pub const SYSCTL_DID1_PRTNO_TM4C123GE6PZ = usize(0x00C00000);  // TM4C123GE6PZ
pub const SYSCTL_DID1_PRTNO_TM4C123GH6PGE = usize(0x00C50000);  // TM4C123GH6PGE
pub const SYSCTL_DID1_PRTNO_TM4C123GH6PM = usize(0x00A10000);  // TM4C123GH6PM
pub const SYSCTL_DID1_PRTNO_TM4C123GH6PZ = usize(0x00C10000);  // TM4C123GH6PZ
pub const SYSCTL_DID1_PRTNO_TM4C123GH6ZRB = usize(0x00E30000);  // TM4C123GH6ZRB
pub const SYSCTL_DID1_PRTNO_TM4C1290NCPDT = usize(0x00190000);  // TM4C1290NCPDT
pub const SYSCTL_DID1_PRTNO_TM4C1290NCZAD = usize(0x001B0000);  // TM4C1290NCZAD
pub const SYSCTL_DID1_PRTNO_TM4C1292NCPDT = usize(0x001C0000);  // TM4C1292NCPDT
pub const SYSCTL_DID1_PRTNO_TM4C1292NCZAD = usize(0x001E0000);  // TM4C1292NCZAD
pub const SYSCTL_DID1_PRTNO_TM4C1294KCPDT = usize(0x00340000);  // TM4C1294KCPDT
pub const SYSCTL_DID1_PRTNO_TM4C1294NCPDT = usize(0x001F0000);  // TM4C1294NCPDT
pub const SYSCTL_DID1_PRTNO_TM4C1294NCZAD = usize(0x00210000);  // TM4C1294NCZAD
pub const SYSCTL_DID1_PRTNO_TM4C1297NCZAD = usize(0x00220000);  // TM4C1297NCZAD
pub const SYSCTL_DID1_PRTNO_TM4C1299KCZAD = usize(0x00360000);  // TM4C1299KCZAD
pub const SYSCTL_DID1_PRTNO_TM4C1299NCZAD = usize(0x00230000);  // TM4C1299NCZAD
pub const SYSCTL_DID1_PRTNO_TM4C129CNCPDT = usize(0x00240000);  // TM4C129CNCPDT
pub const SYSCTL_DID1_PRTNO_TM4C129CNCZAD = usize(0x00260000);  // TM4C129CNCZAD
pub const SYSCTL_DID1_PRTNO_TM4C129DNCPDT = usize(0x00270000);  // TM4C129DNCPDT
pub const SYSCTL_DID1_PRTNO_TM4C129DNCZAD = usize(0x00290000);  // TM4C129DNCZAD
pub const SYSCTL_DID1_PRTNO_TM4C129EKCPDT = usize(0x00350000);  // TM4C129EKCPDT
pub const SYSCTL_DID1_PRTNO_TM4C129ENCPDT = usize(0x002D0000);  // TM4C129ENCPDT
pub const SYSCTL_DID1_PRTNO_TM4C129ENCZAD = usize(0x002F0000);  // TM4C129ENCZAD
pub const SYSCTL_DID1_PRTNO_TM4C129LNCZAD = usize(0x00300000);  // TM4C129LNCZAD
pub const SYSCTL_DID1_PRTNO_TM4C129XKCZAD = usize(0x00370000);  // TM4C129XKCZAD
pub const SYSCTL_DID1_PRTNO_TM4C129XNCZAD = usize(0x00320000);  // TM4C129XNCZAD
pub const SYSCTL_DID1_PINCNT_M = usize(0x0000E000);  // Package Pin Count
pub const SYSCTL_DID1_PINCNT_100 = usize(0x00004000);  // 100-pin LQFP package
pub const SYSCTL_DID1_PINCNT_64 = usize(0x00006000);  // 64-pin LQFP package
pub const SYSCTL_DID1_PINCNT_144 = usize(0x00008000);  // 144-pin LQFP package
pub const SYSCTL_DID1_PINCNT_157 = usize(0x0000A000);  // 157-pin BGA package
pub const SYSCTL_DID1_PINCNT_128 = usize(0x0000C000);  // 128-pin TQFP package
pub const SYSCTL_DID1_TEMP_M = usize(0x000000E0);  // Temperature Range
pub const SYSCTL_DID1_TEMP_C = usize(0x00000000);  // Commercial temperature range
pub const SYSCTL_DID1_TEMP_I = usize(0x00000020);  // Industrial temperature range
pub const SYSCTL_DID1_TEMP_E = usize(0x00000040);  // Extended temperature range
pub const SYSCTL_DID1_TEMP_IE = usize(0x00000060);  // Available in both industrial
                                            // temperature range (-40C to 85C)
                                            // and extended temperature range
                                            // (-40C to 105C) devices. See
pub const SYSCTL_DID1_PKG_M = usize(0x00000018);  // Package Type
pub const SYSCTL_DID1_PKG_QFP = usize(0x00000008);  // QFP package
pub const SYSCTL_DID1_PKG_BGA = usize(0x00000010);  // BGA package
pub const SYSCTL_DID1_ROHS = usize(0x00000004);  // RoHS-Compliance
pub const SYSCTL_DID1_QUAL_M = usize(0x00000003);  // Qualification Status
pub const SYSCTL_DID1_QUAL_ES = usize(0x00000000);  // Engineering Sample (unqualified)
pub const SYSCTL_DID1_QUAL_PP = usize(0x00000001);  // Pilot Production (unqualified)
pub const SYSCTL_DID1_QUAL_FQ = usize(0x00000002);  // Fully Qualified

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC0 register.
//
//*****************************************************************************
pub const SYSCTL_DC0_SRAMSZ_M = usize(0xFFFF0000);  // SRAM Size
pub const SYSCTL_DC0_SRAMSZ_2KB = usize(0x00070000);  // 2 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_4KB = usize(0x000F0000);  // 4 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_6KB = usize(0x00170000);  // 6 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_8KB = usize(0x001F0000);  // 8 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_12KB = usize(0x002F0000);  // 12 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_16KB = usize(0x003F0000);  // 16 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_20KB = usize(0x004F0000);  // 20 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_24KB = usize(0x005F0000);  // 24 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_32KB = usize(0x007F0000);  // 32 KB of SRAM
pub const SYSCTL_DC0_FLASHSZ_M = usize(0x0000FFFF);  // Flash Size
pub const SYSCTL_DC0_FLASHSZ_8KB = usize(0x00000003);  // 8 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_16KB = usize(0x00000007);  // 16 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_32KB = usize(0x0000000F);  // 32 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_64KB = usize(0x0000001F);  // 64 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_96KB = usize(0x0000002F);  // 96 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_128K = usize(0x0000003F);  // 128 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_192K = usize(0x0000005F);  // 192 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_256K = usize(0x0000007F);  // 256 KB of Flash

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC1 register.
//
//*****************************************************************************
pub const SYSCTL_DC1_WDT1 = usize(0x10000000);  // Watchdog Timer1 Present
pub const SYSCTL_DC1_CAN1 = usize(0x02000000);  // CAN Module 1 Present
pub const SYSCTL_DC1_CAN0 = usize(0x01000000);  // CAN Module 0 Present
pub const SYSCTL_DC1_PWM1 = usize(0x00200000);  // PWM Module 1 Present
pub const SYSCTL_DC1_PWM0 = usize(0x00100000);  // PWM Module 0 Present
pub const SYSCTL_DC1_ADC1 = usize(0x00020000);  // ADC Module 1 Present
pub const SYSCTL_DC1_ADC0 = usize(0x00010000);  // ADC Module 0 Present
pub const SYSCTL_DC1_MINSYSDIV_M = usize(0x0000F000);  // System Clock Divider
pub const SYSCTL_DC1_MINSYSDIV_80 = usize(0x00002000);  // Specifies an 80-MHz CPU clock
                                            // with a PLL divider of 2.5
pub const SYSCTL_DC1_MINSYSDIV_50 = usize(0x00003000);  // Specifies a 50-MHz CPU clock
                                            // with a PLL divider of 4
pub const SYSCTL_DC1_MINSYSDIV_40 = usize(0x00004000);  // Specifies a 40-MHz CPU clock
                                            // with a PLL divider of 5
pub const SYSCTL_DC1_MINSYSDIV_25 = usize(0x00007000);  // Specifies a 25-MHz clock with a
                                            // PLL divider of 8
pub const SYSCTL_DC1_MINSYSDIV_20 = usize(0x00009000);  // Specifies a 20-MHz clock with a
                                            // PLL divider of 10
pub const SYSCTL_DC1_ADC1SPD_M = usize(0x00000C00);  // Max ADC1 Speed
pub const SYSCTL_DC1_ADC1SPD_125K = usize(0x00000000);  // 125K samples/second
pub const SYSCTL_DC1_ADC1SPD_250K = usize(0x00000400);  // 250K samples/second
pub const SYSCTL_DC1_ADC1SPD_500K = usize(0x00000800);  // 500K samples/second
pub const SYSCTL_DC1_ADC1SPD_1M = usize(0x00000C00);  // 1M samples/second
pub const SYSCTL_DC1_ADC0SPD_M = usize(0x00000300);  // Max ADC0 Speed
pub const SYSCTL_DC1_ADC0SPD_125K = usize(0x00000000);  // 125K samples/second
pub const SYSCTL_DC1_ADC0SPD_250K = usize(0x00000100);  // 250K samples/second
pub const SYSCTL_DC1_ADC0SPD_500K = usize(0x00000200);  // 500K samples/second
pub const SYSCTL_DC1_ADC0SPD_1M = usize(0x00000300);  // 1M samples/second
pub const SYSCTL_DC1_MPU = usize(0x00000080);  // MPU Present
pub const SYSCTL_DC1_HIB = usize(0x00000040);  // Hibernation Module Present
pub const SYSCTL_DC1_TEMP = usize(0x00000020);  // Temp Sensor Present
pub const SYSCTL_DC1_PLL = usize(0x00000010);  // PLL Present
pub const SYSCTL_DC1_WDT0 = usize(0x00000008);  // Watchdog Timer 0 Present
pub const SYSCTL_DC1_SWO = usize(0x00000004);  // SWO Trace Port Present
pub const SYSCTL_DC1_SWD = usize(0x00000002);  // SWD Present
pub const SYSCTL_DC1_JTAG = usize(0x00000001);  // JTAG Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC2 register.
//
//*****************************************************************************
pub const SYSCTL_DC2_EPI0 = usize(0x40000000);  // EPI Module 0 Present
pub const SYSCTL_DC2_I2S0 = usize(0x10000000);  // I2S Module 0 Present
pub const SYSCTL_DC2_COMP2 = usize(0x04000000);  // Analog Comparator 2 Present
pub const SYSCTL_DC2_COMP1 = usize(0x02000000);  // Analog Comparator 1 Present
pub const SYSCTL_DC2_COMP0 = usize(0x01000000);  // Analog Comparator 0 Present
pub const SYSCTL_DC2_TIMER3 = usize(0x00080000);  // Timer Module 3 Present
pub const SYSCTL_DC2_TIMER2 = usize(0x00040000);  // Timer Module 2 Present
pub const SYSCTL_DC2_TIMER1 = usize(0x00020000);  // Timer Module 1 Present
pub const SYSCTL_DC2_TIMER0 = usize(0x00010000);  // Timer Module 0 Present
pub const SYSCTL_DC2_I2C1HS = usize(0x00008000);  // I2C Module 1 Speed
pub const SYSCTL_DC2_I2C1 = usize(0x00004000);  // I2C Module 1 Present
pub const SYSCTL_DC2_I2C0HS = usize(0x00002000);  // I2C Module 0 Speed
pub const SYSCTL_DC2_I2C0 = usize(0x00001000);  // I2C Module 0 Present
pub const SYSCTL_DC2_QEI1 = usize(0x00000200);  // QEI Module 1 Present
pub const SYSCTL_DC2_QEI0 = usize(0x00000100);  // QEI Module 0 Present
pub const SYSCTL_DC2_SSI1 = usize(0x00000020);  // SSI Module 1 Present
pub const SYSCTL_DC2_SSI0 = usize(0x00000010);  // SSI Module 0 Present
pub const SYSCTL_DC2_UART2 = usize(0x00000004);  // UART Module 2 Present
pub const SYSCTL_DC2_UART1 = usize(0x00000002);  // UART Module 1 Present
pub const SYSCTL_DC2_UART0 = usize(0x00000001);  // UART Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC3 register.
//
//*****************************************************************************
pub const SYSCTL_DC3_32KHZ = usize(0x80000000);  // 32KHz Input Clock Available
pub const SYSCTL_DC3_CCP5 = usize(0x20000000);  // T2CCP1 Pin Present
pub const SYSCTL_DC3_CCP4 = usize(0x10000000);  // T2CCP0 Pin Present
pub const SYSCTL_DC3_CCP3 = usize(0x08000000);  // T1CCP1 Pin Present
pub const SYSCTL_DC3_CCP2 = usize(0x04000000);  // T1CCP0 Pin Present
pub const SYSCTL_DC3_CCP1 = usize(0x02000000);  // T0CCP1 Pin Present
pub const SYSCTL_DC3_CCP0 = usize(0x01000000);  // T0CCP0 Pin Present
pub const SYSCTL_DC3_ADC0AIN7 = usize(0x00800000);  // ADC Module 0 AIN7 Pin Present
pub const SYSCTL_DC3_ADC0AIN6 = usize(0x00400000);  // ADC Module 0 AIN6 Pin Present
pub const SYSCTL_DC3_ADC0AIN5 = usize(0x00200000);  // ADC Module 0 AIN5 Pin Present
pub const SYSCTL_DC3_ADC0AIN4 = usize(0x00100000);  // ADC Module 0 AIN4 Pin Present
pub const SYSCTL_DC3_ADC0AIN3 = usize(0x00080000);  // ADC Module 0 AIN3 Pin Present
pub const SYSCTL_DC3_ADC0AIN2 = usize(0x00040000);  // ADC Module 0 AIN2 Pin Present
pub const SYSCTL_DC3_ADC0AIN1 = usize(0x00020000);  // ADC Module 0 AIN1 Pin Present
pub const SYSCTL_DC3_ADC0AIN0 = usize(0x00010000);  // ADC Module 0 AIN0 Pin Present
pub const SYSCTL_DC3_PWMFAULT = usize(0x00008000);  // PWM Fault Pin Present
pub const SYSCTL_DC3_C2O = usize(0x00004000);  // C2o Pin Present
pub const SYSCTL_DC3_C2PLUS = usize(0x00002000);  // C2+ Pin Present
pub const SYSCTL_DC3_C2MINUS = usize(0x00001000);  // C2- Pin Present
pub const SYSCTL_DC3_C1O = usize(0x00000800);  // C1o Pin Present
pub const SYSCTL_DC3_C1PLUS = usize(0x00000400);  // C1+ Pin Present
pub const SYSCTL_DC3_C1MINUS = usize(0x00000200);  // C1- Pin Present
pub const SYSCTL_DC3_C0O = usize(0x00000100);  // C0o Pin Present
pub const SYSCTL_DC3_C0PLUS = usize(0x00000080);  // C0+ Pin Present
pub const SYSCTL_DC3_C0MINUS = usize(0x00000040);  // C0- Pin Present
pub const SYSCTL_DC3_PWM5 = usize(0x00000020);  // PWM5 Pin Present
pub const SYSCTL_DC3_PWM4 = usize(0x00000010);  // PWM4 Pin Present
pub const SYSCTL_DC3_PWM3 = usize(0x00000008);  // PWM3 Pin Present
pub const SYSCTL_DC3_PWM2 = usize(0x00000004);  // PWM2 Pin Present
pub const SYSCTL_DC3_PWM1 = usize(0x00000002);  // PWM1 Pin Present
pub const SYSCTL_DC3_PWM0 = usize(0x00000001);  // PWM0 Pin Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC4 register.
//
//*****************************************************************************
pub const SYSCTL_DC4_EPHY0 = usize(0x40000000);  // Ethernet PHY Layer 0 Present
pub const SYSCTL_DC4_EMAC0 = usize(0x10000000);  // Ethernet MAC Layer 0 Present
pub const SYSCTL_DC4_E1588 = usize(0x01000000);  // 1588 Capable
pub const SYSCTL_DC4_PICAL = usize(0x00040000);  // PIOSC Calibrate
pub const SYSCTL_DC4_CCP7 = usize(0x00008000);  // T3CCP1 Pin Present
pub const SYSCTL_DC4_CCP6 = usize(0x00004000);  // T3CCP0 Pin Present
pub const SYSCTL_DC4_UDMA = usize(0x00002000);  // Micro-DMA Module Present
pub const SYSCTL_DC4_ROM = usize(0x00001000);  // Internal Code ROM Present
pub const SYSCTL_DC4_GPIOJ = usize(0x00000100);  // GPIO Port J Present
pub const SYSCTL_DC4_GPIOH = usize(0x00000080);  // GPIO Port H Present
pub const SYSCTL_DC4_GPIOG = usize(0x00000040);  // GPIO Port G Present
pub const SYSCTL_DC4_GPIOF = usize(0x00000020);  // GPIO Port F Present
pub const SYSCTL_DC4_GPIOE = usize(0x00000010);  // GPIO Port E Present
pub const SYSCTL_DC4_GPIOD = usize(0x00000008);  // GPIO Port D Present
pub const SYSCTL_DC4_GPIOC = usize(0x00000004);  // GPIO Port C Present
pub const SYSCTL_DC4_GPIOB = usize(0x00000002);  // GPIO Port B Present
pub const SYSCTL_DC4_GPIOA = usize(0x00000001);  // GPIO Port A Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC5 register.
//
//*****************************************************************************
pub const SYSCTL_DC5_PWMFAULT3 = usize(0x08000000);  // PWM Fault 3 Pin Present
pub const SYSCTL_DC5_PWMFAULT2 = usize(0x04000000);  // PWM Fault 2 Pin Present
pub const SYSCTL_DC5_PWMFAULT1 = usize(0x02000000);  // PWM Fault 1 Pin Present
pub const SYSCTL_DC5_PWMFAULT0 = usize(0x01000000);  // PWM Fault 0 Pin Present
pub const SYSCTL_DC5_PWMEFLT = usize(0x00200000);  // PWM Extended Fault Active
pub const SYSCTL_DC5_PWMESYNC = usize(0x00100000);  // PWM Extended SYNC Active
pub const SYSCTL_DC5_PWM7 = usize(0x00000080);  // PWM7 Pin Present
pub const SYSCTL_DC5_PWM6 = usize(0x00000040);  // PWM6 Pin Present
pub const SYSCTL_DC5_PWM5 = usize(0x00000020);  // PWM5 Pin Present
pub const SYSCTL_DC5_PWM4 = usize(0x00000010);  // PWM4 Pin Present
pub const SYSCTL_DC5_PWM3 = usize(0x00000008);  // PWM3 Pin Present
pub const SYSCTL_DC5_PWM2 = usize(0x00000004);  // PWM2 Pin Present
pub const SYSCTL_DC5_PWM1 = usize(0x00000002);  // PWM1 Pin Present
pub const SYSCTL_DC5_PWM0 = usize(0x00000001);  // PWM0 Pin Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC6 register.
//
//*****************************************************************************
pub const SYSCTL_DC6_USB0PHY = usize(0x00000010);  // USB Module 0 PHY Present
pub const SYSCTL_DC6_USB0_M = usize(0x00000003);  // USB Module 0 Present
pub const SYSCTL_DC6_USB0_DEV = usize(0x00000001);  // USB0 is Device Only
pub const SYSCTL_DC6_USB0_HOSTDEV = usize(0x00000002);  // USB is Device or Host
pub const SYSCTL_DC6_USB0_OTG = usize(0x00000003);  // USB0 is OTG

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC7 register.
//
//*****************************************************************************
pub const SYSCTL_DC7_DMACH30 = usize(0x40000000);  // DMA Channel 30
pub const SYSCTL_DC7_DMACH29 = usize(0x20000000);  // DMA Channel 29
pub const SYSCTL_DC7_DMACH28 = usize(0x10000000);  // DMA Channel 28
pub const SYSCTL_DC7_DMACH27 = usize(0x08000000);  // DMA Channel 27
pub const SYSCTL_DC7_DMACH26 = usize(0x04000000);  // DMA Channel 26
pub const SYSCTL_DC7_DMACH25 = usize(0x02000000);  // DMA Channel 25
pub const SYSCTL_DC7_DMACH24 = usize(0x01000000);  // DMA Channel 24
pub const SYSCTL_DC7_DMACH23 = usize(0x00800000);  // DMA Channel 23
pub const SYSCTL_DC7_DMACH22 = usize(0x00400000);  // DMA Channel 22
pub const SYSCTL_DC7_DMACH21 = usize(0x00200000);  // DMA Channel 21
pub const SYSCTL_DC7_DMACH20 = usize(0x00100000);  // DMA Channel 20
pub const SYSCTL_DC7_DMACH19 = usize(0x00080000);  // DMA Channel 19
pub const SYSCTL_DC7_DMACH18 = usize(0x00040000);  // DMA Channel 18
pub const SYSCTL_DC7_DMACH17 = usize(0x00020000);  // DMA Channel 17
pub const SYSCTL_DC7_DMACH16 = usize(0x00010000);  // DMA Channel 16
pub const SYSCTL_DC7_DMACH15 = usize(0x00008000);  // DMA Channel 15
pub const SYSCTL_DC7_DMACH14 = usize(0x00004000);  // DMA Channel 14
pub const SYSCTL_DC7_DMACH13 = usize(0x00002000);  // DMA Channel 13
pub const SYSCTL_DC7_DMACH12 = usize(0x00001000);  // DMA Channel 12
pub const SYSCTL_DC7_DMACH11 = usize(0x00000800);  // DMA Channel 11
pub const SYSCTL_DC7_DMACH10 = usize(0x00000400);  // DMA Channel 10
pub const SYSCTL_DC7_DMACH9 = usize(0x00000200);  // DMA Channel 9
pub const SYSCTL_DC7_DMACH8 = usize(0x00000100);  // DMA Channel 8
pub const SYSCTL_DC7_DMACH7 = usize(0x00000080);  // DMA Channel 7
pub const SYSCTL_DC7_DMACH6 = usize(0x00000040);  // DMA Channel 6
pub const SYSCTL_DC7_DMACH5 = usize(0x00000020);  // DMA Channel 5
pub const SYSCTL_DC7_DMACH4 = usize(0x00000010);  // DMA Channel 4
pub const SYSCTL_DC7_DMACH3 = usize(0x00000008);  // DMA Channel 3
pub const SYSCTL_DC7_DMACH2 = usize(0x00000004);  // DMA Channel 2
pub const SYSCTL_DC7_DMACH1 = usize(0x00000002);  // DMA Channel 1
pub const SYSCTL_DC7_DMACH0 = usize(0x00000001);  // DMA Channel 0

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC8 register.
//
//*****************************************************************************
pub const SYSCTL_DC8_ADC1AIN15 = usize(0x80000000);  // ADC Module 1 AIN15 Pin Present
pub const SYSCTL_DC8_ADC1AIN14 = usize(0x40000000);  // ADC Module 1 AIN14 Pin Present
pub const SYSCTL_DC8_ADC1AIN13 = usize(0x20000000);  // ADC Module 1 AIN13 Pin Present
pub const SYSCTL_DC8_ADC1AIN12 = usize(0x10000000);  // ADC Module 1 AIN12 Pin Present
pub const SYSCTL_DC8_ADC1AIN11 = usize(0x08000000);  // ADC Module 1 AIN11 Pin Present
pub const SYSCTL_DC8_ADC1AIN10 = usize(0x04000000);  // ADC Module 1 AIN10 Pin Present
pub const SYSCTL_DC8_ADC1AIN9 = usize(0x02000000);  // ADC Module 1 AIN9 Pin Present
pub const SYSCTL_DC8_ADC1AIN8 = usize(0x01000000);  // ADC Module 1 AIN8 Pin Present
pub const SYSCTL_DC8_ADC1AIN7 = usize(0x00800000);  // ADC Module 1 AIN7 Pin Present
pub const SYSCTL_DC8_ADC1AIN6 = usize(0x00400000);  // ADC Module 1 AIN6 Pin Present
pub const SYSCTL_DC8_ADC1AIN5 = usize(0x00200000);  // ADC Module 1 AIN5 Pin Present
pub const SYSCTL_DC8_ADC1AIN4 = usize(0x00100000);  // ADC Module 1 AIN4 Pin Present
pub const SYSCTL_DC8_ADC1AIN3 = usize(0x00080000);  // ADC Module 1 AIN3 Pin Present
pub const SYSCTL_DC8_ADC1AIN2 = usize(0x00040000);  // ADC Module 1 AIN2 Pin Present
pub const SYSCTL_DC8_ADC1AIN1 = usize(0x00020000);  // ADC Module 1 AIN1 Pin Present
pub const SYSCTL_DC8_ADC1AIN0 = usize(0x00010000);  // ADC Module 1 AIN0 Pin Present
pub const SYSCTL_DC8_ADC0AIN15 = usize(0x00008000);  // ADC Module 0 AIN15 Pin Present
pub const SYSCTL_DC8_ADC0AIN14 = usize(0x00004000);  // ADC Module 0 AIN14 Pin Present
pub const SYSCTL_DC8_ADC0AIN13 = usize(0x00002000);  // ADC Module 0 AIN13 Pin Present
pub const SYSCTL_DC8_ADC0AIN12 = usize(0x00001000);  // ADC Module 0 AIN12 Pin Present
pub const SYSCTL_DC8_ADC0AIN11 = usize(0x00000800);  // ADC Module 0 AIN11 Pin Present
pub const SYSCTL_DC8_ADC0AIN10 = usize(0x00000400);  // ADC Module 0 AIN10 Pin Present
pub const SYSCTL_DC8_ADC0AIN9 = usize(0x00000200);  // ADC Module 0 AIN9 Pin Present
pub const SYSCTL_DC8_ADC0AIN8 = usize(0x00000100);  // ADC Module 0 AIN8 Pin Present
pub const SYSCTL_DC8_ADC0AIN7 = usize(0x00000080);  // ADC Module 0 AIN7 Pin Present
pub const SYSCTL_DC8_ADC0AIN6 = usize(0x00000040);  // ADC Module 0 AIN6 Pin Present
pub const SYSCTL_DC8_ADC0AIN5 = usize(0x00000020);  // ADC Module 0 AIN5 Pin Present
pub const SYSCTL_DC8_ADC0AIN4 = usize(0x00000010);  // ADC Module 0 AIN4 Pin Present
pub const SYSCTL_DC8_ADC0AIN3 = usize(0x00000008);  // ADC Module 0 AIN3 Pin Present
pub const SYSCTL_DC8_ADC0AIN2 = usize(0x00000004);  // ADC Module 0 AIN2 Pin Present
pub const SYSCTL_DC8_ADC0AIN1 = usize(0x00000002);  // ADC Module 0 AIN1 Pin Present
pub const SYSCTL_DC8_ADC0AIN0 = usize(0x00000001);  // ADC Module 0 AIN0 Pin Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PBORCTL register.
//
//*****************************************************************************
pub const SYSCTL_PBORCTL_BOR0 = usize(0x00000004);  // VDD under BOR0 Event Action
pub const SYSCTL_PBORCTL_BOR1 = usize(0x00000002);  // VDD under BOR1 Event Action

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PTBOCTL register.
//
//*****************************************************************************
pub const SYSCTL_PTBOCTL_VDDA_UBOR_M = usize(0x00000300);  // VDDA under BOR Event Action
pub const SYSCTL_PTBOCTL_VDDA_UBOR_NONE = usize(0x00000000);  // No Action
pub const SYSCTL_PTBOCTL_VDDA_UBOR_SYSINT = usize(0x00000100);  // System control interrupt
pub const SYSCTL_PTBOCTL_VDDA_UBOR_NMI = usize(0x00000200);  // NMI
pub const SYSCTL_PTBOCTL_VDDA_UBOR_RST = usize(0x00000300);  // Reset
pub const SYSCTL_PTBOCTL_VDD_UBOR_M = usize(0x00000003);  // VDD (VDDS) under BOR Event
                                            // Action
pub const SYSCTL_PTBOCTL_VDD_UBOR_NONE = usize(0x00000000);  // No Action
pub const SYSCTL_PTBOCTL_VDD_UBOR_SYSINT = usize(0x00000001);  // System control interrupt
pub const SYSCTL_PTBOCTL_VDD_UBOR_NMI = usize(0x00000002);  // NMI
pub const SYSCTL_PTBOCTL_VDD_UBOR_RST = usize(0x00000003);  // Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR0 register.
//
//*****************************************************************************
pub const SYSCTL_SRCR0_WDT1 = usize(0x10000000);  // WDT1 Reset Control
pub const SYSCTL_SRCR0_CAN1 = usize(0x02000000);  // CAN1 Reset Control
pub const SYSCTL_SRCR0_CAN0 = usize(0x01000000);  // CAN0 Reset Control
pub const SYSCTL_SRCR0_PWM0 = usize(0x00100000);  // PWM Reset Control
pub const SYSCTL_SRCR0_ADC1 = usize(0x00020000);  // ADC1 Reset Control
pub const SYSCTL_SRCR0_ADC0 = usize(0x00010000);  // ADC0 Reset Control
pub const SYSCTL_SRCR0_HIB = usize(0x00000040);  // HIB Reset Control
pub const SYSCTL_SRCR0_WDT0 = usize(0x00000008);  // WDT0 Reset Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR1 register.
//
//*****************************************************************************
pub const SYSCTL_SRCR1_COMP2 = usize(0x04000000);  // Analog Comp 2 Reset Control
pub const SYSCTL_SRCR1_COMP1 = usize(0x02000000);  // Analog Comp 1 Reset Control
pub const SYSCTL_SRCR1_COMP0 = usize(0x01000000);  // Analog Comp 0 Reset Control
pub const SYSCTL_SRCR1_TIMER3 = usize(0x00080000);  // Timer 3 Reset Control
pub const SYSCTL_SRCR1_TIMER2 = usize(0x00040000);  // Timer 2 Reset Control
pub const SYSCTL_SRCR1_TIMER1 = usize(0x00020000);  // Timer 1 Reset Control
pub const SYSCTL_SRCR1_TIMER0 = usize(0x00010000);  // Timer 0 Reset Control
pub const SYSCTL_SRCR1_I2C1 = usize(0x00004000);  // I2C1 Reset Control
pub const SYSCTL_SRCR1_I2C0 = usize(0x00001000);  // I2C0 Reset Control
pub const SYSCTL_SRCR1_QEI1 = usize(0x00000200);  // QEI1 Reset Control
pub const SYSCTL_SRCR1_QEI0 = usize(0x00000100);  // QEI0 Reset Control
pub const SYSCTL_SRCR1_SSI1 = usize(0x00000020);  // SSI1 Reset Control
pub const SYSCTL_SRCR1_SSI0 = usize(0x00000010);  // SSI0 Reset Control
pub const SYSCTL_SRCR1_UART2 = usize(0x00000004);  // UART2 Reset Control
pub const SYSCTL_SRCR1_UART1 = usize(0x00000002);  // UART1 Reset Control
pub const SYSCTL_SRCR1_UART0 = usize(0x00000001);  // UART0 Reset Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR2 register.
//
//*****************************************************************************
pub const SYSCTL_SRCR2_USB0 = usize(0x00010000);  // USB0 Reset Control
pub const SYSCTL_SRCR2_UDMA = usize(0x00002000);  // Micro-DMA Reset Control
pub const SYSCTL_SRCR2_GPIOJ = usize(0x00000100);  // Port J Reset Control
pub const SYSCTL_SRCR2_GPIOH = usize(0x00000080);  // Port H Reset Control
pub const SYSCTL_SRCR2_GPIOG = usize(0x00000040);  // Port G Reset Control
pub const SYSCTL_SRCR2_GPIOF = usize(0x00000020);  // Port F Reset Control
pub const SYSCTL_SRCR2_GPIOE = usize(0x00000010);  // Port E Reset Control
pub const SYSCTL_SRCR2_GPIOD = usize(0x00000008);  // Port D Reset Control
pub const SYSCTL_SRCR2_GPIOC = usize(0x00000004);  // Port C Reset Control
pub const SYSCTL_SRCR2_GPIOB = usize(0x00000002);  // Port B Reset Control
pub const SYSCTL_SRCR2_GPIOA = usize(0x00000001);  // Port A Reset Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RIS register.
//
//*****************************************************************************
pub const SYSCTL_RIS_BOR0RIS = usize(0x00000800);  // VDD under BOR0 Raw Interrupt
                                            // Status
pub const SYSCTL_RIS_VDDARIS = usize(0x00000400);  // VDDA Power OK Event Raw
                                            // Interrupt Status
pub const SYSCTL_RIS_MOSCPUPRIS = usize(0x00000100);  // MOSC Power Up Raw Interrupt
                                            // Status
pub const SYSCTL_RIS_USBPLLLRIS = usize(0x00000080);  // USB PLL Lock Raw Interrupt
                                            // Status
pub const SYSCTL_RIS_PLLLRIS = usize(0x00000040);  // PLL Lock Raw Interrupt Status
pub const SYSCTL_RIS_MOFRIS = usize(0x00000008);  // Main Oscillator Failure Raw
                                            // Interrupt Status
pub const SYSCTL_RIS_BOR1RIS = usize(0x00000002);  // VDD under BOR1 Raw Interrupt
                                            // Status
pub const SYSCTL_RIS_BORRIS = usize(0x00000002);  // Brown-Out Reset Raw Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_IMC register.
//
//*****************************************************************************
pub const SYSCTL_IMC_BOR0IM = usize(0x00000800);  // VDD under BOR0 Interrupt Mask
pub const SYSCTL_IMC_VDDAIM = usize(0x00000400);  // VDDA Power OK Interrupt Mask
pub const SYSCTL_IMC_MOSCPUPIM = usize(0x00000100);  // MOSC Power Up Interrupt Mask
pub const SYSCTL_IMC_USBPLLLIM = usize(0x00000080);  // USB PLL Lock Interrupt Mask
pub const SYSCTL_IMC_PLLLIM = usize(0x00000040);  // PLL Lock Interrupt Mask
pub const SYSCTL_IMC_MOFIM = usize(0x00000008);  // Main Oscillator Failure
                                            // Interrupt Mask
pub const SYSCTL_IMC_BORIM = usize(0x00000002);  // Brown-Out Reset Interrupt Mask
pub const SYSCTL_IMC_BOR1IM = usize(0x00000002);  // VDD under BOR1 Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MISC register.
//
//*****************************************************************************
pub const SYSCTL_MISC_BOR0MIS = usize(0x00000800);  // VDD under BOR0 Masked Interrupt
                                            // Status
pub const SYSCTL_MISC_VDDAMIS = usize(0x00000400);  // VDDA Power OK Masked Interrupt
                                            // Status
pub const SYSCTL_MISC_MOSCPUPMIS = usize(0x00000100);  // MOSC Power Up Masked Interrupt
                                            // Status
pub const SYSCTL_MISC_USBPLLLMIS = usize(0x00000080);  // USB PLL Lock Masked Interrupt
                                            // Status
pub const SYSCTL_MISC_PLLLMIS = usize(0x00000040);  // PLL Lock Masked Interrupt Status
pub const SYSCTL_MISC_MOFMIS = usize(0x00000008);  // Main Oscillator Failure Masked
                                            // Interrupt Status
pub const SYSCTL_MISC_BORMIS = usize(0x00000002);  // BOR Masked Interrupt Status
pub const SYSCTL_MISC_BOR1MIS = usize(0x00000002);  // VDD under BOR1 Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RESC register.
//
//*****************************************************************************
pub const SYSCTL_RESC_MOSCFAIL = usize(0x00010000);  // MOSC Failure Reset
pub const SYSCTL_RESC_HSSR = usize(0x00001000);  // HSSR Reset
pub const SYSCTL_RESC_WDT1 = usize(0x00000020);  // Watchdog Timer 1 Reset
pub const SYSCTL_RESC_SW = usize(0x00000010);  // Software Reset
pub const SYSCTL_RESC_WDT0 = usize(0x00000008);  // Watchdog Timer 0 Reset
pub const SYSCTL_RESC_BOR = usize(0x00000004);  // Brown-Out Reset
pub const SYSCTL_RESC_POR = usize(0x00000002);  // Power-On Reset
pub const SYSCTL_RESC_EXT = usize(0x00000001);  // External Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PWRTC register.
//
//*****************************************************************************
pub const SYSCTL_PWRTC_VDDA_UBOR = usize(0x00000010);  // VDDA Under BOR Status
pub const SYSCTL_PWRTC_VDD_UBOR = usize(0x00000001);  // VDD Under BOR Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCC register.
//
//*****************************************************************************
pub const SYSCTL_RCC_ACG = usize(0x08000000);  // Auto Clock Gating
pub const SYSCTL_RCC_SYSDIV_M = usize(0x07800000);  // System Clock Divisor
pub const SYSCTL_RCC_USESYSDIV = usize(0x00400000);  // Enable System Clock Divider
pub const SYSCTL_RCC_USEPWMDIV = usize(0x00100000);  // Enable PWM Clock Divisor
pub const SYSCTL_RCC_PWMDIV_M = usize(0x000E0000);  // PWM Unit Clock Divisor
pub const SYSCTL_RCC_PWMDIV_2 = usize(0x00000000);  // PWM clock /2
pub const SYSCTL_RCC_PWMDIV_4 = usize(0x00020000);  // PWM clock /4
pub const SYSCTL_RCC_PWMDIV_8 = usize(0x00040000);  // PWM clock /8
pub const SYSCTL_RCC_PWMDIV_16 = usize(0x00060000);  // PWM clock /16
pub const SYSCTL_RCC_PWMDIV_32 = usize(0x00080000);  // PWM clock /32
pub const SYSCTL_RCC_PWMDIV_64 = usize(0x000A0000);  // PWM clock /64
pub const SYSCTL_RCC_PWRDN = usize(0x00002000);  // PLL Power Down
pub const SYSCTL_RCC_BYPASS = usize(0x00000800);  // PLL Bypass
pub const SYSCTL_RCC_XTAL_M = usize(0x000007C0);  // Crystal Value
pub const SYSCTL_RCC_XTAL_4MHZ = usize(0x00000180);  // 4 MHz
pub const SYSCTL_RCC_XTAL_4_09MHZ = usize(0x000001C0);  // 4.096 MHz
pub const SYSCTL_RCC_XTAL_4_91MHZ = usize(0x00000200);  // 4.9152 MHz
pub const SYSCTL_RCC_XTAL_5MHZ = usize(0x00000240);  // 5 MHz
pub const SYSCTL_RCC_XTAL_5_12MHZ = usize(0x00000280);  // 5.12 MHz
pub const SYSCTL_RCC_XTAL_6MHZ = usize(0x000002C0);  // 6 MHz
pub const SYSCTL_RCC_XTAL_6_14MHZ = usize(0x00000300);  // 6.144 MHz
pub const SYSCTL_RCC_XTAL_7_37MHZ = usize(0x00000340);  // 7.3728 MHz
pub const SYSCTL_RCC_XTAL_8MHZ = usize(0x00000380);  // 8 MHz
pub const SYSCTL_RCC_XTAL_8_19MHZ = usize(0x000003C0);  // 8.192 MHz
pub const SYSCTL_RCC_XTAL_10MHZ = usize(0x00000400);  // 10 MHz
pub const SYSCTL_RCC_XTAL_12MHZ = usize(0x00000440);  // 12 MHz
pub const SYSCTL_RCC_XTAL_12_2MHZ = usize(0x00000480);  // 12.288 MHz
pub const SYSCTL_RCC_XTAL_13_5MHZ = usize(0x000004C0);  // 13.56 MHz
pub const SYSCTL_RCC_XTAL_14_3MHZ = usize(0x00000500);  // 14.31818 MHz
pub const SYSCTL_RCC_XTAL_16MHZ = usize(0x00000540);  // 16 MHz
pub const SYSCTL_RCC_XTAL_16_3MHZ = usize(0x00000580);  // 16.384 MHz
pub const SYSCTL_RCC_XTAL_18MHZ = usize(0x000005C0);  // 18.0 MHz (USB)
pub const SYSCTL_RCC_XTAL_20MHZ = usize(0x00000600);  // 20.0 MHz (USB)
pub const SYSCTL_RCC_XTAL_24MHZ = usize(0x00000640);  // 24.0 MHz (USB)
pub const SYSCTL_RCC_XTAL_25MHZ = usize(0x00000680);  // 25.0 MHz (USB)
pub const SYSCTL_RCC_OSCSRC_M = usize(0x00000030);  // Oscillator Source
pub const SYSCTL_RCC_OSCSRC_MAIN = usize(0x00000000);  // MOSC
pub const SYSCTL_RCC_OSCSRC_INT = usize(0x00000010);  // IOSC
pub const SYSCTL_RCC_OSCSRC_INT4 = usize(0x00000020);  // IOSC/4
pub const SYSCTL_RCC_OSCSRC_30 = usize(0x00000030);  // LFIOSC
pub const SYSCTL_RCC_MOSCDIS = usize(0x00000001);  // Main Oscillator Disable
pub const SYSCTL_RCC_SYSDIV_S = usize(23);
pub const SYSCTL_RCC_XTAL_S = usize(6);           // Shift to the XTAL field

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_NMIC register.
//
//*****************************************************************************
pub const SYSCTL_NMIC_MOSCFAIL = usize(0x00010000);  // MOSC Failure NMI
pub const SYSCTL_NMIC_TAMPER = usize(0x00000200);  // Tamper Event NMI
pub const SYSCTL_NMIC_WDT1 = usize(0x00000020);  // Watch Dog Timer (WDT) 1 NMI
pub const SYSCTL_NMIC_WDT0 = usize(0x00000008);  // Watch Dog Timer (WDT) 0 NMI
pub const SYSCTL_NMIC_POWER = usize(0x00000004);  // Power/Brown Out Event NMI
pub const SYSCTL_NMIC_EXTERNAL = usize(0x00000001);  // External Pin NMI

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_GPIOHBCTL
// register.
//
//*****************************************************************************
pub const SYSCTL_GPIOHBCTL_PORTJ = usize(0x00000100);  // Port J Advanced High-Performance
                                            // Bus
pub const SYSCTL_GPIOHBCTL_PORTH = usize(0x00000080);  // Port H Advanced High-Performance
                                            // Bus
pub const SYSCTL_GPIOHBCTL_PORTG = usize(0x00000040);  // Port G Advanced High-Performance
                                            // Bus
pub const SYSCTL_GPIOHBCTL_PORTF = usize(0x00000020);  // Port F Advanced High-Performance
                                            // Bus
pub const SYSCTL_GPIOHBCTL_PORTE = usize(0x00000010);  // Port E Advanced High-Performance
                                            // Bus
pub const SYSCTL_GPIOHBCTL_PORTD = usize(0x00000008);  // Port D Advanced High-Performance
                                            // Bus
pub const SYSCTL_GPIOHBCTL_PORTC = usize(0x00000004);  // Port C Advanced High-Performance
                                            // Bus
pub const SYSCTL_GPIOHBCTL_PORTB = usize(0x00000002);  // Port B Advanced High-Performance
                                            // Bus
pub const SYSCTL_GPIOHBCTL_PORTA = usize(0x00000001);  // Port A Advanced High-Performance
                                            // Bus

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCC2 register.
//
//*****************************************************************************
pub const SYSCTL_RCC2_USERCC2 = usize(0x80000000);  // Use RCC2
pub const SYSCTL_RCC2_DIV400 = usize(0x40000000);  // Divide PLL as 400 MHz vs. 200
                                            // MHz
pub const SYSCTL_RCC2_SYSDIV2_M = usize(0x1F800000);  // System Clock Divisor 2
pub const SYSCTL_RCC2_SYSDIV2LSB = usize(0x00400000);  // Additional LSB for SYSDIV2
pub const SYSCTL_RCC2_USBPWRDN = usize(0x00004000);  // Power-Down USB PLL
pub const SYSCTL_RCC2_PWRDN2 = usize(0x00002000);  // Power-Down PLL 2
pub const SYSCTL_RCC2_BYPASS2 = usize(0x00000800);  // PLL Bypass 2
pub const SYSCTL_RCC2_OSCSRC2_M = usize(0x00000070);  // Oscillator Source 2
pub const SYSCTL_RCC2_OSCSRC2_MO = usize(0x00000000);  // MOSC
pub const SYSCTL_RCC2_OSCSRC2_IO = usize(0x00000010);  // PIOSC
pub const SYSCTL_RCC2_OSCSRC2_IO4 = usize(0x00000020);  // PIOSC/4
pub const SYSCTL_RCC2_OSCSRC2_30 = usize(0x00000030);  // LFIOSC
pub const SYSCTL_RCC2_OSCSRC2_32 = usize(0x00000070);  // 32.768 kHz
pub const SYSCTL_RCC2_SYSDIV2_S = usize(23);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MOSCCTL register.
//
//*****************************************************************************
pub const SYSCTL_MOSCCTL_OSCRNG = usize(0x00000010);  // Oscillator Range
pub const SYSCTL_MOSCCTL_PWRDN = usize(0x00000008);  // Power Down
pub const SYSCTL_MOSCCTL_NOXTAL = usize(0x00000004);  // No Crystal Connected
pub const SYSCTL_MOSCCTL_MOSCIM = usize(0x00000002);  // MOSC Failure Action
pub const SYSCTL_MOSCCTL_CVAL = usize(0x00000001);  // Clock Validation for MOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RSCLKCFG
// register.
//
//*****************************************************************************
pub const SYSCTL_RSCLKCFG_MEMTIMU = usize(0x80000000);  // Memory Timing Register Update
pub const SYSCTL_RSCLKCFG_NEWFREQ = usize(0x40000000);  // New PLLFREQ Accept
pub const SYSCTL_RSCLKCFG_ACG = usize(0x20000000);  // Auto Clock Gating
pub const SYSCTL_RSCLKCFG_USEPLL = usize(0x10000000);  // Use PLL
pub const SYSCTL_RSCLKCFG_PLLSRC_M = usize(0x0F000000);  // PLL Source
pub const SYSCTL_RSCLKCFG_PLLSRC_PIOSC = usize(0x00000000);  // PIOSC is PLL input clock source
pub const SYSCTL_RSCLKCFG_PLLSRC_MOSC = usize(0x03000000);  // MOSC is the PLL input clock
                                            // source
pub const SYSCTL_RSCLKCFG_OSCSRC_M = usize(0x00F00000);  // Oscillator Source
pub const SYSCTL_RSCLKCFG_OSCSRC_PIOSC = usize(0x00000000);  // PIOSC is oscillator source
pub const SYSCTL_RSCLKCFG_OSCSRC_LFIOSC = usize(0x00200000);  // LFIOSC is oscillator source
pub const SYSCTL_RSCLKCFG_OSCSRC_MOSC = usize(0x00300000);  // MOSC is oscillator source
pub const SYSCTL_RSCLKCFG_OSCSRC_RTC = usize(0x00400000);  // Hibernation Module RTC
                                            // Oscillator (RTCOSC)
pub const SYSCTL_RSCLKCFG_OSYSDIV_M = usize(0x000FFC00);  // Oscillator System Clock Divisor
pub const SYSCTL_RSCLKCFG_PSYSDIV_M = usize(0x000003FF);  // PLL System Clock Divisor
pub const SYSCTL_RSCLKCFG_OSYSDIV_S = usize(10);
pub const SYSCTL_RSCLKCFG_PSYSDIV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MEMTIM0 register.
//
//*****************************************************************************
pub const SYSCTL_MEMTIM0_EBCHT_M = usize(0x03C00000);  // EEPROM Clock High Time
pub const SYSCTL_MEMTIM0_EBCHT_0_5 = usize(0x00000000);  // 1/2 system clock period
pub const SYSCTL_MEMTIM0_EBCHT_1 = usize(0x00400000);  // 1 system clock period
pub const SYSCTL_MEMTIM0_EBCHT_1_5 = usize(0x00800000);  // 1.5 system clock periods
pub const SYSCTL_MEMTIM0_EBCHT_2 = usize(0x00C00000);  // 2 system clock periods
pub const SYSCTL_MEMTIM0_EBCHT_2_5 = usize(0x01000000);  // 2.5 system clock periods
pub const SYSCTL_MEMTIM0_EBCHT_3 = usize(0x01400000);  // 3 system clock periods
pub const SYSCTL_MEMTIM0_EBCHT_3_5 = usize(0x01800000);  // 3.5 system clock periods
pub const SYSCTL_MEMTIM0_EBCHT_4 = usize(0x01C00000);  // 4 system clock periods
pub const SYSCTL_MEMTIM0_EBCHT_4_5 = usize(0x02000000);  // 4.5 system clock periods
pub const SYSCTL_MEMTIM0_EBCE = usize(0x00200000);  // EEPROM Bank Clock Edge
pub const SYSCTL_MEMTIM0_MB1 = usize(0x00100010);  // Must be one
pub const SYSCTL_MEMTIM0_EWS_M = usize(0x000F0000);  // EEPROM Wait States
pub const SYSCTL_MEMTIM0_FBCHT_M = usize(0x000003C0);  // Flash Bank Clock High Time
pub const SYSCTL_MEMTIM0_FBCHT_0_5 = usize(0x00000000);  // 1/2 system clock period
pub const SYSCTL_MEMTIM0_FBCHT_1 = usize(0x00000040);  // 1 system clock period
pub const SYSCTL_MEMTIM0_FBCHT_1_5 = usize(0x00000080);  // 1.5 system clock periods
pub const SYSCTL_MEMTIM0_FBCHT_2 = usize(0x000000C0);  // 2 system clock periods
pub const SYSCTL_MEMTIM0_FBCHT_2_5 = usize(0x00000100);  // 2.5 system clock periods
pub const SYSCTL_MEMTIM0_FBCHT_3 = usize(0x00000140);  // 3 system clock periods
pub const SYSCTL_MEMTIM0_FBCHT_3_5 = usize(0x00000180);  // 3.5 system clock periods
pub const SYSCTL_MEMTIM0_FBCHT_4 = usize(0x000001C0);  // 4 system clock periods
pub const SYSCTL_MEMTIM0_FBCHT_4_5 = usize(0x00000200);  // 4.5 system clock periods
pub const SYSCTL_MEMTIM0_FBCE = usize(0x00000020);  // Flash Bank Clock Edge
pub const SYSCTL_MEMTIM0_FWS_M = usize(0x0000000F);  // Flash Wait State
pub const SYSCTL_MEMTIM0_EWS_S = usize(16);
pub const SYSCTL_MEMTIM0_FWS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC0 register.
//
//*****************************************************************************
pub const SYSCTL_RCGC0_WDT1 = usize(0x10000000);  // WDT1 Clock Gating Control
pub const SYSCTL_RCGC0_CAN1 = usize(0x02000000);  // CAN1 Clock Gating Control
pub const SYSCTL_RCGC0_CAN0 = usize(0x01000000);  // CAN0 Clock Gating Control
pub const SYSCTL_RCGC0_PWM0 = usize(0x00100000);  // PWM Clock Gating Control
pub const SYSCTL_RCGC0_ADC1 = usize(0x00020000);  // ADC1 Clock Gating Control
pub const SYSCTL_RCGC0_ADC0 = usize(0x00010000);  // ADC0 Clock Gating Control
pub const SYSCTL_RCGC0_ADC1SPD_M = usize(0x00000C00);  // ADC1 Sample Speed
pub const SYSCTL_RCGC0_ADC1SPD_125K = usize(0x00000000);  // 125K samples/second
pub const SYSCTL_RCGC0_ADC1SPD_250K = usize(0x00000400);  // 250K samples/second
pub const SYSCTL_RCGC0_ADC1SPD_500K = usize(0x00000800);  // 500K samples/second
pub const SYSCTL_RCGC0_ADC1SPD_1M = usize(0x00000C00);  // 1M samples/second
pub const SYSCTL_RCGC0_ADC0SPD_M = usize(0x00000300);  // ADC0 Sample Speed
pub const SYSCTL_RCGC0_ADC0SPD_125K = usize(0x00000000);  // 125K samples/second
pub const SYSCTL_RCGC0_ADC0SPD_250K = usize(0x00000100);  // 250K samples/second
pub const SYSCTL_RCGC0_ADC0SPD_500K = usize(0x00000200);  // 500K samples/second
pub const SYSCTL_RCGC0_ADC0SPD_1M = usize(0x00000300);  // 1M samples/second
pub const SYSCTL_RCGC0_HIB = usize(0x00000040);  // HIB Clock Gating Control
pub const SYSCTL_RCGC0_WDT0 = usize(0x00000008);  // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC1 register.
//
//*****************************************************************************
pub const SYSCTL_RCGC1_COMP2 = usize(0x04000000);  // Analog Comparator 2 Clock Gating
pub const SYSCTL_RCGC1_COMP1 = usize(0x02000000);  // Analog Comparator 1 Clock Gating
pub const SYSCTL_RCGC1_COMP0 = usize(0x01000000);  // Analog Comparator 0 Clock Gating
pub const SYSCTL_RCGC1_TIMER3 = usize(0x00080000);  // Timer 3 Clock Gating Control
pub const SYSCTL_RCGC1_TIMER2 = usize(0x00040000);  // Timer 2 Clock Gating Control
pub const SYSCTL_RCGC1_TIMER1 = usize(0x00020000);  // Timer 1 Clock Gating Control
pub const SYSCTL_RCGC1_TIMER0 = usize(0x00010000);  // Timer 0 Clock Gating Control
pub const SYSCTL_RCGC1_I2C1 = usize(0x00004000);  // I2C1 Clock Gating Control
pub const SYSCTL_RCGC1_I2C0 = usize(0x00001000);  // I2C0 Clock Gating Control
pub const SYSCTL_RCGC1_QEI1 = usize(0x00000200);  // QEI1 Clock Gating Control
pub const SYSCTL_RCGC1_QEI0 = usize(0x00000100);  // QEI0 Clock Gating Control
pub const SYSCTL_RCGC1_SSI1 = usize(0x00000020);  // SSI1 Clock Gating Control
pub const SYSCTL_RCGC1_SSI0 = usize(0x00000010);  // SSI0 Clock Gating Control
pub const SYSCTL_RCGC1_UART2 = usize(0x00000004);  // UART2 Clock Gating Control
pub const SYSCTL_RCGC1_UART1 = usize(0x00000002);  // UART1 Clock Gating Control
pub const SYSCTL_RCGC1_UART0 = usize(0x00000001);  // UART0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC2 register.
//
//*****************************************************************************
pub const SYSCTL_RCGC2_USB0 = usize(0x00010000);  // USB0 Clock Gating Control
pub const SYSCTL_RCGC2_UDMA = usize(0x00002000);  // Micro-DMA Clock Gating Control
pub const SYSCTL_RCGC2_GPIOJ = usize(0x00000100);  // Port J Clock Gating Control
pub const SYSCTL_RCGC2_GPIOH = usize(0x00000080);  // Port H Clock Gating Control
pub const SYSCTL_RCGC2_GPIOG = usize(0x00000040);  // Port G Clock Gating Control
pub const SYSCTL_RCGC2_GPIOF = usize(0x00000020);  // Port F Clock Gating Control
pub const SYSCTL_RCGC2_GPIOE = usize(0x00000010);  // Port E Clock Gating Control
pub const SYSCTL_RCGC2_GPIOD = usize(0x00000008);  // Port D Clock Gating Control
pub const SYSCTL_RCGC2_GPIOC = usize(0x00000004);  // Port C Clock Gating Control
pub const SYSCTL_RCGC2_GPIOB = usize(0x00000002);  // Port B Clock Gating Control
pub const SYSCTL_RCGC2_GPIOA = usize(0x00000001);  // Port A Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC0 register.
//
//*****************************************************************************
pub const SYSCTL_SCGC0_WDT1 = usize(0x10000000);  // WDT1 Clock Gating Control
pub const SYSCTL_SCGC0_CAN1 = usize(0x02000000);  // CAN1 Clock Gating Control
pub const SYSCTL_SCGC0_CAN0 = usize(0x01000000);  // CAN0 Clock Gating Control
pub const SYSCTL_SCGC0_PWM0 = usize(0x00100000);  // PWM Clock Gating Control
pub const SYSCTL_SCGC0_ADC1 = usize(0x00020000);  // ADC1 Clock Gating Control
pub const SYSCTL_SCGC0_ADC0 = usize(0x00010000);  // ADC0 Clock Gating Control
pub const SYSCTL_SCGC0_ADCSPD_M = usize(0x00000F00);  // ADC Sample Speed
pub const SYSCTL_SCGC0_HIB = usize(0x00000040);  // HIB Clock Gating Control
pub const SYSCTL_SCGC0_WDT0 = usize(0x00000008);  // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC1 register.
//
//*****************************************************************************
pub const SYSCTL_SCGC1_COMP2 = usize(0x04000000);  // Analog Comparator 2 Clock Gating
pub const SYSCTL_SCGC1_COMP1 = usize(0x02000000);  // Analog Comparator 1 Clock Gating
pub const SYSCTL_SCGC1_COMP0 = usize(0x01000000);  // Analog Comparator 0 Clock Gating
pub const SYSCTL_SCGC1_TIMER3 = usize(0x00080000);  // Timer 3 Clock Gating Control
pub const SYSCTL_SCGC1_TIMER2 = usize(0x00040000);  // Timer 2 Clock Gating Control
pub const SYSCTL_SCGC1_TIMER1 = usize(0x00020000);  // Timer 1 Clock Gating Control
pub const SYSCTL_SCGC1_TIMER0 = usize(0x00010000);  // Timer 0 Clock Gating Control
pub const SYSCTL_SCGC1_I2C1 = usize(0x00004000);  // I2C1 Clock Gating Control
pub const SYSCTL_SCGC1_I2C0 = usize(0x00001000);  // I2C0 Clock Gating Control
pub const SYSCTL_SCGC1_QEI1 = usize(0x00000200);  // QEI1 Clock Gating Control
pub const SYSCTL_SCGC1_QEI0 = usize(0x00000100);  // QEI0 Clock Gating Control
pub const SYSCTL_SCGC1_SSI1 = usize(0x00000020);  // SSI1 Clock Gating Control
pub const SYSCTL_SCGC1_SSI0 = usize(0x00000010);  // SSI0 Clock Gating Control
pub const SYSCTL_SCGC1_UART2 = usize(0x00000004);  // UART2 Clock Gating Control
pub const SYSCTL_SCGC1_UART1 = usize(0x00000002);  // UART1 Clock Gating Control
pub const SYSCTL_SCGC1_UART0 = usize(0x00000001);  // UART0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC2 register.
//
//*****************************************************************************
pub const SYSCTL_SCGC2_USB0 = usize(0x00010000);  // USB0 Clock Gating Control
pub const SYSCTL_SCGC2_UDMA = usize(0x00002000);  // Micro-DMA Clock Gating Control
pub const SYSCTL_SCGC2_GPIOJ = usize(0x00000100);  // Port J Clock Gating Control
pub const SYSCTL_SCGC2_GPIOH = usize(0x00000080);  // Port H Clock Gating Control
pub const SYSCTL_SCGC2_GPIOG = usize(0x00000040);  // Port G Clock Gating Control
pub const SYSCTL_SCGC2_GPIOF = usize(0x00000020);  // Port F Clock Gating Control
pub const SYSCTL_SCGC2_GPIOE = usize(0x00000010);  // Port E Clock Gating Control
pub const SYSCTL_SCGC2_GPIOD = usize(0x00000008);  // Port D Clock Gating Control
pub const SYSCTL_SCGC2_GPIOC = usize(0x00000004);  // Port C Clock Gating Control
pub const SYSCTL_SCGC2_GPIOB = usize(0x00000002);  // Port B Clock Gating Control
pub const SYSCTL_SCGC2_GPIOA = usize(0x00000001);  // Port A Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC0 register.
//
//*****************************************************************************
pub const SYSCTL_DCGC0_WDT1 = usize(0x10000000);  // WDT1 Clock Gating Control
pub const SYSCTL_DCGC0_CAN1 = usize(0x02000000);  // CAN1 Clock Gating Control
pub const SYSCTL_DCGC0_CAN0 = usize(0x01000000);  // CAN0 Clock Gating Control
pub const SYSCTL_DCGC0_PWM0 = usize(0x00100000);  // PWM Clock Gating Control
pub const SYSCTL_DCGC0_ADC1 = usize(0x00020000);  // ADC1 Clock Gating Control
pub const SYSCTL_DCGC0_ADC0 = usize(0x00010000);  // ADC0 Clock Gating Control
pub const SYSCTL_DCGC0_HIB = usize(0x00000040);  // HIB Clock Gating Control
pub const SYSCTL_DCGC0_WDT0 = usize(0x00000008);  // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC1 register.
//
//*****************************************************************************
pub const SYSCTL_DCGC1_COMP2 = usize(0x04000000);  // Analog Comparator 2 Clock Gating
pub const SYSCTL_DCGC1_COMP1 = usize(0x02000000);  // Analog Comparator 1 Clock Gating
pub const SYSCTL_DCGC1_COMP0 = usize(0x01000000);  // Analog Comparator 0 Clock Gating
pub const SYSCTL_DCGC1_TIMER3 = usize(0x00080000);  // Timer 3 Clock Gating Control
pub const SYSCTL_DCGC1_TIMER2 = usize(0x00040000);  // Timer 2 Clock Gating Control
pub const SYSCTL_DCGC1_TIMER1 = usize(0x00020000);  // Timer 1 Clock Gating Control
pub const SYSCTL_DCGC1_TIMER0 = usize(0x00010000);  // Timer 0 Clock Gating Control
pub const SYSCTL_DCGC1_I2C1 = usize(0x00004000);  // I2C1 Clock Gating Control
pub const SYSCTL_DCGC1_I2C0 = usize(0x00001000);  // I2C0 Clock Gating Control
pub const SYSCTL_DCGC1_QEI1 = usize(0x00000200);  // QEI1 Clock Gating Control
pub const SYSCTL_DCGC1_QEI0 = usize(0x00000100);  // QEI0 Clock Gating Control
pub const SYSCTL_DCGC1_SSI1 = usize(0x00000020);  // SSI1 Clock Gating Control
pub const SYSCTL_DCGC1_SSI0 = usize(0x00000010);  // SSI0 Clock Gating Control
pub const SYSCTL_DCGC1_UART2 = usize(0x00000004);  // UART2 Clock Gating Control
pub const SYSCTL_DCGC1_UART1 = usize(0x00000002);  // UART1 Clock Gating Control
pub const SYSCTL_DCGC1_UART0 = usize(0x00000001);  // UART0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC2 register.
//
//*****************************************************************************
pub const SYSCTL_DCGC2_USB0 = usize(0x00010000);  // USB0 Clock Gating Control
pub const SYSCTL_DCGC2_UDMA = usize(0x00002000);  // Micro-DMA Clock Gating Control
pub const SYSCTL_DCGC2_GPIOJ = usize(0x00000100);  // Port J Clock Gating Control
pub const SYSCTL_DCGC2_GPIOH = usize(0x00000080);  // Port H Clock Gating Control
pub const SYSCTL_DCGC2_GPIOG = usize(0x00000040);  // Port G Clock Gating Control
pub const SYSCTL_DCGC2_GPIOF = usize(0x00000020);  // Port F Clock Gating Control
pub const SYSCTL_DCGC2_GPIOE = usize(0x00000010);  // Port E Clock Gating Control
pub const SYSCTL_DCGC2_GPIOD = usize(0x00000008);  // Port D Clock Gating Control
pub const SYSCTL_DCGC2_GPIOC = usize(0x00000004);  // Port C Clock Gating Control
pub const SYSCTL_DCGC2_GPIOB = usize(0x00000002);  // Port B Clock Gating Control
pub const SYSCTL_DCGC2_GPIOA = usize(0x00000001);  // Port A Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_ALTCLKCFG
// register.
//
//*****************************************************************************
pub const SYSCTL_ALTCLKCFG_ALTCLK_M = usize(0x0000000F);  // Alternate Clock Source
pub const SYSCTL_ALTCLKCFG_ALTCLK_PIOSC = usize(0x00000000);  // PIOSC
pub const SYSCTL_ALTCLKCFG_ALTCLK_RTCOSC = usize(0x00000003);  // Hibernation Module Real-time
                                            // clock output (RTCOSC)
pub const SYSCTL_ALTCLKCFG_ALTCLK_LFIOSC = usize(0x00000004);  // Low-frequency internal
                                            // oscillator (LFIOSC)

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DSLPCLKCFG
// register.
//
//*****************************************************************************
pub const SYSCTL_DSLPCLKCFG_D_M = usize(0x1F800000);  // Divider Field Override
pub const SYSCTL_DSLPCLKCFG_O_M = usize(0x00000070);  // Clock Source
pub const SYSCTL_DSLPCLKCFG_O_IGN = usize(0x00000000);  // MOSC
pub const SYSCTL_DSLPCLKCFG_O_IO = usize(0x00000010);  // PIOSC
pub const SYSCTL_DSLPCLKCFG_O_30 = usize(0x00000030);  // LFIOSC
pub const SYSCTL_DSLPCLKCFG_O_32 = usize(0x00000070);  // 32.768 kHz
pub const SYSCTL_DSLPCLKCFG_PIOSCPD = usize(0x00000002);  // PIOSC Power Down Request
pub const SYSCTL_DSLPCLKCFG_D_S = usize(23);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DSCLKCFG
// register.
//
//*****************************************************************************
pub const SYSCTL_DSCLKCFG_PIOSCPD = usize(0x80000000);  // PIOSC Power Down
pub const SYSCTL_DSCLKCFG_MOSCDPD = usize(0x40000000);  // MOSC Disable Power Down
pub const SYSCTL_DSCLKCFG_DSOSCSRC_M = usize(0x00F00000);  // Deep Sleep Oscillator Source
pub const SYSCTL_DSCLKCFG_DSOSCSRC_PIOSC = usize(0x00000000);  // PIOSC
pub const SYSCTL_DSCLKCFG_DSOSCSRC_LFIOSC = usize(0x00200000);  // LFIOSC
pub const SYSCTL_DSCLKCFG_DSOSCSRC_MOSC = usize(0x00300000);  // MOSC
pub const SYSCTL_DSCLKCFG_DSOSCSRC_RTC = usize(0x00400000);  // Hibernation Module RTCOSC
pub const SYSCTL_DSCLKCFG_DSSYSDIV_M = usize(0x000003FF);  // Deep Sleep Clock Divisor
pub const SYSCTL_DSCLKCFG_DSSYSDIV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DIVSCLK register.
//
//*****************************************************************************
pub const SYSCTL_DIVSCLK_EN = usize(0x80000000);  // DIVSCLK Enable
pub const SYSCTL_DIVSCLK_SRC_M = usize(0x00030000);  // Clock Source
pub const SYSCTL_DIVSCLK_SRC_SYSCLK = usize(0x00000000);  // System Clock
pub const SYSCTL_DIVSCLK_SRC_PIOSC = usize(0x00010000);  // PIOSC
pub const SYSCTL_DIVSCLK_SRC_MOSC = usize(0x00020000);  // MOSC
pub const SYSCTL_DIVSCLK_DIV_M = usize(0x000000FF);  // Divisor Value
pub const SYSCTL_DIVSCLK_DIV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SYSPROP register.
//
//*****************************************************************************
pub const SYSCTL_SYSPROP_FPU = usize(0x00000001);  // FPU Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PIOSCCAL
// register.
//
//*****************************************************************************
pub const SYSCTL_PIOSCCAL_UTEN = usize(0x80000000);  // Use User Trim Value
pub const SYSCTL_PIOSCCAL_CAL = usize(0x00000200);  // Start Calibration
pub const SYSCTL_PIOSCCAL_UPDATE = usize(0x00000100);  // Update Trim
pub const SYSCTL_PIOSCCAL_UT_M = usize(0x0000007F);  // User Trim Value
pub const SYSCTL_PIOSCCAL_UT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PIOSCSTAT
// register.
//
//*****************************************************************************
pub const SYSCTL_PIOSCSTAT_DT_M = usize(0x007F0000);  // Default Trim Value
pub const SYSCTL_PIOSCSTAT_CR_M = usize(0x00000300);  // Calibration Result
pub const SYSCTL_PIOSCSTAT_CRNONE = usize(0x00000000);  // Calibration has not been
                                            // attempted
pub const SYSCTL_PIOSCSTAT_CRPASS = usize(0x00000100);  // The last calibration operation
                                            // completed to meet 1% accuracy
pub const SYSCTL_PIOSCSTAT_CRFAIL = usize(0x00000200);  // The last calibration operation
                                            // failed to meet 1% accuracy
pub const SYSCTL_PIOSCSTAT_CT_M = usize(0x0000007F);  // Calibration Trim Value
pub const SYSCTL_PIOSCSTAT_DT_S = usize(16);
pub const SYSCTL_PIOSCSTAT_CT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLFREQ0
// register.
//
//*****************************************************************************
pub const SYSCTL_PLLFREQ0_PLLPWR = usize(0x00800000);  // PLL Power
pub const SYSCTL_PLLFREQ0_MFRAC_M = usize(0x000FFC00);  // PLL M Fractional Value
pub const SYSCTL_PLLFREQ0_MINT_M = usize(0x000003FF);  // PLL M Integer Value
pub const SYSCTL_PLLFREQ0_MFRAC_S = usize(10);
pub const SYSCTL_PLLFREQ0_MINT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLFREQ1
// register.
//
//*****************************************************************************
pub const SYSCTL_PLLFREQ1_Q_M = usize(0x00001F00);  // PLL Q Value
pub const SYSCTL_PLLFREQ1_N_M = usize(0x0000001F);  // PLL N Value
pub const SYSCTL_PLLFREQ1_Q_S = usize(8);
pub const SYSCTL_PLLFREQ1_N_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLSTAT register.
//
//*****************************************************************************
pub const SYSCTL_PLLSTAT_LOCK = usize(0x00000001);  // PLL Lock

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SLPPWRCFG
// register.
//
//*****************************************************************************
pub const SYSCTL_SLPPWRCFG_FLASHPM_M = usize(0x00000030);  // Flash Power Modes
pub const SYSCTL_SLPPWRCFG_FLASHPM_NRM = usize(0x00000000);  // Active Mode
pub const SYSCTL_SLPPWRCFG_FLASHPM_SLP = usize(0x00000020);  // Low Power Mode
pub const SYSCTL_SLPPWRCFG_SRAMPM_M = usize(0x00000003);  // SRAM Power Modes
pub const SYSCTL_SLPPWRCFG_SRAMPM_NRM = usize(0x00000000);  // Active Mode
pub const SYSCTL_SLPPWRCFG_SRAMPM_SBY = usize(0x00000001);  // Standby Mode
pub const SYSCTL_SLPPWRCFG_SRAMPM_LP = usize(0x00000003);  // Low Power Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DSLPPWRCFG
// register.
//
//*****************************************************************************
pub const SYSCTL_DSLPPWRCFG_LDOSM = usize(0x00000200);  // LDO Sleep Mode
pub const SYSCTL_DSLPPWRCFG_TSPD = usize(0x00000100);  // Temperature Sense Power Down
pub const SYSCTL_DSLPPWRCFG_FLASHPM_M = usize(0x00000030);  // Flash Power Modes
pub const SYSCTL_DSLPPWRCFG_FLASHPM_NRM = usize(0x00000000);  // Active Mode
pub const SYSCTL_DSLPPWRCFG_FLASHPM_SLP = usize(0x00000020);  // Low Power Mode
pub const SYSCTL_DSLPPWRCFG_SRAMPM_M = usize(0x00000003);  // SRAM Power Modes
pub const SYSCTL_DSLPPWRCFG_SRAMPM_NRM = usize(0x00000000);  // Active Mode
pub const SYSCTL_DSLPPWRCFG_SRAMPM_SBY = usize(0x00000001);  // Standby Mode
pub const SYSCTL_DSLPPWRCFG_SRAMPM_LP = usize(0x00000003);  // Low Power Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC9 register.
//
//*****************************************************************************
pub const SYSCTL_DC9_ADC1DC7 = usize(0x00800000);  // ADC1 DC7 Present
pub const SYSCTL_DC9_ADC1DC6 = usize(0x00400000);  // ADC1 DC6 Present
pub const SYSCTL_DC9_ADC1DC5 = usize(0x00200000);  // ADC1 DC5 Present
pub const SYSCTL_DC9_ADC1DC4 = usize(0x00100000);  // ADC1 DC4 Present
pub const SYSCTL_DC9_ADC1DC3 = usize(0x00080000);  // ADC1 DC3 Present
pub const SYSCTL_DC9_ADC1DC2 = usize(0x00040000);  // ADC1 DC2 Present
pub const SYSCTL_DC9_ADC1DC1 = usize(0x00020000);  // ADC1 DC1 Present
pub const SYSCTL_DC9_ADC1DC0 = usize(0x00010000);  // ADC1 DC0 Present
pub const SYSCTL_DC9_ADC0DC7 = usize(0x00000080);  // ADC0 DC7 Present
pub const SYSCTL_DC9_ADC0DC6 = usize(0x00000040);  // ADC0 DC6 Present
pub const SYSCTL_DC9_ADC0DC5 = usize(0x00000020);  // ADC0 DC5 Present
pub const SYSCTL_DC9_ADC0DC4 = usize(0x00000010);  // ADC0 DC4 Present
pub const SYSCTL_DC9_ADC0DC3 = usize(0x00000008);  // ADC0 DC3 Present
pub const SYSCTL_DC9_ADC0DC2 = usize(0x00000004);  // ADC0 DC2 Present
pub const SYSCTL_DC9_ADC0DC1 = usize(0x00000002);  // ADC0 DC1 Present
pub const SYSCTL_DC9_ADC0DC0 = usize(0x00000001);  // ADC0 DC0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_NVMSTAT register.
//
//*****************************************************************************
pub const SYSCTL_NVMSTAT_FWB = usize(0x00000001);  // 32 Word Flash Write Buffer
                                            // Available

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_LDOSPCTL
// register.
//
//*****************************************************************************
pub const SYSCTL_LDOSPCTL_VADJEN = usize(0x80000000);  // Voltage Adjust Enable
pub const SYSCTL_LDOSPCTL_VLDO_M = usize(0x000000FF);  // LDO Output Voltage
pub const SYSCTL_LDOSPCTL_VLDO_0_90V = usize(0x00000012);  // 0.90 V
pub const SYSCTL_LDOSPCTL_VLDO_0_95V = usize(0x00000013);  // 0.95 V
pub const SYSCTL_LDOSPCTL_VLDO_1_00V = usize(0x00000014);  // 1.00 V
pub const SYSCTL_LDOSPCTL_VLDO_1_05V = usize(0x00000015);  // 1.05 V
pub const SYSCTL_LDOSPCTL_VLDO_1_10V = usize(0x00000016);  // 1.10 V
pub const SYSCTL_LDOSPCTL_VLDO_1_15V = usize(0x00000017);  // 1.15 V
pub const SYSCTL_LDOSPCTL_VLDO_1_20V = usize(0x00000018);  // 1.20 V

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_LDODPCTL
// register.
//
//*****************************************************************************
pub const SYSCTL_LDODPCTL_VADJEN = usize(0x80000000);  // Voltage Adjust Enable
pub const SYSCTL_LDODPCTL_VLDO_M = usize(0x000000FF);  // LDO Output Voltage
pub const SYSCTL_LDODPCTL_VLDO_0_90V = usize(0x00000012);  // 0.90 V
pub const SYSCTL_LDODPCTL_VLDO_0_95V = usize(0x00000013);  // 0.95 V
pub const SYSCTL_LDODPCTL_VLDO_1_00V = usize(0x00000014);  // 1.00 V
pub const SYSCTL_LDODPCTL_VLDO_1_05V = usize(0x00000015);  // 1.05 V
pub const SYSCTL_LDODPCTL_VLDO_1_10V = usize(0x00000016);  // 1.10 V
pub const SYSCTL_LDODPCTL_VLDO_1_15V = usize(0x00000017);  // 1.15 V
pub const SYSCTL_LDODPCTL_VLDO_1_20V = usize(0x00000018);  // 1.20 V
pub const SYSCTL_LDODPCTL_VLDO_1_25V = usize(0x00000019);  // 1.25 V
pub const SYSCTL_LDODPCTL_VLDO_1_30V = usize(0x0000001A);  // 1.30 V
pub const SYSCTL_LDODPCTL_VLDO_1_35V = usize(0x0000001B);  // 1.35 V

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RESBEHAVCTL
// register.
//
//*****************************************************************************
pub const SYSCTL_RESBEHAVCTL_WDOG1_M = usize(0x000000C0);  // Watchdog 1 Reset Operation
pub const SYSCTL_RESBEHAVCTL_WDOG1_SYSRST = usize(0x00000080);  // Watchdog 1 issues a system
                                            // reset. The application starts
                                            // within 10 us
pub const SYSCTL_RESBEHAVCTL_WDOG1_POR = usize(0x000000C0);  // Watchdog 1 issues a simulated
                                            // POR sequence. Application starts
                                            // less than 500 us after
                                            // deassertion (Default)
pub const SYSCTL_RESBEHAVCTL_WDOG0_M = usize(0x00000030);  // Watchdog 0 Reset Operation
pub const SYSCTL_RESBEHAVCTL_WDOG0_SYSRST = usize(0x00000020);  // Watchdog 0 issues a system
                                            // reset. The application starts
                                            // within 10 us
pub const SYSCTL_RESBEHAVCTL_WDOG0_POR = usize(0x00000030);  // Watchdog 0 issues a simulated
                                            // POR sequence. Application starts
                                            // less than 500 us after
                                            // deassertion (Default)
pub const SYSCTL_RESBEHAVCTL_BOR_M = usize(0x0000000C);  // BOR Reset operation
pub const SYSCTL_RESBEHAVCTL_BOR_SYSRST = usize(0x00000008);  // Brown Out Reset issues system
                                            // reset. The application starts
                                            // within 10 us
pub const SYSCTL_RESBEHAVCTL_BOR_POR = usize(0x0000000C);  // Brown Out Reset issues a
                                            // simulated POR sequence. The
                                            // application starts less than 500
                                            // us after deassertion (Default)
pub const SYSCTL_RESBEHAVCTL_EXTRES_M = usize(0x00000003);  // External RST Pin Operation
pub const SYSCTL_RESBEHAVCTL_EXTRES_SYSRST = usize(0x00000002);  // External RST assertion issues a
                                            // system reset. The application
                                            // starts within 10 us
pub const SYSCTL_RESBEHAVCTL_EXTRES_POR = usize(0x00000003);  // External RST assertion issues a
                                            // simulated POR sequence.
                                            // Application starts less than 500
                                            // us after deassertion (Default)

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_HSSR register.
//
//*****************************************************************************
pub const SYSCTL_HSSR_KEY_M = usize(0xFF000000);  // Write Key
pub const SYSCTL_HSSR_CDOFF_M = usize(0x00FFFFFF);  // Command Descriptor Pointer
pub const SYSCTL_HSSR_KEY_S = usize(24);
pub const SYSCTL_HSSR_CDOFF_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_USBPDS register.
//
//*****************************************************************************
pub const SYSCTL_USBPDS_MEMSTAT_M = usize(0x0000000C);  // Memory Array Power Status
pub const SYSCTL_USBPDS_MEMSTAT_OFF = usize(0x00000000);  // Array OFF
pub const SYSCTL_USBPDS_MEMSTAT_RETAIN = usize(0x00000004);  // SRAM Retention
pub const SYSCTL_USBPDS_MEMSTAT_ON = usize(0x0000000C);  // Array On
pub const SYSCTL_USBPDS_PWRSTAT_M = usize(0x00000003);  // Power Domain Status
pub const SYSCTL_USBPDS_PWRSTAT_OFF = usize(0x00000000);  // OFF
pub const SYSCTL_USBPDS_PWRSTAT_ON = usize(0x00000003);  // ON

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_USBMPC register.
//
//*****************************************************************************
pub const SYSCTL_USBMPC_PWRCTL_M = usize(0x00000003);  // Memory Array Power Control
pub const SYSCTL_USBMPC_PWRCTL_OFF = usize(0x00000000);  // Array OFF
pub const SYSCTL_USBMPC_PWRCTL_RETAIN = usize(0x00000001);  // SRAM Retention
pub const SYSCTL_USBMPC_PWRCTL_ON = usize(0x00000003);  // Array On

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_EMACPDS register.
//
//*****************************************************************************
pub const SYSCTL_EMACPDS_MEMSTAT_M = usize(0x0000000C);  // Memory Array Power Status
pub const SYSCTL_EMACPDS_MEMSTAT_OFF = usize(0x00000000);  // Array OFF
pub const SYSCTL_EMACPDS_MEMSTAT_ON = usize(0x0000000C);  // Array On
pub const SYSCTL_EMACPDS_PWRSTAT_M = usize(0x00000003);  // Power Domain Status
pub const SYSCTL_EMACPDS_PWRSTAT_OFF = usize(0x00000000);  // OFF
pub const SYSCTL_EMACPDS_PWRSTAT_ON = usize(0x00000003);  // ON

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_EMACMPC register.
//
//*****************************************************************************
pub const SYSCTL_EMACMPC_PWRCTL_M = usize(0x00000003);  // Memory Array Power Control
pub const SYSCTL_EMACMPC_PWRCTL_OFF = usize(0x00000000);  // Array OFF
pub const SYSCTL_EMACMPC_PWRCTL_ON = usize(0x00000003);  // Array On

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_LCDMPC register.
//
//*****************************************************************************
pub const SYSCTL_LCDMPC_PWRCTL_M = usize(0x00000003);  // Memory Array Power Control
pub const SYSCTL_LCDMPC_PWRCTL_OFF = usize(0x00000000);  // Array OFF
pub const SYSCTL_LCDMPC_PWRCTL_ON = usize(0x00000003);  // Array On

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPWD register.
//
//*****************************************************************************
pub const SYSCTL_PPWD_P1 = usize(0x00000002);  // Watchdog Timer 1 Present
pub const SYSCTL_PPWD_P0 = usize(0x00000001);  // Watchdog Timer 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPTIMER register.
//
//*****************************************************************************
pub const SYSCTL_PPTIMER_P7 = usize(0x00000080);  // 16/32-Bit General-Purpose Timer
                                            // 7 Present
pub const SYSCTL_PPTIMER_P6 = usize(0x00000040);  // 16/32-Bit General-Purpose Timer
                                            // 6 Present
pub const SYSCTL_PPTIMER_P5 = usize(0x00000020);  // 16/32-Bit General-Purpose Timer
                                            // 5 Present
pub const SYSCTL_PPTIMER_P4 = usize(0x00000010);  // 16/32-Bit General-Purpose Timer
                                            // 4 Present
pub const SYSCTL_PPTIMER_P3 = usize(0x00000008);  // 16/32-Bit General-Purpose Timer
                                            // 3 Present
pub const SYSCTL_PPTIMER_P2 = usize(0x00000004);  // 16/32-Bit General-Purpose Timer
                                            // 2 Present
pub const SYSCTL_PPTIMER_P1 = usize(0x00000002);  // 16/32-Bit General-Purpose Timer
                                            // 1 Present
pub const SYSCTL_PPTIMER_P0 = usize(0x00000001);  // 16/32-Bit General-Purpose Timer
                                            // 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPGPIO register.
//
//*****************************************************************************
pub const SYSCTL_PPGPIO_P17 = usize(0x00020000);  // GPIO Port T Present
pub const SYSCTL_PPGPIO_P16 = usize(0x00010000);  // GPIO Port S Present
pub const SYSCTL_PPGPIO_P15 = usize(0x00008000);  // GPIO Port R Present
pub const SYSCTL_PPGPIO_P14 = usize(0x00004000);  // GPIO Port Q Present
pub const SYSCTL_PPGPIO_P13 = usize(0x00002000);  // GPIO Port P Present
pub const SYSCTL_PPGPIO_P12 = usize(0x00001000);  // GPIO Port N Present
pub const SYSCTL_PPGPIO_P11 = usize(0x00000800);  // GPIO Port M Present
pub const SYSCTL_PPGPIO_P10 = usize(0x00000400);  // GPIO Port L Present
pub const SYSCTL_PPGPIO_P9 = usize(0x00000200);  // GPIO Port K Present
pub const SYSCTL_PPGPIO_P8 = usize(0x00000100);  // GPIO Port J Present
pub const SYSCTL_PPGPIO_P7 = usize(0x00000080);  // GPIO Port H Present
pub const SYSCTL_PPGPIO_P6 = usize(0x00000040);  // GPIO Port G Present
pub const SYSCTL_PPGPIO_P5 = usize(0x00000020);  // GPIO Port F Present
pub const SYSCTL_PPGPIO_P4 = usize(0x00000010);  // GPIO Port E Present
pub const SYSCTL_PPGPIO_P3 = usize(0x00000008);  // GPIO Port D Present
pub const SYSCTL_PPGPIO_P2 = usize(0x00000004);  // GPIO Port C Present
pub const SYSCTL_PPGPIO_P1 = usize(0x00000002);  // GPIO Port B Present
pub const SYSCTL_PPGPIO_P0 = usize(0x00000001);  // GPIO Port A Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPDMA register.
//
//*****************************************************************************
pub const SYSCTL_PPDMA_P0 = usize(0x00000001);  // uDMA Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPEPI register.
//
//*****************************************************************************
pub const SYSCTL_PPEPI_P0 = usize(0x00000001);  // EPI Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPHIB register.
//
//*****************************************************************************
pub const SYSCTL_PPHIB_P0 = usize(0x00000001);  // Hibernation Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPUART register.
//
//*****************************************************************************
pub const SYSCTL_PPUART_P7 = usize(0x00000080);  // UART Module 7 Present
pub const SYSCTL_PPUART_P6 = usize(0x00000040);  // UART Module 6 Present
pub const SYSCTL_PPUART_P5 = usize(0x00000020);  // UART Module 5 Present
pub const SYSCTL_PPUART_P4 = usize(0x00000010);  // UART Module 4 Present
pub const SYSCTL_PPUART_P3 = usize(0x00000008);  // UART Module 3 Present
pub const SYSCTL_PPUART_P2 = usize(0x00000004);  // UART Module 2 Present
pub const SYSCTL_PPUART_P1 = usize(0x00000002);  // UART Module 1 Present
pub const SYSCTL_PPUART_P0 = usize(0x00000001);  // UART Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPSSI register.
//
//*****************************************************************************
pub const SYSCTL_PPSSI_P3 = usize(0x00000008);  // SSI Module 3 Present
pub const SYSCTL_PPSSI_P2 = usize(0x00000004);  // SSI Module 2 Present
pub const SYSCTL_PPSSI_P1 = usize(0x00000002);  // SSI Module 1 Present
pub const SYSCTL_PPSSI_P0 = usize(0x00000001);  // SSI Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPI2C register.
//
//*****************************************************************************
pub const SYSCTL_PPI2C_P9 = usize(0x00000200);  // I2C Module 9 Present
pub const SYSCTL_PPI2C_P8 = usize(0x00000100);  // I2C Module 8 Present
pub const SYSCTL_PPI2C_P7 = usize(0x00000080);  // I2C Module 7 Present
pub const SYSCTL_PPI2C_P6 = usize(0x00000040);  // I2C Module 6 Present
pub const SYSCTL_PPI2C_P5 = usize(0x00000020);  // I2C Module 5 Present
pub const SYSCTL_PPI2C_P4 = usize(0x00000010);  // I2C Module 4 Present
pub const SYSCTL_PPI2C_P3 = usize(0x00000008);  // I2C Module 3 Present
pub const SYSCTL_PPI2C_P2 = usize(0x00000004);  // I2C Module 2 Present
pub const SYSCTL_PPI2C_P1 = usize(0x00000002);  // I2C Module 1 Present
pub const SYSCTL_PPI2C_P0 = usize(0x00000001);  // I2C Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPUSB register.
//
//*****************************************************************************
pub const SYSCTL_PPUSB_P0 = usize(0x00000001);  // USB Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPEPHY register.
//
//*****************************************************************************
pub const SYSCTL_PPEPHY_P0 = usize(0x00000001);  // Ethernet PHY Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPCAN register.
//
//*****************************************************************************
pub const SYSCTL_PPCAN_P1 = usize(0x00000002);  // CAN Module 1 Present
pub const SYSCTL_PPCAN_P0 = usize(0x00000001);  // CAN Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPADC register.
//
//*****************************************************************************
pub const SYSCTL_PPADC_P1 = usize(0x00000002);  // ADC Module 1 Present
pub const SYSCTL_PPADC_P0 = usize(0x00000001);  // ADC Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPACMP register.
//
//*****************************************************************************
pub const SYSCTL_PPACMP_P0 = usize(0x00000001);  // Analog Comparator Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPPWM register.
//
//*****************************************************************************
pub const SYSCTL_PPPWM_P1 = usize(0x00000002);  // PWM Module 1 Present
pub const SYSCTL_PPPWM_P0 = usize(0x00000001);  // PWM Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPQEI register.
//
//*****************************************************************************
pub const SYSCTL_PPQEI_P1 = usize(0x00000002);  // QEI Module 1 Present
pub const SYSCTL_PPQEI_P0 = usize(0x00000001);  // QEI Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPLPC register.
//
//*****************************************************************************
pub const SYSCTL_PPLPC_P0 = usize(0x00000001);  // LPC Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPPECI register.
//
//*****************************************************************************
pub const SYSCTL_PPPECI_P0 = usize(0x00000001);  // PECI Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPFAN register.
//
//*****************************************************************************
pub const SYSCTL_PPFAN_P0 = usize(0x00000001);  // FAN Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPEEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_PPEEPROM_P0 = usize(0x00000001);  // EEPROM Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_PPWTIMER_P5 = usize(0x00000020);  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Present
pub const SYSCTL_PPWTIMER_P4 = usize(0x00000010);  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Present
pub const SYSCTL_PPWTIMER_P3 = usize(0x00000008);  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Present
pub const SYSCTL_PPWTIMER_P2 = usize(0x00000004);  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Present
pub const SYSCTL_PPWTIMER_P1 = usize(0x00000002);  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Present
pub const SYSCTL_PPWTIMER_P0 = usize(0x00000001);  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPRTS register.
//
//*****************************************************************************
pub const SYSCTL_PPRTS_P0 = usize(0x00000001);  // RTS Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPCCM register.
//
//*****************************************************************************
pub const SYSCTL_PPCCM_P0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPLCD register.
//
//*****************************************************************************
pub const SYSCTL_PPLCD_P0 = usize(0x00000001);  // LCD Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPOWIRE register.
//
//*****************************************************************************
pub const SYSCTL_PPOWIRE_P0 = usize(0x00000001);  // 1-Wire Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPEMAC register.
//
//*****************************************************************************
pub const SYSCTL_PPEMAC_P0 = usize(0x00000001);  // Ethernet Controller Module
                                            // Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPHIM register.
//
//*****************************************************************************
pub const SYSCTL_PPHIM_P0 = usize(0x00000001);  // HIM Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRWD register.
//
//*****************************************************************************
pub const SYSCTL_SRWD_R1 = usize(0x00000002);  // Watchdog Timer 1 Software Reset
pub const SYSCTL_SRWD_R0 = usize(0x00000001);  // Watchdog Timer 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRTIMER register.
//
//*****************************************************************************
pub const SYSCTL_SRTIMER_R7 = usize(0x00000080);  // 16/32-Bit General-Purpose Timer
                                            // 7 Software Reset
pub const SYSCTL_SRTIMER_R6 = usize(0x00000040);  // 16/32-Bit General-Purpose Timer
                                            // 6 Software Reset
pub const SYSCTL_SRTIMER_R5 = usize(0x00000020);  // 16/32-Bit General-Purpose Timer
                                            // 5 Software Reset
pub const SYSCTL_SRTIMER_R4 = usize(0x00000010);  // 16/32-Bit General-Purpose Timer
                                            // 4 Software Reset
pub const SYSCTL_SRTIMER_R3 = usize(0x00000008);  // 16/32-Bit General-Purpose Timer
                                            // 3 Software Reset
pub const SYSCTL_SRTIMER_R2 = usize(0x00000004);  // 16/32-Bit General-Purpose Timer
                                            // 2 Software Reset
pub const SYSCTL_SRTIMER_R1 = usize(0x00000002);  // 16/32-Bit General-Purpose Timer
                                            // 1 Software Reset
pub const SYSCTL_SRTIMER_R0 = usize(0x00000001);  // 16/32-Bit General-Purpose Timer
                                            // 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRGPIO register.
//
//*****************************************************************************
pub const SYSCTL_SRGPIO_R17 = usize(0x00020000);  // GPIO Port T Software Reset
pub const SYSCTL_SRGPIO_R16 = usize(0x00010000);  // GPIO Port S Software Reset
pub const SYSCTL_SRGPIO_R15 = usize(0x00008000);  // GPIO Port R Software Reset
pub const SYSCTL_SRGPIO_R14 = usize(0x00004000);  // GPIO Port Q Software Reset
pub const SYSCTL_SRGPIO_R13 = usize(0x00002000);  // GPIO Port P Software Reset
pub const SYSCTL_SRGPIO_R12 = usize(0x00001000);  // GPIO Port N Software Reset
pub const SYSCTL_SRGPIO_R11 = usize(0x00000800);  // GPIO Port M Software Reset
pub const SYSCTL_SRGPIO_R10 = usize(0x00000400);  // GPIO Port L Software Reset
pub const SYSCTL_SRGPIO_R9 = usize(0x00000200);  // GPIO Port K Software Reset
pub const SYSCTL_SRGPIO_R8 = usize(0x00000100);  // GPIO Port J Software Reset
pub const SYSCTL_SRGPIO_R7 = usize(0x00000080);  // GPIO Port H Software Reset
pub const SYSCTL_SRGPIO_R6 = usize(0x00000040);  // GPIO Port G Software Reset
pub const SYSCTL_SRGPIO_R5 = usize(0x00000020);  // GPIO Port F Software Reset
pub const SYSCTL_SRGPIO_R4 = usize(0x00000010);  // GPIO Port E Software Reset
pub const SYSCTL_SRGPIO_R3 = usize(0x00000008);  // GPIO Port D Software Reset
pub const SYSCTL_SRGPIO_R2 = usize(0x00000004);  // GPIO Port C Software Reset
pub const SYSCTL_SRGPIO_R1 = usize(0x00000002);  // GPIO Port B Software Reset
pub const SYSCTL_SRGPIO_R0 = usize(0x00000001);  // GPIO Port A Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRDMA register.
//
//*****************************************************************************
pub const SYSCTL_SRDMA_R0 = usize(0x00000001);  // uDMA Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SREPI register.
//
//*****************************************************************************
pub const SYSCTL_SREPI_R0 = usize(0x00000001);  // EPI Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRHIB register.
//
//*****************************************************************************
pub const SYSCTL_SRHIB_R0 = usize(0x00000001);  // Hibernation Module Software
                                            // Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRUART register.
//
//*****************************************************************************
pub const SYSCTL_SRUART_R7 = usize(0x00000080);  // UART Module 7 Software Reset
pub const SYSCTL_SRUART_R6 = usize(0x00000040);  // UART Module 6 Software Reset
pub const SYSCTL_SRUART_R5 = usize(0x00000020);  // UART Module 5 Software Reset
pub const SYSCTL_SRUART_R4 = usize(0x00000010);  // UART Module 4 Software Reset
pub const SYSCTL_SRUART_R3 = usize(0x00000008);  // UART Module 3 Software Reset
pub const SYSCTL_SRUART_R2 = usize(0x00000004);  // UART Module 2 Software Reset
pub const SYSCTL_SRUART_R1 = usize(0x00000002);  // UART Module 1 Software Reset
pub const SYSCTL_SRUART_R0 = usize(0x00000001);  // UART Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRSSI register.
//
//*****************************************************************************
pub const SYSCTL_SRSSI_R3 = usize(0x00000008);  // SSI Module 3 Software Reset
pub const SYSCTL_SRSSI_R2 = usize(0x00000004);  // SSI Module 2 Software Reset
pub const SYSCTL_SRSSI_R1 = usize(0x00000002);  // SSI Module 1 Software Reset
pub const SYSCTL_SRSSI_R0 = usize(0x00000001);  // SSI Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRI2C register.
//
//*****************************************************************************
pub const SYSCTL_SRI2C_R9 = usize(0x00000200);  // I2C Module 9 Software Reset
pub const SYSCTL_SRI2C_R8 = usize(0x00000100);  // I2C Module 8 Software Reset
pub const SYSCTL_SRI2C_R7 = usize(0x00000080);  // I2C Module 7 Software Reset
pub const SYSCTL_SRI2C_R6 = usize(0x00000040);  // I2C Module 6 Software Reset
pub const SYSCTL_SRI2C_R5 = usize(0x00000020);  // I2C Module 5 Software Reset
pub const SYSCTL_SRI2C_R4 = usize(0x00000010);  // I2C Module 4 Software Reset
pub const SYSCTL_SRI2C_R3 = usize(0x00000008);  // I2C Module 3 Software Reset
pub const SYSCTL_SRI2C_R2 = usize(0x00000004);  // I2C Module 2 Software Reset
pub const SYSCTL_SRI2C_R1 = usize(0x00000002);  // I2C Module 1 Software Reset
pub const SYSCTL_SRI2C_R0 = usize(0x00000001);  // I2C Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRUSB register.
//
//*****************************************************************************
pub const SYSCTL_SRUSB_R0 = usize(0x00000001);  // USB Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SREPHY register.
//
//*****************************************************************************
pub const SYSCTL_SREPHY_R0 = usize(0x00000001);  // Ethernet PHY Module Software
                                            // Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCAN register.
//
//*****************************************************************************
pub const SYSCTL_SRCAN_R1 = usize(0x00000002);  // CAN Module 1 Software Reset
pub const SYSCTL_SRCAN_R0 = usize(0x00000001);  // CAN Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRADC register.
//
//*****************************************************************************
pub const SYSCTL_SRADC_R1 = usize(0x00000002);  // ADC Module 1 Software Reset
pub const SYSCTL_SRADC_R0 = usize(0x00000001);  // ADC Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRACMP register.
//
//*****************************************************************************
pub const SYSCTL_SRACMP_R0 = usize(0x00000001);  // Analog Comparator Module 0
                                            // Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRPWM register.
//
//*****************************************************************************
pub const SYSCTL_SRPWM_R1 = usize(0x00000002);  // PWM Module 1 Software Reset
pub const SYSCTL_SRPWM_R0 = usize(0x00000001);  // PWM Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRQEI register.
//
//*****************************************************************************
pub const SYSCTL_SRQEI_R1 = usize(0x00000002);  // QEI Module 1 Software Reset
pub const SYSCTL_SRQEI_R0 = usize(0x00000001);  // QEI Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SREEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_SREEPROM_R0 = usize(0x00000001);  // EEPROM Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_SRWTIMER_R5 = usize(0x00000020);  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Software Reset
pub const SYSCTL_SRWTIMER_R4 = usize(0x00000010);  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Software Reset
pub const SYSCTL_SRWTIMER_R3 = usize(0x00000008);  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Software Reset
pub const SYSCTL_SRWTIMER_R2 = usize(0x00000004);  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Software Reset
pub const SYSCTL_SRWTIMER_R1 = usize(0x00000002);  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Software Reset
pub const SYSCTL_SRWTIMER_R0 = usize(0x00000001);  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCCM register.
//
//*****************************************************************************
pub const SYSCTL_SRCCM_R0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRLCD register.
//
//*****************************************************************************
pub const SYSCTL_SRLCD_R0 = usize(0x00000001);  // LCD Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SROWIRE register.
//
//*****************************************************************************
pub const SYSCTL_SROWIRE_R0 = usize(0x00000001);  // 1-Wire Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SREMAC register.
//
//*****************************************************************************
pub const SYSCTL_SREMAC_R0 = usize(0x00000001);  // Ethernet Controller MAC Module 0
                                            // Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCWD register.
//
//*****************************************************************************
pub const SYSCTL_RCGCWD_R1 = usize(0x00000002);  // Watchdog Timer 1 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCWD_R0 = usize(0x00000001);  // Watchdog Timer 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCTIMER_R7 = usize(0x00000080);  // 16/32-Bit General-Purpose Timer
                                            // 7 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R6 = usize(0x00000040);  // 16/32-Bit General-Purpose Timer
                                            // 6 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R5 = usize(0x00000020);  // 16/32-Bit General-Purpose Timer
                                            // 5 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R4 = usize(0x00000010);  // 16/32-Bit General-Purpose Timer
                                            // 4 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R3 = usize(0x00000008);  // 16/32-Bit General-Purpose Timer
                                            // 3 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R2 = usize(0x00000004);  // 16/32-Bit General-Purpose Timer
                                            // 2 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R1 = usize(0x00000002);  // 16/32-Bit General-Purpose Timer
                                            // 1 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R0 = usize(0x00000001);  // 16/32-Bit General-Purpose Timer
                                            // 0 Run Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCGPIO
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCGPIO_R17 = usize(0x00020000);  // GPIO Port T Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R16 = usize(0x00010000);  // GPIO Port S Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R15 = usize(0x00008000);  // GPIO Port R Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R14 = usize(0x00004000);  // GPIO Port Q Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R13 = usize(0x00002000);  // GPIO Port P Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R12 = usize(0x00001000);  // GPIO Port N Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R11 = usize(0x00000800);  // GPIO Port M Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R10 = usize(0x00000400);  // GPIO Port L Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R9 = usize(0x00000200);  // GPIO Port K Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R8 = usize(0x00000100);  // GPIO Port J Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R7 = usize(0x00000080);  // GPIO Port H Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R6 = usize(0x00000040);  // GPIO Port G Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R5 = usize(0x00000020);  // GPIO Port F Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R4 = usize(0x00000010);  // GPIO Port E Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R3 = usize(0x00000008);  // GPIO Port D Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R2 = usize(0x00000004);  // GPIO Port C Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R1 = usize(0x00000002);  // GPIO Port B Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCGPIO_R0 = usize(0x00000001);  // GPIO Port A Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCDMA register.
//
//*****************************************************************************
pub const SYSCTL_RCGCDMA_R0 = usize(0x00000001);  // uDMA Module Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCEPI register.
//
//*****************************************************************************
pub const SYSCTL_RCGCEPI_R0 = usize(0x00000001);  // EPI Module Run Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCHIB register.
//
//*****************************************************************************
pub const SYSCTL_RCGCHIB_R0 = usize(0x00000001);  // Hibernation Module Run Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCUART
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCUART_R7 = usize(0x00000080);  // UART Module 7 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCUART_R6 = usize(0x00000040);  // UART Module 6 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCUART_R5 = usize(0x00000020);  // UART Module 5 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCUART_R4 = usize(0x00000010);  // UART Module 4 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCUART_R3 = usize(0x00000008);  // UART Module 3 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCUART_R2 = usize(0x00000004);  // UART Module 2 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCUART_R1 = usize(0x00000002);  // UART Module 1 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCUART_R0 = usize(0x00000001);  // UART Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCSSI register.
//
//*****************************************************************************
pub const SYSCTL_RCGCSSI_R3 = usize(0x00000008);  // SSI Module 3 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCSSI_R2 = usize(0x00000004);  // SSI Module 2 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCSSI_R1 = usize(0x00000002);  // SSI Module 1 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCSSI_R0 = usize(0x00000001);  // SSI Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCI2C register.
//
//*****************************************************************************
pub const SYSCTL_RCGCI2C_R9 = usize(0x00000200);  // I2C Module 9 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCI2C_R8 = usize(0x00000100);  // I2C Module 8 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCI2C_R7 = usize(0x00000080);  // I2C Module 7 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCI2C_R6 = usize(0x00000040);  // I2C Module 6 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCI2C_R5 = usize(0x00000020);  // I2C Module 5 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCI2C_R4 = usize(0x00000010);  // I2C Module 4 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCI2C_R3 = usize(0x00000008);  // I2C Module 3 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCI2C_R2 = usize(0x00000004);  // I2C Module 2 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCI2C_R1 = usize(0x00000002);  // I2C Module 1 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCI2C_R0 = usize(0x00000001);  // I2C Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCUSB register.
//
//*****************************************************************************
pub const SYSCTL_RCGCUSB_R0 = usize(0x00000001);  // USB Module Run Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCEPHY
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCEPHY_R0 = usize(0x00000001);  // Ethernet PHY Module Run Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCCAN register.
//
//*****************************************************************************
pub const SYSCTL_RCGCCAN_R1 = usize(0x00000002);  // CAN Module 1 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCCAN_R0 = usize(0x00000001);  // CAN Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCADC register.
//
//*****************************************************************************
pub const SYSCTL_RCGCADC_R1 = usize(0x00000002);  // ADC Module 1 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCADC_R0 = usize(0x00000001);  // ADC Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCACMP
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCACMP_R0 = usize(0x00000001);  // Analog Comparator Module 0 Run
                                            // Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCPWM register.
//
//*****************************************************************************
pub const SYSCTL_RCGCPWM_R1 = usize(0x00000002);  // PWM Module 1 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCPWM_R0 = usize(0x00000001);  // PWM Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCQEI register.
//
//*****************************************************************************
pub const SYSCTL_RCGCQEI_R1 = usize(0x00000002);  // QEI Module 1 Run Mode Clock
                                            // Gating Control
pub const SYSCTL_RCGCQEI_R0 = usize(0x00000001);  // QEI Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCEEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCEEPROM_R0 = usize(0x00000001);  // EEPROM Module Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCWTIMER_R5 = usize(0x00000020);  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Run Mode Clock Gating
                                            // Control
pub const SYSCTL_RCGCWTIMER_R4 = usize(0x00000010);  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Run Mode Clock Gating
                                            // Control
pub const SYSCTL_RCGCWTIMER_R3 = usize(0x00000008);  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Run Mode Clock Gating
                                            // Control
pub const SYSCTL_RCGCWTIMER_R2 = usize(0x00000004);  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Run Mode Clock Gating
                                            // Control
pub const SYSCTL_RCGCWTIMER_R1 = usize(0x00000002);  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Run Mode Clock Gating
                                            // Control
pub const SYSCTL_RCGCWTIMER_R0 = usize(0x00000001);  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Run Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCCCM register.
//
//*****************************************************************************
pub const SYSCTL_RCGCCCM_R0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Run Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCLCD register.
//
//*****************************************************************************
pub const SYSCTL_RCGCLCD_R0 = usize(0x00000001);  // LCD Controller Module 0 Run Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCOWIRE
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCOWIRE_R0 = usize(0x00000001);  // 1-Wire Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCEMAC
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCEMAC_R0 = usize(0x00000001);  // Ethernet MAC Module 0 Run Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCWD register.
//
//*****************************************************************************
pub const SYSCTL_SCGCWD_S1 = usize(0x00000002);  // Watchdog Timer 1 Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_SCGCWD_S0 = usize(0x00000001);  // Watchdog Timer 0 Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCTIMER_S7 = usize(0x00000080);  // 16/32-Bit General-Purpose Timer
                                            // 7 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCTIMER_S6 = usize(0x00000040);  // 16/32-Bit General-Purpose Timer
                                            // 6 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCTIMER_S5 = usize(0x00000020);  // 16/32-Bit General-Purpose Timer
                                            // 5 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCTIMER_S4 = usize(0x00000010);  // 16/32-Bit General-Purpose Timer
                                            // 4 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCTIMER_S3 = usize(0x00000008);  // 16/32-Bit General-Purpose Timer
                                            // 3 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCTIMER_S2 = usize(0x00000004);  // 16/32-Bit General-Purpose Timer
                                            // 2 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCTIMER_S1 = usize(0x00000002);  // 16/32-Bit General-Purpose Timer
                                            // 1 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCTIMER_S0 = usize(0x00000001);  // 16/32-Bit General-Purpose Timer
                                            // 0 Sleep Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCGPIO
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCGPIO_S17 = usize(0x00020000);  // GPIO Port T Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S16 = usize(0x00010000);  // GPIO Port S Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S15 = usize(0x00008000);  // GPIO Port R Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S14 = usize(0x00004000);  // GPIO Port Q Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S13 = usize(0x00002000);  // GPIO Port P Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S12 = usize(0x00001000);  // GPIO Port N Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S11 = usize(0x00000800);  // GPIO Port M Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S10 = usize(0x00000400);  // GPIO Port L Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S9 = usize(0x00000200);  // GPIO Port K Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S8 = usize(0x00000100);  // GPIO Port J Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S7 = usize(0x00000080);  // GPIO Port H Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S6 = usize(0x00000040);  // GPIO Port G Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S5 = usize(0x00000020);  // GPIO Port F Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S4 = usize(0x00000010);  // GPIO Port E Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S3 = usize(0x00000008);  // GPIO Port D Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S2 = usize(0x00000004);  // GPIO Port C Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S1 = usize(0x00000002);  // GPIO Port B Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCGPIO_S0 = usize(0x00000001);  // GPIO Port A Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCDMA register.
//
//*****************************************************************************
pub const SYSCTL_SCGCDMA_S0 = usize(0x00000001);  // uDMA Module Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCEPI register.
//
//*****************************************************************************
pub const SYSCTL_SCGCEPI_S0 = usize(0x00000001);  // EPI Module Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCHIB register.
//
//*****************************************************************************
pub const SYSCTL_SCGCHIB_S0 = usize(0x00000001);  // Hibernation Module Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCUART
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCUART_S7 = usize(0x00000080);  // UART Module 7 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCUART_S6 = usize(0x00000040);  // UART Module 6 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCUART_S5 = usize(0x00000020);  // UART Module 5 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCUART_S4 = usize(0x00000010);  // UART Module 4 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCUART_S3 = usize(0x00000008);  // UART Module 3 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCUART_S2 = usize(0x00000004);  // UART Module 2 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCUART_S1 = usize(0x00000002);  // UART Module 1 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCUART_S0 = usize(0x00000001);  // UART Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCSSI register.
//
//*****************************************************************************
pub const SYSCTL_SCGCSSI_S3 = usize(0x00000008);  // SSI Module 3 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCSSI_S2 = usize(0x00000004);  // SSI Module 2 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCSSI_S1 = usize(0x00000002);  // SSI Module 1 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCSSI_S0 = usize(0x00000001);  // SSI Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCI2C register.
//
//*****************************************************************************
pub const SYSCTL_SCGCI2C_S9 = usize(0x00000200);  // I2C Module 9 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCI2C_S8 = usize(0x00000100);  // I2C Module 8 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCI2C_S7 = usize(0x00000080);  // I2C Module 7 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCI2C_S6 = usize(0x00000040);  // I2C Module 6 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCI2C_S5 = usize(0x00000020);  // I2C Module 5 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCI2C_S4 = usize(0x00000010);  // I2C Module 4 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCI2C_S3 = usize(0x00000008);  // I2C Module 3 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCI2C_S2 = usize(0x00000004);  // I2C Module 2 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCI2C_S1 = usize(0x00000002);  // I2C Module 1 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCI2C_S0 = usize(0x00000001);  // I2C Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCUSB register.
//
//*****************************************************************************
pub const SYSCTL_SCGCUSB_S0 = usize(0x00000001);  // USB Module Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCEPHY
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCEPHY_S0 = usize(0x00000001);  // PHY Module Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCCAN register.
//
//*****************************************************************************
pub const SYSCTL_SCGCCAN_S1 = usize(0x00000002);  // CAN Module 1 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCCAN_S0 = usize(0x00000001);  // CAN Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCADC register.
//
//*****************************************************************************
pub const SYSCTL_SCGCADC_S1 = usize(0x00000002);  // ADC Module 1 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCADC_S0 = usize(0x00000001);  // ADC Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCACMP
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCACMP_S0 = usize(0x00000001);  // Analog Comparator Module 0 Sleep
                                            // Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCPWM register.
//
//*****************************************************************************
pub const SYSCTL_SCGCPWM_S1 = usize(0x00000002);  // PWM Module 1 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCPWM_S0 = usize(0x00000001);  // PWM Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCQEI register.
//
//*****************************************************************************
pub const SYSCTL_SCGCQEI_S1 = usize(0x00000002);  // QEI Module 1 Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_SCGCQEI_S0 = usize(0x00000001);  // QEI Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCEEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCEEPROM_S0 = usize(0x00000001);  // EEPROM Module Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCWTIMER_S5 = usize(0x00000020);  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCWTIMER_S4 = usize(0x00000010);  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCWTIMER_S3 = usize(0x00000008);  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCWTIMER_S2 = usize(0x00000004);  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCWTIMER_S1 = usize(0x00000002);  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_SCGCWTIMER_S0 = usize(0x00000001);  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Sleep Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCCCM register.
//
//*****************************************************************************
pub const SYSCTL_SCGCCCM_S0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Sleep Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCLCD register.
//
//*****************************************************************************
pub const SYSCTL_SCGCLCD_S0 = usize(0x00000001);  // LCD Controller Module 0 Sleep
                                            // Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCOWIRE
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCOWIRE_S0 = usize(0x00000001);  // 1-Wire Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCEMAC
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCEMAC_S0 = usize(0x00000001);  // Ethernet MAC Module 0 Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCWD register.
//
//*****************************************************************************
pub const SYSCTL_DCGCWD_D1 = usize(0x00000002);  // Watchdog Timer 1 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCWD_D0 = usize(0x00000001);  // Watchdog Timer 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCTIMER_D7 = usize(0x00000080);  // 16/32-Bit General-Purpose Timer
                                            // 7 Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCTIMER_D6 = usize(0x00000040);  // 16/32-Bit General-Purpose Timer
                                            // 6 Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCTIMER_D5 = usize(0x00000020);  // 16/32-Bit General-Purpose Timer
                                            // 5 Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCTIMER_D4 = usize(0x00000010);  // 16/32-Bit General-Purpose Timer
                                            // 4 Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCTIMER_D3 = usize(0x00000008);  // 16/32-Bit General-Purpose Timer
                                            // 3 Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCTIMER_D2 = usize(0x00000004);  // 16/32-Bit General-Purpose Timer
                                            // 2 Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCTIMER_D1 = usize(0x00000002);  // 16/32-Bit General-Purpose Timer
                                            // 1 Deep-Sleep Mode Clock Gating
                                            // Control
pub const SYSCTL_DCGCTIMER_D0 = usize(0x00000001);  // 16/32-Bit General-Purpose Timer
                                            // 0 Deep-Sleep Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCGPIO
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCGPIO_D17 = usize(0x00020000);  // GPIO Port T Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D16 = usize(0x00010000);  // GPIO Port S Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D15 = usize(0x00008000);  // GPIO Port R Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D14 = usize(0x00004000);  // GPIO Port Q Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D13 = usize(0x00002000);  // GPIO Port P Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D12 = usize(0x00001000);  // GPIO Port N Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D11 = usize(0x00000800);  // GPIO Port M Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D10 = usize(0x00000400);  // GPIO Port L Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D9 = usize(0x00000200);  // GPIO Port K Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D8 = usize(0x00000100);  // GPIO Port J Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D7 = usize(0x00000080);  // GPIO Port H Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D6 = usize(0x00000040);  // GPIO Port G Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D5 = usize(0x00000020);  // GPIO Port F Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D4 = usize(0x00000010);  // GPIO Port E Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D3 = usize(0x00000008);  // GPIO Port D Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D2 = usize(0x00000004);  // GPIO Port C Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D1 = usize(0x00000002);  // GPIO Port B Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCGPIO_D0 = usize(0x00000001);  // GPIO Port A Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCDMA register.
//
//*****************************************************************************
pub const SYSCTL_DCGCDMA_D0 = usize(0x00000001);  // uDMA Module Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCEPI register.
//
//*****************************************************************************
pub const SYSCTL_DCGCEPI_D0 = usize(0x00000001);  // EPI Module Deep-Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCHIB register.
//
//*****************************************************************************
pub const SYSCTL_DCGCHIB_D0 = usize(0x00000001);  // Hibernation Module Deep-Sleep
                                            // Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCUART
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCUART_D7 = usize(0x00000080);  // UART Module 7 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCUART_D6 = usize(0x00000040);  // UART Module 6 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCUART_D5 = usize(0x00000020);  // UART Module 5 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCUART_D4 = usize(0x00000010);  // UART Module 4 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCUART_D3 = usize(0x00000008);  // UART Module 3 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCUART_D2 = usize(0x00000004);  // UART Module 2 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCUART_D1 = usize(0x00000002);  // UART Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCUART_D0 = usize(0x00000001);  // UART Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCSSI register.
//
//*****************************************************************************
pub const SYSCTL_DCGCSSI_D3 = usize(0x00000008);  // SSI Module 3 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCSSI_D2 = usize(0x00000004);  // SSI Module 2 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCSSI_D1 = usize(0x00000002);  // SSI Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCSSI_D0 = usize(0x00000001);  // SSI Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCI2C register.
//
//*****************************************************************************
pub const SYSCTL_DCGCI2C_D9 = usize(0x00000200);  // I2C Module 9 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCI2C_D8 = usize(0x00000100);  // I2C Module 8 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCI2C_D7 = usize(0x00000080);  // I2C Module 7 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCI2C_D6 = usize(0x00000040);  // I2C Module 6 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCI2C_D5 = usize(0x00000020);  // I2C Module 5 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCI2C_D4 = usize(0x00000010);  // I2C Module 4 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCI2C_D3 = usize(0x00000008);  // I2C Module 3 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCI2C_D2 = usize(0x00000004);  // I2C Module 2 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCI2C_D1 = usize(0x00000002);  // I2C Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCI2C_D0 = usize(0x00000001);  // I2C Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCUSB register.
//
//*****************************************************************************
pub const SYSCTL_DCGCUSB_D0 = usize(0x00000001);  // USB Module Deep-Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCEPHY
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCEPHY_D0 = usize(0x00000001);  // PHY Module Deep-Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCCAN register.
//
//*****************************************************************************
pub const SYSCTL_DCGCCAN_D1 = usize(0x00000002);  // CAN Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCCAN_D0 = usize(0x00000001);  // CAN Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCADC register.
//
//*****************************************************************************
pub const SYSCTL_DCGCADC_D1 = usize(0x00000002);  // ADC Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCADC_D0 = usize(0x00000001);  // ADC Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCACMP
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCACMP_D0 = usize(0x00000001);  // Analog Comparator Module 0
                                            // Deep-Sleep Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCPWM register.
//
//*****************************************************************************
pub const SYSCTL_DCGCPWM_D1 = usize(0x00000002);  // PWM Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCPWM_D0 = usize(0x00000001);  // PWM Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCQEI register.
//
//*****************************************************************************
pub const SYSCTL_DCGCQEI_D1 = usize(0x00000002);  // QEI Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
pub const SYSCTL_DCGCQEI_D0 = usize(0x00000001);  // QEI Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCEEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCEEPROM_D0 = usize(0x00000001);  // EEPROM Module Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCWTIMER_D5 = usize(0x00000020);  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Deep-Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_DCGCWTIMER_D4 = usize(0x00000010);  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Deep-Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_DCGCWTIMER_D3 = usize(0x00000008);  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Deep-Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_DCGCWTIMER_D2 = usize(0x00000004);  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Deep-Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_DCGCWTIMER_D1 = usize(0x00000002);  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Deep-Sleep Mode Clock
                                            // Gating Control
pub const SYSCTL_DCGCWTIMER_D0 = usize(0x00000001);  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Deep-Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCCCM register.
//
//*****************************************************************************
pub const SYSCTL_DCGCCCM_D0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Deep-Sleep Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCLCD register.
//
//*****************************************************************************
pub const SYSCTL_DCGCLCD_D0 = usize(0x00000001);  // LCD Controller Module 0
                                            // Deep-Sleep Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCOWIRE
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCOWIRE_D0 = usize(0x00000001);  // 1-Wire Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCEMAC
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCEMAC_D0 = usize(0x00000001);  // Ethernet MAC Module 0 Deep-Sleep
                                            // Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCWD register.
//
//*****************************************************************************
pub const SYSCTL_PCWD_P1 = usize(0x00000002);  // Watchdog Timer 1 Power Control
pub const SYSCTL_PCWD_P0 = usize(0x00000001);  // Watchdog Timer 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCTIMER register.
//
//*****************************************************************************
pub const SYSCTL_PCTIMER_P7 = usize(0x00000080);  // General-Purpose Timer 7 Power
                                            // Control
pub const SYSCTL_PCTIMER_P6 = usize(0x00000040);  // General-Purpose Timer 6 Power
                                            // Control
pub const SYSCTL_PCTIMER_P5 = usize(0x00000020);  // General-Purpose Timer 5 Power
                                            // Control
pub const SYSCTL_PCTIMER_P4 = usize(0x00000010);  // General-Purpose Timer 4 Power
                                            // Control
pub const SYSCTL_PCTIMER_P3 = usize(0x00000008);  // General-Purpose Timer 3 Power
                                            // Control
pub const SYSCTL_PCTIMER_P2 = usize(0x00000004);  // General-Purpose Timer 2 Power
                                            // Control
pub const SYSCTL_PCTIMER_P1 = usize(0x00000002);  // General-Purpose Timer 1 Power
                                            // Control
pub const SYSCTL_PCTIMER_P0 = usize(0x00000001);  // General-Purpose Timer 0 Power
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCGPIO register.
//
//*****************************************************************************
pub const SYSCTL_PCGPIO_P17 = usize(0x00020000);  // GPIO Port T Power Control
pub const SYSCTL_PCGPIO_P16 = usize(0x00010000);  // GPIO Port S Power Control
pub const SYSCTL_PCGPIO_P15 = usize(0x00008000);  // GPIO Port R Power Control
pub const SYSCTL_PCGPIO_P14 = usize(0x00004000);  // GPIO Port Q Power Control
pub const SYSCTL_PCGPIO_P13 = usize(0x00002000);  // GPIO Port P Power Control
pub const SYSCTL_PCGPIO_P12 = usize(0x00001000);  // GPIO Port N Power Control
pub const SYSCTL_PCGPIO_P11 = usize(0x00000800);  // GPIO Port M Power Control
pub const SYSCTL_PCGPIO_P10 = usize(0x00000400);  // GPIO Port L Power Control
pub const SYSCTL_PCGPIO_P9 = usize(0x00000200);  // GPIO Port K Power Control
pub const SYSCTL_PCGPIO_P8 = usize(0x00000100);  // GPIO Port J Power Control
pub const SYSCTL_PCGPIO_P7 = usize(0x00000080);  // GPIO Port H Power Control
pub const SYSCTL_PCGPIO_P6 = usize(0x00000040);  // GPIO Port G Power Control
pub const SYSCTL_PCGPIO_P5 = usize(0x00000020);  // GPIO Port F Power Control
pub const SYSCTL_PCGPIO_P4 = usize(0x00000010);  // GPIO Port E Power Control
pub const SYSCTL_PCGPIO_P3 = usize(0x00000008);  // GPIO Port D Power Control
pub const SYSCTL_PCGPIO_P2 = usize(0x00000004);  // GPIO Port C Power Control
pub const SYSCTL_PCGPIO_P1 = usize(0x00000002);  // GPIO Port B Power Control
pub const SYSCTL_PCGPIO_P0 = usize(0x00000001);  // GPIO Port A Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCDMA register.
//
//*****************************************************************************
pub const SYSCTL_PCDMA_P0 = usize(0x00000001);  // uDMA Module Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCEPI register.
//
//*****************************************************************************
pub const SYSCTL_PCEPI_P0 = usize(0x00000001);  // EPI Module Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCHIB register.
//
//*****************************************************************************
pub const SYSCTL_PCHIB_P0 = usize(0x00000001);  // Hibernation Module Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCUART register.
//
//*****************************************************************************
pub const SYSCTL_PCUART_P7 = usize(0x00000080);  // UART Module 7 Power Control
pub const SYSCTL_PCUART_P6 = usize(0x00000040);  // UART Module 6 Power Control
pub const SYSCTL_PCUART_P5 = usize(0x00000020);  // UART Module 5 Power Control
pub const SYSCTL_PCUART_P4 = usize(0x00000010);  // UART Module 4 Power Control
pub const SYSCTL_PCUART_P3 = usize(0x00000008);  // UART Module 3 Power Control
pub const SYSCTL_PCUART_P2 = usize(0x00000004);  // UART Module 2 Power Control
pub const SYSCTL_PCUART_P1 = usize(0x00000002);  // UART Module 1 Power Control
pub const SYSCTL_PCUART_P0 = usize(0x00000001);  // UART Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCSSI register.
//
//*****************************************************************************
pub const SYSCTL_PCSSI_P3 = usize(0x00000008);  // SSI Module 3 Power Control
pub const SYSCTL_PCSSI_P2 = usize(0x00000004);  // SSI Module 2 Power Control
pub const SYSCTL_PCSSI_P1 = usize(0x00000002);  // SSI Module 1 Power Control
pub const SYSCTL_PCSSI_P0 = usize(0x00000001);  // SSI Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCI2C register.
//
//*****************************************************************************
pub const SYSCTL_PCI2C_P9 = usize(0x00000200);  // I2C Module 9 Power Control
pub const SYSCTL_PCI2C_P8 = usize(0x00000100);  // I2C Module 8 Power Control
pub const SYSCTL_PCI2C_P7 = usize(0x00000080);  // I2C Module 7 Power Control
pub const SYSCTL_PCI2C_P6 = usize(0x00000040);  // I2C Module 6 Power Control
pub const SYSCTL_PCI2C_P5 = usize(0x00000020);  // I2C Module 5 Power Control
pub const SYSCTL_PCI2C_P4 = usize(0x00000010);  // I2C Module 4 Power Control
pub const SYSCTL_PCI2C_P3 = usize(0x00000008);  // I2C Module 3 Power Control
pub const SYSCTL_PCI2C_P2 = usize(0x00000004);  // I2C Module 2 Power Control
pub const SYSCTL_PCI2C_P1 = usize(0x00000002);  // I2C Module 1 Power Control
pub const SYSCTL_PCI2C_P0 = usize(0x00000001);  // I2C Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCUSB register.
//
//*****************************************************************************
pub const SYSCTL_PCUSB_P0 = usize(0x00000001);  // USB Module Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCEPHY register.
//
//*****************************************************************************
pub const SYSCTL_PCEPHY_P0 = usize(0x00000001);  // Ethernet PHY Module Power
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCCAN register.
//
//*****************************************************************************
pub const SYSCTL_PCCAN_P1 = usize(0x00000002);  // CAN Module 1 Power Control
pub const SYSCTL_PCCAN_P0 = usize(0x00000001);  // CAN Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCADC register.
//
//*****************************************************************************
pub const SYSCTL_PCADC_P1 = usize(0x00000002);  // ADC Module 1 Power Control
pub const SYSCTL_PCADC_P0 = usize(0x00000001);  // ADC Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCACMP register.
//
//*****************************************************************************
pub const SYSCTL_PCACMP_P0 = usize(0x00000001);  // Analog Comparator Module 0 Power
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCPWM register.
//
//*****************************************************************************
pub const SYSCTL_PCPWM_P0 = usize(0x00000001);  // PWM Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCQEI register.
//
//*****************************************************************************
pub const SYSCTL_PCQEI_P0 = usize(0x00000001);  // QEI Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCEEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_PCEEPROM_P0 = usize(0x00000001);  // EEPROM Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCCCM register.
//
//*****************************************************************************
pub const SYSCTL_PCCCM_P0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCLCD register.
//
//*****************************************************************************
pub const SYSCTL_PCLCD_P0 = usize(0x00000001);  // LCD Controller Module 0 Power
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCOWIRE register.
//
//*****************************************************************************
pub const SYSCTL_PCOWIRE_P0 = usize(0x00000001);  // 1-Wire Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCEMAC register.
//
//*****************************************************************************
pub const SYSCTL_PCEMAC_P0 = usize(0x00000001);  // Ethernet MAC Module 0 Power
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRWD register.
//
//*****************************************************************************
pub const SYSCTL_PRWD_R1 = usize(0x00000002);  // Watchdog Timer 1 Peripheral
                                            // Ready
pub const SYSCTL_PRWD_R0 = usize(0x00000001);  // Watchdog Timer 0 Peripheral
                                            // Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRTIMER register.
//
//*****************************************************************************
pub const SYSCTL_PRTIMER_R7 = usize(0x00000080);  // 16/32-Bit General-Purpose Timer
                                            // 7 Peripheral Ready
pub const SYSCTL_PRTIMER_R6 = usize(0x00000040);  // 16/32-Bit General-Purpose Timer
                                            // 6 Peripheral Ready
pub const SYSCTL_PRTIMER_R5 = usize(0x00000020);  // 16/32-Bit General-Purpose Timer
                                            // 5 Peripheral Ready
pub const SYSCTL_PRTIMER_R4 = usize(0x00000010);  // 16/32-Bit General-Purpose Timer
                                            // 4 Peripheral Ready
pub const SYSCTL_PRTIMER_R3 = usize(0x00000008);  // 16/32-Bit General-Purpose Timer
                                            // 3 Peripheral Ready
pub const SYSCTL_PRTIMER_R2 = usize(0x00000004);  // 16/32-Bit General-Purpose Timer
                                            // 2 Peripheral Ready
pub const SYSCTL_PRTIMER_R1 = usize(0x00000002);  // 16/32-Bit General-Purpose Timer
                                            // 1 Peripheral Ready
pub const SYSCTL_PRTIMER_R0 = usize(0x00000001);  // 16/32-Bit General-Purpose Timer
                                            // 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRGPIO register.
//
//*****************************************************************************
pub const SYSCTL_PRGPIO_R17 = usize(0x00020000);  // GPIO Port T Peripheral Ready
pub const SYSCTL_PRGPIO_R16 = usize(0x00010000);  // GPIO Port S Peripheral Ready
pub const SYSCTL_PRGPIO_R15 = usize(0x00008000);  // GPIO Port R Peripheral Ready
pub const SYSCTL_PRGPIO_R14 = usize(0x00004000);  // GPIO Port Q Peripheral Ready
pub const SYSCTL_PRGPIO_R13 = usize(0x00002000);  // GPIO Port P Peripheral Ready
pub const SYSCTL_PRGPIO_R12 = usize(0x00001000);  // GPIO Port N Peripheral Ready
pub const SYSCTL_PRGPIO_R11 = usize(0x00000800);  // GPIO Port M Peripheral Ready
pub const SYSCTL_PRGPIO_R10 = usize(0x00000400);  // GPIO Port L Peripheral Ready
pub const SYSCTL_PRGPIO_R9 = usize(0x00000200);  // GPIO Port K Peripheral Ready
pub const SYSCTL_PRGPIO_R8 = usize(0x00000100);  // GPIO Port J Peripheral Ready
pub const SYSCTL_PRGPIO_R7 = usize(0x00000080);  // GPIO Port H Peripheral Ready
pub const SYSCTL_PRGPIO_R6 = usize(0x00000040);  // GPIO Port G Peripheral Ready
pub const SYSCTL_PRGPIO_R5 = usize(0x00000020);  // GPIO Port F Peripheral Ready
pub const SYSCTL_PRGPIO_R4 = usize(0x00000010);  // GPIO Port E Peripheral Ready
pub const SYSCTL_PRGPIO_R3 = usize(0x00000008);  // GPIO Port D Peripheral Ready
pub const SYSCTL_PRGPIO_R2 = usize(0x00000004);  // GPIO Port C Peripheral Ready
pub const SYSCTL_PRGPIO_R1 = usize(0x00000002);  // GPIO Port B Peripheral Ready
pub const SYSCTL_PRGPIO_R0 = usize(0x00000001);  // GPIO Port A Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRDMA register.
//
//*****************************************************************************
pub const SYSCTL_PRDMA_R0 = usize(0x00000001);  // uDMA Module Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PREPI register.
//
//*****************************************************************************
pub const SYSCTL_PREPI_R0 = usize(0x00000001);  // EPI Module Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRHIB register.
//
//*****************************************************************************
pub const SYSCTL_PRHIB_R0 = usize(0x00000001);  // Hibernation Module Peripheral
                                            // Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRUART register.
//
//*****************************************************************************
pub const SYSCTL_PRUART_R7 = usize(0x00000080);  // UART Module 7 Peripheral Ready
pub const SYSCTL_PRUART_R6 = usize(0x00000040);  // UART Module 6 Peripheral Ready
pub const SYSCTL_PRUART_R5 = usize(0x00000020);  // UART Module 5 Peripheral Ready
pub const SYSCTL_PRUART_R4 = usize(0x00000010);  // UART Module 4 Peripheral Ready
pub const SYSCTL_PRUART_R3 = usize(0x00000008);  // UART Module 3 Peripheral Ready
pub const SYSCTL_PRUART_R2 = usize(0x00000004);  // UART Module 2 Peripheral Ready
pub const SYSCTL_PRUART_R1 = usize(0x00000002);  // UART Module 1 Peripheral Ready
pub const SYSCTL_PRUART_R0 = usize(0x00000001);  // UART Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRSSI register.
//
//*****************************************************************************
pub const SYSCTL_PRSSI_R3 = usize(0x00000008);  // SSI Module 3 Peripheral Ready
pub const SYSCTL_PRSSI_R2 = usize(0x00000004);  // SSI Module 2 Peripheral Ready
pub const SYSCTL_PRSSI_R1 = usize(0x00000002);  // SSI Module 1 Peripheral Ready
pub const SYSCTL_PRSSI_R0 = usize(0x00000001);  // SSI Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRI2C register.
//
//*****************************************************************************
pub const SYSCTL_PRI2C_R9 = usize(0x00000200);  // I2C Module 9 Peripheral Ready
pub const SYSCTL_PRI2C_R8 = usize(0x00000100);  // I2C Module 8 Peripheral Ready
pub const SYSCTL_PRI2C_R7 = usize(0x00000080);  // I2C Module 7 Peripheral Ready
pub const SYSCTL_PRI2C_R6 = usize(0x00000040);  // I2C Module 6 Peripheral Ready
pub const SYSCTL_PRI2C_R5 = usize(0x00000020);  // I2C Module 5 Peripheral Ready
pub const SYSCTL_PRI2C_R4 = usize(0x00000010);  // I2C Module 4 Peripheral Ready
pub const SYSCTL_PRI2C_R3 = usize(0x00000008);  // I2C Module 3 Peripheral Ready
pub const SYSCTL_PRI2C_R2 = usize(0x00000004);  // I2C Module 2 Peripheral Ready
pub const SYSCTL_PRI2C_R1 = usize(0x00000002);  // I2C Module 1 Peripheral Ready
pub const SYSCTL_PRI2C_R0 = usize(0x00000001);  // I2C Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRUSB register.
//
//*****************************************************************************
pub const SYSCTL_PRUSB_R0 = usize(0x00000001);  // USB Module Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PREPHY register.
//
//*****************************************************************************
pub const SYSCTL_PREPHY_R0 = usize(0x00000001);  // Ethernet PHY Module Peripheral
                                            // Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRCAN register.
//
//*****************************************************************************
pub const SYSCTL_PRCAN_R1 = usize(0x00000002);  // CAN Module 1 Peripheral Ready
pub const SYSCTL_PRCAN_R0 = usize(0x00000001);  // CAN Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRADC register.
//
//*****************************************************************************
pub const SYSCTL_PRADC_R1 = usize(0x00000002);  // ADC Module 1 Peripheral Ready
pub const SYSCTL_PRADC_R0 = usize(0x00000001);  // ADC Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRACMP register.
//
//*****************************************************************************
pub const SYSCTL_PRACMP_R0 = usize(0x00000001);  // Analog Comparator Module 0
                                            // Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRPWM register.
//
//*****************************************************************************
pub const SYSCTL_PRPWM_R1 = usize(0x00000002);  // PWM Module 1 Peripheral Ready
pub const SYSCTL_PRPWM_R0 = usize(0x00000001);  // PWM Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRQEI register.
//
//*****************************************************************************
pub const SYSCTL_PRQEI_R1 = usize(0x00000002);  // QEI Module 1 Peripheral Ready
pub const SYSCTL_PRQEI_R0 = usize(0x00000001);  // QEI Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PREEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_PREEPROM_R0 = usize(0x00000001);  // EEPROM Module Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_PRWTIMER_R5 = usize(0x00000020);  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Peripheral Ready
pub const SYSCTL_PRWTIMER_R4 = usize(0x00000010);  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Peripheral Ready
pub const SYSCTL_PRWTIMER_R3 = usize(0x00000008);  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Peripheral Ready
pub const SYSCTL_PRWTIMER_R2 = usize(0x00000004);  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Peripheral Ready
pub const SYSCTL_PRWTIMER_R1 = usize(0x00000002);  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Peripheral Ready
pub const SYSCTL_PRWTIMER_R0 = usize(0x00000001);  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRCCM register.
//
//*****************************************************************************
pub const SYSCTL_PRCCM_R0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRLCD register.
//
//*****************************************************************************
pub const SYSCTL_PRLCD_R0 = usize(0x00000001);  // LCD Controller Module 0
                                            // Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PROWIRE register.
//
//*****************************************************************************
pub const SYSCTL_PROWIRE_R0 = usize(0x00000001);  // 1-Wire Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PREMAC register.
//
//*****************************************************************************
pub const SYSCTL_PREMAC_R0 = usize(0x00000001);  // Ethernet MAC Module 0 Peripheral
                                            // Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_UNIQUEID0
// register.
//
//*****************************************************************************
pub const SYSCTL_UNIQUEID0_ID_M = usize(0xFFFFFFFF);  // Unique ID
pub const SYSCTL_UNIQUEID0_ID_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_UNIQUEID1
// register.
//
//*****************************************************************************
pub const SYSCTL_UNIQUEID1_ID_M = usize(0xFFFFFFFF);  // Unique ID
pub const SYSCTL_UNIQUEID1_ID_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_UNIQUEID2
// register.
//
//*****************************************************************************
pub const SYSCTL_UNIQUEID2_ID_M = usize(0xFFFFFFFF);  // Unique ID
pub const SYSCTL_UNIQUEID2_ID_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_UNIQUEID3
// register.
//
//*****************************************************************************
pub const SYSCTL_UNIQUEID3_ID_M = usize(0xFFFFFFFF);  // Unique ID
pub const SYSCTL_UNIQUEID3_ID_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_CCMCGREQ
// register.
//
//*****************************************************************************
pub const SYSCTL_CCMCGREQ_DESCFG = usize(0x00000004);  // DES Clock Gating Request
pub const SYSCTL_CCMCGREQ_AESCFG = usize(0x00000002);  // AES Clock Gating Request
pub const SYSCTL_CCMCGREQ_SHACFG = usize(0x00000001);  // SHA/MD5 Clock Gating Request

//*****************************************************************************
//
// The following definitions are deprecated.
//
//*****************************************************************************

