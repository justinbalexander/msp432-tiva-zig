//*****************************************************************************
//
// hw_memmap.h - Macros defining the memory map of the device.
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
// The following are defines for the base address of the memories and
// peripherals.
//
//*****************************************************************************
pub const FLASH_BASE = usize(0x00000000);  // FLASH memory
pub const SRAM_BASE = usize(0x20000000);  // SRAM memory
pub const WATCHDOG0_BASE = usize(0x40000000);  // Watchdog0
pub const WATCHDOG1_BASE = usize(0x40001000);  // Watchdog1
pub const GPIO_PORTA_BASE = usize(0x40004000);  // GPIO Port A
pub const GPIO_PORTB_BASE = usize(0x40005000);  // GPIO Port B
pub const GPIO_PORTC_BASE = usize(0x40006000);  // GPIO Port C
pub const GPIO_PORTD_BASE = usize(0x40007000);  // GPIO Port D
pub const SSI0_BASE = usize(0x40008000);  // SSI0
pub const SSI1_BASE = usize(0x40009000);  // SSI1
pub const SSI2_BASE = usize(0x4000A000);  // SSI2
pub const SSI3_BASE = usize(0x4000B000);  // SSI3
pub const UART0_BASE = usize(0x4000C000);  // UART0
pub const UART1_BASE = usize(0x4000D000);  // UART1
pub const UART2_BASE = usize(0x4000E000);  // UART2
pub const UART3_BASE = usize(0x4000F000);  // UART3
pub const UART4_BASE = usize(0x40010000);  // UART4
pub const UART5_BASE = usize(0x40011000);  // UART5
pub const UART6_BASE = usize(0x40012000);  // UART6
pub const UART7_BASE = usize(0x40013000);  // UART7
pub const I2C0_BASE = usize(0x40020000);  // I2C0
pub const I2C1_BASE = usize(0x40021000);  // I2C1
pub const I2C2_BASE = usize(0x40022000);  // I2C2
pub const I2C3_BASE = usize(0x40023000);  // I2C3
pub const GPIO_PORTE_BASE = usize(0x40024000);  // GPIO Port E
pub const GPIO_PORTF_BASE = usize(0x40025000);  // GPIO Port F
pub const GPIO_PORTG_BASE = usize(0x40026000);  // GPIO Port G
pub const GPIO_PORTH_BASE = usize(0x40027000);  // GPIO Port H
pub const PWM0_BASE = usize(0x40028000);  // Pulse Width Modulator (PWM)
pub const PWM1_BASE = usize(0x40029000);  // Pulse Width Modulator (PWM)
pub const QEI0_BASE = usize(0x4002C000);  // QEI0
pub const QEI1_BASE = usize(0x4002D000);  // QEI1
pub const TIMER0_BASE = usize(0x40030000);  // Timer0
pub const TIMER1_BASE = usize(0x40031000);  // Timer1
pub const TIMER2_BASE = usize(0x40032000);  // Timer2
pub const TIMER3_BASE = usize(0x40033000);  // Timer3
pub const TIMER4_BASE = usize(0x40034000);  // Timer4
pub const TIMER5_BASE = usize(0x40035000);  // Timer5
pub const WTIMER0_BASE = usize(0x40036000);  // Wide Timer0
pub const WTIMER1_BASE = usize(0x40037000);  // Wide Timer1
pub const ADC0_BASE = usize(0x40038000);  // ADC0
pub const ADC1_BASE = usize(0x40039000);  // ADC1
pub const COMP_BASE = usize(0x4003C000);  // Analog comparators
pub const GPIO_PORTJ_BASE = usize(0x4003D000);  // GPIO Port J
pub const CAN0_BASE = usize(0x40040000);  // CAN0
pub const CAN1_BASE = usize(0x40041000);  // CAN1
pub const WTIMER2_BASE = usize(0x4004C000);  // Wide Timer2
pub const WTIMER3_BASE = usize(0x4004D000);  // Wide Timer3
pub const WTIMER4_BASE = usize(0x4004E000);  // Wide Timer4
pub const WTIMER5_BASE = usize(0x4004F000);  // Wide Timer5
pub const USB0_BASE = usize(0x40050000);  // USB 0 Controller
pub const GPIO_PORTA_AHB_BASE = usize(0x40058000);  // GPIO Port A (high speed)
pub const GPIO_PORTB_AHB_BASE = usize(0x40059000);  // GPIO Port B (high speed)
pub const GPIO_PORTC_AHB_BASE = usize(0x4005A000);  // GPIO Port C (high speed)
pub const GPIO_PORTD_AHB_BASE = usize(0x4005B000);  // GPIO Port D (high speed)
pub const GPIO_PORTE_AHB_BASE = usize(0x4005C000);  // GPIO Port E (high speed)
pub const GPIO_PORTF_AHB_BASE = usize(0x4005D000);  // GPIO Port F (high speed)
pub const GPIO_PORTG_AHB_BASE = usize(0x4005E000);  // GPIO Port G (high speed)
pub const GPIO_PORTH_AHB_BASE = usize(0x4005F000);  // GPIO Port H (high speed)
pub const GPIO_PORTJ_AHB_BASE = usize(0x40060000);  // GPIO Port J (high speed)
pub const GPIO_PORTK_BASE = usize(0x40061000);  // GPIO Port K
pub const GPIO_PORTL_BASE = usize(0x40062000);  // GPIO Port L
pub const GPIO_PORTM_BASE = usize(0x40063000);  // GPIO Port M
pub const GPIO_PORTN_BASE = usize(0x40064000);  // GPIO Port N
pub const GPIO_PORTP_BASE = usize(0x40065000);  // GPIO Port P
pub const GPIO_PORTQ_BASE = usize(0x40066000);  // GPIO Port Q
pub const GPIO_PORTR_BASE = usize(0x40067000);  // General-Purpose Input/Outputs
                                            // (GPIOs)
pub const GPIO_PORTS_BASE = usize(0x40068000);  // General-Purpose Input/Outputs
                                            // (GPIOs)
pub const GPIO_PORTT_BASE = usize(0x40069000);  // General-Purpose Input/Outputs
                                            // (GPIOs)
pub const EEPROM_BASE = usize(0x400AF000);  // EEPROM memory
pub const ONEWIRE0_BASE = usize(0x400B6000);  // 1-Wire Master Module
pub const I2C8_BASE = usize(0x400B8000);  // I2C8
pub const I2C9_BASE = usize(0x400B9000);  // I2C9
pub const I2C4_BASE = usize(0x400C0000);  // I2C4
pub const I2C5_BASE = usize(0x400C1000);  // I2C5
pub const I2C6_BASE = usize(0x400C2000);  // I2C6
pub const I2C7_BASE = usize(0x400C3000);  // I2C7
pub const EPI0_BASE = usize(0x400D0000);  // EPI0
pub const TIMER6_BASE = usize(0x400E0000);  // General-Purpose Timers
pub const TIMER7_BASE = usize(0x400E1000);  // General-Purpose Timers
pub const EMAC0_BASE = usize(0x400EC000);  // Ethernet Controller
pub const SYSEXC_BASE = usize(0x400F9000);  // System Exception Module
pub const HIB_BASE = usize(0x400FC000);  // Hibernation Module
pub const FLASH_CTRL_BASE = usize(0x400FD000);  // FLASH Controller
pub const SYSCTL_BASE = usize(0x400FE000);  // System Control
pub const UDMA_BASE = usize(0x400FF000);  // uDMA Controller
pub const CCM0_BASE = usize(0x44030000);  // Cyclical Redundancy Check (CRC)
pub const SHAMD5_BASE = usize(0x44034000);  // SHA/MD5 Accelerator
pub const AES_BASE = usize(0x44036000);  // Advance Encryption
                                            // Hardware-Accelerated Module
pub const DES_BASE = usize(0x44038000);  // Data Encryption Standard
                                            // Accelerator (DES)
pub const LCD0_BASE = usize(0x44050000);  // LCD Controller
pub const ITM_BASE = usize(0xE0000000);  // Instrumentation Trace Macrocell
pub const DWT_BASE = usize(0xE0001000);  // Data Watchpoint and Trace
pub const FPB_BASE = usize(0xE0002000);  // FLASH Patch and Breakpoint
pub const NVIC_BASE = usize(0xE000E000);  // Nested Vectored Interrupt Ctrl
pub const TPIU_BASE = usize(0xE0040000);  // Trace Port Interface Unit

