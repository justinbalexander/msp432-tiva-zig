//*****************************************************************************
//
// tm4c129dnczad.h - TM4C129DNCZAD Register Definitions
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
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
// Interrupt assignments
//
//*****************************************************************************
pub const INT_GPIOA = usize(16);          // GPIO Port A
pub const INT_GPIOB = usize(17);          // GPIO Port B
pub const INT_GPIOC = usize(18);          // GPIO Port C
pub const INT_GPIOD = usize(19);          // GPIO Port D
pub const INT_GPIOE = usize(20);          // GPIO Port E
pub const INT_UART0 = usize(21);          // UART0
pub const INT_UART1 = usize(22);          // UART1
pub const INT_SSI0 = usize(23);          // SSI0
pub const INT_I2C0 = usize(24);          // I2C0
pub const INT_PWM0_FAULT = usize(25);          // PWM Fault
pub const INT_PWM0_0 = usize(26);          // PWM Generator 0
pub const INT_PWM0_1 = usize(27);          // PWM Generator 1
pub const INT_PWM0_2 = usize(28);          // PWM Generator 2
pub const INT_QEI0 = usize(29);          // QEI0
pub const INT_ADC0SS0 = usize(30);          // ADC0 Sequence 0
pub const INT_ADC0SS1 = usize(31);          // ADC0 Sequence 1
pub const INT_ADC0SS2 = usize(32);          // ADC0 Sequence 2
pub const INT_ADC0SS3 = usize(33);          // ADC0 Sequence 3
pub const INT_WATCHDOG = usize(34);          // Watchdog Timers 0 and 1
pub const INT_TIMER0A = usize(35);          // 16/32-Bit Timer 0A
pub const INT_TIMER0B = usize(36);          // 16/32-Bit Timer 0B
pub const INT_TIMER1A = usize(37);          // 16/32-Bit Timer 1A
pub const INT_TIMER1B = usize(38);          // 16/32-Bit Timer 1B
pub const INT_TIMER2A = usize(39);          // 16/32-Bit Timer 2A
pub const INT_TIMER2B = usize(40);          // 16/32-Bit Timer 2B
pub const INT_COMP0 = usize(41);          // Analog Comparator 0
pub const INT_COMP1 = usize(42);          // Analog Comparator 1
pub const INT_COMP2 = usize(43);          // Analog Comparator 2
pub const INT_SYSCTL = usize(44);          // System Control
pub const INT_FLASH = usize(45);          // Flash Memory Control
pub const INT_GPIOF = usize(46);          // GPIO Port F
pub const INT_GPIOG = usize(47);          // GPIO Port G
pub const INT_GPIOH = usize(48);          // GPIO Port H
pub const INT_UART2 = usize(49);          // UART2
pub const INT_SSI1 = usize(50);          // SSI1
pub const INT_TIMER3A = usize(51);          // 16/32-Bit Timer 3A
pub const INT_TIMER3B = usize(52);          // 16/32-Bit Timer 3B
pub const INT_I2C1 = usize(53);          // I2C1
pub const INT_CAN0 = usize(54);          // CAN 0
pub const INT_CAN1 = usize(55);          // CAN1
pub const INT_EMAC0 = usize(56);          // Ethernet MAC
pub const INT_HIBERNATE = usize(57);          // HIB
pub const INT_USB0 = usize(58);          // USB MAC
pub const INT_PWM0_3 = usize(59);          // PWM Generator 3
pub const INT_UDMA = usize(60);          // uDMA 0 Software
pub const INT_UDMAERR = usize(61);          // uDMA 0 Error
pub const INT_ADC1SS0 = usize(62);          // ADC1 Sequence 0
pub const INT_ADC1SS1 = usize(63);          // ADC1 Sequence 1
pub const INT_ADC1SS2 = usize(64);          // ADC1 Sequence 2
pub const INT_ADC1SS3 = usize(65);          // ADC1 Sequence 3
pub const INT_EPI0 = usize(66);          // EPI 0
pub const INT_GPIOJ = usize(67);          // GPIO Port J
pub const INT_GPIOK = usize(68);          // GPIO Port K
pub const INT_GPIOL = usize(69);          // GPIO Port L
pub const INT_SSI2 = usize(70);          // SSI 2
pub const INT_SSI3 = usize(71);          // SSI 3
pub const INT_UART3 = usize(72);          // UART 3
pub const INT_UART4 = usize(73);          // UART 4
pub const INT_UART5 = usize(74);          // UART 5
pub const INT_UART6 = usize(75);          // UART 6
pub const INT_UART7 = usize(76);          // UART 7
pub const INT_I2C2 = usize(77);          // I2C 2
pub const INT_I2C3 = usize(78);          // I2C 3
pub const INT_TIMER4A = usize(79);          // Timer 4A
pub const INT_TIMER4B = usize(80);          // Timer 4B
pub const INT_TIMER5A = usize(81);          // Timer 5A
pub const INT_TIMER5B = usize(82);          // Timer 5B
pub const INT_SYSEXC = usize(83);          // Floating-Point Exception
                                            // (imprecise)
pub const INT_I2C4 = usize(86);          // I2C 4
pub const INT_I2C5 = usize(87);          // I2C 5
pub const INT_GPIOM = usize(88);          // GPIO Port M
pub const INT_GPION = usize(89);          // GPIO Port N
pub const INT_TAMPER0 = usize(91);          // Tamper
pub const INT_GPIOP0 = usize(92);          // GPIO Port P (Summary or P0)
pub const INT_GPIOP1 = usize(93);          // GPIO Port P1
pub const INT_GPIOP2 = usize(94);          // GPIO Port P2
pub const INT_GPIOP3 = usize(95);          // GPIO Port P3
pub const INT_GPIOP4 = usize(96);          // GPIO Port P4
pub const INT_GPIOP5 = usize(97);          // GPIO Port P5
pub const INT_GPIOP6 = usize(98);          // GPIO Port P6
pub const INT_GPIOP7 = usize(99);          // GPIO Port P7
pub const INT_GPIOQ0 = usize(100);         // GPIO Port Q (Summary or Q0)
pub const INT_GPIOQ1 = usize(101);         // GPIO Port Q1
pub const INT_GPIOQ2 = usize(102);         // GPIO Port Q2
pub const INT_GPIOQ3 = usize(103);         // GPIO Port Q3
pub const INT_GPIOQ4 = usize(104);         // GPIO Port Q4
pub const INT_GPIOQ5 = usize(105);         // GPIO Port Q5
pub const INT_GPIOQ6 = usize(106);         // GPIO Port Q6
pub const INT_GPIOQ7 = usize(107);         // GPIO Port Q7
pub const INT_GPIOR = usize(108);         // GPIO Port R
pub const INT_GPIOS = usize(109);         // GPIO Port S
pub const INT_SHA0 = usize(110);         // SHA/MD5
pub const INT_AES0 = usize(111);         // AES
pub const INT_DES0 = usize(112);         // DES
pub const INT_TIMER6A = usize(114);         // 16/32-Bit Timer 6A
pub const INT_TIMER6B = usize(115);         // 16/32-Bit Timer 6B
pub const INT_TIMER7A = usize(116);         // 16/32-Bit Timer 7A
pub const INT_TIMER7B = usize(117);         // 16/32-Bit Timer 7B
pub const INT_I2C6 = usize(118);         // I2C 6
pub const INT_I2C7 = usize(119);         // I2C 7
pub const INT_I2C8 = usize(125);         // I2C 8
pub const INT_I2C9 = usize(126);         // I2C 9
pub const INT_GPIOT = usize(127);         // GPIO T

//*****************************************************************************
//
// Watchdog Timer registers (WATCHDOG0)
//
//*****************************************************************************
pub const WATCHDOG0_LOAD_R = @intToPtr(*volatile u32, 0x40000000);
pub const WATCHDOG0_VALUE_R = @intToPtr(*volatile u32, 0x40000004);
pub const WATCHDOG0_CTL_R = @intToPtr(*volatile u32, 0x40000008);
pub const WATCHDOG0_ICR_R = @intToPtr(*volatile u32, 0x4000000C);
pub const WATCHDOG0_RIS_R = @intToPtr(*volatile u32, 0x40000010);
pub const WATCHDOG0_MIS_R = @intToPtr(*volatile u32, 0x40000014);
pub const WATCHDOG0_TEST_R = @intToPtr(*volatile u32, 0x40000418);
pub const WATCHDOG0_LOCK_R = @intToPtr(*volatile u32, 0x40000C00);

//*****************************************************************************
//
// Watchdog Timer registers (WATCHDOG1)
//
//*****************************************************************************
pub const WATCHDOG1_LOAD_R = @intToPtr(*volatile u32, 0x40001000);
pub const WATCHDOG1_VALUE_R = @intToPtr(*volatile u32, 0x40001004);
pub const WATCHDOG1_CTL_R = @intToPtr(*volatile u32, 0x40001008);
pub const WATCHDOG1_ICR_R = @intToPtr(*volatile u32, 0x4000100C);
pub const WATCHDOG1_RIS_R = @intToPtr(*volatile u32, 0x40001010);
pub const WATCHDOG1_MIS_R = @intToPtr(*volatile u32, 0x40001014);
pub const WATCHDOG1_TEST_R = @intToPtr(*volatile u32, 0x40001418);
pub const WATCHDOG1_LOCK_R = @intToPtr(*volatile u32, 0x40001C00);

//*****************************************************************************
//
// SSI registers (SSI0)
//
//*****************************************************************************
pub const SSI0_CR0_R = @intToPtr(*volatile u32, 0x40008000);
pub const SSI0_CR1_R = @intToPtr(*volatile u32, 0x40008004);
pub const SSI0_DR_R = @intToPtr(*volatile u32, 0x40008008);
pub const SSI0_SR_R = @intToPtr(*volatile u32, 0x4000800C);
pub const SSI0_CPSR_R = @intToPtr(*volatile u32, 0x40008010);
pub const SSI0_IM_R = @intToPtr(*volatile u32, 0x40008014);
pub const SSI0_RIS_R = @intToPtr(*volatile u32, 0x40008018);
pub const SSI0_MIS_R = @intToPtr(*volatile u32, 0x4000801C);
pub const SSI0_ICR_R = @intToPtr(*volatile u32, 0x40008020);
pub const SSI0_DMACTL_R = @intToPtr(*volatile u32, 0x40008024);
pub const SSI0_PP_R = @intToPtr(*volatile u32, 0x40008FC0);
pub const SSI0_CC_R = @intToPtr(*volatile u32, 0x40008FC8);

//*****************************************************************************
//
// SSI registers (SSI1)
//
//*****************************************************************************
pub const SSI1_CR0_R = @intToPtr(*volatile u32, 0x40009000);
pub const SSI1_CR1_R = @intToPtr(*volatile u32, 0x40009004);
pub const SSI1_DR_R = @intToPtr(*volatile u32, 0x40009008);
pub const SSI1_SR_R = @intToPtr(*volatile u32, 0x4000900C);
pub const SSI1_CPSR_R = @intToPtr(*volatile u32, 0x40009010);
pub const SSI1_IM_R = @intToPtr(*volatile u32, 0x40009014);
pub const SSI1_RIS_R = @intToPtr(*volatile u32, 0x40009018);
pub const SSI1_MIS_R = @intToPtr(*volatile u32, 0x4000901C);
pub const SSI1_ICR_R = @intToPtr(*volatile u32, 0x40009020);
pub const SSI1_DMACTL_R = @intToPtr(*volatile u32, 0x40009024);
pub const SSI1_PP_R = @intToPtr(*volatile u32, 0x40009FC0);
pub const SSI1_CC_R = @intToPtr(*volatile u32, 0x40009FC8);

//*****************************************************************************
//
// SSI registers (SSI2)
//
//*****************************************************************************
pub const SSI2_CR0_R = @intToPtr(*volatile u32, 0x4000A000);
pub const SSI2_CR1_R = @intToPtr(*volatile u32, 0x4000A004);
pub const SSI2_DR_R = @intToPtr(*volatile u32, 0x4000A008);
pub const SSI2_SR_R = @intToPtr(*volatile u32, 0x4000A00C);
pub const SSI2_CPSR_R = @intToPtr(*volatile u32, 0x4000A010);
pub const SSI2_IM_R = @intToPtr(*volatile u32, 0x4000A014);
pub const SSI2_RIS_R = @intToPtr(*volatile u32, 0x4000A018);
pub const SSI2_MIS_R = @intToPtr(*volatile u32, 0x4000A01C);
pub const SSI2_ICR_R = @intToPtr(*volatile u32, 0x4000A020);
pub const SSI2_DMACTL_R = @intToPtr(*volatile u32, 0x4000A024);
pub const SSI2_PP_R = @intToPtr(*volatile u32, 0x4000AFC0);
pub const SSI2_CC_R = @intToPtr(*volatile u32, 0x4000AFC8);

//*****************************************************************************
//
// SSI registers (SSI3)
//
//*****************************************************************************
pub const SSI3_CR0_R = @intToPtr(*volatile u32, 0x4000B000);
pub const SSI3_CR1_R = @intToPtr(*volatile u32, 0x4000B004);
pub const SSI3_DR_R = @intToPtr(*volatile u32, 0x4000B008);
pub const SSI3_SR_R = @intToPtr(*volatile u32, 0x4000B00C);
pub const SSI3_CPSR_R = @intToPtr(*volatile u32, 0x4000B010);
pub const SSI3_IM_R = @intToPtr(*volatile u32, 0x4000B014);
pub const SSI3_RIS_R = @intToPtr(*volatile u32, 0x4000B018);
pub const SSI3_MIS_R = @intToPtr(*volatile u32, 0x4000B01C);
pub const SSI3_ICR_R = @intToPtr(*volatile u32, 0x4000B020);
pub const SSI3_DMACTL_R = @intToPtr(*volatile u32, 0x4000B024);
pub const SSI3_PP_R = @intToPtr(*volatile u32, 0x4000BFC0);
pub const SSI3_CC_R = @intToPtr(*volatile u32, 0x4000BFC8);

//*****************************************************************************
//
// UART registers (UART0)
//
//*****************************************************************************
pub const UART0_DR_R = @intToPtr(*volatile u32, 0x4000C000);
pub const UART0_RSR_R = @intToPtr(*volatile u32, 0x4000C004);
pub const UART0_ECR_R = @intToPtr(*volatile u32, 0x4000C004);
pub const UART0_FR_R = @intToPtr(*volatile u32, 0x4000C018);
pub const UART0_ILPR_R = @intToPtr(*volatile u32, 0x4000C020);
pub const UART0_IBRD_R = @intToPtr(*volatile u32, 0x4000C024);
pub const UART0_FBRD_R = @intToPtr(*volatile u32, 0x4000C028);
pub const UART0_LCRH_R = @intToPtr(*volatile u32, 0x4000C02C);
pub const UART0_CTL_R = @intToPtr(*volatile u32, 0x4000C030);
pub const UART0_IFLS_R = @intToPtr(*volatile u32, 0x4000C034);
pub const UART0_IM_R = @intToPtr(*volatile u32, 0x4000C038);
pub const UART0_RIS_R = @intToPtr(*volatile u32, 0x4000C03C);
pub const UART0_MIS_R = @intToPtr(*volatile u32, 0x4000C040);
pub const UART0_ICR_R = @intToPtr(*volatile u32, 0x4000C044);
pub const UART0_DMACTL_R = @intToPtr(*volatile u32, 0x4000C048);
pub const UART0_9BITADDR_R = @intToPtr(*volatile u32, 0x4000C0A4);
pub const UART0_9BITAMASK_R = @intToPtr(*volatile u32, 0x4000C0A8);
pub const UART0_PP_R = @intToPtr(*volatile u32, 0x4000CFC0);
pub const UART0_CC_R = @intToPtr(*volatile u32, 0x4000CFC8);

//*****************************************************************************
//
// UART registers (UART1)
//
//*****************************************************************************
pub const UART1_DR_R = @intToPtr(*volatile u32, 0x4000D000);
pub const UART1_RSR_R = @intToPtr(*volatile u32, 0x4000D004);
pub const UART1_ECR_R = @intToPtr(*volatile u32, 0x4000D004);
pub const UART1_FR_R = @intToPtr(*volatile u32, 0x4000D018);
pub const UART1_ILPR_R = @intToPtr(*volatile u32, 0x4000D020);
pub const UART1_IBRD_R = @intToPtr(*volatile u32, 0x4000D024);
pub const UART1_FBRD_R = @intToPtr(*volatile u32, 0x4000D028);
pub const UART1_LCRH_R = @intToPtr(*volatile u32, 0x4000D02C);
pub const UART1_CTL_R = @intToPtr(*volatile u32, 0x4000D030);
pub const UART1_IFLS_R = @intToPtr(*volatile u32, 0x4000D034);
pub const UART1_IM_R = @intToPtr(*volatile u32, 0x4000D038);
pub const UART1_RIS_R = @intToPtr(*volatile u32, 0x4000D03C);
pub const UART1_MIS_R = @intToPtr(*volatile u32, 0x4000D040);
pub const UART1_ICR_R = @intToPtr(*volatile u32, 0x4000D044);
pub const UART1_DMACTL_R = @intToPtr(*volatile u32, 0x4000D048);
pub const UART1_9BITADDR_R = @intToPtr(*volatile u32, 0x4000D0A4);
pub const UART1_9BITAMASK_R = @intToPtr(*volatile u32, 0x4000D0A8);
pub const UART1_PP_R = @intToPtr(*volatile u32, 0x4000DFC0);
pub const UART1_CC_R = @intToPtr(*volatile u32, 0x4000DFC8);

//*****************************************************************************
//
// UART registers (UART2)
//
//*****************************************************************************
pub const UART2_DR_R = @intToPtr(*volatile u32, 0x4000E000);
pub const UART2_RSR_R = @intToPtr(*volatile u32, 0x4000E004);
pub const UART2_ECR_R = @intToPtr(*volatile u32, 0x4000E004);
pub const UART2_FR_R = @intToPtr(*volatile u32, 0x4000E018);
pub const UART2_ILPR_R = @intToPtr(*volatile u32, 0x4000E020);
pub const UART2_IBRD_R = @intToPtr(*volatile u32, 0x4000E024);
pub const UART2_FBRD_R = @intToPtr(*volatile u32, 0x4000E028);
pub const UART2_LCRH_R = @intToPtr(*volatile u32, 0x4000E02C);
pub const UART2_CTL_R = @intToPtr(*volatile u32, 0x4000E030);
pub const UART2_IFLS_R = @intToPtr(*volatile u32, 0x4000E034);
pub const UART2_IM_R = @intToPtr(*volatile u32, 0x4000E038);
pub const UART2_RIS_R = @intToPtr(*volatile u32, 0x4000E03C);
pub const UART2_MIS_R = @intToPtr(*volatile u32, 0x4000E040);
pub const UART2_ICR_R = @intToPtr(*volatile u32, 0x4000E044);
pub const UART2_DMACTL_R = @intToPtr(*volatile u32, 0x4000E048);
pub const UART2_9BITADDR_R = @intToPtr(*volatile u32, 0x4000E0A4);
pub const UART2_9BITAMASK_R = @intToPtr(*volatile u32, 0x4000E0A8);
pub const UART2_PP_R = @intToPtr(*volatile u32, 0x4000EFC0);
pub const UART2_CC_R = @intToPtr(*volatile u32, 0x4000EFC8);

//*****************************************************************************
//
// UART registers (UART3)
//
//*****************************************************************************
pub const UART3_DR_R = @intToPtr(*volatile u32, 0x4000F000);
pub const UART3_RSR_R = @intToPtr(*volatile u32, 0x4000F004);
pub const UART3_ECR_R = @intToPtr(*volatile u32, 0x4000F004);
pub const UART3_FR_R = @intToPtr(*volatile u32, 0x4000F018);
pub const UART3_ILPR_R = @intToPtr(*volatile u32, 0x4000F020);
pub const UART3_IBRD_R = @intToPtr(*volatile u32, 0x4000F024);
pub const UART3_FBRD_R = @intToPtr(*volatile u32, 0x4000F028);
pub const UART3_LCRH_R = @intToPtr(*volatile u32, 0x4000F02C);
pub const UART3_CTL_R = @intToPtr(*volatile u32, 0x4000F030);
pub const UART3_IFLS_R = @intToPtr(*volatile u32, 0x4000F034);
pub const UART3_IM_R = @intToPtr(*volatile u32, 0x4000F038);
pub const UART3_RIS_R = @intToPtr(*volatile u32, 0x4000F03C);
pub const UART3_MIS_R = @intToPtr(*volatile u32, 0x4000F040);
pub const UART3_ICR_R = @intToPtr(*volatile u32, 0x4000F044);
pub const UART3_DMACTL_R = @intToPtr(*volatile u32, 0x4000F048);
pub const UART3_9BITADDR_R = @intToPtr(*volatile u32, 0x4000F0A4);
pub const UART3_9BITAMASK_R = @intToPtr(*volatile u32, 0x4000F0A8);
pub const UART3_PP_R = @intToPtr(*volatile u32, 0x4000FFC0);
pub const UART3_CC_R = @intToPtr(*volatile u32, 0x4000FFC8);

//*****************************************************************************
//
// UART registers (UART4)
//
//*****************************************************************************
pub const UART4_DR_R = @intToPtr(*volatile u32, 0x40010000);
pub const UART4_RSR_R = @intToPtr(*volatile u32, 0x40010004);
pub const UART4_ECR_R = @intToPtr(*volatile u32, 0x40010004);
pub const UART4_FR_R = @intToPtr(*volatile u32, 0x40010018);
pub const UART4_ILPR_R = @intToPtr(*volatile u32, 0x40010020);
pub const UART4_IBRD_R = @intToPtr(*volatile u32, 0x40010024);
pub const UART4_FBRD_R = @intToPtr(*volatile u32, 0x40010028);
pub const UART4_LCRH_R = @intToPtr(*volatile u32, 0x4001002C);
pub const UART4_CTL_R = @intToPtr(*volatile u32, 0x40010030);
pub const UART4_IFLS_R = @intToPtr(*volatile u32, 0x40010034);
pub const UART4_IM_R = @intToPtr(*volatile u32, 0x40010038);
pub const UART4_RIS_R = @intToPtr(*volatile u32, 0x4001003C);
pub const UART4_MIS_R = @intToPtr(*volatile u32, 0x40010040);
pub const UART4_ICR_R = @intToPtr(*volatile u32, 0x40010044);
pub const UART4_DMACTL_R = @intToPtr(*volatile u32, 0x40010048);
pub const UART4_9BITADDR_R = @intToPtr(*volatile u32, 0x400100A4);
pub const UART4_9BITAMASK_R = @intToPtr(*volatile u32, 0x400100A8);
pub const UART4_PP_R = @intToPtr(*volatile u32, 0x40010FC0);
pub const UART4_CC_R = @intToPtr(*volatile u32, 0x40010FC8);

//*****************************************************************************
//
// UART registers (UART5)
//
//*****************************************************************************
pub const UART5_DR_R = @intToPtr(*volatile u32, 0x40011000);
pub const UART5_RSR_R = @intToPtr(*volatile u32, 0x40011004);
pub const UART5_ECR_R = @intToPtr(*volatile u32, 0x40011004);
pub const UART5_FR_R = @intToPtr(*volatile u32, 0x40011018);
pub const UART5_ILPR_R = @intToPtr(*volatile u32, 0x40011020);
pub const UART5_IBRD_R = @intToPtr(*volatile u32, 0x40011024);
pub const UART5_FBRD_R = @intToPtr(*volatile u32, 0x40011028);
pub const UART5_LCRH_R = @intToPtr(*volatile u32, 0x4001102C);
pub const UART5_CTL_R = @intToPtr(*volatile u32, 0x40011030);
pub const UART5_IFLS_R = @intToPtr(*volatile u32, 0x40011034);
pub const UART5_IM_R = @intToPtr(*volatile u32, 0x40011038);
pub const UART5_RIS_R = @intToPtr(*volatile u32, 0x4001103C);
pub const UART5_MIS_R = @intToPtr(*volatile u32, 0x40011040);
pub const UART5_ICR_R = @intToPtr(*volatile u32, 0x40011044);
pub const UART5_DMACTL_R = @intToPtr(*volatile u32, 0x40011048);
pub const UART5_9BITADDR_R = @intToPtr(*volatile u32, 0x400110A4);
pub const UART5_9BITAMASK_R = @intToPtr(*volatile u32, 0x400110A8);
pub const UART5_PP_R = @intToPtr(*volatile u32, 0x40011FC0);
pub const UART5_CC_R = @intToPtr(*volatile u32, 0x40011FC8);

//*****************************************************************************
//
// UART registers (UART6)
//
//*****************************************************************************
pub const UART6_DR_R = @intToPtr(*volatile u32, 0x40012000);
pub const UART6_RSR_R = @intToPtr(*volatile u32, 0x40012004);
pub const UART6_ECR_R = @intToPtr(*volatile u32, 0x40012004);
pub const UART6_FR_R = @intToPtr(*volatile u32, 0x40012018);
pub const UART6_ILPR_R = @intToPtr(*volatile u32, 0x40012020);
pub const UART6_IBRD_R = @intToPtr(*volatile u32, 0x40012024);
pub const UART6_FBRD_R = @intToPtr(*volatile u32, 0x40012028);
pub const UART6_LCRH_R = @intToPtr(*volatile u32, 0x4001202C);
pub const UART6_CTL_R = @intToPtr(*volatile u32, 0x40012030);
pub const UART6_IFLS_R = @intToPtr(*volatile u32, 0x40012034);
pub const UART6_IM_R = @intToPtr(*volatile u32, 0x40012038);
pub const UART6_RIS_R = @intToPtr(*volatile u32, 0x4001203C);
pub const UART6_MIS_R = @intToPtr(*volatile u32, 0x40012040);
pub const UART6_ICR_R = @intToPtr(*volatile u32, 0x40012044);
pub const UART6_DMACTL_R = @intToPtr(*volatile u32, 0x40012048);
pub const UART6_9BITADDR_R = @intToPtr(*volatile u32, 0x400120A4);
pub const UART6_9BITAMASK_R = @intToPtr(*volatile u32, 0x400120A8);
pub const UART6_PP_R = @intToPtr(*volatile u32, 0x40012FC0);
pub const UART6_CC_R = @intToPtr(*volatile u32, 0x40012FC8);

//*****************************************************************************
//
// UART registers (UART7)
//
//*****************************************************************************
pub const UART7_DR_R = @intToPtr(*volatile u32, 0x40013000);
pub const UART7_RSR_R = @intToPtr(*volatile u32, 0x40013004);
pub const UART7_ECR_R = @intToPtr(*volatile u32, 0x40013004);
pub const UART7_FR_R = @intToPtr(*volatile u32, 0x40013018);
pub const UART7_ILPR_R = @intToPtr(*volatile u32, 0x40013020);
pub const UART7_IBRD_R = @intToPtr(*volatile u32, 0x40013024);
pub const UART7_FBRD_R = @intToPtr(*volatile u32, 0x40013028);
pub const UART7_LCRH_R = @intToPtr(*volatile u32, 0x4001302C);
pub const UART7_CTL_R = @intToPtr(*volatile u32, 0x40013030);
pub const UART7_IFLS_R = @intToPtr(*volatile u32, 0x40013034);
pub const UART7_IM_R = @intToPtr(*volatile u32, 0x40013038);
pub const UART7_RIS_R = @intToPtr(*volatile u32, 0x4001303C);
pub const UART7_MIS_R = @intToPtr(*volatile u32, 0x40013040);
pub const UART7_ICR_R = @intToPtr(*volatile u32, 0x40013044);
pub const UART7_DMACTL_R = @intToPtr(*volatile u32, 0x40013048);
pub const UART7_9BITADDR_R = @intToPtr(*volatile u32, 0x400130A4);
pub const UART7_9BITAMASK_R = @intToPtr(*volatile u32, 0x400130A8);
pub const UART7_PP_R = @intToPtr(*volatile u32, 0x40013FC0);
pub const UART7_CC_R = @intToPtr(*volatile u32, 0x40013FC8);

//*****************************************************************************
//
// I2C registers (I2C0)
//
//*****************************************************************************
pub const I2C0_MSA_R = @intToPtr(*volatile u32, 0x40020000);
pub const I2C0_MCS_R = @intToPtr(*volatile u32, 0x40020004);
pub const I2C0_MDR_R = @intToPtr(*volatile u32, 0x40020008);
pub const I2C0_MTPR_R = @intToPtr(*volatile u32, 0x4002000C);
pub const I2C0_MIMR_R = @intToPtr(*volatile u32, 0x40020010);
pub const I2C0_MRIS_R = @intToPtr(*volatile u32, 0x40020014);
pub const I2C0_MMIS_R = @intToPtr(*volatile u32, 0x40020018);
pub const I2C0_MICR_R = @intToPtr(*volatile u32, 0x4002001C);
pub const I2C0_MCR_R = @intToPtr(*volatile u32, 0x40020020);
pub const I2C0_MCLKOCNT_R = @intToPtr(*volatile u32, 0x40020024);
pub const I2C0_MBMON_R = @intToPtr(*volatile u32, 0x4002002C);
pub const I2C0_MBLEN_R = @intToPtr(*volatile u32, 0x40020030);
pub const I2C0_MBCNT_R = @intToPtr(*volatile u32, 0x40020034);
pub const I2C0_SOAR_R = @intToPtr(*volatile u32, 0x40020800);
pub const I2C0_SCSR_R = @intToPtr(*volatile u32, 0x40020804);
pub const I2C0_SDR_R = @intToPtr(*volatile u32, 0x40020808);
pub const I2C0_SIMR_R = @intToPtr(*volatile u32, 0x4002080C);
pub const I2C0_SRIS_R = @intToPtr(*volatile u32, 0x40020810);
pub const I2C0_SMIS_R = @intToPtr(*volatile u32, 0x40020814);
pub const I2C0_SICR_R = @intToPtr(*volatile u32, 0x40020818);
pub const I2C0_SOAR2_R = @intToPtr(*volatile u32, 0x4002081C);
pub const I2C0_SACKCTL_R = @intToPtr(*volatile u32, 0x40020820);
pub const I2C0_FIFODATA_R = @intToPtr(*volatile u32, 0x40020F00);
pub const I2C0_FIFOCTL_R = @intToPtr(*volatile u32, 0x40020F04);
pub const I2C0_FIFOSTATUS_R = @intToPtr(*volatile u32, 0x40020F08);
pub const I2C0_PP_R = @intToPtr(*volatile u32, 0x40020FC0);
pub const I2C0_PC_R = @intToPtr(*volatile u32, 0x40020FC4);

//*****************************************************************************
//
// I2C registers (I2C1)
//
//*****************************************************************************
pub const I2C1_MSA_R = @intToPtr(*volatile u32, 0x40021000);
pub const I2C1_MCS_R = @intToPtr(*volatile u32, 0x40021004);
pub const I2C1_MDR_R = @intToPtr(*volatile u32, 0x40021008);
pub const I2C1_MTPR_R = @intToPtr(*volatile u32, 0x4002100C);
pub const I2C1_MIMR_R = @intToPtr(*volatile u32, 0x40021010);
pub const I2C1_MRIS_R = @intToPtr(*volatile u32, 0x40021014);
pub const I2C1_MMIS_R = @intToPtr(*volatile u32, 0x40021018);
pub const I2C1_MICR_R = @intToPtr(*volatile u32, 0x4002101C);
pub const I2C1_MCR_R = @intToPtr(*volatile u32, 0x40021020);
pub const I2C1_MCLKOCNT_R = @intToPtr(*volatile u32, 0x40021024);
pub const I2C1_MBMON_R = @intToPtr(*volatile u32, 0x4002102C);
pub const I2C1_MBLEN_R = @intToPtr(*volatile u32, 0x40021030);
pub const I2C1_MBCNT_R = @intToPtr(*volatile u32, 0x40021034);
pub const I2C1_SOAR_R = @intToPtr(*volatile u32, 0x40021800);
pub const I2C1_SCSR_R = @intToPtr(*volatile u32, 0x40021804);
pub const I2C1_SDR_R = @intToPtr(*volatile u32, 0x40021808);
pub const I2C1_SIMR_R = @intToPtr(*volatile u32, 0x4002180C);
pub const I2C1_SRIS_R = @intToPtr(*volatile u32, 0x40021810);
pub const I2C1_SMIS_R = @intToPtr(*volatile u32, 0x40021814);
pub const I2C1_SICR_R = @intToPtr(*volatile u32, 0x40021818);
pub const I2C1_SOAR2_R = @intToPtr(*volatile u32, 0x4002181C);
pub const I2C1_SACKCTL_R = @intToPtr(*volatile u32, 0x40021820);
pub const I2C1_FIFODATA_R = @intToPtr(*volatile u32, 0x40021F00);
pub const I2C1_FIFOCTL_R = @intToPtr(*volatile u32, 0x40021F04);
pub const I2C1_FIFOSTATUS_R = @intToPtr(*volatile u32, 0x40021F08);
pub const I2C1_PP_R = @intToPtr(*volatile u32, 0x40021FC0);
pub const I2C1_PC_R = @intToPtr(*volatile u32, 0x40021FC4);

//*****************************************************************************
//
// I2C registers (I2C2)
//
//*****************************************************************************
pub const I2C2_MSA_R = @intToPtr(*volatile u32, 0x40022000);
pub const I2C2_MCS_R = @intToPtr(*volatile u32, 0x40022004);
pub const I2C2_MDR_R = @intToPtr(*volatile u32, 0x40022008);
pub const I2C2_MTPR_R = @intToPtr(*volatile u32, 0x4002200C);
pub const I2C2_MIMR_R = @intToPtr(*volatile u32, 0x40022010);
pub const I2C2_MRIS_R = @intToPtr(*volatile u32, 0x40022014);
pub const I2C2_MMIS_R = @intToPtr(*volatile u32, 0x40022018);
pub const I2C2_MICR_R = @intToPtr(*volatile u32, 0x4002201C);
pub const I2C2_MCR_R = @intToPtr(*volatile u32, 0x40022020);
pub const I2C2_MCLKOCNT_R = @intToPtr(*volatile u32, 0x40022024);
pub const I2C2_MBMON_R = @intToPtr(*volatile u32, 0x4002202C);
pub const I2C2_MBLEN_R = @intToPtr(*volatile u32, 0x40022030);
pub const I2C2_MBCNT_R = @intToPtr(*volatile u32, 0x40022034);
pub const I2C2_SOAR_R = @intToPtr(*volatile u32, 0x40022800);
pub const I2C2_SCSR_R = @intToPtr(*volatile u32, 0x40022804);
pub const I2C2_SDR_R = @intToPtr(*volatile u32, 0x40022808);
pub const I2C2_SIMR_R = @intToPtr(*volatile u32, 0x4002280C);
pub const I2C2_SRIS_R = @intToPtr(*volatile u32, 0x40022810);
pub const I2C2_SMIS_R = @intToPtr(*volatile u32, 0x40022814);
pub const I2C2_SICR_R = @intToPtr(*volatile u32, 0x40022818);
pub const I2C2_SOAR2_R = @intToPtr(*volatile u32, 0x4002281C);
pub const I2C2_SACKCTL_R = @intToPtr(*volatile u32, 0x40022820);
pub const I2C2_FIFODATA_R = @intToPtr(*volatile u32, 0x40022F00);
pub const I2C2_FIFOCTL_R = @intToPtr(*volatile u32, 0x40022F04);
pub const I2C2_FIFOSTATUS_R = @intToPtr(*volatile u32, 0x40022F08);
pub const I2C2_PP_R = @intToPtr(*volatile u32, 0x40022FC0);
pub const I2C2_PC_R = @intToPtr(*volatile u32, 0x40022FC4);

//*****************************************************************************
//
// I2C registers (I2C3)
//
//*****************************************************************************
pub const I2C3_MSA_R = @intToPtr(*volatile u32, 0x40023000);
pub const I2C3_MCS_R = @intToPtr(*volatile u32, 0x40023004);
pub const I2C3_MDR_R = @intToPtr(*volatile u32, 0x40023008);
pub const I2C3_MTPR_R = @intToPtr(*volatile u32, 0x4002300C);
pub const I2C3_MIMR_R = @intToPtr(*volatile u32, 0x40023010);
pub const I2C3_MRIS_R = @intToPtr(*volatile u32, 0x40023014);
pub const I2C3_MMIS_R = @intToPtr(*volatile u32, 0x40023018);
pub const I2C3_MICR_R = @intToPtr(*volatile u32, 0x4002301C);
pub const I2C3_MCR_R = @intToPtr(*volatile u32, 0x40023020);
pub const I2C3_MCLKOCNT_R = @intToPtr(*volatile u32, 0x40023024);
pub const I2C3_MBMON_R = @intToPtr(*volatile u32, 0x4002302C);
pub const I2C3_MBLEN_R = @intToPtr(*volatile u32, 0x40023030);
pub const I2C3_MBCNT_R = @intToPtr(*volatile u32, 0x40023034);
pub const I2C3_SOAR_R = @intToPtr(*volatile u32, 0x40023800);
pub const I2C3_SCSR_R = @intToPtr(*volatile u32, 0x40023804);
pub const I2C3_SDR_R = @intToPtr(*volatile u32, 0x40023808);
pub const I2C3_SIMR_R = @intToPtr(*volatile u32, 0x4002380C);
pub const I2C3_SRIS_R = @intToPtr(*volatile u32, 0x40023810);
pub const I2C3_SMIS_R = @intToPtr(*volatile u32, 0x40023814);
pub const I2C3_SICR_R = @intToPtr(*volatile u32, 0x40023818);
pub const I2C3_SOAR2_R = @intToPtr(*volatile u32, 0x4002381C);
pub const I2C3_SACKCTL_R = @intToPtr(*volatile u32, 0x40023820);
pub const I2C3_FIFODATA_R = @intToPtr(*volatile u32, 0x40023F00);
pub const I2C3_FIFOCTL_R = @intToPtr(*volatile u32, 0x40023F04);
pub const I2C3_FIFOSTATUS_R = @intToPtr(*volatile u32, 0x40023F08);
pub const I2C3_PP_R = @intToPtr(*volatile u32, 0x40023FC0);
pub const I2C3_PC_R = @intToPtr(*volatile u32, 0x40023FC4);

//*****************************************************************************
//
// PWM registers (PWM0)
//
//*****************************************************************************
pub const PWM0_CTL_R = @intToPtr(*volatile u32, 0x40028000);
pub const PWM0_SYNC_R = @intToPtr(*volatile u32, 0x40028004);
pub const PWM0_ENABLE_R = @intToPtr(*volatile u32, 0x40028008);
pub const PWM0_INVERT_R = @intToPtr(*volatile u32, 0x4002800C);
pub const PWM0_FAULT_R = @intToPtr(*volatile u32, 0x40028010);
pub const PWM0_INTEN_R = @intToPtr(*volatile u32, 0x40028014);
pub const PWM0_RIS_R = @intToPtr(*volatile u32, 0x40028018);
pub const PWM0_ISC_R = @intToPtr(*volatile u32, 0x4002801C);
pub const PWM0_STATUS_R = @intToPtr(*volatile u32, 0x40028020);
pub const PWM0_FAULTVAL_R = @intToPtr(*volatile u32, 0x40028024);
pub const PWM0_ENUPD_R = @intToPtr(*volatile u32, 0x40028028);
pub const PWM0_0_CTL_R = @intToPtr(*volatile u32, 0x40028040);
pub const PWM0_0_INTEN_R = @intToPtr(*volatile u32, 0x40028044);
pub const PWM0_0_RIS_R = @intToPtr(*volatile u32, 0x40028048);
pub const PWM0_0_ISC_R = @intToPtr(*volatile u32, 0x4002804C);
pub const PWM0_0_LOAD_R = @intToPtr(*volatile u32, 0x40028050);
pub const PWM0_0_COUNT_R = @intToPtr(*volatile u32, 0x40028054);
pub const PWM0_0_CMPA_R = @intToPtr(*volatile u32, 0x40028058);
pub const PWM0_0_CMPB_R = @intToPtr(*volatile u32, 0x4002805C);
pub const PWM0_0_GENA_R = @intToPtr(*volatile u32, 0x40028060);
pub const PWM0_0_GENB_R = @intToPtr(*volatile u32, 0x40028064);
pub const PWM0_0_DBCTL_R = @intToPtr(*volatile u32, 0x40028068);
pub const PWM0_0_DBRISE_R = @intToPtr(*volatile u32, 0x4002806C);
pub const PWM0_0_DBFALL_R = @intToPtr(*volatile u32, 0x40028070);
pub const PWM0_0_FLTSRC0_R = @intToPtr(*volatile u32, 0x40028074);
pub const PWM0_0_FLTSRC1_R = @intToPtr(*volatile u32, 0x40028078);
pub const PWM0_0_MINFLTPER_R = @intToPtr(*volatile u32, 0x4002807C);
pub const PWM0_1_CTL_R = @intToPtr(*volatile u32, 0x40028080);
pub const PWM0_1_INTEN_R = @intToPtr(*volatile u32, 0x40028084);
pub const PWM0_1_RIS_R = @intToPtr(*volatile u32, 0x40028088);
pub const PWM0_1_ISC_R = @intToPtr(*volatile u32, 0x4002808C);
pub const PWM0_1_LOAD_R = @intToPtr(*volatile u32, 0x40028090);
pub const PWM0_1_COUNT_R = @intToPtr(*volatile u32, 0x40028094);
pub const PWM0_1_CMPA_R = @intToPtr(*volatile u32, 0x40028098);
pub const PWM0_1_CMPB_R = @intToPtr(*volatile u32, 0x4002809C);
pub const PWM0_1_GENA_R = @intToPtr(*volatile u32, 0x400280A0);
pub const PWM0_1_GENB_R = @intToPtr(*volatile u32, 0x400280A4);
pub const PWM0_1_DBCTL_R = @intToPtr(*volatile u32, 0x400280A8);
pub const PWM0_1_DBRISE_R = @intToPtr(*volatile u32, 0x400280AC);
pub const PWM0_1_DBFALL_R = @intToPtr(*volatile u32, 0x400280B0);
pub const PWM0_1_FLTSRC0_R = @intToPtr(*volatile u32, 0x400280B4);
pub const PWM0_1_FLTSRC1_R = @intToPtr(*volatile u32, 0x400280B8);
pub const PWM0_1_MINFLTPER_R = @intToPtr(*volatile u32, 0x400280BC);
pub const PWM0_2_CTL_R = @intToPtr(*volatile u32, 0x400280C0);
pub const PWM0_2_INTEN_R = @intToPtr(*volatile u32, 0x400280C4);
pub const PWM0_2_RIS_R = @intToPtr(*volatile u32, 0x400280C8);
pub const PWM0_2_ISC_R = @intToPtr(*volatile u32, 0x400280CC);
pub const PWM0_2_LOAD_R = @intToPtr(*volatile u32, 0x400280D0);
pub const PWM0_2_COUNT_R = @intToPtr(*volatile u32, 0x400280D4);
pub const PWM0_2_CMPA_R = @intToPtr(*volatile u32, 0x400280D8);
pub const PWM0_2_CMPB_R = @intToPtr(*volatile u32, 0x400280DC);
pub const PWM0_2_GENA_R = @intToPtr(*volatile u32, 0x400280E0);
pub const PWM0_2_GENB_R = @intToPtr(*volatile u32, 0x400280E4);
pub const PWM0_2_DBCTL_R = @intToPtr(*volatile u32, 0x400280E8);
pub const PWM0_2_DBRISE_R = @intToPtr(*volatile u32, 0x400280EC);
pub const PWM0_2_DBFALL_R = @intToPtr(*volatile u32, 0x400280F0);
pub const PWM0_2_FLTSRC0_R = @intToPtr(*volatile u32, 0x400280F4);
pub const PWM0_2_FLTSRC1_R = @intToPtr(*volatile u32, 0x400280F8);
pub const PWM0_2_MINFLTPER_R = @intToPtr(*volatile u32, 0x400280FC);
pub const PWM0_3_CTL_R = @intToPtr(*volatile u32, 0x40028100);
pub const PWM0_3_INTEN_R = @intToPtr(*volatile u32, 0x40028104);
pub const PWM0_3_RIS_R = @intToPtr(*volatile u32, 0x40028108);
pub const PWM0_3_ISC_R = @intToPtr(*volatile u32, 0x4002810C);
pub const PWM0_3_LOAD_R = @intToPtr(*volatile u32, 0x40028110);
pub const PWM0_3_COUNT_R = @intToPtr(*volatile u32, 0x40028114);
pub const PWM0_3_CMPA_R = @intToPtr(*volatile u32, 0x40028118);
pub const PWM0_3_CMPB_R = @intToPtr(*volatile u32, 0x4002811C);
pub const PWM0_3_GENA_R = @intToPtr(*volatile u32, 0x40028120);
pub const PWM0_3_GENB_R = @intToPtr(*volatile u32, 0x40028124);
pub const PWM0_3_DBCTL_R = @intToPtr(*volatile u32, 0x40028128);
pub const PWM0_3_DBRISE_R = @intToPtr(*volatile u32, 0x4002812C);
pub const PWM0_3_DBFALL_R = @intToPtr(*volatile u32, 0x40028130);
pub const PWM0_3_FLTSRC0_R = @intToPtr(*volatile u32, 0x40028134);
pub const PWM0_3_FLTSRC1_R = @intToPtr(*volatile u32, 0x40028138);
pub const PWM0_3_MINFLTPER_R = @intToPtr(*volatile u32, 0x4002813C);
pub const PWM0_0_FLTSEN_R = @intToPtr(*volatile u32, 0x40028800);
pub const PWM0_0_FLTSTAT0_R = @intToPtr(*volatile u32, 0x40028804);
pub const PWM0_0_FLTSTAT1_R = @intToPtr(*volatile u32, 0x40028808);
pub const PWM0_1_FLTSEN_R = @intToPtr(*volatile u32, 0x40028880);
pub const PWM0_1_FLTSTAT0_R = @intToPtr(*volatile u32, 0x40028884);
pub const PWM0_1_FLTSTAT1_R = @intToPtr(*volatile u32, 0x40028888);
pub const PWM0_2_FLTSEN_R = @intToPtr(*volatile u32, 0x40028900);
pub const PWM0_2_FLTSTAT0_R = @intToPtr(*volatile u32, 0x40028904);
pub const PWM0_2_FLTSTAT1_R = @intToPtr(*volatile u32, 0x40028908);
pub const PWM0_3_FLTSEN_R = @intToPtr(*volatile u32, 0x40028980);
pub const PWM0_3_FLTSTAT0_R = @intToPtr(*volatile u32, 0x40028984);
pub const PWM0_3_FLTSTAT1_R = @intToPtr(*volatile u32, 0x40028988);
pub const PWM0_PP_R = @intToPtr(*volatile u32, 0x40028FC0);
pub const PWM0_CC_R = @intToPtr(*volatile u32, 0x40028FC8);

//*****************************************************************************
//
// QEI registers (QEI0)
//
//*****************************************************************************
pub const QEI0_CTL_R = @intToPtr(*volatile u32, 0x4002C000);
pub const QEI0_STAT_R = @intToPtr(*volatile u32, 0x4002C004);
pub const QEI0_POS_R = @intToPtr(*volatile u32, 0x4002C008);
pub const QEI0_MAXPOS_R = @intToPtr(*volatile u32, 0x4002C00C);
pub const QEI0_LOAD_R = @intToPtr(*volatile u32, 0x4002C010);
pub const QEI0_TIME_R = @intToPtr(*volatile u32, 0x4002C014);
pub const QEI0_COUNT_R = @intToPtr(*volatile u32, 0x4002C018);
pub const QEI0_SPEED_R = @intToPtr(*volatile u32, 0x4002C01C);
pub const QEI0_INTEN_R = @intToPtr(*volatile u32, 0x4002C020);
pub const QEI0_RIS_R = @intToPtr(*volatile u32, 0x4002C024);
pub const QEI0_ISC_R = @intToPtr(*volatile u32, 0x4002C028);

//*****************************************************************************
//
// Timer registers (TIMER0)
//
//*****************************************************************************
pub const TIMER0_CFG_R = @intToPtr(*volatile u32, 0x40030000);
pub const TIMER0_TAMR_R = @intToPtr(*volatile u32, 0x40030004);
pub const TIMER0_TBMR_R = @intToPtr(*volatile u32, 0x40030008);
pub const TIMER0_CTL_R = @intToPtr(*volatile u32, 0x4003000C);
pub const TIMER0_SYNC_R = @intToPtr(*volatile u32, 0x40030010);
pub const TIMER0_IMR_R = @intToPtr(*volatile u32, 0x40030018);
pub const TIMER0_RIS_R = @intToPtr(*volatile u32, 0x4003001C);
pub const TIMER0_MIS_R = @intToPtr(*volatile u32, 0x40030020);
pub const TIMER0_ICR_R = @intToPtr(*volatile u32, 0x40030024);
pub const TIMER0_TAILR_R = @intToPtr(*volatile u32, 0x40030028);
pub const TIMER0_TBILR_R = @intToPtr(*volatile u32, 0x4003002C);
pub const TIMER0_TAMATCHR_R = @intToPtr(*volatile u32, 0x40030030);
pub const TIMER0_TBMATCHR_R = @intToPtr(*volatile u32, 0x40030034);
pub const TIMER0_TAPR_R = @intToPtr(*volatile u32, 0x40030038);
pub const TIMER0_TBPR_R = @intToPtr(*volatile u32, 0x4003003C);
pub const TIMER0_TAPMR_R = @intToPtr(*volatile u32, 0x40030040);
pub const TIMER0_TBPMR_R = @intToPtr(*volatile u32, 0x40030044);
pub const TIMER0_TAR_R = @intToPtr(*volatile u32, 0x40030048);
pub const TIMER0_TBR_R = @intToPtr(*volatile u32, 0x4003004C);
pub const TIMER0_TAV_R = @intToPtr(*volatile u32, 0x40030050);
pub const TIMER0_TBV_R = @intToPtr(*volatile u32, 0x40030054);
pub const TIMER0_RTCPD_R = @intToPtr(*volatile u32, 0x40030058);
pub const TIMER0_TAPS_R = @intToPtr(*volatile u32, 0x4003005C);
pub const TIMER0_TBPS_R = @intToPtr(*volatile u32, 0x40030060);
pub const TIMER0_DMAEV_R = @intToPtr(*volatile u32, 0x4003006C);
pub const TIMER0_ADCEV_R = @intToPtr(*volatile u32, 0x40030070);
pub const TIMER0_PP_R = @intToPtr(*volatile u32, 0x40030FC0);
pub const TIMER0_CC_R = @intToPtr(*volatile u32, 0x40030FC8);

//*****************************************************************************
//
// Timer registers (TIMER1)
//
//*****************************************************************************
pub const TIMER1_CFG_R = @intToPtr(*volatile u32, 0x40031000);
pub const TIMER1_TAMR_R = @intToPtr(*volatile u32, 0x40031004);
pub const TIMER1_TBMR_R = @intToPtr(*volatile u32, 0x40031008);
pub const TIMER1_CTL_R = @intToPtr(*volatile u32, 0x4003100C);
pub const TIMER1_SYNC_R = @intToPtr(*volatile u32, 0x40031010);
pub const TIMER1_IMR_R = @intToPtr(*volatile u32, 0x40031018);
pub const TIMER1_RIS_R = @intToPtr(*volatile u32, 0x4003101C);
pub const TIMER1_MIS_R = @intToPtr(*volatile u32, 0x40031020);
pub const TIMER1_ICR_R = @intToPtr(*volatile u32, 0x40031024);
pub const TIMER1_TAILR_R = @intToPtr(*volatile u32, 0x40031028);
pub const TIMER1_TBILR_R = @intToPtr(*volatile u32, 0x4003102C);
pub const TIMER1_TAMATCHR_R = @intToPtr(*volatile u32, 0x40031030);
pub const TIMER1_TBMATCHR_R = @intToPtr(*volatile u32, 0x40031034);
pub const TIMER1_TAPR_R = @intToPtr(*volatile u32, 0x40031038);
pub const TIMER1_TBPR_R = @intToPtr(*volatile u32, 0x4003103C);
pub const TIMER1_TAPMR_R = @intToPtr(*volatile u32, 0x40031040);
pub const TIMER1_TBPMR_R = @intToPtr(*volatile u32, 0x40031044);
pub const TIMER1_TAR_R = @intToPtr(*volatile u32, 0x40031048);
pub const TIMER1_TBR_R = @intToPtr(*volatile u32, 0x4003104C);
pub const TIMER1_TAV_R = @intToPtr(*volatile u32, 0x40031050);
pub const TIMER1_TBV_R = @intToPtr(*volatile u32, 0x40031054);
pub const TIMER1_RTCPD_R = @intToPtr(*volatile u32, 0x40031058);
pub const TIMER1_TAPS_R = @intToPtr(*volatile u32, 0x4003105C);
pub const TIMER1_TBPS_R = @intToPtr(*volatile u32, 0x40031060);
pub const TIMER1_DMAEV_R = @intToPtr(*volatile u32, 0x4003106C);
pub const TIMER1_ADCEV_R = @intToPtr(*volatile u32, 0x40031070);
pub const TIMER1_PP_R = @intToPtr(*volatile u32, 0x40031FC0);
pub const TIMER1_CC_R = @intToPtr(*volatile u32, 0x40031FC8);

//*****************************************************************************
//
// Timer registers (TIMER2)
//
//*****************************************************************************
pub const TIMER2_CFG_R = @intToPtr(*volatile u32, 0x40032000);
pub const TIMER2_TAMR_R = @intToPtr(*volatile u32, 0x40032004);
pub const TIMER2_TBMR_R = @intToPtr(*volatile u32, 0x40032008);
pub const TIMER2_CTL_R = @intToPtr(*volatile u32, 0x4003200C);
pub const TIMER2_SYNC_R = @intToPtr(*volatile u32, 0x40032010);
pub const TIMER2_IMR_R = @intToPtr(*volatile u32, 0x40032018);
pub const TIMER2_RIS_R = @intToPtr(*volatile u32, 0x4003201C);
pub const TIMER2_MIS_R = @intToPtr(*volatile u32, 0x40032020);
pub const TIMER2_ICR_R = @intToPtr(*volatile u32, 0x40032024);
pub const TIMER2_TAILR_R = @intToPtr(*volatile u32, 0x40032028);
pub const TIMER2_TBILR_R = @intToPtr(*volatile u32, 0x4003202C);
pub const TIMER2_TAMATCHR_R = @intToPtr(*volatile u32, 0x40032030);
pub const TIMER2_TBMATCHR_R = @intToPtr(*volatile u32, 0x40032034);
pub const TIMER2_TAPR_R = @intToPtr(*volatile u32, 0x40032038);
pub const TIMER2_TBPR_R = @intToPtr(*volatile u32, 0x4003203C);
pub const TIMER2_TAPMR_R = @intToPtr(*volatile u32, 0x40032040);
pub const TIMER2_TBPMR_R = @intToPtr(*volatile u32, 0x40032044);
pub const TIMER2_TAR_R = @intToPtr(*volatile u32, 0x40032048);
pub const TIMER2_TBR_R = @intToPtr(*volatile u32, 0x4003204C);
pub const TIMER2_TAV_R = @intToPtr(*volatile u32, 0x40032050);
pub const TIMER2_TBV_R = @intToPtr(*volatile u32, 0x40032054);
pub const TIMER2_RTCPD_R = @intToPtr(*volatile u32, 0x40032058);
pub const TIMER2_TAPS_R = @intToPtr(*volatile u32, 0x4003205C);
pub const TIMER2_TBPS_R = @intToPtr(*volatile u32, 0x40032060);
pub const TIMER2_DMAEV_R = @intToPtr(*volatile u32, 0x4003206C);
pub const TIMER2_ADCEV_R = @intToPtr(*volatile u32, 0x40032070);
pub const TIMER2_PP_R = @intToPtr(*volatile u32, 0x40032FC0);
pub const TIMER2_CC_R = @intToPtr(*volatile u32, 0x40032FC8);

//*****************************************************************************
//
// Timer registers (TIMER3)
//
//*****************************************************************************
pub const TIMER3_CFG_R = @intToPtr(*volatile u32, 0x40033000);
pub const TIMER3_TAMR_R = @intToPtr(*volatile u32, 0x40033004);
pub const TIMER3_TBMR_R = @intToPtr(*volatile u32, 0x40033008);
pub const TIMER3_CTL_R = @intToPtr(*volatile u32, 0x4003300C);
pub const TIMER3_SYNC_R = @intToPtr(*volatile u32, 0x40033010);
pub const TIMER3_IMR_R = @intToPtr(*volatile u32, 0x40033018);
pub const TIMER3_RIS_R = @intToPtr(*volatile u32, 0x4003301C);
pub const TIMER3_MIS_R = @intToPtr(*volatile u32, 0x40033020);
pub const TIMER3_ICR_R = @intToPtr(*volatile u32, 0x40033024);
pub const TIMER3_TAILR_R = @intToPtr(*volatile u32, 0x40033028);
pub const TIMER3_TBILR_R = @intToPtr(*volatile u32, 0x4003302C);
pub const TIMER3_TAMATCHR_R = @intToPtr(*volatile u32, 0x40033030);
pub const TIMER3_TBMATCHR_R = @intToPtr(*volatile u32, 0x40033034);
pub const TIMER3_TAPR_R = @intToPtr(*volatile u32, 0x40033038);
pub const TIMER3_TBPR_R = @intToPtr(*volatile u32, 0x4003303C);
pub const TIMER3_TAPMR_R = @intToPtr(*volatile u32, 0x40033040);
pub const TIMER3_TBPMR_R = @intToPtr(*volatile u32, 0x40033044);
pub const TIMER3_TAR_R = @intToPtr(*volatile u32, 0x40033048);
pub const TIMER3_TBR_R = @intToPtr(*volatile u32, 0x4003304C);
pub const TIMER3_TAV_R = @intToPtr(*volatile u32, 0x40033050);
pub const TIMER3_TBV_R = @intToPtr(*volatile u32, 0x40033054);
pub const TIMER3_RTCPD_R = @intToPtr(*volatile u32, 0x40033058);
pub const TIMER3_TAPS_R = @intToPtr(*volatile u32, 0x4003305C);
pub const TIMER3_TBPS_R = @intToPtr(*volatile u32, 0x40033060);
pub const TIMER3_DMAEV_R = @intToPtr(*volatile u32, 0x4003306C);
pub const TIMER3_ADCEV_R = @intToPtr(*volatile u32, 0x40033070);
pub const TIMER3_PP_R = @intToPtr(*volatile u32, 0x40033FC0);
pub const TIMER3_CC_R = @intToPtr(*volatile u32, 0x40033FC8);

//*****************************************************************************
//
// Timer registers (TIMER4)
//
//*****************************************************************************
pub const TIMER4_CFG_R = @intToPtr(*volatile u32, 0x40034000);
pub const TIMER4_TAMR_R = @intToPtr(*volatile u32, 0x40034004);
pub const TIMER4_TBMR_R = @intToPtr(*volatile u32, 0x40034008);
pub const TIMER4_CTL_R = @intToPtr(*volatile u32, 0x4003400C);
pub const TIMER4_SYNC_R = @intToPtr(*volatile u32, 0x40034010);
pub const TIMER4_IMR_R = @intToPtr(*volatile u32, 0x40034018);
pub const TIMER4_RIS_R = @intToPtr(*volatile u32, 0x4003401C);
pub const TIMER4_MIS_R = @intToPtr(*volatile u32, 0x40034020);
pub const TIMER4_ICR_R = @intToPtr(*volatile u32, 0x40034024);
pub const TIMER4_TAILR_R = @intToPtr(*volatile u32, 0x40034028);
pub const TIMER4_TBILR_R = @intToPtr(*volatile u32, 0x4003402C);
pub const TIMER4_TAMATCHR_R = @intToPtr(*volatile u32, 0x40034030);
pub const TIMER4_TBMATCHR_R = @intToPtr(*volatile u32, 0x40034034);
pub const TIMER4_TAPR_R = @intToPtr(*volatile u32, 0x40034038);
pub const TIMER4_TBPR_R = @intToPtr(*volatile u32, 0x4003403C);
pub const TIMER4_TAPMR_R = @intToPtr(*volatile u32, 0x40034040);
pub const TIMER4_TBPMR_R = @intToPtr(*volatile u32, 0x40034044);
pub const TIMER4_TAR_R = @intToPtr(*volatile u32, 0x40034048);
pub const TIMER4_TBR_R = @intToPtr(*volatile u32, 0x4003404C);
pub const TIMER4_TAV_R = @intToPtr(*volatile u32, 0x40034050);
pub const TIMER4_TBV_R = @intToPtr(*volatile u32, 0x40034054);
pub const TIMER4_RTCPD_R = @intToPtr(*volatile u32, 0x40034058);
pub const TIMER4_TAPS_R = @intToPtr(*volatile u32, 0x4003405C);
pub const TIMER4_TBPS_R = @intToPtr(*volatile u32, 0x40034060);
pub const TIMER4_DMAEV_R = @intToPtr(*volatile u32, 0x4003406C);
pub const TIMER4_ADCEV_R = @intToPtr(*volatile u32, 0x40034070);
pub const TIMER4_PP_R = @intToPtr(*volatile u32, 0x40034FC0);
pub const TIMER4_CC_R = @intToPtr(*volatile u32, 0x40034FC8);

//*****************************************************************************
//
// Timer registers (TIMER5)
//
//*****************************************************************************
pub const TIMER5_CFG_R = @intToPtr(*volatile u32, 0x40035000);
pub const TIMER5_TAMR_R = @intToPtr(*volatile u32, 0x40035004);
pub const TIMER5_TBMR_R = @intToPtr(*volatile u32, 0x40035008);
pub const TIMER5_CTL_R = @intToPtr(*volatile u32, 0x4003500C);
pub const TIMER5_SYNC_R = @intToPtr(*volatile u32, 0x40035010);
pub const TIMER5_IMR_R = @intToPtr(*volatile u32, 0x40035018);
pub const TIMER5_RIS_R = @intToPtr(*volatile u32, 0x4003501C);
pub const TIMER5_MIS_R = @intToPtr(*volatile u32, 0x40035020);
pub const TIMER5_ICR_R = @intToPtr(*volatile u32, 0x40035024);
pub const TIMER5_TAILR_R = @intToPtr(*volatile u32, 0x40035028);
pub const TIMER5_TBILR_R = @intToPtr(*volatile u32, 0x4003502C);
pub const TIMER5_TAMATCHR_R = @intToPtr(*volatile u32, 0x40035030);
pub const TIMER5_TBMATCHR_R = @intToPtr(*volatile u32, 0x40035034);
pub const TIMER5_TAPR_R = @intToPtr(*volatile u32, 0x40035038);
pub const TIMER5_TBPR_R = @intToPtr(*volatile u32, 0x4003503C);
pub const TIMER5_TAPMR_R = @intToPtr(*volatile u32, 0x40035040);
pub const TIMER5_TBPMR_R = @intToPtr(*volatile u32, 0x40035044);
pub const TIMER5_TAR_R = @intToPtr(*volatile u32, 0x40035048);
pub const TIMER5_TBR_R = @intToPtr(*volatile u32, 0x4003504C);
pub const TIMER5_TAV_R = @intToPtr(*volatile u32, 0x40035050);
pub const TIMER5_TBV_R = @intToPtr(*volatile u32, 0x40035054);
pub const TIMER5_RTCPD_R = @intToPtr(*volatile u32, 0x40035058);
pub const TIMER5_TAPS_R = @intToPtr(*volatile u32, 0x4003505C);
pub const TIMER5_TBPS_R = @intToPtr(*volatile u32, 0x40035060);
pub const TIMER5_DMAEV_R = @intToPtr(*volatile u32, 0x4003506C);
pub const TIMER5_ADCEV_R = @intToPtr(*volatile u32, 0x40035070);
pub const TIMER5_PP_R = @intToPtr(*volatile u32, 0x40035FC0);
pub const TIMER5_CC_R = @intToPtr(*volatile u32, 0x40035FC8);

//*****************************************************************************
//
// ADC registers (ADC0)
//
//*****************************************************************************
pub const ADC0_ACTSS_R = @intToPtr(*volatile u32, 0x40038000);
pub const ADC0_RIS_R = @intToPtr(*volatile u32, 0x40038004);
pub const ADC0_IM_R = @intToPtr(*volatile u32, 0x40038008);
pub const ADC0_ISC_R = @intToPtr(*volatile u32, 0x4003800C);
pub const ADC0_OSTAT_R = @intToPtr(*volatile u32, 0x40038010);
pub const ADC0_EMUX_R = @intToPtr(*volatile u32, 0x40038014);
pub const ADC0_USTAT_R = @intToPtr(*volatile u32, 0x40038018);
pub const ADC0_TSSEL_R = @intToPtr(*volatile u32, 0x4003801C);
pub const ADC0_SSPRI_R = @intToPtr(*volatile u32, 0x40038020);
pub const ADC0_SPC_R = @intToPtr(*volatile u32, 0x40038024);
pub const ADC0_PSSI_R = @intToPtr(*volatile u32, 0x40038028);
pub const ADC0_SAC_R = @intToPtr(*volatile u32, 0x40038030);
pub const ADC0_DCISC_R = @intToPtr(*volatile u32, 0x40038034);
pub const ADC0_CTL_R = @intToPtr(*volatile u32, 0x40038038);
pub const ADC0_SSMUX0_R = @intToPtr(*volatile u32, 0x40038040);
pub const ADC0_SSCTL0_R = @intToPtr(*volatile u32, 0x40038044);
pub const ADC0_SSFIFO0_R = @intToPtr(*volatile u32, 0x40038048);
pub const ADC0_SSFSTAT0_R = @intToPtr(*volatile u32, 0x4003804C);
pub const ADC0_SSOP0_R = @intToPtr(*volatile u32, 0x40038050);
pub const ADC0_SSDC0_R = @intToPtr(*volatile u32, 0x40038054);
pub const ADC0_SSEMUX0_R = @intToPtr(*volatile u32, 0x40038058);
pub const ADC0_SSTSH0_R = @intToPtr(*volatile u32, 0x4003805C);
pub const ADC0_SSMUX1_R = @intToPtr(*volatile u32, 0x40038060);
pub const ADC0_SSCTL1_R = @intToPtr(*volatile u32, 0x40038064);
pub const ADC0_SSFIFO1_R = @intToPtr(*volatile u32, 0x40038068);
pub const ADC0_SSFSTAT1_R = @intToPtr(*volatile u32, 0x4003806C);
pub const ADC0_SSOP1_R = @intToPtr(*volatile u32, 0x40038070);
pub const ADC0_SSDC1_R = @intToPtr(*volatile u32, 0x40038074);
pub const ADC0_SSEMUX1_R = @intToPtr(*volatile u32, 0x40038078);
pub const ADC0_SSTSH1_R = @intToPtr(*volatile u32, 0x4003807C);
pub const ADC0_SSMUX2_R = @intToPtr(*volatile u32, 0x40038080);
pub const ADC0_SSCTL2_R = @intToPtr(*volatile u32, 0x40038084);
pub const ADC0_SSFIFO2_R = @intToPtr(*volatile u32, 0x40038088);
pub const ADC0_SSFSTAT2_R = @intToPtr(*volatile u32, 0x4003808C);
pub const ADC0_SSOP2_R = @intToPtr(*volatile u32, 0x40038090);
pub const ADC0_SSDC2_R = @intToPtr(*volatile u32, 0x40038094);
pub const ADC0_SSEMUX2_R = @intToPtr(*volatile u32, 0x40038098);
pub const ADC0_SSTSH2_R = @intToPtr(*volatile u32, 0x4003809C);
pub const ADC0_SSMUX3_R = @intToPtr(*volatile u32, 0x400380A0);
pub const ADC0_SSCTL3_R = @intToPtr(*volatile u32, 0x400380A4);
pub const ADC0_SSFIFO3_R = @intToPtr(*volatile u32, 0x400380A8);
pub const ADC0_SSFSTAT3_R = @intToPtr(*volatile u32, 0x400380AC);
pub const ADC0_SSOP3_R = @intToPtr(*volatile u32, 0x400380B0);
pub const ADC0_SSDC3_R = @intToPtr(*volatile u32, 0x400380B4);
pub const ADC0_SSEMUX3_R = @intToPtr(*volatile u32, 0x400380B8);
pub const ADC0_SSTSH3_R = @intToPtr(*volatile u32, 0x400380BC);
pub const ADC0_DCRIC_R = @intToPtr(*volatile u32, 0x40038D00);
pub const ADC0_DCCTL0_R = @intToPtr(*volatile u32, 0x40038E00);
pub const ADC0_DCCTL1_R = @intToPtr(*volatile u32, 0x40038E04);
pub const ADC0_DCCTL2_R = @intToPtr(*volatile u32, 0x40038E08);
pub const ADC0_DCCTL3_R = @intToPtr(*volatile u32, 0x40038E0C);
pub const ADC0_DCCTL4_R = @intToPtr(*volatile u32, 0x40038E10);
pub const ADC0_DCCTL5_R = @intToPtr(*volatile u32, 0x40038E14);
pub const ADC0_DCCTL6_R = @intToPtr(*volatile u32, 0x40038E18);
pub const ADC0_DCCTL7_R = @intToPtr(*volatile u32, 0x40038E1C);
pub const ADC0_DCCMP0_R = @intToPtr(*volatile u32, 0x40038E40);
pub const ADC0_DCCMP1_R = @intToPtr(*volatile u32, 0x40038E44);
pub const ADC0_DCCMP2_R = @intToPtr(*volatile u32, 0x40038E48);
pub const ADC0_DCCMP3_R = @intToPtr(*volatile u32, 0x40038E4C);
pub const ADC0_DCCMP4_R = @intToPtr(*volatile u32, 0x40038E50);
pub const ADC0_DCCMP5_R = @intToPtr(*volatile u32, 0x40038E54);
pub const ADC0_DCCMP6_R = @intToPtr(*volatile u32, 0x40038E58);
pub const ADC0_DCCMP7_R = @intToPtr(*volatile u32, 0x40038E5C);
pub const ADC0_PP_R = @intToPtr(*volatile u32, 0x40038FC0);
pub const ADC0_PC_R = @intToPtr(*volatile u32, 0x40038FC4);
pub const ADC0_CC_R = @intToPtr(*volatile u32, 0x40038FC8);

//*****************************************************************************
//
// ADC registers (ADC1)
//
//*****************************************************************************
pub const ADC1_ACTSS_R = @intToPtr(*volatile u32, 0x40039000);
pub const ADC1_RIS_R = @intToPtr(*volatile u32, 0x40039004);
pub const ADC1_IM_R = @intToPtr(*volatile u32, 0x40039008);
pub const ADC1_ISC_R = @intToPtr(*volatile u32, 0x4003900C);
pub const ADC1_OSTAT_R = @intToPtr(*volatile u32, 0x40039010);
pub const ADC1_EMUX_R = @intToPtr(*volatile u32, 0x40039014);
pub const ADC1_USTAT_R = @intToPtr(*volatile u32, 0x40039018);
pub const ADC1_TSSEL_R = @intToPtr(*volatile u32, 0x4003901C);
pub const ADC1_SSPRI_R = @intToPtr(*volatile u32, 0x40039020);
pub const ADC1_SPC_R = @intToPtr(*volatile u32, 0x40039024);
pub const ADC1_PSSI_R = @intToPtr(*volatile u32, 0x40039028);
pub const ADC1_SAC_R = @intToPtr(*volatile u32, 0x40039030);
pub const ADC1_DCISC_R = @intToPtr(*volatile u32, 0x40039034);
pub const ADC1_CTL_R = @intToPtr(*volatile u32, 0x40039038);
pub const ADC1_SSMUX0_R = @intToPtr(*volatile u32, 0x40039040);
pub const ADC1_SSCTL0_R = @intToPtr(*volatile u32, 0x40039044);
pub const ADC1_SSFIFO0_R = @intToPtr(*volatile u32, 0x40039048);
pub const ADC1_SSFSTAT0_R = @intToPtr(*volatile u32, 0x4003904C);
pub const ADC1_SSOP0_R = @intToPtr(*volatile u32, 0x40039050);
pub const ADC1_SSDC0_R = @intToPtr(*volatile u32, 0x40039054);
pub const ADC1_SSEMUX0_R = @intToPtr(*volatile u32, 0x40039058);
pub const ADC1_SSTSH0_R = @intToPtr(*volatile u32, 0x4003905C);
pub const ADC1_SSMUX1_R = @intToPtr(*volatile u32, 0x40039060);
pub const ADC1_SSCTL1_R = @intToPtr(*volatile u32, 0x40039064);
pub const ADC1_SSFIFO1_R = @intToPtr(*volatile u32, 0x40039068);
pub const ADC1_SSFSTAT1_R = @intToPtr(*volatile u32, 0x4003906C);
pub const ADC1_SSOP1_R = @intToPtr(*volatile u32, 0x40039070);
pub const ADC1_SSDC1_R = @intToPtr(*volatile u32, 0x40039074);
pub const ADC1_SSEMUX1_R = @intToPtr(*volatile u32, 0x40039078);
pub const ADC1_SSTSH1_R = @intToPtr(*volatile u32, 0x4003907C);
pub const ADC1_SSMUX2_R = @intToPtr(*volatile u32, 0x40039080);
pub const ADC1_SSCTL2_R = @intToPtr(*volatile u32, 0x40039084);
pub const ADC1_SSFIFO2_R = @intToPtr(*volatile u32, 0x40039088);
pub const ADC1_SSFSTAT2_R = @intToPtr(*volatile u32, 0x4003908C);
pub const ADC1_SSOP2_R = @intToPtr(*volatile u32, 0x40039090);
pub const ADC1_SSDC2_R = @intToPtr(*volatile u32, 0x40039094);
pub const ADC1_SSEMUX2_R = @intToPtr(*volatile u32, 0x40039098);
pub const ADC1_SSTSH2_R = @intToPtr(*volatile u32, 0x4003909C);
pub const ADC1_SSMUX3_R = @intToPtr(*volatile u32, 0x400390A0);
pub const ADC1_SSCTL3_R = @intToPtr(*volatile u32, 0x400390A4);
pub const ADC1_SSFIFO3_R = @intToPtr(*volatile u32, 0x400390A8);
pub const ADC1_SSFSTAT3_R = @intToPtr(*volatile u32, 0x400390AC);
pub const ADC1_SSOP3_R = @intToPtr(*volatile u32, 0x400390B0);
pub const ADC1_SSDC3_R = @intToPtr(*volatile u32, 0x400390B4);
pub const ADC1_SSEMUX3_R = @intToPtr(*volatile u32, 0x400390B8);
pub const ADC1_SSTSH3_R = @intToPtr(*volatile u32, 0x400390BC);
pub const ADC1_DCRIC_R = @intToPtr(*volatile u32, 0x40039D00);
pub const ADC1_DCCTL0_R = @intToPtr(*volatile u32, 0x40039E00);
pub const ADC1_DCCTL1_R = @intToPtr(*volatile u32, 0x40039E04);
pub const ADC1_DCCTL2_R = @intToPtr(*volatile u32, 0x40039E08);
pub const ADC1_DCCTL3_R = @intToPtr(*volatile u32, 0x40039E0C);
pub const ADC1_DCCTL4_R = @intToPtr(*volatile u32, 0x40039E10);
pub const ADC1_DCCTL5_R = @intToPtr(*volatile u32, 0x40039E14);
pub const ADC1_DCCTL6_R = @intToPtr(*volatile u32, 0x40039E18);
pub const ADC1_DCCTL7_R = @intToPtr(*volatile u32, 0x40039E1C);
pub const ADC1_DCCMP0_R = @intToPtr(*volatile u32, 0x40039E40);
pub const ADC1_DCCMP1_R = @intToPtr(*volatile u32, 0x40039E44);
pub const ADC1_DCCMP2_R = @intToPtr(*volatile u32, 0x40039E48);
pub const ADC1_DCCMP3_R = @intToPtr(*volatile u32, 0x40039E4C);
pub const ADC1_DCCMP4_R = @intToPtr(*volatile u32, 0x40039E50);
pub const ADC1_DCCMP5_R = @intToPtr(*volatile u32, 0x40039E54);
pub const ADC1_DCCMP6_R = @intToPtr(*volatile u32, 0x40039E58);
pub const ADC1_DCCMP7_R = @intToPtr(*volatile u32, 0x40039E5C);
pub const ADC1_PP_R = @intToPtr(*volatile u32, 0x40039FC0);
pub const ADC1_PC_R = @intToPtr(*volatile u32, 0x40039FC4);
pub const ADC1_CC_R = @intToPtr(*volatile u32, 0x40039FC8);

//*****************************************************************************
//
// Comparator registers (COMP)
//
//*****************************************************************************
pub const COMP_ACMIS_R = @intToPtr(*volatile u32, 0x4003C000);
pub const COMP_ACRIS_R = @intToPtr(*volatile u32, 0x4003C004);
pub const COMP_ACINTEN_R = @intToPtr(*volatile u32, 0x4003C008);
pub const COMP_ACREFCTL_R = @intToPtr(*volatile u32, 0x4003C010);
pub const COMP_ACSTAT0_R = @intToPtr(*volatile u32, 0x4003C020);
pub const COMP_ACCTL0_R = @intToPtr(*volatile u32, 0x4003C024);
pub const COMP_ACSTAT1_R = @intToPtr(*volatile u32, 0x4003C040);
pub const COMP_ACCTL1_R = @intToPtr(*volatile u32, 0x4003C044);
pub const COMP_ACSTAT2_R = @intToPtr(*volatile u32, 0x4003C060);
pub const COMP_ACCTL2_R = @intToPtr(*volatile u32, 0x4003C064);
pub const COMP_PP_R = @intToPtr(*volatile u32, 0x4003CFC0);

//*****************************************************************************
//
// CAN registers (CAN0)
//
//*****************************************************************************
pub const CAN0_CTL_R = @intToPtr(*volatile u32, 0x40040000);
pub const CAN0_STS_R = @intToPtr(*volatile u32, 0x40040004);
pub const CAN0_ERR_R = @intToPtr(*volatile u32, 0x40040008);
pub const CAN0_BIT_R = @intToPtr(*volatile u32, 0x4004000C);
pub const CAN0_INT_R = @intToPtr(*volatile u32, 0x40040010);
pub const CAN0_TST_R = @intToPtr(*volatile u32, 0x40040014);
pub const CAN0_BRPE_R = @intToPtr(*volatile u32, 0x40040018);
pub const CAN0_IF1CRQ_R = @intToPtr(*volatile u32, 0x40040020);
pub const CAN0_IF1CMSK_R = @intToPtr(*volatile u32, 0x40040024);
pub const CAN0_IF1MSK1_R = @intToPtr(*volatile u32, 0x40040028);
pub const CAN0_IF1MSK2_R = @intToPtr(*volatile u32, 0x4004002C);
pub const CAN0_IF1ARB1_R = @intToPtr(*volatile u32, 0x40040030);
pub const CAN0_IF1ARB2_R = @intToPtr(*volatile u32, 0x40040034);
pub const CAN0_IF1MCTL_R = @intToPtr(*volatile u32, 0x40040038);
pub const CAN0_IF1DA1_R = @intToPtr(*volatile u32, 0x4004003C);
pub const CAN0_IF1DA2_R = @intToPtr(*volatile u32, 0x40040040);
pub const CAN0_IF1DB1_R = @intToPtr(*volatile u32, 0x40040044);
pub const CAN0_IF1DB2_R = @intToPtr(*volatile u32, 0x40040048);
pub const CAN0_IF2CRQ_R = @intToPtr(*volatile u32, 0x40040080);
pub const CAN0_IF2CMSK_R = @intToPtr(*volatile u32, 0x40040084);
pub const CAN0_IF2MSK1_R = @intToPtr(*volatile u32, 0x40040088);
pub const CAN0_IF2MSK2_R = @intToPtr(*volatile u32, 0x4004008C);
pub const CAN0_IF2ARB1_R = @intToPtr(*volatile u32, 0x40040090);
pub const CAN0_IF2ARB2_R = @intToPtr(*volatile u32, 0x40040094);
pub const CAN0_IF2MCTL_R = @intToPtr(*volatile u32, 0x40040098);
pub const CAN0_IF2DA1_R = @intToPtr(*volatile u32, 0x4004009C);
pub const CAN0_IF2DA2_R = @intToPtr(*volatile u32, 0x400400A0);
pub const CAN0_IF2DB1_R = @intToPtr(*volatile u32, 0x400400A4);
pub const CAN0_IF2DB2_R = @intToPtr(*volatile u32, 0x400400A8);
pub const CAN0_TXRQ1_R = @intToPtr(*volatile u32, 0x40040100);
pub const CAN0_TXRQ2_R = @intToPtr(*volatile u32, 0x40040104);
pub const CAN0_NWDA1_R = @intToPtr(*volatile u32, 0x40040120);
pub const CAN0_NWDA2_R = @intToPtr(*volatile u32, 0x40040124);
pub const CAN0_MSG1INT_R = @intToPtr(*volatile u32, 0x40040140);
pub const CAN0_MSG2INT_R = @intToPtr(*volatile u32, 0x40040144);
pub const CAN0_MSG1VAL_R = @intToPtr(*volatile u32, 0x40040160);
pub const CAN0_MSG2VAL_R = @intToPtr(*volatile u32, 0x40040164);

//*****************************************************************************
//
// CAN registers (CAN1)
//
//*****************************************************************************
pub const CAN1_CTL_R = @intToPtr(*volatile u32, 0x40041000);
pub const CAN1_STS_R = @intToPtr(*volatile u32, 0x40041004);
pub const CAN1_ERR_R = @intToPtr(*volatile u32, 0x40041008);
pub const CAN1_BIT_R = @intToPtr(*volatile u32, 0x4004100C);
pub const CAN1_INT_R = @intToPtr(*volatile u32, 0x40041010);
pub const CAN1_TST_R = @intToPtr(*volatile u32, 0x40041014);
pub const CAN1_BRPE_R = @intToPtr(*volatile u32, 0x40041018);
pub const CAN1_IF1CRQ_R = @intToPtr(*volatile u32, 0x40041020);
pub const CAN1_IF1CMSK_R = @intToPtr(*volatile u32, 0x40041024);
pub const CAN1_IF1MSK1_R = @intToPtr(*volatile u32, 0x40041028);
pub const CAN1_IF1MSK2_R = @intToPtr(*volatile u32, 0x4004102C);
pub const CAN1_IF1ARB1_R = @intToPtr(*volatile u32, 0x40041030);
pub const CAN1_IF1ARB2_R = @intToPtr(*volatile u32, 0x40041034);
pub const CAN1_IF1MCTL_R = @intToPtr(*volatile u32, 0x40041038);
pub const CAN1_IF1DA1_R = @intToPtr(*volatile u32, 0x4004103C);
pub const CAN1_IF1DA2_R = @intToPtr(*volatile u32, 0x40041040);
pub const CAN1_IF1DB1_R = @intToPtr(*volatile u32, 0x40041044);
pub const CAN1_IF1DB2_R = @intToPtr(*volatile u32, 0x40041048);
pub const CAN1_IF2CRQ_R = @intToPtr(*volatile u32, 0x40041080);
pub const CAN1_IF2CMSK_R = @intToPtr(*volatile u32, 0x40041084);
pub const CAN1_IF2MSK1_R = @intToPtr(*volatile u32, 0x40041088);
pub const CAN1_IF2MSK2_R = @intToPtr(*volatile u32, 0x4004108C);
pub const CAN1_IF2ARB1_R = @intToPtr(*volatile u32, 0x40041090);
pub const CAN1_IF2ARB2_R = @intToPtr(*volatile u32, 0x40041094);
pub const CAN1_IF2MCTL_R = @intToPtr(*volatile u32, 0x40041098);
pub const CAN1_IF2DA1_R = @intToPtr(*volatile u32, 0x4004109C);
pub const CAN1_IF2DA2_R = @intToPtr(*volatile u32, 0x400410A0);
pub const CAN1_IF2DB1_R = @intToPtr(*volatile u32, 0x400410A4);
pub const CAN1_IF2DB2_R = @intToPtr(*volatile u32, 0x400410A8);
pub const CAN1_TXRQ1_R = @intToPtr(*volatile u32, 0x40041100);
pub const CAN1_TXRQ2_R = @intToPtr(*volatile u32, 0x40041104);
pub const CAN1_NWDA1_R = @intToPtr(*volatile u32, 0x40041120);
pub const CAN1_NWDA2_R = @intToPtr(*volatile u32, 0x40041124);
pub const CAN1_MSG1INT_R = @intToPtr(*volatile u32, 0x40041140);
pub const CAN1_MSG2INT_R = @intToPtr(*volatile u32, 0x40041144);
pub const CAN1_MSG1VAL_R = @intToPtr(*volatile u32, 0x40041160);
pub const CAN1_MSG2VAL_R = @intToPtr(*volatile u32, 0x40041164);

//*****************************************************************************
//
// Univeral Serial Bus registers (USB0)
//
//*****************************************************************************
#define USB0_FADDR_R            (*((volatile uint8_t *)0x40050000))
#define USB0_POWER_R            (*((volatile uint8_t *)0x40050001))
#define USB0_TXIS_R             (*((volatile uint16_t *)0x40050002))
#define USB0_RXIS_R             (*((volatile uint16_t *)0x40050004))
#define USB0_TXIE_R             (*((volatile uint16_t *)0x40050006))
#define USB0_RXIE_R             (*((volatile uint16_t *)0x40050008))
#define USB0_IS_R               (*((volatile uint8_t *)0x4005000A))
#define USB0_IE_R               (*((volatile uint8_t *)0x4005000B))
#define USB0_FRAME_R            (*((volatile uint16_t *)0x4005000C))
#define USB0_EPIDX_R            (*((volatile uint8_t *)0x4005000E))
#define USB0_TEST_R             (*((volatile uint8_t *)0x4005000F))
pub const USB0_FIFO0_R = @intToPtr(*volatile u32, 0x40050020);
pub const USB0_FIFO1_R = @intToPtr(*volatile u32, 0x40050024);
pub const USB0_FIFO2_R = @intToPtr(*volatile u32, 0x40050028);
pub const USB0_FIFO3_R = @intToPtr(*volatile u32, 0x4005002C);
pub const USB0_FIFO4_R = @intToPtr(*volatile u32, 0x40050030);
pub const USB0_FIFO5_R = @intToPtr(*volatile u32, 0x40050034);
pub const USB0_FIFO6_R = @intToPtr(*volatile u32, 0x40050038);
pub const USB0_FIFO7_R = @intToPtr(*volatile u32, 0x4005003C);
#define USB0_DEVCTL_R           (*((volatile uint8_t *)0x40050060))
#define USB0_CCONF_R            (*((volatile uint8_t *)0x40050061))
#define USB0_TXFIFOSZ_R         (*((volatile uint8_t *)0x40050062))
#define USB0_RXFIFOSZ_R         (*((volatile uint8_t *)0x40050063))
#define USB0_TXFIFOADD_R        (*((volatile uint16_t *)0x40050064))
#define USB0_RXFIFOADD_R        (*((volatile uint16_t *)0x40050066))
#define USB0_ULPIVBUSCTL_R      (*((volatile uint8_t *)0x40050070))
#define USB0_ULPIREGDATA_R      (*((volatile uint8_t *)0x40050074))
#define USB0_ULPIREGADDR_R      (*((volatile uint8_t *)0x40050075))
#define USB0_ULPIREGCTL_R       (*((volatile uint8_t *)0x40050076))
#define USB0_EPINFO_R           (*((volatile uint8_t *)0x40050078))
#define USB0_RAMINFO_R          (*((volatile uint8_t *)0x40050079))
#define USB0_CONTIM_R           (*((volatile uint8_t *)0x4005007A))
#define USB0_VPLEN_R            (*((volatile uint8_t *)0x4005007B))
#define USB0_HSEOF_R            (*((volatile uint8_t *)0x4005007C))
#define USB0_FSEOF_R            (*((volatile uint8_t *)0x4005007D))
#define USB0_LSEOF_R            (*((volatile uint8_t *)0x4005007E))
#define USB0_TXFUNCADDR0_R      (*((volatile uint8_t *)0x40050080))
#define USB0_TXHUBADDR0_R       (*((volatile uint8_t *)0x40050082))
#define USB0_TXHUBPORT0_R       (*((volatile uint8_t *)0x40050083))
#define USB0_TXFUNCADDR1_R      (*((volatile uint8_t *)0x40050088))
#define USB0_TXHUBADDR1_R       (*((volatile uint8_t *)0x4005008A))
#define USB0_TXHUBPORT1_R       (*((volatile uint8_t *)0x4005008B))
#define USB0_RXFUNCADDR1_R      (*((volatile uint8_t *)0x4005008C))
#define USB0_RXHUBADDR1_R       (*((volatile uint8_t *)0x4005008E))
#define USB0_RXHUBPORT1_R       (*((volatile uint8_t *)0x4005008F))
#define USB0_TXFUNCADDR2_R      (*((volatile uint8_t *)0x40050090))
#define USB0_TXHUBADDR2_R       (*((volatile uint8_t *)0x40050092))
#define USB0_TXHUBPORT2_R       (*((volatile uint8_t *)0x40050093))
#define USB0_RXFUNCADDR2_R      (*((volatile uint8_t *)0x40050094))
#define USB0_RXHUBADDR2_R       (*((volatile uint8_t *)0x40050096))
#define USB0_RXHUBPORT2_R       (*((volatile uint8_t *)0x40050097))
#define USB0_TXFUNCADDR3_R      (*((volatile uint8_t *)0x40050098))
#define USB0_TXHUBADDR3_R       (*((volatile uint8_t *)0x4005009A))
#define USB0_TXHUBPORT3_R       (*((volatile uint8_t *)0x4005009B))
#define USB0_RXFUNCADDR3_R      (*((volatile uint8_t *)0x4005009C))
#define USB0_RXHUBADDR3_R       (*((volatile uint8_t *)0x4005009E))
#define USB0_RXHUBPORT3_R       (*((volatile uint8_t *)0x4005009F))
#define USB0_TXFUNCADDR4_R      (*((volatile uint8_t *)0x400500A0))
#define USB0_TXHUBADDR4_R       (*((volatile uint8_t *)0x400500A2))
#define USB0_TXHUBPORT4_R       (*((volatile uint8_t *)0x400500A3))
#define USB0_RXFUNCADDR4_R      (*((volatile uint8_t *)0x400500A4))
#define USB0_RXHUBADDR4_R       (*((volatile uint8_t *)0x400500A6))
#define USB0_RXHUBPORT4_R       (*((volatile uint8_t *)0x400500A7))
#define USB0_TXFUNCADDR5_R      (*((volatile uint8_t *)0x400500A8))
#define USB0_TXHUBADDR5_R       (*((volatile uint8_t *)0x400500AA))
#define USB0_TXHUBPORT5_R       (*((volatile uint8_t *)0x400500AB))
#define USB0_RXFUNCADDR5_R      (*((volatile uint8_t *)0x400500AC))
#define USB0_RXHUBADDR5_R       (*((volatile uint8_t *)0x400500AE))
#define USB0_RXHUBPORT5_R       (*((volatile uint8_t *)0x400500AF))
#define USB0_TXFUNCADDR6_R      (*((volatile uint8_t *)0x400500B0))
#define USB0_TXHUBADDR6_R       (*((volatile uint8_t *)0x400500B2))
#define USB0_TXHUBPORT6_R       (*((volatile uint8_t *)0x400500B3))
#define USB0_RXFUNCADDR6_R      (*((volatile uint8_t *)0x400500B4))
#define USB0_RXHUBADDR6_R       (*((volatile uint8_t *)0x400500B6))
#define USB0_RXHUBPORT6_R       (*((volatile uint8_t *)0x400500B7))
#define USB0_TXFUNCADDR7_R      (*((volatile uint8_t *)0x400500B8))
#define USB0_TXHUBADDR7_R       (*((volatile uint8_t *)0x400500BA))
#define USB0_TXHUBPORT7_R       (*((volatile uint8_t *)0x400500BB))
#define USB0_RXFUNCADDR7_R      (*((volatile uint8_t *)0x400500BC))
#define USB0_RXHUBADDR7_R       (*((volatile uint8_t *)0x400500BE))
#define USB0_RXHUBPORT7_R       (*((volatile uint8_t *)0x400500BF))
#define USB0_CSRL0_R            (*((volatile uint8_t *)0x40050102))
#define USB0_CSRH0_R            (*((volatile uint8_t *)0x40050103))
#define USB0_COUNT0_R           (*((volatile uint8_t *)0x40050108))
#define USB0_TYPE0_R            (*((volatile uint8_t *)0x4005010A))
#define USB0_NAKLMT_R           (*((volatile uint8_t *)0x4005010B))
#define USB0_TXMAXP1_R          (*((volatile uint16_t *)0x40050110))
#define USB0_TXCSRL1_R          (*((volatile uint8_t *)0x40050112))
#define USB0_TXCSRH1_R          (*((volatile uint8_t *)0x40050113))
#define USB0_RXMAXP1_R          (*((volatile uint16_t *)0x40050114))
#define USB0_RXCSRL1_R          (*((volatile uint8_t *)0x40050116))
#define USB0_RXCSRH1_R          (*((volatile uint8_t *)0x40050117))
#define USB0_RXCOUNT1_R         (*((volatile uint16_t *)0x40050118))
#define USB0_TXTYPE1_R          (*((volatile uint8_t *)0x4005011A))
#define USB0_TXINTERVAL1_R      (*((volatile uint8_t *)0x4005011B))
#define USB0_RXTYPE1_R          (*((volatile uint8_t *)0x4005011C))
#define USB0_RXINTERVAL1_R      (*((volatile uint8_t *)0x4005011D))
#define USB0_TXMAXP2_R          (*((volatile uint16_t *)0x40050120))
#define USB0_TXCSRL2_R          (*((volatile uint8_t *)0x40050122))
#define USB0_TXCSRH2_R          (*((volatile uint8_t *)0x40050123))
#define USB0_RXMAXP2_R          (*((volatile uint16_t *)0x40050124))
#define USB0_RXCSRL2_R          (*((volatile uint8_t *)0x40050126))
#define USB0_RXCSRH2_R          (*((volatile uint8_t *)0x40050127))
#define USB0_RXCOUNT2_R         (*((volatile uint16_t *)0x40050128))
#define USB0_TXTYPE2_R          (*((volatile uint8_t *)0x4005012A))
#define USB0_TXINTERVAL2_R      (*((volatile uint8_t *)0x4005012B))
#define USB0_RXTYPE2_R          (*((volatile uint8_t *)0x4005012C))
#define USB0_RXINTERVAL2_R      (*((volatile uint8_t *)0x4005012D))
#define USB0_TXMAXP3_R          (*((volatile uint16_t *)0x40050130))
#define USB0_TXCSRL3_R          (*((volatile uint8_t *)0x40050132))
#define USB0_TXCSRH3_R          (*((volatile uint8_t *)0x40050133))
#define USB0_RXMAXP3_R          (*((volatile uint16_t *)0x40050134))
#define USB0_RXCSRL3_R          (*((volatile uint8_t *)0x40050136))
#define USB0_RXCSRH3_R          (*((volatile uint8_t *)0x40050137))
#define USB0_RXCOUNT3_R         (*((volatile uint16_t *)0x40050138))
#define USB0_TXTYPE3_R          (*((volatile uint8_t *)0x4005013A))
#define USB0_TXINTERVAL3_R      (*((volatile uint8_t *)0x4005013B))
#define USB0_RXTYPE3_R          (*((volatile uint8_t *)0x4005013C))
#define USB0_RXINTERVAL3_R      (*((volatile uint8_t *)0x4005013D))
#define USB0_TXMAXP4_R          (*((volatile uint16_t *)0x40050140))
#define USB0_TXCSRL4_R          (*((volatile uint8_t *)0x40050142))
#define USB0_TXCSRH4_R          (*((volatile uint8_t *)0x40050143))
#define USB0_RXMAXP4_R          (*((volatile uint16_t *)0x40050144))
#define USB0_RXCSRL4_R          (*((volatile uint8_t *)0x40050146))
#define USB0_RXCSRH4_R          (*((volatile uint8_t *)0x40050147))
#define USB0_RXCOUNT4_R         (*((volatile uint16_t *)0x40050148))
#define USB0_TXTYPE4_R          (*((volatile uint8_t *)0x4005014A))
#define USB0_TXINTERVAL4_R      (*((volatile uint8_t *)0x4005014B))
#define USB0_RXTYPE4_R          (*((volatile uint8_t *)0x4005014C))
#define USB0_RXINTERVAL4_R      (*((volatile uint8_t *)0x4005014D))
#define USB0_TXMAXP5_R          (*((volatile uint16_t *)0x40050150))
#define USB0_TXCSRL5_R          (*((volatile uint8_t *)0x40050152))
#define USB0_TXCSRH5_R          (*((volatile uint8_t *)0x40050153))
#define USB0_RXMAXP5_R          (*((volatile uint16_t *)0x40050154))
#define USB0_RXCSRL5_R          (*((volatile uint8_t *)0x40050156))
#define USB0_RXCSRH5_R          (*((volatile uint8_t *)0x40050157))
#define USB0_RXCOUNT5_R         (*((volatile uint16_t *)0x40050158))
#define USB0_TXTYPE5_R          (*((volatile uint8_t *)0x4005015A))
#define USB0_TXINTERVAL5_R      (*((volatile uint8_t *)0x4005015B))
#define USB0_RXTYPE5_R          (*((volatile uint8_t *)0x4005015C))
#define USB0_RXINTERVAL5_R      (*((volatile uint8_t *)0x4005015D))
#define USB0_TXMAXP6_R          (*((volatile uint16_t *)0x40050160))
#define USB0_TXCSRL6_R          (*((volatile uint8_t *)0x40050162))
#define USB0_TXCSRH6_R          (*((volatile uint8_t *)0x40050163))
#define USB0_RXMAXP6_R          (*((volatile uint16_t *)0x40050164))
#define USB0_RXCSRL6_R          (*((volatile uint8_t *)0x40050166))
#define USB0_RXCSRH6_R          (*((volatile uint8_t *)0x40050167))
#define USB0_RXCOUNT6_R         (*((volatile uint16_t *)0x40050168))
#define USB0_TXTYPE6_R          (*((volatile uint8_t *)0x4005016A))
#define USB0_TXINTERVAL6_R      (*((volatile uint8_t *)0x4005016B))
#define USB0_RXTYPE6_R          (*((volatile uint8_t *)0x4005016C))
#define USB0_RXINTERVAL6_R      (*((volatile uint8_t *)0x4005016D))
#define USB0_TXMAXP7_R          (*((volatile uint16_t *)0x40050170))
#define USB0_TXCSRL7_R          (*((volatile uint8_t *)0x40050172))
#define USB0_TXCSRH7_R          (*((volatile uint8_t *)0x40050173))
#define USB0_RXMAXP7_R          (*((volatile uint16_t *)0x40050174))
#define USB0_RXCSRL7_R          (*((volatile uint8_t *)0x40050176))
#define USB0_RXCSRH7_R          (*((volatile uint8_t *)0x40050177))
#define USB0_RXCOUNT7_R         (*((volatile uint16_t *)0x40050178))
#define USB0_TXTYPE7_R          (*((volatile uint8_t *)0x4005017A))
#define USB0_TXINTERVAL7_R      (*((volatile uint8_t *)0x4005017B))
#define USB0_RXTYPE7_R          (*((volatile uint8_t *)0x4005017C))
#define USB0_RXINTERVAL7_R      (*((volatile uint8_t *)0x4005017D))
#define USB0_DMAINTR_R          (*((volatile uint8_t *)0x40050200))
#define USB0_DMACTL0_R          (*((volatile uint16_t *)0x40050204))
pub const USB0_DMAADDR0_R = @intToPtr(*volatile u32, 0x40050208);
pub const USB0_DMACOUNT0_R = @intToPtr(*volatile u32, 0x4005020C);
#define USB0_DMACTL1_R          (*((volatile uint16_t *)0x40050214))
pub const USB0_DMAADDR1_R = @intToPtr(*volatile u32, 0x40050218);
pub const USB0_DMACOUNT1_R = @intToPtr(*volatile u32, 0x4005021C);
#define USB0_DMACTL2_R          (*((volatile uint16_t *)0x40050224))
pub const USB0_DMAADDR2_R = @intToPtr(*volatile u32, 0x40050228);
pub const USB0_DMACOUNT2_R = @intToPtr(*volatile u32, 0x4005022C);
#define USB0_DMACTL3_R          (*((volatile uint16_t *)0x40050234))
pub const USB0_DMAADDR3_R = @intToPtr(*volatile u32, 0x40050238);
pub const USB0_DMACOUNT3_R = @intToPtr(*volatile u32, 0x4005023C);
#define USB0_DMACTL4_R          (*((volatile uint16_t *)0x40050244))
pub const USB0_DMAADDR4_R = @intToPtr(*volatile u32, 0x40050248);
pub const USB0_DMACOUNT4_R = @intToPtr(*volatile u32, 0x4005024C);
#define USB0_DMACTL5_R          (*((volatile uint16_t *)0x40050254))
pub const USB0_DMAADDR5_R = @intToPtr(*volatile u32, 0x40050258);
pub const USB0_DMACOUNT5_R = @intToPtr(*volatile u32, 0x4005025C);
#define USB0_DMACTL6_R          (*((volatile uint16_t *)0x40050264))
pub const USB0_DMAADDR6_R = @intToPtr(*volatile u32, 0x40050268);
pub const USB0_DMACOUNT6_R = @intToPtr(*volatile u32, 0x4005026C);
#define USB0_DMACTL7_R          (*((volatile uint16_t *)0x40050274))
pub const USB0_DMAADDR7_R = @intToPtr(*volatile u32, 0x40050278);
pub const USB0_DMACOUNT7_R = @intToPtr(*volatile u32, 0x4005027C);
#define USB0_RQPKTCOUNT1_R      (*((volatile uint16_t *)0x40050304))
#define USB0_RQPKTCOUNT2_R      (*((volatile uint16_t *)0x40050308))
#define USB0_RQPKTCOUNT3_R      (*((volatile uint16_t *)0x4005030C))
#define USB0_RQPKTCOUNT4_R      (*((volatile uint16_t *)0x40050310))
#define USB0_RQPKTCOUNT5_R      (*((volatile uint16_t *)0x40050314))
#define USB0_RQPKTCOUNT6_R      (*((volatile uint16_t *)0x40050318))
#define USB0_RQPKTCOUNT7_R      (*((volatile uint16_t *)0x4005031C))
#define USB0_RXDPKTBUFDIS_R     (*((volatile uint16_t *)0x40050340))
#define USB0_TXDPKTBUFDIS_R     (*((volatile uint16_t *)0x40050342))
#define USB0_CTO_R              (*((volatile uint16_t *)0x40050344))
#define USB0_HHSRTN_R           (*((volatile uint16_t *)0x40050346))
#define USB0_HSBT_R             (*((volatile uint16_t *)0x40050348))
#define USB0_LPMATTR_R          (*((volatile uint16_t *)0x40050360))
#define USB0_LPMCNTRL_R         (*((volatile uint8_t *)0x40050362))
#define USB0_LPMIM_R            (*((volatile uint8_t *)0x40050363))
#define USB0_LPMRIS_R           (*((volatile uint8_t *)0x40050364))
#define USB0_LPMFADDR_R         (*((volatile uint8_t *)0x40050365))
pub const USB0_EPC_R = @intToPtr(*volatile u32, 0x40050400);
pub const USB0_EPCRIS_R = @intToPtr(*volatile u32, 0x40050404);
pub const USB0_EPCIM_R = @intToPtr(*volatile u32, 0x40050408);
pub const USB0_EPCISC_R = @intToPtr(*volatile u32, 0x4005040C);
pub const USB0_DRRIS_R = @intToPtr(*volatile u32, 0x40050410);
pub const USB0_DRIM_R = @intToPtr(*volatile u32, 0x40050414);
pub const USB0_DRISC_R = @intToPtr(*volatile u32, 0x40050418);
pub const USB0_GPCS_R = @intToPtr(*volatile u32, 0x4005041C);
pub const USB0_VDC_R = @intToPtr(*volatile u32, 0x40050430);
pub const USB0_VDCRIS_R = @intToPtr(*volatile u32, 0x40050434);
pub const USB0_VDCIM_R = @intToPtr(*volatile u32, 0x40050438);
pub const USB0_VDCISC_R = @intToPtr(*volatile u32, 0x4005043C);
pub const USB0_PP_R = @intToPtr(*volatile u32, 0x40050FC0);
pub const USB0_PC_R = @intToPtr(*volatile u32, 0x40050FC4);
pub const USB0_CC_R = @intToPtr(*volatile u32, 0x40050FC8);

//*****************************************************************************
//
// GPIO registers (PORTA AHB)
//
//*****************************************************************************
pub const GPIO_PORTA_AHB_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40058000);
pub const GPIO_PORTA_AHB_DATA_R = @intToPtr(*volatile u32, 0x400583FC);
pub const GPIO_PORTA_AHB_DIR_R = @intToPtr(*volatile u32, 0x40058400);
pub const GPIO_PORTA_AHB_IS_R = @intToPtr(*volatile u32, 0x40058404);
pub const GPIO_PORTA_AHB_IBE_R = @intToPtr(*volatile u32, 0x40058408);
pub const GPIO_PORTA_AHB_IEV_R = @intToPtr(*volatile u32, 0x4005840C);
pub const GPIO_PORTA_AHB_IM_R = @intToPtr(*volatile u32, 0x40058410);
pub const GPIO_PORTA_AHB_RIS_R = @intToPtr(*volatile u32, 0x40058414);
pub const GPIO_PORTA_AHB_MIS_R = @intToPtr(*volatile u32, 0x40058418);
pub const GPIO_PORTA_AHB_ICR_R = @intToPtr(*volatile u32, 0x4005841C);
pub const GPIO_PORTA_AHB_AFSEL_R = @intToPtr(*volatile u32, 0x40058420);
pub const GPIO_PORTA_AHB_DR2R_R = @intToPtr(*volatile u32, 0x40058500);
pub const GPIO_PORTA_AHB_DR4R_R = @intToPtr(*volatile u32, 0x40058504);
pub const GPIO_PORTA_AHB_DR8R_R = @intToPtr(*volatile u32, 0x40058508);
pub const GPIO_PORTA_AHB_ODR_R = @intToPtr(*volatile u32, 0x4005850C);
pub const GPIO_PORTA_AHB_PUR_R = @intToPtr(*volatile u32, 0x40058510);
pub const GPIO_PORTA_AHB_PDR_R = @intToPtr(*volatile u32, 0x40058514);
pub const GPIO_PORTA_AHB_SLR_R = @intToPtr(*volatile u32, 0x40058518);
pub const GPIO_PORTA_AHB_DEN_R = @intToPtr(*volatile u32, 0x4005851C);
pub const GPIO_PORTA_AHB_LOCK_R = @intToPtr(*volatile u32, 0x40058520);
pub const GPIO_PORTA_AHB_CR_R = @intToPtr(*volatile u32, 0x40058524);
pub const GPIO_PORTA_AHB_AMSEL_R = @intToPtr(*volatile u32, 0x40058528);
pub const GPIO_PORTA_AHB_PCTL_R = @intToPtr(*volatile u32, 0x4005852C);
pub const GPIO_PORTA_AHB_ADCCTL_R = @intToPtr(*volatile u32, 0x40058530);
pub const GPIO_PORTA_AHB_DMACTL_R = @intToPtr(*volatile u32, 0x40058534);
pub const GPIO_PORTA_AHB_SI_R = @intToPtr(*volatile u32, 0x40058538);
pub const GPIO_PORTA_AHB_DR12R_R = @intToPtr(*volatile u32, 0x4005853C);
pub const GPIO_PORTA_AHB_WAKEPEN_R = @intToPtr(*volatile u32, 0x40058540);
pub const GPIO_PORTA_AHB_WAKELVL_R = @intToPtr(*volatile u32, 0x40058544);
pub const GPIO_PORTA_AHB_WAKESTAT_R = @intToPtr(*volatile u32, 0x40058548);
pub const GPIO_PORTA_AHB_PP_R = @intToPtr(*volatile u32, 0x40058FC0);
pub const GPIO_PORTA_AHB_PC_R = @intToPtr(*volatile u32, 0x40058FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTB AHB)
//
//*****************************************************************************
pub const GPIO_PORTB_AHB_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40059000);
pub const GPIO_PORTB_AHB_DATA_R = @intToPtr(*volatile u32, 0x400593FC);
pub const GPIO_PORTB_AHB_DIR_R = @intToPtr(*volatile u32, 0x40059400);
pub const GPIO_PORTB_AHB_IS_R = @intToPtr(*volatile u32, 0x40059404);
pub const GPIO_PORTB_AHB_IBE_R = @intToPtr(*volatile u32, 0x40059408);
pub const GPIO_PORTB_AHB_IEV_R = @intToPtr(*volatile u32, 0x4005940C);
pub const GPIO_PORTB_AHB_IM_R = @intToPtr(*volatile u32, 0x40059410);
pub const GPIO_PORTB_AHB_RIS_R = @intToPtr(*volatile u32, 0x40059414);
pub const GPIO_PORTB_AHB_MIS_R = @intToPtr(*volatile u32, 0x40059418);
pub const GPIO_PORTB_AHB_ICR_R = @intToPtr(*volatile u32, 0x4005941C);
pub const GPIO_PORTB_AHB_AFSEL_R = @intToPtr(*volatile u32, 0x40059420);
pub const GPIO_PORTB_AHB_DR2R_R = @intToPtr(*volatile u32, 0x40059500);
pub const GPIO_PORTB_AHB_DR4R_R = @intToPtr(*volatile u32, 0x40059504);
pub const GPIO_PORTB_AHB_DR8R_R = @intToPtr(*volatile u32, 0x40059508);
pub const GPIO_PORTB_AHB_ODR_R = @intToPtr(*volatile u32, 0x4005950C);
pub const GPIO_PORTB_AHB_PUR_R = @intToPtr(*volatile u32, 0x40059510);
pub const GPIO_PORTB_AHB_PDR_R = @intToPtr(*volatile u32, 0x40059514);
pub const GPIO_PORTB_AHB_SLR_R = @intToPtr(*volatile u32, 0x40059518);
pub const GPIO_PORTB_AHB_DEN_R = @intToPtr(*volatile u32, 0x4005951C);
pub const GPIO_PORTB_AHB_LOCK_R = @intToPtr(*volatile u32, 0x40059520);
pub const GPIO_PORTB_AHB_CR_R = @intToPtr(*volatile u32, 0x40059524);
pub const GPIO_PORTB_AHB_AMSEL_R = @intToPtr(*volatile u32, 0x40059528);
pub const GPIO_PORTB_AHB_PCTL_R = @intToPtr(*volatile u32, 0x4005952C);
pub const GPIO_PORTB_AHB_ADCCTL_R = @intToPtr(*volatile u32, 0x40059530);
pub const GPIO_PORTB_AHB_DMACTL_R = @intToPtr(*volatile u32, 0x40059534);
pub const GPIO_PORTB_AHB_SI_R = @intToPtr(*volatile u32, 0x40059538);
pub const GPIO_PORTB_AHB_DR12R_R = @intToPtr(*volatile u32, 0x4005953C);
pub const GPIO_PORTB_AHB_WAKEPEN_R = @intToPtr(*volatile u32, 0x40059540);
pub const GPIO_PORTB_AHB_WAKELVL_R = @intToPtr(*volatile u32, 0x40059544);
pub const GPIO_PORTB_AHB_WAKESTAT_R = @intToPtr(*volatile u32, 0x40059548);
pub const GPIO_PORTB_AHB_PP_R = @intToPtr(*volatile u32, 0x40059FC0);
pub const GPIO_PORTB_AHB_PC_R = @intToPtr(*volatile u32, 0x40059FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTC AHB)
//
//*****************************************************************************
pub const GPIO_PORTC_AHB_DATA_BITS_R = @intToPtr([*]volatile u32, 0x4005A000);
pub const GPIO_PORTC_AHB_DATA_R = @intToPtr(*volatile u32, 0x4005A3FC);
pub const GPIO_PORTC_AHB_DIR_R = @intToPtr(*volatile u32, 0x4005A400);
pub const GPIO_PORTC_AHB_IS_R = @intToPtr(*volatile u32, 0x4005A404);
pub const GPIO_PORTC_AHB_IBE_R = @intToPtr(*volatile u32, 0x4005A408);
pub const GPIO_PORTC_AHB_IEV_R = @intToPtr(*volatile u32, 0x4005A40C);
pub const GPIO_PORTC_AHB_IM_R = @intToPtr(*volatile u32, 0x4005A410);
pub const GPIO_PORTC_AHB_RIS_R = @intToPtr(*volatile u32, 0x4005A414);
pub const GPIO_PORTC_AHB_MIS_R = @intToPtr(*volatile u32, 0x4005A418);
pub const GPIO_PORTC_AHB_ICR_R = @intToPtr(*volatile u32, 0x4005A41C);
pub const GPIO_PORTC_AHB_AFSEL_R = @intToPtr(*volatile u32, 0x4005A420);
pub const GPIO_PORTC_AHB_DR2R_R = @intToPtr(*volatile u32, 0x4005A500);
pub const GPIO_PORTC_AHB_DR4R_R = @intToPtr(*volatile u32, 0x4005A504);
pub const GPIO_PORTC_AHB_DR8R_R = @intToPtr(*volatile u32, 0x4005A508);
pub const GPIO_PORTC_AHB_ODR_R = @intToPtr(*volatile u32, 0x4005A50C);
pub const GPIO_PORTC_AHB_PUR_R = @intToPtr(*volatile u32, 0x4005A510);
pub const GPIO_PORTC_AHB_PDR_R = @intToPtr(*volatile u32, 0x4005A514);
pub const GPIO_PORTC_AHB_SLR_R = @intToPtr(*volatile u32, 0x4005A518);
pub const GPIO_PORTC_AHB_DEN_R = @intToPtr(*volatile u32, 0x4005A51C);
pub const GPIO_PORTC_AHB_LOCK_R = @intToPtr(*volatile u32, 0x4005A520);
pub const GPIO_PORTC_AHB_CR_R = @intToPtr(*volatile u32, 0x4005A524);
pub const GPIO_PORTC_AHB_AMSEL_R = @intToPtr(*volatile u32, 0x4005A528);
pub const GPIO_PORTC_AHB_PCTL_R = @intToPtr(*volatile u32, 0x4005A52C);
pub const GPIO_PORTC_AHB_ADCCTL_R = @intToPtr(*volatile u32, 0x4005A530);
pub const GPIO_PORTC_AHB_DMACTL_R = @intToPtr(*volatile u32, 0x4005A534);
pub const GPIO_PORTC_AHB_SI_R = @intToPtr(*volatile u32, 0x4005A538);
pub const GPIO_PORTC_AHB_DR12R_R = @intToPtr(*volatile u32, 0x4005A53C);
pub const GPIO_PORTC_AHB_WAKEPEN_R = @intToPtr(*volatile u32, 0x4005A540);
pub const GPIO_PORTC_AHB_WAKELVL_R = @intToPtr(*volatile u32, 0x4005A544);
pub const GPIO_PORTC_AHB_WAKESTAT_R = @intToPtr(*volatile u32, 0x4005A548);
pub const GPIO_PORTC_AHB_PP_R = @intToPtr(*volatile u32, 0x4005AFC0);
pub const GPIO_PORTC_AHB_PC_R = @intToPtr(*volatile u32, 0x4005AFC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTD AHB)
//
//*****************************************************************************
pub const GPIO_PORTD_AHB_DATA_BITS_R = @intToPtr([*]volatile u32, 0x4005B000);
pub const GPIO_PORTD_AHB_DATA_R = @intToPtr(*volatile u32, 0x4005B3FC);
pub const GPIO_PORTD_AHB_DIR_R = @intToPtr(*volatile u32, 0x4005B400);
pub const GPIO_PORTD_AHB_IS_R = @intToPtr(*volatile u32, 0x4005B404);
pub const GPIO_PORTD_AHB_IBE_R = @intToPtr(*volatile u32, 0x4005B408);
pub const GPIO_PORTD_AHB_IEV_R = @intToPtr(*volatile u32, 0x4005B40C);
pub const GPIO_PORTD_AHB_IM_R = @intToPtr(*volatile u32, 0x4005B410);
pub const GPIO_PORTD_AHB_RIS_R = @intToPtr(*volatile u32, 0x4005B414);
pub const GPIO_PORTD_AHB_MIS_R = @intToPtr(*volatile u32, 0x4005B418);
pub const GPIO_PORTD_AHB_ICR_R = @intToPtr(*volatile u32, 0x4005B41C);
pub const GPIO_PORTD_AHB_AFSEL_R = @intToPtr(*volatile u32, 0x4005B420);
pub const GPIO_PORTD_AHB_DR2R_R = @intToPtr(*volatile u32, 0x4005B500);
pub const GPIO_PORTD_AHB_DR4R_R = @intToPtr(*volatile u32, 0x4005B504);
pub const GPIO_PORTD_AHB_DR8R_R = @intToPtr(*volatile u32, 0x4005B508);
pub const GPIO_PORTD_AHB_ODR_R = @intToPtr(*volatile u32, 0x4005B50C);
pub const GPIO_PORTD_AHB_PUR_R = @intToPtr(*volatile u32, 0x4005B510);
pub const GPIO_PORTD_AHB_PDR_R = @intToPtr(*volatile u32, 0x4005B514);
pub const GPIO_PORTD_AHB_SLR_R = @intToPtr(*volatile u32, 0x4005B518);
pub const GPIO_PORTD_AHB_DEN_R = @intToPtr(*volatile u32, 0x4005B51C);
pub const GPIO_PORTD_AHB_LOCK_R = @intToPtr(*volatile u32, 0x4005B520);
pub const GPIO_PORTD_AHB_CR_R = @intToPtr(*volatile u32, 0x4005B524);
pub const GPIO_PORTD_AHB_AMSEL_R = @intToPtr(*volatile u32, 0x4005B528);
pub const GPIO_PORTD_AHB_PCTL_R = @intToPtr(*volatile u32, 0x4005B52C);
pub const GPIO_PORTD_AHB_ADCCTL_R = @intToPtr(*volatile u32, 0x4005B530);
pub const GPIO_PORTD_AHB_DMACTL_R = @intToPtr(*volatile u32, 0x4005B534);
pub const GPIO_PORTD_AHB_SI_R = @intToPtr(*volatile u32, 0x4005B538);
pub const GPIO_PORTD_AHB_DR12R_R = @intToPtr(*volatile u32, 0x4005B53C);
pub const GPIO_PORTD_AHB_WAKEPEN_R = @intToPtr(*volatile u32, 0x4005B540);
pub const GPIO_PORTD_AHB_WAKELVL_R = @intToPtr(*volatile u32, 0x4005B544);
pub const GPIO_PORTD_AHB_WAKESTAT_R = @intToPtr(*volatile u32, 0x4005B548);
pub const GPIO_PORTD_AHB_PP_R = @intToPtr(*volatile u32, 0x4005BFC0);
pub const GPIO_PORTD_AHB_PC_R = @intToPtr(*volatile u32, 0x4005BFC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTE AHB)
//
//*****************************************************************************
pub const GPIO_PORTE_AHB_DATA_BITS_R = @intToPtr([*]volatile u32, 0x4005C000);
pub const GPIO_PORTE_AHB_DATA_R = @intToPtr(*volatile u32, 0x4005C3FC);
pub const GPIO_PORTE_AHB_DIR_R = @intToPtr(*volatile u32, 0x4005C400);
pub const GPIO_PORTE_AHB_IS_R = @intToPtr(*volatile u32, 0x4005C404);
pub const GPIO_PORTE_AHB_IBE_R = @intToPtr(*volatile u32, 0x4005C408);
pub const GPIO_PORTE_AHB_IEV_R = @intToPtr(*volatile u32, 0x4005C40C);
pub const GPIO_PORTE_AHB_IM_R = @intToPtr(*volatile u32, 0x4005C410);
pub const GPIO_PORTE_AHB_RIS_R = @intToPtr(*volatile u32, 0x4005C414);
pub const GPIO_PORTE_AHB_MIS_R = @intToPtr(*volatile u32, 0x4005C418);
pub const GPIO_PORTE_AHB_ICR_R = @intToPtr(*volatile u32, 0x4005C41C);
pub const GPIO_PORTE_AHB_AFSEL_R = @intToPtr(*volatile u32, 0x4005C420);
pub const GPIO_PORTE_AHB_DR2R_R = @intToPtr(*volatile u32, 0x4005C500);
pub const GPIO_PORTE_AHB_DR4R_R = @intToPtr(*volatile u32, 0x4005C504);
pub const GPIO_PORTE_AHB_DR8R_R = @intToPtr(*volatile u32, 0x4005C508);
pub const GPIO_PORTE_AHB_ODR_R = @intToPtr(*volatile u32, 0x4005C50C);
pub const GPIO_PORTE_AHB_PUR_R = @intToPtr(*volatile u32, 0x4005C510);
pub const GPIO_PORTE_AHB_PDR_R = @intToPtr(*volatile u32, 0x4005C514);
pub const GPIO_PORTE_AHB_SLR_R = @intToPtr(*volatile u32, 0x4005C518);
pub const GPIO_PORTE_AHB_DEN_R = @intToPtr(*volatile u32, 0x4005C51C);
pub const GPIO_PORTE_AHB_LOCK_R = @intToPtr(*volatile u32, 0x4005C520);
pub const GPIO_PORTE_AHB_CR_R = @intToPtr(*volatile u32, 0x4005C524);
pub const GPIO_PORTE_AHB_AMSEL_R = @intToPtr(*volatile u32, 0x4005C528);
pub const GPIO_PORTE_AHB_PCTL_R = @intToPtr(*volatile u32, 0x4005C52C);
pub const GPIO_PORTE_AHB_ADCCTL_R = @intToPtr(*volatile u32, 0x4005C530);
pub const GPIO_PORTE_AHB_DMACTL_R = @intToPtr(*volatile u32, 0x4005C534);
pub const GPIO_PORTE_AHB_SI_R = @intToPtr(*volatile u32, 0x4005C538);
pub const GPIO_PORTE_AHB_DR12R_R = @intToPtr(*volatile u32, 0x4005C53C);
pub const GPIO_PORTE_AHB_WAKEPEN_R = @intToPtr(*volatile u32, 0x4005C540);
pub const GPIO_PORTE_AHB_WAKELVL_R = @intToPtr(*volatile u32, 0x4005C544);
pub const GPIO_PORTE_AHB_WAKESTAT_R = @intToPtr(*volatile u32, 0x4005C548);
pub const GPIO_PORTE_AHB_PP_R = @intToPtr(*volatile u32, 0x4005CFC0);
pub const GPIO_PORTE_AHB_PC_R = @intToPtr(*volatile u32, 0x4005CFC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTF AHB)
//
//*****************************************************************************
pub const GPIO_PORTF_AHB_DATA_BITS_R = @intToPtr([*]volatile u32, 0x4005D000);
pub const GPIO_PORTF_AHB_DATA_R = @intToPtr(*volatile u32, 0x4005D3FC);
pub const GPIO_PORTF_AHB_DIR_R = @intToPtr(*volatile u32, 0x4005D400);
pub const GPIO_PORTF_AHB_IS_R = @intToPtr(*volatile u32, 0x4005D404);
pub const GPIO_PORTF_AHB_IBE_R = @intToPtr(*volatile u32, 0x4005D408);
pub const GPIO_PORTF_AHB_IEV_R = @intToPtr(*volatile u32, 0x4005D40C);
pub const GPIO_PORTF_AHB_IM_R = @intToPtr(*volatile u32, 0x4005D410);
pub const GPIO_PORTF_AHB_RIS_R = @intToPtr(*volatile u32, 0x4005D414);
pub const GPIO_PORTF_AHB_MIS_R = @intToPtr(*volatile u32, 0x4005D418);
pub const GPIO_PORTF_AHB_ICR_R = @intToPtr(*volatile u32, 0x4005D41C);
pub const GPIO_PORTF_AHB_AFSEL_R = @intToPtr(*volatile u32, 0x4005D420);
pub const GPIO_PORTF_AHB_DR2R_R = @intToPtr(*volatile u32, 0x4005D500);
pub const GPIO_PORTF_AHB_DR4R_R = @intToPtr(*volatile u32, 0x4005D504);
pub const GPIO_PORTF_AHB_DR8R_R = @intToPtr(*volatile u32, 0x4005D508);
pub const GPIO_PORTF_AHB_ODR_R = @intToPtr(*volatile u32, 0x4005D50C);
pub const GPIO_PORTF_AHB_PUR_R = @intToPtr(*volatile u32, 0x4005D510);
pub const GPIO_PORTF_AHB_PDR_R = @intToPtr(*volatile u32, 0x4005D514);
pub const GPIO_PORTF_AHB_SLR_R = @intToPtr(*volatile u32, 0x4005D518);
pub const GPIO_PORTF_AHB_DEN_R = @intToPtr(*volatile u32, 0x4005D51C);
pub const GPIO_PORTF_AHB_LOCK_R = @intToPtr(*volatile u32, 0x4005D520);
pub const GPIO_PORTF_AHB_CR_R = @intToPtr(*volatile u32, 0x4005D524);
pub const GPIO_PORTF_AHB_AMSEL_R = @intToPtr(*volatile u32, 0x4005D528);
pub const GPIO_PORTF_AHB_PCTL_R = @intToPtr(*volatile u32, 0x4005D52C);
pub const GPIO_PORTF_AHB_ADCCTL_R = @intToPtr(*volatile u32, 0x4005D530);
pub const GPIO_PORTF_AHB_DMACTL_R = @intToPtr(*volatile u32, 0x4005D534);
pub const GPIO_PORTF_AHB_SI_R = @intToPtr(*volatile u32, 0x4005D538);
pub const GPIO_PORTF_AHB_DR12R_R = @intToPtr(*volatile u32, 0x4005D53C);
pub const GPIO_PORTF_AHB_WAKEPEN_R = @intToPtr(*volatile u32, 0x4005D540);
pub const GPIO_PORTF_AHB_WAKELVL_R = @intToPtr(*volatile u32, 0x4005D544);
pub const GPIO_PORTF_AHB_WAKESTAT_R = @intToPtr(*volatile u32, 0x4005D548);
pub const GPIO_PORTF_AHB_PP_R = @intToPtr(*volatile u32, 0x4005DFC0);
pub const GPIO_PORTF_AHB_PC_R = @intToPtr(*volatile u32, 0x4005DFC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTG AHB)
//
//*****************************************************************************
pub const GPIO_PORTG_AHB_DATA_BITS_R = @intToPtr([*]volatile u32, 0x4005E000);
pub const GPIO_PORTG_AHB_DATA_R = @intToPtr(*volatile u32, 0x4005E3FC);
pub const GPIO_PORTG_AHB_DIR_R = @intToPtr(*volatile u32, 0x4005E400);
pub const GPIO_PORTG_AHB_IS_R = @intToPtr(*volatile u32, 0x4005E404);
pub const GPIO_PORTG_AHB_IBE_R = @intToPtr(*volatile u32, 0x4005E408);
pub const GPIO_PORTG_AHB_IEV_R = @intToPtr(*volatile u32, 0x4005E40C);
pub const GPIO_PORTG_AHB_IM_R = @intToPtr(*volatile u32, 0x4005E410);
pub const GPIO_PORTG_AHB_RIS_R = @intToPtr(*volatile u32, 0x4005E414);
pub const GPIO_PORTG_AHB_MIS_R = @intToPtr(*volatile u32, 0x4005E418);
pub const GPIO_PORTG_AHB_ICR_R = @intToPtr(*volatile u32, 0x4005E41C);
pub const GPIO_PORTG_AHB_AFSEL_R = @intToPtr(*volatile u32, 0x4005E420);
pub const GPIO_PORTG_AHB_DR2R_R = @intToPtr(*volatile u32, 0x4005E500);
pub const GPIO_PORTG_AHB_DR4R_R = @intToPtr(*volatile u32, 0x4005E504);
pub const GPIO_PORTG_AHB_DR8R_R = @intToPtr(*volatile u32, 0x4005E508);
pub const GPIO_PORTG_AHB_ODR_R = @intToPtr(*volatile u32, 0x4005E50C);
pub const GPIO_PORTG_AHB_PUR_R = @intToPtr(*volatile u32, 0x4005E510);
pub const GPIO_PORTG_AHB_PDR_R = @intToPtr(*volatile u32, 0x4005E514);
pub const GPIO_PORTG_AHB_SLR_R = @intToPtr(*volatile u32, 0x4005E518);
pub const GPIO_PORTG_AHB_DEN_R = @intToPtr(*volatile u32, 0x4005E51C);
pub const GPIO_PORTG_AHB_LOCK_R = @intToPtr(*volatile u32, 0x4005E520);
pub const GPIO_PORTG_AHB_CR_R = @intToPtr(*volatile u32, 0x4005E524);
pub const GPIO_PORTG_AHB_AMSEL_R = @intToPtr(*volatile u32, 0x4005E528);
pub const GPIO_PORTG_AHB_PCTL_R = @intToPtr(*volatile u32, 0x4005E52C);
pub const GPIO_PORTG_AHB_ADCCTL_R = @intToPtr(*volatile u32, 0x4005E530);
pub const GPIO_PORTG_AHB_DMACTL_R = @intToPtr(*volatile u32, 0x4005E534);
pub const GPIO_PORTG_AHB_SI_R = @intToPtr(*volatile u32, 0x4005E538);
pub const GPIO_PORTG_AHB_DR12R_R = @intToPtr(*volatile u32, 0x4005E53C);
pub const GPIO_PORTG_AHB_WAKEPEN_R = @intToPtr(*volatile u32, 0x4005E540);
pub const GPIO_PORTG_AHB_WAKELVL_R = @intToPtr(*volatile u32, 0x4005E544);
pub const GPIO_PORTG_AHB_WAKESTAT_R = @intToPtr(*volatile u32, 0x4005E548);
pub const GPIO_PORTG_AHB_PP_R = @intToPtr(*volatile u32, 0x4005EFC0);
pub const GPIO_PORTG_AHB_PC_R = @intToPtr(*volatile u32, 0x4005EFC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTH AHB)
//
//*****************************************************************************
pub const GPIO_PORTH_AHB_DATA_BITS_R = @intToPtr([*]volatile u32, 0x4005F000);
pub const GPIO_PORTH_AHB_DATA_R = @intToPtr(*volatile u32, 0x4005F3FC);
pub const GPIO_PORTH_AHB_DIR_R = @intToPtr(*volatile u32, 0x4005F400);
pub const GPIO_PORTH_AHB_IS_R = @intToPtr(*volatile u32, 0x4005F404);
pub const GPIO_PORTH_AHB_IBE_R = @intToPtr(*volatile u32, 0x4005F408);
pub const GPIO_PORTH_AHB_IEV_R = @intToPtr(*volatile u32, 0x4005F40C);
pub const GPIO_PORTH_AHB_IM_R = @intToPtr(*volatile u32, 0x4005F410);
pub const GPIO_PORTH_AHB_RIS_R = @intToPtr(*volatile u32, 0x4005F414);
pub const GPIO_PORTH_AHB_MIS_R = @intToPtr(*volatile u32, 0x4005F418);
pub const GPIO_PORTH_AHB_ICR_R = @intToPtr(*volatile u32, 0x4005F41C);
pub const GPIO_PORTH_AHB_AFSEL_R = @intToPtr(*volatile u32, 0x4005F420);
pub const GPIO_PORTH_AHB_DR2R_R = @intToPtr(*volatile u32, 0x4005F500);
pub const GPIO_PORTH_AHB_DR4R_R = @intToPtr(*volatile u32, 0x4005F504);
pub const GPIO_PORTH_AHB_DR8R_R = @intToPtr(*volatile u32, 0x4005F508);
pub const GPIO_PORTH_AHB_ODR_R = @intToPtr(*volatile u32, 0x4005F50C);
pub const GPIO_PORTH_AHB_PUR_R = @intToPtr(*volatile u32, 0x4005F510);
pub const GPIO_PORTH_AHB_PDR_R = @intToPtr(*volatile u32, 0x4005F514);
pub const GPIO_PORTH_AHB_SLR_R = @intToPtr(*volatile u32, 0x4005F518);
pub const GPIO_PORTH_AHB_DEN_R = @intToPtr(*volatile u32, 0x4005F51C);
pub const GPIO_PORTH_AHB_LOCK_R = @intToPtr(*volatile u32, 0x4005F520);
pub const GPIO_PORTH_AHB_CR_R = @intToPtr(*volatile u32, 0x4005F524);
pub const GPIO_PORTH_AHB_AMSEL_R = @intToPtr(*volatile u32, 0x4005F528);
pub const GPIO_PORTH_AHB_PCTL_R = @intToPtr(*volatile u32, 0x4005F52C);
pub const GPIO_PORTH_AHB_ADCCTL_R = @intToPtr(*volatile u32, 0x4005F530);
pub const GPIO_PORTH_AHB_DMACTL_R = @intToPtr(*volatile u32, 0x4005F534);
pub const GPIO_PORTH_AHB_SI_R = @intToPtr(*volatile u32, 0x4005F538);
pub const GPIO_PORTH_AHB_DR12R_R = @intToPtr(*volatile u32, 0x4005F53C);
pub const GPIO_PORTH_AHB_WAKEPEN_R = @intToPtr(*volatile u32, 0x4005F540);
pub const GPIO_PORTH_AHB_WAKELVL_R = @intToPtr(*volatile u32, 0x4005F544);
pub const GPIO_PORTH_AHB_WAKESTAT_R = @intToPtr(*volatile u32, 0x4005F548);
pub const GPIO_PORTH_AHB_PP_R = @intToPtr(*volatile u32, 0x4005FFC0);
pub const GPIO_PORTH_AHB_PC_R = @intToPtr(*volatile u32, 0x4005FFC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTJ AHB)
//
//*****************************************************************************
pub const GPIO_PORTJ_AHB_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40060000);
pub const GPIO_PORTJ_AHB_DATA_R = @intToPtr(*volatile u32, 0x400603FC);
pub const GPIO_PORTJ_AHB_DIR_R = @intToPtr(*volatile u32, 0x40060400);
pub const GPIO_PORTJ_AHB_IS_R = @intToPtr(*volatile u32, 0x40060404);
pub const GPIO_PORTJ_AHB_IBE_R = @intToPtr(*volatile u32, 0x40060408);
pub const GPIO_PORTJ_AHB_IEV_R = @intToPtr(*volatile u32, 0x4006040C);
pub const GPIO_PORTJ_AHB_IM_R = @intToPtr(*volatile u32, 0x40060410);
pub const GPIO_PORTJ_AHB_RIS_R = @intToPtr(*volatile u32, 0x40060414);
pub const GPIO_PORTJ_AHB_MIS_R = @intToPtr(*volatile u32, 0x40060418);
pub const GPIO_PORTJ_AHB_ICR_R = @intToPtr(*volatile u32, 0x4006041C);
pub const GPIO_PORTJ_AHB_AFSEL_R = @intToPtr(*volatile u32, 0x40060420);
pub const GPIO_PORTJ_AHB_DR2R_R = @intToPtr(*volatile u32, 0x40060500);
pub const GPIO_PORTJ_AHB_DR4R_R = @intToPtr(*volatile u32, 0x40060504);
pub const GPIO_PORTJ_AHB_DR8R_R = @intToPtr(*volatile u32, 0x40060508);
pub const GPIO_PORTJ_AHB_ODR_R = @intToPtr(*volatile u32, 0x4006050C);
pub const GPIO_PORTJ_AHB_PUR_R = @intToPtr(*volatile u32, 0x40060510);
pub const GPIO_PORTJ_AHB_PDR_R = @intToPtr(*volatile u32, 0x40060514);
pub const GPIO_PORTJ_AHB_SLR_R = @intToPtr(*volatile u32, 0x40060518);
pub const GPIO_PORTJ_AHB_DEN_R = @intToPtr(*volatile u32, 0x4006051C);
pub const GPIO_PORTJ_AHB_LOCK_R = @intToPtr(*volatile u32, 0x40060520);
pub const GPIO_PORTJ_AHB_CR_R = @intToPtr(*volatile u32, 0x40060524);
pub const GPIO_PORTJ_AHB_AMSEL_R = @intToPtr(*volatile u32, 0x40060528);
pub const GPIO_PORTJ_AHB_PCTL_R = @intToPtr(*volatile u32, 0x4006052C);
pub const GPIO_PORTJ_AHB_ADCCTL_R = @intToPtr(*volatile u32, 0x40060530);
pub const GPIO_PORTJ_AHB_DMACTL_R = @intToPtr(*volatile u32, 0x40060534);
pub const GPIO_PORTJ_AHB_SI_R = @intToPtr(*volatile u32, 0x40060538);
pub const GPIO_PORTJ_AHB_DR12R_R = @intToPtr(*volatile u32, 0x4006053C);
pub const GPIO_PORTJ_AHB_WAKEPEN_R = @intToPtr(*volatile u32, 0x40060540);
pub const GPIO_PORTJ_AHB_WAKELVL_R = @intToPtr(*volatile u32, 0x40060544);
pub const GPIO_PORTJ_AHB_WAKESTAT_R = @intToPtr(*volatile u32, 0x40060548);
pub const GPIO_PORTJ_AHB_PP_R = @intToPtr(*volatile u32, 0x40060FC0);
pub const GPIO_PORTJ_AHB_PC_R = @intToPtr(*volatile u32, 0x40060FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTK)
//
//*****************************************************************************
pub const GPIO_PORTK_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40061000);
pub const GPIO_PORTK_DATA_R = @intToPtr(*volatile u32, 0x400613FC);
pub const GPIO_PORTK_DIR_R = @intToPtr(*volatile u32, 0x40061400);
pub const GPIO_PORTK_IS_R = @intToPtr(*volatile u32, 0x40061404);
pub const GPIO_PORTK_IBE_R = @intToPtr(*volatile u32, 0x40061408);
pub const GPIO_PORTK_IEV_R = @intToPtr(*volatile u32, 0x4006140C);
pub const GPIO_PORTK_IM_R = @intToPtr(*volatile u32, 0x40061410);
pub const GPIO_PORTK_RIS_R = @intToPtr(*volatile u32, 0x40061414);
pub const GPIO_PORTK_MIS_R = @intToPtr(*volatile u32, 0x40061418);
pub const GPIO_PORTK_ICR_R = @intToPtr(*volatile u32, 0x4006141C);
pub const GPIO_PORTK_AFSEL_R = @intToPtr(*volatile u32, 0x40061420);
pub const GPIO_PORTK_DR2R_R = @intToPtr(*volatile u32, 0x40061500);
pub const GPIO_PORTK_DR4R_R = @intToPtr(*volatile u32, 0x40061504);
pub const GPIO_PORTK_DR8R_R = @intToPtr(*volatile u32, 0x40061508);
pub const GPIO_PORTK_ODR_R = @intToPtr(*volatile u32, 0x4006150C);
pub const GPIO_PORTK_PUR_R = @intToPtr(*volatile u32, 0x40061510);
pub const GPIO_PORTK_PDR_R = @intToPtr(*volatile u32, 0x40061514);
pub const GPIO_PORTK_SLR_R = @intToPtr(*volatile u32, 0x40061518);
pub const GPIO_PORTK_DEN_R = @intToPtr(*volatile u32, 0x4006151C);
pub const GPIO_PORTK_LOCK_R = @intToPtr(*volatile u32, 0x40061520);
pub const GPIO_PORTK_CR_R = @intToPtr(*volatile u32, 0x40061524);
pub const GPIO_PORTK_AMSEL_R = @intToPtr(*volatile u32, 0x40061528);
pub const GPIO_PORTK_PCTL_R = @intToPtr(*volatile u32, 0x4006152C);
pub const GPIO_PORTK_ADCCTL_R = @intToPtr(*volatile u32, 0x40061530);
pub const GPIO_PORTK_DMACTL_R = @intToPtr(*volatile u32, 0x40061534);
pub const GPIO_PORTK_SI_R = @intToPtr(*volatile u32, 0x40061538);
pub const GPIO_PORTK_DR12R_R = @intToPtr(*volatile u32, 0x4006153C);
pub const GPIO_PORTK_WAKEPEN_R = @intToPtr(*volatile u32, 0x40061540);
pub const GPIO_PORTK_WAKELVL_R = @intToPtr(*volatile u32, 0x40061544);
pub const GPIO_PORTK_WAKESTAT_R = @intToPtr(*volatile u32, 0x40061548);
pub const GPIO_PORTK_PP_R = @intToPtr(*volatile u32, 0x40061FC0);
pub const GPIO_PORTK_PC_R = @intToPtr(*volatile u32, 0x40061FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTL)
//
//*****************************************************************************
pub const GPIO_PORTL_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40062000);
pub const GPIO_PORTL_DATA_R = @intToPtr(*volatile u32, 0x400623FC);
pub const GPIO_PORTL_DIR_R = @intToPtr(*volatile u32, 0x40062400);
pub const GPIO_PORTL_IS_R = @intToPtr(*volatile u32, 0x40062404);
pub const GPIO_PORTL_IBE_R = @intToPtr(*volatile u32, 0x40062408);
pub const GPIO_PORTL_IEV_R = @intToPtr(*volatile u32, 0x4006240C);
pub const GPIO_PORTL_IM_R = @intToPtr(*volatile u32, 0x40062410);
pub const GPIO_PORTL_RIS_R = @intToPtr(*volatile u32, 0x40062414);
pub const GPIO_PORTL_MIS_R = @intToPtr(*volatile u32, 0x40062418);
pub const GPIO_PORTL_ICR_R = @intToPtr(*volatile u32, 0x4006241C);
pub const GPIO_PORTL_AFSEL_R = @intToPtr(*volatile u32, 0x40062420);
pub const GPIO_PORTL_DR2R_R = @intToPtr(*volatile u32, 0x40062500);
pub const GPIO_PORTL_DR4R_R = @intToPtr(*volatile u32, 0x40062504);
pub const GPIO_PORTL_DR8R_R = @intToPtr(*volatile u32, 0x40062508);
pub const GPIO_PORTL_ODR_R = @intToPtr(*volatile u32, 0x4006250C);
pub const GPIO_PORTL_PUR_R = @intToPtr(*volatile u32, 0x40062510);
pub const GPIO_PORTL_PDR_R = @intToPtr(*volatile u32, 0x40062514);
pub const GPIO_PORTL_SLR_R = @intToPtr(*volatile u32, 0x40062518);
pub const GPIO_PORTL_DEN_R = @intToPtr(*volatile u32, 0x4006251C);
pub const GPIO_PORTL_LOCK_R = @intToPtr(*volatile u32, 0x40062520);
pub const GPIO_PORTL_CR_R = @intToPtr(*volatile u32, 0x40062524);
pub const GPIO_PORTL_AMSEL_R = @intToPtr(*volatile u32, 0x40062528);
pub const GPIO_PORTL_PCTL_R = @intToPtr(*volatile u32, 0x4006252C);
pub const GPIO_PORTL_ADCCTL_R = @intToPtr(*volatile u32, 0x40062530);
pub const GPIO_PORTL_DMACTL_R = @intToPtr(*volatile u32, 0x40062534);
pub const GPIO_PORTL_SI_R = @intToPtr(*volatile u32, 0x40062538);
pub const GPIO_PORTL_DR12R_R = @intToPtr(*volatile u32, 0x4006253C);
pub const GPIO_PORTL_WAKEPEN_R = @intToPtr(*volatile u32, 0x40062540);
pub const GPIO_PORTL_WAKELVL_R = @intToPtr(*volatile u32, 0x40062544);
pub const GPIO_PORTL_WAKESTAT_R = @intToPtr(*volatile u32, 0x40062548);
pub const GPIO_PORTL_PP_R = @intToPtr(*volatile u32, 0x40062FC0);
pub const GPIO_PORTL_PC_R = @intToPtr(*volatile u32, 0x40062FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTM)
//
//*****************************************************************************
pub const GPIO_PORTM_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40063000);
pub const GPIO_PORTM_DATA_R = @intToPtr(*volatile u32, 0x400633FC);
pub const GPIO_PORTM_DIR_R = @intToPtr(*volatile u32, 0x40063400);
pub const GPIO_PORTM_IS_R = @intToPtr(*volatile u32, 0x40063404);
pub const GPIO_PORTM_IBE_R = @intToPtr(*volatile u32, 0x40063408);
pub const GPIO_PORTM_IEV_R = @intToPtr(*volatile u32, 0x4006340C);
pub const GPIO_PORTM_IM_R = @intToPtr(*volatile u32, 0x40063410);
pub const GPIO_PORTM_RIS_R = @intToPtr(*volatile u32, 0x40063414);
pub const GPIO_PORTM_MIS_R = @intToPtr(*volatile u32, 0x40063418);
pub const GPIO_PORTM_ICR_R = @intToPtr(*volatile u32, 0x4006341C);
pub const GPIO_PORTM_AFSEL_R = @intToPtr(*volatile u32, 0x40063420);
pub const GPIO_PORTM_DR2R_R = @intToPtr(*volatile u32, 0x40063500);
pub const GPIO_PORTM_DR4R_R = @intToPtr(*volatile u32, 0x40063504);
pub const GPIO_PORTM_DR8R_R = @intToPtr(*volatile u32, 0x40063508);
pub const GPIO_PORTM_ODR_R = @intToPtr(*volatile u32, 0x4006350C);
pub const GPIO_PORTM_PUR_R = @intToPtr(*volatile u32, 0x40063510);
pub const GPIO_PORTM_PDR_R = @intToPtr(*volatile u32, 0x40063514);
pub const GPIO_PORTM_SLR_R = @intToPtr(*volatile u32, 0x40063518);
pub const GPIO_PORTM_DEN_R = @intToPtr(*volatile u32, 0x4006351C);
pub const GPIO_PORTM_LOCK_R = @intToPtr(*volatile u32, 0x40063520);
pub const GPIO_PORTM_CR_R = @intToPtr(*volatile u32, 0x40063524);
pub const GPIO_PORTM_AMSEL_R = @intToPtr(*volatile u32, 0x40063528);
pub const GPIO_PORTM_PCTL_R = @intToPtr(*volatile u32, 0x4006352C);
pub const GPIO_PORTM_ADCCTL_R = @intToPtr(*volatile u32, 0x40063530);
pub const GPIO_PORTM_DMACTL_R = @intToPtr(*volatile u32, 0x40063534);
pub const GPIO_PORTM_SI_R = @intToPtr(*volatile u32, 0x40063538);
pub const GPIO_PORTM_DR12R_R = @intToPtr(*volatile u32, 0x4006353C);
pub const GPIO_PORTM_WAKEPEN_R = @intToPtr(*volatile u32, 0x40063540);
pub const GPIO_PORTM_WAKELVL_R = @intToPtr(*volatile u32, 0x40063544);
pub const GPIO_PORTM_WAKESTAT_R = @intToPtr(*volatile u32, 0x40063548);
pub const GPIO_PORTM_PP_R = @intToPtr(*volatile u32, 0x40063FC0);
pub const GPIO_PORTM_PC_R = @intToPtr(*volatile u32, 0x40063FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTN)
//
//*****************************************************************************
pub const GPIO_PORTN_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40064000);
pub const GPIO_PORTN_DATA_R = @intToPtr(*volatile u32, 0x400643FC);
pub const GPIO_PORTN_DIR_R = @intToPtr(*volatile u32, 0x40064400);
pub const GPIO_PORTN_IS_R = @intToPtr(*volatile u32, 0x40064404);
pub const GPIO_PORTN_IBE_R = @intToPtr(*volatile u32, 0x40064408);
pub const GPIO_PORTN_IEV_R = @intToPtr(*volatile u32, 0x4006440C);
pub const GPIO_PORTN_IM_R = @intToPtr(*volatile u32, 0x40064410);
pub const GPIO_PORTN_RIS_R = @intToPtr(*volatile u32, 0x40064414);
pub const GPIO_PORTN_MIS_R = @intToPtr(*volatile u32, 0x40064418);
pub const GPIO_PORTN_ICR_R = @intToPtr(*volatile u32, 0x4006441C);
pub const GPIO_PORTN_AFSEL_R = @intToPtr(*volatile u32, 0x40064420);
pub const GPIO_PORTN_DR2R_R = @intToPtr(*volatile u32, 0x40064500);
pub const GPIO_PORTN_DR4R_R = @intToPtr(*volatile u32, 0x40064504);
pub const GPIO_PORTN_DR8R_R = @intToPtr(*volatile u32, 0x40064508);
pub const GPIO_PORTN_ODR_R = @intToPtr(*volatile u32, 0x4006450C);
pub const GPIO_PORTN_PUR_R = @intToPtr(*volatile u32, 0x40064510);
pub const GPIO_PORTN_PDR_R = @intToPtr(*volatile u32, 0x40064514);
pub const GPIO_PORTN_SLR_R = @intToPtr(*volatile u32, 0x40064518);
pub const GPIO_PORTN_DEN_R = @intToPtr(*volatile u32, 0x4006451C);
pub const GPIO_PORTN_LOCK_R = @intToPtr(*volatile u32, 0x40064520);
pub const GPIO_PORTN_CR_R = @intToPtr(*volatile u32, 0x40064524);
pub const GPIO_PORTN_AMSEL_R = @intToPtr(*volatile u32, 0x40064528);
pub const GPIO_PORTN_PCTL_R = @intToPtr(*volatile u32, 0x4006452C);
pub const GPIO_PORTN_ADCCTL_R = @intToPtr(*volatile u32, 0x40064530);
pub const GPIO_PORTN_DMACTL_R = @intToPtr(*volatile u32, 0x40064534);
pub const GPIO_PORTN_SI_R = @intToPtr(*volatile u32, 0x40064538);
pub const GPIO_PORTN_DR12R_R = @intToPtr(*volatile u32, 0x4006453C);
pub const GPIO_PORTN_WAKEPEN_R = @intToPtr(*volatile u32, 0x40064540);
pub const GPIO_PORTN_WAKELVL_R = @intToPtr(*volatile u32, 0x40064544);
pub const GPIO_PORTN_WAKESTAT_R = @intToPtr(*volatile u32, 0x40064548);
pub const GPIO_PORTN_PP_R = @intToPtr(*volatile u32, 0x40064FC0);
pub const GPIO_PORTN_PC_R = @intToPtr(*volatile u32, 0x40064FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTP)
//
//*****************************************************************************
pub const GPIO_PORTP_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40065000);
pub const GPIO_PORTP_DATA_R = @intToPtr(*volatile u32, 0x400653FC);
pub const GPIO_PORTP_DIR_R = @intToPtr(*volatile u32, 0x40065400);
pub const GPIO_PORTP_IS_R = @intToPtr(*volatile u32, 0x40065404);
pub const GPIO_PORTP_IBE_R = @intToPtr(*volatile u32, 0x40065408);
pub const GPIO_PORTP_IEV_R = @intToPtr(*volatile u32, 0x4006540C);
pub const GPIO_PORTP_IM_R = @intToPtr(*volatile u32, 0x40065410);
pub const GPIO_PORTP_RIS_R = @intToPtr(*volatile u32, 0x40065414);
pub const GPIO_PORTP_MIS_R = @intToPtr(*volatile u32, 0x40065418);
pub const GPIO_PORTP_ICR_R = @intToPtr(*volatile u32, 0x4006541C);
pub const GPIO_PORTP_AFSEL_R = @intToPtr(*volatile u32, 0x40065420);
pub const GPIO_PORTP_DR2R_R = @intToPtr(*volatile u32, 0x40065500);
pub const GPIO_PORTP_DR4R_R = @intToPtr(*volatile u32, 0x40065504);
pub const GPIO_PORTP_DR8R_R = @intToPtr(*volatile u32, 0x40065508);
pub const GPIO_PORTP_ODR_R = @intToPtr(*volatile u32, 0x4006550C);
pub const GPIO_PORTP_PUR_R = @intToPtr(*volatile u32, 0x40065510);
pub const GPIO_PORTP_PDR_R = @intToPtr(*volatile u32, 0x40065514);
pub const GPIO_PORTP_SLR_R = @intToPtr(*volatile u32, 0x40065518);
pub const GPIO_PORTP_DEN_R = @intToPtr(*volatile u32, 0x4006551C);
pub const GPIO_PORTP_LOCK_R = @intToPtr(*volatile u32, 0x40065520);
pub const GPIO_PORTP_CR_R = @intToPtr(*volatile u32, 0x40065524);
pub const GPIO_PORTP_AMSEL_R = @intToPtr(*volatile u32, 0x40065528);
pub const GPIO_PORTP_PCTL_R = @intToPtr(*volatile u32, 0x4006552C);
pub const GPIO_PORTP_ADCCTL_R = @intToPtr(*volatile u32, 0x40065530);
pub const GPIO_PORTP_DMACTL_R = @intToPtr(*volatile u32, 0x40065534);
pub const GPIO_PORTP_SI_R = @intToPtr(*volatile u32, 0x40065538);
pub const GPIO_PORTP_DR12R_R = @intToPtr(*volatile u32, 0x4006553C);
pub const GPIO_PORTP_WAKEPEN_R = @intToPtr(*volatile u32, 0x40065540);
pub const GPIO_PORTP_WAKELVL_R = @intToPtr(*volatile u32, 0x40065544);
pub const GPIO_PORTP_WAKESTAT_R = @intToPtr(*volatile u32, 0x40065548);
pub const GPIO_PORTP_PP_R = @intToPtr(*volatile u32, 0x40065FC0);
pub const GPIO_PORTP_PC_R = @intToPtr(*volatile u32, 0x40065FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTQ)
//
//*****************************************************************************
pub const GPIO_PORTQ_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40066000);
pub const GPIO_PORTQ_DATA_R = @intToPtr(*volatile u32, 0x400663FC);
pub const GPIO_PORTQ_DIR_R = @intToPtr(*volatile u32, 0x40066400);
pub const GPIO_PORTQ_IS_R = @intToPtr(*volatile u32, 0x40066404);
pub const GPIO_PORTQ_IBE_R = @intToPtr(*volatile u32, 0x40066408);
pub const GPIO_PORTQ_IEV_R = @intToPtr(*volatile u32, 0x4006640C);
pub const GPIO_PORTQ_IM_R = @intToPtr(*volatile u32, 0x40066410);
pub const GPIO_PORTQ_RIS_R = @intToPtr(*volatile u32, 0x40066414);
pub const GPIO_PORTQ_MIS_R = @intToPtr(*volatile u32, 0x40066418);
pub const GPIO_PORTQ_ICR_R = @intToPtr(*volatile u32, 0x4006641C);
pub const GPIO_PORTQ_AFSEL_R = @intToPtr(*volatile u32, 0x40066420);
pub const GPIO_PORTQ_DR2R_R = @intToPtr(*volatile u32, 0x40066500);
pub const GPIO_PORTQ_DR4R_R = @intToPtr(*volatile u32, 0x40066504);
pub const GPIO_PORTQ_DR8R_R = @intToPtr(*volatile u32, 0x40066508);
pub const GPIO_PORTQ_ODR_R = @intToPtr(*volatile u32, 0x4006650C);
pub const GPIO_PORTQ_PUR_R = @intToPtr(*volatile u32, 0x40066510);
pub const GPIO_PORTQ_PDR_R = @intToPtr(*volatile u32, 0x40066514);
pub const GPIO_PORTQ_SLR_R = @intToPtr(*volatile u32, 0x40066518);
pub const GPIO_PORTQ_DEN_R = @intToPtr(*volatile u32, 0x4006651C);
pub const GPIO_PORTQ_LOCK_R = @intToPtr(*volatile u32, 0x40066520);
pub const GPIO_PORTQ_CR_R = @intToPtr(*volatile u32, 0x40066524);
pub const GPIO_PORTQ_AMSEL_R = @intToPtr(*volatile u32, 0x40066528);
pub const GPIO_PORTQ_PCTL_R = @intToPtr(*volatile u32, 0x4006652C);
pub const GPIO_PORTQ_ADCCTL_R = @intToPtr(*volatile u32, 0x40066530);
pub const GPIO_PORTQ_DMACTL_R = @intToPtr(*volatile u32, 0x40066534);
pub const GPIO_PORTQ_SI_R = @intToPtr(*volatile u32, 0x40066538);
pub const GPIO_PORTQ_DR12R_R = @intToPtr(*volatile u32, 0x4006653C);
pub const GPIO_PORTQ_WAKEPEN_R = @intToPtr(*volatile u32, 0x40066540);
pub const GPIO_PORTQ_WAKELVL_R = @intToPtr(*volatile u32, 0x40066544);
pub const GPIO_PORTQ_WAKESTAT_R = @intToPtr(*volatile u32, 0x40066548);
pub const GPIO_PORTQ_PP_R = @intToPtr(*volatile u32, 0x40066FC0);
pub const GPIO_PORTQ_PC_R = @intToPtr(*volatile u32, 0x40066FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTR)
//
//*****************************************************************************
pub const GPIO_PORTR_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40067000);
pub const GPIO_PORTR_DATA_R = @intToPtr(*volatile u32, 0x400673FC);
pub const GPIO_PORTR_DIR_R = @intToPtr(*volatile u32, 0x40067400);
pub const GPIO_PORTR_IS_R = @intToPtr(*volatile u32, 0x40067404);
pub const GPIO_PORTR_IBE_R = @intToPtr(*volatile u32, 0x40067408);
pub const GPIO_PORTR_IEV_R = @intToPtr(*volatile u32, 0x4006740C);
pub const GPIO_PORTR_IM_R = @intToPtr(*volatile u32, 0x40067410);
pub const GPIO_PORTR_RIS_R = @intToPtr(*volatile u32, 0x40067414);
pub const GPIO_PORTR_MIS_R = @intToPtr(*volatile u32, 0x40067418);
pub const GPIO_PORTR_ICR_R = @intToPtr(*volatile u32, 0x4006741C);
pub const GPIO_PORTR_AFSEL_R = @intToPtr(*volatile u32, 0x40067420);
pub const GPIO_PORTR_DR2R_R = @intToPtr(*volatile u32, 0x40067500);
pub const GPIO_PORTR_DR4R_R = @intToPtr(*volatile u32, 0x40067504);
pub const GPIO_PORTR_DR8R_R = @intToPtr(*volatile u32, 0x40067508);
pub const GPIO_PORTR_ODR_R = @intToPtr(*volatile u32, 0x4006750C);
pub const GPIO_PORTR_PUR_R = @intToPtr(*volatile u32, 0x40067510);
pub const GPIO_PORTR_PDR_R = @intToPtr(*volatile u32, 0x40067514);
pub const GPIO_PORTR_SLR_R = @intToPtr(*volatile u32, 0x40067518);
pub const GPIO_PORTR_DEN_R = @intToPtr(*volatile u32, 0x4006751C);
pub const GPIO_PORTR_LOCK_R = @intToPtr(*volatile u32, 0x40067520);
pub const GPIO_PORTR_CR_R = @intToPtr(*volatile u32, 0x40067524);
pub const GPIO_PORTR_AMSEL_R = @intToPtr(*volatile u32, 0x40067528);
pub const GPIO_PORTR_PCTL_R = @intToPtr(*volatile u32, 0x4006752C);
pub const GPIO_PORTR_ADCCTL_R = @intToPtr(*volatile u32, 0x40067530);
pub const GPIO_PORTR_DMACTL_R = @intToPtr(*volatile u32, 0x40067534);
pub const GPIO_PORTR_SI_R = @intToPtr(*volatile u32, 0x40067538);
pub const GPIO_PORTR_DR12R_R = @intToPtr(*volatile u32, 0x4006753C);
pub const GPIO_PORTR_WAKEPEN_R = @intToPtr(*volatile u32, 0x40067540);
pub const GPIO_PORTR_WAKELVL_R = @intToPtr(*volatile u32, 0x40067544);
pub const GPIO_PORTR_WAKESTAT_R = @intToPtr(*volatile u32, 0x40067548);
pub const GPIO_PORTR_PP_R = @intToPtr(*volatile u32, 0x40067FC0);
pub const GPIO_PORTR_PC_R = @intToPtr(*volatile u32, 0x40067FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTS)
//
//*****************************************************************************
pub const GPIO_PORTS_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40068000);
pub const GPIO_PORTS_DATA_R = @intToPtr(*volatile u32, 0x400683FC);
pub const GPIO_PORTS_DIR_R = @intToPtr(*volatile u32, 0x40068400);
pub const GPIO_PORTS_IS_R = @intToPtr(*volatile u32, 0x40068404);
pub const GPIO_PORTS_IBE_R = @intToPtr(*volatile u32, 0x40068408);
pub const GPIO_PORTS_IEV_R = @intToPtr(*volatile u32, 0x4006840C);
pub const GPIO_PORTS_IM_R = @intToPtr(*volatile u32, 0x40068410);
pub const GPIO_PORTS_RIS_R = @intToPtr(*volatile u32, 0x40068414);
pub const GPIO_PORTS_MIS_R = @intToPtr(*volatile u32, 0x40068418);
pub const GPIO_PORTS_ICR_R = @intToPtr(*volatile u32, 0x4006841C);
pub const GPIO_PORTS_AFSEL_R = @intToPtr(*volatile u32, 0x40068420);
pub const GPIO_PORTS_DR2R_R = @intToPtr(*volatile u32, 0x40068500);
pub const GPIO_PORTS_DR4R_R = @intToPtr(*volatile u32, 0x40068504);
pub const GPIO_PORTS_DR8R_R = @intToPtr(*volatile u32, 0x40068508);
pub const GPIO_PORTS_ODR_R = @intToPtr(*volatile u32, 0x4006850C);
pub const GPIO_PORTS_PUR_R = @intToPtr(*volatile u32, 0x40068510);
pub const GPIO_PORTS_PDR_R = @intToPtr(*volatile u32, 0x40068514);
pub const GPIO_PORTS_SLR_R = @intToPtr(*volatile u32, 0x40068518);
pub const GPIO_PORTS_DEN_R = @intToPtr(*volatile u32, 0x4006851C);
pub const GPIO_PORTS_LOCK_R = @intToPtr(*volatile u32, 0x40068520);
pub const GPIO_PORTS_CR_R = @intToPtr(*volatile u32, 0x40068524);
pub const GPIO_PORTS_AMSEL_R = @intToPtr(*volatile u32, 0x40068528);
pub const GPIO_PORTS_PCTL_R = @intToPtr(*volatile u32, 0x4006852C);
pub const GPIO_PORTS_ADCCTL_R = @intToPtr(*volatile u32, 0x40068530);
pub const GPIO_PORTS_DMACTL_R = @intToPtr(*volatile u32, 0x40068534);
pub const GPIO_PORTS_SI_R = @intToPtr(*volatile u32, 0x40068538);
pub const GPIO_PORTS_DR12R_R = @intToPtr(*volatile u32, 0x4006853C);
pub const GPIO_PORTS_WAKEPEN_R = @intToPtr(*volatile u32, 0x40068540);
pub const GPIO_PORTS_WAKELVL_R = @intToPtr(*volatile u32, 0x40068544);
pub const GPIO_PORTS_WAKESTAT_R = @intToPtr(*volatile u32, 0x40068548);
pub const GPIO_PORTS_PP_R = @intToPtr(*volatile u32, 0x40068FC0);
pub const GPIO_PORTS_PC_R = @intToPtr(*volatile u32, 0x40068FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// GPIO registers (PORTT)
//
//*****************************************************************************
pub const GPIO_PORTT_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40069000);
pub const GPIO_PORTT_DATA_R = @intToPtr(*volatile u32, 0x400693FC);
pub const GPIO_PORTT_DIR_R = @intToPtr(*volatile u32, 0x40069400);
pub const GPIO_PORTT_IS_R = @intToPtr(*volatile u32, 0x40069404);
pub const GPIO_PORTT_IBE_R = @intToPtr(*volatile u32, 0x40069408);
pub const GPIO_PORTT_IEV_R = @intToPtr(*volatile u32, 0x4006940C);
pub const GPIO_PORTT_IM_R = @intToPtr(*volatile u32, 0x40069410);
pub const GPIO_PORTT_RIS_R = @intToPtr(*volatile u32, 0x40069414);
pub const GPIO_PORTT_MIS_R = @intToPtr(*volatile u32, 0x40069418);
pub const GPIO_PORTT_ICR_R = @intToPtr(*volatile u32, 0x4006941C);
pub const GPIO_PORTT_AFSEL_R = @intToPtr(*volatile u32, 0x40069420);
pub const GPIO_PORTT_DR2R_R = @intToPtr(*volatile u32, 0x40069500);
pub const GPIO_PORTT_DR4R_R = @intToPtr(*volatile u32, 0x40069504);
pub const GPIO_PORTT_DR8R_R = @intToPtr(*volatile u32, 0x40069508);
pub const GPIO_PORTT_ODR_R = @intToPtr(*volatile u32, 0x4006950C);
pub const GPIO_PORTT_PUR_R = @intToPtr(*volatile u32, 0x40069510);
pub const GPIO_PORTT_PDR_R = @intToPtr(*volatile u32, 0x40069514);
pub const GPIO_PORTT_SLR_R = @intToPtr(*volatile u32, 0x40069518);
pub const GPIO_PORTT_DEN_R = @intToPtr(*volatile u32, 0x4006951C);
pub const GPIO_PORTT_LOCK_R = @intToPtr(*volatile u32, 0x40069520);
pub const GPIO_PORTT_CR_R = @intToPtr(*volatile u32, 0x40069524);
pub const GPIO_PORTT_AMSEL_R = @intToPtr(*volatile u32, 0x40069528);
pub const GPIO_PORTT_PCTL_R = @intToPtr(*volatile u32, 0x4006952C);
pub const GPIO_PORTT_ADCCTL_R = @intToPtr(*volatile u32, 0x40069530);
pub const GPIO_PORTT_DMACTL_R = @intToPtr(*volatile u32, 0x40069534);
pub const GPIO_PORTT_SI_R = @intToPtr(*volatile u32, 0x40069538);
pub const GPIO_PORTT_DR12R_R = @intToPtr(*volatile u32, 0x4006953C);
pub const GPIO_PORTT_WAKEPEN_R = @intToPtr(*volatile u32, 0x40069540);
pub const GPIO_PORTT_WAKELVL_R = @intToPtr(*volatile u32, 0x40069544);
pub const GPIO_PORTT_WAKESTAT_R = @intToPtr(*volatile u32, 0x40069548);
pub const GPIO_PORTT_PP_R = @intToPtr(*volatile u32, 0x40069FC0);
pub const GPIO_PORTT_PC_R = @intToPtr(*volatile u32, 0x40069FC4);
pub const GPIO_PCTL_PR_R = @intToPtr(*volatile u32, 0x00012000);
pub const GPIO_PCTL_PS_R = @intToPtr(*volatile u32, 0x00013000);
pub const GPIO_PCTL_PT_R = @intToPtr(*volatile u32, 0x00014000);

//*****************************************************************************
//
// EEPROM registers (EEPROM)
//
//*****************************************************************************
pub const EEPROM_EESIZE_R = @intToPtr(*volatile u32, 0x400AF000);
pub const EEPROM_EEBLOCK_R = @intToPtr(*volatile u32, 0x400AF004);
pub const EEPROM_EEOFFSET_R = @intToPtr(*volatile u32, 0x400AF008);
pub const EEPROM_EERDWR_R = @intToPtr(*volatile u32, 0x400AF010);
pub const EEPROM_EERDWRINC_R = @intToPtr(*volatile u32, 0x400AF014);
pub const EEPROM_EEDONE_R = @intToPtr(*volatile u32, 0x400AF018);
pub const EEPROM_EESUPP_R = @intToPtr(*volatile u32, 0x400AF01C);
pub const EEPROM_EEUNLOCK_R = @intToPtr(*volatile u32, 0x400AF020);
pub const EEPROM_EEPROT_R = @intToPtr(*volatile u32, 0x400AF030);
pub const EEPROM_EEPASS0_R = @intToPtr(*volatile u32, 0x400AF034);
pub const EEPROM_EEPASS1_R = @intToPtr(*volatile u32, 0x400AF038);
pub const EEPROM_EEPASS2_R = @intToPtr(*volatile u32, 0x400AF03C);
pub const EEPROM_EEINT_R = @intToPtr(*volatile u32, 0x400AF040);
pub const EEPROM_EEHIDE0_R = @intToPtr(*volatile u32, 0x400AF050);
pub const EEPROM_EEHIDE1_R = @intToPtr(*volatile u32, 0x400AF054);
pub const EEPROM_EEHIDE2_R = @intToPtr(*volatile u32, 0x400AF058);
pub const EEPROM_EEDBGME_R = @intToPtr(*volatile u32, 0x400AF080);
pub const EEPROM_PP_R = @intToPtr(*volatile u32, 0x400AFFC0);

//*****************************************************************************
//
// I2C registers (I2C8)
//
//*****************************************************************************
pub const I2C8_MSA_R = @intToPtr(*volatile u32, 0x400B8000);
pub const I2C8_MCS_R = @intToPtr(*volatile u32, 0x400B8004);
pub const I2C8_MDR_R = @intToPtr(*volatile u32, 0x400B8008);
pub const I2C8_MTPR_R = @intToPtr(*volatile u32, 0x400B800C);
pub const I2C8_MIMR_R = @intToPtr(*volatile u32, 0x400B8010);
pub const I2C8_MRIS_R = @intToPtr(*volatile u32, 0x400B8014);
pub const I2C8_MMIS_R = @intToPtr(*volatile u32, 0x400B8018);
pub const I2C8_MICR_R = @intToPtr(*volatile u32, 0x400B801C);
pub const I2C8_MCR_R = @intToPtr(*volatile u32, 0x400B8020);
pub const I2C8_MCLKOCNT_R = @intToPtr(*volatile u32, 0x400B8024);
pub const I2C8_MBMON_R = @intToPtr(*volatile u32, 0x400B802C);
pub const I2C8_MBLEN_R = @intToPtr(*volatile u32, 0x400B8030);
pub const I2C8_MBCNT_R = @intToPtr(*volatile u32, 0x400B8034);
pub const I2C8_SOAR_R = @intToPtr(*volatile u32, 0x400B8800);
pub const I2C8_SCSR_R = @intToPtr(*volatile u32, 0x400B8804);
pub const I2C8_SDR_R = @intToPtr(*volatile u32, 0x400B8808);
pub const I2C8_SIMR_R = @intToPtr(*volatile u32, 0x400B880C);
pub const I2C8_SRIS_R = @intToPtr(*volatile u32, 0x400B8810);
pub const I2C8_SMIS_R = @intToPtr(*volatile u32, 0x400B8814);
pub const I2C8_SICR_R = @intToPtr(*volatile u32, 0x400B8818);
pub const I2C8_SOAR2_R = @intToPtr(*volatile u32, 0x400B881C);
pub const I2C8_SACKCTL_R = @intToPtr(*volatile u32, 0x400B8820);
pub const I2C8_FIFODATA_R = @intToPtr(*volatile u32, 0x400B8F00);
pub const I2C8_FIFOCTL_R = @intToPtr(*volatile u32, 0x400B8F04);
pub const I2C8_FIFOSTATUS_R = @intToPtr(*volatile u32, 0x400B8F08);
pub const I2C8_PP_R = @intToPtr(*volatile u32, 0x400B8FC0);
pub const I2C8_PC_R = @intToPtr(*volatile u32, 0x400B8FC4);

//*****************************************************************************
//
// I2C registers (I2C9)
//
//*****************************************************************************
pub const I2C9_MSA_R = @intToPtr(*volatile u32, 0x400B9000);
pub const I2C9_MCS_R = @intToPtr(*volatile u32, 0x400B9004);
pub const I2C9_MDR_R = @intToPtr(*volatile u32, 0x400B9008);
pub const I2C9_MTPR_R = @intToPtr(*volatile u32, 0x400B900C);
pub const I2C9_MIMR_R = @intToPtr(*volatile u32, 0x400B9010);
pub const I2C9_MRIS_R = @intToPtr(*volatile u32, 0x400B9014);
pub const I2C9_MMIS_R = @intToPtr(*volatile u32, 0x400B9018);
pub const I2C9_MICR_R = @intToPtr(*volatile u32, 0x400B901C);
pub const I2C9_MCR_R = @intToPtr(*volatile u32, 0x400B9020);
pub const I2C9_MCLKOCNT_R = @intToPtr(*volatile u32, 0x400B9024);
pub const I2C9_MBMON_R = @intToPtr(*volatile u32, 0x400B902C);
pub const I2C9_MBLEN_R = @intToPtr(*volatile u32, 0x400B9030);
pub const I2C9_MBCNT_R = @intToPtr(*volatile u32, 0x400B9034);
pub const I2C9_SOAR_R = @intToPtr(*volatile u32, 0x400B9800);
pub const I2C9_SCSR_R = @intToPtr(*volatile u32, 0x400B9804);
pub const I2C9_SDR_R = @intToPtr(*volatile u32, 0x400B9808);
pub const I2C9_SIMR_R = @intToPtr(*volatile u32, 0x400B980C);
pub const I2C9_SRIS_R = @intToPtr(*volatile u32, 0x400B9810);
pub const I2C9_SMIS_R = @intToPtr(*volatile u32, 0x400B9814);
pub const I2C9_SICR_R = @intToPtr(*volatile u32, 0x400B9818);
pub const I2C9_SOAR2_R = @intToPtr(*volatile u32, 0x400B981C);
pub const I2C9_SACKCTL_R = @intToPtr(*volatile u32, 0x400B9820);
pub const I2C9_FIFODATA_R = @intToPtr(*volatile u32, 0x400B9F00);
pub const I2C9_FIFOCTL_R = @intToPtr(*volatile u32, 0x400B9F04);
pub const I2C9_FIFOSTATUS_R = @intToPtr(*volatile u32, 0x400B9F08);
pub const I2C9_PP_R = @intToPtr(*volatile u32, 0x400B9FC0);
pub const I2C9_PC_R = @intToPtr(*volatile u32, 0x400B9FC4);

//*****************************************************************************
//
// I2C registers (I2C4)
//
//*****************************************************************************
pub const I2C4_MSA_R = @intToPtr(*volatile u32, 0x400C0000);
pub const I2C4_MCS_R = @intToPtr(*volatile u32, 0x400C0004);
pub const I2C4_MDR_R = @intToPtr(*volatile u32, 0x400C0008);
pub const I2C4_MTPR_R = @intToPtr(*volatile u32, 0x400C000C);
pub const I2C4_MIMR_R = @intToPtr(*volatile u32, 0x400C0010);
pub const I2C4_MRIS_R = @intToPtr(*volatile u32, 0x400C0014);
pub const I2C4_MMIS_R = @intToPtr(*volatile u32, 0x400C0018);
pub const I2C4_MICR_R = @intToPtr(*volatile u32, 0x400C001C);
pub const I2C4_MCR_R = @intToPtr(*volatile u32, 0x400C0020);
pub const I2C4_MCLKOCNT_R = @intToPtr(*volatile u32, 0x400C0024);
pub const I2C4_MBMON_R = @intToPtr(*volatile u32, 0x400C002C);
pub const I2C4_MBLEN_R = @intToPtr(*volatile u32, 0x400C0030);
pub const I2C4_MBCNT_R = @intToPtr(*volatile u32, 0x400C0034);
pub const I2C4_SOAR_R = @intToPtr(*volatile u32, 0x400C0800);
pub const I2C4_SCSR_R = @intToPtr(*volatile u32, 0x400C0804);
pub const I2C4_SDR_R = @intToPtr(*volatile u32, 0x400C0808);
pub const I2C4_SIMR_R = @intToPtr(*volatile u32, 0x400C080C);
pub const I2C4_SRIS_R = @intToPtr(*volatile u32, 0x400C0810);
pub const I2C4_SMIS_R = @intToPtr(*volatile u32, 0x400C0814);
pub const I2C4_SICR_R = @intToPtr(*volatile u32, 0x400C0818);
pub const I2C4_SOAR2_R = @intToPtr(*volatile u32, 0x400C081C);
pub const I2C4_SACKCTL_R = @intToPtr(*volatile u32, 0x400C0820);
pub const I2C4_FIFODATA_R = @intToPtr(*volatile u32, 0x400C0F00);
pub const I2C4_FIFOCTL_R = @intToPtr(*volatile u32, 0x400C0F04);
pub const I2C4_FIFOSTATUS_R = @intToPtr(*volatile u32, 0x400C0F08);
pub const I2C4_PP_R = @intToPtr(*volatile u32, 0x400C0FC0);
pub const I2C4_PC_R = @intToPtr(*volatile u32, 0x400C0FC4);

//*****************************************************************************
//
// I2C registers (I2C5)
//
//*****************************************************************************
pub const I2C5_MSA_R = @intToPtr(*volatile u32, 0x400C1000);
pub const I2C5_MCS_R = @intToPtr(*volatile u32, 0x400C1004);
pub const I2C5_MDR_R = @intToPtr(*volatile u32, 0x400C1008);
pub const I2C5_MTPR_R = @intToPtr(*volatile u32, 0x400C100C);
pub const I2C5_MIMR_R = @intToPtr(*volatile u32, 0x400C1010);
pub const I2C5_MRIS_R = @intToPtr(*volatile u32, 0x400C1014);
pub const I2C5_MMIS_R = @intToPtr(*volatile u32, 0x400C1018);
pub const I2C5_MICR_R = @intToPtr(*volatile u32, 0x400C101C);
pub const I2C5_MCR_R = @intToPtr(*volatile u32, 0x400C1020);
pub const I2C5_MCLKOCNT_R = @intToPtr(*volatile u32, 0x400C1024);
pub const I2C5_MBMON_R = @intToPtr(*volatile u32, 0x400C102C);
pub const I2C5_MBLEN_R = @intToPtr(*volatile u32, 0x400C1030);
pub const I2C5_MBCNT_R = @intToPtr(*volatile u32, 0x400C1034);
pub const I2C5_SOAR_R = @intToPtr(*volatile u32, 0x400C1800);
pub const I2C5_SCSR_R = @intToPtr(*volatile u32, 0x400C1804);
pub const I2C5_SDR_R = @intToPtr(*volatile u32, 0x400C1808);
pub const I2C5_SIMR_R = @intToPtr(*volatile u32, 0x400C180C);
pub const I2C5_SRIS_R = @intToPtr(*volatile u32, 0x400C1810);
pub const I2C5_SMIS_R = @intToPtr(*volatile u32, 0x400C1814);
pub const I2C5_SICR_R = @intToPtr(*volatile u32, 0x400C1818);
pub const I2C5_SOAR2_R = @intToPtr(*volatile u32, 0x400C181C);
pub const I2C5_SACKCTL_R = @intToPtr(*volatile u32, 0x400C1820);
pub const I2C5_FIFODATA_R = @intToPtr(*volatile u32, 0x400C1F00);
pub const I2C5_FIFOCTL_R = @intToPtr(*volatile u32, 0x400C1F04);
pub const I2C5_FIFOSTATUS_R = @intToPtr(*volatile u32, 0x400C1F08);
pub const I2C5_PP_R = @intToPtr(*volatile u32, 0x400C1FC0);
pub const I2C5_PC_R = @intToPtr(*volatile u32, 0x400C1FC4);

//*****************************************************************************
//
// I2C registers (I2C6)
//
//*****************************************************************************
pub const I2C6_MSA_R = @intToPtr(*volatile u32, 0x400C2000);
pub const I2C6_MCS_R = @intToPtr(*volatile u32, 0x400C2004);
pub const I2C6_MDR_R = @intToPtr(*volatile u32, 0x400C2008);
pub const I2C6_MTPR_R = @intToPtr(*volatile u32, 0x400C200C);
pub const I2C6_MIMR_R = @intToPtr(*volatile u32, 0x400C2010);
pub const I2C6_MRIS_R = @intToPtr(*volatile u32, 0x400C2014);
pub const I2C6_MMIS_R = @intToPtr(*volatile u32, 0x400C2018);
pub const I2C6_MICR_R = @intToPtr(*volatile u32, 0x400C201C);
pub const I2C6_MCR_R = @intToPtr(*volatile u32, 0x400C2020);
pub const I2C6_MCLKOCNT_R = @intToPtr(*volatile u32, 0x400C2024);
pub const I2C6_MBMON_R = @intToPtr(*volatile u32, 0x400C202C);
pub const I2C6_MBLEN_R = @intToPtr(*volatile u32, 0x400C2030);
pub const I2C6_MBCNT_R = @intToPtr(*volatile u32, 0x400C2034);
pub const I2C6_SOAR_R = @intToPtr(*volatile u32, 0x400C2800);
pub const I2C6_SCSR_R = @intToPtr(*volatile u32, 0x400C2804);
pub const I2C6_SDR_R = @intToPtr(*volatile u32, 0x400C2808);
pub const I2C6_SIMR_R = @intToPtr(*volatile u32, 0x400C280C);
pub const I2C6_SRIS_R = @intToPtr(*volatile u32, 0x400C2810);
pub const I2C6_SMIS_R = @intToPtr(*volatile u32, 0x400C2814);
pub const I2C6_SICR_R = @intToPtr(*volatile u32, 0x400C2818);
pub const I2C6_SOAR2_R = @intToPtr(*volatile u32, 0x400C281C);
pub const I2C6_SACKCTL_R = @intToPtr(*volatile u32, 0x400C2820);
pub const I2C6_FIFODATA_R = @intToPtr(*volatile u32, 0x400C2F00);
pub const I2C6_FIFOCTL_R = @intToPtr(*volatile u32, 0x400C2F04);
pub const I2C6_FIFOSTATUS_R = @intToPtr(*volatile u32, 0x400C2F08);
pub const I2C6_PP_R = @intToPtr(*volatile u32, 0x400C2FC0);
pub const I2C6_PC_R = @intToPtr(*volatile u32, 0x400C2FC4);

//*****************************************************************************
//
// I2C registers (I2C7)
//
//*****************************************************************************
pub const I2C7_MSA_R = @intToPtr(*volatile u32, 0x400C3000);
pub const I2C7_MCS_R = @intToPtr(*volatile u32, 0x400C3004);
pub const I2C7_MDR_R = @intToPtr(*volatile u32, 0x400C3008);
pub const I2C7_MTPR_R = @intToPtr(*volatile u32, 0x400C300C);
pub const I2C7_MIMR_R = @intToPtr(*volatile u32, 0x400C3010);
pub const I2C7_MRIS_R = @intToPtr(*volatile u32, 0x400C3014);
pub const I2C7_MMIS_R = @intToPtr(*volatile u32, 0x400C3018);
pub const I2C7_MICR_R = @intToPtr(*volatile u32, 0x400C301C);
pub const I2C7_MCR_R = @intToPtr(*volatile u32, 0x400C3020);
pub const I2C7_MCLKOCNT_R = @intToPtr(*volatile u32, 0x400C3024);
pub const I2C7_MBMON_R = @intToPtr(*volatile u32, 0x400C302C);
pub const I2C7_MBLEN_R = @intToPtr(*volatile u32, 0x400C3030);
pub const I2C7_MBCNT_R = @intToPtr(*volatile u32, 0x400C3034);
pub const I2C7_SOAR_R = @intToPtr(*volatile u32, 0x400C3800);
pub const I2C7_SCSR_R = @intToPtr(*volatile u32, 0x400C3804);
pub const I2C7_SDR_R = @intToPtr(*volatile u32, 0x400C3808);
pub const I2C7_SIMR_R = @intToPtr(*volatile u32, 0x400C380C);
pub const I2C7_SRIS_R = @intToPtr(*volatile u32, 0x400C3810);
pub const I2C7_SMIS_R = @intToPtr(*volatile u32, 0x400C3814);
pub const I2C7_SICR_R = @intToPtr(*volatile u32, 0x400C3818);
pub const I2C7_SOAR2_R = @intToPtr(*volatile u32, 0x400C381C);
pub const I2C7_SACKCTL_R = @intToPtr(*volatile u32, 0x400C3820);
pub const I2C7_FIFODATA_R = @intToPtr(*volatile u32, 0x400C3F00);
pub const I2C7_FIFOCTL_R = @intToPtr(*volatile u32, 0x400C3F04);
pub const I2C7_FIFOSTATUS_R = @intToPtr(*volatile u32, 0x400C3F08);
pub const I2C7_PP_R = @intToPtr(*volatile u32, 0x400C3FC0);
pub const I2C7_PC_R = @intToPtr(*volatile u32, 0x400C3FC4);

//*****************************************************************************
//
// External Peripheral Interface registers (EPI0)
//
//*****************************************************************************
pub const EPI0_CFG_R = @intToPtr(*volatile u32, 0x400D0000);
pub const EPI0_BAUD_R = @intToPtr(*volatile u32, 0x400D0004);
pub const EPI0_BAUD2_R = @intToPtr(*volatile u32, 0x400D0008);
pub const EPI0_HB16CFG_R = @intToPtr(*volatile u32, 0x400D0010);
pub const EPI0_GPCFG_R = @intToPtr(*volatile u32, 0x400D0010);
pub const EPI0_SDRAMCFG_R = @intToPtr(*volatile u32, 0x400D0010);
pub const EPI0_HB8CFG_R = @intToPtr(*volatile u32, 0x400D0010);
pub const EPI0_HB8CFG2_R = @intToPtr(*volatile u32, 0x400D0014);
pub const EPI0_HB16CFG2_R = @intToPtr(*volatile u32, 0x400D0014);
pub const EPI0_ADDRMAP_R = @intToPtr(*volatile u32, 0x400D001C);
pub const EPI0_RSIZE0_R = @intToPtr(*volatile u32, 0x400D0020);
pub const EPI0_RADDR0_R = @intToPtr(*volatile u32, 0x400D0024);
pub const EPI0_RPSTD0_R = @intToPtr(*volatile u32, 0x400D0028);
pub const EPI0_RSIZE1_R = @intToPtr(*volatile u32, 0x400D0030);
pub const EPI0_RADDR1_R = @intToPtr(*volatile u32, 0x400D0034);
pub const EPI0_RPSTD1_R = @intToPtr(*volatile u32, 0x400D0038);
pub const EPI0_STAT_R = @intToPtr(*volatile u32, 0x400D0060);
pub const EPI0_RFIFOCNT_R = @intToPtr(*volatile u32, 0x400D006C);
pub const EPI0_READFIFO0_R = @intToPtr(*volatile u32, 0x400D0070);
pub const EPI0_READFIFO1_R = @intToPtr(*volatile u32, 0x400D0074);
pub const EPI0_READFIFO2_R = @intToPtr(*volatile u32, 0x400D0078);
pub const EPI0_READFIFO3_R = @intToPtr(*volatile u32, 0x400D007C);
pub const EPI0_READFIFO4_R = @intToPtr(*volatile u32, 0x400D0080);
pub const EPI0_READFIFO5_R = @intToPtr(*volatile u32, 0x400D0084);
pub const EPI0_READFIFO6_R = @intToPtr(*volatile u32, 0x400D0088);
pub const EPI0_READFIFO7_R = @intToPtr(*volatile u32, 0x400D008C);
pub const EPI0_FIFOLVL_R = @intToPtr(*volatile u32, 0x400D0200);
pub const EPI0_WFIFOCNT_R = @intToPtr(*volatile u32, 0x400D0204);
pub const EPI0_DMATXCNT_R = @intToPtr(*volatile u32, 0x400D0208);
pub const EPI0_IM_R = @intToPtr(*volatile u32, 0x400D0210);
pub const EPI0_RIS_R = @intToPtr(*volatile u32, 0x400D0214);
pub const EPI0_MIS_R = @intToPtr(*volatile u32, 0x400D0218);
pub const EPI0_EISC_R = @intToPtr(*volatile u32, 0x400D021C);
pub const EPI0_HB8CFG3_R = @intToPtr(*volatile u32, 0x400D0308);
pub const EPI0_HB16CFG3_R = @intToPtr(*volatile u32, 0x400D0308);
pub const EPI0_HB16CFG4_R = @intToPtr(*volatile u32, 0x400D030C);
pub const EPI0_HB8CFG4_R = @intToPtr(*volatile u32, 0x400D030C);
pub const EPI0_HB8TIME_R = @intToPtr(*volatile u32, 0x400D0310);
pub const EPI0_HB16TIME_R = @intToPtr(*volatile u32, 0x400D0310);
pub const EPI0_HB8TIME2_R = @intToPtr(*volatile u32, 0x400D0314);
pub const EPI0_HB16TIME2_R = @intToPtr(*volatile u32, 0x400D0314);
pub const EPI0_HB16TIME3_R = @intToPtr(*volatile u32, 0x400D0318);
pub const EPI0_HB8TIME3_R = @intToPtr(*volatile u32, 0x400D0318);
pub const EPI0_HB8TIME4_R = @intToPtr(*volatile u32, 0x400D031C);
pub const EPI0_HB16TIME4_R = @intToPtr(*volatile u32, 0x400D031C);
pub const EPI0_HBPSRAM_R = @intToPtr(*volatile u32, 0x400D0360);

//*****************************************************************************
//
// Timer registers (TIMER6)
//
//*****************************************************************************
pub const TIMER6_CFG_R = @intToPtr(*volatile u32, 0x400E0000);
pub const TIMER6_TAMR_R = @intToPtr(*volatile u32, 0x400E0004);
pub const TIMER6_TBMR_R = @intToPtr(*volatile u32, 0x400E0008);
pub const TIMER6_CTL_R = @intToPtr(*volatile u32, 0x400E000C);
pub const TIMER6_SYNC_R = @intToPtr(*volatile u32, 0x400E0010);
pub const TIMER6_IMR_R = @intToPtr(*volatile u32, 0x400E0018);
pub const TIMER6_RIS_R = @intToPtr(*volatile u32, 0x400E001C);
pub const TIMER6_MIS_R = @intToPtr(*volatile u32, 0x400E0020);
pub const TIMER6_ICR_R = @intToPtr(*volatile u32, 0x400E0024);
pub const TIMER6_TAILR_R = @intToPtr(*volatile u32, 0x400E0028);
pub const TIMER6_TBILR_R = @intToPtr(*volatile u32, 0x400E002C);
pub const TIMER6_TAMATCHR_R = @intToPtr(*volatile u32, 0x400E0030);
pub const TIMER6_TBMATCHR_R = @intToPtr(*volatile u32, 0x400E0034);
pub const TIMER6_TAPR_R = @intToPtr(*volatile u32, 0x400E0038);
pub const TIMER6_TBPR_R = @intToPtr(*volatile u32, 0x400E003C);
pub const TIMER6_TAPMR_R = @intToPtr(*volatile u32, 0x400E0040);
pub const TIMER6_TBPMR_R = @intToPtr(*volatile u32, 0x400E0044);
pub const TIMER6_TAR_R = @intToPtr(*volatile u32, 0x400E0048);
pub const TIMER6_TBR_R = @intToPtr(*volatile u32, 0x400E004C);
pub const TIMER6_TAV_R = @intToPtr(*volatile u32, 0x400E0050);
pub const TIMER6_TBV_R = @intToPtr(*volatile u32, 0x400E0054);
pub const TIMER6_RTCPD_R = @intToPtr(*volatile u32, 0x400E0058);
pub const TIMER6_TAPS_R = @intToPtr(*volatile u32, 0x400E005C);
pub const TIMER6_TBPS_R = @intToPtr(*volatile u32, 0x400E0060);
pub const TIMER6_DMAEV_R = @intToPtr(*volatile u32, 0x400E006C);
pub const TIMER6_ADCEV_R = @intToPtr(*volatile u32, 0x400E0070);
pub const TIMER6_PP_R = @intToPtr(*volatile u32, 0x400E0FC0);
pub const TIMER6_CC_R = @intToPtr(*volatile u32, 0x400E0FC8);

//*****************************************************************************
//
// Timer registers (TIMER7)
//
//*****************************************************************************
pub const TIMER7_CFG_R = @intToPtr(*volatile u32, 0x400E1000);
pub const TIMER7_TAMR_R = @intToPtr(*volatile u32, 0x400E1004);
pub const TIMER7_TBMR_R = @intToPtr(*volatile u32, 0x400E1008);
pub const TIMER7_CTL_R = @intToPtr(*volatile u32, 0x400E100C);
pub const TIMER7_SYNC_R = @intToPtr(*volatile u32, 0x400E1010);
pub const TIMER7_IMR_R = @intToPtr(*volatile u32, 0x400E1018);
pub const TIMER7_RIS_R = @intToPtr(*volatile u32, 0x400E101C);
pub const TIMER7_MIS_R = @intToPtr(*volatile u32, 0x400E1020);
pub const TIMER7_ICR_R = @intToPtr(*volatile u32, 0x400E1024);
pub const TIMER7_TAILR_R = @intToPtr(*volatile u32, 0x400E1028);
pub const TIMER7_TBILR_R = @intToPtr(*volatile u32, 0x400E102C);
pub const TIMER7_TAMATCHR_R = @intToPtr(*volatile u32, 0x400E1030);
pub const TIMER7_TBMATCHR_R = @intToPtr(*volatile u32, 0x400E1034);
pub const TIMER7_TAPR_R = @intToPtr(*volatile u32, 0x400E1038);
pub const TIMER7_TBPR_R = @intToPtr(*volatile u32, 0x400E103C);
pub const TIMER7_TAPMR_R = @intToPtr(*volatile u32, 0x400E1040);
pub const TIMER7_TBPMR_R = @intToPtr(*volatile u32, 0x400E1044);
pub const TIMER7_TAR_R = @intToPtr(*volatile u32, 0x400E1048);
pub const TIMER7_TBR_R = @intToPtr(*volatile u32, 0x400E104C);
pub const TIMER7_TAV_R = @intToPtr(*volatile u32, 0x400E1050);
pub const TIMER7_TBV_R = @intToPtr(*volatile u32, 0x400E1054);
pub const TIMER7_RTCPD_R = @intToPtr(*volatile u32, 0x400E1058);
pub const TIMER7_TAPS_R = @intToPtr(*volatile u32, 0x400E105C);
pub const TIMER7_TBPS_R = @intToPtr(*volatile u32, 0x400E1060);
pub const TIMER7_DMAEV_R = @intToPtr(*volatile u32, 0x400E106C);
pub const TIMER7_ADCEV_R = @intToPtr(*volatile u32, 0x400E1070);
pub const TIMER7_PP_R = @intToPtr(*volatile u32, 0x400E1FC0);
pub const TIMER7_CC_R = @intToPtr(*volatile u32, 0x400E1FC8);

//*****************************************************************************
//
// EMAC registers (EMAC0)
//
//*****************************************************************************
pub const EMAC0_CFG_R = @intToPtr(*volatile u32, 0x400EC000);
pub const EMAC0_FRAMEFLTR_R = @intToPtr(*volatile u32, 0x400EC004);
pub const EMAC0_HASHTBLH_R = @intToPtr(*volatile u32, 0x400EC008);
pub const EMAC0_HASHTBLL_R = @intToPtr(*volatile u32, 0x400EC00C);
pub const EMAC0_MIIADDR_R = @intToPtr(*volatile u32, 0x400EC010);
pub const EMAC0_MIIDATA_R = @intToPtr(*volatile u32, 0x400EC014);
pub const EMAC0_FLOWCTL_R = @intToPtr(*volatile u32, 0x400EC018);
pub const EMAC0_VLANTG_R = @intToPtr(*volatile u32, 0x400EC01C);
pub const EMAC0_STATUS_R = @intToPtr(*volatile u32, 0x400EC024);
pub const EMAC0_RWUFF_R = @intToPtr(*volatile u32, 0x400EC028);
pub const EMAC0_PMTCTLSTAT_R = @intToPtr(*volatile u32, 0x400EC02C);
pub const EMAC0_RIS_R = @intToPtr(*volatile u32, 0x400EC038);
pub const EMAC0_IM_R = @intToPtr(*volatile u32, 0x400EC03C);
pub const EMAC0_ADDR0H_R = @intToPtr(*volatile u32, 0x400EC040);
pub const EMAC0_ADDR0L_R = @intToPtr(*volatile u32, 0x400EC044);
pub const EMAC0_ADDR1H_R = @intToPtr(*volatile u32, 0x400EC048);
pub const EMAC0_ADDR1L_R = @intToPtr(*volatile u32, 0x400EC04C);
pub const EMAC0_ADDR2H_R = @intToPtr(*volatile u32, 0x400EC050);
pub const EMAC0_ADDR2L_R = @intToPtr(*volatile u32, 0x400EC054);
pub const EMAC0_ADDR3H_R = @intToPtr(*volatile u32, 0x400EC058);
pub const EMAC0_ADDR3L_R = @intToPtr(*volatile u32, 0x400EC05C);
pub const EMAC0_WDOGTO_R = @intToPtr(*volatile u32, 0x400EC0DC);
pub const EMAC0_MMCCTRL_R = @intToPtr(*volatile u32, 0x400EC100);
pub const EMAC0_MMCRXRIS_R = @intToPtr(*volatile u32, 0x400EC104);
pub const EMAC0_MMCTXRIS_R = @intToPtr(*volatile u32, 0x400EC108);
pub const EMAC0_MMCRXIM_R = @intToPtr(*volatile u32, 0x400EC10C);
pub const EMAC0_MMCTXIM_R = @intToPtr(*volatile u32, 0x400EC110);
pub const EMAC0_TXCNTGB_R = @intToPtr(*volatile u32, 0x400EC118);
pub const EMAC0_TXCNTSCOL_R = @intToPtr(*volatile u32, 0x400EC14C);
pub const EMAC0_TXCNTMCOL_R = @intToPtr(*volatile u32, 0x400EC150);
pub const EMAC0_TXOCTCNTG_R = @intToPtr(*volatile u32, 0x400EC164);
pub const EMAC0_RXCNTGB_R = @intToPtr(*volatile u32, 0x400EC180);
pub const EMAC0_RXCNTCRCERR_R = @intToPtr(*volatile u32, 0x400EC194);
pub const EMAC0_RXCNTALGNERR_R = @intToPtr(*volatile u32, 0x400EC198);
pub const EMAC0_RXCNTGUNI_R = @intToPtr(*volatile u32, 0x400EC1C4);
pub const EMAC0_VLNINCREP_R = @intToPtr(*volatile u32, 0x400EC584);
pub const EMAC0_VLANHASH_R = @intToPtr(*volatile u32, 0x400EC588);
pub const EMAC0_TIMSTCTRL_R = @intToPtr(*volatile u32, 0x400EC700);
pub const EMAC0_SUBSECINC_R = @intToPtr(*volatile u32, 0x400EC704);
pub const EMAC0_TIMSEC_R = @intToPtr(*volatile u32, 0x400EC708);
pub const EMAC0_TIMNANO_R = @intToPtr(*volatile u32, 0x400EC70C);
pub const EMAC0_TIMSECU_R = @intToPtr(*volatile u32, 0x400EC710);
pub const EMAC0_TIMNANOU_R = @intToPtr(*volatile u32, 0x400EC714);
pub const EMAC0_TIMADD_R = @intToPtr(*volatile u32, 0x400EC718);
pub const EMAC0_TARGSEC_R = @intToPtr(*volatile u32, 0x400EC71C);
pub const EMAC0_TARGNANO_R = @intToPtr(*volatile u32, 0x400EC720);
pub const EMAC0_HWORDSEC_R = @intToPtr(*volatile u32, 0x400EC724);
pub const EMAC0_TIMSTAT_R = @intToPtr(*volatile u32, 0x400EC728);
pub const EMAC0_PPSCTRL_R = @intToPtr(*volatile u32, 0x400EC72C);
pub const EMAC0_PPS0INTVL_R = @intToPtr(*volatile u32, 0x400EC760);
pub const EMAC0_PPS0WIDTH_R = @intToPtr(*volatile u32, 0x400EC764);
pub const EMAC0_DMABUSMOD_R = @intToPtr(*volatile u32, 0x400ECC00);
pub const EMAC0_TXPOLLD_R = @intToPtr(*volatile u32, 0x400ECC04);
pub const EMAC0_RXPOLLD_R = @intToPtr(*volatile u32, 0x400ECC08);
pub const EMAC0_RXDLADDR_R = @intToPtr(*volatile u32, 0x400ECC0C);
pub const EMAC0_TXDLADDR_R = @intToPtr(*volatile u32, 0x400ECC10);
pub const EMAC0_DMARIS_R = @intToPtr(*volatile u32, 0x400ECC14);
pub const EMAC0_DMAOPMODE_R = @intToPtr(*volatile u32, 0x400ECC18);
pub const EMAC0_DMAIM_R = @intToPtr(*volatile u32, 0x400ECC1C);
pub const EMAC0_MFBOC_R = @intToPtr(*volatile u32, 0x400ECC20);
pub const EMAC0_RXINTWDT_R = @intToPtr(*volatile u32, 0x400ECC24);
pub const EMAC0_HOSTXDESC_R = @intToPtr(*volatile u32, 0x400ECC48);
pub const EMAC0_HOSRXDESC_R = @intToPtr(*volatile u32, 0x400ECC4C);
pub const EMAC0_HOSTXBA_R = @intToPtr(*volatile u32, 0x400ECC50);
pub const EMAC0_HOSRXBA_R = @intToPtr(*volatile u32, 0x400ECC54);
pub const EMAC0_PP_R = @intToPtr(*volatile u32, 0x400ECFC0);
pub const EMAC0_PC_R = @intToPtr(*volatile u32, 0x400ECFC4);
pub const EMAC0_CC_R = @intToPtr(*volatile u32, 0x400ECFC8);
pub const EMAC0_EPHYRIS_R = @intToPtr(*volatile u32, 0x400ECFD0);
pub const EMAC0_EPHYIM_R = @intToPtr(*volatile u32, 0x400ECFD4);
pub const EMAC0_EPHYMISC_R = @intToPtr(*volatile u32, 0x400ECFD8);

//*****************************************************************************
//
// System Exception Module registers (SYSEXC)
//
//*****************************************************************************
pub const SYSEXC_RIS_R = @intToPtr(*volatile u32, 0x400F9000);
pub const SYSEXC_IM_R = @intToPtr(*volatile u32, 0x400F9004);
pub const SYSEXC_MIS_R = @intToPtr(*volatile u32, 0x400F9008);
pub const SYSEXC_IC_R = @intToPtr(*volatile u32, 0x400F900C);

//*****************************************************************************
//
// Hibernation module registers (HIB)
//
//*****************************************************************************
pub const HIB_RTCC_R = @intToPtr(*volatile u32, 0x400FC000);
pub const HIB_RTCM0_R = @intToPtr(*volatile u32, 0x400FC004);
pub const HIB_RTCLD_R = @intToPtr(*volatile u32, 0x400FC00C);
pub const HIB_CTL_R = @intToPtr(*volatile u32, 0x400FC010);
pub const HIB_IM_R = @intToPtr(*volatile u32, 0x400FC014);
pub const HIB_RIS_R = @intToPtr(*volatile u32, 0x400FC018);
pub const HIB_MIS_R = @intToPtr(*volatile u32, 0x400FC01C);
pub const HIB_IC_R = @intToPtr(*volatile u32, 0x400FC020);
pub const HIB_RTCT_R = @intToPtr(*volatile u32, 0x400FC024);
pub const HIB_RTCSS_R = @intToPtr(*volatile u32, 0x400FC028);
pub const HIB_IO_R = @intToPtr(*volatile u32, 0x400FC02C);
pub const HIB_DATA_R = @intToPtr(*volatile u32, 0x400FC030);
pub const HIB_CALCTL_R = @intToPtr(*volatile u32, 0x400FC300);
pub const HIB_CAL0_R = @intToPtr(*volatile u32, 0x400FC310);
pub const HIB_CAL1_R = @intToPtr(*volatile u32, 0x400FC314);
pub const HIB_CALLD0_R = @intToPtr(*volatile u32, 0x400FC320);
pub const HIB_CALLD1_R = @intToPtr(*volatile u32, 0x400FC324);
pub const HIB_CALM0_R = @intToPtr(*volatile u32, 0x400FC330);
pub const HIB_CALM1_R = @intToPtr(*volatile u32, 0x400FC334);
pub const HIB_LOCK_R = @intToPtr(*volatile u32, 0x400FC360);
pub const HIB_TPCTL_R = @intToPtr(*volatile u32, 0x400FC400);
pub const HIB_TPSTAT_R = @intToPtr(*volatile u32, 0x400FC404);
pub const HIB_TPIO_R = @intToPtr(*volatile u32, 0x400FC410);
pub const HIB_TPLOG0_R = @intToPtr(*volatile u32, 0x400FC4E0);
pub const HIB_TPLOG1_R = @intToPtr(*volatile u32, 0x400FC4E4);
pub const HIB_TPLOG2_R = @intToPtr(*volatile u32, 0x400FC4E8);
pub const HIB_TPLOG3_R = @intToPtr(*volatile u32, 0x400FC4EC);
pub const HIB_TPLOG4_R = @intToPtr(*volatile u32, 0x400FC4F0);
pub const HIB_TPLOG5_R = @intToPtr(*volatile u32, 0x400FC4F4);
pub const HIB_TPLOG6_R = @intToPtr(*volatile u32, 0x400FC4F8);
pub const HIB_TPLOG7_R = @intToPtr(*volatile u32, 0x400FC4FC);
pub const HIB_PP_R = @intToPtr(*volatile u32, 0x400FCFC0);
pub const HIB_CC_R = @intToPtr(*volatile u32, 0x400FCFC8);

//*****************************************************************************
//
// FLASH registers (FLASH CTRL)
//
//*****************************************************************************
pub const FLASH_FMA_R = @intToPtr(*volatile u32, 0x400FD000);
pub const FLASH_FMD_R = @intToPtr(*volatile u32, 0x400FD004);
pub const FLASH_FMC_R = @intToPtr(*volatile u32, 0x400FD008);
pub const FLASH_FCRIS_R = @intToPtr(*volatile u32, 0x400FD00C);
pub const FLASH_FCIM_R = @intToPtr(*volatile u32, 0x400FD010);
pub const FLASH_FCMISC_R = @intToPtr(*volatile u32, 0x400FD014);
pub const FLASH_FMC2_R = @intToPtr(*volatile u32, 0x400FD020);
pub const FLASH_FWBVAL_R = @intToPtr(*volatile u32, 0x400FD030);
pub const FLASH_FLPEKEY_R = @intToPtr(*volatile u32, 0x400FD03C);
pub const FLASH_FWBN_R = @intToPtr(*volatile u32, 0x400FD100);
pub const FLASH_PP_R = @intToPtr(*volatile u32, 0x400FDFC0);
pub const FLASH_SSIZE_R = @intToPtr(*volatile u32, 0x400FDFC4);
pub const FLASH_CONF_R = @intToPtr(*volatile u32, 0x400FDFC8);
pub const FLASH_ROMSWMAP_R = @intToPtr(*volatile u32, 0x400FDFCC);
pub const FLASH_DMASZ_R = @intToPtr(*volatile u32, 0x400FDFD0);
pub const FLASH_DMAST_R = @intToPtr(*volatile u32, 0x400FDFD4);
pub const FLASH_RVP_R = @intToPtr(*volatile u32, 0x400FE0D4);
pub const FLASH_BOOTCFG_R = @intToPtr(*volatile u32, 0x400FE1D0);
pub const FLASH_USERREG0_R = @intToPtr(*volatile u32, 0x400FE1E0);
pub const FLASH_USERREG1_R = @intToPtr(*volatile u32, 0x400FE1E4);
pub const FLASH_USERREG2_R = @intToPtr(*volatile u32, 0x400FE1E8);
pub const FLASH_USERREG3_R = @intToPtr(*volatile u32, 0x400FE1EC);
pub const FLASH_FMPRE0_R = @intToPtr(*volatile u32, 0x400FE200);
pub const FLASH_FMPRE1_R = @intToPtr(*volatile u32, 0x400FE204);
pub const FLASH_FMPRE2_R = @intToPtr(*volatile u32, 0x400FE208);
pub const FLASH_FMPRE3_R = @intToPtr(*volatile u32, 0x400FE20C);
pub const FLASH_FMPRE4_R = @intToPtr(*volatile u32, 0x400FE210);
pub const FLASH_FMPRE5_R = @intToPtr(*volatile u32, 0x400FE214);
pub const FLASH_FMPRE6_R = @intToPtr(*volatile u32, 0x400FE218);
pub const FLASH_FMPRE7_R = @intToPtr(*volatile u32, 0x400FE21C);
pub const FLASH_FMPRE8_R = @intToPtr(*volatile u32, 0x400FE220);
pub const FLASH_FMPRE9_R = @intToPtr(*volatile u32, 0x400FE224);
pub const FLASH_FMPRE10_R = @intToPtr(*volatile u32, 0x400FE228);
pub const FLASH_FMPRE11_R = @intToPtr(*volatile u32, 0x400FE22C);
pub const FLASH_FMPRE12_R = @intToPtr(*volatile u32, 0x400FE230);
pub const FLASH_FMPRE13_R = @intToPtr(*volatile u32, 0x400FE234);
pub const FLASH_FMPRE14_R = @intToPtr(*volatile u32, 0x400FE238);
pub const FLASH_FMPRE15_R = @intToPtr(*volatile u32, 0x400FE23C);
pub const FLASH_FMPPE0_R = @intToPtr(*volatile u32, 0x400FE400);
pub const FLASH_FMPPE1_R = @intToPtr(*volatile u32, 0x400FE404);
pub const FLASH_FMPPE2_R = @intToPtr(*volatile u32, 0x400FE408);
pub const FLASH_FMPPE3_R = @intToPtr(*volatile u32, 0x400FE40C);
pub const FLASH_FMPPE4_R = @intToPtr(*volatile u32, 0x400FE410);
pub const FLASH_FMPPE5_R = @intToPtr(*volatile u32, 0x400FE414);
pub const FLASH_FMPPE6_R = @intToPtr(*volatile u32, 0x400FE418);
pub const FLASH_FMPPE7_R = @intToPtr(*volatile u32, 0x400FE41C);
pub const FLASH_FMPPE8_R = @intToPtr(*volatile u32, 0x400FE420);
pub const FLASH_FMPPE9_R = @intToPtr(*volatile u32, 0x400FE424);
pub const FLASH_FMPPE10_R = @intToPtr(*volatile u32, 0x400FE428);
pub const FLASH_FMPPE11_R = @intToPtr(*volatile u32, 0x400FE42C);
pub const FLASH_FMPPE12_R = @intToPtr(*volatile u32, 0x400FE430);
pub const FLASH_FMPPE13_R = @intToPtr(*volatile u32, 0x400FE434);
pub const FLASH_FMPPE14_R = @intToPtr(*volatile u32, 0x400FE438);
pub const FLASH_FMPPE15_R = @intToPtr(*volatile u32, 0x400FE43C);

//*****************************************************************************
//
// System Control registers (SYSCTL)
//
//*****************************************************************************
pub const SYSCTL_DID0_R = @intToPtr(*volatile u32, 0x400FE000);
pub const SYSCTL_DID1_R = @intToPtr(*volatile u32, 0x400FE004);
pub const SYSCTL_PTBOCTL_R = @intToPtr(*volatile u32, 0x400FE038);
pub const SYSCTL_RIS_R = @intToPtr(*volatile u32, 0x400FE050);
pub const SYSCTL_IMC_R = @intToPtr(*volatile u32, 0x400FE054);
pub const SYSCTL_MISC_R = @intToPtr(*volatile u32, 0x400FE058);
pub const SYSCTL_RESC_R = @intToPtr(*volatile u32, 0x400FE05C);
pub const SYSCTL_PWRTC_R = @intToPtr(*volatile u32, 0x400FE060);
pub const SYSCTL_NMIC_R = @intToPtr(*volatile u32, 0x400FE064);
pub const SYSCTL_MOSCCTL_R = @intToPtr(*volatile u32, 0x400FE07C);
pub const SYSCTL_RSCLKCFG_R = @intToPtr(*volatile u32, 0x400FE0B0);
pub const SYSCTL_MEMTIM0_R = @intToPtr(*volatile u32, 0x400FE0C0);
pub const SYSCTL_ALTCLKCFG_R = @intToPtr(*volatile u32, 0x400FE138);
pub const SYSCTL_DSCLKCFG_R = @intToPtr(*volatile u32, 0x400FE144);
pub const SYSCTL_DIVSCLK_R = @intToPtr(*volatile u32, 0x400FE148);
pub const SYSCTL_SYSPROP_R = @intToPtr(*volatile u32, 0x400FE14C);
pub const SYSCTL_PIOSCCAL_R = @intToPtr(*volatile u32, 0x400FE150);
pub const SYSCTL_PIOSCSTAT_R = @intToPtr(*volatile u32, 0x400FE154);
pub const SYSCTL_PLLFREQ0_R = @intToPtr(*volatile u32, 0x400FE160);
pub const SYSCTL_PLLFREQ1_R = @intToPtr(*volatile u32, 0x400FE164);
pub const SYSCTL_PLLSTAT_R = @intToPtr(*volatile u32, 0x400FE168);
pub const SYSCTL_SLPPWRCFG_R = @intToPtr(*volatile u32, 0x400FE188);
pub const SYSCTL_DSLPPWRCFG_R = @intToPtr(*volatile u32, 0x400FE18C);
pub const SYSCTL_NVMSTAT_R = @intToPtr(*volatile u32, 0x400FE1A0);
pub const SYSCTL_LDOSPCTL_R = @intToPtr(*volatile u32, 0x400FE1B4);
pub const SYSCTL_LDODPCTL_R = @intToPtr(*volatile u32, 0x400FE1BC);
pub const SYSCTL_RESBEHAVCTL_R = @intToPtr(*volatile u32, 0x400FE1D8);
pub const SYSCTL_HSSR_R = @intToPtr(*volatile u32, 0x400FE1F4);
pub const SYSCTL_USBPDS_R = @intToPtr(*volatile u32, 0x400FE280);
pub const SYSCTL_USBMPC_R = @intToPtr(*volatile u32, 0x400FE284);
pub const SYSCTL_EMACPDS_R = @intToPtr(*volatile u32, 0x400FE288);
pub const SYSCTL_EMACMPC_R = @intToPtr(*volatile u32, 0x400FE28C);
pub const SYSCTL_PPWD_R = @intToPtr(*volatile u32, 0x400FE300);
pub const SYSCTL_PPTIMER_R = @intToPtr(*volatile u32, 0x400FE304);
pub const SYSCTL_PPGPIO_R = @intToPtr(*volatile u32, 0x400FE308);
pub const SYSCTL_PPDMA_R = @intToPtr(*volatile u32, 0x400FE30C);
pub const SYSCTL_PPEPI_R = @intToPtr(*volatile u32, 0x400FE310);
pub const SYSCTL_PPHIB_R = @intToPtr(*volatile u32, 0x400FE314);
pub const SYSCTL_PPUART_R = @intToPtr(*volatile u32, 0x400FE318);
pub const SYSCTL_PPSSI_R = @intToPtr(*volatile u32, 0x400FE31C);
pub const SYSCTL_PPI2C_R = @intToPtr(*volatile u32, 0x400FE320);
pub const SYSCTL_PPUSB_R = @intToPtr(*volatile u32, 0x400FE328);
pub const SYSCTL_PPEPHY_R = @intToPtr(*volatile u32, 0x400FE330);
pub const SYSCTL_PPCAN_R = @intToPtr(*volatile u32, 0x400FE334);
pub const SYSCTL_PPADC_R = @intToPtr(*volatile u32, 0x400FE338);
pub const SYSCTL_PPACMP_R = @intToPtr(*volatile u32, 0x400FE33C);
pub const SYSCTL_PPPWM_R = @intToPtr(*volatile u32, 0x400FE340);
pub const SYSCTL_PPQEI_R = @intToPtr(*volatile u32, 0x400FE344);
pub const SYSCTL_PPLPC_R = @intToPtr(*volatile u32, 0x400FE348);
pub const SYSCTL_PPPECI_R = @intToPtr(*volatile u32, 0x400FE350);
pub const SYSCTL_PPFAN_R = @intToPtr(*volatile u32, 0x400FE354);
pub const SYSCTL_PPEEPROM_R = @intToPtr(*volatile u32, 0x400FE358);
pub const SYSCTL_PPWTIMER_R = @intToPtr(*volatile u32, 0x400FE35C);
pub const SYSCTL_PPRTS_R = @intToPtr(*volatile u32, 0x400FE370);
pub const SYSCTL_PPCCM_R = @intToPtr(*volatile u32, 0x400FE374);
pub const SYSCTL_PPLCD_R = @intToPtr(*volatile u32, 0x400FE390);
pub const SYSCTL_PPOWIRE_R = @intToPtr(*volatile u32, 0x400FE398);
pub const SYSCTL_PPEMAC_R = @intToPtr(*volatile u32, 0x400FE39C);
pub const SYSCTL_PPHIM_R = @intToPtr(*volatile u32, 0x400FE3A4);
pub const SYSCTL_SRWD_R = @intToPtr(*volatile u32, 0x400FE500);
pub const SYSCTL_SRTIMER_R = @intToPtr(*volatile u32, 0x400FE504);
pub const SYSCTL_SRGPIO_R = @intToPtr(*volatile u32, 0x400FE508);
pub const SYSCTL_SRDMA_R = @intToPtr(*volatile u32, 0x400FE50C);
pub const SYSCTL_SREPI_R = @intToPtr(*volatile u32, 0x400FE510);
pub const SYSCTL_SRHIB_R = @intToPtr(*volatile u32, 0x400FE514);
pub const SYSCTL_SRUART_R = @intToPtr(*volatile u32, 0x400FE518);
pub const SYSCTL_SRSSI_R = @intToPtr(*volatile u32, 0x400FE51C);
pub const SYSCTL_SRI2C_R = @intToPtr(*volatile u32, 0x400FE520);
pub const SYSCTL_SRUSB_R = @intToPtr(*volatile u32, 0x400FE528);
pub const SYSCTL_SRCAN_R = @intToPtr(*volatile u32, 0x400FE534);
pub const SYSCTL_SRADC_R = @intToPtr(*volatile u32, 0x400FE538);
pub const SYSCTL_SRACMP_R = @intToPtr(*volatile u32, 0x400FE53C);
pub const SYSCTL_SRPWM_R = @intToPtr(*volatile u32, 0x400FE540);
pub const SYSCTL_SRQEI_R = @intToPtr(*volatile u32, 0x400FE544);
pub const SYSCTL_SREEPROM_R = @intToPtr(*volatile u32, 0x400FE558);
pub const SYSCTL_SRCCM_R = @intToPtr(*volatile u32, 0x400FE574);
pub const SYSCTL_SREMAC_R = @intToPtr(*volatile u32, 0x400FE59C);
pub const SYSCTL_RCGCWD_R = @intToPtr(*volatile u32, 0x400FE600);
pub const SYSCTL_RCGCTIMER_R = @intToPtr(*volatile u32, 0x400FE604);
pub const SYSCTL_RCGCGPIO_R = @intToPtr(*volatile u32, 0x400FE608);
pub const SYSCTL_RCGCDMA_R = @intToPtr(*volatile u32, 0x400FE60C);
pub const SYSCTL_RCGCEPI_R = @intToPtr(*volatile u32, 0x400FE610);
pub const SYSCTL_RCGCHIB_R = @intToPtr(*volatile u32, 0x400FE614);
pub const SYSCTL_RCGCUART_R = @intToPtr(*volatile u32, 0x400FE618);
pub const SYSCTL_RCGCSSI_R = @intToPtr(*volatile u32, 0x400FE61C);
pub const SYSCTL_RCGCI2C_R = @intToPtr(*volatile u32, 0x400FE620);
pub const SYSCTL_RCGCUSB_R = @intToPtr(*volatile u32, 0x400FE628);
pub const SYSCTL_RCGCCAN_R = @intToPtr(*volatile u32, 0x400FE634);
pub const SYSCTL_RCGCADC_R = @intToPtr(*volatile u32, 0x400FE638);
pub const SYSCTL_RCGCACMP_R = @intToPtr(*volatile u32, 0x400FE63C);
pub const SYSCTL_RCGCPWM_R = @intToPtr(*volatile u32, 0x400FE640);
pub const SYSCTL_RCGCQEI_R = @intToPtr(*volatile u32, 0x400FE644);
pub const SYSCTL_RCGCEEPROM_R = @intToPtr(*volatile u32, 0x400FE658);
pub const SYSCTL_RCGCCCM_R = @intToPtr(*volatile u32, 0x400FE674);
pub const SYSCTL_RCGCEMAC_R = @intToPtr(*volatile u32, 0x400FE69C);
pub const SYSCTL_SCGCWD_R = @intToPtr(*volatile u32, 0x400FE700);
pub const SYSCTL_SCGCTIMER_R = @intToPtr(*volatile u32, 0x400FE704);
pub const SYSCTL_SCGCGPIO_R = @intToPtr(*volatile u32, 0x400FE708);
pub const SYSCTL_SCGCDMA_R = @intToPtr(*volatile u32, 0x400FE70C);
pub const SYSCTL_SCGCEPI_R = @intToPtr(*volatile u32, 0x400FE710);
pub const SYSCTL_SCGCHIB_R = @intToPtr(*volatile u32, 0x400FE714);
pub const SYSCTL_SCGCUART_R = @intToPtr(*volatile u32, 0x400FE718);
pub const SYSCTL_SCGCSSI_R = @intToPtr(*volatile u32, 0x400FE71C);
pub const SYSCTL_SCGCI2C_R = @intToPtr(*volatile u32, 0x400FE720);
pub const SYSCTL_SCGCUSB_R = @intToPtr(*volatile u32, 0x400FE728);
pub const SYSCTL_SCGCCAN_R = @intToPtr(*volatile u32, 0x400FE734);
pub const SYSCTL_SCGCADC_R = @intToPtr(*volatile u32, 0x400FE738);
pub const SYSCTL_SCGCACMP_R = @intToPtr(*volatile u32, 0x400FE73C);
pub const SYSCTL_SCGCPWM_R = @intToPtr(*volatile u32, 0x400FE740);
pub const SYSCTL_SCGCQEI_R = @intToPtr(*volatile u32, 0x400FE744);
pub const SYSCTL_SCGCEEPROM_R = @intToPtr(*volatile u32, 0x400FE758);
pub const SYSCTL_SCGCCCM_R = @intToPtr(*volatile u32, 0x400FE774);
pub const SYSCTL_SCGCEMAC_R = @intToPtr(*volatile u32, 0x400FE79C);
pub const SYSCTL_DCGCWD_R = @intToPtr(*volatile u32, 0x400FE800);
pub const SYSCTL_DCGCTIMER_R = @intToPtr(*volatile u32, 0x400FE804);
pub const SYSCTL_DCGCGPIO_R = @intToPtr(*volatile u32, 0x400FE808);
pub const SYSCTL_DCGCDMA_R = @intToPtr(*volatile u32, 0x400FE80C);
pub const SYSCTL_DCGCEPI_R = @intToPtr(*volatile u32, 0x400FE810);
pub const SYSCTL_DCGCHIB_R = @intToPtr(*volatile u32, 0x400FE814);
pub const SYSCTL_DCGCUART_R = @intToPtr(*volatile u32, 0x400FE818);
pub const SYSCTL_DCGCSSI_R = @intToPtr(*volatile u32, 0x400FE81C);
pub const SYSCTL_DCGCI2C_R = @intToPtr(*volatile u32, 0x400FE820);
pub const SYSCTL_DCGCUSB_R = @intToPtr(*volatile u32, 0x400FE828);
pub const SYSCTL_DCGCCAN_R = @intToPtr(*volatile u32, 0x400FE834);
pub const SYSCTL_DCGCADC_R = @intToPtr(*volatile u32, 0x400FE838);
pub const SYSCTL_DCGCACMP_R = @intToPtr(*volatile u32, 0x400FE83C);
pub const SYSCTL_DCGCPWM_R = @intToPtr(*volatile u32, 0x400FE840);
pub const SYSCTL_DCGCQEI_R = @intToPtr(*volatile u32, 0x400FE844);
pub const SYSCTL_DCGCEEPROM_R = @intToPtr(*volatile u32, 0x400FE858);
pub const SYSCTL_DCGCCCM_R = @intToPtr(*volatile u32, 0x400FE874);
pub const SYSCTL_DCGCEMAC_R = @intToPtr(*volatile u32, 0x400FE89C);
pub const SYSCTL_PCWD_R = @intToPtr(*volatile u32, 0x400FE900);
pub const SYSCTL_PCTIMER_R = @intToPtr(*volatile u32, 0x400FE904);
pub const SYSCTL_PCGPIO_R = @intToPtr(*volatile u32, 0x400FE908);
pub const SYSCTL_PCDMA_R = @intToPtr(*volatile u32, 0x400FE90C);
pub const SYSCTL_PCEPI_R = @intToPtr(*volatile u32, 0x400FE910);
pub const SYSCTL_PCHIB_R = @intToPtr(*volatile u32, 0x400FE914);
pub const SYSCTL_PCUART_R = @intToPtr(*volatile u32, 0x400FE918);
pub const SYSCTL_PCSSI_R = @intToPtr(*volatile u32, 0x400FE91C);
pub const SYSCTL_PCI2C_R = @intToPtr(*volatile u32, 0x400FE920);
pub const SYSCTL_PCUSB_R = @intToPtr(*volatile u32, 0x400FE928);
pub const SYSCTL_PCCAN_R = @intToPtr(*volatile u32, 0x400FE934);
pub const SYSCTL_PCADC_R = @intToPtr(*volatile u32, 0x400FE938);
pub const SYSCTL_PCACMP_R = @intToPtr(*volatile u32, 0x400FE93C);
pub const SYSCTL_PCPWM_R = @intToPtr(*volatile u32, 0x400FE940);
pub const SYSCTL_PCQEI_R = @intToPtr(*volatile u32, 0x400FE944);
pub const SYSCTL_PCEEPROM_R = @intToPtr(*volatile u32, 0x400FE958);
pub const SYSCTL_PCCCM_R = @intToPtr(*volatile u32, 0x400FE974);
pub const SYSCTL_PCEMAC_R = @intToPtr(*volatile u32, 0x400FE99C);
pub const SYSCTL_PRWD_R = @intToPtr(*volatile u32, 0x400FEA00);
pub const SYSCTL_PRTIMER_R = @intToPtr(*volatile u32, 0x400FEA04);
pub const SYSCTL_PRGPIO_R = @intToPtr(*volatile u32, 0x400FEA08);
pub const SYSCTL_PRDMA_R = @intToPtr(*volatile u32, 0x400FEA0C);
pub const SYSCTL_PREPI_R = @intToPtr(*volatile u32, 0x400FEA10);
pub const SYSCTL_PRHIB_R = @intToPtr(*volatile u32, 0x400FEA14);
pub const SYSCTL_PRUART_R = @intToPtr(*volatile u32, 0x400FEA18);
pub const SYSCTL_PRSSI_R = @intToPtr(*volatile u32, 0x400FEA1C);
pub const SYSCTL_PRI2C_R = @intToPtr(*volatile u32, 0x400FEA20);
pub const SYSCTL_PRUSB_R = @intToPtr(*volatile u32, 0x400FEA28);
pub const SYSCTL_PRCAN_R = @intToPtr(*volatile u32, 0x400FEA34);
pub const SYSCTL_PRADC_R = @intToPtr(*volatile u32, 0x400FEA38);
pub const SYSCTL_PRACMP_R = @intToPtr(*volatile u32, 0x400FEA3C);
pub const SYSCTL_PRPWM_R = @intToPtr(*volatile u32, 0x400FEA40);
pub const SYSCTL_PRQEI_R = @intToPtr(*volatile u32, 0x400FEA44);
pub const SYSCTL_PREEPROM_R = @intToPtr(*volatile u32, 0x400FEA58);
pub const SYSCTL_PRCCM_R = @intToPtr(*volatile u32, 0x400FEA74);
pub const SYSCTL_PREMAC_R = @intToPtr(*volatile u32, 0x400FEA9C);
pub const SYSCTL_CCMCGREQ_R = @intToPtr(*volatile u32, 0x44030204);

//*****************************************************************************
//
// Micro Direct Memory Access registers (UDMA)
//
//*****************************************************************************
pub const UDMA_STAT_R = @intToPtr(*volatile u32, 0x400FF000);
pub const UDMA_CFG_R = @intToPtr(*volatile u32, 0x400FF004);
pub const UDMA_CTLBASE_R = @intToPtr(*volatile u32, 0x400FF008);
pub const UDMA_ALTBASE_R = @intToPtr(*volatile u32, 0x400FF00C);
pub const UDMA_WAITSTAT_R = @intToPtr(*volatile u32, 0x400FF010);
pub const UDMA_SWREQ_R = @intToPtr(*volatile u32, 0x400FF014);
pub const UDMA_USEBURSTSET_R = @intToPtr(*volatile u32, 0x400FF018);
pub const UDMA_USEBURSTCLR_R = @intToPtr(*volatile u32, 0x400FF01C);
pub const UDMA_REQMASKSET_R = @intToPtr(*volatile u32, 0x400FF020);
pub const UDMA_REQMASKCLR_R = @intToPtr(*volatile u32, 0x400FF024);
pub const UDMA_ENASET_R = @intToPtr(*volatile u32, 0x400FF028);
pub const UDMA_ENACLR_R = @intToPtr(*volatile u32, 0x400FF02C);
pub const UDMA_ALTSET_R = @intToPtr(*volatile u32, 0x400FF030);
pub const UDMA_ALTCLR_R = @intToPtr(*volatile u32, 0x400FF034);
pub const UDMA_PRIOSET_R = @intToPtr(*volatile u32, 0x400FF038);
pub const UDMA_PRIOCLR_R = @intToPtr(*volatile u32, 0x400FF03C);
pub const UDMA_ERRCLR_R = @intToPtr(*volatile u32, 0x400FF04C);
pub const UDMA_CHASGN_R = @intToPtr(*volatile u32, 0x400FF500);
pub const UDMA_CHMAP0_R = @intToPtr(*volatile u32, 0x400FF510);
pub const UDMA_CHMAP1_R = @intToPtr(*volatile u32, 0x400FF514);
pub const UDMA_CHMAP2_R = @intToPtr(*volatile u32, 0x400FF518);
pub const UDMA_CHMAP3_R = @intToPtr(*volatile u32, 0x400FF51C);

//*****************************************************************************
//
// Micro Direct Memory Access (uDMA) offsets (UDMA)
//
//*****************************************************************************
pub const UDMA_SRCENDP = usize(0x00000000);  // DMA Channel Source Address End
                                            // Pointer
pub const UDMA_DSTENDP = usize(0x00000004);  // DMA Channel Destination Address
                                            // End Pointer
pub const UDMA_CHCTL = usize(0x00000008);  // DMA Channel Control Word

//*****************************************************************************
//
// EC registers (CCM0)
//
//*****************************************************************************
pub const CCM0_CRCCTRL_R = @intToPtr(*volatile u32, 0x44030400);
pub const CCM0_CRCSEED_R = @intToPtr(*volatile u32, 0x44030410);
pub const CCM0_CRCDIN_R = @intToPtr(*volatile u32, 0x44030414);
pub const CCM0_CRCRSLTPP_R = @intToPtr(*volatile u32, 0x44030418);

//*****************************************************************************
//
// SHA/MD5 registers (SHAMD5)
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_A_R = @intToPtr(*volatile u32, 0x44034000);
pub const SHAMD5_ODIGEST_B_R = @intToPtr(*volatile u32, 0x44034004);
pub const SHAMD5_ODIGEST_C_R = @intToPtr(*volatile u32, 0x44034008);
pub const SHAMD5_ODIGEST_D_R = @intToPtr(*volatile u32, 0x4403400C);
pub const SHAMD5_ODIGEST_E_R = @intToPtr(*volatile u32, 0x44034010);
pub const SHAMD5_ODIGEST_F_R = @intToPtr(*volatile u32, 0x44034014);
pub const SHAMD5_ODIGEST_G_R = @intToPtr(*volatile u32, 0x44034018);
pub const SHAMD5_ODIGEST_H_R = @intToPtr(*volatile u32, 0x4403401C);
pub const SHAMD5_IDIGEST_A_R = @intToPtr(*volatile u32, 0x44034020);
pub const SHAMD5_IDIGEST_B_R = @intToPtr(*volatile u32, 0x44034024);
pub const SHAMD5_IDIGEST_C_R = @intToPtr(*volatile u32, 0x44034028);
pub const SHAMD5_IDIGEST_D_R = @intToPtr(*volatile u32, 0x4403402C);
pub const SHAMD5_IDIGEST_E_R = @intToPtr(*volatile u32, 0x44034030);
pub const SHAMD5_IDIGEST_F_R = @intToPtr(*volatile u32, 0x44034034);
pub const SHAMD5_IDIGEST_G_R = @intToPtr(*volatile u32, 0x44034038);
pub const SHAMD5_IDIGEST_H_R = @intToPtr(*volatile u32, 0x4403403C);
pub const SHAMD5_DIGEST_COUNT_R = @intToPtr(*volatile u32, 0x44034040);
pub const SHAMD5_MODE_R = @intToPtr(*volatile u32, 0x44034044);
pub const SHAMD5_LENGTH_R = @intToPtr(*volatile u32, 0x44034048);
pub const SHAMD5_DATA_0_IN_R = @intToPtr(*volatile u32, 0x44034080);
pub const SHAMD5_DATA_1_IN_R = @intToPtr(*volatile u32, 0x44034084);
pub const SHAMD5_DATA_2_IN_R = @intToPtr(*volatile u32, 0x44034088);
pub const SHAMD5_DATA_3_IN_R = @intToPtr(*volatile u32, 0x4403408C);
pub const SHAMD5_DATA_4_IN_R = @intToPtr(*volatile u32, 0x44034090);
pub const SHAMD5_DATA_5_IN_R = @intToPtr(*volatile u32, 0x44034094);
pub const SHAMD5_DATA_6_IN_R = @intToPtr(*volatile u32, 0x44034098);
pub const SHAMD5_DATA_7_IN_R = @intToPtr(*volatile u32, 0x4403409C);
pub const SHAMD5_DATA_8_IN_R = @intToPtr(*volatile u32, 0x440340A0);
pub const SHAMD5_DATA_9_IN_R = @intToPtr(*volatile u32, 0x440340A4);
pub const SHAMD5_DATA_10_IN_R = @intToPtr(*volatile u32, 0x440340A8);
pub const SHAMD5_DATA_11_IN_R = @intToPtr(*volatile u32, 0x440340AC);
pub const SHAMD5_DATA_12_IN_R = @intToPtr(*volatile u32, 0x440340B0);
pub const SHAMD5_DATA_13_IN_R = @intToPtr(*volatile u32, 0x440340B4);
pub const SHAMD5_DATA_14_IN_R = @intToPtr(*volatile u32, 0x440340B8);
pub const SHAMD5_DATA_15_IN_R = @intToPtr(*volatile u32, 0x440340BC);
pub const SHAMD5_REVISION_R = @intToPtr(*volatile u32, 0x44034100);
pub const SHAMD5_SYSCONFIG_R = @intToPtr(*volatile u32, 0x44034110);
pub const SHAMD5_SYSSTATUS_R = @intToPtr(*volatile u32, 0x44034114);
pub const SHAMD5_IRQSTATUS_R = @intToPtr(*volatile u32, 0x44034118);
pub const SHAMD5_IRQENABLE_R = @intToPtr(*volatile u32, 0x4403411C);
pub const SHAMD5_DMAIM_R = @intToPtr(*volatile u32, 0x144030010);
pub const SHAMD5_DMARIS_R = @intToPtr(*volatile u32, 0x144030014);
pub const SHAMD5_DMAMIS_R = @intToPtr(*volatile u32, 0x144030018);
pub const SHAMD5_DMAIC_R = @intToPtr(*volatile u32, 0x14403001C);

//*****************************************************************************
//
// AES registers (AES)
//
//*****************************************************************************
pub const AES_KEY2_6_R = @intToPtr(*volatile u32, 0x44036000);
pub const AES_KEY2_7_R = @intToPtr(*volatile u32, 0x44036004);
pub const AES_KEY2_4_R = @intToPtr(*volatile u32, 0x44036008);
pub const AES_KEY2_5_R = @intToPtr(*volatile u32, 0x4403600C);
pub const AES_KEY2_2_R = @intToPtr(*volatile u32, 0x44036010);
pub const AES_KEY2_3_R = @intToPtr(*volatile u32, 0x44036014);
pub const AES_KEY2_0_R = @intToPtr(*volatile u32, 0x44036018);
pub const AES_KEY2_1_R = @intToPtr(*volatile u32, 0x4403601C);
pub const AES_KEY1_6_R = @intToPtr(*volatile u32, 0x44036020);
pub const AES_KEY1_7_R = @intToPtr(*volatile u32, 0x44036024);
pub const AES_KEY1_4_R = @intToPtr(*volatile u32, 0x44036028);
pub const AES_KEY1_5_R = @intToPtr(*volatile u32, 0x4403602C);
pub const AES_KEY1_2_R = @intToPtr(*volatile u32, 0x44036030);
pub const AES_KEY1_3_R = @intToPtr(*volatile u32, 0x44036034);
pub const AES_KEY1_0_R = @intToPtr(*volatile u32, 0x44036038);
pub const AES_KEY1_1_R = @intToPtr(*volatile u32, 0x4403603C);
pub const AES_IV_IN_0_R = @intToPtr(*volatile u32, 0x44036040);
pub const AES_IV_IN_1_R = @intToPtr(*volatile u32, 0x44036044);
pub const AES_IV_IN_2_R = @intToPtr(*volatile u32, 0x44036048);
pub const AES_IV_IN_3_R = @intToPtr(*volatile u32, 0x4403604C);
pub const AES_CTRL_R = @intToPtr(*volatile u32, 0x44036050);
pub const AES_C_LENGTH_0_R = @intToPtr(*volatile u32, 0x44036054);
pub const AES_C_LENGTH_1_R = @intToPtr(*volatile u32, 0x44036058);
pub const AES_AUTH_LENGTH_R = @intToPtr(*volatile u32, 0x4403605C);
pub const AES_DATA_IN_0_R = @intToPtr(*volatile u32, 0x44036060);
pub const AES_DATA_IN_1_R = @intToPtr(*volatile u32, 0x44036064);
pub const AES_DATA_IN_2_R = @intToPtr(*volatile u32, 0x44036068);
pub const AES_DATA_IN_3_R = @intToPtr(*volatile u32, 0x4403606C);
pub const AES_TAG_OUT_0_R = @intToPtr(*volatile u32, 0x44036070);
pub const AES_TAG_OUT_1_R = @intToPtr(*volatile u32, 0x44036074);
pub const AES_TAG_OUT_2_R = @intToPtr(*volatile u32, 0x44036078);
pub const AES_TAG_OUT_3_R = @intToPtr(*volatile u32, 0x4403607C);
pub const AES_REVISION_R = @intToPtr(*volatile u32, 0x44036080);
pub const AES_SYSCONFIG_R = @intToPtr(*volatile u32, 0x44036084);
pub const AES_SYSSTATUS_R = @intToPtr(*volatile u32, 0x44036088);
pub const AES_IRQSTATUS_R = @intToPtr(*volatile u32, 0x4403608C);
pub const AES_IRQENABLE_R = @intToPtr(*volatile u32, 0x44036090);
pub const AES_DIRTYBITS_R = @intToPtr(*volatile u32, 0x44036094);
pub const AES_DMAIM_R = @intToPtr(*volatile u32, 0x144030020);
pub const AES_DMARIS_R = @intToPtr(*volatile u32, 0x144030024);
pub const AES_DMAMIS_R = @intToPtr(*volatile u32, 0x144030028);
pub const AES_DMAIC_R = @intToPtr(*volatile u32, 0x14403002C);

//*****************************************************************************
//
// DES registers (DES)
//
//*****************************************************************************
pub const DES_KEY3_L_R = @intToPtr(*volatile u32, 0x44038000);
pub const DES_KEY3_H_R = @intToPtr(*volatile u32, 0x44038004);
pub const DES_KEY2_L_R = @intToPtr(*volatile u32, 0x44038008);
pub const DES_KEY2_H_R = @intToPtr(*volatile u32, 0x4403800C);
pub const DES_KEY1_L_R = @intToPtr(*volatile u32, 0x44038010);
pub const DES_KEY1_H_R = @intToPtr(*volatile u32, 0x44038014);
pub const DES_IV_L_R = @intToPtr(*volatile u32, 0x44038018);
pub const DES_IV_H_R = @intToPtr(*volatile u32, 0x4403801C);
pub const DES_CTRL_R = @intToPtr(*volatile u32, 0x44038020);
pub const DES_LENGTH_R = @intToPtr(*volatile u32, 0x44038024);
pub const DES_DATA_L_R = @intToPtr(*volatile u32, 0x44038028);
pub const DES_DATA_H_R = @intToPtr(*volatile u32, 0x4403802C);
pub const DES_REVISION_R = @intToPtr(*volatile u32, 0x44038030);
pub const DES_SYSCONFIG_R = @intToPtr(*volatile u32, 0x44038034);
pub const DES_SYSSTATUS_R = @intToPtr(*volatile u32, 0x44038038);
pub const DES_IRQSTATUS_R = @intToPtr(*volatile u32, 0x4403803C);
pub const DES_IRQENABLE_R = @intToPtr(*volatile u32, 0x44038040);
pub const DES_DIRTYBITS_R = @intToPtr(*volatile u32, 0x44038044);
pub const DES_DMAIM_R = @intToPtr(*volatile u32, 0x144030030);
pub const DES_DMARIS_R = @intToPtr(*volatile u32, 0x144030034);
pub const DES_DMAMIS_R = @intToPtr(*volatile u32, 0x144030038);
pub const DES_DMAIC_R = @intToPtr(*volatile u32, 0x14403003C);

//*****************************************************************************
//
// NVIC registers (NVIC)
//
//*****************************************************************************
pub const NVIC_ACTLR_R = @intToPtr(*volatile u32, 0xE000E008);
pub const NVIC_ST_CTRL_R = @intToPtr(*volatile u32, 0xE000E010);
pub const NVIC_ST_RELOAD_R = @intToPtr(*volatile u32, 0xE000E014);
pub const NVIC_ST_CURRENT_R = @intToPtr(*volatile u32, 0xE000E018);
pub const NVIC_EN0_R = @intToPtr(*volatile u32, 0xE000E100);
pub const NVIC_EN1_R = @intToPtr(*volatile u32, 0xE000E104);
pub const NVIC_EN2_R = @intToPtr(*volatile u32, 0xE000E108);
pub const NVIC_EN3_R = @intToPtr(*volatile u32, 0xE000E10C);
pub const NVIC_DIS0_R = @intToPtr(*volatile u32, 0xE000E180);
pub const NVIC_DIS1_R = @intToPtr(*volatile u32, 0xE000E184);
pub const NVIC_DIS2_R = @intToPtr(*volatile u32, 0xE000E188);
pub const NVIC_DIS3_R = @intToPtr(*volatile u32, 0xE000E18C);
pub const NVIC_PEND0_R = @intToPtr(*volatile u32, 0xE000E200);
pub const NVIC_PEND1_R = @intToPtr(*volatile u32, 0xE000E204);
pub const NVIC_PEND2_R = @intToPtr(*volatile u32, 0xE000E208);
pub const NVIC_PEND3_R = @intToPtr(*volatile u32, 0xE000E20C);
pub const NVIC_UNPEND0_R = @intToPtr(*volatile u32, 0xE000E280);
pub const NVIC_UNPEND1_R = @intToPtr(*volatile u32, 0xE000E284);
pub const NVIC_UNPEND2_R = @intToPtr(*volatile u32, 0xE000E288);
pub const NVIC_UNPEND3_R = @intToPtr(*volatile u32, 0xE000E28C);
pub const NVIC_ACTIVE0_R = @intToPtr(*volatile u32, 0xE000E300);
pub const NVIC_ACTIVE1_R = @intToPtr(*volatile u32, 0xE000E304);
pub const NVIC_ACTIVE2_R = @intToPtr(*volatile u32, 0xE000E308);
pub const NVIC_ACTIVE3_R = @intToPtr(*volatile u32, 0xE000E30C);
pub const NVIC_PRI0_R = @intToPtr(*volatile u32, 0xE000E400);
pub const NVIC_PRI1_R = @intToPtr(*volatile u32, 0xE000E404);
pub const NVIC_PRI2_R = @intToPtr(*volatile u32, 0xE000E408);
pub const NVIC_PRI3_R = @intToPtr(*volatile u32, 0xE000E40C);
pub const NVIC_PRI4_R = @intToPtr(*volatile u32, 0xE000E410);
pub const NVIC_PRI5_R = @intToPtr(*volatile u32, 0xE000E414);
pub const NVIC_PRI6_R = @intToPtr(*volatile u32, 0xE000E418);
pub const NVIC_PRI7_R = @intToPtr(*volatile u32, 0xE000E41C);
pub const NVIC_PRI8_R = @intToPtr(*volatile u32, 0xE000E420);
pub const NVIC_PRI9_R = @intToPtr(*volatile u32, 0xE000E424);
pub const NVIC_PRI10_R = @intToPtr(*volatile u32, 0xE000E428);
pub const NVIC_PRI11_R = @intToPtr(*volatile u32, 0xE000E42C);
pub const NVIC_PRI12_R = @intToPtr(*volatile u32, 0xE000E430);
pub const NVIC_PRI13_R = @intToPtr(*volatile u32, 0xE000E434);
pub const NVIC_PRI14_R = @intToPtr(*volatile u32, 0xE000E438);
pub const NVIC_PRI15_R = @intToPtr(*volatile u32, 0xE000E43C);
pub const NVIC_PRI16_R = @intToPtr(*volatile u32, 0xE000E440);
pub const NVIC_PRI17_R = @intToPtr(*volatile u32, 0xE000E444);
pub const NVIC_PRI18_R = @intToPtr(*volatile u32, 0xE000E448);
pub const NVIC_PRI19_R = @intToPtr(*volatile u32, 0xE000E44C);
pub const NVIC_PRI20_R = @intToPtr(*volatile u32, 0xE000E450);
pub const NVIC_PRI21_R = @intToPtr(*volatile u32, 0xE000E454);
pub const NVIC_PRI22_R = @intToPtr(*volatile u32, 0xE000E458);
pub const NVIC_PRI23_R = @intToPtr(*volatile u32, 0xE000E45C);
pub const NVIC_PRI24_R = @intToPtr(*volatile u32, 0xE000E460);
pub const NVIC_PRI25_R = @intToPtr(*volatile u32, 0xE000E464);
pub const NVIC_PRI26_R = @intToPtr(*volatile u32, 0xE000E468);
pub const NVIC_PRI27_R = @intToPtr(*volatile u32, 0xE000E46C);
pub const NVIC_PRI28_R = @intToPtr(*volatile u32, 0xE000E470);
pub const NVIC_CPUID_R = @intToPtr(*volatile u32, 0xE000ED00);
pub const NVIC_INT_CTRL_R = @intToPtr(*volatile u32, 0xE000ED04);
pub const NVIC_VTABLE_R = @intToPtr(*volatile u32, 0xE000ED08);
pub const NVIC_APINT_R = @intToPtr(*volatile u32, 0xE000ED0C);
pub const NVIC_SYS_CTRL_R = @intToPtr(*volatile u32, 0xE000ED10);
pub const NVIC_CFG_CTRL_R = @intToPtr(*volatile u32, 0xE000ED14);
pub const NVIC_SYS_PRI1_R = @intToPtr(*volatile u32, 0xE000ED18);
pub const NVIC_SYS_PRI2_R = @intToPtr(*volatile u32, 0xE000ED1C);
pub const NVIC_SYS_PRI3_R = @intToPtr(*volatile u32, 0xE000ED20);
pub const NVIC_SYS_HND_CTRL_R = @intToPtr(*volatile u32, 0xE000ED24);
pub const NVIC_FAULT_STAT_R = @intToPtr(*volatile u32, 0xE000ED28);
pub const NVIC_HFAULT_STAT_R = @intToPtr(*volatile u32, 0xE000ED2C);
pub const NVIC_DEBUG_STAT_R = @intToPtr(*volatile u32, 0xE000ED30);
pub const NVIC_MM_ADDR_R = @intToPtr(*volatile u32, 0xE000ED34);
pub const NVIC_FAULT_ADDR_R = @intToPtr(*volatile u32, 0xE000ED38);
pub const NVIC_CPAC_R = @intToPtr(*volatile u32, 0xE000ED88);
pub const NVIC_MPU_TYPE_R = @intToPtr(*volatile u32, 0xE000ED90);
pub const NVIC_MPU_CTRL_R = @intToPtr(*volatile u32, 0xE000ED94);
pub const NVIC_MPU_NUMBER_R = @intToPtr(*volatile u32, 0xE000ED98);
pub const NVIC_MPU_BASE_R = @intToPtr(*volatile u32, 0xE000ED9C);
pub const NVIC_MPU_ATTR_R = @intToPtr(*volatile u32, 0xE000EDA0);
pub const NVIC_MPU_BASE1_R = @intToPtr(*volatile u32, 0xE000EDA4);
pub const NVIC_MPU_ATTR1_R = @intToPtr(*volatile u32, 0xE000EDA8);
pub const NVIC_MPU_BASE2_R = @intToPtr(*volatile u32, 0xE000EDAC);
pub const NVIC_MPU_ATTR2_R = @intToPtr(*volatile u32, 0xE000EDB0);
pub const NVIC_MPU_BASE3_R = @intToPtr(*volatile u32, 0xE000EDB4);
pub const NVIC_MPU_ATTR3_R = @intToPtr(*volatile u32, 0xE000EDB8);
pub const NVIC_DBG_CTRL_R = @intToPtr(*volatile u32, 0xE000EDF0);
pub const NVIC_DBG_XFER_R = @intToPtr(*volatile u32, 0xE000EDF4);
pub const NVIC_DBG_DATA_R = @intToPtr(*volatile u32, 0xE000EDF8);
pub const NVIC_DBG_INT_R = @intToPtr(*volatile u32, 0xE000EDFC);
pub const NVIC_SW_TRIG_R = @intToPtr(*volatile u32, 0xE000EF00);
pub const NVIC_FPCC_R = @intToPtr(*volatile u32, 0xE000EF34);
pub const NVIC_FPCA_R = @intToPtr(*volatile u32, 0xE000EF38);
pub const NVIC_FPDSC_R = @intToPtr(*volatile u32, 0xE000EF3C);

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOAD register.
//
//*****************************************************************************
pub const WDT_LOAD_M = usize(0xFFFFFFFF);  // Watchdog Load Value
pub const WDT_LOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_VALUE register.
//
//*****************************************************************************
pub const WDT_VALUE_M = usize(0xFFFFFFFF);  // Watchdog Value
pub const WDT_VALUE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_CTL register.
//
//*****************************************************************************
pub const WDT_CTL_WRC = usize(0x80000000);  // Write Complete
pub const WDT_CTL_INTTYPE = usize(0x00000004);  // Watchdog Interrupt Type
pub const WDT_CTL_RESEN = usize(0x00000002);  // Watchdog Reset Enable
pub const WDT_CTL_INTEN = usize(0x00000001);  // Watchdog Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_ICR register.
//
//*****************************************************************************
pub const WDT_ICR_M = usize(0xFFFFFFFF);  // Watchdog Interrupt Clear
pub const WDT_ICR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_RIS register.
//
//*****************************************************************************
pub const WDT_RIS_WDTRIS = usize(0x00000001);  // Watchdog Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_MIS register.
//
//*****************************************************************************
pub const WDT_MIS_WDTMIS = usize(0x00000001);  // Watchdog Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_TEST register.
//
//*****************************************************************************
pub const WDT_TEST_STALL = usize(0x00000100);  // Watchdog Stall Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOCK register.
//
//*****************************************************************************
pub const WDT_LOCK_M = usize(0xFFFFFFFF);  // Watchdog Lock
pub const WDT_LOCK_UNLOCKED = usize(0x00000000);  // Unlocked
pub const WDT_LOCK_LOCKED = usize(0x00000001);  // Locked
pub const WDT_LOCK_UNLOCK = usize(0x1ACCE551);  // Unlocks the watchdog timer

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR0 register.
//
//*****************************************************************************
pub const SSI_CR0_SCR_M = usize(0x0000FF00);  // SSI Serial Clock Rate
pub const SSI_CR0_SPH = usize(0x00000080);  // SSI Serial Clock Phase
pub const SSI_CR0_SPO = usize(0x00000040);  // SSI Serial Clock Polarity
pub const SSI_CR0_FRF_M = usize(0x00000030);  // SSI Frame Format Select
pub const SSI_CR0_FRF_MOTO = usize(0x00000000);  // Freescale SPI Frame Format
pub const SSI_CR0_FRF_TI = usize(0x00000010);  // Synchronous Serial Frame Format
pub const SSI_CR0_DSS_M = usize(0x0000000F);  // SSI Data Size Select
pub const SSI_CR0_DSS_4 = usize(0x00000003);  // 4-bit data
pub const SSI_CR0_DSS_5 = usize(0x00000004);  // 5-bit data
pub const SSI_CR0_DSS_6 = usize(0x00000005);  // 6-bit data
pub const SSI_CR0_DSS_7 = usize(0x00000006);  // 7-bit data
pub const SSI_CR0_DSS_8 = usize(0x00000007);  // 8-bit data
pub const SSI_CR0_DSS_9 = usize(0x00000008);  // 9-bit data
pub const SSI_CR0_DSS_10 = usize(0x00000009);  // 10-bit data
pub const SSI_CR0_DSS_11 = usize(0x0000000A);  // 11-bit data
pub const SSI_CR0_DSS_12 = usize(0x0000000B);  // 12-bit data
pub const SSI_CR0_DSS_13 = usize(0x0000000C);  // 13-bit data
pub const SSI_CR0_DSS_14 = usize(0x0000000D);  // 14-bit data
pub const SSI_CR0_DSS_15 = usize(0x0000000E);  // 15-bit data
pub const SSI_CR0_DSS_16 = usize(0x0000000F);  // 16-bit data
pub const SSI_CR0_SCR_S = usize(8);

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR1 register.
//
//*****************************************************************************
pub const SSI_CR1_EOM = usize(0x00000800);  // Stop Frame (End of Message)
pub const SSI_CR1_FSSHLDFRM = usize(0x00000400);  // FSS Hold Frame
pub const SSI_CR1_HSCLKEN = usize(0x00000200);  // High Speed Clock Enable
pub const SSI_CR1_DIR = usize(0x00000100);  // SSI Direction of Operation
pub const SSI_CR1_MODE_M = usize(0x000000C0);  // SSI Mode
pub const SSI_CR1_MODE_LEGACY = usize(0x00000000);  // Legacy SSI mode
pub const SSI_CR1_MODE_BI = usize(0x00000040);  // Bi-SSI mode
pub const SSI_CR1_MODE_QUAD = usize(0x00000080);  // Quad-SSI Mode
pub const SSI_CR1_MODE_ADVANCED = usize(0x000000C0);  // Advanced SSI Mode with 8-bit
                                            // packet size
pub const SSI_CR1_EOT = usize(0x00000010);  // End of Transmission
pub const SSI_CR1_MS = usize(0x00000004);  // SSI Master/Slave Select
pub const SSI_CR1_SSE = usize(0x00000002);  // SSI Synchronous Serial Port
                                            // Enable
pub const SSI_CR1_LBM = usize(0x00000001);  // SSI Loopback Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DR register.
//
//*****************************************************************************
pub const SSI_DR_DATA_M = usize(0x0000FFFF);  // SSI Receive/Transmit Data
pub const SSI_DR_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_SR register.
//
//*****************************************************************************
pub const SSI_SR_BSY = usize(0x00000010);  // SSI Busy Bit
pub const SSI_SR_RFF = usize(0x00000008);  // SSI Receive FIFO Full
pub const SSI_SR_RNE = usize(0x00000004);  // SSI Receive FIFO Not Empty
pub const SSI_SR_TNF = usize(0x00000002);  // SSI Transmit FIFO Not Full
pub const SSI_SR_TFE = usize(0x00000001);  // SSI Transmit FIFO Empty

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CPSR register.
//
//*****************************************************************************
pub const SSI_CPSR_CPSDVSR_M = usize(0x000000FF);  // SSI Clock Prescale Divisor
pub const SSI_CPSR_CPSDVSR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_IM register.
//
//*****************************************************************************
pub const SSI_IM_EOTIM = usize(0x00000040);  // End of Transmit Interrupt Mask
pub const SSI_IM_DMATXIM = usize(0x00000020);  // SSI Transmit DMA Interrupt Mask
pub const SSI_IM_DMARXIM = usize(0x00000010);  // SSI Receive DMA Interrupt Mask
pub const SSI_IM_TXIM = usize(0x00000008);  // SSI Transmit FIFO Interrupt Mask
pub const SSI_IM_RXIM = usize(0x00000004);  // SSI Receive FIFO Interrupt Mask
pub const SSI_IM_RTIM = usize(0x00000002);  // SSI Receive Time-Out Interrupt
                                            // Mask
pub const SSI_IM_RORIM = usize(0x00000001);  // SSI Receive Overrun Interrupt
                                            // Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_RIS register.
//
//*****************************************************************************
pub const SSI_RIS_EOTRIS = usize(0x00000040);  // End of Transmit Raw Interrupt
                                            // Status
pub const SSI_RIS_DMATXRIS = usize(0x00000020);  // SSI Transmit DMA Raw Interrupt
                                            // Status
pub const SSI_RIS_DMARXRIS = usize(0x00000010);  // SSI Receive DMA Raw Interrupt
                                            // Status
pub const SSI_RIS_TXRIS = usize(0x00000008);  // SSI Transmit FIFO Raw Interrupt
                                            // Status
pub const SSI_RIS_RXRIS = usize(0x00000004);  // SSI Receive FIFO Raw Interrupt
                                            // Status
pub const SSI_RIS_RTRIS = usize(0x00000002);  // SSI Receive Time-Out Raw
                                            // Interrupt Status
pub const SSI_RIS_RORRIS = usize(0x00000001);  // SSI Receive Overrun Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_MIS register.
//
//*****************************************************************************
pub const SSI_MIS_EOTMIS = usize(0x00000040);  // End of Transmit Masked Interrupt
                                            // Status
pub const SSI_MIS_DMATXMIS = usize(0x00000020);  // SSI Transmit DMA Masked
                                            // Interrupt Status
pub const SSI_MIS_DMARXMIS = usize(0x00000010);  // SSI Receive DMA Masked Interrupt
                                            // Status
pub const SSI_MIS_TXMIS = usize(0x00000008);  // SSI Transmit FIFO Masked
                                            // Interrupt Status
pub const SSI_MIS_RXMIS = usize(0x00000004);  // SSI Receive FIFO Masked
                                            // Interrupt Status
pub const SSI_MIS_RTMIS = usize(0x00000002);  // SSI Receive Time-Out Masked
                                            // Interrupt Status
pub const SSI_MIS_RORMIS = usize(0x00000001);  // SSI Receive Overrun Masked
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_ICR register.
//
//*****************************************************************************
pub const SSI_ICR_EOTIC = usize(0x00000040);  // End of Transmit Interrupt Clear
pub const SSI_ICR_DMATXIC = usize(0x00000020);  // SSI Transmit DMA Interrupt Clear
pub const SSI_ICR_DMARXIC = usize(0x00000010);  // SSI Receive DMA Interrupt Clear
pub const SSI_ICR_RTIC = usize(0x00000002);  // SSI Receive Time-Out Interrupt
                                            // Clear
pub const SSI_ICR_RORIC = usize(0x00000001);  // SSI Receive Overrun Interrupt
                                            // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DMACTL register.
//
//*****************************************************************************
pub const SSI_DMACTL_TXDMAE = usize(0x00000002);  // Transmit DMA Enable
pub const SSI_DMACTL_RXDMAE = usize(0x00000001);  // Receive DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_PP register.
//
//*****************************************************************************
pub const SSI_PP_FSSHLDFRM = usize(0x00000008);  // FSS Hold Frame Capability
pub const SSI_PP_MODE_M = usize(0x00000006);  // Mode of Operation
pub const SSI_PP_MODE_LEGACY = usize(0x00000000);  // Legacy SSI mode
pub const SSI_PP_MODE_ADVBI = usize(0x00000002);  // Legacy mode, Advanced SSI mode
                                            // and Bi-SSI mode enabled
pub const SSI_PP_MODE_ADVBIQUAD = usize(0x00000004);  // Legacy mode, Advanced mode,
                                            // Bi-SSI and Quad-SSI mode enabled
pub const SSI_PP_HSCLK = usize(0x00000001);  // High Speed Capability

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CC register.
//
//*****************************************************************************
pub const SSI_CC_CS_M = usize(0x0000000F);  // SSI Baud Clock Source
pub const SSI_CC_CS_SYSPLL = usize(0x00000000);  // System clock (based on clock
                                            // source and divisor factor)
pub const SSI_CC_CS_PIOSC = usize(0x00000005);  // PIOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DR register.
//
//*****************************************************************************
pub const UART_DR_OE = usize(0x00000800);  // UART Overrun Error
pub const UART_DR_BE = usize(0x00000400);  // UART Break Error
pub const UART_DR_PE = usize(0x00000200);  // UART Parity Error
pub const UART_DR_FE = usize(0x00000100);  // UART Framing Error
pub const UART_DR_DATA_M = usize(0x000000FF);  // Data Transmitted or Received
pub const UART_DR_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RSR register.
//
//*****************************************************************************
pub const UART_RSR_OE = usize(0x00000008);  // UART Overrun Error
pub const UART_RSR_BE = usize(0x00000004);  // UART Break Error
pub const UART_RSR_PE = usize(0x00000002);  // UART Parity Error
pub const UART_RSR_FE = usize(0x00000001);  // UART Framing Error

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ECR register.
//
//*****************************************************************************
pub const UART_ECR_DATA_M = usize(0x000000FF);  // Error Clear
pub const UART_ECR_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FR register.
//
//*****************************************************************************
pub const UART_FR_RI = usize(0x00000100);  // Ring Indicator
pub const UART_FR_TXFE = usize(0x00000080);  // UART Transmit FIFO Empty
pub const UART_FR_RXFF = usize(0x00000040);  // UART Receive FIFO Full
pub const UART_FR_TXFF = usize(0x00000020);  // UART Transmit FIFO Full
pub const UART_FR_RXFE = usize(0x00000010);  // UART Receive FIFO Empty
pub const UART_FR_BUSY = usize(0x00000008);  // UART Busy
pub const UART_FR_DCD = usize(0x00000004);  // Data Carrier Detect
pub const UART_FR_DSR = usize(0x00000002);  // Data Set Ready
pub const UART_FR_CTS = usize(0x00000001);  // Clear To Send

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ILPR register.
//
//*****************************************************************************
pub const UART_ILPR_ILPDVSR_M = usize(0x000000FF);  // IrDA Low-Power Divisor
pub const UART_ILPR_ILPDVSR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IBRD register.
//
//*****************************************************************************
pub const UART_IBRD_DIVINT_M = usize(0x0000FFFF);  // Integer Baud-Rate Divisor
pub const UART_IBRD_DIVINT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FBRD register.
//
//*****************************************************************************
pub const UART_FBRD_DIVFRAC_M = usize(0x0000003F);  // Fractional Baud-Rate Divisor
pub const UART_FBRD_DIVFRAC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_LCRH register.
//
//*****************************************************************************
pub const UART_LCRH_SPS = usize(0x00000080);  // UART Stick Parity Select
pub const UART_LCRH_WLEN_M = usize(0x00000060);  // UART Word Length
pub const UART_LCRH_WLEN_5 = usize(0x00000000);  // 5 bits (default)
pub const UART_LCRH_WLEN_6 = usize(0x00000020);  // 6 bits
pub const UART_LCRH_WLEN_7 = usize(0x00000040);  // 7 bits
pub const UART_LCRH_WLEN_8 = usize(0x00000060);  // 8 bits
pub const UART_LCRH_FEN = usize(0x00000010);  // UART Enable FIFOs
pub const UART_LCRH_STP2 = usize(0x00000008);  // UART Two Stop Bits Select
pub const UART_LCRH_EPS = usize(0x00000004);  // UART Even Parity Select
pub const UART_LCRH_PEN = usize(0x00000002);  // UART Parity Enable
pub const UART_LCRH_BRK = usize(0x00000001);  // UART Send Break

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CTL register.
//
//*****************************************************************************
pub const UART_CTL_CTSEN = usize(0x00008000);  // Enable Clear To Send
pub const UART_CTL_RTSEN = usize(0x00004000);  // Enable Request to Send
pub const UART_CTL_RTS = usize(0x00000800);  // Request to Send
pub const UART_CTL_DTR = usize(0x00000400);  // Data Terminal Ready
pub const UART_CTL_RXE = usize(0x00000200);  // UART Receive Enable
pub const UART_CTL_TXE = usize(0x00000100);  // UART Transmit Enable
pub const UART_CTL_LBE = usize(0x00000080);  // UART Loop Back Enable
pub const UART_CTL_HSE = usize(0x00000020);  // High-Speed Enable
pub const UART_CTL_EOT = usize(0x00000010);  // End of Transmission
pub const UART_CTL_SMART = usize(0x00000008);  // ISO 7816 Smart Card Support
pub const UART_CTL_SIRLP = usize(0x00000004);  // UART SIR Low-Power Mode
pub const UART_CTL_SIREN = usize(0x00000002);  // UART SIR Enable
pub const UART_CTL_UARTEN = usize(0x00000001);  // UART Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IFLS register.
//
//*****************************************************************************
pub const UART_IFLS_RX_M = usize(0x00000038);  // UART Receive Interrupt FIFO
                                            // Level Select
pub const UART_IFLS_RX1_8 = usize(0x00000000);  // RX FIFO >= 1/8 full
pub const UART_IFLS_RX2_8 = usize(0x00000008);  // RX FIFO >= 1/4 full
pub const UART_IFLS_RX4_8 = usize(0x00000010);  // RX FIFO >= 1/2 full (default)
pub const UART_IFLS_RX6_8 = usize(0x00000018);  // RX FIFO >= 3/4 full
pub const UART_IFLS_RX7_8 = usize(0x00000020);  // RX FIFO >= 7/8 full
pub const UART_IFLS_TX_M = usize(0x00000007);  // UART Transmit Interrupt FIFO
                                            // Level Select
pub const UART_IFLS_TX1_8 = usize(0x00000000);  // TX FIFO <= 1/8 full
pub const UART_IFLS_TX2_8 = usize(0x00000001);  // TX FIFO <= 1/4 full
pub const UART_IFLS_TX4_8 = usize(0x00000002);  // TX FIFO <= 1/2 full (default)
pub const UART_IFLS_TX6_8 = usize(0x00000003);  // TX FIFO <= 3/4 full
pub const UART_IFLS_TX7_8 = usize(0x00000004);  // TX FIFO <= 7/8 full

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IM register.
//
//*****************************************************************************
pub const UART_IM_DMATXIM = usize(0x00020000);  // Transmit DMA Interrupt Mask
pub const UART_IM_DMARXIM = usize(0x00010000);  // Receive DMA Interrupt Mask
pub const UART_IM_9BITIM = usize(0x00001000);  // 9-Bit Mode Interrupt Mask
pub const UART_IM_EOTIM = usize(0x00000800);  // End of Transmission Interrupt
                                            // Mask
pub const UART_IM_OEIM = usize(0x00000400);  // UART Overrun Error Interrupt
                                            // Mask
pub const UART_IM_BEIM = usize(0x00000200);  // UART Break Error Interrupt Mask
pub const UART_IM_PEIM = usize(0x00000100);  // UART Parity Error Interrupt Mask
pub const UART_IM_FEIM = usize(0x00000080);  // UART Framing Error Interrupt
                                            // Mask
pub const UART_IM_RTIM = usize(0x00000040);  // UART Receive Time-Out Interrupt
                                            // Mask
pub const UART_IM_TXIM = usize(0x00000020);  // UART Transmit Interrupt Mask
pub const UART_IM_RXIM = usize(0x00000010);  // UART Receive Interrupt Mask
pub const UART_IM_DSRMIM = usize(0x00000008);  // UART Data Set Ready Modem
                                            // Interrupt Mask
pub const UART_IM_DCDMIM = usize(0x00000004);  // UART Data Carrier Detect Modem
                                            // Interrupt Mask
pub const UART_IM_CTSMIM = usize(0x00000002);  // UART Clear to Send Modem
                                            // Interrupt Mask
pub const UART_IM_RIMIM = usize(0x00000001);  // UART Ring Indicator Modem
                                            // Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RIS register.
//
//*****************************************************************************
pub const UART_RIS_DMATXRIS = usize(0x00020000);  // Transmit DMA Raw Interrupt
                                            // Status
pub const UART_RIS_DMARXRIS = usize(0x00010000);  // Receive DMA Raw Interrupt Status
pub const UART_RIS_9BITRIS = usize(0x00001000);  // 9-Bit Mode Raw Interrupt Status
pub const UART_RIS_EOTRIS = usize(0x00000800);  // End of Transmission Raw
                                            // Interrupt Status
pub const UART_RIS_OERIS = usize(0x00000400);  // UART Overrun Error Raw Interrupt
                                            // Status
pub const UART_RIS_BERIS = usize(0x00000200);  // UART Break Error Raw Interrupt
                                            // Status
pub const UART_RIS_PERIS = usize(0x00000100);  // UART Parity Error Raw Interrupt
                                            // Status
pub const UART_RIS_FERIS = usize(0x00000080);  // UART Framing Error Raw Interrupt
                                            // Status
pub const UART_RIS_RTRIS = usize(0x00000040);  // UART Receive Time-Out Raw
                                            // Interrupt Status
pub const UART_RIS_TXRIS = usize(0x00000020);  // UART Transmit Raw Interrupt
                                            // Status
pub const UART_RIS_RXRIS = usize(0x00000010);  // UART Receive Raw Interrupt
                                            // Status
pub const UART_RIS_DSRRIS = usize(0x00000008);  // UART Data Set Ready Modem Raw
                                            // Interrupt Status
pub const UART_RIS_DCDRIS = usize(0x00000004);  // UART Data Carrier Detect Modem
                                            // Raw Interrupt Status
pub const UART_RIS_CTSRIS = usize(0x00000002);  // UART Clear to Send Modem Raw
                                            // Interrupt Status
pub const UART_RIS_RIRIS = usize(0x00000001);  // UART Ring Indicator Modem Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_MIS register.
//
//*****************************************************************************
pub const UART_MIS_DMATXMIS = usize(0x00020000);  // Transmit DMA Masked Interrupt
                                            // Status
pub const UART_MIS_DMARXMIS = usize(0x00010000);  // Receive DMA Masked Interrupt
                                            // Status
pub const UART_MIS_9BITMIS = usize(0x00001000);  // 9-Bit Mode Masked Interrupt
                                            // Status
pub const UART_MIS_EOTMIS = usize(0x00000800);  // End of Transmission Masked
                                            // Interrupt Status
pub const UART_MIS_OEMIS = usize(0x00000400);  // UART Overrun Error Masked
                                            // Interrupt Status
pub const UART_MIS_BEMIS = usize(0x00000200);  // UART Break Error Masked
                                            // Interrupt Status
pub const UART_MIS_PEMIS = usize(0x00000100);  // UART Parity Error Masked
                                            // Interrupt Status
pub const UART_MIS_FEMIS = usize(0x00000080);  // UART Framing Error Masked
                                            // Interrupt Status
pub const UART_MIS_RTMIS = usize(0x00000040);  // UART Receive Time-Out Masked
                                            // Interrupt Status
pub const UART_MIS_TXMIS = usize(0x00000020);  // UART Transmit Masked Interrupt
                                            // Status
pub const UART_MIS_RXMIS = usize(0x00000010);  // UART Receive Masked Interrupt
                                            // Status
pub const UART_MIS_DSRMIS = usize(0x00000008);  // UART Data Set Ready Modem Masked
                                            // Interrupt Status
pub const UART_MIS_DCDMIS = usize(0x00000004);  // UART Data Carrier Detect Modem
                                            // Masked Interrupt Status
pub const UART_MIS_CTSMIS = usize(0x00000002);  // UART Clear to Send Modem Masked
                                            // Interrupt Status
pub const UART_MIS_RIMIS = usize(0x00000001);  // UART Ring Indicator Modem Masked
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ICR register.
//
//*****************************************************************************
pub const UART_ICR_DMATXIC = usize(0x00020000);  // Transmit DMA Interrupt Clear
pub const UART_ICR_DMARXIC = usize(0x00010000);  // Receive DMA Interrupt Clear
pub const UART_ICR_9BITIC = usize(0x00001000);  // 9-Bit Mode Interrupt Clear
pub const UART_ICR_EOTIC = usize(0x00000800);  // End of Transmission Interrupt
                                            // Clear
pub const UART_ICR_OEIC = usize(0x00000400);  // Overrun Error Interrupt Clear
pub const UART_ICR_BEIC = usize(0x00000200);  // Break Error Interrupt Clear
pub const UART_ICR_PEIC = usize(0x00000100);  // Parity Error Interrupt Clear
pub const UART_ICR_FEIC = usize(0x00000080);  // Framing Error Interrupt Clear
pub const UART_ICR_RTIC = usize(0x00000040);  // Receive Time-Out Interrupt Clear
pub const UART_ICR_TXIC = usize(0x00000020);  // Transmit Interrupt Clear
pub const UART_ICR_RXIC = usize(0x00000010);  // Receive Interrupt Clear
pub const UART_ICR_DSRMIC = usize(0x00000008);  // UART Data Set Ready Modem
                                            // Interrupt Clear
pub const UART_ICR_DCDMIC = usize(0x00000004);  // UART Data Carrier Detect Modem
                                            // Interrupt Clear
pub const UART_ICR_CTSMIC = usize(0x00000002);  // UART Clear to Send Modem
                                            // Interrupt Clear
pub const UART_ICR_RIMIC = usize(0x00000001);  // UART Ring Indicator Modem
                                            // Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DMACTL register.
//
//*****************************************************************************
pub const UART_DMACTL_DMAERR = usize(0x00000004);  // DMA on Error
pub const UART_DMACTL_TXDMAE = usize(0x00000002);  // Transmit DMA Enable
pub const UART_DMACTL_RXDMAE = usize(0x00000001);  // Receive DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITADDR
// register.
//
//*****************************************************************************
pub const UART_9BITADDR_9BITEN = usize(0x00008000);  // Enable 9-Bit Mode
pub const UART_9BITADDR_ADDR_M = usize(0x000000FF);  // Self Address for 9-Bit Mode
pub const UART_9BITADDR_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITAMASK
// register.
//
//*****************************************************************************
pub const UART_9BITAMASK_MASK_M = usize(0x000000FF);  // Self Address Mask for 9-Bit Mode
pub const UART_9BITAMASK_MASK_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_PP register.
//
//*****************************************************************************
pub const UART_PP_MSE = usize(0x00000008);  // Modem Support Extended
pub const UART_PP_MS = usize(0x00000004);  // Modem Support
pub const UART_PP_NB = usize(0x00000002);  // 9-Bit Support
pub const UART_PP_SC = usize(0x00000001);  // Smart Card Support

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CC register.
//
//*****************************************************************************
pub const UART_CC_CS_M = usize(0x0000000F);  // UART Baud Clock Source
pub const UART_CC_CS_SYSCLK = usize(0x00000000);  // System clock (based on clock
                                            // source and divisor factor)
pub const UART_CC_CS_PIOSC = usize(0x00000005);  // PIOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MSA register.
//
//*****************************************************************************
pub const I2C_MSA_SA_M = usize(0x000000FE);  // I2C Slave Address
pub const I2C_MSA_RS = usize(0x00000001);  // Receive not send
pub const I2C_MSA_SA_S = usize(1);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCS register.
//
//*****************************************************************************
pub const I2C_MCS_ACTDMARX = usize(0x80000000);  // DMA RX Active Status
pub const I2C_MCS_ACTDMATX = usize(0x40000000);  // DMA TX Active Status
pub const I2C_MCS_CLKTO = usize(0x00000080);  // Clock Timeout Error
pub const I2C_MCS_BURST = usize(0x00000040);  // Burst Enable
pub const I2C_MCS_BUSBSY = usize(0x00000040);  // Bus Busy
pub const I2C_MCS_IDLE = usize(0x00000020);  // I2C Idle
pub const I2C_MCS_QCMD = usize(0x00000020);  // Quick Command
pub const I2C_MCS_ARBLST = usize(0x00000010);  // Arbitration Lost
pub const I2C_MCS_HS = usize(0x00000010);  // High-Speed Enable
pub const I2C_MCS_ACK = usize(0x00000008);  // Data Acknowledge Enable
pub const I2C_MCS_DATACK = usize(0x00000008);  // Acknowledge Data
pub const I2C_MCS_ADRACK = usize(0x00000004);  // Acknowledge Address
pub const I2C_MCS_STOP = usize(0x00000004);  // Generate STOP
pub const I2C_MCS_ERROR = usize(0x00000002);  // Error
pub const I2C_MCS_START = usize(0x00000002);  // Generate START
pub const I2C_MCS_RUN = usize(0x00000001);  // I2C Master Enable
pub const I2C_MCS_BUSY = usize(0x00000001);  // I2C Busy

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MDR register.
//
//*****************************************************************************
pub const I2C_MDR_DATA_M = usize(0x000000FF);  // This byte contains the data
                                            // transferred during a transaction
pub const I2C_MDR_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MTPR register.
//
//*****************************************************************************
pub const I2C_MTPR_PULSEL_M = usize(0x00070000);  // Glitch Suppression Pulse Width
pub const I2C_MTPR_PULSEL_BYPASS = usize(0x00000000);  // Bypass
pub const I2C_MTPR_PULSEL_1 = usize(0x00010000);  // 1 clock
pub const I2C_MTPR_PULSEL_2 = usize(0x00020000);  // 2 clocks
pub const I2C_MTPR_PULSEL_3 = usize(0x00030000);  // 3 clocks
pub const I2C_MTPR_PULSEL_4 = usize(0x00040000);  // 4 clocks
pub const I2C_MTPR_PULSEL_8 = usize(0x00050000);  // 8 clocks
pub const I2C_MTPR_PULSEL_16 = usize(0x00060000);  // 16 clocks
pub const I2C_MTPR_PULSEL_31 = usize(0x00070000);  // 31 clocks
pub const I2C_MTPR_HS = usize(0x00000080);  // High-Speed Enable
pub const I2C_MTPR_TPR_M = usize(0x0000007F);  // Timer Period
pub const I2C_MTPR_TPR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MIMR register.
//
//*****************************************************************************
pub const I2C_MIMR_RXFFIM = usize(0x00000800);  // Receive FIFO Full Interrupt Mask
pub const I2C_MIMR_TXFEIM = usize(0x00000400);  // Transmit FIFO Empty Interrupt
                                            // Mask
pub const I2C_MIMR_RXIM = usize(0x00000200);  // Receive FIFO Request Interrupt
                                            // Mask
pub const I2C_MIMR_TXIM = usize(0x00000100);  // Transmit FIFO Request Interrupt
                                            // Mask
pub const I2C_MIMR_ARBLOSTIM = usize(0x00000080);  // Arbitration Lost Interrupt Mask
pub const I2C_MIMR_STOPIM = usize(0x00000040);  // STOP Detection Interrupt Mask
pub const I2C_MIMR_STARTIM = usize(0x00000020);  // START Detection Interrupt Mask
pub const I2C_MIMR_NACKIM = usize(0x00000010);  // Address/Data NACK Interrupt Mask
pub const I2C_MIMR_DMATXIM = usize(0x00000008);  // Transmit DMA Interrupt Mask
pub const I2C_MIMR_DMARXIM = usize(0x00000004);  // Receive DMA Interrupt Mask
pub const I2C_MIMR_CLKIM = usize(0x00000002);  // Clock Timeout Interrupt Mask
pub const I2C_MIMR_IM = usize(0x00000001);  // Master Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MRIS register.
//
//*****************************************************************************
pub const I2C_MRIS_RXFFRIS = usize(0x00000800);  // Receive FIFO Full Raw Interrupt
                                            // Status
pub const I2C_MRIS_TXFERIS = usize(0x00000400);  // Transmit FIFO Empty Raw
                                            // Interrupt Status
pub const I2C_MRIS_RXRIS = usize(0x00000200);  // Receive FIFO Request Raw
                                            // Interrupt Status
pub const I2C_MRIS_TXRIS = usize(0x00000100);  // Transmit Request Raw Interrupt
                                            // Status
pub const I2C_MRIS_ARBLOSTRIS = usize(0x00000080);  // Arbitration Lost Raw Interrupt
                                            // Status
pub const I2C_MRIS_STOPRIS = usize(0x00000040);  // STOP Detection Raw Interrupt
                                            // Status
pub const I2C_MRIS_STARTRIS = usize(0x00000020);  // START Detection Raw Interrupt
                                            // Status
pub const I2C_MRIS_NACKRIS = usize(0x00000010);  // Address/Data NACK Raw Interrupt
                                            // Status
pub const I2C_MRIS_DMATXRIS = usize(0x00000008);  // Transmit DMA Raw Interrupt
                                            // Status
pub const I2C_MRIS_DMARXRIS = usize(0x00000004);  // Receive DMA Raw Interrupt Status
pub const I2C_MRIS_CLKRIS = usize(0x00000002);  // Clock Timeout Raw Interrupt
                                            // Status
pub const I2C_MRIS_RIS = usize(0x00000001);  // Master Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MMIS register.
//
//*****************************************************************************
pub const I2C_MMIS_RXFFMIS = usize(0x00000800);  // Receive FIFO Full Interrupt Mask
pub const I2C_MMIS_TXFEMIS = usize(0x00000400);  // Transmit FIFO Empty Interrupt
                                            // Mask
pub const I2C_MMIS_RXMIS = usize(0x00000200);  // Receive FIFO Request Interrupt
                                            // Mask
pub const I2C_MMIS_TXMIS = usize(0x00000100);  // Transmit Request Interrupt Mask
pub const I2C_MMIS_ARBLOSTMIS = usize(0x00000080);  // Arbitration Lost Interrupt Mask
pub const I2C_MMIS_STOPMIS = usize(0x00000040);  // STOP Detection Interrupt Mask
pub const I2C_MMIS_STARTMIS = usize(0x00000020);  // START Detection Interrupt Mask
pub const I2C_MMIS_NACKMIS = usize(0x00000010);  // Address/Data NACK Interrupt Mask
pub const I2C_MMIS_DMATXMIS = usize(0x00000008);  // Transmit DMA Interrupt Status
pub const I2C_MMIS_DMARXMIS = usize(0x00000004);  // Receive DMA Interrupt Status
pub const I2C_MMIS_CLKMIS = usize(0x00000002);  // Clock Timeout Masked Interrupt
                                            // Status
pub const I2C_MMIS_MIS = usize(0x00000001);  // Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MICR register.
//
//*****************************************************************************
pub const I2C_MICR_RXFFIC = usize(0x00000800);  // Receive FIFO Full Interrupt
                                            // Clear
pub const I2C_MICR_TXFEIC = usize(0x00000400);  // Transmit FIFO Empty Interrupt
                                            // Clear
pub const I2C_MICR_RXIC = usize(0x00000200);  // Receive FIFO Request Interrupt
                                            // Clear
pub const I2C_MICR_TXIC = usize(0x00000100);  // Transmit FIFO Request Interrupt
                                            // Clear
pub const I2C_MICR_ARBLOSTIC = usize(0x00000080);  // Arbitration Lost Interrupt Clear
pub const I2C_MICR_STOPIC = usize(0x00000040);  // STOP Detection Interrupt Clear
pub const I2C_MICR_STARTIC = usize(0x00000020);  // START Detection Interrupt Clear
pub const I2C_MICR_NACKIC = usize(0x00000010);  // Address/Data NACK Interrupt
                                            // Clear
pub const I2C_MICR_DMATXIC = usize(0x00000008);  // Transmit DMA Interrupt Clear
pub const I2C_MICR_DMARXIC = usize(0x00000004);  // Receive DMA Interrupt Clear
pub const I2C_MICR_CLKIC = usize(0x00000002);  // Clock Timeout Interrupt Clear
pub const I2C_MICR_IC = usize(0x00000001);  // Master Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCR register.
//
//*****************************************************************************
pub const I2C_MCR_SFE = usize(0x00000020);  // I2C Slave Function Enable
pub const I2C_MCR_MFE = usize(0x00000010);  // I2C Master Function Enable
pub const I2C_MCR_LPBK = usize(0x00000001);  // I2C Loopback

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCLKOCNT register.
//
//*****************************************************************************
pub const I2C_MCLKOCNT_CNTL_M = usize(0x000000FF);  // I2C Master Count
pub const I2C_MCLKOCNT_CNTL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBMON register.
//
//*****************************************************************************
pub const I2C_MBMON_SDA = usize(0x00000002);  // I2C SDA Status
pub const I2C_MBMON_SCL = usize(0x00000001);  // I2C SCL Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBLEN register.
//
//*****************************************************************************
pub const I2C_MBLEN_CNTL_M = usize(0x000000FF);  // I2C Burst Length
pub const I2C_MBLEN_CNTL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBCNT register.
//
//*****************************************************************************
pub const I2C_MBCNT_CNTL_M = usize(0x000000FF);  // I2C Master Burst Count
pub const I2C_MBCNT_CNTL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR register.
//
//*****************************************************************************
pub const I2C_SOAR_OAR_M = usize(0x0000007F);  // I2C Slave Own Address
pub const I2C_SOAR_OAR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SCSR register.
//
//*****************************************************************************
pub const I2C_SCSR_ACTDMARX = usize(0x80000000);  // DMA RX Active Status
pub const I2C_SCSR_ACTDMATX = usize(0x40000000);  // DMA TX Active Status
pub const I2C_SCSR_QCMDRW = usize(0x00000020);  // Quick Command Read / Write
pub const I2C_SCSR_QCMDST = usize(0x00000010);  // Quick Command Status
pub const I2C_SCSR_OAR2SEL = usize(0x00000008);  // OAR2 Address Matched
pub const I2C_SCSR_FBR = usize(0x00000004);  // First Byte Received
pub const I2C_SCSR_RXFIFO = usize(0x00000004);  // RX FIFO Enable
pub const I2C_SCSR_TXFIFO = usize(0x00000002);  // TX FIFO Enable
pub const I2C_SCSR_TREQ = usize(0x00000002);  // Transmit Request
pub const I2C_SCSR_DA = usize(0x00000001);  // Device Active
pub const I2C_SCSR_RREQ = usize(0x00000001);  // Receive Request

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SDR register.
//
//*****************************************************************************
pub const I2C_SDR_DATA_M = usize(0x000000FF);  // Data for Transfer
pub const I2C_SDR_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SIMR register.
//
//*****************************************************************************
pub const I2C_SIMR_RXFFIM = usize(0x00000100);  // Receive FIFO Full Interrupt Mask
pub const I2C_SIMR_TXFEIM = usize(0x00000080);  // Transmit FIFO Empty Interrupt
                                            // Mask
pub const I2C_SIMR_RXIM = usize(0x00000040);  // Receive FIFO Request Interrupt
                                            // Mask
pub const I2C_SIMR_TXIM = usize(0x00000020);  // Transmit FIFO Request Interrupt
                                            // Mask
pub const I2C_SIMR_DMATXIM = usize(0x00000010);  // Transmit DMA Interrupt Mask
pub const I2C_SIMR_DMARXIM = usize(0x00000008);  // Receive DMA Interrupt Mask
pub const I2C_SIMR_STOPIM = usize(0x00000004);  // Stop Condition Interrupt Mask
pub const I2C_SIMR_STARTIM = usize(0x00000002);  // Start Condition Interrupt Mask
pub const I2C_SIMR_DATAIM = usize(0x00000001);  // Data Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SRIS register.
//
//*****************************************************************************
pub const I2C_SRIS_RXFFRIS = usize(0x00000100);  // Receive FIFO Full Raw Interrupt
                                            // Status
pub const I2C_SRIS_TXFERIS = usize(0x00000080);  // Transmit FIFO Empty Raw
                                            // Interrupt Status
pub const I2C_SRIS_RXRIS = usize(0x00000040);  // Receive FIFO Request Raw
                                            // Interrupt Status
pub const I2C_SRIS_TXRIS = usize(0x00000020);  // Transmit Request Raw Interrupt
                                            // Status
pub const I2C_SRIS_DMATXRIS = usize(0x00000010);  // Transmit DMA Raw Interrupt
                                            // Status
pub const I2C_SRIS_DMARXRIS = usize(0x00000008);  // Receive DMA Raw Interrupt Status
pub const I2C_SRIS_STOPRIS = usize(0x00000004);  // Stop Condition Raw Interrupt
                                            // Status
pub const I2C_SRIS_STARTRIS = usize(0x00000002);  // Start Condition Raw Interrupt
                                            // Status
pub const I2C_SRIS_DATARIS = usize(0x00000001);  // Data Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SMIS register.
//
//*****************************************************************************
pub const I2C_SMIS_RXFFMIS = usize(0x00000100);  // Receive FIFO Full Interrupt Mask
pub const I2C_SMIS_TXFEMIS = usize(0x00000080);  // Transmit FIFO Empty Interrupt
                                            // Mask
pub const I2C_SMIS_RXMIS = usize(0x00000040);  // Receive FIFO Request Interrupt
                                            // Mask
pub const I2C_SMIS_TXMIS = usize(0x00000020);  // Transmit FIFO Request Interrupt
                                            // Mask
pub const I2C_SMIS_DMATXMIS = usize(0x00000010);  // Transmit DMA Masked Interrupt
                                            // Status
pub const I2C_SMIS_DMARXMIS = usize(0x00000008);  // Receive DMA Masked Interrupt
                                            // Status
pub const I2C_SMIS_STOPMIS = usize(0x00000004);  // Stop Condition Masked Interrupt
                                            // Status
pub const I2C_SMIS_STARTMIS = usize(0x00000002);  // Start Condition Masked Interrupt
                                            // Status
pub const I2C_SMIS_DATAMIS = usize(0x00000001);  // Data Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SICR register.
//
//*****************************************************************************
pub const I2C_SICR_RXFFIC = usize(0x00000100);  // Receive FIFO Full Interrupt Mask
pub const I2C_SICR_TXFEIC = usize(0x00000080);  // Transmit FIFO Empty Interrupt
                                            // Mask
pub const I2C_SICR_RXIC = usize(0x00000040);  // Receive Request Interrupt Mask
pub const I2C_SICR_TXIC = usize(0x00000020);  // Transmit Request Interrupt Mask
pub const I2C_SICR_DMATXIC = usize(0x00000010);  // Transmit DMA Interrupt Clear
pub const I2C_SICR_DMARXIC = usize(0x00000008);  // Receive DMA Interrupt Clear
pub const I2C_SICR_STOPIC = usize(0x00000004);  // Stop Condition Interrupt Clear
pub const I2C_SICR_STARTIC = usize(0x00000002);  // Start Condition Interrupt Clear
pub const I2C_SICR_DATAIC = usize(0x00000001);  // Data Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR2 register.
//
//*****************************************************************************
pub const I2C_SOAR2_OAR2EN = usize(0x00000080);  // I2C Slave Own Address 2 Enable
pub const I2C_SOAR2_OAR2_M = usize(0x0000007F);  // I2C Slave Own Address 2
pub const I2C_SOAR2_OAR2_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SACKCTL register.
//
//*****************************************************************************
pub const I2C_SACKCTL_ACKOVAL = usize(0x00000002);  // I2C Slave ACK Override Value
pub const I2C_SACKCTL_ACKOEN = usize(0x00000001);  // I2C Slave ACK Override Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_FIFODATA register.
//
//*****************************************************************************
pub const I2C_FIFODATA_DATA_M = usize(0x000000FF);  // I2C TX FIFO Write Data Byte
pub const I2C_FIFODATA_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_FIFOCTL register.
//
//*****************************************************************************
pub const I2C_FIFOCTL_RXASGNMT = usize(0x80000000);  // RX Control Assignment
pub const I2C_FIFOCTL_RXFLUSH = usize(0x40000000);  // RX FIFO Flush
pub const I2C_FIFOCTL_DMARXENA = usize(0x20000000);  // DMA RX Channel Enable
pub const I2C_FIFOCTL_RXTRIG_M = usize(0x00070000);  // RX FIFO Trigger
pub const I2C_FIFOCTL_TXASGNMT = usize(0x00008000);  // TX Control Assignment
pub const I2C_FIFOCTL_TXFLUSH = usize(0x00004000);  // TX FIFO Flush
pub const I2C_FIFOCTL_DMATXENA = usize(0x00002000);  // DMA TX Channel Enable
pub const I2C_FIFOCTL_TXTRIG_M = usize(0x00000007);  // TX FIFO Trigger
pub const I2C_FIFOCTL_RXTRIG_S = usize(16);
pub const I2C_FIFOCTL_TXTRIG_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_FIFOSTATUS
// register.
//
//*****************************************************************************
pub const I2C_FIFOSTATUS_RXABVTRIG = usize(0x00040000);  // RX FIFO Above Trigger Level
pub const I2C_FIFOSTATUS_RXFF = usize(0x00020000);  // RX FIFO Full
pub const I2C_FIFOSTATUS_RXFE = usize(0x00010000);  // RX FIFO Empty
pub const I2C_FIFOSTATUS_TXBLWTRIG = usize(0x00000004);  // TX FIFO Below Trigger Level
pub const I2C_FIFOSTATUS_TXFF = usize(0x00000002);  // TX FIFO Full
pub const I2C_FIFOSTATUS_TXFE = usize(0x00000001);  // TX FIFO Empty

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PP register.
//
//*****************************************************************************
pub const I2C_PP_HS = usize(0x00000001);  // High-Speed Capable

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PC register.
//
//*****************************************************************************
pub const I2C_PC_HS = usize(0x00000001);  // High-Speed Capable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_CTL register.
//
//*****************************************************************************
pub const PWM_CTL_GLOBALSYNC3 = usize(0x00000008);  // Update PWM Generator 3
pub const PWM_CTL_GLOBALSYNC2 = usize(0x00000004);  // Update PWM Generator 2
pub const PWM_CTL_GLOBALSYNC1 = usize(0x00000002);  // Update PWM Generator 1
pub const PWM_CTL_GLOBALSYNC0 = usize(0x00000001);  // Update PWM Generator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_SYNC register.
//
//*****************************************************************************
pub const PWM_SYNC_SYNC3 = usize(0x00000008);  // Reset Generator 3 Counter
pub const PWM_SYNC_SYNC2 = usize(0x00000004);  // Reset Generator 2 Counter
pub const PWM_SYNC_SYNC1 = usize(0x00000002);  // Reset Generator 1 Counter
pub const PWM_SYNC_SYNC0 = usize(0x00000001);  // Reset Generator 0 Counter

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENABLE register.
//
//*****************************************************************************
pub const PWM_ENABLE_PWM7EN = usize(0x00000080);  // MnPWM7 Output Enable
pub const PWM_ENABLE_PWM6EN = usize(0x00000040);  // MnPWM6 Output Enable
pub const PWM_ENABLE_PWM5EN = usize(0x00000020);  // MnPWM5 Output Enable
pub const PWM_ENABLE_PWM4EN = usize(0x00000010);  // MnPWM4 Output Enable
pub const PWM_ENABLE_PWM3EN = usize(0x00000008);  // MnPWM3 Output Enable
pub const PWM_ENABLE_PWM2EN = usize(0x00000004);  // MnPWM2 Output Enable
pub const PWM_ENABLE_PWM1EN = usize(0x00000002);  // MnPWM1 Output Enable
pub const PWM_ENABLE_PWM0EN = usize(0x00000001);  // MnPWM0 Output Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INVERT register.
//
//*****************************************************************************
pub const PWM_INVERT_PWM7INV = usize(0x00000080);  // Invert MnPWM7 Signal
pub const PWM_INVERT_PWM6INV = usize(0x00000040);  // Invert MnPWM6 Signal
pub const PWM_INVERT_PWM5INV = usize(0x00000020);  // Invert MnPWM5 Signal
pub const PWM_INVERT_PWM4INV = usize(0x00000010);  // Invert MnPWM4 Signal
pub const PWM_INVERT_PWM3INV = usize(0x00000008);  // Invert MnPWM3 Signal
pub const PWM_INVERT_PWM2INV = usize(0x00000004);  // Invert MnPWM2 Signal
pub const PWM_INVERT_PWM1INV = usize(0x00000002);  // Invert MnPWM1 Signal
pub const PWM_INVERT_PWM0INV = usize(0x00000001);  // Invert MnPWM0 Signal

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULT register.
//
//*****************************************************************************
pub const PWM_FAULT_FAULT7 = usize(0x00000080);  // MnPWM7 Fault
pub const PWM_FAULT_FAULT6 = usize(0x00000040);  // MnPWM6 Fault
pub const PWM_FAULT_FAULT5 = usize(0x00000020);  // MnPWM5 Fault
pub const PWM_FAULT_FAULT4 = usize(0x00000010);  // MnPWM4 Fault
pub const PWM_FAULT_FAULT3 = usize(0x00000008);  // MnPWM3 Fault
pub const PWM_FAULT_FAULT2 = usize(0x00000004);  // MnPWM2 Fault
pub const PWM_FAULT_FAULT1 = usize(0x00000002);  // MnPWM1 Fault
pub const PWM_FAULT_FAULT0 = usize(0x00000001);  // MnPWM0 Fault

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INTEN register.
//
//*****************************************************************************
pub const PWM_INTEN_INTFAULT3 = usize(0x00080000);  // Interrupt Fault 3
pub const PWM_INTEN_INTFAULT2 = usize(0x00040000);  // Interrupt Fault 2
pub const PWM_INTEN_INTFAULT1 = usize(0x00020000);  // Interrupt Fault 1
pub const PWM_INTEN_INTFAULT0 = usize(0x00010000);  // Interrupt Fault 0
pub const PWM_INTEN_INTPWM3 = usize(0x00000008);  // PWM3 Interrupt Enable
pub const PWM_INTEN_INTPWM2 = usize(0x00000004);  // PWM2 Interrupt Enable
pub const PWM_INTEN_INTPWM1 = usize(0x00000002);  // PWM1 Interrupt Enable
pub const PWM_INTEN_INTPWM0 = usize(0x00000001);  // PWM0 Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_RIS register.
//
//*****************************************************************************
pub const PWM_RIS_INTFAULT3 = usize(0x00080000);  // Interrupt Fault PWM 3
pub const PWM_RIS_INTFAULT2 = usize(0x00040000);  // Interrupt Fault PWM 2
pub const PWM_RIS_INTFAULT1 = usize(0x00020000);  // Interrupt Fault PWM 1
pub const PWM_RIS_INTFAULT0 = usize(0x00010000);  // Interrupt Fault PWM 0
pub const PWM_RIS_INTPWM3 = usize(0x00000008);  // PWM3 Interrupt Asserted
pub const PWM_RIS_INTPWM2 = usize(0x00000004);  // PWM2 Interrupt Asserted
pub const PWM_RIS_INTPWM1 = usize(0x00000002);  // PWM1 Interrupt Asserted
pub const PWM_RIS_INTPWM0 = usize(0x00000001);  // PWM0 Interrupt Asserted

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ISC register.
//
//*****************************************************************************
pub const PWM_ISC_INTFAULT3 = usize(0x00080000);  // FAULT3 Interrupt Asserted
pub const PWM_ISC_INTFAULT2 = usize(0x00040000);  // FAULT2 Interrupt Asserted
pub const PWM_ISC_INTFAULT1 = usize(0x00020000);  // FAULT1 Interrupt Asserted
pub const PWM_ISC_INTFAULT0 = usize(0x00010000);  // FAULT0 Interrupt Asserted
pub const PWM_ISC_INTPWM3 = usize(0x00000008);  // PWM3 Interrupt Status
pub const PWM_ISC_INTPWM2 = usize(0x00000004);  // PWM2 Interrupt Status
pub const PWM_ISC_INTPWM1 = usize(0x00000002);  // PWM1 Interrupt Status
pub const PWM_ISC_INTPWM0 = usize(0x00000001);  // PWM0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_STATUS register.
//
//*****************************************************************************
pub const PWM_STATUS_FAULT3 = usize(0x00000008);  // Generator 3 Fault Status
pub const PWM_STATUS_FAULT2 = usize(0x00000004);  // Generator 2 Fault Status
pub const PWM_STATUS_FAULT1 = usize(0x00000002);  // Generator 1 Fault Status
pub const PWM_STATUS_FAULT0 = usize(0x00000001);  // Generator 0 Fault Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULTVAL register.
//
//*****************************************************************************
pub const PWM_FAULTVAL_PWM7 = usize(0x00000080);  // MnPWM7 Fault Value
pub const PWM_FAULTVAL_PWM6 = usize(0x00000040);  // MnPWM6 Fault Value
pub const PWM_FAULTVAL_PWM5 = usize(0x00000020);  // MnPWM5 Fault Value
pub const PWM_FAULTVAL_PWM4 = usize(0x00000010);  // MnPWM4 Fault Value
pub const PWM_FAULTVAL_PWM3 = usize(0x00000008);  // MnPWM3 Fault Value
pub const PWM_FAULTVAL_PWM2 = usize(0x00000004);  // MnPWM2 Fault Value
pub const PWM_FAULTVAL_PWM1 = usize(0x00000002);  // MnPWM1 Fault Value
pub const PWM_FAULTVAL_PWM0 = usize(0x00000001);  // MnPWM0 Fault Value

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENUPD register.
//
//*****************************************************************************
pub const PWM_ENUPD_ENUPD7_M = usize(0x0000C000);  // MnPWM7 Enable Update Mode
pub const PWM_ENUPD_ENUPD7_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD7_LSYNC = usize(0x00008000);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD7_GSYNC = usize(0x0000C000);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD6_M = usize(0x00003000);  // MnPWM6 Enable Update Mode
pub const PWM_ENUPD_ENUPD6_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD6_LSYNC = usize(0x00002000);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD6_GSYNC = usize(0x00003000);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD5_M = usize(0x00000C00);  // MnPWM5 Enable Update Mode
pub const PWM_ENUPD_ENUPD5_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD5_LSYNC = usize(0x00000800);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD5_GSYNC = usize(0x00000C00);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD4_M = usize(0x00000300);  // MnPWM4 Enable Update Mode
pub const PWM_ENUPD_ENUPD4_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD4_LSYNC = usize(0x00000200);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD4_GSYNC = usize(0x00000300);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD3_M = usize(0x000000C0);  // MnPWM3 Enable Update Mode
pub const PWM_ENUPD_ENUPD3_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD3_LSYNC = usize(0x00000080);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD3_GSYNC = usize(0x000000C0);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD2_M = usize(0x00000030);  // MnPWM2 Enable Update Mode
pub const PWM_ENUPD_ENUPD2_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD2_LSYNC = usize(0x00000020);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD2_GSYNC = usize(0x00000030);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD1_M = usize(0x0000000C);  // MnPWM1 Enable Update Mode
pub const PWM_ENUPD_ENUPD1_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD1_LSYNC = usize(0x00000008);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD1_GSYNC = usize(0x0000000C);  // Globally Synchronized
pub const PWM_ENUPD_ENUPD0_M = usize(0x00000003);  // MnPWM0 Enable Update Mode
pub const PWM_ENUPD_ENUPD0_IMM = usize(0x00000000);  // Immediate
pub const PWM_ENUPD_ENUPD0_LSYNC = usize(0x00000002);  // Locally Synchronized
pub const PWM_ENUPD_ENUPD0_GSYNC = usize(0x00000003);  // Globally Synchronized

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CTL register.
//
//*****************************************************************************
pub const PWM_0_CTL_LATCH = usize(0x00040000);  // Latch Fault Input
pub const PWM_0_CTL_MINFLTPER = usize(0x00020000);  // Minimum Fault Period
pub const PWM_0_CTL_FLTSRC = usize(0x00010000);  // Fault Condition Source
pub const PWM_0_CTL_DBFALLUPD_M = usize(0x0000C000);  // PWMnDBFALL Update Mode
pub const PWM_0_CTL_DBFALLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_0_CTL_DBFALLUPD_LS = usize(0x00008000);  // Locally Synchronized
pub const PWM_0_CTL_DBFALLUPD_GS = usize(0x0000C000);  // Globally Synchronized
pub const PWM_0_CTL_DBRISEUPD_M = usize(0x00003000);  // PWMnDBRISE Update Mode
pub const PWM_0_CTL_DBRISEUPD_I = usize(0x00000000);  // Immediate
pub const PWM_0_CTL_DBRISEUPD_LS = usize(0x00002000);  // Locally Synchronized
pub const PWM_0_CTL_DBRISEUPD_GS = usize(0x00003000);  // Globally Synchronized
pub const PWM_0_CTL_DBCTLUPD_M = usize(0x00000C00);  // PWMnDBCTL Update Mode
pub const PWM_0_CTL_DBCTLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_0_CTL_DBCTLUPD_LS = usize(0x00000800);  // Locally Synchronized
pub const PWM_0_CTL_DBCTLUPD_GS = usize(0x00000C00);  // Globally Synchronized
pub const PWM_0_CTL_GENBUPD_M = usize(0x00000300);  // PWMnGENB Update Mode
pub const PWM_0_CTL_GENBUPD_I = usize(0x00000000);  // Immediate
pub const PWM_0_CTL_GENBUPD_LS = usize(0x00000200);  // Locally Synchronized
pub const PWM_0_CTL_GENBUPD_GS = usize(0x00000300);  // Globally Synchronized
pub const PWM_0_CTL_GENAUPD_M = usize(0x000000C0);  // PWMnGENA Update Mode
pub const PWM_0_CTL_GENAUPD_I = usize(0x00000000);  // Immediate
pub const PWM_0_CTL_GENAUPD_LS = usize(0x00000080);  // Locally Synchronized
pub const PWM_0_CTL_GENAUPD_GS = usize(0x000000C0);  // Globally Synchronized
pub const PWM_0_CTL_CMPBUPD = usize(0x00000020);  // Comparator B Update Mode
pub const PWM_0_CTL_CMPAUPD = usize(0x00000010);  // Comparator A Update Mode
pub const PWM_0_CTL_LOADUPD = usize(0x00000008);  // Load Register Update Mode
pub const PWM_0_CTL_DEBUG = usize(0x00000004);  // Debug Mode
pub const PWM_0_CTL_MODE = usize(0x00000002);  // Counter Mode
pub const PWM_0_CTL_ENABLE = usize(0x00000001);  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_INTEN register.
//
//*****************************************************************************
pub const PWM_0_INTEN_TRCMPBD = usize(0x00002000);  // Trigger for Counter=PWMnCMPB
                                            // Down
pub const PWM_0_INTEN_TRCMPBU = usize(0x00001000);  // Trigger for Counter=PWMnCMPB Up
pub const PWM_0_INTEN_TRCMPAD = usize(0x00000800);  // Trigger for Counter=PWMnCMPA
                                            // Down
pub const PWM_0_INTEN_TRCMPAU = usize(0x00000400);  // Trigger for Counter=PWMnCMPA Up
pub const PWM_0_INTEN_TRCNTLOAD = usize(0x00000200);  // Trigger for Counter=PWMnLOAD
pub const PWM_0_INTEN_TRCNTZERO = usize(0x00000100);  // Trigger for Counter=0
pub const PWM_0_INTEN_INTCMPBD = usize(0x00000020);  // Interrupt for Counter=PWMnCMPB
                                            // Down
pub const PWM_0_INTEN_INTCMPBU = usize(0x00000010);  // Interrupt for Counter=PWMnCMPB
                                            // Up
pub const PWM_0_INTEN_INTCMPAD = usize(0x00000008);  // Interrupt for Counter=PWMnCMPA
                                            // Down
pub const PWM_0_INTEN_INTCMPAU = usize(0x00000004);  // Interrupt for Counter=PWMnCMPA
                                            // Up
pub const PWM_0_INTEN_INTCNTLOAD = usize(0x00000002);  // Interrupt for Counter=PWMnLOAD
pub const PWM_0_INTEN_INTCNTZERO = usize(0x00000001);  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_RIS register.
//
//*****************************************************************************
pub const PWM_0_RIS_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
                                            // Status
pub const PWM_0_RIS_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt Status
pub const PWM_0_RIS_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
                                            // Status
pub const PWM_0_RIS_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt Status
pub const PWM_0_RIS_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt Status
pub const PWM_0_RIS_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_ISC register.
//
//*****************************************************************************
pub const PWM_0_ISC_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
pub const PWM_0_ISC_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt
pub const PWM_0_ISC_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
pub const PWM_0_ISC_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt
pub const PWM_0_ISC_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt
pub const PWM_0_ISC_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_LOAD register.
//
//*****************************************************************************
pub const PWM_0_LOAD_M = usize(0x0000FFFF);  // Counter Load Value
pub const PWM_0_LOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_COUNT register.
//
//*****************************************************************************
pub const PWM_0_COUNT_M = usize(0x0000FFFF);  // Counter Value
pub const PWM_0_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPA register.
//
//*****************************************************************************
pub const PWM_0_CMPA_M = usize(0x0000FFFF);  // Comparator A Value
pub const PWM_0_CMPA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPB register.
//
//*****************************************************************************
pub const PWM_0_CMPB_M = usize(0x0000FFFF);  // Comparator B Value
pub const PWM_0_CMPB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENA register.
//
//*****************************************************************************
pub const PWM_0_GENA_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_0_GENA_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmA
pub const PWM_0_GENA_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmA Low
pub const PWM_0_GENA_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmA High
pub const PWM_0_GENA_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_0_GENA_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmA
pub const PWM_0_GENA_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmA Low
pub const PWM_0_GENA_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmA High
pub const PWM_0_GENA_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_0_GENA_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmA
pub const PWM_0_GENA_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmA Low
pub const PWM_0_GENA_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmA High
pub const PWM_0_GENA_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_0_GENA_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmA
pub const PWM_0_GENA_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmA Low
pub const PWM_0_GENA_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmA High
pub const PWM_0_GENA_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_0_GENA_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTLOAD_INV = usize(0x00000004);  // Invert pwmA
pub const PWM_0_GENA_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmA Low
pub const PWM_0_GENA_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmA High
pub const PWM_0_GENA_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_0_GENA_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENA_ACTZERO_INV = usize(0x00000001);  // Invert pwmA
pub const PWM_0_GENA_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmA Low
pub const PWM_0_GENA_ACTZERO_ONE = usize(0x00000003);  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENB register.
//
//*****************************************************************************
pub const PWM_0_GENB_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_0_GENB_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmB
pub const PWM_0_GENB_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmB Low
pub const PWM_0_GENB_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmB High
pub const PWM_0_GENB_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_0_GENB_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmB
pub const PWM_0_GENB_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmB Low
pub const PWM_0_GENB_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmB High
pub const PWM_0_GENB_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_0_GENB_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmB
pub const PWM_0_GENB_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmB Low
pub const PWM_0_GENB_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmB High
pub const PWM_0_GENB_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_0_GENB_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmB
pub const PWM_0_GENB_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmB Low
pub const PWM_0_GENB_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmB High
pub const PWM_0_GENB_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_0_GENB_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTLOAD_INV = usize(0x00000004);  // Invert pwmB
pub const PWM_0_GENB_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmB Low
pub const PWM_0_GENB_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmB High
pub const PWM_0_GENB_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_0_GENB_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_0_GENB_ACTZERO_INV = usize(0x00000001);  // Invert pwmB
pub const PWM_0_GENB_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmB Low
pub const PWM_0_GENB_ACTZERO_ONE = usize(0x00000003);  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBCTL register.
//
//*****************************************************************************
pub const PWM_0_DBCTL_ENABLE = usize(0x00000001);  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBRISE register.
//
//*****************************************************************************
pub const PWM_0_DBRISE_DELAY_M = usize(0x00000FFF);  // Dead-Band Rise Delay
pub const PWM_0_DBRISE_DELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBFALL register.
//
//*****************************************************************************
pub const PWM_0_DBFALL_DELAY_M = usize(0x00000FFF);  // Dead-Band Fall Delay
pub const PWM_0_DBFALL_DELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM_0_FLTSRC0_FAULT3 = usize(0x00000008);  // Fault3 Input
pub const PWM_0_FLTSRC0_FAULT2 = usize(0x00000004);  // Fault2 Input
pub const PWM_0_FLTSRC0_FAULT1 = usize(0x00000002);  // Fault1 Input
pub const PWM_0_FLTSRC0_FAULT0 = usize(0x00000001);  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM_0_FLTSRC1_DCMP7 = usize(0x00000080);  // Digital Comparator 7
pub const PWM_0_FLTSRC1_DCMP6 = usize(0x00000040);  // Digital Comparator 6
pub const PWM_0_FLTSRC1_DCMP5 = usize(0x00000020);  // Digital Comparator 5
pub const PWM_0_FLTSRC1_DCMP4 = usize(0x00000010);  // Digital Comparator 4
pub const PWM_0_FLTSRC1_DCMP3 = usize(0x00000008);  // Digital Comparator 3
pub const PWM_0_FLTSRC1_DCMP2 = usize(0x00000004);  // Digital Comparator 2
pub const PWM_0_FLTSRC1_DCMP1 = usize(0x00000002);  // Digital Comparator 1
pub const PWM_0_FLTSRC1_DCMP0 = usize(0x00000001);  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM_0_MINFLTPER_M = usize(0x0000FFFF);  // Minimum Fault Period
pub const PWM_0_MINFLTPER_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CTL register.
//
//*****************************************************************************
pub const PWM_1_CTL_LATCH = usize(0x00040000);  // Latch Fault Input
pub const PWM_1_CTL_MINFLTPER = usize(0x00020000);  // Minimum Fault Period
pub const PWM_1_CTL_FLTSRC = usize(0x00010000);  // Fault Condition Source
pub const PWM_1_CTL_DBFALLUPD_M = usize(0x0000C000);  // PWMnDBFALL Update Mode
pub const PWM_1_CTL_DBFALLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_1_CTL_DBFALLUPD_LS = usize(0x00008000);  // Locally Synchronized
pub const PWM_1_CTL_DBFALLUPD_GS = usize(0x0000C000);  // Globally Synchronized
pub const PWM_1_CTL_DBRISEUPD_M = usize(0x00003000);  // PWMnDBRISE Update Mode
pub const PWM_1_CTL_DBRISEUPD_I = usize(0x00000000);  // Immediate
pub const PWM_1_CTL_DBRISEUPD_LS = usize(0x00002000);  // Locally Synchronized
pub const PWM_1_CTL_DBRISEUPD_GS = usize(0x00003000);  // Globally Synchronized
pub const PWM_1_CTL_DBCTLUPD_M = usize(0x00000C00);  // PWMnDBCTL Update Mode
pub const PWM_1_CTL_DBCTLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_1_CTL_DBCTLUPD_LS = usize(0x00000800);  // Locally Synchronized
pub const PWM_1_CTL_DBCTLUPD_GS = usize(0x00000C00);  // Globally Synchronized
pub const PWM_1_CTL_GENBUPD_M = usize(0x00000300);  // PWMnGENB Update Mode
pub const PWM_1_CTL_GENBUPD_I = usize(0x00000000);  // Immediate
pub const PWM_1_CTL_GENBUPD_LS = usize(0x00000200);  // Locally Synchronized
pub const PWM_1_CTL_GENBUPD_GS = usize(0x00000300);  // Globally Synchronized
pub const PWM_1_CTL_GENAUPD_M = usize(0x000000C0);  // PWMnGENA Update Mode
pub const PWM_1_CTL_GENAUPD_I = usize(0x00000000);  // Immediate
pub const PWM_1_CTL_GENAUPD_LS = usize(0x00000080);  // Locally Synchronized
pub const PWM_1_CTL_GENAUPD_GS = usize(0x000000C0);  // Globally Synchronized
pub const PWM_1_CTL_CMPBUPD = usize(0x00000020);  // Comparator B Update Mode
pub const PWM_1_CTL_CMPAUPD = usize(0x00000010);  // Comparator A Update Mode
pub const PWM_1_CTL_LOADUPD = usize(0x00000008);  // Load Register Update Mode
pub const PWM_1_CTL_DEBUG = usize(0x00000004);  // Debug Mode
pub const PWM_1_CTL_MODE = usize(0x00000002);  // Counter Mode
pub const PWM_1_CTL_ENABLE = usize(0x00000001);  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_INTEN register.
//
//*****************************************************************************
pub const PWM_1_INTEN_TRCMPBD = usize(0x00002000);  // Trigger for Counter=PWMnCMPB
                                            // Down
pub const PWM_1_INTEN_TRCMPBU = usize(0x00001000);  // Trigger for Counter=PWMnCMPB Up
pub const PWM_1_INTEN_TRCMPAD = usize(0x00000800);  // Trigger for Counter=PWMnCMPA
                                            // Down
pub const PWM_1_INTEN_TRCMPAU = usize(0x00000400);  // Trigger for Counter=PWMnCMPA Up
pub const PWM_1_INTEN_TRCNTLOAD = usize(0x00000200);  // Trigger for Counter=PWMnLOAD
pub const PWM_1_INTEN_TRCNTZERO = usize(0x00000100);  // Trigger for Counter=0
pub const PWM_1_INTEN_INTCMPBD = usize(0x00000020);  // Interrupt for Counter=PWMnCMPB
                                            // Down
pub const PWM_1_INTEN_INTCMPBU = usize(0x00000010);  // Interrupt for Counter=PWMnCMPB
                                            // Up
pub const PWM_1_INTEN_INTCMPAD = usize(0x00000008);  // Interrupt for Counter=PWMnCMPA
                                            // Down
pub const PWM_1_INTEN_INTCMPAU = usize(0x00000004);  // Interrupt for Counter=PWMnCMPA
                                            // Up
pub const PWM_1_INTEN_INTCNTLOAD = usize(0x00000002);  // Interrupt for Counter=PWMnLOAD
pub const PWM_1_INTEN_INTCNTZERO = usize(0x00000001);  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_RIS register.
//
//*****************************************************************************
pub const PWM_1_RIS_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
                                            // Status
pub const PWM_1_RIS_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt Status
pub const PWM_1_RIS_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
                                            // Status
pub const PWM_1_RIS_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt Status
pub const PWM_1_RIS_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt Status
pub const PWM_1_RIS_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_ISC register.
//
//*****************************************************************************
pub const PWM_1_ISC_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
pub const PWM_1_ISC_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt
pub const PWM_1_ISC_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
pub const PWM_1_ISC_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt
pub const PWM_1_ISC_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt
pub const PWM_1_ISC_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_LOAD register.
//
//*****************************************************************************
pub const PWM_1_LOAD_LOAD_M = usize(0x0000FFFF);  // Counter Load Value
pub const PWM_1_LOAD_LOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_COUNT register.
//
//*****************************************************************************
pub const PWM_1_COUNT_COUNT_M = usize(0x0000FFFF);  // Counter Value
pub const PWM_1_COUNT_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CMPA register.
//
//*****************************************************************************
pub const PWM_1_CMPA_COMPA_M = usize(0x0000FFFF);  // Comparator A Value
pub const PWM_1_CMPA_COMPA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CMPB register.
//
//*****************************************************************************
pub const PWM_1_CMPB_COMPB_M = usize(0x0000FFFF);  // Comparator B Value
pub const PWM_1_CMPB_COMPB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_GENA register.
//
//*****************************************************************************
pub const PWM_1_GENA_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_1_GENA_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmA
pub const PWM_1_GENA_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmA Low
pub const PWM_1_GENA_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmA High
pub const PWM_1_GENA_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_1_GENA_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmA
pub const PWM_1_GENA_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmA Low
pub const PWM_1_GENA_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmA High
pub const PWM_1_GENA_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_1_GENA_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmA
pub const PWM_1_GENA_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmA Low
pub const PWM_1_GENA_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmA High
pub const PWM_1_GENA_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_1_GENA_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmA
pub const PWM_1_GENA_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmA Low
pub const PWM_1_GENA_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmA High
pub const PWM_1_GENA_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_1_GENA_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTLOAD_INV = usize(0x00000004);  // Invert pwmA
pub const PWM_1_GENA_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmA Low
pub const PWM_1_GENA_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmA High
pub const PWM_1_GENA_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_1_GENA_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENA_ACTZERO_INV = usize(0x00000001);  // Invert pwmA
pub const PWM_1_GENA_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmA Low
pub const PWM_1_GENA_ACTZERO_ONE = usize(0x00000003);  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_GENB register.
//
//*****************************************************************************
pub const PWM_1_GENB_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_1_GENB_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmB
pub const PWM_1_GENB_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmB Low
pub const PWM_1_GENB_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmB High
pub const PWM_1_GENB_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_1_GENB_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmB
pub const PWM_1_GENB_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmB Low
pub const PWM_1_GENB_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmB High
pub const PWM_1_GENB_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_1_GENB_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmB
pub const PWM_1_GENB_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmB Low
pub const PWM_1_GENB_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmB High
pub const PWM_1_GENB_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_1_GENB_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmB
pub const PWM_1_GENB_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmB Low
pub const PWM_1_GENB_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmB High
pub const PWM_1_GENB_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_1_GENB_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTLOAD_INV = usize(0x00000004);  // Invert pwmB
pub const PWM_1_GENB_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmB Low
pub const PWM_1_GENB_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmB High
pub const PWM_1_GENB_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_1_GENB_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_1_GENB_ACTZERO_INV = usize(0x00000001);  // Invert pwmB
pub const PWM_1_GENB_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmB Low
pub const PWM_1_GENB_ACTZERO_ONE = usize(0x00000003);  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBCTL register.
//
//*****************************************************************************
pub const PWM_1_DBCTL_ENABLE = usize(0x00000001);  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBRISE register.
//
//*****************************************************************************
pub const PWM_1_DBRISE_RISEDELAY_M = usize(0x00000FFF);  // Dead-Band Rise Delay
pub const PWM_1_DBRISE_RISEDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBFALL register.
//
//*****************************************************************************
pub const PWM_1_DBFALL_FALLDELAY_M = usize(0x00000FFF);  // Dead-Band Fall Delay
pub const PWM_1_DBFALL_FALLDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM_1_FLTSRC0_FAULT3 = usize(0x00000008);  // Fault3 Input
pub const PWM_1_FLTSRC0_FAULT2 = usize(0x00000004);  // Fault2 Input
pub const PWM_1_FLTSRC0_FAULT1 = usize(0x00000002);  // Fault1 Input
pub const PWM_1_FLTSRC0_FAULT0 = usize(0x00000001);  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM_1_FLTSRC1_DCMP7 = usize(0x00000080);  // Digital Comparator 7
pub const PWM_1_FLTSRC1_DCMP6 = usize(0x00000040);  // Digital Comparator 6
pub const PWM_1_FLTSRC1_DCMP5 = usize(0x00000020);  // Digital Comparator 5
pub const PWM_1_FLTSRC1_DCMP4 = usize(0x00000010);  // Digital Comparator 4
pub const PWM_1_FLTSRC1_DCMP3 = usize(0x00000008);  // Digital Comparator 3
pub const PWM_1_FLTSRC1_DCMP2 = usize(0x00000004);  // Digital Comparator 2
pub const PWM_1_FLTSRC1_DCMP1 = usize(0x00000002);  // Digital Comparator 1
pub const PWM_1_FLTSRC1_DCMP0 = usize(0x00000001);  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM_1_MINFLTPER_MFP_M = usize(0x0000FFFF);  // Minimum Fault Period
pub const PWM_1_MINFLTPER_MFP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CTL register.
//
//*****************************************************************************
pub const PWM_2_CTL_LATCH = usize(0x00040000);  // Latch Fault Input
pub const PWM_2_CTL_MINFLTPER = usize(0x00020000);  // Minimum Fault Period
pub const PWM_2_CTL_FLTSRC = usize(0x00010000);  // Fault Condition Source
pub const PWM_2_CTL_DBFALLUPD_M = usize(0x0000C000);  // PWMnDBFALL Update Mode
pub const PWM_2_CTL_DBFALLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_2_CTL_DBFALLUPD_LS = usize(0x00008000);  // Locally Synchronized
pub const PWM_2_CTL_DBFALLUPD_GS = usize(0x0000C000);  // Globally Synchronized
pub const PWM_2_CTL_DBRISEUPD_M = usize(0x00003000);  // PWMnDBRISE Update Mode
pub const PWM_2_CTL_DBRISEUPD_I = usize(0x00000000);  // Immediate
pub const PWM_2_CTL_DBRISEUPD_LS = usize(0x00002000);  // Locally Synchronized
pub const PWM_2_CTL_DBRISEUPD_GS = usize(0x00003000);  // Globally Synchronized
pub const PWM_2_CTL_DBCTLUPD_M = usize(0x00000C00);  // PWMnDBCTL Update Mode
pub const PWM_2_CTL_DBCTLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_2_CTL_DBCTLUPD_LS = usize(0x00000800);  // Locally Synchronized
pub const PWM_2_CTL_DBCTLUPD_GS = usize(0x00000C00);  // Globally Synchronized
pub const PWM_2_CTL_GENBUPD_M = usize(0x00000300);  // PWMnGENB Update Mode
pub const PWM_2_CTL_GENBUPD_I = usize(0x00000000);  // Immediate
pub const PWM_2_CTL_GENBUPD_LS = usize(0x00000200);  // Locally Synchronized
pub const PWM_2_CTL_GENBUPD_GS = usize(0x00000300);  // Globally Synchronized
pub const PWM_2_CTL_GENAUPD_M = usize(0x000000C0);  // PWMnGENA Update Mode
pub const PWM_2_CTL_GENAUPD_I = usize(0x00000000);  // Immediate
pub const PWM_2_CTL_GENAUPD_LS = usize(0x00000080);  // Locally Synchronized
pub const PWM_2_CTL_GENAUPD_GS = usize(0x000000C0);  // Globally Synchronized
pub const PWM_2_CTL_CMPBUPD = usize(0x00000020);  // Comparator B Update Mode
pub const PWM_2_CTL_CMPAUPD = usize(0x00000010);  // Comparator A Update Mode
pub const PWM_2_CTL_LOADUPD = usize(0x00000008);  // Load Register Update Mode
pub const PWM_2_CTL_DEBUG = usize(0x00000004);  // Debug Mode
pub const PWM_2_CTL_MODE = usize(0x00000002);  // Counter Mode
pub const PWM_2_CTL_ENABLE = usize(0x00000001);  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_INTEN register.
//
//*****************************************************************************
pub const PWM_2_INTEN_TRCMPBD = usize(0x00002000);  // Trigger for Counter=PWMnCMPB
                                            // Down
pub const PWM_2_INTEN_TRCMPBU = usize(0x00001000);  // Trigger for Counter=PWMnCMPB Up
pub const PWM_2_INTEN_TRCMPAD = usize(0x00000800);  // Trigger for Counter=PWMnCMPA
                                            // Down
pub const PWM_2_INTEN_TRCMPAU = usize(0x00000400);  // Trigger for Counter=PWMnCMPA Up
pub const PWM_2_INTEN_TRCNTLOAD = usize(0x00000200);  // Trigger for Counter=PWMnLOAD
pub const PWM_2_INTEN_TRCNTZERO = usize(0x00000100);  // Trigger for Counter=0
pub const PWM_2_INTEN_INTCMPBD = usize(0x00000020);  // Interrupt for Counter=PWMnCMPB
                                            // Down
pub const PWM_2_INTEN_INTCMPBU = usize(0x00000010);  // Interrupt for Counter=PWMnCMPB
                                            // Up
pub const PWM_2_INTEN_INTCMPAD = usize(0x00000008);  // Interrupt for Counter=PWMnCMPA
                                            // Down
pub const PWM_2_INTEN_INTCMPAU = usize(0x00000004);  // Interrupt for Counter=PWMnCMPA
                                            // Up
pub const PWM_2_INTEN_INTCNTLOAD = usize(0x00000002);  // Interrupt for Counter=PWMnLOAD
pub const PWM_2_INTEN_INTCNTZERO = usize(0x00000001);  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_RIS register.
//
//*****************************************************************************
pub const PWM_2_RIS_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
                                            // Status
pub const PWM_2_RIS_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt Status
pub const PWM_2_RIS_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
                                            // Status
pub const PWM_2_RIS_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt Status
pub const PWM_2_RIS_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt Status
pub const PWM_2_RIS_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_ISC register.
//
//*****************************************************************************
pub const PWM_2_ISC_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
pub const PWM_2_ISC_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt
pub const PWM_2_ISC_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
pub const PWM_2_ISC_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt
pub const PWM_2_ISC_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt
pub const PWM_2_ISC_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_LOAD register.
//
//*****************************************************************************
pub const PWM_2_LOAD_LOAD_M = usize(0x0000FFFF);  // Counter Load Value
pub const PWM_2_LOAD_LOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_COUNT register.
//
//*****************************************************************************
pub const PWM_2_COUNT_COUNT_M = usize(0x0000FFFF);  // Counter Value
pub const PWM_2_COUNT_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CMPA register.
//
//*****************************************************************************
pub const PWM_2_CMPA_COMPA_M = usize(0x0000FFFF);  // Comparator A Value
pub const PWM_2_CMPA_COMPA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CMPB register.
//
//*****************************************************************************
pub const PWM_2_CMPB_COMPB_M = usize(0x0000FFFF);  // Comparator B Value
pub const PWM_2_CMPB_COMPB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_GENA register.
//
//*****************************************************************************
pub const PWM_2_GENA_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_2_GENA_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmA
pub const PWM_2_GENA_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmA Low
pub const PWM_2_GENA_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmA High
pub const PWM_2_GENA_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_2_GENA_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmA
pub const PWM_2_GENA_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmA Low
pub const PWM_2_GENA_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmA High
pub const PWM_2_GENA_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_2_GENA_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmA
pub const PWM_2_GENA_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmA Low
pub const PWM_2_GENA_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmA High
pub const PWM_2_GENA_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_2_GENA_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmA
pub const PWM_2_GENA_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmA Low
pub const PWM_2_GENA_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmA High
pub const PWM_2_GENA_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_2_GENA_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTLOAD_INV = usize(0x00000004);  // Invert pwmA
pub const PWM_2_GENA_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmA Low
pub const PWM_2_GENA_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmA High
pub const PWM_2_GENA_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_2_GENA_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENA_ACTZERO_INV = usize(0x00000001);  // Invert pwmA
pub const PWM_2_GENA_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmA Low
pub const PWM_2_GENA_ACTZERO_ONE = usize(0x00000003);  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_GENB register.
//
//*****************************************************************************
pub const PWM_2_GENB_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_2_GENB_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmB
pub const PWM_2_GENB_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmB Low
pub const PWM_2_GENB_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmB High
pub const PWM_2_GENB_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_2_GENB_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmB
pub const PWM_2_GENB_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmB Low
pub const PWM_2_GENB_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmB High
pub const PWM_2_GENB_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_2_GENB_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmB
pub const PWM_2_GENB_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmB Low
pub const PWM_2_GENB_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmB High
pub const PWM_2_GENB_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_2_GENB_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmB
pub const PWM_2_GENB_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmB Low
pub const PWM_2_GENB_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmB High
pub const PWM_2_GENB_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_2_GENB_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTLOAD_INV = usize(0x00000004);  // Invert pwmB
pub const PWM_2_GENB_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmB Low
pub const PWM_2_GENB_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmB High
pub const PWM_2_GENB_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_2_GENB_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_2_GENB_ACTZERO_INV = usize(0x00000001);  // Invert pwmB
pub const PWM_2_GENB_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmB Low
pub const PWM_2_GENB_ACTZERO_ONE = usize(0x00000003);  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBCTL register.
//
//*****************************************************************************
pub const PWM_2_DBCTL_ENABLE = usize(0x00000001);  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBRISE register.
//
//*****************************************************************************
pub const PWM_2_DBRISE_RISEDELAY_M = usize(0x00000FFF);  // Dead-Band Rise Delay
pub const PWM_2_DBRISE_RISEDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBFALL register.
//
//*****************************************************************************
pub const PWM_2_DBFALL_FALLDELAY_M = usize(0x00000FFF);  // Dead-Band Fall Delay
pub const PWM_2_DBFALL_FALLDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM_2_FLTSRC0_FAULT3 = usize(0x00000008);  // Fault3 Input
pub const PWM_2_FLTSRC0_FAULT2 = usize(0x00000004);  // Fault2 Input
pub const PWM_2_FLTSRC0_FAULT1 = usize(0x00000002);  // Fault1 Input
pub const PWM_2_FLTSRC0_FAULT0 = usize(0x00000001);  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM_2_FLTSRC1_DCMP7 = usize(0x00000080);  // Digital Comparator 7
pub const PWM_2_FLTSRC1_DCMP6 = usize(0x00000040);  // Digital Comparator 6
pub const PWM_2_FLTSRC1_DCMP5 = usize(0x00000020);  // Digital Comparator 5
pub const PWM_2_FLTSRC1_DCMP4 = usize(0x00000010);  // Digital Comparator 4
pub const PWM_2_FLTSRC1_DCMP3 = usize(0x00000008);  // Digital Comparator 3
pub const PWM_2_FLTSRC1_DCMP2 = usize(0x00000004);  // Digital Comparator 2
pub const PWM_2_FLTSRC1_DCMP1 = usize(0x00000002);  // Digital Comparator 1
pub const PWM_2_FLTSRC1_DCMP0 = usize(0x00000001);  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM_2_MINFLTPER_MFP_M = usize(0x0000FFFF);  // Minimum Fault Period
pub const PWM_2_MINFLTPER_MFP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CTL register.
//
//*****************************************************************************
pub const PWM_3_CTL_LATCH = usize(0x00040000);  // Latch Fault Input
pub const PWM_3_CTL_MINFLTPER = usize(0x00020000);  // Minimum Fault Period
pub const PWM_3_CTL_FLTSRC = usize(0x00010000);  // Fault Condition Source
pub const PWM_3_CTL_DBFALLUPD_M = usize(0x0000C000);  // PWMnDBFALL Update Mode
pub const PWM_3_CTL_DBFALLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_3_CTL_DBFALLUPD_LS = usize(0x00008000);  // Locally Synchronized
pub const PWM_3_CTL_DBFALLUPD_GS = usize(0x0000C000);  // Globally Synchronized
pub const PWM_3_CTL_DBRISEUPD_M = usize(0x00003000);  // PWMnDBRISE Update Mode
pub const PWM_3_CTL_DBRISEUPD_I = usize(0x00000000);  // Immediate
pub const PWM_3_CTL_DBRISEUPD_LS = usize(0x00002000);  // Locally Synchronized
pub const PWM_3_CTL_DBRISEUPD_GS = usize(0x00003000);  // Globally Synchronized
pub const PWM_3_CTL_DBCTLUPD_M = usize(0x00000C00);  // PWMnDBCTL Update Mode
pub const PWM_3_CTL_DBCTLUPD_I = usize(0x00000000);  // Immediate
pub const PWM_3_CTL_DBCTLUPD_LS = usize(0x00000800);  // Locally Synchronized
pub const PWM_3_CTL_DBCTLUPD_GS = usize(0x00000C00);  // Globally Synchronized
pub const PWM_3_CTL_GENBUPD_M = usize(0x00000300);  // PWMnGENB Update Mode
pub const PWM_3_CTL_GENBUPD_I = usize(0x00000000);  // Immediate
pub const PWM_3_CTL_GENBUPD_LS = usize(0x00000200);  // Locally Synchronized
pub const PWM_3_CTL_GENBUPD_GS = usize(0x00000300);  // Globally Synchronized
pub const PWM_3_CTL_GENAUPD_M = usize(0x000000C0);  // PWMnGENA Update Mode
pub const PWM_3_CTL_GENAUPD_I = usize(0x00000000);  // Immediate
pub const PWM_3_CTL_GENAUPD_LS = usize(0x00000080);  // Locally Synchronized
pub const PWM_3_CTL_GENAUPD_GS = usize(0x000000C0);  // Globally Synchronized
pub const PWM_3_CTL_CMPBUPD = usize(0x00000020);  // Comparator B Update Mode
pub const PWM_3_CTL_CMPAUPD = usize(0x00000010);  // Comparator A Update Mode
pub const PWM_3_CTL_LOADUPD = usize(0x00000008);  // Load Register Update Mode
pub const PWM_3_CTL_DEBUG = usize(0x00000004);  // Debug Mode
pub const PWM_3_CTL_MODE = usize(0x00000002);  // Counter Mode
pub const PWM_3_CTL_ENABLE = usize(0x00000001);  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_INTEN register.
//
//*****************************************************************************
pub const PWM_3_INTEN_TRCMPBD = usize(0x00002000);  // Trigger for Counter=PWMnCMPB
                                            // Down
pub const PWM_3_INTEN_TRCMPBU = usize(0x00001000);  // Trigger for Counter=PWMnCMPB Up
pub const PWM_3_INTEN_TRCMPAD = usize(0x00000800);  // Trigger for Counter=PWMnCMPA
                                            // Down
pub const PWM_3_INTEN_TRCMPAU = usize(0x00000400);  // Trigger for Counter=PWMnCMPA Up
pub const PWM_3_INTEN_TRCNTLOAD = usize(0x00000200);  // Trigger for Counter=PWMnLOAD
pub const PWM_3_INTEN_TRCNTZERO = usize(0x00000100);  // Trigger for Counter=0
pub const PWM_3_INTEN_INTCMPBD = usize(0x00000020);  // Interrupt for Counter=PWMnCMPB
                                            // Down
pub const PWM_3_INTEN_INTCMPBU = usize(0x00000010);  // Interrupt for Counter=PWMnCMPB
                                            // Up
pub const PWM_3_INTEN_INTCMPAD = usize(0x00000008);  // Interrupt for Counter=PWMnCMPA
                                            // Down
pub const PWM_3_INTEN_INTCMPAU = usize(0x00000004);  // Interrupt for Counter=PWMnCMPA
                                            // Up
pub const PWM_3_INTEN_INTCNTLOAD = usize(0x00000002);  // Interrupt for Counter=PWMnLOAD
pub const PWM_3_INTEN_INTCNTZERO = usize(0x00000001);  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_RIS register.
//
//*****************************************************************************
pub const PWM_3_RIS_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
                                            // Status
pub const PWM_3_RIS_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt Status
pub const PWM_3_RIS_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
                                            // Status
pub const PWM_3_RIS_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt Status
pub const PWM_3_RIS_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt Status
pub const PWM_3_RIS_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_ISC register.
//
//*****************************************************************************
pub const PWM_3_ISC_INTCMPBD = usize(0x00000020);  // Comparator B Down Interrupt
pub const PWM_3_ISC_INTCMPBU = usize(0x00000010);  // Comparator B Up Interrupt
pub const PWM_3_ISC_INTCMPAD = usize(0x00000008);  // Comparator A Down Interrupt
pub const PWM_3_ISC_INTCMPAU = usize(0x00000004);  // Comparator A Up Interrupt
pub const PWM_3_ISC_INTCNTLOAD = usize(0x00000002);  // Counter=Load Interrupt
pub const PWM_3_ISC_INTCNTZERO = usize(0x00000001);  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_LOAD register.
//
//*****************************************************************************
pub const PWM_3_LOAD_LOAD_M = usize(0x0000FFFF);  // Counter Load Value
pub const PWM_3_LOAD_LOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_COUNT register.
//
//*****************************************************************************
pub const PWM_3_COUNT_COUNT_M = usize(0x0000FFFF);  // Counter Value
pub const PWM_3_COUNT_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CMPA register.
//
//*****************************************************************************
pub const PWM_3_CMPA_COMPA_M = usize(0x0000FFFF);  // Comparator A Value
pub const PWM_3_CMPA_COMPA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CMPB register.
//
//*****************************************************************************
pub const PWM_3_CMPB_COMPB_M = usize(0x0000FFFF);  // Comparator B Value
pub const PWM_3_CMPB_COMPB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_GENA register.
//
//*****************************************************************************
pub const PWM_3_GENA_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_3_GENA_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmA
pub const PWM_3_GENA_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmA Low
pub const PWM_3_GENA_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmA High
pub const PWM_3_GENA_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_3_GENA_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmA
pub const PWM_3_GENA_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmA Low
pub const PWM_3_GENA_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmA High
pub const PWM_3_GENA_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_3_GENA_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmA
pub const PWM_3_GENA_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmA Low
pub const PWM_3_GENA_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmA High
pub const PWM_3_GENA_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_3_GENA_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmA
pub const PWM_3_GENA_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmA Low
pub const PWM_3_GENA_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmA High
pub const PWM_3_GENA_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_3_GENA_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTLOAD_INV = usize(0x00000004);  // Invert pwmA
pub const PWM_3_GENA_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmA Low
pub const PWM_3_GENA_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmA High
pub const PWM_3_GENA_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_3_GENA_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENA_ACTZERO_INV = usize(0x00000001);  // Invert pwmA
pub const PWM_3_GENA_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmA Low
pub const PWM_3_GENA_ACTZERO_ONE = usize(0x00000003);  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_GENB register.
//
//*****************************************************************************
pub const PWM_3_GENB_ACTCMPBD_M = usize(0x00000C00);  // Action for Comparator B Down
pub const PWM_3_GENB_ACTCMPBD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTCMPBD_INV = usize(0x00000400);  // Invert pwmB
pub const PWM_3_GENB_ACTCMPBD_ZERO = usize(0x00000800);  // Drive pwmB Low
pub const PWM_3_GENB_ACTCMPBD_ONE = usize(0x00000C00);  // Drive pwmB High
pub const PWM_3_GENB_ACTCMPBU_M = usize(0x00000300);  // Action for Comparator B Up
pub const PWM_3_GENB_ACTCMPBU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTCMPBU_INV = usize(0x00000100);  // Invert pwmB
pub const PWM_3_GENB_ACTCMPBU_ZERO = usize(0x00000200);  // Drive pwmB Low
pub const PWM_3_GENB_ACTCMPBU_ONE = usize(0x00000300);  // Drive pwmB High
pub const PWM_3_GENB_ACTCMPAD_M = usize(0x000000C0);  // Action for Comparator A Down
pub const PWM_3_GENB_ACTCMPAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTCMPAD_INV = usize(0x00000040);  // Invert pwmB
pub const PWM_3_GENB_ACTCMPAD_ZERO = usize(0x00000080);  // Drive pwmB Low
pub const PWM_3_GENB_ACTCMPAD_ONE = usize(0x000000C0);  // Drive pwmB High
pub const PWM_3_GENB_ACTCMPAU_M = usize(0x00000030);  // Action for Comparator A Up
pub const PWM_3_GENB_ACTCMPAU_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTCMPAU_INV = usize(0x00000010);  // Invert pwmB
pub const PWM_3_GENB_ACTCMPAU_ZERO = usize(0x00000020);  // Drive pwmB Low
pub const PWM_3_GENB_ACTCMPAU_ONE = usize(0x00000030);  // Drive pwmB High
pub const PWM_3_GENB_ACTLOAD_M = usize(0x0000000C);  // Action for Counter=LOAD
pub const PWM_3_GENB_ACTLOAD_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTLOAD_INV = usize(0x00000004);  // Invert pwmB
pub const PWM_3_GENB_ACTLOAD_ZERO = usize(0x00000008);  // Drive pwmB Low
pub const PWM_3_GENB_ACTLOAD_ONE = usize(0x0000000C);  // Drive pwmB High
pub const PWM_3_GENB_ACTZERO_M = usize(0x00000003);  // Action for Counter=0
pub const PWM_3_GENB_ACTZERO_NONE = usize(0x00000000);  // Do nothing
pub const PWM_3_GENB_ACTZERO_INV = usize(0x00000001);  // Invert pwmB
pub const PWM_3_GENB_ACTZERO_ZERO = usize(0x00000002);  // Drive pwmB Low
pub const PWM_3_GENB_ACTZERO_ONE = usize(0x00000003);  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBCTL register.
//
//*****************************************************************************
pub const PWM_3_DBCTL_ENABLE = usize(0x00000001);  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBRISE register.
//
//*****************************************************************************
pub const PWM_3_DBRISE_RISEDELAY_M = usize(0x00000FFF);  // Dead-Band Rise Delay
pub const PWM_3_DBRISE_RISEDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBFALL register.
//
//*****************************************************************************
pub const PWM_3_DBFALL_FALLDELAY_M = usize(0x00000FFF);  // Dead-Band Fall Delay
pub const PWM_3_DBFALL_FALLDELAY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSRC0
// register.
//
//*****************************************************************************
pub const PWM_3_FLTSRC0_FAULT3 = usize(0x00000008);  // Fault3 Input
pub const PWM_3_FLTSRC0_FAULT2 = usize(0x00000004);  // Fault2 Input
pub const PWM_3_FLTSRC0_FAULT1 = usize(0x00000002);  // Fault1 Input
pub const PWM_3_FLTSRC0_FAULT0 = usize(0x00000001);  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSRC1
// register.
//
//*****************************************************************************
pub const PWM_3_FLTSRC1_DCMP7 = usize(0x00000080);  // Digital Comparator 7
pub const PWM_3_FLTSRC1_DCMP6 = usize(0x00000040);  // Digital Comparator 6
pub const PWM_3_FLTSRC1_DCMP5 = usize(0x00000020);  // Digital Comparator 5
pub const PWM_3_FLTSRC1_DCMP4 = usize(0x00000010);  // Digital Comparator 4
pub const PWM_3_FLTSRC1_DCMP3 = usize(0x00000008);  // Digital Comparator 3
pub const PWM_3_FLTSRC1_DCMP2 = usize(0x00000004);  // Digital Comparator 2
pub const PWM_3_FLTSRC1_DCMP1 = usize(0x00000002);  // Digital Comparator 1
pub const PWM_3_FLTSRC1_DCMP0 = usize(0x00000001);  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_MINFLTPER
// register.
//
//*****************************************************************************
pub const PWM_3_MINFLTPER_MFP_M = usize(0x0000FFFF);  // Minimum Fault Period
pub const PWM_3_MINFLTPER_MFP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSEN register.
//
//*****************************************************************************
pub const PWM_0_FLTSEN_FAULT3 = usize(0x00000008);  // Fault3 Sense
pub const PWM_0_FLTSEN_FAULT2 = usize(0x00000004);  // Fault2 Sense
pub const PWM_0_FLTSEN_FAULT1 = usize(0x00000002);  // Fault1 Sense
pub const PWM_0_FLTSEN_FAULT0 = usize(0x00000001);  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM_0_FLTSTAT0_FAULT3 = usize(0x00000008);  // Fault Input 3
pub const PWM_0_FLTSTAT0_FAULT2 = usize(0x00000004);  // Fault Input 2
pub const PWM_0_FLTSTAT0_FAULT1 = usize(0x00000002);  // Fault Input 1
pub const PWM_0_FLTSTAT0_FAULT0 = usize(0x00000001);  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM_0_FLTSTAT1_DCMP7 = usize(0x00000080);  // Digital Comparator 7 Trigger
pub const PWM_0_FLTSTAT1_DCMP6 = usize(0x00000040);  // Digital Comparator 6 Trigger
pub const PWM_0_FLTSTAT1_DCMP5 = usize(0x00000020);  // Digital Comparator 5 Trigger
pub const PWM_0_FLTSTAT1_DCMP4 = usize(0x00000010);  // Digital Comparator 4 Trigger
pub const PWM_0_FLTSTAT1_DCMP3 = usize(0x00000008);  // Digital Comparator 3 Trigger
pub const PWM_0_FLTSTAT1_DCMP2 = usize(0x00000004);  // Digital Comparator 2 Trigger
pub const PWM_0_FLTSTAT1_DCMP1 = usize(0x00000002);  // Digital Comparator 1 Trigger
pub const PWM_0_FLTSTAT1_DCMP0 = usize(0x00000001);  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSEN register.
//
//*****************************************************************************
pub const PWM_1_FLTSEN_FAULT3 = usize(0x00000008);  // Fault3 Sense
pub const PWM_1_FLTSEN_FAULT2 = usize(0x00000004);  // Fault2 Sense
pub const PWM_1_FLTSEN_FAULT1 = usize(0x00000002);  // Fault1 Sense
pub const PWM_1_FLTSEN_FAULT0 = usize(0x00000001);  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM_1_FLTSTAT0_FAULT3 = usize(0x00000008);  // Fault Input 3
pub const PWM_1_FLTSTAT0_FAULT2 = usize(0x00000004);  // Fault Input 2
pub const PWM_1_FLTSTAT0_FAULT1 = usize(0x00000002);  // Fault Input 1
pub const PWM_1_FLTSTAT0_FAULT0 = usize(0x00000001);  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM_1_FLTSTAT1_DCMP7 = usize(0x00000080);  // Digital Comparator 7 Trigger
pub const PWM_1_FLTSTAT1_DCMP6 = usize(0x00000040);  // Digital Comparator 6 Trigger
pub const PWM_1_FLTSTAT1_DCMP5 = usize(0x00000020);  // Digital Comparator 5 Trigger
pub const PWM_1_FLTSTAT1_DCMP4 = usize(0x00000010);  // Digital Comparator 4 Trigger
pub const PWM_1_FLTSTAT1_DCMP3 = usize(0x00000008);  // Digital Comparator 3 Trigger
pub const PWM_1_FLTSTAT1_DCMP2 = usize(0x00000004);  // Digital Comparator 2 Trigger
pub const PWM_1_FLTSTAT1_DCMP1 = usize(0x00000002);  // Digital Comparator 1 Trigger
pub const PWM_1_FLTSTAT1_DCMP0 = usize(0x00000001);  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSEN register.
//
//*****************************************************************************
pub const PWM_2_FLTSEN_FAULT3 = usize(0x00000008);  // Fault3 Sense
pub const PWM_2_FLTSEN_FAULT2 = usize(0x00000004);  // Fault2 Sense
pub const PWM_2_FLTSEN_FAULT1 = usize(0x00000002);  // Fault1 Sense
pub const PWM_2_FLTSEN_FAULT0 = usize(0x00000001);  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM_2_FLTSTAT0_FAULT3 = usize(0x00000008);  // Fault Input 3
pub const PWM_2_FLTSTAT0_FAULT2 = usize(0x00000004);  // Fault Input 2
pub const PWM_2_FLTSTAT0_FAULT1 = usize(0x00000002);  // Fault Input 1
pub const PWM_2_FLTSTAT0_FAULT0 = usize(0x00000001);  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM_2_FLTSTAT1_DCMP7 = usize(0x00000080);  // Digital Comparator 7 Trigger
pub const PWM_2_FLTSTAT1_DCMP6 = usize(0x00000040);  // Digital Comparator 6 Trigger
pub const PWM_2_FLTSTAT1_DCMP5 = usize(0x00000020);  // Digital Comparator 5 Trigger
pub const PWM_2_FLTSTAT1_DCMP4 = usize(0x00000010);  // Digital Comparator 4 Trigger
pub const PWM_2_FLTSTAT1_DCMP3 = usize(0x00000008);  // Digital Comparator 3 Trigger
pub const PWM_2_FLTSTAT1_DCMP2 = usize(0x00000004);  // Digital Comparator 2 Trigger
pub const PWM_2_FLTSTAT1_DCMP1 = usize(0x00000002);  // Digital Comparator 1 Trigger
pub const PWM_2_FLTSTAT1_DCMP0 = usize(0x00000001);  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSEN register.
//
//*****************************************************************************
pub const PWM_3_FLTSEN_FAULT3 = usize(0x00000008);  // Fault3 Sense
pub const PWM_3_FLTSEN_FAULT2 = usize(0x00000004);  // Fault2 Sense
pub const PWM_3_FLTSEN_FAULT1 = usize(0x00000002);  // Fault1 Sense
pub const PWM_3_FLTSEN_FAULT0 = usize(0x00000001);  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSTAT0
// register.
//
//*****************************************************************************
pub const PWM_3_FLTSTAT0_FAULT3 = usize(0x00000008);  // Fault Input 3
pub const PWM_3_FLTSTAT0_FAULT2 = usize(0x00000004);  // Fault Input 2
pub const PWM_3_FLTSTAT0_FAULT1 = usize(0x00000002);  // Fault Input 1
pub const PWM_3_FLTSTAT0_FAULT0 = usize(0x00000001);  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSTAT1
// register.
//
//*****************************************************************************
pub const PWM_3_FLTSTAT1_DCMP7 = usize(0x00000080);  // Digital Comparator 7 Trigger
pub const PWM_3_FLTSTAT1_DCMP6 = usize(0x00000040);  // Digital Comparator 6 Trigger
pub const PWM_3_FLTSTAT1_DCMP5 = usize(0x00000020);  // Digital Comparator 5 Trigger
pub const PWM_3_FLTSTAT1_DCMP4 = usize(0x00000010);  // Digital Comparator 4 Trigger
pub const PWM_3_FLTSTAT1_DCMP3 = usize(0x00000008);  // Digital Comparator 3 Trigger
pub const PWM_3_FLTSTAT1_DCMP2 = usize(0x00000004);  // Digital Comparator 2 Trigger
pub const PWM_3_FLTSTAT1_DCMP1 = usize(0x00000002);  // Digital Comparator 1 Trigger
pub const PWM_3_FLTSTAT1_DCMP0 = usize(0x00000001);  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_PP register.
//
//*****************************************************************************
pub const PWM_PP_ONE = usize(0x00000400);  // One-Shot Mode
pub const PWM_PP_EFAULT = usize(0x00000200);  // Extended Fault
pub const PWM_PP_ESYNC = usize(0x00000100);  // Extended Synchronization
pub const PWM_PP_FCNT_M = usize(0x000000F0);  // Fault Inputs (per PWM unit)
pub const PWM_PP_GCNT_M = usize(0x0000000F);  // Generators
pub const PWM_PP_FCNT_S = usize(4);
pub const PWM_PP_GCNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_CC register.
//
//*****************************************************************************
pub const PWM_CC_USEPWM = usize(0x00000100);  // Use PWM Clock Divisor
pub const PWM_CC_PWMDIV_M = usize(0x00000007);  // PWM Clock Divider
pub const PWM_CC_PWMDIV_2 = usize(0x00000000);  // /2
pub const PWM_CC_PWMDIV_4 = usize(0x00000001);  // /4
pub const PWM_CC_PWMDIV_8 = usize(0x00000002);  // /8
pub const PWM_CC_PWMDIV_16 = usize(0x00000003);  // /16
pub const PWM_CC_PWMDIV_32 = usize(0x00000004);  // /32
pub const PWM_CC_PWMDIV_64 = usize(0x00000005);  // /64

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_CTL register.
//
//*****************************************************************************
pub const QEI_CTL_FILTCNT_M = usize(0x000F0000);  // Input Filter Prescale Count
pub const QEI_CTL_FILTEN = usize(0x00002000);  // Enable Input Filter
pub const QEI_CTL_STALLEN = usize(0x00001000);  // Stall QEI
pub const QEI_CTL_INVI = usize(0x00000800);  // Invert Index Pulse
pub const QEI_CTL_INVB = usize(0x00000400);  // Invert PhB
pub const QEI_CTL_INVA = usize(0x00000200);  // Invert PhA
pub const QEI_CTL_VELDIV_M = usize(0x000001C0);  // Predivide Velocity
pub const QEI_CTL_VELDIV_1 = usize(0x00000000);  // QEI clock /1
pub const QEI_CTL_VELDIV_2 = usize(0x00000040);  // QEI clock /2
pub const QEI_CTL_VELDIV_4 = usize(0x00000080);  // QEI clock /4
pub const QEI_CTL_VELDIV_8 = usize(0x000000C0);  // QEI clock /8
pub const QEI_CTL_VELDIV_16 = usize(0x00000100);  // QEI clock /16
pub const QEI_CTL_VELDIV_32 = usize(0x00000140);  // QEI clock /32
pub const QEI_CTL_VELDIV_64 = usize(0x00000180);  // QEI clock /64
pub const QEI_CTL_VELDIV_128 = usize(0x000001C0);  // QEI clock /128
pub const QEI_CTL_VELEN = usize(0x00000020);  // Capture Velocity
pub const QEI_CTL_RESMODE = usize(0x00000010);  // Reset Mode
pub const QEI_CTL_CAPMODE = usize(0x00000008);  // Capture Mode
pub const QEI_CTL_SIGMODE = usize(0x00000004);  // Signal Mode
pub const QEI_CTL_SWAP = usize(0x00000002);  // Swap Signals
pub const QEI_CTL_ENABLE = usize(0x00000001);  // Enable QEI
pub const QEI_CTL_FILTCNT_S = usize(16);

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_STAT register.
//
//*****************************************************************************
pub const QEI_STAT_DIRECTION = usize(0x00000002);  // Direction of Rotation
pub const QEI_STAT_ERROR = usize(0x00000001);  // Error Detected

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_POS register.
//
//*****************************************************************************
pub const QEI_POS_M = usize(0xFFFFFFFF);  // Current Position Integrator
                                            // Value
pub const QEI_POS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_MAXPOS register.
//
//*****************************************************************************
pub const QEI_MAXPOS_M = usize(0xFFFFFFFF);  // Maximum Position Integrator
                                            // Value
pub const QEI_MAXPOS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_LOAD register.
//
//*****************************************************************************
pub const QEI_LOAD_M = usize(0xFFFFFFFF);  // Velocity Timer Load Value
pub const QEI_LOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_TIME register.
//
//*****************************************************************************
pub const QEI_TIME_M = usize(0xFFFFFFFF);  // Velocity Timer Current Value
pub const QEI_TIME_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_COUNT register.
//
//*****************************************************************************
pub const QEI_COUNT_M = usize(0xFFFFFFFF);  // Velocity Pulse Count
pub const QEI_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_SPEED register.
//
//*****************************************************************************
pub const QEI_SPEED_M = usize(0xFFFFFFFF);  // Velocity
pub const QEI_SPEED_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_INTEN register.
//
//*****************************************************************************
pub const QEI_INTEN_ERROR = usize(0x00000008);  // Phase Error Interrupt Enable
pub const QEI_INTEN_DIR = usize(0x00000004);  // Direction Change Interrupt
                                            // Enable
pub const QEI_INTEN_TIMER = usize(0x00000002);  // Timer Expires Interrupt Enable
pub const QEI_INTEN_INDEX = usize(0x00000001);  // Index Pulse Detected Interrupt
                                            // Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_RIS register.
//
//*****************************************************************************
pub const QEI_RIS_ERROR = usize(0x00000008);  // Phase Error Detected
pub const QEI_RIS_DIR = usize(0x00000004);  // Direction Change Detected
pub const QEI_RIS_TIMER = usize(0x00000002);  // Velocity Timer Expired
pub const QEI_RIS_INDEX = usize(0x00000001);  // Index Pulse Asserted

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_ISC register.
//
//*****************************************************************************
pub const QEI_ISC_ERROR = usize(0x00000008);  // Phase Error Interrupt
pub const QEI_ISC_DIR = usize(0x00000004);  // Direction Change Interrupt
pub const QEI_ISC_TIMER = usize(0x00000002);  // Velocity Timer Expired Interrupt
pub const QEI_ISC_INDEX = usize(0x00000001);  // Index Pulse Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CFG register.
//
//*****************************************************************************
pub const TIMER_CFG_M = usize(0x00000007);  // GPTM Configuration
pub const TIMER_CFG_32_BIT_TIMER = usize(0x00000000);  // For a 16/32-bit timer, this
                                            // value selects the 32-bit timer
                                            // configuration
pub const TIMER_CFG_32_BIT_RTC = usize(0x00000001);  // For a 16/32-bit timer, this
                                            // value selects the 32-bit
                                            // real-time clock (RTC) counter
                                            // configuration
pub const TIMER_CFG_16_BIT = usize(0x00000004);  // For a 16/32-bit timer, this
                                            // value selects the 16-bit timer
                                            // configuration

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMR register.
//
//*****************************************************************************
pub const TIMER_TAMR_TCACT_M = usize(0x0000E000);  // Timer Compare Action Select
pub const TIMER_TAMR_TCACT_NONE = usize(0x00000000);  // Disable compare operations
pub const TIMER_TAMR_TCACT_TOGGLE = usize(0x00002000);  // Toggle State on Time-Out
pub const TIMER_TAMR_TCACT_CLRTO = usize(0x00004000);  // Clear CCP on Time-Out
pub const TIMER_TAMR_TCACT_SETTO = usize(0x00006000);  // Set CCP on Time-Out
pub const TIMER_TAMR_TCACT_SETTOGTO = usize(0x00008000);  // Set CCP immediately and toggle
                                            // on Time-Out
pub const TIMER_TAMR_TCACT_CLRTOGTO = usize(0x0000A000);  // Clear CCP immediately and toggle
                                            // on Time-Out
pub const TIMER_TAMR_TCACT_SETCLRTO = usize(0x0000C000);  // Set CCP immediately and clear on
                                            // Time-Out
pub const TIMER_TAMR_TCACT_CLRSETTO = usize(0x0000E000);  // Clear CCP immediately and set on
                                            // Time-Out
pub const TIMER_TAMR_TACINTD = usize(0x00001000);  // One-shot/Periodic Interrupt
                                            // Disable
pub const TIMER_TAMR_TAPLO = usize(0x00000800);  // GPTM Timer A PWM Legacy
                                            // Operation
pub const TIMER_TAMR_TAMRSU = usize(0x00000400);  // GPTM Timer A Match Register
                                            // Update
pub const TIMER_TAMR_TAPWMIE = usize(0x00000200);  // GPTM Timer A PWM Interrupt
                                            // Enable
pub const TIMER_TAMR_TAILD = usize(0x00000100);  // GPTM Timer A Interval Load Write
pub const TIMER_TAMR_TASNAPS = usize(0x00000080);  // GPTM Timer A Snap-Shot Mode
pub const TIMER_TAMR_TAWOT = usize(0x00000040);  // GPTM Timer A Wait-on-Trigger
pub const TIMER_TAMR_TAMIE = usize(0x00000020);  // GPTM Timer A Match Interrupt
                                            // Enable
pub const TIMER_TAMR_TACDIR = usize(0x00000010);  // GPTM Timer A Count Direction
pub const TIMER_TAMR_TAAMS = usize(0x00000008);  // GPTM Timer A Alternate Mode
                                            // Select
pub const TIMER_TAMR_TACMR = usize(0x00000004);  // GPTM Timer A Capture Mode
pub const TIMER_TAMR_TAMR_M = usize(0x00000003);  // GPTM Timer A Mode
pub const TIMER_TAMR_TAMR_1_SHOT = usize(0x00000001);  // One-Shot Timer mode
pub const TIMER_TAMR_TAMR_PERIOD = usize(0x00000002);  // Periodic Timer mode
pub const TIMER_TAMR_TAMR_CAP = usize(0x00000003);  // Capture mode

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMR register.
//
//*****************************************************************************
pub const TIMER_TBMR_TCACT_M = usize(0x0000E000);  // Timer Compare Action Select
pub const TIMER_TBMR_TCACT_NONE = usize(0x00000000);  // Disable compare operations
pub const TIMER_TBMR_TCACT_TOGGLE = usize(0x00002000);  // Toggle State on Time-Out
pub const TIMER_TBMR_TCACT_CLRTO = usize(0x00004000);  // Clear CCP on Time-Out
pub const TIMER_TBMR_TCACT_SETTO = usize(0x00006000);  // Set CCP on Time-Out
pub const TIMER_TBMR_TCACT_SETTOGTO = usize(0x00008000);  // Set CCP immediately and toggle
                                            // on Time-Out
pub const TIMER_TBMR_TCACT_CLRTOGTO = usize(0x0000A000);  // Clear CCP immediately and toggle
                                            // on Time-Out
pub const TIMER_TBMR_TCACT_SETCLRTO = usize(0x0000C000);  // Set CCP immediately and clear on
                                            // Time-Out
pub const TIMER_TBMR_TCACT_CLRSETTO = usize(0x0000E000);  // Clear CCP immediately and set on
                                            // Time-Out
pub const TIMER_TBMR_TBCINTD = usize(0x00001000);  // One-Shot/Periodic Interrupt
                                            // Disable
pub const TIMER_TBMR_TBPLO = usize(0x00000800);  // GPTM Timer B PWM Legacy
                                            // Operation
pub const TIMER_TBMR_TBMRSU = usize(0x00000400);  // GPTM Timer B Match Register
                                            // Update
pub const TIMER_TBMR_TBPWMIE = usize(0x00000200);  // GPTM Timer B PWM Interrupt
                                            // Enable
pub const TIMER_TBMR_TBILD = usize(0x00000100);  // GPTM Timer B Interval Load Write
pub const TIMER_TBMR_TBSNAPS = usize(0x00000080);  // GPTM Timer B Snap-Shot Mode
pub const TIMER_TBMR_TBWOT = usize(0x00000040);  // GPTM Timer B Wait-on-Trigger
pub const TIMER_TBMR_TBMIE = usize(0x00000020);  // GPTM Timer B Match Interrupt
                                            // Enable
pub const TIMER_TBMR_TBCDIR = usize(0x00000010);  // GPTM Timer B Count Direction
pub const TIMER_TBMR_TBAMS = usize(0x00000008);  // GPTM Timer B Alternate Mode
                                            // Select
pub const TIMER_TBMR_TBCMR = usize(0x00000004);  // GPTM Timer B Capture Mode
pub const TIMER_TBMR_TBMR_M = usize(0x00000003);  // GPTM Timer B Mode
pub const TIMER_TBMR_TBMR_1_SHOT = usize(0x00000001);  // One-Shot Timer mode
pub const TIMER_TBMR_TBMR_PERIOD = usize(0x00000002);  // Periodic Timer mode
pub const TIMER_TBMR_TBMR_CAP = usize(0x00000003);  // Capture mode

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CTL register.
//
//*****************************************************************************
pub const TIMER_CTL_TBPWML = usize(0x00004000);  // GPTM Timer B PWM Output Level
pub const TIMER_CTL_TBOTE = usize(0x00002000);  // GPTM Timer B Output Trigger
                                            // Enable
pub const TIMER_CTL_TBEVENT_M = usize(0x00000C00);  // GPTM Timer B Event Mode
pub const TIMER_CTL_TBEVENT_POS = usize(0x00000000);  // Positive edge
pub const TIMER_CTL_TBEVENT_NEG = usize(0x00000400);  // Negative edge
pub const TIMER_CTL_TBEVENT_BOTH = usize(0x00000C00);  // Both edges
pub const TIMER_CTL_TBSTALL = usize(0x00000200);  // GPTM Timer B Stall Enable
pub const TIMER_CTL_TBEN = usize(0x00000100);  // GPTM Timer B Enable
pub const TIMER_CTL_TAPWML = usize(0x00000040);  // GPTM Timer A PWM Output Level
pub const TIMER_CTL_TAOTE = usize(0x00000020);  // GPTM Timer A Output Trigger
                                            // Enable
pub const TIMER_CTL_RTCEN = usize(0x00000010);  // GPTM RTC Stall Enable
pub const TIMER_CTL_TAEVENT_M = usize(0x0000000C);  // GPTM Timer A Event Mode
pub const TIMER_CTL_TAEVENT_POS = usize(0x00000000);  // Positive edge
pub const TIMER_CTL_TAEVENT_NEG = usize(0x00000004);  // Negative edge
pub const TIMER_CTL_TAEVENT_BOTH = usize(0x0000000C);  // Both edges
pub const TIMER_CTL_TASTALL = usize(0x00000002);  // GPTM Timer A Stall Enable
pub const TIMER_CTL_TAEN = usize(0x00000001);  // GPTM Timer A Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_SYNC register.
//
//*****************************************************************************
pub const TIMER_SYNC_SYNCT7_M = usize(0x0000C000);  // Synchronize GPTM Timer 7
pub const TIMER_SYNC_SYNCT7_NONE = usize(0x00000000);  // GPT7 is not affected
pub const TIMER_SYNC_SYNCT7_TA = usize(0x00004000);  // A timeout event for Timer A of
                                            // GPTM7 is triggered
pub const TIMER_SYNC_SYNCT7_TB = usize(0x00008000);  // A timeout event for Timer B of
                                            // GPTM7 is triggered
pub const TIMER_SYNC_SYNCT7_TATB = usize(0x0000C000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM7 is
                                            // triggered
pub const TIMER_SYNC_SYNCT6_M = usize(0x00003000);  // Synchronize GPTM Timer 6
pub const TIMER_SYNC_SYNCT6_NONE = usize(0x00000000);  // GPTM6 is not affected
pub const TIMER_SYNC_SYNCT6_TA = usize(0x00001000);  // A timeout event for Timer A of
                                            // GPTM6 is triggered
pub const TIMER_SYNC_SYNCT6_TB = usize(0x00002000);  // A timeout event for Timer B of
                                            // GPTM6 is triggered
pub const TIMER_SYNC_SYNCT6_TATB = usize(0x00003000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM6 is
                                            // triggered
pub const TIMER_SYNC_SYNCT5_M = usize(0x00000C00);  // Synchronize GPTM Timer 5
pub const TIMER_SYNC_SYNCT5_NONE = usize(0x00000000);  // GPTM5 is not affected
pub const TIMER_SYNC_SYNCT5_TA = usize(0x00000400);  // A timeout event for Timer A of
                                            // GPTM5 is triggered
pub const TIMER_SYNC_SYNCT5_TB = usize(0x00000800);  // A timeout event for Timer B of
                                            // GPTM5 is triggered
pub const TIMER_SYNC_SYNCT5_TATB = usize(0x00000C00);  // A timeout event for both Timer A
                                            // and Timer B of GPTM5 is
                                            // triggered
pub const TIMER_SYNC_SYNCT4_M = usize(0x00000300);  // Synchronize GPTM Timer 4
pub const TIMER_SYNC_SYNCT4_NONE = usize(0x00000000);  // GPTM4 is not affected
pub const TIMER_SYNC_SYNCT4_TA = usize(0x00000100);  // A timeout event for Timer A of
                                            // GPTM4 is triggered
pub const TIMER_SYNC_SYNCT4_TB = usize(0x00000200);  // A timeout event for Timer B of
                                            // GPTM4 is triggered
pub const TIMER_SYNC_SYNCT4_TATB = usize(0x00000300);  // A timeout event for both Timer A
                                            // and Timer B of GPTM4 is
                                            // triggered
pub const TIMER_SYNC_SYNCT3_M = usize(0x000000C0);  // Synchronize GPTM Timer 3
pub const TIMER_SYNC_SYNCT3_NONE = usize(0x00000000);  // GPTM3 is not affected
pub const TIMER_SYNC_SYNCT3_TA = usize(0x00000040);  // A timeout event for Timer A of
                                            // GPTM3 is triggered
pub const TIMER_SYNC_SYNCT3_TB = usize(0x00000080);  // A timeout event for Timer B of
                                            // GPTM3 is triggered
pub const TIMER_SYNC_SYNCT3_TATB = usize(0x000000C0);  // A timeout event for both Timer A
                                            // and Timer B of GPTM3 is
                                            // triggered
pub const TIMER_SYNC_SYNCT2_M = usize(0x00000030);  // Synchronize GPTM Timer 2
pub const TIMER_SYNC_SYNCT2_NONE = usize(0x00000000);  // GPTM2 is not affected
pub const TIMER_SYNC_SYNCT2_TA = usize(0x00000010);  // A timeout event for Timer A of
                                            // GPTM2 is triggered
pub const TIMER_SYNC_SYNCT2_TB = usize(0x00000020);  // A timeout event for Timer B of
                                            // GPTM2 is triggered
pub const TIMER_SYNC_SYNCT2_TATB = usize(0x00000030);  // A timeout event for both Timer A
                                            // and Timer B of GPTM2 is
                                            // triggered
pub const TIMER_SYNC_SYNCT1_M = usize(0x0000000C);  // Synchronize GPTM Timer 1
pub const TIMER_SYNC_SYNCT1_NONE = usize(0x00000000);  // GPTM1 is not affected
pub const TIMER_SYNC_SYNCT1_TA = usize(0x00000004);  // A timeout event for Timer A of
                                            // GPTM1 is triggered
pub const TIMER_SYNC_SYNCT1_TB = usize(0x00000008);  // A timeout event for Timer B of
                                            // GPTM1 is triggered
pub const TIMER_SYNC_SYNCT1_TATB = usize(0x0000000C);  // A timeout event for both Timer A
                                            // and Timer B of GPTM1 is
                                            // triggered
pub const TIMER_SYNC_SYNCT0_M = usize(0x00000003);  // Synchronize GPTM Timer 0
pub const TIMER_SYNC_SYNCT0_NONE = usize(0x00000000);  // GPTM0 is not affected
pub const TIMER_SYNC_SYNCT0_TA = usize(0x00000001);  // A timeout event for Timer A of
                                            // GPTM0 is triggered
pub const TIMER_SYNC_SYNCT0_TB = usize(0x00000002);  // A timeout event for Timer B of
                                            // GPTM0 is triggered
pub const TIMER_SYNC_SYNCT0_TATB = usize(0x00000003);  // A timeout event for both Timer A
                                            // and Timer B of GPTM0 is
                                            // triggered

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_IMR register.
//
//*****************************************************************************
pub const TIMER_IMR_DMABIM = usize(0x00002000);  // GPTM Timer B DMA Done Interrupt
                                            // Mask
pub const TIMER_IMR_TBMIM = usize(0x00000800);  // GPTM Timer B Match Interrupt
                                            // Mask
pub const TIMER_IMR_CBEIM = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Interrupt Mask
pub const TIMER_IMR_CBMIM = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Interrupt Mask
pub const TIMER_IMR_TBTOIM = usize(0x00000100);  // GPTM Timer B Time-Out Interrupt
                                            // Mask
pub const TIMER_IMR_DMAAIM = usize(0x00000020);  // GPTM Timer A DMA Done Interrupt
                                            // Mask
pub const TIMER_IMR_TAMIM = usize(0x00000010);  // GPTM Timer A Match Interrupt
                                            // Mask
pub const TIMER_IMR_RTCIM = usize(0x00000008);  // GPTM RTC Interrupt Mask
pub const TIMER_IMR_CAEIM = usize(0x00000004);  // GPTM Timer A Capture Mode Event
                                            // Interrupt Mask
pub const TIMER_IMR_CAMIM = usize(0x00000002);  // GPTM Timer A Capture Mode Match
                                            // Interrupt Mask
pub const TIMER_IMR_TATOIM = usize(0x00000001);  // GPTM Timer A Time-Out Interrupt
                                            // Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_RIS register.
//
//*****************************************************************************
pub const TIMER_RIS_DMABRIS = usize(0x00002000);  // GPTM Timer B DMA Done Raw
                                            // Interrupt Status
pub const TIMER_RIS_TBMRIS = usize(0x00000800);  // GPTM Timer B Match Raw Interrupt
pub const TIMER_RIS_CBERIS = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Raw Interrupt
pub const TIMER_RIS_CBMRIS = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Raw Interrupt
pub const TIMER_RIS_TBTORIS = usize(0x00000100);  // GPTM Timer B Time-Out Raw
                                            // Interrupt
pub const TIMER_RIS_DMAARIS = usize(0x00000020);  // GPTM Timer A DMA Done Raw
                                            // Interrupt Status
pub const TIMER_RIS_TAMRIS = usize(0x00000010);  // GPTM Timer A Match Raw Interrupt
pub const TIMER_RIS_RTCRIS = usize(0x00000008);  // GPTM RTC Raw Interrupt
pub const TIMER_RIS_CAERIS = usize(0x00000004);  // GPTM Timer A Capture Mode Event
                                            // Raw Interrupt
pub const TIMER_RIS_CAMRIS = usize(0x00000002);  // GPTM Timer A Capture Mode Match
                                            // Raw Interrupt
pub const TIMER_RIS_TATORIS = usize(0x00000001);  // GPTM Timer A Time-Out Raw
                                            // Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_MIS register.
//
//*****************************************************************************
pub const TIMER_MIS_DMABMIS = usize(0x00002000);  // GPTM Timer B DMA Done Masked
                                            // Interrupt
pub const TIMER_MIS_TBMMIS = usize(0x00000800);  // GPTM Timer B Match Masked
                                            // Interrupt
pub const TIMER_MIS_CBEMIS = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Masked Interrupt
pub const TIMER_MIS_CBMMIS = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Masked Interrupt
pub const TIMER_MIS_TBTOMIS = usize(0x00000100);  // GPTM Timer B Time-Out Masked
                                            // Interrupt
pub const TIMER_MIS_DMAAMIS = usize(0x00000020);  // GPTM Timer A DMA Done Masked
                                            // Interrupt
pub const TIMER_MIS_TAMMIS = usize(0x00000010);  // GPTM Timer A Match Masked
                                            // Interrupt
pub const TIMER_MIS_RTCMIS = usize(0x00000008);  // GPTM RTC Masked Interrupt
pub const TIMER_MIS_CAEMIS = usize(0x00000004);  // GPTM Timer A Capture Mode Event
                                            // Masked Interrupt
pub const TIMER_MIS_CAMMIS = usize(0x00000002);  // GPTM Timer A Capture Mode Match
                                            // Masked Interrupt
pub const TIMER_MIS_TATOMIS = usize(0x00000001);  // GPTM Timer A Time-Out Masked
                                            // Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_ICR register.
//
//*****************************************************************************
pub const TIMER_ICR_DMABINT = usize(0x00002000);  // GPTM Timer B DMA Done Interrupt
                                            // Clear
pub const TIMER_ICR_TBMCINT = usize(0x00000800);  // GPTM Timer B Match Interrupt
                                            // Clear
pub const TIMER_ICR_CBECINT = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Interrupt Clear
pub const TIMER_ICR_CBMCINT = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Interrupt Clear
pub const TIMER_ICR_TBTOCINT = usize(0x00000100);  // GPTM Timer B Time-Out Interrupt
                                            // Clear
pub const TIMER_ICR_DMAAINT = usize(0x00000020);  // GPTM Timer A DMA Done Interrupt
                                            // Clear
pub const TIMER_ICR_TAMCINT = usize(0x00000010);  // GPTM Timer A Match Interrupt
                                            // Clear
pub const TIMER_ICR_RTCCINT = usize(0x00000008);  // GPTM RTC Interrupt Clear
pub const TIMER_ICR_CAECINT = usize(0x00000004);  // GPTM Timer A Capture Mode Event
                                            // Interrupt Clear
pub const TIMER_ICR_CAMCINT = usize(0x00000002);  // GPTM Timer A Capture Mode Match
                                            // Interrupt Clear
pub const TIMER_ICR_TATOCINT = usize(0x00000001);  // GPTM Timer A Time-Out Raw
                                            // Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAILR register.
//
//*****************************************************************************
pub const TIMER_TAILR_M = usize(0xFFFFFFFF);  // GPTM Timer A Interval Load
                                            // Register
pub const TIMER_TAILR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBILR register.
//
//*****************************************************************************
pub const TIMER_TBILR_M = usize(0xFFFFFFFF);  // GPTM Timer B Interval Load
                                            // Register
pub const TIMER_TBILR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMATCHR
// register.
//
//*****************************************************************************
pub const TIMER_TAMATCHR_TAMR_M = usize(0xFFFFFFFF);  // GPTM Timer A Match Register
pub const TIMER_TAMATCHR_TAMR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMATCHR
// register.
//
//*****************************************************************************
pub const TIMER_TBMATCHR_TBMR_M = usize(0xFFFFFFFF);  // GPTM Timer B Match Register
pub const TIMER_TBMATCHR_TBMR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPR register.
//
//*****************************************************************************
pub const TIMER_TAPR_TAPSR_M = usize(0x000000FF);  // GPTM Timer A Prescale
pub const TIMER_TAPR_TAPSR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPR register.
//
//*****************************************************************************
pub const TIMER_TBPR_TBPSR_M = usize(0x000000FF);  // GPTM Timer B Prescale
pub const TIMER_TBPR_TBPSR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPMR register.
//
//*****************************************************************************
pub const TIMER_TAPMR_TAPSMR_M = usize(0x000000FF);  // GPTM TimerA Prescale Match
pub const TIMER_TAPMR_TAPSMR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPMR register.
//
//*****************************************************************************
pub const TIMER_TBPMR_TBPSMR_M = usize(0x000000FF);  // GPTM TimerB Prescale Match
pub const TIMER_TBPMR_TBPSMR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAR register.
//
//*****************************************************************************
pub const TIMER_TAR_M = usize(0xFFFFFFFF);  // GPTM Timer A Register
pub const TIMER_TAR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBR register.
//
//*****************************************************************************
pub const TIMER_TBR_M = usize(0xFFFFFFFF);  // GPTM Timer B Register
pub const TIMER_TBR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAV register.
//
//*****************************************************************************
pub const TIMER_TAV_M = usize(0xFFFFFFFF);  // GPTM Timer A Value
pub const TIMER_TAV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBV register.
//
//*****************************************************************************
pub const TIMER_TBV_M = usize(0xFFFFFFFF);  // GPTM Timer B Value
pub const TIMER_TBV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_RTCPD register.
//
//*****************************************************************************
pub const TIMER_RTCPD_RTCPD_M = usize(0x0000FFFF);  // RTC Predivide Counter Value
pub const TIMER_RTCPD_RTCPD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPS register.
//
//*****************************************************************************
pub const TIMER_TAPS_PSS_M = usize(0x0000FFFF);  // GPTM Timer A Prescaler Snapshot
pub const TIMER_TAPS_PSS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPS register.
//
//*****************************************************************************
pub const TIMER_TBPS_PSS_M = usize(0x0000FFFF);  // GPTM Timer A Prescaler Value
pub const TIMER_TBPS_PSS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_DMAEV register.
//
//*****************************************************************************
pub const TIMER_DMAEV_TBMDMAEN = usize(0x00000800);  // GPTM B Mode Match Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_CBEDMAEN = usize(0x00000400);  // GPTM B Capture Event DMA Trigger
                                            // Enable
pub const TIMER_DMAEV_CBMDMAEN = usize(0x00000200);  // GPTM B Capture Match Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_TBTODMAEN = usize(0x00000100);  // GPTM B Time-Out Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_TAMDMAEN = usize(0x00000010);  // GPTM A Mode Match Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_RTCDMAEN = usize(0x00000008);  // GPTM A RTC Match Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_CAEDMAEN = usize(0x00000004);  // GPTM A Capture Event DMA Trigger
                                            // Enable
pub const TIMER_DMAEV_CAMDMAEN = usize(0x00000002);  // GPTM A Capture Match Event DMA
                                            // Trigger Enable
pub const TIMER_DMAEV_TATODMAEN = usize(0x00000001);  // GPTM A Time-Out Event DMA
                                            // Trigger Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_ADCEV register.
//
//*****************************************************************************
pub const TIMER_ADCEV_TBMADCEN = usize(0x00000800);  // GPTM B Mode Match Event ADC
                                            // Trigger Enable
pub const TIMER_ADCEV_CBEADCEN = usize(0x00000400);  // GPTM B Capture Event ADC Trigger
                                            // Enable
pub const TIMER_ADCEV_CBMADCEN = usize(0x00000200);  // GPTM B Capture Match Event ADC
                                            // Trigger Enable
pub const TIMER_ADCEV_TBTOADCEN = usize(0x00000100);  // GPTM B Time-Out Event ADC
                                            // Trigger Enable
pub const TIMER_ADCEV_TAMADCEN = usize(0x00000010);  // GPTM A Mode Match Event ADC
                                            // Trigger Enable
pub const TIMER_ADCEV_RTCADCEN = usize(0x00000008);  // GPTM RTC Match Event ADC Trigger
                                            // Enable
pub const TIMER_ADCEV_CAEADCEN = usize(0x00000004);  // GPTM A Capture Event ADC Trigger
                                            // Enable
pub const TIMER_ADCEV_CAMADCEN = usize(0x00000002);  // GPTM A Capture Match Event ADC
                                            // Trigger Enable
pub const TIMER_ADCEV_TATOADCEN = usize(0x00000001);  // GPTM A Time-Out Event ADC
                                            // Trigger Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_PP register.
//
//*****************************************************************************
pub const TIMER_PP_ALTCLK = usize(0x00000040);  // Alternate Clock Source
pub const TIMER_PP_SYNCCNT = usize(0x00000020);  // Synchronize Start
pub const TIMER_PP_CHAIN = usize(0x00000010);  // Chain with Other Timers
pub const TIMER_PP_SIZE_M = usize(0x0000000F);  // Count Size
pub const TIMER_PP_SIZE_16 = usize(0x00000000);  // Timer A and Timer B counters are
                                            // 16 bits each with an 8-bit
                                            // prescale counter
pub const TIMER_PP_SIZE_32 = usize(0x00000001);  // Timer A and Timer B counters are
                                            // 32 bits each with a 16-bit
                                            // prescale counter

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CC register.
//
//*****************************************************************************
pub const TIMER_CC_ALTCLK = usize(0x00000001);  // Alternate Clock Source

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_ACTSS register.
//
//*****************************************************************************
pub const ADC_ACTSS_BUSY = usize(0x00010000);  // ADC Busy
pub const ADC_ACTSS_ADEN3 = usize(0x00000800);  // ADC SS3 DMA Enable
pub const ADC_ACTSS_ADEN2 = usize(0x00000400);  // ADC SS2 DMA Enable
pub const ADC_ACTSS_ADEN1 = usize(0x00000200);  // ADC SS1 DMA Enable
pub const ADC_ACTSS_ADEN0 = usize(0x00000100);  // ADC SS1 DMA Enable
pub const ADC_ACTSS_ASEN3 = usize(0x00000008);  // ADC SS3 Enable
pub const ADC_ACTSS_ASEN2 = usize(0x00000004);  // ADC SS2 Enable
pub const ADC_ACTSS_ASEN1 = usize(0x00000002);  // ADC SS1 Enable
pub const ADC_ACTSS_ASEN0 = usize(0x00000001);  // ADC SS0 Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_RIS register.
//
//*****************************************************************************
pub const ADC_RIS_INRDC = usize(0x00010000);  // Digital Comparator Raw Interrupt
                                            // Status
pub const ADC_RIS_DMAINR3 = usize(0x00000800);  // SS3 DMA Raw Interrupt Status
pub const ADC_RIS_DMAINR2 = usize(0x00000400);  // SS2 DMA Raw Interrupt Status
pub const ADC_RIS_DMAINR1 = usize(0x00000200);  // SS1 DMA Raw Interrupt Status
pub const ADC_RIS_DMAINR0 = usize(0x00000100);  // SS0 DMA Raw Interrupt Status
pub const ADC_RIS_INR3 = usize(0x00000008);  // SS3 Raw Interrupt Status
pub const ADC_RIS_INR2 = usize(0x00000004);  // SS2 Raw Interrupt Status
pub const ADC_RIS_INR1 = usize(0x00000002);  // SS1 Raw Interrupt Status
pub const ADC_RIS_INR0 = usize(0x00000001);  // SS0 Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_IM register.
//
//*****************************************************************************
pub const ADC_IM_DCONSS3 = usize(0x00080000);  // Digital Comparator Interrupt on
                                            // SS3
pub const ADC_IM_DCONSS2 = usize(0x00040000);  // Digital Comparator Interrupt on
                                            // SS2
pub const ADC_IM_DCONSS1 = usize(0x00020000);  // Digital Comparator Interrupt on
                                            // SS1
pub const ADC_IM_DCONSS0 = usize(0x00010000);  // Digital Comparator Interrupt on
                                            // SS0
pub const ADC_IM_DMAMASK3 = usize(0x00000800);  // SS3 DMA Interrupt Mask
pub const ADC_IM_DMAMASK2 = usize(0x00000400);  // SS2 DMA Interrupt Mask
pub const ADC_IM_DMAMASK1 = usize(0x00000200);  // SS1 DMA Interrupt Mask
pub const ADC_IM_DMAMASK0 = usize(0x00000100);  // SS0 DMA Interrupt Mask
pub const ADC_IM_MASK3 = usize(0x00000008);  // SS3 Interrupt Mask
pub const ADC_IM_MASK2 = usize(0x00000004);  // SS2 Interrupt Mask
pub const ADC_IM_MASK1 = usize(0x00000002);  // SS1 Interrupt Mask
pub const ADC_IM_MASK0 = usize(0x00000001);  // SS0 Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_ISC register.
//
//*****************************************************************************
pub const ADC_ISC_DCINSS3 = usize(0x00080000);  // Digital Comparator Interrupt
                                            // Status on SS3
pub const ADC_ISC_DCINSS2 = usize(0x00040000);  // Digital Comparator Interrupt
                                            // Status on SS2
pub const ADC_ISC_DCINSS1 = usize(0x00020000);  // Digital Comparator Interrupt
                                            // Status on SS1
pub const ADC_ISC_DCINSS0 = usize(0x00010000);  // Digital Comparator Interrupt
                                            // Status on SS0
pub const ADC_ISC_DMAIN3 = usize(0x00000800);  // SS3 DMA Interrupt Status and
                                            // Clear
pub const ADC_ISC_DMAIN2 = usize(0x00000400);  // SS2 DMA Interrupt Status and
                                            // Clear
pub const ADC_ISC_DMAIN1 = usize(0x00000200);  // SS1 DMA Interrupt Status and
                                            // Clear
pub const ADC_ISC_DMAIN0 = usize(0x00000100);  // SS0 DMA Interrupt Status and
                                            // Clear
pub const ADC_ISC_IN3 = usize(0x00000008);  // SS3 Interrupt Status and Clear
pub const ADC_ISC_IN2 = usize(0x00000004);  // SS2 Interrupt Status and Clear
pub const ADC_ISC_IN1 = usize(0x00000002);  // SS1 Interrupt Status and Clear
pub const ADC_ISC_IN0 = usize(0x00000001);  // SS0 Interrupt Status and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_OSTAT register.
//
//*****************************************************************************
pub const ADC_OSTAT_OV3 = usize(0x00000008);  // SS3 FIFO Overflow
pub const ADC_OSTAT_OV2 = usize(0x00000004);  // SS2 FIFO Overflow
pub const ADC_OSTAT_OV1 = usize(0x00000002);  // SS1 FIFO Overflow
pub const ADC_OSTAT_OV0 = usize(0x00000001);  // SS0 FIFO Overflow

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_EMUX register.
//
//*****************************************************************************
pub const ADC_EMUX_EM3_M = usize(0x0000F000);  // SS3 Trigger Select
pub const ADC_EMUX_EM3_PROCESSOR = usize(0x00000000);  // Processor (default)
pub const ADC_EMUX_EM3_COMP0 = usize(0x00001000);  // Analog Comparator 0
pub const ADC_EMUX_EM3_COMP1 = usize(0x00002000);  // Analog Comparator 1
pub const ADC_EMUX_EM3_COMP2 = usize(0x00003000);  // Analog Comparator 2
pub const ADC_EMUX_EM3_EXTERNAL = usize(0x00004000);  // External (GPIO Pins)
pub const ADC_EMUX_EM3_TIMER = usize(0x00005000);  // Timer
pub const ADC_EMUX_EM3_PWM0 = usize(0x00006000);  // PWM generator 0
pub const ADC_EMUX_EM3_PWM1 = usize(0x00007000);  // PWM generator 1
pub const ADC_EMUX_EM3_PWM2 = usize(0x00008000);  // PWM generator 2
pub const ADC_EMUX_EM3_PWM3 = usize(0x00009000);  // PWM generator 3
pub const ADC_EMUX_EM3_NEVER = usize(0x0000E000);  // Never Trigger
pub const ADC_EMUX_EM3_ALWAYS = usize(0x0000F000);  // Always (continuously sample)
pub const ADC_EMUX_EM2_M = usize(0x00000F00);  // SS2 Trigger Select
pub const ADC_EMUX_EM2_PROCESSOR = usize(0x00000000);  // Processor (default)
pub const ADC_EMUX_EM2_COMP0 = usize(0x00000100);  // Analog Comparator 0
pub const ADC_EMUX_EM2_COMP1 = usize(0x00000200);  // Analog Comparator 1
pub const ADC_EMUX_EM2_COMP2 = usize(0x00000300);  // Analog Comparator 2
pub const ADC_EMUX_EM2_EXTERNAL = usize(0x00000400);  // External (GPIO Pins)
pub const ADC_EMUX_EM2_TIMER = usize(0x00000500);  // Timer
pub const ADC_EMUX_EM2_PWM0 = usize(0x00000600);  // PWM generator 0
pub const ADC_EMUX_EM2_PWM1 = usize(0x00000700);  // PWM generator 1
pub const ADC_EMUX_EM2_PWM2 = usize(0x00000800);  // PWM generator 2
pub const ADC_EMUX_EM2_PWM3 = usize(0x00000900);  // PWM generator 3
pub const ADC_EMUX_EM2_NEVER = usize(0x00000E00);  // Never Trigger
pub const ADC_EMUX_EM2_ALWAYS = usize(0x00000F00);  // Always (continuously sample)
pub const ADC_EMUX_EM1_M = usize(0x000000F0);  // SS1 Trigger Select
pub const ADC_EMUX_EM1_PROCESSOR = usize(0x00000000);  // Processor (default)
pub const ADC_EMUX_EM1_COMP0 = usize(0x00000010);  // Analog Comparator 0
pub const ADC_EMUX_EM1_COMP1 = usize(0x00000020);  // Analog Comparator 1
pub const ADC_EMUX_EM1_COMP2 = usize(0x00000030);  // Analog Comparator 2
pub const ADC_EMUX_EM1_EXTERNAL = usize(0x00000040);  // External (GPIO Pins)
pub const ADC_EMUX_EM1_TIMER = usize(0x00000050);  // Timer
pub const ADC_EMUX_EM1_PWM0 = usize(0x00000060);  // PWM generator 0
pub const ADC_EMUX_EM1_PWM1 = usize(0x00000070);  // PWM generator 1
pub const ADC_EMUX_EM1_PWM2 = usize(0x00000080);  // PWM generator 2
pub const ADC_EMUX_EM1_PWM3 = usize(0x00000090);  // PWM generator 3
pub const ADC_EMUX_EM1_NEVER = usize(0x000000E0);  // Never Trigger
pub const ADC_EMUX_EM1_ALWAYS = usize(0x000000F0);  // Always (continuously sample)
pub const ADC_EMUX_EM0_M = usize(0x0000000F);  // SS0 Trigger Select
pub const ADC_EMUX_EM0_PROCESSOR = usize(0x00000000);  // Processor (default)
pub const ADC_EMUX_EM0_COMP0 = usize(0x00000001);  // Analog Comparator 0
pub const ADC_EMUX_EM0_COMP1 = usize(0x00000002);  // Analog Comparator 1
pub const ADC_EMUX_EM0_COMP2 = usize(0x00000003);  // Analog Comparator 2
pub const ADC_EMUX_EM0_EXTERNAL = usize(0x00000004);  // External (GPIO Pins)
pub const ADC_EMUX_EM0_TIMER = usize(0x00000005);  // Timer
pub const ADC_EMUX_EM0_PWM0 = usize(0x00000006);  // PWM generator 0
pub const ADC_EMUX_EM0_PWM1 = usize(0x00000007);  // PWM generator 1
pub const ADC_EMUX_EM0_PWM2 = usize(0x00000008);  // PWM generator 2
pub const ADC_EMUX_EM0_PWM3 = usize(0x00000009);  // PWM generator 3
pub const ADC_EMUX_EM0_NEVER = usize(0x0000000E);  // Never Trigger
pub const ADC_EMUX_EM0_ALWAYS = usize(0x0000000F);  // Always (continuously sample)

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_USTAT register.
//
//*****************************************************************************
pub const ADC_USTAT_UV3 = usize(0x00000008);  // SS3 FIFO Underflow
pub const ADC_USTAT_UV2 = usize(0x00000004);  // SS2 FIFO Underflow
pub const ADC_USTAT_UV1 = usize(0x00000002);  // SS1 FIFO Underflow
pub const ADC_USTAT_UV0 = usize(0x00000001);  // SS0 FIFO Underflow

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_TSSEL register.
//
//*****************************************************************************
pub const ADC_TSSEL_PS3_M = usize(0x30000000);  // Generator 3 PWM Module Trigger
                                            // Select
pub const ADC_TSSEL_PS3_0 = usize(0x00000000);  // Use Generator 3 (and its
                                            // trigger) in PWM module 0
pub const ADC_TSSEL_PS2_M = usize(0x00300000);  // Generator 2 PWM Module Trigger
                                            // Select
pub const ADC_TSSEL_PS2_0 = usize(0x00000000);  // Use Generator 2 (and its
                                            // trigger) in PWM module 0
pub const ADC_TSSEL_PS1_M = usize(0x00003000);  // Generator 1 PWM Module Trigger
                                            // Select
pub const ADC_TSSEL_PS1_0 = usize(0x00000000);  // Use Generator 1 (and its
                                            // trigger) in PWM module 0
pub const ADC_TSSEL_PS0_M = usize(0x00000030);  // Generator 0 PWM Module Trigger
                                            // Select
pub const ADC_TSSEL_PS0_0 = usize(0x00000000);  // Use Generator 0 (and its
                                            // trigger) in PWM module 0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSPRI register.
//
//*****************************************************************************
pub const ADC_SSPRI_SS3_M = usize(0x00003000);  // SS3 Priority
pub const ADC_SSPRI_SS2_M = usize(0x00000300);  // SS2 Priority
pub const ADC_SSPRI_SS1_M = usize(0x00000030);  // SS1 Priority
pub const ADC_SSPRI_SS0_M = usize(0x00000003);  // SS0 Priority

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SPC register.
//
//*****************************************************************************
pub const ADC_SPC_PHASE_M = usize(0x0000000F);  // Phase Difference
pub const ADC_SPC_PHASE_0 = usize(0x00000000);  // ADC sample lags by 0.0
pub const ADC_SPC_PHASE_22_5 = usize(0x00000001);  // ADC sample lags by 22.5
pub const ADC_SPC_PHASE_45 = usize(0x00000002);  // ADC sample lags by 45.0
pub const ADC_SPC_PHASE_67_5 = usize(0x00000003);  // ADC sample lags by 67.5
pub const ADC_SPC_PHASE_90 = usize(0x00000004);  // ADC sample lags by 90.0
pub const ADC_SPC_PHASE_112_5 = usize(0x00000005);  // ADC sample lags by 112.5
pub const ADC_SPC_PHASE_135 = usize(0x00000006);  // ADC sample lags by 135.0
pub const ADC_SPC_PHASE_157_5 = usize(0x00000007);  // ADC sample lags by 157.5
pub const ADC_SPC_PHASE_180 = usize(0x00000008);  // ADC sample lags by 180.0
pub const ADC_SPC_PHASE_202_5 = usize(0x00000009);  // ADC sample lags by 202.5
pub const ADC_SPC_PHASE_225 = usize(0x0000000A);  // ADC sample lags by 225.0
pub const ADC_SPC_PHASE_247_5 = usize(0x0000000B);  // ADC sample lags by 247.5
pub const ADC_SPC_PHASE_270 = usize(0x0000000C);  // ADC sample lags by 270.0
pub const ADC_SPC_PHASE_292_5 = usize(0x0000000D);  // ADC sample lags by 292.5
pub const ADC_SPC_PHASE_315 = usize(0x0000000E);  // ADC sample lags by 315.0
pub const ADC_SPC_PHASE_337_5 = usize(0x0000000F);  // ADC sample lags by 337.5

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PSSI register.
//
//*****************************************************************************
pub const ADC_PSSI_GSYNC = usize(0x80000000);  // Global Synchronize
pub const ADC_PSSI_SYNCWAIT = usize(0x08000000);  // Synchronize Wait
pub const ADC_PSSI_SS3 = usize(0x00000008);  // SS3 Initiate
pub const ADC_PSSI_SS2 = usize(0x00000004);  // SS2 Initiate
pub const ADC_PSSI_SS1 = usize(0x00000002);  // SS1 Initiate
pub const ADC_PSSI_SS0 = usize(0x00000001);  // SS0 Initiate

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SAC register.
//
//*****************************************************************************
pub const ADC_SAC_AVG_M = usize(0x00000007);  // Hardware Averaging Control
pub const ADC_SAC_AVG_OFF = usize(0x00000000);  // No hardware oversampling
pub const ADC_SAC_AVG_2X = usize(0x00000001);  // 2x hardware oversampling
pub const ADC_SAC_AVG_4X = usize(0x00000002);  // 4x hardware oversampling
pub const ADC_SAC_AVG_8X = usize(0x00000003);  // 8x hardware oversampling
pub const ADC_SAC_AVG_16X = usize(0x00000004);  // 16x hardware oversampling
pub const ADC_SAC_AVG_32X = usize(0x00000005);  // 32x hardware oversampling
pub const ADC_SAC_AVG_64X = usize(0x00000006);  // 64x hardware oversampling

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCISC register.
//
//*****************************************************************************
pub const ADC_DCISC_DCINT7 = usize(0x00000080);  // Digital Comparator 7 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT6 = usize(0x00000040);  // Digital Comparator 6 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT5 = usize(0x00000020);  // Digital Comparator 5 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT4 = usize(0x00000010);  // Digital Comparator 4 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT3 = usize(0x00000008);  // Digital Comparator 3 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT2 = usize(0x00000004);  // Digital Comparator 2 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT1 = usize(0x00000002);  // Digital Comparator 1 Interrupt
                                            // Status and Clear
pub const ADC_DCISC_DCINT0 = usize(0x00000001);  // Digital Comparator 0 Interrupt
                                            // Status and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_CTL register.
//
//*****************************************************************************
pub const ADC_CTL_VREF_M = usize(0x00000001);  // Voltage Reference Select
pub const ADC_CTL_VREF_INTERNAL = usize(0x00000000);  // VDDA and GNDA are the voltage
                                            // references
pub const ADC_CTL_VREF_EXT_3V = usize(0x00000001);  // The external VREFA+ and VREFA-
                                            // inputs are the voltage
                                            // references

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX0 register.
//
//*****************************************************************************
pub const ADC_SSMUX0_MUX7_M = usize(0xF0000000);  // 8th Sample Input Select
pub const ADC_SSMUX0_MUX6_M = usize(0x0F000000);  // 7th Sample Input Select
pub const ADC_SSMUX0_MUX5_M = usize(0x00F00000);  // 6th Sample Input Select
pub const ADC_SSMUX0_MUX4_M = usize(0x000F0000);  // 5th Sample Input Select
pub const ADC_SSMUX0_MUX3_M = usize(0x0000F000);  // 4th Sample Input Select
pub const ADC_SSMUX0_MUX2_M = usize(0x00000F00);  // 3rd Sample Input Select
pub const ADC_SSMUX0_MUX1_M = usize(0x000000F0);  // 2nd Sample Input Select
pub const ADC_SSMUX0_MUX0_M = usize(0x0000000F);  // 1st Sample Input Select
pub const ADC_SSMUX0_MUX7_S = usize(28);
pub const ADC_SSMUX0_MUX6_S = usize(24);
pub const ADC_SSMUX0_MUX5_S = usize(20);
pub const ADC_SSMUX0_MUX4_S = usize(16);
pub const ADC_SSMUX0_MUX3_S = usize(12);
pub const ADC_SSMUX0_MUX2_S = usize(8);
pub const ADC_SSMUX0_MUX1_S = usize(4);
pub const ADC_SSMUX0_MUX0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL0 register.
//
//*****************************************************************************
pub const ADC_SSCTL0_TS7 = usize(0x80000000);  // 8th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE7 = usize(0x40000000);  // 8th Sample Interrupt Enable
pub const ADC_SSCTL0_END7 = usize(0x20000000);  // 8th Sample is End of Sequence
pub const ADC_SSCTL0_D7 = usize(0x10000000);  // 8th Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS6 = usize(0x08000000);  // 7th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE6 = usize(0x04000000);  // 7th Sample Interrupt Enable
pub const ADC_SSCTL0_END6 = usize(0x02000000);  // 7th Sample is End of Sequence
pub const ADC_SSCTL0_D6 = usize(0x01000000);  // 7th Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS5 = usize(0x00800000);  // 6th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE5 = usize(0x00400000);  // 6th Sample Interrupt Enable
pub const ADC_SSCTL0_END5 = usize(0x00200000);  // 6th Sample is End of Sequence
pub const ADC_SSCTL0_D5 = usize(0x00100000);  // 6th Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS4 = usize(0x00080000);  // 5th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE4 = usize(0x00040000);  // 5th Sample Interrupt Enable
pub const ADC_SSCTL0_END4 = usize(0x00020000);  // 5th Sample is End of Sequence
pub const ADC_SSCTL0_D4 = usize(0x00010000);  // 5th Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS3 = usize(0x00008000);  // 4th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE3 = usize(0x00004000);  // 4th Sample Interrupt Enable
pub const ADC_SSCTL0_END3 = usize(0x00002000);  // 4th Sample is End of Sequence
pub const ADC_SSCTL0_D3 = usize(0x00001000);  // 4th Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS2 = usize(0x00000800);  // 3rd Sample Temp Sensor Select
pub const ADC_SSCTL0_IE2 = usize(0x00000400);  // 3rd Sample Interrupt Enable
pub const ADC_SSCTL0_END2 = usize(0x00000200);  // 3rd Sample is End of Sequence
pub const ADC_SSCTL0_D2 = usize(0x00000100);  // 3rd Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS1 = usize(0x00000080);  // 2nd Sample Temp Sensor Select
pub const ADC_SSCTL0_IE1 = usize(0x00000040);  // 2nd Sample Interrupt Enable
pub const ADC_SSCTL0_END1 = usize(0x00000020);  // 2nd Sample is End of Sequence
pub const ADC_SSCTL0_D1 = usize(0x00000010);  // 2nd Sample Differential Input
                                            // Select
pub const ADC_SSCTL0_TS0 = usize(0x00000008);  // 1st Sample Temp Sensor Select
pub const ADC_SSCTL0_IE0 = usize(0x00000004);  // 1st Sample Interrupt Enable
pub const ADC_SSCTL0_END0 = usize(0x00000002);  // 1st Sample is End of Sequence
pub const ADC_SSCTL0_D0 = usize(0x00000001);  // 1st Sample Differential Input
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO0 register.
//
//*****************************************************************************
pub const ADC_SSFIFO0_DATA_M = usize(0x00000FFF);  // Conversion Result Data
pub const ADC_SSFIFO0_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT0 register.
//
//*****************************************************************************
pub const ADC_SSFSTAT0_FULL = usize(0x00001000);  // FIFO Full
pub const ADC_SSFSTAT0_EMPTY = usize(0x00000100);  // FIFO Empty
pub const ADC_SSFSTAT0_HPTR_M = usize(0x000000F0);  // FIFO Head Pointer
pub const ADC_SSFSTAT0_TPTR_M = usize(0x0000000F);  // FIFO Tail Pointer
pub const ADC_SSFSTAT0_HPTR_S = usize(4);
pub const ADC_SSFSTAT0_TPTR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP0 register.
//
//*****************************************************************************
pub const ADC_SSOP0_S7DCOP = usize(0x10000000);  // Sample 7 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S6DCOP = usize(0x01000000);  // Sample 6 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S5DCOP = usize(0x00100000);  // Sample 5 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S4DCOP = usize(0x00010000);  // Sample 4 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S3DCOP = usize(0x00001000);  // Sample 3 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S2DCOP = usize(0x00000100);  // Sample 2 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S1DCOP = usize(0x00000010);  // Sample 1 Digital Comparator
                                            // Operation
pub const ADC_SSOP0_S0DCOP = usize(0x00000001);  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC0 register.
//
//*****************************************************************************
pub const ADC_SSDC0_S7DCSEL_M = usize(0xF0000000);  // Sample 7 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S6DCSEL_M = usize(0x0F000000);  // Sample 6 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S5DCSEL_M = usize(0x00F00000);  // Sample 5 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S4DCSEL_M = usize(0x000F0000);  // Sample 4 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S3DCSEL_M = usize(0x0000F000);  // Sample 3 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S2DCSEL_M = usize(0x00000F00);  // Sample 2 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S1DCSEL_M = usize(0x000000F0);  // Sample 1 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S0DCSEL_M = usize(0x0000000F);  // Sample 0 Digital Comparator
                                            // Select
pub const ADC_SSDC0_S6DCSEL_S = usize(24);
pub const ADC_SSDC0_S5DCSEL_S = usize(20);
pub const ADC_SSDC0_S4DCSEL_S = usize(16);
pub const ADC_SSDC0_S3DCSEL_S = usize(12);
pub const ADC_SSDC0_S2DCSEL_S = usize(8);
pub const ADC_SSDC0_S1DCSEL_S = usize(4);
pub const ADC_SSDC0_S0DCSEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX0 register.
//
//*****************************************************************************
pub const ADC_SSEMUX0_EMUX7 = usize(0x10000000);  // 8th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX6 = usize(0x01000000);  // 7th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX5 = usize(0x00100000);  // 6th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX4 = usize(0x00010000);  // 5th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX3 = usize(0x00001000);  // 4th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX2 = usize(0x00000100);  // 3rd Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX1 = usize(0x00000010);  // 2th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX0_EMUX0 = usize(0x00000001);  // 1st Sample Input Select (Upper
                                            // Bit)

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH0 register.
//
//*****************************************************************************
pub const ADC_SSTSH0_TSH7_M = usize(0xF0000000);  // 8th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH6_M = usize(0x0F000000);  // 7th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH5_M = usize(0x00F00000);  // 6th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH4_M = usize(0x000F0000);  // 5th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH3_M = usize(0x0000F000);  // 4th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH2_M = usize(0x00000F00);  // 3rd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH1_M = usize(0x000000F0);  // 2nd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH0_M = usize(0x0000000F);  // 1st Sample and Hold Period
                                            // Select
pub const ADC_SSTSH0_TSH7_S = usize(28);
pub const ADC_SSTSH0_TSH6_S = usize(24);
pub const ADC_SSTSH0_TSH5_S = usize(20);
pub const ADC_SSTSH0_TSH4_S = usize(16);
pub const ADC_SSTSH0_TSH3_S = usize(12);
pub const ADC_SSTSH0_TSH2_S = usize(8);
pub const ADC_SSTSH0_TSH1_S = usize(4);
pub const ADC_SSTSH0_TSH0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX1 register.
//
//*****************************************************************************
pub const ADC_SSMUX1_MUX3_M = usize(0x0000F000);  // 4th Sample Input Select
pub const ADC_SSMUX1_MUX2_M = usize(0x00000F00);  // 3rd Sample Input Select
pub const ADC_SSMUX1_MUX1_M = usize(0x000000F0);  // 2nd Sample Input Select
pub const ADC_SSMUX1_MUX0_M = usize(0x0000000F);  // 1st Sample Input Select
pub const ADC_SSMUX1_MUX3_S = usize(12);
pub const ADC_SSMUX1_MUX2_S = usize(8);
pub const ADC_SSMUX1_MUX1_S = usize(4);
pub const ADC_SSMUX1_MUX0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL1 register.
//
//*****************************************************************************
pub const ADC_SSCTL1_TS3 = usize(0x00008000);  // 4th Sample Temp Sensor Select
pub const ADC_SSCTL1_IE3 = usize(0x00004000);  // 4th Sample Interrupt Enable
pub const ADC_SSCTL1_END3 = usize(0x00002000);  // 4th Sample is End of Sequence
pub const ADC_SSCTL1_D3 = usize(0x00001000);  // 4th Sample Differential Input
                                            // Select
pub const ADC_SSCTL1_TS2 = usize(0x00000800);  // 3rd Sample Temp Sensor Select
pub const ADC_SSCTL1_IE2 = usize(0x00000400);  // 3rd Sample Interrupt Enable
pub const ADC_SSCTL1_END2 = usize(0x00000200);  // 3rd Sample is End of Sequence
pub const ADC_SSCTL1_D2 = usize(0x00000100);  // 3rd Sample Differential Input
                                            // Select
pub const ADC_SSCTL1_TS1 = usize(0x00000080);  // 2nd Sample Temp Sensor Select
pub const ADC_SSCTL1_IE1 = usize(0x00000040);  // 2nd Sample Interrupt Enable
pub const ADC_SSCTL1_END1 = usize(0x00000020);  // 2nd Sample is End of Sequence
pub const ADC_SSCTL1_D1 = usize(0x00000010);  // 2nd Sample Differential Input
                                            // Select
pub const ADC_SSCTL1_TS0 = usize(0x00000008);  // 1st Sample Temp Sensor Select
pub const ADC_SSCTL1_IE0 = usize(0x00000004);  // 1st Sample Interrupt Enable
pub const ADC_SSCTL1_END0 = usize(0x00000002);  // 1st Sample is End of Sequence
pub const ADC_SSCTL1_D0 = usize(0x00000001);  // 1st Sample Differential Input
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO1 register.
//
//*****************************************************************************
pub const ADC_SSFIFO1_DATA_M = usize(0x00000FFF);  // Conversion Result Data
pub const ADC_SSFIFO1_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT1 register.
//
//*****************************************************************************
pub const ADC_SSFSTAT1_FULL = usize(0x00001000);  // FIFO Full
pub const ADC_SSFSTAT1_EMPTY = usize(0x00000100);  // FIFO Empty
pub const ADC_SSFSTAT1_HPTR_M = usize(0x000000F0);  // FIFO Head Pointer
pub const ADC_SSFSTAT1_TPTR_M = usize(0x0000000F);  // FIFO Tail Pointer
pub const ADC_SSFSTAT1_HPTR_S = usize(4);
pub const ADC_SSFSTAT1_TPTR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP1 register.
//
//*****************************************************************************
pub const ADC_SSOP1_S3DCOP = usize(0x00001000);  // Sample 3 Digital Comparator
                                            // Operation
pub const ADC_SSOP1_S2DCOP = usize(0x00000100);  // Sample 2 Digital Comparator
                                            // Operation
pub const ADC_SSOP1_S1DCOP = usize(0x00000010);  // Sample 1 Digital Comparator
                                            // Operation
pub const ADC_SSOP1_S0DCOP = usize(0x00000001);  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC1 register.
//
//*****************************************************************************
pub const ADC_SSDC1_S3DCSEL_M = usize(0x0000F000);  // Sample 3 Digital Comparator
                                            // Select
pub const ADC_SSDC1_S2DCSEL_M = usize(0x00000F00);  // Sample 2 Digital Comparator
                                            // Select
pub const ADC_SSDC1_S1DCSEL_M = usize(0x000000F0);  // Sample 1 Digital Comparator
                                            // Select
pub const ADC_SSDC1_S0DCSEL_M = usize(0x0000000F);  // Sample 0 Digital Comparator
                                            // Select
pub const ADC_SSDC1_S2DCSEL_S = usize(8);
pub const ADC_SSDC1_S1DCSEL_S = usize(4);
pub const ADC_SSDC1_S0DCSEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX1 register.
//
//*****************************************************************************
pub const ADC_SSEMUX1_EMUX3 = usize(0x00001000);  // 4th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX1_EMUX2 = usize(0x00000100);  // 3rd Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX1_EMUX1 = usize(0x00000010);  // 2th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX1_EMUX0 = usize(0x00000001);  // 1st Sample Input Select (Upper
                                            // Bit)

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH1 register.
//
//*****************************************************************************
pub const ADC_SSTSH1_TSH3_M = usize(0x0000F000);  // 4th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH1_TSH2_M = usize(0x00000F00);  // 3rd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH1_TSH1_M = usize(0x000000F0);  // 2nd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH1_TSH0_M = usize(0x0000000F);  // 1st Sample and Hold Period
                                            // Select
pub const ADC_SSTSH1_TSH3_S = usize(12);
pub const ADC_SSTSH1_TSH2_S = usize(8);
pub const ADC_SSTSH1_TSH1_S = usize(4);
pub const ADC_SSTSH1_TSH0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX2 register.
//
//*****************************************************************************
pub const ADC_SSMUX2_MUX3_M = usize(0x0000F000);  // 4th Sample Input Select
pub const ADC_SSMUX2_MUX2_M = usize(0x00000F00);  // 3rd Sample Input Select
pub const ADC_SSMUX2_MUX1_M = usize(0x000000F0);  // 2nd Sample Input Select
pub const ADC_SSMUX2_MUX0_M = usize(0x0000000F);  // 1st Sample Input Select
pub const ADC_SSMUX2_MUX3_S = usize(12);
pub const ADC_SSMUX2_MUX2_S = usize(8);
pub const ADC_SSMUX2_MUX1_S = usize(4);
pub const ADC_SSMUX2_MUX0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL2 register.
//
//*****************************************************************************
pub const ADC_SSCTL2_TS3 = usize(0x00008000);  // 4th Sample Temp Sensor Select
pub const ADC_SSCTL2_IE3 = usize(0x00004000);  // 4th Sample Interrupt Enable
pub const ADC_SSCTL2_END3 = usize(0x00002000);  // 4th Sample is End of Sequence
pub const ADC_SSCTL2_D3 = usize(0x00001000);  // 4th Sample Differential Input
                                            // Select
pub const ADC_SSCTL2_TS2 = usize(0x00000800);  // 3rd Sample Temp Sensor Select
pub const ADC_SSCTL2_IE2 = usize(0x00000400);  // 3rd Sample Interrupt Enable
pub const ADC_SSCTL2_END2 = usize(0x00000200);  // 3rd Sample is End of Sequence
pub const ADC_SSCTL2_D2 = usize(0x00000100);  // 3rd Sample Differential Input
                                            // Select
pub const ADC_SSCTL2_TS1 = usize(0x00000080);  // 2nd Sample Temp Sensor Select
pub const ADC_SSCTL2_IE1 = usize(0x00000040);  // 2nd Sample Interrupt Enable
pub const ADC_SSCTL2_END1 = usize(0x00000020);  // 2nd Sample is End of Sequence
pub const ADC_SSCTL2_D1 = usize(0x00000010);  // 2nd Sample Differential Input
                                            // Select
pub const ADC_SSCTL2_TS0 = usize(0x00000008);  // 1st Sample Temp Sensor Select
pub const ADC_SSCTL2_IE0 = usize(0x00000004);  // 1st Sample Interrupt Enable
pub const ADC_SSCTL2_END0 = usize(0x00000002);  // 1st Sample is End of Sequence
pub const ADC_SSCTL2_D0 = usize(0x00000001);  // 1st Sample Differential Input
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO2 register.
//
//*****************************************************************************
pub const ADC_SSFIFO2_DATA_M = usize(0x00000FFF);  // Conversion Result Data
pub const ADC_SSFIFO2_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT2 register.
//
//*****************************************************************************
pub const ADC_SSFSTAT2_FULL = usize(0x00001000);  // FIFO Full
pub const ADC_SSFSTAT2_EMPTY = usize(0x00000100);  // FIFO Empty
pub const ADC_SSFSTAT2_HPTR_M = usize(0x000000F0);  // FIFO Head Pointer
pub const ADC_SSFSTAT2_TPTR_M = usize(0x0000000F);  // FIFO Tail Pointer
pub const ADC_SSFSTAT2_HPTR_S = usize(4);
pub const ADC_SSFSTAT2_TPTR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP2 register.
//
//*****************************************************************************
pub const ADC_SSOP2_S3DCOP = usize(0x00001000);  // Sample 3 Digital Comparator
                                            // Operation
pub const ADC_SSOP2_S2DCOP = usize(0x00000100);  // Sample 2 Digital Comparator
                                            // Operation
pub const ADC_SSOP2_S1DCOP = usize(0x00000010);  // Sample 1 Digital Comparator
                                            // Operation
pub const ADC_SSOP2_S0DCOP = usize(0x00000001);  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC2 register.
//
//*****************************************************************************
pub const ADC_SSDC2_S3DCSEL_M = usize(0x0000F000);  // Sample 3 Digital Comparator
                                            // Select
pub const ADC_SSDC2_S2DCSEL_M = usize(0x00000F00);  // Sample 2 Digital Comparator
                                            // Select
pub const ADC_SSDC2_S1DCSEL_M = usize(0x000000F0);  // Sample 1 Digital Comparator
                                            // Select
pub const ADC_SSDC2_S0DCSEL_M = usize(0x0000000F);  // Sample 0 Digital Comparator
                                            // Select
pub const ADC_SSDC2_S2DCSEL_S = usize(8);
pub const ADC_SSDC2_S1DCSEL_S = usize(4);
pub const ADC_SSDC2_S0DCSEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX2 register.
//
//*****************************************************************************
pub const ADC_SSEMUX2_EMUX3 = usize(0x00001000);  // 4th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX2_EMUX2 = usize(0x00000100);  // 3rd Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX2_EMUX1 = usize(0x00000010);  // 2th Sample Input Select (Upper
                                            // Bit)
pub const ADC_SSEMUX2_EMUX0 = usize(0x00000001);  // 1st Sample Input Select (Upper
                                            // Bit)

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH2 register.
//
//*****************************************************************************
pub const ADC_SSTSH2_TSH3_M = usize(0x0000F000);  // 4th Sample and Hold Period
                                            // Select
pub const ADC_SSTSH2_TSH2_M = usize(0x00000F00);  // 3rd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH2_TSH1_M = usize(0x000000F0);  // 2nd Sample and Hold Period
                                            // Select
pub const ADC_SSTSH2_TSH0_M = usize(0x0000000F);  // 1st Sample and Hold Period
                                            // Select
pub const ADC_SSTSH2_TSH3_S = usize(12);
pub const ADC_SSTSH2_TSH2_S = usize(8);
pub const ADC_SSTSH2_TSH1_S = usize(4);
pub const ADC_SSTSH2_TSH0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX3 register.
//
//*****************************************************************************
pub const ADC_SSMUX3_MUX0_M = usize(0x0000000F);  // 1st Sample Input Select
pub const ADC_SSMUX3_MUX0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL3 register.
//
//*****************************************************************************
pub const ADC_SSCTL3_TS0 = usize(0x00000008);  // 1st Sample Temp Sensor Select
pub const ADC_SSCTL3_IE0 = usize(0x00000004);  // Sample Interrupt Enable
pub const ADC_SSCTL3_END0 = usize(0x00000002);  // End of Sequence
pub const ADC_SSCTL3_D0 = usize(0x00000001);  // Sample Differential Input Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO3 register.
//
//*****************************************************************************
pub const ADC_SSFIFO3_DATA_M = usize(0x00000FFF);  // Conversion Result Data
pub const ADC_SSFIFO3_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT3 register.
//
//*****************************************************************************
pub const ADC_SSFSTAT3_FULL = usize(0x00001000);  // FIFO Full
pub const ADC_SSFSTAT3_EMPTY = usize(0x00000100);  // FIFO Empty
pub const ADC_SSFSTAT3_HPTR_M = usize(0x000000F0);  // FIFO Head Pointer
pub const ADC_SSFSTAT3_TPTR_M = usize(0x0000000F);  // FIFO Tail Pointer
pub const ADC_SSFSTAT3_HPTR_S = usize(4);
pub const ADC_SSFSTAT3_TPTR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP3 register.
//
//*****************************************************************************
pub const ADC_SSOP3_S0DCOP = usize(0x00000001);  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC3 register.
//
//*****************************************************************************
pub const ADC_SSDC3_S0DCSEL_M = usize(0x0000000F);  // Sample 0 Digital Comparator
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSEMUX3 register.
//
//*****************************************************************************
pub const ADC_SSEMUX3_EMUX0 = usize(0x00000001);  // 1st Sample Input Select (Upper
                                            // Bit)

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSTSH3 register.
//
//*****************************************************************************
pub const ADC_SSTSH3_TSH0_M = usize(0x0000000F);  // 1st Sample and Hold Period
                                            // Select
pub const ADC_SSTSH3_TSH0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCRIC register.
//
//*****************************************************************************
pub const ADC_DCRIC_DCTRIG7 = usize(0x00800000);  // Digital Comparator Trigger 7
pub const ADC_DCRIC_DCTRIG6 = usize(0x00400000);  // Digital Comparator Trigger 6
pub const ADC_DCRIC_DCTRIG5 = usize(0x00200000);  // Digital Comparator Trigger 5
pub const ADC_DCRIC_DCTRIG4 = usize(0x00100000);  // Digital Comparator Trigger 4
pub const ADC_DCRIC_DCTRIG3 = usize(0x00080000);  // Digital Comparator Trigger 3
pub const ADC_DCRIC_DCTRIG2 = usize(0x00040000);  // Digital Comparator Trigger 2
pub const ADC_DCRIC_DCTRIG1 = usize(0x00020000);  // Digital Comparator Trigger 1
pub const ADC_DCRIC_DCTRIG0 = usize(0x00010000);  // Digital Comparator Trigger 0
pub const ADC_DCRIC_DCINT7 = usize(0x00000080);  // Digital Comparator Interrupt 7
pub const ADC_DCRIC_DCINT6 = usize(0x00000040);  // Digital Comparator Interrupt 6
pub const ADC_DCRIC_DCINT5 = usize(0x00000020);  // Digital Comparator Interrupt 5
pub const ADC_DCRIC_DCINT4 = usize(0x00000010);  // Digital Comparator Interrupt 4
pub const ADC_DCRIC_DCINT3 = usize(0x00000008);  // Digital Comparator Interrupt 3
pub const ADC_DCRIC_DCINT2 = usize(0x00000004);  // Digital Comparator Interrupt 2
pub const ADC_DCRIC_DCINT1 = usize(0x00000002);  // Digital Comparator Interrupt 1
pub const ADC_DCRIC_DCINT0 = usize(0x00000001);  // Digital Comparator Interrupt 0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL0 register.
//
//*****************************************************************************
pub const ADC_DCCTL0_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL0_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL0_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL0_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL0_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL0_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL0_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL0_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL0_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL0_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL0_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL0_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL0_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL0_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL0_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL0_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL0_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL0_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL0_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL0_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL1 register.
//
//*****************************************************************************
pub const ADC_DCCTL1_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL1_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL1_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL1_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL1_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL1_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL1_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL1_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL1_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL1_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL1_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL1_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL1_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL1_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL1_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL1_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL1_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL1_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL1_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL1_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL2 register.
//
//*****************************************************************************
pub const ADC_DCCTL2_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL2_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL2_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL2_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL2_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL2_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL2_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL2_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL2_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL2_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL2_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL2_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL2_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL2_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL2_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL2_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL2_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL2_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL2_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL2_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL3 register.
//
//*****************************************************************************
pub const ADC_DCCTL3_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL3_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL3_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL3_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL3_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL3_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL3_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL3_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL3_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL3_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL3_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL3_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL3_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL3_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL3_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL3_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL3_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL3_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL3_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL3_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL4 register.
//
//*****************************************************************************
pub const ADC_DCCTL4_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL4_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL4_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL4_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL4_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL4_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL4_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL4_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL4_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL4_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL4_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL4_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL4_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL4_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL4_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL4_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL4_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL4_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL4_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL4_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL5 register.
//
//*****************************************************************************
pub const ADC_DCCTL5_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL5_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL5_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL5_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL5_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL5_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL5_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL5_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL5_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL5_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL5_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL5_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL5_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL5_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL5_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL5_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL5_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL5_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL5_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL5_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL6 register.
//
//*****************************************************************************
pub const ADC_DCCTL6_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL6_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL6_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL6_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL6_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL6_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL6_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL6_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL6_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL6_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL6_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL6_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL6_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL6_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL6_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL6_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL6_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL6_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL6_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL6_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL7 register.
//
//*****************************************************************************
pub const ADC_DCCTL7_CTE = usize(0x00001000);  // Comparison Trigger Enable
pub const ADC_DCCTL7_CTC_M = usize(0x00000C00);  // Comparison Trigger Condition
pub const ADC_DCCTL7_CTC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL7_CTC_MID = usize(0x00000400);  // Mid Band
pub const ADC_DCCTL7_CTC_HIGH = usize(0x00000C00);  // High Band
pub const ADC_DCCTL7_CTM_M = usize(0x00000300);  // Comparison Trigger Mode
pub const ADC_DCCTL7_CTM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL7_CTM_ONCE = usize(0x00000100);  // Once
pub const ADC_DCCTL7_CTM_HALWAYS = usize(0x00000200);  // Hysteresis Always
pub const ADC_DCCTL7_CTM_HONCE = usize(0x00000300);  // Hysteresis Once
pub const ADC_DCCTL7_CIE = usize(0x00000010);  // Comparison Interrupt Enable
pub const ADC_DCCTL7_CIC_M = usize(0x0000000C);  // Comparison Interrupt Condition
pub const ADC_DCCTL7_CIC_LOW = usize(0x00000000);  // Low Band
pub const ADC_DCCTL7_CIC_MID = usize(0x00000004);  // Mid Band
pub const ADC_DCCTL7_CIC_HIGH = usize(0x0000000C);  // High Band
pub const ADC_DCCTL7_CIM_M = usize(0x00000003);  // Comparison Interrupt Mode
pub const ADC_DCCTL7_CIM_ALWAYS = usize(0x00000000);  // Always
pub const ADC_DCCTL7_CIM_ONCE = usize(0x00000001);  // Once
pub const ADC_DCCTL7_CIM_HALWAYS = usize(0x00000002);  // Hysteresis Always
pub const ADC_DCCTL7_CIM_HONCE = usize(0x00000003);  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP0 register.
//
//*****************************************************************************
pub const ADC_DCCMP0_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP0_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP0_COMP1_S = usize(16);
pub const ADC_DCCMP0_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP1 register.
//
//*****************************************************************************
pub const ADC_DCCMP1_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP1_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP1_COMP1_S = usize(16);
pub const ADC_DCCMP1_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP2 register.
//
//*****************************************************************************
pub const ADC_DCCMP2_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP2_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP2_COMP1_S = usize(16);
pub const ADC_DCCMP2_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP3 register.
//
//*****************************************************************************
pub const ADC_DCCMP3_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP3_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP3_COMP1_S = usize(16);
pub const ADC_DCCMP3_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP4 register.
//
//*****************************************************************************
pub const ADC_DCCMP4_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP4_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP4_COMP1_S = usize(16);
pub const ADC_DCCMP4_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP5 register.
//
//*****************************************************************************
pub const ADC_DCCMP5_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP5_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP5_COMP1_S = usize(16);
pub const ADC_DCCMP5_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP6 register.
//
//*****************************************************************************
pub const ADC_DCCMP6_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP6_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP6_COMP1_S = usize(16);
pub const ADC_DCCMP6_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP7 register.
//
//*****************************************************************************
pub const ADC_DCCMP7_COMP1_M = usize(0x0FFF0000);  // Compare 1
pub const ADC_DCCMP7_COMP0_M = usize(0x00000FFF);  // Compare 0
pub const ADC_DCCMP7_COMP1_S = usize(16);
pub const ADC_DCCMP7_COMP0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PP register.
//
//*****************************************************************************
pub const ADC_PP_APSHT = usize(0x01000000);  // Application-Programmable
                                            // Sample-and-Hold Time
pub const ADC_PP_TS = usize(0x00800000);  // Temperature Sensor
pub const ADC_PP_RSL_M = usize(0x007C0000);  // Resolution
pub const ADC_PP_TYPE_M = usize(0x00030000);  // ADC Architecture
pub const ADC_PP_TYPE_SAR = usize(0x00000000);  // SAR
pub const ADC_PP_DC_M = usize(0x0000FC00);  // Digital Comparator Count
pub const ADC_PP_CH_M = usize(0x000003F0);  // ADC Channel Count
pub const ADC_PP_MCR_M = usize(0x0000000F);  // Maximum Conversion Rate
pub const ADC_PP_MCR_FULL = usize(0x00000007);  // Full conversion rate (FCONV) as
                                            // defined by TADC and NSH
pub const ADC_PP_RSL_S = usize(18);
pub const ADC_PP_DC_S = usize(10);
pub const ADC_PP_CH_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PC register.
//
//*****************************************************************************
pub const ADC_PC_MCR_M = usize(0x0000000F);  // Conversion Rate
pub const ADC_PC_MCR_1_8 = usize(0x00000001);  // Eighth conversion rate. After a
                                            // conversion completes, the logic
                                            // pauses for 112 TADC periods
                                            // before starting the next
                                            // conversion
pub const ADC_PC_MCR_1_4 = usize(0x00000003);  // Quarter conversion rate. After a
                                            // conversion completes, the logic
                                            // pauses for 48 TADC periods
                                            // before starting the next
                                            // conversion
pub const ADC_PC_MCR_1_2 = usize(0x00000005);  // Half conversion rate. After a
                                            // conversion completes, the logic
                                            // pauses for 16 TADC periods
                                            // before starting the next
                                            // conversion
pub const ADC_PC_MCR_FULL = usize(0x00000007);  // Full conversion rate (FCONV) as
                                            // defined by TADC and NSH

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_CC register.
//
//*****************************************************************************
pub const ADC_CC_CLKDIV_M = usize(0x000003F0);  // PLL VCO Clock Divisor
pub const ADC_CC_CS_M = usize(0x0000000F);  // ADC Clock Source
pub const ADC_CC_CS_SYSPLL = usize(0x00000000);  // PLL VCO divided by CLKDIV
pub const ADC_CC_CS_PIOSC = usize(0x00000001);  // PIOSC
pub const ADC_CC_CS_MOSC = usize(0x00000002);  // MOSC
pub const ADC_CC_CLKDIV_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACMIS register.
//
//*****************************************************************************
pub const COMP_ACMIS_IN2 = usize(0x00000004);  // Comparator 2 Masked Interrupt
                                            // Status
pub const COMP_ACMIS_IN1 = usize(0x00000002);  // Comparator 1 Masked Interrupt
                                            // Status
pub const COMP_ACMIS_IN0 = usize(0x00000001);  // Comparator 0 Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACRIS register.
//
//*****************************************************************************
pub const COMP_ACRIS_IN2 = usize(0x00000004);  // Comparator 2 Interrupt Status
pub const COMP_ACRIS_IN1 = usize(0x00000002);  // Comparator 1 Interrupt Status
pub const COMP_ACRIS_IN0 = usize(0x00000001);  // Comparator 0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACINTEN register.
//
//*****************************************************************************
pub const COMP_ACINTEN_IN2 = usize(0x00000004);  // Comparator 2 Interrupt Enable
pub const COMP_ACINTEN_IN1 = usize(0x00000002);  // Comparator 1 Interrupt Enable
pub const COMP_ACINTEN_IN0 = usize(0x00000001);  // Comparator 0 Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACREFCTL
// register.
//
//*****************************************************************************
pub const COMP_ACREFCTL_EN = usize(0x00000200);  // Resistor Ladder Enable
pub const COMP_ACREFCTL_RNG = usize(0x00000100);  // Resistor Ladder Range
pub const COMP_ACREFCTL_VREF_M = usize(0x0000000F);  // Resistor Ladder Voltage Ref
pub const COMP_ACREFCTL_VREF_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT0 register.
//
//*****************************************************************************
pub const COMP_ACSTAT0_OVAL = usize(0x00000002);  // Comparator Output Value

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL0 register.
//
//*****************************************************************************
pub const COMP_ACCTL0_TOEN = usize(0x00000800);  // Trigger Output Enable
pub const COMP_ACCTL0_ASRCP_M = usize(0x00000600);  // Analog Source Positive
pub const COMP_ACCTL0_ASRCP_PIN = usize(0x00000000);  // Pin value of Cn+
pub const COMP_ACCTL0_ASRCP_PIN0 = usize(0x00000200);  // Pin value of C0+
pub const COMP_ACCTL0_ASRCP_REF = usize(0x00000400);  // Internal voltage reference
pub const COMP_ACCTL0_TSLVAL = usize(0x00000080);  // Trigger Sense Level Value
pub const COMP_ACCTL0_TSEN_M = usize(0x00000060);  // Trigger Sense
pub const COMP_ACCTL0_TSEN_LEVEL = usize(0x00000000);  // Level sense, see TSLVAL
pub const COMP_ACCTL0_TSEN_FALL = usize(0x00000020);  // Falling edge
pub const COMP_ACCTL0_TSEN_RISE = usize(0x00000040);  // Rising edge
pub const COMP_ACCTL0_TSEN_BOTH = usize(0x00000060);  // Either edge
pub const COMP_ACCTL0_ISLVAL = usize(0x00000010);  // Interrupt Sense Level Value
pub const COMP_ACCTL0_ISEN_M = usize(0x0000000C);  // Interrupt Sense
pub const COMP_ACCTL0_ISEN_LEVEL = usize(0x00000000);  // Level sense, see ISLVAL
pub const COMP_ACCTL0_ISEN_FALL = usize(0x00000004);  // Falling edge
pub const COMP_ACCTL0_ISEN_RISE = usize(0x00000008);  // Rising edge
pub const COMP_ACCTL0_ISEN_BOTH = usize(0x0000000C);  // Either edge
pub const COMP_ACCTL0_CINV = usize(0x00000002);  // Comparator Output Invert

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT1 register.
//
//*****************************************************************************
pub const COMP_ACSTAT1_OVAL = usize(0x00000002);  // Comparator Output Value

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL1 register.
//
//*****************************************************************************
pub const COMP_ACCTL1_TOEN = usize(0x00000800);  // Trigger Output Enable
pub const COMP_ACCTL1_ASRCP_M = usize(0x00000600);  // Analog Source Positive
pub const COMP_ACCTL1_ASRCP_PIN = usize(0x00000000);  // Pin value of Cn+
pub const COMP_ACCTL1_ASRCP_PIN0 = usize(0x00000200);  // Pin value of C0+
pub const COMP_ACCTL1_ASRCP_REF = usize(0x00000400);  // Internal voltage reference
pub const COMP_ACCTL1_TSLVAL = usize(0x00000080);  // Trigger Sense Level Value
pub const COMP_ACCTL1_TSEN_M = usize(0x00000060);  // Trigger Sense
pub const COMP_ACCTL1_TSEN_LEVEL = usize(0x00000000);  // Level sense, see TSLVAL
pub const COMP_ACCTL1_TSEN_FALL = usize(0x00000020);  // Falling edge
pub const COMP_ACCTL1_TSEN_RISE = usize(0x00000040);  // Rising edge
pub const COMP_ACCTL1_TSEN_BOTH = usize(0x00000060);  // Either edge
pub const COMP_ACCTL1_ISLVAL = usize(0x00000010);  // Interrupt Sense Level Value
pub const COMP_ACCTL1_ISEN_M = usize(0x0000000C);  // Interrupt Sense
pub const COMP_ACCTL1_ISEN_LEVEL = usize(0x00000000);  // Level sense, see ISLVAL
pub const COMP_ACCTL1_ISEN_FALL = usize(0x00000004);  // Falling edge
pub const COMP_ACCTL1_ISEN_RISE = usize(0x00000008);  // Rising edge
pub const COMP_ACCTL1_ISEN_BOTH = usize(0x0000000C);  // Either edge
pub const COMP_ACCTL1_CINV = usize(0x00000002);  // Comparator Output Invert

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT2 register.
//
//*****************************************************************************
pub const COMP_ACSTAT2_OVAL = usize(0x00000002);  // Comparator Output Value

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL2 register.
//
//*****************************************************************************
pub const COMP_ACCTL2_TOEN = usize(0x00000800);  // Trigger Output Enable
pub const COMP_ACCTL2_ASRCP_M = usize(0x00000600);  // Analog Source Positive
pub const COMP_ACCTL2_ASRCP_PIN = usize(0x00000000);  // Pin value of Cn+
pub const COMP_ACCTL2_ASRCP_PIN0 = usize(0x00000200);  // Pin value of C0+
pub const COMP_ACCTL2_ASRCP_REF = usize(0x00000400);  // Internal voltage reference
pub const COMP_ACCTL2_TSLVAL = usize(0x00000080);  // Trigger Sense Level Value
pub const COMP_ACCTL2_TSEN_M = usize(0x00000060);  // Trigger Sense
pub const COMP_ACCTL2_TSEN_LEVEL = usize(0x00000000);  // Level sense, see TSLVAL
pub const COMP_ACCTL2_TSEN_FALL = usize(0x00000020);  // Falling edge
pub const COMP_ACCTL2_TSEN_RISE = usize(0x00000040);  // Rising edge
pub const COMP_ACCTL2_TSEN_BOTH = usize(0x00000060);  // Either edge
pub const COMP_ACCTL2_ISLVAL = usize(0x00000010);  // Interrupt Sense Level Value
pub const COMP_ACCTL2_ISEN_M = usize(0x0000000C);  // Interrupt Sense
pub const COMP_ACCTL2_ISEN_LEVEL = usize(0x00000000);  // Level sense, see ISLVAL
pub const COMP_ACCTL2_ISEN_FALL = usize(0x00000004);  // Falling edge
pub const COMP_ACCTL2_ISEN_RISE = usize(0x00000008);  // Rising edge
pub const COMP_ACCTL2_ISEN_BOTH = usize(0x0000000C);  // Either edge
pub const COMP_ACCTL2_CINV = usize(0x00000002);  // Comparator Output Invert

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_PP register.
//
//*****************************************************************************
pub const COMP_PP_C2O = usize(0x00040000);  // Comparator Output 2 Present
pub const COMP_PP_C1O = usize(0x00020000);  // Comparator Output 1 Present
pub const COMP_PP_C0O = usize(0x00010000);  // Comparator Output 0 Present
pub const COMP_PP_CMP2 = usize(0x00000004);  // Comparator 2 Present
pub const COMP_PP_CMP1 = usize(0x00000002);  // Comparator 1 Present
pub const COMP_PP_CMP0 = usize(0x00000001);  // Comparator 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_CTL register.
//
//*****************************************************************************
pub const CAN_CTL_TEST = usize(0x00000080);  // Test Mode Enable
pub const CAN_CTL_CCE = usize(0x00000040);  // Configuration Change Enable
pub const CAN_CTL_DAR = usize(0x00000020);  // Disable Automatic-Retransmission
pub const CAN_CTL_EIE = usize(0x00000008);  // Error Interrupt Enable
pub const CAN_CTL_SIE = usize(0x00000004);  // Status Interrupt Enable
pub const CAN_CTL_IE = usize(0x00000002);  // CAN Interrupt Enable
pub const CAN_CTL_INIT = usize(0x00000001);  // Initialization

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_STS register.
//
//*****************************************************************************
pub const CAN_STS_BOFF = usize(0x00000080);  // Bus-Off Status
pub const CAN_STS_EWARN = usize(0x00000040);  // Warning Status
pub const CAN_STS_EPASS = usize(0x00000020);  // Error Passive
pub const CAN_STS_RXOK = usize(0x00000010);  // Received a Message Successfully
pub const CAN_STS_TXOK = usize(0x00000008);  // Transmitted a Message
                                            // Successfully
pub const CAN_STS_LEC_M = usize(0x00000007);  // Last Error Code
pub const CAN_STS_LEC_NONE = usize(0x00000000);  // No Error
pub const CAN_STS_LEC_STUFF = usize(0x00000001);  // Stuff Error
pub const CAN_STS_LEC_FORM = usize(0x00000002);  // Format Error
pub const CAN_STS_LEC_ACK = usize(0x00000003);  // ACK Error
pub const CAN_STS_LEC_BIT1 = usize(0x00000004);  // Bit 1 Error
pub const CAN_STS_LEC_BIT0 = usize(0x00000005);  // Bit 0 Error
pub const CAN_STS_LEC_CRC = usize(0x00000006);  // CRC Error
pub const CAN_STS_LEC_NOEVENT = usize(0x00000007);  // No Event

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_ERR register.
//
//*****************************************************************************
pub const CAN_ERR_RP = usize(0x00008000);  // Received Error Passive
pub const CAN_ERR_REC_M = usize(0x00007F00);  // Receive Error Counter
pub const CAN_ERR_TEC_M = usize(0x000000FF);  // Transmit Error Counter
pub const CAN_ERR_REC_S = usize(8);
pub const CAN_ERR_TEC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_BIT register.
//
//*****************************************************************************
pub const CAN_BIT_TSEG2_M = usize(0x00007000);  // Time Segment after Sample Point
pub const CAN_BIT_TSEG1_M = usize(0x00000F00);  // Time Segment Before Sample Point
pub const CAN_BIT_SJW_M = usize(0x000000C0);  // (Re)Synchronization Jump Width
pub const CAN_BIT_BRP_M = usize(0x0000003F);  // Baud Rate Prescaler
pub const CAN_BIT_TSEG2_S = usize(12);
pub const CAN_BIT_TSEG1_S = usize(8);
pub const CAN_BIT_SJW_S = usize(6);
pub const CAN_BIT_BRP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_INT register.
//
//*****************************************************************************
pub const CAN_INT_INTID_M = usize(0x0000FFFF);  // Interrupt Identifier
pub const CAN_INT_INTID_NONE = usize(0x00000000);  // No interrupt pending
pub const CAN_INT_INTID_STATUS = usize(0x00008000);  // Status Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TST register.
//
//*****************************************************************************
pub const CAN_TST_RX = usize(0x00000080);  // Receive Observation
pub const CAN_TST_TX_M = usize(0x00000060);  // Transmit Control
pub const CAN_TST_TX_CANCTL = usize(0x00000000);  // CAN Module Control
pub const CAN_TST_TX_SAMPLE = usize(0x00000020);  // Sample Point
pub const CAN_TST_TX_DOMINANT = usize(0x00000040);  // Driven Low
pub const CAN_TST_TX_RECESSIVE = usize(0x00000060);  // Driven High
pub const CAN_TST_LBACK = usize(0x00000010);  // Loopback Mode
pub const CAN_TST_SILENT = usize(0x00000008);  // Silent Mode
pub const CAN_TST_BASIC = usize(0x00000004);  // Basic Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_BRPE register.
//
//*****************************************************************************
pub const CAN_BRPE_BRPE_M = usize(0x0000000F);  // Baud Rate Prescaler Extension
pub const CAN_BRPE_BRPE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1CRQ register.
//
//*****************************************************************************
pub const CAN_IF1CRQ_BUSY = usize(0x00008000);  // Busy Flag
pub const CAN_IF1CRQ_MNUM_M = usize(0x0000003F);  // Message Number
pub const CAN_IF1CRQ_MNUM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1CMSK register.
//
//*****************************************************************************
pub const CAN_IF1CMSK_WRNRD = usize(0x00000080);  // Write, Not Read
pub const CAN_IF1CMSK_MASK = usize(0x00000040);  // Access Mask Bits
pub const CAN_IF1CMSK_ARB = usize(0x00000020);  // Access Arbitration Bits
pub const CAN_IF1CMSK_CONTROL = usize(0x00000010);  // Access Control Bits
pub const CAN_IF1CMSK_CLRINTPND = usize(0x00000008);  // Clear Interrupt Pending Bit
pub const CAN_IF1CMSK_NEWDAT = usize(0x00000004);  // Access New Data
pub const CAN_IF1CMSK_TXRQST = usize(0x00000004);  // Access Transmission Request
pub const CAN_IF1CMSK_DATAA = usize(0x00000002);  // Access Data Byte 0 to 3
pub const CAN_IF1CMSK_DATAB = usize(0x00000001);  // Access Data Byte 4 to 7

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MSK1 register.
//
//*****************************************************************************
pub const CAN_IF1MSK1_IDMSK_M = usize(0x0000FFFF);  // Identifier Mask
pub const CAN_IF1MSK1_IDMSK_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MSK2 register.
//
//*****************************************************************************
pub const CAN_IF1MSK2_MXTD = usize(0x00008000);  // Mask Extended Identifier
pub const CAN_IF1MSK2_MDIR = usize(0x00004000);  // Mask Message Direction
pub const CAN_IF1MSK2_IDMSK_M = usize(0x00001FFF);  // Identifier Mask
pub const CAN_IF1MSK2_IDMSK_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1ARB1 register.
//
//*****************************************************************************
pub const CAN_IF1ARB1_ID_M = usize(0x0000FFFF);  // Message Identifier
pub const CAN_IF1ARB1_ID_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1ARB2 register.
//
//*****************************************************************************
pub const CAN_IF1ARB2_MSGVAL = usize(0x00008000);  // Message Valid
pub const CAN_IF1ARB2_XTD = usize(0x00004000);  // Extended Identifier
pub const CAN_IF1ARB2_DIR = usize(0x00002000);  // Message Direction
pub const CAN_IF1ARB2_ID_M = usize(0x00001FFF);  // Message Identifier
pub const CAN_IF1ARB2_ID_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MCTL register.
//
//*****************************************************************************
pub const CAN_IF1MCTL_NEWDAT = usize(0x00008000);  // New Data
pub const CAN_IF1MCTL_MSGLST = usize(0x00004000);  // Message Lost
pub const CAN_IF1MCTL_INTPND = usize(0x00002000);  // Interrupt Pending
pub const CAN_IF1MCTL_UMASK = usize(0x00001000);  // Use Acceptance Mask
pub const CAN_IF1MCTL_TXIE = usize(0x00000800);  // Transmit Interrupt Enable
pub const CAN_IF1MCTL_RXIE = usize(0x00000400);  // Receive Interrupt Enable
pub const CAN_IF1MCTL_RMTEN = usize(0x00000200);  // Remote Enable
pub const CAN_IF1MCTL_TXRQST = usize(0x00000100);  // Transmit Request
pub const CAN_IF1MCTL_EOB = usize(0x00000080);  // End of Buffer
pub const CAN_IF1MCTL_DLC_M = usize(0x0000000F);  // Data Length Code
pub const CAN_IF1MCTL_DLC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DA1 register.
//
//*****************************************************************************
pub const CAN_IF1DA1_DATA_M = usize(0x0000FFFF);  // Data
pub const CAN_IF1DA1_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DA2 register.
//
//*****************************************************************************
pub const CAN_IF1DA2_DATA_M = usize(0x0000FFFF);  // Data
pub const CAN_IF1DA2_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DB1 register.
//
//*****************************************************************************
pub const CAN_IF1DB1_DATA_M = usize(0x0000FFFF);  // Data
pub const CAN_IF1DB1_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DB2 register.
//
//*****************************************************************************
pub const CAN_IF1DB2_DATA_M = usize(0x0000FFFF);  // Data
pub const CAN_IF1DB2_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2CRQ register.
//
//*****************************************************************************
pub const CAN_IF2CRQ_BUSY = usize(0x00008000);  // Busy Flag
pub const CAN_IF2CRQ_MNUM_M = usize(0x0000003F);  // Message Number
pub const CAN_IF2CRQ_MNUM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2CMSK register.
//
//*****************************************************************************
pub const CAN_IF2CMSK_WRNRD = usize(0x00000080);  // Write, Not Read
pub const CAN_IF2CMSK_MASK = usize(0x00000040);  // Access Mask Bits
pub const CAN_IF2CMSK_ARB = usize(0x00000020);  // Access Arbitration Bits
pub const CAN_IF2CMSK_CONTROL = usize(0x00000010);  // Access Control Bits
pub const CAN_IF2CMSK_CLRINTPND = usize(0x00000008);  // Clear Interrupt Pending Bit
pub const CAN_IF2CMSK_NEWDAT = usize(0x00000004);  // Access New Data
pub const CAN_IF2CMSK_TXRQST = usize(0x00000004);  // Access Transmission Request
pub const CAN_IF2CMSK_DATAA = usize(0x00000002);  // Access Data Byte 0 to 3
pub const CAN_IF2CMSK_DATAB = usize(0x00000001);  // Access Data Byte 4 to 7

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MSK1 register.
//
//*****************************************************************************
pub const CAN_IF2MSK1_IDMSK_M = usize(0x0000FFFF);  // Identifier Mask
pub const CAN_IF2MSK1_IDMSK_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MSK2 register.
//
//*****************************************************************************
pub const CAN_IF2MSK2_MXTD = usize(0x00008000);  // Mask Extended Identifier
pub const CAN_IF2MSK2_MDIR = usize(0x00004000);  // Mask Message Direction
pub const CAN_IF2MSK2_IDMSK_M = usize(0x00001FFF);  // Identifier Mask
pub const CAN_IF2MSK2_IDMSK_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2ARB1 register.
//
//*****************************************************************************
pub const CAN_IF2ARB1_ID_M = usize(0x0000FFFF);  // Message Identifier
pub const CAN_IF2ARB1_ID_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2ARB2 register.
//
//*****************************************************************************
pub const CAN_IF2ARB2_MSGVAL = usize(0x00008000);  // Message Valid
pub const CAN_IF2ARB2_XTD = usize(0x00004000);  // Extended Identifier
pub const CAN_IF2ARB2_DIR = usize(0x00002000);  // Message Direction
pub const CAN_IF2ARB2_ID_M = usize(0x00001FFF);  // Message Identifier
pub const CAN_IF2ARB2_ID_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MCTL register.
//
//*****************************************************************************
pub const CAN_IF2MCTL_NEWDAT = usize(0x00008000);  // New Data
pub const CAN_IF2MCTL_MSGLST = usize(0x00004000);  // Message Lost
pub const CAN_IF2MCTL_INTPND = usize(0x00002000);  // Interrupt Pending
pub const CAN_IF2MCTL_UMASK = usize(0x00001000);  // Use Acceptance Mask
pub const CAN_IF2MCTL_TXIE = usize(0x00000800);  // Transmit Interrupt Enable
pub const CAN_IF2MCTL_RXIE = usize(0x00000400);  // Receive Interrupt Enable
pub const CAN_IF2MCTL_RMTEN = usize(0x00000200);  // Remote Enable
pub const CAN_IF2MCTL_TXRQST = usize(0x00000100);  // Transmit Request
pub const CAN_IF2MCTL_EOB = usize(0x00000080);  // End of Buffer
pub const CAN_IF2MCTL_DLC_M = usize(0x0000000F);  // Data Length Code
pub const CAN_IF2MCTL_DLC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DA1 register.
//
//*****************************************************************************
pub const CAN_IF2DA1_DATA_M = usize(0x0000FFFF);  // Data
pub const CAN_IF2DA1_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DA2 register.
//
//*****************************************************************************
pub const CAN_IF2DA2_DATA_M = usize(0x0000FFFF);  // Data
pub const CAN_IF2DA2_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DB1 register.
//
//*****************************************************************************
pub const CAN_IF2DB1_DATA_M = usize(0x0000FFFF);  // Data
pub const CAN_IF2DB1_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DB2 register.
//
//*****************************************************************************
pub const CAN_IF2DB2_DATA_M = usize(0x0000FFFF);  // Data
pub const CAN_IF2DB2_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TXRQ1 register.
//
//*****************************************************************************
pub const CAN_TXRQ1_TXRQST_M = usize(0x0000FFFF);  // Transmission Request Bits
pub const CAN_TXRQ1_TXRQST_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TXRQ2 register.
//
//*****************************************************************************
pub const CAN_TXRQ2_TXRQST_M = usize(0x0000FFFF);  // Transmission Request Bits
pub const CAN_TXRQ2_TXRQST_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_NWDA1 register.
//
//*****************************************************************************
pub const CAN_NWDA1_NEWDAT_M = usize(0x0000FFFF);  // New Data Bits
pub const CAN_NWDA1_NEWDAT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_NWDA2 register.
//
//*****************************************************************************
pub const CAN_NWDA2_NEWDAT_M = usize(0x0000FFFF);  // New Data Bits
pub const CAN_NWDA2_NEWDAT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG1INT register.
//
//*****************************************************************************
pub const CAN_MSG1INT_INTPND_M = usize(0x0000FFFF);  // Interrupt Pending Bits
pub const CAN_MSG1INT_INTPND_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG2INT register.
//
//*****************************************************************************
pub const CAN_MSG2INT_INTPND_M = usize(0x0000FFFF);  // Interrupt Pending Bits
pub const CAN_MSG2INT_INTPND_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG1VAL register.
//
//*****************************************************************************
pub const CAN_MSG1VAL_MSGVAL_M = usize(0x0000FFFF);  // Message Valid Bits
pub const CAN_MSG1VAL_MSGVAL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG2VAL register.
//
//*****************************************************************************
pub const CAN_MSG2VAL_MSGVAL_M = usize(0x0000FFFF);  // Message Valid Bits
pub const CAN_MSG2VAL_MSGVAL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FADDR register.
//
//*****************************************************************************
pub const USB_FADDR_M = usize(0x0000007F);  // Function Address
pub const USB_FADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_POWER register.
//
//*****************************************************************************
pub const USB_POWER_ISOUP = usize(0x00000080);  // Isochronous Update
pub const USB_POWER_SOFTCONN = usize(0x00000040);  // Soft Connect/Disconnect
pub const USB_POWER_HSENAB = usize(0x00000020);  // High Speed Enable
pub const USB_POWER_HSMODE = usize(0x00000010);  // High Speed Enable
pub const USB_POWER_RESET = usize(0x00000008);  // RESET Signaling
pub const USB_POWER_RESUME = usize(0x00000004);  // RESUME Signaling
pub const USB_POWER_SUSPEND = usize(0x00000002);  // SUSPEND Mode
pub const USB_POWER_PWRDNPHY = usize(0x00000001);  // Power Down PHY

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXIS register.
//
//*****************************************************************************
pub const USB_TXIS_EP7 = usize(0x00000080);  // TX Endpoint 7 Interrupt
pub const USB_TXIS_EP6 = usize(0x00000040);  // TX Endpoint 6 Interrupt
pub const USB_TXIS_EP5 = usize(0x00000020);  // TX Endpoint 5 Interrupt
pub const USB_TXIS_EP4 = usize(0x00000010);  // TX Endpoint 4 Interrupt
pub const USB_TXIS_EP3 = usize(0x00000008);  // TX Endpoint 3 Interrupt
pub const USB_TXIS_EP2 = usize(0x00000004);  // TX Endpoint 2 Interrupt
pub const USB_TXIS_EP1 = usize(0x00000002);  // TX Endpoint 1 Interrupt
pub const USB_TXIS_EP0 = usize(0x00000001);  // TX and RX Endpoint 0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXIS register.
//
//*****************************************************************************
pub const USB_RXIS_EP7 = usize(0x00000080);  // RX Endpoint 7 Interrupt
pub const USB_RXIS_EP6 = usize(0x00000040);  // RX Endpoint 6 Interrupt
pub const USB_RXIS_EP5 = usize(0x00000020);  // RX Endpoint 5 Interrupt
pub const USB_RXIS_EP4 = usize(0x00000010);  // RX Endpoint 4 Interrupt
pub const USB_RXIS_EP3 = usize(0x00000008);  // RX Endpoint 3 Interrupt
pub const USB_RXIS_EP2 = usize(0x00000004);  // RX Endpoint 2 Interrupt
pub const USB_RXIS_EP1 = usize(0x00000002);  // RX Endpoint 1 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXIE register.
//
//*****************************************************************************
pub const USB_TXIE_EP7 = usize(0x00000080);  // TX Endpoint 7 Interrupt Enable
pub const USB_TXIE_EP6 = usize(0x00000040);  // TX Endpoint 6 Interrupt Enable
pub const USB_TXIE_EP5 = usize(0x00000020);  // TX Endpoint 5 Interrupt Enable
pub const USB_TXIE_EP4 = usize(0x00000010);  // TX Endpoint 4 Interrupt Enable
pub const USB_TXIE_EP3 = usize(0x00000008);  // TX Endpoint 3 Interrupt Enable
pub const USB_TXIE_EP2 = usize(0x00000004);  // TX Endpoint 2 Interrupt Enable
pub const USB_TXIE_EP1 = usize(0x00000002);  // TX Endpoint 1 Interrupt Enable
pub const USB_TXIE_EP0 = usize(0x00000001);  // TX and RX Endpoint 0 Interrupt
                                            // Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXIE register.
//
//*****************************************************************************
pub const USB_RXIE_EP7 = usize(0x00000080);  // RX Endpoint 7 Interrupt Enable
pub const USB_RXIE_EP6 = usize(0x00000040);  // RX Endpoint 6 Interrupt Enable
pub const USB_RXIE_EP5 = usize(0x00000020);  // RX Endpoint 5 Interrupt Enable
pub const USB_RXIE_EP4 = usize(0x00000010);  // RX Endpoint 4 Interrupt Enable
pub const USB_RXIE_EP3 = usize(0x00000008);  // RX Endpoint 3 Interrupt Enable
pub const USB_RXIE_EP2 = usize(0x00000004);  // RX Endpoint 2 Interrupt Enable
pub const USB_RXIE_EP1 = usize(0x00000002);  // RX Endpoint 1 Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IS register.
//
//*****************************************************************************
pub const USB_IS_VBUSERR = usize(0x00000080);  // VBUS Error (OTG only)
pub const USB_IS_SESREQ = usize(0x00000040);  // SESSION REQUEST (OTG only)
pub const USB_IS_DISCON = usize(0x00000020);  // Session Disconnect (OTG only)
pub const USB_IS_CONN = usize(0x00000010);  // Session Connect
pub const USB_IS_SOF = usize(0x00000008);  // Start of Frame
pub const USB_IS_BABBLE = usize(0x00000004);  // Babble Detected
pub const USB_IS_RESET = usize(0x00000004);  // RESET Signaling Detected
pub const USB_IS_RESUME = usize(0x00000002);  // RESUME Signaling Detected
pub const USB_IS_SUSPEND = usize(0x00000001);  // SUSPEND Signaling Detected

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IE register.
//
//*****************************************************************************
pub const USB_IE_VBUSERR = usize(0x00000080);  // Enable VBUS Error Interrupt (OTG
                                            // only)
pub const USB_IE_SESREQ = usize(0x00000040);  // Enable Session Request (OTG
                                            // only)
pub const USB_IE_DISCON = usize(0x00000020);  // Enable Disconnect Interrupt
pub const USB_IE_CONN = usize(0x00000010);  // Enable Connect Interrupt
pub const USB_IE_SOF = usize(0x00000008);  // Enable Start-of-Frame Interrupt
pub const USB_IE_BABBLE = usize(0x00000004);  // Enable Babble Interrupt
pub const USB_IE_RESET = usize(0x00000004);  // Enable RESET Interrupt
pub const USB_IE_RESUME = usize(0x00000002);  // Enable RESUME Interrupt
pub const USB_IE_SUSPND = usize(0x00000001);  // Enable SUSPEND Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FRAME register.
//
//*****************************************************************************
pub const USB_FRAME_M = usize(0x000007FF);  // Frame Number
pub const USB_FRAME_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPIDX register.
//
//*****************************************************************************
pub const USB_EPIDX_EPIDX_M = usize(0x0000000F);  // Endpoint Index
pub const USB_EPIDX_EPIDX_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TEST register.
//
//*****************************************************************************
pub const USB_TEST_FORCEH = usize(0x00000080);  // Force Host Mode
pub const USB_TEST_FIFOACC = usize(0x00000040);  // FIFO Access
pub const USB_TEST_FORCEFS = usize(0x00000020);  // Force Full-Speed Mode
pub const USB_TEST_FORCEHS = usize(0x00000010);  // Force High-Speed Mode
pub const USB_TEST_TESTPKT = usize(0x00000008);  // Test Packet Mode Enable
pub const USB_TEST_TESTK = usize(0x00000004);  // Test_K Mode Enable
pub const USB_TEST_TESTJ = usize(0x00000002);  // Test_J Mode Enable
pub const USB_TEST_TESTSE0NAK = usize(0x00000001);  // Test_SE0_NAK Test Mode Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO0 register.
//
//*****************************************************************************
pub const USB_FIFO0_EPDATA_M = usize(0xFFFFFFFF);  // Endpoint Data
pub const USB_FIFO0_EPDATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO1 register.
//
//*****************************************************************************
pub const USB_FIFO1_EPDATA_M = usize(0xFFFFFFFF);  // Endpoint Data
pub const USB_FIFO1_EPDATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO2 register.
//
//*****************************************************************************
pub const USB_FIFO2_EPDATA_M = usize(0xFFFFFFFF);  // Endpoint Data
pub const USB_FIFO2_EPDATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO3 register.
//
//*****************************************************************************
pub const USB_FIFO3_EPDATA_M = usize(0xFFFFFFFF);  // Endpoint Data
pub const USB_FIFO3_EPDATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO4 register.
//
//*****************************************************************************
pub const USB_FIFO4_EPDATA_M = usize(0xFFFFFFFF);  // Endpoint Data
pub const USB_FIFO4_EPDATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO5 register.
//
//*****************************************************************************
pub const USB_FIFO5_EPDATA_M = usize(0xFFFFFFFF);  // Endpoint Data
pub const USB_FIFO5_EPDATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO6 register.
//
//*****************************************************************************
pub const USB_FIFO6_EPDATA_M = usize(0xFFFFFFFF);  // Endpoint Data
pub const USB_FIFO6_EPDATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO7 register.
//
//*****************************************************************************
pub const USB_FIFO7_EPDATA_M = usize(0xFFFFFFFF);  // Endpoint Data
pub const USB_FIFO7_EPDATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DEVCTL register.
//
//*****************************************************************************
pub const USB_DEVCTL_DEV = usize(0x00000080);  // Device Mode (OTG only)
pub const USB_DEVCTL_FSDEV = usize(0x00000040);  // Full-Speed Device Detected
pub const USB_DEVCTL_LSDEV = usize(0x00000020);  // Low-Speed Device Detected
pub const USB_DEVCTL_VBUS_M = usize(0x00000018);  // VBUS Level (OTG only)
pub const USB_DEVCTL_VBUS_NONE = usize(0x00000000);  // Below SessionEnd
pub const USB_DEVCTL_VBUS_SEND = usize(0x00000008);  // Above SessionEnd, below AValid
pub const USB_DEVCTL_VBUS_AVALID = usize(0x00000010);  // Above AValid, below VBUSValid
pub const USB_DEVCTL_VBUS_VALID = usize(0x00000018);  // Above VBUSValid
pub const USB_DEVCTL_HOST = usize(0x00000004);  // Host Mode
pub const USB_DEVCTL_HOSTREQ = usize(0x00000002);  // Host Request (OTG only)
pub const USB_DEVCTL_SESSION = usize(0x00000001);  // Session Start/End (OTG only)

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CCONF register.
//
//*****************************************************************************
pub const USB_CCONF_TXEDMA = usize(0x00000002);  // TX Early DMA Enable
pub const USB_CCONF_RXEDMA = usize(0x00000001);  // TX Early DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFIFOSZ register.
//
//*****************************************************************************
pub const USB_TXFIFOSZ_DPB = usize(0x00000010);  // Double Packet Buffer Support
pub const USB_TXFIFOSZ_SIZE_M = usize(0x0000000F);  // Max Packet Size
pub const USB_TXFIFOSZ_SIZE_8 = usize(0x00000000);  // 8
pub const USB_TXFIFOSZ_SIZE_16 = usize(0x00000001);  // 16
pub const USB_TXFIFOSZ_SIZE_32 = usize(0x00000002);  // 32
pub const USB_TXFIFOSZ_SIZE_64 = usize(0x00000003);  // 64
pub const USB_TXFIFOSZ_SIZE_128 = usize(0x00000004);  // 128
pub const USB_TXFIFOSZ_SIZE_256 = usize(0x00000005);  // 256
pub const USB_TXFIFOSZ_SIZE_512 = usize(0x00000006);  // 512
pub const USB_TXFIFOSZ_SIZE_1024 = usize(0x00000007);  // 1024
pub const USB_TXFIFOSZ_SIZE_2048 = usize(0x00000008);  // 2048

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFIFOSZ register.
//
//*****************************************************************************
pub const USB_RXFIFOSZ_DPB = usize(0x00000010);  // Double Packet Buffer Support
pub const USB_RXFIFOSZ_SIZE_M = usize(0x0000000F);  // Max Packet Size
pub const USB_RXFIFOSZ_SIZE_8 = usize(0x00000000);  // 8
pub const USB_RXFIFOSZ_SIZE_16 = usize(0x00000001);  // 16
pub const USB_RXFIFOSZ_SIZE_32 = usize(0x00000002);  // 32
pub const USB_RXFIFOSZ_SIZE_64 = usize(0x00000003);  // 64
pub const USB_RXFIFOSZ_SIZE_128 = usize(0x00000004);  // 128
pub const USB_RXFIFOSZ_SIZE_256 = usize(0x00000005);  // 256
pub const USB_RXFIFOSZ_SIZE_512 = usize(0x00000006);  // 512
pub const USB_RXFIFOSZ_SIZE_1024 = usize(0x00000007);  // 1024
pub const USB_RXFIFOSZ_SIZE_2048 = usize(0x00000008);  // 2048

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFIFOADD
// register.
//
//*****************************************************************************
pub const USB_TXFIFOADD_ADDR_M = usize(0x000001FF);  // Transmit/Receive Start Address
pub const USB_TXFIFOADD_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFIFOADD
// register.
//
//*****************************************************************************
pub const USB_RXFIFOADD_ADDR_M = usize(0x000001FF);  // Transmit/Receive Start Address
pub const USB_RXFIFOADD_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_ULPIVBUSCTL
// register.
//
//*****************************************************************************
pub const USB_ULPIVBUSCTL_USEEXTVBUSIND = usize(0x00000002);  // Use External VBUS Indicator
pub const USB_ULPIVBUSCTL_USEEXTVBUS = usize(0x00000001);  // Use External VBUS

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_ULPIREGDATA
// register.
//
//*****************************************************************************
pub const USB_ULPIREGDATA_REGDATA_M = usize(0x000000FF);  // Register Data
pub const USB_ULPIREGDATA_REGDATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_ULPIREGADDR
// register.
//
//*****************************************************************************
pub const USB_ULPIREGADDR_ADDR_M = usize(0x000000FF);  // Register Address
pub const USB_ULPIREGADDR_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_ULPIREGCTL
// register.
//
//*****************************************************************************
pub const USB_ULPIREGCTL_RDWR = usize(0x00000004);  // Read/Write Control
pub const USB_ULPIREGCTL_REGCMPLT = usize(0x00000002);  // Register Access Complete
pub const USB_ULPIREGCTL_REGACC = usize(0x00000001);  // Initiate Register Access

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPINFO register.
//
//*****************************************************************************
pub const USB_EPINFO_RXEP_M = usize(0x000000F0);  // RX Endpoints
pub const USB_EPINFO_TXEP_M = usize(0x0000000F);  // TX Endpoints
pub const USB_EPINFO_RXEP_S = usize(4);
pub const USB_EPINFO_TXEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RAMINFO register.
//
//*****************************************************************************
pub const USB_RAMINFO_DMACHAN_M = usize(0x000000F0);  // DMA Channels
pub const USB_RAMINFO_RAMBITS_M = usize(0x0000000F);  // RAM Address Bus Width
pub const USB_RAMINFO_DMACHAN_S = usize(4);
pub const USB_RAMINFO_RAMBITS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CONTIM register.
//
//*****************************************************************************
pub const USB_CONTIM_WTCON_M = usize(0x000000F0);  // Connect Wait
pub const USB_CONTIM_WTID_M = usize(0x0000000F);  // Wait ID
pub const USB_CONTIM_WTCON_S = usize(4);
pub const USB_CONTIM_WTID_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VPLEN register.
//
//*****************************************************************************
pub const USB_VPLEN_VPLEN_M = usize(0x000000FF);  // VBUS Pulse Length
pub const USB_VPLEN_VPLEN_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_HSEOF register.
//
//*****************************************************************************
pub const USB_HSEOF_HSEOFG_M = usize(0x000000FF);  // HIgh-Speed End-of-Frame Gap
pub const USB_HSEOF_HSEOFG_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FSEOF register.
//
//*****************************************************************************
pub const USB_FSEOF_FSEOFG_M = usize(0x000000FF);  // Full-Speed End-of-Frame Gap
pub const USB_FSEOF_FSEOFG_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LSEOF register.
//
//*****************************************************************************
pub const USB_LSEOF_LSEOFG_M = usize(0x000000FF);  // Low-Speed End-of-Frame Gap
pub const USB_LSEOF_LSEOFG_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR0
// register.
//
//*****************************************************************************
pub const USB_TXFUNCADDR0_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_TXFUNCADDR0_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR0
// register.
//
//*****************************************************************************
pub const USB_TXHUBADDR0_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_TXHUBADDR0_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT0
// register.
//
//*****************************************************************************
pub const USB_TXHUBPORT0_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_TXHUBPORT0_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR1
// register.
//
//*****************************************************************************
pub const USB_TXFUNCADDR1_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_TXFUNCADDR1_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR1
// register.
//
//*****************************************************************************
pub const USB_TXHUBADDR1_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_TXHUBADDR1_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT1
// register.
//
//*****************************************************************************
pub const USB_TXHUBPORT1_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_TXHUBPORT1_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR1
// register.
//
//*****************************************************************************
pub const USB_RXFUNCADDR1_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_RXFUNCADDR1_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR1
// register.
//
//*****************************************************************************
pub const USB_RXHUBADDR1_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_RXHUBADDR1_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT1
// register.
//
//*****************************************************************************
pub const USB_RXHUBPORT1_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_RXHUBPORT1_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR2
// register.
//
//*****************************************************************************
pub const USB_TXFUNCADDR2_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_TXFUNCADDR2_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR2
// register.
//
//*****************************************************************************
pub const USB_TXHUBADDR2_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_TXHUBADDR2_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT2
// register.
//
//*****************************************************************************
pub const USB_TXHUBPORT2_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_TXHUBPORT2_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR2
// register.
//
//*****************************************************************************
pub const USB_RXFUNCADDR2_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_RXFUNCADDR2_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR2
// register.
//
//*****************************************************************************
pub const USB_RXHUBADDR2_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_RXHUBADDR2_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT2
// register.
//
//*****************************************************************************
pub const USB_RXHUBPORT2_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_RXHUBPORT2_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR3
// register.
//
//*****************************************************************************
pub const USB_TXFUNCADDR3_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_TXFUNCADDR3_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR3
// register.
//
//*****************************************************************************
pub const USB_TXHUBADDR3_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_TXHUBADDR3_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT3
// register.
//
//*****************************************************************************
pub const USB_TXHUBPORT3_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_TXHUBPORT3_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR3
// register.
//
//*****************************************************************************
pub const USB_RXFUNCADDR3_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_RXFUNCADDR3_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR3
// register.
//
//*****************************************************************************
pub const USB_RXHUBADDR3_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_RXHUBADDR3_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT3
// register.
//
//*****************************************************************************
pub const USB_RXHUBPORT3_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_RXHUBPORT3_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR4
// register.
//
//*****************************************************************************
pub const USB_TXFUNCADDR4_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_TXFUNCADDR4_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR4
// register.
//
//*****************************************************************************
pub const USB_TXHUBADDR4_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_TXHUBADDR4_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT4
// register.
//
//*****************************************************************************
pub const USB_TXHUBPORT4_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_TXHUBPORT4_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR4
// register.
//
//*****************************************************************************
pub const USB_RXFUNCADDR4_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_RXFUNCADDR4_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR4
// register.
//
//*****************************************************************************
pub const USB_RXHUBADDR4_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_RXHUBADDR4_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT4
// register.
//
//*****************************************************************************
pub const USB_RXHUBPORT4_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_RXHUBPORT4_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR5
// register.
//
//*****************************************************************************
pub const USB_TXFUNCADDR5_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_TXFUNCADDR5_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR5
// register.
//
//*****************************************************************************
pub const USB_TXHUBADDR5_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_TXHUBADDR5_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT5
// register.
//
//*****************************************************************************
pub const USB_TXHUBPORT5_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_TXHUBPORT5_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR5
// register.
//
//*****************************************************************************
pub const USB_RXFUNCADDR5_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_RXFUNCADDR5_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR5
// register.
//
//*****************************************************************************
pub const USB_RXHUBADDR5_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_RXHUBADDR5_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT5
// register.
//
//*****************************************************************************
pub const USB_RXHUBPORT5_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_RXHUBPORT5_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR6
// register.
//
//*****************************************************************************
pub const USB_TXFUNCADDR6_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_TXFUNCADDR6_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR6
// register.
//
//*****************************************************************************
pub const USB_TXHUBADDR6_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_TXHUBADDR6_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT6
// register.
//
//*****************************************************************************
pub const USB_TXHUBPORT6_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_TXHUBPORT6_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR6
// register.
//
//*****************************************************************************
pub const USB_RXFUNCADDR6_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_RXFUNCADDR6_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR6
// register.
//
//*****************************************************************************
pub const USB_RXHUBADDR6_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_RXHUBADDR6_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT6
// register.
//
//*****************************************************************************
pub const USB_RXHUBPORT6_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_RXHUBPORT6_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR7
// register.
//
//*****************************************************************************
pub const USB_TXFUNCADDR7_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_TXFUNCADDR7_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR7
// register.
//
//*****************************************************************************
pub const USB_TXHUBADDR7_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_TXHUBADDR7_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT7
// register.
//
//*****************************************************************************
pub const USB_TXHUBPORT7_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_TXHUBPORT7_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR7
// register.
//
//*****************************************************************************
pub const USB_RXFUNCADDR7_ADDR_M = usize(0x0000007F);  // Device Address
pub const USB_RXFUNCADDR7_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR7
// register.
//
//*****************************************************************************
pub const USB_RXHUBADDR7_ADDR_M = usize(0x0000007F);  // Hub Address
pub const USB_RXHUBADDR7_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT7
// register.
//
//*****************************************************************************
pub const USB_RXHUBPORT7_PORT_M = usize(0x0000007F);  // Hub Port
pub const USB_RXHUBPORT7_PORT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CSRL0 register.
//
//*****************************************************************************
pub const USB_CSRL0_NAKTO = usize(0x00000080);  // NAK Timeout
pub const USB_CSRL0_SETENDC = usize(0x00000080);  // Setup End Clear
pub const USB_CSRL0_STATUS = usize(0x00000040);  // STATUS Packet
pub const USB_CSRL0_RXRDYC = usize(0x00000040);  // RXRDY Clear
pub const USB_CSRL0_REQPKT = usize(0x00000020);  // Request Packet
pub const USB_CSRL0_STALL = usize(0x00000020);  // Send Stall
pub const USB_CSRL0_SETEND = usize(0x00000010);  // Setup End
pub const USB_CSRL0_ERROR = usize(0x00000010);  // Error
pub const USB_CSRL0_DATAEND = usize(0x00000008);  // Data End
pub const USB_CSRL0_SETUP = usize(0x00000008);  // Setup Packet
pub const USB_CSRL0_STALLED = usize(0x00000004);  // Endpoint Stalled
pub const USB_CSRL0_TXRDY = usize(0x00000002);  // Transmit Packet Ready
pub const USB_CSRL0_RXRDY = usize(0x00000001);  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CSRH0 register.
//
//*****************************************************************************
pub const USB_CSRH0_DISPING = usize(0x00000008);  // PING Disable
pub const USB_CSRH0_DTWE = usize(0x00000004);  // Data Toggle Write Enable
pub const USB_CSRH0_DT = usize(0x00000002);  // Data Toggle
pub const USB_CSRH0_FLUSH = usize(0x00000001);  // Flush FIFO

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_COUNT0 register.
//
//*****************************************************************************
pub const USB_COUNT0_COUNT_M = usize(0x0000007F);  // FIFO Count
pub const USB_COUNT0_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TYPE0 register.
//
//*****************************************************************************
pub const USB_TYPE0_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_TYPE0_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_TYPE0_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_TYPE0_SPEED_LOW = usize(0x000000C0);  // Low

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_NAKLMT register.
//
//*****************************************************************************
pub const USB_NAKLMT_NAKLMT_M = usize(0x0000001F);  // EP0 NAK Limit
pub const USB_NAKLMT_NAKLMT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP1 register.
//
//*****************************************************************************
pub const USB_TXMAXP1_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_TXMAXP1_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL1 register.
//
//*****************************************************************************
pub const USB_TXCSRL1_NAKTO = usize(0x00000080);  // NAK Timeout
pub const USB_TXCSRL1_CLRDT = usize(0x00000040);  // Clear Data Toggle
pub const USB_TXCSRL1_STALLED = usize(0x00000020);  // Endpoint Stalled
pub const USB_TXCSRL1_STALL = usize(0x00000010);  // Send STALL
pub const USB_TXCSRL1_SETUP = usize(0x00000010);  // Setup Packet
pub const USB_TXCSRL1_FLUSH = usize(0x00000008);  // Flush FIFO
pub const USB_TXCSRL1_ERROR = usize(0x00000004);  // Error
pub const USB_TXCSRL1_UNDRN = usize(0x00000004);  // Underrun
pub const USB_TXCSRL1_FIFONE = usize(0x00000002);  // FIFO Not Empty
pub const USB_TXCSRL1_TXRDY = usize(0x00000001);  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH1 register.
//
//*****************************************************************************
pub const USB_TXCSRH1_AUTOSET = usize(0x00000080);  // Auto Set
pub const USB_TXCSRH1_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_TXCSRH1_MODE = usize(0x00000020);  // Mode
pub const USB_TXCSRH1_DMAEN = usize(0x00000010);  // DMA Request Enable
pub const USB_TXCSRH1_FDT = usize(0x00000008);  // Force Data Toggle
pub const USB_TXCSRH1_DMAMOD = usize(0x00000004);  // DMA Request Mode
pub const USB_TXCSRH1_DTWE = usize(0x00000002);  // Data Toggle Write Enable
pub const USB_TXCSRH1_DT = usize(0x00000001);  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP1 register.
//
//*****************************************************************************
pub const USB_RXMAXP1_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_RXMAXP1_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL1 register.
//
//*****************************************************************************
pub const USB_RXCSRL1_CLRDT = usize(0x00000080);  // Clear Data Toggle
pub const USB_RXCSRL1_STALLED = usize(0x00000040);  // Endpoint Stalled
pub const USB_RXCSRL1_STALL = usize(0x00000020);  // Send STALL
pub const USB_RXCSRL1_REQPKT = usize(0x00000020);  // Request Packet
pub const USB_RXCSRL1_FLUSH = usize(0x00000010);  // Flush FIFO
pub const USB_RXCSRL1_DATAERR = usize(0x00000008);  // Data Error
pub const USB_RXCSRL1_NAKTO = usize(0x00000008);  // NAK Timeout
pub const USB_RXCSRL1_OVER = usize(0x00000004);  // Overrun
pub const USB_RXCSRL1_ERROR = usize(0x00000004);  // Error
pub const USB_RXCSRL1_FULL = usize(0x00000002);  // FIFO Full
pub const USB_RXCSRL1_RXRDY = usize(0x00000001);  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH1 register.
//
//*****************************************************************************
pub const USB_RXCSRH1_AUTOCL = usize(0x00000080);  // Auto Clear
pub const USB_RXCSRH1_AUTORQ = usize(0x00000040);  // Auto Request
pub const USB_RXCSRH1_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_RXCSRH1_DMAEN = usize(0x00000020);  // DMA Request Enable
pub const USB_RXCSRH1_DISNYET = usize(0x00000010);  // Disable NYET
pub const USB_RXCSRH1_PIDERR = usize(0x00000010);  // PID Error
pub const USB_RXCSRH1_DMAMOD = usize(0x00000008);  // DMA Request Mode
pub const USB_RXCSRH1_DTWE = usize(0x00000004);  // Data Toggle Write Enable
pub const USB_RXCSRH1_DT = usize(0x00000002);  // Data Toggle
pub const USB_RXCSRH1_INCOMPRX = usize(0x00000001);  // Incomplete RX Transmission
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT1 register.
//
//*****************************************************************************
pub const USB_RXCOUNT1_COUNT_M = usize(0x00001FFF);  // Receive Packet Count
pub const USB_RXCOUNT1_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE1 register.
//
//*****************************************************************************
pub const USB_TXTYPE1_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_TXTYPE1_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_TXTYPE1_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_TXTYPE1_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_TXTYPE1_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_TXTYPE1_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_TXTYPE1_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_TXTYPE1_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_TXTYPE1_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_TXTYPE1_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_TXTYPE1_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_TXTYPE1_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL1
// register.
//
//*****************************************************************************
pub const USB_TXINTERVAL1_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_TXINTERVAL1_TXPOLL_M = usize(0x000000FF);  // TX Polling
pub const USB_TXINTERVAL1_TXPOLL_S = usize(0);
pub const USB_TXINTERVAL1_NAKLMT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE1 register.
//
//*****************************************************************************
pub const USB_RXTYPE1_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_RXTYPE1_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_RXTYPE1_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_RXTYPE1_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_RXTYPE1_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_RXTYPE1_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_RXTYPE1_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_RXTYPE1_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_RXTYPE1_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_RXTYPE1_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_RXTYPE1_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_RXTYPE1_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL1
// register.
//
//*****************************************************************************
pub const USB_RXINTERVAL1_TXPOLL_M = usize(0x000000FF);  // RX Polling
pub const USB_RXINTERVAL1_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_RXINTERVAL1_TXPOLL_S = usize(0);
pub const USB_RXINTERVAL1_NAKLMT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP2 register.
//
//*****************************************************************************
pub const USB_TXMAXP2_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_TXMAXP2_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL2 register.
//
//*****************************************************************************
pub const USB_TXCSRL2_NAKTO = usize(0x00000080);  // NAK Timeout
pub const USB_TXCSRL2_CLRDT = usize(0x00000040);  // Clear Data Toggle
pub const USB_TXCSRL2_STALLED = usize(0x00000020);  // Endpoint Stalled
pub const USB_TXCSRL2_SETUP = usize(0x00000010);  // Setup Packet
pub const USB_TXCSRL2_STALL = usize(0x00000010);  // Send STALL
pub const USB_TXCSRL2_FLUSH = usize(0x00000008);  // Flush FIFO
pub const USB_TXCSRL2_ERROR = usize(0x00000004);  // Error
pub const USB_TXCSRL2_UNDRN = usize(0x00000004);  // Underrun
pub const USB_TXCSRL2_FIFONE = usize(0x00000002);  // FIFO Not Empty
pub const USB_TXCSRL2_TXRDY = usize(0x00000001);  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH2 register.
//
//*****************************************************************************
pub const USB_TXCSRH2_AUTOSET = usize(0x00000080);  // Auto Set
pub const USB_TXCSRH2_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_TXCSRH2_MODE = usize(0x00000020);  // Mode
pub const USB_TXCSRH2_DMAEN = usize(0x00000010);  // DMA Request Enable
pub const USB_TXCSRH2_FDT = usize(0x00000008);  // Force Data Toggle
pub const USB_TXCSRH2_DMAMOD = usize(0x00000004);  // DMA Request Mode
pub const USB_TXCSRH2_DTWE = usize(0x00000002);  // Data Toggle Write Enable
pub const USB_TXCSRH2_DT = usize(0x00000001);  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP2 register.
//
//*****************************************************************************
pub const USB_RXMAXP2_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_RXMAXP2_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL2 register.
//
//*****************************************************************************
pub const USB_RXCSRL2_CLRDT = usize(0x00000080);  // Clear Data Toggle
pub const USB_RXCSRL2_STALLED = usize(0x00000040);  // Endpoint Stalled
pub const USB_RXCSRL2_REQPKT = usize(0x00000020);  // Request Packet
pub const USB_RXCSRL2_STALL = usize(0x00000020);  // Send STALL
pub const USB_RXCSRL2_FLUSH = usize(0x00000010);  // Flush FIFO
pub const USB_RXCSRL2_DATAERR = usize(0x00000008);  // Data Error
pub const USB_RXCSRL2_NAKTO = usize(0x00000008);  // NAK Timeout
pub const USB_RXCSRL2_ERROR = usize(0x00000004);  // Error
pub const USB_RXCSRL2_OVER = usize(0x00000004);  // Overrun
pub const USB_RXCSRL2_FULL = usize(0x00000002);  // FIFO Full
pub const USB_RXCSRL2_RXRDY = usize(0x00000001);  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH2 register.
//
//*****************************************************************************
pub const USB_RXCSRH2_AUTOCL = usize(0x00000080);  // Auto Clear
pub const USB_RXCSRH2_AUTORQ = usize(0x00000040);  // Auto Request
pub const USB_RXCSRH2_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_RXCSRH2_DMAEN = usize(0x00000020);  // DMA Request Enable
pub const USB_RXCSRH2_DISNYET = usize(0x00000010);  // Disable NYET
pub const USB_RXCSRH2_PIDERR = usize(0x00000010);  // PID Error
pub const USB_RXCSRH2_DMAMOD = usize(0x00000008);  // DMA Request Mode
pub const USB_RXCSRH2_DTWE = usize(0x00000004);  // Data Toggle Write Enable
pub const USB_RXCSRH2_DT = usize(0x00000002);  // Data Toggle
pub const USB_RXCSRH2_INCOMPRX = usize(0x00000001);  // Incomplete RX Transmission
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT2 register.
//
//*****************************************************************************
pub const USB_RXCOUNT2_COUNT_M = usize(0x00001FFF);  // Receive Packet Count
pub const USB_RXCOUNT2_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE2 register.
//
//*****************************************************************************
pub const USB_TXTYPE2_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_TXTYPE2_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_TXTYPE2_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_TXTYPE2_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_TXTYPE2_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_TXTYPE2_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_TXTYPE2_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_TXTYPE2_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_TXTYPE2_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_TXTYPE2_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_TXTYPE2_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_TXTYPE2_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL2
// register.
//
//*****************************************************************************
pub const USB_TXINTERVAL2_TXPOLL_M = usize(0x000000FF);  // TX Polling
pub const USB_TXINTERVAL2_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_TXINTERVAL2_NAKLMT_S = usize(0);
pub const USB_TXINTERVAL2_TXPOLL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE2 register.
//
//*****************************************************************************
pub const USB_RXTYPE2_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_RXTYPE2_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_RXTYPE2_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_RXTYPE2_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_RXTYPE2_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_RXTYPE2_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_RXTYPE2_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_RXTYPE2_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_RXTYPE2_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_RXTYPE2_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_RXTYPE2_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_RXTYPE2_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL2
// register.
//
//*****************************************************************************
pub const USB_RXINTERVAL2_TXPOLL_M = usize(0x000000FF);  // RX Polling
pub const USB_RXINTERVAL2_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_RXINTERVAL2_TXPOLL_S = usize(0);
pub const USB_RXINTERVAL2_NAKLMT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP3 register.
//
//*****************************************************************************
pub const USB_TXMAXP3_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_TXMAXP3_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL3 register.
//
//*****************************************************************************
pub const USB_TXCSRL3_NAKTO = usize(0x00000080);  // NAK Timeout
pub const USB_TXCSRL3_CLRDT = usize(0x00000040);  // Clear Data Toggle
pub const USB_TXCSRL3_STALLED = usize(0x00000020);  // Endpoint Stalled
pub const USB_TXCSRL3_SETUP = usize(0x00000010);  // Setup Packet
pub const USB_TXCSRL3_STALL = usize(0x00000010);  // Send STALL
pub const USB_TXCSRL3_FLUSH = usize(0x00000008);  // Flush FIFO
pub const USB_TXCSRL3_ERROR = usize(0x00000004);  // Error
pub const USB_TXCSRL3_UNDRN = usize(0x00000004);  // Underrun
pub const USB_TXCSRL3_FIFONE = usize(0x00000002);  // FIFO Not Empty
pub const USB_TXCSRL3_TXRDY = usize(0x00000001);  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH3 register.
//
//*****************************************************************************
pub const USB_TXCSRH3_AUTOSET = usize(0x00000080);  // Auto Set
pub const USB_TXCSRH3_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_TXCSRH3_MODE = usize(0x00000020);  // Mode
pub const USB_TXCSRH3_DMAEN = usize(0x00000010);  // DMA Request Enable
pub const USB_TXCSRH3_FDT = usize(0x00000008);  // Force Data Toggle
pub const USB_TXCSRH3_DMAMOD = usize(0x00000004);  // DMA Request Mode
pub const USB_TXCSRH3_DTWE = usize(0x00000002);  // Data Toggle Write Enable
pub const USB_TXCSRH3_DT = usize(0x00000001);  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP3 register.
//
//*****************************************************************************
pub const USB_RXMAXP3_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_RXMAXP3_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL3 register.
//
//*****************************************************************************
pub const USB_RXCSRL3_CLRDT = usize(0x00000080);  // Clear Data Toggle
pub const USB_RXCSRL3_STALLED = usize(0x00000040);  // Endpoint Stalled
pub const USB_RXCSRL3_STALL = usize(0x00000020);  // Send STALL
pub const USB_RXCSRL3_REQPKT = usize(0x00000020);  // Request Packet
pub const USB_RXCSRL3_FLUSH = usize(0x00000010);  // Flush FIFO
pub const USB_RXCSRL3_DATAERR = usize(0x00000008);  // Data Error
pub const USB_RXCSRL3_NAKTO = usize(0x00000008);  // NAK Timeout
pub const USB_RXCSRL3_ERROR = usize(0x00000004);  // Error
pub const USB_RXCSRL3_OVER = usize(0x00000004);  // Overrun
pub const USB_RXCSRL3_FULL = usize(0x00000002);  // FIFO Full
pub const USB_RXCSRL3_RXRDY = usize(0x00000001);  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH3 register.
//
//*****************************************************************************
pub const USB_RXCSRH3_AUTOCL = usize(0x00000080);  // Auto Clear
pub const USB_RXCSRH3_AUTORQ = usize(0x00000040);  // Auto Request
pub const USB_RXCSRH3_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_RXCSRH3_DMAEN = usize(0x00000020);  // DMA Request Enable
pub const USB_RXCSRH3_DISNYET = usize(0x00000010);  // Disable NYET
pub const USB_RXCSRH3_PIDERR = usize(0x00000010);  // PID Error
pub const USB_RXCSRH3_DMAMOD = usize(0x00000008);  // DMA Request Mode
pub const USB_RXCSRH3_DTWE = usize(0x00000004);  // Data Toggle Write Enable
pub const USB_RXCSRH3_DT = usize(0x00000002);  // Data Toggle
pub const USB_RXCSRH3_INCOMPRX = usize(0x00000001);  // Incomplete RX Transmission
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT3 register.
//
//*****************************************************************************
pub const USB_RXCOUNT3_COUNT_M = usize(0x00001FFF);  // Receive Packet Count
pub const USB_RXCOUNT3_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE3 register.
//
//*****************************************************************************
pub const USB_TXTYPE3_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_TXTYPE3_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_TXTYPE3_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_TXTYPE3_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_TXTYPE3_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_TXTYPE3_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_TXTYPE3_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_TXTYPE3_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_TXTYPE3_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_TXTYPE3_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_TXTYPE3_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_TXTYPE3_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL3
// register.
//
//*****************************************************************************
pub const USB_TXINTERVAL3_TXPOLL_M = usize(0x000000FF);  // TX Polling
pub const USB_TXINTERVAL3_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_TXINTERVAL3_TXPOLL_S = usize(0);
pub const USB_TXINTERVAL3_NAKLMT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE3 register.
//
//*****************************************************************************
pub const USB_RXTYPE3_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_RXTYPE3_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_RXTYPE3_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_RXTYPE3_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_RXTYPE3_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_RXTYPE3_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_RXTYPE3_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_RXTYPE3_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_RXTYPE3_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_RXTYPE3_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_RXTYPE3_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_RXTYPE3_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL3
// register.
//
//*****************************************************************************
pub const USB_RXINTERVAL3_TXPOLL_M = usize(0x000000FF);  // RX Polling
pub const USB_RXINTERVAL3_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_RXINTERVAL3_TXPOLL_S = usize(0);
pub const USB_RXINTERVAL3_NAKLMT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP4 register.
//
//*****************************************************************************
pub const USB_TXMAXP4_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_TXMAXP4_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL4 register.
//
//*****************************************************************************
pub const USB_TXCSRL4_NAKTO = usize(0x00000080);  // NAK Timeout
pub const USB_TXCSRL4_CLRDT = usize(0x00000040);  // Clear Data Toggle
pub const USB_TXCSRL4_STALLED = usize(0x00000020);  // Endpoint Stalled
pub const USB_TXCSRL4_SETUP = usize(0x00000010);  // Setup Packet
pub const USB_TXCSRL4_STALL = usize(0x00000010);  // Send STALL
pub const USB_TXCSRL4_FLUSH = usize(0x00000008);  // Flush FIFO
pub const USB_TXCSRL4_ERROR = usize(0x00000004);  // Error
pub const USB_TXCSRL4_UNDRN = usize(0x00000004);  // Underrun
pub const USB_TXCSRL4_FIFONE = usize(0x00000002);  // FIFO Not Empty
pub const USB_TXCSRL4_TXRDY = usize(0x00000001);  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH4 register.
//
//*****************************************************************************
pub const USB_TXCSRH4_AUTOSET = usize(0x00000080);  // Auto Set
pub const USB_TXCSRH4_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_TXCSRH4_MODE = usize(0x00000020);  // Mode
pub const USB_TXCSRH4_DMAEN = usize(0x00000010);  // DMA Request Enable
pub const USB_TXCSRH4_FDT = usize(0x00000008);  // Force Data Toggle
pub const USB_TXCSRH4_DMAMOD = usize(0x00000004);  // DMA Request Mode
pub const USB_TXCSRH4_DTWE = usize(0x00000002);  // Data Toggle Write Enable
pub const USB_TXCSRH4_DT = usize(0x00000001);  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP4 register.
//
//*****************************************************************************
pub const USB_RXMAXP4_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_RXMAXP4_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL4 register.
//
//*****************************************************************************
pub const USB_RXCSRL4_CLRDT = usize(0x00000080);  // Clear Data Toggle
pub const USB_RXCSRL4_STALLED = usize(0x00000040);  // Endpoint Stalled
pub const USB_RXCSRL4_STALL = usize(0x00000020);  // Send STALL
pub const USB_RXCSRL4_REQPKT = usize(0x00000020);  // Request Packet
pub const USB_RXCSRL4_FLUSH = usize(0x00000010);  // Flush FIFO
pub const USB_RXCSRL4_NAKTO = usize(0x00000008);  // NAK Timeout
pub const USB_RXCSRL4_DATAERR = usize(0x00000008);  // Data Error
pub const USB_RXCSRL4_OVER = usize(0x00000004);  // Overrun
pub const USB_RXCSRL4_ERROR = usize(0x00000004);  // Error
pub const USB_RXCSRL4_FULL = usize(0x00000002);  // FIFO Full
pub const USB_RXCSRL4_RXRDY = usize(0x00000001);  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH4 register.
//
//*****************************************************************************
pub const USB_RXCSRH4_AUTOCL = usize(0x00000080);  // Auto Clear
pub const USB_RXCSRH4_AUTORQ = usize(0x00000040);  // Auto Request
pub const USB_RXCSRH4_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_RXCSRH4_DMAEN = usize(0x00000020);  // DMA Request Enable
pub const USB_RXCSRH4_DISNYET = usize(0x00000010);  // Disable NYET
pub const USB_RXCSRH4_PIDERR = usize(0x00000010);  // PID Error
pub const USB_RXCSRH4_DMAMOD = usize(0x00000008);  // DMA Request Mode
pub const USB_RXCSRH4_DTWE = usize(0x00000004);  // Data Toggle Write Enable
pub const USB_RXCSRH4_DT = usize(0x00000002);  // Data Toggle
pub const USB_RXCSRH4_INCOMPRX = usize(0x00000001);  // Incomplete RX Transmission
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT4 register.
//
//*****************************************************************************
pub const USB_RXCOUNT4_COUNT_M = usize(0x00001FFF);  // Receive Packet Count
pub const USB_RXCOUNT4_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE4 register.
//
//*****************************************************************************
pub const USB_TXTYPE4_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_TXTYPE4_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_TXTYPE4_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_TXTYPE4_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_TXTYPE4_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_TXTYPE4_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_TXTYPE4_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_TXTYPE4_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_TXTYPE4_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_TXTYPE4_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_TXTYPE4_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_TXTYPE4_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL4
// register.
//
//*****************************************************************************
pub const USB_TXINTERVAL4_TXPOLL_M = usize(0x000000FF);  // TX Polling
pub const USB_TXINTERVAL4_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_TXINTERVAL4_NAKLMT_S = usize(0);
pub const USB_TXINTERVAL4_TXPOLL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE4 register.
//
//*****************************************************************************
pub const USB_RXTYPE4_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_RXTYPE4_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_RXTYPE4_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_RXTYPE4_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_RXTYPE4_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_RXTYPE4_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_RXTYPE4_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_RXTYPE4_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_RXTYPE4_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_RXTYPE4_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_RXTYPE4_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_RXTYPE4_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL4
// register.
//
//*****************************************************************************
pub const USB_RXINTERVAL4_TXPOLL_M = usize(0x000000FF);  // RX Polling
pub const USB_RXINTERVAL4_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_RXINTERVAL4_NAKLMT_S = usize(0);
pub const USB_RXINTERVAL4_TXPOLL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP5 register.
//
//*****************************************************************************
pub const USB_TXMAXP5_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_TXMAXP5_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL5 register.
//
//*****************************************************************************
pub const USB_TXCSRL5_NAKTO = usize(0x00000080);  // NAK Timeout
pub const USB_TXCSRL5_CLRDT = usize(0x00000040);  // Clear Data Toggle
pub const USB_TXCSRL5_STALLED = usize(0x00000020);  // Endpoint Stalled
pub const USB_TXCSRL5_SETUP = usize(0x00000010);  // Setup Packet
pub const USB_TXCSRL5_STALL = usize(0x00000010);  // Send STALL
pub const USB_TXCSRL5_FLUSH = usize(0x00000008);  // Flush FIFO
pub const USB_TXCSRL5_ERROR = usize(0x00000004);  // Error
pub const USB_TXCSRL5_UNDRN = usize(0x00000004);  // Underrun
pub const USB_TXCSRL5_FIFONE = usize(0x00000002);  // FIFO Not Empty
pub const USB_TXCSRL5_TXRDY = usize(0x00000001);  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH5 register.
//
//*****************************************************************************
pub const USB_TXCSRH5_AUTOSET = usize(0x00000080);  // Auto Set
pub const USB_TXCSRH5_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_TXCSRH5_MODE = usize(0x00000020);  // Mode
pub const USB_TXCSRH5_DMAEN = usize(0x00000010);  // DMA Request Enable
pub const USB_TXCSRH5_FDT = usize(0x00000008);  // Force Data Toggle
pub const USB_TXCSRH5_DMAMOD = usize(0x00000004);  // DMA Request Mode
pub const USB_TXCSRH5_DTWE = usize(0x00000002);  // Data Toggle Write Enable
pub const USB_TXCSRH5_DT = usize(0x00000001);  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP5 register.
//
//*****************************************************************************
pub const USB_RXMAXP5_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_RXMAXP5_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL5 register.
//
//*****************************************************************************
pub const USB_RXCSRL5_CLRDT = usize(0x00000080);  // Clear Data Toggle
pub const USB_RXCSRL5_STALLED = usize(0x00000040);  // Endpoint Stalled
pub const USB_RXCSRL5_STALL = usize(0x00000020);  // Send STALL
pub const USB_RXCSRL5_REQPKT = usize(0x00000020);  // Request Packet
pub const USB_RXCSRL5_FLUSH = usize(0x00000010);  // Flush FIFO
pub const USB_RXCSRL5_NAKTO = usize(0x00000008);  // NAK Timeout
pub const USB_RXCSRL5_DATAERR = usize(0x00000008);  // Data Error
pub const USB_RXCSRL5_ERROR = usize(0x00000004);  // Error
pub const USB_RXCSRL5_OVER = usize(0x00000004);  // Overrun
pub const USB_RXCSRL5_FULL = usize(0x00000002);  // FIFO Full
pub const USB_RXCSRL5_RXRDY = usize(0x00000001);  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH5 register.
//
//*****************************************************************************
pub const USB_RXCSRH5_AUTOCL = usize(0x00000080);  // Auto Clear
pub const USB_RXCSRH5_AUTORQ = usize(0x00000040);  // Auto Request
pub const USB_RXCSRH5_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_RXCSRH5_DMAEN = usize(0x00000020);  // DMA Request Enable
pub const USB_RXCSRH5_DISNYET = usize(0x00000010);  // Disable NYET
pub const USB_RXCSRH5_PIDERR = usize(0x00000010);  // PID Error
pub const USB_RXCSRH5_DMAMOD = usize(0x00000008);  // DMA Request Mode
pub const USB_RXCSRH5_DTWE = usize(0x00000004);  // Data Toggle Write Enable
pub const USB_RXCSRH5_DT = usize(0x00000002);  // Data Toggle
pub const USB_RXCSRH5_INCOMPRX = usize(0x00000001);  // Incomplete RX Transmission
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT5 register.
//
//*****************************************************************************
pub const USB_RXCOUNT5_COUNT_M = usize(0x00001FFF);  // Receive Packet Count
pub const USB_RXCOUNT5_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE5 register.
//
//*****************************************************************************
pub const USB_TXTYPE5_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_TXTYPE5_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_TXTYPE5_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_TXTYPE5_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_TXTYPE5_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_TXTYPE5_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_TXTYPE5_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_TXTYPE5_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_TXTYPE5_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_TXTYPE5_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_TXTYPE5_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_TXTYPE5_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL5
// register.
//
//*****************************************************************************
pub const USB_TXINTERVAL5_TXPOLL_M = usize(0x000000FF);  // TX Polling
pub const USB_TXINTERVAL5_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_TXINTERVAL5_NAKLMT_S = usize(0);
pub const USB_TXINTERVAL5_TXPOLL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE5 register.
//
//*****************************************************************************
pub const USB_RXTYPE5_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_RXTYPE5_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_RXTYPE5_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_RXTYPE5_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_RXTYPE5_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_RXTYPE5_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_RXTYPE5_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_RXTYPE5_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_RXTYPE5_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_RXTYPE5_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_RXTYPE5_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_RXTYPE5_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL5
// register.
//
//*****************************************************************************
pub const USB_RXINTERVAL5_TXPOLL_M = usize(0x000000FF);  // RX Polling
pub const USB_RXINTERVAL5_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_RXINTERVAL5_TXPOLL_S = usize(0);
pub const USB_RXINTERVAL5_NAKLMT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP6 register.
//
//*****************************************************************************
pub const USB_TXMAXP6_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_TXMAXP6_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL6 register.
//
//*****************************************************************************
pub const USB_TXCSRL6_NAKTO = usize(0x00000080);  // NAK Timeout
pub const USB_TXCSRL6_CLRDT = usize(0x00000040);  // Clear Data Toggle
pub const USB_TXCSRL6_STALLED = usize(0x00000020);  // Endpoint Stalled
pub const USB_TXCSRL6_STALL = usize(0x00000010);  // Send STALL
pub const USB_TXCSRL6_SETUP = usize(0x00000010);  // Setup Packet
pub const USB_TXCSRL6_FLUSH = usize(0x00000008);  // Flush FIFO
pub const USB_TXCSRL6_ERROR = usize(0x00000004);  // Error
pub const USB_TXCSRL6_UNDRN = usize(0x00000004);  // Underrun
pub const USB_TXCSRL6_FIFONE = usize(0x00000002);  // FIFO Not Empty
pub const USB_TXCSRL6_TXRDY = usize(0x00000001);  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH6 register.
//
//*****************************************************************************
pub const USB_TXCSRH6_AUTOSET = usize(0x00000080);  // Auto Set
pub const USB_TXCSRH6_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_TXCSRH6_MODE = usize(0x00000020);  // Mode
pub const USB_TXCSRH6_DMAEN = usize(0x00000010);  // DMA Request Enable
pub const USB_TXCSRH6_FDT = usize(0x00000008);  // Force Data Toggle
pub const USB_TXCSRH6_DMAMOD = usize(0x00000004);  // DMA Request Mode
pub const USB_TXCSRH6_DTWE = usize(0x00000002);  // Data Toggle Write Enable
pub const USB_TXCSRH6_DT = usize(0x00000001);  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP6 register.
//
//*****************************************************************************
pub const USB_RXMAXP6_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_RXMAXP6_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL6 register.
//
//*****************************************************************************
pub const USB_RXCSRL6_CLRDT = usize(0x00000080);  // Clear Data Toggle
pub const USB_RXCSRL6_STALLED = usize(0x00000040);  // Endpoint Stalled
pub const USB_RXCSRL6_REQPKT = usize(0x00000020);  // Request Packet
pub const USB_RXCSRL6_STALL = usize(0x00000020);  // Send STALL
pub const USB_RXCSRL6_FLUSH = usize(0x00000010);  // Flush FIFO
pub const USB_RXCSRL6_NAKTO = usize(0x00000008);  // NAK Timeout
pub const USB_RXCSRL6_DATAERR = usize(0x00000008);  // Data Error
pub const USB_RXCSRL6_ERROR = usize(0x00000004);  // Error
pub const USB_RXCSRL6_OVER = usize(0x00000004);  // Overrun
pub const USB_RXCSRL6_FULL = usize(0x00000002);  // FIFO Full
pub const USB_RXCSRL6_RXRDY = usize(0x00000001);  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH6 register.
//
//*****************************************************************************
pub const USB_RXCSRH6_AUTOCL = usize(0x00000080);  // Auto Clear
pub const USB_RXCSRH6_AUTORQ = usize(0x00000040);  // Auto Request
pub const USB_RXCSRH6_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_RXCSRH6_DMAEN = usize(0x00000020);  // DMA Request Enable
pub const USB_RXCSRH6_DISNYET = usize(0x00000010);  // Disable NYET
pub const USB_RXCSRH6_PIDERR = usize(0x00000010);  // PID Error
pub const USB_RXCSRH6_DMAMOD = usize(0x00000008);  // DMA Request Mode
pub const USB_RXCSRH6_DTWE = usize(0x00000004);  // Data Toggle Write Enable
pub const USB_RXCSRH6_DT = usize(0x00000002);  // Data Toggle
pub const USB_RXCSRH6_INCOMPRX = usize(0x00000001);  // Incomplete RX Transmission
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT6 register.
//
//*****************************************************************************
pub const USB_RXCOUNT6_COUNT_M = usize(0x00001FFF);  // Receive Packet Count
pub const USB_RXCOUNT6_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE6 register.
//
//*****************************************************************************
pub const USB_TXTYPE6_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_TXTYPE6_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_TXTYPE6_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_TXTYPE6_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_TXTYPE6_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_TXTYPE6_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_TXTYPE6_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_TXTYPE6_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_TXTYPE6_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_TXTYPE6_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_TXTYPE6_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_TXTYPE6_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL6
// register.
//
//*****************************************************************************
pub const USB_TXINTERVAL6_TXPOLL_M = usize(0x000000FF);  // TX Polling
pub const USB_TXINTERVAL6_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_TXINTERVAL6_TXPOLL_S = usize(0);
pub const USB_TXINTERVAL6_NAKLMT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE6 register.
//
//*****************************************************************************
pub const USB_RXTYPE6_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_RXTYPE6_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_RXTYPE6_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_RXTYPE6_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_RXTYPE6_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_RXTYPE6_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_RXTYPE6_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_RXTYPE6_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_RXTYPE6_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_RXTYPE6_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_RXTYPE6_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_RXTYPE6_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL6
// register.
//
//*****************************************************************************
pub const USB_RXINTERVAL6_TXPOLL_M = usize(0x000000FF);  // RX Polling
pub const USB_RXINTERVAL6_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_RXINTERVAL6_NAKLMT_S = usize(0);
pub const USB_RXINTERVAL6_TXPOLL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP7 register.
//
//*****************************************************************************
pub const USB_TXMAXP7_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_TXMAXP7_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL7 register.
//
//*****************************************************************************
pub const USB_TXCSRL7_NAKTO = usize(0x00000080);  // NAK Timeout
pub const USB_TXCSRL7_CLRDT = usize(0x00000040);  // Clear Data Toggle
pub const USB_TXCSRL7_STALLED = usize(0x00000020);  // Endpoint Stalled
pub const USB_TXCSRL7_STALL = usize(0x00000010);  // Send STALL
pub const USB_TXCSRL7_SETUP = usize(0x00000010);  // Setup Packet
pub const USB_TXCSRL7_FLUSH = usize(0x00000008);  // Flush FIFO
pub const USB_TXCSRL7_ERROR = usize(0x00000004);  // Error
pub const USB_TXCSRL7_UNDRN = usize(0x00000004);  // Underrun
pub const USB_TXCSRL7_FIFONE = usize(0x00000002);  // FIFO Not Empty
pub const USB_TXCSRL7_TXRDY = usize(0x00000001);  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH7 register.
//
//*****************************************************************************
pub const USB_TXCSRH7_AUTOSET = usize(0x00000080);  // Auto Set
pub const USB_TXCSRH7_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_TXCSRH7_MODE = usize(0x00000020);  // Mode
pub const USB_TXCSRH7_DMAEN = usize(0x00000010);  // DMA Request Enable
pub const USB_TXCSRH7_FDT = usize(0x00000008);  // Force Data Toggle
pub const USB_TXCSRH7_DMAMOD = usize(0x00000004);  // DMA Request Mode
pub const USB_TXCSRH7_DTWE = usize(0x00000002);  // Data Toggle Write Enable
pub const USB_TXCSRH7_DT = usize(0x00000001);  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP7 register.
//
//*****************************************************************************
pub const USB_RXMAXP7_MAXLOAD_M = usize(0x000007FF);  // Maximum Payload
pub const USB_RXMAXP7_MAXLOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL7 register.
//
//*****************************************************************************
pub const USB_RXCSRL7_CLRDT = usize(0x00000080);  // Clear Data Toggle
pub const USB_RXCSRL7_STALLED = usize(0x00000040);  // Endpoint Stalled
pub const USB_RXCSRL7_REQPKT = usize(0x00000020);  // Request Packet
pub const USB_RXCSRL7_STALL = usize(0x00000020);  // Send STALL
pub const USB_RXCSRL7_FLUSH = usize(0x00000010);  // Flush FIFO
pub const USB_RXCSRL7_DATAERR = usize(0x00000008);  // Data Error
pub const USB_RXCSRL7_NAKTO = usize(0x00000008);  // NAK Timeout
pub const USB_RXCSRL7_ERROR = usize(0x00000004);  // Error
pub const USB_RXCSRL7_OVER = usize(0x00000004);  // Overrun
pub const USB_RXCSRL7_FULL = usize(0x00000002);  // FIFO Full
pub const USB_RXCSRL7_RXRDY = usize(0x00000001);  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH7 register.
//
//*****************************************************************************
pub const USB_RXCSRH7_AUTOCL = usize(0x00000080);  // Auto Clear
pub const USB_RXCSRH7_ISO = usize(0x00000040);  // Isochronous Transfers
pub const USB_RXCSRH7_AUTORQ = usize(0x00000040);  // Auto Request
pub const USB_RXCSRH7_DMAEN = usize(0x00000020);  // DMA Request Enable
pub const USB_RXCSRH7_PIDERR = usize(0x00000010);  // PID Error
pub const USB_RXCSRH7_DISNYET = usize(0x00000010);  // Disable NYET
pub const USB_RXCSRH7_DMAMOD = usize(0x00000008);  // DMA Request Mode
pub const USB_RXCSRH7_DTWE = usize(0x00000004);  // Data Toggle Write Enable
pub const USB_RXCSRH7_DT = usize(0x00000002);  // Data Toggle
pub const USB_RXCSRH7_INCOMPRX = usize(0x00000001);  // Incomplete RX Transmission
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT7 register.
//
//*****************************************************************************
pub const USB_RXCOUNT7_COUNT_M = usize(0x00001FFF);  // Receive Packet Count
pub const USB_RXCOUNT7_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE7 register.
//
//*****************************************************************************
pub const USB_TXTYPE7_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_TXTYPE7_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_TXTYPE7_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_TXTYPE7_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_TXTYPE7_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_TXTYPE7_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_TXTYPE7_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_TXTYPE7_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_TXTYPE7_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_TXTYPE7_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_TXTYPE7_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_TXTYPE7_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL7
// register.
//
//*****************************************************************************
pub const USB_TXINTERVAL7_TXPOLL_M = usize(0x000000FF);  // TX Polling
pub const USB_TXINTERVAL7_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_TXINTERVAL7_NAKLMT_S = usize(0);
pub const USB_TXINTERVAL7_TXPOLL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE7 register.
//
//*****************************************************************************
pub const USB_RXTYPE7_SPEED_M = usize(0x000000C0);  // Operating Speed
pub const USB_RXTYPE7_SPEED_DFLT = usize(0x00000000);  // Default
pub const USB_RXTYPE7_SPEED_HIGH = usize(0x00000040);  // High
pub const USB_RXTYPE7_SPEED_FULL = usize(0x00000080);  // Full
pub const USB_RXTYPE7_SPEED_LOW = usize(0x000000C0);  // Low
pub const USB_RXTYPE7_PROTO_M = usize(0x00000030);  // Protocol
pub const USB_RXTYPE7_PROTO_CTRL = usize(0x00000000);  // Control
pub const USB_RXTYPE7_PROTO_ISOC = usize(0x00000010);  // Isochronous
pub const USB_RXTYPE7_PROTO_BULK = usize(0x00000020);  // Bulk
pub const USB_RXTYPE7_PROTO_INT = usize(0x00000030);  // Interrupt
pub const USB_RXTYPE7_TEP_M = usize(0x0000000F);  // Target Endpoint Number
pub const USB_RXTYPE7_TEP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL7
// register.
//
//*****************************************************************************
pub const USB_RXINTERVAL7_TXPOLL_M = usize(0x000000FF);  // RX Polling
pub const USB_RXINTERVAL7_NAKLMT_M = usize(0x000000FF);  // NAK Limit
pub const USB_RXINTERVAL7_NAKLMT_S = usize(0);
pub const USB_RXINTERVAL7_TXPOLL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAINTR register.
//
//*****************************************************************************
pub const USB_DMAINTR_CH7 = usize(0x00000080);  // Channel 7 DMA Interrupt
pub const USB_DMAINTR_CH6 = usize(0x00000040);  // Channel 6 DMA Interrupt
pub const USB_DMAINTR_CH5 = usize(0x00000020);  // Channel 5 DMA Interrupt
pub const USB_DMAINTR_CH4 = usize(0x00000010);  // Channel 4 DMA Interrupt
pub const USB_DMAINTR_CH3 = usize(0x00000008);  // Channel 3 DMA Interrupt
pub const USB_DMAINTR_CH2 = usize(0x00000004);  // Channel 2 DMA Interrupt
pub const USB_DMAINTR_CH1 = usize(0x00000002);  // Channel 1 DMA Interrupt
pub const USB_DMAINTR_CH0 = usize(0x00000001);  // Channel 0 DMA Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL0 register.
//
//*****************************************************************************
pub const USB_DMACTL0_BRSTM_M = usize(0x00000600);  // Burst Mode
pub const USB_DMACTL0_BRSTM_ANY = usize(0x00000000);  // Bursts of unspecified length
pub const USB_DMACTL0_BRSTM_INC4 = usize(0x00000200);  // INCR4 or unspecified length
pub const USB_DMACTL0_BRSTM_INC8 = usize(0x00000400);  // INCR8, INCR4 or unspecified
                                            // length
pub const USB_DMACTL0_BRSTM_INC16 = usize(0x00000600);  // INCR16, INCR8, INCR4 or
                                            // unspecified length
pub const USB_DMACTL0_ERR = usize(0x00000100);  // Bus Error Bit
pub const USB_DMACTL0_EP_M = usize(0x000000F0);  // Endpoint number
pub const USB_DMACTL0_IE = usize(0x00000008);  // DMA Interrupt Enable
pub const USB_DMACTL0_MODE = usize(0x00000004);  // DMA Transfer Mode
pub const USB_DMACTL0_DIR = usize(0x00000002);  // DMA Direction
pub const USB_DMACTL0_ENABLE = usize(0x00000001);  // DMA Transfer Enable
pub const USB_DMACTL0_EP_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR0 register.
//
//*****************************************************************************
pub const USB_DMAADDR0_ADDR_M = usize(0xFFFFFFFC);  // DMA Address
pub const USB_DMAADDR0_ADDR_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT0
// register.
//
//*****************************************************************************
pub const USB_DMACOUNT0_COUNT_M = usize(0xFFFFFFFC);  // DMA Count
pub const USB_DMACOUNT0_COUNT_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL1 register.
//
//*****************************************************************************
pub const USB_DMACTL1_BRSTM_M = usize(0x00000600);  // Burst Mode
pub const USB_DMACTL1_BRSTM_ANY = usize(0x00000000);  // Bursts of unspecified length
pub const USB_DMACTL1_BRSTM_INC4 = usize(0x00000200);  // INCR4 or unspecified length
pub const USB_DMACTL1_BRSTM_INC8 = usize(0x00000400);  // INCR8, INCR4 or unspecified
                                            // length
pub const USB_DMACTL1_BRSTM_INC16 = usize(0x00000600);  // INCR16, INCR8, INCR4 or
                                            // unspecified length
pub const USB_DMACTL1_ERR = usize(0x00000100);  // Bus Error Bit
pub const USB_DMACTL1_EP_M = usize(0x000000F0);  // Endpoint number
pub const USB_DMACTL1_IE = usize(0x00000008);  // DMA Interrupt Enable
pub const USB_DMACTL1_MODE = usize(0x00000004);  // DMA Transfer Mode
pub const USB_DMACTL1_DIR = usize(0x00000002);  // DMA Direction
pub const USB_DMACTL1_ENABLE = usize(0x00000001);  // DMA Transfer Enable
pub const USB_DMACTL1_EP_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR1 register.
//
//*****************************************************************************
pub const USB_DMAADDR1_ADDR_M = usize(0xFFFFFFFC);  // DMA Address
pub const USB_DMAADDR1_ADDR_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT1
// register.
//
//*****************************************************************************
pub const USB_DMACOUNT1_COUNT_M = usize(0xFFFFFFFC);  // DMA Count
pub const USB_DMACOUNT1_COUNT_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL2 register.
//
//*****************************************************************************
pub const USB_DMACTL2_BRSTM_M = usize(0x00000600);  // Burst Mode
pub const USB_DMACTL2_BRSTM_ANY = usize(0x00000000);  // Bursts of unspecified length
pub const USB_DMACTL2_BRSTM_INC4 = usize(0x00000200);  // INCR4 or unspecified length
pub const USB_DMACTL2_BRSTM_INC8 = usize(0x00000400);  // INCR8, INCR4 or unspecified
                                            // length
pub const USB_DMACTL2_BRSTM_INC16 = usize(0x00000600);  // INCR16, INCR8, INCR4 or
                                            // unspecified length
pub const USB_DMACTL2_ERR = usize(0x00000100);  // Bus Error Bit
pub const USB_DMACTL2_EP_M = usize(0x000000F0);  // Endpoint number
pub const USB_DMACTL2_IE = usize(0x00000008);  // DMA Interrupt Enable
pub const USB_DMACTL2_MODE = usize(0x00000004);  // DMA Transfer Mode
pub const USB_DMACTL2_DIR = usize(0x00000002);  // DMA Direction
pub const USB_DMACTL2_ENABLE = usize(0x00000001);  // DMA Transfer Enable
pub const USB_DMACTL2_EP_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR2 register.
//
//*****************************************************************************
pub const USB_DMAADDR2_ADDR_M = usize(0xFFFFFFFC);  // DMA Address
pub const USB_DMAADDR2_ADDR_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT2
// register.
//
//*****************************************************************************
pub const USB_DMACOUNT2_COUNT_M = usize(0xFFFFFFFC);  // DMA Count
pub const USB_DMACOUNT2_COUNT_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL3 register.
//
//*****************************************************************************
pub const USB_DMACTL3_BRSTM_M = usize(0x00000600);  // Burst Mode
pub const USB_DMACTL3_BRSTM_ANY = usize(0x00000000);  // Bursts of unspecified length
pub const USB_DMACTL3_BRSTM_INC4 = usize(0x00000200);  // INCR4 or unspecified length
pub const USB_DMACTL3_BRSTM_INC8 = usize(0x00000400);  // INCR8, INCR4 or unspecified
                                            // length
pub const USB_DMACTL3_BRSTM_INC16 = usize(0x00000600);  // INCR16, INCR8, INCR4 or
                                            // unspecified length
pub const USB_DMACTL3_ERR = usize(0x00000100);  // Bus Error Bit
pub const USB_DMACTL3_EP_M = usize(0x000000F0);  // Endpoint number
pub const USB_DMACTL3_IE = usize(0x00000008);  // DMA Interrupt Enable
pub const USB_DMACTL3_MODE = usize(0x00000004);  // DMA Transfer Mode
pub const USB_DMACTL3_DIR = usize(0x00000002);  // DMA Direction
pub const USB_DMACTL3_ENABLE = usize(0x00000001);  // DMA Transfer Enable
pub const USB_DMACTL3_EP_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR3 register.
//
//*****************************************************************************
pub const USB_DMAADDR3_ADDR_M = usize(0xFFFFFFFC);  // DMA Address
pub const USB_DMAADDR3_ADDR_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT3
// register.
//
//*****************************************************************************
pub const USB_DMACOUNT3_COUNT_M = usize(0xFFFFFFFC);  // DMA Count
pub const USB_DMACOUNT3_COUNT_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL4 register.
//
//*****************************************************************************
pub const USB_DMACTL4_BRSTM_M = usize(0x00000600);  // Burst Mode
pub const USB_DMACTL4_BRSTM_ANY = usize(0x00000000);  // Bursts of unspecified length
pub const USB_DMACTL4_BRSTM_INC4 = usize(0x00000200);  // INCR4 or unspecified length
pub const USB_DMACTL4_BRSTM_INC8 = usize(0x00000400);  // INCR8, INCR4 or unspecified
                                            // length
pub const USB_DMACTL4_BRSTM_INC16 = usize(0x00000600);  // INCR16, INCR8, INCR4 or
                                            // unspecified length
pub const USB_DMACTL4_ERR = usize(0x00000100);  // Bus Error Bit
pub const USB_DMACTL4_EP_M = usize(0x000000F0);  // Endpoint number
pub const USB_DMACTL4_IE = usize(0x00000008);  // DMA Interrupt Enable
pub const USB_DMACTL4_MODE = usize(0x00000004);  // DMA Transfer Mode
pub const USB_DMACTL4_DIR = usize(0x00000002);  // DMA Direction
pub const USB_DMACTL4_ENABLE = usize(0x00000001);  // DMA Transfer Enable
pub const USB_DMACTL4_EP_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR4 register.
//
//*****************************************************************************
pub const USB_DMAADDR4_ADDR_M = usize(0xFFFFFFFC);  // DMA Address
pub const USB_DMAADDR4_ADDR_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT4
// register.
//
//*****************************************************************************
pub const USB_DMACOUNT4_COUNT_M = usize(0xFFFFFFFC);  // DMA Count
pub const USB_DMACOUNT4_COUNT_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL5 register.
//
//*****************************************************************************
pub const USB_DMACTL5_BRSTM_M = usize(0x00000600);  // Burst Mode
pub const USB_DMACTL5_BRSTM_ANY = usize(0x00000000);  // Bursts of unspecified length
pub const USB_DMACTL5_BRSTM_INC4 = usize(0x00000200);  // INCR4 or unspecified length
pub const USB_DMACTL5_BRSTM_INC8 = usize(0x00000400);  // INCR8, INCR4 or unspecified
                                            // length
pub const USB_DMACTL5_BRSTM_INC16 = usize(0x00000600);  // INCR16, INCR8, INCR4 or
                                            // unspecified length
pub const USB_DMACTL5_ERR = usize(0x00000100);  // Bus Error Bit
pub const USB_DMACTL5_EP_M = usize(0x000000F0);  // Endpoint number
pub const USB_DMACTL5_IE = usize(0x00000008);  // DMA Interrupt Enable
pub const USB_DMACTL5_MODE = usize(0x00000004);  // DMA Transfer Mode
pub const USB_DMACTL5_DIR = usize(0x00000002);  // DMA Direction
pub const USB_DMACTL5_ENABLE = usize(0x00000001);  // DMA Transfer Enable
pub const USB_DMACTL5_EP_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR5 register.
//
//*****************************************************************************
pub const USB_DMAADDR5_ADDR_M = usize(0xFFFFFFFC);  // DMA Address
pub const USB_DMAADDR5_ADDR_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT5
// register.
//
//*****************************************************************************
pub const USB_DMACOUNT5_COUNT_M = usize(0xFFFFFFFC);  // DMA Count
pub const USB_DMACOUNT5_COUNT_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL6 register.
//
//*****************************************************************************
pub const USB_DMACTL6_BRSTM_M = usize(0x00000600);  // Burst Mode
pub const USB_DMACTL6_BRSTM_ANY = usize(0x00000000);  // Bursts of unspecified length
pub const USB_DMACTL6_BRSTM_INC4 = usize(0x00000200);  // INCR4 or unspecified length
pub const USB_DMACTL6_BRSTM_INC8 = usize(0x00000400);  // INCR8, INCR4 or unspecified
                                            // length
pub const USB_DMACTL6_BRSTM_INC16 = usize(0x00000600);  // INCR16, INCR8, INCR4 or
                                            // unspecified length
pub const USB_DMACTL6_ERR = usize(0x00000100);  // Bus Error Bit
pub const USB_DMACTL6_EP_M = usize(0x000000F0);  // Endpoint number
pub const USB_DMACTL6_IE = usize(0x00000008);  // DMA Interrupt Enable
pub const USB_DMACTL6_MODE = usize(0x00000004);  // DMA Transfer Mode
pub const USB_DMACTL6_DIR = usize(0x00000002);  // DMA Direction
pub const USB_DMACTL6_ENABLE = usize(0x00000001);  // DMA Transfer Enable
pub const USB_DMACTL6_EP_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR6 register.
//
//*****************************************************************************
pub const USB_DMAADDR6_ADDR_M = usize(0xFFFFFFFC);  // DMA Address
pub const USB_DMAADDR6_ADDR_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT6
// register.
//
//*****************************************************************************
pub const USB_DMACOUNT6_COUNT_M = usize(0xFFFFFFFC);  // DMA Count
pub const USB_DMACOUNT6_COUNT_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL7 register.
//
//*****************************************************************************
pub const USB_DMACTL7_BRSTM_M = usize(0x00000600);  // Burst Mode
pub const USB_DMACTL7_BRSTM_ANY = usize(0x00000000);  // Bursts of unspecified length
pub const USB_DMACTL7_BRSTM_INC4 = usize(0x00000200);  // INCR4 or unspecified length
pub const USB_DMACTL7_BRSTM_INC8 = usize(0x00000400);  // INCR8, INCR4 or unspecified
                                            // length
pub const USB_DMACTL7_BRSTM_INC16 = usize(0x00000600);  // INCR16, INCR8, INCR4 or
                                            // unspecified length
pub const USB_DMACTL7_ERR = usize(0x00000100);  // Bus Error Bit
pub const USB_DMACTL7_EP_M = usize(0x000000F0);  // Endpoint number
pub const USB_DMACTL7_IE = usize(0x00000008);  // DMA Interrupt Enable
pub const USB_DMACTL7_MODE = usize(0x00000004);  // DMA Transfer Mode
pub const USB_DMACTL7_DIR = usize(0x00000002);  // DMA Direction
pub const USB_DMACTL7_ENABLE = usize(0x00000001);  // DMA Transfer Enable
pub const USB_DMACTL7_EP_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR7 register.
//
//*****************************************************************************
pub const USB_DMAADDR7_ADDR_M = usize(0xFFFFFFFC);  // DMA Address
pub const USB_DMAADDR7_ADDR_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT7
// register.
//
//*****************************************************************************
pub const USB_DMACOUNT7_COUNT_M = usize(0xFFFFFFFC);  // DMA Count
pub const USB_DMACOUNT7_COUNT_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT1
// register.
//
//*****************************************************************************
pub const USB_RQPKTCOUNT1_M = usize(0x0000FFFF);  // Block Transfer Packet Count
pub const USB_RQPKTCOUNT1_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT2
// register.
//
//*****************************************************************************
pub const USB_RQPKTCOUNT2_M = usize(0x0000FFFF);  // Block Transfer Packet Count
pub const USB_RQPKTCOUNT2_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT3
// register.
//
//*****************************************************************************
pub const USB_RQPKTCOUNT3_M = usize(0x0000FFFF);  // Block Transfer Packet Count
pub const USB_RQPKTCOUNT3_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT4
// register.
//
//*****************************************************************************
pub const USB_RQPKTCOUNT4_COUNT_M = usize(0x0000FFFF);  // Block Transfer Packet Count
pub const USB_RQPKTCOUNT4_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT5
// register.
//
//*****************************************************************************
pub const USB_RQPKTCOUNT5_COUNT_M = usize(0x0000FFFF);  // Block Transfer Packet Count
pub const USB_RQPKTCOUNT5_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT6
// register.
//
//*****************************************************************************
pub const USB_RQPKTCOUNT6_COUNT_M = usize(0x0000FFFF);  // Block Transfer Packet Count
pub const USB_RQPKTCOUNT6_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT7
// register.
//
//*****************************************************************************
pub const USB_RQPKTCOUNT7_COUNT_M = usize(0x0000FFFF);  // Block Transfer Packet Count
pub const USB_RQPKTCOUNT7_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXDPKTBUFDIS
// register.
//
//*****************************************************************************
pub const USB_RXDPKTBUFDIS_EP7 = usize(0x00000080);  // EP7 RX Double-Packet Buffer
                                            // Disable
pub const USB_RXDPKTBUFDIS_EP6 = usize(0x00000040);  // EP6 RX Double-Packet Buffer
                                            // Disable
pub const USB_RXDPKTBUFDIS_EP5 = usize(0x00000020);  // EP5 RX Double-Packet Buffer
                                            // Disable
pub const USB_RXDPKTBUFDIS_EP4 = usize(0x00000010);  // EP4 RX Double-Packet Buffer
                                            // Disable
pub const USB_RXDPKTBUFDIS_EP3 = usize(0x00000008);  // EP3 RX Double-Packet Buffer
                                            // Disable
pub const USB_RXDPKTBUFDIS_EP2 = usize(0x00000004);  // EP2 RX Double-Packet Buffer
                                            // Disable
pub const USB_RXDPKTBUFDIS_EP1 = usize(0x00000002);  // EP1 RX Double-Packet Buffer
                                            // Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXDPKTBUFDIS
// register.
//
//*****************************************************************************
pub const USB_TXDPKTBUFDIS_EP7 = usize(0x00000080);  // EP7 TX Double-Packet Buffer
                                            // Disable
pub const USB_TXDPKTBUFDIS_EP6 = usize(0x00000040);  // EP6 TX Double-Packet Buffer
                                            // Disable
pub const USB_TXDPKTBUFDIS_EP5 = usize(0x00000020);  // EP5 TX Double-Packet Buffer
                                            // Disable
pub const USB_TXDPKTBUFDIS_EP4 = usize(0x00000010);  // EP4 TX Double-Packet Buffer
                                            // Disable
pub const USB_TXDPKTBUFDIS_EP3 = usize(0x00000008);  // EP3 TX Double-Packet Buffer
                                            // Disable
pub const USB_TXDPKTBUFDIS_EP2 = usize(0x00000004);  // EP2 TX Double-Packet Buffer
                                            // Disable
pub const USB_TXDPKTBUFDIS_EP1 = usize(0x00000002);  // EP1 TX Double-Packet Buffer
                                            // Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CTO register.
//
//*****************************************************************************
pub const USB_CTO_CCTV_M = usize(0x0000FFFF);  // Configurable Chirp Timeout Value
pub const USB_CTO_CCTV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_HHSRTN register.
//
//*****************************************************************************
pub const USB_HHSRTN_HHSRTN_M = usize(0x0000FFFF);  // HIgh Speed to UTM Operating
                                            // Delay
pub const USB_HHSRTN_HHSRTN_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_HSBT register.
//
//*****************************************************************************
pub const USB_HSBT_HSBT_M = usize(0x0000000F);  // High Speed Timeout Adder
pub const USB_HSBT_HSBT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LPMATTR register.
//
//*****************************************************************************
pub const USB_LPMATTR_ENDPT_M = usize(0x0000F000);  // Endpoint
pub const USB_LPMATTR_RMTWAK = usize(0x00000100);  // Remote Wake
pub const USB_LPMATTR_HIRD_M = usize(0x000000F0);  // Host Initiated Resume Duration
pub const USB_LPMATTR_LS_M = usize(0x0000000F);  // Link State
pub const USB_LPMATTR_LS_L1 = usize(0x00000001);  // Sleep State (L1)
pub const USB_LPMATTR_ENDPT_S = usize(12);
pub const USB_LPMATTR_HIRD_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LPMCNTRL register.
//
//*****************************************************************************
pub const USB_LPMCNTRL_NAK = usize(0x00000010);  // LPM NAK
pub const USB_LPMCNTRL_EN_M = usize(0x0000000C);  // LPM Enable
pub const USB_LPMCNTRL_EN_NONE = usize(0x00000000);  // LPM and Extended transactions
                                            // are not supported. In this case,
                                            // the USB does not respond to LPM
                                            // transactions and LPM
                                            // transactions cause a timeout
pub const USB_LPMCNTRL_EN_EXT = usize(0x00000004);  // LPM is not supported but
                                            // extended transactions are
                                            // supported. In this case, the USB
                                            // does respond to an LPM
                                            // transaction with a STALL
pub const USB_LPMCNTRL_EN_LPMEXT = usize(0x0000000C);  // The USB supports LPM extended
                                            // transactions. In this case, the
                                            // USB responds with a NYET or an
                                            // ACK as determined by the value
                                            // of TXLPM and other conditions
pub const USB_LPMCNTRL_RES = usize(0x00000002);  // LPM Resume
pub const USB_LPMCNTRL_TXLPM = usize(0x00000001);  // Transmit LPM Transaction Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LPMIM register.
//
//*****************************************************************************
pub const USB_LPMIM_ERR = usize(0x00000020);  // LPM Error Interrupt Mask
pub const USB_LPMIM_RES = usize(0x00000010);  // LPM Resume Interrupt Mask
pub const USB_LPMIM_NC = usize(0x00000008);  // LPM NC Interrupt Mask
pub const USB_LPMIM_ACK = usize(0x00000004);  // LPM ACK Interrupt Mask
pub const USB_LPMIM_NY = usize(0x00000002);  // LPM NY Interrupt Mask
pub const USB_LPMIM_STALL = usize(0x00000001);  // LPM STALL Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LPMRIS register.
//
//*****************************************************************************
pub const USB_LPMRIS_ERR = usize(0x00000020);  // LPM Interrupt Status
pub const USB_LPMRIS_RES = usize(0x00000010);  // LPM Resume Interrupt Status
pub const USB_LPMRIS_NC = usize(0x00000008);  // LPM NC Interrupt Status
pub const USB_LPMRIS_ACK = usize(0x00000004);  // LPM ACK Interrupt Status
pub const USB_LPMRIS_NY = usize(0x00000002);  // LPM NY Interrupt Status
pub const USB_LPMRIS_LPMST = usize(0x00000001);  // LPM STALL Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LPMFADDR register.
//
//*****************************************************************************
pub const USB_LPMFADDR_ADDR_M = usize(0x0000007F);  // LPM Function Address
pub const USB_LPMFADDR_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPC register.
//
//*****************************************************************************
pub const USB_EPC_PFLTACT_M = usize(0x00000300);  // Power Fault Action
pub const USB_EPC_PFLTACT_UNCHG = usize(0x00000000);  // Unchanged
pub const USB_EPC_PFLTACT_TRIS = usize(0x00000100);  // Tristate
pub const USB_EPC_PFLTACT_LOW = usize(0x00000200);  // Low
pub const USB_EPC_PFLTACT_HIGH = usize(0x00000300);  // High
pub const USB_EPC_PFLTAEN = usize(0x00000040);  // Power Fault Action Enable
pub const USB_EPC_PFLTSEN_HIGH = usize(0x00000020);  // Power Fault Sense
pub const USB_EPC_PFLTEN = usize(0x00000010);  // Power Fault Input Enable
pub const USB_EPC_EPENDE = usize(0x00000004);  // EPEN Drive Enable
pub const USB_EPC_EPEN_M = usize(0x00000003);  // External Power Supply Enable
                                            // Configuration
pub const USB_EPC_EPEN_LOW = usize(0x00000000);  // Power Enable Active Low
pub const USB_EPC_EPEN_HIGH = usize(0x00000001);  // Power Enable Active High
pub const USB_EPC_EPEN_VBLOW = usize(0x00000002);  // Power Enable High if VBUS Low
                                            // (OTG only)
pub const USB_EPC_EPEN_VBHIGH = usize(0x00000003);  // Power Enable High if VBUS High
                                            // (OTG only)

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPCRIS register.
//
//*****************************************************************************
pub const USB_EPCRIS_PF = usize(0x00000001);  // USB Power Fault Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPCIM register.
//
//*****************************************************************************
pub const USB_EPCIM_PF = usize(0x00000001);  // USB Power Fault Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPCISC register.
//
//*****************************************************************************
pub const USB_EPCISC_PF = usize(0x00000001);  // USB Power Fault Interrupt Status
                                            // and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRRIS register.
//
//*****************************************************************************
pub const USB_DRRIS_RESUME = usize(0x00000001);  // RESUME Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRIM register.
//
//*****************************************************************************
pub const USB_DRIM_RESUME = usize(0x00000001);  // RESUME Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRISC register.
//
//*****************************************************************************
pub const USB_DRISC_RESUME = usize(0x00000001);  // RESUME Interrupt Status and
                                            // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_GPCS register.
//
//*****************************************************************************
pub const USB_GPCS_DEVMOD_M = usize(0x00000007);  // Device Mode
pub const USB_GPCS_DEVMOD_OTG = usize(0x00000000);  // Use USB0VBUS and USB0ID pin
pub const USB_GPCS_DEVMOD_HOST = usize(0x00000002);  // Force USB0VBUS and USB0ID low
pub const USB_GPCS_DEVMOD_DEV = usize(0x00000003);  // Force USB0VBUS and USB0ID high
pub const USB_GPCS_DEVMOD_HOSTVBUS = usize(0x00000004);  // Use USB0VBUS and force USB0ID
                                            // low
pub const USB_GPCS_DEVMOD_DEVVBUS = usize(0x00000005);  // Use USB0VBUS and force USB0ID
                                            // high

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDC register.
//
//*****************************************************************************
pub const USB_VDC_VBDEN = usize(0x00000001);  // VBUS Droop Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDCRIS register.
//
//*****************************************************************************
pub const USB_VDCRIS_VD = usize(0x00000001);  // VBUS Droop Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDCIM register.
//
//*****************************************************************************
pub const USB_VDCIM_VD = usize(0x00000001);  // VBUS Droop Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDCISC register.
//
//*****************************************************************************
pub const USB_VDCISC_VD = usize(0x00000001);  // VBUS Droop Interrupt Status and
                                            // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_PP register.
//
//*****************************************************************************
pub const USB_PP_ECNT_M = usize(0x0000FF00);  // Endpoint Count
pub const USB_PP_USB_M = usize(0x000000C0);  // USB Capability
pub const USB_PP_USB_DEVICE = usize(0x00000040);  // DEVICE
pub const USB_PP_USB_HOSTDEVICE = usize(0x00000080);  // HOST
pub const USB_PP_USB_OTG = usize(0x000000C0);  // OTG
pub const USB_PP_ULPI = usize(0x00000020);  // ULPI Present
pub const USB_PP_PHY = usize(0x00000010);  // PHY Present
pub const USB_PP_TYPE_M = usize(0x0000000F);  // Controller Type
pub const USB_PP_TYPE_0 = usize(0x00000000);  // The first-generation USB
                                            // controller
pub const USB_PP_TYPE_1 = usize(0x00000001);  // Second-generation USB
                                            // controller.The controller
                                            // implemented in post Icestorm
                                            // devices that use the 3.0 version
                                            // of the Mentor controller
pub const USB_PP_ECNT_S = usize(8);

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_PC register.
//
//*****************************************************************************
pub const USB_PC_ULPIEN = usize(0x00010000);  // ULPI Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CC register.
//
//*****************************************************************************
pub const USB_CC_CLKEN = usize(0x00000200);  // USB Clock Enable
pub const USB_CC_CSD = usize(0x00000100);  // Clock Source/Direction
pub const USB_CC_CLKDIV_M = usize(0x0000000F);  // PLL Clock Divisor
pub const USB_CC_CLKDIV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EESIZE register.
//
//*****************************************************************************
pub const EEPROM_EESIZE_BLKCNT_M = usize(0x07FF0000);  // Number of 16-Word Blocks
pub const EEPROM_EESIZE_WORDCNT_M = usize(0x0000FFFF);  // Number of 32-Bit Words
pub const EEPROM_EESIZE_BLKCNT_S = usize(16);
pub const EEPROM_EESIZE_WORDCNT_S = usize(0);

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
pub const EEPROM_EEDONE_WRBUSY = usize(0x00000020);  // Write Busy
pub const EEPROM_EEDONE_NOPERM = usize(0x00000010);  // Write Without Permission
pub const EEPROM_EEDONE_WKCOPY = usize(0x00000008);  // Working on a Copy
pub const EEPROM_EEDONE_WKERASE = usize(0x00000004);  // Working on an Erase
pub const EEPROM_EEDONE_WORKING = usize(0x00000001);  // EEPROM Working

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EESUPP register.
//
//*****************************************************************************
pub const EEPROM_EESUPP_PRETRY = usize(0x00000008);  // Programming Must Be Retried
pub const EEPROM_EESUPP_ERETRY = usize(0x00000004);  // Erase Must Be Retried

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
pub const EEPROM_EEPROT_ACC = usize(0x00000008);  // Access Control
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
pub const EEPROM_EEDBGME_KEY_M = usize(0xFFFF0000);  // Erase Key
pub const EEPROM_EEDBGME_ME = usize(0x00000001);  // Mass Erase
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
// The following are defines for the bit fields in the SYSEXC_RIS register.
//
//*****************************************************************************
pub const SYSEXC_RIS_FPIXCRIS = usize(0x00000020);  // Floating-Point Inexact Exception
                                            // Raw Interrupt Status
pub const SYSEXC_RIS_FPOFCRIS = usize(0x00000010);  // Floating-Point Overflow
                                            // Exception Raw Interrupt Status
pub const SYSEXC_RIS_FPUFCRIS = usize(0x00000008);  // Floating-Point Underflow
                                            // Exception Raw Interrupt Status
pub const SYSEXC_RIS_FPIOCRIS = usize(0x00000004);  // Floating-Point Invalid Operation
                                            // Raw Interrupt Status
pub const SYSEXC_RIS_FPDZCRIS = usize(0x00000002);  // Floating-Point Divide By 0
                                            // Exception Raw Interrupt Status
pub const SYSEXC_RIS_FPIDCRIS = usize(0x00000001);  // Floating-Point Input Denormal
                                            // Exception Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_IM register.
//
//*****************************************************************************
pub const SYSEXC_IM_FPIXCIM = usize(0x00000020);  // Floating-Point Inexact Exception
                                            // Interrupt Mask
pub const SYSEXC_IM_FPOFCIM = usize(0x00000010);  // Floating-Point Overflow
                                            // Exception Interrupt Mask
pub const SYSEXC_IM_FPUFCIM = usize(0x00000008);  // Floating-Point Underflow
                                            // Exception Interrupt Mask
pub const SYSEXC_IM_FPIOCIM = usize(0x00000004);  // Floating-Point Invalid Operation
                                            // Interrupt Mask
pub const SYSEXC_IM_FPDZCIM = usize(0x00000002);  // Floating-Point Divide By 0
                                            // Exception Interrupt Mask
pub const SYSEXC_IM_FPIDCIM = usize(0x00000001);  // Floating-Point Input Denormal
                                            // Exception Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_MIS register.
//
//*****************************************************************************
pub const SYSEXC_MIS_FPIXCMIS = usize(0x00000020);  // Floating-Point Inexact Exception
                                            // Masked Interrupt Status
pub const SYSEXC_MIS_FPOFCMIS = usize(0x00000010);  // Floating-Point Overflow
                                            // Exception Masked Interrupt
                                            // Status
pub const SYSEXC_MIS_FPUFCMIS = usize(0x00000008);  // Floating-Point Underflow
                                            // Exception Masked Interrupt
                                            // Status
pub const SYSEXC_MIS_FPIOCMIS = usize(0x00000004);  // Floating-Point Invalid Operation
                                            // Masked Interrupt Status
pub const SYSEXC_MIS_FPDZCMIS = usize(0x00000002);  // Floating-Point Divide By 0
                                            // Exception Masked Interrupt
                                            // Status
pub const SYSEXC_MIS_FPIDCMIS = usize(0x00000001);  // Floating-Point Input Denormal
                                            // Exception Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_IC register.
//
//*****************************************************************************
pub const SYSEXC_IC_FPIXCIC = usize(0x00000020);  // Floating-Point Inexact Exception
                                            // Interrupt Clear
pub const SYSEXC_IC_FPOFCIC = usize(0x00000010);  // Floating-Point Overflow
                                            // Exception Interrupt Clear
pub const SYSEXC_IC_FPUFCIC = usize(0x00000008);  // Floating-Point Underflow
                                            // Exception Interrupt Clear
pub const SYSEXC_IC_FPIOCIC = usize(0x00000004);  // Floating-Point Invalid Operation
                                            // Interrupt Clear
pub const SYSEXC_IC_FPDZCIC = usize(0x00000002);  // Floating-Point Divide By 0
                                            // Exception Interrupt Clear
pub const SYSEXC_IC_FPIDCIC = usize(0x00000001);  // Floating-Point Input Denormal
                                            // Exception Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCC register.
//
//*****************************************************************************
pub const HIB_RTCC_M = usize(0xFFFFFFFF);  // RTC Counter
pub const HIB_RTCC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCM0 register.
//
//*****************************************************************************
pub const HIB_RTCM0_M = usize(0xFFFFFFFF);  // RTC Match 0
pub const HIB_RTCM0_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCLD register.
//
//*****************************************************************************
pub const HIB_RTCLD_M = usize(0xFFFFFFFF);  // RTC Load
pub const HIB_RTCLD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CTL register.
//
//*****************************************************************************
pub const HIB_CTL_WRC = usize(0x80000000);  // Write Complete/Capable
pub const HIB_CTL_RETCLR = usize(0x40000000);  // GPIO Retention/Clear
pub const HIB_CTL_OSCSEL = usize(0x00080000);  // Oscillator Select
pub const HIB_CTL_OSCDRV = usize(0x00020000);  // Oscillator Drive Capability
pub const HIB_CTL_OSCBYP = usize(0x00010000);  // Oscillator Bypass
pub const HIB_CTL_VBATSEL_M = usize(0x00006000);  // Select for Low-Battery
                                            // Comparator
pub const HIB_CTL_VBATSEL_1_9V = usize(0x00000000);  // 1.9 Volts
pub const HIB_CTL_VBATSEL_2_1V = usize(0x00002000);  // 2.1 Volts (default)
pub const HIB_CTL_VBATSEL_2_3V = usize(0x00004000);  // 2.3 Volts
pub const HIB_CTL_VBATSEL_2_5V = usize(0x00006000);  // 2.5 Volts
pub const HIB_CTL_BATCHK = usize(0x00000400);  // Check Battery Status
pub const HIB_CTL_BATWKEN = usize(0x00000200);  // Wake on Low Battery
pub const HIB_CTL_VDD3ON = usize(0x00000100);  // VDD Powered
pub const HIB_CTL_VABORT = usize(0x00000080);  // Power Cut Abort Enable
pub const HIB_CTL_CLK32EN = usize(0x00000040);  // Clocking Enable
pub const HIB_CTL_PINWEN = usize(0x00000010);  // External Wake Pin Enable
pub const HIB_CTL_RTCWEN = usize(0x00000008);  // RTC Wake-up Enable
pub const HIB_CTL_HIBREQ = usize(0x00000002);  // Hibernation Request
pub const HIB_CTL_RTCEN = usize(0x00000001);  // RTC Timer Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_IM register.
//
//*****************************************************************************
pub const HIB_IM_VDDFAIL = usize(0x00000080);  // VDD Fail Interrupt Mask
pub const HIB_IM_RSTWK = usize(0x00000040);  // Reset Pad I/O Wake-Up Interrupt
                                            // Mask
pub const HIB_IM_PADIOWK = usize(0x00000020);  // Pad I/O Wake-Up Interrupt Mask
pub const HIB_IM_WC = usize(0x00000010);  // External Write Complete/Capable
                                            // Interrupt Mask
pub const HIB_IM_EXTW = usize(0x00000008);  // External Wake-Up Interrupt Mask
pub const HIB_IM_LOWBAT = usize(0x00000004);  // Low Battery Voltage Interrupt
                                            // Mask
pub const HIB_IM_RTCALT0 = usize(0x00000001);  // RTC Alert 0 Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RIS register.
//
//*****************************************************************************
pub const HIB_RIS_VDDFAIL = usize(0x00000080);  // VDD Fail Raw Interrupt Status
pub const HIB_RIS_RSTWK = usize(0x00000040);  // Reset Pad I/O Wake-Up Raw
                                            // Interrupt Status
pub const HIB_RIS_PADIOWK = usize(0x00000020);  // Pad I/O Wake-Up Raw Interrupt
                                            // Status
pub const HIB_RIS_WC = usize(0x00000010);  // Write Complete/Capable Raw
                                            // Interrupt Status
pub const HIB_RIS_EXTW = usize(0x00000008);  // External Wake-Up Raw Interrupt
                                            // Status
pub const HIB_RIS_LOWBAT = usize(0x00000004);  // Low Battery Voltage Raw
                                            // Interrupt Status
pub const HIB_RIS_RTCALT0 = usize(0x00000001);  // RTC Alert 0 Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_MIS register.
//
//*****************************************************************************
pub const HIB_MIS_VDDFAIL = usize(0x00000080);  // VDD Fail Interrupt Mask
pub const HIB_MIS_RSTWK = usize(0x00000040);  // Reset Pad I/O Wake-Up Interrupt
                                            // Mask
pub const HIB_MIS_PADIOWK = usize(0x00000020);  // Pad I/O Wake-Up Interrupt Mask
pub const HIB_MIS_WC = usize(0x00000010);  // Write Complete/Capable Masked
                                            // Interrupt Status
pub const HIB_MIS_EXTW = usize(0x00000008);  // External Wake-Up Masked
                                            // Interrupt Status
pub const HIB_MIS_LOWBAT = usize(0x00000004);  // Low Battery Voltage Masked
                                            // Interrupt Status
pub const HIB_MIS_RTCALT0 = usize(0x00000001);  // RTC Alert 0 Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_IC register.
//
//*****************************************************************************
pub const HIB_IC_VDDFAIL = usize(0x00000080);  // VDD Fail Interrupt Clear
pub const HIB_IC_RSTWK = usize(0x00000040);  // Reset Pad I/O Wake-Up Interrupt
                                            // Clear
pub const HIB_IC_PADIOWK = usize(0x00000020);  // Pad I/O Wake-Up Interrupt Clear
pub const HIB_IC_WC = usize(0x00000010);  // Write Complete/Capable Interrupt
                                            // Clear
pub const HIB_IC_EXTW = usize(0x00000008);  // External Wake-Up Interrupt Clear
pub const HIB_IC_LOWBAT = usize(0x00000004);  // Low Battery Voltage Interrupt
                                            // Clear
pub const HIB_IC_RTCALT0 = usize(0x00000001);  // RTC Alert0 Masked Interrupt
                                            // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCT register.
//
//*****************************************************************************
pub const HIB_RTCT_TRIM_M = usize(0x0000FFFF);  // RTC Trim Value
pub const HIB_RTCT_TRIM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCSS register.
//
//*****************************************************************************
pub const HIB_RTCSS_RTCSSM_M = usize(0x7FFF0000);  // RTC Sub Seconds Match
pub const HIB_RTCSS_RTCSSC_M = usize(0x00007FFF);  // RTC Sub Seconds Count
pub const HIB_RTCSS_RTCSSM_S = usize(16);
pub const HIB_RTCSS_RTCSSC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_IO register.
//
//*****************************************************************************
pub const HIB_IO_IOWRC = usize(0x80000000);  // I/O Write Complete
pub const HIB_IO_WURSTEN = usize(0x00000010);  // Reset Wake Source Enable
pub const HIB_IO_WUUNLK = usize(0x00000001);  // I/O Wake Pad Configuration
                                            // Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_DATA register.
//
//*****************************************************************************
pub const HIB_DATA_RTD_M = usize(0xFFFFFFFF);  // Hibernation Module NV Data
pub const HIB_DATA_RTD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALCTL register.
//
//*****************************************************************************
pub const HIB_CALCTL_CAL24 = usize(0x00000004);  // Calendar Mode
pub const HIB_CALCTL_CALEN = usize(0x00000001);  // RTC Calendar/Counter Mode Select

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CAL0 register.
//
//*****************************************************************************
pub const HIB_CAL0_VALID = usize(0x80000000);  // Valid Calendar Load
pub const HIB_CAL0_AMPM = usize(0x00400000);  // AM/PM Designation
pub const HIB_CAL0_HR_M = usize(0x001F0000);  // Hours
pub const HIB_CAL0_MIN_M = usize(0x00003F00);  // Minutes
pub const HIB_CAL0_SEC_M = usize(0x0000003F);  // Seconds
pub const HIB_CAL0_HR_S = usize(16);
pub const HIB_CAL0_MIN_S = usize(8);
pub const HIB_CAL0_SEC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CAL1 register.
//
//*****************************************************************************
pub const HIB_CAL1_VALID = usize(0x80000000);  // Valid Calendar Load
pub const HIB_CAL1_DOW_M = usize(0x07000000);  // Day of Week
pub const HIB_CAL1_YEAR_M = usize(0x007F0000);  // Year Value
pub const HIB_CAL1_MON_M = usize(0x00000F00);  // Month
pub const HIB_CAL1_DOM_M = usize(0x0000001F);  // Day of Month
pub const HIB_CAL1_DOW_S = usize(24);
pub const HIB_CAL1_YEAR_S = usize(16);
pub const HIB_CAL1_MON_S = usize(8);
pub const HIB_CAL1_DOM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALLD0 register.
//
//*****************************************************************************
pub const HIB_CALLD0_AMPM = usize(0x00400000);  // AM/PM Designation
pub const HIB_CALLD0_HR_M = usize(0x001F0000);  // Hours
pub const HIB_CALLD0_MIN_M = usize(0x00003F00);  // Minutes
pub const HIB_CALLD0_SEC_M = usize(0x0000003F);  // Seconds
pub const HIB_CALLD0_HR_S = usize(16);
pub const HIB_CALLD0_MIN_S = usize(8);
pub const HIB_CALLD0_SEC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALLD1 register.
//
//*****************************************************************************
pub const HIB_CALLD1_DOW_M = usize(0x07000000);  // Day of Week
pub const HIB_CALLD1_YEAR_M = usize(0x007F0000);  // Year Value
pub const HIB_CALLD1_MON_M = usize(0x00000F00);  // Month
pub const HIB_CALLD1_DOM_M = usize(0x0000001F);  // Day of Month
pub const HIB_CALLD1_DOW_S = usize(24);
pub const HIB_CALLD1_YEAR_S = usize(16);
pub const HIB_CALLD1_MON_S = usize(8);
pub const HIB_CALLD1_DOM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALM0 register.
//
//*****************************************************************************
pub const HIB_CALM0_AMPM = usize(0x00400000);  // AM/PM Designation
pub const HIB_CALM0_HR_M = usize(0x001F0000);  // Hours
pub const HIB_CALM0_MIN_M = usize(0x00003F00);  // Minutes
pub const HIB_CALM0_SEC_M = usize(0x0000003F);  // Seconds
pub const HIB_CALM0_HR_S = usize(16);
pub const HIB_CALM0_MIN_S = usize(8);
pub const HIB_CALM0_SEC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CALM1 register.
//
//*****************************************************************************
pub const HIB_CALM1_DOM_M = usize(0x0000001F);  // Day of Month
pub const HIB_CALM1_DOM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_LOCK register.
//
//*****************************************************************************
pub const HIB_LOCK_HIBLOCK_M = usize(0xFFFFFFFF);  // HIbernate Lock
pub const HIB_LOCK_HIBLOCK_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPCTL register.
//
//*****************************************************************************
pub const HIB_TPCTL_WAKE = usize(0x00000800);  // Wake from Hibernate on a Tamper
                                            // Event
pub const HIB_TPCTL_MEMCLR_M = usize(0x00000300);  // HIB Memory Clear on Tamper Event
pub const HIB_TPCTL_MEMCLR_NONE = usize(0x00000000);  // Do not Clear HIB memory on
                                            // tamper event
pub const HIB_TPCTL_MEMCLR_LOW32 = usize(0x00000100);  // Clear Lower 32 Bytes of HIB
                                            // memory on tamper event
pub const HIB_TPCTL_MEMCLR_HIGH32 = usize(0x00000200);  // Clear upper 32 Bytes of HIB
                                            // memory on tamper event
pub const HIB_TPCTL_MEMCLR_ALL = usize(0x00000300);  // Clear all HIB memory on tamper
                                            // event
pub const HIB_TPCTL_TPCLR = usize(0x00000010);  // Tamper Event Clear
pub const HIB_TPCTL_TPEN = usize(0x00000001);  // Tamper Module Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPSTAT register.
//
//*****************************************************************************
pub const HIB_TPSTAT_STATE_M = usize(0x0000000C);  // Tamper Module Status
pub const HIB_TPSTAT_STATE_DISABLED = usize(0x00000000);  // Tamper disabled
pub const HIB_TPSTAT_STATE_CONFIGED = usize(0x00000004);  // Tamper configured
pub const HIB_TPSTAT_STATE_ERROR = usize(0x00000008);  // Tamper pin event occurred
pub const HIB_TPSTAT_XOSCST = usize(0x00000002);  // External Oscillator Status
pub const HIB_TPSTAT_XOSCFAIL = usize(0x00000001);  // External Oscillator Failure

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPIO register.
//
//*****************************************************************************
pub const HIB_TPIO_GFLTR3 = usize(0x08000000);  // TMPR3 Glitch Filtering
pub const HIB_TPIO_PUEN3 = usize(0x04000000);  // TMPR3 Internal Weak Pull-up
                                            // Enable
pub const HIB_TPIO_LEV3 = usize(0x02000000);  // TMPR3 Trigger Level
pub const HIB_TPIO_EN3 = usize(0x01000000);  // TMPR3 Enable
pub const HIB_TPIO_GFLTR2 = usize(0x00080000);  // TMPR2 Glitch Filtering
pub const HIB_TPIO_PUEN2 = usize(0x00040000);  // TMPR2 Internal Weak Pull-up
                                            // Enable
pub const HIB_TPIO_LEV2 = usize(0x00020000);  // TMPR2 Trigger Level
pub const HIB_TPIO_EN2 = usize(0x00010000);  // TMPR2 Enable
pub const HIB_TPIO_GFLTR1 = usize(0x00000800);  // TMPR1 Glitch Filtering
pub const HIB_TPIO_PUEN1 = usize(0x00000400);  // TMPR1 Internal Weak Pull-up
                                            // Enable
pub const HIB_TPIO_LEV1 = usize(0x00000200);  // TMPR1 Trigger Level
pub const HIB_TPIO_EN1 = usize(0x00000100);  // TMPR1Enable
pub const HIB_TPIO_GFLTR0 = usize(0x00000008);  // TMPR0 Glitch Filtering
pub const HIB_TPIO_PUEN0 = usize(0x00000004);  // TMPR0 Internal Weak Pull-up
                                            // Enable
pub const HIB_TPIO_LEV0 = usize(0x00000002);  // TMPR0 Trigger Level
pub const HIB_TPIO_EN0 = usize(0x00000001);  // TMPR0 Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG0 register.
//
//*****************************************************************************
pub const HIB_TPLOG0_TIME_M = usize(0xFFFFFFFF);  // Tamper Log Calendar Information
pub const HIB_TPLOG0_TIME_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG1 register.
//
//*****************************************************************************
pub const HIB_TPLOG1_XOSC = usize(0x00010000);  // Status of external 32
pub const HIB_TPLOG1_TRIG3 = usize(0x00000008);  // Status of TMPR[3] Trigger
pub const HIB_TPLOG1_TRIG2 = usize(0x00000004);  // Status of TMPR[2] Trigger
pub const HIB_TPLOG1_TRIG1 = usize(0x00000002);  // Status of TMPR[1] Trigger
pub const HIB_TPLOG1_TRIG0 = usize(0x00000001);  // Status of TMPR[0] Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG2 register.
//
//*****************************************************************************
pub const HIB_TPLOG2_TIME_M = usize(0xFFFFFFFF);  // Tamper Log Calendar Information
pub const HIB_TPLOG2_TIME_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG3 register.
//
//*****************************************************************************
pub const HIB_TPLOG3_XOSC = usize(0x00010000);  // Status of external 32
pub const HIB_TPLOG3_TRIG3 = usize(0x00000008);  // Status of TMPR[3] Trigger
pub const HIB_TPLOG3_TRIG2 = usize(0x00000004);  // Status of TMPR[2] Trigger
pub const HIB_TPLOG3_TRIG1 = usize(0x00000002);  // Status of TMPR[1] Trigger
pub const HIB_TPLOG3_TRIG0 = usize(0x00000001);  // Status of TMPR[0] Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG4 register.
//
//*****************************************************************************
pub const HIB_TPLOG4_TIME_M = usize(0xFFFFFFFF);  // Tamper Log Calendar Information
pub const HIB_TPLOG4_TIME_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG5 register.
//
//*****************************************************************************
pub const HIB_TPLOG5_XOSC = usize(0x00010000);  // Status of external 32
pub const HIB_TPLOG5_TRIG3 = usize(0x00000008);  // Status of TMPR[3] Trigger
pub const HIB_TPLOG5_TRIG2 = usize(0x00000004);  // Status of TMPR[2] Trigger
pub const HIB_TPLOG5_TRIG1 = usize(0x00000002);  // Status of TMPR[1] Trigger
pub const HIB_TPLOG5_TRIG0 = usize(0x00000001);  // Status of TMPR[0] Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG6 register.
//
//*****************************************************************************
pub const HIB_TPLOG6_TIME_M = usize(0xFFFFFFFF);  // Tamper Log Calendar Information
pub const HIB_TPLOG6_TIME_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_TPLOG7 register.
//
//*****************************************************************************
pub const HIB_TPLOG7_XOSC = usize(0x00010000);  // Status of external 32
pub const HIB_TPLOG7_TRIG3 = usize(0x00000008);  // Status of TMPR[3] Trigger
pub const HIB_TPLOG7_TRIG2 = usize(0x00000004);  // Status of TMPR[2] Trigger
pub const HIB_TPLOG7_TRIG1 = usize(0x00000002);  // Status of TMPR[1] Trigger
pub const HIB_TPLOG7_TRIG0 = usize(0x00000001);  // Status of TMPR[0] Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_PP register.
//
//*****************************************************************************
pub const HIB_PP_TAMPER = usize(0x00000002);  // Tamper Pin Presence
pub const HIB_PP_WAKENC = usize(0x00000001);  // Wake Pin Presence

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CC register.
//
//*****************************************************************************
pub const HIB_CC_SYSCLKEN = usize(0x00000001);  // RTCOSC to System Clock Enable

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
pub const FLASH_PP_SIZE_1MB = usize(0x000001FF);  // 1024 KB of Flash

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_SSIZE register.
//
//*****************************************************************************
pub const FLASH_SSIZE_SIZE_M = usize(0x0000FFFF);  // SRAM Size
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
pub const FLASH_ROMSWMAP_SW7EN_M = usize(0x0000C000);  // ROM SW Region 7 Availability
pub const FLASH_ROMSWMAP_SW7EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW7EN_CORE = usize(0x00004000);  // Region available to core
pub const FLASH_ROMSWMAP_SW6EN_M = usize(0x00003000);  // ROM SW Region 6 Availability
pub const FLASH_ROMSWMAP_SW6EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW6EN_CORE = usize(0x00001000);  // Region available to core
pub const FLASH_ROMSWMAP_SW5EN_M = usize(0x00000C00);  // ROM SW Region 5 Availability
pub const FLASH_ROMSWMAP_SW5EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW5EN_CORE = usize(0x00000400);  // Region available to core
pub const FLASH_ROMSWMAP_SW4EN_M = usize(0x00000300);  // ROM SW Region 4 Availability
pub const FLASH_ROMSWMAP_SW4EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW4EN_CORE = usize(0x00000100);  // Region available to core
pub const FLASH_ROMSWMAP_SW3EN_M = usize(0x000000C0);  // ROM SW Region 3 Availability
pub const FLASH_ROMSWMAP_SW3EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW3EN_CORE = usize(0x00000040);  // Region available to core
pub const FLASH_ROMSWMAP_SW2EN_M = usize(0x00000030);  // ROM SW Region 2 Availability
pub const FLASH_ROMSWMAP_SW2EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW2EN_CORE = usize(0x00000010);  // Region available to core
pub const FLASH_ROMSWMAP_SW1EN_M = usize(0x0000000C);  // ROM SW Region 1 Availability
pub const FLASH_ROMSWMAP_SW1EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW1EN_CORE = usize(0x00000004);  // Region available to core
pub const FLASH_ROMSWMAP_SW0EN_M = usize(0x00000003);  // ROM SW Region 0 Availability
pub const FLASH_ROMSWMAP_SW0EN_NOTVIS = usize(0x00000000);  // Software region not available to
                                            // the core
pub const FLASH_ROMSWMAP_SW0EN_CORE = usize(0x00000001);  // Region available to core

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
// The following are defines for the bit fields in the SYSCTL_DID0 register.
//
//*****************************************************************************
pub const SYSCTL_DID0_VER_M = usize(0x70000000);  // DID0 Version
pub const SYSCTL_DID0_VER_1 = usize(0x10000000);  // Second version of the DID0
                                            // register format.
pub const SYSCTL_DID0_CLASS_M = usize(0x00FF0000);  // Device Class
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
pub const SYSCTL_DID1_PRTNO_TM4C129DNCZAD = usize(0x00290000);  // TM4C129DNCZAD
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
// The following are defines for the bit fields in the SYSCTL_RIS register.
//
//*****************************************************************************
pub const SYSCTL_RIS_MOSCPUPRIS = usize(0x00000100);  // MOSC Power Up Raw Interrupt
                                            // Status
pub const SYSCTL_RIS_PLLLRIS = usize(0x00000040);  // PLL Lock Raw Interrupt Status
pub const SYSCTL_RIS_MOFRIS = usize(0x00000008);  // Main Oscillator Failure Raw
                                            // Interrupt Status
pub const SYSCTL_RIS_BORRIS = usize(0x00000002);  // Brown-Out Reset Raw Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_IMC register.
//
//*****************************************************************************
pub const SYSCTL_IMC_MOSCPUPIM = usize(0x00000100);  // MOSC Power Up Interrupt Mask
pub const SYSCTL_IMC_PLLLIM = usize(0x00000040);  // PLL Lock Interrupt Mask
pub const SYSCTL_IMC_MOFIM = usize(0x00000008);  // Main Oscillator Failure
                                            // Interrupt Mask
pub const SYSCTL_IMC_BORIM = usize(0x00000002);  // Brown-Out Reset Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MISC register.
//
//*****************************************************************************
pub const SYSCTL_MISC_MOSCPUPMIS = usize(0x00000100);  // MOSC Power Up Masked Interrupt
                                            // Status
pub const SYSCTL_MISC_PLLLMIS = usize(0x00000040);  // PLL Lock Masked Interrupt Status
pub const SYSCTL_MISC_MOFMIS = usize(0x00000008);  // Main Oscillator Failure Masked
                                            // Interrupt Status
pub const SYSCTL_MISC_BORMIS = usize(0x00000002);  // BOR Masked Interrupt Status

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
pub const SYSCTL_PPPWM_P0 = usize(0x00000001);  // PWM Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPQEI register.
//
//*****************************************************************************
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
pub const SYSCTL_SRPWM_R0 = usize(0x00000001);  // PWM Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRQEI register.
//
//*****************************************************************************
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
// The following are defines for the bit fields in the SYSCTL_SRCCM register.
//
//*****************************************************************************
pub const SYSCTL_SRCCM_R0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Software Reset

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
pub const SYSCTL_RCGCPWM_R0 = usize(0x00000001);  // PWM Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCQEI register.
//
//*****************************************************************************
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
// The following are defines for the bit fields in the SYSCTL_RCGCCCM register.
//
//*****************************************************************************
pub const SYSCTL_RCGCCCM_R0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Run Mode Clock Gating Control

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
pub const SYSCTL_SCGCPWM_S0 = usize(0x00000001);  // PWM Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCQEI register.
//
//*****************************************************************************
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
// The following are defines for the bit fields in the SYSCTL_SCGCCCM register.
//
//*****************************************************************************
pub const SYSCTL_SCGCCCM_S0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Sleep Mode Clock Gating Control

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
pub const SYSCTL_DCGCPWM_D0 = usize(0x00000001);  // PWM Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCQEI register.
//
//*****************************************************************************
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
// The following are defines for the bit fields in the SYSCTL_DCGCCCM register.
//
//*****************************************************************************
pub const SYSCTL_DCGCCCM_D0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Deep-Sleep Mode Clock Gating
                                            // Control

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
pub const SYSCTL_PRPWM_R0 = usize(0x00000001);  // PWM Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRQEI register.
//
//*****************************************************************************
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
// The following are defines for the bit fields in the SYSCTL_PRCCM register.
//
//*****************************************************************************
pub const SYSCTL_PRCCM_R0 = usize(0x00000001);  // CRC and Cryptographic Modules
                                            // Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PREMAC register.
//
//*****************************************************************************
pub const SYSCTL_PREMAC_R0 = usize(0x00000001);  // Ethernet MAC Module 0 Peripheral
                                            // Ready

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
// The following are defines for the bit fields in the UDMA_STAT register.
//
//*****************************************************************************
pub const UDMA_STAT_DMACHANS_M = usize(0x001F0000);  // Available uDMA Channels Minus 1
pub const UDMA_STAT_STATE_M = usize(0x000000F0);  // Control State Machine Status
pub const UDMA_STAT_STATE_IDLE = usize(0x00000000);  // Idle
pub const UDMA_STAT_STATE_RD_CTRL = usize(0x00000010);  // Reading channel controller data
pub const UDMA_STAT_STATE_RD_SRCENDP = usize(0x00000020);  // Reading source end pointer
pub const UDMA_STAT_STATE_RD_DSTENDP = usize(0x00000030);  // Reading destination end pointer
pub const UDMA_STAT_STATE_RD_SRCDAT = usize(0x00000040);  // Reading source data
pub const UDMA_STAT_STATE_WR_DSTDAT = usize(0x00000050);  // Writing destination data
pub const UDMA_STAT_STATE_WAIT = usize(0x00000060);  // Waiting for uDMA request to
                                            // clear
pub const UDMA_STAT_STATE_WR_CTRL = usize(0x00000070);  // Writing channel controller data
pub const UDMA_STAT_STATE_STALL = usize(0x00000080);  // Stalled
pub const UDMA_STAT_STATE_DONE = usize(0x00000090);  // Done
pub const UDMA_STAT_STATE_UNDEF = usize(0x000000A0);  // Undefined
pub const UDMA_STAT_MASTEN = usize(0x00000001);  // Master Enable Status
pub const UDMA_STAT_DMACHANS_S = usize(16);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CFG register.
//
//*****************************************************************************
pub const UDMA_CFG_MASTEN = usize(0x00000001);  // Controller Master Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CTLBASE register.
//
//*****************************************************************************
pub const UDMA_CTLBASE_ADDR_M = usize(0xFFFFFC00);  // Channel Control Base Address
pub const UDMA_CTLBASE_ADDR_S = usize(10);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTBASE register.
//
//*****************************************************************************
pub const UDMA_ALTBASE_ADDR_M = usize(0xFFFFFFFF);  // Alternate Channel Address
                                            // Pointer
pub const UDMA_ALTBASE_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_WAITSTAT register.
//
//*****************************************************************************
pub const UDMA_WAITSTAT_WAITREQ_M = usize(0xFFFFFFFF);  // Channel [n] Wait Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_SWREQ register.
//
//*****************************************************************************
pub const UDMA_SWREQ_M = usize(0xFFFFFFFF);  // Channel [n] Software Request

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_USEBURSTSET
// register.
//
//*****************************************************************************
pub const UDMA_USEBURSTSET_SET_M = usize(0xFFFFFFFF);  // Channel [n] Useburst Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_USEBURSTCLR
// register.
//
//*****************************************************************************
pub const UDMA_USEBURSTCLR_CLR_M = usize(0xFFFFFFFF);  // Channel [n] Useburst Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_REQMASKSET
// register.
//
//*****************************************************************************
pub const UDMA_REQMASKSET_SET_M = usize(0xFFFFFFFF);  // Channel [n] Request Mask Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_REQMASKCLR
// register.
//
//*****************************************************************************
pub const UDMA_REQMASKCLR_CLR_M = usize(0xFFFFFFFF);  // Channel [n] Request Mask Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ENASET register.
//
//*****************************************************************************
pub const UDMA_ENASET_SET_M = usize(0xFFFFFFFF);  // Channel [n] Enable Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ENACLR register.
//
//*****************************************************************************
pub const UDMA_ENACLR_CLR_M = usize(0xFFFFFFFF);  // Clear Channel [n] Enable Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTSET register.
//
//*****************************************************************************
pub const UDMA_ALTSET_SET_M = usize(0xFFFFFFFF);  // Channel [n] Alternate Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTCLR register.
//
//*****************************************************************************
pub const UDMA_ALTCLR_CLR_M = usize(0xFFFFFFFF);  // Channel [n] Alternate Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_PRIOSET register.
//
//*****************************************************************************
pub const UDMA_PRIOSET_SET_M = usize(0xFFFFFFFF);  // Channel [n] Priority Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_PRIOCLR register.
//
//*****************************************************************************
pub const UDMA_PRIOCLR_CLR_M = usize(0xFFFFFFFF);  // Channel [n] Priority Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ERRCLR register.
//
//*****************************************************************************
pub const UDMA_ERRCLR_ERRCLR = usize(0x00000001);  // uDMA Bus Error Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHASGN register.
//
//*****************************************************************************
pub const UDMA_CHASGN_M = usize(0xFFFFFFFF);  // Channel [n] Assignment Select
pub const UDMA_CHASGN_PRIMARY = usize(0x00000000);  // Use the primary channel
                                            // assignment
pub const UDMA_CHASGN_SECONDARY = usize(0x00000001);  // Use the secondary channel
                                            // assignment

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP0 register.
//
//*****************************************************************************
pub const UDMA_CHMAP0_CH7SEL_M = usize(0xF0000000);  // uDMA Channel 7 Source Select
pub const UDMA_CHMAP0_CH6SEL_M = usize(0x0F000000);  // uDMA Channel 6 Source Select
pub const UDMA_CHMAP0_CH5SEL_M = usize(0x00F00000);  // uDMA Channel 5 Source Select
pub const UDMA_CHMAP0_CH4SEL_M = usize(0x000F0000);  // uDMA Channel 4 Source Select
pub const UDMA_CHMAP0_CH3SEL_M = usize(0x0000F000);  // uDMA Channel 3 Source Select
pub const UDMA_CHMAP0_CH2SEL_M = usize(0x00000F00);  // uDMA Channel 2 Source Select
pub const UDMA_CHMAP0_CH1SEL_M = usize(0x000000F0);  // uDMA Channel 1 Source Select
pub const UDMA_CHMAP0_CH0SEL_M = usize(0x0000000F);  // uDMA Channel 0 Source Select
pub const UDMA_CHMAP0_CH7SEL_S = usize(28);
pub const UDMA_CHMAP0_CH6SEL_S = usize(24);
pub const UDMA_CHMAP0_CH5SEL_S = usize(20);
pub const UDMA_CHMAP0_CH4SEL_S = usize(16);
pub const UDMA_CHMAP0_CH3SEL_S = usize(12);
pub const UDMA_CHMAP0_CH2SEL_S = usize(8);
pub const UDMA_CHMAP0_CH1SEL_S = usize(4);
pub const UDMA_CHMAP0_CH0SEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP1 register.
//
//*****************************************************************************
pub const UDMA_CHMAP1_CH15SEL_M = usize(0xF0000000);  // uDMA Channel 15 Source Select
pub const UDMA_CHMAP1_CH14SEL_M = usize(0x0F000000);  // uDMA Channel 14 Source Select
pub const UDMA_CHMAP1_CH13SEL_M = usize(0x00F00000);  // uDMA Channel 13 Source Select
pub const UDMA_CHMAP1_CH12SEL_M = usize(0x000F0000);  // uDMA Channel 12 Source Select
pub const UDMA_CHMAP1_CH11SEL_M = usize(0x0000F000);  // uDMA Channel 11 Source Select
pub const UDMA_CHMAP1_CH10SEL_M = usize(0x00000F00);  // uDMA Channel 10 Source Select
pub const UDMA_CHMAP1_CH9SEL_M = usize(0x000000F0);  // uDMA Channel 9 Source Select
pub const UDMA_CHMAP1_CH8SEL_M = usize(0x0000000F);  // uDMA Channel 8 Source Select
pub const UDMA_CHMAP1_CH15SEL_S = usize(28);
pub const UDMA_CHMAP1_CH14SEL_S = usize(24);
pub const UDMA_CHMAP1_CH13SEL_S = usize(20);
pub const UDMA_CHMAP1_CH12SEL_S = usize(16);
pub const UDMA_CHMAP1_CH11SEL_S = usize(12);
pub const UDMA_CHMAP1_CH10SEL_S = usize(8);
pub const UDMA_CHMAP1_CH9SEL_S = usize(4);
pub const UDMA_CHMAP1_CH8SEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP2 register.
//
//*****************************************************************************
pub const UDMA_CHMAP2_CH23SEL_M = usize(0xF0000000);  // uDMA Channel 23 Source Select
pub const UDMA_CHMAP2_CH22SEL_M = usize(0x0F000000);  // uDMA Channel 22 Source Select
pub const UDMA_CHMAP2_CH21SEL_M = usize(0x00F00000);  // uDMA Channel 21 Source Select
pub const UDMA_CHMAP2_CH20SEL_M = usize(0x000F0000);  // uDMA Channel 20 Source Select
pub const UDMA_CHMAP2_CH19SEL_M = usize(0x0000F000);  // uDMA Channel 19 Source Select
pub const UDMA_CHMAP2_CH18SEL_M = usize(0x00000F00);  // uDMA Channel 18 Source Select
pub const UDMA_CHMAP2_CH17SEL_M = usize(0x000000F0);  // uDMA Channel 17 Source Select
pub const UDMA_CHMAP2_CH16SEL_M = usize(0x0000000F);  // uDMA Channel 16 Source Select
pub const UDMA_CHMAP2_CH23SEL_S = usize(28);
pub const UDMA_CHMAP2_CH22SEL_S = usize(24);
pub const UDMA_CHMAP2_CH21SEL_S = usize(20);
pub const UDMA_CHMAP2_CH20SEL_S = usize(16);
pub const UDMA_CHMAP2_CH19SEL_S = usize(12);
pub const UDMA_CHMAP2_CH18SEL_S = usize(8);
pub const UDMA_CHMAP2_CH17SEL_S = usize(4);
pub const UDMA_CHMAP2_CH16SEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP3 register.
//
//*****************************************************************************
pub const UDMA_CHMAP3_CH31SEL_M = usize(0xF0000000);  // uDMA Channel 31 Source Select
pub const UDMA_CHMAP3_CH30SEL_M = usize(0x0F000000);  // uDMA Channel 30 Source Select
pub const UDMA_CHMAP3_CH29SEL_M = usize(0x00F00000);  // uDMA Channel 29 Source Select
pub const UDMA_CHMAP3_CH28SEL_M = usize(0x000F0000);  // uDMA Channel 28 Source Select
pub const UDMA_CHMAP3_CH27SEL_M = usize(0x0000F000);  // uDMA Channel 27 Source Select
pub const UDMA_CHMAP3_CH26SEL_M = usize(0x00000F00);  // uDMA Channel 26 Source Select
pub const UDMA_CHMAP3_CH25SEL_M = usize(0x000000F0);  // uDMA Channel 25 Source Select
pub const UDMA_CHMAP3_CH24SEL_M = usize(0x0000000F);  // uDMA Channel 24 Source Select
pub const UDMA_CHMAP3_CH31SEL_S = usize(28);
pub const UDMA_CHMAP3_CH30SEL_S = usize(24);
pub const UDMA_CHMAP3_CH29SEL_S = usize(20);
pub const UDMA_CHMAP3_CH28SEL_S = usize(16);
pub const UDMA_CHMAP3_CH27SEL_S = usize(12);
pub const UDMA_CHMAP3_CH26SEL_S = usize(8);
pub const UDMA_CHMAP3_CH25SEL_S = usize(4);
pub const UDMA_CHMAP3_CH24SEL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_SRCENDP register.
//
//*****************************************************************************
pub const UDMA_SRCENDP_ADDR_M = usize(0xFFFFFFFF);  // Source Address End Pointer
pub const UDMA_SRCENDP_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_DSTENDP register.
//
//*****************************************************************************
pub const UDMA_DSTENDP_ADDR_M = usize(0xFFFFFFFF);  // Destination Address End Pointer
pub const UDMA_DSTENDP_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_CHCTL register.
//
//*****************************************************************************
pub const UDMA_CHCTL_DSTINC_M = usize(0xC0000000);  // Destination Address Increment
pub const UDMA_CHCTL_DSTINC_8 = usize(0x00000000);  // Byte
pub const UDMA_CHCTL_DSTINC_16 = usize(0x40000000);  // Half-word
pub const UDMA_CHCTL_DSTINC_32 = usize(0x80000000);  // Word
pub const UDMA_CHCTL_DSTINC_NONE = usize(0xC0000000);  // No increment
pub const UDMA_CHCTL_DSTSIZE_M = usize(0x30000000);  // Destination Data Size
pub const UDMA_CHCTL_DSTSIZE_8 = usize(0x00000000);  // Byte
pub const UDMA_CHCTL_DSTSIZE_16 = usize(0x10000000);  // Half-word
pub const UDMA_CHCTL_DSTSIZE_32 = usize(0x20000000);  // Word
pub const UDMA_CHCTL_SRCINC_M = usize(0x0C000000);  // Source Address Increment
pub const UDMA_CHCTL_SRCINC_8 = usize(0x00000000);  // Byte
pub const UDMA_CHCTL_SRCINC_16 = usize(0x04000000);  // Half-word
pub const UDMA_CHCTL_SRCINC_32 = usize(0x08000000);  // Word
pub const UDMA_CHCTL_SRCINC_NONE = usize(0x0C000000);  // No increment
pub const UDMA_CHCTL_SRCSIZE_M = usize(0x03000000);  // Source Data Size
pub const UDMA_CHCTL_SRCSIZE_8 = usize(0x00000000);  // Byte
pub const UDMA_CHCTL_SRCSIZE_16 = usize(0x01000000);  // Half-word
pub const UDMA_CHCTL_SRCSIZE_32 = usize(0x02000000);  // Word
pub const UDMA_CHCTL_DSTPROT0 = usize(0x00200000);  // Destination Privilege Access
pub const UDMA_CHCTL_SRCPROT0 = usize(0x00040000);  // Source Privilege Access
pub const UDMA_CHCTL_ARBSIZE_M = usize(0x0003C000);  // Arbitration Size
pub const UDMA_CHCTL_ARBSIZE_1 = usize(0x00000000);  // 1 Transfer
pub const UDMA_CHCTL_ARBSIZE_2 = usize(0x00004000);  // 2 Transfers
pub const UDMA_CHCTL_ARBSIZE_4 = usize(0x00008000);  // 4 Transfers
pub const UDMA_CHCTL_ARBSIZE_8 = usize(0x0000C000);  // 8 Transfers
pub const UDMA_CHCTL_ARBSIZE_16 = usize(0x00010000);  // 16 Transfers
pub const UDMA_CHCTL_ARBSIZE_32 = usize(0x00014000);  // 32 Transfers
pub const UDMA_CHCTL_ARBSIZE_64 = usize(0x00018000);  // 64 Transfers
pub const UDMA_CHCTL_ARBSIZE_128 = usize(0x0001C000);  // 128 Transfers
pub const UDMA_CHCTL_ARBSIZE_256 = usize(0x00020000);  // 256 Transfers
pub const UDMA_CHCTL_ARBSIZE_512 = usize(0x00024000);  // 512 Transfers
pub const UDMA_CHCTL_ARBSIZE_1024 = usize(0x00028000);  // 1024 Transfers
pub const UDMA_CHCTL_XFERSIZE_M = usize(0x00003FF0);  // Transfer Size (minus 1)
pub const UDMA_CHCTL_NXTUSEBURST = usize(0x00000008);  // Next Useburst
pub const UDMA_CHCTL_XFERMODE_M = usize(0x00000007);  // uDMA Transfer Mode
pub const UDMA_CHCTL_XFERMODE_STOP = usize(0x00000000);  // Stop
pub const UDMA_CHCTL_XFERMODE_BASIC = usize(0x00000001);  // Basic
pub const UDMA_CHCTL_XFERMODE_AUTO = usize(0x00000002);  // Auto-Request
pub const UDMA_CHCTL_XFERMODE_PINGPONG = usize(0x00000003);  // Ping-Pong
pub const UDMA_CHCTL_XFERMODE_MEM_SG = usize(0x00000004);  // Memory Scatter-Gather
pub const UDMA_CHCTL_XFERMODE_MEM_SGA = usize(0x00000005);  // Alternate Memory Scatter-Gather
pub const UDMA_CHCTL_XFERMODE_PER_SG = usize(0x00000006);  // Peripheral Scatter-Gather
pub const UDMA_CHCTL_XFERMODE_PER_SGA = usize(0x00000007);  // Alternate Peripheral
                                            // Scatter-Gather
pub const UDMA_CHCTL_XFERSIZE_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the CCM_O_CRCCTRL register.
//
//*****************************************************************************
pub const CCM_CRCCTRL_INIT_M = usize(0x00006000);  // CRC Initialization
pub const CCM_CRCCTRL_INIT_SEED = usize(0x00000000);  // Use the CRCSEED register context
                                            // as the starting value
pub const CCM_CRCCTRL_INIT_0 = usize(0x00004000);  // Initialize to all '0s'
pub const CCM_CRCCTRL_INIT_1 = usize(0x00006000);  // Initialize to all '1s'
pub const CCM_CRCCTRL_SIZE = usize(0x00001000);  // Input Data Size
pub const CCM_CRCCTRL_RESINV = usize(0x00000200);  // Result Inverse Enable
pub const CCM_CRCCTRL_OBR = usize(0x00000100);  // Output Reverse Enable
pub const CCM_CRCCTRL_BR = usize(0x00000080);  // Bit reverse enable
pub const CCM_CRCCTRL_ENDIAN_M = usize(0x00000030);  // Endian Control
pub const CCM_CRCCTRL_ENDIAN_SBHW = usize(0x00000000);  // Configuration unchanged. (B3,
                                            // B2, B1, B0)
pub const CCM_CRCCTRL_ENDIAN_SHW = usize(0x00000010);  // Bytes are swapped in half-words
                                            // but half-words are not swapped
                                            // (B2, B3, B0, B1)
pub const CCM_CRCCTRL_ENDIAN_SHWNB = usize(0x00000020);  // Half-words are swapped but bytes
                                            // are not swapped in half-word.
                                            // (B1, B0, B3, B2)
pub const CCM_CRCCTRL_ENDIAN_SBSW = usize(0x00000030);  // Bytes are swapped in half-words
                                            // and half-words are swapped. (B0,
                                            // B1, B2, B3)
pub const CCM_CRCCTRL_TYPE_M = usize(0x0000000F);  // Operation Type
pub const CCM_CRCCTRL_TYPE_P8055 = usize(0x00000000);  // Polynomial 0x8005
pub const CCM_CRCCTRL_TYPE_P1021 = usize(0x00000001);  // Polynomial 0x1021
pub const CCM_CRCCTRL_TYPE_P4C11DB7 = usize(0x00000002);  // Polynomial 0x4C11DB7
pub const CCM_CRCCTRL_TYPE_P1EDC6F41 = usize(0x00000003);  // Polynomial 0x1EDC6F41
pub const CCM_CRCCTRL_TYPE_TCPCHKSUM = usize(0x00000008);  // TCP checksum

//*****************************************************************************
//
// The following are defines for the bit fields in the CCM_O_CRCSEED register.
//
//*****************************************************************************
pub const CCM_CRCSEED_SEED_M = usize(0xFFFFFFFF);  // SEED/Context Value
pub const CCM_CRCSEED_SEED_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CCM_O_CRCDIN register.
//
//*****************************************************************************
pub const CCM_CRCDIN_DATAIN_M = usize(0xFFFFFFFF);  // Data Input
pub const CCM_CRCDIN_DATAIN_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the CCM_O_CRCRSLTPP
// register.
//
//*****************************************************************************
pub const CCM_CRCRSLTPP_RSLTPP_M = usize(0xFFFFFFFF);  // Post Processing Result
pub const CCM_CRCRSLTPP_RSLTPP_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_A
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_A_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_A_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_B
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_B_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_B_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_C
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_C_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_C_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_D
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_D_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_D_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_E
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_E_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_E_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_F
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_F_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_F_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_G
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_G_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_G_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_ODIGEST_H
// register.
//
//*****************************************************************************
pub const SHAMD5_ODIGEST_H_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_ODIGEST_H_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_A
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_A_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_A_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_B
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_B_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_B_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_C
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_C_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_C_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_D
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_D_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_D_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_E
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_E_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_E_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_F
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_F_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_F_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_G
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_G_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_G_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IDIGEST_H
// register.
//
//*****************************************************************************
pub const SHAMD5_IDIGEST_H_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_IDIGEST_H_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DIGEST_COUNT
// register.
//
//*****************************************************************************
pub const SHAMD5_DIGEST_COUNT_M = usize(0xFFFFFFFF);  // Digest Count
pub const SHAMD5_DIGEST_COUNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_MODE register.
//
//*****************************************************************************
pub const SHAMD5_MODE_HMAC_OUTER_HASH = usize(0x00000080);  // HMAC Outer Hash Processing
                                            // Enable
pub const SHAMD5_MODE_HMAC_KEY_PROC = usize(0x00000020);  // HMAC Key Processing Enable
pub const SHAMD5_MODE_CLOSE_HASH = usize(0x00000010);  // Performs the padding, the
                                            // Hash/HMAC will be 'closed' at
                                            // the end of the block, as per
                                            // MD5/SHA-1/SHA-2 specification
pub const SHAMD5_MODE_ALGO_CONSTANT = usize(0x00000008);  // The initial digest register will
                                            // be overwritten with the
                                            // algorithm constants for the
                                            // selected algorithm when hashing
                                            // and the initial digest count
                                            // register will be reset to 0
pub const SHAMD5_MODE_ALGO_M = usize(0x00000007);  // Hash Algorithm
pub const SHAMD5_MODE_ALGO_MD5 = usize(0x00000000);  // MD5
pub const SHAMD5_MODE_ALGO_SHA1 = usize(0x00000002);  // SHA-1
pub const SHAMD5_MODE_ALGO_SHA224 = usize(0x00000004);  // SHA-224
pub const SHAMD5_MODE_ALGO_SHA256 = usize(0x00000006);  // SHA-256

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_LENGTH
// register.
//
//*****************************************************************************
pub const SHAMD5_LENGTH_M = usize(0xFFFFFFFF);  // Block Length/Remaining Byte
                                            // Count
pub const SHAMD5_LENGTH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_0_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_0_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_0_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_1_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_1_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_1_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_2_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_2_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_2_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_3_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_3_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_3_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_4_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_4_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_4_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_5_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_5_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_5_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_6_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_6_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_6_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_7_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_7_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_7_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_8_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_8_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_8_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_9_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_9_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_9_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_10_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_10_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_10_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_11_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_11_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_11_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_12_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_12_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_12_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_13_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_13_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_13_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_14_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_14_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_14_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DATA_15_IN
// register.
//
//*****************************************************************************
pub const SHAMD5_DATA_15_IN_DATA_M = usize(0xFFFFFFFF);  // Digest/Key Data
pub const SHAMD5_DATA_15_IN_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_REVISION
// register.
//
//*****************************************************************************
pub const SHAMD5_REVISION_M = usize(0xFFFFFFFF);  // Revision Number
pub const SHAMD5_REVISION_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_SYSCONFIG
// register.
//
//*****************************************************************************
pub const SHAMD5_SYSCONFIG_SADVANCED = usize(0x00000080);  // Advanced Mode Enable
pub const SHAMD5_SYSCONFIG_SIDLE_M = usize(0x00000030);  // Sidle mode
pub const SHAMD5_SYSCONFIG_SIDLE_FORCE = usize(0x00000000);  // Force-idle mode
pub const SHAMD5_SYSCONFIG_DMA_EN = usize(0x00000008);  // uDMA Request Enable
pub const SHAMD5_SYSCONFIG_IT_EN = usize(0x00000004);  // Interrupt Enable
pub const SHAMD5_SYSCONFIG_SOFTRESET = usize(0x00000002);  // Soft reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_SYSSTATUS
// register.
//
//*****************************************************************************
pub const SHAMD5_SYSSTATUS_RESETDONE = usize(0x00000001);  // Reset done status

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IRQSTATUS
// register.
//
//*****************************************************************************
pub const SHAMD5_IRQSTATUS_CONTEXT_READY = usize(0x00000008);  // Context Ready Status
pub const SHAMD5_IRQSTATUS_INPUT_READY = usize(0x00000002);  // Input Ready Status
pub const SHAMD5_IRQSTATUS_OUTPUT_READY = usize(0x00000001);  // Output Ready Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_IRQENABLE
// register.
//
//*****************************************************************************
pub const SHAMD5_IRQENABLE_CONTEXT_READY = usize(0x00000008);  // Mask for context ready interrupt
pub const SHAMD5_IRQENABLE_INPUT_READY = usize(0x00000002);  // Mask for input ready interrupt
pub const SHAMD5_IRQENABLE_OUTPUT_READY = usize(0x00000001);  // Mask for output ready interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DMAIM register.
//
//*****************************************************************************
pub const SHAMD5_DMAIM_COUT = usize(0x00000004);  // Context Out DMA Done Interrupt
                                            // Mask
pub const SHAMD5_DMAIM_DIN = usize(0x00000002);  // Data In DMA Done Interrupt Mask
pub const SHAMD5_DMAIM_CIN = usize(0x00000001);  // Context In DMA Done Interrupt
                                            // Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DMARIS
// register.
//
//*****************************************************************************
pub const SHAMD5_DMARIS_COUT = usize(0x00000004);  // Context Out DMA Done Raw
                                            // Interrupt Status
pub const SHAMD5_DMARIS_DIN = usize(0x00000002);  // Data In DMA Done Raw Interrupt
                                            // Status
pub const SHAMD5_DMARIS_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DMAMIS
// register.
//
//*****************************************************************************
pub const SHAMD5_DMAMIS_COUT = usize(0x00000004);  // Context Out DMA Done Masked
                                            // Interrupt Status
pub const SHAMD5_DMAMIS_DIN = usize(0x00000002);  // Data In DMA Done Masked
                                            // Interrupt Status
pub const SHAMD5_DMAMIS_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SHAMD5_O_DMAIC register.
//
//*****************************************************************************
pub const SHAMD5_DMAIC_COUT = usize(0x00000004);  // Context Out DMA Done Masked
                                            // Interrupt Status
pub const SHAMD5_DMAIC_DIN = usize(0x00000002);  // Data In DMA Done Interrupt Clear
pub const SHAMD5_DMAIC_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_6 register.
//
//*****************************************************************************
pub const AES_KEY2_6_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY2_6_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_7 register.
//
//*****************************************************************************
pub const AES_KEY2_7_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY2_7_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_4 register.
//
//*****************************************************************************
pub const AES_KEY2_4_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY2_4_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_5 register.
//
//*****************************************************************************
pub const AES_KEY2_5_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY2_5_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_2 register.
//
//*****************************************************************************
pub const AES_KEY2_2_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY2_2_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_3 register.
//
//*****************************************************************************
pub const AES_KEY2_3_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY2_3_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_0 register.
//
//*****************************************************************************
pub const AES_KEY2_0_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY2_0_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY2_1 register.
//
//*****************************************************************************
pub const AES_KEY2_1_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY2_1_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_6 register.
//
//*****************************************************************************
pub const AES_KEY1_6_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY1_6_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_7 register.
//
//*****************************************************************************
pub const AES_KEY1_7_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY1_7_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_4 register.
//
//*****************************************************************************
pub const AES_KEY1_4_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY1_4_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_5 register.
//
//*****************************************************************************
pub const AES_KEY1_5_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY1_5_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_2 register.
//
//*****************************************************************************
pub const AES_KEY1_2_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY1_2_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_3 register.
//
//*****************************************************************************
pub const AES_KEY1_3_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY1_3_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_0 register.
//
//*****************************************************************************
pub const AES_KEY1_0_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY1_0_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_KEY1_1 register.
//
//*****************************************************************************
pub const AES_KEY1_1_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const AES_KEY1_1_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IV_IN_0 register.
//
//*****************************************************************************
pub const AES_IV_IN_0_DATA_M = usize(0xFFFFFFFF);  // Initialization Vector Input
pub const AES_IV_IN_0_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IV_IN_1 register.
//
//*****************************************************************************
pub const AES_IV_IN_1_DATA_M = usize(0xFFFFFFFF);  // Initialization Vector Input
pub const AES_IV_IN_1_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IV_IN_2 register.
//
//*****************************************************************************
pub const AES_IV_IN_2_DATA_M = usize(0xFFFFFFFF);  // Initialization Vector Input
pub const AES_IV_IN_2_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IV_IN_3 register.
//
//*****************************************************************************
pub const AES_IV_IN_3_DATA_M = usize(0xFFFFFFFF);  // Initialization Vector Input
pub const AES_IV_IN_3_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_CTRL register.
//
//*****************************************************************************
pub const AES_CTRL_CTXTRDY = usize(0x80000000);  // Context Data Registers Ready
pub const AES_CTRL_SVCTXTRDY = usize(0x40000000);  // AES TAG/IV Block(s) Ready
pub const AES_CTRL_SAVE_CONTEXT = usize(0x20000000);  // TAG or Result IV Save
pub const AES_CTRL_CCM_M_M = usize(0x01C00000);  // Counter with CBC-MAC (CCM)
pub const AES_CTRL_CCM_L_M = usize(0x00380000);  // L Value
pub const AES_CTRL_CCM_L_2 = usize(0x00080000);  // width = 2
pub const AES_CTRL_CCM_L_4 = usize(0x00180000);  // width = 4
pub const AES_CTRL_CCM_L_8 = usize(0x00380000);  // width = 8
pub const AES_CTRL_CCM = usize(0x00040000);  // AES-CCM Mode Enable
pub const AES_CTRL_GCM_M = usize(0x00030000);  // AES-GCM Mode Enable
pub const AES_CTRL_GCM_NOP = usize(0x00000000);  // No operation
pub const AES_CTRL_GCM_HLY0ZERO = usize(0x00010000);  // GHASH with H loaded and
                                            // Y0-encrypted forced to zero
pub const AES_CTRL_GCM_HLY0CALC = usize(0x00020000);  // GHASH with H loaded and
                                            // Y0-encrypted calculated
                                            // internally
pub const AES_CTRL_GCM_HY0CALC = usize(0x00030000);  // Autonomous GHASH (both H and
                                            // Y0-encrypted calculated
                                            // internally)
pub const AES_CTRL_CBCMAC = usize(0x00008000);  // AES-CBC MAC Enable
pub const AES_CTRL_F9 = usize(0x00004000);  // AES f9 Mode Enable
pub const AES_CTRL_F8 = usize(0x00002000);  // AES f8 Mode Enable
pub const AES_CTRL_XTS_M = usize(0x00001800);  // AES-XTS Operation Enabled
pub const AES_CTRL_XTS_NOP = usize(0x00000000);  // No operation
pub const AES_CTRL_XTS_TWEAKJL = usize(0x00000800);  // Previous/intermediate tweak
                                            // value and j loaded (value is
                                            // loaded via IV, j is loaded via
                                            // the AAD length register)
pub const AES_CTRL_XTS_K2IJL = usize(0x00001000);  // Key2, n and j are loaded (n is
                                            // loaded via IV, j is loaded via
                                            // the AAD length register)
pub const AES_CTRL_XTS_K2ILJ0 = usize(0x00001800);  // Key2 and n are loaded; j=0 (n is
                                            // loaded via IV)
pub const AES_CTRL_CFB = usize(0x00000400);  // Full block AES cipher feedback
                                            // mode (CFB128) Enable
pub const AES_CTRL_ICM = usize(0x00000200);  // AES Integer Counter Mode (ICM)
                                            // Enable
pub const AES_CTRL_CTR_WIDTH_M = usize(0x00000180);  // AES-CTR Mode Counter Width
pub const AES_CTRL_CTR_WIDTH_32 = usize(0x00000000);  // Counter is 32 bits
pub const AES_CTRL_CTR_WIDTH_64 = usize(0x00000080);  // Counter is 64 bits
pub const AES_CTRL_CTR_WIDTH_96 = usize(0x00000100);  // Counter is 96 bits
pub const AES_CTRL_CTR_WIDTH_128 = usize(0x00000180);  // Counter is 128 bits
pub const AES_CTRL_CTR = usize(0x00000040);  // Counter Mode
pub const AES_CTRL_MODE = usize(0x00000020);  // ECB/CBC Mode
pub const AES_CTRL_KEY_SIZE_M = usize(0x00000018);  // Key Size
pub const AES_CTRL_KEY_SIZE_128 = usize(0x00000008);  // Key is 128 bits
pub const AES_CTRL_KEY_SIZE_192 = usize(0x00000010);  // Key is 192 bits
pub const AES_CTRL_KEY_SIZE_256 = usize(0x00000018);  // Key is 256 bits
pub const AES_CTRL_DIRECTION = usize(0x00000004);  // Encryption/Decryption Selection
pub const AES_CTRL_INPUT_READY = usize(0x00000002);  // Input Ready Status
pub const AES_CTRL_OUTPUT_READY = usize(0x00000001);  // Output Ready Status
pub const AES_CTRL_CCM_M_S = usize(22);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_C_LENGTH_0
// register.
//
//*****************************************************************************
pub const AES_C_LENGTH_0_LENGTH_M = usize(0xFFFFFFFF);  // Data Length
pub const AES_C_LENGTH_0_LENGTH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_C_LENGTH_1
// register.
//
//*****************************************************************************
pub const AES_C_LENGTH_1_LENGTH_M = usize(0xFFFFFFFF);  // Data Length
pub const AES_C_LENGTH_1_LENGTH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_AUTH_LENGTH
// register.
//
//*****************************************************************************
pub const AES_AUTH_LENGTH_AUTH_M = usize(0xFFFFFFFF);  // Authentication Data Length
pub const AES_AUTH_LENGTH_AUTH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DATA_IN_0
// register.
//
//*****************************************************************************
pub const AES_DATA_IN_0_DATA_M = usize(0xFFFFFFFF);  // Secure Data RW
                                            // Plaintext/Ciphertext
pub const AES_DATA_IN_0_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DATA_IN_1
// register.
//
//*****************************************************************************
pub const AES_DATA_IN_1_DATA_M = usize(0xFFFFFFFF);  // Secure Data RW
                                            // Plaintext/Ciphertext
pub const AES_DATA_IN_1_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DATA_IN_2
// register.
//
//*****************************************************************************
pub const AES_DATA_IN_2_DATA_M = usize(0xFFFFFFFF);  // Secure Data RW
                                            // Plaintext/Ciphertext
pub const AES_DATA_IN_2_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DATA_IN_3
// register.
//
//*****************************************************************************
pub const AES_DATA_IN_3_DATA_M = usize(0xFFFFFFFF);  // Secure Data RW
                                            // Plaintext/Ciphertext
pub const AES_DATA_IN_3_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_TAG_OUT_0
// register.
//
//*****************************************************************************
pub const AES_TAG_OUT_0_HASH_M = usize(0xFFFFFFFF);  // Hash Result
pub const AES_TAG_OUT_0_HASH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_TAG_OUT_1
// register.
//
//*****************************************************************************
pub const AES_TAG_OUT_1_HASH_M = usize(0xFFFFFFFF);  // Hash Result
pub const AES_TAG_OUT_1_HASH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_TAG_OUT_2
// register.
//
//*****************************************************************************
pub const AES_TAG_OUT_2_HASH_M = usize(0xFFFFFFFF);  // Hash Result
pub const AES_TAG_OUT_2_HASH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_TAG_OUT_3
// register.
//
//*****************************************************************************
pub const AES_TAG_OUT_3_HASH_M = usize(0xFFFFFFFF);  // Hash Result
pub const AES_TAG_OUT_3_HASH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_REVISION register.
//
//*****************************************************************************
pub const AES_REVISION_M = usize(0xFFFFFFFF);  // Revision number
pub const AES_REVISION_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_SYSCONFIG
// register.
//
//*****************************************************************************
pub const AES_SYSCONFIG_K3 = usize(0x00001000);  // K3 Select
pub const AES_SYSCONFIG_KEYENC = usize(0x00000800);  // Key Encoding
pub const AES_SYSCONFIG_MAP_CONTEXT_OUT_ON_DATA_OUT = usize(0x00000200);  // Map Context Out on Data Out
                                            // Enable
pub const AES_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN = usize(0x00000100);  // DMA Request Context Out Enable
pub const AES_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN = usize(0x00000080);  // DMA Request Context In Enable
pub const AES_SYSCONFIG_DMA_REQ_DATA_OUT_EN = usize(0x00000040);  // DMA Request Data Out Enable
pub const AES_SYSCONFIG_DMA_REQ_DATA_IN_EN = usize(0x00000020);  // DMA Request Data In Enable
pub const AES_SYSCONFIG_SOFTRESET = usize(0x00000002);  // Soft reset

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_SYSSTATUS
// register.
//
//*****************************************************************************
pub const AES_SYSSTATUS_RESETDONE = usize(0x00000001);  // Reset Done

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IRQSTATUS
// register.
//
//*****************************************************************************
pub const AES_IRQSTATUS_CONTEXT_OUT = usize(0x00000008);  // Context Output Interrupt Status
pub const AES_IRQSTATUS_DATA_OUT = usize(0x00000004);  // Data Out Interrupt Status
pub const AES_IRQSTATUS_DATA_IN = usize(0x00000002);  // Data In Interrupt Status
pub const AES_IRQSTATUS_CONTEXT_IN = usize(0x00000001);  // Context In Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_IRQENABLE
// register.
//
//*****************************************************************************
pub const AES_IRQENABLE_CONTEXT_OUT = usize(0x00000008);  // Context Out Interrupt Enable
pub const AES_IRQENABLE_DATA_OUT = usize(0x00000004);  // Data Out Interrupt Enable
pub const AES_IRQENABLE_DATA_IN = usize(0x00000002);  // Data In Interrupt Enable
pub const AES_IRQENABLE_CONTEXT_IN = usize(0x00000001);  // Context In Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DIRTYBITS
// register.
//
//*****************************************************************************
pub const AES_DIRTYBITS_S_DIRTY = usize(0x00000002);  // AES Dirty Bit
pub const AES_DIRTYBITS_S_ACCESS = usize(0x00000001);  // AES Access Bit

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DMAIM register.
//
//*****************************************************************************
pub const AES_DMAIM_DOUT = usize(0x00000008);  // Data Out DMA Done Interrupt Mask
pub const AES_DMAIM_DIN = usize(0x00000004);  // Data In DMA Done Interrupt Mask
pub const AES_DMAIM_COUT = usize(0x00000002);  // Context Out DMA Done Interrupt
                                            // Mask
pub const AES_DMAIM_CIN = usize(0x00000001);  // Context In DMA Done Interrupt
                                            // Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DMARIS register.
//
//*****************************************************************************
pub const AES_DMARIS_DOUT = usize(0x00000008);  // Data Out DMA Done Raw Interrupt
                                            // Status
pub const AES_DMARIS_DIN = usize(0x00000004);  // Data In DMA Done Raw Interrupt
                                            // Status
pub const AES_DMARIS_COUT = usize(0x00000002);  // Context Out DMA Done Raw
                                            // Interrupt Status
pub const AES_DMARIS_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DMAMIS register.
//
//*****************************************************************************
pub const AES_DMAMIS_DOUT = usize(0x00000008);  // Data Out DMA Done Masked
                                            // Interrupt Status
pub const AES_DMAMIS_DIN = usize(0x00000004);  // Data In DMA Done Masked
                                            // Interrupt Status
pub const AES_DMAMIS_COUT = usize(0x00000002);  // Context Out DMA Done Masked
                                            // Interrupt Status
pub const AES_DMAMIS_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the AES_O_DMAIC register.
//
//*****************************************************************************
pub const AES_DMAIC_DOUT = usize(0x00000008);  // Data Out DMA Done Interrupt
                                            // Clear
pub const AES_DMAIC_DIN = usize(0x00000004);  // Data In DMA Done Interrupt Clear
pub const AES_DMAIC_COUT = usize(0x00000002);  // Context Out DMA Done Masked
                                            // Interrupt Status
pub const AES_DMAIC_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY3_L register.
//
//*****************************************************************************
pub const DES_KEY3_L_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const DES_KEY3_L_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY3_H register.
//
//*****************************************************************************
pub const DES_KEY3_H_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const DES_KEY3_H_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY2_L register.
//
//*****************************************************************************
pub const DES_KEY2_L_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const DES_KEY2_L_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY2_H register.
//
//*****************************************************************************
pub const DES_KEY2_H_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const DES_KEY2_H_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY1_L register.
//
//*****************************************************************************
pub const DES_KEY1_L_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const DES_KEY1_L_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_KEY1_H register.
//
//*****************************************************************************
pub const DES_KEY1_H_KEY_M = usize(0xFFFFFFFF);  // Key Data
pub const DES_KEY1_H_KEY_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_IV_L register.
//
//*****************************************************************************
pub const DES_IV_L_M = usize(0xFFFFFFFF);  // Initialization vector for CBC,
                                            // CFB modes (LSW)
pub const DES_IV_L_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_IV_H register.
//
//*****************************************************************************
pub const DES_IV_H_M = usize(0xFFFFFFFF);  // Initialization vector for CBC,
                                            // CFB modes (MSW)
pub const DES_IV_H_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_CTRL register.
//
//*****************************************************************************
pub const DES_CTRL_CONTEXT = usize(0x80000000);  // If 1, this read-only status bit
                                            // indicates that the context data
                                            // registers can be overwritten and
                                            // the host is permitted to write
                                            // the next context
pub const DES_CTRL_MODE_M = usize(0x00000030);  // Select CBC, ECB or CFB mode0x0:
                                            // ECB mode0x1: CBC mode0x2: CFB
                                            // mode0x3: reserved
pub const DES_CTRL_TDES = usize(0x00000008);  // Select DES or triple DES
                                            // encryption/decryption
pub const DES_CTRL_DIRECTION = usize(0x00000004);  // Select encryption/decryption
                                            // 0x0: decryption is selected0x1:
                                            // Encryption is selected
pub const DES_CTRL_INPUT_READY = usize(0x00000002);  // When 1, ready to encrypt/decrypt
                                            // data
pub const DES_CTRL_OUTPUT_READY = usize(0x00000001);  // When 1, Data decrypted/encrypted
                                            // ready
pub const DES_CTRL_MODE_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_LENGTH register.
//
//*****************************************************************************
pub const DES_LENGTH_M = usize(0xFFFFFFFF);  // Cryptographic data length in
                                            // bytes for all modes
pub const DES_LENGTH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DATA_L register.
//
//*****************************************************************************
pub const DES_DATA_L_M = usize(0xFFFFFFFF);  // Data for encryption/decryption,
                                            // LSW
pub const DES_DATA_L_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DATA_H register.
//
//*****************************************************************************
pub const DES_DATA_H_M = usize(0xFFFFFFFF);  // Data for encryption/decryption,
                                            // MSW
pub const DES_DATA_H_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_REVISION register.
//
//*****************************************************************************
pub const DES_REVISION_M = usize(0xFFFFFFFF);  // Revision number
pub const DES_REVISION_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_SYSCONFIG
// register.
//
//*****************************************************************************
pub const DES_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN = usize(0x00000080);  // DMA Request Context In Enable
pub const DES_SYSCONFIG_DMA_REQ_DATA_OUT_EN = usize(0x00000040);  // DMA Request Data Out Enable
pub const DES_SYSCONFIG_DMA_REQ_DATA_IN_EN = usize(0x00000020);  // DMA Request Data In Enable
pub const DES_SYSCONFIG_SIDLE_M = usize(0x0000000C);  // Sidle mode
pub const DES_SYSCONFIG_SIDLE_FORCE = usize(0x00000000);  // Force-idle mode
pub const DES_SYSCONFIG_SOFTRESET = usize(0x00000002);  // Soft reset

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_SYSSTATUS
// register.
//
//*****************************************************************************
pub const DES_SYSSTATUS_RESETDONE = usize(0x00000001);  // Reset Done

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_IRQSTATUS
// register.
//
//*****************************************************************************
pub const DES_IRQSTATUS_DATA_OUT = usize(0x00000004);  // This bit indicates data output
                                            // interrupt is active and triggers
                                            // the interrupt output
pub const DES_IRQSTATUS_DATA_IN = usize(0x00000002);  // This bit indicates data input
                                            // interrupt is active and triggers
                                            // the interrupt output
pub const DES_IRQSTATUS_CONTEX_IN = usize(0x00000001);  // This bit indicates context
                                            // interrupt is active and triggers
                                            // the interrupt output

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_IRQENABLE
// register.
//
//*****************************************************************************
pub const DES_IRQENABLE_M_DATA_OUT = usize(0x00000004);  // If this bit is set to 1 the data
                                            // output interrupt is enabled
pub const DES_IRQENABLE_M_DATA_IN = usize(0x00000002);  // If this bit is set to 1 the data
                                            // input interrupt is enabled
pub const DES_IRQENABLE_M_CONTEX_IN = usize(0x00000001);  // If this bit is set to 1 the
                                            // context interrupt is enabled

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DIRTYBITS
// register.
//
//*****************************************************************************
pub const DES_DIRTYBITS_S_DIRTY = usize(0x00000002);  // This bit is set to 1 by the
                                            // module if any of the DES_*
                                            // registers is written
pub const DES_DIRTYBITS_S_ACCESS = usize(0x00000001);  // This bit is set to 1 by the
                                            // module if any of the DES_*
                                            // registers is read

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DMAIM register.
//
//*****************************************************************************
pub const DES_DMAIM_DOUT = usize(0x00000004);  // Data Out DMA Done Interrupt Mask
pub const DES_DMAIM_DIN = usize(0x00000002);  // Data In DMA Done Interrupt Mask
pub const DES_DMAIM_CIN = usize(0x00000001);  // Context In DMA Done Interrupt
                                            // Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DMARIS register.
//
//*****************************************************************************
pub const DES_DMARIS_DOUT = usize(0x00000004);  // Data Out DMA Done Raw Interrupt
                                            // Status
pub const DES_DMARIS_DIN = usize(0x00000002);  // Data In DMA Done Raw Interrupt
                                            // Status
pub const DES_DMARIS_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DMAMIS register.
//
//*****************************************************************************
pub const DES_DMAMIS_DOUT = usize(0x00000004);  // Data Out DMA Done Masked
                                            // Interrupt Status
pub const DES_DMAMIS_DIN = usize(0x00000002);  // Data In DMA Done Masked
                                            // Interrupt Status
pub const DES_DMAMIS_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the DES_O_DMAIC register.
//
//*****************************************************************************
pub const DES_DMAIC_DOUT = usize(0x00000004);  // Data Out DMA Done Interrupt
                                            // Clear
pub const DES_DMAIC_DIN = usize(0x00000002);  // Data In DMA Done Interrupt Clear
pub const DES_DMAIC_CIN = usize(0x00000001);  // Context In DMA Done Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTLR register.
//
//*****************************************************************************
pub const NVIC_ACTLR_DISOOFP = usize(0x00000200);  // Disable Out-Of-Order Floating
                                            // Point
pub const NVIC_ACTLR_DISFPCA = usize(0x00000100);  // Disable CONTROL
pub const NVIC_ACTLR_DISFOLD = usize(0x00000004);  // Disable IT Folding
pub const NVIC_ACTLR_DISWBUF = usize(0x00000002);  // Disable Write Buffer
pub const NVIC_ACTLR_DISMCYC = usize(0x00000001);  // Disable Interrupts of Multiple
                                            // Cycle Instructions

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_CTRL register.
//
//*****************************************************************************
pub const NVIC_ST_CTRL_COUNT = usize(0x00010000);  // Count Flag
pub const NVIC_ST_CTRL_CLK_SRC = usize(0x00000004);  // Clock Source
pub const NVIC_ST_CTRL_INTEN = usize(0x00000002);  // Interrupt Enable
pub const NVIC_ST_CTRL_ENABLE = usize(0x00000001);  // Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_RELOAD register.
//
//*****************************************************************************
pub const NVIC_ST_RELOAD_M = usize(0x00FFFFFF);  // Reload Value
pub const NVIC_ST_RELOAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_CURRENT
// register.
//
//*****************************************************************************
pub const NVIC_ST_CURRENT_M = usize(0x00FFFFFF);  // Current Value
pub const NVIC_ST_CURRENT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN0 register.
//
//*****************************************************************************
pub const NVIC_EN0_INT_M = usize(0xFFFFFFFF);  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN1 register.
//
//*****************************************************************************
pub const NVIC_EN1_INT_M = usize(0xFFFFFFFF);  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN2 register.
//
//*****************************************************************************
pub const NVIC_EN2_INT_M = usize(0xFFFFFFFF);  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN3 register.
//
//*****************************************************************************
pub const NVIC_EN3_INT_M = usize(0xFFFFFFFF);  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS0 register.
//
//*****************************************************************************
pub const NVIC_DIS0_INT_M = usize(0xFFFFFFFF);  // Interrupt Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS1 register.
//
//*****************************************************************************
pub const NVIC_DIS1_INT_M = usize(0xFFFFFFFF);  // Interrupt Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS2 register.
//
//*****************************************************************************
pub const NVIC_DIS2_INT_M = usize(0xFFFFFFFF);  // Interrupt Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS3 register.
//
//*****************************************************************************
pub const NVIC_DIS3_INT_M = usize(0xFFFFFFFF);  // Interrupt Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND0 register.
//
//*****************************************************************************
pub const NVIC_PEND0_INT_M = usize(0xFFFFFFFF);  // Interrupt Set Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND1 register.
//
//*****************************************************************************
pub const NVIC_PEND1_INT_M = usize(0xFFFFFFFF);  // Interrupt Set Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND2 register.
//
//*****************************************************************************
pub const NVIC_PEND2_INT_M = usize(0xFFFFFFFF);  // Interrupt Set Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND3 register.
//
//*****************************************************************************
pub const NVIC_PEND3_INT_M = usize(0xFFFFFFFF);  // Interrupt Set Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND0 register.
//
//*****************************************************************************
pub const NVIC_UNPEND0_INT_M = usize(0xFFFFFFFF);  // Interrupt Clear Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND1 register.
//
//*****************************************************************************
pub const NVIC_UNPEND1_INT_M = usize(0xFFFFFFFF);  // Interrupt Clear Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND2 register.
//
//*****************************************************************************
pub const NVIC_UNPEND2_INT_M = usize(0xFFFFFFFF);  // Interrupt Clear Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND3 register.
//
//*****************************************************************************
pub const NVIC_UNPEND3_INT_M = usize(0xFFFFFFFF);  // Interrupt Clear Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE0 register.
//
//*****************************************************************************
pub const NVIC_ACTIVE0_INT_M = usize(0xFFFFFFFF);  // Interrupt Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE1 register.
//
//*****************************************************************************
pub const NVIC_ACTIVE1_INT_M = usize(0xFFFFFFFF);  // Interrupt Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE2 register.
//
//*****************************************************************************
pub const NVIC_ACTIVE2_INT_M = usize(0xFFFFFFFF);  // Interrupt Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE3 register.
//
//*****************************************************************************
pub const NVIC_ACTIVE3_INT_M = usize(0xFFFFFFFF);  // Interrupt Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI0 register.
//
//*****************************************************************************
pub const NVIC_PRI0_INT3_M = usize(0xE0000000);  // Interrupt 3 Priority Mask
pub const NVIC_PRI0_INT2_M = usize(0x00E00000);  // Interrupt 2 Priority Mask
pub const NVIC_PRI0_INT1_M = usize(0x0000E000);  // Interrupt 1 Priority Mask
pub const NVIC_PRI0_INT0_M = usize(0x000000E0);  // Interrupt 0 Priority Mask
pub const NVIC_PRI0_INT3_S = usize(29);
pub const NVIC_PRI0_INT2_S = usize(21);
pub const NVIC_PRI0_INT1_S = usize(13);
pub const NVIC_PRI0_INT0_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI1 register.
//
//*****************************************************************************
pub const NVIC_PRI1_INT7_M = usize(0xE0000000);  // Interrupt 7 Priority Mask
pub const NVIC_PRI1_INT6_M = usize(0x00E00000);  // Interrupt 6 Priority Mask
pub const NVIC_PRI1_INT5_M = usize(0x0000E000);  // Interrupt 5 Priority Mask
pub const NVIC_PRI1_INT4_M = usize(0x000000E0);  // Interrupt 4 Priority Mask
pub const NVIC_PRI1_INT7_S = usize(29);
pub const NVIC_PRI1_INT6_S = usize(21);
pub const NVIC_PRI1_INT5_S = usize(13);
pub const NVIC_PRI1_INT4_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI2 register.
//
//*****************************************************************************
pub const NVIC_PRI2_INT11_M = usize(0xE0000000);  // Interrupt 11 Priority Mask
pub const NVIC_PRI2_INT10_M = usize(0x00E00000);  // Interrupt 10 Priority Mask
pub const NVIC_PRI2_INT9_M = usize(0x0000E000);  // Interrupt 9 Priority Mask
pub const NVIC_PRI2_INT8_M = usize(0x000000E0);  // Interrupt 8 Priority Mask
pub const NVIC_PRI2_INT11_S = usize(29);
pub const NVIC_PRI2_INT10_S = usize(21);
pub const NVIC_PRI2_INT9_S = usize(13);
pub const NVIC_PRI2_INT8_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI3 register.
//
//*****************************************************************************
pub const NVIC_PRI3_INT15_M = usize(0xE0000000);  // Interrupt 15 Priority Mask
pub const NVIC_PRI3_INT14_M = usize(0x00E00000);  // Interrupt 14 Priority Mask
pub const NVIC_PRI3_INT13_M = usize(0x0000E000);  // Interrupt 13 Priority Mask
pub const NVIC_PRI3_INT12_M = usize(0x000000E0);  // Interrupt 12 Priority Mask
pub const NVIC_PRI3_INT15_S = usize(29);
pub const NVIC_PRI3_INT14_S = usize(21);
pub const NVIC_PRI3_INT13_S = usize(13);
pub const NVIC_PRI3_INT12_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI4 register.
//
//*****************************************************************************
pub const NVIC_PRI4_INT19_M = usize(0xE0000000);  // Interrupt 19 Priority Mask
pub const NVIC_PRI4_INT18_M = usize(0x00E00000);  // Interrupt 18 Priority Mask
pub const NVIC_PRI4_INT17_M = usize(0x0000E000);  // Interrupt 17 Priority Mask
pub const NVIC_PRI4_INT16_M = usize(0x000000E0);  // Interrupt 16 Priority Mask
pub const NVIC_PRI4_INT19_S = usize(29);
pub const NVIC_PRI4_INT18_S = usize(21);
pub const NVIC_PRI4_INT17_S = usize(13);
pub const NVIC_PRI4_INT16_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI5 register.
//
//*****************************************************************************
pub const NVIC_PRI5_INT23_M = usize(0xE0000000);  // Interrupt 23 Priority Mask
pub const NVIC_PRI5_INT22_M = usize(0x00E00000);  // Interrupt 22 Priority Mask
pub const NVIC_PRI5_INT21_M = usize(0x0000E000);  // Interrupt 21 Priority Mask
pub const NVIC_PRI5_INT20_M = usize(0x000000E0);  // Interrupt 20 Priority Mask
pub const NVIC_PRI5_INT23_S = usize(29);
pub const NVIC_PRI5_INT22_S = usize(21);
pub const NVIC_PRI5_INT21_S = usize(13);
pub const NVIC_PRI5_INT20_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI6 register.
//
//*****************************************************************************
pub const NVIC_PRI6_INT27_M = usize(0xE0000000);  // Interrupt 27 Priority Mask
pub const NVIC_PRI6_INT26_M = usize(0x00E00000);  // Interrupt 26 Priority Mask
pub const NVIC_PRI6_INT25_M = usize(0x0000E000);  // Interrupt 25 Priority Mask
pub const NVIC_PRI6_INT24_M = usize(0x000000E0);  // Interrupt 24 Priority Mask
pub const NVIC_PRI6_INT27_S = usize(29);
pub const NVIC_PRI6_INT26_S = usize(21);
pub const NVIC_PRI6_INT25_S = usize(13);
pub const NVIC_PRI6_INT24_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI7 register.
//
//*****************************************************************************
pub const NVIC_PRI7_INT31_M = usize(0xE0000000);  // Interrupt 31 Priority Mask
pub const NVIC_PRI7_INT30_M = usize(0x00E00000);  // Interrupt 30 Priority Mask
pub const NVIC_PRI7_INT29_M = usize(0x0000E000);  // Interrupt 29 Priority Mask
pub const NVIC_PRI7_INT28_M = usize(0x000000E0);  // Interrupt 28 Priority Mask
pub const NVIC_PRI7_INT31_S = usize(29);
pub const NVIC_PRI7_INT30_S = usize(21);
pub const NVIC_PRI7_INT29_S = usize(13);
pub const NVIC_PRI7_INT28_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI8 register.
//
//*****************************************************************************
pub const NVIC_PRI8_INT35_M = usize(0xE0000000);  // Interrupt 35 Priority Mask
pub const NVIC_PRI8_INT34_M = usize(0x00E00000);  // Interrupt 34 Priority Mask
pub const NVIC_PRI8_INT33_M = usize(0x0000E000);  // Interrupt 33 Priority Mask
pub const NVIC_PRI8_INT32_M = usize(0x000000E0);  // Interrupt 32 Priority Mask
pub const NVIC_PRI8_INT35_S = usize(29);
pub const NVIC_PRI8_INT34_S = usize(21);
pub const NVIC_PRI8_INT33_S = usize(13);
pub const NVIC_PRI8_INT32_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI9 register.
//
//*****************************************************************************
pub const NVIC_PRI9_INT39_M = usize(0xE0000000);  // Interrupt 39 Priority Mask
pub const NVIC_PRI9_INT38_M = usize(0x00E00000);  // Interrupt 38 Priority Mask
pub const NVIC_PRI9_INT37_M = usize(0x0000E000);  // Interrupt 37 Priority Mask
pub const NVIC_PRI9_INT36_M = usize(0x000000E0);  // Interrupt 36 Priority Mask
pub const NVIC_PRI9_INT39_S = usize(29);
pub const NVIC_PRI9_INT38_S = usize(21);
pub const NVIC_PRI9_INT37_S = usize(13);
pub const NVIC_PRI9_INT36_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI10 register.
//
//*****************************************************************************
pub const NVIC_PRI10_INT43_M = usize(0xE0000000);  // Interrupt 43 Priority Mask
pub const NVIC_PRI10_INT42_M = usize(0x00E00000);  // Interrupt 42 Priority Mask
pub const NVIC_PRI10_INT41_M = usize(0x0000E000);  // Interrupt 41 Priority Mask
pub const NVIC_PRI10_INT40_M = usize(0x000000E0);  // Interrupt 40 Priority Mask
pub const NVIC_PRI10_INT43_S = usize(29);
pub const NVIC_PRI10_INT42_S = usize(21);
pub const NVIC_PRI10_INT41_S = usize(13);
pub const NVIC_PRI10_INT40_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI11 register.
//
//*****************************************************************************
pub const NVIC_PRI11_INT47_M = usize(0xE0000000);  // Interrupt 47 Priority Mask
pub const NVIC_PRI11_INT46_M = usize(0x00E00000);  // Interrupt 46 Priority Mask
pub const NVIC_PRI11_INT45_M = usize(0x0000E000);  // Interrupt 45 Priority Mask
pub const NVIC_PRI11_INT44_M = usize(0x000000E0);  // Interrupt 44 Priority Mask
pub const NVIC_PRI11_INT47_S = usize(29);
pub const NVIC_PRI11_INT46_S = usize(21);
pub const NVIC_PRI11_INT45_S = usize(13);
pub const NVIC_PRI11_INT44_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI12 register.
//
//*****************************************************************************
pub const NVIC_PRI12_INT51_M = usize(0xE0000000);  // Interrupt 51 Priority Mask
pub const NVIC_PRI12_INT50_M = usize(0x00E00000);  // Interrupt 50 Priority Mask
pub const NVIC_PRI12_INT49_M = usize(0x0000E000);  // Interrupt 49 Priority Mask
pub const NVIC_PRI12_INT48_M = usize(0x000000E0);  // Interrupt 48 Priority Mask
pub const NVIC_PRI12_INT51_S = usize(29);
pub const NVIC_PRI12_INT50_S = usize(21);
pub const NVIC_PRI12_INT49_S = usize(13);
pub const NVIC_PRI12_INT48_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI13 register.
//
//*****************************************************************************
pub const NVIC_PRI13_INT55_M = usize(0xE0000000);  // Interrupt 55 Priority Mask
pub const NVIC_PRI13_INT54_M = usize(0x00E00000);  // Interrupt 54 Priority Mask
pub const NVIC_PRI13_INT53_M = usize(0x0000E000);  // Interrupt 53 Priority Mask
pub const NVIC_PRI13_INT52_M = usize(0x000000E0);  // Interrupt 52 Priority Mask
pub const NVIC_PRI13_INT55_S = usize(29);
pub const NVIC_PRI13_INT54_S = usize(21);
pub const NVIC_PRI13_INT53_S = usize(13);
pub const NVIC_PRI13_INT52_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI14 register.
//
//*****************************************************************************
pub const NVIC_PRI14_INTD_M = usize(0xE0000000);  // Interrupt 59 Priority Mask
pub const NVIC_PRI14_INTC_M = usize(0x00E00000);  // Interrupt 58 Priority Mask
pub const NVIC_PRI14_INTB_M = usize(0x0000E000);  // Interrupt 57 Priority Mask
pub const NVIC_PRI14_INTA_M = usize(0x000000E0);  // Interrupt 56 Priority Mask
pub const NVIC_PRI14_INTD_S = usize(29);
pub const NVIC_PRI14_INTC_S = usize(21);
pub const NVIC_PRI14_INTB_S = usize(13);
pub const NVIC_PRI14_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI15 register.
//
//*****************************************************************************
pub const NVIC_PRI15_INTD_M = usize(0xE0000000);  // Interrupt 63 Priority Mask
pub const NVIC_PRI15_INTC_M = usize(0x00E00000);  // Interrupt 62 Priority Mask
pub const NVIC_PRI15_INTB_M = usize(0x0000E000);  // Interrupt 61 Priority Mask
pub const NVIC_PRI15_INTA_M = usize(0x000000E0);  // Interrupt 60 Priority Mask
pub const NVIC_PRI15_INTD_S = usize(29);
pub const NVIC_PRI15_INTC_S = usize(21);
pub const NVIC_PRI15_INTB_S = usize(13);
pub const NVIC_PRI15_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI16 register.
//
//*****************************************************************************
pub const NVIC_PRI16_INTD_M = usize(0xE0000000);  // Interrupt 67 Priority Mask
pub const NVIC_PRI16_INTC_M = usize(0x00E00000);  // Interrupt 66 Priority Mask
pub const NVIC_PRI16_INTB_M = usize(0x0000E000);  // Interrupt 65 Priority Mask
pub const NVIC_PRI16_INTA_M = usize(0x000000E0);  // Interrupt 64 Priority Mask
pub const NVIC_PRI16_INTD_S = usize(29);
pub const NVIC_PRI16_INTC_S = usize(21);
pub const NVIC_PRI16_INTB_S = usize(13);
pub const NVIC_PRI16_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI17 register.
//
//*****************************************************************************
pub const NVIC_PRI17_INTD_M = usize(0xE0000000);  // Interrupt 71 Priority Mask
pub const NVIC_PRI17_INTC_M = usize(0x00E00000);  // Interrupt 70 Priority Mask
pub const NVIC_PRI17_INTB_M = usize(0x0000E000);  // Interrupt 69 Priority Mask
pub const NVIC_PRI17_INTA_M = usize(0x000000E0);  // Interrupt 68 Priority Mask
pub const NVIC_PRI17_INTD_S = usize(29);
pub const NVIC_PRI17_INTC_S = usize(21);
pub const NVIC_PRI17_INTB_S = usize(13);
pub const NVIC_PRI17_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI18 register.
//
//*****************************************************************************
pub const NVIC_PRI18_INTD_M = usize(0xE0000000);  // Interrupt 75 Priority Mask
pub const NVIC_PRI18_INTC_M = usize(0x00E00000);  // Interrupt 74 Priority Mask
pub const NVIC_PRI18_INTB_M = usize(0x0000E000);  // Interrupt 73 Priority Mask
pub const NVIC_PRI18_INTA_M = usize(0x000000E0);  // Interrupt 72 Priority Mask
pub const NVIC_PRI18_INTD_S = usize(29);
pub const NVIC_PRI18_INTC_S = usize(21);
pub const NVIC_PRI18_INTB_S = usize(13);
pub const NVIC_PRI18_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI19 register.
//
//*****************************************************************************
pub const NVIC_PRI19_INTD_M = usize(0xE0000000);  // Interrupt 79 Priority Mask
pub const NVIC_PRI19_INTC_M = usize(0x00E00000);  // Interrupt 78 Priority Mask
pub const NVIC_PRI19_INTB_M = usize(0x0000E000);  // Interrupt 77 Priority Mask
pub const NVIC_PRI19_INTA_M = usize(0x000000E0);  // Interrupt 76 Priority Mask
pub const NVIC_PRI19_INTD_S = usize(29);
pub const NVIC_PRI19_INTC_S = usize(21);
pub const NVIC_PRI19_INTB_S = usize(13);
pub const NVIC_PRI19_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI20 register.
//
//*****************************************************************************
pub const NVIC_PRI20_INTD_M = usize(0xE0000000);  // Interrupt 83 Priority Mask
pub const NVIC_PRI20_INTC_M = usize(0x00E00000);  // Interrupt 82 Priority Mask
pub const NVIC_PRI20_INTB_M = usize(0x0000E000);  // Interrupt 81 Priority Mask
pub const NVIC_PRI20_INTA_M = usize(0x000000E0);  // Interrupt 80 Priority Mask
pub const NVIC_PRI20_INTD_S = usize(29);
pub const NVIC_PRI20_INTC_S = usize(21);
pub const NVIC_PRI20_INTB_S = usize(13);
pub const NVIC_PRI20_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI21 register.
//
//*****************************************************************************
pub const NVIC_PRI21_INTD_M = usize(0xE0000000);  // Interrupt 87 Priority Mask
pub const NVIC_PRI21_INTC_M = usize(0x00E00000);  // Interrupt 86 Priority Mask
pub const NVIC_PRI21_INTB_M = usize(0x0000E000);  // Interrupt 85 Priority Mask
pub const NVIC_PRI21_INTA_M = usize(0x000000E0);  // Interrupt 84 Priority Mask
pub const NVIC_PRI21_INTD_S = usize(29);
pub const NVIC_PRI21_INTC_S = usize(21);
pub const NVIC_PRI21_INTB_S = usize(13);
pub const NVIC_PRI21_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI22 register.
//
//*****************************************************************************
pub const NVIC_PRI22_INTD_M = usize(0xE0000000);  // Interrupt 91 Priority Mask
pub const NVIC_PRI22_INTC_M = usize(0x00E00000);  // Interrupt 90 Priority Mask
pub const NVIC_PRI22_INTB_M = usize(0x0000E000);  // Interrupt 89 Priority Mask
pub const NVIC_PRI22_INTA_M = usize(0x000000E0);  // Interrupt 88 Priority Mask
pub const NVIC_PRI22_INTD_S = usize(29);
pub const NVIC_PRI22_INTC_S = usize(21);
pub const NVIC_PRI22_INTB_S = usize(13);
pub const NVIC_PRI22_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI23 register.
//
//*****************************************************************************
pub const NVIC_PRI23_INTD_M = usize(0xE0000000);  // Interrupt 95 Priority Mask
pub const NVIC_PRI23_INTC_M = usize(0x00E00000);  // Interrupt 94 Priority Mask
pub const NVIC_PRI23_INTB_M = usize(0x0000E000);  // Interrupt 93 Priority Mask
pub const NVIC_PRI23_INTA_M = usize(0x000000E0);  // Interrupt 92 Priority Mask
pub const NVIC_PRI23_INTD_S = usize(29);
pub const NVIC_PRI23_INTC_S = usize(21);
pub const NVIC_PRI23_INTB_S = usize(13);
pub const NVIC_PRI23_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI24 register.
//
//*****************************************************************************
pub const NVIC_PRI24_INTD_M = usize(0xE0000000);  // Interrupt 99 Priority Mask
pub const NVIC_PRI24_INTC_M = usize(0x00E00000);  // Interrupt 98 Priority Mask
pub const NVIC_PRI24_INTB_M = usize(0x0000E000);  // Interrupt 97 Priority Mask
pub const NVIC_PRI24_INTA_M = usize(0x000000E0);  // Interrupt 96 Priority Mask
pub const NVIC_PRI24_INTD_S = usize(29);
pub const NVIC_PRI24_INTC_S = usize(21);
pub const NVIC_PRI24_INTB_S = usize(13);
pub const NVIC_PRI24_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI25 register.
//
//*****************************************************************************
pub const NVIC_PRI25_INTD_M = usize(0xE0000000);  // Interrupt 103 Priority Mask
pub const NVIC_PRI25_INTC_M = usize(0x00E00000);  // Interrupt 102 Priority Mask
pub const NVIC_PRI25_INTB_M = usize(0x0000E000);  // Interrupt 101 Priority Mask
pub const NVIC_PRI25_INTA_M = usize(0x000000E0);  // Interrupt 100 Priority Mask
pub const NVIC_PRI25_INTD_S = usize(29);
pub const NVIC_PRI25_INTC_S = usize(21);
pub const NVIC_PRI25_INTB_S = usize(13);
pub const NVIC_PRI25_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI26 register.
//
//*****************************************************************************
pub const NVIC_PRI26_INTD_M = usize(0xE0000000);  // Interrupt 107 Priority Mask
pub const NVIC_PRI26_INTC_M = usize(0x00E00000);  // Interrupt 106 Priority Mask
pub const NVIC_PRI26_INTB_M = usize(0x0000E000);  // Interrupt 105 Priority Mask
pub const NVIC_PRI26_INTA_M = usize(0x000000E0);  // Interrupt 104 Priority Mask
pub const NVIC_PRI26_INTD_S = usize(29);
pub const NVIC_PRI26_INTC_S = usize(21);
pub const NVIC_PRI26_INTB_S = usize(13);
pub const NVIC_PRI26_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI27 register.
//
//*****************************************************************************
pub const NVIC_PRI27_INTD_M = usize(0xE0000000);  // Interrupt 111 Priority Mask
pub const NVIC_PRI27_INTC_M = usize(0x00E00000);  // Interrupt 110 Priority Mask
pub const NVIC_PRI27_INTB_M = usize(0x0000E000);  // Interrupt 109 Priority Mask
pub const NVIC_PRI27_INTA_M = usize(0x000000E0);  // Interrupt 108 Priority Mask
pub const NVIC_PRI27_INTD_S = usize(29);
pub const NVIC_PRI27_INTC_S = usize(21);
pub const NVIC_PRI27_INTB_S = usize(13);
pub const NVIC_PRI27_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI28 register.
//
//*****************************************************************************
pub const NVIC_PRI28_INTD_M = usize(0xE0000000);  // Interrupt 115 Priority Mask
pub const NVIC_PRI28_INTC_M = usize(0x00E00000);  // Interrupt 114 Priority Mask
pub const NVIC_PRI28_INTB_M = usize(0x0000E000);  // Interrupt 113 Priority Mask
pub const NVIC_PRI28_INTA_M = usize(0x000000E0);  // Interrupt 112 Priority Mask
pub const NVIC_PRI28_INTD_S = usize(29);
pub const NVIC_PRI28_INTC_S = usize(21);
pub const NVIC_PRI28_INTB_S = usize(13);
pub const NVIC_PRI28_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CPUID register.
//
//*****************************************************************************
pub const NVIC_CPUID_IMP_M = usize(0xFF000000);  // Implementer Code
pub const NVIC_CPUID_IMP_ARM = usize(0x41000000);  // ARM
pub const NVIC_CPUID_VAR_M = usize(0x00F00000);  // Variant Number
pub const NVIC_CPUID_CON_M = usize(0x000F0000);  // Constant
pub const NVIC_CPUID_PARTNO_M = usize(0x0000FFF0);  // Part Number
pub const NVIC_CPUID_PARTNO_CM4 = usize(0x0000C240);  // Cortex-M4 processor
pub const NVIC_CPUID_REV_M = usize(0x0000000F);  // Revision Number

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_INT_CTRL register.
//
//*****************************************************************************
pub const NVIC_INT_CTRL_NMI_SET = usize(0x80000000);  // NMI Set Pending
pub const NVIC_INT_CTRL_PEND_SV = usize(0x10000000);  // PendSV Set Pending
pub const NVIC_INT_CTRL_UNPEND_SV = usize(0x08000000);  // PendSV Clear Pending
pub const NVIC_INT_CTRL_PENDSTSET = usize(0x04000000);  // SysTick Set Pending
pub const NVIC_INT_CTRL_PENDSTCLR = usize(0x02000000);  // SysTick Clear Pending
pub const NVIC_INT_CTRL_ISR_PRE = usize(0x00800000);  // Debug Interrupt Handling
pub const NVIC_INT_CTRL_ISR_PEND = usize(0x00400000);  // Interrupt Pending
pub const NVIC_INT_CTRL_VEC_PEN_M = usize(0x000FF000);  // Interrupt Pending Vector Number
pub const NVIC_INT_CTRL_VEC_PEN_NMI = usize(0x00002000);  // NMI
pub const NVIC_INT_CTRL_VEC_PEN_HARD = usize(0x00003000);  // Hard fault
pub const NVIC_INT_CTRL_VEC_PEN_MEM = usize(0x00004000);  // Memory management fault
pub const NVIC_INT_CTRL_VEC_PEN_BUS = usize(0x00005000);  // Bus fault
pub const NVIC_INT_CTRL_VEC_PEN_USG = usize(0x00006000);  // Usage fault
pub const NVIC_INT_CTRL_VEC_PEN_SVC = usize(0x0000B000);  // SVCall
pub const NVIC_INT_CTRL_VEC_PEN_PNDSV = usize(0x0000E000);  // PendSV
pub const NVIC_INT_CTRL_VEC_PEN_TICK = usize(0x0000F000);  // SysTick
pub const NVIC_INT_CTRL_RET_BASE = usize(0x00000800);  // Return to Base
pub const NVIC_INT_CTRL_VEC_ACT_M = usize(0x000000FF);  // Interrupt Pending Vector Number
pub const NVIC_INT_CTRL_VEC_ACT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_VTABLE register.
//
//*****************************************************************************
pub const NVIC_VTABLE_OFFSET_M = usize(0xFFFFFC00);  // Vector Table Offset
pub const NVIC_VTABLE_OFFSET_S = usize(10);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_APINT register.
//
//*****************************************************************************
pub const NVIC_APINT_VECTKEY_M = usize(0xFFFF0000);  // Register Key
pub const NVIC_APINT_VECTKEY = usize(0x05FA0000);  // Vector key
pub const NVIC_APINT_ENDIANESS = usize(0x00008000);  // Data Endianess
pub const NVIC_APINT_PRIGROUP_M = usize(0x00000700);  // Interrupt Priority Grouping
pub const NVIC_APINT_PRIGROUP_7_1 = usize(0x00000000);  // Priority group 7.1 split
pub const NVIC_APINT_PRIGROUP_6_2 = usize(0x00000100);  // Priority group 6.2 split
pub const NVIC_APINT_PRIGROUP_5_3 = usize(0x00000200);  // Priority group 5.3 split
pub const NVIC_APINT_PRIGROUP_4_4 = usize(0x00000300);  // Priority group 4.4 split
pub const NVIC_APINT_PRIGROUP_3_5 = usize(0x00000400);  // Priority group 3.5 split
pub const NVIC_APINT_PRIGROUP_2_6 = usize(0x00000500);  // Priority group 2.6 split
pub const NVIC_APINT_PRIGROUP_1_7 = usize(0x00000600);  // Priority group 1.7 split
pub const NVIC_APINT_PRIGROUP_0_8 = usize(0x00000700);  // Priority group 0.8 split
pub const NVIC_APINT_SYSRESETREQ = usize(0x00000004);  // System Reset Request
pub const NVIC_APINT_VECT_CLR_ACT = usize(0x00000002);  // Clear Active NMI / Fault
pub const NVIC_APINT_VECT_RESET = usize(0x00000001);  // System Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_CTRL register.
//
//*****************************************************************************
pub const NVIC_SYS_CTRL_SEVONPEND = usize(0x00000010);  // Wake Up on Pending
pub const NVIC_SYS_CTRL_SLEEPDEEP = usize(0x00000004);  // Deep Sleep Enable
pub const NVIC_SYS_CTRL_SLEEPEXIT = usize(0x00000002);  // Sleep on ISR Exit

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CFG_CTRL register.
//
//*****************************************************************************
pub const NVIC_CFG_CTRL_STKALIGN = usize(0x00000200);  // Stack Alignment on Exception
                                            // Entry
pub const NVIC_CFG_CTRL_BFHFNMIGN = usize(0x00000100);  // Ignore Bus Fault in NMI and
                                            // Fault
pub const NVIC_CFG_CTRL_DIV0 = usize(0x00000010);  // Trap on Divide by 0
pub const NVIC_CFG_CTRL_UNALIGNED = usize(0x00000008);  // Trap on Unaligned Access
pub const NVIC_CFG_CTRL_MAIN_PEND = usize(0x00000002);  // Allow Main Interrupt Trigger
pub const NVIC_CFG_CTRL_BASE_THR = usize(0x00000001);  // Thread State Control

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI1 register.
//
//*****************************************************************************
pub const NVIC_SYS_PRI1_USAGE_M = usize(0x00E00000);  // Usage Fault Priority
pub const NVIC_SYS_PRI1_BUS_M = usize(0x0000E000);  // Bus Fault Priority
pub const NVIC_SYS_PRI1_MEM_M = usize(0x000000E0);  // Memory Management Fault Priority
pub const NVIC_SYS_PRI1_USAGE_S = usize(21);
pub const NVIC_SYS_PRI1_BUS_S = usize(13);
pub const NVIC_SYS_PRI1_MEM_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI2 register.
//
//*****************************************************************************
pub const NVIC_SYS_PRI2_SVC_M = usize(0xE0000000);  // SVCall Priority
pub const NVIC_SYS_PRI2_SVC_S = usize(29);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI3 register.
//
//*****************************************************************************
pub const NVIC_SYS_PRI3_TICK_M = usize(0xE0000000);  // SysTick Exception Priority
pub const NVIC_SYS_PRI3_PENDSV_M = usize(0x00E00000);  // PendSV Priority
pub const NVIC_SYS_PRI3_DEBUG_M = usize(0x000000E0);  // Debug Priority
pub const NVIC_SYS_PRI3_TICK_S = usize(29);
pub const NVIC_SYS_PRI3_PENDSV_S = usize(21);
pub const NVIC_SYS_PRI3_DEBUG_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_HND_CTRL
// register.
//
//*****************************************************************************
pub const NVIC_SYS_HND_CTRL_USAGE = usize(0x00040000);  // Usage Fault Enable
pub const NVIC_SYS_HND_CTRL_BUS = usize(0x00020000);  // Bus Fault Enable
pub const NVIC_SYS_HND_CTRL_MEM = usize(0x00010000);  // Memory Management Fault Enable
pub const NVIC_SYS_HND_CTRL_SVC = usize(0x00008000);  // SVC Call Pending
pub const NVIC_SYS_HND_CTRL_BUSP = usize(0x00004000);  // Bus Fault Pending
pub const NVIC_SYS_HND_CTRL_MEMP = usize(0x00002000);  // Memory Management Fault Pending
pub const NVIC_SYS_HND_CTRL_USAGEP = usize(0x00001000);  // Usage Fault Pending
pub const NVIC_SYS_HND_CTRL_TICK = usize(0x00000800);  // SysTick Exception Active
pub const NVIC_SYS_HND_CTRL_PNDSV = usize(0x00000400);  // PendSV Exception Active
pub const NVIC_SYS_HND_CTRL_MON = usize(0x00000100);  // Debug Monitor Active
pub const NVIC_SYS_HND_CTRL_SVCA = usize(0x00000080);  // SVC Call Active
pub const NVIC_SYS_HND_CTRL_USGA = usize(0x00000008);  // Usage Fault Active
pub const NVIC_SYS_HND_CTRL_BUSA = usize(0x00000002);  // Bus Fault Active
pub const NVIC_SYS_HND_CTRL_MEMA = usize(0x00000001);  // Memory Management Fault Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FAULT_STAT
// register.
//
//*****************************************************************************
pub const NVIC_FAULT_STAT_DIV0 = usize(0x02000000);  // Divide-by-Zero Usage Fault
pub const NVIC_FAULT_STAT_UNALIGN = usize(0x01000000);  // Unaligned Access Usage Fault
pub const NVIC_FAULT_STAT_NOCP = usize(0x00080000);  // No Coprocessor Usage Fault
pub const NVIC_FAULT_STAT_INVPC = usize(0x00040000);  // Invalid PC Load Usage Fault
pub const NVIC_FAULT_STAT_INVSTAT = usize(0x00020000);  // Invalid State Usage Fault
pub const NVIC_FAULT_STAT_UNDEF = usize(0x00010000);  // Undefined Instruction Usage
                                            // Fault
pub const NVIC_FAULT_STAT_BFARV = usize(0x00008000);  // Bus Fault Address Register Valid
pub const NVIC_FAULT_STAT_BLSPERR = usize(0x00002000);  // Bus Fault on Floating-Point Lazy
                                            // State Preservation
pub const NVIC_FAULT_STAT_BSTKE = usize(0x00001000);  // Stack Bus Fault
pub const NVIC_FAULT_STAT_BUSTKE = usize(0x00000800);  // Unstack Bus Fault
pub const NVIC_FAULT_STAT_IMPRE = usize(0x00000400);  // Imprecise Data Bus Error
pub const NVIC_FAULT_STAT_PRECISE = usize(0x00000200);  // Precise Data Bus Error
pub const NVIC_FAULT_STAT_IBUS = usize(0x00000100);  // Instruction Bus Error
pub const NVIC_FAULT_STAT_MMARV = usize(0x00000080);  // Memory Management Fault Address
                                            // Register Valid
pub const NVIC_FAULT_STAT_MLSPERR = usize(0x00000020);  // Memory Management Fault on
                                            // Floating-Point Lazy State
                                            // Preservation
pub const NVIC_FAULT_STAT_MSTKE = usize(0x00000010);  // Stack Access Violation
pub const NVIC_FAULT_STAT_MUSTKE = usize(0x00000008);  // Unstack Access Violation
pub const NVIC_FAULT_STAT_DERR = usize(0x00000002);  // Data Access Violation
pub const NVIC_FAULT_STAT_IERR = usize(0x00000001);  // Instruction Access Violation

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_HFAULT_STAT
// register.
//
//*****************************************************************************
pub const NVIC_HFAULT_STAT_DBG = usize(0x80000000);  // Debug Event
pub const NVIC_HFAULT_STAT_FORCED = usize(0x40000000);  // Forced Hard Fault
pub const NVIC_HFAULT_STAT_VECT = usize(0x00000002);  // Vector Table Read Fault

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DEBUG_STAT
// register.
//
//*****************************************************************************
pub const NVIC_DEBUG_STAT_EXTRNL = usize(0x00000010);  // EDBGRQ asserted
pub const NVIC_DEBUG_STAT_VCATCH = usize(0x00000008);  // Vector catch
pub const NVIC_DEBUG_STAT_DWTTRAP = usize(0x00000004);  // DWT match
pub const NVIC_DEBUG_STAT_BKPT = usize(0x00000002);  // Breakpoint instruction
pub const NVIC_DEBUG_STAT_HALTED = usize(0x00000001);  // Halt request

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MM_ADDR register.
//
//*****************************************************************************
pub const NVIC_MM_ADDR_M = usize(0xFFFFFFFF);  // Fault Address
pub const NVIC_MM_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FAULT_ADDR
// register.
//
//*****************************************************************************
pub const NVIC_FAULT_ADDR_M = usize(0xFFFFFFFF);  // Fault Address
pub const NVIC_FAULT_ADDR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CPAC register.
//
//*****************************************************************************
pub const NVIC_CPAC_CP11_M = usize(0x00C00000);  // CP11 Coprocessor Access
                                            // Privilege
pub const NVIC_CPAC_CP11_DIS = usize(0x00000000);  // Access Denied
pub const NVIC_CPAC_CP11_PRIV = usize(0x00400000);  // Privileged Access Only
pub const NVIC_CPAC_CP11_FULL = usize(0x00C00000);  // Full Access
pub const NVIC_CPAC_CP10_M = usize(0x00300000);  // CP10 Coprocessor Access
                                            // Privilege
pub const NVIC_CPAC_CP10_DIS = usize(0x00000000);  // Access Denied
pub const NVIC_CPAC_CP10_PRIV = usize(0x00100000);  // Privileged Access Only
pub const NVIC_CPAC_CP10_FULL = usize(0x00300000);  // Full Access

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_TYPE register.
//
//*****************************************************************************
pub const NVIC_MPU_TYPE_IREGION_M = usize(0x00FF0000);  // Number of I Regions
pub const NVIC_MPU_TYPE_DREGION_M = usize(0x0000FF00);  // Number of D Regions
pub const NVIC_MPU_TYPE_SEPARATE = usize(0x00000001);  // Separate or Unified MPU
pub const NVIC_MPU_TYPE_IREGION_S = usize(16);
pub const NVIC_MPU_TYPE_DREGION_S = usize(8);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_CTRL register.
//
//*****************************************************************************
pub const NVIC_MPU_CTRL_PRIVDEFEN = usize(0x00000004);  // MPU Default Region
pub const NVIC_MPU_CTRL_HFNMIENA = usize(0x00000002);  // MPU Enabled During Faults
pub const NVIC_MPU_CTRL_ENABLE = usize(0x00000001);  // MPU Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_NUMBER
// register.
//
//*****************************************************************************
pub const NVIC_MPU_NUMBER_M = usize(0x00000007);  // MPU Region to Access
pub const NVIC_MPU_NUMBER_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE register.
//
//*****************************************************************************
pub const NVIC_MPU_BASE_ADDR_M = usize(0xFFFFFFE0);  // Base Address Mask
pub const NVIC_MPU_BASE_VALID = usize(0x00000010);  // Region Number Valid
pub const NVIC_MPU_BASE_REGION_M = usize(0x00000007);  // Region Number
pub const NVIC_MPU_BASE_ADDR_S = usize(5);
pub const NVIC_MPU_BASE_REGION_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR register.
//
//*****************************************************************************
pub const NVIC_MPU_ATTR_XN = usize(0x10000000);  // Instruction Access Disable
pub const NVIC_MPU_ATTR_AP_M = usize(0x07000000);  // Access Privilege
pub const NVIC_MPU_ATTR_TEX_M = usize(0x00380000);  // Type Extension Mask
pub const NVIC_MPU_ATTR_SHAREABLE = usize(0x00040000);  // Shareable
pub const NVIC_MPU_ATTR_CACHEABLE = usize(0x00020000);  // Cacheable
pub const NVIC_MPU_ATTR_BUFFRABLE = usize(0x00010000);  // Bufferable
pub const NVIC_MPU_ATTR_SRD_M = usize(0x0000FF00);  // Subregion Disable Bits
pub const NVIC_MPU_ATTR_SIZE_M = usize(0x0000003E);  // Region Size Mask
pub const NVIC_MPU_ATTR_ENABLE = usize(0x00000001);  // Region Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE1 register.
//
//*****************************************************************************
pub const NVIC_MPU_BASE1_ADDR_M = usize(0xFFFFFFE0);  // Base Address Mask
pub const NVIC_MPU_BASE1_VALID = usize(0x00000010);  // Region Number Valid
pub const NVIC_MPU_BASE1_REGION_M = usize(0x00000007);  // Region Number
pub const NVIC_MPU_BASE1_ADDR_S = usize(5);
pub const NVIC_MPU_BASE1_REGION_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR1 register.
//
//*****************************************************************************
pub const NVIC_MPU_ATTR1_XN = usize(0x10000000);  // Instruction Access Disable
pub const NVIC_MPU_ATTR1_AP_M = usize(0x07000000);  // Access Privilege
pub const NVIC_MPU_ATTR1_TEX_M = usize(0x00380000);  // Type Extension Mask
pub const NVIC_MPU_ATTR1_SHAREABLE = usize(0x00040000);  // Shareable
pub const NVIC_MPU_ATTR1_CACHEABLE = usize(0x00020000);  // Cacheable
pub const NVIC_MPU_ATTR1_BUFFRABLE = usize(0x00010000);  // Bufferable
pub const NVIC_MPU_ATTR1_SRD_M = usize(0x0000FF00);  // Subregion Disable Bits
pub const NVIC_MPU_ATTR1_SIZE_M = usize(0x0000003E);  // Region Size Mask
pub const NVIC_MPU_ATTR1_ENABLE = usize(0x00000001);  // Region Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE2 register.
//
//*****************************************************************************
pub const NVIC_MPU_BASE2_ADDR_M = usize(0xFFFFFFE0);  // Base Address Mask
pub const NVIC_MPU_BASE2_VALID = usize(0x00000010);  // Region Number Valid
pub const NVIC_MPU_BASE2_REGION_M = usize(0x00000007);  // Region Number
pub const NVIC_MPU_BASE2_ADDR_S = usize(5);
pub const NVIC_MPU_BASE2_REGION_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR2 register.
//
//*****************************************************************************
pub const NVIC_MPU_ATTR2_XN = usize(0x10000000);  // Instruction Access Disable
pub const NVIC_MPU_ATTR2_AP_M = usize(0x07000000);  // Access Privilege
pub const NVIC_MPU_ATTR2_TEX_M = usize(0x00380000);  // Type Extension Mask
pub const NVIC_MPU_ATTR2_SHAREABLE = usize(0x00040000);  // Shareable
pub const NVIC_MPU_ATTR2_CACHEABLE = usize(0x00020000);  // Cacheable
pub const NVIC_MPU_ATTR2_BUFFRABLE = usize(0x00010000);  // Bufferable
pub const NVIC_MPU_ATTR2_SRD_M = usize(0x0000FF00);  // Subregion Disable Bits
pub const NVIC_MPU_ATTR2_SIZE_M = usize(0x0000003E);  // Region Size Mask
pub const NVIC_MPU_ATTR2_ENABLE = usize(0x00000001);  // Region Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE3 register.
//
//*****************************************************************************
pub const NVIC_MPU_BASE3_ADDR_M = usize(0xFFFFFFE0);  // Base Address Mask
pub const NVIC_MPU_BASE3_VALID = usize(0x00000010);  // Region Number Valid
pub const NVIC_MPU_BASE3_REGION_M = usize(0x00000007);  // Region Number
pub const NVIC_MPU_BASE3_ADDR_S = usize(5);
pub const NVIC_MPU_BASE3_REGION_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR3 register.
//
//*****************************************************************************
pub const NVIC_MPU_ATTR3_XN = usize(0x10000000);  // Instruction Access Disable
pub const NVIC_MPU_ATTR3_AP_M = usize(0x07000000);  // Access Privilege
pub const NVIC_MPU_ATTR3_TEX_M = usize(0x00380000);  // Type Extension Mask
pub const NVIC_MPU_ATTR3_SHAREABLE = usize(0x00040000);  // Shareable
pub const NVIC_MPU_ATTR3_CACHEABLE = usize(0x00020000);  // Cacheable
pub const NVIC_MPU_ATTR3_BUFFRABLE = usize(0x00010000);  // Bufferable
pub const NVIC_MPU_ATTR3_SRD_M = usize(0x0000FF00);  // Subregion Disable Bits
pub const NVIC_MPU_ATTR3_SIZE_M = usize(0x0000003E);  // Region Size Mask
pub const NVIC_MPU_ATTR3_ENABLE = usize(0x00000001);  // Region Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_CTRL register.
//
//*****************************************************************************
pub const NVIC_DBG_CTRL_DBGKEY_M = usize(0xFFFF0000);  // Debug key mask
pub const NVIC_DBG_CTRL_DBGKEY = usize(0xA05F0000);  // Debug key
pub const NVIC_DBG_CTRL_S_RESET_ST = usize(0x02000000);  // Core has reset since last read
pub const NVIC_DBG_CTRL_S_RETIRE_ST = usize(0x01000000);  // Core has executed insruction
                                            // since last read
pub const NVIC_DBG_CTRL_S_LOCKUP = usize(0x00080000);  // Core is locked up
pub const NVIC_DBG_CTRL_S_SLEEP = usize(0x00040000);  // Core is sleeping
pub const NVIC_DBG_CTRL_S_HALT = usize(0x00020000);  // Core status on halt
pub const NVIC_DBG_CTRL_S_REGRDY = usize(0x00010000);  // Register read/write available
pub const NVIC_DBG_CTRL_C_SNAPSTALL = usize(0x00000020);  // Breaks a stalled load/store
pub const NVIC_DBG_CTRL_C_MASKINT = usize(0x00000008);  // Mask interrupts when stepping
pub const NVIC_DBG_CTRL_C_STEP = usize(0x00000004);  // Step the core
pub const NVIC_DBG_CTRL_C_HALT = usize(0x00000002);  // Halt the core
pub const NVIC_DBG_CTRL_C_DEBUGEN = usize(0x00000001);  // Enable debug

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_XFER register.
//
//*****************************************************************************
pub const NVIC_DBG_XFER_REG_WNR = usize(0x00010000);  // Write or not read
pub const NVIC_DBG_XFER_REG_SEL_M = usize(0x0000001F);  // Register
pub const NVIC_DBG_XFER_REG_R0 = usize(0x00000000);  // Register R0
pub const NVIC_DBG_XFER_REG_R1 = usize(0x00000001);  // Register R1
pub const NVIC_DBG_XFER_REG_R2 = usize(0x00000002);  // Register R2
pub const NVIC_DBG_XFER_REG_R3 = usize(0x00000003);  // Register R3
pub const NVIC_DBG_XFER_REG_R4 = usize(0x00000004);  // Register R4
pub const NVIC_DBG_XFER_REG_R5 = usize(0x00000005);  // Register R5
pub const NVIC_DBG_XFER_REG_R6 = usize(0x00000006);  // Register R6
pub const NVIC_DBG_XFER_REG_R7 = usize(0x00000007);  // Register R7
pub const NVIC_DBG_XFER_REG_R8 = usize(0x00000008);  // Register R8
pub const NVIC_DBG_XFER_REG_R9 = usize(0x00000009);  // Register R9
pub const NVIC_DBG_XFER_REG_R10 = usize(0x0000000A);  // Register R10
pub const NVIC_DBG_XFER_REG_R11 = usize(0x0000000B);  // Register R11
pub const NVIC_DBG_XFER_REG_R12 = usize(0x0000000C);  // Register R12
pub const NVIC_DBG_XFER_REG_R13 = usize(0x0000000D);  // Register R13
pub const NVIC_DBG_XFER_REG_R14 = usize(0x0000000E);  // Register R14
pub const NVIC_DBG_XFER_REG_R15 = usize(0x0000000F);  // Register R15
pub const NVIC_DBG_XFER_REG_FLAGS = usize(0x00000010);  // xPSR/Flags register
pub const NVIC_DBG_XFER_REG_MSP = usize(0x00000011);  // Main SP
pub const NVIC_DBG_XFER_REG_PSP = usize(0x00000012);  // Process SP
pub const NVIC_DBG_XFER_REG_DSP = usize(0x00000013);  // Deep SP
pub const NVIC_DBG_XFER_REG_CFBP = usize(0x00000014);  // Control/Fault/BasePri/PriMask

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_DATA register.
//
//*****************************************************************************
pub const NVIC_DBG_DATA_M = usize(0xFFFFFFFF);  // Data temporary cache
pub const NVIC_DBG_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_INT register.
//
//*****************************************************************************
pub const NVIC_DBG_INT_HARDERR = usize(0x00000400);  // Debug trap on hard fault
pub const NVIC_DBG_INT_INTERR = usize(0x00000200);  // Debug trap on interrupt errors
pub const NVIC_DBG_INT_BUSERR = usize(0x00000100);  // Debug trap on bus error
pub const NVIC_DBG_INT_STATERR = usize(0x00000080);  // Debug trap on usage fault state
pub const NVIC_DBG_INT_CHKERR = usize(0x00000040);  // Debug trap on usage fault check
pub const NVIC_DBG_INT_NOCPERR = usize(0x00000020);  // Debug trap on coprocessor error
pub const NVIC_DBG_INT_MMERR = usize(0x00000010);  // Debug trap on mem manage fault
pub const NVIC_DBG_INT_RESET = usize(0x00000008);  // Core reset status
pub const NVIC_DBG_INT_RSTPENDCLR = usize(0x00000004);  // Clear pending core reset
pub const NVIC_DBG_INT_RSTPENDING = usize(0x00000002);  // Core reset is pending
pub const NVIC_DBG_INT_RSTVCATCH = usize(0x00000001);  // Reset vector catch

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SW_TRIG register.
//
//*****************************************************************************
pub const NVIC_SW_TRIG_INTID_M = usize(0x000000FF);  // Interrupt ID
pub const NVIC_SW_TRIG_INTID_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FPCC register.
//
//*****************************************************************************
pub const NVIC_FPCC_ASPEN = usize(0x80000000);  // Automatic State Preservation
                                            // Enable
pub const NVIC_FPCC_LSPEN = usize(0x40000000);  // Lazy State Preservation Enable
pub const NVIC_FPCC_MONRDY = usize(0x00000100);  // Monitor Ready
pub const NVIC_FPCC_BFRDY = usize(0x00000040);  // Bus Fault Ready
pub const NVIC_FPCC_MMRDY = usize(0x00000020);  // Memory Management Fault Ready
pub const NVIC_FPCC_HFRDY = usize(0x00000010);  // Hard Fault Ready
pub const NVIC_FPCC_THREAD = usize(0x00000008);  // Thread Mode
pub const NVIC_FPCC_USER = usize(0x00000002);  // User Privilege Level
pub const NVIC_FPCC_LSPACT = usize(0x00000001);  // Lazy State Preservation Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FPCA register.
//
//*****************************************************************************
pub const NVIC_FPCA_ADDRESS_M = usize(0xFFFFFFF8);  // Address
pub const NVIC_FPCA_ADDRESS_S = usize(3);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FPDSC register.
//
//*****************************************************************************
pub const NVIC_FPDSC_AHP = usize(0x04000000);  // AHP Bit Default
pub const NVIC_FPDSC_DN = usize(0x02000000);  // DN Bit Default
pub const NVIC_FPDSC_FZ = usize(0x01000000);  // FZ Bit Default
pub const NVIC_FPDSC_RMODE_M = usize(0x00C00000);  // RMODE Bit Default
pub const NVIC_FPDSC_RMODE_RN = usize(0x00000000);  // Round to Nearest (RN) mode
pub const NVIC_FPDSC_RMODE_RP = usize(0x00400000);  // Round towards Plus Infinity (RP)
                                            // mode
pub const NVIC_FPDSC_RMODE_RM = usize(0x00800000);  // Round towards Minus Infinity
                                            // (RM) mode
pub const NVIC_FPDSC_RMODE_RZ = usize(0x00C00000);  // Round towards Zero (RZ) mode

//*****************************************************************************
//
// The following definitions are deprecated.
//
//*****************************************************************************

