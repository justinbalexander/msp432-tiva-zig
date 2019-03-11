//*****************************************************************************
//
// tm4c123fh6pm.h - TM4C123FH6PM Register Definitions
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
pub const INT_PWM0_FAULT = usize(25);          // PWM0 Fault
pub const INT_PWM0_0 = usize(26);          // PWM0 Generator 0
pub const INT_PWM0_1 = usize(27);          // PWM0 Generator 1
pub const INT_PWM0_2 = usize(28);          // PWM0 Generator 2
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
pub const INT_SYSCTL = usize(44);          // System Control
pub const INT_FLASH = usize(45);          // Flash Memory Control and EEPROM
                                            // Control
pub const INT_GPIOF = usize(46);          // GPIO Port F
pub const INT_GPIOG = usize(47);          // GPIO Port G
pub const INT_UART2 = usize(49);          // UART2
pub const INT_SSI1 = usize(50);          // SSI1
pub const INT_TIMER3A = usize(51);          // 16/32-Bit Timer 3A
pub const INT_TIMER3B = usize(52);          // Timer 3B
pub const INT_I2C1 = usize(53);          // I2C1
pub const INT_QEI1 = usize(54);          // QEI1
pub const INT_CAN0 = usize(55);          // CAN0
pub const INT_CAN1 = usize(56);          // CAN1
pub const INT_USB0 = usize(60);          // USB
pub const INT_PWM0_3 = usize(61);          // PWM Generator 3
pub const INT_UDMA = usize(62);          // uDMA Software
pub const INT_UDMAERR = usize(63);          // uDMA Error
pub const INT_ADC1SS0 = usize(64);          // ADC1 Sequence 0
pub const INT_ADC1SS1 = usize(65);          // ADC1 Sequence 1
pub const INT_ADC1SS2 = usize(66);          // ADC1 Sequence 2
pub const INT_ADC1SS3 = usize(67);          // ADC1 Sequence 3
pub const INT_SSI2 = usize(73);          // SSI2
pub const INT_SSI3 = usize(74);          // SSI3
pub const INT_UART3 = usize(75);          // UART3
pub const INT_UART4 = usize(76);          // UART4
pub const INT_UART5 = usize(77);          // UART5
pub const INT_UART6 = usize(78);          // UART6
pub const INT_UART7 = usize(79);          // UART7
pub const INT_I2C2 = usize(84);          // I2C2
pub const INT_I2C3 = usize(85);          // I2C3
pub const INT_TIMER4A = usize(86);          // 16/32-Bit Timer 4A
pub const INT_TIMER4B = usize(87);          // 16/32-Bit Timer 4B
pub const INT_TIMER5A = usize(108);         // 16/32-Bit Timer 5A
pub const INT_TIMER5B = usize(109);         // 16/32-Bit Timer 5B
pub const INT_WTIMER0A = usize(110);         // 32/64-Bit Timer 0A
pub const INT_WTIMER0B = usize(111);         // 32/64-Bit Timer 0B
pub const INT_WTIMER1A = usize(112);         // 32/64-Bit Timer 1A
pub const INT_WTIMER1B = usize(113);         // 32/64-Bit Timer 1B
pub const INT_WTIMER2A = usize(114);         // 32/64-Bit Timer 2A
pub const INT_WTIMER2B = usize(115);         // 32/64-Bit Timer 2B
pub const INT_WTIMER3A = usize(116);         // 32/64-Bit Timer 3A
pub const INT_WTIMER3B = usize(117);         // 32/64-Bit Timer 3B
pub const INT_WTIMER4A = usize(118);         // 32/64-Bit Timer 4A
pub const INT_WTIMER4B = usize(119);         // 32/64-Bit Timer 4B
pub const INT_WTIMER5A = usize(120);         // 32/64-Bit Timer 5A
pub const INT_WTIMER5B = usize(121);         // 32/64-Bit Timer 5B
pub const INT_SYSEXC = usize(122);         // System Exception (imprecise)
pub const INT_I2C4 = usize(125);         // I2C4
pub const INT_I2C5 = usize(126);         // I2C5
pub const INT_PWM1_0 = usize(150);         // PWM1 Generator 0
pub const INT_PWM1_1 = usize(151);         // PWM1 Generator 1
pub const INT_PWM1_2 = usize(152);         // PWM1 Generator 2
pub const INT_PWM1_3 = usize(153);         // PWM1 Generator 3
pub const INT_PWM1_FAULT = usize(154);         // PWM1 Fault

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
// GPIO registers (PORTA)
//
//*****************************************************************************
pub const GPIO_PORTA_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40004000);
pub const GPIO_PORTA_DATA_R = @intToPtr(*volatile u32, 0x400043FC);
pub const GPIO_PORTA_DIR_R = @intToPtr(*volatile u32, 0x40004400);
pub const GPIO_PORTA_IS_R = @intToPtr(*volatile u32, 0x40004404);
pub const GPIO_PORTA_IBE_R = @intToPtr(*volatile u32, 0x40004408);
pub const GPIO_PORTA_IEV_R = @intToPtr(*volatile u32, 0x4000440C);
pub const GPIO_PORTA_IM_R = @intToPtr(*volatile u32, 0x40004410);
pub const GPIO_PORTA_RIS_R = @intToPtr(*volatile u32, 0x40004414);
pub const GPIO_PORTA_MIS_R = @intToPtr(*volatile u32, 0x40004418);
pub const GPIO_PORTA_ICR_R = @intToPtr(*volatile u32, 0x4000441C);
pub const GPIO_PORTA_AFSEL_R = @intToPtr(*volatile u32, 0x40004420);
pub const GPIO_PORTA_DR2R_R = @intToPtr(*volatile u32, 0x40004500);
pub const GPIO_PORTA_DR4R_R = @intToPtr(*volatile u32, 0x40004504);
pub const GPIO_PORTA_DR8R_R = @intToPtr(*volatile u32, 0x40004508);
pub const GPIO_PORTA_ODR_R = @intToPtr(*volatile u32, 0x4000450C);
pub const GPIO_PORTA_PUR_R = @intToPtr(*volatile u32, 0x40004510);
pub const GPIO_PORTA_PDR_R = @intToPtr(*volatile u32, 0x40004514);
pub const GPIO_PORTA_SLR_R = @intToPtr(*volatile u32, 0x40004518);
pub const GPIO_PORTA_DEN_R = @intToPtr(*volatile u32, 0x4000451C);
pub const GPIO_PORTA_LOCK_R = @intToPtr(*volatile u32, 0x40004520);
pub const GPIO_PORTA_CR_R = @intToPtr(*volatile u32, 0x40004524);
pub const GPIO_PORTA_AMSEL_R = @intToPtr(*volatile u32, 0x40004528);
pub const GPIO_PORTA_PCTL_R = @intToPtr(*volatile u32, 0x4000452C);
pub const GPIO_PORTA_ADCCTL_R = @intToPtr(*volatile u32, 0x40004530);
pub const GPIO_PORTA_DMACTL_R = @intToPtr(*volatile u32, 0x40004534);

//*****************************************************************************
//
// GPIO registers (PORTB)
//
//*****************************************************************************
pub const GPIO_PORTB_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40005000);
pub const GPIO_PORTB_DATA_R = @intToPtr(*volatile u32, 0x400053FC);
pub const GPIO_PORTB_DIR_R = @intToPtr(*volatile u32, 0x40005400);
pub const GPIO_PORTB_IS_R = @intToPtr(*volatile u32, 0x40005404);
pub const GPIO_PORTB_IBE_R = @intToPtr(*volatile u32, 0x40005408);
pub const GPIO_PORTB_IEV_R = @intToPtr(*volatile u32, 0x4000540C);
pub const GPIO_PORTB_IM_R = @intToPtr(*volatile u32, 0x40005410);
pub const GPIO_PORTB_RIS_R = @intToPtr(*volatile u32, 0x40005414);
pub const GPIO_PORTB_MIS_R = @intToPtr(*volatile u32, 0x40005418);
pub const GPIO_PORTB_ICR_R = @intToPtr(*volatile u32, 0x4000541C);
pub const GPIO_PORTB_AFSEL_R = @intToPtr(*volatile u32, 0x40005420);
pub const GPIO_PORTB_DR2R_R = @intToPtr(*volatile u32, 0x40005500);
pub const GPIO_PORTB_DR4R_R = @intToPtr(*volatile u32, 0x40005504);
pub const GPIO_PORTB_DR8R_R = @intToPtr(*volatile u32, 0x40005508);
pub const GPIO_PORTB_ODR_R = @intToPtr(*volatile u32, 0x4000550C);
pub const GPIO_PORTB_PUR_R = @intToPtr(*volatile u32, 0x40005510);
pub const GPIO_PORTB_PDR_R = @intToPtr(*volatile u32, 0x40005514);
pub const GPIO_PORTB_SLR_R = @intToPtr(*volatile u32, 0x40005518);
pub const GPIO_PORTB_DEN_R = @intToPtr(*volatile u32, 0x4000551C);
pub const GPIO_PORTB_LOCK_R = @intToPtr(*volatile u32, 0x40005520);
pub const GPIO_PORTB_CR_R = @intToPtr(*volatile u32, 0x40005524);
pub const GPIO_PORTB_AMSEL_R = @intToPtr(*volatile u32, 0x40005528);
pub const GPIO_PORTB_PCTL_R = @intToPtr(*volatile u32, 0x4000552C);
pub const GPIO_PORTB_ADCCTL_R = @intToPtr(*volatile u32, 0x40005530);
pub const GPIO_PORTB_DMACTL_R = @intToPtr(*volatile u32, 0x40005534);

//*****************************************************************************
//
// GPIO registers (PORTC)
//
//*****************************************************************************
pub const GPIO_PORTC_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40006000);
pub const GPIO_PORTC_DATA_R = @intToPtr(*volatile u32, 0x400063FC);
pub const GPIO_PORTC_DIR_R = @intToPtr(*volatile u32, 0x40006400);
pub const GPIO_PORTC_IS_R = @intToPtr(*volatile u32, 0x40006404);
pub const GPIO_PORTC_IBE_R = @intToPtr(*volatile u32, 0x40006408);
pub const GPIO_PORTC_IEV_R = @intToPtr(*volatile u32, 0x4000640C);
pub const GPIO_PORTC_IM_R = @intToPtr(*volatile u32, 0x40006410);
pub const GPIO_PORTC_RIS_R = @intToPtr(*volatile u32, 0x40006414);
pub const GPIO_PORTC_MIS_R = @intToPtr(*volatile u32, 0x40006418);
pub const GPIO_PORTC_ICR_R = @intToPtr(*volatile u32, 0x4000641C);
pub const GPIO_PORTC_AFSEL_R = @intToPtr(*volatile u32, 0x40006420);
pub const GPIO_PORTC_DR2R_R = @intToPtr(*volatile u32, 0x40006500);
pub const GPIO_PORTC_DR4R_R = @intToPtr(*volatile u32, 0x40006504);
pub const GPIO_PORTC_DR8R_R = @intToPtr(*volatile u32, 0x40006508);
pub const GPIO_PORTC_ODR_R = @intToPtr(*volatile u32, 0x4000650C);
pub const GPIO_PORTC_PUR_R = @intToPtr(*volatile u32, 0x40006510);
pub const GPIO_PORTC_PDR_R = @intToPtr(*volatile u32, 0x40006514);
pub const GPIO_PORTC_SLR_R = @intToPtr(*volatile u32, 0x40006518);
pub const GPIO_PORTC_DEN_R = @intToPtr(*volatile u32, 0x4000651C);
pub const GPIO_PORTC_LOCK_R = @intToPtr(*volatile u32, 0x40006520);
pub const GPIO_PORTC_CR_R = @intToPtr(*volatile u32, 0x40006524);
pub const GPIO_PORTC_AMSEL_R = @intToPtr(*volatile u32, 0x40006528);
pub const GPIO_PORTC_PCTL_R = @intToPtr(*volatile u32, 0x4000652C);
pub const GPIO_PORTC_ADCCTL_R = @intToPtr(*volatile u32, 0x40006530);
pub const GPIO_PORTC_DMACTL_R = @intToPtr(*volatile u32, 0x40006534);

//*****************************************************************************
//
// GPIO registers (PORTD)
//
//*****************************************************************************
pub const GPIO_PORTD_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40007000);
pub const GPIO_PORTD_DATA_R = @intToPtr(*volatile u32, 0x400073FC);
pub const GPIO_PORTD_DIR_R = @intToPtr(*volatile u32, 0x40007400);
pub const GPIO_PORTD_IS_R = @intToPtr(*volatile u32, 0x40007404);
pub const GPIO_PORTD_IBE_R = @intToPtr(*volatile u32, 0x40007408);
pub const GPIO_PORTD_IEV_R = @intToPtr(*volatile u32, 0x4000740C);
pub const GPIO_PORTD_IM_R = @intToPtr(*volatile u32, 0x40007410);
pub const GPIO_PORTD_RIS_R = @intToPtr(*volatile u32, 0x40007414);
pub const GPIO_PORTD_MIS_R = @intToPtr(*volatile u32, 0x40007418);
pub const GPIO_PORTD_ICR_R = @intToPtr(*volatile u32, 0x4000741C);
pub const GPIO_PORTD_AFSEL_R = @intToPtr(*volatile u32, 0x40007420);
pub const GPIO_PORTD_DR2R_R = @intToPtr(*volatile u32, 0x40007500);
pub const GPIO_PORTD_DR4R_R = @intToPtr(*volatile u32, 0x40007504);
pub const GPIO_PORTD_DR8R_R = @intToPtr(*volatile u32, 0x40007508);
pub const GPIO_PORTD_ODR_R = @intToPtr(*volatile u32, 0x4000750C);
pub const GPIO_PORTD_PUR_R = @intToPtr(*volatile u32, 0x40007510);
pub const GPIO_PORTD_PDR_R = @intToPtr(*volatile u32, 0x40007514);
pub const GPIO_PORTD_SLR_R = @intToPtr(*volatile u32, 0x40007518);
pub const GPIO_PORTD_DEN_R = @intToPtr(*volatile u32, 0x4000751C);
pub const GPIO_PORTD_LOCK_R = @intToPtr(*volatile u32, 0x40007520);
pub const GPIO_PORTD_CR_R = @intToPtr(*volatile u32, 0x40007524);
pub const GPIO_PORTD_AMSEL_R = @intToPtr(*volatile u32, 0x40007528);
pub const GPIO_PORTD_PCTL_R = @intToPtr(*volatile u32, 0x4000752C);
pub const GPIO_PORTD_ADCCTL_R = @intToPtr(*volatile u32, 0x40007530);
pub const GPIO_PORTD_DMACTL_R = @intToPtr(*volatile u32, 0x40007534);

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
pub const I2C0_MCR2_R = @intToPtr(*volatile u32, 0x40020038);
pub const I2C0_SOAR_R = @intToPtr(*volatile u32, 0x40020800);
pub const I2C0_SCSR_R = @intToPtr(*volatile u32, 0x40020804);
pub const I2C0_SDR_R = @intToPtr(*volatile u32, 0x40020808);
pub const I2C0_SIMR_R = @intToPtr(*volatile u32, 0x4002080C);
pub const I2C0_SRIS_R = @intToPtr(*volatile u32, 0x40020810);
pub const I2C0_SMIS_R = @intToPtr(*volatile u32, 0x40020814);
pub const I2C0_SICR_R = @intToPtr(*volatile u32, 0x40020818);
pub const I2C0_SOAR2_R = @intToPtr(*volatile u32, 0x4002081C);
pub const I2C0_SACKCTL_R = @intToPtr(*volatile u32, 0x40020820);
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
pub const I2C1_MCR2_R = @intToPtr(*volatile u32, 0x40021038);
pub const I2C1_SOAR_R = @intToPtr(*volatile u32, 0x40021800);
pub const I2C1_SCSR_R = @intToPtr(*volatile u32, 0x40021804);
pub const I2C1_SDR_R = @intToPtr(*volatile u32, 0x40021808);
pub const I2C1_SIMR_R = @intToPtr(*volatile u32, 0x4002180C);
pub const I2C1_SRIS_R = @intToPtr(*volatile u32, 0x40021810);
pub const I2C1_SMIS_R = @intToPtr(*volatile u32, 0x40021814);
pub const I2C1_SICR_R = @intToPtr(*volatile u32, 0x40021818);
pub const I2C1_SOAR2_R = @intToPtr(*volatile u32, 0x4002181C);
pub const I2C1_SACKCTL_R = @intToPtr(*volatile u32, 0x40021820);
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
pub const I2C2_MCR2_R = @intToPtr(*volatile u32, 0x40022038);
pub const I2C2_SOAR_R = @intToPtr(*volatile u32, 0x40022800);
pub const I2C2_SCSR_R = @intToPtr(*volatile u32, 0x40022804);
pub const I2C2_SDR_R = @intToPtr(*volatile u32, 0x40022808);
pub const I2C2_SIMR_R = @intToPtr(*volatile u32, 0x4002280C);
pub const I2C2_SRIS_R = @intToPtr(*volatile u32, 0x40022810);
pub const I2C2_SMIS_R = @intToPtr(*volatile u32, 0x40022814);
pub const I2C2_SICR_R = @intToPtr(*volatile u32, 0x40022818);
pub const I2C2_SOAR2_R = @intToPtr(*volatile u32, 0x4002281C);
pub const I2C2_SACKCTL_R = @intToPtr(*volatile u32, 0x40022820);
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
pub const I2C3_MCR2_R = @intToPtr(*volatile u32, 0x40023038);
pub const I2C3_SOAR_R = @intToPtr(*volatile u32, 0x40023800);
pub const I2C3_SCSR_R = @intToPtr(*volatile u32, 0x40023804);
pub const I2C3_SDR_R = @intToPtr(*volatile u32, 0x40023808);
pub const I2C3_SIMR_R = @intToPtr(*volatile u32, 0x4002380C);
pub const I2C3_SRIS_R = @intToPtr(*volatile u32, 0x40023810);
pub const I2C3_SMIS_R = @intToPtr(*volatile u32, 0x40023814);
pub const I2C3_SICR_R = @intToPtr(*volatile u32, 0x40023818);
pub const I2C3_SOAR2_R = @intToPtr(*volatile u32, 0x4002381C);
pub const I2C3_SACKCTL_R = @intToPtr(*volatile u32, 0x40023820);
pub const I2C3_PP_R = @intToPtr(*volatile u32, 0x40023FC0);
pub const I2C3_PC_R = @intToPtr(*volatile u32, 0x40023FC4);

//*****************************************************************************
//
// GPIO registers (PORTE)
//
//*****************************************************************************
pub const GPIO_PORTE_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40024000);
pub const GPIO_PORTE_DATA_R = @intToPtr(*volatile u32, 0x400243FC);
pub const GPIO_PORTE_DIR_R = @intToPtr(*volatile u32, 0x40024400);
pub const GPIO_PORTE_IS_R = @intToPtr(*volatile u32, 0x40024404);
pub const GPIO_PORTE_IBE_R = @intToPtr(*volatile u32, 0x40024408);
pub const GPIO_PORTE_IEV_R = @intToPtr(*volatile u32, 0x4002440C);
pub const GPIO_PORTE_IM_R = @intToPtr(*volatile u32, 0x40024410);
pub const GPIO_PORTE_RIS_R = @intToPtr(*volatile u32, 0x40024414);
pub const GPIO_PORTE_MIS_R = @intToPtr(*volatile u32, 0x40024418);
pub const GPIO_PORTE_ICR_R = @intToPtr(*volatile u32, 0x4002441C);
pub const GPIO_PORTE_AFSEL_R = @intToPtr(*volatile u32, 0x40024420);
pub const GPIO_PORTE_DR2R_R = @intToPtr(*volatile u32, 0x40024500);
pub const GPIO_PORTE_DR4R_R = @intToPtr(*volatile u32, 0x40024504);
pub const GPIO_PORTE_DR8R_R = @intToPtr(*volatile u32, 0x40024508);
pub const GPIO_PORTE_ODR_R = @intToPtr(*volatile u32, 0x4002450C);
pub const GPIO_PORTE_PUR_R = @intToPtr(*volatile u32, 0x40024510);
pub const GPIO_PORTE_PDR_R = @intToPtr(*volatile u32, 0x40024514);
pub const GPIO_PORTE_SLR_R = @intToPtr(*volatile u32, 0x40024518);
pub const GPIO_PORTE_DEN_R = @intToPtr(*volatile u32, 0x4002451C);
pub const GPIO_PORTE_LOCK_R = @intToPtr(*volatile u32, 0x40024520);
pub const GPIO_PORTE_CR_R = @intToPtr(*volatile u32, 0x40024524);
pub const GPIO_PORTE_AMSEL_R = @intToPtr(*volatile u32, 0x40024528);
pub const GPIO_PORTE_PCTL_R = @intToPtr(*volatile u32, 0x4002452C);
pub const GPIO_PORTE_ADCCTL_R = @intToPtr(*volatile u32, 0x40024530);
pub const GPIO_PORTE_DMACTL_R = @intToPtr(*volatile u32, 0x40024534);

//*****************************************************************************
//
// GPIO registers (PORTF)
//
//*****************************************************************************
pub const GPIO_PORTF_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40025000);
pub const GPIO_PORTF_DATA_R = @intToPtr(*volatile u32, 0x400253FC);
pub const GPIO_PORTF_DIR_R = @intToPtr(*volatile u32, 0x40025400);
pub const GPIO_PORTF_IS_R = @intToPtr(*volatile u32, 0x40025404);
pub const GPIO_PORTF_IBE_R = @intToPtr(*volatile u32, 0x40025408);
pub const GPIO_PORTF_IEV_R = @intToPtr(*volatile u32, 0x4002540C);
pub const GPIO_PORTF_IM_R = @intToPtr(*volatile u32, 0x40025410);
pub const GPIO_PORTF_RIS_R = @intToPtr(*volatile u32, 0x40025414);
pub const GPIO_PORTF_MIS_R = @intToPtr(*volatile u32, 0x40025418);
pub const GPIO_PORTF_ICR_R = @intToPtr(*volatile u32, 0x4002541C);
pub const GPIO_PORTF_AFSEL_R = @intToPtr(*volatile u32, 0x40025420);
pub const GPIO_PORTF_DR2R_R = @intToPtr(*volatile u32, 0x40025500);
pub const GPIO_PORTF_DR4R_R = @intToPtr(*volatile u32, 0x40025504);
pub const GPIO_PORTF_DR8R_R = @intToPtr(*volatile u32, 0x40025508);
pub const GPIO_PORTF_ODR_R = @intToPtr(*volatile u32, 0x4002550C);
pub const GPIO_PORTF_PUR_R = @intToPtr(*volatile u32, 0x40025510);
pub const GPIO_PORTF_PDR_R = @intToPtr(*volatile u32, 0x40025514);
pub const GPIO_PORTF_SLR_R = @intToPtr(*volatile u32, 0x40025518);
pub const GPIO_PORTF_DEN_R = @intToPtr(*volatile u32, 0x4002551C);
pub const GPIO_PORTF_LOCK_R = @intToPtr(*volatile u32, 0x40025520);
pub const GPIO_PORTF_CR_R = @intToPtr(*volatile u32, 0x40025524);
pub const GPIO_PORTF_AMSEL_R = @intToPtr(*volatile u32, 0x40025528);
pub const GPIO_PORTF_PCTL_R = @intToPtr(*volatile u32, 0x4002552C);
pub const GPIO_PORTF_ADCCTL_R = @intToPtr(*volatile u32, 0x40025530);
pub const GPIO_PORTF_DMACTL_R = @intToPtr(*volatile u32, 0x40025534);

//*****************************************************************************
//
// GPIO registers (PORTG)
//
//*****************************************************************************
pub const GPIO_PORTG_DATA_BITS_R = @intToPtr([*]volatile u32, 0x40026000);
pub const GPIO_PORTG_DATA_R = @intToPtr(*volatile u32, 0x400263FC);
pub const GPIO_PORTG_DIR_R = @intToPtr(*volatile u32, 0x40026400);
pub const GPIO_PORTG_IS_R = @intToPtr(*volatile u32, 0x40026404);
pub const GPIO_PORTG_IBE_R = @intToPtr(*volatile u32, 0x40026408);
pub const GPIO_PORTG_IEV_R = @intToPtr(*volatile u32, 0x4002640C);
pub const GPIO_PORTG_IM_R = @intToPtr(*volatile u32, 0x40026410);
pub const GPIO_PORTG_RIS_R = @intToPtr(*volatile u32, 0x40026414);
pub const GPIO_PORTG_MIS_R = @intToPtr(*volatile u32, 0x40026418);
pub const GPIO_PORTG_ICR_R = @intToPtr(*volatile u32, 0x4002641C);
pub const GPIO_PORTG_AFSEL_R = @intToPtr(*volatile u32, 0x40026420);
pub const GPIO_PORTG_DR2R_R = @intToPtr(*volatile u32, 0x40026500);
pub const GPIO_PORTG_DR4R_R = @intToPtr(*volatile u32, 0x40026504);
pub const GPIO_PORTG_DR8R_R = @intToPtr(*volatile u32, 0x40026508);
pub const GPIO_PORTG_ODR_R = @intToPtr(*volatile u32, 0x4002650C);
pub const GPIO_PORTG_PUR_R = @intToPtr(*volatile u32, 0x40026510);
pub const GPIO_PORTG_PDR_R = @intToPtr(*volatile u32, 0x40026514);
pub const GPIO_PORTG_SLR_R = @intToPtr(*volatile u32, 0x40026518);
pub const GPIO_PORTG_DEN_R = @intToPtr(*volatile u32, 0x4002651C);
pub const GPIO_PORTG_LOCK_R = @intToPtr(*volatile u32, 0x40026520);
pub const GPIO_PORTG_CR_R = @intToPtr(*volatile u32, 0x40026524);
pub const GPIO_PORTG_AMSEL_R = @intToPtr(*volatile u32, 0x40026528);
pub const GPIO_PORTG_PCTL_R = @intToPtr(*volatile u32, 0x4002652C);
pub const GPIO_PORTG_ADCCTL_R = @intToPtr(*volatile u32, 0x40026530);
pub const GPIO_PORTG_DMACTL_R = @intToPtr(*volatile u32, 0x40026534);

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

//*****************************************************************************
//
// PWM registers (PWM1)
//
//*****************************************************************************
pub const PWM1_CTL_R = @intToPtr(*volatile u32, 0x40029000);
pub const PWM1_SYNC_R = @intToPtr(*volatile u32, 0x40029004);
pub const PWM1_ENABLE_R = @intToPtr(*volatile u32, 0x40029008);
pub const PWM1_INVERT_R = @intToPtr(*volatile u32, 0x4002900C);
pub const PWM1_FAULT_R = @intToPtr(*volatile u32, 0x40029010);
pub const PWM1_INTEN_R = @intToPtr(*volatile u32, 0x40029014);
pub const PWM1_RIS_R = @intToPtr(*volatile u32, 0x40029018);
pub const PWM1_ISC_R = @intToPtr(*volatile u32, 0x4002901C);
pub const PWM1_STATUS_R = @intToPtr(*volatile u32, 0x40029020);
pub const PWM1_FAULTVAL_R = @intToPtr(*volatile u32, 0x40029024);
pub const PWM1_ENUPD_R = @intToPtr(*volatile u32, 0x40029028);
pub const PWM1_0_CTL_R = @intToPtr(*volatile u32, 0x40029040);
pub const PWM1_0_INTEN_R = @intToPtr(*volatile u32, 0x40029044);
pub const PWM1_0_RIS_R = @intToPtr(*volatile u32, 0x40029048);
pub const PWM1_0_ISC_R = @intToPtr(*volatile u32, 0x4002904C);
pub const PWM1_0_LOAD_R = @intToPtr(*volatile u32, 0x40029050);
pub const PWM1_0_COUNT_R = @intToPtr(*volatile u32, 0x40029054);
pub const PWM1_0_CMPA_R = @intToPtr(*volatile u32, 0x40029058);
pub const PWM1_0_CMPB_R = @intToPtr(*volatile u32, 0x4002905C);
pub const PWM1_0_GENA_R = @intToPtr(*volatile u32, 0x40029060);
pub const PWM1_0_GENB_R = @intToPtr(*volatile u32, 0x40029064);
pub const PWM1_0_DBCTL_R = @intToPtr(*volatile u32, 0x40029068);
pub const PWM1_0_DBRISE_R = @intToPtr(*volatile u32, 0x4002906C);
pub const PWM1_0_DBFALL_R = @intToPtr(*volatile u32, 0x40029070);
pub const PWM1_0_FLTSRC0_R = @intToPtr(*volatile u32, 0x40029074);
pub const PWM1_0_FLTSRC1_R = @intToPtr(*volatile u32, 0x40029078);
pub const PWM1_0_MINFLTPER_R = @intToPtr(*volatile u32, 0x4002907C);
pub const PWM1_1_CTL_R = @intToPtr(*volatile u32, 0x40029080);
pub const PWM1_1_INTEN_R = @intToPtr(*volatile u32, 0x40029084);
pub const PWM1_1_RIS_R = @intToPtr(*volatile u32, 0x40029088);
pub const PWM1_1_ISC_R = @intToPtr(*volatile u32, 0x4002908C);
pub const PWM1_1_LOAD_R = @intToPtr(*volatile u32, 0x40029090);
pub const PWM1_1_COUNT_R = @intToPtr(*volatile u32, 0x40029094);
pub const PWM1_1_CMPA_R = @intToPtr(*volatile u32, 0x40029098);
pub const PWM1_1_CMPB_R = @intToPtr(*volatile u32, 0x4002909C);
pub const PWM1_1_GENA_R = @intToPtr(*volatile u32, 0x400290A0);
pub const PWM1_1_GENB_R = @intToPtr(*volatile u32, 0x400290A4);
pub const PWM1_1_DBCTL_R = @intToPtr(*volatile u32, 0x400290A8);
pub const PWM1_1_DBRISE_R = @intToPtr(*volatile u32, 0x400290AC);
pub const PWM1_1_DBFALL_R = @intToPtr(*volatile u32, 0x400290B0);
pub const PWM1_1_FLTSRC0_R = @intToPtr(*volatile u32, 0x400290B4);
pub const PWM1_1_FLTSRC1_R = @intToPtr(*volatile u32, 0x400290B8);
pub const PWM1_1_MINFLTPER_R = @intToPtr(*volatile u32, 0x400290BC);
pub const PWM1_2_CTL_R = @intToPtr(*volatile u32, 0x400290C0);
pub const PWM1_2_INTEN_R = @intToPtr(*volatile u32, 0x400290C4);
pub const PWM1_2_RIS_R = @intToPtr(*volatile u32, 0x400290C8);
pub const PWM1_2_ISC_R = @intToPtr(*volatile u32, 0x400290CC);
pub const PWM1_2_LOAD_R = @intToPtr(*volatile u32, 0x400290D0);
pub const PWM1_2_COUNT_R = @intToPtr(*volatile u32, 0x400290D4);
pub const PWM1_2_CMPA_R = @intToPtr(*volatile u32, 0x400290D8);
pub const PWM1_2_CMPB_R = @intToPtr(*volatile u32, 0x400290DC);
pub const PWM1_2_GENA_R = @intToPtr(*volatile u32, 0x400290E0);
pub const PWM1_2_GENB_R = @intToPtr(*volatile u32, 0x400290E4);
pub const PWM1_2_DBCTL_R = @intToPtr(*volatile u32, 0x400290E8);
pub const PWM1_2_DBRISE_R = @intToPtr(*volatile u32, 0x400290EC);
pub const PWM1_2_DBFALL_R = @intToPtr(*volatile u32, 0x400290F0);
pub const PWM1_2_FLTSRC0_R = @intToPtr(*volatile u32, 0x400290F4);
pub const PWM1_2_FLTSRC1_R = @intToPtr(*volatile u32, 0x400290F8);
pub const PWM1_2_MINFLTPER_R = @intToPtr(*volatile u32, 0x400290FC);
pub const PWM1_3_CTL_R = @intToPtr(*volatile u32, 0x40029100);
pub const PWM1_3_INTEN_R = @intToPtr(*volatile u32, 0x40029104);
pub const PWM1_3_RIS_R = @intToPtr(*volatile u32, 0x40029108);
pub const PWM1_3_ISC_R = @intToPtr(*volatile u32, 0x4002910C);
pub const PWM1_3_LOAD_R = @intToPtr(*volatile u32, 0x40029110);
pub const PWM1_3_COUNT_R = @intToPtr(*volatile u32, 0x40029114);
pub const PWM1_3_CMPA_R = @intToPtr(*volatile u32, 0x40029118);
pub const PWM1_3_CMPB_R = @intToPtr(*volatile u32, 0x4002911C);
pub const PWM1_3_GENA_R = @intToPtr(*volatile u32, 0x40029120);
pub const PWM1_3_GENB_R = @intToPtr(*volatile u32, 0x40029124);
pub const PWM1_3_DBCTL_R = @intToPtr(*volatile u32, 0x40029128);
pub const PWM1_3_DBRISE_R = @intToPtr(*volatile u32, 0x4002912C);
pub const PWM1_3_DBFALL_R = @intToPtr(*volatile u32, 0x40029130);
pub const PWM1_3_FLTSRC0_R = @intToPtr(*volatile u32, 0x40029134);
pub const PWM1_3_FLTSRC1_R = @intToPtr(*volatile u32, 0x40029138);
pub const PWM1_3_MINFLTPER_R = @intToPtr(*volatile u32, 0x4002913C);
pub const PWM1_0_FLTSEN_R = @intToPtr(*volatile u32, 0x40029800);
pub const PWM1_0_FLTSTAT0_R = @intToPtr(*volatile u32, 0x40029804);
pub const PWM1_0_FLTSTAT1_R = @intToPtr(*volatile u32, 0x40029808);
pub const PWM1_1_FLTSEN_R = @intToPtr(*volatile u32, 0x40029880);
pub const PWM1_1_FLTSTAT0_R = @intToPtr(*volatile u32, 0x40029884);
pub const PWM1_1_FLTSTAT1_R = @intToPtr(*volatile u32, 0x40029888);
pub const PWM1_2_FLTSEN_R = @intToPtr(*volatile u32, 0x40029900);
pub const PWM1_2_FLTSTAT0_R = @intToPtr(*volatile u32, 0x40029904);
pub const PWM1_2_FLTSTAT1_R = @intToPtr(*volatile u32, 0x40029908);
pub const PWM1_3_FLTSEN_R = @intToPtr(*volatile u32, 0x40029980);
pub const PWM1_3_FLTSTAT0_R = @intToPtr(*volatile u32, 0x40029984);
pub const PWM1_3_FLTSTAT1_R = @intToPtr(*volatile u32, 0x40029988);
pub const PWM1_PP_R = @intToPtr(*volatile u32, 0x40029FC0);

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
// QEI registers (QEI1)
//
//*****************************************************************************
pub const QEI1_CTL_R = @intToPtr(*volatile u32, 0x4002D000);
pub const QEI1_STAT_R = @intToPtr(*volatile u32, 0x4002D004);
pub const QEI1_POS_R = @intToPtr(*volatile u32, 0x4002D008);
pub const QEI1_MAXPOS_R = @intToPtr(*volatile u32, 0x4002D00C);
pub const QEI1_LOAD_R = @intToPtr(*volatile u32, 0x4002D010);
pub const QEI1_TIME_R = @intToPtr(*volatile u32, 0x4002D014);
pub const QEI1_COUNT_R = @intToPtr(*volatile u32, 0x4002D018);
pub const QEI1_SPEED_R = @intToPtr(*volatile u32, 0x4002D01C);
pub const QEI1_INTEN_R = @intToPtr(*volatile u32, 0x4002D020);
pub const QEI1_RIS_R = @intToPtr(*volatile u32, 0x4002D024);
pub const QEI1_ISC_R = @intToPtr(*volatile u32, 0x4002D028);

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
pub const TIMER0_TAPV_R = @intToPtr(*volatile u32, 0x40030064);
pub const TIMER0_TBPV_R = @intToPtr(*volatile u32, 0x40030068);
pub const TIMER0_PP_R = @intToPtr(*volatile u32, 0x40030FC0);

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
pub const TIMER1_TAPV_R = @intToPtr(*volatile u32, 0x40031064);
pub const TIMER1_TBPV_R = @intToPtr(*volatile u32, 0x40031068);
pub const TIMER1_PP_R = @intToPtr(*volatile u32, 0x40031FC0);

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
pub const TIMER2_TAPV_R = @intToPtr(*volatile u32, 0x40032064);
pub const TIMER2_TBPV_R = @intToPtr(*volatile u32, 0x40032068);
pub const TIMER2_PP_R = @intToPtr(*volatile u32, 0x40032FC0);

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
pub const TIMER3_TAPV_R = @intToPtr(*volatile u32, 0x40033064);
pub const TIMER3_TBPV_R = @intToPtr(*volatile u32, 0x40033068);
pub const TIMER3_PP_R = @intToPtr(*volatile u32, 0x40033FC0);

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
pub const TIMER4_TAPV_R = @intToPtr(*volatile u32, 0x40034064);
pub const TIMER4_TBPV_R = @intToPtr(*volatile u32, 0x40034068);
pub const TIMER4_PP_R = @intToPtr(*volatile u32, 0x40034FC0);

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
pub const TIMER5_TAPV_R = @intToPtr(*volatile u32, 0x40035064);
pub const TIMER5_TBPV_R = @intToPtr(*volatile u32, 0x40035068);
pub const TIMER5_PP_R = @intToPtr(*volatile u32, 0x40035FC0);

//*****************************************************************************
//
// Timer registers (WTIMER0)
//
//*****************************************************************************
pub const WTIMER0_CFG_R = @intToPtr(*volatile u32, 0x40036000);
pub const WTIMER0_TAMR_R = @intToPtr(*volatile u32, 0x40036004);
pub const WTIMER0_TBMR_R = @intToPtr(*volatile u32, 0x40036008);
pub const WTIMER0_CTL_R = @intToPtr(*volatile u32, 0x4003600C);
pub const WTIMER0_SYNC_R = @intToPtr(*volatile u32, 0x40036010);
pub const WTIMER0_IMR_R = @intToPtr(*volatile u32, 0x40036018);
pub const WTIMER0_RIS_R = @intToPtr(*volatile u32, 0x4003601C);
pub const WTIMER0_MIS_R = @intToPtr(*volatile u32, 0x40036020);
pub const WTIMER0_ICR_R = @intToPtr(*volatile u32, 0x40036024);
pub const WTIMER0_TAILR_R = @intToPtr(*volatile u32, 0x40036028);
pub const WTIMER0_TBILR_R = @intToPtr(*volatile u32, 0x4003602C);
pub const WTIMER0_TAMATCHR_R = @intToPtr(*volatile u32, 0x40036030);
pub const WTIMER0_TBMATCHR_R = @intToPtr(*volatile u32, 0x40036034);
pub const WTIMER0_TAPR_R = @intToPtr(*volatile u32, 0x40036038);
pub const WTIMER0_TBPR_R = @intToPtr(*volatile u32, 0x4003603C);
pub const WTIMER0_TAPMR_R = @intToPtr(*volatile u32, 0x40036040);
pub const WTIMER0_TBPMR_R = @intToPtr(*volatile u32, 0x40036044);
pub const WTIMER0_TAR_R = @intToPtr(*volatile u32, 0x40036048);
pub const WTIMER0_TBR_R = @intToPtr(*volatile u32, 0x4003604C);
pub const WTIMER0_TAV_R = @intToPtr(*volatile u32, 0x40036050);
pub const WTIMER0_TBV_R = @intToPtr(*volatile u32, 0x40036054);
pub const WTIMER0_RTCPD_R = @intToPtr(*volatile u32, 0x40036058);
pub const WTIMER0_TAPS_R = @intToPtr(*volatile u32, 0x4003605C);
pub const WTIMER0_TBPS_R = @intToPtr(*volatile u32, 0x40036060);
pub const WTIMER0_TAPV_R = @intToPtr(*volatile u32, 0x40036064);
pub const WTIMER0_TBPV_R = @intToPtr(*volatile u32, 0x40036068);
pub const WTIMER0_PP_R = @intToPtr(*volatile u32, 0x40036FC0);

//*****************************************************************************
//
// Timer registers (WTIMER1)
//
//*****************************************************************************
pub const WTIMER1_CFG_R = @intToPtr(*volatile u32, 0x40037000);
pub const WTIMER1_TAMR_R = @intToPtr(*volatile u32, 0x40037004);
pub const WTIMER1_TBMR_R = @intToPtr(*volatile u32, 0x40037008);
pub const WTIMER1_CTL_R = @intToPtr(*volatile u32, 0x4003700C);
pub const WTIMER1_SYNC_R = @intToPtr(*volatile u32, 0x40037010);
pub const WTIMER1_IMR_R = @intToPtr(*volatile u32, 0x40037018);
pub const WTIMER1_RIS_R = @intToPtr(*volatile u32, 0x4003701C);
pub const WTIMER1_MIS_R = @intToPtr(*volatile u32, 0x40037020);
pub const WTIMER1_ICR_R = @intToPtr(*volatile u32, 0x40037024);
pub const WTIMER1_TAILR_R = @intToPtr(*volatile u32, 0x40037028);
pub const WTIMER1_TBILR_R = @intToPtr(*volatile u32, 0x4003702C);
pub const WTIMER1_TAMATCHR_R = @intToPtr(*volatile u32, 0x40037030);
pub const WTIMER1_TBMATCHR_R = @intToPtr(*volatile u32, 0x40037034);
pub const WTIMER1_TAPR_R = @intToPtr(*volatile u32, 0x40037038);
pub const WTIMER1_TBPR_R = @intToPtr(*volatile u32, 0x4003703C);
pub const WTIMER1_TAPMR_R = @intToPtr(*volatile u32, 0x40037040);
pub const WTIMER1_TBPMR_R = @intToPtr(*volatile u32, 0x40037044);
pub const WTIMER1_TAR_R = @intToPtr(*volatile u32, 0x40037048);
pub const WTIMER1_TBR_R = @intToPtr(*volatile u32, 0x4003704C);
pub const WTIMER1_TAV_R = @intToPtr(*volatile u32, 0x40037050);
pub const WTIMER1_TBV_R = @intToPtr(*volatile u32, 0x40037054);
pub const WTIMER1_RTCPD_R = @intToPtr(*volatile u32, 0x40037058);
pub const WTIMER1_TAPS_R = @intToPtr(*volatile u32, 0x4003705C);
pub const WTIMER1_TBPS_R = @intToPtr(*volatile u32, 0x40037060);
pub const WTIMER1_TAPV_R = @intToPtr(*volatile u32, 0x40037064);
pub const WTIMER1_TBPV_R = @intToPtr(*volatile u32, 0x40037068);
pub const WTIMER1_PP_R = @intToPtr(*volatile u32, 0x40037FC0);

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
pub const ADC0_SSMUX1_R = @intToPtr(*volatile u32, 0x40038060);
pub const ADC0_SSCTL1_R = @intToPtr(*volatile u32, 0x40038064);
pub const ADC0_SSFIFO1_R = @intToPtr(*volatile u32, 0x40038068);
pub const ADC0_SSFSTAT1_R = @intToPtr(*volatile u32, 0x4003806C);
pub const ADC0_SSOP1_R = @intToPtr(*volatile u32, 0x40038070);
pub const ADC0_SSDC1_R = @intToPtr(*volatile u32, 0x40038074);
pub const ADC0_SSMUX2_R = @intToPtr(*volatile u32, 0x40038080);
pub const ADC0_SSCTL2_R = @intToPtr(*volatile u32, 0x40038084);
pub const ADC0_SSFIFO2_R = @intToPtr(*volatile u32, 0x40038088);
pub const ADC0_SSFSTAT2_R = @intToPtr(*volatile u32, 0x4003808C);
pub const ADC0_SSOP2_R = @intToPtr(*volatile u32, 0x40038090);
pub const ADC0_SSDC2_R = @intToPtr(*volatile u32, 0x40038094);
pub const ADC0_SSMUX3_R = @intToPtr(*volatile u32, 0x400380A0);
pub const ADC0_SSCTL3_R = @intToPtr(*volatile u32, 0x400380A4);
pub const ADC0_SSFIFO3_R = @intToPtr(*volatile u32, 0x400380A8);
pub const ADC0_SSFSTAT3_R = @intToPtr(*volatile u32, 0x400380AC);
pub const ADC0_SSOP3_R = @intToPtr(*volatile u32, 0x400380B0);
pub const ADC0_SSDC3_R = @intToPtr(*volatile u32, 0x400380B4);
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
pub const ADC1_SSMUX1_R = @intToPtr(*volatile u32, 0x40039060);
pub const ADC1_SSCTL1_R = @intToPtr(*volatile u32, 0x40039064);
pub const ADC1_SSFIFO1_R = @intToPtr(*volatile u32, 0x40039068);
pub const ADC1_SSFSTAT1_R = @intToPtr(*volatile u32, 0x4003906C);
pub const ADC1_SSOP1_R = @intToPtr(*volatile u32, 0x40039070);
pub const ADC1_SSDC1_R = @intToPtr(*volatile u32, 0x40039074);
pub const ADC1_SSMUX2_R = @intToPtr(*volatile u32, 0x40039080);
pub const ADC1_SSCTL2_R = @intToPtr(*volatile u32, 0x40039084);
pub const ADC1_SSFIFO2_R = @intToPtr(*volatile u32, 0x40039088);
pub const ADC1_SSFSTAT2_R = @intToPtr(*volatile u32, 0x4003908C);
pub const ADC1_SSOP2_R = @intToPtr(*volatile u32, 0x40039090);
pub const ADC1_SSDC2_R = @intToPtr(*volatile u32, 0x40039094);
pub const ADC1_SSMUX3_R = @intToPtr(*volatile u32, 0x400390A0);
pub const ADC1_SSCTL3_R = @intToPtr(*volatile u32, 0x400390A4);
pub const ADC1_SSFIFO3_R = @intToPtr(*volatile u32, 0x400390A8);
pub const ADC1_SSFSTAT3_R = @intToPtr(*volatile u32, 0x400390AC);
pub const ADC1_SSOP3_R = @intToPtr(*volatile u32, 0x400390B0);
pub const ADC1_SSDC3_R = @intToPtr(*volatile u32, 0x400390B4);
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
// Timer registers (WTIMER2)
//
//*****************************************************************************
pub const WTIMER2_CFG_R = @intToPtr(*volatile u32, 0x4004C000);
pub const WTIMER2_TAMR_R = @intToPtr(*volatile u32, 0x4004C004);
pub const WTIMER2_TBMR_R = @intToPtr(*volatile u32, 0x4004C008);
pub const WTIMER2_CTL_R = @intToPtr(*volatile u32, 0x4004C00C);
pub const WTIMER2_SYNC_R = @intToPtr(*volatile u32, 0x4004C010);
pub const WTIMER2_IMR_R = @intToPtr(*volatile u32, 0x4004C018);
pub const WTIMER2_RIS_R = @intToPtr(*volatile u32, 0x4004C01C);
pub const WTIMER2_MIS_R = @intToPtr(*volatile u32, 0x4004C020);
pub const WTIMER2_ICR_R = @intToPtr(*volatile u32, 0x4004C024);
pub const WTIMER2_TAILR_R = @intToPtr(*volatile u32, 0x4004C028);
pub const WTIMER2_TBILR_R = @intToPtr(*volatile u32, 0x4004C02C);
pub const WTIMER2_TAMATCHR_R = @intToPtr(*volatile u32, 0x4004C030);
pub const WTIMER2_TBMATCHR_R = @intToPtr(*volatile u32, 0x4004C034);
pub const WTIMER2_TAPR_R = @intToPtr(*volatile u32, 0x4004C038);
pub const WTIMER2_TBPR_R = @intToPtr(*volatile u32, 0x4004C03C);
pub const WTIMER2_TAPMR_R = @intToPtr(*volatile u32, 0x4004C040);
pub const WTIMER2_TBPMR_R = @intToPtr(*volatile u32, 0x4004C044);
pub const WTIMER2_TAR_R = @intToPtr(*volatile u32, 0x4004C048);
pub const WTIMER2_TBR_R = @intToPtr(*volatile u32, 0x4004C04C);
pub const WTIMER2_TAV_R = @intToPtr(*volatile u32, 0x4004C050);
pub const WTIMER2_TBV_R = @intToPtr(*volatile u32, 0x4004C054);
pub const WTIMER2_RTCPD_R = @intToPtr(*volatile u32, 0x4004C058);
pub const WTIMER2_TAPS_R = @intToPtr(*volatile u32, 0x4004C05C);
pub const WTIMER2_TBPS_R = @intToPtr(*volatile u32, 0x4004C060);
pub const WTIMER2_TAPV_R = @intToPtr(*volatile u32, 0x4004C064);
pub const WTIMER2_TBPV_R = @intToPtr(*volatile u32, 0x4004C068);
pub const WTIMER2_PP_R = @intToPtr(*volatile u32, 0x4004CFC0);

//*****************************************************************************
//
// Timer registers (WTIMER3)
//
//*****************************************************************************
pub const WTIMER3_CFG_R = @intToPtr(*volatile u32, 0x4004D000);
pub const WTIMER3_TAMR_R = @intToPtr(*volatile u32, 0x4004D004);
pub const WTIMER3_TBMR_R = @intToPtr(*volatile u32, 0x4004D008);
pub const WTIMER3_CTL_R = @intToPtr(*volatile u32, 0x4004D00C);
pub const WTIMER3_SYNC_R = @intToPtr(*volatile u32, 0x4004D010);
pub const WTIMER3_IMR_R = @intToPtr(*volatile u32, 0x4004D018);
pub const WTIMER3_RIS_R = @intToPtr(*volatile u32, 0x4004D01C);
pub const WTIMER3_MIS_R = @intToPtr(*volatile u32, 0x4004D020);
pub const WTIMER3_ICR_R = @intToPtr(*volatile u32, 0x4004D024);
pub const WTIMER3_TAILR_R = @intToPtr(*volatile u32, 0x4004D028);
pub const WTIMER3_TBILR_R = @intToPtr(*volatile u32, 0x4004D02C);
pub const WTIMER3_TAMATCHR_R = @intToPtr(*volatile u32, 0x4004D030);
pub const WTIMER3_TBMATCHR_R = @intToPtr(*volatile u32, 0x4004D034);
pub const WTIMER3_TAPR_R = @intToPtr(*volatile u32, 0x4004D038);
pub const WTIMER3_TBPR_R = @intToPtr(*volatile u32, 0x4004D03C);
pub const WTIMER3_TAPMR_R = @intToPtr(*volatile u32, 0x4004D040);
pub const WTIMER3_TBPMR_R = @intToPtr(*volatile u32, 0x4004D044);
pub const WTIMER3_TAR_R = @intToPtr(*volatile u32, 0x4004D048);
pub const WTIMER3_TBR_R = @intToPtr(*volatile u32, 0x4004D04C);
pub const WTIMER3_TAV_R = @intToPtr(*volatile u32, 0x4004D050);
pub const WTIMER3_TBV_R = @intToPtr(*volatile u32, 0x4004D054);
pub const WTIMER3_RTCPD_R = @intToPtr(*volatile u32, 0x4004D058);
pub const WTIMER3_TAPS_R = @intToPtr(*volatile u32, 0x4004D05C);
pub const WTIMER3_TBPS_R = @intToPtr(*volatile u32, 0x4004D060);
pub const WTIMER3_TAPV_R = @intToPtr(*volatile u32, 0x4004D064);
pub const WTIMER3_TBPV_R = @intToPtr(*volatile u32, 0x4004D068);
pub const WTIMER3_PP_R = @intToPtr(*volatile u32, 0x4004DFC0);

//*****************************************************************************
//
// Timer registers (WTIMER4)
//
//*****************************************************************************
pub const WTIMER4_CFG_R = @intToPtr(*volatile u32, 0x4004E000);
pub const WTIMER4_TAMR_R = @intToPtr(*volatile u32, 0x4004E004);
pub const WTIMER4_TBMR_R = @intToPtr(*volatile u32, 0x4004E008);
pub const WTIMER4_CTL_R = @intToPtr(*volatile u32, 0x4004E00C);
pub const WTIMER4_SYNC_R = @intToPtr(*volatile u32, 0x4004E010);
pub const WTIMER4_IMR_R = @intToPtr(*volatile u32, 0x4004E018);
pub const WTIMER4_RIS_R = @intToPtr(*volatile u32, 0x4004E01C);
pub const WTIMER4_MIS_R = @intToPtr(*volatile u32, 0x4004E020);
pub const WTIMER4_ICR_R = @intToPtr(*volatile u32, 0x4004E024);
pub const WTIMER4_TAILR_R = @intToPtr(*volatile u32, 0x4004E028);
pub const WTIMER4_TBILR_R = @intToPtr(*volatile u32, 0x4004E02C);
pub const WTIMER4_TAMATCHR_R = @intToPtr(*volatile u32, 0x4004E030);
pub const WTIMER4_TBMATCHR_R = @intToPtr(*volatile u32, 0x4004E034);
pub const WTIMER4_TAPR_R = @intToPtr(*volatile u32, 0x4004E038);
pub const WTIMER4_TBPR_R = @intToPtr(*volatile u32, 0x4004E03C);
pub const WTIMER4_TAPMR_R = @intToPtr(*volatile u32, 0x4004E040);
pub const WTIMER4_TBPMR_R = @intToPtr(*volatile u32, 0x4004E044);
pub const WTIMER4_TAR_R = @intToPtr(*volatile u32, 0x4004E048);
pub const WTIMER4_TBR_R = @intToPtr(*volatile u32, 0x4004E04C);
pub const WTIMER4_TAV_R = @intToPtr(*volatile u32, 0x4004E050);
pub const WTIMER4_TBV_R = @intToPtr(*volatile u32, 0x4004E054);
pub const WTIMER4_RTCPD_R = @intToPtr(*volatile u32, 0x4004E058);
pub const WTIMER4_TAPS_R = @intToPtr(*volatile u32, 0x4004E05C);
pub const WTIMER4_TBPS_R = @intToPtr(*volatile u32, 0x4004E060);
pub const WTIMER4_TAPV_R = @intToPtr(*volatile u32, 0x4004E064);
pub const WTIMER4_TBPV_R = @intToPtr(*volatile u32, 0x4004E068);
pub const WTIMER4_PP_R = @intToPtr(*volatile u32, 0x4004EFC0);

//*****************************************************************************
//
// Timer registers (WTIMER5)
//
//*****************************************************************************
pub const WTIMER5_CFG_R = @intToPtr(*volatile u32, 0x4004F000);
pub const WTIMER5_TAMR_R = @intToPtr(*volatile u32, 0x4004F004);
pub const WTIMER5_TBMR_R = @intToPtr(*volatile u32, 0x4004F008);
pub const WTIMER5_CTL_R = @intToPtr(*volatile u32, 0x4004F00C);
pub const WTIMER5_SYNC_R = @intToPtr(*volatile u32, 0x4004F010);
pub const WTIMER5_IMR_R = @intToPtr(*volatile u32, 0x4004F018);
pub const WTIMER5_RIS_R = @intToPtr(*volatile u32, 0x4004F01C);
pub const WTIMER5_MIS_R = @intToPtr(*volatile u32, 0x4004F020);
pub const WTIMER5_ICR_R = @intToPtr(*volatile u32, 0x4004F024);
pub const WTIMER5_TAILR_R = @intToPtr(*volatile u32, 0x4004F028);
pub const WTIMER5_TBILR_R = @intToPtr(*volatile u32, 0x4004F02C);
pub const WTIMER5_TAMATCHR_R = @intToPtr(*volatile u32, 0x4004F030);
pub const WTIMER5_TBMATCHR_R = @intToPtr(*volatile u32, 0x4004F034);
pub const WTIMER5_TAPR_R = @intToPtr(*volatile u32, 0x4004F038);
pub const WTIMER5_TBPR_R = @intToPtr(*volatile u32, 0x4004F03C);
pub const WTIMER5_TAPMR_R = @intToPtr(*volatile u32, 0x4004F040);
pub const WTIMER5_TBPMR_R = @intToPtr(*volatile u32, 0x4004F044);
pub const WTIMER5_TAR_R = @intToPtr(*volatile u32, 0x4004F048);
pub const WTIMER5_TBR_R = @intToPtr(*volatile u32, 0x4004F04C);
pub const WTIMER5_TAV_R = @intToPtr(*volatile u32, 0x4004F050);
pub const WTIMER5_TBV_R = @intToPtr(*volatile u32, 0x4004F054);
pub const WTIMER5_RTCPD_R = @intToPtr(*volatile u32, 0x4004F058);
pub const WTIMER5_TAPS_R = @intToPtr(*volatile u32, 0x4004F05C);
pub const WTIMER5_TBPS_R = @intToPtr(*volatile u32, 0x4004F060);
pub const WTIMER5_TAPV_R = @intToPtr(*volatile u32, 0x4004F064);
pub const WTIMER5_TBPV_R = @intToPtr(*volatile u32, 0x4004F068);
pub const WTIMER5_PP_R = @intToPtr(*volatile u32, 0x4004FFC0);

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
#define USB0_TXFIFOSZ_R         (*((volatile uint8_t *)0x40050062))
#define USB0_RXFIFOSZ_R         (*((volatile uint8_t *)0x40050063))
#define USB0_TXFIFOADD_R        (*((volatile uint16_t *)0x40050064))
#define USB0_RXFIFOADD_R        (*((volatile uint16_t *)0x40050066))
#define USB0_CONTIM_R           (*((volatile uint8_t *)0x4005007A))
#define USB0_VPLEN_R            (*((volatile uint8_t *)0x4005007B))
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
#define USB0_RQPKTCOUNT1_R      (*((volatile uint16_t *)0x40050304))
#define USB0_RQPKTCOUNT2_R      (*((volatile uint16_t *)0x40050308))
#define USB0_RQPKTCOUNT3_R      (*((volatile uint16_t *)0x4005030C))
#define USB0_RQPKTCOUNT4_R      (*((volatile uint16_t *)0x40050310))
#define USB0_RQPKTCOUNT5_R      (*((volatile uint16_t *)0x40050314))
#define USB0_RQPKTCOUNT6_R      (*((volatile uint16_t *)0x40050318))
#define USB0_RQPKTCOUNT7_R      (*((volatile uint16_t *)0x4005031C))
#define USB0_RXDPKTBUFDIS_R     (*((volatile uint16_t *)0x40050340))
#define USB0_TXDPKTBUFDIS_R     (*((volatile uint16_t *)0x40050342))
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
pub const USB0_IDVRIS_R = @intToPtr(*volatile u32, 0x40050444);
pub const USB0_IDVIM_R = @intToPtr(*volatile u32, 0x40050448);
pub const USB0_IDVISC_R = @intToPtr(*volatile u32, 0x4005044C);
pub const USB0_DMASEL_R = @intToPtr(*volatile u32, 0x40050450);
pub const USB0_PP_R = @intToPtr(*volatile u32, 0x40050FC0);

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
pub const EEPROM_EEHIDE_R = @intToPtr(*volatile u32, 0x400AF050);
pub const EEPROM_EEDBGME_R = @intToPtr(*volatile u32, 0x400AF080);
pub const EEPROM_PP_R = @intToPtr(*volatile u32, 0x400AFFC0);

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
pub const I2C4_MCR2_R = @intToPtr(*volatile u32, 0x400C0038);
pub const I2C4_SOAR_R = @intToPtr(*volatile u32, 0x400C0800);
pub const I2C4_SCSR_R = @intToPtr(*volatile u32, 0x400C0804);
pub const I2C4_SDR_R = @intToPtr(*volatile u32, 0x400C0808);
pub const I2C4_SIMR_R = @intToPtr(*volatile u32, 0x400C080C);
pub const I2C4_SRIS_R = @intToPtr(*volatile u32, 0x400C0810);
pub const I2C4_SMIS_R = @intToPtr(*volatile u32, 0x400C0814);
pub const I2C4_SICR_R = @intToPtr(*volatile u32, 0x400C0818);
pub const I2C4_SOAR2_R = @intToPtr(*volatile u32, 0x400C081C);
pub const I2C4_SACKCTL_R = @intToPtr(*volatile u32, 0x400C0820);
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
pub const I2C5_MCR2_R = @intToPtr(*volatile u32, 0x400C1038);
pub const I2C5_SOAR_R = @intToPtr(*volatile u32, 0x400C1800);
pub const I2C5_SCSR_R = @intToPtr(*volatile u32, 0x400C1804);
pub const I2C5_SDR_R = @intToPtr(*volatile u32, 0x400C1808);
pub const I2C5_SIMR_R = @intToPtr(*volatile u32, 0x400C180C);
pub const I2C5_SRIS_R = @intToPtr(*volatile u32, 0x400C1810);
pub const I2C5_SMIS_R = @intToPtr(*volatile u32, 0x400C1814);
pub const I2C5_SICR_R = @intToPtr(*volatile u32, 0x400C1818);
pub const I2C5_SOAR2_R = @intToPtr(*volatile u32, 0x400C181C);
pub const I2C5_SACKCTL_R = @intToPtr(*volatile u32, 0x400C1820);
pub const I2C5_PP_R = @intToPtr(*volatile u32, 0x400C1FC0);
pub const I2C5_PC_R = @intToPtr(*volatile u32, 0x400C1FC4);

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
pub const FLASH_FWBN_R = @intToPtr(*volatile u32, 0x400FD100);
pub const FLASH_FSIZE_R = @intToPtr(*volatile u32, 0x400FDFC0);
pub const FLASH_SSIZE_R = @intToPtr(*volatile u32, 0x400FDFC4);
pub const FLASH_ROMSWMAP_R = @intToPtr(*volatile u32, 0x400FDFCC);
pub const FLASH_RMCTL_R = @intToPtr(*volatile u32, 0x400FE0F0);
pub const FLASH_BOOTCFG_R = @intToPtr(*volatile u32, 0x400FE1D0);
pub const FLASH_USERREG0_R = @intToPtr(*volatile u32, 0x400FE1E0);
pub const FLASH_USERREG1_R = @intToPtr(*volatile u32, 0x400FE1E4);
pub const FLASH_USERREG2_R = @intToPtr(*volatile u32, 0x400FE1E8);
pub const FLASH_USERREG3_R = @intToPtr(*volatile u32, 0x400FE1EC);
pub const FLASH_FMPRE0_R = @intToPtr(*volatile u32, 0x400FE200);
pub const FLASH_FMPRE1_R = @intToPtr(*volatile u32, 0x400FE204);
pub const FLASH_FMPRE2_R = @intToPtr(*volatile u32, 0x400FE208);
pub const FLASH_FMPRE3_R = @intToPtr(*volatile u32, 0x400FE20C);
pub const FLASH_FMPPE0_R = @intToPtr(*volatile u32, 0x400FE400);
pub const FLASH_FMPPE1_R = @intToPtr(*volatile u32, 0x400FE404);
pub const FLASH_FMPPE2_R = @intToPtr(*volatile u32, 0x400FE408);
pub const FLASH_FMPPE3_R = @intToPtr(*volatile u32, 0x400FE40C);

//*****************************************************************************
//
// System Control registers (SYSCTL)
//
//*****************************************************************************
pub const SYSCTL_DID0_R = @intToPtr(*volatile u32, 0x400FE000);
pub const SYSCTL_DID1_R = @intToPtr(*volatile u32, 0x400FE004);
pub const SYSCTL_DC0_R = @intToPtr(*volatile u32, 0x400FE008);
pub const SYSCTL_DC1_R = @intToPtr(*volatile u32, 0x400FE010);
pub const SYSCTL_DC2_R = @intToPtr(*volatile u32, 0x400FE014);
pub const SYSCTL_DC3_R = @intToPtr(*volatile u32, 0x400FE018);
pub const SYSCTL_DC4_R = @intToPtr(*volatile u32, 0x400FE01C);
pub const SYSCTL_DC5_R = @intToPtr(*volatile u32, 0x400FE020);
pub const SYSCTL_DC6_R = @intToPtr(*volatile u32, 0x400FE024);
pub const SYSCTL_DC7_R = @intToPtr(*volatile u32, 0x400FE028);
pub const SYSCTL_DC8_R = @intToPtr(*volatile u32, 0x400FE02C);
pub const SYSCTL_PBORCTL_R = @intToPtr(*volatile u32, 0x400FE030);
pub const SYSCTL_SRCR0_R = @intToPtr(*volatile u32, 0x400FE040);
pub const SYSCTL_SRCR1_R = @intToPtr(*volatile u32, 0x400FE044);
pub const SYSCTL_SRCR2_R = @intToPtr(*volatile u32, 0x400FE048);
pub const SYSCTL_RIS_R = @intToPtr(*volatile u32, 0x400FE050);
pub const SYSCTL_IMC_R = @intToPtr(*volatile u32, 0x400FE054);
pub const SYSCTL_MISC_R = @intToPtr(*volatile u32, 0x400FE058);
pub const SYSCTL_RESC_R = @intToPtr(*volatile u32, 0x400FE05C);
pub const SYSCTL_RCC_R = @intToPtr(*volatile u32, 0x400FE060);
pub const SYSCTL_GPIOHBCTL_R = @intToPtr(*volatile u32, 0x400FE06C);
pub const SYSCTL_RCC2_R = @intToPtr(*volatile u32, 0x400FE070);
pub const SYSCTL_MOSCCTL_R = @intToPtr(*volatile u32, 0x400FE07C);
pub const SYSCTL_RCGC0_R = @intToPtr(*volatile u32, 0x400FE100);
pub const SYSCTL_RCGC1_R = @intToPtr(*volatile u32, 0x400FE104);
pub const SYSCTL_RCGC2_R = @intToPtr(*volatile u32, 0x400FE108);
pub const SYSCTL_SCGC0_R = @intToPtr(*volatile u32, 0x400FE110);
pub const SYSCTL_SCGC1_R = @intToPtr(*volatile u32, 0x400FE114);
pub const SYSCTL_SCGC2_R = @intToPtr(*volatile u32, 0x400FE118);
pub const SYSCTL_DCGC0_R = @intToPtr(*volatile u32, 0x400FE120);
pub const SYSCTL_DCGC1_R = @intToPtr(*volatile u32, 0x400FE124);
pub const SYSCTL_DCGC2_R = @intToPtr(*volatile u32, 0x400FE128);
pub const SYSCTL_DSLPCLKCFG_R = @intToPtr(*volatile u32, 0x400FE144);
pub const SYSCTL_SYSPROP_R = @intToPtr(*volatile u32, 0x400FE14C);
pub const SYSCTL_PIOSCCAL_R = @intToPtr(*volatile u32, 0x400FE150);
pub const SYSCTL_PLLFREQ0_R = @intToPtr(*volatile u32, 0x400FE160);
pub const SYSCTL_PLLFREQ1_R = @intToPtr(*volatile u32, 0x400FE164);
pub const SYSCTL_PLLSTAT_R = @intToPtr(*volatile u32, 0x400FE168);
pub const SYSCTL_SLPPWRCFG_R = @intToPtr(*volatile u32, 0x400FE188);
pub const SYSCTL_DSLPPWRCFG_R = @intToPtr(*volatile u32, 0x400FE18C);
pub const SYSCTL_DC9_R = @intToPtr(*volatile u32, 0x400FE190);
pub const SYSCTL_NVMSTAT_R = @intToPtr(*volatile u32, 0x400FE1A0);
pub const SYSCTL_LDOSPCTL_R = @intToPtr(*volatile u32, 0x400FE1B4);
pub const SYSCTL_LDODPCTL_R = @intToPtr(*volatile u32, 0x400FE1BC);
pub const SYSCTL_PPWD_R = @intToPtr(*volatile u32, 0x400FE300);
pub const SYSCTL_PPTIMER_R = @intToPtr(*volatile u32, 0x400FE304);
pub const SYSCTL_PPGPIO_R = @intToPtr(*volatile u32, 0x400FE308);
pub const SYSCTL_PPDMA_R = @intToPtr(*volatile u32, 0x400FE30C);
pub const SYSCTL_PPHIB_R = @intToPtr(*volatile u32, 0x400FE314);
pub const SYSCTL_PPUART_R = @intToPtr(*volatile u32, 0x400FE318);
pub const SYSCTL_PPSSI_R = @intToPtr(*volatile u32, 0x400FE31C);
pub const SYSCTL_PPI2C_R = @intToPtr(*volatile u32, 0x400FE320);
pub const SYSCTL_PPUSB_R = @intToPtr(*volatile u32, 0x400FE328);
pub const SYSCTL_PPCAN_R = @intToPtr(*volatile u32, 0x400FE334);
pub const SYSCTL_PPADC_R = @intToPtr(*volatile u32, 0x400FE338);
pub const SYSCTL_PPACMP_R = @intToPtr(*volatile u32, 0x400FE33C);
pub const SYSCTL_PPPWM_R = @intToPtr(*volatile u32, 0x400FE340);
pub const SYSCTL_PPQEI_R = @intToPtr(*volatile u32, 0x400FE344);
pub const SYSCTL_PPEEPROM_R = @intToPtr(*volatile u32, 0x400FE358);
pub const SYSCTL_PPWTIMER_R = @intToPtr(*volatile u32, 0x400FE35C);
pub const SYSCTL_SRWD_R = @intToPtr(*volatile u32, 0x400FE500);
pub const SYSCTL_SRTIMER_R = @intToPtr(*volatile u32, 0x400FE504);
pub const SYSCTL_SRGPIO_R = @intToPtr(*volatile u32, 0x400FE508);
pub const SYSCTL_SRDMA_R = @intToPtr(*volatile u32, 0x400FE50C);
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
pub const SYSCTL_SRWTIMER_R = @intToPtr(*volatile u32, 0x400FE55C);
pub const SYSCTL_RCGCWD_R = @intToPtr(*volatile u32, 0x400FE600);
pub const SYSCTL_RCGCTIMER_R = @intToPtr(*volatile u32, 0x400FE604);
pub const SYSCTL_RCGCGPIO_R = @intToPtr(*volatile u32, 0x400FE608);
pub const SYSCTL_RCGCDMA_R = @intToPtr(*volatile u32, 0x400FE60C);
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
pub const SYSCTL_RCGCWTIMER_R = @intToPtr(*volatile u32, 0x400FE65C);
pub const SYSCTL_SCGCWD_R = @intToPtr(*volatile u32, 0x400FE700);
pub const SYSCTL_SCGCTIMER_R = @intToPtr(*volatile u32, 0x400FE704);
pub const SYSCTL_SCGCGPIO_R = @intToPtr(*volatile u32, 0x400FE708);
pub const SYSCTL_SCGCDMA_R = @intToPtr(*volatile u32, 0x400FE70C);
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
pub const SYSCTL_SCGCWTIMER_R = @intToPtr(*volatile u32, 0x400FE75C);
pub const SYSCTL_DCGCWD_R = @intToPtr(*volatile u32, 0x400FE800);
pub const SYSCTL_DCGCTIMER_R = @intToPtr(*volatile u32, 0x400FE804);
pub const SYSCTL_DCGCGPIO_R = @intToPtr(*volatile u32, 0x400FE808);
pub const SYSCTL_DCGCDMA_R = @intToPtr(*volatile u32, 0x400FE80C);
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
pub const SYSCTL_DCGCWTIMER_R = @intToPtr(*volatile u32, 0x400FE85C);
pub const SYSCTL_PRWD_R = @intToPtr(*volatile u32, 0x400FEA00);
pub const SYSCTL_PRTIMER_R = @intToPtr(*volatile u32, 0x400FEA04);
pub const SYSCTL_PRGPIO_R = @intToPtr(*volatile u32, 0x400FEA08);
pub const SYSCTL_PRDMA_R = @intToPtr(*volatile u32, 0x400FEA0C);
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
pub const SYSCTL_PRWTIMER_R = @intToPtr(*volatile u32, 0x400FEA5C);

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
pub const UDMA_CHIS_R = @intToPtr(*volatile u32, 0x400FF504);
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
pub const NVIC_EN4_R = @intToPtr(*volatile u32, 0xE000E110);
pub const NVIC_DIS0_R = @intToPtr(*volatile u32, 0xE000E180);
pub const NVIC_DIS1_R = @intToPtr(*volatile u32, 0xE000E184);
pub const NVIC_DIS2_R = @intToPtr(*volatile u32, 0xE000E188);
pub const NVIC_DIS3_R = @intToPtr(*volatile u32, 0xE000E18C);
pub const NVIC_DIS4_R = @intToPtr(*volatile u32, 0xE000E190);
pub const NVIC_PEND0_R = @intToPtr(*volatile u32, 0xE000E200);
pub const NVIC_PEND1_R = @intToPtr(*volatile u32, 0xE000E204);
pub const NVIC_PEND2_R = @intToPtr(*volatile u32, 0xE000E208);
pub const NVIC_PEND3_R = @intToPtr(*volatile u32, 0xE000E20C);
pub const NVIC_PEND4_R = @intToPtr(*volatile u32, 0xE000E210);
pub const NVIC_UNPEND0_R = @intToPtr(*volatile u32, 0xE000E280);
pub const NVIC_UNPEND1_R = @intToPtr(*volatile u32, 0xE000E284);
pub const NVIC_UNPEND2_R = @intToPtr(*volatile u32, 0xE000E288);
pub const NVIC_UNPEND3_R = @intToPtr(*volatile u32, 0xE000E28C);
pub const NVIC_UNPEND4_R = @intToPtr(*volatile u32, 0xE000E290);
pub const NVIC_ACTIVE0_R = @intToPtr(*volatile u32, 0xE000E300);
pub const NVIC_ACTIVE1_R = @intToPtr(*volatile u32, 0xE000E304);
pub const NVIC_ACTIVE2_R = @intToPtr(*volatile u32, 0xE000E308);
pub const NVIC_ACTIVE3_R = @intToPtr(*volatile u32, 0xE000E30C);
pub const NVIC_ACTIVE4_R = @intToPtr(*volatile u32, 0xE000E310);
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
pub const NVIC_PRI29_R = @intToPtr(*volatile u32, 0xE000E474);
pub const NVIC_PRI30_R = @intToPtr(*volatile u32, 0xE000E478);
pub const NVIC_PRI31_R = @intToPtr(*volatile u32, 0xE000E47C);
pub const NVIC_PRI32_R = @intToPtr(*volatile u32, 0xE000E480);
pub const NVIC_PRI33_R = @intToPtr(*volatile u32, 0xE000E484);
pub const NVIC_PRI34_R = @intToPtr(*volatile u32, 0xE000E488);
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
// The following are defines for the bit fields in the GPIO_O_IM register.
//
//*****************************************************************************
pub const GPIO_IM_GPIO_M = usize(0x000000FF);  // GPIO Interrupt Mask Enable
pub const GPIO_IM_GPIO_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_RIS register.
//
//*****************************************************************************
pub const GPIO_RIS_GPIO_M = usize(0x000000FF);  // GPIO Interrupt Raw Status
pub const GPIO_RIS_GPIO_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_MIS register.
//
//*****************************************************************************
pub const GPIO_MIS_GPIO_M = usize(0x000000FF);  // GPIO Masked Interrupt Status
pub const GPIO_MIS_GPIO_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_ICR register.
//
//*****************************************************************************
pub const GPIO_ICR_GPIO_M = usize(0x000000FF);  // GPIO Interrupt Clear
pub const GPIO_ICR_GPIO_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_LOCK register.
//
//*****************************************************************************
pub const GPIO_LOCK_M = usize(0xFFFFFFFF);  // GPIO Lock
pub const GPIO_LOCK_UNLOCKED = usize(0x00000000);  // The GPIOCR register is unlocked
                                            // and may be modified
pub const GPIO_LOCK_LOCKED = usize(0x00000001);  // The GPIOCR register is locked
                                            // and may not be modified
pub const GPIO_LOCK_KEY = usize(0x4C4F434B);  // Unlocks the GPIO_CR register

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port A.
//
//*****************************************************************************
pub const GPIO_PCTL_PA7_M = usize(0xF0000000);  // PA7 Mask
pub const GPIO_PCTL_PA7_I2C1SDA = usize(0x30000000);  // I2C1SDA on PA7
pub const GPIO_PCTL_PA7_M1PWM3 = usize(0x50000000);  // M1PWM3 on PA7
pub const GPIO_PCTL_PA6_M = usize(0x0F000000);  // PA6 Mask
pub const GPIO_PCTL_PA6_I2C1SCL = usize(0x03000000);  // I2C1SCL on PA6
pub const GPIO_PCTL_PA6_M1PWM2 = usize(0x05000000);  // M1PWM2 on PA6
pub const GPIO_PCTL_PA5_M = usize(0x00F00000);  // PA5 Mask
pub const GPIO_PCTL_PA5_SSI0TX = usize(0x00200000);  // SSI0TX on PA5
pub const GPIO_PCTL_PA4_M = usize(0x000F0000);  // PA4 Mask
pub const GPIO_PCTL_PA4_SSI0RX = usize(0x00020000);  // SSI0RX on PA4
pub const GPIO_PCTL_PA3_M = usize(0x0000F000);  // PA3 Mask
pub const GPIO_PCTL_PA3_SSI0FSS = usize(0x00002000);  // SSI0FSS on PA3
pub const GPIO_PCTL_PA2_M = usize(0x00000F00);  // PA2 Mask
pub const GPIO_PCTL_PA2_SSI0CLK = usize(0x00000200);  // SSI0CLK on PA2
pub const GPIO_PCTL_PA1_M = usize(0x000000F0);  // PA1 Mask
pub const GPIO_PCTL_PA1_U0TX = usize(0x00000010);  // U0TX on PA1
pub const GPIO_PCTL_PA1_CAN1TX = usize(0x00000080);  // CAN1TX on PA1
pub const GPIO_PCTL_PA0_M = usize(0x0000000F);  // PA0 Mask
pub const GPIO_PCTL_PA0_U0RX = usize(0x00000001);  // U0RX on PA0
pub const GPIO_PCTL_PA0_CAN1RX = usize(0x00000008);  // CAN1RX on PA0

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port B.
//
//*****************************************************************************
pub const GPIO_PCTL_PB7_M = usize(0xF0000000);  // PB7 Mask
pub const GPIO_PCTL_PB7_SSI2TX = usize(0x20000000);  // SSI2TX on PB7
pub const GPIO_PCTL_PB7_I2C5SDA = usize(0x30000000);  // I2C5SDA on PB7
pub const GPIO_PCTL_PB7_M0PWM1 = usize(0x40000000);  // M0PWM1 on PB7
pub const GPIO_PCTL_PB7_T0CCP1 = usize(0x70000000);  // T0CCP1 on PB7
pub const GPIO_PCTL_PB6_M = usize(0x0F000000);  // PB6 Mask
pub const GPIO_PCTL_PB6_SSI2RX = usize(0x02000000);  // SSI2RX on PB6
pub const GPIO_PCTL_PB6_I2C5SCL = usize(0x03000000);  // I2C5SCL on PB6
pub const GPIO_PCTL_PB6_M0PWM0 = usize(0x04000000);  // M0PWM0 on PB6
pub const GPIO_PCTL_PB6_T0CCP0 = usize(0x07000000);  // T0CCP0 on PB6
pub const GPIO_PCTL_PB5_M = usize(0x00F00000);  // PB5 Mask
pub const GPIO_PCTL_PB5_SSI2FSS = usize(0x00200000);  // SSI2FSS on PB5
pub const GPIO_PCTL_PB5_M0PWM3 = usize(0x00400000);  // M0PWM3 on PB5
pub const GPIO_PCTL_PB5_T1CCP1 = usize(0x00700000);  // T1CCP1 on PB5
pub const GPIO_PCTL_PB5_CAN0TX = usize(0x00800000);  // CAN0TX on PB5
pub const GPIO_PCTL_PB4_M = usize(0x000F0000);  // PB4 Mask
pub const GPIO_PCTL_PB4_SSI2CLK = usize(0x00020000);  // SSI2CLK on PB4
pub const GPIO_PCTL_PB4_M0PWM2 = usize(0x00040000);  // M0PWM2 on PB4
pub const GPIO_PCTL_PB4_T1CCP0 = usize(0x00070000);  // T1CCP0 on PB4
pub const GPIO_PCTL_PB4_CAN0RX = usize(0x00080000);  // CAN0RX on PB4
pub const GPIO_PCTL_PB3_M = usize(0x0000F000);  // PB3 Mask
pub const GPIO_PCTL_PB3_I2C0SDA = usize(0x00003000);  // I2C0SDA on PB3
pub const GPIO_PCTL_PB3_T3CCP1 = usize(0x00007000);  // T3CCP1 on PB3
pub const GPIO_PCTL_PB2_M = usize(0x00000F00);  // PB2 Mask
pub const GPIO_PCTL_PB2_I2C0SCL = usize(0x00000300);  // I2C0SCL on PB2
pub const GPIO_PCTL_PB2_T3CCP0 = usize(0x00000700);  // T3CCP0 on PB2
pub const GPIO_PCTL_PB1_M = usize(0x000000F0);  // PB1 Mask
pub const GPIO_PCTL_PB1_USB0VBUS = usize(0x00000000);  // USB0VBUS on PB1
pub const GPIO_PCTL_PB1_U1TX = usize(0x00000010);  // U1TX on PB1
pub const GPIO_PCTL_PB1_T2CCP1 = usize(0x00000070);  // T2CCP1 on PB1
pub const GPIO_PCTL_PB0_M = usize(0x0000000F);  // PB0 Mask
pub const GPIO_PCTL_PB0_USB0ID = usize(0x00000000);  // USB0ID on PB0
pub const GPIO_PCTL_PB0_U1RX = usize(0x00000001);  // U1RX on PB0
pub const GPIO_PCTL_PB0_T2CCP0 = usize(0x00000007);  // T2CCP0 on PB0

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port C.
//
//*****************************************************************************
pub const GPIO_PCTL_PC7_M = usize(0xF0000000);  // PC7 Mask
pub const GPIO_PCTL_PC7_U3TX = usize(0x10000000);  // U3TX on PC7
pub const GPIO_PCTL_PC7_WT1CCP1 = usize(0x70000000);  // WT1CCP1 on PC7
pub const GPIO_PCTL_PC7_USB0PFLT = usize(0x80000000);  // USB0PFLT on PC7
pub const GPIO_PCTL_PC6_M = usize(0x0F000000);  // PC6 Mask
pub const GPIO_PCTL_PC6_U3RX = usize(0x01000000);  // U3RX on PC6
pub const GPIO_PCTL_PC6_PHB1 = usize(0x06000000);  // PHB1 on PC6
pub const GPIO_PCTL_PC6_WT1CCP0 = usize(0x07000000);  // WT1CCP0 on PC6
pub const GPIO_PCTL_PC6_USB0EPEN = usize(0x08000000);  // USB0EPEN on PC6
pub const GPIO_PCTL_PC5_M = usize(0x00F00000);  // PC5 Mask
pub const GPIO_PCTL_PC5_U4TX = usize(0x00100000);  // U4TX on PC5
pub const GPIO_PCTL_PC5_U1TX = usize(0x00200000);  // U1TX on PC5
pub const GPIO_PCTL_PC5_M0PWM7 = usize(0x00400000);  // M0PWM7 on PC5
pub const GPIO_PCTL_PC5_PHA1 = usize(0x00600000);  // PHA1 on PC5
pub const GPIO_PCTL_PC5_WT0CCP1 = usize(0x00700000);  // WT0CCP1 on PC5
pub const GPIO_PCTL_PC5_U1CTS = usize(0x00800000);  // U1CTS on PC5
pub const GPIO_PCTL_PC4_M = usize(0x000F0000);  // PC4 Mask
pub const GPIO_PCTL_PC4_U4RX = usize(0x00010000);  // U4RX on PC4
pub const GPIO_PCTL_PC4_U1RX = usize(0x00020000);  // U1RX on PC4
pub const GPIO_PCTL_PC4_M0PWM6 = usize(0x00040000);  // M0PWM6 on PC4
pub const GPIO_PCTL_PC4_IDX1 = usize(0x00060000);  // IDX1 on PC4
pub const GPIO_PCTL_PC4_WT0CCP0 = usize(0x00070000);  // WT0CCP0 on PC4
pub const GPIO_PCTL_PC4_U1RTS = usize(0x00080000);  // U1RTS on PC4
pub const GPIO_PCTL_PC3_M = usize(0x0000F000);  // PC3 Mask
pub const GPIO_PCTL_PC3_TDO = usize(0x00001000);  // TDO on PC3
pub const GPIO_PCTL_PC3_T5CCP1 = usize(0x00007000);  // T5CCP1 on PC3
pub const GPIO_PCTL_PC2_M = usize(0x00000F00);  // PC2 Mask
pub const GPIO_PCTL_PC2_TDI = usize(0x00000100);  // TDI on PC2
pub const GPIO_PCTL_PC2_T5CCP0 = usize(0x00000700);  // T5CCP0 on PC2
pub const GPIO_PCTL_PC1_M = usize(0x000000F0);  // PC1 Mask
pub const GPIO_PCTL_PC1_TMS = usize(0x00000010);  // TMS on PC1
pub const GPIO_PCTL_PC1_T4CCP1 = usize(0x00000070);  // T4CCP1 on PC1
pub const GPIO_PCTL_PC0_M = usize(0x0000000F);  // PC0 Mask
pub const GPIO_PCTL_PC0_TCK = usize(0x00000001);  // TCK on PC0
pub const GPIO_PCTL_PC0_T4CCP0 = usize(0x00000007);  // T4CCP0 on PC0

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port D.
//
//*****************************************************************************
pub const GPIO_PCTL_PD7_M = usize(0xF0000000);  // PD7 Mask
pub const GPIO_PCTL_PD7_U2TX = usize(0x10000000);  // U2TX on PD7
pub const GPIO_PCTL_PD7_M0FAULT1 = usize(0x40000000);  // M0FAULT1 on PD7
pub const GPIO_PCTL_PD7_PHB0 = usize(0x60000000);  // PHB0 on PD7
pub const GPIO_PCTL_PD7_WT5CCP1 = usize(0x70000000);  // WT5CCP1 on PD7
pub const GPIO_PCTL_PD7_NMI = usize(0x80000000);  // NMI on PD7
pub const GPIO_PCTL_PD6_M = usize(0x0F000000);  // PD6 Mask
pub const GPIO_PCTL_PD6_U2RX = usize(0x01000000);  // U2RX on PD6
pub const GPIO_PCTL_PD6_M0FAULT0 = usize(0x04000000);  // M0FAULT0 on PD6
pub const GPIO_PCTL_PD6_PHA0 = usize(0x06000000);  // PHA0 on PD6
pub const GPIO_PCTL_PD6_WT5CCP0 = usize(0x07000000);  // WT5CCP0 on PD6
pub const GPIO_PCTL_PD5_M = usize(0x00F00000);  // PD5 Mask
pub const GPIO_PCTL_PD5_USB0DP = usize(0x00000000);  // USB0DP on PD5
pub const GPIO_PCTL_PD5_U6TX = usize(0x00100000);  // U6TX on PD5
pub const GPIO_PCTL_PD5_WT4CCP1 = usize(0x00700000);  // WT4CCP1 on PD5
pub const GPIO_PCTL_PD4_M = usize(0x000F0000);  // PD4 Mask
pub const GPIO_PCTL_PD4_USB0DM = usize(0x00000000);  // USB0DM on PD4
pub const GPIO_PCTL_PD4_U6RX = usize(0x00010000);  // U6RX on PD4
pub const GPIO_PCTL_PD4_WT4CCP0 = usize(0x00070000);  // WT4CCP0 on PD4
pub const GPIO_PCTL_PD3_M = usize(0x0000F000);  // PD3 Mask
pub const GPIO_PCTL_PD3_AIN4 = usize(0x00000000);  // AIN4 on PD3
pub const GPIO_PCTL_PD3_SSI3TX = usize(0x00001000);  // SSI3TX on PD3
pub const GPIO_PCTL_PD3_SSI1TX = usize(0x00002000);  // SSI1TX on PD3
pub const GPIO_PCTL_PD3_IDX0 = usize(0x00006000);  // IDX0 on PD3
pub const GPIO_PCTL_PD3_WT3CCP1 = usize(0x00007000);  // WT3CCP1 on PD3
pub const GPIO_PCTL_PD3_USB0PFLT = usize(0x00008000);  // USB0PFLT on PD3
pub const GPIO_PCTL_PD2_M = usize(0x00000F00);  // PD2 Mask
pub const GPIO_PCTL_PD2_AIN5 = usize(0x00000000);  // AIN5 on PD2
pub const GPIO_PCTL_PD2_SSI3RX = usize(0x00000100);  // SSI3RX on PD2
pub const GPIO_PCTL_PD2_SSI1RX = usize(0x00000200);  // SSI1RX on PD2
pub const GPIO_PCTL_PD2_M0FAULT0 = usize(0x00000400);  // M0FAULT0 on PD2
pub const GPIO_PCTL_PD2_WT3CCP0 = usize(0x00000700);  // WT3CCP0 on PD2
pub const GPIO_PCTL_PD2_USB0EPEN = usize(0x00000800);  // USB0EPEN on PD2
pub const GPIO_PCTL_PD1_M = usize(0x000000F0);  // PD1 Mask
pub const GPIO_PCTL_PD1_AIN6 = usize(0x00000000);  // AIN6 on PD1
pub const GPIO_PCTL_PD1_SSI3FSS = usize(0x00000010);  // SSI3FSS on PD1
pub const GPIO_PCTL_PD1_SSI1FSS = usize(0x00000020);  // SSI1FSS on PD1
pub const GPIO_PCTL_PD1_I2C3SDA = usize(0x00000030);  // I2C3SDA on PD1
pub const GPIO_PCTL_PD1_M0PWM7 = usize(0x00000040);  // M0PWM7 on PD1
pub const GPIO_PCTL_PD1_M1PWM1 = usize(0x00000050);  // M1PWM1 on PD1
pub const GPIO_PCTL_PD1_WT2CCP1 = usize(0x00000070);  // WT2CCP1 on PD1
pub const GPIO_PCTL_PD0_M = usize(0x0000000F);  // PD0 Mask
pub const GPIO_PCTL_PD0_AIN7 = usize(0x00000000);  // AIN7 on PD0
pub const GPIO_PCTL_PD0_SSI3CLK = usize(0x00000001);  // SSI3CLK on PD0
pub const GPIO_PCTL_PD0_SSI1CLK = usize(0x00000002);  // SSI1CLK on PD0
pub const GPIO_PCTL_PD0_I2C3SCL = usize(0x00000003);  // I2C3SCL on PD0
pub const GPIO_PCTL_PD0_M0PWM6 = usize(0x00000004);  // M0PWM6 on PD0
pub const GPIO_PCTL_PD0_M1PWM0 = usize(0x00000005);  // M1PWM0 on PD0
pub const GPIO_PCTL_PD0_WT2CCP0 = usize(0x00000007);  // WT2CCP0 on PD0

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port E.
//
//*****************************************************************************
pub const GPIO_PCTL_PE5_M = usize(0x00F00000);  // PE5 Mask
pub const GPIO_PCTL_PE5_AIN8 = usize(0x00000000);  // AIN8 on PE5
pub const GPIO_PCTL_PE5_U5TX = usize(0x00100000);  // U5TX on PE5
pub const GPIO_PCTL_PE5_I2C2SDA = usize(0x00300000);  // I2C2SDA on PE5
pub const GPIO_PCTL_PE5_M0PWM5 = usize(0x00400000);  // M0PWM5 on PE5
pub const GPIO_PCTL_PE5_M1PWM3 = usize(0x00500000);  // M1PWM3 on PE5
pub const GPIO_PCTL_PE5_CAN0TX = usize(0x00800000);  // CAN0TX on PE5
pub const GPIO_PCTL_PE4_M = usize(0x000F0000);  // PE4 Mask
pub const GPIO_PCTL_PE4_AIN9 = usize(0x00000000);  // AIN9 on PE4
pub const GPIO_PCTL_PE4_U5RX = usize(0x00010000);  // U5RX on PE4
pub const GPIO_PCTL_PE4_I2C2SCL = usize(0x00030000);  // I2C2SCL on PE4
pub const GPIO_PCTL_PE4_M0PWM4 = usize(0x00040000);  // M0PWM4 on PE4
pub const GPIO_PCTL_PE4_M1PWM2 = usize(0x00050000);  // M1PWM2 on PE4
pub const GPIO_PCTL_PE4_CAN0RX = usize(0x00080000);  // CAN0RX on PE4
pub const GPIO_PCTL_PE3_M = usize(0x0000F000);  // PE3 Mask
pub const GPIO_PCTL_PE3_AIN0 = usize(0x00000000);  // AIN0 on PE3
pub const GPIO_PCTL_PE2_M = usize(0x00000F00);  // PE2 Mask
pub const GPIO_PCTL_PE2_AIN1 = usize(0x00000000);  // AIN1 on PE2
pub const GPIO_PCTL_PE1_M = usize(0x000000F0);  // PE1 Mask
pub const GPIO_PCTL_PE1_AIN2 = usize(0x00000000);  // AIN2 on PE1
pub const GPIO_PCTL_PE1_U7TX = usize(0x00000010);  // U7TX on PE1
pub const GPIO_PCTL_PE0_M = usize(0x0000000F);  // PE0 Mask
pub const GPIO_PCTL_PE0_AIN3 = usize(0x00000000);  // AIN3 on PE0
pub const GPIO_PCTL_PE0_U7RX = usize(0x00000001);  // U7RX on PE0

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port F.
//
//*****************************************************************************
pub const GPIO_PCTL_PF4_M = usize(0x000F0000);  // PF4 Mask
pub const GPIO_PCTL_PF4_M0FAULT2 = usize(0x00040000);  // M0FAULT2 on PF4
pub const GPIO_PCTL_PF4_M1FAULT0 = usize(0x00050000);  // M1FAULT0 on PF4
pub const GPIO_PCTL_PF4_IDX0 = usize(0x00060000);  // IDX0 on PF4
pub const GPIO_PCTL_PF4_T2CCP0 = usize(0x00070000);  // T2CCP0 on PF4
pub const GPIO_PCTL_PF4_USB0EPEN = usize(0x00080000);  // USB0EPEN on PF4
pub const GPIO_PCTL_PF3_M = usize(0x0000F000);  // PF3 Mask
pub const GPIO_PCTL_PF3_SSI1FSS = usize(0x00002000);  // SSI1FSS on PF3
pub const GPIO_PCTL_PF3_CAN0TX = usize(0x00003000);  // CAN0TX on PF3
pub const GPIO_PCTL_PF3_M0FAULT1 = usize(0x00004000);  // M0FAULT1 on PF3
pub const GPIO_PCTL_PF3_M1PWM7 = usize(0x00005000);  // M1PWM7 on PF3
pub const GPIO_PCTL_PF3_T1CCP1 = usize(0x00007000);  // T1CCP1 on PF3
pub const GPIO_PCTL_PF3_TRCLK = usize(0x0000E000);  // TRCLK on PF3
pub const GPIO_PCTL_PF2_M = usize(0x00000F00);  // PF2 Mask
pub const GPIO_PCTL_PF2_SSI1CLK = usize(0x00000200);  // SSI1CLK on PF2
pub const GPIO_PCTL_PF2_M0FAULT0 = usize(0x00000400);  // M0FAULT0 on PF2
pub const GPIO_PCTL_PF2_M1PWM6 = usize(0x00000500);  // M1PWM6 on PF2
pub const GPIO_PCTL_PF2_T1CCP0 = usize(0x00000700);  // T1CCP0 on PF2
pub const GPIO_PCTL_PF2_TRD0 = usize(0x00000E00);  // TRD0 on PF2
pub const GPIO_PCTL_PF1_M = usize(0x000000F0);  // PF1 Mask
pub const GPIO_PCTL_PF1_U1CTS = usize(0x00000010);  // U1CTS on PF1
pub const GPIO_PCTL_PF1_SSI1TX = usize(0x00000020);  // SSI1TX on PF1
pub const GPIO_PCTL_PF1_M1PWM5 = usize(0x00000050);  // M1PWM5 on PF1
pub const GPIO_PCTL_PF1_PHB0 = usize(0x00000060);  // PHB0 on PF1
pub const GPIO_PCTL_PF1_T0CCP1 = usize(0x00000070);  // T0CCP1 on PF1
pub const GPIO_PCTL_PF1_C1O = usize(0x00000090);  // C1O on PF1
pub const GPIO_PCTL_PF1_TRD1 = usize(0x000000E0);  // TRD1 on PF1
pub const GPIO_PCTL_PF0_M = usize(0x0000000F);  // PF0 Mask
pub const GPIO_PCTL_PF0_U1RTS = usize(0x00000001);  // U1RTS on PF0
pub const GPIO_PCTL_PF0_SSI1RX = usize(0x00000002);  // SSI1RX on PF0
pub const GPIO_PCTL_PF0_CAN0RX = usize(0x00000003);  // CAN0RX on PF0
pub const GPIO_PCTL_PF0_M1PWM4 = usize(0x00000005);  // M1PWM4 on PF0
pub const GPIO_PCTL_PF0_PHA0 = usize(0x00000006);  // PHA0 on PF0
pub const GPIO_PCTL_PF0_T0CCP0 = usize(0x00000007);  // T0CCP0 on PF0
pub const GPIO_PCTL_PF0_NMI = usize(0x00000008);  // NMI on PF0
pub const GPIO_PCTL_PF0_C0O = usize(0x00000009);  // C0O on PF0

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port G.
//
//*****************************************************************************
pub const GPIO_PCTL_PG5_M = usize(0x00F00000);  // PG5 Mask
pub const GPIO_PCTL_PG5_U2TX = usize(0x00100000);  // U2TX on PG5
pub const GPIO_PCTL_PG5_I2C1SDA = usize(0x00300000);  // I2C1SDA on PG5
pub const GPIO_PCTL_PG5_M0PWM5 = usize(0x00400000);  // M0PWM5 on PG5
pub const GPIO_PCTL_PG5_M1PWM3 = usize(0x00500000);  // M1PWM3 on PG5
pub const GPIO_PCTL_PG5_IDX1 = usize(0x00600000);  // IDX1 on PG5
pub const GPIO_PCTL_PG5_WT0CCP1 = usize(0x00700000);  // WT0CCP1 on PG5
pub const GPIO_PCTL_PG5_USB0PFLT = usize(0x00800000);  // USB0PFLT on PG5
pub const GPIO_PCTL_PG4_M = usize(0x000F0000);  // PG4 Mask
pub const GPIO_PCTL_PG4_U2RX = usize(0x00010000);  // U2RX on PG4
pub const GPIO_PCTL_PG4_I2C1SCL = usize(0x00030000);  // I2C1SCL on PG4
pub const GPIO_PCTL_PG4_M0PWM4 = usize(0x00040000);  // M0PWM4 on PG4
pub const GPIO_PCTL_PG4_M1PWM2 = usize(0x00050000);  // M1PWM2 on PG4
pub const GPIO_PCTL_PG4_PHB1 = usize(0x00060000);  // PHB1 on PG4
pub const GPIO_PCTL_PG4_WT0CCP0 = usize(0x00070000);  // WT0CCP0 on PG4
pub const GPIO_PCTL_PG4_USB0EPEN = usize(0x00080000);  // USB0EPEN on PG4
pub const GPIO_PCTL_PG3_M = usize(0x0000F000);  // PG3 Mask
pub const GPIO_PCTL_PG3_I2C4SDA = usize(0x00003000);  // I2C4SDA on PG3
pub const GPIO_PCTL_PG3_M0FAULT2 = usize(0x00004000);  // M0FAULT2 on PG3
pub const GPIO_PCTL_PG3_M1PWM1 = usize(0x00005000);  // M1PWM1 on PG3
pub const GPIO_PCTL_PG3_PHA1 = usize(0x00006000);  // PHA1 on PG3
pub const GPIO_PCTL_PG3_T5CCP1 = usize(0x00007000);  // T5CCP1 on PG3
pub const GPIO_PCTL_PG2_M = usize(0x00000F00);  // PG2 Mask
pub const GPIO_PCTL_PG2_I2C4SCL = usize(0x00000300);  // I2C4SCL on PG2
pub const GPIO_PCTL_PG2_M0FAULT1 = usize(0x00000400);  // M0FAULT1 on PG2
pub const GPIO_PCTL_PG2_M1PWM0 = usize(0x00000500);  // M1PWM0 on PG2
pub const GPIO_PCTL_PG2_T5CCP0 = usize(0x00000700);  // T5CCP0 on PG2
pub const GPIO_PCTL_PG1_M = usize(0x000000F0);  // PG1 Mask
pub const GPIO_PCTL_PG1_I2C3SDA = usize(0x00000030);  // I2C3SDA on PG1
pub const GPIO_PCTL_PG1_M1FAULT2 = usize(0x00000050);  // M1FAULT2 on PG1
pub const GPIO_PCTL_PG1_PHB1 = usize(0x00000060);  // PHB1 on PG1
pub const GPIO_PCTL_PG1_T4CCP1 = usize(0x00000070);  // T4CCP1 on PG1
pub const GPIO_PCTL_PG0_M = usize(0x0000000F);  // PG0 Mask
pub const GPIO_PCTL_PG0_I2C3SCL = usize(0x00000003);  // I2C3SCL on PG0
pub const GPIO_PCTL_PG0_M1FAULT1 = usize(0x00000005);  // M1FAULT1 on PG0
pub const GPIO_PCTL_PG0_PHA1 = usize(0x00000006);  // PHA1 on PG0
pub const GPIO_PCTL_PG0_T4CCP0 = usize(0x00000007);  // T4CCP0 on PG0

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
pub const SSI_CR0_FRF_NMW = usize(0x00000020);  // MICROWIRE Frame Format
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
pub const UART_FR_TXFE = usize(0x00000080);  // UART Transmit FIFO Empty
pub const UART_FR_RXFF = usize(0x00000040);  // UART Receive FIFO Full
pub const UART_FR_TXFF = usize(0x00000020);  // UART Transmit FIFO Full
pub const UART_FR_RXFE = usize(0x00000010);  // UART Receive FIFO Empty
pub const UART_FR_BUSY = usize(0x00000008);  // UART Busy
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
pub const UART_IM_9BITIM = usize(0x00001000);  // 9-Bit Mode Interrupt Mask
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
pub const UART_IM_CTSMIM = usize(0x00000002);  // UART Clear to Send Modem
                                            // Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RIS register.
//
//*****************************************************************************
pub const UART_RIS_9BITRIS = usize(0x00001000);  // 9-Bit Mode Raw Interrupt Status
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
pub const UART_RIS_CTSRIS = usize(0x00000002);  // UART Clear to Send Modem Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_MIS register.
//
//*****************************************************************************
pub const UART_MIS_9BITMIS = usize(0x00001000);  // 9-Bit Mode Masked Interrupt
                                            // Status
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
pub const UART_MIS_CTSMIS = usize(0x00000002);  // UART Clear to Send Modem Masked
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ICR register.
//
//*****************************************************************************
pub const UART_ICR_9BITIC = usize(0x00001000);  // 9-Bit Mode Interrupt Clear
pub const UART_ICR_OEIC = usize(0x00000400);  // Overrun Error Interrupt Clear
pub const UART_ICR_BEIC = usize(0x00000200);  // Break Error Interrupt Clear
pub const UART_ICR_PEIC = usize(0x00000100);  // Parity Error Interrupt Clear
pub const UART_ICR_FEIC = usize(0x00000080);  // Framing Error Interrupt Clear
pub const UART_ICR_RTIC = usize(0x00000040);  // Receive Time-Out Interrupt Clear
pub const UART_ICR_TXIC = usize(0x00000020);  // Transmit Interrupt Clear
pub const UART_ICR_RXIC = usize(0x00000010);  // Receive Interrupt Clear
pub const UART_ICR_CTSMIC = usize(0x00000002);  // UART Clear to Send Modem
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
pub const I2C_MCS_CLKTO = usize(0x00000080);  // Clock Timeout Error
pub const I2C_MCS_BUSBSY = usize(0x00000040);  // Bus Busy
pub const I2C_MCS_IDLE = usize(0x00000020);  // I2C Idle
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
pub const I2C_MTPR_HS = usize(0x00000080);  // High-Speed Enable
pub const I2C_MTPR_TPR_M = usize(0x0000007F);  // Timer Period
pub const I2C_MTPR_TPR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MIMR register.
//
//*****************************************************************************
pub const I2C_MIMR_CLKIM = usize(0x00000002);  // Clock Timeout Interrupt Mask
pub const I2C_MIMR_IM = usize(0x00000001);  // Master Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MRIS register.
//
//*****************************************************************************
pub const I2C_MRIS_CLKRIS = usize(0x00000002);  // Clock Timeout Raw Interrupt
                                            // Status
pub const I2C_MRIS_RIS = usize(0x00000001);  // Master Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MMIS register.
//
//*****************************************************************************
pub const I2C_MMIS_CLKMIS = usize(0x00000002);  // Clock Timeout Masked Interrupt
                                            // Status
pub const I2C_MMIS_MIS = usize(0x00000001);  // Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MICR register.
//
//*****************************************************************************
pub const I2C_MICR_CLKIC = usize(0x00000002);  // Clock Timeout Interrupt Clear
pub const I2C_MICR_IC = usize(0x00000001);  // Master Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCR register.
//
//*****************************************************************************
pub const I2C_MCR_GFE = usize(0x00000040);  // I2C Glitch Filter Enable
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
// The following are defines for the bit fields in the I2C_O_MCR2 register.
//
//*****************************************************************************
pub const I2C_MCR2_GFPW_M = usize(0x00000070);  // I2C Glitch Filter Pulse Width
pub const I2C_MCR2_GFPW_BYPASS = usize(0x00000000);  // Bypass
pub const I2C_MCR2_GFPW_1 = usize(0x00000010);  // 1 clock
pub const I2C_MCR2_GFPW_2 = usize(0x00000020);  // 2 clocks
pub const I2C_MCR2_GFPW_3 = usize(0x00000030);  // 3 clocks
pub const I2C_MCR2_GFPW_4 = usize(0x00000040);  // 4 clocks
pub const I2C_MCR2_GFPW_8 = usize(0x00000050);  // 8 clocks
pub const I2C_MCR2_GFPW_16 = usize(0x00000060);  // 16 clocks
pub const I2C_MCR2_GFPW_31 = usize(0x00000070);  // 31 clocks

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
pub const I2C_SCSR_OAR2SEL = usize(0x00000008);  // OAR2 Address Matched
pub const I2C_SCSR_FBR = usize(0x00000004);  // First Byte Received
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
pub const I2C_SIMR_STOPIM = usize(0x00000004);  // Stop Condition Interrupt Mask
pub const I2C_SIMR_STARTIM = usize(0x00000002);  // Start Condition Interrupt Mask
pub const I2C_SIMR_DATAIM = usize(0x00000001);  // Data Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SRIS register.
//
//*****************************************************************************
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
pub const TIMER_SYNC_SYNCWT5_M = usize(0x00C00000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 5
pub const TIMER_SYNC_SYNCWT5_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 5 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT5_TA = usize(0x00400000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 5 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT5_TB = usize(0x00800000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 5 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT5_TATB = usize(0x00C00000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 5 is triggered
pub const TIMER_SYNC_SYNCWT4_M = usize(0x00300000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 4
pub const TIMER_SYNC_SYNCWT4_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 4 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT4_TA = usize(0x00100000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 4 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT4_TB = usize(0x00200000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 4 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT4_TATB = usize(0x00300000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 4 is triggered
pub const TIMER_SYNC_SYNCWT3_M = usize(0x000C0000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 3
pub const TIMER_SYNC_SYNCWT3_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 3 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT3_TA = usize(0x00040000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 3 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT3_TB = usize(0x00080000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 3 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT3_TATB = usize(0x000C0000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 3 is triggered
pub const TIMER_SYNC_SYNCWT2_M = usize(0x00030000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 2
pub const TIMER_SYNC_SYNCWT2_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 2 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT2_TA = usize(0x00010000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 2 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT2_TB = usize(0x00020000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 2 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT2_TATB = usize(0x00030000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 2 is triggered
pub const TIMER_SYNC_SYNCWT1_M = usize(0x0000C000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 1
pub const TIMER_SYNC_SYNCWT1_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 1 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT1_TA = usize(0x00004000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 1 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT1_TB = usize(0x00008000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 1 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT1_TATB = usize(0x0000C000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 1 is triggered
pub const TIMER_SYNC_SYNCWT0_M = usize(0x00003000);  // Synchronize GPTM 32/64-Bit Timer
                                            // 0
pub const TIMER_SYNC_SYNCWT0_NONE = usize(0x00000000);  // GPTM 32/64-Bit Timer 0 is not
                                            // affected
pub const TIMER_SYNC_SYNCWT0_TA = usize(0x00001000);  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 0 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT0_TB = usize(0x00002000);  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 0 is
                                            // triggered
pub const TIMER_SYNC_SYNCWT0_TATB = usize(0x00003000);  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 0 is triggered
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
pub const TIMER_IMR_WUEIM = usize(0x00010000);  // 32/64-Bit Wide GPTM Write Update
                                            // Error Interrupt Mask
pub const TIMER_IMR_TBMIM = usize(0x00000800);  // GPTM Timer B Match Interrupt
                                            // Mask
pub const TIMER_IMR_CBEIM = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Interrupt Mask
pub const TIMER_IMR_CBMIM = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Interrupt Mask
pub const TIMER_IMR_TBTOIM = usize(0x00000100);  // GPTM Timer B Time-Out Interrupt
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
pub const TIMER_RIS_WUERIS = usize(0x00010000);  // 32/64-Bit Wide GPTM Write Update
                                            // Error Raw Interrupt Status
pub const TIMER_RIS_TBMRIS = usize(0x00000800);  // GPTM Timer B Match Raw Interrupt
pub const TIMER_RIS_CBERIS = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Raw Interrupt
pub const TIMER_RIS_CBMRIS = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Raw Interrupt
pub const TIMER_RIS_TBTORIS = usize(0x00000100);  // GPTM Timer B Time-Out Raw
                                            // Interrupt
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
pub const TIMER_MIS_WUEMIS = usize(0x00010000);  // 32/64-Bit Wide GPTM Write Update
                                            // Error Masked Interrupt Status
pub const TIMER_MIS_TBMMIS = usize(0x00000800);  // GPTM Timer B Match Masked
                                            // Interrupt
pub const TIMER_MIS_CBEMIS = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Masked Interrupt
pub const TIMER_MIS_CBMMIS = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Masked Interrupt
pub const TIMER_MIS_TBTOMIS = usize(0x00000100);  // GPTM Timer B Time-Out Masked
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
pub const TIMER_ICR_WUECINT = usize(0x00010000);  // 32/64-Bit Wide GPTM Write Update
                                            // Error Interrupt Clear
pub const TIMER_ICR_TBMCINT = usize(0x00000800);  // GPTM Timer B Match Interrupt
                                            // Clear
pub const TIMER_ICR_CBECINT = usize(0x00000400);  // GPTM Timer B Capture Mode Event
                                            // Interrupt Clear
pub const TIMER_ICR_CBMCINT = usize(0x00000200);  // GPTM Timer B Capture Mode Match
                                            // Interrupt Clear
pub const TIMER_ICR_TBTOCINT = usize(0x00000100);  // GPTM Timer B Time-Out Interrupt
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
pub const TIMER_TAPR_TAPSRH_M = usize(0x0000FF00);  // GPTM Timer A Prescale High Byte
pub const TIMER_TAPR_TAPSR_M = usize(0x000000FF);  // GPTM Timer A Prescale
pub const TIMER_TAPR_TAPSRH_S = usize(8);
pub const TIMER_TAPR_TAPSR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPR register.
//
//*****************************************************************************
pub const TIMER_TBPR_TBPSRH_M = usize(0x0000FF00);  // GPTM Timer B Prescale High Byte
pub const TIMER_TBPR_TBPSR_M = usize(0x000000FF);  // GPTM Timer B Prescale
pub const TIMER_TBPR_TBPSRH_S = usize(8);
pub const TIMER_TBPR_TBPSR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPMR register.
//
//*****************************************************************************
pub const TIMER_TAPMR_TAPSMRH_M = usize(0x0000FF00);  // GPTM Timer A Prescale Match High
                                            // Byte
pub const TIMER_TAPMR_TAPSMR_M = usize(0x000000FF);  // GPTM TimerA Prescale Match
pub const TIMER_TAPMR_TAPSMRH_S = usize(8);
pub const TIMER_TAPMR_TAPSMR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPMR register.
//
//*****************************************************************************
pub const TIMER_TBPMR_TBPSMRH_M = usize(0x0000FF00);  // GPTM Timer B Prescale Match High
                                            // Byte
pub const TIMER_TBPMR_TBPSMR_M = usize(0x000000FF);  // GPTM TimerB Prescale Match
pub const TIMER_TBPMR_TBPSMRH_S = usize(8);
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
// The following are defines for the bit fields in the TIMER_O_TAPV register.
//
//*****************************************************************************
pub const TIMER_TAPV_PSV_M = usize(0x0000FFFF);  // GPTM Timer A Prescaler Value
pub const TIMER_TAPV_PSV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPV register.
//
//*****************************************************************************
pub const TIMER_TBPV_PSV_M = usize(0x0000FFFF);  // GPTM Timer B Prescaler Value
pub const TIMER_TBPV_PSV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_PP register.
//
//*****************************************************************************
pub const TIMER_PP_SIZE_M = usize(0x0000000F);  // Count Size
pub const TIMER_PP_SIZE_16 = usize(0x00000000);  // Timer A and Timer B counters are
                                            // 16 bits each with an 8-bit
                                            // prescale counter
pub const TIMER_PP_SIZE_32 = usize(0x00000001);  // Timer A and Timer B counters are
                                            // 32 bits each with a 16-bit
                                            // prescale counter

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_ACTSS register.
//
//*****************************************************************************
pub const ADC_ACTSS_BUSY = usize(0x00010000);  // ADC Busy
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
pub const ADC_EMUX_EM3_EXTERNAL = usize(0x00004000);  // External (GPIO Pins)
pub const ADC_EMUX_EM3_TIMER = usize(0x00005000);  // Timer
pub const ADC_EMUX_EM3_PWM0 = usize(0x00006000);  // PWM generator 0
pub const ADC_EMUX_EM3_PWM1 = usize(0x00007000);  // PWM generator 1
pub const ADC_EMUX_EM3_PWM2 = usize(0x00008000);  // PWM generator 2
pub const ADC_EMUX_EM3_PWM3 = usize(0x00009000);  // PWM generator 3
pub const ADC_EMUX_EM3_ALWAYS = usize(0x0000F000);  // Always (continuously sample)
pub const ADC_EMUX_EM2_M = usize(0x00000F00);  // SS2 Trigger Select
pub const ADC_EMUX_EM2_PROCESSOR = usize(0x00000000);  // Processor (default)
pub const ADC_EMUX_EM2_COMP0 = usize(0x00000100);  // Analog Comparator 0
pub const ADC_EMUX_EM2_COMP1 = usize(0x00000200);  // Analog Comparator 1
pub const ADC_EMUX_EM2_EXTERNAL = usize(0x00000400);  // External (GPIO Pins)
pub const ADC_EMUX_EM2_TIMER = usize(0x00000500);  // Timer
pub const ADC_EMUX_EM2_PWM0 = usize(0x00000600);  // PWM generator 0
pub const ADC_EMUX_EM2_PWM1 = usize(0x00000700);  // PWM generator 1
pub const ADC_EMUX_EM2_PWM2 = usize(0x00000800);  // PWM generator 2
pub const ADC_EMUX_EM2_PWM3 = usize(0x00000900);  // PWM generator 3
pub const ADC_EMUX_EM2_ALWAYS = usize(0x00000F00);  // Always (continuously sample)
pub const ADC_EMUX_EM1_M = usize(0x000000F0);  // SS1 Trigger Select
pub const ADC_EMUX_EM1_PROCESSOR = usize(0x00000000);  // Processor (default)
pub const ADC_EMUX_EM1_COMP0 = usize(0x00000010);  // Analog Comparator 0
pub const ADC_EMUX_EM1_COMP1 = usize(0x00000020);  // Analog Comparator 1
pub const ADC_EMUX_EM1_EXTERNAL = usize(0x00000040);  // External (GPIO Pins)
pub const ADC_EMUX_EM1_TIMER = usize(0x00000050);  // Timer
pub const ADC_EMUX_EM1_PWM0 = usize(0x00000060);  // PWM generator 0
pub const ADC_EMUX_EM1_PWM1 = usize(0x00000070);  // PWM generator 1
pub const ADC_EMUX_EM1_PWM2 = usize(0x00000080);  // PWM generator 2
pub const ADC_EMUX_EM1_PWM3 = usize(0x00000090);  // PWM generator 3
pub const ADC_EMUX_EM1_ALWAYS = usize(0x000000F0);  // Always (continuously sample)
pub const ADC_EMUX_EM0_M = usize(0x0000000F);  // SS0 Trigger Select
pub const ADC_EMUX_EM0_PROCESSOR = usize(0x00000000);  // Processor (default)
pub const ADC_EMUX_EM0_COMP0 = usize(0x00000001);  // Analog Comparator 0
pub const ADC_EMUX_EM0_COMP1 = usize(0x00000002);  // Analog Comparator 1
pub const ADC_EMUX_EM0_EXTERNAL = usize(0x00000004);  // External (GPIO Pins)
pub const ADC_EMUX_EM0_TIMER = usize(0x00000005);  // Timer
pub const ADC_EMUX_EM0_PWM0 = usize(0x00000006);  // PWM generator 0
pub const ADC_EMUX_EM0_PWM1 = usize(0x00000007);  // PWM generator 1
pub const ADC_EMUX_EM0_PWM2 = usize(0x00000008);  // PWM generator 2
pub const ADC_EMUX_EM0_PWM3 = usize(0x00000009);  // PWM generator 3
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
pub const ADC_TSSEL_PS3_1 = usize(0x10000000);  // Use Generator 3 (and its
                                            // trigger) in PWM module 1
pub const ADC_TSSEL_PS2_M = usize(0x00300000);  // Generator 2 PWM Module Trigger
                                            // Select
pub const ADC_TSSEL_PS2_0 = usize(0x00000000);  // Use Generator 2 (and its
                                            // trigger) in PWM module 0
pub const ADC_TSSEL_PS2_1 = usize(0x00100000);  // Use Generator 2 (and its
                                            // trigger) in PWM module 1
pub const ADC_TSSEL_PS1_M = usize(0x00003000);  // Generator 1 PWM Module Trigger
                                            // Select
pub const ADC_TSSEL_PS1_0 = usize(0x00000000);  // Use Generator 1 (and its
                                            // trigger) in PWM module 0
pub const ADC_TSSEL_PS1_1 = usize(0x00001000);  // Use Generator 1 (and its
                                            // trigger) in PWM module 1
pub const ADC_TSSEL_PS0_M = usize(0x00000030);  // Generator 0 PWM Module Trigger
                                            // Select
pub const ADC_TSSEL_PS0_0 = usize(0x00000000);  // Use Generator 0 (and its
                                            // trigger) in PWM module 0
pub const ADC_TSSEL_PS0_1 = usize(0x00000010);  // Use Generator 0 (and its
                                            // trigger) in PWM module 1

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
pub const ADC_PP_TS = usize(0x00800000);  // Temperature Sensor
pub const ADC_PP_RSL_M = usize(0x007C0000);  // Resolution
pub const ADC_PP_TYPE_M = usize(0x00030000);  // ADC Architecture
pub const ADC_PP_TYPE_SAR = usize(0x00000000);  // SAR
pub const ADC_PP_DC_M = usize(0x0000FC00);  // Digital Comparator Count
pub const ADC_PP_CH_M = usize(0x000003F0);  // ADC Channel Count
pub const ADC_PP_MSR_M = usize(0x0000000F);  // Maximum ADC Sample Rate
pub const ADC_PP_MSR_125K = usize(0x00000001);  // 125 ksps
pub const ADC_PP_MSR_250K = usize(0x00000003);  // 250 ksps
pub const ADC_PP_MSR_500K = usize(0x00000005);  // 500 ksps
pub const ADC_PP_MSR_1M = usize(0x00000007);  // 1 Msps
pub const ADC_PP_RSL_S = usize(18);
pub const ADC_PP_DC_S = usize(10);
pub const ADC_PP_CH_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PC register.
//
//*****************************************************************************
pub const ADC_PC_SR_M = usize(0x0000000F);  // ADC Sample Rate
pub const ADC_PC_SR_125K = usize(0x00000001);  // 125 ksps
pub const ADC_PC_SR_250K = usize(0x00000003);  // 250 ksps
pub const ADC_PC_SR_500K = usize(0x00000005);  // 500 ksps
pub const ADC_PC_SR_1M = usize(0x00000007);  // 1 Msps

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_CC register.
//
//*****************************************************************************
pub const ADC_CC_CS_M = usize(0x0000000F);  // ADC Clock Source
pub const ADC_CC_CS_SYSPLL = usize(0x00000000);  // PLL VCO divided by CLKDIV
pub const ADC_CC_CS_PIOSC = usize(0x00000001);  // PIOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACMIS register.
//
//*****************************************************************************
pub const COMP_ACMIS_IN1 = usize(0x00000002);  // Comparator 1 Masked Interrupt
                                            // Status
pub const COMP_ACMIS_IN0 = usize(0x00000001);  // Comparator 0 Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACRIS register.
//
//*****************************************************************************
pub const COMP_ACRIS_IN1 = usize(0x00000002);  // Comparator 1 Interrupt Status
pub const COMP_ACRIS_IN0 = usize(0x00000001);  // Comparator 0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACINTEN register.
//
//*****************************************************************************
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
// The following are defines for the bit fields in the COMP_O_PP register.
//
//*****************************************************************************
pub const COMP_PP_C1O = usize(0x00020000);  // Comparator Output 1 Present
pub const COMP_PP_C0O = usize(0x00010000);  // Comparator Output 0 Present
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
pub const USB_GPCS_DEVMODOTG = usize(0x00000002);  // Enable Device Mode
pub const USB_GPCS_DEVMOD = usize(0x00000001);  // Device Mode

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
// The following are defines for the bit fields in the USB_O_IDVRIS register.
//
//*****************************************************************************
pub const USB_IDVRIS_ID = usize(0x00000001);  // ID Valid Detect Raw Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IDVIM register.
//
//*****************************************************************************
pub const USB_IDVIM_ID = usize(0x00000001);  // ID Valid Detect Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IDVISC register.
//
//*****************************************************************************
pub const USB_IDVISC_ID = usize(0x00000001);  // ID Valid Detect Interrupt Status
                                            // and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMASEL register.
//
//*****************************************************************************
pub const USB_DMASEL_DMACTX_M = usize(0x00F00000);  // DMA C TX Select
pub const USB_DMASEL_DMACRX_M = usize(0x000F0000);  // DMA C RX Select
pub const USB_DMASEL_DMABTX_M = usize(0x0000F000);  // DMA B TX Select
pub const USB_DMASEL_DMABRX_M = usize(0x00000F00);  // DMA B RX Select
pub const USB_DMASEL_DMAATX_M = usize(0x000000F0);  // DMA A TX Select
pub const USB_DMASEL_DMAARX_M = usize(0x0000000F);  // DMA A RX Select
pub const USB_DMASEL_DMACTX_S = usize(20);
pub const USB_DMASEL_DMACRX_S = usize(16);
pub const USB_DMASEL_DMABTX_S = usize(12);
pub const USB_DMASEL_DMABRX_S = usize(8);
pub const USB_DMASEL_DMAATX_S = usize(4);
pub const USB_DMASEL_DMAARX_S = usize(0);

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
pub const USB_PP_PHY = usize(0x00000010);  // PHY Present
pub const USB_PP_TYPE_M = usize(0x0000000F);  // Controller Type
pub const USB_PP_TYPE_0 = usize(0x00000000);  // The first-generation USB
                                            // controller
pub const USB_PP_ECNT_S = usize(8);

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
// The following are defines for the bit fields in the EEPROM_EEHIDE register.
//
//*****************************************************************************
pub const EEPROM_EEHIDE_HN_M = usize(0xFFFFFFFE);  // Hide Block

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
pub const EEPROM_PP_SIZE_M = usize(0x0000001F);  // EEPROM Size
pub const EEPROM_PP_SIZE_S = usize(0);

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
// The following are defines for the bit fields in the FLASH_FMA register.
//
//*****************************************************************************
pub const FLASH_FMA_OFFSET_M = usize(0x0003FFFF);  // Address Offset
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
// The following are defines for the bit fields in the FLASH_FWBN register.
//
//*****************************************************************************
pub const FLASH_FWBN_DATA_M = usize(0xFFFFFFFF);  // Data

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FSIZE register.
//
//*****************************************************************************
pub const FLASH_FSIZE_SIZE_M = usize(0x0000FFFF);  // Flash Size
pub const FLASH_FSIZE_SIZE_256KB = usize(0x0000007F);  // 256 KB of Flash

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_SSIZE register.
//
//*****************************************************************************
pub const FLASH_SSIZE_SIZE_M = usize(0x0000FFFF);  // SRAM Size
pub const FLASH_SSIZE_SIZE_32KB = usize(0x0000007F);  // 32 KB of SRAM

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_ROMSWMAP register.
//
//*****************************************************************************
pub const FLASH_ROMSWMAP_SAFERTOS = usize(0x00000001);  // SafeRTOS Present

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
// The following are defines for the bit fields in the SYSCTL_DID0 register.
//
//*****************************************************************************
pub const SYSCTL_DID0_VER_M = usize(0x70000000);  // DID0 Version
pub const SYSCTL_DID0_VER_1 = usize(0x10000000);  // Second version of the DID0
                                            // register format.
pub const SYSCTL_DID0_CLASS_M = usize(0x00FF0000);  // Device Class
pub const SYSCTL_DID0_CLASS_TM4C123 = usize(0x00050000);  // Tiva TM4C123x and TM4E123x
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
pub const SYSCTL_DID1_PRTNO_TM4C123FH6PM = usize(0x00B10000);  // TM4C123FH6PM
pub const SYSCTL_DID1_PINCNT_M = usize(0x0000E000);  // Package Pin Count
pub const SYSCTL_DID1_PINCNT_100 = usize(0x00004000);  // 100-pin LQFP package
pub const SYSCTL_DID1_PINCNT_64 = usize(0x00006000);  // 64-pin LQFP package
pub const SYSCTL_DID1_PINCNT_144 = usize(0x00008000);  // 144-pin LQFP package
pub const SYSCTL_DID1_PINCNT_157 = usize(0x0000A000);  // 157-pin BGA package
pub const SYSCTL_DID1_PINCNT_128 = usize(0x0000C000);  // 128-pin TQFP package
pub const SYSCTL_DID1_TEMP_M = usize(0x000000E0);  // Temperature Range
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
// The following are defines for the bit fields in the SYSCTL_SRCR0 register.
//
//*****************************************************************************
pub const SYSCTL_SRCR0_WDT1 = usize(0x10000000);  // WDT1 Reset Control
pub const SYSCTL_SRCR0_CAN1 = usize(0x02000000);  // CAN1 Reset Control
pub const SYSCTL_SRCR0_CAN0 = usize(0x01000000);  // CAN0 Reset Control
pub const SYSCTL_SRCR0_PWM0 = usize(0x00100000);  // PWM Reset Control
pub const SYSCTL_SRCR0_ADC1 = usize(0x00020000);  // ADC1 Reset Control
pub const SYSCTL_SRCR0_ADC0 = usize(0x00010000);  // ADC0 Reset Control
pub const SYSCTL_SRCR0_WDT0 = usize(0x00000008);  // WDT0 Reset Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR1 register.
//
//*****************************************************************************
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
pub const SYSCTL_MISC_BOR1MIS = usize(0x00000002);  // VDD under BOR1 Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RESC register.
//
//*****************************************************************************
pub const SYSCTL_RESC_MOSCFAIL = usize(0x00010000);  // MOSC Failure Reset
pub const SYSCTL_RESC_WDT1 = usize(0x00000020);  // Watchdog Timer 1 Reset
pub const SYSCTL_RESC_SW = usize(0x00000010);  // Software Reset
pub const SYSCTL_RESC_WDT0 = usize(0x00000008);  // Watchdog Timer 0 Reset
pub const SYSCTL_RESC_BOR = usize(0x00000004);  // Brown-Out Reset
pub const SYSCTL_RESC_POR = usize(0x00000002);  // Power-On Reset
pub const SYSCTL_RESC_EXT = usize(0x00000001);  // External Reset

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

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_GPIOHBCTL
// register.
//
//*****************************************************************************
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
pub const SYSCTL_RCC2_SYSDIV2_S = usize(23);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MOSCCTL register.
//
//*****************************************************************************
pub const SYSCTL_MOSCCTL_NOXTAL = usize(0x00000004);  // No Crystal Connected
pub const SYSCTL_MOSCCTL_MOSCIM = usize(0x00000002);  // MOSC Failure Action
pub const SYSCTL_MOSCCTL_CVAL = usize(0x00000001);  // Clock Validation for MOSC

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
pub const SYSCTL_RCGC0_WDT0 = usize(0x00000008);  // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC1 register.
//
//*****************************************************************************
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
pub const SYSCTL_SCGC0_WDT0 = usize(0x00000008);  // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC1 register.
//
//*****************************************************************************
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
pub const SYSCTL_DCGC0_WDT0 = usize(0x00000008);  // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC1 register.
//
//*****************************************************************************
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
pub const SYSCTL_DCGC2_GPIOG = usize(0x00000040);  // Port G Clock Gating Control
pub const SYSCTL_DCGC2_GPIOF = usize(0x00000020);  // Port F Clock Gating Control
pub const SYSCTL_DCGC2_GPIOE = usize(0x00000010);  // Port E Clock Gating Control
pub const SYSCTL_DCGC2_GPIOD = usize(0x00000008);  // Port D Clock Gating Control
pub const SYSCTL_DCGC2_GPIOC = usize(0x00000004);  // Port C Clock Gating Control
pub const SYSCTL_DCGC2_GPIOB = usize(0x00000002);  // Port B Clock Gating Control
pub const SYSCTL_DCGC2_GPIOA = usize(0x00000001);  // Port A Clock Gating Control

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
pub const SYSCTL_DSLPCLKCFG_PIOSCPD = usize(0x00000002);  // PIOSC Power Down Request
pub const SYSCTL_DSLPCLKCFG_D_S = usize(23);

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
pub const SYSCTL_PIOSCCAL_UPDATE = usize(0x00000100);  // Update Trim
pub const SYSCTL_PIOSCCAL_UT_M = usize(0x0000007F);  // User Trim Value
pub const SYSCTL_PIOSCCAL_UT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLFREQ0
// register.
//
//*****************************************************************************
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
// The following are defines for the bit fields in the UDMA_CHIS register.
//
//*****************************************************************************
pub const UDMA_CHIS_M = usize(0xFFFFFFFF);  // Channel [n] Interrupt Status

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
// The following are defines for the bit fields in the NVIC_EN4 register.
//
//*****************************************************************************
pub const NVIC_EN4_INT_M = usize(0x000007FF);  // Interrupt Enable

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
// The following are defines for the bit fields in the NVIC_DIS4 register.
//
//*****************************************************************************
pub const NVIC_DIS4_INT_M = usize(0x000007FF);  // Interrupt Disable

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
// The following are defines for the bit fields in the NVIC_PEND4 register.
//
//*****************************************************************************
pub const NVIC_PEND4_INT_M = usize(0x000007FF);  // Interrupt Set Pending

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
// The following are defines for the bit fields in the NVIC_UNPEND4 register.
//
//*****************************************************************************
pub const NVIC_UNPEND4_INT_M = usize(0x000007FF);  // Interrupt Clear Pending

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
// The following are defines for the bit fields in the NVIC_ACTIVE4 register.
//
//*****************************************************************************
pub const NVIC_ACTIVE4_INT_M = usize(0x000007FF);  // Interrupt Active

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
// The following are defines for the bit fields in the NVIC_PRI29 register.
//
//*****************************************************************************
pub const NVIC_PRI29_INTD_M = usize(0xE0000000);  // Interrupt 119 Priority Mask
pub const NVIC_PRI29_INTC_M = usize(0x00E00000);  // Interrupt 118 Priority Mask
pub const NVIC_PRI29_INTB_M = usize(0x0000E000);  // Interrupt 117 Priority Mask
pub const NVIC_PRI29_INTA_M = usize(0x000000E0);  // Interrupt 116 Priority Mask
pub const NVIC_PRI29_INTD_S = usize(29);
pub const NVIC_PRI29_INTC_S = usize(21);
pub const NVIC_PRI29_INTB_S = usize(13);
pub const NVIC_PRI29_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI30 register.
//
//*****************************************************************************
pub const NVIC_PRI30_INTD_M = usize(0xE0000000);  // Interrupt 123 Priority Mask
pub const NVIC_PRI30_INTC_M = usize(0x00E00000);  // Interrupt 122 Priority Mask
pub const NVIC_PRI30_INTB_M = usize(0x0000E000);  // Interrupt 121 Priority Mask
pub const NVIC_PRI30_INTA_M = usize(0x000000E0);  // Interrupt 120 Priority Mask
pub const NVIC_PRI30_INTD_S = usize(29);
pub const NVIC_PRI30_INTC_S = usize(21);
pub const NVIC_PRI30_INTB_S = usize(13);
pub const NVIC_PRI30_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI31 register.
//
//*****************************************************************************
pub const NVIC_PRI31_INTD_M = usize(0xE0000000);  // Interrupt 127 Priority Mask
pub const NVIC_PRI31_INTC_M = usize(0x00E00000);  // Interrupt 126 Priority Mask
pub const NVIC_PRI31_INTB_M = usize(0x0000E000);  // Interrupt 125 Priority Mask
pub const NVIC_PRI31_INTA_M = usize(0x000000E0);  // Interrupt 124 Priority Mask
pub const NVIC_PRI31_INTD_S = usize(29);
pub const NVIC_PRI31_INTC_S = usize(21);
pub const NVIC_PRI31_INTB_S = usize(13);
pub const NVIC_PRI31_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI32 register.
//
//*****************************************************************************
pub const NVIC_PRI32_INTD_M = usize(0xE0000000);  // Interrupt 131 Priority Mask
pub const NVIC_PRI32_INTC_M = usize(0x00E00000);  // Interrupt 130 Priority Mask
pub const NVIC_PRI32_INTB_M = usize(0x0000E000);  // Interrupt 129 Priority Mask
pub const NVIC_PRI32_INTA_M = usize(0x000000E0);  // Interrupt 128 Priority Mask
pub const NVIC_PRI32_INTD_S = usize(29);
pub const NVIC_PRI32_INTC_S = usize(21);
pub const NVIC_PRI32_INTB_S = usize(13);
pub const NVIC_PRI32_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI33 register.
//
//*****************************************************************************
pub const NVIC_PRI33_INTD_M = usize(0xE0000000);  // Interrupt Priority for Interrupt
                                            // [4n+3]
pub const NVIC_PRI33_INTC_M = usize(0x00E00000);  // Interrupt Priority for Interrupt
                                            // [4n+2]
pub const NVIC_PRI33_INTB_M = usize(0x0000E000);  // Interrupt Priority for Interrupt
                                            // [4n+1]
pub const NVIC_PRI33_INTA_M = usize(0x000000E0);  // Interrupt Priority for Interrupt
                                            // [4n]
pub const NVIC_PRI33_INTD_S = usize(29);
pub const NVIC_PRI33_INTC_S = usize(21);
pub const NVIC_PRI33_INTB_S = usize(13);
pub const NVIC_PRI33_INTA_S = usize(5);

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI34 register.
//
//*****************************************************************************
pub const NVIC_PRI34_INTD_M = usize(0xE0000000);  // Interrupt Priority for Interrupt
                                            // [4n+3]
pub const NVIC_PRI34_INTC_M = usize(0x00E00000);  // Interrupt Priority for Interrupt
                                            // [4n+2]
pub const NVIC_PRI34_INTB_M = usize(0x0000E000);  // Interrupt Priority for Interrupt
                                            // [4n+1]
pub const NVIC_PRI34_INTA_M = usize(0x000000E0);  // Interrupt Priority for Interrupt
                                            // [4n]
pub const NVIC_PRI34_INTD_S = usize(29);
pub const NVIC_PRI34_INTC_S = usize(21);
pub const NVIC_PRI34_INTB_S = usize(13);
pub const NVIC_PRI34_INTA_S = usize(5);

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

