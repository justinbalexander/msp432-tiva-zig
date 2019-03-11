//*****************************************************************************
//
// hw_uart.h - Macros and defines used when accessing the UART hardware.
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
// The following are defines for the UART register offsets.
//
//*****************************************************************************
pub const UART_O_DR = usize(0x00000000);  // UART Data
pub const UART_O_RSR = usize(0x00000004);  // UART Receive Status/Error Clear
pub const UART_O_ECR = usize(0x00000004);  // UART Receive Status/Error Clear
pub const UART_O_FR = usize(0x00000018);  // UART Flag
pub const UART_O_ILPR = usize(0x00000020);  // UART IrDA Low-Power Register
pub const UART_O_IBRD = usize(0x00000024);  // UART Integer Baud-Rate Divisor
pub const UART_O_FBRD = usize(0x00000028);  // UART Fractional Baud-Rate
                                            // Divisor
pub const UART_O_LCRH = usize(0x0000002C);  // UART Line Control
pub const UART_O_CTL = usize(0x00000030);  // UART Control
pub const UART_O_IFLS = usize(0x00000034);  // UART Interrupt FIFO Level Select
pub const UART_O_IM = usize(0x00000038);  // UART Interrupt Mask
pub const UART_O_RIS = usize(0x0000003C);  // UART Raw Interrupt Status
pub const UART_O_MIS = usize(0x00000040);  // UART Masked Interrupt Status
pub const UART_O_ICR = usize(0x00000044);  // UART Interrupt Clear
pub const UART_O_DMACTL = usize(0x00000048);  // UART DMA Control
pub const UART_O_9BITADDR = usize(0x000000A4);  // UART 9-Bit Self Address
pub const UART_O_9BITAMASK = usize(0x000000A8);  // UART 9-Bit Self Address Mask
pub const UART_O_PP = usize(0x00000FC0);  // UART Peripheral Properties
pub const UART_O_CC = usize(0x00000FC8);  // UART Clock Configuration

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

