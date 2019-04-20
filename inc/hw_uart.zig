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
pub const offsetOf = struct {
    pub const DR = usize(0x00000000); // UART Data
    pub const RSR = usize(0x00000004); // UART Receive Status/Error Clear
    pub const ECR = usize(0x00000004); // UART Receive Status/Error Clear
    pub const FR = usize(0x00000018); // UART Flag
    pub const ILPR = usize(0x00000020); // UART IrDA Low-Power Register
    pub const IBRD = usize(0x00000024); // UART Integer Baud-Rate Divisor
    pub const FBRD = usize(0x00000028); // UART Fractional Baud-Rate
    // Divisor
    pub const LCRH = usize(0x0000002C); // UART Line Control
    pub const CTL = usize(0x00000030); // UART Control
    pub const IFLS = usize(0x00000034); // UART Interrupt FIFO Level Select
    pub const IM = usize(0x00000038); // UART Interrupt Mask
    pub const RIS = usize(0x0000003C); // UART Raw Interrupt Status
    pub const MIS = usize(0x00000040); // UART Masked Interrupt Status
    pub const ICR = usize(0x00000044); // UART Interrupt Clear
    pub const DMACTL = usize(0x00000048); // UART DMA Control
    pub const _9BITADDR = usize(0x000000A4); // UART 9-Bit Self Address
    pub const _9BITAMASK = usize(0x000000A8); // UART 9-Bit Self Address Mask
    pub const PP = usize(0x00000FC0); // UART Peripheral Properties
    pub const CC = usize(0x00000FC8); // UART Clock Configuration
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DR register.
//
//*****************************************************************************
pub const DR = struct {
    pub const OE = usize(0x00000800); // UART Overrun Error
    pub const BE = usize(0x00000400); // UART Break Error
    pub const PE = usize(0x00000200); // UART Parity Error
    pub const FE = usize(0x00000100); // UART Framing Error
    pub const DATA_M = usize(0x000000FF); // Data Transmitted or Received
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RSR register.
//
//*****************************************************************************
pub const RSR = struct {
    pub const OE = usize(0x00000008); // UART Overrun Error
    pub const BE = usize(0x00000004); // UART Break Error
    pub const PE = usize(0x00000002); // UART Parity Error
    pub const FE = usize(0x00000001); // UART Framing Error
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ECR register.
//
//*****************************************************************************
pub const ECR = struct {
    pub const DATA_M = usize(0x000000FF); // Error Clear
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FR register.
//
//*****************************************************************************
pub const FR = struct {
    pub const RI = usize(0x00000100); // Ring Indicator
    pub const TXFE = usize(0x00000080); // UART Transmit FIFO Empty
    pub const RXFF = usize(0x00000040); // UART Receive FIFO Full
    pub const TXFF = usize(0x00000020); // UART Transmit FIFO Full
    pub const RXFE = usize(0x00000010); // UART Receive FIFO Empty
    pub const BUSY = usize(0x00000008); // UART Busy
    pub const DCD = usize(0x00000004); // Data Carrier Detect
    pub const DSR = usize(0x00000002); // Data Set Ready
    pub const CTS = usize(0x00000001); // Clear To Send
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ILPR register.
//
//*****************************************************************************
pub const ILPR = struct {
    pub const ILPDVSR_M = usize(0x000000FF); // IrDA Low-Power Divisor
    pub const ILPDVSR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IBRD register.
//
//*****************************************************************************
pub const IBRD = struct {
    pub const DIVINT_M = usize(0x0000FFFF); // Integer Baud-Rate Divisor
    pub const DIVINT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FBRD register.
//
//*****************************************************************************
pub const FBRD = struct {
    pub const DIVFRAC_M = usize(0x0000003F); // Fractional Baud-Rate Divisor
    pub const DIVFRAC_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_LCRH register.
//
//*****************************************************************************
pub const LCRH = struct {
    pub const SPS = usize(0x00000080); // UART Stick Parity Select
    pub const WLEN_M = usize(0x00000060); // UART Word Length
    pub const WLEN_5 = usize(0x00000000); // 5 bits (default)
    pub const WLEN_6 = usize(0x00000020); // 6 bits
    pub const WLEN_7 = usize(0x00000040); // 7 bits
    pub const WLEN_8 = usize(0x00000060); // 8 bits
    pub const FEN = usize(0x00000010); // UART Enable FIFOs
    pub const STP2 = usize(0x00000008); // UART Two Stop Bits Select
    pub const EPS = usize(0x00000004); // UART Even Parity Select
    pub const PEN = usize(0x00000002); // UART Parity Enable
    pub const BRK = usize(0x00000001); // UART Send Break
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CTL register.
//
//*****************************************************************************
pub const CTL = struct {
    pub const CTSEN = usize(0x00008000); // Enable Clear To Send
    pub const RTSEN = usize(0x00004000); // Enable Request to Send
    pub const RTS = usize(0x00000800); // Request to Send
    pub const DTR = usize(0x00000400); // Data Terminal Ready
    pub const RXE = usize(0x00000200); // UART Receive Enable
    pub const TXE = usize(0x00000100); // UART Transmit Enable
    pub const LBE = usize(0x00000080); // UART Loop Back Enable
    pub const HSE = usize(0x00000020); // High-Speed Enable
    pub const EOT = usize(0x00000010); // End of Transmission
    pub const SMART = usize(0x00000008); // ISO 7816 Smart Card Support
    pub const SIRLP = usize(0x00000004); // UART SIR Low-Power Mode
    pub const SIREN = usize(0x00000002); // UART SIR Enable
    pub const UARTEN = usize(0x00000001); // UART Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IFLS register.
//
//*****************************************************************************
pub const IFLS = struct {
    pub const RX_M = usize(0x00000038); // UART Receive Interrupt FIFO
    // Level Select
    pub const RX1_8 = usize(0x00000000); // RX FIFO >= 1/8 full
    pub const RX2_8 = usize(0x00000008); // RX FIFO >= 1/4 full
    pub const RX4_8 = usize(0x00000010); // RX FIFO >= 1/2 full (default)
    pub const RX6_8 = usize(0x00000018); // RX FIFO >= 3/4 full
    pub const RX7_8 = usize(0x00000020); // RX FIFO >= 7/8 full
    pub const TX_M = usize(0x00000007); // UART Transmit Interrupt FIFO
    // Level Select
    pub const TX1_8 = usize(0x00000000); // TX FIFO <= 1/8 full
    pub const TX2_8 = usize(0x00000001); // TX FIFO <= 1/4 full
    pub const TX4_8 = usize(0x00000002); // TX FIFO <= 1/2 full (default)
    pub const TX6_8 = usize(0x00000003); // TX FIFO <= 3/4 full
    pub const TX7_8 = usize(0x00000004); // TX FIFO <= 7/8 full
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IM register.
//
//*****************************************************************************
pub const IM = struct {
    pub const DMATXIM = usize(0x00020000); // Transmit DMA Interrupt Mask
    pub const DMARXIM = usize(0x00010000); // Receive DMA Interrupt Mask
    pub const _9BITIM = usize(0x00001000); // 9-Bit Mode Interrupt Mask
    pub const EOTIM = usize(0x00000800); // End of Transmission Interrupt
    // Mask
    pub const OEIM = usize(0x00000400); // UART Overrun Error Interrupt
    // Mask
    pub const BEIM = usize(0x00000200); // UART Break Error Interrupt Mask
    pub const PEIM = usize(0x00000100); // UART Parity Error Interrupt Mask
    pub const FEIM = usize(0x00000080); // UART Framing Error Interrupt
    // Mask
    pub const RTIM = usize(0x00000040); // UART Receive Time-Out Interrupt
    // Mask
    pub const TXIM = usize(0x00000020); // UART Transmit Interrupt Mask
    pub const RXIM = usize(0x00000010); // UART Receive Interrupt Mask
    pub const DSRMIM = usize(0x00000008); // UART Data Set Ready Modem
    // Interrupt Mask
    pub const DCDMIM = usize(0x00000004); // UART Data Carrier Detect Modem
    // Interrupt Mask
    pub const CTSMIM = usize(0x00000002); // UART Clear to Send Modem
    // Interrupt Mask
    pub const RIMIM = usize(0x00000001); // UART Ring Indicator Modem
    // Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RIS register.
//
//*****************************************************************************
pub const RIS = struct {
    pub const DMATXRIS = usize(0x00020000); // Transmit DMA Raw Interrupt
    // Status
    pub const DMARXRIS = usize(0x00010000); // Receive DMA Raw Interrupt Status
    pub const _9BITRIS = usize(0x00001000); // 9-Bit Mode Raw Interrupt Status
    pub const EOTRIS = usize(0x00000800); // End of Transmission Raw
    // Interrupt Status
    pub const OERIS = usize(0x00000400); // UART Overrun Error Raw Interrupt
    // Status
    pub const BERIS = usize(0x00000200); // UART Break Error Raw Interrupt
    // Status
    pub const PERIS = usize(0x00000100); // UART Parity Error Raw Interrupt
    // Status
    pub const FERIS = usize(0x00000080); // UART Framing Error Raw Interrupt
    // Status
    pub const RTRIS = usize(0x00000040); // UART Receive Time-Out Raw
    // Interrupt Status
    pub const TXRIS = usize(0x00000020); // UART Transmit Raw Interrupt
    // Status
    pub const RXRIS = usize(0x00000010); // UART Receive Raw Interrupt
    // Status
    pub const DSRRIS = usize(0x00000008); // UART Data Set Ready Modem Raw
    // Interrupt Status
    pub const DCDRIS = usize(0x00000004); // UART Data Carrier Detect Modem
    // Raw Interrupt Status
    pub const CTSRIS = usize(0x00000002); // UART Clear to Send Modem Raw
    // Interrupt Status
    pub const RIRIS = usize(0x00000001); // UART Ring Indicator Modem Raw
    // Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_MIS register.
//
//*****************************************************************************
pub const MIS = struct {
    pub const DMATXMIS = usize(0x00020000); // Transmit DMA Masked Interrupt
    // Status
    pub const DMARXMIS = usize(0x00010000); // Receive DMA Masked Interrupt
    // Status
    pub const _9BITMIS = usize(0x00001000); // 9-Bit Mode Masked Interrupt
    // Status
    pub const EOTMIS = usize(0x00000800); // End of Transmission Masked
    // Interrupt Status
    pub const OEMIS = usize(0x00000400); // UART Overrun Error Masked
    // Interrupt Status
    pub const BEMIS = usize(0x00000200); // UART Break Error Masked
    // Interrupt Status
    pub const PEMIS = usize(0x00000100); // UART Parity Error Masked
    // Interrupt Status
    pub const FEMIS = usize(0x00000080); // UART Framing Error Masked
    // Interrupt Status
    pub const RTMIS = usize(0x00000040); // UART Receive Time-Out Masked
    // Interrupt Status
    pub const TXMIS = usize(0x00000020); // UART Transmit Masked Interrupt
    // Status
    pub const RXMIS = usize(0x00000010); // UART Receive Masked Interrupt
    // Status
    pub const DSRMIS = usize(0x00000008); // UART Data Set Ready Modem Masked
    // Interrupt Status
    pub const DCDMIS = usize(0x00000004); // UART Data Carrier Detect Modem
    // Masked Interrupt Status
    pub const CTSMIS = usize(0x00000002); // UART Clear to Send Modem Masked
    // Interrupt Status
    pub const RIMIS = usize(0x00000001); // UART Ring Indicator Modem Masked
    // Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ICR register.
//
//*****************************************************************************
pub const ICR = struct {
    pub const DMATXIC = usize(0x00020000); // Transmit DMA Interrupt Clear
    pub const DMARXIC = usize(0x00010000); // Receive DMA Interrupt Clear
    pub const _9BITIC = usize(0x00001000); // 9-Bit Mode Interrupt Clear
    pub const EOTIC = usize(0x00000800); // End of Transmission Interrupt
    // Clear
    pub const OEIC = usize(0x00000400); // Overrun Error Interrupt Clear
    pub const BEIC = usize(0x00000200); // Break Error Interrupt Clear
    pub const PEIC = usize(0x00000100); // Parity Error Interrupt Clear
    pub const FEIC = usize(0x00000080); // Framing Error Interrupt Clear
    pub const RTIC = usize(0x00000040); // Receive Time-Out Interrupt Clear
    pub const TXIC = usize(0x00000020); // Transmit Interrupt Clear
    pub const RXIC = usize(0x00000010); // Receive Interrupt Clear
    pub const DSRMIC = usize(0x00000008); // UART Data Set Ready Modem
    // Interrupt Clear
    pub const DCDMIC = usize(0x00000004); // UART Data Carrier Detect Modem
    // Interrupt Clear
    pub const CTSMIC = usize(0x00000002); // UART Clear to Send Modem
    // Interrupt Clear
    pub const RIMIC = usize(0x00000001); // UART Ring Indicator Modem
    // Interrupt Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DMACTL register.
//
//*****************************************************************************
pub const DMACTL = struct {
    pub const DMAERR = usize(0x00000004); // DMA on Error
    pub const TXDMAE = usize(0x00000002); // Transmit DMA Enable
    pub const RXDMAE = usize(0x00000001); // Receive DMA Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITADDR
// register.
//
//*****************************************************************************
pub const _9BITADDR = struct {
    pub const _9BITEN = usize(0x00008000); // Enable 9-Bit Mode
    pub const ADDR_M = usize(0x000000FF); // Self Address for 9-Bit Mode
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITAMASK
// register.
//
//*****************************************************************************
pub const _9BITAMASK = struct {
    pub const MASK_M = usize(0x000000FF); // Self Address Mask for 9-Bit Mode
    pub const MASK_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_PP register.
//
//*****************************************************************************
pub const PP = struct {
    pub const MSE = usize(0x00000008); // Modem Support Extended
    pub const MS = usize(0x00000004); // Modem Support
    pub const NB = usize(0x00000002); // 9-Bit Support
    pub const SC = usize(0x00000001); // Smart Card Support
};

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CC register.
//
//*****************************************************************************
pub const CC = struct {
    pub const CS_M = usize(0x0000000F); // UART Baud Clock Source
    pub const CS_SYSCLK = usize(0x00000000); // System clock (based on clock
    // source and divisor factor)
    pub const CS_PIOSC = usize(0x00000005); // PIOSC
};
