//*****************************************************************************
//
// hw_i2c.h - Macros used when accessing the I2C master and slave hardware.
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
// The following are defines for the I2C register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const MSA = usize(0x00000000); // I2C Master Slave Address
    pub const MCS = usize(0x00000004); // I2C Master Control/Status
    pub const MDR = usize(0x00000008); // I2C Master Data
    pub const MTPR = usize(0x0000000C); // I2C Master Timer Period
    pub const MIMR = usize(0x00000010); // I2C Master Interrupt Mask
    pub const MRIS = usize(0x00000014); // I2C Master Raw Interrupt Status
    pub const MMIS = usize(0x00000018); // I2C Master Masked Interrupt
    // Status
    pub const MICR = usize(0x0000001C); // I2C Master Interrupt Clear
    pub const MCR = usize(0x00000020); // I2C Master Configuration
    pub const MCLKOCNT = usize(0x00000024); // I2C Master Clock Low Timeout
    // Count
    pub const MBMON = usize(0x0000002C); // I2C Master Bus Monitor
    pub const MBLEN = usize(0x00000030); // I2C Master Burst Length
    pub const MBCNT = usize(0x00000034); // I2C Master Burst Count
    pub const MCR2 = usize(0x00000038); // I2C Master Configuration 2
    pub const SOAR = usize(0x00000800); // I2C Slave Own Address
    pub const SCSR = usize(0x00000804); // I2C Slave Control/Status
    pub const SDR = usize(0x00000808); // I2C Slave Data
    pub const SIMR = usize(0x0000080C); // I2C Slave Interrupt Mask
    pub const SRIS = usize(0x00000810); // I2C Slave Raw Interrupt Status
    pub const SMIS = usize(0x00000814); // I2C Slave Masked Interrupt
    // Status
    pub const SICR = usize(0x00000818); // I2C Slave Interrupt Clear
    pub const SOAR2 = usize(0x0000081C); // I2C Slave Own Address 2
    pub const SACKCTL = usize(0x00000820); // I2C Slave ACK Control
    pub const FIFODATA = usize(0x00000F00); // I2C FIFO Data
    pub const FIFOCTL = usize(0x00000F04); // I2C FIFO Control
    pub const FIFOSTATUS = usize(0x00000F08); // I2C FIFO Status
    pub const PP = usize(0x00000FC0); // I2C Peripheral Properties
    pub const PC = usize(0x00000FC4); // I2C Peripheral Configuration
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MSA register.
//
//*****************************************************************************
pub const MSA = struct {
    pub const SA_M = usize(0x000000FE); // I2C Slave Address
    pub const RS = usize(0x00000001); // Receive not send
    pub const SA_S = usize(1);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCS register.
//
//*****************************************************************************
pub const MCS = struct {
    pub const ACTDMARX = usize(0x80000000); // DMA RX Active Status
    pub const ACTDMATX = usize(0x40000000); // DMA TX Active Status
    pub const CLKTO = usize(0x00000080); // Clock Timeout Error
    pub const BURST = usize(0x00000040); // Burst Enable
    pub const BUSBSY = usize(0x00000040); // Bus Busy
    pub const IDLE = usize(0x00000020); // I2C Idle
    pub const QCMD = usize(0x00000020); // Quick Command
    pub const ARBLST = usize(0x00000010); // Arbitration Lost
    pub const HS = usize(0x00000010); // High-Speed Enable
    pub const ACK = usize(0x00000008); // Data Acknowledge Enable
    pub const DATACK = usize(0x00000008); // Acknowledge Data
    pub const ADRACK = usize(0x00000004); // Acknowledge Address
    pub const STOP = usize(0x00000004); // Generate STOP
    pub const ERROR = usize(0x00000002); // Error
    pub const START = usize(0x00000002); // Generate START
    pub const RUN = usize(0x00000001); // I2C Master Enable
    pub const BUSY = usize(0x00000001); // I2C Busy
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MDR register.
//
//*****************************************************************************
pub const MDR = struct {
    pub const DATA_M = usize(0x000000FF); // This byte contains the data
    // transferred during a transaction
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MTPR register.
//
//*****************************************************************************
pub const MTPR = struct {
    pub const PULSEL_M = usize(0x00070000); // Glitch Suppression Pulse Width
    pub const PULSEL_BYPASS = usize(0x00000000); // Bypass
    pub const PULSEL_1 = usize(0x00010000); // 1 clock
    pub const PULSEL_2 = usize(0x00020000); // 2 clocks
    pub const PULSEL_3 = usize(0x00030000); // 3 clocks
    pub const PULSEL_4 = usize(0x00040000); // 4 clocks
    pub const PULSEL_8 = usize(0x00050000); // 8 clocks
    pub const PULSEL_16 = usize(0x00060000); // 16 clocks
    pub const PULSEL_31 = usize(0x00070000); // 31 clocks
    pub const HS = usize(0x00000080); // High-Speed Enable
    pub const TPR_M = usize(0x0000007F); // Timer Period
    pub const TPR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MIMR register.
//
//*****************************************************************************
pub const MIMR = struct {
    pub const RXFFIM = usize(0x00000800); // Receive FIFO Full Interrupt Mask
    pub const TXFEIM = usize(0x00000400); // Transmit FIFO Empty Interrupt
    // Mask
    pub const RXIM = usize(0x00000200); // Receive FIFO Request Interrupt
    // Mask
    pub const TXIM = usize(0x00000100); // Transmit FIFO Request Interrupt
    // Mask
    pub const ARBLOSTIM = usize(0x00000080); // Arbitration Lost Interrupt Mask
    pub const STOPIM = usize(0x00000040); // STOP Detection Interrupt Mask
    pub const STARTIM = usize(0x00000020); // START Detection Interrupt Mask
    pub const NACKIM = usize(0x00000010); // Address/Data NACK Interrupt Mask
    pub const DMATXIM = usize(0x00000008); // Transmit DMA Interrupt Mask
    pub const DMARXIM = usize(0x00000004); // Receive DMA Interrupt Mask
    pub const CLKIM = usize(0x00000002); // Clock Timeout Interrupt Mask
    pub const IM = usize(0x00000001); // Master Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MRIS register.
//
//*****************************************************************************
pub const MRIS = struct {
    pub const RXFFRIS = usize(0x00000800); // Receive FIFO Full Raw Interrupt
    // Status
    pub const TXFERIS = usize(0x00000400); // Transmit FIFO Empty Raw
    // Interrupt Status
    pub const RXRIS = usize(0x00000200); // Receive FIFO Request Raw
    // Interrupt Status
    pub const TXRIS = usize(0x00000100); // Transmit Request Raw Interrupt
    // Status
    pub const ARBLOSTRIS = usize(0x00000080); // Arbitration Lost Raw Interrupt
    // Status
    pub const STOPRIS = usize(0x00000040); // STOP Detection Raw Interrupt
    // Status
    pub const STARTRIS = usize(0x00000020); // START Detection Raw Interrupt
    // Status
    pub const NACKRIS = usize(0x00000010); // Address/Data NACK Raw Interrupt
    // Status
    pub const DMATXRIS = usize(0x00000008); // Transmit DMA Raw Interrupt
    // Status
    pub const DMARXRIS = usize(0x00000004); // Receive DMA Raw Interrupt Status
    pub const CLKRIS = usize(0x00000002); // Clock Timeout Raw Interrupt
    // Status
    pub const RIS = usize(0x00000001); // Master Raw Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MMIS register.
//
//*****************************************************************************
pub const MMIS = struct {
    pub const RXFFMIS = usize(0x00000800); // Receive FIFO Full Interrupt Mask
    pub const TXFEMIS = usize(0x00000400); // Transmit FIFO Empty Interrupt
    // Mask
    pub const RXMIS = usize(0x00000200); // Receive FIFO Request Interrupt
    // Mask
    pub const TXMIS = usize(0x00000100); // Transmit Request Interrupt Mask
    pub const ARBLOSTMIS = usize(0x00000080); // Arbitration Lost Interrupt Mask
    pub const STOPMIS = usize(0x00000040); // STOP Detection Interrupt Mask
    pub const STARTMIS = usize(0x00000020); // START Detection Interrupt Mask
    pub const NACKMIS = usize(0x00000010); // Address/Data NACK Interrupt Mask
    pub const DMATXMIS = usize(0x00000008); // Transmit DMA Interrupt Status
    pub const DMARXMIS = usize(0x00000004); // Receive DMA Interrupt Status
    pub const CLKMIS = usize(0x00000002); // Clock Timeout Masked Interrupt
    // Status
    pub const MIS = usize(0x00000001); // Masked Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MICR register.
//
//*****************************************************************************
pub const MICR = struct {
    pub const RXFFIC = usize(0x00000800); // Receive FIFO Full Interrupt
    // Clear
    pub const TXFEIC = usize(0x00000400); // Transmit FIFO Empty Interrupt
    // Clear
    pub const RXIC = usize(0x00000200); // Receive FIFO Request Interrupt
    // Clear
    pub const TXIC = usize(0x00000100); // Transmit FIFO Request Interrupt
    // Clear
    pub const ARBLOSTIC = usize(0x00000080); // Arbitration Lost Interrupt Clear
    pub const STOPIC = usize(0x00000040); // STOP Detection Interrupt Clear
    pub const STARTIC = usize(0x00000020); // START Detection Interrupt Clear
    pub const NACKIC = usize(0x00000010); // Address/Data NACK Interrupt
    // Clear
    pub const DMATXIC = usize(0x00000008); // Transmit DMA Interrupt Clear
    pub const DMARXIC = usize(0x00000004); // Receive DMA Interrupt Clear
    pub const CLKIC = usize(0x00000002); // Clock Timeout Interrupt Clear
    pub const IC = usize(0x00000001); // Master Interrupt Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCR register.
//
//*****************************************************************************
pub const MCR = struct {
    pub const GFE = usize(0x00000040); // I2C Glitch Filter Enable
    pub const SFE = usize(0x00000020); // I2C Slave Function Enable
    pub const MFE = usize(0x00000010); // I2C Master Function Enable
    pub const LPBK = usize(0x00000001); // I2C Loopback
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCLKOCNT register.
//
//*****************************************************************************
pub const MCLKOCNT = struct {
    pub const CNTL_M = usize(0x000000FF); // I2C Master Count
    pub const CNTL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBMON register.
//
//*****************************************************************************
pub const MBMON = struct {
    pub const SDA = usize(0x00000002); // I2C SDA Status
    pub const SCL = usize(0x00000001); // I2C SCL Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBLEN register.
//
//*****************************************************************************
pub const MBLEN = struct {
    pub const CNTL_M = usize(0x000000FF); // I2C Burst Length
    pub const CNTL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBCNT register.
//
//*****************************************************************************
pub const MBCNT = struct {
    pub const CNTL_M = usize(0x000000FF); // I2C Master Burst Count
    pub const CNTL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCR2 register.
//
//*****************************************************************************
pub const MCR2 = struct {
    pub const GFPW_M = usize(0x00000070); // I2C Glitch Filter Pulse Width
    pub const GFPW_BYPASS = usize(0x00000000); // Bypass
    pub const GFPW_1 = usize(0x00000010); // 1 clock
    pub const GFPW_2 = usize(0x00000020); // 2 clocks
    pub const GFPW_3 = usize(0x00000030); // 3 clocks
    pub const GFPW_4 = usize(0x00000040); // 4 clocks
    pub const GFPW_8 = usize(0x00000050); // 8 clocks
    pub const GFPW_16 = usize(0x00000060); // 16 clocks
    pub const GFPW_31 = usize(0x00000070); // 31 clocks
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR register.
//
//*****************************************************************************
pub const SOAR = struct {
    pub const OAR_M = usize(0x0000007F); // I2C Slave Own Address
    pub const OAR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SCSR register.
//
//*****************************************************************************
pub const SCSR = struct {
    pub const ACTDMARX = usize(0x80000000); // DMA RX Active Status
    pub const ACTDMATX = usize(0x40000000); // DMA TX Active Status
    pub const QCMDRW = usize(0x00000020); // Quick Command Read / Write
    pub const QCMDST = usize(0x00000010); // Quick Command Status
    pub const OAR2SEL = usize(0x00000008); // OAR2 Address Matched
    pub const FBR = usize(0x00000004); // First Byte Received
    pub const RXFIFO = usize(0x00000004); // RX FIFO Enable
    pub const TXFIFO = usize(0x00000002); // TX FIFO Enable
    pub const TREQ = usize(0x00000002); // Transmit Request
    pub const DA = usize(0x00000001); // Device Active
    pub const RREQ = usize(0x00000001); // Receive Request
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SDR register.
//
//*****************************************************************************
pub const SDR = struct {
    pub const DATA_M = usize(0x000000FF); // Data for Transfer
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SIMR register.
//
//*****************************************************************************
pub const SIMR = struct {
    pub const RXFFIM = usize(0x00000100); // Receive FIFO Full Interrupt Mask
    pub const TXFEIM = usize(0x00000080); // Transmit FIFO Empty Interrupt
    // Mask
    pub const RXIM = usize(0x00000040); // Receive FIFO Request Interrupt
    // Mask
    pub const TXIM = usize(0x00000020); // Transmit FIFO Request Interrupt
    // Mask
    pub const DMATXIM = usize(0x00000010); // Transmit DMA Interrupt Mask
    pub const DMARXIM = usize(0x00000008); // Receive DMA Interrupt Mask
    pub const STOPIM = usize(0x00000004); // Stop Condition Interrupt Mask
    pub const STARTIM = usize(0x00000002); // Start Condition Interrupt Mask
    pub const DATAIM = usize(0x00000001); // Data Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SRIS register.
//
//*****************************************************************************
pub const SRIS = struct {
    pub const RXFFRIS = usize(0x00000100); // Receive FIFO Full Raw Interrupt
    // Status
    pub const TXFERIS = usize(0x00000080); // Transmit FIFO Empty Raw
    // Interrupt Status
    pub const RXRIS = usize(0x00000040); // Receive FIFO Request Raw
    // Interrupt Status
    pub const TXRIS = usize(0x00000020); // Transmit Request Raw Interrupt
    // Status
    pub const DMATXRIS = usize(0x00000010); // Transmit DMA Raw Interrupt
    // Status
    pub const DMARXRIS = usize(0x00000008); // Receive DMA Raw Interrupt Status
    pub const STOPRIS = usize(0x00000004); // Stop Condition Raw Interrupt
    // Status
    pub const STARTRIS = usize(0x00000002); // Start Condition Raw Interrupt
    // Status
    pub const DATARIS = usize(0x00000001); // Data Raw Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SMIS register.
//
//*****************************************************************************
pub const SMIS = struct {
    pub const RXFFMIS = usize(0x00000100); // Receive FIFO Full Interrupt Mask
    pub const TXFEMIS = usize(0x00000080); // Transmit FIFO Empty Interrupt
    // Mask
    pub const RXMIS = usize(0x00000040); // Receive FIFO Request Interrupt
    // Mask
    pub const TXMIS = usize(0x00000020); // Transmit FIFO Request Interrupt
    // Mask
    pub const DMATXMIS = usize(0x00000010); // Transmit DMA Masked Interrupt
    // Status
    pub const DMARXMIS = usize(0x00000008); // Receive DMA Masked Interrupt
    // Status
    pub const STOPMIS = usize(0x00000004); // Stop Condition Masked Interrupt
    // Status
    pub const STARTMIS = usize(0x00000002); // Start Condition Masked Interrupt
    // Status
    pub const DATAMIS = usize(0x00000001); // Data Masked Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SICR register.
//
//*****************************************************************************
pub const SICR = struct {
    pub const RXFFIC = usize(0x00000100); // Receive FIFO Full Interrupt Mask
    pub const TXFEIC = usize(0x00000080); // Transmit FIFO Empty Interrupt
    // Mask
    pub const RXIC = usize(0x00000040); // Receive Request Interrupt Mask
    pub const TXIC = usize(0x00000020); // Transmit Request Interrupt Mask
    pub const DMATXIC = usize(0x00000010); // Transmit DMA Interrupt Clear
    pub const DMARXIC = usize(0x00000008); // Receive DMA Interrupt Clear
    pub const STOPIC = usize(0x00000004); // Stop Condition Interrupt Clear
    pub const STARTIC = usize(0x00000002); // Start Condition Interrupt Clear
    pub const DATAIC = usize(0x00000001); // Data Interrupt Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR2 register.
//
//*****************************************************************************
pub const SOAR2 = struct {
    pub const OAR2EN = usize(0x00000080); // I2C Slave Own Address 2 Enable
    pub const OAR2_M = usize(0x0000007F); // I2C Slave Own Address 2
    pub const OAR2_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SACKCTL register.
//
//*****************************************************************************
pub const SACKCTL = struct {
    pub const ACKOVAL = usize(0x00000002); // I2C Slave ACK Override Value
    pub const ACKOEN = usize(0x00000001); // I2C Slave ACK Override Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_FIFODATA register.
//
//*****************************************************************************
pub const FIFODATA = struct {
    pub const DATA_M = usize(0x000000FF); // I2C TX FIFO Write Data Byte
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_FIFOCTL register.
//
//*****************************************************************************
pub const FIFOCTL = struct {
    pub const RXASGNMT = usize(0x80000000); // RX Control Assignment
    pub const RXFLUSH = usize(0x40000000); // RX FIFO Flush
    pub const DMARXENA = usize(0x20000000); // DMA RX Channel Enable
    pub const RXTRIG_M = usize(0x00070000); // RX FIFO Trigger
    pub const TXASGNMT = usize(0x00008000); // TX Control Assignment
    pub const TXFLUSH = usize(0x00004000); // TX FIFO Flush
    pub const DMATXENA = usize(0x00002000); // DMA TX Channel Enable
    pub const TXTRIG_M = usize(0x00000007); // TX FIFO Trigger
    pub const RXTRIG_S = usize(16);
    pub const TXTRIG_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_FIFOSTATUS
// register.
//
//*****************************************************************************
pub const FIFOSTATUS = struct {
    pub const RXABVTRIG = usize(0x00040000); // RX FIFO Above Trigger Level
    pub const RXFF = usize(0x00020000); // RX FIFO Full
    pub const RXFE = usize(0x00010000); // RX FIFO Empty
    pub const TXBLWTRIG = usize(0x00000004); // TX FIFO Below Trigger Level
    pub const TXFF = usize(0x00000002); // TX FIFO Full
    pub const TXFE = usize(0x00000001); // TX FIFO Empty
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PP register.
//
//*****************************************************************************
pub const PP = struct {
    pub const HS = usize(0x00000001); // High-Speed Capable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PC register.
//
//*****************************************************************************
pub const PC = struct {
    pub const HS = usize(0x00000001); // High-Speed Capable
};
