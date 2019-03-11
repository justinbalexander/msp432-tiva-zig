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
pub const I2C_O_MSA = usize(0x00000000);  // I2C Master Slave Address
pub const I2C_O_MCS = usize(0x00000004);  // I2C Master Control/Status
pub const I2C_O_MDR = usize(0x00000008);  // I2C Master Data
pub const I2C_O_MTPR = usize(0x0000000C);  // I2C Master Timer Period
pub const I2C_O_MIMR = usize(0x00000010);  // I2C Master Interrupt Mask
pub const I2C_O_MRIS = usize(0x00000014);  // I2C Master Raw Interrupt Status
pub const I2C_O_MMIS = usize(0x00000018);  // I2C Master Masked Interrupt
                                            // Status
pub const I2C_O_MICR = usize(0x0000001C);  // I2C Master Interrupt Clear
pub const I2C_O_MCR = usize(0x00000020);  // I2C Master Configuration
pub const I2C_O_MCLKOCNT = usize(0x00000024);  // I2C Master Clock Low Timeout
                                            // Count
pub const I2C_O_MBMON = usize(0x0000002C);  // I2C Master Bus Monitor
pub const I2C_O_MBLEN = usize(0x00000030);  // I2C Master Burst Length
pub const I2C_O_MBCNT = usize(0x00000034);  // I2C Master Burst Count
pub const I2C_O_MCR2 = usize(0x00000038);  // I2C Master Configuration 2
pub const I2C_O_SOAR = usize(0x00000800);  // I2C Slave Own Address
pub const I2C_O_SCSR = usize(0x00000804);  // I2C Slave Control/Status
pub const I2C_O_SDR = usize(0x00000808);  // I2C Slave Data
pub const I2C_O_SIMR = usize(0x0000080C);  // I2C Slave Interrupt Mask
pub const I2C_O_SRIS = usize(0x00000810);  // I2C Slave Raw Interrupt Status
pub const I2C_O_SMIS = usize(0x00000814);  // I2C Slave Masked Interrupt
                                            // Status
pub const I2C_O_SICR = usize(0x00000818);  // I2C Slave Interrupt Clear
pub const I2C_O_SOAR2 = usize(0x0000081C);  // I2C Slave Own Address 2
pub const I2C_O_SACKCTL = usize(0x00000820);  // I2C Slave ACK Control
pub const I2C_O_FIFODATA = usize(0x00000F00);  // I2C FIFO Data
pub const I2C_O_FIFOCTL = usize(0x00000F04);  // I2C FIFO Control
pub const I2C_O_FIFOSTATUS = usize(0x00000F08);  // I2C FIFO Status
pub const I2C_O_PP = usize(0x00000FC0);  // I2C Peripheral Properties
pub const I2C_O_PC = usize(0x00000FC4);  // I2C Peripheral Configuration

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

