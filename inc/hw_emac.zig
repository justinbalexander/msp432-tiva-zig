//*****************************************************************************
//
// hw_emac.h - Macros used when accessing the EMAC hardware.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
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
// The following are defines for the EMAC register offsets.
//
//*****************************************************************************
pub const EMAC_O_CFG = usize(0x00000000);  // Ethernet MAC Configuration
pub const EMAC_O_FRAMEFLTR = usize(0x00000004);  // Ethernet MAC Frame Filter
pub const EMAC_O_HASHTBLH = usize(0x00000008);  // Ethernet MAC Hash Table High
pub const EMAC_O_HASHTBLL = usize(0x0000000C);  // Ethernet MAC Hash Table Low
pub const EMAC_O_MIIADDR = usize(0x00000010);  // Ethernet MAC MII Address
pub const EMAC_O_MIIDATA = usize(0x00000014);  // Ethernet MAC MII Data Register
pub const EMAC_O_FLOWCTL = usize(0x00000018);  // Ethernet MAC Flow Control
pub const EMAC_O_VLANTG = usize(0x0000001C);  // Ethernet MAC VLAN Tag
pub const EMAC_O_STATUS = usize(0x00000024);  // Ethernet MAC Status
pub const EMAC_O_RWUFF = usize(0x00000028);  // Ethernet MAC Remote Wake-Up
                                            // Frame Filter
pub const EMAC_O_PMTCTLSTAT = usize(0x0000002C);  // Ethernet MAC PMT Control and
                                            // Status Register
pub const EMAC_O_LPICTLSTAT = usize(0x00000030);  // Ethernet MAC Low Power Idle
                                            // Control and Status Register
pub const EMAC_O_LPITIMERCTL = usize(0x00000034);  // Ethernet MAC Low Power Idle
                                            // Timer Control Register
pub const EMAC_O_RIS = usize(0x00000038);  // Ethernet MAC Raw Interrupt
                                            // Status
pub const EMAC_O_IM = usize(0x0000003C);  // Ethernet MAC Interrupt Mask
pub const EMAC_O_ADDR0H = usize(0x00000040);  // Ethernet MAC Address 0 High
pub const EMAC_O_ADDR0L = usize(0x00000044);  // Ethernet MAC Address 0 Low
                                            // Register
pub const EMAC_O_ADDR1H = usize(0x00000048);  // Ethernet MAC Address 1 High
pub const EMAC_O_ADDR1L = usize(0x0000004C);  // Ethernet MAC Address 1 Low
pub const EMAC_O_ADDR2H = usize(0x00000050);  // Ethernet MAC Address 2 High
pub const EMAC_O_ADDR2L = usize(0x00000054);  // Ethernet MAC Address 2 Low
pub const EMAC_O_ADDR3H = usize(0x00000058);  // Ethernet MAC Address 3 High
pub const EMAC_O_ADDR3L = usize(0x0000005C);  // Ethernet MAC Address 3 Low
pub const EMAC_O_WDOGTO = usize(0x000000DC);  // Ethernet MAC Watchdog Timeout
pub const EMAC_O_MMCCTRL = usize(0x00000100);  // Ethernet MAC MMC Control
pub const EMAC_O_MMCRXRIS = usize(0x00000104);  // Ethernet MAC MMC Receive Raw
                                            // Interrupt Status
pub const EMAC_O_MMCTXRIS = usize(0x00000108);  // Ethernet MAC MMC Transmit Raw
                                            // Interrupt Status
pub const EMAC_O_MMCRXIM = usize(0x0000010C);  // Ethernet MAC MMC Receive
                                            // Interrupt Mask
pub const EMAC_O_MMCTXIM = usize(0x00000110);  // Ethernet MAC MMC Transmit
                                            // Interrupt Mask
pub const EMAC_O_TXCNTGB = usize(0x00000118);  // Ethernet MAC Transmit Frame
                                            // Count for Good and Bad Frames
pub const EMAC_O_TXCNTSCOL = usize(0x0000014C);  // Ethernet MAC Transmit Frame
                                            // Count for Frames Transmitted
                                            // after Single Collision
pub const EMAC_O_TXCNTMCOL = usize(0x00000150);  // Ethernet MAC Transmit Frame
                                            // Count for Frames Transmitted
                                            // after Multiple Collisions
pub const EMAC_O_TXOCTCNTG = usize(0x00000164);  // Ethernet MAC Transmit Octet
                                            // Count Good
pub const EMAC_O_RXCNTGB = usize(0x00000180);  // Ethernet MAC Receive Frame Count
                                            // for Good and Bad Frames
pub const EMAC_O_RXCNTCRCERR = usize(0x00000194);  // Ethernet MAC Receive Frame Count
                                            // for CRC Error Frames
pub const EMAC_O_RXCNTALGNERR = usize(0x00000198);  // Ethernet MAC Receive Frame Count
                                            // for Alignment Error Frames
pub const EMAC_O_RXCNTGUNI = usize(0x000001C4);  // Ethernet MAC Receive Frame Count
                                            // for Good Unicast Frames
pub const EMAC_O_VLNINCREP = usize(0x00000584);  // Ethernet MAC VLAN Tag Inclusion
                                            // or Replacement
pub const EMAC_O_VLANHASH = usize(0x00000588);  // Ethernet MAC VLAN Hash Table
pub const EMAC_O_TIMSTCTRL = usize(0x00000700);  // Ethernet MAC Timestamp Control
pub const EMAC_O_SUBSECINC = usize(0x00000704);  // Ethernet MAC Sub-Second
                                            // Increment
pub const EMAC_O_TIMSEC = usize(0x00000708);  // Ethernet MAC System Time -
                                            // Seconds
pub const EMAC_O_TIMNANO = usize(0x0000070C);  // Ethernet MAC System Time -
                                            // Nanoseconds
pub const EMAC_O_TIMSECU = usize(0x00000710);  // Ethernet MAC System Time -
                                            // Seconds Update
pub const EMAC_O_TIMNANOU = usize(0x00000714);  // Ethernet MAC System Time -
                                            // Nanoseconds Update
pub const EMAC_O_TIMADD = usize(0x00000718);  // Ethernet MAC Timestamp Addend
pub const EMAC_O_TARGSEC = usize(0x0000071C);  // Ethernet MAC Target Time Seconds
pub const EMAC_O_TARGNANO = usize(0x00000720);  // Ethernet MAC Target Time
                                            // Nanoseconds
pub const EMAC_O_HWORDSEC = usize(0x00000724);  // Ethernet MAC System Time-Higher
                                            // Word Seconds
pub const EMAC_O_TIMSTAT = usize(0x00000728);  // Ethernet MAC Timestamp Status
pub const EMAC_O_PPSCTRL = usize(0x0000072C);  // Ethernet MAC PPS Control
pub const EMAC_O_PPS0INTVL = usize(0x00000760);  // Ethernet MAC PPS0 Interval
pub const EMAC_O_PPS0WIDTH = usize(0x00000764);  // Ethernet MAC PPS0 Width
pub const EMAC_O_DMABUSMOD = usize(0x00000C00);  // Ethernet MAC DMA Bus Mode
pub const EMAC_O_TXPOLLD = usize(0x00000C04);  // Ethernet MAC Transmit Poll
                                            // Demand
pub const EMAC_O_RXPOLLD = usize(0x00000C08);  // Ethernet MAC Receive Poll Demand
pub const EMAC_O_RXDLADDR = usize(0x00000C0C);  // Ethernet MAC Receive Descriptor
                                            // List Address
pub const EMAC_O_TXDLADDR = usize(0x00000C10);  // Ethernet MAC Transmit Descriptor
                                            // List Address
pub const EMAC_O_DMARIS = usize(0x00000C14);  // Ethernet MAC DMA Interrupt
                                            // Status
pub const EMAC_O_DMAOPMODE = usize(0x00000C18);  // Ethernet MAC DMA Operation Mode
pub const EMAC_O_DMAIM = usize(0x00000C1C);  // Ethernet MAC DMA Interrupt Mask
                                            // Register
pub const EMAC_O_MFBOC = usize(0x00000C20);  // Ethernet MAC Missed Frame and
                                            // Buffer Overflow Counter
pub const EMAC_O_RXINTWDT = usize(0x00000C24);  // Ethernet MAC Receive Interrupt
                                            // Watchdog Timer
pub const EMAC_O_HOSTXDESC = usize(0x00000C48);  // Ethernet MAC Current Host
                                            // Transmit Descriptor
pub const EMAC_O_HOSRXDESC = usize(0x00000C4C);  // Ethernet MAC Current Host
                                            // Receive Descriptor
pub const EMAC_O_HOSTXBA = usize(0x00000C50);  // Ethernet MAC Current Host
                                            // Transmit Buffer Address
pub const EMAC_O_HOSRXBA = usize(0x00000C54);  // Ethernet MAC Current Host
                                            // Receive Buffer Address
pub const EMAC_O_PP = usize(0x00000FC0);  // Ethernet MAC Peripheral Property
                                            // Register
pub const EMAC_O_PC = usize(0x00000FC4);  // Ethernet MAC Peripheral
                                            // Configuration Register
pub const EMAC_O_CC = usize(0x00000FC8);  // Ethernet MAC Clock Configuration
                                            // Register
pub const EMAC_O_EPHYRIS = usize(0x00000FD0);  // Ethernet PHY Raw Interrupt
                                            // Status
pub const EMAC_O_EPHYIM = usize(0x00000FD4);  // Ethernet PHY Interrupt Mask
pub const EMAC_O_EPHYMISC = usize(0x00000FD8);  // Ethernet PHY Masked Interrupt
                                            // Status and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_CFG register.
//
//*****************************************************************************
pub const EMAC_CFG_TWOKPEN = usize(0x08000000);  // IEEE 802
pub const EMAC_CFG_CST = usize(0x02000000);  // CRC Stripping for Type Frames
pub const EMAC_CFG_WDDIS = usize(0x00800000);  // Watchdog Disable
pub const EMAC_CFG_JD = usize(0x00400000);  // Jabber Disable
pub const EMAC_CFG_JFEN = usize(0x00100000);  // Jumbo Frame Enable
pub const EMAC_CFG_IFG_M = usize(0x000E0000);  // Inter-Frame Gap (IFG)
pub const EMAC_CFG_IFG_96 = usize(0x00000000);  // 96 bit times
pub const EMAC_CFG_IFG_88 = usize(0x00020000);  // 88 bit times
pub const EMAC_CFG_IFG_80 = usize(0x00040000);  // 80 bit times
pub const EMAC_CFG_IFG_72 = usize(0x00060000);  // 72 bit times
pub const EMAC_CFG_IFG_64 = usize(0x00080000);  // 64 bit times
pub const EMAC_CFG_IFG_56 = usize(0x000A0000);  // 56 bit times
pub const EMAC_CFG_IFG_48 = usize(0x000C0000);  // 48 bit times
pub const EMAC_CFG_IFG_40 = usize(0x000E0000);  // 40 bit times
pub const EMAC_CFG_DISCRS = usize(0x00010000);  // Disable Carrier Sense During
                                            // Transmission
pub const EMAC_CFG_PS = usize(0x00008000);  // Port Select
pub const EMAC_CFG_FES = usize(0x00004000);  // Speed
pub const EMAC_CFG_DRO = usize(0x00002000);  // Disable Receive Own
pub const EMAC_CFG_LOOPBM = usize(0x00001000);  // Loopback Mode
pub const EMAC_CFG_DUPM = usize(0x00000800);  // Duplex Mode
pub const EMAC_CFG_IPC = usize(0x00000400);  // Checksum Offload
pub const EMAC_CFG_DR = usize(0x00000200);  // Disable Retry
pub const EMAC_CFG_ACS = usize(0x00000080);  // Automatic Pad or CRC Stripping
pub const EMAC_CFG_BL_M = usize(0x00000060);  // Back-Off Limit
pub const EMAC_CFG_BL_1024 = usize(0x00000000);  // k = min (n,10)
pub const EMAC_CFG_BL_256 = usize(0x00000020);  // k = min (n,8)
pub const EMAC_CFG_BL_8 = usize(0x00000040);  // k = min (n,4)
pub const EMAC_CFG_BL_2 = usize(0x00000060);  // k = min (n,1)
pub const EMAC_CFG_DC = usize(0x00000010);  // Deferral Check
pub const EMAC_CFG_TE = usize(0x00000008);  // Transmitter Enable
pub const EMAC_CFG_RE = usize(0x00000004);  // Receiver Enable
pub const EMAC_CFG_PRELEN_M = usize(0x00000003);  // Preamble Length for Transmit
                                            // Frames
pub const EMAC_CFG_PRELEN_7 = usize(0x00000000);  // 7 bytes of preamble
pub const EMAC_CFG_PRELEN_5 = usize(0x00000001);  // 5 bytes of preamble
pub const EMAC_CFG_PRELEN_3 = usize(0x00000002);  // 3 bytes of preamble

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_FRAMEFLTR
// register.
//
//*****************************************************************************
pub const EMAC_FRAMEFLTR_RA = usize(0x80000000);  // Receive All
pub const EMAC_FRAMEFLTR_VTFE = usize(0x00010000);  // VLAN Tag Filter Enable
pub const EMAC_FRAMEFLTR_HPF = usize(0x00000400);  // Hash or Perfect Filter
pub const EMAC_FRAMEFLTR_SAF = usize(0x00000200);  // Source Address Filter Enable
pub const EMAC_FRAMEFLTR_SAIF = usize(0x00000100);  // Source Address (SA) Inverse
                                            // Filtering
pub const EMAC_FRAMEFLTR_PCF_M = usize(0x000000C0);  // Pass Control Frames
pub const EMAC_FRAMEFLTR_PCF_ALL = usize(0x00000000);  // The MAC filters all control
                                            // frames from reaching application
pub const EMAC_FRAMEFLTR_PCF_PAUSE = usize(0x00000040);  // MAC forwards all control frames
                                            // except PAUSE control frames to
                                            // application even if they fail
                                            // the address filter
pub const EMAC_FRAMEFLTR_PCF_NONE = usize(0x00000080);  // MAC forwards all control frames
                                            // to application even if they fail
                                            // the address Filter
pub const EMAC_FRAMEFLTR_PCF_ADDR = usize(0x000000C0);  // MAC forwards control frames that
                                            // pass the address Filter
pub const EMAC_FRAMEFLTR_DBF = usize(0x00000020);  // Disable Broadcast Frames
pub const EMAC_FRAMEFLTR_PM = usize(0x00000010);  // Pass All Multicast
pub const EMAC_FRAMEFLTR_DAIF = usize(0x00000008);  // Destination Address (DA) Inverse
                                            // Filtering
pub const EMAC_FRAMEFLTR_HMC = usize(0x00000004);  // Hash Multicast
pub const EMAC_FRAMEFLTR_HUC = usize(0x00000002);  // Hash Unicast
pub const EMAC_FRAMEFLTR_PR = usize(0x00000001);  // Promiscuous Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_HASHTBLH
// register.
//
//*****************************************************************************
pub const EMAC_HASHTBLH_HTH_M = usize(0xFFFFFFFF);  // Hash Table High
pub const EMAC_HASHTBLH_HTH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_HASHTBLL
// register.
//
//*****************************************************************************
pub const EMAC_HASHTBLL_HTL_M = usize(0xFFFFFFFF);  // Hash Table Low
pub const EMAC_HASHTBLL_HTL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_MIIADDR register.
//
//*****************************************************************************
pub const EMAC_MIIADDR_PLA_M = usize(0x0000F800);  // Physical Layer Address
pub const EMAC_MIIADDR_MII_M = usize(0x000007C0);  // MII Register
pub const EMAC_MIIADDR_CR_M = usize(0x0000003C);  // Clock Reference Frequency
                                            // Selection
pub const EMAC_MIIADDR_CR_60_100 = usize(0x00000000);  // The frequency of the System
                                            // Clock is 60 to 100 MHz providing
                                            // a MDIO clock of SYSCLK/42
pub const EMAC_MIIADDR_CR_100_150 = usize(0x00000004);  // The frequency of the System
                                            // Clock is 100 to 150 MHz
                                            // providing a MDIO clock of
                                            // SYSCLK/62
pub const EMAC_MIIADDR_CR_20_35 = usize(0x00000008);  // The frequency of the System
                                            // Clock is 20-35 MHz providing a
                                            // MDIO clock of System Clock/16
pub const EMAC_MIIADDR_CR_35_60 = usize(0x0000000C);  // The frequency of the System
                                            // Clock is 35 to 60 MHz providing
                                            // a MDIO clock of System Clock/26
pub const EMAC_MIIADDR_MIIW = usize(0x00000002);  // MII Write
pub const EMAC_MIIADDR_MIIB = usize(0x00000001);  // MII Busy
pub const EMAC_MIIADDR_PLA_S = usize(11);
pub const EMAC_MIIADDR_MII_S = usize(6);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_MIIDATA register.
//
//*****************************************************************************
pub const EMAC_MIIDATA_DATA_M = usize(0x0000FFFF);  // MII Data
pub const EMAC_MIIDATA_DATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_FLOWCTL register.
//
//*****************************************************************************
pub const EMAC_FLOWCTL_PT_M = usize(0xFFFF0000);  // Pause Time
pub const EMAC_FLOWCTL_DZQP = usize(0x00000080);  // Disable Zero-Quanta Pause
pub const EMAC_FLOWCTL_UP = usize(0x00000008);  // Unicast Pause Frame Detect
pub const EMAC_FLOWCTL_RFE = usize(0x00000004);  // Receive Flow Control Enable
pub const EMAC_FLOWCTL_TFE = usize(0x00000002);  // Transmit Flow Control Enable
pub const EMAC_FLOWCTL_FCBBPA = usize(0x00000001);  // Flow Control Busy or
                                            // Back-pressure Activate
pub const EMAC_FLOWCTL_PT_S = usize(16);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_VLANTG register.
//
//*****************************************************************************
pub const EMAC_VLANTG_VTHM = usize(0x00080000);  // VLAN Tag Hash Table Match Enable
pub const EMAC_VLANTG_ESVL = usize(0x00040000);  // Enable S-VLAN
pub const EMAC_VLANTG_VTIM = usize(0x00020000);  // VLAN Tag Inverse Match Enable
pub const EMAC_VLANTG_ETV = usize(0x00010000);  // Enable 12-Bit VLAN Tag
                                            // Comparison
pub const EMAC_VLANTG_VL_M = usize(0x0000FFFF);  // VLAN Tag Identifier for Receive
                                            // Frames
pub const EMAC_VLANTG_VL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_STATUS register.
//
//*****************************************************************************
pub const EMAC_STATUS_TXFF = usize(0x02000000);  // TX/RX Controller TX FIFO Full
                                            // Status
pub const EMAC_STATUS_TXFE = usize(0x01000000);  // TX/RX Controller TX FIFO Not
                                            // Empty Status
pub const EMAC_STATUS_TWC = usize(0x00400000);  // TX/RX Controller TX FIFO Write
                                            // Controller Active Status
pub const EMAC_STATUS_TRC_M = usize(0x00300000);  // TX/RX Controller's TX FIFO Read
                                            // Controller Status
pub const EMAC_STATUS_TRC_IDLE = usize(0x00000000);  // IDLE state
pub const EMAC_STATUS_TRC_READ = usize(0x00100000);  // READ state (transferring data to
                                            // MAC transmitter)
pub const EMAC_STATUS_TRC_WAIT = usize(0x00200000);  // Waiting for TX Status from MAC
                                            // transmitter
pub const EMAC_STATUS_TRC_WRFLUSH = usize(0x00300000);  // Writing the received TX Status
                                            // or flushing the TX FIFO
pub const EMAC_STATUS_TXPAUSED = usize(0x00080000);  // MAC Transmitter PAUSE
pub const EMAC_STATUS_TFC_M = usize(0x00060000);  // MAC Transmit Frame Controller
                                            // Status
pub const EMAC_STATUS_TFC_IDLE = usize(0x00000000);  // IDLE state
pub const EMAC_STATUS_TFC_STATUS = usize(0x00020000);  // Waiting for status of previous
                                            // frame or IFG or backoff period
                                            // to be over
pub const EMAC_STATUS_TFC_PAUSE = usize(0x00040000);  // Generating and transmitting a
                                            // PAUSE control frame (in the
                                            // full-duplex mode)
pub const EMAC_STATUS_TFC_INPUT = usize(0x00060000);  // Transferring input frame for
                                            // transmission
pub const EMAC_STATUS_TPE = usize(0x00010000);  // MAC MII Transmit Protocol Engine
                                            // Status
pub const EMAC_STATUS_RXF_M = usize(0x00000300);  // TX/RX Controller RX FIFO
                                            // Fill-level Status
pub const EMAC_STATUS_RXF_EMPTY = usize(0x00000000);  // RX FIFO Empty
pub const EMAC_STATUS_RXF_BELOW = usize(0x00000100);  // RX FIFO fill level is below the
                                            // flow-control deactivate
                                            // threshold
pub const EMAC_STATUS_RXF_ABOVE = usize(0x00000200);  // RX FIFO fill level is above the
                                            // flow-control activate threshold
pub const EMAC_STATUS_RXF_FULL = usize(0x00000300);  // RX FIFO Full
pub const EMAC_STATUS_RRC_M = usize(0x00000060);  // TX/RX Controller Read Controller
                                            // State
pub const EMAC_STATUS_RRC_IDLE = usize(0x00000000);  // IDLE state
pub const EMAC_STATUS_RRC_STATUS = usize(0x00000020);  // Reading frame data
pub const EMAC_STATUS_RRC_DATA = usize(0x00000040);  // Reading frame status (or
                                            // timestamp)
pub const EMAC_STATUS_RRC_FLUSH = usize(0x00000060);  // Flushing the frame data and
                                            // status
pub const EMAC_STATUS_RWC = usize(0x00000010);  // TX/RX Controller RX FIFO Write
                                            // Controller Active Status
pub const EMAC_STATUS_RFCFC_M = usize(0x00000006);  // MAC Receive Frame Controller
                                            // FIFO Status
pub const EMAC_STATUS_RPE = usize(0x00000001);  // MAC MII Receive Protocol Engine
                                            // Status
pub const EMAC_STATUS_RFCFC_S = usize(1);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_RWUFF register.
//
//*****************************************************************************
pub const EMAC_RWUFF_WAKEUPFIL_M = usize(0xFFFFFFFF);  // Remote Wake-Up Frame Filter
pub const EMAC_RWUFF_WAKEUPFIL_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_PMTCTLSTAT
// register.
//
//*****************************************************************************
pub const EMAC_PMTCTLSTAT_WUPFRRST = usize(0x80000000);  // Wake-Up Frame Filter Register
                                            // Pointer Reset
pub const EMAC_PMTCTLSTAT_RWKPTR_M = usize(0x07000000);  // Remote Wake-Up FIFO Pointer
pub const EMAC_PMTCTLSTAT_GLBLUCAST = usize(0x00000200);  // Global Unicast
pub const EMAC_PMTCTLSTAT_WUPRX = usize(0x00000040);  // Wake-Up Frame Received
pub const EMAC_PMTCTLSTAT_MGKPRX = usize(0x00000020);  // Magic Packet Received
pub const EMAC_PMTCTLSTAT_WUPFREN = usize(0x00000004);  // Wake-Up Frame Enable
pub const EMAC_PMTCTLSTAT_MGKPKTEN = usize(0x00000002);  // Magic Packet Enable
pub const EMAC_PMTCTLSTAT_PWRDWN = usize(0x00000001);  // Power Down
pub const EMAC_PMTCTLSTAT_RWKPTR_S = usize(24);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_LPICTLSTAT
// register.
//
//*****************************************************************************
pub const EMAC_LPICTLSTAT_LPITXA = usize(0x00080000);  // LPI TX Automate
pub const EMAC_LPICTLSTAT_PLSEN = usize(0x00040000);  // PHY Link Status Enable
pub const EMAC_LPICTLSTAT_PLS = usize(0x00020000);  // PHY Link Status
pub const EMAC_LPICTLSTAT_LPIEN = usize(0x00010000);  // LPI Enable
pub const EMAC_LPICTLSTAT_RLPIST = usize(0x00000200);  // Receive LPI State
pub const EMAC_LPICTLSTAT_TLPIST = usize(0x00000100);  // Transmit LPI State
pub const EMAC_LPICTLSTAT_RLPIEX = usize(0x00000008);  // Receive LPI Exit
pub const EMAC_LPICTLSTAT_RLPIEN = usize(0x00000004);  // Receive LPI Entry
pub const EMAC_LPICTLSTAT_TLPIEX = usize(0x00000002);  // Transmit LPI Exit
pub const EMAC_LPICTLSTAT_TLPIEN = usize(0x00000001);  // Transmit LPI Entry

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_LPITIMERCTL
// register.
//
//*****************************************************************************
pub const EMAC_LPITIMERCTL_LST_M = usize(0x03FF0000);  // Low Power Idle LS Timer
pub const EMAC_LPITIMERCTL_LST_S = usize(16);
pub const EMAC_LPITIMERCTL_TWT_M = usize(0x0000FFFF);  // Low Power Idle TW Timer
pub const EMAC_LPITIMERCTL_TWT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_RIS register.
//
//*****************************************************************************
pub const EMAC_RIS_LPI = usize(0x00000400);  // LPI Interrupt Status
pub const EMAC_RIS_TS = usize(0x00000200);  // Timestamp Interrupt Status
pub const EMAC_RIS_MMCTX = usize(0x00000040);  // MMC Transmit Interrupt Status
pub const EMAC_RIS_MMCRX = usize(0x00000020);  // MMC Receive Interrupt Status
pub const EMAC_RIS_MMC = usize(0x00000010);  // MMC Interrupt Status
pub const EMAC_RIS_PMT = usize(0x00000008);  // PMT Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_IM register.
//
//*****************************************************************************
pub const EMAC_IM_LPI = usize(0x00000400);  // LPI Interrupt Mask
pub const EMAC_IM_TSI = usize(0x00000200);  // Timestamp Interrupt Mask
pub const EMAC_IM_PMT = usize(0x00000008);  // PMT Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_ADDR0H register.
//
//*****************************************************************************
pub const EMAC_ADDR0H_AE = usize(0x80000000);  // Address Enable
pub const EMAC_ADDR0H_ADDRHI_M = usize(0x0000FFFF);  // MAC Address0 [47:32]
pub const EMAC_ADDR0H_ADDRHI_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_ADDR0L register.
//
//*****************************************************************************
pub const EMAC_ADDR0L_ADDRLO_M = usize(0xFFFFFFFF);  // MAC Address0 [31:0]
pub const EMAC_ADDR0L_ADDRLO_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_ADDR1H register.
//
//*****************************************************************************
pub const EMAC_ADDR1H_AE = usize(0x80000000);  // Address Enable
pub const EMAC_ADDR1H_SA = usize(0x40000000);  // Source Address
pub const EMAC_ADDR1H_MBC_M = usize(0x3F000000);  // Mask Byte Control
pub const EMAC_ADDR1H_ADDRHI_M = usize(0x0000FFFF);  // MAC Address1 [47:32]
pub const EMAC_ADDR1H_MBC_S = usize(24);
pub const EMAC_ADDR1H_ADDRHI_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_ADDR1L register.
//
//*****************************************************************************
pub const EMAC_ADDR1L_ADDRLO_M = usize(0xFFFFFFFF);  // MAC Address1 [31:0]
pub const EMAC_ADDR1L_ADDRLO_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_ADDR2H register.
//
//*****************************************************************************
pub const EMAC_ADDR2H_AE = usize(0x80000000);  // Address Enable
pub const EMAC_ADDR2H_SA = usize(0x40000000);  // Source Address
pub const EMAC_ADDR2H_MBC_M = usize(0x3F000000);  // Mask Byte Control
pub const EMAC_ADDR2H_ADDRHI_M = usize(0x0000FFFF);  // MAC Address2 [47:32]
pub const EMAC_ADDR2H_MBC_S = usize(24);
pub const EMAC_ADDR2H_ADDRHI_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_ADDR2L register.
//
//*****************************************************************************
pub const EMAC_ADDR2L_ADDRLO_M = usize(0xFFFFFFFF);  // MAC Address2 [31:0]
pub const EMAC_ADDR2L_ADDRLO_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_ADDR3H register.
//
//*****************************************************************************
pub const EMAC_ADDR3H_AE = usize(0x80000000);  // Address Enable
pub const EMAC_ADDR3H_SA = usize(0x40000000);  // Source Address
pub const EMAC_ADDR3H_MBC_M = usize(0x3F000000);  // Mask Byte Control
pub const EMAC_ADDR3H_ADDRHI_M = usize(0x0000FFFF);  // MAC Address3 [47:32]
pub const EMAC_ADDR3H_MBC_S = usize(24);
pub const EMAC_ADDR3H_ADDRHI_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_ADDR3L register.
//
//*****************************************************************************
pub const EMAC_ADDR3L_ADDRLO_M = usize(0xFFFFFFFF);  // MAC Address3 [31:0]
pub const EMAC_ADDR3L_ADDRLO_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_WDOGTO register.
//
//*****************************************************************************
pub const EMAC_WDOGTO_PWE = usize(0x00010000);  // Programmable Watchdog Enable
pub const EMAC_WDOGTO_WTO_M = usize(0x00003FFF);  // Watchdog Timeout
pub const EMAC_WDOGTO_WTO_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_MMCCTRL register.
//
//*****************************************************************************
pub const EMAC_MMCCTRL_UCDBC = usize(0x00000100);  // Update MMC Counters for Dropped
                                            // Broadcast Frames
pub const EMAC_MMCCTRL_CNTPRSTLVL = usize(0x00000020);  // Full/Half Preset Level Value
pub const EMAC_MMCCTRL_CNTPRST = usize(0x00000010);  // Counters Preset
pub const EMAC_MMCCTRL_CNTFREEZ = usize(0x00000008);  // MMC Counter Freeze
pub const EMAC_MMCCTRL_RSTONRD = usize(0x00000004);  // Reset on Read
pub const EMAC_MMCCTRL_CNTSTPRO = usize(0x00000002);  // Counters Stop Rollover
pub const EMAC_MMCCTRL_CNTRST = usize(0x00000001);  // Counters Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_MMCRXRIS
// register.
//
//*****************************************************************************
pub const EMAC_MMCRXRIS_UCGF = usize(0x00020000);  // MMC Receive Unicast Good Frame
                                            // Counter Interrupt Status
pub const EMAC_MMCRXRIS_ALGNERR = usize(0x00000040);  // MMC Receive Alignment Error
                                            // Frame Counter Interrupt Status
pub const EMAC_MMCRXRIS_CRCERR = usize(0x00000020);  // MMC Receive CRC Error Frame
                                            // Counter Interrupt Status
pub const EMAC_MMCRXRIS_GBF = usize(0x00000001);  // MMC Receive Good Bad Frame
                                            // Counter Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_MMCTXRIS
// register.
//
//*****************************************************************************
pub const EMAC_MMCTXRIS_OCTCNT = usize(0x00100000);  // Octet Counter Interrupt Status
pub const EMAC_MMCTXRIS_MCOLLGF = usize(0x00008000);  // MMC Transmit Multiple Collision
                                            // Good Frame Counter Interrupt
                                            // Status
pub const EMAC_MMCTXRIS_SCOLLGF = usize(0x00004000);  // MMC Transmit Single Collision
                                            // Good Frame Counter Interrupt
                                            // Status
pub const EMAC_MMCTXRIS_GBF = usize(0x00000002);  // MMC Transmit Good Bad Frame
                                            // Counter Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_MMCRXIM register.
//
//*****************************************************************************
pub const EMAC_MMCRXIM_UCGF = usize(0x00020000);  // MMC Receive Unicast Good Frame
                                            // Counter Interrupt Mask
pub const EMAC_MMCRXIM_ALGNERR = usize(0x00000040);  // MMC Receive Alignment Error
                                            // Frame Counter Interrupt Mask
pub const EMAC_MMCRXIM_CRCERR = usize(0x00000020);  // MMC Receive CRC Error Frame
                                            // Counter Interrupt Mask
pub const EMAC_MMCRXIM_GBF = usize(0x00000001);  // MMC Receive Good Bad Frame
                                            // Counter Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_MMCTXIM register.
//
//*****************************************************************************
pub const EMAC_MMCTXIM_OCTCNT = usize(0x00100000);  // MMC Transmit Good Octet Counter
                                            // Interrupt Mask
pub const EMAC_MMCTXIM_MCOLLGF = usize(0x00008000);  // MMC Transmit Multiple Collision
                                            // Good Frame Counter Interrupt
                                            // Mask
pub const EMAC_MMCTXIM_SCOLLGF = usize(0x00004000);  // MMC Transmit Single Collision
                                            // Good Frame Counter Interrupt
                                            // Mask
pub const EMAC_MMCTXIM_GBF = usize(0x00000002);  // MMC Transmit Good Bad Frame
                                            // Counter Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TXCNTGB register.
//
//*****************************************************************************
pub const EMAC_TXCNTGB_TXFRMGB_M = usize(0xFFFFFFFF);  // This field indicates the number
                                            // of good and bad frames
                                            // transmitted, exclusive of
                                            // retried frames
pub const EMAC_TXCNTGB_TXFRMGB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TXCNTSCOL
// register.
//
//*****************************************************************************
pub const EMAC_TXCNTSCOL_TXSNGLCOLG_M = usize(0xFFFFFFFF);  // This field indicates the number
                                            // of successfully transmitted
                                            // frames after a single collision
                                            // in the half-duplex mode
pub const EMAC_TXCNTSCOL_TXSNGLCOLG_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TXCNTMCOL
// register.
//
//*****************************************************************************
pub const EMAC_TXCNTMCOL_TXMULTCOLG_M = usize(0xFFFFFFFF);  // This field indicates the number
                                            // of successfully transmitted
                                            // frames after multiple collisions
                                            // in the half-duplex mode
pub const EMAC_TXCNTMCOL_TXMULTCOLG_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TXOCTCNTG
// register.
//
//*****************************************************************************
pub const EMAC_TXOCTCNTG_TXOCTG_M = usize(0xFFFFFFFF);  // This field indicates the number
                                            // of bytes transmitted, exclusive
                                            // of preamble, in good frames
pub const EMAC_TXOCTCNTG_TXOCTG_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_RXCNTGB register.
//
//*****************************************************************************
pub const EMAC_RXCNTGB_RXFRMGB_M = usize(0xFFFFFFFF);  // This field indicates the number
                                            // of received good and bad frames
pub const EMAC_RXCNTGB_RXFRMGB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_RXCNTCRCERR
// register.
//
//*****************************************************************************
pub const EMAC_RXCNTCRCERR_RXCRCERR_M = usize(0xFFFFFFFF);  // This field indicates the number
                                            // of frames received with CRC
                                            // error
pub const EMAC_RXCNTCRCERR_RXCRCERR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_RXCNTALGNERR
// register.
//
//*****************************************************************************
pub const EMAC_RXCNTALGNERR_RXALGNERR_M = usize(0xFFFFFFFF);  // This field indicates the number
                                            // of frames received with
                                            // alignment (dribble) error
pub const EMAC_RXCNTALGNERR_RXALGNERR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_RXCNTGUNI
// register.
//
//*****************************************************************************
pub const EMAC_RXCNTGUNI_RXUCASTG_M = usize(0xFFFFFFFF);  // This field indicates the number
                                            // of received good unicast frames
pub const EMAC_RXCNTGUNI_RXUCASTG_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_VLNINCREP
// register.
//
//*****************************************************************************
pub const EMAC_VLNINCREP_CSVL = usize(0x00080000);  // C-VLAN or S-VLAN
pub const EMAC_VLNINCREP_VLP = usize(0x00040000);  // VLAN Priority Control
pub const EMAC_VLNINCREP_VLC_M = usize(0x00030000);  // VLAN Tag Control in Transmit
                                            // Frames
pub const EMAC_VLNINCREP_VLC_NONE = usize(0x00000000);  // No VLAN tag deletion, insertion,
                                            // or replacement
pub const EMAC_VLNINCREP_VLC_TAGDEL = usize(0x00010000);  // VLAN tag deletion
pub const EMAC_VLNINCREP_VLC_TAGINS = usize(0x00020000);  // VLAN tag insertion
pub const EMAC_VLNINCREP_VLC_TAGREP = usize(0x00030000);  // VLAN tag replacement
pub const EMAC_VLNINCREP_VLT_M = usize(0x0000FFFF);  // VLAN Tag for Transmit Frames
pub const EMAC_VLNINCREP_VLT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_VLANHASH
// register.
//
//*****************************************************************************
pub const EMAC_VLANHASH_VLHT_M = usize(0x0000FFFF);  // VLAN Hash Table
pub const EMAC_VLANHASH_VLHT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TIMSTCTRL
// register.
//
//*****************************************************************************
pub const EMAC_TIMSTCTRL_PTPFLTR = usize(0x00040000);  // Enable MAC address for PTP Frame
                                            // Filtering
pub const EMAC_TIMSTCTRL_SELPTP_M = usize(0x00030000);  // Select PTP packets for Taking
                                            // Snapshots
pub const EMAC_TIMSTCTRL_TSMAST = usize(0x00008000);  // Enable Snapshot for Messages
                                            // Relevant to Master
pub const EMAC_TIMSTCTRL_TSEVNT = usize(0x00004000);  // Enable Timestamp Snapshot for
                                            // Event Messages
pub const EMAC_TIMSTCTRL_PTPIPV4 = usize(0x00002000);  // Enable Processing of PTP Frames
                                            // Sent over IPv4-UDP
pub const EMAC_TIMSTCTRL_PTPIPV6 = usize(0x00001000);  // Enable Processing of PTP Frames
                                            // Sent Over IPv6-UDP
pub const EMAC_TIMSTCTRL_PTPETH = usize(0x00000800);  // Enable Processing of PTP Over
                                            // Ethernet Frames
pub const EMAC_TIMSTCTRL_PTPVER2 = usize(0x00000400);  // Enable PTP Packet Processing For
                                            // Version 2 Format
pub const EMAC_TIMSTCTRL_DGTLBIN = usize(0x00000200);  // Timestamp Digital or Binary
                                            // Rollover Control
pub const EMAC_TIMSTCTRL_ALLF = usize(0x00000100);  // Enable Timestamp For All Frames
pub const EMAC_TIMSTCTRL_ADDREGUP = usize(0x00000020);  // Addend Register Update
pub const EMAC_TIMSTCTRL_INTTRIG = usize(0x00000010);  // Timestamp Interrupt Trigger
                                            // Enable
pub const EMAC_TIMSTCTRL_TSUPDT = usize(0x00000008);  // Timestamp Update
pub const EMAC_TIMSTCTRL_TSINIT = usize(0x00000004);  // Timestamp Initialize
pub const EMAC_TIMSTCTRL_TSFCUPDT = usize(0x00000002);  // Timestamp Fine or Coarse Update
pub const EMAC_TIMSTCTRL_TSEN = usize(0x00000001);  // Timestamp Enable
pub const EMAC_TIMSTCTRL_SELPTP_S = usize(16);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_SUBSECINC
// register.
//
//*****************************************************************************
pub const EMAC_SUBSECINC_SSINC_M = usize(0x000000FF);  // Sub-second Increment Value
pub const EMAC_SUBSECINC_SSINC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TIMSEC register.
//
//*****************************************************************************
pub const EMAC_TIMSEC_TSS_M = usize(0xFFFFFFFF);  // Timestamp Second
pub const EMAC_TIMSEC_TSS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TIMNANO register.
//
//*****************************************************************************
pub const EMAC_TIMNANO_TSSS_M = usize(0x7FFFFFFF);  // Timestamp Sub-Seconds
pub const EMAC_TIMNANO_TSSS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TIMSECU register.
//
//*****************************************************************************
pub const EMAC_TIMSECU_TSS_M = usize(0xFFFFFFFF);  // Timestamp Second
pub const EMAC_TIMSECU_TSS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TIMNANOU
// register.
//
//*****************************************************************************
pub const EMAC_TIMNANOU_ADDSUB = usize(0x80000000);  // Add or subtract time
pub const EMAC_TIMNANOU_TSSS_M = usize(0x7FFFFFFF);  // Timestamp Sub-Second
pub const EMAC_TIMNANOU_TSSS_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TIMADD register.
//
//*****************************************************************************
pub const EMAC_TIMADD_TSAR_M = usize(0xFFFFFFFF);  // Timestamp Addend Register
pub const EMAC_TIMADD_TSAR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TARGSEC register.
//
//*****************************************************************************
pub const EMAC_TARGSEC_TSTR_M = usize(0xFFFFFFFF);  // Target Time Seconds Register
pub const EMAC_TARGSEC_TSTR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TARGNANO
// register.
//
//*****************************************************************************
pub const EMAC_TARGNANO_TRGTBUSY = usize(0x80000000);  // Target Time Register Busy
pub const EMAC_TARGNANO_TTSLO_M = usize(0x7FFFFFFF);  // Target Timestamp Low Register
pub const EMAC_TARGNANO_TTSLO_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_HWORDSEC
// register.
//
//*****************************************************************************
pub const EMAC_HWORDSEC_TSHWR_M = usize(0x0000FFFF);  // Target Timestamp Higher Word
                                            // Register
pub const EMAC_HWORDSEC_TSHWR_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TIMSTAT register.
//
//*****************************************************************************
pub const EMAC_TIMSTAT_TSTARGT = usize(0x00000002);  // Timestamp Target Time Reached
pub const EMAC_TIMSTAT_TSSOVF = usize(0x00000001);  // Timestamp Seconds Overflow

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_PPSCTRL register.
//
//*****************************************************************************
pub const EMAC_PPSCTRL_TRGMODS0_M = usize(0x00000060);  // Target Time Register Mode for
                                            // PPS0 Output
pub const EMAC_PPSCTRL_TRGMODS0_INTONLY = usize(0x00000000);  // Indicates that the Target Time
                                            // registers are programmed only
                                            // for generating the interrupt
                                            // event
pub const EMAC_PPSCTRL_TRGMODS0_INTPPS0 = usize(0x00000040);  // Indicates that the Target Time
                                            // registers are programmed for
                                            // generating the interrupt event
                                            // and starting or stopping the
                                            // generation of the EN0PPS output
                                            // signal
pub const EMAC_PPSCTRL_TRGMODS0_PPS0ONLY = usize(0x00000060);  // Indicates that the Target Time
                                            // registers are programmed only
                                            // for starting or stopping the
                                            // generation of the EN0PPS output
                                            // signal. No interrupt is asserted
pub const EMAC_PPSCTRL_PPSEN0 = usize(0x00000010);  // Flexible PPS Output Mode Enable
pub const EMAC_PPSCTRL_PPSCTRL_M = usize(0x0000000F);  // EN0PPS Output Frequency Control
                                            // (PPSCTRL) or Command Control
                                            // (PPSCMD)

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_PPS0INTVL
// register.
//
//*****************************************************************************
pub const EMAC_PPS0INTVL_PPS0INT_M = usize(0xFFFFFFFF);  // PPS0 Output Signal Interval
pub const EMAC_PPS0INTVL_PPS0INT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_PPS0WIDTH
// register.
//
//*****************************************************************************
pub const EMAC_PPS0WIDTH_M = usize(0xFFFFFFFF);  // EN0PPS Output Signal Width
pub const EMAC_PPS0WIDTH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_DMABUSMOD
// register.
//
//*****************************************************************************
pub const EMAC_DMABUSMOD_RIB = usize(0x80000000);  // Rebuild Burst
pub const EMAC_DMABUSMOD_TXPR = usize(0x08000000);  // Transmit Priority
pub const EMAC_DMABUSMOD_MB = usize(0x04000000);  // Mixed Burst
pub const EMAC_DMABUSMOD_AAL = usize(0x02000000);  // Address Aligned Beats
pub const EMAC_DMABUSMOD_8XPBL = usize(0x01000000);  // 8 x Programmable Burst Length
                                            // (PBL) Mode
pub const EMAC_DMABUSMOD_USP = usize(0x00800000);  // Use Separate Programmable Burst
                                            // Length (PBL)
pub const EMAC_DMABUSMOD_RPBL_M = usize(0x007E0000);  // RX DMA Programmable Burst Length
                                            // (PBL)
pub const EMAC_DMABUSMOD_FB = usize(0x00010000);  // Fixed Burst
pub const EMAC_DMABUSMOD_PR_M = usize(0x0000C000);  // Priority Ratio
pub const EMAC_DMABUSMOD_PBL_M = usize(0x00003F00);  // Programmable Burst Length
pub const EMAC_DMABUSMOD_ATDS = usize(0x00000080);  // Alternate Descriptor Size
pub const EMAC_DMABUSMOD_DSL_M = usize(0x0000007C);  // Descriptor Skip Length
pub const EMAC_DMABUSMOD_DA = usize(0x00000002);  // DMA Arbitration Scheme
pub const EMAC_DMABUSMOD_SWR = usize(0x00000001);  // DMA Software Reset
pub const EMAC_DMABUSMOD_RPBL_S = usize(17);
pub const EMAC_DMABUSMOD_PR_S = usize(14);
pub const EMAC_DMABUSMOD_PBL_S = usize(8);
pub const EMAC_DMABUSMOD_DSL_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TXPOLLD register.
//
//*****************************************************************************
pub const EMAC_TXPOLLD_TPD_M = usize(0xFFFFFFFF);  // Transmit Poll Demand
pub const EMAC_TXPOLLD_TPD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_RXPOLLD register.
//
//*****************************************************************************
pub const EMAC_RXPOLLD_RPD_M = usize(0xFFFFFFFF);  // Receive Poll Demand
pub const EMAC_RXPOLLD_RPD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_RXDLADDR
// register.
//
//*****************************************************************************
pub const EMAC_RXDLADDR_STRXLIST_M = usize(0xFFFFFFFC);  // Start of Receive List
pub const EMAC_RXDLADDR_STRXLIST_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_TXDLADDR
// register.
//
//*****************************************************************************
pub const EMAC_TXDLADDR_TXDLADDR_M = usize(0xFFFFFFFC);  // Start of Transmit List Base
                                            // Address
pub const EMAC_TXDLADDR_TXDLADDR_S = usize(2);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_DMARIS register.
//
//*****************************************************************************
pub const EMAC_DMARIS_LPI = usize(0x40000000);  // LPI Trigger Interrupt Status
pub const EMAC_DMARIS_TT = usize(0x20000000);  // Timestamp Trigger Interrupt
                                            // Status
pub const EMAC_DMARIS_PMT = usize(0x10000000);  // MAC PMT Interrupt Status
pub const EMAC_DMARIS_MMC = usize(0x08000000);  // MAC MMC Interrupt
pub const EMAC_DMARIS_AE_M = usize(0x03800000);  // Access Error
pub const EMAC_DMARIS_AE_RXDMAWD = usize(0x00000000);  // Error during RX DMA Write Data
                                            // Transfer
pub const EMAC_DMARIS_AE_TXDMARD = usize(0x01800000);  // Error during TX DMA Read Data
                                            // Transfer
pub const EMAC_DMARIS_AE_RXDMADW = usize(0x02000000);  // Error during RX DMA Descriptor
                                            // Write Access
pub const EMAC_DMARIS_AE_TXDMADW = usize(0x02800000);  // Error during TX DMA Descriptor
                                            // Write Access
pub const EMAC_DMARIS_AE_RXDMADR = usize(0x03000000);  // Error during RX DMA Descriptor
                                            // Read Access
pub const EMAC_DMARIS_AE_TXDMADR = usize(0x03800000);  // Error during TX DMA Descriptor
                                            // Read Access
pub const EMAC_DMARIS_TS_M = usize(0x00700000);  // Transmit Process State
pub const EMAC_DMARIS_TS_STOP = usize(0x00000000);  // Stopped; Reset or Stop transmit
                                            // command processed
pub const EMAC_DMARIS_TS_RUNTXTD = usize(0x00100000);  // Running; Fetching transmit
                                            // transfer descriptor
pub const EMAC_DMARIS_TS_STATUS = usize(0x00200000);  // Running; Waiting for status
pub const EMAC_DMARIS_TS_RUNTX = usize(0x00300000);  // Running; Reading data from host
                                            // memory buffer and queuing it to
                                            // transmit buffer (TX FIFO)
pub const EMAC_DMARIS_TS_TSTAMP = usize(0x00400000);  // Writing Timestamp
pub const EMAC_DMARIS_TS_SUSPEND = usize(0x00600000);  // Suspended; Transmit descriptor
                                            // unavailable or transmit buffer
                                            // underflow
pub const EMAC_DMARIS_TS_RUNCTD = usize(0x00700000);  // Running; Closing transmit
                                            // descriptor
pub const EMAC_DMARIS_RS_M = usize(0x000E0000);  // Received Process State
pub const EMAC_DMARIS_RS_STOP = usize(0x00000000);  // Stopped: Reset or stop receive
                                            // command issued
pub const EMAC_DMARIS_RS_RUNRXTD = usize(0x00020000);  // Running: Fetching receive
                                            // transfer descriptor
pub const EMAC_DMARIS_RS_RUNRXD = usize(0x00060000);  // Running: Waiting for receive
                                            // packet
pub const EMAC_DMARIS_RS_SUSPEND = usize(0x00080000);  // Suspended: Receive descriptor
                                            // unavailable
pub const EMAC_DMARIS_RS_RUNCRD = usize(0x000A0000);  // Running: Closing receive
                                            // descriptor
pub const EMAC_DMARIS_RS_TSWS = usize(0x000C0000);  // Writing Timestamp
pub const EMAC_DMARIS_RS_RUNTXD = usize(0x000E0000);  // Running: Transferring the
                                            // receive packet data from receive
                                            // buffer to host memory
pub const EMAC_DMARIS_NIS = usize(0x00010000);  // Normal Interrupt Summary
pub const EMAC_DMARIS_AIS = usize(0x00008000);  // Abnormal Interrupt Summary
pub const EMAC_DMARIS_ERI = usize(0x00004000);  // Early Receive Interrupt
pub const EMAC_DMARIS_FBI = usize(0x00002000);  // Fatal Bus Error Interrupt
pub const EMAC_DMARIS_ETI = usize(0x00000400);  // Early Transmit Interrupt
pub const EMAC_DMARIS_RWT = usize(0x00000200);  // Receive Watchdog Timeout
pub const EMAC_DMARIS_RPS = usize(0x00000100);  // Receive Process Stopped
pub const EMAC_DMARIS_RU = usize(0x00000080);  // Receive Buffer Unavailable
pub const EMAC_DMARIS_RI = usize(0x00000040);  // Receive Interrupt
pub const EMAC_DMARIS_UNF = usize(0x00000020);  // Transmit Underflow
pub const EMAC_DMARIS_OVF = usize(0x00000010);  // Receive Overflow
pub const EMAC_DMARIS_TJT = usize(0x00000008);  // Transmit Jabber Timeout
pub const EMAC_DMARIS_TU = usize(0x00000004);  // Transmit Buffer Unavailable
pub const EMAC_DMARIS_TPS = usize(0x00000002);  // Transmit Process Stopped
pub const EMAC_DMARIS_TI = usize(0x00000001);  // Transmit Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_DMAOPMODE
// register.
//
//*****************************************************************************
pub const EMAC_DMAOPMODE_DT = usize(0x04000000);  // Disable Dropping of TCP/IP
                                            // Checksum Error Frames
pub const EMAC_DMAOPMODE_RSF = usize(0x02000000);  // Receive Store and Forward
pub const EMAC_DMAOPMODE_DFF = usize(0x01000000);  // Disable Flushing of Received
                                            // Frames
pub const EMAC_DMAOPMODE_TSF = usize(0x00200000);  // Transmit Store and Forward
pub const EMAC_DMAOPMODE_FTF = usize(0x00100000);  // Flush Transmit FIFO
pub const EMAC_DMAOPMODE_TTC_M = usize(0x0001C000);  // Transmit Threshold Control
pub const EMAC_DMAOPMODE_TTC_64 = usize(0x00000000);  // 64 bytes
pub const EMAC_DMAOPMODE_TTC_128 = usize(0x00004000);  // 128 bytes
pub const EMAC_DMAOPMODE_TTC_192 = usize(0x00008000);  // 192 bytes
pub const EMAC_DMAOPMODE_TTC_256 = usize(0x0000C000);  // 256 bytes
pub const EMAC_DMAOPMODE_TTC_40 = usize(0x00010000);  // 40 bytes
pub const EMAC_DMAOPMODE_TTC_32 = usize(0x00014000);  // 32 bytes
pub const EMAC_DMAOPMODE_TTC_24 = usize(0x00018000);  // 24 bytes
pub const EMAC_DMAOPMODE_TTC_16 = usize(0x0001C000);  // 16 bytes
pub const EMAC_DMAOPMODE_ST = usize(0x00002000);  // Start or Stop Transmission
                                            // Command
pub const EMAC_DMAOPMODE_FEF = usize(0x00000080);  // Forward Error Frames
pub const EMAC_DMAOPMODE_FUF = usize(0x00000040);  // Forward Undersized Good Frames
pub const EMAC_DMAOPMODE_DGF = usize(0x00000020);  // Drop Giant Frame Enable
pub const EMAC_DMAOPMODE_RTC_M = usize(0x00000018);  // Receive Threshold Control
pub const EMAC_DMAOPMODE_RTC_64 = usize(0x00000000);  // 64 bytes
pub const EMAC_DMAOPMODE_RTC_32 = usize(0x00000008);  // 32 bytes
pub const EMAC_DMAOPMODE_RTC_96 = usize(0x00000010);  // 96 bytes
pub const EMAC_DMAOPMODE_RTC_128 = usize(0x00000018);  // 128 bytes
pub const EMAC_DMAOPMODE_OSF = usize(0x00000004);  // Operate on Second Frame
pub const EMAC_DMAOPMODE_SR = usize(0x00000002);  // Start or Stop Receive

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_DMAIM register.
//
//*****************************************************************************
pub const EMAC_DMAIM_NIE = usize(0x00010000);  // Normal Interrupt Summary Enable
pub const EMAC_DMAIM_AIE = usize(0x00008000);  // Abnormal Interrupt Summary
                                            // Enable
pub const EMAC_DMAIM_ERE = usize(0x00004000);  // Early Receive Interrupt Enable
pub const EMAC_DMAIM_FBE = usize(0x00002000);  // Fatal Bus Error Enable
pub const EMAC_DMAIM_ETE = usize(0x00000400);  // Early Transmit Interrupt Enable
pub const EMAC_DMAIM_RWE = usize(0x00000200);  // Receive Watchdog Timeout Enable
pub const EMAC_DMAIM_RSE = usize(0x00000100);  // Receive Stopped Enable
pub const EMAC_DMAIM_RUE = usize(0x00000080);  // Receive Buffer Unavailable
                                            // Enable
pub const EMAC_DMAIM_RIE = usize(0x00000040);  // Receive Interrupt Enable
pub const EMAC_DMAIM_UNE = usize(0x00000020);  // Underflow Interrupt Enable
pub const EMAC_DMAIM_OVE = usize(0x00000010);  // Overflow Interrupt Enable
pub const EMAC_DMAIM_TJE = usize(0x00000008);  // Transmit Jabber Timeout Enable
pub const EMAC_DMAIM_TUE = usize(0x00000004);  // Transmit Buffer Unvailable
                                            // Enable
pub const EMAC_DMAIM_TSE = usize(0x00000002);  // Transmit Stopped Enable
pub const EMAC_DMAIM_TIE = usize(0x00000001);  // Transmit Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_MFBOC register.
//
//*****************************************************************************
pub const EMAC_MFBOC_OVFCNTOVF = usize(0x10000000);  // Overflow Bit for FIFO Overflow
                                            // Counter
pub const EMAC_MFBOC_OVFFRMCNT_M = usize(0x0FFE0000);  // Overflow Frame Counter
pub const EMAC_MFBOC_MISCNTOVF = usize(0x00010000);  // Overflow bit for Missed Frame
                                            // Counter
pub const EMAC_MFBOC_MISFRMCNT_M = usize(0x0000FFFF);  // Missed Frame Counter
pub const EMAC_MFBOC_OVFFRMCNT_S = usize(17);
pub const EMAC_MFBOC_MISFRMCNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_RXINTWDT
// register.
//
//*****************************************************************************
pub const EMAC_RXINTWDT_RIWT_M = usize(0x000000FF);  // Receive Interrupt Watchdog Timer
                                            // Count
pub const EMAC_RXINTWDT_RIWT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_HOSTXDESC
// register.
//
//*****************************************************************************
pub const EMAC_HOSTXDESC_CURTXDESC_M = usize(0xFFFFFFFF);  // Host Transmit Descriptor Address
                                            // Pointer
pub const EMAC_HOSTXDESC_CURTXDESC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_HOSRXDESC
// register.
//
//*****************************************************************************
pub const EMAC_HOSRXDESC_CURRXDESC_M = usize(0xFFFFFFFF);  // Host Receive Descriptor Address
                                            // Pointer
pub const EMAC_HOSRXDESC_CURRXDESC_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_HOSTXBA register.
//
//*****************************************************************************
pub const EMAC_HOSTXBA_CURTXBUFA_M = usize(0xFFFFFFFF);  // Host Transmit Buffer Address
                                            // Pointer
pub const EMAC_HOSTXBA_CURTXBUFA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_HOSRXBA register.
//
//*****************************************************************************
pub const EMAC_HOSRXBA_CURRXBUFA_M = usize(0xFFFFFFFF);  // Host Receive Buffer Address
                                            // Pointer
pub const EMAC_HOSRXBA_CURRXBUFA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_PP register.
//
//*****************************************************************************
pub const EMAC_PP_MACTYPE_M = usize(0x00000700);  // Ethernet MAC Type
pub const EMAC_PP_MACTYPE_1 = usize(0x00000100);  // Tiva TM4E129x-class MAC
pub const EMAC_PP_PHYTYPE_M = usize(0x00000007);  // Ethernet PHY Type
pub const EMAC_PP_PHYTYPE_NONE = usize(0x00000000);  // No PHY
pub const EMAC_PP_PHYTYPE_1 = usize(0x00000003);  // Snowflake class PHY

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_PC register.
//
//*****************************************************************************
pub const EMAC_PC_PHYEXT = usize(0x80000000);  // PHY Select
pub const EMAC_PC_PINTFS_M = usize(0x70000000);  // Ethernet Interface Select
pub const EMAC_PC_PINTFS_IMII = usize(0x00000000);  // MII (default) Used for internal
                                            // PHY or external PHY connected
                                            // via MII
pub const EMAC_PC_PINTFS_RMII = usize(0x40000000);  // RMII: Used for external PHY
                                            // connected via RMII
pub const EMAC_PC_DIGRESTART = usize(0x02000000);  // PHY Soft Restart
pub const EMAC_PC_NIBDETDIS = usize(0x01000000);  // Odd Nibble TXER Detection
                                            // Disable
pub const EMAC_PC_RXERIDLE = usize(0x00800000);  // RXER Detection During Idle
pub const EMAC_PC_ISOMIILL = usize(0x00400000);  // Isolate MII in Link Loss
pub const EMAC_PC_LRR = usize(0x00200000);  // Link Loss Recovery
pub const EMAC_PC_TDRRUN = usize(0x00100000);  // TDR Auto Run
pub const EMAC_PC_FASTLDMODE_M = usize(0x000F8000);  // Fast Link Down Mode
pub const EMAC_PC_POLSWAP = usize(0x00004000);  // Polarity Swap
pub const EMAC_PC_MDISWAP = usize(0x00002000);  // MDI Swap
pub const EMAC_PC_RBSTMDIX = usize(0x00001000);  // Robust Auto MDI-X
pub const EMAC_PC_FASTMDIX = usize(0x00000800);  // Fast Auto MDI-X
pub const EMAC_PC_MDIXEN = usize(0x00000400);  // MDIX Enable
pub const EMAC_PC_FASTRXDV = usize(0x00000200);  // Fast RXDV Detection
pub const EMAC_PC_FASTLUPD = usize(0x00000100);  // FAST Link-Up in Parallel Detect
pub const EMAC_PC_EXTFD = usize(0x00000080);  // Extended Full Duplex Ability
pub const EMAC_PC_FASTANEN = usize(0x00000040);  // Fast Auto Negotiation Enable
pub const EMAC_PC_FASTANSEL_M = usize(0x00000030);  // Fast Auto Negotiation Select
pub const EMAC_PC_ANEN = usize(0x00000008);  // Auto Negotiation Enable
pub const EMAC_PC_ANMODE_M = usize(0x00000006);  // Auto Negotiation Mode
pub const EMAC_PC_ANMODE_10HD = usize(0x00000000);  // When ANEN = 0x0, the mode is
                                            // 10Base-T, Half-Duplex
pub const EMAC_PC_ANMODE_10FD = usize(0x00000002);  // When ANEN = 0x0, the mode is
                                            // 10Base-T, Full-Duplex
pub const EMAC_PC_ANMODE_100HD = usize(0x00000004);  // When ANEN = 0x0, the mode is
                                            // 100Base-TX, Half-Duplex
pub const EMAC_PC_ANMODE_100FD = usize(0x00000006);  // When ANEN = 0x0, the mode is
                                            // 100Base-TX, Full-Duplex
pub const EMAC_PC_PHYHOLD = usize(0x00000001);  // Ethernet PHY Hold
pub const EMAC_PC_FASTLDMODE_S = usize(15);
pub const EMAC_PC_FASTANSEL_S = usize(4);

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_CC register.
//
//*****************************************************************************
pub const EMAC_CC_PTPCEN = usize(0x00040000);  // PTP Clock Reference Enable
pub const EMAC_CC_POL = usize(0x00020000);  // LED Polarity Control
pub const EMAC_CC_CLKEN = usize(0x00010000);  // EN0RREF_CLK Signal Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_EPHYRIS register.
//
//*****************************************************************************
pub const EMAC_EPHYRIS_INT = usize(0x00000001);  // Ethernet PHY Raw Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_EPHYIM register.
//
//*****************************************************************************
pub const EMAC_EPHYIM_INT = usize(0x00000001);  // Ethernet PHY Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the EMAC_O_EPHYMISC
// register.
//
//*****************************************************************************
pub const EMAC_EPHYMISC_INT = usize(0x00000001);  // Ethernet PHY Status and Clear
                                            // register

//*****************************************************************************
//
// The following are defines for the EPHY register offsets.
//
//*****************************************************************************
pub const EPHY_BMCR = usize(0x00000000);  // Ethernet PHY Basic Mode Control
pub const EPHY_BMSR = usize(0x00000001);  // Ethernet PHY Basic Mode Status
pub const EPHY_ID1 = usize(0x00000002);  // Ethernet PHY Identifier Register
                                            // 1
pub const EPHY_ID2 = usize(0x00000003);  // Ethernet PHY Identifier Register
                                            // 2
pub const EPHY_ANA = usize(0x00000004);  // Ethernet PHY Auto-Negotiation
                                            // Advertisement
pub const EPHY_ANLPA = usize(0x00000005);  // Ethernet PHY Auto-Negotiation
                                            // Link Partner Ability
pub const EPHY_ANER = usize(0x00000006);  // Ethernet PHY Auto-Negotiation
                                            // Expansion
pub const EPHY_ANNPTR = usize(0x00000007);  // Ethernet PHY Auto-Negotiation
                                            // Next Page TX
pub const EPHY_ANLNPTR = usize(0x00000008);  // Ethernet PHY Auto-Negotiation
                                            // Link Partner Ability Next Page
pub const EPHY_CFG1 = usize(0x00000009);  // Ethernet PHY Configuration 1
pub const EPHY_CFG2 = usize(0x0000000A);  // Ethernet PHY Configuration 2
pub const EPHY_CFG3 = usize(0x0000000B);  // Ethernet PHY Configuration 3
pub const EPHY_REGCTL = usize(0x0000000D);  // Ethernet PHY Register Control
pub const EPHY_ADDAR = usize(0x0000000E);  // Ethernet PHY Address or Data
pub const EPHY_STS = usize(0x00000010);  // Ethernet PHY Status
pub const EPHY_SCR = usize(0x00000011);  // Ethernet PHY Specific Control
pub const EPHY_MISR1 = usize(0x00000012);  // Ethernet PHY MII Interrupt
                                            // Status 1
pub const EPHY_MISR2 = usize(0x00000013);  // Ethernet PHY MII Interrupt
                                            // Status 2
pub const EPHY_FCSCR = usize(0x00000014);  // Ethernet PHY False Carrier Sense
                                            // Counter
pub const EPHY_RXERCNT = usize(0x00000015);  // Ethernet PHY Receive Error Count
pub const EPHY_BISTCR = usize(0x00000016);  // Ethernet PHY BIST Control
pub const EPHY_LEDCR = usize(0x00000018);  // Ethernet PHY LED Control
pub const EPHY_CTL = usize(0x00000019);  // Ethernet PHY Control
pub const EPHY_10BTSC = usize(0x0000001A);  // Ethernet PHY 10Base-T
                                            // Status/Control - MR26
pub const EPHY_BICSR1 = usize(0x0000001B);  // Ethernet PHY BIST Control and
                                            // Status 1
pub const EPHY_BICSR2 = usize(0x0000001C);  // Ethernet PHY BIST Control and
                                            // Status 2
pub const EPHY_CDCR = usize(0x0000001E);  // Ethernet PHY Cable Diagnostic
                                            // Control
pub const EPHY_RCR = usize(0x0000001F);  // Ethernet PHY Reset Control
pub const EPHY_LEDCFG = usize(0x00000025);  // Ethernet PHY LED Configuration

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_BMCR register.
//
//*****************************************************************************
pub const EPHY_BMCR_MIIRESET = usize(0x00008000);  // MII Register reset
pub const EPHY_BMCR_MIILOOPBK = usize(0x00004000);  // MII Loopback
pub const EPHY_BMCR_SPEED = usize(0x00002000);  // Speed Select
pub const EPHY_BMCR_ANEN = usize(0x00001000);  // Auto-Negotiate Enable
pub const EPHY_BMCR_PWRDWN = usize(0x00000800);  // Power Down
pub const EPHY_BMCR_ISOLATE = usize(0x00000400);  // Port Isolate
pub const EPHY_BMCR_RESTARTAN = usize(0x00000200);  // Restart Auto-Negotiation
pub const EPHY_BMCR_DUPLEXM = usize(0x00000100);  // Duplex Mode
pub const EPHY_BMCR_COLLTST = usize(0x00000080);  // Collision Test

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_BMSR register.
//
//*****************************************************************************
pub const EPHY_BMSR_100BTXFD = usize(0x00004000);  // 100Base-TX Full Duplex Capable
pub const EPHY_BMSR_100BTXHD = usize(0x00002000);  // 100Base-TX Half Duplex Capable
pub const EPHY_BMSR_10BTFD = usize(0x00001000);  // 10 Base-T Full Duplex Capable
pub const EPHY_BMSR_10BTHD = usize(0x00000800);  // 10 Base-T Half Duplex Capable
pub const EPHY_BMSR_MFPRESUP = usize(0x00000040);  // Preamble Suppression Capable
pub const EPHY_BMSR_ANC = usize(0x00000020);  // Auto-Negotiation Complete
pub const EPHY_BMSR_RFAULT = usize(0x00000010);  // Remote Fault
pub const EPHY_BMSR_ANEN = usize(0x00000008);  // Auto Negotiation Enabled
pub const EPHY_BMSR_LINKSTAT = usize(0x00000004);  // Link Status
pub const EPHY_BMSR_JABBER = usize(0x00000002);  // Jabber Detect
pub const EPHY_BMSR_EXTEN = usize(0x00000001);  // Extended Capability Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_ID1 register.
//
//*****************************************************************************
pub const EPHY_ID1_OUIMSB_M = usize(0x0000FFFF);  // OUI Most Significant Bits
pub const EPHY_ID1_OUIMSB_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_ID2 register.
//
//*****************************************************************************
pub const EPHY_ID2_OUILSB_M = usize(0x0000FC00);  // OUI Least Significant Bits
pub const EPHY_ID2_VNDRMDL_M = usize(0x000003F0);  // Vendor Model Number
pub const EPHY_ID2_MDLREV_M = usize(0x0000000F);  // Model Revision Number
pub const EPHY_ID2_OUILSB_S = usize(10);
pub const EPHY_ID2_VNDRMDL_S = usize(4);
pub const EPHY_ID2_MDLREV_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_ANA register.
//
//*****************************************************************************
pub const EPHY_ANA_NP = usize(0x00008000);  // Next Page Indication
pub const EPHY_ANA_RF = usize(0x00002000);  // Remote Fault
pub const EPHY_ANA_ASMDUP = usize(0x00000800);  // Asymmetric PAUSE support for
                                            // Full Duplex Links
pub const EPHY_ANA_PAUSE = usize(0x00000400);  // PAUSE Support for Full Duplex
                                            // Links
pub const EPHY_ANA_100BT4 = usize(0x00000200);  // 100Base-T4 Support
pub const EPHY_ANA_100BTXFD = usize(0x00000100);  // 100Base-TX Full Duplex Support
pub const EPHY_ANA_100BTX = usize(0x00000080);  // 100Base-TX Support
pub const EPHY_ANA_10BTFD = usize(0x00000040);  // 10Base-T Full Duplex Support
pub const EPHY_ANA_10BT = usize(0x00000020);  // 10Base-T Support
pub const EPHY_ANA_SELECT_M = usize(0x0000001F);  // Protocol Selection
pub const EPHY_ANA_SELECT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_ANLPA register.
//
//*****************************************************************************
pub const EPHY_ANLPA_NP = usize(0x00008000);  // Next Page Indication
pub const EPHY_ANLPA_ACK = usize(0x00004000);  // Acknowledge
pub const EPHY_ANLPA_RF = usize(0x00002000);  // Remote Fault
pub const EPHY_ANLPA_ASMDUP = usize(0x00000800);  // Asymmetric PAUSE
pub const EPHY_ANLPA_PAUSE = usize(0x00000400);  // PAUSE
pub const EPHY_ANLPA_100BT4 = usize(0x00000200);  // 100Base-T4 Support
pub const EPHY_ANLPA_100BTXFD = usize(0x00000100);  // 100Base-TX Full Duplex Support
pub const EPHY_ANLPA_100BTX = usize(0x00000080);  // 100Base-TX Support
pub const EPHY_ANLPA_10BTFD = usize(0x00000040);  // 10Base-T Full Duplex Support
pub const EPHY_ANLPA_10BT = usize(0x00000020);  // 10Base-T Support
pub const EPHY_ANLPA_SELECT_M = usize(0x0000001F);  // Protocol Selection
pub const EPHY_ANLPA_SELECT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_ANER register.
//
//*****************************************************************************
pub const EPHY_ANER_PDF = usize(0x00000010);  // Parallel Detection Fault
pub const EPHY_ANER_LPNPABLE = usize(0x00000008);  // Link Partner Next Page Able
pub const EPHY_ANER_NPABLE = usize(0x00000004);  // Next Page Able
pub const EPHY_ANER_PAGERX = usize(0x00000002);  // Link Code Word Page Received
pub const EPHY_ANER_LPANABLE = usize(0x00000001);  // Link Partner Auto-Negotiation
                                            // Able

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_ANNPTR register.
//
//*****************************************************************************
pub const EPHY_ANNPTR_NP = usize(0x00008000);  // Next Page Indication
pub const EPHY_ANNPTR_MP = usize(0x00002000);  // Message Page
pub const EPHY_ANNPTR_ACK2 = usize(0x00001000);  // Acknowledge 2
pub const EPHY_ANNPTR_TOGTX = usize(0x00000800);  // Toggle
pub const EPHY_ANNPTR_CODE_M = usize(0x000007FF);  // Code
pub const EPHY_ANNPTR_CODE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_ANLNPTR register.
//
//*****************************************************************************
pub const EPHY_ANLNPTR_NP = usize(0x00008000);  // Next Page Indication
pub const EPHY_ANLNPTR_ACK = usize(0x00004000);  // Acknowledge
pub const EPHY_ANLNPTR_MP = usize(0x00002000);  // Message Page
pub const EPHY_ANLNPTR_ACK2 = usize(0x00001000);  // Acknowledge 2
pub const EPHY_ANLNPTR_TOG = usize(0x00000800);  // Toggle
pub const EPHY_ANLNPTR_CODE_M = usize(0x000007FF);  // Code
pub const EPHY_ANLNPTR_CODE_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_CFG1 register.
//
//*****************************************************************************
pub const EPHY_CFG1_DONE = usize(0x00008000);  // Configuration Done
pub const EPHY_CFG1_TDRAR = usize(0x00000100);  // TDR Auto-Run at Link Down
pub const EPHY_CFG1_LLR = usize(0x00000080);  // Link Loss Recovery
pub const EPHY_CFG1_FAMDIX = usize(0x00000040);  // Fast Auto MDI/MDIX
pub const EPHY_CFG1_RAMDIX = usize(0x00000020);  // Robust Auto MDI/MDIX
pub const EPHY_CFG1_FASTANEN = usize(0x00000010);  // Fast Auto Negotiation Enable
pub const EPHY_CFG1_FANSEL_M = usize(0x0000000C);  // Fast Auto-Negotiation Select
                                            // Configuration
pub const EPHY_CFG1_FANSEL_BLT80 = usize(0x00000000);  // Break Link Timer: 80 ms
pub const EPHY_CFG1_FANSEL_BLT120 = usize(0x00000004);  // Break Link Timer: 120 ms
pub const EPHY_CFG1_FANSEL_BLT240 = usize(0x00000008);  // Break Link Timer: 240 ms
pub const EPHY_CFG1_FRXDVDET = usize(0x00000002);  // FAST RXDV Detection

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_CFG2 register.
//
//*****************************************************************************
pub const EPHY_CFG2_FLUPPD = usize(0x00000040);  // Fast Link-Up in Parallel Detect
                                            // Mode
pub const EPHY_CFG2_EXTFD = usize(0x00000020);  // Extended Full-Duplex Ability
pub const EPHY_CFG2_ENLEDLINK = usize(0x00000010);  // Enhanced LED Functionality
pub const EPHY_CFG2_ISOMIILL = usize(0x00000008);  // Isolate MII outputs when
                                            // Enhanced Link is not Achievable
pub const EPHY_CFG2_RXERRIDLE = usize(0x00000004);  // Detection of Receive Symbol
                                            // Error During IDLE State
pub const EPHY_CFG2_ODDNDETDIS = usize(0x00000002);  // Detection of Transmit Error

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_CFG3 register.
//
//*****************************************************************************
pub const EPHY_CFG3_POLSWAP = usize(0x00000080);  // Polarity Swap
pub const EPHY_CFG3_MDIMDIXS = usize(0x00000040);  // MDI/MDIX Swap
pub const EPHY_CFG3_FLDWNM_M = usize(0x0000001F);  // Fast Link Down Modes
pub const EPHY_CFG3_FLDWNM_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_REGCTL register.
//
//*****************************************************************************
pub const EPHY_REGCTL_FUNC_M = usize(0x0000C000);  // Function
pub const EPHY_REGCTL_FUNC_ADDR = usize(0x00000000);  // Address
pub const EPHY_REGCTL_FUNC_DATANI = usize(0x00004000);  // Data, no post increment
pub const EPHY_REGCTL_FUNC_DATAPIRW = usize(0x00008000);  // Data, post increment on read and
                                            // write
pub const EPHY_REGCTL_FUNC_DATAPIWO = usize(0x0000C000);  // Data, post increment on write
                                            // only
pub const EPHY_REGCTL_DEVAD_M = usize(0x0000001F);  // Device Address
pub const EPHY_REGCTL_DEVAD_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_ADDAR register.
//
//*****************************************************************************
pub const EPHY_ADDAR_ADDRDATA_M = usize(0x0000FFFF);  // Address or Data
pub const EPHY_ADDAR_ADDRDATA_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_STS register.
//
//*****************************************************************************
pub const EPHY_STS_MDIXM = usize(0x00004000);  // MDI-X Mode
pub const EPHY_STS_RXLERR = usize(0x00002000);  // Receive Error Latch
pub const EPHY_STS_POLSTAT = usize(0x00001000);  // Polarity Status
pub const EPHY_STS_FCSL = usize(0x00000800);  // False Carrier Sense Latch
pub const EPHY_STS_SD = usize(0x00000400);  // Signal Detect
pub const EPHY_STS_DL = usize(0x00000200);  // Descrambler Lock
pub const EPHY_STS_PAGERX = usize(0x00000100);  // Link Code Page Received
pub const EPHY_STS_MIIREQ = usize(0x00000080);  // MII Interrupt Pending
pub const EPHY_STS_RF = usize(0x00000040);  // Remote Fault
pub const EPHY_STS_JD = usize(0x00000020);  // Jabber Detect
pub const EPHY_STS_ANS = usize(0x00000010);  // Auto-Negotiation Status
pub const EPHY_STS_MIILB = usize(0x00000008);  // MII Loopback Status
pub const EPHY_STS_DUPLEX = usize(0x00000004);  // Duplex Status
pub const EPHY_STS_SPEED = usize(0x00000002);  // Speed Status
pub const EPHY_STS_LINK = usize(0x00000001);  // Link Status

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_SCR register.
//
//*****************************************************************************
pub const EPHY_SCR_DISCLK = usize(0x00008000);  // Disable CLK
pub const EPHY_SCR_PSEN = usize(0x00004000);  // Power Saving Modes Enable
pub const EPHY_SCR_PSMODE_M = usize(0x00003000);  // Power Saving Modes
pub const EPHY_SCR_PSMODE_NORMAL = usize(0x00000000);  // Normal: Normal operation mode.
                                            // PHY is fully functional
pub const EPHY_SCR_PSMODE_LOWPWR = usize(0x00001000);  // IEEE Power Down
pub const EPHY_SCR_PSMODE_ACTWOL = usize(0x00002000);  // Active Sleep
pub const EPHY_SCR_PSMODE_PASWOL = usize(0x00003000);  // Passive Sleep
pub const EPHY_SCR_SBPYASS = usize(0x00000800);  // Scrambler Bypass
pub const EPHY_SCR_LBFIFO_M = usize(0x00000300);  // Loopback FIFO Depth
pub const EPHY_SCR_LBFIFO_4 = usize(0x00000000);  // Four nibble FIFO
pub const EPHY_SCR_LBFIFO_5 = usize(0x00000100);  // Five nibble FIFO
pub const EPHY_SCR_LBFIFO_6 = usize(0x00000200);  // Six nibble FIFO
pub const EPHY_SCR_LBFIFO_8 = usize(0x00000300);  // Eight nibble FIFO
pub const EPHY_SCR_COLFDM = usize(0x00000010);  // Collision in Full-Duplex Mode
pub const EPHY_SCR_TINT = usize(0x00000004);  // Test Interrupt
pub const EPHY_SCR_INTEN = usize(0x00000002);  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_MISR1 register.
//
//*****************************************************************************
pub const EPHY_MISR1_LINKSTAT = usize(0x00002000);  // Change of Link Status Interrupt
pub const EPHY_MISR1_SPEED = usize(0x00001000);  // Change of Speed Status Interrupt
pub const EPHY_MISR1_DUPLEXM = usize(0x00000800);  // Change of Duplex Status
                                            // Interrupt
pub const EPHY_MISR1_ANC = usize(0x00000400);  // Auto-Negotiation Complete
                                            // Interrupt
pub const EPHY_MISR1_FCHF = usize(0x00000200);  // False Carrier Counter Half-Full
                                            // Interrupt
pub const EPHY_MISR1_RXHF = usize(0x00000100);  // Receive Error Counter Half-Full
                                            // Interrupt
pub const EPHY_MISR1_LINKSTATEN = usize(0x00000020);  // Link Status Interrupt Enable
pub const EPHY_MISR1_SPEEDEN = usize(0x00000010);  // Speed Change Interrupt Enable
pub const EPHY_MISR1_DUPLEXMEN = usize(0x00000008);  // Duplex Status Interrupt Enable
pub const EPHY_MISR1_ANCEN = usize(0x00000004);  // Auto-Negotiation Complete
                                            // Interrupt Enable
pub const EPHY_MISR1_FCHFEN = usize(0x00000002);  // False Carrier Counter Register
                                            // half-full Interrupt Enable
pub const EPHY_MISR1_RXHFEN = usize(0x00000001);  // Receive Error Counter Register
                                            // Half-Full Event Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_MISR2 register.
//
//*****************************************************************************
pub const EPHY_MISR2_ANERR = usize(0x00004000);  // Auto-Negotiation Error Interrupt
pub const EPHY_MISR2_PAGERX = usize(0x00002000);  // Page Receive Interrupt
pub const EPHY_MISR2_LBFIFO = usize(0x00001000);  // Loopback FIFO Overflow/Underflow
                                            // Event Interrupt
pub const EPHY_MISR2_MDICO = usize(0x00000800);  // MDI/MDIX Crossover Status
                                            // Changed Interrupt
pub const EPHY_MISR2_SLEEP = usize(0x00000400);  // Sleep Mode Event Interrupt
pub const EPHY_MISR2_POLINT = usize(0x00000200);  // Polarity Changed Interrupt
pub const EPHY_MISR2_JABBER = usize(0x00000100);  // Jabber Detect Event Interrupt
pub const EPHY_MISR2_ANERREN = usize(0x00000040);  // Auto-Negotiation Error Interrupt
                                            // Enable
pub const EPHY_MISR2_PAGERXEN = usize(0x00000020);  // Page Receive Interrupt Enable
pub const EPHY_MISR2_LBFIFOEN = usize(0x00000010);  // Loopback FIFO Overflow/Underflow
                                            // Interrupt Enable
pub const EPHY_MISR2_MDICOEN = usize(0x00000008);  // MDI/MDIX Crossover Status
                                            // Changed Interrupt Enable
pub const EPHY_MISR2_SLEEPEN = usize(0x00000004);  // Sleep Mode Event Interrupt
                                            // Enable
pub const EPHY_MISR2_POLINTEN = usize(0x00000002);  // Polarity Changed Interrupt
                                            // Enable
pub const EPHY_MISR2_JABBEREN = usize(0x00000001);  // Jabber Detect Event Interrupt
                                            // Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_FCSCR register.
//
//*****************************************************************************
pub const EPHY_FCSCR_FCSCNT_M = usize(0x000000FF);  // False Carrier Event Counter
pub const EPHY_FCSCR_FCSCNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_RXERCNT register.
//
//*****************************************************************************
pub const EPHY_RXERCNT_RXERRCNT_M = usize(0x0000FFFF);  // Receive Error Count
pub const EPHY_RXERCNT_RXERRCNT_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_BISTCR register.
//
//*****************************************************************************
pub const EPHY_BISTCR_PRBSM = usize(0x00004000);  // PRBS Single/Continuous Mode
pub const EPHY_BISTCR_PRBSPKT = usize(0x00002000);  // Generated PRBS Packets
pub const EPHY_BISTCR_PKTEN = usize(0x00001000);  // Packet Generation Enable
pub const EPHY_BISTCR_PRBSCHKLK = usize(0x00000800);  // PRBS Checker Lock Indication
pub const EPHY_BISTCR_PRBSCHKSYNC = usize(0x00000400);  // PRBS Checker Lock Sync Loss
                                            // Indication
pub const EPHY_BISTCR_PKTGENSTAT = usize(0x00000200);  // Packet Generator Status
                                            // Indication
pub const EPHY_BISTCR_PWRMODE = usize(0x00000100);  // Power Mode Indication
pub const EPHY_BISTCR_TXMIILB = usize(0x00000040);  // Transmit Data in MII Loopback
                                            // Mode
pub const EPHY_BISTCR_LBMODE_M = usize(0x0000001F);  // Loopback Mode Select
pub const EPHY_BISTCR_LBMODE_NPCSIN = usize(0x00000001);  // Near-end loopback: PCS Input
                                            // Loopback
pub const EPHY_BISTCR_LBMODE_NPCSOUT = usize(0x00000002);  // Near-end loopback: PCS Output
                                            // Loopback (In 100Base-TX only)
pub const EPHY_BISTCR_LBMODE_NDIG = usize(0x00000004);  // Near-end loopback: Digital
                                            // Loopback
pub const EPHY_BISTCR_LBMODE_NANA = usize(0x00000008);  // Near-end loopback: Analog
                                            // Loopback (requires 100 Ohm
                                            // termination)
pub const EPHY_BISTCR_LBMODE_FREV = usize(0x00000010);  // Far-end Loopback: Reverse
                                            // Loopback

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_LEDCR register.
//
//*****************************************************************************
pub const EPHY_LEDCR_BLINKRATE_M = usize(0x00000600);  // LED Blinking Rate (ON/OFF
                                            // duration):
pub const EPHY_LEDCR_BLINKRATE_20HZ = usize(0x00000000);  // 20 Hz (50 ms)
pub const EPHY_LEDCR_BLINKRATE_10HZ = usize(0x00000200);  // 10 Hz (100 ms)
pub const EPHY_LEDCR_BLINKRATE_5HZ = usize(0x00000400);  // 5 Hz (200 ms)
pub const EPHY_LEDCR_BLINKRATE_2HZ = usize(0x00000600);  // 2 Hz (500 ms)

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_CTL register.
//
//*****************************************************************************
pub const EPHY_CTL_AUTOMDI = usize(0x00008000);  // Auto-MDIX Enable
pub const EPHY_CTL_FORCEMDI = usize(0x00004000);  // Force MDIX
pub const EPHY_CTL_PAUSERX = usize(0x00002000);  // Pause Receive Negotiated Status
pub const EPHY_CTL_PAUSETX = usize(0x00001000);  // Pause Transmit Negotiated Status
pub const EPHY_CTL_MIILNKSTAT = usize(0x00000800);  // MII Link Status
pub const EPHY_CTL_BYPLEDSTRCH = usize(0x00000080);  // Bypass LED Stretching

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_10BTSC register.
//
//*****************************************************************************
pub const EPHY_10BTSC_RXTHEN = usize(0x00002000);  // Lower Receiver Threshold Enable
pub const EPHY_10BTSC_SQUELCH_M = usize(0x00001E00);  // Squelch Configuration
pub const EPHY_10BTSC_NLPDIS = usize(0x00000080);  // Normal Link Pulse (NLP)
                                            // Transmission Control
pub const EPHY_10BTSC_POLSTAT = usize(0x00000010);  // 10 Mb Polarity Status
pub const EPHY_10BTSC_JABBERD = usize(0x00000001);  // Jabber Disable
pub const EPHY_10BTSC_SQUELCH_S = usize(9);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_BICSR1 register.
//
//*****************************************************************************
pub const EPHY_BICSR1_ERRCNT_M = usize(0x0000FF00);  // BIST Error Count
pub const EPHY_BICSR1_IPGLENGTH_M = usize(0x000000FF);  // BIST IPG Length
pub const EPHY_BICSR1_ERRCNT_S = usize(8);
pub const EPHY_BICSR1_IPGLENGTH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_BICSR2 register.
//
//*****************************************************************************
pub const EPHY_BICSR2_PKTLENGTH_M = usize(0x000007FF);  // BIST Packet Length
pub const EPHY_BICSR2_PKTLENGTH_S = usize(0);

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_CDCR register.
//
//*****************************************************************************
pub const EPHY_CDCR_START = usize(0x00008000);  // Cable Diagnostic Process Start
pub const EPHY_CDCR_LINKQUAL_M = usize(0x00000300);  // Link Quality Indication
pub const EPHY_CDCR_LINKQUAL_GOOD = usize(0x00000100);  // Good Quality Link Indication
pub const EPHY_CDCR_LINKQUAL_MILD = usize(0x00000200);  // Mid- Quality Link Indication
pub const EPHY_CDCR_LINKQUAL_POOR = usize(0x00000300);  // Poor Quality Link Indication
pub const EPHY_CDCR_DONE = usize(0x00000002);  // Cable Diagnostic Process Done
pub const EPHY_CDCR_FAIL = usize(0x00000001);  // Cable Diagnostic Process Fail

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_RCR register.
//
//*****************************************************************************
pub const EPHY_RCR_SWRST = usize(0x00008000);  // Software Reset
pub const EPHY_RCR_SWRESTART = usize(0x00004000);  // Software Restart

//*****************************************************************************
//
// The following are defines for the bit fields in the EPHY_LEDCFG register.
//
//*****************************************************************************
pub const EPHY_LEDCFG_LED2_M = usize(0x00000F00);  // LED2 Configuration
pub const EPHY_LEDCFG_LED2_LINK = usize(0x00000000);  // Link OK
pub const EPHY_LEDCFG_LED2_RXTX = usize(0x00000100);  // RX/TX Activity
pub const EPHY_LEDCFG_LED2_TX = usize(0x00000200);  // TX Activity
pub const EPHY_LEDCFG_LED2_RX = usize(0x00000300);  // RX Activity
pub const EPHY_LEDCFG_LED2_COL = usize(0x00000400);  // Collision
pub const EPHY_LEDCFG_LED2_100BT = usize(0x00000500);  // 100-Base TX
pub const EPHY_LEDCFG_LED2_10BT = usize(0x00000600);  // 10-Base TX
pub const EPHY_LEDCFG_LED2_FD = usize(0x00000700);  // Full Duplex
pub const EPHY_LEDCFG_LED2_LINKTXRX = usize(0x00000800);  // Link OK/Blink on TX/RX Activity
pub const EPHY_LEDCFG_LED1_M = usize(0x000000F0);  // LED1 Configuration
pub const EPHY_LEDCFG_LED1_LINK = usize(0x00000000);  // Link OK
pub const EPHY_LEDCFG_LED1_RXTX = usize(0x00000010);  // RX/TX Activity
pub const EPHY_LEDCFG_LED1_TX = usize(0x00000020);  // TX Activity
pub const EPHY_LEDCFG_LED1_RX = usize(0x00000030);  // RX Activity
pub const EPHY_LEDCFG_LED1_COL = usize(0x00000040);  // Collision
pub const EPHY_LEDCFG_LED1_100BT = usize(0x00000050);  // 100-Base TX
pub const EPHY_LEDCFG_LED1_10BT = usize(0x00000060);  // 10-Base TX
pub const EPHY_LEDCFG_LED1_FD = usize(0x00000070);  // Full Duplex
pub const EPHY_LEDCFG_LED1_LINKTXRX = usize(0x00000080);  // Link OK/Blink on TX/RX Activity
pub const EPHY_LEDCFG_LED0_M = usize(0x0000000F);  // LED0 Configuration
pub const EPHY_LEDCFG_LED0_LINK = usize(0x00000000);  // Link OK
pub const EPHY_LEDCFG_LED0_RXTX = usize(0x00000001);  // RX/TX Activity
pub const EPHY_LEDCFG_LED0_TX = usize(0x00000002);  // TX Activity
pub const EPHY_LEDCFG_LED0_RX = usize(0x00000003);  // RX Activity
pub const EPHY_LEDCFG_LED0_COL = usize(0x00000004);  // Collision
pub const EPHY_LEDCFG_LED0_100BT = usize(0x00000005);  // 100-Base TX
pub const EPHY_LEDCFG_LED0_10BT = usize(0x00000006);  // 10-Base TX
pub const EPHY_LEDCFG_LED0_FD = usize(0x00000007);  // Full Duplex
pub const EPHY_LEDCFG_LED0_LINKTXRX = usize(0x00000008);  // Link OK/Blink on TX/RX Activity

//*****************************************************************************
//
// The following definitions are deprecated.
//
//*****************************************************************************

