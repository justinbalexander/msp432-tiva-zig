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

pub const EMAC = struct {
    //*****************************************************************************
    //
    // The following are defines for the EMAC register offsets.
    //
    //*****************************************************************************
    pub const offsetOf = struct {
        pub const CFG = usize(0x00000000); // Ethernet MAC Configuration
        pub const FRAMEFLTR = usize(0x00000004); // Ethernet MAC Frame Filter
        pub const HASHTBLH = usize(0x00000008); // Ethernet MAC Hash Table High
        pub const HASHTBLL = usize(0x0000000C); // Ethernet MAC Hash Table Low
        pub const MIIADDR = usize(0x00000010); // Ethernet MAC MII Address
        pub const MIIDATA = usize(0x00000014); // Ethernet MAC MII Data Register
        pub const FLOWCTL = usize(0x00000018); // Ethernet MAC Flow Control
        pub const VLANTG = usize(0x0000001C); // Ethernet MAC VLAN Tag
        pub const STATUS = usize(0x00000024); // Ethernet MAC Status
        pub const RWUFF = usize(0x00000028); // Ethernet MAC Remote Wake-Up
        // Frame F
        pub const PMTCTLSTAT = usize(0x0000002C); // Ethernet MAC PMT Control and
        // Status r
        pub const LPICTLSTAT = usize(0x00000030); // Ethernet MAC Low Power Idle
        // Controlatus Register
        pub const LPITIMERCTL = usize(0x00000034); // Ethernet MAC Low Power Idle
        // Timer CRegister
        pub const RIS = usize(0x00000038); // Ethernet MAC Raw Interrupt
        // Status
        pub const IM = usize(0x0000003C); // Ethernet MAC Interrupt Mask
        pub const ADDR0H = usize(0x00000040); // Ethernet MAC Address 0 High
        pub const ADDR0L = usize(0x00000044); // Ethernet MAC Address 0 Low
        // Registe
        pub const ADDR1H = usize(0x00000048); // Ethernet MAC Address 1 High
        pub const ADDR1L = usize(0x0000004C); // Ethernet MAC Address 1 Low
        pub const ADDR2H = usize(0x00000050); // Ethernet MAC Address 2 High
        pub const ADDR2L = usize(0x00000054); // Ethernet MAC Address 2 Low
        pub const ADDR3H = usize(0x00000058); // Ethernet MAC Address 3 High
        pub const ADDR3L = usize(0x0000005C); // Ethernet MAC Address 3 Low
        pub const WDOGTO = usize(0x000000DC); // Ethernet MAC Watchdog Timeout
        pub const MMCCTRL = usize(0x00000100); // Ethernet MAC MMC Control
        pub const MMCRXRIS = usize(0x00000104); // Ethernet MAC MMC Receive Raw
        // Interruus
        pub const MMCTXRIS = usize(0x00000108); // Ethernet MAC MMC Transmit Raw
        // Interruus
        pub const MMCRXIM = usize(0x0000010C); // Ethernet MAC MMC Receive
        // Interru
        pub const MMCTXIM = usize(0x00000110); // Ethernet MAC MMC Transmit
        // Interru
        pub const TXCNTGB = usize(0x00000118); // Ethernet MAC Transmit Frame
        // Count f and Bad Frames
        pub const TXCNTSCOL = usize(0x0000014C); // Ethernet MAC Transmit Frame
        // Count fes Transmitted
        // after Sollision
        pub const TXCNTMCOL = usize(0x00000150); // Ethernet MAC Transmit Frame
        // Count fes Transmitted
        // after M Collisions
        pub const TXOCTCNTG = usize(0x00000164); // Ethernet MAC Transmit Octet
        // Count G
        pub const RXCNTGB = usize(0x00000180); // Ethernet MAC Receive Frame Count
        // for Gooad Frames
        pub const RXCNTCRCERR = usize(0x00000194); // Ethernet MAC Receive Frame Count
        // for CRCFrames
        pub const RXCNTALGNERR = usize(0x00000198); // Ethernet MAC Receive Frame Count
        // for AliError Frames
        pub const RXCNTGUNI = usize(0x000001C4); // Ethernet MAC Receive Frame Count
        // for Goost Frames
        pub const VLNINCREP = usize(0x00000584); // Ethernet MAC VLAN Tag Inclusion
        // or Repl
        pub const VLANHASH = usize(0x00000588); // Ethernet MAC VLAN Hash Table
        pub const TIMSTCTRL = usize(0x00000700); // Ethernet MAC Timestamp Control
        pub const SUBSECINC = usize(0x00000704); // Ethernet MAC Sub-Second
        // Increme
        pub const TIMSEC = usize(0x00000708); // Ethernet MAC System Time -
        // Seconds
        pub const TIMNANO = usize(0x0000070C); // Ethernet MAC System Time -
        // Nanosec
        pub const TIMSECU = usize(0x00000710); // Ethernet MAC System Time -
        // Seconds
        pub const TIMNANOU = usize(0x00000714); // Ethernet MAC System Time -
        // Nanosecdate
        pub const TIMADD = usize(0x00000718); // Ethernet MAC Timestamp Addend
        pub const TARGSEC = usize(0x0000071C); // Ethernet MAC Target Time Seconds
        pub const TARGNANO = usize(0x00000720); // Ethernet MAC Target Time
        // Nanosec
        pub const HWORDSEC = usize(0x00000724); // Ethernet MAC System Time-Higher
        // Word Se
        pub const TIMSTAT = usize(0x00000728); // Ethernet MAC Timestamp Status
        pub const PPSCTRL = usize(0x0000072C); // Ethernet MAC PPS Control
        pub const PPS0INTVL = usize(0x00000760); // Ethernet MAC PPS0 Interval
        pub const PPS0WIDTH = usize(0x00000764); // Ethernet MAC PPS0 Width
        pub const DMABUSMOD = usize(0x00000C00); // Ethernet MAC DMA Bus Mode
        pub const TXPOLLD = usize(0x00000C04); // Ethernet MAC Transmit Poll
        // Demand
        pub const RXPOLLD = usize(0x00000C08); // Ethernet MAC Receive Poll Demand
        pub const RXDLADDR = usize(0x00000C0C); // Ethernet MAC Receive Descriptor
        // List Ad
        pub const TXDLADDR = usize(0x00000C10); // Ethernet MAC Transmit Descriptor
        // List Ad
        pub const DMARIS = usize(0x00000C14); // Ethernet MAC DMA Interrupt
        // Status
        pub const DMAOPMODE = usize(0x00000C18); // Ethernet MAC DMA Operation Mode
        pub const DMAIM = usize(0x00000C1C); // Ethernet MAC DMA Interrupt Mask
        // Registe
        pub const MFBOC = usize(0x00000C20); // Ethernet MAC Missed Frame and
        // Buffer w Counter
        pub const RXINTWDT = usize(0x00000C24); // Ethernet MAC Receive Interrupt
        // Watchdo
        pub const HOSTXDESC = usize(0x00000C48); // Ethernet MAC Current Host
        // Transmiiptor
        pub const HOSRXDESC = usize(0x00000C4C); // Ethernet MAC Current Host
        // Receiveptor
        pub const HOSTXBA = usize(0x00000C50); // Ethernet MAC Current Host
        // Transmir Address
        pub const HOSRXBA = usize(0x00000C54); // Ethernet MAC Current Host
        // Receive Address
        pub const PP = usize(0x00000FC0); // Ethernet MAC Peripheral Property
        // Registe
        pub const PC = usize(0x00000FC4); // Ethernet MAC Peripheral
        // ConfiguRegister
        pub const CC = usize(0x00000FC8); // Ethernet MAC Clock Configuration
        // Registe
        pub const EPHYRIS = usize(0x00000FD0); // Ethernet PHY Raw Interrupt
        // Status
        pub const EPHYIM = usize(0x00000FD4); // Ethernet PHY Interrupt Mask
        pub const EPHYMISC = usize(0x00000FD8); // Ethernet PHY Masked Interrupt
    };
    // Status and Clear

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_CFG register.
    //
    //*****************************************************************************
    pub const CFG = struct {
        pub const TWOKPEN = usize(0x08000000); // IEEE 802
        pub const CST = usize(0x02000000); // CRC Stripping for Type Frames
        pub const WDDIS = usize(0x00800000); // Watchdog Disable
        pub const JD = usize(0x00400000); // Jabber Disable
        pub const JFEN = usize(0x00100000); // Jumbo Frame Enable
        pub const IFG_M = usize(0x000E0000); // Inter-Frame Gap (IFG)
        pub const IFG_96 = usize(0x00000000); // 96 bit times
        pub const IFG_88 = usize(0x00020000); // 88 bit times
        pub const IFG_80 = usize(0x00040000); // 80 bit times
        pub const IFG_72 = usize(0x00060000); // 72 bit times
        pub const IFG_64 = usize(0x00080000); // 64 bit times
        pub const IFG_56 = usize(0x000A0000); // 56 bit times
        pub const IFG_48 = usize(0x000C0000); // 48 bit times
        pub const IFG_40 = usize(0x000E0000); // 40 bit times
        pub const DISCRS = usize(0x00010000); // Disable Carrier Sense During
        // Transmission
        pub const PS = usize(0x00008000); // Port Select
        pub const FES = usize(0x00004000); // Speed
        pub const DRO = usize(0x00002000); // Disable Receive Own
        pub const LOOPBM = usize(0x00001000); // Loopback Mode
        pub const DUPM = usize(0x00000800); // Duplex Mode
        pub const IPC = usize(0x00000400); // Checksum Offload
        pub const DR = usize(0x00000200); // Disable Retry
        pub const ACS = usize(0x00000080); // Automatic Pad or CRC Stripping
        pub const BL_M = usize(0x00000060); // Back-Off Limit
        pub const BL_1024 = usize(0x00000000); // k = min (n,10)
        pub const BL_256 = usize(0x00000020); // k = min (n,8)
        pub const BL_8 = usize(0x00000040); // k = min (n,4)
        pub const BL_2 = usize(0x00000060); // k = min (n,1)
        pub const DC = usize(0x00000010); // Deferral Check
        pub const TE = usize(0x00000008); // Transmitter Enable
        pub const RE = usize(0x00000004); // Receiver Enable
        pub const PRELEN_M = usize(0x00000003); // Preamble Length for Transmit
        // Frames
        pub const PRELEN_7 = usize(0x00000000); // 7 bytes of preamble
        pub const PRELEN_5 = usize(0x00000001); // 5 bytes of preamble
        pub const PRELEN_3 = usize(0x00000002); // 3 bytes of preamble
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_FRAMEFLTR
    // register.
    //
    //*****************************************************************************
    pub const FRAMEFLTR = struct {
        pub const RA = usize(0x80000000); // Receive All
        pub const VTFE = usize(0x00010000); // VLAN Tag Filter Enable
        pub const HPF = usize(0x00000400); // Hash or Perfect Filter
        pub const SAF = usize(0x00000200); // Source Address Filter Enable
        pub const SAIF = usize(0x00000100); // Source Address (SA) Inverse
        // Filtering
        pub const PCF_M = usize(0x000000C0); // Pass Control Frames
        pub const PCF_ALL = usize(0x00000000); // The MAC filters all control
        // frames from reaching application
        pub const PCF_PAUSE = usize(0x00000040); // MAC forwards all control frames
        // except PAUSE control frames to
        // application even if they fail
        // the address filter
        pub const PCF_NONE = usize(0x00000080); // MAC forwards all control frames
        // to application even if they fail
        // the address Filter
        pub const PCF_ADDR = usize(0x000000C0); // MAC forwards control frames that
        // pass the address Filter
        pub const DBF = usize(0x00000020); // Disable Broadcast Frames
        pub const PM = usize(0x00000010); // Pass All Multicast
        pub const DAIF = usize(0x00000008); // Destination Address (DA) Inverse
        // Filtering
        pub const HMC = usize(0x00000004); // Hash Multicast
        pub const HUC = usize(0x00000002); // Hash Unicast
        pub const PR = usize(0x00000001); // Promiscuous Mode
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_HASHTBLH
    // register.
    //
    //*****************************************************************************
    pub const HASHTBLH = struct {
        pub const HTH_M = usize(0xFFFFFFFF); // Hash Table High
        pub const HTH_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_HASHTBLL
    // register.
    //
    //*****************************************************************************
    pub const HASHTBLL = struct {
        pub const HTL_M = usize(0xFFFFFFFF); // Hash Table Low
        pub const HTL_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_MIIADDR register.
    //
    //*****************************************************************************
    pub const MIIADDR = struct {
        pub const PLA_M = usize(0x0000F800); // Physical Layer Address
        pub const MII_M = usize(0x000007C0); // MII Register
        pub const CR_M = usize(0x0000003C); // Clock Reference Frequency
        // Selection
        pub const CR_60_100 = usize(0x00000000); // The frequency of the System
        // Clock is 60 to 100 MHz providing
        // a MDIO clock of SYSCLK/42
        pub const CR_100_150 = usize(0x00000004); // The frequency of the System
        // Clock is 100 to 150 MHz
        // providing a MDIO clock of
        // SYSCLK/62
        pub const CR_20_35 = usize(0x00000008); // The frequency of the System
        // Clock is 20-35 MHz providing a
        // MDIO clock of System Clock/16
        pub const CR_35_60 = usize(0x0000000C); // The frequency of the System
        // Clock is 35 to 60 MHz providing
        // a MDIO clock of System Clock/26
        pub const MIIW = usize(0x00000002); // MII Write
        pub const MIIB = usize(0x00000001); // MII Busy
        pub const PLA_S = usize(11);
        pub const MII_S = usize(6);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_MIIDATA register.
    //
    //*****************************************************************************
    pub const MIIDATA = struct {
        pub const DATA_M = usize(0x0000FFFF); // MII Data
        pub const DATA_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_FLOWCTL register.
    //
    //*****************************************************************************
    pub const FLOWCTL = struct {
        pub const PT_M = usize(0xFFFF0000); // Pause Time
        pub const DZQP = usize(0x00000080); // Disable Zero-Quanta Pause
        pub const UP = usize(0x00000008); // Unicast Pause Frame Detect
        pub const RFE = usize(0x00000004); // Receive Flow Control Enable
        pub const TFE = usize(0x00000002); // Transmit Flow Control Enable
        pub const FCBBPA = usize(0x00000001); // Flow Control Busy or
        // Back-pressure Activate
        pub const PT_S = usize(16);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_VLANTG register.
    //
    //*****************************************************************************
    pub const VLANTG = struct {
        pub const VTHM = usize(0x00080000); // VLAN Tag Hash Table Match Enable
        pub const ESVL = usize(0x00040000); // Enable S-VLAN
        pub const VTIM = usize(0x00020000); // VLAN Tag Inverse Match Enable
        pub const ETV = usize(0x00010000); // Enable 12-Bit VLAN Tag
        // Comparison
        pub const VL_M = usize(0x0000FFFF); // VLAN Tag Identifier for Receive
        // Frames
        pub const VL_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_STATUS register.
    //
    //*****************************************************************************
    pub const STATUS = struct {
        pub const TXFF = usize(0x02000000); // TX/RX Controller TX FIFO Full
        // Status
        pub const TXFE = usize(0x01000000); // TX/RX Controller TX FIFO Not
        // Empty Status
        pub const TWC = usize(0x00400000); // TX/RX Controller TX FIFO Write
        // Controller Active Status
        pub const TRC_M = usize(0x00300000); // TX/RX Controller's TX FIFO Read
        // Controller Status
        pub const TRC_IDLE = usize(0x00000000); // IDLE state
        pub const TRC_READ = usize(0x00100000); // READ state (transferring data to
        // MAC transmitter)
        pub const TRC_WAIT = usize(0x00200000); // Waiting for TX Status from MAC
        // transmitter
        pub const TRC_WRFLUSH = usize(0x00300000); // Writing the received TX Status
        // or flushing the TX FIFO
        pub const TXPAUSED = usize(0x00080000); // MAC Transmitter PAUSE
        pub const TFC_M = usize(0x00060000); // MAC Transmit Frame Controller
        // Status
        pub const TFC_IDLE = usize(0x00000000); // IDLE state
        pub const TFC_STATUS = usize(0x00020000); // Waiting for status of previous
        // frame or IFG or backoff period
        // to be over
        pub const TFC_PAUSE = usize(0x00040000); // Generating and transmitting a
        // PAUSE control frame (in the
        // full-duplex mode)
        pub const TFC_INPUT = usize(0x00060000); // Transferring input frame for
        // transmission
        pub const TPE = usize(0x00010000); // MAC MII Transmit Protocol Engine
        // Status
        pub const RXF_M = usize(0x00000300); // TX/RX Controller RX FIFO
        // Fill-level Status
        pub const RXF_EMPTY = usize(0x00000000); // RX FIFO Empty
        pub const RXF_BELOW = usize(0x00000100); // RX FIFO fill level is below the
        // flow-control deactivate
        // threshold
        pub const RXF_ABOVE = usize(0x00000200); // RX FIFO fill level is above the
        // flow-control activate threshold
        pub const RXF_FULL = usize(0x00000300); // RX FIFO Full
        pub const RRC_M = usize(0x00000060); // TX/RX Controller Read Controller
        // State
        pub const RRC_IDLE = usize(0x00000000); // IDLE state
        pub const RRC_STATUS = usize(0x00000020); // Reading frame data
        pub const RRC_DATA = usize(0x00000040); // Reading frame status (or
        // timestamp)
        pub const RRC_FLUSH = usize(0x00000060); // Flushing the frame data and
        // status
        pub const RWC = usize(0x00000010); // TX/RX Controller RX FIFO Write
        // Controller Active Status
        pub const RFCFC_M = usize(0x00000006); // MAC Receive Frame Controller
        // FIFO Status
        pub const RPE = usize(0x00000001); // MAC MII Receive Protocol Engine
        // Status
        pub const RFCFC_S = usize(1);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_RWUFF register.
    //
    //*****************************************************************************
    pub const RWUFF = struct {
        pub const WAKEUPFIL_M = usize(0xFFFFFFFF); // Remote Wake-Up Frame Filter
        pub const WAKEUPFIL_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_PMTCTLSTAT
    // register.
    //
    //*****************************************************************************
    pub const PMTCTLSTAT = struct {
        pub const WUPFRRST = usize(0x80000000); // Wake-Up Frame Filter Register
        // Pointer Reset
        pub const RWKPTR_M = usize(0x07000000); // Remote Wake-Up FIFO Pointer
        pub const GLBLUCAST = usize(0x00000200); // Global Unicast
        pub const WUPRX = usize(0x00000040); // Wake-Up Frame Received
        pub const MGKPRX = usize(0x00000020); // Magic Packet Received
        pub const WUPFREN = usize(0x00000004); // Wake-Up Frame Enable
        pub const MGKPKTEN = usize(0x00000002); // Magic Packet Enable
        pub const PWRDWN = usize(0x00000001); // Power Down
        pub const RWKPTR_S = usize(24);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_LPICTLSTAT
    // register.
    //
    //*****************************************************************************
    pub const LPICTLSTAT = struct {
        pub const LPITXA = usize(0x00080000); // LPI TX Automate
        pub const PLSEN = usize(0x00040000); // PHY Link Status Enable
        pub const PLS = usize(0x00020000); // PHY Link Status
        pub const LPIEN = usize(0x00010000); // LPI Enable
        pub const RLPIST = usize(0x00000200); // Receive LPI State
        pub const TLPIST = usize(0x00000100); // Transmit LPI State
        pub const RLPIEX = usize(0x00000008); // Receive LPI Exit
        pub const RLPIEN = usize(0x00000004); // Receive LPI Entry
        pub const TLPIEX = usize(0x00000002); // Transmit LPI Exit
        pub const TLPIEN = usize(0x00000001); // Transmit LPI Entry
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_LPITIMERCTL
    // register.
    //
    //*****************************************************************************
    pub const LPITIMERCTL = struct {
        pub const LST_M = usize(0x03FF0000); // Low Power Idle LS Timer
        pub const LST_S = usize(16);
        pub const TWT_M = usize(0x0000FFFF); // Low Power Idle TW Timer
        pub const TWT_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_RIS register.
    //
    //*****************************************************************************
    pub const RIS = struct {
        pub const LPI = usize(0x00000400); // LPI Interrupt Status
        pub const TS = usize(0x00000200); // Timestamp Interrupt Status
        pub const MMCTX = usize(0x00000040); // MMC Transmit Interrupt Status
        pub const MMCRX = usize(0x00000020); // MMC Receive Interrupt Status
        pub const MMC = usize(0x00000010); // MMC Interrupt Status
        pub const PMT = usize(0x00000008); // PMT Interrupt Status
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_IM register.
    //
    //*****************************************************************************
    pub const IM = struct {
        pub const LPI = usize(0x00000400); // LPI Interrupt Mask
        pub const TSI = usize(0x00000200); // Timestamp Interrupt Mask
        pub const PMT = usize(0x00000008); // PMT Interrupt Mask
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_ADDR0H register.
    //
    //*****************************************************************************
    pub const ADDR0H = struct {
        pub const AE = usize(0x80000000); // Address Enable
        pub const ADDRHI_M = usize(0x0000FFFF); // MAC Address0 [47:32]
        pub const ADDRHI_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_ADDR0L register.
    //
    //*****************************************************************************
    pub const ADDR0L = struct {
        pub const ADDRLO_M = usize(0xFFFFFFFF); // MAC Address0 [31:0]
        pub const ADDRLO_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_ADDR1H register.
    //
    //*****************************************************************************
    pub const ADDR1H = struct {
        pub const AE = usize(0x80000000); // Address Enable
        pub const SA = usize(0x40000000); // Source Address
        pub const MBC_M = usize(0x3F000000); // Mask Byte Control
        pub const ADDRHI_M = usize(0x0000FFFF); // MAC Address1 [47:32]
        pub const MBC_S = usize(24);
        pub const ADDRHI_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_ADDR1L register.
    //
    //*****************************************************************************
    pub const ADDR1L = struct {
        pub const ADDRLO_M = usize(0xFFFFFFFF); // MAC Address1 [31:0]
        pub const ADDRLO_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_ADDR2H register.
    //
    //*****************************************************************************
    pub const ADDR2H = struct {
        pub const AE = usize(0x80000000); // Address Enable
        pub const SA = usize(0x40000000); // Source Address
        pub const MBC_M = usize(0x3F000000); // Mask Byte Control
        pub const ADDRHI_M = usize(0x0000FFFF); // MAC Address2 [47:32]
        pub const MBC_S = usize(24);
        pub const ADDRHI_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_ADDR2L register.
    //
    //*****************************************************************************
    pub const ADDR2L = struct {
        pub const ADDRLO_M = usize(0xFFFFFFFF); // MAC Address2 [31:0]
        pub const ADDRLO_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_ADDR3H register.
    //
    //*****************************************************************************
    pub const ADDR3H = struct {
        pub const AE = usize(0x80000000); // Address Enable
        pub const SA = usize(0x40000000); // Source Address
        pub const MBC_M = usize(0x3F000000); // Mask Byte Control
        pub const ADDRHI_M = usize(0x0000FFFF); // MAC Address3 [47:32]
        pub const MBC_S = usize(24);
        pub const ADDRHI_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_ADDR3L register.
    //
    //*****************************************************************************
    pub const ADDR3L = struct {
        pub const ADDRLO_M = usize(0xFFFFFFFF); // MAC Address3 [31:0]
        pub const ADDRLO_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_WDOGTO register.
    //
    //*****************************************************************************
    pub const WDOGTO = struct {
        pub const PWE = usize(0x00010000); // Programmable Watchdog Enable
        pub const WTO_M = usize(0x00003FFF); // Watchdog Timeout
        pub const WTO_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_MMCCTRL register.
    //
    //*****************************************************************************
    pub const MMCCTRL = struct {
        pub const UCDBC = usize(0x00000100); // Update MMC Counters for Dropped
        // Broadcast Frames
        pub const CNTPRSTLVL = usize(0x00000020); // Full/Half Preset Level Value
        pub const CNTPRST = usize(0x00000010); // Counters Preset
        pub const CNTFREEZ = usize(0x00000008); // MMC Counter Freeze
        pub const RSTONRD = usize(0x00000004); // Reset on Read
        pub const CNTSTPRO = usize(0x00000002); // Counters Stop Rollover
        pub const CNTRST = usize(0x00000001); // Counters Reset
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_MMCRXRIS
    // register.
    //
    //*****************************************************************************
    pub const MMCRXRIS = struct {
        pub const UCGF = usize(0x00020000); // MMC Receive Unicast Good Frame
        // Counter Interrupt Status
        pub const ALGNERR = usize(0x00000040); // MMC Receive Alignment Error
        // Frame Counter Interrupt Status
        pub const CRCERR = usize(0x00000020); // MMC Receive CRC Error Frame
        // Counter Interrupt Status
        pub const GBF = usize(0x00000001); // MMC Receive Good Bad Frame
        // Counter Interrupt Status
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_MMCTXRIS
    // register.
    //
    //*****************************************************************************
    pub const MMCTXRIS = struct {
        pub const OCTCNT = usize(0x00100000); // Octet Counter Interrupt Status
        pub const MCOLLGF = usize(0x00008000); // MMC Transmit Multiple Collision
        // Good Frame Counter Interrupt
        // Status
        pub const SCOLLGF = usize(0x00004000); // MMC Transmit Single Collision
        // Good Frame Counter Interrupt
        // Status
        pub const GBF = usize(0x00000002); // MMC Transmit Good Bad Frame
        // Counter Interrupt Status
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_MMCRXIM register.
    //
    //*****************************************************************************
    pub const MMCRXIM = struct {
        pub const UCGF = usize(0x00020000); // MMC Receive Unicast Good Frame
        // Counter Interrupt Mask
        pub const ALGNERR = usize(0x00000040); // MMC Receive Alignment Error
        // Frame Counter Interrupt Mask
        pub const CRCERR = usize(0x00000020); // MMC Receive CRC Error Frame
        // Counter Interrupt Mask
        pub const GBF = usize(0x00000001); // MMC Receive Good Bad Frame
        // Counter Interrupt Mask
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_MMCTXIM register.
    //
    //*****************************************************************************
    pub const MMCTXIM = struct {
        pub const OCTCNT = usize(0x00100000); // MMC Transmit Good Octet Counter
        // Interrupt Mask
        pub const MCOLLGF = usize(0x00008000); // MMC Transmit Multiple Collision
        // Good Frame Counter Interrupt
        // Mask
        pub const SCOLLGF = usize(0x00004000); // MMC Transmit Single Collision
        // Good Frame Counter Interrupt
        // Mask
        pub const GBF = usize(0x00000002); // MMC Transmit Good Bad Frame
        // Counter Interrupt Mask
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TXCNTGB register.
    //
    //*****************************************************************************
    pub const TXCNTGB = struct {
        pub const TXFRMGB_M = usize(0xFFFFFFFF); // This field indicates the number
        // of good and bad frames
        // transmitted, exclusive of
        // retried frames
        pub const TXFRMGB_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TXCNTSCOL
    // register.
    //
    //*****************************************************************************
    pub const TXCNTSCOL = struct {
        pub const TXSNGLCOLG_M = usize(0xFFFFFFFF); // This field indicates the number
        // of successfully transmitted
        // frames after a single collision
        // in the half-duplex mode
        pub const TXSNGLCOLG_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TXCNTMCOL
    // register.
    //
    //*****************************************************************************
    pub const TXCNTMCOL = struct {
        pub const TXMULTCOLG_M = usize(0xFFFFFFFF); // This field indicates the number
        // of successfully transmitted
        // frames after multiple collisions
        // in the half-duplex mode
        pub const TXMULTCOLG_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TXOCTCNTG
    // register.
    //
    //*****************************************************************************
    pub const TXOCTCNTG = struct {
        pub const TXOCTG_M = usize(0xFFFFFFFF); // This field indicates the number
        // of bytes transmitted, exclusive
        // of preamble, in good frames
        pub const TXOCTG_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_RXCNTGB register.
    //
    //*****************************************************************************
    pub const RXCNTGB = struct {
        pub const RXFRMGB_M = usize(0xFFFFFFFF); // This field indicates the number
        // of received good and bad frames
        pub const RXFRMGB_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_RXCNTCRCERR
    // register.
    //
    //*****************************************************************************
    pub const RXCNTCRCERR = struct {
        pub const RXCRCERR_M = usize(0xFFFFFFFF); // This field indicates the number
        // of frames received with CRC
        // error
        pub const RXCRCERR_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_RXCNTALGNERR
    // register.
    //
    //*****************************************************************************
    pub const RXCNTALGNERR = struct {
        pub const RXALGNERR_M = usize(0xFFFFFFFF); // This field indicates the number
        // of frames received with
        // alignment (dribble) error
        pub const RXALGNERR_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_RXCNTGUNI
    // register.
    //
    //*****************************************************************************
    pub const RXCNTGUNI = struct {
        pub const RXUCASTG_M = usize(0xFFFFFFFF); // This field indicates the number
        // of received good unicast frames
        pub const RXUCASTG_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_VLNINCREP
    // register.
    //
    //*****************************************************************************
    pub const VLNINCREP = struct {
        pub const CSVL = usize(0x00080000); // C-VLAN or S-VLAN
        pub const VLP = usize(0x00040000); // VLAN Priority Control
        pub const VLC_M = usize(0x00030000); // VLAN Tag Control in Transmit
        // Frames
        pub const VLC_NONE = usize(0x00000000); // No VLAN tag deletion, insertion,
        // or replacement
        pub const VLC_TAGDEL = usize(0x00010000); // VLAN tag deletion
        pub const VLC_TAGINS = usize(0x00020000); // VLAN tag insertion
        pub const VLC_TAGREP = usize(0x00030000); // VLAN tag replacement
        pub const VLT_M = usize(0x0000FFFF); // VLAN Tag for Transmit Frames
        pub const VLT_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_VLANHASH
    // register.
    //
    //*****************************************************************************
    pub const VLANHASH = struct {
        pub const VLHT_M = usize(0x0000FFFF); // VLAN Hash Table
        pub const VLHT_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TIMSTCTRL
    // register.
    //
    //*****************************************************************************
    pub const TIMSTCTRL = struct {
        pub const PTPFLTR = usize(0x00040000); // Enable MAC address for PTP Frame
        // Filtering
        pub const SELPTP_M = usize(0x00030000); // Select PTP packets for Taking
        // Snapshots
        pub const TSMAST = usize(0x00008000); // Enable Snapshot for Messages
        // Relevant to Master
        pub const TSEVNT = usize(0x00004000); // Enable Timestamp Snapshot for
        // Event Messages
        pub const PTPIPV4 = usize(0x00002000); // Enable Processing of PTP Frames
        // Sent over IPv4-UDP
        pub const PTPIPV6 = usize(0x00001000); // Enable Processing of PTP Frames
        // Sent Over IPv6-UDP
        pub const PTPETH = usize(0x00000800); // Enable Processing of PTP Over
        // Ethernet Frames
        pub const PTPVER2 = usize(0x00000400); // Enable PTP Packet Processing For
        // Version 2 Format
        pub const DGTLBIN = usize(0x00000200); // Timestamp Digital or Binary
        // Rollover Control
        pub const ALLF = usize(0x00000100); // Enable Timestamp For All Frames
        pub const ADDREGUP = usize(0x00000020); // Addend Register Update
        pub const INTTRIG = usize(0x00000010); // Timestamp Interrupt Trigger
        // Enable
        pub const TSUPDT = usize(0x00000008); // Timestamp Update
        pub const TSINIT = usize(0x00000004); // Timestamp Initialize
        pub const TSFCUPDT = usize(0x00000002); // Timestamp Fine or Coarse Update
        pub const TSEN = usize(0x00000001); // Timestamp Enable
        pub const SELPTP_S = usize(16);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_SUBSECINC
    // register.
    //
    //*****************************************************************************
    pub const SUBSECINC = struct {
        pub const SSINC_M = usize(0x000000FF); // Sub-second Increment Value
        pub const SSINC_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TIMSEC register.
    //
    //*****************************************************************************
    pub const TIMSEC = struct {
        pub const TSS_M = usize(0xFFFFFFFF); // Timestamp Second
        pub const TSS_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TIMNANO register.
    //
    //*****************************************************************************
    pub const TIMNANO = struct {
        pub const TSSS_M = usize(0x7FFFFFFF); // Timestamp Sub-Seconds
        pub const TSSS_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TIMSECU register.
    //
    //*****************************************************************************
    pub const TIMSECU = struct {
        pub const TSS_M = usize(0xFFFFFFFF); // Timestamp Second
        pub const TSS_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TIMNANOU
    // register.
    //
    //*****************************************************************************
    pub const TIMNANOU = struct {
        pub const ADDSUB = usize(0x80000000); // Add or subtract time
        pub const TSSS_M = usize(0x7FFFFFFF); // Timestamp Sub-Second
        pub const TSSS_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TIMADD register.
    //
    //*****************************************************************************
    pub const TIMADD = struct {
        pub const TSAR_M = usize(0xFFFFFFFF); // Timestamp Addend Register
        pub const TSAR_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TARGSEC register.
    //
    //*****************************************************************************
    pub const TARGSEC = struct {
        pub const TSTR_M = usize(0xFFFFFFFF); // Target Time Seconds Register
        pub const TSTR_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TARGNANO
    // register.
    //
    //*****************************************************************************
    pub const TARGNANO = struct {
        pub const TRGTBUSY = usize(0x80000000); // Target Time Register Busy
        pub const TTSLO_M = usize(0x7FFFFFFF); // Target Timestamp Low Register
        pub const TTSLO_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_HWORDSEC
    // register.
    //
    //*****************************************************************************
    pub const HWORDSEC = struct {
        pub const TSHWR_M = usize(0x0000FFFF); // Target Timestamp Higher Word
        // Register
        pub const TSHWR_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TIMSTAT register.
    //
    //*****************************************************************************
    pub const TIMSTAT = struct {
        pub const TSTARGT = usize(0x00000002); // Timestamp Target Time Reached
        pub const TSSOVF = usize(0x00000001); // Timestamp Seconds Overflow
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_PPSCTRL register.
    //
    //*****************************************************************************
    pub const PPSCTRL = struct {
        pub const TRGMODS0_M = usize(0x00000060); // Target Time Register Mode for
        // PPS0 Output
        pub const TRGMODS0_INTONLY = usize(0x00000000); // Indicates that the Target Time
        // registers are programmed only
        // for generating the interrupt
        // event
        pub const TRGMODS0_INTPPS0 = usize(0x00000040); // Indicates that the Target Time
        // registers are programmed for
        // generating the interrupt event
        // and starting or stopping the
        // generation of the EN0PPS output
        // signal
        pub const TRGMODS0_PPS0ONLY = usize(0x00000060); // Indicates that the Target Time
        // registers are programmed only
        // for starting or stopping the
        // generation of the EN0PPS output
        // signal. No interrupt is asserted
        pub const PPSEN0 = usize(0x00000010); // Flexible PPS Output Mode Enable
        pub const PPSCTRL_M = usize(0x0000000F); // EN0PPS Output Frequency Control
        // (PPSCTRL) or Command Control
        // (PPSCMD)
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_PPS0INTVL
    // register.
    //
    //*****************************************************************************
    pub const PPS0INTVL = struct {
        pub const PPS0INT_M = usize(0xFFFFFFFF); // PPS0 Output Signal Interval
        pub const PPS0INT_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_PPS0WIDTH
    // register.
    //
    //*****************************************************************************
    pub const PPS0WIDTH = struct {
        pub const M = usize(0xFFFFFFFF); // EN0PPS Output Signal Width
        pub const S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_DMABUSMOD
    // register.
    //
    //*****************************************************************************
    pub const DMABUSMOD = struct {
        pub const RIB = usize(0x80000000); // Rebuild Burst
        pub const TXPR = usize(0x08000000); // Transmit Priority
        pub const MB = usize(0x04000000); // Mixed Burst
        pub const AAL = usize(0x02000000); // Address Aligned Beats
        pub const _8XPBL = usize(0x01000000); // 8 x Programmable Burst Length
        // (PBL) Mode
        pub const USP = usize(0x00800000); // Use Separate Programmable Burst
        // Length (PBL)
        pub const RPBL_M = usize(0x007E0000); // RX DMA Programmable Burst Length
        // (PBL)
        pub const FB = usize(0x00010000); // Fixed Burst
        pub const PR_M = usize(0x0000C000); // Priority Ratio
        pub const PBL_M = usize(0x00003F00); // Programmable Burst Length
        pub const ATDS = usize(0x00000080); // Alternate Descriptor Size
        pub const DSL_M = usize(0x0000007C); // Descriptor Skip Length
        pub const DA = usize(0x00000002); // DMA Arbitration Scheme
        pub const SWR = usize(0x00000001); // DMA Software Reset
        pub const RPBL_S = usize(17);
        pub const PR_S = usize(14);
        pub const PBL_S = usize(8);
        pub const DSL_S = usize(2);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TXPOLLD register.
    //
    //*****************************************************************************
    pub const TXPOLLD = struct {
        pub const TPD_M = usize(0xFFFFFFFF); // Transmit Poll Demand
        pub const TPD_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_RXPOLLD register.
    //
    //*****************************************************************************
    pub const RXPOLLD = struct {
        pub const RPD_M = usize(0xFFFFFFFF); // Receive Poll Demand
        pub const RPD_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_RXDLADDR
    // register.
    //
    //*****************************************************************************
    pub const RXDLADDR = struct {
        pub const STRXLIST_M = usize(0xFFFFFFFC); // Start of Receive List
        pub const STRXLIST_S = usize(2);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_TXDLADDR
    // register.
    //
    //*****************************************************************************
    pub const TXDLADDR = struct {
        pub const TXDLADDR_M = usize(0xFFFFFFFC); // Start of Transmit List Base
        // Address
        pub const TXDLADDR_S = usize(2);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_DMARIS register.
    //
    //*****************************************************************************
    pub const DMARIS = struct {
        pub const LPI = usize(0x40000000); // LPI Trigger Interrupt Status
        pub const TT = usize(0x20000000); // Timestamp Trigger Interrupt
        // Status
        pub const PMT = usize(0x10000000); // MAC PMT Interrupt Status
        pub const MMC = usize(0x08000000); // MAC MMC Interrupt
        pub const AE_M = usize(0x03800000); // Access Error
        pub const AE_RXDMAWD = usize(0x00000000); // Error during RX DMA Write Data
        // Transfer
        pub const AE_TXDMARD = usize(0x01800000); // Error during TX DMA Read Data
        // Transfer
        pub const AE_RXDMADW = usize(0x02000000); // Error during RX DMA Descriptor
        // Write Access
        pub const AE_TXDMADW = usize(0x02800000); // Error during TX DMA Descriptor
        // Write Access
        pub const AE_RXDMADR = usize(0x03000000); // Error during RX DMA Descriptor
        // Read Access
        pub const AE_TXDMADR = usize(0x03800000); // Error during TX DMA Descriptor
        // Read Access
        pub const TS_M = usize(0x00700000); // Transmit Process State
        pub const TS_STOP = usize(0x00000000); // Stopped; Reset or Stop transmit
        // command processed
        pub const TS_RUNTXTD = usize(0x00100000); // Running; Fetching transmit
        // transfer descriptor
        pub const TS_STATUS = usize(0x00200000); // Running; Waiting for status
        pub const TS_RUNTX = usize(0x00300000); // Running; Reading data from host
        // memory buffer and queuing it to
        // transmit buffer (TX FIFO)
        pub const TS_TSTAMP = usize(0x00400000); // Writing Timestamp
        pub const TS_SUSPEND = usize(0x00600000); // Suspended; Transmit descriptor
        // unavailable or transmit buffer
        // underflow
        pub const TS_RUNCTD = usize(0x00700000); // Running; Closing transmit
        // descriptor
        pub const RS_M = usize(0x000E0000); // Received Process State
        pub const RS_STOP = usize(0x00000000); // Stopped: Reset or stop receive
        // command issued
        pub const RS_RUNRXTD = usize(0x00020000); // Running: Fetching receive
        // transfer descriptor
        pub const RS_RUNRXD = usize(0x00060000); // Running: Waiting for receive
        // packet
        pub const RS_SUSPEND = usize(0x00080000); // Suspended: Receive descriptor
        // unavailable
        pub const RS_RUNCRD = usize(0x000A0000); // Running: Closing receive
        // descriptor
        pub const RS_TSWS = usize(0x000C0000); // Writing Timestamp
        pub const RS_RUNTXD = usize(0x000E0000); // Running: Transferring the
        // receive packet data from receive
        // buffer to host memory
        pub const NIS = usize(0x00010000); // Normal Interrupt Summary
        pub const AIS = usize(0x00008000); // Abnormal Interrupt Summary
        pub const ERI = usize(0x00004000); // Early Receive Interrupt
        pub const FBI = usize(0x00002000); // Fatal Bus Error Interrupt
        pub const ETI = usize(0x00000400); // Early Transmit Interrupt
        pub const RWT = usize(0x00000200); // Receive Watchdog Timeout
        pub const RPS = usize(0x00000100); // Receive Process Stopped
        pub const RU = usize(0x00000080); // Receive Buffer Unavailable
        pub const RI = usize(0x00000040); // Receive Interrupt
        pub const UNF = usize(0x00000020); // Transmit Underflow
        pub const OVF = usize(0x00000010); // Receive Overflow
        pub const TJT = usize(0x00000008); // Transmit Jabber Timeout
        pub const TU = usize(0x00000004); // Transmit Buffer Unavailable
        pub const TPS = usize(0x00000002); // Transmit Process Stopped
        pub const TI = usize(0x00000001); // Transmit Interrupt
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_DMAOPMODE
    // register.
    //
    //*****************************************************************************
    pub const DMAOPMODE = struct {
        pub const DT = usize(0x04000000); // Disable Dropping of TCP/IP
        // Checksum Error Frames
        pub const RSF = usize(0x02000000); // Receive Store and Forward
        pub const DFF = usize(0x01000000); // Disable Flushing of Received
        // Frames
        pub const TSF = usize(0x00200000); // Transmit Store and Forward
        pub const FTF = usize(0x00100000); // Flush Transmit FIFO
        pub const TTC_M = usize(0x0001C000); // Transmit Threshold Control
        pub const TTC_64 = usize(0x00000000); // 64 bytes
        pub const TTC_128 = usize(0x00004000); // 128 bytes
        pub const TTC_192 = usize(0x00008000); // 192 bytes
        pub const TTC_256 = usize(0x0000C000); // 256 bytes
        pub const TTC_40 = usize(0x00010000); // 40 bytes
        pub const TTC_32 = usize(0x00014000); // 32 bytes
        pub const TTC_24 = usize(0x00018000); // 24 bytes
        pub const TTC_16 = usize(0x0001C000); // 16 bytes
        pub const ST = usize(0x00002000); // Start or Stop Transmission
        // Command
        pub const FEF = usize(0x00000080); // Forward Error Frames
        pub const FUF = usize(0x00000040); // Forward Undersized Good Frames
        pub const DGF = usize(0x00000020); // Drop Giant Frame Enable
        pub const RTC_M = usize(0x00000018); // Receive Threshold Control
        pub const RTC_64 = usize(0x00000000); // 64 bytes
        pub const RTC_32 = usize(0x00000008); // 32 bytes
        pub const RTC_96 = usize(0x00000010); // 96 bytes
        pub const RTC_128 = usize(0x00000018); // 128 bytes
        pub const OSF = usize(0x00000004); // Operate on Second Frame
        pub const SR = usize(0x00000002); // Start or Stop Receive
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_DMAIM register.
    //
    //*****************************************************************************
    pub const DMAIM = struct {
        pub const NIE = usize(0x00010000); // Normal Interrupt Summary Enable
        pub const AIE = usize(0x00008000); // Abnormal Interrupt Summary
        // Enable
        pub const ERE = usize(0x00004000); // Early Receive Interrupt Enable
        pub const FBE = usize(0x00002000); // Fatal Bus Error Enable
        pub const ETE = usize(0x00000400); // Early Transmit Interrupt Enable
        pub const RWE = usize(0x00000200); // Receive Watchdog Timeout Enable
        pub const RSE = usize(0x00000100); // Receive Stopped Enable
        pub const RUE = usize(0x00000080); // Receive Buffer Unavailable
        // Enable
        pub const RIE = usize(0x00000040); // Receive Interrupt Enable
        pub const UNE = usize(0x00000020); // Underflow Interrupt Enable
        pub const OVE = usize(0x00000010); // Overflow Interrupt Enable
        pub const TJE = usize(0x00000008); // Transmit Jabber Timeout Enable
        pub const TUE = usize(0x00000004); // Transmit Buffer Unvailable
        // Enable
        pub const TSE = usize(0x00000002); // Transmit Stopped Enable
        pub const TIE = usize(0x00000001); // Transmit Interrupt Enable
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_MFBOC register.
    //
    //*****************************************************************************
    pub const MFBOC = struct {
        pub const OVFCNTOVF = usize(0x10000000); // Overflow Bit for FIFO Overflow
        // Counter
        pub const OVFFRMCNT_M = usize(0x0FFE0000); // Overflow Frame Counter
        pub const MISCNTOVF = usize(0x00010000); // Overflow bit for Missed Frame
        // Counter
        pub const MISFRMCNT_M = usize(0x0000FFFF); // Missed Frame Counter
        pub const OVFFRMCNT_S = usize(17);
        pub const MISFRMCNT_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_RXINTWDT
    // register.
    //
    //*****************************************************************************
    pub const RXINTWDT = struct {
        pub const RIWT_M = usize(0x000000FF); // Receive Interrupt Watchdog Timer
        // Count
        pub const RIWT_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_HOSTXDESC
    // register.
    //
    //*****************************************************************************
    pub const HOSTXDESC = struct {
        pub const CURTXDESC_M = usize(0xFFFFFFFF); // Host Transmit Descriptor Address
        // Pointer
        pub const CURTXDESC_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_HOSRXDESC
    // register.
    //
    //*****************************************************************************
    pub const HOSRXDESC = struct {
        pub const CURRXDESC_M = usize(0xFFFFFFFF); // Host Receive Descriptor Address
        // Pointer
        pub const CURRXDESC_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_HOSTXBA register.
    //
    //*****************************************************************************
    pub const HOSTXBA = struct {
        pub const CURTXBUFA_M = usize(0xFFFFFFFF); // Host Transmit Buffer Address
        // Pointer
        pub const CURTXBUFA_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_HOSRXBA register.
    //
    //*****************************************************************************
    pub const HOSRXBA = struct {
        pub const CURRXBUFA_M = usize(0xFFFFFFFF); // Host Receive Buffer Address
        // Pointer
        pub const CURRXBUFA_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_PP register.
    //
    //*****************************************************************************
    pub const PP = struct {
        pub const MACTYPE_M = usize(0x00000700); // Ethernet MAC Type
        pub const MACTYPE_1 = usize(0x00000100); // Tiva TM4E129x-class MAC
        pub const PHYTYPE_M = usize(0x00000007); // Ethernet PHY Type
        pub const PHYTYPE_NONE = usize(0x00000000); // No PHY
        pub const PHYTYPE_1 = usize(0x00000003); // Snowflake class PHY
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_PC register.
    //
    //*****************************************************************************
    pub const PC = struct {
        pub const PHYEXT = usize(0x80000000); // PHY Select
        pub const PINTFS_M = usize(0x70000000); // Ethernet Interface Select
        pub const PINTFS_IMII = usize(0x00000000); // MII (default) Used for internal
        // PHY or external PHY connected
        // via MII
        pub const PINTFS_RMII = usize(0x40000000); // RMII: Used for external PHY
        // connected via RMII
        pub const DIGRESTART = usize(0x02000000); // PHY Soft Restart
        pub const NIBDETDIS = usize(0x01000000); // Odd Nibble TXER Detection
        // Disable
        pub const RXERIDLE = usize(0x00800000); // RXER Detection During Idle
        pub const ISOMIILL = usize(0x00400000); // Isolate MII in Link Loss
        pub const LRR = usize(0x00200000); // Link Loss Recovery
        pub const TDRRUN = usize(0x00100000); // TDR Auto Run
        pub const FASTLDMODE_M = usize(0x000F8000); // Fast Link Down Mode
        pub const POLSWAP = usize(0x00004000); // Polarity Swap
        pub const MDISWAP = usize(0x00002000); // MDI Swap
        pub const RBSTMDIX = usize(0x00001000); // Robust Auto MDI-X
        pub const FASTMDIX = usize(0x00000800); // Fast Auto MDI-X
        pub const MDIXEN = usize(0x00000400); // MDIX Enable
        pub const FASTRXDV = usize(0x00000200); // Fast RXDV Detection
        pub const FASTLUPD = usize(0x00000100); // FAST Link-Up in Parallel Detect
        pub const EXTFD = usize(0x00000080); // Extended Full Duplex Ability
        pub const FASTANEN = usize(0x00000040); // Fast Auto Negotiation Enable
        pub const FASTANSEL_M = usize(0x00000030); // Fast Auto Negotiation Select
        pub const ANEN = usize(0x00000008); // Auto Negotiation Enable
        pub const ANMODE_M = usize(0x00000006); // Auto Negotiation Mode
        pub const ANMODE_10HD = usize(0x00000000); // When ANEN = 0x0, the mode is
        // 10Base-T, Half-Duplex
        pub const ANMODE_10FD = usize(0x00000002); // When ANEN = 0x0, the mode is
        // 10Base-T, Full-Duplex
        pub const ANMODE_100HD = usize(0x00000004); // When ANEN = 0x0, the mode is
        // 100Base-TX, Half-Duplex
        pub const ANMODE_100FD = usize(0x00000006); // When ANEN = 0x0, the mode is
        // 100Base-TX, Full-Duplex
        pub const PHYHOLD = usize(0x00000001); // Ethernet PHY Hold
        pub const FASTLDMODE_S = usize(15);
        pub const FASTANSEL_S = usize(4);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_CC register.
    //
    //*****************************************************************************
    pub const CC = struct {
        pub const PTPCEN = usize(0x00040000); // PTP Clock Reference Enable
        pub const POL = usize(0x00020000); // LED Polarity Control
        pub const CLKEN = usize(0x00010000); // EN0RREF_CLK Signal Enable
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_EPHYRIS register.
    //
    //*****************************************************************************
    pub const EPHYRIS = struct {
        pub const INT = usize(0x00000001); // Ethernet PHY Raw Interrupt
        // Status
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_EPHYIM register.
    //
    //*****************************************************************************
    pub const EPHYIM = struct {
        pub const INT = usize(0x00000001); // Ethernet PHY Interrupt Mask
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EMAC_O_EPHYMISC
    // register.
    //
    //*****************************************************************************
    pub const EPHYMISC = struct {
        pub const INT = usize(0x00000001); // Ethernet PHY Status and Clear
        // register
    };
};
pub const EPHY = struct {
    //*****************************************************************************
    //
    // The following are defines for the EPHY register offsets.
    //
    //*****************************************************************************
    pub const offsetOf = struct {
        pub const BMCR = usize(0x00000000); // Ethernet PHY Basic Mode Control
        pub const BMSR = usize(0x00000001); // Ethernet PHY Basic Mode Status
        pub const ID1 = usize(0x00000002); // Ethernet PHY Identifier Register
        // 1
        pub const ID2 = usize(0x00000003); // Ethernet PHY Identifier Register
        // 2
        pub const ANA = usize(0x00000004); // Ethernet PHY Auto-Negotiation
        // Advertit
        pub const ANLPA = usize(0x00000005); // Ethernet PHY Auto-Negotiation
        // Link Pa Ability
        pub const ANER = usize(0x00000006); // Ethernet PHY Auto-Negotiation
        // Expansi
        pub const ANNPTR = usize(0x00000007); // Ethernet PHY Auto-Negotiation
        // Next Pa
        pub const ANLNPTR = usize(0x00000008); // Ethernet PHY Auto-Negotiation
        // Link Pa Ability Next Page
        pub const CFG1 = usize(0x00000009); // Ethernet PHY Configuration 1
        pub const CFG2 = usize(0x0000000A); // Ethernet PHY Configuration 2
        pub const CFG3 = usize(0x0000000B); // Ethernet PHY Configuration 3
        pub const REGCTL = usize(0x0000000D); // Ethernet PHY Register Control
        pub const ADDAR = usize(0x0000000E); // Ethernet PHY Address or Data
        pub const STS = usize(0x00000010); // Ethernet PHY Status
        pub const SCR = usize(0x00000011); // Ethernet PHY Specific Control
        pub const MISR1 = usize(0x00000012); // Ethernet PHY MII Interrupt
        // Status
        pub const MISR2 = usize(0x00000013); // Ethernet PHY MII Interrupt
        // Status
        pub const FCSCR = usize(0x00000014); // Ethernet PHY False Carrier Sense
        // Counter
        pub const RXERCNT = usize(0x00000015); // Ethernet PHY Receive Error Count
        pub const BISTCR = usize(0x00000016); // Ethernet PHY BIST Control
        pub const LEDCR = usize(0x00000018); // Ethernet PHY LED Control
        pub const CTL = usize(0x00000019); // Ethernet PHY Control
        pub const _10BTSC = usize(0x0000001A); // Ethernet PHY 10Base-T
        // Status/ol - MR26
        pub const BICSR1 = usize(0x0000001B); // Ethernet PHY BIST Control and
        // Status
        pub const BICSR2 = usize(0x0000001C); // Ethernet PHY BIST Control and
        // Status
        pub const CDCR = usize(0x0000001E); // Ethernet PHY Cable Diagnostic
        // Control
        pub const RCR = usize(0x0000001F); // Ethernet PHY Reset Control
        pub const LEDCFG = usize(0x00000025); // Ethernet PHY LED Configuration
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_BMCR register.
    //
    //*****************************************************************************
    pub const BMCR = struct {
        pub const MIIRESET = usize(0x00008000); // MII Register reset
        pub const MIILOOPBK = usize(0x00004000); // MII Loopback
        pub const SPEED = usize(0x00002000); // Speed Select
        pub const ANEN = usize(0x00001000); // Auto-Negotiate Enable
        pub const PWRDWN = usize(0x00000800); // Power Down
        pub const ISOLATE = usize(0x00000400); // Port Isolate
        pub const RESTARTAN = usize(0x00000200); // Restart Auto-Negotiation
        pub const DUPLEXM = usize(0x00000100); // Duplex Mode
        pub const COLLTST = usize(0x00000080); // Collision Test
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_BMSR register.
    //
    //*****************************************************************************
    pub const BMSR = struct {
        pub const _100BTXFD = usize(0x00004000); // 100Base-TX Full Duplex Capable
        pub const _100BTXHD = usize(0x00002000); // 100Base-TX Half Duplex Capable
        pub const _10BTFD = usize(0x00001000); // 10 Base-T Full Duplex Capable
        pub const _10BTHD = usize(0x00000800); // 10 Base-T Half Duplex Capable
        pub const MFPRESUP = usize(0x00000040); // Preamble Suppression Capable
        pub const ANC = usize(0x00000020); // Auto-Negotiation Complete
        pub const RFAULT = usize(0x00000010); // Remote Fault
        pub const ANEN = usize(0x00000008); // Auto Negotiation Enabled
        pub const LINKSTAT = usize(0x00000004); // Link Status
        pub const JABBER = usize(0x00000002); // Jabber Detect
        pub const EXTEN = usize(0x00000001); // Extended Capability Enable
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_ID1 register.
    //
    //*****************************************************************************
    pub const ID1 = struct {
        pub const OUIMSB_M = usize(0x0000FFFF); // OUI Most Significant Bits
        pub const OUIMSB_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_ID2 register.
    //
    //*****************************************************************************
    pub const ID2 = struct {
        pub const OUILSB_M = usize(0x0000FC00); // OUI Least Significant Bits
        pub const VNDRMDL_M = usize(0x000003F0); // Vendor Model Number
        pub const MDLREV_M = usize(0x0000000F); // Model Revision Number
        pub const OUILSB_S = usize(10);
        pub const VNDRMDL_S = usize(4);
        pub const MDLREV_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_ANA register.
    //
    //*****************************************************************************
    pub const ANA = struct {
        pub const NP = usize(0x00008000); // Next Page Indication
        pub const RF = usize(0x00002000); // Remote Fault
        pub const ASMDUP = usize(0x00000800); // Asymmetric PAUSE support for
        // Full Duplex Links
        pub const PAUSE = usize(0x00000400); // PAUSE Support for Full Duplex
        // Links
        pub const _100BT4 = usize(0x00000200); // 100Base-T4 Support
        pub const _100BTXFD = usize(0x00000100); // 100Base-TX Full Duplex Support
        pub const _100BTX = usize(0x00000080); // 100Base-TX Support
        pub const _10BTFD = usize(0x00000040); // 10Base-T Full Duplex Support
        pub const _10BT = usize(0x00000020); // 10Base-T Support
        pub const SELECT_M = usize(0x0000001F); // Protocol Selection
        pub const SELECT_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_ANLPA register.
    //
    //*****************************************************************************
    pub const ANLPA = struct {
        pub const NP = usize(0x00008000); // Next Page Indication
        pub const ACK = usize(0x00004000); // Acknowledge
        pub const RF = usize(0x00002000); // Remote Fault
        pub const ASMDUP = usize(0x00000800); // Asymmetric PAUSE
        pub const PAUSE = usize(0x00000400); // PAUSE
        pub const _100BT4 = usize(0x00000200); // 100Base-T4 Support
        pub const _100BTXFD = usize(0x00000100); // 100Base-TX Full Duplex Support
        pub const _100BTX = usize(0x00000080); // 100Base-TX Support
        pub const _10BTFD = usize(0x00000040); // 10Base-T Full Duplex Support
        pub const _10BT = usize(0x00000020); // 10Base-T Support
        pub const SELECT_M = usize(0x0000001F); // Protocol Selection
        pub const SELECT_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_ANER register.
    //
    //*****************************************************************************
    pub const ANER = struct {
        pub const PDF = usize(0x00000010); // Parallel Detection Fault
        pub const LPNPABLE = usize(0x00000008); // Link Partner Next Page Able
        pub const NPABLE = usize(0x00000004); // Next Page Able
        pub const PAGERX = usize(0x00000002); // Link Code Word Page Received
        pub const LPANABLE = usize(0x00000001); // Link Partner Auto-Negotiation
        // Able
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_ANNPTR register.
    //
    //*****************************************************************************
    pub const ANNPTR = struct {
        pub const NP = usize(0x00008000); // Next Page Indication
        pub const MP = usize(0x00002000); // Message Page
        pub const ACK2 = usize(0x00001000); // Acknowledge 2
        pub const TOGTX = usize(0x00000800); // Toggle
        pub const CODE_M = usize(0x000007FF); // Code
        pub const CODE_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_ANLNPTR register.
    //
    //*****************************************************************************
    pub const ANLNPTR = struct {
        pub const NP = usize(0x00008000); // Next Page Indication
        pub const ACK = usize(0x00004000); // Acknowledge
        pub const MP = usize(0x00002000); // Message Page
        pub const ACK2 = usize(0x00001000); // Acknowledge 2
        pub const TOG = usize(0x00000800); // Toggle
        pub const CODE_M = usize(0x000007FF); // Code
        pub const CODE_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_CFG1 register.
    //
    //*****************************************************************************
    pub const CFG1 = struct {
        pub const DONE = usize(0x00008000); // Configuration Done
        pub const TDRAR = usize(0x00000100); // TDR Auto-Run at Link Down
        pub const LLR = usize(0x00000080); // Link Loss Recovery
        pub const FAMDIX = usize(0x00000040); // Fast Auto MDI/MDIX
        pub const RAMDIX = usize(0x00000020); // Robust Auto MDI/MDIX
        pub const FASTANEN = usize(0x00000010); // Fast Auto Negotiation Enable
        pub const FANSEL_M = usize(0x0000000C); // Fast Auto-Negotiation Select
        // Configuration
        pub const FANSEL_BLT80 = usize(0x00000000); // Break Link Timer: 80 ms
        pub const FANSEL_BLT120 = usize(0x00000004); // Break Link Timer: 120 ms
        pub const FANSEL_BLT240 = usize(0x00000008); // Break Link Timer: 240 ms
        pub const FRXDVDET = usize(0x00000002); // FAST RXDV Detection
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_CFG2 register.
    //
    //*****************************************************************************
    pub const CFG2 = struct {
        pub const FLUPPD = usize(0x00000040); // Fast Link-Up in Parallel Detect
        // Mode
        pub const EXTFD = usize(0x00000020); // Extended Full-Duplex Ability
        pub const ENLEDLINK = usize(0x00000010); // Enhanced LED Functionality
        pub const ISOMIILL = usize(0x00000008); // Isolate MII outputs when
        // Enhanced Link is not Achievable
        pub const RXERRIDLE = usize(0x00000004); // Detection of Receive Symbol
        // Error During IDLE State
        pub const ODDNDETDIS = usize(0x00000002); // Detection of Transmit Error
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_CFG3 register.
    //
    //*****************************************************************************
    pub const CFG3 = struct {
        pub const POLSWAP = usize(0x00000080); // Polarity Swap
        pub const MDIMDIXS = usize(0x00000040); // MDI/MDIX Swap
        pub const FLDWNM_M = usize(0x0000001F); // Fast Link Down Modes
        pub const FLDWNM_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_REGCTL register.
    //
    //*****************************************************************************
    pub const REGCTL = struct {
        pub const FUNC_M = usize(0x0000C000); // Function
        pub const FUNC_ADDR = usize(0x00000000); // Address
        pub const FUNC_DATANI = usize(0x00004000); // Data, no post increment
        pub const FUNC_DATAPIRW = usize(0x00008000); // Data, post increment on read and
        // write
        pub const FUNC_DATAPIWO = usize(0x0000C000); // Data, post increment on write
        // only
        pub const DEVAD_M = usize(0x0000001F); // Device Address
        pub const DEVAD_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_ADDAR register.
    //
    //*****************************************************************************
    pub const ADDAR = struct {
        pub const ADDRDATA_M = usize(0x0000FFFF); // Address or Data
        pub const ADDRDATA_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_STS register.
    //
    //*****************************************************************************
    pub const STS = struct {
        pub const MDIXM = usize(0x00004000); // MDI-X Mode
        pub const RXLERR = usize(0x00002000); // Receive Error Latch
        pub const POLSTAT = usize(0x00001000); // Polarity Status
        pub const FCSL = usize(0x00000800); // False Carrier Sense Latch
        pub const SD = usize(0x00000400); // Signal Detect
        pub const DL = usize(0x00000200); // Descrambler Lock
        pub const PAGERX = usize(0x00000100); // Link Code Page Received
        pub const MIIREQ = usize(0x00000080); // MII Interrupt Pending
        pub const RF = usize(0x00000040); // Remote Fault
        pub const JD = usize(0x00000020); // Jabber Detect
        pub const ANS = usize(0x00000010); // Auto-Negotiation Status
        pub const MIILB = usize(0x00000008); // MII Loopback Status
        pub const DUPLEX = usize(0x00000004); // Duplex Status
        pub const SPEED = usize(0x00000002); // Speed Status
        pub const LINK = usize(0x00000001); // Link Status
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_SCR register.
    //
    //*****************************************************************************
    pub const SCR = struct {
        pub const DISCLK = usize(0x00008000); // Disable CLK
        pub const PSEN = usize(0x00004000); // Power Saving Modes Enable
        pub const PSMODE_M = usize(0x00003000); // Power Saving Modes
        pub const PSMODE_NORMAL = usize(0x00000000); // Normal: Normal operation mode.
        // PHY is fully functional
        pub const PSMODE_LOWPWR = usize(0x00001000); // IEEE Power Down
        pub const PSMODE_ACTWOL = usize(0x00002000); // Active Sleep
        pub const PSMODE_PASWOL = usize(0x00003000); // Passive Sleep
        pub const SBPYASS = usize(0x00000800); // Scrambler Bypass
        pub const LBFIFO_M = usize(0x00000300); // Loopback FIFO Depth
        pub const LBFIFO_4 = usize(0x00000000); // Four nibble FIFO
        pub const LBFIFO_5 = usize(0x00000100); // Five nibble FIFO
        pub const LBFIFO_6 = usize(0x00000200); // Six nibble FIFO
        pub const LBFIFO_8 = usize(0x00000300); // Eight nibble FIFO
        pub const COLFDM = usize(0x00000010); // Collision in Full-Duplex Mode
        pub const TINT = usize(0x00000004); // Test Interrupt
        pub const INTEN = usize(0x00000002); // Interrupt Enable
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_MISR1 register.
    //
    //*****************************************************************************
    pub const MISR1 = struct {
        pub const LINKSTAT = usize(0x00002000); // Change of Link Status Interrupt
        pub const SPEED = usize(0x00001000); // Change of Speed Status Interrupt
        pub const DUPLEXM = usize(0x00000800); // Change of Duplex Status
        // Interrupt
        pub const ANC = usize(0x00000400); // Auto-Negotiation Complete
        // Interrupt
        pub const FCHF = usize(0x00000200); // False Carrier Counter Half-Full
        // Interrupt
        pub const RXHF = usize(0x00000100); // Receive Error Counter Half-Full
        // Interrupt
        pub const LINKSTATEN = usize(0x00000020); // Link Status Interrupt Enable
        pub const SPEEDEN = usize(0x00000010); // Speed Change Interrupt Enable
        pub const DUPLEXMEN = usize(0x00000008); // Duplex Status Interrupt Enable
        pub const ANCEN = usize(0x00000004); // Auto-Negotiation Complete
        // Interrupt Enable
        pub const FCHFEN = usize(0x00000002); // False Carrier Counter Register
        // half-full Interrupt Enable
        pub const RXHFEN = usize(0x00000001); // Receive Error Counter Register
        // Half-Full Event Interrupt
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_MISR2 register.
    //
    //*****************************************************************************
    pub const MISR2 = struct {
        pub const ANERR = usize(0x00004000); // Auto-Negotiation Error Interrupt
        pub const PAGERX = usize(0x00002000); // Page Receive Interrupt
        pub const LBFIFO = usize(0x00001000); // Loopback FIFO Overflow/Underflow
        // Event Interrupt
        pub const MDICO = usize(0x00000800); // MDI/MDIX Crossover Status
        // Changed Interrupt
        pub const SLEEP = usize(0x00000400); // Sleep Mode Event Interrupt
        pub const POLINT = usize(0x00000200); // Polarity Changed Interrupt
        pub const JABBER = usize(0x00000100); // Jabber Detect Event Interrupt
        pub const ANERREN = usize(0x00000040); // Auto-Negotiation Error Interrupt
        // Enable
        pub const PAGERXEN = usize(0x00000020); // Page Receive Interrupt Enable
        pub const LBFIFOEN = usize(0x00000010); // Loopback FIFO Overflow/Underflow
        // Interrupt Enable
        pub const MDICOEN = usize(0x00000008); // MDI/MDIX Crossover Status
        // Changed Interrupt Enable
        pub const SLEEPEN = usize(0x00000004); // Sleep Mode Event Interrupt
        // Enable
        pub const POLINTEN = usize(0x00000002); // Polarity Changed Interrupt
        // Enable
        pub const JABBEREN = usize(0x00000001); // Jabber Detect Event Interrupt
        // Enable
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_FCSCR register.
    //
    //*****************************************************************************
    pub const FCSCR = struct {
        pub const FCSCNT_M = usize(0x000000FF); // False Carrier Event Counter
        pub const FCSCNT_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_RXERCNT register.
    //
    //*****************************************************************************
    pub const RXERCNT = struct {
        pub const RXERRCNT_M = usize(0x0000FFFF); // Receive Error Count
        pub const RXERRCNT_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_BISTCR register.
    //
    //*****************************************************************************
    pub const BISTCR = struct {
        pub const PRBSM = usize(0x00004000); // PRBS Single/Continuous Mode
        pub const PRBSPKT = usize(0x00002000); // Generated PRBS Packets
        pub const PKTEN = usize(0x00001000); // Packet Generation Enable
        pub const PRBSCHKLK = usize(0x00000800); // PRBS Checker Lock Indication
        pub const PRBSCHKSYNC = usize(0x00000400); // PRBS Checker Lock Sync Loss
        // Indication
        pub const PKTGENSTAT = usize(0x00000200); // Packet Generator Status
        // Indication
        pub const PWRMODE = usize(0x00000100); // Power Mode Indication
        pub const TXMIILB = usize(0x00000040); // Transmit Data in MII Loopback
        // Mode
        pub const LBMODE_M = usize(0x0000001F); // Loopback Mode Select
        pub const LBMODE_NPCSIN = usize(0x00000001); // Near-end loopback: PCS Input
        // Loopback
        pub const LBMODE_NPCSOUT = usize(0x00000002); // Near-end loopback: PCS Output
        // Loopback (In 100Base-TX only)
        pub const LBMODE_NDIG = usize(0x00000004); // Near-end loopback: Digital
        // Loopback
        pub const LBMODE_NANA = usize(0x00000008); // Near-end loopback: Analog
        // Loopback (requires 100 Ohm
        // termination)
        pub const LBMODE_FREV = usize(0x00000010); // Far-end Loopback: Reverse
        // Loopback
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_LEDCR register.
    //
    //*****************************************************************************
    pub const LEDCR = struct {
        pub const BLINKRATE_M = usize(0x00000600); // LED Blinking Rate (ON/OFF
        // duration):
        pub const BLINKRATE_20HZ = usize(0x00000000); // 20 Hz (50 ms)
        pub const BLINKRATE_10HZ = usize(0x00000200); // 10 Hz (100 ms)
        pub const BLINKRATE_5HZ = usize(0x00000400); // 5 Hz (200 ms)
        pub const BLINKRATE_2HZ = usize(0x00000600); // 2 Hz (500 ms)
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_CTL register.
    //
    //*****************************************************************************
    pub const CTL = struct {
        pub const AUTOMDI = usize(0x00008000); // Auto-MDIX Enable
        pub const FORCEMDI = usize(0x00004000); // Force MDIX
        pub const PAUSERX = usize(0x00002000); // Pause Receive Negotiated Status
        pub const PAUSETX = usize(0x00001000); // Pause Transmit Negotiated Status
        pub const MIILNKSTAT = usize(0x00000800); // MII Link Status
        pub const BYPLEDSTRCH = usize(0x00000080); // Bypass LED Stretching
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_10BTSC register.
    //
    //*****************************************************************************
    pub const _10BTSC = struct {
        pub const RXTHEN = usize(0x00002000); // Lower Receiver Threshold Enable
        pub const SQUELCH_M = usize(0x00001E00); // Squelch Configuration
        pub const NLPDIS = usize(0x00000080); // Normal Link Pulse (NLP)
        // Transmission Control
        pub const POLSTAT = usize(0x00000010); // 10 Mb Polarity Status
        pub const JABBERD = usize(0x00000001); // Jabber Disable
        pub const SQUELCH_S = usize(9);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_BICSR1 register.
    //
    //*****************************************************************************
    pub const BICSR1 = struct {
        pub const ERRCNT_M = usize(0x0000FF00); // BIST Error Count
        pub const IPGLENGTH_M = usize(0x000000FF); // BIST IPG Length
        pub const ERRCNT_S = usize(8);
        pub const IPGLENGTH_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_BICSR2 register.
    //
    //*****************************************************************************
    pub const BICSR2 = struct {
        pub const PKTLENGTH_M = usize(0x000007FF); // BIST Packet Length
        pub const PKTLENGTH_S = usize(0);
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_CDCR register.
    //
    //*****************************************************************************
    pub const CDCR = struct {
        pub const START = usize(0x00008000); // Cable Diagnostic Process Start
        pub const LINKQUAL_M = usize(0x00000300); // Link Quality Indication
        pub const LINKQUAL_GOOD = usize(0x00000100); // Good Quality Link Indication
        pub const LINKQUAL_MILD = usize(0x00000200); // Mid- Quality Link Indication
        pub const LINKQUAL_POOR = usize(0x00000300); // Poor Quality Link Indication
        pub const DONE = usize(0x00000002); // Cable Diagnostic Process Done
        pub const FAIL = usize(0x00000001); // Cable Diagnostic Process Fail
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_RCR register.
    //
    //*****************************************************************************
    pub const RCR = struct {
        pub const SWRST = usize(0x00008000); // Software Reset
        pub const SWRESTART = usize(0x00004000); // Software Restart
    };

    //*****************************************************************************
    //
    // The following are defines for the bit fields in the EPHY_LEDCFG register.
    //
    //*****************************************************************************
    pub const LEDCFG = struct {
        pub const LED2_M = usize(0x00000F00); // LED2 Configuration
        pub const LED2_LINK = usize(0x00000000); // Link OK
        pub const LED2_RXTX = usize(0x00000100); // RX/TX Activity
        pub const LED2_TX = usize(0x00000200); // TX Activity
        pub const LED2_RX = usize(0x00000300); // RX Activity
        pub const LED2_COL = usize(0x00000400); // Collision
        pub const LED2_100BT = usize(0x00000500); // 100-Base TX
        pub const LED2_10BT = usize(0x00000600); // 10-Base TX
        pub const LED2_FD = usize(0x00000700); // Full Duplex
        pub const LED2_LINKTXRX = usize(0x00000800); // Link OK/Blink on TX/RX Activity
        pub const LED1_M = usize(0x000000F0); // LED1 Configuration
        pub const LED1_LINK = usize(0x00000000); // Link OK
        pub const LED1_RXTX = usize(0x00000010); // RX/TX Activity
        pub const LED1_TX = usize(0x00000020); // TX Activity
        pub const LED1_RX = usize(0x00000030); // RX Activity
        pub const LED1_COL = usize(0x00000040); // Collision
        pub const LED1_100BT = usize(0x00000050); // 100-Base TX
        pub const LED1_10BT = usize(0x00000060); // 10-Base TX
        pub const LED1_FD = usize(0x00000070); // Full Duplex
        pub const LED1_LINKTXRX = usize(0x00000080); // Link OK/Blink on TX/RX Activity
        pub const LED0_M = usize(0x0000000F); // LED0 Configuration
        pub const LED0_LINK = usize(0x00000000); // Link OK
        pub const LED0_RXTX = usize(0x00000001); // RX/TX Activity
        pub const LED0_TX = usize(0x00000002); // TX Activity
        pub const LED0_RX = usize(0x00000003); // RX Activity
        pub const LED0_COL = usize(0x00000004); // Collision
        pub const LED0_100BT = usize(0x00000005); // 100-Base TX
        pub const LED0_10BT = usize(0x00000006); // 10-Base TX
        pub const LED0_FD = usize(0x00000007); // Full Duplex
        pub const LED0_LINKTXRX = usize(0x00000008); // Link OK/Blink on TX/RX Activity
    };

    //*****************************************************************************
    //
    // The following definitions are deprecated.
    //
    //*****************************************************************************
};
