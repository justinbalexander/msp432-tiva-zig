//*****************************************************************************
//
// hw_can.h - Defines and macros used when accessing the CAN controllers.
//
// Copyright (c) 2006-2017 Texas Instruments Incorporated.  All rights reserved.
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
// The following are defines for the CAN register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const CTL = usize(0x00000000); // CAN Control
    pub const STS = usize(0x00000004); // CAN Status
    pub const ERR = usize(0x00000008); // CAN Error Counter
    pub const BIT = usize(0x0000000C); // CAN Bit Timing
    pub const INT = usize(0x00000010); // CAN Interrupt
    pub const TST = usize(0x00000014); // CAN Test
    pub const BRPE = usize(0x00000018); // CAN Baud Rate Prescaler
    // Extension
    pub const IF1CRQ = usize(0x00000020); // CAN IF1 Command Request
    pub const IF1CMSK = usize(0x00000024); // CAN IF1 Command Mask
    pub const IF1MSK1 = usize(0x00000028); // CAN IF1 Mask 1
    pub const IF1MSK2 = usize(0x0000002C); // CAN IF1 Mask 2
    pub const IF1ARB1 = usize(0x00000030); // CAN IF1 Arbitration 1
    pub const IF1ARB2 = usize(0x00000034); // CAN IF1 Arbitration 2
    pub const IF1MCTL = usize(0x00000038); // CAN IF1 Message Control
    pub const IF1DA1 = usize(0x0000003C); // CAN IF1 Data A1
    pub const IF1DA2 = usize(0x00000040); // CAN IF1 Data A2
    pub const IF1DB1 = usize(0x00000044); // CAN IF1 Data B1
    pub const IF1DB2 = usize(0x00000048); // CAN IF1 Data B2
    pub const IF2CRQ = usize(0x00000080); // CAN IF2 Command Request
    pub const IF2CMSK = usize(0x00000084); // CAN IF2 Command Mask
    pub const IF2MSK1 = usize(0x00000088); // CAN IF2 Mask 1
    pub const IF2MSK2 = usize(0x0000008C); // CAN IF2 Mask 2
    pub const IF2ARB1 = usize(0x00000090); // CAN IF2 Arbitration 1
    pub const IF2ARB2 = usize(0x00000094); // CAN IF2 Arbitration 2
    pub const IF2MCTL = usize(0x00000098); // CAN IF2 Message Control
    pub const IF2DA1 = usize(0x0000009C); // CAN IF2 Data A1
    pub const IF2DA2 = usize(0x000000A0); // CAN IF2 Data A2
    pub const IF2DB1 = usize(0x000000A4); // CAN IF2 Data B1
    pub const IF2DB2 = usize(0x000000A8); // CAN IF2 Data B2
    pub const TXRQ1 = usize(0x00000100); // CAN Transmission Request 1
    pub const TXRQ2 = usize(0x00000104); // CAN Transmission Request 2
    pub const NWDA1 = usize(0x00000120); // CAN New Data 1
    pub const NWDA2 = usize(0x00000124); // CAN New Data 2
    pub const MSG1INT = usize(0x00000140); // CAN Message 1 Interrupt Pending
    pub const MSG2INT = usize(0x00000144); // CAN Message 2 Interrupt Pending
    pub const MSG1VAL = usize(0x00000160); // CAN Message 1 Valid
    pub const MSG2VAL = usize(0x00000164); // CAN Message 2 Valid
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_CTL register.
//
//*****************************************************************************
pub const CTL = struct {
    pub const TEST = usize(0x00000080); // Test Mode Enable
    pub const CCE = usize(0x00000040); // Configuration Change Enable
    pub const DAR = usize(0x00000020); // Disable Automatic-Retransmission
    pub const EIE = usize(0x00000008); // Error Interrupt Enable
    pub const SIE = usize(0x00000004); // Status Interrupt Enable
    pub const IE = usize(0x00000002); // CAN Interrupt Enable
    pub const INIT = usize(0x00000001); // Initialization
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_STS register.
//
//*****************************************************************************
pub const STS = struct {
    pub const BOFF = usize(0x00000080); // Bus-Off Status
    pub const EWARN = usize(0x00000040); // Warning Status
    pub const EPASS = usize(0x00000020); // Error Passive
    pub const RXOK = usize(0x00000010); // Received a Message Successfully
    pub const TXOK = usize(0x00000008); // Transmitted a Message
    // Successfully
    pub const LEC_M = usize(0x00000007); // Last Error Code
    pub const LEC_NONE = usize(0x00000000); // No Error
    pub const LEC_STUFF = usize(0x00000001); // Stuff Error
    pub const LEC_FORM = usize(0x00000002); // Format Error
    pub const LEC_ACK = usize(0x00000003); // ACK Error
    pub const LEC_BIT1 = usize(0x00000004); // Bit 1 Error
    pub const LEC_BIT0 = usize(0x00000005); // Bit 0 Error
    pub const LEC_CRC = usize(0x00000006); // CRC Error
    pub const LEC_NOEVENT = usize(0x00000007); // No Event
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_ERR register.
//
//*****************************************************************************
pub const ERR = struct {
    pub const RP = usize(0x00008000); // Received Error Passive
    pub const REC_M = usize(0x00007F00); // Receive Error Counter
    pub const TEC_M = usize(0x000000FF); // Transmit Error Counter
    pub const REC_S = usize(8);
    pub const TEC_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_BIT register.
//
//*****************************************************************************
pub const BIT = struct {
    pub const TSEG2_M = usize(0x00007000); // Time Segment after Sample Point
    pub const TSEG1_M = usize(0x00000F00); // Time Segment Before Sample Point
    pub const SJW_M = usize(0x000000C0); // (Re)Synchronization Jump Width
    pub const BRP_M = usize(0x0000003F); // Baud Rate Prescaler
    pub const TSEG2_S = usize(12);
    pub const TSEG1_S = usize(8);
    pub const SJW_S = usize(6);
    pub const BRP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_INT register.
//
//*****************************************************************************
pub const INT = struct {
    pub const INTID_M = usize(0x0000FFFF); // Interrupt Identifier
    pub const INTID_NONE = usize(0x00000000); // No interrupt pending
    pub const INTID_STATUS = usize(0x00008000); // Status Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TST register.
//
//*****************************************************************************
pub const TST = struct {
    pub const RX = usize(0x00000080); // Receive Observation
    pub const TX_M = usize(0x00000060); // Transmit Control
    pub const TX_CANCTL = usize(0x00000000); // CAN Module Control
    pub const TX_SAMPLE = usize(0x00000020); // Sample Point
    pub const TX_DOMINANT = usize(0x00000040); // Driven Low
    pub const TX_RECESSIVE = usize(0x00000060); // Driven High
    pub const LBACK = usize(0x00000010); // Loopback Mode
    pub const SILENT = usize(0x00000008); // Silent Mode
    pub const BASIC = usize(0x00000004); // Basic Mode
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_BRPE register.
//
//*****************************************************************************
pub const BRPE = struct {
    pub const BRPE_M = usize(0x0000000F); // Baud Rate Prescaler Extension
    pub const BRPE_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1CRQ register.
//
//*****************************************************************************
pub const IF1CRQ = struct {
    pub const BUSY = usize(0x00008000); // Busy Flag
    pub const MNUM_M = usize(0x0000003F); // Message Number
    pub const MNUM_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1CMSK register.
//
//*****************************************************************************
pub const IF1CMSK = struct {
    pub const WRNRD = usize(0x00000080); // Write, Not Read
    pub const MASK = usize(0x00000040); // Access Mask Bits
    pub const ARB = usize(0x00000020); // Access Arbitration Bits
    pub const CONTROL = usize(0x00000010); // Access Control Bits
    pub const CLRINTPND = usize(0x00000008); // Clear Interrupt Pending Bit
    pub const NEWDAT = usize(0x00000004); // Access New Data
    pub const TXRQST = usize(0x00000004); // Access Transmission Request
    pub const DATAA = usize(0x00000002); // Access Data Byte 0 to 3
    pub const DATAB = usize(0x00000001); // Access Data Byte 4 to 7
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MSK1 register.
//
//*****************************************************************************
pub const IF1MSK1 = struct {
    pub const IDMSK_M = usize(0x0000FFFF); // Identifier Mask
    pub const IDMSK_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MSK2 register.
//
//*****************************************************************************
pub const IF1MSK2 = struct {
    pub const MXTD = usize(0x00008000); // Mask Extended Identifier
    pub const MDIR = usize(0x00004000); // Mask Message Direction
    pub const IDMSK_M = usize(0x00001FFF); // Identifier Mask
    pub const IDMSK_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1ARB1 register.
//
//*****************************************************************************
pub const IF1ARB1 = struct {
    pub const ID_M = usize(0x0000FFFF); // Message Identifier
    pub const ID_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1ARB2 register.
//
//*****************************************************************************
pub const IF1ARB2 = struct {
    pub const MSGVAL = usize(0x00008000); // Message Valid
    pub const XTD = usize(0x00004000); // Extended Identifier
    pub const DIR = usize(0x00002000); // Message Direction
    pub const ID_M = usize(0x00001FFF); // Message Identifier
    pub const ID_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MCTL register.
//
//*****************************************************************************
pub const IF1MCTL = struct {
    pub const NEWDAT = usize(0x00008000); // New Data
    pub const MSGLST = usize(0x00004000); // Message Lost
    pub const INTPND = usize(0x00002000); // Interrupt Pending
    pub const UMASK = usize(0x00001000); // Use Acceptance Mask
    pub const TXIE = usize(0x00000800); // Transmit Interrupt Enable
    pub const RXIE = usize(0x00000400); // Receive Interrupt Enable
    pub const RMTEN = usize(0x00000200); // Remote Enable
    pub const TXRQST = usize(0x00000100); // Transmit Request
    pub const EOB = usize(0x00000080); // End of Buffer
    pub const DLC_M = usize(0x0000000F); // Data Length Code
    pub const DLC_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DA1 register.
//
//*****************************************************************************
pub const IF1DA1 = struct {
    pub const DATA_M = usize(0x0000FFFF); // Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DA2 register.
//
//*****************************************************************************
pub const IF1DA2 = struct {
    pub const DATA_M = usize(0x0000FFFF); // Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DB1 register.
//
//*****************************************************************************
pub const IF1DB1 = struct {
    pub const DATA_M = usize(0x0000FFFF); // Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DB2 register.
//
//*****************************************************************************
pub const IF1DB2 = struct {
    pub const DATA_M = usize(0x0000FFFF); // Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2CRQ register.
//
//*****************************************************************************
pub const IF2CRQ = struct {
    pub const BUSY = usize(0x00008000); // Busy Flag
    pub const MNUM_M = usize(0x0000003F); // Message Number
    pub const MNUM_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2CMSK register.
//
//*****************************************************************************
pub const IF2CMSK = struct {
    pub const WRNRD = usize(0x00000080); // Write, Not Read
    pub const MASK = usize(0x00000040); // Access Mask Bits
    pub const ARB = usize(0x00000020); // Access Arbitration Bits
    pub const CONTROL = usize(0x00000010); // Access Control Bits
    pub const CLRINTPND = usize(0x00000008); // Clear Interrupt Pending Bit
    pub const NEWDAT = usize(0x00000004); // Access New Data
    pub const TXRQST = usize(0x00000004); // Access Transmission Request
    pub const DATAA = usize(0x00000002); // Access Data Byte 0 to 3
    pub const DATAB = usize(0x00000001); // Access Data Byte 4 to 7
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MSK1 register.
//
//*****************************************************************************
pub const IF2MSK1 = struct {
    pub const IDMSK_M = usize(0x0000FFFF); // Identifier Mask
    pub const IDMSK_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MSK2 register.
//
//*****************************************************************************
pub const IF2MSK2 = struct {
    pub const MXTD = usize(0x00008000); // Mask Extended Identifier
    pub const MDIR = usize(0x00004000); // Mask Message Direction
    pub const IDMSK_M = usize(0x00001FFF); // Identifier Mask
    pub const IDMSK_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2ARB1 register.
//
//*****************************************************************************
pub const IF2ARB1 = struct {
    pub const ID_M = usize(0x0000FFFF); // Message Identifier
    pub const ID_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2ARB2 register.
//
//*****************************************************************************
pub const IF2ARB2 = struct {
    pub const MSGVAL = usize(0x00008000); // Message Valid
    pub const XTD = usize(0x00004000); // Extended Identifier
    pub const DIR = usize(0x00002000); // Message Direction
    pub const ID_M = usize(0x00001FFF); // Message Identifier
    pub const ID_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MCTL register.
//
//*****************************************************************************
pub const IF2MCTL = struct {
    pub const NEWDAT = usize(0x00008000); // New Data
    pub const MSGLST = usize(0x00004000); // Message Lost
    pub const INTPND = usize(0x00002000); // Interrupt Pending
    pub const UMASK = usize(0x00001000); // Use Acceptance Mask
    pub const TXIE = usize(0x00000800); // Transmit Interrupt Enable
    pub const RXIE = usize(0x00000400); // Receive Interrupt Enable
    pub const RMTEN = usize(0x00000200); // Remote Enable
    pub const TXRQST = usize(0x00000100); // Transmit Request
    pub const EOB = usize(0x00000080); // End of Buffer
    pub const DLC_M = usize(0x0000000F); // Data Length Code
    pub const DLC_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DA1 register.
//
//*****************************************************************************
pub const IF2DA1 = struct {
    pub const DATA_M = usize(0x0000FFFF); // Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DA2 register.
//
//*****************************************************************************
pub const IF2DA2 = struct {
    pub const DATA_M = usize(0x0000FFFF); // Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DB1 register.
//
//*****************************************************************************
pub const IF2DB1 = struct {
    pub const DATA_M = usize(0x0000FFFF); // Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DB2 register.
//
//*****************************************************************************
pub const IF2DB2 = struct {
    pub const DATA_M = usize(0x0000FFFF); // Data
    pub const DATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TXRQ1 register.
//
//*****************************************************************************
pub const TXRQ1 = struct {
    pub const TXRQST_M = usize(0x0000FFFF); // Transmission Request Bits
    pub const TXRQST_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TXRQ2 register.
//
//*****************************************************************************
pub const TXRQ2 = struct {
    pub const TXRQST_M = usize(0x0000FFFF); // Transmission Request Bits
    pub const TXRQST_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_NWDA1 register.
//
//*****************************************************************************
pub const NWDA1 = struct {
    pub const NEWDAT_M = usize(0x0000FFFF); // New Data Bits
    pub const NEWDAT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_NWDA2 register.
//
//*****************************************************************************
pub const NWDA2 = struct {
    pub const NEWDAT_M = usize(0x0000FFFF); // New Data Bits
    pub const NEWDAT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG1INT register.
//
//*****************************************************************************
pub const MSG1INT = struct {
    pub const INTPND_M = usize(0x0000FFFF); // Interrupt Pending Bits
    pub const INTPND_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG2INT register.
//
//*****************************************************************************
pub const MSG2INT = struct {
    pub const INTPND_M = usize(0x0000FFFF); // Interrupt Pending Bits
    pub const INTPND_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG1VAL register.
//
//*****************************************************************************
pub const MSG1VAL = struct {
    pub const MSGVAL_M = usize(0x0000FFFF); // Message Valid Bits
    pub const MSGVAL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG2VAL register.
//
//*****************************************************************************
pub const MSG2VAL = struct {
    pub const MSGVAL_M = usize(0x0000FFFF); // Message Valid Bits
    pub const MSGVAL_S = usize(0);
};
