//*****************************************************************************
//
// hw_usb.h - Macros for use in accessing the USB registers.
//
// Copyright (c) 2007-2017 Texas Instruments Incorporated.  All rights reserved.
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
// The following are defines for the Univeral Serial Bus register offsets.
//
//*****************************************************************************
pub const offsetOf = struct {
    pub const FADDR = usize(0x00000000); // USB Device Functional Address
    pub const POWER = usize(0x00000001); // USB Power
    pub const TXIS = usize(0x00000002); // USB Transmit Interrupt Status
    pub const RXIS = usize(0x00000004); // USB Receive Interrupt Status
    pub const TXIE = usize(0x00000006); // USB Transmit Interrupt Enable
    pub const RXIE = usize(0x00000008); // USB Receive Interrupt Enable
    pub const IS = usize(0x0000000A); // USB General Interrupt Status
    pub const IE = usize(0x0000000B); // USB Interrupt Enable
    pub const FRAME = usize(0x0000000C); // USB Frame Value
    pub const EPIDX = usize(0x0000000E); // USB Endpoint Index
    pub const TEST = usize(0x0000000F); // USB Test Mode
    pub const FIFO0 = usize(0x00000020); // USB FIFO Endpoint 0
    pub const FIFO1 = usize(0x00000024); // USB FIFO Endpoint 1
    pub const FIFO2 = usize(0x00000028); // USB FIFO Endpoint 2
    pub const FIFO3 = usize(0x0000002C); // USB FIFO Endpoint 3
    pub const FIFO4 = usize(0x00000030); // USB FIFO Endpoint 4
    pub const FIFO5 = usize(0x00000034); // USB FIFO Endpoint 5
    pub const FIFO6 = usize(0x00000038); // USB FIFO Endpoint 6
    pub const FIFO7 = usize(0x0000003C); // USB FIFO Endpoint 7
    pub const DEVCTL = usize(0x00000060); // USB Device Control
    pub const CCONF = usize(0x00000061); // USB Common Configuration
    pub const TXFIFOSZ = usize(0x00000062); // USB Transmit Dynamic FIFO Sizing
    pub const RXFIFOSZ = usize(0x00000063); // USB Receive Dynamic FIFO Sizing
    pub const TXFIFOADD = usize(0x00000064); // USB Transmit FIFO Start Address
    pub const RXFIFOADD = usize(0x00000066); // USB Receive FIFO Start Address
    pub const ULPIVBUSCTL = usize(0x00000070); // USB ULPI VBUS Control
    pub const ULPIREGDATA = usize(0x00000074); // USB ULPI Register Data
    pub const ULPIREGADDR = usize(0x00000075); // USB ULPI Register Address
    pub const ULPIREGCTL = usize(0x00000076); // USB ULPI Register Control
    pub const EPINFO = usize(0x00000078); // USB Endpoint Information
    pub const RAMINFO = usize(0x00000079); // USB RAM Information
    pub const CONTIM = usize(0x0000007A); // USB Connect Timing
    pub const VPLEN = usize(0x0000007B); // USB OTG VBUS Pulse Timing
    pub const HSEOF = usize(0x0000007C); // USB High-Speed Last Transaction
    // to End of Frame Timing
    pub const FSEOF = usize(0x0000007D); // USB Full-Speed Last Transaction
    // to End of Frame Timing
    pub const LSEOF = usize(0x0000007E); // USB Low-Speed Last Transaction
    // to End of Frame Timing
    pub const TXFUNCADDR0 = usize(0x00000080); // USB Transmit Functional Address
    // Endpoint 0
    pub const TXHUBADDR0 = usize(0x00000082); // USB Transmit Hub Address
    // Endpoint 0
    pub const TXHUBPORT0 = usize(0x00000083); // USB Transmit Hub Port Endpoint 0
    pub const TXFUNCADDR1 = usize(0x00000088); // USB Transmit Functional Address
    // Endpoint 1
    pub const TXHUBADDR1 = usize(0x0000008A); // USB Transmit Hub Address
    // Endpoint 1
    pub const TXHUBPORT1 = usize(0x0000008B); // USB Transmit Hub Port Endpoint 1
    pub const RXFUNCADDR1 = usize(0x0000008C); // USB Receive Functional Address
    // Endpoint 1
    pub const RXHUBADDR1 = usize(0x0000008E); // USB Receive Hub Address Endpoint
    // 1
    pub const RXHUBPORT1 = usize(0x0000008F); // USB Receive Hub Port Endpoint 1
    pub const TXFUNCADDR2 = usize(0x00000090); // USB Transmit Functional Address
    // Endpoint 2
    pub const TXHUBADDR2 = usize(0x00000092); // USB Transmit Hub Address
    // Endpoint 2
    pub const TXHUBPORT2 = usize(0x00000093); // USB Transmit Hub Port Endpoint 2
    pub const RXFUNCADDR2 = usize(0x00000094); // USB Receive Functional Address
    // Endpoint 2
    pub const RXHUBADDR2 = usize(0x00000096); // USB Receive Hub Address Endpoint
    // 2
    pub const RXHUBPORT2 = usize(0x00000097); // USB Receive Hub Port Endpoint 2
    pub const TXFUNCADDR3 = usize(0x00000098); // USB Transmit Functional Address
    // Endpoint 3
    pub const TXHUBADDR3 = usize(0x0000009A); // USB Transmit Hub Address
    // Endpoint 3
    pub const TXHUBPORT3 = usize(0x0000009B); // USB Transmit Hub Port Endpoint 3
    pub const RXFUNCADDR3 = usize(0x0000009C); // USB Receive Functional Address
    // Endpoint 3
    pub const RXHUBADDR3 = usize(0x0000009E); // USB Receive Hub Address Endpoint
    // 3
    pub const RXHUBPORT3 = usize(0x0000009F); // USB Receive Hub Port Endpoint 3
    pub const TXFUNCADDR4 = usize(0x000000A0); // USB Transmit Functional Address
    // Endpoint 4
    pub const TXHUBADDR4 = usize(0x000000A2); // USB Transmit Hub Address
    // Endpoint 4
    pub const TXHUBPORT4 = usize(0x000000A3); // USB Transmit Hub Port Endpoint 4
    pub const RXFUNCADDR4 = usize(0x000000A4); // USB Receive Functional Address
    // Endpoint 4
    pub const RXHUBADDR4 = usize(0x000000A6); // USB Receive Hub Address Endpoint
    // 4
    pub const RXHUBPORT4 = usize(0x000000A7); // USB Receive Hub Port Endpoint 4
    pub const TXFUNCADDR5 = usize(0x000000A8); // USB Transmit Functional Address
    // Endpoint 5
    pub const TXHUBADDR5 = usize(0x000000AA); // USB Transmit Hub Address
    // Endpoint 5
    pub const TXHUBPORT5 = usize(0x000000AB); // USB Transmit Hub Port Endpoint 5
    pub const RXFUNCADDR5 = usize(0x000000AC); // USB Receive Functional Address
    // Endpoint 5
    pub const RXHUBADDR5 = usize(0x000000AE); // USB Receive Hub Address Endpoint
    // 5
    pub const RXHUBPORT5 = usize(0x000000AF); // USB Receive Hub Port Endpoint 5
    pub const TXFUNCADDR6 = usize(0x000000B0); // USB Transmit Functional Address
    // Endpoint 6
    pub const TXHUBADDR6 = usize(0x000000B2); // USB Transmit Hub Address
    // Endpoint 6
    pub const TXHUBPORT6 = usize(0x000000B3); // USB Transmit Hub Port Endpoint 6
    pub const RXFUNCADDR6 = usize(0x000000B4); // USB Receive Functional Address
    // Endpoint 6
    pub const RXHUBADDR6 = usize(0x000000B6); // USB Receive Hub Address Endpoint
    // 6
    pub const RXHUBPORT6 = usize(0x000000B7); // USB Receive Hub Port Endpoint 6
    pub const TXFUNCADDR7 = usize(0x000000B8); // USB Transmit Functional Address
    // Endpoint 7
    pub const TXHUBADDR7 = usize(0x000000BA); // USB Transmit Hub Address
    // Endpoint 7
    pub const TXHUBPORT7 = usize(0x000000BB); // USB Transmit Hub Port Endpoint 7
    pub const RXFUNCADDR7 = usize(0x000000BC); // USB Receive Functional Address
    // Endpoint 7
    pub const RXHUBADDR7 = usize(0x000000BE); // USB Receive Hub Address Endpoint
    // 7
    pub const RXHUBPORT7 = usize(0x000000BF); // USB Receive Hub Port Endpoint 7
    pub const CSRL0 = usize(0x00000102); // USB Control and Status Endpoint
    // 0 Low
    pub const CSRH0 = usize(0x00000103); // USB Control and Status Endpoint
    // 0 High
    pub const COUNT0 = usize(0x00000108); // USB Receive Byte Count Endpoint
    // 0
    pub const TYPE0 = usize(0x0000010A); // USB Type Endpoint 0
    pub const NAKLMT = usize(0x0000010B); // USB NAK Limit
    pub const TXMAXP1 = usize(0x00000110); // USB Maximum Transmit Data
    // Endpoint 1
    pub const TXCSRL1 = usize(0x00000112); // USB Transmit Control and Status
    // Endpoint 1 Low
    pub const TXCSRH1 = usize(0x00000113); // USB Transmit Control and Status
    // Endpoint 1 High
    pub const RXMAXP1 = usize(0x00000114); // USB Maximum Receive Data
    // Endpoint 1
    pub const RXCSRL1 = usize(0x00000116); // USB Receive Control and Status
    // Endpoint 1 Low
    pub const RXCSRH1 = usize(0x00000117); // USB Receive Control and Status
    // Endpoint 1 High
    pub const RXCOUNT1 = usize(0x00000118); // USB Receive Byte Count Endpoint
    // 1
    pub const TXTYPE1 = usize(0x0000011A); // USB Host Transmit Configure Type
    // Endpoint 1
    pub const TXINTERVAL1 = usize(0x0000011B); // USB Host Transmit Interval
    // Endpoint 1
    pub const RXTYPE1 = usize(0x0000011C); // USB Host Configure Receive Type
    // Endpoint 1
    pub const RXINTERVAL1 = usize(0x0000011D); // USB Host Receive Polling
    // Interval Endpoint 1
    pub const TXMAXP2 = usize(0x00000120); // USB Maximum Transmit Data
    // Endpoint 2
    pub const TXCSRL2 = usize(0x00000122); // USB Transmit Control and Status
    // Endpoint 2 Low
    pub const TXCSRH2 = usize(0x00000123); // USB Transmit Control and Status
    // Endpoint 2 High
    pub const RXMAXP2 = usize(0x00000124); // USB Maximum Receive Data
    // Endpoint 2
    pub const RXCSRL2 = usize(0x00000126); // USB Receive Control and Status
    // Endpoint 2 Low
    pub const RXCSRH2 = usize(0x00000127); // USB Receive Control and Status
    // Endpoint 2 High
    pub const RXCOUNT2 = usize(0x00000128); // USB Receive Byte Count Endpoint
    // 2
    pub const TXTYPE2 = usize(0x0000012A); // USB Host Transmit Configure Type
    // Endpoint 2
    pub const TXINTERVAL2 = usize(0x0000012B); // USB Host Transmit Interval
    // Endpoint 2
    pub const RXTYPE2 = usize(0x0000012C); // USB Host Configure Receive Type
    // Endpoint 2
    pub const RXINTERVAL2 = usize(0x0000012D); // USB Host Receive Polling
    // Interval Endpoint 2
    pub const TXMAXP3 = usize(0x00000130); // USB Maximum Transmit Data
    // Endpoint 3
    pub const TXCSRL3 = usize(0x00000132); // USB Transmit Control and Status
    // Endpoint 3 Low
    pub const TXCSRH3 = usize(0x00000133); // USB Transmit Control and Status
    // Endpoint 3 High
    pub const RXMAXP3 = usize(0x00000134); // USB Maximum Receive Data
    // Endpoint 3
    pub const RXCSRL3 = usize(0x00000136); // USB Receive Control and Status
    // Endpoint 3 Low
    pub const RXCSRH3 = usize(0x00000137); // USB Receive Control and Status
    // Endpoint 3 High
    pub const RXCOUNT3 = usize(0x00000138); // USB Receive Byte Count Endpoint
    // 3
    pub const TXTYPE3 = usize(0x0000013A); // USB Host Transmit Configure Type
    // Endpoint 3
    pub const TXINTERVAL3 = usize(0x0000013B); // USB Host Transmit Interval
    // Endpoint 3
    pub const RXTYPE3 = usize(0x0000013C); // USB Host Configure Receive Type
    // Endpoint 3
    pub const RXINTERVAL3 = usize(0x0000013D); // USB Host Receive Polling
    // Interval Endpoint 3
    pub const TXMAXP4 = usize(0x00000140); // USB Maximum Transmit Data
    // Endpoint 4
    pub const TXCSRL4 = usize(0x00000142); // USB Transmit Control and Status
    // Endpoint 4 Low
    pub const TXCSRH4 = usize(0x00000143); // USB Transmit Control and Status
    // Endpoint 4 High
    pub const RXMAXP4 = usize(0x00000144); // USB Maximum Receive Data
    // Endpoint 4
    pub const RXCSRL4 = usize(0x00000146); // USB Receive Control and Status
    // Endpoint 4 Low
    pub const RXCSRH4 = usize(0x00000147); // USB Receive Control and Status
    // Endpoint 4 High
    pub const RXCOUNT4 = usize(0x00000148); // USB Receive Byte Count Endpoint
    // 4
    pub const TXTYPE4 = usize(0x0000014A); // USB Host Transmit Configure Type
    // Endpoint 4
    pub const TXINTERVAL4 = usize(0x0000014B); // USB Host Transmit Interval
    // Endpoint 4
    pub const RXTYPE4 = usize(0x0000014C); // USB Host Configure Receive Type
    // Endpoint 4
    pub const RXINTERVAL4 = usize(0x0000014D); // USB Host Receive Polling
    // Interval Endpoint 4
    pub const TXMAXP5 = usize(0x00000150); // USB Maximum Transmit Data
    // Endpoint 5
    pub const TXCSRL5 = usize(0x00000152); // USB Transmit Control and Status
    // Endpoint 5 Low
    pub const TXCSRH5 = usize(0x00000153); // USB Transmit Control and Status
    // Endpoint 5 High
    pub const RXMAXP5 = usize(0x00000154); // USB Maximum Receive Data
    // Endpoint 5
    pub const RXCSRL5 = usize(0x00000156); // USB Receive Control and Status
    // Endpoint 5 Low
    pub const RXCSRH5 = usize(0x00000157); // USB Receive Control and Status
    // Endpoint 5 High
    pub const RXCOUNT5 = usize(0x00000158); // USB Receive Byte Count Endpoint
    // 5
    pub const TXTYPE5 = usize(0x0000015A); // USB Host Transmit Configure Type
    // Endpoint 5
    pub const TXINTERVAL5 = usize(0x0000015B); // USB Host Transmit Interval
    // Endpoint 5
    pub const RXTYPE5 = usize(0x0000015C); // USB Host Configure Receive Type
    // Endpoint 5
    pub const RXINTERVAL5 = usize(0x0000015D); // USB Host Receive Polling
    // Interval Endpoint 5
    pub const TXMAXP6 = usize(0x00000160); // USB Maximum Transmit Data
    // Endpoint 6
    pub const TXCSRL6 = usize(0x00000162); // USB Transmit Control and Status
    // Endpoint 6 Low
    pub const TXCSRH6 = usize(0x00000163); // USB Transmit Control and Status
    // Endpoint 6 High
    pub const RXMAXP6 = usize(0x00000164); // USB Maximum Receive Data
    // Endpoint 6
    pub const RXCSRL6 = usize(0x00000166); // USB Receive Control and Status
    // Endpoint 6 Low
    pub const RXCSRH6 = usize(0x00000167); // USB Receive Control and Status
    // Endpoint 6 High
    pub const RXCOUNT6 = usize(0x00000168); // USB Receive Byte Count Endpoint
    // 6
    pub const TXTYPE6 = usize(0x0000016A); // USB Host Transmit Configure Type
    // Endpoint 6
    pub const TXINTERVAL6 = usize(0x0000016B); // USB Host Transmit Interval
    // Endpoint 6
    pub const RXTYPE6 = usize(0x0000016C); // USB Host Configure Receive Type
    // Endpoint 6
    pub const RXINTERVAL6 = usize(0x0000016D); // USB Host Receive Polling
    // Interval Endpoint 6
    pub const TXMAXP7 = usize(0x00000170); // USB Maximum Transmit Data
    // Endpoint 7
    pub const TXCSRL7 = usize(0x00000172); // USB Transmit Control and Status
    // Endpoint 7 Low
    pub const TXCSRH7 = usize(0x00000173); // USB Transmit Control and Status
    // Endpoint 7 High
    pub const RXMAXP7 = usize(0x00000174); // USB Maximum Receive Data
    // Endpoint 7
    pub const RXCSRL7 = usize(0x00000176); // USB Receive Control and Status
    // Endpoint 7 Low
    pub const RXCSRH7 = usize(0x00000177); // USB Receive Control and Status
    // Endpoint 7 High
    pub const RXCOUNT7 = usize(0x00000178); // USB Receive Byte Count Endpoint
    // 7
    pub const TXTYPE7 = usize(0x0000017A); // USB Host Transmit Configure Type
    // Endpoint 7
    pub const TXINTERVAL7 = usize(0x0000017B); // USB Host Transmit Interval
    // Endpoint 7
    pub const RXTYPE7 = usize(0x0000017C); // USB Host Configure Receive Type
    // Endpoint 7
    pub const RXINTERVAL7 = usize(0x0000017D); // USB Host Receive Polling
    // Interval Endpoint 7
    pub const DMAINTR = usize(0x00000200); // USB DMA Interrupt
    pub const DMACTL0 = usize(0x00000204); // USB DMA Control 0
    pub const DMAADDR0 = usize(0x00000208); // USB DMA Address 0
    pub const DMACOUNT0 = usize(0x0000020C); // USB DMA Count 0
    pub const DMACTL1 = usize(0x00000214); // USB DMA Control 1
    pub const DMAADDR1 = usize(0x00000218); // USB DMA Address 1
    pub const DMACOUNT1 = usize(0x0000021C); // USB DMA Count 1
    pub const DMACTL2 = usize(0x00000224); // USB DMA Control 2
    pub const DMAADDR2 = usize(0x00000228); // USB DMA Address 2
    pub const DMACOUNT2 = usize(0x0000022C); // USB DMA Count 2
    pub const DMACTL3 = usize(0x00000234); // USB DMA Control 3
    pub const DMAADDR3 = usize(0x00000238); // USB DMA Address 3
    pub const DMACOUNT3 = usize(0x0000023C); // USB DMA Count 3
    pub const DMACTL4 = usize(0x00000244); // USB DMA Control 4
    pub const DMAADDR4 = usize(0x00000248); // USB DMA Address 4
    pub const DMACOUNT4 = usize(0x0000024C); // USB DMA Count 4
    pub const DMACTL5 = usize(0x00000254); // USB DMA Control 5
    pub const DMAADDR5 = usize(0x00000258); // USB DMA Address 5
    pub const DMACOUNT5 = usize(0x0000025C); // USB DMA Count 5
    pub const DMACTL6 = usize(0x00000264); // USB DMA Control 6
    pub const DMAADDR6 = usize(0x00000268); // USB DMA Address 6
    pub const DMACOUNT6 = usize(0x0000026C); // USB DMA Count 6
    pub const DMACTL7 = usize(0x00000274); // USB DMA Control 7
    pub const DMAADDR7 = usize(0x00000278); // USB DMA Address 7
    pub const DMACOUNT7 = usize(0x0000027C); // USB DMA Count 7
    pub const RQPKTCOUNT1 = usize(0x00000304); // USB Request Packet Count in
    // Block Transfer Endpoint 1
    pub const RQPKTCOUNT2 = usize(0x00000308); // USB Request Packet Count in
    // Block Transfer Endpoint 2
    pub const RQPKTCOUNT3 = usize(0x0000030C); // USB Request Packet Count in
    // Block Transfer Endpoint 3
    pub const RQPKTCOUNT4 = usize(0x00000310); // USB Request Packet Count in
    // Block Transfer Endpoint 4
    pub const RQPKTCOUNT5 = usize(0x00000314); // USB Request Packet Count in
    // Block Transfer Endpoint 5
    pub const RQPKTCOUNT6 = usize(0x00000318); // USB Request Packet Count in
    // Block Transfer Endpoint 6
    pub const RQPKTCOUNT7 = usize(0x0000031C); // USB Request Packet Count in
    // Block Transfer Endpoint 7
    pub const RXDPKTBUFDIS = usize(0x00000340); // USB Receive Double Packet Buffer
    // Disable
    pub const TXDPKTBUFDIS = usize(0x00000342); // USB Transmit Double Packet
    // Buffer Disable
    pub const CTO = usize(0x00000344); // USB Chirp Timeout
    pub const HHSRTN = usize(0x00000346); // USB High Speed to UTM Operating
    // Delay
    pub const HSBT = usize(0x00000348); // USB High Speed Time-out Adder
    pub const LPMATTR = usize(0x00000360); // USB LPM Attributes
    pub const LPMCNTRL = usize(0x00000362); // USB LPM Control
    pub const LPMIM = usize(0x00000363); // USB LPM Interrupt Mask
    pub const LPMRIS = usize(0x00000364); // USB LPM Raw Interrupt Status
    pub const LPMFADDR = usize(0x00000365); // USB LPM Function Address
    pub const EPC = usize(0x00000400); // USB External Power Control
    pub const EPCRIS = usize(0x00000404); // USB External Power Control Raw
    // Interrupt Status
    pub const EPCIM = usize(0x00000408); // USB External Power Control
    // Interrupt Mask
    pub const EPCISC = usize(0x0000040C); // USB External Power Control
    // Interrupt Status and Clear
    pub const DRRIS = usize(0x00000410); // USB Device RESUME Raw Interrupt
    // Status
    pub const DRIM = usize(0x00000414); // USB Device RESUME Interrupt Mask
    pub const DRISC = usize(0x00000418); // USB Device RESUME Interrupt
    // Status and Clear
    pub const GPCS = usize(0x0000041C); // USB General-Purpose Control and
    // Status
    pub const VDC = usize(0x00000430); // USB VBUS Droop Control
    pub const VDCRIS = usize(0x00000434); // USB VBUS Droop Control Raw
    // Interrupt Status
    pub const VDCIM = usize(0x00000438); // USB VBUS Droop Control Interrupt
    // Mask
    pub const VDCISC = usize(0x0000043C); // USB VBUS Droop Control Interrupt
    // Status and Clear
    pub const IDVRIS = usize(0x00000444); // USB ID Valid Detect Raw
    // Interrupt Status
    pub const IDVIM = usize(0x00000448); // USB ID Valid Detect Interrupt
    // Mask
    pub const IDVISC = usize(0x0000044C); // USB ID Valid Detect Interrupt
    // Status and Clear
    pub const DMASEL = usize(0x00000450); // USB DMA Select
    pub const PP = usize(0x00000FC0); // USB Peripheral Properties
    pub const PC = usize(0x00000FC4); // USB Peripheral Configuration
    pub const CC = usize(0x00000FC8); // USB Clock Configuration
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FADDR register.
//
//*****************************************************************************
pub const FADDR = struct {
    pub const M = usize(0x0000007F); // Function Address
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_POWER register.
//
//*****************************************************************************
pub const POWER = struct {
    pub const ISOUP = usize(0x00000080); // Isochronous Update
    pub const SOFTCONN = usize(0x00000040); // Soft Connect/Disconnect
    pub const HSENAB = usize(0x00000020); // High Speed Enable
    pub const HSMODE = usize(0x00000010); // High Speed Enable
    pub const RESET = usize(0x00000008); // RESET Signaling
    pub const RESUME = usize(0x00000004); // RESUME Signaling
    pub const SUSPEND = usize(0x00000002); // SUSPEND Mode
    pub const PWRDNPHY = usize(0x00000001); // Power Down PHY
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXIS register.
//
//*****************************************************************************
pub const TXIS = struct {
    pub const EP7 = usize(0x00000080); // TX Endpoint 7 Interrupt
    pub const EP6 = usize(0x00000040); // TX Endpoint 6 Interrupt
    pub const EP5 = usize(0x00000020); // TX Endpoint 5 Interrupt
    pub const EP4 = usize(0x00000010); // TX Endpoint 4 Interrupt
    pub const EP3 = usize(0x00000008); // TX Endpoint 3 Interrupt
    pub const EP2 = usize(0x00000004); // TX Endpoint 2 Interrupt
    pub const EP1 = usize(0x00000002); // TX Endpoint 1 Interrupt
    pub const EP0 = usize(0x00000001); // TX and RX Endpoint 0 Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXIS register.
//
//*****************************************************************************
pub const RXIS = struct {
    pub const EP7 = usize(0x00000080); // RX Endpoint 7 Interrupt
    pub const EP6 = usize(0x00000040); // RX Endpoint 6 Interrupt
    pub const EP5 = usize(0x00000020); // RX Endpoint 5 Interrupt
    pub const EP4 = usize(0x00000010); // RX Endpoint 4 Interrupt
    pub const EP3 = usize(0x00000008); // RX Endpoint 3 Interrupt
    pub const EP2 = usize(0x00000004); // RX Endpoint 2 Interrupt
    pub const EP1 = usize(0x00000002); // RX Endpoint 1 Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXIE register.
//
//*****************************************************************************
pub const TXIE = struct {
    pub const EP7 = usize(0x00000080); // TX Endpoint 7 Interrupt Enable
    pub const EP6 = usize(0x00000040); // TX Endpoint 6 Interrupt Enable
    pub const EP5 = usize(0x00000020); // TX Endpoint 5 Interrupt Enable
    pub const EP4 = usize(0x00000010); // TX Endpoint 4 Interrupt Enable
    pub const EP3 = usize(0x00000008); // TX Endpoint 3 Interrupt Enable
    pub const EP2 = usize(0x00000004); // TX Endpoint 2 Interrupt Enable
    pub const EP1 = usize(0x00000002); // TX Endpoint 1 Interrupt Enable
    pub const EP0 = usize(0x00000001); // TX and RX Endpoint 0 Interrupt
    // Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXIE register.
//
//*****************************************************************************
pub const RXIE = struct {
    pub const EP7 = usize(0x00000080); // RX Endpoint 7 Interrupt Enable
    pub const EP6 = usize(0x00000040); // RX Endpoint 6 Interrupt Enable
    pub const EP5 = usize(0x00000020); // RX Endpoint 5 Interrupt Enable
    pub const EP4 = usize(0x00000010); // RX Endpoint 4 Interrupt Enable
    pub const EP3 = usize(0x00000008); // RX Endpoint 3 Interrupt Enable
    pub const EP2 = usize(0x00000004); // RX Endpoint 2 Interrupt Enable
    pub const EP1 = usize(0x00000002); // RX Endpoint 1 Interrupt Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IS register.
//
//*****************************************************************************
pub const IS = struct {
    pub const VBUSERR = usize(0x00000080); // VBUS Error (OTG only)
    pub const SESREQ = usize(0x00000040); // SESSION REQUEST (OTG only)
    pub const DISCON = usize(0x00000020); // Session Disconnect (OTG only)
    pub const CONN = usize(0x00000010); // Session Connect
    pub const SOF = usize(0x00000008); // Start of Frame
    pub const BABBLE = usize(0x00000004); // Babble Detected
    pub const RESET = usize(0x00000004); // RESET Signaling Detected
    pub const RESUME = usize(0x00000002); // RESUME Signaling Detected
    pub const SUSPEND = usize(0x00000001); // SUSPEND Signaling Detected
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IE register.
//
//*****************************************************************************
pub const IE = struct {
    pub const VBUSERR = usize(0x00000080); // Enable VBUS Error Interrupt (OTG
    // only)
    pub const SESREQ = usize(0x00000040); // Enable Session Request (OTG
    // only)
    pub const DISCON = usize(0x00000020); // Enable Disconnect Interrupt
    pub const CONN = usize(0x00000010); // Enable Connect Interrupt
    pub const SOF = usize(0x00000008); // Enable Start-of-Frame Interrupt
    pub const BABBLE = usize(0x00000004); // Enable Babble Interrupt
    pub const RESET = usize(0x00000004); // Enable RESET Interrupt
    pub const RESUME = usize(0x00000002); // Enable RESUME Interrupt
    pub const SUSPND = usize(0x00000001); // Enable SUSPEND Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FRAME register.
//
//*****************************************************************************
pub const FRAME = struct {
    pub const M = usize(0x000007FF); // Frame Number
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPIDX register.
//
//*****************************************************************************
pub const EPIDX = struct {
    pub const EPIDX_M = usize(0x0000000F); // Endpoint Index
    pub const EPIDX_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TEST register.
//
//*****************************************************************************
pub const TEST = struct {
    pub const FORCEH = usize(0x00000080); // Force Host Mode
    pub const FIFOACC = usize(0x00000040); // FIFO Access
    pub const FORCEFS = usize(0x00000020); // Force Full-Speed Mode
    pub const FORCEHS = usize(0x00000010); // Force High-Speed Mode
    pub const TESTPKT = usize(0x00000008); // Test Packet Mode Enable
    pub const TESTK = usize(0x00000004); // Test_K Mode Enable
    pub const TESTJ = usize(0x00000002); // Test_J Mode Enable
    pub const TESTSE0NAK = usize(0x00000001); // Test_SE0_NAK Test Mode Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO0 register.
//
//*****************************************************************************
pub const FIFO0 = struct {
    pub const EPDATA_M = usize(0xFFFFFFFF); // Endpoint Data
    pub const EPDATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO1 register.
//
//*****************************************************************************
pub const FIFO1 = struct {
    pub const EPDATA_M = usize(0xFFFFFFFF); // Endpoint Data
    pub const EPDATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO2 register.
//
//*****************************************************************************
pub const FIFO2 = struct {
    pub const EPDATA_M = usize(0xFFFFFFFF); // Endpoint Data
    pub const EPDATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO3 register.
//
//*****************************************************************************
pub const FIFO3 = struct {
    pub const EPDATA_M = usize(0xFFFFFFFF); // Endpoint Data
    pub const EPDATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO4 register.
//
//*****************************************************************************
pub const FIFO4 = struct {
    pub const EPDATA_M = usize(0xFFFFFFFF); // Endpoint Data
    pub const EPDATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO5 register.
//
//*****************************************************************************
pub const FIFO5 = struct {
    pub const EPDATA_M = usize(0xFFFFFFFF); // Endpoint Data
    pub const EPDATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO6 register.
//
//*****************************************************************************
pub const FIFO6 = struct {
    pub const EPDATA_M = usize(0xFFFFFFFF); // Endpoint Data
    pub const EPDATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO7 register.
//
//*****************************************************************************
pub const FIFO7 = struct {
    pub const EPDATA_M = usize(0xFFFFFFFF); // Endpoint Data
    pub const EPDATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DEVCTL register.
//
//*****************************************************************************
pub const DEVCTL = struct {
    pub const DEV = usize(0x00000080); // Device Mode (OTG only)
    pub const FSDEV = usize(0x00000040); // Full-Speed Device Detected
    pub const LSDEV = usize(0x00000020); // Low-Speed Device Detected
    pub const VBUS_M = usize(0x00000018); // VBUS Level (OTG only)
    pub const VBUS_NONE = usize(0x00000000); // Below SessionEnd
    pub const VBUS_SEND = usize(0x00000008); // Above SessionEnd, below AValid
    pub const VBUS_AVALID = usize(0x00000010); // Above AValid, below VBUSValid
    pub const VBUS_VALID = usize(0x00000018); // Above VBUSValid
    pub const HOST = usize(0x00000004); // Host Mode
    pub const HOSTREQ = usize(0x00000002); // Host Request (OTG only)
    pub const SESSION = usize(0x00000001); // Session Start/End (OTG only)
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CCONF register.
//
//*****************************************************************************
pub const CCONF = struct {
    pub const TXEDMA = usize(0x00000002); // TX Early DMA Enable
    pub const RXEDMA = usize(0x00000001); // TX Early DMA Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFIFOSZ register.
//
//*****************************************************************************
pub const TXFIFOSZ = struct {
    pub const DPB = usize(0x00000010); // Double Packet Buffer Support
    pub const SIZE_M = usize(0x0000000F); // Max Packet Size
    pub const SIZE_8 = usize(0x00000000); // 8
    pub const SIZE_16 = usize(0x00000001); // 16
    pub const SIZE_32 = usize(0x00000002); // 32
    pub const SIZE_64 = usize(0x00000003); // 64
    pub const SIZE_128 = usize(0x00000004); // 128
    pub const SIZE_256 = usize(0x00000005); // 256
    pub const SIZE_512 = usize(0x00000006); // 512
    pub const SIZE_1024 = usize(0x00000007); // 1024
    pub const SIZE_2048 = usize(0x00000008); // 2048
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFIFOSZ register.
//
//*****************************************************************************
pub const RXFIFOSZ = struct {
    pub const DPB = usize(0x00000010); // Double Packet Buffer Support
    pub const SIZE_M = usize(0x0000000F); // Max Packet Size
    pub const SIZE_8 = usize(0x00000000); // 8
    pub const SIZE_16 = usize(0x00000001); // 16
    pub const SIZE_32 = usize(0x00000002); // 32
    pub const SIZE_64 = usize(0x00000003); // 64
    pub const SIZE_128 = usize(0x00000004); // 128
    pub const SIZE_256 = usize(0x00000005); // 256
    pub const SIZE_512 = usize(0x00000006); // 512
    pub const SIZE_1024 = usize(0x00000007); // 1024
    pub const SIZE_2048 = usize(0x00000008); // 2048
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFIFOADD
// register.
//
//*****************************************************************************
pub const TXFIFOADD = struct {
    pub const ADDR_M = usize(0x000001FF); // Transmit/Receive Start Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFIFOADD
// register.
//
//*****************************************************************************
pub const RXFIFOADD = struct {
    pub const ADDR_M = usize(0x000001FF); // Transmit/Receive Start Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_ULPIVBUSCTL
// register.
//
//*****************************************************************************
pub const ULPIVBUSCTL = struct {
    pub const USEEXTVBUSIND = usize(0x00000002); // Use External VBUS Indicator
    pub const USEEXTVBUS = usize(0x00000001); // Use External VBUS
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_ULPIREGDATA
// register.
//
//*****************************************************************************
pub const ULPIREGDATA = struct {
    pub const REGDATA_M = usize(0x000000FF); // Register Data
    pub const REGDATA_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_ULPIREGADDR
// register.
//
//*****************************************************************************
pub const ULPIREGADDR = struct {
    pub const ADDR_M = usize(0x000000FF); // Register Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_ULPIREGCTL
// register.
//
//*****************************************************************************
pub const ULPIREGCTL = struct {
    pub const RDWR = usize(0x00000004); // Read/Write Control
    pub const REGCMPLT = usize(0x00000002); // Register Access Complete
    pub const REGACC = usize(0x00000001); // Initiate Register Access
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPINFO register.
//
//*****************************************************************************
pub const EPINFO = struct {
    pub const RXEP_M = usize(0x000000F0); // RX Endpoints
    pub const TXEP_M = usize(0x0000000F); // TX Endpoints
    pub const RXEP_S = usize(4);
    pub const TXEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RAMINFO register.
//
//*****************************************************************************
pub const RAMINFO = struct {
    pub const DMACHAN_M = usize(0x000000F0); // DMA Channels
    pub const RAMBITS_M = usize(0x0000000F); // RAM Address Bus Width
    pub const DMACHAN_S = usize(4);
    pub const RAMBITS_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CONTIM register.
//
//*****************************************************************************
pub const CONTIM = struct {
    pub const WTCON_M = usize(0x000000F0); // Connect Wait
    pub const WTID_M = usize(0x0000000F); // Wait ID
    pub const WTCON_S = usize(4);
    pub const WTID_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VPLEN register.
//
//*****************************************************************************
pub const VPLEN = struct {
    pub const VPLEN_M = usize(0x000000FF); // VBUS Pulse Length
    pub const VPLEN_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_HSEOF register.
//
//*****************************************************************************
pub const HSEOF = struct {
    pub const HSEOFG_M = usize(0x000000FF); // HIgh-Speed End-of-Frame Gap
    pub const HSEOFG_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FSEOF register.
//
//*****************************************************************************
pub const FSEOF = struct {
    pub const FSEOFG_M = usize(0x000000FF); // Full-Speed End-of-Frame Gap
    pub const FSEOFG_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LSEOF register.
//
//*****************************************************************************
pub const LSEOF = struct {
    pub const LSEOFG_M = usize(0x000000FF); // Low-Speed End-of-Frame Gap
    pub const LSEOFG_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR0
// register.
//
//*****************************************************************************
pub const TXFUNCADDR0 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR0
// register.
//
//*****************************************************************************
pub const TXHUBADDR0 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT0
// register.
//
//*****************************************************************************
pub const TXHUBPORT0 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR1
// register.
//
//*****************************************************************************
pub const TXFUNCADDR1 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR1
// register.
//
//*****************************************************************************
pub const TXHUBADDR1 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT1
// register.
//
//*****************************************************************************
pub const TXHUBPORT1 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR1
// register.
//
//*****************************************************************************
pub const RXFUNCADDR1 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR1
// register.
//
//*****************************************************************************
pub const RXHUBADDR1 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT1
// register.
//
//*****************************************************************************
pub const RXHUBPORT1 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR2
// register.
//
//*****************************************************************************
pub const TXFUNCADDR2 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR2
// register.
//
//*****************************************************************************
pub const TXHUBADDR2 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT2
// register.
//
//*****************************************************************************
pub const TXHUBPORT2 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR2
// register.
//
//*****************************************************************************
pub const RXFUNCADDR2 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR2
// register.
//
//*****************************************************************************
pub const RXHUBADDR2 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT2
// register.
//
//*****************************************************************************
pub const RXHUBPORT2 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR3
// register.
//
//*****************************************************************************
pub const TXFUNCADDR3 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR3
// register.
//
//*****************************************************************************
pub const TXHUBADDR3 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT3
// register.
//
//*****************************************************************************
pub const TXHUBPORT3 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR3
// register.
//
//*****************************************************************************
pub const RXFUNCADDR3 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR3
// register.
//
//*****************************************************************************
pub const RXHUBADDR3 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT3
// register.
//
//*****************************************************************************
pub const RXHUBPORT3 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR4
// register.
//
//*****************************************************************************
pub const TXFUNCADDR4 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR4
// register.
//
//*****************************************************************************
pub const TXHUBADDR4 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT4
// register.
//
//*****************************************************************************
pub const TXHUBPORT4 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR4
// register.
//
//*****************************************************************************
pub const RXFUNCADDR4 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR4
// register.
//
//*****************************************************************************
pub const RXHUBADDR4 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT4
// register.
//
//*****************************************************************************
pub const RXHUBPORT4 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR5
// register.
//
//*****************************************************************************
pub const TXFUNCADDR5 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR5
// register.
//
//*****************************************************************************
pub const TXHUBADDR5 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT5
// register.
//
//*****************************************************************************
pub const TXHUBPORT5 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR5
// register.
//
//*****************************************************************************
pub const RXFUNCADDR5 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR5
// register.
//
//*****************************************************************************
pub const RXHUBADDR5 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT5
// register.
//
//*****************************************************************************
pub const RXHUBPORT5 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR6
// register.
//
//*****************************************************************************
pub const TXFUNCADDR6 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR6
// register.
//
//*****************************************************************************
pub const TXHUBADDR6 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT6
// register.
//
//*****************************************************************************
pub const TXHUBPORT6 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR6
// register.
//
//*****************************************************************************
pub const RXFUNCADDR6 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR6
// register.
//
//*****************************************************************************
pub const RXHUBADDR6 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT6
// register.
//
//*****************************************************************************
pub const RXHUBPORT6 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR7
// register.
//
//*****************************************************************************
pub const TXFUNCADDR7 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR7
// register.
//
//*****************************************************************************
pub const TXHUBADDR7 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT7
// register.
//
//*****************************************************************************
pub const TXHUBPORT7 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR7
// register.
//
//*****************************************************************************
pub const RXFUNCADDR7 = struct {
    pub const ADDR_M = usize(0x0000007F); // Device Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR7
// register.
//
//*****************************************************************************
pub const RXHUBADDR7 = struct {
    pub const ADDR_M = usize(0x0000007F); // Hub Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT7
// register.
//
//*****************************************************************************
pub const RXHUBPORT7 = struct {
    pub const PORT_M = usize(0x0000007F); // Hub Port
    pub const PORT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CSRL0 register.
//
//*****************************************************************************
pub const CSRL0 = struct {
    pub const NAKTO = usize(0x00000080); // NAK Timeout
    pub const SETENDC = usize(0x00000080); // Setup End Clear
    pub const STATUS = usize(0x00000040); // STATUS Packet
    pub const RXRDYC = usize(0x00000040); // RXRDY Clear
    pub const REQPKT = usize(0x00000020); // Request Packet
    pub const STALL = usize(0x00000020); // Send Stall
    pub const SETEND = usize(0x00000010); // Setup End
    pub const ERROR = usize(0x00000010); // Error
    pub const DATAEND = usize(0x00000008); // Data End
    pub const SETUP = usize(0x00000008); // Setup Packet
    pub const STALLED = usize(0x00000004); // Endpoint Stalled
    pub const TXRDY = usize(0x00000002); // Transmit Packet Ready
    pub const RXRDY = usize(0x00000001); // Receive Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CSRH0 register.
//
//*****************************************************************************
pub const CSRH0 = struct {
    pub const DISPING = usize(0x00000008); // PING Disable
    pub const DTWE = usize(0x00000004); // Data Toggle Write Enable
    pub const DT = usize(0x00000002); // Data Toggle
    pub const FLUSH = usize(0x00000001); // Flush FIFO
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_COUNT0 register.
//
//*****************************************************************************
pub const COUNT0 = struct {
    pub const COUNT_M = usize(0x0000007F); // FIFO Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TYPE0 register.
//
//*****************************************************************************
pub const TYPE0 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_NAKLMT register.
//
//*****************************************************************************
pub const NAKLMT = struct {
    pub const NAKLMT_M = usize(0x0000001F); // EP0 NAK Limit
    pub const NAKLMT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP1 register.
//
//*****************************************************************************
pub const TXMAXP1 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL1 register.
//
//*****************************************************************************
pub const TXCSRL1 = struct {
    pub const NAKTO = usize(0x00000080); // NAK Timeout
    pub const CLRDT = usize(0x00000040); // Clear Data Toggle
    pub const STALLED = usize(0x00000020); // Endpoint Stalled
    pub const STALL = usize(0x00000010); // Send STALL
    pub const SETUP = usize(0x00000010); // Setup Packet
    pub const FLUSH = usize(0x00000008); // Flush FIFO
    pub const ERROR = usize(0x00000004); // Error
    pub const UNDRN = usize(0x00000004); // Underrun
    pub const FIFONE = usize(0x00000002); // FIFO Not Empty
    pub const TXRDY = usize(0x00000001); // Transmit Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH1 register.
//
//*****************************************************************************
pub const TXCSRH1 = struct {
    pub const AUTOSET = usize(0x00000080); // Auto Set
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const MODE = usize(0x00000020); // Mode
    pub const DMAEN = usize(0x00000010); // DMA Request Enable
    pub const FDT = usize(0x00000008); // Force Data Toggle
    pub const DMAMOD = usize(0x00000004); // DMA Request Mode
    pub const DTWE = usize(0x00000002); // Data Toggle Write Enable
    pub const DT = usize(0x00000001); // Data Toggle
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP1 register.
//
//*****************************************************************************
pub const RXMAXP1 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL1 register.
//
//*****************************************************************************
pub const RXCSRL1 = struct {
    pub const CLRDT = usize(0x00000080); // Clear Data Toggle
    pub const STALLED = usize(0x00000040); // Endpoint Stalled
    pub const STALL = usize(0x00000020); // Send STALL
    pub const REQPKT = usize(0x00000020); // Request Packet
    pub const FLUSH = usize(0x00000010); // Flush FIFO
    pub const DATAERR = usize(0x00000008); // Data Error
    pub const NAKTO = usize(0x00000008); // NAK Timeout
    pub const OVER = usize(0x00000004); // Overrun
    pub const ERROR = usize(0x00000004); // Error
    pub const FULL = usize(0x00000002); // FIFO Full
    pub const RXRDY = usize(0x00000001); // Receive Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH1 register.
//
//*****************************************************************************
pub const RXCSRH1 = struct {
    pub const AUTOCL = usize(0x00000080); // Auto Clear
    pub const AUTORQ = usize(0x00000040); // Auto Request
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const DMAEN = usize(0x00000020); // DMA Request Enable
    pub const DISNYET = usize(0x00000010); // Disable NYET
    pub const PIDERR = usize(0x00000010); // PID Error
    pub const DMAMOD = usize(0x00000008); // DMA Request Mode
    pub const DTWE = usize(0x00000004); // Data Toggle Write Enable
    pub const DT = usize(0x00000002); // Data Toggle
    pub const INCOMPRX = usize(0x00000001); // Incomplete RX Transmission
    // Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT1 register.
//
//*****************************************************************************
pub const RXCOUNT1 = struct {
    pub const COUNT_M = usize(0x00001FFF); // Receive Packet Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE1 register.
//
//*****************************************************************************
pub const TXTYPE1 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL1
// register.
//
//*****************************************************************************
pub const TXINTERVAL1 = struct {
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const TXPOLL_M = usize(0x000000FF); // TX Polling
    pub const TXPOLL_S = usize(0);
    pub const NAKLMT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE1 register.
//
//*****************************************************************************
pub const RXTYPE1 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL1
// register.
//
//*****************************************************************************
pub const RXINTERVAL1 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // RX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const TXPOLL_S = usize(0);
    pub const NAKLMT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP2 register.
//
//*****************************************************************************
pub const TXMAXP2 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL2 register.
//
//*****************************************************************************
pub const TXCSRL2 = struct {
    pub const NAKTO = usize(0x00000080); // NAK Timeout
    pub const CLRDT = usize(0x00000040); // Clear Data Toggle
    pub const STALLED = usize(0x00000020); // Endpoint Stalled
    pub const SETUP = usize(0x00000010); // Setup Packet
    pub const STALL = usize(0x00000010); // Send STALL
    pub const FLUSH = usize(0x00000008); // Flush FIFO
    pub const ERROR = usize(0x00000004); // Error
    pub const UNDRN = usize(0x00000004); // Underrun
    pub const FIFONE = usize(0x00000002); // FIFO Not Empty
    pub const TXRDY = usize(0x00000001); // Transmit Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH2 register.
//
//*****************************************************************************
pub const TXCSRH2 = struct {
    pub const AUTOSET = usize(0x00000080); // Auto Set
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const MODE = usize(0x00000020); // Mode
    pub const DMAEN = usize(0x00000010); // DMA Request Enable
    pub const FDT = usize(0x00000008); // Force Data Toggle
    pub const DMAMOD = usize(0x00000004); // DMA Request Mode
    pub const DTWE = usize(0x00000002); // Data Toggle Write Enable
    pub const DT = usize(0x00000001); // Data Toggle
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP2 register.
//
//*****************************************************************************
pub const RXMAXP2 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL2 register.
//
//*****************************************************************************
pub const RXCSRL2 = struct {
    pub const CLRDT = usize(0x00000080); // Clear Data Toggle
    pub const STALLED = usize(0x00000040); // Endpoint Stalled
    pub const REQPKT = usize(0x00000020); // Request Packet
    pub const STALL = usize(0x00000020); // Send STALL
    pub const FLUSH = usize(0x00000010); // Flush FIFO
    pub const DATAERR = usize(0x00000008); // Data Error
    pub const NAKTO = usize(0x00000008); // NAK Timeout
    pub const ERROR = usize(0x00000004); // Error
    pub const OVER = usize(0x00000004); // Overrun
    pub const FULL = usize(0x00000002); // FIFO Full
    pub const RXRDY = usize(0x00000001); // Receive Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH2 register.
//
//*****************************************************************************
pub const RXCSRH2 = struct {
    pub const AUTOCL = usize(0x00000080); // Auto Clear
    pub const AUTORQ = usize(0x00000040); // Auto Request
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const DMAEN = usize(0x00000020); // DMA Request Enable
    pub const DISNYET = usize(0x00000010); // Disable NYET
    pub const PIDERR = usize(0x00000010); // PID Error
    pub const DMAMOD = usize(0x00000008); // DMA Request Mode
    pub const DTWE = usize(0x00000004); // Data Toggle Write Enable
    pub const DT = usize(0x00000002); // Data Toggle
    pub const INCOMPRX = usize(0x00000001); // Incomplete RX Transmission
    // Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT2 register.
//
//*****************************************************************************
pub const RXCOUNT2 = struct {
    pub const COUNT_M = usize(0x00001FFF); // Receive Packet Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE2 register.
//
//*****************************************************************************
pub const TXTYPE2 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL2
// register.
//
//*****************************************************************************
pub const TXINTERVAL2 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // TX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const NAKLMT_S = usize(0);
    pub const TXPOLL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE2 register.
//
//*****************************************************************************
pub const RXTYPE2 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL2
// register.
//
//*****************************************************************************
pub const RXINTERVAL2 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // RX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const TXPOLL_S = usize(0);
    pub const NAKLMT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP3 register.
//
//*****************************************************************************
pub const TXMAXP3 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL3 register.
//
//*****************************************************************************
pub const TXCSRL3 = struct {
    pub const NAKTO = usize(0x00000080); // NAK Timeout
    pub const CLRDT = usize(0x00000040); // Clear Data Toggle
    pub const STALLED = usize(0x00000020); // Endpoint Stalled
    pub const SETUP = usize(0x00000010); // Setup Packet
    pub const STALL = usize(0x00000010); // Send STALL
    pub const FLUSH = usize(0x00000008); // Flush FIFO
    pub const ERROR = usize(0x00000004); // Error
    pub const UNDRN = usize(0x00000004); // Underrun
    pub const FIFONE = usize(0x00000002); // FIFO Not Empty
    pub const TXRDY = usize(0x00000001); // Transmit Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH3 register.
//
//*****************************************************************************
pub const TXCSRH3 = struct {
    pub const AUTOSET = usize(0x00000080); // Auto Set
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const MODE = usize(0x00000020); // Mode
    pub const DMAEN = usize(0x00000010); // DMA Request Enable
    pub const FDT = usize(0x00000008); // Force Data Toggle
    pub const DMAMOD = usize(0x00000004); // DMA Request Mode
    pub const DTWE = usize(0x00000002); // Data Toggle Write Enable
    pub const DT = usize(0x00000001); // Data Toggle
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP3 register.
//
//*****************************************************************************
pub const RXMAXP3 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL3 register.
//
//*****************************************************************************
pub const RXCSRL3 = struct {
    pub const CLRDT = usize(0x00000080); // Clear Data Toggle
    pub const STALLED = usize(0x00000040); // Endpoint Stalled
    pub const STALL = usize(0x00000020); // Send STALL
    pub const REQPKT = usize(0x00000020); // Request Packet
    pub const FLUSH = usize(0x00000010); // Flush FIFO
    pub const DATAERR = usize(0x00000008); // Data Error
    pub const NAKTO = usize(0x00000008); // NAK Timeout
    pub const ERROR = usize(0x00000004); // Error
    pub const OVER = usize(0x00000004); // Overrun
    pub const FULL = usize(0x00000002); // FIFO Full
    pub const RXRDY = usize(0x00000001); // Receive Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH3 register.
//
//*****************************************************************************
pub const RXCSRH3 = struct {
    pub const AUTOCL = usize(0x00000080); // Auto Clear
    pub const AUTORQ = usize(0x00000040); // Auto Request
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const DMAEN = usize(0x00000020); // DMA Request Enable
    pub const DISNYET = usize(0x00000010); // Disable NYET
    pub const PIDERR = usize(0x00000010); // PID Error
    pub const DMAMOD = usize(0x00000008); // DMA Request Mode
    pub const DTWE = usize(0x00000004); // Data Toggle Write Enable
    pub const DT = usize(0x00000002); // Data Toggle
    pub const INCOMPRX = usize(0x00000001); // Incomplete RX Transmission
    // Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT3 register.
//
//*****************************************************************************
pub const RXCOUNT3 = struct {
    pub const COUNT_M = usize(0x00001FFF); // Receive Packet Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE3 register.
//
//*****************************************************************************
pub const TXTYPE3 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL3
// register.
//
//*****************************************************************************
pub const TXINTERVAL3 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // TX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const TXPOLL_S = usize(0);
    pub const NAKLMT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE3 register.
//
//*****************************************************************************
pub const RXTYPE3 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL3
// register.
//
//*****************************************************************************
pub const RXINTERVAL3 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // RX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const TXPOLL_S = usize(0);
    pub const NAKLMT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP4 register.
//
//*****************************************************************************
pub const TXMAXP4 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL4 register.
//
//*****************************************************************************
pub const TXCSRL4 = struct {
    pub const NAKTO = usize(0x00000080); // NAK Timeout
    pub const CLRDT = usize(0x00000040); // Clear Data Toggle
    pub const STALLED = usize(0x00000020); // Endpoint Stalled
    pub const SETUP = usize(0x00000010); // Setup Packet
    pub const STALL = usize(0x00000010); // Send STALL
    pub const FLUSH = usize(0x00000008); // Flush FIFO
    pub const ERROR = usize(0x00000004); // Error
    pub const UNDRN = usize(0x00000004); // Underrun
    pub const FIFONE = usize(0x00000002); // FIFO Not Empty
    pub const TXRDY = usize(0x00000001); // Transmit Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH4 register.
//
//*****************************************************************************
pub const TXCSRH4 = struct {
    pub const AUTOSET = usize(0x00000080); // Auto Set
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const MODE = usize(0x00000020); // Mode
    pub const DMAEN = usize(0x00000010); // DMA Request Enable
    pub const FDT = usize(0x00000008); // Force Data Toggle
    pub const DMAMOD = usize(0x00000004); // DMA Request Mode
    pub const DTWE = usize(0x00000002); // Data Toggle Write Enable
    pub const DT = usize(0x00000001); // Data Toggle
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP4 register.
//
//*****************************************************************************
pub const RXMAXP4 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL4 register.
//
//*****************************************************************************
pub const RXCSRL4 = struct {
    pub const CLRDT = usize(0x00000080); // Clear Data Toggle
    pub const STALLED = usize(0x00000040); // Endpoint Stalled
    pub const STALL = usize(0x00000020); // Send STALL
    pub const REQPKT = usize(0x00000020); // Request Packet
    pub const FLUSH = usize(0x00000010); // Flush FIFO
    pub const NAKTO = usize(0x00000008); // NAK Timeout
    pub const DATAERR = usize(0x00000008); // Data Error
    pub const OVER = usize(0x00000004); // Overrun
    pub const ERROR = usize(0x00000004); // Error
    pub const FULL = usize(0x00000002); // FIFO Full
    pub const RXRDY = usize(0x00000001); // Receive Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH4 register.
//
//*****************************************************************************
pub const RXCSRH4 = struct {
    pub const AUTOCL = usize(0x00000080); // Auto Clear
    pub const AUTORQ = usize(0x00000040); // Auto Request
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const DMAEN = usize(0x00000020); // DMA Request Enable
    pub const DISNYET = usize(0x00000010); // Disable NYET
    pub const PIDERR = usize(0x00000010); // PID Error
    pub const DMAMOD = usize(0x00000008); // DMA Request Mode
    pub const DTWE = usize(0x00000004); // Data Toggle Write Enable
    pub const DT = usize(0x00000002); // Data Toggle
    pub const INCOMPRX = usize(0x00000001); // Incomplete RX Transmission
    // Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT4 register.
//
//*****************************************************************************
pub const RXCOUNT4 = struct {
    pub const COUNT_M = usize(0x00001FFF); // Receive Packet Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE4 register.
//
//*****************************************************************************
pub const TXTYPE4 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL4
// register.
//
//*****************************************************************************
pub const TXINTERVAL4 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // TX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const NAKLMT_S = usize(0);
    pub const TXPOLL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE4 register.
//
//*****************************************************************************
pub const RXTYPE4 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL4
// register.
//
//*****************************************************************************
pub const RXINTERVAL4 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // RX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const NAKLMT_S = usize(0);
    pub const TXPOLL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP5 register.
//
//*****************************************************************************
pub const TXMAXP5 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL5 register.
//
//*****************************************************************************
pub const TXCSRL5 = struct {
    pub const NAKTO = usize(0x00000080); // NAK Timeout
    pub const CLRDT = usize(0x00000040); // Clear Data Toggle
    pub const STALLED = usize(0x00000020); // Endpoint Stalled
    pub const SETUP = usize(0x00000010); // Setup Packet
    pub const STALL = usize(0x00000010); // Send STALL
    pub const FLUSH = usize(0x00000008); // Flush FIFO
    pub const ERROR = usize(0x00000004); // Error
    pub const UNDRN = usize(0x00000004); // Underrun
    pub const FIFONE = usize(0x00000002); // FIFO Not Empty
    pub const TXRDY = usize(0x00000001); // Transmit Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH5 register.
//
//*****************************************************************************
pub const TXCSRH5 = struct {
    pub const AUTOSET = usize(0x00000080); // Auto Set
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const MODE = usize(0x00000020); // Mode
    pub const DMAEN = usize(0x00000010); // DMA Request Enable
    pub const FDT = usize(0x00000008); // Force Data Toggle
    pub const DMAMOD = usize(0x00000004); // DMA Request Mode
    pub const DTWE = usize(0x00000002); // Data Toggle Write Enable
    pub const DT = usize(0x00000001); // Data Toggle
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP5 register.
//
//*****************************************************************************
pub const RXMAXP5 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL5 register.
//
//*****************************************************************************
pub const RXCSRL5 = struct {
    pub const CLRDT = usize(0x00000080); // Clear Data Toggle
    pub const STALLED = usize(0x00000040); // Endpoint Stalled
    pub const STALL = usize(0x00000020); // Send STALL
    pub const REQPKT = usize(0x00000020); // Request Packet
    pub const FLUSH = usize(0x00000010); // Flush FIFO
    pub const NAKTO = usize(0x00000008); // NAK Timeout
    pub const DATAERR = usize(0x00000008); // Data Error
    pub const ERROR = usize(0x00000004); // Error
    pub const OVER = usize(0x00000004); // Overrun
    pub const FULL = usize(0x00000002); // FIFO Full
    pub const RXRDY = usize(0x00000001); // Receive Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH5 register.
//
//*****************************************************************************
pub const RXCSRH5 = struct {
    pub const AUTOCL = usize(0x00000080); // Auto Clear
    pub const AUTORQ = usize(0x00000040); // Auto Request
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const DMAEN = usize(0x00000020); // DMA Request Enable
    pub const DISNYET = usize(0x00000010); // Disable NYET
    pub const PIDERR = usize(0x00000010); // PID Error
    pub const DMAMOD = usize(0x00000008); // DMA Request Mode
    pub const DTWE = usize(0x00000004); // Data Toggle Write Enable
    pub const DT = usize(0x00000002); // Data Toggle
    pub const INCOMPRX = usize(0x00000001); // Incomplete RX Transmission
    // Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT5 register.
//
//*****************************************************************************
pub const RXCOUNT5 = struct {
    pub const COUNT_M = usize(0x00001FFF); // Receive Packet Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE5 register.
//
//*****************************************************************************
pub const TXTYPE5 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL5
// register.
//
//*****************************************************************************
pub const TXINTERVAL5 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // TX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const NAKLMT_S = usize(0);
    pub const TXPOLL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE5 register.
//
//*****************************************************************************
pub const RXTYPE5 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL5
// register.
//
//*****************************************************************************
pub const RXINTERVAL5 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // RX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const TXPOLL_S = usize(0);
    pub const NAKLMT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP6 register.
//
//*****************************************************************************
pub const TXMAXP6 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL6 register.
//
//*****************************************************************************
pub const TXCSRL6 = struct {
    pub const NAKTO = usize(0x00000080); // NAK Timeout
    pub const CLRDT = usize(0x00000040); // Clear Data Toggle
    pub const STALLED = usize(0x00000020); // Endpoint Stalled
    pub const STALL = usize(0x00000010); // Send STALL
    pub const SETUP = usize(0x00000010); // Setup Packet
    pub const FLUSH = usize(0x00000008); // Flush FIFO
    pub const ERROR = usize(0x00000004); // Error
    pub const UNDRN = usize(0x00000004); // Underrun
    pub const FIFONE = usize(0x00000002); // FIFO Not Empty
    pub const TXRDY = usize(0x00000001); // Transmit Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH6 register.
//
//*****************************************************************************
pub const TXCSRH6 = struct {
    pub const AUTOSET = usize(0x00000080); // Auto Set
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const MODE = usize(0x00000020); // Mode
    pub const DMAEN = usize(0x00000010); // DMA Request Enable
    pub const FDT = usize(0x00000008); // Force Data Toggle
    pub const DMAMOD = usize(0x00000004); // DMA Request Mode
    pub const DTWE = usize(0x00000002); // Data Toggle Write Enable
    pub const DT = usize(0x00000001); // Data Toggle
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP6 register.
//
//*****************************************************************************
pub const RXMAXP6 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL6 register.
//
//*****************************************************************************
pub const RXCSRL6 = struct {
    pub const CLRDT = usize(0x00000080); // Clear Data Toggle
    pub const STALLED = usize(0x00000040); // Endpoint Stalled
    pub const REQPKT = usize(0x00000020); // Request Packet
    pub const STALL = usize(0x00000020); // Send STALL
    pub const FLUSH = usize(0x00000010); // Flush FIFO
    pub const NAKTO = usize(0x00000008); // NAK Timeout
    pub const DATAERR = usize(0x00000008); // Data Error
    pub const ERROR = usize(0x00000004); // Error
    pub const OVER = usize(0x00000004); // Overrun
    pub const FULL = usize(0x00000002); // FIFO Full
    pub const RXRDY = usize(0x00000001); // Receive Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH6 register.
//
//*****************************************************************************
pub const RXCSRH6 = struct {
    pub const AUTOCL = usize(0x00000080); // Auto Clear
    pub const AUTORQ = usize(0x00000040); // Auto Request
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const DMAEN = usize(0x00000020); // DMA Request Enable
    pub const DISNYET = usize(0x00000010); // Disable NYET
    pub const PIDERR = usize(0x00000010); // PID Error
    pub const DMAMOD = usize(0x00000008); // DMA Request Mode
    pub const DTWE = usize(0x00000004); // Data Toggle Write Enable
    pub const DT = usize(0x00000002); // Data Toggle
    pub const INCOMPRX = usize(0x00000001); // Incomplete RX Transmission
    // Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT6 register.
//
//*****************************************************************************
pub const RXCOUNT6 = struct {
    pub const COUNT_M = usize(0x00001FFF); // Receive Packet Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE6 register.
//
//*****************************************************************************
pub const TXTYPE6 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL6
// register.
//
//*****************************************************************************
pub const TXINTERVAL6 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // TX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const TXPOLL_S = usize(0);
    pub const NAKLMT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE6 register.
//
//*****************************************************************************
pub const RXTYPE6 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL6
// register.
//
//*****************************************************************************
pub const RXINTERVAL6 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // RX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const NAKLMT_S = usize(0);
    pub const TXPOLL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP7 register.
//
//*****************************************************************************
pub const TXMAXP7 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL7 register.
//
//*****************************************************************************
pub const TXCSRL7 = struct {
    pub const NAKTO = usize(0x00000080); // NAK Timeout
    pub const CLRDT = usize(0x00000040); // Clear Data Toggle
    pub const STALLED = usize(0x00000020); // Endpoint Stalled
    pub const STALL = usize(0x00000010); // Send STALL
    pub const SETUP = usize(0x00000010); // Setup Packet
    pub const FLUSH = usize(0x00000008); // Flush FIFO
    pub const ERROR = usize(0x00000004); // Error
    pub const UNDRN = usize(0x00000004); // Underrun
    pub const FIFONE = usize(0x00000002); // FIFO Not Empty
    pub const TXRDY = usize(0x00000001); // Transmit Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH7 register.
//
//*****************************************************************************
pub const TXCSRH7 = struct {
    pub const AUTOSET = usize(0x00000080); // Auto Set
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const MODE = usize(0x00000020); // Mode
    pub const DMAEN = usize(0x00000010); // DMA Request Enable
    pub const FDT = usize(0x00000008); // Force Data Toggle
    pub const DMAMOD = usize(0x00000004); // DMA Request Mode
    pub const DTWE = usize(0x00000002); // Data Toggle Write Enable
    pub const DT = usize(0x00000001); // Data Toggle
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP7 register.
//
//*****************************************************************************
pub const RXMAXP7 = struct {
    pub const MAXLOAD_M = usize(0x000007FF); // Maximum Payload
    pub const MAXLOAD_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL7 register.
//
//*****************************************************************************
pub const RXCSRL7 = struct {
    pub const CLRDT = usize(0x00000080); // Clear Data Toggle
    pub const STALLED = usize(0x00000040); // Endpoint Stalled
    pub const REQPKT = usize(0x00000020); // Request Packet
    pub const STALL = usize(0x00000020); // Send STALL
    pub const FLUSH = usize(0x00000010); // Flush FIFO
    pub const DATAERR = usize(0x00000008); // Data Error
    pub const NAKTO = usize(0x00000008); // NAK Timeout
    pub const ERROR = usize(0x00000004); // Error
    pub const OVER = usize(0x00000004); // Overrun
    pub const FULL = usize(0x00000002); // FIFO Full
    pub const RXRDY = usize(0x00000001); // Receive Packet Ready
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH7 register.
//
//*****************************************************************************
pub const RXCSRH7 = struct {
    pub const AUTOCL = usize(0x00000080); // Auto Clear
    pub const ISO = usize(0x00000040); // Isochronous Transfers
    pub const AUTORQ = usize(0x00000040); // Auto Request
    pub const DMAEN = usize(0x00000020); // DMA Request Enable
    pub const PIDERR = usize(0x00000010); // PID Error
    pub const DISNYET = usize(0x00000010); // Disable NYET
    pub const DMAMOD = usize(0x00000008); // DMA Request Mode
    pub const DTWE = usize(0x00000004); // Data Toggle Write Enable
    pub const DT = usize(0x00000002); // Data Toggle
    pub const INCOMPRX = usize(0x00000001); // Incomplete RX Transmission
    // Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT7 register.
//
//*****************************************************************************
pub const RXCOUNT7 = struct {
    pub const COUNT_M = usize(0x00001FFF); // Receive Packet Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE7 register.
//
//*****************************************************************************
pub const TXTYPE7 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL7
// register.
//
//*****************************************************************************
pub const TXINTERVAL7 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // TX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const NAKLMT_S = usize(0);
    pub const TXPOLL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE7 register.
//
//*****************************************************************************
pub const RXTYPE7 = struct {
    pub const SPEED_M = usize(0x000000C0); // Operating Speed
    pub const SPEED_DFLT = usize(0x00000000); // Default
    pub const SPEED_HIGH = usize(0x00000040); // High
    pub const SPEED_FULL = usize(0x00000080); // Full
    pub const SPEED_LOW = usize(0x000000C0); // Low
    pub const PROTO_M = usize(0x00000030); // Protocol
    pub const PROTO_CTRL = usize(0x00000000); // Control
    pub const PROTO_ISOC = usize(0x00000010); // Isochronous
    pub const PROTO_BULK = usize(0x00000020); // Bulk
    pub const PROTO_INT = usize(0x00000030); // Interrupt
    pub const TEP_M = usize(0x0000000F); // Target Endpoint Number
    pub const TEP_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL7
// register.
//
//*****************************************************************************
pub const RXINTERVAL7 = struct {
    pub const TXPOLL_M = usize(0x000000FF); // RX Polling
    pub const NAKLMT_M = usize(0x000000FF); // NAK Limit
    pub const NAKLMT_S = usize(0);
    pub const TXPOLL_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAINTR register.
//
//*****************************************************************************
pub const DMAINTR = struct {
    pub const CH7 = usize(0x00000080); // Channel 7 DMA Interrupt
    pub const CH6 = usize(0x00000040); // Channel 6 DMA Interrupt
    pub const CH5 = usize(0x00000020); // Channel 5 DMA Interrupt
    pub const CH4 = usize(0x00000010); // Channel 4 DMA Interrupt
    pub const CH3 = usize(0x00000008); // Channel 3 DMA Interrupt
    pub const CH2 = usize(0x00000004); // Channel 2 DMA Interrupt
    pub const CH1 = usize(0x00000002); // Channel 1 DMA Interrupt
    pub const CH0 = usize(0x00000001); // Channel 0 DMA Interrupt
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL0 register.
//
//*****************************************************************************
pub const DMACTL0 = struct {
    pub const BRSTM_M = usize(0x00000600); // Burst Mode
    pub const BRSTM_ANY = usize(0x00000000); // Bursts of unspecified length
    pub const BRSTM_INC4 = usize(0x00000200); // INCR4 or unspecified length
    pub const BRSTM_INC8 = usize(0x00000400); // INCR8, INCR4 or unspecified
    // length
    pub const BRSTM_INC16 = usize(0x00000600); // INCR16, INCR8, INCR4 or
    // unspecified length
    pub const ERR = usize(0x00000100); // Bus Error Bit
    pub const EP_M = usize(0x000000F0); // Endpoint number
    pub const IE = usize(0x00000008); // DMA Interrupt Enable
    pub const MODE = usize(0x00000004); // DMA Transfer Mode
    pub const DIR = usize(0x00000002); // DMA Direction
    pub const ENABLE = usize(0x00000001); // DMA Transfer Enable
    pub const EP_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR0 register.
//
//*****************************************************************************
pub const DMAADDR0 = struct {
    pub const ADDR_M = usize(0xFFFFFFFC); // DMA Address
    pub const ADDR_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT0
// register.
//
//*****************************************************************************
pub const DMACOUNT0 = struct {
    pub const COUNT_M = usize(0xFFFFFFFC); // DMA Count
    pub const COUNT_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL1 register.
//
//*****************************************************************************
pub const DMACTL1 = struct {
    pub const BRSTM_M = usize(0x00000600); // Burst Mode
    pub const BRSTM_ANY = usize(0x00000000); // Bursts of unspecified length
    pub const BRSTM_INC4 = usize(0x00000200); // INCR4 or unspecified length
    pub const BRSTM_INC8 = usize(0x00000400); // INCR8, INCR4 or unspecified
    // length
    pub const BRSTM_INC16 = usize(0x00000600); // INCR16, INCR8, INCR4 or
    // unspecified length
    pub const ERR = usize(0x00000100); // Bus Error Bit
    pub const EP_M = usize(0x000000F0); // Endpoint number
    pub const IE = usize(0x00000008); // DMA Interrupt Enable
    pub const MODE = usize(0x00000004); // DMA Transfer Mode
    pub const DIR = usize(0x00000002); // DMA Direction
    pub const ENABLE = usize(0x00000001); // DMA Transfer Enable
    pub const EP_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR1 register.
//
//*****************************************************************************
pub const DMAADDR1 = struct {
    pub const ADDR_M = usize(0xFFFFFFFC); // DMA Address
    pub const ADDR_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT1
// register.
//
//*****************************************************************************
pub const DMACOUNT1 = struct {
    pub const COUNT_M = usize(0xFFFFFFFC); // DMA Count
    pub const COUNT_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL2 register.
//
//*****************************************************************************
pub const DMACTL2 = struct {
    pub const BRSTM_M = usize(0x00000600); // Burst Mode
    pub const BRSTM_ANY = usize(0x00000000); // Bursts of unspecified length
    pub const BRSTM_INC4 = usize(0x00000200); // INCR4 or unspecified length
    pub const BRSTM_INC8 = usize(0x00000400); // INCR8, INCR4 or unspecified
    // length
    pub const BRSTM_INC16 = usize(0x00000600); // INCR16, INCR8, INCR4 or
    // unspecified length
    pub const ERR = usize(0x00000100); // Bus Error Bit
    pub const EP_M = usize(0x000000F0); // Endpoint number
    pub const IE = usize(0x00000008); // DMA Interrupt Enable
    pub const MODE = usize(0x00000004); // DMA Transfer Mode
    pub const DIR = usize(0x00000002); // DMA Direction
    pub const ENABLE = usize(0x00000001); // DMA Transfer Enable
    pub const EP_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR2 register.
//
//*****************************************************************************
pub const DMAADDR2 = struct {
    pub const ADDR_M = usize(0xFFFFFFFC); // DMA Address
    pub const ADDR_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT2
// register.
//
//*****************************************************************************
pub const DMACOUNT2 = struct {
    pub const COUNT_M = usize(0xFFFFFFFC); // DMA Count
    pub const COUNT_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL3 register.
//
//*****************************************************************************
pub const DMACTL3 = struct {
    pub const BRSTM_M = usize(0x00000600); // Burst Mode
    pub const BRSTM_ANY = usize(0x00000000); // Bursts of unspecified length
    pub const BRSTM_INC4 = usize(0x00000200); // INCR4 or unspecified length
    pub const BRSTM_INC8 = usize(0x00000400); // INCR8, INCR4 or unspecified
    // length
    pub const BRSTM_INC16 = usize(0x00000600); // INCR16, INCR8, INCR4 or
    // unspecified length
    pub const ERR = usize(0x00000100); // Bus Error Bit
    pub const EP_M = usize(0x000000F0); // Endpoint number
    pub const IE = usize(0x00000008); // DMA Interrupt Enable
    pub const MODE = usize(0x00000004); // DMA Transfer Mode
    pub const DIR = usize(0x00000002); // DMA Direction
    pub const ENABLE = usize(0x00000001); // DMA Transfer Enable
    pub const EP_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR3 register.
//
//*****************************************************************************
pub const DMAADDR3 = struct {
    pub const ADDR_M = usize(0xFFFFFFFC); // DMA Address
    pub const ADDR_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT3
// register.
//
//*****************************************************************************
pub const DMACOUNT3 = struct {
    pub const COUNT_M = usize(0xFFFFFFFC); // DMA Count
    pub const COUNT_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL4 register.
//
//*****************************************************************************
pub const DMACTL4 = struct {
    pub const BRSTM_M = usize(0x00000600); // Burst Mode
    pub const BRSTM_ANY = usize(0x00000000); // Bursts of unspecified length
    pub const BRSTM_INC4 = usize(0x00000200); // INCR4 or unspecified length
    pub const BRSTM_INC8 = usize(0x00000400); // INCR8, INCR4 or unspecified
    // length
    pub const BRSTM_INC16 = usize(0x00000600); // INCR16, INCR8, INCR4 or
    // unspecified length
    pub const ERR = usize(0x00000100); // Bus Error Bit
    pub const EP_M = usize(0x000000F0); // Endpoint number
    pub const IE = usize(0x00000008); // DMA Interrupt Enable
    pub const MODE = usize(0x00000004); // DMA Transfer Mode
    pub const DIR = usize(0x00000002); // DMA Direction
    pub const ENABLE = usize(0x00000001); // DMA Transfer Enable
    pub const EP_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR4 register.
//
//*****************************************************************************
pub const DMAADDR4 = struct {
    pub const ADDR_M = usize(0xFFFFFFFC); // DMA Address
    pub const ADDR_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT4
// register.
//
//*****************************************************************************
pub const DMACOUNT4 = struct {
    pub const COUNT_M = usize(0xFFFFFFFC); // DMA Count
    pub const COUNT_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL5 register.
//
//*****************************************************************************
pub const DMACTL5 = struct {
    pub const BRSTM_M = usize(0x00000600); // Burst Mode
    pub const BRSTM_ANY = usize(0x00000000); // Bursts of unspecified length
    pub const BRSTM_INC4 = usize(0x00000200); // INCR4 or unspecified length
    pub const BRSTM_INC8 = usize(0x00000400); // INCR8, INCR4 or unspecified
    // length
    pub const BRSTM_INC16 = usize(0x00000600); // INCR16, INCR8, INCR4 or
    // unspecified length
    pub const ERR = usize(0x00000100); // Bus Error Bit
    pub const EP_M = usize(0x000000F0); // Endpoint number
    pub const IE = usize(0x00000008); // DMA Interrupt Enable
    pub const MODE = usize(0x00000004); // DMA Transfer Mode
    pub const DIR = usize(0x00000002); // DMA Direction
    pub const ENABLE = usize(0x00000001); // DMA Transfer Enable
    pub const EP_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR5 register.
//
//*****************************************************************************
pub const DMAADDR5 = struct {
    pub const ADDR_M = usize(0xFFFFFFFC); // DMA Address
    pub const ADDR_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT5
// register.
//
//*****************************************************************************
pub const DMACOUNT5 = struct {
    pub const COUNT_M = usize(0xFFFFFFFC); // DMA Count
    pub const COUNT_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL6 register.
//
//*****************************************************************************
pub const DMACTL6 = struct {
    pub const BRSTM_M = usize(0x00000600); // Burst Mode
    pub const BRSTM_ANY = usize(0x00000000); // Bursts of unspecified length
    pub const BRSTM_INC4 = usize(0x00000200); // INCR4 or unspecified length
    pub const BRSTM_INC8 = usize(0x00000400); // INCR8, INCR4 or unspecified
    // length
    pub const BRSTM_INC16 = usize(0x00000600); // INCR16, INCR8, INCR4 or
    // unspecified length
    pub const ERR = usize(0x00000100); // Bus Error Bit
    pub const EP_M = usize(0x000000F0); // Endpoint number
    pub const IE = usize(0x00000008); // DMA Interrupt Enable
    pub const MODE = usize(0x00000004); // DMA Transfer Mode
    pub const DIR = usize(0x00000002); // DMA Direction
    pub const ENABLE = usize(0x00000001); // DMA Transfer Enable
    pub const EP_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR6 register.
//
//*****************************************************************************
pub const DMAADDR6 = struct {
    pub const ADDR_M = usize(0xFFFFFFFC); // DMA Address
    pub const ADDR_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT6
// register.
//
//*****************************************************************************
pub const DMACOUNT6 = struct {
    pub const COUNT_M = usize(0xFFFFFFFC); // DMA Count
    pub const COUNT_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACTL7 register.
//
//*****************************************************************************
pub const DMACTL7 = struct {
    pub const BRSTM_M = usize(0x00000600); // Burst Mode
    pub const BRSTM_ANY = usize(0x00000000); // Bursts of unspecified length
    pub const BRSTM_INC4 = usize(0x00000200); // INCR4 or unspecified length
    pub const BRSTM_INC8 = usize(0x00000400); // INCR8, INCR4 or unspecified
    // length
    pub const BRSTM_INC16 = usize(0x00000600); // INCR16, INCR8, INCR4 or
    // unspecified length
    pub const ERR = usize(0x00000100); // Bus Error Bit
    pub const EP_M = usize(0x000000F0); // Endpoint number
    pub const IE = usize(0x00000008); // DMA Interrupt Enable
    pub const MODE = usize(0x00000004); // DMA Transfer Mode
    pub const DIR = usize(0x00000002); // DMA Direction
    pub const ENABLE = usize(0x00000001); // DMA Transfer Enable
    pub const EP_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMAADDR7 register.
//
//*****************************************************************************
pub const DMAADDR7 = struct {
    pub const ADDR_M = usize(0xFFFFFFFC); // DMA Address
    pub const ADDR_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMACOUNT7
// register.
//
//*****************************************************************************
pub const DMACOUNT7 = struct {
    pub const COUNT_M = usize(0xFFFFFFFC); // DMA Count
    pub const COUNT_S = usize(2);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT1
// register.
//
//*****************************************************************************
pub const RQPKTCOUNT1 = struct {
    pub const M = usize(0x0000FFFF); // Block Transfer Packet Count
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT2
// register.
//
//*****************************************************************************
pub const RQPKTCOUNT2 = struct {
    pub const M = usize(0x0000FFFF); // Block Transfer Packet Count
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT3
// register.
//
//*****************************************************************************
pub const RQPKTCOUNT3 = struct {
    pub const M = usize(0x0000FFFF); // Block Transfer Packet Count
    pub const S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT4
// register.
//
//*****************************************************************************
pub const RQPKTCOUNT4 = struct {
    pub const COUNT_M = usize(0x0000FFFF); // Block Transfer Packet Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT5
// register.
//
//*****************************************************************************
pub const RQPKTCOUNT5 = struct {
    pub const COUNT_M = usize(0x0000FFFF); // Block Transfer Packet Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT6
// register.
//
//*****************************************************************************
pub const RQPKTCOUNT6 = struct {
    pub const COUNT_M = usize(0x0000FFFF); // Block Transfer Packet Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT7
// register.
//
//*****************************************************************************
pub const RQPKTCOUNT7 = struct {
    pub const COUNT_M = usize(0x0000FFFF); // Block Transfer Packet Count
    pub const COUNT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXDPKTBUFDIS
// register.
//
//*****************************************************************************
pub const RXDPKTBUFDIS = struct {
    pub const EP7 = usize(0x00000080); // EP7 RX Double-Packet Buffer
    // Disable
    pub const EP6 = usize(0x00000040); // EP6 RX Double-Packet Buffer
    // Disable
    pub const EP5 = usize(0x00000020); // EP5 RX Double-Packet Buffer
    // Disable
    pub const EP4 = usize(0x00000010); // EP4 RX Double-Packet Buffer
    // Disable
    pub const EP3 = usize(0x00000008); // EP3 RX Double-Packet Buffer
    // Disable
    pub const EP2 = usize(0x00000004); // EP2 RX Double-Packet Buffer
    // Disable
    pub const EP1 = usize(0x00000002); // EP1 RX Double-Packet Buffer
    // Disable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXDPKTBUFDIS
// register.
//
//*****************************************************************************
pub const TXDPKTBUFDIS = struct {
    pub const EP7 = usize(0x00000080); // EP7 TX Double-Packet Buffer
    // Disable
    pub const EP6 = usize(0x00000040); // EP6 TX Double-Packet Buffer
    // Disable
    pub const EP5 = usize(0x00000020); // EP5 TX Double-Packet Buffer
    // Disable
    pub const EP4 = usize(0x00000010); // EP4 TX Double-Packet Buffer
    // Disable
    pub const EP3 = usize(0x00000008); // EP3 TX Double-Packet Buffer
    // Disable
    pub const EP2 = usize(0x00000004); // EP2 TX Double-Packet Buffer
    // Disable
    pub const EP1 = usize(0x00000002); // EP1 TX Double-Packet Buffer
    // Disable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CTO register.
//
//*****************************************************************************
pub const CTO = struct {
    pub const CCTV_M = usize(0x0000FFFF); // Configurable Chirp Timeout Value
    pub const CCTV_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_HHSRTN register.
//
//*****************************************************************************
pub const HHSRTN = struct {
    pub const HHSRTN_M = usize(0x0000FFFF); // HIgh Speed to UTM Operating
    // Delay
    pub const HHSRTN_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_HSBT register.
//
//*****************************************************************************
pub const HSBT = struct {
    pub const HSBT_M = usize(0x0000000F); // High Speed Timeout Adder
    pub const HSBT_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LPMATTR register.
//
//*****************************************************************************
pub const LPMATTR = struct {
    pub const ENDPT_M = usize(0x0000F000); // Endpoint
    pub const RMTWAK = usize(0x00000100); // Remote Wake
    pub const HIRD_M = usize(0x000000F0); // Host Initiated Resume Duration
    pub const LS_M = usize(0x0000000F); // Link State
    pub const LS_L1 = usize(0x00000001); // Sleep State (L1)
    pub const ENDPT_S = usize(12);
    pub const HIRD_S = usize(4);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LPMCNTRL register.
//
//*****************************************************************************
pub const LPMCNTRL = struct {
    pub const NAK = usize(0x00000010); // LPM NAK
    pub const EN_M = usize(0x0000000C); // LPM Enable
    pub const EN_NONE = usize(0x00000000); // LPM and Extended transactions
    // are not supported. In this case,
    // the USB does not respond to LPM
    // transactions and LPM
    // transactions cause a timeout
    pub const EN_EXT = usize(0x00000004); // LPM is not supported but
    // extended transactions are
    // supported. In this case, the USB
    // does respond to an LPM
    // transaction with a STALL
    pub const EN_LPMEXT = usize(0x0000000C); // The USB supports LPM extended
    // transactions. In this case, the
    // USB responds with a NYET or an
    // ACK as determined by the value
    // of TXLPM and other conditions
    pub const RES = usize(0x00000002); // LPM Resume
    pub const TXLPM = usize(0x00000001); // Transmit LPM Transaction Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LPMIM register.
//
//*****************************************************************************
pub const LPMIM = struct {
    pub const ERR = usize(0x00000020); // LPM Error Interrupt Mask
    pub const RES = usize(0x00000010); // LPM Resume Interrupt Mask
    pub const NC = usize(0x00000008); // LPM NC Interrupt Mask
    pub const ACK = usize(0x00000004); // LPM ACK Interrupt Mask
    pub const NY = usize(0x00000002); // LPM NY Interrupt Mask
    pub const STALL = usize(0x00000001); // LPM STALL Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LPMRIS register.
//
//*****************************************************************************
pub const LPMRIS = struct {
    pub const ERR = usize(0x00000020); // LPM Interrupt Status
    pub const RES = usize(0x00000010); // LPM Resume Interrupt Status
    pub const NC = usize(0x00000008); // LPM NC Interrupt Status
    pub const ACK = usize(0x00000004); // LPM ACK Interrupt Status
    pub const NY = usize(0x00000002); // LPM NY Interrupt Status
    pub const LPMST = usize(0x00000001); // LPM STALL Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LPMFADDR register.
//
//*****************************************************************************
pub const LPMFADDR = struct {
    pub const ADDR_M = usize(0x0000007F); // LPM Function Address
    pub const ADDR_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPC register.
//
//*****************************************************************************
pub const EPC = struct {
    pub const PFLTACT_M = usize(0x00000300); // Power Fault Action
    pub const PFLTACT_UNCHG = usize(0x00000000); // Unchanged
    pub const PFLTACT_TRIS = usize(0x00000100); // Tristate
    pub const PFLTACT_LOW = usize(0x00000200); // Low
    pub const PFLTACT_HIGH = usize(0x00000300); // High
    pub const PFLTAEN = usize(0x00000040); // Power Fault Action Enable
    pub const PFLTSEN_HIGH = usize(0x00000020); // Power Fault Sense
    pub const PFLTEN = usize(0x00000010); // Power Fault Input Enable
    pub const EPENDE = usize(0x00000004); // EPEN Drive Enable
    pub const EPEN_M = usize(0x00000003); // External Power Supply Enable
    // Configuration
    pub const EPEN_LOW = usize(0x00000000); // Power Enable Active Low
    pub const EPEN_HIGH = usize(0x00000001); // Power Enable Active High
    pub const EPEN_VBLOW = usize(0x00000002); // Power Enable High if VBUS Low
    // (OTG only)
    pub const EPEN_VBHIGH = usize(0x00000003); // Power Enable High if VBUS High
    // (OTG only)
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPCRIS register.
//
//*****************************************************************************
pub const EPCRIS = struct {
    pub const PF = usize(0x00000001); // USB Power Fault Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPCIM register.
//
//*****************************************************************************
pub const EPCIM = struct {
    pub const PF = usize(0x00000001); // USB Power Fault Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPCISC register.
//
//*****************************************************************************
pub const EPCISC = struct {
    pub const PF = usize(0x00000001); // USB Power Fault Interrupt Status
    // and Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRRIS register.
//
//*****************************************************************************
pub const DRRIS = struct {
    pub const RESUME = usize(0x00000001); // RESUME Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRIM register.
//
//*****************************************************************************
pub const DRIM = struct {
    pub const RESUME = usize(0x00000001); // RESUME Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRISC register.
//
//*****************************************************************************
pub const DRISC = struct {
    pub const RESUME = usize(0x00000001); // RESUME Interrupt Status and
    // Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_GPCS register.
//
//*****************************************************************************
pub const GPCS = struct {
    pub const DEVMOD_M = usize(0x00000007); // Device Mode
    pub const DEVMOD_OTG = usize(0x00000000); // Use USB0VBUS and USB0ID pin
    pub const DEVMOD_HOST = usize(0x00000002); // Force USB0VBUS and USB0ID low
    pub const DEVMOD_DEV = usize(0x00000003); // Force USB0VBUS and USB0ID high
    pub const DEVMOD_HOSTVBUS = usize(0x00000004); // Use USB0VBUS and force USB0ID
    // low
    pub const DEVMOD_DEVVBUS = usize(0x00000005); // Use USB0VBUS and force USB0ID
    // high
    pub const DEVMODOTG = usize(0x00000002); // Enable Device Mode
    pub const DEVMOD = usize(0x00000001); // Device Mode
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDC register.
//
//*****************************************************************************
pub const VDC = struct {
    pub const VBDEN = usize(0x00000001); // VBUS Droop Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDCRIS register.
//
//*****************************************************************************
pub const VDCRIS = struct {
    pub const VD = usize(0x00000001); // VBUS Droop Raw Interrupt Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDCIM register.
//
//*****************************************************************************
pub const VDCIM = struct {
    pub const VD = usize(0x00000001); // VBUS Droop Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDCISC register.
//
//*****************************************************************************
pub const VDCISC = struct {
    pub const VD = usize(0x00000001); // VBUS Droop Interrupt Status and
    // Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IDVRIS register.
//
//*****************************************************************************
pub const IDVRIS = struct {
    pub const ID = usize(0x00000001); // ID Valid Detect Raw Interrupt
    // Status
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IDVIM register.
//
//*****************************************************************************
pub const IDVIM = struct {
    pub const ID = usize(0x00000001); // ID Valid Detect Interrupt Mask
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IDVISC register.
//
//*****************************************************************************
pub const IDVISC = struct {
    pub const ID = usize(0x00000001); // ID Valid Detect Interrupt Status
    // and Clear
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMASEL register.
//
//*****************************************************************************
pub const DMASEL = struct {
    pub const DMACTX_M = usize(0x00F00000); // DMA C TX Select
    pub const DMACRX_M = usize(0x000F0000); // DMA C RX Select
    pub const DMABTX_M = usize(0x0000F000); // DMA B TX Select
    pub const DMABRX_M = usize(0x00000F00); // DMA B RX Select
    pub const DMAATX_M = usize(0x000000F0); // DMA A TX Select
    pub const DMAARX_M = usize(0x0000000F); // DMA A RX Select
    pub const DMACTX_S = usize(20);
    pub const DMACRX_S = usize(16);
    pub const DMABTX_S = usize(12);
    pub const DMABRX_S = usize(8);
    pub const DMAATX_S = usize(4);
    pub const DMAARX_S = usize(0);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_PP register.
//
//*****************************************************************************
pub const PP = struct {
    pub const ECNT_M = usize(0x0000FF00); // Endpoint Count
    pub const USB_M = usize(0x000000C0); // USB Capability
    pub const USB_DEVICE = usize(0x00000040); // DEVICE
    pub const USB_HOSTDEVICE = usize(0x00000080); // HOST
    pub const USB_OTG = usize(0x000000C0); // OTG
    pub const ULPI = usize(0x00000020); // ULPI Present
    pub const PHY = usize(0x00000010); // PHY Present
    pub const TYPE_M = usize(0x0000000F); // Controller Type
    pub const TYPE_0 = usize(0x00000000); // The first-generation USB
    // controller
    pub const TYPE_1 = usize(0x00000001); // Second-generation USB
    // controller.The controller
    // implemented in post Icestorm
    // devices that use the 3.0 version
    // of the Mentor controller
    pub const ECNT_S = usize(8);
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_PC register.
//
//*****************************************************************************
pub const PC = struct {
    pub const ULPIEN = usize(0x00010000); // ULPI Enable
};

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CC register.
//
//*****************************************************************************
pub const CC = struct {
    pub const CLKEN = usize(0x00000200); // USB Clock Enable
    pub const CSD = usize(0x00000100); // Clock Source/Direction
    pub const CLKDIV_M = usize(0x0000000F); // PLL Clock Divisor
    pub const CLKDIV_S = usize(0);
};
