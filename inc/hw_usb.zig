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
pub const USB_O_FADDR = usize(0x00000000);  // USB Device Functional Address
pub const USB_O_POWER = usize(0x00000001);  // USB Power
pub const USB_O_TXIS = usize(0x00000002);  // USB Transmit Interrupt Status
pub const USB_O_RXIS = usize(0x00000004);  // USB Receive Interrupt Status
pub const USB_O_TXIE = usize(0x00000006);  // USB Transmit Interrupt Enable
pub const USB_O_RXIE = usize(0x00000008);  // USB Receive Interrupt Enable
pub const USB_O_IS = usize(0x0000000A);  // USB General Interrupt Status
pub const USB_O_IE = usize(0x0000000B);  // USB Interrupt Enable
pub const USB_O_FRAME = usize(0x0000000C);  // USB Frame Value
pub const USB_O_EPIDX = usize(0x0000000E);  // USB Endpoint Index
pub const USB_O_TEST = usize(0x0000000F);  // USB Test Mode
pub const USB_O_FIFO0 = usize(0x00000020);  // USB FIFO Endpoint 0
pub const USB_O_FIFO1 = usize(0x00000024);  // USB FIFO Endpoint 1
pub const USB_O_FIFO2 = usize(0x00000028);  // USB FIFO Endpoint 2
pub const USB_O_FIFO3 = usize(0x0000002C);  // USB FIFO Endpoint 3
pub const USB_O_FIFO4 = usize(0x00000030);  // USB FIFO Endpoint 4
pub const USB_O_FIFO5 = usize(0x00000034);  // USB FIFO Endpoint 5
pub const USB_O_FIFO6 = usize(0x00000038);  // USB FIFO Endpoint 6
pub const USB_O_FIFO7 = usize(0x0000003C);  // USB FIFO Endpoint 7
pub const USB_O_DEVCTL = usize(0x00000060);  // USB Device Control
pub const USB_O_CCONF = usize(0x00000061);  // USB Common Configuration
pub const USB_O_TXFIFOSZ = usize(0x00000062);  // USB Transmit Dynamic FIFO Sizing
pub const USB_O_RXFIFOSZ = usize(0x00000063);  // USB Receive Dynamic FIFO Sizing
pub const USB_O_TXFIFOADD = usize(0x00000064);  // USB Transmit FIFO Start Address
pub const USB_O_RXFIFOADD = usize(0x00000066);  // USB Receive FIFO Start Address
pub const USB_O_ULPIVBUSCTL = usize(0x00000070);  // USB ULPI VBUS Control
pub const USB_O_ULPIREGDATA = usize(0x00000074);  // USB ULPI Register Data
pub const USB_O_ULPIREGADDR = usize(0x00000075);  // USB ULPI Register Address
pub const USB_O_ULPIREGCTL = usize(0x00000076);  // USB ULPI Register Control
pub const USB_O_EPINFO = usize(0x00000078);  // USB Endpoint Information
pub const USB_O_RAMINFO = usize(0x00000079);  // USB RAM Information
pub const USB_O_CONTIM = usize(0x0000007A);  // USB Connect Timing
pub const USB_O_VPLEN = usize(0x0000007B);  // USB OTG VBUS Pulse Timing
pub const USB_O_HSEOF = usize(0x0000007C);  // USB High-Speed Last Transaction
                                            // to End of Frame Timing
pub const USB_O_FSEOF = usize(0x0000007D);  // USB Full-Speed Last Transaction
                                            // to End of Frame Timing
pub const USB_O_LSEOF = usize(0x0000007E);  // USB Low-Speed Last Transaction
                                            // to End of Frame Timing
pub const USB_O_TXFUNCADDR0 = usize(0x00000080);  // USB Transmit Functional Address
                                            // Endpoint 0
pub const USB_O_TXHUBADDR0 = usize(0x00000082);  // USB Transmit Hub Address
                                            // Endpoint 0
pub const USB_O_TXHUBPORT0 = usize(0x00000083);  // USB Transmit Hub Port Endpoint 0
pub const USB_O_TXFUNCADDR1 = usize(0x00000088);  // USB Transmit Functional Address
                                            // Endpoint 1
pub const USB_O_TXHUBADDR1 = usize(0x0000008A);  // USB Transmit Hub Address
                                            // Endpoint 1
pub const USB_O_TXHUBPORT1 = usize(0x0000008B);  // USB Transmit Hub Port Endpoint 1
pub const USB_O_RXFUNCADDR1 = usize(0x0000008C);  // USB Receive Functional Address
                                            // Endpoint 1
pub const USB_O_RXHUBADDR1 = usize(0x0000008E);  // USB Receive Hub Address Endpoint
                                            // 1
pub const USB_O_RXHUBPORT1 = usize(0x0000008F);  // USB Receive Hub Port Endpoint 1
pub const USB_O_TXFUNCADDR2 = usize(0x00000090);  // USB Transmit Functional Address
                                            // Endpoint 2
pub const USB_O_TXHUBADDR2 = usize(0x00000092);  // USB Transmit Hub Address
                                            // Endpoint 2
pub const USB_O_TXHUBPORT2 = usize(0x00000093);  // USB Transmit Hub Port Endpoint 2
pub const USB_O_RXFUNCADDR2 = usize(0x00000094);  // USB Receive Functional Address
                                            // Endpoint 2
pub const USB_O_RXHUBADDR2 = usize(0x00000096);  // USB Receive Hub Address Endpoint
                                            // 2
pub const USB_O_RXHUBPORT2 = usize(0x00000097);  // USB Receive Hub Port Endpoint 2
pub const USB_O_TXFUNCADDR3 = usize(0x00000098);  // USB Transmit Functional Address
                                            // Endpoint 3
pub const USB_O_TXHUBADDR3 = usize(0x0000009A);  // USB Transmit Hub Address
                                            // Endpoint 3
pub const USB_O_TXHUBPORT3 = usize(0x0000009B);  // USB Transmit Hub Port Endpoint 3
pub const USB_O_RXFUNCADDR3 = usize(0x0000009C);  // USB Receive Functional Address
                                            // Endpoint 3
pub const USB_O_RXHUBADDR3 = usize(0x0000009E);  // USB Receive Hub Address Endpoint
                                            // 3
pub const USB_O_RXHUBPORT3 = usize(0x0000009F);  // USB Receive Hub Port Endpoint 3
pub const USB_O_TXFUNCADDR4 = usize(0x000000A0);  // USB Transmit Functional Address
                                            // Endpoint 4
pub const USB_O_TXHUBADDR4 = usize(0x000000A2);  // USB Transmit Hub Address
                                            // Endpoint 4
pub const USB_O_TXHUBPORT4 = usize(0x000000A3);  // USB Transmit Hub Port Endpoint 4
pub const USB_O_RXFUNCADDR4 = usize(0x000000A4);  // USB Receive Functional Address
                                            // Endpoint 4
pub const USB_O_RXHUBADDR4 = usize(0x000000A6);  // USB Receive Hub Address Endpoint
                                            // 4
pub const USB_O_RXHUBPORT4 = usize(0x000000A7);  // USB Receive Hub Port Endpoint 4
pub const USB_O_TXFUNCADDR5 = usize(0x000000A8);  // USB Transmit Functional Address
                                            // Endpoint 5
pub const USB_O_TXHUBADDR5 = usize(0x000000AA);  // USB Transmit Hub Address
                                            // Endpoint 5
pub const USB_O_TXHUBPORT5 = usize(0x000000AB);  // USB Transmit Hub Port Endpoint 5
pub const USB_O_RXFUNCADDR5 = usize(0x000000AC);  // USB Receive Functional Address
                                            // Endpoint 5
pub const USB_O_RXHUBADDR5 = usize(0x000000AE);  // USB Receive Hub Address Endpoint
                                            // 5
pub const USB_O_RXHUBPORT5 = usize(0x000000AF);  // USB Receive Hub Port Endpoint 5
pub const USB_O_TXFUNCADDR6 = usize(0x000000B0);  // USB Transmit Functional Address
                                            // Endpoint 6
pub const USB_O_TXHUBADDR6 = usize(0x000000B2);  // USB Transmit Hub Address
                                            // Endpoint 6
pub const USB_O_TXHUBPORT6 = usize(0x000000B3);  // USB Transmit Hub Port Endpoint 6
pub const USB_O_RXFUNCADDR6 = usize(0x000000B4);  // USB Receive Functional Address
                                            // Endpoint 6
pub const USB_O_RXHUBADDR6 = usize(0x000000B6);  // USB Receive Hub Address Endpoint
                                            // 6
pub const USB_O_RXHUBPORT6 = usize(0x000000B7);  // USB Receive Hub Port Endpoint 6
pub const USB_O_TXFUNCADDR7 = usize(0x000000B8);  // USB Transmit Functional Address
                                            // Endpoint 7
pub const USB_O_TXHUBADDR7 = usize(0x000000BA);  // USB Transmit Hub Address
                                            // Endpoint 7
pub const USB_O_TXHUBPORT7 = usize(0x000000BB);  // USB Transmit Hub Port Endpoint 7
pub const USB_O_RXFUNCADDR7 = usize(0x000000BC);  // USB Receive Functional Address
                                            // Endpoint 7
pub const USB_O_RXHUBADDR7 = usize(0x000000BE);  // USB Receive Hub Address Endpoint
                                            // 7
pub const USB_O_RXHUBPORT7 = usize(0x000000BF);  // USB Receive Hub Port Endpoint 7
pub const USB_O_CSRL0 = usize(0x00000102);  // USB Control and Status Endpoint
                                            // 0 Low
pub const USB_O_CSRH0 = usize(0x00000103);  // USB Control and Status Endpoint
                                            // 0 High
pub const USB_O_COUNT0 = usize(0x00000108);  // USB Receive Byte Count Endpoint
                                            // 0
pub const USB_O_TYPE0 = usize(0x0000010A);  // USB Type Endpoint 0
pub const USB_O_NAKLMT = usize(0x0000010B);  // USB NAK Limit
pub const USB_O_TXMAXP1 = usize(0x00000110);  // USB Maximum Transmit Data
                                            // Endpoint 1
pub const USB_O_TXCSRL1 = usize(0x00000112);  // USB Transmit Control and Status
                                            // Endpoint 1 Low
pub const USB_O_TXCSRH1 = usize(0x00000113);  // USB Transmit Control and Status
                                            // Endpoint 1 High
pub const USB_O_RXMAXP1 = usize(0x00000114);  // USB Maximum Receive Data
                                            // Endpoint 1
pub const USB_O_RXCSRL1 = usize(0x00000116);  // USB Receive Control and Status
                                            // Endpoint 1 Low
pub const USB_O_RXCSRH1 = usize(0x00000117);  // USB Receive Control and Status
                                            // Endpoint 1 High
pub const USB_O_RXCOUNT1 = usize(0x00000118);  // USB Receive Byte Count Endpoint
                                            // 1
pub const USB_O_TXTYPE1 = usize(0x0000011A);  // USB Host Transmit Configure Type
                                            // Endpoint 1
pub const USB_O_TXINTERVAL1 = usize(0x0000011B);  // USB Host Transmit Interval
                                            // Endpoint 1
pub const USB_O_RXTYPE1 = usize(0x0000011C);  // USB Host Configure Receive Type
                                            // Endpoint 1
pub const USB_O_RXINTERVAL1 = usize(0x0000011D);  // USB Host Receive Polling
                                            // Interval Endpoint 1
pub const USB_O_TXMAXP2 = usize(0x00000120);  // USB Maximum Transmit Data
                                            // Endpoint 2
pub const USB_O_TXCSRL2 = usize(0x00000122);  // USB Transmit Control and Status
                                            // Endpoint 2 Low
pub const USB_O_TXCSRH2 = usize(0x00000123);  // USB Transmit Control and Status
                                            // Endpoint 2 High
pub const USB_O_RXMAXP2 = usize(0x00000124);  // USB Maximum Receive Data
                                            // Endpoint 2
pub const USB_O_RXCSRL2 = usize(0x00000126);  // USB Receive Control and Status
                                            // Endpoint 2 Low
pub const USB_O_RXCSRH2 = usize(0x00000127);  // USB Receive Control and Status
                                            // Endpoint 2 High
pub const USB_O_RXCOUNT2 = usize(0x00000128);  // USB Receive Byte Count Endpoint
                                            // 2
pub const USB_O_TXTYPE2 = usize(0x0000012A);  // USB Host Transmit Configure Type
                                            // Endpoint 2
pub const USB_O_TXINTERVAL2 = usize(0x0000012B);  // USB Host Transmit Interval
                                            // Endpoint 2
pub const USB_O_RXTYPE2 = usize(0x0000012C);  // USB Host Configure Receive Type
                                            // Endpoint 2
pub const USB_O_RXINTERVAL2 = usize(0x0000012D);  // USB Host Receive Polling
                                            // Interval Endpoint 2
pub const USB_O_TXMAXP3 = usize(0x00000130);  // USB Maximum Transmit Data
                                            // Endpoint 3
pub const USB_O_TXCSRL3 = usize(0x00000132);  // USB Transmit Control and Status
                                            // Endpoint 3 Low
pub const USB_O_TXCSRH3 = usize(0x00000133);  // USB Transmit Control and Status
                                            // Endpoint 3 High
pub const USB_O_RXMAXP3 = usize(0x00000134);  // USB Maximum Receive Data
                                            // Endpoint 3
pub const USB_O_RXCSRL3 = usize(0x00000136);  // USB Receive Control and Status
                                            // Endpoint 3 Low
pub const USB_O_RXCSRH3 = usize(0x00000137);  // USB Receive Control and Status
                                            // Endpoint 3 High
pub const USB_O_RXCOUNT3 = usize(0x00000138);  // USB Receive Byte Count Endpoint
                                            // 3
pub const USB_O_TXTYPE3 = usize(0x0000013A);  // USB Host Transmit Configure Type
                                            // Endpoint 3
pub const USB_O_TXINTERVAL3 = usize(0x0000013B);  // USB Host Transmit Interval
                                            // Endpoint 3
pub const USB_O_RXTYPE3 = usize(0x0000013C);  // USB Host Configure Receive Type
                                            // Endpoint 3
pub const USB_O_RXINTERVAL3 = usize(0x0000013D);  // USB Host Receive Polling
                                            // Interval Endpoint 3
pub const USB_O_TXMAXP4 = usize(0x00000140);  // USB Maximum Transmit Data
                                            // Endpoint 4
pub const USB_O_TXCSRL4 = usize(0x00000142);  // USB Transmit Control and Status
                                            // Endpoint 4 Low
pub const USB_O_TXCSRH4 = usize(0x00000143);  // USB Transmit Control and Status
                                            // Endpoint 4 High
pub const USB_O_RXMAXP4 = usize(0x00000144);  // USB Maximum Receive Data
                                            // Endpoint 4
pub const USB_O_RXCSRL4 = usize(0x00000146);  // USB Receive Control and Status
                                            // Endpoint 4 Low
pub const USB_O_RXCSRH4 = usize(0x00000147);  // USB Receive Control and Status
                                            // Endpoint 4 High
pub const USB_O_RXCOUNT4 = usize(0x00000148);  // USB Receive Byte Count Endpoint
                                            // 4
pub const USB_O_TXTYPE4 = usize(0x0000014A);  // USB Host Transmit Configure Type
                                            // Endpoint 4
pub const USB_O_TXINTERVAL4 = usize(0x0000014B);  // USB Host Transmit Interval
                                            // Endpoint 4
pub const USB_O_RXTYPE4 = usize(0x0000014C);  // USB Host Configure Receive Type
                                            // Endpoint 4
pub const USB_O_RXINTERVAL4 = usize(0x0000014D);  // USB Host Receive Polling
                                            // Interval Endpoint 4
pub const USB_O_TXMAXP5 = usize(0x00000150);  // USB Maximum Transmit Data
                                            // Endpoint 5
pub const USB_O_TXCSRL5 = usize(0x00000152);  // USB Transmit Control and Status
                                            // Endpoint 5 Low
pub const USB_O_TXCSRH5 = usize(0x00000153);  // USB Transmit Control and Status
                                            // Endpoint 5 High
pub const USB_O_RXMAXP5 = usize(0x00000154);  // USB Maximum Receive Data
                                            // Endpoint 5
pub const USB_O_RXCSRL5 = usize(0x00000156);  // USB Receive Control and Status
                                            // Endpoint 5 Low
pub const USB_O_RXCSRH5 = usize(0x00000157);  // USB Receive Control and Status
                                            // Endpoint 5 High
pub const USB_O_RXCOUNT5 = usize(0x00000158);  // USB Receive Byte Count Endpoint
                                            // 5
pub const USB_O_TXTYPE5 = usize(0x0000015A);  // USB Host Transmit Configure Type
                                            // Endpoint 5
pub const USB_O_TXINTERVAL5 = usize(0x0000015B);  // USB Host Transmit Interval
                                            // Endpoint 5
pub const USB_O_RXTYPE5 = usize(0x0000015C);  // USB Host Configure Receive Type
                                            // Endpoint 5
pub const USB_O_RXINTERVAL5 = usize(0x0000015D);  // USB Host Receive Polling
                                            // Interval Endpoint 5
pub const USB_O_TXMAXP6 = usize(0x00000160);  // USB Maximum Transmit Data
                                            // Endpoint 6
pub const USB_O_TXCSRL6 = usize(0x00000162);  // USB Transmit Control and Status
                                            // Endpoint 6 Low
pub const USB_O_TXCSRH6 = usize(0x00000163);  // USB Transmit Control and Status
                                            // Endpoint 6 High
pub const USB_O_RXMAXP6 = usize(0x00000164);  // USB Maximum Receive Data
                                            // Endpoint 6
pub const USB_O_RXCSRL6 = usize(0x00000166);  // USB Receive Control and Status
                                            // Endpoint 6 Low
pub const USB_O_RXCSRH6 = usize(0x00000167);  // USB Receive Control and Status
                                            // Endpoint 6 High
pub const USB_O_RXCOUNT6 = usize(0x00000168);  // USB Receive Byte Count Endpoint
                                            // 6
pub const USB_O_TXTYPE6 = usize(0x0000016A);  // USB Host Transmit Configure Type
                                            // Endpoint 6
pub const USB_O_TXINTERVAL6 = usize(0x0000016B);  // USB Host Transmit Interval
                                            // Endpoint 6
pub const USB_O_RXTYPE6 = usize(0x0000016C);  // USB Host Configure Receive Type
                                            // Endpoint 6
pub const USB_O_RXINTERVAL6 = usize(0x0000016D);  // USB Host Receive Polling
                                            // Interval Endpoint 6
pub const USB_O_TXMAXP7 = usize(0x00000170);  // USB Maximum Transmit Data
                                            // Endpoint 7
pub const USB_O_TXCSRL7 = usize(0x00000172);  // USB Transmit Control and Status
                                            // Endpoint 7 Low
pub const USB_O_TXCSRH7 = usize(0x00000173);  // USB Transmit Control and Status
                                            // Endpoint 7 High
pub const USB_O_RXMAXP7 = usize(0x00000174);  // USB Maximum Receive Data
                                            // Endpoint 7
pub const USB_O_RXCSRL7 = usize(0x00000176);  // USB Receive Control and Status
                                            // Endpoint 7 Low
pub const USB_O_RXCSRH7 = usize(0x00000177);  // USB Receive Control and Status
                                            // Endpoint 7 High
pub const USB_O_RXCOUNT7 = usize(0x00000178);  // USB Receive Byte Count Endpoint
                                            // 7
pub const USB_O_TXTYPE7 = usize(0x0000017A);  // USB Host Transmit Configure Type
                                            // Endpoint 7
pub const USB_O_TXINTERVAL7 = usize(0x0000017B);  // USB Host Transmit Interval
                                            // Endpoint 7
pub const USB_O_RXTYPE7 = usize(0x0000017C);  // USB Host Configure Receive Type
                                            // Endpoint 7
pub const USB_O_RXINTERVAL7 = usize(0x0000017D);  // USB Host Receive Polling
                                            // Interval Endpoint 7
pub const USB_O_DMAINTR = usize(0x00000200);  // USB DMA Interrupt
pub const USB_O_DMACTL0 = usize(0x00000204);  // USB DMA Control 0
pub const USB_O_DMAADDR0 = usize(0x00000208);  // USB DMA Address 0
pub const USB_O_DMACOUNT0 = usize(0x0000020C);  // USB DMA Count 0
pub const USB_O_DMACTL1 = usize(0x00000214);  // USB DMA Control 1
pub const USB_O_DMAADDR1 = usize(0x00000218);  // USB DMA Address 1
pub const USB_O_DMACOUNT1 = usize(0x0000021C);  // USB DMA Count 1
pub const USB_O_DMACTL2 = usize(0x00000224);  // USB DMA Control 2
pub const USB_O_DMAADDR2 = usize(0x00000228);  // USB DMA Address 2
pub const USB_O_DMACOUNT2 = usize(0x0000022C);  // USB DMA Count 2
pub const USB_O_DMACTL3 = usize(0x00000234);  // USB DMA Control 3
pub const USB_O_DMAADDR3 = usize(0x00000238);  // USB DMA Address 3
pub const USB_O_DMACOUNT3 = usize(0x0000023C);  // USB DMA Count 3
pub const USB_O_DMACTL4 = usize(0x00000244);  // USB DMA Control 4
pub const USB_O_DMAADDR4 = usize(0x00000248);  // USB DMA Address 4
pub const USB_O_DMACOUNT4 = usize(0x0000024C);  // USB DMA Count 4
pub const USB_O_DMACTL5 = usize(0x00000254);  // USB DMA Control 5
pub const USB_O_DMAADDR5 = usize(0x00000258);  // USB DMA Address 5
pub const USB_O_DMACOUNT5 = usize(0x0000025C);  // USB DMA Count 5
pub const USB_O_DMACTL6 = usize(0x00000264);  // USB DMA Control 6
pub const USB_O_DMAADDR6 = usize(0x00000268);  // USB DMA Address 6
pub const USB_O_DMACOUNT6 = usize(0x0000026C);  // USB DMA Count 6
pub const USB_O_DMACTL7 = usize(0x00000274);  // USB DMA Control 7
pub const USB_O_DMAADDR7 = usize(0x00000278);  // USB DMA Address 7
pub const USB_O_DMACOUNT7 = usize(0x0000027C);  // USB DMA Count 7
pub const USB_O_RQPKTCOUNT1 = usize(0x00000304);  // USB Request Packet Count in
                                            // Block Transfer Endpoint 1
pub const USB_O_RQPKTCOUNT2 = usize(0x00000308);  // USB Request Packet Count in
                                            // Block Transfer Endpoint 2
pub const USB_O_RQPKTCOUNT3 = usize(0x0000030C);  // USB Request Packet Count in
                                            // Block Transfer Endpoint 3
pub const USB_O_RQPKTCOUNT4 = usize(0x00000310);  // USB Request Packet Count in
                                            // Block Transfer Endpoint 4
pub const USB_O_RQPKTCOUNT5 = usize(0x00000314);  // USB Request Packet Count in
                                            // Block Transfer Endpoint 5
pub const USB_O_RQPKTCOUNT6 = usize(0x00000318);  // USB Request Packet Count in
                                            // Block Transfer Endpoint 6
pub const USB_O_RQPKTCOUNT7 = usize(0x0000031C);  // USB Request Packet Count in
                                            // Block Transfer Endpoint 7
pub const USB_O_RXDPKTBUFDIS = usize(0x00000340);  // USB Receive Double Packet Buffer
                                            // Disable
pub const USB_O_TXDPKTBUFDIS = usize(0x00000342);  // USB Transmit Double Packet
                                            // Buffer Disable
pub const USB_O_CTO = usize(0x00000344);  // USB Chirp Timeout
pub const USB_O_HHSRTN = usize(0x00000346);  // USB High Speed to UTM Operating
                                            // Delay
pub const USB_O_HSBT = usize(0x00000348);  // USB High Speed Time-out Adder
pub const USB_O_LPMATTR = usize(0x00000360);  // USB LPM Attributes
pub const USB_O_LPMCNTRL = usize(0x00000362);  // USB LPM Control
pub const USB_O_LPMIM = usize(0x00000363);  // USB LPM Interrupt Mask
pub const USB_O_LPMRIS = usize(0x00000364);  // USB LPM Raw Interrupt Status
pub const USB_O_LPMFADDR = usize(0x00000365);  // USB LPM Function Address
pub const USB_O_EPC = usize(0x00000400);  // USB External Power Control
pub const USB_O_EPCRIS = usize(0x00000404);  // USB External Power Control Raw
                                            // Interrupt Status
pub const USB_O_EPCIM = usize(0x00000408);  // USB External Power Control
                                            // Interrupt Mask
pub const USB_O_EPCISC = usize(0x0000040C);  // USB External Power Control
                                            // Interrupt Status and Clear
pub const USB_O_DRRIS = usize(0x00000410);  // USB Device RESUME Raw Interrupt
                                            // Status
pub const USB_O_DRIM = usize(0x00000414);  // USB Device RESUME Interrupt Mask
pub const USB_O_DRISC = usize(0x00000418);  // USB Device RESUME Interrupt
                                            // Status and Clear
pub const USB_O_GPCS = usize(0x0000041C);  // USB General-Purpose Control and
                                            // Status
pub const USB_O_VDC = usize(0x00000430);  // USB VBUS Droop Control
pub const USB_O_VDCRIS = usize(0x00000434);  // USB VBUS Droop Control Raw
                                            // Interrupt Status
pub const USB_O_VDCIM = usize(0x00000438);  // USB VBUS Droop Control Interrupt
                                            // Mask
pub const USB_O_VDCISC = usize(0x0000043C);  // USB VBUS Droop Control Interrupt
                                            // Status and Clear
pub const USB_O_IDVRIS = usize(0x00000444);  // USB ID Valid Detect Raw
                                            // Interrupt Status
pub const USB_O_IDVIM = usize(0x00000448);  // USB ID Valid Detect Interrupt
                                            // Mask
pub const USB_O_IDVISC = usize(0x0000044C);  // USB ID Valid Detect Interrupt
                                            // Status and Clear
pub const USB_O_DMASEL = usize(0x00000450);  // USB DMA Select
pub const USB_O_PP = usize(0x00000FC0);  // USB Peripheral Properties
pub const USB_O_PC = usize(0x00000FC4);  // USB Peripheral Configuration
pub const USB_O_CC = usize(0x00000FC8);  // USB Clock Configuration

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

