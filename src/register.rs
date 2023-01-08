use core::convert::Infallible;

use modular_bitfield::prelude::*;
use radio::Register;

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u8)]
pub struct IoCfg2 {
    #[skip]
    __: B1,
    pub gdo2_inv: B1,
    pub gdo2_cfg: GpioOutSelect,
}

impl Register for IoCfg2 {
    const ADDRESS: u8 = Registers::IOCFG2 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u8)]
pub struct Pktlen {
    pub packet_length: u8,
}

impl Register for Pktlen {
    const ADDRESS: u8 = Registers::PKTLEN as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u8)]
pub struct Partnum {
    pub part_no: u8
}

impl Register for Partnum {
    const ADDRESS: u8 = Registers::PARTNUM as u8 | 0x40u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u8)]
pub struct Version {
    pub version: u8
}

impl Register for Version {
    const ADDRESS: u8 = Registers::VERSION as u8 | 0x40u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[derive(Copy, Clone, Debug)]
#[allow(non_camel_case_types)]
pub enum Registers {
    IOCFG2      = 0x00,
    IOCFG1      = 0x01,
    IOCFG0      = 0x02,
    FIFOTHR     = 0x03,
    SYNC1       = 0x04,
    SYNC0       = 0x05,
    PKTLEN      = 0x06,
    PKTCTRL1    = 0x07,
    PKTCTRL0    = 0x08,
    ADDR        = 0x09,
    CHANNR      = 0x0a,
    FSCTRL1     = 0x0b,
    FSCTRL0     = 0x0c,
    FREQ2       = 0x0d,
    FREQ1       = 0x0e,
    FREQ0       = 0x0f,
    MDMCFG4     = 0x10,
    MDMCFG3     = 0x11,
    MDMCFG2     = 0x12,
    MDMCFG1     = 0x13,
    MDMCFG0     = 0x14,
    DEVIATN     = 0x15,
    MCSM2       = 0x16,
    MCSM1       = 0x17,
    MCSM0       = 0x18,
    FOCCFG      = 0x19,
    BSCFG       = 0x1a,
    AGCCTRL2    = 0x1b,
    AGCCTRL1    = 0x1c,
    AGCCTRL0    = 0x1d,
    WOREVT1     = 0x1e,
    WOREVT0     = 0x1f,
    WORCTRL     = 0x20,
    FREND1      = 0x21,
    FREND0      = 0x22,
    FSCAL3      = 0x23,
    FSCAL2      = 0x24,
    FSCAL1      = 0x25,
    FSCAL0      = 0x26,
    RCCTRL1     = 0x27,
    RCCTRL0     = 0x28,
    FSTEST      = 0x29,
    PTEST       = 0x2a,
    AGCTEST     = 0x2b,
    TEST2       = 0x2c,
    TEST1       = 0x2d,
    TEST0       = 0x2e,
    /// Chip part number
    PARTNUM     = 0x30,
    VERSION     = 0x31,
}

/// GPIO Pin Operation
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 6]
pub enum GpioOutSelect {
    RxFifoThr       = 0x00, // Associated to the RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold. De-asserts when RX FIFO is drained below the same threshold.
    RxFifoEmpty     = 0x01, // Associated to the RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold or the end of packet is reached. De-asserts when the RX FIFO is empty.
    TxFifoThr       = 0x02, // Associated to the TX FIFO: Asserts when the TX FIFO is filled at or above the TX FIFO threshold. De-asserts when the TX FIFO is below the same threshold.
    TxFifoEmpty     = 0x03, // Associated to the TX FIFO: Asserts when TX FIFO is full. De-asserts when the TX FIFO is drained below theTX FIFO threshold.
    RxFifoOverflow  = 0x04, // Asserts when the RX FIFO has overflowed. De-asserts when the FIFO has been flushed.
    TxFifoOverflow  = 0x05, // Asserts when the TX FIFO has underflowed. De-asserts when the FIFO is flushed. 
    SyncWordState   = 0x06, // Asserts when sync word has been sent / received, and de-asserts at the end of the packet. In RX, the pin will de-assert when the optional address check fails or the RX FIFO overflows. In TX the pin will de-assert if the TX FIFO underflows.
    
    /** NOTE:
     *  There are 3 GDO pins, but only one CLK_XOSC/n can be selected as an output at any
     *  time. If CLK_XOSC/n is to be monitored on one of the GDO pins, the other two GDO 
     *  pins must be configured to values less than 0x30. The GDO0 default value is CLK_XOSC/192.
     * 
     *  To optimize rf performance, these signal should not be used while the radio is in RX or TX mode.
     */
    ClkXoscDiv1     = 0x30,
    ClkXoscDiv1p5   = 0x31,
    ClkXoscDiv2     = 0x32,
    ClkXoscDiv3     = 0x33,
    ClkXoscDiv4     = 0x34,
    ClkXoscDiv6     = 0x35,
    ClkXoscDiv8     = 0x36,
    ClkXoscDiv12    = 0x37,
    ClkXoscDiv16    = 0x38,
    ClkXoscDiv24    = 0x39,
    ClkXoscDiv32    = 0x3a,
    ClkXoscDiv48    = 0x3b,
    ClkXoscDiv64    = 0x3c,
    ClkXoscDiv96    = 0x3d,
    ClkXoscDiv128   = 0x3e,
    ClkXoscDiv192   = 0x3f,
}
