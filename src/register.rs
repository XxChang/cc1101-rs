use core::convert::Infallible;

use modular_bitfield::prelude::*;
use radio::Register;

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u8)]
pub struct IoCfg2 {
    pub gdo2_cfg: GpioOutSelect,
    pub gdo2_inv: B1,
    #[skip]
    __: B1,
}

impl Register for IoCfg2 {
    const ADDRESS: u8 = Registers::IOCFG2 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u8)]
pub struct IoCfg1 {
    pub gdo1_cfg: GpioOutSelect,
    pub gdo1_inv: B1,
    #[skip]
    __: B1,
}

impl Register for IoCfg1 {
    const ADDRESS: u8 = Registers::IOCFG1 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u8)]
pub struct IoCfg0 {
    pub gdo0_cfg: GpioOutSelect,
    pub gdo0_inv: B1,
    #[skip]
    __: B1,
}

impl Register for IoCfg0 {
    const ADDRESS: u8 = Registers::IOCFG0 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u8)]
pub struct FifoThr {
    pub fifo_thr: FifoThrSelect,
    pub close_in_rx: RxAttenuationValue,
    pub adc_retention: B1,
    #[skip]
    __: B1,
}

impl Register for FifoThr {
    const ADDRESS: u8 = Registers::FIFOTHR as u8 ;
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
pub struct PktCtrl1 {
    pub adr_chk: B2,
    pub append_status: B1,
    pub crc_autoflush: B1,
    #[skip]
    __: B1,
    pub pqt: B3,
}

impl Register for PktCtrl1 {
    const ADDRESS: u8 = Registers::PKTCTRL1 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u8)]
pub struct PktCtrl0 {
    pub length_config: LengthCfg,
    pub crc_en: B1,
    #[skip]
    __: B1,
    pub pkt_format: PktFormat,
    pub white_data: B1,
    #[skip]
    __: B1,
}

impl Register for PktCtrl0 {
    const ADDRESS: u8 = Registers::PKTCTRL0 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct Channr {
    pub chan: u8,
}

impl Register for Channr {
    const ADDRESS: u8 = Registers::CHANNR as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct FsCtrl1 {
    pub freq_if: B5,
    #[skip]
    __: B3,
}

impl Register for FsCtrl1 {
    const ADDRESS: u8 = Registers::FSCTRL1 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct FsCtrl0 {
    pub freqoff: u8,
}

impl Register for FsCtrl0 {
    const ADDRESS: u8 = Registers::FSCTRL0 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct Freq2 {
    pub freq: B6,
    #[skip]
    __: B2,
}

impl Register for Freq2 {
    const ADDRESS: u8 = Registers::FREQ2 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct Freq1 {
    pub freq: u8,
}

impl Register for Freq1 {
    const ADDRESS: u8 = Registers::FREQ1 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct Freq0 {
    pub freq: u8,
}

impl Register for Freq0 {
    const ADDRESS: u8 = Registers::FREQ0 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct MdmCfg4 {
    pub drate_e: B4,
    pub chanbw_m: B2,
    pub chanbw_e: B2,
}

impl Register for MdmCfg4 {
    const ADDRESS: u8 = Registers::MDMCFG4 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct MdmCfg3 {
    pub drate_m: u8,
}

impl Register for MdmCfg3 {
    const ADDRESS: u8 = Registers::MDMCFG3 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct MdmCfg2 {
    pub sync_mode: SyncMode,
    pub manchester_en: B1,
    pub mod_format: ModulationFormat,
    pub dem_dcfilt_off: B1,
}

impl Register for MdmCfg2 {
    const ADDRESS: u8 = Registers::MDMCFG2 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct MdmCfg1 {
    pub chanspc_e: B2,
    #[skip]
    __: B2,
    pub num_preamble: B3,
    pub fec_en: B1,
}

impl Register for MdmCfg1 {
    const ADDRESS: u8 = Registers::MDMCFG1 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct MdmCfg0 {
    pub chanspc_m: u8,
}

impl Register for MdmCfg0 {
    const ADDRESS: u8 = Registers::MDMCFG0 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub struct Deviatn {
    pub deviation_m: B3,
    #[skip]
    __: B1,
    pub deviaion_e: B3,
    #[skip]
    __: B1,
}

impl Register for Deviatn {
    const ADDRESS: u8 = Registers::DEVIATN as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u8)]
pub struct Mcsm0 {
    pub xosc_force_on: B1,
    pub pin_ctrl_en: B1,
    pub po_timeout: B2,
    pub fs_autocal: B2,
    #[skip]
    __: B2,
}

impl Register for Mcsm0 {
    const ADDRESS: u8 = Registers::MCSM0 as u8 ;
    type Word = u8 ;
    type Error = Infallible ;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u8)]
pub struct Foccfg {
    pub foc_limit: B2,
    pub foc_post_k: B1,
    pub foc_pre_k: B2,
    pub foc_bs_cs_gate: B1,
    #[skip]
    __: B2,
}

impl Register for Foccfg {
    const ADDRESS: u8 = Registers::FOCCFG as u8 ;
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

#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum LengthCfg {
    Fixed = 0,
    Variable = 1,
    Infinite = 2,
}

#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum PktFormat {
    NormalMode = 0,
    SyncSerialMode = 1,
    RandomTxMode = 2,
    AsyncSerialMode = 3,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 4]
pub enum FifoThrSelect {
    TX_61_RX_4  = 0,
    TX_57_RX_8  = 1,
    TX_53_RX_12 = 2,
    TX_49_RX_16 = 3,
    TX_45_RX_20 = 4,
    TX_41_RX_24 = 5,
    TX_37_RX_28 = 6,
    TX_33_RX_32 = 7,
    TX_29_RX_36 = 8,
    TX_25_RX_40 = 9,
    TX_21_RX_44 = 10,
    TX_17_RX_48 = 11,
    TX_13_RX_52 = 12,
    TX_9_RX_56  = 13,
    TX_5_RX_60  = 14,
    TX_1_RX_64  = 15,
}

#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 3]
pub enum ModulationFormat {
    BinaryFSK = 0,
    GFSK = 1,
    ASK = 3,
    MSK = 7,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 3]
pub enum SyncMode {
    NoPreamble = 0,
    SyncWordDetectd_15_16 = 1,
    SyncWordDetectd_16_16 = 2,
    SyncWordDetectd_30_32 = 3,
    AboveThr = 4,
    AboveThr_15_16 = 5,
    AboveThr_16_16 = 6,
    AboveThr_30_32 = 7,
}

#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum RxAttenuationValue {
    ZerodB      = 0,
    SixdB       = 1,
    TwelvedB    = 2,
    EighteendB  = 3,
}
