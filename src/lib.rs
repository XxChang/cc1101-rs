#![no_std]

extern crate embedded_hal as hal;

use core::fmt::Debug;
use hal::blocking::spi::{Transfer, Write};
use hal::blocking::delay::* ;
use hal::digital::v2::OutputPin;
use hal::digital::v2::InputPin;
use radio::{Registers, Channel};
use register::*;

pub mod transmit;
pub mod registers;
pub mod register;
pub mod channel;
pub mod state;
pub mod receive;

pub const FXOSC: u64 = 26_000_000 ;

pub struct CC1101<SPI, CS, GDO0, Delay> {
    spi: SPI,
    cs: CS,
    gd0: GDO0,
    delay: Delay,
}

pub enum Command {
    SRES    = 0x30, // Reset Chip
    SFSTXON = 0x31, // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA): Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
    SXOFF   = 0x32, // Turn off crystal oscillator.
    SCAL    = 0x33, // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
    SRX     = 0x34, // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
    STX     = 0x35, // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled: Only go to TX if channel is clear.
    SIDEL   = 0x36, // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
    SWOR    = 0x38, // Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0.
    SPWD    = 0x39, // Enter power down mode when CSn goes high.
    SFRX    = 0x3a, // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
    SFTX    = 0x3b, // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
    SWORRST = 0x3c, // Reset real time clock to Event1 value.
    SNOP    = 0x3d, // No operation. May be used to get access to the chip status byte.
}

#[derive(Clone, PartialEq, Debug)]
pub struct FskChannel {
    pub base_freq: u64,
    pub freq: u64,
}

impl Default for FskChannel {
    fn default() -> Self {
        FskChannel { base_freq: 400_000_000, freq: 430_000_000 }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct PktCfg {
    pub whitening_data: bool,
    pub crc_en: bool,
    pub length_mode: LengthCfg,
    pub addr: u8,
    pub addr_mode: AddrMode,
}

impl Default for PktCfg {
    fn default() -> Self {
        PktCfg { 
            whitening_data: false, 
            crc_en: true, 
            length_mode: LengthCfg::Variable,
            addr: 0x05,
            addr_mode: AddrMode::BoardAll, 
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct Deviation {
    pub devition_e: u8,
    pub devition_m: u8,
}

impl Default for Deviation {
    fn default() -> Self {
        Deviation { devition_e: 1, devition_m: 5 }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct ModemCfg {
    pub chanbw_e: u8,
    pub chanbw_m: u8,
    pub drate_e: u8,
    pub drate_m: u8,
    pub dem_dcfilt_off: bool,
    pub modulation: ModulationFormat,
    pub manchester_en: bool,
    pub sync_mode: SyncMode,
    pub chanspc_e: u8,
    pub num_preamble: u8,
    pub fec_en: bool,
}

impl Default for ModemCfg {
    fn default() -> Self {
        ModemCfg { 
            chanbw_e: 3, 
            chanbw_m: 3, 
            drate_e: 6, 
            drate_m: 67, 
            dem_dcfilt_off: false, 
            modulation: ModulationFormat::GFSK, 
            manchester_en: false,
            sync_mode: SyncMode::SyncWordDetectd_30_32,
            chanspc_e: 2,
            num_preamble: 7,
            fec_en: false,
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct SyncWord {
    pub high_byte: u8,
    pub low_byte: u8,
}

impl Default for SyncWord {
    fn default() -> Self {
        SyncWord { high_byte: 0x81, low_byte: 0x99 }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct MachineStateCfg {
    pub fs_autocal: FsAutoCal,
    pub po_timeout: u8,
    pub pin_ctrl_en: bool,
    pub xosc_force_en: bool,
    pub cca_mode: CcaMode,
    pub rxoff_mode: RxOffMode,
    pub txoff_mode: TxOffMode,
}

impl Default for MachineStateCfg {
    fn default() -> Self {
        MachineStateCfg { 
            fs_autocal: FsAutoCal::FromIdleToActive, 
            po_timeout: 2, 
            pin_ctrl_en: false, 
            xosc_force_en: false,
            cca_mode: CcaMode::RSSIThrReceivePkt,
            rxoff_mode: RxOffMode::TX,
            txoff_mode: TxOffMode::RX,
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct FreqOffsetCfg {
    pub foc_bs_cs_gate: bool,
    pub foc_pre_k: u8,
    pub foc_post_k: bool,
    pub foc_limit: u8,
}

impl Default for FreqOffsetCfg {
    fn default() -> Self {
        FreqOffsetCfg { 
            foc_bs_cs_gate: false, 
            foc_pre_k: 2, 
            foc_post_k: true, 
            foc_limit: 2 
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct WakeOnRadioCfg {
    pub rc_pd: bool,
    pub event1: u8,
    pub rc_cal: bool,
    pub wor_res: u8,
}

impl Default for WakeOnRadioCfg {
    fn default() -> Self {
        WakeOnRadioCfg { rc_pd: true, event1: 7, rc_cal: true, wor_res: 3 }
    }
}

pub struct Config {
    pub pkt_cfg: PktCfg,
    pub fifo_thr: FifoThrSelect,
    pub channel: FskChannel,
    pub modem: ModemCfg,
    pub deviation: Deviation,
    pub machine_cfg: MachineStateCfg,
    pub freq_offset_cfg: FreqOffsetCfg,
    pub wake_on_radio_cfg: WakeOnRadioCfg,
    pub sync_word: SyncWord,
}

impl Default for Config {
    fn default() -> Self {
        Config { 
            pkt_cfg: PktCfg::default(),
            channel: FskChannel::default(),
            fifo_thr: FifoThrSelect::TX_33_RX_32,
            modem: ModemCfg::default(),
            deviation: Deviation::default(),
            machine_cfg: MachineStateCfg::default(),
            freq_offset_cfg: FreqOffsetCfg::default(),
            wake_on_radio_cfg: WakeOnRadioCfg::default(),
            sync_word: SyncWord::default(),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum State {
    Sleep = 0x00,
    StandbyRc = 0x02,
    StandbyXosc = 0x03,
    Fs = 0x04,
    Rx = 0x05,
    Tx = 0x06,
    IDLE = 0x07,
}

impl radio::RadioState for State {
    fn idle() -> Self {
        Self::IDLE
    }

    fn sleep() -> Self {
        Self::Sleep
    }
}

/// Receive packet information
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PacketInfo {
    pub rssi: i16,
    pub rssi_sync: Option<i16>,
    pub snr: Option<i16>,
}

impl radio::ReceiveInfo for PacketInfo {
    fn rssi(&self) -> i16 {
        self.rssi
    }
}

impl Default for PacketInfo {
    fn default() -> Self {
        Self {
            rssi: -100,
            rssi_sync: None,
            snr: None,
        }
    }
}

impl<SPI, CS, GD0, Delay, SpiE, GpioE> CC1101<SPI, CS, GD0, Delay> 
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = GpioE>,
    GD0: InputPin<Error = GpioE>,
    Delay: DelayMs<u32> + DelayUs<u32>,
    SpiE: Debug,
    GpioE: Debug,
{
    pub fn new(spi: SPI, cs: CS, gd0: GD0, delay: Delay, config: &Config) -> Result<Self, Error<SpiE, GpioE>> {
        let mut cc1101 = CC1101 { spi, cs, gd0, delay } ;

        cc1101.reset()?;

        cc1101.delay.delay_us(40) ;

        cc1101.configure(config)?;

        Ok(cc1101)
    }

    pub fn cmd_strobe(&mut self, cmd: Command) -> Result<(), Error<SpiE,GpioE>> {
        self.cs.set_low().map_err(Error::Gpio)?;

        self.spi.write(&[cmd as u8]).map_err(Error::Spi)?;

        self.cs.set_high().map_err(Error::Gpio)?;

        Ok(())
    }

    pub fn info(&mut self) -> Result<(u8, u8), Error<SpiE, GpioE>> {
        Ok((
            self.read_register::<register::Partnum>().map(|r| r.part_no() )?,
            self.read_register::<register::Version>().map(|r| r.version() )?
        ))
    }

    pub fn reset(&mut self) -> Result<(), Error<SpiE, GpioE>> {
        self.cs.set_high().map_err(Error::Gpio)?;
        self.cs.set_low().map_err(Error::Gpio)?;
        self.cs.set_high().map_err(Error::Gpio)?;
        self.delay.delay_us(40) ;
        self.cmd_strobe(Command::SRES)
    }

    pub fn configure(&mut self, config: &Config) -> Result<(), Error<SpiE, GpioE>> {
        self.write_register::<IoCfg0>(IoCfg0::new().with_gdo0_cfg(GpioOutSelect::SyncWordState).with_gdo0_inv(1))?;
        self.set_fifo_thr(config.fifo_thr)?;
        self.set_pkt(&config.pkt_cfg)?;
        self.set_channel(&config.channel)?;
        self.set_modem(&config.modem)?;
        self.set_deviation(&config.deviation)?;
        self.set_state_machine(&config.machine_cfg)?;
        self.set_freq_offset(&config.freq_offset_cfg)?;
        self.set_wake_on_radio(&config.wake_on_radio_cfg)?;
        self.set_freq_synthe_cal()?;
        self.write_register::<Test2>(0x81u8.into())?;
        self.write_register::<Test1>(0x35u8.into())?;
        self.write_register::<Sync1>(config.sync_word.high_byte.into())?;
        self.write_register::<Sync0>(config.sync_word.low_byte.into())?;
        Ok(())
    }

    pub fn set_freq_synthe_cal(&mut self) -> Result<(), Error<SpiE, GpioE>> {
        self.write_register::<Fscal3>(0xe9u8.into())?;
        self.write_register::<Fscal2>(0x2au8.into())?;
        self.write_register::<Fscal1>(0x00u8.into())?;
        self.write_register::<Fscal0>(0x1fu8.into())?;
        Ok(())
    }

    pub fn get_freq_synthe_cal(&mut self) -> Result<(u8, u8, u8, u8), Error<SpiE, GpioE>> {
        Ok((
            self.read_register::<Fscal0>()?.into(),
            self.read_register::<Fscal1>()?.into(),
            self.read_register::<Fscal2>()?.into(),
            self.read_register::<Fscal3>()?.into(),
        ))
    }

    pub fn set_address(&mut self, addr: u8, addr_mode: AddrMode) -> Result<(), Error<SpiE, GpioE>> {
        let pkt_ctrl1: u8 = self.read_register::<PktCtrl1>()?.into() ;
        let pkt_ctrl1 = pkt_ctrl1 & !0x03u8 ;
        self.write_register::<Addr>(Addr::new().with_device_addr(addr))?;
        let pkt_ctrl1 = match addr_mode {
                        AddrMode::BoardAll => pkt_ctrl1,
                        AddrMode::BoardNo => pkt_ctrl1 | 0x01u8,
                        AddrMode::Board0 => pkt_ctrl1 | 0x02u8,
                        AddrMode::Board0And255 => pkt_ctrl1 | 0x03u8,
                    } ;
        self.write_register::<PktCtrl1>(pkt_ctrl1.into())?;
        Ok(())
    }

    pub fn get_address(&mut self) -> Result<u8, Error<SpiE, GpioE>> {
        Ok(
            self.read_register::<Addr>()?.into()
        )
    }

    pub fn set_wake_on_radio(&mut self, cfg: &WakeOnRadioCfg) -> Result<(), Error<SpiE, GpioE>> {
        let wake_on_radio_cfg = Worctrl::new().with_rc_pd(if cfg.rc_pd {1} else {0})
                                         .with_event1(cfg.event1)
                                         .with_rc_cal(if cfg.rc_cal {1} else {0})
                                         .with_wor_res(cfg.wor_res) ;
        self.write_register::<Worctrl>(wake_on_radio_cfg)?;
        Ok(())
    }

    pub fn get_wake_on_radio(&mut self) -> Result<u8, Error<SpiE, GpioE>> {
        Ok(self.read_register::<Worctrl>()?.into())
    }

    pub fn set_freq_offset(&mut self, cfg: &FreqOffsetCfg) -> Result<(), Error<SpiE, GpioE>> {
        let freq_offset_cfg = Foccfg::new().with_foc_limit(cfg.foc_limit)
                                      .with_foc_post_k(if cfg.foc_post_k {1} else {0})
                                      .with_foc_pre_k(cfg.foc_pre_k)
                                      .with_foc_bs_cs_gate(if cfg.foc_bs_cs_gate {1} else {0});
        self.write_register::<Foccfg>(freq_offset_cfg)?;
        Ok(())
    }

    pub fn get_freq_offset(&mut self) -> Result<u8, Error<SpiE, GpioE>> {
        Ok(self.read_register::<Foccfg>()?.into())
    }

    pub fn set_state_machine(&mut self, cfg: &MachineStateCfg) -> Result<(), Error<SpiE, GpioE>> {
        let mcsm0 = Mcsm0::new().with_fs_autocal(cfg.fs_autocal)
                           .with_po_timeout(cfg.po_timeout)
                           .with_pin_ctrl_en(if cfg.pin_ctrl_en {1} else {0})
                           .with_xosc_force_on(if cfg.xosc_force_en {1} else {0}) ;
        self.write_register::<Mcsm0>(mcsm0)?;

        let mcsm1 = Mcsm1::new().with_cca_mode(cfg.cca_mode).with_rxoff_mode(cfg.rxoff_mode).with_txoff_mode(cfg.txoff_mode);
        self.write_register::<Mcsm1>(mcsm1)?;

        Ok(())
    }

    pub fn get_state_machine_cfg(&mut self) -> Result<(u8, u8), Error<SpiE, GpioE>> {
        Ok((
            self.read_register::<Mcsm0>()?.into(),
            self.read_register::<Mcsm1>()?.into(),
        ))
    }

    pub fn set_deviation(&mut self, value: &Deviation) -> Result<(), Error<SpiE, GpioE>> {
        let deviatn_e = value.devition_e ;
        let deviatn_m = value.devition_m ;
        self.write_register::<Deviatn>(Deviatn::new().with_deviaion_e(deviatn_e).with_deviation_m(deviatn_m as u8))?;
        Ok(())
    }

    pub fn get_deviation(&mut self) -> Result<f32, Error<SpiE, GpioE>> {
        let deviatn = self.read_register::<Deviatn>()?;
        let deviatn = (FXOSC as f32) / (1u32.rotate_left(17) as f32)*(8f32 + (deviatn.deviation_m() as f32))*(1u32.rotate_left(deviatn.deviaion_e() as u32) as f32); 
        Ok(deviatn)
    }

    pub fn set_modem(&mut self, cfg: &ModemCfg) -> Result<(), Error<SpiE, GpioE>> {
        let chanbw_e = cfg.chanbw_e as u64 ;
        let chanbw_m = cfg.chanbw_m ;

        let drate_e = cfg.drate_e ;
        let drate_m = cfg.drate_m ;
        self.write_register::<MdmCfg4>(MdmCfg4::new().with_chanbw_e(chanbw_e as u8).with_chanbw_m(chanbw_m).with_drate_e(drate_e))?;
        self.write_register::<MdmCfg3>(MdmCfg3::new().with_drate_m(drate_m))?;

        let mdm_cfg2 = MdmCfg2::new().with_sync_mode(cfg.sync_mode)
                                .with_manchester_en(if cfg.manchester_en {1} else {0})
                                .with_mod_format(cfg.modulation)
                                .with_dem_dcfilt_off(if cfg.dem_dcfilt_off {1} else {0});
        self.write_register::<MdmCfg2>(mdm_cfg2)?;

        let mdm_cfg1 = MdmCfg1::new().with_chanspc_e(cfg.chanspc_e)
                                .with_num_preamble(cfg.num_preamble)
                                .with_fec_en(if cfg.fec_en {1} else {0}) ;
        self.write_register::<MdmCfg1>(mdm_cfg1)?;

        Ok(())
    }

    pub fn get_modem(&mut self) -> Result<(f32, f32, u8, u8), Error<SpiE, GpioE>> {
        let mdm_cfg4 = self.read_register::<MdmCfg4>()?;
        let mdm_cfg3 = self.read_register::<MdmCfg3>()?;
        let mdm_cfg2 = self.read_register::<MdmCfg2>()?;
        let mdm_cfg2: u8 = mdm_cfg2.into() ;
        let chanbw_e = mdm_cfg4.chanbw_e() as u32 ;
        let chanbw_m = mdm_cfg4.chanbw_m() as f32 ;

        let chanbw = (FXOSC as f32) / (8f32*(4f32+chanbw_m)*(1u32.rotate_left(chanbw_e) as f32) );

        let drate_m = mdm_cfg3.drate_m() as f32 ;
        let drate_e = mdm_cfg4.drate_e() as u32 ;

        let drate = (FXOSC as f32) * (256f32 + drate_m) * (1u32.rotate_left(drate_e) as f32) / (1u32.rotate_left(28) as f32) ;
        Ok((chanbw, drate, mdm_cfg2, self.read_register::<MdmCfg1>()?.into()))
    }

    pub fn set_pkt(&mut self, cfg: &PktCfg) -> Result<(), Error<SpiE, GpioE>> {
        let mut pkt_cfg0 = PktCtrl0::new() ;
        pkt_cfg0 = if cfg.crc_en { pkt_cfg0.with_crc_en(1) } else { pkt_cfg0.with_crc_en(0) } ;
        pkt_cfg0 = if cfg.whitening_data { pkt_cfg0.with_white_data(1) } else { pkt_cfg0.with_white_data(0) } ;
        pkt_cfg0 = pkt_cfg0.with_length_config(cfg.length_mode) ;
        self.write_register::<PktCtrl0>(pkt_cfg0)?;
        self.set_address(cfg.addr, cfg.addr_mode)?;
        Ok(())
    }

    pub fn get_pkt(&mut self) -> Result<(u8, AddrMode), Error<SpiE, GpioE>> {
        Ok((
            self.read_register::<PktCtrl0>()?.into(),
            self.read_register::<PktCtrl1>()?.adr_chk(),
        ))
    }

    pub fn set_fifo_thr(&mut self, thr: FifoThrSelect) -> Result<(), Error<SpiE, GpioE>> {
        self.write_register::<FifoThr>(FifoThr::new().with_fifo_thr(thr).with_adc_retention(1))?;
        Ok(())
    }

    pub fn get_fifo_thr(&mut self) -> Result<FifoThrSelect, Error<SpiE, GpioE>> {
        let fifo_thr = self.read_register::<FifoThr>().map(|r| r.fifo_thr())?;
        Ok(fifo_thr)
    }

    pub fn set_base_frequency(&mut self, f: u64) -> Result<(u8, u8, u8), Error<SpiE, GpioE>> {
        let (freq0, freq1, freq2) = from_frequency(f) ;
        self.write_register::<Freq0>(freq0.into())?;
        self.write_register::<Freq1>(freq1.into())?;
        self.write_register::<Freq2>(freq2.into())?;
        Ok((freq0, freq1, freq2))
    }

    pub fn get_base_frequency(&mut self) -> Result<u64, Error<SpiE, GpioE>> {
        let freq0 = self.read_register::<Freq0>().map(|r| r.freq())?;
        let freq1 = self.read_register::<Freq1>().map(|r| r.freq())?;
        let freq2 = self.read_register::<Freq2>().map(|r| r.freq())?;
        let tmp_v = ((freq2 as u64) << 16) | ((freq1 as u64) << 8) | (freq0 as u64) ;
        Ok(tmp_v * FXOSC / 1u64.rotate_left(16))
    }

    pub fn set_chan(&mut self, ch: u8) -> Result<(), Error<SpiE, GpioE>> {
        self.write_register::<Channr>(ch.into())?;
        Ok(())
    }

    pub fn get_chan(&mut self) -> Result<u8, Error<SpiE, GpioE>> {
        let chan = self.read_register::<Channr>().map(|r| r.chan())?;
        Ok(chan)
    }

    pub fn get_chanspc(&mut self) -> Result<u32, Error<SpiE, GpioE>> {
        let chanspc_e = self.read_register::<MdmCfg1>().map(|r| r.chanspc_e())?;
        let chanspc_m = self.read_register::<MdmCfg0>().map(|r| r.chanspc_m())?;
        let chanspc = (256 + (chanspc_m as u32)) * 1u32.rotate_left( (chanspc_e - 2) as u32) ;
        Ok(chanspc)
    }
}

impl <SPI, CS, GD0, Delay> DelayUs<u32> for CC1101<SPI, CS, GD0, Delay>
where
    Delay: DelayMs<u32> + DelayUs<u32>
{
    fn delay_us(&mut self, us: u32) {
        self.delay.delay_us(us)
    }
}

#[derive(Debug)]
pub enum Error<SpiE, GpioE> {
    Spi(SpiE),
    Gpio(GpioE),

    #[cfg_attr(feature = "thiserror", error("Unexpected register value (reg: 0x{0:02x} val: 0x:{1:02x}"))]
    UnexpectedValue(u8, u8),

    #[cfg_attr(feature = "thiserror", error("Unexpected register value (reg: 0x{0:02x} val: 0x:{1:02x}"))]
    UnexpectedValueU64(u8, u64),

    #[cfg_attr(feature = "thiserror", error("invalid frequency or frequency out of range"))]
    InvalidFrequency,

    #[cfg_attr(feature = "thiserror", error("invalid state command"))]
    /// Invalid state command
    InvalidStateCommand,

    #[cfg_attr(feature = "thiserror", error("invalid message length"))]
    /// Invalid message length
    InvalidLength,
}

pub const fn from_frequency(hz: u64) -> (u8, u8, u8) {
    let freq = hz * 1u64.rotate_left(16) / FXOSC;
    let freq0 = (freq & 0xff) as u8;
    let freq1 = ((freq >> 8) & 0xff) as u8;
    let freq2 = ((freq >> 16) & 0xff) as u8;
    (freq0, freq1, freq2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {

    }
}
