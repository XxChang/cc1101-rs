#![no_std]

extern crate embedded_hal as hal;

use core::fmt::Debug;
use hal::blocking::spi::{Transfer, Write};
use hal::blocking::delay::* ;
use hal::digital::v2::OutputPin;
use radio::{Registers, Channel};
use register::{Freq0, Freq1, Freq2, Channr, MdmCfg1, MdmCfg0, FifoThrSelect, FifoThr, LengthCfg, PktCtrl0, PktFormat, MdmCfg4, MdmCfg3, ModulationFormat, SyncMode};

pub mod registers;
pub mod register;
pub mod channel;

pub const FXOSC: u64 = 26_000_000 ;

pub struct CC1101<SPI, CS, Delay> {
    spi: SPI,
    cs: CS,
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
}

impl Default for PktCfg {
    fn default() -> Self {
        PktCfg { whitening_data: false, crc_en: true, length_mode: LengthCfg::Variable }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct ModemCfg {
    pub chanbw_e: u8,
    pub chanbw: u64,
    pub drate_e: u8,
    pub drate: u64,
    pub dem_dcfilt_off: bool,
    pub modulation: ModulationFormat,
    pub manchester_en: bool,
    pub sync_mode: SyncMode,
}

impl Default for ModemCfg {
    fn default() -> Self {
        ModemCfg { 
            chanbw_e: 3, 
            chanbw: 58000, 
            drate_e: 6, 
            drate: 2003, 
            dem_dcfilt_off: false, 
            modulation: ModulationFormat::GFSK, 
            manchester_en: false,
            sync_mode: SyncMode::SyncWordDetectd_30_32,
        }
    }
}

pub struct Config {
    pub pkt_cfg: PktCfg,
    pub fifo_thr: FifoThrSelect,
    pub channel: FskChannel,
    pub modem: ModemCfg,
}

impl Default for Config {
    fn default() -> Self {
        Config { 
            pkt_cfg: PktCfg::default(),
            channel: FskChannel::default(),
            fifo_thr: FifoThrSelect::TX_33_RX_32,
            modem: ModemCfg::default(),
        }
    }
}

impl<SPI, CS, Delay, SpiE, GpioE> CC1101<SPI, CS, Delay> 
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = GpioE>,
    Delay: DelayMs<u32> + DelayUs<u32>,
    SpiE: Debug,
    GpioE: Debug,
{
    pub fn new(spi: SPI, cs: CS, delay: Delay, config: &Config) -> Result<Self, Error<SpiE, GpioE>> {
        let mut cc1101 = CC1101 { spi, cs, delay } ;

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
        self.set_fifo_thr(config.fifo_thr)?;
        self.set_pkt(&config.pkt_cfg)?;
        self.set_channel(&config.channel)?;
        self.set_modem(&config.modem)?;
        Ok(())
    }

    pub fn set_modem(&mut self, cfg: &ModemCfg) -> Result<(), Error<SpiE, GpioE>> {
        let chanbw_e = cfg.chanbw_e as u64 ;
        let chanbw_m = FXOSC / (cfg.chanbw * 8 * 1u64.rotate_left(chanbw_e as u32)) - 4 ;
        let chanbw_m = chanbw_m as u8 ;

        let drate_e = cfg.drate_e ;
        let drate_m = (cfg.drate as u64)*1u64.rotate_left(28)/( FXOSC * 1u64.rotate_left(drate_e as u32) ) - 256 ;
        let drate_m = drate_m as u8 ;
        self.write_register::<MdmCfg4>(MdmCfg4::new().with_chanbw_e(chanbw_e as u8).with_chanbw_m(chanbw_m).with_drate_e(drate_e))?;
        self.write_register::<MdmCfg3>(MdmCfg3::new().with_drate_m(drate_m))?;
        Ok(())
    }

    pub fn get_modem(&mut self) -> Result<(u8, u8, u8, u8), Error<SpiE, GpioE>> {
        let mdm_cfg4 = self.read_register::<MdmCfg4>()?;
        let mdm_cfg3 = self.read_register::<MdmCfg3>()?;
        Ok((mdm_cfg4.chanbw_e(), mdm_cfg4.chanbw_m(), mdm_cfg4.drate_e(), mdm_cfg3.drate_m()))
    }

    pub fn set_pkt(&mut self, cfg: &PktCfg) -> Result<(), Error<SpiE, GpioE>> {
        let mut pkt_cfg0 = PktCtrl0::new() ;
        pkt_cfg0 = if cfg.crc_en { pkt_cfg0.with_crc_en(1) } else { pkt_cfg0.with_crc_en(0) } ;
        pkt_cfg0 = if cfg.whitening_data { pkt_cfg0.with_white_data(1) } else { pkt_cfg0.with_white_data(0) } ;
        pkt_cfg0 = pkt_cfg0.with_length_config(cfg.length_mode) ;
        self.write_register::<PktCtrl0>(pkt_cfg0)?;
        Ok(())
    }

    pub fn get_pkt(&mut self) -> Result<u8, Error<SpiE, GpioE>> {
        let value: u8 = self.read_register::<PktCtrl0>().map(|r| r.into())?;
        Ok(value)
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
