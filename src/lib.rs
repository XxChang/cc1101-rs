#![no_std]

extern crate embedded_hal as hal;

use core::fmt::Debug;
use hal::blocking::spi::{Transfer, Write};
use hal::blocking::delay::* ;
use hal::digital::v2::OutputPin;
use radio::Registers;

pub mod registers;
pub mod register;

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

impl<SPI, CS, Delay, SpiE, GpioE> CC1101<SPI, CS, Delay> 
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = GpioE>,
    Delay: DelayMs<u32> + DelayUs<u32>,
    SpiE: Debug,
    GpioE: Debug,
{
    pub fn new(spi: SPI, cs: CS, delay: Delay) -> Result<Self, Error<SpiE, GpioE>> {
        let mut cc1101 = CC1101 { spi, cs, delay } ;

        cc1101.reset()?;

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

}

#[derive(Debug)]
pub enum Error<SpiE, GpioE> {
    Spi(SpiE),
    Gpio(GpioE),

    #[cfg_attr(feature = "thiserror", error("Unexpected register value (reg: 0x{0:02x} val: 0x:{1:02x}"))]
    UnexpectedValue(u8, u8),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {

    }
}
