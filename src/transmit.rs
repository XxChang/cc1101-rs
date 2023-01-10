use core::fmt::Debug;
use hal::blocking::spi::{Transfer, Write};
use hal::blocking::delay::* ;
use hal::digital::v2::{OutputPin, InputPin};
use radio::{State, Registers};
use crate::register::*;

use crate::CC1101;
use crate::Error;
use crate::register::AddrMode;

impl<SPI, CS, GD0, Delay, SpiE, GpioE> radio::Transmit for CC1101<SPI, CS, GD0, Delay>
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = GpioE>,
    GD0: InputPin<Error = GpioE>,
    Delay: DelayMs<u32> + DelayUs<u32>,
    SpiE: Debug,
    GpioE: Debug,    
{
    type Error = Error<SpiE, GpioE>;

    fn start_transmit(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.clear_tx_fifo()?;
        
        let (_, addr_mode) = self.get_pkt()?;
        
        match addr_mode {
            AddrMode::BoardAll => {
                let len = data.len() as u8 ;
                self.write_register::<TxFifo>(len.into())? ;
            },
            _ => {
                return Err(Error::InvalidStateCommand);
            }
        };

        self.write_brust_register::<TxFifo>(data)?;
        self.set_state(crate::State::Tx)?;
        Ok(())
    }

    fn check_transmit(&mut self) -> Result<bool, Self::Error> {
        self.gd0.is_low().map_err(Error::Gpio)
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
    fn clear_tx_fifo(&mut self) -> Result<(), Error<SpiE, GpioE>> {
        self.set_state(crate::State::IDLE)?;
        self.cmd_strobe(crate::Command::SFTX)?;
        Ok(())
    }
}
