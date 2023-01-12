use core::fmt::Debug;
use hal::blocking::spi::{Transfer, Write};
use hal::blocking::delay::* ;
use hal::digital::v2::OutputPin;
use hal::digital::v2::InputPin;
use crate::CC1101State;
use crate::{CC1101, Command};
use crate::Error;

impl<SPI, CS, GD0, Delay, SpiE, GpioE> radio::State for CC1101<SPI, CS, GD0, Delay>
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = GpioE>,
    GD0: InputPin<Error = GpioE>,
    Delay: DelayMs<u32> + DelayUs<u32>,
    SpiE: Debug,
    GpioE: Debug,  
{
    type Error = Error<SpiE, GpioE> ;
    type State = CC1101State ;

    fn set_state(&mut self, state: Self::State) -> Result<(), Self::Error> {
        let command = match state {
            CC1101State::IDLE => Command::SIDEL,
            CC1101State::Tx => {
                Command::STX
            },
            CC1101State::Rx => {
                Command::SRX
            },
            _ => return Err(Error::InvalidStateCommand),
        };
        self.cmd_strobe(command)?;
        Ok(())
    }

    fn get_state(&mut self) -> Result<Self::State, Self::Error> {
        todo!()
    }
}
