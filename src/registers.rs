use crate::CC1101;
use crate::Error;

use core::fmt::Debug;
use hal::blocking::spi::{Transfer, Write};
use hal::blocking::delay::* ;
use hal::digital::v2::OutputPin;

use radio::Registers;

impl<SPI, CS, Delay, SpiE, GpioE> Registers<u8> for CC1101<SPI, CS, Delay>
where 
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = GpioE>,
    Delay: DelayMs<u32> + DelayUs<u32>,
    SpiE: Debug,
    GpioE: Debug,
{
    type Error = Error<SpiE, GpioE>;

    fn read_register<R: radio::Register<Word = u8>>(&mut self) -> Result<R, Self::Error> {
        self.cs.set_low().map_err(Error::Gpio)?;
        
        let mut buffer = [ 0x80 | R::ADDRESS, 0u8] ;
        self.spi.transfer(&mut buffer).map_err(Error::Spi)?;

        self.cs.set_high().map_err(Error::Gpio)?;

        R::try_from(buffer[1]).map_err(|_| Error::UnexpectedValue(R::ADDRESS, buffer[1]))
    }

    fn write_register<R: radio::Register<Word = u8>>(&mut self, value: R) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(Error::Gpio)?;
        
        self.spi.write(&mut [R::ADDRESS as u8, value.into()]).map_err(Error::Spi)?;

        self.cs.set_high().map_err(Error::Gpio)?;
        
        Ok(())
    }
}
