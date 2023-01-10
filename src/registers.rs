use crate::CC1101;
use crate::Error;

use core::fmt::Debug;
use hal::blocking::spi::{Transfer, Write};
use hal::blocking::delay::* ;
use hal::digital::v2::OutputPin;

use radio::Registers;

impl<SPI, CS, Delay, GD0, SpiE, GpioE> Registers<u8> for CC1101<SPI, CS, GD0, Delay>
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

impl<SPI, CS, GD0, Delay, SpiE, GpioE> CC1101<SPI, CS, GD0, Delay>
where 
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = GpioE>,
    Delay: DelayMs<u32> + DelayUs<u32>,
    SpiE: Debug,
    GpioE: Debug,
{
    pub fn write_brust_register<R: radio::Register<Word = u8>>(&mut self, value: &[u8]) -> Result<(), Error<SpiE, GpioE>> {
        self.cs.set_low().map_err(Error::Gpio)?;
        
        let mut addr = [ 0x40 | R::ADDRESS] ;

        self.spi.transfer(&mut addr).map_err(Error::Spi)?;
        self.spi.write(value).map_err(Error::Spi)?;

        self.cs.set_high().map_err(Error::Gpio)?;

        Ok(())
    }

    pub fn read_brust_register<R: radio::Register<Word = u8>>(&mut self, buff: &mut [u8]) -> Result<(), Error<SpiE, GpioE>> {
        self.cs.set_low().map_err(Error::Gpio)?;
        
        let mut addr = [ 0xC0 | R::ADDRESS] ;

        self.spi.transfer(&mut addr).map_err(Error::Spi)?;
        
        for b in buff {
            for _ in 1..20 {

            }
            let mut tmp = [0u8] ;
            self.spi.transfer(&mut tmp).map_err(Error::Spi)?;
            *b = tmp[0] ;
        }

        self.cs.set_high().map_err(Error::Gpio)?;
        
        Ok(())
    }
}
