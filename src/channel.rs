use radio::Channel;
use crate::FskChannel;
use crate::FXOSC;
use crate::Error;

use core::fmt::Debug;

use hal::blocking::spi::{Transfer, Write};
use hal::blocking::delay::* ;
use hal::digital::v2::OutputPin;

use crate::CC1101;

impl<SPI, CS, Delay, SpiE, GpioE> Channel for CC1101<SPI, CS, Delay>
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = GpioE>,
    Delay: DelayMs<u32> + DelayUs<u32>,
    SpiE: Debug,
    GpioE: Debug,
{
    type Channel = FskChannel ;
    type Error = Error<SpiE, GpioE> ;

    fn set_channel(&mut self, channel: &Self::Channel) -> Result<(), Self::Error> {
        
        let chanspc = self.get_chanspc()? as u64;
        let freq = channel.freq as u64 ;
        
        let chan = (
            (freq - (channel.base_freq as u64)) * 1u64.rotate_left(16) / (chanspc * FXOSC)
        ) as u8;
        
        if chanspc != 504 {
            return Err(Error::UnexpectedValueU64(0, chanspc));
        }
        self.set_base_frequency(channel.base_freq as u64)?;
        self.set_chan(chan)?;
        Ok(())
    }
}

