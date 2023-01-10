use core::fmt::Debug;
use hal::blocking::spi::{Transfer, Write};
use hal::blocking::delay::* ;
use hal::digital::v2::OutputPin;
use hal::digital::v2::InputPin;
use radio::{Registers, State};
use crate::{register::*, PacketInfo};
use crate::CC1101;
use crate::Error;

impl<SPI, CS, GD0, Delay, SpiE, GpioE> radio::Receive for CC1101<SPI, CS, GD0, Delay>
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = GpioE>,
    GD0: InputPin<Error = GpioE>,
    Delay: DelayMs<u32> + DelayUs<u32>,
    SpiE: Debug,
    GpioE: Debug, 
{
    type Info = PacketInfo ;
    type Error = Error<SpiE, GpioE>;

    fn check_receive(&mut self, _: bool) -> Result<bool, Self::Error> {
        self.gd0.is_low().map_err(Error::Gpio)
    }

    fn get_received(&mut self, buff: &mut [u8]) -> Result<(usize, Self::Info), Self::Error> {
        let len = self.read_register::<Rxbytes>()?.num_rxbytes() ;
        
        if buff.len() < len as usize {
            return Err(Error::InvalidLength) ;
        }

        let mut pkt_len : u8= self.read_register::<TxFifo>()?.into() ;

        let (_, addr_mode) = self.get_pkt()? ;

        if addr_mode != AddrMode::BoardAll {
            return Ok((0, PacketInfo::default()));
        }

        if pkt_len == 0 {
            return Ok((0, PacketInfo::default()));
        }

        pkt_len = pkt_len - 1 ;

        self.read_brust_register::<TxFifo>(&mut buff[..pkt_len as usize])?;
        self.clear_rx_fifo()?;
        Ok((pkt_len as usize, PacketInfo::default()))
    }

    fn start_receive(&mut self) -> Result<(), Self::Error> {
        self.set_state(crate::State::Rx)?;
        Ok(())
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
    pub fn clear_rx_fifo(&mut self) -> Result<(), Error<SpiE, GpioE>> {
        self.set_state(crate::State::IDLE)?;
        self.cmd_strobe(crate::Command::SFRX)?;
        Ok(())
    }
}
