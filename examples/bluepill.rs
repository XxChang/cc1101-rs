#![allow(clippy::empty_loop)]
#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_rtt_target as _;

use cortex_m_rt::entry;

use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::{
    gpio::gpioa::PA4,
    gpio::{Output, PushPull},
    pac::{Peripherals, SPI1},
    prelude::*,
    spi::{Pins, Spi, Spi1NoRemap},
    timer::SysDelay,
};

use embedded_hal::spi::{Mode, Phase, Polarity};
use cc1101_rs::CC1101;

pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

// use stm32f1xx_hal as hal;

fn setup() -> (
    Spi<SPI1, Spi1NoRemap, impl Pins<Spi1NoRemap>, u8>,
    PA4<Output<PushPull>>,
    SysDelay,
) {
    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();

    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let cs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    let delay = cp.SYST.delay(&clocks) ;

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        MODE,
        1.MHz(),
        clocks,
    );

    (spi, cs, delay)
}

#[entry]
fn main() -> ! {
    rtt_init_print!() ;

    let (spi, cs, delay) = setup();

    rprintln!("Hello world!") ;

    let mut cc1101 = CC1101::new(spi, cs, delay).unwrap() ;
    let (part, version) = cc1101.info().unwrap() ;

    rprintln!("part number: {}, version: {}", part, version) ;
    
    loop {}
}