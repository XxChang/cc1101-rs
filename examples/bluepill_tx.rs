#![allow(clippy::empty_loop)]
#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_rtt_target as _;

use cortex_m_rt::entry;

use radio::Transmit;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::{
    gpio::gpioa::PA4,
    gpio::gpioa::PA8,
    gpio::gpioc::PC13,
    gpio::{Output, PushPull, Input, PullUp},
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
    PC13<Output<PushPull>>,
    PA8<Input<PullUp>>,
) {
    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();
    let mut gpioc = dp.GPIOC.split() ;

    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh) ;

    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let cs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let gd0 = gpioa.pa8.into_pull_up_input(&mut gpioa.crh);

    let delay = cp.SYST.delay(&clocks) ;

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        MODE,
        1.MHz(),
        clocks,
    );

    (spi, cs, delay, led, gd0)
}

#[entry]
fn main() -> ! {
    rtt_init_print!() ;

    let (spi, cs, delay, mut led, gd0) = setup();

    let cfg = cc1101_rs::Config::default() ;
    let mut cc1101 = CC1101::new(spi, cs, gd0, delay, &cfg).unwrap() ;
    let (part, version) = cc1101.info().unwrap() ;

    rprintln!("part number: {}, version: {}", part, version) ;

    let fifo_thr = cc1101.get_fifo_thr().unwrap() ;
    rprintln!("fifo thr: {:?}", fifo_thr) ;

    let base_freq = cc1101.get_base_frequency().unwrap() ;
    rprintln!("base frequency: {}", base_freq);

    let chanspc = cc1101.get_chanspc().unwrap() as u64 ;
    rprintln!("chanspc: {}", chanspc);

    let chan = cc1101.get_chan().unwrap() ;
    rprintln!("chan: {}", chan);

    let (pkt_cfg0, addr_mode) = cc1101.get_pkt().unwrap() ;
    rprintln!("PKT_CFG0: {:02x}, addr_mode: {:?}", pkt_cfg0, addr_mode) ;

    let (chanbw, drate, mdm_cfg2, mdm_cfg1) = cc1101.get_modem().unwrap() ;
    rprintln!("chanbw: {}, drate: {}, mdm_cfg2: {:02x}, mdm_cfg1: {:02x}", chanbw, drate, mdm_cfg2, mdm_cfg1) ;

    let dev = cc1101.get_deviation().unwrap() ;
    rprintln!("deviation: {}", dev) ;

    let (mcsm0, mcsm1 ) = cc1101.get_state_machine_cfg().unwrap() ;
    rprintln!("mcsm0: {:02x}, mscm1: {:02x}", mcsm0, mcsm1) ;

    let foc_cfg = cc1101.get_freq_offset().unwrap() ;
    rprintln!("foc_cfg: {:02x}", foc_cfg) ;
    
    let wor_cfg = cc1101.get_wake_on_radio().unwrap() ;
    rprintln!("wor_cfg: {:02x}", wor_cfg) ;

    let (fscal0, fscal1, fscal2, fscal3) = cc1101.get_freq_synthe_cal().unwrap() ;
    rprintln!("fscal0: {:02x}, fscal1: {:02x}, fscal2: {:02x}, fscal3: {:02x}", fscal0, fscal1, fscal2, fscal3);
    
    let addr = cc1101.get_address().unwrap() ;
    rprintln!("addr: {}", addr);

    loop {
        led.toggle() ;
        let ashining = b"ashining" ;
        cc1101.start_transmit(ashining).unwrap() ;
        while !cc1101.check_transmit().unwrap() {
            cc1101.delay_us(1000);
        }
        cc1101.delay_us(500000);
    }
}