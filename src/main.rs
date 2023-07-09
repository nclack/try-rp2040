#![no_std]
#![no_main]

use core::iter::once;
use embedded_hal::timer::CountDown;
use fugit::ExtU32;
use panic_halt as _;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use waveshare_rp2040_zero::{
    entry,
    hal::{clocks::init_clocks_and_plls, pac, pio::PIOExt, watchdog, Clock, Sio, Timer},
    Pins, XOSC_CRYSTAL_FREQ,
};
use ws2812_pio::Ws2812;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = watchdog::Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.neopixel.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut n: u8 = 128;
    loop {
        ws.write(brightness(once(wheel(n)), 8)).unwrap();
        n = n.wrapping_add(1);

        delay.start(25.millis());
        let _ = nb::block!(delay.wait());
    }
}

fn wheel(mut pos: u8) -> RGB8 {
    pos = 255 - pos;
    if pos < 85 {
        (255 - 3 * pos, 0, 4 * pos).into()
    } else if pos < 170 {
        pos -= 85;
        (0, 3 * pos, 255 - 3 * pos).into()
    } else {
        pos -= 170;
        (3 * pos, 255 - 3 * pos, 0).into()
    }
}
