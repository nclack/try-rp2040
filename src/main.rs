#![no_std]
#![no_main]

use core::iter::once;
use embedded_hal::timer::CountDown;
use fugit::ExtU32;
use panic_halt as _;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use waveshare_rp2040_zero::hal::gpio::{FunctionPio0, Pin};
use waveshare_rp2040_zero::{
    entry, hal,
    hal::{clocks::init_clocks_and_plls, pac, pio::PIOExt, watchdog, Clock, Sio, Timer},
    Pins, XOSC_CRYSTAL_FREQ,
};
use waveshare_rp2040_zero::hal::pio::PIOBuilder;
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

    let row0 = pins.gp0.into_mode::<FunctionPio0>();
    let program={
        let mut a = pio::Assembler::<{ pio::RP2040_MAX_PROGRAM_SIZE }>::new();
        let mut loop_label = a.label();
        a.set(pio::SetDestination::PINDIRS,1);
        a.bind(&mut loop_label);
        a.set_with_delay(pio::SetDestination::PINS,1,2);
        a.set_with_delay(pio::SetDestination::PINS,0,1);
        a.jmp(pio::JmpCondition::Always, &mut loop_label);
        a.assemble_program()
    };
    let (mut pio0, sm0, sm1, sm2, sm3) = pac.PIO0.split(&mut pac.RESETS);

    let installed = pio0.install(&program).unwrap();
    let (sm1,rx,tx) = PIOBuilder::from_program(installed)
        .set_pins(row0.id().num,1)
        .clock_divisor_fixed_point(0,0) // (0,0) is as slow as possible
        .build(sm1);
    sm1.start();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    let mut ws = Ws2812::new(
        pins.neopixel.into_mode(),
        &mut pio0,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut n: u8 = 0;
    loop {
        ws.write(brightness(once(wheel(n)), 64)).unwrap();
        n = n.wrapping_add(1);

        delay.start(10.millis());
        let _ = nb::block!(delay.wait());
    }
}

fn wheel(pos: u8) -> RGB8 {
    let mut pos = 255 - pos;
    if pos < 85 {
        (255 - 3 * pos, 0, 3 * pos).into()
    } else if pos < 170 {
        pos -= 85;
        (0, 3 * pos, 255 - 3 * pos).into()
    } else {
        pos -= 170;
        (3 * pos, 255 - 3 * pos, 0).into()
    }
}
