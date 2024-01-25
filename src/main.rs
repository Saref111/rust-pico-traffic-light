#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::Interrupt as InterruptEnum,
    pac,
    pac::interrupt,
    sio::Sio,
    watchdog::Watchdog,
    Timer,
};
use core::cell::Cell;
use critical_section::Mutex;
use debounced_pin::prelude::*;
use debounced_pin::ActiveHigh;

type RedLedPin = bsp::hal::gpio::Pin<
    bsp::hal::gpio::bank0::Gpio13,
    bsp::hal::gpio::FunctionSioOutput,
    bsp::hal::gpio::PullNone,
>;

type YellowLedPin = bsp::hal::gpio::Pin<
    bsp::hal::gpio::bank0::Gpio15,
    bsp::hal::gpio::FunctionSioOutput,
    bsp::hal::gpio::PullNone,
>;
type GreenLedPin = bsp::hal::gpio::Pin<
    bsp::hal::gpio::bank0::Gpio16,
    bsp::hal::gpio::FunctionSioOutput,
    bsp::hal::gpio::PullNone,
>;

type ButtonPin = DebouncedInputPin<
    bsp::hal::gpio::Pin<
        bsp::hal::gpio::bank0::Gpio14,
        bsp::hal::gpio::FunctionSioInput,
        bsp::hal::gpio::PullUp,
    >,
    ActiveHigh,
>;

type ButtonAndState =
    (ButtonPin, RedLedPin, YellowLedPin, GreenLedPin, i32);

static GLOBAL_STATE: Mutex<Cell<Option<ButtonAndState>>> =
    Mutex::new(Cell::new(None));

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let _timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let _delay = cortex_m::delay::Delay::new(
        core.SYST,
        clocks.system_clock.freq().to_Hz(),
    );

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let button = pins.gpio14.reconfigure();

    let red_led = pins.gpio13.reconfigure();
    let yellow_led = pins.gpio15.reconfigure();
    let green_led = pins.gpio16.reconfigure();
    let state = 0;

    button.set_interrupt_enabled(InterruptEnum::EdgeLow, true);
    let button = DebouncedInputPin::new(button, ActiveHigh);

    critical_section::with(|cs| {
        GLOBAL_STATE.borrow(cs).replace(Some((
            button, red_led, yellow_led, green_led, state,
        )));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }
    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut BUTTON_AND_STATE: Option<ButtonAndState> = None;

    if BUTTON_AND_STATE.is_none() {
        critical_section::with(|cs| {
            *BUTTON_AND_STATE = GLOBAL_STATE.borrow(cs).take();
        });
    }

    if let Some(bs) = BUTTON_AND_STATE {
        let (button, red_led, yellow_led, green_led, state) = bs;

        match button.update().unwrap() {
            DebounceState::Debouncing => return,
            DebounceState::Reset => return,
            DebounceState::NotActive => return,
            DebounceState::Active => {
                match state {
                    0 => {
                        red_led.set_high().unwrap();
                        yellow_led.set_low().unwrap();
                        green_led.set_low().unwrap();
                    }
                    1 => {
                        red_led.set_low().unwrap();
                        yellow_led.set_high().unwrap();
                        green_led.set_low().unwrap();
                    }
                    2 => {
                        red_led.set_low().unwrap();
                        yellow_led.set_low().unwrap();
                        green_led.set_high().unwrap();
                    }
                    _ => {}
                }
                *state += 1;

                if *state > 2 {
                    *state = 0;
                }

                button.pin.clear_interrupt(InterruptEnum::EdgeLow);
            }
        }
    }
}
