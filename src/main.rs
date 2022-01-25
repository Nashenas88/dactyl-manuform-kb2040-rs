//! # Rainbow Example for the Adafruit KB2040
//!
//! Runs a rainbow-effect colour wheel on the on-board LED.
//!
//! Uses the `ws2812_pio` driver to control the LED, which in turns uses the
//! RP2040's PIO block.

#![no_std]
#![no_main]

use panic_halt as _;

#[cfg(feature = "kb2040")]
use adafruit_kb2040 as bsp;
#[cfg(feature = "rp-pico")]
use rp_pico as bsp;

#[cfg(feature = "kb2040")]
use bsp::hal::clocks::Clock;
use bsp::hal::clocks::{init_clocks_and_plls, PeripheralClock};
#[cfg(feature = "kb2040")]
use bsp::hal::pio::PIOExt;
use bsp::hal::timer::Timer;
use bsp::hal::usb::UsbBus;
use bsp::hal::watchdog::Watchdog;
use bsp::hal::{pac, Sio};
use bsp::pac::{PIO0, RESETS};
use bsp::XOSC_CRYSTAL_FREQ;
#[cfg(feature = "kb2040")]
use core::fmt::Debug;
#[cfg(feature = "kb2040")]
use smart_leds::SmartLedsWrite;
#[cfg(feature = "kb2040")]
use smart_leds::RGB8;
use usb_device::class_prelude::UsbBusAllocator;
#[cfg(feature = "kb2040")]
use ws2812_pio::Ws2812;

#[cfg(feature = "kb2040")]
mod leds;

static mut USB_BUS: Option<UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

#[rtic::app(device = bsp::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use super::*;

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    /// Entry point to our bare-metal application.
    ///
    /// The `#[entry]` macro ensures the Cortex-M start-up code calls this
    /// function as soon as all global variables are initialised.
    ///
    /// The function configures the RP2040 peripherals, then the LED, then runs
    /// the colour wheel in an infinite loop.
    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Configure the RP2040 peripherals

        let mut pac = pac::Peripherals::take().unwrap();
        let mut watchdog = Watchdog::new(pac.WATCHDOG);

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

        let pins = bsp::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
        let mut _delay = timer.count_down();

        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        ));

        let kb_leds = get_leds(
            pac.PIO0,
            &mut pac.RESETS,
            pins,
            &clocks.peripheral_clock,
            &timer,
        );
        unsafe {
            USB_BUS = Some(usb_bus);
        }

        let _usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, kb_leds);
        let _usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });

        (Shared {}, Local {}, init::Monotonics())

        // loop {
        //     delay.start(25.milliseconds());
        //     let _ = nb::block!(delay.wait());
        // }
    }
}

#[cfg(feature = "kb2040")]
fn get_leds<'a>(
    pio0: PIO0,
    resets: &'a mut RESETS,
    pins: bsp::Pins,
    peripheral_clock: &'a PeripheralClock,
    timer: &'a Timer,
) -> leds::KbLeds<impl SmartLedsWrite<Color = impl From<RGB8>, Error = impl Debug> + 'a> {
    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pio0.split(resets);
    let ws = Ws2812::new(
        pins.neopixel.into_mode(),
        &mut pio,
        sm0,
        peripheral_clock.freq(),
        timer.count_down(),
    );

    leds::KbLeds::new(ws)
}

#[cfg(feature = "pico")]
fn get_leds(
    _pac: PIO0,
    _resets: &mut RESETS,
    _pins: bsp::Pins,
    _peripheral_clock: &PeripheralClock,
    _timer: &Timer,
) {
}
