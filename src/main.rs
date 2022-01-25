//! # Rainbow Example for the Adafruit KB2040
//!
//! Runs a rainbow-effect colour wheel on the on-board LED.
//!
//! Uses the `ws2812_pio` driver to control the LED, which in turns uses the
//! RP2040's PIO block.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::timer::CountDown;
use embedded_time::duration::Extensions;
use panic_halt as _;

use adafruit_kb2040::hal::clocks::{init_clocks_and_plls, Clock};
use adafruit_kb2040::hal::pio::PIOExt;
use adafruit_kb2040::hal::timer::Timer;
use adafruit_kb2040::hal::usb::UsbBus;
use adafruit_kb2040::hal::watchdog::Watchdog;
use adafruit_kb2040::hal::{pac, Sio};
use adafruit_kb2040::XOSC_CRYSTAL_FREQ;
use usb_device::class_prelude::UsbBusAllocator;
use ws2812_pio::Ws2812;

mod leds;

static mut USB_BUS: Option<UsbBusAllocator<adafruit_kb2040::hal::usb::UsbBus>> = None;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this
/// function as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then the LED, then runs
/// the colour wheel in an infinite loop.
#[entry]
fn main() -> ! {
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

    let pins = adafruit_kb2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let ws = Ws2812::new(
        pins.neopixel.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let kb_leds = leds::KbLeds::new(ws);
    unsafe {
        USB_BUS = Some(usb_bus);
    }

    let _usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, kb_leds);
    let _usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });

    loop {
        delay.start(25.milliseconds());
        let _ = nb::block!(delay.wait());
    }
}
