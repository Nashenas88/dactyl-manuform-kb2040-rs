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

use bsp::hal::clocks::init_clocks_and_plls;
use bsp::hal::timer::Timer;
use bsp::hal::usb::UsbBus;
use bsp::hal::watchdog::Watchdog;
use bsp::hal::{pac, Sio};
use bsp::XOSC_CRYSTAL_FREQ;
use usb_device::class_prelude::UsbBusAllocator;

static mut USB_BUS: Option<UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

#[rtic::app(device = bsp::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use super::*;

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, bsp::hal::usb::UsbBus>,
        usb_class: keyberon::hid::HidClass<
            'static,
            bsp::hal::usb::UsbBus,
            keyberon::keyboard::Keyboard<()>,
        >,
    }

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

        unsafe {
            USB_BUS = Some(usb_bus);
        }

        let usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ());
        let usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });

        (Shared { usb_class, usb_dev }, Local {}, init::Monotonics())

        // loop {
        //     delay.start(25.milliseconds());
        //     let _ = nb::block!(delay.wait());
        // }
    }
}
