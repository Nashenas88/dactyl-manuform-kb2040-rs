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
use usb_device::class::UsbClass;
use usb_device::class_prelude::UsbBusAllocator;

static mut USB_BUS: Option<UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

#[rtic::app(device = bsp::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
    use usbd_serial::SerialPort;

    use super::*;

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

    #[shared]
    struct Shared {
        serial: SerialPort<'static, bsp::hal::usb::UsbBus>,
        serial_usb_dev: usb_device::device::UsbDevice<'static, bsp::hal::usb::UsbBus>,
        // usb_dev: usb_device::device::UsbDevice<'static, bsp::hal::usb::UsbBus>,
        // usb_class: keyberon::hid::HidClass<
        //     'static,
        //     bsp::hal::usb::UsbBus,
        //     keyberon::keyboard::Keyboard<()>,
        // >,
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

        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let mut resets = c.device.RESETS;

        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // let sio = Sio::new(c.device.SIO);

        // let pins = bsp::Pins::new(
        //     c.device.IO_BANK0,
        //     c.device.PADS_BANK0,
        //     sio.gpio_bank0,
        //     &mut resets,
        // );

        // let timer = Timer::new(c.device.TIMER, &mut resets);
        // let mut _delay = timer.count_down();

        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));

        unsafe {
            USB_BUS = Some(usb_bus);
        }

        let serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
        let serial_usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2)
        .build();

        // let usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ());
        // let usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });

        (
            Shared {
                serial,
                serial_usb_dev,
                // usb_class,
                // usb_dev,
            },
            Local {},
            init::Monotonics(),
        )

        // loop {
        //     delay.start(25.milliseconds());
        //     let _ = nb::block!(delay.wait());
        // }
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [serial, serial_usb_dev])] //, usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        let mut serial = c.shared.serial;
        let mut serial_usb_dev = c.shared.serial_usb_dev;
        serial_usb_dev.lock(|d| {
            serial.lock(|s| {
                if d.poll(&mut [s]) {
                    let mut buf = [0u8; 64];
                    match s.read(&mut buf) {
                        Err(_e) => {
                            // do nothing... nowhere to log
                        }
                        Ok(0) => {
                            // nothing to read
                        }
                        Ok(count) => {
                            buf.iter_mut().take(count).for_each(|b| {
                                b.make_ascii_uppercase();
                            });
                            let mut write_ptr = &buf[..count];
                            while !write_ptr.is_empty() {
                                match s.write(write_ptr) {
                                    Ok(len) => write_ptr = &write_ptr[len..],
                                    // On error, just drop unwritten data.
                                    // One possible error is Err(WouldBlock), meaning the USB write buffer is full.
                                    Err(_) => break,
                                }
                            }
                        }
                    }
                }
            })
        });

        // let mut _usb_d = c.shared.usb_dev;
        // let mut _usb_c = c.shared.usb_class;
        // usb_d.lock(|d| {
        //     usb_c.lock(|c| {
        //         if d.poll(&mut [c]) {
        //             c.poll();
        //         }
        //     })
        // });
    }
}
