//! # Rainbow Example for the Adafruit KB2040
//!
//! Runs a rainbow-effect colour wheel on the on-board LED.
//!
//! Uses the `ws2812_pio` driver to control the LED, which in turns uses the
//! RP2040's PIO block.

#![no_std]
#![no_main]

#[cfg(feature = "kb2040")]
use adafruit_kb2040 as bsp;
#[cfg(feature = "rp-pico")]
use rp_pico as bsp;

#[cfg(feature = "serial-comms")]
use crate::fmt::Wrapper;
use crate::ws2812::Ws2812;
use bsp::hal::clocks::init_clocks_and_plls;
use bsp::hal::gpio::DynPin;
use bsp::hal::gpio::OutputSlewRate;
use bsp::hal::pio::PIOExt;
use bsp::hal::timer::Timer;
use bsp::hal::usb::UsbBus;
use bsp::hal::watchdog::Watchdog;
use bsp::hal::Clock;
use bsp::hal::Sio;
use bsp::XOSC_CRYSTAL_FREQ;
#[cfg(feature = "serial-comms")]
use core::fmt::Write;
use embedded_hal::prelude::*;
use embedded_time::duration::Extensions;
use keyberon::debounce::Debouncer;
use keyberon::key_code;
use keyberon::layout::Layout;
use keyberon::matrix::PressedKeys;
use panic_halt as _;
use usb_device::class::UsbClass;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceState;
#[cfg(feature = "serial-comms")]
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
#[cfg(feature = "serial-comms")]
use usbd_serial::SerialPort;

#[allow(dead_code)]
static mut USB_BUS: Option<UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

#[cfg(feature = "serial-comms")]
mod fmt;
mod layout;
mod matrix;
mod ws2812;

#[rtic::app(device = bsp::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {

    use super::*;

    const NCOLS: usize = 6;
    const NROWS: usize = 7;
    const SCAN_TIME_US: u32 = 1000;
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

    #[shared]
    struct Shared {
        // #[cfg(feature = "serial-comms")]
        // serial: SerialPort<'static, bsp::hal::usb::UsbBus>,
        usb_dev: usb_device::device::UsbDevice<'static, bsp::hal::usb::UsbBus>,
        usb_class: keyberon::hid::HidClass<
            'static,
            bsp::hal::usb::UsbBus,
            keyberon::keyboard::Keyboard<()>,
        >,
        timer: bsp::hal::timer::Timer,
        alarm: bsp::hal::timer::Alarm0,
        #[lock_free]
        last_report: Option<[u8; 8]>,
        #[lock_free]
        watchdog: bsp::hal::watchdog::Watchdog,
        ws: ws2812::Ws2812,
        #[lock_free]
        matrix: matrix::Matrix<DynPin, DynPin, NCOLS, NROWS>,
        layout: Layout,
        #[lock_free]
        debouncer: Debouncer<PressedKeys<NCOLS, NROWS>>,
        transform: fn(keyberon::layout::Event) -> keyberon::layout::Event,
        is_right: bool,
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

        let sio = Sio::new(c.device.SIO);

        let pins = bsp::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let col1 = pins.d2;
        let col2 = pins.d3;
        let col3 = pins.d4;
        let col4 = pins.d5;
        let col5 = pins.d6;
        let col6 = pins.d7;

        let mut row1 = pins.d8;
        let mut row2 = pins.d9;
        let mut row3 = pins.d10;
        let mut row4 = pins.a0;
        let mut row5 = pins.a1;
        let mut row6 = pins.a2;
        let mut row7 = pins.a3;

        // Set the row slew rates fast so the matrix doesn't need to delay
        // between row checks.
        row1.set_slew_rate(OutputSlewRate::Fast);
        row2.set_slew_rate(OutputSlewRate::Fast);
        row3.set_slew_rate(OutputSlewRate::Fast);
        row4.set_slew_rate(OutputSlewRate::Fast);
        row5.set_slew_rate(OutputSlewRate::Fast);
        row6.set_slew_rate(OutputSlewRate::Fast);
        row7.set_slew_rate(OutputSlewRate::Fast);

        let mut timer = Timer::new(c.device.TIMER, &mut resets);
        let (mut pio, sm0, _, _, _) = c.device.PIO0.split(&mut resets);
        let neopixel = pins.neopixel;
        let ws = Ws2812::new(
            neopixel.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        // let side = pins.rx.into_floating_input();
        cortex_m::asm::delay(1000);

        let is_right = true; //side.is_high().unwrap();

        let transform: fn(keyberon::layout::Event) -> keyberon::layout::Event = if is_right {
            |e| e.transform(|i: u8, j: u8| -> (u8, u8) { (i, 11 - j) })
        } else {
            |e| e
        };

        let matrix: matrix::Matrix<DynPin, DynPin, NCOLS, NROWS> =
            cortex_m::interrupt::free(move |_cs| {
                matrix::Matrix::new(
                    [
                        col1.into_pull_up_input().into(),
                        col2.into_pull_up_input().into(),
                        col3.into_pull_up_input().into(),
                        col4.into_pull_up_input().into(),
                        col5.into_pull_up_input().into(),
                        col6.into_pull_up_input().into(),
                    ],
                    [
                        row1.into_push_pull_output().into(),
                        row2.into_push_pull_output().into(),
                        row3.into_push_pull_output().into(),
                        row4.into_push_pull_output().into(),
                        row5.into_push_pull_output().into(),
                        row6.into_push_pull_output().into(),
                        row7.into_push_pull_output().into(),
                    ],
                )
            })
            .unwrap();

        let layout = Layout::new(layout::LAYERS);
        let debouncer: keyberon::debounce::Debouncer<keyberon::matrix::PressedKeys<NCOLS, NROWS>> =
            Debouncer::new(PressedKeys::default(), PressedKeys::default(), 30);

        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());
        alarm.enable_interrupt(&mut timer);

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

        #[cfg(feature = "serial-comms")]
        let serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        let usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ());
        let usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });
        // let usb_dev = UsbDeviceBuilder::new(
        //     unsafe { USB_BUS.as_ref().unwrap() },
        //     UsbVidPid(0x16c0, 0x27dd),
        // )
        // .manufacturer("Fake company")
        // .product("Serial port")
        // .serial_number("TEST")
        // .device_class(2)
        // .build();

        // Start watchdog and feed it with the lowest priority task at 1000hz
        watchdog.start(10_000.microseconds());

        (
            Shared {
                // #[cfg(feature = "serial-comms")]
                // serial,
                usb_class,
                usb_dev,
                timer,
                alarm,
                watchdog,
                matrix,
                last_report: None,
                ws,
                layout,
                debouncer,
                transform,
                is_right,
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[cfg(feature = "serial-comms")]
    fn write_serial(s: &mut SerialPort<'static, bsp::hal::usb::UsbBus>, buf: &str) {
        let buf = buf.as_bytes();
        let mut write_ptr = &buf[..];
        while !write_ptr.is_empty() {
            match s.write(write_ptr) {
                Ok(len) => write_ptr = &write_ptr[len..],
                // On error, just drop unwritten data.
                // One possible error is Err(WouldBlock), meaning the USB write buffer is full.
                Err(_) => break,
            }
        }
        let _ = s.write("\n".as_bytes());
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [/*serial, */usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        let usb_d = c.shared.usb_dev;
        let usb_c = c.shared.usb_class;
        // #[cfg(feature = "serial-comms")]
        // let serial = c.shared.serial;
        (
            usb_d, usb_c,
            // serial,
        )
            .lock(
                |d,
                 c,
                //  s
                 | {
                    let _ = d.poll(&mut [c]);
                    // #[cfg(feature="serial-comms")]
                    // if d.poll(&mut [s]) {
                    //     let mut buf = [0u8; 64];
                    //     loop {
                    //         if let Err(_) | Ok(0) = s.read(&mut buf) {
                    //             break;
                    //         }
                    //     }
                    // }
                },
            );
    }

    #[task(priority = 2, capacity = 8, shared = [usb_dev, usb_class, layout, /*serial, last_report*/])]
    fn handle_event(mut c: handle_event::Context, event: Option<keyberon::layout::Event>) {
        // let mut serial = c.shared.serial;
        // #[cfg(feature = "serial-comms")]
        // {
        //     serial.lock(|s| {
        //         let mut buf = [0u8; 32];
        //         if let Some(e) = &event {
        //             match e {
        //                 keyberon::layout::Event::Press(x, y) => {
        //                     let _ = write!(Wrapper::new(&mut buf), "Press: {}, {}\n", x, y);
        //                 }
        //                 keyberon::layout::Event::Release(x, y) => {
        //                     let _ = write!(Wrapper::new(&mut buf), "Release: {}, {}\n", x, y);
        //                 }
        //             }
        //         }
        //         let _ = s.write(&buf);
        //     });
        // }
        if let Some(e) = event {
            // TODO: Support Uf2 for the side not performing USB HID
            // The right side only passes None here and buffers the keys
            // for USB to send out when polled by the host
            c.shared
                .layout
                .lock(|l: &mut keyberon::layout::Layout| l.event(e));
            return;
        };

        let report: key_code::KbHidReport = c.shared.layout.lock(|l| {
            l.tick();
            l.keycodes().collect()
        });
        // if Some(report.as_bytes()) != c.shared.last_report.as_ref().map(|b| &b[..]) {
        //     let mut buf = [0u8; 128];
        //     let _ = write!(
        //         Wrapper::new(&mut buf),
        //         "o:{:?}\nn:{:?}\n",
        //         c.shared.last_report,
        //         report
        //     );
        //     serial.lock(|s| {
        //         let _ = s.write(&buf);
        //     });

        //     match c.shared.last_report.as_mut() {
        //         Some(l) => l.clone_from_slice(report.as_bytes()),
        //         None => {
        //             let mut l = [0u8; 8];
        //             l.clone_from_slice(report.as_bytes());
        //             *c.shared.last_report = Some(l);
        //         }
        //     }
        // }
        if !c
            .shared
            .usb_class
            .lock(|k| k.device_mut().set_keyboard_report(report.clone()))
        {
            return;
        }
        if c.shared.usb_dev.lock(|d| d.state()) != UsbDeviceState::Configured {
            return;
        }
        while let Ok(0) = c.shared.usb_class.lock(|k| k.write(report.as_bytes())) {}
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [matrix, debouncer, watchdog, timer, alarm, &transform, &is_right],
    )]
    fn scan_timer_irq(c: scan_timer_irq::Context) {
        let timer = c.shared.timer;
        let alarm = c.shared.alarm;
        (timer, alarm).lock(|t, a| {
            a.clear_interrupt(t);
            let _ = a.schedule(SCAN_TIME_US.microseconds());
        });

        c.shared.watchdog.feed();

        let matrix = c.shared.matrix;
        let keys_pressed = matrix.get().unwrap();
        let deb_events = c
            .shared
            .debouncer
            .events(keys_pressed)
            .map(c.shared.transform);

        if *c.shared.is_right {
            for event in deb_events {
                handle_event::spawn(Some(event)).unwrap();
            }
            handle_event::spawn(None).unwrap();
        } /* else {
              // coordinate and press/release is encoded in a single byte
              // the first 6 bits are the coordinate and therefore cannot go past 63
              // The last bit is to signify if it is the last byte to be sent, but
              // this is not currently used as serial rx is the highest priority
              // end? press=1/release=0 key_number
              //   7         6            543210
              let mut es: [Option<keyberon::layout::Event>; 16] = [None; 16];
              for (i, e) in deb_events.enumerate() {
                  es[i] = Some(e);
              }
              let stop_index = es.iter().position(|&v| v == None).unwrap();
              for i in 0..(stop_index + 1) {
                  let mut byte: u8;
                  if let Some(ev) = es[i] {
                      if ev.coord().1 <= 0b0011_1111 {
                          byte = ev.coord().1;
                      } else {
                          byte = 0b0011_1111;
                      }
                      byte |= (ev.is_press() as u8) << 6;
                      if i == stop_index + 1 {
                          byte |= 0b1000_0000;
                      }
                      // Watchdog will catch any possibility for an infinite loop
                      // while c.shared.uart.lock(|u| u.uartfr.read().txff().bit_is_set()) {}
                      // c.shared
                      //     .uart
                      //     .lock(|u| u.uartdr.write(|w| unsafe { w.data().bits(byte) }));
                  }
              }
          } */
    }
}
