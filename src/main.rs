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

use bsp::hal::clocks::init_clocks_and_plls;
use bsp::hal::gpio::DynPin;
use bsp::hal::timer::Timer;
use bsp::hal::usb::UsbBus;
use bsp::hal::watchdog::Watchdog;
use bsp::hal::Sio;
use bsp::XOSC_CRYSTAL_FREQ;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::prelude::*;
use embedded_time::duration::Extensions;
use keyberon::debounce::Debouncer;
use keyberon::key_code;
use keyberon::layout::Layout;
use keyberon::matrix::{Matrix, PressedKeys};
use panic_halt as _;
use usb_device::class::UsbClass;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::{UsbDeviceBuilder, UsbDeviceState, UsbVidPid};
use usbd_serial::SerialPort;

static mut USB_BUS: Option<UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

mod layout;

#[rtic::app(device = bsp::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {

    use super::*;

    const SCAN_TIME_US: u32 = 1000;
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

    #[shared]
    struct Shared {
        serial: SerialPort<'static, bsp::hal::usb::UsbBus>,
        // serial_usb_dev: usb_device::device::UsbDevice<'static, bsp::hal::usb::UsbBus>,
        usb_dev: usb_device::device::UsbDevice<'static, bsp::hal::usb::UsbBus>,
        usb_class: keyberon::hid::HidClass<
            'static,
            bsp::hal::usb::UsbBus,
            keyberon::keyboard::Keyboard<()>,
        >,
        timer: bsp::hal::timer::Timer,
        alarm: bsp::hal::timer::Alarm0,
        #[lock_free]
        watchdog: bsp::hal::watchdog::Watchdog,
        // #[lock_free]
        // chording: Chording<4>,
        #[lock_free]
        matrix: Matrix<DynPin, DynPin, 6, 7>,
        layout: Layout,
        #[lock_free]
        debouncer: Debouncer<PressedKeys<6, 7>>,
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
        let row1 = pins.d8;
        let row2 = pins.d9;
        let row3 = pins.d10;
        let row4 = pins.mosi;
        let row5 = pins.miso;
        let row6 = pins.sclk;
        let row7 = pins.a0;

        let mut timer = Timer::new(c.device.TIMER, &mut resets);
        let side = pins.rx.into_floating_input();
        cortex_m::asm::delay(1000);

        let is_right = side.is_high().unwrap();

        let transform: fn(keyberon::layout::Event) -> keyberon::layout::Event = if is_right {
            |e| {
                e.transform(|i: u8, j: u8| -> (u8, u8) {
                    // 0 -> 5,  5 -> 15, 10 -> 25
                    let x = ((j / 5) * 10) + (j % 5) + 5;
                    (i, x)
                })
            }
        } else {
            |e| {
                e.transform(|i: u8, j: u8| -> (u8, u8) {
                    let x = ((j / 5) * 10) + 4 - (j % 5);
                    (i, x)
                })
            }
        };

        let matrix: Matrix<DynPin, DynPin, 6, 7> = cortex_m::interrupt::free(move |_cs| {
            Matrix::new(
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
        let debouncer: keyberon::debounce::Debouncer<keyberon::matrix::PressedKeys<6, 7>> =
            Debouncer::new(PressedKeys::default(), PressedKeys::default(), 30);

        // let chording = Chording::new(&layout::CHORDS);
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

        let serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
        // let serial_usb_dev = UsbDeviceBuilder::new(
        //     unsafe { USB_BUS.as_ref().unwrap() },
        //     UsbVidPid(0x16c0, 0x27dd),
        // )
        // .manufacturer("Fake company")
        // .product("Serial port")
        // .serial_number("TEST")
        // .device_class(2)
        // .build();

        let usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ());
        let usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });

        // Start watchdog and feed it with the lowest priority task at 1000hz
        watchdog.start(10_000.microseconds());

        (
            Shared {
                serial,
                // serial_usb_dev,
                usb_class,
                usb_dev,
                timer,
                alarm,
                watchdog,
                // chording,
                matrix,
                layout,
                debouncer,
                transform,
                is_right,
            },
            Local {},
            init::Monotonics(),
        )

        // loop {
        //     delay.start(25.milliseconds());
        //     let _ = nb::block!(delay.wait());
        // }
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [serial, /*serial_usb_dev, */usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        // let mut serial = c.shared.serial;
        // let mut serial_usb_dev = c.shared.serial_usb_dev;
        // serial_usb_dev.lock(|d| {
        //     serial.lock(|s| {
        //         if d.poll(&mut [s]) {
        //             let mut buf = [0u8; 64];
        //             match s.read(&mut buf) {
        //                 Err(_e) => {
        //                     // do nothing... nowhere to log
        //                 }
        //                 Ok(0) => {
        //                     // nothing to read
        //                 }
        //                 Ok(count) => {
        //                     buf.iter_mut().take(count).for_each(|b| {
        //                         b.make_ascii_uppercase();
        //                     });
        //                     let mut write_ptr = &buf[..count];
        //                     while !write_ptr.is_empty() {
        //                         match s.write(write_ptr) {
        //                             Ok(len) => write_ptr = &write_ptr[len..],
        //                             // On error, just drop unwritten data.
        //                             // One possible error is Err(WouldBlock), meaning the USB write buffer is full.
        //                             Err(_) => break,
        //                         }
        //                     }
        //                 }
        //             }
        //         }
        //     })
        // });

        let mut usb_d = c.shared.usb_dev;
        let mut usb_c = c.shared.usb_class;
        usb_d.lock(|d| {
            usb_c.lock(|c| {
                if d.poll(&mut [c]) {
                    c.poll();
                }
            })
        });
    }

    #[task(priority = 2, capacity = 8, shared = [usb_dev, usb_class, layout])]
    fn handle_event(mut c: handle_event::Context, event: Option<keyberon::layout::Event>) {
        if let Some(e) = event {
            // TODO: Support Uf2 for the side not performing USB HID
            // The right side only passes None here and buffers the keys
            // for USB to send out when polled by the host
            c.shared.layout.lock(|l| l.event(e));
            return;
        };

        let report: key_code::KbHidReport = c.shared.layout.lock(|l| l.keycodes().collect());
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
        let keys_pressed = c.shared.matrix.get().unwrap();
        let deb_events = c
            .shared
            .debouncer
            .events(keys_pressed)
            .map(c.shared.transform);
        // TODO: right now chords cannot only be exclusively on one side
        // let events = c.shared.chording.tick(deb_events.collect()).into_iter();

        // TODO: With a TRS cable, we only can have one device support USB
        if *c.shared.is_right {
            for event in deb_events {
                handle_event::spawn(Some(event)).unwrap();
            }
            handle_event::spawn(None).unwrap();
        } /*else {
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
