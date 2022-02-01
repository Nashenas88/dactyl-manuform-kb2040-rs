//! Keyboard firmware for Dactyl-Manuform 6x6 layout on an Adafruit kb2040.

#![no_std]
#![no_main]
// Should just be for the `ws` in Shared, but rtic proc macros don't support attributes =/
#![allow(dead_code)]

#[cfg(feature = "serial-comms")]
use crate::fmt::Wrapper;
use crate::ws2812::Ws2812;
use adafruit_kb2040 as bsp;
use bsp::hal::clocks::init_clocks_and_plls;
use bsp::hal::gpio::DynPin;
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
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceState;
#[cfg(feature = "serial-comms")]
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
#[cfg(feature = "serial-comms")]
use usbd_serial::SerialPort;

#[cfg(feature = "serial-comms")]
mod fmt;
mod layout;
mod matrix;
mod ws2812;

// Rtic entry point. Uses the kb2040's Peripheral Access API (pac), and uses the
// PIO0_IRQ_0 interrupt to dispatch to the handlers.
#[rtic::app(device = bsp::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use super::*;

    /// The number of times a switch needs to be in the same state for the
    /// debouncer to consider it a press.
    const DEBOUNCER_MIN_STATE_COUNT: u16 = 30;

    /// The number of columns on the keyboard.
    const NCOLS: usize = 6;

    /// The number of rows on the keyboard.
    const NROWS: usize = 7;

    /// The amount of time between each scan of the matrix for switch presses.
    /// This is always the time from the *start* of the previous scan.
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

    /// Setup for the keyboard.
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

        let row1 = pins.d8;
        let row2 = pins.d9;
        let row3 = pins.d10;
        let row4 = pins.a0;
        let row5 = pins.a1;
        let row6 = pins.a2;
        let row7 = pins.a3;

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
        // When other side built, this will be needed to ensure both keyboard halves
        // have finished their startup, and the pin signal can be used to determine the
        // keyboard side.
        cortex_m::asm::delay(1000);

        let is_right = true; //side.is_high().unwrap();

        // Map the keys on the right side of the keyboard since the wiring will be reversed across
        // the columns.
        let transform: fn(keyberon::layout::Event) -> keyberon::layout::Event = if is_right {
            |e| e.transform(|i: u8, j: u8| -> (u8, u8) { (i, 11 - j) })
        } else {
            |e| e
        };

        // Build the matrix that will inform which specific combinations of row and col switches
        // are being pressed.
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

        // Load the defined layout which is used later to map the matrix to specific keycodes.
        let layout = Layout::new(layout::LAYERS);
        // The debouncer prevents very tiny bounces from being registered as multiple switch
        // presses.
        let debouncer: keyberon::debounce::Debouncer<keyberon::matrix::PressedKeys<NCOLS, NROWS>> =
            Debouncer::new(
                PressedKeys::default(),
                PressedKeys::default(),
                DEBOUNCER_MIN_STATE_COUNT,
            );

        // The alarm used to trigger the matrix scan.
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());
        alarm.enable_interrupt(&mut timer);

        // The bus that is used to manage the device and class below.
        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));

        // We store the bus in a static to make the borrows satisfy the rtic model, since rtic
        // needs all references to be 'static.
        unsafe {
            USB_BUS = Some(usb_bus);
        }

        #[cfg(feature = "serial-comms")]
        let serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        // The class which specifies this device supports HID Keyboard reports.
        let usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ());
        // The device which represents the device to the system as being a "Keyberon"
        // keyboard.
        let usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });
        // The fake modem device used to generate a serial connection to the host.
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

    /// Helper function to ensure all data is written across the serial interface.
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

    /// Usb interrupt handler. Runs every time the host requests new data.
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
                    // No need to poll the class since that's already being done by the device in
                    // its `poll` method.
                    let _ = d.poll(&mut [c]);
                    // Read the serial data otherwise the serial interface is considered broken
                    // by the host.
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

    /// Process events for the layout then generate and send the Keyboard HID Report.
    #[task(priority = 2, capacity = 8, shared = [usb_dev, usb_class, layout])]
    fn handle_event(mut c: handle_event::Context, event: Option<keyberon::layout::Event>) {
        // If there's an event, process it with the layout and return early. We use `None`
        // to signify that the current scan is done with its events.
        if let Some(e) = event {
            c.shared.layout.lock(|l| l.event(e));
            return;
        };

        // "Tick" the layout so that it gets to a consistent state, then read the keycodes into
        // a Keyboard HID Report.
        let report: key_code::KbHidReport = c.shared.layout.lock(|l| {
            l.tick();
            l.keycodes().collect()
        });

        // Set the keyboard report on the usb class.
        if !c
            .shared
            .usb_class
            .lock(|k| k.device_mut().set_keyboard_report(report.clone()))
        {
            return;
        }

        // If the device is not configured yet, we need to bail out.
        if c.shared.usb_dev.lock(|d| d.state()) != UsbDeviceState::Configured {
            return;
        }

        // Watchdog will prevent the keyboard from getting stuck in this loop.
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

        // Immediately clear the interrupt and schedule the next scan alarm.
        (timer, alarm).lock(|t, a| {
            a.clear_interrupt(t);
            let _ = a.schedule(SCAN_TIME_US.microseconds());
        });

        // Feed the watchdog so it knows we haven't frozen/crashed.
        c.shared.watchdog.feed();

        // Get debounced, pressed keys.
        let matrix = c.shared.matrix;
        let keys_pressed = matrix.get().unwrap();
        let deb_events = c
            .shared
            .debouncer
            .events(keys_pressed)
            .map(c.shared.transform);

        // If we're on the right side of the keyboard, just send the events to the event handler
        // so they can be sent over USB.
        if *c.shared.is_right {
            for event in deb_events {
                handle_event::spawn(Some(event)).unwrap();
            }
            handle_event::spawn(None).unwrap();
        } /* else {
              // TODO:Implement handling for the left half of the keyboard once it's physically built
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
