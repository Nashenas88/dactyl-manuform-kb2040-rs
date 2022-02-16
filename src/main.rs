//! Keyboard firmware for Dactyl-Manuform 6x6 layout on an Adafruit kb2040.

#![no_std]
#![no_main]
// Should just be for the `ws` in Shared, but rtic proc macros don't support attributes =/
#![allow(dead_code)]

use crate::fmt::Wrapper;
use crate::layout::CustomAction;
use crate::ws2812::Ws2812;
use adafruit_kb2040 as bsp;
use bsp::hal::clocks::init_clocks_and_plls;
use bsp::hal::gpio::bank0::{Gpio0, Gpio1};
use bsp::hal::gpio::DynPin;
use bsp::hal::gpio::{FunctionUart, Pin};
use bsp::hal::pio::PIOExt;
use bsp::hal::timer::Timer;
use bsp::hal::uart::{self, Enabled, UartPeripheral};
use bsp::hal::usb::UsbBus;
use bsp::hal::watchdog::Watchdog;
use bsp::hal::Clock;
use bsp::hal::Sio;
use bsp::pac::UART0;
use bsp::XOSC_CRYSTAL_FREQ;
use core::fmt::Write;
use embedded_hal::prelude::*;
use embedded_time::duration::Extensions;
use keyberon::debounce::Debouncer;
use keyberon::key_code;
use keyberon::layout::{Event, Layout};
use keyberon::matrix::PressedKeys;
use panic_halt as _;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceState;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::SerialPort;

mod fmt;
mod layout;
mod matrix;
mod ws2812;

#[cfg(feature = "left-kb")]
const IS_RIGHT: bool = false;
#[cfg(not(feature = "left-kb"))]
const IS_RIGHT: bool = true;

type UartTx = Pin<Gpio0, FunctionUart>;
type UartRx = Pin<Gpio1, FunctionUart>;

// Rtic entry point. Uses the kb2040's Peripheral Access API (pac), and uses the
// PIO0_IRQ_0 interrupt to dispatch to the handlers.
#[rtic::app(device = bsp::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {

    use bsp::hal::uart::ReadErrorType;

    use super::*;

    /// The number of times a switch needs to be in the same state for the
    /// debouncer to consider it a press.
    const DEBOUNCER_MIN_STATE_COUNT: u16 = 30;

    const STOP_BIT_MASK: u8 = 0b1000_0000;
    const PRESSED_BIT_MASK: u8 = 0b0100_0000;
    const PRESSED_SHIFT: u8 = 6;
    const ROW_BIT_MASK: u8 = 0b0011_1000;
    const ROW_SHIFT: u8 = 3;
    const COL_BIT_MASK: u8 = 0b0000_0111;
    const COL_SHIFT: u8 = 0;

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
        serial: SerialPort<'static, bsp::hal::usb::UsbBus>,
        usb_dev: usb_device::device::UsbDevice<'static, bsp::hal::usb::UsbBus>,
        usb_class: Option<
            keyberon::hid::HidClass<
                'static,
                bsp::hal::usb::UsbBus,
                keyberon::keyboard::Keyboard<()>,
            >,
        >,
        uart: UartPeripheral<Enabled, UART0, (UartTx, UartRx)>,
        timer: bsp::hal::timer::Timer,
        alarm: bsp::hal::timer::Alarm0,
        #[lock_free]
        watchdog: bsp::hal::watchdog::Watchdog,
        ws: ws2812::Ws2812,
        #[lock_free]
        matrix: matrix::Matrix<DynPin, DynPin, NCOLS, NROWS>,
        layout: Layout<CustomAction>,
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

        let tx = pins.tx.into_mode::<FunctionUart>();
        let rx = pins.rx.into_mode::<FunctionUart>();

        let mut timer = Timer::new(c.device.TIMER, &mut resets);
        let (mut pio, sm0, _, _, _) = c.device.PIO0.split(&mut resets);
        let neopixel = pins.neopixel;
        let ws = Ws2812::new(
            neopixel.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        cortex_m::asm::delay(1000);

        let is_right = IS_RIGHT; //side.is_high().unwrap();

        // Map the keys on the right side of the keyboard since the wiring will be reversed across
        // the columns.
        let transform: fn(keyberon::layout::Event) -> keyberon::layout::Event = if is_right {
            |e| e.transform(|i: u8, j: u8| -> (u8, u8) { (i, 11 - j) })
        } else {
            |e| e
        };

        let mut uart = UartPeripheral::<_, _, _>::new(c.device.UART0, (tx, rx), &mut resets)
            .enable(
                uart::common_configs::_9600_8_N_1,
                clocks.peripheral_clock.into(),
            )
            .unwrap();

        if is_right {
            uart.enable_rx_interrupt();
        }

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
        let layout = Layout::<CustomAction>::new(layout::LAYERS);
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

        let serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        // The class which specifies this device supports HID Keyboard reports.
        let usb_class =
            IS_RIGHT.then(|| keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ()));
        // The device which represents the device to the system as being a "Keyberon"
        // keyboard.
        let usb_dev = if IS_RIGHT {
            keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() })
        } else {
            // The fake modem device used to generate a serial connection to the host.
            UsbDeviceBuilder::new(
                unsafe { USB_BUS.as_ref().unwrap() },
                UsbVidPid(0x16c0, 0x27dd),
            )
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(2)
            .build()
        };

        // Start watchdog and feed it with the lowest priority task at 1000hz
        watchdog.start(10_000.microseconds());

        (
            Shared {
                serial,
                usb_class,
                usb_dev,
                uart,
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
    fn write_serial(s: &mut SerialPort<'static, bsp::hal::usb::UsbBus>, buf: &str) {
        let mut write_ptr = buf.as_bytes();
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
    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [serial, usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        let usb_d = c.shared.usb_dev;
        let usb_c = c.shared.usb_class;
        let serial = c.shared.serial;
        (usb_d, usb_c, serial).lock(|d, c, s| {
            // !!!!!!!!!!!!!!!!!!!!! VERY IMPORTANT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            // Order of classes list here MUST match the order they were constructed in!
            // If this does not happen the USB device hangs and the host will report
            // weird, hard to debug errors!
            // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            let polled = if IS_RIGHT {
                d.poll(&mut [s, c.as_mut().unwrap()])
            } else {
                d.poll(&mut [s])
            };

            if polled {
                // No need to poll the class since that's already being done by usb_d in its `poll`
                // method.

                // Read the serial data otherwise the serial interface is considered broken
                // by the host.
                let mut buf = [0u8; 64];
                loop {
                    if let Err(_) | Ok(0) = s.read(&mut buf) {
                        break;
                    }
                }
            }
        });
    }

    /// Process events for the layout then generate and send the Keyboard HID Report.
    #[task(priority = 2, capacity = 8, shared = [serial, usb_dev, usb_class, layout])]
    fn handle_event(mut c: handle_event::Context, event: Option<keyberon::layout::Event>) {
        // If there's an event, process it with the layout and return early. We use `None`
        // to signify that the current scan is done with its events.
        if let Some(e) = event {
            c.shared.serial.lock(|s| {
                let mut buf = [0u8; 64];
                let _ = writeln!(Wrapper::new(&mut buf), "{:?}", e);
                write_serial(s, unsafe { core::str::from_utf8_unchecked(&buf) });
            });
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
        let was_report_modified = c.shared.usb_class.lock(|k| {
            k.as_mut()
                // This function is never called on the left half. usb_class is always populated on
                // the right half.
                .unwrap()
                .device_mut()
                .set_keyboard_report(report.clone())
        });
        if !was_report_modified {
            return;
        }

        // If the device is not configured yet, we need to bail out.
        if c.shared.usb_dev.lock(|d| d.state()) != UsbDeviceState::Configured {
            return;
        }

        // Watchdog will prevent the keyboard from getting stuck in this loop.
        while let Ok(0) = c
            .shared
            .usb_class
            .lock(|k| k.as_mut().unwrap().write(report.as_bytes()))
        {}
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [matrix, debouncer, watchdog, timer, alarm, uart, serial, &transform, &is_right],
    )]
    fn scan_timer_irq(mut c: scan_timer_irq::Context) {
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
        } else {
            // coordinate and press/release is encoded in a single byte
            // the first 6 bits are the coordinate and therefore cannot go past 63
            // The last bit is to signify if it is the last byte to be sent, but
            // this is not currently used as serial rx is the highest priority
            // end? press=1/release=0 key_number
            //   7         6            543210
            let mut es: [Option<keyberon::layout::Event>; 16] = [None; 16];
            let mut last = 0;
            for (i, e) in deb_events.enumerate() {
                es[i] = Some(e);
                last = i;
            }
            let stop_index = last + 1;
            let mut byte: u8;
            for (idx, e) in es.iter().enumerate().take(stop_index) {
                if let Some(ev) = e {
                    let (i, j) = ev.coord();
                    // 7 is the max value we can encode, and the highest row/col count on this board.
                    if i > 7 || j > 7 {
                        continue;
                    }

                    byte = (j << COL_SHIFT) & COL_BIT_MASK;
                    byte |= (i << ROW_SHIFT) & ROW_BIT_MASK;
                    byte |= (ev.is_press() as u8) << PRESSED_SHIFT;
                    if idx + 1 == stop_index {
                        byte |= 0b1000_0000;
                    }

                    let mut buf = [0u8; 128];
                    let _ = writeln!(
                        Wrapper::new(&mut buf),
                        "Sending byte {:08b} for event {:?}",
                        byte,
                        ev
                    );
                    c.shared
                        .serial
                        .lock(|s| write_serial(s, unsafe { core::str::from_utf8_unchecked(&buf) }));
                    c.shared.uart.lock(|u| u.write_full_blocking(&[byte]));
                }
            }
        }
    }

    #[task(binds = UART0_IRQ, priority = 4, shared = [uart, serial])]
    fn rx(mut c: rx::Context) {
        let mut data = [0u8; 1];
        if let Err(e) = c.shared.uart.lock(|u| u.read_full_blocking(&mut data)) {
            let message: &'static str = match e {
                ReadErrorType::Overrun => "Read error: overrun",
                ReadErrorType::Break => "Read error: break",
                ReadErrorType::Parity => "Read error: parity",
                ReadErrorType::Framing => "Read error: framing",
            };
            c.shared.serial.lock(|s| write_serial(s, message));
            return;
        }
        let [d] = data;
        let i = (d & ROW_BIT_MASK) >> ROW_SHIFT;
        let j = (d & COL_BIT_MASK) >> COL_SHIFT;
        let is_pressed = (d & PRESSED_BIT_MASK) > 0;

        handle_event::spawn(Some(if is_pressed {
            Event::Press(i, j)
        } else {
            Event::Release(i, j)
        }))
        .unwrap();
    }
}
