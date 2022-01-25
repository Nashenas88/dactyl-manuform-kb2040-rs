use adafruit_kb2040::hal::gpio::{Function, FunctionConfig, PinId, ValidPinMode};
use adafruit_kb2040::hal::pio::{PIOExt, StateMachineIndex};
use core::iter::once;
use embedded_hal::timer::CountDown;
use embedded_time::duration::Microseconds;
use keyberon::keyboard::Leds;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

pub(crate) struct KbLeds<P, SM, C, I>
where
    I: PinId,
    C: CountDown,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    pub(crate) ws: Ws2812<P, SM, C, I>,
    color: RGB8,
}

impl<P, SM, C, I> KbLeds<P, SM, C, I>
where
    I: PinId,
    C: CountDown,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    pub(crate) fn new(ws: Ws2812<P, SM, C, I>) -> Self {
        Self {
            ws,
            color: RGB8 { r: 0, g: 0, b: 0 },
        }
    }
}

impl<P, SM, C, I> Leds for KbLeds<P, SM, C, I>
where
    I: PinId,
    C: CountDown,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
    <C as CountDown>::Time: From<Microseconds>,
{
    fn num_lock(&mut self, status: bool) {
        self.color.r = if status { 255 } else { 0 };
        self.ws.write(brightness(once(self.color), 32)).unwrap();
    }

    fn caps_lock(&mut self, status: bool) {
        self.color.g = if status { 255 } else { 0 };
        self.ws.write(brightness(once(self.color), 32)).unwrap();
    }

    fn scroll_lock(&mut self, status: bool) {
        self.color.b = if status { 255 } else { 0 };
        self.ws.write(brightness(once(self.color), 32)).unwrap();
    }

    fn compose(&mut self, _status: bool) {}

    fn kana(&mut self, _status: bool) {}
}
