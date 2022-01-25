use core::fmt::Debug;
use core::iter::once;
use keyberon::keyboard::Leds;
use smart_leds::{brightness, SmartLedsWrite, RGB8};

pub(crate) struct KbLeds<WS> {
    pub(crate) ws: WS,
    color: RGB8,
}

impl<WS> KbLeds<WS> {
    pub(crate) fn new(ws: WS) -> Self {
        Self {
            ws,
            color: RGB8 { r: 0, g: 0, b: 0 },
        }
    }
}

impl<WS> Leds for KbLeds<WS>
where
    WS: SmartLedsWrite,
    <WS as SmartLedsWrite>::Error: Debug,
    <WS as SmartLedsWrite>::Color: From<RGB8>,
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
