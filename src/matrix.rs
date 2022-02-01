//! A modified version of Keyberon's matrix mod that includes delays between
//! the row check to fix a timing issue on the kb2040.

use embedded_hal::digital::v2::{InputPin, OutputPin};
use keyberon::matrix::PressedKeys;

pub struct Matrix<C, R, const CS: usize, const RS: usize>
where
    C: InputPin,
    R: OutputPin,
{
    cols: [C; CS],
    rows: [R; RS],
}

impl<C, R, const CS: usize, const RS: usize> Matrix<C, R, CS, RS>
where
    C: InputPin,
    R: OutputPin,
{
    pub fn new<E>(cols: [C; CS], rows: [R; RS]) -> Result<Self, E>
    where
        C: InputPin<Error = E>,
        R: OutputPin<Error = E>,
    {
        let mut res = Self { cols, rows };
        res.clear()?;
        Ok(res)
    }
    pub fn clear<E>(&mut self) -> Result<(), E>
    where
        C: InputPin<Error = E>,
        R: OutputPin<Error = E>,
    {
        for r in self.rows.iter_mut() {
            r.set_high()?;
        }
        Ok(())
    }
    pub fn get<E>(&mut self) -> Result<PressedKeys<CS, RS>, E>
    where
        C: InputPin<Error = E>,
        R: OutputPin<Error = E>,
    {
        let mut keys = PressedKeys::default();

        for (ri, row) in self.rows.iter_mut().enumerate() {
            row.set_low()?;
            // Delay to let signal propagate. Without this, the previous row
            // signal might still be low while the current row is low, and two
            // presses on the same column get registered for a single key press.
            cortex_m::asm::delay(100);

            // TODO: Read all pins at once to speed this section up.
            // let pins = unsafe { &(*bsp::pac::SIO::ptr()).gpio_in.read().bits() };
            for (ci, col) in self.cols.iter().enumerate() {
                if col.is_low()? {
                    keys.0[ri][ci] = true;
                }
            }
            row.set_high()?;
        }
        Ok(keys)
    }
}
