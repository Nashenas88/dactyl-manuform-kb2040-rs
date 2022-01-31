use adafruit_kb2040 as bsp;
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

        for (ri, row) in (&mut self.rows).iter_mut().enumerate() {
            row.set_low()?;
            // Delay to let signal propagate.
            cortex_m::asm::delay(100);
            // let pins = unsafe { &(*bsp::pac::SIO::ptr()).gpio_in.read().bits() };
            for (ci, col) in (&self.cols).iter().enumerate() {
                if col.is_low()? {
                    keys.0[ri][ci] = true;
                }
            }
            row.set_high()?;
        }
        Ok(keys)
    }

    pub fn set_row_low<E>(&mut self, y: usize) -> Result<(), E>
    where
        R: OutputPin<Error = E>,
    {
        self.rows[y].set_low()
    }

    pub fn is_col_low<E>(&mut self, x: usize) -> Result<bool, E>
    where
        C: InputPin<Error = E>,
        R: OutputPin<Error = E>,
    {
        self.cols[x].is_low()
    }
}
