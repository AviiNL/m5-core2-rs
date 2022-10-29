use display_interface::DisplayError;
use esp32_hal::i2c::Error as I2CError;

#[derive(Debug)]
pub enum Error {
    I2C(I2CError),
    DisplayError(DisplayError),
    Voltage,
}

impl From<I2CError> for Error {
    fn from(err: I2CError) -> Self {
        Error::I2C(err)
    }
}

impl From<DisplayError> for Error {
    fn from(err: DisplayError) -> Self {
        Error::DisplayError(err)
    }
}