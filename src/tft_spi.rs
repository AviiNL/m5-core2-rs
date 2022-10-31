use esp32_hal::{spi::Spi, pac::SPI3, gpio::*, gpio_types::*, prelude::*, Delay};

pub struct TftSpi {
    spi: Spi<SPI3>,
    dc: Gpio15<Output<PushPull>>,
    cs: Gpio5<Output<PushPull>>,
    addr_row: i32,
    addr_col: i32
}

impl TftSpi {
    pub fn new(spi: Spi<SPI3>, dc: Gpio15<Output<PushPull>>, cs: Gpio5<Output<PushPull>>) -> Self {
        TftSpi { spi, dc, cs, addr_row: 0xFFFF, addr_col: 0xFFFF }
    }

    pub fn init(&mut self, delay: &mut Delay) {
        self.cs.set_high().unwrap();
        self.set_as_data();

        // software reset
        self.write(0x01, &[]);

        delay.delay_ms(150u32); // Wait for reset to complete

        self.write(0xC8, &[0xFF, 0x93, 0x42]); // Enable external commands

        self.write(0xC0, &[0x12, 0x12]); // ILI9341_PWCTR1
        self.write(0xC1, &[0x03]); // ILI9341_PWCTR2

        self.write(0xB0, &[0xE0]); // ??
        self.write(0xF6, &[0x00, 0x01, 0x01]); // ?

        self.write(0x36, &[0x80 | 0x20 | 0x08]); // ILI9341_MADCTL

        self.write(0x3A, &[0x55]); // ILI9341_PIXFMT
        self.write(0xB6, &[0x08, 0x82, 0x27]); // ILI9341_DFUNCTR

        self.write(0xE0, &[
            0x00, 0x0C, 0x11, 0x04,
            0x11, 0x08, 0x37, 0x89,
            0x4C, 0x06, 0x0C, 0x0A,
            0x2E, 0x34, 0x0F,
            ]); // GAMMA 1

        self.write(0xE1, &[
            0x00, 0x0B, 0x11, 0x05,
            0x13, 0x09, 0x33, 0x67,
            0x48, 0x07, 0x0E, 0x0B,
            0x2E, 0x33, 0x0F,
        ]); // GAMMA 2

        self.write(0x11, &[]); // ILI9341_SLPOUT

        self.spi_end();
        delay.delay_ms(120u32); // Wait for display to wake up
        self.spi_begin();

        self.write(0x29, &[]); // ILI9341_DISPON

        self.write(0x21, &[]); // ILI9341_INVON

        self.spi_end();

        self.set_rotation(0, delay);
    }

    pub fn set_rotation(&mut self, mode: u8, delay: &mut Delay) {
        let rotation = mode % 8;

        self.spi_begin();

        match rotation {
            0 => {
                self.write(0x36, &[0x80 | 0x20 | 0x08]);
            },
            1 => {
                self.write(0x36, &[0x08]);
            },
            2 => {
                self.write(0x36, &[0x20 | 0x40 | 0x08]);
            },
            3 => {
                self.write(0x36, &[0x40 | 0x80 | 0x08]);
            },
            4 => {
                self.write(0x36, &[0x40 | 0x80 | 0x20 | 0x08]);
            },
            5 => {
                self.write(0x36, &[0x80 | 0x08]);
            },
            6 => {
                self.write(0x36, &[0x20 | 0x08]);
            },
            7 => {
                self.write(0x36, &[0x40 | 0x08]);
            },
            _ => {
                /* noop */
            },
        };

        delay.delay_us(10u32);

        self.spi_end();
    }

    pub fn draw_pixel(&mut self, x: i32, y: i32, color: u32) {
        
        self.spi_begin();

        // DC_C
        self.set_as_command();

        if x != self.addr_col {
            self.write_8(0x2A);

            let xa = ((x) << 8 | (x) >> 8) | (((x) << 8 | (x) >> 8) << 16);

            self.set_as_data();

            self.write_32(xa as u32);

            self.set_as_command();

            self.addr_col = x;
        }

        if y != self.addr_row {
            self.write_8(0x2B);
            
            let ya = ((y) << 8 | (y) >> 8) | (((y) << 8 | (y) >> 8) << 16);
            
            self.set_as_data();

            self.write_32(ya as u32);

            self.set_as_command();
            
            self.addr_row = y;
        }

        self.write_8(0x2C);

        // DC_D
        self.set_as_data();

        self.write_16(color);

        self.spi_end();

    }

    fn set_as_data(&mut self) {
        self.dc.set_high().unwrap();
    }

    fn set_as_command(&mut self) {
        self.dc.set_low().unwrap();
    }

    fn spi_begin(&mut self) {
        self.cs.set_low().unwrap();
        self.cs.set_low().unwrap();
    }

    fn spi_end(&mut self) {
        self.cs.set_high().unwrap();
    }

    fn write_command(&mut self, cmd: u8) {
        self.spi_begin();

        self.set_as_command();

        self.spi.send(cmd).unwrap();

        self.set_as_data();
        
        self.spi_end();
    }

    fn write_data(&mut self, data: &[u8]) {
        // iterate over data and send each byte
        for byte in data {
            self.spi_begin();

            self.set_as_data(); // Play safe, but should already be in data mode

            self.spi.send(*byte).unwrap();

            // allow more hold time for low VDI rail ?
            self.cs.set_low().unwrap();

            self.spi_end();
        }
    }

    fn write(&mut self, cmd: u8, data: &[u8]) {
        self.write_command(cmd);
        self.write_data(data);
    }

    fn write_8(&mut self, data: u8) {
        let mut buffer = [data];
        self.spi.transfer(&mut buffer).unwrap();
    }

    fn write_16(&mut self, data: u32) {

        let a = (data & 0xF800) >> 8;
        let b = (data & 0x07E0) >> 3;
        let c = (data & 0x001F) << 3;

        self.spi.transfer(&mut [a as u8]).unwrap();
        self.spi.transfer(&mut [b as u8]).unwrap();
        self.spi.transfer(&mut [c as u8]).unwrap();

    }

    fn write_32(&mut self, data: u32) {
        let mut buffer = data.to_be_bytes();
        self.spi.transfer(&mut buffer).unwrap();
    }

}
