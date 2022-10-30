
use embedded_hal::{blocking::i2c::{Write as _, WriteRead}, prelude::_embedded_hal_blocking_delay_DelayMs};
use esp32_hal::{i2c::I2C, pac::I2C0, Delay};
use esp_println::println;
use crate::error::Error;

type I2cController = I2C<I2C0>;

#[allow(dead_code)]
#[derive(Debug, PartialEq)]
enum DCDC {
    DCDC0 = 0,
    DCDC1,
    DCDC2,
}

#[allow(dead_code)]
#[derive(Debug, PartialEq)]
enum LDO {
    LDO2 = 2,
    LDO3 = 3,
}

#[allow(dead_code, non_camel_case_types)]
enum CHGCurrent {
    kCHG_100mA = 0,
    kCHG_190mA,
    kCHG_280mA,
    kCHG_360mA,
    kCHG_450mA,
    kCHG_550mA,
    kCHG_630mA,
    kCHG_700mA,
    kCHG_780mA,
    kCHG_880mA,
    kCHG_960mA,
    kCHG_1000mA,
    kCHG_1080mA,
    kCHG_1160mA,
    kCHG_1240mA,
    kCHG_1320mA,
}

#[allow(dead_code, non_camel_case_types)]
#[derive(PartialEq)]
enum BusPowerMode {
    kMBusModeOutput,
    kMBusModeInput,
}

pub struct Axp192 {
    i2c: I2cController,
    address: u8,
}

impl Axp192 {
    pub fn new(i2c: I2cController) -> Self {
        Self { i2c, address: 0x34 }
    }

    pub fn init(&mut self, delay: &mut Delay) -> Result<(), Error> {

        let data = self.read(0x30)?;
        self.write(0x30, data & 0x04 | 0x02)?;
        println!("axp: vbus limit off");

        let data = self.read(0x92)?;
        self.write(0x92, data & 0xf8)?;
        println!("axp: gpio1 init");
        
        let data = self.read(0x93)?;
        self.write(0x93, data & 0xf8)?;
        println!("axp: gpio2 init");

        let data = self.read(0x35)?;
        self.write(0x35, (data & 0x1c) | 0xa2)?;
        println!("axp: rtc battery charging enabled");

        self.set_esp_voltage(3350)?;
        println!("axp: esp32 power voltage was set to 3.35v");
        
        self.set_lcd_voltage(2800)?;
        println!("axp: lcd backlight voltage was set to 2.80v");

        self.set_ldo_voltage(LDO::LDO2, 3300)?;
        println!("axp: lcd logic and sdcard voltage preset to 3.3v");

        self.set_ldo_voltage(LDO::LDO3, 2000)?;
        println!("axp: vibrator voltage preset to 2v");

        self.set_ldo_enable(LDO::LDO2, true)?;

        self.set_lcd(true)?;
        println!("axp: lcd backlight enabled");
        self.set_led(true)?;
        println!("axp: led enabled");

        self.set_chg_current(CHGCurrent::kCHG_100mA)?;

        let data = self.read(0x95)?;
        self.write(0x95, (data & 0x72) | 0x84)?;

        self.write(0x36, 0x4c)?;

        self.write(0x82, 0xff)?;

        self.set_lcd_reset(false)?;
        delay.delay_ms(120u32);
        
        self.set_lcd_reset(true)?;
        delay.delay_ms(120u32);
        println!("axp: lcd reset set to high");

        self.set_peripherals_power(true)?;

        // axp: check v-bus status
        if self.read(0x00)? & 0x08 > 0x00 {
            println!("axp: v-bus is present");
            
            let data = self.read(0x30)?;
            self.write(0x30, data | 0x80)?;

            self.set_bus_power_mode(BusPowerMode::kMBusModeInput)?;
        } else {
            println!("axp: v-bus is not present, using m-bus");
            self.set_bus_power_mode(BusPowerMode::kMBusModeOutput)?;
        }

        Ok(())
    }

    pub fn set_led(&mut self, enable: bool) -> Result<(), Error> {
        let mut data = self.read(0x94)?;

        if enable {
            data &= 0xfd;
        } else {
            data |= 0x02;
        }

        self.write(0x94, data)?;

        Ok(())
    }

    pub fn set_lcd(&mut self, enable: bool) -> Result<(), Error> {
        let mut data = self.read(0x12)?;

        if enable {
            data = (1 << 1) | data;
        } else {
            data = !(1 << 1) & data;
        }

        self.write(0x12, data)?;

        Ok(())
    }

    fn set_ldo_enable(&mut self, ldo: LDO, state: bool) -> Result<(), Error> {
        let mut mark = 0x01;

        mark <<= ldo as u8;

        let data = self.read(0x12)?;

        if state {
            self.write(0x12, data | mark)?;
        } else {
            self.write(0x12, data & !mark)?;
        }

        Ok(())
    }

    fn set_ldo_voltage(&mut self, ldo: LDO, voltage: u16) -> Result<(), Error> {
        let vdata = self.calc_voltage_data(voltage, 3300, 1800, 100) & 0x0f;

        match ldo {
            LDO::LDO2 => {
                let data = self.read(0x28)?;
                self.write(0x28, data & 0xF0 | (vdata << 4))?;
            },
            LDO::LDO3 => {
                let data = self.read(0x28)?;
                self.write(0x28, data & 0x0F | vdata)?;
            },
        }

        Ok(())
    }

    fn set_lcd_voltage(&mut self, voltage: u16) -> Result<(), Error> {
        if voltage < 2500 || voltage > 3300 {
            return Err(Error::Voltage);
        }

        self.set_dc_voltage(DCDC::DCDC2, voltage)?;
        Ok(())
    }

    fn set_esp_voltage(&mut self, voltage: u16) -> Result<(), Error> {
        if voltage < 3000 || voltage > 3400 {
            return Err(Error::Voltage)
        }
        
        self.set_dc_voltage(DCDC::DCDC0, voltage)?;
        Ok(())
    }

    fn set_dc_voltage(&mut self, dcdc: DCDC, voltage: u16) -> Result<(), Error> {
        match dcdc {
            DCDC::DCDC0 => {
                let addr = 0x26;
                let vdata = self.calc_voltage_data(voltage, 3500, 700, 25) & 0x7f;

                let mut data = self.read(addr)?;
                data &= 0x80;
                data |= vdata;

                self.write(addr, data)?;
            }
            DCDC::DCDC1 => {
                let addr = 0x25;
                let vdata = self.calc_voltage_data(voltage, 2275, 700, 25) & 0x3f;

                let mut data = self.read(addr)?;
                data &= 0x80;
                data |= vdata;

                self.write(addr, data)?;
            }
            DCDC::DCDC2 => {
                let addr = 0x27;
                let vdata = self.calc_voltage_data(voltage, 3500, 700, 25) & 0x7f;

                let mut data = self.read(addr)?;
                data &= 0x80;
                data |= vdata;

                self.write(addr, data)?;
            }
        };

        Ok(())
    }

    fn calc_voltage_data(&self, value: u16, max_value: u16, min_value: u16, step: u16) -> u8 {
        if value > max_value { return max_value as u8; }
        if value < min_value { return min_value as u8; }
        ((value - min_value) / step) as u8
    }

    fn set_chg_current(&mut self, current: CHGCurrent) -> Result<(), Error> {
        let mut data = self.read(0x33)?;

        let state = current as u8;

        data &= 0xf0;
        data |= state & 0x0f;

        self.write(0x33, data)?;

        Ok(())
    }

    pub fn set_lcd_reset(&mut self, state: bool) -> Result<(), Error> {
        let mut data = self.read(0x96)?;

        if state {
            data |= 0x02;
        } else {
            data &= !0x02;
        }

        self.write(0x96, data)?;

        Ok(())
    }

    fn set_peripherals_power(&mut self, state: bool) -> Result<(), Error> {
        let data = self.read(0x10)?;

        if state {
            self.write(0x10, data & 0xFB)?;
        } else {
            self.write(0x10, data | 0x04)?;
        }

        Ok(())
    }

    fn set_bus_power_mode(&mut self, mode: BusPowerMode) -> Result<(), Error> {
        if mode == BusPowerMode::kMBusModeOutput {
            // Set GPIO to 3.3V (LDO OUTPUT mode)
            let data = self.read(0x91)?;
            self.write(0x91, (data & 0x0F) | 0xF0)?;
            // Set GPIO0 to LDO OUTPUT, pullup N_VBUSEN to disable VBUS supply from BUS_5V
            let data = self.read(0x90)?;
            self.write(0x90, (data & 0xF8) | 0x02)?;
            // Set EXTEN to enable 5v boost
            let data = self.read(0x10)?;
            self.write(0x10, data | 0x04)?;
        } else {
            let data = self.read(0x10)?;
            self.write(0x10, data & !0x04)?;
            let data = self.read(0x90)?;
            self.write(0x90, (data & 0xF8) | 0x07)?;
        }

        Ok(())
    }

    fn read(&mut self, addr: u8) -> Result<u8, Error> {
        let mut data = [0];
        self.i2c.write_read(self.address, &[addr], &mut data)?;
        Ok(data[0])
    }

    fn write(&mut self, addr: u8, value: u8) -> Result<(), Error> {
        self.i2c.write(self.address, &[addr, value])?;
        Ok(())
    }

}
