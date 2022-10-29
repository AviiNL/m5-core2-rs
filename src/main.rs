#![no_std]
#![no_main]

extern crate alloc;

use bitflags::bitflags;

use esp32_hal::{
    clock::ClockControl,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
    i2c::I2C,
    IO,
    spi::Spi, spi::SpiMode
};

use esp_backtrace as _;
use esp_println::println;
use mipidsi::instruction::Instruction;

use display_interface::{DataFormat, WriteOnlyDataCommand};

use crate::axp192::Axp192;
use crate::error::Error;


mod axp192;
pub mod error;

// TODO: Move me to display driver
bitflags! {
    struct MAD: u8 {
        const MAD_MY = 0x80;
        const MAD_MX = 0x40;
        const MAD_MV = 0x20;
        const MAD_ML = 0x10;
        const MAD_BGR = 0x08;
        const MAD_MH = 0x04;
        const MAD_RGB = 0x00;
    }
}



#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
        static mut _heap_end: u32;
    }

    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        let heap_end = &_heap_end as *const _ as usize;
        assert!(
            heap_end - heap_start > HEAP_SIZE,
            "Not enough available heap memory."
        );
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}

#[xtensa_lx_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.DPORT.split();
    let mut p_clock_control = system.peripheral_clock_control;
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut delay = Delay::new(&clocks);

    init_heap();
    
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    
    let i2c0 = match I2C::new(
        peripherals.I2C0,
        io.pins.gpio21,
        io.pins.gpio22,
        400u32.kHz(),
        &mut p_clock_control,
        &clocks,
    ) {
        Ok(i2c) => i2c,
        Err(e) => panic!("Failed to initialize I2C: {:?}", e),
    };

    let mut axp = Axp192::new(i2c0);
    
    match axp.init(&mut delay) {
        Ok(_) => println!("AXP192 initialized"),
        Err(e) => println!("AXP192 init failed: {:?}", e),
    };

    let mut spi = Spi::new_no_cs(
        peripherals.SPI2,
        io.pins.gpio18,
        io.pins.gpio23,
        io.pins.gpio38,
        10u32.kHz(),
        SpiMode::Mode0,
        &mut p_clock_control,
        &clocks,
    );

    let gpio4 = io.pins.gpio4;
    gpio4.into_push_pull_output().set_low().unwrap();

    let gpio5 = io.pins.gpio5;
    gpio5.into_push_pull_output().set_high().unwrap();

    // match spi.write(&[0x04]) {
    //     Ok(_) => println!("SPI write ok"),
    //     Err(e) => println!("SPI write failed: {:?}", e),
    // };

    // let di = SPIInterfaceNoCS::new(spi, io.pins.gpio15.into_push_pull_output());

    // let mut display = match Builder::ili9342c_rgb565(di)
    //     .with_orientation(Orientation::Portrait(false))
    //     .init(&mut delay, None::<Gpio19<Output<PushPull>>>) {
    //         Ok(display) => display,
    //         Err(e) => panic!("Failed to initialize display: {:?}", e),
    //     };

    // let (mut di, _, _) = display.release();

    // let (mut spi, _) = di.release();

    // Start with a software reset
    // match spi.write(&[Instruction::SWRESET as u8]) {
    //     Ok(_) => println!("SWRESET"),
    //     Err(e) => println!("SWRESET failed: {:?}", e),
    // };

    // // // delay 120ms
    // delay.delay_ms(240u32);

    // // // Turn on the external command if setting EXTC1[7:0] = 0xFF, EXTC2[7:0] = 0x93, and EXTC3[7:0] = 0x42
    // match spi.write(&[0xc8, 0xFF, 0x93, 0x42]) {
    //     Ok(_) => println!("EXTC command sent"),
    //     Err(e) => println!("EXTC command failed: {:?}", e),
    // };

    delay.delay_ms(240u32);

    let mut buffer = [0x04, 0, 0, 0];
    let result = match spi.transfer(&mut buffer) {
        Ok(res) => {
            println!("SPI transfer ok");
            res
        },
        Err(e) => panic!("SPI transfer failed: {:?}", e),
    };

    println!("Result: {:x?}", result);

    

    // 

    
    
    // match display.clear(Rgb565::BLUE) {
    //     Ok(_) => println!("Display cleared"),
    //     Err(e) => println!("Display clear failed: {:?}", e),
    // };


    

    // Start with a software reset
    // match write_command(&mut di, Instruction::SWRESET, &[]) {
    //     Ok(_) => println!("SWRESET"),
    //     Err(e) => println!("SWRESET failed: {:?}", e),
    // };

    // // delay 120ms
    // delay.delay_ms(240u32);

    // // Turn on the external command if setting EXTC1[7:0] = 0xFF, EXTC2[7:0] = 0x93, and EXTC3[7:0] = 0x42
    // match write_command(&mut di, Instruction::EXTC, &[0xFF, 0x93, 0x42]) {
    //     Ok(_) => println!("EXTC command sent"),
    //     Err(e) => println!("EXTC command failed: {:?}", e),
    // };

    // // PWCTR1
    // match write_command(&mut di, Instruction::PWR1, &[
    //     0x12, // VRH1[4:0]: Sets the VREG1OUT voltage for positive gamma 
    //     0x12, // VRH2[4:0]: Sets the VREG2OUT voltage for negative gamma
    // ]) {
    //     Ok(_) => println!("PWR1 command sent"),
    //     Err(e) => println!("PWR1 command failed: {:?}", e),
    // };

    // // PWCTR2
    // match write_command(&mut di, Instruction::PWR2, &[
    //     0x03, // Sets the factor used in the step-up circuits. 
    // ]) {
    //     Ok(_) => println!("PWR2 command sent"),
    //     Err(e) => println!("PWR2 command failed: {:?}", e),
    // };

    // // RGB Interface Signal Control
    // // Sets the operation status of the display interface. The setting becomes effective as soon as the command is received. 
    // match write_command(&mut di, Instruction::RGBISC, &[
    //     0x80, // 0b10000000 Set ByPass_MODE to 1 (Memory)
    // ]) {
    //     Ok(_) => println!("RGBISC command sent"),
    //     Err(e) => println!("RGBISC command failed: {:?}", e),
    // };

    // // Interface Control
    // // The set value of MADCTL is used in the IC is derived as exclusive OR between 1st Parameter of IFCTL and MADCTL
    // match write_command(&mut di, Instruction::IC, &[
    //     0x00, // WEMODE=0: When the transfer number of data exceeds (EC-SC+1)*(EP-SP+1), the exceeding data will be ignored. 
    //     0x01, // Select the method of display data transferring. (Not sure to what though)
    //     0x01, // Specify the RGB interface mode when the RGB interface is used. These bits should be set before display operation
    //           // through the RGB interface and should not be set during operation. ( 6- bit RGB interface (3 transfer/pixel) )
    //           // If COLMOD == b110, we get 262k colors, if COLMOD == b101, we get 65k colors
    //           // This byte also contains Endianess (D5), 1 = LSB, 0 = MSB (default)
    //           // Display operation mode is set to Internal Clock Operation (0:0)
    // ]) {
    //     Ok(_) => println!("IC command sent"),
    //     Err(e) => println!("IC command failed: {:?}", e),
    // };

    // match write_command(&mut di, Instruction::MADCTL, &[
    //     MAD::MAD_MY.bits | // Page Address Order (MY): 1 = Bottom to top) (Image will be upside down?)
    //     MAD::MAD_MV.bits | // Vertical Refresh Order Bit (MV): 1 = Left to right (Image will be Rotated 90°CC?)
    //     MAD::MAD_BGR.bits  // RGB-BGR Order (BGR): 1 = BGR
    // ]) {
    //     Ok(_) => println!("MADCTL command sent"),
    //     Err(e) => println!("MADCTL command failed: {:?}", e),
    // };

    // // ILI9341_PIXFMT
    // match write_command(&mut di, Instruction::COLMOD, &[
    //     0x55, // 0b01010101 16-bit/pixel
    // ]) {
    //     Ok(_) => println!("COLMOD command sent"),
    //     Err(e) => println!("COLMOD command failed: {:?}", e),
    // };

    // // ILI9341_DFUNCTR
    // match write_command(&mut di, Instruction::DFC, &[
    //     0x08, // 0b00001000 // Interval scan
    //     0x82, // 0b10000010 // LCD Type (black or white = white) and REV = 5 frames (85ms)
    //     0x27, // 0b00100111 // NL =  This value is not allowed O_o unless it's reversed in the docs, then it's 240 lines (which makes sense because 320x240)
    //     0x00, // 4th parameter omitted? PCDIV (weird formula anyway)
    // ]) {
    //     Ok(_) => println!("DFUNC command sent"),
    //     Err(e) => println!("DFUNC command failed: {:?}", e),
    // };

    // // Positive Gamma Control
    // // Set the gray scale voltage to adjust the gamma characteristics of the TFT panel.
    // match write_command(&mut di, Instruction::PGC, &[
    //     0x00,0x0C,0x11,0x04,
    //     0x11,0x08,0x37,0x89,
    //     0x4C,0x06,0x0C,0x0A,
    //     0x2E,0x34,0x0F,
    // ]) {
    //     Ok(_) => println!("PGC command sent"),
    //     Err(e) => println!("PGC command failed: {:?}", e),
    // };

    // // Negative Gamma Correction
    // // Set the gray scale voltage to adjust the gamma characteristics of the TFT panel.
    // match write_command(&mut di, Instruction::NGC, &[
    //     0x00,0x0B,0x11,0x05,
    //     0x13,0x09,0x33,0x67,
    //     0x48,0x07,0x0E,0x0B,
    //     0x2E,0x33,0x0F,
    // ]) {
    //     Ok(_) => println!("NGC command sent"),
    //     Err(e) => println!("NGC command failed: {:?}", e),
    // };

    // // exit sleep
    // match write_command(&mut di, Instruction::SLPOUT, &[]) {
    //     Ok(_) => println!("SLPOUT command sent"),
    //     Err(e) => println!("SLPOUT command failed: {:?}", e),
    // };

    // // need to somehow set pin 5 high here, but SPI / DI has ownership of the pin

    // // gpio_out_w1tc.write(|w| unsafe { w.bits(1<<5) });
    // // unsafe {
    // //     esp_hal_common::pac::Peripherals::steal().GPIO.out1_w1ts.write(|w| w.bits(1<<5));
    // // }

    // // wait for 120ms
    // delay.delay_ms(120u32);

    // // set CS low
    // // need to somehow set pin 5 low here, but SPI / DI has ownership of the pin
    // // unsafe { Peripherals::steal().GPIO.out1_w1ts.write(|w| w.bits(1<<5)); }
    // //     Peripherals::steal().GPIO.out1_w1tc.write(|w| w.bits(1<<5));
    // // }

    // // display on
    // match write_command(&mut di, Instruction::DISPON, &[]) {
    //     Ok(_) => println!("DISPON command sent"),
    //     Err(e) => println!("DISPON command failed: {:?}", e),
    // };

    // match write_command(&mut di, Instruction::RAMWR, &[]) {
    //     Ok(_) => println!("RAMWR command sent"),
    //     Err(e) => println!("RAMWR command failed: {:?}", e),
    // };

    // // loop 320x240 times
    // for _ in 0..(320*240) { // make screen blue?
    //     write_16(&mut di, 0x00FE).unwrap();
    // }

    // println!("Screen should now be blue!");

    // let mut display = match Builder::ili9342c_rgb565(di)
    //     .init(&mut delay, None::<Gpio19<Output<PushPull>>>) {
    //     Ok(display) => {
    //         println!("Display initialized");
    //         display
    //     },
    //     Err(e) => panic!("Failed to initialize display: {:?}", e),
    // };

    // match display.clear(Rgb565::GREEN) {
    //     Ok(_) => println!("Display cleared"),
    //     Err(e) => println!("Display clear failed: {:?}", e),
    // };

    loop {
        delay.delay_ms(100u32);
    }
}

pub fn write_command<DI>(di: &mut DI, command: Instruction, params: &[u8]) -> Result<(), Error>
where
    DI: WriteOnlyDataCommand,
{
    di.send_commands(DataFormat::U8(&[command as u8]))?;

    if !params.is_empty() {
        di.send_data(DataFormat::U8(params))?;
        Ok(())
    } else {
        Ok(())
    }
}

// pub fn read_byte<DI>(di: &mut DI, command: Instruction) -> Result<u8, Error>
// where
//     DI: WriteOnlyDataCommand,
// {
//     di.send_commands(DataFormat::U8(&[command as u8]))?;
//     let mut buf = [0u8; 1];
//     di.read_data(DataFormat::U8(&mut buf))?;
//     Ok(buf[0])
// }

pub fn write_16<DI>(di: &mut DI, data: u16) -> Result<(), Error>
where
    DI: WriteOnlyDataCommand,
{   
    // Convert 16 bit colour to 18 bit and write in 3 bytes
    let a = ((data & 0xF800) >> 8) as u8;
    let b = ((data & 0x07E0) >> 3) as u8;
    let c = ((data & 0x001F) << 3) as u8;

    let data = [a, b, c];

    di.send_data(DataFormat::U8(&data))?;

    Ok(())
}
