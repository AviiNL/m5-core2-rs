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
use tft_spi::TftSpi;

use crate::axp192::Axp192;

mod axp192;
mod tft_spi;
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

    let tft_spi = Spi::new_no_cs(
        peripherals.SPI3,
        io.pins.gpio18,
        io.pins.gpio23,
        io.pins.gpio38,
        40u32.MHz(),
        SpiMode::Mode0,
        &mut p_clock_control,
        &clocks,
    );

    // let mut sd = io.pins.gpio4.into_push_pull_output();
    let dc = io.pins.gpio15.into_push_pull_output();
    let cs = io.pins.gpio5.into_push_pull_output();

    let mut tft_spi = TftSpi::new(tft_spi, dc, cs);

    tft_spi.init(&mut delay);

    // for loop 150..170 as x
    for x in 150..170 {
        // for loop 150..170 as y
        for y in 110..130 {
            // write_data!(0x2C, [0xFF, 0x00, 0x00]);
            tft_spi.draw_pixel(x, y, 0, &mut delay);
        }
    }

    println!("Square drawn");

    loop {
        delay.delay_ms(100u32);
    }
}
