#![no_std]
#![no_main]

mod display_target;
mod display_task;
mod progress_bar;
mod sdram_drv;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::fmc::Fmc;
use embassy_stm32::gpio::{AfType, Flex, Level, Output, OutputType, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::ltdc::{
    B0Pin, B1Pin, B2Pin, B3Pin, B4Pin, B5Pin, B6Pin, B7Pin, ClkPin, DePin, G0Pin, G1Pin, G2Pin,
    G3Pin, G4Pin, G5Pin, G6Pin, G7Pin, HsyncPin, Ltdc, LtdcConfiguration, PolarityActive,
    PolarityEdge, R0Pin, R1Pin, R2Pin, R3Pin, R4Pin, R5Pin, R6Pin, R7Pin, VsyncPin,
};
use embassy_stm32::time::mhz;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::Timer;

extern crate alloc;
use embedded_alloc::TlsfHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use {defmt_rtt as _, panic_probe as _};

static TOUCH_SIGNAL: embassy_sync::signal::Signal<ThreadModeRawMutex, ft5336::TouchState> =
    embassy_sync::signal::Signal::new();

bind_interrupts!(struct Irqs {
    I2C3_EV => embassy_stm32::i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C3>;
    I2C3_ER => embassy_stm32::i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllRDiv, PllSource, Sysclk,
    };

    let mut config = embassy_stm32::Config::default();
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;

    // HSE is on and ready
    config.rcc.hse = Some(Hse {
        freq: mhz(25),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;

    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV25,  // PLLM
        mul: PllMul::MUL400,       // PLLN
        divp: Some(PllPDiv::DIV2), // SYSCLK = 400/2 = 200 MHz
        divq: Some(PllQDiv::DIV9), // PLLQ = 400/9 = 44.44 MHz
        divr: None,
    });

    // This seems to be working, the values in the RCC.PLLSAICFGR are correct according to the debugger. Also on and ready according to CR
    config.rcc.pllsai = Some(Pll {
        prediv: PllPreDiv::DIV25,  // Actually ignored
        mul: PllMul::MUL384,       // PLLN
        divp: Some(PllPDiv::DIV8), // PLLP
        divq: Some(PllQDiv::DIV2), // PLLQ
        divr: Some(PllRDiv::DIV5), // PLLR
    });

    // PLLI2S
    config.rcc.plli2s = Some(Pll {
        prediv: PllPreDiv::DIV25,  // Actually ignored
        mul: PllMul::MUL100,       // PLLN
        divp: Some(PllPDiv::DIV2), // PLLP
        divq: Some(PllQDiv::DIV2), // PLLQ
        divr: Some(PllRDiv::DIV2), // PLLR (I2S PLLR is always 2)
    });

    let p = embassy_stm32::init(config);

    // This is very important, I2C won't work without it
    // Not sure why, the RM doesn't say anything about this
    embassy_time::Timer::after(embassy_time::Duration::from_millis(40)).await;

    info!("Starting...");

    // Config SDRAM
    // ----------------------------------------------------------
    // Configure MPU for external SDRAM (64 Mbit = 8 Mbyte)
    // MPU is disabled by default
    const SDRAM_SIZE: usize = 8 * 1024 * 1024;

    #[rustfmt::skip]
    let mut sdram = Fmc::sdram_a12bits_d16bits_4banks_bank1(
        p.FMC,
        // A0-A11
        p.PF0, p.PF1, p.PF2, p.PF3, p.PF4, p.PF5, p.PF12, p.PF13, p.PF14, p.PF15, p.PG0, p.PG1,
        // BA0-BA1
        p.PG4, p.PG5,
        // D0-D15
        p.PD14, p.PD15, p.PD0, p.PD1, p.PE7, p.PE8, p.PE9, p.PE10, p.PE11, p.PE12, p.PE13, p.PE14, p.PE15, p.PD8, p.PD9, p.PD10,
        // NBL0 - NBL1
        p.PE0, p.PE1,
        p.PC3,  // SDCKE0
        p.PG8,  // SDCLK
        p.PG15, // SDNCAS
        p.PH3,  // SDNE0 (!CS)
        p.PF11, // SDRAS
        p.PH5,  // SDNWE
        sdram_drv::mt48lc4m32b2_6::Mt48lc4m32b2 {},
    );

    let mut delay = embassy_time::Delay;

    unsafe {
        // Initialise controller and SDRAM
        let ram_ptr: *mut u32 = sdram.init(&mut delay) as *mut _;

        info!("SDRAM Initialized at {:x}", ram_ptr as usize);

        // Convert raw pointer to slice
        HEAP.init(ram_ptr as usize, SDRAM_SIZE)
    };

    // Configure the LTDC Pins
    const DATA_AF: AfType = AfType::output(OutputType::PushPull, Speed::Low);

    // R: PI15, PJ0..6, 8 bits
    let ltdc_r0_af = p.PI15.af_num();
    let mut ltdc_r0 = Flex::new(p.PI15);
    ltdc_r0.set_as_af_unchecked(ltdc_r0_af, DATA_AF);

    let ltdc_r1_af = p.PJ0.af_num();
    let mut ltdc_r1 = Flex::new(p.PJ0);
    ltdc_r1.set_as_af_unchecked(ltdc_r1_af, DATA_AF);

    let ltdc_r2_af = p.PJ1.af_num();
    let mut ltdc_r2 = Flex::new(p.PJ1);
    ltdc_r2.set_as_af_unchecked(ltdc_r2_af, DATA_AF);

    let ltdc_r3_af = p.PJ2.af_num();
    let mut ltdc_r3 = Flex::new(p.PJ2);
    ltdc_r3.set_as_af_unchecked(ltdc_r3_af, DATA_AF);

    let ltdc_r4_af = p.PJ3.af_num();
    let mut ltdc_r4 = Flex::new(p.PJ3);
    ltdc_r4.set_as_af_unchecked(ltdc_r4_af, DATA_AF);

    let ltdc_r5_af = p.PJ4.af_num();
    let mut ltdc_r5 = Flex::new(p.PJ4);
    ltdc_r5.set_as_af_unchecked(ltdc_r5_af, DATA_AF);

    let ltdc_r6_af = p.PJ5.af_num();
    let mut ltdc_r6 = Flex::new(p.PJ5);
    ltdc_r6.set_as_af_unchecked(ltdc_r6_af, DATA_AF);

    let ltdc_r7_af = p.PJ6.af_num();
    let mut ltdc_r7 = Flex::new(p.PJ6);
    ltdc_r7.set_as_af_unchecked(ltdc_r7_af, DATA_AF);

    // G: PJ7..11, PK0..2, 8 bits
    let ltdc_g0_af = p.PJ7.af_num();
    let mut ltdc_g0 = Flex::new(p.PJ7);
    ltdc_g0.set_as_af_unchecked(ltdc_g0_af, DATA_AF);

    let ltdc_g1_af = p.PJ8.af_num();
    let mut ltdc_g1 = Flex::new(p.PJ8);
    ltdc_g1.set_as_af_unchecked(ltdc_g1_af, DATA_AF);

    let ltdc_g2_af = p.PJ9.af_num();
    let mut ltdc_g2 = Flex::new(p.PJ9);
    ltdc_g2.set_as_af_unchecked(ltdc_g2_af, DATA_AF);

    let ltdc_g3_af = p.PJ10.af_num();
    let mut ltdc_g3 = Flex::new(p.PJ10);
    ltdc_g3.set_as_af_unchecked(ltdc_g3_af, DATA_AF);

    let ltdc_g4_af = p.PJ11.af_num();
    let mut ltdc_g4 = Flex::new(p.PJ11);
    ltdc_g4.set_as_af_unchecked(ltdc_g4_af, DATA_AF);

    let ltdc_g5_af = p.PK0.af_num();
    let mut ltdc_g5 = Flex::new(p.PK0);
    ltdc_g5.set_as_af_unchecked(ltdc_g5_af, DATA_AF);

    let ltdc_g6_af = p.PK1.af_num();
    let mut ltdc_g6 = Flex::new(p.PK1);
    ltdc_g6.set_as_af_unchecked(ltdc_g6_af, DATA_AF);

    let ltdc_g7_af = p.PK2.af_num();
    let mut ltdc_g7 = Flex::new(p.PK2);
    ltdc_g7.set_as_af_unchecked(ltdc_g7_af, DATA_AF);

    // B: PE4, PJ13..15, PG12, PK4..6, 8 bits
    let ltdc_b0_af = p.PE4.af_num();
    let mut ltdc_b0 = Flex::new(p.PE4);
    ltdc_b0.set_as_af_unchecked(ltdc_b0_af, DATA_AF);

    let ltdc_b1_af = p.PJ13.af_num();
    let mut ltdc_b1 = Flex::new(p.PJ13);
    ltdc_b1.set_as_af_unchecked(ltdc_b1_af, DATA_AF);

    let ltdc_b2_af = p.PJ14.af_num();
    let mut ltdc_b2 = Flex::new(p.PJ14);
    ltdc_b2.set_as_af_unchecked(ltdc_b2_af, DATA_AF);

    let ltdc_b3_af = p.PJ15.af_num();
    let mut ltdc_b3 = Flex::new(p.PJ15);
    ltdc_b3.set_as_af_unchecked(ltdc_b3_af, DATA_AF);

    let ltdc_b4_af = <embassy_stm32::peripherals::PG12 as B4Pin<
        embassy_stm32::peripherals::LTDC,
    >>::af_num(&p.PG12);
    let mut ltdc_b4 = Flex::new(p.PG12);
    ltdc_b4.set_as_af_unchecked(ltdc_b4_af, DATA_AF);

    let ltdc_b5_af = p.PK4.af_num();
    let mut ltdc_b5 = Flex::new(p.PK4);
    ltdc_b5.set_as_af_unchecked(ltdc_b5_af, DATA_AF);

    let ltdc_b6_af = p.PK5.af_num();
    let mut ltdc_b6 = Flex::new(p.PK5);
    ltdc_b6.set_as_af_unchecked(ltdc_b6_af, DATA_AF);

    let ltdc_b7_af = p.PK6.af_num();
    let mut ltdc_b7 = Flex::new(p.PK6);
    ltdc_b7.set_as_af_unchecked(ltdc_b7_af, DATA_AF);

    // HSYNC: PI10
    let ltdc_hsync_af = p.PI10.af_num();
    let mut ltdc_hsync = Flex::new(p.PI10);
    ltdc_hsync.set_as_af_unchecked(ltdc_hsync_af, DATA_AF);

    // VSYNC: PI9
    let ltdc_vsync_af = p.PI9.af_num();
    let mut ltdc_vsync = Flex::new(p.PI9);
    ltdc_vsync.set_as_af_unchecked(ltdc_vsync_af, DATA_AF);

    // CLK: PI14
    let ltdc_clk_af = p.PI14.af_num();
    let mut ltdc_clk = Flex::new(p.PI14);
    ltdc_clk.set_as_af_unchecked(ltdc_clk_af, DATA_AF);

    // DE: PK7
    let ltdc_de_af = p.PK7.af_num();
    let mut ltdc_de = Flex::new(p.PK7);
    ltdc_de.set_as_af_unchecked(ltdc_de_af, DATA_AF);

    // Enable the LCD-TFT controller
    let _lcd_en = Output::new(p.PI12, Level::High, Speed::Low);

    // Enable the backlight
    let _backlight = Output::new(p.PK3, Level::High, Speed::Low);

    let mut ltdc = Ltdc::new(p.LTDC);

    // Setup PllSAI for LTDC clock
    {
        // Enable PLLSAI
        embassy_stm32::pac::RCC
            .dckcfgr1()
            .modify(|w| w.set_pllsaidivr(embassy_stm32::pac::rcc::vals::Pllsaidivr::DIV8));
    }

    let ltdc_config = LtdcConfiguration {
        active_width: 533,
        active_height: 283,
        h_back_porch: 53,
        h_front_porch: 0,
        v_back_porch: 11,
        v_front_porch: 0,
        h_sync: 41,
        v_sync: 10,
        h_sync_polarity: PolarityActive::ActiveLow,
        v_sync_polarity: PolarityActive::ActiveLow,
        data_enable_polarity: PolarityActive::ActiveLow,
        pixel_clock_polarity: PolarityEdge::RisingEdge,
    };
    ltdc.init(&ltdc_config);

    let mut i2c_config = embassy_stm32::i2c::Config::default();
    i2c_config.frequency = embassy_stm32::time::Hertz(400_000);
    i2c_config.gpio_speed = Speed::Low;
    i2c_config.scl_pullup = false;
    i2c_config.sda_pullup = false;
    i2c_config.timeout = embassy_time::Duration::from_millis(1000);

    let mut i2c = I2c::new_blocking(p.I2C3, p.PH7, p.PH8, i2c_config);
    // Scan for devices
    {
        info!("Scanning I2C bus...");
        let mut found = 0;
        for addr in [0x38, 0x1a] {
            match i2c.blocking_write(addr, &[]) {
                Ok(()) => {
                    info!("Found device at address {:x}", addr);
                    found += 1;
                }
                Err(_e) => {}
            }
        }
        info!("Found {} devices", found);
    }

    {
        let mut buf: [u8; 1] = [0];
        // Read the ID register of the FT5336
        // Addr: 0xA8
        match i2c.blocking_read(0x38, &mut buf) {
            Ok(_) => info!("FT5336: read default {:x}", buf[0]),
            Err(e) => info!("FT5336: error reading default {:?}", e),
        }
        match i2c.blocking_read(0x38, &mut buf) {
            Ok(_) => info!("FT5336: read default 2 {:x}", buf[0]),
            Err(e) => info!("FT5336: error reading default {:?}", e),
        }
        match i2c.blocking_write(0x38, &[0xA8]) {
            Ok(()) => info!("FT5336: wrote register address"),
            Err(e) => info!("FT5336: error writing register address: {:?}", e),
        }
        let id = match i2c.blocking_write_read(0x38, &[0xA8], &mut buf) {
            Ok(()) => Ok(buf[0]),
            Err(e) => Err(e),
        };
        info!("Touch controller ID: {:#x}", id);
    }

    let mut touch = ft5336::Ft5336::new(&i2c, 0x38, &mut delay).unwrap();
    touch.init(&mut i2c);

    // let dev_mode_result = touch.dev_mode(&mut i2c, 1);
    // info!("Set dev mode result: {:?}", dev_mode_result);

    info!(
        "Touch controller initialized, chip id: {:x}",
        touch.chip_id(&mut i2c)
    );

    // Start the display task
    _spawner.spawn(display_task::display_task()).unwrap();

    let mut led = Output::new(p.PI1, Level::High, Speed::Low);

    let mut touch_detected: bool = false;
    loop {
        Timer::after_millis(20).await;

        let touch_det = touch.detect_touch(&mut i2c);
        match touch_det {
            Ok(count) => {
                for i in 0..count {
                    match touch.get_touch(&mut i2c, i + 1) {
                        Ok(touch_data) => {
                            trace!("Touch detected: {:?}", defmt::Debug2Format(&touch_data));
                            if i == 0 {
                                // Only send the first touch point
                                let _ = TOUCH_SIGNAL.signal(touch_data);
                                touch_detected = true;
                            }
                        }
                        Err(e) => {
                            error!("Error getting touch data: {:?}", e);
                        }
                    }
                }
                if count == 0 && touch_detected {
                    touch_detected = false;

                    let _ = TOUCH_SIGNAL.signal(ft5336::TouchState {
                        event: ft5336::TouchEvent::LiftUp,
                        x: 0,
                        y: 0,
                        weight: 0,
                        misc: 0,
                    });
                }
            }
            Err(e) => {
                info!("Error detecting touch: {:?}", e);
            }
        }
    }
}
