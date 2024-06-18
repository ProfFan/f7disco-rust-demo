#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::fmc::Fmc;
use embassy_stm32::gpio::{AfType, Flex, Level, Output, OutputType, Speed};
use embassy_stm32::ltdc::{
    B0Pin, B1Pin, B2Pin, B3Pin, B4Pin, B5Pin, B6Pin, B7Pin, ClkPin, DePin, G0Pin, G1Pin, G2Pin,
    G3Pin, G4Pin, G5Pin, G6Pin, G7Pin, HsyncPin, Ltdc, R0Pin, R1Pin, R2Pin, R3Pin, R4Pin, R5Pin,
    R6Pin, R7Pin, VsyncPin,
};
use embassy_stm32::pac::ltdc::vals::{Bf1, Bf2, Imr, Pf};
use embassy_stm32::pac::RCC;
use embassy_stm32::time::mhz;
use embassy_time::Timer;

use {defmt_rtt as _, panic_probe as _};

static FERRIS_IMAGE: &[u8; 480 * 272 * 3] = include_bytes!("image.bin");

#[embassy_executor::task()]
async fn display_task() -> ! {
    use embassy_stm32::pac::LTDC;

    info!("Display task started");

    const LCD_X_SIZE: u16 = 480;
    const LCD_Y_SIZE: u16 = 272;

    /* Initialize the LCD pixel width and pixel height */
    const WINDOW_X0: u16 = 0;
    const WINDOW_X1: u16 = LCD_X_SIZE; // 480 for ferris
    const WINDOW_Y0: u16 = 0;
    const WINDOW_Y1: u16 = LCD_Y_SIZE; // 800 for ferris
    const PIXEL_FORMAT: Pf = Pf::RGB888;
    //const FBStartAdress: u16 = FB_Address;
    const ALPHA: u8 = 255;
    const ALPHA0: u8 = 0;
    const BACKCOLOR_BLUE: u8 = 0;
    const BACKCOLOR_GREEN: u8 = 0;
    const BACKCOLOR_RED: u8 = 0;
    const IMAGE_WIDTH: u16 = LCD_X_SIZE; // 480 for ferris
    const IMAGE_HEIGHT: u16 = LCD_Y_SIZE; // 800 for ferris

    const PIXEL_SIZE: u8 = match PIXEL_FORMAT {
        Pf::ARGB8888 => 4,
        Pf::RGB888 => 3,
        Pf::ARGB4444 | Pf::RGB565 | Pf::ARGB1555 | Pf::AL88 => 2,
        _ => 1,
    };

    // Configure the horizontal start and stop position
    LTDC.layer(0).whpcr().write(|w| {
        w.set_whstpos(LTDC.bpcr().read().ahbp() + 1 + WINDOW_X0);
        w.set_whsppos(LTDC.bpcr().read().ahbp() + WINDOW_X1);
    });

    // Configures the vertical start and stop position
    LTDC.layer(0).wvpcr().write(|w| {
        w.set_wvstpos(LTDC.bpcr().read().avbp() + 1 + WINDOW_Y0);
        w.set_wvsppos(LTDC.bpcr().read().avbp() + WINDOW_Y1);
    });

    // Specify the pixel format
    LTDC.layer(0).pfcr().write(|w| w.set_pf(PIXEL_FORMAT));

    // Configures the default color values as zero
    LTDC.layer(0).dccr().modify(|w| {
        w.set_dcblue(BACKCOLOR_BLUE);
        w.set_dcgreen(BACKCOLOR_GREEN);
        w.set_dcred(BACKCOLOR_RED);
        w.set_dcalpha(ALPHA0);
    });

    // Specifies the constant ALPHA value
    LTDC.layer(0).cacr().write(|w| w.set_consta(ALPHA));

    // Specifies the blending factors
    LTDC.layer(0).bfcr().write(|w| {
        w.set_bf1(Bf1::CONSTANT);
        w.set_bf2(Bf2::CONSTANT);
    });

    // Configure the color frame buffer start address
    let fb_start_address: u32 = &FERRIS_IMAGE[0] as *const _ as u32;
    info!(
        "Setting Framebuffer Start Address: {:010x}",
        fb_start_address
    );
    LTDC.layer(0)
        .cfbar()
        .write(|w| w.set_cfbadd(fb_start_address));

    // Configures the color frame buffer pitch in byte
    LTDC.layer(0).cfblr().write(|w| {
        w.set_cfbp(IMAGE_WIDTH * PIXEL_SIZE as u16);
        w.set_cfbll(((WINDOW_X1 - WINDOW_X0) * PIXEL_SIZE as u16) + 3);
    });

    // Configures the frame buffer line number
    LTDC.layer(0)
        .cfblnr()
        .write(|w| w.set_cfblnbr(IMAGE_HEIGHT));

    // Enable LTDC_Layer by setting LEN bit
    LTDC.layer(0).cr().modify(|w| w.set_len(true));

    //LTDC->SRCR = LTDC_SRCR_IMR;
    LTDC.srcr().modify(|w| w.set_imr(Imr::RELOAD));

    loop {
        Timer::after_millis(20).await;
    }
}

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
    info!("Starting...");

    let mut core_peri = cortex_m::Peripherals::take().unwrap();
    // Config SDRAM
    // ----------------------------------------------------------
    // Configure MPU for external SDRAM
    // MPU config for SDRAM write-through
    let sdram_size = 32 * 1024 * 1024;

    {
        let mpu = core_peri.MPU;
        let scb = &mut core_peri.SCB;
        let size = sdram_size;
        // Refer to ARM®v7-M Architecture Reference Manual ARM DDI 0403
        // Version E.b Section B3.5
        const MEMFAULTENA: u32 = 1 << 16;

        unsafe {
            /* Make sure outstanding transfers are done */
            cortex_m::asm::dmb();

            scb.shcsr.modify(|r| r & !MEMFAULTENA);

            /* Disable the MPU and clear the control register*/
            mpu.ctrl.write(0);
        }

        const REGION_NUMBER0: u32 = 0x00;
        const REGION_BASE_ADDRESS: u32 = 0xD000_0000;

        const REGION_FULL_ACCESS: u32 = 0x03;
        const REGION_CACHEABLE: u32 = 0x01;
        const REGION_WRITE_BACK: u32 = 0x01;
        const REGION_ENABLE: u32 = 0x01;

        crate::assert_eq!(size & (size - 1), 0, "SDRAM memory region size must be a power of 2");
        crate::assert_eq!(size & 0x1F, 0, "SDRAM memory region size must be 32 bytes or more");
        fn log2minus1(sz: u32) -> u32 {
            for i in 5..=31 {
                if sz == (1 << i) {
                    return i - 1;
                }
            }
            crate::panic!("Unknown SDRAM memory region size!");
        }

        //info!("SDRAM Memory Size 0x{:x}", log2minus1(size as u32));

        // Configure region 0
        //
        // Cacheable, outer and inner write-back, no write allocate. So
        // reads are cached, but writes always write all the way to SDRAM
        unsafe {
            mpu.rnr.write(REGION_NUMBER0);
            mpu.rbar.write(REGION_BASE_ADDRESS);
            mpu.rasr.write(
                (REGION_FULL_ACCESS << 24)
                    | (REGION_CACHEABLE << 17)
                    | (REGION_WRITE_BACK << 16)
                    | (log2minus1(size as u32) << 1)
                    | REGION_ENABLE,
            );
        }

        const MPU_ENABLE: u32 = 0x01;
        const MPU_DEFAULT_MMAP_FOR_PRIVILEGED: u32 = 0x04;

        // Enable
        unsafe {
            mpu.ctrl.modify(|r| r | MPU_DEFAULT_MMAP_FOR_PRIVILEGED | MPU_ENABLE);

            scb.shcsr.modify(|r| r | MEMFAULTENA);

            // Ensure MPU settings take effect
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }
    }

    let mut sdram = Fmc::sdram_a12bits_d16bits_4banks_bank1(
        p.FMC,
        // A0-A11
        p.PF0,
        p.PF1,
        p.PF2,
        p.PF3,
        p.PF4,
        p.PF5,
        p.PF12,
        p.PF13,
        p.PF14,
        p.PF15,
        p.PG0,
        p.PG1,
        // BA0-BA1
        p.PG4,
        p.PG5,
        // D0-D31
        p.PD14,
        p.PD15,
        p.PD0,
        p.PD1,
        p.PE7,
        p.PE8,
        p.PE9,
        p.PE10,
        p.PE11,
        p.PE12,
        p.PE13,
        p.PE14,
        p.PE15,
        p.PD8,
        p.PD9,
        p.PD10,
        // NBL0 - NBL3
        p.PE0,
        p.PE1,
        p.PC3,  // SDCKE1
        p.PG8,  // SDCLK
        p.PG15, // SDNCAS
        p.PH3,  // SDNE0 (!CS)
        p.PF11, // SDRAS
        p.PH5,  // SDNWE
        stm32_fmc::devices::mt48lc4m32b2_6::Mt48lc4m32b2 {},
    );

    let mut delay = embassy_time::Delay;

    let ram_slice = unsafe {
        // Initialise controller and SDRAM
        let ram_ptr: *mut u32 = sdram.init(&mut delay) as *mut _;

        // Convert raw pointer to slice
        core::slice::from_raw_parts_mut(ram_ptr, sdram_size / core::mem::size_of::<u32>())
    };
    
    // // ----------------------------------------------------------
    // // Use memory in SDRAM
    info!("RAM contents before writing: {:x}", ram_slice[..10]);

    ram_slice[0] = 1;
    ram_slice[1] = 2;
    ram_slice[2] = 3;
    ram_slice[3] = 4;

    info!("RAM contents after writing: {:x}", ram_slice[..10]);

    crate::assert_eq!(ram_slice[0], 1);
    crate::assert_eq!(ram_slice[1], 2);
    crate::assert_eq!(ram_slice[2], 3);
    crate::assert_eq!(ram_slice[3], 4);

    info!("Assertions succeeded.");


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

    let ltdc_b4_af = B4Pin::af_num(&p.PG12);
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

    critical_section::with(|_cs| {
        // RM says the pllsaidivr should only be changed when pllsai is off. But this could have other unintended side effects. So let's just give it a try like this.
        // According to the debugger, this bit gets set, anyway.
        RCC.dckcfgr1()
            .modify(|w| w.set_pllsaidivr(embassy_stm32::pac::rcc::vals::Pllsaidivr::DIV8));
    });

    embassy_stm32::rcc::enable_and_reset::<embassy_stm32::peripherals::LTDC>();

    ltdc.disable();

    use embassy_stm32::pac::LTDC;

    // Set the LTDC to 480x272
    LTDC.gcr().modify(|w| {
        w.set_hspol(embassy_stm32::pac::ltdc::vals::Hspol::ACTIVELOW);
        w.set_vspol(embassy_stm32::pac::ltdc::vals::Vspol::ACTIVELOW);
        w.set_depol(embassy_stm32::pac::ltdc::vals::Depol::ACTIVELOW);
        w.set_pcpol(embassy_stm32::pac::ltdc::vals::Pcpol::RISINGEDGE);
    });

    // Set Sync signals
    LTDC.sscr().write(|w| {
        w.set_hsw(41);
        w.set_vsh(10);
    });

    // Set Accumulated Back porch
    LTDC.bpcr().modify(|w| {
        w.set_ahbp(53);
        w.set_avbp(11);
    });

    // Set Accumulated Active Width
    LTDC.awcr().modify(|w| {
        w.set_aah(283);
        w.set_aaw(533);
    });

    // Set Total Width
    LTDC.twcr().modify(|w| {
        w.set_totalh(285);
        w.set_totalw(565);
    });

    // Set the background color value
    LTDC.bccr().modify(|w| {
        w.set_bcred(0);
        w.set_bcgreen(0);
        w.set_bcblue(0)
    });

    // Enable the Transfer Error and FIFO underrun interrupts
    LTDC.ier().modify(|w| {
        w.set_terrie(true);
        w.set_fuie(true);
    });

    // Enable the LTDC
    ltdc.enable();

    // Start the display task
    let spawner = Spawner::for_current_executor().await;

    spawner.spawn(display_task()).unwrap();

    let mut led = Output::new(p.PI1, Level::High, Speed::Low);

    loop {
        info!("high");
        led.set_high();
        Timer::after_millis(1000).await;

        info!("low");
        led.set_low();
        Timer::after_millis(1000).await;
    }
}
