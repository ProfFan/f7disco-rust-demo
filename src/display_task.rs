use alloc::boxed::Box;
use defmt::*;
use embassy_stm32::pac::ltdc::vals::{Bf1, Bf2, Imr, Pf};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::Timer;
use embedded_graphics::geometry::Size;
use embedded_graphics::mono_font::{self, ascii};
use embedded_graphics::pixelcolor::{Rgb888, RgbColor, WebColors};
use ft5336::TouchState;
use kolibri_embedded_gui::button::Button;
use kolibri_embedded_gui::checkbox::Checkbox;
use kolibri_embedded_gui::icon::IconWidget;
use kolibri_embedded_gui::iconbutton::IconButton;
use kolibri_embedded_gui::icons::size24px;
use kolibri_embedded_gui::label::Label;
use kolibri_embedded_gui::smartstate::SmartstateProvider;
use kolibri_embedded_gui::ui::{Interaction, Ui};

use crate::display_target;

const LCD_X_SIZE: u16 = 480;
const LCD_Y_SIZE: u16 = 272;

pub fn medsize_rgb888_style() -> kolibri_embedded_gui::style::Style<Rgb888> {
    kolibri_embedded_gui::style::Style {
        background_color: Rgb888::new(0x40, 0x80, 0x40), // pretty dark gray
        item_background_color: Rgb888::new(0x20, 0x40, 0x20), // darker gray
        highlight_item_background_color: Rgb888::new(0x10, 0x20, 0x10),
        border_color: Rgb888::WHITE,
        highlight_border_color: Rgb888::WHITE,
        primary_color: Rgb888::CSS_DARK_CYAN,
        secondary_color: Rgb888::YELLOW,
        icon_color: Rgb888::WHITE,
        text_color: Rgb888::WHITE,
        default_widget_height: 16,
        border_width: 0,
        highlight_border_width: 1,
        corner_radius: 0,
        default_font: mono_font::iso_8859_10::FONT_9X15,
        spacing: kolibri_embedded_gui::style::Spacing {
            item_spacing: Size::new(8, 4),
            button_padding: Size::new(5, 5),
            default_padding: Size::new(1, 1),
            window_border_padding: Size::new(3, 3),
        },
    }
}

/// Pink theme for RGB888 displays.
///
/// Features a peach background with pink accents and black text.
pub fn medsize_sakura_rgb888_style() -> kolibri_embedded_gui::style::Style<Rgb888> {
    kolibri_embedded_gui::style::Style {
        background_color: Rgb888::CSS_PEACH_PUFF,
        item_background_color: Rgb888::CSS_LIGHT_PINK,
        highlight_item_background_color: Rgb888::CSS_HOT_PINK,
        border_color: Rgb888::CSS_WHITE,
        highlight_border_color: Rgb888::CSS_BLACK,
        primary_color: Rgb888::CSS_DEEP_PINK,
        secondary_color: Rgb888::YELLOW,
        icon_color: Rgb888::CSS_BLACK,
        text_color: Rgb888::CSS_BLACK,
        default_widget_height: 16,
        border_width: 0,
        highlight_border_width: 1,
        default_font: mono_font::ascii::FONT_9X15,
        spacing: kolibri_embedded_gui::style::Spacing {
            item_spacing: Size::new(8, 4),
            button_padding: Size::new(6, 5),
            default_padding: Size::new(1, 1),
            window_border_padding: Size::new(3, 3),
        },
        corner_radius: 8,
    }
}

use embassy_stm32::pac::LTDC;
const DISPLAY_BUFFER_SIZE: usize = LCD_X_SIZE as usize * LCD_Y_SIZE as usize;

async fn setup_display(
    buffer_1: &mut Box<[u32; DISPLAY_BUFFER_SIZE]>,
    buffer_2: &mut Box<[u32; DISPLAY_BUFFER_SIZE]>,
) {
    /* Initialize the LCD pixel width and pixel height */
    const WINDOW_X0: u16 = 0;
    const WINDOW_X1: u16 = LCD_X_SIZE; // 480 for ferris
    const WINDOW_Y0: u16 = 0;
    const WINDOW_Y1: u16 = LCD_Y_SIZE; // 800 for ferris
    const PIXEL_FORMAT: Pf = Pf::ARGB8888;
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

    LTDC.layer(0)
        .cfbar()
        .write(|w| w.set_cfbadd(&buffer_1[0] as *const _ as u32));

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

    // Disable the layer
    LTDC.layer(0).cr().modify(|w| w.set_len(false));

    // replace the buffer with the new one
    LTDC.layer(0)
        .cfbar()
        .write(|w| w.set_cfbadd(&buffer_1[0] as *const _ as u32));

    // Configures the color frame buffer pitch in byte
    LTDC.layer(0).cfblr().write(|w| {
        w.set_cfbp(IMAGE_WIDTH * 4 as u16);
        w.set_cfbll(((WINDOW_X1 - WINDOW_X0) * 4 as u16) + 3);
    });

    // Configures the frame buffer line number
    LTDC.layer(0)
        .cfblnr()
        .write(|w| w.set_cfblnbr(IMAGE_HEIGHT));

    // Use ARGB8888 pixel format
    LTDC.layer(0).pfcr().write(|w| w.set_pf(Pf::ARGB8888));

    // Enable the layer
    LTDC.layer(0).cr().modify(|w| w.set_len(true));

    // Immediately refresh the display
    LTDC.srcr().modify(|w| w.set_imr(Imr::RELOAD));
}

#[embassy_executor::task()]
pub async fn display_task() -> ! {
    info!("Display task started");

    // Allocate a buffer for the display on the heap
    let mut display_buffer_1 = Box::<[u32; DISPLAY_BUFFER_SIZE]>::new([0; DISPLAY_BUFFER_SIZE]);
    let mut display_buffer_2 = Box::<[u32; DISPLAY_BUFFER_SIZE]>::new([0; DISPLAY_BUFFER_SIZE]);
    info!(
        "Display buffer allocated at {:x}, {:x}",
        &display_buffer_1[0] as *const _, &display_buffer_2[0] as *const _
    );

    // Setup the display controller
    setup_display(&mut display_buffer_1, &mut display_buffer_2).await;

    // Delay for 1s
    Timer::after_millis(10).await;

    // Create a display buffer
    let mut display_fb1 = display_target::DisplayBuffer {
        buf: &mut display_buffer_1.as_mut_slice(),
        width: LCD_X_SIZE as i32,
        height: LCD_Y_SIZE as i32,
    };

    let mut display_fb2 = display_target::DisplayBuffer {
        buf: &mut display_buffer_2.as_mut_slice(),
        width: LCD_X_SIZE as i32,
        height: LCD_Y_SIZE as i32,
    };

    let display_fb = [&mut display_fb1, &mut display_fb2];

    let theme = medsize_sakura_rgb888_style();

    for active_buffer in 0..2 {
        let mut ui = Ui::new_fullscreen(display_fb[active_buffer], theme);
        ui.clear_background().unwrap();
    }

    let mut smp = [
        SmartstateProvider::<10>::new(),
        SmartstateProvider::<10>::new(),
    ];

    let mut i: i32 = 0;
    let mut checked = true;
    let mut active_buffer = 0;

    // input handling variables
    let mut point_down = false;
    let mut last_down = false;
    let mut location = embedded_graphics::geometry::Point::new(0, 0);

    let mut refresh_target = embassy_time::Ticker::every(embassy_time::Duration::from_millis(20));
    loop {
        // Switch buffers
        if active_buffer == 0 {
            active_buffer = 1;
        } else {
            active_buffer = 0;
        }

        // create UI (needs to be done each frame)
        let mut ui = Ui::new_fullscreen(display_fb[active_buffer], theme);

        let (smp1, smp2) = smp.split_at_mut(1);
        let (cur_smp, other_smp) = if active_buffer == 0 {
            (&mut smp1[0], &mut smp2[0])
        } else {
            (&mut smp2[0], &mut smp1[0])
        };

        // handle input
        match (last_down, point_down, location) {
            (false, true, loc) => {
                trace!("Click at ({}, {})", loc.x, loc.y);
                ui.interact(Interaction::Click(loc));
            }
            (true, true, loc) => {
                trace!("Drag at ({}, {})", loc.x, loc.y);
                ui.interact(Interaction::Drag(loc));
            }
            (true, false, loc) => {
                trace!("Release at ({}, {})", loc.x, loc.y);
                ui.interact(Interaction::Release(loc));
            }
            (false, false, loc) => {
                ui.interact(Interaction::Hover(loc));
            }
        }

        last_down = point_down;

        // === ACTUAL UI CODE STARTS HERE ===
        cur_smp.restart_counter();

        ui.add(
            Label::new("Basic Example")
                .with_font(ascii::FONT_10X20)
                .smartstate(cur_smp.nxt()),
        );

        ui.add(Label::new("Basic Counter (7LOC)").smartstate(cur_smp.nxt()));

        if ui
            .add_horizontal(
                IconButton::new(size24px::navigation::ArrowDown).smartstate(cur_smp.nxt()),
            )
            .clicked()
        {
            debug!("Decrementing counter");
            i = i.saturating_sub(1);

            cur_smp.peek().force_redraw();
            other_smp.get(cur_smp.get_pos()).force_redraw();
        }
        ui.add_horizontal(
            Label::new(alloc::format!("Clicked {} times", i).as_ref()).smartstate(cur_smp.nxt()),
        );
        if ui
            .add_horizontal(
                IconButton::new(size24px::navigation::ArrowUp).smartstate(cur_smp.nxt()),
            )
            .clicked()
        {
            debug!("Incrementing counter");
            i = i.saturating_add(1);
            cur_smp.prev().force_redraw();
            other_smp.get(cur_smp.get_pos() - 2).force_redraw();
        }

        ui.new_row();

        if ui
            .add(Checkbox::new(&mut checked).smartstate(cur_smp.nxt()))
            .clicked()
        {
            info!(
                "Checkbox is now {}",
                if checked { "checked" } else { "unchecked" }
            );
            checked = !checked;
        }

        // one row offset
        ui.new_row();

        ui.add_horizontal(IconButton::new(size24px::navigation::ArrowUpCircle));
        ui.add(IconWidget::<size24px::actions::RefreshDouble>::new_from_type());

        // replace the buffer with the new one
        LTDC.layer(0)
            .cfbar()
            .write(|w| w.set_cfbadd(&display_fb[active_buffer].buf[0] as *const _ as u32));

        // Immediately refresh the display
        LTDC.srcr().modify(|w| w.set_imr(Imr::RELOAD));

        // Interactions
        if let Some(touch) = crate::TOUCH_SIGNAL.try_take() {
            let x = touch.x as i32;
            let y = touch.y as i32;
            match touch.event {
                ft5336::TouchEvent::PressDown => {
                    point_down = true;
                    location = embedded_graphics::geometry::Point::new(y, x);
                }
                ft5336::TouchEvent::Contact => {
                    location = embedded_graphics::geometry::Point::new(y, x);
                }
                ft5336::TouchEvent::LiftUp => {
                    point_down = false;
                }
                ft5336::TouchEvent::NoEvent => {}
            }
        }

        refresh_target.next().await;
    }
}
