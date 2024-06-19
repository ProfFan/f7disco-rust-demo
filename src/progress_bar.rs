use embedded_graphics::{
    draw_target::DrawTarget,
    pixelcolor::{Rgb888, RgbColor},
    prelude::{Point, Primitive, Size},
    primitives::{PrimitiveStyle, Rectangle},
    Drawable,
};
use embedded_layout::{
    align::{horizontal, vertical, Align},
    View,
};

pub struct ProgressBar {
    progress: u32,
    bounds: Rectangle,
}
impl ProgressBar {
    /// The progress bar has a configurable position and size
    pub fn new(position: Point, size: Size) -> Self {
        Self {
            bounds: Rectangle::new(position, size),
            progress: 0,
        }
    }

    pub fn with_progress(self, progress: u32) -> Self {
        Self {
            bounds: self.bounds,
            progress,
        }
    }
}

/// Implementing `View` is required by the layout and alignment operations
/// `View` teaches `embedded-layout` where our object is, how big it is and how to move it.
impl View for ProgressBar {
    #[inline]
    fn translate_impl(&mut self, by: Point) {
        // make sure you don't accidentally call `translate`!
        self.bounds.translate_mut(by);
    }

    #[inline]
    fn bounds(&self) -> Rectangle {
        self.bounds
    }
}

/// Need to implement `Drawable` for a _reference_ of our view
impl Drawable for ProgressBar {
    type Color = Rgb888;
    type Output = ();

    fn draw<D: DrawTarget<Color = Rgb888>>(&self, display: &mut D) -> Result<(), D::Error> {
        // Create styles
        let border_style = PrimitiveStyle::with_stroke(Rgb888::WHITE, 1);
        let progress_style = PrimitiveStyle::with_fill(Rgb888::WHITE);

        // Create a 1px border
        let border = self.bounds.into_styled(border_style);

        // Create a rectangle that will indicate progress
        let progress = Rectangle::new(
            Point::zero(),
            // sizes are calculated so that the rectangle will have a 1px margin
            Size::new(
                (self.bounds.size().width - 4) * self.progress / 100,
                self.bounds.size().height - 4,
            ),
        )
        .into_styled(progress_style);

        // Align progress bar within border
        let progress = progress
            .align_to(&border, horizontal::Left, vertical::Center)
            .translate(Point::new(2, 0));

        // Draw views
        border.draw(display)?;
        progress.draw(display)?;

        Ok(())
    }
}
