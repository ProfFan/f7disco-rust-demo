pub struct DisplayBuffer<'a> {
    pub buf: &'a mut [u32],
    pub width: i32,
    pub height: i32,
}

use embedded_graphics::{
    Pixel, geometry,
    pixelcolor::{IntoStorage, Rgb888},
};

// Implement DrawTarget for
impl embedded_graphics::draw_target::DrawTarget for DisplayBuffer<'_> {
    type Color = Rgb888;
    type Error = ();

    /// Draw a pixel
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for pixel in pixels {
            let Pixel(point, color) = pixel;
            let argb = color.into_storage() | 0xFF00_0000u32;

            if point.x >= 0 && point.y >= 0 && point.x < self.width && point.y < self.height {
                let index = point.y * self.width + point.x;
                self.buf[index as usize] = argb;
            } else {
                // Ignore invalid points
            }
        }

        Ok(())
    }
}
impl geometry::OriginDimensions for DisplayBuffer<'_> {
    /// Return the size of the display
    fn size(&self) -> geometry::Size {
        geometry::Size::new(self.width as u32, self.height as u32)
    }
}

impl DisplayBuffer<'_> {
    /// Clears the buffer
    pub fn clear(&mut self) {
        let pixels = self.width * self.height;

        for a in self.buf[..pixels as usize].iter_mut() {
            *a = 0xFF00_0000u32; // Solid black
        }
    }
}
