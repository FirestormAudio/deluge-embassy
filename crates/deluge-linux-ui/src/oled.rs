//! `OledTarget`: an embedded-graphics `DrawTarget` over the Linux OLED
//! framebuffer (`deluge_hal_linux::Deluge::oled_write`).
//!
//! The Linux `deluge-oled` fb is linear/row-major: 43 rows × 16 bytes, 1bpp,
//! 128 px/row. Byte `y * 16 + x / 8`, bit `7 - (x % 8)` (MSB = leftmost pixel).
//! A set bit is a lit pixel (`BinaryColor::On`). No 5px offset — this is the
//! visible area already.
//!
//! Bit order caveat: MSB-first is a working assumption, not yet confirmed
//! against the kernel `deluge-oled` fb driver (its source was not available
//! in this checkout) or on real hardware. If on-device testing later shows
//! the panel is mirrored within 8px groups, flip `let bit = 7 - (x % 8);` to
//! `let bit = x % 8;` in `OledTarget::set` and update the tests below
//! accordingly. Tracked for on-device verification in a later task.

use deluge_hal_linux::{Deluge, Error};
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{OriginDimensions, Size},
    pixelcolor::BinaryColor,
    Pixel,
};

pub const WIDTH: usize = 128;
pub const HEIGHT: usize = 43;
pub const LINE_BYTES: usize = 16;
pub const FRAME_BYTES: usize = LINE_BYTES * HEIGHT; // 688

pub struct OledTarget {
    buf: [u8; FRAME_BYTES],
}

impl OledTarget {
    pub fn new() -> Self {
        Self {
            buf: [0; FRAME_BYTES],
        }
    }

    pub fn clear_frame(&mut self) {
        self.buf = [0; FRAME_BYTES];
    }

    pub fn frame(&self) -> &[u8; FRAME_BYTES] {
        &self.buf
    }

    /// Blit the current frame to the panel.
    pub fn flush(&mut self, dlg: &mut Deluge) -> Result<(), Error> {
        dlg.oled_write(&self.buf)
    }

    #[inline]
    fn set(&mut self, x: usize, y: usize, on: bool) {
        if x >= WIDTH || y >= HEIGHT {
            return;
        }
        let idx = y * LINE_BYTES + x / 8;
        let bit = 7 - (x % 8);
        if on {
            self.buf[idx] |= 1 << bit;
        } else {
            self.buf[idx] &= !(1 << bit);
        }
    }
}

impl Default for OledTarget {
    fn default() -> Self {
        Self::new()
    }
}

impl OriginDimensions for OledTarget {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl DrawTarget for OledTarget {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels {
            if coord.x < 0 || coord.y < 0 {
                continue;
            }
            self.set(coord.x as usize, coord.y as usize, color.is_on());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_graphics::{
        pixelcolor::BinaryColor,
        prelude::*,
        primitives::{PrimitiveStyle, Rectangle},
    };

    #[test]
    fn frame_is_688_zeroed_bytes_initially() {
        let t = OledTarget::new();
        assert_eq!(t.frame().len(), 688);
        assert!(t.frame().iter().all(|&b| b == 0));
    }

    #[test]
    fn top_left_pixel_sets_msb_of_byte_0() {
        let mut t = OledTarget::new();
        Pixel(Point::new(0, 0), BinaryColor::On)
            .draw(&mut t)
            .unwrap();
        assert_eq!(t.frame()[0], 0b1000_0000, "x=0 is the MSB of byte 0");
    }

    #[test]
    fn pixel_at_x8_y1_sets_msb_of_row1_byte0() {
        let mut t = OledTarget::new();
        // y=1 starts at byte 16 (LINE_BYTES); x=8 is byte offset 1 within the row.
        Pixel(Point::new(8, 1), BinaryColor::On)
            .draw(&mut t)
            .unwrap();
        assert_eq!(t.frame()[16 + 1], 0b1000_0000);
    }

    #[test]
    fn off_clears_a_set_pixel() {
        let mut t = OledTarget::new();
        Pixel(Point::new(3, 0), BinaryColor::On)
            .draw(&mut t)
            .unwrap();
        assert_eq!(t.frame()[0], 0b0001_0000);
        Pixel(Point::new(3, 0), BinaryColor::Off)
            .draw(&mut t)
            .unwrap();
        assert_eq!(t.frame()[0], 0);
    }

    #[test]
    fn out_of_bounds_pixels_are_ignored() {
        let mut t = OledTarget::new();
        Pixel(Point::new(-1, 0), BinaryColor::On)
            .draw(&mut t)
            .unwrap();
        Pixel(Point::new(128, 0), BinaryColor::On)
            .draw(&mut t)
            .unwrap();
        Pixel(Point::new(0, 43), BinaryColor::On)
            .draw(&mut t)
            .unwrap();
        assert!(t.frame().iter().all(|&b| b == 0));
    }

    #[test]
    fn fill_rect_full_row_sets_all_line_bytes() {
        let mut t = OledTarget::new();
        Rectangle::new(Point::new(0, 0), Size::new(128, 1))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(&mut t)
            .unwrap();
        assert!(t.frame()[0..16].iter().all(|&b| b == 0xFF));
        assert!(t.frame()[16..].iter().all(|&b| b == 0));
    }

    #[test]
    fn clear_frame_zeroes_everything() {
        let mut t = OledTarget::new();
        Pixel(Point::new(0, 0), BinaryColor::On)
            .draw(&mut t)
            .unwrap();
        t.clear_frame();
        assert!(t.frame().iter().all(|&b| b == 0));
    }
}
