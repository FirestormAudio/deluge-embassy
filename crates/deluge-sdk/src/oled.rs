//! The Deluge OLED display (128 × 48, 1-bit).

use deluge_bsp::oled::{self, FrameBuffer};
use embedded_graphics_core::Pixel;

/// Run the panel init sequence (device: SSD1309 bring-up; sim/linux: no-op).
pub(crate) async fn init_panel() {
    crate::plat::oled_init_panel().await;
}
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::geometry::{OriginDimensions, Point, Size};
use embedded_graphics_core::pixelcolor::BinaryColor;

/// The Deluge OLED display.
///
/// Obtained once from [`Deluge::oled`](crate::Deluge::oled), which also brings up
/// the PIC service the display's chip-select handshake rides on. Drawing happens
/// into an in-memory frame buffer; call [`flush`](Oled::flush) to push it to the
/// panel.
///
/// `Oled` is an `embedded-graphics` [`DrawTarget`] over [`BinaryColor`], so the
/// whole `embedded-graphics` ecosystem (text, fonts, shapes) draws straight onto
/// it; [`clear`](Oled::clear) / [`text`](Oled::text) cover the common cases
/// without pulling that in.
///
/// # Visible area
///
/// The draw surface is the full `128 × 48` panel, but the **top
/// [`VISIBLE_TOP`](Oled::VISIBLE_TOP) rows are hidden behind the faceplate** — only
/// [`VISIBLE_HEIGHT`](Oled::VISIBLE_HEIGHT) rows (`43`) are actually visible. Offset
/// content down by `VISIBLE_TOP` to keep it on-screen (the menu toolkit's
/// `MenuStyle::top_inset` does this).
pub struct Oled {
    fb: FrameBuffer,
    _not_send: crate::NotSend,
}

impl Oled {
    /// Rows hidden behind the faceplate at the top of the panel; offset drawing
    /// down by this much to keep it visible. See [`deluge_bsp::oled::VISIBLE_TOP`].
    pub const VISIBLE_TOP: usize = oled::VISIBLE_TOP;
    /// Visible pixel rows (`48` panel − [`VISIBLE_TOP`](Oled::VISIBLE_TOP) = `43`).
    pub const VISIBLE_HEIGHT: usize = oled::VISIBLE_HEIGHT;

    /// Internal; apps obtain the display via [`Deluge::oled`](crate::Deluge::oled).
    pub(crate) fn new() -> Self {
        Self {
            fb: FrameBuffer::new(),
            _not_send: crate::NOT_SEND,
        }
    }

    /// Clear the off-screen buffer (all pixels off). Takes effect on the next
    /// [`flush`](Oled::flush).
    #[inline]
    pub fn clear(&mut self) {
        self.fb.fill(0x00);
    }

    /// Draw an ASCII string at pixel (`x`, `y`) using the built-in 5×7 font.
    ///
    /// For richer text/graphics, draw onto `self` with `embedded-graphics`
    /// instead.
    #[inline]
    pub fn text(&mut self, x: usize, y: usize, s: &str) {
        oled::text::draw_str(&mut self.fb, x, y, s.as_bytes());
    }

    /// Push the current buffer to the panel.
    ///
    /// Acquires the shared RSPI0 bus (waiting out any concurrent CV write) and
    /// streams the frame over DMA — see `deluge_bsp::bus`. On the host simulator
    /// it copies the frame into the shared panel for the GUI to render.
    #[inline]
    pub async fn flush(&self) {
        crate::plat::oled_flush(&self.fb).await;
    }

    /// Direct access to the underlying frame buffer (raw pixel ops).
    #[inline]
    pub fn frame(&mut self) -> &mut FrameBuffer {
        &mut self.fb
    }
}

// ── embedded-graphics integration ──────────────────────────────────────────────

impl OriginDimensions for Oled {
    #[inline]
    fn size(&self) -> Size {
        Size::new(oled::WIDTH as u32, oled::HEIGHT as u32)
    }
}

impl DrawTarget for Oled {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(Point { x, y }, color) in pixels {
            if x >= 0 && y >= 0 {
                // `set_pixel` bounds-checks the upper edge.
                self.fb.set_pixel(x as usize, y as usize, color.is_on());
            }
        }
        Ok(())
    }
}
