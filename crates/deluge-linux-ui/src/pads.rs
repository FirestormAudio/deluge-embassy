//! `PadTarget`: the pad-grid analogue of [`crate::OledTarget`]. Packs a
//! `deluge_grid_toolkit::Grid` into the 432-byte Deluge pad RGB frame and
//! flushes it via `Deluge::pads_write`.
//!
//! The Linux pad fb is 18×8, 24bpp RGB, row-major, stride 54: byte
//! `row*54 + col*3` holds `[r, g, b]` for pad (row, col).

use deluge_grid_toolkit::imode::PadInput;
use deluge_grid_toolkit::{Grid, GRID_COLS, GRID_ROWS, Pad};
use deluge_hal_linux::{Deluge, Error, Event};

pub const STRIDE: usize = GRID_COLS * 3; // 54
pub const PAD_FRAME_BYTES: usize = STRIDE * GRID_ROWS; // 432

/// Event kind for pad events (mirrors `DELUGE_EV_PAD` in `include/deluge/input.h`).
pub const EV_PAD: u8 = 0;

pub struct PadTarget {
    buf: [u8; PAD_FRAME_BYTES],
}

impl PadTarget {
    pub fn new() -> Self {
        Self { buf: [0; PAD_FRAME_BYTES] }
    }

    pub fn frame(&self) -> &[u8; PAD_FRAME_BYTES] {
        &self.buf
    }

    pub fn clear_frame(&mut self) {
        self.buf = [0; PAD_FRAME_BYTES];
    }

    /// Pack a grid-toolkit `Grid` into the 432-byte RGB frame.
    pub fn blit(&mut self, grid: &Grid) {
        for row in 0..GRID_ROWS {
            for col in 0..GRID_COLS {
                let c = grid.get_pad(row, col);
                let o = row * STRIDE + col * 3;
                self.buf[o] = c.r;
                self.buf[o + 1] = c.g;
                self.buf[o + 2] = c.b;
            }
        }
    }

    /// Blit the current frame to the pad LEDs.
    pub fn flush(&mut self, dlg: &mut Deluge) -> Result<(), Error> {
        dlg.pads_write(&self.buf)
    }
}

impl Default for PadTarget {
    fn default() -> Self {
        Self::new()
    }
}

/// Convert a Deluge pad event into a `(Pad, pressed)` pair.
/// Returns `None` for non-pad events or out-of-grid coordinates.
pub fn pad_from_event(ev: &Event) -> Option<(Pad, bool)> {
    if ev.kind != EV_PAD {
        return None;
    }
    let (row, col) = (ev.y as usize, ev.x as usize);
    if row >= GRID_ROWS || col >= GRID_COLS {
        return None;
    }
    Some((Pad::new(row, col), ev.value != 0))
}

/// Build a single-event `PadInput` frame from a Deluge event (empty if the
/// event is not a pad event). The keyboard widget reads only `input.events`.
pub fn pad_input(ev: &Event) -> PadInput {
    let mut input = PadInput::new();
    if let Some((pad, pressed)) = pad_from_event(ev) {
        if pressed {
            input.press(pad);
        } else {
            input.release(pad);
        }
    }
    input
}

#[cfg(test)]
mod tests {
    use super::*;
    use deluge_grid_toolkit::Color;

    #[test]
    fn frame_is_432_zeroed_initially() {
        let t = PadTarget::new();
        assert_eq!(t.frame().len(), 432);
        assert!(t.frame().iter().all(|&b| b == 0));
    }

    #[test]
    fn blit_packs_rgb_at_stride_54() {
        let mut g = Grid::new();
        g.set_pad(0, 0, Color { r: 10, g: 20, b: 30 });
        g.set_pad(1, 2, Color { r: 1, g: 2, b: 3 });
        let mut t = PadTarget::new();
        t.blit(&g);
        assert_eq!(&t.frame()[0..3], &[10, 20, 30]);
        // row 1, col 2 -> offset 1*54 + 2*3 = 60
        assert_eq!(&t.frame()[60..63], &[1, 2, 3]);
    }

    fn ev(kind: u8, value: i16, x: u16, y: u16) -> Event {
        Event { kind, id: 0, value, x, y, pressure: 0 }
    }

    #[test]
    fn pad_event_maps_to_pad_and_pressed() {
        let (pad, pressed) = pad_from_event(&ev(EV_PAD, 1, 3, 2)).unwrap();
        assert_eq!(pad, Pad::new(2, 3));
        assert!(pressed);
    }

    #[test]
    fn release_has_pressed_false() {
        assert_eq!(pad_from_event(&ev(EV_PAD, 0, 1, 1)).unwrap().1, false);
    }

    #[test]
    fn non_pad_event_is_none() {
        assert!(pad_from_event(&ev(1, 1, 0, 0)).is_none());
    }

    #[test]
    fn out_of_grid_is_none() {
        assert!(pad_from_event(&ev(EV_PAD, 1, 99, 0)).is_none());
    }
}
