//! Immediate-mode UI for the pad grid, with a built-in repaint gate.
//!
//! The UI is a pure function of state, evaluated each frame: one pass both paints
//! the [`Grid`] and reports interactions, so the pad↔meaning mapping is computed
//! once (not split across a render pass and an input pass).
//!
//! **The repaint gate is structural.** [`GridUi::run`] only invokes the UI
//! closure when *something changed* — the first frame, pad input, an in-pass
//! [`Frame::request_repaint`] (continuous animation), or an elapsed
//! [`Frame::request_repaint_after`] / [`GridUi::request_repaint_at`] (timed
//! animation). A clean frame is skipped entirely, so it costs ~nothing.
//!
//! Non-pad input (encoders, engine events, transport ticks) is *not* grid input:
//! the app mutates its own state and calls [`GridUi::request_repaint`].
//!
//! Layout is just sub-rectangles of the fixed grid — see [`Frame::region`].
//!
//! ```
//! use deluge_grid_toolkit::imode::{GridUi, PadInput};
//! use deluge_grid_toolkit::Pad;
//!
//! let mut ui = GridUi::new();
//! // First frame always paints; a following clean frame is skipped.
//! assert!(ui.run(0, PadInput::new(), |_f| {}).was_painted());
//! assert!(!ui.run(16, PadInput::new(), |_f| {}).was_painted());
//!
//! // A pad press opens the gate and is delivered to the button under it.
//! let mut input = PadInput::new();
//! input.press(Pad::new(0, 0));
//! let clicked = ui
//!     .run(32, input, |f| f.button(Pad::new(0, 0), deluge_grid_toolkit::Color::RED).clicked())
//!     .painted()
//!     .unwrap();
//! assert!(clicked);
//! ```

use crate::pad::{GRID_COLS, GRID_ROWS};
use crate::{Color, Grid, Pad};
use heapless::Vec;

/// Maximum pad events carried in a single [`PadInput`] frame.
pub const MAX_PAD_EVENTS: usize = 32;

// ── input ────────────────────────────────────────────────────────────────────

/// An edge-triggered pad event for this frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PadEvent {
    pub pad: Pad,
    pub pressed: bool,
}

/// Bitset of currently-held pads (`bits[col] & (1 << row)`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PadMask {
    bits: [u8; GRID_COLS],
}

impl PadMask {
    pub const fn new() -> Self {
        Self {
            bits: [0; GRID_COLS],
        }
    }

    /// Mark `pad` as held or released.
    pub fn set(&mut self, pad: Pad, held: bool) {
        let bit = 1u8 << pad.row;
        if held {
            self.bits[pad.col] |= bit;
        } else {
            self.bits[pad.col] &= !bit;
        }
    }

    /// Whether `pad` is currently held.
    pub fn contains(&self, pad: Pad) -> bool {
        self.bits[pad.col] & (1u8 << pad.row) != 0
    }

    /// Whether any pad is held.
    pub fn any(&self) -> bool {
        self.bits.iter().any(|&b| b != 0)
    }
}

/// Pad input gathered since the previous frame. Pads are the only input
/// intrinsic to the grid; everything else drives [`GridUi::request_repaint`].
#[derive(Clone, Debug, Default)]
pub struct PadInput {
    pub events: Vec<PadEvent, MAX_PAD_EVENTS>,
    pub held: PadMask,
}

impl PadInput {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a press (pushes an event and marks the pad held).
    pub fn press(&mut self, pad: Pad) {
        let _ = self.events.push(PadEvent { pad, pressed: true });
        self.held.set(pad, true);
    }

    /// Record a release (pushes an event and clears the held bit).
    pub fn release(&mut self, pad: Pad) {
        let _ = self.events.push(PadEvent {
            pad,
            pressed: false,
        });
        self.held.set(pad, false);
    }

    /// Whether nothing happened this frame (no press/release events). A steadily
    /// held pad is not new activity; widgets that animate while held should call
    /// [`Frame::request_repaint`].
    pub fn is_idle(&self) -> bool {
        self.events.is_empty()
    }
}

// ── geometry ─────────────────────────────────────────────────────────────────

/// An inclusive rectangle of pads. Also used as a 1×1 region via `From<Pad>`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rect {
    pub top: usize,
    pub left: usize,
    pub bottom: usize,
    pub right: usize,
}

impl Rect {
    /// An empty rect (`top > bottom`), produced by degenerate splits.
    pub const EMPTY: Rect = Rect {
        top: 1,
        left: 1,
        bottom: 0,
        right: 0,
    };

    pub const fn new(top: usize, left: usize, bottom: usize, right: usize) -> Self {
        Self {
            top,
            left,
            bottom,
            right,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.top > self.bottom || self.left > self.right
    }

    pub fn width(&self) -> usize {
        if self.left > self.right {
            0
        } else {
            self.right - self.left + 1
        }
    }

    pub fn height(&self) -> usize {
        if self.top > self.bottom {
            0
        } else {
            self.bottom - self.top + 1
        }
    }

    pub fn contains(&self, pad: Pad) -> bool {
        pad.row >= self.top
            && pad.row <= self.bottom
            && pad.col >= self.left
            && pad.col <= self.right
    }
}

impl From<Pad> for Rect {
    fn from(p: Pad) -> Self {
        Rect::new(p.row, p.col, p.row, p.col)
    }
}

// ── response ─────────────────────────────────────────────────────────────────

/// The result of interacting with a region this frame. Pads are **local** to the
/// frame/pane that produced the response.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Response {
    pub pressed: Option<Pad>,
    pub released: Option<Pad>,
    pub held: bool,
}

impl Response {
    /// A pad in the region went down this frame (press-to-trigger).
    pub fn clicked(&self) -> bool {
        self.pressed.is_some()
    }
    pub fn pressed(&self) -> bool {
        self.pressed.is_some()
    }
    pub fn released(&self) -> bool {
        self.released.is_some()
    }
    pub fn held(&self) -> bool {
        self.held
    }
}

// ── repaint signal ───────────────────────────────────────────────────────────

#[derive(Default)]
struct RepaintSignal {
    now: bool,
    at: Option<u32>,
}

impl RepaintSignal {
    fn request_now(&mut self) {
        self.now = true;
    }
    fn request_at(&mut self, t: u32) {
        self.at = Some(self.at.map_or(t, |e| e.min(t)));
    }
}

// ── frame ────────────────────────────────────────────────────────────────────

/// The per-frame UI context: a clipped, translated view of the [`Grid`] plus the
/// frame's pad input. `(clip.top, clip.left)` is this frame's local origin.
pub struct Frame<'a> {
    grid: &'a mut Grid,
    input: &'a PadInput,
    now_ms: u32,
    repaint: &'a mut RepaintSignal,
    clip: Rect,
}

impl<'a> Frame<'a> {
    /// This frame's pad input. Note `events` here are in **global** coordinates;
    /// inside a [`region`](Frame::region) use [`events`](Frame::events) for
    /// local, region-filtered events.
    pub fn input(&self) -> &PadInput {
        self.input
    }

    /// This frame's press/release events, translated into the frame's **local**
    /// coordinates and filtered to its region. Use this (not `input().events`)
    /// for multi-touch widgets, where `interact` only reports one pad and raw
    /// events are global.
    pub fn events(&self) -> impl Iterator<Item = PadEvent> + '_ {
        let clip = self.clip;
        self.input.events.iter().filter_map(move |ev| {
            clip.contains(ev.pad).then(|| PadEvent {
                pad: Pad {
                    row: ev.pad.row - clip.top,
                    col: ev.pad.col - clip.left,
                },
                pressed: ev.pressed,
            })
        })
    }

    /// The caller-supplied timestamp for this frame.
    pub fn now_ms(&self) -> u32 {
        self.now_ms
    }

    /// This frame's size in pads (the clip region).
    pub fn size(&self) -> (usize, usize) {
        (self.clip.height(), self.clip.width())
    }

    /// Escape hatch for advanced compositing (e.g. `GridLayer`). Coordinates are
    /// global; prefer [`paint`](Frame::paint) for clipped, local drawing.
    pub fn grid_mut(&mut self) -> &mut Grid {
        self.grid
    }

    fn to_global(&self, local: Rect) -> Rect {
        Rect {
            top: self.clip.top + local.top,
            left: self.clip.left + local.left,
            bottom: (self.clip.top + local.bottom).min(self.clip.bottom),
            right: (self.clip.left + local.right).min(self.clip.right),
        }
    }

    /// Paint a single pad (local coords; clipped to this frame).
    pub fn paint(&mut self, pad: Pad, color: Color) {
        let grow = self.clip.top + pad.row;
        let gcol = self.clip.left + pad.col;
        if grow <= self.clip.bottom && gcol <= self.clip.right {
            self.grid.set_pad(grow, gcol, color);
        }
    }

    /// Fill a region with a colour (local coords; clipped).
    pub fn fill(&mut self, area: impl Into<Rect>, color: Color) {
        let g = self.to_global(area.into());
        if g.is_empty() {
            return;
        }
        for row in g.top..=g.bottom {
            for col in g.left..=g.right {
                self.grid.set_pad(row, col, color);
            }
        }
    }

    /// Hit-test a region against this frame's input (does not paint).
    pub fn interact(&mut self, area: impl Into<Rect>) -> Response {
        let g = self.to_global(area.into());
        let mut r = Response::default();
        if g.is_empty() {
            return r;
        }
        for ev in self.input.events.iter() {
            if g.contains(ev.pad) {
                let local = Pad {
                    row: ev.pad.row - self.clip.top,
                    col: ev.pad.col - self.clip.left,
                };
                if ev.pressed {
                    r.pressed = Some(local);
                } else {
                    r.released = Some(local);
                }
            }
        }
        'held: for row in g.top..=g.bottom {
            for col in g.left..=g.right {
                if self.input.held.contains(Pad { row, col }) {
                    r.held = true;
                    break 'held;
                }
            }
        }
        r
    }

    /// Paint a pad and hit-test it in one call.
    pub fn button(&mut self, pad: Pad, color: Color) -> Response {
        self.paint(pad, color);
        self.interact(pad)
    }

    /// Run `body` against a child frame scoped (clipped + translated) to `rect`.
    /// The child's local `(0,0)` is `rect`'s top-left, and it only sees input
    /// inside `rect` — so panes route their own input for free.
    pub fn region<R>(&mut self, rect: impl Into<Rect>, body: impl FnOnce(&mut Frame) -> R) -> R {
        let clip = self.to_global(rect.into());
        let mut child = Frame {
            grid: &mut *self.grid,
            input: self.input,
            now_ms: self.now_ms,
            repaint: &mut *self.repaint,
            clip,
        };
        body(&mut child)
    }

    /// Split into `(top, bottom)` local rects at local row `at`.
    pub fn split_rows(&self, at: usize) -> (Rect, Rect) {
        let (h, w) = (self.clip.height(), self.clip.width());
        let at = at.min(h);
        let top = if at == 0 {
            Rect::EMPTY
        } else {
            Rect::new(0, 0, at - 1, w.saturating_sub(1))
        };
        let bottom = if at >= h {
            Rect::EMPTY
        } else {
            Rect::new(at, 0, h - 1, w.saturating_sub(1))
        };
        (top, bottom)
    }

    /// Split into `(left, right)` local rects at local column `at`.
    pub fn split_cols(&self, at: usize) -> (Rect, Rect) {
        let (h, w) = (self.clip.height(), self.clip.width());
        let at = at.min(w);
        let left = if at == 0 {
            Rect::EMPTY
        } else {
            Rect::new(0, 0, h.saturating_sub(1), at - 1)
        };
        let right = if at >= w {
            Rect::EMPTY
        } else {
            Rect::new(0, at, h.saturating_sub(1), w - 1)
        };
        (left, right)
    }

    /// Ask the gate to run again next frame (continuous animation).
    pub fn request_repaint(&mut self) {
        self.repaint.request_now();
    }

    /// Ask the gate to run again at/after `now_ms + delay_ms` (timed animation).
    pub fn request_repaint_after(&mut self, delay_ms: u32) {
        self.repaint
            .request_at(self.now_ms.saturating_add(delay_ms));
    }
}

// ── context + gate ───────────────────────────────────────────────────────────

/// The outcome of a [`GridUi::run`] call.
pub enum FrameOutput<R> {
    /// The gate was closed; the UI closure was not run and the grid is unchanged.
    Skipped,
    /// The UI closure ran and painted the grid; `R` is its return value.
    Painted(R),
}

impl<R> FrameOutput<R> {
    pub fn was_painted(&self) -> bool {
        matches!(self, FrameOutput::Painted(_))
    }
    pub fn painted(self) -> Option<R> {
        match self {
            FrameOutput::Painted(r) => Some(r),
            FrameOutput::Skipped => None,
        }
    }
}

/// The immediate-mode grid context. Owns the framebuffer and the repaint gate.
pub struct GridUi {
    grid: Grid,
    repaint: RepaintSignal,
    first: bool,
}

impl Default for GridUi {
    fn default() -> Self {
        Self::new()
    }
}

impl GridUi {
    pub fn new() -> Self {
        Self {
            grid: Grid::new(),
            repaint: RepaintSignal::default(),
            first: true,
        }
    }

    /// The last painted framebuffer. Blit it to the hardware after a `Painted`.
    pub fn grid(&self) -> &Grid {
        &self.grid
    }

    /// The next time the gate wants to run for a timed animation, if any — lets
    /// the driver sleep precisely instead of polling.
    pub fn next_repaint_at(&self) -> Option<u32> {
        self.repaint.at
    }

    /// External repaint request (e.g. an encoder turn or engine event changed
    /// app state): open the next gate.
    pub fn request_repaint(&mut self) {
        self.repaint.request_now();
    }

    /// External timed repaint request: open the gate at/after `at_ms`.
    pub fn request_repaint_at(&mut self, at_ms: u32) {
        self.repaint.request_at(at_ms);
    }

    /// Run one frame. If nothing changed (idle input, no pending/timed repaint,
    /// not the first frame), the gate is closed: `ui` is never called and
    /// [`FrameOutput::Skipped`] is returned. Otherwise the grid is redrawn from
    /// scratch by `ui` and [`FrameOutput::Painted`] is returned.
    pub fn run<R>(
        &mut self,
        now_ms: u32,
        input: PadInput,
        ui: impl FnOnce(&mut Frame) -> R,
    ) -> FrameOutput<R> {
        let timed_due = self.repaint.at.is_some_and(|t| now_ms >= t);
        let needs = self.first || self.repaint.now || timed_due || !input.is_idle();

        self.first = false;
        self.repaint.now = false; // one-shot; the ui pass may re-arm it
        if timed_due {
            self.repaint.at = None;
        }

        if !needs {
            return FrameOutput::Skipped;
        }

        self.grid.blank(); // immediate-mode: each frame starts fresh
        let clip = Rect::new(0, 0, GRID_ROWS - 1, GRID_COLS - 1);
        let mut frame = Frame {
            grid: &mut self.grid,
            input: &input,
            now_ms,
            repaint: &mut self.repaint,
            clip,
        };
        let value = ui(&mut frame);
        FrameOutput::Painted(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gate_skips_clean_frames() {
        let mut ui = GridUi::new();
        assert!(ui.run(0, PadInput::new(), |_| {}).was_painted()); // first frame
        assert!(!ui.run(16, PadInput::new(), |_| {}).was_painted()); // clean → skip
        assert!(!ui.run(32, PadInput::new(), |_| {}).was_painted());
    }

    #[test]
    fn pad_input_opens_gate_and_clicks() {
        let mut ui = GridUi::new();
        ui.run(0, PadInput::new(), |_| {}); // consume first-frame

        let mut input = PadInput::new();
        input.press(Pad::new(2, 3));
        let clicked = ui
            .run(16, input, |f| {
                f.button(Pad::new(2, 3), Color::RED).clicked()
            })
            .painted()
            .unwrap();
        assert!(clicked);
    }

    #[test]
    fn continuous_repaint_is_self_limiting() {
        let mut ui = GridUi::new();
        let mut input = PadInput::new();
        input.press(Pad::new(0, 0));
        // triggering frame requests another
        assert!(ui.run(0, input, |f| f.request_repaint()).was_painted());
        // next frame runs from the pending request, but does not re-request
        assert!(ui.run(16, PadInput::new(), |_| {}).was_painted());
        // with nothing pending, the following frame is skipped
        assert!(!ui.run(32, PadInput::new(), |_| {}).was_painted());
    }

    #[test]
    fn external_request_opens_gate() {
        let mut ui = GridUi::new();
        ui.run(0, PadInput::new(), |_| {});
        ui.request_repaint(); // e.g. an encoder turn between frames
        assert!(ui.run(16, PadInput::new(), |_| {}).was_painted());
        assert!(!ui.run(32, PadInput::new(), |_| {}).was_painted());
    }

    #[test]
    fn timed_repaint_waits_then_fires() {
        let mut ui = GridUi::new();
        let mut input = PadInput::new();
        input.press(Pad::new(0, 0));
        ui.run(0, input, |f| f.request_repaint_after(250));
        assert_eq!(ui.next_repaint_at(), Some(250));
        assert!(!ui.run(100, PadInput::new(), |_| {}).was_painted()); // before due
        assert!(ui.run(300, PadInput::new(), |_| {}).was_painted()); // due → fires
        assert_eq!(ui.next_repaint_at(), None);
    }

    #[test]
    fn region_routes_input_to_the_right_pane() {
        let mut ui = GridUi::new();
        ui.run(0, PadInput::new(), |_| {});

        // Press a pad in the bottom pane (global row 5).
        let mut input = PadInput::new();
        input.press(Pad::new(5, 2));

        let (top_hit, bottom_local) = ui
            .run(16, input, |f| {
                let (top, bottom) = f.split_rows(3); // rows 0..3 / 3..8
                let top_hit = f.region(top, |p| p.interact(Rect::new(0, 0, 2, 17)).clicked());
                // bottom pane: global row 5 → local row 2 (5 - 3)
                let bottom_local = f.region(bottom, |p| p.interact(Rect::new(0, 0, 4, 17)).pressed);
                (top_hit, bottom_local)
            })
            .painted()
            .unwrap();

        assert!(!top_hit, "press must not leak into the top pane");
        assert_eq!(
            bottom_local,
            Some(Pad::new(2, 2)),
            "press is local to the pane"
        );
    }

    #[test]
    fn region_events_are_local() {
        let mut ui = GridUi::new();
        ui.run(0, PadInput::new(), |_| {});

        let mut input = PadInput::new();
        input.press(Pad::new(5, 2)); // global

        let locals = ui
            .run(16, input, |f| {
                let (_top, bottom) = f.split_rows(3); // bottom origin = global row 3
                f.region(bottom, |p| {
                    let v: alloc::vec::Vec<_> = p.events().map(|e| e.pad).collect();
                    v
                })
            })
            .painted()
            .unwrap();
        assert_eq!(locals, alloc::vec![Pad::new(2, 2)]); // 5 - 3 = local row 2
    }

    #[test]
    fn region_clips_out_of_bounds_paint() {
        let mut ui = GridUi::new();
        let grid = ui
            .run(0, PadInput::new(), |f| {
                let (_top, bottom) = f.split_rows(7); // bottom = local row 0 == global row 7
                f.region(bottom, |p| {
                    p.paint(Pad::new(0, 0), Color::RED); // global (7,0): in bounds
                    p.paint(Pad::new(5, 0), Color::GREEN); // global (12,0): clipped away
                });
            })
            .was_painted();
        assert!(grid);
        assert_eq!(ui.grid().get_pad(7, 0), Color::RED);
        // nothing painted out of the pane:
        assert_eq!(ui.grid().get_pad(0, 0), Color::BLACK);
    }
}
