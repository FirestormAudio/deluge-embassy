//! Windowed clip-list widget: a vertical window of clip rows (the Deluge "rows"
//! layout). Each visible row is a [`ClipRowData`] drawn with a playhead; the
//! window scrolls over a larger list. Engine-free — emits per-row
//! [`ClipListEvent`]s. Ported from spark-grid's `ClipListComponent`.

use crate::imode::{Frame, MAX_PAD_EVENTS};
use crate::widgets::clip_row::{ClipRowData, draw_clip_row};
use heapless::Vec;

pub use crate::widgets::clip_grid::SHORT_PRESS_MS;

const PULSE_PERIOD_MS: u32 = 1000;

/// Per-row interaction events (the host maps these to launch/open/select).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClipListEvent {
    Tap { row: usize },
    Hold { row: usize },
}

#[derive(Clone, Copy, Debug)]
struct RowPress {
    row: usize,
    at_ms: u32,
    hold_fired: bool,
}

/// Caller-owned interaction state for [`clip_list`].
#[derive(Clone, Debug, Default)]
pub struct ClipListState {
    /// First visible row index.
    pub scroll_offset: usize,
    /// The currently selected row, if any.
    pub selected: Option<usize>,
    press: Option<RowPress>,
}

impl ClipListState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the scroll offset, clamped so the `win_rows`-tall window stays within
    /// `num_rows`.
    pub fn scroll_to(&mut self, offset: usize, num_rows: usize, win_rows: usize) {
        self.scroll_offset = offset.min(num_rows.saturating_sub(win_rows));
    }

    /// Scroll by a signed delta, clamped to `[0, num_rows - win_rows]`.
    pub fn scroll_by(&mut self, d: isize, num_rows: usize, win_rows: usize) {
        let n = (self.scroll_offset as isize + d).max(0) as usize;
        self.scroll_to(n, num_rows, win_rows);
    }
}

fn pulse_phase(now_ms: u32) -> f32 {
    (now_ms % PULSE_PERIOD_MS) as f32 / PULSE_PERIOD_MS as f32
}

/// Render the visible window and drive interaction for one frame. `row(index)`
/// supplies each visible row's state.
pub fn clip_list<F>(
    f: &mut Frame,
    state: &mut ClipListState,
    num_rows: usize,
    row: F,
) -> Vec<ClipListEvent, MAX_PAD_EVENTS>
where
    F: Fn(usize) -> ClipRowData,
{
    let mut events = Vec::new();
    let (rows, _cols) = f.size();
    let now = f.now_ms();
    let scroll = state.scroll_offset;

    // ── input / FSM (per-row tap/hold; no two-finger gesture) ────────────────
    for ev in f.events() {
        let logical = scroll + ev.pad.row;
        if logical >= num_rows {
            continue;
        }
        if ev.pressed {
            if state.press.is_none() {
                state.press = Some(RowPress {
                    row: logical,
                    at_ms: now,
                    hold_fired: false,
                });
                state.selected = Some(logical);
            }
        } else if let Some(p) = state.press {
            if p.row == logical {
                if !p.hold_fired && now.saturating_sub(p.at_ms) < SHORT_PRESS_MS {
                    let _ = events.push(ClipListEvent::Tap { row: logical });
                }
                state.press = None;
            }
        }
    }

    if let Some(p) = state.press.as_mut() {
        if !p.hold_fired {
            let elapsed = now.saturating_sub(p.at_ms);
            if elapsed >= SHORT_PRESS_MS {
                let _ = events.push(ClipListEvent::Hold { row: p.row });
                p.hold_fired = true;
            } else {
                f.request_repaint_after(SHORT_PRESS_MS - elapsed);
            }
        }
    }

    // ── render visible window ────────────────────────────────────────────────
    let phase = pulse_phase(now);
    let mut any_animating = false;
    let end = (scroll + rows).min(num_rows);
    for logical in scroll..end {
        let display = logical - scroll;
        let data = row(logical);
        let selected = state.selected == Some(logical);
        any_animating |= data.animating(selected);
        draw_clip_row(f, display, &data, selected, phase);
    }

    if any_animating {
        f.request_repaint_after(33);
    }

    events
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Color;
    use crate::Pad;
    use crate::imode::{GridUi, PadInput};

    fn plain(_row: usize) -> ClipRowData {
        ClipRowData {
            color: Color::rgb(0, 0, 255),
            ..Default::default()
        }
    }

    #[test]
    fn windows_and_scrolls() {
        let mut state = ClipListState::new();
        let rowfn = |i: usize| ClipRowData {
            color: if i == 5 {
                Color::rgb(255, 0, 0)
            } else {
                Color::rgb(0, 0, 255)
            },
            ..Default::default()
        };
        let mut ui = GridUi::new();

        ui.run(0, PadInput::new(), |f| {
            clip_list(f, &mut state, 20, rowfn);
        });
        // row 5 (red, stopped → dim) lands on display row 5 with scroll 0
        let r5 = ui.grid().get_pad(5, 0);
        assert!(r5.r > 0 && r5.g == 0 && r5.b == 0);

        state.scroll_to(3, 20, 8); // row 5 → display row 2
        ui.request_repaint();
        ui.run(16, PadInput::new(), |f| {
            clip_list(f, &mut state, 20, rowfn);
        });
        let r2 = ui.grid().get_pad(2, 0);
        assert!(r2.r > 0 && r2.g == 0 && r2.b == 0);
    }

    #[test]
    fn scroll_clamps() {
        let mut state = ClipListState::new();
        state.scroll_to(999, 20, 8);
        assert_eq!(state.scroll_offset, 12); // 20 - 8
        state.scroll_to(5, 4, 8); // fewer rows than window
        assert_eq!(state.scroll_offset, 0);
    }

    #[test]
    fn tap_and_hold_per_row() {
        let mut state = ClipListState::new();
        let mut ui = GridUi::new();

        let mut input = PadInput::new();
        input.press(Pad::new(3, 5));
        ui.run(0, input, |f| clip_list(f, &mut state, 20, plain));
        assert_eq!(state.selected, Some(3));

        let mut input = PadInput::new();
        input.release(Pad::new(3, 5));
        let evs = ui
            .run(50, input, |f| clip_list(f, &mut state, 20, plain))
            .painted()
            .unwrap();
        assert_eq!(evs.as_slice(), &[ClipListEvent::Tap { row: 3 }]);

        // hold a different row
        let mut input = PadInput::new();
        input.press(Pad::new(1, 0));
        ui.run(100, input, |f| clip_list(f, &mut state, 20, plain));
        let evs = ui
            .run(100 + SHORT_PRESS_MS, PadInput::new(), |f| {
                clip_list(f, &mut state, 20, plain)
            })
            .painted()
            .unwrap();
        assert_eq!(evs.as_slice(), &[ClipListEvent::Hold { row: 1 }]);
    }
}
