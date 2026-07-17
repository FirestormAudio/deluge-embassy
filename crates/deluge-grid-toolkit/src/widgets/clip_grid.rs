//! Windowed clip-grid widget.
//!
//! A resizable pad window scrolled over a larger logical grid of
//! `sections × tracks` (up to 128 each), with selection and a two-finger clone
//! gesture. Engine-free: it emits mode-agnostic [`ClipGridEvent`]s that the host
//! maps to its own commands.
//!
//! The window size is the frame/region size, so place it with
//! `f.region(rect, |g| clip_grid(g, …))` and put a sidebar/list in sibling
//! regions. Ported from spark-grid's `ClipGridComponent` (windowing) and
//! `SessionView` (the clone/press FSM), decoupled from the engine.

use crate::Pad;
use crate::imode::{Frame, MAX_PAD_EVENTS};
use crate::widgets::ClipCellComponent;
use heapless::Vec;

/// Maximum sections / tracks (matches the Deluge session model).
pub const MAX_DIM: usize = 128;
/// A press shorter than this (ms) is a [`Tap`](ClipGridEvent::Tap); longer is a
/// [`Hold`](ClipGridEvent::Hold).
pub const SHORT_PRESS_MS: u32 = 500;

const PULSE_PERIOD_MS: u32 = 1000;

/// Logical dimensions of the clip grid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClipGridDims {
    pub sections: usize,
    pub tracks: usize,
}

/// Mode-agnostic interaction events. The host decides what each means for its
/// current mode (launch / open / create / clone).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClipGridEvent {
    /// A pad was just pressed (the press edge of a new primary gesture). Hosts
    /// that act immediately on touch — select, create, launch — handle this; the
    /// later [`Tap`](ClipGridEvent::Tap) is the short-release follow-up. A second
    /// finger that becomes a [`Clone`](ClipGridEvent::Clone) target does not emit
    /// `Press` (when the clone gesture is enabled).
    Press { section: usize, track: usize },
    /// A pad was pressed and released within [`SHORT_PRESS_MS`].
    Tap { section: usize, track: usize },
    /// A pad was held for at least [`SHORT_PRESS_MS`].
    Hold { section: usize, track: usize },
    /// Two-finger clone: a held source pad + a tapped target pad.
    Clone {
        from: (usize, usize),
        to: (usize, usize),
    },
}

/// Per-call behaviour for [`clip_grid`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClipGridConfig {
    /// Whether a second finger while one pad is held is a clone gesture
    /// (suppressing the primary's tap/hold and emitting [`ClipGridEvent::Clone`]
    /// on the target's release). When `false`, every press is independent: it
    /// emits its own [`ClipGridEvent::Press`] and becomes the held cell — suited
    /// to modes (e.g. macros) where multi-touch is many launches, not a clone.
    pub clone: bool,
}

impl Default for ClipGridConfig {
    fn default() -> Self {
        Self { clone: true }
    }
}

#[derive(Clone, Copy, Debug)]
struct Press {
    cell: (usize, usize),
    at_ms: u32,
    hold_fired: bool,
    consumed: bool, // a second finger arrived → suppress this pad's tap/hold
}

/// Caller-owned interaction state for [`clip_grid`] (scroll window + selection +
/// gesture FSM).
#[derive(Clone, Debug, Default)]
pub struct ClipGridState {
    /// First visible track (X window origin).
    pub scroll_x: usize,
    /// First visible section (Y window origin).
    pub scroll_y: usize,
    /// The currently selected `(section, track)`, if any.
    pub selected: Option<(usize, usize)>,
    press: Option<Press>,
    clone_target: Option<(usize, usize)>,
}

impl ClipGridState {
    pub fn new() -> Self {
        Self::default()
    }

    /// The currently-held primary cell `(section, track)`, if any. Tracks the
    /// active press across frames (through a hold, and while a clone target is
    /// being chosen) until release — letting hosts drive encoder-while-held /
    /// delete-held gestures without their own press bookkeeping.
    pub fn held(&self) -> Option<(usize, usize)> {
        self.press.map(|p| p.cell)
    }

    /// Set the scroll origin, clamped so the `win = (rows, cols)` window stays
    /// within `dims`.
    pub fn scroll_to(&mut self, x: usize, y: usize, dims: ClipGridDims, win: (usize, usize)) {
        self.scroll_x = x.min(dims.tracks.saturating_sub(win.1));
        self.scroll_y = y.min(dims.sections.saturating_sub(win.0));
    }

    /// Scroll by a signed delta, clamped to `[0, dims - win]`.
    pub fn scroll_by(&mut self, dx: isize, dy: isize, dims: ClipGridDims, win: (usize, usize)) {
        let nx = (self.scroll_x as isize + dx).max(0) as usize;
        let ny = (self.scroll_y as isize + dy).max(0) as usize;
        self.scroll_to(nx, ny, dims, win);
    }
}

fn pulse_phase(now_ms: u32) -> f32 {
    (now_ms % PULSE_PERIOD_MS) as f32 / PULSE_PERIOD_MS as f32
}

/// Render the visible window and drive interaction for one frame. Returns the
/// semantic events that occurred (typically 0–1).
///
/// `cell(section, track)` supplies the visual state for each visible logical
/// cell; the widget applies the current selection highlight before painting.
pub fn clip_grid<F>(
    f: &mut Frame,
    state: &mut ClipGridState,
    dims: ClipGridDims,
    config: ClipGridConfig,
    cell: F,
) -> Vec<ClipGridEvent, MAX_PAD_EVENTS>
where
    F: Fn(usize, usize) -> ClipCellComponent,
{
    let mut events = Vec::new();
    let (rows, cols) = f.size();
    let now = f.now_ms();
    let (sx, sy) = (state.scroll_x, state.scroll_y);

    let to_logical = |pad: Pad| -> Option<(usize, usize)> {
        let section = sy + pad.row;
        let track = sx + pad.col;
        (section < dims.sections && track < dims.tracks).then_some((section, track))
    };

    // ── input / gesture FSM ──────────────────────────────────────────────────
    for ev in f.events() {
        let Some(cell_pos) = to_logical(ev.pad) else {
            continue;
        };
        if ev.pressed {
            match state.press {
                // Second finger while one is held, clone enabled → clone target;
                // suppress the primary's tap/hold.
                Some(ref mut p) if config.clone => {
                    state.clone_target = Some(cell_pos);
                    p.consumed = true;
                }
                // Primary press, or (clone disabled) an independent press: start a
                // fresh press, select, and announce the press edge.
                _ => {
                    state.press = Some(Press {
                        cell: cell_pos,
                        at_ms: now,
                        hold_fired: false,
                        consumed: false,
                    });
                    state.selected = Some(cell_pos);
                    let _ = events.push(ClipGridEvent::Press {
                        section: cell_pos.0,
                        track: cell_pos.1,
                    });
                }
            }
        } else if config.clone && state.clone_target == Some(cell_pos) {
            if let Some(p) = state.press {
                let _ = events.push(ClipGridEvent::Clone {
                    from: p.cell,
                    to: cell_pos,
                });
            }
            state.clone_target = None;
        } else if let Some(p) = state.press {
            if p.cell == cell_pos {
                if !p.consumed && !p.hold_fired && now.saturating_sub(p.at_ms) < SHORT_PRESS_MS {
                    let _ = events.push(ClipGridEvent::Tap {
                        section: cell_pos.0,
                        track: cell_pos.1,
                    });
                }
                state.press = None;
            }
        }
    }

    // ── hold detection (time-based, no event) ────────────────────────────────
    if let Some(p) = state.press.as_mut() {
        if !p.hold_fired && !p.consumed {
            let elapsed = now.saturating_sub(p.at_ms);
            if elapsed >= SHORT_PRESS_MS {
                let _ = events.push(ClipGridEvent::Hold {
                    section: p.cell.0,
                    track: p.cell.1,
                });
                p.hold_fired = true;
            } else {
                // wake exactly at the threshold instead of busy-polling
                f.request_repaint_after(SHORT_PRESS_MS - elapsed);
            }
        }
    }

    // ── render the visible window ────────────────────────────────────────────
    let phase = pulse_phase(now);
    let mut any_animating = false;
    let sec_end = (sy + rows).min(dims.sections);
    let trk_end = (sx + cols).min(dims.tracks);
    for section in sy..sec_end {
        let row = section - sy;
        for track in sx..trk_end {
            let col = track - sx;
            let mut c = cell(section, track);
            if state.selected == Some((section, track)) {
                c = c.with_selected(true).with_pulse_phase(phase);
            }
            any_animating |= c.animating();
            f.paint(Pad::new(row, col), c.get_color());
        }
    }

    if any_animating {
        f.request_repaint_after(33); // ~30 Hz while pulsing
    }

    events
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Color;
    use crate::imode::{GridUi, PadInput};
    use uuid::Uuid;

    const DIMS: ClipGridDims = ClipGridDims {
        sections: 32,
        tracks: 32,
    };

    fn empty(_s: usize, _t: usize) -> ClipCellComponent {
        ClipCellComponent::empty()
    }

    #[test]
    fn windowing_maps_logical_to_display() {
        let target = (5usize, 9usize);
        let cf = move |s: usize, t: usize| {
            if (s, t) == target {
                ClipCellComponent::new(Uuid::from_u128(1), Color::RED)
            } else {
                ClipCellComponent::empty()
            }
        };

        let mut state = ClipGridState::new();
        let mut ui = GridUi::new();

        // scroll (0,0): target at (row 5, col 9)
        ui.run(0, PadInput::new(), |f| {
            clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), cf);
        });
        assert_ne!(ui.grid().get_pad(5, 9), Color::BLACK);
        assert_eq!(ui.grid().get_pad(0, 0), Color::BLACK);

        // scroll to track=2, section=4 → target now at (row 1, col 7)
        state.scroll_to(2, 4, DIMS, (8, 16));
        ui.request_repaint();
        ui.run(16, PadInput::new(), |f| {
            clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), cf);
        });
        assert_ne!(ui.grid().get_pad(1, 7), Color::BLACK);
        assert_eq!(ui.grid().get_pad(5, 9), Color::BLACK);
    }

    #[test]
    fn scroll_clamps_to_window() {
        let mut state = ClipGridState::new();
        // 32 tracks, 16-wide window → max scroll_x = 16; 32 sections, 8 tall → 24
        state.scroll_to(999, 999, DIMS, (8, 16));
        assert_eq!((state.scroll_x, state.scroll_y), (16, 24));
        // smaller-than-window logical grid → clamp to 0
        let tiny = ClipGridDims {
            sections: 4,
            tracks: 8,
        };
        state.scroll_to(5, 5, tiny, (8, 16));
        assert_eq!((state.scroll_x, state.scroll_y), (0, 0));
    }

    #[test]
    fn tap_on_press_then_quick_release() {
        let mut state = ClipGridState::new();
        let mut ui = GridUi::new();

        let mut input = PadInput::new();
        input.press(Pad::new(2, 3));
        let evs = ui
            .run(0, input, |f| {
                clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), empty)
            })
            .painted()
            .unwrap();
        // the press edge announces itself; the tap follows on release
        assert_eq!(
            evs.as_slice(),
            &[ClipGridEvent::Press {
                section: 2,
                track: 3
            }]
        );
        assert_eq!(state.held(), Some((2, 3)));

        let mut input = PadInput::new();
        input.release(Pad::new(2, 3));
        let evs = ui
            .run(100, input, |f| {
                clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), empty)
            })
            .painted()
            .unwrap();
        assert_eq!(
            evs.as_slice(),
            &[ClipGridEvent::Tap {
                section: 2,
                track: 3
            }]
        );
    }

    #[test]
    fn hold_fires_once_at_threshold() {
        let mut state = ClipGridState::new();
        let mut ui = GridUi::new();

        let mut input = PadInput::new();
        input.press(Pad::new(1, 1));
        ui.run(0, input, |f| {
            clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), empty)
        });
        // the press should have scheduled a wake at the hold threshold
        assert_eq!(ui.next_repaint_at(), Some(SHORT_PRESS_MS));

        let evs = ui
            .run(SHORT_PRESS_MS, PadInput::new(), |f| {
                clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), empty)
            })
            .painted()
            .unwrap();
        assert_eq!(
            evs.as_slice(),
            &[ClipGridEvent::Hold {
                section: 1,
                track: 1
            }]
        );

        // a later clean frame must not fire Hold again
        ui.request_repaint();
        let evs = ui
            .run(SHORT_PRESS_MS + 100, PadInput::new(), |f| {
                clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), empty)
            })
            .painted()
            .unwrap();
        assert!(evs.is_empty());
    }

    #[test]
    fn two_finger_clone() {
        let mut state = ClipGridState::new();
        let mut ui = GridUi::new();

        let mut i = PadInput::new();
        i.press(Pad::new(0, 0)); // source
        ui.run(0, i, |f| {
            clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), empty)
        });

        let mut i = PadInput::new();
        i.press(Pad::new(1, 2)); // target (second finger)
        i.held.set(Pad::new(0, 0), true);
        ui.run(10, i, |f| {
            clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), empty)
        });

        let mut i = PadInput::new();
        i.release(Pad::new(1, 2));
        i.held.set(Pad::new(0, 0), true);
        let evs = ui
            .run(20, i, |f| {
                clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), empty)
            })
            .painted()
            .unwrap();
        assert_eq!(
            evs.as_slice(),
            &[ClipGridEvent::Clone {
                from: (0, 0),
                to: (1, 2),
            }]
        );

        // releasing the still-held source afterwards must NOT emit a Tap
        let mut i = PadInput::new();
        i.release(Pad::new(0, 0));
        let evs = ui
            .run(30, i, |f| {
                clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), empty)
            })
            .painted()
            .unwrap();
        assert!(evs.is_empty());
    }

    #[test]
    fn press_sets_and_brightens_selection() {
        let cf = |_s: usize, _t: usize| ClipCellComponent::new(Uuid::from_u128(1), Color::RED);
        let mut state = ClipGridState::new();
        let mut ui = GridUi::new();

        let mut input = PadInput::new();
        input.press(Pad::new(0, 0));
        // now=250 → pulse phase 0.25 so the selection blend is visible
        ui.run(250, input, |f| {
            clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), cf)
        });

        assert_eq!(state.selected, Some((0, 0)));
        let selected = ui.grid().get_pad(0, 0); // selected RED cell
        let plain = ui.grid().get_pad(1, 1); // identical unselected RED cell
        assert!(selected.g > plain.g, "selected cell should be brighter");
    }

    #[test]
    fn held_tracks_press_and_clears_on_release() {
        let mut state = ClipGridState::new();
        let mut ui = GridUi::new();
        assert_eq!(state.held(), None);

        let mut input = PadInput::new();
        input.press(Pad::new(3, 4));
        ui.run(0, input, |f| {
            clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), empty)
        });
        assert_eq!(state.held(), Some((3, 4)));

        let mut input = PadInput::new();
        input.release(Pad::new(3, 4));
        ui.run(50, input, |f| {
            clip_grid(f, &mut state, DIMS, ClipGridConfig::default(), empty)
        });
        assert_eq!(state.held(), None);
    }

    #[test]
    fn clone_disabled_makes_each_press_independent() {
        let cfg = ClipGridConfig { clone: false };
        let mut state = ClipGridState::new();
        let mut ui = GridUi::new();

        // First finger lands.
        let mut i = PadInput::new();
        i.press(Pad::new(0, 0));
        let evs = ui
            .run(0, i, |f| clip_grid(f, &mut state, DIMS, cfg, empty))
            .painted()
            .unwrap();
        assert_eq!(
            evs.as_slice(),
            &[ClipGridEvent::Press {
                section: 0,
                track: 0
            }]
        );

        // Second finger while the first is held → its own Press (no Clone), and
        // it becomes the held cell.
        let mut i = PadInput::new();
        i.press(Pad::new(1, 2));
        i.held.set(Pad::new(0, 0), true);
        let evs = ui
            .run(10, i, |f| clip_grid(f, &mut state, DIMS, cfg, empty))
            .painted()
            .unwrap();
        assert_eq!(
            evs.as_slice(),
            &[ClipGridEvent::Press {
                section: 1,
                track: 2
            }]
        );
        assert_eq!(state.held(), Some((1, 2)));
    }
}
