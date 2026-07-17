//! Clip row widget: a single clip rendered as a horizontal bar with a moving
//! playhead, used by [`clip_list`](crate::widgets::clip_list). Ported from
//! spark-grid's `ClipRowComponent`.

use crate::color::ColorExt as _;
use crate::imode::Frame;
use crate::{Color, Pad};

/// Visual state of one clip row (intrinsic state only — selection and pulse
/// phase are applied by the caller / [`clip_list`](crate::widgets::clip_list)).
#[derive(Clone, Copy, Debug)]
pub struct ClipRowData {
    pub color: Color,
    pub muted: bool,
    pub soloed: bool,
    pub armed_for_recording: bool,
    pub playing: bool,
    pub armed_for_launch: bool,
    /// Playhead position as a fraction `0.0..=1.0`.
    pub position: f32,
}

impl Default for ClipRowData {
    fn default() -> Self {
        Self {
            color: Color::BLACK,
            muted: false,
            soloed: false,
            armed_for_recording: false,
            playing: false,
            armed_for_launch: false,
            position: 0.0,
        }
    }
}

impl ClipRowData {
    /// The status-indicator colour for a sidebar (solo/playing/armed/queued).
    pub fn status_color(&self) -> Color {
        if self.soloed {
            Color::rgb(0, 255, 0)
        } else if self.playing {
            Color::rgb(0, 255, 255)
        } else if self.armed_for_recording {
            Color::rgb(255, 0, 0)
        } else if self.armed_for_launch {
            Color::rgb(255, 128, 0)
        } else {
            Color::rgb(32, 32, 32)
        }
    }

    /// Whether this row animates (playhead motion / launch blink / selection).
    pub fn animating(&self, selected: bool) -> bool {
        self.playing || self.armed_for_launch || selected
    }
}

/// Paint a clip row at local `row` across the full frame width. `selected` and
/// `pulse_phase` drive the selection highlight and launch blink.
pub fn draw_clip_row(
    f: &mut Frame,
    row: usize,
    data: &ClipRowData,
    selected: bool,
    pulse_phase: f32,
) {
    let width = f.size().1;
    let base = if selected {
        data.color.blend(data.color.brighten(0.5), pulse_phase)
    } else {
        data.color
    };

    for col in 0..width {
        let color = if data.muted {
            base.dim_float(0.02)
        } else if data.armed_for_recording {
            Color::rgb(255, 0, 0)
        } else if data.armed_for_launch {
            if ((pulse_phase * 4.0) as u32).is_multiple_of(2) {
                base
            } else {
                base.dim_float(0.2)
            }
        } else if data.playing {
            let head = (data.position * width as f32) as usize;
            if col < head {
                base
            } else if col == head {
                Color::rgb(255, 255, 255)
            } else {
                base.dim_float(0.3)
            }
        } else {
            base.dim_float(0.1)
        };
        f.paint(Pad::new(row, col), color);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Grid;
    use crate::imode::{GridUi, PadInput};

    fn render(data: ClipRowData) -> Grid {
        let mut ui = GridUi::new();
        ui.run(0, PadInput::new(), |f| {
            draw_clip_row(f, 0, &data, false, 0.0)
        });
        ui.grid().clone()
    }

    #[test]
    fn playing_row_has_white_playhead() {
        let data = ClipRowData {
            color: Color::rgb(255, 0, 0),
            playing: true,
            position: 0.5,
            ..Default::default()
        };
        let grid = render(data);
        // width is the full 18-col grid here; head = 0.5 * 18 = 9
        assert_eq!(grid.get_pad(0, 9), Color::rgb(255, 255, 255));
        assert_ne!(grid.get_pad(0, 0), Color::rgb(255, 255, 255)); // behind = base
    }

    #[test]
    fn status_color_priority() {
        let mut d = ClipRowData {
            color: Color::rgb(128, 128, 128),
            ..Default::default()
        };
        assert_eq!(d.status_color(), Color::rgb(32, 32, 32));
        d.playing = true;
        assert_eq!(d.status_color(), Color::rgb(0, 255, 255));
        d.soloed = true;
        assert_eq!(d.status_color(), Color::rgb(0, 255, 0)); // solo wins
    }
}
