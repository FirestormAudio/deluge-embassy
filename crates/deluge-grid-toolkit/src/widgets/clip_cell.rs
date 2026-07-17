//! Clip cell widget.
//!
//! Renders a single clip as a cell in a session grid view — transforming clip
//! visual state into a colour on a single grid pad.

use crate::Pad;
use crate::color::ColorExt as _;
use crate::imode::{Frame, Response};
use deluge_bsp::rgb::Color as RGB;
use uuid::Uuid;

/// Playback state for a clip cell (from transport/engine, not the project).
#[derive(Debug, Clone, Copy, Default)]
pub struct CellPlaybackState {
    /// Whether the clip is currently playing.
    pub playing: bool,
    /// Whether the clip is armed for launch (starts on the next quantize point).
    pub armed_for_launch: bool,
}

/// Visual state for a clip cell.
#[derive(Debug, Clone, Copy)]
pub struct ClipCellComponent {
    /// Clip ID for reference back to the project (`None` if empty slot).
    pub clip_id: Option<Uuid>,
    /// Colour (e.g. from the track's properties).
    pub color: RGB,
    /// Whether the clip is muted.
    pub muted: bool,
    /// Whether the clip is soloed.
    pub soloed: bool,
    /// Whether the clip/track is armed for recording.
    pub armed_for_recording: bool,
    /// Playback state (from transport).
    pub playback: CellPlaybackState,
    /// Whether this cell is selected.
    pub selected: bool,
    /// Pulse animation progress (0.0–1.0) for selection/armed highlight.
    pub pulse_phase: f32,
    /// Whether solo mode is globally active (dims non-soloed clips).
    pub global_solo_active: bool,
}

impl Default for ClipCellComponent {
    fn default() -> Self {
        Self::empty()
    }
}

impl ClipCellComponent {
    /// Create an empty cell (no clip).
    pub fn empty() -> Self {
        Self {
            clip_id: None,
            color: RGB::new(0, 0, 0),
            muted: false,
            soloed: false,
            armed_for_recording: false,
            playback: CellPlaybackState::default(),
            selected: false,
            pulse_phase: 0.0,
            global_solo_active: false,
        }
    }

    /// Create a new clip cell.
    pub fn new(clip_id: Uuid, color: RGB) -> Self {
        Self {
            clip_id: Some(clip_id),
            color,
            ..Self::empty()
        }
    }

    /// Create from a Deluge hue value (0–191).
    pub fn from_hue(clip_id: Uuid, hue: u8) -> Self {
        Self::new(clip_id, RGB::from_hue(hue as i32))
    }

    /// Set playback state.
    pub fn with_playback(mut self, playback: CellPlaybackState) -> Self {
        self.playback = playback;
        self
    }

    /// Set muted state.
    pub fn with_muted(mut self, muted: bool) -> Self {
        self.muted = muted;
        self
    }

    /// Set soloed state.
    pub fn with_soloed(mut self, soloed: bool) -> Self {
        self.soloed = soloed;
        self
    }

    /// Set armed-for-recording state.
    pub fn with_armed(mut self, armed: bool) -> Self {
        self.armed_for_recording = armed;
        self
    }

    /// Set selected state.
    pub fn with_selected(mut self, selected: bool) -> Self {
        self.selected = selected;
        self
    }

    /// Set the pulse animation phase (0.0–1.0).
    pub fn with_pulse_phase(mut self, phase: f32) -> Self {
        self.pulse_phase = phase.clamp(0.0, 1.0);
        self
    }

    /// Set global-solo-active state.
    pub fn with_global_solo(mut self, active: bool) -> Self {
        self.global_solo_active = active;
        self
    }

    /// Whether this is an empty slot.
    pub fn is_empty(&self) -> bool {
        self.clip_id.is_none()
    }

    /// The rendered colour for this cell.
    pub fn get_color(&self) -> RGB {
        if self.is_empty() {
            return RGB::new(0, 0, 0);
        }

        let base_color = self.color;

        let color = if self.selected {
            let bright = base_color.brighten(0.5);
            base_color.blend(bright, self.pulse_phase)
        } else {
            base_color
        };

        if self.armed_for_recording {
            return if ((self.pulse_phase * 4.0) as u32).is_multiple_of(2) {
                RGB::new(255, 0, 64)
            } else {
                color.dim_float(0.5)
            };
        }

        if self.playback.armed_for_launch {
            return if ((self.pulse_phase * 4.0) as u32).is_multiple_of(2) {
                color
            } else {
                RGB::new(0, 0, 0)
            };
        }

        if self.soloed {
            return color;
        }

        if self.playback.playing {
            return color;
        }

        if self.global_solo_active && !self.soloed {
            return color.dim_float(0.1);
        }

        if self.muted {
            return color.dim_float(0.1);
        }

        color.dim_float(0.25)
    }

    /// Whether the cell is animating (pulsing). The caller should keep the
    /// repaint gate open (e.g. `f.request_repaint_after(33)`) while true. An
    /// empty slot never animates — it renders black regardless of selection.
    pub fn animating(&self) -> bool {
        !self.is_empty()
            && (self.selected || self.playback.armed_for_launch || self.armed_for_recording)
    }

    /// Paint the cell on `pad` and report interaction in one pass.
    pub fn show(&self, f: &mut Frame, pad: Pad) -> Response {
        f.paint(pad, self.get_color());
        f.interact(pad)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_cell() {
        let cell = ClipCellComponent::empty();
        assert!(cell.is_empty());
        assert_eq!(cell.get_color(), RGB::new(0, 0, 0));
    }

    #[test]
    fn test_create_cell() {
        let cell = ClipCellComponent::new(Uuid::from_u128(1), RGB::new(255, 0, 0));
        assert!(!cell.is_empty());
        assert_eq!(cell.get_color().r, (255.0 * 0.25) as u8);
    }

    #[test]
    fn test_playing_cell() {
        let cell = ClipCellComponent::new(Uuid::from_u128(1), RGB::new(255, 0, 0)).with_playback(
            CellPlaybackState {
                playing: true,
                armed_for_launch: false,
            },
        );
        assert_eq!(cell.get_color().r, 255);
    }

    #[test]
    fn test_muted_cell() {
        let cell =
            ClipCellComponent::new(Uuid::from_u128(1), RGB::new(255, 255, 255)).with_muted(true);
        let expected = (255.0 * 0.1) as u8;
        let color = cell.get_color();
        assert_eq!(color.r, expected);
        assert_eq!(color.g, expected);
        assert_eq!(color.b, expected);
    }

    #[test]
    fn test_builder_pattern() {
        let cell = ClipCellComponent::new(Uuid::from_u128(1), RGB::new(0, 0, 255))
            .with_muted(true)
            .with_selected(true)
            .with_pulse_phase(0.5);
        assert!(cell.muted);
        assert!(cell.selected);
        assert_eq!(cell.pulse_phase, 0.5);
    }
}
