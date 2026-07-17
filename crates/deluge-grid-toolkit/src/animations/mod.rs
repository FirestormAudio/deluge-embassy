//! Grid transition animations.
//!
//! Ported from Deluge C++ firmware (`pad_leds.cpp`) to provide smooth visual
//! transitions between grid views. Each animation interpolates from one [`Grid`]
//! to another, yielding frames via [`Animation::tick`].
//!
//! Driving an animation (timing, task spawning) is left to the host — call
//! [`build_animation`] and pump [`Animation::tick`] from your own loop.

pub mod expand_collapse;
pub mod explode;
pub mod fade;
pub mod scroll;
pub mod smear_scroll;
pub mod zoom;

use crate::Grid;
use alloc::boxed::Box;

pub use expand_collapse::Direction as ExpandCollapseDirection;
pub use explode::ExplodeDirection;
pub use scroll::ScrollDirection;
pub use smear_scroll::{HorizontalSmearScrollAnimation, VerticalSmearScrollAnimation};

/// Animation type variants.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AnimationType {
    /// Simple cross-fade between two grids.
    Fade { duration_ms: u32 },
    /// Horizontal scroll (left or right) — discrete pixel steps.
    HorizontalScroll {
        direction: ScrollDirection,
        scroll_to_black: bool,
    },
    /// Vertical scroll (up or down) — discrete pixel steps.
    VerticalScroll {
        direction: ScrollDirection,
        scroll_to_black: bool,
    },
    /// Smooth horizontal scroll with sub-pixel interpolation.
    HorizontalSmearScroll {
        direction: ScrollDirection,
        scroll_to_black: bool,
    },
    /// Smooth vertical scroll with sub-pixel interpolation.
    VerticalSmearScroll {
        direction: ScrollDirection,
        scroll_to_black: bool,
    },
    /// Explode from a point or implode to a point.
    Explode {
        origin: (f32, f32),
        direction: ExplodeDirection,
    },
    /// Zoom in or out with cross-fade.
    Zoom { magnitude: i8, zoom_in: bool },
    /// Expand/collapse for session ↔ clip transitions.
    ExpandCollapse { expand: bool },
}

/// Core animation trait.
///
/// Animations process frames and return [`Grid`] updates.
pub trait Animation: Send {
    /// Update animation state and render the next frame.
    ///
    /// Returns `Some(Grid)` with the next frame, or `None` when complete.
    /// `delta_ms` is the time since the last tick.
    fn tick(&mut self, delta_ms: f32) -> Option<Grid>;

    /// The total duration of this animation in milliseconds.
    fn duration_ms(&self) -> f32;

    /// Whether the animation is complete.
    fn is_complete(&self) -> bool;
}

/// Build a boxed [`Animation`] from an [`AnimationType`] and from/to grid states.
pub fn build_animation(from: Grid, to: Grid, anim_type: AnimationType) -> Box<dyn Animation> {
    match anim_type {
        AnimationType::Fade { duration_ms } => {
            Box::new(fade::FadeAnimation::new(from, to, duration_ms))
        }
        AnimationType::HorizontalSmearScroll {
            direction,
            scroll_to_black,
        } => Box::new(smear_scroll::HorizontalSmearScrollAnimation::new(
            from,
            to,
            direction,
            scroll_to_black,
            300,
        )),
        AnimationType::VerticalSmearScroll {
            direction,
            scroll_to_black,
        } => Box::new(smear_scroll::VerticalSmearScrollAnimation::new(
            from,
            to,
            direction,
            scroll_to_black,
            300,
        )),
        AnimationType::Explode { origin, direction } => Box::new(explode::ExplodeAnimation::new(
            from, to, direction, origin.0, origin.1, 300,
        )),
        AnimationType::HorizontalScroll {
            direction,
            scroll_to_black,
        } => Box::new(scroll::HorizontalScrollAnimation::new(
            from,
            to,
            direction,
            scroll_to_black,
            300,
        )),
        AnimationType::VerticalScroll {
            direction,
            scroll_to_black,
        } => Box::new(scroll::VerticalScrollAnimation::new(
            from,
            to,
            direction,
            scroll_to_black,
            300,
        )),
        AnimationType::Zoom { magnitude, zoom_in } => Box::new(zoom::ZoomAnimation::new(
            from, to, zoom_in, magnitude, 9.0, 300,
        )),
        AnimationType::ExpandCollapse { expand } => {
            let direction = if expand {
                expand_collapse::Direction::Expand
            } else {
                expand_collapse::Direction::Collapse
            };
            Box::new(expand_collapse::ExpandCollapseAnimation::new(
                from, to, direction, 0, 7, 300,
            ))
        }
    }
}
