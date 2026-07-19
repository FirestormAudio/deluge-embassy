//! Bridge between `deluge-linux` (Linux hardware bindings) and
//! `deluge-ui-toolkit` (the shared OLED UI toolkit).
//!
//! - [`OledTarget`] is an `embedded-graphics` `DrawTarget` over the 688-byte
//!   Linux OLED framebuffer; render toolkit widgets into it and `flush`.
//! - [`controls`] holds the named front-panel control ids the Linux SDK reports.

pub mod controls;
pub mod oled;
pub use oled::{OledTarget, FRAME_BYTES, HEIGHT, WIDTH};

pub mod pads;
pub use pads::{PadTarget, PAD_FRAME_BYTES};
pub use deluge_grid_toolkit;
