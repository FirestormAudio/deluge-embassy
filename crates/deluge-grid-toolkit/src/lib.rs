//! Reusable, resizable UI components for the Deluge 18 × 8 RGB pad grid.
//!
//! A portable toolkit extracted from the `spark` project: colour maths, a
//! [`Grid`] frame buffer, compositable [`GridLayer`]s, the resizable
//! [`Component`]/[`FlexibleComponent`] contract, transition [`animations`], and
//! a set of stateless render [`widgets`]. The plain [`Color`] type is shared with
//! the BSP; [`ColorExt`] layers the rich colour maths on top.
//!
//! Render a [`Grid`] and push it to the hardware with [`Grid::blit`] into a
//! [`deluge_bsp::rgb::PadLeds`].
//!
//! `no_std` by default; the optional `simd` feature enables NEON-accelerated
//! colour interpolation (nightly).

#![cfg_attr(not(test), no_std)]
#![cfg_attr(feature = "simd", feature(portable_simd))]
#![cfg_attr(
    all(feature = "simd", target_arch = "arm"),
    feature(stdarch_arm_neon_intrinsics)
)]
#![cfg_attr(
    all(feature = "simd", any(target_arch = "arm", target_arch = "aarch64")),
    feature(arm_target_feature)
)]

extern crate alloc;

pub mod animations;
pub mod color;
mod float_ext;
mod grid;
pub mod imode;
pub mod layer;
pub mod pad;
pub mod palette;
pub mod widgets;

pub use animations::{Animation, AnimationType, build_animation};
pub use color::ColorExt;
pub use deluge_bsp::rgb::Color;
pub use grid::{Grid, GridRgb};
pub use imode::{Frame, FrameOutput, GridUi, PadEvent, PadInput, PadMask, Rect, Response};
pub use layer::{BlendMode, GridCompositor, GridLayer};
pub use pad::{GRID_COLS, GRID_MAIN_COLS, GRID_ROWS, GRID_SIDE_COLS, Pad};
pub use widgets::{
    AutomationColor, BipolarAutomationColumn, CellPlaybackState, ClipCellComponent, ClipGridConfig,
    ClipGridDims, ClipGridEvent, ClipGridState, ClipListEvent, ClipListState, ClipRowData,
    KeyPress, KeyboardLayout, NoteVelocityColumn, SidebarRow, TextKeyboardComponent,
    UnipolarAutomationColumn, clip_grid, clip_list, draw_clip_row, draw_waveform, status_sidebar,
};
