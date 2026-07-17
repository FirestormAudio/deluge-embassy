//! USB **host**-side USB Audio Class (UAC2) capture driver.
//!
//! Records multichannel audio from a class-compliant USB audio interface.
//! Mirrors [`super::midi`]: generic over the allocator so it unit-tests against
//! [`super::mock`]. See the design of record,
//! `docs/superpowers/specs/2026-07-16-usb-uac-host-design.md`.
//!
//! Playback (iso OUT) is Phase 2 (`out.rs`) and not present yet.

pub mod resample;
pub mod ring;

/// Widest capture channel count this driver supports. Bounds the fixed-size
/// per-frame scratch and the ring; a device declaring more is rejected.
pub const MAX_CHANNELS: usize = 8;
