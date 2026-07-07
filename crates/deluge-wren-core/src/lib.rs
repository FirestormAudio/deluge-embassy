//! Portable Deluge Wren runtime.
//!
//! The foreign-binding glue, native DSP engine, and a [`Host`] trait that the
//! firmware (hardware) and the web simulator (wasm) each implement. This crate is
//! `no_std` and target-agnostic: it is the single source of truth for the Wren
//! scripting surface, so the on-device firmware and the browser tool can never
//! drift apart.
//!
//! ## Wiring a host
//! A target constructs one `'static` [`Host`], registers it once at boot with
//! [`set_host`], boots the VM with [`CLASSES`]/[`METHODS`] + [`prelude_ptr`], then
//! on its VM thread:
//! - calls [`tick`] each loop iteration (advances CV slew + fires metros),
//! - feeds input via [`midi_rx`], [`input_dispatch`], [`enc_turn`],
//! - drains [`Cmd`]s from its `Host::audio_cmd` into a `deluge_audio_graph::Engine`
//!   it renders.

#![no_std]

mod audio;
mod bindings;
mod bindings_audio;
mod host;
mod slotapi;
#[cfg(feature = "wren-sys-backend")]
mod slotapi_wrensys;
#[cfg(all(feature = "wren-sys-backend", any(test, feature = "test-support")))]
pub mod test_support;

pub use bindings::{
    enc_turn_impl, input_dispatch_impl, midi_rx_impl, prelude_str, register_foreign, reset,
    tick_impl,
};
#[cfg(feature = "wren-sys-backend")]
pub use bindings::{CLASSES, METHODS, enc_turn, input_dispatch, midi_rx, prelude_ptr, tick};
pub use deluge_audio_graph::{BusId, Cmd, Input, Kind, NodeId};
pub use host::{CV_CHANNELS, GATE_CHANNELS, Host, set_host};
pub use slotapi::{Handle, SlotApi, WrenForeign, WrenType};
