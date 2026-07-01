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
//! - drains [`Cmd`]s from its `Host::audio_cmd` into an [`Engine`] it renders.

#![no_std]

mod audio;
mod bindings;
mod engine;
mod host;
mod slotapi;
mod slotapi_wrensys;

pub use bindings::{CLASSES, METHODS, enc_turn, input_dispatch, midi_rx, prelude_ptr, reset, tick};
pub use engine::{
    Cmd, Engine, Input, K_ADD, K_ENV, K_LPF, K_MUL, K_NOISE, K_SAW, K_SINE, K_SQUARE, K_SUB, K_TRI,
    MAX_NODES,
};
pub use host::{CV_CHANNELS, GATE_CHANNELS, Host, set_host};
pub use slotapi::{Handle, SlotApi, WrenForeign, WrenType};
