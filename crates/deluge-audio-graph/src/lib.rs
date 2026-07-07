//! Portable `no_std` block-rendering audio graph engine.
//!
//! Nodes (a uniform enum wrapping `deluge-dsp-kernels` structs) evaluate in
//! topological order into a pooled output arena; ports are addressed `(node,
//! port)`; buses carry stereo bundles + fan-in; a `Host` transports control-rate
//! `Cmd`s to the engine. Sizes are const-generic; sample rate is runtime.
#![no_std]

pub mod frame;
pub mod ids;

pub use frame::StereoFrame;
pub use ids::{BusId, Input, NodeId};
