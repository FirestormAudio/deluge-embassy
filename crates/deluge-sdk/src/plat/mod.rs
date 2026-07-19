//! Backend selection. Each capability module delegates its backend-divergent
//! operations here; exactly one submodule compiles per build:
//!   device  — target_os = "none"                     (deluge-bsp peripherals)
//!   sim     — host, no `linux` feature                (deluge-sim-link panel)
//!   linux   — the `linux` feature                     (libdeluge; Phase 1b)
#[cfg(target_os = "none")]
mod device;
#[cfg(target_os = "none")]
pub(crate) use device::*;

#[cfg(feature = "sim")]
mod sim;
#[cfg(feature = "sim")]
pub(crate) use sim::*;

#[cfg(feature = "linux")]
mod linux;
#[cfg(feature = "linux")]
pub(crate) use linux::*;
