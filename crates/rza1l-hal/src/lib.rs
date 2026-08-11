//! `rza1` — Embassy HAL for the Renesas RZ/A1L (Cortex-A9, R7S721001).
//!
//! Peripheral base addresses are taken from the RZ/A1L Hardware Manual.

#![cfg_attr(target_os = "none", no_std)]
#![allow(dead_code)]

/// Offset from a cached address to its uncached mirror alias.
///
/// The RZ/A1L memory map provides uncached mirrors of the SRAM and SDRAM
/// regions offset by 0x4000_0000.  For example, cached SRAM at 0x2002_0000
/// has an uncached alias at 0x6002_0000.  Using the uncached alias for
/// DMA-shared buffers avoids stale cache lines without requiring explicit
/// cache maintenance operations.
pub const UNCACHED_MIRROR_OFFSET: usize = 0x4000_0000;

pub mod adc;
// startup, mmu and cache use ARM coprocessor assembly and cannot compile on the host.
pub mod bsc;
#[cfg(target_os = "none")]
pub mod cache;
pub mod dmac;
pub mod gic;
/// Pure memory-system math (cache-line ranges + MMU section table) used by the
/// bare-metal-only `cache`/`mmu` modules; non-gated so it unit-tests on host.
pub mod memmap;
/// MMIO seam: real volatile access on firmware, a shadow + access log under
/// test, so register write *sequences* can be asserted on the host.
pub mod mmio;
#[cfg(target_os = "none")]
pub mod mmu;
pub mod mtu2;
pub mod ostm;
pub mod rspi;
pub mod scux;
pub mod sdhi;
pub mod spibsc;
pub mod ssi;
#[cfg(target_os = "none")]
pub mod startup;
pub mod stb;
// The OSTM-backed embassy time driver does raw MMIO and registers the global
// `_embassy_time_now`/`_embassy_time_schedule_wake` symbols. On host the
// simulator uses embassy-time's `std` wall-clock driver instead, so keep this
// device-only to avoid a faulting driver and a duplicate-symbol link error.
#[cfg(target_os = "none")]
pub mod time_driver;
/// Pure arithmetic for the time driver — always compiled so its tests run on the host.
pub mod time_math;
pub mod uart;
pub mod usb;

pub mod gpio;
