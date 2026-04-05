//! `rza1` — Embassy HAL for the Renesas RZ/A1L (Cortex-A9, R7S721001).
//!
//! Peripheral base addresses are taken from the RZ/A1L Hardware Manual.

#![cfg_attr(target_os = "none", no_std)]
#![feature(allocator_api)]
#![allow(dead_code)]

/// Offset from a cached address to its uncached mirror alias.
///
/// The RZ/A1L memory map provides uncached mirrors of the SRAM and SDRAM
/// regions offset by 0x4000_0000.  For example, cached SRAM at 0x2002_0000
/// has an uncached alias at 0x6002_0000.  Using the uncached alias for
/// DMA-shared buffers avoids stale cache lines without requiring explicit
/// cache maintenance operations.
pub const UNCACHED_MIRROR_OFFSET: usize = 0x4000_0000;

// startup, mmu and cache use ARM coprocessor assembly and cannot compile on the host.
pub mod bsc;
#[cfg(target_os = "none")]
pub mod cache;
pub mod dmac;
pub mod gic;
#[cfg(target_os = "none")]
pub mod mmu;
pub mod mtu2;
pub mod ostm;
pub mod rspi;
pub mod rusb1;
pub mod scux;
pub mod sdhi;
pub mod ssi;
#[cfg(target_os = "none")]
pub mod startup;
pub mod stb;
pub mod time_driver;
pub mod uart;

pub mod allocator;
pub mod gpio;
