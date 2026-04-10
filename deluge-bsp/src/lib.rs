#![cfg_attr(target_os = "none", no_std)]
#![allow(dead_code)]

// Startup lives in rza1l_hal::startup. When rza1 is linked into any binary,
// startup.rs is included automatically because _start and the vector table
// are referenced by the linker script.

pub mod audio;
pub mod controls;
pub mod cv_gate;
#[cfg(target_os = "none")]
pub mod encoder;
pub mod fat;
pub mod midi_gate;
pub mod oled;
pub mod pads;
pub mod pic;
pub mod scux_dvu_path;
pub mod scux_src_path;
pub mod scux_usb_tx_path;
pub mod sd;
pub mod sdram;
pub mod system;
pub mod uart;
pub mod usb;

/// Guards concurrent RSPI0 access between the OLED DMA path and `cv_gate`.
///
/// The OLED and CV DAC share RSPI0 (channel 0). The OLED driver uses DMAC
/// channel 4 for frame transfers (8-bit mode); the CV DAC uses
/// `rspi::send32_blocking` (32-bit mode). This flag is set to `true` for the
/// duration of an OLED DMA transfer. `cv_set_blocking` spins on it before
/// reconfiguring RSPI0.
#[cfg(target_os = "none")]
pub static RSPI0_DMA_ACTIVE: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);
