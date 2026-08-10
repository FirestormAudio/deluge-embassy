#![cfg_attr(target_os = "none", no_std)]
// `#[embassy_executor::task]` (used by `usb::host`'s supervisor and per-device
// tasks) expands to an associated type with `impl Trait`. Only needed on the
// bare-metal target, where those tasks are compiled.
#![cfg_attr(target_os = "none", feature(impl_trait_in_assoc_type))]
#![allow(dead_code)]

// Startup lives in rza1l_hal::startup. When rza1 is linked into any binary,
// startup.rs is included automatically because _start and the vector table
// are referenced by the linker script.

pub mod audio;
#[cfg(target_os = "none")]
pub mod audio_block;
pub mod battery;
#[cfg(target_os = "none")]
pub mod bus;
pub mod controls;
pub mod cv_gate;
pub mod encoder;
/// Pure quadrature detent accumulation used by the bare-metal `encoder` driver;
/// non-gated so it unit-tests on the host.
pub mod encoder_detent;
// `fat` builds on `sd`, and `midi_gate` pulls in `cortex_ar` — both depend on
// items only available on the bare-metal target, so they are excluded from the
// host/QEMU test build (the pure-logic modules below still compile there).
#[cfg(all(target_os = "none", feature = "fat"))]
pub mod fat;
/// Deluge SPI-NOR flash profile (chip geometry + board memory map).  Pure
/// data + re-exports over the SoC `rza1l_hal::spibsc` controller.  Opt-in via
/// the `flash` feature — only the app-loader writes to flash.
#[cfg(feature = "flash")]
pub mod flash;
pub mod jacks;
#[cfg(target_os = "none")]
pub mod midi_gate;
pub mod oled;
pub mod pads;
pub mod pic;
pub mod rgb;
/// Pure audio sample-format conversion + dither used by the bare-metal
/// `audio_block` ring driver; non-gated so it unit-tests on the host.
pub mod sample_fmt;
pub mod scux_dvu_path;
pub mod scux_src_path;
pub mod scux_usb_tx_path;
/// SD card driver: real SDHI1 hardware on the firmware target, a file-backed
/// disk image standing in for the card on host/QEMU. See the module docs.
pub mod sd;
pub mod sdram;
pub mod system;
#[cfg(target_os = "none")]
pub mod trigger_clock;
pub mod uart;
// Not gated to `target_os = "none"` as a whole: `usb::host`'s class drivers are
// generic over `UsbHostAllocator` and are unit-tested under QEMU on
// armv7-unknown-linux-gnueabihf. The device-side children that need
// `embassy-usb` are gated individually inside the module.
pub mod usb;

// RSPI0 arbitration between the OLED DMA path and the CV DAC now lives in
// [`bus`] as an owned, mutex-guarded resource (replacing the former
// `RSPI0_DMA_ACTIVE` spin-flag). See the Advanced developer guide
// (`docs/advanced-guide.md`, §7 — *Dropping down to the BSP & HAL*).
