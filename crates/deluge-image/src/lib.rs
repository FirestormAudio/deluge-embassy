//! Pure firmware-image format logic for the Deluge SSB, with **no hardware
//! dependencies** so it unit-tests on the host and is shared by the on-device
//! [`app-loader`] — keeping the format decisions in one place.
//!
//! * [`elf`] — pure `PT_LOAD` classification used by the streaming SD loader
//!   (uncached-mirror resolution, SRAM-staging / SDRAM-direct decisions), the
//!   slice-sourced load plan the USB dev-upload path uses, and the FSB-metadata
//!   validation ([`elf::validate_fsb_metadata`]) the flash-store path runs
//!   before programming an image.
//! * [`crc`] — the shared CRC-32 the USB upload framing and the on-flash
//!   settings record agree on, so the host tool and the device compute the same
//!   checksum.
//!
//! The crate is `no_std`; `cfg(test)` pulls in `std` so the test harness works.
#![cfg_attr(not(test), no_std)]

pub mod crc;
pub mod elf;
pub mod settings;

pub use crc::{Crc32, crc32};
