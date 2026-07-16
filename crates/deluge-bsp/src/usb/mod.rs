//! USB class layer for the Deluge.
//!
//! The chip-level RUSB1 device/host drivers (register map, FIFO/pipe layers,
//! `embassy-usb-driver` implementations, `init_device_mode` /
//! `init_host_mode`) live in [`rza1l_hal::usb`].  This module holds only the
//! board/class layer on top:
//!
//! - [`classes`]: UAC2 audio, USB-MIDI, and MSC class implementations
//!   (generic over any `embassy_usb::driver::Driver`).
//! - [`bot`]: USB Mass Storage Bulk-Only Transport engine.
//! - [`host`]: **host**-side class drivers for devices the Deluge hosts
//!   (generic over any `embassy_usb_driver::host::UsbHostAllocator`).
//!
//! See [`rza1l_hal::usb`] for the quick-start examples (device mode, host
//! mode, ISR wiring).

// Device-side pieces need `embassy-usb`, which is only a dependency on the
// bare-metal target.
#[cfg(target_os = "none")]
pub mod bot;
#[cfg(target_os = "none")]
pub mod classes;
// Host-side class drivers are generic over `UsbHostAllocator` and build (and
// are tested) on the QEMU ARM target too — see `tools/test.sh`.
pub mod host;
pub mod ids;
