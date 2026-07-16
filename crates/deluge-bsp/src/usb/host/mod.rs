//! USB **host**-side class drivers.
//!
//! The device-side classes live in [`super::classes`]. This module is the
//! mirror image: drivers for devices the Deluge *hosts*, built on
//! `embassy-usb-host` and the RUSB1 host driver in [`rza1l_hal::usb`].
//!
//! Class drivers here are generic over [`UsbHostAllocator`], so they are unit
//! tested against [`mock::MockAlloc`] under QEMU — see `tools/test.sh`.
//!
//! [`UsbHostAllocator`]: embassy_usb_driver::host::UsbHostAllocator

pub mod midi;

#[cfg(all(test, not(target_os = "none")))]
pub(crate) mod mock;
