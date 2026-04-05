//! FAT filesystem layer for the Deluge SD card.
//!
//! Wraps [`embedded_sdmmc::VolumeManager`] with the Deluge-specific block
//! device ([`DelugeBlockDevice`]) and time source ([`DelugeTimeSource`]).
//!
//! ## Quick start
//!
//! ```ignore
//! use deluge_bsp::fat::{self, Mode};
//!
//! // Call sd::init().await first, then:
//! let mut vm = fat::new_volume_manager();
//!
//! // Open the first (and usually only) FAT volume on the card.
//! let volume = vm.open_raw_volume(fat::VolumeIdx(0)).unwrap();
//! let root   = vm.open_root_dir(volume).unwrap();
//!
//! // List root directory entries (short names only).
//! vm.iterate_dir(root, |entry| {
//!     // entry.name is a ShortFileName
//! }).unwrap();
//!
//! // Read a file.
//! let file = vm.open_file_in_dir(root, "SONG.XML", Mode::ReadOnly).unwrap();
//! let mut buf = [0u8; 512];
//! let n = vm.read(file, &mut buf).unwrap();
//! vm.close_file(file).unwrap();
//!
//! // Write / append a file.
//! let file = vm.open_file_in_dir(root, "LOG.TXT",
//!                                Mode::ReadWriteCreateOrAppend).unwrap();
//! vm.write(file, b"hello\n").unwrap();
//! vm.flush_file(file).unwrap();
//! vm.close_file(file).unwrap();
//!
//! vm.close_dir(root).unwrap();
//! vm.close_volume(volume).unwrap();
//! ```
//!
//! ## Limits (defaults)
//!
//! | Parameter     | Value |
//! |---------------|-------|
//! | `MAX_DIRS`    | 4     |
//! | `MAX_FILES`   | 4     |
//! | `MAX_VOLUMES` | 1     |
//!
//! Call [`new_volume_manager_with_limits`] if you need different values.

use embedded_sdmmc::VolumeManager;

use crate::sd::{DelugeBlockDevice, DelugeTimeSource, SdError};

// ---------------------------------------------------------------------------
// Public re-exports — everything a caller needs to use the FAT API.
// ---------------------------------------------------------------------------

pub use embedded_sdmmc::{
    DirEntry, Mode, RawDirectory, RawFile, RawVolume, ShortFileName, VolumeIdx,
};

/// The [`embedded_sdmmc`] error type parameterised for the Deluge block device.
pub type FatError = embedded_sdmmc::Error<SdError>;

// ---------------------------------------------------------------------------
// Type alias
// ---------------------------------------------------------------------------

/// A [`VolumeManager`] pre-configured for the Deluge SD card.
///
/// Default limits: `MAX_DIRS = 4`, `MAX_FILES = 4`, `MAX_VOLUMES = 1`.
/// These are heap-free and stack-allocated.
///
/// [`sd::init`](crate::sd::init) **must** have completed successfully before
/// calling any method on this type.
pub type DelugeVolumeManager = VolumeManager<DelugeBlockDevice, DelugeTimeSource>;

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

/// Create a [`DelugeVolumeManager`] with the default open-handle limits.
///
/// Equivalent to `VolumeManager::new(DelugeBlockDevice, DelugeTimeSource)`.
#[inline]
pub fn new_volume_manager() -> DelugeVolumeManager {
    VolumeManager::new(DelugeBlockDevice, DelugeTimeSource)
}

/// Create a [`VolumeManager`] with custom open-handle limits.
///
/// Use this when you need more than 4 open directories or files, or more than
/// 1 open volume simultaneously.
///
/// # Example
///
/// ```ignore
/// let mut vm = fat::new_volume_manager_with_limits::<8, 8, 2>(100);
/// ```
#[inline]
pub fn new_volume_manager_with_limits<
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    id_offset: u32,
) -> VolumeManager<DelugeBlockDevice, DelugeTimeSource, MAX_DIRS, MAX_FILES, MAX_VOLUMES> {
    VolumeManager::new_with_limits(DelugeBlockDevice, DelugeTimeSource, id_offset)
}
