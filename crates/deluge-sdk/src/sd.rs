//! SD-card file access (FAT).

#[cfg(target_os = "none")]
use deluge_bsp::fat::FatError;
#[cfg(target_os = "none")]
use deluge_bsp::sd::SdError;

/// Initialise the SD card. On the device this powers up and probes the card; on
/// the host simulator the "card" is a local directory, so there is nothing to do.
#[cfg(target_os = "none")]
pub(crate) async fn init_card() -> Result<(), SdError> {
    crate::plat::sd_init_card().await
}
/// Host: the simulated card is always available.
#[cfg(not(target_os = "none"))]
pub(crate) async fn init_card() -> Result<(), SdError> {
    crate::plat::sd_init_card().await
}

/// SD-card files, taken once from [`Deluge::sd`](crate::Deluge::sd).
///
/// A small convenience API for whole-file reads/writes of files in the card's
/// **root** directory (the common maker need: a config/preset/sample file). For
/// directory trees or streaming, drop down to [`deluge_bsp::fat`] /
/// [`embedded_sdmmc`].
///
/// Each call mounts the FAT volume, does the operation, and unmounts — so handles
/// never leak across calls. The card hardware is initialised once when the handle
/// is taken.
///
/// On the host simulator the "card root" is a local directory (the
/// `DELUGE_SIM_SD` env var, default `./sim-sd`), so reads/writes hit real files.
pub struct Sd {
    _not_send: crate::NotSend,
}

#[cfg(target_os = "none")]
impl Sd {
    pub(crate) fn new() -> Self {
        Self {
            _not_send: crate::NOT_SEND,
        }
    }

    /// Read a root-directory file into `buf`; returns the number of bytes read
    /// (capped at `buf.len()`).
    pub fn read(&mut self, name: &str, buf: &mut [u8]) -> Result<usize, FatError> {
        crate::plat::sd_read(name, buf)
    }

    /// Write `data` to a root-directory file, creating or truncating it.
    pub fn write(&mut self, name: &str, data: &[u8]) -> Result<(), FatError> {
        crate::plat::sd_write(name, data)
    }
}

// ── Host (desktop simulator) ─────────────────────────────────────────────────

/// Host SD-card hardware error (mirrors [`deluge_bsp::sd::SdError`]'s role).
#[cfg(not(target_os = "none"))]
#[derive(Debug)]
#[non_exhaustive]
pub enum SdError {
    /// I/O error talking to the simulated card directory.
    Io,
}

/// Host filesystem error from [`Sd`] read/write.
#[cfg(not(target_os = "none"))]
#[derive(Debug)]
#[non_exhaustive]
pub enum FatError {
    /// The file does not exist.
    NotFound,
    /// Other I/O error.
    Io,
}

#[cfg(not(target_os = "none"))]
impl From<std::io::Error> for FatError {
    fn from(e: std::io::Error) -> Self {
        match e.kind() {
            std::io::ErrorKind::NotFound => FatError::NotFound,
            _ => FatError::Io,
        }
    }
}

#[cfg(not(target_os = "none"))]
impl Sd {
    pub(crate) fn new() -> Self {
        Self {
            _not_send: crate::NOT_SEND,
        }
    }

    /// Read a root-directory file into `buf`; returns the number of bytes read.
    pub fn read(&mut self, name: &str, buf: &mut [u8]) -> Result<usize, FatError> {
        crate::plat::sd_read(name, buf)
    }

    /// Write `data` to a root-directory file, creating or truncating it.
    pub fn write(&mut self, name: &str, data: &[u8]) -> Result<(), FatError> {
        crate::plat::sd_write(name, data)
    }
}
