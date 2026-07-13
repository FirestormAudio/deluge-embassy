//! Persistent app-loader settings, stored in the SPI-flash **settings sector**
//! ([`flash::SETTINGS_ADDR`]) so they survive a power-cycle and do not
//! depend on the SD card.
//!
//! The only setting today is the **dev-mode** flag: when on, `boot_task` brings
//! up a USB CDC upload listener in the background (see [`crate::devupload`]) and
//! disables the auto-boot countdown so an upload can arrive at any time.  USB
//! firmware acceptance is impossible unless the user has explicitly turned dev
//! mode on, which is why it is persisted here rather than re-derived.
//!
//! The on-flash **format** (magic / version / flags / CRC) and its encode/decode
//! are the host-tested [`deluge_image::settings`] functions; this module only
//! adds the hardware read (memory-mapped window) and write (erase + program),
//! reusing the exact flash path the app-slot store uses.

use deluge_bsp::flash;

pub use deluge_image::settings::{
    AUTO_BOOT_LABEL_MAX, AutoBoot, BootMode, RECORD_LEN, Settings, boot_mode, decode, encode,
};

/// Address of the settings sector in the **uncached** memory-mapped flash mirror
/// (`0x5840_0000`).  We read settings through this mirror, not the cached
/// `SETTINGS_SECTOR_ADDR` window, because the RZ/A1L's caches ignore maintenance
/// ops: after [`write`] programs the sector, the cached window can still return
/// the stale pre-write line, which would make the post-write read-back (and a
/// same-session re-read) lie.  The uncached mirror always sees current flash.
const SETTINGS_UNCACHED_ADDR: u32 = flash::SETTINGS_ADDR + rza1l_hal::UNCACHED_MIRROR_OFFSET as u32;

/// Read the persisted settings from the flash settings sector (via the uncached
/// mirror), falling back to [`Settings::default`] on a blank or invalid record
/// (erased flash reads `0xFF` and fails the magic check).
pub fn read() -> Settings {
    let mut buf = [0u8; RECORD_LEN];
    for (i, b) in buf.iter_mut().enumerate() {
        *b = unsafe { core::ptr::read_volatile((SETTINGS_UNCACHED_ADDR + i as u32) as *const u8) };
    }
    decode(&buf).unwrap_or_default()
}

/// Persist `s` to the flash settings sector: erase the sector, then program the
/// encoded record into its first page.  Reuses the exact erase/program path the
/// flash app-slot store uses (`flash::MAP.erase_range` + `.program`); a
/// single 256 B page per write keeps wear negligible.
///
/// Returns `true` if the record was programmed and reads back correctly.  A
/// `false` return means the erase/program was rejected by the chip (e.g. the
/// sector is write-protected) — the caller should surface that rather than
/// assume the toggle persisted.
///
/// # Safety
/// Switches the SPI-flash bus out of memory-mapped read mode (no code may run
/// from flash during the call — true in the SSB, which runs from SRAM).  Call
/// only from the boot menu context, like the app-slot store.
pub async unsafe fn write(s: &Settings) -> bool {
    let record = encode(s);
    // `FlashMap::erase_range`/`program` each run their manual-mode window with
    // interrupts masked and a bounded WIP-wait internally — required on this
    // board, where taking the OLED SPI-DMA interrupt mid-erase freezes the
    // machine, and where a rejected erase would otherwise spin forever (see the
    // SPIBSC module docs and `DelugeBootloader/src/spibsc_init2.c`).
    unsafe {
        flash::MAP.erase_range(flash::SETTINGS_OFFSET, flash::PAGE as u32);
        flash::MAP.program(flash::SETTINGS_OFFSET, &record);
    }
    // Verify through the uncached mirror (the cached window may be stale).
    read() == *s
}
