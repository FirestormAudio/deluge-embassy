//! Persistent app-loader settings record — the pure encode/decode for the
//! settings the SSB stores in the SPI-flash settings sector: the dev-mode flag
//! and the auto-boot delay.
//!
//! The hardware read/write (flash erase/program through the memory-mapped
//! window) lives in the on-device `app-loader::settings` wrapper; the *format* —
//! magic, version, flags, CRC — lives here so it is host-testable and has a
//! single definition, exactly like the ELF/FSB helpers in [`crate::elf`].
//!
//! ## On-flash layout (one 256 B page; the rest of the sector stays erased)
//!
//! | Offset | Field       | Notes                                                |
//! |--------|-------------|------------------------------------------------------|
//! | 0..4   | `magic`     | `b"DSET"`                                             |
//! | 4      | `version`   | record version (`2`; version `1` is still readable)  |
//! | 5      | `flags`     | bit 0 = dev_mode; other bits reserved (0)            |
//! | 6      | `auto_boot` | `0` = instant, `1..=20` = seconds, `21` = never      |
//! | 7      | reserved    | 0                                                     |
//! | 8..12  | `crc32`     | CRC-32 (IEEE) of bytes `0..8`, little-endian          |
//!
//! Erased flash reads `0xFF`, so a blank sector fails the magic check and the
//! device falls back to [`Settings::default`] (dev mode off, 5 s countdown).
//!
//! ## Version 1 → 2
//!
//! Version 1 had no `auto_boot` byte: offset 6 was reserved and written as `0`.
//! Because `0` now *means* "boot instantly, never show the menu", the byte could
//! not simply be claimed — reinterpreting it would silently delete the boot menu
//! from every unit already in the field.  So [`decode`] reads both versions (a v1
//! record yields [`AutoBoot::default`], i.e. today's 5-second countdown) while
//! [`encode`] only ever writes v2, migrating the record on the next write.

use crate::crc::crc32;

/// How long the boot menu waits before launching the default entry.
///
/// The values form a single dial, ordered by how long the unit waits:
/// `Instant → Secs(1) → … → Secs(20) → Never`.  See [`AutoBoot::step`] (added in
/// the settings-UI task) for walking it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AutoBoot {
    /// Launch the default entry immediately — the menu is never drawn.  Only the
    /// recovery gesture (SELECT held at power-on) can reach the menu on a unit
    /// set this way.
    Instant,
    /// Draw the menu and auto-boot the default entry after `1..=MAX_AUTO_BOOT_SECS`
    /// seconds.
    Secs(u8),
    /// Draw the menu and wait indefinitely — dev mode's behaviour without dev
    /// mode's USB upload listener.
    Never,
}

/// The out-of-the-box delay, and what a version-1 record decodes to.
pub const DEFAULT_AUTO_BOOT_SECS: u8 = 5;
/// The longest countdown that can be selected.
pub const MAX_AUTO_BOOT_SECS: u8 = 20;
/// The byte that encodes [`AutoBoot::Never`] — one past the longest countdown,
/// which also makes it the top of the dial.
const NEVER_BYTE: u8 = MAX_AUTO_BOOT_SECS + 1;

impl Default for AutoBoot {
    fn default() -> Self {
        AutoBoot::Secs(DEFAULT_AUTO_BOOT_SECS)
    }
}

impl AutoBoot {
    /// The on-flash byte (also the dial position): `0` instant, `1..=20` seconds,
    /// `21` never.
    pub fn to_byte(self) -> u8 {
        match self {
            AutoBoot::Instant => 0,
            AutoBoot::Secs(n) => n.clamp(1, MAX_AUTO_BOOT_SECS),
            AutoBoot::Never => NEVER_BYTE,
        }
    }

    /// Inverse of [`AutoBoot::to_byte`].  Any byte past the dial decodes to
    /// [`AutoBoot::Never`] rather than failing: a unit that always shows its menu
    /// is recoverable, one that instant-boots on garbage is not.
    pub fn from_byte(b: u8) -> Self {
        match b {
            0 => AutoBoot::Instant,
            n if n <= MAX_AUTO_BOOT_SECS => AutoBoot::Secs(n),
            _ => AutoBoot::Never,
        }
    }
}

/// Persistent loader settings.  Room to grow: add a field here plus a flag bit
/// (or a new versioned field) in [`encode`]/[`decode`].
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Settings {
    /// When `true`, the loader listens for USB uploads and never auto-boots.
    /// Default `false` — a stock unit never accepts firmware over USB.
    pub dev_mode: bool,
    /// How long the boot menu waits before launching the default entry.
    pub auto_boot: AutoBoot,
}

/// Magic identifying a valid settings record.
pub const MAGIC: [u8; 4] = *b"DSET";
/// Current record version — what [`encode`] writes.
pub const VERSION: u8 = 2;
/// The previous record version, still accepted by [`decode`] (no `auto_boot` byte).
const VERSION_V1: u8 = 1;
/// `flags` bit: dev mode enabled.
const FLAG_DEV_MODE: u8 = 1 << 0;
/// Bytes covered by the CRC (everything before the CRC word).
const PAYLOAD_LEN: usize = 8;
/// Total encoded record length.
pub const RECORD_LEN: usize = 12;

/// Encode `s` into its fixed-size on-flash record (magic + version + flags +
/// auto-boot + reserved + CRC-32 of the payload).  Always writes [`VERSION`].
pub fn encode(s: &Settings) -> [u8; RECORD_LEN] {
    let mut buf = [0u8; RECORD_LEN];
    buf[0..4].copy_from_slice(&MAGIC);
    buf[4] = VERSION;
    buf[5] = if s.dev_mode { FLAG_DEV_MODE } else { 0 };
    buf[6] = s.auto_boot.to_byte();
    // buf[7] reserved, already zero.
    let crc = crc32(&buf[..PAYLOAD_LEN]);
    buf[8..12].copy_from_slice(&crc.to_le_bytes());
    buf
}

/// Decode a settings record, or `None` if `buf` is not a valid one (too short,
/// wrong magic, unknown version, or a CRC mismatch — including blank `0xFF`
/// flash).
///
/// Both [`VERSION`] and the older version 1 are accepted; a v1 record has no
/// auto-boot byte and yields [`AutoBoot::default`].
pub fn decode(buf: &[u8]) -> Option<Settings> {
    if buf.len() < RECORD_LEN {
        return None;
    }
    if buf[0..4] != MAGIC {
        return None;
    }
    let stored = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
    if crc32(&buf[..PAYLOAD_LEN]) != stored {
        return None;
    }
    let auto_boot = match buf[4] {
        // v1 wrote byte 6 as a reserved zero — it must NOT be read as "instant".
        VERSION_V1 => AutoBoot::default(),
        VERSION => AutoBoot::from_byte(buf[6]),
        _ => return None,
    };
    Some(Settings {
        dev_mode: buf[5] & FLAG_DEV_MODE != 0,
        auto_boot,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a version-1 record by hand — the format that shipped: no auto-boot
    /// byte, `flags` bit 0 = dev_mode, byte 6 reserved (zero), CRC over 0..8.
    /// This is what units already in the field have on their flash.
    fn v1_record(dev_mode: bool) -> [u8; RECORD_LEN] {
        let mut buf = [0u8; RECORD_LEN];
        buf[0..4].copy_from_slice(&MAGIC);
        buf[4] = 1;
        buf[5] = if dev_mode { FLAG_DEV_MODE } else { 0 };
        let crc = crc32(&buf[..PAYLOAD_LEN]);
        buf[8..12].copy_from_slice(&crc.to_le_bytes());
        buf
    }

    #[test]
    fn round_trips_every_auto_boot_setting() {
        let mut settings = vec![AutoBoot::Instant, AutoBoot::Never];
        settings.extend((1..=MAX_AUTO_BOOT_SECS).map(AutoBoot::Secs));
        for auto_boot in settings {
            for dev_mode in [false, true] {
                let s = Settings {
                    dev_mode,
                    auto_boot,
                };
                assert_eq!(decode(&encode(&s)), Some(s), "round-trip {:?}", s);
            }
        }
    }

    #[test]
    fn encode_always_emits_version_2() {
        let rec = encode(&Settings::default());
        assert_eq!(rec[4], 2);
    }

    #[test]
    fn auto_boot_byte_encoding_is_stable() {
        // The on-flash contract: 0 = instant, 1..=20 = seconds, 21 = never.
        assert_eq!(encode(&settings_with(AutoBoot::Instant))[6], 0);
        assert_eq!(encode(&settings_with(AutoBoot::Secs(5)))[6], 5);
        assert_eq!(encode(&settings_with(AutoBoot::Secs(20)))[6], 20);
        assert_eq!(encode(&settings_with(AutoBoot::Never))[6], 21);
    }

    fn settings_with(auto_boot: AutoBoot) -> Settings {
        Settings {
            dev_mode: false,
            auto_boot,
        }
    }

    #[test]
    fn v1_record_migrates_preserving_dev_mode() {
        // The upgrade case: a unit in the field keeps its dev-mode flag and its
        // current 5-second countdown.
        for dev_mode in [false, true] {
            assert_eq!(
                decode(&v1_record(dev_mode)),
                Some(Settings {
                    dev_mode,
                    auto_boot: AutoBoot::Secs(DEFAULT_AUTO_BOOT_SECS),
                }),
            );
        }
    }

    #[test]
    fn out_of_range_auto_boot_byte_clamps_to_never() {
        // Never auto-booting is the safe direction: worst case the unit always
        // shows its menu.  Crucially, the record is *not* rejected — that would
        // throw away a valid dev_mode flag alongside the bad byte.
        let mut rec = encode(&Settings {
            dev_mode: true,
            auto_boot: AutoBoot::Never,
        });
        for byte in [22u8, 100, 0xFE] {
            rec[6] = byte;
            let crc = crc32(&rec[..PAYLOAD_LEN]);
            rec[8..12].copy_from_slice(&crc.to_le_bytes());
            assert_eq!(
                decode(&rec),
                Some(Settings {
                    dev_mode: true,
                    auto_boot: AutoBoot::Never,
                }),
                "byte {byte}",
            );
        }
    }

    #[test]
    fn blank_or_zeroed_flash_is_rejected() {
        // Erased flash reads all 0xFF; a never-written sector may read 0x00.
        assert_eq!(decode(&[0xFFu8; RECORD_LEN]), None);
        assert_eq!(decode(&[0x00u8; RECORD_LEN]), None);
    }

    #[test]
    fn rejects_corruption_and_unknown_version() {
        // Flip a payload bit without fixing the CRC.
        let mut rec = encode(&Settings {
            dev_mode: true,
            auto_boot: AutoBoot::Secs(3),
        });
        rec[5] ^= 0x02;
        assert_eq!(decode(&rec), None);

        // Truncated record.
        let rec = encode(&Settings::default());
        assert_eq!(decode(&rec[..RECORD_LEN - 1]), None);

        // A version we do not know how to read (0 and 3 bracket the supported 1..=2).
        for version in [0u8, 3, 0xFF] {
            let mut rec = encode(&Settings::default());
            rec[4] = version;
            let crc = crc32(&rec[..PAYLOAD_LEN]);
            rec[8..12].copy_from_slice(&crc.to_le_bytes());
            assert_eq!(decode(&rec), None, "version {version}");
        }
    }

    #[test]
    fn default_is_dev_mode_off_and_five_seconds() {
        let d = Settings::default();
        assert!(!d.dev_mode);
        assert_eq!(d.auto_boot, AutoBoot::Secs(DEFAULT_AUTO_BOOT_SECS));
    }
}
