# Configurable Auto-Boot Timer + Hold-SELECT Recovery — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the app-loader's auto-boot delay a persistent, on-device setting (`INSTANT` / `1–20 s` / `NEVER`) and add a hold-SELECT-at-power-on gesture that always reaches the boot menu.

**Architecture:** All the decidable logic — the on-flash record, the `INSTANT → 1S … 20S → NEVER` dial, and the boot-mode decision — goes into `crates/deluge-image/src/settings.rs`, which is a `no_std` crate that unit-tests on the host and is already where the settings record lives. The `app-loader` (a `no_std` binary with `test = false`, so it has no unit tests) is left holding only hardware wiring and OLED drawing.

**Tech Stack:** Rust 2024, `no_std`, Embassy (executor + time), `cargo test --target x86_64-unknown-linux-gnu` for host tests, `cargo build-app-loader` for the ARM build.

**Spec:** `docs/superpowers/specs/2026-07-13-boot-autoboot-and-recovery-design.md`

## Global Constraints

- **Host tests must pass the explicit target.** `.cargo/config.toml` sets `build.target = "armv7a-none-eabihf"`, so a bare `cargo test` tries to build tests for the ARM target and fails. Always: `cargo test -p deluge-image --target x86_64-unknown-linux-gnu`.
- **The ARM build command is an alias:** `cargo build-app-loader` (it passes the `-Zbuild-std=core` flags the bare-metal target needs). A plain `cargo build -p app-loader` will not link.
- **`deluge-image` is `no_std`** (`#![cfg_attr(not(test), no_std)]`). No `std`, no `alloc`, no floating point in this crate's non-test code.
- **`app-loader` has `test = false`** in its `Cargo.toml`. Do not add unit tests to `app-loader`; if logic needs a test, it belongs in `deluge-image`.
- **The OLED font advances 6 px per character** and the panel is 128 px wide. Menu entry text starts at `x = 8`, so an entry label may be at most **20 characters**.
- **Byte encoding of the auto-boot setting is fixed:** `0` = `INSTANT`, `1..=20` = seconds, `21` = `NEVER`. Anything above `21` decodes to `NEVER`.
- **Never break units in the field:** an existing version-1 settings record must keep its `dev_mode` flag and behave as it does today (5 s countdown).

## File Structure

| File | Responsibility |
|------|----------------|
| `crates/deluge-image/src/settings.rs` | **modify** — `AutoBoot` type + dial + label, v2 record with v1 migration, `boot_mode` decision. All host-tested. |
| `app-loader/src/main.rs` | **modify** — `SELECT_SEEN` latch, recovery splash, `boot_mode` dispatch, `auto_boot_allowed`, `SETTINGS` menu entry + its flash write. |
| `app-loader/src/ui.rs` | **modify** — titled `run_menu`, the `run_settings` screen, the SELECT-arming fix. |
| `app-loader/src/settings.rs` | **unchanged** — the flash read/write wrapper re-exports whatever `deluge_image::settings` defines. |
| `docs/app-loader.md`, `app-loader/README.md`, `docs/getting-started.md` | **modify** — menu structure, controls table, new SETTINGS/RECOVERY sections. |

---

### Task 0: Verify the PIC reports an already-held button (hardware spike)

The recovery gesture assumes the PIC32 reports a button that was **already held when it powered up**. `pic::init()` sends `CMD_RESEND_BUTTON_STATES`, which should make it do so — but this is unverified on hardware, and every later task depends on it. **This task writes no committed code.**

**Files:**
- Temporarily modify: `app-loader/src/main.rs:129-162` (`pic_rx_task`)

- [ ] **Step 1: Add a temporary log of every button event**

In `pic_rx_task`, inside the `match parser.push(byte)` block, add a temporary arm **before** the existing arms:

```rust
Some(Event::ButtonPress { id }) => {
    log::info!("SPIKE: ButtonPress id={}", id);
    // fall through to the real handling below by duplicating it here:
    if id == controls::encoder_button::SELECT {
        ui::SELECT_DOWN.store(true, Ordering::Release);
    }
    if id == controls::button::BACK {
        crate::BACK_PRESSED.store(true, Ordering::Release);
    }
}
```

- [ ] **Step 2: Build and flash with RTT logging**

Run: `cargo build-app-loader --features rtt`
Then flash the resulting binary to the unit as you normally would and attach the RTT viewer.

- [ ] **Step 3: Power on with SELECT held**

Hold the **SELECT encoder button** down, power-cycle the Deluge, and keep holding until the boot menu appears.

Expected: `SPIKE: ButtonPress id=31` appears in the RTT log within the first ~500 ms (31 is `controls::encoder_button::SELECT`).

- [ ] **Step 4: Decide**

- **If `id=31` appears:** the assumption holds. Revert the spike (`git checkout app-loader/src/main.rs`) and continue to Task 1.
- **If it does not appear:** the PIC does not report already-held buttons. **Stop and report this to the user** — the fallbacks are picking a button the PIC does report on its first scan, or reading the button matrix directly, and that is a design decision, not an implementation one. Do not guess.

- [ ] **Step 5: Revert the spike**

```bash
git checkout app-loader/src/main.rs
```

Nothing is committed by this task.

---

### Task 1: `AutoBoot` type and the version-2 settings record

**Files:**
- Modify: `crates/deluge-image/src/settings.rs` (the whole file — module docs, `Settings`, `encode`, `decode`, tests)

**Interfaces:**
- Consumes: `crate::crc::crc32` (already imported in this file).
- Produces:
  - `pub enum AutoBoot { Instant, Secs(u8), Never }` — `Clone + Copy + PartialEq + Eq + Debug`, `Default = Secs(5)`.
  - `pub const DEFAULT_AUTO_BOOT_SECS: u8 = 5;`
  - `pub const MAX_AUTO_BOOT_SECS: u8 = 20;`
  - `impl AutoBoot { pub fn to_byte(self) -> u8; pub fn from_byte(b: u8) -> Self; }`
  - `pub struct Settings { pub dev_mode: bool, pub auto_boot: AutoBoot }` — the existing `dev_mode` field is unchanged.
  - `pub const VERSION: u8 = 2;` (was `1`)
  - `encode`/`decode` keep their existing signatures: `fn encode(s: &Settings) -> [u8; RECORD_LEN]`, `fn decode(buf: &[u8]) -> Option<Settings>`. `RECORD_LEN` stays `12`.

- [ ] **Step 1: Write the failing tests**

Replace the entire `#[cfg(test)] mod tests` block at the bottom of `crates/deluge-image/src/settings.rs` with:

```rust
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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p deluge-image --target x86_64-unknown-linux-gnu settings`
Expected: FAIL — compile errors, `cannot find type AutoBoot in this scope` and `struct Settings has no field named auto_boot`.

- [ ] **Step 3: Implement `AutoBoot` and the v2 record**

In `crates/deluge-image/src/settings.rs`, replace the module doc-comment's on-flash layout table and the `Settings` / constants / `encode` / `decode` items (everything above the `#[cfg(test)]` block) with:

```rust
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
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p deluge-image --target x86_64-unknown-linux-gnu settings`
Expected: PASS — 8 tests (`round_trips_every_auto_boot_setting`, `encode_always_emits_version_2`, `auto_boot_byte_encoding_is_stable`, `v1_record_migrates_preserving_dev_mode`, `out_of_range_auto_boot_byte_clamps_to_never`, `blank_or_zeroed_flash_is_rejected`, `rejects_corruption_and_unknown_version`, `default_is_dev_mode_off_and_five_seconds`).

- [ ] **Step 5: Confirm the app-loader still builds**

`app-loader/src/settings.rs` re-exports `Settings` and constructs it in `main.rs`'s dev-mode toggle (`Settings { dev_mode: !cfg.dev_mode }`), which is now missing a field.

Run: `cargo build-app-loader`
Expected: FAIL — `missing field auto_boot in initializer of Settings` at `app-loader/src/main.rs:482`.

Fix it minimally (the dev-mode toggle keeps the rest of the settings as they were — this line is rewritten entirely in Task 3 anyway):

```rust
            let new_cfg = settings::Settings {
                dev_mode: !cfg.dev_mode,
                ..cfg
            };
```

Run: `cargo build-app-loader`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-image/src/settings.rs app-loader/src/main.rs
git commit -m "feat(settings): AutoBoot setting + v2 record with v1 migration"
```

---

### Task 2: The dial, the label, and the boot-mode decision

Everything the UI and the boot loop need to *reason* about auto-boot, so neither has to.

**Files:**
- Modify: `crates/deluge-image/src/settings.rs` (add to `impl AutoBoot`, add `BootMode` + `boot_mode`, add tests)

**Interfaces:**
- Consumes: `AutoBoot`, `Settings`, `MAX_AUTO_BOOT_SECS`, `NEVER_BYTE` from Task 1.
- Produces:
  - `impl AutoBoot { pub fn step(self, detents: i8) -> Self; pub fn label(self, buf: &mut [u8; AUTO_BOOT_LABEL_MAX]) -> &[u8]; }`
  - `pub const AUTO_BOOT_LABEL_MAX: usize = 7;`
  - `pub enum BootMode { Instant, Countdown(u8), Wait }` — `Clone + Copy + PartialEq + Eq + Debug`.
  - `pub fn boot_mode(cfg: &Settings, boot_total: usize, recovery: bool, auto_boot_allowed: bool) -> BootMode`

- [ ] **Step 1: Write the failing tests**

Append these tests **inside** the existing `#[cfg(test)] mod tests` block in `crates/deluge-image/src/settings.rs` (after the tests from Task 1):

```rust
    // ---- dial ---------------------------------------------------------------

    #[test]
    fn step_walks_the_dial_in_both_directions() {
        assert_eq!(AutoBoot::Instant.step(1), AutoBoot::Secs(1));
        assert_eq!(AutoBoot::Secs(1).step(-1), AutoBoot::Instant);
        assert_eq!(AutoBoot::Secs(5).step(1), AutoBoot::Secs(6));
        assert_eq!(AutoBoot::Secs(5).step(-1), AutoBoot::Secs(4));
        // The top of the dial is NEVER, one step past the longest countdown.
        assert_eq!(AutoBoot::Secs(MAX_AUTO_BOOT_SECS).step(1), AutoBoot::Never);
        assert_eq!(AutoBoot::Never.step(-1), AutoBoot::Secs(MAX_AUTO_BOOT_SECS));
    }

    #[test]
    fn step_clamps_at_both_ends() {
        assert_eq!(AutoBoot::Instant.step(-1), AutoBoot::Instant);
        assert_eq!(AutoBoot::Instant.step(-100), AutoBoot::Instant);
        assert_eq!(AutoBoot::Never.step(1), AutoBoot::Never);
        assert_eq!(AutoBoot::Never.step(100), AutoBoot::Never);
    }

    #[test]
    fn step_handles_multi_detent_jumps() {
        assert_eq!(AutoBoot::Instant.step(5), AutoBoot::Secs(5));
        assert_eq!(AutoBoot::Secs(10).step(-4), AutoBoot::Secs(6));
    }

    // ---- label --------------------------------------------------------------

    #[test]
    fn labels_render_for_the_whole_dial() {
        let mut buf = [0u8; AUTO_BOOT_LABEL_MAX];
        assert_eq!(AutoBoot::Instant.label(&mut buf), b"INSTANT");
        assert_eq!(AutoBoot::Never.label(&mut buf), b"NEVER");
        assert_eq!(AutoBoot::Secs(1).label(&mut buf), b"1S");
        assert_eq!(AutoBoot::Secs(9).label(&mut buf), b"9S");
        // Two digits — the 10..=20 range the OLED title bar also has to handle.
        assert_eq!(AutoBoot::Secs(10).label(&mut buf), b"10S");
        assert_eq!(AutoBoot::Secs(20).label(&mut buf), b"20S");
    }

    #[test]
    fn every_label_fits_the_buffer() {
        // AUTO_BOOT_LABEL_MAX is what callers stack-allocate; nothing may exceed it.
        let mut buf = [0u8; AUTO_BOOT_LABEL_MAX];
        let mut all = vec![AutoBoot::Instant, AutoBoot::Never];
        all.extend((1..=MAX_AUTO_BOOT_SECS).map(AutoBoot::Secs));
        for ab in all {
            assert!(ab.label(&mut buf).len() <= AUTO_BOOT_LABEL_MAX, "{ab:?}");
        }
    }

    // ---- boot_mode ----------------------------------------------------------

    /// Settings with everything at its default except the auto-boot dial.
    fn cfg(auto_boot: AutoBoot, dev_mode: bool) -> Settings {
        Settings {
            dev_mode,
            auto_boot,
        }
    }

    #[test]
    fn boot_mode_honours_the_dial() {
        assert_eq!(
            boot_mode(&cfg(AutoBoot::Instant, false), 1, false, true),
            BootMode::Instant
        );
        assert_eq!(
            boot_mode(&cfg(AutoBoot::Secs(5), false), 1, false, true),
            BootMode::Countdown(5)
        );
        assert_eq!(
            boot_mode(&cfg(AutoBoot::Secs(20), false), 3, false, true),
            BootMode::Countdown(20)
        );
        assert_eq!(
            boot_mode(&cfg(AutoBoot::Never, false), 1, false, true),
            BootMode::Wait
        );
    }

    #[test]
    fn recovery_beats_every_setting() {
        // The whole point of the gesture: an INSTANT unit must still reach the menu.
        for auto_boot in [AutoBoot::Instant, AutoBoot::Secs(5), AutoBoot::Never] {
            assert_eq!(
                boot_mode(&cfg(auto_boot, false), 1, true, true),
                BootMode::Wait,
                "{auto_boot:?}",
            );
        }
    }

    #[test]
    fn dev_mode_never_auto_boots() {
        assert_eq!(
            boot_mode(&cfg(AutoBoot::Instant, true), 1, false, true),
            BootMode::Wait
        );
        assert_eq!(
            boot_mode(&cfg(AutoBoot::Secs(5), true), 1, false, true),
            BootMode::Wait
        );
    }

    #[test]
    fn no_boot_targets_never_auto_boots() {
        // Nothing to launch — the menu is all there is.
        assert_eq!(
            boot_mode(&cfg(AutoBoot::Instant, false), 0, false, true),
            BootMode::Wait
        );
        assert_eq!(
            boot_mode(&cfg(AutoBoot::Secs(5), false), 0, false, true),
            BootMode::Wait
        );
    }

    #[test]
    fn later_menu_passes_never_auto_boot() {
        // Returning from DATA TRANSFER must not launch the firmware out from
        // under the user, nor restart the countdown behind their back.
        assert_eq!(
            boot_mode(&cfg(AutoBoot::Instant, false), 1, false, false),
            BootMode::Wait
        );
        assert_eq!(
            boot_mode(&cfg(AutoBoot::Secs(5), false), 1, false, false),
            BootMode::Wait
        );
    }
}
```

Note: the closing `}` above is the existing `mod tests` closing brace — do not add a second one.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p deluge-image --target x86_64-unknown-linux-gnu settings`
Expected: FAIL — `no method named step found`, `cannot find function boot_mode`, `cannot find value AUTO_BOOT_LABEL_MAX`.

- [ ] **Step 3: Implement the dial, the label, and the decision**

In `crates/deluge-image/src/settings.rs`, add these two methods to the existing `impl AutoBoot` block (after `from_byte`):

```rust
    /// Move along the dial by `detents` (positive = wait longer), clamped at both
    /// ends: `Instant → 1S → … → 20S → Never`.
    pub fn step(self, detents: i8) -> Self {
        let pos = i16::from(self.to_byte()) + i16::from(detents);
        Self::from_byte(pos.clamp(0, i16::from(NEVER_BYTE)) as u8)
    }

    /// Render the setting as an OLED label (`INSTANT`, `5S`, `20S`, `NEVER`) into
    /// `buf`, returning the used slice.
    pub fn label(self, buf: &mut [u8; AUTO_BOOT_LABEL_MAX]) -> &[u8] {
        match self {
            AutoBoot::Instant => {
                buf[..7].copy_from_slice(b"INSTANT");
                &buf[..7]
            }
            AutoBoot::Never => {
                buf[..5].copy_from_slice(b"NEVER");
                &buf[..5]
            }
            AutoBoot::Secs(n) => {
                let n = n.clamp(1, MAX_AUTO_BOOT_SECS);
                let mut i = 0;
                if n >= 10 {
                    buf[i] = b'0' + n / 10;
                    i += 1;
                }
                buf[i] = b'0' + n % 10;
                i += 1;
                buf[i] = b'S';
                i += 1;
                &buf[..i]
            }
        }
    }
```

Then add, after the `impl AutoBoot` block:

```rust
/// Longest label [`AutoBoot::label`] can produce (`INSTANT`).  Callers stack-
/// allocate a buffer of this size.
pub const AUTO_BOOT_LABEL_MAX: usize = 7;

/// What the loader should do once it has probed the flash slot and the SD card.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BootMode {
    /// Skip the menu entirely and launch the default entry.
    Instant,
    /// Draw the menu, auto-booting the default entry after `n` seconds.
    Countdown(u8),
    /// Draw the menu and wait for a selection.
    Wait,
}

/// Decide how to boot.
///
/// * `boot_total` — the number of real boot targets (the flash image, if any,
///   plus the SD `/APPS` entries).  The synthetic `DATA TRANSFER` / `SETTINGS`
///   entries do not count.
/// * `recovery` — SELECT was pressed or held between power-on and this decision.
/// * `auto_boot_allowed` — this is the *first* pass of the boot loop.  Later
///   passes (returning from `DATA TRANSFER`, from the settings screen, or from a
///   flash write) are user-driven: the loader must not launch behind their back,
///   which on an `Instant` unit would fire the moment they pressed BACK.
///
/// Precedence is first-match-wins, in the order written.
pub fn boot_mode(
    cfg: &Settings,
    boot_total: usize,
    recovery: bool,
    auto_boot_allowed: bool,
) -> BootMode {
    if recovery || cfg.dev_mode || boot_total == 0 || !auto_boot_allowed {
        return BootMode::Wait;
    }
    match cfg.auto_boot {
        AutoBoot::Instant => BootMode::Instant,
        AutoBoot::Secs(n) => BootMode::Countdown(n.clamp(1, MAX_AUTO_BOOT_SECS)),
        AutoBoot::Never => BootMode::Wait,
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p deluge-image --target x86_64-unknown-linux-gnu settings`
Expected: PASS — 17 tests total in the module.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-image/src/settings.rs
git commit -m "feat(settings): auto-boot dial, OLED label, and boot_mode decision"
```

---

### Task 3: Wire the boot loop — recovery gesture and boot-mode dispatch

After this task the setting is *honoured* (a v1 record still gives 5 s, so behaviour is unchanged on existing units) and the recovery gesture works. Editing the value comes in Task 4.

**Files:**
- Modify: `app-loader/src/main.rs` — `BOOT_COUNTDOWN_SECS` (line 28-30), `SELECT_SEEN` static (near `BACK_PRESSED`, line 44), `pic_rx_task` (line 148-154), `boot_task` (lines 305-435)
- Modify: `app-loader/src/ui.rs` — `run_selector` (line 198-302)

**Interfaces:**
- Consumes: `deluge_image::settings::{BootMode, boot_mode}` (Task 2), re-exported through `app-loader/src/settings.rs`.
- Produces: `pub(crate) static SELECT_SEEN: AtomicBool` in `main.rs`.

- [ ] **Step 1: Re-export the new items from the app-loader settings wrapper**

In `app-loader/src/settings.rs`, extend the existing `pub use` (line 18):

```rust
pub use deluge_image::settings::{
    AUTO_BOOT_LABEL_MAX, AutoBoot, BootMode, RECORD_LEN, Settings, boot_mode, decode, encode,
};
```

- [ ] **Step 2: Delete the hardcoded countdown and add the recovery latch**

In `app-loader/src/main.rs`, delete the `BOOT_COUNTDOWN_SECS` const (lines 28-30) entirely.

Next to the existing `BACK_PRESSED` static (line 44), add:

```rust
/// Latched by `pic_rx_task` on the first SELECT press — including a SELECT held
/// down at power-on, which the PIC reports in response to the
/// `CMD_RESEND_BUTTON_STATES` that `pic::init` sends.
///
/// `boot_task` samples this once, at the boot decision, as the **recovery
/// gesture**: it forces the boot menu no matter what the persisted auto-boot
/// setting says.  A latch rather than a level check, so the gesture is forgiving
/// — any SELECT press between reset and the decision counts, and the user does
/// not have to guess when the loader looks.  Without it, a unit set to
/// `AUTO-BOOT: INSTANT` with broken firmware in its flash slot would be
/// unrecoverable.
pub(crate) static SELECT_SEEN: AtomicBool = AtomicBool::new(false);
```

- [ ] **Step 3: Set the latch in `pic_rx_task`**

In `app-loader/src/main.rs`, change the SELECT press arm (lines 148-151) to also set the latch:

```rust
            // Track the SELECT button held-state so the selector can tell a
            // short tap (confirm) from a long-press (write-to-flash), and latch
            // the first press as the recovery gesture (see SELECT_SEEN).
            Some(Event::ButtonPress { id }) if id == controls::encoder_button::SELECT => {
                ui::SELECT_DOWN.store(true, Ordering::Release);
                crate::SELECT_SEEN.store(true, Ordering::Release);
            }
```

- [ ] **Step 4: Sample the gesture and show the RECOVERY splash**

In `app-loader/src/main.rs`, in `boot_task`, immediately after `oled::init().await; info!("OLED: ready");` (line 305-306) and **before** `let mut first_pass = true;`, insert:

```rust
    // Recovery gesture: SELECT pressed or held at any point between reset and
    // here (see SELECT_SEEN).  Sampled once — later menu passes are user-driven
    // and must not re-trigger it.  The splash matters most on an INSTANT unit,
    // which otherwise gives no sign that the *hold* — rather than a failed boot —
    // is why the menu appeared.  Nothing is persisted: the user's setting stands.
    let recovery = SELECT_SEEN.load(core::sync::atomic::Ordering::Acquire);
    if recovery {
        info!("Recovery: SELECT held at boot — forcing the boot menu");
        ui::show_message(b"RECOVERY", b"BOOT MENU").await;
        embassy_time::Timer::after(embassy_time::Duration::from_millis(700)).await;
    }

    // Auto-boot (countdown *or* instant) only ever fires on the first pass of the
    // loop below.  See `settings::boot_mode`.
    let mut auto_boot_allowed = true;
```

- [ ] **Step 5: Replace the countdown decision with the `boot_mode` dispatch**

In `app-loader/src/main.rs`, replace the countdown block (lines 401-435 — from the `// Countdown auto-boots the default only when...` comment through the end of the `let selection = ...` expression) with:

```rust
        // How to boot: skip the menu entirely (INSTANT), run it with a countdown,
        // or sit on it.  All the precedence lives in the host-tested decision fn.
        let mode = settings::boot_mode(&cfg, boot_total, recovery, auto_boot_allowed);
        // Every pass after this one is user-driven — never auto-boot again.
        auto_boot_allowed = false;
        info!("Boot mode: {:?}", mode);

        let selection = if mode == settings::BootMode::Instant {
            // The menu is never drawn: launch the default entry (index 0 — the
            // flash image when present, else the first SD app).  `boot_mode`
            // guarantees `boot_total > 0` here, so index 0 is a real boot target.
            info!("Auto-boot: INSTANT — launching the default entry");
            ui::Selection {
                index: 0,
                long_press: false,
            }
        } else {
            let countdown = match mode {
                settings::BootMode::Countdown(secs) => secs,
                // Wait: `run_selector` treats 0 as "no countdown, wait for a pick".
                _ => 0,
            };

            // In dev mode, race the menu selector against the background USB upload
            // listener: whichever resolves first wins.  The listener only ever
            // "returns" by loading and launching a received image (it never hands a
            // value back), so the menu branch is the only one that yields a selection
            // here.  Outside dev mode, just run the selector.
            //
            // Crucially, bring the USB device up *before* `run_selector` starts
            // drawing: USB bring-up reconfigures interrupts/clocks, and doing that
            // while an OLED frame DMA + PIC handshake is in flight can wedge the
            // display so the menu never redraws (the proven `usbmsc` path builds USB
            // before starting its OLED loop for the same reason).
            if cfg.dev_mode {
                use embassy_futures::select::{Either, select};
                let listener = devupload::prepare();
                match select(
                    ui::run_selector(&name_refs[..menu_total], 0, countdown),
                    listener.run(),
                )
                .await
                {
                    Either::First(sel) => sel,
                    Either::Second(never) => never,
                }
            } else {
                ui::run_selector(&name_refs[..menu_total], 0, countdown).await
            }
        };
```

The `let ui::Selection { index: selected, long_press } = selection;` destructuring on the next line is unchanged.

- [ ] **Step 6: Fix the SELECT-arming bug in `run_selector`**

A SELECT still held from the recovery gesture is currently read as a *fresh* press, so it crosses the 700 ms long-press threshold and pops the write-to-flash confirmation. `run_selector` must see a release first.

In `app-loader/src/ui.rs`, after `let mut press_at: Option<Instant> = None;` (line 213), add:

```rust
    // A SELECT press held from before the selector opened — the recovery gesture,
    // or the press that confirmed a previous screen — is not a new press.  Wait
    // for the release before acting on the button at all; otherwise the hold
    // crosses the long-press threshold below and fires a write-to-flash prompt.
    let mut armed = !SELECT_DOWN.load(Ordering::Acquire);
```

Then replace the SELECT button edge block (lines 267-292) with:

```rust
        // SELECT button edge handling (state pumped by pic_rx_task in main.rs).
        let down = SELECT_DOWN.load(Ordering::Acquire);
        if !armed {
            // Still letting go of a press that predates this screen.
            armed = !down;
        } else {
            match (press_at, down) {
                (None, true) => {
                    // Rising edge: a press began. Any press cancels the countdown.
                    countdown_active = false;
                    press_at = Some(Instant::now());
                }
                (Some(at), true) => {
                    // Still held — fire as soon as it becomes a long-press.
                    if at.elapsed() >= Duration::from_millis(LONG_PRESS_MS) {
                        return Selection {
                            index: cursor,
                            long_press: true,
                        };
                    }
                }
                (Some(_), false) => {
                    // Falling edge before the threshold: a short tap = confirm.
                    return Selection {
                        index: cursor,
                        long_press: false,
                    };
                }
                (None, false) => {}
            }
        }
```

- [ ] **Step 7: Build**

Run: `cargo build-app-loader`
Expected: PASS, no warnings about unused items (`BOOT_COUNTDOWN_SECS` is gone).

- [ ] **Step 8: Verify on hardware**

Flash the loader (`cargo build-app-loader-bin`, then flash `target/armv7a-none-eabihf/release/app-loader.bin` as usual) onto a unit whose settings sector holds an existing v1 record.

Expected:
- Normal power-on: the menu appears with a **5 s** countdown and auto-boots — unchanged from before, proving the v1 migration.
- Power-on **holding SELECT**: the `RECOVERY / BOOT MENU` splash appears for ~0.7 s, then the menu with **no countdown**, waiting indefinitely. Releasing SELECT and turning the encoder moves the cursor normally; **no write-to-flash prompt appears** (the arming fix).
- Dev mode, if it was on, is still on.

- [ ] **Step 9: Commit**

```bash
git add app-loader/src/main.rs app-loader/src/ui.rs app-loader/src/settings.rs
git commit -m "feat(app-loader): honour the auto-boot setting + hold-SELECT recovery gesture"
```

---

### Task 4: The `SETTINGS` screen

**Files:**
- Modify: `app-loader/src/ui.rs` — `render` (line 73-114), `run_selector` (line 198), new `run_settings`
- Modify: `app-loader/src/main.rs` — menu labels (lines 33-37), menu build (lines 376-394), the dev-mode toggle block (lines 477-518)

**Interfaces:**
- Consumes: `settings::{AutoBoot, Settings, AUTO_BOOT_LABEL_MAX}` (Tasks 1-2); `crate::BACK_PRESSED`; `ui::SELECT_DOWN`.
- Produces: `pub async fn ui::run_settings(cfg: &mut Settings)` — edits `cfg` in place and returns when the user leaves the screen. It performs **no** flash I/O; the caller compares against the settings it read and writes once if they differ.

- [ ] **Step 1: Give `render` a title, and wrap it in `run_menu`**

In `app-loader/src/ui.rs`, change `render`'s signature and its title line (lines 73-88). Replace:

```rust
fn render(
    fb: &mut FrameBuffer,
    entries: &[&[u8]],
    scroll: usize,
    cursor: usize,
    countdown: Option<u8>,
) {
    fb.fill(0x00);

    // Title bar — show the countdown while it is running, otherwise the label.
    let mut cd_buf = [0u8; 12];
    let title: &[u8] = match countdown {
        Some(secs) => countdown_title(&mut cd_buf, secs),
        None => b"SELECT APP",
    };
    draw_str(fb, 4, TITLE_ROW, title);
```

with:

```rust
fn render(
    fb: &mut FrameBuffer,
    title: &[u8],
    entries: &[&[u8]],
    scroll: usize,
    cursor: usize,
    countdown: Option<u8>,
) {
    fb.fill(0x00);

    // Title bar — show the countdown while it is running, otherwise the label.
    let mut cd_buf = [0u8; 12];
    let title: &[u8] = match countdown {
        Some(secs) => countdown_title(&mut cd_buf, secs),
        None => title,
    };
    draw_str(fb, 4, TITLE_ROW, title);
```

Also update `render`'s doc-comment (line 67-72) to document the new parameter:

```rust
/// Render a frame showing the selector list.
///
/// * `title`    — title-bar text, shown when no countdown is running
/// * `entries`  — full sorted list of entry names (full `BASE.EXT` filenames)
/// * `scroll`   — index of the first visible entry
/// * `cursor`   — index of the highlighted entry (absolute, not relative)
/// * `countdown`— `Some(secs_remaining)` shows a boot countdown in the title bar,
///                replacing `title`.
```

Then in `run_selector`, update the one call site (line 238):

```rust
        render(&mut fb, b"SELECT APP", entries, scroll, cursor, remaining);
```

- [ ] **Step 2: Build to confirm the refactor is clean**

Run: `cargo build-app-loader`
Expected: PASS.

- [ ] **Step 3: Write the settings screen**

In `app-loader/src/ui.rs`, add at the end of the file:

```rust
// ── Settings screen ───────────────────────────────────────────────────────────

/// Entry rows on the settings screen.
const SET_AUTO_BOOT: usize = 0;
const SET_DEV_MODE: usize = 1;
const SET_BACK: usize = 2;
const SET_ROWS: usize = 3;

/// Longest settings row: `AUTO-BOOT: <INSTANT>` — 20 chars, which at the font's
/// 6 px advance is exactly the 120 px available from `x = 8` to the right edge.
const SET_LINE_MAX: usize = 20;

/// Format the auto-boot row: `AUTO-BOOT: 5S`, or `AUTO-BOOT: <5S>` while the
/// value is being edited (the brackets are the only cue that the encoder now
/// changes the value instead of moving the cursor).
fn auto_boot_line<'a>(
    buf: &'a mut [u8; SET_LINE_MAX],
    auto_boot: crate::settings::AutoBoot,
    editing: bool,
) -> &'a [u8] {
    const PREFIX: &[u8] = b"AUTO-BOOT: ";
    let mut n = 0;
    for &b in PREFIX {
        buf[n] = b;
        n += 1;
    }
    if editing {
        buf[n] = b'<';
        n += 1;
    }
    let mut val = [0u8; crate::settings::AUTO_BOOT_LABEL_MAX];
    for &b in auto_boot.label(&mut val) {
        buf[n] = b;
        n += 1;
    }
    if editing {
        buf[n] = b'>';
        n += 1;
    }
    &buf[..n]
}

/// The `SETTINGS` screen: edit `cfg` in place and return when the user leaves.
///
/// Does **no** flash I/O — the caller compares the result against what it read
/// and writes once, so a session of encoder turns costs one erase/program rather
/// than one per keypress.
///
/// * SELECT on `AUTO-BOOT` enters an in-place edit: the encoder walks
///   `INSTANT → 1S … 20S → NEVER`, SELECT confirms, BACK restores the value the
///   edit started from.
/// * SELECT on `DEV MODE` flips the flag.
/// * SELECT on `BACK`, or the BACK button, leaves.
pub async fn run_settings(cfg: &mut crate::settings::Settings) {
    use embassy_time::{Duration, Timer};

    const ENC: usize = deluge_bsp::controls::encoder::SELECT as usize;
    let mut edge_acc: i8 = 0;
    let mut cursor = SET_AUTO_BOOT;
    let mut editing = false;
    // The value the current edit began at, restored if the user backs out.
    let mut pre_edit = cfg.auto_boot;

    // The SELECT press that opened this screen is still held; don't read it as a
    // press *on* this screen (same reason as `run_selector`'s `armed`).
    let mut armed = !SELECT_DOWN.load(Ordering::Acquire);
    let mut select_prev = false;

    // A BACK latched by a previous mode (e.g. leaving DATA TRANSFER) must not
    // immediately dismiss this screen.
    crate::BACK_PRESSED.store(false, Ordering::Release);

    loop {
        // ---- draw ----
        let mut line = [0u8; SET_LINE_MAX];
        let rows: [&[u8]; SET_ROWS] = [
            auto_boot_line(&mut line, cfg.auto_boot, editing),
            if cfg.dev_mode {
                b"DEV MODE: ON"
            } else {
                b"DEV MODE: OFF"
            },
            b"BACK",
        ];

        let mut fb = FrameBuffer::new();
        render(&mut fb, b"SETTINGS", &rows, 0, cursor, None);
        oled::send_frame(&fb).await;

        // Poll at ~60 Hz, like the selector.
        Timer::after(Duration::from_millis(16)).await;

        // ---- encoder: adjust the value while editing, else move the cursor ----
        let detents = deluge_bsp::encoder::take_detents(ENC, &mut edge_acc);
        if detents != 0 {
            if editing {
                cfg.auto_boot = cfg.auto_boot.step(detents);
            } else if detents > 0 {
                cursor = (cursor + 1).min(SET_BACK);
            } else {
                cursor = cursor.saturating_sub(1);
            }
        }

        // ---- BACK button: cancel an edit, or leave the screen ----
        if crate::BACK_PRESSED.swap(false, Ordering::AcqRel) {
            if editing {
                cfg.auto_boot = pre_edit;
                editing = false;
            } else {
                return;
            }
        }

        // ---- SELECT: rising edge only ----
        let down = SELECT_DOWN.load(Ordering::Acquire);
        if !armed {
            armed = !down;
            select_prev = down;
            continue;
        }
        let pressed = down && !select_prev;
        select_prev = down;
        if !pressed {
            continue;
        }

        if editing {
            // Confirm — the value is already live in `cfg`.
            editing = false;
        } else {
            match cursor {
                SET_AUTO_BOOT => {
                    pre_edit = cfg.auto_boot;
                    editing = true;
                }
                SET_DEV_MODE => cfg.dev_mode = !cfg.dev_mode,
                _ => return, // SET_BACK
            }
        }
    }
}
```

Note the `rows` array borrows `line` mutably for the frame; it is rebuilt each iteration, which is why `line` is declared inside the loop.

- [ ] **Step 4: Replace the root menu's DEV MODE entry with SETTINGS**

In `app-loader/src/main.rs`, replace the label consts (lines 35-37):

```rust
/// Label for the synthetic entry that opens the settings screen.
const SETTINGS_MENU_LABEL: &[u8] = b"SETTINGS";
```

In the menu build, rename `dev_idx` to `settings_idx` (lines 377, 390-394):

```rust
        let data_idx = boot_total; // DATA TRANSFER follows the boot targets
        let settings_idx = boot_total + 1; // SETTINGS follows DATA TRANSFER
        let menu_total = boot_total + 2;
```

and:

```rust
        name_refs[data_idx] = DATA_MENU_LABEL;
        name_refs[settings_idx] = SETTINGS_MENU_LABEL;
```

The `is_sd_entry` check (line 451) still reads `selected < data_idx`, which remains correct.

- [ ] **Step 5: Replace the dev-mode toggle block with the settings screen**

In `app-loader/src/main.rs`, replace the whole `if selected == dev_idx { ... }` block (lines 477-518) with:

```rust
        // ---- SETTINGS screen (persists on exit, rebuilds the menu) ----
        // The screen edits a copy; we write flash once, only if something
        // actually changed, then `continue` so the next pass re-reads it (updating
        // the auto-boot mode and whether the USB listener runs).  Nothing is
        // launched.
        if selected == settings_idx {
            // Close FAT handles before touching the flash bus (the settings write
            // leaves memory-mapped read mode, like the app-slot store).
            if let Some((volume, _, entries)) = sd_listing {
                let _ = vm.close_volume(volume);
                drop(entries);
            }
            drop(vm);

            let mut new_cfg = cfg;
            ui::run_settings(&mut new_cfg).await;
            if new_cfg == cfg {
                continue;
            }
            info!("Settings: {:?} -> {:?}", cfg, new_cfg);

            let ok = unsafe { settings::write(&new_cfg).await };
            if ok {
                ui::show_message(b"SETTINGS", b"SAVED").await;
                embassy_time::Timer::after(embassy_time::Duration::from_millis(700)).await;
            } else {
                // The flash write didn't stick (the device stays responsive
                // thanks to the bounded SPIBSC waits). Show the JEDEC ID and
                // status register so the failure can be diagnosed: ID `01 02 20`
                // confirms manual-mode works; status `BP[2:0]` (bits 2-4) set
                // means the settings sector is write-protected.
                let id = rza1l_hal::spibsc::read_id();
                let sr = rza1l_hal::spibsc::read_status_reg();
                error!(
                    "Settings flash write failed: JEDEC={:02x} {:02x} {:02x}, SR={:#04x}",
                    id[0], id[1], id[2], sr
                );
                let mut line = *b"ID...... SR..";
                hex2(&mut line[2..4], id[0]);
                hex2(&mut line[4..6], id[1]);
                hex2(&mut line[6..8], id[2]);
                hex2(&mut line[11..13], sr);
                ui::show_message(b"FLASH WRITE FAIL", &line).await;
                embassy_time::Timer::after(embassy_time::Duration::from_secs(4)).await;
            }
            continue;
        }
```

- [ ] **Step 6: Build**

Run: `cargo build-app-loader`
Expected: PASS. If `DEV_MODE_ON_LABEL` / `DEV_MODE_OFF_LABEL` produce dead-code warnings, they were not deleted in Step 4 — delete them.

- [ ] **Step 7: Verify on hardware**

Flash and check the whole feature:

- The root menu's last entry is `SETTINGS`; selecting it opens the screen with `AUTO-BOOT: 5S`, `DEV MODE: OFF/ON`, `BACK`.
- SELECT on `AUTO-BOOT` shows `AUTO-BOOT: <5S>`; the encoder walks down to `<INSTANT>` and up through `<20S>` to `<NEVER>`, clamping at both ends. SELECT confirms; BACK instead restores the value you started from.
- `BACK` leaves the screen; `SETTINGS / SAVED` appears only when something changed, and the menu rebuilds with the new countdown.
- Set `INSTANT`, power-cycle: the unit boots straight to the flash firmware with no menu. Power-cycle **holding SELECT**: `RECOVERY` → the menu.
- Set `NEVER`: the menu sits indefinitely, and no USB device enumerates on the host (that is the difference from dev mode).
- Set `20S`: the title bar counts down `BOOT IN 20S` … `1S` with two digits rendering correctly.
- From the menu, enter `DATA TRANSFER`, press BACK: the menu returns and does **not** auto-boot or restart a countdown.
- Toggle `DEV MODE: ON` in the settings screen, exit, and confirm `cargo deluge run` still uploads.

- [ ] **Step 8: Commit**

```bash
git add app-loader/src/main.rs app-loader/src/ui.rs
git commit -m "feat(app-loader): SETTINGS screen with the auto-boot dial and dev-mode toggle"
```

---

### Task 5: Documentation

**Files:**
- Modify: `docs/app-loader.md` — boot sequence (line 87-88), menu diagram (lines 99-118), controls table (lines 122-136), synthetic entries (lines 154-164), dev mode (line 177)
- Modify: `app-loader/README.md` — lines 23, 32, 49
- Modify: `docs/getting-started.md` — lines 47-48, 425

- [ ] **Step 1: Update `docs/app-loader.md`**

In the boot-sequence list, item 4 currently says "shows it on the OLED with a 5-second auto-boot countdown of the default entry." Replace with:

```markdown
4. **Builds the boot menu** and shows it on the OLED, auto-booting the default
   entry after the configured delay (5 seconds out of the box — see
   [`SETTINGS`](#settings--auto-boot-and-dev-mode)).
```

In the menu diagram, replace the `DATA TRANSFER` comment line and add `SETTINGS`:

```text
┌───────────────────────────────┐
│ BOOT IN 5s                  ▓ │   ← title bar (countdown, then "SELECT APP")
│ ──────────────────────────────│
│ ▶ BOOT FLASH                ▓ │   ← default entry (on-flash firmware)
│   MYSYNTH.ELF               ░ │   ← SD /APPS images
│   SEQUENCER.ELF             ░ │
│   DATA TRANSFER             ░ │   ← synthetic entries (always present)
│   SETTINGS                  ░ │
└───────────────────────────────┘
     scrollbar on the right edge ┘
```

Replace list item 4 under "The menu lists, in order:" with:

```markdown
4. **`SETTINGS`** — a synthetic entry opening the settings screen (auto-boot
   delay and dev mode).
```

Extend the controls table:

```markdown
| Action | Control |
|--------|---------|
| Move the cursor | Turn the **SELECT** encoder |
| Launch the highlighted entry | **Short-press** SELECT |
| Store an SD app to flash | **Long-press** SELECT (hold ≥ 0.7 s) on an SD entry |
| Cancel the auto-boot countdown | Turn the encoder (any movement) |
| Exit a USB mode back to the menu | Press **BACK** |
| **Force the boot menu (recovery)** | **Hold SELECT while powering on** |
```

Replace the paragraph beginning "The countdown auto-boots the default entry after **5 seconds**." with:

```markdown
The countdown auto-boots the default entry after the delay set in
[`SETTINGS`](#settings--auto-boot-and-dev-mode) — **5 seconds** out of the box.
Turning the encoder cancels it and hands control to you indefinitely. It does not
run when there is no real boot target, when dev mode is on, or once you have
already been round the menu once this session (returning from `DATA TRANSFER`
will not boot out from under you).

### Recovery — always reach the menu

**Hold the SELECT encoder button down while powering the unit on.** The loader
shows `RECOVERY`, then the boot menu with no countdown, waiting indefinitely —
whatever the auto-boot setting says.

This is the way back from an `AUTO-BOOT: INSTANT` unit whose flashed firmware is
broken: the loader runs before that firmware ever does, so the menu (and with it
`DATA TRANSFER`, re-flashing from SD, and `SETTINGS`) is always reachable.
Nothing is persisted — your auto-boot setting is left exactly as you set it.
```

Replace the `### DEV MODE — toggle USB upload` section with:

```markdown
### `SETTINGS` — auto-boot and dev mode

Selecting **`SETTINGS`** opens a screen with three rows:

```text
SETTINGS
▶ AUTO-BOOT: 5S
  DEV MODE: OFF
  BACK
```

- **`AUTO-BOOT`** — how long the menu waits before launching the default entry.
  Press SELECT to edit (the value is bracketed, `<5S>`, while live), turn the
  encoder to walk the dial, press SELECT to confirm, or BACK to cancel the edit.

  | Value | Behaviour |
  |-------|-----------|
  | `INSTANT` | The menu is **never drawn** — the default entry launches at once. Hold SELECT at power-on to get the menu back. |
  | `1S` … `20S` | The menu is shown and counts down, then boots the default entry. |
  | `NEVER` | The menu is shown and waits indefinitely. Dev mode's menu behaviour, without dev mode's USB listener. |

- **`DEV MODE`** — press SELECT to flip the persistent dev-mode flag. See
  [Dev mode](#dev-mode) below.

- **`BACK`** — leave the screen. Settings are written to the flash settings
  sector **on exit**, and only if you changed something; `SETTINGS SAVED`
  confirms, and the menu rebuilds with the new values. **Nothing is launched.**

If the flash write does not stick, the loader stays responsive and shows a
diagnostic line with the chip's JEDEC ID and status register (e.g.
`ID 01 02 20 SR..`) so a write-protected settings sector can be diagnosed.
```

In the "Dev mode" section, the bullet "**disables the auto-boot countdown**, so the unit waits indefinitely on the menu…" stays accurate — but append a sentence:

```markdown
- **disables the auto-boot countdown**, so the unit waits indefinitely on the
  menu for either a menu selection or a USB upload. (This overrides `AUTO-BOOT`
  entirely, including `INSTANT`. If you want a menu that waits but no USB
  listener, set `AUTO-BOOT: NEVER` instead.)
```

- [ ] **Step 2: Update `app-loader/README.md`**

Line 23 references `BOOT_COUNTDOWN_SECS` (5 s), which no longer exists. Replace that sentence with:

```markdown
   configurable auto-boot of the default entry (`AUTO-BOOT`, 5 s out of the box;
   see `SETTINGS` below). A valid on-flash
```

(keeping the rest of the sentence intact).

Line 32's `DEV MODE: ON / DEV MODE: OFF` menu bullet becomes:

```markdown
- **`SETTINGS`** — opens the settings screen: the `AUTO-BOOT` delay
  (`INSTANT` / `1S`–`20S` / `NEVER`) and the persistent dev-mode flag. Written to
  flash on exit.
- **Recovery** — hold **SELECT** while powering on to force the boot menu with no
  countdown, whatever `AUTO-BOOT` is set to.
```

- [ ] **Step 3: Update `docs/getting-started.md`**

Lines 47-48 tell the user to select `DEV MODE: OFF` on the boot menu. It now lives one level down:

```markdown
- **DEV MODE: ON** — required for `cargo deluge run`'s USB upload. On the boot
  menu, select **`SETTINGS`**, then press SELECT on **`DEV MODE: OFF`** to flip it
  to **`DEV MODE: ON`**, and choose **`BACK`** to save (persistent,
```

(keeping the remainder of the sentence intact).

Line 425's troubleshooting bullet ("**`run` can't find the Deluge**") is still correct — it only says DEV MODE must be ON — but check it does not tell the user *where* to find the toggle; if it does, point it at `SETTINGS`.

- [ ] **Step 4: Check the docs against the build**

Run: `grep -rn "BOOT_COUNTDOWN_SECS" docs/ app-loader/ crates/`
Expected: no matches.

Run: `grep -rn "DEV MODE: OFF" docs/ app-loader/README.md`
Expected: matches only where the settings screen is being described, never as a root-menu entry.

- [ ] **Step 5: Commit**

```bash
git add docs/app-loader.md app-loader/README.md docs/getting-started.md
git commit -m "docs: auto-boot setting, SETTINGS screen, and hold-SELECT recovery"
```

---

## Final verification

- [ ] Run: `cargo test -p deluge-image --target x86_64-unknown-linux-gnu`
      Expected: PASS, all settings tests green.
- [ ] Run: `cargo build-app-loader`
      Expected: PASS, no warnings.
- [ ] Run: `cargo clippy -p deluge-image --target x86_64-unknown-linux-gnu -- -D warnings`
      Expected: PASS.
- [ ] The on-hardware checklist in Task 4 Step 7 has been run end-to-end on a real unit, including the `INSTANT` + hold-SELECT recovery case.
