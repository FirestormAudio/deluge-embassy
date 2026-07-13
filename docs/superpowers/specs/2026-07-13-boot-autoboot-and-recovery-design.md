# Configurable auto-boot timer + hold-SELECT recovery

**Status:** design approved, ready for planning
**Date:** 2026-07-13
**Area:** `app-loader/` (second-stage bootloader), `crates/deluge-image/src/settings.rs`

## Problem

The app-loader's boot menu counts down for a hardcoded `BOOT_COUNTDOWN_SECS = 5`
before auto-booting the default entry. Users want that delay to be theirs to
choose: zero (skip the menu, boot straight to firmware), a few seconds, a long
20 s pause, or never (sit on the menu until told otherwise).

Making zero mean "no menu" creates a lockout hazard: a unit set to instant-boot
with broken firmware in its flash slot has no way back to the loader. So the
timer setting is only safe alongside a control that *unconditionally* reaches the
menu. Holding SELECT while powering on becomes that control.

## Requirements

1. The auto-boot delay is persistent, survives power-cycles, and is editable on
   the device with no host tooling.
2. Range: **instant** (no menu at all), **1–20 s** countdown, or **never**
   (menu waits indefinitely).
3. `NEVER` is dev mode's menu behaviour *without* dev mode: no USB upload
   listener, no persisted dev flag.
4. Holding SELECT from power-on always reaches the boot menu with the countdown
   suppressed, regardless of the persisted setting.
5. Units already in the field keep their current behaviour and their dev-mode
   flag across the upgrade.

## Design

### 1. Settings record — version 2

`deluge_image::settings::Settings` gains an auto-boot field:

```rust
pub enum AutoBoot {
    Instant,     // launch the default entry immediately; the menu is never drawn
    Secs(u8),    // 1..=20 — draw the menu, auto-boot the default on expiry
    Never,       // draw the menu, wait indefinitely
}

pub struct Settings {
    pub dev_mode: bool,
    pub auto_boot: AutoBoot,   // Default::default() == Secs(5)
}
```

On-flash layout (12-byte record, one 256 B page — unchanged except byte 6):

| Offset | Field      | Notes                                                    |
|--------|------------|----------------------------------------------------------|
| 0..4   | `magic`    | `b"DSET"`                                                 |
| 4      | `version`  | **`2`** (was `1`)                                         |
| 5      | `flags`    | bit 0 = `dev_mode`; other bits reserved (0)              |
| 6      | `auto_boot`| `0` = Instant, `1..=20` = seconds, `21` = Never          |
| 7      | reserved   | 0                                                         |
| 8..12  | `crc32`    | CRC-32 (IEEE) of bytes `0..8`, little-endian              |

The two reserved bytes could not simply be claimed: existing v1 records carry
`reserved == 0`, and `0` now means *instant boot*. Reinterpreting them would
silently delete the boot menu from every unit in the field on upgrade. Hence a
version bump with an explicit migration:

- **`decode` accepts version 1 and version 2.** A v1 record yields its
  `dev_mode` plus `auto_boot = Secs(5)` — precisely today's behaviour.
- **`encode` always emits version 2.** A v1 record is therefore rewritten as v2
  on the next settings write; until then it keeps decoding correctly.
- Byte 6 values **above 21 clamp to `Never`** rather than rejecting the record.
  Rejecting would discard a valid `dev_mode` alongside the bad byte, and `Never`
  is the safe direction — worst case the unit always shows its menu.

Constants `DEFAULT_AUTO_BOOT_SECS = 5`, `MAX_AUTO_BOOT_SECS = 20`, and the
sentinel `NEVER_BYTE = 21` live here, replacing `app-loader`'s
`BOOT_COUNTDOWN_SECS`.

### 2. Boot decision — one pure function

`run_selector`'s `countdown: u8` parameter currently overloads `0` to mean "wait
forever" (that's what dev mode passes). With `0` now meaning *instant*, the
decision has to become explicit. It moves out of `boot_task`'s inline `if` and
into a pure, host-testable function beside the record:

```rust
pub enum BootMode {
    Instant,        // skip the menu, launch the default entry
    Countdown(u8),  // draw the menu, auto-boot after n seconds
    Wait,           // draw the menu, wait for a selection
}

pub fn boot_mode(
    cfg: &Settings,
    boot_total: usize,       // real boot targets (flash image + SD /APPS entries)
    recovery: bool,          // SELECT was held/tapped during boot
    auto_boot_allowed: bool, // first pass of the boot loop only
) -> BootMode
```

Precedence, first match wins:

| Condition                | Result          | Why                                            |
|--------------------------|-----------------|------------------------------------------------|
| `recovery`               | `Wait`          | the recovery control must beat every setting   |
| `cfg.dev_mode`           | `Wait`          | unchanged: dev mode waits for a menu pick or an upload |
| `boot_total == 0`        | `Wait`          | nothing to auto-boot                           |
| `!auto_boot_allowed`     | `Wait`          | the user has already used the menu this session |
| `auto_boot == Instant`   | `Instant`       |                                                |
| `auto_boot == Secs(n)`   | `Countdown(n)`  |                                                |
| `auto_boot == Never`     | `Wait`          |                                                |

`boot_task` maps the result: `Instant` skips `run_selector` and falls into the
existing launch path with `index = 0`; `Countdown(n)` calls
`run_selector(.., n)`; `Wait` calls `run_selector(.., 0)`.

**`auto_boot_allowed` is a deliberate behaviour change.** It is `true` only on
the *first* pass of the boot loop. Today the loop re-runs the countdown on every
pass, so returning from `DATA TRANSFER` with BACK restarts a 5 s countdown and
auto-boots; on an instant-boot unit it would launch the firmware the moment you
pressed BACK. Once the user has interacted with the menu, the loader must not
boot behind their back.

### 3. Recovery hold

`pic_rx_task` already tracks SELECT press/release into `ui::SELECT_DOWN`. It
gains a latch — `SELECT_SEEN`, set on the first SELECT `ButtonPress` and never
cleared — which `boot_task` samples once, at the boot decision.

A latch rather than a level sample at one instant, because it makes the gesture
forgiving: any SELECT press between power-on and the boot decision counts, so the
user does not have to guess when the loader looks. `pic::init()` already sends
`CMD_RESEND_BUTTON_STATES`, so a button held from power-on is reported as a press
and the latch catches it.

On recovery: a ~700 ms `RECOVERY` splash confirms the gesture registered (without
it, an instant-boot unit gives no signal that the hold — rather than a failed
boot — is why the menu appeared), then the normal menu with no countdown.
**Nothing is persisted**; the setting the user chose is left alone.

**Bug this exposes:** `run_selector` treats `SELECT_DOWN == true` on entry as a
fresh rising edge, so a still-held recovery hold fires a 700 ms long-press and
pops the *write-to-flash* confirmation. `run_selector` must observe a release
before arming SELECT. The fix is correct independent of this feature.

### 4. `SETTINGS` submenu

`ui::render` currently hardcodes the `"SELECT APP"` title. It takes a title
parameter, and `run_selector` becomes a thin wrapper over a shared
`run_menu(title, entries, default_idx, countdown) -> Selection`. The settings
screen reuses it.

The root menu's trailing `DEV MODE: ON/OFF` entry is replaced by `SETTINGS`,
keeping the root list about booting:

```
  root                        SETTINGS
▶ BOOT FLASH                ▶ AUTO-BOOT: 5S
  MYAPP.ELF                   DEV MODE: OFF
  DATA TRANSFER               BACK
  SETTINGS
```

`run_settings(&mut Settings)` edits a working copy in place:

- SELECT on `DEV MODE` flips the copy's flag.
- SELECT on `AUTO-BOOT` enters an in-place edit state: the value renders
  bracketed (`AUTO-BOOT: <5S>`) to show it is live, the SELECT encoder walks the
  dial `INSTANT → 1S … 20S → NEVER` (clamped at both ends), SELECT confirms and
  BACK restores the pre-edit value.
- `BACK` — the entry or the button — leaves the screen.

The setting is named `AUTO-BOOT`, not `COUNTDOWN`: "COUNTDOWN: NEVER" and
"COUNTDOWN: INSTANT" both read as nonsense, whereas `AUTO-BOOT` reads correctly
across all three states.

Flash is written **once on exit**, and only when the working copy differs from
what was read — one erase/program covering both fields rather than one per
keypress. It reuses `settings::write` verbatim, **including** the existing
JEDEC-ID / status-register diagnostic screen on a failed write (today that is the
only way a write-protected settings sector surfaces). The boot loop then
`continue`s, rebuilding the menu against the new values exactly as the dev-mode
toggle does now.

`BACK_PRESSED` already exists as a latch for the USB modes. The settings screen
clears it on entry and on read, so a stale BACK from a previous mode cannot
immediately dismiss the screen.

## Testing

Host tests in `deluge-image` (`cargo test -p deluge-image`), where the existing
settings tests live:

- **Record:** v2 round-trips across `Instant`, every `Secs(1..=20)`, and `Never`.
  **A v1 record decodes with `dev_mode` preserved and `auto_boot == Secs(5)`** —
  the migration case that protects units in the field. Bytes above 21 clamp to
  `Never`. Corruption, blank `0xFF` flash, bad magic, and unknown versions still
  reject. `encode` always emits version 2.
- **`boot_mode`:** the full precedence table above — recovery beats everything,
  dev mode beats `Instant`, zero boot targets never auto-boots,
  `auto_boot_allowed == false` never auto-boots.

The OLED UI and the flash write cannot be host-tested. On-device checklist:

- Hold SELECT at power-on on an `INSTANT` unit → `RECOVERY` splash, then the menu
  (the lockout-proof case).
- `DATA TRANSFER` → BACK does **not** auto-launch.
- A 20 s countdown renders two digits (`countdown_title` already handles `>= 10`;
  this is a check, not a change).
- A unit carrying an existing v1 record keeps its dev-mode state after the
  upgrade, and shows `AUTO-BOOT: 5S`.
- `NEVER` sits on the menu indefinitely with no USB device enumerated.

## Risk

The recovery gesture assumes the PIC reports an *already-held* button in response
to `CMD_RESEND_BUTTON_STATES`. This follows from reading `pic::init`, but is
unverified on hardware. **First implementation step** is a throwaway RTT log of
the events arriving in the first 500 ms with SELECT held at power-on. If the PIC
does not report held buttons, the fallbacks are a button the PIC *does* report on
its first scan, or reading the button matrix directly — and that decision comes
back to the user rather than being guessed at.

## Files

- `crates/deluge-image/src/settings.rs` — `AutoBoot`, v2 record, v1 migration,
  `boot_mode`, host tests.
- `app-loader/src/main.rs` — drop `BOOT_COUNTDOWN_SECS`, `SELECT_SEEN` latch,
  `boot_mode` dispatch, `auto_boot_allowed`, `SETTINGS` entry replacing
  `DEV MODE`.
- `app-loader/src/ui.rs` — titled `run_menu`, `run_settings`, the auto-boot edit
  state, the `RECOVERY` splash, the SELECT-arming fix.
- `docs/app-loader.md` — menu structure, the countdown table (currently states a
  flat 5 s), new `SETTINGS` and `RECOVERY` sections.
- `docs/getting-started.md` — two places tell users to select `DEV MODE: OFF` on
  the boot menu; it now lives under `SETTINGS`.
