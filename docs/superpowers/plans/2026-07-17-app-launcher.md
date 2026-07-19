# App Launcher (Reference Linux Appliance) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the reference Linux appliance — a Rust app launcher that lists the executables in `/LINUX/APPS/` on the OLED, runs the one you pick with the select encoder, and always returns to the menu when you hold SHIFT+TRIPLETS+LEARN for ~1 second.

**Architecture:** Two new workspace members. `crates/deluge-linux-ui` is a reusable bridge: an `embedded-graphics` `DrawTarget` over `deluge-linux`'s 688-byte OLED framebuffer plus the named control ids. `examples/launcher` is the launcher binary: a state machine (`model`) driven by an `mpsc` channel fed by the SDK input thread and a child-reaper thread; a `supervisor` forks/execs the chosen app in its own process group; a `killwatch` pure state machine detects the escape chord; `ui` renders the list with the shared OLED toolkit. It runs as `/usr/bin/deluge-app` inside the existing `LINUX.ELF` runit service.

**Tech Stack:** Rust 2021, `deluge-linux` bindings, `deluge-ui-toolkit` + `embedded-graphics` 0.8 (from the sibling `~/GitHub/deluge-sdk` checkout), `libc` for `setsid`/`kill`, cross-compiled to `armv7-unknown-linux-musleabihf` with `crt-static` via `cargo xtask image`.

## Global Constraints

- **Spec:** `docs/superpowers/specs/2026-07-17-app-launcher-design.md`. Read it before starting.
- **OLED frame:** exactly `688` bytes = 128 wide × 43 tall, 1bpp, `line_length` 16 bytes/row, **row-major** (the Linux fb is linear; the kernel driver converts to SSD1309 pages). **No 5px offset** (unlike the embedded SDK).
- **Control ids** (raw PIC space `0..=35`, from `deluge-bsp::controls`): `LEARN=7`, `SHIFT=8`, `TRIPLETS=17`, select-encoder **click** button id `31`, select-encoder **rotation** encoder id `5`.
- **Event kinds** (from `deluge-linux`): `BUTTON=1`, `ENCODER=2` (`PAD=0`, `CLOCK=3` ignored). Button `value`: `1`=press, `0`=release. Encoder `value`: signed detent delta.
- **Kill chord:** SHIFT+TRIPLETS+LEARN held simultaneously for **~1000 ms**.
- **Apps dir:** `/LINUX/APPS/`; overridable via env `LAUNCHER_APPS_DIR` for host testing. App display name = filename, upper-cased. Only regular, executable files are listed.
- **On exit:** always return to the list. Clean exit (code 0) and user-kill return silently; abnormal exit shows a transient toast.
- **Static linking:** the shipped binary is `armv7-unknown-linux-musleabihf` + `RUSTFLAGS='-C target-feature=+crt-static'` (xtask handles this). Host builds/tests link the host libdeluge prefix.
- **Toolkit dependency:** path dependency to the sibling checkout `../../../deluge-sdk/crates/deluge-ui-toolkit` (both repos live under `~/GitHub/`).

## Prerequisites

- The sibling `~/GitHub/deluge-sdk` checkout is present (provides `deluge-ui-toolkit`, `deluge-fonts`).
- Host libdeluge is built so `cargo test -p …` can link (`deluge-sys` links it). If `cargo build` at the repo root already succeeds today, this is satisfied; otherwise build the host prefix first the way the repo already does for `crates/deluge-sys`.

---

### Task 1: Scaffold `deluge-linux-ui` crate with the `controls` module

**Files:**
- Create: `crates/deluge-linux-ui/Cargo.toml`
- Create: `crates/deluge-linux-ui/src/lib.rs`
- Create: `crates/deluge-linux-ui/src/controls.rs`
- Modify: `Cargo.toml` (workspace `members` + `default-members`)

**Interfaces:**
- Produces: module `deluge_linux_ui::controls` with `pub const LEARN: u8 = 7; SHIFT: u8 = 8; TRIPLETS: u8 = 17; SELECT_CLICK: u8 = 31; ENC_SELECT: u8 = 5;`

- [ ] **Step 1: Create the crate manifest**

`crates/deluge-linux-ui/Cargo.toml`:

```toml
[package]
name = "deluge-linux-ui"
version = "0.1.0"
edition = "2021"
description = "Bridge from deluge-linux's OLED framebuffer to the deluge-ui-toolkit (embedded-graphics)."

[dependencies]
deluge-linux = { path = "../deluge-linux" }
embedded-graphics = "0.8"
deluge-ui-toolkit = { path = "../../../deluge-sdk/crates/deluge-ui-toolkit" }
```

- [ ] **Step 2: Create the crate root**

`crates/deluge-linux-ui/src/lib.rs`:

```rust
//! Bridge between `deluge-linux` (Linux hardware bindings) and
//! `deluge-ui-toolkit` (the shared OLED UI toolkit).
//!
//! - [`OledTarget`] is an `embedded-graphics` `DrawTarget` over the 688-byte
//!   Linux OLED framebuffer; render toolkit widgets into it and `flush`.
//! - [`controls`] holds the named front-panel control ids the Linux SDK reports.

pub mod controls;
```

- [ ] **Step 3: Write the failing test for control ids**

`crates/deluge-linux-ui/src/controls.rs`:

```rust
//! Named front-panel control ids in the raw PIC button-id space (`0..=35`).
//!
//! Mirrors `deluge-bsp::controls` (in the `deluge-sdk` repo). Mirrored rather
//! than depended-on to keep the embedded `no_std` HAL out of a Linux binary.
//! The test below asserts the values match the documented mapping.

/// LEARN button.
pub const LEARN: u8 = 7;
/// SHIFT button.
pub const SHIFT: u8 = 8;
/// TRIPLETS VIEW button.
pub const TRIPLETS: u8 = 17;
/// Select-encoder shaft click (reported as a button press).
pub const SELECT_CLICK: u8 = 31;
/// Select-encoder rotation id (encoder space `0..=5`).
pub const ENC_SELECT: u8 = 5;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_ids_match_deluge_bsp() {
        // From deluge-bsp::controls (button::*, encoder_button::SELECT, encoder::SELECT).
        assert_eq!(LEARN, 7);
        assert_eq!(SHIFT, 8);
        assert_eq!(TRIPLETS, 17);
        assert_eq!(SELECT_CLICK, 31);
        assert_eq!(ENC_SELECT, 5);
        // Kill-chord ids are distinct.
        let chord = [SHIFT, TRIPLETS, LEARN];
        let mut seen = [false; 36];
        for id in chord {
            assert!(!seen[id as usize], "duplicate chord id {id}");
            seen[id as usize] = true;
        }
    }
}
```

- [ ] **Step 4: Wire the crate into the workspace**

In the root `Cargo.toml`, add `"crates/deluge-linux-ui"` to **both** `members` and `default-members` (it is host-buildable, like `deluge-sys`/`deluge-linux`):

```toml
[workspace]
members = ["crates/deluge-sys", "crates/deluge-linux", "crates/deluge-linux-ui", "xtask", "examples/rust-app"]
default-members = ["crates/deluge-sys", "crates/deluge-linux", "crates/deluge-linux-ui", "xtask"]
resolver = "2"
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `cargo test -p deluge-linux-ui controls`
Expected: PASS (`control_ids_match_deluge_bsp`).

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-linux-ui/Cargo.toml crates/deluge-linux-ui/src/lib.rs crates/deluge-linux-ui/src/controls.rs Cargo.toml
git commit -m "deluge-linux-ui: scaffold crate + controls id mirror"
```

---

### Task 2: `OledTarget` — an embedded-graphics DrawTarget over the 688-byte OLED framebuffer

**Files:**
- Create: `crates/deluge-linux-ui/src/oled.rs`
- Modify: `crates/deluge-linux-ui/src/lib.rs` (add `pub mod oled; pub use oled::{OledTarget, WIDTH, HEIGHT, FRAME_BYTES};`)
- Test: inline `#[cfg(test)]` in `oled.rs`

**Interfaces:**
- Consumes: `deluge_linux::Deluge::oled_write(&[u8]) -> Result<(), deluge_linux::Error>`
- Produces:
  - `pub const WIDTH: usize = 128; pub const HEIGHT: usize = 43; pub const LINE_BYTES: usize = 16; pub const FRAME_BYTES: usize = 688;`
  - `pub struct OledTarget { buf: [u8; FRAME_BYTES] }`
  - `OledTarget::new() -> Self`
  - `OledTarget::clear_frame(&mut self)`
  - `OledTarget::frame(&self) -> &[u8; FRAME_BYTES]`
  - `OledTarget::flush(&mut self, dlg: &mut deluge_linux::Deluge) -> Result<(), deluge_linux::Error>`
  - `impl embedded_graphics::draw_target::DrawTarget<Color = BinaryColor>` and `impl embedded_graphics::geometry::OriginDimensions` for `OledTarget`.

- [ ] **Step 1: Write the failing test for pixel packing**

`crates/deluge-linux-ui/src/oled.rs` (test module first; packing is row-major, **MSB-first within a byte** as the initial assumption — see the verification step at the end of this task):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use embedded_graphics::{
        pixelcolor::BinaryColor,
        prelude::*,
        primitives::{PrimitiveStyle, Rectangle},
    };

    #[test]
    fn frame_is_688_zeroed_bytes_initially() {
        let t = OledTarget::new();
        assert_eq!(t.frame().len(), 688);
        assert!(t.frame().iter().all(|&b| b == 0));
    }

    #[test]
    fn top_left_pixel_sets_msb_of_byte_0() {
        let mut t = OledTarget::new();
        Pixel(Point::new(0, 0), BinaryColor::On).draw(&mut t).unwrap();
        assert_eq!(t.frame()[0], 0b1000_0000, "x=0 is the MSB of byte 0");
    }

    #[test]
    fn pixel_at_x8_y1_sets_msb_of_row1_byte0() {
        let mut t = OledTarget::new();
        // y=1 starts at byte 16 (LINE_BYTES); x=8 is byte offset 1 within the row.
        Pixel(Point::new(8, 1), BinaryColor::On).draw(&mut t).unwrap();
        assert_eq!(t.frame()[16 + 1], 0b1000_0000);
    }

    #[test]
    fn off_clears_a_set_pixel() {
        let mut t = OledTarget::new();
        Pixel(Point::new(3, 0), BinaryColor::On).draw(&mut t).unwrap();
        assert_eq!(t.frame()[0], 0b0001_0000);
        Pixel(Point::new(3, 0), BinaryColor::Off).draw(&mut t).unwrap();
        assert_eq!(t.frame()[0], 0);
    }

    #[test]
    fn out_of_bounds_pixels_are_ignored() {
        let mut t = OledTarget::new();
        Pixel(Point::new(-1, 0), BinaryColor::On).draw(&mut t).unwrap();
        Pixel(Point::new(128, 0), BinaryColor::On).draw(&mut t).unwrap();
        Pixel(Point::new(0, 43), BinaryColor::On).draw(&mut t).unwrap();
        assert!(t.frame().iter().all(|&b| b == 0));
    }

    #[test]
    fn fill_rect_full_row_sets_all_line_bytes() {
        let mut t = OledTarget::new();
        Rectangle::new(Point::new(0, 0), Size::new(128, 1))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(&mut t)
            .unwrap();
        assert!(t.frame()[0..16].iter().all(|&b| b == 0xFF));
        assert!(t.frame()[16..].iter().all(|&b| b == 0));
    }

    #[test]
    fn clear_frame_zeroes_everything() {
        let mut t = OledTarget::new();
        Pixel(Point::new(0, 0), BinaryColor::On).draw(&mut t).unwrap();
        t.clear_frame();
        assert!(t.frame().iter().all(|&b| b == 0));
    }
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test -p deluge-linux-ui oled`
Expected: FAIL to compile (`OledTarget` not defined).

- [ ] **Step 3: Implement `OledTarget`**

Prepend to `crates/deluge-linux-ui/src/oled.rs`:

```rust
//! `OledTarget`: an embedded-graphics `DrawTarget` over the Linux OLED
//! framebuffer (`deluge_linux::Deluge::oled_write`).
//!
//! The Linux `deluge-oled` fb is linear/row-major: 43 rows × 16 bytes, 1bpp,
//! 128 px/row. Byte `y * 16 + x / 8`, bit `7 - (x % 8)` (MSB = leftmost pixel).
//! A set bit is a lit pixel (`BinaryColor::On`). No 5px offset — this is the
//! visible area already.

use deluge_linux::{Deluge, Error};
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{OriginDimensions, Size},
    pixelcolor::BinaryColor,
    prelude::*,
    Pixel,
};

pub const WIDTH: usize = 128;
pub const HEIGHT: usize = 43;
pub const LINE_BYTES: usize = 16;
pub const FRAME_BYTES: usize = LINE_BYTES * HEIGHT; // 688

pub struct OledTarget {
    buf: [u8; FRAME_BYTES],
}

impl OledTarget {
    pub fn new() -> Self {
        Self { buf: [0; FRAME_BYTES] }
    }

    pub fn clear_frame(&mut self) {
        self.buf = [0; FRAME_BYTES];
    }

    pub fn frame(&self) -> &[u8; FRAME_BYTES] {
        &self.buf
    }

    /// Blit the current frame to the panel.
    pub fn flush(&mut self, dlg: &mut Deluge) -> Result<(), Error> {
        dlg.oled_write(&self.buf)
    }

    #[inline]
    fn set(&mut self, x: usize, y: usize, on: bool) {
        if x >= WIDTH || y >= HEIGHT {
            return;
        }
        let idx = y * LINE_BYTES + x / 8;
        let bit = 7 - (x % 8);
        if on {
            self.buf[idx] |= 1 << bit;
        } else {
            self.buf[idx] &= !(1 << bit);
        }
    }
}

impl Default for OledTarget {
    fn default() -> Self {
        Self::new()
    }
}

impl OriginDimensions for OledTarget {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl DrawTarget for OledTarget {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels {
            if coord.x < 0 || coord.y < 0 {
                continue;
            }
            self.set(coord.x as usize, coord.y as usize, color.is_on());
        }
        Ok(())
    }
}
```

Add to `crates/deluge-linux-ui/src/lib.rs`:

```rust
pub mod oled;
pub use oled::{OledTarget, FRAME_BYTES, HEIGHT, WIDTH};
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p deluge-linux-ui oled`
Expected: PASS (all 7 tests).

- [ ] **Step 5: Verify bit order against the driver (documentation step)**

The MSB-first assumption must be confirmed against the kernel `deluge-oled` fb driver (in the `deluge-linux` platform repo / base bundle) or on hardware with a known asymmetric pattern (e.g. set only `x=0,y=0` and confirm the leftmost pixel lights). If the panel is mirrored within 8px groups, change `let bit = 7 - (x % 8);` to `let bit = x % 8;` and flip the expected bytes in the tests. Record the confirmed order in the module doc comment.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-linux-ui/src/oled.rs crates/deluge-linux-ui/src/lib.rs
git commit -m "deluge-linux-ui: OledTarget DrawTarget over the 688-byte OLED fb"
```

---

### Task 3: Scaffold `examples/launcher` + `apps` enumeration

**Files:**
- Create: `examples/launcher/Cargo.toml`
- Create: `examples/launcher/src/main.rs` (temporary stub `fn main() {}`)
- Create: `examples/launcher/src/apps.rs`
- Modify: `Cargo.toml` (workspace `members` — **not** `default-members`)

**Interfaces:**
- Produces:
  - `pub struct AppEntry { pub path: std::path::PathBuf, pub display_name: String }`
  - `pub fn scan(dir: &std::path::Path) -> Vec<AppEntry>` — regular executable files only, sorted by `display_name`, name = filename upper-cased. Missing/unreadable dir → empty vec.
  - `pub fn apps_dir() -> std::path::PathBuf` — `$LAUNCHER_APPS_DIR` or `/LINUX/APPS`.

- [ ] **Step 1: Create the crate manifest**

`examples/launcher/Cargo.toml`:

```toml
[package]
name = "launcher"
version = "0.1.0"
edition = "2021"
publish = false

[dependencies]
deluge-linux = { path = "../../crates/deluge-linux" }
deluge-linux-ui = { path = "../../crates/deluge-linux-ui" }
deluge-ui-toolkit = { path = "../../../deluge-sdk/crates/deluge-ui-toolkit" }
embedded-graphics = "0.8"
libc = "0.2"
```

`examples/launcher/src/main.rs` (temporary stub, replaced in Task 8):

```rust
mod apps;

fn main() {}
```

- [ ] **Step 2: Wire the crate into the workspace (members only)**

In the root `Cargo.toml`, add `"examples/launcher"` to `members` (leave `default-members` unchanged — it is a libdeluge-linking binary built via `cargo xtask`, like `rust-app`):

```toml
members = ["crates/deluge-sys", "crates/deluge-linux", "crates/deluge-linux-ui", "xtask", "examples/rust-app", "examples/launcher"]
```

- [ ] **Step 3: Write the failing test for `scan`**

`examples/launcher/src/apps.rs`:

```rust
//! Enumerate runnable apps in the SD card's app folder (`/LINUX/APPS/`).

use std::path::{Path, PathBuf};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AppEntry {
    pub path: PathBuf,
    pub display_name: String,
}

/// The app directory: `$LAUNCHER_APPS_DIR` if set, else `/LINUX/APPS`.
pub fn apps_dir() -> PathBuf {
    std::env::var_os("LAUNCHER_APPS_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/LINUX/APPS"))
}

/// List regular, executable files in `dir`, sorted by display name.
/// A missing or unreadable directory yields an empty list.
pub fn scan(dir: &Path) -> Vec<AppEntry> {
    use std::os::unix::fs::PermissionsExt;

    let mut out = Vec::new();
    let Ok(rd) = std::fs::read_dir(dir) else {
        return out;
    };
    for entry in rd.flatten() {
        let path = entry.path();
        let Ok(meta) = entry.metadata() else { continue };
        if !meta.is_file() {
            continue;
        }
        if meta.permissions().mode() & 0o111 == 0 {
            continue; // not executable by anyone
        }
        let Some(stem) = path.file_name().and_then(|s| s.to_str()) else {
            continue;
        };
        out.push(AppEntry {
            path: path.clone(),
            display_name: stem.to_uppercase(),
        });
    }
    out.sort_by(|a, b| a.display_name.cmp(&b.display_name));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::PermissionsExt;

    fn write_exec(dir: &Path, name: &str) {
        let p = dir.join(name);
        std::fs::write(&p, b"#!/bin/sh\n").unwrap();
        std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o755)).unwrap();
    }

    fn tmpdir() -> PathBuf {
        let base = std::env::temp_dir().join(format!("launcher-apps-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        std::fs::create_dir_all(&base).unwrap();
        base
    }

    #[test]
    fn lists_executables_uppercased_and_sorted() {
        let d = tmpdir();
        write_exec(&d, "spark");
        write_exec(&d, "wren");
        let apps = scan(&d);
        assert_eq!(apps.len(), 2);
        assert_eq!(apps[0].display_name, "SPARK");
        assert_eq!(apps[1].display_name, "WREN");
        assert_eq!(apps[0].path, d.join("spark"));
    }

    #[test]
    fn skips_non_executable_and_non_files() {
        let d = tmpdir();
        write_exec(&d, "runnable");
        std::fs::write(d.join("readme.txt"), b"hi").unwrap(); // not executable
        std::fs::create_dir(d.join("subdir")).unwrap(); // not a file
        let apps = scan(&d);
        assert_eq!(apps.len(), 1);
        assert_eq!(apps[0].display_name, "RUNNABLE");
    }

    #[test]
    fn missing_dir_yields_empty() {
        assert!(scan(Path::new("/no/such/dir/xyz")).is_empty());
    }
}
```

Update the `main.rs` stub to keep `apps` referenced (already `mod apps;` above).

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p launcher apps`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add examples/launcher/Cargo.toml examples/launcher/src/main.rs examples/launcher/src/apps.rs Cargo.toml
git commit -m "launcher: scaffold crate + /LINUX/APPS enumeration"
```

---

### Task 4: `killwatch` — the kill-chord state machine

**Files:**
- Create: `examples/launcher/src/killwatch.rs`
- Modify: `examples/launcher/src/main.rs` (add `mod killwatch;`)
- Test: inline `#[cfg(test)]`

**Interfaces:**
- Consumes: `deluge_linux_ui::controls::{SHIFT, TRIPLETS, LEARN}`
- Produces:
  - `pub struct KillWatch` with `pub fn new(hold_ms: u64) -> Self`
  - `pub fn on_button(&mut self, id: u8, pressed: bool, now_ms: u64) -> bool` — updates chord state; returns `true` iff the chord just fired.
  - `pub fn poll(&mut self, now_ms: u64) -> bool` — returns `true` iff the hold elapsed since this call and the chord is still held (fires once).
  - `pub fn reset(&mut self)` — clears all held state (call when a child exits).

- [ ] **Step 1: Write the failing tests**

`examples/launcher/src/killwatch.rs`:

```rust
//! The always-available escape chord: SHIFT + TRIPLETS + LEARN held ~1s.
//!
//! Pure state machine driven by an external monotonic clock (`now_ms`) so it is
//! fully deterministic under test. Fires exactly once per hold.

use deluge_linux_ui::controls::{LEARN, SHIFT, TRIPLETS};

pub struct KillWatch {
    hold_ms: u64,
    shift: bool,
    triplets: bool,
    learn: bool,
    armed_at: Option<u64>,
    fired: bool,
}

impl KillWatch {
    pub fn new(hold_ms: u64) -> Self {
        Self {
            hold_ms,
            shift: false,
            triplets: false,
            learn: false,
            armed_at: None,
            fired: false,
        }
    }

    fn all_held(&self) -> bool {
        self.shift && self.triplets && self.learn
    }

    fn recompute(&mut self, now_ms: u64) {
        if self.all_held() {
            if self.armed_at.is_none() {
                self.armed_at = Some(now_ms);
            }
        } else {
            self.armed_at = None;
            self.fired = false;
        }
    }

    fn maybe_fire(&mut self, now_ms: u64) -> bool {
        if self.fired {
            return false;
        }
        if let Some(start) = self.armed_at {
            if now_ms.saturating_sub(start) >= self.hold_ms {
                self.fired = true;
                return true;
            }
        }
        false
    }

    pub fn on_button(&mut self, id: u8, pressed: bool, now_ms: u64) -> bool {
        match id {
            SHIFT => self.shift = pressed,
            TRIPLETS => self.triplets = pressed,
            LEARN => self.learn = pressed,
            _ => return false,
        }
        self.recompute(now_ms);
        self.maybe_fire(now_ms)
    }

    pub fn poll(&mut self, now_ms: u64) -> bool {
        self.maybe_fire(now_ms)
    }

    pub fn reset(&mut self) {
        *self = KillWatch::new(self.hold_ms);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fires_after_hold_when_all_three_held() {
        let mut kw = KillWatch::new(1000);
        assert!(!kw.on_button(SHIFT, true, 0));
        assert!(!kw.on_button(TRIPLETS, true, 10));
        assert!(!kw.on_button(LEARN, true, 20)); // armed at 20, not yet elapsed
        assert!(!kw.poll(1000)); // 1000-20 = 980 < 1000
        assert!(kw.poll(1020)); // 1020-20 = 1000 >= 1000 -> fire
    }

    #[test]
    fn fires_only_once_per_hold() {
        let mut kw = KillWatch::new(1000);
        kw.on_button(SHIFT, true, 0);
        kw.on_button(TRIPLETS, true, 0);
        kw.on_button(LEARN, true, 0);
        assert!(kw.poll(1000));
        assert!(!kw.poll(1001));
        assert!(!kw.poll(5000));
    }

    #[test]
    fn releasing_before_hold_cancels() {
        let mut kw = KillWatch::new(1000);
        kw.on_button(SHIFT, true, 0);
        kw.on_button(TRIPLETS, true, 0);
        kw.on_button(LEARN, true, 0);
        assert!(!kw.on_button(LEARN, false, 500)); // released early
        assert!(!kw.poll(2000)); // never fires
    }

    #[test]
    fn re_holding_after_release_can_fire_again() {
        let mut kw = KillWatch::new(1000);
        kw.on_button(SHIFT, true, 0);
        kw.on_button(TRIPLETS, true, 0);
        kw.on_button(LEARN, true, 0);
        assert!(kw.poll(1000)); // first fire
        kw.on_button(LEARN, false, 1100); // release
        assert!(!kw.on_button(LEARN, true, 1200)); // re-arm at 1200
        assert!(kw.poll(2200)); // fires again
    }

    #[test]
    fn unrelated_buttons_do_not_arm() {
        let mut kw = KillWatch::new(1000);
        assert!(!kw.on_button(31, true, 0)); // select click
        assert!(!kw.on_button(SHIFT, true, 0));
        assert!(!kw.poll(5000));
    }
}
```

Add `mod killwatch;` to `examples/launcher/src/main.rs`.

- [ ] **Step 2: Run the tests to verify they fail then pass**

Run: `cargo test -p launcher killwatch`
Expected: PASS (5 tests). (If a step is done TDD-strictly, temporarily comment the impl to confirm the tests fail first.)

- [ ] **Step 3: Commit**

```bash
git add examples/launcher/src/killwatch.rs examples/launcher/src/main.rs
git commit -m "launcher: kill-chord state machine (SHIFT+TRIPLETS+LEARN hold)"
```

---

### Task 5: `supervisor` — fork/exec in own process group, classify exit, kill group

**Files:**
- Create: `examples/launcher/src/supervisor.rs`
- Modify: `examples/launcher/src/main.rs` (add `mod supervisor;`)
- Test: inline `#[cfg(test)]`

**Interfaces:**
- Produces:
  - `pub enum Exit { Clean, Nonzero(i32), Signalled(i32) }`
  - `pub fn classify(status: std::process::ExitStatus) -> Exit`
  - `pub struct Launched { pub pid: i32, pub child: std::process::Child }`
  - `pub fn launch(path: &std::path::Path) -> std::io::Result<Launched>` — spawns in a new session/group via `setsid` in `pre_exec`; `pid` is the group id.
  - `pub fn kill_group(pid: i32)` — `kill(-pid, SIGKILL)`.

- [ ] **Step 1: Write the failing tests**

`examples/launcher/src/supervisor.rs`:

```rust
//! Launch and supervise a child application.
//!
//! The child is put in its own session/process group (`setsid`) so the kill
//! chord can tear down the whole group with `kill(-pgid, SIGKILL)`.

use std::io;
use std::os::unix::process::{CommandExt, ExitStatusExt};
use std::path::Path;
use std::process::{Child, Command, ExitStatus};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Exit {
    Clean,
    Nonzero(i32),
    Signalled(i32),
}

pub fn classify(status: ExitStatus) -> Exit {
    if let Some(code) = status.code() {
        if code == 0 {
            Exit::Clean
        } else {
            Exit::Nonzero(code)
        }
    } else if let Some(sig) = status.signal() {
        Exit::Signalled(sig)
    } else {
        Exit::Nonzero(-1)
    }
}

pub struct Launched {
    pub pid: i32,
    pub child: Child,
}

pub fn launch(path: &Path) -> io::Result<Launched> {
    let mut cmd = Command::new(path);
    // SAFETY: setsid is async-signal-safe and touches no Rust heap state.
    unsafe {
        cmd.pre_exec(|| {
            if libc::setsid() < 0 {
                return Err(io::Error::last_os_error());
            }
            Ok(())
        });
    }
    let child = cmd.spawn()?;
    let pid = child.id() as i32; // setsid makes pgid == pid
    Ok(Launched { pid, child })
}

pub fn kill_group(pid: i32) {
    // Negative pid signals the whole process group. Ignore ESRCH (already gone).
    unsafe {
        libc::kill(-pid, libc::SIGKILL);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn clean_exit_classifies_clean() {
        let mut l = launch(Path::new("/bin/true")).unwrap();
        let status = l.child.wait().unwrap();
        assert_eq!(classify(status), Exit::Clean);
    }

    #[test]
    fn nonzero_exit_classifies_nonzero() {
        let mut l = launch(Path::new("/bin/false")).unwrap();
        let status = l.child.wait().unwrap();
        assert_eq!(classify(status), Exit::Nonzero(1));
    }

    #[test]
    fn kill_group_signals_the_child() {
        // /bin/sleep 60 — kill its group, expect SIGKILL.
        let mut l = launch(&PathBuf::from("/bin/sleep")).unwrap();
        // give exec a beat; not strictly required for kill to reach the group
        std::thread::sleep(std::time::Duration::from_millis(50));
        kill_group(l.pid);
        let status = l.child.wait().unwrap();
        assert_eq!(classify(status), Exit::Signalled(libc::SIGKILL));
    }

    #[test]
    fn launching_missing_binary_errors() {
        assert!(launch(Path::new("/no/such/binary/zzz")).is_err());
    }
}
```

Note: `sleep 60` — pass the arg. Adjust `kill_group_signals_the_child` to build the command with an argument by using `launch` plus a variant, OR launch `/bin/sleep` with no arg (it errors quickly). To keep `launch` single-purpose, in this test spawn `sleep` **with** an argument via a small local command instead:

Replace the body of `kill_group_signals_the_child` with:

```rust
    #[test]
    fn kill_group_signals_the_child() {
        use std::os::unix::process::CommandExt;
        let mut cmd = std::process::Command::new("/bin/sleep");
        cmd.arg("60");
        unsafe {
            cmd.pre_exec(|| {
                if libc::setsid() < 0 {
                    return Err(std::io::Error::last_os_error());
                }
                Ok(())
            });
        }
        let mut child = cmd.spawn().unwrap();
        let pid = child.id() as i32;
        std::thread::sleep(std::time::Duration::from_millis(50));
        kill_group(pid);
        let status = child.wait().unwrap();
        assert_eq!(classify(status), Exit::Signalled(libc::SIGKILL));
    }
```

Add `mod supervisor;` to `examples/launcher/src/main.rs`.

- [ ] **Step 2: Run the tests to verify they pass**

Run: `cargo test -p launcher supervisor`
Expected: PASS (4 tests). Requires `/bin/true`, `/bin/false`, `/bin/sleep` (present on Linux).

- [ ] **Step 3: Commit**

```bash
git add examples/launcher/src/supervisor.rs examples/launcher/src/main.rs
git commit -m "launcher: process supervisor (setsid launch, exit classify, kill group)"
```

---

### Task 6: `model` — the pure launcher state machine

**Files:**
- Create: `examples/launcher/src/model.rs`
- Modify: `examples/launcher/src/main.rs` (add `mod model;`)
- Test: inline `#[cfg(test)]`

**Interfaces:**
- Consumes: `crate::apps::AppEntry`, `crate::supervisor::Exit`
- Produces:
  - `pub struct Model { pub entries: Vec<AppEntry>, pub selected: usize, pub running: Option<String>, pub toast: Option<String> }`
  - `Model::new(entries: Vec<AppEntry>) -> Self`
  - `pub fn set_entries(&mut self, entries: Vec<AppEntry>)` — used on rescan; clamps `selected`.
  - `pub fn move_selection(&mut self, delta: i32)` — clamp `0..len` (no-op when running or empty).
  - `pub fn selected_path(&self) -> Option<std::path::PathBuf>` — `None` if empty or already running.
  - `pub fn begin_running(&mut self, name: String)`
  - `pub fn on_child_exit(&mut self, exit: Exit, killed_by_user: bool, now_ms: u64)` — clears `running`, sets `toast` (with expiry) on abnormal exit.
  - `pub fn tick(&mut self, now_ms: u64)` — clears `toast` when expired.
  - `pub fn is_browsing(&self) -> bool`

- [ ] **Step 1: Write the failing tests**

`examples/launcher/src/model.rs`:

```rust
//! Pure launcher state machine. No I/O — every transition is unit-testable.

use crate::apps::AppEntry;
use crate::supervisor::Exit;
use std::path::PathBuf;

const TOAST_MS: u64 = 2500;

pub struct Model {
    pub entries: Vec<AppEntry>,
    pub selected: usize,
    pub running: Option<String>,
    pub toast: Option<String>,
    toast_until: u64,
}

impl Model {
    pub fn new(entries: Vec<AppEntry>) -> Self {
        Self {
            entries,
            selected: 0,
            running: None,
            toast: None,
            toast_until: 0,
        }
    }

    pub fn is_browsing(&self) -> bool {
        self.running.is_none()
    }

    pub fn set_entries(&mut self, entries: Vec<AppEntry>) {
        self.entries = entries;
        if self.selected >= self.entries.len() {
            self.selected = self.entries.len().saturating_sub(1);
        }
    }

    pub fn move_selection(&mut self, delta: i32) {
        if !self.is_browsing() || self.entries.is_empty() {
            return;
        }
        let last = (self.entries.len() - 1) as i32;
        let next = (self.selected as i32 + delta).clamp(0, last);
        self.selected = next as usize;
    }

    pub fn selected_path(&self) -> Option<PathBuf> {
        if !self.is_browsing() {
            return None;
        }
        self.entries.get(self.selected).map(|e| e.path.clone())
    }

    pub fn begin_running(&mut self, name: String) {
        self.running = Some(name);
    }

    pub fn on_child_exit(&mut self, exit: Exit, killed_by_user: bool, now_ms: u64) {
        let name = self.running.take().unwrap_or_default();
        if killed_by_user {
            return; // silent
        }
        let msg = match exit {
            Exit::Clean => None,
            Exit::Nonzero(code) => Some(format!("{name} EXITED ({code})")),
            Exit::Signalled(sig) => Some(format!("{name} CRASHED ({sig})")),
        };
        if let Some(m) = msg {
            self.toast = Some(m);
            self.toast_until = now_ms + TOAST_MS;
        }
    }

    pub fn tick(&mut self, now_ms: u64) {
        if self.toast.is_some() && now_ms >= self.toast_until {
            self.toast = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(name: &str) -> AppEntry {
        AppEntry {
            path: PathBuf::from(format!("/LINUX/APPS/{name}")),
            display_name: name.to_uppercase(),
        }
    }

    fn model3() -> Model {
        Model::new(vec![entry("a"), entry("b"), entry("c")])
    }

    #[test]
    fn selection_clamps_at_both_ends() {
        let mut m = model3();
        m.move_selection(-1);
        assert_eq!(m.selected, 0);
        m.move_selection(1);
        m.move_selection(1);
        m.move_selection(1);
        assert_eq!(m.selected, 2);
    }

    #[test]
    fn selected_path_returns_current_entry() {
        let mut m = model3();
        m.move_selection(1);
        assert_eq!(m.selected_path(), Some(PathBuf::from("/LINUX/APPS/b")));
    }

    #[test]
    fn no_launch_when_empty() {
        let m = Model::new(vec![]);
        assert_eq!(m.selected_path(), None);
    }

    #[test]
    fn no_launch_or_move_while_running() {
        let mut m = model3();
        m.begin_running("A".into());
        assert!(!m.is_browsing());
        assert_eq!(m.selected_path(), None);
        m.move_selection(1);
        assert_eq!(m.selected, 0);
    }

    #[test]
    fn kill_returns_silently() {
        let mut m = model3();
        m.begin_running("A".into());
        m.on_child_exit(Exit::Signalled(9), true, 0);
        assert!(m.is_browsing());
        assert_eq!(m.toast, None);
    }

    #[test]
    fn clean_exit_no_toast() {
        let mut m = model3();
        m.begin_running("A".into());
        m.on_child_exit(Exit::Clean, false, 0);
        assert_eq!(m.toast, None);
    }

    #[test]
    fn crash_shows_toast_until_expiry() {
        let mut m = model3();
        m.begin_running("SPARK".into());
        m.on_child_exit(Exit::Signalled(11), false, 1000);
        assert_eq!(m.toast.as_deref(), Some("SPARK CRASHED (11)"));
        m.tick(1000 + 2499);
        assert!(m.toast.is_some());
        m.tick(1000 + 2500);
        assert_eq!(m.toast, None);
    }

    #[test]
    fn nonzero_exit_shows_exit_toast() {
        let mut m = model3();
        m.begin_running("WREN".into());
        m.on_child_exit(Exit::Nonzero(2), false, 0);
        assert_eq!(m.toast.as_deref(), Some("WREN EXITED (2)"));
    }

    #[test]
    fn set_entries_clamps_selection() {
        let mut m = model3();
        m.move_selection(1);
        m.move_selection(1); // selected = 2
        m.set_entries(vec![entry("only")]);
        assert_eq!(m.selected, 0);
    }
}
```

Add `mod model;` to `examples/launcher/src/main.rs`.

- [ ] **Step 2: Run the tests to verify they pass**

Run: `cargo test -p launcher model`
Expected: PASS (9 tests).

- [ ] **Step 3: Commit**

```bash
git add examples/launcher/src/model.rs examples/launcher/src/main.rs
git commit -m "launcher: pure state machine (selection, launch gating, exit toasts)"
```

---

### Task 7: `ui` — render the list, empty state, and toast onto `OledTarget`

**Files:**
- Create: `examples/launcher/src/ui.rs`
- Modify: `examples/launcher/src/main.rs` (add `mod ui;`)
- Test: inline `#[cfg(test)]` (render-smoke)

**Interfaces:**
- Consumes: `deluge_linux_ui::OledTarget`, `crate::model::Model`, toolkit `ListMenuView`, `RowIcon`, `components::Header`
- Produces:
  - `pub struct View { list: ListMenuView }`
  - `View::new() -> Self`
  - `pub fn tick(&mut self, model: &Model, delta_ms: u32)` — advance label-scroll animation for the selected row.
  - `pub fn render(&self, model: &Model, target: &mut OledTarget)` — clears the frame, draws header "APPS", the list (or "NO APPS"), and any toast. Does **not** flush.

- [ ] **Step 1: Write the render-smoke test**

`examples/launcher/src/ui.rs`:

```rust
//! Render the launcher list to the OLED using the shared UI toolkit.

use deluge_linux_ui::OledTarget;
use deluge_ui_toolkit::components::Header;
use deluge_ui_toolkit::text::{draw_text, Font, TextStyle};
use deluge_ui_toolkit::{ListMenuView, RowIcon};
use embedded_graphics::{prelude::*, pixelcolor::BinaryColor};

use crate::model::Model;

pub struct View {
    list: ListMenuView,
}

impl View {
    pub fn new() -> Self {
        Self {
            list: ListMenuView::new(),
        }
    }

    pub fn tick(&mut self, model: &Model, delta_ms: u32) {
        if model.entries.is_empty() {
            return;
        }
        let name = &model.entries[model.selected].display_name;
        self.list.tick(
            model.selected,
            name,
            RowIcon::None,
            model.entries.len(),
            delta_ms,
        );
    }

    pub fn render(&self, model: &Model, target: &mut OledTarget) {
        target.clear_frame();

        // Title bar.
        let _ = Header::new("APPS").draw(target);

        // The list (empty_message shows when there are no rows).
        let rows: Vec<(&str, RowIcon)> = model
            .entries
            .iter()
            .map(|e| (e.display_name.as_str(), RowIcon::None))
            .collect();
        self.list
            .render(target, &rows, model.selected, Some("NO APPS"));

        // Transient toast, drawn over the bottom row.
        if let Some(msg) = &model.toast {
            let style = TextStyle::new(Font::FontApple).with_color(BinaryColor::On);
            let _ = draw_text(target, msg, Point::new(3, 35), style);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::apps::AppEntry;
    use std::path::PathBuf;

    fn entry(name: &str) -> AppEntry {
        AppEntry {
            path: PathBuf::from(name),
            display_name: name.to_uppercase(),
        }
    }

    #[test]
    fn renders_list_without_panicking() {
        let m = Model::new(vec![entry("spark"), entry("wren")]);
        let v = View::new();
        let mut target = OledTarget::new();
        v.render(&m, &mut target);
        // Something was drawn (title + rows) — frame is not all-zero.
        assert!(target.frame().iter().any(|&b| b != 0));
    }

    #[test]
    fn renders_empty_state_without_panicking() {
        let m = Model::new(vec![]);
        let v = View::new();
        let mut target = OledTarget::new();
        v.render(&m, &mut target);
        assert!(target.frame().iter().any(|&b| b != 0)); // "NO APPS" text
    }

    #[test]
    fn renders_toast_without_panicking() {
        let mut m = Model::new(vec![entry("spark")]);
        m.begin_running("SPARK".into());
        m.on_child_exit(crate::supervisor::Exit::Signalled(11), false, 0);
        let v = View::new();
        let mut target = OledTarget::new();
        v.render(&m, &mut target);
        assert!(target.frame().iter().any(|&b| b != 0));
    }
}
```

Add `mod ui;` to `examples/launcher/src/main.rs`.

**Note on import paths:** `ListMenuView` and `RowIcon` are re-exported at the toolkit crate root; `Header` is under `deluge_ui_toolkit::components`. If `draw_text`/`Font`/`TextStyle` paths differ in the installed toolkit version, adjust to the actual `deluge_ui_toolkit::text` re-exports (confirmed present: `Font`, `TextStyle` at crate root; `draw_text` in `text`).

- [ ] **Step 2: Run the render-smoke tests to verify they pass**

Run: `cargo test -p launcher ui`
Expected: PASS (3 tests).

- [ ] **Step 3: Commit**

```bash
git add examples/launcher/src/ui.rs examples/launcher/src/main.rs
git commit -m "launcher: OLED rendering (header, app list, empty state, toast)"
```

---

### Task 8: `main` — wire input, reaper, and the event loop

**Files:**
- Modify: `examples/launcher/src/main.rs` (replace the stub with the full loop)

**Interfaces:**
- Consumes: `apps`, `killwatch::KillWatch`, `supervisor::{launch, kill_group, classify, Launched}`, `model::Model`, `ui::View`, `deluge_linux::{Deluge, Event}`, `deluge_linux_ui::{OledTarget, controls}`

- [ ] **Step 1: Replace `main.rs` with the full wiring**

`examples/launcher/src/main.rs`:

```rust
//! Reference Linux appliance: the app launcher.
//!
//! Lists `/LINUX/APPS/` on the OLED, launches the selected app with the select
//! encoder, and always returns here on SHIFT+TRIPLETS+LEARN held ~1s.

mod apps;
mod killwatch;
mod model;
mod supervisor;
mod ui;

use deluge_linux::{Deluge, Event};
use deluge_linux_ui::{controls, OledTarget};
use killwatch::KillWatch;
use model::Model;
use std::sync::mpsc::{self, RecvTimeoutError};
use std::time::{Duration, Instant};
use ui::View;

const FRAME: Duration = Duration::from_millis(33);
const HOLD_MS: u64 = 1000;

// Event kinds (mirror deluge-linux / include/deluge/input.h).
const EV_BUTTON: u8 = 1;
const EV_ENCODER: u8 = 2;

enum Msg {
    Input(Event),
    ChildExited(supervisor::Exit),
}

fn main() {
    let mut dlg = match Deluge::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("launcher: deluge_open failed: {e}");
            std::process::exit(1);
        }
    };

    let (tx, rx) = mpsc::channel::<Msg>();

    // Input: the SDK-owned thread forwards every event onto the channel.
    let input_tx = tx.clone();
    if let Err(e) = dlg.input_start(move |ev| {
        let _ = input_tx.send(Msg::Input(ev));
    }) {
        eprintln!("launcher: input_start failed: {e}");
        std::process::exit(1);
    }

    let mut target = OledTarget::new();
    let mut view = View::new();
    let mut model = Model::new(apps::scan(&apps::apps_dir()));
    let mut kw = KillWatch::new(HOLD_MS);

    // While a child runs: its pid (process group) and whether we initiated a kill.
    let mut running_pid: Option<i32> = None;
    let mut killed_by_user = false;

    let start = Instant::now();
    let now_ms = || start.elapsed().as_millis() as u64;

    // Initial paint.
    view.render(&model, &mut target);
    let _ = target.flush(&mut dlg);

    let mut last_tick = Instant::now();

    loop {
        match rx.recv_timeout(FRAME) {
            Ok(Msg::Input(ev)) => {
                let t = now_ms();
                if ev.kind == EV_BUTTON {
                    let pressed = ev.value == 1;
                    // Kill-chord watch runs in every mode.
                    if kw.on_button(ev.id, pressed, t) {
                        if let Some(pid) = running_pid {
                            killed_by_user = true;
                            supervisor::kill_group(pid);
                        }
                    }
                    // Select-click launches while browsing.
                    if model.is_browsing() && ev.id == controls::SELECT_CLICK && pressed {
                        launch_selected(&mut model, &tx, &mut running_pid, &mut killed_by_user);
                    }
                } else if ev.kind == EV_ENCODER
                    && ev.id == controls::ENC_SELECT
                    && model.is_browsing()
                {
                    model.move_selection(ev.value.signum() as i32);
                }
            }
            Ok(Msg::ChildExited(exit)) => {
                running_pid = None;
                model.on_child_exit(exit, killed_by_user, now_ms());
                killed_by_user = false;
                kw.reset();
                model.set_entries(apps::scan(&apps::apps_dir())); // rescan on return
                view.render(&model, &mut target);
                let _ = target.flush(&mut dlg);
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }

        // Frame cadence: poll the chord timer, expire toasts, animate + repaint
        // (only while browsing — a running child owns the OLED).
        let t = now_ms();
        if kw.poll(t) {
            if let Some(pid) = running_pid {
                killed_by_user = true;
                supervisor::kill_group(pid);
            }
        }
        model.tick(t);

        if model.is_browsing() {
            let delta = last_tick.elapsed().as_millis() as u32;
            last_tick = Instant::now();
            view.tick(&model, delta);
            view.render(&model, &mut target);
            let _ = target.flush(&mut dlg);
        } else {
            last_tick = Instant::now();
        }
    }
}

/// Launch the highlighted app: spawn it, mark the model running, and start a
/// reaper thread that reports the exit status back onto the channel.
fn launch_selected(
    model: &mut Model,
    tx: &mpsc::Sender<Msg>,
    running_pid: &mut Option<i32>,
    killed_by_user: &mut bool,
) {
    let Some(path) = model.selected_path() else {
        return;
    };
    let name = model
        .entries
        .get(model.selected)
        .map(|e| e.display_name.clone())
        .unwrap_or_default();

    match supervisor::launch(&path) {
        Ok(supervisor::Launched { pid, mut child }) => {
            *running_pid = Some(pid);
            *killed_by_user = false;
            model.begin_running(name);
            let reap_tx = tx.clone();
            std::thread::spawn(move || {
                let status = child.wait();
                let exit = match status {
                    Ok(s) => supervisor::classify(s),
                    Err(_) => supervisor::Exit::Nonzero(-1),
                };
                let _ = reap_tx.send(Msg::ChildExited(exit));
            });
        }
        Err(_) => {
            // exec failed — show a toast and stay in the list.
            model.begin_running(name.clone());
            model.on_child_exit(supervisor::Exit::Nonzero(-1), false, 0);
        }
    }
}
```

- [ ] **Step 2: Verify the whole crate compiles and all logic tests still pass**

Run: `cargo build -p launcher` (host) — expected: builds (links host libdeluge).
Run: `cargo test -p launcher` — expected: PASS (apps + killwatch + supervisor + model + ui, all green).

- [ ] **Step 3: Manual host smoke (no hardware)**

Run with a fake app dir to confirm it starts, scans, and exits cleanly without a panic on a host lacking the panel (input/display unavailable → `Deluge::open`/`input_start` error path exits non-zero, which is expected):

```bash
LAUNCHER_APPS_DIR=/tmp/fakeapps cargo run -p launcher; echo "exit: $?"
```

Expected: prints an `input_start`/`deluge_open` failure and exits `1` on a host with no Deluge hardware. (Full behavior is verified on-device in Task 9's manual checklist.)

- [ ] **Step 4: Commit**

```bash
git add examples/launcher/src/main.rs
git commit -m "launcher: event loop wiring (input channel, reaper, kill chord)"
```

---

### Task 9: `cargo xtask image` output-name override + docs, and on-device verification

**Files:**
- Modify: `xtask/src/main.rs` (add an optional `-o <NAME>`/`--out <NAME>` to override the derived ELF name)
- Modify: `docs/building-an-app.md` (document building the launcher as `LINUX.ELF`)

**Interfaces:**
- Consumes: existing xtask arg parsing (`image|bare`, `<app>`, `--profile`)
- Produces: `cargo xtask image launcher --out LINUX` → `target/LINUX.ELF`

- [ ] **Step 1: Add the `--out` override to xtask**

In `xtask/src/main.rs`, extend the arg loop that currently parses `--profile` to also accept `--out <NAME>` (or `-o <NAME>`), storing `Option<String> out_name`. Then change the `image` output line:

```rust
// was: "image" => PathBuf::from(format!("target/{}.ELF", app.to_uppercase())),
"image" => {
    let stem = out_name.clone().unwrap_or_else(|| app.to_uppercase());
    PathBuf::from(format!("target/{stem}.ELF"))
}
```

Add `--out`/`-o` to the `usage()` string:

```
usage: cargo xtask <image|bare> <app> [--profile <name>] [--out <NAME>]
  --out <NAME>   image mode only: name the ELF target/<NAME>.ELF
                 (default: <app> upper-cased). Use --out LINUX for the launcher.
```

(`--out` applies to `image` only; ignore/`usage()` it under `bare`, matching how `--profile` is image-only.)

- [ ] **Step 2: Build the launcher image for the device**

Run:

```bash
export DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-0.1.0   # or your unpacked bundle
cargo xtask image launcher --out LINUX
```

Expected: `target/LINUX.ELF` is produced (deluge-mkimage prints success). This links `armv7-unknown-linux-musleabihf` with `crt-static` and rejects a dynamic binary — a clean build confirms static linking.

- [ ] **Step 3: Document the launcher build in `docs/building-an-app.md`**

Add a short section after the Rust build section:

```markdown
## The launcher (reference appliance)

The generic `LINUX` image's app is the launcher (`examples/launcher`). Build it
as `LINUX.ELF`:

    cargo xtask image launcher --out LINUX     # -> target/LINUX.ELF for /APPS/

Copy `LINUX.ELF` to the card's `/APPS/`, and drop bare app binaries
(`cargo xtask bare <app>`) into `/LINUX/APPS/`. On boot the app-loader launches
`LINUX.ELF`; the launcher lists `/LINUX/APPS/` on the OLED, the select encoder
browses and launches, and SHIFT+TRIPLETS+LEARN (held ~1s) always returns to it.
```

- [ ] **Step 4: On-device manual verification checklist**

Deploy `target/LINUX.ELF` to `/APPS/` and one or more bare apps to `/LINUX/APPS/`, boot it, and confirm:

- The app list renders (title "APPS", rows for each `/LINUX/APPS/` binary), and "NO APPS" shows when the folder is empty.
- Select-encoder rotation moves the highlight; the label scrolls when it overflows; the scrollbar appears with >3 apps.
- Select-encoder click launches the highlighted app (it takes over the OLED).
- Holding SHIFT+TRIPLETS+LEARN for ~1s kills the running app and returns to the list, regardless of what the app is doing; a shorter hold does not.
- A clean-exiting app returns silently; a crashing app shows the `… CRASHED (sig)` toast briefly.
- The OLED image is correct (not mirrored/inverted). If wrong within 8px groups, apply the bit-order fix from Task 2 Step 5.

- [ ] **Step 5: Commit**

```bash
git add xtask/src/main.rs docs/building-an-app.md
git commit -m "xtask: --out override for LINUX.ELF; docs: launcher build"
```

---

## Self-Review Notes

- **Spec coverage:** browse-scope=app-list (Task 3), toolkit bridge crate (Tasks 1–2), kill chord hold ~1s (Task 4 + main), filename-only names (Task 3), on-exit toast policy (Task 6), OLED no-offset row-major fb (Task 2), no-EVIOCGRAB shared input enabling the watchdog (Task 8 design), setsid + kill-group supervision (Task 5), `LINUX.ELF` naming via xtask (Task 9), runit unchanged (documented, no task needed).
- **Type consistency:** `Exit`/`classify`/`Launched` (Task 5) are consumed unchanged by `model` (Task 6) and `main` (Task 8); `controls::{SHIFT,TRIPLETS,LEARN,SELECT_CLICK,ENC_SELECT}` (Task 1) used by `killwatch` (Task 4) and `main` (Task 8); `OledTarget`/`frame()`/`clear_frame()`/`flush()` (Task 2) used by `ui` (Task 7) and `main` (Task 8); `Model` accessors (Task 6) used by `ui` and `main`.
- **Open detail carried forward:** OLED in-byte bit order (Task 2 Step 5 / Task 9 Step 4) and the reaper-thread-vs-signalfd choice (reaper thread chosen in Task 8) — both flagged in the spec as non-blocking.
