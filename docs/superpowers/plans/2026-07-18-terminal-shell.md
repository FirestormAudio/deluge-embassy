# Terminal (PTY shell) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A bare `/LINUX/APPS/` example app that runs an interactive BusyBox shell on a PTY, renders its output as a 25×6 character terminal on the OLED, and takes input from the on-pad QWERTY keyboard plus front-panel keys.

**Architecture:** A new `pads` bridge in `deluge-linux-ui` packs a `deluge-grid-toolkit` `Grid` into the 432-byte pad frame and adapts `Event`→`PadInput`. The `TextKeyboardComponent` widget is fixed to be shift-aware. The `terminal` app spawns `/bin/sh -i` via `forkpty`, feeds a `vte`-parsed screen model, and renders it with an `embedded-graphics` mono font.

**Tech Stack:** Rust 2021, `deluge-linux`, `deluge-linux-ui`, `deluge-grid-toolkit`, `deluge-bsp`, `embedded-graphics` 0.8, `vte` 0.15, `libc`.

## Global Constraints

- Cross-build the app: `export DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-0.1.0` then `cargo xtask bare terminal` → `target/bare/terminal` (copied to card `/LINUX/APPS/`).
- Host unit tests/builds require the host libdeluge prefix on the env: `export DELUGE_SDK_ROOT=/home/kate/GitHub/deluge-linux-sdk/stage` (contains `include/deluge/deluge.h` + `lib/libdeluge.a`). Then `DELUGE_SDK_ROOT=$PWD/stage cargo test -p <crate>`. Without it, `deluge-sys`'s build script fails with `'deluge/deluge.h' file not found`.
- All apps are static (`crt-static`) — handled by `cargo xtask`; do not add dynamic deps.
- Deluge I/O calls in app code are best-effort: `let _ = dlg.oled_flush(...)` etc., exactly like `examples/snake`.
- Terminal grid is fixed at `COLS = 25`, `ROWS = 6` (from `FONT_5X7`'s 5×7 cell on the 128×43 panel).
- Pad frame is 432 bytes, 18×8, stride 54 (`row*54 + col*3` = `[r,g,b]`).
- The widget fix (Task 3) lands in the **sibling repo** `../deluge-sdk` (`crates/deluge-grid-toolkit`) and is committed there; everything else lands in this repo.
- Keep dependencies minimal and match existing file/comment style in each crate.

---

### Task 1: `PadTarget` — pack a Grid into the 432-byte pad frame

**Files:**
- Create: `crates/deluge-linux-ui/src/pads.rs`
- Modify: `crates/deluge-linux-ui/src/lib.rs`
- Modify: `crates/deluge-linux-ui/Cargo.toml`

**Interfaces:**
- Produces: `pads::PadTarget` with `new() -> Self`, `frame() -> &[u8; PAD_FRAME_BYTES]`, `clear_frame(&mut self)`, `blit(&mut self, grid: &deluge_grid_toolkit::Grid)`, `flush(&mut self, dlg: &mut deluge_linux::Deluge) -> Result<(), deluge_linux::Error>`. Const `pads::PAD_FRAME_BYTES = 432`, `pads::STRIDE = 54`.

- [ ] **Step 1: Add the grid-toolkit dependency**

Modify `crates/deluge-linux-ui/Cargo.toml`, adding to `[dependencies]`:

```toml
deluge-grid-toolkit = { path = "../../../deluge-sdk/crates/deluge-grid-toolkit" }
```

- [ ] **Step 2: Write the failing test**

Create `crates/deluge-linux-ui/src/pads.rs`:

```rust
//! `PadTarget`: the pad-grid analogue of [`crate::OledTarget`]. Packs a
//! `deluge_grid_toolkit::Grid` into the 432-byte Deluge pad RGB frame and
//! flushes it via `Deluge::pads_write`.
//!
//! The Linux pad fb is 18×8, 24bpp RGB, row-major, stride 54: byte
//! `row*54 + col*3` holds `[r, g, b]` for pad (row, col).

use deluge_grid_toolkit::{Grid, GRID_COLS, GRID_ROWS};
use deluge_linux::{Deluge, Error};

pub const STRIDE: usize = GRID_COLS * 3; // 54
pub const PAD_FRAME_BYTES: usize = STRIDE * GRID_ROWS; // 432

pub struct PadTarget {
    buf: [u8; PAD_FRAME_BYTES],
}

impl PadTarget {
    pub fn new() -> Self {
        Self { buf: [0; PAD_FRAME_BYTES] }
    }

    pub fn frame(&self) -> &[u8; PAD_FRAME_BYTES] {
        &self.buf
    }

    pub fn clear_frame(&mut self) {
        self.buf = [0; PAD_FRAME_BYTES];
    }

    /// Pack a grid-toolkit `Grid` into the 432-byte RGB frame.
    pub fn blit(&mut self, grid: &Grid) {
        for row in 0..GRID_ROWS {
            for col in 0..GRID_COLS {
                let c = grid.get_pad(row, col);
                let o = row * STRIDE + col * 3;
                self.buf[o] = c.r;
                self.buf[o + 1] = c.g;
                self.buf[o + 2] = c.b;
            }
        }
    }

    /// Blit the current frame to the pad LEDs.
    pub fn flush(&mut self, dlg: &mut Deluge) -> Result<(), Error> {
        dlg.pads_write(&self.buf)
    }
}

impl Default for PadTarget {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use deluge_grid_toolkit::Color;

    #[test]
    fn frame_is_432_zeroed_initially() {
        let t = PadTarget::new();
        assert_eq!(t.frame().len(), 432);
        assert!(t.frame().iter().all(|&b| b == 0));
    }

    #[test]
    fn blit_packs_rgb_at_stride_54() {
        let mut g = Grid::new();
        g.set_pad(0, 0, Color { r: 10, g: 20, b: 30 });
        g.set_pad(1, 2, Color { r: 1, g: 2, b: 3 });
        let mut t = PadTarget::new();
        t.blit(&g);
        assert_eq!(&t.frame()[0..3], &[10, 20, 30]);
        // row 1, col 2 -> offset 1*54 + 2*3 = 60
        assert_eq!(&t.frame()[60..63], &[1, 2, 3]);
    }
}
```

- [ ] **Step 3: Wire the module into the crate**

Modify `crates/deluge-linux-ui/src/lib.rs`, adding after the existing `pub mod`/`pub use` lines:

```rust
pub mod pads;
pub use pads::{PadTarget, PAD_FRAME_BYTES};
pub use deluge_grid_toolkit;
```

- [ ] **Step 4: Run the tests**

Run: `cargo test -p deluge-linux-ui pads`
Expected: PASS (both `pads::tests`).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-linux-ui/src/pads.rs crates/deluge-linux-ui/src/lib.rs crates/deluge-linux-ui/Cargo.toml
git commit -m "feat(ui): add PadTarget bridge from grid-toolkit Grid to pad frame"
```

---

### Task 2: Pad input adapter (`Event` → `PadInput`)

**Files:**
- Modify: `crates/deluge-linux-ui/src/pads.rs`

**Interfaces:**
- Consumes: `deluge_linux::Event` (fields `kind: u8`, `id: u8`, `value: i16`, `x: u16`, `y: u16`, `pressure: u16`).
- Produces: `pads::EV_PAD = 0`; `pads::pad_from_event(&Event) -> Option<(deluge_grid_toolkit::Pad, bool)>`; `pads::pad_input(&Event) -> deluge_grid_toolkit::imode::PadInput`.

- [ ] **Step 1: Write the failing test**

Append to `crates/deluge-linux-ui/src/pads.rs` (add the two `use` lines to the top-of-file imports, and the functions above the `#[cfg(test)]` block):

```rust
use deluge_grid_toolkit::imode::PadInput;
use deluge_grid_toolkit::Pad;
use deluge_linux::Event;

/// Event kind for pad events (mirrors `DELUGE_EV_PAD` in `include/deluge/input.h`).
pub const EV_PAD: u8 = 0;

/// Convert a Deluge pad event into a `(Pad, pressed)` pair.
/// Returns `None` for non-pad events or out-of-grid coordinates.
pub fn pad_from_event(ev: &Event) -> Option<(Pad, bool)> {
    if ev.kind != EV_PAD {
        return None;
    }
    let (row, col) = (ev.y as usize, ev.x as usize);
    if row >= GRID_ROWS || col >= GRID_COLS {
        return None;
    }
    Some((Pad::new(row, col), ev.value != 0))
}

/// Build a single-event `PadInput` frame from a Deluge event (empty if the
/// event is not a pad event). The keyboard widget reads only `input.events`.
pub fn pad_input(ev: &Event) -> PadInput {
    let mut input = PadInput::new();
    if let Some((pad, pressed)) = pad_from_event(ev) {
        if pressed {
            input.press(pad);
        } else {
            input.release(pad);
        }
    }
    input
}
```

Add these tests inside the existing `#[cfg(test)] mod tests`:

```rust
    fn ev(kind: u8, value: i16, x: u16, y: u16) -> Event {
        Event { kind, id: 0, value, x, y, pressure: 0 }
    }

    #[test]
    fn pad_event_maps_to_pad_and_pressed() {
        let (pad, pressed) = pad_from_event(&ev(EV_PAD, 1, 3, 2)).unwrap();
        assert_eq!(pad, Pad::new(2, 3));
        assert!(pressed);
    }

    #[test]
    fn release_has_pressed_false() {
        assert_eq!(pad_from_event(&ev(EV_PAD, 0, 1, 1)).unwrap().1, false);
    }

    #[test]
    fn non_pad_event_is_none() {
        assert!(pad_from_event(&ev(1, 1, 0, 0)).is_none());
    }

    #[test]
    fn out_of_grid_is_none() {
        assert!(pad_from_event(&ev(EV_PAD, 1, 99, 0)).is_none());
    }
```

- [ ] **Step 2: Run the tests**

Run: `cargo test -p deluge-linux-ui pads`
Expected: PASS (six tests total).

- [ ] **Step 3: Commit**

```bash
git add crates/deluge-linux-ui/src/pads.rs
git commit -m "feat(ui): add Event->PadInput adapter for the pad grid"
```

---

### Task 3: Shift-aware `TextKeyboardComponent` (sibling repo `deluge-sdk`)

**Files:**
- Modify: `../deluge-sdk/crates/deluge-grid-toolkit/src/widgets/text_keyboard.rs`

**Interfaces:**
- Produces: shift-aware output from `TextKeyboardComponent::show(f, shift_held)`; new QWERTY keys `/`, `;`, `\` reachable via `char_at`; private `resolve(base: char, shift: bool) -> char` (module-testable).

**Context for the implementer:** `char_at(x, y)` returns the physical key's label char (upper-case letters, e.g. `'Q'`). `show()` currently returns `KeyPress::Char(char_at(...))` and ignores `shift_held` for output. `QWERTY_HOME_ROW = 4`; char rows 0..4 map to grid rows `HOME-2..HOME+2`; grid col = `3 + char_col`. The SHIFT pads occupy `(HOME+1, 1)`, `(HOME+1, 2)`, `(HOME+1, 13)`, `(HOME+1, 14)` — so the char-row-3 col-10 slot (grid `(HOME+1, 13)`) is unusable; only 3 empty slots are free.

- [ ] **Step 1: Write the failing tests**

Add to the `#[cfg(test)] mod tests` in `text_keyboard.rs`:

```rust
    #[test]
    fn resolve_toggles_letter_case() {
        assert_eq!(resolve('A', false), 'a');
        assert_eq!(resolve('A', true), 'A');
        assert_eq!(resolve('z', false), 'z');
    }

    #[test]
    fn resolve_shifted_symbols() {
        assert_eq!(resolve('1', true), '!');
        assert_eq!(resolve('7', true), '&');
        assert_eq!(resolve('-', true), '_');
        assert_eq!(resolve(',', true), '<');
        assert_eq!(resolve('.', true), '>');
        assert_eq!(resolve('\'', true), '"');
        assert_eq!(resolve('/', true), '?');
        assert_eq!(resolve(';', true), ':');
        assert_eq!(resolve('\\', true), '|');
    }

    #[test]
    fn new_qwerty_symbol_keys_present() {
        let kb = TextKeyboardComponent::new(KeyboardLayout::Qwerty);
        assert_eq!(kb.char_at(13, QWERTY_HOME_ROW - 1), Some('/')); // char row1 col10
        assert_eq!(kb.char_at(12, QWERTY_HOME_ROW), Some(';'));     // char row2 col9
        assert_eq!(kb.char_at(12, QWERTY_HOME_ROW + 1), Some('\\')); // char row3 col9
    }

    #[test]
    fn space_stays_space() {
        assert_eq!(resolve(' ', false), ' ');
        assert_eq!(resolve(' ', true), ' ');
    }
```

- [ ] **Step 2: Run the tests to verify they fail**

Run (from `../deluge-sdk`): `cargo test -p deluge-grid-toolkit text_keyboard`
Expected: FAIL — `resolve` not found; `char_at(13, ...)` returns `None`.

- [ ] **Step 3: Add the new symbol keys to the QWERTY table**

In `KEYBOARD_CHARS`, edit only the **QWERTY** block (index 0), replacing the three `'\0'` slots noted below:

```rust
    // QWERTY
    [
        ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-'],
        ['Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', '/'],   // col10: '/'
        ['A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', ';', '\''],  // col9: ';'
        ['Z', 'X', 'C', 'V', 'B', 'N', 'M', ',', '.', '\\', '\0'], // col9: '\'
        ['\0', '\0', ' ', ' ', ' ', ' ', ' ', ' ', '\0', '\0', '\0'],
    ],
```

- [ ] **Step 4: Add `resolve`/`shifted` and make `show` shift-aware**

Add these free functions near the top of the `impl TextKeyboardComponent` area (module scope):

```rust
/// Map a base (unshifted) char to its shifted form on the US-QWERTY layout.
fn shifted(base: char) -> char {
    match base {
        '1' => '!', '2' => '@', '3' => '#', '4' => '$', '5' => '%',
        '6' => '^', '7' => '&', '8' => '*', '9' => '(', '0' => ')',
        '-' => '_', '\'' => '"', ',' => '<', '.' => '>',
        '/' => '?', ';' => ':', '\\' => '|',
        c if c.is_ascii_lowercase() => c.to_ascii_uppercase(),
        other => other,
    }
}

/// Resolve a physical key's label char to the character it emits, given shift.
/// Letters are lower-case unshifted; `shift` upper-cases letters and applies
/// the shifted-symbol map.
fn resolve(base: char, shift: bool) -> char {
    let lower = if base.is_ascii_uppercase() {
        base.to_ascii_lowercase()
    } else {
        base
    };
    if shift {
        shifted(lower)
    } else {
        lower
    }
}
```

In `show()`, change the char branch from returning `KeyPress::Char(ch)` to applying `resolve`:

```rust
            if let Some(ch) = self.char_at(x, y) {
                return Some(KeyPress::Char(resolve(ch, shift_held)));
            }
```

- [ ] **Step 5: Light the two new keys that the draw loop misses**

In `draw()`, after the existing `for i in 0..11 { … }` loop and its explicit special-key paints, add (matching the dim letter-key shade already used in this function):

```rust
        // New symbol keys filling previously-empty QWERTY slots. '(HOME, 12)'
        // (';') is already lit by the home-row loop; these two are not.
        f.paint(Pad::new(QWERTY_HOME_ROW - 1, 13), RGB::new(10, 10, 10)); // '/'
        f.paint(Pad::new(QWERTY_HOME_ROW + 1, 12), RGB::new(10, 10, 10)); // '\'
```

- [ ] **Step 6: Run the tests to verify they pass**

Run (from `../deluge-sdk`): `cargo test -p deluge-grid-toolkit text_keyboard`
Expected: PASS (new tests + all pre-existing `text_keyboard` tests still green).

- [ ] **Step 7: Commit (in the deluge-sdk repo)**

```bash
cd ../deluge-sdk
git add crates/deluge-grid-toolkit/src/widgets/text_keyboard.rs
git commit -m "feat(grid-toolkit): shift-aware TextKeyboardComponent + shell symbol keys"
cd -
```

---

### Task 4: Scaffold the `terminal` crate

**Files:**
- Create: `examples/terminal/Cargo.toml`
- Create: `examples/terminal/src/main.rs`
- Modify: `Cargo.toml` (workspace `members`)

**Interfaces:**
- Produces: a buildable `terminal` workspace package.

- [ ] **Step 1: Create the crate manifest**

Create `examples/terminal/Cargo.toml`:

```toml
[package]
name = "terminal"
version = "0.1.0"
edition = "2021"
publish = false

[dependencies]
deluge-linux = { path = "../../crates/deluge-linux" }
deluge-linux-ui = { path = "../../crates/deluge-linux-ui" }
embedded-graphics = "0.8"
vte = "0.15"
libc = "0.2"
```

- [ ] **Step 2: Create a placeholder main**

Create `examples/terminal/src/main.rs`:

```rust
//! Terminal: an interactive BusyBox shell on the OLED, typed on the pad grid.
fn main() {}
```

- [ ] **Step 3: Add to the workspace**

Modify the root `Cargo.toml` `members` array to include `"examples/terminal"` (leave `default-members` unchanged — the app links libdeluge and is built via `cargo xtask`):

```toml
members = ["crates/deluge-sys", "crates/deluge-linux", "crates/deluge-linux-ui", "xtask", "examples/rust-app", "examples/launcher", "examples/snake", "examples/terminal"]
```

- [ ] **Step 4: Verify it builds**

Run: `cargo build -p terminal`
Expected: PASS (links host `libdeluge` from `stage/`; if it fails to link, build the host lib per `docs/building-an-app.md`).

- [ ] **Step 5: Commit**

```bash
git add examples/terminal/Cargo.toml examples/terminal/src/main.rs Cargo.toml
git commit -m "chore(terminal): scaffold terminal example crate"
```

---

### Task 5: `term` — screen model + vte parser

**Files:**
- Create: `examples/terminal/src/term.rs`
- Modify: `examples/terminal/src/main.rs` (add `mod term;`)

**Interfaces:**
- Produces: `term::COLS = 25`, `term::ROWS = 6`; `term::Screen { pub cells: [[u8; COLS]; ROWS], pub cx: usize, pub cy: usize, pub dirty: bool }` with `Screen::new()`; `term::Term` with `new()`, `feed(&mut self, &[u8])`, and public field `screen: Screen`.

- [ ] **Step 1: Write the failing tests**

Create `examples/terminal/src/term.rs`:

```rust
//! A fixed 25×6 character screen driven by a `vte` parser. Enough VT100 to
//! show BusyBox `ash` output on the 1bpp OLED: printable chars, C0 controls
//! (LF/CR/BS/TAB), and a minimal CSI subset (cursor moves + erase). Colours
//! (SGR) and everything else are parsed and ignored.

use vte::{Params, Parser, Perform};

pub const COLS: usize = 25;
pub const ROWS: usize = 6;

pub struct Screen {
    pub cells: [[u8; COLS]; ROWS],
    pub cx: usize,
    pub cy: usize,
    pub dirty: bool,
}

impl Screen {
    pub fn new() -> Self {
        Self { cells: [[b' '; COLS]; ROWS], cx: 0, cy: 0, dirty: true }
    }

    fn scroll_up(&mut self) {
        for r in 1..ROWS {
            self.cells[r - 1] = self.cells[r];
        }
        self.cells[ROWS - 1] = [b' '; COLS];
    }

    fn line_feed(&mut self) {
        if self.cy + 1 >= ROWS {
            self.scroll_up();
        } else {
            self.cy += 1;
        }
    }

    fn putc(&mut self, c: char) {
        if self.cx >= COLS {
            self.cx = 0;
            self.line_feed();
        }
        // Only store single-byte printables; anything else becomes '?'.
        let b = if c.is_ascii() && !c.is_control() { c as u8 } else { b'?' };
        self.cells[self.cy][self.cx] = b;
        self.cx += 1;
        self.dirty = true;
    }

    fn erase_line(&mut self, mode: usize) {
        let row = &mut self.cells[self.cy];
        match mode {
            0 => row[self.cx..].fill(b' '),
            1 => row[..=self.cx.min(COLS - 1)].fill(b' '),
            _ => row.fill(b' '),
        }
        self.dirty = true;
    }

    fn erase_display(&mut self, mode: usize) {
        match mode {
            0 => {
                self.erase_line(0);
                for r in (self.cy + 1)..ROWS {
                    self.cells[r] = [b' '; COLS];
                }
            }
            1 => {
                for r in 0..self.cy {
                    self.cells[r] = [b' '; COLS];
                }
                self.erase_line(1);
            }
            _ => {
                self.cells = [[b' '; COLS]; ROWS];
                self.cx = 0;
                self.cy = 0;
            }
        }
        self.dirty = true;
    }
}

/// First CSI parameter, defaulting to 1 (used for relative moves / counts).
fn p1(params: &Params) -> usize {
    params
        .iter()
        .next()
        .and_then(|p| p.first().copied())
        .map(|v| v as usize)
        .filter(|&v| v != 0)
        .unwrap_or(1)
}

/// Nth CSI parameter as a raw value (0 if absent).
fn pn(params: &Params, i: usize) -> usize {
    params
        .iter()
        .nth(i)
        .and_then(|p| p.first().copied())
        .map(|v| v as usize)
        .unwrap_or(0)
}

impl Perform for Screen {
    fn print(&mut self, c: char) {
        self.putc(c);
    }

    fn execute(&mut self, byte: u8) {
        match byte {
            b'\n' => self.line_feed(),
            b'\r' => self.cx = 0,
            0x08 => {
                self.cx = self.cx.saturating_sub(1);
            }
            b'\t' => {
                self.cx = ((self.cx / 8) + 1) * 8;
                if self.cx >= COLS {
                    self.cx = COLS - 1;
                }
            }
            _ => {}
        }
        self.dirty = true;
    }

    fn csi_dispatch(&mut self, params: &Params, _intermediates: &[u8], _ignore: bool, action: char) {
        match action {
            'H' | 'f' => {
                let row = pn(params, 0).max(1) - 1;
                let col = pn(params, 1).max(1) - 1;
                self.cy = row.min(ROWS - 1);
                self.cx = col.min(COLS - 1);
            }
            'A' => self.cy = self.cy.saturating_sub(p1(params)),
            'B' => self.cy = (self.cy + p1(params)).min(ROWS - 1),
            'C' => self.cx = (self.cx + p1(params)).min(COLS - 1),
            'D' => self.cx = self.cx.saturating_sub(p1(params)),
            'K' => self.erase_line(pn(params, 0)),
            'J' => self.erase_display(pn(params, 0)),
            _ => {}
        }
        self.dirty = true;
    }
}

pub struct Term {
    parser: Parser,
    pub screen: Screen,
}

impl Term {
    pub fn new() -> Self {
        Self { parser: Parser::new(), screen: Screen::new() }
    }

    pub fn feed(&mut self, bytes: &[u8]) {
        self.parser.advance(&mut self.screen, bytes);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row_str(s: &Screen, r: usize) -> String {
        s.cells[r].iter().map(|&b| b as char).collect()
    }

    #[test]
    fn prints_at_origin() {
        let mut t = Term::new();
        t.feed(b"hi");
        assert_eq!(&row_str(&t.screen, 0)[..2], "hi");
        assert_eq!((t.screen.cx, t.screen.cy), (2, 0));
    }

    #[test]
    fn wraps_at_right_edge() {
        let mut t = Term::new();
        t.feed(&[b'x'; 26]);
        assert_eq!(t.screen.cells[0][24], b'x');
        assert_eq!(t.screen.cells[1][0], b'x');
        assert_eq!((t.screen.cx, t.screen.cy), (1, 1));
    }

    #[test]
    fn crlf_moves_to_next_line_start() {
        let mut t = Term::new();
        t.feed(b"ab\r\ncd");
        assert_eq!(&row_str(&t.screen, 0)[..2], "ab");
        assert_eq!(&row_str(&t.screen, 1)[..2], "cd");
    }

    #[test]
    fn scrolls_when_past_bottom() {
        let mut t = Term::new();
        t.feed(b"1\r\n2\r\n3\r\n4\r\n5\r\n6\r\n7");
        // rows 2..7 visible after scroll: bottom row starts "7"
        assert_eq!(t.screen.cells[ROWS - 1][0], b'7');
        assert_eq!(t.screen.cells[0][0], b'2');
    }

    #[test]
    fn backspace_moves_left() {
        let mut t = Term::new();
        t.feed(b"ab\x08");
        assert_eq!(t.screen.cx, 1);
    }

    #[test]
    fn cup_sets_cursor() {
        let mut t = Term::new();
        t.feed(b"\x1b[2;3H");
        assert_eq!((t.screen.cx, t.screen.cy), (2, 1));
    }

    #[test]
    fn erase_line_to_end_clears_tail() {
        let mut t = Term::new();
        t.feed(b"abcde\r\x1b[C\x1b[C\x1b[K"); // print, CR, right x2, erase-to-end
        assert_eq!(t.screen.cells[0][0], b'a');
        assert_eq!(t.screen.cells[0][1], b'b');
        assert_eq!(t.screen.cells[0][2], b' ');
    }

    #[test]
    fn unknown_csi_sgr_is_ignored() {
        let mut t = Term::new();
        t.feed(b"\x1b[31mred");
        assert_eq!(&row_str(&t.screen, 0)[..3], "red");
        assert_eq!(t.screen.cx, 3);
    }
}
```

- [ ] **Step 2: Add the module**

Modify `examples/terminal/src/main.rs`:

```rust
//! Terminal: an interactive BusyBox shell on the OLED, typed on the pad grid.
mod term;

fn main() {}
```

- [ ] **Step 3: Run the tests to verify they fail then pass**

Run: `cargo test -p terminal term`
Expected: PASS (all eight `term::tests`).

- [ ] **Step 4: Commit**

```bash
git add examples/terminal/src/term.rs examples/terminal/src/main.rs
git commit -m "feat(terminal): 25x6 vte-backed screen model"
```

---

### Task 6: `render` — draw the screen onto the OLED

**Files:**
- Create: `examples/terminal/src/render.rs`
- Modify: `examples/terminal/src/main.rs` (add `mod render;`)

**Interfaces:**
- Consumes: `term::Screen`, `deluge_linux_ui::OledTarget`.
- Produces: `render::render(&term::Screen, &mut OledTarget)`; `render::render_exited(&term::Screen, i32, &mut OledTarget)`.

- [ ] **Step 1: Write the failing tests**

Create `examples/terminal/src/render.rs`:

```rust
//! Draw the `term::Screen` onto the OLED with a 5×7 mono font (25×6 grid),
//! plus an underline cursor.

use crate::term::{Screen, COLS, ROWS};
use deluge_linux_ui::OledTarget;
use embedded_graphics::{
    mono_font::{ascii::FONT_5X7, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};

const CELL_W: i32 = 5;
const CELL_H: i32 = 7;

fn draw_text(screen: &Screen, oled: &mut OledTarget) {
    let style = MonoTextStyle::new(&FONT_5X7, BinaryColor::On);
    for (r, row) in screen.cells.iter().enumerate() {
        let s: String = row.iter().map(|&b| b as char).collect();
        let _ = Text::with_baseline(&s, Point::new(0, r as i32 * CELL_H), style, Baseline::Top)
            .draw(oled);
    }
}

fn draw_cursor(screen: &Screen, oled: &mut OledTarget) {
    let cx = screen.cx.min(COLS - 1) as i32;
    let cy = screen.cy.min(ROWS - 1) as i32;
    let _ = Rectangle::new(
        Point::new(cx * CELL_W, cy * CELL_H + CELL_H - 1),
        Size::new(CELL_W as u32, 1),
    )
    .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
    .draw(oled);
}

pub fn render(screen: &Screen, oled: &mut OledTarget) {
    oled.clear_frame();
    draw_text(screen, oled);
    draw_cursor(screen, oled);
}

pub fn render_exited(screen: &Screen, status: i32, oled: &mut OledTarget) {
    oled.clear_frame();
    draw_text(screen, oled);
    // Overwrite the bottom row with the footer.
    let y = (ROWS as i32 - 1) * CELL_H;
    let _ = Rectangle::new(Point::new(0, y), Size::new(128, CELL_H as u32))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(oled);
    let code = (status >> 8) & 0xff; // WEXITSTATUS
    let msg = format!("[exit {}] ENTER=restart", code);
    let style = MonoTextStyle::new(&FONT_5X7, BinaryColor::On);
    let _ = Text::with_baseline(&msg, Point::new(0, y), style, Baseline::Top).draw(oled);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cursor_underline_is_set() {
        let screen = Screen::new(); // cursor at (0,0)
        let mut oled = OledTarget::new();
        render(&screen, &mut oled);
        // Underline row0 is at y=6 -> fb byte 6*16, x=0..5 -> top 5 bits set.
        assert_eq!(oled.frame()[6 * 16] & 0b1111_1000, 0b1111_1000);
    }

    #[test]
    fn a_char_lights_pixels_in_its_row() {
        let mut screen = Screen::new();
        screen.cells[1][2] = b'A';
        let mut oled = OledTarget::new();
        render(&screen, &mut oled);
        // Row 1 spans fb y=7..14 -> bytes 7*16..14*16.
        assert!(oled.frame()[7 * 16..14 * 16].iter().any(|&b| b != 0));
    }
}
```

- [ ] **Step 2: Add the module**

Modify `examples/terminal/src/main.rs` to add `mod render;` after `mod term;`.

- [ ] **Step 3: Run the tests**

Run: `cargo test -p terminal render`
Expected: PASS (both `render::tests`).

- [ ] **Step 4: Commit**

```bash
git add examples/terminal/src/render.rs examples/terminal/src/main.rs
git commit -m "feat(terminal): render screen + cursor + exit footer to OLED"
```

---

### Task 7: `keys` — input → pty bytes

**Files:**
- Create: `examples/terminal/src/keys.rs`
- Modify: `examples/terminal/src/main.rs` (add `mod keys;`)

**Interfaces:**
- Consumes: `deluge_linux_ui::deluge_grid_toolkit::KeyPress`; `deluge_linux_ui::controls`.
- Produces: `keys::EV_BUTTON = 1`, `keys::EV_ENCODER = 2`; `keys::keypress_bytes(KeyPress) -> Option<Vec<u8>>`; `keys::button_bytes(u8) -> Option<&'static [u8]>`; `keys::encoder_bytes(u8, i16) -> Vec<u8>`.

- [ ] **Step 1: Write the failing tests**

Create `examples/terminal/src/keys.rs`:

```rust
//! Map decoded input to the byte stream written to the shell's PTY.

use deluge_linux_ui::controls;
use deluge_linux_ui::deluge_grid_toolkit::KeyPress;

/// Event kinds (mirror `include/deluge/input.h`).
pub const EV_BUTTON: u8 = 1;
pub const EV_ENCODER: u8 = 2;

/// Bytes for a resolved keyboard key. `Shift` returns `None` (the caller toggles
/// shift state instead of writing anything).
pub fn keypress_bytes(k: KeyPress) -> Option<Vec<u8>> {
    match k {
        KeyPress::Char(c) => {
            let mut b = [0u8; 4];
            Some(c.encode_utf8(&mut b).as_bytes().to_vec())
        }
        KeyPress::Backspace => Some(vec![0x7f]),
        KeyPress::Enter => Some(vec![b'\r']),
        KeyPress::Shift => None,
    }
}

/// Control byte(s) for a front-panel button id (on a `value == 1` press).
pub fn button_bytes(id: u8) -> Option<&'static [u8]> {
    match id {
        controls::button::BACK => Some(&[0x03]),          // Ctrl-C
        controls::button::SAVE => Some(&[0x04]),          // Ctrl-D (EOF)
        controls::button::LOAD => Some(&[0x09]),          // Tab
        controls::button::AFFECT_ENTIRE => Some(&[0x1b]), // Esc
        _ => None,
    }
}

/// Arrow-key escapes for an encoder rotation (`delta` signed detents).
pub fn encoder_bytes(id: u8, delta: i16) -> Vec<u8> {
    if delta == 0 {
        return Vec::new();
    }
    let arrow: &[u8] = match id {
        controls::encoder::SELECT => if delta > 0 { b"\x1b[A" } else { b"\x1b[B" }, // up / down
        controls::encoder::TEMPO => if delta > 0 { b"\x1b[C" } else { b"\x1b[D" },  // right / left
        _ => return Vec::new(),
    };
    arrow.repeat(delta.unsigned_abs() as usize)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn char_encodes_utf8() {
        assert_eq!(keypress_bytes(KeyPress::Char('a')).unwrap(), b"a");
        assert_eq!(keypress_bytes(KeyPress::Char('/')).unwrap(), b"/");
    }

    #[test]
    fn enter_is_cr_backspace_is_del() {
        assert_eq!(keypress_bytes(KeyPress::Enter).unwrap(), b"\r");
        assert_eq!(keypress_bytes(KeyPress::Backspace).unwrap(), vec![0x7f]);
    }

    #[test]
    fn shift_writes_nothing() {
        assert!(keypress_bytes(KeyPress::Shift).is_none());
    }

    #[test]
    fn back_is_ctrl_c() {
        assert_eq!(button_bytes(controls::button::BACK), Some(&[0x03u8][..]));
        assert_eq!(button_bytes(controls::button::LOAD), Some(&[0x09u8][..]));
        assert_eq!(button_bytes(controls::button::PLAY), None);
    }

    #[test]
    fn encoder_arrows_repeat_by_delta() {
        assert_eq!(encoder_bytes(controls::encoder::SELECT, 1), b"\x1b[A");
        assert_eq!(encoder_bytes(controls::encoder::TEMPO, -2), b"\x1b[D\x1b[D");
        assert!(encoder_bytes(controls::encoder::SELECT, 0).is_empty());
    }
}
```

- [ ] **Step 2: Add the module**

Modify `examples/terminal/src/main.rs` to add `mod keys;`.

- [ ] **Step 3: Run the tests**

Run: `cargo test -p terminal keys`
Expected: PASS (five `keys::tests`).

- [ ] **Step 4: Commit**

```bash
git add examples/terminal/src/keys.rs examples/terminal/src/main.rs
git commit -m "feat(terminal): map keyboard/buttons/encoders to pty bytes"
```

---

### Task 8: `pty` — devpts setup + forkpty shell

**Files:**
- Create: `examples/terminal/src/pty.rs`
- Modify: `examples/terminal/src/main.rs` (add `mod pty;`)

**Interfaces:**
- Produces: `pty::ensure_devpts()`; `pty::spawn(cols: u16, rows: u16) -> io::Result<Pty>`; `pty::spawn_cmd(prog: &str, argv: &[&str], cols: u16, rows: u16) -> io::Result<Pty>`; `Pty { pub master: RawFd, pub child: libc::pid_t }` with `write_all(&self, &[u8]) -> io::Result<()>`, `read(&self, &mut [u8]) -> io::Result<usize>`, `try_reap(&self) -> Option<i32>`.

- [ ] **Step 1: Write the failing test**

Create `examples/terminal/src/pty.rs`:

```rust
//! Spawn a child on a pseudo-terminal and talk to it over the master fd.
//!
//! The appliance rootfs does not mount `/dev/pts`; [`ensure_devpts`] mounts it
//! (idempotently) before [`spawn`]. `forkpty` then opens `/dev/ptmx` (present on
//! the auto-mounted devtmpfs) and allocates a slave.

use std::ffi::CString;
use std::io;
use std::os::unix::io::RawFd;
use std::ptr;

pub struct Pty {
    pub master: RawFd,
    pub child: libc::pid_t,
}

/// Best-effort mount of devpts at `/dev/pts`. Ignores "already exists / mounted".
pub fn ensure_devpts() {
    let dir = CString::new("/dev/pts").unwrap();
    unsafe {
        libc::mkdir(dir.as_ptr(), 0o755);
    }
    let src = CString::new("devpts").unwrap();
    let fstype = CString::new("devpts").unwrap();
    let opts = CString::new("mode=0620,ptmxmode=0666").unwrap();
    unsafe {
        libc::mount(
            src.as_ptr(),
            dir.as_ptr(),
            fstype.as_ptr(),
            0,
            opts.as_ptr() as *const libc::c_void,
        );
    }
}

pub fn spawn(cols: u16, rows: u16) -> io::Result<Pty> {
    spawn_cmd("/bin/sh", &["sh", "-i"], cols, rows)
}

pub fn spawn_cmd(prog: &str, argv: &[&str], cols: u16, rows: u16) -> io::Result<Pty> {
    let mut master: libc::c_int = 0;
    let ws = libc::winsize { ws_row: rows, ws_col: cols, ws_xpixel: 0, ws_ypixel: 0 };
    let pid = unsafe { libc::forkpty(&mut master, ptr::null_mut(), ptr::null(), &ws) };
    if pid < 0 {
        return Err(io::Error::last_os_error());
    }
    if pid == 0 {
        // Child: die if the parent (launcher-killed) dies, set TERM, exec.
        unsafe {
            libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGKILL as libc::c_ulong, 0, 0, 0);
            let term = CString::new("TERM").unwrap();
            let vt = CString::new("vt100").unwrap();
            libc::setenv(term.as_ptr(), vt.as_ptr(), 1);

            let cprog = CString::new(prog).unwrap();
            let cargs: Vec<CString> = argv.iter().map(|a| CString::new(*a).unwrap()).collect();
            let mut ptrs: Vec<*const libc::c_char> = cargs.iter().map(|c| c.as_ptr()).collect();
            ptrs.push(ptr::null());
            libc::execv(cprog.as_ptr(), ptrs.as_ptr());
            libc::_exit(127);
        }
    }
    Ok(Pty { master, child: pid })
}

impl Pty {
    pub fn write_all(&self, mut buf: &[u8]) -> io::Result<()> {
        while !buf.is_empty() {
            let n = unsafe { libc::write(self.master, buf.as_ptr() as *const libc::c_void, buf.len()) };
            if n < 0 {
                return Err(io::Error::last_os_error());
            }
            buf = &buf[n as usize..];
        }
        Ok(())
    }

    pub fn read(&self, buf: &mut [u8]) -> io::Result<usize> {
        let n = unsafe { libc::read(self.master, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        if n < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(n as usize)
        }
    }

    /// Reap the child if it exited; returns its raw wait status if so.
    pub fn try_reap(&self) -> Option<i32> {
        let mut status: libc::c_int = 0;
        let r = unsafe { libc::waitpid(self.child, &mut status, libc::WNOHANG) };
        if r == self.child {
            Some(status)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn cat_echoes_input() {
        // On the dev host /dev/pts is already mounted, so no ensure_devpts().
        let pty = spawn_cmd("/bin/cat", &["cat"], 25, 6).expect("spawn cat");
        pty.write_all(b"hello\n").expect("write");

        let mut acc = String::new();
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline && !acc.contains("hello") {
            let mut buf = [0u8; 256];
            if let Ok(n) = pty.read(&mut buf) {
                if n > 0 {
                    acc.push_str(&String::from_utf8_lossy(&buf[..n]));
                }
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        assert!(acc.contains("hello"), "pty did not echo; got {:?}", acc);
        unsafe { libc::kill(pty.child, libc::SIGKILL); }
    }
}
```

- [ ] **Step 2: Add the module**

Modify `examples/terminal/src/main.rs` to add `mod pty;`.

- [ ] **Step 3: Run the test**

Run: `cargo test -p terminal pty`
Expected: PASS (`cat_echoes_input`). Requires `/bin/cat` and a mounted `/dev/pts` on the host (both standard on Linux dev machines).

- [ ] **Step 4: Commit**

```bash
git add examples/terminal/src/pty.rs examples/terminal/src/main.rs
git commit -m "feat(terminal): forkpty shell spawn + devpts setup"
```

---

### Task 9: `main` — wire everything together

**Files:**
- Modify: `examples/terminal/src/main.rs`

**Interfaces:**
- Consumes: everything above.

- [ ] **Step 1: Replace `main.rs` with the full wiring**

Replace the body of `examples/terminal/src/main.rs` (keep the `mod` lines) with:

```rust
//! Terminal: an interactive BusyBox shell on the OLED, typed on the pad grid.
//! Launched by the launcher; the SHIFT+TRIPLETS+LEARN kill-chord exits.

mod keys;
mod pty;
mod render;
mod term;

use deluge_linux::{Deluge, Event};
use deluge_linux_ui::deluge_grid_toolkit::imode::{GridUi, PadInput};
use deluge_linux_ui::deluge_grid_toolkit::{KeyPress, KeyboardLayout, TextKeyboardComponent};
use deluge_linux_ui::{pads, OledTarget, PadTarget};
use std::sync::mpsc::{self, Sender};
use std::sync::{Arc, Mutex};
use term::Term;

enum Msg {
    Input(Event),
    TermChanged,
    ShellExited(i32),
}

/// A running shell: the pty, the shared screen model, and its reader thread.
struct Session {
    pty: Arc<pty::Pty>,
    term: Arc<Mutex<Term>>,
}

impl Session {
    fn start(tx: &Sender<Msg>) -> Session {
        let pty = Arc::new(pty::spawn(term::COLS as u16, term::ROWS as u16).expect("spawn shell"));
        let term = Arc::new(Mutex::new(Term::new()));
        let (rp, rt, txr) = (pty.clone(), term.clone(), tx.clone());
        std::thread::spawn(move || {
            let mut buf = [0u8; 1024];
            loop {
                match rp.read(&mut buf) {
                    Ok(0) => {
                        let st = rp.try_reap().unwrap_or(0);
                        let _ = txr.send(Msg::ShellExited(st));
                        break;
                    }
                    Ok(n) => {
                        rt.lock().unwrap().feed(&buf[..n]);
                        let _ = txr.send(Msg::TermChanged);
                    }
                    Err(_) => {
                        let st = rp.try_reap().unwrap_or(0);
                        let _ = txr.send(Msg::ShellExited(st));
                        break;
                    }
                }
            }
        });
        Session { pty, term }
    }
}

fn now_ms() -> u32 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: writing a timespec we own; CLOCK_MONOTONIC always exists.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    (ts.tv_sec as u64 * 1000 + ts.tv_nsec as u64 / 1_000_000) as u32
}

fn relight_pads(
    ui: &mut GridUi,
    keyboard: &TextKeyboardComponent,
    shift: bool,
    input: PadInput,
    pad_target: &mut PadTarget,
    dlg: &mut Deluge,
) -> Option<KeyPress> {
    let key = ui.run(now_ms(), input, |f| keyboard.show(f, shift)).painted().flatten();
    pad_target.blit(ui.grid());
    let _ = pad_target.flush(dlg);
    key
}

fn main() {
    let mut dlg = match Deluge::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("terminal: deluge_open failed: {e}");
            std::process::exit(1);
        }
    };

    let (tx, rx) = mpsc::channel::<Msg>();
    let tx_in = tx.clone();
    if let Err(e) = dlg.input_start(move |ev| {
        let _ = tx_in.send(Msg::Input(ev));
    }) {
        eprintln!("terminal: input_start failed: {e}");
        std::process::exit(1);
    }

    pty::ensure_devpts();
    let mut session = Session::start(&tx);

    let mut oled = OledTarget::new();
    let mut pad_target = PadTarget::new();
    let keyboard = TextKeyboardComponent::new(KeyboardLayout::Qwerty);
    let mut ui = GridUi::new();
    let mut shift = false;
    let mut exited: Option<i32> = None;

    // Initial paint: light the keyboard on the pads, draw the empty screen.
    relight_pads(&mut ui, &keyboard, shift, PadInput::new(), &mut pad_target, &mut dlg);
    render::render(&session.term.lock().unwrap().screen, &mut oled);
    let _ = oled.flush(&mut dlg);

    for msg in rx {
        match msg {
            Msg::Input(ev) if pads::pad_from_event(&ev).is_some() => {
                let input = pads::pad_input(&ev);
                let key = relight_pads(&mut ui, &keyboard, shift, input, &mut pad_target, &mut dlg);
                let Some(k) = key else { continue };
                match k {
                    KeyPress::Shift => shift = !shift,
                    KeyPress::Enter if exited.is_some() => {
                        session = Session::start(&tx);
                        exited = None;
                        render::render(&session.term.lock().unwrap().screen, &mut oled);
                        let _ = oled.flush(&mut dlg);
                    }
                    _ if exited.is_some() => {} // ignore other keys once exited
                    other => {
                        if let Some(bytes) = keys::keypress_bytes(other) {
                            let _ = session.pty.write_all(&bytes);
                        }
                        shift = false; // sticky one-shot
                    }
                }
            }
            Msg::Input(ev) if ev.kind == keys::EV_BUTTON && ev.value == 1 => {
                if let Some(b) = keys::button_bytes(ev.id) {
                    let _ = session.pty.write_all(b);
                }
            }
            Msg::Input(ev) if ev.kind == keys::EV_ENCODER => {
                let b = keys::encoder_bytes(ev.id, ev.value);
                if !b.is_empty() {
                    let _ = session.pty.write_all(&b);
                }
            }
            Msg::Input(_) => {}
            Msg::TermChanged => {
                if exited.is_none() {
                    render::render(&session.term.lock().unwrap().screen, &mut oled);
                    let _ = oled.flush(&mut dlg);
                }
            }
            Msg::ShellExited(status) => {
                exited = Some(status);
                render::render_exited(&session.term.lock().unwrap().screen, status, &mut oled);
                let _ = oled.flush(&mut dlg);
            }
        }
    }
}
```

- [ ] **Step 2: Build the bare app**

Run:
```bash
export DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-0.1.0
cargo xtask bare terminal
```
Expected: PASS — `target/bare/terminal` produced.

- [ ] **Step 3: Commit**

```bash
git add examples/terminal/src/main.rs
git commit -m "feat(terminal): main event loop wiring the pty shell to OLED + pads"
```

- [ ] **Step 4: On-device manual verification**

Copy `target/bare/terminal` to the card's `/LINUX/APPS/`, boot the launcher, select `TERMINAL`, and confirm:
- The QWERTY keyboard lights on the pad grid; the two new symbol keys (`/`, `\`) and `;` are lit.
- Typing letters appears on the OLED; the underline cursor tracks; the `ash` prompt renders.
- `ls /`, `echo hi`, `dmesg | head`, and a path with `/` all work; shift gives capitals and `|`/`>`/`<`.
- The `BACK` button (`Ctrl-C`) interrupts `yes` or `sleep 30`.
- The `SELECT` encoder recalls previous commands (up/down history).
- `exit` (or `SAVE` = Ctrl-D) shows `[exit N] ENTER=restart`; pressing Enter starts a fresh shell.
- The kill-chord (`SHIFT+TRIPLETS+LEARN`, ~1s) returns to the launcher; a later `ps` from a new shell shows no orphaned `sh`.

---

### Task 10: Document the terminal app

**Files:**
- Modify: `docs/building-an-app.md`

- [ ] **Step 1: Add a Terminal section**

Append after the Snake section in `docs/building-an-app.md`:

```markdown
## Terminal (PTY shell bare app)

`examples/terminal` is a bare `/LINUX/APPS/` app that runs an interactive
BusyBox shell (`/bin/sh -i`) on a pseudo-terminal and renders it as a 25×6
character terminal on the OLED. Input comes from the on-pad QWERTY keyboard
(`deluge-grid-toolkit`'s `TextKeyboardComponent`, driven through the
`deluge-linux-ui::pads` bridge) plus front-panel keys. Build the bare binary:

    cargo xtask bare terminal      # -> target/bare/terminal  for /LINUX/APPS/

Copy `target/bare/terminal` onto the card's `/LINUX/APPS/`, boot the launcher,
and select `TERMINAL`. Type on the pad keyboard; `Enter` runs the line. Shift
(on-pad or the `SHIFT` button) gives capitals and symbols. `BACK` sends Ctrl-C,
`SAVE` sends Ctrl-D (EOF), `LOAD` sends Tab; the `SELECT` encoder scrolls shell
history. On `exit`/Ctrl-D the OLED shows `[exit N] ENTER=restart`.
SHIFT+TRIPLETS+LEARN (held ~1s) returns to the launcher.

Notes:
- The app mounts `devpts` at `/dev/pts` itself on startup (the appliance rootfs
  does not) before `forkpty`; the child sets `PR_SET_PDEATHSIG(SIGKILL)` so the
  shell is reaped if the launcher force-kills the app.
- The reduced pad keyboard exposes the shell-critical symbols
  (`/ \ | ; : ? < > _ " ! @ # $ % ^ & * ( )`); `~` and `` ` `` are omitted (no
  free pad — the last slot collides with the SHIFT pad). A future symbols layer
  could add them.
- Full-screen TUI programs (`vi`, `top`) are out of scope: only a minimal
  VT100 subset (cursor moves + erase) is emulated; colours are ignored (1bpp).
```

- [ ] **Step 2: Commit**

```bash
git add docs/building-an-app.md
git commit -m "docs: document the terminal PTY-shell example"
```

---

## Self-Review

**Spec coverage:**
- Pad bridge (`PadTarget` + `Event`→`PadInput` + re-export) → Tasks 1–2. ✓
- Widget shift fix + symbol keys → Task 3. ✓
- `pty` (devpts + forkpty + PDEATHSIG + winsize + reap) → Task 8. ✓
- `term` (vte + minimal CSI subset) → Task 5. ✓
- `render` (FONT_5X7 25×6 + cursor + exit footer) → Task 6. ✓
- `keys` (KeyPress/button/encoder → bytes) → Task 7. ✓
- `main` (threads, restart-on-Enter, best-effort I/O) → Task 9. ✓
- Build/packaging + docs → Tasks 4, 10. ✓
- Testing (host units for each pure module + pty host integration + on-device checklist) → embedded in each task. ✓

**Placeholder scan:** No TBD/TODO; every code step shows complete code. ✓

**Type consistency:** `pads::pad_input`/`pad_from_event`, `term::{Term,Screen,COLS,ROWS}`, `render::{render,render_exited}`, `keys::{keypress_bytes,button_bytes,encoder_bytes,EV_BUTTON,EV_ENCODER}`, `pty::{ensure_devpts,spawn,spawn_cmd,Pty}`, and the grid-toolkit `resolve`/`shifted` names are used identically across tasks. `KeyPress`/`TextKeyboardComponent`/`GridUi`/`PadInput` come from the `deluge-linux-ui` re-export in every consumer. ✓

**Known deviations from the spec (approved during planning):** the widget delivers `/ \ ; ` and shifted forms `? | :` (3 keys, not 4 — the 4th pad slot collides with SHIFT), dropping `~`/`` ` ``. Documented in Task 10.
