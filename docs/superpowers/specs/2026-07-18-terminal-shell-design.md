# Terminal (PTY shell) — design

**Date:** 2026-07-18
**Status:** Approved design, pending implementation plan

## Summary

A bare `/LINUX/APPS/` example app named **`terminal`** that runs a real
interactive BusyBox shell (`/bin/sh -i`) on a pseudo-terminal, renders the
shell's output as a small character terminal on the OLED, and takes input from
the on-pad QWERTY keyboard plus a few front-panel keys for control characters.

It is the reference app for two things at once:

1. **Driving the pad grid from a Linux app** via the shared
   `deluge-grid-toolkit` widgets — something no Linux example does today (snake
   hand-rolls its 432-byte pad frame). This requires a new, reusable bridge in
   `deluge-linux-ui`.
2. **Hosting a real subprocess on a PTY** and emulating enough of a terminal to
   display its output on a 1bpp panel.

Launched by the launcher; exited by the standard `SHIFT+TRIPLETS+LEARN`
kill-chord (enforced by the launcher, not the app).

## Goals

- A working interactive shell: type `ls`, `echo`, `cat`, `dmesg`, pipes; see
  real output on the OLED.
- The on-pad QWERTY keyboard (`TextKeyboardComponent`) is the primary input.
- `Ctrl-C` interrupts a running command (e.g. `yes`, `sleep`).
- Establish the `deluge-linux-ui` pad bridge so future pad-grid Linux apps
  reuse it instead of hand-rolling buffers.

## Non-goals (YAGNI)

- Full VT100/xterm fidelity. We implement a **minimal but correct** subset.
- Colours/attributes (the panel is 1bpp — SGR is parsed and discarded).
- Full-screen / alt-screen TUI programs (`vi`, `top`). Deferred, noted below.
- Scrollback history viewing. Deferred (encoder-driven scrollback is a natural
  follow-up).
- Tab-completion polish beyond forwarding the `Tab` byte to the shell.

## Target facts (verified in this checkout)

- **OLED:** 128×43, 1bpp, driven by `deluge_linux_ui::OledTarget` (an
  `embedded-graphics` `DrawTarget`).
- **Pads:** 18×8 RGB, 432-byte frame, stride 54 (`row*54 + col*3`), via
  `Deluge::pads_write`. Pad presses arrive through `Deluge::input_start` as
  `Event { kind: EV_PAD (0), x, y, value, pressure }`.
- **Shell:** `/bin/sh` → BusyBox `ash` is present in the base rootfs.
- **Kernel:** `CONFIG_UNIX98_PTYS=y`, `CONFIG_DEVTMPFS=y`,
  `CONFIG_DEVTMPFS_MOUNT=y` — `/dev` is a writable devtmpfs with `/dev/ptmx`
  available at runtime.
- **Gap:** the appliance rootfs does **not** mount `/dev/pts`. The app must
  `mkdir /dev/pts` and `mount -t devpts …` at startup (idempotent) before
  `forkpty()` will work.
- **Font:** `embedded-graphics` `FONT_5X7` has a 5×7 cell → **25 columns × 6
  rows** on the panel (uses 125×42 of 128×43).

## Architecture

```
 pad press ─► Event(EV_PAD) ─► PadInput ─► TextKeyboardComponent ─► KeyPress
                                                                       │
 panel button ─► Event(EV_BUTTON) ─────────────────────────► control bytes
 encoder ──────► Event(EV_ENCODER) ────────────────────────► arrow escapes
                                                                       │
                                                                       ▼
                                                            write() ─► PTY master
                                                                       │
                                                             /bin/sh -i (child)
                                                                       │
                                                          echo + program output
                                                                       ▼
                              read() ◄──────────────────────── PTY master
                                 │
                                 ▼
                        vte::Parser ─► Term grid (COLS×ROWS + cursor)
                                 │
                                 ▼
                    render ─► OledTarget ─► oled_write
```

Two threads plus the shell process:

- **Input thread** (SDK-owned, via `input_start`): pushes every `Event` to an
  `mpsc` channel.
- **PTY-reader thread:** blocks on `read(master)`, feeds bytes to the `vte`
  parser which mutates a `Mutex<Term>`, then nudges the main loop to repaint.
- **Main thread:** owns the `Deluge` handle. Selects between input events and
  repaint nudges; translates input to master writes; renders OLED + pads.
- **Shell:** `/bin/sh -i`, child of `forkpty`, with `PR_SET_PDEATHSIG(SIGKILL)`
  so it dies if the launcher `SIGKILL`s the app (no orphan).

`Deluge` is not `Sync`-shared: only the main thread calls `oled_write` /
`pads_write` / `input_start`. The PTY reader touches only the master fd and the
`Mutex<Term>`.

## New reusable bridge: `deluge-linux-ui::pads`

This is the "the keyboard UI is part of the ui crate" piece. A new module
symmetric with the existing `oled` module.

- **`PadTarget`** — owns the 432-byte pad RGB frame. Packs a
  `deluge_grid_toolkit::Grid` (or `GridRgb`) into the stride-54 layout and
  exposes `flush(&mut Deluge)` → `pads_write`. The pad analogue of `OledTarget`.
  - `fn blit(&mut self, grid: &Grid)` — iterate 18×8 cells, write
    `buf[row*54 + col*3 .. +3] = [r, g, b]`.
  - `fn flush(&mut self, dlg: &mut Deluge) -> Result<(), Error>`.
- **Input adapter** — build a `deluge_grid_toolkit::imode::PadInput` from Deluge
  events:
  - `fn pad_from_event(ev: &Event) -> Option<(Pad, bool)>` — for `kind == EV_PAD`,
    returns `(Pad::new(row = ev.y, col = ev.x), pressed = ev.value != 0)`.
  - A small `PadInputBuilder` that accumulates press/release events into a
    `PadInput` (and maintains the `held` `PadMask`) between repaints.
- **Re-export** `pub use deluge_grid_toolkit;` so apps reach
  `TextKeyboardComponent`, `Grid`, `imode::{GridUi, Frame, PadInput}`, `Pad`,
  `Color`, etc. from the ui crate.
- **Cargo:** add `deluge-grid-toolkit` (path dep, pulls in `deluge-bsp`) to
  `deluge-linux-ui`.

The bridge is deliberately app-agnostic: any future pad-grid Linux app uses
`PadTarget` + the input adapter instead of hand-rolling a 432-byte buffer.

## The `terminal` app

New workspace member `examples/terminal`, modules:

### `pty` — subprocess on a pseudo-terminal
- `ensure_devpts()` — if `/dev/pts` is not already a mountpoint,
  `mkdir("/dev/pts")` then `mount("devpts", "/dev/pts", "devpts", 0,
  "mode=0620,ptmxmode=0666")`. Treat `EBUSY` / already-mounted as success.
- `spawn(cols, rows) -> Pty` — `forkpty()`:
  - **child:** `prctl(PR_SET_PDEATHSIG, SIGKILL)`; `setenv("TERM", "vt100")`;
    `execvp("/bin/sh", ["sh", "-i"])`.
  - **parent:** set window size via `ioctl(master, TIOCSWINSZ, {rows, cols})`;
    return `Pty { master_fd, child_pid }`.
- `write_all(&self, bytes)` and a raw `master_fd()` for the reader thread.
- `reap()` — `waitpid(child, WNOHANG)`; report exit status for the footer.

### `term` — terminal-emulator model
- `Term { cols: 25, rows: 6, cells: [[u8; 25]; 6], cursor: (col, row) }`.
- Implements `vte::Perform`:
  - `print(c)` — write at cursor, advance; wrap to next line at the right edge.
  - `execute(byte)` — `\n` line-feed (scroll up when past the last row), `\r`
    carriage-return, `\b`/`0x7f` backspace (move left), `\t` tab-to-next-8,
    `0x07` bell (ignored).
  - `csi_dispatch` — minimal subset: cursor position (`CUP`/`HVP`), relative
    moves (`CUU`/`CUD`/`CUF`/`CUB`), erase-in-line (`EL` 0/1/2), erase-in-display
    (`ED` 0/1/2). `SGR` and everything else are accepted and ignored.
  - `esc_dispatch` / `hook` / `put` / `osc` — no-ops (enough to not corrupt on
    unknown sequences).
- Non-ASCII / control bytes we don't model are dropped, never rendered raw.
- A `dirty` flag so the main loop only repaints when the grid changed.

### `render` — OLED drawing
- Draw each cell with `MonoTextStyle::new(&FONT_5X7, BinaryColor::On)` onto the
  `OledTarget`, origin per `(col*5, row*7 + baseline)`.
- Draw a block/underline cursor at the cursor cell (XOR or filled rect).
- When the shell has exited, draw a footer line: `[exited N] press ⏎`.

### Widget fix: shift-aware `TextKeyboardComponent` (in `deluge-grid-toolkit`)

The widget today takes `shift_held` only for **drawing** — `show()`/`char_at`
ignore it and the char table stores **upper-case** letters, so shift produces no
change and there is no way to type shell-critical symbols (`/`, `|`, `>`, `<`,
`~`, `;`, `:`). This is fixed in the shared widget (benefits every consumer),
not worked around in the Linux app:

- `char_at` / `show` become **shift-aware**: letters are lower-case unshifted,
  upper-case when `shift_held` (works for all layouts).
- A shifted-symbol map for the QWERTY layout: digits → `! @ # $ % ^ & * ( )`,
  `-`→`_`, `'`→`"`, `,`→`<`, `.`→`>`.
- The four empty QWERTY slots gain the standard missing symbol keys so a shell
  is usable: `/`(shift `?`), `;`(shift `:`), `\`(shift `|`), `` ` ``(shift `~`).
  `KEYBOARD_CHARS`, `draw()`, and `char_at` bounds are updated to render/handle
  them. AZERTY/QWERTZ/Dvorak get letter-case shift + the digit-symbol row; their
  punctuation is left as-is (QWERTY is the default and the layout the example
  uses).
- No new `KeyPress` variant: space is already `KeyPress::Char(' ')`.

### `keys` — input mapping (app side)
- **Pad keyboard:** run the grid immediate-mode frame each repaint —
  `GridUi::run(now_ms, pad_input, |f| keyboard.show(f, shift_held))` — and map
  the returned `KeyPress` (the widget has already applied shift to the char):
  - `Char(c)` → the UTF-8 byte(s) of `c`. `Backspace` → `0x7f`.
    `Enter` → `\r`. `Shift` → toggle `shift_held` (sticky until the next key).
  - Then `PadTarget::blit(ui.grid())` + `flush` so the keyboard is lit on pads.
- **Front-panel buttons** (`kind == EV_BUTTON`, ids from
  `deluge_linux_ui::controls`) → control bytes:
  - `BACK` → `Ctrl-C` (`0x03`)
  - `SAVE` → `Ctrl-D` (`0x04`, EOF)
  - `LOAD` → `Tab` (`0x09`)
  - `SHIFT` → mirror the sticky-shift toggle (in addition to on-pad shift)
  - (`AFFECT_ENTIRE` → `Esc` `0x1b` — optional, for prefixing escapes)
- **Encoders** (`kind == EV_ENCODER`, `controls::encoder`) → arrow escapes for
  shell history / line editing:
  - `SELECT` rotation → up/down arrows (`ESC[A` / `ESC[B`) — history
  - `TEMPO` rotation → left/right arrows (`ESC[C` / `ESC[D`) — cursor
- The exact button assignments are provisional and finalised during
  implementation against the on-device layout; the mapping table lives in one
  place in `keys.rs`.

### `main` — wiring
1. `Deluge::open()`; `input_start(cb)` → `mpsc<Event>`.
2. `pty::ensure_devpts()`; `pty::spawn(25, 6)`.
3. Spawn the PTY-reader thread: loop `read(master)` → `parser.advance(&mut term, &buf)`
   → on change, mark dirty + wake the main loop (a second `mpsc` or a
   `Condvar`/self-pipe). On `read == 0` / `EIO`, set the "shell exited" state and
   wake.
4. Main loop:
   - Drain input events: pad events → `keys` → master writes + pad relight;
     button/encoder events → control/arrow bytes.
   - On dirty/term-changed → `render` to OLED + `flush`.
   - On "shell exited" → render footer; on next `Enter`, re-`spawn` a fresh
     shell and clear the grid (**footer + restart on Enter**, per design
     decision).

## Data flow (one keystroke)

`pad press → Event(EV_PAD, x, y, value) → PadInput::press(Pad) →
TextKeyboardComponent::show → KeyPress::Char('l') → write "l" to master → sh
echoes "l" → read → vte print → Term cell (0,cursor) = 'l' → render → OLED`.

## Error handling

- `ensure_devpts` / `forkpty` / `open(master)` failure → `eprintln!` + `exit(1)`;
  the launcher shows its `… CRASHED` toast.
- `read(master)` returns `0` or `EIO` → shell exited: footer + restart-on-Enter,
  app stays alive.
- `write(master)` `EPIPE` → treat as shell exit (same path).
- `Deluge` I/O (`oled_write`/`pads_write`) → best-effort: log and continue, as
  snake does (`let _ = …`).
- `PR_SET_PDEATHSIG(SIGKILL)` in the child guarantees no orphaned shell when the
  launcher force-kills the app via the kill-chord.

## Testing

**Host unit tests** (pure, no hardware — same style as snake's `render_pads`
tests):

- `term`: feed byte sequences to `vte::Parser` + `Term`, assert grid contents
  and cursor for: plain print, right-edge wrap, `\n` scroll at bottom, `\r`,
  `\b`, `\t`, `CUP`, `CUU/CUD/CUF/CUB`, `EL 0/1/2`, `ED 2`. Assert unknown CSI
  (e.g. an `SGR` colour) is a no-op that doesn't corrupt the grid or move the
  cursor.
- `keys`: table-driven `KeyPress`/button/encoder → exact byte sequence
  (including shift-case and the arrow escapes).
- `deluge-linux-ui::pads`: `Grid` → 432-byte buffer packing — assert byte
  offsets and stride for a few known cells (mirrors snake's `cell()` test).
  `pad_from_event` — `EV_PAD` with `x,y,value` → `(Pad, pressed)`; non-pad
  events → `None`.
- `deluge-grid-toolkit` widget: `char_at` unshifted → lower-case letter, shifted
  → upper-case; shifted digit → symbol (`'1'`→`'!'`); the new symbol keys
  (`/ ; \ `` ` ``) and their shifted forms (`? : | ~`); space stays `' '`.

**On-device manual checklist** (add to `docs/building-an-app.md`):

- The QWERTY keyboard lights up on the pad grid; shift keys change colour.
- Typing letters appears on the OLED on the current line; the block cursor
  tracks; the shell prompt renders.
- `ls`, `echo hi`, `dmesg | head`, and a pipe all produce correct output.
- `Ctrl-C` (mapped button) interrupts `yes` / `sleep 30`.
- `SELECT` encoder recalls previous commands (up/down arrow history).
- `exit` / `Ctrl-D` shows `[exited N] press ⏎`; Enter starts a fresh shell.
- Kill-chord (`SHIFT+TRIPLETS+LEARN`, ~1s) returns to the launcher; the shell
  process is gone (no orphan — check with a later `ps`).

## Build / packaging

- Add `examples/terminal` to the workspace `members` in `Cargo.toml` (kept out
  of `default-members`, like the other examples that link libdeluge).
- Bare binary: `cargo xtask bare terminal` → `target/bare/terminal` for
  `/LINUX/APPS/` (copied to the card manually, like snake).
- Dependencies: `deluge-linux`, `deluge-linux-ui` (→ `deluge-grid-toolkit`,
  `deluge-bsp`), `embedded-graphics`, `vte`, `libc`. `vte` (Alacritty's parser)
  is small and vendored via cargo — acceptable per project preference.
- **Cross-repo:** the shift-aware widget fix lands in the sibling `deluge-sdk`
  repo (`crates/deluge-grid-toolkit`), committed there; the rest lands in this
  repo.
- Docs: add a "Terminal (PTY shell)" section to `docs/building-an-app.md`
  mirroring the Snake section (build command, controls, kill-chord, manual
  checklist).

## Open questions / deferred

- **Scrollback:** an encoder-scrolled history buffer above the live screen — a
  natural next iteration, out of MVP scope.
- **Alt-screen:** supporting `vi`/`top` needs alt-screen buffer + more CSI/DEC
  private modes; deferred.
- **Final button map:** the `keys.rs` control-byte assignments are provisional
  and will be confirmed against the physical layout during implementation.
