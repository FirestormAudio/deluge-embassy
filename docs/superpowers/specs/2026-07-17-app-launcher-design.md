# Reference Linux appliance: the app launcher

**Date:** 2026-07-17
**Status:** approved, not yet implemented
**Repo:** `deluge-linux-sdk` (consumes the OLED UI toolkit from `deluge-sdk`)

## Problem

The [contained-ELF overlay design](2026-07-13-contained-elf-overlay-design.md)
defines two products from one compiled app: self-contained **appliance images**
(`*.ELF` in `/APPS/`, each booted directly by the Deluge app-loader) and **bare
static-musl executables** (`/LINUX/APPS/`, run at runtime by "the launcher inside
the generic `LINUX` image"). That spec deliberately deferred **the launcher's own
design — its UI, how it enumerates and execs programs from `/LINUX/APPS/`, and
what else lives under `/LINUX/`.** This is that spec.

The launcher is the reference Linux appliance app: a Rust program that browses
the SD card's app folder on the OLED with the select encoder and runs the chosen
application, with an always-available OS-level escape back to itself.

## Goal

A user boots `LINUX.ELF` from the app-loader, sees a scrollable list of the apps
in `/LINUX/APPS/` on the OLED, spins the select encoder to highlight one, clicks
to launch it, and can **always** get back to the launcher by holding
**SHIFT + TRIPLETS + LEARN** for ~1 second — no matter what the running app is
doing. It is the reference consumer of the shared OLED UI toolkit on Linux.

## Non-goals

- A general file browser. The launcher shows only the app list in
  `/LINUX/APPS/` (see "Browse scope" below). Navigating arbitrary directories or
  launching files elsewhere on the card is out of scope.
- Per-app manifests / metadata files. Apps are identified by filename only.
- Building, packing, or installing apps. That is `deluge-mkimage` /
  `deluge_add_app()` / `cargo xtask`, already specified.
- The generic `LINUX` image's rootfs, kernel, U-Boot, or runit wiring. The
  launcher plugs into the *existing* `deluge-app` runit service unchanged; it
  only has to *be* `/usr/bin/deluge-app`.
- On-device automated testing. The hardware path (real panel, real evdev, a real
  child app) is verified manually; all decision logic is host-testable.

## Platform facts this design relies on

Established by reading the SDK, recorded here so the plan does not have to
re-derive them:

- **OLED framebuffer.** `deluge-linux`'s `Deluge::oled_write(&[u8])` blits a full
  frame of exactly `DELUGE_OLED_BYTES = 688` bytes: 128×43, 1bpp, `line_length`
  16 bytes/row. This is already the *visible* area — unlike the embedded SDK's
  128×48 `Oled` (where the faceplate hides the top 5 rows and toolkit content is
  offset down by 5px), the Linux fb needs **no offset**.
- **Input.** `Deluge::input_start(cb)` spawns an SDK-owned thread that calls `cb`
  once per decoded `Event { kind, id, value, x, y, pressure }`. Event kinds:
  `PAD=0, BUTTON=1, ENCODER=2, CLOCK=3`. Button ids are `0..=35`; encoder
  rotation ids are `0..=5`.
- **No exclusive grab.** The C input layer opens each evdev node
  `O_RDONLY | O_NONBLOCK | O_CLOEXEC` and never issues `EVIOCGRAB`. Multiple
  processes therefore each receive a copy of every input event. This is what
  makes the OS-level kill chord possible: the launcher can keep the buttons
  device open and watch for the chord *while a child app also reads input*, and
  the watchdog never steals events from the app.
- **Control ids** (from `deluge-bsp::controls`, raw PIC button-id space `0..=35`):
  `LEARN = 7`, `SHIFT = 8`, `TRIPLETS = 17`. Select encoder: **rotation** is
  encoder id `5` (`encoder::SELECT`); **shaft click** is button id `31`
  (`encoder_button::SELECT`, reported as a button press).
- **UI toolkit.** `deluge-ui-toolkit` (in `deluge-sdk`) is `no_std` but links
  std for its own tests, and draws onto any `embedded-graphics`
  `DrawTarget<Color = BinaryColor>`. It ships list/menu/header/scrollbar
  components suited to a launcher.

## Design decisions (locked)

| Decision | Choice |
| --- | --- |
| Browse scope | App list only: `/LINUX/APPS/`, non-app files ignored |
| Toolkit bridge location | New reusable crate `deluge-linux-ui` |
| Kill chord activation | SHIFT+TRIPLETS+LEARN held **~1 s** |
| App naming | Filename, upper-cased. No metadata/manifest format |
| On app exit | Return to menu; silent on clean exit and on kill; transient toast on abnormal exit |

## Architecture

### Two new workspace members

**`crates/deluge-linux-ui`** — the reusable bridge between `deluge-linux` and the
OLED UI toolkit. Any Rust appliance wanting the toolkit on Linux needs this, so
it lives on its own rather than inside the launcher, and it keeps the
`embedded-graphics` dependency out of the thin `deluge-linux` binding crate.

- `OledTarget` — implements `embedded_graphics::draw_target::DrawTarget<Color =
  BinaryColor>` and `OriginDimensions` (128×43), backed by a `[u8; 688]`
  framebuffer. `draw_iter` writes bits into the packed buffer; `flush(&mut
  Deluge)` calls `oled_write(&buf)`. The exact bit order (which bit is x=0 within
  a byte; `line_length` 16) is pinned against the kernel `deluge-oled` fb driver
  / `deluge-sdk`'s `oled.rs` during implementation and locked by unit tests. No
  5px offset.
- `controls` — the named ids the launcher uses (`LEARN`, `SHIFT`, `TRIPLETS`,
  select rotation `5`, select click `31`), mirrored from `deluge-bsp::controls`
  with a test asserting the values. Mirroring (rather than depending on
  `deluge-bsp`) avoids pulling a `no_std` embedded HAL into a Linux binary; if a
  third consumer appears, extract a shared `no_std` `deluge-controls` crate.

**`examples/launcher`** — the launcher binary, placed alongside
`examples/rust-app` and kept out of `default-members` (it links static-musl
libdeluge, which a bare host `cargo build` cannot). It is built only via the
image path — it lives *inside* `LINUX.ELF`, it is not itself a `/LINUX/APPS/`
bare app.

> Placement note: `examples/launcher` matches the established worked-example
> build path (`cargo xtask image <pkg>`). It could instead be a top-level
> `crates/launcher` since it is a shipped product; this spec uses
> `examples/launcher` and the choice does not affect any other section.

### Launcher modules

Small, single-purpose modules:

- **`apps.rs`** — enumerate `/LINUX/APPS/`: keep regular, executable files only;
  display name = filename upper-cased; sort deterministically (by name).
  Returns `Vec<AppEntry { path, display_name }>`. Rescanned on every return to
  the menu, so newly-copied apps appear without a restart.
- **`ui.rs`** — render the app list via the toolkit's list/header/scrollbar
  components into an `OledTarget`. Also renders the `NO APPS`, `LAUNCHING…`, and
  transient toast states.
- **`supervisor.rs`** — `fork`/`exec` the selected app in its **own process
  group** (`setsid`). A reaper thread `waitpid`s the child and reports the exit
  status. Kill = `kill(-pgid, SIGKILL)` so any helper processes the app spawned
  are torn down too. Kill is idempotent with natural exit (the reaper reaps
  whichever happens first).
- **`killwatch.rs`** — a pure state machine tracking SHIFT/TRIPLETS/LEARN
  down/up. When all three are held it starts a ~1 s timer; if all three are still
  held at expiry it fires; any release cancels and resets. Driven by a clock
  passed in, so tests use a fake clock.
- **`main.rs`** — open `Deluge`, wire `input_start` into an `mpsc` channel, and
  run the state machine.

### State machine

- **Browsing** — the launcher owns the OLED. Select-encoder **rotation**
  (encoder id 5) moves the highlight; select-encoder **click** (button id 31)
  launches the highlighted app. The kill chord is a no-op here (nothing to kill).
  Empty list → `NO APPS` screen.
- **Running** — a child owns the OLED and opens its own input. The launcher goes
  **quiescent on the display** (draws nothing, so it never clobbers the child's
  frames) but **keeps reading buttons** to run the kill-watcher. Because evdev is
  not grabbed, both processes see every event; the watchdog is purely additive.
- Transition back to **Browsing** on either `ChildExited(status)` (from the
  reaper) or a fired kill chord. Rescan `/LINUX/APPS/` and redraw. Abnormal exit
  shows a transient toast first.

### Concurrency

Three threads, joined by one `mpsc` channel of messages into the main loop:

- **main** — state machine + display (when Browsing).
- **SDK input thread** — `input_start` callback forwards each `Event` to the
  channel.
- **reaper thread** — blocks on `waitpid(child)` and forwards
  `ChildExited(status)`.

The main loop issues `kill(-pgid, SIGKILL)` when the kill-watcher fires; the
reaper then delivers the resulting `ChildExited`.

### Data flow

```
evdev ──(no grab; copy to every reader)──> launcher's deluge_input
                                              └─ input_start cb ─> mpsc ─┐
                                                                        v
child app  <── fork/exec (own pgid) ── main loop / state machine <── mpsc
   │  owns OLED + own deluge_input while Running       │
   └── waitpid ── reaper thread ── ChildExited ────────┘  (Browsing: draw list;
                                                            kill chord -> kill(-pgid))
```

## On-exit UX

Always redraw the list on any termination. **Clean exit (code 0) and kill-chord
return silently.** Abnormal exit (non-zero code or terminating signal) shows a
brief transient toast — e.g. `SPARK crashed (139)` — then the list.

## Error handling

- `Deluge::open` fails (no hardware) → log to console, exit non-zero; the runit
  service restarts it.
- `/LINUX/APPS/` missing or empty → `NO APPS` screen; rescanned on the next
  return to Browsing.
- `exec` fails (ENOENT, not executable) → toast `CAN'T RUN <NAME>`, stay in
  Browsing.
- Incompatible / dynamically-linked binary → surfaces as an abnormal-exit toast
  via the normal reaper path.
- Kill chord in Browsing → no-op.

## Build & packaging

The launcher ships **only** via the image path (it is the app inside
`LINUX.ELF`, not a `/LINUX/APPS/` bare executable):

    export DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-<ver>
    RUSTFLAGS='-C target-feature=+crt-static' \
      cargo xtask image launcher        # target: armv7-unknown-linux-musleabihf

The canonical output name for the generic image is **`LINUX.ELF`**. Today
`cargo xtask image` derives the ELF name from the package (`RUST-APP.ELF`), so
this needs a small xtask output-name override (`-o` / explicit name); that
override is folded into the implementation. It plugs into the existing
`overlay/base/etc/service/deluge-app/run` runit service with no change to the
service.

## Testing (TDD, host-runnable)

- **`deluge-linux-ui`** — `OledTarget` pixel-packing tests (draw known shapes,
  assert framebuffer bytes); `controls` value tests mirroring
  `deluge-bsp::controls`.
- **`apps.rs`** — enumeration/filter/sort/name-derivation against a temp dir.
- **`killwatch.rs`** — chord state machine with synthetic events and a fake
  clock: held <1 s does not fire, ≥1 s fires, any release resets.
- **`supervisor.rs`** — fork real trivial children (`/bin/true`, `/bin/false`, a
  sleeper): classify clean vs non-zero vs signalled exits; SIGKILL a sleeper and
  confirm it is reaped.
- **`ui.rs`** — render-smoke test: the list renders without panicking; byte
  snapshot (mirrors the toolkit's own render-smoke tests).
- **On-device (manual)** — real panel, real evdev, and launching/killing a real
  child app.

## Open implementation details (not design blockers)

- Exact OLED 1bpp bit order within a byte — pin against the fb driver /
  `oled.rs`, lock with a test.
- Whether the reaper uses a dedicated `waitpid` thread or a `signalfd` folded
  into the main loop — either satisfies the design; pick during implementation.
