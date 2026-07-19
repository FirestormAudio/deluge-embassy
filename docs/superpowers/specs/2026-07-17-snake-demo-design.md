# Snake — pad-grid demo (non-appliance SDK app)

## Context

The SDK can already build **non-appliance ("bare") apps** — static binaries that
sit on the SD card under `/LINUX/APPS/` and are exec'd at runtime by the launcher
inside the generic `LINUX` image (`cargo xtask bare <app>` → `target/bare/<app>`,
or CMake `deluge_add_app`). The only existing bare example, `examples/rust-app`,
is a blink stub that proves the pipeline but exercises nothing. There is no
worked example that reads the pad grid, drives the pad LEDs, or draws the OLED
from a bare app.

This spec adds `examples/snake`: a playable Snake game on the 16×8 pad grid,
steered with the tempo encoder. It doubles as the reference "real" bare-app —
the thing a developer copies when starting their own `/LINUX/APPS/` app. No new
build machinery is required; `examples/snake` is just a new workspace member
built the same way `rust-app` is.

## Hardware surfaces used

From `crates/deluge-linux` (safe wrappers over libdeluge), confirmed against
`include/deluge/input.h` and `include/deluge/display.h`:

- **Pad LEDs** — `Deluge::pads_write(&[u8; 432])`. Frame is 18×8, 24bpp RGB,
  row stride 54 bytes (`DELUGE_PADS_BYTES = 432`). Pad `(col,row)` →
  byte offset `row*54 + col*3`, then `[R, G, B]`. Cols 0–15 = main grid,
  cols 16–17 = the two sidebar columns (left dark by this app).
- **Input** — `Deluge::input_start(cb)` delivers `Event { kind, id, value, x, y, pressure }`
  on an SDK-owned thread. `kind`: `0=PAD`, `1=BUTTON`, `2=ENCODER`, `3=CLOCK`.
- **OLED** — rendered via the existing `deluge-linux-ui` `OledTarget` bridge
  (128×43, 688 bytes) + `deluge-ui-toolkit` widgets, then `OledTarget::flush`.

### Control ids

Steering is the **tempo encoder**. The named ids live in
`crates/deluge-linux-ui/src/controls.rs`, which mirrors `deluge-bsp::controls`
(source of truth in the `deluge-sdk` repo) as four submodules
(`button`, `encoder_button`, `encoder`, `knob`):

- `controls::encoder::TEMPO = 1` — rotation id in encoder space
  (`0=SCROLL_X, 1=TEMPO, 2=MOD_0, 3=MOD_1, 4=SCROLL_Y, 5=SELECT`). Arrives as
  `Event { kind: ENCODER(2), id: 1, value: <signed detents> }`.
- `controls::encoder_button::TEMPO = 13` — the tempo shaft click, arrives as
  `Event { kind: BUTTON(1), id: 13, value: 1|0 }`.

Both are already mirrored (the full BSP table — buttons, encoder shaft-clicks,
encoder rotations, and gold-knob columns — was mirrored ahead of this work), so
Snake just consumes `controls::encoder::TEMPO` and
`controls::encoder_button::TEMPO`; no controls change is needed here.

## Gameplay

- **Field:** the main 16×8 grid = 128 cells. Sidebar cols 16–17 stay dark.
- **Start:** a length-3 snake mid-grid, heading right (+x), one food placed on a
  random empty cell. Game begins in `Playing`.
- **Steering (relative):** each tempo detent turns the head relative to its
  current heading. `value > 0` (CW) → turn right; `value < 0` (CCW) → turn left.
  Multiple detents in one event apply that many quarter-turns. Because turns are
  relative, a 180° reversal into the neck is impossible by construction.
- **Tick:** the head advances one cell per tick in the current heading.
  Tick starts at **180 ms** and shortens by **10 ms per food eaten**, floored at
  **90 ms**. The main loop's `recv_timeout` uses the game's current tick period.
- **Food / growth:** moving onto the food cell grows the snake by 1 (the tail is
  not popped that tick) and spawns new food on a uniformly-random empty cell.
- **Death (classic):** moving into a wall (next cell out of `0..16 × 0..8`) or
  into an occupied body cell ends the game → `GameOver`. (The tail cell the head
  could otherwise chase is still treated as occupied — no "tail-follow" special
  case, matching classic Snake death-on-self.)
- **Restart:** in `GameOver`, a tempo **click** resets to the start state.
  Rotation is ignored while in `GameOver`.
- **Exit:** none in-app. The launcher's SHIFT+TRIPLETS+LEARN kill-chord SIGKILLs
  the process and repaints the launcher. (No pad-clear on exit is possible under
  SIGKILL; acceptable for a demo — the next launched app repaints the pads.)

## Rendering

- **Pads** (`render_pads.rs`): produce the 432-byte frame from game state each
  tick — body dim green (e.g. `[0, 20, 0]`), head bright green (`[0, 120, 0]`),
  food red (`[120, 0, 0]`), everything else off. Sidebar columns always off.
  On `GameOver`, the final snake+food stay lit (a static "you died here" frame).
- **OLED** (`ui.rs`): reuse the `OledTarget` + `deluge-ui-toolkit` `Title`
  (`Title::new("SNAKE").with_separator(true)` — canonical text+underline header)
  plus `draw_text` for the score. `Playing`: title + `SCORE <n>` where score =
  food eaten. `GameOver`: `GAME OVER`, `SCORE <n>`, and `TEMPO = RESTART`.

## Architecture

Mirrors the launcher's split (pure state machine + thin IO shell):

- **`game.rs`** — pure, hardware-free state machine. Holds grid dims, the snake
  as a `VecDeque<(u8,u8)>` (head at front), `heading`, `food`, food count, tick
  period, `State { Playing, GameOver }`, and a small xorshift64 RNG. Methods:
  `new(seed)`, `turn(detents: i16)`, `step()` (advance one tick, returns an
  outcome), `restart()`, plus read accessors for the renderers. No `std::time`,
  no IO — fully unit-testable.
- **`render_pads.rs`** — `fn frame(game: &Game) -> [u8; 432]`. Pure.
- **`ui.rs`** — `fn render(game: &Game, target: &mut OledTarget)`. Pure over the
  frame buffer (same pattern as the launcher's `ui.rs`).
- **`main.rs`** — opens the device, seeds the RNG from `CLOCK_MONOTONIC` nanos
  (via `libc::clock_gettime`; no `rand` crate), `input_start` forwards events
  onto an `mpsc` channel, and the main loop does
  `rx.recv_timeout(game.tick_period())`:
  - `kind==ENCODER, id==controls::encoder::TEMPO (1)` while `Playing`
    → `game.turn(value)`.
  - `kind==BUTTON, id==controls::encoder_button::TEMPO (13)`, press, while
    `GameOver` → `game.restart()`.
  - timeout → `game.step()`.
  After any state change: render pads + OLED and flush both.

New crate: `examples/snake/{Cargo.toml, src/{main,game,render_pads,ui}.rs}`,
depending on `deluge-linux`, `deluge-linux-ui`, `deluge-ui-toolkit`,
`embedded-graphics`, and `libc` (as the launcher does).

## Build & deploy

- Add `"examples/snake"` to `[workspace].members` in the root `Cargo.toml`
  (leave it out of `default-members`, exactly like `rust-app`, so a bare host
  `cargo build` doesn't try to link libdeluge).
- `cargo xtask bare snake` → `target/bare/snake` (static armv7 musl binary,
  validated by `deluge-mkimage --bare`).
- On device: copy `target/bare/snake` to the SD card's `/LINUX/APPS/`, boot the
  launcher, select `SNAKE`, click to launch. **This card copy is manual** — the
  dev-mode USB upload only streams the appliance `LINUX.ELF` into RAM; it cannot
  place files on the card. (Documented as the one manual step; inherent to the
  `/LINUX/APPS/` model.)
- Add a short "Snake (pad-grid bare app)" subsection to `docs/building-an-app.md`
  showing the `cargo xtask bare snake` → card copy → launch flow.

## Testing

Headless `cargo test` (host build; needs `DELUGE_SDK_ROOT=$PWD/stage`):

- **`game.rs`**: advance moves head by heading; relative `turn` maps CW→right /
  CCW→left and cannot reverse into the neck; wall step → `GameOver`;
  self-collision step → `GameOver`; eating food grows length by 1 and increments
  score; new food never lands on an occupied cell (test on a nearly-full grid);
  tick period shortens per food and floors at 90 ms; `restart()` from `GameOver`
  restores the start state; rotation ignored in `GameOver`.
- **`render_pads.rs`**: head/body/food written at the correct `row*54+col*3`
  offsets with the expected colors; sidebar cols 16–17 all zero; off cells zero.
- **`ui.rs`**: render-smoke asserts (content drawn below the header; `GAME OVER`
  text present in the game-over state), following the launcher's `ui.rs` tests.

On-device manual check: launch from the launcher; tempo turns steer; eating
grows + speeds up; wall/self ends the game with the game-over OLED; tempo click
restarts; kill-chord returns to the launcher.

## Out of scope (YAGNI)

High scores/persistence, sound, pause, difficulty menu, multiplayer, wrap-around
walls, sidebar controls, and any in-app exit. Wall behavior is fixed to
die-on-hit; speed ramp is fixed to −10 ms/food to a 90 ms floor.
