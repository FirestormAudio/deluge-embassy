# Snake Pad-Grid Demo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `examples/snake`, a playable Snake game on the Deluge 16×8 pad grid steered by the tempo encoder, as the SDK's reference "real" non-appliance (`/LINUX/APPS/`) app.

**Architecture:** A pure, hardware-free state machine (`game.rs`) is driven by a thin IO shell (`main.rs`) that forwards input events over an `mpsc` channel and ticks on `recv_timeout`. Two pure renderers turn game state into a 432-byte pad RGB frame (`render_pads.rs`) and an OLED frame (`ui.rs`). This mirrors the existing `examples/launcher` split exactly.

**Tech Stack:** Rust 2021, `deluge-linux` (safe libdeluge wrappers), `deluge-linux-ui` (OLED bridge + control-id mirror), `deluge-ui-toolkit` (`Title`, text), `embedded-graphics` 0.8, `libc` 0.2. Cross target `armv7-unknown-linux-musleabihf`, `crt-static`.

## Global Constraints

- **Play field:** the main **16×8** grid (cols 0–15). Sidebar cols 16–17 stay dark. `COLS = 16`, `ROWS = 8`.
- **Pad frame:** 432 bytes, 18×8, 24bpp RGB, row stride **54**. Pad `(col,row)` → offset `row*54 + col*3`, bytes `[R,G,B]`.
- **Steering:** tempo encoder rotation `controls::encoder::TEMPO (1)`, **relative**: `value > 0` (CW) → turn **right**, `value < 0` (CCW) → turn **left**. A 180° reversal into the neck is disallowed. Restart on `controls::encoder_button::TEMPO (13)` press.
- **Speed:** tick starts at **180 ms**, shortens **10 ms per food eaten**, floored at **90 ms**.
- **Death:** move into a wall or any occupied snake cell (tail treated as solid — no tail-follow special case).
- **Colors:** head `[0,120,0]`, body `[0,20,0]`, food `[120,0,0]`, empty `[0,0,0]`.
- **No `rand` crate.** RNG is a seeded xorshift64; seed from `CLOCK_MONOTONIC` via `libc::clock_gettime`.
- **Bare app:** static; `examples/snake` is a workspace member but NOT a default-member (like `rust-app`).
- **Host tests** run with `DELUGE_SDK_ROOT=$PWD/stage` (the snake bin links `deluge-linux`).
- **Exit** is the launcher's SHIFT+TRIPLETS+LEARN kill-chord; the app has no in-app exit.

Spec: `docs/superpowers/specs/2026-07-17-snake-demo-design.md`.

## File Structure

- `examples/snake/Cargo.toml` — crate manifest (bin `snake`).
- `examples/snake/src/game.rs` — pure state machine: `Dir`, `State`, `StepResult`, `Game`. All game rules + RNG. Fully unit-tested, no IO.
- `examples/snake/src/render_pads.rs` — `frame(&Game) -> [u8; 432]`. Pure.
- `examples/snake/src/ui.rs` — `render(&Game, &mut OledTarget)`. Pure over the frame buffer.
- `examples/snake/src/main.rs` — device open, input→mpsc, tick loop, repaint. IO only.
- `Cargo.toml` (root) — add `"examples/snake"` to `[workspace].members`.
- `docs/building-an-app.md` — add a "Snake (pad-grid bare app)" subsection.

`controls::encoder::TEMPO` and `controls::encoder_button::TEMPO` already exist in `crates/deluge-linux-ui/src/controls.rs` (mirrored ahead of this work) — no controls change is needed.

---

### Task 1: Scaffold crate + game skeleton with initial state

**Files:**
- Create: `examples/snake/Cargo.toml`
- Create: `examples/snake/src/game.rs`
- Create: `examples/snake/src/main.rs` (temporary stub so the bin compiles)
- Modify: `Cargo.toml` (root workspace members)
- Test: in `examples/snake/src/game.rs` (`#[cfg(test)] mod tests`)

**Interfaces:**
- Produces: `game::{COLS, ROWS, Dir, State, StepResult, Game}`. `Game::new(seed: u64) -> Game`, `Game::head(&self) -> (u8,u8)`, `Game::food(&self) -> (u8,u8)`, `Game::state(&self) -> State`, `Game::score(&self) -> u32`, `Game::body(&self) -> impl Iterator<Item=(u8,u8)>`, `Game::tick_period(&self) -> std::time::Duration`. `Dir::{Up,Down,Left,Right}`, `State::{Playing,GameOver}`, `StepResult::{Moved,Ate,Died}`.

- [ ] **Step 1: Create the crate manifest**

`examples/snake/Cargo.toml`:
```toml
[package]
name = "snake"
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

- [ ] **Step 2: Add the crate to the workspace (member only, not default)**

In the root `Cargo.toml`, add `"examples/snake"` to `[workspace].members` (leave `default-members` unchanged). Result:
```toml
members = ["crates/deluge-sys", "crates/deluge-linux", "crates/deluge-linux-ui", "xtask", "examples/rust-app", "examples/launcher", "examples/snake"]
```

- [ ] **Step 3: Write a temporary main stub so the bin target compiles**

`examples/snake/src/main.rs`:
```rust
mod game;

fn main() {
    // Replaced with the real event loop in Task 7.
}
```

- [ ] **Step 4: Write the failing test for initial state**

`examples/snake/src/game.rs` — start the file with only the test module plus the type names it needs (implementation added in Step 6):
```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_game_starts_length_3_heading_right_playing() {
        let g = Game::new(1);
        assert_eq!(g.head(), (8, 4));
        assert_eq!(g.body().count(), 3);
        assert_eq!(g.dir, Dir::Right);
        assert_eq!(g.state(), State::Playing);
        assert_eq!(g.score(), 0);
    }

    #[test]
    fn new_game_food_is_not_on_the_snake() {
        // Try several seeds; food must always land on an empty cell.
        for seed in 1..50u64 {
            let g = Game::new(seed);
            assert!(!g.body().any(|c| c == g.food()), "seed {seed}");
        }
    }
}
```

- [ ] **Step 5: Run the test to verify it fails to compile**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake`
Expected: FAIL — `cannot find type Game` / `Dir` (not yet defined).

- [ ] **Step 6: Write the minimal implementation above the test module**

Prepend to `examples/snake/src/game.rs`:
```rust
//! Pure Snake state machine — no IO, no timing. Driven by `main.rs`.

use std::collections::VecDeque;
use std::time::Duration;

pub const COLS: u8 = 16;
pub const ROWS: u8 = 8;

const START_TICK_MS: u64 = 180;
const MIN_TICK_MS: u64 = 90;
const TICK_STEP_MS: u64 = 10;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Dir {
    Up,
    Down,
    Left,
    Right,
}

impl Dir {
    /// (dcol, drow). Row 0 is the top, so Up decreases the row.
    pub fn delta(self) -> (i8, i8) {
        match self {
            Dir::Up => (0, -1),
            Dir::Down => (0, 1),
            Dir::Left => (-1, 0),
            Dir::Right => (1, 0),
        }
    }

    pub fn turn_cw(self) -> Dir {
        match self {
            Dir::Up => Dir::Right,
            Dir::Right => Dir::Down,
            Dir::Down => Dir::Left,
            Dir::Left => Dir::Up,
        }
    }

    pub fn turn_ccw(self) -> Dir {
        match self {
            Dir::Up => Dir::Left,
            Dir::Left => Dir::Down,
            Dir::Down => Dir::Right,
            Dir::Right => Dir::Up,
        }
    }

    pub fn reverse(self) -> Dir {
        match self {
            Dir::Up => Dir::Down,
            Dir::Down => Dir::Up,
            Dir::Left => Dir::Right,
            Dir::Right => Dir::Left,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum State {
    Playing,
    GameOver,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StepResult {
    Moved,
    Ate,
    Died,
}

pub struct Game {
    snake: VecDeque<(u8, u8)>, // head at the front
    dir: Dir,                  // heading for the next step
    moved: Dir,                // heading used by the last committed step
    food: (u8, u8),
    score: u32,
    tick_ms: u64,
    state: State,
    rng: u64,
}

impl Game {
    pub fn new(seed: u64) -> Self {
        let mut snake = VecDeque::new();
        snake.push_back((8, 4)); // head
        snake.push_back((7, 4));
        snake.push_back((6, 4));
        let mut g = Self {
            snake,
            dir: Dir::Right,
            moved: Dir::Right,
            food: (0, 0),
            score: 0,
            tick_ms: START_TICK_MS,
            state: State::Playing,
            rng: seed | 1, // xorshift needs a nonzero state
        };
        g.spawn_food();
        g
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn score(&self) -> u32 {
        self.score
    }

    pub fn head(&self) -> (u8, u8) {
        *self.snake.front().expect("snake is never empty")
    }

    pub fn food(&self) -> (u8, u8) {
        self.food
    }

    pub fn body(&self) -> impl Iterator<Item = (u8, u8)> + '_ {
        self.snake.iter().copied()
    }

    pub fn tick_period(&self) -> Duration {
        Duration::from_millis(self.tick_ms)
    }

    fn rng_next(&mut self) -> u64 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng = x;
        x
    }

    /// Place food on a uniformly-random empty cell. If the grid is full
    /// (a win), end the game rather than loop forever.
    fn spawn_food(&mut self) {
        let total = COLS as usize * ROWS as usize;
        if self.snake.len() >= total {
            self.state = State::GameOver;
            return;
        }
        let empties = total - self.snake.len();
        let mut k = (self.rng_next() % empties as u64) as usize;
        for r in 0..ROWS {
            for c in 0..COLS {
                if self.snake.contains(&(c, r)) {
                    continue;
                }
                if k == 0 {
                    self.food = (c, r);
                    return;
                }
                k -= 1;
            }
        }
    }
}
```

- [ ] **Step 7: Run the tests to verify they pass**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake`
Expected: PASS (2 tests).

- [ ] **Step 8: Commit**

```bash
git add examples/snake/Cargo.toml examples/snake/src/game.rs examples/snake/src/main.rs Cargo.toml
git commit -m "snake: scaffold crate + game skeleton (initial state, food spawn)"
```

---

### Task 2: Relative steering (tempo turns, no reversal)

**Files:**
- Modify: `examples/snake/src/game.rs`
- Test: same file

**Interfaces:**
- Produces: `Game::turn(&mut self, detents: i16)` — applies `|detents|` quarter-turns (CW if positive, CCW if negative) to `dir`, but never sets `dir` to the reverse of the last committed movement (`moved`). No-op unless `Playing`.

- [ ] **Step 1: Write the failing tests**

Add to the `tests` module in `game.rs`:
```rust
#[test]
fn single_cw_detent_turns_right_from_right_to_down() {
    let mut g = Game::new(1);
    g.turn(1); // CW
    assert_eq!(g.dir, Dir::Down);
}

#[test]
fn single_ccw_detent_turns_left_from_right_to_up() {
    let mut g = Game::new(1);
    g.turn(-1); // CCW
    assert_eq!(g.dir, Dir::Up);
}

#[test]
fn two_cw_detents_cannot_reverse_into_the_neck() {
    // Moving Right; two CW detents would be Right->Down->Left, but Left is the
    // reverse of the committed heading, so it is blocked and we stay Down.
    let mut g = Game::new(1);
    g.turn(2);
    assert_eq!(g.dir, Dir::Down);
}

#[test]
fn turn_is_ignored_when_game_over() {
    let mut g = Game::new(1);
    g.state = State::GameOver;
    let before = g.dir;
    g.turn(1);
    assert_eq!(g.dir, before);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake turn`
Expected: FAIL — `no method named turn`.

- [ ] **Step 3: Implement `turn`**

Add inside `impl Game`:
```rust
    /// Turn the head relative to its current heading: `detents > 0` = clockwise
    /// (right), `< 0` = counter-clockwise (left). A turn that would face the
    /// reverse of the last committed movement is dropped, so the snake can
    /// never U-turn into its own neck between steps.
    pub fn turn(&mut self, detents: i16) {
        if self.state != State::Playing {
            return;
        }
        let cw = detents > 0;
        for _ in 0..detents.unsigned_abs() {
            let candidate = if cw {
                self.dir.turn_cw()
            } else {
                self.dir.turn_ccw()
            };
            if candidate != self.moved.reverse() {
                self.dir = candidate;
            }
        }
    }
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake turn`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add examples/snake/src/game.rs
git commit -m "snake: relative tempo steering with no-reversal guard"
```

---

### Task 3: Stepping — movement, death, growth, speed-up

**Files:**
- Modify: `examples/snake/src/game.rs`
- Test: same file

**Interfaces:**
- Produces: `Game::step(&mut self) -> StepResult`. Advances one tick: sets `moved = dir`, moves the head, returns `Died` on wall/self collision (and sets `GameOver`), `Ate` when it lands on food (grows, `score += 1`, tick shortens by 10 ms floored at 90, respawns food), else `Moved` (pops the tail).

- [ ] **Step 1: Write the failing tests**

Add to the `tests` module:
```rust
#[test]
fn step_moves_head_one_cell_and_keeps_length() {
    let mut g = Game::new(1);
    g.food = (0, 0); // out of the way
    let len = g.body().count();
    assert_eq!(g.step(), StepResult::Moved);
    assert_eq!(g.head(), (9, 4));
    assert_eq!(g.body().count(), len);
}

#[test]
fn stepping_into_a_wall_ends_the_game() {
    let mut g = Game::new(1);
    g.food = (0, 0);
    g.dir = Dir::Up;
    g.moved = Dir::Up;
    // From row 4 heading Up: rows 3,2,1,0 are Moved, then row -1 Died.
    for _ in 0..4 {
        assert_eq!(g.step(), StepResult::Moved);
    }
    assert_eq!(g.step(), StepResult::Died);
    assert_eq!(g.state(), State::GameOver);
}

#[test]
fn stepping_into_the_body_ends_the_game() {
    let mut g = Game::new(1);
    // Head at (5,5); the cell directly above, (5,4), is part of the body.
    g.snake = VecDeque::from(vec![(5, 5), (6, 5), (6, 4), (5, 4)]);
    g.food = (0, 0);
    g.dir = Dir::Up;
    g.moved = Dir::Left; // Up is not a reverse of Left
    assert_eq!(g.step(), StepResult::Died);
    assert_eq!(g.state(), State::GameOver);
}

#[test]
fn eating_food_grows_scores_speeds_up_and_respawns() {
    let mut g = Game::new(1);
    g.food = (9, 4); // directly ahead of the head at (8,4) heading Right
    let len = g.body().count();
    let tick = g.tick_ms;
    assert_eq!(g.step(), StepResult::Ate);
    assert_eq!(g.body().count(), len + 1);
    assert_eq!(g.score(), 1);
    assert_eq!(g.tick_ms, tick - TICK_STEP_MS);
    assert_ne!(g.food, (9, 4));
    assert!(!g.body().any(|c| c == g.food));
}

#[test]
fn tick_never_drops_below_the_floor() {
    let mut g = Game::new(1);
    g.tick_ms = MIN_TICK_MS;
    g.food = (9, 4);
    assert_eq!(g.step(), StepResult::Ate);
    assert_eq!(g.tick_ms, MIN_TICK_MS);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake step`
Expected: FAIL — `no method named step`.

- [ ] **Step 3: Implement `step`**

Add inside `impl Game`:
```rust
    /// Advance one tick. Returns what happened this tick.
    pub fn step(&mut self) -> StepResult {
        if self.state != State::Playing {
            return StepResult::Died;
        }
        self.moved = self.dir;

        let (hc, hr) = self.head();
        let (dc, dr) = self.dir.delta();
        let nc = hc as i8 + dc;
        let nr = hr as i8 + dr;
        if nc < 0 || nc >= COLS as i8 || nr < 0 || nr >= ROWS as i8 {
            self.state = State::GameOver;
            return StepResult::Died;
        }
        let next = (nc as u8, nr as u8);
        // The tail is treated as solid (no tail-follow special case).
        if self.snake.contains(&next) {
            self.state = State::GameOver;
            return StepResult::Died;
        }

        self.snake.push_front(next);
        if next == self.food {
            self.score += 1;
            self.tick_ms = self.tick_ms.saturating_sub(TICK_STEP_MS).max(MIN_TICK_MS);
            self.spawn_food();
            StepResult::Ate
        } else {
            self.snake.pop_back();
            StepResult::Moved
        }
    }
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake step`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add examples/snake/src/game.rs
git commit -m "snake: step() — movement, wall/self death, growth, speed-up"
```

---

### Task 4: Restart + uniform food on a near-full grid

**Files:**
- Modify: `examples/snake/src/game.rs`
- Test: same file

**Interfaces:**
- Produces: `Game::restart(&mut self)` — resets to a fresh start state, continuing the RNG sequence so the new food differs. `spawn_food` already exists (Task 1); this task pins its uniform-empty-cell behaviour.

- [ ] **Step 1: Write the failing tests**

Add to the `tests` module:
```rust
#[test]
fn restart_from_game_over_restores_the_start_state() {
    let mut g = Game::new(1);
    g.food = (0, 0);
    g.dir = Dir::Up;
    g.moved = Dir::Up;
    for _ in 0..10 {
        if g.step() == StepResult::Died {
            break;
        }
    }
    assert_eq!(g.state(), State::GameOver);

    g.restart();
    assert_eq!(g.state(), State::Playing);
    assert_eq!(g.head(), (8, 4));
    assert_eq!(g.body().count(), 3);
    assert_eq!(g.score(), 0);
    assert_eq!(g.tick_ms, START_TICK_MS);
}

#[test]
fn spawn_food_picks_the_only_empty_cell() {
    let mut g = Game::new(1);
    // Fill every cell except (15,7).
    let mut s = VecDeque::new();
    for r in 0..ROWS {
        for c in 0..COLS {
            if !(c == 15 && r == 7) {
                s.push_back((c, r));
            }
        }
    }
    g.snake = s;
    g.spawn_food();
    assert_eq!(g.food, (15, 7));
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake restart`
Expected: FAIL — `no method named restart`.

- [ ] **Step 3: Implement `restart`**

Add inside `impl Game`:
```rust
    /// Reset to a fresh game, continuing the RNG so the new food differs.
    pub fn restart(&mut self) {
        *self = Game::new(self.rng);
    }
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake`
Expected: PASS (all game tests, 13 total).

- [ ] **Step 5: Commit**

```bash
git add examples/snake/src/game.rs
git commit -m "snake: restart() + uniform food on a near-full grid"
```

---

### Task 5: Pad renderer

**Files:**
- Create: `examples/snake/src/render_pads.rs`
- Modify: `examples/snake/src/main.rs` (add `mod render_pads;`)
- Test: in `render_pads.rs`

**Interfaces:**
- Consumes: `game::Game` (`head`, `body`, `food`).
- Produces: `render_pads::frame(game: &Game) -> [u8; 432]`.

- [ ] **Step 1: Declare the module**

In `examples/snake/src/main.rs`, add below `mod game;`:
```rust
mod render_pads;
```

- [ ] **Step 2: Write the failing test**

`examples/snake/src/render_pads.rs`:
```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::game::Game;
    use std::collections::VecDeque;

    fn cell(frame: &[u8; 432], col: u8, row: u8) -> [u8; 3] {
        let o = row as usize * 54 + col as usize * 3;
        [frame[o], frame[o + 1], frame[o + 2]]
    }

    #[test]
    fn draws_head_body_food_and_leaves_the_rest_dark() {
        let mut g = Game::new(1);
        g.snake = VecDeque::from(vec![(2, 1), (1, 1)]); // head (2,1), body (1,1)
        g.food = (5, 3);
        let f = frame(&g);
        assert_eq!(cell(&f, 2, 1), [0, 120, 0], "head bright green");
        assert_eq!(cell(&f, 1, 1), [0, 20, 0], "body dim green");
        assert_eq!(cell(&f, 5, 3), [120, 0, 0], "food red");
        assert_eq!(cell(&f, 0, 0), [0, 0, 0], "empty cell dark");
        assert_eq!(cell(&f, 16, 0), [0, 0, 0], "sidebar col 16 dark");
        assert_eq!(cell(&f, 17, 7), [0, 0, 0], "sidebar col 17 dark");
    }
}
```
Note: the test sets `g.snake` / `g.food` directly, which requires these fields to be visible to `render_pads`. They are private to `game.rs`, so add `pub(crate)` visibility in Step 4-prep below.

- [ ] **Step 3: Run the test to verify it fails**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake frame`
Expected: FAIL — `frame` not found / private fields.

- [ ] **Step 4: Make the fields crate-visible, then implement `frame`**

In `game.rs`, change the two fields the renderer test manipulates to `pub(crate)`:
```rust
    snake: VecDeque<(u8, u8)>, // head at the front
```
becomes
```rust
    pub(crate) snake: VecDeque<(u8, u8)>, // head at the front
```
and
```rust
    food: (u8, u8),
```
becomes
```rust
    pub(crate) food: (u8, u8),
```

Then write `examples/snake/src/render_pads.rs` (above the test module):
```rust
//! Turn game state into the 432-byte Deluge pad RGB frame (18×8, stride 54).

use crate::game::Game;

const HEAD: [u8; 3] = [0, 120, 0];
const BODY: [u8; 3] = [0, 20, 0];
const FOOD: [u8; 3] = [120, 0, 0];

fn put(frame: &mut [u8; 432], col: u8, row: u8, rgb: [u8; 3]) {
    let o = row as usize * 54 + col as usize * 3;
    frame[o..o + 3].copy_from_slice(&rgb);
}

/// Body first (dim), then the head overwrites its cell (bright), then food.
pub fn frame(game: &Game) -> [u8; 432] {
    let mut frame = [0u8; 432];
    for (c, r) in game.body() {
        put(&mut frame, c, r, BODY);
    }
    let (hc, hr) = game.head();
    put(&mut frame, hc, hr, HEAD);
    let (fc, fr) = game.food();
    put(&mut frame, fc, fr, FOOD);
    frame
}
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake frame`
Expected: PASS (1 test).

- [ ] **Step 6: Commit**

```bash
git add examples/snake/src/render_pads.rs examples/snake/src/main.rs examples/snake/src/game.rs
git commit -m "snake: pad-grid renderer (head/body/food, sidebar dark)"
```

---

### Task 6: OLED renderer

**Files:**
- Create: `examples/snake/src/ui.rs`
- Modify: `examples/snake/src/main.rs` (add `mod ui;`)
- Test: in `ui.rs`

**Interfaces:**
- Consumes: `game::{Game, State}`, `deluge_linux_ui::OledTarget`, `deluge_ui_toolkit::{components::Title, text::{draw_text, Font, TextStyle}}`.
- Produces: `ui::render(game: &Game, target: &mut OledTarget)`.

- [ ] **Step 1: Declare the module**

In `examples/snake/src/main.rs`, add below `mod render_pads;`:
```rust
mod ui;
```

- [ ] **Step 2: Write the failing render-smoke tests**

`examples/snake/src/ui.rs`:
```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::game::{Game, StepResult};

    /// True if any pixel is set in OLED rows [y0, y1). The header fills the top
    /// ~11 rows, so asserting on rows below it proves the body drew.
    fn any_set_in_rows(frame: &[u8], y0: usize, y1: usize) -> bool {
        frame[y0 * 16..y1 * 16].iter().any(|&b| b != 0)
    }

    #[test]
    fn playing_draws_score_below_the_header() {
        let g = Game::new(1);
        let mut t = OledTarget::new();
        render(&g, &mut t);
        assert!(any_set_in_rows(t.frame(), 14, 43));
    }

    #[test]
    fn game_over_draws_below_the_header() {
        let mut g = Game::new(1);
        // Drive straight until we hit a wall (<= 8 steps going one way).
        for _ in 0..40 {
            if g.step() == StepResult::Died {
                break;
            }
        }
        assert_eq!(g.state(), crate::game::State::GameOver);
        let mut t = OledTarget::new();
        render(&g, &mut t);
        assert!(any_set_in_rows(t.frame(), 14, 43));
    }
}
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake ui`
Expected: FAIL — `render` not found.

- [ ] **Step 4: Implement `render`**

Write `examples/snake/src/ui.rs` (above the test module):
```rust
//! Render game state to the OLED: canonical `Title` + score, and a game-over
//! screen. Mirrors `examples/launcher/src/ui.rs`.

use crate::game::{Game, State};
use deluge_linux_ui::OledTarget;
use deluge_ui_toolkit::components::Title;
use deluge_ui_toolkit::text::{draw_text, Font, TextStyle};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};

pub fn render(game: &Game, target: &mut OledTarget) {
    target.clear_frame();
    let _ = Title::new("SNAKE").with_separator(true).draw(target);

    let style = TextStyle::new(Font::FontApple).with_color(BinaryColor::On);
    match game.state() {
        State::Playing => {
            let _ = draw_text(target, &format!("SCORE {}", game.score()), Point::new(3, 16), style);
        }
        State::GameOver => {
            let _ = draw_text(target, "GAME OVER", Point::new(3, 16), style);
            let _ = draw_text(target, &format!("SCORE {}", game.score()), Point::new(3, 26), style);
            let _ = draw_text(target, "TEMPO = RESTART", Point::new(3, 36), style);
        }
    }
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake ui`
Expected: PASS (2 tests).

- [ ] **Step 6: Commit**

```bash
git add examples/snake/src/ui.rs examples/snake/src/main.rs
git commit -m "snake: OLED renderer (SNAKE title, score, game-over screen)"
```

---

### Task 7: Event loop wiring (`main.rs`)

**Files:**
- Modify: `examples/snake/src/main.rs` (replace the stub with the real loop)

**Interfaces:**
- Consumes: `deluge_linux::{Deluge, Event}`, `deluge_linux_ui::{controls, OledTarget}`, `game::{Game, State}`, `render_pads::frame`, `ui::render`.
- Produces: the `snake` binary. (No unit test — IO-only; verified by a clean build.)

- [ ] **Step 1: Write the full `main.rs`**

Replace `examples/snake/src/main.rs` entirely with:
```rust
//! Snake on the Deluge pad grid — a reference non-appliance (`/LINUX/APPS/`)
//! app. Steer with the tempo encoder; the launcher's kill-chord exits.

mod game;
mod render_pads;
mod ui;

use deluge_linux::{Deluge, Event};
use deluge_linux_ui::{controls, OledTarget};
use game::{Game, State};
use std::sync::mpsc::{self, RecvTimeoutError};

// Event kinds (mirror deluge-linux / include/deluge/input.h).
const EV_BUTTON: u8 = 1;
const EV_ENCODER: u8 = 2;

/// Seed the RNG from the monotonic clock (no `rand` crate).
fn seed() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: writing a valid timespec we own; CLOCK_MONOTONIC always exists.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    (((ts.tv_sec as u64) << 32) ^ (ts.tv_nsec as u64)) | 1
}

fn repaint(game: &Game, dlg: &mut Deluge, oled: &mut OledTarget) {
    let _ = dlg.pads_write(&render_pads::frame(game));
    ui::render(game, oled);
    let _ = oled.flush(dlg);
}

fn main() {
    let mut dlg = match Deluge::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("snake: deluge_open failed: {e}");
            std::process::exit(1);
        }
    };

    let (tx, rx) = mpsc::channel::<Event>();
    if let Err(e) = dlg.input_start(move |ev| {
        let _ = tx.send(ev);
    }) {
        eprintln!("snake: input_start failed: {e}");
        std::process::exit(1);
    }

    let mut game = Game::new(seed());
    let mut oled = OledTarget::new();
    repaint(&game, &mut dlg, &mut oled);

    loop {
        let mut dirty = false;
        match rx.recv_timeout(game.tick_period()) {
            Ok(ev) => {
                if ev.kind == EV_ENCODER
                    && ev.id == controls::encoder::TEMPO
                    && game.state() == State::Playing
                {
                    game.turn(ev.value);
                    dirty = true;
                } else if ev.kind == EV_BUTTON
                    && ev.id == controls::encoder_button::TEMPO
                    && ev.value == 1
                    && game.state() == State::GameOver
                {
                    game.restart();
                    dirty = true;
                }
            }
            Err(RecvTimeoutError::Timeout) => {
                game.step();
                dirty = true;
            }
            Err(RecvTimeoutError::Disconnected) => break,
        }
        if dirty {
            repaint(&game, &mut dlg, &mut oled);
        }
    }
}
```

- [ ] **Step 2: Verify the whole crate builds and all tests pass**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p snake`
Expected: PASS (all 16 tests: 13 game + 1 render_pads + 2 ui). Also run `DELUGE_SDK_ROOT="$PWD/stage" cargo build -p snake` and expect a clean host build.

- [ ] **Step 3: Commit**

```bash
git add examples/snake/src/main.rs
git commit -m "snake: event loop wiring (input channel, tempo steer, tick, repaint)"
```

---

### Task 8: Cross-build the bare artifact + document it

**Files:**
- Modify: `docs/building-an-app.md`

**Interfaces:** none (build + docs).

- [ ] **Step 1: Cross-build the bare binary**

Run:
```bash
export DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-0.1.0
cargo xtask bare snake
```
Expected: ends with `deluge-mkimage` success and writes `target/bare/snake` (a static armv7 musl binary, validated by `--bare`). Confirm with:
```bash
file target/bare/snake
```
Expected: `ELF 32-bit LSB ... ARM ... statically linked`.

- [ ] **Step 2: Add a docs subsection**

In `docs/building-an-app.md`, after the existing "The launcher (reference appliance)" section, add:
```markdown
## Snake (pad-grid bare app)

`examples/snake` is the reference *non-appliance* app: a static binary for
`/LINUX/APPS/` that the launcher exec's. It reads the pad grid and tempo
encoder and drives the pad LEDs + OLED. Build the bare binary:

    cargo xtask bare snake      # -> target/bare/snake for /LINUX/APPS/

Copy `target/bare/snake` onto the SD card's `/LINUX/APPS/`, boot the launcher,
and select `SNAKE`. Steer with the tempo encoder (turn = relative left/right);
eating food grows the snake and speeds it up; hitting a wall or yourself ends
the game — press the tempo encoder to restart. SHIFT+TRIPLETS+LEARN (held ~1s)
returns to the launcher.

> The bare binary must be copied to the card manually — the dev-mode USB upload
> only streams the appliance `LINUX.ELF` into RAM; it cannot place files on the
> card.
```

- [ ] **Step 3: Commit**

```bash
git add docs/building-an-app.md
git commit -m "docs: snake bare-app build + on-device flow"
```

---

## Self-Review

**Spec coverage:**
- Play field 16×8, sidebar dark → Task 1 constants + Task 5 renderer/test. ✓
- Tempo relative steering, no reversal → Task 2. ✓
- Tick 180→90 by −10/food → Task 3 (`eating…speeds_up`, `tick_never_drops_below_the_floor`). ✓
- Die on wall/self (tail solid) → Task 3. ✓
- Food on random empty cell → Task 1 (`spawn_food`) + Task 4 near-full-grid test. ✓
- Restart on tempo click → Task 4 (`restart`) + Task 7 (button wiring). ✓
- Pad colors + offsets → Task 5. ✓
- OLED Title + score + game-over → Task 6. ✓
- RNG from CLOCK_MONOTONIC, no `rand` → Task 1 (`rng_next`) + Task 7 (`seed`). ✓
- Workspace member (not default) → Task 1. ✓
- `cargo xtask bare snake` + card-copy docs → Task 8. ✓
- Uses existing `controls::encoder::TEMPO` / `encoder_button::TEMPO` → Task 7. ✓
- Exit via launcher kill-chord (no in-app exit) → no code; documented Task 8. ✓

**Placeholder scan:** none — every code step is complete.

**Type consistency:** `frame(&Game) -> [u8;432]`, `render(&Game, &mut OledTarget)`, `Game::{new,turn,step,restart,head,food,body,score,state,tick_period}`, `Dir`/`State`/`StepResult` variants, and `controls::encoder::TEMPO` / `controls::encoder_button::TEMPO` are used identically across tasks. Fields `snake`/`food` are `pub(crate)` from Task 5 so the sibling-module renderer test can set them. ✓
