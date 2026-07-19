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
    pub(crate) snake: VecDeque<(u8, u8)>, // head at the front
    dir: Dir,                             // heading for the next step
    moved: Dir,                           // heading used by the last committed step
    pub(crate) food: (u8, u8),
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

    /// Reset to a fresh game, continuing the RNG so the new food differs.
    pub fn restart(&mut self) {
        *self = Game::new(self.rng);
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
        // Moving Right; two CW detents would be Right->Down->Left, but Left is
        // the reverse of the committed heading, so it is blocked and we stay Down.
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
}
