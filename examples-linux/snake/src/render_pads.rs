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
