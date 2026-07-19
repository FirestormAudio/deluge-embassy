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
        assert_eq!(g.state(), State::GameOver);
        let mut t = OledTarget::new();
        render(&g, &mut t);
        assert!(any_set_in_rows(t.frame(), 14, 43));
    }
}
