//! QWERTY/AZERTY/QWERTZ/Dvorak text keyboard widget.
//!
//! Renders a text-input keyboard on the grid pads.
//!
//! Layout mapping on the 8×16 grid (row 0 = top of screen):
//! - Row 2: number row (1–0) and backspace
//! - Row 3: top letter row (QWERTY: Q–P)
//! - Row 4: home row (QWERTY: A–L) and Enter
//! - Row 5: bottom letter row (QWERTY: Z–M) and Shift
//! - Row 6: space bar (6 pads wide)

use crate::Pad;
use crate::color::ColorExt as _;
use crate::imode::Frame;
use deluge_bsp::rgb::Color as RGB;

const QWERTY_HOME_ROW: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyboardLayout {
    Qwerty,
    Azerty,
    Qwertz,
    Dvorak,
}

/// A key activated by a pad press on the keyboard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyPress {
    Char(char),
    Backspace,
    Enter,
    Shift,
}

/// The character layout for each keyboard type: `[layout][row][column]`.
const KEYBOARD_CHARS: [[[char; 11]; 5]; 4] = [
    // QWERTY
    [
        ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-'],
        ['Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', '\0'],
        ['A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', '\0', '\''],
        ['Z', 'X', 'C', 'V', 'B', 'N', 'M', ',', '.', '\0', '\0'],
        ['\0', '\0', ' ', ' ', ' ', ' ', ' ', ' ', '\0', '\0', '\0'],
    ],
    // AZERTY
    [
        ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-'],
        ['A', 'Z', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', '\0'],
        ['Q', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 'M', '\''],
        ['W', 'X', 'C', 'V', 'B', 'N', ',', '.', '\0', '\0', '\0'],
        ['\0', '\0', ' ', ' ', ' ', ' ', ' ', ' ', '\0', '\0', '\0'],
    ],
    // QWERTZ
    [
        ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-'],
        ['Q', 'W', 'E', 'R', 'T', 'Z', 'U', 'I', 'O', 'P', 'Ü'],
        ['A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 'Ö', 'Ä'],
        ['Y', 'X', 'C', 'V', 'B', 'N', 'M', ',', '.', '\'', '\0'],
        ['\0', '\0', ' ', ' ', ' ', ' ', ' ', ' ', '\0', '\0', '\0'],
    ],
    // DVORAK
    [
        ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-'],
        ['\'', ',', '.', 'P', 'Y', 'F', 'G', 'C', 'R', 'L', '\0'],
        ['A', 'O', 'E', 'U', 'I', 'D', 'H', 'T', 'N', 'S', '/'],
        [';', 'Q', 'J', 'K', 'X', 'B', 'M', 'W', 'V', 'Z', '\0'],
        ['\0', '\0', ' ', ' ', ' ', ' ', ' ', ' ', '\0', '\0', '\0'],
    ],
];

#[derive(Debug, Clone)]
pub struct TextKeyboardComponent {
    layout: KeyboardLayout,
}

impl TextKeyboardComponent {
    pub fn new(layout: KeyboardLayout) -> Self {
        Self { layout }
    }

    pub fn set_layout(&mut self, layout: KeyboardLayout) {
        self.layout = layout;
    }

    pub fn layout(&self) -> KeyboardLayout {
        self.layout
    }

    /// The character at a grid position, if any.
    pub fn char_at(&self, x: usize, y: usize) -> Option<char> {
        if !(3..14).contains(&x) {
            return None;
        }

        let row = if y >= QWERTY_HOME_ROW.saturating_sub(2) && y <= QWERTY_HOME_ROW + 2 {
            y - (QWERTY_HOME_ROW - 2)
        } else {
            return None;
        };

        let col = x - 3;
        if row >= 5 || col >= 11 {
            return None;
        }

        let ch = KEYBOARD_CHARS[self.layout as usize][row][col];
        if ch == '\0' { None } else { Some(ch) }
    }

    /// Whether a position is the backspace key.
    pub fn is_backspace(&self, x: usize, y: usize) -> bool {
        y == QWERTY_HOME_ROW - 2 && (14..16).contains(&x)
    }

    /// Whether a position is the enter key.
    pub fn is_enter(&self, x: usize, y: usize) -> bool {
        y == QWERTY_HOME_ROW && (14..16).contains(&x)
    }

    /// Whether a position is a shift key.
    pub fn is_shift(&self, x: usize, y: usize) -> bool {
        y == QWERTY_HOME_ROW + 1 && ((1..3).contains(&x) || (13..15).contains(&x))
    }

    /// Whether a position is the space bar.
    pub fn is_space(&self, x: usize, y: usize) -> bool {
        y == QWERTY_HOME_ROW + 2 && (5..11).contains(&x)
    }
}

impl Default for TextKeyboardComponent {
    fn default() -> Self {
        Self::new(KeyboardLayout::Qwerty)
    }
}

impl TextKeyboardComponent {
    /// Paint the keyboard into the current frame. `shift_held` is caller-owned.
    pub fn draw(&self, f: &mut Frame, shift_held: bool) {
        for i in 0..11 {
            if i < 10 {
                f.paint(Pad::new(QWERTY_HOME_ROW - 2, 3 + i), RGB::new(64, 64, 64));
                f.paint(Pad::new(QWERTY_HOME_ROW - 1, 3 + i), RGB::new(10, 10, 10));
            }
            f.paint(Pad::new(QWERTY_HOME_ROW, 3 + i), RGB::new(10, 10, 10));
            if i < 9 {
                f.paint(Pad::new(QWERTY_HOME_ROW + 1, 3 + i), RGB::new(10, 10, 10));
            }
        }

        f.paint(Pad::new(QWERTY_HOME_ROW - 2, 13), RGB::new(10, 10, 10));
        f.paint(Pad::new(QWERTY_HOME_ROW, 3), RGB::new(64, 64, 64));
        f.paint(Pad::new(QWERTY_HOME_ROW, 4), RGB::new(64, 64, 64));
        f.paint(Pad::new(QWERTY_HOME_ROW, 5), RGB::new(64, 64, 64));
        f.paint(Pad::new(QWERTY_HOME_ROW, 6), RGB::new(160, 160, 160));
        f.paint(Pad::new(QWERTY_HOME_ROW, 9), RGB::new(160, 160, 160));
        f.paint(Pad::new(QWERTY_HOME_ROW, 10), RGB::new(64, 64, 64));
        f.paint(Pad::new(QWERTY_HOME_ROW, 11), RGB::new(64, 64, 64));
        f.paint(Pad::new(QWERTY_HOME_ROW, 12), RGB::new(64, 64, 64));
        for i in 0..6 {
            f.paint(
                Pad::new(QWERTY_HOME_ROW + 2, 5 + i),
                RGB::new(160, 160, 160),
            );
        }

        for x in 14..16 {
            f.paint(Pad::new(QWERTY_HOME_ROW - 2, x), RGB::new(255, 0, 0));
        }
        for x in 14..16 {
            f.paint(Pad::new(QWERTY_HOME_ROW, x), RGB::new(0, 255, 0));
        }
        for x in 1..3 {
            f.paint(Pad::new(QWERTY_HOME_ROW + 1, x), RGB::new(0, 0, 255));
        }
        for x in 13..15 {
            f.paint(Pad::new(QWERTY_HOME_ROW + 1, x), RGB::new(0, 0, 255));
        }

        if shift_held {
            for x in 1..3 {
                f.paint(Pad::new(QWERTY_HOME_ROW + 1, x), RGB::new(100, 100, 255));
            }
            for x in 13..15 {
                f.paint(Pad::new(QWERTY_HOME_ROW + 1, x), RGB::new(100, 100, 255));
            }
        }
    }

    /// Paint the keyboard and read input in one pass, returning the first key
    /// pressed this frame (if any).
    pub fn show(&self, f: &mut Frame, shift_held: bool) -> Option<KeyPress> {
        self.draw(f, shift_held);
        for ev in f.input().events.iter() {
            if !ev.pressed {
                continue;
            }
            let (x, y) = (ev.pad.col, ev.pad.row);
            if let Some(ch) = self.char_at(x, y) {
                return Some(KeyPress::Char(ch));
            }
            if self.is_backspace(x, y) {
                return Some(KeyPress::Backspace);
            }
            if self.is_enter(x, y) {
                return Some(KeyPress::Enter);
            }
            if self.is_shift(x, y) {
                return Some(KeyPress::Shift);
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_qwerty_layout() {
        let keyboard = TextKeyboardComponent::new(KeyboardLayout::Qwerty);
        assert_eq!(keyboard.char_at(3, QWERTY_HOME_ROW - 2), Some('1'));
        assert_eq!(keyboard.char_at(12, QWERTY_HOME_ROW - 2), Some('0'));
        assert_eq!(keyboard.char_at(3, QWERTY_HOME_ROW - 1), Some('Q'));
        assert_eq!(keyboard.char_at(3, QWERTY_HOME_ROW), Some('A'));
        assert_eq!(keyboard.char_at(6, QWERTY_HOME_ROW), Some('F'));
    }

    #[test]
    fn test_azerty_layout() {
        let keyboard = TextKeyboardComponent::new(KeyboardLayout::Azerty);
        assert_eq!(keyboard.char_at(3, QWERTY_HOME_ROW - 1), Some('A'));
        assert_eq!(keyboard.char_at(4, QWERTY_HOME_ROW - 1), Some('Z'));
        assert_eq!(keyboard.char_at(3, QWERTY_HOME_ROW), Some('Q'));
    }

    #[test]
    fn test_special_keys() {
        let keyboard = TextKeyboardComponent::new(KeyboardLayout::Qwerty);
        assert!(keyboard.is_backspace(14, QWERTY_HOME_ROW - 2));
        assert!(keyboard.is_enter(14, QWERTY_HOME_ROW));
        assert!(keyboard.is_shift(1, QWERTY_HOME_ROW + 1));
        assert!(keyboard.is_space(5, QWERTY_HOME_ROW + 2));
    }

    #[test]
    fn draws_and_reports_keypress() {
        use crate::imode::{GridUi, PadInput};
        let kb = TextKeyboardComponent::new(KeyboardLayout::Qwerty);

        let mut ui = GridUi::new();
        ui.run(0, PadInput::new(), |f| kb.draw(f, false));
        // Enter key (green) at the home row, col 14.
        assert_eq!(ui.grid().get_pad(QWERTY_HOME_ROW, 14), RGB::new(0, 255, 0));

        // `show` reads input: pressing the home-row col-3 pad yields 'A'.
        let mut input = PadInput::new();
        input.press(Pad::new(QWERTY_HOME_ROW, 3));
        let pressed = ui.run(16, input, |f| kb.show(f, false)).painted().unwrap();
        assert_eq!(pressed, Some(KeyPress::Char('A')));
    }
}
