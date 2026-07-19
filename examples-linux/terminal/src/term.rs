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
}

impl Screen {
    pub fn new() -> Self {
        Self { cells: [[b' '; COLS]; ROWS], cx: 0, cy: 0 }
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
    }

    fn erase_line(&mut self, mode: usize) {
        let row = &mut self.cells[self.cy];
        match mode {
            0 => row[self.cx..].fill(b' '),
            1 => row[..=self.cx.min(COLS - 1)].fill(b' '),
            _ => row.fill(b' '),
        }
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
                // ED mode 2: clear the whole screen but leave the cursor where
                // it is (VT100-correct; `clear`/`ash` send an explicit ESC[H).
                self.cells = [[b' '; COLS]; ROWS];
            }
        }
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

    #[test]
    fn erase_display_all_clears_but_keeps_cursor() {
        let mut t = Term::new();
        t.feed(b"\x1b[2;3Hhi"); // cursor to (row 2, col 3), print "hi" -> cursor (2,4)
        t.feed(b"\x1b[2J"); // ED mode 2: clear whole screen
        assert!(t.screen.cells.iter().all(|r| r.iter().all(|&b| b == b' ')));
        assert_eq!((t.screen.cx, t.screen.cy), (4, 1)); // cursor unchanged
    }
}
