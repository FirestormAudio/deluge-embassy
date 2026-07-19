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
