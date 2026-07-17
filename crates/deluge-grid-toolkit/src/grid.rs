//! Grid buffer for rendering pad colours.

use crate::pad::{GRID_COLS, GRID_ROWS, Pad};
use deluge_bsp::rgb::{Color, PadLeds};

/// Buffer containing colour data for all pads in the grid.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Grid {
    pub buffer: [[Option<Color>; GRID_COLS]; GRID_ROWS],
}

/// A fully-opaque grid (no transparent cells).
pub type GridRgb = [[Color; GRID_COLS]; GRID_ROWS];

impl Grid {
    /// Create a new grid buffer with default (black) colours.
    pub fn new() -> Self {
        Self {
            buffer: [[Some(Color::BLACK); GRID_COLS]; GRID_ROWS],
        }
    }

    /// Set the colour of a pad.
    pub fn set_pad(&mut self, row: usize, col: usize, color: Color) {
        if let Some(row) = self.buffer.get_mut(row) {
            if let Some(pixel) = row.get_mut(col) {
                *pixel = Some(color);
            }
        }
    }

    /// Get the colour of a pad.
    pub fn get_pad(&self, row: usize, col: usize) -> Color {
        self.buffer[row][col].unwrap_or(Color::BLACK)
    }

    /// Get the grid as opaque RGB values (unwrapping `Option`s).
    pub fn to_rgb(&self) -> GridRgb {
        self.buffer
            .map(|row| row.map(|opt| opt.unwrap_or(Color::BLACK)))
    }

    /// Create a grid from opaque RGB values.
    pub fn from_rgb(rgb: GridRgb) -> Self {
        Self {
            buffer: rgb.map(|row| row.map(Some)),
        }
    }

    /// Clear all pads to transparent.
    pub fn clear(&mut self) {
        self.buffer = [[None; GRID_COLS]; GRID_ROWS];
    }

    /// Set all pads to black.
    pub fn blank(&mut self) {
        self.buffer = [[Some(Color::BLACK); GRID_COLS]; GRID_ROWS];
    }

    /// Fill a rectangular region with a colour.
    pub fn fill_rect(&mut self, start: Pad, end: Pad, color: Color) {
        for row in start.row..=end.row {
            for col in start.col..=end.col {
                self.set_pad(row, col, color);
            }
        }
    }

    /// Draw a horizontal line.
    pub fn draw_horizontal_line(
        &mut self,
        row: usize,
        start_col: usize,
        end_col: usize,
        color: Color,
    ) {
        for col in start_col..=end_col {
            self.set_pad(row, col, color);
        }
    }

    /// Draw a vertical line.
    pub fn draw_vertical_line(
        &mut self,
        col: usize,
        start_row: usize,
        end_row: usize,
        color: Color,
    ) {
        for row in start_row..=end_row {
            self.set_pad(row, col, color);
        }
    }

    /// Return a copy of this grid with all pad colours dimmed by `level`
    /// (each channel right-shifted, i.e. ÷2^`level`).
    pub fn dimmed(&self, level: u8) -> Self {
        let buffer = self
            .buffer
            .map(|row| row.map(|opt| opt.map(|c| c.dim(level))));
        Self { buffer }
    }

    /// Overlay `other` onto this grid (non-transparent cells win).
    pub fn compose(&mut self, other: &Grid) {
        for row in 0..GRID_ROWS {
            for col in 0..GRID_COLS {
                if let Some(color) = other.buffer[row][col] {
                    self.buffer[row][col] = Some(color);
                }
            }
        }
    }

    /// Overlay `other` onto this grid, translated by `(row_offset, col_offset)`.
    pub fn compose_with_translation(&mut self, other: &Grid, row_offset: isize, col_offset: isize) {
        for row in 0..GRID_ROWS {
            for col in 0..GRID_COLS {
                let target_row = row as isize + row_offset;
                let target_col = col as isize + col_offset;
                if target_row >= 0
                    && (target_row as usize) < GRID_ROWS
                    && target_col >= 0
                    && (target_col as usize) < GRID_COLS
                {
                    if let Some(color) = other.buffer[row][col] {
                        self.buffer[target_row as usize][target_col as usize] = Some(color);
                    }
                }
            }
        }
    }

    /// Write this grid into a hardware [`PadLeds`] frame buffer.
    ///
    /// The grid is row-major (`buffer[row][col]`); `PadLeds` is column-major
    /// (`set(x = col, y = row, …)`). Transparent cells are written as black.
    /// Call [`PadLeds::flush`] afterwards to push to the panel.
    pub fn blit(&self, leds: &mut PadLeds) {
        for row in 0..GRID_ROWS {
            for col in 0..GRID_COLS {
                let color = self.buffer[row][col].unwrap_or(Color::BLACK);
                leds.set(col, row, color.to_rgb());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::color::ColorExt as _;

    #[test]
    fn test_grid_buffer() {
        let mut buffer = Grid::new();
        buffer.set_pad(0, 0, Color::red());
        assert_eq!(buffer.get_pad(0, 0), Color::red());
        assert_eq!(buffer.get_pad(1, 1), Color::BLACK);
    }

    #[test]
    fn test_grid_buffer_fill() {
        let mut buffer = Grid::new();
        buffer.fill_rect(Pad::new(0, 0), Pad::new(2, 2), Color::blue());
        assert_eq!(buffer.get_pad(0, 0), Color::blue());
        assert_eq!(buffer.get_pad(2, 2), Color::blue());
        assert_eq!(buffer.get_pad(3, 3), Color::BLACK);
    }

    #[test]
    fn blit_maps_row_col_to_x_y() {
        let mut grid = Grid::new();
        grid.set_pad(7, 17, Color::rgb(1, 2, 3));
        let mut leds = PadLeds::new();
        grid.blit(&mut leds);
        // PadLeds is column-major: grid (row=7, col=17) -> leds (x=17, y=7).
        assert_eq!(leds.grid()[17][7], [1, 2, 3]);
    }
}
