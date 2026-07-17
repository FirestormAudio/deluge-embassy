//! Expand/collapse vertical animation.
//!
//! Animates rows expanding or collapsing vertically with fractional
//! interpolation. Ported from Deluge `pad_leds.cpp`
//! (`renderInstrumentClipCollapseAnimation`).

use super::Animation;
use crate::color::lerp_slice;
use crate::grid::GridRgb;
use crate::{Color, Grid};

/// Expand or collapse direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Expand,
    Collapse,
}

/// Vertical expand/collapse animation.
pub struct ExpandCollapseAnimation {
    from_grid: GridRgb,
    to_grid: GridRgb,
    direction: Direction,
    from_row: usize,
    to_row: usize,
    duration_ms: f32,
    elapsed_ms: f32,
}

impl ExpandCollapseAnimation {
    /// Create a new expand/collapse animation.
    ///
    /// # Panics
    /// If the `from_row..=to_row` range is invalid.
    pub fn new(
        from: Grid,
        to: Grid,
        direction: Direction,
        from_row: usize,
        to_row: usize,
        duration_ms: u32,
    ) -> Self {
        assert!(
            from_row < 8 && to_row < 8 && from_row <= to_row,
            "Invalid row range"
        );

        Self {
            from_grid: from.to_rgb(),
            to_grid: to.to_rgb(),
            direction,
            from_row,
            to_row,
            duration_ms: duration_ms as f32,
            elapsed_ms: 0.0,
        }
    }

    fn render_frame(&self, progress: f32) -> Grid {
        let mut output_rgb = [[Color::BLACK; 18]; 8];

        let anim_progress = match self.direction {
            Direction::Expand => progress,
            Direction::Collapse => 1.0 - progress,
        };

        const FIXED_SCALE: i32 = 65536;

        let mut intensity1 = [0u32; 8];
        let mut intensity2 = [0u32; 8];
        let mut target_row1 = [0usize; 8];
        let mut target_row2 = [0usize; 8];

        for src_row in self.from_row..=self.to_row {
            let from_pos = src_row as f32;
            let center = (self.from_row + self.to_row) as f32 / 2.0;
            let offset = src_row as f32 - center;
            let to_pos = center + offset * (1.0 + anim_progress);

            let row_pos_fixed = (from_pos * FIXED_SCALE as f32
                + (to_pos - from_pos) * FIXED_SCALE as f32 * anim_progress)
                as i32;

            let row_integer = (row_pos_fixed >> 16).clamp(0, 7) as usize;
            let row_fraction = (row_pos_fixed & 0xFFFF) as u32;

            intensity1[src_row] = (FIXED_SCALE as u32) - row_fraction;
            intensity2[src_row] = row_fraction;
            target_row1[src_row] = row_integer;
            target_row2[src_row] = (row_integer + 1).min(7);
        }

        let mut interpolated_rows = [[Color::BLACK; 18]; 8];
        for (src_row, interp) in (self.from_row..=self.to_row)
            .zip(interpolated_rows[self.from_row..=self.to_row].iter_mut())
        {
            *interp = self.interpolate_row(src_row, anim_progress);
        }

        for (out_row, out_row_pixels) in output_rgb.iter_mut().enumerate() {
            let mut accumulated_row = [Color::BLACK; 18];
            let mut total_intensity = 0u32;

            for src_row in self.from_row..=self.to_row {
                if target_row1[src_row] == out_row {
                    let intensity = intensity1[src_row];
                    accumulate_row_scaled(
                        &interpolated_rows[src_row],
                        intensity,
                        &mut accumulated_row,
                    );
                    total_intensity += intensity;
                }
                if target_row2[src_row] == out_row {
                    let intensity = intensity2[src_row];
                    accumulate_row_scaled(
                        &interpolated_rows[src_row],
                        intensity,
                        &mut accumulated_row,
                    );
                    total_intensity += intensity;
                }
            }

            if total_intensity > 0 {
                normalize_row(&accumulated_row, total_intensity, out_row_pixels);
            } else {
                *out_row_pixels = [Color::rgb(30, 30, 30); 18];
            }
        }

        for (row, out_row) in output_rgb.iter_mut().enumerate() {
            if row < self.from_row || row > self.to_row {
                let interpolated = self.interpolate_row(row, progress);
                out_row.copy_from_slice(&interpolated);
            }
        }

        Grid::from_rgb(output_rgb)
    }

    fn interpolate_row(&self, row: usize, progress: f32) -> [Color; 18] {
        let mut result = [Color::BLACK; 18];
        lerp_slice(
            &self.from_grid[row],
            &self.to_grid[row],
            progress,
            &mut result,
        );
        result
    }
}

/// Accumulate a source row into `output` with intensity scaling (16.16 fixed).
fn accumulate_row_scaled(source: &[Color; 18], intensity: u32, output: &mut [Color; 18]) {
    for i in 0..18 {
        let existing = output[i];
        let src = source[i];
        output[i] = Color::rgb(
            ((existing.r as u32 * 65536 + src.r as u32 * intensity) / 65536).min(255) as u8,
            ((existing.g as u32 * 65536 + src.g as u32 * intensity) / 65536).min(255) as u8,
            ((existing.b as u32 * 65536 + src.b as u32 * intensity) / 65536).min(255) as u8,
        );
    }
}

/// Normalize an accumulated row by total intensity.
fn normalize_row(accumulated: &[Color; 18], total_intensity: u32, output: &mut [Color; 18]) {
    for i in 0..18 {
        let acc = accumulated[i];
        output[i] = Color::rgb(
            (acc.r as u32 / total_intensity).min(255) as u8,
            (acc.g as u32 / total_intensity).min(255) as u8,
            (acc.b as u32 / total_intensity).min(255) as u8,
        );
    }
}

impl Animation for ExpandCollapseAnimation {
    fn tick(&mut self, delta_ms: f32) -> Option<Grid> {
        if self.is_complete() {
            return None;
        }
        self.elapsed_ms += delta_ms;
        let progress = (self.elapsed_ms / self.duration_ms).clamp(0.0, 1.0);
        Some(self.render_frame(progress))
    }

    fn duration_ms(&self) -> f32 {
        self.duration_ms
    }

    fn is_complete(&self) -> bool {
        self.elapsed_ms >= self.duration_ms
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_expand_animation() {
        let mut from = Grid::new();
        let mut to = Grid::new();
        for row in 2..6 {
            for col in 0..18 {
                from.set_pad(row, col, Color::rgb(100, 0, 0));
                to.set_pad(row, col, Color::rgb(0, 0, 100));
            }
        }

        let mut anim = ExpandCollapseAnimation::new(from, to, Direction::Expand, 2, 5, 200);
        assert!(anim.tick(100.0).is_some());
        anim.tick(100.0);
        assert!(anim.is_complete());
    }

    #[test]
    fn test_collapse_animation() {
        let mut from = Grid::new();
        let mut to = Grid::new();
        for row in 0..8 {
            for col in 0..18 {
                from.set_pad(row, col, Color::rgb(100, 0, 0));
                to.set_pad(row, col, Color::rgb(0, 0, 100));
            }
        }

        let mut anim = ExpandCollapseAnimation::new(from, to, Direction::Collapse, 1, 6, 200);
        assert!(anim.tick(50.0).is_some());
        assert!(!anim.is_complete());
    }
}
