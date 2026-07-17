//! Zoom transition animation.
//!
//! Cross-fade between two grid states at different zoom levels.
//! Ported from Deluge `pad_leds.cpp` (`renderZoomWithProgress`).

use super::Animation;
#[allow(unused_imports)] // needed on targets whose `core` lacks inherent f32 math
use crate::float_ext::F32Ext as _;
use crate::grid::GridRgb;
use crate::{Color, Grid};

/// Zoom animation with dual-buffer cross-fade.
pub struct ZoomAnimation {
    inner_grid: GridRgb,
    outer_grid: GridRgb,
    zoom_in: bool,
    magnitude: i8,
    pin_squares: [f32; 8],
    duration_ms: f32,
    elapsed_ms: f32,
}

impl ZoomAnimation {
    /// Create a new zoom animation.
    pub fn new(
        from: Grid,
        to: Grid,
        zoom_in: bool,
        magnitude: i8,
        pin_x: f32,
        duration_ms: u32,
    ) -> Self {
        let (inner_grid, outer_grid) = if zoom_in {
            (to.to_rgb(), from.to_rgb())
        } else {
            (from.to_rgb(), to.to_rgb())
        };

        Self {
            inner_grid,
            outer_grid,
            zoom_in,
            magnitude,
            pin_squares: [pin_x; 8],
            duration_ms: duration_ms as f32,
            elapsed_ms: 0.0,
        }
    }

    fn render_frame(&self, progress: f32) -> Grid {
        let mut output_rgb = [[Color::BLACK; 18]; 8];

        let fade_progress = if self.zoom_in {
            progress
        } else {
            1.0 - progress
        };

        let zoom_factor = 1.0 + (self.magnitude as f32 * progress);
        let inner_scale = if self.zoom_in {
            zoom_factor
        } else {
            1.0 / zoom_factor
        };
        let outer_scale = inner_scale * 2.0_f32.powi(self.magnitude as i32);

        for (y, row_out) in output_rgb.iter_mut().enumerate() {
            let pin_x = self.pin_squares[y];

            let mut inner_row = [Color::BLACK; 18];
            let mut outer_row = [Color::BLACK; 18];
            let mut weights = [0.0f32; 18];

            for x in 0..18 {
                let out_x = x as f32;
                let offset_from_pin = out_x - pin_x;

                let inner_x = pin_x + (offset_from_pin / inner_scale);
                let outer_x = pin_x + (offset_from_pin / outer_scale);

                inner_row[x] = self.sample_grid(&self.inner_grid, y, inner_x);
                outer_row[x] = self.sample_grid(&self.outer_grid, y, outer_x);

                let inner_coverage = self.calculate_coverage(out_x, pin_x, inner_scale);
                weights[x] = (inner_coverage * fade_progress).clamp(0.0, 1.0);
            }

            *row_out = blend_rows_variable(&inner_row, &outer_row, &weights);
        }

        Grid::from_rgb(output_rgb)
    }

    fn sample_grid(&self, grid: &GridRgb, y: usize, x: f32) -> Color {
        if !(0.0..17.0).contains(&x) || y >= 8 {
            return Color::BLACK;
        }

        let x0 = x.floor() as usize;
        let x1 = (x0 + 1).min(17);
        let frac = x - x0 as f32;

        if x0 >= 18 {
            return Color::BLACK;
        }

        let c0 = grid[y][x0];
        let c1 = grid[y][x1];

        Color::rgb(
            (c0.r as f32 * (1.0 - frac) + c1.r as f32 * frac) as u8,
            (c0.g as f32 * (1.0 - frac) + c1.g as f32 * frac) as u8,
            (c0.b as f32 * (1.0 - frac) + c1.b as f32 * frac) as u8,
        )
    }

    fn calculate_coverage(&self, out_x: f32, pin_x: f32, inner_scale: f32) -> f32 {
        let out_left = out_x;
        let out_right = out_x + 1.0;
        let inner_left = pin_x - (pin_x / inner_scale);
        let inner_right = pin_x + ((17.0 - pin_x) / inner_scale);
        let overlap = (out_right.min(inner_right) - out_left.max(inner_left)).max(0.0);
        overlap.clamp(0.0, 1.0)
    }
}

/// Blend two rows with a per-pixel weight (`result = inner·w + outer·(1-w)`).
fn blend_rows_variable(
    inner: &[Color; 18],
    outer: &[Color; 18],
    weights: &[f32; 18],
) -> [Color; 18] {
    let mut result = [Color::BLACK; 18];
    for i in 0..18 {
        let w = weights[i];
        result[i] = Color::rgb(
            (inner[i].r as f32 * w + outer[i].r as f32 * (1.0 - w)) as u8,
            (inner[i].g as f32 * w + outer[i].g as f32 * (1.0 - w)) as u8,
            (inner[i].b as f32 * w + outer[i].b as f32 * (1.0 - w)) as u8,
        );
    }
    result
}

impl Animation for ZoomAnimation {
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
    use crate::color::ColorExt as _; // Color::rgb constructor in the test

    #[test]
    fn test_zoom_animation() {
        let mut from = Grid::new();
        let mut to = Grid::new();
        for row in 0..8 {
            for col in 0..18 {
                from.set_pad(row, col, Color::rgb(100, 0, 0));
                to.set_pad(row, col, Color::rgb(0, 0, 100));
            }
        }

        let mut anim = ZoomAnimation::new(from, to, true, 1, 9.0, 200);
        assert!(anim.tick(100.0).is_some());
        anim.tick(100.0);
        assert!(anim.is_complete());
    }
}
