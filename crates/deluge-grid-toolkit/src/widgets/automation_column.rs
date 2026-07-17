//! Vertical bar-graph columns for automation / velocity values.

use crate::Pad;
use crate::color::ColorExt as _;
#[allow(unused_imports)] // needed on targets whose `core` lacks inherent f32 math
use crate::float_ext::F32Ext as _;
use crate::imode::Frame;
use crate::pad::GRID_ROWS;
use deluge_bsp::rgb::Color as RGB;

// Normalized thresholds for the 8-row bar graph. Each element is the minimum
// normalized value (0.0–1.0) that lights up that row.
const UNIPOLAR_THRESHOLDS: [f32; 8] = [
    0.0078, 0.1406, 0.2734, 0.4063, 0.5391, 0.6719, 0.8047, 0.9375,
];
const VELOCITY_THRESHOLDS: [f32; 8] = [
    0.0000, 0.1328, 0.2578, 0.3828, 0.5078, 0.6328, 0.7578, 0.8828,
];
const BIPOLAR_UPPER_THRESHOLDS: [f32; 4] = [0.1328, 0.3984, 0.6641, 0.9297];
const BIPOLAR_LOWER_THRESHOLDS: [f32; 4] = [0.9297, 0.6641, 0.3984, 0.1328];

/// The normalized value (0.0–1.0) for a unipolar pad press at `row`.
pub fn unipolar_pad_value(row: u8) -> f32 {
    match row {
        0 => 0.0,
        1 => 0.1406,
        2 => 0.2891,
        3 => 0.4297,
        4 => 0.5703,
        5 => 0.7109,
        6 => 0.8594,
        7 => 1.0,
        _ => 0.0,
    }
}

/// The normalized value (-1.0–1.0) for a bipolar pad press at `row`.
pub fn bipolar_pad_value(row: u8) -> f32 {
    match row {
        0 => -1.0,
        1 => -0.7031,
        2 => -0.4688,
        3 => -0.2344,
        4 => 0.2344,
        5 => 0.4688,
        6 => 0.7031,
        7 => 1.0,
        _ => 0.0,
    }
}

/// Unipolar automation column: a vertical bar for a 0.0–1.0 parameter.
pub struct UnipolarAutomationColumn {
    value: f32,
    is_automated: bool,
    color: AutomationColor,
}

impl UnipolarAutomationColumn {
    pub fn new(value: f32, is_automated: bool) -> Self {
        Self {
            value: value.clamp(0.0, 1.0),
            is_automated,
            color: AutomationColor::default(),
        }
    }

    pub fn set_value(&mut self, value: f32) {
        self.value = value.clamp(0.0, 1.0);
    }

    pub fn set_automated(&mut self, is_automated: bool) {
        self.is_automated = is_automated;
    }

    pub fn set_color(&mut self, color: AutomationColor) {
        self.color = color;
    }
}

impl UnipolarAutomationColumn {
    /// Paint the column into the current frame (column 0, rows 0..8).
    pub fn draw(&self, f: &mut Frame) {
        for (row, &threshold) in UNIPOLAR_THRESHOLDS.iter().enumerate() {
            if self.value >= threshold {
                let color = if self.is_automated {
                    self.color.get_main_color(row)
                } else {
                    self.color.get_tail_color(row)
                };
                f.paint(Pad::new(row, 0), color);
            }
        }
    }
}

/// Bipolar automation column: a vertical bar for a -1.0–1.0 parameter (center =
/// no pads lit; top 4 = positive, bottom 4 = negative).
pub struct BipolarAutomationColumn {
    value: f32,
    is_automated: bool,
    color: AutomationColor,
}

impl BipolarAutomationColumn {
    pub fn new(value: f32, is_automated: bool) -> Self {
        Self {
            value: value.clamp(-1.0, 1.0),
            is_automated,
            color: AutomationColor::default(),
        }
    }

    pub fn set_value(&mut self, value: f32) {
        self.value = value.clamp(-1.0, 1.0);
    }

    pub fn set_automated(&mut self, is_automated: bool) {
        self.is_automated = is_automated;
    }

    pub fn set_color(&mut self, color: AutomationColor) {
        self.color = color;
    }
}

impl BipolarAutomationColumn {
    /// Paint the column into the current frame (column 0, rows 0..8).
    pub fn draw(&self, f: &mut Frame) {
        if self.value.abs() < 0.001 {
            return;
        }

        let is_positive = self.value > 0.0;
        let abs_val = self.value.abs();

        for row in 0..GRID_ROWS {
            if is_positive && row < 4 {
                continue;
            }
            if !is_positive && row > 3 {
                continue;
            }

            let should_render = if is_positive {
                abs_val >= BIPOLAR_UPPER_THRESHOLDS[row - 4]
            } else {
                abs_val >= BIPOLAR_LOWER_THRESHOLDS[row]
            };

            if should_render {
                let color = if self.is_automated {
                    if is_positive {
                        self.color.get_bipolar_up_color(7 - row)
                    } else {
                        self.color.get_bipolar_down_color(row)
                    }
                } else if is_positive {
                    self.color.get_bipolar_up_tail_color(7 - row)
                } else {
                    self.color.get_bipolar_down_tail_color(row)
                };
                f.paint(Pad::new(row, 0), color);
            }
        }
    }
}

/// Colour scheme for automation columns.
#[derive(Debug, Clone, Copy)]
pub struct AutomationColor {
    main_colors: [RGB; 8],
    tail_colors: [RGB; 8],
    bipolar_up_colors: [RGB; 4],
    bipolar_up_tail_colors: [RGB; 4],
    bipolar_down_colors: [RGB; 4],
    bipolar_down_tail_colors: [RGB; 4],
}

impl Default for AutomationColor {
    fn default() -> Self {
        Self::green_red()
    }
}

impl AutomationColor {
    /// Green-to-red gradient (default, for levels/volumes).
    pub fn green_red() -> Self {
        Self {
            main_colors: [
                RGB::new(0, 255, 0),
                RGB::new(64, 255, 0),
                RGB::new(128, 255, 0),
                RGB::new(192, 255, 0),
                RGB::new(255, 192, 0),
                RGB::new(255, 128, 0),
                RGB::new(255, 64, 0),
                RGB::new(255, 0, 0),
            ],
            tail_colors: [
                RGB::new(0, 128, 0),
                RGB::new(32, 128, 0),
                RGB::new(64, 128, 0),
                RGB::new(96, 128, 0),
                RGB::new(128, 96, 0),
                RGB::new(128, 64, 0),
                RGB::new(128, 32, 0),
                RGB::new(128, 0, 0),
            ],
            bipolar_up_colors: [
                RGB::new(255, 64, 0),
                RGB::new(255, 128, 0),
                RGB::new(255, 192, 0),
                RGB::new(255, 255, 0),
            ],
            bipolar_up_tail_colors: [
                RGB::new(128, 32, 0),
                RGB::new(128, 64, 0),
                RGB::new(128, 96, 0),
                RGB::new(128, 128, 0),
            ],
            bipolar_down_colors: [
                RGB::new(0, 255, 0),
                RGB::new(64, 255, 0),
                RGB::new(128, 255, 0),
                RGB::new(192, 255, 0),
            ],
            bipolar_down_tail_colors: [
                RGB::new(0, 128, 0),
                RGB::new(32, 128, 0),
                RGB::new(64, 128, 0),
                RGB::new(96, 128, 0),
            ],
        }
    }

    /// Blue-to-red gradient (for note velocity).
    pub fn blue_red() -> Self {
        Self {
            main_colors: [
                RGB::new(0, 0, 255),
                RGB::new(0, 64, 255),
                RGB::new(0, 128, 255),
                RGB::new(0, 192, 255),
                RGB::new(255, 192, 0),
                RGB::new(255, 128, 0),
                RGB::new(255, 64, 0),
                RGB::new(255, 0, 0),
            ],
            tail_colors: [
                RGB::new(0, 0, 128),
                RGB::new(0, 32, 128),
                RGB::new(0, 64, 128),
                RGB::new(0, 96, 128),
                RGB::new(128, 96, 0),
                RGB::new(128, 64, 0),
                RGB::new(128, 32, 0),
                RGB::new(128, 0, 0),
            ],
            bipolar_up_colors: [
                RGB::new(255, 64, 0),
                RGB::new(255, 128, 0),
                RGB::new(255, 192, 0),
                RGB::new(255, 255, 0),
            ],
            bipolar_up_tail_colors: [
                RGB::new(128, 32, 0),
                RGB::new(128, 64, 0),
                RGB::new(128, 96, 0),
                RGB::new(128, 128, 0),
            ],
            bipolar_down_colors: [
                RGB::new(0, 0, 255),
                RGB::new(0, 64, 255),
                RGB::new(0, 128, 255),
                RGB::new(0, 192, 255),
            ],
            bipolar_down_tail_colors: [
                RGB::new(0, 0, 128),
                RGB::new(0, 32, 128),
                RGB::new(0, 64, 128),
                RGB::new(0, 96, 128),
            ],
        }
    }

    fn get_main_color(&self, y: usize) -> RGB {
        self.main_colors[y]
    }
    fn get_tail_color(&self, y: usize) -> RGB {
        self.tail_colors[y]
    }
    fn get_bipolar_up_color(&self, y: usize) -> RGB {
        self.bipolar_up_colors[y]
    }
    fn get_bipolar_up_tail_color(&self, y: usize) -> RGB {
        self.bipolar_up_tail_colors[y]
    }
    fn get_bipolar_down_color(&self, y: usize) -> RGB {
        self.bipolar_down_colors[y]
    }
    fn get_bipolar_down_tail_color(&self, y: usize) -> RGB {
        self.bipolar_down_tail_colors[y]
    }
}

/// Note-velocity automation column (0.0–1.0, blue-to-red gradient).
pub struct NoteVelocityColumn {
    value: f32,
    is_automated: bool,
}

impl NoteVelocityColumn {
    pub fn new(value: f32, is_automated: bool) -> Self {
        Self {
            value: value.clamp(0.0, 1.0),
            is_automated,
        }
    }

    pub fn set_value(&mut self, value: f32) {
        self.value = value.clamp(0.0, 1.0);
    }

    pub fn set_automated(&mut self, is_automated: bool) {
        self.is_automated = is_automated;
    }
}

impl NoteVelocityColumn {
    /// Paint the column into the current frame (column 0, rows 0..8).
    pub fn draw(&self, f: &mut Frame) {
        if self.value < 0.001 {
            return;
        }

        let color = AutomationColor::blue_red();
        for (y, &threshold) in VELOCITY_THRESHOLDS.iter().enumerate() {
            if self.value >= threshold {
                let pad_color = if self.is_automated {
                    color.get_main_color(y)
                } else {
                    color.get_tail_color(y)
                };
                f.paint(Pad::new(y, 0), pad_color);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Grid;
    use crate::imode::{Frame, GridUi, PadInput};

    fn render(draw: impl FnOnce(&mut Frame)) -> Grid {
        let mut ui = GridUi::new();
        ui.run(0, PadInput::new(), draw);
        ui.grid().clone()
    }

    #[test]
    fn test_unipolar_extremes() {
        let zero = render(|f| UnipolarAutomationColumn::new(0.0, true).draw(f));
        let max = render(|f| UnipolarAutomationColumn::new(1.0, true).draw(f));
        for y in 0..8 {
            assert_eq!(zero.get_pad(y, 0), RGB::new(0, 0, 0));
            assert_ne!(max.get_pad(y, 0), RGB::new(0, 0, 0));
        }
    }

    #[test]
    fn test_bipolar_split() {
        let pos = render(|f| BipolarAutomationColumn::new(1.0, true).draw(f));
        for y in 0..4 {
            assert_eq!(pos.get_pad(y, 0), RGB::new(0, 0, 0));
        }
        for y in 4..8 {
            assert_ne!(pos.get_pad(y, 0), RGB::new(0, 0, 0));
        }
    }

    #[test]
    fn test_pad_values_monotonic() {
        for i in 0..7 {
            assert!(unipolar_pad_value(i) < unipolar_pad_value(i + 1));
            assert!(bipolar_pad_value(i) < bipolar_pad_value(i + 1));
        }
        assert!(bipolar_pad_value(3) < 0.0);
        assert!(bipolar_pad_value(4) > 0.0);
    }
}
