//! Canvas renderer for dynamic simulator elements (pad grid, OLED, buttons, encoders, LEDs).
//!
//! This module contains `DynamicElementsRenderer` which implements `canvas::Program`
//! and owns all drawing and hit-testing logic. Layout constants are co-located here
//! since both drawing and hit-testing share the same coordinate tables.

use crate::app::SimulatorMessage;
use crate::display::SimulatorDisplay;
use crate::hardware::{HardwareButton, HardwareEncoder, HardwareLED};
use crate::hardware_state::DelugeHardware;
use crate::pad_grid::{PadGrid, ToIcedColor};
use crate::rgb::RGB;
use iced::{
    Color, Point, Rectangle as IcedRect, Renderer, Size, event, mouse,
    widget::canvas::{self, Frame},
};
use std::collections::HashSet;

const OLED_WIDTH: usize = 128;
const OLED_HEIGHT: usize = 43;
const PAD_COLS: usize = 18; // 16 main grid + 2 audition/mute columns
const PAD_ROWS: usize = 8;

// ================================
// SVG-derived coordinate tables
// ================================
// Single source of truth for all element positions — used by both
// drawing and hit-testing code.  Coordinates are in SVG viewBox space
// (2178 × 1482) and are scaled at render time.

/// Encoder positions: (id, centre-x, centre-y, radius)
const ENCODER_POSITIONS: [(HardwareEncoder, f32, f32, f32); 7] = [
    (HardwareEncoder::HorizontalEncoder, 370.71, 210.28, 61.0),
    (HardwareEncoder::VerticalEncoder, 166.65, 454.27, 61.0),
    (HardwareEncoder::UpperGold, 776.04, 210.26, 61.0),
    (HardwareEncoder::LowerGold, 573.04, 454.25, 61.0),
    (HardwareEncoder::Select, 1036.32, 330.58, 56.0),
    (HardwareEncoder::Tempo, 1701.66, 210.28, 61.0),
    (HardwareEncoder::Volume, 1936.03, 210.26, 61.0),
];

/// Button positions: (id, centre-x, centre-y, radius)
const BUTTON_POSITIONS: [(HardwareButton, f32, f32, f32); 28] = [
    // Far right
    (HardwareButton::Play, 1934.60, 332.13, 22.5),
    (HardwareButton::Record, 1934.60, 413.93, 22.5),
    (HardwareButton::Shift, 1934.60, 495.06, 22.5),
    // Middle-right column
    (HardwareButton::Fill, 1701.66, 413.93, 22.5),
    (HardwareButton::Select, 1701.66, 495.06, 22.5),
    (HardwareButton::TapTempo, 1701.66, 332.13, 22.5),
    // Screen-right column
    (HardwareButton::Back, 1449.21, 252.47, 22.5),
    (HardwareButton::Load, 1449.21, 332.13, 22.5),
    (HardwareButton::Save, 1449.21, 413.93, 22.5),
    (HardwareButton::Copy, 1449.21, 495.06, 22.5),
    // Under-screen row
    (HardwareButton::Time, 1143.22, 414.04, 22.5),
    (HardwareButton::Quantize, 1216.34, 414.04, 22.5),
    (HardwareButton::Automation, 1289.45, 414.04, 22.5),
    (HardwareButton::Transform, 1362.57, 414.04, 22.5),
    // View toggle
    (HardwareButton::Session, 856.35, 413.93, 22.5),
    (HardwareButton::Clip, 856.35, 495.74, 22.5),
    (HardwareButton::Scope, 694.40, 453.37, 22.5),
    // Mode row
    (HardwareButton::Keyboard, 1017.07, 495.74, 22.5),
    (HardwareButton::Scale, 1159.26, 495.74, 22.5),
    (HardwareButton::Loop, 1289.45, 495.74, 22.5),
    // Encoder function buttons
    (HardwareButton::EncoderFunction1, 370.13, 332.13, 22.5),
    (HardwareButton::EncoderFunction2, 451.13, 332.13, 22.5),
    (HardwareButton::EncoderFunction3, 532.45, 332.13, 22.5),
    (HardwareButton::EncoderFunction4, 614.06, 332.13, 22.5),
    (HardwareButton::EncoderFunction5, 694.02, 332.13, 22.5),
    (HardwareButton::EncoderFunction6, 775.33, 332.13, 22.5),
    (HardwareButton::EncoderFunction7, 856.35, 332.13, 22.5),
    (HardwareButton::EncoderFunction8, 937.66, 332.13, 22.5),
];

/// Pad grid constants
const PAD_BASE_X: f32 = 135.153;
const PAD_BASE_Y: f32 = 565.641 + 10.0; // Adjusted to align with SVG pads
const PAD_SIZE: f32 = 69.5;
const PAD_RADIUS: f32 = 6.9;

/// Per-column X offsets (18 columns: 16 main + 2 sidebar)
const PAD_COL_OFFSETS: [f32; 18] = [
    0.0, 106.47, 213.0, 319.47, 425.0, 531.47, 638.0, 744.47, 851.0, 957.47, 1064.0, 1170.47,
    1276.0, 1382.47, 1489.0, 1595.47, 1764.0, 1870.47,
];

/// Per-row Y offsets (8 rows)
const PAD_ROW_OFFSETS: [f32; 8] = [
    -11.4482, 94.9539, 201.356, 307.758, 414.16, 520.562, 626.964, 733.366,
];

/// Persistent canvas renderer for dynamic elements (pad grid, OLED, interactive overlays).
/// Lives as a field on `DelugeSimulator` — never cloned. Each rendering layer
/// has its own `canvas::Cache` that is only redrawn when explicitly invalidated.
pub(crate) struct DynamicElementsRenderer {
    pub(crate) display: SimulatorDisplay,
    pub(crate) grid: PadGrid,
    pub(crate) hardware: DelugeHardware,
    pub(crate) sticky_keys_enabled: bool,
    pub(crate) pressed_buttons: HashSet<HardwareButton>,

    // Cached rendering layers — only redrawn when cleared
    pub(crate) oled_cache: canvas::Cache,
    pub(crate) pad_cache: canvas::Cache,
    pub(crate) controls_cache: canvas::Cache,
}

/// Persistent state for the canvas program (survives across re-renders)
#[derive(Default)]
pub(crate) struct CanvasState {
    /// Accumulated pixel scroll delta for smooth trackpad scrolling
    scroll_accumulator: f32,
    /// Pad currently held down by the left mouse button, as (col, row).
    /// Used to emit a release when the cursor is dragged off the pad.
    held_pad: Option<(usize, usize)>,
}

impl DynamicElementsRenderer {
    pub(crate) fn new(display: SimulatorDisplay, grid: PadGrid, hardware: DelugeHardware) -> Self {
        Self {
            display,
            grid,
            hardware,
            sticky_keys_enabled: false,
            pressed_buttons: HashSet::new(),
            oled_cache: canvas::Cache::new(),
            pad_cache: canvas::Cache::new(),
            controls_cache: canvas::Cache::new(),
        }
    }
}

impl canvas::Program<SimulatorMessage> for DynamicElementsRenderer {
    type State = CanvasState;

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &iced::Theme,
        bounds: IcedRect,
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry<Renderer>> {
        // Calculate dynamic scale based on actual bounds
        // Original SVG is 2178x1482, we need to scale based on the smaller dimension to maintain aspect ratio
        let scale_x = bounds.width / 2178.0;
        let scale_y = bounds.height / 1482.0;
        let scale = scale_x.min(scale_y); // Use smaller scale to fit within bounds

        // Calculate centering offset for letterboxing/pillarboxing
        let scaled_width = 2178.0 * scale;
        let scaled_height = 1482.0 * scale;
        let offset_x = (bounds.width - scaled_width) / 2.0;
        let offset_y = (bounds.height - scaled_height) / 2.0;

        // Each layer is cached independently and only redrawn when invalidated
        let pad_geo = self.pad_cache.draw(renderer, bounds.size(), |frame| {
            Self::draw_pad_grid(frame, &self.grid, scale, offset_x, offset_y);
        });

        let oled_geo = self.oled_cache.draw(renderer, bounds.size(), |frame| {
            Self::draw_oled(frame, &self.display, scale, offset_x, offset_y);
        });

        let controls_geo = self.controls_cache.draw(renderer, bounds.size(), |frame| {
            self.draw_encoders(frame, scale, offset_x, offset_y);
            self.draw_buttons(frame, scale, offset_x, offset_y);
            self.draw_leds(frame, scale, offset_x, offset_y);
            if self.sticky_keys_enabled {
                self.draw_sticky_keys_indicator(frame, scale, offset_x, offset_y);
            }
        });

        vec![pad_geo, oled_geo, controls_geo]
    }

    fn mouse_interaction(
        &self,
        _state: &Self::State,
        bounds: IcedRect,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if cursor.is_over(bounds) {
            mouse::Interaction::Pointer
        } else {
            mouse::Interaction::default()
        }
    }

    fn update(
        &self,
        state: &mut Self::State,
        event: &event::Event,
        bounds: IcedRect,
        cursor: mouse::Cursor,
    ) -> Option<canvas::Action<SimulatorMessage>> {
        // Calculate dynamic scale and offset (same as draw method)
        let scale_x = bounds.width / 2178.0;
        let scale_y = bounds.height / 1482.0;
        let scale = scale_x.min(scale_y);

        let scaled_width = 2178.0 * scale;
        let scaled_height = 1482.0 * scale;
        let offset_x = (bounds.width - scaled_width) / 2.0;
        let offset_y = (bounds.height - scaled_height) / 2.0;

        // Dragging the cursor off a held pad releases it: the pad is no longer
        // under the mouse, so it can't still be held. The cursor may have left
        // the canvas entirely, so this is checked before `position_in`.
        if let event::Event::Mouse(mouse::Event::CursorMoved { .. }) = event
            && let Some(held) = state.held_pad
        {
            let cursor_pad = cursor
                .position_in(bounds)
                .and_then(|p| Self::get_pad_at_position(p, scale, offset_x, offset_y));
            if cursor_pad != Some(held) {
                state.held_pad = None;
                return Some(canvas::Action::publish(SimulatorMessage::PadReleased {
                    col: held.0,
                    row: held.1,
                }));
            }
        }

        if let Some(position) = cursor.position_in(bounds) {
            match event {
                event::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                    // Check pad grid clicks
                    if let Some((col, row)) =
                        Self::get_pad_at_position(position, scale, offset_x, offset_y)
                    {
                        state.held_pad = Some((col, row));
                        return Some(
                            canvas::Action::publish(SimulatorMessage::PadPressed { col, row })
                                .and_capture(),
                        );
                    }

                    // Check button clicks
                    if let Some(button) =
                        self.get_button_at_position(position, scale, offset_x, offset_y)
                    {
                        return Some(
                            canvas::Action::publish(SimulatorMessage::ButtonPressed(button))
                                .and_capture(),
                        );
                    }

                    // Check encoder clicks - now pressing the encoder itself
                    if let Some(encoder) =
                        self.get_encoder_at_position(position, scale, offset_x, offset_y)
                    {
                        return Some(
                            canvas::Action::publish(SimulatorMessage::EncoderPressed(encoder))
                                .and_capture(),
                        );
                    }
                }
                event::Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                    // Check button releases
                    if let Some(button) =
                        self.get_button_at_position(position, scale, offset_x, offset_y)
                    {
                        return Some(
                            canvas::Action::publish(SimulatorMessage::ButtonReleased(button))
                                .and_capture(),
                        );
                    }

                    // Release the held pad (the one pressed), regardless of where
                    // the cursor currently sits within that pad.
                    if let Some((col, row)) = state.held_pad.take() {
                        return Some(
                            canvas::Action::publish(SimulatorMessage::PadReleased { col, row })
                                .and_capture(),
                        );
                    }

                    // Check encoder releases
                    if let Some(encoder) =
                        self.get_encoder_at_position(position, scale, offset_x, offset_y)
                    {
                        return Some(
                            canvas::Action::publish(SimulatorMessage::EncoderReleased(encoder))
                                .and_capture(),
                        );
                    }
                }
                event::Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                    // Check if scrolling over an encoder
                    if let Some(encoder) =
                        self.get_encoder_at_position(position, scale, offset_x, offset_y)
                    {
                        let rotation_delta = match delta {
                            mouse::ScrollDelta::Lines { y, .. } => {
                                // Line-based scrolling (discrete mouse wheels):
                                // Each line = one encoder tick. Reset pixel accumulator.
                                state.scroll_accumulator = 0.0;
                                if *y > 0.0 {
                                    -1
                                } else if *y < 0.0 {
                                    1
                                } else {
                                    0
                                }
                            }
                            mouse::ScrollDelta::Pixels { y, .. } => {
                                // Pixel-based scrolling (trackpads):
                                // Accumulate small deltas and fire when threshold is reached.
                                const PIXEL_THRESHOLD: f32 = 20.0;
                                state.scroll_accumulator += y;
                                if state.scroll_accumulator > PIXEL_THRESHOLD {
                                    state.scroll_accumulator -= PIXEL_THRESHOLD;
                                    -1 // scroll up = counter-clockwise
                                } else if state.scroll_accumulator < -PIXEL_THRESHOLD {
                                    state.scroll_accumulator += PIXEL_THRESHOLD;
                                    1 // scroll down = clockwise
                                } else {
                                    0
                                }
                            }
                        };

                        if rotation_delta != 0 {
                            return Some(
                                canvas::Action::publish(SimulatorMessage::EncoderRotated {
                                    encoder,
                                    delta: rotation_delta,
                                })
                                .and_capture(),
                            );
                        }

                        // Capture the event even if no tick yet — prevents iced from scrolling the window
                        return Some(canvas::Action::capture());
                    }
                }
                _ => {}
            }
        }

        // Handle keyboard events (not position-dependent)
        if let event::Event::Keyboard(iced::keyboard::Event::KeyPressed { key, .. }) = event
            && let iced::keyboard::Key::Character(c) = key
            && (c.as_str() == "s" || c.as_str() == "S")
        {
            return Some(canvas::Action::publish(SimulatorMessage::ToggleStickyKeys));
        }

        None
    }
}

// Drawing methods
impl DynamicElementsRenderer {
    fn draw_pad_grid(frame: &mut Frame, grid: &PadGrid, scale: f32, offset_x: f32, offset_y: f32) {
        for (row, &row_offset) in PAD_ROW_OFFSETS.iter().enumerate() {
            for (col, &col_offset) in PAD_COL_OFFSETS.iter().enumerate() {
                let x = offset_x + (PAD_BASE_X + col_offset) * scale;
                let y = offset_y + (PAD_BASE_Y + row_offset) * scale;
                let w = PAD_SIZE * scale;
                let h = PAD_SIZE * scale;
                let center_x = x + w / 2.0;
                let center_y = y + h / 2.0;

                let pad_color = grid.get(col, PAD_ROWS - row - 1);
                let is_pressed = grid.is_pressed(col, PAD_ROWS - row - 1);

                let rounded_rect = canvas::Path::rounded_rectangle(
                    Point::new(x, y),
                    Size::new(w, h),
                    (PAD_RADIUS * scale).into(),
                );

                // Determine if the LED is lit (color is not black)
                let is_lit = pad_color != RGB::black();

                if is_lit {
                    // Draw LED glow effects underneath/through the pad

                    let led_color = if is_pressed {
                        pad_color.dim_float(0.6)
                    } else {
                        pad_color
                    };

                    // Layer 1: Outer glow (large, soft halo) - simulates light bleeding
                    let outer_radius = w.max(h) * 0.7;
                    Self::draw_radial_glow(
                        frame,
                        center_x,
                        center_y,
                        outer_radius,
                        led_color.to_iced_color(),
                        0.15, // Low opacity for soft glow
                    );

                    // Layer 2: Medium glow - main light diffusion through translucent material
                    let mid_radius = w.max(h) * 0.5;
                    Self::draw_radial_glow(
                        frame,
                        center_x,
                        center_y,
                        mid_radius,
                        led_color.to_iced_color(),
                        0.4, // Medium opacity
                    );

                    // Layer 3: Inner glow - bright LED hotspot
                    let inner_radius = w.max(h) * 0.3;
                    Self::draw_radial_glow(
                        frame,
                        center_x,
                        center_y,
                        inner_radius,
                        led_color.brighten(0.3).to_iced_color(),
                        0.7, // High opacity for bright center
                    );

                    // Draw translucent white/grey pad material on top
                    // This simulates the diffused light through the translucent silicone
                    let pad_base_color = Color::from_rgba(
                        0.9, // Slight warm tint
                        0.9, 0.92, 0.3, // Semi-transparent to show LED glow beneath
                    );
                    frame.fill(&rounded_rect, pad_base_color);

                    // Add the LED color mixed into the pad surface
                    let surface_color = Color::from_rgba(
                        led_color.r as f32 / 255.0,
                        led_color.g as f32 / 255.0,
                        led_color.b as f32 / 255.0,
                        0.6, // Translucent
                    );
                    frame.fill(&rounded_rect, surface_color);
                } else {
                    // Pad is off - draw translucent grey pad only
                    let pad_off_color = Color::from_rgba(0.85, 0.85, 0.87, 0.4);
                    frame.fill(&rounded_rect, pad_off_color);
                }

                // Border with rounded corners (darker for depth)
                frame.stroke(
                    &rounded_rect,
                    iced::widget::canvas::Stroke::default()
                        .with_color(Color::from_rgba(0.12, 0.12, 0.12, 0.6))
                        .with_width(1.5 * scale),
                );
            }
        }
    }

    /// Draw a radial glow effect (simulates light diffusion)
    fn draw_radial_glow(
        frame: &mut Frame,
        center_x: f32,
        center_y: f32,
        radius: f32,
        color: Color,
        max_alpha: f32,
    ) {
        // Draw concentric circles with decreasing opacity to simulate radial gradient
        let steps = 12; // Number of gradient steps
        for i in 0..steps {
            let t = i as f32 / steps as f32;
            let r = radius * (1.0 - t);

            // Quadratic falloff for more natural glow
            let alpha = max_alpha * (1.0 - t * t);

            let glow_color = Color::from_rgba(color.r, color.g, color.b, alpha);

            frame.fill(
                &canvas::Path::circle(Point::new(center_x, center_y), r),
                glow_color,
            );
        }
    }

    fn draw_oled(
        frame: &mut Frame,
        display: &SimulatorDisplay,
        scale: f32,
        offset_x: f32,
        offset_y: f32,
    ) {
        // From SVG: <rect x="1105.9" y="274.347" width="275.943" height="88.191"/>
        let x = offset_x + 1105.9 * scale;
        let y = offset_y + 274.347 * scale;
        let w = 275.943 * scale;
        let h = 88.191 * scale;

        // OLED background (dark)
        frame.fill_rectangle(
            Point::new(x, y),
            Size::new(w, h),
            Color::from_rgb(0.0, 0.0, 0.05),
        );

        // Draw OLED pixels scaled to fit the rectangle
        let pixel_w = w / OLED_WIDTH as f32;
        let pixel_h = h / OLED_HEIGHT as f32;

        for py in 0..OLED_HEIGHT {
            for px in 0..OLED_WIDTH {
                if display.get_pixel(px, py) {
                    frame.fill_rectangle(
                        Point::new(x + px as f32 * pixel_w, y + py as f32 * pixel_h),
                        Size::new(pixel_w, pixel_h),
                        Color::from_rgb(0.3, 0.6, 0.9), // Blue OLED
                    );
                }
            }
        }

        // OLED border
        frame.stroke_rectangle(
            Point::new(x, y),
            Size::new(w, h),
            iced::widget::canvas::Stroke::default()
                .with_color(Color::from_rgb(0.2, 0.2, 0.2))
                .with_width(2.0 * scale),
        );
    }

    fn draw_encoders(&self, frame: &mut Frame, scale: f32, offset_x: f32, offset_y: f32) {
        for (encoder, cx, cy, radius) in ENCODER_POSITIONS {
            let scaled_cx = offset_x + cx * scale;
            let scaled_cy = offset_y + cy * scale;
            let scaled_r = radius * scale;

            // Always draw static encoder outline
            frame.stroke(
                &canvas::Path::circle(Point::new(scaled_cx, scaled_cy), scaled_r),
                iced::widget::canvas::Stroke::default()
                    .with_color(Color::from_rgb(0.3, 0.3, 0.3)) // Gray outline
                    .with_width(2.0 * scale),
            );

            // Rotation indicator angle. The Volume knob is a pot: its 0–100 %
            // value sweeps a fixed ~270° arc (8 o'clock → 4 o'clock through the
            // top). Every other encoder is endless, so its accumulated value just
            // keeps spinning.
            let value = self.hardware.get_encoder_value(encoder);
            let angle = if encoder == HardwareEncoder::Volume {
                let frac = (value.clamp(0, 100) as f32) / 100.0;
                -std::f32::consts::FRAC_PI_2 + (frac - 0.5) * 1.5 * std::f32::consts::PI
            } else {
                let raw_angle = value as f32 * 0.1;
                raw_angle.rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
            };

            // Draw rotation indicator line
            let line_len = scaled_r * 0.6;
            let end_x = scaled_cx + angle.cos() * line_len;
            let end_y = scaled_cy + angle.sin() * line_len;

            frame.stroke(
                &canvas::Path::line(Point::new(scaled_cx, scaled_cy), Point::new(end_x, end_y)),
                iced::widget::canvas::Stroke::default()
                    .with_color(Color::from_rgba(1.0, 1.0, 1.0, 0.8))
                    .with_width(3.0 * scale),
            );

            // Draw small center dot
            frame.fill(
                &canvas::Path::circle(Point::new(scaled_cx, scaled_cy), 4.0 * scale),
                Color::from_rgba(1.0, 1.0, 1.0, 0.9),
            );
        }
    }

    fn draw_buttons(&self, frame: &mut Frame, scale: f32, offset_x: f32, offset_y: f32) {
        for (button, x, y, radius) in BUTTON_POSITIONS {
            let scaled_x = offset_x + x * scale;
            let scaled_y = offset_y + y * scale;
            let scaled_r = radius * scale;

            let led = HardwareLED::from(button);
            let led_on = self.hardware.is_led_on(led);
            let pressed = self.pressed_buttons.contains(&button);
            let is_sticky = self.sticky_keys_enabled && pressed;

            // Button *illumination* is the app's job: draw the coloured LED glow
            // whenever the app lights the LED — independent of whether the button
            // is physically held. (Drawn first so the ring stays visible on top.)
            if led_on {
                let (r, g, b) = led.color();
                let led_color = Color::from_rgb(r, g, b);
                Self::draw_radial_glow(frame, scaled_x, scaled_y, scaled_r * 1.3, led_color, 0.4);
                Self::draw_radial_glow(frame, scaled_x, scaled_y, scaled_r * 0.9, led_color, 0.6);
                Self::draw_radial_glow(frame, scaled_x, scaled_y, scaled_r * 0.5, led_color, 1.0);
                frame.fill(
                    &canvas::Path::circle(Point::new(scaled_x, scaled_y), scaled_r * 0.8),
                    Color::from_rgba(0.95, 0.95, 0.97, 0.1),
                );
            }

            // The only press feedback is the ring: a brighter/thicker outline
            // while held (cyan for a sticky-held button). A press never lights
            // the cap — that would impersonate an app-controlled LED.
            let (ring_color, ring_width) = if is_sticky {
                (Color::from_rgb(0.0, 0.9, 1.0), 2.5)
            } else if pressed {
                (Color::from_rgb(0.9, 0.9, 0.95), 2.5)
            } else {
                (Color::from_rgb(0.4, 0.4, 0.4), 1.5)
            };
            frame.stroke(
                &canvas::Path::circle(Point::new(scaled_x, scaled_y), scaled_r),
                iced::widget::canvas::Stroke::default()
                    .with_color(ring_color)
                    .with_width(ring_width * scale),
            );
        }
    }

    fn draw_leds(&self, frame: &mut Frame, scale: f32, offset_x: f32, offset_y: f32) {
        // LED positions from SVG
        // Upper gold encoder indicators (4 segments)
        // Base rect: x="470.39" y="393.283" width="21.097" height="20.803"
        // Transforms: translate_y = -143.217, -176.24, -209.264, -242.287
        let upper_leds = [
            (HardwareLED::UpperGoldIndicator1, 672.914, 250.066), // y = 393.283 - 143.217
            (HardwareLED::UpperGoldIndicator2, 672.914, 217.043), // y = 393.283 - 176.24
            (HardwareLED::UpperGoldIndicator3, 672.914, 184.019), // y = 393.283 - 209.264
            (HardwareLED::UpperGoldIndicator4, 672.914, 150.996), // y = 393.283 - 242.287
        ];

        // Lower gold encoder indicators (4 segments)
        // Transforms: translate_y = 99.4142, 66.3906, 33.3671, 0.343557 (with -2 base transform)
        let lower_leds = [
            (HardwareLED::LowerGoldIndicator1, 470.041, 490.697), // y = 393.283 + 99.4142 - 2
            (HardwareLED::LowerGoldIndicator2, 470.041, 457.674), // y = 393.283 + 66.3906 - 2
            (HardwareLED::LowerGoldIndicator3, 470.041, 424.650), // y = 393.283 + 33.3671 - 2
            (HardwareLED::LowerGoldIndicator4, 470.041, 391.627), // y = 393.283 + 0.343557 - 2
        ];

        let led_width = 21.097;
        let led_height = 20.803;

        // Draw upper indicators with glow
        for (led, x, y) in upper_leds {
            if self.hardware.is_led_on(led) {
                let scaled_x = offset_x + x * scale;
                let scaled_y = offset_y + y * scale;
                let scaled_w = led_width * scale;
                let scaled_h = led_height * scale;
                let center_x = scaled_x + scaled_w / 2.0;
                let center_y = scaled_y + scaled_h / 2.0;

                let led_color = Color::from_rgb(1.0, 0.6, 0.0); // Gold/amber, matching the function buttons

                // Glow layers
                Self::draw_radial_glow(frame, center_x, center_y, scaled_w * 0.9, led_color, 0.25);
                Self::draw_radial_glow(frame, center_x, center_y, scaled_w * 0.6, led_color, 0.5);

                // LED core
                frame.fill_rectangle(
                    Point::new(scaled_x, scaled_y),
                    Size::new(scaled_w, scaled_h),
                    Color::from_rgba(1.0, 0.6, 0.0, 0.9),
                );
            }
        }

        // Draw lower indicators with glow
        for (led, x, y) in lower_leds {
            if self.hardware.is_led_on(led) {
                let scaled_x = offset_x + x * scale;
                let scaled_y = offset_y + y * scale;
                let scaled_w = led_width * scale;
                let scaled_h = led_height * scale;
                let center_x = scaled_x + scaled_w / 2.0;
                let center_y = scaled_y + scaled_h / 2.0;

                let led_color = Color::from_rgb(1.0, 0.6, 0.0); // Gold/amber, matching the function buttons

                // Glow layers
                Self::draw_radial_glow(frame, center_x, center_y, scaled_w * 0.9, led_color, 0.25);
                Self::draw_radial_glow(frame, center_x, center_y, scaled_w * 0.6, led_color, 0.5);

                // LED core
                frame.fill_rectangle(
                    Point::new(scaled_x, scaled_y),
                    Size::new(scaled_w, scaled_h),
                    Color::from_rgba(1.0, 0.6, 0.0, 0.9),
                );
            }
        }

        // Synced LED - from SVG: transform="matrix(0.799982,0,0,0.799982,358.176,50.5576)"
        // Base circle: cx="1791.35" cy="252.765" r="8.109"
        // Actual position: x = 0.799982 * 1791.35 + 358.176 = 1791.23
        //                  y = 0.799982 * 252.765 + 50.5576 = 252.78
        if self.hardware.is_led_on(HardwareLED::Synced) {
            let synced_x = offset_x + 1791.23 * scale;
            let synced_y = offset_y + 252.78 * scale;
            let synced_r = 8.109 * scale; // Match SVG radius

            let led_color = Color::from_rgb(1.0, 0.0, 0.0); // Red LED

            // Glow layers for synced LED
            Self::draw_radial_glow(frame, synced_x, synced_y, synced_r * 2.5, led_color, 0.2);
            Self::draw_radial_glow(frame, synced_x, synced_y, synced_r * 1.5, led_color, 0.4);

            // LED core
            frame.fill(
                &canvas::Path::circle(Point::new(synced_x, synced_y), synced_r),
                Color::from_rgba(1.0, 0.0, 0.0, 0.9),
            );
        }
    }

    fn draw_sticky_keys_indicator(
        &self,
        frame: &mut Frame,
        scale: f32,
        offset_x: f32,
        offset_y: f32,
    ) {
        // Draw a cyan "STICKY KEYS" indicator in the top-left corner
        let indicator_x = offset_x + 20.0 * scale;
        let indicator_y = offset_y + 20.0 * scale;
        let box_width = 200.0 * scale;
        let box_height = 40.0 * scale;

        // Draw background rectangle
        frame.fill_rectangle(
            Point::new(indicator_x, indicator_y),
            Size::new(box_width, box_height),
            Color::from_rgba(0.0, 0.8, 0.8, 0.8), // Cyan with some transparency
        );

        // Draw border
        let border_path = canvas::Path::rectangle(
            Point::new(indicator_x, indicator_y),
            Size::new(box_width, box_height),
        );
        frame.stroke(
            &border_path,
            canvas::Stroke::default()
                .with_color(Color::from_rgb(0.0, 1.0, 1.0))
                .with_width(2.0 * scale),
        );

        // Draw "STICKY KEYS" text
        let text = canvas::Text {
            content: format!("STICKY KEYS ({})", self.pressed_buttons.len()),
            position: Point::new(indicator_x + 10.0 * scale, indicator_y + 8.0 * scale),
            color: Color::BLACK,
            size: iced::Pixels(18.0 * scale),
            ..Default::default()
        };
        frame.fill_text(text);
    }
}

// Hit-testing methods
impl DynamicElementsRenderer {
    fn get_pad_at_position(
        position: Point,
        scale: f32,
        offset_x: f32,
        offset_y: f32,
    ) -> Option<(usize, usize)> {
        let base_x = offset_x + PAD_BASE_X * scale;
        let base_y = offset_y + PAD_BASE_Y * scale;
        let pad_width = PAD_SIZE * scale;
        let pad_height = PAD_SIZE * scale;

        // Check each pad
        for (row, &row_offset) in PAD_ROW_OFFSETS.iter().enumerate() {
            for (col, &col_offset) in PAD_COL_OFFSETS.iter().enumerate() {
                let x = base_x + col_offset * scale;
                let y = base_y + row_offset * scale;

                if position.x >= x
                    && position.x <= x + pad_width
                    && position.y >= y
                    && position.y <= y + pad_height
                {
                    return Some((col, 7 - row));
                }
            }
        }

        None
    }

    fn get_button_at_position(
        &self,
        position: Point,
        scale: f32,
        offset_x: f32,
        offset_y: f32,
    ) -> Option<HardwareButton> {
        for (button, x, y, radius) in BUTTON_POSITIONS {
            let scaled_x = offset_x + x * scale;
            let scaled_y = offset_y + y * scale;
            let scaled_r = radius * scale;

            // Check if position is within button circle
            let dx = position.x - scaled_x;
            let dy = position.y - scaled_y;
            let distance = (dx * dx + dy * dy).sqrt();

            if distance <= scaled_r {
                return Some(button);
            }
        }

        None
    }

    fn get_encoder_at_position(
        &self,
        position: Point,
        scale: f32,
        offset_x: f32,
        offset_y: f32,
    ) -> Option<HardwareEncoder> {
        for (encoder, cx, cy, radius) in ENCODER_POSITIONS {
            let scaled_cx = offset_x + cx * scale;
            let scaled_cy = offset_y + cy * scale;
            let scaled_r = radius * scale;

            // Check if position is within encoder circle
            let dx = position.x - scaled_cx;
            let dy = position.y - scaled_cy;
            let distance = (dx * dx + dy * dy).sqrt();

            if distance <= scaled_r {
                return Some(encoder);
            }
        }

        None
    }
}
