//! Oscilloscope trace rendering, ported from spark's vello visualiser
//! (`apps/desktop/.../viz_renderer.rs::build_waveform_scene`) to iced Canvas.
//!
//! The look matches spark's rack analyzer scope: a faint centre line, a wide
//! semi-transparent "glow" underlay, then the main trace on top. An optional
//! rising-edge trigger (search for a rising zero-crossing in the middle of the
//! buffer, place it at the horizontal centre) makes a periodic signal stand
//! still, exactly like a hardware scope with the trigger at 50 %. CV/gate are
//! slow and aperiodic, so they pass `trigger = false` and just show the rolling
//! window; audio passes `trigger = true`.

use iced::widget::canvas::{Frame, Path, Stroke};
use iced::{Color, Point, Rectangle, Size};

/// Scope well background (spark `BG_COLOR`).
pub const BG: Color = Color::from_rgb(0.063, 0.063, 0.071);
/// Centre baseline (spark `CENTER_LINE_COLOR`, white @ ~9 %).
const CENTER: Color = Color::from_rgba(1.0, 1.0, 1.0, 0.094);
/// Hairline well border.
const BORDER: Color = Color::from_rgb(0.20, 0.20, 0.24);

/// Draw a scope trace of `samples` (each in `-1.0..=1.0`, oldest first) into
/// `area`, in `color`. `trigger` enables the rising-edge 50 % trigger.
pub fn draw_trace(
    frame: &mut Frame,
    area: Rectangle,
    samples: &[f32],
    color: Color,
    trigger: bool,
) {
    let Rectangle {
        x: ox,
        y: oy,
        width: w,
        height: h,
    } = area;

    let well = Path::new(|b| b.rounded_rectangle(Point::new(ox, oy), Size::new(w, h), 2.0.into()));
    frame.fill(&well, BG);
    frame.stroke(&well, Stroke::default().with_color(BORDER).with_width(1.0));

    let half_h = h * 0.5;
    let cy = oy + half_h;
    frame.stroke(
        &Path::line(Point::new(ox, cy), Point::new(ox + w, cy)),
        Stroke::default().with_color(CENTER).with_width(1.0),
    );

    if samples.len() < 2 {
        return;
    }
    let n = samples.len();

    // ── Rising-edge trigger (spark): display half the buffer, centred on the
    // crossing so a periodic wave is stationary. Aperiodic signals skip it.
    let (start, len) = if trigger {
        let display_n = (n / 2).max(2);
        let half = display_n / 2;
        let trigger = (half..n.saturating_sub(half))
            .find(|&i| samples[i - 1] < 0.0 && samples[i] >= 0.0)
            .unwrap_or(half);
        (trigger - half, display_n)
    } else {
        (0, n)
    };
    let view = &samples[start..(start + len).min(n)];
    let vn = view.len();
    if vn < 2 {
        return;
    }

    let pt = |i: usize, s: f32| -> Point {
        let x = ox + (i as f32 / (vn - 1) as f32) * w;
        let y = cy - s.clamp(-1.0, 1.0) * half_h * 0.9;
        Point::new(x, y)
    };
    let trace = || {
        Path::new(|b| {
            b.move_to(pt(0, view[0]));
            for (i, &s) in view.iter().enumerate().skip(1) {
                b.line_to(pt(i, s));
            }
        })
    };

    // Glow underlay, then the crisp line on top (spark's two-pass stroke).
    let glow = Color { a: 0.25, ..color };
    frame.stroke(&trace(), Stroke::default().with_color(glow).with_width(4.0));
    frame.stroke(
        &trace(),
        Stroke::default().with_color(color).with_width(1.5),
    );
}
