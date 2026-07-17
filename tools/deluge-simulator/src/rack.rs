//! The CV/gate visualiser strip that sits **above the faceplate** — over the
//! Deluge's back-panel jacks. Each CV and gate output gets an indicator placed
//! directly above its port (the jack x-positions are baked from the faceplate
//! SVG), so the strip reads as an extension of the back panel.
//!
//! Two display modes, toggled by clicking the strip:
//!   - **meters** (default): a vertical bar per CV channel (0–10 V) and a lit
//!     dot per gate channel — a quick at-a-glance indicator.
//!   - **scopes**: a rolling oscilloscope trace per channel, drawn by [`scope`]
//!     (ported from spark's analyzer scope).
//!
//! The strip shares the faceplate's horizontal scale (`width / SVG_WIDTH`), so
//! indicators line up with the jacks below. Fed by the same illumination stream:
//! the in-process link mirrors `SharedPanel`'s CV/gate snapshots, the protocol
//! link applies `ToDeluge::SetCv` / `SetGate`, and the GUI samples a history
//! point each frame so the scopes have a time axis.

use std::collections::VecDeque;

use iced::widget::canvas::{self, Frame, Path, Stroke};
use iced::{Color, Point, Rectangle, Renderer, Size, Theme, event, mouse};

use crate::app::SimulatorMessage;
use crate::scope;

/// CV output channels on the device (`deluge::Cv::CHANNELS`).
const CV_CHANNELS: usize = 2;
/// Gate output channels on the device (`deluge::Gate::CHANNELS`).
const GATE_CHANNELS: usize = 4;

/// Padding (px) above and below the bottom-aligned indicators.
const TOP_PAD: f32 = 10.0;
const BOTTOM_PAD: f32 = 12.0;

/// Faceplate SVG width — the shared coordinate space for jack alignment.
const SVG_WIDTH: f32 = 2178.0;
/// Jack centre x (SVG space) for GATE OUT 1–4, read off the faceplate art.
const GATE_X: [f32; GATE_CHANNELS] = [530.0, 623.0, 716.0, 810.0];
/// Jack centre x (SVG space) for CV 1–2.
const CV_X: [f32; CV_CHANNELS] = [900.0, 988.0];

/// Width (px) of a CV bar meter (compact mode) — deliberately thin.
const METER_W: f32 = 16.0;
/// Width (px) of a CV/gate scope well (expanded mode). Narrow so adjacent
/// channels don't overlap at the tight jack spacing (CV jacks are ~35 px apart
/// at the faceplate scale).
const SCOPE_W: f32 = 30.0;

/// History depth for the CV/gate scope traces (frames; ~4 s at 60 Hz).
const HIST_CAP: usize = 240;
/// History depth for the audio scope (samples; ~23 ms at 44.1 kHz).
const AUDIO_HIST: usize = 1024;

/// Centre x (SVG space) for the L and R audio output scopes, over L/MONO–R
/// OUTPUT. Spaced 320 SVG (≈160 px) apart so the 150 px wells don't overlap.
const AUDIO_L_X: f32 = 1510.0;
const AUDIO_R_X: f32 = 1890.0;
/// Width (px) of each audio scope well.
const AUDIO_W: f32 = 180.0;

/// Centre x (SVG space) for the MIDI IN / OUT activity indicators.
const MIDI_IN_X: f32 = 1100.0;
const MIDI_OUT_X: f32 = 1250.0;
/// Per-frame decay of the MIDI activity flash (so a burst fades over ~0.3 s).
const MIDI_DECAY: f32 = 0.85;

/// Strip background (matches the faceplate's near-black back panel).
const BG: Color = Color::from_rgb(0.04, 0.04, 0.05);
const BORDER: Color = Color::from_rgb(0.20, 0.20, 0.24);
const LABEL: Color = Color::from_rgb(0.62, 0.62, 0.68);
const WELL: Color = Color::from_rgb(0.03, 0.03, 0.04);
/// CV trace/fill (teal-green).
const CV_COLOR: Color = Color::from_rgb(0.20, 0.80, 0.60);
/// Gate trace/fill (amber).
const GATE_COLOR: Color = Color::from_rgb(0.95, 0.65, 0.25);
const GATE_OFF: Color = Color::from_rgb(0.12, 0.13, 0.15);
/// Audio trace (blue).
const AUDIO_COLOR: Color = Color::from_rgb(0.35, 0.65, 0.95);

/// The back-panel CV/gate strip. Owns its own render cache + signal history;
/// lives as a field on `DelugeSimulator` and is never cloned.
pub(crate) struct InstrumentRack {
    /// Raw 16-bit CV DAC codes, indexed by channel.
    cv: [u16; 4],
    /// Gate line states, indexed by channel.
    gate: [bool; 4],
    /// Per-CV-channel scope history, normalised to −1..1 (0 V → −1, 10 V → +1).
    cv_hist: [VecDeque<f32>; CV_CHANNELS],
    /// Per-gate-channel scope history (−0.9 / +0.9 square).
    gate_hist: [VecDeque<f32>; GATE_CHANNELS],
    /// Audio output scope history per channel (−1..1, sample-rate).
    audio_l_hist: VecDeque<f32>,
    audio_r_hist: VecDeque<f32>,
    /// MIDI IN / OUT activity levels (1.0 on traffic, decaying each frame).
    midi_in_level: f32,
    midi_out_level: f32,
    /// Scope mode (vs. compact meters). Toggled by clicking the strip body.
    expanded: bool,
    /// Collapsed into a thin handle bar. Toggled by the triangle handle.
    collapsed: bool,
    cache: canvas::Cache,
}

/// Collapse-handle (triangle) tab geometry: a bottom-left tab, inset from the
/// left edge so it sits left of the faceplate graphic.
const HANDLE_W: f32 = 42.0;
const HANDLE_H: f32 = 14.0;
const HANDLE_MARGIN: f32 = 10.0;

impl InstrumentRack {
    pub(crate) fn new() -> Self {
        Self {
            cv: [0; 4],
            gate: [false; 4],
            // Pre-fill the histories to their full length with a baseline so the
            // scopes show a fixed-width, scrolling time window from the very first
            // frame, rather than a trace that accumulates/rescales as it fills.
            cv_hist: std::array::from_fn(|_| VecDeque::from(vec![-1.0; HIST_CAP])),
            gate_hist: std::array::from_fn(|_| VecDeque::from(vec![-0.9; HIST_CAP])),
            audio_l_hist: VecDeque::from(vec![0.0; AUDIO_HIST]),
            audio_r_hist: VecDeque::from(vec![0.0; AUDIO_HIST]),
            midi_in_level: 0.0,
            midi_out_level: 0.0,
            expanded: false,
            collapsed: false,
            cache: canvas::Cache::new(),
        }
    }

    /// Flash the MIDI IN indicator (GUI → app traffic this frame).
    pub(crate) fn flash_midi_in(&mut self) {
        self.midi_in_level = 1.0;
    }

    /// Flash the MIDI OUT indicator (app → GUI traffic this frame).
    pub(crate) fn flash_midi_out(&mut self) {
        self.midi_out_level = 1.0;
    }

    /// Append one stereo audio frame to the L/R scope histories (called per
    /// drained frame each frame from the output monitor tap).
    pub(crate) fn push_audio(&mut self, l: f32, r: f32) {
        push_capped(&mut self.audio_l_hist, l, AUDIO_HIST);
        push_capped(&mut self.audio_r_hist, r, AUDIO_HIST);
    }

    /// Collapse/expand the strip (hide its contents down to the handle, or restore them).
    pub(crate) fn toggle_collapsed(&mut self) {
        self.collapsed = !self.collapsed;
        self.cache.clear();
    }

    /// Replace all CV codes (in-process snapshot path).
    pub(crate) fn set_cv(&mut self, cv: [u16; 4]) {
        self.cv = cv;
        self.cache.clear();
    }

    /// Replace all gate states (in-process snapshot path).
    pub(crate) fn set_gate(&mut self, gate: [bool; 4]) {
        self.gate = gate;
        self.cache.clear();
    }

    /// Set a single CV channel (protocol `SetCv` path).
    pub(crate) fn set_cv_channel(&mut self, ch: usize, code: u16) {
        if let Some(v) = self.cv.get_mut(ch) {
            *v = code;
            self.cache.clear();
        }
    }

    /// Set a single gate channel (protocol `SetGate` path).
    pub(crate) fn set_gate_channel(&mut self, ch: usize, on: bool) {
        if let Some(g) = self.gate.get_mut(ch) {
            *g = on;
            self.cache.clear();
        }
    }

    /// Toggle between compact meters and scope traces.
    pub(crate) fn toggle_expanded(&mut self) {
        self.expanded = !self.expanded;
        self.cache.clear();
    }

    /// Append one history point per channel from the current values. Called once
    /// per GUI frame so the scopes have an even time axis regardless of how often
    /// the app writes CV/gate.
    pub(crate) fn sample(&mut self) {
        for ch in 0..CV_CHANNELS {
            let norm = (self.cv[ch] as f32 / u16::MAX as f32) * 2.0 - 1.0;
            push(&mut self.cv_hist[ch], norm);
        }
        for ch in 0..GATE_CHANNELS {
            push(
                &mut self.gate_hist[ch],
                if self.gate[ch] { 0.9 } else { -0.9 },
            );
        }
        self.midi_in_level *= MIDI_DECAY;
        self.midi_out_level *= MIDI_DECAY;
        // The audio scope is always live, so repaint every frame.
        self.cache.clear();
    }
}

fn push(buf: &mut VecDeque<f32>, v: f32) {
    push_capped(buf, v, HIST_CAP);
}

fn push_capped(buf: &mut VecDeque<f32>, v: f32, cap: usize) {
    if buf.len() >= cap {
        buf.pop_front();
    }
    buf.push_back(v);
}

impl canvas::Program<SimulatorMessage> for InstrumentRack {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry<Renderer>> {
        let geometry = self.cache.draw(renderer, bounds.size(), |frame| {
            frame.fill(&Path::rectangle(Point::ORIGIN, bounds.size()), BG);
            // Seam line along the bottom, where the strip meets the faceplate edge.
            frame.stroke(
                &Path::line(
                    Point::new(0.0, bounds.height),
                    Point::new(bounds.width, bounds.height),
                ),
                Stroke::default().with_color(BORDER).with_width(1.0),
            );

            // Collapsed: nothing but the handle bar (triangle points down = expand).
            if self.collapsed {
                draw_handle(frame, bounds, false);
                return;
            }

            // Share the faceplate's width-based scale so indicators sit over jacks.
            let scale = bounds.width / SVG_WIDTH;
            let h = bounds.height;
            // Everything is bottom-aligned to a common baseline, with a little
            // padding above and below; no labels.
            let baseline = h - BOTTOM_PAD;
            let well_h = (baseline - TOP_PAD).max(8.0);
            let well_top = baseline - well_h;

            for ch in 0..CV_CHANNELS {
                let cx = CV_X[ch] * scale;
                if self.expanded {
                    let area = well_at(cx, well_top, SCOPE_W, well_h);
                    scope::draw_trace(
                        frame,
                        area,
                        slice(&self.cv_hist[ch]).as_slice(),
                        CV_COLOR,
                        false,
                    );
                } else {
                    draw_cv_meter(frame, well_at(cx, well_top, METER_W, well_h), self.cv[ch]);
                }
            }

            for ch in 0..GATE_CHANNELS {
                let cx = GATE_X[ch] * scale;
                if self.expanded {
                    let area = well_at(cx, well_top, SCOPE_W, well_h);
                    scope::draw_trace(
                        frame,
                        area,
                        slice(&self.gate_hist[ch]).as_slice(),
                        GATE_COLOR,
                        false,
                    );
                } else {
                    let r = (well_h * 0.30).min(13.0);
                    draw_gate_dot(frame, Point::new(cx, baseline - r), r, self.gate[ch]);
                }
            }

            // Audio output: a triggered scope per channel (L / R), over the
            // L/MONO–R OUTPUT jacks.
            for (cx, hist) in [
                (AUDIO_L_X, &self.audio_l_hist),
                (AUDIO_R_X, &self.audio_r_hist),
            ] {
                scope::draw_trace(
                    frame,
                    well_at(cx * scale, well_top, AUDIO_W, well_h),
                    slice(hist).as_slice(),
                    AUDIO_COLOR,
                    true,
                );
            }

            // MIDI IN / OUT activity indicators, over the MIDI ports.
            let dot_r = (well_h * 0.28).min(11.0);
            draw_midi_dot(
                frame,
                Point::new(MIDI_IN_X * scale, baseline - dot_r),
                dot_r,
                self.midi_in_level,
            );
            draw_midi_dot(
                frame,
                Point::new(MIDI_OUT_X * scale, baseline - dot_r),
                dot_r,
                self.midi_out_level,
            );

            // Collapse handle (triangle points up = collapse), bottom-left.
            draw_handle(frame, bounds, true);
        });
        vec![geometry]
    }

    fn update(
        &self,
        _state: &mut Self::State,
        event: &event::Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Option<canvas::Action<SimulatorMessage>> {
        if let event::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) = event
            && let Some(pos) = cursor.position_in(bounds)
        {
            // The triangle handle collapses/expands; the rest of the strip body
            // toggles meters ⇄ scopes (only meaningful while expanded).
            let msg = if handle_rect(bounds).contains(pos) {
                SimulatorMessage::ToggleRackCollapsed
            } else if self.collapsed {
                return None;
            } else {
                SimulatorMessage::ToggleRackScopes
            };
            return Some(canvas::Action::publish(msg).and_capture());
        }
        None
    }

    fn mouse_interaction(
        &self,
        _state: &Self::State,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if cursor.is_over(bounds) {
            mouse::Interaction::Pointer
        } else {
            mouse::Interaction::default()
        }
    }
}

/// Bottom-anchored CV bar meter in `area` (0–10 V over the well height).
fn draw_cv_meter(frame: &mut Frame, area: Rectangle, code: u16) {
    let well = Path::new(|b| {
        b.rounded_rectangle(area.position(), area.size(), 2.0.into());
    });
    frame.fill(&well, WELL);
    frame.stroke(&well, Stroke::default().with_color(BORDER).with_width(1.0));

    let frac = code as f32 / u16::MAX as f32;
    let fill_h = frac * (area.height - 2.0);
    if fill_h > 0.5 {
        let fill = Path::new(|b| {
            b.rectangle(
                Point::new(area.x + 1.0, area.y + (area.height - 1.0) - fill_h),
                Size::new(area.width - 2.0, fill_h),
            );
        });
        frame.fill(&fill, CV_COLOR);
    }
}

/// MIDI activity dot, brightness `level` (0 = idle, 1 = fresh traffic): the
/// off colour blended toward a bright lit colour.
fn draw_midi_dot(frame: &mut Frame, center: Point, r: f32, level: f32) {
    let t = level.clamp(0.0, 1.0);
    let lit = Color::from_rgb(0.30, 0.95, 0.55);
    let color = Color::from_rgb(
        GATE_OFF.r + (lit.r - GATE_OFF.r) * t,
        GATE_OFF.g + (lit.g - GATE_OFF.g) * t,
        GATE_OFF.b + (lit.b - GATE_OFF.b) * t,
    );
    let dot = Path::circle(center, r);
    frame.fill(&dot, color);
    frame.stroke(&dot, Stroke::default().with_color(BORDER).with_width(1.0));
}

/// Gate indicator dot (glowing when asserted).
fn draw_gate_dot(frame: &mut Frame, center: Point, r: f32, on: bool) {
    let dot = Path::circle(center, r);
    frame.fill(&dot, if on { GATE_COLOR } else { GATE_OFF });
    frame.stroke(&dot, Stroke::default().with_color(BORDER).with_width(1.0));
}

/// A `width`×`height` well centred horizontally on `cx`, top at `top`.
fn well_at(cx: f32, top: f32, width: f32, height: f32) -> Rectangle {
    Rectangle::new(Point::new(cx - width / 2.0, top), Size::new(width, height))
}

/// Copy a history ring to a contiguous oldest-first slice for the scope.
fn slice(buf: &VecDeque<f32>) -> Vec<f32> {
    buf.iter().copied().collect()
}

/// The collapse-handle hit/draw rectangle: a tab at the bottom-left, left of the
/// faceplate graphic.
fn handle_rect(bounds: Rectangle) -> Rectangle {
    let hh = HANDLE_H.min(bounds.height);
    Rectangle::new(
        Point::new(HANDLE_MARGIN, bounds.height - hh),
        Size::new(HANDLE_W, hh),
    )
}

/// Draw the collapse handle: a rounded tab with a triangle pointing `up` (to
/// collapse) or down (to expand). The whole tab is the click target.
fn draw_handle(frame: &mut Frame, bounds: Rectangle, up: bool) {
    let r = handle_rect(bounds);
    let tab = Path::new(|b| b.rounded_rectangle(r.position(), r.size(), 3.0.into()));
    frame.fill(&tab, Color::from_rgb(0.10, 0.10, 0.12));
    frame.stroke(&tab, Stroke::default().with_color(BORDER).with_width(1.0));

    // Triangle centred in the tab.
    let cx = r.x + r.width / 2.0;
    let cy = r.y + r.height / 2.0;
    let hw = 6.0;
    let hh = 4.0;
    let tri = Path::new(|b| {
        if up {
            b.move_to(Point::new(cx, cy - hh));
            b.line_to(Point::new(cx - hw, cy + hh));
            b.line_to(Point::new(cx + hw, cy + hh));
        } else {
            b.move_to(Point::new(cx, cy + hh));
            b.line_to(Point::new(cx - hw, cy - hh));
            b.line_to(Point::new(cx + hw, cy - hh));
        }
        b.close();
    });
    frame.fill(&tri, LABEL);
}
