//! The simulator application shell (iced): render the panel from inbound
//! illumination frames, forward input as [`FromDeluge`]. The Deluge "brain" on the
//! other end of the [`PanelLink`] owns all UI logic — this is a faithful front panel,
//! not a reimplementation. (Trimmed from spark's `simulator_app`: the engine /
//! DelugeUI / settings / remote wiring is gone, replaced by the protocol link.)

use crate::display::SimulatorDisplay;
use crate::hardware::{HardwareButton, HardwareEncoder, HardwareLED};
use crate::hardware_state::DelugeHardware;
use crate::hardware_link::HardwareMirror;
use crate::link::{self, LinkKind};
use crate::pad_grid::PadGrid;
#[cfg(feature = "rack")]
use crate::rack::InstrumentRack;
use crate::renderer::DynamicElementsRenderer;
use crate::rgb::RGB;

use deluge_protocol::{FromDeluge, ToDeluge};
use deluge_sim_link::audio::HeapCons;
#[cfg(feature = "rack")]
use deluge_sim_link::audio::Consumer;

use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use iced::{
    Color, Element, Length, Task, Theme,
    widget::{Canvas, Image, Stack, container, image},
};
#[cfg(feature = "rack")]
use iced::widget::column;

/// Window width (px): the faceplate's half-SVG width.
pub(crate) const WINDOW_WIDTH: f32 = 1089.0;
/// Faceplate height (px): half the SVG's 1482.
pub(crate) const FACEPLATE_HEIGHT: f32 = 741.0;
/// Height (px) of the CV/gate rack strip above the faceplate.
pub(crate) const RACK_HEIGHT: f32 = 92.0;
/// Total window height: the faceplate, plus the rack strip when the `rack`
/// feature is enabled. With the feature off the top panel is compiled out and
/// the window is faceplate-only.
pub(crate) const WINDOW_HEIGHT: f32 =
    FACEPLATE_HEIGHT + if cfg!(feature = "rack") { RACK_HEIGHT } else { 0.0 };

/// Messages produced by the canvas (input) and the periodic tick.
#[derive(Debug, Clone)]
pub enum SimulatorMessage {
    /// 60 Hz tick: drain inbound illumination frames.
    Tick,
    PadPressed { col: usize, row: usize },
    PadReleased { col: usize, row: usize },
    ButtonPressed(HardwareButton),
    ButtonReleased(HardwareButton),
    EncoderRotated { encoder: HardwareEncoder, delta: i32 },
    EncoderPressed(HardwareEncoder),
    EncoderReleased(HardwareEncoder),
    ToggleStickyKeys,
    /// Toggle the CV/gate rack strip between meters and scope traces.
    #[cfg(feature = "rack")]
    ToggleRackScopes,
    /// Collapse/expand the rack strip (and resize the window to match).
    #[cfg(feature = "rack")]
    ToggleRackCollapsed,
}

pub struct DelugeSimulator {
    /// Persistent canvas renderer — owns display, pad grid, and hardware state.
    renderer: DynamicElementsRenderer,
    /// CV/gate visualiser to the right of the faceplate.
    #[cfg(feature = "rack")]
    rack: InstrumentRack,
    /// Pre-rasterised SVG faceplate (drawn behind the canvas).
    svg_background: Option<image::Handle>,
    /// How the panel is driven (protocol brain, in-process SDK app, or passive).
    link: LinkKind,
    /// A physical Deluge attached over serial as an additional control surface,
    /// mirroring the in-process brain (`--hardware`). `None` when not attached;
    /// dropped if the device disconnects mid-session.
    hardware: Option<HardwareMirror>,
    /// Stereo tap of the output audio (in-process link only) feeding the rack's
    /// L/R audio oscilloscopes. Drained each frame.
    #[cfg(feature = "rack")]
    audio_monitor: Option<HeapCons<[f32; 2]>>,
    /// Master output volume, driven by the faceplate Volume knob (0.0–1.0).
    volume: crate::audio::Volume,
}

/// Percentage points the Volume knob moves per scroll detent.
const VOLUME_STEP: i32 = 4;

// Top-panel (rack) integration seam. The CV/gate/audio "top panel" is an
// optional `rack` feature: when enabled these forward to the [`InstrumentRack`];
// when disabled the panel is compiled out and these are no-ops, so the call
// sites in `update`/`apply`/`apply_shared_panel` stay feature-agnostic.
#[cfg(feature = "rack")]
impl DelugeSimulator {
    fn rack_set_cv(&mut self, cv: [u16; 4]) {
        self.rack.set_cv(cv);
    }
    fn rack_set_gate(&mut self, gate: [bool; 4]) {
        self.rack.set_gate(gate);
    }
    fn rack_set_cv_channel(&mut self, ch: usize, code: u16) {
        self.rack.set_cv_channel(ch, code);
    }
    fn rack_set_gate_channel(&mut self, ch: usize, on: bool) {
        self.rack.set_gate_channel(ch, on);
    }
    fn rack_flash_midi_in(&mut self) {
        self.rack.flash_midi_in();
    }
    fn rack_flash_midi_out(&mut self) {
        self.rack.flash_midi_out();
    }
    fn rack_sample(&mut self) {
        self.rack.sample();
    }
    fn drain_audio_monitor(&mut self) {
        if let Some(mon) = self.audio_monitor.as_mut() {
            while let Some([l, r]) = mon.try_pop() {
                self.rack.push_audio(l, r);
            }
        }
    }
}

#[cfg(not(feature = "rack"))]
impl DelugeSimulator {
    fn rack_set_cv(&mut self, _cv: [u16; 4]) {}
    fn rack_set_gate(&mut self, _gate: [bool; 4]) {}
    fn rack_set_cv_channel(&mut self, _ch: usize, _code: u16) {}
    fn rack_set_gate_channel(&mut self, _ch: usize, _on: bool) {}
    fn rack_flash_midi_in(&mut self) {}
    fn rack_flash_midi_out(&mut self) {}
    fn rack_sample(&mut self) {}
    fn drain_audio_monitor(&mut self) {}
}

impl DelugeSimulator {
    pub fn new(
        link: LinkKind,
        svg_background: Option<image::Handle>,
        audio_monitor: Option<HeapCons<[f32; 2]>>,
        volume: crate::audio::Volume,
        hardware: Option<HardwareMirror>,
    ) -> Self {
        // With the `rack` feature off there is no top panel to feed, so the
        // output-audio tap is unused.
        #[cfg(not(feature = "rack"))]
        let _ = audio_monitor;
        let mut renderer = DynamicElementsRenderer::new(
            SimulatorDisplay::new(),
            PadGrid::new(),
            DelugeHardware::new(),
        );
        // The Volume knob is a pot (0–100 %); start at full to match the default
        // master gain (1.0).
        renderer.hardware.set_encoder_value(HardwareEncoder::Volume, 100);
        Self {
            renderer,
            #[cfg(feature = "rack")]
            rack: InstrumentRack::new(),
            svg_background,
            link,
            hardware,
            #[cfg(feature = "rack")]
            audio_monitor,
            volume,
        }
    }

    pub fn update(&mut self, message: SimulatorMessage) -> Task<SimulatorMessage> {
        match message {
            SimulatorMessage::Tick => {
                self.drain_inbound();
                self.service_hardware();
                // Drain the audio monitor (mono output tap) into the rack's audio
                // scope history before sampling CV/gate for the same frame, then
                // append a rack history point so the scopes scroll evenly
                // regardless of how often the app writes CV/gate. Both are no-ops
                // when the `rack` feature is disabled.
                self.drain_audio_monitor();
                self.rack_sample();
            }

            SimulatorMessage::PadPressed { col, row } => {
                self.renderer.grid.set_pressed(col, row, true);
                self.renderer.pad_cache.clear();
                self.send(FromDeluge::PadPressed { col: col as u8, row: row as u8 });
            }
            SimulatorMessage::PadReleased { col, row } => {
                self.renderer.grid.set_pressed(col, row, false);
                self.renderer.pad_cache.clear();
                self.send(FromDeluge::PadReleased { col: col as u8, row: row as u8 });
            }

            SimulatorMessage::ButtonPressed(b) => {
                self.renderer.pressed_buttons.insert(b);
                self.renderer.controls_cache.clear();
                self.send(FromDeluge::ButtonPressed { id: link::button_to_id(b) });
            }
            SimulatorMessage::ButtonReleased(b) => {
                self.renderer.pressed_buttons.remove(&b);
                self.renderer.controls_cache.clear();
                self.send(FromDeluge::ButtonReleased { id: link::button_to_id(b) });
            }

            SimulatorMessage::EncoderRotated { encoder, delta } => {
                if encoder == HardwareEncoder::Volume {
                    // The Volume knob is the sim's master output level, not an app
                    // control: a 0–100 % pot driving the audio output gain.
                    let cur = self.renderer.hardware.get_encoder_value(encoder);
                    let pct = (cur + delta * VOLUME_STEP).clamp(0, 100);
                    self.renderer.hardware.set_encoder_value(encoder, pct);
                    self.volume.store(
                        (pct as f32 / 100.0).to_bits(),
                        std::sync::atomic::Ordering::Relaxed,
                    );
                    self.renderer.controls_cache.clear();
                    return Task::none();
                }
                self.renderer.hardware.rotate_encoder(encoder, delta);
                self.renderer.controls_cache.clear();
                if let Some(id) = link::encoder_to_id(encoder) {
                    self.send(FromDeluge::EncoderRotated {
                        id,
                        delta: delta.clamp(-128, 127) as i8,
                    });
                }
            }
            SimulatorMessage::EncoderPressed(e) => {
                if let Some(id) = link::encoder_push_id(e) {
                    self.send(FromDeluge::ButtonPressed { id });
                }
            }
            SimulatorMessage::EncoderReleased(e) => {
                if let Some(id) = link::encoder_push_id(e) {
                    self.send(FromDeluge::ButtonReleased { id });
                }
            }

            SimulatorMessage::ToggleStickyKeys => {
                self.renderer.sticky_keys_enabled = !self.renderer.sticky_keys_enabled;
                self.renderer.controls_cache.clear();
            }

            #[cfg(feature = "rack")]
            SimulatorMessage::ToggleRackScopes => self.rack.toggle_expanded(),

            // The strip keeps a fixed height; collapsing just hides its contents
            // (down to the handle), so the faceplate never moves. We can't shrink
            // the window to reclaim the space — Wayland blocks client resize and a
            // tiling compositor would tile the window if it were made resizable.
            #[cfg(feature = "rack")]
            SimulatorMessage::ToggleRackCollapsed => self.rack.toggle_collapsed(),
        }
        Task::none()
    }

    /// Service the physical control surface (if attached): feed its input into
    /// the brain like a GUI click, and push the brain's illumination back out so
    /// the real OLED/pads/LEDs/CV-gate track the screen. Drops the mirror if the
    /// device has disconnected.
    fn service_hardware(&mut self) {
        if self.hardware.is_none() {
            return;
        }
        if !self.hardware.as_ref().unwrap().is_alive() {
            eprintln!("deluge-simulator: hardware control surface disconnected");
            self.hardware = None;
            return;
        }
        // Drain input and push illumination while borrowing the mirror, then drop
        // the borrow before forwarding input (which borrows `self` for `send`).
        let input = {
            let hw = self.hardware.as_mut().unwrap();
            let input = hw.poll_input();
            hw.retry_handshake();
            hw.mirror();
            input
        };
        for msg in input {
            self.send(msg);
        }
    }

    fn send(&self, msg: FromDeluge) {
        match &self.link {
            LinkKind::Protocol(link) => {
                let _ = link.outbound.send(msg);
            }
            LinkKind::InProcess(link) => link.send_input(msg),
            LinkKind::None => {}
        }
    }

    /// Drain inbound illumination and apply it to the panel.
    fn drain_inbound(&mut self) {
        match &self.link {
            LinkKind::Protocol(_) => self.drain_protocol(),
            LinkKind::InProcess(_) => self.apply_shared_panel(),
            LinkKind::None => {}
        }
    }

    /// Protocol link: decode framed `ToDeluge` messages and apply them.
    fn drain_protocol(&mut self) {
        // Collect first so we don't hold an immutable borrow of `self.link` while
        // applying (which borrows `self` mutably).
        let mut frames = Vec::new();
        if let LinkKind::Protocol(link) = &self.link {
            while let Ok(frame) = link.inbound.try_recv() {
                frames.push(frame);
            }
        }
        for (type_byte, data) in frames {
            if let Some(msg) = ToDeluge::decode(type_byte, &data) {
                self.apply(msg);
            }
        }
    }

    /// In-process link: mirror the shared panel into the renderer, repainting only
    /// the categories whose change-generation advanced since the last frame.
    fn apply_shared_panel(&mut self) {
        // Snapshot what changed, dropping the `&self.link` borrow before mutating.
        let LinkKind::InProcess(link) = &self.link else {
            return;
        };
        let panel = link.panel.clone();
        let (seen_d, seen_p, seen_c) = (link.seen_display, link.seen_pads, link.seen_controls);
        let (seen_cv, seen_gate) = (link.seen_cv, link.seen_gate);
        let (gen_d, gen_p, gen_c) = (panel.display_gen(), panel.pads_gen(), panel.controls_gen());
        let (gen_cv, gen_gate) = (panel.cv_gen(), panel.gate_gen());

        let display = (gen_d != seen_d).then(|| panel.display_snapshot());
        let pads = (gen_p != seen_p).then(|| panel.pads_snapshot());
        let controls = (gen_c != seen_c)
            .then(|| (panel.leds_snapshot(), panel.knobs_snapshot(), panel.synced_led()));
        let cv = (gen_cv != seen_cv).then(|| panel.cv_snapshot());
        let gate = (gen_gate != seen_gate).then(|| panel.gate_snapshot());
        let (seen_min, seen_mout) = (link.seen_midi_in, link.seen_midi_out);
        let (gen_min, gen_mout) = (panel.midi_in_gen(), panel.midi_out_gen());
        let midi_in_active = gen_min != seen_min;
        let midi_out_active = gen_mout != seen_mout;

        if let Some(buf) = display {
            self.update_display_from_buffer(&buf);
        }
        if let Some(grid) = pads {
            for (col, rows) in grid.iter().enumerate() {
                for (row, rgb) in rows.iter().enumerate() {
                    self.renderer.grid.set(col, row, RGB::new(rgb[0], rgb[1], rgb[2]));
                }
            }
            self.renderer.pad_cache.clear();
        }
        if let Some((leds, knobs, synced)) = controls {
            for (index, on) in leds.iter().enumerate() {
                if let Some(led) = link::led_index_to_led(index as u8) {
                    self.renderer.hardware.set_led_state(led, *on);
                }
            }
            self.renderer.hardware.set_led_state(HardwareLED::Synced, synced);
            self.set_knob_indicator(0, knobs[0]);
            self.set_knob_indicator(1, knobs[1]);
            self.renderer.controls_cache.clear();
        }
        if let Some(cv) = cv {
            self.rack_set_cv(cv);
        }
        if let Some(gate) = gate {
            self.rack_set_gate(gate);
        }
        if midi_in_active {
            self.rack_flash_midi_in();
        }
        if midi_out_active {
            self.rack_flash_midi_out();
        }

        // Record the generations we've now applied.
        if let LinkKind::InProcess(link) = &mut self.link {
            link.seen_display = gen_d;
            link.seen_pads = gen_p;
            link.seen_controls = gen_c;
            link.seen_cv = gen_cv;
            link.seen_gate = gen_gate;
            link.seen_midi_in = gen_min;
            link.seen_midi_out = gen_mout;
        }
    }

    fn apply(&mut self, msg: ToDeluge) {
        match msg {
            ToDeluge::UpdateDisplay(buf) => self.update_display_from_buffer(buf),
            ToDeluge::ClearDisplay => {
                self.renderer.display.clear(BinaryColor::Off).unwrap();
                self.renderer.oled_cache.clear();
            }
            ToDeluge::SetPadRgb { col, row, rgb } => {
                self.renderer
                    .grid
                    .set(col as usize, row as usize, RGB::new(rgb[0], rgb[1], rgb[2]));
                self.renderer.pad_cache.clear();
            }
            ToDeluge::SetAllPads(buf) => {
                for col in 0..18usize {
                    for row in 0..8usize {
                        let o = (col * 8 + row) * 3;
                        self.renderer
                            .grid
                            .set(col, row, RGB::new(buf[o], buf[o + 1], buf[o + 2]));
                    }
                }
                self.renderer.pad_cache.clear();
            }
            ToDeluge::ClearAllPads => {
                self.renderer.grid.clear();
                self.renderer.pad_cache.clear();
            }
            ToDeluge::SetLed { index, on } => {
                if let Some(led) = link::led_index_to_led(index) {
                    self.renderer.hardware.set_led_state(led, on);
                    self.renderer.controls_cache.clear();
                }
            }
            ToDeluge::SetSyncedLed(on) => {
                self.renderer.hardware.set_led_state(HardwareLED::Synced, on);
                self.renderer.controls_cache.clear();
            }
            ToDeluge::SetKnobIndicator { which, levels } => self.set_knob_indicator(which, levels),
            ToDeluge::ClearAllLeds => {
                for led in HardwareLED::all_leds() {
                    self.renderer.hardware.set_led_state(led, false);
                }
                self.renderer.controls_cache.clear();
            }
            ToDeluge::SetCv { channel, value } => {
                self.rack_set_cv_channel(channel as usize, value);
            }
            ToDeluge::SetGate { channel, on } => {
                self.rack_set_gate_channel(channel as usize, on);
            }
            // No panel representation (audio/brightness/handshake).
            ToDeluge::SetBrightness(_) | ToDeluge::GetVersion | ToDeluge::Ping => {}
        }
    }

    /// Light the 4 segment LEDs of a gold knob ring from inbound levels (`which`:
    /// 0 = lower, 1 = upper). A segment is on when its level is non-zero.
    fn set_knob_indicator(&mut self, which: u8, levels: [u8; 4]) {
        use HardwareLED::*;
        let segs = if which == 0 {
            [LowerGoldIndicator1, LowerGoldIndicator2, LowerGoldIndicator3, LowerGoldIndicator4]
        } else {
            [UpperGoldIndicator1, UpperGoldIndicator2, UpperGoldIndicator3, UpperGoldIndicator4]
        };
        for (i, led) in segs.into_iter().enumerate() {
            self.renderer.hardware.set_led_state(led, levels[i] > 0);
        }
        self.renderer.controls_cache.clear();
    }

    /// Unpack a 768-byte SSD1309 page-major framebuffer into the canvas display.
    /// `buf[page*128 + col]`, bit `b` = panel row `page*8 + b`; the 43 visible rows
    /// start at panel row 5 (the bezel rows above are skipped).
    fn update_display_from_buffer(&mut self, buffer: &[u8]) {
        const HARDWARE_TOPMOST: usize = 5;
        const OLED_HEIGHT: usize = 43;
        if buffer.len() != 768 {
            return;
        }
        self.renderer.display.clear(BinaryColor::Off).unwrap();
        for page in 0..6usize {
            for col in 0..128usize {
                let byte = buffer[page * 128 + col];
                for bit in 0..8usize {
                    if byte & (1 << bit) != 0 {
                        let panel_row = page * 8 + bit;
                        if panel_row < HARDWARE_TOPMOST {
                            continue;
                        }
                        let ui_row = panel_row - HARDWARE_TOPMOST;
                        if ui_row < OLED_HEIGHT {
                            self.renderer.display.set_pixel(col, ui_row, true);
                        }
                    }
                }
            }
        }
        self.renderer.oled_cache.clear();
    }

    pub fn view(&self) -> Element<'_, SimulatorMessage> {
        let canvas = Canvas::new(&self.renderer).width(Length::Fill).height(Length::Fill);
        let content = if let Some(ref svg_handle) = self.svg_background {
            let background: Image = image(svg_handle.clone())
                .content_fit(iced::ContentFit::Contain)
                .filter_method(image::FilterMethod::Linear)
                .width(Length::Fill)
                .height(Length::Fill);
            container(
                Stack::new()
                    .push(background)
                    .push(canvas)
                    .width(Length::Fill)
                    .height(Length::Fill),
            )
        } else {
            container(canvas)
        };
        let faceplate = content
            .width(Length::Fill)
            .height(Length::Fill)
            .center_x(Length::Fill)
            .center_y(Length::Fill)
            .style(|_theme| container::Style {
                background: Some(Color::from_rgb(0.0, 0.0, 0.0).into()),
                border: iced::Border {
                    color: Color::from_rgb(0.2, 0.2, 0.2),
                    width: 4.0,
                    radius: 8.0.into(),
                },
                ..Default::default()
            });

        // The CV/gate rack is a separate strip above the faceplate (over the
        // back-panel jacks), not overlapping the deluge graphic. Collapsing
        // shrinks the strip to its handle bar and resizes the window to match.
        // With the `rack` feature off the strip is compiled out entirely and the
        // window shows only the faceplate.
        #[cfg(feature = "rack")]
        {
            let rack = Canvas::new(&self.rack)
                .width(Length::Fill)
                .height(Length::Fixed(RACK_HEIGHT));
            column![rack, faceplate].width(Length::Fill).height(Length::Fill).into()
        }
        #[cfg(not(feature = "rack"))]
        {
            faceplate.into()
        }
    }

    pub fn theme(&self) -> Theme {
        Theme::Dark
    }

    pub fn title(&self) -> String {
        String::from("Deluge Emulator")
    }

    pub fn subscription(&self) -> iced::Subscription<SimulatorMessage> {
        iced::time::every(std::time::Duration::from_millis(16)).map(|_| SimulatorMessage::Tick)
    }
}
