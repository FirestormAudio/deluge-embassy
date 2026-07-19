//! Native Linux backend ops (real `libdeluge`, via `deluge-hal-linux`). OLED
//! and input are wired to the real device; everything else is unimplemented
//! for now (Phase 1b Task 2 scope — OLED + input only).
use core::sync::atomic::{AtomicBool, Ordering};

use deluge_bsp::oled::FrameBuffer;
use deluge_bsp::rgb::PadLeds;
use embassy_executor::Spawner;
use embassy_time::Instant;

/// Linux: not implemented yet — the codec audio path doesn't exist on this
/// backend.
pub(crate) async fn audio_run<F: FnMut(&mut [crate::audio::StereoFrame])>(mut f: F) -> ! {
    let _ = &mut f;
    unimplemented!("audio_run is not on the linux backend yet")
}

/// Linux: no panel init sequence needed — `libdeluge` owns the display.
pub(crate) async fn oled_init_panel() {}
/// Linux: push the framebuffer to the OLED over `libdeluge`.
///
/// The SDK's [`FrameBuffer`] is page-major SSD1309 (128×48; the top
/// `VISIBLE_TOP` rows sit behind the faceplate). The Linux `deluge-oled` fb is
/// linear/row-major 1bpp — the 43 visible rows × 16 bytes (128 px/row, MSB =
/// leftmost pixel), the format `deluge-linux-ui::OledTarget` targets. Transpose
/// the visible rows into that layout before handing it to `libdeluge`.
pub(crate) async fn oled_flush(fb: &FrameBuffer) {
    use deluge_bsp::oled::{VISIBLE_HEIGHT, VISIBLE_TOP, WIDTH};
    const LINE_BYTES: usize = WIDTH / 8; // 16
    let mut out = [0u8; VISIBLE_HEIGHT * LINE_BYTES]; // 43 × 16 = 688
    for y in 0..VISIBLE_HEIGHT {
        for x in 0..WIDTH {
            if fb.get_pixel(x, y + VISIBLE_TOP) {
                out[y * LINE_BYTES + x / 8] |= 1 << (7 - (x % 8));
            }
        }
    }
    if let Err(e) = crate::linux::dev().oled_write(&out) {
        log::warn!("oled_write failed: {e}");
    }
}

pub(crate) async fn pads_flush(_leds: &mut PadLeds) {
    unimplemented!("pads_flush is not on the linux backend yet")
}
pub(crate) async fn pads_set_brightness_interval(_interval: u8) {
    unimplemented!("pads_set_brightness_interval is not on the linux backend yet")
}

pub(crate) async fn leds_set(_id: u8, _on: bool) {
    unimplemented!("leds_set is not on the linux backend yet")
}
pub(crate) async fn leds_clear() {
    unimplemented!("leds_clear is not on the linux backend yet")
}
pub(crate) async fn leds_gold_knob(_knob: u8, _brightness: [u8; 4]) {
    unimplemented!("leds_gold_knob is not on the linux backend yet")
}

/// Linux: no PIC co-processor to wait on.
pub(crate) async fn pic_wait_ready() {}
/// Linux: no PIC co-processor to bring up.
pub(crate) fn pic_ensure_started(_spawner: Spawner) {}

pub(crate) fn sync_led_init() -> bool {
    // Initialize the LED to off; the returned bool is the initial on/off state
    // (mirrors the sim), not an availability flag. A missing LED is non-fatal —
    // `sync_led_set` logs and no-ops.
    if let Err(e) = crate::linux::dev().leds_sync(false) {
        log::warn!("sync LED unavailable: {e}");
    }
    false
}
pub(crate) fn sync_led_set(on: bool) {
    if let Err(e) = crate::linux::dev().leds_sync(on) {
        log::warn!("sync_led set failed: {e}");
    }
}
pub(crate) fn sync_led_is_set_high(state: bool) -> bool {
    state
}
pub(crate) fn sync_led_is_set_low(state: bool) -> bool {
    !state
}

pub(crate) fn cv_gate_init() {
    unimplemented!("cv_gate_init is not on the linux backend yet")
}
pub(crate) async fn cv_set(_ch: u8, _code: u16) {
    unimplemented!("cv_set is not on the linux backend yet")
}
pub(crate) fn gate_set(_ch: u8, _on: bool) {
    unimplemented!("gate_set is not on the linux backend yet")
}

pub(crate) fn midi_init() {
    unimplemented!("midi_init is not on the linux backend yet")
}
pub(crate) async fn midi_send(_data: &[u8]) {
    unimplemented!("midi_send is not on the linux backend yet")
}
pub(crate) async fn midi_recv() -> u8 {
    unimplemented!("midi_recv is not on the linux backend yet")
}
pub(crate) fn midi_try_recv() -> Option<u8> {
    unimplemented!("midi_try_recv is not on the linux backend yet")
}

pub(crate) fn clock_in_init() {
    unimplemented!("clock_in_init is not on the linux backend yet")
}
pub(crate) async fn clock_in_wait_edge() -> u64 {
    unimplemented!("clock_in_wait_edge is not on the linux backend yet")
}
pub(crate) fn clock_in_count() -> u32 {
    unimplemented!("clock_in_count is not on the linux backend yet")
}
pub(crate) fn clock_in_last_edge() -> Option<Instant> {
    unimplemented!("clock_in_last_edge is not on the linux backend yet")
}

pub(crate) fn jacks_init() {
    unimplemented!("jacks_init is not on the linux backend yet")
}
pub(crate) fn jacks_headphone() -> bool {
    unimplemented!("jacks_headphone is not on the linux backend yet")
}
pub(crate) fn jacks_line_in() -> bool {
    unimplemented!("jacks_line_in is not on the linux backend yet")
}
pub(crate) fn jacks_mic() -> bool {
    unimplemented!("jacks_mic is not on the linux backend yet")
}
pub(crate) fn jacks_line_out_left() -> bool {
    unimplemented!("jacks_line_out_left is not on the linux backend yet")
}
pub(crate) fn jacks_line_out_right() -> bool {
    unimplemented!("jacks_line_out_right is not on the linux backend yet")
}
pub(crate) fn jacks_set_speaker(_on: bool) {
    unimplemented!("jacks_set_speaker is not on the linux backend yet")
}

pub(crate) async fn sd_init_card() -> Result<(), crate::sd::SdError> {
    unimplemented!("sd_init_card is not on the linux backend yet")
}
pub(crate) fn sd_read(_name: &str, _buf: &mut [u8]) -> Result<usize, crate::sd::FatError> {
    unimplemented!("sd_read is not on the linux backend yet")
}
pub(crate) fn sd_write(_name: &str, _data: &[u8]) -> Result<(), crate::sd::FatError> {
    unimplemented!("sd_write is not on the linux backend yet")
}

static INPUT_PUMP_STARTED: AtomicBool = AtomicBool::new(false);

/// Linux: start `libdeluge`'s input delivery thread, mapping decoded events
/// into the SDK's [`crate::input::EVENTS`] queue. `spawner` is unused —
/// `libdeluge` owns the delivery thread, not the SDK's executor. Idempotent:
/// the runtime calls this unconditionally at startup, and `Deluge::input()`
/// may call it again — only the first call actually starts the thread.
pub(crate) fn input_start_pump(_spawner: Spawner) {
    if INPUT_PUMP_STARTED.swap(true, Ordering::Relaxed) {
        return;
    }
    let cb = |ev: deluge_hal_linux::Event| {
        // Kinds mirror `DELUGE_EV_*` in `deluge/input.h`: 0=pad, 1=button,
        // 2=encoder, 3=clock (not an `Event` variant — dropped, like
        // `route_pic_event`'s unmapped variants).
        let mapped = match ev.kind {
            0 => crate::input::Event::Pad {
                x: ev.x as u8,
                y: ev.y as u8,
                pressed: ev.value != 0,
            },
            1 => crate::input::Event::Button {
                id: ev.id,
                pressed: ev.value != 0,
            },
            2 => crate::input::Event::Encoder {
                index: ev.id,
                delta: ev.value as i8,
            },
            _ => return,
        };
        let _ = crate::input::EVENTS.try_send(mapped);
    };
    if let Err(e) = crate::linux::dev().input_start(cb) {
        // No input device is non-fatal (e.g. running off-device or without
        // the input nodes present) — log and keep going.
        log::warn!("linux input_start failed: {e}");
    }
}
