//! Host-simulator backend ops (the deluge-sim-link SharedPanel). Bodies moved
//! verbatim from the capability modules' `#[cfg(not(target_os = "none"))]` arms.
use deluge_bsp::oled::FrameBuffer;
use deluge_bsp::rgb::{COLS, PadLeds, ROWS};

pub(crate) async fn oled_init_panel() {}
pub(crate) async fn oled_flush(fb: &FrameBuffer) {
    crate::host::panel().set_display(fb.as_bytes());
}

pub(crate) async fn pads_flush(leds: &mut PadLeds) {
    let grid = leds.grid();
    let mut buf = [0u8; deluge_sim_link::ALL_PADS_BYTES];
    for col in 0..COLS {
        for row in 0..ROWS {
            let o = (col * ROWS + row) * 3;
            buf[o..o + 3].copy_from_slice(&grid[col][row]);
        }
    }
    crate::host::panel().set_all_pads(&buf);
}
pub(crate) async fn pads_set_brightness_interval(interval: u8) {
    let _ = interval;
}

pub(crate) async fn leds_set(id: u8, on: bool) {
    crate::host::panel().set_led(id as usize, on);
}
pub(crate) async fn leds_clear() {
    crate::host::panel().clear_all_leds();
}
pub(crate) async fn leds_gold_knob(knob: u8, brightness: [u8; 4]) {
    crate::host::panel().set_knob_indicator(knob as usize, brightness);
}

pub(crate) fn sync_led_init() -> bool {
    false
}
pub(crate) fn sync_led_set(on: bool) {
    crate::host::panel().set_synced_led(on);
}
pub(crate) fn sync_led_is_set_high(state: bool) -> bool {
    state
}
pub(crate) fn sync_led_is_set_low(state: bool) -> bool {
    !state
}

pub(crate) fn cv_gate_init() {}
pub(crate) async fn cv_set(ch: u8, code: u16) {
    crate::host::panel().set_cv(ch as usize, code);
}
pub(crate) fn gate_set(ch: u8, on: bool) {
    crate::host::panel().set_gate(ch as usize, on);
}

pub(crate) fn midi_init() {}
pub(crate) async fn midi_send(data: &[u8]) {
    // Hand the bytes to the simulator panel (lights the MIDI OUT activity
    // indicator; the GUI can forward them to a host port).
    crate::host::panel().push_midi_out(data);
}
pub(crate) async fn midi_recv() -> u8 {
    // Drain bytes the simulator's MIDI bridge pushed into the panel, polling
    // at ~1 ms when the queue is empty (DIN MIDI is slow, so the latency is
    // inaudible).
    loop {
        if let Some(b) = crate::host::panel().pop_midi_in() {
            return b;
        }
        embassy_time::Timer::after_millis(1).await;
    }
}
pub(crate) fn midi_try_recv() -> Option<u8> {
    crate::host::panel().pop_midi_in()
}
