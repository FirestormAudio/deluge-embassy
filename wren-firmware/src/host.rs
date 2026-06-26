//! The firmware's [`deluge_wren_core::Host`] implementation: maps the bindings'
//! control-rate effects onto the SDK output tasks (CV/gate, MIDI, LEDs, OLED) and
//! the audio command ring.
//!
//! `FwHost` is a zero-sized forwarder — all real state lives in the firmware's
//! task-shared rings/atomics (`crate::CV_TARGET`, `crate::OLED_FB`, …). It is
//! registered once at boot via [`deluge_wren_core::set_host`].

use deluge_wren_core::{Cmd, Host};
use embassy_time::Instant;

/// Convert volts to a MAX5136 16-bit code (unipolar 0..~10 V, ~6552 codes/V).
fn volts_to_code(v: f32) -> u16 {
    let c = v * 6552.0;
    if c <= 0.0 {
        0
    } else if c >= 65535.0 {
        65535
    } else {
        c as u16
    }
}

/// The Deluge hardware host. Stateless: forwards to the firmware's output tasks.
pub struct FwHost;

/// The single host instance, registered at boot.
pub static mut FW_HOST: FwHost = FwHost;

impl Host for FwHost {
    fn now_ms(&mut self) -> u64 {
        Instant::now().as_millis()
    }

    fn cv_set(&mut self, ch: u8, volts: f32) {
        crate::cv_set_target(ch, volts_to_code(volts));
    }
    fn gate_set(&mut self, ch: u8, on: bool) {
        crate::gate_set_target(ch, on);
    }

    fn midi_tx(&mut self, msg: &[u8]) {
        crate::midi_tx_push(msg);
    }

    fn led(&mut self, id: u8, on: bool) {
        crate::led_cmd(id, on);
    }

    fn oled_clear(&mut self) {
        crate::oled_clear();
    }
    fn oled_text(&mut self, x: usize, y: usize, text: &[u8]) {
        crate::oled_text(x, y, text);
    }
    fn oled_pixel(&mut self, x: usize, y: usize, on: bool) {
        crate::oled_pixel(x, y, on);
    }
    fn oled_show(&mut self) {
        crate::oled_show();
    }

    fn audio_cmd(&mut self, cmd: Cmd) {
        crate::audio::submit(cmd);
    }
}
