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

// Device-side `Wavetable.from(...)` now builds + binds a real table inline:
// `upload_table` below forwards to `crate::audio::upload_table`, which builds
// the mip pyramid directly into a pool region of the audio engine and returns
// its handle. This runs synchronously inside the Wren foreign call on the one
// cooperative executor and is real-time-safe under that model — see
// `audio.rs`'s `## Concurrency` docs and the device-upload spec for why the
// sub-millisecond IFFT build fits the audio write-ahead lead without stalling
// `audio_task` or aliasing its engine borrow.
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

    fn upload_table(&mut self, base: &[f32]) -> Option<deluge_audio_graph::PoolHandle> {
        crate::audio::upload_table(base)
    }

    fn upload_table_2d(
        &mut self,
        nframes: usize,
        fill_frame: &mut dyn FnMut(usize, &mut [f32]),
    ) -> Option<deluge_audio_graph::PoolHandle> {
        crate::audio::upload_table_2d(nframes, fill_frame)
    }

    fn pool_set(&mut self, h: deluge_audio_graph::PoolHandle, index: usize, value: f32) {
        crate::audio::pool_set(h, index, value);
    }

    fn alloc_buffer(&mut self, len: usize) -> Option<deluge_audio_graph::PoolHandle> {
        crate::audio::alloc_buffer(len)
    }

    // Host (sim): hand the node+ring+path off to the host-only WAV prefetch
    // (`crate::stream::stream_task`), which loads/decodes the file and fills
    // the ring as playback advances — see `stream.rs`'s module docs. Device:
    // no filesystem/prefetch yet (device streaming is Sa-3b slice 5); the
    // node simply stays silent until then.
    fn stream_register(
        &mut self,
        node: deluge_audio_graph::NodeId,
        handle: deluge_audio_graph::PoolHandle,
        path: &str,
    ) {
        #[cfg(not(target_os = "none"))]
        crate::stream::register(node, handle, path);
        #[cfg(target_os = "none")]
        let _ = (node, handle, path); // device prefetch is slice 5
    }
}
