//! Native Linux backend ops (real `libdeluge`, via `deluge-hal-linux`). OLED
//! and input are wired to the real device; everything else is unimplemented
//! for now (Phase 1b Task 2 scope — OLED + input only).
use core::sync::atomic::{AtomicBool, Ordering};

use deluge_bsp::oled::FrameBuffer;
use deluge_bsp::rgb::PadLeds;
use embassy_executor::Spawner;
use embassy_time::Instant;

/// Linux: hand the DSP closure to `libdeluge`, which runs it on its own audio
/// thread, then park.
///
/// This is the ring-free path: the app's closure executes *inside* libdeluge's
/// per-period callback, so there is no buffer or queue between the DSP and the
/// codec — only [`adapt_block`](crate::audio::adapt_block)'s single memcpy that
/// converts libdeluge's split slices to the SDK's in-place block contract.
pub(crate) async fn audio_run<F: FnMut(&mut [crate::audio::StereoFrame]) + Send + 'static>(
    mut f: F,
) -> ! {
    let mut rt_checked = false;

    let shim = move |inp: &[[f32; 2]], out: &mut [[f32; 2]]| {
        // libdeluge's audio thread self-elevates to SCHED_FIFO (audio.c), but
        // that fails *soft* to a stderr warning the appliance never shows. At
        // 128-frame periods a non-RT thread will xrun under load, so confirm it
        // once from inside the callback — this is that thread. `shim` is `move`
        // and invoked by exactly one thread (libdeluge's audio thread), so a
        // captured local latches this without needing an atomic — and unlike a
        // `static` inside this generic fn, it's correctly per-instantiation
        // (a `static` here would NOT be monomorphized per `F`, so every `F`
        // would share one latch).
        if !core::mem::replace(&mut rt_checked, true) {
            // Hard, not `debug_assert!`: `cargo deluge linux` builds `--release`,
            // so a debug assert here would compile out and this becomes a length
            // trusted from C with no check at all. If libdeluge's period
            // (`DELUGE_PERIOD`) is ever raised, SDK apps that size fixed-length
            // scratch buffers off `EXPECTED_BLOCK_FRAMES` (e.g. `additive_osc`'s
            // `MAX_BLOCK`, checked only by a `debug_assert!` of its own) would
            // silently overflow those buffers on this thread instead. Checked
            // once, on the first callback, via the same latch as the RT check
            // above — costs nothing per period after that.
            assert_eq!(
                inp.len(),
                crate::audio::EXPECTED_BLOCK_FRAMES,
                "libdeluge's period (DELUGE_PERIOD) no longer matches the SDK's \
                 assumed block length ({} frames) — got {} frames per callback; \
                 apps that size fixed buffers off this assumption will overflow \
                 them",
                crate::audio::EXPECTED_BLOCK_FRAMES,
                inp.len(),
            );

            // SAFETY: `sched_getscheduler(0)` queries the calling thread and has
            // no preconditions.
            match unsafe { libc::sched_getscheduler(0) } {
                -1 => log::warn!(
                    "sched_getscheduler query failed: {}",
                    std::io::Error::last_os_error()
                ),
                libc::SCHED_FIFO | libc::SCHED_RR => {} // real-time: as expected, nothing to report
                _ => log::warn!(
                    "audio thread is NOT real-time (SCHED_FIFO/SCHED_RR) — expect \
                     xruns under load (missing CAP_SYS_NICE?)"
                ),
            }
        }
        crate::audio::adapt_block(&mut f, inp, out);
    };

    if let Err(e) = crate::linux::dev().audio_start(shim) {
        // Unlike the sync LED and input pump, missing audio is not survivable:
        // an app that called `.audio()` needs it, and silent no-sound is harder
        // to diagnose than an abort naming the cause.
        panic!("linux audio unavailable: {e} (is the ALSA 'Deluge' card present?)");
    }

    // The DSP now lives on libdeluge's thread. This task has no further work but
    // must not return (`-> !`).
    core::future::pending().await
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

/// Bytes per pad-fb row: 18 px × 3 (24bpp RGB). libdeluge calls this the fb's
/// `line_length`; `DELUGE_PADS_BYTES` (432) is `ROWS * LINE_BYTES`.
const PAD_LINE_BYTES: usize = deluge_bsp::rgb::COLS * 3;
/// Full pad-fb frame size. Must equal libdeluge's `DELUGE_PADS_BYTES` — a
/// mismatch is rejected by `deluge_display_pads_write`, which requires the
/// exact frame length, so this would fail at run time rather than silently
/// blit garbage.
const PAD_FRAME_BYTES: usize = deluge_bsp::rgb::ROWS * PAD_LINE_BYTES;
const _: () = assert!(PAD_FRAME_BYTES == 432);

/// Linux: push the pad-LED grid to the `deluge-pad` framebuffer.
///
/// Transposes like [`oled_flush`], for the same reason: the SDK's [`PadLeds`]
/// stores `grid[col][row]` (the PIC's column-pair wire order), while
/// libdeluge's pad fb is **row-major** 24bpp RGB — 8 rows of 18 pixels,
/// `line_length` 54.
///
/// **Flips the row axis**, because the two backends anchor row 0 to opposite
/// physical rows and the SDK coordinate must mean the same pad on both:
///
/// - Device: `PadLeds::pack_pair` copies `grid[col][0..8]` straight into the
///   PIC wire order, and PIC rows run bottom-up — so SDK row 0 is the
///   **bottom** row.
/// - Linux: the `deluge-pad` fb is documented top-left origin, y-down, and the
///   driver re-flips it (`led_index = (PAD_H-1) - row`) — so fb row 0 is the
///   **top** row.
///
/// Writing `y` straight through renders every app vertically mirrored versus
/// device: caught on hardware because `additive_osc`'s highest pitch appeared
/// bottom-right instead of top-right. Nothing in the type system or the frame
/// length catches this — a mirrored frame is exactly as valid as a correct one.
///
/// Blits the whole frame every call, ignoring `PadLeds`' `last_sent` /
/// `dirty_all` cache. That cache exists to skip unchanged column-pairs on the
/// device's slow 31250-baud PIC link; here the whole frame is a 432-byte write
/// to a memory-mapped fb, so tracking dirtiness would cost more than it saves.
pub(crate) async fn pads_flush(leds: &mut PadLeds) {
    use deluge_bsp::rgb::{COLS, ROWS};
    let mut out = [0u8; PAD_FRAME_BYTES];
    let grid = leds.grid();
    for y in 0..ROWS {
        for x in 0..COLS {
            let [r, g, b] = grid[x][y];
            let o = (ROWS - 1 - y) * PAD_LINE_BYTES + x * 3;
            out[o] = r;
            out[o + 1] = g;
            out[o + 2] = b;
        }
    }
    if let Err(e) = crate::linux::dev().pads_write(&out) {
        log::warn!("pads_write failed: {e}");
    }
}

/// Linux: no-op — there is no PIC refresh interval to set.
///
/// On device this tunes the PIC's pad-LED refresh period. The Linux pad fb is
/// driven by the kernel `deluge-pad` driver, which owns its own refresh, so
/// there is nothing here to configure. A no-op rather than `unimplemented!`
/// deliberately: apps call this to tune brightness/refresh, and panicking over
/// a knob that simply does not exist on this backend would break otherwise
/// portable apps for no benefit.
pub(crate) async fn pads_set_brightness_interval(_interval: u8) {}

pub(crate) async fn leds_set(id: u8, on: bool) {
    if let Err(e) = crate::linux::dev().leds_indicator(id as i32, on) {
        log::warn!("leds_indicator({id}) failed: {e}");
    }
}
pub(crate) async fn leds_clear() {
    for id in 0..crate::leds::Leds::NUM_INDICATOR_LEDS {
        leds_set(id, false).await;
    }
}
/// Linux: set a gold-knob column's four indicator LEDs.
///
/// `brightness[i]` maps to libdeluge's `deluge_leds_gold(col, i, brightness)`
/// (col 0..1, i 0..3, brightness 0..255) — the SDK's per-knob array is exactly
/// that column's four LEDs, so this is a straight fan-out.
pub(crate) async fn leds_gold_knob(knob: u8, brightness: [u8; 4]) {
    for (i, b) in brightness.iter().enumerate() {
        if let Err(e) = crate::linux::dev().leds_gold(knob as i32, i as i32, *b as i32) {
            log::warn!("leds_gold(knob {knob}, {i}) failed: {e}");
            break;
        }
    }
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
                // Flip the row axis, exactly as `pads_flush` does and for the
                // same reason: libdeluge reports pads **top-origin** (the
                // kernel's `pad_id_to_coord` documents "PIC rows run bottom-up;
                // report top-left"), while the SDK's convention — fixed by the
                // device backend, where `pack_pair` feeds the PIC's bottom-up
                // rows straight through — is y=0 at the **bottom**.
                //
                // Flipping only one of the two paths is worse than flipping
                // neither: input and output then disagree by a mirror, so a
                // pressed pad lights up its reflection. That is how this was
                // found on hardware, after `pads_flush` was fixed alone.
                y: (deluge_bsp::rgb::ROWS as isize - 1 - ev.y as isize)
                    .clamp(0, deluge_bsp::rgb::ROWS as isize - 1)
                    as u8,
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
