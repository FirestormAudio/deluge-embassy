//! Native Linux backend ops (real `libdeluge`, via `deluge-hal-linux`).
//!
//! Implemented: audio, OLED, pads, indicator/gold/sync LEDs, pad brightness,
//! input, jacks, CV/gate, DIN MIDI, trigger-clock input.
//!
//! Still `unimplemented!()`: `sd_*`. Not an oversight — the SDK's `Sd` is a
//! sector-backed FAT abstraction, while on this backend the kernel has already
//! mounted the card (the appliance's `/init` does `mount -t vfat` on `/sd`), so
//! raw sector access would mean fighting the VFS for a device it owns. What the
//! Linux backend should expose instead is an open design question, not a
//! missing function.
//!
//! Two recurring shapes worth knowing before adding to this file:
//!
//! - **Coordinate origins differ.** libdeluge/the kernel are top-origin for the
//!   pad grid; the SDK is bottom-origin (set by the device backend). Anything
//!   touching pad coordinates must flip, on *both* the input and output paths —
//!   see `pads_flush` and the input pump.
//! - **Errors log rather than propagate.** Most `plat` ops are infallible in the
//!   SDK's signatures, because the device backend cannot fail at them (a GPIO
//!   write has no error path). Here they can, so they `log::warn!` and continue
//!   with the least-surprising fallback rather than panicking an app over, say,
//!   an absent LED.
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

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

/// Linux: set the PIC's pad-LED refresh interval, matching the device backend.
///
/// Goes to the same place as `plat::device`'s `pic::set_refresh_time`: PIC
/// command 19, interval in ms, **lower is brighter** (it is the refresh period,
/// so a shorter period is a higher duty cycle). Here it travels via libdeluge to
/// the `deluge-pic` driver's `refresh_time` sysfs attribute rather than over the
/// SDK's own PIC transport, but it is the identical command and range.
///
/// Deliberately NOT wired to `deluge-pad`'s `fb_deferred_io` delay, which is the
/// other plausible reading of "refresh": that controls how often Linux flushes a
/// frame, a different knob with a different audible/visible effect. Wiring it
/// there would appear to work while silently diverging from the device backend.
///
/// The kernel rejects values above 25; the SDK's `interval` is a `u8`, so clamp
/// rather than pass a value that would be refused — matching the device backend,
/// which likewise cannot express an out-of-range interval.
pub(crate) async fn pads_set_brightness_interval(interval: u8) {
    let clamped = interval.min(25);
    if let Err(e) = crate::linux::dev().pads_set_refresh(clamped as i32) {
        log::warn!("pads_set_refresh({clamped}) failed: {e}");
    }
}

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

/// Linux: nothing to configure — the kernel owns the CV DAC and gate GPIOs, and
/// libdeluge opened them when the handle was created. (On device this runs the
/// DAC's ~10 ms linearity init over RSPI0.)
pub(crate) fn cv_gate_init() {}

/// Linux: set a CV channel from a raw DAC code.
///
/// `code` is the same 16-bit DAC code the device backend writes, so an app
/// computing codes itself gets identical output on both backends. libdeluge also
/// offers a volts-based setter; deliberately not used here, because the SDK's
/// `cv_set` contract is raw codes and routing it through a float conversion
/// would quantise differently from device.
pub(crate) async fn cv_set(ch: u8, code: u16) {
    if let Err(e) = crate::linux::dev().cv_set_raw(ch as i32, code) {
        log::warn!("cv_set(ch {ch}) failed: {e}");
    }
}
pub(crate) fn gate_set(ch: u8, on: bool) {
    if let Err(e) = crate::linux::dev().gate_set(ch as i32, on) {
        log::warn!("gate_set(ch {ch}) failed: {e}");
    }
}

/// Received DIN-MIDI bytes not yet handed to the app.
///
/// libdeluge's `midi_read` is a non-blocking bulk read over ALSA rawmidi, while
/// the SDK's receive API is byte-at-a-time. Reading one byte per call would
/// issue a syscall per byte at 31250 baud; this drains what the kernel has into
/// a small buffer and serves bytes from it.
///
/// A `std::sync::Mutex` rather than a bare `static mut`: the app's tasks all run
/// on one thread (SDK handles are `!Send`), but `midi_send`/`midi_recv` are
/// plain functions with no such marker, so nothing structurally prevents a
/// second thread from calling them.
static MIDI_RX: std::sync::Mutex<std::collections::VecDeque<u8>> =
    std::sync::Mutex::new(std::collections::VecDeque::new());

/// How long to sleep between polls in [`midi_recv`]. libdeluge exposes no
/// pollable MIDI fd, so waiting means polling. One DIN byte is ~320 us at 31250
/// baud, so 500 us keeps worst-case added latency below a byte time while
/// costing ~2000 wakeups/s when a task is parked on MIDI input.
const MIDI_POLL_US: u64 = 500;

/// Linux: nothing to configure — libdeluge opened rawmidi at handle creation.
pub(crate) fn midi_init() {}

pub(crate) async fn midi_send(data: &[u8]) {
    match crate::linux::dev().midi_write(data) {
        Ok(n) if n < data.len() => {
            // Short write: rawmidi is opened non-blocking, so a full output
            // buffer truncates rather than waiting. Report it — silently
            // dropping the tail of a SysEx would be very hard to diagnose.
            log::warn!("midi_send wrote {n}/{} bytes (output buffer full)", data.len());
        }
        Ok(_) => {}
        Err(e) => log::warn!("midi_send failed: {e}"),
    }
}

pub(crate) fn midi_try_recv() -> Option<u8> {
    let mut q = MIDI_RX.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(b) = q.pop_front() {
        return Some(b);
    }
    let mut buf = [0u8; 64];
    match crate::linux::dev().midi_read(&mut buf) {
        Ok(0) => None,
        Ok(n) => {
            q.extend(&buf[..n]);
            q.pop_front()
        }
        Err(e) => {
            log::warn!("midi_read failed: {e}");
            None
        }
    }
}

/// Linux: wait for one DIN-MIDI byte.
///
/// Polls, because libdeluge exposes no MIDI fd to await on. This is the one
/// place the Linux backend is meaningfully worse than device, where a byte
/// arrives by DMA + interrupt and the waker fires immediately: here a byte can
/// sit up to [`MIDI_POLL_US`] before the task observes it.
pub(crate) async fn midi_recv() -> u8 {
    loop {
        if let Some(b) = midi_try_recv() {
            return b;
        }
        embassy_time::Timer::after(embassy_time::Duration::from_micros(MIDI_POLL_US)).await;
    }
}

// ── Trigger-clock input ───────────────────────────────────────────────────────
//
// The kernel exposes the clock-in pin as an input device ("Deluge Clock In"),
// so edges arrive through libdeluge's input stream as `DELUGE_EV_CLOCK` (kind 3)
// rather than needing an API of their own. `input_start_pump` routes them here
// instead of dropping them. Mirrors the device backend's `trigger_clock`
// statics, so the SDK-facing semantics are identical — only the edge source
// differs (evdev callback vs IRQ handler).

static CLOCK_COUNT: AtomicU32 = AtomicU32::new(0);
/// Embassy tick of the most recent edge; 0 = none seen yet (matching the device
/// backend's sentinel, where tick 0 is likewise treated as "no edge").
static CLOCK_LAST_TICKS: AtomicU64 = AtomicU64::new(0);
static CLOCK_WAKER: embassy_sync::waitqueue::AtomicWaker =
    embassy_sync::waitqueue::AtomicWaker::new();

/// Record one clock edge. Called from libdeluge's input thread.
fn clock_note_edge() {
    CLOCK_LAST_TICKS.store(Instant::now().as_ticks(), Ordering::Relaxed);
    CLOCK_COUNT.fetch_add(1, Ordering::Relaxed);
    CLOCK_WAKER.wake();
}

/// Linux: nothing to arm — the kernel owns the IRQ and the input pump is already
/// running (the runtime starts it at boot, before any app code).
pub(crate) fn clock_in_init() {}

pub(crate) async fn clock_in_wait_edge() -> u64 {
    // Sample the count first and compare rather than waiting for a "next" flag:
    // an edge landing between this load and the first poll must still satisfy
    // the wait, otherwise a fast clock can be missed entirely.
    let start = CLOCK_COUNT.load(Ordering::Relaxed);
    core::future::poll_fn(|cx| {
        CLOCK_WAKER.register(cx.waker());
        if CLOCK_COUNT.load(Ordering::Relaxed) != start {
            core::task::Poll::Ready(())
        } else {
            core::task::Poll::Pending
        }
    })
    .await;
    CLOCK_LAST_TICKS.load(Ordering::Relaxed)
}
pub(crate) fn clock_in_count() -> u32 {
    CLOCK_COUNT.load(Ordering::Relaxed)
}
pub(crate) fn clock_in_last_edge() -> Option<Instant> {
    match CLOCK_LAST_TICKS.load(Ordering::Relaxed) {
        0 => None,
        t => Some(Instant::from_ticks(t)),
    }
}

// `deluge_jack` discriminants (deluge/jacks.h), ordered to match
// `deluge_bsp::jacks::Jack` so both backends enumerate identically.
const JACK_HEADPHONE: u32 = 0;
const JACK_LINE_IN: u32 = 1;
const JACK_MIC: u32 = 2;
const JACK_LINE_OUT_L: u32 = 3;
const JACK_LINE_OUT_R: u32 = 4;

/// Read one jack-detect line, reporting "not inserted" if it cannot be read.
///
/// These are plain GPIO inputs with no interrupt, so every call samples the pin
/// — the same polling model as the device backend, just via the sound card's
/// read-only jack kcontrols instead of a direct register read.
///
/// A read failure returns `false` rather than propagating: the SDK's jack API is
/// infallible `-> bool` (set by the device backend, where a GPIO read cannot
/// fail), and "no jack detected" is the safe answer — for the speaker policy it
/// errs toward leaving the speaker enabled rather than silently muting it.
fn jack_inserted(jack: u32) -> bool {
    match crate::linux::dev().jack_inserted(jack) {
        Ok(v) => v,
        Err(e) => {
            log::warn!("jack {jack} read failed: {e}");
            false
        }
    }
}

/// Linux: nothing to configure — the kernel owns the detect GPIOs and exports
/// them as card controls; libdeluge opened them when the handle was created.
pub(crate) fn jacks_init() {}
pub(crate) fn jacks_headphone() -> bool {
    jack_inserted(JACK_HEADPHONE)
}
pub(crate) fn jacks_line_in() -> bool {
    jack_inserted(JACK_LINE_IN)
}
pub(crate) fn jacks_mic() -> bool {
    jack_inserted(JACK_MIC)
}
pub(crate) fn jacks_line_out_left() -> bool {
    jack_inserted(JACK_LINE_OUT_L)
}
pub(crate) fn jacks_line_out_right() -> bool {
    jack_inserted(JACK_LINE_OUT_R)
}
/// Linux: request the on-board speaker amplifier on/off.
///
/// **Advisory here, authoritative on device — the one place the two backends
/// genuinely differ.** On device the SDK drives the amp GPIO directly, so
/// `set_speaker(true)` energises it unconditionally. On Linux the kernel owns
/// the policy (`deluge-audio.c`: amp on = this request AND no *output* jack
/// inserted, re-evaluated by a poll), and this sets only the user-intent half
/// via the card's "Speaker Playback Switch". So with headphones plugged in,
/// `set_speaker(true)` leaves the speaker muted.
///
/// That divergence is deliberate rather than an oversight: the kernel already
/// implements this policy for its own ALSA users, and having the SDK bypass it
/// would let an app drive the speaker while headphones are inserted. Apps that
/// need the true state should read the jacks and decide, which works on both
/// backends.
pub(crate) fn jacks_set_speaker(on: bool) {
    if let Err(e) = crate::linux::dev().jacks_set_speaker(on) {
        log::warn!("jacks_set_speaker({on}) failed: {e}");
    }
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
        // 2=encoder, 3=clock. Clock is not an `Event` variant — it feeds the
        // trigger-clock statics instead, which is how `clock_in_*` is served on
        // this backend (the kernel exposes the clock pin as an input device, so
        // edges arrive here rather than through an IRQ of our own).
        if ev.kind == 3 {
            clock_note_edge();
            return;
        }
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
