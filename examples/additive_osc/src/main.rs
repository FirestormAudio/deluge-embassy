//! Deluge SDK example: a polyphonic **additive** synth whose DSP core is
//! **C++/Argon** (ARM NEON SIMD) reached over **FFI**.
//!
//! It is the `test_osc` pad-keyboard synth — the 16-wide grid as an isomorphic
//! keyboard (one pad right = +1 semitone, one pad up = +5 = a perfect fourth),
//! with an OLED oscilloscope of its own output — but the per-block DSP is *not*
//! written in Rust. Every audio block, the active voices are handed to
//! [`additive_render`], a C++ function in [`csrc/additive.cpp`] that synthesises
//! the sound with [Argon](https://github.com/stellar-aria/argon), a header-only
//! zero-overhead C++ wrapper over NEON. Each voice is summed from up to 64 sine
//! partials (additive synthesis), four samples at a time in SIMD.
//!
//! This exercises two things at once:
//!   - **FFI** — Rust ↔ C++ over a C ABI ([`AdditiveVoice`] is shared
//!     field-for-field; the C++ is compiled by `build.rs` with `cc`);
//!   - **real-world SIMD DSP** — a vectorised parabolic sine + harmonic sum in
//!     Argon, on NEON (device) or SIMDe (`cargo deluge sim` host).
//!
//! **Timbre** is live: the **first gold encoder** sets the harmonic count
//! (1–32, brightness/bandwidth) and the **second** sets the spectral roll-off
//! order (0 = buzzy/equal, 1 ≈ sawtooth, 2–3 = darker). The OLED shows both.
//!
//! It is a **synth, not an effect**: it ignores codec line-in and writes its own
//! output — just press pads and listen.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![feature(impl_trait_in_assoc_type)]

use core::fmt::Write as _;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use deluge::prelude::*;

/// Codec sample rate (Hz). Matches `audio_block::SAMPLE_RATE_HZ` on the device
/// and the simulator bridge on the host.
const SAMPLE_RATE: f32 = 44_100.0;

/// Maximum simultaneous voices (chord size).
const VOICES: usize = 8;

/// Upper bound on the audio block length; the per-block mono scratch is sized to
/// this. Both the device and the host bridge use 128-frame blocks.
const MAX_BLOCK: usize = 256;

/// Main-grid columns used as keys (the two sidebar columns, x = 16/17, are
/// left dark).
const KEY_COLS: usize = 16;
/// Semitones added per row going up — a perfect fourth = the classic isomorphic
/// ("fourths") tuning.
const ROW_INTERVAL: usize = 5;
/// MIDI note at the bottom-left key pad (x = 0, y = 0). 36 = C2.
const BASE_NOTE: usize = 36;

/// Per-block amplitude smoothing coefficient (one-pole toward the gate target).
/// One step per ~2.9 ms block; ≈ 12 ms time constant — kills the key click of a
/// hard gate without an audible step.
const AMP_SMOOTH: f32 = 0.2;

/// Per-voice output level before the soft-clip. Keeps a full eight-voice chord
/// from clipping hard while a single note still has body.
const VOICE_GAIN: f32 = 0.18;

/// Timbre, set live by the gold encoders and read by the audio half.
static N_HARMONICS: AtomicU32 = AtomicU32::new(8);
static ROLLOFF: AtomicU32 = AtomicU32::new(1);
const HARMONICS_MIN: u32 = 1;
const HARMONICS_MAX: u32 = 32;
const ROLLOFF_MAX: u32 = 3;

/// OLED panel width in pixels — also the number of samples drawn per frame.
const OLED_W: usize = 128;
/// Full OLED height (`VISIBLE_TOP` bezel rows + the visible rows).
const OLED_H: usize = Oled::VISIBLE_TOP + Oled::VISIBLE_HEIGHT;
/// Vertical span of the scope trace (below the title row).
const SCOPE_TOP: usize = Oled::VISIBLE_TOP + 10;
const SCOPE_BOT: usize = OLED_H - 1;
/// Display-only gain so a single quiet voice still draws a visible trace.
const SCOPE_GAIN: f32 = 2.5;
/// Scope refresh period (~30 fps).
const SCOPE_REFRESH_MS: u64 = 33;

/// Captured-sample buffer length. Holds `OLED_W` drawn samples **plus** a search
/// window of at least one full period of the lowest note for the rising-edge
/// trigger (C2 ≈ 65 Hz ≈ 674 samples at 44.1 kHz).
const SCOPE_CAP: usize = 1024;

// === FFI boundary: the C++/Argon additive DSP core (csrc/additive.cpp) ===

/// One synth voice, shared field-for-field with the C++ `struct AdditiveVoice`.
/// `freq`/`amp_start`/`amp` are set by Rust each block; `phase` (the fundamental
/// phase in cycles, [0, 1)) is owned by Rust but advanced *in place* by the C++
/// renderer so partials stay phase-locked across blocks.
///
/// The envelope is a **pair**: `amp_start` is where the block begins and `amp`
/// where it ends, and the renderer interpolates between them per sample. A
/// single `amp` held constant for the whole block makes the envelope a
/// staircase that jumps every 128 frames — inaudible in steady state (where
/// successive values are equal) but a burst of ~10 audible steps through every
/// attack and release, which is what "static on note onset and release" turned
/// out to be on hardware.
///
/// Field order is load-bearing: this is `#[repr(C)]` and must match
/// `csrc/additive.cpp`'s struct exactly.
#[repr(C)]
#[derive(Clone, Copy)]
struct AdditiveVoice {
    freq: f32,
    /// Amplitude at the START of the block (the previous block's `amp`).
    amp_start: f32,
    /// Amplitude at the END of the block; also the envelope's running state.
    amp: f32,
    phase: f32,
}

unsafe extern "C" {
    /// Sum every active voice's additive tone into `out` (mono, `n` samples,
    /// overwritten). See `csrc/additive.cpp` for the full contract.
    fn additive_render(
        out: *mut f32,
        n: usize,
        voices: *mut AdditiveVoice,
        n_voices: usize,
        n_harmonics: u32,
        rolloff: u32,
        sample_rate: f32,
    );
}

/// Control state shared from the UI half to the audio half, one slot per voice.
/// `freq` is the target frequency in Hz stored as `f32` bits; `gate` is the
/// key-down flag the audio envelope chases.
struct VoiceState {
    freq: [AtomicU32; VOICES],
    gate: [AtomicBool; VOICES],
}

static SYNTH: VoiceState = VoiceState {
    freq: [const { AtomicU32::new(0) }; VOICES],
    gate: [const { AtomicBool::new(false) }; VOICES],
};

/// Captured output samples (`f32` bits) for the OLED scope, plus a one-shot
/// "fresh data ready" flag (see `test_osc` for the handshake).
static SCOPE: [AtomicU32; SCOPE_CAP] = [const { AtomicU32::new(0) }; SCOPE_CAP];
static SCOPE_FILLED: AtomicBool = AtomicBool::new(false);

#[deluge::app]
async fn main(dlg: Deluge) {
    // Bring up the codec first (its one-time init blocks briefly), then the
    // input stream and the pad LEDs.
    let audio = dlg.audio();
    let input = dlg.input();
    let mut pads = dlg.pads().await;
    let mut oled = dlg.oled().await;

    // Paint the resting keyboard: every key pad dim, root notes (C) brighter as
    // landmarks. Sidebar columns stay off.
    pads.clear();
    for y in 0..Pads::ROWS {
        for x in 0..KEY_COLS {
            if let Some(note) = pad_to_note(x, y) {
                pads.set(x, y, key_color(note, false));
            }
        }
    }
    pads.flush().await;

    // --- Audio half: render the active voices in C++/Argon, forever. ---
    let audio_half = async move {
        // Per-voice DSP state owned by the callback. `phase` persists across
        // blocks (the C++ renderer advances it); `amp` is the smoothed envelope.
        let mut voices = [AdditiveVoice {
            freq: 0.0,
            amp_start: 0.0,
            amp: 0.0,
            phase: 0.0,
        }; VOICES];
        // Write cursor into SCOPE while a capture is in progress.
        let mut cap_i = 0usize;

        audio
            .process(move |block: &mut [StereoFrame]| {
                let n = block.len();
                debug_assert!(n <= MAX_BLOCK);
                let n_harmonics = N_HARMONICS.load(Ordering::Relaxed);
                let rolloff = ROLLOFF.load(Ordering::Relaxed);

                // Pull each voice's target from the shared state and step its
                // one-pole amplitude envelope toward the gate.
                for (v, voice) in voices.iter_mut().enumerate() {
                    voice.freq = f32::from_bits(SYNTH.freq[v].load(Ordering::Relaxed));
                    let target = if SYNTH.gate[v].load(Ordering::Relaxed) {
                        1.0
                    } else {
                        0.0
                    };
                    // Hand the renderer both ends of this block's envelope so it
                    // can interpolate per sample; stepping `amp` alone would
                    // leave a discontinuity at every block boundary.
                    voice.amp_start = voice.amp;
                    voice.amp += (target - voice.amp) * AMP_SMOOTH;
                }

                // Hand the whole voice bank to the C++/Argon additive renderer.
                let mut mono = [0.0f32; MAX_BLOCK];
                // SAFETY: `mono` holds `n <= MAX_BLOCK` samples and `voices` holds
                // exactly `VOICES`; the C++ writes `out[0..n]` and advances each
                // voice's `phase` in place. `AdditiveVoice` is `repr(C)` and
                // layout-identical to the C++ struct.
                unsafe {
                    additive_render(
                        mono.as_mut_ptr(),
                        n,
                        voices.as_mut_ptr(),
                        VOICES,
                        n_harmonics,
                        rolloff,
                        SAMPLE_RATE,
                    );
                }

                // Capture into the scope buffer only while the OLED half is
                // waiting for fresh data.
                let capture = !SCOPE_FILLED.load(Ordering::Relaxed);
                if !capture {
                    cap_i = 0;
                }

                // Mono signal to both channels, soft-clipped for headroom.
                for (frame, &m) in block.iter_mut().zip(mono.iter()) {
                    let out = soft_clip(m * VOICE_GAIN);
                    frame.l = out;
                    frame.r = out;
                    if capture && cap_i < SCOPE_CAP {
                        SCOPE[cap_i].store(out.to_bits(), Ordering::Relaxed);
                        cap_i += 1;
                    }
                }

                // Buffer full → hand it to the OLED half.
                if capture && cap_i >= SCOPE_CAP {
                    SCOPE_FILLED.store(true, Ordering::Relaxed);
                }
            })
            .await
    };

    // --- UI half: map pad presses to voices, encoders to timbre. ---
    let ui_half = async move {
        // Which MIDI note each voice is currently holding (UI-owned bookkeeping).
        let mut note_of: [Option<u8>; VOICES] = [None; VOICES];

        loop {
            match input.next().await {
                Event::Pad { x, y, pressed } => {
                    let (xu, yu) = (x as usize, y as usize);
                    let Some(note) = pad_to_note(xu, yu) else {
                        continue;
                    };
                    if pressed {
                        voice_on(&mut note_of, note);
                    } else {
                        voice_off(&mut note_of, note);
                    }
                    pads.set(xu, yu, key_color(note, pressed));
                    pads.flush().await;
                }
                // Gold encoder 0 → harmonic count, encoder 1 → roll-off order.
                Event::Encoder { index, delta } => match index {
                    0 => adjust(&N_HARMONICS, delta, HARMONICS_MIN, HARMONICS_MAX),
                    1 => adjust(&ROLLOFF, delta, 0, ROLLOFF_MAX),
                    _ => {}
                },
                _ => {}
            }
        }
    };

    // --- Scope half: redraw the OLED waveform at ~30 fps. ---
    let scope_half = async move {
        use embassy_time::{Duration, Timer};
        loop {
            draw_scope(&mut oled);
            oled.flush().await;
            SCOPE_FILLED.store(false, Ordering::Relaxed);
            Timer::after(Duration::from_millis(SCOPE_REFRESH_MS)).await;
        }
    };

    // Run all three forever (none completes).
    embassy_futures::join::join3(audio_half, ui_half, scope_half).await;
}

/// Apply a signed encoder delta to a clamped counter.
fn adjust(cell: &AtomicU32, delta: i8, lo: u32, hi: u32) {
    let next = (cell.load(Ordering::Relaxed) as i32 + delta as i32).clamp(lo as i32, hi as i32);
    cell.store(next as u32, Ordering::Relaxed);
}

/// Read one captured sample (display-scaled and clamped) from the scope buffer.
#[inline]
fn scope_sample(i: usize) -> f32 {
    (f32::from_bits(SCOPE[i].load(Ordering::Relaxed)) * SCOPE_GAIN).clamp(-1.0, 1.0)
}

/// Find the trigger index: the first rising zero-crossing, drawn at the centre
/// column, so the trace holds still. Falls back to the centre index on silence.
fn scope_trigger() -> usize {
    let half = OLED_W / 2;
    for i in (half + 1)..(SCOPE_CAP - half) {
        if scope_sample(i - 1) <= 0.0 && scope_sample(i) > 0.0 {
            return i;
        }
    }
    half
}

/// Render the captured scope buffer into the OLED frame: a live timbre title, a
/// dotted centre axis, and the triggered waveform drawn as a connected trace.
fn draw_scope(oled: &mut Oled) {
    oled.clear();

    // Title line shows the live additive timbre.
    let mut title = FmtBuf::<24>::new();
    let _ = write!(
        title,
        "additive H{} R{}",
        N_HARMONICS.load(Ordering::Relaxed),
        ROLLOFF.load(Ordering::Relaxed)
    );
    oled.text(0, Oled::VISIBLE_TOP, title.as_str());

    let mid = (SCOPE_TOP + SCOPE_BOT) / 2;
    let half = ((SCOPE_BOT - SCOPE_TOP) / 2) as f32;
    let start = scope_trigger() - OLED_W / 2;
    let fb = oled.frame();

    // Dotted zero-line.
    for x in (0..OLED_W).step_by(4) {
        fb.set_pixel(x, mid, true);
    }

    // Plot OLED_W samples centred on the trigger, connecting consecutive samples
    // with a vertical run so the trace stays continuous on steep slopes.
    let mut prev_y = mid as i32;
    for x in 0..OLED_W {
        let s = scope_sample(start + x);
        let y = (mid as f32 - s * half) as i32;
        let (lo, hi) = if y < prev_y { (y, prev_y) } else { (prev_y, y) };
        for yy in lo..=hi {
            if yy >= SCOPE_TOP as i32 && yy <= SCOPE_BOT as i32 {
                fb.set_pixel(x, yy as usize, true);
            }
        }
        prev_y = y;
    }
}

/// Grid coordinate → MIDI note for the isomorphic layout, or `None` for the
/// sidebar / out-of-range pads.
fn pad_to_note(x: usize, y: usize) -> Option<u8> {
    if x >= KEY_COLS {
        return None;
    }
    let n = BASE_NOTE + x + ROW_INTERVAL * y;
    if n > 127 { None } else { Some(n as u8) }
}

/// Resting / pressed colour for a key, hued by pitch class. Root notes (C) are
/// shown brighter and a touch desaturated so they read as landmarks.
fn key_color(note: u8, pressed: bool) -> Color {
    let pc = note % 12;
    let hue = (pc as u16 * 256 / 12) as u8;
    let (sat, val) = if pressed {
        (255, 255)
    } else if pc == 0 {
        (170, 90) // root landmark
    } else {
        (255, 18) // dim key
    };
    Color::hsv(hue, sat, val)
}

/// Assign `note` to a free voice (stealing voice 0 if all are busy) and gate it
/// on.
fn voice_on(note_of: &mut [Option<u8>; VOICES], note: u8) {
    let idx = note_of.iter().position(Option::is_none).unwrap_or(0);
    note_of[idx] = Some(note);
    SYNTH.freq[idx].store(midi_to_freq(note).to_bits(), Ordering::Relaxed);
    SYNTH.gate[idx].store(true, Ordering::Relaxed);
}

/// Gate off whichever voice is holding `note`.
fn voice_off(note_of: &mut [Option<u8>; VOICES], note: u8) {
    if let Some(idx) = note_of.iter().position(|&n| n == Some(note)) {
        note_of[idx] = None;
        SYNTH.gate[idx].store(false, Ordering::Relaxed);
    }
}

/// Equal-tempered MIDI-note → frequency (A4 = 440 Hz at note 69), computed
/// without `libm`: split into whole octaves (powers of two) and the twelve
/// semitone ratios. (The tritone ratio is `2^(6/12)`, which is exactly √2 — hence
/// the `approx_constant` allow; these are musical ratios, not the constant.)
#[allow(clippy::approx_constant, clippy::excessive_precision)]
fn midi_to_freq(note: u8) -> f32 {
    const RATIO: [f32; 12] = [
        1.000_000, 1.059_463, 1.122_462, 1.189_207, 1.259_921, 1.334_840, 1.414_214, 1.498_307,
        1.587_401, 1.681_793, 1.781_797, 1.887_749,
    ];
    let rel = note as i32 - 69;
    let oct = rel.div_euclid(12);
    let semi = rel.rem_euclid(12) as usize;
    let mut f = 440.0 * RATIO[semi];
    if oct >= 0 {
        f *= (1u32 << oct) as f32;
    } else {
        f /= (1u32 << (-oct)) as f32;
    }
    f
}

/// Cubic soft saturator: `1.5x - 0.5x³`, output in [-1, 1].
#[inline]
fn soft_clip(x: f32) -> f32 {
    let x = x.clamp(-1.0, 1.0);
    1.5 * x - 0.5 * x * x * x
}

/// Tiny fixed-capacity `core::fmt::Write` sink — formats the OLED title without
/// an allocator.
struct FmtBuf<const N: usize> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> FmtBuf<N> {
    fn new() -> Self {
        Self {
            buf: [0; N],
            len: 0,
        }
    }
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl<const N: usize> core::fmt::Write for FmtBuf<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let take = bytes.len().min(N - self.len);
        self.buf[self.len..self.len + take].copy_from_slice(&bytes[..take]);
        self.len += take;
        Ok(())
    }
}
