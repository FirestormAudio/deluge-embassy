//! Deluge SDK example: the pad grid as an isomorphic synth keyboard.
//!
//! Turns the 16-wide pad grid into a polyphonic test oscillator. The layout is
//! *isomorphic* (note-layout independent of key): stepping one pad **right** is
//! +1 semitone and one pad **up** is +5 semitones (a perfect fourth) — the same
//! fourths tuning a bass guitar or a LinnStrument uses, so a chord or scale
//! shape is the same anywhere on the grid.
//!
//! It ties four SDK capabilities together: [`Input`] (pad presses), [`Pads`]
//! (LED feedback), [`Audio`] (per-block DSP), and [`Oled`] (an oscilloscope of
//! the synth's own output). Pad presses set per-voice frequency + gate through a
//! small lock-free shared state; the audio half reads it and sums up to
//! [`VOICES`] band-unlimited sine voices with a click-free amplitude envelope,
//! and also snapshots each output block into a scope buffer the OLED half draws.
//! All halves run concurrently via `join`.
//!
//! It is a **synth, not an effect**: it ignores codec line-in and writes its own
//! output, so no input signal is needed — just press pads and listen.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![feature(impl_trait_in_assoc_type)]

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use deluge::prelude::*;

/// Codec sample rate (Hz). Matches `audio_block::SAMPLE_RATE_HZ` on the device
/// and the simulator bridge on the host.
const SAMPLE_RATE: f32 = 44_100.0;

/// Maximum simultaneous voices (chord size). Eight is plenty for the grid and
/// cheap enough to mix per-sample.
const VOICES: usize = 8;

/// Main-grid columns used as keys (the two sidebar columns, x = 16/17, are
/// left dark).
const KEY_COLS: usize = 16;
/// Semitones added per row going up — a perfect fourth = the classic isomorphic
/// ("fourths") tuning.
const ROW_INTERVAL: usize = 5;
/// MIDI note at the bottom-left key pad (x = 0, y = 0). 36 = C2.
const BASE_NOTE: usize = 36;

/// Per-sample amplitude smoothing coefficient (one-pole toward the gate target).
/// ≈ 11 ms at 44.1 kHz — fast enough to feel immediate, slow enough to kill the
/// key click of a hard gate.
const SMOOTH: f32 = 0.002;

/// Per-voice output level before the soft-clip. Keeps a full eight-voice chord
/// from clipping hard while a single note still has body.
const VOICE_GAIN: f32 = 0.18;

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

/// Captured-sample buffer length. Must hold `OLED_W` drawn samples **plus** a
/// search window of at least one full period of the lowest note, so the
/// rising-edge trigger always finds a lock point (C2 ≈ 65 Hz ≈ 674 samples at
/// 44.1 kHz). `1024 − 128 = 896` of headroom covers the whole keyboard range.
const SCOPE_CAP: usize = 1024;

/// Control state shared from the UI half to the audio half, one slot per voice.
///
/// `freq` is the target frequency in Hz stored as `f32` bits; `gate` is the
/// key-down flag the audio envelope chases. Atomics make this safe to write from
/// the UI loop and read from the DSP callback without a lock.
struct VoiceState {
    freq: [AtomicU32; VOICES],
    gate: [AtomicBool; VOICES],
}

static SYNTH: VoiceState = VoiceState {
    freq: [const { AtomicU32::new(0) }; VOICES],
    gate: [const { AtomicBool::new(false) }; VOICES],
};

/// Captured output samples (`f32` bits) for the OLED scope, plus a one-shot
/// "fresh data ready" flag. When the flag is clear the audio half appends each
/// block's output here until the buffer is full, then sets the flag; the OLED
/// half searches it for a trigger point, draws, and clears the flag to request
/// the next capture. A few-block fill at ~30 fps is negligible cost.
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

    // --- Audio half: sum the active voices, forever. ---
    let audio_half = async move {
        // Per-voice running phase in cycles [0, 1) and smoothed amplitude, owned
        // entirely by the DSP callback.
        let mut phase = [0.0f32; VOICES];
        let mut amp = [0.0f32; VOICES];
        // Write cursor into SCOPE while a capture is in progress.
        let mut cap_i = 0usize;

        audio
            .process(move |block: &mut [StereoFrame]| {
                // Capture into the scope buffer only while the OLED half is
                // waiting for fresh data; once it has the flag set it stops
                // until the OLED half consumes it and clears it again.
                let capture = !SCOPE_FILLED.load(Ordering::Relaxed);
                if !capture {
                    // Idle: reset so the next capture starts at the buffer head.
                    cap_i = 0;
                }

                for frame in block.iter_mut() {
                    let mut mix = 0.0f32;
                    for v in 0..VOICES {
                        let f = f32::from_bits(SYNTH.freq[v].load(Ordering::Relaxed));
                        let target = if SYNTH.gate[v].load(Ordering::Relaxed) {
                            1.0
                        } else {
                            0.0
                        };
                        // One-pole envelope toward the gate (no clicks).
                        amp[v] += (target - amp[v]) * SMOOTH;

                        // Advance and wrap the phase, then accumulate.
                        phase[v] += f / SAMPLE_RATE;
                        if phase[v] >= 1.0 {
                            phase[v] -= 1.0;
                        }
                        mix += sine(phase[v]) * amp[v];
                    }
                    // Mono signal to both channels, soft-clipped for headroom.
                    let out = soft_clip(mix * VOICE_GAIN);
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

    // --- UI half: map pad presses to voices and light them up. ---
    let ui_half = async move {
        // Which MIDI note each voice is currently holding (UI-owned bookkeeping).
        let mut note_of: [Option<u8>; VOICES] = [None; VOICES];

        loop {
            if let Event::Pad { x, y, pressed } = input.next().await {
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
        }
    };

    // --- Scope half: redraw the OLED waveform at ~30 fps. ---
    let scope_half = async move {
        use embassy_time::{Duration, Timer};
        loop {
            // Draw whatever the audio half last captured, then clear the flag to
            // request a fresh block.
            draw_scope(&mut oled);
            oled.flush().await;
            SCOPE_FILLED.store(false, Ordering::Relaxed);
            Timer::after(Duration::from_millis(SCOPE_REFRESH_MS)).await;
        }
    };

    // Run all three forever (none completes).
    embassy_futures::join::join3(audio_half, ui_half, scope_half).await;
}

/// Read one captured sample (display-scaled and clamped) from the scope buffer.
#[inline]
fn scope_sample(i: usize) -> f32 {
    (f32::from_bits(SCOPE[i].load(Ordering::Relaxed)) * SCOPE_GAIN).clamp(-1.0, 1.0)
}

/// Find the trigger index: the first rising zero-crossing, drawn at the centre
/// column. Locking each frame to the same phase is what holds the waveform still
/// — without it, the trace slides because each capture begins at a random phase.
/// The search leaves a half-screen of samples on either side so the crossing can
/// sit in the middle; it falls back to the centre index (e.g. silence, where
/// there is no crossing).
fn scope_trigger() -> usize {
    let half = OLED_W / 2;
    for i in (half + 1)..(SCOPE_CAP - half) {
        if scope_sample(i - 1) <= 0.0 && scope_sample(i) > 0.0 {
            return i;
        }
    }
    half
}

/// Render the captured scope buffer into the OLED frame: a title, a dotted
/// centre axis, and the triggered waveform drawn as a connected trace.
fn draw_scope(oled: &mut Oled) {
    oled.clear();
    oled.text(0, Oled::VISIBLE_TOP, "test_osc");

    let mid = (SCOPE_TOP + SCOPE_BOT) / 2;
    let half = ((SCOPE_BOT - SCOPE_TOP) / 2) as f32;
    // Offset so the trigger sample lands on the centre column.
    let start = scope_trigger() - OLED_W / 2;
    let fb = oled.frame();

    // Dotted zero-line.
    for x in (0..OLED_W).step_by(4) {
        fb.set_pixel(x, mid, true);
    }

    // Plot OLED_W samples centred on the trigger, connecting consecutive samples
    // with a vertical run so the trace stays continuous even on steep slopes.
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
/// semitone ratios.
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

/// `|x|` via bit-masking — `f32::abs` isn't in `core` on the device target.
#[inline]
fn fabs(x: f32) -> f32 {
    f32::from_bits(x.to_bits() & 0x7FFF_FFFF)
}

/// Fast parabolic sine approximation. `phase` is in cycles; only its [0, 1)
/// fraction is used (the caller keeps it wrapped). One refinement step keeps the
/// harmonic distortion low enough for a clean test tone.
fn sine(phase: f32) -> f32 {
    use core::f32::consts::PI;
    // [0, 1) → [-PI, PI).
    let a = (phase * 2.0 - 1.0) * PI;
    let y = (4.0 / PI) * a - (4.0 / (PI * PI)) * a * fabs(a);
    -(0.225 * (y * fabs(y) - y) + y)
}

/// Cubic soft saturator: `1.5x - 0.5x³`, output in [-1, 1].
#[inline]
fn soft_clip(x: f32) -> f32 {
    let x = x.clamp(-1.0, 1.0);
    1.5 * x - 0.5 * x * x * x
}
