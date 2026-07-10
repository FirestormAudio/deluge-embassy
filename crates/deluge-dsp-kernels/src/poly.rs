//! Polyphony kernels: a per-voice settable source (`PolyCtrl`), a poly
//! oscillator (`PolyOsc`), and the voice→mono collapse (`voice_sum`). All poly
//! buffers are voice-interleaved (sample-major): `tile[i * VOICES + v]` is voice
//! `v` at sample `i`, so the voice loop vectorizes to `f32x8` on NEON.

#[cfg(not(feature = "simd"))]
use crate::{fast_sin, floorf};
use crate::env::Ar;
use crate::In;

/// Voices processed in parallel per poly node. Fixed at compile time.
pub const VOICES: usize = 8;

/// A per-voice settable scalar source: lane `v` outputs `values[v]`. No input.
/// The allocator's write-target (Sy-3). Output tile is voice-interleaved.
#[derive(Clone, Copy)]
pub struct PolyCtrl {
    values: [f32; VOICES],
}
impl PolyCtrl {
    pub fn new() -> PolyCtrl {
        PolyCtrl { values: [0.0; VOICES] }
    }
    pub fn set_voice(&mut self, v: usize, x: f32) {
        if v < VOICES {
            self.values[v] = x;
        }
    }
    /// `out` is voice-interleaved, length `VOICES * n_samples`.
    pub fn process(&mut self, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            for v in 0..VOICES {
                out[i * VOICES + v] = self.values[v];
            }
        }
    }
}
impl Default for PolyCtrl {
    fn default() -> Self {
        Self::new()
    }
}

// The f32x8 poly path assumes exactly 8 voices. Changing VOICES requires
// revisiting the SIMD width (e.g. f32x16 or 2× f32x8).
#[cfg(feature = "simd")]
const _: () = assert!(VOICES == 8);

/// A poly oscillator. `pitch` is a voice-interleaved tile of per-voice Hz;
/// writes a voice-interleaved audio tile. SoA phase; voice loop is the inner
/// (vectorizable) dimension. Raw sine shape (band-limiting is a later concern).
#[derive(Clone, Copy)]
pub struct PolyOsc {
    phase: [f32; VOICES], // [0,1) per voice
}
impl PolyOsc {
    pub fn new() -> PolyOsc {
        PolyOsc { phase: [0.0; VOICES] }
    }
    /// `pitch` and `out` are voice-interleaved, length `VOICES * n_samples`.
    pub fn process(&mut self, pitch: &[f32], dt: f32, out: &mut [f32]) {
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            let n = out.len() / VOICES;
            let one = f32x8::splat(1.0);
            let dtv = f32x8::splat(dt);
            let mut ph = f32x8::from_array(self.phase);
            for i in 0..n {
                let f = f32x8::from_slice(&pitch[i * VOICES..]);
                let mut p = ph + f * dtv;
                // wrap: p -= trunc-floor(p), matching scalar `floorf`
                let t: f32x8 = p.cast::<i32>().cast::<f32>();
                let fl = t.simd_gt(p).select(t - one, t);
                p -= fl;
                ph = p;
                crate::fast_sin_x8(p).copy_to_slice(&mut out[i * VOICES..]);
            }
            self.phase = ph.to_array();
        }
        #[cfg(not(feature = "simd"))]
        {
            let n = out.len() / VOICES;
            for i in 0..n {
                for v in 0..VOICES {
                    let f = pitch[i * VOICES + v];
                    let mut p = self.phase[v] + f * dt;
                    p -= floorf(p); // wrap [0,1)
                    self.phase[v] = p;
                    out[i * VOICES + v] = fast_sin(p);
                }
            }
        }
    }
}
impl Default for PolyOsc {
    fn default() -> Self {
        Self::new()
    }
}

/// A pure poly envelope source: 8 independent attack/release envelopes with
/// per-voice gate state. attack/release are shared (mono) controls; output is a
/// voice-interleaved tile of levels ∈ [0,1]. No audio input — a modulation
/// source. Scalar (the AR state machine is branchy and cheap).
#[derive(Clone, Copy)]
pub struct PolyAr {
    voices: [Ar; VOICES],
}
impl PolyAr {
    pub fn new() -> PolyAr {
        PolyAr { voices: [Ar::new(); VOICES] }
    }
    pub fn gate_voice(&mut self, v: usize, on: bool) {
        if v < VOICES { self.voices[v].gate(on); }
    }
    pub fn trigger_voice(&mut self, v: usize) {
        if v < VOICES { self.voices[v].trigger(); }
    }
    /// attack/release mono controls; writes a voice-interleaved env tile.
    pub fn process(&mut self, attack: In, release: In, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            let atk = attack.at(i);
            let rel = release.at(i);
            for v in 0..VOICES {
                out[i * VOICES + v] = self.voices[v].tick(atk, rel, dt);
            }
        }
    }
}
impl Default for PolyAr {
    fn default() -> Self { Self::new() }
}

/// Collapse a voice-interleaved tile to mono: `out[i] = Σ_v tile[i*VOICES + v]`.
/// `tile.len() == VOICES * out.len()`.
pub fn voice_sum(tile: &[f32], out: &mut [f32]) {
    for i in 0..out.len() {
        let mut s = 0.0;
        for v in 0..VOICES {
            s += tile[i * VOICES + v];
        }
        out[i] = s;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::{fast_sin, floorf}; // explicit so tests build under `--features simd` too

    #[test]
    fn polyctrl_fills_interleaved_lanes() {
        let mut c = PolyCtrl::new();
        for v in 0..VOICES {
            c.set_voice(v, v as f32 + 1.0); // 1..=8
        }
        let n = 5;
        let mut out = std::vec![0.0f32; VOICES * n];
        c.process(&mut out);
        for i in 0..n {
            for v in 0..VOICES {
                assert_eq!(out[i * VOICES + v], v as f32 + 1.0, "lane {v} sample {i}");
            }
        }
    }

    #[test]
    fn polyctrl_ignores_out_of_range_voice() {
        let mut c = PolyCtrl::new();
        c.set_voice(VOICES, 99.0); // no-op, no panic
        let mut out = std::vec![0.0f32; VOICES];
        c.process(&mut out);
        assert!(out.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn polyosc_renders_independent_per_voice_partials() {
        // Voice v runs at (v+1)*100 Hz. Each lane must equal a reference sine at
        // its own frequency, and lanes must not cross-contaminate.
        let dt = 1.0 / 48_000.0;
        let n = 480;
        let mut pitch = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                pitch[i * VOICES + v] = (v as f32 + 1.0) * 100.0;
            }
        }
        let mut osc = PolyOsc::new();
        let mut out = std::vec![0.0f32; VOICES * n];
        osc.process(&pitch, dt, &mut out);
        // Reference: independent phase accumulators per voice.
        let mut ph = [0.0f32; VOICES];
        for i in 0..n {
            for v in 0..VOICES {
                let f = (v as f32 + 1.0) * 100.0;
                ph[v] += f * dt;
                ph[v] -= floorf(ph[v]);
                let want = fast_sin(ph[v]);
                assert!((out[i * VOICES + v] - want).abs() < 1e-5, "voice {v} sample {i}");
            }
        }
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.0001));
    }

    #[cfg(feature = "simd")]
    #[test]
    fn fast_sin_x8_matches_scalar() {
        use core::simd::f32x8;
        // Sweep phases across two full cycles; every lane must match scalar fast_sin.
        for base in 0..250 {
            let ps: [f32; 8] = core::array::from_fn(|k| (base as f32 * 8.0 + k as f32) / 1000.0);
            let v = crate::fast_sin_x8(f32x8::from_array(ps)).to_array();
            for k in 0..8 {
                assert!((v[k] - crate::fast_sin(ps[k])).abs() < 1e-6, "phase {} lane {k}", ps[k]);
            }
        }
    }

    #[test]
    fn voice_sum_adds_lanes_per_sample() {
        let n = 4;
        let mut tile = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                tile[i * VOICES + v] = (i * VOICES + v) as f32; // distinct
            }
        }
        let mut out = std::vec![0.0f32; n];
        voice_sum(&tile, &mut out);
        for i in 0..n {
            let want: f32 = (0..VOICES).map(|v| (i * VOICES + v) as f32).sum();
            assert_eq!(out[i], want, "sample {i}");
        }
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn polyosc_bounded(f in 0.0f32..8000.0) {
            let dt = 1.0 / 48_000.0;
            let n = 64;
            let pitch = std::vec![f; VOICES * n];
            let mut osc = PolyOsc::new();
            let mut out = std::vec![0.0f32; VOICES * n];
            osc.process(&pitch, dt, &mut out);
            for &s in &out {
                prop_assert!(s.is_finite() && s.abs() <= 1.0001, "polyosc unbounded: {s}");
            }
        }
    }

    #[test]
    fn polyar_gates_per_voice_independently() {
        let dt = 1.0 / 48_000.0;
        let mut env = PolyAr::new();
        env.gate_voice(0, true); // voice 0 attacks
        env.gate_voice(3, true); // voice 3 attacks
        let n = 4800; // 100 ms
        // attack/release are mono broadcast blocks (same for all voices per sample).
        let atk_mono: std::vec::Vec<f32> = (0..n).map(|_| 0.01).collect(); // 10 ms attack
        let rel_mono: std::vec::Vec<f32> = (0..n).map(|_| 0.05).collect();
        let mut out = std::vec![0.0f32; VOICES * n];
        env.process(In::A(&atk_mono), In::A(&rel_mono), dt, &mut out);
        // Gated voices reach ~1.0 by the end of a 100 ms window (10 ms attack).
        assert!((out[(n - 1) * VOICES + 0] - 1.0).abs() < 1e-3, "voice 0 reached sustain");
        assert!((out[(n - 1) * VOICES + 3] - 1.0).abs() < 1e-3, "voice 3 reached sustain");
        // Ungated voices stay silent.
        for v in [1usize, 2, 4, 5, 6, 7] {
            assert!(out[(n - 1) * VOICES + v].abs() < 1e-9, "voice {v} silent");
        }
        // Monotonic rise for voice 0 over the attack.
        assert!(out[10 * VOICES + 0] > out[0 * VOICES + 0]);
    }

    #[test]
    fn polyar_release_returns_to_zero() {
        let dt = 1.0 / 48_000.0;
        let mut env = PolyAr::new();
        env.gate_voice(0, true);
        let n = 480;
        let atk: std::vec::Vec<f32> = (0..n).map(|_| 0.001).collect(); // 1 ms
        let rel: std::vec::Vec<f32> = (0..n).map(|_| 0.001).collect();
        let mut out = std::vec![0.0f32; VOICES * n];
        env.process(In::A(&atk), In::A(&rel), dt, &mut out);
        assert!((out[(n - 1) * VOICES] - 1.0).abs() < 1e-3, "reached sustain");
        env.gate_voice(0, false); // release
        let mut out2 = std::vec![0.0f32; VOICES * n];
        env.process(In::A(&atk), In::A(&rel), dt, &mut out2);
        assert!(out2[(n - 1) * VOICES].abs() < 1e-3, "released to 0");
    }
}
