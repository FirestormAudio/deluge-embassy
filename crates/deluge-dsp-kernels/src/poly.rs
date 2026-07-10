//! Polyphony kernels: a per-voice settable source (`PolyCtrl`), a poly
//! oscillator (`PolyOsc`), and the voice→mono collapse (`voice_sum`). All poly
//! buffers are voice-interleaved (sample-major): `tile[i * VOICES + v]` is voice
//! `v` at sample `i`, so the voice loop vectorizes to `f32x8` on NEON.

#[cfg(not(feature = "simd"))]
use crate::{fast_sin, floorf};
use crate::env::Ar;
#[cfg(not(feature = "simd"))]
use crate::filter::{Svf, SvfResp};
use crate::filter::{svf_coeffs, svf_k_from_res, svf_tan_prewarp};
use crate::quant::semitones_to_hz;
use crate::In;
use core::f32::consts::PI;

/// Voices processed in parallel per poly node. Fixed at compile time.
pub const VOICES: usize = 8;

/// Scalar SVF coefficients for a shared (mono) cutoff/res. Returns (k, a1, a2, a3).
/// Uses the polynomial prewarp (matches scalar `Svf`'s audio-rate path).
#[inline]
fn poly_svf_coeffs(fc: f32, res: f32, dt: f32) -> (f32, f32, f32, f32) {
    let theta = (PI * fc.max(1.0) * dt).min(0.49 * PI);
    let g = svf_tan_prewarp(theta);
    let k = svf_k_from_res(res);
    let (a1, a2, a3) = svf_coeffs(g, k);
    (k, a1, a2, a3)
}

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

#[cfg(feature = "simd")]
const _: () = assert!(VOICES == 8);

/// Poly SVF lowpass. Poly audio in → 8 filtered lanes; shared mono cutoff/res.
/// Scalar path holds `[Svf; VOICES]` and reuses the audited `Svf::tick`; the
/// SIMD path keeps SoA `f32x8` state register-resident. Both use identical
/// scalar coeffs, so they agree.
#[derive(Clone, Copy)]
pub struct PolySvf {
    #[cfg(not(feature = "simd"))]
    voices: [Svf; VOICES],
    #[cfg(feature = "simd")]
    ic1: [f32; VOICES],
    #[cfg(feature = "simd")]
    ic2: [f32; VOICES],
}
impl PolySvf {
    #[cfg(not(feature = "simd"))]
    pub fn new() -> PolySvf {
        PolySvf { voices: [Svf::new(); VOICES] }
    }
    #[cfg(feature = "simd")]
    pub fn new() -> PolySvf {
        PolySvf { ic1: [0.0; VOICES], ic2: [0.0; VOICES] }
    }

    /// `audio` = voice-interleaved poly input; cutoff/res mono; LP output tile.
    pub fn process(&mut self, audio: &[f32], cutoff: In, res: In, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            let two = f32x8::splat(2.0);
            let mut ic1 = f32x8::from_array(self.ic1);
            let mut ic2 = f32x8::from_array(self.ic2);
            for i in 0..n {
                let (_k, a1, a2, a3) = poly_svf_coeffs(cutoff.at(i), res.at(i), dt);
                let (a1v, a2v, a3v) = (f32x8::splat(a1), f32x8::splat(a2), f32x8::splat(a3));
                let v0 = f32x8::from_slice(&audio[i * VOICES..]);
                let v3 = v0 - ic2;
                let v1 = a1v * ic1 + a2v * v3;
                let v2 = ic2 + a2v * ic1 + a3v * v3;
                ic1 = two * v1 - ic1;
                ic2 = two * v2 - ic2;
                v2.copy_to_slice(&mut out[i * VOICES..]); // LP = v2
            }
            self.ic1 = ic1.to_array();
            self.ic2 = ic2.to_array();
        }
        #[cfg(not(feature = "simd"))]
        {
            for i in 0..n {
                let (k, a1, a2, a3) = poly_svf_coeffs(cutoff.at(i), res.at(i), dt);
                for v in 0..VOICES {
                    out[i * VOICES + v] =
                        self.voices[v].tick(audio[i * VOICES + v], k, a1, a2, a3, SvfResp::Lp);
                }
            }
        }
    }
}
impl Default for PolySvf {
    fn default() -> Self { Self::new() }
}

/// Poly × poly, lanewise: `out[j] = a[j] * b[j]`. The VCA. Auto-vectorizes.
pub fn poly_mul(a: &[f32], b: &[f32], out: &mut [f32]) {
    for j in 0..out.len() {
        out[j] = a[j] * b[j];
    }
}

/// Poly semitone→Hz: `out[v] = ref_hz · 2^(semitone[v]/12)`. Poly-in note-offset
/// lanes → Hz lanes. Scalar (pitch is control-rate, not a recurrent kernel).
#[derive(Clone, Copy)]
pub struct PolyMtof {
    ref_hz: f32,
}
impl PolyMtof {
    pub fn new() -> PolyMtof {
        PolyMtof { ref_hz: 440.0 }
    }
    pub fn set_ref(&mut self, hz: f32) {
        self.ref_hz = hz;
    }
    /// `semitones` = voice-interleaved note-offset tile; writes an Hz tile
    /// (element-wise, so the interleave is preserved trivially).
    pub fn process(&mut self, semitones: &[f32], out: &mut [f32]) {
        for j in 0..out.len() {
            out[j] = semitones_to_hz(semitones[j], self.ref_hz);
        }
    }
}
impl Default for PolyMtof {
    fn default() -> Self { Self::new() }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::{fast_sin, floorf}; // explicit so tests build under `--features simd` too

    fn make_sine_tile(freq: f32, n: usize, dt: f32) -> std::vec::Vec<f32> {
        let mut t = std::vec![0.0f32; VOICES * n];
        let mut ph = 0.0f32;
        for i in 0..n {
            ph += freq * dt;
            ph -= floorf(ph);
            let s = fast_sin(ph);
            for v in 0..VOICES {
                t[i * VOICES + v] = s;
            }
        }
        t
    }

    #[test]
    fn polysvf_attenuates_above_cutoff() {
        // A high-frequency input (18 kHz) fed to a low cutoff (300 Hz) comes out
        // much smaller; a low tone (100 Hz) passes ~unchanged.
        let dt = 1.0 / 48_000.0;
        let n = 4800;
        let hi = make_sine_tile(18_000.0, n, dt);
        let lo = make_sine_tile(100.0, n, dt);
        let cutoff: std::vec::Vec<f32> = (0..n).map(|_| 300.0).collect();
        let res: std::vec::Vec<f32> = (0..n).map(|_| 0.0).collect();
        let mut out_hi = std::vec![0.0f32; VOICES * n];
        let mut out_lo = std::vec![0.0f32; VOICES * n];
        PolySvf::new().process(&hi, In::A(&cutoff), In::A(&res), dt, &mut out_hi);
        PolySvf::new().process(&lo, In::A(&cutoff), In::A(&res), dt, &mut out_lo);
        let peak = |t: &[f32]| t[VOICES * n / 2..].iter().fold(0.0f32, |m, &s| m.max(s.abs()));
        assert!(peak(&out_hi) < 0.3, "18 kHz attenuated: {}", peak(&out_hi));
        assert!(peak(&out_lo) > 0.7, "100 Hz passes: {}", peak(&out_lo));
    }

    #[test]
    fn polysvf_matches_scalar_svf_single_voice() {
        // Cross-check: PolySvf voice 0 ≈ a scalar Svf on the same signal.
        use crate::filter::{Svf, SvfResp};
        let dt = 1.0 / 48_000.0;
        let n = 2000;
        let sig = make_sine_tile(1000.0, n, dt); // all 8 voices identical here
        let cutoff: std::vec::Vec<f32> = (0..n).map(|i| 500.0 + i as f32).collect(); // audio-rate → both use prewarp
        let res: std::vec::Vec<f32> = (0..n).map(|_| 0.3).collect();
        let mut poly_out = std::vec![0.0f32; VOICES * n];
        PolySvf::new().process(&sig, In::A(&cutoff), In::A(&res), dt, &mut poly_out);
        let mono_in: std::vec::Vec<f32> = (0..n).map(|i| sig[i * VOICES]).collect();
        let mut mono_out = std::vec![0.0f32; n];
        Svf::new().process(In::A(&mono_in), In::A(&cutoff), In::A(&res), SvfResp::Lp, dt, &mut mono_out);
        for i in 0..n {
            assert!((poly_out[i * VOICES] - mono_out[i]).abs() < 1e-4, "sample {i}");
        }
    }

    #[test]
    fn poly_mul_lanewise() {
        let a: [f32; VOICES * 2] = core::array::from_fn(|j| j as f32);
        let b: [f32; VOICES * 2] = core::array::from_fn(|_| 2.0);
        let mut out = [0.0f32; VOICES * 2];
        poly_mul(&a, &b, &mut out);
        for j in 0..VOICES * 2 {
            assert_eq!(out[j], j as f32 * 2.0);
        }
    }

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

    #[test]
    fn polyar_trigger_voice_is_one_shot() {
        // trigger_voice → attack then release with no sustain, on the addressed
        // lane only. Voice 2 rises to ~1 then decays back to 0; others stay 0.
        let dt = 1.0 / 48_000.0;
        let mut env = PolyAr::new();
        env.trigger_voice(2);
        let n = 2000;
        let atk: std::vec::Vec<f32> = (0..n).map(|_| 0.002).collect(); // 2 ms
        let rel: std::vec::Vec<f32> = (0..n).map(|_| 0.005).collect(); // 5 ms
        let mut out = std::vec![0.0f32; VOICES * n];
        env.process(In::A(&atk), In::A(&rel), dt, &mut out);
        let peak = (0..n).map(|i| out[i * VOICES + 2]).fold(0.0f32, f32::max);
        assert!(peak > 0.99, "one-shot reaches peak: {peak}");
        assert!(out[(n - 1) * VOICES + 2] < 0.05, "one-shot decays after: {}", out[(n - 1) * VOICES + 2]);
        assert!(out[(n - 1) * VOICES].abs() < 1e-9, "untriggered voice 0 silent");
    }

    #[test]
    fn polymtof_maps_semitones_to_hz() {
        let mut m = PolyMtof::new(); // ref 440
        let offsets = [0.0f32, 12.0, -12.0, 7.0, -9.0, 24.0, 1.0, -1.0];
        let n = 3;
        let mut inp = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                inp[i * VOICES + v] = offsets[v];
            }
        }
        let mut out = std::vec![0.0f32; VOICES * n];
        m.process(&inp, &mut out);
        assert!((out[0] - 440.0).abs() < 1e-2, "0 → 440: {}", out[0]);
        assert!((out[1] - 880.0).abs() < 1e-2, "+12 → 880: {}", out[1]);
        assert!((out[2] - 220.0).abs() < 1e-2, "-12 → 220: {}", out[2]);
        assert!(out.iter().all(|&h| h.is_finite() && h > 0.0));
    }

    #[test]
    fn polymtof_set_ref_retunes() {
        let mut m = PolyMtof::new();
        m.set_ref(100.0);
        let mut out = std::vec![0.0f32; VOICES];
        m.process(&std::vec![0.0f32; VOICES], &mut out);
        assert!(out.iter().all(|&h| (h - 100.0).abs() < 1e-3));
    }
}
