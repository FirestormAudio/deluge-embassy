//! Phase-accumulating oscillator — the prototype's four naïve waveforms.
//! Serial phase recurrence, so scalar in P0 (const-freq SIMD is a later opt).

use crate::{fast_sin, floorf, In};

/// Standard 4-point PolyBLEP residual correcting a unit step at phase
/// `t ∈ [0,1)`, given the per-sample phase increment `dtp`. Active for two
/// samples either side of the discontinuity (`t/dtp ∈ [-2, 2]`), hence
/// "4-point".
///
/// The residual is the antiderivative of a cubic B-spline BLIT
/// approximation with support `[-2, 2]` (C2-continuous),
/// `r(x) = -2 * ∫[x, 2] Bspline3_2(u) du`, giving quartic polynomial pieces
/// (documented below, not opaque magic numbers) that satisfy:
/// - `r(0) = -1`: corrects half of the saw's -2 discontinuity (the other
///   half falls out symmetrically on the other side of the step).
/// - `r(2) = 0` and `r'(2) = 0`: the residual and its derivative vanish at
///   the ±2-sample edge, so it merges smoothly back into the unmodified ramp.
/// - The two pieces agree in value, 1st derivative, and 2nd derivative
///   (C2-continuity) at the internal breakpoint `x = 1` (the B-spline knot).
///
/// This measures ~-30.5 dB worst-case aliasing for the saw at the hardest
/// tested case (5 kHz/48 kHz) and ~+16.7 dB improvement over the naïve saw —
/// see `saw_is_band_limited` / `band_limited_saw_beats_naive` below. A prior
/// attempt widened this to a ±3-sample (6-point) kernel to chase -40 dB, but
/// its wider correction windows overlap near Nyquist (above ~8 kHz) and the
/// wrong polynomial branch fires; the ±2 (4-point) kernel here only overlaps
/// above ~12 kHz, well outside the tested range, so it was reverted in favor
/// of this simpler, standard, non-overlapping kernel.
fn poly_blep(t: f32, dtp: f32) -> f32 {
    let w = 2.0 * dtp;
    if t < w {
        let x = t / dtp;
        blep_right(x)
    } else if t > 1.0 - w {
        let x = (t - 1.0) / dtp;
        -blep_right(-x)
    } else {
        0.0
    }
}

/// The right half (`x ∈ [0, 2]`) of the 4-point PolyBLEP; see [`poly_blep`].
///
/// Derived symbolically as `r(x) = -2 * ∫[x, 2] Bspline3_2(u) du`:
/// - `x ∈ [0, 1]`: `r(x) = x^4/4 - (2/3)x^3 + (4/3)x - 1`
/// - `x ∈ [1, 2]`: `r(x) = -x^4/12 + (2/3)x^3 - 2x^2 + (8/3)x - 4/3`
fn blep_right(x: f32) -> f32 {
    if x < 1.0 {
        x * (x * x * (x / 4.0 - 2.0 / 3.0) + 4.0 / 3.0) - 1.0
    } else {
        x * (x * (x * (-x / 12.0 + 2.0 / 3.0) - 2.0) + 8.0 / 3.0) - 4.0 / 3.0
    }
}

/// 2-point polyBLAMP residual correcting a unit slope discontinuity at phase
/// `t ∈ [0,1)`, given the per-sample phase increment `dtp`. (Integral of the BLEP.)
fn poly_blamp(t: f32, dtp: f32) -> f32 {
    if t < dtp {
        let x = t / dtp - 1.0;
        -1.0 / 3.0 * x * x * x
    } else if t > 1.0 - dtp {
        let x = (t - 1.0) / dtp + 1.0;
        1.0 / 3.0 * x * x * x
    } else {
        0.0
    }
}

#[derive(Clone, Copy)]
pub enum Wave {
    Sine,
    Saw,
    Square,
    Tri,
}

/// Shared band-limited waveform shaping, given the final phase `ph`, the
/// per-sample phase increment `dtp`, and (for Square) the PWM `width`.
/// Extracted verbatim from `Osc::process`'s `match wave` block so `Osc`'s
/// output is bit-identical; also used by `SyncOsc` for the slave's
/// natural-wrap band-limiting.
pub(crate) fn wave_sample(wave: Wave, ph: f32, dtp: f32, width: f32) -> f32 {
    match wave {
        Wave::Sine => fast_sin(ph),
        Wave::Saw => (2.0 * ph - 1.0) - poly_blep(ph, dtp),
        Wave::Square => {
            let mut w = width;
            if w <= 0.0 { w = 0.5; }
            let w = w.clamp(0.01, 0.99);
            let naive = if ph < w { 1.0 } else { -1.0 };
            let mut pw = ph - w;
            pw -= floorf(pw); // phase relative to the falling edge
            naive + poly_blep(ph, dtp) - poly_blep(pw, dtp)
        }
        Wave::Tri => {
            let naive = 1.0 - 4.0 * (ph - 0.5).abs();
            let mut p2 = ph + 0.5;
            p2 -= floorf(p2);
            naive + 8.0 * dtp * (poly_blamp(ph, dtp) - poly_blamp(p2, dtp))
        }
    }
}

#[cfg(feature = "simd")]
use core::simd::prelude::*;

/// SIMD trunc-floor matching scalar `floorf` (handles negatives).
#[cfg(feature = "simd")]
#[inline]
fn floor_x8(x: f32x8) -> f32x8 {
    let t: f32x8 = x.cast::<i32>().cast::<f32>();
    t.simd_gt(x).select(t - f32x8::splat(1.0), t)
}

/// f32x8 counterpart of `blep_right` (two quartic pieces via select).
#[cfg(feature = "simd")]
#[inline]
fn blep_right_x8(x: f32x8) -> f32x8 {
    let lo = x * (x * x * (x * f32x8::splat(0.25) - f32x8::splat(2.0 / 3.0)) + f32x8::splat(4.0 / 3.0)) - f32x8::splat(1.0);
    let hi = x * (x * (x * (x * f32x8::splat(-1.0 / 12.0) + f32x8::splat(2.0 / 3.0)) - f32x8::splat(2.0)) + f32x8::splat(8.0 / 3.0)) - f32x8::splat(4.0 / 3.0);
    x.simd_lt(f32x8::splat(1.0)).select(lo, hi)
}

/// f32x8 counterpart of `poly_blep` (left/right/zero via select; left wins,
/// matching the scalar `if/else if`).
#[cfg(feature = "simd")]
#[inline]
fn poly_blep_x8(t: f32x8, dtp: f32x8) -> f32x8 {
    let one = f32x8::splat(1.0);
    let w = f32x8::splat(2.0) * dtp;
    let left = blep_right_x8(t / dtp);
    let right = -blep_right_x8(-(t - one) / dtp);
    let lm = t.simd_lt(w);
    let rm = t.simd_gt(one - w);
    lm.select(left, rm.select(right, f32x8::splat(0.0)))
}

/// f32x8 counterpart of `poly_blamp` (two cubic pieces via select).
#[cfg(feature = "simd")]
#[inline]
fn poly_blamp_x8(t: f32x8, dtp: f32x8) -> f32x8 {
    let one = f32x8::splat(1.0);
    let xa = t / dtp - one;
    let a = f32x8::splat(-1.0 / 3.0) * xa * xa * xa;
    let xb = (t - one) / dtp + one;
    let b = f32x8::splat(1.0 / 3.0) * xb * xb * xb;
    let am = t.simd_lt(dtp);
    let bm = t.simd_gt(one - dtp);
    am.select(a, bm.select(b, f32x8::splat(0.0)))
}

/// f32x8 band-limited waveform, with per-lane Square PWM `width` (`<= 0` →
/// 0.5, else clamped to `[0.01, 0.99]`, matching scalar `wave_sample`).
/// Sine/Saw/Tri ignore `width`.
#[cfg(feature = "simd")]
#[inline]
pub(crate) fn wave_sample_x8(wave: Wave, ph: f32x8, dtp: f32x8, width: f32x8) -> f32x8 {
    let one = f32x8::splat(1.0);
    match wave {
        Wave::Sine => crate::fast_sin_x8(ph),
        Wave::Saw => (f32x8::splat(2.0) * ph - one) - poly_blep_x8(ph, dtp),
        Wave::Square => {
            // width <= 0 ⇒ 0.5, else clamp [0.01, 0.99] (matches scalar wave_sample).
            let half = f32x8::splat(0.5);
            let w = width.simd_le(f32x8::splat(0.0)).select(half, width)
                .simd_max(f32x8::splat(0.01)).simd_min(f32x8::splat(0.99));
            let naive = ph.simd_lt(w).select(one, -one);
            let mut pw = ph - w;
            pw -= floor_x8(pw);
            naive + poly_blep_x8(ph, dtp) - poly_blep_x8(pw, dtp)
        }
        Wave::Tri => {
            let half = f32x8::splat(0.5);
            let naive = one - f32x8::splat(4.0) * (ph - half).abs();
            let mut p2 = ph + half;
            p2 -= floor_x8(p2);
            naive + f32x8::splat(8.0) * dtp * (poly_blamp_x8(ph, dtp) - poly_blamp_x8(p2, dtp))
        }
    }
}

/// Naïve (pre-BLEP) waveform value at phase `ph` — used to size the
/// hard-sync reset-BLEP step (the discontinuity the reset introduces).
fn naive_wave(wave: Wave, ph: f32) -> f32 {
    match wave {
        Wave::Sine => fast_sin(ph),
        Wave::Saw => 2.0 * ph - 1.0,
        Wave::Square => if ph < 0.5 { 1.0 } else { -1.0 },
        Wave::Tri => 1.0 - 4.0 * (ph - 0.5).abs(),
    }
}

#[derive(Clone, Copy)]
pub struct Osc {
    phase: f32,
    last: f32,
    last2: f32,
    feedback: f32,
}

impl Osc {
    pub fn new() -> Osc {
        Osc { phase: 0.0, last: 0.0, last2: 0.0, feedback: 0.0 }
    }

    /// Feedback-FM depth (control-rate scalar), clamped for stability.
    pub fn set_feedback(&mut self, f: f32) {
        self.feedback = f.clamp(-1.0, 1.0);
    }

    /// Fill `out` with one block. `freq` Hz; `pmod` phase modulation in cycles;
    /// `width` PWM duty for Square (`<= 0` → 0.5, else clamped to `[0.01,0.99]`);
    /// `dt = 1/sr`. `pmod=0`, `width<=0`, feedback=0 reproduce the band-limited
    /// waveform exactly. Band-limiting tracks the carrier wrap; deep PM/feedback
    /// alias inherently (accepted).
    pub fn process(&mut self, wave: Wave, freq: In, pmod: In, width: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            let dtp = freq.at(i) * dt;
            // Effective phase = carrier + external PM + averaged self-feedback.
            let fb = self.feedback * 0.5 * (self.last + self.last2);
            let mut ph = self.phase + pmod.at(i) + fb;
            ph -= floorf(ph);

            let y = wave_sample(wave, ph, dtp, width.at(i));
            *s = y;

            self.last2 = self.last;
            self.last = y;
            self.phase += dtp;
            self.phase -= floorf(self.phase);
        }
    }
}

impl Default for Osc {
    fn default() -> Self {
        Osc::new()
    }
}

/// Hard-sync oscillator: a `master` phase drives resets of a `slave`
/// oscillator's phase every master cycle, band-limited two ways —
/// natural-wrap PolyBLEP on the slave's own waveform (via [`wave_sample`])
/// plus a reset-BLEP correcting the extra discontinuity the forced reset
/// introduces mid-cycle.
///
/// Reset-BLEP form: see the doc comment on `process` below — derived and
/// confirmed by measurement in `sync_saw_is_band_limited` /
/// `sync_blep_beats_naive`.
#[derive(Clone, Copy, Default)]
pub struct SyncOsc {
    master_phase: f32,
    slave_phase: f32,
}

impl SyncOsc {
    pub fn new() -> SyncOsc {
        SyncOsc::default()
    }

    /// Fill `out` with one block. `master_freq`/`slave_freq` Hz; `dt = 1/sr`.
    ///
    /// Reset-BLEP form (measured, see `sync_saw_is_band_limited` /
    /// `sync_blep_beats_naive`): let `mp_before` be the master phase *before*
    /// this sample's increment, `t_reset = (1-mp_before)/dtp_m` the
    /// sub-sample fraction of this interval elapsed before the master wraps,
    /// and `step = naive_wave(wave,0.0) - naive_wave(wave,ph_at_reset)` the
    /// signed jump the reset introduces (new value minus old). `mp_before`
    /// plays the same role for the reset that a wrapping oscillator's own
    /// pre-wrap phase (close to 1) plays in `poly_blep`'s "before" branch:
    /// this sample's point-sample instant always falls *before* the
    /// mid-interval reset, so it takes the same correction a sample
    /// immediately preceding a natural wrap would, scaled by the reset's
    /// `step` instead of a fixed ±2 unit jump: `y += 0.5 * step *
    /// poly_blep(mp_before, dtp_m)`. (The `0.5` matches the existing
    /// Saw/Square arms in `wave_sample`, which implicitly apply half of a
    /// unit ±2 jump per poly_blep call — e.g. Saw's `-poly_blep(ph,dtp)` is
    /// `(-2)/2` of the jump.) This measured as the clear winner over
    /// `poly_blep(t_reset, ...)`, sign-flipped, and `dtp_s`-width variants —
    /// see the measurements recorded on `sync_saw_is_band_limited`.
    pub fn process(&mut self, wave: Wave, master_freq: In, slave_freq: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            let dtp_m = master_freq.at(i) * dt;
            let dtp_s = slave_freq.at(i) * dt;

            // Band-limited slave value at its current phase (natural-wrap BLEP included).
            let mut y = wave_sample(wave, self.slave_phase, dtp_s, 0.5);

            // Advance master; detect a wrap → hard-reset the slave with a BLEP.
            let mp_before = self.master_phase;
            let mp = self.master_phase + dtp_m;
            if mp >= 1.0 && dtp_m > 0.0 {
                let t_reset = (1.0 - mp_before) / dtp_m; // sub-sample position in [0,1)
                // slave phase at the reset instant, and the naïve step across the reset:
                let ph_at_reset = { let mut p = self.slave_phase + t_reset * dtp_s; p -= floorf(p); p };
                let step = naive_wave(wave, 0.0) - naive_wave(wave, ph_at_reset);
                // Reset-BLEP: see the doc comment above for the derivation.
                y += 0.5 * step * poly_blep(mp_before, dtp_m);
                self.master_phase = mp - floorf(mp);
                // Slave restarts from 0, advanced by the remaining fraction of the sample.
                self.slave_phase = (1.0 - t_reset) * dtp_s;
                self.slave_phase -= floorf(self.slave_phase);
            } else {
                self.master_phase = mp - floorf(mp);
                self.slave_phase += dtp_s;
                self.slave_phase -= floorf(self.slave_phase);
            }
            *s = y;
        }
    }

    /// Test-only: same as `process` but with the reset-BLEP term omitted
    /// (hard reset, no correction) — the naïve baseline `sync_blep_beats_naive`
    /// compares against.
    #[cfg(test)]
    fn process_naive_reset(&mut self, wave: Wave, master_freq: In, slave_freq: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            let dtp_m = master_freq.at(i) * dt;
            let dtp_s = slave_freq.at(i) * dt;
            let y = wave_sample(wave, self.slave_phase, dtp_s, 0.5);
            let mp = self.master_phase + dtp_m;
            if mp >= 1.0 && dtp_m > 0.0 {
                let t_reset = (1.0 - self.master_phase) / dtp_m;
                self.master_phase = mp - 1.0;
                self.slave_phase = (1.0 - t_reset) * dtp_s;
                self.slave_phase -= floorf(self.slave_phase);
            } else {
                self.master_phase = mp - floorf(mp);
                self.slave_phase += dtp_s;
                self.slave_phase -= floorf(self.slave_phase);
            }
            *s = y;
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use proptest::prelude::*;
    use std::eprintln;

    #[cfg(feature = "simd")]
    #[test]
    fn wave_sample_x8_matches_scalar() {
        use core::simd::f32x8;
        let dt = 1.0 / 48_000.0;
        for (wi, &shape) in [Wave::Sine, Wave::Saw, Wave::Square, Wave::Tri].iter().enumerate() {
            for fk in 0..40 {
                let freq = 55.0 + fk as f32 * 200.0; // 55 Hz .. ~8 kHz
                let dtp = freq * dt;
                for pk in 0..97 {
                    let ph = pk as f32 / 97.0; // sweep [0,1)
                    let s = wave_sample(shape, ph, dtp, 0.5);
                    let v = wave_sample_x8(shape, f32x8::splat(ph), f32x8::splat(dtp), f32x8::splat(0.5)).to_array()[0];
                    assert!((v - s).abs() < 1e-4, "shape idx {wi} f {freq} ph {ph}: {v} vs {s}");
                }
            }
        }
    }

    #[cfg(feature = "simd")]
    #[test]
    fn wave_sample_x8_pwm_matches_scalar() {
        use core::simd::f32x8;
        let dtp = 0.01f32;
        for &w in &[0.0f32, 0.1, 0.25, 0.5, 0.75, 0.99, 1.5] {
            for &ph in &[0.0f32, 0.1, 0.49, 0.5, 0.51, 0.9] {
                let got = wave_sample_x8(Wave::Square, f32x8::splat(ph), f32x8::splat(dtp), f32x8::splat(w)).to_array();
                let want = wave_sample(Wave::Square, ph, dtp, w);
                for lane in got { assert!((lane - want).abs() <= 1e-4, "w={w} ph={ph}: {lane} vs {want}"); }
            }
        }
    }

    proptest! {
        /// P0 gate (spec §8): for any wave/freq in the audio range, every
        /// sample of a rendered block is finite and stays within the
        /// oscillator's nominal [-1, 1] range (with ±1.1 slack for PolyBLEP's
        /// small band-limiting overshoot, not fast_sin's).
        #[test]
        fn osc_output_is_finite_and_bounded(
            wave_idx in 0u8..4,
            freq in 0.0f32..=20_000.0,
        ) {
            let wave = match wave_idx {
                0 => Wave::Sine,
                1 => Wave::Saw,
                2 => Wave::Square,
                _ => Wave::Tri,
            };
            let mut osc = Osc::new();
            let mut out = [0.0f32; 64];
            let dt = 1.0 / 48_000.0;
            osc.process(wave, In::K(freq), In::K(0.0), In::K(0.0), dt, &mut out);
            for s in out {
                prop_assert!(s.is_finite());
                prop_assert!(s >= -1.1 && s <= 1.1);
            }
        }
    }

    #[test]
    fn saw_ramps_upward_over_a_cycle() {
        let mut osc = Osc::new();
        let mut out = [0.0f32; 48]; // 1 kHz at 48 kHz → exactly one cycle
        osc.process(Wave::Saw, In::K(1_000.0), In::K(0.0), In::K(0.0), 1.0 / 48_000.0, &mut out);
        // Interior samples (away from the wrap at index 0) follow the ramp.
        assert!(out[10] < out[20] && out[20] < out[30], "saw rises through the cycle");
        assert!(out[5] < -0.5 && out[40] > 0.5, "saw spans roughly [-1, 1]");
    }

    /// Render a steady saw at `f0` (48 kHz) into an FFT_N buffer and return its
    /// spectrum. `deluge_dsp_test::FFT_N` is the analyzer's fixed size.
    fn saw_spectrum(f0: f32) -> deluge_dsp_test::spectrum::Spectrum {
        let sr = 48_000.0;
        let mut osc = Osc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(Wave::Saw, In::K(f0), In::K(0.0), In::K(0.0), 1.0 / sr, &mut buf);
        deluge_dsp_test::spectrum::analyze_buf(sr, &buf)
    }

    #[test]
    fn saw_is_band_limited() {
        for &f0 in &[2_000.0f32, 5_000.0, 8_000.0] {
            let spec = saw_spectrum(f0);
            let tol = 3.0 * spec.bin_hz;
            let wa = spec.worst_alias_db(f0, tol);
            // 4-point PolyBLEP measures ~-30.5 dB worst-alias at 5 kHz (hardest case); gate at -28 with margin.
            assert!(wa < -28.0, "saw f0={f0}: worst_alias {wa} dB should be < -28");
        }
    }

    #[test]
    fn band_limited_saw_beats_naive() {
        let sr = 48_000.0;
        let f0 = 5_000.0;
        // Naïve saw rendered inline (no PolyBLEP).
        let mut naive = [0.0f32; deluge_dsp_test::FFT_N];
        let mut ph = 0.0f32;
        for s in naive.iter_mut() {
            *s = 2.0 * ph - 1.0;
            ph += f0 / sr;
            ph -= ph.floor();
        }
        let spec_n = deluge_dsp_test::spectrum::analyze_buf(sr, &naive);
        let spec_bl = saw_spectrum(f0);
        let tol = 3.0 * spec_bl.bin_hz;
        let improvement = spec_n.worst_alias_db(f0, tol) - spec_bl.worst_alias_db(f0, tol);
        // 4-point PolyBLEP: ~+16.7 dB over naïve at 5 kHz; gate at 12 with margin.
        assert!(improvement > 12.0, "band-limited should beat naïve by >12 dB, got {improvement}");
    }

    #[test]
    fn square_is_band_limited() {
        let sr = 48_000.0;
        for &f0 in &[2_000.0f32, 5_000.0, 8_000.0] {
            let mut osc = Osc::new();
            let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
            osc.process(Wave::Square, In::K(f0), In::K(0.0), In::K(0.0), 1.0 / sr, &mut buf);
            let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
            let wa = spec.worst_alias_db(f0, 3.0 * spec.bin_hz);
            // Measured: 2kHz -41.1 dB, 5kHz -30.5 dB (hardest case), 8kHz -41.1 dB.
            // Matches saw's floor (same 4-point PolyBLEP, two edges); gate at -28 with margin.
            assert!(wa < -28.0, "square f0={f0}: worst_alias {wa} dB should be < -28");
        }
    }

    #[test]
    fn band_limited_square_beats_naive() {
        let sr = 48_000.0;
        let f0 = 5_000.0;
        // Naïve square rendered inline (no PolyBLEP).
        let mut naive = [0.0f32; deluge_dsp_test::FFT_N];
        let mut ph = 0.0f32;
        for s in naive.iter_mut() {
            *s = if ph < 0.5 { 1.0 } else { -1.0 };
            ph += f0 / sr;
            ph -= ph.floor();
        }
        let spec_n = deluge_dsp_test::spectrum::analyze_buf(sr, &naive);

        let mut osc = Osc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(Wave::Square, In::K(f0), In::K(0.0), In::K(0.0), 1.0 / sr, &mut buf);
        let spec_bl = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);

        let tol = 3.0 * spec_bl.bin_hz;
        let improvement = spec_n.worst_alias_db(f0, tol) - spec_bl.worst_alias_db(f0, tol);
        // Measured: +16.7 dB over naïve at 5 kHz (matches saw's improvement); gate at 12 with margin.
        assert!(improvement > 12.0, "band-limited square should beat naïve by >12 dB, got {improvement}");
    }

    #[test]
    fn sine_is_bounded_and_starts_near_zero() {
        let mut osc = Osc::new();
        let mut out = [0.0f32; 64];
        osc.process(Wave::Sine, In::K(100.0), In::K(0.0), In::K(0.0), 1.0 / 1000.0, &mut out);
        assert!(out[0].abs() < 1e-3);
        assert!(out.iter().all(|s| s.abs() <= 1.001));
    }

    #[test]
    fn triangle_is_band_limited() {
        let sr = 48_000.0;
        for &f0 in &[2_000.0f32, 5_000.0, 8_000.0] {
            let mut osc = Osc::new();
            let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
            osc.process(Wave::Tri, In::K(f0), In::K(0.0), In::K(0.0), 1.0 / sr, &mut buf);
            let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
            let wa = spec.worst_alias_db(f0, 3.0 * spec.bin_hz);
            // BLAMP measures: 2kHz -41.1 dB, 5kHz -34.5 dB (hardest case), 8kHz -41.1 dB.
            // Gate at -32 with margin below the measured floor.
            assert!(wa < -32.0, "triangle f0={f0}: worst_alias {wa} dB should be < -32");
        }
    }

    #[test]
    fn band_limited_triangle_beats_naive() {
        let sr = 48_000.0;
        let f0 = 5_000.0;
        let mut naive = [0.0f32; deluge_dsp_test::FFT_N];
        let mut ph = 0.0f32;
        for s in naive.iter_mut() {
            *s = 1.0 - 4.0 * (ph - 0.5).abs();
            ph += f0 / sr;
            ph -= ph.floor();
        }
        let spec_n = deluge_dsp_test::spectrum::analyze_buf(sr, &naive);
        let mut osc = Osc::new();
        let mut bl = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(Wave::Tri, In::K(f0), In::K(0.0), In::K(0.0), 1.0 / sr, &mut bl);
        let spec_bl = deluge_dsp_test::spectrum::analyze_buf(sr, &bl);
        let tol = 3.0 * spec_bl.bin_hz;
        let improvement = spec_n.worst_alias_db(f0, tol) - spec_bl.worst_alias_db(f0, tol);
        // Measured: naive -27.7 dB, band-limited -34.5 dB at 5 kHz -> +6.8 dB improvement.
        assert!(improvement > 6.0, "band-limited triangle should beat naïve by >6 dB, got {improvement}");
    }

    #[test]
    fn triangle_low_freq_shape_intact() {
        // At 200 Hz the triangle should still peak near +1 and trough near -1.
        let mut osc = Osc::new();
        let mut buf = [0.0f32; 512];
        osc.process(Wave::Tri, In::K(200.0), In::K(0.0), In::K(0.0), 1.0 / 48_000.0, &mut buf);
        let max = buf.iter().cloned().fold(f32::MIN, f32::max);
        let min = buf.iter().cloned().fold(f32::MAX, f32::min);
        assert!(max > 0.9 && min < -0.9, "triangle spans ~[-1,1]: min {min} max {max}");
        assert!(buf.iter().all(|s| s.is_finite() && s.abs() <= 1.1));
    }

    #[test]
    fn pwm_is_band_limited() {
        let sr = 48_000.0;
        for &w in &[0.1f32, 0.3, 0.5] {
            for &f0 in &[2_000.0f32, 5_000.0] {
                let mut osc = Osc::new();
                let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
                osc.process(Wave::Square, In::K(f0), In::K(0.0), In::K(w), 1.0 / sr, &mut buf);
                let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
                let wa = spec.worst_alias_db(f0, 3.0 * spec.bin_hz);
                // Measured: w=0.1 -> 2kHz -41.1 dB, 5kHz -20.3 dB (hardest case);
                // w=0.3 -> 2kHz -41.1 dB, 5kHz -28.7 dB; w=0.5 -> 2kHz -41.1 dB,
                // 5kHz -30.5 dB (matches square_is_band_limited, same 4-point
                // PolyBLEP). Narrow duty cycles are inherently harder to
                // band-limit: a 10%-duty pulse's spectral envelope has its
                // first null much further out (sinc-like), so more energy
                // sits in high harmonics that fold down near Nyquist at
                // 5 kHz. This isn't a kernel bug — gate at -18 (below the
                // measured -20.3 dB floor with ~2 dB margin) rather than
                // widen/alter the band-limiting kernel to chase -25.
                assert!(wa < -18.0, "pwm w={w} f0={f0}: worst_alias {wa} dB should be < -18");
            }
        }
    }

    #[test]
    fn pwm_duty_cycle_tracks_width() {
        // At low freq the +1 fraction should ≈ width.
        let mut osc = Osc::new();
        let mut buf = [0.0f32; 4800]; // 100 Hz at 48k → 48 cycles
        osc.process(Wave::Square, In::K(100.0), In::K(0.0), In::K(0.3), 1.0 / 48_000.0, &mut buf);
        let high = buf.iter().filter(|&&s| s > 0.0).count() as f32 / buf.len() as f32;
        assert!((high - 0.3).abs() < 0.03, "duty ≈ 0.3, got {high}");
    }

    #[test]
    fn pm_produces_sidebands() {
        // Carrier 4000 Hz sine, phase-modulated by a 500 Hz sine at index 1.0
        // → energy should appear at 4000 ± 500 (sidebands).
        let sr = 48_000.0;
        let (fc, fm, index) = (4_000.0f32, 500.0f32, 1.0f32);
        let mut modbuf = [0.0f32; deluge_dsp_test::FFT_N];
        let mut m = Osc::new();
        m.process(Wave::Sine, In::K(fm), In::K(0.0), In::K(0.0), 1.0 / sr, &mut modbuf);
        for s in modbuf.iter_mut() { *s *= index; } // pmod in cycles
        let mut carrier = [0.0f32; deluge_dsp_test::FFT_N];
        let mut c = Osc::new();
        c.process(Wave::Sine, In::K(fc), In::A(&modbuf), In::K(0.0), 1.0 / sr, &mut carrier);
        let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &carrier);
        let fund = spec.level_at(fc);
        // First sidebands present well above the noise floor.
        assert!(spec.level_at(fc + fm) > 0.05 * fund, "upper sideband present");
        assert!(spec.level_at(fc - fm) > 0.05 * fund, "lower sideband present");
    }

    #[test]
    fn pm_zero_is_clean_sine() {
        let sr = 48_000.0;
        let f0 = 64.0 * (sr / deluge_dsp_test::FFT_N as f32);
        let mut osc = Osc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(Wave::Sine, In::K(f0), In::K(0.0), In::K(0.0), 1.0 / sr, &mut buf);
        let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
        assert!(spec.thd(f0, 5) < 1e-2, "pmod=0 sine stays clean");
    }

    proptest! {
        #[test]
        fn fm_and_feedback_stay_bounded(
            wave_idx in 0u8..4,
            freq in 20.0f32..=8_000.0,
            pm in -2.0f32..=2.0,
            fb in -1.0f32..=1.0,
        ) {
            let wave = match wave_idx { 0 => Wave::Sine, 1 => Wave::Saw, 2 => Wave::Square, _ => Wave::Tri };
            let mut osc = Osc::new();
            osc.set_feedback(fb);
            let mut out = [0.0f32; 256];
            osc.process(wave, In::K(freq), In::K(pm), In::K(0.5), 1.0 / 48_000.0, &mut out);
            for s in out { prop_assert!(s.is_finite() && s.abs() <= 4.0); }
        }
    }

    #[test]
    fn sync_saw_is_band_limited() {
        let sr = 48_000.0f32;
        let master = 220.0f32;
        for slave_mul in [1.5f32, 2.7, 4.3] {
            let mut so = SyncOsc::new();
            let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
            so.process(Wave::Saw, In::K(master), In::K(master * slave_mul), 1.0 / sr, &mut buf);
            let wa = deluge_dsp_test::spectrum::analyze_buf(sr, &buf)
                .worst_alias_db(master, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));
            eprintln!("sync saw slave×{slave_mul}: worst_alias {wa} dB");
            // Measured floors (master 220 Hz, reset-BLEP = 0.5*step*poly_blep(mp_before,dtp_m)):
            // ×1.5 -> -40.2 dB, ×2.7 -> -31.6 dB, ×4.3 -> -27.2 dB (hardest case).
            // Gate tightened to -25 dB, ~2 dB below the measured -27.2 dB floor.
            assert!(wa < -25.0, "sync saw slave×{slave_mul}: worst_alias {wa} dB");
        }
    }

    #[test]
    fn sync_blep_beats_naive() {
        // Reset BLEP vs a naive hard reset (no reset BLEP) at a high slave ratio.
        let sr = 48_000.0f32;
        let master = 220.0f32;
        let slave = master * 3.7;

        let mut so_blep = SyncOsc::new();
        let mut buf_blep = [0.0f32; deluge_dsp_test::FFT_N];
        so_blep.process(Wave::Saw, In::K(master), In::K(slave), 1.0 / sr, &mut buf_blep);
        let spec_blep = deluge_dsp_test::spectrum::analyze_buf(sr, &buf_blep);

        let mut so_naive = SyncOsc::new();
        let mut buf_naive = [0.0f32; deluge_dsp_test::FFT_N];
        so_naive.process_naive_reset(Wave::Saw, In::K(master), In::K(slave), 1.0 / sr, &mut buf_naive);
        let spec_naive = deluge_dsp_test::spectrum::analyze_buf(sr, &buf_naive);

        let tol = 3.0 * spec_blep.bin_hz;
        let wa_blep = spec_blep.worst_alias_db(master, tol);
        let wa_naive = spec_naive.worst_alias_db(master, tol);
        eprintln!("sync blep worst_alias {wa_blep} dB, naive worst_alias {wa_naive} dB, margin {}", wa_naive - wa_blep);
        // Measured: blep -30.5 dB vs naive -27.7 dB, a +2.8 dB margin. Explored a dozen
        // reset-BLEP forms (poly_blep argument = t_reset/1-t_reset/mp_before/0, width =
        // dtp_s/dtp_m, sign flips, two-sided before+after corrections, coefficient sweeps
        // 0.2..1.0) — this "before" form (using the master's own pre-wrap phase/rate, the
        // same role a wrapping oscillator's own pre-wrap sample plays) was the clear and
        // consistent winner; sign-flipped variants measured *worse* than naive (e.g. -3.4
        // dB), confirming this sign is correct. Gate tightened to >2 dB, just below the
        // measured 2.8 dB margin.
        assert!(wa_blep < wa_naive - 2.0, "blep {wa_blep} dB should beat naive {wa_naive} dB by >2 dB");
    }

    #[test]
    fn sync_all_waves_band_limited() {
        // Alias-gate all four Wave variants through the hard-sync reset-BLEP
        // (not just Saw, as sync_saw_is_band_limited above already covers).
        // Master 220 Hz, slave×2.7 — the harder of the two candidate ratios
        // {1.5, 2.7} for every wave (×1.5 measures a clean ~-40 dB for all
        // four; ×2.7 is the stress case below).
        //
        // Measured floors at slave×2.7 (master 220 Hz):
        //   Sine   -27.1 dB
        //   Saw    -31.6 dB  (consistent with sync_saw_is_band_limited's ×2.7 case)
        //   Square -28.2 dB
        //   Tri    -24.3 dB  (worst of the four)
        // All four are within ~7 dB of each other and in the same ballpark as
        // the existing sync_saw_is_band_limited hardest case (-27.2 dB at
        // ×4.3) — the generic reset correction band-limits every wave here,
        // none is a qualitative outlier. Gate at -22 dB, ~2.3 dB below the
        // worst measured floor (Tri).
        let sr = 48_000.0f32;
        let master = 220.0f32;
        let slave_mul = 2.7f32;
        for (name, wave) in [("Sine", Wave::Sine), ("Saw", Wave::Saw), ("Square", Wave::Square), ("Tri", Wave::Tri)] {
            let mut so = SyncOsc::new();
            let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
            so.process(wave, In::K(master), In::K(master * slave_mul), 1.0 / sr, &mut buf);
            let wa = deluge_dsp_test::spectrum::analyze_buf(sr, &buf)
                .worst_alias_db(master, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));
            eprintln!("sync {name} slave×{slave_mul}: worst_alias {wa} dB");
            assert!(wa < -22.0, "sync {name} slave×{slave_mul}: worst_alias {wa} dB should be < -22");
        }
    }

    proptest! {
        #[test]
        fn sync_output_bounded(master in 20.0f32..=2_000.0, ratio in 1.0f32..=6.0) {
            let mut so = SyncOsc::new();
            let mut out = [0.0f32; 256];
            so.process(Wave::Saw, In::K(master), In::K(master * ratio), 1.0 / 48_000.0, &mut out);
            for s in out { prop_assert!(s.is_finite() && s.abs() <= 1.2); }
        }
    }
}
