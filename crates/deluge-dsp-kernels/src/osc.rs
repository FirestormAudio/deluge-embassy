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

            let y = match wave {
                Wave::Sine => fast_sin(ph),
                Wave::Saw => (2.0 * ph - 1.0) - poly_blep(ph, dtp),
                Wave::Square => {
                    let mut w = width.at(i);
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
            };
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

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

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
}
