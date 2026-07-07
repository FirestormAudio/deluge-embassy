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
}

impl Osc {
    pub fn new() -> Osc {
        Osc { phase: 0.0 }
    }

    /// Fill `out` with one block. `freq` in Hz (const or audio-rate); `dt = 1/sr`.
    pub fn process(&mut self, wave: Wave, freq: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            let p = self.phase;
            let dtp = freq.at(i) * dt;
            *s = match wave {
                Wave::Sine => fast_sin(p),
                Wave::Saw => (2.0 * p - 1.0) - poly_blep(p, dtp),
                Wave::Square => {
                    let naive = if p < 0.5 { 1.0 } else { -1.0 };
                    let p2 = p + 0.5;
                    let p2 = p2 - floorf(p2);
                    naive + poly_blep(p, dtp) - poly_blep(p2, dtp)
                }
                Wave::Tri => 1.0 - 4.0 * (p - 0.5).abs(), // (band-limited in Task 3)
            };
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
            osc.process(wave, In::K(freq), dt, &mut out);
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
        osc.process(Wave::Saw, In::K(1_000.0), 1.0 / 48_000.0, &mut out);
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
        osc.process(Wave::Saw, In::K(f0), 1.0 / sr, &mut buf);
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
            osc.process(Wave::Square, In::K(f0), 1.0 / sr, &mut buf);
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
        osc.process(Wave::Square, In::K(f0), 1.0 / sr, &mut buf);
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
        osc.process(Wave::Sine, In::K(100.0), 1.0 / 1000.0, &mut out);
        assert!(out[0].abs() < 1e-3);
        assert!(out.iter().all(|s| s.abs() <= 1.001));
    }
}
