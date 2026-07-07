//! Phase-accumulating oscillator — the prototype's four naïve waveforms.
//! Serial phase recurrence, so scalar in P0 (const-freq SIMD is a later opt).

use crate::{fast_sin, floorf, In};

/// Half-width (in samples) of the PolyBLEP corrective window on each side of
/// the discontinuity. See [`poly_blep`] for why this is 3, not the textbook 1.
const BLEP_M: f32 = 3.0;

/// Wide-kernel PolyBLEP residual correcting a unit step at phase `t ∈ [0,1)`,
/// given the per-sample phase increment `dtp`.
///
/// The classic 2-point PolyBLEP (a quadratic residual built from a triangular
/// approximation to the band-limited impulse, active for one sample either
/// side of the discontinuity) measured only ~-22 dB worst-case aliasing for
/// the saw at 5 kHz/48 kHz — short of the -40 dB target and short of even
/// 20 dB improvement over the naïve saw. A `BLEP_M = 2` (4-point) cubic
/// B-spline variant improved that to ~-30.5 dB / +16.7 dB — still short of
/// both. Widening the window to `BLEP_M = 3` (three samples either side, six
/// total) reaches ~-41 dB / +27 dB, clearing both bars. The polynomial pieces
/// in [`blep4_right`] are the antiderivative of a `BLEP_M`-wide cubic
/// B-spline BLIT approximation (C2-continuous, support `[-BLEP_M, BLEP_M]`),
/// `r(x) = -2 * ∫[x, BLEP_M] Bspline3_M(u) du`, solved symbolically so that
/// `r(0) = -1` (half of the saw's -2 discontinuity), `r(BLEP_M) = r'(BLEP_M)
/// = 0` (smooth merge back into the unmodified ramp), and the two pieces
/// agree in value/derivative/2nd-derivative at the internal breakpoint
/// `x = BLEP_M / 2`.
fn poly_blep(t: f32, dtp: f32) -> f32 {
    let w = BLEP_M * dtp;
    if t < w {
        let x = t / dtp;
        blep4_right(x)
    } else if t > 1.0 - w {
        let x = (t - 1.0) / dtp;
        -blep4_right(-x)
    } else {
        0.0
    }
}

/// The right half (`x ∈ [0, BLEP_M]`) of the wide-kernel PolyBLEP; see [`poly_blep`].
fn blep4_right(x: f32) -> f32 {
    if x < 1.5 {
        x * (x * x * (4.0 * x / 81.0 - 16.0 / 81.0) + 8.0 / 9.0) - 1.0
    } else {
        x * (x * (x * (16.0 / 81.0 - 4.0 * x / 243.0) - 8.0 / 9.0) + 16.0 / 9.0) - 4.0 / 3.0
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
                    // (filled in Task 2 — keep the naïve value for now)
                    if p < 0.5 {
                        1.0
                    } else {
                        -1.0
                    }
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
            assert!(wa < -40.0, "saw f0={f0}: worst_alias {wa} dB should be < -40");
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
        assert!(improvement > 20.0, "band-limited should beat naïve by >20 dB, got {improvement}");
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
