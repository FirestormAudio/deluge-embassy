//! Spectral analysis: render → Hann window → real FFT → linear magnitude bins,
//! plus relative metrics (Tasks 2–3). Built on `realfft` (an independent,
//! reference-grade FFT — the harness never measures with code under test).

use realfft::RealFftPlanner;

use crate::FFT_N;

/// Linear amplitude ratio → decibels (floored so silence doesn't produce -inf).
fn lin_to_db(ratio: f32) -> f32 {
    20.0 * ratio.max(1e-12).log10()
}

/// Periodic Hann window `w[i] = 0.5 - 0.5·cos(2π i / N)`. Periodic (not symmetric)
/// so a bin-centered tone produces an exact 3-bin spectrum with no far leakage.
fn hann_in_place(buf: &mut [f32]) {
    let n = buf.len() as f32;
    for (i, s) in buf.iter_mut().enumerate() {
        let w = 0.5 - 0.5 * (std::f32::consts::TAU * i as f32 / n).cos();
        *s *= w;
    }
}

/// One-sided linear magnitude spectrum of a windowed real signal.
pub struct Spectrum {
    /// Linear magnitudes for bins `0..=FFT_N/2` (length `FFT_N/2 + 1`).
    pub bins: Vec<f32>,
    /// Hz per bin (`sample_rate / FFT_N`).
    pub bin_hz: f32,
    pub sample_rate: f32,
}

/// Render `FFT_N` samples from `renderer`, then analyze.
pub fn analyze(sample_rate: f32, mut renderer: impl FnMut(&mut [f32])) -> Spectrum {
    let mut buf = [0.0f32; FFT_N];
    renderer(&mut buf);
    analyze_buf(sample_rate, &buf)
}

/// Analyze an already-rendered `FFT_N`-length buffer.
pub fn analyze_buf(sample_rate: f32, signal: &[f32; FFT_N]) -> Spectrum {
    // realfft's `process` uses its input slice as scratch, so work on a copy.
    let mut input: Vec<f32> = signal.to_vec();
    hann_in_place(&mut input);

    // Plan a forward real FFT for FFT_N points (per-call is fine for a test tool).
    let r2c = RealFftPlanner::<f32>::new().plan_fft_forward(FFT_N);
    let mut out = r2c.make_output_vec(); // Vec<Complex<f32>>, len FFT_N/2 + 1
    r2c.process(&mut input, &mut out).expect("realfft forward");

    // One-sided complex bins → linear magnitude of each (`num_complex::norm`).
    let bins: Vec<f32> = out.iter().map(|c| c.norm()).collect();

    Spectrum { bins, bin_hz: sample_rate / FFT_N as f32, sample_rate }
}

impl Spectrum {
    /// Index of the loudest bin.
    pub fn peak_bin(&self) -> usize {
        let mut best = 0;
        let mut best_v = f32::NEG_INFINITY;
        for (i, &v) in self.bins.iter().enumerate() {
            if v > best_v {
                best_v = v;
                best = i;
            }
        }
        best
    }

    /// Linear magnitude of the loudest bin.
    pub fn peak_linear(&self) -> f32 {
        self.bins[self.peak_bin()]
    }

    /// Nearest bin index to `hz` (clamped to the valid range).
    pub fn bin_of_hz(&self, hz: f32) -> usize {
        let b = (hz / self.bin_hz).round() as isize;
        b.clamp(0, self.bins.len() as isize - 1) as usize
    }

    /// Linear magnitude at the bin nearest `hz`.
    pub fn level_at(&self, hz: f32) -> f32 {
        self.bins[self.bin_of_hz(hz)]
    }

    /// Peak LINEAR magnitude within `±tol_bins` of `hz`.
    fn peak_near_linear(&self, hz: f32, tol_bins: usize) -> f32 {
        let center = self.bin_of_hz(hz) as isize;
        let lo = (center - tol_bins as isize).max(0) as usize;
        let hi = (center + tol_bins as isize).min(self.bins.len() as isize - 1) as usize;
        self.bins[lo..=hi].iter().cloned().fold(0.0f32, f32::max)
    }

    /// dB of the peak within `±tol` bins of `hz`, relative to `ref_lin`.
    fn peak_near_db(&self, hz: f32, tol_bins: usize, ref_lin: f32) -> f32 {
        lin_to_db(self.peak_near_linear(hz, tol_bins) / ref_lin)
    }

    /// dB of each harmonic `k·f0` (k = 1..=count) relative to the fundamental.
    pub fn harmonics_db(&self, f0: f32, count: usize) -> Vec<f32> {
        let fund = self.level_at(f0).max(1e-20);
        (1..=count)
            .map(|k| self.peak_near_db(k as f32 * f0, 2, fund))
            .collect()
    }

    /// Total harmonic distortion: `sqrt(Σ_{k≥2} h_k²) / h_1` (linear ratio).
    ///
    /// Uses the same `±2`-bin peak search as `harmonics_db` so the two metrics
    /// agree, since a real oscillator's fundamental (and each harmonic) is
    /// generally not bin-centered — a single-bin lookup would understate
    /// off-bin amplitudes due to Hann scalloping loss.
    pub fn thd(&self, f0: f32, count: usize) -> f32 {
        let fund = self.peak_near_linear(f0, 2).max(1e-20);
        let sum_sq: f32 = (2..=count)
            .map(|k| {
                let h = self.peak_near_linear(k as f32 * f0, 2);
                h * h
            })
            .sum();
        sum_sq.sqrt() / fund
    }

    /// dB of the loudest bin that is NOT a harmonic of `f0`, relative to the
    /// fundamental. Excludes DC and bins within `tol_hz` of any `k·f0`.
    pub fn worst_alias_db(&self, f0: f32, tol_hz: f32) -> f32 {
        let fund = self.level_at(f0).max(1e-20);
        let mut worst = 0.0f32;
        for (i, &mag) in self.bins.iter().enumerate() {
            let hz = i as f32 * self.bin_hz;
            if hz <= tol_hz {
                continue; // skip DC / very low bins (window leakage)
            }
            // Is `hz` within tol of some harmonic k*f0 (k ≥ 1)?
            let k = (hz / f0).round().max(1.0);
            let nearest_harmonic = k * f0;
            if (hz - nearest_harmonic).abs() <= tol_hz {
                continue; // legitimate harmonic, not an alias
            }
            worst = worst.max(mag);
        }
        lin_to_db(worst / fund)
    }

    /// dB of the median bin relative to the peak (a spectral noise-floor proxy).
    pub fn noise_floor_db(&self) -> f32 {
        let mut v = self.bins.clone();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = v[v.len() / 2];
        lin_to_db(median / self.peak_linear().max(1e-20))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FFT_N;
    use std::f32::consts::TAU;

    /// A sine whose frequency lands exactly on an FFT bin (no leakage bias):
    /// bin_hz = sr/N, so f0 = 64*bin_hz is exactly bin 64.
    fn bin_centered_f0(sample_rate: f32) -> f32 {
        64.0 * (sample_rate / FFT_N as f32)
    }

    #[test]
    fn pure_sine_peaks_at_its_bin() {
        let sr = 48_000.0;
        let f0 = bin_centered_f0(sr); // 375.0 Hz at 48k/8192
        let spec = analyze(sr, |b| {
            for (i, s) in b.iter_mut().enumerate() {
                *s = (TAU * f0 * i as f32 / sr).sin();
            }
        });
        assert_eq!(spec.peak_bin(), 64, "peak should be at bin 64 (f0)");
        assert!((spec.bin_of_hz(f0) as i32 - 64).abs() <= 1);
        // The fundamental dominates: peak is far above a mid-spectrum bin.
        assert!(spec.peak_linear() > 100.0 * spec.level_at(f0 * 10.0));
    }

    fn render_sines(sr: f32, comps: &[(f32, f32)]) -> Spectrum {
        analyze(sr, |b| {
            for (i, s) in b.iter_mut().enumerate() {
                let t = i as f32 / sr;
                *s = comps.iter().map(|&(f, a)| a * (TAU * f * t).sin()).sum();
            }
        })
    }

    #[test]
    fn thd_matches_a_planted_second_harmonic() {
        let sr = 48_000.0;
        let f0 = bin_centered_f0(sr); // 375 Hz (bin 64)
        // fundamental at 1.0, 2nd harmonic (750 Hz, bin 128) at 0.1.
        let spec = render_sines(sr, &[(f0, 1.0), (2.0 * f0, 0.1)]);
        assert!((spec.thd(f0, 5) - 0.1).abs() < 0.01, "THD ≈ 0.1");
        let h = spec.harmonics_db(f0, 2);
        assert!((h[0] - 0.0).abs() < 0.1, "fundamental ≈ 0 dB");
        assert!((h[1] - (-20.0)).abs() < 1.0, "2nd harmonic ≈ -20 dB");
    }

    #[test]
    fn worst_alias_detects_an_inharmonic_partial() {
        let sr = 48_000.0;
        let f0 = bin_centered_f0(sr); // 375 Hz; harmonics at 375,750,1125,...
        // Plant a -40 dB inharmonic partial at a bin-centered 1234.5-ish freq
        // well away from any k*f0.
        let alias_hz = 210.0 * (sr / FFT_N as f32); // bin 210 ≈ 1230.5 Hz
        let spec = render_sines(sr, &[(f0, 1.0), (alias_hz, 0.01)]);
        let tol = 3.0 * spec.bin_hz;
        let wa = spec.worst_alias_db(f0, tol);
        assert!(wa < -30.0 && wa > -50.0, "worst alias ≈ -40 dB, got {wa}");
    }

    #[test]
    fn thd_is_robust_off_bin() {
        let sr = 48_000.0;
        // Deliberately NOT bin-centered: half a bin above bin 64.
        let f0 = 64.5 * (sr / FFT_N as f32);
        // fundamental 1.0, 2nd harmonic at 0.1 (also off-bin, at 2*f0).
        let spec = render_sines(sr, &[(f0, 1.0), (2.0 * f0, 0.1)]);
        let thd = spec.thd(f0, 5);
        assert!((thd - 0.1).abs() < 0.02, "off-bin THD should stay ~0.1, got {thd}");
    }

    #[test]
    fn pure_sine_has_low_thd_and_alias() {
        let sr = 48_000.0;
        let f0 = bin_centered_f0(sr);
        let spec = render_sines(sr, &[(f0, 1.0)]);
        assert!(spec.thd(f0, 5) < 1e-2, "pure sine THD ~ 0");
        assert!(spec.worst_alias_db(f0, 3.0 * spec.bin_hz) < -60.0);
    }
}
