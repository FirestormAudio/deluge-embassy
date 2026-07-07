//! Spectral analysis: render → Hann window → real FFT → linear magnitude bins,
//! plus relative metrics (Tasks 2–3). Built on `realfft` (an independent,
//! reference-grade FFT — the harness never measures with code under test).

use realfft::RealFftPlanner;

use crate::FFT_N;

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
}
