use crate::buf::FftBuf;
use crate::complex::Complex;
use crate::trig::{TWO_PI, cos_f32};

// ---------------------------------------------------------------------------
// Compile-time Hann window table
// ---------------------------------------------------------------------------

/// Compile-time Hann window coefficients for a length-`N` window.
///
/// Computed once at compile time using the same `const fn` trig as the
/// twiddle tables — zero runtime cosine evaluations.
struct HannTable<const N: usize>;

impl<const N: usize> HannTable<N> {
    const COEFFS: [f32; N] = {
        let mut t = [0.0f32; N];
        let mut i = 0usize;
        while i < N {
            t[i] = (0.5 * (1.0 - cos_f32(TWO_PI * i as f64 / N as f64) as f64)) as f32;
            i += 1;
        }
        t
    };
}

// ---------------------------------------------------------------------------

/// Write `|bin|` for the first `N/2 + 1` bins into `out` (length ≥ `N/2 + 1`).
pub fn magnitude_spectrum<const N: usize>(fft_out: &[Complex; N], out: &mut [f32]) {
    let bins = (N / 2 + 1).min(out.len());
    for i in 0..bins {
        out[i] = fft_out[i].abs();
    }
}

/// Apply a Hann window to an AoS `[Complex; N]` buffer in-place before FFT.
pub fn apply_hann_window<const N: usize>(buf: &mut [Complex; N]) {
    for (s, &w) in buf.iter_mut().zip(HannTable::<N>::COEFFS.iter()) {
        s.re *= w;
        s.im *= w;
    }
}

/// Write `|bin|` for the first `N/2 + 1` bins from an `FftBuf<N>`.
pub fn magnitude_spectrum_soa<const N: usize>(buf: &FftBuf<N>, out: &mut [f32]) {
    let bins = (N / 2 + 1).min(out.len());
    for (i, out_val) in out.iter_mut().enumerate().take(bins) {
        let re = buf.re[i];
        let im = buf.im[i];
        *out_val = crate::complex::sqrt_f32_pub(re * re + im * im);
    }
}

/// Apply a Hann window to an `FftBuf<N>` in-place before FFT.
pub fn apply_hann_window_soa<const N: usize>(buf: &mut FftBuf<N>) {
    for (i, &w) in HannTable::<N>::COEFFS.iter().enumerate() {
        buf.re[i] *= w;
        buf.im[i] *= w;
    }
}

/// Apply a Hann window to a real `[f32; N]` buffer in-place.
///
/// Use this before passing real samples to `RealFft::process`.
pub fn apply_hann_window_real<const N: usize>(buf: &mut [f32; N]) {
    for (val, &w) in buf.iter_mut().zip(HannTable::<N>::COEFFS.iter()) {
        *val *= w;
    }
}
