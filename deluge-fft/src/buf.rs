use crate::complex::Complex;

/// Split-of-arrays (SoA) buffer for `N` complex `f32` samples.
///
/// Keeping real and imaginary parts in separate contiguous arrays lets the
/// SIMD butterfly path load/store with a single `from_slice` / `copy_to_slice`
/// per component — no deinterleave overhead.
///
/// # Typical usage
///
/// ```ignore
/// // Real audio input:
/// let mut buf = FftBuf::from_real(&samples);
/// apply_hann_window_soa(&mut buf);
/// Fft::<512, 4>::process_simd_soa(&mut buf);
/// magnitude_spectrum_soa(&buf, &mut magnitudes);
/// ```
#[derive(Clone)]
pub struct FftBuf<const N: usize> {
    pub re: [f32; N],
    pub im: [f32; N],
}

impl<const N: usize> FftBuf<N> {
    /// All-zeros buffer.
    pub const ZERO: Self = Self {
        re: [0.0; N],
        im: [0.0; N],
    };

    /// Load real samples; imaginary parts are set to zero.
    #[inline]
    pub fn from_real(samples: &[f32; N]) -> Self {
        Self {
            re: *samples,
            im: [0.0; N],
        }
    }

    /// Convert to the interleaved `[Complex; N]` layout.
    #[inline]
    pub fn to_complex(&self) -> [Complex; N] {
        let mut out = [Complex::ZERO; N];
        for (i, out_elem) in out.iter_mut().enumerate() {
            *out_elem = Complex {
                re: self.re[i],
                im: self.im[i],
            };
        }
        out
    }

    /// Load from interleaved `[Complex; N]`.
    #[inline]
    pub fn from_complex(src: &[Complex; N]) -> Self {
        let mut buf = Self::ZERO;
        for (i, s) in src.iter().enumerate() {
            buf.re[i] = s.re;
            buf.im[i] = s.im;
        }
        buf
    }
}
