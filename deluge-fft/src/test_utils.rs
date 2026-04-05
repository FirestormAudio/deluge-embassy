use crate::complex::Complex;
use crate::trig::{TWO_PI, cos_f32, sin_f32};

/// Naïve O(N²) DFT — reference for tests only.
///
/// Computes twiddles inline so it is independent of the table layout.
pub fn dft<const N: usize>(buf: &[Complex; N]) -> [Complex; N] {
    let mut out = [Complex::ZERO; N];
    for k in 0..N {
        let mut sum = Complex::ZERO;
        for n in 0..N {
            // W_N^((k*n) % N) = exp(-2πi·(k*n mod N)/N)
            // Modulo keeps the angle in (-2π, 0], equivalent by periodicity.
            let idx = (k * n) % N;
            let angle = -TWO_PI * idx as f64 / N as f64;
            let w = Complex {
                re: cos_f32(angle),
                im: sin_f32(angle),
            };
            sum = sum + buf[n] * w;
        }
        out[k] = sum;
    }
    out
}

/// Maximum absolute error between two complex arrays.
pub fn max_error<const N: usize>(a: &[Complex; N], b: &[Complex; N]) -> f32 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| {
            (Complex {
                re: x.re - y.re,
                im: x.im - y.im,
            })
            .abs()
        })
        .fold(0.0f32, f32::max)
}
