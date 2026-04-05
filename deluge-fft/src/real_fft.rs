// ---------------------------------------------------------------------------
// Real-input FFT
// ---------------------------------------------------------------------------
//
// Algorithm (half-size complex trick)
// ------------------------------------
// For N real samples x[0..N], the DFT has conjugate symmetry X[N-k] = X[k]*.
// We exploit this by packing x[] into a half-size complex array:
//
//   z[k] = x[2k] + j·x[2k+1],   k = 0 .. N/2-1
//
// Running a length-N/2 complex FFT on z gives Z[].  Then for k = 0 .. N/2:
//
//   X[k] = (Z[k] + conj(Z[N/2 - k])) / 2
//         + W_N^(-k) · (Z[k] - conj(Z[N/2 - k])) / (2j)
//
// where W_N^k = exp(-2πi·k/N).  Rearranging the /2j into real operations:
//
//   let a = (Z[k] + conj(Z[N/2-k])) / 2;          // = (ar, ai)
//   let d = (Z[k] - conj(Z[N/2-k])) / 2;          // = (dr, di)
//   // -j * d = (di, -dr)
//   // X[k] = a + W_N^k * (-j * d)
//   //   td_r = wr*di + wi*dr
//   //   td_i = wi*di - wr*dr
//   //   (where W_N^k = wr + j*wi, wi < 0 for 0 < k < N/2)
//   X[k] = (a.re + td_r, a.im + td_i)
//
// Special cases:
//   X[0]   = Z[0].re + Z[0].im  (purely real)
//   X[N/2] = Z[0].re - Z[0].im  (purely real)
//
// Output: N/2 + 1 complex bins stored in `out[0..=N/2]`.
// This requires #![feature(generic_const_exprs)] for the array output size.

use core::ops::{Add, Mul, Sub};
use core::simd::Simd;

use crate::buf::FftBuf;
use crate::twiddle::TwiddleTable;

/// Real-input FFT: processes `N` real `f32` samples and produces the
/// `N/2 + 1` independent complex frequency bins.
///
/// Internally runs a length-`N/2` complex FFT (half the work), then applies
/// a post-processing step to recover the full spectrum from conjugate symmetry.
///
/// `N` must be a power of two ≥ 4.
///
/// # Performance
///
/// Equivalent cost to `Fft::<{N/2}, LANES>::process_simd_soa` plus O(N) post-
/// processing.  Approximately 2× faster than a length-N complex FFT for pure
/// real input.
pub struct RealFft<const N: usize, const LANES: usize>;

impl<const N: usize, const LANES: usize> RealFft<N, LANES>
where
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
    [(); N]:,
    [(); 2 * (N / 2)]:,
{
    /// Process `N` real samples and write `N/2 + 1` complex bins to `out`.
    ///
    /// `out` must have length ≥ `N/2 + 1`.
    pub fn process(input: &[f32; N], out: &mut [crate::complex::Complex; N / 2 + 1]) {
        assert!(
            N >= 8 && N.is_power_of_two(),
            "RealFft N must be a power of two >= 8"
        );

        // Half-size = N/2.
        const fn half<const N: usize>() -> usize {
            N / 2
        }
        let hn = half::<N>();

        // 1. Pack real samples into a half-size SoA buffer.
        //    z[k] = x[2k] + j*x[2k+1]
        let mut zbuf = FftBuf::<{ N / 2 }>::ZERO;
        for k in 0..hn {
            zbuf.re[k] = input[2 * k];
            zbuf.im[k] = input[2 * k + 1];
        }

        // 2. Run the half-size complex FFT using the radix-4 path (half passes).
        crate::radix4::process_r4_simd_soa::<{ N / 2 }, LANES>(&mut zbuf);

        // 3. Special bins.
        // Z[0] = zbuf.re[0] + j*zbuf.im[0]
        // X[0]    = Z[0].re + Z[0].im  (real part of real signal's DC)
        // X[N/2]  = Z[0].re - Z[0].im
        let z0r = zbuf.re[0];
        let z0i = zbuf.im[0];
        out[0] = crate::complex::Complex {
            re: z0r + z0i,
            im: 0.0,
        };
        out[hn] = crate::complex::Complex {
            re: z0r - z0i,
            im: 0.0,
        };

        // 4. General bins k = 1 .. N/2-1.
        //    The twiddle table TwiddleTable::<N> holds W_N^k.
        //    For the post-processing we need W_N^k for k = 1..N/2-1.
        //    Those live at the *last* stage's twiddle sub-table: half_m = N/2,
        //    offset = N/2 - 1,  W_N^j = entry [N/2-1 + j] (j = 0..N/2).
        //    But our table is size N, so the last stage is for half_m = N/2,
        //    covering j = 0..N/2-1 — perfect.
        let tw_off = hn - 1;

        for (k, out_k) in out.iter_mut().enumerate().take(hn).skip(1) {
            let nk = hn - k; // index of the conjugate partner in Z

            let zkr = zbuf.re[k];
            let zki = zbuf.im[k];
            let znkr = zbuf.re[nk];
            let znki = zbuf.im[nk];

            // a = (Z[k] + conj(Z[nk])) * 0.5
            let ar = (zkr + znkr) * 0.5;
            let ai = (zki - znki) * 0.5;

            // d = (Z[k] - conj(Z[nk])) * 0.5
            let dr = (zkr - znkr) * 0.5;
            let di = (zki + znki) * 0.5;

            // -j * d = (di, -dr)   then multiply by W_N^k = (wr, wi)
            //   note: we want W_N^(-k) = (wr, -wi) but applied as:
            // X[k] = a + W_N^k * (-j * d)
            //
            //   W_N^k = (wr, wi)   [stored in table, wi = sin(-2πk/N) < 0 for 0<k<N/2]
            //   -j * d = (di, -dr)
            //   product (wr + j*wi)(di + j*(-dr)):
            //     re = wr*di - wi*(-dr) = wr*di + wi*dr
            //     im = wr*(-dr) + wi*di = wi*di - wr*dr
            let wr = TwiddleTable::<N>::re(tw_off + k);
            let wi = TwiddleTable::<N>::im(tw_off + k); // W_N^k, wi < 0

            // W_N^k * (-j * d):
            let td_r = wr * di + wi * dr;
            let td_i = wi * di - wr * dr;

            *out_k = crate::complex::Complex {
                re: ar + td_r,
                im: ai + td_i,
            };
            // X[N-k] = conj(X[k]), but since we only output 0..N/2+1 that's
            // the same as X[N/2-(k-(N/2))] = conj(X[k]); we don't store it.
        }
    }
}
