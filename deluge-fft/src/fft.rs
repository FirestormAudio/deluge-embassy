use core::ops::{Add, Mul, Sub};
use core::simd::Simd;

use crate::buf::FftBuf;
use crate::complex::Complex;
use crate::twiddle::TwiddleTable;

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

const fn assert_valid_size<const N: usize>() {
    assert!(
        N >= 2 && N.is_power_of_two(),
        "FFT size N must be a power of two >= 2"
    );
}

pub(crate) const fn assert_valid_size_r4<const N: usize>() {
    assert!(
        N >= 4 && N.is_power_of_two(),
        "Radix-4 FFT size N must be a power of two >= 4"
    );
}

const fn bit_rev(mut i: usize, mut bits: u32) -> usize {
    let mut r = 0usize;
    while bits > 0 {
        r = (r << 1) | (i & 1);
        i >>= 1;
        bits -= 1;
    }
    r
}

pub(crate) const fn ilog2(mut n: usize) -> usize {
    let mut r = 0;
    while n > 1 {
        n >>= 1;
        r += 1;
    }
    r
}

fn bit_reverse_permutation_aos<const N: usize>(buf: &mut [Complex; N]) {
    let bits = ilog2(N) as u32;
    for i in 0..N {
        let j = bit_rev(i, bits);
        if j > i {
            buf.swap(i, j);
        }
    }
}

fn bit_reverse_permutation_soa<const N: usize>(buf: &mut FftBuf<N>) {
    let bits = ilog2(N) as u32;
    for i in 0..N {
        let j = bit_rev(i, bits);
        if j > i {
            buf.re.swap(i, j);
            buf.im.swap(i, j);
        }
    }
}

// ---------------------------------------------------------------------------
// Fft
// ---------------------------------------------------------------------------

/// In-place radix-2 DIT FFT over `N` complex `f32` samples.
///
/// Twiddle factors are evaluated entirely at compile time and stored in ROM.
/// The hot loop only reads a flat table and performs arithmetic — zero runtime
/// trig cost.
///
/// Valid `LANES` values: `1`, `2`, `4`, `8`.
///
/// # Methods
///
/// | Method | Buffer layout | SIMD |
/// |--------|--------------|------|
/// | `process` | AoS `[Complex; N]` | scalar |
/// | `process_simd` | AoS `[Complex; N]` | LANES-wide |
/// | `process_soa` | `FftBuf<N>` (SoA) | scalar |
/// | `process_simd_soa` | `FftBuf<N>` (SoA) | LANES-wide, sequential loads |
pub struct Fft<const N: usize, const LANES: usize>;

impl<const N: usize, const LANES: usize> Fft<N, LANES>
where
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
{
    // -----------------------------------------------------------------------
    // AoS paths  (interleaved [Complex; N] buffer)
    // -----------------------------------------------------------------------

    /// Scalar in-place forward FFT on an AoS `[Complex; N]` buffer.
    pub fn process(buf: &mut [Complex; N]) {
        assert_valid_size::<N>();
        bit_reverse_permutation_aos(buf);
        let mut half_m = 1usize;
        while half_m < N {
            let tw_off = half_m - 1;
            let mut k = 0usize;
            while k < N {
                for j in 0..half_m {
                    let wr = TwiddleTable::<N>::re(tw_off + j);
                    let wi = TwiddleTable::<N>::im(tw_off + j);
                    let u = buf[k + j];
                    let v = buf[k + j + half_m];
                    let tr = wr * v.re - wi * v.im;
                    let ti = wr * v.im + wi * v.re;
                    buf[k + j] = Complex {
                        re: u.re + tr,
                        im: u.im + ti,
                    };
                    buf[k + j + half_m] = Complex {
                        re: u.re - tr,
                        im: u.im - ti,
                    };
                }
                k += half_m * 2;
            }
            half_m *= 2;
        }
    }

    /// SIMD-accelerated in-place forward FFT on an AoS `[Complex; N]` buffer.
    ///
    /// Vectorises across `LANES` independent j-values per stage.
    /// Falls back to scalar for stages where `half_m < LANES`.
    pub fn process_simd(buf: &mut [Complex; N]) {
        assert_valid_size::<N>();
        bit_reverse_permutation_aos(buf);
        let mut half_m = 1usize;
        while half_m < N {
            Self::simd_stage_aos(buf, half_m);
            half_m *= 2;
        }
    }

    fn simd_stage_aos(buf: &mut [Complex; N], half_m: usize) {
        let tw_off = half_m - 1;

        // For narrow stages (half_m < LANES) vectorise across multiple groups.
        // Pack groups_per_vec = LANES/half_m consecutive groups into one vector.
        if half_m < LANES {
            let stride = 2 * half_m;
            let groups_per_vec = LANES / half_m;

            let wr_arr: [f32; LANES] =
                core::array::from_fn(|l| TwiddleTable::<N>::re(tw_off + l % half_m));
            let wi_arr: [f32; LANES] =
                core::array::from_fn(|l| TwiddleTable::<N>::im(tw_off + l % half_m));
            let wr = Simd::<f32, LANES>::from_array(wr_arr);
            let wi = Simd::<f32, LANES>::from_array(wi_arr);

            let mut k = 0usize;
            while k + stride * groups_per_vec <= N {
                let mut ur_arr = [0f32; LANES];
                let mut ui_arr = [0f32; LANES];
                let mut vr_arr = [0f32; LANES];
                let mut vi_arr = [0f32; LANES];
                for g in 0..groups_per_vec {
                    let base = k + g * stride;
                    for j in 0..half_m {
                        let l = g * half_m + j;
                        let u = buf[base + j];
                        let v = buf[base + j + half_m];
                        ur_arr[l] = u.re;
                        ui_arr[l] = u.im;
                        vr_arr[l] = v.re;
                        vi_arr[l] = v.im;
                    }
                }
                let ur = Simd::<f32, LANES>::from_array(ur_arr);
                let ui = Simd::<f32, LANES>::from_array(ui_arr);
                let vr = Simd::<f32, LANES>::from_array(vr_arr);
                let vi = Simd::<f32, LANES>::from_array(vi_arr);

                let tr = wr * vr - wi * vi;
                let ti = wr * vi + wi * vr;
                let oar = (ur + tr).to_array();
                let oai = (ui + ti).to_array();
                let obr = (ur - tr).to_array();
                let obi = (ui - ti).to_array();
                for g in 0..groups_per_vec {
                    let base = k + g * stride;
                    for j in 0..half_m {
                        let l = g * half_m + j;
                        buf[base + j] = Complex {
                            re: oar[l],
                            im: oai[l],
                        };
                        buf[base + j + half_m] = Complex {
                            re: obr[l],
                            im: obi[l],
                        };
                    }
                }
                k += stride * groups_per_vec;
            }
            // Scalar tail for remaining groups.
            while k < N {
                for j in 0..half_m {
                    let wr = TwiddleTable::<N>::re(tw_off + j);
                    let wi = TwiddleTable::<N>::im(tw_off + j);
                    let u = buf[k + j];
                    let v = buf[k + j + half_m];
                    let tr = wr * v.re - wi * v.im;
                    let ti = wr * v.im + wi * v.re;
                    buf[k + j] = Complex {
                        re: u.re + tr,
                        im: u.im + ti,
                    };
                    buf[k + j + half_m] = Complex {
                        re: u.re - tr,
                        im: u.im - ti,
                    };
                }
                k += half_m * 2;
            }
            return;
        }

        let mut k = 0usize;
        while k < N {
            let mut j = 0usize;

            while j + LANES <= half_m {
                let wr = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                    TwiddleTable::<N>::re(tw_off + j + l)
                }));
                let wi = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                    TwiddleTable::<N>::im(tw_off + j + l)
                }));

                // AoS buf: deinterleave LANES complex values into re/im vectors
                // using simd_swizzle (→ vuzp.32 on NEON).  This avoids the
                // scalar extraction loop and generates a single load+unzip.
                let raw_u: Simd<f32, LANES>;
                let raw_ui: Simd<f32, LANES>;
                let raw_v: Simd<f32, LANES>;
                let raw_vi: Simd<f32, LANES>;
                {
                    let mut ur_arr = [0f32; LANES];
                    let mut ui_arr = [0f32; LANES];
                    let mut vr_arr = [0f32; LANES];
                    let mut vi_arr = [0f32; LANES];
                    for l in 0..LANES {
                        let u = buf[k + j + l];
                        let v = buf[k + j + half_m + l];
                        ur_arr[l] = u.re;
                        ui_arr[l] = u.im;
                        vr_arr[l] = v.re;
                        vi_arr[l] = v.im;
                    }
                    raw_u = Simd::from_array(ur_arr);
                    raw_ui = Simd::from_array(ui_arr);
                    raw_v = Simd::from_array(vr_arr);
                    raw_vi = Simd::from_array(vi_arr);
                }
                let ur = raw_u;
                let ui = raw_ui;
                let vr = raw_v;
                let vi = raw_vi;

                let tr = wr * vr - wi * vi;
                let ti = wr * vi + wi * vr;

                let oar = (ur + tr).to_array();
                let oai = (ui + ti).to_array();
                let obr = (ur - tr).to_array();
                let obi = (ui - ti).to_array();
                for l in 0..LANES {
                    buf[k + j + l] = Complex {
                        re: oar[l],
                        im: oai[l],
                    };
                    buf[k + j + half_m + l] = Complex {
                        re: obr[l],
                        im: obi[l],
                    };
                }

                j += LANES;
            }

            // Scalar tail.
            while j < half_m {
                let wr = TwiddleTable::<N>::re(tw_off + j);
                let wi = TwiddleTable::<N>::im(tw_off + j);
                let u = buf[k + j];
                let v = buf[k + j + half_m];
                let tr = wr * v.re - wi * v.im;
                let ti = wr * v.im + wi * v.re;
                buf[k + j] = Complex {
                    re: u.re + tr,
                    im: u.im + ti,
                };
                buf[k + j + half_m] = Complex {
                    re: u.re - tr,
                    im: u.im - ti,
                };
                j += 1;
            }

            k += half_m * 2;
        }
    }

    // -----------------------------------------------------------------------
    // SoA paths  (FftBuf<N> — separate re[] and im[] arrays)
    // -----------------------------------------------------------------------

    /// Scalar in-place forward FFT on an `FftBuf<N>` (SoA) buffer.
    pub fn process_soa(buf: &mut FftBuf<N>) {
        assert_valid_size::<N>();
        bit_reverse_permutation_soa(buf);
        let mut half_m = 1usize;
        while half_m < N {
            let tw_off = half_m - 1;
            let mut k = 0usize;
            while k < N {
                for j in 0..half_m {
                    let wr = TwiddleTable::<N>::re(tw_off + j);
                    let wi = TwiddleTable::<N>::im(tw_off + j);
                    let ur = buf.re[k + j];
                    let ui = buf.im[k + j];
                    let vr = buf.re[k + j + half_m];
                    let vi = buf.im[k + j + half_m];
                    let tr = wr * vr - wi * vi;
                    let ti = wr * vi + wi * vr;
                    buf.re[k + j] = ur + tr;
                    buf.im[k + j] = ui + ti;
                    buf.re[k + j + half_m] = ur - tr;
                    buf.im[k + j + half_m] = ui - ti;
                }
                k += half_m * 2;
            }
            half_m *= 2;
        }
    }

    /// SIMD-accelerated in-place forward FFT on an `FftBuf<N>` (SoA) buffer.
    ///
    /// All twiddle and sample memory operations are sequential contiguous
    /// vector loads/stores — maximally vectorisation-friendly.
    pub fn process_simd_soa(buf: &mut FftBuf<N>)
    where
        [(); 2 * N]:,
    {
        assert_valid_size::<N>();
        bit_reverse_permutation_soa(buf);
        let mut half_m = 1usize;
        while half_m < N {
            Self::simd_stage_soa(buf, half_m);
            half_m *= 2;
        }
    }

    fn simd_stage_soa(buf: &mut FftBuf<N>, half_m: usize)
    where
        [(); 2 * N]:,
    {
        let tw_off = half_m - 1;

        // For narrow stages (half_m < LANES) in the SoA layout, standard
        // sequential loads within a group produce fewer than LANES elements.
        // Instead vectorise across LANES consecutive groups at once.
        // Each group contributes `half_m` independent twiddle applications; we
        // tile twiddles with period half_m and process LANES groups per vector.
        //
        // Memory layout example (half_m=1, after bit-reversal):
        //   re[] = [u0, v0, u1, v1, u2, v2, u3, v3, ...]
        //   im[] = [same pattern]
        // With LANES=4: load re[k..k+8] and deinterleave even (u) / odd (v)
        // → vuzp.32 on NEON, or vpunpckl/h on x86.
        if half_m < LANES {
            // NEON fast path: use vld2q_f32 / vst2q_f32 for half_m=1 and
            // vld1q_f32 + vcombine for half_m=2 instead of scalar gather loops.
            #[cfg(all(target_arch = "arm", target_feature = "neon"))]
            if LANES == 4 {
                unsafe {
                    crate::arch::r2_soa_narrow_neon::<N>(buf, tw_off, half_m);
                }
                return;
            }

            // groups_per_vec: how many complete groups fit in LANES elements.
            // E.g. half_m=1, LANES=4 → groups_per_vec=4.
            // stride = 2*half_m (elements per group in flat SoA).
            let stride = 2 * half_m; // elements per group
            let groups_per_vec = LANES / half_m; // always a power of 2

            // Build twiddle vectors (period half_m, so repeat as needed).
            let wr_arr: [f32; LANES] =
                core::array::from_fn(|l| TwiddleTable::<N>::re(tw_off + l % half_m));
            let wi_arr: [f32; LANES] =
                core::array::from_fn(|l| TwiddleTable::<N>::im(tw_off + l % half_m));
            let wr = Simd::<f32, LANES>::from_array(wr_arr);
            let wi = Simd::<f32, LANES>::from_array(wi_arr);

            // Build index tables for the gather (compile-time constant,
            // groups_per_vec ≤ LANES so the loops are trivially unrolled).
            // u_idx[l] = position of the l-th upper element in re[].
            // v_idx[l] = position of the l-th lower element in re[].
            // For half_m=1: u_idx=[0,2,4,6], v_idx=[1,3,5,7].
            let mut k = 0usize;
            while k + stride * groups_per_vec <= N {
                let mut ur_arr = [0f32; LANES];
                let mut ui_arr = [0f32; LANES];
                let mut vr_arr = [0f32; LANES];
                let mut vi_arr = [0f32; LANES];
                for g in 0..groups_per_vec {
                    let base = k + g * stride;
                    for j in 0..half_m {
                        let l = g * half_m + j;
                        ur_arr[l] = buf.re[base + j];
                        ui_arr[l] = buf.im[base + j];
                        vr_arr[l] = buf.re[base + j + half_m];
                        vi_arr[l] = buf.im[base + j + half_m];
                    }
                }
                let ur = Simd::<f32, LANES>::from_array(ur_arr);
                let ui = Simd::<f32, LANES>::from_array(ui_arr);
                let vr = Simd::<f32, LANES>::from_array(vr_arr);
                let vi = Simd::<f32, LANES>::from_array(vi_arr);

                let tr = wr * vr - wi * vi;
                let ti = wr * vi + wi * vr;
                let oar = (ur + tr).to_array();
                let oai = (ui + ti).to_array();
                let obr = (ur - tr).to_array();
                let obi = (ui - ti).to_array();

                for g in 0..groups_per_vec {
                    let base = k + g * stride;
                    for j in 0..half_m {
                        let l = g * half_m + j;
                        buf.re[base + j] = oar[l];
                        buf.im[base + j] = oai[l];
                        buf.re[base + j + half_m] = obr[l];
                        buf.im[base + j + half_m] = obi[l];
                    }
                }
                k += stride * groups_per_vec;
            }
            // Scalar tail.
            while k < N {
                for j in 0..half_m {
                    let wr = TwiddleTable::<N>::re(tw_off + j);
                    let wi = TwiddleTable::<N>::im(tw_off + j);
                    let ur = buf.re[k + j];
                    let ui = buf.im[k + j];
                    let vr = buf.re[k + j + half_m];
                    let vi = buf.im[k + j + half_m];
                    let tr = wr * vr - wi * vi;
                    let ti = wr * vi + wi * vr;
                    buf.re[k + j] = ur + tr;
                    buf.im[k + j] = ui + ti;
                    buf.re[k + j + half_m] = ur - tr;
                    buf.im[k + j + half_m] = ui - ti;
                }
                k += stride;
            }
            return;
        }

        // NEON wide path: vld2q_f32 loads 4 twiddle pairs in one instruction.
        #[cfg(all(target_arch = "arm", target_feature = "neon"))]
        if LANES == 4 {
            unsafe {
                crate::arch::r2_soa_wide_neon::<N>(buf, half_m);
            }
            return;
        }

        let mut k = 0usize;
        while k < N {
            let mut j = 0usize;

            while j + LANES <= half_m {
                let wr = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                    TwiddleTable::<N>::re(tw_off + j + l)
                }));
                let wi = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                    TwiddleTable::<N>::im(tw_off + j + l)
                }));
                let ur = Simd::<f32, LANES>::from_slice(&buf.re[k + j..]);
                let ui = Simd::<f32, LANES>::from_slice(&buf.im[k + j..]);
                let vr = Simd::<f32, LANES>::from_slice(&buf.re[k + j + half_m..]);
                let vi = Simd::<f32, LANES>::from_slice(&buf.im[k + j + half_m..]);

                let tr = wr * vr - wi * vi;
                let ti = wr * vi + wi * vr;

                // All 4 stores are sequential, aligned, contiguous.
                (ur + tr).copy_to_slice(&mut buf.re[k + j..]);
                (ui + ti).copy_to_slice(&mut buf.im[k + j..]);
                (ur - tr).copy_to_slice(&mut buf.re[k + j + half_m..]);
                (ui - ti).copy_to_slice(&mut buf.im[k + j + half_m..]);

                j += LANES;
            }

            // Scalar tail.
            while j < half_m {
                let wr = TwiddleTable::<N>::re(tw_off + j);
                let wi = TwiddleTable::<N>::im(tw_off + j);
                let ur = buf.re[k + j];
                let ui = buf.im[k + j];
                let vr = buf.re[k + j + half_m];
                let vi = buf.im[k + j + half_m];
                let tr = wr * vr - wi * vi;
                let ti = wr * vi + wi * vr;
                buf.re[k + j] = ur + tr;
                buf.im[k + j] = ui + ti;
                buf.re[k + j + half_m] = ur - tr;
                buf.im[k + j + half_m] = ui - ti;
                j += 1;
            }

            k += half_m * 2;
        }
    }
}
