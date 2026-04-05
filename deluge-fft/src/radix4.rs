// ---------------------------------------------------------------------------
// Radix-4 DIT FFT (merged two-stage Cooley-Tukey)
// ---------------------------------------------------------------------------
//
// Theory
// ------
// We merge two consecutive DIT radix-2 stages (A and B) into one pass.
//
// Given 4 elements at positions {p0, p1, p2, p3} = {k+j, k+j+qm, k+j+2qm, k+j+3qm}
// in bit-reversed order, the merged computation is:
//
//   Stage A (half_m = qm, span = 2qm, twiddle w2 = W_{2qm}^j):
//     a = x0 + w2*x1
//     b = x0 - w2*x1
//     c = x2 + w2*x3
//     d = x2 - w2*x3
//
//   Stage B (half_m = 2qm, span = 4qm, twiddle w1 = W_{4qm}^j):
//     y0 = a + w1*c   → position p0
//     y1 = b - j*w1*d → position p1    (j-rotation since W_{4qm}^{j+qm} = -j * W_{4qm}^j)
//     y2 = a - w1*c   → position p2
//     y3 = b + j*w1*d → position p3
//
// where:
//   w2 = W_{2qm}^j  from stage A sub-table: offset (qm-1), index j
//   w1 = W_{4qm}^j  from stage B sub-table: offset (2qm-1), index j
//
// Total: 4 complex multiplies per butterfly (3 muls for 4 outputs — one j-rotation is free).
// Passes: log₄N for even log₂N, 1 initial r2 + log₄(N/2) for odd log₂N.

use core::ops::{Add, Mul, Sub};
use core::simd::Simd;

use crate::buf::FftBuf;
use crate::complex::Complex;
use crate::fft::{assert_valid_size_r4, ilog2};
use crate::twiddle::TwiddleTable;

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

/// Radix-4 DIT in-place FFT on an AoS `[Complex; N]` buffer, `LANES`-wide SIMD.
pub fn process_r4_simd<const N: usize, const LANES: usize>(buf: &mut [Complex; N])
where
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
{
    assert_valid_size_r4::<N>();
    bit_reverse_aos(buf);

    let log2n = ilog2(N);
    let mut stage = 0usize;

    // If log₂N is odd, handle stage 0 (half_m=1) alone as radix-2.
    if log2n & 1 == 1 {
        r2_stage_aos::<N, LANES>(buf, 1);
        stage = 1;
    }

    // Merge stage pairs as radix-4.
    while stage + 1 < log2n {
        let qm = 1usize << stage; // half_m of stage A = 2^stage
        r4_stage_aos::<N, LANES>(buf, qm);
        stage += 2;
    }
}

/// Radix-4 DIT in-place FFT on an `FftBuf<N>` (SoA) buffer, `LANES`-wide SIMD.
pub fn process_r4_simd_soa<const N: usize, const LANES: usize>(buf: &mut FftBuf<N>)
where
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
    [(); 2 * N]:,
{
    assert_valid_size_r4::<N>();
    bit_reverse_soa(buf);

    let log2n = ilog2(N);
    let mut stage = 0usize;

    if log2n & 1 == 1 {
        r2_stage_soa::<N, LANES>(buf, 1);
        stage = 1;
    }

    while stage + 1 < log2n {
        let qm = 1usize << stage;
        r4_stage_soa::<N, LANES>(buf, qm);
        stage += 2;
    }
}

// ---------------------------------------------------------------------------
// Bit-reversal
// ---------------------------------------------------------------------------

fn bit_rev(mut i: usize, mut bits: u32) -> usize {
    let mut r = 0usize;
    while bits > 0 {
        r = (r << 1) | (i & 1);
        i >>= 1;
        bits -= 1;
    }
    r
}

fn bit_reverse_aos<const N: usize>(buf: &mut [Complex; N]) {
    let bits = ilog2(N) as u32;
    for i in 0..N {
        let j = bit_rev(i, bits);
        if j > i {
            buf.swap(i, j);
        }
    }
}

pub(crate) fn bit_reverse_soa<const N: usize>(buf: &mut FftBuf<N>) {
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
// Radix-2 single stage
// ---------------------------------------------------------------------------

fn r2_stage_aos<const N: usize, const LANES: usize>(buf: &mut [Complex; N], half_m: usize)
where
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
{
    let tw_off = half_m - 1;
    // For half_m < LANES, vectorise across LANES groups at once.
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
            k += stride;
        }
        return;
    }
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
}

pub(crate) fn r2_stage_soa<const N: usize, const LANES: usize>(buf: &mut FftBuf<N>, half_m: usize)
where
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
{
    let tw_off = half_m - 1;
    // For half_m < LANES, vectorise across LANES groups simultaneously.
    if half_m < LANES {
        #[cfg(all(target_arch = "arm", target_feature = "neon"))]
        if LANES == 4 {
            unsafe {
                crate::arch::r2_soa_narrow_neon::<N>(buf, tw_off, half_m);
            }
            return;
        }

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
}

// ---------------------------------------------------------------------------
// Merged two-stage radix-4 butterfly (AoS)
// ---------------------------------------------------------------------------
//
// `qm` = half_m of stage A = 2^stage.
// Group span = 4*qm.  Within each group, j = 0..qm:
//
//   Stage A twiddle: w2 = W_{2qm}^j  → SoA table offset (qm - 1), index j
//   Stage B twiddle: w1 = W_{4qm}^j  → SoA table offset (2qm - 1), index j
//
//   a = x0 + w2*x1,  b = x0 - w2*x1
//   c = x2 + w2*x3,  d = x2 - w2*x3
//   y0 = a + w1*c,   y2 = a - w1*c
//   y1 = b - j*w1*d, y3 = b + j*w1*d
//
//   Written to positions p0, p1=p0+qm, p2=p0+2qm, p3=p0+3qm.

fn r4_stage_aos<const N: usize, const LANES: usize>(buf: &mut [Complex; N], qm: usize)
where
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
{
    // SoA table offsets.
    let tw2_off = qm - 1; // W_{2qm}^j: stage A (half_m = qm)
    let tw1_off = 2 * qm - 1; // W_{4qm}^j: stage B (half_m = 2qm)
    let m4 = 4 * qm; // group span

    let mut k = 0usize;
    while k < N {
        let mut j = 0usize;

        // SIMD path.
        while j + LANES <= qm {
            // Twiddles — two sequential contiguous loads each.
            let w2r = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                TwiddleTable::<N>::re(tw2_off + j + l)
            }));
            let w2i = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                TwiddleTable::<N>::im(tw2_off + j + l)
            }));
            let w1r = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                TwiddleTable::<N>::re(tw1_off + j + l)
            }));
            let w1i = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                TwiddleTable::<N>::im(tw1_off + j + l)
            }));

            // Load x0..x3 (AoS: extract re/im from interleaved Complex).
            let (x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i) = {
                let mut x0r = [0f32; LANES];
                let mut x0i = [0f32; LANES];
                let mut x1r = [0f32; LANES];
                let mut x1i = [0f32; LANES];
                let mut x2r = [0f32; LANES];
                let mut x2i = [0f32; LANES];
                let mut x3r = [0f32; LANES];
                let mut x3i = [0f32; LANES];
                for l in 0..LANES {
                    let c0 = buf[k + j + l];
                    let c1 = buf[k + j + l + qm];
                    let c2 = buf[k + j + l + 2 * qm];
                    let c3 = buf[k + j + l + 3 * qm];
                    x0r[l] = c0.re;
                    x0i[l] = c0.im;
                    x1r[l] = c1.re;
                    x1i[l] = c1.im;
                    x2r[l] = c2.re;
                    x2i[l] = c2.im;
                    x3r[l] = c3.re;
                    x3i[l] = c3.im;
                }
                (
                    Simd::from_array(x0r),
                    Simd::from_array(x0i),
                    Simd::from_array(x1r),
                    Simd::from_array(x1i),
                    Simd::from_array(x2r),
                    Simd::from_array(x2i),
                    Simd::from_array(x3r),
                    Simd::from_array(x3i),
                )
            };

            // Stage A: butterfly within pairs {x0,x1} and {x2,x3}.
            // t1 = w2 * x1
            let t1r = w2r * x1r - w2i * x1i;
            let t1i = w2r * x1i + w2i * x1r;
            // t3 = w2 * x3
            let t3r = w2r * x3r - w2i * x3i;
            let t3i = w2r * x3i + w2i * x3r;

            let ar = x0r + t1r;
            let ai = x0i + t1i; // a = x0 + w2*x1
            let br = x0r - t1r;
            let bi = x0i - t1i; // b = x0 - w2*x1
            let cr = x2r + t3r;
            let ci = x2i + t3i; // c = x2 + w2*x3
            let dr = x2r - t3r;
            let di = x2i - t3i; // d = x2 - w2*x3

            // Stage B: butterfly across pairs {a,c} and {b,d}.
            // e = w1 * c
            let er = w1r * cr - w1i * ci;
            let ei = w1r * ci + w1i * cr;
            // f = w1 * d
            let fr = w1r * dr - w1i * di;
            let fi = w1r * di + w1i * dr;

            // y0 = a + e, y2 = a - e
            // y1 = b - j*f = (br + fi, bi - fr)   [since -j*(fr,fi) = (fi,-fr)]
            // y3 = b + j*f = (br - fi, bi + fr)
            let y0r = (ar + er).to_array();
            let y0i = (ai + ei).to_array();
            let y2r = (ar - er).to_array();
            let y2i = (ai - ei).to_array();
            let y1r = (br + fi).to_array();
            let y1i = (bi - fr).to_array();
            let y3r = (br - fi).to_array();
            let y3i = (bi + fr).to_array();

            for l in 0..LANES {
                buf[k + j + l] = Complex {
                    re: y0r[l],
                    im: y0i[l],
                };
                buf[k + j + l + qm] = Complex {
                    re: y1r[l],
                    im: y1i[l],
                };
                buf[k + j + l + 2 * qm] = Complex {
                    re: y2r[l],
                    im: y2i[l],
                };
                buf[k + j + l + 3 * qm] = Complex {
                    re: y3r[l],
                    im: y3i[l],
                };
            }

            j += LANES;
        }

        // Scalar tail.
        while j < qm {
            let w2r = TwiddleTable::<N>::re(tw2_off + j);
            let w2i = TwiddleTable::<N>::im(tw2_off + j);
            let w1r = TwiddleTable::<N>::re(tw1_off + j);
            let w1i = TwiddleTable::<N>::im(tw1_off + j);

            let x0 = buf[k + j];
            let x1 = buf[k + j + qm];
            let x2 = buf[k + j + 2 * qm];
            let x3 = buf[k + j + 3 * qm];

            let t1r = w2r * x1.re - w2i * x1.im;
            let t1i = w2r * x1.im + w2i * x1.re;
            let t3r = w2r * x3.re - w2i * x3.im;
            let t3i = w2r * x3.im + w2i * x3.re;

            let ar = x0.re + t1r;
            let ai = x0.im + t1i;
            let br = x0.re - t1r;
            let bi = x0.im - t1i;
            let cr = x2.re + t3r;
            let ci = x2.im + t3i;
            let dr = x2.re - t3r;
            let di = x2.im - t3i;

            let er = w1r * cr - w1i * ci;
            let ei = w1r * ci + w1i * cr;
            let fr = w1r * dr - w1i * di;
            let fi = w1r * di + w1i * dr;

            buf[k + j] = Complex {
                re: ar + er,
                im: ai + ei,
            };
            buf[k + j + qm] = Complex {
                re: br + fi,
                im: bi - fr,
            };
            buf[k + j + 2 * qm] = Complex {
                re: ar - er,
                im: ai - ei,
            };
            buf[k + j + 3 * qm] = Complex {
                re: br - fi,
                im: bi + fr,
            };

            j += 1;
        }

        k += m4;
    }
}

// ---------------------------------------------------------------------------
// Merged two-stage radix-4 butterfly (SoA)
// ---------------------------------------------------------------------------

pub(crate) fn r4_stage_soa<const N: usize, const LANES: usize>(buf: &mut FftBuf<N>, qm: usize)
where
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
    [(); 2 * N]:,
{
    let tw2_off = qm - 1;
    let tw1_off = 2 * qm - 1;
    let m4 = 4 * qm;

    // For qm < LANES, the j-loop never fills a SIMD vector.
    // Pack LANES/qm consecutive k-groups into one vector instead.
    if qm < LANES {
        #[cfg(all(target_arch = "arm", target_feature = "neon"))]
        if LANES == 4 {
            unsafe {
                crate::arch::r4_soa_narrow_neon::<N>(buf, qm, tw2_off, tw1_off);
            }
            return;
        }

        let groups_per_vec = LANES / qm;
        let w2r_arr: [f32; LANES] =
            core::array::from_fn(|l| TwiddleTable::<N>::re(tw2_off + l % qm));
        let w2i_arr: [f32; LANES] =
            core::array::from_fn(|l| TwiddleTable::<N>::im(tw2_off + l % qm));
        let w1r_arr: [f32; LANES] =
            core::array::from_fn(|l| TwiddleTable::<N>::re(tw1_off + l % qm));
        let w1i_arr: [f32; LANES] =
            core::array::from_fn(|l| TwiddleTable::<N>::im(tw1_off + l % qm));
        let w2r = Simd::<f32, LANES>::from_array(w2r_arr);
        let w2i = Simd::<f32, LANES>::from_array(w2i_arr);
        let w1r = Simd::<f32, LANES>::from_array(w1r_arr);
        let w1i = Simd::<f32, LANES>::from_array(w1i_arr);

        let mut k = 0usize;
        while k + m4 * groups_per_vec <= N {
            let mut x0r_a = [0f32; LANES];
            let mut x0i_a = [0f32; LANES];
            let mut x1r_a = [0f32; LANES];
            let mut x1i_a = [0f32; LANES];
            let mut x2r_a = [0f32; LANES];
            let mut x2i_a = [0f32; LANES];
            let mut x3r_a = [0f32; LANES];
            let mut x3i_a = [0f32; LANES];
            for g in 0..groups_per_vec {
                let base = k + g * m4;
                for j in 0..qm {
                    let l = g * qm + j;
                    x0r_a[l] = buf.re[base + j];
                    x0i_a[l] = buf.im[base + j];
                    x1r_a[l] = buf.re[base + j + qm];
                    x1i_a[l] = buf.im[base + j + qm];
                    x2r_a[l] = buf.re[base + j + 2 * qm];
                    x2i_a[l] = buf.im[base + j + 2 * qm];
                    x3r_a[l] = buf.re[base + j + 3 * qm];
                    x3i_a[l] = buf.im[base + j + 3 * qm];
                }
            }
            let x0r = Simd::<f32, LANES>::from_array(x0r_a);
            let x0i = Simd::<f32, LANES>::from_array(x0i_a);
            let x1r = Simd::<f32, LANES>::from_array(x1r_a);
            let x1i = Simd::<f32, LANES>::from_array(x1i_a);
            let x2r = Simd::<f32, LANES>::from_array(x2r_a);
            let x2i = Simd::<f32, LANES>::from_array(x2i_a);
            let x3r = Simd::<f32, LANES>::from_array(x3r_a);
            let x3i = Simd::<f32, LANES>::from_array(x3i_a);

            let t1r = w2r * x1r - w2i * x1i;
            let t1i = w2r * x1i + w2i * x1r;
            let t3r = w2r * x3r - w2i * x3i;
            let t3i = w2r * x3i + w2i * x3r;

            let ar = x0r + t1r;
            let ai = x0i + t1i;
            let br = x0r - t1r;
            let bi = x0i - t1i;
            let cr = x2r + t3r;
            let ci = x2i + t3i;
            let dr = x2r - t3r;
            let di = x2i - t3i;

            let er = w1r * cr - w1i * ci;
            let ei = w1r * ci + w1i * cr;
            let fr = w1r * dr - w1i * di;
            let fi = w1r * di + w1i * dr;

            let o0r = (ar + er).to_array();
            let o0i = (ai + ei).to_array();
            let o1r = (br + fi).to_array();
            let o1i = (bi - fr).to_array();
            let o2r = (ar - er).to_array();
            let o2i = (ai - ei).to_array();
            let o3r = (br - fi).to_array();
            let o3i = (bi + fr).to_array();

            for g in 0..groups_per_vec {
                let base = k + g * m4;
                for j in 0..qm {
                    let l = g * qm + j;
                    buf.re[base + j] = o0r[l];
                    buf.im[base + j] = o0i[l];
                    buf.re[base + j + qm] = o1r[l];
                    buf.im[base + j + qm] = o1i[l];
                    buf.re[base + j + 2 * qm] = o2r[l];
                    buf.im[base + j + 2 * qm] = o2i[l];
                    buf.re[base + j + 3 * qm] = o3r[l];
                    buf.im[base + j + 3 * qm] = o3i[l];
                }
            }
            k += m4 * groups_per_vec;
        }
        // scalar tail (shouldn't be reached for power-of-two N)
        while k < N {
            for j in 0..qm {
                let w2r = TwiddleTable::<N>::re(tw2_off + j);
                let w2i = TwiddleTable::<N>::im(tw2_off + j);
                let w1r = TwiddleTable::<N>::re(tw1_off + j);
                let w1i = TwiddleTable::<N>::im(tw1_off + j);
                let x0r = buf.re[k + j];
                let x0i = buf.im[k + j];
                let x1r = buf.re[k + j + qm];
                let x1i = buf.im[k + j + qm];
                let x2r = buf.re[k + j + 2 * qm];
                let x2i = buf.im[k + j + 2 * qm];
                let x3r = buf.re[k + j + 3 * qm];
                let x3i = buf.im[k + j + 3 * qm];
                let t1r = w2r * x1r - w2i * x1i;
                let t1i = w2r * x1i + w2i * x1r;
                let t3r = w2r * x3r - w2i * x3i;
                let t3i = w2r * x3i + w2i * x3r;
                let ar = x0r + t1r;
                let ai = x0i + t1i;
                let br = x0r - t1r;
                let bi = x0i - t1i;
                let cr = x2r + t3r;
                let ci = x2i + t3i;
                let dr = x2r - t3r;
                let di = x2i - t3i;
                let er = w1r * cr - w1i * ci;
                let ei = w1r * ci + w1i * cr;
                let fr = w1r * dr - w1i * di;
                let fi = w1r * di + w1i * dr;
                buf.re[k + j] = ar + er;
                buf.im[k + j] = ai + ei;
                buf.re[k + j + qm] = br + fi;
                buf.im[k + j + qm] = bi - fr;
                buf.re[k + j + 2 * qm] = ar - er;
                buf.im[k + j + 2 * qm] = ai - ei;
                buf.re[k + j + 3 * qm] = br - fi;
                buf.im[k + j + 3 * qm] = bi + fr;
            }
            k += m4;
        }
        return;
    }

    // NEON wide path: vld1q_f32 loads 4 consecutive twiddle values per register.
    #[cfg(all(target_arch = "arm", target_feature = "neon"))]
    if LANES == 4 {
        unsafe {
            crate::arch::r4_soa_wide_neon::<N>(buf, qm);
        }
        return;
    }

    let mut k = 0usize;
    while k < N {
        let mut j = 0usize;

        while j + LANES <= qm {
            let w2r = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                TwiddleTable::<N>::re(tw2_off + j + l)
            }));
            let w2i = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                TwiddleTable::<N>::im(tw2_off + j + l)
            }));
            let w1r = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                TwiddleTable::<N>::re(tw1_off + j + l)
            }));
            let w1i = Simd::<f32, LANES>::from_array(core::array::from_fn(|l| {
                TwiddleTable::<N>::im(tw1_off + j + l)
            }));

            // All 8 sample loads are sequential contiguous reads.
            let x0r = Simd::<f32, LANES>::from_slice(&buf.re[k + j..]);
            let x0i = Simd::<f32, LANES>::from_slice(&buf.im[k + j..]);
            let x1r = Simd::<f32, LANES>::from_slice(&buf.re[k + j + qm..]);
            let x1i = Simd::<f32, LANES>::from_slice(&buf.im[k + j + qm..]);
            let x2r = Simd::<f32, LANES>::from_slice(&buf.re[k + j + 2 * qm..]);
            let x2i = Simd::<f32, LANES>::from_slice(&buf.im[k + j + 2 * qm..]);
            let x3r = Simd::<f32, LANES>::from_slice(&buf.re[k + j + 3 * qm..]);
            let x3i = Simd::<f32, LANES>::from_slice(&buf.im[k + j + 3 * qm..]);

            let t1r = w2r * x1r - w2i * x1i;
            let t1i = w2r * x1i + w2i * x1r;
            let t3r = w2r * x3r - w2i * x3i;
            let t3i = w2r * x3i + w2i * x3r;

            let ar = x0r + t1r;
            let ai = x0i + t1i;
            let br = x0r - t1r;
            let bi = x0i - t1i;
            let cr = x2r + t3r;
            let ci = x2i + t3i;
            let dr = x2r - t3r;
            let di = x2i - t3i;

            let er = w1r * cr - w1i * ci;
            let ei = w1r * ci + w1i * cr;
            let fr = w1r * dr - w1i * di;
            let fi = w1r * di + w1i * dr;

            // All 8 stores are sequential contiguous writes.
            (ar + er).copy_to_slice(&mut buf.re[k + j..]);
            (ai + ei).copy_to_slice(&mut buf.im[k + j..]);
            (br + fi).copy_to_slice(&mut buf.re[k + j + qm..]);
            (bi - fr).copy_to_slice(&mut buf.im[k + j + qm..]);
            (ar - er).copy_to_slice(&mut buf.re[k + j + 2 * qm..]);
            (ai - ei).copy_to_slice(&mut buf.im[k + j + 2 * qm..]);
            (br - fi).copy_to_slice(&mut buf.re[k + j + 3 * qm..]);
            (bi + fr).copy_to_slice(&mut buf.im[k + j + 3 * qm..]);

            j += LANES;
        }

        // Scalar tail.
        while j < qm {
            let w2r = TwiddleTable::<N>::re(tw2_off + j);
            let w2i = TwiddleTable::<N>::im(tw2_off + j);
            let w1r = TwiddleTable::<N>::re(tw1_off + j);
            let w1i = TwiddleTable::<N>::im(tw1_off + j);

            let x0r = buf.re[k + j];
            let x0i = buf.im[k + j];
            let x1r = buf.re[k + j + qm];
            let x1i = buf.im[k + j + qm];
            let x2r = buf.re[k + j + 2 * qm];
            let x2i = buf.im[k + j + 2 * qm];
            let x3r = buf.re[k + j + 3 * qm];
            let x3i = buf.im[k + j + 3 * qm];

            let t1r = w2r * x1r - w2i * x1i;
            let t1i = w2r * x1i + w2i * x1r;
            let t3r = w2r * x3r - w2i * x3i;
            let t3i = w2r * x3i + w2i * x3r;

            let ar = x0r + t1r;
            let ai = x0i + t1i;
            let br = x0r - t1r;
            let bi = x0i - t1i;
            let cr = x2r + t3r;
            let ci = x2i + t3i;
            let dr = x2r - t3r;
            let di = x2i - t3i;

            let er = w1r * cr - w1i * ci;
            let ei = w1r * ci + w1i * cr;
            let fr = w1r * dr - w1i * di;
            let fi = w1r * di + w1i * dr;

            buf.re[k + j] = ar + er;
            buf.im[k + j] = ai + ei;
            buf.re[k + j + qm] = br + fi;
            buf.im[k + j + qm] = bi - fr;
            buf.re[k + j + 2 * qm] = ar - er;
            buf.im[k + j + 2 * qm] = ai - ei;
            buf.re[k + j + 3 * qm] = br - fi;
            buf.im[k + j + 3 * qm] = bi + fr;

            j += 1;
        }

        k += m4;
    }
}
