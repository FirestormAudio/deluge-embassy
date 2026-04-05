// ---------------------------------------------------------------------------
// Radix-8 DIT FFT (merged three-stage Cooley-Tukey)
// ---------------------------------------------------------------------------
//
// Theory
// ------
// We merge three consecutive DIT radix-2 stages (A, B, C) into one pass.
//
// Given 8 elements at positions {k+j, k+j+qm, ..., k+j+7qm} in bit-reversed
// order, with:
//   w4[j] = W_{8qm}^j   (span = 8qm, stage A twiddle)
//   w2[j] = W_{4qm}^j   (span = 4qm, stage B twiddle)
//   w1[j] = W_{2qm}^j   (span = 2qm, stage C twiddle)
//
//   Stage A: aₙ = xₙ + w4*x_{n+4}, bₙ = xₙ - w4*x_{n+4}  (n=0..3)
//   Stage B: cₙ = a_{2n} + w2*a_{2n+1}; dₙ = a_{2n} - w2*a_{2n+1}  etc.
//   Stage C: yₙ = c/d/e/f pairs combined with w1.
//
// Output ordering (bit-reversal maps input[n] → output[bit_rev(n, 3)]):
//   y0 = c0 + w1*c1   → position k+j+0
//   y1 = c0 - w1*c1   → position k+j+qm
//   y2 = d0 + w1*d1   → position k+j+2qm
//   y3 = d0 - w1*d1   → position k+j+3qm
//   y4 = e0 + w1*e1   → position k+j+4qm
//   y5 = e0 - w1*e1   → position k+j+5qm
//   y6 = f0 + w1*f1   → position k+j+6qm
//   y7 = f0 - w1*f1   → position k+j+7qm
//
// Total per output: 7 complex multiplies for 8 outputs (vs 12 for 3×r2 serial).
// Passes over the array: log₈N for sizes that are a power of 8.
//   log₂1024=10  → 1 r4 stage + 1 r8 stage  (floor(10/3)=3 but 10 mod 3 = 1)
//   Actually: 10 = 1×1 + 3×3 = 1 r2 + 3 r8 stages? No.
//   Best for 1024: 10 stages, factored as r8+r8+r4 (3+3+2+2) = 2 r8 + 1 r4:
//     stage 0-2 (qm=1): r8 merges stages 0,1,2
//     stage 3-5 (qm=8): r8 merges stages 3,4,5
//     stage 6-7 (qm=64): r4 merges stages 6,7
//     (log2(1024)=10 = 3+3+2+2 doesn't add up; see schedule below)
//
// Stage schedule for N=1024 (log2N=10):
//   Merged r8: stages 0,1,2 → qm=1, m8=8
//   Merged r8: stages 3,4,5 → qm=8, m8=64
//   Merged r8: stages 6,7,8 → qm=64, m8=512
//   Remaining: stage 9     → r2 with half_m=512
//   Total: 3 r8 passes + 1 r2 pass.
//
// For N=2048 (log2N=11):
//   r2: stage 0 (half_m=1)
//   r8: stages 1,2,3 → qm=2, m8=16
//   r8: stages 4,5,6 → qm=16, m8=128
//   r8: stages 7,8,9 → qm=128, m8=1024
//   r2: stage 10 (half_m=1024)  ← Wait, that's wrong.
//   Actually log2N=11: 11 = 1 + 3*3 + 1 = 1 r2 + 3 r8 + 1 r2? No.
//   11 = 2 + 3*3: 1 r4 initial + 3 r8? Also awkward.
//   Simplest: use process_r4_simd_soa for odd stages.
//
// Entry point schedule (handled in process_r8_simd_soa):
//   log2N mod 3 == 0: pure r8 passes
//   log2N mod 3 == 1: 1 leading r2 pass, then r8 pairs
//   log2N mod 3 == 2: 1 leading r4 pass, then r8 triples
// ---------------------------------------------------------------------------

use core::ops::{Add, Mul, Sub};
use core::simd::Simd;

use crate::buf::FftBuf;
use crate::fft::{assert_valid_size_r4, ilog2};
use crate::radix4::{bit_reverse_soa, r2_stage_soa, r4_stage_soa};
use crate::twiddle::TwiddleTable;

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

/// Radix-8 DIT in-place FFT on an `FftBuf<N>` (SoA), `LANES`-wide SIMD.
///
/// Falls back to radix-4 stages when log₂N is not a multiple of 3.
/// For N=1024, runs 3 radix-8 passes + 1 radix-2 pass.
pub fn process_r8_simd_soa<const N: usize, const LANES: usize>(buf: &mut FftBuf<N>)
where
    [(); 2 * N]:,
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
{
    assert_valid_size_r4::<N>();
    bit_reverse_soa(buf);

    let log2n = ilog2(N);
    let mut stage = 0usize;

    // Handle leading stages if log2N is not divisible by 3.
    match log2n % 3 {
        1 => {
            // One r2 stage (half_m = 1).
            r2_stage_soa::<N, LANES>(buf, 1);
            stage = 1;
        }
        2 => {
            // One r4 stage (qm = 1, merges stages 0 and 1).
            r4_stage_soa::<N, LANES>(buf, 1);
            stage = 2;
        }
        _ => {} // 0: no leading stage needed
    }

    // Now do r8 triples for the remaining (log2N - stage) stages.
    while stage + 2 < log2n {
        let qm = 1usize << stage;
        if qm < LANES {
            // qm too small for the wide NEON r8 path; decompose into r2 + r4
            // so the narrow NEON kernels handle the small stages efficiently.
            r2_stage_soa::<N, LANES>(buf, qm);
            r4_stage_soa::<N, LANES>(buf, 2 * qm);
        } else {
            r8_stage_soa::<N, LANES>(buf, qm);
        }
        stage += 3;
    }

    // If one stage remains (shouldn't happen given the schedule, but guard anyway).
    if stage + 1 == log2n {
        r2_stage_soa::<N, LANES>(buf, 1 << stage);
    }
}

// ---------------------------------------------------------------------------
// Radix-8 merged three-stage butterfly (SoA)
// ---------------------------------------------------------------------------

pub(crate) fn r8_stage_soa<const N: usize, const LANES: usize>(buf: &mut FftBuf<N>, qm: usize)
where
    [(); 2 * N]:,
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
{
    let m8 = 8 * qm;

    // Stages applied in DIT order: innermost first, outermost last.
    //
    // Stage 1 (innermost, half_m=qm, span=2qm): pairs (n, n+qm)
    //   Twiddle W_{2qm}^j at flat offset qm-1+j
    //
    // Stage 2 (middle, half_m=2qm, span=4qm): pairs (n, n+2qm)
    //   Twiddle W_{4qm}^j     at flat offset 2qm-1+j   (for n with bit1=0)
    //   Twiddle W_{4qm}^{j+qm} = -W_{4qm}^j  (for n with bit1=1)
    //
    // Stage 3 (outermost, half_m=4qm, span=8qm): pairs (n, n+4qm)
    //   Twiddle W_{8qm}^{j+n_off} at flat offset 4qm-1+j+n_off, n_off in {0,qm,2qm,3qm}
    //
    // We index the 8 outputs at positions 0,1,2,3,4,5,6,7 (×qm from k+j).
    // After Stage 1:   [s0,s1,s2,s3,s4,s5,s6,s7]
    // After Stage 2:   [t0,t1,t2,t3,t4,t5,t6,t7]
    // After Stage 3:   [y0,y1,y2,y3,y4,y5,y6,y7]

    // Stage 1 twiddle offset
    let tw1_off = qm - 1;
    // Stage 2 twiddle offset (for even sub-positions)
    let tw2_off = 2 * qm - 1;
    // Stage 3 twiddle offset (base; add n_off for each pair)
    let tw3_off = 4 * qm - 1;

    // NEON wide path.
    #[cfg(all(target_arch = "arm", target_feature = "neon"))]
    if LANES == 4 {
        unsafe {
            crate::arch::r8_soa_wide_neon::<N>(buf, qm);
        }
        return;
    }

    let mut k = 0usize;
    while k < N {
        let mut j = 0usize;
        while j < qm {
            // --- Load all 8 inputs ---
            let x0r = buf.re[k + j];
            let x0i = buf.im[k + j];
            let x1r = buf.re[k + j + qm];
            let x1i = buf.im[k + j + qm];
            let x2r = buf.re[k + j + 2 * qm];
            let x2i = buf.im[k + j + 2 * qm];
            let x3r = buf.re[k + j + 3 * qm];
            let x3i = buf.im[k + j + 3 * qm];
            let x4r = buf.re[k + j + 4 * qm];
            let x4i = buf.im[k + j + 4 * qm];
            let x5r = buf.re[k + j + 5 * qm];
            let x5i = buf.im[k + j + 5 * qm];
            let x6r = buf.re[k + j + 6 * qm];
            let x6i = buf.im[k + j + 6 * qm];
            let x7r = buf.re[k + j + 7 * qm];
            let x7i = buf.im[k + j + 7 * qm];

            // --- Stage 1: half_m=qm, pairs (0,1),(2,3),(4,5),(6,7) ---
            // All 4 pairs share the same twiddle W_{2qm}^j.
            let w1r = TwiddleTable::<N>::re(tw1_off + j);
            let w1i = TwiddleTable::<N>::im(tw1_off + j);

            macro_rules! r2 {
                ($ur:expr, $ui:expr, $vr:expr, $vi:expr) => {{
                    let tr = w1r * $vr - w1i * $vi;
                    let ti = w1r * $vi + w1i * $vr;
                    (($ur + tr, $ui + ti), ($ur - tr, $ui - ti))
                }};
            }

            let ((s0r, s0i), (s1r, s1i)) = r2!(x0r, x0i, x1r, x1i);
            let ((s2r, s2i), (s3r, s3i)) = r2!(x2r, x2i, x3r, x3i);
            let ((s4r, s4i), (s5r, s5i)) = r2!(x4r, x4i, x5r, x5i);
            let ((s6r, s6i), (s7r, s7i)) = r2!(x6r, x6i, x7r, x7i);

            // --- Stage 2: half_m=2qm, pairs (0,2),(1,3),(4,6),(5,7) ---
            // Even sub-positions (0,4): W_{4qm}^j
            // Odd sub-positions (1,5): W_{4qm}^{j+qm}
            let w2er = TwiddleTable::<N>::re(tw2_off + j);
            let w2ei = TwiddleTable::<N>::im(tw2_off + j);
            let w2or = TwiddleTable::<N>::re(tw2_off + j + qm);
            let w2oi = TwiddleTable::<N>::im(tw2_off + j + qm);

            // Pair (s0, s2): twiddle W_{4qm}^j
            let tr02r = w2er * s2r - w2ei * s2i;
            let tr02i = w2er * s2i + w2ei * s2r;
            let (t0r, t0i) = (s0r + tr02r, s0i + tr02i);
            let (t2r, t2i) = (s0r - tr02r, s0i - tr02i);

            // Pair (s1, s3): twiddle W_{4qm}^{j+qm}
            let tr13r = w2or * s3r - w2oi * s3i;
            let tr13i = w2or * s3i + w2oi * s3r;
            let (t1r, t1i) = (s1r + tr13r, s1i + tr13i);
            let (t3r, t3i) = (s1r - tr13r, s1i - tr13i);

            // Pair (s4, s6): twiddle W_{4qm}^j
            let tr46r = w2er * s6r - w2ei * s6i;
            let tr46i = w2er * s6i + w2ei * s6r;
            let (t4r, t4i) = (s4r + tr46r, s4i + tr46i);
            let (t6r, t6i) = (s4r - tr46r, s4i - tr46i);

            // Pair (s5, s7): twiddle W_{4qm}^{j+qm}
            let tr57r = w2or * s7r - w2oi * s7i;
            let tr57i = w2or * s7i + w2oi * s7r;
            let (t5r, t5i) = (s5r + tr57r, s5i + tr57i);
            let (t7r, t7i) = (s5r - tr57r, s5i - tr57i);

            // --- Stage 3: half_m=4qm, pairs (0,4),(1,5),(2,6),(3,7) ---
            // Each pair has its own twiddle: W_{8qm}^{j+n_off}
            let w3_0r = TwiddleTable::<N>::re(tw3_off + j);
            let w3_0i = TwiddleTable::<N>::im(tw3_off + j);
            let w3_1r = TwiddleTable::<N>::re(tw3_off + j + qm);
            let w3_1i = TwiddleTable::<N>::im(tw3_off + j + qm);
            let w3_2r = TwiddleTable::<N>::re(tw3_off + j + 2 * qm);
            let w3_2i = TwiddleTable::<N>::im(tw3_off + j + 2 * qm);
            let w3_3r = TwiddleTable::<N>::re(tw3_off + j + 3 * qm);
            let w3_3i = TwiddleTable::<N>::im(tw3_off + j + 3 * qm);

            // Pair (t0, t4)
            let tr04r = w3_0r * t4r - w3_0i * t4i;
            let tr04i = w3_0r * t4i + w3_0i * t4r;
            // Pair (t1, t5)
            let tr15r = w3_1r * t5r - w3_1i * t5i;
            let tr15i = w3_1r * t5i + w3_1i * t5r;
            // Pair (t2, t6)
            let tr26r = w3_2r * t6r - w3_2i * t6i;
            let tr26i = w3_2r * t6i + w3_2i * t6r;
            // Pair (t3, t7)
            let tr37r = w3_3r * t7r - w3_3i * t7i;
            let tr37i = w3_3r * t7i + w3_3i * t7r;

            buf.re[k + j] = t0r + tr04r;
            buf.im[k + j] = t0i + tr04i;
            buf.re[k + j + qm] = t1r + tr15r;
            buf.im[k + j + qm] = t1i + tr15i;
            buf.re[k + j + 2 * qm] = t2r + tr26r;
            buf.im[k + j + 2 * qm] = t2i + tr26i;
            buf.re[k + j + 3 * qm] = t3r + tr37r;
            buf.im[k + j + 3 * qm] = t3i + tr37i;
            buf.re[k + j + 4 * qm] = t0r - tr04r;
            buf.im[k + j + 4 * qm] = t0i - tr04i;
            buf.re[k + j + 5 * qm] = t1r - tr15r;
            buf.im[k + j + 5 * qm] = t1i - tr15i;
            buf.re[k + j + 6 * qm] = t2r - tr26r;
            buf.im[k + j + 6 * qm] = t2i - tr26i;
            buf.re[k + j + 7 * qm] = t3r - tr37r;
            buf.im[k + j + 7 * qm] = t3i - tr37i;
            j += 1;
        }
        k += m8;
    }
}
