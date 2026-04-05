// ---------------------------------------------------------------------------
// arch.rs — NEON-accelerated FFT stage kernels
//
// All unsafe is quarantined here.  Callers dispatch via `unsafe { arch::… }`
// under `#[cfg(…)]` guards.
//
// Twiddle layout
// --------------
// Narrow paths (half_m / qm < 4) use TwiddleTable::<N>::RE / ::IM.
// Wide paths  (half_m / qm >= 4) use TwiddleTableRI::<N>::RI, where every
// two consecutive entries are (re, im) — a single vld2q_f32 loads 4 complex
// twiddles in two float32x4_t registers, saving one load instruction vs two
// separate vld1q_f32 calls.
// ---------------------------------------------------------------------------

#[cfg(all(target_arch = "arm", target_feature = "neon"))]
mod neon {
    use core::arch::arm::*;

    use crate::buf::FftBuf;
    use crate::twiddle::{TwiddleTable, TwiddleTableRI};

    // -----------------------------------------------------------------------
    // Radix-2 SoA WIDE path  (half_m >= 4)
    //
    // Each inner iteration processes 4 consecutive j-values.  Twiddles are
    // loaded with a single vld2q_f32 from the interleaved RI table:
    //   .0 = {wr0..wr3}   .1 = {wi0..wi3}
    // saving one load instruction per loop iteration vs two vld1q_f32.
    // -----------------------------------------------------------------------
    #[inline]
    pub(crate) unsafe fn r2_soa_wide_neon<const N: usize>(buf: &mut FftBuf<N>, half_m: usize)
    where
        [(); 2 * N]:,
    {
        unsafe {
            let tw_base = half_m - 1;
            let ri = &TwiddleTableRI::<N>::RI;

            let mut k = 0usize;
            while k < N {
                let mut j = 0usize;
                while j + 4 <= half_m {
                    let tw = vld2q_f32(ri[2 * (tw_base + j)..].as_ptr());
                    let wr = tw.0;
                    let wi = tw.1;

                    let ur = vld1q_f32(buf.re[k + j..].as_ptr());
                    let ui = vld1q_f32(buf.im[k + j..].as_ptr());
                    let vr = vld1q_f32(buf.re[k + j + half_m..].as_ptr());
                    let vi = vld1q_f32(buf.im[k + j + half_m..].as_ptr());

                    let tr = vmlsq_f32(vmulq_f32(wr, vr), wi, vi);
                    let ti = vmlaq_f32(vmulq_f32(wr, vi), wi, vr);

                    vst1q_f32(buf.re[k + j..].as_mut_ptr(), vaddq_f32(ur, tr));
                    vst1q_f32(buf.im[k + j..].as_mut_ptr(), vaddq_f32(ui, ti));
                    vst1q_f32(buf.re[k + j + half_m..].as_mut_ptr(), vsubq_f32(ur, tr));
                    vst1q_f32(buf.im[k + j + half_m..].as_mut_ptr(), vsubq_f32(ui, ti));

                    j += 4;
                }
                // scalar tail for half_m not divisible by 4 (won't happen for power-of-two FFT)
                while j < half_m {
                    let wr = TwiddleTable::<N>::re(tw_base + j);
                    let wi = TwiddleTable::<N>::im(tw_base + j);
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

    // -----------------------------------------------------------------------
    // Radix-4 SoA WIDE path  (qm >= 4)
    //
    // Two vld2q_f32 per inner loop iteration load 4 w2 and 4 w1 twiddle pairs.
    // -----------------------------------------------------------------------
    #[inline]
    pub(crate) unsafe fn r4_soa_wide_neon<const N: usize>(buf: &mut FftBuf<N>, qm: usize)
    where
        [(); 2 * N]:,
    {
        unsafe {
            let tw2_base = qm - 1;
            let tw1_base = 2 * qm - 1;
            let m4 = 4 * qm;
            let ri = &TwiddleTableRI::<N>::RI;

            let mut k = 0usize;
            while k < N {
                let mut j = 0usize;
                while j + 4 <= qm {
                    let tw2 = vld2q_f32(ri[2 * (tw2_base + j)..].as_ptr());
                    let tw1 = vld2q_f32(ri[2 * (tw1_base + j)..].as_ptr());
                    let w2r = tw2.0;
                    let w2i = tw2.1;
                    let w1r = tw1.0;
                    let w1i = tw1.1;

                    let x0r = vld1q_f32(buf.re[k + j..].as_ptr());
                    let x0i = vld1q_f32(buf.im[k + j..].as_ptr());
                    let x1r = vld1q_f32(buf.re[k + j + qm..].as_ptr());
                    let x1i = vld1q_f32(buf.im[k + j + qm..].as_ptr());
                    let x2r = vld1q_f32(buf.re[k + j + 2 * qm..].as_ptr());
                    let x2i = vld1q_f32(buf.im[k + j + 2 * qm..].as_ptr());
                    let x3r = vld1q_f32(buf.re[k + j + 3 * qm..].as_ptr());
                    let x3i = vld1q_f32(buf.im[k + j + 3 * qm..].as_ptr());

                    let t1r = vmlsq_f32(vmulq_f32(w2r, x1r), w2i, x1i);
                    let t1i = vmlaq_f32(vmulq_f32(w2r, x1i), w2i, x1r);
                    let t3r = vmlsq_f32(vmulq_f32(w2r, x3r), w2i, x3i);
                    let t3i = vmlaq_f32(vmulq_f32(w2r, x3i), w2i, x3r);

                    let ar = vaddq_f32(x0r, t1r);
                    let ai = vaddq_f32(x0i, t1i);
                    let br = vsubq_f32(x0r, t1r);
                    let bi = vsubq_f32(x0i, t1i);
                    let cr = vaddq_f32(x2r, t3r);
                    let ci = vaddq_f32(x2i, t3i);
                    let dr = vsubq_f32(x2r, t3r);
                    let di = vsubq_f32(x2i, t3i);

                    let er = vmlsq_f32(vmulq_f32(w1r, cr), w1i, ci);
                    let ei = vmlaq_f32(vmulq_f32(w1r, ci), w1i, cr);
                    let fr = vmlsq_f32(vmulq_f32(w1r, dr), w1i, di);
                    let fi = vmlaq_f32(vmulq_f32(w1r, di), w1i, dr);

                    vst1q_f32(buf.re[k + j..].as_mut_ptr(), vaddq_f32(ar, er));
                    vst1q_f32(buf.im[k + j..].as_mut_ptr(), vaddq_f32(ai, ei));
                    vst1q_f32(buf.re[k + j + qm..].as_mut_ptr(), vaddq_f32(br, fi));
                    vst1q_f32(buf.im[k + j + qm..].as_mut_ptr(), vsubq_f32(bi, fr));
                    vst1q_f32(buf.re[k + j + 2 * qm..].as_mut_ptr(), vsubq_f32(ar, er));
                    vst1q_f32(buf.im[k + j + 2 * qm..].as_mut_ptr(), vsubq_f32(ai, ei));
                    vst1q_f32(buf.re[k + j + 3 * qm..].as_mut_ptr(), vsubq_f32(br, fi));
                    vst1q_f32(buf.im[k + j + 3 * qm..].as_mut_ptr(), vaddq_f32(bi, fr));

                    j += 4;
                }
                while j < qm {
                    let w2r = TwiddleTable::<N>::re(tw2_base + j);
                    let w2i = TwiddleTable::<N>::im(tw2_base + j);
                    let w1r = TwiddleTable::<N>::re(tw1_base + j);
                    let w1i = TwiddleTable::<N>::im(tw1_base + j);
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
    }

    // -----------------------------------------------------------------------
    // Radix-8 SoA WIDE path
    //
    // Merges three consecutive radix-2 stages into one pass.
    // qm = N/8 per k-group; em = 4*qm; m8 = 8*qm.
    //
    // Twiddle layout:
    //   w4[j] = W_{8qm}^j    at offset  4qm-1+j   (stage A, half_m=4qm)
    //   w2[j] = W_{4qm}^j    at offset  2qm-1+j   (stage B, half_m=2qm)
    //   w1[j] = W_{2qm}^j    at offset   qm-1+j   (stage C, half_m=qm)
    //
    // The full butterfly on [x0..x7] for one j:
    //   Stage A (half_m = 4qm):
    //     a0=x0+w4*x4, a1=x1+w4*x5, a2=x2+w4*x6, a3=x3+w4*x7
    //     b0=x0-w4*x4, b1=x1-w4*x5, b2=x2-w4*x6, b3=x3-w4*x7
    //   Stage B (half_m = 2qm):
    //     c0=a0+w2*a2, c1=a1+w2*a3,  d0=a0-w2*a2, d1=a1-w2*a3
    //     e0=b0+w2*b2, e1=b1+w2*b3,  f0=b0-w2*b2, f1=b1-w2*b3
    //   Stage C (half_m = qm):
    //     y0=c0+w1*c1,  y1=c0-w1*c1
    //     y2=d0-j*w1*d1 = (d0r+w1i*d1i+w1r*(-d1r),…) — actually the standard
    //       w_{2qm}^{j+qm} = -j * w_{2qm}^j rotation applies here:
    //     Actually for stage C (half_m=qm, span=2qm): twiddle for position j
    //     is W_{2qm}^j at offset qm-1+j.  So:
    //     y0=c0+w1*c1, y1=c0-w1*c1  (c0,c1 share same w1 since same j)
    //     y4=d0+jw1*d1? No — let's use the correct formulation.
    //
    // Standard 8-point DIT butterfly (Cooley-Tukey, decimation in time):
    //   Given bit-reversed input [x0..x7], outputs [y0..y7]:
    //   Using stage A twiddle tw4=W_8^j, stage B tw2=W_4^j, stage C tw1=W_2^j=1:
    //
    // For general radix-8 with arbitrary j we fold three r2 stages.  The
    // twiddle for the innermost (stage C) j-subrange is w1=W_{2qm}^j.
    //
    // Implementation: process 4 j-values in parallel using NEON f32x4, so
    // we do one complete 8-point butterfly pass per group of 4 j-values.
    // Each iteration: 6 vld2q_f32 for twiddles + 16 vld1q_f32 for data.
    // -----------------------------------------------------------------------
    #[inline]
    pub(crate) unsafe fn r8_soa_wide_neon<const N: usize>(buf: &mut FftBuf<N>, qm: usize)
    where
        [(); 2 * N]:,
    {
        // Merged DIT radix-8 butterfly: three stages applied innermost-first.
        //
        // Stage 1 (innermost, span=2qm):  pairs (0,1),(2,3),(4,5),(6,7)
        //   all use twiddle W_{2qm}^j at RI[2*(qm-1+j)]
        // Stage 2 (middle, span=4qm):  pairs (0,2),(1,3),(4,6),(5,7)
        //   even sub-positions: W_{4qm}^j     at RI[2*(2qm-1+j)]
        //   odd  sub-positions: W_{4qm}^{j+qm} at RI[2*(3qm-1+j)]
        // Stage 3 (outermost, span=8qm): pairs (0,4),(1,5),(2,6),(3,7)
        //   pair (0,4): W_{8qm}^j       at RI[2*(4qm-1+j)]
        //   pair (1,5): W_{8qm}^{j+qm}   at RI[2*(5qm-1+j)]
        //   pair (2,6): W_{8qm}^{j+2qm}  at RI[2*(6qm-1+j)]
        //   pair (3,7): W_{8qm}^{j+3qm}  at RI[2*(7qm-1+j)]
        //
        // 7 vld2q_f32 per inner loop iteration.

        let m8 = 8 * qm;

        // Twiddle base offsets (flat index into twiddle table)
        let tw1_base = qm - 1; // Stage 1: W_{2qm}^j
        let tw2e_base = 2 * qm - 1; // Stage 2 even: W_{4qm}^j
        let tw2o_base = 3 * qm - 1; // Stage 2 odd:  W_{4qm}^{j+qm}
        let tw3_0_base = 4 * qm - 1; // Stage 3 pair (0,4): W_{8qm}^j
        let tw3_1_base = 5 * qm - 1; // Stage 3 pair (1,5): W_{8qm}^{j+qm}
        let tw3_2_base = 6 * qm - 1; // Stage 3 pair (2,6): W_{8qm}^{j+2qm}
        let tw3_3_base = 7 * qm - 1; // Stage 3 pair (3,7): W_{8qm}^{j+3qm}

        let ri = &TwiddleTableRI::<N>::RI;

        let mut k = 0usize;
        while k < N {
            let mut j = 0usize;
            while j + 4 <= qm {
                // --- Load twiddles (7 vld2q_f32) ---
                let t1 = unsafe { vld2q_f32(ri[2 * (tw1_base + j)..].as_ptr()) };
                let t2e = unsafe { vld2q_f32(ri[2 * (tw2e_base + j)..].as_ptr()) };
                let t2o = unsafe { vld2q_f32(ri[2 * (tw2o_base + j)..].as_ptr()) };
                let t3_0 = unsafe { vld2q_f32(ri[2 * (tw3_0_base + j)..].as_ptr()) };
                let t3_1 = unsafe { vld2q_f32(ri[2 * (tw3_1_base + j)..].as_ptr()) };
                let t3_2 = unsafe { vld2q_f32(ri[2 * (tw3_2_base + j)..].as_ptr()) };
                let t3_3 = unsafe { vld2q_f32(ri[2 * (tw3_3_base + j)..].as_ptr()) };

                let w1r = t1.0;
                let w1i = t1.1;
                let w2er = t2e.0;
                let w2ei = t2e.1;
                let w2or = t2o.0;
                let w2oi = t2o.1;
                let w3_0r = t3_0.0;
                let w3_0i = t3_0.1;
                let w3_1r = t3_1.0;
                let w3_1i = t3_1.1;
                let w3_2r = t3_2.0;
                let w3_2i = t3_2.1;
                let w3_3r = t3_3.0;
                let w3_3i = t3_3.1;

                // --- Load data (16 vld1q_f32) ---
                macro_rules! ld {
                    ($off:expr) => {{
                        let r = unsafe { vld1q_f32(buf.re[k + j + $off..].as_ptr()) };
                        let i = unsafe { vld1q_f32(buf.im[k + j + $off..].as_ptr()) };
                        (r, i)
                    }};
                }
                let (x0r, x0i) = ld!(0);
                let (x1r, x1i) = ld!(qm);
                let (x2r, x2i) = ld!(2 * qm);
                let (x3r, x3i) = ld!(3 * qm);
                let (x4r, x4i) = ld!(4 * qm);
                let (x5r, x5i) = ld!(5 * qm);
                let (x6r, x6i) = ld!(6 * qm);
                let (x7r, x7i) = ld!(7 * qm);

                // Complex multiply helper
                macro_rules! cmul {
                    ($wr:expr, $wi:expr, $xr:expr, $xi:expr) => {{
                        let tr = unsafe { vmlsq_f32(vmulq_f32($wr, $xr), $wi, $xi) };
                        let ti = unsafe { vmlaq_f32(vmulq_f32($wr, $xi), $wi, $xr) };
                        (tr, ti)
                    }};
                }

                // --- Stage 1: pairs (0,1),(2,3),(4,5),(6,7) — all use w1 ---
                let (p1r, p1i) = cmul!(w1r, w1i, x1r, x1i);
                let (p3r, p3i) = cmul!(w1r, w1i, x3r, x3i);
                let (p5r, p5i) = cmul!(w1r, w1i, x5r, x5i);
                let (p7r, p7i) = cmul!(w1r, w1i, x7r, x7i);

                let s0r = unsafe { vaddq_f32(x0r, p1r) };
                let s0i = unsafe { vaddq_f32(x0i, p1i) };
                let s1r = unsafe { vsubq_f32(x0r, p1r) };
                let s1i = unsafe { vsubq_f32(x0i, p1i) };
                let s2r = unsafe { vaddq_f32(x2r, p3r) };
                let s2i = unsafe { vaddq_f32(x2i, p3i) };
                let s3r = unsafe { vsubq_f32(x2r, p3r) };
                let s3i = unsafe { vsubq_f32(x2i, p3i) };
                let s4r = unsafe { vaddq_f32(x4r, p5r) };
                let s4i = unsafe { vaddq_f32(x4i, p5i) };
                let s5r = unsafe { vsubq_f32(x4r, p5r) };
                let s5i = unsafe { vsubq_f32(x4i, p5i) };
                let s6r = unsafe { vaddq_f32(x6r, p7r) };
                let s6i = unsafe { vaddq_f32(x6i, p7i) };
                let s7r = unsafe { vsubq_f32(x6r, p7r) };
                let s7i = unsafe { vsubq_f32(x6i, p7i) };

                // --- Stage 2: pairs (0,2),(1,3),(4,6),(5,7) ---
                // Even — w2e: (s0,s2), (s4,s6)
                let (q2r, q2i) = cmul!(w2er, w2ei, s2r, s2i);
                let (q6r, q6i) = cmul!(w2er, w2ei, s6r, s6i);
                // Odd  — w2o: (s1,s3), (s5,s7)
                let (q3r, q3i) = cmul!(w2or, w2oi, s3r, s3i);
                let (q7r, q7i) = cmul!(w2or, w2oi, s7r, s7i);

                let t0r = unsafe { vaddq_f32(s0r, q2r) };
                let t0i = unsafe { vaddq_f32(s0i, q2i) };
                let t2r = unsafe { vsubq_f32(s0r, q2r) };
                let t2i = unsafe { vsubq_f32(s0i, q2i) };
                let t1r = unsafe { vaddq_f32(s1r, q3r) };
                let t1i = unsafe { vaddq_f32(s1i, q3i) };
                let t3r = unsafe { vsubq_f32(s1r, q3r) };
                let t3i = unsafe { vsubq_f32(s1i, q3i) };
                let t4r = unsafe { vaddq_f32(s4r, q6r) };
                let t4i = unsafe { vaddq_f32(s4i, q6i) };
                let t6r = unsafe { vsubq_f32(s4r, q6r) };
                let t6i = unsafe { vsubq_f32(s4i, q6i) };
                let t5r = unsafe { vaddq_f32(s5r, q7r) };
                let t5i = unsafe { vaddq_f32(s5i, q7i) };
                let t7r = unsafe { vsubq_f32(s5r, q7r) };
                let t7i = unsafe { vsubq_f32(s5i, q7i) };

                // --- Stage 3: pairs (0,4),(1,5),(2,6),(3,7) — distinct twiddles ---
                let (r4r, r4i) = cmul!(w3_0r, w3_0i, t4r, t4i);
                let (r5r, r5i) = cmul!(w3_1r, w3_1i, t5r, t5i);
                let (r6r, r6i) = cmul!(w3_2r, w3_2i, t6r, t6i);
                let (r7r, r7i) = cmul!(w3_3r, w3_3i, t7r, t7i);

                let y0r = unsafe { vaddq_f32(t0r, r4r) };
                let y0i = unsafe { vaddq_f32(t0i, r4i) };
                let y4r = unsafe { vsubq_f32(t0r, r4r) };
                let y4i = unsafe { vsubq_f32(t0i, r4i) };
                let y1r = unsafe { vaddq_f32(t1r, r5r) };
                let y1i = unsafe { vaddq_f32(t1i, r5i) };
                let y5r = unsafe { vsubq_f32(t1r, r5r) };
                let y5i = unsafe { vsubq_f32(t1i, r5i) };
                let y2r = unsafe { vaddq_f32(t2r, r6r) };
                let y2i = unsafe { vaddq_f32(t2i, r6i) };
                let y6r = unsafe { vsubq_f32(t2r, r6r) };
                let y6i = unsafe { vsubq_f32(t2i, r6i) };
                let y3r = unsafe { vaddq_f32(t3r, r7r) };
                let y3i = unsafe { vaddq_f32(t3i, r7i) };
                let y7r = unsafe { vsubq_f32(t3r, r7r) };
                let y7i = unsafe { vsubq_f32(t3i, r7i) };

                // --- Store ---
                macro_rules! st {
                    ($off:expr, $vr:expr, $vi:expr) => {
                        unsafe {
                            vst1q_f32(buf.re[k + j + $off..].as_mut_ptr(), $vr);
                            vst1q_f32(buf.im[k + j + $off..].as_mut_ptr(), $vi);
                        }
                    };
                }
                st!(0, y0r, y0i);
                st!(qm, y1r, y1i);
                st!(2 * qm, y2r, y2i);
                st!(3 * qm, y3r, y3i);
                st!(4 * qm, y4r, y4i);
                st!(5 * qm, y5r, y5i);
                st!(6 * qm, y6r, y6i);
                st!(7 * qm, y7r, y7i);

                j += 4;
            }
            // Scalar tail for qm not divisible by 4
            while j < qm {
                let w1r = TwiddleTable::<N>::re(tw1_base + j);
                let w1i = TwiddleTable::<N>::im(tw1_base + j);
                let w2er = TwiddleTable::<N>::re(tw2e_base + j);
                let w2ei = TwiddleTable::<N>::im(tw2e_base + j);
                let w2or = TwiddleTable::<N>::re(tw2o_base + j);
                let w2oi = TwiddleTable::<N>::im(tw2o_base + j);
                let w3_0r = TwiddleTable::<N>::re(tw3_0_base + j);
                let w3_0i = TwiddleTable::<N>::im(tw3_0_base + j);
                let w3_1r = TwiddleTable::<N>::re(tw3_1_base + j);
                let w3_1i = TwiddleTable::<N>::im(tw3_1_base + j);
                let w3_2r = TwiddleTable::<N>::re(tw3_2_base + j);
                let w3_2i = TwiddleTable::<N>::im(tw3_2_base + j);
                let w3_3r = TwiddleTable::<N>::re(tw3_3_base + j);
                let w3_3i = TwiddleTable::<N>::im(tw3_3_base + j);

                macro_rules! scmul {
                    ($wr:expr, $wi:expr, $xr:expr, $xi:expr) => {{ ($wr * $xr - $wi * $xi, $wr * $xi + $wi * $xr) }};
                }
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

                // Stage 1
                let (p1r, p1i) = scmul!(w1r, w1i, x1r, x1i);
                let (p3r, p3i) = scmul!(w1r, w1i, x3r, x3i);
                let (p5r, p5i) = scmul!(w1r, w1i, x5r, x5i);
                let (p7r, p7i) = scmul!(w1r, w1i, x7r, x7i);
                let (s0r, s0i) = (x0r + p1r, x0i + p1i);
                let (s1r, s1i) = (x0r - p1r, x0i - p1i);
                let (s2r, s2i) = (x2r + p3r, x2i + p3i);
                let (s3r, s3i) = (x2r - p3r, x2i - p3i);
                let (s4r, s4i) = (x4r + p5r, x4i + p5i);
                let (s5r, s5i) = (x4r - p5r, x4i - p5i);
                let (s6r, s6i) = (x6r + p7r, x6i + p7i);
                let (s7r, s7i) = (x6r - p7r, x6i - p7i);

                // Stage 2
                let (q2r, q2i) = scmul!(w2er, w2ei, s2r, s2i);
                let (q3r, q3i) = scmul!(w2or, w2oi, s3r, s3i);
                let (q6r, q6i) = scmul!(w2er, w2ei, s6r, s6i);
                let (q7r, q7i) = scmul!(w2or, w2oi, s7r, s7i);
                let (t0r, t0i) = (s0r + q2r, s0i + q2i);
                let (t2r, t2i) = (s0r - q2r, s0i - q2i);
                let (t1r, t1i) = (s1r + q3r, s1i + q3i);
                let (t3r, t3i) = (s1r - q3r, s1i - q3i);
                let (t4r, t4i) = (s4r + q6r, s4i + q6i);
                let (t6r, t6i) = (s4r - q6r, s4i - q6i);
                let (t5r, t5i) = (s5r + q7r, s5i + q7i);
                let (t7r, t7i) = (s5r - q7r, s5i - q7i);

                // Stage 3
                let (r4r, r4i) = scmul!(w3_0r, w3_0i, t4r, t4i);
                let (r5r, r5i) = scmul!(w3_1r, w3_1i, t5r, t5i);
                let (r6r, r6i) = scmul!(w3_2r, w3_2i, t6r, t6i);
                let (r7r, r7i) = scmul!(w3_3r, w3_3i, t7r, t7i);
                buf.re[k + j] = t0r + r4r;
                buf.im[k + j] = t0i + r4i;
                buf.re[k + j + qm] = t1r + r5r;
                buf.im[k + j + qm] = t1i + r5i;
                buf.re[k + j + 2 * qm] = t2r + r6r;
                buf.im[k + j + 2 * qm] = t2i + r6i;
                buf.re[k + j + 3 * qm] = t3r + r7r;
                buf.im[k + j + 3 * qm] = t3i + r7i;
                buf.re[k + j + 4 * qm] = t0r - r4r;
                buf.im[k + j + 4 * qm] = t0i - r4i;
                buf.re[k + j + 5 * qm] = t1r - r5r;
                buf.im[k + j + 5 * qm] = t1i - r5i;
                buf.re[k + j + 6 * qm] = t2r - r6r;
                buf.im[k + j + 6 * qm] = t2i - r6i;
                buf.re[k + j + 7 * qm] = t3r - r7r;
                buf.im[k + j + 7 * qm] = t3i - r7i;
                j += 1;
            }
            k += m8;
        }
    }
    // -----------------------------------------------------------------------
    // Narrow paths — half_m or qm < 4: process multiple groups per SIMD vector
    // -----------------------------------------------------------------------

    /// Radix-2 SoA narrow butterfly for `half_m == 1`.
    #[inline]
    pub(crate) unsafe fn r2_soa_hm1_neon<const N: usize>(buf: &mut FftBuf<N>, tw_off: usize) {
        unsafe {
            let wr = vdupq_n_f32(TwiddleTable::<N>::RE[tw_off]);
            let wi = vdupq_n_f32(TwiddleTable::<N>::IM[tw_off]);

            let mut k = 0usize;
            while k + 8 <= N {
                let re_uv = vld2q_f32(buf.re[k..].as_ptr());
                let im_uv = vld2q_f32(buf.im[k..].as_ptr());
                let ur = re_uv.0;
                let ui = im_uv.0;
                let vr = re_uv.1;
                let vi = im_uv.1;

                let tr = vmlsq_f32(vmulq_f32(wr, vr), wi, vi);
                let ti = vmlaq_f32(vmulq_f32(wr, vi), wi, vr);

                vst2q_f32(
                    buf.re[k..].as_mut_ptr(),
                    float32x4x2_t(vaddq_f32(ur, tr), vsubq_f32(ur, tr)),
                );
                vst2q_f32(
                    buf.im[k..].as_mut_ptr(),
                    float32x4x2_t(vaddq_f32(ui, ti), vsubq_f32(ui, ti)),
                );
                k += 8;
            }
        }
    }

    /// Radix-2 SoA narrow butterfly for `half_m == 2`.
    #[inline]
    pub(crate) unsafe fn r2_soa_hm2_neon<const N: usize>(buf: &mut FftBuf<N>, tw_off: usize) {
        unsafe {
            let wr_pair = vld1_f32(TwiddleTable::<N>::RE[tw_off..].as_ptr());
            let wi_pair = vld1_f32(TwiddleTable::<N>::IM[tw_off..].as_ptr());
            let wr = vcombine_f32(wr_pair, wr_pair);
            let wi = vcombine_f32(wi_pair, wi_pair);

            let mut k = 0usize;
            while k + 8 <= N {
                let re0 = vld1q_f32(buf.re[k..].as_ptr());
                let re1 = vld1q_f32(buf.re[k + 4..].as_ptr());
                let im0 = vld1q_f32(buf.im[k..].as_ptr());
                let im1 = vld1q_f32(buf.im[k + 4..].as_ptr());

                let ur = vcombine_f32(vget_low_f32(re0), vget_low_f32(re1));
                let vr = vcombine_f32(vget_high_f32(re0), vget_high_f32(re1));
                let ui = vcombine_f32(vget_low_f32(im0), vget_low_f32(im1));
                let vi = vcombine_f32(vget_high_f32(im0), vget_high_f32(im1));

                let tr = vmlsq_f32(vmulq_f32(wr, vr), wi, vi);
                let ti = vmlaq_f32(vmulq_f32(wr, vi), wi, vr);

                let oar = vaddq_f32(ur, tr);
                let oai = vaddq_f32(ui, ti);
                let obr = vsubq_f32(ur, tr);
                let obi = vsubq_f32(ui, ti);

                vst1q_f32(
                    buf.re[k..].as_mut_ptr(),
                    vcombine_f32(vget_low_f32(oar), vget_low_f32(obr)),
                );
                vst1q_f32(
                    buf.re[k + 4..].as_mut_ptr(),
                    vcombine_f32(vget_high_f32(oar), vget_high_f32(obr)),
                );
                vst1q_f32(
                    buf.im[k..].as_mut_ptr(),
                    vcombine_f32(vget_low_f32(oai), vget_low_f32(obi)),
                );
                vst1q_f32(
                    buf.im[k + 4..].as_mut_ptr(),
                    vcombine_f32(vget_high_f32(oai), vget_high_f32(obi)),
                );
                k += 8;
            }
        }
    }

    /// Radix-4 SoA narrow butterfly for `qm == 1` (twiddles = 1+0i).
    #[inline]
    pub(crate) unsafe fn r4_soa_qm1_neon<const N: usize>(buf: &mut FftBuf<N>) {
        unsafe {
            let mut k = 0usize;
            while k + 16 <= N {
                let re = vld4q_f32(buf.re[k..].as_ptr());
                let im = vld4q_f32(buf.im[k..].as_ptr());
                let x0r = re.0;
                let x0i = im.0;
                let x1r = re.1;
                let x1i = im.1;
                let x2r = re.2;
                let x2i = im.2;
                let x3r = re.3;
                let x3i = im.3;

                let ar = vaddq_f32(x0r, x1r);
                let ai = vaddq_f32(x0i, x1i);
                let br = vsubq_f32(x0r, x1r);
                let bi = vsubq_f32(x0i, x1i);
                let cr = vaddq_f32(x2r, x3r);
                let ci = vaddq_f32(x2i, x3i);
                let dr = vsubq_f32(x2r, x3r);
                let di = vsubq_f32(x2i, x3i);

                let y0r = vaddq_f32(ar, cr);
                let y0i = vaddq_f32(ai, ci);
                let y2r = vsubq_f32(ar, cr);
                let y2i = vsubq_f32(ai, ci);
                let y1r = vaddq_f32(br, di);
                let y1i = vsubq_f32(bi, dr);
                let y3r = vsubq_f32(br, di);
                let y3i = vaddq_f32(bi, dr);

                vst4q_f32(buf.re[k..].as_mut_ptr(), float32x4x4_t(y0r, y1r, y2r, y3r));
                vst4q_f32(buf.im[k..].as_mut_ptr(), float32x4x4_t(y0i, y1i, y2i, y3i));
                k += 16;
            }
        }
    }

    /// Radix-4 SoA narrow butterfly for `qm == 2`.
    #[inline]
    pub(crate) unsafe fn r4_soa_qm2_neon<const N: usize>(
        buf: &mut FftBuf<N>,
        tw2_off: usize,
        tw1_off: usize,
    ) {
        unsafe {
            let w2r_pair = vld1_f32(TwiddleTable::<N>::RE[tw2_off..].as_ptr());
            let w2i_pair = vld1_f32(TwiddleTable::<N>::IM[tw2_off..].as_ptr());
            let w1r_pair = vld1_f32(TwiddleTable::<N>::RE[tw1_off..].as_ptr());
            let w1i_pair = vld1_f32(TwiddleTable::<N>::IM[tw1_off..].as_ptr());
            let w2r = vcombine_f32(w2r_pair, w2r_pair);
            let w2i = vcombine_f32(w2i_pair, w2i_pair);
            let w1r = vcombine_f32(w1r_pair, w1r_pair);
            let w1i = vcombine_f32(w1i_pair, w1i_pair);

            let mut k = 0usize;
            while k + 16 <= N {
                let re_a = vld1q_f32(buf.re[k..].as_ptr());
                let re_b = vld1q_f32(buf.re[k + 4..].as_ptr());
                let re_c = vld1q_f32(buf.re[k + 8..].as_ptr());
                let re_d = vld1q_f32(buf.re[k + 12..].as_ptr());
                let im_a = vld1q_f32(buf.im[k..].as_ptr());
                let im_b = vld1q_f32(buf.im[k + 4..].as_ptr());
                let im_c = vld1q_f32(buf.im[k + 8..].as_ptr());
                let im_d = vld1q_f32(buf.im[k + 12..].as_ptr());

                let x0r = vcombine_f32(vget_low_f32(re_a), vget_low_f32(re_c));
                let x1r = vcombine_f32(vget_high_f32(re_a), vget_high_f32(re_c));
                let x2r = vcombine_f32(vget_low_f32(re_b), vget_low_f32(re_d));
                let x3r = vcombine_f32(vget_high_f32(re_b), vget_high_f32(re_d));
                let x0i = vcombine_f32(vget_low_f32(im_a), vget_low_f32(im_c));
                let x1i = vcombine_f32(vget_high_f32(im_a), vget_high_f32(im_c));
                let x2i = vcombine_f32(vget_low_f32(im_b), vget_low_f32(im_d));
                let x3i = vcombine_f32(vget_high_f32(im_b), vget_high_f32(im_d));

                let t1r = vmlsq_f32(vmulq_f32(w2r, x1r), w2i, x1i);
                let t1i = vmlaq_f32(vmulq_f32(w2r, x1i), w2i, x1r);
                let t3r = vmlsq_f32(vmulq_f32(w2r, x3r), w2i, x3i);
                let t3i = vmlaq_f32(vmulq_f32(w2r, x3i), w2i, x3r);

                let ar = vaddq_f32(x0r, t1r);
                let ai = vaddq_f32(x0i, t1i);
                let br = vsubq_f32(x0r, t1r);
                let bi = vsubq_f32(x0i, t1i);
                let cr = vaddq_f32(x2r, t3r);
                let ci = vaddq_f32(x2i, t3i);
                let dr = vsubq_f32(x2r, t3r);
                let di = vsubq_f32(x2i, t3i);

                let er = vmlsq_f32(vmulq_f32(w1r, cr), w1i, ci);
                let ei = vmlaq_f32(vmulq_f32(w1r, ci), w1i, cr);
                let fr = vmlsq_f32(vmulq_f32(w1r, dr), w1i, di);
                let fi = vmlaq_f32(vmulq_f32(w1r, di), w1i, dr);

                let y0r = vaddq_f32(ar, er);
                let y0i = vaddq_f32(ai, ei);
                let y1r = vaddq_f32(br, fi);
                let y1i = vsubq_f32(bi, fr);
                let y2r = vsubq_f32(ar, er);
                let y2i = vsubq_f32(ai, ei);
                let y3r = vsubq_f32(br, fi);
                let y3i = vaddq_f32(bi, fr);

                let re_a_out = vcombine_f32(vget_low_f32(y0r), vget_low_f32(y1r));
                let re_b_out = vcombine_f32(vget_low_f32(y2r), vget_low_f32(y3r));
                let re_c_out = vcombine_f32(vget_high_f32(y0r), vget_high_f32(y1r));
                let re_d_out = vcombine_f32(vget_high_f32(y2r), vget_high_f32(y3r));
                let im_a_out = vcombine_f32(vget_low_f32(y0i), vget_low_f32(y1i));
                let im_b_out = vcombine_f32(vget_low_f32(y2i), vget_low_f32(y3i));
                let im_c_out = vcombine_f32(vget_high_f32(y0i), vget_high_f32(y1i));
                let im_d_out = vcombine_f32(vget_high_f32(y2i), vget_high_f32(y3i));

                vst1q_f32(buf.re[k..].as_mut_ptr(), re_a_out);
                vst1q_f32(buf.re[k + 4..].as_mut_ptr(), re_b_out);
                vst1q_f32(buf.re[k + 8..].as_mut_ptr(), re_c_out);
                vst1q_f32(buf.re[k + 12..].as_mut_ptr(), re_d_out);
                vst1q_f32(buf.im[k..].as_mut_ptr(), im_a_out);
                vst1q_f32(buf.im[k + 4..].as_mut_ptr(), im_b_out);
                vst1q_f32(buf.im[k + 8..].as_mut_ptr(), im_c_out);
                vst1q_f32(buf.im[k + 12..].as_mut_ptr(), im_d_out);
                k += 16;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Public dispatch shims
    // -----------------------------------------------------------------------
    #[inline]
    pub(crate) unsafe fn r2_soa_narrow_neon<const N: usize>(
        buf: &mut FftBuf<N>,
        tw_off: usize,
        half_m: usize,
    ) {
        if half_m == 1 {
            unsafe {
                r2_soa_hm1_neon::<N>(buf, tw_off);
            }
        } else {
            unsafe {
                r2_soa_hm2_neon::<N>(buf, tw_off);
            }
        }
    }

    #[inline]
    pub(crate) unsafe fn r4_soa_narrow_neon<const N: usize>(
        buf: &mut FftBuf<N>,
        qm: usize,
        tw2_off: usize,
        tw1_off: usize,
    ) {
        if qm == 1 {
            unsafe {
                r4_soa_qm1_neon::<N>(buf);
            }
        } else {
            unsafe {
                r4_soa_qm2_neon::<N>(buf, tw2_off, tw1_off);
            }
        }
    }
}

#[cfg(all(target_arch = "arm", target_feature = "neon"))]
pub(crate) use neon::{
    r2_soa_narrow_neon, r2_soa_wide_neon, r4_soa_narrow_neon, r4_soa_wide_neon, r8_soa_wide_neon,
};
