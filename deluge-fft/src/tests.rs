use crate::buf::FftBuf;
use crate::complex::Complex;
use crate::fft::Fft;
use crate::radix4::{process_r4_simd, process_r4_simd_soa};
use crate::radix8::process_r8_simd_soa;
use crate::real_fft::RealFft;
use crate::spectrum::{apply_hann_window, apply_hann_window_soa};
use crate::test_utils::{dft, max_error};
use crate::trig::{TWO_PI, sin_f32};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn impulse_aos<const N: usize>() -> [Complex; N] {
    let mut buf = [Complex::ZERO; N];
    buf[0] = Complex::new(1.0, 0.0);
    buf
}

fn impulse_soa<const N: usize>() -> FftBuf<N> {
    let mut buf = FftBuf::ZERO;
    buf.re[0] = 1.0;
    buf
}

fn sine_aos<const N: usize>(freq_bin: usize) -> [Complex; N] {
    let mut buf = [Complex::ZERO; N];
    for i in 0..N {
        let phase = (freq_bin * i) % N;
        buf[i].re = sin_f32(TWO_PI * phase as f64 / N as f64);
    }
    buf
}

fn sine_real<const N: usize>(freq_bin: usize) -> [f32; N] {
    let mut buf = [0f32; N];
    for i in 0..N {
        let phase = (freq_bin * i) % N;
        buf[i] = sin_f32(TWO_PI * phase as f64 / N as f64);
    }
    buf
}

// ---------------------------------------------------------------------------
// AoS radix-2 (existing tests, kept for regression)
// ---------------------------------------------------------------------------

#[test]
fn impulse_scalar_8() {
    let mut buf = impulse_aos::<8>();
    Fft::<8, 1>::process(&mut buf);
    for c in buf.iter() {
        assert!((c.re - 1.0).abs() < 1e-5, "re={}", c.re);
        assert!(c.im.abs() < 1e-5, "im={}", c.im);
    }
}

#[test]
fn impulse_simd4_64() {
    let mut a = impulse_aos::<64>();
    let mut b = a;
    Fft::<64, 4>::process_simd(&mut a);
    Fft::<64, 1>::process(&mut b);
    let err = max_error(&a, &b);
    assert!(err < 1e-4, "SIMD4 vs scalar: {err}");
}

#[test]
fn sine_512_bin5_peak() {
    let mut buf = sine_aos::<512>(5);
    Fft::<512, 4>::process_simd(&mut buf);
    let peak = buf
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.norm_sq().partial_cmp(&b.norm_sq()).unwrap())
        .map(|(i, _)| i)
        .unwrap();
    assert!(peak == 5 || peak == 507, "peak at {peak}");
}

#[test]
fn simd_matches_scalar_512() {
    let base = sine_aos::<512>(7);
    let mut a = base;
    let mut b = base;
    Fft::<512, 1>::process(&mut a);
    Fft::<512, 4>::process_simd(&mut b);
    let err = max_error(&a, &b);
    assert!(err < 5e-3, "SIMD/scalar mismatch: {err}");
}

#[test]
fn simd_matches_dft_64() {
    let base = sine_aos::<64>(3);
    let expected = dft(&base);
    let mut got = base;
    Fft::<64, 4>::process_simd(&mut got);
    let err = max_error(&got, &expected);
    assert!(err < 1e-3, "vs DFT: {err}");
}

#[test]
fn hann_no_nan_aos() {
    let mut buf = sine_aos::<256>(10);
    apply_hann_window(&mut buf);
    Fft::<256, 4>::process_simd(&mut buf);
    for c in buf.iter() {
        assert!(c.re.is_finite() && c.im.is_finite());
    }
}

// ---------------------------------------------------------------------------
// SoA radix-2
// ---------------------------------------------------------------------------

#[test]
fn soa_scalar_matches_aos_scalar_64() {
    let base = sine_aos::<64>(3);
    let mut aos = base;
    let mut soa = FftBuf::from_complex(&base);
    Fft::<64, 1>::process(&mut aos);
    Fft::<64, 1>::process_soa(&mut soa);
    let soa_c = soa.to_complex();
    let err = max_error(&aos, &soa_c);
    assert!(err < 1e-4, "SoA scalar vs AoS scalar: {err}");
}

#[test]
fn soa_simd_matches_aos_simd_512() {
    let base = sine_aos::<512>(7);
    let mut aos = base;
    let mut soa = FftBuf::from_complex(&base);
    Fft::<512, 4>::process_simd(&mut aos);
    Fft::<512, 4>::process_simd_soa(&mut soa);
    let soa_c = soa.to_complex();
    let err = max_error(&aos, &soa_c);
    assert!(err < 5e-3, "SoA SIMD vs AoS SIMD: {err}");
}

#[test]
fn soa_impulse_8() {
    let mut buf = impulse_soa::<8>();
    Fft::<8, 1>::process_soa(&mut buf);
    for i in 0..8 {
        assert!((buf.re[i] - 1.0).abs() < 1e-5, "re[{}]={}", i, buf.re[i]);
        assert!(buf.im[i].abs() < 1e-5, "im[{}]={}", i, buf.im[i]);
    }
}

#[test]
fn hann_no_nan_soa() {
    let mut buf = FftBuf::from_complex(&sine_aos::<256>(10));
    apply_hann_window_soa(&mut buf);
    Fft::<256, 4>::process_simd_soa(&mut buf);
    for i in 0..256 {
        assert!(buf.re[i].is_finite() && buf.im[i].is_finite());
    }
}

// ---------------------------------------------------------------------------
// Radix-4 correctness: must match radix-2 output
// ---------------------------------------------------------------------------

#[test]
fn r4_aos_matches_r2_64() {
    let base = sine_aos::<64>(3);
    let mut r2 = base;
    let mut r4 = base;
    Fft::<64, 4>::process_simd(&mut r2);
    process_r4_simd::<64, 4>(&mut r4);
    let err = max_error(&r2, &r4);
    assert!(err < 1e-3, "r4 AoS vs r2 AoS (N=64): {err}");
}

#[test]
fn r4_aos_matches_r2_256() {
    let base = sine_aos::<256>(11);
    let mut r2 = base;
    let mut r4 = base;
    Fft::<256, 4>::process_simd(&mut r2);
    process_r4_simd::<256, 4>(&mut r4);
    let err = max_error(&r2, &r4);
    assert!(err < 1e-3, "r4 AoS vs r2 AoS (N=256): {err}");
}

#[test]
fn r4_aos_matches_r2_512() {
    let base = sine_aos::<512>(7);
    let mut r2 = base;
    let mut r4 = base;
    Fft::<512, 4>::process_simd(&mut r2);
    process_r4_simd::<512, 4>(&mut r4);
    let err = max_error(&r2, &r4);
    assert!(err < 1e-3, "r4 AoS vs r2 AoS (N=512): {err}");
}

#[test]
fn r4_soa_matches_r2_512() {
    let base = sine_aos::<512>(7);
    let mut r2 = base;
    let mut r4 = FftBuf::from_complex(&base);
    Fft::<512, 4>::process_simd(&mut r2);
    process_r4_simd_soa::<512, 4>(&mut r4);
    let r4_c = r4.to_complex();
    let err = max_error(&r2, &r4_c);
    assert!(err < 1e-3, "r4 SoA vs r2 AoS (N=512): {err}");
}

// ---------------------------------------------------------------------------
// Real-input FFT
// ---------------------------------------------------------------------------

// Helper: run a full complex FFT and extract the first N/2+1 bins.
fn complex_fft_bins<const N: usize>(real_input: &[f32; N]) -> [Complex; { N / 2 + 1 }] {
    let mut buf: [Complex; N] = [Complex::ZERO; N];
    for i in 0..N {
        buf[i].re = real_input[i];
    }
    Fft::<N, 4>::process_simd(&mut buf);
    let mut out = [Complex::ZERO; { N / 2 + 1 }];
    for i in 0..=N / 2 {
        out[i] = buf[i];
    }
    out
}

fn max_error_slice(a: &[Complex], b: &[Complex]) -> f32 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| {
            Complex {
                re: x.re - y.re,
                im: x.im - y.im,
            }
            .abs()
        })
        .fold(0.0f32, f32::max)
}

#[test]
fn real_fft_impulse_512() {
    let mut input = [0f32; 512];
    input[0] = 1.0;
    let mut out = [Complex::ZERO; { 512 / 2 + 1 }];
    RealFft::<512, 4>::process(&input, &mut out);
    // All bins should be 1+0j.
    for (i, c) in out.iter().enumerate() {
        assert!((c.re - 1.0).abs() < 1e-3, "bin {i} re={}", c.re);
        assert!(c.im.abs() < 1e-3, "bin {i} im={}", c.im);
    }
}

#[test]
fn real_fft_matches_complex_fft_512() {
    let input = sine_real::<512>(5);
    let mut out_real = [Complex::ZERO; { 512 / 2 + 1 }];
    RealFft::<512, 4>::process(&input, &mut out_real);
    let out_cx = complex_fft_bins::<512>(&input);
    let err = max_error_slice(&out_real, &out_cx);
    assert!(err < 1e-2, "RealFft vs complex FFT: {err}");
}

#[test]
fn real_fft_sine_peak_512() {
    let input = sine_real::<512>(13);
    let mut out = [Complex::ZERO; { 512 / 2 + 1 }];
    RealFft::<512, 4>::process(&input, &mut out);
    let peak = out
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| {
            (a.re * a.re + a.im * a.im)
                .partial_cmp(&(b.re * b.re + b.im * b.im))
                .unwrap()
        })
        .map(|(i, _)| i)
        .unwrap();
    assert_eq!(peak, 13, "peak at {peak}, expected 13");
}

// ---------------------------------------------------------------------------
// Radix-8 SoA
// ---------------------------------------------------------------------------

#[test]
fn r8_soa_matches_r4_soa_512() {
    let base = sine_aos::<512>(7);
    let mut r4 = FftBuf::from_complex(&base);
    let mut r8 = FftBuf::from_complex(&base);
    process_r4_simd_soa::<512, 4>(&mut r4);
    process_r8_simd_soa::<512, 4>(&mut r8);
    let r4_c = r4.to_complex();
    let r8_c = r8.to_complex();
    let err = max_error(&r4_c, &r8_c);
    assert!(err < 1e-3, "r8 SoA vs r4 SoA (N=512): {err}");
}

#[test]
fn r8_soa_matches_r4_soa_1024() {
    let base = sine_aos::<1024>(13);
    let mut r4 = FftBuf::from_complex(&base);
    let mut r8 = FftBuf::from_complex(&base);
    process_r4_simd_soa::<1024, 4>(&mut r4);
    process_r8_simd_soa::<1024, 4>(&mut r8);
    let r4_c = r4.to_complex();
    let r8_c = r8.to_complex();
    let err = max_error(&r4_c, &r8_c);
    assert!(err < 1e-3, "r8 SoA vs r4 SoA (N=1024): {err}");
}

#[test]
fn r8_soa_matches_r4_soa_256() {
    let base = sine_aos::<256>(5);
    let mut r4 = FftBuf::from_complex(&base);
    let mut r8 = FftBuf::from_complex(&base);
    process_r4_simd_soa::<256, 4>(&mut r4);
    process_r8_simd_soa::<256, 4>(&mut r8);
    let r4_c = r4.to_complex();
    let r8_c = r8.to_complex();
    let err = max_error(&r4_c, &r8_c);
    assert!(err < 1e-3, "r8 SoA vs r4 SoA (N=256): {err}");
}
