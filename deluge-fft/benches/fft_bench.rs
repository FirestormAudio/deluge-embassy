#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

//! Benchmark suite: deluge-fft vs RustFFT
//!
//! # How to run
//!
//! ## Native (host CPU, no special flags)
//! ```sh
//! cargo bench --target x86_64-unknown-linux-gnu \
//!   --manifest-path deluge-fft/Cargo.toml
//! ```
//!
//! ## Native with cpu=native (enables AVX2/AVX-512 etc.)
//! ```sh
//! RUSTFLAGS="-C target-cpu=native" \
//!   cargo bench --target x86_64-unknown-linux-gnu \
//!   --manifest-path deluge-fft/Cargo.toml
//! ```
//!
//! ## QEMU Cortex-A9 NEON (armv7-unknown-linux-gnueabihf + qemu-arm)
//! ```sh
//! cargo bench --target armv7-unknown-linux-gnueabihf \
//!   --manifest-path deluge-fft/Cargo.toml
//! ```
//! (Requires `[target.armv7-unknown-linux-gnueabihf]` runner in .cargo/config.toml)
//!
//! Criterion HTML reports land in `target/criterion/`.

use criterion::{BatchSize, BenchmarkId, Criterion, black_box, criterion_group, criterion_main};

#[cfg(feature = "cmsis-bench")]
use cmsis_dsp::fft::{CfftF32, RfftF32};
use deluge_fft::{
    Complex, Fft, FftBuf, RealFft, process_r4_simd, process_r4_simd_soa, process_r8_simd_soa,
};
use rustfft::{FftPlanner, num_complex::Complex as RComplex};

// ---------------------------------------------------------------------------
// Sizes to benchmark.  Typical audio analysis windows on the Deluge.
// ---------------------------------------------------------------------------
const SIZES: &[usize] = &[256, 512, 1024, 2048];

// ---------------------------------------------------------------------------
// Input generators (setup phase — not included in measured time)
// ---------------------------------------------------------------------------

fn make_input_aos<const N: usize>() -> [Complex; N] {
    let mut buf = [Complex::ZERO; N];
    for i in 0..N {
        buf[i].re = (i as f32) * 0.001;
        buf[i].im = -(i as f32) * 0.0005;
    }
    buf
}

fn make_input_soa<const N: usize>() -> FftBuf<N> {
    let mut buf = FftBuf::ZERO;
    for i in 0..N {
        buf.re[i] = (i as f32) * 0.001;
        buf.im[i] = -(i as f32) * 0.0005;
    }
    buf
}

fn make_real_input<const N: usize>() -> [f32; N] {
    let mut buf = [0f32; N];
    for i in 0..N {
        buf[i] = (i as f32) * 0.001;
    }
    buf
}

// ---------------------------------------------------------------------------
// deluge-fft: radix-2 scalar  (Fft<N, 1>::process)
// ---------------------------------------------------------------------------
fn bench_deluge_scalar(c: &mut Criterion) {
    let mut g = c.benchmark_group("deluge/r2_scalar");
    macro_rules! one {
        ($n:expr) => {{
            let template = make_input_aos::<$n>();
            g.bench_with_input(BenchmarkId::from_parameter($n), &$n, |b, _| {
                b.iter_batched(
                    || template,
                    |mut buf| Fft::<$n, 1>::process(black_box(&mut buf)),
                    BatchSize::SmallInput,
                );
            });
        }};
    }
    one!(256);
    one!(512);
    one!(1024);
    one!(2048);
    g.finish();
}

// ---------------------------------------------------------------------------
// deluge-fft: radix-2 SIMD4 AoS  (Fft<N, 4>::process_simd)
// ---------------------------------------------------------------------------
fn bench_deluge_simd4_aos(c: &mut Criterion) {
    let mut g = c.benchmark_group("deluge/r2_simd4_aos");
    macro_rules! one {
        ($n:expr) => {{
            let template = make_input_aos::<$n>();
            g.bench_with_input(BenchmarkId::from_parameter($n), &$n, |b, _| {
                b.iter_batched(
                    || template,
                    |mut buf| Fft::<$n, 4>::process_simd(black_box(&mut buf)),
                    BatchSize::SmallInput,
                );
            });
        }};
    }
    one!(256);
    one!(512);
    one!(1024);
    one!(2048);
    g.finish();
}

// ---------------------------------------------------------------------------
// deluge-fft: radix-2 SIMD4 SoA  (Fft<N, 4>::process_simd_soa)
// ---------------------------------------------------------------------------
fn bench_deluge_simd4_soa(c: &mut Criterion) {
    let mut g = c.benchmark_group("deluge/r2_simd4_soa");
    macro_rules! one {
        ($n:expr) => {{
            let template = make_input_soa::<$n>();
            g.bench_with_input(BenchmarkId::from_parameter($n), &$n, |b, _| {
                b.iter_batched(
                    || template.clone(),
                    |mut buf| Fft::<$n, 4>::process_simd_soa(black_box(&mut buf)),
                    BatchSize::SmallInput,
                );
            });
        }};
    }
    one!(256);
    one!(512);
    one!(1024);
    one!(2048);
    g.finish();
}

// ---------------------------------------------------------------------------
// deluge-fft: radix-4 SIMD4 AoS  (process_r4_simd<N, 4>)
// ---------------------------------------------------------------------------
fn bench_deluge_r4_simd4_aos(c: &mut Criterion) {
    let mut g = c.benchmark_group("deluge/r4_simd4_aos");
    macro_rules! one {
        ($n:expr) => {{
            let template = make_input_aos::<$n>();
            g.bench_with_input(BenchmarkId::from_parameter($n), &$n, |b, _| {
                b.iter_batched(
                    || template,
                    |mut buf| process_r4_simd::<$n, 4>(black_box(&mut buf)),
                    BatchSize::SmallInput,
                );
            });
        }};
    }
    one!(256);
    one!(512);
    one!(1024);
    one!(2048);
    g.finish();
}

// ---------------------------------------------------------------------------
// deluge-fft: radix-4 SIMD4 SoA  (process_r4_simd_soa<N, 4>)
// ---------------------------------------------------------------------------
fn bench_deluge_r4_simd4_soa(c: &mut Criterion) {
    let mut g = c.benchmark_group("deluge/r4_simd4_soa");
    macro_rules! one {
        ($n:expr) => {{
            let template = make_input_soa::<$n>();
            g.bench_with_input(BenchmarkId::from_parameter($n), &$n, |b, _| {
                b.iter_batched(
                    || template.clone(),
                    |mut buf| process_r4_simd_soa::<$n, 4>(black_box(&mut buf)),
                    BatchSize::SmallInput,
                );
            });
        }};
    }
    one!(256);
    one!(512);
    one!(1024);
    one!(2048);
    g.finish();
}

fn bench_deluge_r8_simd4_soa(c: &mut Criterion) {
    let mut g = c.benchmark_group("deluge/r8_simd4_soa");
    macro_rules! one {
        ($n:expr) => {{
            let template = make_input_soa::<$n>();
            g.bench_with_input(BenchmarkId::from_parameter($n), &$n, |b, _| {
                b.iter_batched(
                    || template.clone(),
                    |mut buf| process_r8_simd_soa::<$n, 4>(black_box(&mut buf)),
                    BatchSize::SmallInput,
                );
            });
        }};
    }
    one!(256);
    one!(512);
    one!(1024);
    one!(2048);
    g.finish();
}

// ---------------------------------------------------------------------------
// deluge-fft: real FFT  (RealFft<N, 4>::process)
//
// Input is pure-real; this runs a half-size complex FFT + twiddle post-pass.
// ---------------------------------------------------------------------------
fn bench_deluge_real(c: &mut Criterion) {
    let mut g = c.benchmark_group("deluge/real_simd4");
    macro_rules! one {
        ($n:expr) => {{
            let input = make_real_input::<$n>();
            g.bench_with_input(BenchmarkId::from_parameter($n), &$n, |b, _| {
                let mut out = [Complex::ZERO; $n / 2 + 1];
                b.iter(|| {
                    // Input is read-only so no need for iter_batched here.
                    RealFft::<$n, 4>::process(black_box(&input), black_box(&mut out));
                });
            });
        }};
    }
    one!(256);
    one!(512);
    one!(1024);
    one!(2048);
    g.finish();
}

// ---------------------------------------------------------------------------
// RustFFT: complex FFT (planned, heap-allocated, runtime-dispatch SIMD)
// ---------------------------------------------------------------------------
fn bench_rustfft_complex(c: &mut Criterion) {
    let mut g = c.benchmark_group("rustfft/complex");
    for &n in SIZES {
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(n);
        let template: Vec<RComplex<f32>> = (0..n)
            .map(|i| RComplex::new(i as f32 * 0.001, -(i as f32) * 0.0005))
            .collect();
        let scratch_len = fft.get_inplace_scratch_len();
        g.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter_batched(
                || (template.clone(), vec![RComplex::new(0.0, 0.0); scratch_len]),
                |(mut buf, mut scratch)| {
                    fft.process_with_scratch(black_box(&mut buf), black_box(&mut scratch))
                },
                BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

// ---------------------------------------------------------------------------
// RustFFT: half-size complex FFT — matches the inner FFT cost of RealFft<N>
//
// Pack N real samples as N/2 complex z[k] = x[2k] + j*x[2k+1], run N/2
// complex FFT.  Comparable to deluge/real_simd4 minus the twiddle post-pass.
// ---------------------------------------------------------------------------
fn bench_rustfft_real(c: &mut Criterion) {
    let mut g = c.benchmark_group("rustfft/real_half_fft");
    for &n in SIZES {
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(n / 2);
        let real_input: Vec<f32> = (0..n).map(|i| i as f32 * 0.001).collect();
        let packed_template: Vec<RComplex<f32>> = (0..n / 2)
            .map(|k| RComplex::new(real_input[2 * k], real_input[2 * k + 1]))
            .collect();
        let scratch_len = fft.get_inplace_scratch_len();
        g.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter_batched(
                || {
                    (
                        packed_template.clone(),
                        vec![RComplex::new(0.0, 0.0); scratch_len],
                    )
                },
                |(mut buf, mut scratch)| {
                    fft.process_with_scratch(black_box(&mut buf), black_box(&mut scratch))
                },
                BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

// ---------------------------------------------------------------------------
// CMSIS-DSP: complex FFT f32
//
// On ARM+NEON targets: out-of-place API — three separate buffers required.
// On x86 (host): scalar in-place API.
//
// The instance is initialised once and reused for all iterations.
// ---------------------------------------------------------------------------
#[cfg(feature = "cmsis-bench")]
fn bench_cmsis_cfft(c: &mut Criterion) {
    let mut g = c.benchmark_group("cmsis-dsp/cfft_f32");
    for &n in SIZES {
        let instance = CfftF32::new(n as u16).expect("cmsis cfft init failed");
        let src: Vec<f32> = (0..2 * n)
            .map(|i| {
                if i % 2 == 0 {
                    i as f32 * 0.001
                } else {
                    -(i as f32) * 0.0005
                }
            })
            .collect();

        #[cfg(target_arch = "arm")]
        g.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            let mut dst = vec![0.0f32; 2 * n];
            let mut buf = vec![0.0f32; 2 * n];
            b.iter(|| {
                instance.process_forward(black_box(&src), black_box(&mut dst), black_box(&mut buf))
            });
        });

        #[cfg(not(target_arch = "arm"))]
        g.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter_batched(
                || src.clone(),
                |mut data| instance.process_forward(black_box(&mut data)),
                BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

// ---------------------------------------------------------------------------
// CMSIS-DSP: real FFT fast f32
// ---------------------------------------------------------------------------
#[cfg(feature = "cmsis-bench")]
fn bench_cmsis_rfft(c: &mut Criterion) {
    let mut g = c.benchmark_group("cmsis-dsp/rfft_fast_f32");
    for &n in SIZES {
        let instance = RfftF32::new(n as u16).expect("cmsis rfft init failed");
        let real_input: Vec<f32> = (0..n).map(|i| i as f32 * 0.001).collect();

        #[cfg(target_arch = "arm")]
        g.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            let mut dst = vec![0.0f32; n];
            let mut buf = vec![0.0f32; n];
            b.iter(|| {
                instance.process_forward(
                    black_box(&real_input),
                    black_box(&mut dst),
                    black_box(&mut buf),
                )
            });
        });

        #[cfg(not(target_arch = "arm"))]
        g.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter_batched(
                || (real_input.clone(), vec![0.0f32; n]),
                |(mut p, mut out)| instance.process_forward(black_box(&mut p), black_box(&mut out)),
                BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

// ---------------------------------------------------------------------------
// Criterion entry points
// ---------------------------------------------------------------------------
criterion_group!(
    deluge_benches,
    bench_deluge_scalar,
    bench_deluge_simd4_aos,
    bench_deluge_simd4_soa,
    bench_deluge_r4_simd4_aos,
    bench_deluge_r4_simd4_soa,
    bench_deluge_r8_simd4_soa,
    bench_deluge_real,
);
criterion_group!(rustfft_benches, bench_rustfft_complex, bench_rustfft_real);
#[cfg(feature = "cmsis-bench")]
criterion_group!(cmsis_benches, bench_cmsis_cfft, bench_cmsis_rfft);
#[cfg(feature = "cmsis-bench")]
criterion_main!(deluge_benches, rustfft_benches, cmsis_benches);
#[cfg(not(feature = "cmsis-bench"))]
criterion_main!(deluge_benches, rustfft_benches);
