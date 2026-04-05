//! `deluge-fft` — portable-SIMD, no-std, const-generic FFT crate.
//!
//! # Design
//!
//! * **No heap** — all working memory is on the stack; no global allocator needed.
//! * **Compile-time twiddle table** — `W_N^k` factors are evaluated once at
//!   compile time via Taylor-series `const fn`s over `f64`, stored in
//!   Flash/ROM on embedded targets.  The hot loop reads a flat table and does
//!   arithmetic — zero runtime trig cost.
//! * **Portable SIMD** — butterfly inner loops operate on
//!   `core::simd::Simd<f32, LANES>`.  On Cortex-A9 with `+neon`, LLVM emits
//!   `float32x4_t` instructions for `LANES = 4`.
//! * **Multiple layouts** — AoS `[Complex; N]` for interop; SoA `FftBuf<N>`
//!   for maximally-vectorisable sequential loads/stores.
//! * **Radix-4** — `process_r4_simd` / `process_r4_simd_soa` merge stage
//!   pairs for ~2× fewer passes over the array.
//! * **Real-input FFT** — `RealFft<N, LANES>` packs N real samples into a
//!   length-N/2 complex FFT + post-processing, ~2× faster for real audio.

#![no_std]
#![feature(portable_simd)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![cfg_attr(
    all(target_arch = "arm", target_feature = "neon"),
    feature(stdarch_arm_neon_intrinsics)
)]

mod arch;
pub mod buf;
mod complex;
mod fft;
mod radix4;
mod radix8;
mod real_fft;
mod spectrum;
mod trig;
mod twiddle;

pub use buf::FftBuf;
pub use complex::Complex;
pub use fft::Fft;
pub use radix4::{process_r4_simd, process_r4_simd_soa};
pub use radix8::process_r8_simd_soa;
pub use real_fft::RealFft;
pub use spectrum::{
    apply_hann_window, apply_hann_window_real, apply_hann_window_soa, magnitude_spectrum,
    magnitude_spectrum_soa,
};

#[cfg(any(test, feature = "test-utils"))]
pub mod test_utils;

#[cfg(test)]
mod tests;
