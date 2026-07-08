//! Host-side DSP measurement & validation harness for the Deluge audio suites.
//!
//! Operates on `&[f32]` buffers and `FnMut(&mut [f32])` renderers — it never
//! depends on the kernel or graph crates, so any suite can pull it in under
//! `[dev-dependencies]` and assert on the returned metrics.

/// FFT size used by [`spectrum`] analysis. A fixed default for caller simplicity;
/// `realfft` plans any size at runtime, so this isn't a hard constraint.
pub const FFT_N: usize = 8192;

pub mod guards;
pub mod spectrum;
pub mod cpu;
pub mod filter_meas;
