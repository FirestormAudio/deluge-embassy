# QA — DSP measurement & validation harness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `deluge-dsp-test`, a host-side measurement toolkit — FFT-based spectral analysis (frequency response, THD, aliasing), a comparison-focused CPU-cost harness, and numeric guards/null-tests — that the vocabulary suites use to prove their DSP.

**Architecture:** A new `std`, host-only crate (workspace-*excluded*, like the other host tooling) depending only on `deluge-fft`. It operates on `&[f32]` buffers and `FnMut(&mut [f32])` renderer closures, never on kernel/graph types. Spectral analysis is Hann-window → `deluge_fft::RealFft` → magnitudes → relative metrics; the CPU harness is median-of-N wall-clock for A/B comparison (not an absolute gate); guards are the reviewed home for finite/bounded/denormal/null checks.

**Tech Stack:** Rust, `std` (host-only), nightly toolchain (pinned), `deluge-fft` (portable-SIMD real FFT). Tests run on `x86_64-unknown-linux-gnu`.

**Reference spec:** [QA design](../specs/2026-07-07-qa-measurement-harness-design.md). Depends on P0 only via `deluge-fft` (in the workspace).

## Global Constraints

- `deluge-dsp-test` is **`std`, host-only**, and **excluded from the workspace** (`cargo build --workspace` on the device target must not try to build it). It is used by suites as a `[dev-dependencies]` path dep.
- Its only workspace dependency is `deluge-fft` (path). Package fields inherited from `[workspace.package]`.
- **Fixed FFT size:** `pub const FFT_N: usize = 8192;` with `LANES = 4`. The spec's generic `analyze::<N>` is realized as a fixed-`N` function to avoid propagating `deluge-fft`'s `generic_const_exprs` bounds through this crate (a concrete `RealFft::<8192, 4>` call needs no such feature).
- Spectral metrics are **relative** (dB relative to the fundamental or the peak), so window/scale factors cancel — do not correct for Hann coherent gain.
- CPU timing is **relative/comparison only** — never assert absolute ns/block; regression assertions use wide tolerance.
- Test command (this crate is excluded, so use `--manifest-path` + the host target):
  `cargo test --manifest-path crates/deluge-dsp-test/Cargo.toml --target x86_64-unknown-linux-gnu`
- Test output pristine (no warnings). Commit after each task.

---

## File structure

**`crates/deluge-dsp-test/`** (new, std, workspace-excluded):
- `Cargo.toml`
- `src/lib.rs` — crate root; `FFT_N`/`LANES` consts; module decls + re-exports.
- `src/guards.rs` — `assert_finite_bounded`, `assert_no_denormals`, `max_abs_diff`, `null`, `rms`.
- `src/spectrum.rs` — `Spectrum`, `analyze`/`analyze_buf`, `peak_*`/`level_at`, and the harmonic metrics (`harmonics_db`/`thd`/`worst_alias_db`/`noise_floor_db`).
- `src/cpu.rs` — `CostReport`, `measure`, `compare`.

**Workspace root `Cargo.toml`** — add `crates/deluge-dsp-test` to the `exclude` list.

---

## Task 1: Scaffold + guards

**Files:**
- Create: `crates/deluge-dsp-test/Cargo.toml`, `crates/deluge-dsp-test/src/lib.rs`, `crates/deluge-dsp-test/src/guards.rs`
- Modify: `Cargo.toml` (workspace `exclude`)

**Interfaces:**
- Produces:
  - `pub const FFT_N: usize = 8192;` `pub const LANES: usize = 4;`
  - `guards::assert_finite_bounded(buf: &[f32], max_abs: f32)`
  - `guards::assert_no_denormals(buf: &[f32])`
  - `guards::max_abs_diff(a: &[f32], b: &[f32]) -> f32`
  - `guards::null(a: &[f32], b: &[f32], tol: f32)`
  - `guards::rms(buf: &[f32]) -> f32`

- [ ] **Step 1: Exclude the crate from the workspace**

In `/home/kate/GitHub/deluge-sdk/Cargo.toml`, add to the `exclude` array (next to the other host-only tooling like `tools/wren-web-debug`):

```toml
  "crates/deluge-dsp-test",
```

- [ ] **Step 2: Write `Cargo.toml`**

`crates/deluge-dsp-test/Cargo.toml`:

```toml
[package]
name = "deluge-dsp-test"
version.workspace = true
edition.workspace = true
authors.workspace = true
license.workspace = true
repository.workspace = true
homepage.workspace = true
description = "Host-side DSP measurement & validation harness (spectral analysis, CPU cost, guards) for the Deluge audio suites"
categories = ["development-tools::testing", "multimedia::audio"]
keywords = ["dsp", "audio", "testing", "fft", "spectral"]

[dependencies]
deluge-fft = { path = "../deluge-fft" }

[lib]
name = "deluge_dsp_test"
path = "src/lib.rs"
```

- [ ] **Step 3: Write the failing guards test**

`crates/deluge-dsp-test/src/guards.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finite_bounded_passes_clean_and_catches_nan() {
        assert_finite_bounded(&[0.0, 0.5, -1.0], 1.0); // clean
        let r = std::panic::catch_unwind(|| assert_finite_bounded(&[0.0, f32::NAN], 1.0));
        assert!(r.is_err(), "NaN must trip the guard");
        let r = std::panic::catch_unwind(|| assert_finite_bounded(&[2.0], 1.0));
        assert!(r.is_err(), "out-of-bounds must trip the guard");
    }

    #[test]
    fn denormals_are_caught() {
        assert_no_denormals(&[0.0, 1.0, -0.5]); // zero + normals OK
        let sub = f32::MIN_POSITIVE / 2.0; // a subnormal
        let r = std::panic::catch_unwind(|| assert_no_denormals(&[sub]));
        assert!(r.is_err(), "subnormal must trip the guard");
    }

    #[test]
    fn diff_null_and_rms() {
        let a = [1.0, -1.0, 1.0, -1.0];
        assert_eq!(max_abs_diff(&a, &a), 0.0);
        null(&a, &a, 0.0);
        assert!((max_abs_diff(&[0.0, 0.0], &[0.1, -0.2]) - 0.2).abs() < 1e-6);
        assert!((rms(&a) - 1.0).abs() < 1e-6);
    }
}
```

- [ ] **Step 4: Run, verify failure**

Run: `cargo test --manifest-path crates/deluge-dsp-test/Cargo.toml --target x86_64-unknown-linux-gnu`
Expected: FAIL — `assert_finite_bounded` etc. not found (and `lib.rs`/module not yet wired).

- [ ] **Step 5: Write `lib.rs` + the guards impl**

`crates/deluge-dsp-test/src/lib.rs`:

```rust
//! Host-side DSP measurement & validation harness for the Deluge audio suites.
//!
//! Operates on `&[f32]` buffers and `FnMut(&mut [f32])` renderers — it never
//! depends on the kernel or graph crates, so any suite can pull it in under
//! `[dev-dependencies]` and assert on the returned metrics.

/// FFT size used by [`spectrum`] analysis. Fixed (not generic) so callers need
/// no `generic_const_exprs`; a concrete `RealFft::<FFT_N, LANES>` monomorphizes.
pub const FFT_N: usize = 8192;
/// SIMD lane count for the real FFT (matches `deluge-fft`'s host tests).
pub const LANES: usize = 4;

pub mod guards;
pub mod spectrum;
pub mod cpu;
```

Prepend to `crates/deluge-dsp-test/src/guards.rs` (above the tests):

```rust
//! Numeric guards + null tests — the reviewed home for the finite/bounded/
//! denormal/agreement checks the suites would otherwise copy-paste.

/// Assert every sample is finite and within `[-max_abs, max_abs]`.
pub fn assert_finite_bounded(buf: &[f32], max_abs: f32) {
    for (i, &x) in buf.iter().enumerate() {
        assert!(x.is_finite(), "sample {i} is not finite: {x}");
        assert!(x.abs() <= max_abs, "sample {i} = {x} exceeds |{max_abs}|");
    }
}

/// Assert no sample is a subnormal (nonzero with magnitude below the smallest
/// normal float) — denormals wreck real-time performance on some FPUs.
pub fn assert_no_denormals(buf: &[f32]) {
    for (i, &x) in buf.iter().enumerate() {
        assert!(
            x == 0.0 || x.abs() >= f32::MIN_POSITIVE,
            "sample {i} = {x} is a subnormal"
        );
    }
}

/// Largest absolute difference between two equal-length buffers.
pub fn max_abs_diff(a: &[f32], b: &[f32]) -> f32 {
    assert_eq!(a.len(), b.len(), "buffers differ in length");
    a.iter().zip(b).map(|(x, y)| (x - y).abs()).fold(0.0, f32::max)
}

/// Assert two buffers agree within `tol` (a null test).
pub fn null(a: &[f32], b: &[f32], tol: f32) {
    let d = max_abs_diff(a, b);
    assert!(d <= tol, "buffers differ by {d} > tol {tol}");
}

/// Root-mean-square level of a buffer.
pub fn rms(buf: &[f32]) -> f32 {
    if buf.is_empty() {
        return 0.0;
    }
    let sum_sq: f32 = buf.iter().map(|x| x * x).sum();
    (sum_sq / buf.len() as f32).sqrt()
}
```

- [ ] **Step 6: Add empty module stubs so `lib.rs` compiles**

`lib.rs` declares `spectrum` and `cpu` (filled in later tasks). Create minimal stubs so this task compiles:
`crates/deluge-dsp-test/src/spectrum.rs`:
```rust
//! Spectral analysis (filled in Tasks 2–3).
```
`crates/deluge-dsp-test/src/cpu.rs`:
```rust
//! CPU-cost harness (filled in Task 4).
```

- [ ] **Step 7: Run, verify pass**

Run: `cargo test --manifest-path crates/deluge-dsp-test/Cargo.toml --target x86_64-unknown-linux-gnu`
Expected: PASS (3 guard tests), zero warnings.

- [ ] **Step 8: Commit**

```bash
git add crates/deluge-dsp-test Cargo.toml
git commit -m "feat(dsp-test): scaffold host measurement crate + numeric guards"
```

---

## Task 2: Spectral core — `Spectrum`, `analyze`, peak/level

**Files:**
- Modify: `crates/deluge-dsp-test/src/spectrum.rs`

**Interfaces:**
- Consumes: `FFT_N`, `LANES` (Task 1); `deluge_fft::{RealFft, Complex, apply_hann_window_real}`.
- Produces:
  - `pub struct Spectrum { pub bins: Vec<f32>, pub bin_hz: f32, pub sample_rate: f32 }` — `bins` are **linear** magnitudes, one-sided (`FFT_N/2 + 1` of them).
  - `pub fn analyze(sample_rate: f32, renderer: impl FnMut(&mut [f32])) -> Spectrum`
  - `pub fn analyze_buf(sample_rate: f32, signal: &[f32; FFT_N]) -> Spectrum`
  - `Spectrum::peak_bin(&self) -> usize`, `Spectrum::peak_linear(&self) -> f32`
  - `Spectrum::bin_of_hz(&self, hz: f32) -> usize`, `Spectrum::level_at(&self, hz: f32) -> f32` (linear, nearest bin)

- [ ] **Step 1: Write the failing test (pure sine → peak at f0)**

Add to `crates/deluge-dsp-test/src/spectrum.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::FFT_N;
    use std::f32::consts::TAU;

    /// A sine whose frequency lands exactly on an FFT bin (no leakage bias):
    /// bin_hz = sr/N, so f0 = 64*bin_hz is exactly bin 64.
    fn bin_centered_f0(sample_rate: f32) -> f32 {
        64.0 * (sample_rate / FFT_N as f32)
    }

    #[test]
    fn pure_sine_peaks_at_its_bin() {
        let sr = 48_000.0;
        let f0 = bin_centered_f0(sr); // 375.0 Hz at 48k/8192
        let spec = analyze(sr, |b| {
            for (i, s) in b.iter_mut().enumerate() {
                *s = (TAU * f0 * i as f32 / sr).sin();
            }
        });
        assert_eq!(spec.peak_bin(), 64, "peak should be at bin 64 (f0)");
        assert!((spec.bin_of_hz(f0) as i32 - 64).abs() <= 1);
        // The fundamental dominates: peak is far above a mid-spectrum bin.
        assert!(spec.peak_linear() > 100.0 * spec.level_at(f0 * 10.0));
    }
}
```

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --manifest-path crates/deluge-dsp-test/Cargo.toml --target x86_64-unknown-linux-gnu`
Expected: FAIL — `analyze`/`Spectrum` not found.

- [ ] **Step 3: Implement the spectral core**

Prepend to `crates/deluge-dsp-test/src/spectrum.rs` (above the tests, replacing the stub doc line):

```rust
//! Spectral analysis: render → Hann window → real FFT → linear magnitude bins,
//! plus relative metrics (Tasks 2–3). Built on `deluge-fft`.

use deluge_fft::{apply_hann_window_real, Complex, RealFft};

use crate::{FFT_N, LANES};

/// One-sided linear magnitude spectrum of a windowed real signal.
pub struct Spectrum {
    /// Linear magnitudes for bins `0..=FFT_N/2` (length `FFT_N/2 + 1`).
    pub bins: Vec<f32>,
    /// Hz per bin (`sample_rate / FFT_N`).
    pub bin_hz: f32,
    pub sample_rate: f32,
}

/// Render `FFT_N` samples from `renderer`, then analyze.
pub fn analyze(sample_rate: f32, mut renderer: impl FnMut(&mut [f32])) -> Spectrum {
    let mut buf = [0.0f32; FFT_N];
    renderer(&mut buf);
    analyze_buf(sample_rate, &buf)
}

/// Analyze an already-rendered `FFT_N`-length buffer.
pub fn analyze_buf(sample_rate: f32, signal: &[f32; FFT_N]) -> Spectrum {
    let mut windowed = *signal;
    apply_hann_window_real::<FFT_N>(&mut windowed);

    let mut out = [Complex::ZERO; FFT_N / 2 + 1];
    RealFft::<FFT_N, LANES>::process(&windowed, &mut out);

    // Real-FFT gives N/2+1 complex bins; take linear magnitude of each.
    // (deluge-fft's `magnitude_spectrum` is sized for a full [Complex; N], so we
    // compute magnitudes directly from the one-sided output here.)
    let bins: Vec<f32> = out.iter().map(|c| c.abs()).collect();

    Spectrum { bins, bin_hz: sample_rate / FFT_N as f32, sample_rate }
}

impl Spectrum {
    /// Index of the loudest bin.
    pub fn peak_bin(&self) -> usize {
        let mut best = 0;
        let mut best_v = f32::NEG_INFINITY;
        for (i, &v) in self.bins.iter().enumerate() {
            if v > best_v {
                best_v = v;
                best = i;
            }
        }
        best
    }

    /// Linear magnitude of the loudest bin.
    pub fn peak_linear(&self) -> f32 {
        self.bins[self.peak_bin()]
    }

    /// Nearest bin index to `hz` (clamped to the valid range).
    pub fn bin_of_hz(&self, hz: f32) -> usize {
        let b = (hz / self.bin_hz).round() as isize;
        b.clamp(0, self.bins.len() as isize - 1) as usize
    }

    /// Linear magnitude at the bin nearest `hz`.
    pub fn level_at(&self, hz: f32) -> f32 {
        self.bins[self.bin_of_hz(hz)]
    }
}
```

- [ ] **Step 4: Run, verify pass**

Run: `cargo test --manifest-path crates/deluge-dsp-test/Cargo.toml --target x86_64-unknown-linux-gnu`
Expected: PASS (guards + the new spectral test), zero warnings.

If a `feature(portable_simd)` error appears when instantiating `RealFft` (it should not, since this crate names only `RealFft`, not `Simd`), add `#![feature(portable_simd)]` to the top of `lib.rs` — the toolchain is nightly.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-test/src
git commit -m "feat(dsp-test): spectral core — Spectrum, analyze, peak/level"
```

---

## Task 3: Spectral metrics — harmonics, THD, aliasing, noise floor

**Files:**
- Modify: `crates/deluge-dsp-test/src/spectrum.rs`

**Interfaces:**
- Consumes: `Spectrum`, `level_at`, `peak_linear`, `bin_hz` (Task 2).
- Produces (methods on `Spectrum`):
  - `harmonics_db(&self, f0: f32, count: usize) -> Vec<f32>` — dB of each `k·f0` (k=1..=count) relative to the fundamental (so k=1 ≈ 0 dB).
  - `thd(&self, f0: f32, count: usize) -> f32` — `sqrt(Σ_{k=2..=count} h_k²) / h_1` (linear ratio).
  - `worst_alias_db(&self, f0: f32, tol_hz: f32) -> f32` — dB of the loudest **non-harmonic** bin (excluding DC and bins within `tol_hz` of any `k·f0`), relative to the fundamental.
  - `noise_floor_db(&self) -> f32` — dB of the median bin relative to the peak.

- [ ] **Step 1: Write the failing tests (THD + aliasing detection)**

Add these tests inside the existing `mod tests` in `spectrum.rs`:

```rust
    fn render_sines(sr: f32, comps: &[(f32, f32)]) -> Spectrum {
        analyze(sr, |b| {
            for (i, s) in b.iter_mut().enumerate() {
                let t = i as f32 / sr;
                *s = comps.iter().map(|&(f, a)| a * (TAU * f * t).sin()).sum();
            }
        })
    }

    #[test]
    fn thd_matches_a_planted_second_harmonic() {
        let sr = 48_000.0;
        let f0 = bin_centered_f0(sr); // 375 Hz (bin 64)
        // fundamental at 1.0, 2nd harmonic (750 Hz, bin 128) at 0.1.
        let spec = render_sines(sr, &[(f0, 1.0), (2.0 * f0, 0.1)]);
        assert!((spec.thd(f0, 5) - 0.1).abs() < 0.01, "THD ≈ 0.1");
        let h = spec.harmonics_db(f0, 2);
        assert!((h[0] - 0.0).abs() < 0.1, "fundamental ≈ 0 dB");
        assert!((h[1] - (-20.0)).abs() < 1.0, "2nd harmonic ≈ -20 dB");
    }

    #[test]
    fn worst_alias_detects_an_inharmonic_partial() {
        let sr = 48_000.0;
        let f0 = bin_centered_f0(sr); // 375 Hz; harmonics at 375,750,1125,...
        // Plant a -40 dB inharmonic partial at a bin-centered 1234.5-ish freq
        // well away from any k*f0.
        let alias_hz = 210.0 * (sr / FFT_N as f32); // bin 210 ≈ 1230.5 Hz
        let spec = render_sines(sr, &[(f0, 1.0), (alias_hz, 0.01)]);
        let tol = 3.0 * spec.bin_hz;
        let wa = spec.worst_alias_db(f0, tol);
        assert!(wa < -30.0 && wa > -50.0, "worst alias ≈ -40 dB, got {wa}");
    }

    #[test]
    fn pure_sine_has_low_thd_and_alias() {
        let sr = 48_000.0;
        let f0 = bin_centered_f0(sr);
        let spec = render_sines(sr, &[(f0, 1.0)]);
        assert!(spec.thd(f0, 5) < 1e-2, "pure sine THD ~ 0");
        assert!(spec.worst_alias_db(f0, 3.0 * spec.bin_hz) < -60.0);
    }
```

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --manifest-path crates/deluge-dsp-test/Cargo.toml --target x86_64-unknown-linux-gnu`
Expected: FAIL — `thd`/`harmonics_db`/`worst_alias_db` not found.

- [ ] **Step 3: Implement the metrics**

Add to the `impl Spectrum { … }` block in `spectrum.rs`:

```rust
    /// dB of the peak within `±tol` bins of `hz`, relative to `ref_lin`.
    fn peak_near_db(&self, hz: f32, tol_bins: usize, ref_lin: f32) -> f32 {
        let center = self.bin_of_hz(hz) as isize;
        let lo = (center - tol_bins as isize).max(0) as usize;
        let hi = (center + tol_bins as isize).min(self.bins.len() as isize - 1) as usize;
        let peak = self.bins[lo..=hi].iter().cloned().fold(0.0f32, f32::max);
        lin_to_db(peak / ref_lin)
    }

    /// dB of each harmonic `k·f0` (k = 1..=count) relative to the fundamental.
    pub fn harmonics_db(&self, f0: f32, count: usize) -> Vec<f32> {
        let fund = self.level_at(f0).max(1e-20);
        (1..=count)
            .map(|k| self.peak_near_db(k as f32 * f0, 2, fund))
            .collect()
    }

    /// Total harmonic distortion: `sqrt(Σ_{k≥2} h_k²) / h_1` (linear ratio).
    pub fn thd(&self, f0: f32, count: usize) -> f32 {
        let fund = self.level_at(f0).max(1e-20);
        let sum_sq: f32 = (2..=count)
            .map(|k| {
                let h = self.level_at(k as f32 * f0);
                h * h
            })
            .sum();
        sum_sq.sqrt() / fund
    }

    /// dB of the loudest bin that is NOT a harmonic of `f0`, relative to the
    /// fundamental. Excludes DC and bins within `tol_hz` of any `k·f0`.
    pub fn worst_alias_db(&self, f0: f32, tol_hz: f32) -> f32 {
        let fund = self.level_at(f0).max(1e-20);
        let mut worst = 0.0f32;
        for (i, &mag) in self.bins.iter().enumerate() {
            let hz = i as f32 * self.bin_hz;
            if hz <= tol_hz {
                continue; // skip DC / very low bins (window leakage)
            }
            // Is `hz` within tol of some harmonic k*f0 (k ≥ 1)?
            let k = (hz / f0).round().max(1.0);
            let nearest_harmonic = k * f0;
            if (hz - nearest_harmonic).abs() <= tol_hz {
                continue; // legitimate harmonic, not an alias
            }
            worst = worst.max(mag);
        }
        lin_to_db(worst / fund)
    }

    /// dB of the median bin relative to the peak (a spectral noise-floor proxy).
    pub fn noise_floor_db(&self) -> f32 {
        let mut v = self.bins.clone();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = v[v.len() / 2];
        lin_to_db(median / self.peak_linear().max(1e-20))
    }
```

Add this free function near the top of `spectrum.rs` (below the `use`s):

```rust
/// Linear amplitude ratio → decibels (floored so silence doesn't produce -inf).
fn lin_to_db(ratio: f32) -> f32 {
    20.0 * ratio.max(1e-12).log10()
}
```

- [ ] **Step 4: Run, verify pass**

Run: `cargo test --manifest-path crates/deluge-dsp-test/Cargo.toml --target x86_64-unknown-linux-gnu`
Expected: PASS (guards + all spectral tests), zero warnings.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-test/src/spectrum.rs
git commit -m "feat(dsp-test): spectral metrics — harmonics, THD, worst-alias, noise floor"
```

---

## Task 4: CPU-cost harness

**Files:**
- Modify: `crates/deluge-dsp-test/src/cpu.rs`

**Interfaces:**
- Produces:
  - `pub struct CostReport { pub ns_per_block: f64, pub times_realtime: f64 }`
  - `pub fn measure(sample_rate: f32, block_len: usize, iters: usize, render_block: impl FnMut()) -> CostReport`
  - `pub fn compare(a: &CostReport, b: &CostReport) -> f64` — `a.ns_per_block / b.ns_per_block`.

- [ ] **Step 1: Write the failing test (2× work ≈ 2× cost)**

`crates/deluge-dsp-test/src/cpu.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use std::hint::black_box;

    fn work(iters: usize) {
        let mut acc = 0.0f32;
        for i in 0..iters {
            acc += (i as f32).sin();
        }
        black_box(acc);
    }

    #[test]
    fn measure_reports_positive_and_realtime_ratio() {
        let r = measure(48_000.0, 32, 200, || work(1_000));
        assert!(r.ns_per_block > 0.0);
        assert!(r.times_realtime > 0.0);
    }

    #[test]
    fn compare_tracks_relative_workload() {
        let a = measure(48_000.0, 32, 200, || work(2_000));
        let b = measure(48_000.0, 32, 200, || work(1_000));
        let ratio = compare(&a, &b);
        // Host timing is noisy — assert only that 2× work costs roughly 2×.
        assert!(ratio > 1.4 && ratio < 3.0, "ratio ~2, got {ratio}");
    }
}
```

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --manifest-path crates/deluge-dsp-test/Cargo.toml --target x86_64-unknown-linux-gnu`
Expected: FAIL — `measure`/`compare`/`CostReport` not found.

- [ ] **Step 3: Implement the harness**

Prepend to `crates/deluge-dsp-test/src/cpu.rs` (above the tests, replacing the stub doc line):

```rust
//! CPU-cost harness. Host wall-clock is noisy, so this is for *comparison and
//! regression*, never an absolute gate: median-of-N per-block timing, plus a
//! ratio helper for A/B (e.g. band-limited vs naïve). The absolute per-block
//! budget gate lives on-device (deferred — see the QA spec §5).

use std::time::Instant;

/// Result of a timing run.
pub struct CostReport {
    /// Median wall-clock nanoseconds for one `render_block` call.
    pub ns_per_block: f64,
    /// Cost relative to real time: `ns_per_block / (block_len/sample_rate in ns)`.
    /// < 1.0 means faster than real time for that block.
    pub times_realtime: f64,
}

/// Warm up, then time `iters` calls of `render_block` and take the median.
pub fn measure(
    sample_rate: f32,
    block_len: usize,
    iters: usize,
    mut render_block: impl FnMut(),
) -> CostReport {
    assert!(iters > 0, "iters must be > 0");
    // Warmup (fill caches / let the CPU ramp) — not measured.
    for _ in 0..(iters / 10).max(1) {
        render_block();
    }
    let mut samples = Vec::with_capacity(iters);
    for _ in 0..iters {
        let t = Instant::now();
        render_block();
        samples.push(t.elapsed().as_nanos() as f64);
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let ns_per_block = samples[samples.len() / 2];

    let block_ns = (block_len as f64 / sample_rate as f64) * 1e9;
    CostReport {
        ns_per_block,
        times_realtime: ns_per_block / block_ns,
    }
}

/// Ratio of two reports' per-block cost. `> 1.0` means `a` is costlier than `b`.
pub fn compare(a: &CostReport, b: &CostReport) -> f64 {
    a.ns_per_block / b.ns_per_block.max(f64::MIN_POSITIVE)
}
```

- [ ] **Step 4: Run, verify pass**

Run: `cargo test --manifest-path crates/deluge-dsp-test/Cargo.toml --target x86_64-unknown-linux-gnu`
Expected: PASS (all tasks' tests), zero warnings. (If the noisy `compare` ratio test flakes on a loaded machine, it is comparison-only by design; re-run — but it should sit near 2.0.)

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-test/src/cpu.rs
git commit -m "feat(dsp-test): CPU-cost harness — measure + compare (relative)"
```

---

## Self-review notes

- **Spec coverage:** spectral analysis with `worst_alias_db` aliasing metric (Tasks 2–3); THD + harmonics (Task 3); CPU comparison harness, relative-not-absolute (Task 4); guards/null (Task 1); host-only, workspace-excluded, deluge-fft-only dependency, buffer/closure API (Task 1 scaffold + throughout); self-validation against known signals (pure sine, planted harmonic, planted inharmonic partial, subnormal/NaN — Tasks 1–4).
- **Deliberate deviations from the spec:** (1) fixed `FFT_N = 8192` instead of `analyze::<N>` generic, to avoid propagating `deluge-fft`'s `generic_const_exprs` bounds (noted in Global Constraints). (2) `Spectrum.bins` stores **linear** magnitudes (not dB); `*_db` methods convert on read — cleaner for `thd` (linear) and keeps metrics relative.
- **Deferred (per spec §5, not in this plan):** on-device cycle-counter profiler / absolute budget gate; golden-vector regeneration framework; migrating P0's inline guards to `guards`.
- **Known follow-up:** the `compare` ratio test is timing-based; if it proves flaky in CI, widen the tolerance or mark it `#[ignore]` for CI and keep it as a local check — it is a comparison sanity test, not a correctness gate.
