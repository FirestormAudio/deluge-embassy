# QA — DSP measurement & validation harness — design & spec

Sub-project `QA` of the [DSP library cluster](2026-07-06-dsp-library-vision-design.md).
A host-side measurement toolkit that the vocabulary suites (`Osc`/`Fi`/`Mod`/`Ef`)
use to *prove* their DSP: FFT-based spectral analysis (frequency response,
THD, aliasing), a CPU-cost comparison harness, and reusable numeric guards +
null tests. It systematizes the ad-hoc golden/property/null-test patterns P0 and
P1 scattered inline.

> **Status:** design proposal. Depends on P0 (merged) only via `deluge-fft`
> (already in the workspace). Meant to be **stood up now and grown** as the
> suites need more — this is the first cut, focused on what unblocks `Osc`.

**First-cut scope (chosen):** the suite-serving measurement toolkit. **Deferred:**
on-device (Cortex-A9 cycle-counter) profiling and a systematic golden-vector
regeneration *framework* — see §5.

---

## 1. Goals & non-goals

**Goals**

- A **spectral analyzer** built on `deluge-fft` that turns a rendered tone into
  frequency-domain metrics — fundamental, harmonics, THD, and a **worst-case
  aliasing level** — so a suite can assert e.g. "band-limited saw's worst alias
  is below −60 dB."
- A **CPU-cost harness** for host-side *relative* measurement and regression:
  ns/block, × realtime, and A/B ratios (e.g. band-limited vs naïve cost).
- **Numeric guards + null tests** — finite/bounded/denormal checks, `null(a,b)`,
  `rms`, `max_abs_diff` — one reviewed home for the checks P0/P1 copy-pasted.
- **Decoupled**: operate on `&[f32]` buffers and `FnMut(&mut [f32])` renderer
  closures; depend only on `deluge-fft` + `std`, never on `deluge-dsp-kernels`
  or `deluge-audio-graph`, so any suite can use it without a dependency tangle.
- **Self-validating**: the harness's own tests pin its analyzers against known
  signals.

**Non-goals (deferred / out of scope)**

- **On-device profiling** (Cortex-A9 PMU/cycle counter, an *absolute* per-block
  CPU-budget merge gate). Host measurement is *relative* — it catches "got 10×
  slower," not "exceeds N cycles on hardware." The absolute device gate is a
  later addition to this sub-project.
- A golden-vector *regeneration framework*. P0/P1's pinned-golden pattern stays
  as-is (literal expected arrays); QA adds guard helpers, not a codegen system.
- Plotting / visualization. Metrics are numbers asserted in tests.
- Being a `no_std` library. `deluge-dsp-test` is host-only (`std`), dev-dep only.
- Real-time audio measurement on the device itself.

---

## 2. Crate structure

A new crate `crates/deluge-dsp-test` — **host-only, `std`, dev-dependency only**.
It is a workspace member (built/tested on the host triple, like the other
host-tested crates, via `--target x86_64-unknown-linux-gnu`).

```
deluge-fft (exists, no_std) ──► deluge-dsp-test (new, std)
                                  ├─ src/spectrum.rs   render → windowed RealFFT → metrics
                                  ├─ src/cpu.rs        renderer-closure timing harness
                                  ├─ src/guards.rs     finite/denormal/bounds/null helpers
                                  └─ src/lib.rs        re-exports
                                       ▲ (dev-dependency)
   crates/deluge-dsp-kernels/tests, crates/deluge-audio-graph/tests,
   and the future Osc/Fi/Mod/Ef suites' tests
```

`Cargo.toml`: `[dependencies] deluge-fft = { path = "../deluge-fft" }`; no other
workspace crates. Package fields inherited from `[workspace.package]`.

**Interface principle:** the toolkit never names a kernel or node type. A suite
renders its own signal (through its kernel or a built graph) into a buffer or
exposes a `FnMut(&mut [f32])` renderer, then calls the toolkit and asserts on the
returned metrics.

---

## 3. The toolkits

### 3.1 Spectral analysis (`spectrum`) — the `Osc`-critical piece

```rust
/// One-sided magnitude spectrum (dB) of a windowed, real signal.
pub struct Spectrum { /* bins: Vec<f32> (dB), bin_hz: f32, sample_rate: f32 */ }

/// Render N samples from `renderer`, Hann-window, RealFFT, magnitude in dB.
/// N is a power of two (e.g. 8192); larger N = finer bins.
pub fn analyze<const N: usize>(sample_rate: f32, renderer: impl FnMut(&mut [f32])) -> Spectrum;

/// Analyze an already-rendered buffer of length N.
pub fn analyze_buf<const N: usize>(sample_rate: f32, signal: &[f32; N]) -> Spectrum;

impl Spectrum {
    pub fn peak_db(&self) -> f32;                       // loudest bin
    pub fn level_at_db(&self, hz: f32) -> f32;          // nearest-bin level
    pub fn harmonics_db(&self, f0: f32, count: usize) -> Vec<f32>; // levels at k·f0
    pub fn thd(&self, f0: f32, count: usize) -> f32;    // total harmonic distortion
    /// Loudest bin NOT within `tol_hz` of an integer multiple of `f0`, relative
    /// to the fundamental (dB). The aliasing metric: how far down the worst
    /// foldback / inharmonic component sits.
    pub fn worst_alias_db(&self, f0: f32, tol_hz: f32) -> f32;
    pub fn noise_floor_db(&self) -> f32;                // median bin (for noise sources)
}
```

A suite test for a band-limited oscillator reads:
```rust
let spec = spectrum::analyze::<8192>(48_000.0, |b| render_saw(b, 8_000.0));
assert!(spec.worst_alias_db(8_000.0, spec_bin_tol) < -60.0);
```

Built on `deluge_fft::RealFft::<N, LANES>::process` + `deluge_fft::spectrum`'s
`apply_hann_window_real` / `magnitude_spectrum`. Windowing (Hann) is applied
before the FFT so a non-bin-centered `f0` doesn't smear into a false alias floor.

### 3.2 CPU-cost harness (`cpu`) — relative, not an absolute gate

```rust
pub struct CostReport { pub ns_per_block: f64, pub times_realtime: f64, /* … */ }

/// Warm up, then median-of-`iters` wall-clock timing of one `render_block` call.
/// `block_len`/`sample_rate` derive `times_realtime`. Host timing is noisy →
/// report + compare, do not gate on absolute values.
pub fn measure(sample_rate: f32, block_len: usize, iters: usize,
               render_block: impl FnMut()) -> CostReport;

/// A/B cost ratio (e.g. band-limited vs naïve). > 1.0 means `a` is costlier.
pub fn compare(a: &CostReport, b: &CostReport) -> f64;
```

Host wall-clock is noisy, so the harness is for **comparison and regression**:
`compare` for A/B ("band-limited saw is 3.1× naïve"), and regression assertions
use a **generous tolerance** (e.g. "not > 1.5× a recorded baseline"). The
*absolute* per-block CPU-budget merge gate the vision described is device-side
(Cortex-A9 cycles) and stays deferred (§1, §5).

### 3.3 Guards & null tests (`guards`)

```rust
pub fn assert_finite_bounded(buf: &[f32], max_abs: f32);   // no NaN/inf, |x| ≤ max
pub fn assert_no_denormals(buf: &[f32]);                   // no subnormal outputs
pub fn max_abs_diff(a: &[f32], b: &[f32]) -> f32;
pub fn null(a: &[f32], b: &[f32], tol: f32);               // assert max_abs_diff ≤ tol
pub fn rms(buf: &[f32]) -> f32;
```

These replace the copy-pasted `is_finite() && abs()<=1` checks and the
SIMD-vs-scalar `null` comparisons in P0's kernel tests with one reviewed home.
(Migrating existing tests to them is optional cleanup, not required by QA.)

---

## 4. How it's validated (the harness tests itself)

`deluge-dsp-test`'s own tests pin its analyzers against signals with known answers:

- **Pure sine** at a bin-centered `f0` → `peak_db` at `f0`, `harmonics_db` for
  k≥2 near the noise floor, `thd() ≈ 0`, `worst_alias_db` near the floor.
- **Known-aliased signal** (a naïve high-frequency saw, or a synthetic tone with
  a planted inharmonic partial) → `worst_alias_db` detects the partial at the
  expected level.
- **Sine + a low-amplitude harmonic** → `thd()` matches the analytic value within
  tolerance.
- **Guards:** a buffer with a planted NaN trips `assert_finite_bounded`; a
  subnormal trips `assert_no_denormals`; identical buffers give `max_abs_diff` 0.
- **cpu:** `measure` on a fixed-work closure returns a stable-ish median; `compare`
  of a 1× vs a 2× workload returns ≈ 2.0 (with wide tolerance).

Run on the host: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-test`.

---

## 5. Deferred (tracked follow-ups)

- **On-device profiler**: Cortex-A9 cycle counter (PMU/`PMCCNTR`), an absolute
  per-block CPU-budget assertion usable as a merge gate on real hardware. This is
  the "device" half of the vision's "device + sim" profiler.
- **Golden-vector regeneration framework**: a systematic capture/pin/regenerate
  workflow (P0/P1 do it by hand today).
- Optional migration of P0's inline null/bounds checks to `guards`.

---

## 6. Open questions (resolved during implementation)

- FFT size default (`N` = 8192 vs 4096) and the `LANES` const for `RealFft` on
  host; the `tol_hz` convention for harmonic/alias bin matching (derived from
  `bin_hz`).
- Whether `Spectrum` stores bins in a `Vec<f32>` or a fixed `[f32; N/2+1]`
  (host `std`, so `Vec` is fine; pick the simpler ergonomics).
- `cpu::measure` iteration/warmup counts and the regression tolerance the suites
  adopt.
- Whether to migrate P0's existing inline guards now or leave them (§5).
