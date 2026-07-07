# Osc part 3a — named static wavetables Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a single-cycle, band-limited, mip-mapped wavetable oscillator that plays a handful of **built-in named** waveforms (`WT.Saw/Square/Sine/Tri` + 2 richer) pitch-safely, with the mip data pre-baked as `&'static` consts — no Pool, no dynamic tables (those are Plan 3b).

**Architecture:** A new `no_std` kernel `WtOsc` reads its mips through a borrowed `MipSet` (mip-select by pitch → cubic-Hermite interpolation within a level → linear crossfade between the two bracketing levels). A new `no_std` `mipgen` crate builds a band-limited mip pyramid by **additive resynthesis** (forward-FFT the base cycle with `deluge-fft`, then sum only the safe harmonics per octave). A committed `gen-tables` binary emits the `&'static` mip consts. The graph gains `Kind::Wavetable`/`State::Wt` and a `Cmd::BindTable`; Wren gains `Osc.wavetable(WT.x, freq)`.

**Tech Stack:** Rust, `no_std` kernel + `no_std` `deluge-fft` (forward `RealFft<2048,4>`), `deluge-audio-graph`, `deluge-wren-core` (real `wren_sys` VM under `test-support`). Host test target `x86_64-unknown-linux-gnu`; QA via `deluge-dsp-test` (independent realfft) as a dev-dep.

**Reference spec:** [Osc part 3 design](../specs/2026-07-07-osc-wavetable-design.md). This plan is the **static** half; the dynamic (Pool-backed, user-supplied) half is Plan 3b.

## Global Constraints

- **No `MAX_INPUTS` change** — the wavetable node uses ports 0 (freq) and 1 (pmod); port 2 unused (reserved for morph `position` later). `MAX_INPUTS = 3` stays.
- **Transparency:** purely additive. The existing goldens (`golden_saw_lpf_env_first_block` in `deluge-audio-graph`, `golden_saw_lpf_renders_expected_block` in `deluge-wren-core`) and all prior Osc-1/Osc-2 tests must pass **unchanged**. Wavetables are a new `Kind`/`State`/module — nothing existing is edited in a way that changes a default render.
- **`no_std`** in the kernel and in `mipgen`: use `floorf`/`fabsf` etc. (never `f32::fract()`/`f32::abs()` std methods). `deluge-dsp-test` is a dev-dependency only (host tests).
- **Build with one FFT, grade with another:** `mipgen` builds tables via `deluge-fft`'s forward `RealFft` + additive synthesis; every acceptance test measures the result with `deluge-dsp-test`'s independent realfft (`worst_alias_db`, `level_at`, `thd`).
- **Table parameters (defaults; QA-tuned):** base table length `N = 2048`; one mip level per octave, `LEVELS = 11` (level 0 = fullest band ≈ 1024 harmonics, each higher level halves the harmonic count); **cubic Hermite (Catmull-Rom)** interpolation within a level, **linear crossfade** between the two bracketing levels. Thresholds/params are set from measurement, not predicted (the Osc-1 lesson).
- **Test commands:**
  - kernel: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
  - mipgen: `cargo test --target x86_64-unknown-linux-gnu -p mipgen`
  - graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph` **and** `--features simd`
  - wren: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
- Test output pristine (zero warnings). Commit after each task.

---

## Task 1: `mipgen` crate — additive band-limited mip builder

**Files:**
- Create: `crates/mipgen/Cargo.toml`, `crates/mipgen/src/lib.rs`
- Modify: root `Cargo.toml` (add `"crates/mipgen"` to `members`)

**Interfaces:**
- Produces:
  - `pub const N: usize = 2048;` `pub const LEVELS: usize = 11;`
  - `pub struct Harmonics { pub amp: [f32; N/2 + 1], pub phase: [f32; N/2 + 1] }`
  - `pub fn analyze(base: &[f32; N]) -> Harmonics` — forward `RealFft<N,4>`, magnitude+phase per bin.
  - `pub fn max_harmonic(level: usize) -> usize` — the highest harmonic kept at `level` (level 0 → `N/2`, halving per level, floor at 1).
  - `pub fn synth_level(h: &Harmonics, level: usize, out: &mut [f32; N])` — additive resynth of harmonics `1..=max_harmonic(level)` into `out`.
  - `pub fn build_all(base: &[f32; N], out: &mut [[f32; N]; LEVELS])` — fill every level.
- Consumes: `deluge_fft::{RealFft, Complex}`; a sine (use `libm::sinf`).

- [ ] **Step 1: Create the crate skeleton**

`crates/mipgen/Cargo.toml`:

```toml
[package]
name = "mipgen"
version = "0.1.0"
edition = "2021"

[dependencies]
deluge-fft = { path = "../deluge-fft" }
libm = "0.2"

[dev-dependencies]
deluge-dsp-test = { path = "../deluge-dsp-test" }
```

Add `"crates/mipgen",` to the `members` list in the root `Cargo.toml` (next to `"crates/deluge-fft",`).

- [ ] **Step 2: Write the failing tests**

Create `crates/mipgen/src/lib.rs` with only the tests first (module + `#![no_std]` header added in Step 4). For now put this in a `#[cfg(test)] mod tests` you will keep:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    // A naive full-harmonic saw single cycle as the base.
    fn saw_base() -> [f32; N] {
        let mut b = [0.0f32; N];
        for (i, s) in b.iter_mut().enumerate() {
            *s = 2.0 * (i as f32 / N as f32) - 1.0;
        }
        b
    }

    #[test]
    fn max_harmonic_halves_per_level() {
        assert_eq!(max_harmonic(0), N / 2);
        assert_eq!(max_harmonic(1), N / 4);
        assert!(max_harmonic(LEVELS - 1) >= 1);
    }

    #[test]
    fn synth_level_is_band_limited() {
        // Build level 3, FFT it with the INDEPENDENT realfft, assert no
        // energy above its cutoff harmonic.
        let h = analyze(&saw_base());
        let mut lvl = [0.0f32; N];
        synth_level(&h, 3, &mut lvl);

        // Render the single cycle repeated to FFT_N and measure.
        let sr = 48_000.0f32;
        let f0 = sr / N as f32; // one cycle spans N samples → fundamental = sr/N
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        for (i, s) in buf.iter_mut().enumerate() {
            *s = lvl[i % N];
        }
        let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
        let cutoff_hz = max_harmonic(3) as f32 * f0;
        // Energy at 2 harmonics above the cutoff should be ~silent.
        let above = spec.level_at(cutoff_hz + 2.0 * f0);
        let fund = spec.level_at(f0);
        assert!(above < 1e-3 * fund, "band-limited: above={above} fund={fund}");
    }

    #[test]
    fn build_all_is_deterministic() {
        let base = saw_base();
        let mut a = [[0.0f32; N]; LEVELS];
        let mut b = [[0.0f32; N]; LEVELS];
        build_all(&base, &mut a);
        build_all(&base, &mut b);
        assert_eq!(a, b);
    }
}
```

- [ ] **Step 3: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p mipgen`
Expected: FAIL to compile (`analyze`/`synth_level`/`max_harmonic`/`build_all`/`N`/`LEVELS` undefined).

- [ ] **Step 4: Implement `mipgen`**

Prepend to `crates/mipgen/src/lib.rs` (above the test module):

```rust
#![no_std]

use deluge_fft::{Complex, RealFft};
use libm::sinf;

pub const N: usize = 2048;
pub const LEVELS: usize = 11;

const CORE_TWO_PI: f32 = core::f32::consts::TAU;

pub struct Harmonics {
    pub amp: [f32; N / 2 + 1],
    pub phase: [f32; N / 2 + 1],
}

/// Forward real FFT of one single-cycle base → per-harmonic amplitude & phase.
/// Amplitude is normalized so harmonic k reconstructs as `amp[k]*sin(2π k t + phase[k])`.
pub fn analyze(base: &[f32; N]) -> Harmonics {
    let mut spec = [Complex { re: 0.0, im: 0.0 }; N / 2 + 1];
    RealFft::<N, 4>::process(base, &mut spec);
    let mut amp = [0.0f32; N / 2 + 1];
    let mut phase = [0.0f32; N / 2 + 1];
    for k in 0..(N / 2 + 1) {
        let re = spec[k].re;
        let im = spec[k].im;
        // Real-FFT bin magnitude → time-domain sine amplitude: 2/N for 1..N/2-1.
        amp[k] = 2.0 / N as f32 * libm::sqrtf(re * re + im * im);
        // X[k] = sum x[n] e^{-j2πkn/N}; a sine sin(θ+φ) matches phase = atan2(re, -im)... (§note)
        phase[k] = libm::atan2f(re, -im);
    }
    Harmonics { amp, phase }
}

/// Highest harmonic retained at `level` (0 = fullest, halving per octave, floor 1).
pub fn max_harmonic(level: usize) -> usize {
    let full = N / 2;
    let k = full >> level;
    if k < 1 { 1 } else { k }
}

/// Additive resynthesis of harmonics `1..=max_harmonic(level)` into `out`.
pub fn synth_level(h: &Harmonics, level: usize, out: &mut [f32; N]) {
    let kmax = max_harmonic(level);
    for (i, s) in out.iter_mut().enumerate() {
        let t = i as f32 / N as f32; // cycle position [0,1)
        let mut acc = 0.0f32;
        for k in 1..=kmax {
            acc += h.amp[k] * sinf(CORE_TWO_PI * k as f32 * t + h.phase[k]);
        }
        *s = acc;
    }
}

/// Build all `LEVELS` mip levels from one base cycle.
pub fn build_all(base: &[f32; N], out: &mut [[f32; N]; LEVELS]) {
    let h = analyze(base);
    for level in 0..LEVELS {
        synth_level(&h, level, &mut out[level]);
    }
}
```

> **§note on phase convention:** the exact `phase[k]` formula that makes
> `amp[k]*sin(2π k t + phase[k])` reconstruct the analyzed harmonic depends on
> `deluge-fft`'s sign/normalization. **Verify empirically in Step 5:** build
> level 0 and compare against the base cycle's low harmonics via the realfft
> `harmonics_db`; if reconstruction is off, adjust the `phase[k]` expression
> (`atan2f(re,-im)` vs `atan2f(-im,re)` vs adding `π/2`) until level-0 low
> harmonics match the base. This is a measured calibration, not a guess.

- [ ] **Step 5: Run, verify pass (calibrate phase)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p mipgen`
Expected: PASS. If `synth_level_is_band_limited` passes but the reconstructed
shape looks wrong (add a temporary assert comparing `synth_level(0)` low
harmonics to the base's), fix the `phase[k]` convention per the §note. Record the
final phase convention in a code comment. Zero warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/mipgen Cargo.toml
git commit -m "feat(mipgen): additive band-limited mip builder over deluge-fft"
```

---

## Task 2: Kernel `WtOsc` — mip-select + interpolation

**Files:**
- Create: `crates/deluge-dsp-kernels/src/wavetable.rs`
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (add `pub mod wavetable;`)
- Modify: `crates/deluge-dsp-kernels/Cargo.toml` (add `mipgen` as a **dev-dependency** for tests that build tables)

**Interfaces:**
- Produces:
  - `pub struct WtOsc { phase: f32 }`
  - `pub struct MipSet<'a> { pub levels: &'a [&'a [f32]] }` (each level slice length `N`, `levels[0]` fullest band)
  - `pub fn process(&mut self, mips: MipSet, freq: In, pmod: In, dt: f32, out: &mut [f32])`
- Consumes: `In`, `floorf` (already in `lib.rs` / `osc.rs` — reuse the crate's existing `floorf`; if it is private to `osc.rs`, promote it to a shared `pub(crate) fn floorf` in `lib.rs`). Base table length `N = 2048` (match `mipgen::N`; define a local `const N: usize = 2048` and a debug-assert that `levels[0].len() == N`).

- [ ] **Step 1: Write the failing tests**

Create `crates/deluge-dsp-kernels/src/wavetable.rs` with a test module (helpers build real tables via `mipgen`):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::In;

    fn saw_mips() -> [[f32; mipgen::N]; mipgen::LEVELS] {
        let mut base = [0.0f32; mipgen::N];
        for (i, s) in base.iter_mut().enumerate() {
            *s = 2.0 * (i as f32 / mipgen::N as f32) - 1.0;
        }
        let mut out = [[0.0f32; mipgen::N]; mipgen::LEVELS];
        mipgen::build_all(&base, &mut out);
        out
    }

    fn mipset(levels: &[[f32; mipgen::N]; mipgen::LEVELS]) -> [&[f32]; mipgen::LEVELS] {
        core::array::from_fn(|i| &levels[i][..])
    }

    #[test]
    fn wavetable_saw_is_band_limited_high() {
        let sr = 48_000.0f32;
        let m = saw_mips();
        let refs = mipset(&m);
        for &f0 in &[2_000.0f32, 5_000.0] {
            let mut osc = WtOsc::new();
            let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
            osc.process(MipSet { levels: &refs }, In::K(f0), In::K(0.0), 1.0 / sr, &mut buf);
            let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
            let wa = spec.worst_alias_db(f0, 3.0 * spec.bin_hz);
            // Measured floor set in Step 3; start with a loose gate.
            assert!(wa < -20.0, "wt saw f0={f0}: worst_alias {wa} dB");
        }
    }

    #[test]
    fn wavetable_beats_naive_single_table() {
        // A naive single-table (no mip-select) reference at high freq aliases;
        // the mip'd version must be meaningfully better.
        let sr = 48_000.0f32;
        let f0 = 5_000.0f32;
        let m = saw_mips();
        let refs = mipset(&m);

        let mut osc = WtOsc::new();
        let mut bl = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(MipSet { levels: &refs }, In::K(f0), In::K(0.0), 1.0 / sr, &mut bl);
        let bl_wa = deluge_dsp_test::spectrum::analyze_buf(sr, &bl).worst_alias_db(f0, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));

        // Naive: always read level 0 (full band) with linear interp, no mip-select.
        let mut naive = [0.0f32; deluge_dsp_test::FFT_N];
        let mut ph = 0.0f32;
        let dtp = f0 / sr;
        for s in naive.iter_mut() {
            let x = ph * mipgen::N as f32;
            let i0 = x as usize % mipgen::N;
            *s = m[0][i0];
            ph += dtp;
            ph -= floorf(ph);
        }
        let nv_wa = deluge_dsp_test::spectrum::analyze_buf(sr, &naive).worst_alias_db(f0, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));

        assert!(bl_wa < nv_wa - 10.0, "mip'd {bl_wa} should beat naive {nv_wa} by >10 dB");
    }

    #[test]
    fn wavetable_low_freq_has_harmonics() {
        let sr = 48_000.0f32;
        let f0 = 110.0f32;
        let m = saw_mips();
        let refs = mipset(&m);
        let mut osc = WtOsc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(MipSet { levels: &refs }, In::K(f0), In::K(0.0), 1.0 / sr, &mut buf);
        let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
        // Saw has 1st..Nth harmonics; check 2nd & 3rd are present.
        assert!(spec.level_at(2.0 * f0) > 0.05 * spec.level_at(f0));
        assert!(spec.level_at(3.0 * f0) > 0.02 * spec.level_at(f0));
    }

    #[test]
    fn wavetable_mip_boundary_is_continuous() {
        // Sweep across an octave boundary; assert no sample jump (the linear
        // inter-mip crossfade). Compare block rendered at freqs straddling a
        // mip switch — output RMS should vary smoothly, no discontinuity.
        let sr = 48_000.0f32;
        let m = saw_mips();
        let refs = mipset(&m);
        let mut prev_rms = None;
        let mut f = 1_000.0f32;
        while f < 1_100.0 {
            let mut osc = WtOsc::new();
            let mut buf = [0.0f32; 512];
            osc.process(MipSet { levels: &refs }, In::K(f), In::K(0.0), 1.0 / sr, &mut buf);
            let rms = (buf.iter().map(|s| s * s).sum::<f32>() / buf.len() as f32).sqrt();
            if let Some(p) = prev_rms {
                assert!((rms - p).abs() < 0.05, "rms jump at f={f}: {p}->{rms}");
            }
            prev_rms = Some(rms);
            f += 5.0;
        }
    }

    proptest::proptest! {
        #[test]
        fn wavetable_output_bounded(freq in 20.0f32..=8_000.0, pm in -2.0f32..=2.0) {
            let m = saw_mips();
            let refs = mipset(&m);
            let mut osc = WtOsc::new();
            let mut out = [0.0f32; 256];
            osc.process(MipSet { levels: &refs }, In::K(freq), In::K(pm), 1.0 / 48_000.0, &mut out);
            for s in out { proptest::prop_assert!(s.is_finite() && s.abs() <= 1.2); }
        }
    }
}
```

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: FAIL to compile (`WtOsc`/`MipSet`/`process` undefined; `mipgen` dev-dep + `deluge-dsp-test` in scope).

First add to `crates/deluge-dsp-kernels/Cargo.toml` under `[dev-dependencies]`:
```toml
mipgen = { path = "../mipgen" }
```
(`deluge-dsp-test` and `proptest` are already dev-deps from Osc-1/Osc-2.)

- [ ] **Step 3: Implement `WtOsc`**

Prepend to `crates/deluge-dsp-kernels/src/wavetable.rs`:

```rust
//! Single-cycle band-limited wavetable oscillator. Reads a borrowed mip pyramid
//! (`MipSet`), selects the level safe for the current pitch, cubic-Hermite
//! interpolates within it, and linearly crossfades to the adjacent level across
//! octave boundaries. `no_std`, no alloc, no FFT (tables are built offline).

use crate::In;

const N: usize = 2048; // must equal mipgen::N

/// Borrowed view of a mip pyramid: `levels[0]` = fullest band, each higher level
/// halves the harmonic count. Every level slice has length `N`.
pub struct MipSet<'a> {
    pub levels: &'a [&'a [f32]],
}

#[derive(Clone, Copy)]
pub struct WtOsc {
    phase: f32,
}

impl WtOsc {
    pub fn new() -> WtOsc {
        WtOsc { phase: 0.0 }
    }

    pub fn process(&mut self, mips: MipSet, freq: In, pmod: In, dt: f32, out: &mut [f32]) {
        let nlev = mips.levels.len();
        for (i, s) in out.iter_mut().enumerate() {
            let dtp = freq.at(i) * dt; // cycles/sample
            // Mip select: as dtp doubles (one octave up), drop one level of
            // harmonics. level0 is safe while its top harmonic (N/2) stays below
            // Nyquist: dtp*(N/2) < 0.5 → dtp < 1/N. Each octave above adds 1.
            // fractional level → crossfade weight.
            let flevel = if dtp <= 0.0 {
                0.0
            } else {
                // log2(dtp * N) clamped ≥ 0
                let x = dtp * N as f32;
                let l = log2f(x);
                if l < 0.0 { 0.0 } else { l }
            };
            let lo = flevel as usize;
            let lo = if lo >= nlev { nlev - 1 } else { lo };
            let hi = if lo + 1 >= nlev { nlev - 1 } else { lo + 1 };
            let frac = flevel - lo as f32;

            let mut ph = self.phase + pmod.at(i);
            ph -= floorf(ph);

            let a = interp_cubic(mips.levels[lo], ph);
            let b = interp_cubic(mips.levels[hi], ph);
            *s = a + (b - a) * frac.clamp(0.0, 1.0);

            self.phase += dtp;
            self.phase -= floorf(self.phase);
        }
    }
}

impl Default for WtOsc {
    fn default() -> Self {
        WtOsc::new()
    }
}

/// 4-point Catmull-Rom at fractional phase `ph` in [0,1) over a length-N table.
fn interp_cubic(table: &[f32], ph: f32) -> f32 {
    let n = table.len();
    let x = ph * n as f32;
    let i1 = x as usize % n;
    let frac = x - floorf(x);
    let i0 = (i1 + n - 1) % n;
    let i2 = (i1 + 1) % n;
    let i3 = (i1 + 2) % n;
    let (y0, y1, y2, y3) = (table[i0], table[i1], table[i2], table[i3]);
    let a = -0.5 * y0 + 1.5 * y1 - 1.5 * y2 + 0.5 * y3;
    let b = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
    let c = -0.5 * y0 + 0.5 * y2;
    ((a * frac + b) * frac + c) * frac + y1
}

#[inline]
fn floorf(x: f32) -> f32 {
    libm::floorf(x)
}

#[inline]
fn log2f(x: f32) -> f32 {
    libm::log2f(x)
}
```

> If the crate already exposes a shared `floorf` (from `osc.rs`/`lib.rs`), use
> that instead of re-defining it here; do not introduce a second `floorf`. Check
> `lib.rs` first and reuse. `libm` is already a kernel dependency (used by
> `osc.rs`). If `log2f` is unavailable, use `libm::log2f`.

- [ ] **Step 4: Run, verify pass; record the alias floor**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: PASS. Record the measured `worst_alias_db` for saw at 2k/5k. If it
clears the loose `< -20` gate comfortably, tighten the gate to just below the
measured floor and note the measured value in a comment (Osc-1 procedure). If the
mip-boundary continuity test is flaky, that means the crossfade or level formula
is off — fix the kernel, not the test. Zero warnings.

- [ ] **Step 5: Wire the module + commit**

Add `pub mod wavetable;` to `crates/deluge-dsp-kernels/src/lib.rs` (next to `pub mod osc;`).

```bash
git add crates/deluge-dsp-kernels
git commit -m "feat(dsp-kernels): band-limited wavetable oscillator (mip-select + cubic interp)"
```

---

## Task 3: Generated static tables + registry

**Files:**
- Create: `crates/mipgen/src/bin/gen_tables.rs` (the generator binary)
- Create: `crates/deluge-dsp-kernels/src/wavetables_generated.rs` (committed, generated)
- Modify: `crates/deluge-dsp-kernels/src/wavetable.rs` (add the `TableId` + static registry lookup)

**Interfaces:**
- Produces:
  - In `wavetable.rs`: `#[derive(Clone, Copy, PartialEq)] pub struct TableId(pub u16);` and `pub fn static_mipset(id: TableId) -> Option<MipSet<'static>>` looking up `wavetables_generated::TABLES`.
  - In `wavetables_generated.rs`: `pub static SAW_MIPS: [[f32; 2048]; 11] = …;` (+ SQUARE/SINE/TRI + 2 richer), `pub static TABLES: [&[&[f32]]; 6]` (or an accessor returning per-id `&'static [&'static [f32]]`).
- Consumes: `mipgen::{build_all, N, LEVELS}`.

- [ ] **Step 1: Write the generator binary**

`crates/mipgen/src/bin/gen_tables.rs` — defines each named base cycle
(mathematically), builds its mips, and prints Rust source for
`wavetables_generated.rs` to stdout. Bases: saw (`2t-1`), square (`sign(0.5-t)`),
sine (`sin 2πt`), tri (`1-4|t-0.5|`), plus two richer (e.g. `sin+0.5·sin3+0.25·sin5`
"organ", and a formant-ish `sin·(1+cos)`), final palette chosen here.

```rust
// Emits crates/deluge-dsp-kernels/src/wavetables_generated.rs to stdout.
// Run: cargo run -p mipgen --bin gen_tables > crates/deluge-dsp-kernels/src/wavetables_generated.rs
use mipgen::{build_all, LEVELS, N};

fn emit(name: &str, base: &[f32; N]) {
    let mut mips = [[0.0f32; N]; LEVELS];
    build_all(base, &mut mips);
    println!("pub static {name}: [[f32; {N}]; {LEVELS}] = [");
    for lvl in mips.iter() {
        print!("  [");
        for (j, v) in lvl.iter().enumerate() {
            if j % 8 == 0 { print!("\n    "); }
            print!("{:.9}, ", v);
        }
        println!("\n  ],");
    }
    println!("];\n");
}

fn main() {
    println!("// @generated by `cargo run -p mipgen --bin gen_tables`. Do not edit.");
    println!("// Regenerate when mipgen or the base waveforms change.\n");
    let mut saw = [0.0f32; N];
    for (i, s) in saw.iter_mut().enumerate() { *s = 2.0 * (i as f32 / N as f32) - 1.0; }
    emit("SAW_MIPS", &saw);
    // … square, sine, tri, organ, formant (same pattern) …
    // Then the registry:
    println!("pub static SAW: [&[f32]; {LEVELS}] = [{}];",
        (0..LEVELS).map(|i| format!("&SAW_MIPS[{i}]")).collect::<Vec<_>>().join(", "));
    // … one per table …
    println!("pub static TABLES: [&[&[f32]]; 6] = [&SAW, &SQUARE, &SINE, &TRI, &ORGAN, &FORMANT];");
}
```

- [ ] **Step 2: Generate the committed file**

Run: `cargo run -p mipgen --bin gen_tables > crates/deluge-dsp-kernels/src/wavetables_generated.rs`
Then `cargo fmt -p deluge-dsp-kernels` to normalize. Add `#[rustfmt::skip]` or a
`mod` wrapper if the arrays are too large for fmt; the file is `@generated`.

- [ ] **Step 3: Write the failing registry test**

In `wavetable.rs`'s test module add:

```rust
    #[test]
    fn static_saw_table_is_band_limited() {
        let sr = 48_000.0f32;
        let m = static_mipset(TableId(0)).expect("saw table");
        let mut osc = WtOsc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(m, In::K(5_000.0), In::K(0.0), 1.0 / sr, &mut buf);
        let wa = deluge_dsp_test::spectrum::analyze_buf(sr, &buf).worst_alias_db(5_000.0, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));
        assert!(wa < -20.0, "static saw worst_alias {wa} dB");
    }
```

- [ ] **Step 4: Implement the registry + wire the module**

Add to `lib.rs`: `mod wavetables_generated;` (private is fine — accessed via the registry). In `wavetable.rs`:

```rust
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct TableId(pub u16);

/// Resolve a named static table to a borrowed `MipSet`. `None` if out of range.
pub fn static_mipset(id: TableId) -> Option<MipSet<'static>> {
    let idx = id.0 as usize;
    let tables = &crate::wavetables_generated::TABLES;
    if idx >= tables.len() {
        return None;
    }
    Some(MipSet { levels: tables[idx] })
}
```

- [ ] **Step 5: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: PASS (the generated saw clears the alias floor). Zero warnings. If the
generated file is huge and slows compiles unacceptably, note it (a 3b follow-up
could move static tables behind a feature); not a blocker here.

- [ ] **Step 6: Commit**

```bash
git add crates/mipgen/src/bin/gen_tables.rs crates/deluge-dsp-kernels/src/wavetables_generated.rs crates/deluge-dsp-kernels/src/wavetable.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(dsp-kernels): generated static wavetables + id registry"
```

---

## Task 4: Graph — `Kind::Wavetable` + `Cmd::BindTable`

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs`, `src/cmd.rs`, `src/engine.rs`

**Interfaces:**
- Consumes: `deluge_dsp_kernels::wavetable::{WtOsc, MipSet, TableId, static_mipset}`.
- Produces:
  - `Kind::Wavetable` (in `Kind`), `State::Wt(WtOsc)` (in `State`), plus a per-node table binding `TableSrc` stored on the node.
  - `pub enum TableSrc { Static(TableId) }` (Plan 3b adds `Pooled(PoolHandle)`).
  - `Cmd::BindTable { node: NodeId, src: TableSrc }`.
  - `Node::bind_table(&mut self, src: TableSrc)`; the render arm resolves the binding to a `MipSet` and calls `WtOsc::process(mips, ins[0], ins[1], dt, outs.port(0))`.

- [ ] **Step 1: Confirm the shape / write the failing test**

Add to `engine.rs`'s tests:

```rust
    #[test]
    fn wavetable_node_renders_bounded_nonsilent() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Wavetable);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(220.0);
        e.apply(Cmd::BindTable { node: NodeId(0), src: TableSrc::Static(deluge_dsp_kernels::wavetable::TableId(0)) });
        e.render_block();
        let out = e.node_output(NodeId(0), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
        assert!(out.iter().any(|&s| s != 0.0));
    }
```

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: FAIL to compile (`Kind::Wavetable`, `Cmd::BindTable`, `TableSrc` undefined).

- [ ] **Step 3: Add `Kind`/`State`/`TableSrc` + node binding + dispatch**

In `node.rs`:
- Add `Wavetable,` to `enum Kind`.
- Add `Wt(WtOsc),` to `enum State` (import `use deluge_dsp_kernels::wavetable::{WtOsc, TableId, static_mipset};` — the kernel exports `WtOsc`/`MipSet`/`TableId`/`static_mipset`; `TableSrc` is a **graph-local** type defined here, not in the kernel). Define in `node.rs`:

```rust
#[derive(Clone, Copy)]
pub enum TableSrc {
    Static(deluge_dsp_kernels::wavetable::TableId),
}
```

- In `Node::new`'s match: `Kind::Wavetable => State::Wt(WtOsc::new()),`.
- Add a `table: Option<TableSrc>` field to `Node` (default `None`), and:

```rust
    pub fn bind_table(&mut self, src: TableSrc) {
        self.table = Some(src);
    }
```

- In `process_resolved`, add an arm:

```rust
        Kind::Wavetable => {
            if let (State::Wt(o), Some(TableSrc::Static(id))) = (&mut self.state, self.table) {
                if let Some(mips) = static_mipset(id) {
                    o.process(mips, ins[0], ins[1], dt, outs.port(0));
                }
            }
        }
```

(`out_width(Kind::Wavetable)` falls through to the default `1` — no change needed.)

In `cmd.rs`: add `BindTable { node: NodeId, src: crate::node::TableSrc },` to `enum Cmd`. (Confirm `Cmd` derives — match the existing derives, e.g. `Clone, Copy, PartialEq`; `TableSrc` must derive the same. Add `#[derive(Clone, Copy, PartialEq)]` to `TableSrc`, and `TableId` already derives `PartialEq, Eq` from Task 3.)

In `engine.rs`'s `apply`: add

```rust
        Cmd::BindTable { node, src } => {
            if let Some(n) = self.arena.node_mut(node) {
                n.bind_table(src);
            }
        }
```

- [ ] **Step 4: Run, verify pass (both modes) incl. transparency golden**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features simd`
Expected: PASS both — the new test plus `golden_saw_lpf_env_first_block`
unchanged. Zero warnings.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src
git commit -m "feat(audio-graph): Kind::Wavetable + Cmd::BindTable (static tables)"
```

---

## Task 5: Wren — `Osc.wavetable(WT.x, freq)`

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (a `bind_table` emitter), `src/bindings_audio.rs` (a `node_wavetable_impl` factory), `src/bindings.rs` (METHODS table entry), `wren/prelude.wren` (`Osc.wavetable` + `WT` holder), `tests/audio_bindings.rs`

**Interfaces:**
- Produces: `audio::new_wavetable(id, table_id, freq)` (emits `Cmd::NewNode { kind: Kind::Wavetable, args:[freq,Const0,Const0] }` then `Cmd::BindTable { node, src: TableSrc::Static(TableId(table_id)) }`); a `Node.wavetable_(_,_)` static factory; Wren `Osc.wavetable` sugar + a `WT` holder mapping names → ids.

- [ ] **Step 1: Write the failing Cmd-sequence test**

Add to `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn osc_wavetable_emits_newnode_and_bindtable() {
    let cmds = run_and_capture_cmds("var s = Osc.wavetable(WT.Saw, 220)");
    assert!(cmds.iter().any(|c| matches!(c,
        Cmd::NewNode { node: NodeId(0), kind: Kind::Wavetable, .. })));
    assert!(cmds.iter().any(|c| matches!(c,
        Cmd::BindTable { node: NodeId(0), src: TableSrc::Static(id) } if id.0 == 0)));
    // WT.Saw == id 0 — Saw is index 0 in the generated `TABLES` (see note)
}
```

> **Registry-order note:** the generated `TABLES` order (Task 3) fixes the ids.
> Set `WT.Saw`'s id in the test to whatever index Saw occupies in `TABLES` (the
> generator emits `[&SAW, &SQUARE, &SINE, &TRI, …]` → Saw = 0). Adjust the test's
> expected `id.0` to match the committed registry; do not guess — read
> `wavetables_generated.rs`'s `TABLES` line.

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: FAIL — `Osc.wavetable`/`WT` unknown; `TableSrc`/`Cmd::BindTable` maybe not imported in the test.

- [ ] **Step 3: Add the emitter + factory**

In `audio.rs` next to `new_node`:

```rust
pub fn new_wavetable(id: u16, table_id: u16, freq: Input) {
    if id == NULL_ID { return; }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind: Kind::Wavetable,
        args: [freq, Input::Const(0.0), Input::Const(0.0)],
    });
    host().audio_cmd(Cmd::BindTable {
        node: NodeId(id),
        src: deluge_audio_graph::node::TableSrc::Static(
            deluge_dsp_kernels::wavetable::TableId(table_id),
        ),
    });
}
```

(Confirm the exact `Cmd::NewNode` field names/`args` arity against `cmd.rs` —
Task 4/recon show `NewNode { node, kind, args }` with `args: [Input; MAX_ARGS]`.)

In `bindings_audio.rs`:

```rust
pub(crate) fn node_wavetable_impl<S: SlotApi>(vm: &S) {
    let table_id = vm.get_f(1) as u16;
    let freq = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_wavetable(id, table_id, freq);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_wavetable(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_wavetable_impl(&vm);
}
```

Register in `register_audio`: `method("main", "Node", true, "wavetable_(_,_)", node_wavetable_impl::<S>);` and add the matching `METHODS` entry in `bindings.rs` (mirror `src_`).

- [ ] **Step 4: Add the prelude sugar + `WT` holder**

In `prelude.wren`, add the static factory to the `Node` foreign class:
`foreign static wavetable_(table, freq)`. Add to the `Osc` class:
`static wavetable(t, f) { Node.wavetable_(t, f) }`. Add a `WT` holder whose members
return the registry ids (order per Task 3's `TABLES`):

```wren
class WT {
  static Saw { 0 }
  static Square { 1 }
  static Sine { 2 }
  static Tri { 3 }
  static Organ { 4 }
  static Formant { 5 }
}
```

- [ ] **Step 5: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: PASS — the new Cmd-sequence test plus all existing tests (goldens
unchanged). If the test needs `TableSrc`/`Kind`/`Cmd::BindTable` in scope, import
them at the top of `audio_bindings.rs` (they re-export from `deluge_audio_graph`).
Zero warnings.

- [ ] **Step 6: Add an end-to-end render smoke test + commit**

Add one `run_and_render`-style test (mirroring the existing golden test's harness)
that builds `Osc.wavetable(WT.Saw, 220)`, routes it to the root bus, renders a
block, and asserts finite + non-silent output (not a pinned golden — wavetable
output is new; a bounded/non-silent check suffices). Then:

```bash
git add crates/deluge-wren-core
git commit -m "feat(wren-core): Osc.wavetable(WT.x, freq) static wavetables"
```

---

## Self-review notes

- **Spec coverage (static half):** mip-mapped band-limited playback (Task 2), builder correctness graded by independent FFT (Task 1), generated named tables (Task 3), graph `Kind`/`Cmd` (Task 4), Wren surface `Osc.wavetable`/`WT` (Task 5), pitch-safety + improvement-over-naive + mip continuity + low-freq fidelity + bounds (Task 2 QA), transparency goldens unchanged (Tasks 4/5). Deferred to **3b**: Pool integration, user-supplied `Wavetable.from`, `SlotApi` list-read, exhaustion/lifecycle.
- **Measured, not predicted:** the Task-2 `< -20 dB` gate and the `>10 dB` beats-naive margin are set from the first measurement, then tightened just below the floor (Osc-1 procedure). The phase convention in `mipgen::analyze` is empirically calibrated in Task 1 Step 5, not assumed.
- **Transparency:** every change is additive — a new `Kind` arm, a new `State` variant, a new `Cmd`, new modules. No existing render path is altered, so both goldens hold (verified Tasks 4 default+simd, 5).
- **`MAX_INPUTS` untouched** — ports 0/1 only; port 2 reserved for morph.
- **Risk/known-nit:** the generated `wavetables_generated.rs` (6 tables × 11 levels × 2048 f32 ≈ 135k floats) is a large source file; if compile time suffers, a 3b follow-up can gate it behind a feature or load from a build artifact. Flagged, not blocking.
- **Cross-task type consistency:** `TableId(u16)` (kernel) ↔ `TableSrc::Static(TableId)` (graph) ↔ `new_wavetable(table_id: u16)` (wren) — the id is a plain `u16` end to end; the `WT` holder's member ids must equal the committed `TABLES` order (Task 3), which Tasks 4/5 tests read rather than guess.
