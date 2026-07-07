# Osc 3c — inverse real FFT + IFFT-based mipgen Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a dedicated half-size inverse real FFT to `deluge-fft`, then switch `mipgen`'s runtime mip build to forward-FFT → zero-high-bins → inverse-FFT (~16× cheaper than additive), keeping additive as the equivalence oracle.

**Architecture:** `RealFft::process_inverse` reverses the merged forward `RealFft::process` step-for-step: inverse-split (recover the half-size complex spectrum `Z` from the `N/2+1` bins) → inner N/2 inverse complex FFT via `conj(fft(conj(·)))` (reusing the existing forward radix-4, plus a trivial `Complex::conj`) → unpack + `1/hn` scale. `mipgen` gains an IFFT build path; the additive path is renamed `build_all_additive` and kept as the oracle. The 3b runtime `build_pyramid_into` switches to a new flat-region `mipgen::build_pyramid_flat` (IFFT); `gen_tables` stays additive so the committed static tables aren't regenerated.

**Tech Stack:** Rust, `no_std` `deluge-fft` (portable-SIMD, const-generic, compile-time twiddles), `mipgen`, `deluge-wren-core`. Host test target `x86_64-unknown-linux-gnu`. `rustfft` (already a `deluge-fft` test-dep) + `deluge-dsp-test` realfft as independent oracles.

**Reference spec:** [Osc 3c design](../specs/2026-07-07-osc-wavetable-3c-irfft-design.md).

## Global Constraints

- **`no_std`** in `deluge-fft` and `mipgen` (`libm`/core only; the inverse uses the existing forward radix-4 + `TwiddleTable`, no new float-transcendental code).
- **Forward is unscaled**; the inverse owns the normalization, which is **`1/hn` where `hn = N/2`** (derived + verified; pinned by the round-trip test — do not guess a different constant).
- **Additive stays the oracle.** Rename `build_all` → `build_all_additive`; the IFFT path must equal it within tolerance (a test). The IFFT path must **zero DC (bin 0) as well as bins above the cutoff**, because additive `synth_level` sums `k=1..=kmax` (no DC) — matching this is required for the equivalence test to pass.
- **Transparency:** the committed `crates/deluge-dsp-kernels/src/wavetables_generated.rs` is **NOT** regenerated (`gen_tables` stays additive). All merged 3a/3b/Osc-1/Osc-2 tests + both goldens pass unchanged. The 3b runtime round-trip still passes on the IFFT path within the existing alias-floor margins.
- **Twiddle convention (verbatim from the forward code):** `TwiddleTable::<N>::re(tw_off+k) = cos(2πk/N)`, `im(...) = −sin(2πk/N)` (= `W_N^k`), with `tw_off = N/2 − 1`. The inverse uses `conj(W) = (wr, −wi)`.
- **Test commands:** `deluge-fft`: `cargo test --target x86_64-unknown-linux-gnu -p deluge-fft`. `mipgen`: `-p mipgen`. `wren-core`: `-p deluge-wren-core --features test-support`. `dsp-kernels` (static tables unchanged): `-p deluge-dsp-kernels`.
- Zero warnings. Commit after each task.

---

## Task 1: `deluge-fft` — `Complex::conj` + `RealFft::process_inverse`

**Files:**
- Modify: `crates/deluge-fft/src/complex.rs` (add `conj`), `crates/deluge-fft/src/real_fft.rs` (add `process_inverse`).
- Test: `crates/deluge-fft/src/tests.rs` and/or `crates/deluge-fft/tests/rustfft_oracle.rs`.

**Interfaces:**
- Produces: `Complex::conj(self) -> Complex`; `RealFft::<N,LANES>::process_inverse(bins: &[Complex; N/2 + 1], out: &mut [f32; N])`.
- Consumes: `TwiddleTable::<N>::{re,im}` (crate-internal), `FftBuf::<{N/2}>`, `crate::radix4::process_r4_simd_soa::<{N/2}, LANES>`.

- [ ] **Step 1: Add `Complex::conj` + its test**

In `complex.rs`, inside `impl Complex`:

```rust
    #[inline(always)]
    pub fn conj(self) -> Self {
        Self { re: self.re, im: -self.im }
    }
```

Add a unit test (in the crate's test module): `assert_eq!(Complex::new(1.0,2.0).conj(), Complex::new(1.0,-2.0));`.

- [ ] **Step 2: Write the failing round-trip test**

In `crates/deluge-fft/tests/rustfft_oracle.rs` (mirrors the existing `real_oracle!` idiom — deterministic xorshift input, per-size, `sqrt(N)`-scaled tolerance), add a round-trip test:

```rust
// Forward then inverse reconstructs the original real input.
macro_rules! real_roundtrip {
    ($name:ident, $n:literal) => {
        #[test]
        fn $name() {
            const N: usize = $n;
            let mut st: u64 = 0x1234_5678_9abc_def0;
            let mut next = || { st ^= st << 13; st ^= st >> 7; st ^= st << 17; (st >> 40) as f32 / (1u32 << 24) as f32 - 0.5 };
            let mut x = [0.0f32; N];
            for v in x.iter_mut() { *v = next(); }
            let mut bins = [deluge_fft::Complex::ZERO; N / 2 + 1];
            deluge_fft::RealFft::<N, 4>::process(&x, &mut bins);
            let mut y = [0.0f32; N];
            deluge_fft::RealFft::<N, 4>::process_inverse(&bins, &mut y);
            let tol = 1e-4 * (N as f32).sqrt();
            for i in 0..N {
                assert!((x[i] - y[i]).abs() < tol, "N={} i={} x={} y={}", N, i, x[i], y[i]);
            }
        }
    };
}
real_roundtrip!(real_roundtrip_16, 16);
real_roundtrip!(real_roundtrip_64, 64);
real_roundtrip!(real_roundtrip_256, 256);
real_roundtrip!(real_roundtrip_512, 512);
real_roundtrip!(real_roundtrip_2048, 2048);
```

- [ ] **Step 3: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-fft`
Expected: FAIL to compile — `process_inverse` undefined.

- [ ] **Step 4: Implement `process_inverse`**

In `real_fft.rs`, inside the `impl<const N, const LANES> RealFft<N, LANES>` block (same `where` bounds as `process`):

```rust
    /// Inverse of [`process`]: `N/2+1` one-sided complex bins → `N` real samples.
    /// Reverses the forward's split → half-size FFT → pack, with a single `1/hn`
    /// (`hn = N/2`) normalization. Uses `conj(fft(conj(·)))` for the inner
    /// inverse (reusing the forward radix-4), so no new butterfly code.
    pub fn process_inverse(bins: &[crate::complex::Complex; N / 2 + 1], out: &mut [f32; N]) {
        assert!(
            N >= 8 && N.is_power_of_two(),
            "RealFft N must be a power of two >= 8"
        );
        let hn = N / 2;
        let tw_off = hn - 1;

        // 1. Inverse-split: recover Z[0..hn], stored as conj(Z) into zbuf (SoA)
        //    so the inner forward FFT computes conj(fft(conj(Z))) = hn * ifft(Z).
        let mut zbuf = FftBuf::<{ N / 2 }>::ZERO;

        // Z[0]: X[0] = Z0.re + Z0.im, X[hn] = Z0.re - Z0.im (both real).
        let x0 = bins[0].re;
        let xh = bins[hn].re;
        let z0r = (x0 + xh) * 0.5;
        let z0i = (x0 - xh) * 0.5;
        zbuf.re[0] = z0r;   // conj(Z[0]).re = Z0.re
        zbuf.im[0] = -z0i;  // conj(Z[0]).im = -Z0.im

        for k in 1..hn {
            let nk = hn - k;
            let p = bins[k];
            let q = bins[nk].conj(); // conj(X[nk])
            // a = (P + Q)/2
            let ar = (p.re + q.re) * 0.5;
            let ai = (p.im + q.im) * 0.5;
            // s = (P - Q)/2 ; Wd = j*s = (-s.im, s.re)
            let sr = (p.re - q.re) * 0.5;
            let si = (p.im - q.im) * 0.5;
            let wdr = -si;
            let wdi = sr;
            // d = conj(W) * Wd,  conj(W) = (wr, -wi)
            let wr = TwiddleTable::<N>::re(tw_off + k);
            let wi = TwiddleTable::<N>::im(tw_off + k);
            let dr = wr * wdr + wi * wdi; // wr*wdr - (-wi)*wdi
            let di = wr * wdi - wi * wdr; // wr*wdi + (-wi)*wdr
            // Z[k] = a + d ; store conj(Z[k])
            zbuf.re[k] = ar + dr;
            zbuf.im[k] = -(ai + di);
        }

        // 2. Inner forward FFT of conj(Z).
        crate::radix4::process_r4_simd_soa::<{ N / 2 }, LANES>(&mut zbuf);

        // 3. z = conj(result) / hn ; unpack z[k] = x[2k] + j*x[2k+1].
        let inv = 1.0 / hn as f32;
        for k in 0..hn {
            out[2 * k] = zbuf.re[k] * inv;       // conj → re unchanged
            out[2 * k + 1] = -zbuf.im[k] * inv;  // conj → im negated
        }
    }
```

- [ ] **Step 5: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-fft`
Expected: PASS — round-trip at all sizes + existing forward tests + the `conj` test. If a size fails, the likely culprits are the inverse-split constants or the `1/hn` scale — debug against the all-ones vector (`x=[1;N] → bins=[N,0,…,0] → out=[1;N]`) and the impulse (`x=[1,0,…] → bins=[1,1,…,1] → out=[1,0,…]`); do NOT loosen the tolerance to pass. Zero warnings.

- [ ] **Step 6: Add impulse/DC/Nyquist edge tests + `rustfft` inverse cross-check**

Add (in `tests.rs` or the oracle file): impulse round-trips; a DC-only spectrum (`bins[0]=N`, rest 0) → constant `1.0`; a Nyquist-only spectrum (`bins[hn]=±1`, rest 0) → alternating. If `rustfft` exposes an inverse real FFT (`RealToComplex`/`ComplexToReal` via `realfft`, or `plan_fft_inverse` on the full complex buffer built from the one-sided bins), add a cross-check vs it at N=512 with `1e-2·sqrt(N)` tolerance. (If wiring rustfft's inverse is awkward, the round-trip + edge tests are the required gate; note the omission.)

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-fft/src/complex.rs crates/deluge-fft/src/real_fft.rs crates/deluge-fft/src/tests.rs crates/deluge-fft/tests/rustfft_oracle.rs
git commit -m "feat(deluge-fft): dedicated inverse real FFT (RealFft::process_inverse) + Complex::conj"
```

---

## Task 2: `mipgen` — IFFT build path (additive kept as oracle)

**Files:**
- Modify: `crates/mipgen/src/lib.rs` (rename additive; add IFFT path), `crates/mipgen/src/bin/gen_tables.rs` (call the renamed additive).

**Interfaces:**
- Produces: `build_all_additive(base, out)` (the current `build_all`, renamed); `build_level_ifft(spectrum: &[Complex; N/2+1], level, out: &mut [f32; N])`; `build_all_ifft(base, out: &mut [[f32; N]; LEVELS])`; `build_pyramid_flat(base: &[f32], region: &mut [f32])` (flat `N*LEVELS` region, for the 3b runtime path); `build_all` = `build_all_ifft` (the default). `pub use deluge_fft::Complex;` (so consumers can name the spectrum type if needed — though `build_pyramid_flat` hides it).
- Consumes: `deluge_fft::{RealFft, Complex}`.

- [ ] **Step 1: Rename additive + point `gen_tables` at it**

In `lib.rs`, rename `pub fn build_all` → `pub fn build_all_additive` (body unchanged; keeps calling `analyze`/`synth_level`). In `gen_tables.rs`, change the `build_all(base, &mut mips)` call to `build_all_additive(base, &mut mips)`. Also update the existing 3a test `build_all_is_deterministic` (which calls `build_all`) to call `build_all_additive`, so additive determinism stays covered — the new `build_all` (IFFT, added in Step 4) gets its own coverage via the equivalence + band-limit tests.

Verify build-time generator still produces the SAME bytes: `cargo run -p mipgen --bin gen_tables --target x86_64-unknown-linux-gnu > /tmp/regen.rs` then `diff /tmp/regen.rs crates/deluge-dsp-kernels/src/wavetables_generated.rs` — must be identical (additive path is byte-unchanged). Do NOT commit any regenerated file.

- [ ] **Step 2: Write the failing IFFT-equivalence + band-limit tests**

In `lib.rs`'s `mod tests`:

```rust
    #[test]
    fn ifft_matches_additive() {
        // Use an arbitrary-phase base (saw's all-0/π phases would hide sign bugs).
        let mut base = [0.0f32; N];
        for (i, s) in base.iter_mut().enumerate() {
            let t = i as f32 / N as f32;
            *s = libm::sinf(core::f32::consts::TAU * t + 0.7)
               + 0.5 * libm::sinf(core::f32::consts::TAU * 3.0 * t + 2.3)
               + 0.25 * libm::sinf(core::f32::consts::TAU * 5.0 * t - 1.1);
        }
        let mut add = [[0.0f32; N]; LEVELS];
        let mut ift = [[0.0f32; N]; LEVELS];
        build_all_additive(&base, &mut add);
        build_all_ifft(&base, &mut ift);
        for level in 0..LEVELS {
            for i in 0..N {
                assert!((add[level][i] - ift[level][i]).abs() < 1e-3,
                    "level {level} i {i}: additive {} vs ifft {}", add[level][i], ift[level][i]);
            }
        }
    }

    #[test]
    fn ifft_level_is_band_limited() {
        let mut base = [0.0f32; N];
        for (i, s) in base.iter_mut().enumerate() { *s = 2.0 * (i as f32 / N as f32) - 1.0; }
        let mut ift = [[0.0f32; N]; LEVELS];
        build_all_ifft(&base, &mut ift);
        let sr = 48_000.0f32;
        let f0 = sr / N as f32;
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        for (i, s) in buf.iter_mut().enumerate() { *s = ift[3][i % N]; }
        let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
        let above = spec.level_at(max_harmonic(3) as f32 * f0 + 2.0 * f0);
        let fund = spec.level_at(f0);
        assert!(above < 1e-3 * fund, "ifft band-limited: above={above} fund={fund}");
    }

    #[test]
    fn build_pyramid_flat_matches_build_all_ifft() {
        let mut base = [0.0f32; N];
        for (i, s) in base.iter_mut().enumerate() { *s = 2.0 * (i as f32 / N as f32) - 1.0; }
        let mut nested = [[0.0f32; N]; LEVELS];
        build_all_ifft(&base, &mut nested);
        let mut flat = [0.0f32; N * LEVELS];
        build_pyramid_flat(&base, &mut flat);
        for level in 0..LEVELS {
            for i in 0..N {
                assert_eq!(flat[level * N + i], nested[level][i]);
            }
        }
    }
```

- [ ] **Step 3: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p mipgen`
Expected: FAIL to compile (`build_all_ifft`/`build_level_ifft`/`build_pyramid_flat` undefined).

- [ ] **Step 4: Implement the IFFT path**

Add to `lib.rs`:

```rust
pub use deluge_fft::Complex;

/// Band-limit one level from a full forward spectrum: keep harmonics `1..=kmax`
/// (drop DC and everything above the cutoff — matching additive's `k=1..=kmax`),
/// then inverse-FFT.
pub fn build_level_ifft(spectrum: &[Complex; N / 2 + 1], level: usize, out: &mut [f32; N]) {
    let kmax = max_harmonic(level);
    let mut s = *spectrum;
    s[0] = Complex::ZERO; // drop DC to match additive (which sums k>=1)
    for (k, bin) in s.iter_mut().enumerate() {
        if k > kmax {
            *bin = Complex::ZERO;
        }
    }
    deluge_fft::RealFft::<N, 4>::process_inverse(&s, out);
}

/// Build all levels from one base via forward FFT + per-level band-limit + inverse.
pub fn build_all_ifft(base: &[f32; N], out: &mut [[f32; N]; LEVELS]) {
    let mut spectrum = [Complex::ZERO; N / 2 + 1];
    deluge_fft::RealFft::<N, 4>::process(base, &mut spectrum);
    for (level, out_level) in out.iter_mut().enumerate() {
        build_level_ifft(&spectrum, level, out_level);
    }
}

/// Build a full pyramid into a flat `N*LEVELS` region (for the 3b runtime path).
/// `base` is padded/truncated to `N`.
pub fn build_pyramid_flat(base: &[f32], region: &mut [f32]) {
    if region.len() < N * LEVELS {
        return;
    }
    let mut b = [0.0f32; N];
    let n = N.min(base.len());
    b[..n].copy_from_slice(&base[..n]);
    let mut spectrum = [Complex::ZERO; N / 2 + 1];
    deluge_fft::RealFft::<N, 4>::process(&b, &mut spectrum);
    let mut lvl = [0.0f32; N];
    for level in 0..LEVELS {
        build_level_ifft(&spectrum, level, &mut lvl);
        region[level * N..(level + 1) * N].copy_from_slice(&lvl);
    }
}

/// Default build path is now the fast IFFT one.
pub fn build_all(base: &[f32; N], out: &mut [[f32; N]; LEVELS]) {
    build_all_ifft(base, out);
}
```

- [ ] **Step 5: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p mipgen`
Expected: PASS — `ifft_matches_additive` (the key equivalence oracle), `ifft_level_is_band_limited`, `build_pyramid_flat_matches_build_all_ifft`, and the existing additive/analyze/determinism tests. If `ifft_matches_additive` fails at Nyquist for level 0, check the DC-zeroing and that additive's Nyquist amplitude (the 3b `1/N` fix) agrees with the raw-bin inverse within `1e-3` (loosen ONLY that one comparison's tolerance to a documented, still-tight value if it's a genuine f32-rounding gap at a single bin — not a blanket loosening). Zero warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/mipgen/src/lib.rs crates/mipgen/src/bin/gen_tables.rs
git commit -m "feat(mipgen): IFFT build path (build_all/build_pyramid_flat); additive kept as oracle"
```

---

## Task 3: Switch the 3b runtime path to IFFT

**Files:**
- Modify: `crates/deluge-wren-core/src/host.rs` (`build_pyramid_into` → delegate to `mipgen::build_pyramid_flat`).

**Interfaces:**
- Consumes: `mipgen::build_pyramid_flat`.
- Produces: no API change — `build_pyramid_into(base, region)` keeps its signature; only its body changes.

- [ ] **Step 1: Confirm current behavior / write the regression test**

The 3b end-to-end round-trip (`crates/deluge-wren-core/tests/audio_bindings.rs`, the `Wavetable.from(...)` → `Osc.wavetable` → `run_and_render` test) already exercises `build_pyramid_into`. Confirm it currently passes (it does, on additive). No new test needed here — the switch must keep it green. Optionally add an assertion that the rendered output is unchanged within tolerance vs the additive build (not required; the existing finite/non-silent/bounded assertion is the gate).

- [ ] **Step 2: Switch the body**

In `crates/deluge-wren-core/src/host.rs`, `build_pyramid_into` currently does `analyze` + per-level `synth_level` into `region`. Replace its body with a single delegation:

```rust
pub fn build_pyramid_into(base: &[f32], region: &mut [f32]) {
    // IFFT path (Osc 3c) — ~16× cheaper than the previous additive build.
    mipgen::build_pyramid_flat(base, region);
}
```

(`mipgen` is already a dependency of `deluge-wren-core` from 3b.)

- [ ] **Step 3: Run, verify pass (incl. 3b round-trip + static tables unchanged)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: PASS — the 3b dynamic-wavetable round-trip still finite/non-silent/bounded, both goldens unchanged.
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: PASS — the static-table alias tests unchanged (the committed `wavetables_generated.rs` was never touched; verify `git status` shows it unmodified).
Zero warnings.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-wren-core/src/host.rs
git commit -m "perf(wren-core): dynamic wavetable upload builds via IFFT mipgen (~16x)"
```

---

## Self-review notes

- **Spec coverage:** inverse real FFT + `conj` (Task 1); `mipgen` IFFT path + additive-as-oracle + gen_tables stays additive (Task 2); runtime `build_pyramid_into` → IFFT (Task 3). QA: round-trip + edge + rustfft (Task 1), IFFT≡additive equivalence + band-limit + determinism (Task 2), 3b regression + static-tables-unchanged (Task 3).
- **Derived, not guessed:** the inverse-split algebra (`a=(X[k]+conj(X[nk]))/2`, `d=conj(W)·j·(X[k]−conj(X[nk]))/2`, `Z[k]=a+d`; DC/Nyquist → `Z[0]`) and the `1/hn` normalization are worked out against the forward code and verified on all-ones/impulse; the round-trip test pins them. The plan gives the full `process_inverse` body.
- **The DC-match subtlety:** `build_level_ifft` zeros bin 0 to match additive's `k=1..=kmax` (no DC) — without this, `ifft_matches_additive` fails on any base with DC. Called out in Task 2.
- **Transparency:** `gen_tables` stays additive → `wavetables_generated.rs` byte-unchanged (verified in Task 2 Step 1 and Task 3 Step 3); all goldens + 3a/3b tests unchanged.
- **No new float-transcendental / no new butterfly:** the inverse reuses `process_r4_simd_soa` + `TwiddleTable` via `conj(fft(conj))`; only `Complex::conj` and O(N) split/unpack passes are new.
- **Known follow-ups:** device-upload real-time path (now with the ~16× cheaper build); multi-frame morph; a dedicated inverse *complex* butterfly (negligible gain today).
