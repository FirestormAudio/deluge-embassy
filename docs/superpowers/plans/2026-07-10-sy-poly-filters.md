# Sy-2c: Poly Filters (Moog/Ms20) + Footgun Close — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `Moog.lp/lp2` and `Ms20.lp/hp` work inside a Wren `Synth` builder (polyphonic, f32x8 across voices), and guarantee no free-standing class ever silently builds a mono node on a poly edge.

**Architecture:** Three layers mirroring the existing PolySvf wiring. Kernel: two vectorized saturators + `PolyMoog<POLES>`/`PolyMs20` structs (dual scalar-oracle/f32x8 path). Graph: `Kind`/`State`/`poly_process` arms. Wren: poly factories + class-route flips + `Fiber.abort` guards on every still-mono class.

**Tech Stack:** Rust `no_std`, `core::simd` f32x8 (behind `simd` feature), Wren scripting via `deluge-wren-core`.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP paths. ARM Cortex-A9 (VFPv3+NEON) target; host x86 for tests only.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **`VOICES == 8`**, the crate const. f32x8 SIMD is across voices, SoA per-voice state. Guard any f32x8 item with `#[cfg(feature = "simd")] const _: () = assert!(VOICES == 8);` (one such guard already exists in poly.rs:185 — do not add a second).
- **SIMD convention:** the scalar path is the correctness oracle. The `#[cfg(feature = "simd")]` f32x8 fast path is null-tested lane-for-lane against scalar to `≤ 1e-4`. Doc comments attach to the struct, never to a cfg'd const guard.
- **Poly control convention:** the leading poly edge is the audio input (`poly_in_count == 1`); cutoff/res are trailing *shared mono* controls, computed once per sample and splatted to all lanes.
- **Test invocation (per-crate, never `--workspace`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <filter>` and again with `--features simd`.
  - Graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph <filter>` and again with `--features deluge-dsp-kernels/simd`.
  - Wren: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core <name>` and again with `--features deluge-dsp-kernels/simd`.

## File Structure

- `crates/deluge-dsp-kernels/src/filter.rs` — add `pade_tanh_x8`, `ms20_clip_x8`, `moog_coeffs`, `ms20_coeffs`, `Ms20::tick`; refactor mono `Moog::process`/`Ms20::process` to call the shared helpers (behavior-identical).
- `crates/deluge-dsp-kernels/src/poly.rs` — add `PolyMoog<POLES>`, `PolyMs20`, and a `#[cfg(feature="simd")]` `heun_step_x8` helper; null-tests.
- `crates/deluge-dsp-kernels/src/lib.rs` — re-export `PolyMoog`, `PolyMs20` if the crate re-exports poly types (match existing PolySvf export).
- `crates/deluge-audio-graph/src/node.rs` — `Kind::{PolyMoogLp4,PolyMoogLp2,PolyMs20Lp,PolyMs20Hp}`, `State::{PolyMoog4,PolyMoog2,PolyMs20}`, constructor + predicate + `poly_process` arms; graph render tests.
- `crates/deluge-wren-core/src/bindings_audio.rs` — `node_polymoog_impl`, `node_polyms20_impl`.
- `crates/deluge-wren-core/src/bindings.rs` — register `polymoog_`/`polyms20_` in METHODS + `register_audio`.
- `crates/deluge-wren-core/wren/prelude.wren` — flip `Moog`/`Ms20` classes; guard `Svf`/`Tb303`/`Resonator` + 9 FX classes.
- `crates/deluge-wren-core/tests/audio_bindings.rs` — end-to-end + abort tests.

---

## Task 1: Vectorized saturators (`pade_tanh_x8`, `ms20_clip_x8`)

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/filter.rs` (add beside scalar `pade_tanh` @:171 and `ms20_clip` @:457)
- Test: same file, `#[cfg(test)]` module (existing `filter` tests live there)

**Interfaces:**
- Consumes: scalar `pade_tanh` (:171), `ms20_clip` (:457).
- Produces: `#[cfg(feature = "simd")] pub(crate) fn pade_tanh_x8(x: f32x8) -> f32x8`; `#[cfg(feature = "simd")] pub(crate) fn ms20_clip_x8(x: f32x8) -> f32x8`. Used by Tasks 2 and 3.

- [ ] **Step 1: Write the failing null-test**

Add to filter.rs test module:

```rust
#[cfg(feature = "simd")]
#[test]
fn pade_tanh_x8_matches_scalar() {
    use core::simd::prelude::*;
    // Sweep including saturating tails (scalar clamps to ±1 at |x|≥3).
    let xs = [-8.0f32, -3.5, -3.0, -1.7, -0.4, 0.0, 0.25, 1.0, 2.9, 3.0, 5.0, 8.0];
    for &x in &xs {
        let v = pade_tanh_x8(f32x8::splat(x)).to_array();
        for lane in v {
            assert!((lane - pade_tanh(x)).abs() <= 1e-4, "x={x} got {lane} want {}", pade_tanh(x));
        }
    }
}

#[cfg(feature = "simd")]
#[test]
fn ms20_clip_x8_matches_scalar() {
    use core::simd::prelude::*;
    let xs = [-8.0f32, -3.0, -1.0, -0.3, 0.0, 0.3, 1.0, 3.0, 8.0];
    for &x in &xs {
        let v = ms20_clip_x8(f32x8::splat(x)).to_array();
        for lane in v {
            assert!((lane - ms20_clip(x)).abs() <= 1e-4, "x={x} got {lane} want {}", ms20_clip(x));
        }
    }
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd pade_tanh_x8_matches_scalar ms20_clip_x8_matches_scalar`
Expected: FAIL — `pade_tanh_x8` / `ms20_clip_x8` not found.

- [ ] **Step 3: Implement the saturators**

Add beside the scalar versions in filter.rs (the `use` of f32x8 is local to keep non-simd builds clean):

```rust
/// Lane-parallel `pade_tanh` (Task-1 null-tested against the scalar oracle).
#[cfg(feature = "simd")]
#[inline]
pub(crate) fn pade_tanh_x8(x: core::simd::f32x8) -> core::simd::f32x8 {
    use core::simd::prelude::*;
    let x2 = x * x;
    let s = x * (f32x8::splat(27.0) + x2) / (f32x8::splat(27.0) + f32x8::splat(9.0) * x2);
    s.simd_max(f32x8::splat(-1.0)).simd_min(f32x8::splat(1.0))
}

/// Lane-parallel `ms20_clip` (biased `pade_tanh`, de-biased). Matches scalar exactly.
#[cfg(feature = "simd")]
#[inline]
pub(crate) fn ms20_clip_x8(x: core::simd::f32x8) -> core::simd::f32x8 {
    use core::simd::prelude::*;
    const B: f32 = 0.5;
    pade_tanh_x8(x + f32x8::splat(B)) - pade_tanh_x8(f32x8::splat(B))
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd pade_tanh_x8_matches_scalar ms20_clip_x8_matches_scalar`
Expected: PASS (2 tests).

Also confirm the non-simd build is unaffected: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels filter` — Expected: PASS, no warnings about the new items (they are `#[cfg(feature="simd")]`).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/filter.rs
git commit -m "feat(kernels): f32x8 pade_tanh_x8 + ms20_clip_x8 saturators (null-tested)"
```

---

## Task 2: `PolyMoog<POLES>` kernel

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/filter.rs` (extract `moog_coeffs`, refactor `Moog::process`)
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (add `heun_step_x8`, `PolyMoog<POLES>`)
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (re-export `PolyMoog` if PolySvf is re-exported there)
- Test: `poly.rs` test module

**Interfaces:**
- Consumes: `pade_tanh_x8` (Task 1), `DiodeLadder<POLES>` (filter.rs:182) and its `process` (:226), `moog_k` (:381), `MOOG_OVERSAMPLE` (:371), `Moog<POLES>` (:393), the `In` type (used throughout poly.rs).
- Produces: `pub(crate) fn moog_coeffs(cutoff: f32, res: f32, dt: f32, poles: usize) -> (f32 /*fh*/, f32 /*ladder_res*/, f32 /*k*/)`; `pub struct PolyMoog<const POLES: usize>` with `pub fn new() -> Self`, `pub fn set_drive(&mut self, d: f32)`, `pub fn process(&mut self, audio: &[f32], cutoff: In, res: In, dt: f32, out: &mut [f32])`. Used by Task 4.

- [ ] **Step 1: Extract the shared coeff helper (pure refactor)**

In filter.rs, add above `Moog`:

```rust
/// Moog per-sample coefficients (drive-independent), shared by the mono
/// `Moog::process` and the poly `PolyMoog` so both agree bit-for-bit.
/// Returns (`fh` = prewarped cutoff, `ladder_res` = internal resonance,
/// `k` = loop gain `ladder_res·4` for the input compensation `1+k`).
#[inline]
pub(crate) fn moog_coeffs(cutoff: f32, res: f32, dt: f32, poles: usize) -> (f32, f32, f32) {
    let two_pi_dt_os = 2.0 * core::f32::consts::PI * dt / MOOG_OVERSAMPLE as f32;
    let cutoff = cutoff.clamp(20.0, 18_000.0);
    let res = res.clamp(0.0, 1.0);
    let fh = two_pi_dt_os * cutoff;
    let ladder_res = res * moog_k(poles);
    (fh, ladder_res, ladder_res * 4.0)
}
```

Refactor `Moog::<POLES>::process` (filter.rs:408) body to use it — behavior identical:

```rust
    pub fn process(&mut self, input: In, cutoff: In, res: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            let (fh, ladder_res, k) = moog_coeffs(cutoff.at(i), res.at(i), dt, POLES);
            let x = input.at(i) * self.drive * (1.0 + k);
            *s = self.ladder.process(x, fh, ladder_res, MOOG_OVERSAMPLE);
        }
    }
```

- [ ] **Step 2: Verify the refactor is behavior-preserving**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels moog`
Then: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd moog`
Expected: existing `moog_tests` PASS unchanged (pure refactor, no new behavior).

- [ ] **Step 3: Write the failing PolyMoog null-test**

In poly.rs test module:

```rust
#[test]
fn polymoog_matches_scalar_oracle_both_slopes() {
    use deluge_dsp_kernels::filter::Moog; // adjust path if Moog is re-exported at crate root
    let dt = 1.0 / 48_000.0;
    let n = 200usize;
    // Shared controls; per-voice distinct audio so lanes are genuinely independent.
    let cutoff = 1200.0f32;
    let res = 0.8f32;
    check_slope::<4>(dt, n, cutoff, res);
    check_slope::<2>(dt, n, cutoff, res);

    fn check_slope<const P: usize>(dt: f32, n: usize, cutoff: f32, res: f32) {
        let mut poly = PolyMoog::<P>::new();
        let mut refs: [Moog<P>; VOICES] = core::array::from_fn(|_| Moog::<P>::new());
        // Voice v gets a saw-ish ramp scaled per voice.
        let audio: alloc::vec::Vec<f32> = (0..n * VOICES)
            .map(|j| { let i = j / VOICES; let v = j % VOICES; ((i as f32 * 0.017 + v as f32 * 0.03) % 1.0) * 2.0 - 1.0 })
            .collect();
        let mut out = alloc::vec![0.0f32; n * VOICES];
        poly.process(&audio, In::Const(cutoff), In::Const(res), dt, &mut out);
        for v in 0..VOICES {
            let vin: alloc::vec::Vec<f32> = (0..n).map(|i| audio[i * VOICES + v]).collect();
            let mut vout = alloc::vec![0.0f32; n];
            refs[v].process(In::Buf(&vin), In::Const(cutoff), In::Const(res), dt, &mut vout);
            for i in 0..n {
                assert!((out[i * VOICES + v] - vout[i]).abs() <= 1e-4,
                    "slope {P} voice {v} sample {i}: {} vs {}", out[i * VOICES + v], vout[i]);
            }
        }
    }
}
```

> Note: match the test module's existing conventions for `In::Buf`/`In::Const`, `alloc::vec`, and the `Moog` import path used by the current poly.rs tests (e.g. PolySvf's test imports `Svf`). If poly.rs tests already `use` these, drop the redundant `use`.

- [ ] **Step 4: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels polymoog_matches_scalar_oracle_both_slopes`
Expected: FAIL — `PolyMoog` not found.

- [ ] **Step 5: Implement the f32x8 Heun ladder helper**

In poly.rs, add (SIMD only):

```rust
/// One Heun (RK2) diode-ladder sub-step across `VOICES` lanes. Mirrors the
/// scalar `DiodeLadder::heun_step` (filter.rs) exactly; `state` is SoA per-stage.
/// `fh`/`res` are shared (splatted); `input` is per-voice.
#[cfg(feature = "simd")]
#[inline]
fn heun_step_x8<const STAGES: usize>(
    state: &mut [core::simd::f32x8; STAGES],
    input: core::simd::f32x8,
    fh: core::simd::f32x8,
    res4: core::simd::f32x8, // res * 4.0, pre-multiplied
) {
    use core::simd::prelude::*;
    use crate::filter::pade_tanh_x8;
    let feedback = pade_tanh_x8(state[STAGES - 1]) * res4;
    let x = input - feedback;

    // Predictor (Euler)
    let mut temp = *state;
    let mut d = [f32x8::splat(0.0); STAGES];
    d[0] = fh * (x - pade_tanh_x8(temp[0]));
    for i in 1..STAGES {
        d[i] = fh * (pade_tanh_x8(temp[i - 1]) - pade_tanh_x8(temp[i]));
    }
    for i in 0..STAGES {
        temp[i] += d[i];
    }

    // Corrector (derivative at predicted state)
    let fb_p = pade_tanh_x8(temp[STAGES - 1]) * res4;
    let x_p = input - fb_p;
    let mut dp = [f32x8::splat(0.0); STAGES];
    dp[0] = fh * (x_p - pade_tanh_x8(temp[0]));
    for i in 1..STAGES {
        dp[i] = fh * (pade_tanh_x8(temp[i - 1]) - pade_tanh_x8(temp[i]));
    }
    for i in 0..STAGES {
        state[i] += (d[i] + dp[i]) * f32x8::splat(0.5);
    }
}
```

> The scalar `DiodeLadder::heun_step` multiplies `res * 4.0` internally; here we pass `res4 = ladder_res * 4.0` pre-multiplied. `moog_coeffs` returns `k = ladder_res * 4.0`, which is exactly this value — pass `k` as `res4`.

- [ ] **Step 6: Implement `PolyMoog<POLES>`**

```rust
#[cfg(feature = "simd")]
const _: () = assert!(VOICES == 8); // already present at :185 — reuse, do not duplicate

/// Poly Moog transistor-ladder. Poly audio in → 8 filtered lanes; shared mono
/// cutoff/res. Scalar path holds `[DiodeLadder<POLES>; VOICES]` and reuses the
/// audited `DiodeLadder::process`; the SIMD path runs the Heun ladder across
/// `f32x8` lanes via `heun_step_x8`. Both use `moog_coeffs`, so they agree.
#[derive(Clone, Copy)]
pub struct PolyMoog<const POLES: usize> {
    drive: f32,
    #[cfg(not(feature = "simd"))]
    ladders: [DiodeLadder<POLES>; VOICES],
    #[cfg(feature = "simd")]
    state: [core::simd::f32x8; POLES],
}

impl<const POLES: usize> PolyMoog<POLES> {
    #[cfg(not(feature = "simd"))]
    pub fn new() -> Self {
        PolyMoog { drive: 1.0, ladders: [DiodeLadder::new(); VOICES] }
    }
    #[cfg(feature = "simd")]
    pub fn new() -> Self {
        PolyMoog { drive: 1.0, state: [core::simd::f32x8::splat(0.0); POLES] }
    }

    pub fn set_drive(&mut self, d: f32) {
        self.drive = d.max(0.0);
    }

    /// `audio` = voice-interleaved poly input; cutoff/res shared mono; LP tile out.
    pub fn process(&mut self, audio: &[f32], cutoff: In, res: In, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            use crate::filter::pade_tanh_x8;
            let drive = f32x8::splat(self.drive);
            for i in 0..n {
                // res4 = ladder_res*4 = k, so moog_coeffs' k is exactly heun_step_x8's res4.
                let (fh, _ladder_res, k) = moog_coeffs(cutoff.at(i), res.at(i), dt, POLES);
                let (fhv, res4) = (f32x8::splat(fh), f32x8::splat(k));
                let gain = drive * f32x8::splat(1.0 + k);
                let x = f32x8::from_slice(&audio[i * VOICES..]) * gain;
                for _ in 0..MOOG_OVERSAMPLE {
                    heun_step_x8::<POLES>(&mut self.state, x, fhv, res4);
                }
                let y = pade_tanh_x8(self.state[POLES - 1]); // clipped last stage
                y.copy_to_slice(&mut out[i * VOICES..]);
            }
        }
        #[cfg(not(feature = "simd"))]
        {
            for i in 0..n {
                let (fh, ladder_res, k) = moog_coeffs(cutoff.at(i), res.at(i), dt, POLES);
                let gain = self.drive * (1.0 + k);
                for v in 0..VOICES {
                    let x = audio[i * VOICES + v] * gain;
                    out[i * VOICES + v] =
                        self.ladders[v].process(x, fh, ladder_res, MOOG_OVERSAMPLE);
                }
            }
        }
    }
}

impl<const POLES: usize> Default for PolyMoog<POLES> {
    fn default() -> Self { Self::new() }
}
```

> Imports: add `use crate::filter::{DiodeLadder, moog_coeffs, MOOG_OVERSAMPLE};` (and whatever the poly.rs import block already pulls from `filter`). `DiodeLadder`/`moog_coeffs`/`MOOG_OVERSAMPLE` are `pub(crate)` — reachable from poly.rs (same crate). If `DiodeLadder::process`/`new` are not `pub(crate)`-visible to poly.rs, widen them to `pub(crate)` in the same commit (they are already `pub`).

- [ ] **Step 7: Run to verify it passes (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels polymoog`
Then: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd polymoog`
Expected: PASS both. (Scalar config exercises the `[DiodeLadder; VOICES]` oracle path against `Moog`; simd config exercises `heun_step_x8` against `Moog`.)

- [ ] **Step 8: Add a boundedness sanity test**

```rust
#[test]
fn polymoog_stays_bounded_at_high_res() {
    let dt = 1.0 / 48_000.0;
    let n = 4096;
    let mut poly = PolyMoog::<4>::new();
    let audio = alloc::vec![0.5f32; n * VOICES]; // constant excitation
    let mut out = alloc::vec![0.0f32; n * VOICES];
    poly.process(&audio, In::Const(1000.0), In::Const(1.0), dt, &mut out);
    for &s in &out { assert!(s.abs() <= 1.0001, "moog diverged: {s}"); }
}
```

Run both configs (`polymoog_stays_bounded`); Expected: PASS (the ladder output is `pade_tanh`-clipped to ±1 by construction).

- [ ] **Step 9: Commit**

```bash
git add crates/deluge-dsp-kernels/src/filter.rs crates/deluge-dsp-kernels/src/poly.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(kernels): PolyMoog<POLES> (scalar oracle + f32x8 Heun ladder), null-tested"
```

---

## Task 3: `PolyMs20` kernel

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/filter.rs` (extract `ms20_coeffs` + `Ms20::tick`, refactor `Ms20::process`)
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (add `PolyMs20`)
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (re-export `PolyMs20` alongside `PolyMoog`)
- Test: `poly.rs` test module

**Interfaces:**
- Consumes: `pade_tanh_x8`/`ms20_clip_x8` (Task 1), `svf_tan_prewarp` (filter.rs:148), `ms20_clip`/`pade_tanh` (scalar), `Ms20Resp` (:466), `MS20_OVERSAMPLE` (:432), `MS20_DC_HP_HZ` (:450), `OnePoleHp` (:243), `Ms20` (:475), the `In` type.
- Produces: `pub(crate) fn ms20_coeffs(cutoff: f32, res: f32, dt: f32) -> (f32 /*k*/, f32 /*a1*/, f32 /*a2*/, f32 /*a3*/)`; a per-sample `pub(crate) fn Ms20::tick(&mut self, input: f32, k: f32, a1: f32, a2: f32, a3: f32, resp: Ms20Resp) -> f32`; `pub struct PolyMs20 { .. }` with `new`, `set_drive`, `process(&mut self, audio: &[f32], cutoff: In, res: In, resp: Ms20Resp, dt: f32, out: &mut [f32])`. Used by Task 4.

- [ ] **Step 1: Extract `ms20_coeffs` + `Ms20::tick` (pure refactor)**

In filter.rs, add the coeff helper above `Ms20`:

```rust
/// MS-20 per-sample coefficients, shared by the mono `Ms20::process` and the
/// poly `PolyMs20`. `k` is the resonance damping (`2(1−res)`, floored); a1/a2/a3
/// are the TPT integrator coefficients at the oversampled step.
#[inline]
pub(crate) fn ms20_coeffs(cutoff: f32, res: f32, dt: f32) -> (f32, f32, f32, f32) {
    let fc = cutoff.clamp(20.0, 18_000.0);
    let res = res.clamp(0.0, 1.0);
    let k = (2.0 * (1.0 - res)).max(1e-4);
    let theta = (core::f32::consts::PI * fc * dt / MS20_OVERSAMPLE as f32)
        .min(0.49 * core::f32::consts::PI);
    let g = svf_tan_prewarp(theta);
    let a1 = 1.0 / (1.0 + g * (g + k));
    let a2 = g * a1;
    let a3 = g * a2;
    (k, a1, a2, a3)
}
```

Add the per-sample tick as an `Ms20` method (the loop body of the current `process`, verbatim — DC blocker `self.dc` and `self.drive` are instance state):

```rust
    /// One output sample. Coeffs precomputed by `ms20_coeffs`; `self.dc`/`self.drive`
    /// are the DC-blocker state and drive gain. Shared by mono `process` and `PolyMs20`.
    #[inline]
    pub(crate) fn tick(&mut self, input: f32, k: f32, a1: f32, a2: f32, a3: f32, resp: Ms20Resp) -> f32 {
        let mut y = 0.0;
        for _ in 0..MS20_OVERSAMPLE {
            let v0 = input * self.drive - k * ms20_clip(self.drive * self.ic1eq);
            let v3 = v0 - self.ic2eq;
            let v1 = a1 * self.ic1eq + a2 * v3;
            let v2 = self.ic2eq + a2 * self.ic1eq + a3 * v3;
            self.ic1eq = 2.0 * v1 - self.ic1eq;
            self.ic2eq = 2.0 * v2 - self.ic2eq;
            y = match resp {
                Ms20Resp::Lp => v2,
                Ms20Resp::Hp => v0 - k * v1 - v2,
            };
        }
        8.0 * pade_tanh(self.dc.process(y) / 8.0)
    }
```

Refactor `Ms20::process` (filter.rs:492) to compute the DC coeff on `dt` change (unchanged) then loop calling `ms20_coeffs` + `tick`:

```rust
    pub fn process(&mut self, input: In, cutoff: In, res: In, resp: Ms20Resp, dt: f32, out: &mut [f32]) {
        if dt != self.cached_dt {
            self.dc.set_coeff(MS20_DC_HP_HZ * 2.0 * core::f64::consts::PI / (1.0 / dt as f64));
            self.cached_dt = dt;
        }
        for (i, s) in out.iter_mut().enumerate() {
            let (k, a1, a2, a3) = ms20_coeffs(cutoff.at(i), res.at(i), dt);
            *s = self.tick(input.at(i), k, a1, a2, a3, resp);
        }
    }
```

> `OnePoleHp::process` (:256) and its `x_prev`/`y_prev`/`a` fields must be reachable from `PolyMs20` in poly.rs. `OnePoleHp` is `pub(crate)` (:243) with private fields; for the SIMD DC-blocker in Step 5 you need the coeff `a` and to run the recurrence on `f32x8` state. Add a `pub(crate) fn coeff(&self) -> f32 { self.a }` accessor to `OnePoleHp` in this step so poly.rs can splat it (do NOT make the fields public).

- [ ] **Step 2: Verify the refactor is behavior-preserving**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels ms20`
Then with `--features simd`.
Expected: existing `ms20_tests` PASS unchanged.

- [ ] **Step 3: Write the failing PolyMs20 null-test**

```rust
#[test]
fn polyms20_matches_scalar_oracle_both_responses() {
    use deluge_dsp_kernels::filter::{Ms20, Ms20Resp}; // adjust to the poly.rs import convention
    let dt = 1.0 / 48_000.0;
    let n = 200usize;
    for resp in [Ms20Resp::Lp, Ms20Resp::Hp] {
        let mut poly = PolyMs20::new();
        let mut refs: [Ms20; VOICES] = core::array::from_fn(|_| Ms20::new());
        let audio: alloc::vec::Vec<f32> = (0..n * VOICES)
            .map(|j| { let i = j / VOICES; let v = j % VOICES; ((i as f32 * 0.021 + v as f32 * 0.04) % 1.0) * 2.0 - 1.0 })
            .collect();
        let mut out = alloc::vec![0.0f32; n * VOICES];
        poly.process(&audio, In::Const(1500.0), In::Const(0.8), resp, dt, &mut out);
        for v in 0..VOICES {
            let vin: alloc::vec::Vec<f32> = (0..n).map(|i| audio[i * VOICES + v]).collect();
            let mut vout = alloc::vec![0.0f32; n];
            refs[v].process(In::Buf(&vin), In::Const(1500.0), In::Const(0.8), resp, dt, &mut vout);
            for i in 0..n {
                assert!((out[i * VOICES + v] - vout[i]).abs() <= 1e-4,
                    "{resp:?} voice {v} sample {i}: {} vs {}", out[i * VOICES + v], vout[i]);
            }
        }
    }
}
```

- [ ] **Step 4: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels polyms20_matches_scalar`
Expected: FAIL — `PolyMs20` not found.

- [ ] **Step 5: Implement `PolyMs20`**

```rust
/// Poly MS-20 (Korg35) Sallen-Key. Poly audio in → 8 filtered lanes; shared
/// mono cutoff/res. Scalar path holds `[Ms20; VOICES]` and reuses `Ms20::tick`;
/// the SIMD path runs the ZDF two-integrator + `ms20_clip_x8` feedback + DC
/// blocker + `pade_tanh_x8` limiter across `f32x8`. Both use `ms20_coeffs`.
#[derive(Clone, Copy)]
pub struct PolyMs20 {
    drive: f32,
    cached_dt: f32,
    #[cfg(not(feature = "simd"))]
    voices: [Ms20; VOICES],
    #[cfg(feature = "simd")]
    ic1: core::simd::f32x8,
    #[cfg(feature = "simd")]
    ic2: core::simd::f32x8,
    #[cfg(feature = "simd")]
    dc_x: core::simd::f32x8,
    #[cfg(feature = "simd")]
    dc_y: core::simd::f32x8,
    #[cfg(feature = "simd")]
    dc_a: f32, // DC-blocker coeff (shared; depends only on dt)
}

impl PolyMs20 {
    #[cfg(not(feature = "simd"))]
    pub fn new() -> Self {
        PolyMs20 { drive: 1.0, cached_dt: 0.0, voices: [Ms20::new(); VOICES] }
    }
    #[cfg(feature = "simd")]
    pub fn new() -> Self {
        use core::simd::f32x8;
        PolyMs20 { drive: 1.0, cached_dt: 0.0,
            ic1: f32x8::splat(0.0), ic2: f32x8::splat(0.0),
            dc_x: f32x8::splat(0.0), dc_y: f32x8::splat(0.0), dc_a: 0.0 }
    }

    pub fn set_drive(&mut self, d: f32) {
        let d = d.max(0.0);
        self.drive = d;
        #[cfg(not(feature = "simd"))]
        for v in &mut self.voices { v.set_drive(d); }
    }

    pub fn process(&mut self, audio: &[f32], cutoff: In, res: In, resp: Ms20Resp, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            use crate::filter::{ms20_clip_x8, pade_tanh_x8};
            if dt != self.cached_dt {
                // Reuse the scalar OnePoleHp coeff computation for exact agreement.
                let mut hp = crate::filter::OnePoleHp::default();
                hp.set_coeff(crate::filter::MS20_DC_HP_HZ * 2.0 * core::f64::consts::PI / (1.0 / dt as f64));
                self.dc_a = hp.coeff();
                self.cached_dt = dt;
            }
            let drive = f32x8::splat(self.drive);
            let a = f32x8::splat(self.dc_a);
            let b = (f32x8::splat(1.0) + a) * f32x8::splat(0.5);
            let two = f32x8::splat(2.0);
            let eight = f32x8::splat(8.0);
            for i in 0..n {
                let (k, a1, a2, a3) = ms20_coeffs(cutoff.at(i), res.at(i), dt);
                let (kv, a1v, a2v, a3v) = (f32x8::splat(k), f32x8::splat(a1), f32x8::splat(a2), f32x8::splat(a3));
                let input = f32x8::from_slice(&audio[i * VOICES..]);
                let mut y = f32x8::splat(0.0);
                for _ in 0..MS20_OVERSAMPLE {
                    let v0 = input * drive - kv * ms20_clip_x8(drive * self.ic1);
                    let v3 = v0 - self.ic2;
                    let v1 = a1v * self.ic1 + a2v * v3;
                    let v2 = self.ic2 + a2v * self.ic1 + a3v * v3;
                    self.ic1 = two * v1 - self.ic1;
                    self.ic2 = two * v2 - self.ic2;
                    y = match resp {
                        Ms20Resp::Lp => v2,
                        Ms20Resp::Hp => v0 - kv * v1 - v2,
                    };
                }
                // DC blocker (OnePoleHp recurrence) then ±8 tanh limiter.
                let dc = b * y - b * self.dc_x + a * self.dc_y;
                self.dc_x = y;
                self.dc_y = dc;
                let s = eight * pade_tanh_x8(dc / eight);
                s.copy_to_slice(&mut out[i * VOICES..]);
            }
        }
        #[cfg(not(feature = "simd"))]
        {
            for i in 0..n {
                let (k, a1, a2, a3) = ms20_coeffs(cutoff.at(i), res.at(i), dt);
                for v in 0..VOICES {
                    // Ms20::tick recomputes DC coeff on dt change via process? No —
                    // tick assumes dc coeff set. Mirror mono process: set on dt change.
                    out[i * VOICES + v] = self.voices[v].tick_with_dt(audio[i * VOICES + v], k, a1, a2, a3, resp, dt);
                }
            }
        }
    }
}

impl Default for PolyMs20 {
    fn default() -> Self { Self::new() }
}
```

> **DC-coeff timing (scalar path):** `Ms20::tick` (Task 3 Step 1) assumes `self.dc` is already coeff-set (mono `process` sets it on `dt` change before the loop). For the scalar poly path each `Ms20` voice needs the same. Rather than expose the private `dc`, add a thin `pub(crate) fn tick_with_dt(&mut self, input: f32, k: f32, a1: f32, a2: f32, a3: f32, resp: Ms20Resp, dt: f32) -> f32` to `Ms20` that does the `if dt != self.cached_dt { self.dc.set_coeff(..); self.cached_dt = dt; }` guard then calls `self.tick(...)`. Have mono `Ms20::process` call `tick_with_dt` too (fold the dt-guard into it) so there is exactly one dt-guard implementation and the scalar poly path is bit-identical to the oracle. Update Step 1's `process` accordingly (it becomes a loop over `tick_with_dt`).
>
> **SIMD DC coeff:** computed once via a throwaway scalar `OnePoleHp` (same `set_coeff` math) → `hp.coeff()`, guaranteeing the SIMD DC blocker uses the identical `a`/`b` as the scalar oracle. Requires `MS20_DC_HP_HZ`, `OnePoleHp`, and `OnePoleHp::coeff()` to be `pub(crate)` (coeff accessor added in Step 1).

- [ ] **Step 6: Run to verify it passes (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels polyms20`
Then with `--features simd`.
Expected: PASS both (`≤ 1e-4` lane-for-lane vs scalar `Ms20`, both responses).

- [ ] **Step 7: Add a boundedness sanity test**

```rust
#[test]
fn polyms20_stays_bounded_at_high_res_and_drive() {
    let dt = 1.0 / 48_000.0;
    let n = 4096;
    let mut poly = PolyMs20::new();
    poly.set_drive(6.0);
    let audio = alloc::vec![0.5f32; n * VOICES];
    let mut out = alloc::vec![0.0f32; n * VOICES];
    poly.process(&audio, In::Const(8000.0), In::Const(0.99), Ms20Resp::Lp, dt, &mut out);
    for &s in &out { assert!(s.abs() <= 8.0001, "ms20 diverged: {s}"); }
}
```

Run both configs; Expected: PASS (output is `8·pade_tanh(·/8)`-bounded by construction).

- [ ] **Step 8: Commit**

```bash
git add crates/deluge-dsp-kernels/src/filter.rs crates/deluge-dsp-kernels/src/poly.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(kernels): PolyMs20 (scalar oracle + f32x8 ZDF/clip), null-tested"
```

---

## Task 4: Graph nodes (`PolyMoogLp4/Lp2`, `PolyMs20Lp/Hp`)

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs`
- Test: `node.rs` test module (mirror the existing `poly_svf` graph test)

**Interfaces:**
- Consumes: `PolyMoog<POLES>`, `PolyMs20` (Tasks 2–3), `Ms20Resp` (already imported for mono Ms20), `In`, `VOICES`.
- Produces: `Kind::PolyMoogLp4`, `Kind::PolyMoogLp2`, `Kind::PolyMs20Lp`, `Kind::PolyMs20Hp`; `State::PolyMoog4(PolyMoog<4>)`, `State::PolyMoog2(PolyMoog<2>)`, `State::PolyMs20(PolyMs20)`. Used by Task 5 factories.

- [ ] **Step 1: Add the `Kind` and `State` variants**

In the `Kind` enum (near the poly variants, node.rs:82-90) add `PolyMoogLp4, PolyMoogLp2, PolyMs20Lp, PolyMs20Hp`.
In the `State` enum (near `PolySvf(PolySvf)` :127) add `PolyMoog4(PolyMoog<4>), PolyMoog2(PolyMoog<2>), PolyMs20(PolyMs20)`.
Add constructor arms (beside :194):

```rust
            Kind::PolyMoogLp4 => State::PolyMoog4(PolyMoog::<4>::new()),
            Kind::PolyMoogLp2 => State::PolyMoog2(PolyMoog::<2>::new()),
            Kind::PolyMs20Lp | Kind::PolyMs20Hp => State::PolyMs20(PolyMs20::new()),
```

Add the imports for `PolyMoog`, `PolyMs20` to node.rs's `use deluge_dsp_kernels::...` block (beside `PolySvf`).

- [ ] **Step 2: Register in the three predicates**

Extend the match/`matches!` lists (node.rs:210, :218, :225) to include all four new kinds:

```rust
    // out_width (:210) — add to the VOICES arm:
    Kind::PolyCtrl | Kind::PolyOsc | Kind::PolyAr | Kind::PolySvf | Kind::PolyMul
        | Kind::PolyMtof | Kind::PolyAdd | Kind::PolyNoise
        | Kind::PolyMoogLp4 | Kind::PolyMoogLp2 | Kind::PolyMs20Lp | Kind::PolyMs20Hp => VOICES,

    // is_poly (:218) — add all four to the matches! list.

    // poly_in_count (:225) — add all four to the `=> 1` arm (audio is the one poly edge):
    Kind::PolyOsc | Kind::PolySvf | Kind::VoiceSum | Kind::PolyMtof
        | Kind::PolyMoogLp4 | Kind::PolyMoogLp2 | Kind::PolyMs20Lp | Kind::PolyMs20Hp => 1,
```

Also add the four kinds to the no-op `process_resolved` poly arm (:703) so the exhaustive match still compiles.

- [ ] **Step 3: Add the `poly_process` arms**

Beside the `PolySvf` arm (:737):

```rust
            Kind::PolyMoogLp4 => {
                if let (State::PolyMoog4(m), Some(audio)) = (&mut self.state, poly_in[0]) {
                    m.process(audio, ins[1], ins[2], dt, out);
                }
            }
            Kind::PolyMoogLp2 => {
                if let (State::PolyMoog2(m), Some(audio)) = (&mut self.state, poly_in[0]) {
                    m.process(audio, ins[1], ins[2], dt, out);
                }
            }
            Kind::PolyMs20Lp | Kind::PolyMs20Hp => {
                let resp = if matches!(self.kind, Kind::PolyMs20Hp) { Ms20Resp::Hp } else { Ms20Resp::Lp };
                if let (State::PolyMs20(m), Some(audio)) = (&mut self.state, poly_in[0]) {
                    m.process(audio, ins[1], ins[2], resp, dt, out);
                }
            }
```

> `self.kind` is read before the `if let` borrows `self.state` mutably — bind `let kind = self.kind;` first if the borrow checker complains, mirroring the mono `Ms20Lp | Ms20Hp` arm (:455-456).

- [ ] **Step 4: Write the graph render test**

Mirror the existing poly-SVF graph test (the one that builds `PolyCtrl→PolyOsc→PolySvf→PolyMul(·,PolyAr)→VoiceSum`). Add a parametrized version over the new filter kinds:

```rust
#[test]
fn poly_moog_and_ms20_voice_render_sound() {
    for fkind in [Kind::PolyMoogLp4, Kind::PolyMoogLp2, Kind::PolyMs20Lp, Kind::PolyMs20Hp] {
        // Build the same voice graph as the PolySvf test, substituting `fkind`
        // for the filter node; gate one voice, render a block, assert the summed
        // output is finite, bounded (|s| <= 8.1), and non-silent (some |s| > 1e-4).
        // (Reuse the PolySvf graph-test harness/helpers verbatim, swapping the kind.)
    }
}
```

> Follow the exact construction the existing PolySvf graph test uses (engine setup, `Cmd::GateVoice`, block size). The assertions: `s.is_finite()`, `s.abs() <= 8.1` (Ms20's ±8 contract), and at least one sample `> 1e-4`.

- [ ] **Step 5: Run to verify (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph poly_moog_and_ms20`
Then with `--features deluge-dsp-kernels/simd`.
Expected: PASS both.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(graph): PolyMoog/PolyMs20 nodes (Lp4/Lp2, Lp/Hp) + voice render test"
```

---

## Task 5: Wren surface + footgun close

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (add `node_polymoog_impl`, `node_polyms20_impl`)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (register `polymoog_`, `polyms20_`)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (flip `Moog`/`Ms20`; guard the rest)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Kind::PolyMoogLp4/PolyMoogLp2/PolyMs20Lp/PolyMs20Hp` (Task 4); the poly-node construction path (`audio::new_node` / the poly equivalent used by `node_polysvf_impl` @:1029); `Node.polyMode_` (Wren).
- Produces: Wren `Node.polymoog_(input, cutoff, res, poles)`, `Node.polyms20_(input, cutoff, res, resp)`; poly-aware `Moog`/`Ms20` classes; aborting `Svf`/`Tb303`/`Resonator` + 9 FX classes.

- [ ] **Step 1: Add the Rust poly factories**

In bindings_audio.rs, mirror `node_polysvf_impl` (@:1029, arity 3 audio+cutoff+res). Add `node_polymoog_impl` (arity 4: audio, cutoff, res, poles) selecting `Kind::PolyMoogLp4` when poles==4 else `Kind::PolyMoogLp2`; and `node_polyms20_impl` (arity 4: audio, cutoff, res, resp) selecting `Kind::PolyMs20Hp` when resp==1 else `Kind::PolyMs20Lp`. Each builds the poly node with inputs `[audio, cutoff, res]` (poles/resp choose the Kind, they are NOT input ports), exactly as `node_polysvf_impl` does for its `[audio, cutoff, res]`.

```rust
// sketch — match node_polysvf_impl's exact node-construction call and arg extraction
fn node_polymoog_impl(vm: &VM) {
    let audio = /* arg 1 as node/port handle, as polysvf does */;
    let cutoff = /* arg 2 */;
    let res = /* arg 3 */;
    let poles = /* arg 4 as f64 → usize */;
    let kind = if poles as i32 == 4 { Kind::PolyMoogLp4 } else { Kind::PolyMoogLp2 };
    /* new poly node of `kind` with inputs [audio, cutoff, res]; return handle */
}
// node_polyms20_impl identical, arg 4 = resp (0 => PolyMs20Lp, 1 => PolyMs20Hp)
```

- [ ] **Step 2: Register the factories**

In bindings.rs: add `polymoog_(_,_,_,_)` and `polyms20_(_,_,_,_)` to the METHODS table and wire them to the impls in `register_audio`, exactly as `polysvf_(_,_,_)` (@:1462) is wired.

- [ ] **Step 3: Flip `Moog`/`Ms20` in prelude.wren**

Replace the `Moog` class (:452) and `Ms20` class (:461):

```wren
class Moog {
  static lp(input, cutoff, res)  {
    if (Node.polyMode_ == 1) return Node.polymoog_(input, cutoff, res, 4)
    return Node.moog_(input, cutoff, res, 4)
  }
  static lp2(input, cutoff, res) {
    if (Node.polyMode_ == 1) return Node.polymoog_(input, cutoff, res, 2)
    return Node.moog_(input, cutoff, res, 2)
  }
}
class Ms20 {
  static lp(input, cutoff, res) {
    if (Node.polyMode_ == 1) return Node.polyms20_(input, cutoff, res, 0)
    return Node.ms20_(input, cutoff, res, 0)
  }
  static hp(input, cutoff, res) {
    if (Node.polyMode_ == 1) return Node.polyms20_(input, cutoff, res, 1)
    return Node.ms20_(input, cutoff, res, 1)
  }
}
```

- [ ] **Step 4: Guard the remaining classes**

Add a leading `if (Node.polyMode_ == 1) Fiber.abort("...")` as the first statement of every remaining still-mono factory. Filters:

```wren
// Svf (:432) — every factory. Poly SVF LP is available via `.lpf(cutoff)`.
static lp(input, cutoff, res) {
  if (Node.polyMode_ == 1) Fiber.abort("Svf inside a Synth isn't poly yet — use .lpf(cutoff) for a poly lowpass (Sy-2c)")
  return Node.svf_(input, cutoff, res, 0)
}
// ...hp/bp/notch: same guard message.

// Tb303 (:444)
static lp(input, cutoff, res) {
  if (Node.polyMode_ == 1) Fiber.abort("Tb303 not usable in a Synth yet (Sy-2c)")
  return Node.tb303_(input, cutoff, res)
}

// Resonator (:475)
static new(input, freq, damping) {
  if (Node.polyMode_ == 1) Fiber.abort("Resonator not usable in a Synth yet (Sy-2c)")
  return Node.modal_(input, freq, damping)
}
```

FX classes (`Pan` :486, `Delay` :499, `Chorus` :507, `Flanger` :515, `Room` :523, `Hall` :532, `Plate` :541, `Drive` :549, `EQ` :560) — first statement of each factory:

```wren
if (Node.polyMode_ == 1) Fiber.abort("<Class> is an effect — apply it after the Synth's .out, not inside the voice")
```

(Use the class name in each message. These run post-VoiceSum, so the message says "after `.out`", not "yet".)

- [ ] **Step 5: Write the end-to-end + abort tests**

In tests/audio_bindings.rs (mirror the Sy-2b `synth_add_and_noise_render_sound` harness):

```rust
#[test]
fn synth_moog_and_ms20_render_sound() {
    // Moog voice
    run_and_render("Synth.new { |p| Moog.lp(Osc.saw(p), 1200, 0.85) * Env.ar(0.01, 0.3) }",
        /* noteOn, render, assert finite + bounded(|s|<=8.1) + non-silent */);
    // Ms20 voice
    run_and_render("Synth.new { |p| Ms20.hp(Osc.saw(p), 1200, 0.9) * Env.ar(0.01, 0.3) }", ...);
}

#[test]
fn non_poly_classes_abort_inside_synth() {
    for src in [
        "Synth.new { |p| Tb303.lp(Osc.saw(p), 400, 0.9) * Env.ar(0.01, 0.3) }",
        "Synth.new { |p| Resonator.new(Osc.saw(p), 220, 0.4) * Env.ar(0.01, 0.3) }",
        "Synth.new { |p| Svf.lp(Osc.saw(p), 1200, 0.6) * Env.ar(0.01, 0.3) }",
        "Synth.new { |p| Delay.new(Osc.saw(p), 0.2, 0.4) * Env.ar(0.01, 0.3) }",
    ] {
        assert_aborts(src); // the harness's existing "expect a Fiber.abort / VM error" helper
    }
}
```

> Use the exact end-to-end harness the Sy-2b tests use (`run_and_render` / the abort-expecting helper — match their real names). Assertions mirror Sy-2b: finite, `|s| <= 8.1`, at least one `|s| > 1e-4`.

- [ ] **Step 6: Run to verify (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core synth_moog_and_ms20 non_poly_classes_abort`
Then with `--features deluge-dsp-kernels/simd`.
Expected: PASS both.

- [ ] **Step 7: Regression — full wren-core suite, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core`
Then with `--features deluge-dsp-kernels/simd`.
Expected: PASS (no existing mono `Moog.lp`/`Ms20.hp`/FX test broken — the guards only fire in poly mode).

- [ ] **Step 8: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): poly Moog/Ms20 in Synth + close silent-mono footgun (guard non-poly classes)"
```

---

## Self-Review Notes (for the executor)

- **Oracle discipline:** every f32x8 path shares its coefficient/DC helper with the scalar path (`moog_coeffs`, `ms20_coeffs`, `Ms20::tick`/`tick_with_dt`, the throwaway-`OnePoleHp` DC coeff). If you find yourself re-deriving a coefficient in the SIMD branch, stop — route it through the shared helper so the null-test can actually pass at `1e-4`.
- **Phase/state convention:** PolyMoog/PolyMs20 are *post-increment* like the other poly nodes (state updated, then output written) — but here that is automatic because the scalar oracle is the identical per-sample kernel, not a separately-phased mono osc. No index-offset gymnastics like the Sy-2b saw cross-check.
- **`self.kind` borrow:** in the `poly_process` Ms20 arm, read `self.kind` into a local before borrowing `self.state`.
- **Deferred (NOT this plan):** Tb303 poly, Modal poly, Osc.sync/wavetable/PWM, Noise.pink/brown — all stay aborting. Do not poly-ify them here.
- **Import paths in tests:** the exact `use` for `Moog`/`Ms20`/`Ms20Resp`/`In`/`alloc::vec` must match what the current poly.rs and node.rs test modules already do — copy their convention rather than the illustrative paths above.
