# Wavetable SIMD read path Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.

**Goal:** A const-frequency fast path for `WtOsc::process` + `process_morph` that hoists the per-sample `log2f` mip-select (bit-exact scalar win) and vectorizes the cubic interp + crossfades with `core::simd` under the `simd` feature (tolerance-equivalent, null-tested).

**Architecture:** Extract a `sample_at_level(lo_levels_slices, hi_slices, frac, ph)` inner from `sample_one` (the mip-select stays in `sample_one`; the "interp at precomputed levels" becomes shared). Add a const-freq branch to `process`/`process_morph` that hoists the mip-select once per block; a `#[cfg(feature="simd")]` sub-branch vectorizes across `f32x8` lanes (closed-form phases + gather). The general (audio-rate) path is unchanged. Mirrors `math.rs`'s SIMD-fast-path + scalar-fallback + null-test.

**Tech Stack:** Rust `no_std` (`deluge-dsp-kernels`, `core::simd` behind `simd`); QA via `deluge-dsp-test` (`spectrum`, `cpu`). Host target `x86_64-unknown-linux-gnu`.

**Reference spec:** [wavetable SIMD](../specs/2026-07-08-wavetable-simd-design.md).

## Global Constraints

- **Scalar path bit-exact:** the const-freq scalar path (mip-select hoisted, phase still iterative) produces output BIT-IDENTICAL to today; all existing wavetable/compaction/morph tests + goldens pass unchanged on the DEFAULT build. The audio-rate (`In::A`) path is untouched.
- **SIMD path tolerance-equivalent:** under `--features simd`, output ≈ scalar within a documented f32 tolerance (NOT bit-exact — SIMD reorders/FMAs). Null-tested vs scalar, `math.rs`-style.
- **Alias thresholds hold on both paths** (default + `--features simd`) — don't loosen; re-measure only within tolerance drift.
- `no_std`; `core::simd` only under `#[cfg(feature="simd")]`; deterministic. Zero warnings.
- **Honest limit:** the on-device (Cortex-A9, NEON no-gather) speedup is a maintainer measurement; the plan measures host speedup (informational) — do NOT claim a device number.
- Test commands: `-p deluge-dsp-kernels` (default AND `--features simd`), `-p deluge-audio-graph` (+`--features simd`), `-p deluge-wren-core --features test-support`.

## Reference values

- `wavetable.rs` (post-morph): `WtOsc { phase }`; `sample_one(mips: &MipSet, ph: f32, dtp: f32) -> f32` (computes `flevel=log2f(dtp*N)`, `lo=flevel as usize` clamped, `hi=min(lo+1,nlev-1)`, `frac=flevel-lo`; `interp_cubic(levels[lo], ph)` + `interp_cubic(levels[hi], ph)`; crossfade `a+(b-a)*frac.clamp(0,1)`); `interp_cubic(table: &[f32], ph) -> f32` (uses `catmull_rom`, wrapped indices); `catmull_rom(y0,y1,y2,y3,t)`; `process(&mut self, mips: MipSet, freq: In, pmod: In, dt, out)` (per sample `dtp=freq.at(i)*dt`, `ph=frac(phase+pmod.at(i))`, `y=sample_one(&mips,ph,dtp)`, advance); `process_morph(&mut self, region, frames, freq, pmod, position, dt, out)` (per sample: 4 clamped frames, `sample_one` each, `catmull_rom` frame blend); `COMPACT_LEN`, `compact_levels`, `LEVELS`, `N`, `floorf`. `In::as_const() -> Option<f32>`.
- `deluge-dsp-test`: `spectrum::{analyze_buf, worst_alias_db}`, `cpu` module (timing/compare). Crate `#![cfg_attr(feature="simd", feature(portable_simd))]`; `math.rs` uses `core::simd::f32x8` behind `#[cfg(feature="simd")]`.

---

## Task 1: Const-freq restructure + mip-select hoist (bit-exact scalar)

**Files:** Modify `crates/deluge-dsp-kernels/src/wavetable.rs`.

**Interfaces:**
- Produces: `fn mip_select(dtp: f32, nlev: usize) -> (usize, usize, f32)` (the `flevel`/`lo`/`hi`/`frac` computation extracted from `sample_one`); `fn sample_at_level(lo: &[f32], hi: &[f32], frac: f32, ph: f32) -> f32` (the 2×`interp_cubic` + crossfade); `sample_one` = `mip_select` then `sample_at_level`. Const-freq scalar branches in `process`/`process_morph`.

- [ ] **Step 1: Extract `mip_select` + `sample_at_level` (bit-exact)**

Split `sample_one`'s body: `mip_select(dtp, nlev) -> (lo, hi, frac)` (the `flevel`/clamp/`frac` computation VERBATIM); `sample_at_level(lo_slice, hi_slice, frac, ph)` = `let a=interp_cubic(lo_slice,ph); let b=interp_cubic(hi_slice,ph); a+(b-a)*frac.clamp(0.0,1.0)` (VERBATIM). `sample_one(mips, ph, dtp)` = `let (lo,hi,frac)=mip_select(dtp, mips.levels.len()); sample_at_level(mips.levels[lo], mips.levels[hi], frac, ph)`. **`sample_one` output MUST be bit-identical** (existing single-cycle/compaction/morph tests unchanged).

- [ ] **Step 2: Write the bit-exact const-freq regression test**

```rust
    #[test]
    fn const_freq_hoist_is_bit_exact() {
        // The hoisted const-freq scalar path must equal the general per-sample path bit-for-bit.
        let m = /* build a saw MipSet (as existing tests do) */;
        let mut a = WtOsc::new(); let mut oa = [0.0f32; 512];
        a.process(/* MipSet */, In::K(220.0), In::K(0.0), 1.0/48_000.0, &mut oa);
        // Reference: force the general path by using an audio-rate (but constant-valued) buffer,
        // which takes the unchanged per-sample loop.
        let fbuf = [220.0f32; 512]; let pbuf = [0.0f32; 512];
        let mut b = WtOsc::new(); let mut ob = [0.0f32; 512];
        b.process(/* same MipSet */, In::A(&fbuf), In::A(&pbuf), 1.0/48_000.0, &mut ob);
        assert_eq!(oa, ob); // const-freq (hoisted) == audio-rate-const-valued (general) bit-for-bit
    }
```
(This pins that hoisting the mip-select doesn't change the numbers — the const path and the constant-valued audio-rate path must agree bit-for-bit. Add the analogous test for `process_morph` with const vs const-valued-audio-rate position/freq.)

- [ ] **Step 3: Run, verify fail (test references const/audio-rate parity not yet guaranteed) → implement**

Add the const-freq branch to `process`:
```rust
    pub fn process(&mut self, mips: MipSet, freq: In, pmod: In, dt: f32, out: &mut [f32]) {
        if let (Some(f), Some(pm)) = (freq.as_const(), pmod.as_const()) {
            let dtp = f * dt;
            let (lo, hi, frac) = mip_select(dtp, mips.levels.len());
            let (lo_s, hi_s) = (mips.levels[lo], mips.levels[hi]);
            for s in out.iter_mut() {
                let mut ph = self.phase + pm; ph -= floorf(ph);
                *s = sample_at_level(lo_s, hi_s, frac, ph);
                self.phase += dtp; self.phase -= floorf(self.phase);
            }
            return;
        }
        // general per-sample path (UNCHANGED) ...
    }
```
(Phase advance stays iterative → bit-exact. `pmod` const folded into `ph` per sample same as the general path.) Do the analogous const-freq branch in `process_morph`: `freq`/`pmod`/`position` all const → hoist `mip_select` (shared across frames) + the frame bracket, loop `catmull_rom` over `sample_at_level(frame_f_lo, frame_f_hi, frac, ph)` for the 4 frames.

- [ ] **Step 4: Run, verify pass; confirm bit-exact + no per-sample log2f**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`. Expected: PASS — the new bit-exact tests + ALL existing wavetable/compaction/morph tests UNCHANGED (the const-freq path is what those tests exercise, now hoisted, bit-identical). Confirm `log2f` (via `mip_select`) is now called once per block in the const path, not per sample. Zero warnings.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/wavetable.rs
git commit -m "perf(wavetable): const-freq mip-select hoist (bit-exact; log2f out of hot loop)"
```

---

## Task 2: SIMD const-freq blocks + null-test + speedup

**Files:** Modify `crates/deluge-dsp-kernels/src/wavetable.rs`; add a `cpu`-based speedup test.

**Interfaces:** `#[cfg(feature="simd")]` SIMD sub-branch inside the const-freq path of `process`/`process_morph`; `simd_sample_lanes(...)` (or inline) using `f32x8`.

- [ ] **Step 1: Write the failing null-test (tolerance)**

```rust
    #[cfg(feature = "simd")]
    #[test]
    fn simd_matches_scalar_within_tol() {
        // Build a saw MipSet; render via the SIMD const-freq path.
        // Compare against the scalar const-freq path (temporarily forced, or the audio-rate
        // path) — must match within f32 tolerance (NOT bit-exact).
        // ... assert per-sample |simd - scalar| < 1e-4 (tol set from measurement) ...
    }
```
Add the analogous morph null-test. (Gate the whole test `#[cfg(feature="simd")]` so it only runs under the feature.)

- [ ] **Step 2: Run, verify fail (no SIMD path yet) → implement the SIMD block**

Inside the const-freq branch of `process`, add (before the scalar loop):
```rust
        #[cfg(feature = "simd")]
        {
            use core::simd::{f32x8, Simd, num::SimdFloat, StdFloat};
            // Precompute lane offsets [0,1,..,7]; process 8 output samples per chunk:
            //  - phase lanes = frac(phase + pm + [0..8]*dtp)   (closed-form; tolerance vs iterative)
            //  - for lo_s and hi_s: per-lane x=ph*n, i1 = (x as idx) % n, fr = x-floor(x),
            //    gather the 4 consecutive taps (i0,i1,i2,i3 with wrap) → f32x8 vectors,
            //    Catmull-Rom via `catmull_rom`-equivalent f32x8 arithmetic,
            //  - crossfade lo/hi by the constant `frac` (FMA across lanes),
            //  - store 8; advance phase by 8*dtp.
            // Scalar-tail the remaining < 8 samples via sample_at_level.
            // (Model the structure on math.rs's f32x8 chunk loop + gather_or for the taps.)
            // ... return;
        }
```
Mirror `math.rs`'s chunk structure. Use `Simd::gather_or` (or per-tap index gather) for the table taps; keep the Catmull-Rom coefficient math identical in form to `catmull_rom` but on `f32x8`. Do the analogous SIMD block in `process_morph` (4 frames' lanes + the frame-axis Catmull-Rom, all vectorized). If a construct (gather signature, `StdFloat::floor`) differs from what's available in this toolchain's `core::simd`, adapt — read `math.rs` for the exact imports that compile here.

- [ ] **Step 3: Run, verify pass (null-test) + tune tolerance**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd`. The null-tests must pass; MEASURE the max `|simd - scalar|` and set the tolerance just above it (document; expect ~1e-5–1e-4 from FMA/closed-form-phase differences). If the diff is large (> ~1e-3), the SIMD math has a bug (wrong gather index, wrong coefficient) — fix it, don't loosen. Also run WITHOUT `--features simd` (scalar path unchanged, bit-exact tests still pass).

- [ ] **Step 4: Add the speedup measurement (informational)**

Add a test (or `#[ignore]`d bench) using `deluge_dsp_test::cpu` to time the SIMD vs scalar const-freq `process` + `process_morph` block render; log the ratio. This is HOST (x86) — report it as informational (shows the log2f-hoist + arithmetic-vectorization win). **Comment that the real Cortex-A9 speedup is a maintainer on-device measurement (NEON has no gather).** Do NOT assert a specific speedup (host timing is noisy + not the device).

- [ ] **Step 5: Verify both feature modes + downstream**

Run: `cargo test -p deluge-dsp-kernels` (default: scalar bit-exact) AND `--features simd` (null-test + thresholds). Run `-p deluge-audio-graph` (+`--features simd`) and `-p deluge-wren-core --features test-support` — all green, wavetable alias thresholds hold on both paths, goldens unchanged. Zero warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-dsp-kernels/src/wavetable.rs
git commit -m "perf(wavetable): SIMD const-freq interp (f32x8) for process + process_morph; null-tested"
```

---

## Self-review notes

- **Spec coverage:** const-freq hoist (Task 1) + SIMD vectorization (Task 2), both `process`/`process_morph`; QA bit-exact-scalar + null-test-SIMD + thresholds-both-paths + cpu speedup (Tasks 1–2).
- **Scalar bit-exact:** `sample_one`/`sample_at_level`/`mip_select` extraction is behavior-preserving; the const-freq scalar path keeps iterative phase → bit-identical (the `const_freq_hoist_is_bit_exact` test pins it, plus all existing tests unchanged on the default build).
- **SIMD tolerance:** closed-form phase + f32x8 FMA → not bit-exact; the null-test asserts ≈ scalar within a measured tolerance (`math.rs` precedent). NOT compared to goldens (goldens are the scalar default).
- **Honest device limit:** host speedup measured/reported; Cortex-A9 (no-gather) number is the maintainer's — stated, not asserted.
- **Known follow-ups:** audio-rate-freq SIMD (prefix-sum phases); `Osc`/`SyncOsc` SIMD; on-device tuning; SoA-across-voices.
