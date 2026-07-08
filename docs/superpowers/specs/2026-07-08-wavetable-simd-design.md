# Wavetable SIMD read path — design & spec

A performance pass over the wavetable oscillator: a **const-frequency fast path** that
(a) hoists the per-sample mip-select (`log2f`) out of the loop — a bit-exact scalar win —
and (b) vectorizes the cubic interpolation + crossfades with `core::simd` under the
`simd` feature. Covers both `WtOsc::process` (single-cycle) and `process_morph` (2D).

> **Status:** design proposal. Depends on merged wavetables (single-cycle, compaction,
> 2D morph) + the `simd` feature (`core::simd`, portable-SIMD → NEON/AVX) already used by
> `math.rs`. Mirrors `math.rs`'s precedent: `#[cfg(feature="simd")]` fast path + scalar
> fallback, the two null-tested against each other.

**Scope (chosen):** const-freq restructure + SIMD for `WtOsc::process` AND
`process_morph`. **Deferred:** audio-rate-freq vectorization (prefix-sum phases); SIMD on
the analytic `Osc`/`SyncOsc` (branchy PolyBLEP — separate, harder).

---

## 1. Goals & non-goals

**Goals**
- **Const-freq mip-select hoist (bit-exact scalar win).** When `freq`/`pmod` are const
  over a block (`In::as_const()`), compute the mip-select (`flevel=log2f(dtp·N)`,
  `lo`/`hi`/`frac`) ONCE per block instead of per sample. The per-sample `log2f` (a
  flagged device cost) leaves the hot loop. Phase advance stays iterative → **bit-exact**
  with today's scalar output.
- **SIMD interp (feature-gated, tolerance-equivalent).** Under `simd`, vectorize across
  output lanes (`f32x8`): closed-form phase vector, gather the cubic taps, Catmull-Rom +
  inter-mip (and, for morph, inter-frame) blend with constant coefficients (mip/frame
  brackets are const per block). Covers `WtOsc::process` + `process_morph`.
- **Scalar fallback = default = reference.** Audio-rate `freq`/`pmod`/`position`, or
  `simd` off → the current per-sample path (unchanged). Goldens + wavetable `worst_alias_db`
  thresholds hold on every path.
- **QA-proven:** const-freq scalar path bit-exact vs today; SIMD path null-tested ≈ scalar
  within f32 tolerance; alias thresholds hold on both; a `cpu` speedup measurement.

**Non-goals (deferred / out of scope)**
- Vectorizing the **audio-rate-freq** path (true serial phase recurrence with per-sample
  `pmod` — would need a prefix-sum; stays scalar this cut).
- SIMD on `Osc`/`SyncOsc` (per-sample PolyBLEP branch — a separate, harder pass).
- Bit-exact SIMD (impossible — SIMD reorders/FMAs f32; tolerance is the contract).
- On-device speedup *verification* (Cortex-A9 NEON has no gather — the real device number
  is a maintainer measurement; §4).

---

## 2. Architecture (`deluge-dsp-kernels/src/wavetable.rs`)

Both `process` and `process_morph` gain the same three-way structure (mirroring
`math.rs`): **const-freq SIMD** → **const-freq scalar (hoisted)** → **general scalar**.

```
fn process(...):
    if freq.as_const().is_some() && pmod.as_const().is_some() /* (+ position for morph) */ {
        let (lo, hi, frac, dtp) = mip_select_once(dtp_const);   // hoisted; no per-sample log2f
        #[cfg(feature = "simd")] { return simd_const_freq_block(lo, hi, frac, dtp, ...); }
        return scalar_const_freq_block(lo, hi, frac, dtp, ...);  // bit-exact: iterative phase
    }
    // general path: the current per-sample loop (audio-rate freq/pmod) — UNCHANGED.
```

- **`scalar_const_freq_block`**: iterate `phase += dtp` as now; each sample calls the
  existing `sample_one` inner (cubic + crossfade) with the hoisted `lo`/`hi`/`frac`.
  Output **bit-identical** to the current per-sample loop (same phase sequence, same
  mip values). The only change: mip-select computed once, not per sample.
- **`simd_const_freq_block`** (feature): process `f32x8` lanes. Per chunk: phase lane
  vector `= frac(phase0 + [0..8]·dtp)`; for the two bracketing mip levels compute per-lane
  table indices + fractions, **gather** the 4 cubic taps (consecutive per lane), evaluate
  Catmull-Rom with `f32x8` (coefficients derived from the gathered taps), and blend the
  two levels by the constant `frac` (an FMA across lanes). Advance `phase0 += 8·dtp` per
  chunk; scalar-tail the remainder. Closed-form phase → **tolerance-equivalent**, not
  bit-exact.
- **`process_morph`**: same, but const `position` → the frame bracket (`f-1..f+2`) is
  constant per block, so the four frames' mip-select is hoisted once and the four
  per-frame cubic reads + the Catmull-Rom frame blend vectorize across lanes. (If
  `position` is audio-rate, fall to scalar.)

**The gather.** `Simd::gather_or` with per-lane index vectors. A cubic tap is 4
*consecutive* table entries → contiguous; the parallelism is *across lanes* (8 different
base indices) → a gather, which LLVM lowers to native gather on AVX/SVE and to scalar
loads on Cortex-A9 NEON (the memory half stays scalar on-device; the arithmetic
vectorizes). Handle table wrap at the ends (indices `% n`) in the index computation.

`no_std`; the `simd` path is `#[cfg(feature = "simd")]` + `feature(portable_simd)` (already
enabled crate-wide behind `simd`). Deterministic.

---

## 3. Graph / callers

No graph or Wren change. `node.rs`'s `Kind::Wavetable` arm calls `process`/`process_morph`
as today; the fast path is internal. The `simd` feature already flows through the
`deluge-audio-graph` test matrix (`--features simd`), so the SIMD path is exercised there
automatically; the default build uses the (now-hoisted) scalar path.

---

## 4. QA acceptance & testing

`deluge-dsp-kernels` tests:

- **Const-freq scalar bit-exact (key regression).** For a const-freq block, the new
  hoisted scalar `process`/`process_morph` output equals the PRE-CHANGE output bit-for-bit
  (`assert_eq!` against a saved reference, or against a scalar re-implementation of the
  old per-sample loop). The default-build wavetable tests + goldens stay unchanged.
- **SIMD null-test (tolerance).** With `--features simd`, `process`/`process_morph` output
  ≈ the scalar path within a documented f32 tolerance (e.g. `< 1e-4` abs, or matching
  `worst_alias_db`), for single-cycle and morph, across freqs/positions. This is the
  `math.rs`-style two-paths-null-test. NOT bit-exact (SIMD reorders).
- **Alias thresholds hold on both paths.** The existing `worst_alias_db < gate`,
  compact-fidelity, and morph tests pass under BOTH default and `--features simd`
  (re-measure if a tight gate drifts within tolerance — do not loosen).
- **Speedup measurement.** Use `deluge_dsp_test::cpu` to time the scalar vs SIMD (and vs
  the old per-sample) block render for `process` + `process_morph`; report the ratio
  (informational, host x86 — shows the arithmetic + log2f-hoist win). **Honest limit:
  the real Cortex-A9 speedup (NEON no-gather) is a maintainer on-device measurement** —
  stated in the report, not asserted.
- **Audio-rate path unchanged.** A block with audio-rate `freq` (`In::A`) takes the
  general scalar path and is bit-identical to today (the fast path only triggers on
  const inputs).

**Determinism:** no RNG; scalar path reproducible; SIMD path reproducible per platform
(f32 order fixed by the code, tolerance covers platform FMA differences).

---

## 5. Deferred / follow-ups

- Audio-rate-freq SIMD (prefix-sum phase generation across the block).
- SIMD on `Osc`/`SyncOsc` (branchy PolyBLEP — const-freq closed-form + masked correction).
- On-device (Cortex-A9) speedup verification + tuning (gather cost, lane width).
- SoA-across-voices batching (vectorize N voices rather than N samples) — a different axis.
