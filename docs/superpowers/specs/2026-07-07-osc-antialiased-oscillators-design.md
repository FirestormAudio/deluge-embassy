# Osc — anti-aliased classic oscillators (PolyBLEP) — design & spec

Sub-project `Osc` of the [DSP library cluster](2026-07-06-dsp-library-vision-design.md),
first cut. Replace P0's naïve saw/square/triangle — which alias badly at high
frequencies — with **band-limited versions using PolyBLEP**, proven with the
[`QA` harness](2026-07-07-qa-measurement-harness-design.md)'s `worst_alias_db`
aliasing metric. Sine is already alias-free and is left unchanged.

> **Status:** design proposal. Depends on P0 (the `Osc` kernel + graph nodes) and
> QA (the `deluge-dsp-test` aliasing/spectral harness), both merged. Resolves the
> cluster vision's open question "anti-aliasing strategy — PolyBLEP vs wavetable"
> in favour of **PolyBLEP** for the analytic classics.

**First-cut scope (chosen):** anti-alias the classic waveforms only. **Deferred to
follow-on sub-projects:** wavetable oscillators, dedicated FM operators
(feedback/through-zero — basic FM/PM already works via patching a node into the
freq input), hard sync, and noise variants. A sub oscillator is just composition
(`Osc.saw(f/2)`), not a new kind.

---

## 1. Goals & non-goals

**Goals**

- Band-limited **saw**, **square**, **triangle** via PolyBLEP (saw/square) and
  BLAMP (triangle), replacing the naïve waveforms **in place** — same node
  `Kind`s, same Wren surface.
- **QA-proven**: each wave's `worst_alias_db` clears a target across the audio
  range, and is demonstrably far better than the naïve version.
- **Transparent upgrade**: `Osc::process`'s signature is unchanged, so the graph
  (`deluge-audio-graph`) and Wren (`deluge-wren-core`) layers are untouched.
- Correct behaviour under **modulated frequency** (`In::A` freq), since PolyBLEP
  detects discontinuities per sample — the property that lets the suite grow into
  FM/sync later.

**Non-goals (deferred / out of scope)**

- Wavetable oscillators, dedicated FM operators, hard sync, noise variants, sub
  oscillator as a kind (see scope note above).
- SIMD. Per-sample discontinuity detection doesn't vectorize cleanly; oscillators
  stay scalar, consistent with P0 (SIMD remains a later optimization).
- Improving `Sine`/`fast_sin`. A pure sine has no harmonics to alias; its ~0.1%
  approximation error sits ~−60 dB. QA may spot-check it, but it's not a target.
- An absolute CPU-budget gate (that's the deferred on-device profiler); host CPU
  cost is measured only comparatively/informationally.

---

## 2. Architecture — a transparent kernel upgrade

The entire change lives in **`crates/deluge-dsp-kernels/src/osc.rs`**.

- `Osc::process(wave, freq: In, dt, out)` keeps its exact signature; only the
  per-sample computation changes (naïve wave → naïve + band-limiting correction).
- **Graph unchanged.** `Kind::Sine/Saw/Square/Tri` still map to
  `State::Osc(Osc::new())` and call the same `process`. No new kinds, no node
  changes.
- **Wren unchanged.** `Osc.saw(f)` etc. simply stop aliasing — a drop-in quality
  upgrade; existing scripts benefit for free.
- **`Osc` stays `{ phase: f32 }`** (the BLAMP triangle path needs no extra state).
  *If* the integrated-square triangle fallback (§3) is chosen instead, it adds one
  `tri_z: f32` integrator field — an internal detail, still transparent to callers.

Two ripples this sub-project owns:

1. **Golden regeneration.** P0's `golden_saw_lpf_env_first_block` and P1's
   end-to-end golden pinned the *naïve* saw's samples; band-limiting changes them.
   Re-pin both (an intended, reviewed characterization update).
2. **First QA consumer.** `deluge-dsp-kernels` gains `deluge-dsp-test` as a
   **dev-dependency** (path to the workspace-excluded crate — fine for host tests;
   device builds don't pull dev-deps). Its acceptance tests use `worst_alias_db`.

---

## 3. The PolyBLEP kernel

Band-limiting adds a correction at each waveform discontinuity, using the
per-sample phase increment `dtp = freq.at(i) * dt` as the correction width.

### 3.1 The 2-point PolyBLEP helper (band-limited step over ±1 sample)

```rust
/// Residual to correct a unit step discontinuity at phase `t` (in [0,1)),
/// given the per-sample phase increment `dtp`.
fn poly_blep(t: f32, dtp: f32) -> f32 {
    if t < dtp {
        let x = t / dtp;
        2.0 * x - x * x - 1.0
    } else if t > 1.0 - dtp {
        let x = (t - 1.0) / dtp;
        x * x + 2.0 * x + 1.0
    } else {
        0.0
    }
}
```

### 3.2 Per waveform

- **Saw:** `y = 2*phase - 1;  y -= poly_blep(phase, dtp);` (corrects the −2 jump
  at the wrap).
- **Square:** `y = if phase < 0.5 { 1.0 } else { -1.0 };
  y += poly_blep(phase, dtp);  y -= poly_blep((phase + 0.5).fract(), dtp);`
  (rising edge at 0, falling edge at 0.5).
- **Triangle:** slope (not step) discontinuities at phase 0 and 0.5, corrected
  with **BLAMP** (band-limited ramp = integral of BLEP) — keeps `Osc` stateless
  beyond `phase` and avoids DC drift. **Fallback** (impl choice if BLAMP is
  fiddly): leaky-integrate a band-limited square (adds `tri_z`); QA picks whichever
  clears §4 cleaner within budget.
- **Sine:** unchanged (`fast_sin`).

Per-sample loop: read `freq.at(i)`, compute `dtp`, emit naïve wave + correction,
advance and wrap `phase` (via the existing `floorf`).

### 3.3 Order as a QA-tuned knob

Start with 2-point (above), which gives roughly −40…−50 dB worst-case alias at
high fundamentals. If §4's thresholds aren't met within the CPU budget, raise
saw/square to **4-point PolyBLEP** (a wider polynomial residual). The design
commits to *"the cheapest order that clears §4,"* not a fixed order.

### 3.4 Edge behaviour

- Modulated freq (`In::A`): `dtp` varies per sample — handled naturally.
- Near Nyquist (`dtp → 0.5`): 2-point PolyBLEP degrades (expected); §4 tests up to
  ~8 kHz where it holds.
- Very low freq (`dtp` tiny): correction windows shrink to nothing — naïve wave,
  which is already alias-free at low freq.

---

## 4. QA acceptance & testing

`deluge-dsp-kernels`'s tests (host, via the QA dev-dependency) prove the result:

- **Aliasing gate (per wave).** Render each band-limited wave at high fundamentals
  (2 k / 5 k / 8 k Hz at 48 kHz) into a buffer, `deluge_dsp_test::spectrum::analyze`
  it, and assert `worst_alias_db` below target: **saw/square < −40 dB**,
  **triangle < −50 dB**, up to ~8 kHz. (Targets; the impl tunes PolyBLEP order to
  meet them and the tests record the measured values.)
- **Improvement over naïve.** In the same test, render a *naïve* saw inline at the
  same frequency and assert the band-limited version is **> 20 dB better** —
  proving the correction does real work, not just clearing an absolute bar.
- **Low-frequency fidelity.** At ~100 Hz, assert the tone is intact (fundamental
  present, harmonic series present, `worst_alias_db` already deep) — band-limiting
  must not gut the sound where there's nothing to fix.
- **Bounds property test (loosened).** P0's osc proptest asserts output in ±1.001;
  PolyBLEP can slightly overshoot at discontinuities (Gibbs-like), so loosen the
  bound to **±1.1** — still finite, no NaN/denormal, over random wave/freq.
- **Golden regeneration.** Re-capture `golden_saw_lpf_env_first_block`
  (deluge-audio-graph) and P1's end-to-end golden (deluge-wren-core) expected
  arrays; document as an intended re-pin caused by band-limiting.
- **CPU cost (informational).** Optionally `deluge_dsp_test::cpu::compare`
  band-limited vs naïve and log the ratio — reported, not gated.

**Determinism:** oscillators are deterministic (no RNG), so golden re-pins and
`worst_alias_db` values are reproducible across host and device.

---

## 5. Deferred (tracked follow-ups)

- Wavetable oscillator (band-limited tables + mip-mapping) — its own sub-project;
  needs table storage (the buffer pool, whose node integration is itself deferred).
- Dedicated **FM operators** (feedback, through-zero) — basic FM/PM already works
  via patching.
- **Hard sync** (anti-aliased via BLEP at the sync reset).
- **Noise variants** (pink/brown) — separable from oscillators.
- **SIMD** oscillator path (e.g. const-freq closed-form + vectorized correction).
- 4-point / higher-order PolyBLEP if a tighter alias floor is wanted later.

---

## 6. Open questions (resolved during implementation)

- Triangle method: **BLAMP** (preferred, stateless) vs integrated-band-limited-
  square (fallback, adds `tri_z`) — decided by which clears §4 within budget.
- PolyBLEP order per wave (2- vs 4-point) — the cheapest that meets §4.
- Exact `worst_alias_db` thresholds and the fundamental frequencies swept (the §4
  numbers are targets; final values set from measured results).
- Whether `Osc::process` keeps a single `match wave` or splits into per-wave
  helpers for readability (the correction code differs enough per wave).
