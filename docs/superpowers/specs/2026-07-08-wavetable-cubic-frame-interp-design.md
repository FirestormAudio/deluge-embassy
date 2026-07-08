# Cubic inter-frame interpolation — design & spec

A small refinement to [multi-frame morph](2026-07-07-wavetable-multiframe-morph-design.md):
upgrade the frame-axis blend in `WtOsc::process_morph` from **linear** (2 frames) to
**Catmull-Rom cubic** (4 frames), removing the slope discontinuity in timbre as
`position` crosses a frame boundary.

> **Status:** design proposal. Depends on merged multi-frame morph (`process_morph`,
> `sample_one`). Kernel-only change; no graph/Wren surface change.

---

## 1. Goals & non-goals

**Goals**
- **Smooth morph.** `position` crossing a frame boundary no longer has a slope kink
  (audible on fast / audio-rate position sweeps). Catmull-Rom over the 4 frames
  `f-1, f0, f1, f2` (each clamped to `[0, FRAMES-1]`), blended by `ffrac`.
- **Reuse the existing cubic.** Same Catmull-Rom coefficients as the phase-axis
  `interp_cubic` (`a=-0.5y0+1.5y1-1.5y2+0.5y3`, etc.), applied to the 4 frames'
  `sample_one` outputs.
- **Spectrally safe (unchanged).** Catmull-Rom is a linear filter → the result stays a
  linear combination of band-limited frames → still band-limited (no new harmonics).
  It's no longer convex, so slight amplitude overshoot near boundaries is possible —
  already covered by the existing `±1.2` bounds gate.

**Non-goals**
- Graph/Wren changes (the morph API, `.position`, `from2d`, banks are untouched).
- `FRAMES==1` (single-cycle `process` fast path) — untouched.
- Selectable linear-vs-cubic — **replace** linear (cubic is strictly smoother; the cost
  applies only to morph nodes).

---

## 2. Change (`deluge-dsp-kernels/src/wavetable.rs`)

`process_morph`'s per-sample frame blend:

- **Before:** `f0 = floor(fpos)`, `f1 = min(f0+1, last)`, `out = lerp(sample_one(f0), sample_one(f1), ffrac)`.
- **After:** `fm1 = max(f0-1, 0)`, `f0`, `f1 = min(f0+1, last)`, `f2 = min(f0+2, last)` (all clamped `[0, last]`); sample each via `sample_one` at the same `ph`/`dtp`; Catmull-Rom blend by `ffrac`:
  `y = catmull_rom(y_{fm1}, y_{f0}, y_{f1}, y_{f2}, ffrac)`.

`catmull_rom(y0,y1,y2,y3,t)` reuses the coefficient form already in `interp_cubic`
(extract it as a shared `fn catmull_rom(y0,y1,y2,y3,t) -> f32` so both the phase-axis and
frame-axis interps use one implementation). `FRAMES==2`: `fm1` clamps to `f0`, `f2`
clamps to `f1` — a valid, smooth degenerate case. Cost rises from 2 to 4 `sample_one`
calls per sample on morph nodes.

---

## 3. QA acceptance & testing

- **Endpoints/exact-frame unchanged.** At `position` on an exact frame (`ffrac==0`),
  Catmull-Rom returns `y_{f0}` exactly (the interpolant passes through the control
  point) — so exact-frame renders equal the single-frame render, same as before.
  Update `morph_position_endpoints_and_midpoint`: endpoints still exact; the MIDPOINT is
  now the Catmull-Rom value (≈ but not exactly the linear average for 4 distinct frames)
  — assert it's between the two bracketing frames / matches the cubic formula, not the
  linear average.
- **Smoother than linear (the point).** A `position` sweep across a boundary has a
  continuous first derivative — assert the RMS (or per-sample) trend has no slope jump
  at the integer boundary that the linear version had. (A gentle continuity gate; the
  existing `morph_sweep_is_continuous` should still pass, ideally tighter.)
- **Bounded.** Output still finite and within `±1.2` (overshoot allowed) over swept
  position — the existing bounds proptest/gate.
- **Band-limit preserved.** A morphed output at high pitch still clears the `< -21 dB`
  alias gate (linear combination of band-limited frames).
- **`FRAMES==1` bit-exact** (routes to `process`, untouched) — unchanged.
- **Shared `catmull_rom`** doesn't change the phase-axis `interp_cubic` output
  (bit-exact after extraction) — the single-cycle + compaction tests stay green.

---

## 4. Deferred / follow-ups

- Hermite/other spline variants; selectable interpolation order.
- (Unrelated, tracked elsewhere:) async device upload of large dynamic banks; hard sync;
  noise variants.
