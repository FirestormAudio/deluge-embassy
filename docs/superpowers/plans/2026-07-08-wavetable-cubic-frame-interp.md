# Cubic inter-frame interpolation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Upgrade `WtOsc::process_morph`'s frame-axis blend from linear (2 frames) to Catmull-Rom cubic (4 clamped frames), reusing the existing cubic coefficients — smoother morph, still band-limited, `FRAMES==1` untouched.

**Architecture:** Extract the Catmull-Rom coefficient math from `interp_cubic` into a shared `fn catmull_rom(y0,y1,y2,y3,t) -> f32`; `interp_cubic` and the new frame blend both call it. Single-file kernel change (`crates/deluge-dsp-kernels/src/wavetable.rs`).

**Reference spec:** [cubic frame interp](../specs/2026-07-08-wavetable-cubic-frame-interp-design.md).

## Global Constraints

- **`interp_cubic` bit-exact** after extracting `catmull_rom` (the phase-axis single-cycle/compaction tests must be UNCHANGED).
- **`FRAMES==1`** routes to `process` (graph, unchanged) — no morph path involved; single-cycle bit-exact.
- Catmull-Rom is a linear filter → morph stays band-limited (no new anti-aliasing). Overshoot allowed within the existing `±1.2` bounds gate.
- `no_std`; zero warnings. Test: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels` (+ `-p deluge-audio-graph`, `-p deluge-wren-core --features test-support` for regression).

---

## Task 1: Catmull-Rom frame blend in `process_morph`

**Files:** Modify `crates/deluge-dsp-kernels/src/wavetable.rs`.

**Interfaces:** `fn catmull_rom(y0: f32, y1: f32, y2: f32, y3: f32, t: f32) -> f32` (shared); `process_morph`'s frame blend uses 4 clamped frames.

- [ ] **Step 1: Extract `catmull_rom` (behavior-preserving)**

`interp_cubic` currently computes (verbatim from the current code) something like:
```rust
let a = -0.5*y0 + 1.5*y1 - 1.5*y2 + 0.5*y3;
let b = y0 - 2.5*y1 + 2.0*y2 - 0.5*y3;
let c = -0.5*y0 + 0.5*y2;
((a*frac + b)*frac + c)*frac + y1
```
Extract exactly that into `fn catmull_rom(y0,y1,y2,y3,t) -> f32` (use the CURRENT coefficients verbatim — read the file, don't assume) and have `interp_cubic` call it with its 4 taps + `frac`. `interp_cubic`'s output must be bit-identical (verify: existing single-cycle/compaction tests unchanged).

- [ ] **Step 2: Write the failing frame-cubic tests**

Update/add in the tests:
- `morph_position_endpoints_and_midpoint`: endpoints still exact (`ffrac==0` → `catmull_rom` returns the control point `y_{f0}` exactly — Catmull-Rom passes through control points); change the MIDPOINT assertion from "linear average" to the Catmull-Rom value (compute the expected `catmull_rom(y_fm1, y_f0, y_f1, y_f2, 0.5)` for the test's frames, or assert the output is between the bracketing frames and matches the 4-tap formula — NOT the 2-tap average).
- Add `morph_cubic_frame_read_uses_four_frames` (or similar): with ≥4 distinct frames, at a non-integer position the output depends on `f-1` and `f+2` (differs from the 2-frame linear result) — proving the cubic reads 4 frames.

- [ ] **Step 3: Run, verify fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`. Expected: FAIL (midpoint assertion / four-frame test against the still-linear blend).

- [ ] **Step 4: Implement the cubic frame blend**

In `process_morph`, replace the 2-frame linear blend. Per sample (after `fpos`, `f0=floor(fpos)`, `ffrac=fpos-f0`, `last=frames-1`):
```rust
let fm1 = if f0 == 0 { 0 } else { f0 - 1 };
let f0c = if f0 > last { last } else { f0 };
let f1 = if f0c + 1 > last { last } else { f0c + 1 };
let f2 = if f0c + 2 > last { last } else { f0c + 2 };
let ym1 = sample_one(&MipSet { levels: &compact_levels(&region[fm1*COMPACT_LEN..(fm1+1)*COMPACT_LEN]) }, ph, dtp);
let y0  = sample_one(&MipSet { levels: &compact_levels(&region[f0c*COMPACT_LEN..(f0c+1)*COMPACT_LEN]) }, ph, dtp);
let y1  = sample_one(&MipSet { levels: &compact_levels(&region[f1 *COMPACT_LEN..(f1 +1)*COMPACT_LEN]) }, ph, dtp);
let y2  = sample_one(&MipSet { levels: &compact_levels(&region[f2 *COMPACT_LEN..(f2 +1)*COMPACT_LEN]) }, ph, dtp);
*s = catmull_rom(ym1, y0, y1, y2, ffrac.clamp(0.0, 1.0));
```
(Keep the existing `frames==0 || region.len() < frames*COMPACT_LEN` guard. `FRAMES==1` never reaches here — the graph routes it to `process`; but if `process_morph` is ever called with `frames==1`, `fm1=f0c=f1=f2=0` → `catmull_rom(y,y,y,y,t)=y`, still correct.)

- [ ] **Step 5: Run, verify pass (all suites)**

Run: `cargo test -p deluge-dsp-kernels` (frame-cubic tests + single-cycle/compaction unchanged), `-p deluge-audio-graph` (+`--features simd`), `-p deluge-wren-core --features test-support` (morph round-trip still finite/non-silent — it may render slightly different values now; the from2d test asserts finite/non-silent/position-selects, which still holds — update if it asserted an exact linear midpoint). All green, `±1.2` bounds hold, `< -21 dB` gate holds, Osc goldens unchanged. Zero warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-dsp-kernels/src/wavetable.rs
git commit -m "feat(wavetable): Catmull-Rom cubic inter-frame morph interpolation"
```

---

## Self-review notes

- **Spec coverage:** shared `catmull_rom` + 4-frame cubic blend (Task 1); QA endpoints/midpoint/four-frame/bounds/band-limit (Task 1 tests).
- **`interp_cubic` bit-exact** after extraction — verified by unchanged single-cycle/compaction tests; `catmull_rom` uses the CURRENT coefficients verbatim.
- **Cost:** 2→4 `sample_one`/sample on morph nodes only; `FRAMES==1`/single-cycle untouched.
- **Spectrally safe:** linear filter over band-limited frames; overshoot within `±1.2`.
- **Watch:** any morph test that asserted the *linear* midpoint average must be updated to the Catmull-Rom value (the endpoints stay exact).
