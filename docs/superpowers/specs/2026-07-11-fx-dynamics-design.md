# Fx-1: Dynamics — Compressor / Limiter — Design Spec

**Date:** 2026-07-11
**Suite:** Fx (audio effects) — sub-project Fx-1, the FIRST of a new effects suite,
following the completed Sy voice/synth suite (poly infra → sources → filters →
Wren Synth surface → Sy-5 expressiveness → Sy-6 spatial: 6a stereo spread, 6b
live re-pan/re-pitch).
**Status:** Approved — ready for implementation plan

## Goal

A feed-forward dynamics processor on the established effect pipeline. `Comp.new`
is a full compressor (threshold, ratio, attack, release, knee, makeup, detector);
`Comp.limit` is a limiter preset (high ratio, fast attack, peak detect, hard
knee). Mono-in / mono-out, `out_width == 1`, built on the existing `Drive`/`Eq`
effect template.

## Background

The effect layer is already fully wired end-to-end (3 reverbs, chorus, flanger,
delay, drive, EQ, pan, all filters — each with a kernel, a graph `Kind`, a Wren
factory in both registration tables, and a prelude class). There is **no
dynamics processor** — no compressor, limiter, gate, or expander kernel exists.
Dynamics is the most conspicuously missing effect for an instrument (glue,
loudness, punch, peak control).

The pipeline to add an effect is established: a simple mono→mono effect follows
`Drive` (`drive.rs:43` kernel → `Kind::Drive` at `node.rs:72`, process arm
`node.rs:734`, param scalars via `set_param` → `node_drive_impl`
`bindings_audio.rs:806` → both tables → `class Drive` `prelude.wren:711`). A
compressor is exactly this shape: one signal input, several scalar params, mono
out.

## Scope (Fx-1)

A single `Comp` kernel and `Kind::Comp` node exposing a feed-forward peak/RMS
compressor with a soft knee, exposed via two Wren factories (`Comp.new` full,
`Comp.limit` preset) plus live setters.

### Explicitly out of scope / deferred

- **Gate / expander** (downward expansion below threshold — a different transfer
  curve / kernel mode). A later Fx sub-project.
- **Lookahead** (a delayed signal path so the gain envelope anticipates
  transients) — deferred; v1 is zero-latency feed-forward.
- **Stereo-linked / stereo-in detection** — like every existing effect, `Comp`
  is mono-in (reads port 0). Stereo-in effects (so effects preserve a 6a stereo
  synth image) are a separate architectural item, deferred.
- **Sidechain input, gain-reduction metering, program-dependent auto-release.**
- The rest of the effects backlog: bitcrush/decimate, phaser, tremolo/auto-pan,
  mid-side/width, true stereo-in effects.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP paths. `libm` is already a
  `deluge-dsp-kernels` dependency (dB↔linear via `libm::powf`/`log10f`/`sqrtf`).
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** purely additive — a new kernel, a new `Kind`, a new Wren
  class. No existing effect, node, or test changes behavior. Signals that never
  use `Comp` render byte-identically.
- **Two registration tables:** the `node_comp_impl` foreign factory registers in
  BOTH `install_methods` (`bindings_audio.rs`) AND the static `METHODS` table
  (`bindings.rs`), plus a prelude `class Comp`. (Missing either table = unbound
  method at runtime.)
- **Test invocation (per-crate, never `--workspace`; both configs; `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## Architecture

### §1 — The `Comp` kernel (`crates/deluge-dsp-kernels/src/dynamics.rs`, new file)

Scalar, per-sample, stateful/recursive (like the filter kernels). Signal chain
per sample `x`:

```rust
pub enum Detector { Peak, Rms }

pub struct Comp {
    // params (linear/seconds as noted)
    threshold_db: f32,   // dB
    ratio: f32,          // n:1 (>= 1)
    attack_s: f32,       // seconds
    release_s: f32,      // seconds
    knee_db: f32,        // dB width, 0 = hard knee
    makeup_db: f32,      // dB
    detector: Detector,
    // state
    rms_sq: f32,         // smoothed x^2 for the RMS detector
    gr_db: f32,          // current (smoothed) gain reduction in dB, >= 0
}
```

`process(&mut self, input: &[f32], dt: f32, out: &mut [f32])` — for each sample:

1. **Detector level (dB).**
   - `Peak`: `level = |x|`.
   - `Rms`: `rms_sq += (x*x - rms_sq) * rms_coeff(dt)`; `level = sqrt(rms_sq)`.
   - `level_db = 20*log10(max(level, EPS))` (EPS ~1e-9 avoids log(0)/−inf).
2. **Gain computer (target GR in dB), soft knee `knee_db`:**
   - `over = level_db - threshold_db`.
   - hard: `target_gr = if over <= 0 { 0 } else { over * (1 - 1/ratio) }`.
   - soft knee (when `knee_db > 0`): within `[-knee/2, +knee/2]` around threshold,
     quadratic interpolation — `if over <= -knee/2 { 0 }` / `if over >= knee/2 {
     over*(1-1/ratio) }` / else `((1-1/ratio) * (over + knee/2)^2) / (2*knee)`.
   - `target_gr >= 0` (attenuation only).
3. **Ballistics:** one-pole toward `target_gr`, attack coeff if `target_gr >
   gr_db` (GR increasing), release coeff otherwise. `coeff(t_s) = 1 -
   exp(-dt / max(t_s, dt))` (a per-sample smoothing factor; `exp` via `libm`).
   `gr_db += (target_gr - gr_db) * coeff`.
4. **Output:** `gain = 10^((-gr_db + makeup_db) / 20)`; `out[i] = x * gain`.

Setters on `Comp` for each param (`set_threshold`, `set_ratio`, …,
`set_detector`); `new(...)` with the full set. Clamp `ratio >= 1.0`,
`knee_db >= 0.0`, times `>= 0.0` at the setter boundary; no panics.

### §2 — Graph node (`crates/deluge-audio-graph/src/node.rs`)

`Kind::Comp` — `out_width == 1`, one signal input (port 0 = audio),
`State::Comp(Comp)`. `set_param` maps a scalar index to a `Comp` setter:

| param | field |
|-------|-------|
| 0 | threshold_db |
| 1 | ratio |
| 2 | attack_s |
| 3 | release_s |
| 4 | knee_db |
| 5 | makeup_db |
| 6 | detector (0.0 = Peak, else Rms) |

Process arm mirrors `Drive` (`node.rs:734`): `if let State::Comp(c) = &mut
self.state { c.process(ins[0]…, dt, out); }` — match the exact `In`/`out`
signature the neighboring width-1 processor arms use.

### §3 — Wren surface (`bindings_audio.rs`, `bindings.rs`, `prelude.wren`)

- **`node_comp_impl`** foreign factory (mono, no pool buffer — the `Drive`
  template): read the input + the constructor params, `audio::new_node(Kind::Comp)`,
  `set_param` each, return the node id. Paired `extern "C"` shim. Register
  `Node.comp_(...)` (or the file's factory-foreign convention) in BOTH tables.
- Setters: `comp_set_*` foreign(s) writing the relevant `set_param` index (mirror
  how `Drive`/`Eq` setters are wired), in both tables.
- **prelude `class Comp`:**
  - `Comp.new(input, threshold, ratio, attack, release)` — defaults
    `knee ≈ 6` dB, `makeup = 0`, detector = **RMS** (musical default).
  - `Comp.limit(input, threshold)` — `ratio = 20`, `attack ≈ 0.001` s,
    `release ≈ 0.1` s, `knee = 0`, detector = **Peak** (brickwall-ish).
  - instance setters: `threshold=`, `ratio=`, `attack=`, `release=`, `knee=`,
    `makeup=`, `detector=` (peak/rms). `detector=` accepts a small sentinel (e.g.
    `Comp.peak` / `Comp.rms` static returning 0/1, or a string→number — match the
    file's convention for enum-ish params like `Drive` shapes / `Eq` types).

### §4 — Composition note

`Comp` is **mono-in** (reads port 0), consistent with every existing effect. On a
stereo (6a `width > 0`) `synth.out`, it processes the left port only. Place
dynamics before the stereo spread, or on a mono chain, until the deferred
stereo-in-effects work lands. Documented, not solved here.

## Data Flow

```
Comp.new(src, thr, ratio, atk, rel) → Node.comp_(src, thr, ratio, atk, rel, knee=6, makeup=0, det=RMS)
  → Kind::Comp node (out_width 1), ins[0] = src audio
per sample x:
  level = Peak(|x|) | Rms(sqrt(smoothed x^2)) → level_db
  target_gr = gainComputer(level_db, threshold, ratio, knee)   [dB, >= 0]
  gr_db += (target_gr - gr_db) * coeff(attack if rising else release)
  out = x * 10^((-gr_db + makeup)/20)
→ Out.patch(comp)  (mono → dual-mono via existing width-1 center bus write)
```

## Error Handling

Setter-boundary clamps (`ratio >= 1`, `knee/times >= 0`); `log10` guarded by an
EPS floor (no −inf); `gain` is finite (bounded params → bounded exponent). No
heap, no panics, `no_std`. A `Comp` with `ratio == 1` (or level always below
threshold) is unity-gain × makeup — a transparent pass-through.

## Testing

Both feature configs, per-crate. Oracle-driven where possible.

1. **Below threshold = pass-through** (kernel): constant input below threshold,
   makeup 0 → output ≈ input (unity), any detector.
2. **Static curve steady-state** (kernel): constant input above threshold, run
   past settling → gain reduction converges to the textbook static value
   `GR = (in_db − threshold_db) · (1 − 1/ratio)`. E.g. threshold −20 dB,
   ratio 4:1, input −8 dB → `GR ≈ 9` dB (output ≈ −17 dB), within tolerance.
3. **Limiter clamp** (kernel): ratio 20, fast attack, peak detector, a loud input
   (e.g. 0 dB, threshold −10) → output settles near threshold (≈ −10 dB),
   well below the input.
4. **Soft knee** (kernel): input exactly at threshold with `knee > 0` → GR is
   partial (strictly between 0 and the above-knee full value), i.e. the knee
   softens the onset; with `knee == 0` the same input gives ~0 GR (hard corner).
5. **Ballistics direction** (kernel): after a step UP in input, `gr_db` increases
   over successive samples (attack); after a step DOWN, it decreases (release);
   both settle. (Coarse — direction + settling, not exact time.)
6. **Graph node** (node.rs): `Kind::Comp` `out_width == 1`, `set_param(0..6)`
   drives the corresponding `Comp` field; process arm reduces a loud input.
7. **Wren e2e** (`tests/audio_bindings.rs`): `Out.patch(Comp.new(Osc.saw(110),
   -20, 4, 0.005, 0.1))` renders finite/bounded/non-silent; a loud source through
   `Comp.limit(src, -10)` renders bounded and quieter (lower peak) than the dry
   source; `Comp` with `ratio = 1` (or a quiet source) is ~transparent; existing
   synths/effects unchanged.

## Success Criteria

- `Comp.new(src, threshold, ratio, attack, release)` compresses a signal to the
  textbook static curve at steady state, with a soft knee and RMS or peak
  detection; `Comp.limit(src, threshold)` brickwall-limits peaks near threshold.
- Live setters (`.threshold=`, `.ratio=`, …, `.detector=`) update the running
  compressor.
- Purely additive: everything that doesn't use `Comp` is byte-unchanged.
- Both feature configs green, per-crate.
