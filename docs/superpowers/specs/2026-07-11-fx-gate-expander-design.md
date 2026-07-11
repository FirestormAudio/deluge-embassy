# Fx-1b: Dynamics — Gate / Expander — Design Spec

**Date:** 2026-07-11
**Suite:** Fx (audio effects), sub-project Fx-1b — the second dynamics processor,
after Fx-1 (compressor/limiter). See [[fx-suite-effects]].
**Status:** Approved — ready for implementation plan

## Goal

A downward expander / noise gate — attenuate signal *below* threshold (the
inverse of the Fx-1 compressor, which attenuates *above* threshold). Exposed as
two Wren classes over one kernel: `Expander.new(input, threshold, ratio, attack,
release)` (gentle downward expansion) and `NoiseGate.new(input, threshold, attack,
release, hold)` (a hard noise gate with a hold stage). Mono-in/mono-out,
`out_width == 1`, on the same effect template as Fx-1 `Comp`.

## Background

Fx-1 added a `Comp` compressor to `crates/deluge-dsp-kernels/src/dynamics.rs` with
a reusable scaffolding: `enum Detector { Peak, Rms }`, `lin_to_db`/`db_to_lin`
(libm), and `one_pole_coeff(time_s, dt)`. A gate/expander is the same *shape*
(detector → gain computer → ballistics → output) but:
- The **gain computer** applies gain reduction *below* threshold, not above
  (downward expansion), capped by a **range** floor.
- The **ballistics attack/release semantics invert**: *attack* = how fast the
  gate **opens** (GR falls to 0 as the signal rises above threshold); *release* =
  how fast it **closes** (GR rises as the signal drops below). So the one-pole
  picks the attack coefficient when GR is *decreasing* — the opposite of `Comp`.
- A **hold** stage keeps the gate open for `hold` seconds after the signal drops
  below threshold, before releasing — preventing chatter on decaying/fluctuating
  signals. This needs a per-sample hold countdown.

## Scope (Fx-1b)

A single `Gate` kernel (downward expansion + range floor + hold + inverted
ballistics), a `Kind::Gate` node, and two Wren classes (`Expander`, `NoiseGate`) over
it with live setters.

### Explicitly out of scope / deferred

- **Lookahead, sidechain/external-key input, GR metering** — deferred (as for
  Fx-1).
- **Stereo-linked / stereo-in detection** — `Gate` is mono-in (port 0), like every
  effect. The stereo-in-effects architectural item stays deferred.
- **Hysteresis (separate open/close thresholds)** — the hold stage covers the
  main anti-chatter need; dual-threshold hysteresis is a later refinement.
- **Duck/upward expansion, program-dependent release.**

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP paths. `libm` for all
  transcendentals (NO `std`-only float methods — the crate is cross-compiled
  `no_std` for the device). Bounded loops.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** purely additive — a new `Gate` struct in `dynamics.rs`, a new
  `Kind::Gate`, new Wren classes. No existing kernel/node/effect/test changes.
  `Comp` (Fx-1) is untouched.
- **Two registration tables:** `node_gate_impl` factory + the 7 setters register in
  BOTH `install_methods` (`bindings_audio.rs`) AND the static `METHODS` table
  (`bindings.rs`), with IDENTICAL selector strings, plus prelude `foreign` decls.
  Both `Expander` and `Gate` classes carry the `polyMode_` effect-guard.
- **Test invocation (per-crate, never `--workspace`; both configs; `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## Architecture

### §1 — The `Gate` kernel (`crates/deluge-dsp-kernels/src/dynamics.rs`, ADD to the file)

Reuses the existing `Detector`, `lin_to_db`, `db_to_lin`, `one_pole_coeff`.

```rust
pub struct Gate {
    threshold_db: f32,
    ratio: f32,      // expansion ratio, >= 1
    attack_s: f32,   // open time
    release_s: f32,  // close time
    hold_s: f32,     // hold-open time after dropping below threshold
    range_db: f32,   // max attenuation depth (dB, >= 0); e.g. 80 = near-mute gate
    detector: Detector,
    // state
    rms_sq: f32,
    gr_db: f32,        // current gain reduction (dB, >= 0)
    hold_ctr: i32,     // samples remaining in the hold-open window
}
```

`process(&mut self, input: In, dt: f32, out: &mut [f32])` — per sample `x`:

1. **Detector level (dB)** — identical to `Comp`: Peak `|x|`, or RMS
   `sqrt(one-pole(x²))`, then `lin_to_db` (EPS-floored).
2. **Gain computer + hold state machine:**
   - `over = level_db - threshold_db`.
   - if `over >= 0` (**above threshold — open**): `target_gr = 0`; reset
     `hold_ctr = (hold_s / dt) as i32` (recomputed reset value; keep the gate
     open).
   - else if `hold_ctr > 0` (**holding open**): `target_gr = 0`;
     `hold_ctr -= 1`.
   - else (**below threshold, hold expired — closing / expanding**):
     `target_gr = min((-over) * (ratio - 1.0), range_db)`  (dB, >= 0; capped by
     the range floor).
3. **Ballistics (INVERTED vs Comp):** `c = if target_gr < self.gr_db { atk_c }
   else { rel_c }` — attack when GR is falling (opening), release when rising
   (closing). `gr_db += (target_gr - gr_db) * c`.
4. **Output:** `out[i] = x * db_to_lin(-gr_db)`. (No makeup gain for a gate.)

`new(threshold_db, ratio, attack_s, release_s, hold_s, range_db, detector)` +
per-param setters. Clamp `ratio >= 1`, times & `range_db >= 0`. `#[derive(Clone,
Copy)]` (node `State` requires Copy — like `Comp`). No panics, no heap.

> The `hold_ctr` reset uses the per-sample `dt`; `(hold_s / dt) as i32` is
> non-negative (both `>= 0`) and bounded (hold_s is a small time). `hold_s == 0`
> ⇒ reset to 0 ⇒ no hold (immediate close) — the Expander default.

### §2 — Graph node (`crates/deluge-audio-graph/src/node.rs`)

`Kind::Gate` — `out_width == 1`, one signal input (port 0), `State::Gate(Gate)`.
`set_param` mapping:

| param | field |
|-------|-------|
| 0 | threshold_db |
| 1 | ratio |
| 2 | attack_s |
| 3 | release_s |
| 4 | hold_s |
| 5 | range_db |
| 6 | detector (0.0 = Peak, else Rms) |

Constructor default: a gentle expander, e.g. `Gate::new(-40.0, 2.0, 0.001, 0.1,
0.0, 20.0, Detector::Peak)`. Process arm mirrors `Comp` (`node.rs`):
`if let State::Gate(g) = &mut self.state { g.process(ins[0], dt, outs.port(0)); }`.

### §3 — Wren surface (`bindings_audio.rs`, `bindings.rs`, `prelude.wren`)

- **`node_gate_impl`** factory (mono, no buffer — the `Comp`/`Drive` template):
  8 args (input + threshold, ratio, attack, release, hold, range, detector),
  `alloc_node_id`, `new_node(Kind::Gate, [input, Const, Const])`, `set_param`
  0–6, `return_node`. Paired `extern "C"` shim. Selector `gate_(_,_,_,_,_,_,_,_)`.
- **Setters:** `gateThreshold=`/`gateRatio=`/`gateAttack=`/`gateRelease=`/
  `gateHold=`/`gateRange=`/`gateDetector=` (indices 0–6), each `set_param`-index
  style like the `Comp` setters. Register the factory + 7 setters in BOTH tables.
- **prelude — two classes over the one factory, each with the `polyMode_` guard:**
  - `class Expander`:
    `Expander.new(input, threshold, ratio, attack, release)` →
    `Node.gate_(input, threshold, ratio, attack, release, 0, 20, 0)`
    (no hold, 20 dB range, Peak — a gentle downward expander).
  - `class NoiseGate` (named to avoid the pre-existing `foreign class Gate` for hardware gate jacks):
    `NoiseGate.new(input, threshold, attack, release, hold)` →
    `Node.gate_(input, threshold, 10, attack, release, hold, 80, 0)`
    (ratio 10, 80 dB range = near-mute, Peak — a hard noise gate; the user
    controls the timing that matters most for a gate).
  - Both return a `Node`, so the `gateX=` setters work on either.

## Data Flow

```
Expander.new(src, thr, ratio, atk, rel) → gate_(src, thr, ratio, atk, rel, 0, 20, 0)
NoiseGate.new(src, thr, atk, rel, hold)      → gate_(src, thr, 10, atk, rel, hold, 80, 0)
  → Kind::Gate node (out_width 1), ins[0] = src audio
per sample x:
  level = Peak(|x|) | Rms(...) → level_db;  over = level_db - threshold
  above thr (over>=0):  target_gr = 0;      hold_ctr = hold_s/dt   (open)
  holding (hold_ctr>0): target_gr = 0;      hold_ctr -= 1          (hold open)
  below, hold done:     target_gr = min((-over)(ratio-1), range)   (close/expand)
  gr_db += (target_gr - gr_db) * (attack if falling else release)  [INVERTED vs Comp]
  out = x * 10^(-gr_db/20)
→ Out.patch(gate)
```

## Error Handling

Setter-boundary clamps (`ratio>=1`, times/range `>=0`); `log10` EPS-floored;
`gr_db`/gain finite (bounded params); `hold_ctr` non-negative and bounded. No
heap, no panics, `no_std`. A `Gate` whose input never drops below threshold is
unity pass-through (GR stays 0).

## Testing

Both feature configs, per-crate. Oracle-driven.

1. **Above threshold = unity** (kernel): constant input above threshold → GR ≈ 0,
   output ≈ input, any detector.
2. **Downward expansion steady-state** (kernel): constant input below threshold,
   `hold=0`, run past settling → `GR = min((thr − level)(ratio − 1), range)`. E.g.
   thr −40, ratio 2:1, level −50, range 20 → `GR = (10)(1) = 10` dB (below the 20
   dB floor), output ≈ −60 dB.
3. **Range floor** (kernel): input far below threshold (would exceed range) →
   GR clamps at `range_db` (e.g. −70 dB input, thr −40, ratio 4, range 20 → raw
   would be 90 dB but caps at 20).
4. **Hold keeps the gate open** (kernel): drive above threshold to settle open
   (GR ≈ 0), then drop below threshold with `hold` set; assert GR stays ≈ 0 for
   the hold window (sample count `hold/dt`), then rises toward the expansion
   target after the hold expires. (Directional — the hold state machine.)
5. **Ballistics direction** (kernel): after a rise above threshold, GR falls to 0
   (opens, attack); after dropping below (past hold), GR rises (closes, release).
6. **Graph node** (node.rs): `Kind::Gate` `out_width == 1`, `set_param(0..6)`
   drives the fields; a below-threshold input is attenuated.
7. **Wren e2e** (`tests/audio_bindings.rs`): `Out.patch(Expander.new(Osc.saw(110),
   -6, 2, 0.001, 0.1))` renders finite/bounded/non-silent (a saw near 0 dB is
   mostly above a −6 dB threshold → largely passes). **Gate closes below
   threshold — discriminated via a HIGH threshold above the source level:**
   `NoiseGate.new(Osc.saw(110), 6, 0.001, 0.05, 0.001)` sets threshold +6 dB, above the
   ~0 dB saw peak, so the gate never opens → heavily attenuated; assert its peak
   is much lower than the dry `Osc.saw(110)` peak (`peak(gated) < peak(dry)`, a
   strict discriminating inequality — no need to synthesize a quiet source). A
   `Gate.new` with a LOW threshold (e.g. −40, below the source) passes (stays
   open, renders non-silent). Existing synths/effects unchanged. Use the
   `run_and_render<N>` N-frame window (Fx-1) with enough frames (e.g. 128) for the
   ballistics/hold to act.

## Success Criteria

- `Expander.new(src, threshold, ratio, attack, release)` applies downward
  expansion below threshold to the static curve (capped by range); `NoiseGate.new(src,
  threshold, attack, release, hold)` gates out signal below threshold, holding
  open for `hold` seconds to avoid chatter.
- The hold stage demonstrably keeps the gate open across the hold window before
  releasing.
- Live setters (`.gateThreshold=`, …) update the running processor.
- Purely additive: everything that doesn't use `Gate` is byte-unchanged; `Comp`
  is untouched.
- Both feature configs green, per-crate.
