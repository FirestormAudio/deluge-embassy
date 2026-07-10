# Sy-2c: Poly Filters + Footgun Close — Design Spec

**Date:** 2026-07-10
**Suite:** Sy (synth/voice), sub-project Sy-2c
**Status:** Approved — ready for implementation plan

## Goal

Make the resonant analog-modeled voice filters `Moog.lp/lp2` and `Ms20.lp/hp`
work inside a Wren `Synth` builder (polyphonic, f32x8 across voices), and
guarantee that **no** free-standing Wren class ever silently builds a mono node
on a poly edge.

## Background

After Sy-2b, inside a `Synth` builder these work in poly mode: `Osc.sine/saw/square/tri`,
`+`, `Noise.new` (white), `.lpf` (PolySvf), `Env.ar` (PolyAr), `*` (PolyMul).
Still `Fiber.abort` in poly mode: the richer filters, `Osc.sync`, wavetable,
per-voice PWM, `Noise.pink/brown`.

Two problems motivate Sy-2c:

1. **No resonant character filters in a voice.** Only the SVF lowpass (`.lpf`)
   is available inside a Synth. The Moog ladder and MS-20 Sallen-Key are the
   two most-reached-for analog-modeled synth-voice filters and have no poly path.

2. **A silent-mono footgun (carried from Sy-4/Sy-2b).** The free-standing Wren
   filter classes (`Svf`, `Tb303`, `Moog`, `Ms20`, `Resonator`) and all 9 FX
   classes (`Pan`, `Delay`, `Chorus`, `Flanger`, `Room`, `Hall`, `Plate`,
   `Drive`, `EQ`) call their foreign factory **unguarded** — inside a `Synth`
   (poly mode) they build a **mono** node on a poly edge instead of aborting.
   The only poly-aware filter route today is the `.lpf()` *method*, which
   checks `Node.polyMode_ == 1` and swaps in `PolySvf`.

## Scope

Two deliverables in one coherent pass:

1. **Poly Moog + Ms20** through all three layers (kernel → graph → Wren), with an
   f32x8 fast path null-tested lane-for-lane against the scalar oracle, following
   the locked PolySvf recipe.
2. **Footgun close:** every free-standing class not poly-routed gets a leading
   `if (Node.polyMode_ == 1) Fiber.abort(...)`.

### Explicitly out of scope (deferred)

- **Tb303** poly — ~19 f32 of per-voice state (2 diode ladders + 6 fixed HPs +
  resonance HP). f32x8-across-voices would spill far past the Cortex-A9's 16
  Q-registers; scalar-loop poly would forfeit the cross-voice-ILP rationale that
  justifies `VOICES = 8`. Wants its own careful pass. **Aborts** in a Synth.
- **Modal (`Resonator`)** poly — a 16-mode resonator bank (32 f32 state), a
  physical-model/exciter primitive, not a subtractive voice filter; reads its
  controls per-block, not audio-rate. Belongs with a physical-modeling effort.
  **Aborts** in a Synth.
- **Osc.sync / wavetable / per-voice PWM / Noise.pink/brown** — oscillator/source
  breadth, a separate coherent chunk (candidate Sy-2d). Already abort today; stay
  aborting.
- **Per-voice cutoff/res** (key-tracking, per-voice filter mod) — cutoff/res stay
  *shared mono controls* in Sy-2c, same as PolySvf. Per-voice control is a Sy-5
  expressiveness concern.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP paths. ARM Cortex-A9
  (VFPv3 + NEON) is the target; host x86 is for tests only.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **`VOICES == 8`**, the crate const. f32x8 SIMD is across voices (SoA per-voice
  state). `const _: () = assert!(VOICES == 8)` guards any f32x8 code.
- **SIMD convention:** the scalar path is the correctness oracle. The
  `#[cfg(feature = "simd")]` f32x8 fast path is null-tested lane-for-lane against
  scalar to `≤ 1e-4`. Doc comments attach to the struct, never to a cfg'd const
  guard.
- **Poly control convention:** the leading poly edge is the audio input
  (`poly_in_count == 1`); cutoff/res are trailing *shared mono* controls,
  computed once per sample and splatted to all lanes (the PolySvf model).
- **Test invocation:** per-crate on the host, never `--workspace`. Both feature
  configs: `default` and `--features deluge-dsp-kernels/simd` (or
  `--features simd` for the kernels crate directly).

## Architecture

Three layers, each mirroring the existing PolySvf wiring exactly.

### Layer 1 — Kernels (`crates/deluge-dsp-kernels/src/`)

**Vectorized saturators** (added beside their scalar originals in `filter.rs`,
guarded by `#[cfg(feature = "simd")]`):

- `pade_tanh_x8(x: f32x8) -> f32x8` — the Padé rational approximation of `tanh`,
  the lane-parallel counterpart of the existing scalar `pade_tanh`. Used by both
  the Moog ladder feedback and the MS-20 output limiter.
- `ms20_clip_x8(x: f32x8) -> f32x8` — the MS-20 asymmetric diode clip, counterpart
  of scalar `ms20_clip`.

Each is a pure function of its scalar original and gets a direct null-test across
a swept input range (including the saturating tails) asserting `≤ 1e-4` per lane.

**Poly filter structs** (`poly.rs`), each dual-path exactly like `PolySvf`:

- **`PolyMoog<const POLES: usize>`** — SoA ladder state (per stage, `f32x8` in the
  SIMD build / `[Moog<POLES>; VOICES]` reusing the scalar kernel in the scalar
  build). The f32x8 inner loop runs the 2× Heun predictor/corrector sub-steps
  across lanes, calling `pade_tanh_x8` for the ladder nonlinearity. Cutoff/res are
  shared mono controls; coeffs computed once per sample and splatted. Both slopes
  (`POLES = 4` and `2`) supported — same kernel, const-generic. `drive` defaults
  to 1.0 (shared), matching the mono `moog_` factory (which has no drive arg).
  Scalar oracle: `Moog::process` / `DiodeLadder::process`.
- **`PolyMs20`** — SoA `ic1eq`/`ic2eq` + per-voice DC-blocker state. 2× oversampled
  inner loop calling `ms20_clip_x8` for the resonance feedback and `pade_tanh_x8`
  for the output limiter. Response (Lp/Hp) selected by an enum/int arg. `drive`
  defaults to 1.0 (shared). Scalar oracle: `Ms20::process`.

**Register-budget check** (why these two, not Tb303/Modal): PolyMoog`<4>` carries
4 × `f32x8` of ladder state = 8 Q-registers; PolyMs20 ~4 × `f32x8` = 8 Q-regs.
Both leave headroom under the A9's 16 Q-registers, so state stays register-resident
(no spill) — the precondition for f32x8-across-voices to actually win.

### Layer 2 — Graph node (`crates/deluge-audio-graph/src/node.rs`)

Parallel to `PolySvf`:

- `Kind::PolyMoog`, `Kind::PolyMs20` (alongside existing mono `MoogLp4/MoogLp2/
  Ms20Lp/Ms20Hp`).
- `State::PolyMoog(PolyMoog<…>)`, `State::PolyMs20(PolyMs20)` arms + constructors.
  Slope (4/2) and response (Lp/Hp) carried as node config, the same way mono
  `moog_kind`/`ms20_kind` selectors work.
- Predicate registration: `out_width => VOICES`, `is_poly => true`,
  `poly_in_count => 1` for both.
- `poly_process` match arms dispatching to the kernel `process`.

### Layer 3 — Wren (`crates/deluge-wren-core/`)

**New poly factories** (`bindings_audio.rs` + `bindings.rs` METHODS +
`register_audio`):

- `polymoog_(input, cutoff, res, poles)` → `node_polymoog_impl` (arity 4).
- `polyms20_(input, cutoff, res, resp)` → `node_polyms20_impl` (arity 4).

**Flip the classes** (the Sy-2b class-route pattern) in `wren/prelude.wren`:

```wren
class Moog {
  static lp(input, cutoff, res) {
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

**Guard everything else** with a leading abort in poly mode, matching the
existing idiom (`if (Node.polyMode_ == 1) Fiber.abort("...")`):

- `Svf` (all four factories — poly SVF is LP-only and already reachable via
  `.lpf(cutoff)`; the message points there).
- `Tb303`, `Resonator` — deferred filters; `"... not usable in a Synth yet (Sy-2c)"`.
- The 9 FX classes (`Pan`, `Delay`, `Chorus`, `Flanger`, `Room`, `Hall`, `Plate`,
  `Drive`, `EQ`) — these run *after* VoiceSum (mono/stereo post-voice), so the
  message says they belong after `.out`, not "yet."

## Data Flow

Inside a `Synth` builder, a Moog voice:

```
PolyCtrl(pitch) → PolyOsc(saw) → PolyMoog(cutoff, res) → PolyMul(·, PolyAr) → VoiceSum → out
```

`PolyMoog`/`PolyMs20` consume one poly audio edge (8 voice-interleaved lanes,
`tile[i*VOICES + v]`) and two shared mono controls (cutoff, res), producing one
poly edge of width `VOICES`.

## Error Handling

The only new failure mode is misuse — a non-poly class used inside a `Synth`
builder — handled by the `Fiber.abort` guards. No new runtime error paths in the
DSP: kernels are total functions with clamped inputs and outputs bounded by
construction (the ±8 kernel contract, enforced by the tanh/clip saturators).

## Testing

All tests run in both feature configs (`default` and
`--features deluge-dsp-kernels/simd`).

1. **Kernel null-tests** (`poly.rs`):
   - `pade_tanh_x8` and `ms20_clip_x8` vs their scalar originals across a swept
     input range including saturating tails, `≤ 1e-4` per lane.
   - `PolyMoog` (both slopes) and `PolyMs20` (both responses) rendered lane-for-lane
     against the scalar `Moog`/`Ms20` (voice 0) with shared cutoff/res, `≤ 1e-4`.
   - Boundedness/self-oscillation sanity: at high res (and drive) output stays
     within the ±8 kernel contract and does not diverge.

2. **Graph test** (`node.rs`): a full poly voice
   `PolyCtrl→PolyOsc→PolyMoog→PolyMul(·,PolyAr)→VoiceSum` renders non-silent,
   bounded output; same for `PolyMs20`.

3. **Wren end-to-end** (`tests/audio_bindings.rs`):
   - `Synth.new { |p| Moog.lp(Osc.saw(p), 800, 0.8) * Env.ar(a, r) }` renders
     bounded, non-silent sound; same for `Ms20.hp`.
   - **Abort tests** (footgun closed): `Tb303.lp`, `Resonator.new`, `Svf.lp`, and a
     representative FX (`Delay` or `Room`) each `Fiber.abort` when used inside a
     `Synth` builder.

## Success Criteria

- A playable poly Moog and poly MS-20 voice from Wren, e.g.
  `Synth.new { |p| Moog.lp(Osc.saw(p), 1200, 0.85) * Env.ar(0.01, 0.4) }`.
- f32x8 fast path bit-close (`≤ 1e-4`) to the scalar oracle for both filters,
  both slopes/responses, verified lane-for-lane.
- Mono paths (`Moog.lp` / `Ms20.hp` outside a Synth) unchanged.
- No free-standing class silently builds a mono node in poly mode — every
  non-poly-routed filter and FX class aborts with a clear message.
- Both feature configs green, per-crate.
