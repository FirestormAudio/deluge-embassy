# Poly FM Primitives — Design Spec

**Date:** 2026-07-12
**Status:** Approved (design), pending implementation plan
**Sub-project:** Osc suite — FM operators (the last Osc-suite north-star gap)

## Problem & framing

FM (frequency/phase modulation) synthesis is the one remaining gap in the Osc
suite. But an audit found that **mono 2-operator FM already works today with
zero new engine code**: the mono `Osc` kernel (`crates/deluge-dsp-kernels/src/osc.rs`)
has a dedicated per-sample phase-modulation input `pmod: In` (port 1, distinct
from `freq` on port 0) plus DX7-exact self-`feedback` (`feedback*0.5*(last+last2)`
added to phase). A script writes `carrier.pm = modulator * index` and gets real
sidebands (tested: `pm_produces_sidebands`, `fm_and_feedback_stay_bounded`).

What is genuinely missing is **poly FM**: the poly oscillators used inside
`Synth.new { }` have no phase-mod input and no self-feedback state, so the FM
patching idiom is mono/top-level only, not per-voice. This sub-project closes
that gap for the two poly oscillators the user selected — `PolyOsc` (the four
basic waveforms) and `PolyWt` (wavetable) — by giving them the same per-voice
`pm` + `feedback` the mono `Osc` already has. FM topologies then emerge from
patching poly oscillators; there is **no dedicated multi-operator FM instrument
node** (deliberately — the library favors composable primitives).

Deferred (out of scope): `PolySync` pm/feedback; a multi-operator algorithm
layer (fixed DX-style carrier/modulator matrices); mono `WtOsc` pm (a
pre-existing mono gap, orthogonal to this poly work).

## Grounding — current state

- **Mono `Osc`** (osc.rs:209) `{ phase: f32, last: f32, last2: f32, feedback: f32 }`;
  `process(&mut self, wave: Wave, freq: In, pmod: In, width: In, dt: f32, out: &mut [f32])`.
  Per sample: `ph = phase + pmod.at(i) + feedback*0.5*(last+last2)`, wrap, sample,
  then `last2=last; last=y; phase += freq.at(i)*dt`. `set_feedback` clamps [-1,1].
  **This is the reference model.**
- **`PolyOsc`** (poly.rs:83) `{ phase: [f32; VOICES], shape: Wave }`;
  `process(&mut self, pitch: &[f32], width: &[f32], dt: f32, out: &mut [f32])`.
  Voice-interleaved (`tile[i*VOICES+v]`), scalar oracle + `#[cfg(feature="simd")]`
  `f32x8` fast path. `poly_in_count == 2` (pitch=port0, width=port1). **No pm, no
  feedback today.**
- **`PolyWt`** (poly.rs) — pooled wavetable poly source, `poly_in_count == 1`
  (pitch=port0), per-voice phase. Wraps `[WtOsc; VOICES]`. **`WtOsc` ALREADY has a
  `pmod: In`** (`WtOsc::process(mips, freq, pmod, dt, out)`, phase read `ph =
  phase + pmod.at(i)`), and the node arm already threads it — but from the **mono**
  input `ins[1]` (shared across voices), NOT a per-voice poly edge. **`WtOsc` has
  NO feedback** (`WtOsc { phase: f32 }` only) — unlike mono `Osc`.

**Scope decision (from a code-reality discovery during planning):** PolyWt gets
FULL parity with PolyOsc — per-voice `pm` AND `feedback`. Two consequences:
1. **`pm`** is a *conversion*, not an addition: PolyWt's existing MONO pmod
   (`ins[1]`) becomes a per-voice **poly edge**, because the modulator inside a
   Synth is itself poly (VOICES-wide) and a mono port would misread an interleaved
   tile.
2. **`feedback`** is added to the shared **`WtOsc`** kernel (`last`/`last2` +
   scalar `feedback`, mirroring mono `Osc`). Since `WtOsc` also backs the **mono
   `Wavetable`** oscillator, mono `Wavetable` gains a `feedback=` too — an accepted
   bonus, and the mono path must be tested for non-regression.
- **Poly edges** are voice-interleaved `VOICES*BLOCK` tiles, de-interleaved per
  lane in `node.rs::poly_process`. The modulator inside a Synth is itself a poly
  oscillator, so its output is a VOICES-wide tile — `pm` must be a **poly edge**,
  threaded exactly like `pitch`. VOICES == 8.
- **Wren surface already exists**: `foreign pm=(v)` and `foreign feedback=(v)` on
  `class Node` (prelude.wren), routing via `node_set_pm_impl`/`node_set_feedback`.
  Today these target the mono `Osc` ports/param; the poly nodes have no such
  port/param, so inside a Synth they're currently inert/absent.

## Design

### Kernel changes (scalar path is the oracle; `f32x8` fast path must match)

Mirror the mono `Osc` model onto both poly kernels.

- **`PolyOsc`** gains:
  - a per-voice phase-mod input — `process` signature becomes
    `process(&mut self, pitch: &[f32], width: &[f32], pm: &[f32], dt: f32, out: &mut [f32])`
    where `pm` is voice-interleaved (`pm[i*VOICES+v]`), each lane's value added to
    that lane's phase before wrapping (cycles), exactly like mono `pmod.at(i)`.
  - per-voice self-feedback state `last: [f32; VOICES]`, `last2: [f32; VOICES]`
    and a scalar `feedback: f32` (same depth for all voices, like mono). Per lane:
    `ph = phase[v] + pm[..] + feedback*0.5*(last[v]+last2[v])`, wrap, sample →
    `y`, then `last2[v]=last[v]; last[v]=y`. `set_feedback(f)` clamps [-1,1].
  - `f32x8` path: load `pm`, `last`, `last2` as `f32x8`, `fb = splat(feedback)*0.5*(last+last2)`,
    add to the phase vector; update `last2=last; last=y_vec` per block sample.
- **`WtOsc`** (`wavetable.rs`, SHARED with mono `Wavetable`): add `last`/`last2`
  state + scalar `feedback` + `set_feedback` (clamp [-1,1]), mirroring mono `Osc`.
  The phase read becomes `ph = phase + pmod.at(i) + feedback*0.5*(last+last2)`, and
  `last2=last; last=y` each sample. `feedback == 0` (default) → bit-identical to
  today (mono `Wavetable` non-regression). The `pmod` input is unchanged.
- **`PolyWt`** (`poly.rs`): already forwards `pmod` per voice via `[WtOsc; VOICES]`
  (`process_voice`/`process_voice_morph` take `pmod: In`) — the kernel just gains a
  `set_feedback(f)` that fans to all 8 `WtOsc`s. Making PolyWt's `pm` **per-voice**
  is a GRAPH change (convert the mono `ins[1]` pmod source into a poly edge), not a
  kernel signature change — see below.

### Graph changes (`crates/deluge-audio-graph/src/node.rs`)

- `poly_in_count`: `PolyOsc` 2 → **3** (pitch=0, width=1, **pm=2**); `PolyWt` 1 →
  **2** (pitch=0, **pm=1**). Update the existing assertions
  (`poly_in_count(PolyOsc)==2` at node.rs:1839, and any PolyWt count assertion).
- `poly_process` arms: **`PolyOsc`** de-interleaves the new `pm` poly edge per lane
  into a `MAX_BLOCK` scratch column (like the existing `pitch` de-interleave) and
  passes it to the extended `process`. **`PolyWt`** already calls `process_voice(v,
  …, pmod, …)` with `ins[1]` as the mono pmod; change that to de-interleave the new
  `pm` poly edge (`poly_in[1]`) into a per-lane column and pass `In::A(&pmcol)` as
  the pmod. When the `pm` edge is **unconnected**, it must resolve to a per-voice
  **zero** tile (so an un-patched poly osc is byte-identical to today — see
  Non-breaking).
- `set_param` feedback arms: `PolyOsc` and `PolyWt` `param == <feedback slot>` →
  `set_feedback(value)`; the mono `Wavetable`'s node arm (`State::Wt`/`WtOsc`) also
  gets a `param == <feedback slot> => set_feedback` arm now that `WtOsc` has it. The
  slot must align with what the `feedback=` Wren setter emits per node kind (see
  Wren surface — the setter is poly/kind-aware). Mono `Osc` today uses `param 0`;
  `PolyOsc`'s `param 0` may be `set_shape`, so the poly feedback slot is chosen to
  avoid collision and the setter emits it — resolved in the plan.
- `State::PolyOsc(PolyOsc)` / `State::PolyWt(PolyWt)` construction unchanged
  (kernel `new()` initializes the new state to zero).

### Wren surface (no new API — but poly-aware routing)

Reuse the existing `pm=` / `feedback=` Node setter METHOD NAMES — no new prelude
methods, no new factories, identical script API. BUT the setter *implementations*
must become **poly-aware**, because the mono and poly nodes use different
port/param indices for these:
- **`pm`**: mono `Osc` `pmod` is input **port 1**; on `PolyOsc` it's poly edge
  **port 2** (pitch=0, width=1, pm=2) and on `PolyWt` **port 1** (pitch=0, pm=1).
- **`feedback`**: mono `Osc` uses `set_param(id, 0, v)`; on the poly kernels
  param 0 may already be `set_shape` (PolyOsc), so the poly feedback param slot
  must be chosen NOT to collide and the setter must emit that slot for poly nodes.

The `NodeObj` returned by every factory carries a `poly: u8` flag (see
`return_node_ex`), so `node_set_pm_impl` / `node_set_feedback` branch on it:
mono → the mono port/param; poly → the poly node's pm port / feedback slot. The
exact indices are resolved in the plan against the current `node_set_pm_impl` /
`node_set_feedback` and the poly `set_param` arm. No `Kind` disambiguation beyond
mono-vs-poly is needed (PolyOsc pm=port2, PolyWt pm=port1 differ, so the setter
may additionally need the node width or a poly-osc marker — resolve in the plan;
worst case a distinct internal setter selector per poly kind, still behind the
same Wren `pm=`/`feedback=` names). Inside a Synth:

```wren
Synth.new { |p|
  var mod = Osc.sine(p * 2)   // poly modulator (VOICES-wide tile)
  var car = Osc.sine(p)       // poly carrier
  car.pm = mod * 3            // per-voice phase modulation (index 3)
  car.feedback = 0.4          // optional per-voice self-FM
  car
}
```

(`mod * 3` is already a poly `Mul` tile — the Sy voice layer threads math over
VOICES-wide signals.)

## Non-breaking guarantee

A poly oscillator with **no `pm` patched** and `feedback == 0` must be
byte-identical to today, in BOTH the Cmd stream and the render:
- Kernel default `feedback = 0.0` and `last/last2 = 0` → the feedback term is
  exactly `0`, so `ph` is unchanged.
- An unconnected `pm` edge resolves to a zero tile → `pm[..] == 0`, phase
  unchanged.
- No existing factory or prelude call emits a `pm`/`feedback` for a plain
  `Osc.sine(p)` — so the Cmd stream for existing poly patches is unchanged.

Mono `Osc`/`SyncOsc`/`PolySync` are untouched. All existing poly-synth and
oscillator tests stay green in BOTH feature configs.

## Testing

- **Kernel oracle tests** (`poly.rs`): poly `pm` produces per-voice sidebands at
  `fc ± fm` (mirror mono `pm_produces_sidebands`, applied to a single lane and
  cross-checked against the mono `Osc` output for that lane); poly `feedback`
  stays bounded (mirror `fm_and_feedback_stay_bounded`); a lane with `pm==0` &
  `feedback==0` is bit-identical to the pre-change `PolyOsc` output. `PolyWt` gets
  the analogous pm-offset + feedback tests. **Scalar == `f32x8`** for every case.
- **Graph tests** (`node.rs`): `poly_in_count` updated (PolyOsc 3, PolyWt 2);
  `poly_process` reads the pm edge into the right lane; feedback `set_param` arm
  reaches `set_feedback`.
- **e2e** (`tests/audio_bindings.rs`): a poly FM patch (`car.pm = mod * index`
  inside `Synth.new{}`) renders finite/bounded/non-silent and polyphonic (two
  notes sound); a `PolyWt`-carrier FM patch renders. The **non-breaking** proof is
  two-part and does NOT require cross-version comparison: (a) the kernel oracle
  test asserting a `pm==0 && feedback==0` lane is bit-identical to the
  pre-change `PolyOsc`/`PolyWt` math (equivalently, identical to the mono `Osc`
  with `pmod==0`), and (b) every pre-existing poly-synth render test — which never
  patches pm/feedback — staying green unchanged. Those two together establish that
  an un-patched poly oscillator is unaffected.

## Constraints (global)

- `no_std`, no heap, no panic on any input. VOICES == 8 (compile-time).
- Scalar path is the correctness oracle; `#[cfg(feature="simd")]` `f32x8` fast
  path must match it; tests pass in BOTH configs (default + `--features simd`
  for kernels, `--features deluge-dsp-kernels/simd` for graph/wren-core).
- MIT/Apache-2.0 only.
- Both Wren registration tables + prelude — but this sub-project adds **no new
  Wren methods** (reuses existing `pm=`/`feedback=`), so only routing changes.
- `pm`-absent / `feedback==0` is byte-identical in BOTH the Cmd stream and render.
- New binding reads (if any) use the `checked_*` slotapi helpers — but this
  sub-project reuses existing setters, so likely no new slot reads.

## Out of scope / deferred

- `PolySync` pm + feedback (full-parity option not chosen).
- Multi-operator DX-style algorithm routing (fixed carrier/modulator matrices,
  N>2 operators, per-op ratio/level tables) — a possible follow-on built on
  these primitives.
- Mono `WtOsc`/`Wavetable` PER-VOICE pm is N/A (mono); the mono `Wavetable`
  `pmod` input already exists. Mono `Wavetable` `feedback=` is now IN scope as a
  side-effect of adding feedback to the shared `WtOsc` kernel (accepted bonus).
- True linear FM (freq-domain) as distinct from phase modulation — PM is the
  DX-style, already-modeled approach; `freq=` audio-rate patching remains
  available for the mono case.

## Likely task decomposition (for writing-plans)

1. `PolyOsc` kernel: add `pm` param + `last/last2/feedback` state + `set_feedback`,
   both scalar & `f32x8` paths; oracle tests (sidebands, bounded, pm=0 identity,
   scalar==simd).
2. `WtOsc` kernel: add `last/last2/feedback` + `set_feedback` (SHARED — mono
   `Wavetable` gains feedback), `feedback==0` bit-identity; `PolyWt::set_feedback`
   fans to all 8 voices; tests (mono `WtOsc` feedback bounded + feedback=0 identity,
   both paths).
3. Graph wiring: `poly_in_count` bumps (PolyOsc 2→3, PolyWt 1→2) + assertions;
   `poly_process` pm de-interleave for `PolyOsc` (new edge) and `PolyWt` (mono
   `ins[1]` → poly edge), zero when unconnected; `set_param` feedback arms for
   `PolyOsc`, `PolyWt`, AND mono `Wavetable`; node tests.
4. Wren `pm=`/`feedback=` poly/kind-aware routing (branch on `NodeObj.poly` /
   width to emit the correct port/slot per node kind); binding tests.
5. e2e: poly sine-FM + poly wavetable-FM render non-silent/polyphonic; mono
   `Wavetable.feedback=` renders bounded; no-pm/feedback=0 non-breaking baseline.
