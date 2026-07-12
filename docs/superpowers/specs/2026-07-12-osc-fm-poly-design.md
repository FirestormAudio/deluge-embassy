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

### Engine change (`crates/deluge-audio-graph/src/engine.rs`) — REQUIRED

The engine's poly render path hardcodes **`poly_in: [Option<&[f32]>; 2]`** and a
`poly_scratch` of 2 buffers — i.e. today's max `poly_in_count` is **2**. PolyOsc
needs a 3rd poly edge (pitch/width/pm), so **widen `poly_in` and `poly_scratch`
from 2 to 3** (add the `if count > 2 { Some(poly_scratch[2]…) }` arm and bump the
scratch array). This is unavoidable for a 3-poly-edge oscillator and touches the
shared poly render path — every poly node's render must stay green.

### Port layout (locked) & graph changes (`node.rs`)

Setters route on **`audio::poly_mode()`** (true inside a `Synth.new{}` block), the
SAME mechanism `node_set_width_impl` already uses (`port = if poly_mode() {1}
else {2}`). So the poly-vs-mono split needs no `NodeObj` discriminator. `pm` is
unified at **poly port 2** across both poly oscillators; `position` moves to poly
port 1 for the poly wavetable (mirroring how `width` moves to poly port 1 for
PolyOsc).

- `poly_in_count`: `PolyOsc` 2 → **3** (pitch=0, width=1, **pm=2**); `PolyWt` 1 →
  **3** (pitch=0, **position=1**, **pm=2**). Update the assertions
  (`poly_in_count(PolyOsc)==2` at node.rs:1839, any PolyWt assertion).
- `poly_process` arms — the two kernels differ in shape:
  - **`PolyOsc::process`** consumes WHOLE voice-interleaved tiles (de-interleaves
    internally). `pm` is another full tile passed whole: bump the arm to
    `(…, Some(pitch), Some(width), Some(pm)) = (…, poly_in[0], poly_in[1],
    poly_in[2])`, call `o.process(pitch, width, pm, dt, out)`.
  - **`PolyWt`** loops voices calling `process_voice(v, …, pmod: In, …)` /
    `process_voice_morph(v, …, pmod, position, …)` with per-lane columns. Change
    the arm: de-interleave `poly_in[2]` (pm) into a `pmcol` and pass
    `In::A(&pmcol[..n])` as pmod (replacing the old mono `ins[1]`); for the MORPH
    branch, de-interleave `poly_in[1]` (position) into a `poscol` and pass
    `In::A(&poscol[..n])` as position (replacing the old mono `ins[2]`).
  - **Unconnected `pm`/`position` = automatic zero:** the engine broadcasts an
    all-zero tile for an unconnected poly edge (how PolyOsc's optional `width`
    works today). So an un-patched `pm` arrives as zero → `pm==0` → byte-identical.
    A poly wavetable with no `.position=` gets a zero position tile — matching the
    old unset-`ins[2]` behavior (confirm the old default was also 0).
- `set_param` feedback arms (slots locked to match the setter):
  - `PolyOsc`: its `match param` currently has `0 => set_shape`; ADD `1 =>
    o.set_feedback(value)` (poly feedback = **param 1**).
  - `PolyWt`: no `set_param` arm today — ADD `State::PolyWt(w) if param == 1 =>
    w.set_feedback(value)` (**param 1**).
  - mono `Wavetable`: its `State` arm (whatever backs `Kind::Wavetable`) gets
    `param == 0 => set_feedback(value)` (**param 0**, matching mono `Osc`). Confirm
    param 0 is free for the mono wavetable node (position/morph is an input port,
    not a param) — resolve in the plan.
- `State::PolyOsc(PolyOsc)` / `State::PolyWt(PolyWt)` construction unchanged
  (kernel `new()` initializes the new state to zero).

### Wren surface (no new API — `poly_mode()`-aware routing)

Reuse the existing `pm=` / `feedback=` (and now `position=`) Node setter METHOD
NAMES — no new prelude methods, no new factories, identical script API. The setter
*implementations* become **`poly_mode()`-aware**, EXACTLY mirroring the existing
`node_set_width_impl` (`let port = if audio::poly_mode() { 1 } else { 2 };`).
`poly_mode()` is true inside a `Synth.new{}` block (where the node is poly) and
false at top level (mono). Locked routing:
- `node_set_pm_impl`: `let port = if audio::poly_mode() { 2 } else { 1 };
  set_input(self_id(vm), port, arg_input(vm,1))`. (mono `Osc` pmod=port 1; poly=port 2)
- `node_set_feedback_impl`: `let param = if audio::poly_mode() { 1 } else { 0 };
  set_param(self_id(vm), param, get_f(vm,1))`. (mono feedback=param 0; poly=param 1)
- `node_set_position_impl`: `let port = if audio::poly_mode() { 1 } else { 2 };
  set_input(self_id(vm), port, arg_input(vm,1))`. (mono `Wavetable` position=port 2;
  poly `PolyWt` position=poly port 1 — moved to make room for the unified pm=port 2)

Mono paths (pm port 1 / feedback param 0 / position port 2) are unchanged →
existing mono `Osc.pm=`/`.feedback=` and mono `Wavetable.position=` behavior is
byte-identical. This is the identical pattern and limitation as the existing
`width=` setter (it assumes the setter is called while `poly_mode()` reflects the
target node — true for the Synth DSL, which calls these inside the block). Inside
a Synth:

```wren
Synth.new { |p|
  var mod = Osc.sine(p * 2)   // poly modulator (VOICES-wide tile)
  var car = Osc.sine(p)       // poly carrier
  car.pm = mod                // per-voice phase modulation (unity ~±1-cycle depth)
  car.feedback = 0.4          // optional per-voice self-FM (continuous depth)
  return car
}
```

> **SETTING FM INDEX (depth).** Use **`car.pm = mod.scale(3)`** for a constant
> modulation index, or **`car.pm = mod * Env.adsr(...)`** for a modulated /
> enveloped index (the expressive DX case). Both work: the kernel adds `pm`
> UNSCALED (`rp = p + pm + fb`), so scaling the modulator signal upstream sets the
> depth. `car.feedback = f` gives continuous self-FM depth; bare `car.pm = mod` is
> a fixed unity (~±1 cycle) index.
>
> Historical note: the design's originally-pitched `car.pm = mod * 3` (bare Num)
> ABORTS — a pre-existing Synth-DSL guard (`prelude.wren`, locked by
> `synth_error_cases_abort`) refuses `poly-audio-node * constant` to steer
> amplitude through `Env.ar`. The `.scale(k)` helper (added by the FM-index
> follow-up, main-merged 2026-07-12 — see `2026-07-12-fm-index-scale-design.md`)
> is the sanctioned path: it wraps the constant in a control node (`this *
> Node.ctrl_(k)`), which the guard permits and the engine broadcasts across
> voices. The guard itself is unchanged. (Multi-statement Synth block bodies also
> require an explicit `return`.)

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
