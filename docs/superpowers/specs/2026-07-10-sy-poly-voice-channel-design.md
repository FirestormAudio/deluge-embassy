# Sy-2: Poly voice channel — per-voice gate, envelope, filter — design & spec

The second **Sy (synth/voice)** sub-project. Sy-1 gave the graph poly *plumbing*
(`VOICES=8`, voice-interleaved tiles, `PolyCtrl`/`PolyOsc`/`VoiceSum`,
`poly_process` dispatch). Sy-2 turns that into a **complete, per-voice-gated
synth voice**: a pure **poly envelope** (`PolyAr`), a **poly filter**
(`PolySvf`), a **poly multiply** (`PolyMul`, the VCA), and **per-voice
gate/trigger**, so `PolyCtrl → PolyOsc → PolySvf → PolyMul(·, PolyAr) →
VoiceSum` renders 8 independently-gateable voices. Still no allocation (Sy-3) or
Wren surface (Sy-4).

> **Status:** design proposal. Depends on merged Sy-1
> (`deluge_dsp_kernels::poly`, `Kind::PolyCtrl/PolyOsc/VoiceSum`,
> `Node::{is_poly,poly_process}`, engine poly-edge resolution) and the merged
> scalar `Ar` (`env.rs`) + `Svf` (`filter.rs`). MIT/Apache. `no_std`, no-heap,
> device-first (Cortex-A9, VFPv3+NEON).

**Sy roadmap:** Sy-1 poly infra ✓ → **Sy-2 poly voice channel (this)** → Sy-3
allocation & note handling → Sy-4 Wren `Synth`/`Voices` surface + MIDI (the
single-name ergonomics: `Osc`/`Noise`/`Env` inside a `Synth` builder emit the
poly kinds) → Sy-2b poly breadth (remaining oscillators/filters) → Sy-5
expressiveness.

---

## 1. Goals & non-goals

**Goals**
- **Per-voice gate/trigger.** `Cmd::GateVoice { node, voice, on }` /
  `Cmd::TriggerVoice { node, voice }` → `Node::gate_voice(v, on)` /
  `trigger_voice(v)` → the addressed voice-lane of a `PolyAr`.
- **`PolyAr` — a pure poly envelope source.** 8 independent attack/release
  envelopes (mirrors the merged scalar `Ar`), per-voice gate state, shared
  `attack`/`release` mono controls, output = 8 lanes ∈ [0,1]. No audio input —
  it is a *source*, reusable to modulate anything (amp, cutoff, …). Reuses an
  extracted `Ar::tick`.
- **`PolySvf` — a poly state-variable lowpass.** Poly audio in (8 lanes) →
  8 filtered lanes; shared mono `cutoff`/`res`. SoA `ic1eq`/`ic2eq` state.
  Scalar path reuses the existing `Svf::tick` (oracle); `f32x8` fast path behind
  the `simd` feature, null-tested — the `PolyOsc` pattern.
- **`PolyMul` — poly × poly (the VCA / poly arithmetic).** Two poly inputs,
  `out[v] = a[v]·b[v]`. Element-wise; the VCA is `PolyMul(filter_out, env)`.
- **Engine: two poly inputs.** Generalize Sy-1's single poly edge to
  `Node::poly_in_count(kind) ∈ {0,1,2}` (the *first k ports* are poly); the
  engine resolves up to two voice-interleaved `poly_scratch` tiles;
  `poly_process` gains the mono control inputs and a `[Option<&[f32]>; 2]` poly
  array.
- **QA-proven:** a per-voice-gated full voice — gated voices sound, ungated
  voices are silent; the filter attenuates highs; the envelope ramps; `PolyMul`
  multiplies lanewise; `f32x8`/scalar `PolySvf` agree; the `Ar::tick` refactor
  leaves scalar `Ar` behavior unchanged; all bounded/finite/deterministic.

**Non-goals (deferred)**
- **`PolyAdd` / 3-poly-input nodes** (osc mixing, complex routing) — trivial
  follow-on once two-poly-input lands; not needed for one voice.
- **Per-voice cutoff (key-tracking), velocity scaling, ADSR, poly Noise/Sync/
  Wavetable/other filters** — Sy-2b breadth / Sy-5.
- **Voice allocation, note-on/off, `mtof`** — Sy-3.
- **Wren surface / single-name ergonomics** — Sy-4.

---

## 2. Per-voice gate/trigger + engine two-poly-input model

**Per-voice gate/trigger.** New commands `Cmd::GateVoice { node: NodeId, voice:
u8, on: bool }` and `Cmd::TriggerVoice { node: NodeId, voice: u8 }`; engine
`apply()` routes them to `Node::gate_voice(v, on)` / `Node::trigger_voice(v)`,
which address `State::PolyAr`'s `voices[v]`. The whole-node `Cmd::Gate` /
`Node::gate` (mono `Ar`/`Lfo`) are unchanged. (Sy-3's allocator will emit these
on note-on/off.)

**Two poly inputs.** Sy-1's boolean `has_poly_in` becomes
`Node::poly_in_count(kind) -> usize` (0/1/2). Convention: **the first `k` input
ports are poly edges** (each references a `VOICES`-wide source), ports `k..3`
are mono controls. Mapping:

| Kind | poly_in_count | port layout |
|------|---------------|-------------|
| `PolyCtrl` | 0 | (source, per-voice `set_param`) |
| `PolyAr` | 0 | 0=attack, 1=release (mono) |
| `PolyOsc` | 1 | 0=pitch (poly) |
| `PolySvf` | 1 | 0=audio (poly), 1=cutoff, 2=res (mono) |
| `VoiceSum` | 1 | 0=poly |
| `PolyMul` | 2 | 0=a, 1=b (poly) |

Engine `render_block`: resolve up to two voice-interleaved `poly_scratch` tiles
(ports `0..poly_in_count`) with the same row-copy-preserves-interleave discipline
as Sy-1; still copy-before-mutate (fill from the immutable arena read before the
mutable output borrow). Dispatch:

```
poly_process(&mut self, ins: &[In; MAX_INPUTS], poly_in: [Option<&[f32]>; 2], dt, out)
```

`ins` = the existing 3-row mono scratch (poly nodes read only their control
ports). `poly_in[j]` = `Some` for `j < poly_in_count`, else `None`. Sy-1's three
poly kinds ignore `ins`/`poly_in[1]` (two Sy-1 node-layer test call sites update
to the new signature). A dangling/short poly edge → silence (unchanged guard).

---

## 3. Kernel DSP (`deluge-dsp-kernels`)

### 3.1 `Ar::tick` extraction (`env.rs`, DRY, behavior-preserving)

Extract the per-sample body of `Ar::process` into:

```rust
impl Ar {
    /// Advance one sample; returns the new level. (attack/release in seconds.)
    pub fn tick(&mut self, attack: f32, release: f32, dt: f32) -> f32 {
        let atk = attack.max(0.0001);
        let rel = release.max(0.0001);
        // (identical Attack/Sustain/Release/Idle state machine as today)
        self.level
    }
}
```

`Ar::process` becomes `for (i, s) in out { *s = self.tick(attack.at(i),
release.at(i), dt) }`. No behavior change — the existing `env` tests must still
pass verbatim.

### 3.2 `PolyAr` (`poly.rs`) — pure poly envelope source

```rust
pub struct PolyAr { voices: [Ar; VOICES] }
impl PolyAr {
    pub fn new() -> PolyAr;
    pub fn gate_voice(&mut self, v: usize, on: bool);   // voices[v].gate(on)
    pub fn trigger_voice(&mut self, v: usize);          // voices[v].trigger()
    /// attack/release mono controls; writes a voice-interleaved env tile [0,1].
    pub fn process(&mut self, attack: In, release: In, dt: f32, out: &mut [f32]);
}
```

Per sample: `for v in 0..VOICES { out[i*VOICES+v] = self.voices[v].tick(attack.at(i),
release.at(i), dt) }`. Scalar (branchy state machine — cheap; SIMD would diverge
on per-voice stages). `no_std`, deterministic.

### 3.3 `PolySvf` (`poly.rs`) — poly SVF lowpass

Reuses the existing `crate::filter::{Svf, svf_coeffs, svf_k_from_res,
svf_tan_prewarp, SvfResp}` (all `pub(crate)`). Cutoff/res are **shared mono**, so
coefficients `(k, a1, a2, a3)` are computed **scalar, once per sample** (or once
per block on the const path via `libm::tanf`, matching scalar `Svf`) and applied
to all 8 voices — the transcendental stays scalar; only the recurrence
vectorizes.

```rust
pub struct PolySvf {
    // State representation is cfg-selected so the scalar path can reuse the
    // audited Svf::tick (whose ic1eq/ic2eq are private) while the SIMD path
    // keeps state register-resident as f32x8:
    #[cfg(not(feature = "simd"))] voices: [Svf; VOICES],
    #[cfg(feature = "simd")]      ic1: [f32; VOICES],
    #[cfg(feature = "simd")]      ic2: [f32; VOICES],
}
impl PolySvf {
    pub fn new() -> PolySvf;
    /// audio = voice-interleaved poly input; cutoff/res mono; LP output tile.
    pub fn process(&mut self, audio: &[f32], cutoff: In, res: In, dt: f32, out: &mut [f32]);
}
```

**Scalar path (`#[cfg(not(feature = "simd"))]`, the oracle):** holds
`voices: [Svf; VOICES]` and calls the existing public `Svf::tick(v0, k, a1, a2,
a3, SvfResp::Lp)` per voice — full reuse of the audited step, no field access.
Both paths compute the *same* scalar coeffs. Requirement: scalar path ==
`Svf::tick`; `f32x8` path == the same recurrence on `f32x8`; null-tested against
each other, plus a cross-check that a single-voice `PolySvf` matches a scalar
`Svf::process` on the same signal.

Per-sample coeffs (mirrors scalar `Svf`): `theta = (π·fc·dt).min(0.49π)`;
`g = libm::tanf(theta)` (const cutoff) or `svf_tan_prewarp(theta)` (audio-rate);
`k = svf_k_from_res(res)`; `(a1,a2,a3) = svf_coeffs(g,k)`. LP output = `v2`.

**`f32x8` path (`#[cfg(feature = "simd")]`):** splat scalar `(a1,a2,a3)`, run the
branchless TPT recurrence on `f32x8` lanes (`v3 = v0 - ic2; v1 = a1·ic1 + a2·v3;
v2 = ic2 + a2·ic1 + a3·v3; ic1 = 2·v1 - ic1; ic2 = 2·v2 - ic2; out = v2`) — one
`f32x8` step = the 8 voices. `const _: () = assert!(VOICES == 8)` guards it.

### 3.4 `PolyMul` (`poly.rs`) — poly × poly

```rust
/// out[i*VOICES+v] = a[i*VOICES+v] * b[i*VOICES+v]. Stateless.
pub fn poly_mul(a: &[f32], b: &[f32], out: &mut [f32]);
```

Element-wise over the two interleaved tiles; auto-vectorizes (no hand-SIMD). The
VCA and the seed of poly arithmetic.

---

## 4. Graph & engine integration (`deluge-audio-graph`)

- `enum Kind`: add `PolyAr`, `PolySvf`, `PolyMul`. `enum State`:
  `PolyAr(PolyAr)`, `PolySvf(PolySvf)`; `PolyMul` is `State::Stateless`.
- `Node::new`: construct each. `out_width` → `VOICES` for all three (poly out).
- `Node::poly_in_count(kind)`: per the §2 table (replaces `has_poly_in`;
  `is_poly` still true for every poly kind). Update the engine + any caller.
- `set_param`: all three are **paramless** (all controls are input ports, like
  `PolyOsc`) — `PolyAr` attack/release = ports 0/1, `PolySvf` cutoff/res =
  ports 1/2, `PolyMul` none.
- `Node::gate_voice(v, on)` / `trigger_voice(v)`: route to `State::PolyAr`.
- Render (`poly_process`): `PolyAr` → `a.process(ins[0], ins[1], dt, out)`;
  `PolySvf` → `s.process(poly_in[0].unwrap(), ins[1], ins[2], dt, out)`;
  `PolyMul` → `poly_mul(poly_in[0].unwrap(), poly_in[1].unwrap(), out)`.
- `cmd.rs`: add `GateVoice`/`TriggerVoice`; engine `apply()` routes them.

---

## 5. QA acceptance & testing (Rust; kernel + graph/engine, both feature configs)

**Kernels:**
- **`Ar::tick` refactor:** the existing `env` tests pass unchanged; a direct
  `tick` test matches a hand-computed attack ramp / release decay.
- **`PolyAr`:** `gate_voice(v,true)` ramps voice `v` from 0→1 over ~attack;
  `gate_voice(v,false)` releases to 0; ungated voices stay 0; `trigger_voice`
  one-shots; lanes independent; output ∈ [0,1].
- **`PolySvf`:** a bright input (e.g. a high-frequency saw or noise burst) comes
  out attenuated above cutoff; a sub-cutoff tone passes ~unchanged; per-voice
  state independent; bounded/finite. **Null test:** scalar vs `f32x8` PolySvf
  agree to < 1e-4 over swept cutoff/res on a shared input tile.
- **`poly_mul`:** lanewise product of two known interleaved tiles.

**Graph / engine:**
- `poly_in_count` per the table; two-poly-edge resolution feeds `PolyMul` both
  operands (a `PolyMul` of two poly sources = their lanewise product, rendered).
- **Full voice, per-voice gate (the headline test):** build
  `PolyCtrl(8 distinct pitches) → PolyOsc → PolySvf(cutoff) → PolyMul(·,
  PolyAr) → VoiceSum`. Gate voices 0–1 on, 2–7 off; render: output is
  non-silent and, after the release of all gates + enough samples, trends to
  silence; with **all** gates off the sum is ~0 (ungated voices contribute
  nothing); raising cutoff brightens the sum. Runs in both feature configs.
- Per-voice `GateVoice`/`TriggerVoice` reach only the addressed lane
  (gate voice 3 → only lane 3's envelope moves).

**Determinism:** no RNG; reproducible; proptests seeded; `f32x8` null-tested
against the scalar oracle.

---

## 6. Deferred / follow-ups

- **`PolyAdd` and ≥3-poly-input nodes** (osc mixing, crossfade) — extend the
  poly-input count; trivial once two-poly-input lands.
- **Per-voice cutoff (key-tracking), velocity → level/cutoff, ADSR** — Sy-5.
- **Sy-2b poly breadth:** apply the locked recipe (SoA/`tick` + null-test) to the
  remaining oscillators (`Sync`, `Noise`, `Wavetable`) and filters (`Tb303`,
  `Moog`, `Ms20`, `Modal`) in one grouped pass.
- **Sy-3** allocation/note handling drives `PolyCtrl` pitch + `GateVoice`.
- **Sy-4** Wren `Synth` builder — `Osc`/`Noise`/`.lpf`/`Env.ar` inside it emit
  the poly kinds (single-name ergonomics); the poly kinds stay internal.
