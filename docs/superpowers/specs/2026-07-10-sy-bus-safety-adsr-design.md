# Sy-2e: Bus Poly-Safety + ADSR — Design Spec

**Date:** 2026-07-10
**Suite:** Sy (synth/voice), sub-project Sy-2e
**Status:** Approved — ready for implementation plan

## Goal

Close the last known silent-mono-on-poly footgun — `Bus.write` of a poly node
inside a `Synth` — and add a full ADSR envelope (`Env.adsr(a,d,s,r)`) alongside
the existing `Env.ar`.

## Background

After Sy-2d, one poly footgun remains: `Bus.write(src)` routes through
`write_source_to_bus` → the engine bus-write path, which reads the source as a
single mono `[f32; BLOCK]` row. A poly node's output is a voice-interleaved tile
(`VOICES * n`), so writing a poly node to a bus reads interleaved garbage,
bypassing the `VoiceSum` collapse that `.out` auto-inserts. `Bus.write` has no
`polyMode_` guard (unlike the Sy-2c/2d free-standing classes), and inside a
`Synth` builder (`polyMode_ == 1`) a script can reach it with a raw poly node.

Separately, the only envelope today is `Env.ar` — an attack/sustain-at-1.0/release
machine. A full ADSR (attack, decay, variable sustain level, release) is the
natural next expressiveness primitive.

## Scope

One branch, two independent deliverables:

1. **Bus poly-safety** — a surgical Wren-layer guard.
2. **ADSR** — a new `Adsr`/`PolyAdsr` kernel + graph node + `Env.adsr` surface.

### Explicitly out of scope (deferred)

- **Release-tail voice protection** — the VoiceAllocator frees a lane immediately
  on note-off (release is best-effort, plays unless the lane is reallocated).
  ADSR keeps this exact behavior (identical to `Env.ar` today). A "releasing"
  lane state that reclaims only after the envelope reaches Idle is deferred (it
  needs the kernel to signal idle back to the control plane, or a time estimate).
- **Multiple envelopes per voice** — `Synth.new` still aborts on
  `polyGateCount_ > 1`; a second (filter/mod) envelope per voice is unchanged.
- **Tb303/Modal poly filters, per-voice wavetable morph position** — still deferred.
- **f32x8 ADSR** — the envelope is a branchy serial state machine; scalar-per-voice
  (like `PolyAr`), no SIMD.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP paths. ARM Cortex-A9;
  host x86 for tests only. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Additive, no regression:** `Ar`/`PolyAr` and `Env.ar` stay untouched.
- **Test invocation (per-crate, never `--workspace`; both feature configs):**
  `cargo test --target x86_64-unknown-linux-gnu -p <crate>` and again with
  `--features simd` (kernels) / `--features deluge-dsp-kernels/simd` (graph, wren).
  cargo rejects multiple bare positional test names — use `-- name1 name2`.

## Architecture

### §1 — Bus poly-safety

**File:** `crates/deluge-wren-core/wren/prelude.wren` (+ registration in
`bindings.rs` / `bindings_audio.rs` for the foreign-method rename).

Rename the foreign method `write` → `write_` and interpose a Wren `write` wrapper
that guards:

```wren
foreign class Bus {
  foreign static new_()
  foreign write_(src)
  static new() { new_() }
  write(src) {
    if (Node.polyMode_ == 1 && !(src is Num) && src.isPoly_ == 1) {
      Fiber.abort("can't write a poly voice to a Bus inside a Synth — route the Synth's .out to a bus instead (Sy-2e)")
    }
    write_(src)
  }
}
```

- `!(src is Num)` first: `isPoly_` is a Node/Port getter; a bare numeric constant
  has none and is always a safe mono write.
- Requires `isPoly_` on **both** `Node` and `Port` (Sy-2d added it for the
  operator guards — verify `Port` has it; add if missing).
- Registration: `"write(_)"` → `"write_(_)"` in `bindings_audio.rs`.
- Mono/constant writes and all `Bus.write` **outside** a Synth are unaffected.

This is the exact footgun and nothing more: a poly node written to a mono bus.

### §2 — ADSR kernel

**Files:** `crates/deluge-dsp-kernels/src/env.rs` (new `Adsr`),
`crates/deluge-dsp-kernels/src/poly.rs` (new `PolyAdsr`).

Extend the `Ar` stage machine with a `Decay` stage and a variable `sustain` level.
`Ar` itself stays unchanged; `Adsr` is a new struct.

```rust
pub enum Stage { Idle, Attack, Decay, Sustain, Release }   // Decay added

pub struct Adsr { level: f32, stage: Stage, sustain: f32 }

impl Adsr {
    pub fn new() -> Adsr { Adsr { level: 0.0, stage: Stage::Idle, sustain: 1.0 } }
    pub fn gate(&mut self, on: bool) {
        self.stage = if on { Stage::Attack } else { Stage::Release };
    }
    pub fn trigger(&mut self) { self.stage = Stage::Attack; } // enters attack; a percussive AD (no held tail) is achieved with sustain=0 (decay falls to 0), so Adsr needs no separate oneshot flag
    pub fn set_sustain(&mut self, s: f32) { self.sustain = s.clamp(0.0, 1.0); }

    /// One sample. a/d/r in seconds; sustain is the held level.
    pub fn tick(&mut self, attack: f32, decay: f32, release: f32, dt: f32) -> f32 {
        match self.stage {
            Stage::Attack => {
                self.level += dt / attack.max(1e-4);
                if self.level >= 1.0 { self.level = 1.0; self.stage = Stage::Decay; }
            }
            Stage::Decay => {
                self.level -= dt / decay.max(1e-4);
                if self.level <= self.sustain { self.level = self.sustain; self.stage = Stage::Sustain; }
            }
            Stage::Sustain => self.level = self.sustain,
            Stage::Release => {
                self.level -= dt / release.max(1e-4);
                if self.level <= 0.0 { self.level = 0.0; self.stage = Stage::Idle; }
            }
            Stage::Idle => self.level = 0.0,
        }
        self.level
    }

    pub fn process(&mut self, attack: In, decay: In, release: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            *s = self.tick(attack.at(i), decay.at(i), release.at(i), dt);
        }
    }
}
```

- **Decay convention:** `dt/decay` = a full-scale (1.0→0) slope over `decay`
  seconds, so the 1.0→S segment takes `decay·(1−S)` s — the standard linear ADSR.
- **Release from the current level** — correct if note-off arrives mid-attack or
  mid-decay (releases from wherever `level` is).
- If the `Stage` enum is shared with `Ar`, adding `Decay` must not change `Ar`'s
  behavior (Ar never enters Decay). Confirm `Ar`'s match stays exhaustive/correct.

`PolyAdsr` mirrors `PolyAr` (scalar-per-voice):

```rust
pub struct PolyAdsr { voices: [Adsr; VOICES] }
impl PolyAdsr {
    pub fn new() -> Self { PolyAdsr { voices: [Adsr::new(); VOICES] } }
    pub fn gate_voice(&mut self, v: usize, on: bool) { if v < VOICES { self.voices[v].gate(on); } }
    pub fn trigger_voice(&mut self, v: usize) { if v < VOICES { self.voices[v].trigger(); } }
    pub fn set_sustain(&mut self, s: f32) { for a in &mut self.voices { a.set_sustain(s); } }
    pub fn process(&mut self, attack: In, decay: In, release: In, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            for v in 0..VOICES {
                out[i * VOICES + v] = self.voices[v].tick(attack.at(i), decay.at(i), release.at(i), dt);
            }
        }
    }
}
```

Shared mono a/d/r controls; sustain is a per-node level applied to all voices via
`set_sustain`. Serial recurrence → scalar, no f32x8.

### §3 — Graph node

**File:** `crates/deluge-audio-graph/src/node.rs`.

- `Kind::Adsr` (mono) + `Kind::PolyAdsr`; `State::Adsr(Adsr)` + `State::PolyAdsr(PolyAdsr)`.
- Constructors; `out_width` (Adsr→1, PolyAdsr→VOICES); `is_poly`(PolyAdsr);
  `poly_in_count(PolyAdsr) => 0` (pure source, like PolyAr); process_resolved
  arm (Adsr: `a.process(ins[0], ins[1], ins[2], dt, out)`) and poly_process arm
  (PolyAdsr: same, voice-interleaved out).
- **`set_param` for sustain:** add `State::Adsr(a) if param == 0 => a.set_sustain(value)`
  and `State::PolyAdsr(a) if param == 0 => a.set_sustain(value)` to `Node::set_param`
  (the first envelope `set_param` — small addition).
- **Gate dispatch:** add `State::PolyAdsr` to `Node::gate_voice`/`trigger_voice`
  alongside `State::PolyAr`, so the VoiceAllocator's `GateVoice` drives it
  identically — **no VoiceAllocator change**.

Input mapping (`MAX_INPUTS == 3`): attack=`ins[0]`, decay=`ins[1]`,
release=`ins[2]` (all audio-rate); sustain via `set_param(0)`.

### §4 — Wren surface

**Files:** `crates/deluge-wren-core/src/bindings_audio.rs`, `bindings.rs`,
`wren/prelude.wren`.

```wren
class Env {
  static ar(attack, release) {                 // unchanged
    if (Node.polyMode_ == 1) return Node.polyar_(attack, release)
    return Node.env_(attack, release)
  }
  static adsr(attack, decay, sustain, release) {
    if (Node.polyMode_ == 1) return Node.polyadsr_(attack, decay, sustain, release)
    return Node.adsr_(attack, decay, sustain, release)
  }
}
```

- `node_adsr_impl` / `node_polyadsr_impl` (arity 4): build the node with Inputs
  `[attack, decay, release]`, then emit `SetParam(node, 0, sustain)` for the
  sustain level. `node_polyadsr_impl` also calls `poly_record_gate(id)` (like
  `polyar_`) so ADSR registers as the amp gate — `polyGateCount_` counts it, the
  Synth "exactly one amp envelope" rule holds, and the VoiceAllocator drives it.
- `foreign static adsr_`/`polyadsr_` decls in the prelude `Node` class (the
  Sy-2c gotcha — a missing decl errors "metaclass does not implement"); registered
  in `bindings.rs` METHODS + `register_audio`.

## Data Flow

A sustained ADSR voice:
```
PolyCtrl(pitch) → PolyOsc → PolyMul(·, PolyAdsr) → VoiceSum → out
                                    ↑
   VoiceAllocator: note-on → GateVoice(lane,on) → Attack→Decay→Sustain@S;
                   note-off → GateVoice(lane,off) → Release→Idle (lane freed
                   immediately, release best-effort)
```
`PolyAdsr` is a pure source (`poly_in_count 0`) producing a `VOICES`-wide envelope
tile; sustain is a shared per-node level.

## Error Handling

The only new failure mode is the Bus poly-write misuse, handled by the
`Fiber.abort` guard. No new runtime error paths in the DSP: the `Adsr` kernel is
total, clamped (a/d/r `.max(1e-4)`, sustain clamped [0,1]), output bounded [0,1].

## Testing

Both feature configs, per-crate.

1. **Kernel** (`env.rs`, `poly.rs`): `Adsr` — attack rises to 1.0; decays to
   sustain S and holds; gate-off releases from the current level to 0 → Idle;
   sustain respected (S=0.5 holds 0.5); mid-decay note-off releases from the
   current level; output bounded [0,1]. `PolyAdsr` — each lane == an independent
   mono `Adsr` fed the same a/d/r + its own gate/sustain (bit-identical per-voice
   oracle); per-voice gates independent (gate lane 3 off while others held).
2. **Graph** (`node.rs`): `PolyAdsr` node + `set_param(0, S)` renders the sustain
   hold; `gate_voice`/`trigger_voice` reach the PolyAdsr lane; a `PolyOsc *
   PolyAdsr` voice renders finite/bounded/non-silent with an audible sustain plateau.
3. **Wren** (`tests/audio_bindings.rs`): `Synth.new { |p| Osc.saw(p) *
   Env.adsr(0.01, 0.1, 0.6, 0.3) }` builds (`polyGateCount_ == 1`), renders a
   note, note-off releases; assert bounded/non-silent and the sustained region
   sits near `0.6 · osc`. **Bus guard:** `Bus.write(Osc.saw(p))` (poly) inside a
   Synth aborts; `Bus.write(0.5)` and a mono `Bus.write` outside a Synth do not.

## Success Criteria

- `Bus.write(polyNode)` inside a `Synth` aborts with a clear message; mono/constant
  bus writes and all out-of-Synth bus writes unaffected.
- A playable ADSR voice from Wren: `Synth.new { |p| Osc.saw(p) * Env.adsr(0.01,
  0.1, 0.6, 0.3) }` — attack, decay to 0.6 sustain, hold on a held note, release
  on note-off.
- `PolyAdsr` bit-identical per-voice to the mono `Adsr` oracle.
- `Ar`/`PolyAr`/`Env.ar` unchanged (no regression).
- Both feature configs green, per-crate.
