# Sy-3: Voice allocation & note handling — design & spec

The third **Sy (synth/voice)** sub-project — where MIDI notes become sound. Sy-2
gave a per-voice-gated voice channel (`PolyCtrl` pitch lanes, per-voice
`GateVoice`/`TriggerVoice`, `PolyAr`/`PolySvf`/`PolyMul`). Sy-3 adds the
**control-plane** that maps note events onto the 8 voice lanes: a `PolyMtof`
semitone→Hz node and a `VoiceAllocator` that, on note-on, picks a free (or
stolen) lane and drives its pitch + gate; on note-off, releases it. Graph/engine
level only — no Wren surface (Sy-4).

> **Status:** design proposal. Depends on merged Sy-2 (`PolyCtrl`, `PolyOsc`,
> `PolySvf`, `PolyMul`, `PolyAr`, `Cmd::{SetParam, GateVoice}`, `poly_in_count`)
> and the merged `Mtof` (`quant.rs`). MIT/Apache. `no_std`, no-heap,
> device-first.

**Sy roadmap:** Sy-1 poly infra ✓ → Sy-2 poly voice channel ✓ → **Sy-3 voice
allocation (this)** → Sy-4 Wren `Synth`/`Voices` surface + MIDI → Sy-2b poly
breadth → Sy-5 expressiveness.

---

## 1. Goals & non-goals

**Goals**
- **`PolyMtof`** — the 8-lane version of `Mtof`: poly-in note-offset lanes → Hz
  lanes, `out[v] = ref_hz · 2^(semitone[v]/12)`. `poly_in_count = 1`,
  `out_width = VOICES`, `param 0 = ref_hz` (default 440). Reuses a shared
  `semitones_to_hz` helper extracted from `Mtof` (DRY). Scalar (`libm::exp2f`
  per lane — not recurrent; input is control-rate).
- **`VoiceAllocator`** — a control-plane struct (`voice.rs`) holding the voice's
  `pitch_node` (a `PolyCtrl`) + `gate_node` (a `PolyAr`) ids and per-lane state.
  Emits `Cmd`s through a closure sink (`&mut impl FnMut(Cmd)`), decoupling it
  from `Engine`'s const generics.
  - **`note_on(note, vel, emit)`:** `vel == 0` → treat as note-off; else pick a
    free lane, else **steal the oldest** (LRU); assign the lane; emit
    `SetParam(pitch_node, lane, note − 69)` (semitones above A440) +
    `GateVoice(gate_node, lane, true)` (re-attacks the envelope from any stage).
  - **`note_off(note, emit)`:** find the most-recently-allocated lane playing
    `note`; emit `GateVoice(gate_node, lane, false)`; free the lane. Absent note
    → no-op.
- **The note-driven voice chain:** `PolyCtrl(semitone) → PolyMtof → PolyOsc →
  PolySvf → PolyMul(·, PolyAr) → VoiceSum`.
- **QA-proven:** notes fill free lanes; a 9th note steals the oldest; note-off
  releases the correct lane (silences it); `vel == 0` is a note-off; a note-off
  for an unheld note is a no-op; `PolyMtof` maps `0 → 440 Hz`, `+12 → 880 Hz`,
  per-lane independent; end-to-end, played notes sound at the right pitch and
  note-off returns to silence.

**Non-goals (deferred)**
- **Velocity → amplitude / level** (needs a per-voice gain path) — Sy-5. Sy-3
  uses velocity only for the `vel == 0` → note-off convention.
- **Sustain pedal, mono/legato modes, unison/detune, same-note retrigger mode,
  quietest-voice stealing, configurable root note / microtuning** — later.
- **Wren surface / `Midi.onNoteOn` binding** — Sy-4 (drives the allocator).
- **`f32x8` `PolyMtof`** — YAGNI: pitch is control-rate (near-constant per
  block), not a recurrent hot kernel; scalar per-lane `exp2f` suffices.

---

## 2. `PolyMtof` (`deluge-dsp-kernels/src/poly.rs` + graph node)

### 2.1 Shared `semitones_to_hz` helper (`quant.rs`, DRY)

Extract the `Mtof` per-sample formula into a shared helper and have `Mtof` use it:

```rust
/// Semitones above `ref_hz` → Hz: `ref_hz · 2^(st/12)`.
#[inline]
pub(crate) fn semitones_to_hz(st: f32, ref_hz: f32) -> f32 {
    ref_hz * libm::exp2f(st / 12.0)
}
```

`Mtof::process` becomes `out[i] = semitones_to_hz(input.at(i), self.ref_hz)`
(behavior-preserving — existing `quant` tests pass verbatim).

### 2.2 `PolyMtof` kernel (`poly.rs`)

```rust
use crate::quant::semitones_to_hz;

/// Poly semitone→Hz: `out[v] = ref_hz · 2^(semitone[v]/12)`. Poly-in note-offset
/// lanes → Hz lanes. `ref_hz` set via `set_ref`. Scalar (control-rate pitch).
#[derive(Clone, Copy)]
pub struct PolyMtof { ref_hz: f32 }
impl PolyMtof {
    pub fn new() -> PolyMtof;                 // ref_hz = 440
    pub fn set_ref(&mut self, hz: f32);
    /// `semitones` = voice-interleaved note-offset tile; writes an Hz tile.
    pub fn process(&mut self, semitones: &[f32], out: &mut [f32]);
}
```

Per lane: `out[i*VOICES+v] = semitones_to_hz(semitones[i*VOICES+v], self.ref_hz)`.
`no_std`, deterministic.

### 2.3 Graph node
- `enum Kind`: add `PolyMtof`. `enum State`: `PolyMtof(PolyMtof)`.
- `Node::new`: construct. `out_width → VOICES`. `is_poly` true. `poly_in_count → 1`.
- `set_param`: `0 => set_ref(value)`.
- `poly_process`: `PolyMtof` → `if let (State::PolyMtof(m), Some(pin)) =
  (&mut self.state, poly_in[0]) { m.process(pin, out) }` (no `dt`), mirroring
  `PolySvf`'s poly-in handling.

---

## 3. `VoiceAllocator` (`deluge-audio-graph/src/voice.rs`, new)

```rust
use crate::{cmd::Cmd, NodeId};
use deluge_dsp_kernels::poly::VOICES;

pub struct VoiceAllocator {
    pitch_node: NodeId,   // a PolyCtrl: SetParam(pitch_node, lane, note-69)
    gate_node: NodeId,    // a PolyAr:   GateVoice(gate_node, lane, on/off)
    lane_note: [Option<u8>; VOICES],
    lane_age: [u32; VOICES],
    clock: u32,
}
impl VoiceAllocator {
    pub fn new(pitch_node: NodeId, gate_node: NodeId) -> VoiceAllocator;
    pub fn note_on(&mut self, note: u8, vel: u8, emit: &mut impl FnMut(Cmd));
    pub fn note_off(&mut self, note: u8, emit: &mut impl FnMut(Cmd));
    /// Release every held lane (panic/all-notes-off).
    pub fn all_notes_off(&mut self, emit: &mut impl FnMut(Cmd));
}
```

**Allocation (`note_on`):**
1. `vel == 0` → `note_off(note, emit)`; return.
2. Pick a lane: first `v` with `lane_note[v] == None`; if none, the `v` with the
   smallest `lane_age[v]` (oldest — LRU steal). Stealing needs no explicit gate-
   off: reassigning + `GateVoice(on)` re-attacks the stolen lane's envelope.
3. `lane_note[v] = Some(note)`, `lane_age[v] = self.clock`, `self.clock += 1`.
4. `emit(Cmd::SetParam { node: pitch_node, param: v as u8, value: note as f32 - 69.0 })`.
5. `emit(Cmd::GateVoice { node: gate_node, voice: v as u8, on: true })`.

**Release (`note_off`):** find the `v` with `lane_note[v] == Some(note)` and the
largest `lane_age[v]` (most recent). If found: `emit(GateVoice{gate_node, v,
false})`, `lane_note[v] = None`. If none, no-op (never panic).

**`clock` wrap:** `u32` monotonic; at ~millions of notes it wraps — acceptable
(a stale age only mis-orders one steal). No unsafe, no heap, no panics.

The allocator is transport-agnostic: the closure sink applies each `Cmd` (tests
use `|c| engine.apply(c)`; Sy-4 routes through the `Host` transport).

---

## 4. Integration — the note-driven voice

The full voice a `Synth` (Sy-4) will build, and Sy-3 builds in tests:

```
PolyCtrl(id=pitch) → PolyMtof → PolyOsc → PolySvf(cutoff,res) →
    PolyMul(·, PolyAr(id=gate)) → VoiceSum → out
```

`VoiceAllocator::new(pitch_id, gate_id)` drives `PolyCtrl` (semitone lanes) and
`PolyAr` (gates). `PolyMtof` converts semitone → Hz for `PolyOsc`. No graph
topology changes beyond inserting `PolyMtof` between `PolyCtrl` and `PolyOsc`.

---

## 5. QA acceptance & testing (Rust; kernel + allocator + engine)

**`PolyMtof` (kernel):** `0 → 440 Hz`, `+12 → 880`, `−12 → 220`; per-lane
independent (8 distinct note-offsets → 8 distinct Hz); `set_ref` retunes; bounded,
finite, > 0. `Mtof` refactor behavior-preserving (existing `quant` tests pass).

**`VoiceAllocator` (unit, no engine — capture emitted `Cmd`s into a `Vec`):**
- Note-on to an empty allocator uses lane 0 and emits `SetParam(pitch, 0, note-69)`
  + `GateVoice(gate, 0, true)`.
- Eight distinct note-ons fill lanes 0..8 (each a free lane).
- A ninth note-on steals the **oldest** lane (lowest age) — verify the reused
  lane and that its `SetParam`/`GateVoice(on)` re-emit.
- `note_off(note)` emits `GateVoice(gate, lane, false)` on the lane that played
  `note` and frees it (a later note-on reuses it).
- `note_off` for an unheld note emits nothing (no-op, no panic).
- `note_on(note, 0, …)` behaves as `note_off(note)` (no `GateVoice(on)`).
- `all_notes_off` gates every held lane off.

**End-to-end (engine):** build the §4 voice; construct `VoiceAllocator(pitch_id,
gate_id)`; play a note (`note_on(69, 100, |c| e.apply(c))`) → render → the sum is
non-silent and at ~440 Hz (dominant partial near A4, via spectrum or
zero-crossing rate); `note_off(69, …)` → after the release window, output returns
to ~silence; two simultaneous notes → two partials; a 9th note with all lanes
held steals and still sounds. Runs in both feature configs.

**Determinism:** no RNG; reproducible; the allocator is pure integer/lane logic.

---

## 6. Deferred / follow-ups

- **Velocity → amplitude/level** (a per-voice gain `PolyCtrl` × the VCA), sustain
  pedal, mono/legato, unison/detune, same-note retrigger mode — Sy-5.
- **Quietest-voice stealing** (needs per-voice envelope-level readback from
  `PolyAr`), configurable root note / microtuning — later refinements.
- **Sy-4** Wren `Synth`/`Voices` + `Midi.onNoteOn/off` → `VoiceAllocator`
  (single-name ergonomics: `Osc`/`Env`/`.lpf` inside a `Synth` builder emit the
  poly kinds; the `Synth` owns a `VoiceAllocator`).
- **`f32x8` `PolyMtof`** only if profiling shows the scalar `exp2f×8` matters
  (it shouldn't — control-rate).
