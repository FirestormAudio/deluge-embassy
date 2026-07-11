# Sy-5a: Velocity → per-voice signal — Design Spec

**Date:** 2026-07-11
**Suite:** Sy (synth/voice), sub-project Sy-5a (first of the Sy-5 expressiveness suite)
**Status:** Approved — ready for implementation plan

## Goal

Route MIDI note-on velocity — currently received by the `VoiceAllocator` but
dropped — to a per-voice `[0,1]` signal, exposed as an optional second `Synth`
builder parameter: `Synth.new { |pitch, vel| Osc.saw(pitch) * Env.adsr(…) * vel }`.
Existing single-parameter synths (`{ |pitch| … }`) are unaffected.

## Background

`VoiceAllocator::note_on(note, vel, emit)` (voice.rs:33) uses `vel` only for the
MIDI `vel==0 → note_off` convention; the value is otherwise discarded. The Wren
path already carries real velocity: `Synth.noteOn(note, vel)` / `bindMidi` →
`synth_note_on_impl` → `alloc.note_on(note, vel, …)`. Only the allocator and the
voice graph ignore it.

The per-voice control mechanism needed already exists: `PolyCtrl`
(`poly.rs`, `values: [f32; VOICES]`, `set_voice(v, x)`), written per-lane via
`Cmd::SetParam{param=lane}` (this is exactly how per-voice pitch is delivered).
So velocity needs **no new kernel** — a second `PolyCtrl` carries it.

## Scope (Sy-5a only)

Velocity → an exposed per-voice control signal. Amp scaling, velocity→cutoff,
and sensitivity curves are all **user-composed** on that signal (the library's
SuperCollider-like philosophy), not baked in.

### Explicitly out of scope / deferred

- The other Sy-5 sub-projects, in recommended order: **portamento**
  (`PolySlew` in the pitch path), **multiple envelopes** (VoiceAllocator
  multi-gate fan-out + relax `polyGateCount_==1`), **release-tail voice
  protection** (per-lane releasing lifecycle), **unison** (one-note→N-lanes
  allocator rewrite).
- A velocity *curve* primitive (users compose `vel.curve(k)` / `vel.to(a,b)`).
- Tb303/Modal poly filters, per-voice wavetable morph position (deferred elsewhere).

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** existing `Synth.new { |pitch| … }` synths build and render
  bit-identically (no velocity node created for arity-1 builders).
- **Node budget:** the velocity `PolyCtrl` is one node + `VOICES` output slots,
  created only when the builder takes velocity (lazy). Engine `NODES=64`,
  `OUTS=128` — the lazy path keeps arity-1 synths at their current footprint.
- **Test invocation (per-crate, never `--workspace`; both feature configs; cargo
  rejects multiple bare positional names — use `-- name1 name2`):**
  - Graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).

## Architecture

### §1 — Velocity carrier (no new kernel)

A second `PolyCtrl` node holds per-voice velocity in `[0,1]`. It is a
width-`VOICES` poly node but is flagged as a **control** signal in Wren
(`isPoly_ == 0`, exactly like the pitch `PolyCtrl` and `Env`), so:
- `audioSignal * vel` → `PolyMul(audioSignal, vel)` lanewise (both are `Node`s;
  the Sy-2d `*` operator does `polymul_` for a Node operand).
- `vel * 0.5 + 0.5` (sensitivity) → the Sy-2d control-scaling path (scalar wrapped
  in a `Ctrl` node, broadcast) — works because `vel` is a control Node.
- `vel.to(400, 4000)` / `vel.curve(k)` compose for velocity→cutoff etc.

Velocity adds **no gate** (it is a `PolyCtrl`, not a `PolyAr`/`PolyAdsr`), so the
`polyGateCount_ == 1` amp-gate rule is untouched.

### §2 — VoiceAllocator (`crates/deluge-audio-graph/src/voice.rs`)

- Add `vel_node: Option<NodeId>`; constructor `new(pitch_node, gate_node, vel_node)`.
- In `note_on`, after the pitch `SetParam` and before the `GateVoice`:
  ```rust
  if let Some(vn) = self.vel_node {
      emit(Cmd::SetParam { node: vn, param: lane as u8, value: vel as f32 / 127.0 });
  }
  ```
- `note_off` / `all_notes_off` unchanged — velocity is a steady per-voice
  multiplier read only while the voice sounds; the amp env's release owns the
  tail. A stolen/re-attacked lane receives its new velocity from the next
  `note_on` (same lifecycle as pitch).
- `vel==0 → note_off` convention unchanged (velocity never written on note-off).

### §3 — Wren surface (`prelude.wren`, `bindings_audio.rs`, `audio.rs`, `bindings.rs`)

`Synth.new` inspects the builder's arity (`Fn.arity`) and lazily creates the
velocity carrier:

```wren
static new(builder) {
  if (Node.polyMode_ == 1) Fiber.abort("nested Synth not supported")
  var pitch = Node.polyBegin_()
  var out
  if (builder.arity >= 2) {
    var vel = Node.polyVelBegin_()   // creates the velocity PolyCtrl, records it in PolyCtx
    out = builder.call(pitch, vel)
  } else {
    out = builder.call(pitch)
  }
  if (Node.polyGateCount_ == 0) Fiber.abort("a Synth voice needs an Env.ar (the amp gate)")
  if (Node.polyGateCount_ > 1) Fiber.abort("multiple Env.ar in a Synth isn't supported yet")
  return Node.polyEnd_(out)
}
```

- **`polyVelBegin_`** (new foreign, `bindings_audio.rs`): `audio::alloc_node_id()`
  → `new_node(id, Kind::PolyCtrl, [Const(0.0); 3])`, store `id` in a new
  `PolyCtx.vel_node: Option<u16>`, `return_node` (control-flagged, `isPoly_==0`).
  Registered in BOTH method tables (`register_audio` in `bindings_audio.rs` AND
  the `wren-sys-backend` `METHODS` in `bindings.rs` — the table the test VM boots)
  + a `foreign static polyVelBegin_()` decl in the prelude `Node` class.
- **`poly_end`** (`audio.rs`): returns the recorded `vel_node` (or `None`)
  alongside `(pitch_ctrl, gate_ar)`; `node_poly_end_impl` passes it to
  `VoiceAllocator::new(pitch, gate, vel_node)`.
- **`PolyCtx`** gains `vel_node: Option<u16>`, reset in `poly_begin`, set by
  `poly_vel_begin`.

Everything else in the build path is unchanged.

## Data Flow

```
noteOn(note, vel) → VoiceAllocator.note_on:
    SetParam(pitch_node, lane, note-69)         (pitch)
    SetParam(vel_node,   lane, vel/127)         (velocity — only if vel_node Some)
    GateVoice(gate_node, lane, on:true)         (amp gate)

voice graph: Osc(pitch→PolyMtof) * Env.adsr(gate) * vel  →  PolyMul lanewise
```

`vel` is a width-`VOICES` `PolyCtrl` output; the user multiplies/routes it. High
velocity → larger per-lane value → louder (or brighter, if routed to cutoff).

## Value Mapping

Velocity `1..127` → `vel as f32 / 127.0` ∈ `(0.008, 1.0]` (standard MIDI linear
normalization). Deliberately linear — shaping is user-composed on the exposed
signal. `vel==0` is note-off and never written.

## Error Handling

No new failure modes. Arity detection is total (1 vs ≥2). A builder that takes
`vel` but doesn't route it still works (a steady, unused multiplier). No new aborts.

## Testing

Both feature configs, per-crate.

1. **VoiceAllocator** (`voice.rs`): with `Some(vel_node)`, `note_on` emits 3 Cmds
   in order — pitch `SetParam`, velocity `SetParam` (value ≈ `vel/127` on the
   allocated lane), `GateVoice(on)`; with `None`, exactly 2 (unchanged). A high vs
   low velocity writes proportionally larger/smaller values on the correct lane.
   Existing allocator tests updated to `new(…, None)` and still green.
2. **Wren end-to-end** (`tests/audio_bindings.rs`):
   - `Synth.new { |pitch, vel| Osc.saw(pitch) * Env.adsr(0.001,0.001,1,0.1) * vel }`
     builds (arity-2 path); `noteOn(60,127)` renders a higher RMS than
     `noteOn(60,20)` (velocity actually scales amplitude).
   - **Backward-compat gate:** an existing arity-1 `Synth.new { |pitch| … }` builds
     and renders identically to before (no velocity node created).
   - `vel` composes as a control signal: `… .lpf(vel.to(400, 4000))` (velocity→cutoff)
     and `vel * 0.5 + 0.5` (sensitivity) both build (exercise the Sy-2d
     control-scaling path).

## Success Criteria

- `Synth.new { |pitch, vel| Osc.saw(pitch) * Env.adsr(…) * vel }` — velocity
  scales amplitude; higher velocity is louder.
- Existing `{ |pitch| … }` synths build/render bit-identically (non-breaking).
- `vel` is a first-class per-voice control signal, routable anywhere (amp,
  cutoff, mod) and shapeable (`* k + b`, `.to`, `.curve`).
- Both feature configs green, per-crate.
