# Sy-5b: Mono / Legato Glide — Design Spec

**Date:** 2026-07-11
**Suite:** Sy (synth/voice), sub-project Sy-5b (2nd of the Sy-5 expressiveness suite; after Sy-5a velocity)
**Status:** Approved — ready for implementation plan

## Goal

Add a monophonic `Synth.mono { |pitch| … }` mode with true-legato portamento:
successive **overlapping** notes glide the pitch (the amp envelope keeps flowing,
no re-attack); a note from **silence** jumps to pitch and attacks. Glide time is a
live Synth property (`synth.glide = seconds`).

## Background

The poly pitch path is `PolyCtrl` (per-voice semitones, written by the
`VoiceAllocator` as `note-69`) → `PolyMtof` (`2^(semi/12)` Hz) → oscillator. No
slew today. A mono one-pole `Slew` kernel exists (`modutil.rs`, `z += (input-z)·c`,
`c = min(dt/max(time,dt), 1)`), but no poly slew.

**Mono and unison are the same underlying mechanism** — one note driving a *group*
of detuned lanes with a shared gate. So mono is built on the **poly lanes** (not a
separate 1-voice graph), restricted to lane 0 for now; unison (Sy-5e) later widens
"note → lane 0" to "note → N detuned lanes". This keeps the architecture
unison-ready and reuses the audited poly kernels. Glide slews **semitones**
(pre-`PolyMtof`), giving a constant-time exponential glide (musically uniform
across the keyboard).

## Scope (Sy-5b only)

A monophonic voice mode with true-legato glide, last-note priority, built on the
poly graph's lane 0, with a new `PolySlew` kernel and a `MonoAllocator`.

### Explicitly out of scope / deferred

- **Unison** (Sy-5e): widen the `MonoAllocator`'s single lane to N detuned lanes.
  The design leaves lane-grouping as the natural extension point.
- **Retrigger / legato toggle:** Sy-5b ships true-legato only (no re-attack on
  overlap). A settable mode is a later add.
- **Poly-per-voice glide-on-steal:** the minor poly effect, deferred.
- Multiple envelopes (5c), release-tail (5d); Tb303/Modal poly; per-voice
  wavetable morph; f32x8 for the (scalar) slew.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** poly `Synth.new` is entirely unchanged (no `PolySlew`, no
  behavior change). Mono is a new, separate build path.
- **Two registration tables:** every new Wren foreign method goes in BOTH
  `register_audio` (`bindings_audio.rs`) AND the `wren-sys-backend` `METHODS`
  table (`bindings.rs`) — the latter is the one the test VM boots.
- **Test invocation (per-crate, never `--workspace`; both configs; cargo rejects
  multiple bare positional names — use `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## Architecture

### §1 — Voice shape

Mono reuses the poly voice graph (built via the poly factories), with a slew
inserted in the pitch path and allocation restricted to lane 0:

```
PolyCtrl(target semitones) → PolySlew(glide) → PolyMtof → oscillator …
                                                              × Env(gate) → VoiceSum → out
```

`Synth.new` (poly) keeps `PolyCtrl → PolyMtof` (no slew) unchanged.

### §2 — `PolySlew` kernel (`crates/deluge-dsp-kernels/src/poly.rs`)

Mirrors mono `Slew` across `VOICES`, scalar-per-voice, with a scalar glide-`time`
field (set via `set_param`) and a per-lane snap:

```rust
/// Poly one-pole slew/lag (glide). Per-voice `z`; scalar `time` (seconds, set via
/// set_param). `trigger_voice(v)` snaps lane v to its target on the next sample
/// (note-from-silence → no swoop from the previous pitch).
#[derive(Clone, Copy)]
pub struct PolySlew {
    z: [f32; VOICES],
    snap: [bool; VOICES],
    time: f32,
}
impl PolySlew {
    pub fn new() -> Self { PolySlew { z: [0.0; VOICES], snap: [false; VOICES], time: 0.0 } }
    pub fn set_time(&mut self, t: f32) { self.time = t.max(0.0); }
    pub fn trigger_voice(&mut self, v: usize) { if v < VOICES { self.snap[v] = true; } }
    /// `target` = voice-interleaved input tile; writes the slewed tile.
    pub fn process(&mut self, target: &[f32], dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        let c = (dt / self.time.max(dt)).min(1.0); // time≤dt ⇒ c=1 ⇒ snap every sample
        for i in 0..n {
            for v in 0..VOICES {
                let t = target[i * VOICES + v];
                if self.snap[v] { self.z[v] = t; self.snap[v] = false; }
                else { self.z[v] += (t - self.z[v]) * c; }
                out[i * VOICES + v] = self.z[v];
            }
        }
    }
}
```

`c` exactly mirrors mono `Slew`'s coefficient (`dt/max(time,dt)`, clamped 1).
Null-test: one lane vs the mono `Slew` fed `In::Const(time)` is bit-identical;
`trigger_voice` snaps; `time = 0` passes the target through (snap every sample).

### §3 — Graph node (`crates/deluge-audio-graph/src/node.rs`)

- `Kind::PolySlew` + `State::PolySlew(PolySlew)`; constructor; `out_width VOICES`,
  `is_poly`, `poly_in_count 1` (target is the one poly edge; no time port —
  glide time is a `set_param` field).
- `poly_process` arm: `if let (State::PolySlew(s), Some(t)) = (&mut self.state, poly_in[0]) { s.process(t, dt, out); }`.
- `set_param(0)` → `set_time`; `trigger_voice(v)` dispatch → `PolySlew::trigger_voice`
  (alongside the existing `PolyAr`/`PolyAdsr` arms). Add to the `process_resolved`
  no-op poly arm.

### §4 — `MonoAllocator` (`crates/deluge-audio-graph/src/voice.rs`)

Pure control-plane, no heap/panics. Alongside `VoiceAllocator`:

```rust
pub struct MonoAllocator {
    pitch_node: NodeId,
    slew_node:  NodeId,
    gate_node:  NodeId,
    vel_node:   Option<NodeId>,
    notes: [u8; MONO_STACK],  // held-note stack, press order; last = sounding (last-note priority)
    len:   usize,
}
```

Behavior (all on lane 0):
- **`note_on(note, vel, emit)`** (`vel==0` → `note_off`): if `len == 0` (from
  silence): push; `SetParam(pitch, 0, note-69)`; **`TriggerVoice(slew_node, 0)`**
  (snap); optional `SetParam(vel_node, 0, vel/127)`; `GateVoice(gate_node, 0, on)`.
  Else (legato/overlap): push; `SetParam(pitch, 0, note-69)` (glide); optional
  velocity `SetParam`; **no snap, no gate** (true legato). Stack full (`len ==
  MONO_STACK`) → drop the oldest held note (shift), still push the new one.
- **`note_off(note, emit)`**: remove `note` from the stack (first match). If it was
  the top (sounding) note and `len > 0` after removal → `SetParam(pitch, 0,
  newTop-69)` (glide to the fallback note; gate stays on). If `len == 0` →
  `GateVoice(gate_node, 0, off)` (release). If it wasn't the top → stack-only.
  Unheld note → no-op.
- **`all_notes_off(emit)`**: clear the stack; `GateVoice(gate_node, 0, off)`.

`MONO_STACK` is a fixed small const (e.g. 16). Last-note priority = the top of the
stack.

### §5 — Wren surface (`audio.rs`, `bindings_audio.rs`, `bindings.rs`, `prelude.wren`)

- **`PolyCtx`** gains `slew_node: u16` (NULL_ID = none), reset in both begins.
- **`mono_begin()`** (parallel to `poly_begin`): `PolyCtrl` → **`PolySlew`** →
  `PolyMtof`; records `pitch_ctrl` + `slew_node`; `mode=true`; returns the
  `PolyMtof` output (the `pitch` handed to the builder). Velocity/arity detection
  reuses Sy-5a (`polyVelBegin_`).
- **`mono_end(out)`** → `(pitch_ctrl, slew_node, gate_ar, vel_node)`;
  `node_mono_end_impl` builds a `SynthObj` holding a `MonoAllocator`.
- **`SynthObj`** holds an allocator enum `{ Poly(VoiceAllocator), Mono(MonoAllocator) }`;
  `noteOn`/`noteOff`/`out`/`bindMidi`/`allNotesOff` dispatch to the active variant.
- **`synth.glide = seconds`**: a foreign setter emitting `SetParam(slew_node, 0, t)`
  (→ `PolySlew::set_time`). The mono `SynthObj` records `slew_node`; on a poly
  synth `glide=` is a no-op or aborts (a poly synth has no slew) — abort with a
  clear message.
- **Prelude `Synth.mono(builder)`**: mirrors `Synth.new` (nested-abort, `Fn.arity`
  velocity branch, `polyGateCount_` checks) but calls `monoBegin_`/`monoEnd_`.
  `foreign glide=(seconds)` on `Synth`. New foreigns registered in BOTH tables.

## Data Flow

```
mono noteOn(note, vel):
  from silence → SetParam(pitch,0,note-69) + TriggerVoice(slew,0)[snap] + [vel] + GateVoice(env,0,on)
  legato       → SetParam(pitch,0,note-69)[glide] + [vel]                (no re-gate)
mono noteOff(note):
  → top, stack non-empty → SetParam(pitch,0,newTop-69)[glide]
  → stack empty          → GateVoice(env,0,off)

PolyCtrl(lane0 target) → PolySlew(glide/snap) → PolyMtof → osc … × Env(gate) → VoiceSum → out
```

## Error Handling

Held-note stack overflow drops the oldest held note (bounded, no panic). `glide=`
on a poly synth aborts with a clear message. No new DSP failure modes — `PolySlew`
is total, `time.max(0)`, `c` clamped.

## Testing

Both feature configs, per-crate.

1. **Kernel** (`poly.rs`): `PolySlew` lane bit-identical to mono `Slew`
   (`In::Const(time)`) for a swept target; `trigger_voice` snaps instantly;
   `time=0` passes the target through (snap every sample).
2. **MonoAllocator** (`voice.rs`): exact `Cmd` sequences — first note
   (SetParam + TriggerVoice + GateVoice-on), legato note (SetParam only, **no**
   re-gate/trigger), note-off-to-held (SetParam glide, no gate change),
   note-off-to-silence (GateVoice-off), last-note priority (stack order),
   stack-overflow drop, `all_notes_off`. Velocity written on note-on when present.
3. **Graph** (`node.rs`): `PolyCtrl→PolySlew→PolyMtof` glides lane 0 toward a
   changed target; `set_param(0)` sets glide time; `trigger_voice` snaps.
4. **Wren e2e** (`tests/audio_bindings.rs`): `Synth.mono { |p| Osc.saw(p).lpf(1500)
   * Env.adsr(…) }` builds + renders; two overlapping notes glide (pitch ramps)
   with **no** amp re-attack; a from-silence note jumps; `synth.glide = 0.1`
   changes the glide; `Synth.mono { |p, vel| … * vel }` (velocity parity);
   poly `Synth.new` regression unchanged; `glide=` on a poly synth aborts.

## Success Criteria

- `Synth.mono { |p| Osc.saw(p).lpf(1200) * Env.adsr(0.005,0.1,0.7,0.2) }` +
  `synth.glide = 0.08` — a legato line glides between overlapping notes with no
  amp re-attack; staccato notes jump and attack.
- `PolySlew` lane bit-identical to the mono `Slew` oracle.
- Poly `Synth.new` builds/renders unchanged (non-breaking).
- Mono keeps velocity parity (`{ |p, vel| }`).
- Both feature configs green, per-crate.
