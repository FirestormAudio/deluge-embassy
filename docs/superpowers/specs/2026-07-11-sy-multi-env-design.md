# Sy-5c: Multiple Envelopes per Voice — Design Spec

**Date:** 2026-07-11
**Suite:** Sy (synth/voice), sub-project Sy-5c (3rd of the Sy-5 expressiveness suite; after 5a velocity, 5b mono-glide)
**Status:** Approved — ready for implementation plan

## Goal

Allow up to **4 envelopes** in a `Synth.new` / `Synth.mono` voice, each gated by
the same note-on/off, so a filter/mod envelope works alongside the amp envelope:

```wren
Synth.new { |p| Osc.saw(p).lpf(Env.adsr(0.01,0.2,0.3,0.4).to(400,4000)) * Env.adsr(0.005,0.1,0.7,0.2) }
```

## Background

Today a Synth voice allows exactly ONE envelope. `PolyCtx` records `gate_ar` (the
LAST `Env.ar`/`Env.adsr`, overwritten each time) + `gate_count`; `poly_end`/
`mono_end` return that single id; both `VoiceAllocator` and `MonoAllocator` hold a
single `gate_node` and emit every `GateVoice` to it. `Synth.new`/`Synth.mono`
abort unless `polyGateCount_ == 1`. A second envelope's `PolyAr`/`PolyAdsr` node
exists in the graph but never receives `GateVoice`, so it stays `Idle` (outputs 0)
— which is why `> 1` is currently rejected.

The kernels (`PolyAr`/`PolyAdsr`) already support independent instances, and
`Node::gate_voice`/`trigger_voice` already dispatch per node id. The change is
purely **control-plane**: record a list of gate ids and fan `GateVoice` out to all
of them.

## Scope (Sy-5c only)

Up to 4 gated envelopes per voice, in both poly and mono modes. All envelopes are
**equal** — no distinguished "amp env"; each recorded envelope is gated by the
note, and the voice sounds because the builder multiplies the signal by one (the
amp) and routes another to a target (e.g. cutoff). The requirement relaxes from
"exactly 1 envelope" to "1–4 envelopes".

### Explicitly out of scope / deferred

- **Release-tail voice protection** (5d): note-off still frees the lane
  immediately; release is best-effort. With multiple envelopes, note-off gates
  ALL of them off.
- **Unison** (5e), **poly-per-voice glide**, Tb303/Modal poly filters, per-voice
  wavetable morph.
- Per-envelope trigger vs gate modes, or a distinguished amp env (all envelopes
  are gated identically).

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** a 1-envelope Synth (`n_gates == 1`) builds/renders
  byte-identically to today (both poly and mono).
- **Budget:** each envelope is a `PolyAr`/`PolyAdsr` node = `VOICES = 8` of the
  engine's 128 output slots. 4 envelopes (32 slots) leaves room for the rest of a
  voice.
- **Two registration tables:** no new Wren foreign methods here (guard is a
  relaxed number), so no table change — but keep the rule in mind for any incidental foreign.
- **Test invocation (per-crate, never `--workspace`; both configs; cargo rejects
  multiple bare positional names — use `-- name1 name2`):**
  - Graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## Architecture

### §1 — `MAX_GATES` + the gate list

`pub const MAX_GATES: usize = 4;` in `deluge-audio-graph` (near the allocators),
imported by wren-core as `deluge_audio_graph::MAX_GATES`.

### §2 — Allocators (`crates/deluge-audio-graph/src/voice.rs`)

Both allocators replace the single `gate_node: NodeId` with a fixed list:

```rust
// VoiceAllocator
gates: [NodeId; MAX_GATES],
n_gates: usize,
// new(pitch_node, gates: [NodeId; MAX_GATES], n_gates: usize, vel_node: Option<NodeId>)

// MonoAllocator
gates: [NodeId; MAX_GATES],
n_gates: usize,
// new(pitch_node, slew_node, gates: [NodeId; MAX_GATES], n_gates: usize, vel_node: Option<NodeId>)
```

Every `GateVoice` emit fans out:
```rust
for g in &self.gates[..self.n_gates] {
    emit(Cmd::GateVoice { node: *g, voice: lane as u8, on });
}
```

- **`VoiceAllocator`:** note-on emits pitch `SetParam` + [velocity `SetParam`] +
  `GateVoice(on)` to each gate; note-off + `all_notes_off` emit `GateVoice(off)` to
  each gate; voice-steal re-attacks all gates via the note-on path.
- **`MonoAllocator`:** from-silence → pitch `SetParam` + [vel] + `TriggerVoice(slew)`
  + `GateVoice(on)` to **each** gate; legato → pitch `SetParam` only (re-gates NO
  envelope — true legato holds); note-off-to-held → pitch `SetParam` (glide back);
  note-off-to-silence + `all_notes_off` → `GateVoice(off)` to each gate.

`n_gates == 1` reproduces today's exact single-gate emit sequence (order: pitch,
[vel], [trigger for mono-from-silence], then the one gate).

### §3 — `PolyCtx` + build path (`crates/deluge-wren-core/src/audio.rs`, `bindings_audio.rs`)

- `PolyCtx.gate_ar: u16` → `gates: [u16; MAX_GATES]`; keep `gate_count: u8`.
- `poly_record_gate(id)`: `if gate_count < MAX_GATES { gates[gate_count] = id; }`
  then `gate_count += 1` (saturating). A 5th+ envelope increments `gate_count`
  past `MAX_GATES` (so the Wren guard aborts) but never writes past the array.
- Reset `gates`/`gate_count` in both `poly_begin` and `mono_begin` (already reset
  `gate_count`; also clear the array or rely on `gate_count` bounding reads).
- `poly_end()` / `mono_end()`: return the gate list + count alongside their
  existing tuple (e.g. `poly_end() -> (pitch_ctrl, gates: [u16; MAX_GATES],
  gate_count: u8)`; `mono_end` adds `slew`). `node_poly_end_impl` /
  `node_mono_end_impl` map the `u16` ids to `[NodeId; MAX_GATES]` (only the first
  `gate_count` are meaningful) and pass `n_gates = gate_count.min(MAX_GATES)`.
- `SynthAlloc` (the `{ Poly | Mono }` enum) is unchanged — only the wrapped
  constructors' arguments changed.

### §4 — Wren guard (`crates/deluge-wren-core/wren/prelude.wren`)

In both `Synth.new` and `Synth.mono`, relax the count guard:

```wren
if (Node.polyGateCount_ == 0) Fiber.abort("a Synth voice needs an Env.ar (the amp gate)")
if (Node.polyGateCount_ > 4)  Fiber.abort("more than 4 envelopes per voice isn't supported")
```

(`> 1` → `> 4`.) No other Wren change; `Env.ar`/`Env.adsr` can appear up to 4
times and each is auto-recorded + gated.

## Data Flow

```
build: each Env.ar/adsr → poly_record_gate(id) → PolyCtx.gates[0..count]
poly_end/mono_end → (pitch, [slew], gates[0..count]) → Allocator::new(…, gates, n_gates, …)

noteOn(note): SetParam(pitch) + [vel] + [TriggerVoice(slew) mono-from-silence]
              + GateVoice(on) × n_gates   (each recorded envelope attacks)
noteOff(note): GateVoice(off) × n_gates   (each envelope releases)
```

Each envelope runs independently on `VOICES` lanes; the builder wires each one's
output to its target (amp = `× env`, filter = `.lpf(env.to(a,b))`, etc.).

## Error Handling

`poly_record_gate` never writes past `MAX_GATES` (bounded push; `gate_count` still
counts for the guard). The `> 4` Wren abort is the only new failure mode. No new
DSP paths.

## Testing

Both feature configs, per-crate.

1. **Allocator** (`voice.rs`): with `n_gates == 2`, `VoiceAllocator::note_on`
   emits pitch `SetParam` + `GateVoice(on)` to BOTH gate nodes (in order);
   `note_off` emits `GateVoice(off)` to both; voice-steal re-gates both; velocity
   still written; `n_gates == 1` byte-identical to today. Same battery for
   `MonoAllocator` (from-silence gates all + trigger slew; legato = pitch-only, no
   gate to any envelope; note-off-to-silence = gate-off all; last-note priority
   unaffected). Update existing single-gate tests to the new constructor.
2. **Wren e2e** (`tests/audio_bindings.rs`): a 2-envelope `Synth.new` (amp + a
   filter env via `.lpf(Env.adsr(…).to(400,4000))`) builds (`polyGateCount_ == 2`)
   and renders finite/bounded/non-silent; a 4-envelope voice builds; a 5-envelope
   voice **aborts**; a 0-envelope voice still aborts; `Synth.mono` with 2
   envelopes builds + renders. Existing 1-envelope synths unchanged.

## Success Criteria

- `Synth.new { |p| Osc.saw(p).lpf(Env.adsr(…).to(400,4000)) * Env.adsr(…) }` — the
  filter sweeps on its own envelope while the amp shapes level; both gate on the
  same note, release on note-off.
- Up to 4 envelopes per voice; a 5th aborts with a clear message; 0 still aborts.
- 1-envelope synths (poly and mono) build/render byte-identically (non-breaking).
- Works in both poly (`Synth.new`) and mono (`Synth.mono`) modes.
- Both feature configs green, per-crate.
