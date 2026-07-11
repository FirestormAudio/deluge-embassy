# Sy-5e: Unison — Design Spec

**Date:** 2026-07-11
**Suite:** Sy (synth/voice), sub-project Sy-5e (5th and last of the Sy-5 expressiveness suite; after 5a velocity, 5b mono-glide, 5c multi-env, 5d release-tail)
**Status:** Approved — ready for implementation plan

## Goal

Stack `U` detuned voices per note for fat leads/pads/bass, in BOTH mono
(`Synth.mono`) and poly (`Synth.new`) modes, controlled by `synth.unison = N`
and `synth.detune = cents`.

## Background

`VOICES = 8` is a hard compile-time cap. Unison spends U of the 8 lanes on one
note (poly: `floor(8/U)` simultaneous notes; mono: always one note, U voices).
The per-voice pitch mechanism already supports per-lane values (`PolyCtrl`
`set_voice(lane, x)` via `SetParam{param=lane}`), so detune = writing `note-69 +
offset[u]` to each of a note's U lanes. Both allocators already gate a lane
(`gate_all`, Sy-5c fan-out); unison drives U lanes per note. The mono voice
collapses via `VoiceSum` (sums all VOICES lanes → mono).

## Scope (Sy-5e)

Both **mono-unison** (`MonoAllocator` drives lanes 0..U) and **poly-unison**
(`VoiceAllocator` grabs U lanes per note), a shared detune spread, `VoiceSum`
1/√U amplitude normalization, and two live Synth setters.

### Explicitly out of scope / deferred

- **Stereo spread** (per-lane pan for width) — needs a stereo sum, not the mono
  `VoiceSum`. A later pass.
- **Group-partitioned poly-unison** — the chosen model is mark-and-pick over the
  existing per-lane `LaneState` (reuses 5d release-tail), NOT fixed contiguous
  groups.
- **Live re-pitch on detune change** — unison/detune take effect on the next
  note-on (standard).
- Tb303/Modal poly filters, per-voice wavetable morph.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** `unison == 1` is byte-identical to today (poly AND mono): one
  lane, offset 0, `VoiceSum` gain 1/√1 = 1.0. `VoiceSum`'s default gain is 1.0.
- **Two registration tables:** new Wren foreign setters go in BOTH `register_audio`
  (`bindings_audio.rs`) AND the `wren-sys-backend` `METHODS` table (`bindings.rs`).
- **Test invocation (per-crate, never `--workspace`; both configs; use `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## Architecture

### §1 — Shared detune helper (`crates/deluge-audio-graph/src/voice.rs`)

A free fn used by both allocators:

```rust
/// Semitone offset for unison voice `u` of `count`, spread symmetrically and
/// evenly over ±`detune_cents`. `count <= 1` ⇒ 0 (no detune).
fn unison_offset(u: usize, count: usize, detune_cents: f32) -> f32 {
    if count <= 1 { return 0.0; }
    let t = -1.0 + 2.0 * (u as f32) / ((count - 1) as f32); // -1..+1
    t * detune_cents / 100.0
}
```

### §2 — Poly unison (`VoiceAllocator`)

Add `unison: usize` (default 1) + `detune_cents: f32` (default 0) + `set_unison(n)`
(clamp 1..=VOICES) / `set_detune(cents)`.

- **`note_on(note, vel, emit)`** (`vel==0` → `note_off`): `U = self.unison.min(VOICES)`.
  For `u in 0..U`:
  - `lane = pick_lane()`; `lane_state[lane] = Held(note)`; `lane_age[lane] =
    clock`; `clock = clock.wrapping_add(1)` — **mark before the next `pick_lane`**
    so the U lanes are distinct.
  - emit `SetParam(pitch_node, lane, note as f32 - A440_NOTE + unison_offset(u, U,
    detune_cents))`; if `vel_node`, `SetParam(vel_node, lane, vel/127)`;
    `gate_all(lane, true)`.
  - (U=1 ⇒ one lane, offset 0 ⇒ today's exact single-lane emit.)
- **`note_off(note, emit)`**: release EVERY lane whose state is `Held(note)`: for
  each, `gate_all(lane, false)`, `lane_state[lane] = Releasing`, `lane_age =
  clock`, `clock += 1`. (Was: most-recent only.)
- `all_notes_off` unchanged (already releases all held lanes → Releasing).
- Release-tail (5d) composes: a note's U lanes all become `Releasing` together; a
  new note's `pick_lane`×U prefers Free → oldest-Releasing → steal.

### §3 — Mono unison (`MonoAllocator`)

Add the same `unison` + `detune_cents` + setters. Monophonic → always the first
**U lanes** (0..U):

- **`note_on` from-silence**: for `u in 0..U`: `SetParam(pitch_node, u, note-69 +
  offset)`, [`SetParam(vel_node, u, vel/127)`], `TriggerVoice(slew_node, u)` (snap
  each lane's glide), `gate_all(u, true)` (attack each).
- **`note_on` legato**: for `u in 0..U`: `SetParam(pitch_node, u, note-69 +
  offset)` only — all U glide, no re-gate (true legato holds across unison voices).
- **`note_off` to-silence** (stack empty): for `u in 0..U`: `gate_all(u, false)`.
- **`note_off` glide-back** (to a held note): for `u in 0..U`: `SetParam(pitch_node,
  u, newTop-69 + offset)`.

`gate_all(lane, on, emit)` already takes a lane; call it per unison lane. U=1
reproduces today's lane-0-only mono exactly.

### §4 — `VoiceSum` gain + Wren surface

- **`VoiceSum` gains a gain** (`crates/deluge-dsp-kernels/src/poly.rs` +
  `node.rs`): make it stateful — `State::VoiceSum(f32 /*gain*/)` (default 1.0),
  `set_param(0)` = gain, and the `voice_sum` sum multiplied by the gain
  (`out[i] = gain * Σ_v tile[i*VOICES+v]`). Default 1.0 ⇒ existing synths
  byte-unchanged. (Currently `Kind::VoiceSum` is stateless calling a free
  `voice_sum(tile, out)`; give it state + a gain arg.)
- **Wren** (`bindings_audio.rs`, `bindings.rs`, `prelude.wren`): two foreign
  setters on the `Synth` foreign class:
  - `synth.unison = N`: clamp N to 1..=VOICES; call the active allocator's
    `set_unison(N)` (via a `SynthAlloc::set_unison` dispatch); emit
    `SetParam(out_node, 0, 1/(N as f32).sqrt())` (the `VoiceSum` amplitude
    normalization; `SynthObj.out_node` is the `VoiceSum` id).
  - `synth.detune = cents`: call the active allocator's `set_detune(cents)` (via
    `SynthAlloc::set_detune`).
  - `SynthAlloc` gains `set_unison(n)` / `set_detune(c)` dispatch methods
    (matching on Poly/Mono), like the existing `note_on`/`mono_slew`.
  - Register `unison=(_)` / `detune=(_)` as `Synth` instance setters in BOTH
    tables + `foreign` decls in the prelude `Synth` class.

## Data Flow

```
synth.unison = N → SynthAlloc.set_unison(N)  +  SetParam(VoiceSum, 0, 1/√N)
synth.detune = c → SynthAlloc.set_detune(c)

poly noteOn(note): for u in 0..U: lane=pick_lane(); mark Held(note);
    SetParam(pitch, lane, note-69 + offset[u]) + [vel] + gate_all(lane, on)
poly noteOff(note): for each lane Held(note): gate_all(off) + mark Releasing

mono noteOn(note) from silence: for u in 0..U:
    SetParam(pitch, u, note-69 + offset[u]) + [vel] + TriggerVoice(slew, u) + gate_all(u, on)

VoiceSum: out = (1/√U) · Σ lanes   (U detuned voices → fat, level-normalized)
```

## Error Handling

`unison` clamped to `1..=VOICES`; `detune`/`unison`/`gain` setters are total; no
new panics. `unison == 1` is the byte-identical baseline.

## Testing

Both feature configs, per-crate.

1. **`unison_offset`** (voice.rs unit): U=1 → 0.0; U=2 @ 10¢ → ±0.1 semitone;
   U=3 @ 10¢ → −0.1, 0.0, +0.1 — symmetric, evenly spaced, cents/100.
2. **`VoiceSum` gain** (poly.rs kernel): default gain 1.0 = plain sum
   (bit-identical to today); `set_param(0, g)` scales the sum by `g`.
3. **Poly `VoiceAllocator`** (voice.rs): `set_unison(3)`, note-on allocates 3
   DISTINCT lanes, each `SetParam` pitch = `note-69 + offset[u]` (the three
   offsets) + `gate_all(on)`; note-off releases ALL 3 (each `Releasing`); a 2nd
   note grabs 3 more; U=1 byte-identical (one lane, offset 0); release-tail: after
   releasing a unison note, a new note prefers Free lanes.
4. **Mono `MonoAllocator`** (voice.rs): `set_unison(2)`, from-silence → lanes 0
   and 1 each get pitch+offset + `TriggerVoice` + gate; legato → both lanes glide
   (pitch only, no gate); note-off → both gate off; U=1 byte-identical.
5. **Wren e2e** (`tests/audio_bindings.rs`): `Synth.new { |p| Osc.saw(p) *
   Env.adsr(…) }` + `s.unison = 4` + `s.detune = 12` builds + renders
   finite/bounded/non-silent; `Synth.mono` + unison builds + renders; `s.unison =
   1` behaves as before; the `VoiceSum` gain is set to 1/√4 ≈ 0.5 on
   `unison = 4` (assert via the emitted `SetParam` or a lower rendered level than
   an un-normalized 4× sum). Existing 1-voice synths unchanged.

## Success Criteria

- `Synth.new { |p| Osc.saw(p) * Env.adsr(…) }` with `s.unison = 7; s.detune = 15`
  — a fat, detuned, level-normalized poly voice (fewer simultaneous notes);
  `Synth.mono` + unison — a fat mono lead/bass.
- `synth.detune` spreads the U voices symmetrically; `synth.unison` sets the count
  (clamped 1..8) and the 1/√U normalization.
- `unison == 1` builds/renders byte-identically (poly and mono).
- Release-tail (5d) and mono legato-glide (5b) compose with unison.
- Both feature configs green, per-crate.
