# Sy-6b: Live Re-Pan / Re-Pitch — Design Spec

**Date:** 2026-07-11
**Suite:** Sy (synth/voice), sub-project Sy-6b — second of the Sy-6 spatial /
voice-management suite (after 6a stereo spread; the Sy-5 expressiveness suite is
complete).
**Status:** Approved — ready for implementation plan

## Goal

`synth.detune = cents` and `synth.width = amount` take effect on **currently-
sounding** notes, not just the next note-on. Turn the knob and hear each held
note's unison spread re-detune and re-pan in real time. Poly (`Synth.new`) and
mono (`Synth.mono`).

## Background

Unison (Sy-5e) writes `note-69 + unison_offset(u,U,detune)` to each voice's pitch
lane at note-on; stereo spread (Sy-6a) writes `width_offset(u,U,width)` to each
voice's pan slot on the `StereoVoiceSum` at note-on. Both are **next-note**: the
allocator emits once, at `note_on`, then `set_detune`/`set_width` only store the
new value for future notes — a held note keeps its play-time spread.

Making them live means: when `set_detune`/`set_width` is called, re-emit the
per-lane pitch/pan `SetParam`s for the voices that are currently sounding. Two
things are missing today:

1. **Per-voice group context.** The offset for a voice is `f(u, U, value)`, so
   re-emitting needs each sounding voice's `u`-index and its group size `U`.
   `LaneState::Held(u8)` (Sy-5d) stores only the MIDI note.
2. **An emit path in the setters.** `set_detune`/`set_width` currently just store
   a field (`crates/deluge-audio-graph/src/voice.rs`) with no `emit` sink.

The pitch/pan mechanism itself already supports per-lane live updates
(`PolyCtrl::set_voice` via `SetParam{param=lane}`; `StereoVoiceSum` pan via
`set_param(lane+1)`), so no kernel or node change is needed — this is a
control-plane (allocator + Wren) sub-project.

## Scope (Sy-6b)

Live re-emit of **detune and width** on sounding voices, in both allocators, via
an `emit`-taking `set_detune`/`set_width`, backed by per-lane group memory.

### Explicitly out of scope / deferred

- **Live unison-COUNT change** — `synth.unison = N` re-voicing a sounding note
  (gating new voices on / removed ones off mid-note) is re-voicing, not
  re-parameterization: new voices would attack mid-note, and the steal choice is
  ambiguous. Stays next-note. (Its `1/√U` amplitude gain already updates live via
  the shared `set_param(0)` slot — that is unchanged.)
- **Poly pitch/pan smoothing** — poly re-emit is an instant jump (poly pitch has
  no slew node; pan is an instant per-lane gain), so a *fast* knob sweep could
  zipper. Accepted at knob speed; a dedicated poly-smoothing pass (adding a slew
  to the poly pitch path) is a later sub-project. Mono re-pitch already glides
  through `PolySlew`.
- **Re-centering reused Free/Releasing lanes on `width→0`** — live `set_width`
  re-centers currently-*Held* voices, but a lane that was released (or freed) then
  reused by a new note at `width==0` still inherits stale pan (the Sy-6a wart, for
  new notes). Note-on's `width!=0` guard is unchanged. Deferred.
- **Live glide, Tb303/Modal poly filters, f32x8 ADSR, Bus poly, MPE.**

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** a synth that never calls `synth.detune`/`synth.width` is
  unaffected. Calling either setter with **no sounding voices** (the normal
  `s.detune = …` / `s.width = …` before `noteOn`) re-emits nothing, so every
  note-on Cmd stream — and the Sy-6a `width==0` byte-identical render proof — is
  preserved. Existing allocator tests pass with only the mechanical
  setter-signature (`emit` param) threaded through their call sites.
- **Two registration tables:** NO new foreign is added — `unison=`/`detune=`/
  `width=` are already registered (Sy-5e/6a). Only the Rust setter impls change.
- **Test invocation (per-crate, never `--workspace`; both configs; `-- name1 name2`):**
  - Graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## Architecture

### §1 — Per-lane group memory

**Poly `VoiceAllocator`** — add a parallel array:

```rust
lane_ctx: [(u8, u8); VOICES],   // (u-index, group size U) for each Held lane
```

Written in `note_on`'s `for u in 0..u_count` loop: `self.lane_ctx[lane] = (u as u8,
u_count as u8);` alongside `lane_state[lane] = Held(note)`. Kept as a parallel
array (not by extending `LaneState::Held`) so the proven Sy-5d/5e allocation and
steal logic (`pick_lane`, the `== Held(note)` comparisons) is untouched. `lane_ctx`
is only read for lanes whose state is `Held`.

**Mono `MonoAllocator`** — add one field:

```rust
mono_active_u: usize,   // the sounding note's U-at-play (unison is next-note)
```

Set in `note_on` to `u_count = self.unison.min(VOICES)`. Mono's sounding voices are
always lanes `0..mono_active_u` on the top-of-stack note, so no per-lane array is
needed; `u`-index == lane index, `U == mono_active_u`.

### §2 — `set_detune` / `set_width` re-emit

Both setters gain an `emit: &mut impl FnMut(Cmd)` parameter. They store the new
value, then re-emit for the currently-sounding voices.

**Poly:**

```rust
pub fn set_detune(&mut self, cents: f32, emit: &mut impl FnMut(Cmd)) {
    self.detune_cents = cents;
    for lane in 0..VOICES {
        if let LaneState::Held(note) = self.lane_state[lane] {
            let (u, count) = self.lane_ctx[lane];
            let value = note as f32 - A440_NOTE
                + unison_offset(u as usize, count as usize, cents);
            emit(Cmd::SetParam { node: self.pitch_node, param: lane as u8, value });
        }
    }
}

pub fn set_width(&mut self, amount: f32, emit: &mut impl FnMut(Cmd)) {
    self.width_amount = amount;
    for lane in 0..VOICES {
        if let LaneState::Held(_) = self.lane_state[lane] {
            let (u, count) = self.lane_ctx[lane];
            emit(Cmd::SetParam {
                node: self.sum_node,
                param: (lane + 1) as u8,
                value: width_offset(u as usize, count as usize, amount),
            });
        }
    }
}
```

`set_width` re-emits pan even when `amount == 0.0`, so sounding voices re-center on
`width→0`. (Note-on's `width!=0` emit guard is unchanged; the two paths differ by
design.) Only `Held` lanes are touched — `Releasing` tails keep their frozen
spread.

**Mono** — re-emit for lanes `0..mono_active_u` on the top note when a note sounds:

```rust
pub fn set_detune(&mut self, cents: f32, emit: &mut impl FnMut(Cmd)) {
    self.detune_cents = cents;
    if self.len > 0 {
        let top = self.notes[self.len - 1];
        let u_count = self.mono_active_u;
        for u in 0..u_count {
            let value = top as f32 - A440_NOTE + unison_offset(u, u_count, cents);
            emit(Cmd::SetParam { node: self.pitch_node, param: u as u8, value });
        }
    }
}
// set_width: same loop, emit SetParam(sum_node, u+1, width_offset(u, u_count, amount))
```

Mono pitch flows through `PolySlew`, so a live detune change **glides**; mono pan
(and all poly re-emit) is instant.

### §3 — Wren surface

No new foreign, no new registration. Only the impls change to pass a host emit
sink (like `synth_note_on`/`synth_note_off` already do):

- `synth_set_detune_impl` / `synth_set_width_impl`:
  `self_synth(vm).alloc.set_detune(cents, &mut |c| crate::host::host().audio_cmd(c))`
  (respectively `set_width`).
- `SynthAlloc::set_detune` / `set_width` dispatch gains the `emit` param, forwarding
  to `Poly(a)` / `Mono(m)`.

(`synth.unison`'s impl is unchanged — its live `set_param(out_node, 0, 1/√N)` gain
already applies immediately; only the voice count stays next-note.)

## Data Flow

```
synth.detune = c → SynthAlloc.set_detune(c, emit)
    poly: for each Held lane: SetParam(pitch, lane, note-69 + unison_offset(u,U,c))
    mono: for u in 0..active_u: SetParam(pitch, u, top-69 + unison_offset(u,U,c))  → glides via PolySlew

synth.width = a → SynthAlloc.set_width(a, emit)
    poly: for each Held lane: SetParam(sum_node, lane+1, width_offset(u,U,a))
    mono: for u in 0..active_u: SetParam(sum_node, u+1, width_offset(u,U,a))

(no sounding voice ⇒ the re-emit loop finds no Held lane ⇒ emits nothing)
```

## Error Handling

Total setters; no clamp needed at the boundary (`unison_offset`/`width_offset`
guard `count<=1`, the pan law clamps). No new panics, no heap. Bounded loops over
`VOICES` / `mono_active_u <= VOICES`. `(lane+1) as u8` and `u as u8` are safe
(`< VOICES == 8`). A setter with no sounding voice is a no-op emit.

## Testing

Both feature configs, per-crate.

1. **Poly live re-detune** (voice.rs): `set_unison(3)`, `note_on(69,100)` (3 Held
   lanes), then `set_detune(20.0, emit)` → 3 pitch `SetParam`s on the 3 Held lanes
   with the new spread (`unison_offset(u,3,20)`); a subsequent `set_detune` with
   **no note** (after `note_off`) emits nothing.
2. **Poly live re-width** (voice.rs): `set_unison(3)` + `note_on`, then
   `set_width(1.0, emit)` → 3 pan `SetParam`s on `sum_node` (params lane+1);
   `set_width(0.0, emit)` afterward re-emits 3 pan-0 `SetParam`s (re-center); no
   sounding note ⇒ none.
3. **Poly ctx correctness** (voice.rs): two notes, unison=2 each (4 Held lanes);
   `set_detune` re-emits 4 pitch `SetParam`s, each lane's value using ITS own
   `(u,U)` and ITS own note — not cross-contaminated.
4. **Mono live re-detune/width** (voice.rs): `set_unison(2)` + `note_on` (lanes
   0,1), `set_detune`/`set_width` re-emit on lanes 0,1 for the top note; after
   note-off to silence, they emit nothing; `mono_active_u` reflects play-time U
   even if `set_unison` changed after the note started.
5. **Non-breaking** (voice.rs): calling `set_detune`/`set_width` before any
   `note_on` emits nothing (the pre-note `s.detune = …` case); existing allocator
   tests pass with only the mechanical `emit`-arg threading.
6. **Wren e2e** (`tests/audio_bindings.rs`): `Synth.new`+`unison=4`, `noteOn`,
   render buffer A; then `s.detune = 30` (or `s.width = 1`), render buffer B →
   B differs from A (the sounding note actually moved: pitch shifted / L≠R spread
   widened). Same for `Synth.mono`. Setting the value BEFORE `noteOn` reproduces
   today's render (non-breaking).

## Success Criteria

- With a note held, `synth.detune = …` audibly re-detunes its unison voices and
  `synth.width = …` re-spreads them across the stereo field — live, no re-trigger.
- Each sounding voice re-computes from its own `(u, U, note)`; multiple held notes
  don't cross-contaminate.
- Mono live re-detune glides (via `PolySlew`); poly is an instant (accepted) jump.
- A setter call with no sounding voice, and any synth that never calls the
  setters, render byte-identically to today.
- Both feature configs green, per-crate.
