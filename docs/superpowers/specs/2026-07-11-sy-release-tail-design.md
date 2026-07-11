# Sy-5d: Release-Tail Voice Protection — Design Spec

**Date:** 2026-07-11
**Suite:** Sy (synth/voice), sub-project Sy-5d (4th of the Sy-5 expressiveness suite; after 5a velocity, 5b mono-glide, 5c multi-env)
**Status:** Approved — ready for implementation plan

## Goal

Stop a new note from reusing a still-releasing voice's lane when a fresher or
unused lane is available, so long release tails aren't audibly cut. Purely
control-plane: `VoiceAllocator` gains a `Releasing` lane state and age-ordered
steal priority. No time source, no kernel feedback, no release-duration knowledge.

## Background

Today `VoiceAllocator::note_off` sets `lane_note[lane] = None` (fully free), and
`note_on` picks `free_lane().unwrap_or_else(oldest_lane)` — `free_lane` returns
the FIRST `None` lane. So a just-released lane (fresh, loud tail still ringing)
can be reused before an older-released (nearly-decayed) lane sitting later in the
array — cutting the louder tail. The allocator picks the wrong lane to reuse.

The `VoiceAllocator` is control-plane only, driven by note events; its only
"time" is a monotonic `clock` counter (increments per note event, not wall time).
There is NO kernel→control-plane feedback: `PolyAr`/`PolyAdsr` never report
"reached Idle". After Sy-5c there is no distinguished amp env (all envelopes are
equal), so the allocator knows neither which env's release matters nor its value.

The main benefit needs none of that: it's fixable by ordering the steal so a
new note prefers a never-used lane, then the *oldest*-released lane (most decayed),
before stealing any held note — using the existing event-clock as an **ordering**,
not absolute time.

## Scope (Sy-5d only)

A three-state lane model in `VoiceAllocator` with Free → oldest-Releasing →
oldest-Held allocation priority. `note_off` marks the lane `Releasing` (occupied)
instead of `Free`.

### Explicitly out of scope / deferred

- **Exact "never reuse a still-ringing lane when an idle one exists"** — needs a
  real time source or kernel per-voice Idle feedback (engine→allocator path +
  resolving which env is the amp). Deferred; the age-ordered heuristic delivers
  the practical benefit.
- **`MonoAllocator`** — one lane, note-stack driven, always reused; release-tail
  doesn't apply. Unchanged.
- **Unison (5e), Tb303/Modal poly filters, per-voice wavetable morph.**

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking (behavior):** the emit sequences for a note-on that allocates a
  Free lane, and for a note-off, are the SAME `Cmd`s as today (pitch `SetParam`,
  [velocity], `gate_all`) — only the LANE CHOICE changes, and one existing test
  (`note_off…frees_it`) updates to the new protective behavior (it encoded the
  bug). `MonoAllocator` and all kernels are byte-unchanged.
- **Test invocation (per-crate, never `--workspace`; both configs; cargo rejects
  multiple bare positional names — use `-- name1 name2`):**
  - Graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## Architecture

### §1 — The `LaneState` model (`crates/deluge-audio-graph/src/voice.rs`)

Replace `lane_note: [Option<u8>; VOICES]` with a three-state enum:

```rust
#[derive(Clone, Copy, PartialEq)]
enum LaneState {
    Free,       // never used (or never reclaimed — no time source to reclaim)
    Held(u8),   // sounding a held note (the u8 = MIDI note)
    Releasing,  // note-off fired, gate off, tail still ringing (lane occupied)
}
```

`VoiceAllocator` fields:
```rust
lane_state: [LaneState; VOICES],   // was lane_note
lane_age:   [u32; VOICES],          // clock at last transition: note-on (Held) / note-off (Releasing)
clock: u32,                          // increments on EVERY transition → total event order
```

Behavior (the `gate_all` fan-out from Sy-5c is unchanged — it gates all
configured envelopes):

- **`note_on(note, vel, emit)`** (`vel==0` → `note_off`):
  1. `lane = pick_lane()` — priority:
     - **Free:** first lane with `LaneState::Free`.
     - else **oldest Releasing:** the `Releasing` lane with the smallest `lane_age`
       (released longest ago = most decayed).
     - else **oldest Held:** the lane with the smallest `lane_age` (oldest allocation).
  2. `lane_state[lane] = Held(note)`, `lane_age[lane] = clock`, `clock =
     clock.wrapping_add(1)`.
  3. emit pitch `SetParam` + [velocity `SetParam`] + `gate_all(lane, true)`
     (re-gating a stolen/releasing lane re-attacks — same as today's steal).
- **`note_off(note, emit)`**: find the most-recent `Held(note)` lane (largest
  `lane_age` among lanes whose state is `Held(note)`); if found → `gate_all(lane,
  false)`, `lane_state[lane] = Releasing`, `lane_age[lane] = clock`, `clock =
  clock.wrapping_add(1)`. Unheld note → no-op.
- **`all_notes_off(emit)`**: for each `Held` lane → `gate_all(lane, false)`,
  `lane_state[lane] = Releasing`, `lane_age[lane] = clock`, `clock += 1`. (Leave
  already-`Releasing`/`Free` lanes untouched.)

`pick_lane` replaces `free_lane`/`oldest_lane`. Comparisons are always *within* a
state group (all Releasing, or all Held), so the single `lane_age` field (meaning
"clock at last transition") orders each group correctly.

## Data Flow

```
note_on: pick_lane (Free > oldest-Releasing > oldest-Held) → Held(note),
         SetParam(pitch) + [vel] + gate_all(on)
note_off: Held(note) → Releasing, gate_all(off)   (lane stays occupied)

Steal only happens when no Free and no Releasing lane exists (all 8 held).
A released lane is reused (oldest-first) before any held lane is stolen.
```

## Error Handling

No new failure modes. `pick_lane` always returns a lane (the final oldest-Held
fallback covers the all-held case). Bounded arrays, no panic, no heap.

## Testing

Both feature configs, `-p deluge-audio-graph`.

1. **Tail protection:** note A → lane 0; `note_off` A (lane 0 → `Releasing`); note
   B → a **Free** lane (1), NOT lane 0. (a fresh lane wins over a releasing one)
2. **Reuse oldest-releasing before stealing held:** fill all 8 (held); release
   lane 2 then lane 5 (both `Releasing`, 2 released first); a new note reuses lane
   **2** (oldest release); a second new note reuses lane **5** — no held lane
   stolen while releasing lanes remain.
3. **Steal held only when forced:** 8 held, none releasing → new note steals
   oldest-held (lane 0) — unchanged from today (regression).
4. **Re-gate on reuse:** reusing a `Releasing` lane emits `SetParam(pitch)` +
   `gate_all(on)` (re-attack) — same emit shape as a fresh allocation.
5. **`all_notes_off`** marks all held lanes `Releasing` and gates all off.
6. **Update** the existing `note_off_releases_the_right_lane_and_frees_it` test to
   the new protective expectation (next note goes to a Free lane, not the released
   one). Confirm the rest of the allocator suite (steal, velocity, `all_notes_off`,
   the Sy-5c gate fan-out) unchanged, and `MonoAllocator` untouched.

## Success Criteria

- After releasing a note, a new note prefers a fresh/unused lane, leaving the
  released voice's tail to ring out.
- When polyphony is exceeded, the *oldest*-released lane is reused before any
  held note is stolen, and a fresher release tail is preferred over an older one.
- 8-held-notes steal behavior unchanged; `note_off`/`all_notes_off` emit the same
  gate `Cmd`s (only lane choice / lane state changed).
- `MonoAllocator` and all kernels byte-unchanged.
- Both feature configs green.
