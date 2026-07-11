# Sy-5d: Release-Tail Voice Protection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop a new note from reusing a still-releasing voice's lane when a fresher/unused lane exists, so long release tails aren't audibly cut — a control-plane-only change to `VoiceAllocator`.

**Architecture:** Replace `VoiceAllocator`'s two-state `lane_note: [Option<u8>]` with a three-state `LaneState { Free, Held(u8), Releasing }`; `note_off` marks the lane `Releasing` (occupied) instead of freeing it; allocation picks Free → oldest-Releasing → oldest-Held using the existing event-clock as ordering. No time source, no kernel feedback.

**Tech Stack:** Rust `no_std`, `deluge-audio-graph`.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking (behavior):** the `Cmd` emit sequences for a Free-lane note-on and for a note-off are the SAME as today (pitch `SetParam`, [velocity], `gate_all`) — only the LANE CHOICE and lane STATE change. One existing test (`note_off_releases_the_right_lane_and_frees_it`) updates to the new protective behavior (it encoded the bug). `MonoAllocator` and all kernels are byte-unchanged.
- **Test invocation (per-crate, never `--workspace`; both configs; cargo rejects multiple bare positional names — use `-- name1 name2`):**
  - `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## File Structure

- `crates/deluge-audio-graph/src/voice.rs` — `LaneState` enum + `VoiceAllocator` state model + `pick_lane`. This is the only file changed.

---

## Task 1: `VoiceAllocator` release-tail lane model

**Files:**
- Modify: `crates/deluge-audio-graph/src/voice.rs` (`VoiceAllocator` struct + `new`/`note_on`/`note_off`/`all_notes_off`; replace `free_lane`/`oldest_lane` with `pick_lane`)
- Test: `voice.rs` test module

**Interfaces:**
- Consumes: `Cmd::{SetParam, GateVoice}`, `NodeId`, `VOICES`, `A440_NOTE`, `MAX_GATES`, the existing `gate_all(lane, on, emit)` method (Sy-5c fan-out).
- Produces: `enum LaneState { Free, Held(u8), Releasing }`; `VoiceAllocator` with `lane_state: [LaneState; VOICES]` (replaces `lane_note`); `pick_lane(&self) -> usize`. Public `new`/`note_on`/`note_off`/`all_notes_off` signatures UNCHANGED.

- [ ] **Step 1: Write the failing tests**

Add to the voice.rs test module (mirror the existing `on`/`off` capture helpers — `on(&mut a, note, vel) -> Vec<Cmd>`, `off(&mut a, note) -> Vec<Cmd>`; and the way tests read `Cmd::SetParam { param, .. }` where `param` is the lane):

```rust
#[test]
fn released_lane_is_protected_new_note_takes_a_free_lane() {
    let mut a = mk(); // helper builds a VoiceAllocator with pitch=NodeId(10), 1 gate=NodeId(20), no vel
    on(&mut a, 60, 100);        // A → lane 0
    off(&mut a, 60);            // lane 0 → Releasing (tail ringing)
    let c = on(&mut a, 64, 100); // B → should take a FREE lane (1), NOT reuse lane 0
    match c[0] {
        Cmd::SetParam { param, .. } => assert_eq!(param, 1, "new note protects lane 0's tail → lane 1"),
        _ => panic!("expected pitch SetParam"),
    }
}

#[test]
fn reuse_oldest_releasing_before_stealing_held() {
    let mut a = mk();
    for k in 0..VOICES { on(&mut a, 60 + k as u8, 100); } // lanes 0..7 all Held
    off(&mut a, 62); // lane 2 → Releasing (released first)
    off(&mut a, 65); // lane 5 → Releasing (released second)
    // new note reuses the OLDEST releasing lane (2), not a held lane, not lane 5
    let c1 = on(&mut a, 80, 100);
    match c1[0] { Cmd::SetParam { param, .. } => assert_eq!(param, 2, "oldest releasing"), _ => panic!() }
    // next new note reuses the remaining releasing lane (5)
    let c2 = on(&mut a, 81, 100);
    match c2[0] { Cmd::SetParam { param, .. } => assert_eq!(param, 5, "next releasing"), _ => panic!() }
}

#[test]
fn all_held_steals_oldest_held_unchanged() {
    let mut a = mk();
    for k in 0..VOICES { on(&mut a, 60 + k as u8, 100); } // lanes 0..7 Held, lane 0 oldest
    let c = on(&mut a, 72, 100); // no free/releasing → steal oldest held (lane 0)
    match c[0] { Cmd::SetParam { param, .. } => assert_eq!(param, 0, "steal oldest held"), _ => panic!() }
    assert!(c.iter().any(|cmd| matches!(cmd, Cmd::GateVoice { voice: 0, on: true, .. })));
}

#[test]
fn reusing_a_releasing_lane_re_gates_it() {
    let mut a = mk();
    on(&mut a, 60, 100); // lane 0
    off(&mut a, 60);     // lane 0 Releasing
    // fill lanes 1..7 so the next note has no Free lane → must reuse releasing lane 0
    for k in 1..VOICES { on(&mut a, 61 + k as u8, 100); }
    let c = on(&mut a, 90, 100); // only lane 0 is Releasing (rest Held) → reuse lane 0, re-attack
    assert!(matches!(c[0], Cmd::SetParam { param: 0, .. }));
    assert!(c.iter().any(|cmd| matches!(cmd, Cmd::GateVoice { voice: 0, on: true, .. })), "re-gate on reuse");
}

#[test]
fn all_notes_off_marks_releasing_and_gates_off() {
    let mut a = mk();
    on(&mut a, 60, 100);
    on(&mut a, 64, 100);
    let mut c: Vec<Cmd> = Vec::new();
    { let mut e = |x: Cmd| c.push(x); a.all_notes_off(&mut e); }
    assert_eq!(c.len(), 2); // gate-off for the two held lanes
    assert!(c.iter().all(|cmd| matches!(cmd, Cmd::GateVoice { on: false, .. })));
    // after all-notes-off, the two lanes are Releasing (occupied), so a new note
    // still prefers the remaining Free lanes:
    let c2 = on(&mut a, 67, 100);
    match c2[0] { Cmd::SetParam { param, .. } => assert!(param >= 2, "new note avoids the two releasing lanes"), _ => panic!() }
}
```

Add a small `mk()` helper in the test module that builds a 1-gate `VoiceAllocator` (mirror the existing tests' construction — e.g. `{ let mut g = [NodeId(0); MAX_GATES]; g[0] = NodeId(20); VoiceAllocator::new(NodeId(10), g, 1, None) }`).

Also UPDATE the existing `note_off_releases_the_right_lane_and_frees_it` test: it currently asserts that after `off(60)` (lane 0) the next `on(67)` REUSES lane 0 (`param: 0`). Under release-tail protection the next note takes a FREE lane (lane 2, since lanes 0 and 1 were used) — change that final assertion from `param: 0` to `param: 2` (or assert the released lane 0 is NOT reused while free lanes exist). Keep the earlier assertion (the note-off still emits `GateVoice { voice: 0, on: false }`).

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- released_lane_is_protected reuse_oldest_releasing all_held_steals reusing_a_releasing all_notes_off_marks`
Expected: the new tests FAIL (`LaneState` / `mk` not defined, or the current allocator reuses lane 0 so `released_lane_is_protected` fails asserting `param==1`).

- [ ] **Step 3: Add `LaneState` + rework the fields**

Add the enum (near the top of voice.rs, after the imports):

```rust
/// Per-lane lifecycle for release-tail-aware allocation.
#[derive(Clone, Copy, PartialEq)]
enum LaneState {
    Free,      // never used (never reclaimed — the allocator has no time source)
    Held(u8),  // sounding a held note (the MIDI note)
    Releasing, // note-off fired, gate off, tail still ringing — lane stays occupied
}
```

Change the `VoiceAllocator` struct: replace `lane_note: [Option<u8>; VOICES]` with `lane_state: [LaneState; VOICES]` (keep `pitch_node`, `gates`, `n_gates`, `vel_node`, `lane_age`, `clock`). In `new`, init `lane_state: [LaneState::Free; VOICES]` (keep the rest).

- [ ] **Step 4: `pick_lane` + reworked note handlers**

Replace `free_lane`/`oldest_lane` with `pick_lane`:

```rust
/// Choose a lane for a new note: a never-used lane first, else the lane released
/// longest ago (most decayed), else steal the oldest-allocated held lane.
fn pick_lane(&self) -> usize {
    // 1. a Free lane
    if let Some(v) = (0..VOICES).find(|&v| self.lane_state[v] == LaneState::Free) {
        return v;
    }
    // 2. the oldest Releasing lane (min lane_age among Releasing)
    let mut best: Option<usize> = None;
    for v in 0..VOICES {
        if self.lane_state[v] == LaneState::Releasing
            && best.map_or(true, |b| self.lane_age[v] < self.lane_age[b])
        {
            best = Some(v);
        }
    }
    if let Some(v) = best {
        return v;
    }
    // 3. all Held → steal the oldest-allocated (min lane_age overall)
    let mut best = 0;
    for v in 1..VOICES {
        if self.lane_age[v] < self.lane_age[best] {
            best = v;
        }
    }
    best
}
```

`note_on` (pitch/velocity/gate emit UNCHANGED — only the lane pick + state write change):

```rust
pub fn note_on(&mut self, note: u8, vel: u8, emit: &mut impl FnMut(Cmd)) {
    if vel == 0 {
        self.note_off(note, emit);
        return;
    }
    let lane = self.pick_lane();
    self.lane_state[lane] = LaneState::Held(note);
    self.lane_age[lane] = self.clock;
    self.clock = self.clock.wrapping_add(1);
    emit(Cmd::SetParam { node: self.pitch_node, param: lane as u8, value: note as f32 - A440_NOTE });
    if let Some(vn) = self.vel_node {
        emit(Cmd::SetParam { node: vn, param: lane as u8, value: vel as f32 / 127.0 });
    }
    self.gate_all(lane, true, emit);
}
```

`note_off` — mark `Releasing`, not free (find the most-recent `Held(note)`):

```rust
pub fn note_off(&mut self, note: u8, emit: &mut impl FnMut(Cmd)) {
    let mut best: Option<usize> = None;
    for v in 0..VOICES {
        if self.lane_state[v] == LaneState::Held(note)
            && best.map_or(true, |b| self.lane_age[v] > self.lane_age[b])
        {
            best = Some(v);
        }
    }
    if let Some(lane) = best {
        self.gate_all(lane, false, emit);
        self.lane_state[lane] = LaneState::Releasing;
        self.lane_age[lane] = self.clock;
        self.clock = self.clock.wrapping_add(1);
    }
    // Unheld note → no-op.
}
```

`all_notes_off` — each Held → Releasing:

```rust
pub fn all_notes_off(&mut self, emit: &mut impl FnMut(Cmd)) {
    for v in 0..VOICES {
        if matches!(self.lane_state[v], LaneState::Held(_)) {
            self.gate_all(v, false, emit);
            self.lane_state[v] = LaneState::Releasing;
            self.lane_age[v] = self.clock;
            self.clock = self.clock.wrapping_add(1);
        }
    }
}
```

> `LaneState::Held(note) == self.lane_state[v]` uses `PartialEq` on the enum (derived) — comparing `Held(60)` matches only a lane holding note 60. `matches!(…, LaneState::Held(_))` is the any-held check for `all_notes_off`. `pick_lane`'s step-3 fallback (`min lane_age`) is only reached when every lane is `Held` (no Free, no Releasing), giving oldest-held steal — identical to the old `oldest_lane`.

- [ ] **Step 5: Run to verify they pass + full regression**

Run the new + updated tests both configs, then the full graph crate both configs:
`cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph` and `--features deluge-dsp-kernels/simd`.
Expected: PASS both — new release-tail tests green; the updated `note_off…frees_it` test green; all other allocator tests (velocity fan-out, steal, `MonoAllocator`) unchanged.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-audio-graph/src/voice.rs
git commit -m "feat(graph): release-tail voice protection — Releasing lane state, Free>oldest-Releasing>oldest-Held"
```

---

## Self-Review Notes (for the executor)

- **Non-breaking emits:** a Free-lane note-on and a note-off emit the SAME `Cmd`s as before (pitch `SetParam`, [velocity], `gate_all`). Only the lane choice and the lane STATE (Releasing vs freed) changed. The `MonoAllocator` block is NOT touched.
- **The one flipped test** (`note_off_releases_the_right_lane_and_frees_it`) is intentional — its old assertion (next note reuses the just-released lane) was the bug this fixes. Update it to expect a Free lane; keep the gate-off assertion.
- **`clock` increments on BOTH note_on and note_off** so release order is totally ordered (two releases with no intervening note-on get distinct ages → `oldest-Releasing` is unambiguous). Held lanes keep their note-on age.
- **`pick_lane` step 3** must only be reached when all lanes are `Held` — steps 1 (Free) and 2 (Releasing) return first. When all held, min-`lane_age` = oldest allocation = the old `oldest_lane` behavior.
- **No new failure modes:** `pick_lane` always returns (step 3 falls through to a lane index); bounded arrays; no heap; no panic.
- **Deferred (do NOT implement):** exact kernel per-voice Idle feedback, unison (5e), Tb303/Modal poly filters, per-voice wavetable morph.
