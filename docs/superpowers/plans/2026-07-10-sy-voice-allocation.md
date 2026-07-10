# Sy-3: Voice Allocation & Note Handling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the merged poly voice channel into a note-driven synth — a `PolyMtof` semitone→Hz node and a `VoiceAllocator` that maps note-on/off onto the 8 voice lanes (oldest-LRU steal), emitting pitch + gate `Cmd`s.

**Architecture:** `PolyMtof` is the 8-lane version of `Mtof`, reusing a `semitones_to_hz` helper extracted from it. `VoiceAllocator` is a control-plane struct in `deluge-audio-graph` holding the voice's `PolyCtrl` (pitch) + `PolyAr` (gate) node ids; its `note_on`/`note_off` emit `Cmd`s through a closure sink (`&mut impl FnMut(Cmd)`), decoupled from `Engine`'s const generics.

**Tech Stack:** Rust `no_std` (`deluge-dsp-kernels`), audio graph/engine (`deluge-audio-graph`), `libm`.

## Global Constraints

- `no_std`, no heap, pure `f32`, deterministic. Poly tiles voice-interleaved.
- Host tests per-crate on x86: `cargo test --target x86_64-unknown-linux-gnu -p <crate>`; the engine test also `--features deluge-dsp-kernels/simd`. Never `--workspace` on host.
- LSP armv7a "can't find crate for test/std" diagnostics are noise — ignore.
- The `Mtof` refactor (extract `semitones_to_hz`) is **behavior-preserving** — the existing `quant` tests pass verbatim.
- No Wren surface (Sy-4). `VoiceAllocator` never panics, never allocates. Tuning: `note − 69` semitones above A440 (MIDI convention); `PolyMtof.ref_hz` default 440.
- Poly port convention (Sy-2): first `poly_in_count(kind)` ports are poly. `PolyMtof` → `poly_in_count = 1`, `out_width = VOICES`, `param 0 = ref_hz`.

---

### Task 1: `PolyMtof` — `semitones_to_hz` extraction + poly kernel + graph node

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/quant.rs` (extract `semitones_to_hz`, refactor `Mtof`)
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (add `PolyMtof`)
- Modify: `crates/deluge-audio-graph/src/node.rs` (graph node)

**Interfaces:**
- Consumes: `crate::In`, `libm::exp2f`, `poly::VOICES`, the Sy-2 `poly_process(ins, [Option;2], dt, out)` shape.
- Produces: `pub(crate) fn semitones_to_hz(st: f32, ref_hz: f32) -> f32`; `PolyMtof` with `new()`, `set_ref(hz)`, `process(semitones: &[f32], out: &mut [f32])`; `Kind::PolyMtof` (poly_in_count 1, out_width VOICES, param 0 = ref).

- [ ] **Step 1: Extract `semitones_to_hz` and refactor `Mtof` (quant.rs)**

Just above `pub struct Mtof` in `quant.rs`, add:

```rust
/// Semitones above `ref_hz` → Hz: `ref_hz · 2^(st/12)`.
#[inline]
pub(crate) fn semitones_to_hz(st: f32, ref_hz: f32) -> f32 {
    ref_hz * libm::exp2f(st / 12.0)
}
```

Change `Mtof::process`'s body to use it (behavior-preserving):

```rust
    pub fn process(&mut self, input: In, out: &mut [f32]) {
        for i in 0..out.len() {
            out[i] = semitones_to_hz(input.at(i), self.ref_hz);
        }
    }
```

- [ ] **Step 2: Verify the refactor is behavior-preserving**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels quant`
Expected: PASS — existing `Mtof` tests (`mtof_octaves_exact`, `mtof_set_ref`) unchanged and passing.

- [ ] **Step 3: Add `PolyMtof` to `poly.rs`**

Add `use crate::quant::semitones_to_hz;` (next to the existing uses). Then:

```rust
/// Poly semitone→Hz: `out[v] = ref_hz · 2^(semitone[v]/12)`. Poly-in note-offset
/// lanes → Hz lanes. Scalar (pitch is control-rate, not a recurrent kernel).
#[derive(Clone, Copy)]
pub struct PolyMtof {
    ref_hz: f32,
}
impl PolyMtof {
    pub fn new() -> PolyMtof {
        PolyMtof { ref_hz: 440.0 }
    }
    pub fn set_ref(&mut self, hz: f32) {
        self.ref_hz = hz;
    }
    /// `semitones` = voice-interleaved note-offset tile; writes an Hz tile
    /// (element-wise, so the interleave is preserved trivially).
    pub fn process(&mut self, semitones: &[f32], out: &mut [f32]) {
        for j in 0..out.len() {
            out[j] = semitones_to_hz(semitones[j], self.ref_hz);
        }
    }
}
impl Default for PolyMtof {
    fn default() -> Self { Self::new() }
}
```

- [ ] **Step 4: Add `PolyMtof` kernel tests (poly.rs)**

```rust
    #[test]
    fn polymtof_maps_semitones_to_hz() {
        let mut m = PolyMtof::new(); // ref 440
        let offsets = [0.0f32, 12.0, -12.0, 7.0, -9.0, 24.0, 1.0, -1.0];
        let n = 3;
        let mut inp = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                inp[i * VOICES + v] = offsets[v];
            }
        }
        let mut out = std::vec![0.0f32; VOICES * n];
        m.process(&inp, &mut out);
        assert!((out[0] - 440.0).abs() < 1e-2, "0 → 440: {}", out[0]);
        assert!((out[1] - 880.0).abs() < 1e-2, "+12 → 880: {}", out[1]);
        assert!((out[2] - 220.0).abs() < 1e-2, "-12 → 220: {}", out[2]);
        assert!(out.iter().all(|&h| h.is_finite() && h > 0.0));
    }

    #[test]
    fn polymtof_set_ref_retunes() {
        let mut m = PolyMtof::new();
        m.set_ref(100.0);
        let mut out = std::vec![0.0f32; VOICES];
        m.process(&std::vec![0.0f32; VOICES], &mut out);
        assert!(out.iter().all(|&h| (h - 100.0).abs() < 1e-3));
    }
```

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels poly`
Expected: PASS (both new tests + existing).

- [ ] **Step 5: Add the graph node (node.rs)**

Extend the `use deluge_dsp_kernels::{ ... poly::{...} }` line to include `PolyMtof`.

`enum Kind`: after `PolyMul,` add `PolyMtof,`. `enum State`: after `PolySvf(PolySvf),` add `PolyMtof(PolyMtof),`.

`Node::new`: add `Kind::PolyMtof => State::PolyMtof(PolyMtof::new()),`.

`out_width`: append `| Kind::PolyMtof` to the poly `=> VOICES,` arm.

`is_poly`: append `| Kind::PolyMtof` to the `matches!(...)`.

`poly_in_count`: append `| Kind::PolyMtof` to the `=> 1` arm (with `PolyOsc | PolySvf | VoiceSum`).

`set_param`: add
```rust
            State::PolyMtof(m) => match param {
                0 => m.set_ref(value),
                _ => {}
            },
```

`poly_process`: add
```rust
            Kind::PolyMtof => {
                if let (State::PolyMtof(m), Some(pin)) = (&mut self.state, poly_in[0]) {
                    m.process(pin, out);
                }
            }
```

`process_resolved` poly no-op arm: append `| Kind::PolyMtof` to the poly-kinds `=> {}` arm.

- [ ] **Step 6: Add a graph node test (node.rs)**

```rust
    #[test]
    fn polymtof_node_converts_semitones() {
        assert_eq!(Node::out_width(Kind::PolyMtof), VOICES);
        assert_eq!(Node::poly_in_count(Kind::PolyMtof), 1);
        let mut n = Node::new(Kind::PolyMtof, 0);
        n.set_param(0, 440.0);
        let mut semis = [0.0f32; VOICES * 2];
        for i in 0..2 {
            semis[i * VOICES] = 0.0; // voice 0 → 440
            semis[i * VOICES + 1] = 12.0; // voice 1 → 880
        }
        let dummy = [In::K(0.0); MAX_INPUTS];
        let mut out = [0.0f32; VOICES * 2];
        n.poly_process(&dummy, [Some(&semis), None], 1.0 / 48_000.0, &mut out);
        assert!((out[0] - 440.0).abs() < 1e-2 && (out[1] - 880.0).abs() < 1e-2);
    }
```

- [ ] **Step 7: Run the tests (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -p deluge-audio-graph`
Expected: PASS — kernel + node tests. No warnings.
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features deluge-dsp-kernels/simd`
Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add crates/deluge-dsp-kernels/src/quant.rs crates/deluge-dsp-kernels/src/poly.rs crates/deluge-audio-graph/src/node.rs
git commit -m "feat: PolyMtof poly semitone→Hz node (semitones_to_hz extracted from Mtof)"
```

---

### Task 2: `VoiceAllocator` controller

**Files:**
- Create: `crates/deluge-audio-graph/src/voice.rs`
- Modify: `crates/deluge-audio-graph/src/lib.rs` (`pub mod voice;` + re-export)

**Interfaces:**
- Consumes: `crate::cmd::Cmd`, `crate::NodeId`, `deluge_dsp_kernels::poly::VOICES`.
- Produces: `VoiceAllocator` with `new(pitch_node: NodeId, gate_node: NodeId)`, `note_on(&mut self, note: u8, vel: u8, emit: &mut impl FnMut(Cmd))`, `note_off(&mut self, note: u8, emit: &mut impl FnMut(Cmd))`, `all_notes_off(&mut self, emit: &mut impl FnMut(Cmd))`. Re-exported as `deluge_audio_graph::VoiceAllocator`.

- [ ] **Step 1: Create `voice.rs`**

```rust
//! Note→voice allocation: maps note-on/off events onto the `VOICES` poly lanes,
//! emitting the low-level `Cmd`s (pitch `SetParam` on a `PolyCtrl` + per-voice
//! `GateVoice` on a `PolyAr`). Oldest-LRU voice stealing. Control-plane only —
//! no audio, no heap, no panics.

use crate::cmd::Cmd;
use crate::NodeId;
use deluge_dsp_kernels::poly::VOICES;

/// MIDI note of the `PolyMtof` reference (A4 = 440 Hz). The allocator writes
/// `note - A440_NOTE` (semitones above the reference) to the pitch `PolyCtrl`.
const A440_NOTE: f32 = 69.0;

pub struct VoiceAllocator {
    pitch_node: NodeId, // a PolyCtrl: SetParam(pitch_node, lane, note - 69)
    gate_node: NodeId,  // a PolyAr:   GateVoice(gate_node, lane, on/off)
    lane_note: [Option<u8>; VOICES],
    lane_age: [u32; VOICES],
    clock: u32,
}

impl VoiceAllocator {
    pub fn new(pitch_node: NodeId, gate_node: NodeId) -> VoiceAllocator {
        VoiceAllocator {
            pitch_node,
            gate_node,
            lane_note: [None; VOICES],
            lane_age: [0; VOICES],
            clock: 0,
        }
    }

    pub fn note_on(&mut self, note: u8, vel: u8, emit: &mut impl FnMut(Cmd)) {
        if vel == 0 {
            self.note_off(note, emit); // MIDI: note-on vel 0 == note-off
            return;
        }
        // A free lane, else steal the oldest (lowest age). Reassigning +
        // GateVoice(on) re-attacks the stolen lane — no separate gate-off needed.
        let lane = self.free_lane().unwrap_or_else(|| self.oldest_lane());
        self.lane_note[lane] = Some(note);
        self.lane_age[lane] = self.clock;
        self.clock = self.clock.wrapping_add(1);
        emit(Cmd::SetParam {
            node: self.pitch_node,
            param: lane as u8,
            value: note as f32 - A440_NOTE,
        });
        emit(Cmd::GateVoice { node: self.gate_node, voice: lane as u8, on: true });
    }

    pub fn note_off(&mut self, note: u8, emit: &mut impl FnMut(Cmd)) {
        // Release the most-recently-allocated lane playing `note`.
        let mut best: Option<usize> = None;
        for v in 0..VOICES {
            if self.lane_note[v] == Some(note)
                && best.map_or(true, |b| self.lane_age[v] > self.lane_age[b])
            {
                best = Some(v);
            }
        }
        if let Some(lane) = best {
            emit(Cmd::GateVoice { node: self.gate_node, voice: lane as u8, on: false });
            self.lane_note[lane] = None;
        }
        // Unheld note → no-op.
    }

    pub fn all_notes_off(&mut self, emit: &mut impl FnMut(Cmd)) {
        for v in 0..VOICES {
            if self.lane_note[v].is_some() {
                emit(Cmd::GateVoice { node: self.gate_node, voice: v as u8, on: false });
                self.lane_note[v] = None;
            }
        }
    }

    fn free_lane(&self) -> Option<usize> {
        (0..VOICES).find(|&v| self.lane_note[v].is_none())
    }

    fn oldest_lane(&self) -> usize {
        let mut best = 0;
        for v in 1..VOICES {
            if self.lane_age[v] < self.lane_age[best] {
                best = v;
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    // Capture the Cmds emitted by one note event.
    fn on(a: &mut VoiceAllocator, note: u8, vel: u8) -> Vec<Cmd> {
        let mut c: Vec<Cmd> = Vec::new();
        {
            let mut e = |x: Cmd| c.push(x);
            a.note_on(note, vel, &mut e);
        }
        c
    }
    fn off(a: &mut VoiceAllocator, note: u8) -> Vec<Cmd> {
        let mut c: Vec<Cmd> = Vec::new();
        {
            let mut e = |x: Cmd| c.push(x);
            a.note_off(note, &mut e);
        }
        c
    }

    #[test]
    fn note_on_emits_pitch_and_gate_on_lane_0() {
        let mut a = VoiceAllocator::new(NodeId(10), NodeId(20));
        let c = on(&mut a, 69, 100); // A4 → semitone 0
        assert_eq!(c.len(), 2);
        match c[0] {
            Cmd::SetParam { node, param, value } => {
                assert_eq!(node.0, 10);
                assert_eq!(param, 0);
                assert!((value - 0.0).abs() < 1e-6);
            }
            _ => panic!("expected SetParam"),
        }
        match c[1] {
            Cmd::GateVoice { node, voice, on } => {
                assert_eq!(node.0, 20);
                assert_eq!(voice, 0);
                assert!(on);
            }
            _ => panic!("expected GateVoice"),
        }
    }

    #[test]
    fn eight_notes_fill_lanes_then_ninth_steals_oldest() {
        let mut a = VoiceAllocator::new(NodeId(0), NodeId(1));
        for k in 0..VOICES {
            let c = on(&mut a, 60 + k as u8, 100);
            match c[0] {
                Cmd::SetParam { param, .. } => assert_eq!(param as usize, k, "note {k} → lane {k}"),
                _ => panic!(),
            }
        }
        // All 8 lanes held; lane 0 is oldest. A 9th note steals lane 0.
        let c = on(&mut a, 72, 100);
        match c[0] {
            Cmd::SetParam { param, value, .. } => {
                assert_eq!(param, 0, "steals oldest lane 0");
                assert!((value - (72.0 - 69.0)).abs() < 1e-6);
            }
            _ => panic!(),
        }
        assert!(matches!(c[1], Cmd::GateVoice { voice: 0, on: true, .. }));
    }

    #[test]
    fn note_off_releases_the_right_lane_and_frees_it() {
        let mut a = VoiceAllocator::new(NodeId(0), NodeId(1));
        on(&mut a, 60, 100); // lane 0
        on(&mut a, 64, 100); // lane 1
        let c = off(&mut a, 60);
        assert_eq!(c.len(), 1);
        assert!(matches!(c[0], Cmd::GateVoice { voice: 0, on: false, .. }));
        // Lane 0 is free again → next note-on reuses it.
        let c2 = on(&mut a, 67, 100);
        assert!(matches!(c2[0], Cmd::SetParam { param: 0, .. }));
    }

    #[test]
    fn note_on_velocity_zero_is_note_off() {
        let mut a = VoiceAllocator::new(NodeId(0), NodeId(1));
        on(&mut a, 60, 100); // lane 0 held
        let c = on(&mut a, 60, 0); // vel 0 → note-off
        assert_eq!(c.len(), 1);
        assert!(matches!(c[0], Cmd::GateVoice { voice: 0, on: false, .. }));
    }

    #[test]
    fn note_off_for_unheld_note_is_noop() {
        let mut a = VoiceAllocator::new(NodeId(0), NodeId(1));
        let c = off(&mut a, 60);
        assert!(c.is_empty());
    }

    #[test]
    fn all_notes_off_releases_every_held_lane() {
        let mut a = VoiceAllocator::new(NodeId(0), NodeId(1));
        on(&mut a, 60, 100);
        on(&mut a, 64, 100);
        let mut c: Vec<Cmd> = Vec::new();
        {
            let mut e = |x: Cmd| c.push(x);
            a.all_notes_off(&mut e);
        }
        assert_eq!(c.len(), 2);
        assert!(c.iter().all(|cmd| matches!(cmd, Cmd::GateVoice { on: false, .. })));
    }
}
```

- [ ] **Step 2: Register the module + re-export (lib.rs)**

In `crates/deluge-audio-graph/src/lib.rs`, add `pub mod voice;` (after `pub mod pool;`) and `pub use voice::VoiceAllocator;` (near the other `pub use`s).

- [ ] **Step 3: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph voice`
Expected: PASS — all six allocator unit tests. No warnings.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-audio-graph/src/voice.rs crates/deluge-audio-graph/src/lib.rs
git commit -m "feat(audio-graph): VoiceAllocator — note→lane allocation with oldest-LRU steal"
```

---

### Task 3: End-to-end note-driven voice (engine test)

**Files:**
- Modify: `crates/deluge-audio-graph/src/engine.rs` (tests module)

**Interfaces:**
- Consumes: `Kind::{PolyCtrl, PolyMtof, PolyOsc, PolySvf, PolyMul, PolyAr, VoiceSum}` (Task 1 + Sy-2), `VoiceAllocator` (Task 2), `Cmd`, `Input`, `NodeId`.

- [ ] **Step 1: Write the note-driven engine tests**

In `engine.rs`'s `#[cfg(test)] mod tests`, add (`VoiceAllocator` is in the crate root — reference it as `crate::voice::VoiceAllocator`):

```rust
    fn build_voice(e: &mut Engine<64, 8, 56, 4, 45056, 2048>) {
        // PolyCtrl(0) → PolyMtof(1) → PolyOsc(2) → PolySvf(3) →
        //     PolyMul(4, PolyAr(5)) → VoiceSum(6)
        e.create(NodeId(0), Kind::PolyCtrl);
        e.create(NodeId(1), Kind::PolyMtof);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.create(NodeId(2), Kind::PolyOsc);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        e.create(NodeId(3), Kind::PolySvf);
        *e.node_input_mut(NodeId(3), 0).unwrap() = Input::Node { node: NodeId(2), port: 0 };
        *e.node_input_mut(NodeId(3), 1).unwrap() = Input::Const(2000.0);
        *e.node_input_mut(NodeId(3), 2).unwrap() = Input::Const(0.2);
        e.create(NodeId(5), Kind::PolyAr);
        *e.node_input_mut(NodeId(5), 0).unwrap() = Input::Const(0.001); // attack
        *e.node_input_mut(NodeId(5), 1).unwrap() = Input::Const(0.005); // release
        e.create(NodeId(4), Kind::PolyMul);
        *e.node_input_mut(NodeId(4), 0).unwrap() = Input::Node { node: NodeId(3), port: 0 };
        *e.node_input_mut(NodeId(4), 1).unwrap() = Input::Node { node: NodeId(5), port: 0 };
        e.create(NodeId(6), Kind::VoiceSum);
        *e.node_input_mut(NodeId(6), 0).unwrap() = Input::Node { node: NodeId(4), port: 0 };
    }

    #[test]
    fn note_on_sounds_then_note_off_silences() {
        type PE = Engine<64, 8, 56, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        build_voice(&mut e);
        let mut alloc = crate::voice::VoiceAllocator::new(NodeId(0), NodeId(5));

        { let mut emit = |c: Cmd| e.apply(c); alloc.note_on(69, 100, &mut emit); }
        for _ in 0..8 { e.render_block(); } // let the fast envelope attack
        let out = e.node_output(NodeId(6), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 8.5), "bounded");
        assert!(out.iter().any(|&s| s.abs() > 1e-3), "note-on sounds");

        { let mut emit = |c: Cmd| e.apply(c); alloc.note_off(69, &mut emit); }
        for _ in 0..300 { e.render_block(); } // past the 5 ms release
        let out2 = e.node_output(NodeId(6), 0);
        assert!(out2.iter().all(|&s| s.abs() < 1e-4), "note-off silences");
    }

    #[test]
    fn ninth_note_steals_and_still_sounds() {
        type PE = Engine<64, 8, 56, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        build_voice(&mut e);
        let mut alloc = crate::voice::VoiceAllocator::new(NodeId(0), NodeId(5));
        {
            let mut emit = |c: Cmd| e.apply(c);
            for k in 0..9u8 {
                alloc.note_on(60 + k, 100, &mut emit); // 9 notes → one steal
            }
        }
        for _ in 0..8 { e.render_block(); }
        let out = e.node_output(NodeId(6), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 8.5), "bounded after steal");
        assert!(out.iter().any(|&s| s.abs() > 1e-3), "still sounds after steal");
    }
```

- [ ] **Step 2: Run both feature configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph note_on_sounds ninth_note_steals`
Expected: PASS (scalar).
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features deluge-dsp-kernels/simd note_on_sounds ninth_note_steals`
Expected: PASS (f32x8 PolyOsc/PolySvf through the note-driven voice). No warnings.

- [ ] **Step 3: Commit**

```bash
git add crates/deluge-audio-graph/src/engine.rs
git commit -m "test(audio-graph): end-to-end note-driven voice (allocator → sound → silence, steal)"
```

---

## Notes for the implementer

- `PolyMtof::process` is element-wise, so the interleave is trivially preserved — no `VOICES` indexing needed.
- The closure-sink pattern in tests: scope the `let mut emit = |c| e.apply(c);` in its own `{ }` block so the mutable borrow of `e` ends before `e.render_block()`.
- `Cmd`/`NodeId` field access in tests uses `matches!` + destructuring (and `node.0` for the id) — robust regardless of derives.
- If an anchor moved, grep the named symbol (`poly_in_count`, `Kind::PolyMul`, the poly-kinds no-op arm) — the relationship matters, not line numbers.
- Node ids in `build_voice` are created out of numeric order (5 before 4) intentionally — the engine derives eval order from input dependencies, not id order.
