# Sy-5b: Mono / Legato Glide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A monophonic `Synth.mono { |pitch| … }` mode with true-legato portamento — overlapping notes glide the pitch (no amp re-attack); notes from silence jump and attack; glide time via `synth.glide = seconds`.

**Architecture:** Mono reuses the poly voice graph on lane 0 (unison-ready), inserting a new `PolySlew` in the pitch path (`PolyCtrl → PolySlew → PolyMtof`) and driving it with a new `MonoAllocator` (held-note stack, last-note priority). Poly `Synth.new` is untouched.

**Tech Stack:** Rust `no_std`, `deluge-dsp-kernels` / `deluge-audio-graph` / `deluge-wren-core` (Wren scripting).

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** poly `Synth.new` builds/renders unchanged (no `PolySlew`).
- **Two registration tables:** every new Wren foreign method goes in BOTH `register_audio` (`bindings_audio.rs`) AND the `wren-sys-backend` `METHODS` table (`bindings.rs`) — the latter is what the test VM boots.
- **Test invocation (per-crate, never `--workspace`; both configs; cargo rejects multiple bare positional names — use `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## File Structure

- `crates/deluge-dsp-kernels/src/poly.rs` — new `PolySlew`.
- `crates/deluge-audio-graph/src/node.rs` — `Kind::PolySlew` wiring.
- `crates/deluge-audio-graph/src/voice.rs` — new `MonoAllocator`.
- `crates/deluge-wren-core/src/audio.rs` — `PolyCtx.slew_node`, `mono_begin`, `mono_end`.
- `crates/deluge-wren-core/src/bindings_audio.rs` — `SynthAlloc` enum, mono impls, glide setter.
- `crates/deluge-wren-core/src/bindings.rs` — register new foreigns.
- `crates/deluge-wren-core/wren/prelude.wren` — `Synth.mono`, `glide=`.
- Tests: respective crate test modules + `crates/deluge-wren-core/tests/audio_bindings.rs`.

---

## Task 1: `PolySlew` kernel

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/poly.rs`
- Test: `poly.rs` test module

**Interfaces:**
- Consumes: `In` (for the null-test's mono `Slew` oracle), `VOICES`, mono `Slew` (`crate::modutil::Slew`).
- Produces: `pub struct PolySlew` with `new()`, `set_time(&mut self, t: f32)`, `trigger_voice(&mut self, v: usize)`, `process(&mut self, target: &[f32], dt: f32, out: &mut [f32])`. Used by Tasks 2–4.

- [ ] **Step 1: Write the failing tests**

```rust
#[test]
fn polyslew_lane_matches_mono_slew() {
    use crate::modutil::Slew;
    let dt = 1.0 / 48_000.0;
    let n = 300usize;
    let time = 0.05f32;
    let mut poly = PolySlew::new();
    poly.set_time(time);
    let mut mono = Slew::new();
    // per-voice-distinct target ramps
    let target: std::vec::Vec<f32> = (0..n * VOICES)
        .map(|j| { let i = j / VOICES; let v = j % VOICES; (i as f32 * 0.01) + v as f32 })
        .collect();
    let mut out = std::vec![0.0f32; n * VOICES];
    poly.process(&target, dt, &mut out);
    // compare lane 0 to a mono Slew fed lane 0's target + constant time
    let vin: std::vec::Vec<f32> = (0..n).map(|i| target[i * VOICES]).collect();
    let mut mout = std::vec![0.0f32; n];
    mono.process(In::A(&vin), In::K(time), dt, &mut mout);
    for i in 0..n {
        assert_eq!(out[i * VOICES], mout[i], "lane0 sample {i}: {} vs {}", out[i * VOICES], mout[i]);
    }
}

#[test]
fn polyslew_trigger_snaps_lane() {
    let dt = 1.0 / 48_000.0;
    let mut s = PolySlew::new();
    s.set_time(1.0); // long glide
    // one block toward target 5.0 — without snap it barely moves
    let target = std::vec![5.0f32; VOICES];
    let mut out = std::vec![0.0f32; VOICES];
    s.trigger_voice(0);
    s.process(&target, dt, &mut out);
    assert_eq!(out[0], 5.0, "snapped lane 0 jumps to target");
    assert!(out[1] < 1.0, "un-snapped lane 1 barely moves: {}", out[1]);
}

#[test]
fn polyslew_time_zero_passes_through() {
    let dt = 1.0 / 48_000.0;
    let mut s = PolySlew::new(); // time defaults 0
    let target = std::vec![3.0f32; VOICES * 2];
    let mut out = std::vec![0.0f32; VOICES * 2];
    s.process(&target, dt, &mut out);
    for &o in &out { assert_eq!(o, 3.0, "time=0 snaps every sample"); }
}
```

> Match the poly.rs test-module conventions for `In::A`/`In::K`/`std::vec` (read an existing poly test). `In::K` is the constant-`In` variant used elsewhere.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels polyslew`
Expected: FAIL — `PolySlew` not found.

- [ ] **Step 3: Implement `PolySlew`**

```rust
/// Poly one-pole slew/lag (glide). Per-voice `z`; scalar `time` (seconds, set via
/// `set_param`). `trigger_voice(v)` snaps lane v to its target on the next sample
/// (note-from-silence → no swoop from the previous pitch). Mirrors mono `Slew`.
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
        let c = (dt / self.time.max(dt)).min(1.0);
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
impl Default for PolySlew {
    fn default() -> Self { Self::new() }
}
```

> `c` mirrors mono `Slew` (`modutil.rs`): `c = (dt / time.max(dt)).min(1.0)`. Scalar-per-voice (a one-pole recurrence — no f32x8). Add `use crate::modutil::Slew;` in the test module only.

- [ ] **Step 4: Run to verify they pass (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels polyslew` and again `--features simd`.
Expected: PASS both.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/poly.rs
git commit -m "feat(kernels): PolySlew (poly glide, per-lane snap) — mirrors mono Slew"
```

---

## Task 2: `Kind::PolySlew` graph node

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs`
- Test: `node.rs` test module

**Interfaces:**
- Consumes: `PolySlew` (Task 1), `In`, `VOICES`.
- Produces: `Kind::PolySlew`; `State::PolySlew(PolySlew)`; `set_param(0)` = glide time; `trigger_voice` dispatch to `PolySlew`. Used by Task 4.

- [ ] **Step 1: Add Kind/State/constructor + predicates**

- `Kind::PolySlew` (alongside the poly kinds); `State::PolySlew(PolySlew)`.
- Constructor arm: `Kind::PolySlew => State::PolySlew(PolySlew::new()),`.
- `out_width`: add `Kind::PolySlew` to the `=> VOICES` arm.
- `is_poly`: add to the `matches!` list.
- `poly_in_count`: `Kind::PolySlew => 1` (target is the one poly edge; glide time is a `set_param`, not a port). Add alongside `PolyOsc`/`PolySvf` etc.
- Add `Kind::PolySlew` to the `process_resolved` no-op poly arm.
- Import `PolySlew` into node.rs (beside the other poly-kernel imports).

- [ ] **Step 2: Add the `poly_process` arm, `set_param`, and `trigger_voice` dispatch**

`poly_process` (beside `PolyOsc`):

```rust
            Kind::PolySlew => {
                if let (State::PolySlew(s), Some(target)) = (&mut self.state, poly_in[0]) {
                    s.process(target, dt, out);
                }
            }
```

`set_param` (beside the ADSR-sustain arm): `State::PolySlew(s) if param == 0 => s.set_time(value),`.

`trigger_voice` — add `State::PolySlew` to the existing match:

```rust
    pub fn trigger_voice(&mut self, v: usize) {
        match &mut self.state {
            State::PolyAr(a) => a.trigger_voice(v),
            State::PolyAdsr(a) => a.trigger_voice(v),
            State::PolySlew(s) => s.trigger_voice(v),
            _ => {}
        }
    }
```

- [ ] **Step 3: Write + run the graph test**

```rust
#[test]
fn polyslew_node_glides_and_snaps() {
    // Build a PolySlew node, set_param(0, glide_time), feed a poly target tile via
    // poly_process, assert lane 0 ramps toward a changed target; then trigger_voice(0)
    // + process again asserts lane 0 snaps to target. Mirror the existing PolyOsc/PolySvf
    // node test harness (how it drives poly_process with a poly_in tile and reads out).
}
```

> Follow the existing poly-node test harness in node.rs (how it constructs a `Node`, supplies a `poly_in[0]` tile, and reads the interleaved `out`). Assert the glide (lane 0 moves partway toward the target over samples) and the snap (after `trigger_voice(0)`, lane 0 == target).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph polyslew` and again `--features deluge-dsp-kernels/simd`. Also the full graph crate both configs.
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(graph): PolySlew node — set_param glide time + trigger_voice snap"
```

---

## Task 3: `MonoAllocator`

**Files:**
- Modify: `crates/deluge-audio-graph/src/voice.rs`
- Test: `voice.rs` test module

**Interfaces:**
- Consumes: `Cmd::{SetParam, GateVoice, TriggerVoice}`, `NodeId`, the `A440_NOTE` const (voice.rs).
- Produces: `pub struct MonoAllocator` with `new(pitch_node, slew_node, gate_node, vel_node: Option<NodeId>)`, `note_on(note, vel, emit)`, `note_off(note, emit)`, `all_notes_off(emit)`, and a `slew_node()` accessor (for the glide setter). `pub const MONO_STACK: usize = 16`. Used by Task 4.

- [ ] **Step 1: Write the failing tests**

Mirror the existing `VoiceAllocator` test harness (`on`/`off` helpers capturing `Vec<Cmd>`):

```rust
#[test]
fn mono_first_note_snaps_and_gates() {
    let mut m = MonoAllocator::new(NodeId(10), NodeId(20), NodeId(30), None);
    let c = mon(&mut m, 69, 100); // A4 from silence
    // SetParam(pitch=10, lane0, 0.0) + TriggerVoice(slew=20, 0) + GateVoice(gate=30, 0, on)
    assert_eq!(c.len(), 3);
    assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), param: 0, .. }));
    assert!(matches!(c[1], Cmd::TriggerVoice { node: NodeId(20), voice: 0 }));
    assert!(matches!(c[2], Cmd::GateVoice { node: NodeId(30), voice: 0, on: true }));
}

#[test]
fn mono_legato_note_glides_without_regate() {
    let mut m = MonoAllocator::new(NodeId(10), NodeId(20), NodeId(30), None);
    mon(&mut m, 60, 100);       // first note (from silence)
    let c = mon(&mut m, 64, 100); // legato (60 still held)
    // Only a pitch SetParam (glide) — NO TriggerVoice, NO GateVoice
    assert_eq!(c.len(), 1);
    assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), param: 0, .. }));
}

#[test]
fn mono_note_off_falls_back_to_held_note() {
    let mut m = MonoAllocator::new(NodeId(10), NodeId(20), NodeId(30), None);
    mon(&mut m, 60, 100);
    mon(&mut m, 64, 100); // 64 sounding, 60 held
    let c = moff(&mut m, 64); // release 64 → glide back to 60
    assert_eq!(c.len(), 1);
    match c[0] {
        Cmd::SetParam { node: NodeId(10), param: 0, value } => assert!((value - (60.0 - 69.0)).abs() < 1e-6),
        _ => panic!("expected glide-back SetParam to 60"),
    }
}

#[test]
fn mono_last_note_off_releases() {
    let mut m = MonoAllocator::new(NodeId(10), NodeId(20), NodeId(30), None);
    mon(&mut m, 60, 100);
    let c = moff(&mut m, 60); // stack empty → release
    assert_eq!(c.len(), 1);
    assert!(matches!(c[0], Cmd::GateVoice { node: NodeId(30), voice: 0, on: false }));
}

#[test]
fn mono_velocity_written_when_present() {
    let mut m = MonoAllocator::new(NodeId(10), NodeId(20), NodeId(30), Some(NodeId(40)));
    let c = mon(&mut m, 69, 100);
    // pitch SetParam, velocity SetParam(node 40, 100/127), TriggerVoice, GateVoice
    assert!(c.iter().any(|cmd| matches!(cmd,
        Cmd::SetParam { node: NodeId(40), param: 0, value } if (value - 100.0/127.0).abs() < 1e-6)));
}

#[test]
fn mono_note_off_unheld_is_noop() {
    let mut m = MonoAllocator::new(NodeId(10), NodeId(20), NodeId(30), None);
    let c = moff(&mut m, 60);
    assert!(c.is_empty());
}
```

Add `mon`/`moff` capture helpers mirroring the existing `on`/`off` (velocity for `mon`, note only for `moff`).

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- mono_first_note mono_legato mono_note_off mono_last_note mono_velocity`
Expected: FAIL — `MonoAllocator` not found.

- [ ] **Step 3: Implement `MonoAllocator`**

```rust
/// Max simultaneously-held notes tracked for last-note priority.
pub const MONO_STACK: usize = 16;

/// Monophonic note allocator with last-note priority and legato glide. Drives
/// lane 0 of the poly graph: pitch (a PolyCtrl), a PolySlew (snapped from silence,
/// glides on legato), and one gate (PolyAr/PolyAdsr). Control-plane only.
pub struct MonoAllocator {
    pitch_node: NodeId,
    slew_node: NodeId,
    gate_node: NodeId,
    vel_node: Option<NodeId>,
    notes: [u8; MONO_STACK], // press order; top (notes[len-1]) = sounding
    len: usize,
}

impl MonoAllocator {
    pub fn new(pitch_node: NodeId, slew_node: NodeId, gate_node: NodeId, vel_node: Option<NodeId>) -> MonoAllocator {
        MonoAllocator { pitch_node, slew_node, gate_node, vel_node, notes: [0; MONO_STACK], len: 0 }
    }

    pub fn slew_node(&self) -> NodeId { self.slew_node }

    pub fn note_on(&mut self, note: u8, vel: u8, emit: &mut impl FnMut(Cmd)) {
        if vel == 0 { self.note_off(note, emit); return; }
        let from_silence = self.len == 0;
        // push (drop oldest if full)
        if self.len == MONO_STACK {
            for i in 1..MONO_STACK { self.notes[i - 1] = self.notes[i]; }
            self.len -= 1;
        }
        self.notes[self.len] = note;
        self.len += 1;
        // pitch (always) → PolySlew glides toward it (or snaps, below)
        emit(Cmd::SetParam { node: self.pitch_node, param: 0, value: note as f32 - A440_NOTE });
        if let Some(vn) = self.vel_node {
            emit(Cmd::SetParam { node: vn, param: 0, value: vel as f32 / 127.0 });
        }
        if from_silence {
            emit(Cmd::TriggerVoice { node: self.slew_node, voice: 0 });          // snap the glide
            emit(Cmd::GateVoice { node: self.gate_node, voice: 0, on: true });   // attack
        }
        // legato (else): no snap, no re-gate — true legato
    }

    pub fn note_off(&mut self, note: u8, emit: &mut impl FnMut(Cmd)) {
        // find first match
        let mut idx = None;
        for i in 0..self.len { if self.notes[i] == note { idx = Some(i); break; } }
        let Some(i) = idx else { return; }; // unheld → no-op
        let was_top = i == self.len - 1;
        // remove (shift down)
        for j in (i + 1)..self.len { self.notes[j - 1] = self.notes[j]; }
        self.len -= 1;
        if self.len == 0 {
            emit(Cmd::GateVoice { node: self.gate_node, voice: 0, on: false }); // release
        } else if was_top {
            let top = self.notes[self.len - 1];
            emit(Cmd::SetParam { node: self.pitch_node, param: 0, value: top as f32 - A440_NOTE }); // glide back
        }
        // removing a non-top held note → stack-only, no sound change
    }

    pub fn all_notes_off(&mut self, emit: &mut impl FnMut(Cmd)) {
        if self.len > 0 {
            emit(Cmd::GateVoice { node: self.gate_node, voice: 0, on: false });
        }
        self.len = 0;
    }
}
```

> `A440_NOTE` is the existing const in voice.rs (69.0). `Cmd::TriggerVoice { node, voice }` and `Cmd::GateVoice { node, voice, on }` are the existing variants (check exact field names in cmd.rs and match them). No heap, no panic (`let Some(i) = … else { return }` handles the unheld case).

- [ ] **Step 4: Run to verify they pass + full crate regression**

Run the mono tests both configs, then the full graph crate both configs. Expected: PASS (existing `VoiceAllocator` tests untouched).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src/voice.rs
git commit -m "feat(graph): MonoAllocator — last-note stack, true-legato glide, snap-from-silence"
```

---

## Task 4: Wren mono build path + `SynthAlloc` dispatch

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (`PolyCtx.slew_node`, `mono_begin`, `mono_end`)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`SynthAlloc` enum, `SynthObj`, mono impls, glide setter, registration)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (METHODS table)

**Interfaces:**
- Consumes: `MonoAllocator` (Task 3), `Kind::PolySlew` (Task 2), `poly_vel_begin`/`VoiceAllocator` (Sy-5a), `NULL_ID`, `alloc_node_id`/`new_node`.
- Produces: Wren foreigns `Node.monoBegin_()`, `Node.monoEnd_(out)`, `Synth.glide=(seconds)`; `SynthAlloc { Poly(VoiceAllocator), Mono(MonoAllocator) }`. Used by Task 5.

- [ ] **Step 1: `PolyCtx.slew_node` + `mono_begin`/`mono_end` (audio.rs)**

Add `slew_node: u16` to `PolyCtx` (init/reset `NULL_ID` in the constructor AND in both `poly_begin` and `mono_begin`). Add:

```rust
/// Mono voice build: PolyCtrl → PolySlew → PolyMtof (the slew is the only
/// difference from poly_begin). Records pitch_ctrl + slew_node. Returns the
/// PolyMtof output (the `pitch` handed to the builder).
pub fn mono_begin() -> u16 {
    let ctrl = alloc_node_id();
    new_node(ctrl, Kind::PolyCtrl, [Input::Const(0.0); 3]);
    let slew = alloc_node_id();
    new_node(slew, Kind::PolySlew, [Input::Node { node: NodeId(ctrl), port: 0 }, Input::Const(0.0), Input::Const(0.0)]);
    let mtof = alloc_node_id();
    new_node(mtof, Kind::PolyMtof, [Input::Node { node: NodeId(slew), port: 0 }, Input::Const(0.0), Input::Const(0.0)]);
    let p = poly();
    p.mode = true;
    p.pitch_ctrl = ctrl;
    p.slew_node = slew;
    p.gate_ar = NULL_ID;
    p.gate_count = 0;
    p.vel_node = NULL_ID;
    mtof
}

/// Returns (pitch_ctrl, slew_node, gate_ar, vel_node) for the mono SynthObj.
pub fn mono_end() -> (u16, u16, u16, u16) {
    let p = poly();
    p.mode = false;
    (p.pitch_ctrl, p.slew_node, p.gate_ar, p.vel_node)
}
```

Also reset `p.slew_node = NULL_ID;` in the existing `poly_begin` (so a poly synth after a mono one carries no stale slew).

- [ ] **Step 2: `SynthAlloc` enum + `SynthObj` + mono impls (bindings_audio.rs)**

Replace `SynthObj.alloc: VoiceAllocator` with an enum wrapper so the note-handling call sites stay unchanged:

```rust
pub(crate) enum SynthAlloc {
    Poly(deluge_audio_graph::VoiceAllocator),
    Mono(deluge_audio_graph::MonoAllocator),
}
impl SynthAlloc {
    fn note_on(&mut self, note: u8, vel: u8, emit: &mut impl FnMut(deluge_audio_graph::Cmd)) {
        match self { SynthAlloc::Poly(a) => a.note_on(note, vel, emit), SynthAlloc::Mono(m) => m.note_on(note, vel, emit) }
    }
    fn note_off(&mut self, note: u8, emit: &mut impl FnMut(deluge_audio_graph::Cmd)) {
        match self { SynthAlloc::Poly(a) => a.note_off(note, emit), SynthAlloc::Mono(m) => m.note_off(note, emit) }
    }
    fn all_notes_off(&mut self, emit: &mut impl FnMut(deluge_audio_graph::Cmd)) {
        match self { SynthAlloc::Poly(a) => a.all_notes_off(emit), SynthAlloc::Mono(m) => m.all_notes_off(emit) }
    }
    fn mono_slew(&self) -> Option<deluge_audio_graph::NodeId> {
        match self { SynthAlloc::Mono(m) => Some(m.slew_node()), SynthAlloc::Poly(_) => None }
    }
}
```

`SynthObj { alloc: SynthAlloc, out_node: u16 }`. Update `node_poly_end_impl` to wrap its `VoiceAllocator` in `SynthAlloc::Poly(alloc)`. The `synth_note_on_impl`/`synth_note_off_impl` call sites (`self_synth(vm).alloc.note_on(...)`) work unchanged (the enum has the same method names). Confirm the `Cmd` type path used in the emit closure matches (`deluge_audio_graph::Cmd`).

Add `node_mono_begin_impl` (mirror `node_poly_begin_impl`: `let mtof = audio::mono_begin(); return_node(vm, mtof)`) and `node_mono_end_impl`:

```rust
pub(crate) fn node_mono_end_impl<S: SlotApi>(vm: &S) {
    let out = arg_input(vm, 1);
    let (pitch, slew, gate, vel_raw) = audio::mono_end();
    let vel = if vel_raw == audio::NULL_ID { None } else { Some(NodeId(vel_raw)) };
    let sum = audio::alloc_node_id();
    audio::new_node(sum, Kind::VoiceSum, [out, Input::Const(0.0), Input::Const(0.0)]);
    let alloc = SynthAlloc::Mono(deluge_audio_graph::MonoAllocator::new(NodeId(pitch), NodeId(slew), NodeId(gate), vel));
    unsafe { vm.new_foreign_in(0, SynthObj { alloc, out_node: sum }) };
}
```

Add the glide setter:

```rust
pub(crate) fn synth_set_glide_impl<S: SlotApi>(vm: &S) {
    let t = vm.get_f(1) as f32;
    match self_synth(vm).alloc.mono_slew() {
        Some(slew) => audio::set_param(slew.0, 0, t), // PolySlew set_time
        None => { /* poly synth: abort — see note */ }
    }
}
```

> For the poly-synth abort: use the crate's Wren-abort mechanism (find how an existing foreign aborts — e.g. `Fiber.abort` is Wren-side; a Rust foreign signals an error differently). Simplest robust option: make `Synth.glide=` a Wren wrapper in the prelude that aborts if the synth is poly BEFORE calling the foreign — but the prelude can't easily know poly-vs-mono. Alternative: the foreign sets an error slot / no-ops on poly. Match how other Rust foreigns report a misuse error (grep for how `bus_write_impl`-style or a validating foreign signals failure); if there's no clean Rust-side abort, no-op on poly (glide has no effect on a poly synth) and document it. Prefer a real error if the codebase has a foreign-error path.

- [ ] **Step 3: Register the new foreigns in BOTH tables**

- `register_audio` (bindings_audio.rs): `monoBegin_()` (mirror `polyBegin_`, static), `monoEnd_(_)` (mirror `polyEnd_`, static), and `glide=(_)` as an instance method on `Synth` (mirror how `Synth`'s `noteOn(_,_)`/`out` are registered).
- `bindings.rs` METHODS: add `monoBegin_`, `monoEnd_`, and the `Synth` `glide=(_)` setter alongside the existing `Synth`/`Node` entries.

- [ ] **Step 4: Verify it compiles + existing tests unchanged**

Run the full wren-core suite both configs. Nothing calls the mono path yet (Task 5), but the `SynthAlloc` refactor + `node_poly_end_impl` wrapping must keep every existing poly-synth test green.
Expected: PASS both.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs
git commit -m "feat(wren): mono build path (mono_begin/end) + SynthAlloc Poly|Mono + glide setter"
```

---

## Task 5: `Synth.mono` prelude + end-to-end

**Files:**
- Modify: `crates/deluge-wren-core/wren/prelude.wren`
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Node.monoBegin_()`/`Node.monoEnd_(_)`/`Synth.glide=(_)` (Task 4); the Sy-5a `Node.polyVelBegin_`/`Fn.arity` pattern.
- Produces: `Synth.mono(builder)`; `synth.glide = seconds`.

- [ ] **Step 1: Write the failing e2e tests**

In `tests/audio_bindings.rs` (mirror the existing Synth render harness; newline-separated statements — this Wren dialect rejects `;`):

```rust
#[test]
fn synth_mono_builds_and_renders() {
    assert!(run_script_ok(
        "var s = Synth.mono { |p| Osc.saw(p).lpf(1500) * Env.adsr(0.005,0.1,0.7,0.2) }\ns.glide = 0.08\nOut.patch(s.out)\ns.noteOn(60,100)"
    ));
}

#[test]
fn synth_mono_velocity_parity() {
    assert!(run_script_ok(
        "var s = Synth.mono { |p, vel| Osc.saw(p) * Env.adsr(0.005,0.1,0.7,0.2) * vel }\nOut.patch(s.out)\ns.noteOn(60,100)"
    ));
}

#[test]
fn synth_poly_new_glide_aborts() {
    // glide= on a poly synth is a misuse. (If the design no-ops instead of aborting,
    // change this to assert it builds+runs harmlessly — match Task 4's chosen behavior.)
    let ok = run_script_ok("var s = Synth.new { |p| Osc.saw(p) * Env.adsr(0.01,0.1,0.6,0.3) }\ns.glide = 0.1");
    // assert per the chosen semantics (abort → !ok, or no-op → ok)
    let _ = ok;
}
```

> If the harness can measure rendered pitch/level, add a stronger mono test: play `noteOn(60,…)` then a legato `noteOn(72,…)` with `glide=0.1` and assert the rendered pitch RAMPS (not steps) and the amp does NOT re-attack (no new attack transient). If only build/run is available, the build+render assertions above are the guard; keep the allocator-level Cmd-sequence tests (Task 3) as the behavior proof.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- synth_mono synth_poly_new_glide`
Expected: `synth_mono_*` FAIL — `Synth.mono` undefined.

- [ ] **Step 3: Add `Synth.mono` + `glide=` decl (prelude.wren)**

In the `Synth` foreign class add `foreign glide=(seconds)` and the mono constructor (mirror `Synth.new`'s arity branch):

```wren
  static mono(builder) {
    if (Node.polyMode_ == 1) Fiber.abort("nested Synth not supported")
    var pitch = Node.monoBegin_()
    var out
    if (builder.arity >= 2) {
      var vel = Node.polyVelBegin_()
      out = builder.call(pitch, vel)
    } else {
      out = builder.call(pitch)
    }
    if (Node.polyGateCount_ == 0) Fiber.abort("a Synth voice needs an Env.ar (the amp gate)")
    if (Node.polyGateCount_ > 1) Fiber.abort("multiple Env.ar in a Synth isn't supported yet")
    return Node.monoEnd_(out)
  }
```

- [ ] **Step 4: Run to verify they pass + full regression**

Run the mono e2e both configs, then the FULL wren-core suite both configs (poly `Synth.new` unchanged).
Expected: PASS both.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): Synth.mono { |pitch| } mono/legato glide voice + synth.glide"
```

---

## Self-Review Notes (for the executor)

- **Non-breaking is the hard gate:** poly `Synth.new` must build/render unchanged. The `SynthAlloc::Poly` wrap (Task 4) and the untouched `poly_begin` (no slew) enforce it — the full wren-core regression is the proof.
- **`PolyCtx` state leak:** `mono_begin` and `poly_begin` must BOTH reset `slew_node` (and mono resets `vel_node`/`gate_*` like poly) so a mono build after a poly build (or vice-versa) can't inherit a stale id. Same class of bug as Sy-5a's `vel_node` reset.
- **True legato = the gate discipline:** the `MonoAllocator` gates ON only from silence and OFF only to silence; legato notes emit a pitch `SetParam` ONLY. A stray `GateVoice`/`TriggerVoice` on a legato note would re-attack (wrong) — the Cmd-sequence tests (Task 3) are the guard.
- **Snap vs glide:** `TriggerVoice(slew_node, 0)` snaps the `PolySlew` (from silence); its absence on legato notes is what makes them glide. Verify the allocator sends `TriggerVoice` to the SLEW node (not the gate node).
- **Both registration tables** (`register_audio` + `bindings.rs` METHODS) for `monoBegin_`/`monoEnd_`/`glide=`.
- **Confirm exact names** (`In::K`/`In::A`, `Cmd` field names, `set_param`/`get_f`/`return_node`, the Rust-foreign abort mechanism) against the codebase before transcribing.
- **Deferred (do NOT implement):** unison (widen lane 0 → N detuned lanes), retrigger/legato toggle, poly-per-voice glide, multi-env, release-tail; Tb303/Modal poly; per-voice wavetable morph; f32x8 slew.
