# Sy-5a: Velocity → per-voice signal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Route MIDI note-on velocity to a per-voice `[0,1]` signal exposed as an optional 2nd `Synth` builder param — `Synth.new { |pitch, vel| … * vel }` — non-breaking for existing `{ |pitch| }` synths.

**Architecture:** No new kernel — velocity rides a second `PolyCtrl` (the pitch machinery). The `VoiceAllocator` gains an optional `vel_node` and emits one extra per-voice `SetParam(vel/127)` on note-on. `Synth.new` uses `Fn.arity` to lazily create the velocity carrier only when the builder takes it.

**Tech Stack:** Rust `no_std`, Wren scripting via `deluge-wren-core`.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** existing `Synth.new { |pitch| … }` synths build + render bit-identically (no velocity node created for arity-1 builders).
- **Two registration tables:** any new Wren foreign method must be registered in BOTH `register_audio` (`bindings_audio.rs`) AND the `wren-sys-backend` `METHODS` table (`bindings.rs`) — the latter is what the test harness's VM boots. (A miss there → "metaclass does not implement".)
- **Test invocation (per-crate, never `--workspace`; both configs; cargo rejects multiple bare positional names — use `-- name1 name2`):**
  - Graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## File Structure

- `crates/deluge-audio-graph/src/voice.rs` — `VoiceAllocator` gains `vel_node`.
- `crates/deluge-wren-core/src/audio.rs` — `PolyCtx.vel_node`; `poly_vel_begin`; `poly_end` returns vel.
- `crates/deluge-wren-core/src/bindings_audio.rs` — `node_poly_vel_begin_impl`; `node_poly_end_impl` passes vel; registration.
- `crates/deluge-wren-core/src/bindings.rs` — register `polyVelBegin_` in the METHODS table.
- `crates/deluge-wren-core/wren/prelude.wren` — `Synth.new` arity branch + `foreign static polyVelBegin_()`.
- Tests: `voice.rs` test module + `crates/deluge-wren-core/tests/audio_bindings.rs`.

---

## Task 1: VoiceAllocator velocity emit

**Files:**
- Modify: `crates/deluge-audio-graph/src/voice.rs`
- Test: `voice.rs` test module

**Interfaces:**
- Consumes: `Cmd::SetParam`, `NodeId`, `VOICES`.
- Produces: `VoiceAllocator::new(pitch_node: NodeId, gate_node: NodeId, vel_node: Option<NodeId>)`; `note_on` emits an extra `SetParam(vel_node, lane, vel/127)` when `vel_node` is `Some`. Used by Task 2.

- [ ] **Step 1: Update existing tests to the new constructor + write the velocity tests**

Every existing test constructs `VoiceAllocator::new(NodeId(a), NodeId(b))` — update ALL to `VoiceAllocator::new(NodeId(a), NodeId(b), None)` (the no-velocity path must stay behavior-identical: still 2 Cmds). Then add:

```rust
#[test]
fn note_on_emits_velocity_when_vel_node_present() {
    let mut a = VoiceAllocator::new(NodeId(10), NodeId(20), Some(NodeId(30)));
    let c = on(&mut a, 69, 100); // A4, vel 100
    assert_eq!(c.len(), 3, "pitch + velocity + gate");
    // c[0] = pitch SetParam(node 10), c[1] = velocity SetParam(node 30), c[2] = GateVoice(node 20)
    match c[1] {
        Cmd::SetParam { node, param, value } => {
            assert_eq!(node.0, 30, "velocity node");
            assert_eq!(param, 0, "lane 0");
            assert!((value - 100.0 / 127.0).abs() < 1e-6, "vel/127, got {value}");
        }
        _ => panic!("expected velocity SetParam at index 1"),
    }
    assert!(matches!(c[2], Cmd::GateVoice { node: NodeId(20), voice: 0, on: true }));
}

#[test]
fn note_on_no_velocity_node_emits_two_cmds() {
    let mut a = VoiceAllocator::new(NodeId(10), NodeId(20), None);
    let c = on(&mut a, 69, 100);
    assert_eq!(c.len(), 2, "pitch + gate only");
    assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), .. }));
    assert!(matches!(c[1], Cmd::GateVoice { node: NodeId(20), .. }));
}

#[test]
fn velocity_value_is_proportional() {
    let mut a = VoiceAllocator::new(NodeId(0), NodeId(1), Some(NodeId(2)));
    let hi = on(&mut a, 60, 127);
    let lo = on(&mut a, 62, 20);
    let vhi = match hi[1] { Cmd::SetParam { value, .. } => value, _ => panic!() };
    let vlo = match lo[1] { Cmd::SetParam { value, .. } => value, _ => panic!() };
    assert!(vhi > vlo, "higher velocity → larger value: {vhi} vs {vlo}");
    assert!((vhi - 1.0).abs() < 1e-6, "127 → 1.0");
}
```

> `NodeId` may be a tuple struct — match the existing `matches!(…, NodeId(20))` / `node.0` style used in the current tests.

- [ ] **Step 2: Run to verify they fail (compile error / wrong arity)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- note_on_emits_velocity note_on_no_velocity velocity_value_is_proportional`
Expected: FAIL to COMPILE — `VoiceAllocator::new` takes 2 args, tests pass 3.

- [ ] **Step 3: Add `vel_node` + the emit**

Struct + constructor:

```rust
pub struct VoiceAllocator {
    pitch_node: NodeId,
    gate_node: NodeId,
    vel_node: Option<NodeId>, // a PolyCtrl carrying per-voice velocity, or None
    lane_note: [Option<u8>; VOICES],
    lane_age: [u32; VOICES],
    clock: u32,
}

impl VoiceAllocator {
    pub fn new(pitch_node: NodeId, gate_node: NodeId, vel_node: Option<NodeId>) -> VoiceAllocator {
        VoiceAllocator {
            pitch_node,
            gate_node,
            vel_node,
            lane_note: [None; VOICES],
            lane_age: [0; VOICES],
            clock: 0,
        }
    }
```

In `note_on`, between the pitch `SetParam` and the `GateVoice`:

```rust
        emit(Cmd::SetParam {
            node: self.pitch_node,
            param: lane as u8,
            value: note as f32 - A440_NOTE,
        });
        if let Some(vn) = self.vel_node {
            emit(Cmd::SetParam { node: vn, param: lane as u8, value: vel as f32 / 127.0 });
        }
        emit(Cmd::GateVoice { node: self.gate_node, voice: lane as u8, on: true });
```

`note_off`/`all_notes_off` are unchanged.

- [ ] **Step 4: Run to verify they pass + full crate regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- note_on_emits_velocity note_on_no_velocity velocity_value_is_proportional` and again `--features deluge-dsp-kernels/simd`.
Then the full crate both configs — all existing allocator tests (updated to `…, None`) still pass.
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src/voice.rs
git commit -m "feat(graph): VoiceAllocator emits per-voice velocity SetParam (vel_node: Option)"
```

---

## Task 2: Wren velocity carrier plumbing

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (`PolyCtx.vel_node`, `poly_vel_begin`, `poly_end`)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_poly_vel_begin_impl`, `node_poly_end_impl`, registration)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (METHODS table registration)

**Interfaces:**
- Consumes: `VoiceAllocator::new(_, _, Option<NodeId>)` (Task 1); `alloc_node_id`/`new_node`/`Kind::PolyCtrl`/`NULL_ID`/`poly()` (audio.rs); `return_node`/`arg_input` (bindings_audio.rs).
- Produces: Wren foreign `Node.polyVelBegin_()` → returns a control-flagged velocity `PolyCtrl` and records it; `poly_end` returns `(pitch_ctrl, gate_ar, vel_node)`. Used by Task 3.

- [ ] **Step 1: `PolyCtx.vel_node` + `poly_vel_begin` + `poly_end` (audio.rs)**

Add `vel_node` to the struct (sentinel `NULL_ID` = none), reset it in `poly_begin`, add the vel-begin fn, and return it from `poly_end`:

```rust
struct PolyCtx {
    mode: bool,
    pitch_ctrl: u16,
    gate_ar: u16,
    gate_count: u8,
    vel_node: u16, // velocity PolyCtrl id, or NULL_ID if the builder didn't take velocity
}
```

- `PolyCtx::default`/init (audio.rs:85): add `vel_node: NULL_ID`.
- In `poly_begin`, after `p.gate_count = 0;` add `p.vel_node = NULL_ID;`.
- Add:

```rust
/// Create the per-voice velocity carrier (a second PolyCtrl) and record it.
/// Returns the PolyCtrl node id (the `vel` signal handed to the builder).
pub fn poly_vel_begin() -> u16 {
    let ctrl = alloc_node_id();
    new_node(ctrl, Kind::PolyCtrl, [Input::Const(0.0); 3]);
    poly().vel_node = ctrl;
    ctrl
}
```

- Change `poly_end` to return the vel id:

```rust
pub fn poly_end() -> (u16, u16, u16) {
    let p = poly();
    p.mode = false;
    (p.pitch_ctrl, p.gate_ar, p.vel_node)
}
```

- [ ] **Step 2: `node_poly_vel_begin_impl` + update `node_poly_end_impl` (bindings_audio.rs)**

```rust
pub(crate) fn node_poly_vel_begin_impl<S: SlotApi>(vm: &S) {
    let ctrl = audio::poly_vel_begin();
    unsafe { return_node(vm, ctrl) }; // return_node (NOT return_poly_node) → control-flagged, isPoly_==0
}
```

Update `node_poly_end_impl` to destructure the 3-tuple and pass velocity as `Option<NodeId>`:

```rust
pub(crate) fn node_poly_end_impl<S: SlotApi>(vm: &S) {
    let out = arg_input(vm, 1);
    let (pitch_ctrl, gate_ar, vel_raw) = audio::poly_end();
    let vel = if vel_raw == audio::NULL_ID { None } else { Some(NodeId(vel_raw)) };
    let sum = audio::alloc_node_id();
    audio::new_node(sum, Kind::VoiceSum, [out, Input::Const(0.0), Input::Const(0.0)]);
    let alloc = deluge_audio_graph::VoiceAllocator::new(NodeId(pitch_ctrl), NodeId(gate_ar), vel);
    unsafe { vm.new_foreign_in(0, SynthObj { alloc, out_node: sum }) };
}
```

> Confirm `NULL_ID` is reachable as `audio::NULL_ID` (it's `pub const` in audio.rs). Match the existing `NodeId(..)` construction style.

- [ ] **Step 3: Register `polyVelBegin_` in BOTH tables**

- `bindings_audio.rs` `register_audio` (beside `polyBegin_`): `method("main", "Node", false, "polyVelBegin_()", node_poly_vel_begin_impl::<S>);` — arity 0, static-on-Node (match how `polyBegin_` is registered: check whether it's `true`/`false` for the instance flag and mirror it exactly).
- `bindings.rs` `METHODS` table (the `wren-sys-backend` one — find where `polyBegin_` is listed and add `polyVelBegin_` alongside, same arity/signature).

- [ ] **Step 4: Verify it compiles + existing tests unchanged**

There's no new behavior yet (nothing calls `polyVelBegin_`), but `poly_end`'s signature change touches its one caller (`node_poly_end_impl`, updated above). Build + run the full wren-core suite both configs:

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` and again `--features deluge-dsp-kernels/simd`.
Expected: PASS — existing synths (arity-1) still build; `poly_end` now returns `NULL_ID` for vel → `None` → `VoiceAllocator::new(…, None)` → identical 2-Cmd note-on behavior.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs
git commit -m "feat(wren): polyVelBegin_ velocity carrier + poly_end returns vel_node"
```

---

## Task 3: `Synth.new` arity branch + end-to-end

**Files:**
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`Synth.new`, `foreign static polyVelBegin_()`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Node.polyVelBegin_()` (Task 2); the existing `Node.polyBegin_`/`polyEnd_`/`polyGateCount_`; Wren `Fn.arity`/`Fn.call`.
- Produces: `Synth.new { |pitch, vel| … }` (arity-2) support; `{ |pitch| … }` unchanged.

- [ ] **Step 1: Write the failing end-to-end tests**

In `tests/audio_bindings.rs` (mirror the existing Synth render harness — find `run_and_render`/`run_script_ok`; use NEWLINE-separated statements, this Wren dialect rejects `;`). Compare loudness of two velocities:

```rust
#[test]
fn synth_velocity_scales_amplitude() {
    // Arity-2 builder; velocity multiplies the amp. noteOn(60,127) must render a
    // higher RMS than noteOn(60,20). Use the harness helper that plays a note and
    // returns/samples output (mirror how an existing render test measures level).
    // Script (newline-separated):
    //   var s = Synth.new { |pitch, vel| Osc.saw(pitch) * Env.adsr(0.001,0.001,1,0.1) * vel }
    //   Out.patch(s.out)
    //   s.noteOn(60, 127)   → render → rms_hi
    //   (fresh) s.noteOn(60, 20) → render → rms_lo
    //   assert rms_hi > rms_lo (and both > 0)
}

#[test]
fn synth_arity1_still_builds_without_velocity_node() {
    // Backward-compat: an existing single-param synth builds + renders (no velocity node).
    assert!(run_script_ok(
        "var s = Synth.new { |p| Osc.saw(p) * Env.adsr(0.01,0.1,0.6,0.3) }\nOut.patch(s.out)\ns.noteOn(60,100)"
    ));
}

#[test]
fn synth_velocity_composes_as_control_signal() {
    // vel routes to cutoff and scales by a constant — exercises the Sy-2d control path.
    assert!(run_script_ok(
        "var s = Synth.new { |p, vel| Osc.saw(p).lpf(vel.to(400, 4000)) * Env.adsr(0.01,0.1,0.6,0.3) }\nOut.patch(s.out)\ns.noteOn(60,100)"
    ));
    assert!(run_script_ok(
        "var s = Synth.new { |p, vel| Osc.saw(p) * Env.adsr(0.01,0.1,0.6,0.3) * (vel * 0.5 + 0.5) }\nOut.patch(s.out)\ns.noteOn(60,100)"
    ));
}
```

> Use the exact harness helpers the existing render/level tests use. For `synth_velocity_scales_amplitude`, find how an existing test reads output level (an RMS/peak helper over rendered blocks) and reuse it; if the harness can't easily reset between two note-ons, build two separate Synths (one per velocity) and compare.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- synth_velocity_scales_amplitude synth_arity1_still_builds synth_velocity_composes`
Expected: `synth_velocity_*` FAIL — the arity-2 builder is called with 1 arg (Wren arity error) since `Synth.new` doesn't yet branch; `synth_arity1` may already pass.

- [ ] **Step 3: Add the `foreign static` decl + the arity branch (prelude.wren)**

In the `Node` class, add beside `polyBegin_`: `foreign static polyVelBegin_()`.

Replace `Synth.new`:

```wren
  static new(builder) {
    if (Node.polyMode_ == 1) Fiber.abort("nested Synth not supported")
    var pitch = Node.polyBegin_()
    var out
    if (builder.arity >= 2) {
      var vel = Node.polyVelBegin_()
      out = builder.call(pitch, vel)
    } else {
      out = builder.call(pitch)
    }
    if (Node.polyGateCount_ == 0) Fiber.abort("a Synth voice needs an Env.ar (the amp gate)")
    if (Node.polyGateCount_ > 1) Fiber.abort("multiple Env.ar in a Synth isn't supported yet")
    return Node.polyEnd_(out)
  }
```

> `builder.arity` is Wren's `Fn.arity` (parameter count). `builder.call(pitch, vel)` calls a 2-arg fn; `builder.call(pitch)` a 1-arg fn — the arity branch guarantees the call arity matches the closure, so no Wren arity abort.

- [ ] **Step 4: Run to verify they pass + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- synth_velocity_scales_amplitude synth_arity1_still_builds synth_velocity_composes` and again `--features deluge-dsp-kernels/simd`.
Then the FULL wren-core suite both configs — every existing Synth test (all arity-1) still builds/renders identically.
Expected: PASS both.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): Synth.new { |pitch, vel| } — velocity as optional 2nd builder param"
```

---

## Self-Review Notes (for the executor)

- **Non-breaking is the hard gate:** an arity-1 `Synth.new { |pitch| }` must create NO velocity node and behave bit-identically. The `poly_end` `NULL_ID → None` path and the `builder.arity` branch both enforce this — the `synth_arity1_still_builds` test plus the full-suite regression are the proof.
- **`vel` is control-flagged (`isPoly_==0`):** `node_poly_vel_begin_impl` uses `return_node`, NOT `return_poly_node` — so `* vel` is `PolyMul` (both Nodes) and `vel * k` uses the Sy-2d control-scaling path. Do not flag it poly.
- **Both registration tables:** `polyVelBegin_` must be in `register_audio` AND the `bindings.rs` METHODS table (the test VM boots the latter) — a miss there errors "metaclass does not implement" only under the test harness.
- **Emit order:** velocity `SetParam` goes between the pitch `SetParam` and the `GateVoice` — all three land in the same control batch before the next render block, so the voice renders at the right velocity from sample 0.
- **Deferred (do NOT implement):** portamento, multiple envelopes, release-tail protection, unison; a velocity curve primitive; Tb303/Modal poly filters; per-voice wavetable morph.
