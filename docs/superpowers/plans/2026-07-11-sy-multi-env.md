# Sy-5c: Multiple Envelopes per Voice Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Allow up to 4 note-gated envelopes in a `Synth`/`Synth.mono` voice by fanning `GateVoice` out from a single gate node to a small list — no new kernel, control-plane only.

**Architecture:** `PolyCtx` records a list of envelope node ids (not just the last); both `VoiceAllocator` and `MonoAllocator` replace their single `gate_node` with a `[NodeId; MAX_GATES]` + `n_gates`, and every `GateVoice` emit loops over the list; the Wren `polyGateCount_ > 1` guard relaxes to `> 4`.

**Tech Stack:** Rust `no_std`, `deluge-audio-graph` / `deluge-wren-core` (Wren scripting).

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** a 1-envelope Synth (`n_gates == 1`) builds/renders byte-identically to today (poly AND mono).
- **`MAX_GATES == 4`** (a new `pub const` in `deluge-audio-graph`, imported by wren-core).
- **Test invocation (per-crate, never `--workspace`; both configs; cargo rejects multiple bare positional names — use `-- name1 name2`):**
  - Graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## File Structure

- `crates/deluge-audio-graph/src/voice.rs` — `MAX_GATES`, both allocators' gate list + fan-out.
- `crates/deluge-audio-graph/src/lib.rs` — export `MAX_GATES`.
- `crates/deluge-wren-core/src/audio.rs` — `PolyCtx.gates`, `poly_record_gate`, `poly_end`/`mono_end`.
- `crates/deluge-wren-core/src/bindings_audio.rs` — `node_poly_end_impl`/`node_mono_end_impl` mapping.
- `crates/deluge-wren-core/wren/prelude.wren` — relaxed guard.
- Tests: `voice.rs` test module + `crates/deluge-wren-core/tests/audio_bindings.rs`.

---

## Task 1: Allocator gate fan-out (`voice.rs`)

**Files:**
- Modify: `crates/deluge-audio-graph/src/voice.rs`
- Modify: `crates/deluge-audio-graph/src/lib.rs` (export `MAX_GATES`)
- Test: `voice.rs` test module

**Interfaces:**
- Consumes: `Cmd::{SetParam, GateVoice, TriggerVoice}`, `NodeId`, `VOICES`, `A440_NOTE`, `MONO_STACK`.
- Produces: `pub const MAX_GATES: usize = 4;`; `VoiceAllocator::new(pitch_node: NodeId, gates: [NodeId; MAX_GATES], n_gates: usize, vel_node: Option<NodeId>)`; `MonoAllocator::new(pitch_node: NodeId, slew_node: NodeId, gates: [NodeId; MAX_GATES], n_gates: usize, vel_node: Option<NodeId>)`. Used by Task 3.

- [ ] **Step 1: Update existing tests to the new constructors + add fan-out tests**

Every existing `VoiceAllocator::new(NodeId(p), NodeId(g), vel)` and `MonoAllocator::new(NodeId(p), NodeId(s), NodeId(g), vel)` call site (in voice.rs tests AND engine.rs tests) becomes a 1-gate list. Add a helper in the voice.rs test module:

```rust
#[cfg(test)]
fn one_gate(g: u16) -> ([NodeId; MAX_GATES], usize) {
    let mut arr = [NodeId(0); MAX_GATES];
    arr[0] = NodeId(g);
    (arr, 1)
}
```

Update existing constructions, e.g. `VoiceAllocator::new(NodeId(10), NodeId(20), None)` → `{ let (g, n) = one_gate(20); VoiceAllocator::new(NodeId(10), g, n, None) }`. The `n_gates==1` emit sequences (Cmd order/count) must be IDENTICAL to today — the existing assertions stay unchanged.

New fan-out tests:

```rust
#[test]
fn poly_note_on_gates_all_envelopes_in_order() {
    let mut gates = [NodeId(0); MAX_GATES];
    gates[0] = NodeId(20); gates[1] = NodeId(21);
    let mut a = VoiceAllocator::new(NodeId(10), gates, 2, None);
    let c = on(&mut a, 69, 100); // pitch SetParam + GateVoice(20) + GateVoice(21)
    assert_eq!(c.len(), 3);
    assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), .. }));
    assert!(matches!(c[1], Cmd::GateVoice { node: NodeId(20), voice: 0, on: true }));
    assert!(matches!(c[2], Cmd::GateVoice { node: NodeId(21), voice: 0, on: true }));
}

#[test]
fn poly_note_off_releases_all_envelopes() {
    let mut gates = [NodeId(0); MAX_GATES];
    gates[0] = NodeId(20); gates[1] = NodeId(21);
    let mut a = VoiceAllocator::new(NodeId(10), gates, 2, None);
    on(&mut a, 69, 100);
    let c = off(&mut a, 69);
    assert_eq!(c.len(), 2);
    assert!(matches!(c[0], Cmd::GateVoice { node: NodeId(20), on: false, .. }));
    assert!(matches!(c[1], Cmd::GateVoice { node: NodeId(21), on: false, .. }));
}

#[test]
fn mono_from_silence_gates_all_envelopes_legato_gates_none() {
    let mut gates = [NodeId(0); MAX_GATES];
    gates[0] = NodeId(30); gates[1] = NodeId(31);
    let mut m = MonoAllocator::new(NodeId(10), NodeId(20), gates, 2, None);
    let c = mon(&mut m, 60, 100); // pitch SetParam + TriggerVoice(slew 20) + GateVoice(30) + GateVoice(31)
    assert_eq!(c.len(), 4);
    assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), .. }));
    assert!(matches!(c[1], Cmd::TriggerVoice { node: NodeId(20), voice: 0 }));
    assert!(matches!(c[2], Cmd::GateVoice { node: NodeId(30), voice: 0, on: true }));
    assert!(matches!(c[3], Cmd::GateVoice { node: NodeId(31), voice: 0, on: true }));
    // legato note: pitch SetParam ONLY, no gate to any envelope
    let c2 = mon(&mut m, 64, 100);
    assert_eq!(c2.len(), 1);
    assert!(matches!(c2[0], Cmd::SetParam { node: NodeId(10), .. }));
}
```

> Match the existing `on`/`off`/`mon`/`moff` capture-helper style. Confirm `NodeId` construction (`NodeId(0)`) matches the tuple-struct form used in the file.

- [ ] **Step 2: Run to verify they fail (compile error / wrong arity)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- poly_note_on_gates_all poly_note_off_releases_all mono_from_silence_gates_all`
Expected: FAIL to COMPILE — `VoiceAllocator::new`/`MonoAllocator::new` take the old arity.

- [ ] **Step 3: Add `MAX_GATES` + the gate list + fan-out**

`pub const MAX_GATES: usize = 4;` near the top of voice.rs.

`VoiceAllocator`: replace `gate_node: NodeId` with `gates: [NodeId; MAX_GATES]` + `n_gates: usize`; constructor:

```rust
pub fn new(pitch_node: NodeId, gates: [NodeId; MAX_GATES], n_gates: usize, vel_node: Option<NodeId>) -> VoiceAllocator {
    VoiceAllocator { pitch_node, gates, n_gates, vel_node, lane_note: [None; VOICES], lane_age: [0; VOICES], clock: 0 }
}
```

Replace each `emit(Cmd::GateVoice { node: self.gate_node, voice: lane as u8, on: X })` with a fan-out helper — add a private method and call it:

```rust
fn gate_all(&self, lane: usize, on: bool, emit: &mut impl FnMut(Cmd)) {
    for g in &self.gates[..self.n_gates] {
        emit(Cmd::GateVoice { node: *g, voice: lane as u8, on });
    }
}
```

Use `self.gate_all(lane, true, emit)` in `note_on` (after pitch/vel), `self.gate_all(lane, false, emit)` in `note_off`, and in `all_notes_off` for each held lane. The emit ORDER stays: pitch `SetParam`, [velocity `SetParam`], then the gates.

`MonoAllocator`: same — replace `gate_node` with `gates`/`n_gates`, constructor gains the two params, add the same `gate_all(lane, on, emit)` helper (lane is always 0 for mono), and use it in: `note_on` from-silence branch (after `TriggerVoice(slew)`), `note_off` stack-empty branch, and `all_notes_off`. The legato branch and the glide-back branch emit NO gate (unchanged). `TriggerVoice(slew_node, 0)` stays a single emit (the slew is one node, not a gate).

- [ ] **Step 4: Export `MAX_GATES` + run**

In `crates/deluge-audio-graph/src/lib.rs`, add `MAX_GATES` to the `pub use voice::{…}` line: `pub use voice::{MAX_GATES, MonoAllocator, VoiceAllocator};`.

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph` and again `--features deluge-dsp-kernels/simd`.
Expected: PASS both — new fan-out tests green, all existing 1-gate tests (updated to the new constructor) green.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src/voice.rs crates/deluge-audio-graph/src/lib.rs
git commit -m "feat(graph): gate fan-out — VoiceAllocator/MonoAllocator drive up to MAX_GATES envelopes"
```

---

## Task 2: `PolyCtx` gate list + build-path mapping (`audio.rs` + `bindings_audio.rs`)

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs`
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs`

**Interfaces:**
- Consumes: `deluge_audio_graph::MAX_GATES` + `VoiceAllocator::new`/`MonoAllocator::new` (Task 1), `NULL_ID`.
- Produces: `PolyCtx.gates: [u16; MAX_GATES]` + `gate_count`; `poly_end() -> (u16 /*pitch*/, [u16; MAX_GATES] /*gates*/, u8 /*count*/, u16 /*vel*/)`; `mono_end() -> (u16, u16 /*slew*/, [u16; MAX_GATES], u8, u16)`; updated `node_poly_end_impl`/`node_mono_end_impl`. Used by Task 3.

> This task changes `poly_end`/`mono_end`'s signatures AND their only callers (`node_poly_end_impl`/`node_mono_end_impl`) — they must land atomically (the crate won't compile between them), so `audio.rs` and `bindings_audio.rs` are one task/commit.

- [ ] **Step 1: `PolyCtx.gates` + bounded `poly_record_gate`**

Add `use deluge_audio_graph::MAX_GATES;` to audio.rs. Change `PolyCtx`:

```rust
struct PolyCtx {
    mode: bool,
    pitch_ctrl: u16,
    gates: [u16; MAX_GATES], // envelope node ids (first `gate_count`)
    gate_count: u8,          // total envelopes created this build (may exceed MAX_GATES → guard aborts)
    vel_node: u16,
    slew_node: u16,
}
```

`PolyCtx` constructor/default: `gates: [NULL_ID; MAX_GATES]`, `gate_count: 0` (keep the other fields).

`poly_record_gate` — bounded push:

```rust
pub fn poly_record_gate(id: u16) {
    let p = poly();
    if (p.gate_count as usize) < MAX_GATES {
        p.gates[p.gate_count as usize] = id;
    }
    p.gate_count = p.gate_count.saturating_add(1);
}
```

- [ ] **Step 2: Reset in both begins + return the list**

In `poly_begin` and `mono_begin`, where `gate_count` is reset to 0, also reset the array: `p.gates = [NULL_ID; MAX_GATES];` (or rely on `gate_count` bounding reads — but reset the array too for cleanliness). Keep the existing `gate_count = 0`.

Change the end functions:

```rust
pub fn poly_end() -> (u16, [u16; MAX_GATES], u8, u16) {
    let p = poly();
    p.mode = false;
    (p.pitch_ctrl, p.gates, p.gate_count, p.vel_node)
}
pub fn mono_end() -> (u16, u16, [u16; MAX_GATES], u8, u16) {
    let p = poly();
    p.mode = false;
    (p.pitch_ctrl, p.slew_node, p.gates, p.gate_count, p.vel_node)
}
```

- [ ] **Step 3: Map the gate ids in the end-impls (`bindings_audio.rs`)**

Update the two callers of `poly_end`/`mono_end` in the SAME task (the signature change + callers land together). `node_poly_end_impl`:

```rust
pub(crate) fn node_poly_end_impl<S: SlotApi>(vm: &S) {
    let out = arg_input(vm, 1);
    let (pitch_ctrl, gates_raw, gate_count, vel_raw) = audio::poly_end();
    let vel = if vel_raw == audio::NULL_ID { None } else { Some(NodeId(vel_raw)) };
    let gates: [NodeId; deluge_audio_graph::MAX_GATES] = core::array::from_fn(|i| NodeId(gates_raw[i]));
    let n_gates = (gate_count as usize).min(deluge_audio_graph::MAX_GATES);
    let sum = audio::alloc_node_id();
    audio::new_node(sum, Kind::VoiceSum, [out, Input::Const(0.0), Input::Const(0.0)]);
    let alloc = SynthAlloc::Poly(deluge_audio_graph::VoiceAllocator::new(NodeId(pitch_ctrl), gates, n_gates, vel));
    unsafe { vm.new_foreign_in(0, SynthObj { alloc, out_node: sum }) };
}
```

`node_mono_end_impl`:

```rust
pub(crate) fn node_mono_end_impl<S: SlotApi>(vm: &S) {
    let out = arg_input(vm, 1);
    let (pitch, slew, gates_raw, gate_count, vel_raw) = audio::mono_end();
    let vel = if vel_raw == audio::NULL_ID { None } else { Some(NodeId(vel_raw)) };
    let gates: [NodeId; deluge_audio_graph::MAX_GATES] = core::array::from_fn(|i| NodeId(gates_raw[i]));
    let n_gates = (gate_count as usize).min(deluge_audio_graph::MAX_GATES);
    let sum = audio::alloc_node_id();
    audio::new_node(sum, Kind::VoiceSum, [out, Input::Const(0.0), Input::Const(0.0)]);
    let alloc = SynthAlloc::Mono(deluge_audio_graph::MonoAllocator::new(NodeId(pitch), NodeId(slew), gates, n_gates, vel));
    unsafe { vm.new_foreign_in(0, SynthObj { alloc, out_node: sum }) };
}
```

> `n_gates` is `gate_count.min(MAX_GATES)` — a build with >4 envelopes still reaches here (the Wren guard aborts BEFORE `monoEnd_`/`polyEnd_` for >4 only after Task 3; the `.min` is defensive so this task alone never overruns). At this point `polyGateCount_ >= 1` is guaranteed by the existing `== 0` guard, so `n_gates >= 1`.

- [ ] **Step 4: Run + commit**

Run the full wren-core suite both configs:
`cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` and `--features deluge-dsp-kernels/simd`.
Expected: PASS both — existing 1-envelope synths unchanged (`gate_count == 1` → `n_gates == 1` → single-gate emit, byte-identical to today). No Wren guard change yet (Task 3), so existing 1-env tests still pass and any >1 test still aborts (the old `> 1` guard is still in place until the next task).

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs
git commit -m "feat(wren): PolyCtx records a gate list; end-impls pass it to the allocators"
```

---

## Task 3: Relax the Wren guard + end-to-end

**Files:**
- Modify: `crates/deluge-wren-core/wren/prelude.wren`
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: the gate-list build path (Tasks 1–3); `Node.polyGateCount_`.
- Produces: `Synth.new`/`Synth.mono` accept 1–4 envelopes.

- [ ] **Step 1: Write the failing e2e tests**

In `tests/audio_bindings.rs` (mirror the existing Synth harness; NEWLINE-separated statements — this Wren dialect rejects `;`):

```rust
#[test]
fn synth_two_envelopes_amp_and_filter_builds_and_renders() {
    assert!(run_script_ok(
        "var s = Synth.new { |p| Osc.saw(p).lpf(Env.adsr(0.01,0.2,0.3,0.4).to(400,4000)) * Env.adsr(0.005,0.1,0.7,0.2) }\nOut.patch(s.out)\ns.noteOn(60,100)"
    ));
}

#[test]
fn synth_four_envelopes_builds() {
    // amp + 3 mod envelopes (routed harmlessly) — at the cap
    assert!(run_script_ok(
        "var s = Synth.new { |p| var e2 = Env.adsr(0.01,0.1,0.5,0.2)\nvar e3 = Env.adsr(0.01,0.1,0.5,0.2)\nvar e4 = Env.adsr(0.01,0.1,0.5,0.2)\nOsc.saw(p).lpf(e2.to(400,4000) + e3*0 + e4*0) * Env.adsr(0.005,0.1,0.7,0.2) }\nOut.patch(s.out)\ns.noteOn(60,100)"
    ));
}

#[test]
fn synth_five_envelopes_aborts() {
    assert!(!run_script_ok(
        "var s = Synth.new { |p| var a = Env.adsr(0.01,0.1,0.5,0.2)\nvar b = Env.adsr(0.01,0.1,0.5,0.2)\nvar c = Env.adsr(0.01,0.1,0.5,0.2)\nvar d = Env.adsr(0.01,0.1,0.5,0.2)\nOsc.saw(p) * a * b * c * d * Env.adsr(0.005,0.1,0.7,0.2) }"
    ));
}

#[test]
fn synth_zero_envelopes_still_aborts() {
    assert!(!run_script_ok("var s = Synth.new { |p| Osc.saw(p) }"));
}

#[test]
fn synth_mono_two_envelopes_builds_and_renders() {
    assert!(run_script_ok(
        "var s = Synth.mono { |p| Osc.saw(p).lpf(Env.adsr(0.01,0.2,0.3,0.4).to(400,4000)) * Env.adsr(0.005,0.1,0.7,0.2) }\nOut.patch(s.out)\ns.noteOn(60,100)"
    ));
}
```

> Adjust the exact builder expressions to whatever the DSL supports for combining/routing extra envelopes harmlessly (the point is `polyGateCount_` counts them, not the audio routing). `e3*0` etc. keep the extra envelopes in the graph (so they're recorded) while contributing nothing audible — if `*0` on an env is rejected, route them via any accepted no-effect path, or simply reference them so they're created. The four-env test only needs FOUR `Env.adsr` created in the builder.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- synth_two_envelopes synth_four_envelopes synth_five_envelopes synth_mono_two`
Expected: `synth_two_envelopes`/`synth_four_envelopes`/`synth_mono_two` FAIL (current guard aborts at `> 1`); `synth_five`/`synth_zero` may already pass (they abort).

- [ ] **Step 3: Relax the guard**

In `prelude.wren`, in BOTH `Synth.new` and `Synth.mono`, change `> 1` to `> 4`:

```wren
    if (Node.polyGateCount_ == 0) Fiber.abort("a Synth voice needs an Env.ar (the amp gate)")
    if (Node.polyGateCount_ > 4)  Fiber.abort("more than 4 envelopes per voice isn't supported")
```

(Both the `== 0` and the relaxed `> 4` lines, in both constructors.)

- [ ] **Step 4: Run to verify they pass + full regression**

Run the e2e tests both configs, then the FULL wren-core suite both configs. Existing 1-envelope synths unchanged; 2–4 build; 5 aborts; 0 aborts.
Expected: PASS both.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): allow up to 4 envelopes per Synth voice (relax gate-count guard 1→4)"
```

---

## Self-Review Notes (for the executor)

- **Non-breaking is the hard gate:** `n_gates == 1` must reproduce today's exact emit sequences (order: pitch, [vel], [trigger for mono-from-silence], then the single gate). The updated-to-1-gate existing allocator tests + the full wren-core regression are the proof.
- **Task 2 spans `audio.rs` + `bindings_audio.rs` in one commit:** the `poly_end`/`mono_end` signature change and its callers (`node_poly_end_impl`/`node_mono_end_impl`) must land atomically (the crate won't compile between them otherwise).
- **Bounded push:** `poly_record_gate` writes only when `gate_count < MAX_GATES` but always increments `gate_count`, so a >4-envelope build still trips the Wren `> 4` guard (which fires before `polyEnd_`) — the array is never overrun.
- **All envelopes equal:** every recorded envelope is gated identically. The amp is just whichever the builder multiplies into the signal; the filter env is whichever routes to cutoff. No special-casing in the allocator.
- **Mono legato composes:** the fan-out only touches the from-silence and to-silence gate paths; the legato and glide-back paths stay gate-free (true legato holds across all envelopes).
- **Confirm exact names** (`NodeId(..)`, `Cmd` fields, `core::array::from_fn`, the `on`/`mon` test helpers) against the codebase before transcribing.
- **Deferred (do NOT implement):** release-tail (5d), unison (5e), poly-per-voice glide, Tb303/Modal poly, per-voice wavetable morph.
