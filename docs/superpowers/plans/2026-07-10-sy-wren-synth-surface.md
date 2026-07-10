# Sy-4: Wren `Synth` Surface + MIDI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the merged poly voice machinery usable from a Wren patch — a `Synth.new { |pitch| … }` builder where plain factories emit the poly kinds via a poly-context flag, backed by a `Synth` foreign object owning a `VoiceAllocator`, driven by MIDI.

**Architecture:** A `poly_mode` flag + voice-build state in the wren-core audio module. New poly foreign factories (`polyosc_`/`polysvf_`/`polyar_`/`polymul_` + `polyMode_`/`polyBegin_`/`polyEnd_`/`polyGateCount_`) mirror the existing `node_*_impl` pattern. `polyEnd_` builds a `Synth` foreign object (a `SynthObj` holding a `VoiceAllocator`). The prelude routes the poly-capable factories on `Node.polyMode_ == 1` and `Fiber.abort`s the unsupported ones inside a Synth.

**Tech Stack:** Rust `no_std` (`deluge-wren-core`), Wren (prelude), `deluge-audio-graph` (`VoiceAllocator`, poly kinds).

## Global Constraints

- Host tests: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core`; the render tests also `--features deluge-dsp-kernels/simd`. Never `--workspace` on host. LSP armv7a diagnostics are noise.
- **Wren has NO `?:` ternary** — use `if (cond) return A` then `return B`. **In Wren `0` is truthy** (only `false`/`null` are falsy) — so `polyMode_`/`polyGateCount_` return a **number**, compared `== 1` / `== 0` / `> 1` in the prelude.
- Register every new foreign method in **both** tables: `bindings.rs` `METHODS` (wren-sys) AND `bindings_audio.rs::register_audio`. Static factories use `static_method`/`true`; instance methods use `method`/`false`.
- The audio VM is single-threaded — poly-context state is a `static mut` reached via a `&'static mut` accessor, like the existing `ALLOC`.
- `polyBegin_` **resets** poly state fresh (so an aborted prior build doesn't poison the next); no Rust-side fiber abort is available, so all user-facing errors are `Fiber.abort` in the prelude.
- MVP poly-capable set: `Osc.sine`, `.lpf` (→ `PolySvf`, default res 0.2), `Env.ar`, `*` (poly×poly VCA). Everything else aborts inside a Synth.
- Follow the existing patterns exactly: `node_lfo_impl<S>` + `#[cfg(feature = "wren-sys-backend")]` extern wrapper; `arg_input`/`return_node`/`self_id`; `NodeObj` foreign shape.

---

### Task 1: Poly-context state + poly factories

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (poly-context state + helpers)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (poly factory impls + register_audio)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (METHODS)

**Interfaces:**
- Consumes: `audio::{alloc_node_id, new_node, set_param}`, `deluge_audio_graph::{Kind, Input, NodeId}`, `arg_input`, `return_node`, the Sy-1–3 poly kinds.
- Produces (Rust): `audio::poly_*` state accessors; foreign impls `node_poly_mode_impl`, `node_poly_begin_impl`, `node_polyosc_impl`, `node_polysvf_impl`, `node_polyar_impl`, `node_polymul_impl`, `node_poly_gate_count_impl` (each + a `#[cfg(feature = "wren-sys-backend")]` extern wrapper). Wren foreign methods `Node.polyMode_`, `polyBegin_()`, `polyGateCount_`, `polyosc_(_)`, `polysvf_(_,_,_)`, `polyar_(_,_)`, `polymul_(_,_)`.

- [ ] **Step 1: Add poly-context state (audio.rs)**

Below the existing `static mut ALLOC` block, add:

```rust
/// Voice-build state for `Synth.new` (set between polyBegin_/polyEnd_).
struct PolyCtx {
    mode: bool,
    pitch_ctrl: u16, // the PolyCtrl created by polyBegin_
    gate_ar: u16,    // the PolyAr recorded by polyar_ (amp gate)
    gate_count: u8,  // number of Env.ar created this build (must be 1)
}
impl PolyCtx {
    const fn new() -> Self {
        PolyCtx { mode: false, pitch_ctrl: NULL_ID, gate_ar: NULL_ID, gate_count: 0 }
    }
}
static mut POLY: PolyCtx = PolyCtx::new();

#[allow(static_mut_refs)]
fn poly() -> &'static mut PolyCtx {
    // SAFETY: single-threaded VM, same discipline as `ALLOC`.
    unsafe { &mut POLY }
}

pub fn poly_mode() -> bool {
    poly().mode
}
pub fn poly_gate_count() -> u8 {
    poly().gate_count
}
/// Begin a voice build: reset state fresh (clears any stale flag from an
/// aborted prior build), create the PolyCtrl pitch source + PolyMtof, return
/// the PolyMtof id (the `pitch` node).
pub fn poly_begin() -> u16 {
    let ctrl = alloc_node_id();
    new_node(ctrl, Kind::PolyCtrl, [Input::Const(0.0); 3]);
    let mtof = alloc_node_id();
    new_node(mtof, Kind::PolyMtof, [Input::Node { node: NodeId(ctrl), port: 0 }, Input::Const(0.0), Input::Const(0.0)]);
    let p = poly();
    p.mode = true;
    p.pitch_ctrl = ctrl;
    p.gate_ar = NULL_ID;
    p.gate_count = 0;
    mtof
}
/// Record a PolyAr as the voice's amp gate.
pub fn poly_record_gate(id: u16) {
    let p = poly();
    p.gate_ar = id;
    p.gate_count = p.gate_count.saturating_add(1);
}
/// End a voice build: clear the flag; returns (pitch_ctrl, gate_ar) for the allocator.
pub fn poly_end() -> (u16, u16) {
    let p = poly();
    p.mode = false;
    (p.pitch_ctrl, p.gate_ar)
}
```

(`NULL_ID`, `alloc_node_id`, `new_node`, `NodeId`, `Kind`, `Input` are already in scope in `audio.rs`.)

Extend the existing `pub fn reset()` (audio.rs:248) to clear the poly context too
(so a test script that leaves `poly_mode` set — e.g. a raw-factory test with no
`polyEnd_` — does not poison the next `crate::reset()`-bounded run):

```rust
pub fn reset() {
    alloc().reset();
    *poly() = PolyCtx::new();
    host().audio_cmd(Cmd::Reset);
}
```

- [ ] **Step 2: Add the poly factory impls (bindings_audio.rs)**

After the existing Mod-3/Sy factory impls, add (mirroring `node_lfo_impl` + its extern wrapper — every one needs the `#[cfg(feature = "wren-sys-backend")]` `unsafe extern "C" fn` wrapper too):

```rust
/// `Node.polyMode_` — 1.0 while inside a Synth build, else 0.0 (number, since
/// Wren treats 0 as truthy — the prelude compares `== 1`).
pub(crate) fn node_poly_mode_impl<S: SlotApi>(vm: &S) {
    vm.set_f(0, if audio::poly_mode() { 1.0 } else { 0.0 });
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_poly_mode(raw: *mut WrenVM) { let vm = Vm(raw); node_poly_mode_impl(&vm); }

/// `Node.polyGateCount_` — number of Env.ar created this build.
pub(crate) fn node_poly_gate_count_impl<S: SlotApi>(vm: &S) {
    vm.set_f(0, audio::poly_gate_count() as f64);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_poly_gate_count(raw: *mut WrenVM) { let vm = Vm(raw); node_poly_gate_count_impl(&vm); }

/// `Node.polyBegin_()` — start a voice build; returns the `pitch` node (PolyMtof).
pub(crate) fn node_poly_begin_impl<S: SlotApi>(vm: &S) {
    let mtof = audio::poly_begin();
    unsafe { return_node(vm, mtof) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_poly_begin(raw: *mut WrenVM) { let vm = Vm(raw); node_poly_begin_impl(&vm); }

/// `Node.polyosc_(pitch)` — poly sine oscillator. Ports 0 = pitch (poly Hz).
pub(crate) fn node_polyosc_impl<S: SlotApi>(vm: &S) {
    let pitch = arg_input(vm, 1);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolyOsc, [pitch, Input::Const(0.0), Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_polyosc(raw: *mut WrenVM) { let vm = Vm(raw); node_polyosc_impl(&vm); }

/// `Node.polysvf_(audio, cutoff, res)` — poly SVF lowpass. Port 0 = audio (poly),
/// ports 1/2 = cutoff/res (mono).
pub(crate) fn node_polysvf_impl<S: SlotApi>(vm: &S) {
    let audio_in = arg_input(vm, 1);
    let cutoff = arg_input(vm, 2);
    let res = arg_input(vm, 3);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolySvf, [audio_in, cutoff, res]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_polysvf(raw: *mut WrenVM) { let vm = Vm(raw); node_polysvf_impl(&vm); }

/// `Node.polyar_(attack, release)` — poly AR envelope (the amp gate). Ports
/// 0/1 = attack/release (mono). Records itself as the voice's gate.
pub(crate) fn node_polyar_impl<S: SlotApi>(vm: &S) {
    let attack = arg_input(vm, 1);
    let release = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolyAr, [attack, release, Input::Const(0.0)]);
    audio::poly_record_gate(id);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_polyar(raw: *mut WrenVM) { let vm = Vm(raw); node_polyar_impl(&vm); }

/// `Node.polymul_(a, b)` — poly × poly (the VCA). Ports 0/1 = both poly.
pub(crate) fn node_polymul_impl<S: SlotApi>(vm: &S) {
    let a = arg_input(vm, 1);
    let b = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolyMul, [a, b, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_polymul(raw: *mut WrenVM) { let vm = Vm(raw); node_polymul_impl(&vm); }
```

- [ ] **Step 3: Register in both tables**

In `bindings_audio.rs::register_audio`, add (near the other `Node` methods):

```rust
    method("main", "Node", true, "polyMode_", node_poly_mode_impl::<S>);
    method("main", "Node", true, "polyGateCount_", node_poly_gate_count_impl::<S>);
    method("main", "Node", true, "polyBegin_()", node_poly_begin_impl::<S>);
    method("main", "Node", true, "polyosc_(_)", node_polyosc_impl::<S>);
    method("main", "Node", true, "polysvf_(_,_,_)", node_polysvf_impl::<S>);
    method("main", "Node", true, "polyar_(_,_)", node_polyar_impl::<S>);
    method("main", "Node", true, "polymul_(_,_)", node_polymul_impl::<S>);
```

In `bindings.rs::METHODS`, add:

```rust
    static_method("Node", "polyMode_", bindings_audio::node_poly_mode),
    static_method("Node", "polyGateCount_", bindings_audio::node_poly_gate_count),
    static_method("Node", "polyBegin_()", bindings_audio::node_poly_begin),
    static_method("Node", "polyosc_(_)", bindings_audio::node_polyosc),
    static_method("Node", "polysvf_(_,_,_)", bindings_audio::node_polysvf),
    static_method("Node", "polyar_(_,_)", bindings_audio::node_polyar),
    static_method("Node", "polymul_(_,_)", bindings_audio::node_polymul),
```

Add the `foreign static` declarations to `foreign class Node` in `wren/prelude.wren`:

```wren
  foreign static polyMode_
  foreign static polyGateCount_
  foreign static polyBegin_()
  foreign static polyosc_(pitch)
  foreign static polysvf_(audio, cutoff, res)
  foreign static polyar_(attack, release)
  foreign static polymul_(a, b)
```

- [ ] **Step 4: Test the factories (bindings_audio tests)**

In `crates/deluge-wren-core/tests/audio_bindings.rs`, add (capture the emitted `Cmd`s via `run_and_capture_cmds`, driving the raw factories directly since prelude routing lands in Task 3):

```rust
#[test]
fn poly_factories_emit_poly_kinds() {
    let cmds = run_and_capture_cmds(
        "var p = Node.polyBegin_()\nvar o = Node.polyosc_(p)\nvar f = Node.polysvf_(o, 1200, 0.2)\nvar e = Node.polyar_(0.01, 0.3)\nvar v = Node.polymul_(f, e)",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyCtrl, .. })), "PolyCtrl");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyMtof, .. })), "PolyMtof");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyOsc, .. })), "PolyOsc");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolySvf, .. })), "PolySvf");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyAr, .. })), "PolyAr");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyMul, .. })), "PolyMul");
}

#[test]
fn poly_mode_reflects_build_state() {
    // polyMode_ is 1 during a build, 0 after (script leaves it set; a follow-up
    // script starts fresh — poly_begin resets). Here: 1 right after polyBegin_.
    let cmds = run_and_capture_cmds("var m = Node.polyMode_\nvar p = Node.polyBegin_()");
    // Nothing to assert on m directly via Cmds; the render/error behavior in
    // Task 3 exercises polyMode_. This test just confirms polyBegin_ emits nodes.
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyCtrl, .. })));
}
```

- [ ] **Step 5: Run + commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core poly_factories poly_mode`
Expected: PASS. No warnings.

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): poly-context state + poly node factories"
```

---

### Task 2: `SynthObj` foreign object + `polyEnd_` + note methods

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (SynthObj + polyEnd_ + synth methods)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (METHODS)

**Interfaces:**
- Consumes: `deluge_audio_graph::VoiceAllocator` (Sy-3), `audio::{poly_end, new_node, alloc_node_id, audio_cmd}` or `host().audio_cmd`, `return_node`, `NodeObj`.
- Produces: `SynthObj` (WrenForeign, class "Synth"); `node_poly_end_impl(out)` → a `Synth`; `synth_note_on_impl(note, vel)`, `synth_note_off_impl(note)`, `synth_out_impl`. Wren `foreign class Synth` with `noteOn(_,_)`, `noteOff(_)`, `out`, `polyEnd_(_)` (static on Node).

- [ ] **Step 1: Define `SynthObj` (bindings_audio.rs)**

Near `NodeObj`:

```rust
/// A polyphonic instrument: owns a VoiceAllocator + its VoiceSum output node.
/// Created by `Node.polyEnd_`; not used as a node input (no shared tag).
#[repr(C)]
pub(crate) struct SynthObj {
    pub alloc: deluge_audio_graph::VoiceAllocator,
    pub out_node: u16,
}
impl WrenForeign for SynthObj {
    fn module_name() -> &'static str { "main" }
    fn class_name() -> &'static str { "Synth" }
}
```

- [ ] **Step 2: `polyEnd_` + synth methods (bindings_audio.rs)**

```rust
/// `Node.polyEnd_(out)` — finish a voice: VoiceSum(out) → build a VoiceAllocator
/// into a Synth foreign object (in slot 0). The prelude has already validated
/// exactly-one Env.ar via polyGateCount_.
pub(crate) fn node_poly_end_impl<S: SlotApi>(vm: &S) {
    let out = arg_input(vm, 1);
    let (pitch_ctrl, gate_ar) = audio::poly_end();
    let sum = audio::alloc_node_id();
    audio::new_node(sum, Kind::VoiceSum, [out, Input::Const(0.0), Input::Const(0.0)]);
    let alloc = deluge_audio_graph::VoiceAllocator::new(NodeId(pitch_ctrl), NodeId(gate_ar));
    unsafe { vm.new_foreign_in(0, SynthObj { alloc, out_node: sum }) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_poly_end(raw: *mut WrenVM) { let vm = Vm(raw); node_poly_end_impl(&vm); }

fn self_synth<S: SlotApi>(vm: &S) -> &mut SynthObj {
    unsafe { vm.foreign_mut::<SynthObj>(0) }
}

pub(crate) fn synth_note_on_impl<S: SlotApi>(vm: &S) {
    let note = vm.get_f(1) as u8;
    let vel = vm.get_f(2) as u8;
    self_synth(vm).alloc.note_on(note, vel, &mut |c| crate::host::host().audio_cmd(c));
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn synth_note_on(raw: *mut WrenVM) { let vm = Vm(raw); synth_note_on_impl(&vm); }

pub(crate) fn synth_note_off_impl<S: SlotApi>(vm: &S) {
    let note = vm.get_f(1) as u8;
    self_synth(vm).alloc.note_off(note, &mut |c| crate::host::host().audio_cmd(c));
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn synth_note_off(raw: *mut WrenVM) { let vm = Vm(raw); synth_note_off_impl(&vm); }

/// `synth.out` — the mono VoiceSum node, for routing (`Out.patch(synth.out)`).
pub(crate) fn synth_out_impl<S: SlotApi>(vm: &S) {
    let id = self_synth(vm).out_node;
    unsafe { return_node(vm, id) }; // return_node overwrites slot 0 with a NodeObj
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn synth_out(raw: *mut WrenVM) { let vm = Vm(raw); synth_out_impl(&vm); }
```

(Confirm `crate::host::host().audio_cmd(cmd)` is the emit path — it is the same call the `audio::*` emitters use. If `audio_cmd` takes `&self`, the closure `|c| host().audio_cmd(c)` works.)

- [ ] **Step 3: Register (register_audio + METHODS) + prelude `foreign class Synth`**

`register_audio`:
```rust
    method("main", "Node", true, "polyEnd_(_)", node_poly_end_impl::<S>);
    method("main", "Synth", false, "noteOn(_,_)", synth_note_on_impl::<S>);
    method("main", "Synth", false, "noteOff(_)", synth_note_off_impl::<S>);
    method("main", "Synth", false, "out", synth_out_impl::<S>);
```

`METHODS`:
```rust
    static_method("Node", "polyEnd_(_)", bindings_audio::node_poly_end),
    method("Synth", "noteOn(_,_)", bindings_audio::synth_note_on),
    method("Synth", "noteOff(_)", bindings_audio::synth_note_off),
    method("Synth", "out", bindings_audio::synth_out),
```

In `prelude.wren`, add the `foreign static polyEnd_(out)` to `class Node` and a `foreign class Synth` skeleton (the full `Synth` class with `new`/`bindMidi` lands in Task 3; here just the foreign declarations so the methods bind):

```wren
foreign class Synth {
  foreign noteOn(note, vel)
  foreign noteOff(note)
  foreign out
}
```

(Mirror the `NodeObj`/`foreign class Node` registration exactly. If wren-sys requires the foreign class to be reachable before `new_foreign_in`, ensure `foreign class Synth` is declared in the prelude that loads before any script — same as `Node`.)

- [ ] **Step 4: Test polyEnd + noteOn emits (audio_bindings.rs)**

```rust
#[test]
fn synth_note_on_emits_pitch_and_gate() {
    let cmds = run_and_capture_cmds(
        "var p = Node.polyBegin_()\nvar v = Node.polymul_(Node.polysvf_(Node.polyosc_(p), 1200, 0.2), Node.polyar_(0.01, 0.3))\nvar s = Node.polyEnd_(v)\ns.noteOn(69, 100)",
    );
    // VoiceSum built at polyEnd_.
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::VoiceSum, .. })), "VoiceSum");
    // note_on → SetParam(pitch lane 0 = note-69 = 0) + GateVoice(gate, 0, true).
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if value.abs() < 1e-6)), "pitch");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::GateVoice { voice: 0, on: true, .. })), "gate on");
}
```

- [ ] **Step 5: Run + commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core synth_note_on`
Expected: PASS. No warnings.

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): SynthObj + polyEnd_ + noteOn/noteOff/out"
```

---

### Task 3: Prelude routing + `Synth` class + errors

**Files:**
- Modify: `crates/deluge-wren-core/wren/prelude.wren`
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: the Task 1/2 foreign methods (`Node.polyMode_`, `polyosc_`, `polysvf_`, `polyar_`, `polymul_`, `polyBegin_`, `polyEnd_`, `polyGateCount_`) and `foreign class Synth`.
- Produces: poly-routed `Osc`/`.lpf`/`Env.ar`/`*` + `Fiber.abort` guards; `class Synth` with `static new(builder)` + `bindMidi()`.

- [ ] **Step 1: Route the poly-capable factories (prelude.wren)**

`class Osc` — route `sine`, abort the others in poly mode:
```wren
class Osc {
  static sine(f) {
    if (Node.polyMode_ == 1) return Node.polyosc_(f)
    return Node.src_(0, f)
  }
  static saw(f) {
    if (Node.polyMode_ == 1) Fiber.abort("Osc.saw not usable in a Synth yet (Sy-2b poly breadth)")
    return Node.src_(1, f)
  }
  static square(f) {
    if (Node.polyMode_ == 1) Fiber.abort("Osc.square not usable in a Synth yet (Sy-2b)")
    return Node.src_(2, f)
  }
  static tri(f) {
    if (Node.polyMode_ == 1) Fiber.abort("Osc.tri not usable in a Synth yet (Sy-2b)")
    return Node.src_(3, f)
  }
  // ... leave sync*/wavetable/pink/brown as-is but add a poly-mode guard to each (below)
}
```

For each remaining `Osc` factory (`syncSine`/`syncSaw`/`syncSquare`/`syncTri`/`wavetable`) and the `Noise` class (`new`), and `pink`/`brown`, add a first line `if (Node.polyMode_ == 1) Fiber.abort("<name> not usable in a Synth yet (Sy-2b)")` before the existing body.

`.lpf` on **both** `class Node` and `foreign class Port` (and `Node`'s `*`/`+`/`-`):
```wren
  lpf(cutoff) {
    if (Node.polyMode_ == 1) return Node.polysvf_(this, cutoff, 0.2)
    return Node.lpf_(this, cutoff)
  }
  *(o) {
    if (Node.polyMode_ == 1) {
      if (o is Num) Fiber.abort("multiply by a constant inside a Synth isn't supported yet — the amp comes from Env.ar")
      return Node.polymul_(this, o)
    }
    return Node.binop_(0, this, o)
  }
  +(o) {
    if (Node.polyMode_ == 1) Fiber.abort("`+` inside a Synth isn't supported yet (Sy-2b PolyAdd)")
    return Node.binop_(1, this, o)
  }
  -(o) {
    if (Node.polyMode_ == 1) Fiber.abort("`-` inside a Synth isn't supported yet")
    return Node.binop_(2, this, o)
  }
```

`class Env`:
```wren
class Env {
  static ar(attack, release) {
    if (Node.polyMode_ == 1) return Node.polyar_(attack, release)
    return Node.env_(attack, release)
  }
}
```

- [ ] **Step 2: The `Synth` class (prelude.wren)**

Replace the `foreign class Synth { … }` skeleton from Task 2 with the full class (keep the three `foreign` method declarations):

```wren
// A polyphonic instrument. Build a voice once with a `{ |pitch| … }` closure;
// the plain factories emit poly nodes inside it. Route `.out` and play it:
//   var bass = Synth.new { |pitch| Osc.sine(pitch).lpf(1200) * Env.ar(0.01, 0.3) }
//   Out.patch(bass.out)
//   bass.bindMidi()
foreign class Synth {
  foreign noteOn(note, vel)
  foreign noteOff(note)
  foreign out
  bindMidi() {
    Midi.onNoteOn = Fn.new { |ch, note, vel| this.noteOn(note, vel) }
    Midi.onNoteOff = Fn.new { |ch, note, vel| this.noteOff(note) }
  }
  static new(builder) {
    var pitch = Node.polyBegin_()
    var out = builder.call(pitch)
    if (Node.polyGateCount_ == 0) Fiber.abort("a Synth voice needs an Env.ar (the amp gate)")
    if (Node.polyGateCount_ > 1) Fiber.abort("multiple Env.ar in a Synth isn't supported yet")
    return Node.polyEnd_(out)
  }
}
```

- [ ] **Step 3: Tests (audio_bindings.rs)**

```rust
#[test]
fn synth_builds_the_poly_graph() {
    let cmds = run_and_capture_cmds(
        "var bass = Synth.new { |p| Osc.sine(p).lpf(1200) * Env.ar(0.01, 0.3) }",
    );
    for k in [Kind::PolyCtrl, Kind::PolyMtof, Kind::PolyOsc, Kind::PolySvf, Kind::PolyAr, Kind::PolyMul, Kind::VoiceSum] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind, .. } if *kind == k)), "missing {:?}", k);
    }
    // No mono Osc/Svf leaked in.
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Saw | Kind::Sine, .. })), "no mono osc");
}

#[test]
fn poly_mode_is_scoped_after_synth() {
    // After Synth.new returns, poly mode is cleared → Osc.saw builds mono Saw.
    let cmds = run_and_capture_cmds(
        "var b = Synth.new { |p| Osc.sine(p) * Env.ar(0.01, 0.3) }\nOut.patch(Osc.saw(110))",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Saw, .. })), "mono Saw after Synth");
}

#[test]
fn synth_error_cases_abort() {
    // Osc.saw inside a Synth aborts; a no-env voice aborts; two envs abort.
    assert!(!run_script_ok("Synth.new { |p| Osc.saw(p) * Env.ar(0.01,0.3) }"), "Osc.saw aborts");
    assert!(!run_script_ok("Synth.new { |p| Osc.sine(p) }"), "no Env.ar aborts");
    assert!(!run_script_ok("Synth.new { |p| Osc.sine(p) * Env.ar(0.01,0.3) * Env.ar(0.01,0.3) }"), "two Env.ar aborts");
    // Sanity: a valid Synth interprets fine.
    assert!(run_script_ok("Synth.new { |p| Osc.sine(p) * Env.ar(0.01,0.3) }"), "valid Synth ok");
}
```

Add this `run_script_ok` helper to `test_support.rs` (it mirrors
`run_and_capture_cmds` but does NOT assert on the script's interpret result —
it returns whether the script succeeded, so error cases return `false`):

```rust
/// Interpret `src`; return true iff it ran without a compile/runtime error.
pub fn run_script_ok(src: &str) -> bool {
    let _guard = CAP_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // SAFETY: serialized single-threaded test helper; VM freed before return.
    unsafe {
        crate::set_host(&mut *core::ptr::addr_of_mut!(CAP_HOST));
        let vm = wren_sys::boot_with_foreign(crate::METHODS, crate::CLASSES);
        assert!(!vm.is_null(), "VM boot failed");
        let r0 = wren_sys::interpret(vm, c"main".as_ptr(), crate::prelude_ptr());
        assert_eq!(r0, wren_sys::WREN_RESULT_SUCCESS, "prelude failed");
        let mut buf = [0u8; 8192];
        let n = src.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&src.as_bytes()[..n]);
        buf[n] = 0;
        let r = wren_sys::interpret(vm, c"main".as_ptr(), buf.as_ptr() as *const core::ffi::c_char);
        wren_sys::wrenFreeVM(vm);
        crate::reset();
        r == wren_sys::WREN_RESULT_SUCCESS
    }
}
```

- [ ] **Step 4: Run + commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core synth_builds poly_mode_is_scoped synth_error`
Expected: PASS. No warnings.

```bash
git add crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs crates/deluge-wren-core/src/test_support.rs
git commit -m "feat(wren-core): Synth prelude routing + builder + Fiber.abort guards"
```

---

### Task 4: End-to-end render + MIDI

**Files:**
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: everything from Tasks 1–3, plus `test_support::run_and_render` / a host harness and `midi_rx` for the MIDI path.

- [ ] **Step 1: Render — a Synth sounds when played**

```rust
#[test]
fn synth_note_on_renders_sound() {
    // Build a Synth, route it, play a note in-script, then render.
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var bass = Synth.new { |p| Osc.sine(p).lpf(2000) * Env.ar(0.001, 0.05) }\nOut.patch(bass.out)\nbass.noteOn(69, 100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "bounded/finite");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3), "note-on sounds");
}
```

- [ ] **Step 2: MIDI — `bindMidi` + an incoming note drives the allocator**

Prove the MIDI wiring at the `Cmd` layer: after `bindMidi`, an incoming DIN
note-on (`midi_rx(vm, 0x90, note, vel)`) fires the bound closure → `synth.noteOn`
→ the allocator emits `SetParam` + `GateVoice`. (The audible path is already
covered by Step 1's direct-`noteOn` render.) Add this helper to `test_support.rs`
— it mirrors `run_and_capture_cmds` but injects one `midi_rx` after the setup and
captures only that event's `Cmd`s:

```rust
/// Interpret `setup`, feed one DIN-MIDI message, capture the Cmds it emits.
pub fn run_midi_capture_cmds(setup: &str, status: u8, d1: u8, d2: u8) -> Vec<crate::Cmd> {
    let _guard = CAP_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // SAFETY: serialized single-threaded test helper; VM freed before return.
    unsafe {
        let h = &mut *core::ptr::addr_of_mut!(CAP_HOST);
        h.cmds.clear();
        crate::set_host(&mut *core::ptr::addr_of_mut!(CAP_HOST));
        let vm = wren_sys::boot_with_foreign(crate::METHODS, crate::CLASSES);
        assert!(!vm.is_null(), "VM boot failed");
        assert_eq!(wren_sys::interpret(vm, c"main".as_ptr(), crate::prelude_ptr()), wren_sys::WREN_RESULT_SUCCESS, "prelude failed");
        let mut buf = [0u8; 8192];
        let n = setup.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&setup.as_bytes()[..n]);
        buf[n] = 0;
        assert_eq!(wren_sys::interpret(vm, c"main".as_ptr(), buf.as_ptr() as *const core::ffi::c_char), wren_sys::WREN_RESULT_SUCCESS, "setup failed");
        (*core::ptr::addr_of_mut!(CAP_HOST)).cmds.clear(); // ignore build-time cmds; capture only the MIDI event's
        // `midi_rx` fires the bound `Midi.onNoteOn` closure with (ch, note, vel).
        // Construct the `Vm` wrapper the same way the extern binding wrappers do.
        crate::bindings::midi_rx_impl(&crate::bindings::Vm(vm), status, d1, d2);
        let out = (*core::ptr::addr_of_mut!(CAP_HOST)).cmds.clone();
        wren_sys::wrenFreeVM(vm);
        crate::reset();
        out
    }
}
```

```rust
#[test]
fn synth_plays_from_midi() {
    let cmds = run_midi_capture_cmds(
        "var bass = Synth.new { |p| Osc.sine(p).lpf(2000) * Env.ar(0.001, 0.05) }\nOut.patch(bass.out)\nbass.bindMidi()",
        0x90, 69, 100, // note-on A4 vel 100
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::GateVoice { on: true, .. })), "MIDI note-on gates a voice");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { .. })), "MIDI note-on sets pitch");
}
```

(`midi_rx_impl` and the `Vm` wrapper are `pub(crate)` in `crate::bindings`; if the
exact path differs, mirror how the `#[cfg(feature = "wren-sys-backend")]` extern
wrappers construct `Vm(raw)` and call `midi_rx_impl`.)

- [ ] **Step 3: Run both feature configs + commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core synth_note_on_renders synth_plays_from_midi`
Expected: PASS (scalar).
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd synth_note_on_renders synth_plays_from_midi`
Expected: PASS (f32x8 PolyOsc/PolySvf). No warnings.

```bash
git add crates/deluge-wren-core/tests/audio_bindings.rs crates/deluge-wren-core/src/test_support.rs
git commit -m "test(wren-core): end-to-end Synth render + MIDI-driven voice"
```

---

## Notes for the implementer

- **Wren gotchas:** no `?:` ternary (use `if/return`); `0` is truthy (compare `polyMode_ == 1`); `o is Num` type-checks a scalar operand; `Fiber.abort(msg)` raises a catchable runtime error.
- Every new foreign method needs BOTH the generic `_impl<S>` and the `#[cfg(feature = "wren-sys-backend")]` `extern "C"` wrapper, registered in BOTH `METHODS` (wren-sys) and `register_audio` (main). Mirror `node_lfo` / `node_set_phase` exactly.
- `SynthObj` mirrors `NodeObj`'s foreign registration; it holds a `VoiceAllocator` (POD: `NodeId`s, `[Option<u8>;8]`, `[u32;8]`, `u32` — no `Drop`). If wren-sys needs the `Synth` foreign class reachable before `new_foreign_in`, confirm `foreign class Synth` is in the prelude (it is, from Task 2).
- If `host().audio_cmd` signature differs from `|c| host().audio_cmd(c)` (e.g. needs `&mut`), adapt the closure — the requirement is that `note_on`/`note_off`'s `&mut impl FnMut(Cmd)` reaches the same transport the `audio::*` emitters use.
- If an anchor moved, grep the named symbol — the relationship (register alongside `lfo_`, route inside `class Osc`) matters, not line numbers.
