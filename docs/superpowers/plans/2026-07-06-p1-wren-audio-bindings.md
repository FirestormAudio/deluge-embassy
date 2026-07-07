# P1 — Low-level Wren audio bindings Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Retarget `deluge-wren-core`'s Wren audio surface onto the `deluge-audio-graph` engine — parity with the prototype `Osc`/`Env`/`Out`, plus output ports, buses, and explicit `.free()` — validated by real-VM host tests, keeping the workspace compiling.

**Architecture:** Refactor `deluge-wren-core` in place: delete the old `engine.rs`, rewrite `audio.rs` into client-side id allocators + `deluge_audio_graph::Cmd` emitters, split the audio bindings into `bindings_audio.rs`, and retarget `Host::audio_cmd` to the new `Cmd`. The one in-workspace consumer, `wren-firmware`, is minimally re-wired to the new engine. Tests boot the real wren-sys VM with a capturing/applying `Host`.

**Tech Stack:** Rust, `no_std` (`deluge-wren-core`), the `wren-sys` C VM (behind `wren-sys-backend`), `deluge-audio-graph` (P0). Host tests run on `x86_64-unknown-linux-gnu`.

**Reference spec:** [P1 design](../specs/2026-07-06-p1-wren-audio-bindings-design.md). Depends on P0 (merged to `main`).

## Global Constraints

- `deluge-wren-core` stays `#![no_std]`; no heap/locks in binding or emitter code (VM-thread-single-threaded process-globals, matching existing `host()`/`state()` patterns). Tests may use `std`.
- Host tests require `--target x86_64-unknown-linux-gnu` (the workspace `.cargo/config.toml` defaults to the embedded `armv7a-none-eabihf` target) and the `test-support` feature (which enables `wren-sys-backend`). Command: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`.
- Every audio binding must be registered in BOTH `register_foreign` (the `SlotApi`-generic table) AND the `#[cfg(feature = "wren-sys-backend")]` `METHODS`/`CLASSES` static tables — they enumerate the same surface for two backends.
- Parity: existing scripts using `Osc`/`Env`/`Noise`/`Out`/`*`/`+`/`-`/`.lpf`/`.freq=`/`.cutoff=`/`.gate`/`.trigger` keep working.
- `WREN_MAX_NODES = 64`, `WREN_MAX_BUSES = 8`, master bus id `0`. A host `Engine<BLOCK, NODES, OUTS, BUSES>` must satisfy `NODES >= 64`, `BUSES >= 8`.
- Commit after each task with the message in its final step.

---

## File structure

**`crates/deluge-wren-core/`:**
- `Cargo.toml` — add `deluge-audio-graph` dependency.
- `src/engine.rs` — **deleted** (old prototype engine).
- `src/audio.rs` — **rewritten**: `Alloc` (NodeId free-list + BusId allocator) + `Cmd` emitters.
- `src/bindings_audio.rs` — **new**: `NodeObj`/`PortObj`/`BusObj` foreign classes + their binding bodies + `arg_input`.
- `src/bindings.rs` — remove the old audio (`Node`) section; keep everything else; register the new audio surface.
- `src/host.rs` — `Host::audio_cmd` carries `deluge_audio_graph::Cmd`.
- `src/lib.rs` — re-export `Cmd`/`Engine`/`Input`/`Kind` from `deluge-audio-graph`; add `mod bindings_audio;`.
- `src/test_support.rs` — add `CmdCaptureHost` + `EngineHost` + run helpers.
- `wren/prelude.wren` — rewrite the "Audio" section.
- `tests/audio_bindings.rs` — **new**: Cmd-sequence + golden-audio tests.

**`wren-firmware/src/audio.rs`** — minimal re-wire to the new engine.

**`crates/deluge-audio-graph/`** — Task 1 only: add derives to public types.

---

## Task 1: Debug/PartialEq derives on `deluge-audio-graph` public types

**Files:**
- Modify: `crates/deluge-audio-graph/src/ids.rs` (`NodeId`, `BusId`, `Input`)
- Modify: `crates/deluge-audio-graph/src/cmd.rs` (`Cmd`)
- Modify: `crates/deluge-audio-graph/src/node.rs` (`Kind`)

**Interfaces:**
- Produces: `Cmd`, `Input`, `Kind` gain `#[derive(Debug, PartialEq)]`; `NodeId`, `BusId` gain `Debug, PartialEq, Eq` (already have some — add what's missing). Enables `assert_eq!` on captured command vectors.

- [ ] **Step 1: Write the failing test**

Add to `crates/deluge-audio-graph/src/cmd.rs` tests module:

```rust
#[test]
fn cmds_are_comparable_and_debuggable() {
    use crate::{Input, NodeId};
    use crate::node::Kind;
    let a = Cmd::NewNode { node: NodeId(1), kind: Kind::Saw, args: [Input::Const(110.0), Input::Const(0.0), Input::Const(0.0)] };
    let b = Cmd::NewNode { node: NodeId(1), kind: Kind::Saw, args: [Input::Const(110.0), Input::Const(0.0), Input::Const(0.0)] };
    assert_eq!(a, b);
    assert_ne!(a, Cmd::Reset);
    let _ = format!("{a:?}"); // Debug present
}
```

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: FAIL — `Cmd`/`Input`/`Kind` don't implement `PartialEq`/`Debug`.

- [ ] **Step 3: Add the derives**

- `ids.rs`: `NodeId`, `BusId` → ensure `#[derive(Clone, Copy, PartialEq, Eq, Debug)]` (already present per P0). `Input` → change to `#[derive(Clone, Copy, Debug, PartialEq)]` (no `Eq` — it holds `f32`).
- `cmd.rs`: `Cmd` → `#[derive(Clone, Copy, Debug, PartialEq)]`.
- `node.rs`: `Kind` → `#[derive(Clone, Copy, Debug, PartialEq, Eq)]`.

- [ ] **Step 4: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: PASS (all existing + the new test). Also run `--features simd` — PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src
git commit -m "feat(audio-graph): Debug/PartialEq on public Cmd/Input/Kind for test assertions"
```

---

## Task 2: Core swap — retarget `deluge-wren-core` audio to the new engine (parity)

This is the atomic type-swap. Its deliverable: **`deluge-wren-core` compiles on the new engine with the prototype Wren surface preserved**, verified by a capturing host + Cmd-sequence tests. `wren-firmware` is intentionally left broken until Task 3 (this task's verification builds only `-p deluge-wren-core`).

**Files:**
- Modify: `crates/deluge-wren-core/Cargo.toml`
- Delete: `crates/deluge-wren-core/src/engine.rs`
- Rewrite: `crates/deluge-wren-core/src/audio.rs`
- Create: `crates/deluge-wren-core/src/bindings_audio.rs`
- Modify: `crates/deluge-wren-core/src/bindings.rs` (remove audio section; register new surface), `src/host.rs`, `src/lib.rs`, `src/test_support.rs`, `wren/prelude.wren`
- Create: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `deluge_audio_graph::{Cmd, Input, Kind, NodeId, BusId}` (Task 1 derives).
- Produces (in `audio.rs`, `pub(crate)` unless noted):
  - `pub const WREN_MAX_NODES: usize = 64;` `pub const WREN_MAX_BUSES: usize = 8;` `pub const MASTER_BUS: u16 = 0;` `const NULL_ID: u16 = u16::MAX;`
  - `alloc_node_id() -> u16`, `free_node_id(id: u16)`, `alloc_bus_id() -> u16`
  - `new_node(id: u16, kind: Kind, args: [Input; 3])`, `set_input(id: u16, port: u8, src: Input)`, `gate(id: u16, on: bool)`, `trigger(id: u16)`, `bus_write(src: Input, bus: u16)`, `set_root(bus: u16)`, `free(id: u16)`, `reset()`
- Produces (in `bindings_audio.rs`): `NodeObj{id:u16}`, `arg_input<S: SlotApi>(vm,slot)->Input`, the parity binding bodies (`node_src_impl` … `node_trigger_impl`), and `register_audio` (called from `register_foreign`).
- Produces (in `host.rs`): `Host::audio_cmd(&mut self, cmd: deluge_audio_graph::Cmd)`.

- [ ] **Step 1: Add the dependency, delete the old engine, retarget re-exports**

`crates/deluge-wren-core/Cargo.toml` — add under `[dependencies]`:

```toml
deluge-audio-graph = { path = "../deluge-audio-graph" }
```

Delete `crates/deluge-wren-core/src/engine.rs`. In `src/lib.rs`: remove `mod engine;`, add `mod bindings_audio;`, and replace the old engine re-export block with:

```rust
pub use deluge_audio_graph::{BusId, Cmd, Input, Kind, NodeId};
```

(Remove `K_*`/`MAX_NODES`/`Engine` re-exports from the old engine; a host that needs `Engine` imports it from `deluge_audio_graph` directly.)

- [ ] **Step 2: Change the `Host` seam type**

`src/host.rs`: change the import `use crate::engine::Cmd;` to `use deluge_audio_graph::Cmd;`, and the trait method doc/type stays `fn audio_cmd(&mut self, cmd: Cmd);` (now the new `Cmd`). No other change.

- [ ] **Step 3: Rewrite `audio.rs` (allocators + emitters)**

Replace the entire contents of `crates/deluge-wren-core/src/audio.rs` with:

```rust
//! Client-side id allocators + control-rate command emitters for the Wren audio
//! bindings. The binding owns the `NodeId` free-list and `BusId` allocation (no
//! round-trip); each emitter builds a `deluge_audio_graph::Cmd` and ships it
//! through the registered [`Host`](crate::Host). Single-threaded VM context.

use deluge_audio_graph::{BusId, Cmd, Input, Kind, NodeId};

use crate::host::host;

/// Binding-side node-id capacity. A host `Engine` must have `NODES >= this`.
pub const WREN_MAX_NODES: usize = 64;
/// Binding-side bus capacity. A host `Engine` must have `BUSES >= this`.
pub const WREN_MAX_BUSES: usize = 8;
/// The implicit master bus (render root for `Out.patch(node)`).
pub const MASTER_BUS: u16 = 0;
/// Returned when the pool is exhausted; factories no-op on it (inert node).
pub const NULL_ID: u16 = u16::MAX;

struct Alloc {
    next: u16,                     // bump pointer for node ids
    free: [u16; WREN_MAX_NODES],   // stack of freed node ids
    free_len: usize,
    next_bus: u16,                 // bump pointer for bus ids (1.. ; 0 = master)
}

impl Alloc {
    const fn new() -> Self {
        Alloc { next: 0, free: [0; WREN_MAX_NODES], free_len: 0, next_bus: 1 }
    }
    fn alloc_node(&mut self) -> u16 {
        if self.free_len > 0 {
            self.free_len -= 1;
            return self.free[self.free_len];
        }
        if (self.next as usize) < WREN_MAX_NODES {
            let id = self.next;
            self.next += 1;
            id
        } else {
            NULL_ID
        }
    }
    fn free_node(&mut self, id: u16) {
        if (id as usize) < WREN_MAX_NODES && self.free_len < WREN_MAX_NODES {
            self.free[self.free_len] = id;
            self.free_len += 1;
        }
    }
    fn alloc_bus(&mut self) -> u16 {
        if (self.next_bus as usize) < WREN_MAX_BUSES {
            let id = self.next_bus;
            self.next_bus += 1;
            id
        } else {
            NULL_ID
        }
    }
    fn reset(&mut self) {
        self.next = 0;
        self.free_len = 0;
        self.next_bus = 1;
    }
}

// SAFETY: single-threaded VM context, like the rest of the binding state.
static mut ALLOC: Alloc = Alloc::new();

#[inline]
fn alloc() -> &'static mut Alloc {
    // SAFETY: sole accessor is the VM thread.
    unsafe { &mut *core::ptr::addr_of_mut!(ALLOC) }
}

pub fn alloc_node_id() -> u16 {
    alloc().alloc_node()
}
pub fn free_node_id(id: u16) {
    alloc().free_node(id);
}
pub fn alloc_bus_id() -> u16 {
    alloc().alloc_bus()
}

pub fn new_node(id: u16, kind: Kind, args: [Input; 3]) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode { node: NodeId(id), kind, args });
}
pub fn set_input(id: u16, port: u8, src: Input) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::SetInput { node: NodeId(id), port, src });
}
pub fn gate(id: u16, on: bool) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::Gate { node: NodeId(id), on });
}
pub fn trigger(id: u16) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::Trigger { node: NodeId(id) });
}
pub fn bus_write(src: Input, bus: u16) {
    if bus == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::BusWrite { src, bus: BusId(bus) });
}
pub fn set_root(bus: u16) {
    if bus == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::SetRoot { bus: BusId(bus) });
}
pub fn free(id: u16) {
    if id == NULL_ID {
        return;
    }
    free_node_id(id);
    host().audio_cmd(Cmd::Free { node: NodeId(id) });
}
pub fn reset() {
    alloc().reset();
    host().audio_cmd(Cmd::Reset);
}
```

- [ ] **Step 4: Create `bindings_audio.rs` (parity classes)**

`crates/deluge-wren-core/src/bindings_audio.rs`:

```rust
//! The Wren audio foreign classes (`Node`/`Out`), retargeted onto
//! `deluge-audio-graph`. Factory statics allocate an id and emit `NewNode`;
//! instance methods mutate via the `crate::audio` emitters. `Port` and `Bus`
//! are added in later P1 tasks.

use deluge_audio_graph::{Input, Kind, NodeId};

use crate::audio;
use crate::slotapi::{SlotApi, WrenForeign, WrenType};

// All audio foreign objects lead with a `tag: u8` (offset 0 under `repr(C)`) so
// `arg_input` can discriminate a Node/Port/Bus argument by reading that byte —
// no VM class query, no `SlotApi` change. `Port`/`Bus` land in later tasks.
pub(crate) const TAG_NODE: u8 = 0;
pub(crate) const TAG_PORT: u8 = 1;
pub(crate) const TAG_BUS: u8 = 2;

#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct NodeObj {
    pub tag: u8,
    pub id: u16,
}
impl WrenForeign for NodeObj {
    fn module_name() -> &'static str {
        "main"
    }
    fn class_name() -> &'static str {
        "Node"
    }
}

/// Map a prelude waveform/op code to a `Kind`. Codes match the prelude
/// (`Osc.sine=0 saw=1 square=2 tri=3`; binop `mul=0 add=1 sub=2`).
fn src_kind(code: u8) -> Kind {
    match code {
        0 => Kind::Sine,
        1 => Kind::Saw,
        2 => Kind::Square,
        _ => Kind::Tri,
    }
}
fn binop_kind(code: u8) -> Kind {
    match code {
        0 => Kind::Mul,
        1 => Kind::Add,
        _ => Kind::Sub,
    }
}

/// Resolve a number / Node / Port / Bus argument at `slot` into an engine
/// `Input`, discriminating foreign objects by their leading `tag` byte. Task 2
/// only has `Node`; Tasks 4/5 add the `Port`/`Bus` arms.
pub(crate) fn arg_input<S: SlotApi>(vm: &S, slot: i32) -> Input {
    match vm.slot_type(slot) {
        WrenType::Num => Input::Const(vm.get_f(slot) as f32),
        WrenType::Foreign => {
            // SAFETY: every foreign arg to an audio method is one of our tagged
            // objects; the tag is at offset 0 under `repr(C)`.
            let tag = unsafe { *vm.foreign_mut::<u8>(slot) };
            match tag {
                // Port / Bus arms are inserted here in Tasks 4 / 5.
                _ => {
                    let id = unsafe { vm.foreign_mut::<NodeObj>(slot) }.id;
                    Input::Node { node: NodeId(id), port: 0 }
                }
            }
        }
        _ => Input::Const(0.0),
    }
}

fn self_id<S: SlotApi>(vm: &S) -> u16 {
    unsafe { vm.foreign_mut::<NodeObj>(0) }.id
}
unsafe fn return_node<S: SlotApi>(vm: &S, id: u16) {
    unsafe { vm.new_foreign_in(0, NodeObj { tag: TAG_NODE, id }) };
}

// ── Factory statics (return a Node) ──────────────────────────────────────────

pub(crate) fn node_src_impl<S: SlotApi>(vm: &S) {
    let kind = src_kind(vm.get_f(1) as u8);
    let freq = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, kind, [freq, Input::Const(0.0), Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
pub(crate) fn node_env_impl<S: SlotApi>(vm: &S) {
    let a = arg_input(vm, 1);
    let b = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Env, [a, b, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
pub(crate) fn node_noise_impl<S: SlotApi>(vm: &S) {
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Noise, [Input::Const(0.0); 3]);
    unsafe { return_node(vm, id) };
}
pub(crate) fn node_binop_impl<S: SlotApi>(vm: &S) {
    let kind = binop_kind(vm.get_f(1) as u8);
    let a = arg_input(vm, 2);
    let b = arg_input(vm, 3);
    let id = audio::alloc_node_id();
    audio::new_node(id, kind, [a, b, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
pub(crate) fn node_lpf_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let cutoff = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Lpf, [input, cutoff, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}

// ── Out (patch / reset) — master-bus sugar ───────────────────────────────────

pub(crate) fn node_patch_impl<S: SlotApi>(vm: &S) {
    // `Out.patch(node)` → write node into the master bus, set it as root.
    let id = unsafe { vm.foreign_mut::<NodeObj>(1) }.id;
    audio::bus_write(Input::Node { node: NodeId(id), port: 0 }, audio::MASTER_BUS);
    audio::set_root(audio::MASTER_BUS);
}
pub(crate) fn node_reset_impl<S: SlotApi>(_vm: &S) {
    audio::reset();
}

// ── Instance methods (self = slot 0) ─────────────────────────────────────────

pub(crate) fn node_set_freq_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 0, v);
}
pub(crate) fn node_set_cutoff_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 1, v);
}
pub(crate) fn node_gate_impl<S: SlotApi>(vm: &S) {
    let on = vm.get_bool(1);
    audio::gate(self_id(vm), on);
}
pub(crate) fn node_trigger_impl<S: SlotApi>(vm: &S) {
    audio::trigger(self_id(vm));
}

/// Register the audio surface into a caller-provided method/class registrar
/// (called from `bindings::register_foreign`).
pub(crate) fn register_audio<S: SlotApi>(
    method: &mut impl FnMut(&'static str, &'static str, bool, &'static str, fn(&S)),
) {
    method("main", "Node", true, "src_(_,_)", node_src_impl::<S>);
    method("main", "Node", true, "env_(_,_)", node_env_impl::<S>);
    method("main", "Node", true, "noise_()", node_noise_impl::<S>);
    method("main", "Node", true, "binop_(_,_,_)", node_binop_impl::<S>);
    method("main", "Node", true, "lpf_(_,_)", node_lpf_impl::<S>);
    method("main", "Node", true, "patch_(_)", node_patch_impl::<S>);
    method("main", "Node", true, "reset_()", node_reset_impl::<S>);
    method("main", "Node", false, "freq=(_)", node_set_freq_impl::<S>);
    method("main", "Node", false, "cutoff=(_)", node_set_cutoff_impl::<S>);
    method("main", "Node", false, "gate(_)", node_gate_impl::<S>);
    method("main", "Node", false, "trigger()", node_trigger_impl::<S>);
}
```

- [ ] **Step 5: Remove the old audio section from `bindings.rs`; wire the new one**

In `crates/deluge-wren-core/src/bindings.rs`:
- Delete the entire "Audio: DSP node graph (`Node` foreign class)" section (the `NodeObj`, `arg_input`, `self_id`, `return_node`, all `node_*_impl` and their `extern "C"` wrappers) — it now lives in `bindings_audio.rs`.
- In `register_foreign`, replace the block of `method("main", "Node", …)` lines with a single call: `crate::bindings_audio::register_audio(&mut method);`
- In the `#[cfg(feature = "wren-sys-backend")]` `METHODS` static table, the `Node` entries currently bind the local `extern "C"` wrappers (`node_src`, …). Those wrappers moved. Add matching `#[cfg(feature = "wren-sys-backend")] unsafe extern "C"` wrappers in `bindings_audio.rs` for each of the 11 methods (each: `let vm = Vm(raw); node_*_impl(&vm);`, mirroring the deleted ones — use the existing `crate::bindings::Vm` type, made `pub(crate)` if needed), and reference them from `METHODS` via `static_method("Node", "src_(_,_)", bindings_audio::node_src)` etc. Keep the `Node` methods exactly as before (same signatures).

(The `Node` class itself is a *foreign* class only in the sense of holding `NodeObj`; it has no allocator entry in `CLASSES` — the prototype had none either, since `Node` objects are created via `new_foreign_in`, not a Wren `construct`. Leave `CLASSES` unchanged.)

- [ ] **Step 6: Rewrite the prelude audio section**

In `crates/deluge-wren-core/wren/prelude.wren`, replace the "Audio: native DSP graph" section's `Node`/`Osc`/`Env`/`Noise`/`Out` classes with the parity-equivalent (unchanged surface — the foreign signatures are identical, so this is mostly the same text; keep `src_`/`env_`/`noise_`/`binop_`/`lpf_`/`patch_`/`reset_` and the `*`/`+`/`-`/`lpf` sugar). Verify the waveform codes match `src_kind` (sine=0 saw=1 square=2 tri=3) and binop codes match `binop_kind` (mul=0 add=1 sub=2) — they do in the current prelude.

- [ ] **Step 7: Add a capturing host + a run-and-capture helper to `test_support.rs`**

In `crates/deluge-wren-core/src/test_support.rs`, add (using `std` — this module is only built for host tests under `test-support`):

```rust
extern crate std;
use std::vec::Vec;

/// A host that records every audio command for assertions.
pub struct CmdCaptureHost {
    pub cmds: Vec<crate::Cmd>,
}
impl CmdCaptureHost {
    pub const fn new() -> Self { CmdCaptureHost { cmds: Vec::new() } }
}
impl Host for CmdCaptureHost {
    fn now_ms(&mut self) -> u64 { 0 }
    fn cv_set(&mut self, _ch: u8, _v: f32) {}
    fn gate_set(&mut self, _ch: u8, _on: bool) {}
    fn midi_tx(&mut self, _m: &[u8]) {}
    fn led(&mut self, _id: u8, _on: bool) {}
    fn oled_clear(&mut self) {}
    fn oled_text(&mut self, _x: usize, _y: usize, _t: &[u8]) {}
    fn oled_pixel(&mut self, _x: usize, _y: usize, _on: bool) {}
    fn oled_show(&mut self) {}
    fn audio_cmd(&mut self, cmd: crate::Cmd) { self.cmds.push(cmd); }
}

static mut CAP_HOST: CmdCaptureHost = CmdCaptureHost::new();

/// Boot a VM, run `src`, and return the audio commands it emitted.
pub fn run_and_capture_cmds(src: &str) -> Vec<crate::Cmd> {
    // SAFETY: single-threaded test helper; VM freed before return.
    unsafe {
        let h = &mut *core::ptr::addr_of_mut!(CAP_HOST);
        h.cmds.clear();
        crate::set_host(&mut *core::ptr::addr_of_mut!(CAP_HOST));
        let vm = wren_sys::boot_with_foreign(crate::METHODS, crate::CLASSES);
        assert!(!vm.is_null(), "VM boot failed");
        let r = wren_sys::interpret(vm, c"main".as_ptr(), crate::prelude_ptr());
        assert_eq!(r, wren_sys::WREN_RESULT_SUCCESS, "prelude failed");
        let mut buf = [0u8; 8192];
        let n = src.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&src.as_bytes()[..n]);
        buf[n] = 0;
        let r = wren_sys::interpret(vm, c"main".as_ptr(), buf.as_ptr() as *const core::ffi::c_char);
        assert_eq!(r, wren_sys::WREN_RESULT_SUCCESS, "script failed (line {})", LAST_ERR_LINE.load(Ordering::Relaxed));
        let out = (*core::ptr::addr_of_mut!(CAP_HOST)).cmds.clone();
        wren_sys::wrenFreeVM(vm);
        crate::reset();
        out
    }
}
```

Ensure `test_support` is exported for integration tests (`lib.rs` already gates `pub mod test_support` on `test-support`). Keep the existing `TestHost::audio_cmd` no-op as-is.

- [ ] **Step 8: Write the failing parity Cmd-sequence tests**

`crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#![cfg(feature = "test-support")]
use deluge_wren_core::test_support::run_and_capture_cmds;
use deluge_wren_core::{BusId, Cmd, Input, Kind, NodeId};

fn saw(freq: f32) -> Cmd {
    Cmd::NewNode { node: NodeId(0), kind: Kind::Saw, args: [Input::Const(freq), Input::Const(0.0), Input::Const(0.0)] }
}

#[test]
fn osc_saw_emits_newnode() {
    let cmds = run_and_capture_cmds("Osc.saw(110)");
    assert_eq!(cmds, vec![saw(110.0)]);
}

#[test]
fn patch_writes_master_bus_and_sets_root() {
    let cmds = run_and_capture_cmds("Out.patch(Osc.saw(110))");
    assert_eq!(cmds, vec![
        saw(110.0),
        Cmd::BusWrite { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(0) },
        Cmd::SetRoot { bus: BusId(0) },
    ]);
}

#[test]
fn lpf_and_binop_resolve_node_inputs() {
    // Osc.saw(110).lpf(800) → node0 saw, node1 lpf(node0, 800)
    let cmds = run_and_capture_cmds("var x = Osc.saw(110).lpf(800)");
    assert_eq!(cmds, vec![
        saw(110.0),
        Cmd::NewNode { node: NodeId(1), kind: Kind::Lpf, args: [Input::Node { node: NodeId(0), port: 0 }, Input::Const(800.0), Input::Const(0.0)] },
    ]);
}

#[test]
fn reset_emits_reset() {
    let cmds = run_and_capture_cmds("Out.reset()");
    assert_eq!(cmds, vec![Cmd::Reset]);
}
```

- [ ] **Step 9: Run — verify the crate compiles and the tests pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: PASS — the new audio_bindings tests plus the existing `golden_sim`/`slotapi`/`no_backend` tests (CV/gate behavior is unchanged). `wren-firmware` is NOT built by `-p deluge-wren-core`; it is fixed in Task 3.

- [ ] **Step 10: Commit**

```bash
git add crates/deluge-wren-core crates/deluge-audio-graph
git commit -m "feat(wren-core): retarget audio bindings onto deluge-audio-graph (parity)"
```

---

## Task 3: Re-wire `wren-firmware` to the new engine (workspace green)

**Files:**
- Modify: `wren-firmware/Cargo.toml` (add `deluge-audio-graph` dependency), `wren-firmware/src/audio.rs`

**Interfaces:**
- Consumes: `deluge_audio_graph::{Engine, StereoFrame}`, `deluge_wren_core::Cmd` (the re-exported `deluge_audio_graph::Cmd`).

**Context:** the old task held `static mut ENGINE: Engine = Engine::new();` (const) and rendered mono per-frame via `render_frame()`. The new `Engine::new(sample_rate)` is not `const`, the engine is large (~20 KB — keep it in a static, not on the task stack), and it renders a stereo block via `render(&mut [StereoFrame])` which fills `min(out.len(), BLOCK)` frames — so a longer SDK block must be rendered in `BLOCK`-sized chunks.

- [ ] **Step 1: Confirm the break, then define the engine type**

Run: `cargo build --target x86_64-unknown-linux-gnu -p wren-firmware` (host sim build)
Expected: FAIL — `deluge_wren_core::Engine`/`Cmd`/`render_frame` no longer exist.

Add `deluge-audio-graph` to `wren-firmware/Cargo.toml` under `[dependencies]`:

```toml
deluge-audio-graph = { path = "../crates/deluge-audio-graph" }
```

Add near the top of `wren-firmware/src/audio.rs`:

```rust
use core::mem::MaybeUninit;
use deluge_audio_graph::Engine;

// Sizes satisfy the binding contract: NODES >= WREN_MAX_NODES(64), BUSES >= 8.
type Eng = Engine<32, 64, 128, 8>; // BLOCK, NODES, OUTS, BUSES
const SAMPLE_RATE: f32 = 44_100.0;
```

- [ ] **Step 2: Swap the command ring and engine storage**

Change `CmdRing`'s element type and the filler from the old `Cmd::Nop` to the new one. The new `Cmd` has a `Nop` variant too (`deluge_audio_graph::Cmd::Nop`), so `CmdRing::new()` still uses `Cmd::Nop`. Update the import: `use deluge_wren_core::Cmd;` stays (it now re-exports the new `Cmd`).

Replace the engine static:

```rust
// SAFETY: ENGINE is initialized once at the top of audio_task and thereafter
// touched only by that task (single accessor).
static mut ENGINE: MaybeUninit<Eng> = MaybeUninit::uninit();
```

- [ ] **Step 3: Rewrite the render body**

In `audio_task`, before the `process` loop, init the engine once:

```rust
// SAFETY: audio_task is the sole accessor; init happens once before use.
let eng: &mut Eng = unsafe {
    let p = &mut *core::ptr::addr_of_mut!(ENGINE);
    p.write(Eng::new(SAMPLE_RATE));
    p.assume_init_mut()
};
```

Replace the per-frame render with drain-then-chunk-render:

```rust
audio.process(|block: &mut [StereoFrame]| {
    // Apply all pending control-rate commands.
    while let Some(c) = CMD_RING.lock(|r| r.borrow_mut().pop()) {
        eng.apply(c);
    }
    // Render the SDK block in engine-BLOCK-sized chunks.
    for chunk in block.chunks_mut(32) {
        eng.render(chunk);
    }
})
.await
```

(`StereoFrame` is now `deluge_audio_graph::StereoFrame`; update the import from `deluge::StereoFrame` if the SDK's differs — if the SDK expects its own `StereoFrame`, map field-by-field in the chunk loop instead: render into a local `[deluge_audio_graph::StereoFrame; 32]` and copy `.l`/`.r` into the SDK block. Choose whichever the SDK `Audio::process` signature requires; keep it minimal.)

- [ ] **Step 4: Verify the workspace compiles again**

Run: `cargo build --target x86_64-unknown-linux-gnu -p wren-firmware`
Expected: PASS (host sim build).
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: still PASS.

- [ ] **Step 5: Commit**

```bash
git add wren-firmware/src/audio.rs
git commit -m "feat(wren-firmware): re-wire audio task to deluge-audio-graph engine"
```

---

## Task 4: Output ports — `Node.out(p)` + `Port` + `Split`

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs`, `src/bindings.rs` (register), `wren/prelude.wren`, `tests/audio_bindings.rs`

**Interfaces:**
- Produces: `PortObj { node: u16, port: u8 }` foreign class; `node_out_impl` (`Node.out(_)` → returns a `Port`); `Split` factory (`node_split_impl` → `Kind::Split2`); `arg_input` extended to resolve `PortObj → Input::Node { node, port }`.

- [ ] **Step 1: Write the failing tests**

Add to `tests/audio_bindings.rs`:

```rust
#[test]
fn out_port_resolves_to_that_port() {
    // Split.new(Osc.saw(110)); consumer reads .out(1)
    let cmds = run_and_capture_cmds(
        "var s = Split.new(Osc.saw(110))\n\
         var c = s.out(1) * 2",
    );
    // node0 saw, node1 split2(node0), node2 mul(node1.port1, 2)
    assert!(cmds.iter().any(|c| *c == Cmd::NewNode {
        node: NodeId(2), kind: Kind::Mul,
        args: [Input::Node { node: NodeId(1), port: 1 }, Input::Const(2.0), Input::Const(0.0)],
    }));
}
```

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: FAIL — `Split`/`out` unknown.

- [ ] **Step 3: Implement `Port` + `Split` + `out`**

In `bindings_audio.rs`, add the tagged `Port` object, the `.out(p)` binding, and
the `Split` factory:

```rust
#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct PortObj {
    pub tag: u8,
    pub node: u16,
    pub port: u8,
}
impl WrenForeign for PortObj {
    fn module_name() -> &'static str { "main" }
    fn class_name() -> &'static str { "Port" }
}

pub(crate) fn node_out_impl<S: SlotApi>(vm: &S) {
    let node = self_id(vm);
    let port = vm.get_f(1) as u8;
    unsafe { vm.new_foreign_in(0, PortObj { tag: TAG_PORT, node, port }) };
}

pub(crate) fn node_split_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Split2, [input, Input::Const(0.0), Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
```

Insert a `Port` arm into `arg_input`'s tag match (replace the `// Port / Bus arms
…` comment line with the `TAG_PORT` arm):

```rust
                TAG_PORT => {
                    let p = unsafe { vm.foreign_mut::<PortObj>(slot) };
                    Input::Node { node: NodeId(p.node), port: p.port }
                }
```

- [ ] **Step 4: Register `Split` + `out`**

In `register_audio` add:
```rust
method("main", "Node", true, "split_(_)", node_split_impl::<S>);
method("main", "Node", false, "out(_)", node_out_impl::<S>);
```
Add matching `#[cfg(feature = "wren-sys-backend")]` `extern "C"` wrappers +
`METHODS` entries (note: `Port` is created via `new_foreign_in`, no allocator —
like `Node`, no `CLASSES` entry). In `prelude.wren`, add `foreign out(p)` and
`foreign static split_(input)` to the `Node` foreign class, a
`class Split { static new(input) { Node.split_(input) } }`, and a
`foreign class Port {}` declaration so the VM knows the class. No `SlotApi` or
backend changes — discrimination is via the `tag` byte only.

- [ ] **Step 5: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core
git commit -m "feat(wren-core): output ports — Node.out(p), Port, Split"
```

---

## Task 5: Buses — `Bus.new`, `.write`, bus-as-input, `Out.patch(bus)`

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs`, `src/bindings.rs`, `wren/prelude.wren`, `tests/audio_bindings.rs`

**Interfaces:**
- Produces: `BusObj { id: u16 }` foreign class; `bus_new_impl` (`Bus.new_()` → alloc bus id, return `Bus`), `bus_write_impl` (`.write(src)` → `BusWrite`); `node_patch_impl` extended to accept a `Bus` (set it as root directly); `arg_input` `Bus` arm (added in Task 4's snippet).

- [ ] **Step 1: Write the failing tests**

Add to `tests/audio_bindings.rs`:

```rust
#[test]
fn bus_write_and_patch() {
    let cmds = run_and_capture_cmds(
        "var m = Bus.new()\n\
         m.write(Osc.saw(110))\n\
         Out.patch(m)",
    );
    assert_eq!(cmds, vec![
        saw(110.0),
        Cmd::BusWrite { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(1) },
        Cmd::SetRoot { bus: BusId(1) },
    ]);
}
```

(First allocated bus is `BusId(1)` — `0` is the reserved master.)

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: FAIL — `Bus` unknown.

- [ ] **Step 3: Implement `Bus`**

In `bindings_audio.rs`:

```rust
#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct BusObj {
    pub tag: u8,
    pub id: u16,
}
impl WrenForeign for BusObj {
    fn module_name() -> &'static str { "main" }
    fn class_name() -> &'static str { "Bus" }
}

pub(crate) fn bus_new_impl<S: SlotApi>(vm: &S) {
    let id = audio::alloc_bus_id();
    unsafe { vm.new_foreign_in(0, BusObj { tag: TAG_BUS, id }) };
}
pub(crate) fn bus_write_impl<S: SlotApi>(vm: &S) {
    let id = unsafe { vm.foreign_mut::<BusObj>(0) }.id;
    let src = arg_input(vm, 1);
    audio::bus_write(src, id);
}
```

Insert a `Bus` arm into `arg_input`'s tag match (above the default arm), so a bus
used as an operand reads its signal:

```rust
                TAG_BUS => {
                    let b = unsafe { vm.foreign_mut::<BusObj>(slot) };
                    Input::Bus(deluge_audio_graph::BusId(b.id))
                }
```

Extend `node_patch_impl` to set a `Bus` as root directly, else write a
`Node`/`Port` to the master bus (read the tag to discriminate):

```rust
pub(crate) fn node_patch_impl<S: SlotApi>(vm: &S) {
    // SAFETY: the patch argument is one of our tagged foreign objects.
    let tag = unsafe { *vm.foreign_mut::<u8>(1) };
    if tag == TAG_BUS {
        let bus = unsafe { vm.foreign_mut::<BusObj>(1) }.id;
        audio::set_root(bus);
    } else {
        let src = arg_input(vm, 1);
        audio::bus_write(src, audio::MASTER_BUS);
        audio::set_root(audio::MASTER_BUS);
    }
}
```

(This replaces the Task 2 `node_patch_impl`, which only handled the master-bus
sugar. Now `Out.patch` accepts Node/Port/Bus.)

- [ ] **Step 4: Register `Bus`**

`register_audio`: `method("main", "Bus", true, "new_()", bus_new_impl::<S>);` and `method("main", "Bus", false, "write(_)", bus_write_impl::<S>);`. Add matching `#[cfg(feature = "wren-sys-backend")]` `extern "C"` wrappers + `METHODS` entries. `Bus` is created via `new_foreign_in` (like `Node`), so it has **no `construct` and no `CLASSES` entry**. In `prelude.wren`, declare it exactly:

```wren
foreign class Bus {
  foreign static new_()      // returns a fresh Bus foreign in slot 0
  foreign write(src)
  static new() { new_() }    // the public Bus.new() from the spec
}
```

- [ ] **Step 5: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core
git commit -m "feat(wren-core): buses — Bus.new/.write, bus-as-input, Out.patch(bus)"
```

---

## Task 6: Explicit lifecycle — `Node.free()`

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs`, `src/bindings.rs`, `wren/prelude.wren`, `tests/audio_bindings.rs`

**Interfaces:**
- Produces: `node_free_impl` (`.free()` → `audio::free(id)`, which returns the id to the free-list and emits `Cmd::Free`).

- [ ] **Step 1: Write the failing test**

Add to `tests/audio_bindings.rs`:

```rust
#[test]
fn free_emits_free_and_reuses_id() {
    // Allocate n0, free it, allocate again → id 0 reused.
    let cmds = run_and_capture_cmds(
        "var a = Osc.saw(110)\n\
         a.free()\n\
         var b = Osc.saw(220)",
    );
    assert_eq!(cmds, vec![
        Cmd::NewNode { node: NodeId(0), kind: Kind::Saw, args: [Input::Const(110.0), Input::Const(0.0), Input::Const(0.0)] },
        Cmd::Free { node: NodeId(0) },
        Cmd::NewNode { node: NodeId(0), kind: Kind::Saw, args: [Input::Const(220.0), Input::Const(0.0), Input::Const(0.0)] },
    ]);
}
```

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: FAIL — `free` unknown.

- [ ] **Step 3: Implement `.free()`**

In `bindings_audio.rs`:

```rust
pub(crate) fn node_free_impl<S: SlotApi>(vm: &S) {
    audio::free(self_id(vm));
}
```

Register `method("main", "Node", false, "free()", node_free_impl::<S>);` + wrapper + `METHODS` entry; add `foreign free()` to the `Node` class in `prelude.wren`.

- [ ] **Step 4: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: PASS (id reuse confirms the free-list works end to end through the VM).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core
git commit -m "feat(wren-core): explicit Node.free() lifecycle"
```

---

## Task 7: End-to-end golden audio + compat check

**Files:**
- Modify: `crates/deluge-wren-core/src/test_support.rs` (add `EngineHost` + render helper), `tests/audio_bindings.rs`
- Reference: `wren-firmware/examples/midi_synth.wren`

**Interfaces:**
- Produces: `EngineHost` (owns a concrete `Engine`, applies `audio_cmd` to it) + `run_and_render(src, out: &mut [StereoFrame])` helper.

- [ ] **Step 1: Add `EngineHost` + render helper to `test_support.rs`**

```rust
use deluge_audio_graph::{Engine, StereoFrame};

type TestEng = Engine<32, 64, 128, 8>;

pub struct EngineHost {
    pub eng: TestEng,
}
impl Host for EngineHost {
    fn now_ms(&mut self) -> u64 { 0 }
    fn cv_set(&mut self, _c: u8, _v: f32) {}
    fn gate_set(&mut self, _c: u8, _o: bool) {}
    fn midi_tx(&mut self, _m: &[u8]) {}
    fn led(&mut self, _i: u8, _o: bool) {}
    fn oled_clear(&mut self) {}
    fn oled_text(&mut self, _x: usize, _y: usize, _t: &[u8]) {}
    fn oled_pixel(&mut self, _x: usize, _y: usize, _o: bool) {}
    fn oled_show(&mut self) {}
    fn audio_cmd(&mut self, cmd: crate::Cmd) { self.eng.apply(cmd); }
}

static mut ENGINE_HOST: Option<EngineHost> = None;

/// Boot a VM, run `src`, then render one 32-frame block into `out`.
pub fn run_and_render(src: &str, out: &mut [StereoFrame; 32]) {
    unsafe {
        let slot = &mut *core::ptr::addr_of_mut!(ENGINE_HOST);
        *slot = Some(EngineHost { eng: TestEng::new(44_100.0) });
        crate::set_host(slot.as_mut().unwrap());
        let vm = wren_sys::boot_with_foreign(crate::METHODS, crate::CLASSES);
        assert!(!vm.is_null());
        assert_eq!(wren_sys::interpret(vm, c"main".as_ptr(), crate::prelude_ptr()), wren_sys::WREN_RESULT_SUCCESS);
        let mut buf = [0u8; 8192];
        let n = src.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&src.as_bytes()[..n]);
        buf[n] = 0;
        assert_eq!(wren_sys::interpret(vm, c"main".as_ptr(), buf.as_ptr() as *const core::ffi::c_char), wren_sys::WREN_RESULT_SUCCESS, "script failed (line {})", LAST_ERR_LINE.load(Ordering::Relaxed));
        slot.as_mut().unwrap().eng.render(out);
        wren_sys::wrenFreeVM(vm);
        crate::reset();
    }
}
```

- [ ] **Step 2: Write the failing golden + compat tests**

Add to `tests/audio_bindings.rs`:

```rust
use deluge_wren_core::test_support::run_and_render;
use deluge_audio_graph::StereoFrame;

#[test]
fn golden_saw_lpf_renders_expected_block() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Osc.saw(110).lpf(800))", &mut out);
    // Characterization golden: pin the first 4 L samples (regenerate only on an
    // intended, reviewed output change). Values captured from a first run.
    let expected = [/* FILL: run once, paste the 4 values */];
    for i in 0..4 {
        assert!((out[i].l - expected[i]).abs() < 1e-6, "sample {i}: {} vs {}", out[i].l, expected[i]);
    }
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
}

#[test]
fn ports_and_buses_render_finite() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var m = Bus.new()\n\
         var s = Split.new(Osc.saw(110))\n\
         m.write(s.out(0))\n\
         m.write(s.out(1))\n\
         Out.patch(m)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 2.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn midi_synth_example_parses_and_runs() {
    let src = include_str!("../../../wren-firmware/examples/midi_synth.wren");
    // Runs without a VM error (may reference Midi/output — that's fine, those
    // bindings exist); we only assert it boots + executes top-level code.
    let _ = deluge_wren_core::test_support::run_and_capture_cmds(src);
}
```

- [ ] **Step 3: Fill the golden values**

Run the golden test once (with `expected` as `[0.0; 4]`) to see the actual first four `out[i].l` values in the failure message; paste them into `expected` (documented as a characterization pin). If `midi_synth.wren` uses a construct not yet supported, adjust the compat test to a smaller representative script and note it.

- [ ] **Step 4: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: PASS. Also `cargo build --target x86_64-unknown-linux-gnu -p wren-firmware` — still PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core
git commit -m "test(wren-core): end-to-end golden audio + midi_synth compat"
```

---

## Self-review notes

- **Spec coverage:** in-place refactor + module split (Task 2); `Host::audio_cmd` retarget (Task 2); parity surface (Task 2); ports (Task 4); buses + `Out.patch` unifier (Task 5); `.free()` lifecycle (Task 6); client-side allocators + caps + contract (Task 2 `audio.rs`); `wren-firmware` re-wire (Task 3); capturing-host Cmd tests (Task 2) + applying-host golden (Task 7) + `midi_synth` compat (Task 7); `Debug`/`PartialEq` derives (Task 1).
- **Deliberate deviations from the spec:** tests live in `deluge-wren-core/tests/` via an extended `test_support`, not `tools/wren-web-debug` (in-workspace, runs in CI; wren-web-debug wiring is an optional nicety per the spec's own note). `Node`/`Port`/`Bus` foreign args are disambiguated by a leading `tag: u8` on each `repr(C)` object (read via `foreign_mut::<u8>`), avoiding any `SlotApi`/backend change.
- **Atomicity note:** Task 2 is a single atomic type-swap (the crate can't compile mid-swap); it is verified by compile + existing CV tests + new Cmd-sequence tests. Task 3 restores the whole-workspace build (Task 2 intentionally leaves `wren-firmware` broken, but `-p deluge-wren-core` builds and tests green).
- **Known follow-ups (not P1):** `tools/wren-web` (wasm) migration; device performance validation; wiring `wren-web-debug`'s `RecordingHost` to an `Engine` for interactive use.
