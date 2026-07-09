# Mod-2 Sample&Hold / Slew / Steps Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add three modulation-utility nodes — `SampleHold` (latch on clock edge), `Slew` (one-pole glide), `Steps` (inline-array step sequencer) — as Wren `SampleHold.new`/`Slew.new`/`Steps.new`.

**Architecture:** Three tiny per-sample mono kernels in a new `deluge-dsp-kernels/src/modutil.rs` (no buffer, no pool — `Steps` stores its values inline in `[f32; 16]`). Three mono graph Kinds like `Svf`/`Lfo`. Named Wren factories; `Steps.new` reads a Wren list and emits per-value `SetParam`s.

**Tech Stack:** Rust, `no_std`/no-heap, Wren via `wren-sys`, proptest.

## Global Constraints

- **`no_std`, no heap, pure `f32`, deterministic.** Per-sample mono kernels, no buffer, no pool.
- **Rising-edge clock:** `fn rising(prev, cur) -> bool { prev <= 0.0 && cur > 0.0 }` — a bipolar square LFO or a unipolar-from-0 gate both clock correctly.
- **`SampleHold`:** ports 0=input, 1=clock. `if rising { held = input }; out = held`.
- **`Slew`:** ports 0=input, 1=time (s). `c = (dt / time.max(dt)).min(1.0); z += (input − z)·c; out = z`.
- **`Steps`:** port 0=clock; inline `[f32; MAX_STEPS=16]` + `len` + `idx` + `first`. On a rising edge: first edge keeps `idx=0` (plays step 0), subsequent advance `idx = (idx+1) % len`; `out = values[idx]`. `set_param` 0 = len (clamp 1..16), `k∈1..=16` = `values[k−1]`.
- **Param map (`Steps` only):** `set_param` 0=len, 1..16=step values. `SampleHold`/`Slew` have no params (all ports). `out_width` = 1 for all three.
- **Wren:** `SampleHold.new(input, clock)`, `Slew.new(input, time)`, `Steps.new([values], clock)`; `Steps.new` reads the Wren list (like `Wavetable.from`) → `set_param(0, len)` + `set_param(k+1, values[k])`. All mono (`return_node`, plain `new_node`). No new setters.
- Host tests run with `--target x86_64-unknown-linux-gnu`.

---

### Task 1: `modutil.rs` kernels

**Files:**
- Create: `crates/deluge-dsp-kernels/src/modutil.rs`
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (add `pub mod modutil;`)

**Interfaces:**
- Consumes: `crate::In`.
- Produces: `pub struct SampleHold`/`Slew`/`Steps`; `pub const MAX_STEPS`; each with `new()` + `process(...)`; `Steps::set_len`/`set_value`. Task 2 consumes these.

- [ ] **Step 1: Write the module (impl + tests)**

Create `crates/deluge-dsp-kernels/src/modutil.rs`:

```rust
//! Modulation utilities: sample & hold, one-pole slew (glide), and an inline
//! step sequencer. Tiny per-sample mono kernels — no heap, no buffer.

use crate::In;

/// Rising-edge detector: the clock crossed 0 upward.
#[inline]
fn rising(prev: f32, cur: f32) -> bool {
    prev <= 0.0 && cur > 0.0
}

/// Sample & hold: latch `input` on each rising clock edge. Ports 0=input, 1=clock.
#[derive(Clone, Copy)]
pub struct SampleHold {
    held: f32,
    prev_clock: f32,
}
impl SampleHold {
    pub fn new() -> SampleHold {
        SampleHold { held: 0.0, prev_clock: 0.0 }
    }
    pub fn process(&mut self, input: In, clock: In, out: &mut [f32]) {
        for i in 0..out.len() {
            let c = clock.at(i);
            if rising(self.prev_clock, c) {
                self.held = input.at(i);
            }
            out[i] = self.held;
            self.prev_clock = c;
        }
    }
}
impl Default for SampleHold {
    fn default() -> Self {
        Self::new()
    }
}

/// One-pole slew / lag (glide). Ports 0=input, 1=time (seconds).
#[derive(Clone, Copy)]
pub struct Slew {
    z: f32,
}
impl Slew {
    pub fn new() -> Slew {
        Slew { z: 0.0 }
    }
    pub fn process(&mut self, input: In, time: In, dt: f32, out: &mut [f32]) {
        for i in 0..out.len() {
            let c = (dt / time.at(i).max(dt)).min(1.0);
            self.z += (input.at(i) - self.z) * c;
            out[i] = self.z;
        }
    }
}
impl Default for Slew {
    fn default() -> Self {
        Self::new()
    }
}

/// Step sequencer over an inline value array, clocked by port 0.
pub const MAX_STEPS: usize = 16;
#[derive(Clone, Copy)]
pub struct Steps {
    values: [f32; MAX_STEPS],
    len: usize,
    idx: usize,
    prev_clock: f32,
    first: bool,
}
impl Steps {
    pub fn new() -> Steps {
        Steps { values: [0.0; MAX_STEPS], len: 1, idx: 0, prev_clock: 0.0, first: true }
    }
    pub fn set_len(&mut self, n: u8) {
        self.len = (n as usize).clamp(1, MAX_STEPS);
    }
    /// `i` is the 1-based param index (`set_param` k → `values[k−1]`).
    pub fn set_value(&mut self, i: usize, v: f32) {
        if (1..=MAX_STEPS).contains(&i) {
            self.values[i - 1] = v;
        }
    }
    pub fn process(&mut self, clock: In, out: &mut [f32]) {
        for i in 0..out.len() {
            let c = clock.at(i);
            if rising(self.prev_clock, c) {
                if self.first {
                    self.first = false;
                } else {
                    self.idx = (self.idx + 1) % self.len.max(1);
                }
            }
            out[i] = self.values[self.idx.min(MAX_STEPS - 1)];
            self.prev_clock = c;
        }
    }
}
impl Default for Steps {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;

    // A clock that is −1 everywhere except +1 on the given sample indices' runs.
    fn clock_with_edges(n: usize, edges: &[usize]) -> std::vec::Vec<f32> {
        let mut c = std::vec![-1.0f32; n];
        // Hold +1 from each edge for 3 samples so the rising transition is clean.
        for &e in edges {
            for s in e..(e + 3).min(n) {
                c[s] = 1.0;
            }
        }
        c
    }

    #[test]
    fn sample_hold_latches_and_holds() {
        let n = 40;
        let input: std::vec::Vec<f32> = (0..n).map(|i| i as f32 * 0.01).collect();
        let clock = clock_with_edges(n, &[10, 20]);
        let mut sh = SampleHold::new();
        let mut out = std::vec![0.0f32; n];
        sh.process(In::A(&input), In::A(&clock), &mut out);
        assert!(out[5].abs() < 1e-6, "before any edge: initial held 0");
        assert!((out[15] - 0.10).abs() < 1e-6, "latched input at edge 10 (0.10)");
        assert!((out[19] - 0.10).abs() < 1e-6, "holds between edges");
        assert!((out[25] - 0.20).abs() < 1e-6, "latched input at edge 20 (0.20)");
    }

    #[test]
    fn slew_converges_and_passes_dc() {
        let dt = 1.0 / 48_000.0;
        let n = 2_000;
        let input = std::vec![1.0f32; n]; // step 0→1
        let time = std::vec![0.005f32; n]; // 5 ms
        let mut s = Slew::new();
        let mut out = std::vec![0.0f32; n];
        s.process(In::A(&input), In::A(&time), dt, &mut out);
        assert!(out[0] > 0.0 && out[0] < 0.1, "starts rising from 0: {}", out[0]);
        assert!(out[1] > out[0], "monotonic rise");
        assert!(out[n - 1] > 0.99, "converges to the step: {}", out[n - 1]);
    }

    #[test]
    fn steps_walks_values_wrapping() {
        let n = 50;
        let clock = clock_with_edges(n, &[10, 20, 30, 40]);
        let mut st = Steps::new();
        st.set_len(3);
        st.set_value(1, 10.0);
        st.set_value(2, 20.0);
        st.set_value(3, 30.0);
        let mut out = std::vec![0.0f32; n];
        st.process(In::A(&clock), &mut out);
        assert!((out[5] - 10.0).abs() < 1e-6, "step 0 before any advance");
        assert!((out[15] - 10.0).abs() < 1e-6, "first edge keeps step 0");
        assert!((out[25] - 20.0).abs() < 1e-6, "2nd edge → step 1");
        assert!((out[35] - 30.0).abs() < 1e-6, "3rd edge → step 2");
        assert!((out[45] - 10.0).abs() < 1e-6, "4th edge → wrap to step 0");
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn slew_stays_bounded(amp in -1.0f32..1.0, time_ms in 0.1f32..100.0) {
            let dt = 1.0 / 48_000.0;
            let input = std::vec![amp; 2000];
            let time = std::vec![time_ms * 1e-3; 2000];
            let mut s = Slew::new();
            let mut out = std::vec![0.0f32; 2000];
            s.process(In::A(&input), In::A(&time), dt, &mut out);
            for &v in &out {
                prop_assert!(v.is_finite() && v.abs() <= 1.0001, "slew unbounded: {v}");
            }
        }
    }
}
```

- [ ] **Step 2: Register the module + run**

In `crates/deluge-dsp-kernels/src/lib.rs`, add `pub mod modutil;` after `pub mod math;` (keep ordered):

```rust
pub mod math;
pub mod modutil;
pub mod noise;
```

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels modutil`
Expected: PASS — the module ships impl + tests together (a self-contained new module; no separate RED step); the 4 tests pass, no warnings.

(If a behavioral test fails on a number, STOP and report DONE_WITH_CONCERNS with observed vs expected — do not weaken the assertion.)

- [ ] **Step 3: Commit**

```bash
git add crates/deluge-dsp-kernels/src/modutil.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(dsp-kernels): SampleHold / Slew / Steps modulation utilities"
```

---

### Task 2: Graph — `Kind::SampleHold`/`Slew`/`Steps`

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs` (import, Kind, State, `Node::new`, `set_param`, render arms, tests)

**Interfaces:**
- Consumes: `deluge_dsp_kernels::modutil::{SampleHold, Slew, Steps}`.
- Produces: `Kind::SampleHold`/`Slew`/`Steps`. Task 3's factories create them.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-audio-graph/src/node.rs`'s `mod tests`:

```rust
    #[test]
    fn sample_hold_node_latches() {
        let mut n = Node::new(Kind::SampleHold, 0);
        assert_eq!(Node::out_width(Kind::SampleHold), 1);
        let input: [f32; 16] = core::array::from_fn(|i| i as f32 * 0.1);
        let clock: [f32; 16] = core::array::from_fn(|i| if (4..7).contains(&i) { 1.0 } else { -1.0 });
        let ins = [In::A(&input), In::A(&clock), In::A(&[0.0; 16])];
        let mut buf = [0.0f32; 16];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!(buf[2].abs() < 1e-6, "held 0 before edge");
        assert!((buf[8] - 0.4).abs() < 1e-6, "latched input at edge (sample 4 = 0.4)");
    }

    #[test]
    fn steps_node_sequences() {
        let mut n = Node::new(Kind::Steps, 0);
        assert_eq!(Node::out_width(Kind::Steps), 1);
        n.set_param(0, 2.0); // len
        n.set_param(1, 5.0); // values[0]
        n.set_param(2, 9.0); // values[1]
        let clock: [f32; 32] = core::array::from_fn(|i| if (8..11).contains(&i) || (16..19).contains(&i) { 1.0 } else { -1.0 });
        let ins = [In::A(&clock), In::A(&[0.0; 32]), In::A(&[0.0; 32])];
        let mut buf = [0.0f32; 32];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!((buf[4] - 5.0).abs() < 1e-6, "step 0");
        assert!((buf[12] - 5.0).abs() < 1e-6, "first edge keeps step 0");
        assert!((buf[20] - 9.0).abs() < 1e-6, "2nd edge → step 1");
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph sample_hold steps`
Expected: FAIL — no variant `SampleHold`/`Steps`.

- [ ] **Step 3: Add the variants, state, construction, params**

In `crates/deluge-audio-graph/src/node.rs`:

Add the import (extend the existing `deluge_dsp_kernels` imports):

```rust
use deluge_dsp_kernels::modutil::{SampleHold, Slew, Steps};
```

Add to `enum Kind` (after `Lfo`):

```rust
    Lfo,
    SampleHold,
    Slew,
    Steps,
```

Add to `enum State` (after `Lfo(Lfo)`):

```rust
    SampleHold(SampleHold),
    Slew(Slew),
    Steps(Steps),
```

In `Node::new`, add arms (after `Kind::Lfo => …`):

```rust
            Kind::SampleHold => State::SampleHold(SampleHold::new()),
            Kind::Slew => State::Slew(Slew::new()),
            Kind::Steps => State::Steps(Steps::new()),
```

(`out_width` needs no change — all mono, fall through `_ => 1`.)

In `set_param`, add an arm (only `Steps` has params):

```rust
            State::Steps(s) => match param {
                0 => s.set_len(value as u8),
                k => s.set_value(k as usize, value),
            },
```

- [ ] **Step 4: Add the render arms**

In `process_resolved`, add after the `Kind::Lfo` arm:

```rust
            Kind::SampleHold => {
                if let State::SampleHold(sh) = &mut self.state {
                    sh.process(ins[0], ins[1], outs.port(0));
                }
            }
            Kind::Slew => {
                if let State::Slew(s) = &mut self.state {
                    s.process(ins[0], ins[1], dt, outs.port(0));
                }
            }
            Kind::Steps => {
                if let State::Steps(s) = &mut self.state {
                    s.process(ins[0], outs.port(0)); // clock on port 0
                }
            }
```

- [ ] **Step 5: Run the new tests + full suite**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph sample_hold steps`
Expected: PASS (2 new tests).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: PASS — all existing tests too.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(audio-graph): SampleHold / Slew / Steps nodes"
```

---

### Task 3: Wren `SampleHold.new` / `Slew.new` / `Steps.new`

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_sh_impl`/`node_slew_impl`/`node_steps_impl` + wrappers, register)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (wren-sys registration)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`sh_`/`slew_`/`steps_` + `class SampleHold`/`Slew`/`Steps`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Kind::{SampleHold, Slew, Steps}` (Task 2), `deluge_dsp_kernels::modutil::MAX_STEPS`, `audio::{alloc_node_id, new_node, set_param}`, `arg_input`, `return_node`, the Wren list API (`get_list_count`/`get_list_element`/`ensure_slots`).
- Produces: Wren `SampleHold.new`/`Slew.new`/`Steps.new`.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn sample_hold_and_slew_factories_emit_nodes() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var a = SampleHold.new(Noise.pink(), Osc.square(4))\nvar b = Slew.new(a, 0.05)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::SampleHold, .. })), "S&H: {cmds:?}");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Slew, .. })), "Slew: {cmds:?}");
}

#[test]
fn steps_factory_emits_len_and_values() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var s = Steps.new([10, 20, 30], Osc.square(2))");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Steps, .. })), "Steps: {cmds:?}");
    // len = 3 → SetParam(0, 3)
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value } if (*value - 3.0).abs() < 1e-4)), "len: {cmds:?}");
    // values → SetParam(1, 10) SetParam(2, 20) SetParam(3, 30)
    for (p, v) in [(1u8, 10.0f32), (2, 20.0), (3, 30.0)] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, value } if *param == p && (*value - v).abs() < 1e-4)), "value {p}={v}: {cmds:?}");
    }
}

#[test]
fn steps_renders_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Steps.new([0.2, -0.2], Osc.square(1000)))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings sample_hold steps slew`
Expected: FAIL — Wren `SampleHold`/`Slew`/`Steps` undefined.

- [ ] **Step 3: Add the factories**

In `crates/deluge-wren-core/src/bindings_audio.rs`, add (near the other factories):

```rust
/// `Node.sh_(input, clock)` — sample & hold. Ports 0=input, 1=clock.
pub(crate) fn node_sh_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let clock = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::SampleHold, [input, clock, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_sh(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_sh_impl(&vm);
}

/// `Node.slew_(input, time)` — one-pole glide. Ports 0=input, 1=time.
pub(crate) fn node_slew_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let time = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Slew, [input, time, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_slew(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_slew_impl(&vm);
}

/// `Node.steps_(values, clock)` — step sequencer. Reads the Wren list (slot 1)
/// into `set_param` calls: param 0 = length, param k+1 = values[k] (up to
/// MAX_STEPS; longer lists are truncated). Clock on port 0.
pub(crate) fn node_steps_impl<S: SlotApi>(vm: &S) {
    let count = vm.get_list_count(1).max(0) as usize;
    let len = count.min(deluge_dsp_kernels::modutil::MAX_STEPS);
    let clock = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Steps, [clock, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, len as f32);
    vm.ensure_slots(3); // slot 2 = per-element scratch
    for k in 0..len {
        vm.get_list_element(1, k as i32, 2);
        let v = vm.get_f(2) as f32;
        audio::set_param(id, (k + 1) as u8, v);
    }
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_steps(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_steps_impl(&vm);
}
```

Register in `register_audio`:

```rust
    method("main", "Node", true, "sh_(_,_)", node_sh_impl::<S>);
    method("main", "Node", true, "slew_(_,_)", node_slew_impl::<S>);
    method("main", "Node", true, "steps_(_,_)", node_steps_impl::<S>);
```

- [ ] **Step 4: Register in the wren-sys table + prelude**

In `crates/deluge-wren-core/src/bindings.rs`, add near the other audio registrations:

```rust
    static_method("Node", "sh_(_,_)", bindings_audio::node_sh),
    static_method("Node", "slew_(_,_)", bindings_audio::node_slew),
    static_method("Node", "steps_(_,_)", bindings_audio::node_steps),
```

In `crates/deluge-wren-core/wren/prelude.wren`, add to `foreign class Node`:

```wren
  foreign static sh_(input, clock)
  foreign static slew_(input, time)
  foreign static steps_(values, clock)
```

And the sugar classes (near the other modulation sources, e.g. after `class LFO`):

```wren
// Sample & hold: latch `input` on each rising edge of `clock` (any signal — an
// LFO, a square, a metro):
//   var rnd = SampleHold.new(Noise.pink(), Osc.square(4))   // stepped random
class SampleHold {
  static new(input, clock) { Node.sh_(input, clock) }
}

// Slew / glide: one-pole lag over `time` seconds. Smooths steps / portamento:
//   osc.freq = Slew.new(pitchSeq, 0.02)
class Slew {
  static new(input, time) { Node.slew_(input, time) }
}

// Step sequencer: cycles a list of values, one per rising `clock` edge (up to 16
// steps; the first clock plays step 0):
//   var seq = Steps.new([0, 7, 5, 12], Osc.square(2))
//   osc.freq = seq.to(110, 880)
class Steps {
  static new(values, clock) { Node.steps_(values, clock) }
}
```

- [ ] **Step 5: Run the new tests + full suite**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings sample_hold steps slew`
Expected: PASS (all 3).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core`
Expected: PASS — existing bindings unchanged, no warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): SampleHold / Slew / Steps factories"
```

---

## Scope notes (deliberate deferrals)

- **`Steps` trigger/gate reset (to step 0), per-step gates/ratchets, > 16 steps (pooled), swing**; **`Slew` separate rise/fall or linear mode**; **internal clock / tempo** for S&H and Steps — later.
- **Mod-3 modulation scaling / attenuators / macros** (extends Mod-1's `.to()`).

## Post-implementation

After all three tasks pass, use **superpowers:finishing-a-development-branch** to verify the suites and merge.
