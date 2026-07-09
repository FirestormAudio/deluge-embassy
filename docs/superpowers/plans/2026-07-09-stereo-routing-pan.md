# Stereo Bus Routing + Pan Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the audio graph a stereo signal path — a constant-power `Pan` mono→stereo node and per-side bus-write gains — so mono sources can be placed in the stereo field and width-2 (stereo) nodes route distinct L/R to the output.

**Architecture:** A stereo signal is a width-2 node (port 0 = L, port 1 = R) — reusing the graph's existing width-2 machinery (`Kind::Split2`/`OutView::pair`). `Kind::Pan` is a stateless mono→stereo node using a trig-free constant-power `sqrt` law. Bus writes gain a per-side `(gl, gr)` variant (`Cmd::BusWriteGains`) added *alongside* the unchanged center `Cmd::BusWrite`; a stereo source routes as two writes `(1,0)`/`(0,1)`. The Wren `NodeObj` gains a `width` byte so `Out.patch`/`Bus.write` route stereo vs mono.

**Tech Stack:** Rust, `no_std`/no-heap, Wren via `wren-sys`, `libm` for `sqrtf`.

## Global Constraints

- **Stereo convention:** a stereo signal IS a width-2 node — **port 0 = L, port 1 = R**. Mono signals stay width-1 (route to center, L = R).
- **Pan law (constant power, no trig):** `p = (position.clamp(-1.0, 1.0) + 1.0) * 0.5; gl = sqrt(1.0 - p); gr = sqrt(p)`. Hard-left (−1) → `(1,0)`; center (0) → `(0.707…, 0.707…)`; hard-right (+1) → `(0,1)`; `gl² + gr² == 1`.
- **`Kind::Pan`:** stateless; ports 0 = input (mono), 1 = position (a *port*, modulatable); `out_width` = 2; no control params.
- **Per-side write gains:** the mono center write `(gl,gr) = (1,1)` MUST stay byte-identical to today. A stereo (width-2) source routes as TWO writes: `(node.out(0), bus, 1.0, 0.0)` and `(node.out(1), bus, 0.0, 1.0)`.
- **Additive, low-churn:** keep `Cmd::BusWrite { src, bus }` and `Engine::bus_write(src, bus)` and `audio::bus_write(src, bus)` unchanged (center). ADD `Cmd::BusWriteGains { src, bus, gl, gr }`, `Engine::bus_write_gains(src, bus, gl, gr)`, `audio::bus_write_gains(src, bus, gl, gr)`. Do NOT change the signatures of the existing three — no existing bus test/assertion should need editing.
- **`NodeObj` layout:** `{ tag: u8, width: u8, id: u16 }` (still 4 bytes). `arg_input` reads only the leading `tag` byte for foreigns, so the added `width` field is safe. Mono factories set `width = 1`; the `Pan` factory sets `width = 2`.
- **Mono context for a stereo node:** `arg_input` on a width-2 node yields its **port 0 (L)** (existing behavior — `arg_input` always uses port 0 for a `Node`). Documented, not special-cased.
- `no_std`, no heap, deterministic. Host tests run with `--target x86_64-unknown-linux-gnu` (repo default target is `armv7a-none-eabihf`; plain `cargo test` fails to link).

---

### Task 1: `pan_gains` constant-power helper (kernels)

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/math.rs` (add `pan_gains` + tests)

**Interfaces:**
- Produces: `pub fn pan_gains(position: f32) -> (f32, f32)` returning `(gain_l, gain_r)`. Task 2 (the graph `Kind::Pan` arm) consumes it.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-dsp-kernels/src/math.rs`'s test module (if it has no `#[cfg(test)] mod tests`, create one at the end of the file):

```rust
#[cfg(test)]
mod pan_tests {
    use super::*;

    #[test]
    fn pan_hard_left_and_right() {
        let (l, r) = pan_gains(-1.0);
        assert!((l - 1.0).abs() < 1e-6 && r.abs() < 1e-6, "hard left: {l},{r}");
        let (l, r) = pan_gains(1.0);
        assert!(l.abs() < 1e-6 && (r - 1.0).abs() < 1e-6, "hard right: {l},{r}");
    }

    #[test]
    fn pan_center_is_minus_3db() {
        let (l, r) = pan_gains(0.0);
        let c = (0.5f32).sqrt(); // 0.7071…
        assert!((l - c).abs() < 1e-6 && (r - c).abs() < 1e-6, "center: {l},{r}");
    }

    #[test]
    fn pan_is_constant_power_and_clamped() {
        // gl² + gr² == 1 across the range, and out-of-range clamps.
        for k in -12..=12 {
            let pos = k as f32 / 10.0; // -1.2 .. 1.2 (exercises the clamp)
            let (l, r) = pan_gains(pos);
            assert!((l * l + r * r - 1.0).abs() < 1e-5, "power@{pos}: {l},{r}");
            assert!(l >= 0.0 && r >= 0.0);
        }
        // Beyond the range stays pinned to the ends.
        assert_eq!(pan_gains(-5.0), pan_gains(-1.0));
        assert_eq!(pan_gains(5.0), pan_gains(1.0));
    }
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels pan`
Expected: FAIL — `cannot find function pan_gains`.

- [ ] **Step 3: Implement `pan_gains`**

Add to `crates/deluge-dsp-kernels/src/math.rs` (near the top-level fns, not inside `mod tests`):

```rust
/// Constant-power stereo pan gains for `position ∈ [-1, +1]` (clamped):
/// `-1` = hard left `(1, 0)`, `0` = center `(√½, √½)` (−3 dB), `+1` = hard
/// right `(0, 1)`. `gain_l² + gain_r² == 1`. Trig-free (`sqrtf`; the Cortex-A9
/// has hardware `VSQRT`).
#[inline]
pub fn pan_gains(position: f32) -> (f32, f32) {
    let p = (position.clamp(-1.0, 1.0) + 1.0) * 0.5; // → [0, 1]
    (libm::sqrtf(1.0 - p), libm::sqrtf(p))
}
```

(`libm` is already a dependency of this crate — confirm `libm::sqrtf` resolves; other kernels use `libm` similarly.)

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels pan`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/math.rs
git commit -m "feat(dsp-kernels): constant-power pan_gains helper"
```

---

### Task 2: `Kind::Pan` graph node

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs` (Kind, `Node::new`, `out_width`, render arm, tests)

**Interfaces:**
- Consumes: `deluge_dsp_kernels::math::pan_gains` (Task 1).
- Produces: `Kind::Pan` (a stateless width-2 node; ports 0 = input, 1 = position). Task 4's `node_pan_impl` creates it.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-audio-graph/src/node.rs`'s `mod tests`:

```rust
    #[test]
    fn pan_node_writes_l_r_constant_power() {
        let mut n = Node::new(Kind::Pan, 0);
        assert_eq!(Node::out_width(Kind::Pan), 2);
        let x = [0.8f32; 8];
        // hard left
        let posl = [-1.0f32; 8];
        let ins = [In::A(&x), In::A(&posl), In::A(&[0.0; 8])];
        let mut p0 = [0.0f32; 8];
        let mut p1 = [0.0f32; 8];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!((p0[0] - 0.8).abs() < 1e-5, "L port = input at hard-left: {}", p0[0]);
        assert!(p1[0].abs() < 1e-5, "R port silent at hard-left: {}", p1[0]);

        // center → both ≈ 0.8 * 0.7071, constant power
        let posc = [0.0f32; 8];
        let ins = [In::A(&x), In::A(&posc), In::A(&[0.0; 8])];
        let mut p0 = [0.0f32; 8];
        let mut p1 = [0.0f32; 8];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        let c = 0.8 * (0.5f32).sqrt();
        assert!((p0[0] - c).abs() < 1e-5 && (p1[0] - c).abs() < 1e-5);
        // total power preserved
        assert!((p0[0] * p0[0] + p1[0] * p1[0] - 0.8 * 0.8).abs() < 1e-4);
    }

    #[test]
    fn pan_node_position_modulates() {
        // A per-sample position sweep moves energy from L to R.
        let mut n = Node::new(Kind::Pan, 0);
        let x = [1.0f32; 4];
        let pos = [-1.0f32, -0.3, 0.3, 1.0]; // sweep L→R
        let ins = [In::A(&x), In::A(&pos), In::A(&[0.0; 4])];
        let mut p0 = [0.0f32; 4];
        let mut p1 = [0.0f32; 4];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        // L decreasing, R increasing across the block
        assert!(p0[0] > p0[3], "L should fall L→R: {:?}", p0);
        assert!(p1[0] < p1[3], "R should rise L→R: {:?}", p1);
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph pan`
Expected: FAIL — `no variant Pan`.

- [ ] **Step 3: Add the `Kind::Pan` variant + construction**

In `crates/deluge-audio-graph/src/node.rs`:

Add `Pan` to the `Kind` enum (after `Split2`):

```rust
    Split2, // width-2 test node: input → both ports
    Pan,    // mono→stereo: port0 = L, port1 = R (constant-power)
    Wavetable,
    Delay,
```

In `Node::new`, add `Kind::Pan` to the stateless arm:

```rust
            Kind::Mul | Kind::Add | Kind::Sub | Kind::Split2 | Kind::Pan => State::Stateless,
```

In `out_width`, add the width-2 case:

```rust
    pub fn out_width(kind: Kind) -> usize {
        match kind {
            Kind::Split2 | Kind::Pan => 2,
            _ => 1,
        }
    }
```

- [ ] **Step 4: Add the render arm**

In `process_resolved`, add an arm (after the `Kind::Split2` arm). Import is already present (`node.rs` uses `math`); reference `math::pan_gains`:

```rust
            Kind::Pan => {
                // Mono→stereo constant-power pan. ins[0] = input, ins[1] =
                // position (∈[-1,1], modulatable). port0 = L, port1 = R.
                // Compute both ports' lengths up front, then borrow each.
                let n = outs.port(0).len();
                for i in 0..n {
                    let (gl, gr) = math::pan_gains(ins[1].at(i));
                    let x = ins[0].at(i);
                    outs.port(0)[i] = x * gl;
                    outs.port(1)[i] = x * gr;
                }
            }
```

- [ ] **Step 5: Run to verify they pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph pan`
Expected: PASS (2 new tests).

- [ ] **Step 6: Full graph suite (no regression)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: PASS — all existing tests too.

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(audio-graph): Kind::Pan constant-power mono→stereo node"
```

---

### Task 3: Per-side bus-write gains (engine)

**Files:**
- Modify: `crates/deluge-audio-graph/src/engine.rs` (`writes` array type, `bus_write`/`bus_write_gains`, `apply`, `render` apply loop, test)
- Modify: `crates/deluge-audio-graph/src/cmd.rs` (add `Cmd::BusWriteGains` variant)

**Interfaces:**
- Produces: `Cmd::BusWriteGains { src: Input, bus: BusId, gl: f32, gr: f32 }`; `Engine::bus_write_gains(&mut self, src: Input, bus: BusId, gl: f32, gr: f32)`. Task 4's `audio::bus_write_gains` emits the Cmd.

- [ ] **Step 1: Write the failing test**

Add to `crates/deluge-audio-graph/src/engine.rs`'s `mod tests`:

```rust
    #[test]
    fn per_side_write_gains_route_l_and_r_separately() {
        // node0 = const 0.5 (Add of const+0). Two gained writes: (1,0)→L only,
        // (0,1)→R only. Plus a plain center write from a second source to
        // prove (1,1) is unchanged.
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.apply(Cmd::BusWriteGains { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(0), gl: 1.0, gr: 0.0 });
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        e.render(&mut out);
        assert!((out[0].l - 0.5).abs() < 1e-6, "L got the (1,0) write");
        assert!(out[0].r.abs() < 1e-6, "R silent for a (1,0) write");

        // A (0,1) write lands only on R.
        let mut e2 = E::new(16.0);
        e2.create(NodeId(0), Kind::Add);
        *e2.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e2.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e2.apply(Cmd::BusWriteGains { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(0), gl: 0.0, gr: 1.0 });
        e2.set_root(BusId(0));
        let mut out2 = [StereoFrame::default(); 16];
        e2.render(&mut out2);
        assert!(out2[0].l.abs() < 1e-6 && (out2[0].r - 0.5).abs() < 1e-6);
    }
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph per_side`
Expected: FAIL — no variant `BusWriteGains`.

- [ ] **Step 3: Add the `Cmd::BusWriteGains` variant**

In `crates/deluge-audio-graph/src/cmd.rs`, add to the `Cmd` enum (after `BusWrite`):

```rust
    BusWrite { src: Input, bus: BusId },
    BusWriteGains { src: Input, bus: BusId, gl: f32, gr: f32 },
```

- [ ] **Step 4: Widen the writes array + add `bus_write_gains`**

In `crates/deluge-audio-graph/src/engine.rs`:

Change the `writes` field type (the `[None; NODES]` init is unchanged):

```rust
    writes: [Option<(Input, BusId, f32, f32)>; NODES],
```

Change `Engine::bus_write` to delegate, and add `bus_write_gains`:

```rust
    /// Record a center (L = R) bus write — unchanged mono behavior.
    pub fn bus_write(&mut self, src: Input, bus: BusId) {
        self.bus_write_gains(src, bus, 1.0, 1.0);
    }

    /// Record a bus write with per-side gains (`gl` → L, `gr` → R). A stereo
    /// source routes as two of these: `(port0, 1, 0)` and `(port1, 0, 1)`.
    pub fn bus_write_gains(&mut self, src: Input, bus: BusId, gl: f32, gr: f32) {
        if self.writes_len < self.writes.len() {
            self.writes[self.writes_len] = Some((src, bus, gl, gr));
            self.writes_len += 1;
        }
    }
```

Add the `apply` arm (after `Cmd::BusWrite`):

```rust
            Cmd::BusWrite { src, bus } => self.bus_write(src, bus),
            Cmd::BusWriteGains { src, bus, gl, gr } => self.bus_write_gains(src, bus, gl, gr),
```

- [ ] **Step 5: Apply the gains in `render`**

In `render`, update the bus-write apply loop. Change the destructure and the accumulation:

```rust
        // Apply bus writes (per-side gains; mono center = (1,1)).
        let arr = unsafe { &*self.outs.get() };
        for w in 0..self.writes_len {
            if let Some((src, bus, gl, gr)) = self.writes[w] {
                let b = bus.0 as usize;
                for i in 0..BLOCK {
                    let v = match src {
                        Input::Const(c) => c,
                        Input::Node { node, port } => match self.arena.out_base(node) {
                            Some(base) if base + (port as usize) < OUTS => arr[base + port as usize][i],
                            _ => 0.0, // dangling ref or out-of-range port → contributes silence
                        },
                        Input::Bus(_) => 0.0, // bus→bus not in P0
                    };
                    self.bus_l[b][i] += v * gl;
                    self.bus_r[b][i] += v * gr;
                }
            }
        }
```

- [ ] **Step 6: Run the new test + the full suite**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: PASS — the new `per_side_write_gains…` test AND every existing bus/render test (center writes `(1,1)` are byte-identical, so `two_nodes_sum_into_master_bus`, `render_clamps_to_unit_range`, `dangling_and_oob_refs…` all still pass).

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-audio-graph/src/engine.rs crates/deluge-audio-graph/src/cmd.rs
git commit -m "feat(audio-graph): per-side bus-write gains (Cmd::BusWriteGains)"
```

---

### Task 4: Wren `Pan` factory + `NodeObj` width + `bus_write_gains` emitter

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`NodeObj` width field, `return_node`/`return_node_w`, `node_pan_impl` + wrapper, register)
- Modify: `crates/deluge-wren-core/src/audio.rs` (`bus_write_gains` emitter)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (wren-sys registration)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`pan_` decl + `class Pan`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Kind::Pan` (Task 2), `audio::{alloc_node_id, new_node}`, `arg_input`.
- Produces: Wren `Pan.new(input, position)`; `NodeObj { tag, width, id }`; `audio::bus_write_gains(src, bus, gl, gr)`; `return_node_w(vm, id, width)`. Task 5's width-aware routing consumes `NodeObj.width` and `bus_write_gains`.

- [ ] **Step 1: Write the failing test**

Add to `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn pan_new_emits_kind_pan_node() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var p = Pan.new(Osc.saw(110), -0.5)");
    assert!(
        cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Pan, .. })),
        "expected a NewNode(Pan): {cmds:?}"
    );
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings pan_new`
Expected: FAIL — Wren `Pan` undefined (script interpret error).

- [ ] **Step 3: Add the `width` field to `NodeObj` + width-aware return helper**

In `crates/deluge-wren-core/src/bindings_audio.rs`, change `NodeObj`:

```rust
#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct NodeObj {
    pub tag: u8,
    pub width: u8, // 1 = mono, 2 = stereo (port0=L, port1=R). Read by width-aware routing.
    pub id: u16,
}
```

Change `return_node` to a width-taking helper with a mono default (both used by existing factories, which stay width-1):

```rust
unsafe fn return_node_w<S: SlotApi>(vm: &S, id: u16, width: u8) {
    unsafe { vm.new_foreign_in(0, NodeObj { tag: TAG_NODE, width, id }) };
}
unsafe fn return_node<S: SlotApi>(vm: &S, id: u16) {
    unsafe { return_node_w(vm, id, 1) };
}
```

(All existing `return_node(vm, id)` call sites keep compiling — they now produce width-1 nodes. `self_id` reads `.id` and is unaffected.)

- [ ] **Step 4: Add the `Pan` factory**

In `crates/deluge-wren-core/src/bindings_audio.rs`, add (near the other factories):

```rust
/// `Node.pan_(input, position)` — a constant-power mono→stereo pan node
/// (`Kind::Pan`, width-2: port0=L, port1=R). `position` is port 1 (∈[-1,1],
/// modulatable). Returns a WIDTH-2 node so width-aware routing sends its two
/// ports to L/R (see `write_source_to_bus`, Task 5).
pub(crate) fn node_pan_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let position = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Pan, [input, position, Input::Const(0.0)]);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_pan(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_pan_impl(&vm);
}
```

Register in `register_audio` (with the other factories):

```rust
    method("main", "Node", true, "pan_(_,_)", node_pan_impl::<S>);
```

- [ ] **Step 5: Add the `audio::bus_write_gains` emitter**

In `crates/deluge-wren-core/src/audio.rs`, add after `bus_write`:

```rust
/// Emit a per-side gained bus write (`gl` → L, `gr` → R). Used by width-aware
/// routing to send a stereo (width-2) source's two ports to L and R.
pub fn bus_write_gains(src: Input, bus: u16, gl: f32, gr: f32) {
    if bus == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::BusWriteGains { src, bus: BusId(bus), gl, gr });
}
```

- [ ] **Step 6: Register in the wren-sys table + prelude**

In `crates/deluge-wren-core/src/bindings.rs`, add near the other audio static methods:

```rust
    static_method("Node", "pan_(_,_)", bindings_audio::node_pan),
```

In `crates/deluge-wren-core/wren/prelude.wren`, add to `foreign class Node` (near the other factories):

```wren
  foreign static pan_(input, position)
```

And add the sugar class (e.g. after `class Resonator`):

```wren
// Constant-power stereo pan: a mono input placed in the stereo field.
// `position` -1 = hard left, 0 = center (-3 dB), +1 = hard right; it is a
// port, so it can be modulated (auto-pan):
//   Out.patch(Pan.new(Osc.saw(110), -0.3))
//   var p = Pan.new(pad, 0)
//   p.freq = ...            // (position is arg 2 at construction; patch via a node)
// Pan is a stereo (width-2) node: Out.patch / a bus routes its L/R to the
// stereo output.
class Pan {
  static new(input, position) { Node.pan_(input, position) }
}
```

- [ ] **Step 7: Run the new test + full suite**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings pan_new`
Expected: PASS.

Then the full suite (no regression from the `NodeObj` layout change):

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core`
Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): Pan.new factory + NodeObj width + bus_write_gains emitter"
```

---

### Task 5: Width-aware `Out.patch` / `Bus.write` (stereo routing)

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`write_source_to_bus` helper; rewire `bus_write_impl` and `node_patch_impl`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `NodeObj.width` (Task 4), `audio::{bus_write, bus_write_gains, set_root, MASTER_BUS}`, `arg_input`, `TAG_NODE`, `WrenType`.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn patch_stereo_node_emits_two_side_writes() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    // Out.patch of a width-2 Pan → two BusWriteGains: (1,0) and (0,1) to master.
    let cmds = run_and_capture_cmds("Out.patch(Pan.new(Osc.saw(110), 0.0))");
    let gains: std::vec::Vec<(f32, f32)> = cmds
        .iter()
        .filter_map(|c| match c {
            Cmd::BusWriteGains { gl, gr, .. } => Some((*gl, *gr)),
            _ => None,
        })
        .collect();
    assert!(gains.contains(&(1.0, 0.0)), "missing L-side write: {gains:?}");
    assert!(gains.contains(&(0.0, 1.0)), "missing R-side write: {gains:?}");
}

#[test]
fn patch_mono_node_still_emits_single_center_write() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    // A mono node patched → exactly one plain center BusWrite, no gained writes.
    let cmds = run_and_capture_cmds("Out.patch(Osc.saw(110))");
    let center = cmds.iter().filter(|c| matches!(c, Cmd::BusWrite { .. })).count();
    let gained = cmds.iter().filter(|c| matches!(c, Cmd::BusWriteGains { .. })).count();
    assert_eq!(center, 1, "mono patch should emit one center write: {cmds:?}");
    assert_eq!(gained, 0, "mono patch should emit no gained writes: {cmds:?}");
}

#[test]
fn stereo_pan_renders_distinct_l_and_r() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    // Hard-left pan → L carries signal, R ≈ silent.
    run_and_render("Out.patch(Pan.new(Osc.saw(110), -1.0))", &mut out);
    let suml: f32 = out.iter().map(|f| f.l.abs()).sum();
    let sumr: f32 = out.iter().map(|f| f.r.abs()).sum();
    assert!(suml > 0.01, "L should carry the hard-left pan: {suml}");
    assert!(sumr < 1e-3, "R should be ~silent at hard-left: {sumr}");
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite()));
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings patch_stereo stereo_pan`
Expected: FAIL — `patch_stereo_node_emits_two_side_writes` and `stereo_pan_renders_distinct_l_and_r` fail (a stereo node currently routes as a single center write, so R is not silent / no gained writes emitted). `patch_mono_node_still_emits_single_center_write` may already pass.

- [ ] **Step 3: Add the width-aware routing helper**

In `crates/deluge-wren-core/src/bindings_audio.rs`, add:

```rust
/// Route a source argument (at `slot`) to `bus`, honoring stereo width. A
/// width-2 `Node` (e.g. `Pan`, `Chorus`) emits TWO per-side writes — port0→L
/// `(1,0)`, port1→R `(0,1)`. Everything else (mono `Node`, `Port`, number,
/// `Bus`) emits one center write `(1,1)` — unchanged behavior.
fn write_source_to_bus<S: SlotApi>(vm: &S, slot: i32, bus: u16) {
    if vm.slot_type(slot) == WrenType::Foreign {
        // SAFETY: reads the leading tag byte, then (for TAG_NODE) the 4-byte
        // NodeObj — same access discipline as `arg_input`.
        let tag = unsafe { *vm.foreign_mut::<u8>(slot) };
        if tag == TAG_NODE {
            let n = unsafe { vm.foreign_mut::<NodeObj>(slot) };
            if n.width == 2 {
                let id = n.id;
                audio::bus_write_gains(Input::Node { node: NodeId(id), port: 0 }, bus, 1.0, 0.0);
                audio::bus_write_gains(Input::Node { node: NodeId(id), port: 1 }, bus, 0.0, 1.0);
                return;
            }
        }
    }
    audio::bus_write(arg_input(vm, slot), bus); // mono center (1,1)
}
```

- [ ] **Step 4: Rewire `bus_write_impl` and `node_patch_impl`**

In `crates/deluge-wren-core/src/bindings_audio.rs`, change `bus_write_impl` to use the helper:

```rust
pub(crate) fn bus_write_impl<S: SlotApi>(vm: &S) {
    let id = unsafe { vm.foreign_mut::<BusObj>(0) }.id;
    write_source_to_bus(vm, 1, id);
}
```

And change the non-Bus branch of `node_patch_impl`:

```rust
pub(crate) fn node_patch_impl<S: SlotApi>(vm: &S) {
    // `Out.patch(arg)` → if `arg` is a Bus, set it as root directly; otherwise
    // route the source (stereo-aware) into the master bus and set it as root.
    let tag = unsafe { *vm.foreign_mut::<u8>(1) };
    if tag == TAG_BUS {
        let bus = unsafe { vm.foreign_mut::<BusObj>(1) }.id;
        audio::set_root(bus);
    } else {
        write_source_to_bus(vm, 1, audio::MASTER_BUS);
        audio::set_root(audio::MASTER_BUS);
    }
}
```

(Note: reading the `tag` byte when the arg is a *number* is a non-issue — `node_patch_impl` is only ever called with a foreign or number arg; a number `slot_type` is handled inside `write_source_to_bus`'s `WrenType::Foreign` guard, and the `TAG_BUS` check above only matters for foreigns. If `arg` is a number, `*foreign_mut::<u8>(1)` is not read as a bus because the fall-through calls `write_source_to_bus`, which re-checks `slot_type`. Keep the existing pattern: the pre-existing code already reads the tag byte unconditionally here, so this is unchanged behavior.)

- [ ] **Step 5: Run the new tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings patch_stereo patch_mono stereo_pan`
Expected: PASS (all three).

- [ ] **Step 6: Full wren-core suite (no regression)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core`
Expected: PASS — existing `Out.patch`/`Bus.write` tests (mono center) unchanged; the merged Delay/wavetable/Resonator tests still pass.

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): width-aware Out.patch/Bus.write stereo routing"
```

---

## Scope notes (deliberate deferrals)

- **Chorus/Flanger** (next sub-project, Ef-2): the first *effect* mono→stereo nodes, built on this infra (a `ModDelay` kernel with an internal LFO modulating `read_hermite`, phase-spread voices for chorus, feedback for flanger; width-2 L/R output routed via `write_source_to_bus`).
- **Stereo→stereo effect chaining** (2-in/2-out): a per-effect concern when a stereo effect needs a stereo input; expressible now via explicit `.out(0)`/`.out(1)` ports. Not built here.
- **Direct pan-on-write** (`gl`/`gr` from a pan value at the write site, no `Pan` node): the `bus_write_gains` generality already allows it; add a Wren `write(src, pan:)` sugar only if a use case appears.

## Post-implementation

After all five tasks pass, use **superpowers:finishing-a-development-branch** to verify the suites and merge.
