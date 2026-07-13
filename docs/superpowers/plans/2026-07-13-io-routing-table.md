# Persistent Routing Table — Write Invalidation (IO-2a) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Invalidate a node's bus writes when it is freed (keyed by source node), and reuse freed write slots, so a reused node id inherits no stale routing.

**Architecture:** Engine-internal changes to `deluge-audio-graph/src/engine.rs`: `bus_write_gains` reuses `None` holes; `Cmd::Free{node}` nulls that node's writes; `Cmd::Reset` clears the array. No new `Cmd`, no binding/Wren change.

**Tech Stack:** Rust `no_std` (`deluge-audio-graph`).

## Global Constraints

- `no_std`, no heap, no panic. 32-bit-`usize` safe. MIT/Apache-2.0.
- **Fresh-engine routing MUST be byte-identical** (hole-reuse must not perturb append order when there are no holes).
- Engine-internal only — no binding/prelude/Wren change.
- Tests: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph` (+ `--features deluge-dsp-kernels/simd`). `armv7a … can't find crate for test` LSP output is noise.

---

### Task 1: Write invalidation + slot-reuse

**Files:**
- Modify: `crates/deluge-audio-graph/src/engine.rs` (`bus_write_gains`; `Cmd::Free` arm; `Cmd::Reset` arm)
- Modify/Test: `crates/deluge-audio-graph/src/cmd.rs` (correct `free_then_reuse_keeps_eval_order_sound`; new tests)
- Test: inline `#[cfg(test)]` in `engine.rs` (slot-reuse) + `cmd.rs` (reuse semantics)

**Interfaces:**
- Consumes: existing `writes: [Option<(Input, BusId, f32, f32)>; NODES]`, `writes_len`, `Input::Node`.
- Produces: `Cmd::Free` now invalidates the freed node's writes; `bus_write_gains` reuses holes.

- [ ] **Step 1: Hole-reuse in `bus_write_gains`.** Replace the append-only body:

```rust
    pub fn bus_write_gains(&mut self, src: Input, bus: BusId, gl: f32, gr: f32) {
        // Reuse a freed (`None`) slot first so free/patch cycles don't leak
        // slots; otherwise append. No holes exist without a prior `Free`, so a
        // fresh engine appends in the same order as before (byte-identical).
        for w in 0..self.writes_len {
            if self.writes[w].is_none() {
                self.writes[w] = Some((src, bus, gl, gr));
                return;
            }
        }
        if self.writes_len < self.writes.len() {
            self.writes[self.writes_len] = Some((src, bus, gl, gr));
            self.writes_len += 1;
        }
    }
```

- [ ] **Step 2: Invalidate on `Cmd::Free`.** In the `Cmd::Free { node } => { ... }` arm (after the existing pool-free / `stream_state` clear / `arena.free`), add:

```rust
                // Invalidate this node's bus writes so a reused id inherits no
                // stale routing (IO-2a). Const/Bus-sourced writes are untouched.
                for w in self.writes.iter_mut() {
                    if let Some((Input::Node { node: n, .. }, ..)) = w {
                        if *n == node {
                            *w = None;
                        }
                    }
                }
```

- [ ] **Step 3: Clear the array on `Cmd::Reset`.** In the `Cmd::Reset` arm, alongside `self.writes_len = 0;`:

```rust
                self.writes = [None; NODES];
                self.writes_len = 0;
```

- [ ] **Step 4: Add the slot-reuse engine test.** In `engine.rs` tests (`type E = Engine<16, 8, 8, 4, 45056, 2048>;`):

```rust
    #[test]
    fn bus_write_reuses_freed_slot() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.25);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0)); // writes_len -> 1
        e.apply(Cmd::Free { node: NodeId(0) }); // nulls that write (hole at 0)
        // A new write should reuse the hole, not grow writes_len.
        e.create(NodeId(1), Kind::Add);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Const(0.4);
        *e.node_input_mut(NodeId(1), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(1), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.4).abs() < 1e-6, "only node1's write is live: {}", out[0].l);
    }
```

(If `Cmd` isn't already imported in the `engine.rs` test module, add `use crate::Cmd;` — it is used by the master-chain tests, so it should be present.)

- [ ] **Step 5: Add the no-stale-rebind test + correct the existing one.** In `crates/deluge-audio-graph/src/cmd.rs` tests:

Add:

```rust
    #[test]
    fn free_then_reuse_no_stale_rebind() {
        // Free a routed node, recreate the id WITHOUT re-adding a write: the
        // recreated node inherits no routing (its predecessor's write was
        // invalidated on Free), so its bus is silent.
        let mut e = E::new(16.0);
        saw_patch(&mut e); // node0 -> bus0, root = bus0
        e.apply(Cmd::Free { node: NodeId(0) });
        e.apply(Cmd::NewNode {
            node: NodeId(0),
            kind: Kind::Add,
            args: [Input::Const(0.25), Input::Const(0.0), Input::Const(0.0)],
        });
        e.apply(Cmd::SetRoot { bus: BusId(0) });
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!(out[0].l.abs() < 1e-6, "recreated node inherits no write: {}", out[0].l);
    }
```

And CORRECT the existing `free_then_reuse_keeps_eval_order_sound` (lines ~76-99): after IO-2a, `Free` invalidates `saw_patch`'s write, so only the re-added `BusWrite` remains → the bus sums the node's output ONCE. Change the comment + the assertion:

```rust
        // IO-2a: `Cmd::Free` invalidates `saw_patch`'s pre-free write (keyed by
        // source node), so it no longer re-binds to the recreated `NodeId(0)`.
        // Only the re-added `BusWrite` is live, so bus0 = 0.25 (single source).
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.25).abs() < 1e-6);
        assert!(out[0].l.is_finite() && out[0].l.abs() <= 1.0);
```

- [ ] **Step 6: Run the targeted tests.**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph free_then_reuse` and `... bus_write_reuses_freed_slot`.
Expected: `free_then_reuse_no_stale_rebind`, `free_then_reuse_keeps_eval_order_sound` (now 0.25), `bus_write_reuses_freed_slot` all PASS.

- [ ] **Step 7: Full graph crate, both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features deluge-dsp-kernels/simd
```
Expected: green both configs. In particular the existing routing tests (`two_nodes_sum_into_master_bus`, per-side-gains, etc.) are unchanged — fresh-engine routing is byte-identical.

- [ ] **Step 8: Confirm dependent crates still build.**

Run: `cargo build --target x86_64-unknown-linux-gnu -p deluge-wren-core` and `cargo build -p wren-firmware --target armv7a-none-eabihf`. Expected: clean (no signature/API change).

- [ ] **Step 9: Commit.**

```bash
git add crates/deluge-audio-graph/src/engine.rs crates/deluge-audio-graph/src/cmd.rs
git commit -m "feat(graph): invalidate a node's bus writes on Free + reuse freed write slots (IO-2a)"
```

---

## Self-Review

**Spec coverage:** hole-reuse (`bus_write_gains`) → Step 1. Free-invalidation → Step 2. Reset array-clear → Step 3. Tests (slot-reuse, no-stale-rebind, corrected free_then_reuse, byte-identical via existing tests) → Steps 4-5, 7. All spec sections covered.

**Placeholder scan:** all steps carry exact code; the `use crate::Cmd;` note is a conditional confirmation, not a gap.

**Type consistency:** the `Input::Node { node: n, .. }` destructure (Step 2) matches the `Input` enum's `Node { node, port }` variant. `writes` is `[Option<(Input, BusId, f32, f32)>; NODES]` throughout.
