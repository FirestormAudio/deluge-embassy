# Bus-Fed Effects — One-Block-Delayed Bus Reads (IO-2e) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `Input::Bus` node-reads see the previous block's bus content (one-block-delayed) instead of `0.0`, so a bus can feed an effect node — by reordering `render` (zero buses after node eval) and zeroing buses on Reset.

**Architecture:** Swap the bus-zeroing to after `render_block` so node evaluation reads the previous block's end-of-block bus rows; add `bus_l`/`bus_r` zeroing to `Cmd::Reset`.

**Tech Stack:** Rust `no_std` (`deluge-audio-graph`).

## Global Constraints

- `no_std`, no heap, no panic. 32-bit-`usize` safe. MIT/Apache-2.0. Engine-internal only.
- `Input::Bus` node-reads are one block (BLOCK samples) late.
- **Existing non-bus-reading graphs MUST render byte-identically.**
- Tests: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph` (+ `--features deluge-dsp-kernels/simd`). `armv7a … can't find crate for test` LSP output is noise.

---

### Task 1: Render-order swap + Reset bus-zero

**Files:**
- Modify: `crates/deluge-audio-graph/src/engine.rs` (the `render` zero/eval order; the `Cmd::Reset` arm)
- Test: inline `#[cfg(test)]` in `engine.rs`

**Interfaces:**
- Consumes: existing `render_block`, `bus_l`/`bus_r`, `Input::Bus` resolution (unchanged).
- Produces: `Input::Bus` node-reads now return the previous block's bus content.

- [ ] **Step 1: Swap the zero + render_block order.** In `render` (currently `// Zero buses.` block precedes `// Evaluate nodes. self.render_block();`), reorder so `render_block` runs first, then the buses are zeroed:

Replace:

```rust
        // Zero buses.
        for b in 0..BUSES {
            self.bus_l[b] = [0.0; BLOCK];
            self.bus_r[b] = [0.0; BLOCK];
        }
        // Evaluate nodes.
        self.render_block();
```

with:

```rust
        // Evaluate nodes FIRST, so an `Input::Bus` node-read sees the PREVIOUS
        // block's bus content (one-block-delayed) rather than a freshly-zeroed
        // bus — this is what lets a bus feed an effect node (aux returns).
        self.render_block();
        // NOW zero the buses and refill them from this block's node outputs.
        for b in 0..BUSES {
            self.bus_l[b] = [0.0; BLOCK];
            self.bus_r[b] = [0.0; BLOCK];
        }
```

(The write loop, sends, gain, master chain, and root-copy that follow are UNCHANGED.)

- [ ] **Step 2: Zero buses on Reset.** In the `Cmd::Reset` arm, add bus-row zeroing (buses now carry state across renders, so Reset must clear them). After the existing `self.bus_sends = [None; NODES]; self.bus_sends_len = 0;`:

```rust
                self.bus_sends = [None; NODES];
                self.bus_sends_len = 0;
                self.bus_l = [[0.0; BLOCK]; BUSES];
                self.bus_r = [[0.0; BLOCK]; BUSES];
```

- [ ] **Step 3: Write the failing engine tests.** In `engine.rs` tests (`type E = Engine<16, 8, 8, 4, 45056, 2048>;`). `Kind::Add` with one const input and one `Input::Bus` input is a bus-reading passthrough (Add(bus, 0) = bus):

```rust
    #[test]
    fn bus_fed_node_reads_previous_block() {
        let mut e = E::new(16.0);
        // node0 = const 0.5, written into bus1.
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(1));
        // node1 = reads bus1 (Add(bus1, 0)), routed to master bus0.
        e.create(NodeId(1), Kind::Add);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Bus(BusId(1));
        *e.node_input_mut(NodeId(1), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(1), port: 0 }, BusId(0));
        e.set_root(BusId(0));

        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        // Block 1: node1 reads bus1 = 0 (initial), so master ~0.
        e.render(&mut out, &sil);
        assert!(out[0].l.abs() < 1e-6, "block1 bus read should be 0: {}", out[0].l);
        // Block 2: node1 reads bus1 = block1's node0 write (0.5) → master = 0.5.
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6, "block2 one-block-late read: {}", out[0].l);
    }

    #[test]
    fn existing_routing_byte_identical_after_reorder() {
        // Mirrors two_nodes_sum_into_master_bus: no Input::Bus reads → unchanged.
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.3);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.create(NodeId(1), Kind::Add);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Const(0.4);
        *e.node_input_mut(NodeId(1), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.bus_write(Input::Node { node: NodeId(1), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.7).abs() < 1e-6); // 0.3 + 0.4, same as before the reorder
    }

    #[test]
    fn reset_zeros_buses() {
        let mut e = E::new(16.0);
        // Drive bus1 with content over a block.
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.9);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(1));
        e.set_root(BusId(1));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil); // bus1 now holds 0.9
        e.apply(Cmd::Reset);
        // Rebuild a bus1-reading graph; first render must read a zeroed bus1.
        e.create(NodeId(1), Kind::Add);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Bus(BusId(1));
        *e.node_input_mut(NodeId(1), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(1), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.render(&mut out, &sil);
        assert!(out[0].l.abs() < 1e-6, "Reset must zero buses; got stale {}", out[0].l);
    }
```

- [ ] **Step 4: Run the new tests, then the full graph crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph bus_fed_node_reads_previous_block
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph existing_routing_byte_identical_after_reorder
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph reset_zeros_buses
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features deluge-dsp-kernels/simd
```
Expected: the 3 new tests PASS; full crate green both configs. In particular the existing render/routing tests (`two_nodes_sum_into_master_bus`, per-side-gains, master-chain, bus-gain, bus-send, etc.) are unchanged — the reorder is output-preserving for non-`Input::Bus` graphs.

- [ ] **Step 5: Confirm dependent crates still build.**

Run: `cargo build --target x86_64-unknown-linux-gnu -p deluge-wren-core` and `cargo build -p wren-firmware --target armv7a-none-eabihf`. Expected: clean (no API change).

- [ ] **Step 6: Commit.**

```bash
git add crates/deluge-audio-graph/src/engine.rs
git commit -m "feat(graph): one-block-delayed Input::Bus reads (eval before zero) + Reset bus-zero (IO-2e)"
```

---

## Self-Review

**Spec coverage:** render-order swap → Step 1. Reset bus-zero → Step 2. Tests (one-block-late read, byte-identical, reset-zeros) → Step 3. All spec sections covered.

**Placeholder scan:** all steps carry exact code; no gaps.

**Type consistency:** the reorder touches only statement order in `render`; `Input::Bus`/`bus_l`/`bus_r`/`render_block` are unchanged. The Reset addition mirrors the existing `[[0.0; BLOCK]; BUSES]` initializer form used in `Engine::new`.
