# Bus→Bus Routing (IO-2c) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add stereo-preserving bus→bus sends (fold a source bus into a target at a gain), settable from Wren via `Bus.send(dst, level)`, ordered descending-`from` (rule: `from > to`, master=0 sink).

**Architecture:** Engine `bus_sends: [Option<(BusId, BusId, f32)>; NODES]` applied after the node-write loop, before the per-bus gain (pre-fader), in descending source-id order; a `Cmd::BusSend` + a Wren `Bus.send` instance method.

**Tech Stack:** Rust `no_std` (`deluge-audio-graph`, `deluge-wren-core`); Wren via `wren-sys`.

## Global Constraints

- `no_std`, no heap, no panic (bounds/self-send guards, `checked_tagged_foreign` for `dst`, `NULL_ID` facade guard). 32-bit-`usize` safe. MIT/Apache-2.0.
- Sends flow high-id → low-id (`from > to`; master=0 sink). Stereo-preserving (L→L, R→R), pre-fader.
- **No-sends path byte-identical** (seam is a no-op when `bus_sends_len == 0`).
- Wren `dst` arg via `checked_tagged_foreign::<BusObj>` guard; `get_f` for level is safe.
- Tests per-crate, BOTH configs (`--features deluge-dsp-kernels/simd` for graph/wren-core), target `x86_64-unknown-linux-gnu`. `armv7a … can't find crate for test` LSP output is noise.

---

### Task 1: Engine bus→bus sends

**Files:**
- Modify: `crates/deluge-audio-graph/src/cmd.rs` (add `BusSend` after `BusGain`)
- Modify: `crates/deluge-audio-graph/src/engine.rs` (fields; `new` init; `Reset`; `Cmd::BusSend` apply arm; `bus_send` helper; send seam)
- Test: inline `#[cfg(test)]` in `engine.rs`

**Interfaces:**
- Produces: `Cmd::BusSend { from: BusId, to: BusId, gain: f32 }`; `Engine::bus_send(&mut self, BusId, BusId, f32)`; fields `bus_sends`/`bus_sends_len`.

- [ ] **Step 1: Add the `Cmd` variant.** In `cmd.rs`, after `BusGain { bus: BusId, gain: f32 },`:

```rust
    BusGain { bus: BusId, gain: f32 },
    /// Fold a source bus into a target bus at `gain` (stereo-preserving). Applied
    /// after the node writes, before per-bus gain, descending `from` (rule from > to).
    BusSend { from: BusId, to: BusId, gain: f32 },
```

- [ ] **Step 2: Add the fields.** In `engine.rs`, after the `bus_gain` field:

```rust
    pub(crate) bus_gain: [f32; BUSES],
    // Bus→bus sends: (from, to, gain), applied after the node-write loop,
    // descending `from` id (rule: from > to). Stereo-preserving, pre-fader.
    bus_sends: [Option<(BusId, BusId, f32)>; NODES],
    bus_sends_len: usize,
```

- [ ] **Step 3: Init in `new`.** Next to `bus_gain: [1.0; BUSES],`:

```rust
            bus_gain: [1.0; BUSES],
            bus_sends: [None; NODES],
            bus_sends_len: 0,
```

- [ ] **Step 4: Clear on Reset.** In the `Cmd::Reset` arm, after `self.bus_gain = [1.0; BUSES];`:

```rust
                self.bus_gain = [1.0; BUSES];
                self.bus_sends = [None; NODES];
                self.bus_sends_len = 0;
```

- [ ] **Step 5: Add the apply arm + `bus_send` helper.** In `Engine::apply`, after the `Cmd::BusGain { bus, gain } => self.set_bus_gain(bus, gain),` arm:

```rust
            Cmd::BusGain { bus, gain } => self.set_bus_gain(bus, gain),
            Cmd::BusSend { from, to, gain } => self.bus_send(from, to, gain),
```

And add the helper near `set_bus_gain` (hole-reuse, mirroring `bus_write_gains`):

```rust
    /// Record a bus→bus send (reuses a freed slot first, else appends).
    pub fn bus_send(&mut self, from: BusId, to: BusId, gain: f32) {
        for s in 0..self.bus_sends_len {
            if self.bus_sends[s].is_none() {
                self.bus_sends[s] = Some((from, to, gain));
                return;
            }
        }
        if self.bus_sends_len < self.bus_sends.len() {
            self.bus_sends[self.bus_sends_len] = Some((from, to, gain));
            self.bus_sends_len += 1;
        }
    }
```

- [ ] **Step 6: Add the send seam.** In `render`, insert between the end of the node-write loop and the `// Per-bus gain ...` comment:

```rust
                    self.bus_l[b][i] += v * gl;
                    self.bus_r[b][i] += v * gr;
                }
            }
        }
        // Bus→bus sends (stereo-preserving, pre-fader): fold source buses into
        // targets in descending `from` id (rule: from > to, master=0 = sink) so a
        // source is fully filled before it feeds a lower bus. No-op when none.
        for from in (0..BUSES).rev() {
            for s in 0..self.bus_sends_len {
                if let Some((f, t, g)) = self.bus_sends[s] {
                    let (fi, ti) = (f.0 as usize, t.0 as usize);
                    if fi == from && fi < BUSES && ti < BUSES && fi != ti {
                        for i in 0..BLOCK {
                            self.bus_l[ti][i] += self.bus_l[fi][i] * g;
                            self.bus_r[ti][i] += self.bus_r[fi][i] * g;
                        }
                    }
                }
            }
        }
        // Per-bus gain (default 1.0 = no-op, byte-identical): fader before the master chain.
```

- [ ] **Step 7: Write the failing engine tests.** In `engine.rs` tests (`type E = Engine<16, 8, 8, 4, 45056, 2048>;` — BUSES=4). To feed an asymmetric L≠R source, `bus_write_gains` a node into busA with different `gl`/`gr`:

```rust
    #[test]
    fn bus_send_folds_stereo_into_target() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        // node0 → busA(1) with L=0.8, R=0.2 (asymmetric, proves L→L/R→R).
        e.bus_write_gains(Input::Node { node: NodeId(0), port: 0 }, BusId(1), 1.0, 0.25);
        e.apply(Cmd::BusSend { from: BusId(1), to: BusId(0), gain: 0.5 });
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        // busA L=0.8, R=0.2 → sent ×0.5 into master → L=0.4, R=0.1.
        assert!((out[0].l - 0.4).abs() < 1e-6, "L: {}", out[0].l);
        assert!((out[0].r - 0.1).abs() < 1e-6, "R: {}", out[0].r);
    }

    #[test]
    fn bus_send_chain_routes_same_block() {
        // bus2 → bus1 → bus0 (master), each from > to, descending-from ordering.
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.4);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(2)); // node → bus2
        e.apply(Cmd::BusSend { from: BusId(2), to: BusId(1), gain: 1.0 });
        e.apply(Cmd::BusSend { from: BusId(1), to: BusId(0), gain: 1.0 });
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.4).abs() < 1e-6, "chain to master: {}", out[0].l);
    }

    #[test]
    fn bus_send_none_is_byte_identical() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.7);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.7).abs() < 1e-6); // no sends → unchanged
    }

    #[test]
    fn bus_send_self_and_oob_are_noop() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.6);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::BusSend { from: BusId(0), to: BusId(0), gain: 2.0 }); // self → no-op
        e.apply(Cmd::BusSend { from: BusId(9), to: BusId(0), gain: 2.0 }); // oob → no-op
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil); // must not panic
        assert!((out[0].l - 0.6).abs() < 1e-6, "self/oob send no-op: {}", out[0].l);
    }
```

- [ ] **Step 8: Run engine tests, then full graph crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph bus_send
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features deluge-dsp-kernels/simd
```
Expected: 4 new tests PASS; full crate green both configs (no-sends path byte-identical → existing tests unchanged).

- [ ] **Step 9: Confirm dependent crates build.**

Run: `cargo build --target x86_64-unknown-linux-gnu -p deluge-wren-core` and `cargo build -p wren-firmware --target armv7a-none-eabihf`. Expected: clean.

- [ ] **Step 10: Commit.**

```bash
git add crates/deluge-audio-graph/src/cmd.rs crates/deluge-audio-graph/src/engine.rs
git commit -m "feat(graph): Cmd::BusSend + stereo bus→bus sends, descending-from ordering (IO-2c)"
```

---

### Task 2: Wren `Bus.send(dst, level)` + e2e

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (add `bus_send` near `set_bus_gain`)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`bus_send_impl` + shim near `bus_set_gain_impl`; `register_audio` Bus entry)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (`METHODS` Bus entry)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`foreign send_` + `send` wrapper in `class Bus`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Cmd::BusSend { from, to, gain }` (Task 1); `BusObj`, `TAG_BUS`, `checked_tagged_foreign`.
- Produces: Wren `Bus` instance method `send(dst, level)`.

- [ ] **Step 1: Write the failing Cmd-capture tests.** In `crates/deluge-wren-core/tests/audio_bindings.rs`. `Bus.new()` twice → ids 1 and 2:

```rust
#[test]
fn bus_send_setter_emits_bus_send() {
    let cmds = run_and_capture_cmds("var a = Bus.new()\nvar m = Bus.new()\na.send(m, 0.3)");
    assert!(
        cmds.iter().any(|c| matches!(c, Cmd::BusSend { gain, .. } if (*gain - 0.3).abs() < 1e-6)),
        "expected a BusSend{{gain:0.3}}, got {:?}", cmds);
}

#[test]
fn bus_send_non_bus_dst_is_noop() {
    // dst is a number, not a Bus → checked_tagged_foreign degrades to no-op.
    let cmds = run_and_capture_cmds("var a = Bus.new()\na.send(5, 0.3)");
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::BusSend { .. })),
        "non-Bus dst must not emit BusSend, got {:?}", cmds);
}
```

- [ ] **Step 2: Run to verify fail.**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support --test audio_bindings bus_send`
Expected: FAIL (Wren `Bus.send` unknown).

- [ ] **Step 3: Add the facade.** In `crates/deluge-wren-core/src/audio.rs`, near `set_bus_gain`:

```rust
pub fn bus_send(from: u16, to: u16, gain: f32) {
    if from == NULL_ID || to == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::BusSend { from: BusId(from), to: BusId(to), gain });
}
```

- [ ] **Step 4: Add the binding impl + shim.** In `crates/deluge-wren-core/src/bindings_audio.rs`, near `bus_set_gain_impl`:

```rust
pub(crate) fn bus_send_impl<S: SlotApi>(vm: &S) {
    let from = unsafe { vm.foreign_mut::<BusObj>(0) }.id; // receiver — guaranteed a Bus
    // `dst` arg is user-supplied → the checked guard (degrade to no-op on non-Bus).
    let to = match checked_tagged_foreign::<BusObj, _>(vm, 1, TAG_BUS) {
        Some(o) => o.id,
        None => return,
    };
    let level = vm.get_f(2) as f32;
    audio::bus_send(from, to, level);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn bus_send(raw: *mut WrenVM) {
    let vm = Vm(raw);
    bus_send_impl(&vm);
}
```

- [ ] **Step 5: Register in `register_audio`.** In `bindings_audio.rs` next to `gain=(_)`:

```rust
    method("main", "Bus", false, "send_(_,_)", bus_send_impl::<S>);
```

- [ ] **Step 6: Register in `METHODS`.** In `crates/deluge-wren-core/src/bindings.rs` next to the `Bus` `gain=(_)` line:

```rust
    method("Bus", "send_(_,_)", bindings_audio::bus_send),
```

- [ ] **Step 7: Add the prelude decl + wrapper.** In `crates/deluge-wren-core/wren/prelude.wren`, inside `foreign class Bus { ... }`, after `foreign gain=(v)`:

```wren
  foreign send_(dst, level)  // native stereo bus→bus send
  send(dst, level) { send_(dst, level) }
```

(implicit-return one-liner — a single-line explicit-`return` does NOT compile in this Wren fork.)

- [ ] **Step 8: Run the Cmd-capture tests, then full crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support --test audio_bindings bus_send
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd
```
Expected: both `bus_send_*` PASS; full crate green both configs.

- [ ] **Step 9: Write the e2e test.** In `tests/audio_bindings.rs`, use `run_and_render`. A source on an aux bus sent to master at 0.5 renders at ~half of the same source patched directly at unity:

```rust
#[test]
fn bus_send_aux_to_master_half_level() {
    // Direct (unity): source on master.
    let mut direct = [StereoFrame::default(); 64];
    run_and_render("var m = Bus.new()\nm.write(Osc.saw(110) * 0.4)\nOut.patch(m)", &mut direct);
    let direct_peak = direct.iter().fold(0.0f32, |a, f| a.max(f.l.abs()));

    // Aux → master at 0.5: source on an aux, aux sent to master at half.
    let mut aux = [StereoFrame::default(); 64];
    run_and_render(
        "var master = Bus.new()\nvar send = Bus.new()\nsend.write(Osc.saw(110) * 0.4)\nsend.send(master, 0.5)\nOut.patch(master)",
        &mut aux,
    );
    let aux_peak = aux.iter().fold(0.0f32, |a, f| a.max(f.l.abs()));

    assert!(direct_peak > 1e-3, "direct not silent: {}", direct_peak);
    assert!((aux_peak - direct_peak * 0.5).abs() < direct_peak * 0.1,
        "aux→master at 0.5 should be ~half: direct={} aux={}", direct_peak, aux_peak);
}
```

Note the id ordering: `master = Bus.new()` gets id 1, `send = Bus.new()` gets id 2, so the send is `from=2 → to=1` (`from > to`, correct). Adjust the exact Wren to the harness if `Out.patch`/`write` construction differs, but keep the direct-vs-aux half comparison as the real assertion. Do NOT weaken to always-true.

- [ ] **Step 10: Run the e2e, then full crate both configs; cross-build; commit.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support --test audio_bindings bus_send_aux
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd
cargo build -p wren-firmware --target armv7a-none-eabihf
```
Expected: PASS; full crate green both configs; firmware clean. Then:

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs \
        crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren \
        crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): Bus.send(dst, level) stereo bus→bus send binding + e2e (IO-2c)"
```

---

## Self-Review

**Spec coverage:** engine `bus_sends` + `Cmd::BusSend` + descending-from seam + Reset → Task 1. Wren `Bus.send` (checked `dst`) both tables + shim + prelude → Task 2. Stereo-preserving (asymmetric L/R test), chain, byte-identical, self/oob no-op → Task 1 tests. e2e half-level → Task 2. All spec sections covered.

**Placeholder scan:** Task 2 Step 9 notes "adjust the exact Wren to the harness" (the direct-vs-aux comparison is the real assertion) — a follow-the-harness instruction, not a gap. All engine/binding steps carry exact code.

**Type consistency:** `Cmd::BusSend { from: BusId, to: BusId, gain: f32 }` (Task 1) emitted identically by `audio::bus_send` (Task 2) and asserted in the Task 2 Cmd-capture test. `bus_send_impl` reads the receiver `foreign_mut::<BusObj>(0).id` (like `bus_write_impl`) and the `dst` via `checked_tagged_foreign::<BusObj, _>(vm, 1, TAG_BUS)`. Instance-method registration uses `method(...)` / `false`, mirroring `write_`/`gain=`.
