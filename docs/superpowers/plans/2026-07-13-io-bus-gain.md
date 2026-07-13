# Per-Bus Gain (IO-2b) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a per-bus mono gain (channel fader), settable from Wren via `Bus.gain=`, applied to each bus before the master chain.

**Architecture:** Engine `bus_gain: [f32; BUSES]` (default 1.0) scaled into the bus rows after the write loop, guarded so the default path is byte-identical; a `Cmd::BusGain` + a Wren `Bus.gain=` instance setter.

**Tech Stack:** Rust `no_std` (`deluge-audio-graph`, `deluge-wren-core`); Wren via `wren-sys`.

## Global Constraints

- `no_std`, no heap, no panic on any input (bounds-guarded apply, `NULL_ID` facade guard). 32-bit-`usize` safe. MIT/Apache-2.0.
- Per-bus MONO gain, default 1.0. **Default path byte-identical** (the `if g != 1.0` guard).
- Wren reads only `get_f` + the receiver `BusObj` (same as `bus_write_impl`).
- Tests per-crate, BOTH configs (`--features deluge-dsp-kernels/simd` for graph/wren-core), target `x86_64-unknown-linux-gnu`. `armv7a … can't find crate for test` LSP output is noise.

---

### Task 1: Engine per-bus gain

**Files:**
- Modify: `crates/deluge-audio-graph/src/cmd.rs` (add `BusGain` variant after `SetRoot`)
- Modify: `crates/deluge-audio-graph/src/engine.rs` (field ~39; `new` init ~81; `Cmd::BusGain` apply arm ~176; `set_bus_gain` helper near `set_root`; `Reset` ~220; gain seam ~437)
- Test: inline `#[cfg(test)]` in `engine.rs`

**Interfaces:**
- Produces: `Cmd::BusGain { bus: BusId, gain: f32 }`; `Engine::set_bus_gain(&mut self, BusId, f32)`; field `bus_gain: [f32; BUSES]` (default/reset 1.0).

- [ ] **Step 1: Add the `Cmd` variant.** In `cmd.rs`, after `SetRoot { bus: BusId },`:

```rust
    SetRoot { bus: BusId },
    /// Set a bus's mono gain (channel fader). Applied to the bus's L/R rows after
    /// the write loop, before the master chain. Default 1.0 (unity).
    BusGain { bus: BusId, gain: f32 },
```

- [ ] **Step 2: Add the field.** In `engine.rs`, after the `bus_r` field:

```rust
    pub(crate) bus_r: [[f32; BLOCK]; BUSES],
    // Per-bus mono gain (channel fader), default 1.0. Applied to each bus's rows
    // after the write loop, before the master chain.
    pub(crate) bus_gain: [f32; BUSES],
```

- [ ] **Step 3: Init in `new`.** In `Engine::new`, next to `bus_r: [[0.0; BLOCK]; BUSES],`:

```rust
            bus_r: [[0.0; BLOCK]; BUSES],
            bus_gain: [1.0; BUSES],
```

- [ ] **Step 4: Reset to unity.** In the `Cmd::Reset` arm, add (note the 1.0, unlike the other reset fields):

```rust
                self.master_eq = None;
                self.bus_gain = [1.0; BUSES];
```

- [ ] **Step 5: Add the apply arm + helper.** In `Engine::apply`, after the `Cmd::SetRoot { bus } => self.set_root(bus),` arm:

```rust
            Cmd::SetRoot { bus } => self.set_root(bus),
            Cmd::BusGain { bus, gain } => self.set_bus_gain(bus, gain),
```

And add the helper near `set_root` (e.g. right after it):

```rust
    /// Set a bus's mono gain (bounds-guarded; out-of-range is a no-op).
    pub fn set_bus_gain(&mut self, bus: BusId, gain: f32) {
        let b = bus.0 as usize;
        if b < BUSES {
            self.bus_gain[b] = gain;
        }
    }
```

- [ ] **Step 6: Add the gain seam.** In `render`, insert between the end of the write-accumulation loop and the `// Master chain (opt-in): ...` comment:

```rust
            }
        }
        // Per-bus gain (default 1.0 = no-op, byte-identical): fader before the master chain.
        for b in 0..BUSES {
            let g = self.bus_gain[b];
            if g != 1.0 {
                for i in 0..BLOCK {
                    self.bus_l[b][i] *= g;
                    self.bus_r[b][i] *= g;
                }
            }
        }
        // Master chain (opt-in): DC-block → limiter, on the root bus before the clamp.
```

- [ ] **Step 7: Write the failing engine tests.** In `engine.rs` tests (`type E = Engine<16, 8, 8, 4, 45056, 2048>;`, `use crate::Cmd;`). Note BUSES=4 in `E`:

```rust
    #[test]
    fn bus_gain_scales_output() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::BusGain { bus: BusId(0), gain: 0.5 });
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.4).abs() < 1e-6, "0.8 * 0.5 = 0.4, got {}", out[0].l);
    }

    #[test]
    fn bus_gain_default_is_byte_identical() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.7);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.7).abs() < 1e-6); // no BusGain → unity, unchanged
    }

    #[test]
    fn bus_gain_reset_restores_unity() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::BusGain { bus: BusId(0), gain: 0.5 });
        e.apply(Cmd::Reset);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.8).abs() < 1e-6, "Reset → unity, got {}", out[0].l);
    }

    #[test]
    fn bus_gain_out_of_range_is_noop() {
        let mut e = E::new(16.0);
        e.apply(Cmd::BusGain { bus: BusId(99), gain: 0.5 }); // BUSES=4 → no-op, no panic
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.6);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.6).abs() < 1e-6); // unaffected
    }
```

- [ ] **Step 8: Run engine tests, then full graph crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph bus_gain
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features deluge-dsp-kernels/simd
```
Expected: 4 new tests PASS; full crate green both configs (existing routing tests unchanged — the default gain path is byte-identical).

- [ ] **Step 9: Confirm dependent crates build.**

Run: `cargo build --target x86_64-unknown-linux-gnu -p deluge-wren-core` and `cargo build -p wren-firmware --target armv7a-none-eabihf`. Expected: clean.

- [ ] **Step 10: Commit.**

```bash
git add crates/deluge-audio-graph/src/cmd.rs crates/deluge-audio-graph/src/engine.rs
git commit -m "feat(graph): Cmd::BusGain + per-bus mono gain applied before the master chain (IO-2b)"
```

---

### Task 2: Wren `Bus.gain=` + e2e

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (add `set_bus_gain` near `set_root`)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`bus_set_gain_impl` + shim near `bus_write_impl`; `register_audio` Bus entry)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (`METHODS` Bus entry)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`foreign gain=(v)` in `class Bus`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Cmd::BusGain { bus, gain }` (Task 1); `BusObj` (tag+id), `audio::alloc_bus_id`.
- Produces: Wren `Bus` instance setter `gain=(v)`.

- [ ] **Step 1: Write the failing Cmd-capture test.** In `crates/deluge-wren-core/tests/audio_bindings.rs`. `Bus.new()` allocates the first non-master id (1), so the emitted bus is `BusId(1)`:

```rust
#[test]
fn bus_gain_setter_emits_bus_gain() {
    let cmds = run_and_capture_cmds("var m = Bus.new()\nm.gain = 0.5");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::BusGain { gain, .. } if (*gain - 0.5).abs() < 1e-6)),
        "expected a BusGain{{gain:0.5}}, got {:?}", cmds);
}
```

(A `matches!` check is used rather than an exact-vec assert because `Bus.new()` may or may not emit its own Cmd; the assertion pins the `BusGain` value.)

- [ ] **Step 2: Run to verify fail.**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support --test audio_bindings bus_gain_setter`
Expected: FAIL (Wren `Bus.gain=` unknown).

- [ ] **Step 3: Add the facade.** In `crates/deluge-wren-core/src/audio.rs`, near `set_root`:

```rust
pub fn set_bus_gain(bus: u16, gain: f32) {
    if bus == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::BusGain { bus: BusId(bus), gain });
}
```

- [ ] **Step 4: Add the binding impl + shim.** In `crates/deluge-wren-core/src/bindings_audio.rs`, near `bus_write_impl`:

```rust
pub(crate) fn bus_set_gain_impl<S: SlotApi>(vm: &S) {
    let id = unsafe { vm.foreign_mut::<BusObj>(0) }.id;
    let g = vm.get_f(1) as f32;
    audio::set_bus_gain(id, g);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn bus_set_gain(raw: *mut WrenVM) {
    let vm = Vm(raw);
    bus_set_gain_impl(&vm);
}
```

- [ ] **Step 5: Register in `register_audio` (instance method → `false`).** In `bindings_audio.rs` next to `write_(_)`:

```rust
    method("main", "Bus", false, "gain=(_)", bus_set_gain_impl::<S>);
```

- [ ] **Step 6: Register in `METHODS`.** In `crates/deluge-wren-core/src/bindings.rs` next to the `Bus` `write_(_)` line:

```rust
    method("Bus", "gain=(_)", bindings_audio::bus_set_gain),
```

- [ ] **Step 7: Add the prelude decl.** In `crates/deluge-wren-core/wren/prelude.wren`, inside `foreign class Bus { ... }`, after `foreign write_(src)`:

```wren
  foreign gain=(v)           // per-bus mono gain (channel fader)
```

- [ ] **Step 8: Run the Cmd-capture test, then full crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support --test audio_bindings bus_gain_setter
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd
```
Expected: PASS; full crate green both configs.

- [ ] **Step 9: Write the e2e test.** In `tests/audio_bindings.rs`, use `run_and_render`. A bus at gain 0.5 renders at half the unity level. Route a source through an explicit bus, patch it, set gain:

```rust
#[test]
fn bus_gain_halves_render() {
    // Unity: source through a bus, patched to output.
    let mut base = [StereoFrame::default(); 64];
    run_and_render("var m = Bus.new()\nm.write(Osc.saw(110) * 0.4)\nOut.patch(m)", &mut base);
    let base_peak = base.iter().fold(0.0f32, |a, f| a.max(f.l.abs()));

    // Half gain on the same bus.
    let mut half = [StereoFrame::default(); 64];
    run_and_render("var m = Bus.new()\nm.write(Osc.saw(110) * 0.4)\nOut.patch(m)\nm.gain = 0.5", &mut half);
    let half_peak = half.iter().fold(0.0f32, |a, f| a.max(f.l.abs()));

    assert!(base_peak > 1e-3, "baseline not silent: {}", base_peak);
    assert!((half_peak - base_peak * 0.5).abs() < base_peak * 0.1,
        "half gain should ~halve the peak: base={} half={}", base_peak, half_peak);
}
```

(If `Bus.write(Osc.saw(...) * 0.4)` routing needs the source patched differently, mirror the existing `bus_write_and_patch` Cmd-capture test's construction — but for the render harness, `Out.patch(m)` sets `m` as root and `m.write(...)` feeds it. Adjust the exact Wren to whatever the harness renders; keep the base-vs-half peak comparison as the real assertion. Do NOT weaken to always-true.)

- [ ] **Step 10: Run the e2e, then full crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support --test audio_bindings bus_gain_halves
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd
```
Expected: PASS; full crate green both configs.

- [ ] **Step 11: Cross-build + commit.**

Run: `cargo build -p wren-firmware --target armv7a-none-eabihf` (clean), then:

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs \
        crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren \
        crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): Bus.gain= per-bus gain binding + e2e (IO-2b)"
```

---

## Self-Review

**Spec coverage:** engine `bus_gain` field + `Cmd::BusGain` + seam + Reset → Task 1. Wren `Bus.gain=` both tables + shim + prelude → Task 2. Byte-identical default (guard) → Task 1 test. e2e half-gain → Task 2. All spec sections covered.

**Placeholder scan:** Task 2 Step 9 notes "adjust the exact Wren to the harness" (the base-vs-half comparison is the real assertion) — a follow-the-harness instruction, not a gap. All engine/binding steps carry exact code.

**Type consistency:** `Cmd::BusGain { bus: BusId, gain: f32 }` (Task 1) emitted identically by `audio::set_bus_gain` (Task 2) and asserted in the Task 2 Cmd-capture test. `bus_set_gain_impl` reads `foreign_mut::<BusObj>(0).id` exactly as `bus_write_impl` does. The instance-method registration uses `method(...)` (not `static_method`) with `false` in `register_audio`, mirroring `write_(_)`.
