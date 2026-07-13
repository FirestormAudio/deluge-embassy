# Master DC-Block (IO-3b) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in, per-channel stereo DC-blocker on the render root bus, ordered before the master limiter, configured from Wren via `Out.dcBlock(...)`.

**Architecture:** A new `pub MasterDcBlock` kernel in `filter.rs` wrapping two existing `OnePoleHp` (L/R). The engine holds it as `Option<MasterDcBlock>` (`None` = render byte-unchanged), applies it at the render seam before the limiter, and configures it via a new `Cmd::SetMasterDcBlock`. A Wren `Out.dcBlock(cutoff)` factory emits that Cmd. Mirrors the merged IO-3a limiter machinery.

**Tech Stack:** Rust `no_std` (crates `deluge-dsp-kernels`, `deluge-audio-graph`, `deluge-wren-core`); `libm::tan` (in `OnePoleHp::set_coeff`); Wren via `wren-sys`.

## Global Constraints

- `no_std`, no heap, **no panic on any input** — `cutoff_hz` finite-guarded + clamped to `[0.1, 20_000.0]`; `dt` floored to a small positive.
- 32-bit-`usize` safe (index `0..len` only).
- **Per-channel** (two independent `OnePoleHp`, no linking). Cutoff configurable, default `20.0` Hz.
- **Scalar** — per-sample IIR recursion; no `f32x8`/NEON path. Tests pass identically both configs.
- **Opt-in, additive:** `Option<MasterDcBlock>` `None` default ⇒ render byte-identical when unused. The limiter and the `[-1,1]` clamp are UNCHANGED. `OnePoleHp` visibility UNCHANGED (`MasterDcBlock` lives in `filter.rs`, same module, so it reaches the private `process`).
- Wren reads only `get_f` (safe — no `checked_*`). Prelude bodies: single-line **implicit** return only (a single-line explicit-`return` does NOT compile in this Wren fork).
- Tests per-crate (NEVER `--workspace`), BOTH configs (default + `--features simd` for kernels; `--features deluge-dsp-kernels/simd` for graph/wren-core), target `x86_64-unknown-linux-gnu`. `armv7a … can't find crate for test` LSP output is noise.
- MIT/Apache-2.0.

---

## File Structure

- `crates/deluge-dsp-kernels/src/filter.rs` — NEW `pub struct MasterDcBlock` (Task 1).
- `crates/deluge-audio-graph/src/cmd.rs` — add `Cmd::SetMasterDcBlock` (Task 2).
- `crates/deluge-audio-graph/src/engine.rs` — `master_dcblock` field + init + apply arm + Reset clear + render seam (Task 2).
- `crates/deluge-wren-core/src/audio.rs` — `set_master_dcblock` facade (Task 3).
- `crates/deluge-wren-core/src/bindings_audio.rs` — `node_master_dcblock_impl` + shim + `register_audio` (Task 3).
- `crates/deluge-wren-core/src/bindings.rs` — `METHODS` registration (Task 3).
- `crates/deluge-wren-core/wren/prelude.wren` — `foreign static masterDcBlock_` + `class Out` methods (Task 3).
- `crates/deluge-wren-core/tests/audio_bindings.rs` — Cmd-capture (Task 3) + e2e (Task 4).

---

### Task 1: `MasterDcBlock` kernel

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/filter.rs` (add `MasterDcBlock` after the `OnePoleHp` impl, ~line 281)
- Test: inline `#[cfg(test)]` in `filter.rs`

**Interfaces:**
- Consumes: existing `pub(crate) struct OnePoleHp` (Default, `set_coeff(wc: f64)`, private `process(f32)->f32`) — same module.
- Produces: `pub struct MasterDcBlock` with `new(cutoff_hz: f32, dt: f32)`, `set_cutoff(&mut self, cutoff_hz: f32, dt: f32)`, `process(&mut self, l: &mut [f32], r: &mut [f32])`.

- [ ] **Step 1: Add the `MasterDcBlock` struct + impl.** In `crates/deluge-dsp-kernels/src/filter.rs`, after the `impl OnePoleHp { ... }` block (~line 281):

```rust
/// Default master DC-block corner (Hz). Low enough to leave audible bass intact.
pub const DEFAULT_DC_HZ: f32 = 20.0;

/// Clamp a requested DC-block corner to a sane, finite, positive range so the
/// bilinear `tan(wc/2)` stays finite (NaN/inf/0/neg → default; else clamped).
fn sanitize_dc_hz(hz: f32) -> f32 {
    if hz.is_finite() { hz.clamp(0.1, 20_000.0) } else { DEFAULT_DC_HZ }
}

/// Stereo master DC-blocker: two independent one-pole high-passes (L/R). Applied
/// to the render root bus before the master limiter — DC eats headroom and biases
/// the limiter's peak detector, so it is removed first. Per-channel (no linking).
pub struct MasterDcBlock {
    ch: [OnePoleHp; 2], // [L, R]
}

impl MasterDcBlock {
    pub fn new(cutoff_hz: f32, dt: f32) -> MasterDcBlock {
        let mut m = MasterDcBlock { ch: [OnePoleHp::default(); 2] };
        m.set_cutoff(cutoff_hz, dt);
        m
    }
    /// Set both channels' corner. `wc = 2π·f_corner·dt` (radians/sample).
    pub fn set_cutoff(&mut self, cutoff_hz: f32, dt: f32) {
        let wc = sanitize_dc_hz(cutoff_hz) as f64
            * core::f64::consts::TAU
            * dt.max(1e-6) as f64;
        self.ch[0].set_coeff(wc);
        self.ch[1].set_coeff(wc);
    }
    /// High-pass `l`/`r` in place, per channel.
    pub fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        let n = l.len().min(r.len());
        for i in 0..n {
            l[i] = self.ch[0].process(l[i]);
            r[i] = self.ch[1].process(r[i]);
        }
    }
}
```

Note: `OnePoleHp::process` is module-private (`fn process`) — `MasterDcBlock` in the SAME `filter.rs` module can call it. If a `dead_code`/visibility error appears on `process` (e.g. it was only reached via other private callers), it is still in-module-accessible; do NOT change `OnePoleHp`'s visibility.

- [ ] **Step 2: Write the failing tests.** In `filter.rs`'s `#[cfg(test)] mod tests` (it already exists — it opens `use super::*;` and has `extern crate std;` per the crate convention; if not, add them):

```rust
    #[test]
    fn dcblock_removes_dc_offset() {
        const DT: f32 = 1.0 / 48_000.0;
        let mut dc = MasterDcBlock::new(20.0, DT);
        // Constant DC input → HP output decays toward 0 over a short settle.
        let mut l = [0.5f32; 4096];
        let mut r = [0.5f32; 4096];
        dc.process(&mut l, &mut r);
        assert!(l[4095].abs() < 1e-3, "L DC not removed: {}", l[4095]);
        assert!(r[4095].abs() < 1e-3, "R DC not removed: {}", r[4095]);
    }

    #[test]
    fn dcblock_preserves_ac_removes_dc() {
        const DT: f32 = 1.0 / 48_000.0;
        let mut dc = MasterDcBlock::new(20.0, DT);
        // 0.4 DC + a 1 kHz sine (amp 0.3). After settle: block mean ~0, AC intact.
        let f = 1000.0f32;
        let mut l = [0.0f32; 8192];
        let mut r = [0.0f32; 8192];
        for i in 0..8192 {
            let s = 0.4 + 0.3 * libm::sinf(core::f32::consts::TAU * f * (i as f32) * DT);
            l[i] = s;
            r[i] = s;
        }
        dc.process(&mut l, &mut r);
        // Mean of the settled tail ~0 (DC removed).
        let tail = &l[4096..];
        let mean: f32 = tail.iter().sum::<f32>() / tail.len() as f32;
        assert!(mean.abs() < 1e-2, "DC not removed, mean={}", mean);
        // AC amplitude preserved: settled peak still near 0.3.
        let peak = tail.iter().fold(0.0f32, |m, &x| m.max(x.abs()));
        assert!((peak - 0.3).abs() < 0.05, "AC amplitude lost, peak={}", peak);
    }

    #[test]
    fn dcblock_channels_independent() {
        const DT: f32 = 1.0 / 48_000.0;
        let mut dc = MasterDcBlock::new(20.0, DT);
        // Different per-channel DC offsets both removed independently.
        let mut l = [0.7f32; 4096];
        let mut r = [-0.3f32; 4096];
        dc.process(&mut l, &mut r);
        assert!(l[4095].abs() < 1e-3);
        assert!(r[4095].abs() < 1e-3);
    }

    #[test]
    fn dcblock_no_panic_on_adversarial_params() {
        let mut dc = MasterDcBlock::new(f32::NAN, 0.0);     // sanitized cutoff + floored dt
        let mut l = [0.5f32; 16];
        let mut r = [0.5f32; 16];
        dc.process(&mut l, &mut r);
        dc.set_cutoff(f32::INFINITY, f32::NAN);             // both sanitized
        dc.set_cutoff(-5.0, 1.0 / 48_000.0);               // negative cutoff clamped
        dc.set_cutoff(1e30, 1.0 / 48_000.0);               // huge cutoff clamped
        dc.process(&mut l, &mut r);                         // must not panic
        assert!(l[15].is_finite());
    }
```

- [ ] **Step 3: Run the tests, both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels dcblock
cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd dcblock
```
Expected: all 4 PASS both configs. (If `libm::sinf` isn't imported in the test module, use the fully-qualified `libm::sinf` — it is a crate dep.)

- [ ] **Step 4: Full kernels crate, both configs.**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels` and again `--features simd`. Expected: green (`OnePoleHp` unchanged; existing tests unaffected).

- [ ] **Step 5: Commit.**

```bash
git add crates/deluge-dsp-kernels/src/filter.rs
git commit -m "feat(kernels): MasterDcBlock — stereo one-pole DC-blocker (two OnePoleHp)"
```

---

### Task 2: Engine wiring (`Cmd::SetMasterDcBlock` + render seam)

**Files:**
- Modify: `crates/deluge-audio-graph/src/cmd.rs` (add variant after `SetMasterLimit`, ~line 29)
- Modify: `crates/deluge-audio-graph/src/engine.rs` (import ~19; field ~54; `new` ~81; `apply` ~170; `Reset` ~184-190; render seam ~400)
- Test: inline `#[cfg(test)]` in `engine.rs`

**Interfaces:**
- Consumes: `MasterDcBlock::{new, set_cutoff, process}` (Task 1).
- Produces: `Cmd::SetMasterDcBlock { cutoff_hz: f32 }`; field `master_dcblock: Option<MasterDcBlock>` reset on `Cmd::Reset`.

- [ ] **Step 1: Add the `Cmd` variant.** In `crates/deluge-audio-graph/src/cmd.rs`, after the `SetMasterLimit { ceiling: f32, release: f32 }` variant (~line 29):

```rust
    SetMasterLimit { ceiling: f32, release: f32 },
    /// Enable/configure the master DC-blocker on the root bus (before the limiter).
    /// Creates it if absent, else updates the corner in place.
    SetMasterDcBlock { cutoff_hz: f32 },
    Free { node: NodeId },
```

- [ ] **Step 2: Import + add the field.** In `crates/deluge-audio-graph/src/engine.rs`, near the existing `use deluge_dsp_kernels::limiter::MasterLimiter;` (~line 19):

```rust
use deluge_dsp_kernels::filter::MasterDcBlock;
```

And add the field next to `master_limiter` (~line 54):

```rust
    master_limiter: Option<MasterLimiter>,
    // Opt-in master DC-blocker on the root bus, applied at the render seam BEFORE
    // the limiter. `None` = disabled (render path byte-unchanged).
    master_dcblock: Option<MasterDcBlock>,
```

- [ ] **Step 3: Init in `new`.** Next to `master_limiter: None,` (~line 81):

```rust
            master_limiter: None,
            master_dcblock: None,
```

- [ ] **Step 4: Clear on Reset.** In the `Cmd::Reset` arm, after `self.master_limiter = None;`:

```rust
                self.master_limiter = None;
                self.master_dcblock = None;
```

- [ ] **Step 5: Add the apply arm.** After the `Cmd::SetMasterLimit { .. } => ...` arm (~line 170):

```rust
            Cmd::SetMasterDcBlock { cutoff_hz } => match &mut self.master_dcblock {
                Some(dc) => dc.set_cutoff(cutoff_hz, self.dt),
                None => self.master_dcblock = Some(MasterDcBlock::new(cutoff_hz, self.dt)),
            },
```

- [ ] **Step 6: Add the render seam (DC-block before the limiter).** Replace the existing limiter-only seam block:

```rust
        // Master limiter (opt-in): process the root bus in place before the clamp.
        if let Some(root) = self.root {
            if let Some(lim) = &mut self.master_limiter {
                let b = root.0 as usize;
                lim.process(&mut self.bus_l[b], &mut self.bus_r[b], self.dt);
            }
        }
```

with the combined master-chain seam (DC-block first, then limiter):

```rust
        // Master chain (opt-in): DC-block → limiter, on the root bus before the clamp.
        if let Some(root) = self.root {
            let b = root.0 as usize;
            if let Some(dc) = &mut self.master_dcblock {
                dc.process(&mut self.bus_l[b], &mut self.bus_r[b]);
            }
            if let Some(lim) = &mut self.master_limiter {
                lim.process(&mut self.bus_l[b], &mut self.bus_r[b], self.dt);
            }
        }
```

(The `// Copy root bus to output, clamped.` block that follows is UNCHANGED.)

- [ ] **Step 7: Write the failing engine tests.** In `engine.rs` tests (`type E = Engine<16, 8, 8, 4, 45056, 2048>;`, `use crate::Cmd;`):

```rust
    #[test]
    fn master_dcblock_removes_dc_when_enabled() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5); // DC
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::SetMasterDcBlock { cutoff_hz: 20.0 });
        // Render several blocks so the HP settles.
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        for _ in 0..512 { e.render(&mut out, &sil); }
        assert!(out[15].l.abs() < 1e-2, "DC not removed: {}", out[15].l);
    }

    #[test]
    fn master_dcblock_disabled_is_byte_identical() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6); // no DC-block → 0.5 passes
    }

    #[test]
    fn master_dcblock_reset_clears() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::SetMasterDcBlock { cutoff_hz: 20.0 });
        e.apply(Cmd::Reset);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6); // DC-block cleared → 0.5 passes
    }
```

- [ ] **Step 8: Run engine tests, then full graph crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph master_dcblock
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features deluge-dsp-kernels/simd
```
Expected: 3 new tests PASS; full crate green both configs (existing `master_limiter_*` tests still pass — the seam refactor is behavior-preserving when `master_dcblock` is `None`).

- [ ] **Step 9: Confirm dependent crates still build.**

Run: `cargo build --target x86_64-unknown-linux-gnu -p deluge-wren-core` and `cargo build -p wren-firmware --target armv7a-none-eabihf`. Expected: clean.

- [ ] **Step 10: Commit.**

```bash
git add crates/deluge-audio-graph/src/cmd.rs crates/deluge-audio-graph/src/engine.rs
git commit -m "feat(graph): Cmd::SetMasterDcBlock + master DC-block before the limiter"
```

---

### Task 3: Wren `Out.dcBlock(...)` binding

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (add `set_master_dcblock` near `set_master_limit` ~545)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_master_dcblock_impl` + shim near `node_master_limit` ~2553; `register_audio` ~2734)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (`METHODS` ~889)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`foreign static masterDcBlock_` ~128; `class Out` ~984)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Cmd::SetMasterDcBlock { cutoff_hz }` (Task 2).
- Produces: Wren `Out.dcBlock()` / `Out.dcBlock(cutoff)`.

- [ ] **Step 1: Write the failing Cmd-capture tests.** In `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn out_dcblock_no_arg_emits_default_cutoff() {
    let cmds = run_and_capture_cmds("Out.dcBlock()");
    assert_eq!(cmds, vec![Cmd::SetMasterDcBlock { cutoff_hz: 20.0 }]);
}

#[test]
fn out_dcblock_arg_emits_cutoff() {
    let cmds = run_and_capture_cmds("Out.dcBlock(10)");
    assert_eq!(cmds, vec![Cmd::SetMasterDcBlock { cutoff_hz: 10.0 }]);
}
```

- [ ] **Step 2: Run to verify fail.**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support out_dcblock`
Expected: FAIL (Wren `Out.dcBlock` / `Node.masterDcBlock_` unknown).

- [ ] **Step 3: Add the facade.** In `crates/deluge-wren-core/src/audio.rs`, near `set_master_limit` (~545):

```rust
pub fn set_master_dcblock(cutoff_hz: f32) {
    host().audio_cmd(Cmd::SetMasterDcBlock { cutoff_hz });
}
```

- [ ] **Step 4: Add the binding impl + shim.** In `crates/deluge-wren-core/src/bindings_audio.rs`, near `node_master_limit_impl` (~2544):

```rust
pub(crate) fn node_master_dcblock_impl<S: SlotApi>(vm: &S) {
    let cutoff = vm.get_f(1) as f32;
    audio::set_master_dcblock(cutoff);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_master_dcblock(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_master_dcblock_impl(&vm);
}
```

- [ ] **Step 5: Register in `register_audio`.** In `bindings_audio.rs` near line 2734 (next to `masterLimit_`):

```rust
    method("main", "Node", true, "masterDcBlock_(_)", node_master_dcblock_impl::<S>);
```

- [ ] **Step 6: Register in `METHODS`.** In `crates/deluge-wren-core/src/bindings.rs` near line 889 (next to `masterLimit_`):

```rust
    static_method("Node", "masterDcBlock_(_)", bindings_audio::node_master_dcblock),
```

- [ ] **Step 7: Add the prelude decls.** In `crates/deluge-wren-core/wren/prelude.wren`, add to `class Node` (after `foreign static masterLimit_(ceiling, release)`, ~line 128):

```wren
  foreign static masterDcBlock_(cutoff)
```

And extend `class Out` (after the `limit` methods) — single-line **implicit** return (no `return` keyword):

```wren
  static dcBlock() { Node.masterDcBlock_(20.0) }
  static dcBlock(cutoff) { Node.masterDcBlock_(cutoff) }
```

- [ ] **Step 8: Run the Cmd-capture tests, then full crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support out_dcblock
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd
```
Expected: both `out_dcblock_*` PASS; full crate green both configs.

- [ ] **Step 9: Cross-build check.**

Run: `cargo build -p wren-firmware --target armv7a-none-eabihf`. Expected: clean.

- [ ] **Step 10: Commit.**

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs \
        crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren \
        crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): Out.dcBlock(cutoff) master DC-block binding"
```

---

### Task 4: e2e — a DC-offset source cleaned by `Out.dcBlock`

**Files:**
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Out.dcBlock(...)` (Task 3) + the real-`EngineHost` render harness (the `out_limit_*` / `sample_new_*` render tests are the template).

- [ ] **Step 1: Write the failing e2e test.** In `crates/deluge-wren-core/tests/audio_bindings.rs`, mirror the existing real-`EngineHost` render tests (grep `EngineHost` — the ones rendering actual frames, e.g. the IO-3a `out_limit_bounds_a_loud_render`). Build a DC-offset source, apply `Out.dcBlock()`, render enough blocks to settle, assert the block mean is near 0:

```rust
#[test]
fn out_dcblock_removes_dc_from_a_render() {
    // A DC-ish source (a constant-ish offset) through Out.dcBlock(): settled mean ~0.
    // Use the exact real-EngineHost harness + render entry the out_limit_* tests use.
    let mut h = /* real EngineHost harness, sample rate 48_000.0 */;
    h.run("Out.patch(Osc.saw(110) * 0.3 + 0.4)"); // AC + DC offset, if `+` is supported
    h.run("Out.dcBlock()");
    let mut mean = 0.0f32;
    let mut last = Vec::new();
    for _ in 0..512 { last = h.render_block(); } // settle
    for f in &last { mean += f.l; }
    mean /= last.len() as f32;
    assert!(mean.abs() < 5e-2, "DC not removed, mean={}", mean);
    assert!(last.iter().any(|f| f.l.abs() > 1e-4), "should not be silent");
}
```

Adapt to the ACTUAL `EngineHost` API in that file. If building `Osc.saw(...) * 0.3 + 0.4` (adding a DC offset) is not supported by the graph DSL, use whatever produces a DC-biased signal the harness allows — e.g. a source with a constant offset, or just a loud saw and assert the settled mean is near 0 (a symmetric saw already has ~0 mean, so prefer a genuinely DC-offset source; if none is expressible, assert the DC-block render stays finite + non-silent + its mean is no larger than the undisturbed baseline). Do NOT weaken to always-true; the goal is to show the DC-block runs end-to-end and reduces DC.

- [ ] **Step 2: Run it, then full crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support out_dcblock_removes_dc
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd
```
Expected: PASS; full crate green both configs.

- [ ] **Step 3: Commit.**

```bash
git add crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "test(wren): e2e — Out.dcBlock removes DC from a render"
```

---

## Self-Review

**Spec coverage:** `MasterDcBlock` kernel (two `OnePoleHp`, per-channel, no-panic) → Task 1. Engine `Option<MasterDcBlock>` + `Cmd::SetMasterDcBlock` + render seam before the limiter + Reset clear → Task 2. `Out.dcBlock()`/`Out.dcBlock(cutoff)` both tables + shim + prelude, 20.0 default → Task 3. e2e DC removed → Task 4. All spec sections covered.

**Placeholder scan:** Task 4's harness calls are marked "adapt to the actual `EngineHost` API" (the named `out_limit_*` sibling is the template) — a genuine follow-the-template instruction, not a gap. All kernel/engine/binding steps carry exact code.

**Type consistency:** `MasterDcBlock::{new, set_cutoff, process}` (Task 1) consumed with those exact names in Task 2's apply arm + seam. `Cmd::SetMasterDcBlock { cutoff_hz: f32 }` (Task 2) emitted identically by `audio::set_master_dcblock` (Task 3) and asserted in the Task 3 tests. Default `20.0` consistent across `DEFAULT_DC_HZ`, the prelude nullary form, and the Task 3 test.
