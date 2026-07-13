# Master Limiter (IO-3a) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in, stereo-linked, feedforward peak limiter on the render root bus (before the existing `[-1,1]` clamp), configured from Wren via `Out.limit(...)`.

**Architecture:** A new scalar `MasterLimiter` kernel (peak → instant-attack / one-pole-release gain envelope, one linked gain for L+R). The engine holds it as `Option<MasterLimiter>` (`None` = render byte-unchanged), invokes it at the render seam, and configures it via a new `Cmd::SetMasterLimit`. A Wren `Out.limit(ceiling[, release])` factory emits that Cmd.

**Tech Stack:** Rust `no_std` (crates `deluge-dsp-kernels`, `deluge-audio-graph`, `deluge-wren-core`); `libm` for `expf`/`fabsf`; Wren via `wren-sys`.

## Global Constraints

- `no_std`, no heap, **no panic on any input** — ceiling/release finite-guarded + floored (`.max(1e-4)`); non-finite input samples pass through at gain 1.0 (finite-check `peak` before the ceiling compare); the `peak > ceiling > 0` guard prevents div-by-zero.
- 32-bit-`usize` safe (index `0..len` only).
- **Linked stereo:** ONE shared gain applied to both L and R (no image shift). Ceiling **linear** (0..1, default `0.95`); release seconds (default `0.05`).
- **Scalar** — serial gain-envelope recursion; no `f32x8`/NEON path (per prefer-neon-simd "don't force it"). Kernel tests pass identically in both crate configs.
- **Opt-in, additive:** `Option<MasterLimiter>` `None` default ⇒ render path byte-identical when unused. The final `[-1,1]` clamp REMAINS.
- Wren reads only `get_f` (safe — no `checked_*`). Prelude bodies: single-line **implicit** return is fine (`{ Node.x_(...) }`); a single-line body with explicit `return` does NOT compile — avoid it.
- Tests per-crate (NEVER `--workspace`), BOTH configs (default + `--features simd` for kernels; `--features deluge-dsp-kernels/simd` for graph/wren-core), target `x86_64-unknown-linux-gnu`. `armv7a … can't find crate for test` LSP output is noise.
- MIT/Apache-2.0.

---

## File Structure

- `crates/deluge-dsp-kernels/src/limiter.rs` — NEW: `MasterLimiter` kernel (Task 1).
- `crates/deluge-dsp-kernels/src/lib.rs` — add `pub mod limiter;` (Task 1).
- `crates/deluge-audio-graph/src/cmd.rs` — add `Cmd::SetMasterLimit` variant (Task 2).
- `crates/deluge-audio-graph/src/engine.rs` — `master_limiter` field + init + apply arm + Reset clear + render seam (Task 2).
- `crates/deluge-wren-core/src/audio.rs` — `set_master_limit` facade (Task 3).
- `crates/deluge-wren-core/src/bindings_audio.rs` — `node_master_limit_impl` + shim + `register_audio` (Task 3).
- `crates/deluge-wren-core/src/bindings.rs` — `METHODS` registration (Task 3).
- `crates/deluge-wren-core/wren/prelude.wren` — `foreign static masterLimit_` + `class Out` methods (Task 3).
- `crates/deluge-wren-core/tests/audio_bindings.rs` — Cmd-capture (Task 3) + e2e (Task 4).

---

### Task 1: `MasterLimiter` kernel

**Files:**
- Create: `crates/deluge-dsp-kernels/src/limiter.rs`
- Modify: `crates/deluge-dsp-kernels/src/lib.rs:17` (add `pub mod limiter;` after `pub mod lfo;`)
- Test: inline `#[cfg(test)] mod tests` in `limiter.rs`

**Interfaces:**
- Produces: `MasterLimiter` with `new(ceiling: f32, release: f32) -> MasterLimiter`, `set_ceiling(&mut self, f32)`, `set_release(&mut self, f32)`, `process(&mut self, l: &mut [f32], r: &mut [f32], dt: f32)`.

- [ ] **Step 1: Add the module.** In `crates/deluge-dsp-kernels/src/lib.rs`, add after line 17 (`pub mod lfo;`):

```rust
pub mod limiter;
```

- [ ] **Step 2: Write the failing kernel.** Create `crates/deluge-dsp-kernels/src/limiter.rs`:

```rust
//! Master limiter: a stereo-linked feedforward peak limiter applied to the
//! render root bus before the output clamp. Scalar — the gain envelope is a
//! serial recursion (`gain[i]` depends on `gain[i-1]`), so it does not
//! vectorize (see prefer-neon-simd "don't force it").

const DEFAULT_CEILING: f32 = 0.95;
const DEFAULT_RELEASE: f32 = 0.05;
const FLOOR: f32 = 1e-4;

/// A finite, strictly-positive ceiling (0/neg/NaN → default). Prevents both a
/// div-by-zero and an over-attenuate-to-silence on a bad param.
fn sanitize_ceiling(v: f32) -> f32 {
    if v.is_finite() { v.max(FLOOR) } else { DEFAULT_CEILING }
}
/// A finite, strictly-positive release time (0/neg/NaN → default).
fn sanitize_release(v: f32) -> f32 {
    if v.is_finite() { v.max(FLOOR) } else { DEFAULT_RELEASE }
}

#[derive(Clone, Copy)]
pub struct MasterLimiter {
    gain: f32,      // current gain reduction, 1.0 = no reduction (envelope state)
    ceiling: f32,   // linear output ceiling (0..1)
    release_s: f32, // release time, seconds
}

impl MasterLimiter {
    pub fn new(ceiling: f32, release: f32) -> MasterLimiter {
        MasterLimiter { gain: 1.0, ceiling: sanitize_ceiling(ceiling), release_s: sanitize_release(release) }
    }
    pub fn set_ceiling(&mut self, v: f32) { self.ceiling = sanitize_ceiling(v); }
    pub fn set_release(&mut self, v: f32) { self.release_s = sanitize_release(v); }

    /// Limit `l`/`r` in place with a single linked gain. `dt` = 1/sample_rate.
    pub fn process(&mut self, l: &mut [f32], r: &mut [f32], dt: f32) {
        // One-pole release coefficient: 1 - e^(-dt/tau), tau floored at dt.
        let rel_c = 1.0 - libm::expf(-dt / self.release_s.max(dt));
        let n = l.len().min(r.len());
        for i in 0..n {
            let li = l[i];
            let ri = r[i];
            let peak = libm::fabsf(li).max(libm::fabsf(ri));
            // Non-finite peak (NaN/inf sample) → pass through at unity, no panic.
            let target = if peak.is_finite() && peak > self.ceiling {
                self.ceiling / peak // peak > ceiling > 0 ⇒ divisor strictly positive
            } else {
                1.0
            };
            // Instant attack (snap down), one-pole release toward target.
            let c = if target < self.gain { 1.0 } else { rel_c };
            self.gain += (target - self.gain) * c;
            l[i] = li * self.gain;
            r[i] = ri * self.gain;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    const DT: f32 = 1.0 / 48_000.0;

    #[test]
    fn limits_peak_to_ceiling() {
        // Constant 0.8 in both channels, ceiling 0.5 → every out |sample| <= 0.5.
        let mut lim = MasterLimiter::new(0.5, 0.05);
        let mut l = [0.8f32; 64];
        let mut r = [0.8f32; 64];
        lim.process(&mut l, &mut r, DT);
        for i in 0..64 {
            assert!(l[i].abs() <= 0.5 + 1e-6, "l[{}]={}", i, l[i]);
            assert!(r[i].abs() <= 0.5 + 1e-6, "r[{}]={}", i, r[i]);
        }
        // Instant attack ⇒ even sample 0 is clamped.
        assert!((l[0] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn linked_preserves_channel_ratio() {
        // Loud L, quiet R, same gain ⇒ R/L ratio preserved (no image shift).
        let mut lim = MasterLimiter::new(0.5, 0.05);
        let mut l = [0.8f32; 64];
        let mut r = [0.2f32; 64];
        lim.process(&mut l, &mut r, DT);
        // both scaled by the same gain: out_r/out_l == in_r/in_l == 0.25
        assert!((l[0] - 0.5).abs() < 1e-6);          // L clamped to ceiling
        assert!((r[0] / l[0] - 0.25).abs() < 1e-6);  // ratio preserved
    }

    #[test]
    fn passes_below_ceiling_unchanged() {
        let mut lim = MasterLimiter::new(0.5, 0.05);
        let mut l = [0.3f32; 64];
        let mut r = [0.3f32; 64];
        lim.process(&mut l, &mut r, DT);
        for i in 0..64 {
            assert!((l[i] - 0.3).abs() < 1e-7);
            assert!((r[i] - 0.3).abs() < 1e-7);
        }
    }

    #[test]
    fn gain_recovers_after_loud() {
        // Drive gain down with a loud block, then feed quiet blocks; the applied
        // gain (out/in) should climb back toward 1.0.
        let mut lim = MasterLimiter::new(0.5, 0.001); // fast release for the test
        let mut l = [1.0f32; 64];
        let mut r = [1.0f32; 64];
        lim.process(&mut l, &mut r, DT); // gain now ~0.5
        let mut gains = std::vec::Vec::new();
        for _ in 0..40 {
            let mut ql = [0.1f32; 64];
            let mut qr = [0.1f32; 64];
            lim.process(&mut ql, &mut qr, DT);
            gains.push(ql[63] / 0.1); // applied gain this block
        }
        assert!(gains[0] < gains[gains.len() - 1], "gain should recover");
        assert!(*gains.last().unwrap() > 0.9, "gain should approach 1.0");
    }

    #[test]
    fn no_panic_on_adversarial_input() {
        // NaN/inf/zero/negative/huge params + non-finite samples must not panic.
        let mut lim = MasterLimiter::new(f32::NAN, -1.0); // → sanitized defaults
        let mut l = [f32::NAN, f32::INFINITY, 0.0, 1e30, -1e30, 0.5];
        let mut r = [0.0, f32::NEG_INFINITY, f32::NAN, 1.0, -1.0, 0.5];
        lim.process(&mut l, &mut r, DT);
        lim.set_ceiling(0.0);        // floored, not div-by-zero
        lim.set_release(f32::NAN);   // → default
        let mut l2 = [2.0f32; 8];
        let mut r2 = [2.0f32; 8];
        lim.process(&mut l2, &mut r2, DT); // must not panic
        assert!(l2[7].is_finite());
    }
}
```

- [ ] **Step 3: Run the tests, both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels limiter::
cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd limiter::
```
Expected: all 5 tests PASS in both configs.

- [ ] **Step 4: Full kernels crate, both configs.**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels` and again `--features simd`. Expected: green (existing tests unaffected).

- [ ] **Step 5: Commit.**

```bash
git add crates/deluge-dsp-kernels/src/limiter.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(kernels): MasterLimiter — stereo-linked feedforward peak limiter"
```

---

### Task 2: Engine wiring (`Cmd::SetMasterLimit` + render seam)

**Files:**
- Modify: `crates/deluge-audio-graph/src/cmd.rs:26` (add variant after `SetRoot`)
- Modify: `crates/deluge-audio-graph/src/engine.rs` (import; struct field ~50; `new` ~76; `apply` ~161; `Reset` ~175-180; render seam ~388)
- Test: inline `#[cfg(test)] mod tests` in `engine.rs`

**Interfaces:**
- Consumes: `MasterLimiter::{new, set_ceiling, set_release, process}` (Task 1).
- Produces: `Cmd::SetMasterLimit { ceiling: f32, release: f32 }`; engine field `master_limiter: Option<MasterLimiter>` reset on `Cmd::Reset`.

- [ ] **Step 1: Add the `Cmd` variant.** In `crates/deluge-audio-graph/src/cmd.rs`, add after `SetRoot { bus: BusId },` (line 26):

```rust
    SetRoot { bus: BusId },
    /// Enable/configure the master limiter on the root bus. Creates it if absent,
    /// else updates params in place (preserving the running gain envelope).
    SetMasterLimit { ceiling: f32, release: f32 },
    Free { node: NodeId },
```

(`Cmd` derives `Clone, Copy, Debug, PartialEq`; all-`f32` fields satisfy them.)

- [ ] **Step 2: Import + add the engine field.** In `crates/deluge-audio-graph/src/engine.rs`, add near the top `use` lines:

```rust
use deluge_dsp_kernels::limiter::MasterLimiter;
```

And add the field to the `Engine` struct after `stream_state` (line 50):

```rust
    stream_state: [Option<crate::stream::StreamCursors>; NODES],
    // Opt-in master limiter on the root bus, applied at the render seam before
    // the output clamp. `None` = disabled (render path byte-unchanged).
    master_limiter: Option<MasterLimiter>,
```

- [ ] **Step 3: Init the field in `new`.** In `Engine::new` (after `stream_state: [None; NODES],`, line 76):

```rust
            stream_state: [None; NODES],
            master_limiter: None,
```

- [ ] **Step 4: Clear it on Reset.** In the `Cmd::Reset` arm (lines 175-180), add the clear:

```rust
            Cmd::Reset => {
                self.arena.reset();
                self.writes_len = 0;
                self.root = None;
                self.stream_state = [None; NODES];
                self.master_limiter = None;
            }
```

- [ ] **Step 5: Add the apply arm.** After the `Cmd::SetRoot { bus } => self.set_root(bus),` arm (line 161):

```rust
            Cmd::SetRoot { bus } => self.set_root(bus),
            Cmd::SetMasterLimit { ceiling, release } => match &mut self.master_limiter {
                Some(lim) => { lim.set_ceiling(ceiling); lim.set_release(release); }
                None => self.master_limiter = Some(MasterLimiter::new(ceiling, release)),
            },
```

- [ ] **Step 6: Add the render seam call.** In `render`, between the end of the bus-write loop (line 388, the closing `}` of the `for w in ...` loop) and the `// Copy root bus to output, clamped.` comment (line 389):

```rust
        }
        // Master limiter (opt-in): process the root bus in place before the clamp.
        if let Some(root) = self.root {
            if let Some(lim) = &mut self.master_limiter {
                let b = root.0 as usize;
                lim.process(&mut self.bus_l[b], &mut self.bus_r[b], self.dt);
            }
        }
        // Copy root bus to output, clamped.
        let n = out.len().min(BLOCK);
```

(`master_limiter`, `bus_l`, `bus_r`, `dt` are disjoint `Engine` fields — no borrow conflict; the `arr` borrow from line 371 is last used at line 388, before this seam.)

- [ ] **Step 7: Write the failing engine tests.** In `engine.rs`'s `#[cfg(test)] mod tests` (uses `type E = Engine<16, 8, 8, 4, 45056, 2048>;`, and `use crate::Cmd;`):

```rust
    #[test]
    fn master_limiter_bounds_root_bus_when_enabled() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::SetMasterLimit { ceiling: 0.5, release: 0.05 });
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        for i in 0..16 {
            assert!(out[i].l.abs() <= 0.5 + 1e-6, "out[{}].l={}", i, out[i].l);
        }
    }

    #[test]
    fn master_limiter_disabled_is_raw_then_clamp() {
        // Without SetMasterLimit, a 0.8 root bus passes at 0.8 (below the clamp).
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.8).abs() < 1e-6); // unlimited, only the [-1,1] clamp would act
    }

    #[test]
    fn master_limiter_reset_clears() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::SetMasterLimit { ceiling: 0.5, release: 0.05 });
        e.apply(Cmd::Reset);
        // After Reset the graph is torn down; rebuild the same patch, no limiter.
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.8).abs() < 1e-6); // limiter cleared → unlimited
    }
```

(Confirm `e.apply(...)`, `e.create(...)`, `e.node_input_mut(...)`, `e.bus_write(...)`, `e.set_root(...)` are the exact test-facing methods — they appear in the existing `render_clamps_to_unit_range` / `cmd.rs` tests. `E::new` takes the sample rate; use `48_000.0` so `dt` matches the kernel's ballistics.)

- [ ] **Step 8: Run the engine tests, then the full graph crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph master_limiter
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features deluge-dsp-kernels/simd
```
Expected: the 3 new tests PASS; full crate green both configs.

- [ ] **Step 9: Confirm dependent crates still build.**

Run: `cargo build --target x86_64-unknown-linux-gnu -p deluge-wren-core` and `cargo build -p wren-firmware --target armv7a-none-eabihf`. Expected: clean (this task is additive; no signature changes).

- [ ] **Step 10: Commit.**

```bash
git add crates/deluge-audio-graph/src/cmd.rs crates/deluge-audio-graph/src/engine.rs
git commit -m "feat(graph): Cmd::SetMasterLimit + master limiter on the render root bus"
```

---

### Task 3: Wren `Out.limit(...)` binding

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (add `set_master_limit` near `set_root` ~537)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_master_limit_impl` + shim near `node_reset` ~2535; `register_audio` ~2722)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (`METHODS` ~888)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`foreign static masterLimit_` in `class Node` ~127; `class Out` ~983)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Cmd::SetMasterLimit { ceiling, release }` (Task 2).
- Produces: Wren `Out.limit(ceiling)` and `Out.limit(ceiling, release)`.

- [ ] **Step 1: Write the failing Cmd-capture tests.** In `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn out_limit_one_arg_emits_set_master_limit_with_default_release() {
    let cmds = run_and_capture_cmds("Out.limit(0.9)");
    assert_eq!(cmds, vec![Cmd::SetMasterLimit { ceiling: 0.9, release: 0.05 }]);
}

#[test]
fn out_limit_two_arg_emits_set_master_limit() {
    let cmds = run_and_capture_cmds("Out.limit(0.5, 0.1)");
    assert_eq!(cmds, vec![Cmd::SetMasterLimit { ceiling: 0.5, release: 0.1 }]);
}
```

- [ ] **Step 2: Run to verify fail.**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support out_limit`
Expected: FAIL (Wren `Out.limit` / `Node.masterLimit_` unknown).

- [ ] **Step 3: Add the `audio::set_master_limit` facade.** In `crates/deluge-wren-core/src/audio.rs`, near `set_root` (line 537):

```rust
pub fn set_master_limit(ceiling: f32, release: f32) {
    host().audio_cmd(Cmd::SetMasterLimit { ceiling, release });
}
```

- [ ] **Step 4: Add the binding impl + shim.** In `crates/deluge-wren-core/src/bindings_audio.rs`, near `node_reset_impl` (line 2535):

```rust
pub(crate) fn node_master_limit_impl<S: SlotApi>(vm: &S) {
    let ceiling = vm.get_f(1) as f32;
    let release = vm.get_f(2) as f32;
    audio::set_master_limit(ceiling, release);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_master_limit(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_master_limit_impl(&vm);
}
```

- [ ] **Step 5: Register in `register_audio`.** In `bindings_audio.rs` near line 2722 (next to `reset_()`):

```rust
    method("main", "Node", true, "masterLimit_(_,_)", node_master_limit_impl::<S>);
```

- [ ] **Step 6: Register in `METHODS`.** In `crates/deluge-wren-core/src/bindings.rs` near line 888 (next to `reset_()`):

```rust
    static_method("Node", "masterLimit_(_,_)", bindings_audio::node_master_limit),
```

- [ ] **Step 7: Add the prelude decls.** In `crates/deluge-wren-core/wren/prelude.wren`, add to `class Node` (after `foreign static reset_()`, line 127):

```wren
  foreign static masterLimit_(ceiling, release)
```

And extend `class Out` (lines 983-986) — use single-line **implicit** return (no `return` keyword; a single-line explicit-`return` body does NOT compile in this Wren fork):

```wren
class Out {
  static patch(node) { Node.patch_(node) }
  static reset() { Node.reset_() }
  static limit(ceiling) { Node.masterLimit_(ceiling, 0.05) }
  static limit(ceiling, release) { Node.masterLimit_(ceiling, release) }
}
```

- [ ] **Step 8: Run the Cmd-capture tests, then full crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support out_limit
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd
```
Expected: both `out_limit_*` PASS; full crate green both configs.

- [ ] **Step 9: Cross-build check.**

Run: `cargo build -p wren-firmware --target armv7a-none-eabihf`. Expected: clean.

- [ ] **Step 10: Commit.**

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs \
        crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren \
        crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): Out.limit(ceiling[, release]) master limiter binding"
```

---

### Task 4: e2e — a loud synth limited near the ceiling

**Files:**
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Out.limit(...)` (Task 3) + the real-`EngineHost` render harness (the `sample_new_*` / `line_in_*` render round-trips are the template).

- [ ] **Step 1: Write the failing e2e test.** In `crates/deluge-wren-core/tests/audio_bindings.rs`, mirror the existing real-`EngineHost` render tests (grep `EngineHost` in that file — the ones that render actual audio frames, not `CmdCaptureHost`). Build a loud patch, apply `Out.limit`, render, assert the output is bounded near the ceiling:

```rust
#[test]
fn out_limit_bounds_a_loud_render() {
    // A loud DC-ish source patched to master + Out.limit(0.5): output |sample| <= 0.5+tol.
    // (Use the exact real-EngineHost harness + render entry the sample_new_* tests use.)
    let mut h = /* real EngineHost harness */;
    h.run("Out.patch(Osc.saw(110) * 4.0); Out.limit(0.5)");
    let out = h.render_block(); // whatever the harness's render-and-collect entry is
    for f in &out {
        assert!(f.l.abs() <= 0.5 + 1e-3, "left over ceiling: {}", f.l);
        assert!(f.r.abs() <= 0.5 + 1e-3, "right over ceiling: {}", f.r);
    }
    // and it is not silent (the limiter attenuates, doesn't mute)
    assert!(out.iter().any(|f| f.l.abs() > 1e-4));
}

#[test]
fn out_without_limit_unbounded_by_limiter() {
    // Sanity: same patch WITHOUT Out.limit still renders (only the [-1,1] clamp acts).
    let mut h = /* real EngineHost harness */;
    h.run("Out.patch(Osc.saw(110) * 4.0)");
    let out = h.render_block();
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite()));
}
```

Adapt the harness construction/`run`/render-collect calls to the ACTUAL `EngineHost` API in that test file (the `Osc.saw(110) * 4.0` overdrives past 0.5 so the limiter visibly acts). If `EngineHost::new` needs a sample rate, pass `48_000.0` so the limiter ballistics match. Do NOT weaken the ceiling assertion to always-true.

- [ ] **Step 2: Run it, then full crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support out_limit_bounds
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd
```
Expected: PASS; full crate green both configs.

- [ ] **Step 3: Commit.**

```bash
git add crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "test(wren): e2e — Out.limit bounds a loud render near the ceiling"
```

---

## Self-Review

**Spec coverage:**
- `MasterLimiter` kernel (peak → linked gain envelope, instant attack / one-pole release, in-place stereo) → Task 1. ✅
- No-panic guards (ceiling/release floor, non-finite peak pass-through, div-by-zero prevention) → Task 1 (`sanitize_*`, `no_panic_on_adversarial_input`). ✅
- Linked stereo (one shared gain) → Task 1 (`linked_preserves_channel_ratio`). ✅
- Scalar / both-config identical → Task 1 Steps 3-4. ✅
- Engine `Option<MasterLimiter>` field + `None` default + Reset clear → Task 2. ✅
- `Cmd::SetMasterLimit` create-or-update-in-place → Task 2 Step 5. ✅
- Render seam before the clamp; clamp remains → Task 2 Step 6. ✅
- Disabled ⇒ byte-identical render → Task 2 (`master_limiter_disabled_is_raw_then_clamp`). ✅
- Wren `Out.limit(ceiling)` / `Out.limit(ceiling, release)`, both tables + shim + prelude, `get_f`-only → Task 3. ✅
- Prelude implicit-return one-liner (avoid explicit-`return` one-liner) → Task 3 Step 7. ✅
- e2e loud-synth bounded near ceiling → Task 4. ✅

**Placeholder scan:** Task 4's harness calls are marked "adapt to the actual `EngineHost` API" — a genuine "follow the named sibling template" instruction (the `sample_new_*`/`line_in_*` render tests), not a gap; all kernel/engine/binding steps carry exact code.

**Type consistency:** `MasterLimiter::{new, set_ceiling, set_release, process}` (Task 1) are consumed with those exact names/signatures in Task 2's apply arm + render seam. `Cmd::SetMasterLimit { ceiling: f32, release: f32 }` (Task 2) is emitted identically by `audio::set_master_limit` (Task 3) and asserted in the Task 3 Cmd-capture tests. Default release `0.05` is consistent between the kernel default, the `Out.limit(ceiling)` prelude one-arg form, and the Task 3 test assertion.
