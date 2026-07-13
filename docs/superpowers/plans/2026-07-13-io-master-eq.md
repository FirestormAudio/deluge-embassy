# Master EQ (IO-3c) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in, stereo single-band EQ on the render root bus, between the DC-block and the limiter, configured from Wren via `Out.eq`/`Out.eqLowShelf`/`Out.eqHighShelf`.

**Architecture:** A new `pub MasterEq` in `eq.rs` wrapping two existing `Eq` (L/R) plus a small additive `Eq::process_in_place`. The engine holds it as `Option<MasterEq>` (`None` = render byte-unchanged), applies it at the render seam between DC-block and limiter, and configures it via a new `Cmd::SetMasterEq`. Wren `Out.eq(...)` emits the Cmd. Completes the IO-3 master chain (DC-block → EQ → limiter). Mirrors merged IO-3a/3b.

**Tech Stack:** Rust `no_std` (crates `deluge-dsp-kernels`, `deluge-audio-graph`, `deluge-wren-core`); `Eq`/RBJ biquad; Wren via `wren-sys`.

## Global Constraints

- `no_std`, no heap, **no panic on any input** — `freq`/`gain_db`/`q` finite-guarded in `MasterEq::set_params` (NaN/inf → safe default) before `Eq`'s range-clamps; `dt` guarded in `MasterEq::process`.
- 32-bit-`usize` safe (index `0..len`).
- **Stereo** = two `Eq` (L/R), same params. Type codes `0=Peak, 1=LowShelf, 2=HighShelf`.
- **Scalar** — biquad IIR recursion; no `f32x8`/NEON path. Tests pass identically both configs.
- **Opt-in, additive:** `Option<MasterEq>` `None` default ⇒ render byte-identical when unused. The DC-block, limiter, and `[-1,1]` clamp are UNCHANGED. `Eq::process` and `Kind::Eq` UNCHANGED (only the additive `Eq::process_in_place` is new).
- Wren reads only `get_f` (safe — no `checked_*`). Prelude bodies: single-line **implicit** return only.
- Tests per-crate (NEVER `--workspace`), BOTH configs (default + `--features simd` for kernels; `--features deluge-dsp-kernels/simd` for graph/wren-core), target `x86_64-unknown-linux-gnu`. `armv7a … can't find crate for test` LSP output is noise.
- MIT/Apache-2.0.

---

## File Structure

- `crates/deluge-dsp-kernels/src/eq.rs` — NEW `pub struct MasterEq` + `Eq::process_in_place` (Task 1).
- `crates/deluge-audio-graph/src/cmd.rs` — add `Cmd::SetMasterEq` (Task 2).
- `crates/deluge-audio-graph/src/engine.rs` — `master_eq` field + init + apply arm + Reset clear + render seam (Task 2).
- `crates/deluge-wren-core/src/audio.rs` — `set_master_eq` facade (Task 3).
- `crates/deluge-wren-core/src/bindings_audio.rs` — `node_master_eq_impl` + shim + `register_audio` (Task 3).
- `crates/deluge-wren-core/src/bindings.rs` — `METHODS` registration (Task 3).
- `crates/deluge-wren-core/wren/prelude.wren` — `foreign static masterEq_` + `class Out` methods (Task 3).
- `crates/deluge-wren-core/tests/audio_bindings.rs` — Cmd-capture (Task 3) + e2e (Task 4).

---

### Task 1: `MasterEq` kernel + `Eq::process_in_place`

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/eq.rs` (add `Eq::process_in_place` in the `impl Eq`; add `MasterEq` after it)
- Test: inline `#[cfg(test)]` in `eq.rs`

**Interfaces:**
- Consumes: `pub struct Eq` (Clone/Copy; `new(EqType)`, `set_freq/set_gain/set_q(f32)`, `set_type(u8)`), the private `rbj_coeffs(ty, freq, gain, q, dt)`, `Biquad::{set_coeffs, tick}` — all in `eq.rs`.
- Produces: `Eq::process_in_place(&mut self, buf: &mut [f32], dt: f32)`; `pub struct MasterEq` with `new(freq, gain_db, q, eq_type: u8)`, `set_params(&mut self, freq, gain_db, q, eq_type: u8)`, `process(&mut self, l: &mut [f32], r: &mut [f32], dt: f32)`.

- [ ] **Step 1: Add `Eq::process_in_place`.** Inside `impl Eq` in `eq.rs` (right after the existing `process`), add an in-place variant (mirrors `process` but reads+writes `buf` at the same index — tick reads `buf[i]` before writing it, so in-place is safe):

```rust
    /// Like `process`, but filters `buf` in place (reads index `i` before writing
    /// it — the biquad state is internal, so same-index in-place is safe). Used by
    /// the stereo `MasterEq` on the root bus.
    pub fn process_in_place(&mut self, buf: &mut [f32], dt: f32) {
        let (b0, b1, b2, a1, a2) = rbj_coeffs(self.ty, self.freq, self.gain, self.q, dt);
        self.biquad.set_coeffs(b0, b1, b2, a1, a2);
        for i in 0..buf.len() {
            buf[i] = self.biquad.tick(buf[i]);
        }
    }
```

(Confirm the exact `rbj_coeffs(...)` argument order + return tuple against the existing `Eq::process` — copy it verbatim, only changing the loop to in-place.)

- [ ] **Step 2: Add `MasterEq`.** After the `impl Eq` block (and after the existing `impl Default for Eq`), add:

```rust
/// Stereo master EQ: two independent one-band `Eq`s (L/R) sharing the same
/// params. Applied to the render root bus between the DC-block and the limiter.
pub struct MasterEq {
    ch: [Eq; 2], // [L, R]
}

impl MasterEq {
    pub fn new(freq: f32, gain_db: f32, q: f32, eq_type: u8) -> MasterEq {
        let mut m = MasterEq { ch: [Eq::new(EqType::Peak); 2] };
        m.set_params(freq, gain_db, q, eq_type);
        m
    }
    /// Set both channels' band params. Finite-guarded (NaN/inf → a safe default)
    /// before `Eq`'s own range-clamps, so a bad param never poisons the output.
    pub fn set_params(&mut self, freq: f32, gain_db: f32, q: f32, eq_type: u8) {
        let freq = if freq.is_finite() { freq } else { 1000.0 };
        let gain_db = if gain_db.is_finite() { gain_db } else { 0.0 };
        let q = if q.is_finite() { q } else { 0.707 };
        for e in self.ch.iter_mut() {
            e.set_type(eq_type);
            e.set_freq(freq);
            e.set_gain(gain_db);
            e.set_q(q);
        }
    }
    /// EQ `l`/`r` in place. `dt` = 1/sample_rate (guarded — a non-finite dt would
    /// otherwise poison the coeffs on the master output).
    pub fn process(&mut self, l: &mut [f32], r: &mut [f32], dt: f32) {
        let dt = if dt.is_finite() && dt > 0.0 { dt } else { 1.0 / 48_000.0 };
        self.ch[0].process_in_place(l, dt);
        self.ch[1].process_in_place(r, dt);
    }
}
```

(`Eq` derives `Clone, Copy`, so `[Eq::new(EqType::Peak); 2]` is valid.)

- [ ] **Step 3: Write the failing tests.** In `eq.rs`'s `#[cfg(test)] mod tests` (add `use super::*;` / `extern crate std;` at the top if absent — the tests below use only fixed arrays + `libm`, so `std` is not required):

```rust
    fn tail_peak(b: &[f32]) -> f32 {
        b[b.len() / 2..].iter().fold(0.0f32, |m, &x| m.max(x.abs()))
    }

    #[test]
    fn mastereq_peak_boost_amplifies() {
        const DT: f32 = 1.0 / 48_000.0;
        let f = 1000.0f32;
        let mut l = [0.0f32; 8192];
        let mut r = [0.0f32; 8192];
        for i in 0..8192 {
            let s = 0.2 * libm::sinf(core::f32::consts::TAU * f * (i as f32) * DT);
            l[i] = s;
            r[i] = s;
        }
        let in_peak = tail_peak(&l);
        let mut eq = MasterEq::new(f, 12.0, 2.0, 0); // +12 dB peak at 1 kHz
        eq.process(&mut l, &mut r, DT);
        let out_peak = tail_peak(&l);
        assert!(out_peak > in_peak * 1.5, "peak boost should amplify: in={} out={}", in_peak, out_peak);
        assert!((tail_peak(&l) - tail_peak(&r)).abs() < 1e-4, "L/R same response");
    }

    #[test]
    fn mastereq_cut_attenuates() {
        const DT: f32 = 1.0 / 48_000.0;
        let f = 1000.0f32;
        let mut l = [0.0f32; 8192];
        let mut r = [0.0f32; 8192];
        for i in 0..8192 {
            let s = 0.2 * libm::sinf(core::f32::consts::TAU * f * (i as f32) * DT);
            l[i] = s;
            r[i] = s;
        }
        let in_peak = tail_peak(&l);
        let mut eq = MasterEq::new(f, -12.0, 2.0, 0); // -12 dB cut at 1 kHz
        eq.process(&mut l, &mut r, DT);
        assert!(tail_peak(&l) < in_peak * 0.7, "cut should attenuate");
    }

    #[test]
    fn mastereq_flat_passes_approximately() {
        const DT: f32 = 1.0 / 48_000.0;
        let f = 1000.0f32;
        let mut l = [0.0f32; 8192];
        let mut r = [0.0f32; 8192];
        for i in 0..8192 {
            let s = 0.2 * libm::sinf(core::f32::consts::TAU * f * (i as f32) * DT);
            l[i] = s;
            r[i] = s;
        }
        let in_peak = tail_peak(&l);
        let mut eq = MasterEq::new(f, 0.0, 2.0, 0); // flat (0 dB peak)
        eq.process(&mut l, &mut r, DT);
        assert!((tail_peak(&l) - in_peak).abs() < in_peak * 0.1, "flat EQ ~unchanged");
    }

    #[test]
    fn mastereq_no_panic_on_adversarial_params() {
        let mut eq = MasterEq::new(f32::NAN, f32::INFINITY, -1.0, 99);
        let mut l = [0.5f32; 16];
        let mut r = [0.5f32; 16];
        eq.process(&mut l, &mut r, 1.0 / 48_000.0);
        eq.set_params(f32::NAN, f32::NAN, f32::NAN, 1);
        eq.process(&mut l, &mut r, f32::NAN); // dt guarded → finite
        assert!(l[15].is_finite(), "must stay finite: {}", l[15]);
    }
```

- [ ] **Step 4: Run the tests, both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels mastereq
cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd mastereq
```
Expected: all 4 PASS both configs. (If the `eq.rs` test module lacks `use super::*;`/`extern crate std;`, add them; `libm` is a crate dep.)

- [ ] **Step 5: Full kernels crate, both configs.**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels` and again `--features simd`. Expected: green (existing `Eq`/`Kind::Eq` tests unaffected — `Eq::process` untouched).

- [ ] **Step 6: Commit.**

```bash
git add crates/deluge-dsp-kernels/src/eq.rs
git commit -m "feat(kernels): MasterEq — stereo single-band EQ (two Eq) + Eq::process_in_place"
```

---

### Task 2: Engine wiring (`Cmd::SetMasterEq` + render seam)

**Files:**
- Modify: `crates/deluge-audio-graph/src/cmd.rs` (add variant after `SetMasterDcBlock`)
- Modify: `crates/deluge-audio-graph/src/engine.rs` (import; `master_eq` field; `new`; `apply`; `Reset`; render seam)
- Test: inline `#[cfg(test)]` in `engine.rs`

**Interfaces:**
- Consumes: `MasterEq::{new, set_params, process}` (Task 1).
- Produces: `Cmd::SetMasterEq { freq: f32, gain_db: f32, q: f32, eq_type: u8 }`; field `master_eq: Option<MasterEq>` reset on `Cmd::Reset`.

- [ ] **Step 1: Add the `Cmd` variant.** In `cmd.rs`, after the `SetMasterDcBlock { cutoff_hz: f32 }` variant:

```rust
    SetMasterDcBlock { cutoff_hz: f32 },
    /// Enable/configure the master EQ on the root bus (between DC-block and limiter).
    /// Creates it if absent, else updates the band params in place.
    SetMasterEq { freq: f32, gain_db: f32, q: f32, eq_type: u8 },
    Free { node: NodeId },
```

- [ ] **Step 2: Import + add the field.** In `engine.rs`, near `use deluge_dsp_kernels::filter::MasterDcBlock;`:

```rust
use deluge_dsp_kernels::eq::MasterEq;
```

And add the field next to `master_dcblock`:

```rust
    master_dcblock: Option<MasterDcBlock>,
    // Opt-in master EQ on the root bus, applied at the render seam between the
    // DC-block and the limiter. `None` = disabled (render path byte-unchanged).
    master_eq: Option<MasterEq>,
```

- [ ] **Step 3: Init in `new`.** Next to `master_dcblock: None,`:

```rust
            master_dcblock: None,
            master_eq: None,
```

- [ ] **Step 4: Clear on Reset.** In the `Cmd::Reset` arm, after `self.master_dcblock = None;`:

```rust
                self.master_dcblock = None;
                self.master_eq = None;
```

- [ ] **Step 5: Add the apply arm.** After the `Cmd::SetMasterDcBlock { .. } => ...` arm:

```rust
            Cmd::SetMasterEq { freq, gain_db, q, eq_type } => match &mut self.master_eq {
                Some(eq) => eq.set_params(freq, gain_db, q, eq_type),
                None => self.master_eq = Some(MasterEq::new(freq, gain_db, q, eq_type)),
            },
```

- [ ] **Step 6: Insert the EQ in the render seam (between DC-block and limiter).** In the master-chain block, add the EQ line between the `master_dcblock` and `master_limiter` blocks:

```rust
        if let Some(root) = self.root {
            let b = root.0 as usize;
            if let Some(dc) = &mut self.master_dcblock {
                dc.process(&mut self.bus_l[b], &mut self.bus_r[b]);
            }
            if let Some(eq) = &mut self.master_eq {
                eq.process(&mut self.bus_l[b], &mut self.bus_r[b], self.dt);
            }
            if let Some(lim) = &mut self.master_limiter {
                lim.process(&mut self.bus_l[b], &mut self.bus_r[b], self.dt);
            }
        }
```

- [ ] **Step 7: Write the failing engine tests.** In `engine.rs` tests (`type E = Engine<16, 8, 8, 4, 45056, 2048>;`, `use crate::Cmd;`). Note BLOCK=16 is short for a biquad to settle at low freqs, so use a high-Q peak at a mid freq and render many blocks; the disabled/reset tests use a DC-ish const (any signal works):

```rust
    #[test]
    fn master_eq_changes_render_when_enabled() {
        // A +12 dB peak at 2 kHz should raise the level of a 2 kHz-ish content vs
        // the un-EQ'd render. Simplest robust check: enabling the EQ changes the
        // output (a boost makes a nonzero signal larger). Use a const 0.2 source
        // (broadband DC step) and assert the enabled render differs from disabled.
        let build = |eq: bool| -> f32 {
            let mut e = E::new(48_000.0);
            e.create(NodeId(0), Kind::Add);
            *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.2);
            *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
            e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
            e.set_root(BusId(0));
            if eq {
                e.apply(Cmd::SetMasterEq { freq: 2000.0, gain_db: 12.0, q: 2.0, eq_type: 0 });
            }
            let mut out = [StereoFrame::default(); 16];
            let sil = [StereoFrame::default(); 16];
            for _ in 0..64 { e.render(&mut out, &sil); }
            out[15].l
        };
        let off = build(false);
        let on = build(true);
        assert!((on - off).abs() > 1e-4, "EQ should change the render (off={}, on={})", off, on);
    }

    #[test]
    fn master_eq_disabled_is_byte_identical() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6); // no EQ → 0.5 passes
    }

    #[test]
    fn master_eq_reset_clears() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::SetMasterEq { freq: 2000.0, gain_db: 12.0, q: 2.0, eq_type: 0 });
        e.apply(Cmd::Reset);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6); // EQ cleared → 0.5 passes
    }
```

- [ ] **Step 8: Run engine tests, then full graph crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph master_eq
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph
cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features deluge-dsp-kernels/simd
```
Expected: 3 new tests PASS; full crate green both configs (existing `master_dcblock_*`/`master_limiter_*` tests still pass — the seam adds a line, behavior-preserving when `master_eq` is `None`).

- [ ] **Step 9: Confirm dependent crates still build.**

Run: `cargo build --target x86_64-unknown-linux-gnu -p deluge-wren-core` and `cargo build -p wren-firmware --target armv7a-none-eabihf`. Expected: clean.

- [ ] **Step 10: Commit.**

```bash
git add crates/deluge-audio-graph/src/cmd.rs crates/deluge-audio-graph/src/engine.rs
git commit -m "feat(graph): Cmd::SetMasterEq + master EQ between DC-block and limiter"
```

---

### Task 3: Wren `Out.eq(...)` binding

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (add `set_master_eq` near `set_master_dcblock`)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_master_eq_impl` + shim near `node_master_dcblock`; `register_audio` entry)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (`METHODS`)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`foreign static masterEq_`; `class Out` methods)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Cmd::SetMasterEq { freq, gain_db, q, eq_type }` (Task 2).
- Produces: Wren `Out.eq(freq, gain, q)` / `Out.eqLowShelf(...)` / `Out.eqHighShelf(...)`.

- [ ] **Step 1: Write the failing Cmd-capture tests.** In `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn out_eq_peak_emits_set_master_eq() {
    let cmds = run_and_capture_cmds("Out.eq(1000, 6, 1.0)");
    assert_eq!(cmds, vec![Cmd::SetMasterEq { freq: 1000.0, gain_db: 6.0, q: 1.0, eq_type: 0 }]);
}

#[test]
fn out_eq_low_shelf_emits_type_1() {
    let cmds = run_and_capture_cmds("Out.eqLowShelf(200, -3, 0.7)");
    assert_eq!(cmds, vec![Cmd::SetMasterEq { freq: 200.0, gain_db: -3.0, q: 0.7, eq_type: 1 }]);
}

#[test]
fn out_eq_high_shelf_emits_type_2() {
    let cmds = run_and_capture_cmds("Out.eqHighShelf(8000, 4, 0.7)");
    assert_eq!(cmds, vec![Cmd::SetMasterEq { freq: 8000.0, gain_db: 4.0, q: 0.7, eq_type: 2 }]);
}
```

- [ ] **Step 2: Run to verify fail.**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support --test audio_bindings out_eq`
Expected: FAIL (Wren `Out.eq` / `Node.masterEq_` unknown).

- [ ] **Step 3: Add the facade.** In `crates/deluge-wren-core/src/audio.rs`, near `set_master_dcblock`:

```rust
pub fn set_master_eq(freq: f32, gain_db: f32, q: f32, eq_type: u8) {
    host().audio_cmd(Cmd::SetMasterEq { freq, gain_db, q, eq_type });
}
```

- [ ] **Step 4: Add the binding impl + shim.** In `crates/deluge-wren-core/src/bindings_audio.rs`, near `node_master_dcblock_impl`:

```rust
pub(crate) fn node_master_eq_impl<S: SlotApi>(vm: &S) {
    let freq = vm.get_f(1) as f32;
    let gain_db = vm.get_f(2) as f32;
    let q = vm.get_f(3) as f32;
    let eq_type = vm.get_f(4) as u8;
    audio::set_master_eq(freq, gain_db, q, eq_type);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_master_eq(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_master_eq_impl(&vm);
}
```

- [ ] **Step 5: Register in `register_audio`.** In `bindings_audio.rs` next to `masterDcBlock_`:

```rust
    method("main", "Node", true, "masterEq_(_,_,_,_)", node_master_eq_impl::<S>);
```

- [ ] **Step 6: Register in `METHODS`.** In `crates/deluge-wren-core/src/bindings.rs` next to `masterDcBlock_`:

```rust
    static_method("Node", "masterEq_(_,_,_,_)", bindings_audio::node_master_eq),
```

- [ ] **Step 7: Add the prelude decls.** In `crates/deluge-wren-core/wren/prelude.wren`, add to `class Node` (after `foreign static masterDcBlock_(cutoff)`):

```wren
  foreign static masterEq_(freq, gain, q, type)
```

And extend `class Out` (after the `dcBlock` methods) — single-line **implicit** return:

```wren
  static eq(freq, gain, q) { Node.masterEq_(freq, gain, q, 0) }
  static eqLowShelf(freq, gain, q) { Node.masterEq_(freq, gain, q, 1) }
  static eqHighShelf(freq, gain, q) { Node.masterEq_(freq, gain, q, 2) }
```

- [ ] **Step 8: Run the Cmd-capture tests, then full crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support --test audio_bindings out_eq
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd
```
Expected: all three `out_eq_*` PASS; full crate green both configs.

- [ ] **Step 9: Cross-build check.**

Run: `cargo build -p wren-firmware --target armv7a-none-eabihf`. Expected: clean.

- [ ] **Step 10: Commit.**

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs \
        crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren \
        crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): Out.eq / eqLowShelf / eqHighShelf master EQ binding"
```

---

### Task 4: e2e — a tone EQ'd by `Out.eq`

**Files:**
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Out.eq(...)` (Task 3) + the real-`EngineHost` render harness (`run_and_render` fills successive 32-frame chunks; a larger `out` gives settle time).

- [ ] **Step 1: Write the failing e2e test.** In `crates/deluge-wren-core/tests/audio_bindings.rs`, use `run_and_render` (renders successive 32-frame chunks to fill the whole buffer — a large buffer settles the biquad). A saw has harmonic content, so a peak boost raises its level; assert the EQ-boosted render has a larger peak than the un-EQ'd one:

```rust
#[test]
fn out_eq_boost_raises_level() {
    let mut base = [StereoFrame::default(); 2048];
    run_and_render("Out.patch(Osc.saw(220) * 0.2)", &mut base);
    let base_peak = base[1024..].iter().fold(0.0f32, |m, f| m.max(f.l.abs()));

    let mut eqd = [StereoFrame::default(); 2048];
    run_and_render("Out.patch(Osc.saw(220) * 0.2)\nOut.eq(2000, 18, 2.0)", &mut eqd);
    let eq_peak = eqd[1024..].iter().fold(0.0f32, |m, f| m.max(f.l.abs()));

    assert!(eqd.iter().all(|f| f.l.is_finite() && f.r.is_finite()), "finite");
    assert!(eq_peak > base_peak * 1.1, "EQ boost should raise the level: base={} eq={}", base_peak, eq_peak);
}
```

If the specific freq/gain don't produce a clear boost through the saw's spectrum + the harness's sample rate (TestEng is 44.1 kHz), adjust the EQ freq to sit on a strong saw harmonic (a 220 Hz saw has harmonics at 440, 660, 880, …; e.g. `Out.eq(880, 18, 4.0)`) so the boost is unambiguous. Do NOT weaken to always-true; the point is to show the EQ audibly boosts end-to-end.

- [ ] **Step 2: Run it, then full crate both configs.**

Run:
```
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support --test audio_bindings out_eq_boost
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core
cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd
```
Expected: PASS; full crate green both configs.

- [ ] **Step 3: Commit.**

```bash
git add crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "test(wren): e2e — Out.eq boost raises a saw's level"
```

---

## Self-Review

**Spec coverage:** `MasterEq` kernel (two `Eq`, finite-guarded, in-place via `Eq::process_in_place`) → Task 1. Engine `Option<MasterEq>` + `Cmd::SetMasterEq` + render seam (DC→EQ→limiter) + Reset clear → Task 2. `Out.eq`/`eqLowShelf`/`eqHighShelf` both tables + shim + prelude, type 0/1/2 → Task 3. e2e boost → Task 4. All spec sections covered.

**Placeholder scan:** Task 4 notes "adjust the EQ freq to a strong saw harmonic if the boost isn't clear" — a concrete tuning instruction with a worked example (`Out.eq(880, 18, 4.0)`), not a gap. All kernel/engine/binding steps carry exact code.

**Type consistency:** `MasterEq::{new, set_params, process}` (Task 1) consumed with those exact names in Task 2's apply arm + seam. `Cmd::SetMasterEq { freq: f32, gain_db: f32, q: f32, eq_type: u8 }` (Task 2) emitted identically by `audio::set_master_eq` (Task 3) and asserted in the Task 3 tests. Type codes `0/1/2` consistent across the prelude methods, the Task 3 tests, and `Eq::set_type`.
