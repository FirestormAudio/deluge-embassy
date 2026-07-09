# Ef-3a Room Reverb (Freeverb) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Schroeder-Moorer (freeverb-topology) room reverb — `Comb`/`Allpass` primitives + a `Freeverb` engine over one partitioned pooled buffer — as a mono→stereo Wren `Room.new` node.

**Architecture:** A new `deluge-dsp-kernels/src/reverb.rs` with `Comb` (damped-feedback comb) and `Allpass` (Schroeder diffuser) primitives and a `Freeverb` engine (8 combs + 4 allpasses per channel, +23-sample stereospread on the right). The reverb node binds ONE big pool region (`REVERB_BUF_SAMPLES`); the kernel carves it by walking a fixed length layout and each `tick` indexes `buf[off + cursor]`. `Kind::Room` is a width-2 stereo node routed by the merged width-aware `Out.patch`. Implemented from the public Schroeder-Moorer structure — NOT the GPL freeverb source.

**Tech Stack:** Rust, `no_std`/no-heap, Wren via `wren-sys`, proptest.

## Global Constraints

- **`no_std`, no heap, pure `f32`, deterministic.** The kernel owns NO delay storage — the buffer is a borrowed `&mut [f32]` passed per call. No `f32::fract()` etc.
- **Implemented from the public Schroeder-Moorer structure**, MIT/Apache — do NOT read or port the GPL `freeverb` C++. Standard tunings only.
- **Tunings (samples @ 44.1 kHz):** combs `[1116,1188,1277,1356,1422,1491,1557,1617]`, allpasses `[556,441,341,225]`, `SPREAD=23`, input `GAIN=0.015`, allpass feedback `0.5`, comb `feedback = roomsize·0.28 + 0.7`, comb damp coeff `= damp·0.4`.
- **`Comb::tick`:** `out = buf[off+c]; damp_z = out·(1−damp) + damp_z·damp; buf[off+c] = x + damp_z·feedback; advance c (mod len); return out`.
- **`Allpass::tick`:** `bufout = buf[off+c]; out = -x + bufout; buf[off+c] = x + bufout·0.5; advance c (mod len); return out`.
- **`Freeverb` output:** `in_g = x·GAIN`; `ol = Σ combs_l.tick`, then series `ol = aps_l.tick(ol)`; same for `or` on R lines. `wet1 = mix·(width·0.5+0.5); wet2 = mix·((1−width)·0.5); dry = 1−mix`. `out_l = x·dry + ol·wet1 + or·wet2; out_r = x·dry + or·wet1 + ol·wet2`.
- **`REVERB_BUF_SAMPLES = 25_450`** (Σ of all 24 line lengths: 8 L combs + 8 R combs(+23) + 4 L allpasses + 4 R allpasses(+23)). The kernel computes each line's offset by walking the length layout (running `off`), so the final `off` equals this const.
- **Buffer-length safety:** `Freeverb::process` dry-passes-through (both ports = input) if `buf.len() < REVERB_BUF_SAMPLES` — never index out of range. The graph render arm ALSO guards `buf.len() >= REVERB_BUF_SAMPLES`.
- **Param map:** `set_param` 0=mix, 1=damp, 2=roomsize, 3=width. `out_width(Kind::Room) = 2`.
- **Wren:** `Room.new(input, roomsize, damp, mix)` → width-2 node. Setters: `mix=` (reused → param 0), `damp=` (reused from Ef-1 Delay → param 1), **`size=`** (new → param 2), **`spread=`** (new → param 3 stereo width — NOT `width=`, the Osc's PWM port setter).
- Host tests run with `--target x86_64-unknown-linux-gnu`.

---

### Task 1: `reverb.rs` kernel — `Comb`, `Allpass`, `Freeverb`

**Files:**
- Create: `crates/deluge-dsp-kernels/src/reverb.rs`
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (add `pub mod reverb;`)

**Interfaces:**
- Consumes: `crate::In`.
- Produces: `pub struct Comb`/`Allpass`/`Freeverb`; `Freeverb::new()`, `set_mix`/`set_damp`/`set_roomsize`/`set_width`, `process(&mut self, input: In, dt: f32, buf: &mut [f32], out_l: &mut [f32], out_r: &mut [f32])`; `pub const REVERB_BUF_SAMPLES: usize`. Task 2 consumes these.

- [ ] **Step 1: Write the failing tests**

Create `crates/deluge-dsp-kernels/src/reverb.rs`:

```rust
//! Schroeder-Moorer (freeverb-topology) room reverb: damped-feedback `Comb` +
//! Schroeder `Allpass` primitives over a borrowed, partitioned ring buffer, and
//! a `Freeverb` engine (8 combs + 4 allpasses per channel, stereospread on the
//! right). Implemented from the public algorithm structure — not the GPL
//! freeverb source. `no_std`, no heap.

use crate::In;

const COMB: [usize; 8] = [1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617];
const AP: [usize; 4] = [556, 441, 341, 225];
const SPREAD: usize = 23;
const GAIN: f32 = 0.015;

/// Total samples the shared buffer must hold (Σ of all 24 line lengths).
pub const REVERB_BUF_SAMPLES: usize = 25_450;

/// Damped-feedback comb. Fixed integer delay = the slice length; cursor + one-pole
/// damp state. `off`/`len` locate this comb's slice in the shared buffer.
#[derive(Clone, Copy)]
pub struct Comb {
    c: usize,
    damp_z: f32,
}
impl Comb {
    pub fn new() -> Comb {
        Comb { c: 0, damp_z: 0.0 }
    }
    pub fn tick(&mut self, buf: &mut [f32], off: usize, len: usize, x: f32, feedback: f32, damp: f32) -> f32 {
        let out = buf[off + self.c];
        self.damp_z = out * (1.0 - damp) + self.damp_z * damp;
        buf[off + self.c] = x + self.damp_z * feedback;
        self.c += 1;
        if self.c >= len {
            self.c = 0;
        }
        out
    }
}

/// Schroeder allpass diffuser (fixed feedback 0.5). Cursor only.
#[derive(Clone, Copy)]
pub struct Allpass {
    c: usize,
}
impl Allpass {
    pub fn new() -> Allpass {
        Allpass { c: 0 }
    }
    pub fn tick(&mut self, buf: &mut [f32], off: usize, len: usize, x: f32) -> f32 {
        let bufout = buf[off + self.c];
        let out = -x + bufout;
        buf[off + self.c] = x + bufout * 0.5;
        self.c += 1;
        if self.c >= len {
            self.c = 0;
        }
        out
    }
}

/// Freeverb room reverb: 8 combs + 4 allpasses per channel over a shared
/// partitioned buffer; mono in → stereo out.
#[derive(Clone, Copy)]
pub struct Freeverb {
    combs_l: [Comb; 8],
    combs_r: [Comb; 8],
    aps_l: [Allpass; 4],
    aps_r: [Allpass; 4],
    roomsize: f32,
    damp: f32,
    width: f32,
    mix: f32,
}
impl Freeverb {
    pub fn new() -> Freeverb {
        Freeverb {
            combs_l: [Comb::new(); 8],
            combs_r: [Comb::new(); 8],
            aps_l: [Allpass::new(); 4],
            aps_r: [Allpass::new(); 4],
            roomsize: 0.5,
            damp: 0.5,
            width: 1.0,
            mix: 0.5,
        }
    }
    pub fn set_mix(&mut self, v: f32) { self.mix = v.clamp(0.0, 1.0); }
    pub fn set_damp(&mut self, v: f32) { self.damp = v.clamp(0.0, 1.0); }
    pub fn set_roomsize(&mut self, v: f32) { self.roomsize = v.clamp(0.0, 1.0); }
    pub fn set_width(&mut self, v: f32) { self.width = v.clamp(0.0, 1.0); }

    pub fn process(&mut self, input: In, _dt: f32, buf: &mut [f32], out_l: &mut [f32], out_r: &mut [f32]) {
        // Real-time safety: a too-small region → dry passthrough, never panic.
        if buf.len() < REVERB_BUF_SAMPLES {
            for i in 0..out_l.len() {
                let x = input.at(i);
                out_l[i] = x;
                out_r[i] = x;
            }
            return;
        }
        let feedback = self.roomsize * 0.28 + 0.7;
        let dc = self.damp * 0.4;
        let wet1 = self.mix * (self.width * 0.5 + 0.5);
        let wet2 = self.mix * ((1.0 - self.width) * 0.5);
        let dry = 1.0 - self.mix;
        for i in 0..out_l.len() {
            let x = input.at(i);
            let in_g = x * GAIN;
            let mut ol = 0.0;
            let mut or = 0.0;
            let mut off = 0usize;
            for k in 0..8 {
                let len = COMB[k];
                ol += self.combs_l[k].tick(buf, off, len, in_g, feedback, dc);
                off += len;
            }
            for k in 0..8 {
                let len = COMB[k] + SPREAD;
                or += self.combs_r[k].tick(buf, off, len, in_g, feedback, dc);
                off += len;
            }
            for k in 0..4 {
                let len = AP[k];
                ol = self.aps_l[k].tick(buf, off, len, ol);
                off += len;
            }
            for k in 0..4 {
                let len = AP[k] + SPREAD;
                or = self.aps_r[k].tick(buf, off, len, or);
                off += len;
            }
            out_l[i] = x * dry + ol * wet1 + or * wet2;
            out_r[i] = x * dry + or * wet1 + ol * wet2;
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;

    fn render(fv: &mut Freeverb, buf: &mut [f32], input: &[f32]) -> (std::vec::Vec<f32>, std::vec::Vec<f32>) {
        let mut l = std::vec![0.0f32; input.len()];
        let mut r = std::vec![0.0f32; input.len()];
        fv.process(In::A(input), 1.0 / 44_100.0, buf, &mut l, &mut r);
        (l, r)
    }

    #[test]
    fn layout_sums_to_reverb_buf_samples() {
        let combs: usize = COMB.iter().sum::<usize>() + COMB.iter().map(|c| c + SPREAD).sum::<usize>();
        let aps: usize = AP.iter().sum::<usize>() + AP.iter().map(|a| a + SPREAD).sum::<usize>();
        assert_eq!(combs + aps, REVERB_BUF_SAMPLES);
    }

    #[test]
    fn impulse_produces_decaying_diffuse_tail() {
        let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
        let mut fv = Freeverb::new();
        fv.set_mix(1.0);
        fv.set_roomsize(0.7);
        let mut input = std::vec![0.0f32; 20_000];
        input[0] = 1.0;
        let (l, r) = render(&mut fv, &mut buf, &input);
        assert!(l.iter().all(|v| v.is_finite()) && r.iter().all(|v| v.is_finite()));
        // Non-silent well after the impulse (recirculating tail).
        let late_energy: f32 = l[10_000..12_000].iter().map(|v| v * v).sum();
        assert!(late_energy > 1e-6, "tail died too fast: {late_energy}");
        // Decaying: an earlier window has more energy than a later one.
        let early: f32 = l[2_000..4_000].iter().map(|v| v * v).sum();
        let later: f32 = l[14_000..16_000].iter().map(|v| v * v).sum();
        assert!(early > later, "tail not decaying: {early} → {later}");
    }

    #[test]
    fn roomsize_lengthens_tail() {
        let tail = |size: f32| -> f32 {
            let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
            let mut fv = Freeverb::new();
            fv.set_mix(1.0);
            fv.set_roomsize(size);
            let mut input = std::vec![0.0f32; 30_000];
            input[0] = 1.0;
            let (l, _r) = render(&mut fv, &mut buf, &input);
            l[24_000..26_000].iter().map(|v| v * v).sum()
        };
        assert!(tail(0.9) > tail(0.3), "bigger room should decay slower");
    }

    #[test]
    fn damping_darkens_tail() {
        let hf = |damp: f32| -> f32 {
            let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
            let mut fv = Freeverb::new();
            fv.set_mix(1.0);
            fv.set_roomsize(0.8);
            fv.set_damp(damp);
            let mut input = std::vec![0.0f32; 20_000];
            input[0] = 1.0;
            let (l, _r) = render(&mut fv, &mut buf, &input);
            l[8_000..12_000].windows(2).map(|w| (w[1] - w[0]).powi(2)).sum()
        };
        assert!(hf(0.9) < hf(0.05), "more damping → less HF in the tail");
    }

    #[test]
    fn output_is_stereo() {
        let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
        let mut fv = Freeverb::new();
        fv.set_mix(1.0);
        let input: std::vec::Vec<f32> = (0..8_000).map(|i| (i as f32 * 0.03).sin()).collect();
        let (l, r) = render(&mut fv, &mut buf, &input);
        let diff: f32 = l.iter().zip(&r).map(|(a, b)| (a - b).abs()).sum();
        assert!(diff > 1.0, "reverb should be stereo (l != r): {diff}");
    }

    #[test]
    fn mix_zero_is_dry() {
        let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
        let mut fv = Freeverb::new();
        fv.set_mix(0.0);
        let input = std::vec![0.5f32; 256];
        let (l, r) = render(&mut fv, &mut buf, &input);
        assert!((l[100] - 0.5).abs() < 1e-4 && (r[100] - 0.5).abs() < 1e-4);
    }

    #[test]
    fn short_buffer_is_dry_passthrough() {
        let mut buf = std::vec![0.0f32; 64]; // < REVERB_BUF_SAMPLES
        let mut fv = Freeverb::new();
        fv.set_mix(1.0);
        let input = std::vec![0.3f32; 32];
        let (l, r) = render(&mut fv, &mut buf, &input);
        assert!(l.iter().all(|&v| (v - 0.3).abs() < 1e-6) && r.iter().all(|&v| (v - 0.3).abs() < 1e-6));
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 32, ..ProptestConfig::default() })]
        #[test]
        fn freeverb_is_finite_and_bounded(
            roomsize in 0.0f32..1.0,
            damp in 0.0f32..1.0,
            width in 0.0f32..1.0,
            mix in 0.0f32..1.0,
            amp in 0.0f32..1.0,
        ) {
            let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
            let mut fv = Freeverb::new();
            fv.set_roomsize(roomsize); fv.set_damp(damp); fv.set_width(width); fv.set_mix(mix);
            let input = std::vec![amp; 4_000];
            let (l, r) = render(&mut fv, &mut buf, &input);
            for (a, b) in l.iter().zip(&r) {
                prop_assert!(a.is_finite() && b.is_finite());
                prop_assert!(a.abs() <= 16.0 && b.abs() <= 16.0, "unbounded: {a},{b}");
            }
        }
    }
}
```

- [ ] **Step 2: Register the module**

In `crates/deluge-dsp-kernels/src/lib.rs`, add `pub mod reverb;` after `pub mod osc;` (keep the list ordered):

```rust
pub mod osc;
pub mod reverb;
pub mod wavetable;
```

- [ ] **Step 3: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels reverb`
Expected: PASS (8 tests incl. the layout-sum check and the boundedness proptest), no warnings.

(If `layout_sums_to_reverb_buf_samples` fails, the tuning constants and `REVERB_BUF_SAMPLES` disagree — do NOT change the assertion; recompute the const from the actual `COMB`/`AP`/`SPREAD` sum and report.)

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-dsp-kernels/src/reverb.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(dsp-kernels): Freeverb room reverb (Comb/Allpass + partitioned buffer)"
```

---

### Task 2: Graph — `Kind::Room`

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs` (import, Kind, State, `Node::new`, `out_width`, `set_param`, render arm, tests)

**Interfaces:**
- Consumes: `deluge_dsp_kernels::reverb::{Freeverb, REVERB_BUF_SAMPLES}`.
- Produces: `Kind::Room`. Task 3's factory creates it.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-audio-graph/src/node.rs`'s `mod tests`:

```rust
    #[test]
    fn room_node_renders_stereo_with_buffer() {
        use deluge_dsp_kernels::reverb::REVERB_BUF_SAMPLES;
        let mut n = Node::new(Kind::Room, 0);
        assert_eq!(Node::out_width(Kind::Room), 2);
        n.set_param(0, 1.0); // mix wet
        n.set_param(2, 0.7); // roomsize
        let input: [f32; 64] = core::array::from_fn(|i| if i == 0 { 1.0 } else { 0.0 });
        let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut ring = std::vec![0.0f32; REVERB_BUF_SAMPLES];
        let mut p0 = [0.0f32; 64];
        let mut p1 = [0.0f32; 64];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 44_100.0, &mut outs, Some(&mut ring));
        }
        assert!(p0.iter().all(|s| s.is_finite() && s.abs() <= 16.0));
        assert!(p1.iter().all(|s| s.is_finite() && s.abs() <= 16.0));
    }

    #[test]
    fn room_node_without_buffer_is_dry_both_ports() {
        let mut n = Node::new(Kind::Room, 0);
        let input = [0.4f32; 16];
        let ins = [In::A(&input), In::A(&[0.0; 16]), In::A(&[0.0; 16])];
        let mut p0 = [0.0f32; 16];
        let mut p1 = [0.0f32; 16];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 44_100.0, &mut outs, None);
        }
        assert!(p0.iter().all(|&s| (s - 0.4).abs() < 1e-6));
        assert!(p1.iter().all(|&s| (s - 0.4).abs() < 1e-6));
    }
```

(The `node.rs` test module already uses `std` via other tests — if `std::vec!` is not in scope, add `extern crate std;` at the top of `mod tests`.)

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph room`
Expected: FAIL — no variant `Room`.

- [ ] **Step 3: Add the variant, state, construction, params**

In `crates/deluge-audio-graph/src/node.rs`:

Extend the kernel import (the file already imports `delay::{Delay, ModDelay}`):

```rust
use deluge_dsp_kernels::reverb::{Freeverb, REVERB_BUF_SAMPLES};
```

Add to `enum Kind` (after `Flanger`):

```rust
    Flanger,
    Room,
```

Add to `enum State` (after `Flanger(ModDelay<1>)`):

```rust
    Room(Freeverb),
```

In `Node::new`, add an arm (after the `Kind::Flanger => …` arm):

```rust
            Kind::Room => State::Room(Freeverb::new()),
```

In `out_width`, extend the width-2 arm:

```rust
            Kind::Split2 | Kind::Pan | Kind::Chorus | Kind::Flanger | Kind::Room => 2,
```

In `set_param`, add an arm (alongside `State::Chorus`/`Flanger`):

```rust
            State::Room(fv) => match param {
                0 => fv.set_mix(value),
                1 => fv.set_damp(value),
                2 => fv.set_roomsize(value),
                3 => fv.set_width(value),
                _ => {}
            },
```

- [ ] **Step 4: Add the render arm**

In `process_resolved`, add after the `Kind::Chorus | Kind::Flanger` arm:

```rust
            Kind::Room => {
                // Mono→stereo reverb. Bound region must hold the full partitioned
                // layout (≥ REVERB_BUF_SAMPLES); otherwise dry passthrough both.
                let (out_l, out_r) = outs.port_pair();
                let ran = if let Some(buf) = pool_region {
                    if buf.len() >= REVERB_BUF_SAMPLES {
                        if let State::Room(fv) = &mut self.state {
                            fv.process(ins[0], dt, buf, out_l, out_r);
                            true
                        } else {
                            false
                        }
                    } else {
                        false
                    }
                } else {
                    false
                };
                if !ran {
                    for i in 0..out_l.len() {
                        let x = ins[0].at(i);
                        out_l[i] = x;
                        out_r[i] = x;
                    }
                }
            }
```

- [ ] **Step 5: Run the new tests + full suite**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph room`
Expected: PASS (2 new tests).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: PASS — all existing tests too.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(audio-graph): Kind::Room reverb node"
```

---

### Task 3: Wren `Room.new` surface

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`REVERB_BUF_SAMPLES` const, `node_room_impl`, `node_set_size_impl`/`node_set_spread_impl` + wrappers, register)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (wren-sys registration)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`room_` + `size=`/`spread=` + `class Room`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Kind::Room` (Task 2), `audio::{alloc_buffer, new_pooled_node, alloc_node_id, set_param}`, `arg_input`, `return_node_w`.
- Produces: Wren `Room.new(input, roomsize, damp, mix)`, `size=`/`spread=`.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn room_new_emits_node_and_params_no_bind_on_capture_host() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var r = Room.new(Osc.saw(110), 0.7, 0.4, 0.5)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Room, .. })));
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::BindTable { .. })), "no pool → no BindTable: {cmds:?}");
    for p in [0u8, 1, 2] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, .. } if *param == p)), "missing SetParam {p}: {cmds:?}");
    }
}

#[test]
fn room_patch_routes_stereo() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds("Out.patch(Room.new(Osc.saw(110), 0.7, 0.4, 0.5))");
    let gains: std::vec::Vec<(f32, f32)> = cmds.iter().filter_map(|c| match c {
        Cmd::BusWriteGains { gl, gr, .. } => Some((*gl, *gr)),
        _ => None,
    }).collect();
    assert!(gains.contains(&(1.0, 0.0)) && gains.contains(&(0.0, 1.0)), "not stereo-routed: {gains:?}");
}

#[test]
fn room_size_and_spread_set_params_2_and_3() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds(
        "var r = Room.new(Osc.saw(110), 0.5, 0.5, 0.5)\nr.size = 0.9\nr.spread = 0.3",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 2, .. })), "size→2: {cmds:?}");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 3, .. })), "spread→3: {cmds:?}");
}

#[test]
fn room_renders_stereo_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Room.new(Osc.saw(110), 0.7, 0.4, 0.6))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite() && f.l.abs() <= 1.0 && f.r.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0 || f.r != 0.0), "reverb should be non-silent");
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings room`
Expected: FAIL — Wren `Room` undefined.

- [ ] **Step 3: Add the const, factory, and setters**

In `crates/deluge-wren-core/src/bindings_audio.rs`, add near the `CHORUS_BUF_SAMPLES` const:

```rust
/// Ring length for a Room reverb node. Keep in sync with
/// `deluge_dsp_kernels::reverb::REVERB_BUF_SAMPLES` (Σ of the 24 line lengths).
pub(crate) const REVERB_BUF_SAMPLES: usize = 25_450;
```

Add the factory (near `node_chorus_impl`):

```rust
/// `Node.room_(input, roomsize, damp, mix)` — allocate the partitioned reverb
/// buffer, create a width-2 `Kind::Room` node, and set roomsize/damp/mix. A host
/// with no pool leaves it unbound (dry passthrough).
pub(crate) fn node_room_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let roomsize = vm.get_f(2) as f32;
    let damp = vm.get_f(3) as f32;
    let mix = vm.get_f(4) as f32;
    let handle = audio::alloc_buffer(REVERB_BUF_SAMPLES);
    let id = audio::alloc_node_id();
    audio::new_pooled_node(id, Kind::Room, handle, input);
    audio::set_param(id, 2, roomsize);
    audio::set_param(id, 1, damp);
    audio::set_param(id, 0, mix);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_room(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_room_impl(&vm);
}
```

Add the setters (near `node_set_rate_impl`):

```rust
pub(crate) fn node_set_size_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 2 = roomsize (Kind::Room)
    audio::set_param(self_id(vm), 2, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_size(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_size_impl(&vm);
}

// `spread=` (stereo width, param 3) — NOT `width=` (the Osc's PWM port setter).
pub(crate) fn node_set_spread_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 3 = width (Kind::Room)
    audio::set_param(self_id(vm), 3, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_spread(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_spread_impl(&vm);
}
```

Register in `register_audio`:

```rust
    method("main", "Node", true, "room_(_,_,_,_)", node_room_impl::<S>);
    method("main", "Node", false, "size=(_)", node_set_size_impl::<S>);
    method("main", "Node", false, "spread=(_)", node_set_spread_impl::<S>);
```

(`mix=` and `damp=` are already registered — Room reuses them, do not re-register.)

- [ ] **Step 4: Register in the wren-sys table + prelude**

In `crates/deluge-wren-core/src/bindings.rs`, add near the other audio registrations:

```rust
    static_method("Node", "room_(_,_,_,_)", bindings_audio::node_room),
    method("Node", "size=(_)", bindings_audio::node_set_size),
    method("Node", "spread=(_)", bindings_audio::node_set_spread),
```

In `crates/deluge-wren-core/wren/prelude.wren`, add to `foreign class Node`:

```wren
  foreign static room_(input, roomsize, damp, mix)
  foreign size=(v)
  foreign spread=(v)     // Room stereo width (NOT width= — that's the Osc's PWM)
```

And the sugar class (after `class Flanger`):

```wren
// Room reverb (Schroeder-Moorer). `roomsize` [0,1] decay/size, `damp` [0,1]
// high-frequency absorption, `mix` dry/wet. Stereo (width-2):
//   Out.patch(Room.new(Osc.saw(110), 0.7, 0.4, 0.4))
//   var r = Room.new(pad, 0.8, 0.3, 0.5); r.size = 0.9; r.damp = 0.6; r.spread = 0.8
class Room {
  static new(input, roomsize, damp, mix) { Node.room_(input, roomsize, damp, mix) }
}
```

- [ ] **Step 5: Run the new tests + full suite**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings room`
Expected: PASS (all 4).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core`
Expected: PASS — existing bindings (Delay/Chorus/Pan/Resonator/wavetable, incl. the reused `mix=`/`damp=`) unchanged, no warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): Room.new reverb surface"
```

---

## Scope notes (deliberate deferrals)

- **Ef-3b Hall (FDN)** / **Ef-3c Plate (Dattorro)** — reuse `Comb`/`Allpass` + the partitioned-buffer pattern established here.
- **Sample-rate-scaled tunings** (fixed 44.1 kHz sample counts now), **pre-delay**, **tail modulation**, **freeze** — later.
- **`REVERB_BUF_SAMPLES` is duplicated** as a local const in the Wren layer (matching the `DELAY_MAX_SAMPLES`/`CHORUS_BUF_SAMPLES` pattern); the kernel's `reverb::REVERB_BUF_SAMPLES` is authoritative and the graph guard uses it. Keep them in sync.

## Post-implementation

After all three tasks pass, use **superpowers:finishing-a-development-branch** to verify the suites and merge.
