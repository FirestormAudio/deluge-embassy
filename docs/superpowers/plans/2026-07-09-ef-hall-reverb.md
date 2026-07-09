# Ef-3b Hall Reverb (FDN) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an 8-line modulated Feedback Delay Network hall reverb — a `Fdn8` engine (8 `DelayLine`s + normalized Hadamard feedback + per-line damping + light modulation) over one partitioned pooled buffer — as a mono→stereo Wren `Hall.new` node that reuses the Room param surface.

**Architecture:** `Fdn8` (in `deluge-dsp-kernels/src/reverb.rs`, alongside `Freeverb`) holds 8 `DelayLine`s over a partitioned buffer, mixes their damped taps through a fast Walsh-Hadamard transform (`fwht8`, orthonormal), scales by a decay gain `g < 1`, injects the input, and writes back. Even taps → L, odd → R. `Kind::Hall` mirrors `Kind::Room`. Hall reuses the Room Wren setters (`mix=`/`damp=`/`size=`/`spread=`) — only a `hall_` factory + `class Hall` are new.

**Tech Stack:** Rust, `no_std`/no-heap, `libm` (floorf), Wren via `wren-sys`, proptest.

## Global Constraints

- **`no_std`, no heap, pure `f32`, deterministic.** Kernel owns NO delay storage (buffer borrowed). LFO is a phase accumulator; no RNG. Wrap phase with `libm::floorf` (not `f32::fract()`); `fast_sin` needs its arg in `[0,1)`.
- **Constants (pinned):** `BASE_LEN = [1499, 1889, 2311, 2749, 3187, 3571, 3931, 4483]` (8 coprime primes); `MOD_MARGIN = 16` (per-line slice headroom); `MOD_DEPTH = 8.0`; `LFO_RATE = 0.7` (Hz); `INJ = 0.05` (input injection); `OUT = 1.0` (tap output gain); `HADAMARD_NORM = 0.353_553_39` (= 1/√8). Slice `k` length = `BASE_LEN[k] + MOD_MARGIN`.
- **`HALL_BUF_SAMPLES = 23_748`** = Σ`BASE_LEN` (23_620) + 8·`MOD_MARGIN` (128). A layout-sum test pins this.
- **`fwht8(v: &mut [f32; 8])`:** in-place fast Walsh-Hadamard (butterfly stages `h = 1,2,4`: `a=v[j]; b=v[j+h]; v[j]=a+b; v[j+h]=a-b`), then multiply all by `HADAMARD_NORM` → orthonormal (energy-preserving).
- **Per sample:** advance `lfo_phase += LFO_RATE·dt` (wrap via floorf). Read: `d_k = BASE_LEN[k] − MOD_DEPTH·(0.5 + 0.5·fast_sin(frac(lfo_phase + k/8)))`; `s[k] = lines[k].read_hermite(slice_k, d_k)`. Damp: `damp_z[k] = s[k]·(1−dc) + damp_z[k]·dc` (`dc = damp·0.4`); `sd = damp_z`. Mix: `h = sd; fwht8(&mut h)`. Write: `g = size·0.25 + 0.7`; `lines[k].write(slice_k, x·INJ + g·h[k])`. Out: `ol = (s0+s2+s4+s6)·OUT; or = (s1+s3+s5+s7)·OUT`; `wet1 = mix·(width·0.5+0.5); wet2 = mix·((1−width)·0.5); dry = 1−mix`; `out_l = x·dry + ol·wet1 + or·wet2; out_r = x·dry + or·wet1 + ol·wet2` (`x = input.at(i)`).
- **Buffer safety:** `Fdn8::process` dry-passes-through (both = input) if `buf.len() < HALL_BUF_SAMPLES`; the graph render arm also guards `>= HALL_BUF_SAMPLES`.
- **Stability:** `H` orthonormal (‖H·sd‖=‖sd‖), damping passive (≤1), `g ≤ 0.95 < 1` ⇒ contraction ⇒ BIBO. Bounded regardless of line lengths.
- **Param map:** `set_param` 0=mix, 1=damp, 2=size, 3=width. `out_width(Kind::Hall) = 2`.
- **Wren:** `Hall.new(input, size, damp, mix)` → width-2 node; REUSES the existing `mix=`/`damp=`/`size=`/`spread=` setters (no new setters). `HALL_BUF_SAMPLES` duped as a Wren-layer local const (keep in sync with the kernel).
- Host tests run with `--target x86_64-unknown-linux-gnu`.

---

### Task 1: `Fdn8` kernel + `fwht8`

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/reverb.rs` (add `fwht8`, `Fdn8`, `HALL_BUF_SAMPLES` + consts; tests)

**Interfaces:**
- Consumes: `crate::In`, `crate::fast_sin`, `crate::delay::DelayLine`, `libm::floorf`.
- Produces: `pub struct Fdn8`; `Fdn8::new()`, `set_mix`/`set_damp`/`set_size`/`set_width`, `process(&mut self, input: In, dt: f32, buf: &mut [f32], out_l: &mut [f32], out_r: &mut [f32])`; `pub const HALL_BUF_SAMPLES: usize`. Task 2 consumes these.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-dsp-kernels/src/reverb.rs`'s `mod tests` (it already has `extern crate std;` + `use super::*;` + `use crate::In;`):

```rust
    fn render_hall(fdn: &mut Fdn8, buf: &mut [f32], input: &[f32]) -> (std::vec::Vec<f32>, std::vec::Vec<f32>) {
        let mut l = std::vec![0.0f32; input.len()];
        let mut r = std::vec![0.0f32; input.len()];
        fdn.process(In::A(input), 1.0 / 44_100.0, buf, &mut l, &mut r);
        (l, r)
    }

    #[test]
    fn hall_layout_sums_to_buf_samples() {
        let sum: usize = BASE_LEN.iter().map(|b| b + MOD_MARGIN).sum();
        assert_eq!(sum, HALL_BUF_SAMPLES);
    }

    #[test]
    fn fwht8_is_orthonormal() {
        // Energy preserved: ‖fwht8(v)‖ ≈ ‖v‖.
        let mut v = [0.3, -0.7, 1.1, 0.2, -0.5, 0.9, -0.1, 0.4];
        let e_in: f32 = v.iter().map(|x| x * x).sum();
        fwht8(&mut v);
        let e_out: f32 = v.iter().map(|x| x * x).sum();
        assert!((e_in - e_out).abs() < 1e-4, "not energy-preserving: {e_in} vs {e_out}");
        // All-ones → only bin 0 = √8, rest 0.
        let mut ones = [1.0f32; 8];
        fwht8(&mut ones);
        assert!((ones[0] - (8.0f32).sqrt()).abs() < 1e-4);
        assert!(ones[1..].iter().all(|x| x.abs() < 1e-4));
    }

    #[test]
    fn hall_impulse_produces_decaying_tail() {
        let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
        let mut fdn = Fdn8::new();
        fdn.set_mix(1.0);
        fdn.set_size(0.8);
        let mut input = std::vec![0.0f32; 40_000];
        input[0] = 1.0;
        let (l, r) = render_hall(&mut fdn, &mut buf, &input);
        assert!(l.iter().all(|v| v.is_finite()) && r.iter().all(|v| v.is_finite()));
        let late: f32 = l[20_000..22_000].iter().map(|v| v * v).sum();
        assert!(late > 1e-8, "tail died too fast: {late}");
        let early: f32 = l[4_000..6_000].iter().map(|v| v * v).sum();
        let later: f32 = l[30_000..32_000].iter().map(|v| v * v).sum();
        assert!(early > later, "tail not decaying: {early} → {later}");
    }

    #[test]
    fn hall_size_lengthens_tail() {
        let tail = |size: f32| -> f32 {
            let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
            let mut fdn = Fdn8::new();
            fdn.set_mix(1.0);
            fdn.set_size(size);
            let mut input = std::vec![0.0f32; 60_000];
            input[0] = 1.0;
            let (l, _r) = render_hall(&mut fdn, &mut buf, &input);
            l[50_000..52_000].iter().map(|v| v * v).sum()
        };
        assert!(tail(0.95) > tail(0.3), "bigger size should decay slower");
    }

    #[test]
    fn hall_damping_darkens_tail() {
        let hf = |damp: f32| -> f32 {
            let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
            let mut fdn = Fdn8::new();
            fdn.set_mix(1.0);
            fdn.set_size(0.85);
            fdn.set_damp(damp);
            let mut input = std::vec![0.0f32; 30_000];
            input[0] = 1.0;
            let (l, _r) = render_hall(&mut fdn, &mut buf, &input);
            l[15_000..19_000].windows(2).map(|w| (w[1] - w[0]).powi(2)).sum()
        };
        assert!(hf(0.9) < hf(0.05), "more damping → less HF in the tail");
    }

    #[test]
    fn hall_output_is_stereo() {
        let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
        let mut fdn = Fdn8::new();
        fdn.set_mix(1.0);
        let input: std::vec::Vec<f32> = (0..12_000).map(|i| (i as f32 * 0.03).sin()).collect();
        let (l, r) = render_hall(&mut fdn, &mut buf, &input);
        let diff: f32 = l.iter().zip(&r).map(|(a, b)| (a - b).abs()).sum();
        assert!(diff > 1.0, "hall should be stereo (l != r): {diff}");
    }

    #[test]
    fn hall_mix_zero_is_dry() {
        let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
        let mut fdn = Fdn8::new();
        fdn.set_mix(0.0);
        let input = std::vec![0.5f32; 256];
        let (l, r) = render_hall(&mut fdn, &mut buf, &input);
        assert!((l[100] - 0.5).abs() < 1e-4 && (r[100] - 0.5).abs() < 1e-4);
    }

    #[test]
    fn hall_short_buffer_is_dry_passthrough() {
        let mut buf = std::vec![0.0f32; 64];
        let mut fdn = Fdn8::new();
        fdn.set_mix(1.0);
        let input = std::vec![0.3f32; 32];
        let (l, r) = render_hall(&mut fdn, &mut buf, &input);
        assert!(l.iter().all(|&v| (v - 0.3).abs() < 1e-6) && r.iter().all(|&v| (v - 0.3).abs() < 1e-6));
    }

    proptest! {
        #![proptest_config(ProptestConfig { cases: 24, ..ProptestConfig::default() })]
        #[test]
        fn hall_is_finite_and_bounded(
            size in 0.0f32..1.0,
            damp in 0.0f32..1.0,
            width in 0.0f32..1.0,
            mix in 0.0f32..1.0,
            amp in 0.0f32..1.0,
        ) {
            let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
            let mut fdn = Fdn8::new();
            fdn.set_size(size); fdn.set_damp(damp); fdn.set_width(width); fdn.set_mix(mix);
            let input = std::vec![amp; 6_000];
            let (l, r) = render_hall(&mut fdn, &mut buf, &input);
            for (a, b) in l.iter().zip(&r) {
                prop_assert!(a.is_finite() && b.is_finite());
                prop_assert!(a.abs() <= 16.0 && b.abs() <= 16.0, "unbounded: {a},{b}");
            }
        }
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels hall`
Expected: FAIL — `Fdn8`/`fwht8`/`HALL_BUF_SAMPLES` not found.

- [ ] **Step 3: Implement `fwht8` + `Fdn8`**

In `crates/deluge-dsp-kernels/src/reverb.rs`, add near the top (the file already has `use crate::In;`):

```rust
use crate::delay::DelayLine;
use crate::fast_sin;

const BASE_LEN: [usize; 8] = [1499, 1889, 2311, 2749, 3187, 3571, 3931, 4483];
const MOD_MARGIN: usize = 16;
const MOD_DEPTH: f32 = 8.0;
const LFO_RATE: f32 = 0.7;
const INJ: f32 = 0.05;
const OUT: f32 = 1.0;
const HADAMARD_NORM: f32 = 0.353_553_39; // 1/√8

/// Σ of the 8 line slice lengths (`base + MOD_MARGIN`).
pub const HALL_BUF_SAMPLES: usize = 23_748;

/// In-place fast Walsh-Hadamard transform on 8 samples, normalized by 1/√8
/// (orthonormal → energy-preserving). Adds/subtracts only.
pub(crate) fn fwht8(v: &mut [f32; 8]) {
    let mut h = 1;
    while h < 8 {
        let mut i = 0;
        while i < 8 {
            for j in i..i + h {
                let a = v[j];
                let b = v[j + h];
                v[j] = a + b;
                v[j + h] = a - b;
            }
            i += 2 * h;
        }
        h *= 2;
    }
    for x in v.iter_mut() {
        *x *= HADAMARD_NORM;
    }
}
```

Add the `Fdn8` struct + impl (after `Freeverb`, before `#[cfg(test)]`):

```rust
/// 8-line modulated FDN hall reverb over a shared partitioned buffer; mono in →
/// stereo out. Feedback = `g · Hadamard · damp(taps)` with `g < 1` and an
/// orthonormal matrix ⇒ BIBO-stable by construction.
#[derive(Clone, Copy)]
pub struct Fdn8 {
    lines: [DelayLine; 8],
    damp_z: [f32; 8],
    lfo_phase: f32,
    size: f32,
    damp: f32,
    width: f32,
    mix: f32,
}
impl Fdn8 {
    pub fn new() -> Fdn8 {
        Fdn8 {
            lines: [DelayLine::new(); 8],
            damp_z: [0.0; 8],
            lfo_phase: 0.0,
            size: 0.5,
            damp: 0.5,
            width: 1.0,
            mix: 0.5,
        }
    }
    pub fn set_mix(&mut self, v: f32) { self.mix = v.clamp(0.0, 1.0); }
    pub fn set_damp(&mut self, v: f32) { self.damp = v.clamp(0.0, 1.0); }
    pub fn set_size(&mut self, v: f32) { self.size = v.clamp(0.0, 1.0); }
    pub fn set_width(&mut self, v: f32) { self.width = v.clamp(0.0, 1.0); }

    pub fn process(&mut self, input: In, dt: f32, buf: &mut [f32], out_l: &mut [f32], out_r: &mut [f32]) {
        if buf.len() < HALL_BUF_SAMPLES {
            for i in 0..out_l.len() {
                let x = input.at(i);
                out_l[i] = x;
                out_r[i] = x;
            }
            return;
        }
        let dc = self.damp * 0.4;
        let g = self.size * 0.25 + 0.7;
        let wet1 = self.mix * (self.width * 0.5 + 0.5);
        let wet2 = self.mix * ((1.0 - self.width) * 0.5);
        let dry = 1.0 - self.mix;
        for i in 0..out_l.len() {
            let x = input.at(i);
            let p = self.lfo_phase + LFO_RATE * dt;
            self.lfo_phase = p - libm::floorf(p);
            // Read (modulated) all 8 lines.
            let mut s = [0.0f32; 8];
            let mut off = 0usize;
            for k in 0..8 {
                let len = BASE_LEN[k] + MOD_MARGIN;
                let t = self.lfo_phase + k as f32 / 8.0;
                let lfo = fast_sin(t - libm::floorf(t));
                let d = BASE_LEN[k] as f32 - MOD_DEPTH * (0.5 + 0.5 * lfo);
                s[k] = self.lines[k].read_hermite(&buf[off..off + len], d);
                off += len;
            }
            // Per-line damping.
            let mut h = [0.0f32; 8];
            for k in 0..8 {
                self.damp_z[k] = s[k] * (1.0 - dc) + self.damp_z[k] * dc;
                h[k] = self.damp_z[k];
            }
            // Orthonormal Hadamard mix.
            fwht8(&mut h);
            // Feedback + inject + write.
            let mut off = 0usize;
            for k in 0..8 {
                let len = BASE_LEN[k] + MOD_MARGIN;
                self.lines[k].write(&mut buf[off..off + len], x * INJ + g * h[k]);
                off += len;
            }
            // Output: even taps → L, odd → R.
            let ol = (s[0] + s[2] + s[4] + s[6]) * OUT;
            let or = (s[1] + s[3] + s[5] + s[7]) * OUT;
            out_l[i] = x * dry + ol * wet1 + or * wet2;
            out_r[i] = x * dry + or * wet1 + ol * wet2;
        }
    }
}
impl Default for Fdn8 {
    fn default() -> Self {
        Self::new()
    }
}
```

- [ ] **Step 4: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels hall`
Expected: PASS (9 tests incl. `hall_layout_sums_to_buf_samples`, `fwht8_is_orthonormal`, and the boundedness proptest), no warnings.

(If `hall_layout_sums_to_buf_samples` fails, `HALL_BUF_SAMPLES` and the `BASE_LEN`+`MOD_MARGIN` sum disagree — recompute the const from the actual sum, do NOT edit the assertion, and report. If a behavioral threshold fails, STOP and report DONE_WITH_CONCERNS with the numbers — do not weaken it.)

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/reverb.rs
git commit -m "feat(dsp-kernels): Fdn8 hall reverb (Hadamard FDN + modulated lines)"
```

---

### Task 2: Graph — `Kind::Hall`

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs` (import, Kind, State, `Node::new`, `out_width`, `set_param`, render arm, tests)

**Interfaces:**
- Consumes: `deluge_dsp_kernels::reverb::{Fdn8, HALL_BUF_SAMPLES}`.
- Produces: `Kind::Hall`. Task 3's factory creates it.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-audio-graph/src/node.rs`'s `mod tests` (`extern crate std;` was added there in Ef-3a):

```rust
    #[test]
    fn hall_node_renders_stereo_with_buffer() {
        use deluge_dsp_kernels::reverb::HALL_BUF_SAMPLES;
        let mut n = Node::new(Kind::Hall, 0);
        assert_eq!(Node::out_width(Kind::Hall), 2);
        n.set_param(0, 1.0); // mix wet
        n.set_param(2, 0.8); // size
        let input: [f32; 64] = core::array::from_fn(|i| if i == 0 { 1.0 } else { 0.0 });
        let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut ring = std::vec![0.0f32; HALL_BUF_SAMPLES];
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
    fn hall_node_without_buffer_is_dry_both_ports() {
        let mut n = Node::new(Kind::Hall, 0);
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

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph hall`
Expected: FAIL — no variant `Hall`.

- [ ] **Step 3: Add the variant, state, construction, params**

In `crates/deluge-audio-graph/src/node.rs`:

Extend the reverb import (the file already imports `reverb::{Freeverb, REVERB_BUF_SAMPLES}`):

```rust
use deluge_dsp_kernels::reverb::{Fdn8, Freeverb, HALL_BUF_SAMPLES, REVERB_BUF_SAMPLES};
```

Add to `enum Kind` (after `Room`):

```rust
    Room,
    Hall,
```

Add to `enum State` (after `Room(Freeverb)`):

```rust
    Hall(Fdn8),
```

In `Node::new`, add an arm (after `Kind::Room => …`):

```rust
            Kind::Hall => State::Hall(Fdn8::new()),
```

In `out_width`, extend the width-2 arm:

```rust
            Kind::Split2 | Kind::Pan | Kind::Chorus | Kind::Flanger | Kind::Room | Kind::Hall => 2,
```

In `set_param`, add an arm (alongside `State::Room`):

```rust
            State::Hall(f) => match param {
                0 => f.set_mix(value),
                1 => f.set_damp(value),
                2 => f.set_size(value),
                3 => f.set_width(value),
                _ => {}
            },
```

- [ ] **Step 4: Add the render arm**

In `process_resolved`, add after the `Kind::Room` arm:

```rust
            Kind::Hall => {
                let (out_l, out_r) = outs.port_pair();
                let ran = if let Some(buf) = pool_region {
                    if buf.len() >= HALL_BUF_SAMPLES {
                        if let State::Hall(f) = &mut self.state {
                            f.process(ins[0], dt, buf, out_l, out_r);
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

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph hall`
Expected: PASS (2 new tests).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: PASS — all existing tests too.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(audio-graph): Kind::Hall FDN reverb node"
```

---

### Task 3: Wren `Hall.new` surface

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`HALL_BUF_SAMPLES` const, `node_hall_impl` + wrapper, register)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (wren-sys registration)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`hall_` + `class Hall`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Kind::Hall` (Task 2), `audio::{alloc_buffer, new_pooled_node, alloc_node_id, set_param}`, `arg_input`, `return_node_w`.
- Produces: Wren `Hall.new(input, size, damp, mix)` (reuses `mix=`/`damp=`/`size=`/`spread=`).

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn hall_new_emits_node_and_params_no_bind_on_capture_host() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var h = Hall.new(Osc.saw(110), 0.8, 0.4, 0.5)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Hall, .. })));
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::BindTable { .. })), "no pool → no BindTable: {cmds:?}");
    for p in [0u8, 1, 2] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, .. } if *param == p)), "missing SetParam {p}: {cmds:?}");
    }
}

#[test]
fn hall_patch_routes_stereo_and_size_setter_reused() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    // Reuses the Room `size=` setter (param 2) — proves the shared surface works on a Hall.
    let cmds = run_and_capture_cmds("var h = Hall.new(Osc.saw(110), 0.5, 0.5, 0.5)\nh.size = 0.9\nOut.patch(h)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 2, .. })), "size→2: {cmds:?}");
    let gains: std::vec::Vec<(f32, f32)> = cmds.iter().filter_map(|c| match c {
        Cmd::BusWriteGains { gl, gr, .. } => Some((*gl, *gr)),
        _ => None,
    }).collect();
    assert!(gains.contains(&(1.0, 0.0)) && gains.contains(&(0.0, 1.0)), "not stereo-routed: {gains:?}");
}

#[test]
fn hall_renders_stereo_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Hall.new(Osc.saw(110), 0.85, 0.4, 0.6))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite() && f.l.abs() <= 1.0 && f.r.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0 || f.r != 0.0), "hall should be non-silent");
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings hall`
Expected: FAIL — Wren `Hall` undefined.

- [ ] **Step 3: Add the const + factory**

In `crates/deluge-wren-core/src/bindings_audio.rs`, add near the `REVERB_BUF_SAMPLES` const:

```rust
/// Ring length for a Hall (FDN) reverb node. Keep in sync with
/// `deluge_dsp_kernels::reverb::HALL_BUF_SAMPLES`.
pub(crate) const HALL_BUF_SAMPLES: usize = 23_748;
```

Add the factory (near `node_room_impl`):

```rust
/// `Node.hall_(input, size, damp, mix)` — allocate the FDN buffer, create a
/// width-2 `Kind::Hall` node, and set size/damp/mix. Reuses the Room setters
/// (`mix=`/`damp=`/`size=`/`spread=`). Unbound → dry passthrough.
pub(crate) fn node_hall_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let size = vm.get_f(2) as f32;
    let damp = vm.get_f(3) as f32;
    let mix = vm.get_f(4) as f32;
    let handle = audio::alloc_buffer(HALL_BUF_SAMPLES);
    let id = audio::alloc_node_id();
    audio::new_pooled_node(id, Kind::Hall, handle, input);
    audio::set_param(id, 2, size);
    audio::set_param(id, 1, damp);
    audio::set_param(id, 0, mix);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_hall(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_hall_impl(&vm);
}
```

Register in `register_audio` (only the factory — no new setters):

```rust
    method("main", "Node", true, "hall_(_,_,_,_)", node_hall_impl::<S>);
```

- [ ] **Step 4: Register in the wren-sys table + prelude**

In `crates/deluge-wren-core/src/bindings.rs`, add near the other audio registrations:

```rust
    static_method("Node", "hall_(_,_,_,_)", bindings_audio::node_hall),
```

In `crates/deluge-wren-core/wren/prelude.wren`, add to `foreign class Node`:

```wren
  foreign static hall_(input, size, damp, mix)
```

And the sugar class (after `class Room`):

```wren
// Hall reverb (8-line modulated FDN) — a dense, smooth, lush tail. `size` [0,1]
// decay/length, `damp` [0,1] HF absorption, `mix` dry/wet. Stereo (width-2);
// reuses the Room controls (size=/damp=/spread=):
//   Out.patch(Hall.new(Osc.saw(110), 0.85, 0.4, 0.4))
//   var h = Hall.new(pad, 0.9, 0.3, 0.5); h.size = 0.95; h.spread = 0.8
class Hall {
  static new(input, size, damp, mix) { Node.hall_(input, size, damp, mix) }
}
```

- [ ] **Step 5: Run the new tests + full suite**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings hall`
Expected: PASS (all 3).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core`
Expected: PASS — existing bindings (Room/Delay/Chorus/Pan/... and the reused `mix=`/`damp=`/`size=`/`spread=`) unchanged, no warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): Hall.new FDN reverb surface"
```

---

## Scope notes (deliberate deferrals)

- **Ef-3c Plate (Dattorro)** — the last reverb; reuses `Comb`/`Allpass`/`DelayLine` + the partitioned-buffer pattern.
- **Sample-rate-scaled tunings** (fixed 44.1 kHz sample counts), **pre-delay**, **early reflections**, **per-line diffusion allpasses**, **freeze**, **tempo-sync** — later.
- **`HALL_BUF_SAMPLES` is duplicated** as a Wren-layer local const (matching the `REVERB_BUF_SAMPLES`/`CHORUS_BUF_SAMPLES` convention); the kernel's is authoritative and the graph guard uses it.

## Post-implementation

After all three tasks pass, use **superpowers:finishing-a-development-branch** to verify the suites and merge.
