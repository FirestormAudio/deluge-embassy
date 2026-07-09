# Ef-3c Plate Reverb (Dattorro) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a faithful Dattorro plate reverb — input diffusion → figure-8 tank of modulated allpasses → the characteristic multi-tap output network — as a mono→stereo Wren `Plate.new` node that reuses the Room param surface. Completes the Ef-3 Reverb suite.

**Architecture:** A `Dattorro` engine (in `deluge-dsp-kernels/src/reverb.rs`, alongside `Freeverb`/`Fdn8`) partitions one pooled buffer into 12 delay elements addressed by 12 cursors, with standalone `ap`/`ap_read`/`delayproc`/`tap` helpers implementing the Dattorro allpass/delay recurrences (a different form from the freeverb `Allpass`, so `Allpass` is NOT modified). `Kind::Plate` mirrors `Kind::Hall`. Plate reuses the Room Wren setters — only a `plate_` factory + `class Plate` are new.

**Tech Stack:** Rust, `no_std`/no-heap, `libm` (floorf), `fast_sin`, Wren via `wren-sys`, proptest.

## Global Constraints

- **`no_std`, no heap, pure `f32`, deterministic.** Kernel owns NO delay storage (buffer borrowed). Wrap LFO phase with `libm::floorf`; `fast_sin` arg in `[0,1)`.
- **From the PUBLIC Dattorro 1997 paper structure** (constants cross-checked against the MIT-licensed `el-visio/dattorro-verb` and Louis Couka's implementation) — MIT/Apache, no GPL source. All reads are INTEGER offsets (no fractional interpolation); excursion is a slow integer read-offset modulation.
- **Element lengths (12), order `[idf0,idf1,idf2,idf3, ad1L,ad1R, daL,daR, ad2L,ad2R, dbL,dbR]`:** `LEN = [142,107,379,277, 672,908, 4453,4217, 1800,2656, 3720,3163]`. Cumulative offsets → **`PLATE_BUF_SAMPLES = 22_494`** (Σ LEN). A layout-sum test pins this.
- **Gains/consts:** input diffusers `0.75,0.75,0.625,0.625`; `DDIFF1 = 0.7` (applied as **−0.7**); `DDIFF2 = 0.5`; `BANDWIDTH = 0.9995`; `EXC_DEPTH = 16`; `LFO_RATE = 1.0` Hz; `OUT_SCALE = 0.6`.
- **Dattorro allpass** (helper `ap`/`ap_read`): `d = slice[read]; w = x − gain·d; slice[cursor] = w; advance; return d + gain·w`. `ap` reads at the full length (`slice[cursor]`); `ap_read` reads at `read_back` samples (for the modulated ad1). **Delay** (`delayproc`): `out = slice[cursor]; slice[cursor] = x; advance; return out`. **Tap** (`tap`, read-only): `slice[(cursor + len − offset) % len]`.
- **Per sample:** bandwidth `bw += (x − bw)·BANDWIDTH`; run 4 input diffusers in series on `bw`; advance excursion LFO, `exc = (EXC_DEPTH·(0.5+0.5·fast_sin(phase))) as usize`; read the two `db` outputs as cross-feed (`fb_l = dbR_out·decay`, `fb_r = dbL_out·decay`); process each half `[s + fb] → ap_read(ad1, −DDIFF1, LEN−exc) → delayproc(da) → damping LP → ×decay → ap(ad2, DDIFF2) → delayproc(db)`; then read the L/R 7-tap networks (offsets below), `×OUT_SCALE`; blend `out_l = x·dry + yl·wet1 + yr·wet2; out_r = x·dry + yr·wet1 + yl·wet2`.
- **Output taps** (element index; `+`/`−`): **yL** = `+daR@266 +daR@2974 −ad2R@1913 +dbR@1996 −daL@1990 −ad2L@187 −dbL@1066`; **yR** = `+daL@353 +daL@3627 −ad2L@1228 +dbL@2673 −daR@2111 −ad2R@335 −dbR@121`.
- **Mappings:** `decay = size·0.4 + 0.5` (∈ [0.5, 0.9]); damping coeff `damp_c = 1 − damp·0.9` (∈ [0.1, 1]); `wet1 = mix·(width·0.5+0.5); wet2 = mix·((1−width)·0.5); dry = 1−mix`.
- **Buffer safety:** `process` dry-passes-through if `buf.len() < PLATE_BUF_SAMPLES`; the graph render arm also guards `>= PLATE_BUF_SAMPLES`.
- **Stability:** allpasses unity-gain, damping passive, every loop path × `decay < 1` ⇒ bounded (boundedness proptest is the gate).
- **Param map:** `set_param` 0=mix, 1=damp, 2=size, 3=width. `out_width(Kind::Plate) = 2`.
- **Wren:** `Plate.new(input, size, damp, mix)` → width-2; REUSES the Room setters `mix=`/`damp=`/`size=`/`spread=` (NO new setters). `PLATE_BUF_SAMPLES` duped as a Wren-layer local const.
- Host tests run with `--target x86_64-unknown-linux-gnu`.

---

### Task 1: `Dattorro` plate kernel

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/reverb.rs` (add helpers, `Dattorro`, `PLATE_BUF_SAMPLES` + consts; tests)

**Interfaces:**
- Consumes: `crate::In`, `crate::fast_sin`, `libm::floorf`.
- Produces: `pub struct Dattorro`; `Dattorro::new()`, `set_mix`/`set_damp`/`set_size`/`set_width`, `process(&mut self, input: In, dt: f32, buf: &mut [f32], out_l: &mut [f32], out_r: &mut [f32])`; `pub const PLATE_BUF_SAMPLES: usize`. Task 2 consumes these.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-dsp-kernels/src/reverb.rs`'s `mod tests`:

```rust
    fn render_plate(d: &mut Dattorro, buf: &mut [f32], input: &[f32]) -> (std::vec::Vec<f32>, std::vec::Vec<f32>) {
        let mut l = std::vec![0.0f32; input.len()];
        let mut r = std::vec![0.0f32; input.len()];
        d.process(In::A(input), 1.0 / 44_100.0, buf, &mut l, &mut r);
        (l, r)
    }

    #[test]
    fn plate_layout_sums_to_buf_samples() {
        let sum: usize = PLATE_LEN.iter().sum();
        assert_eq!(sum, PLATE_BUF_SAMPLES);
    }

    #[test]
    fn plate_impulse_produces_decaying_tail() {
        let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
        let mut d = Dattorro::new();
        d.set_mix(1.0);
        d.set_size(0.8);
        let mut input = std::vec![0.0f32; 40_000];
        input[0] = 1.0;
        let (l, r) = render_plate(&mut d, &mut buf, &input);
        assert!(l.iter().all(|v| v.is_finite()) && r.iter().all(|v| v.is_finite()));
        let late: f32 = l[20_000..22_000].iter().map(|v| v * v).sum();
        assert!(late > 1e-9, "tail died too fast: {late}");
        let early: f32 = l[4_000..6_000].iter().map(|v| v * v).sum();
        let later: f32 = l[30_000..32_000].iter().map(|v| v * v).sum();
        assert!(early > later, "tail not decaying: {early} → {later}");
    }

    #[test]
    fn plate_size_lengthens_tail() {
        let tail = |size: f32| -> f32 {
            let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
            let mut d = Dattorro::new();
            d.set_mix(1.0);
            d.set_size(size);
            let mut input = std::vec![0.0f32; 50_000];
            input[0] = 1.0;
            let (l, _r) = render_plate(&mut d, &mut buf, &input);
            l[40_000..42_000].iter().map(|v| v * v).sum()
        };
        assert!(tail(0.95) > tail(0.2), "bigger size should decay slower");
    }

    #[test]
    fn plate_damping_darkens_tail() {
        let hf = |damp: f32| -> f32 {
            let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
            let mut d = Dattorro::new();
            d.set_mix(1.0);
            d.set_size(0.85);
            d.set_damp(damp);
            let mut input = std::vec![0.0f32; 25_000];
            input[0] = 1.0;
            let (l, _r) = render_plate(&mut d, &mut buf, &input);
            l[12_000..16_000].windows(2).map(|w| (w[1] - w[0]).powi(2)).sum()
        };
        assert!(hf(0.9) < hf(0.05), "more damping → less HF in the tail");
    }

    #[test]
    fn plate_output_is_stereo() {
        let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
        let mut d = Dattorro::new();
        d.set_mix(1.0);
        let input: std::vec::Vec<f32> = (0..12_000).map(|i| (i as f32 * 0.03).sin()).collect();
        let (l, r) = render_plate(&mut d, &mut buf, &input);
        let diff: f32 = l.iter().zip(&r).map(|(a, b)| (a - b).abs()).sum();
        assert!(diff > 1.0, "plate should be stereo (l != r): {diff}");
    }

    #[test]
    fn plate_mix_zero_is_dry() {
        let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
        let mut d = Dattorro::new();
        d.set_mix(0.0);
        let input = std::vec![0.5f32; 256];
        let (l, r) = render_plate(&mut d, &mut buf, &input);
        assert!((l[100] - 0.5).abs() < 1e-4 && (r[100] - 0.5).abs() < 1e-4);
    }

    #[test]
    fn plate_short_buffer_is_dry_passthrough() {
        let mut buf = std::vec![0.0f32; 64];
        let mut d = Dattorro::new();
        d.set_mix(1.0);
        let input = std::vec![0.3f32; 32];
        let (l, r) = render_plate(&mut d, &mut buf, &input);
        assert!(l.iter().all(|&v| (v - 0.3).abs() < 1e-6) && r.iter().all(|&v| (v - 0.3).abs() < 1e-6));
    }

    proptest! {
        #![proptest_config(ProptestConfig { cases: 24, ..ProptestConfig::default() })]
        #[test]
        fn plate_is_finite_and_bounded(
            size in 0.0f32..1.0,
            damp in 0.0f32..1.0,
            width in 0.0f32..1.0,
            mix in 0.0f32..1.0,
            amp in 0.0f32..1.0,
        ) {
            let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
            let mut d = Dattorro::new();
            d.set_size(size); d.set_damp(damp); d.set_width(width); d.set_mix(mix);
            let input = std::vec![amp; 6_000];
            let (l, r) = render_plate(&mut d, &mut buf, &input);
            for (a, b) in l.iter().zip(&r) {
                prop_assert!(a.is_finite() && b.is_finite());
                prop_assert!(a.abs() <= 32.0 && b.abs() <= 32.0, "unbounded: {a},{b}");
            }
        }
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels plate`
Expected: FAIL — `Dattorro`/`PLATE_LEN`/`PLATE_BUF_SAMPLES` not found.

- [ ] **Step 3: Implement the helpers + `Dattorro`**

In `crates/deluge-dsp-kernels/src/reverb.rs`, add near the top (the file already has `use crate::In;`; add `use crate::fast_sin;` if not already present from the Fdn8 task):

```rust
/// Dattorro element lengths, order [idf0,idf1,idf2,idf3, ad1L,ad1R, daL,daR, ad2L,ad2R, dbL,dbR].
const PLATE_LEN: [usize; 12] = [142, 107, 379, 277, 672, 908, 4453, 4217, 1800, 2656, 3720, 3163];
pub const PLATE_BUF_SAMPLES: usize = 22_494;
const PLATE_BANDWIDTH: f32 = 0.9995;
const PLATE_DDIFF1: f32 = 0.7;
const PLATE_DDIFF2: f32 = 0.5;
const PLATE_EXC_DEPTH: usize = 16;
const PLATE_LFO_RATE: f32 = 1.0;
const PLATE_OUT_SCALE: f32 = 0.6;

/// Dattorro allpass reading at the full slice length (`slice[cursor]`).
fn plate_ap(slice: &mut [f32], c: &mut usize, x: f32, gain: f32) -> f32 {
    let len = slice.len();
    let d = slice[*c];
    let w = x - gain * d;
    slice[*c] = w;
    *c += 1;
    if *c >= len {
        *c = 0;
    }
    d + gain * w
}

/// Dattorro allpass reading `read_back` samples behind the cursor (modulated ad1).
fn plate_ap_read(slice: &mut [f32], c: &mut usize, x: f32, gain: f32, read_back: usize) -> f32 {
    let len = slice.len();
    let d = slice[(*c + len - read_back) % len];
    let w = x - gain * d;
    slice[*c] = w;
    *c += 1;
    if *c >= len {
        *c = 0;
    }
    d + gain * w
}

/// Plain delay: return the len-samples-ago output, write `x`, advance.
fn plate_delay(slice: &mut [f32], c: &mut usize, x: f32) -> f32 {
    let len = slice.len();
    let out = slice[*c];
    slice[*c] = x;
    *c += 1;
    if *c >= len {
        *c = 0;
    }
    out
}

/// Read `offset` samples behind the cursor (output tap; no state change).
#[inline]
fn plate_tap(slice: &[f32], c: usize, offset: usize) -> f32 {
    let len = slice.len();
    slice[(c + len - offset) % len]
}

/// Dattorro plate reverb: input diffusion + a figure-8 modulated-allpass tank +
/// a multi-tap output network, over one partitioned buffer. Mono in → stereo out.
#[derive(Clone, Copy)]
pub struct Dattorro {
    c: [usize; 12],   // per-element cursors
    damp_z: [f32; 2], // per-half damping LP state
    bw: f32,          // input bandwidth LP state
    lfo_phase: f32,   // excursion LFO
    size: f32,
    damp: f32,
    width: f32,
    mix: f32,
}
impl Dattorro {
    pub fn new() -> Dattorro {
        Dattorro { c: [0; 12], damp_z: [0.0; 2], bw: 0.0, lfo_phase: 0.0, size: 0.5, damp: 0.5, width: 1.0, mix: 0.5 }
    }
    pub fn set_mix(&mut self, v: f32) { self.mix = v.clamp(0.0, 1.0); }
    pub fn set_damp(&mut self, v: f32) { self.damp = v.clamp(0.0, 1.0); }
    pub fn set_size(&mut self, v: f32) { self.size = v.clamp(0.0, 1.0); }
    pub fn set_width(&mut self, v: f32) { self.width = v.clamp(0.0, 1.0); }

    pub fn process(&mut self, input: In, dt: f32, buf: &mut [f32], out_l: &mut [f32], out_r: &mut [f32]) {
        if buf.len() < PLATE_BUF_SAMPLES {
            for i in 0..out_l.len() {
                let x = input.at(i);
                out_l[i] = x;
                out_r[i] = x;
            }
            return;
        }
        // Element offsets (cumulative walk).
        let mut off = [0usize; 12];
        {
            let mut o = 0;
            for k in 0..12 {
                off[k] = o;
                o += PLATE_LEN[k];
            }
        }
        let decay = self.size * 0.4 + 0.5;
        let damp_c = 1.0 - self.damp * 0.9;
        let wet1 = self.mix * (self.width * 0.5 + 0.5);
        let wet2 = self.mix * ((1.0 - self.width) * 0.5);
        let dry = 1.0 - self.mix;
        // Convenience: a mutable slice for element k.
        macro_rules! el {
            ($k:expr) => {
                &mut buf[off[$k]..off[$k] + PLATE_LEN[$k]]
            };
        }
        for i in 0..out_l.len() {
            let x = input.at(i);
            // Input bandwidth low-pass.
            self.bw += (x - self.bw) * PLATE_BANDWIDTH;
            let mut s = self.bw;
            // 4 series input diffusers.
            for k in 0..4 {
                let g = if k < 2 { 0.75 } else { 0.625 };
                s = plate_ap(el!(k), &mut self.c[k], s, g);
            }
            // Excursion LFO → integer read-offset for the two ad1 allpasses.
            let p = self.lfo_phase + PLATE_LFO_RATE * dt;
            self.lfo_phase = p - libm::floorf(p);
            let exc = (PLATE_EXC_DEPTH as f32 * (0.5 + 0.5 * fast_sin(self.lfo_phase))) as usize;
            // Cross-feed: read both post-damping-delay (db) outputs before writes.
            let fb_l = buf[off[11] + self.c[11]] * decay; // dbR output → left half
            let fb_r = buf[off[10] + self.c[10]] * decay; // dbL output → right half
            // Left half (elements 4=ad1L, 6=daL, 8=ad2L, 10=dbL).
            let mut t = s + fb_l;
            t = plate_ap_read(el!(4), &mut self.c[4], t, -PLATE_DDIFF1, PLATE_LEN[4] - exc);
            t = plate_delay(el!(6), &mut self.c[6], t);
            self.damp_z[0] += (t - self.damp_z[0]) * damp_c;
            t = self.damp_z[0] * decay;
            t = plate_ap(el!(8), &mut self.c[8], t, PLATE_DDIFF2);
            plate_delay(el!(10), &mut self.c[10], t);
            // Right half (elements 5=ad1R, 7=daR, 9=ad2R, 11=dbR).
            let mut u = s + fb_r;
            u = plate_ap_read(el!(5), &mut self.c[5], u, -PLATE_DDIFF1, PLATE_LEN[5] - exc);
            u = plate_delay(el!(7), &mut self.c[7], u);
            self.damp_z[1] += (u - self.damp_z[1]) * damp_c;
            u = self.damp_z[1] * decay;
            u = plate_ap(el!(9), &mut self.c[9], u, PLATE_DDIFF2);
            plate_delay(el!(11), &mut self.c[11], u);
            // Output taps (read-only, after all writes/advances).
            let ta = |k: usize, o: usize| plate_tap(&buf[off[k]..off[k] + PLATE_LEN[k]], self.c[k], o);
            let yl = PLATE_OUT_SCALE
                * (ta(7, 266) + ta(7, 2974) - ta(9, 1913) + ta(11, 1996) - ta(6, 1990) - ta(8, 187) - ta(10, 1066));
            let yr = PLATE_OUT_SCALE
                * (ta(6, 353) + ta(6, 3627) - ta(8, 1228) + ta(10, 2673) - ta(7, 2111) - ta(9, 335) - ta(11, 121));
            out_l[i] = x * dry + yl * wet1 + yr * wet2;
            out_r[i] = x * dry + yr * wet1 + yl * wet2;
        }
    }
}
impl Default for Dattorro {
    fn default() -> Self {
        Self::new()
    }
}
```

- [ ] **Step 4: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels plate`
Expected: PASS (8 tests incl. `plate_layout_sums_to_buf_samples` and the boundedness proptest), no warnings.

(If `plate_layout_sums_to_buf_samples` fails, `PLATE_BUF_SAMPLES` and `Σ PLATE_LEN` disagree — recompute the const from the sum, do NOT edit the assertion, and report. If a behavioral threshold fails, STOP and report DONE_WITH_CONCERNS with the numbers.)

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/reverb.rs
git commit -m "feat(dsp-kernels): Dattorro plate reverb (figure-8 tank + multi-tap network)"
```

---

### Task 2: Graph — `Kind::Plate`

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs` (import, Kind, State, `Node::new`, `out_width`, `set_param`, render arm, tests)

**Interfaces:**
- Consumes: `deluge_dsp_kernels::reverb::{Dattorro, PLATE_BUF_SAMPLES}`.
- Produces: `Kind::Plate`. Task 3's factory creates it.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-audio-graph/src/node.rs`'s `mod tests`:

```rust
    #[test]
    fn plate_node_renders_stereo_with_buffer() {
        use deluge_dsp_kernels::reverb::PLATE_BUF_SAMPLES;
        let mut n = Node::new(Kind::Plate, 0);
        assert_eq!(Node::out_width(Kind::Plate), 2);
        n.set_param(0, 1.0);
        n.set_param(2, 0.8);
        let input: [f32; 64] = core::array::from_fn(|i| if i == 0 { 1.0 } else { 0.0 });
        let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut ring = std::vec![0.0f32; PLATE_BUF_SAMPLES];
        let mut p0 = [0.0f32; 64];
        let mut p1 = [0.0f32; 64];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 44_100.0, &mut outs, Some(&mut ring));
        }
        assert!(p0.iter().all(|s| s.is_finite() && s.abs() <= 32.0));
        assert!(p1.iter().all(|s| s.is_finite() && s.abs() <= 32.0));
    }

    #[test]
    fn plate_node_without_buffer_is_dry_both_ports() {
        let mut n = Node::new(Kind::Plate, 0);
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

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph plate`
Expected: FAIL — no variant `Plate`.

- [ ] **Step 3: Add the variant, state, construction, params**

In `crates/deluge-audio-graph/src/node.rs`:

Extend the reverb import (the file already imports `reverb::{Fdn8, Freeverb, HALL_BUF_SAMPLES, REVERB_BUF_SAMPLES}`):

```rust
use deluge_dsp_kernels::reverb::{Dattorro, Fdn8, Freeverb, HALL_BUF_SAMPLES, PLATE_BUF_SAMPLES, REVERB_BUF_SAMPLES};
```

Add to `enum Kind` (after `Hall`):

```rust
    Hall,
    Plate,
```

Add to `enum State` (after `Hall(Fdn8)`):

```rust
    Plate(Dattorro),
```

In `Node::new`, add an arm (after `Kind::Hall => …`):

```rust
            Kind::Plate => State::Plate(Dattorro::new()),
```

In `out_width`, extend the width-2 arm:

```rust
            Kind::Split2 | Kind::Pan | Kind::Chorus | Kind::Flanger | Kind::Room | Kind::Hall | Kind::Plate => 2,
```

In `set_param`, add an arm (alongside `State::Hall`):

```rust
            State::Plate(d) => match param {
                0 => d.set_mix(value),
                1 => d.set_damp(value),
                2 => d.set_size(value),
                3 => d.set_width(value),
                _ => {}
            },
```

- [ ] **Step 4: Add the render arm**

In `process_resolved`, add after the `Kind::Hall` arm:

```rust
            Kind::Plate => {
                let (out_l, out_r) = outs.port_pair();
                let ran = if let Some(buf) = pool_region {
                    if buf.len() >= PLATE_BUF_SAMPLES {
                        if let State::Plate(d) = &mut self.state {
                            d.process(ins[0], dt, buf, out_l, out_r);
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

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph plate`
Expected: PASS (2 new tests).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: PASS — all existing tests too.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(audio-graph): Kind::Plate Dattorro reverb node"
```

---

### Task 3: Wren `Plate.new` surface

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`PLATE_BUF_SAMPLES` const, `node_plate_impl` + wrapper, register)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (wren-sys registration)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`plate_` + `class Plate`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Kind::Plate` (Task 2), `audio::{alloc_buffer, new_pooled_node, alloc_node_id, set_param}`, `arg_input`, `return_node_w`.
- Produces: Wren `Plate.new(input, size, damp, mix)` (reuses `mix=`/`damp=`/`size=`/`spread=`).

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn plate_new_emits_node_and_params_no_bind_on_capture_host() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var p = Plate.new(Osc.saw(110), 0.8, 0.4, 0.5)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Plate, .. })));
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::BindTable { .. })), "no pool → no BindTable: {cmds:?}");
    for p in [0u8, 1, 2] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, .. } if *param == p)), "missing SetParam {p}: {cmds:?}");
    }
}

#[test]
fn plate_patch_routes_stereo() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds("Out.patch(Plate.new(Osc.saw(110), 0.8, 0.4, 0.5))");
    let gains: std::vec::Vec<(f32, f32)> = cmds.iter().filter_map(|c| match c {
        Cmd::BusWriteGains { gl, gr, .. } => Some((*gl, *gr)),
        _ => None,
    }).collect();
    assert!(gains.contains(&(1.0, 0.0)) && gains.contains(&(0.0, 1.0)), "not stereo-routed: {gains:?}");
}

#[test]
fn plate_renders_stereo_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Plate.new(Osc.saw(110), 0.85, 0.4, 0.6))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite() && f.l.abs() <= 1.0 && f.r.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0 || f.r != 0.0), "plate should be non-silent");
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings plate`
Expected: FAIL — Wren `Plate` undefined.

- [ ] **Step 3: Add the const + factory**

In `crates/deluge-wren-core/src/bindings_audio.rs`, add near the `HALL_BUF_SAMPLES` const:

```rust
/// Ring length for a Plate (Dattorro) reverb node. Keep in sync with
/// `deluge_dsp_kernels::reverb::PLATE_BUF_SAMPLES`.
pub(crate) const PLATE_BUF_SAMPLES: usize = 22_494;
```

Add the factory (near `node_hall_impl`):

```rust
/// `Node.plate_(input, size, damp, mix)` — allocate the Dattorro buffer, create a
/// width-2 `Kind::Plate` node, and set size/damp/mix. Reuses the Room setters.
/// Unbound → dry passthrough.
pub(crate) fn node_plate_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let size = vm.get_f(2) as f32;
    let damp = vm.get_f(3) as f32;
    let mix = vm.get_f(4) as f32;
    let handle = audio::alloc_buffer(PLATE_BUF_SAMPLES);
    let id = audio::alloc_node_id();
    audio::new_pooled_node(id, Kind::Plate, handle, input);
    audio::set_param(id, 2, size);
    audio::set_param(id, 1, damp);
    audio::set_param(id, 0, mix);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_plate(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_plate_impl(&vm);
}
```

Register in `register_audio` (only the factory — no new setters):

```rust
    method("main", "Node", true, "plate_(_,_,_,_)", node_plate_impl::<S>);
```

- [ ] **Step 4: Register in the wren-sys table + prelude**

In `crates/deluge-wren-core/src/bindings.rs`, add near the other audio registrations:

```rust
    static_method("Node", "plate_(_,_,_,_)", bindings_audio::node_plate),
```

In `crates/deluge-wren-core/wren/prelude.wren`, add to `foreign class Node`:

```wren
  foreign static plate_(input, size, damp, mix)
```

And the sugar class (after `class Hall`):

```wren
// Plate reverb (Dattorro) — a bright, dense, metallic-smooth plate. `size` [0,1]
// decay/length, `damp` [0,1] HF absorption, `mix` dry/wet. Stereo (width-2);
// reuses the Room controls (size=/damp=/spread=):
//   Out.patch(Plate.new(Osc.saw(110), 0.85, 0.4, 0.4))
//   var p = Plate.new(pad, 0.9, 0.3, 0.5); p.size = 0.95; p.spread = 0.8
class Plate {
  static new(input, size, damp, mix) { Node.plate_(input, size, damp, mix) }
}
```

- [ ] **Step 5: Run the new tests + full suite**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings plate`
Expected: PASS (all 3).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core`
Expected: PASS — existing bindings (Room/Hall/Delay/Chorus/... + reused `mix=`/`damp=`/`size=`/`spread=`) unchanged, no warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): Plate.new Dattorro reverb surface"
```

---

## Scope notes (deliberate deferrals)

- **Pre-delay, separate bandwidth/excursion/decay-diffusion params, fractional (smoother) excursion, freeze, tempo-sync, SR-scaled tunings** — later plate refinements (excursion is integer-stepped in v1).
- **`PLATE_BUF_SAMPLES` is duplicated** as a Wren-layer local const (matching the `HALL_BUF_SAMPLES`/`REVERB_BUF_SAMPLES` convention); the kernel's is authoritative and the graph guard uses it.
- **Completes Ef-3 Reverb** (Room + Hall + Plate). Optional later: a unified `Reverb.new(type, …)` selector over the three.

## Post-implementation

After all three tasks pass, use **superpowers:finishing-a-development-branch** to verify the suites and merge.
