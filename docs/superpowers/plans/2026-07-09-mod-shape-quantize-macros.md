# Mod-3: Shaping / Quantize / Macros Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the modulation-*shaping* layer to the Deluge DSP graph — a `Curve`
(non-linear transfer), `QuantStep`/`QuantPitch` quantizers, an `Mtof`
semitone→Hz converter, a settable `Ctrl` macro source, and Wren scaling sugar.

**Architecture:** Five mono kernels across two new kernel modules
(`shape.rs`, `quant.rs`), exposed through the established 3-crate pattern:
kernel → `deluge-audio-graph` node (`Kind`/`State`/`Node::new`/`set_param`/render
arm) → `deluge-wren-core` factory + prelude sugar. `Curve` is stateless (a free
function like `math::mul`); the rest carry small kernel structs. `Mtof` uses
`libm::exp2f` (already a crate dependency).

**Tech Stack:** Rust `no_std` (`deluge-dsp-kernels`), the audio graph
(`deluge-audio-graph`), Wren bindings (`deluge-wren-core`), `proptest`, `libm`.

## Global Constraints

- `no_std`, no heap, pure `f32`, deterministic. All five nodes are **mono**
  (`out_width` default `_ => 1`). No buffers, no pool.
- Host tests run on the x86 target: `cargo test --target x86_64-unknown-linux-gnu -p <crate>`.
  Never `--workspace` on host (firmware crates are ARM-only and won't unify).
- LSP "can't find crate for `test`/`std`" diagnostics are armv7a-target noise — ignore.
- **`no_std` numerics:** `core` lacks `f32::floor`/`round`/`exp2`. Use the
  crate-root `pub(crate) fn floorf`; `round(x)` = `floorf(x + 0.5)` (ties → +∞).
  `2^x` = `libm::exp2f`. `clamp` is fine; for `abs` use `if x < 0.0 { -x } else { x }`.
- Kernels are odd-symmetric / bounded exactly as specified — do not weaken a
  numeric assertion to make a test pass; if a behavioral number is off, STOP and
  report it.
- Flat Node selector namespace: new setter selectors must not collide with
  existing ones. Only one new setter here: `value=` (→ `set_param(0)` on `Ctrl`).
- Register every new Wren foreign method in **both** binding tables
  (`bindings.rs` static table AND `bindings_audio.rs::register_audio`).
- Scale masks: bit `i` set ⇒ pitch-class `i` allowed, root-relative, LSB = root.
  `Chromatic = 4095`, `Major = 2741` ({0,2,4,5,7,9,11}), `Minor = 1453`
  ({0,2,3,5,7,8,10}), `HarmonicMinor = 2477` ({0,2,3,5,7,8,11}),
  `Dorian = 1709` ({0,2,3,5,7,9,10}), `Mixolydian = 1717` ({0,2,4,5,7,9,10}),
  `MajorPentatonic = 661` ({0,2,4,7,9}), `MinorPentatonic = 1193` ({0,3,5,7,10}),
  `WholeTone = 1365` ({0,2,4,6,8,10}), `Blues = 1257` ({0,3,5,6,7,10}).

---

### Task 1: `shape.rs` kernels — `Curve` (stateless) + `Ctrl`

**Files:**
- Create: `crates/deluge-dsp-kernels/src/shape.rs`
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (add `pub mod shape;`)

**Interfaces:**
- Consumes: `crate::In` (control/audio input abstraction; `.at(i)` reads a sample).
- Produces:
  - `pub fn curve(input: In, k: In, out: &mut [f32])` — stateless odd-symmetric
    Schlick bias. `k ∈ [-1,1]`.
  - `pub struct Ctrl { value: f32 }` with `Ctrl::new() -> Ctrl` (value 0),
    `set_value(&mut self, v: f32)`, `process(&mut self, out: &mut [f32])`.

- [ ] **Step 1: Create `shape.rs` with the kernels + tests**

Create `crates/deluge-dsp-kernels/src/shape.rs`:

```rust
//! Modulation shaping: a non-linear transfer curve and a settable scalar source.
//! Tiny per-sample mono kernels — no heap, no buffer.

use crate::In;

/// Odd-symmetric non-linear transfer (Schlick's *bias*). Stateless, so this is
/// a free function (like `math::mul`). `in0` = input, `in1` = k ∈ [-1,1]:
/// `k=0` linear, `k>0` exponential (ease-in), `k<0` logarithmic (ease-out).
/// Bipolar-in → bipolar-out, monotonic, fixed points at 0 and ±1.
pub fn curve(input: In, k: In, out: &mut [f32]) {
    for i in 0..out.len() {
        let x = input.at(i).clamp(-1.0, 1.0);
        let b = ((k.at(i) + 1.0) * 0.5).clamp(0.05, 0.95); // k∈[-1,1] → b∈[0.05,0.95]
        let s = if x < 0.0 { -1.0 } else { 1.0 };
        let u = if x < 0.0 { -x } else { x }; // |x| ∈ [0,1]
        let y = u / ((1.0 / b - 2.0) * (1.0 - u) + 1.0); // (0,0)→(1,1)
        out[i] = s * y;
    }
}

/// A held, runtime-settable scalar source — the macro primitive. No input
/// ports; `param 0 = value`. Fills the output block with `value`.
#[derive(Clone, Copy)]
pub struct Ctrl {
    value: f32,
}
impl Ctrl {
    pub fn new() -> Ctrl {
        Ctrl { value: 0.0 }
    }
    pub fn set_value(&mut self, v: f32) {
        self.value = v;
    }
    pub fn process(&mut self, out: &mut [f32]) {
        for s in out.iter_mut() {
            *s = self.value;
        }
    }
}
impl Default for Ctrl {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;

    fn ramp(n: usize) -> std::vec::Vec<f32> {
        (0..n).map(|i| i as f32 / (n - 1) as f32 * 2.0 - 1.0).collect() // -1..=1
    }

    #[test]
    fn curve_linear_at_k0() {
        let x = ramp(21);
        let mut out = std::vec![0.0f32; 21];
        curve(In::A(&x), In::K(0.0), &mut out);
        for i in 0..21 {
            assert!((out[i] - x[i]).abs() < 1e-6, "k=0 is identity at {i}: {} vs {}", out[i], x[i]);
        }
    }

    #[test]
    fn curve_fixed_points_and_midpoint_gain() {
        // Fixed points 0 and ±1 for any k; k>0 boosts the midpoint, k<0 attenuates.
        let mut out = std::vec![0.0f32; 3];
        curve(In::A(&[-1.0, 0.0, 1.0]), In::K(0.6), &mut out);
        assert!((out[0] + 1.0).abs() < 1e-6 && out[1].abs() < 1e-6 && (out[2] - 1.0).abs() < 1e-6);
        // b = (0.6+1)/2 = 0.8; bias(0.5,0.8) = 0.8.
        let mut m = std::vec![0.0f32; 1];
        curve(In::A(&[0.5]), In::K(0.6), &mut m);
        assert!((m[0] - 0.8).abs() < 1e-4, "k=0.6 boosts 0.5→0.8: {}", m[0]);
        // b = 0.2; bias(0.5,0.2) = 0.2.
        curve(In::A(&[0.5]), In::K(-0.6), &mut m);
        assert!((m[0] - 0.2).abs() < 1e-4, "k=-0.6 attenuates 0.5→0.2: {}", m[0]);
    }

    #[test]
    fn curve_odd_symmetric_and_monotonic() {
        let x = ramp(41);
        let mut out = std::vec![0.0f32; 41];
        curve(In::A(&x), In::K(0.7), &mut out);
        // odd symmetry: out[mid-j] == -out[mid+j]
        for j in 0..=20 {
            assert!((out[20 - j] + out[20 + j]).abs() < 1e-5, "odd symmetry at {j}");
        }
        // monotonic non-decreasing, bounded [-1,1]
        for i in 1..41 {
            assert!(out[i] >= out[i - 1] - 1e-6, "monotonic at {i}");
            assert!(out[i].abs() <= 1.0001, "bounded at {i}: {}", out[i]);
        }
    }

    #[test]
    fn ctrl_holds_and_updates() {
        let mut c = Ctrl::new();
        let mut out = std::vec![9.0f32; 4];
        c.process(&mut out);
        assert_eq!(out, std::vec![0.0, 0.0, 0.0, 0.0], "new Ctrl outputs 0");
        c.set_value(3.5);
        c.process(&mut out);
        assert_eq!(out, std::vec![3.5, 3.5, 3.5, 3.5], "set_value changes output");
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn curve_stays_bounded(x in -2.0f32..2.0, k in -1.0f32..1.0) {
            let mut out = std::vec![0.0f32; 8];
            curve(In::A(&std::vec![x; 8]), In::K(k), &mut out);
            for &v in &out {
                prop_assert!(v.is_finite() && v.abs() <= 1.0001, "curve unbounded: {v}");
            }
        }
    }
}
```

- [ ] **Step 2: Register the module in `lib.rs`**

In `crates/deluge-dsp-kernels/src/lib.rs`, add `pub mod shape;` in alphabetical
order (after `pub mod reverb;`, before `pub mod wavetable;` — i.e. `shape`
sorts after `reverb`; place it right before `pub mod wavetable;`).

- [ ] **Step 3: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels shape`
Expected: PASS — `curve_linear_at_k0`, `curve_fixed_points_and_midpoint_gain`,
`curve_odd_symmetric_and_monotonic`, `ctrl_holds_and_updates`,
`curve_stays_bounded`. No warnings.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-dsp-kernels/src/shape.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(dsp-kernels): Curve (Schlick bias) + Ctrl macro source"
```

---

### Task 2: `quant.rs` kernels — `QuantStep`, `QuantPitch`, `Mtof`

**Files:**
- Create: `crates/deluge-dsp-kernels/src/quant.rs`
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (add `pub mod quant;`)

**Interfaces:**
- Consumes: `crate::In`, the crate-root `crate::floorf`, `libm::exp2f`.
- Produces:
  - `pub struct QuantStep { levels: u16 }` — `new()` (levels 2),
    `set_levels(&mut self, n: u16)` (clamp ≥ 2), `process(&mut self, input: In, out: &mut [f32])`.
  - `pub struct QuantPitch { mask: u16, root: u8 }` — `new()` (mask `0xFFF`, root 0),
    `set_mask(&mut self, m: u16)`, `set_root(&mut self, r: u8)`,
    `process(&mut self, input: In, out: &mut [f32])`.
  - `pub struct Mtof { ref_hz: f32 }` — `new()` (ref 440), `set_ref(&mut self, hz: f32)`,
    `process(&mut self, input: In, out: &mut [f32])`.

- [ ] **Step 1: Create `quant.rs` with the kernels + tests**

Create `crates/deluge-dsp-kernels/src/quant.rs`:

```rust
//! Quantizers and pitch conversion: snap a signal to N equal levels
//! (`QuantStep`), snap a semitone signal to a musical scale (`QuantPitch`), and
//! convert semitones to Hz (`Mtof`). Per-sample mono kernels — no heap.

use crate::{floorf, In};

/// Snap a bipolar `[-1,1]` signal to `N` evenly-spaced levels (endpoints
/// inclusive). Port 0 = input; `param 0 = N` (clamp ≥ 2).
#[derive(Clone, Copy)]
pub struct QuantStep {
    levels: u16,
}
impl QuantStep {
    pub fn new() -> QuantStep {
        QuantStep { levels: 2 }
    }
    pub fn set_levels(&mut self, n: u16) {
        self.levels = n.max(2);
    }
    pub fn process(&mut self, input: In, out: &mut [f32]) {
        let n = self.levels.max(2) as f32;
        for i in 0..out.len() {
            let u = (input.at(i).clamp(-1.0, 1.0) + 1.0) * 0.5; // [0,1]
            let q = floorf(u * (n - 1.0) + 0.5); // nearest of N levels
            out[i] = (q / (n - 1.0)) * 2.0 - 1.0; // back to [-1,1]
        }
    }
}
impl Default for QuantStep {
    fn default() -> Self {
        Self::new()
    }
}

/// Snap a semitone-valued signal to the nearest degree of a scale. Port 0 =
/// input (semitones). `param 0 = 12-bit pitch-class mask` (bit i ⇒ pc i allowed,
/// root-relative), `param 1 = root` (0–11). Output in semitones.
#[derive(Clone, Copy)]
pub struct QuantPitch {
    mask: u16,
    root: u8,
}
impl QuantPitch {
    pub fn new() -> QuantPitch {
        QuantPitch { mask: 0xFFF, root: 0 }
    }
    pub fn set_mask(&mut self, m: u16) {
        self.mask = m & 0xFFF;
    }
    pub fn set_root(&mut self, r: u8) {
        self.root = r % 12;
    }
    pub fn process(&mut self, input: In, out: &mut [f32]) {
        // Search order resolves ties upward and covers a full octave of pitch
        // classes, so a non-empty mask always matches within ±6 semitones.
        const SEARCH: [i32; 13] = [0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6];
        for i in 0..out.len() {
            let n = floorf(input.at(i) + 0.5) as i32; // round, ties → +∞
            let mut best = n;
            for d in SEARCH {
                let cand = n + d;
                let pc = (cand - self.root as i32).rem_euclid(12) as u16;
                if self.mask == 0 || (self.mask & (1 << pc)) != 0 {
                    best = cand;
                    break;
                }
            }
            out[i] = best as f32;
        }
    }
}
impl Default for QuantPitch {
    fn default() -> Self {
        Self::new()
    }
}

/// Semitone → Hz: `ref · 2^(x/12)`. Port 0 = input (semitones above the
/// reference), `param 0 = reference Hz` (default 440).
#[derive(Clone, Copy)]
pub struct Mtof {
    ref_hz: f32,
}
impl Mtof {
    pub fn new() -> Mtof {
        Mtof { ref_hz: 440.0 }
    }
    pub fn set_ref(&mut self, hz: f32) {
        self.ref_hz = hz;
    }
    pub fn process(&mut self, input: In, out: &mut [f32]) {
        for i in 0..out.len() {
            out[i] = self.ref_hz * libm::exp2f(input.at(i) / 12.0);
        }
    }
}
impl Default for Mtof {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;

    const MAJOR: u16 = 0b101010110101; // {0,2,4,5,7,9,11}
    const MINOR: u16 = 0b010110101101; // {0,2,3,5,7,8,10}

    #[test]
    fn qstep_two_levels() {
        let mut q = QuantStep::new(); // N = 2 → {-1, +1}
        let mut out = std::vec![0.0f32; 4];
        q.process(In::A(&[-0.3, 0.3, -0.9, 0.9]), &mut out);
        assert_eq!(out, std::vec![-1.0, 1.0, -1.0, 1.0]);
    }

    #[test]
    fn qstep_three_levels() {
        let mut q = QuantStep::new();
        q.set_levels(3); // {-1, 0, +1}
        let mut out = std::vec![0.0f32; 4];
        q.process(In::A(&[-0.6, -0.1, 0.1, 0.6]), &mut out);
        assert_eq!(out, std::vec![-1.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn qstep_clamps_levels_min_two() {
        let mut q = QuantStep::new();
        q.set_levels(0); // clamp to 2
        let mut out = std::vec![0.0f32; 2];
        q.process(In::A(&[-0.4, 0.4]), &mut out);
        assert_eq!(out, std::vec![-1.0, 1.0]);
    }

    #[test]
    fn qpitch_major_root0_snaps() {
        let mut q = QuantPitch::new();
        q.set_mask(MAJOR);
        q.set_root(0);
        let mut out = std::vec![0.0f32; 5];
        //  1.0→2 (tie-up),  3.0→4,  6.4→7,  0.0→0,  11.6→12 (octave wrap)
        q.process(In::A(&[1.0, 3.0, 6.4, 0.0, 11.6]), &mut out);
        assert_eq!(out, std::vec![2.0, 4.0, 7.0, 0.0, 12.0]);
    }

    #[test]
    fn qpitch_root_shift_moves_grid() {
        let mut q = QuantPitch::new();
        q.set_mask(MAJOR);
        q.set_root(2); // D major: allowed pcs {2,4,6,7,9,11,1}
        let mut out = std::vec![0.0f32; 2];
        q.process(In::A(&[3.0, 5.0]), &mut out);
        assert_eq!(out, std::vec![4.0, 6.0], "3→4, 5→6 under root 2");
    }

    #[test]
    fn qpitch_chromatic_is_identity() {
        let mut q = QuantPitch::new(); // mask 0xFFF
        let mut out = std::vec![0.0f32; 3];
        q.process(In::A(&[5.2, -3.4, 7.5]), &mut out);
        assert_eq!(out, std::vec![5.0, -3.0, 8.0], "nearest integer semitone");
    }

    #[test]
    fn qpitch_minor_snaps_known() {
        let mut q = QuantPitch::new();
        q.set_mask(MINOR); // {0,2,3,5,7,8,10}
        let mut out = std::vec![0.0f32; 3];
        q.process(In::A(&[1.0, 4.0, 6.0]), &mut out);
        //  1→2, 4→3 or 5 (tie→up 5), 6→5 or 7 (tie→up 7)
        assert_eq!(out, std::vec![2.0, 5.0, 7.0]);
    }

    #[test]
    fn mtof_octaves_exact() {
        let mut m = Mtof::new(); // ref 440
        let mut out = std::vec![0.0f32; 3];
        m.process(In::A(&[0.0, 12.0, -12.0]), &mut out);
        assert!((out[0] - 440.0).abs() < 1e-2, "0 → ref: {}", out[0]);
        assert!((out[1] - 880.0).abs() < 1e-2, "+12 → 2×ref: {}", out[1]);
        assert!((out[2] - 220.0).abs() < 1e-2, "-12 → ½×ref: {}", out[2]);
    }

    #[test]
    fn mtof_set_ref() {
        let mut m = Mtof::new();
        m.set_ref(100.0);
        let mut out = std::vec![0.0f32; 2];
        m.process(In::A(&[0.0, 12.0]), &mut out);
        assert!((out[0] - 100.0).abs() < 1e-3 && (out[1] - 200.0).abs() < 1e-3);
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn qstep_bounded(x in -2.0f32..2.0, n in 2u16..64) {
            let mut q = QuantStep::new();
            q.set_levels(n);
            let mut out = std::vec![0.0f32; 8];
            q.process(In::A(&std::vec![x; 8]), &mut out);
            for &v in &out {
                prop_assert!(v.is_finite() && v.abs() <= 1.0001, "qstep unbounded: {v}");
            }
        }
        #[test]
        fn mtof_positive_finite(x in -72.0f32..72.0) {
            let mut m = Mtof::new();
            let mut out = std::vec![0.0f32; 4];
            m.process(In::A(&std::vec![x; 4]), &mut out);
            for &v in &out {
                prop_assert!(v.is_finite() && v > 0.0, "mtof non-positive: {v}");
            }
        }
    }
}
```

- [ ] **Step 2: Register the module in `lib.rs`**

In `crates/deluge-dsp-kernels/src/lib.rs`, add `pub mod quant;` in alphabetical
order (after `pub mod osc;`, before `pub mod reverb;`).

- [ ] **Step 3: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels quant`
Expected: PASS — all 8 unit tests + 2 proptests. No warnings.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-dsp-kernels/src/quant.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(dsp-kernels): QuantStep / QuantPitch / Mtof"
```

---

### Task 3: Graph nodes — `Curve`, `QuantStep`, `QuantPitch`, `Mtof`, `Ctrl`

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs`

**Interfaces:**
- Consumes: `deluge_dsp_kernels::shape::{self, Ctrl}`,
  `deluge_dsp_kernels::quant::{QuantStep, QuantPitch, Mtof}` (from Tasks 1–2).
- Produces: `Kind::Curve`, `Kind::QuantStep`, `Kind::QuantPitch`, `Kind::Mtof`,
  `Kind::Ctrl` (all mono, `out_width == 1`). `set_param`:
  `QuantStep 0 => set_levels`; `QuantPitch 0 => set_mask, 1 => set_root`;
  `Mtof 0 => set_ref`; `Ctrl 0 => set_value`; `Curve` has no params.

This mirrors the merged `Lfo`/`SampleHold`/`Slew`/`Steps` node wiring exactly.
Grep `Kind::Steps`/`State::Steps` in `node.rs` to see the four edit sites.

- [ ] **Step 1: Add the kernel imports**

In the `use deluge_dsp_kernels::{ ... }` block (currently ends with
`reverb::{...}`), add these two lines inside the braces (next to
`modutil::{SampleHold, Slew, Steps},`):

```rust
    quant::{Mtof, QuantPitch, QuantStep},
    shape::{self, Ctrl},
```

- [ ] **Step 2: Add the `Kind` variants**

In `pub enum Kind` (grep `Steps,` — it's the last Mod-2 entry), add after `Steps,`:

```rust
    Curve,
    QuantStep,
    QuantPitch,
    Mtof,
    Ctrl,
```

- [ ] **Step 3: Add the `State` variants**

In `enum State` (grep `Steps(Steps),`), add after it (before `Stateless,`):

```rust
    QuantStep(QuantStep),
    QuantPitch(QuantPitch),
    Mtof(Mtof),
    Ctrl(Ctrl),
```

(`Curve` is stateless — it uses `State::Stateless`, no variant of its own.)

- [ ] **Step 4: Add `Node::new` construction arms**

`Curve` joins the stateless arm. Change the existing line
`Kind::Mul | Kind::Add | Kind::Sub | Kind::Split2 | Kind::Pan => State::Stateless,`
to include `Kind::Curve`:

```rust
            Kind::Mul | Kind::Add | Kind::Sub | Kind::Split2 | Kind::Pan | Kind::Curve => State::Stateless,
```

Then after `Kind::Steps => State::Steps(Steps::new()),` add:

```rust
            Kind::QuantStep => State::QuantStep(QuantStep::new()),
            Kind::QuantPitch => State::QuantPitch(QuantPitch::new()),
            Kind::Mtof => State::Mtof(Mtof::new()),
            Kind::Ctrl => State::Ctrl(Ctrl::new()),
```

(`out_width` needs no change — all five fall through the default `_ => 1` arm.)

- [ ] **Step 5: Add `set_param` arms**

In `set_param`, after the `State::Steps(s) => match param { ... },` arm (before
the final `_ => {}`), add:

```rust
            State::QuantStep(q) => match param {
                0 => q.set_levels(value as u16),
                _ => {}
            },
            State::QuantPitch(q) => match param {
                0 => q.set_mask(value as u16),
                1 => q.set_root(value as u8),
                _ => {}
            },
            State::Mtof(m) => match param {
                0 => m.set_ref(value),
                _ => {}
            },
            State::Ctrl(c) => match param {
                0 => c.set_value(value),
                _ => {}
            },
```

- [ ] **Step 6: Add render arms**

In `process_resolved`'s `match kind` (grep the `Kind::Steps => { ... }` render
arm — it's the last one before the closing brace), add after it:

```rust
            Kind::Curve => shape::curve(ins[0], ins[1], outs.port(0)),
            Kind::QuantStep => {
                if let State::QuantStep(q) = &mut self.state {
                    q.process(ins[0], outs.port(0));
                }
            }
            Kind::QuantPitch => {
                if let State::QuantPitch(q) = &mut self.state {
                    q.process(ins[0], outs.port(0));
                }
            }
            Kind::Mtof => {
                if let State::Mtof(m) = &mut self.state {
                    m.process(ins[0], outs.port(0));
                }
            }
            Kind::Ctrl => {
                if let State::Ctrl(c) = &mut self.state {
                    c.process(outs.port(0));
                }
            }
```

- [ ] **Step 7: Add the graph behavioral tests**

In `node.rs`'s `#[cfg(test)] mod tests`, after `fn steps_node_sequences()`
(grep it), add:

```rust
    #[test]
    fn curve_node_is_identity_at_k0() {
        let mut n = Node::new(Kind::Curve, 0);
        assert_eq!(Node::out_width(Kind::Curve), 1);
        let input: [f32; 8] = core::array::from_fn(|i| i as f32 / 7.0 * 2.0 - 1.0);
        let k = [0.0f32; 8];
        let ins = [In::A(&input), In::A(&k), In::A(&[0.0; 8])];
        let mut buf = [0.0f32; 8];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        for i in 0..8 {
            assert!((buf[i] - input[i]).abs() < 1e-6, "k=0 identity at {i}");
        }
    }

    #[test]
    fn ctrl_node_holds_param_value() {
        let mut n = Node::new(Kind::Ctrl, 0);
        assert_eq!(Node::out_width(Kind::Ctrl), 1);
        n.set_param(0, 2.5);
        let ins = [In::A(&[0.0; 4]), In::A(&[0.0; 4]), In::A(&[0.0; 4])];
        let mut buf = [0.0f32; 4];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert_eq!(buf, [2.5, 2.5, 2.5, 2.5]);
    }

    #[test]
    fn qstep_node_snaps_to_levels() {
        let mut n = Node::new(Kind::QuantStep, 0);
        n.set_param(0, 2.0); // {-1, +1}
        let input = [-0.4f32, 0.4, -0.9, 0.9];
        let ins = [In::A(&input), In::A(&[0.0; 4]), In::A(&[0.0; 4])];
        let mut buf = [0.0f32; 4];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert_eq!(buf, [-1.0, 1.0, -1.0, 1.0]);
    }

    #[test]
    fn qpitch_node_snaps_major() {
        let mut n = Node::new(Kind::QuantPitch, 0);
        n.set_param(0, 2741.0); // 0b101010110101 major
        n.set_param(1, 0.0); // root 0
        let input = [1.0f32, 3.0, 6.4];
        let ins = [In::A(&input), In::A(&[0.0; 3]), In::A(&[0.0; 3])];
        let mut buf = [0.0f32; 3];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert_eq!(buf, [2.0, 4.0, 7.0]);
    }

    #[test]
    fn mtof_node_converts_octaves() {
        let mut n = Node::new(Kind::Mtof, 0);
        n.set_param(0, 440.0);
        let input = [0.0f32, 12.0, -12.0];
        let ins = [In::A(&input), In::A(&[0.0; 3]), In::A(&[0.0; 3])];
        let mut buf = [0.0f32; 3];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!((buf[0] - 440.0).abs() < 1e-2 && (buf[1] - 880.0).abs() < 1e-2 && (buf[2] - 220.0).abs() < 1e-2);
    }
```

- [ ] **Step 8: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: PASS — the full crate suite plus the 5 new node tests
(`curve_node_is_identity_at_k0`, `ctrl_node_holds_param_value`,
`qstep_node_snaps_to_levels`, `qpitch_node_snaps_major`,
`mtof_node_converts_octaves`). No warnings from `node.rs`.

- [ ] **Step 9: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(audio-graph): Curve / QuantStep / QuantPitch / Mtof / Ctrl nodes"
```

---

### Task 4: Wren surface — factories, `value=` setter, prelude sugar

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (factory impls + `register_audio`)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (wren-sys static table)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (classes + sugar)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Kind::{Curve, QuantStep, QuantPitch, Mtof, Ctrl}` from Task 3;
  helpers `arg_input(vm, slot)`, `vm.get_f(slot)`, `self_id(vm)`,
  `audio::{alloc_node_id, new_node, set_param}`, `return_node(vm, id)`.
- Produces Wren API: `Node.curve_(input, k)`, `Node.ctrl_(value)`,
  `Node.qstep_(input, n)`, `Node.qpitch_(input, mask, root)`,
  `Node.mtof_(input, ref)`, setter `Node.value=(v)`; prelude classes
  `Curve`, `Macro`, `Scale` and sugar methods on `Node`/`Port`.

- [ ] **Step 1: Add the factory + setter impls in `bindings_audio.rs`**

After `node_steps_impl` (and its `#[cfg(feature = "wren-sys-backend")]`
`node_steps` wrapper), add — mirroring `node_sh_impl`/`node_lfo_impl`/
`node_set_phase_impl` exactly (generic impl + wren-sys extern wrapper each):

```rust
/// `Node.curve_(input, k)` — odd-symmetric Schlick-bias transfer. Ports 0=input,
/// 1=k (∈[-1,1], modulatable). Stateless (`Kind::Curve`).
pub(crate) fn node_curve_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let k = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Curve, [input, k, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_curve(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_curve_impl(&vm);
}

/// `Node.ctrl_(value)` — a settable scalar macro source. No inputs; `param 0 = value`.
pub(crate) fn node_ctrl_impl<S: SlotApi>(vm: &S) {
    let value = vm.get_f(1) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Ctrl, [Input::Const(0.0); 3]);
    audio::set_param(id, 0, value);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_ctrl(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_ctrl_impl(&vm);
}

/// `Node.qstep_(input, n)` — snap to N equal levels. Port 0=input, `param 0 = N`.
pub(crate) fn node_qstep_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let n = vm.get_f(2) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::QuantStep, [input, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, n);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_qstep(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_qstep_impl(&vm);
}

/// `Node.qpitch_(input, mask, root)` — snap semitones to a scale. Port 0=input,
/// `param 0 = 12-bit mask`, `param 1 = root`.
pub(crate) fn node_qpitch_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let mask = vm.get_f(2) as f32;
    let root = vm.get_f(3) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::QuantPitch, [input, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, mask);
    audio::set_param(id, 1, root);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_qpitch(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_qpitch_impl(&vm);
}

/// `Node.mtof_(input, ref)` — semitone → Hz. Port 0=input, `param 0 = ref Hz`.
pub(crate) fn node_mtof_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let reference = vm.get_f(2) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Mtof, [input, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, reference);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_mtof(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_mtof_impl(&vm);
}

/// `macro.value = v` — set a `Ctrl` node's held value (`param 0`).
pub(crate) fn node_set_value_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32;
    audio::set_param(self_id(vm), 0, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_value(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_value_impl(&vm);
}
```

- [ ] **Step 2: Register in `register_audio` (`bindings_audio.rs`)**

After the `steps_(_,_)` registration line
(`method("main", "Node", true, "steps_(_,_)", node_steps_impl::<S>);`), add:

```rust
    method("main", "Node", true, "curve_(_,_)", node_curve_impl::<S>);
    method("main", "Node", true, "ctrl_(_)", node_ctrl_impl::<S>);
    method("main", "Node", true, "qstep_(_,_)", node_qstep_impl::<S>);
    method("main", "Node", true, "qpitch_(_,_,_)", node_qpitch_impl::<S>);
    method("main", "Node", true, "mtof_(_,_)", node_mtof_impl::<S>);
    method("main", "Node", false, "value=(_)", node_set_value_impl::<S>);
```

- [ ] **Step 3: Register in the wren-sys static table (`bindings.rs`)**

After the `steps_(_,_)` line
(`static_method("Node", "steps_(_,_)", bindings_audio::node_steps),`), add:

```rust
    static_method("Node", "curve_(_,_)", bindings_audio::node_curve),
    static_method("Node", "ctrl_(_)", bindings_audio::node_ctrl),
    static_method("Node", "qstep_(_,_)", bindings_audio::node_qstep),
    static_method("Node", "qpitch_(_,_,_)", bindings_audio::node_qpitch),
    static_method("Node", "mtof_(_,_)", bindings_audio::node_mtof),
    method("Node", "value=(_)", bindings_audio::node_set_value),
```

- [ ] **Step 4: Add the prelude classes + sugar (`prelude.wren`)**

Add the shaping sugar methods to **both** `class Node` and `foreign class Port`.
In each, after the existing `to(lo, hi) { ... }` line (before the closing `}`),
add:

```wren
  atten(k)       { this * k }
  offset(c)      { this + c }
  invert()       { this * -1 }
  unipolar()     { this * 0.5 + 0.5 }   // [-1,1] → [0,1]
  bipolar()      { this * 2 - 1 }       // [0,1] → [-1,1]
  scale(m, a)    { this * m + a }
  curve(k)       { Node.curve_(this, k) }
  steps(n)       { Node.qstep_(this, n) }
  quantize(s, r) { Node.qpitch_(this, s, r) }
  hz(ref)        { Node.mtof_(this, ref) }
```

Then, after the `class Steps { ... }` block, add the three new classes:

```wren
// Non-linear response curve (Schlick bias), odd-symmetric on [-1,1].
//   var shaped = env.curve(0.6)          // ease-in
//   Out.patch(Osc.saw(110) * env.curve(-0.4))
class Curve {
  static new(sig, k) { Node.curve_(sig, k) }
  static exp(sig)    { Node.curve_(sig, 0.6) }
  static log(sig)    { Node.curve_(sig, -0.6) }
}

// A macro control: one settable value that fans out to many destinations
// (each via its own `* depth`/`.to(...)`). Drive it live from an encoder:
//   var m = Macro.new(0.5)
//   filter.cutoff = m.to(200, 2000)
//   osc.width = m * 0.3 + 0.5
//   Enc.onTurn = Fn.new { |i, d| m.value = (m.value + d * 0.05) }
class Macro {
  static new(v) { Node.ctrl_(v) }
}

// Musical scales as 12-bit pitch-class masks (bit i ⇒ pc i allowed, relative to
// the root passed to `.quantize`). Use with a semitone signal:
//   var note = seq.to(0, 24).quantize(Scale.Minor, 0)
//   osc.freq = note.hz(220)
class Scale {
  static Chromatic        { 4095 }  // 0xFFF, {0..11}
  static Major            { 2741 }  // {0,2,4,5,7,9,11}
  static Minor            { 1453 }  // {0,2,3,5,7,8,10}  (natural)
  static HarmonicMinor    { 2477 }  // {0,2,3,5,7,8,11}
  static Dorian           { 1709 }  // {0,2,3,5,7,9,10}
  static Mixolydian       { 1717 }  // {0,2,4,5,7,9,10}
  static MajorPentatonic  { 661 }   // {0,2,4,7,9}
  static MinorPentatonic  { 1193 }  // {0,3,5,7,10}
  static WholeTone        { 1365 }  // {0,2,4,6,8,10}
  static Blues            { 1257 }  // {0,3,5,6,7,10}
}
```

Note: `value=` is a setter on `Node`, so `m.value = x` works on the node
`Macro.new` returns; no method is needed on the `Macro` class itself.

- [ ] **Step 5: Add the Wren-surface tests (`tests/audio_bindings.rs`)**

Add these tests (mirroring the existing `steps_factory_emits_len_and_values` /
`sample_hold_and_slew_factories_emit_nodes` style — grep them for the exact
`run_and_capture_cmds` / `run_and_render` / `Cmd` usage):

```rust
#[test]
fn shaping_factories_emit_nodes() {
    let cmds = run_and_capture_cmds(
        "var m = Macro.new(0.5)\nvar c = Curve.exp(m)\nvar q = m.quantize(Scale.Major, 0)\nvar f = q.hz(220)\nvar s = m.steps(4)",
    );
    // Ctrl (macro) with SetParam(0, 0.5)
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Ctrl, .. })), "Ctrl node");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if (*value - 0.5).abs() < 1e-6)), "Ctrl value 0.5");
    // Curve node (from Curve.exp)
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Curve, .. })), "Curve node");
    // QuantPitch with mask 2741 (Scale.Major) + root 0
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::QuantPitch, .. })), "QuantPitch node");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if (*value - 2741.0).abs() < 0.5)), "major mask 2741");
    // Mtof with ref 220 + QuantStep with N 4
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Mtof, .. })), "Mtof node");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::QuantStep, .. })), "QuantStep node");
}

#[test]
fn macro_value_setter_emits_setparam() {
    let cmds = run_and_capture_cmds("var m = Macro.new(0.0)\nm.value = 0.75");
    // One NewNode(Ctrl) + SetParam(0, 0.0) at build, then SetParam(0, 0.75) from the setter.
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if (*value - 0.75).abs() < 1e-6)), "value= → SetParam(0, 0.75)");
}

#[test]
fn pitch_chain_renders_bounded_nonsilent() {
    // seq (a bipolar LFO) → range → quantize (minor) → hz → osc.freq
    let mut out = [StereoFrame::default(); 64];
    run_and_render(
        "var note = LFO.saw(4).to(0, 24).quantize(Scale.Minor, 0)\nOut.patch(Osc.saw(note.hz(110)))",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0), "finite/bounded");
    assert!(out.iter().any(|f| f.l != 0.0), "non-silent");
}

#[test]
fn curve_sugar_renders_bounded() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Osc.saw(110) * Env.ar(0.0, 0.1).curve(0.6))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0), "finite/bounded");
}
```

- [ ] **Step 6: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core`
Expected: PASS — the full `audio_bindings` suite plus the 4 new tests. No
warnings from this crate (pre-existing `deluge-fft` dead-code warnings are
unrelated).

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): Curve / Macro / Scale + quantize/hz/scaling sugar"
```

---

## Notes for the implementer

- The whole feature is additive; no existing behavior changes. If a step's
  "add after X" anchor has moved, grep for the named symbol — the *relationship*
  (add next to the Mod-2 equivalent) is what matters, not line numbers.
- `Curve` is the only stateless node here: it has **no** `State::Curve` variant,
  **no** `set_param` arm, and its render arm calls the free function
  `shape::curve(...)` directly (exactly like `Kind::Mul => math::mul(...)`).
- `libm::exp2f` is exact — the `Mtof` tests use tolerances only to absorb `f32`
  rounding, not approximation error.
- Do not add setters for Curve-k / Mtof-ref / QuantStep-n / mask / root (they are
  construction-time for v1, per the spec). Only `value=` (Ctrl) is a new setter.
