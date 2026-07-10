# Sy-2b: Poly Breadth Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Poly-ify the core-subtractive kernels — band-limited `PolyOsc` waveshapes, `PolyAdd`, `PolyNoise` — and flip the Wren `Synth` `Osc.saw/square/tri` / `+` / `Noise` aborts into working routes.

**Architecture:** `PolyOsc` gains a `shape`; its scalar path reuses the mono `Osc`'s band-limited `wave_sample` (the oracle), and a new branchless `f32x8` `wave_sample_x8` (with `poly_blep_x8`/`blep_right_x8`/`poly_blamp_x8` in `osc.rs`) is the fast path, null-tested. `PolyAdd`/`PolyNoise` are small kernels. The `polyosc_` factory gains a shape arg; the prelude routes flip.

**Tech Stack:** Rust `no_std` (`deluge-dsp-kernels`), `core::simd`, the audio graph, Wren (prelude).

## Global Constraints

- `no_std`, no heap, pure `f32`, deterministic. Voice-interleaved tiles `tile[i*VOICES+v]`.
- Host tests per-crate on x86: `cargo test --target x86_64-unknown-linux-gnu -p <crate>`; kernels/graph/wren-core also `--features [deluge-dsp-kernels/]simd`. Never `--workspace`.
- LSP armv7a "can't find crate for test/std" diagnostics are noise — ignore.
- The `f32x8` band-limiting is null-tested against the scalar `wave_sample` oracle (the `math.rs`/`PolyOsc` convention). `#[cfg(feature="simd")] const _: () = assert!(VOICES == 8)` already guards `PolyOsc`.
- Square PWM fixed at **0.5**; poly Noise is **white only**; `Noise.pink()`/`.brown()`, poly Sync/Wavetable, and the poly filters stay aborting (Sy-2c).
- Wren gotchas (from Sy-4): no `?:` ternary (use `if/return`); `0` is truthy → branch on `Node.polyMode_ == 1`.
- Register new foreign methods in BOTH tables (`bindings.rs` METHODS + `register_audio`) + prelude `foreign static` decls.

---

### Task 1: `f32x8` band-limiting in `osc.rs`

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/osc.rs`

**Interfaces:**
- Consumes: the private scalar `poly_blep`/`blep_right`/`poly_blamp`/`wave_sample`, `Wave`, `fast_sin_x8`, `floorf`.
- Produces: `pub(crate) fn wave_sample(wave: Wave, ph: f32, dtp: f32, width: f32) -> f32` (visibility widened); `#[cfg(feature="simd")] pub(crate) fn wave_sample_x8(wave: Wave, ph: f32x8, dtp: f32x8) -> f32x8` (Square width 0.5). Both used by `PolyOsc` (Task 2).

- [ ] **Step 1: Widen `wave_sample` visibility**

Change `fn wave_sample(...)` to `pub(crate) fn wave_sample(...)` in `osc.rs` (no
behavior change — `Osc`/`SyncOsc` still use it). The sub-helpers
(`poly_blep`/`blep_right`/`poly_blamp`) stay private.

- [ ] **Step 2: Add the branchless `f32x8` band-limiting helpers**

Add to `osc.rs` (all `#[cfg(feature = "simd")]`; use `core::simd::prelude::*`):

```rust
#[cfg(feature = "simd")]
use core::simd::prelude::*;

/// SIMD trunc-floor matching scalar `floorf` (handles negatives).
#[cfg(feature = "simd")]
#[inline]
fn floor_x8(x: f32x8) -> f32x8 {
    let t: f32x8 = x.cast::<i32>().cast::<f32>();
    t.simd_gt(x).select(t - f32x8::splat(1.0), t)
}

/// f32x8 counterpart of `blep_right` (two quartic pieces via select).
#[cfg(feature = "simd")]
#[inline]
fn blep_right_x8(x: f32x8) -> f32x8 {
    let lo = x * (x * x * (x * f32x8::splat(0.25) - f32x8::splat(2.0 / 3.0)) + f32x8::splat(4.0 / 3.0)) - f32x8::splat(1.0);
    let hi = x * (x * (x * (x * f32x8::splat(-1.0 / 12.0) + f32x8::splat(2.0 / 3.0)) - f32x8::splat(2.0)) + f32x8::splat(8.0 / 3.0)) - f32x8::splat(4.0 / 3.0);
    x.simd_lt(f32x8::splat(1.0)).select(lo, hi)
}

/// f32x8 counterpart of `poly_blep` (left/right/zero via select; left wins,
/// matching the scalar `if/else if`).
#[cfg(feature = "simd")]
#[inline]
fn poly_blep_x8(t: f32x8, dtp: f32x8) -> f32x8 {
    let one = f32x8::splat(1.0);
    let w = f32x8::splat(2.0) * dtp;
    let left = blep_right_x8(t / dtp);
    let right = -blep_right_x8(-(t - one) / dtp);
    let lm = t.simd_lt(w);
    let rm = t.simd_gt(one - w);
    lm.select(left, rm.select(right, f32x8::splat(0.0)))
}

/// f32x8 counterpart of `poly_blamp` (two cubic pieces via select).
#[cfg(feature = "simd")]
#[inline]
fn poly_blamp_x8(t: f32x8, dtp: f32x8) -> f32x8 {
    let one = f32x8::splat(1.0);
    let xa = t / dtp - one;
    let a = f32x8::splat(-1.0 / 3.0) * xa * xa * xa;
    let xb = (t - one) / dtp + one;
    let b = f32x8::splat(1.0 / 3.0) * xb * xb * xb;
    let am = t.simd_lt(dtp);
    let bm = t.simd_gt(one - dtp);
    am.select(a, bm.select(b, f32x8::splat(0.0)))
}

/// f32x8 band-limited waveform (Square PWM fixed 0.5). Matches scalar
/// `wave_sample(..., 0.5)` lane-for-lane.
#[cfg(feature = "simd")]
#[inline]
pub(crate) fn wave_sample_x8(wave: Wave, ph: f32x8, dtp: f32x8) -> f32x8 {
    let one = f32x8::splat(1.0);
    match wave {
        Wave::Sine => crate::fast_sin_x8(ph),
        Wave::Saw => (f32x8::splat(2.0) * ph - one) - poly_blep_x8(ph, dtp),
        Wave::Square => {
            let half = f32x8::splat(0.5);
            let naive = ph.simd_lt(half).select(one, -one);
            let mut pw = ph - half;
            pw -= floor_x8(pw);
            naive + poly_blep_x8(ph, dtp) - poly_blep_x8(pw, dtp)
        }
        Wave::Tri => {
            let half = f32x8::splat(0.5);
            let naive = one - f32x8::splat(4.0) * (ph - half).abs();
            let mut p2 = ph + half;
            p2 -= floor_x8(p2);
            naive + f32x8::splat(8.0) * dtp * (poly_blamp_x8(ph, dtp) - poly_blamp_x8(p2, dtp))
        }
    }
}
```

- [ ] **Step 3: Null test — `wave_sample_x8` matches scalar `wave_sample`**

In `osc.rs`'s `#[cfg(test)] mod tests` (add if absent; it likely has osc tests), add:

```rust
    #[cfg(feature = "simd")]
    #[test]
    fn wave_sample_x8_matches_scalar() {
        use core::simd::f32x8;
        let dt = 1.0 / 48_000.0;
        for &shape in &[Wave::Sine, Wave::Saw, Wave::Square, Wave::Tri] {
            for fk in 0..40 {
                let freq = 55.0 + fk as f32 * 200.0; // 55 Hz .. ~8 kHz
                let dtp = freq * dt;
                for pk in 0..97 {
                    let ph = pk as f32 / 97.0; // sweep [0,1)
                    let s = wave_sample(shape, ph, dtp, 0.5);
                    let v = wave_sample_x8(shape, f32x8::splat(ph), f32x8::splat(dtp)).to_array()[0];
                    assert!((v - s).abs() < 1e-4, "shape {:?} f {freq} ph {ph}: {v} vs {s}", shape as u8);
                }
            }
        }
    }
```

(`Wave` derives `Clone, Copy`; `shape as u8` for the message — if `Wave` isn't
`#[repr(u8)]`, drop the `as u8` and print the loop index instead.)

- [ ] **Step 4: Run both configs + commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels osc` (scalar — the new test is cfg'd out here, existing osc tests pass).
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd osc` — PASS incl. `wave_sample_x8_matches_scalar`. No warnings.

```bash
git add crates/deluge-dsp-kernels/src/osc.rs
git commit -m "feat(dsp-kernels): f32x8 band-limited waveforms (wave_sample_x8) + expose wave_sample"
```

---

### Task 2: `PolyOsc` shape + `poly_add` + `PolyNoise`

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/poly.rs`

**Interfaces:**
- Consumes: `crate::osc::{wave_sample, Wave}` (scalar), `crate::osc::wave_sample_x8` (simd), `poly::VOICES`, `crate::floorf`.
- Produces: `PolyOsc` with `set_shape(code: u8)` + shape-aware `process`; `pub fn poly_add(a: &[f32], b: &[f32], out: &mut [f32])`; `PolyNoise` with `new()`/`process(out)`.

- [ ] **Step 1: `PolyOsc` gains a shape**

Add `use crate::osc::{wave_sample, Wave};` and `#[cfg(feature = "simd")] use crate::osc::wave_sample_x8;`. Replace `PolyOsc`:

```rust
#[derive(Clone, Copy)]
pub struct PolyOsc {
    phase: [f32; VOICES],
    shape: Wave,
}
impl PolyOsc {
    pub fn new() -> PolyOsc {
        PolyOsc { phase: [0.0; VOICES], shape: Wave::Sine }
    }
    pub fn set_shape(&mut self, code: u8) {
        self.shape = match code {
            1 => Wave::Saw,
            2 => Wave::Square,
            3 => Wave::Tri,
            _ => Wave::Sine,
        };
    }
    pub fn process(&mut self, pitch: &[f32], dt: f32, out: &mut [f32]) {
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            let n = out.len() / VOICES;
            let one = f32x8::splat(1.0);
            let dtv = f32x8::splat(dt);
            let mut ph = f32x8::from_array(self.phase);
            for i in 0..n {
                let f = f32x8::from_slice(&pitch[i * VOICES..]);
                let dtp = f * dtv;
                let mut p = ph + dtp;
                let t: f32x8 = p.cast::<i32>().cast::<f32>();
                let fl = t.simd_gt(p).select(t - one, t);
                p -= fl;
                ph = p;
                wave_sample_x8(self.shape, p, dtp).copy_to_slice(&mut out[i * VOICES..]);
            }
            self.phase = ph.to_array();
        }
        #[cfg(not(feature = "simd"))]
        {
            let n = out.len() / VOICES;
            for i in 0..n {
                for v in 0..VOICES {
                    let f = pitch[i * VOICES + v];
                    let dtp = f * dt;
                    let mut p = self.phase[v] + dtp;
                    p -= floorf(p);
                    self.phase[v] = p;
                    out[i * VOICES + v] = wave_sample(self.shape, p, dtp, 0.5);
                }
            }
        }
    }
}
// Default unchanged
```

- [ ] **Step 2: `poly_add` + `PolyNoise`**

```rust
/// out[j] = a[j] + b[j]. Two poly inputs. Stateless (auto-vectorizes).
pub fn poly_add(a: &[f32], b: &[f32], out: &mut [f32]) {
    for j in 0..out.len() {
        out[j] = a[j] + b[j];
    }
}

/// 8 independent xorshift white-noise lanes with decorrelated per-voice seeds.
/// A poly source (no input). Scalar (noise is cheap, non-band-limited).
#[derive(Clone, Copy)]
pub struct PolyNoise {
    rng: [u32; VOICES],
}
impl PolyNoise {
    pub fn new() -> PolyNoise {
        let mut rng = [0u32; VOICES];
        for v in 0..VOICES {
            let s = 0x2545_F491u32 ^ 0x9E37_79B9u32.wrapping_mul(v as u32 + 1);
            rng[v] = if s == 0 { 0x2545_F491 } else { s }; // xorshift must be nonzero
        }
        PolyNoise { rng }
    }
    pub fn process(&mut self, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            for v in 0..VOICES {
                let mut r = self.rng[v];
                r ^= r << 13;
                r ^= r >> 17;
                r ^= r << 5;
                self.rng[v] = r;
                out[i * VOICES + v] = (r as i32 as f32) / (i32::MAX as f32); // matches noise.rs white
            }
        }
    }
}
impl Default for PolyNoise {
    fn default() -> Self { Self::new() }
}
```

- [ ] **Step 3: Tests**

```rust
    #[test]
    fn polyosc_saw_matches_mono_osc() {
        // A PolyOsc saw voice equals the mono Osc saw on the same frequency
        // (both go through wave_sample) — proves band-limiting is reused.
        use crate::osc::{Osc, Wave};
        let dt = 1.0 / 48_000.0;
        let n = 400;
        let freq = 220.0;
        let mut po = PolyOsc::new();
        po.set_shape(1); // Saw
        let mut out = std::vec![0.0f32; VOICES * n];
        po.process(&std::vec![freq; VOICES * n], dt, &mut out);
        // Mono reference: Osc::process(wave, freq, pmod, width, dt, out).
        let mut mono = Osc::new();
        let mut mref = std::vec![0.0f32; n];
        mono.process(Wave::Saw, In::K(freq), In::K(0.0), In::K(0.0), dt, &mut mref);
        for i in 0..n {
            assert!((out[i * VOICES] - mref[i]).abs() < 1e-4, "sample {i}");
        }
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
    }

    #[test]
    fn polynoise_lanes_independent_and_bounded() {
        let mut nz = PolyNoise::new();
        let n = 64;
        let mut out = std::vec![0.0f32; VOICES * n];
        nz.process(&mut out);
        assert!(out.iter().all(|&s| s.is_finite() && s.abs() <= 1.0), "bounded");
        assert!(out.iter().any(|&s| s != 0.0), "non-silent");
        // Decorrelated: lane 0 and lane 1 differ.
        assert!((0..n).any(|i| out[i * VOICES] != out[i * VOICES + 1]), "lanes decorrelated");
    }

    #[test]
    fn poly_add_lanewise() {
        let a: [f32; VOICES * 2] = core::array::from_fn(|j| j as f32);
        let b: [f32; VOICES * 2] = core::array::from_fn(|_| 10.0);
        let mut out = [0.0f32; VOICES * 2];
        poly_add(&a, &b, &mut out);
        for j in 0..VOICES * 2 {
            assert_eq!(out[j], j as f32 + 10.0);
        }
    }
```

(The `In::K(0.0)` `width` arg makes the mono `Osc` clamp width→0.5; `Saw` ignores
width anyway. Both accumulate phase by `freq·dt` from 0, so their `wave_sample`
saws match.)

- [ ] **Step 4: Run both configs + commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels poly` (scalar).
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd poly` — the saw test now exercises `wave_sample_x8` through `PolyOsc`. Both PASS, no warnings.

```bash
git add crates/deluge-dsp-kernels/src/poly.rs
git commit -m "feat(dsp-kernels): PolyOsc waveshapes + poly_add + PolyNoise"
```

---

### Task 3: Graph nodes — `PolyAdd` / `PolyNoise` + `PolyOsc` shape

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs`

**Interfaces:**
- Consumes: `deluge_dsp_kernels::poly::{poly_add, PolyNoise}` + `PolyOsc::set_shape` (Task 2).
- Produces: `Kind::PolyAdd`/`PolyNoise`; `poly_in_count(PolyAdd)==2`, `(PolyNoise)==0`, both `out_width==VOICES`, `is_poly`; `PolyOsc` `set_param` `0 => set_shape`.

- [ ] **Step 1: Kinds/State/new/width/predicates**

Extend the `use deluge_dsp_kernels::{ ... poly::{...} }` line with `poly_add, PolyNoise`. `enum Kind`: add `PolyAdd`, `PolyNoise`. `enum State`: add `PolyNoise(PolyNoise)` (PolyAdd is stateless).

`Node::new`: add `Kind::PolyAdd` to the `State::Stateless` arm; add `Kind::PolyNoise => State::PolyNoise(PolyNoise::new()),`.

`out_width`: append `| Kind::PolyAdd | Kind::PolyNoise` to the `=> VOICES,` arm.

`is_poly`: append `| Kind::PolyAdd | Kind::PolyNoise`.

`poly_in_count`: add `Kind::PolyAdd => 2,` (alongside `PolyMul => 2`); `PolyNoise` falls through `_ => 0`.

- [ ] **Step 2: `set_param` for `PolyOsc` + render arms**

`set_param`: add
```rust
            State::PolyOsc(o) => match param {
                0 => o.set_shape(value as u8),
                _ => {}
            },
```

`poly_process`: add
```rust
            Kind::PolyAdd => {
                if let (Some(a), Some(b)) = (poly_in[0], poly_in[1]) {
                    poly_add(a, b, out);
                }
            }
            Kind::PolyNoise => {
                if let State::PolyNoise(nz) = &mut self.state {
                    nz.process(out);
                }
            }
```

`process_resolved` poly no-op arm: append `| Kind::PolyAdd | Kind::PolyNoise`.

- [ ] **Step 3: Graph tests**

```rust
    #[test]
    fn polyosc_shape_and_new_kinds() {
        assert_eq!(Node::poly_in_count(Kind::PolyAdd), 2);
        assert_eq!(Node::poly_in_count(Kind::PolyNoise), 0);
        assert_eq!(Node::out_width(Kind::PolyAdd), VOICES);
        assert_eq!(Node::out_width(Kind::PolyNoise), VOICES);
        // PolyOsc set_param(0, code) selects the shape (a saw is band-limited,
        // so a mid-phase sample differs from the sine at the same phase).
        let mut n = Node::new(Kind::PolyOsc, 0);
        n.set_param(0, 1.0); // Saw
        let pitch = [220.0f32; VOICES * 4];
        let ins = [In::A(&[0.0; VOICES * 4]); MAX_INPUTS];
        let mut out = [0.0f32; VOICES * 4];
        n.poly_process(&ins, [Some(&pitch), None], 1.0 / 48_000.0, &mut out);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
        assert!(out.iter().any(|&s| s != 0.0), "saw renders");
    }

    #[test]
    fn poly_noise_node_renders_bounded() {
        let mut n = Node::new(Kind::PolyNoise, 0);
        let ins = [In::A(&[0.0; VOICES * 4]); MAX_INPUTS];
        let mut out = [0.0f32; VOICES * 4];
        n.poly_process(&ins, [None, None], 1.0 / 48_000.0, &mut out);
        assert!(out.iter().all(|&s| s.is_finite() && s.abs() <= 1.0) && out.iter().any(|&s| s != 0.0));
    }
```

- [ ] **Step 4: Run both configs + commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph` and `... --features deluge-dsp-kernels/simd` — both PASS, no warnings.

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(audio-graph): PolyAdd / PolyNoise nodes + PolyOsc shape param"
```

---

### Task 4: Wren surface — `polyosc_` shape arg + `polyadd_`/`polynoise_` + prelude flips

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs`, `crates/deluge-wren-core/src/bindings.rs`, `crates/deluge-wren-core/wren/prelude.wren`
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Kind::{PolyAdd, PolyNoise}`, `PolyOsc` `set_param(0, shape)` (Task 3).
- Produces: `polyosc_(pitch, shape)` (arity change), `polyadd_(a, b)`, `polynoise_()`; prelude routes for `Osc.*`/`+`/`Noise.new` under `polyMode_ == 1`.

- [ ] **Step 1: Extend `node_polyosc_impl` (shape arg) + add factories**

Change `node_polyosc_impl` to read a shape and set it:
```rust
pub(crate) fn node_polyosc_impl<S: SlotApi>(vm: &S) {
    let pitch = arg_input(vm, 1);
    let shape = vm.get_f(2) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolyOsc, [pitch, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, shape);
    unsafe { return_node(vm, id) };
}
```

Add `node_polyadd_impl` + `node_polynoise_impl` (each + its `#[cfg(feature = "wren-sys-backend")]` extern wrapper, mirroring `node_polymul`):
```rust
pub(crate) fn node_polyadd_impl<S: SlotApi>(vm: &S) {
    let a = arg_input(vm, 1);
    let b = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolyAdd, [a, b, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
pub(crate) fn node_polynoise_impl<S: SlotApi>(vm: &S) {
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolyNoise, [Input::Const(0.0); 3]);
    unsafe { return_node(vm, id) };
}
```
(Add the `node_polyadd`/`node_polynoise` extern wrappers too.)

- [ ] **Step 2: Registration (both tables) + prelude foreign decls**

In `register_audio`: change `polyosc_(_)` to `polyosc_(_,_)` and add:
```rust
    method("main", "Node", true, "polyadd_(_,_)", node_polyadd_impl::<S>);
    method("main", "Node", true, "polynoise_()", node_polynoise_impl::<S>);
```
In `METHODS`: change the `polyosc_(_)` line to `polyosc_(_,_)` and add:
```rust
    static_method("Node", "polyadd_(_,_)", bindings_audio::node_polyadd),
    static_method("Node", "polynoise_()", bindings_audio::node_polynoise),
```
In the prelude `Node` class: change `foreign static polyosc_(pitch)` to
`foreign static polyosc_(pitch, shape)` and add `foreign static polyadd_(a, b)`
and `foreign static polynoise_()`.

- [ ] **Step 3: Prelude route flips**

`class Osc` — route all four shapes (sine now passes the shape code 0):
```wren
  static sine(f) {
    if (Node.polyMode_ == 1) return Node.polyosc_(f, 0)
    return Node.src_(0, f)
  }
  static saw(f) {
    if (Node.polyMode_ == 1) return Node.polyosc_(f, 1)
    return Node.src_(1, f)
  }
  static square(f) {
    if (Node.polyMode_ == 1) return Node.polyosc_(f, 2)
    return Node.src_(2, f)
  }
  static tri(f) {
    if (Node.polyMode_ == 1) return Node.polyosc_(f, 3)
    return Node.src_(3, f)
  }
```

`+(o)` on **both** `class Node` and `foreign class Port` — route poly add (guard scalar like `*`):
```wren
  +(o) {
    if (Node.polyMode_ == 1) {
      if (o is Num) Fiber.abort("`+` a constant inside a Synth isn't supported yet")
      return Node.polyadd_(this, o)
    }
    return Node.binop_(1, this, o)
  }
```

`class Noise` — `new` routes poly; `pink`/`brown` keep aborting:
```wren
  static new() {
    if (Node.polyMode_ == 1) return Node.polynoise_()
    return Node.noise_()
  }
```
(Leave `Noise.pink()`/`.brown()` — add a `if (Node.polyMode_ == 1) Fiber.abort("poly pink/brown noise not supported yet (Sy-2c)")` guard as their first line if not already present.)

- [ ] **Step 4: Tests (audio_bindings.rs)**

Update `synth_error_cases_abort`: REMOVE the `Osc.saw`, `+`, and `Noise` abort
assertions (they work now); KEEP the still-deferred ones (nested Synth, two
Env.ar, no Env.ar, `Osc.sine * 0.5` scalar, and add `Noise.pink()` inside a
Synth aborts). Add:

```rust
#[test]
fn synth_saw_and_add_and_noise_build() {
    // Osc.saw → PolyOsc with SetParam(0,1); `+` → PolyAdd; Noise.new → PolyNoise.
    let cmds = run_and_capture_cmds(
        "var s = Synth.new { |p| (Osc.saw(p) + Noise.new()).lpf(1200) * Env.ar(0.01,0.3) }",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyOsc, .. })), "PolyOsc");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if (*value - 1.0).abs() < 1e-6)), "saw shape");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyAdd, .. })), "PolyAdd");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyNoise, .. })), "PolyNoise");
}

#[test]
fn synth_saw_renders_sound() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| Osc.saw(p).lpf(2000) * Env.ar(0.001,0.05) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "bounded");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3), "saw voice sounds");
}
```

- [ ] **Step 5: Run both configs + commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core synth_saw synth_error` (scalar).
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features deluge-dsp-kernels/simd synth_saw synth_error` — both PASS (the render exercises the `f32x8` saw). No warnings.

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): Synth Osc.saw/square/tri + `+` mixing + Noise (poly routes)"
```

---

## Notes for the implementer

- `wave_sample_x8` computes BOTH quartic/cubic branches per lane and `select`s — no div-by-zero panic (nan in an unused lane is discarded by `select`); it matches the scalar `if/else if` by giving the left/first branch priority.
- The `polyosc_` arity change (1→2) is why `Osc.sine` must now pass `0` — update it in the same edit, or existing Synth-sine patches break.
- `PolyNoise` seeds must be nonzero (xorshift fixed point at 0); the `if s == 0` guard covers the (astronomically unlikely) collision.
- If an anchor moved, grep the named symbol (`poly_in_count`, `Kind::PolyMul`, the `polyosc_` registration, `class Osc`) — the relationship matters, not line numbers.
