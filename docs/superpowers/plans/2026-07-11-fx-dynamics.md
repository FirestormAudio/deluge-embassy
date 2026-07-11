# Fx-1: Dynamics — Compressor / Limiter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A feed-forward peak/RMS compressor with soft knee, exposed as `Comp.new(...)` (full) and `Comp.limit(...)` (limiter preset), mono-in/mono-out on the existing `Drive` effect template.

**Architecture:** New `Comp` kernel (detector → soft-knee gain computer → attack/release ballistics → makeup) in `deluge-dsp-kernels`; a `Kind::Comp` width-1 graph node with 7 scalar `set_param`s; a Wren `class Comp` with two factories + live setters. Purely additive.

**Tech Stack:** Rust `no_std` (3 crates: kernels → graph → wren-core), `libm` (already a kernels dep) for dB↔linear, Wren scripting.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP paths. `libm` already a `deluge-dsp-kernels` dep.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** purely additive (new kernel, new `Kind`, new Wren class). No existing effect/node/test changes; signals that never use `Comp` render byte-identically.
- **Two registration tables:** `node_comp_impl` (+ setters) register in BOTH `install_methods` (`bindings_audio.rs`) AND the static `METHODS` table (`bindings.rs`), plus a prelude `class Comp`. Missing either = unbound method at runtime.
- **Test invocation (per-crate, never `--workspace`; both configs; `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

---

### Task 1: `Comp` kernel

**Files:**
- Create: `crates/deluge-dsp-kernels/src/dynamics.rs`
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (add `pub mod dynamics;` — alphabetical, AFTER `pub mod drive;` at line 11, BEFORE `pub mod env;` at line 12)
- Test: `dynamics.rs` `#[cfg(test)]` module

**Interfaces:**
- Consumes: `crate::In` (the per-sample input accessor with `.at(i)` — import it exactly as `drive.rs` does at the top of that file), `libm`.
- Produces: `pub enum Detector { Peak, Rms }`; `pub struct Comp` with `pub fn new(threshold_db, ratio, attack_s, release_s, knee_db, makeup_db, detector) -> Comp`, per-param setters, and `pub fn process(&mut self, input: In, dt: f32, out: &mut [f32])`. Used by Task 2.

- [ ] **Step 1: Write the failing oracle tests**

In a `#[cfg(test)] mod tests` in `dynamics.rs`. A helper to run a constant input for N samples and read the settled output gain:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    // dB helpers for tests.
    fn db(x: f32) -> f32 { 20.0 * libm::log10f(x.abs().max(1e-9)) }
    fn lin(db: f32) -> f32 { libm::powf(10.0, db / 20.0) }

    // `In::A(&slice)` is the array input accessor (see drive.rs node/kernel
    // tests: `In::A(&input)`). A constant input is a slice of the constant.
    fn run_const(c: &mut Comp, level_lin: f32, samples: usize, dt: f32) -> f32 {
        let arr = [level_lin; 64];
        let mut out = [0.0f32; 64];
        let mut last = 0.0;
        for _ in 0..(samples / 64) {
            c.process(In::A(&arr), dt, &mut out);
            last = out[out.len() - 1];
        }
        last / level_lin // = applied gain (linear)
    }

    const DT: f32 = 1.0 / 48_000.0;

    #[test]
    fn comp_below_threshold_is_unity() {
        // threshold -20 dB, ratio 4, no makeup, no knee; input -40 dB (below).
        let mut c = Comp::new(-20.0, 4.0, 0.001, 0.05, 0.0, 0.0, Detector::Peak);
        let g = run_const(&mut c, lin(-40.0), 4096, DT);
        assert!((db(g)).abs() < 0.2, "below threshold ≈ unity gain, got {} dB", db(g));
    }

    #[test]
    fn comp_static_curve_steady_state() {
        // thr -20, ratio 4:1, input -8 dB → GR = (in-thr)(1-1/ratio) = 12*0.75 = 9 dB.
        let mut c = Comp::new(-20.0, 4.0, 0.001, 0.05, 0.0, 0.0, Detector::Peak);
        let g = run_const(&mut c, lin(-8.0), 8192, DT);
        // applied gain in dB should be ≈ -9 dB (attenuation).
        assert!((db(g) - (-9.0)).abs() < 0.5, "static curve GR ≈ 9 dB, got {} dB", -db(g));
    }

    #[test]
    fn comp_limiter_clamps_near_threshold() {
        // ratio 20, fast attack, peak; input 0 dB, threshold -10 → output ≈ -10 dB.
        let mut c = Comp::new(-10.0, 20.0, 0.001, 0.05, 0.0, 0.0, Detector::Peak);
        let g = run_const(&mut c, lin(0.0), 8192, DT);
        let out_db = db(lin(0.0) * g); // input 0 dB + gain
        assert!(out_db < -8.5 && out_db > -11.5, "limiter clamps near -10 dB, got {}", out_db);
    }

    #[test]
    fn comp_soft_knee_partial_at_threshold() {
        // input exactly at threshold. hard knee → ~0 GR; soft knee → partial GR.
        let mut hard = Comp::new(-20.0, 4.0, 0.001, 0.05, 0.0, 0.0, Detector::Peak);
        let gh = run_const(&mut hard, lin(-20.0), 8192, DT);
        let mut soft = Comp::new(-20.0, 4.0, 0.001, 0.05, 12.0, 0.0, Detector::Peak);
        let gs = run_const(&mut soft, lin(-20.0), 8192, DT);
        assert!(db(gh).abs() < 0.3, "hard knee at threshold ≈ 0 GR, got {} dB", -db(gh));
        assert!(-db(gs) > 0.3, "soft knee at threshold has partial GR, got {} dB", -db(gs));
    }

    #[test]
    fn comp_ballistics_move_and_settle() {
        // After a step up in input, GR increases (gain drops); after step down, recovers.
        let mut c = Comp::new(-20.0, 8.0, 0.01, 0.05, 0.0, 0.0, Detector::Peak);
        let quiet = run_const(&mut c, lin(-40.0), 2048, DT); // settle quiet → ~unity
        let loud = run_const(&mut c, lin(0.0), 4096, DT);    // settle loud → strong GR
        let recovered = run_const(&mut c, lin(-40.0), 8192, DT); // release back
        assert!(db(loud) < db(quiet) - 1.0, "loud input reduces gain vs quiet");
        assert!(db(recovered) > db(loud) + 1.0, "release recovers gain after loud");
    }
}
```

> First confirm the exact way kernel tests build a constant `In` (grep `drive.rs`/`filter.rs` test modules for how they construct the `In` passed to `process`). Use that idiom for `run_const`'s `inp`. If `In` is an enum with a `Const` variant, `In::Const(level_lin)`; otherwise copy the sibling test helper.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels comp_`
Expected: FAIL (`Comp`/`Detector` not defined).

- [ ] **Step 3: Implement the kernel**

`crates/deluge-dsp-kernels/src/dynamics.rs`:

```rust
//! Feed-forward dynamics (Fx-1): a peak/RMS compressor with a soft knee,
//! attack/release ballistics, and makeup gain. Mono, scalar, `no_std`.

use crate::In; // same import path drive.rs uses for the per-sample input accessor

const LEVEL_EPS: f32 = 1e-9; // floor for log10 (avoid -inf on silence)
const RMS_WINDOW_S: f32 = 0.01; // 10 ms RMS averaging window

#[derive(Clone, Copy, PartialEq)]
pub enum Detector {
    Peak,
    Rms,
}

pub struct Comp {
    threshold_db: f32,
    ratio: f32,
    attack_s: f32,
    release_s: f32,
    knee_db: f32,
    makeup_db: f32,
    detector: Detector,
    rms_sq: f32, // smoothed x^2 (RMS state)
    gr_db: f32,  // current smoothed gain reduction, dB, >= 0
}

#[inline]
fn lin_to_db(x: f32) -> f32 {
    20.0 * libm::log10f(x.max(LEVEL_EPS))
}

#[inline]
fn db_to_lin(db: f32) -> f32 {
    libm::powf(10.0, db / 20.0)
}

#[inline]
fn one_pole_coeff(time_s: f32, dt: f32) -> f32 {
    // 1 - e^(-dt/tau); tau floored at dt so a ~0 time is ~1 (near-instant).
    1.0 - libm::expf(-dt / time_s.max(dt))
}

impl Comp {
    pub fn new(
        threshold_db: f32,
        ratio: f32,
        attack_s: f32,
        release_s: f32,
        knee_db: f32,
        makeup_db: f32,
        detector: Detector,
    ) -> Comp {
        Comp {
            threshold_db,
            ratio: ratio.max(1.0),
            attack_s: attack_s.max(0.0),
            release_s: release_s.max(0.0),
            knee_db: knee_db.max(0.0),
            makeup_db,
            detector,
            rms_sq: 0.0,
            gr_db: 0.0,
        }
    }

    pub fn set_threshold(&mut self, db: f32) { self.threshold_db = db; }
    pub fn set_ratio(&mut self, r: f32) { self.ratio = r.max(1.0); }
    pub fn set_attack(&mut self, s: f32) { self.attack_s = s.max(0.0); }
    pub fn set_release(&mut self, s: f32) { self.release_s = s.max(0.0); }
    pub fn set_knee(&mut self, db: f32) { self.knee_db = db.max(0.0); }
    pub fn set_makeup(&mut self, db: f32) { self.makeup_db = db; }
    pub fn set_detector(&mut self, d: Detector) { self.detector = d; }

    /// Static gain-reduction curve (dB, >= 0) for a given over-threshold amount.
    #[inline]
    fn gain_reduction(&self, over_db: f32) -> f32 {
        let slope = 1.0 - 1.0 / self.ratio; // 0 at 1:1, →1 at ∞:1
        let half = self.knee_db * 0.5;
        if self.knee_db > 0.0 && over_db > -half && over_db < half {
            // soft knee: quadratic, C0-continuous with the two branches.
            let t = over_db + half;
            slope * t * t / (2.0 * self.knee_db)
        } else if over_db <= 0.0 {
            0.0
        } else {
            slope * over_db
        }
    }

    pub fn process(&mut self, input: In, dt: f32, out: &mut [f32]) {
        let rms_c = one_pole_coeff(RMS_WINDOW_S, dt);
        let atk_c = one_pole_coeff(self.attack_s, dt);
        let rel_c = one_pole_coeff(self.release_s, dt);
        let makeup_lin = db_to_lin(self.makeup_db);
        for i in 0..out.len() {
            let x = input.at(i);
            // 1. detector level (dB)
            let level = match self.detector {
                Detector::Peak => libm::fabsf(x),
                Detector::Rms => {
                    self.rms_sq += (x * x - self.rms_sq) * rms_c;
                    libm::sqrtf(self.rms_sq)
                }
            };
            let level_db = lin_to_db(level);
            // 2. target gain reduction (dB, >= 0)
            let target_gr = self.gain_reduction(level_db - self.threshold_db);
            // 3. ballistics: attack when GR rising, release when falling
            let c = if target_gr > self.gr_db { atk_c } else { rel_c };
            self.gr_db += (target_gr - self.gr_db) * c;
            // 4. apply
            out[i] = x * db_to_lin(-self.gr_db) * makeup_lin;
        }
    }
}
```

Add `pub mod dynamics;` to `crates/deluge-dsp-kernels/src/lib.rs` between `drive` (line 11) and `env` (line 12).

- [ ] **Step 4: Run to verify the tests pass (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels comp_` (then with `--features simd`).
Then FULL kernels crate both configs (nothing else changed). Expected: PASS.

> If `comp_static_curve_steady_state` or `comp_limiter_clamps_near_threshold` misses the tolerance, the settle length may be too short for the release/attack chosen — increase `samples`, do NOT loosen the physics. The steady-state gain MUST match the static curve within the stated tolerance.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/dynamics.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(kernels): Comp — feed-forward peak/RMS compressor with soft knee + ballistics"
```

---

### Task 2: `Kind::Comp` graph node

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs` (import ~line 10; `Kind` enum ~72; `State` enum ~133; constructor ~212; `set_param` ~388; process arm ~734)
- Test: `node.rs` test module

**Interfaces:**
- Consumes: `Comp`/`Detector` (Task 1); `In`; the width-1 processor arm idiom (`Kind::Drive` at node.rs:734).
- Produces: `Kind::Comp` (out_width 1, one signal input), `State::Comp(Comp)`, `set_param(0..=6)`. Used by Task 3.

- [ ] **Step 1: Write the failing test**

Mirror the `Kind::Drive` node test (node.rs:1400). Drive a `Kind::Comp` node with a loud constant input over enough samples and assert the output is attenuated + `out_width`/`set_param` wiring:

```rust
#[test]
fn comp_node_wires_params_and_compresses() {
    assert_eq!(Node::out_width(Kind::Comp), 1);
    let mut n = Node::new(Kind::Comp, 0);
    n.set_param(0, -20.0); // threshold
    n.set_param(1, 8.0);   // ratio
    n.set_param(2, 0.001); // attack
    n.set_param(3, 0.05);  // release
    n.set_param(4, 0.0);   // knee (hard)
    n.set_param(5, 0.0);   // makeup
    n.set_param(6, 0.0);   // Peak
    // Loud constant 1.0 (0 dB, well above -20) on port 0; run several blocks to
    // settle the ballistics, then assert the output is well below the input.
    // Scaffolding copied from `drive_node_renders_mono_bounded` (node.rs:1399).
    let input = [1.0f32; 64];
    let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
    let mut buf = [0.0f32; 64];
    for _ in 0..20 {
        let mut outs = OutView::single(&mut buf);
        n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
    }
    assert!(buf.iter().all(|s| s.is_finite()), "finite");
    // 0 dB through thr=-20/ratio=8 → ~15 dB GR → settled magnitude ≈ 0.18, well < 1.0.
    assert!(buf[buf.len() - 1].abs() < 0.5, "compressed well below input, got {}", buf[buf.len() - 1]);
}
```

> The scaffolding (`In::A`, `OutView::single`, `process_resolved(&ins, dt, &mut outs, None)`) is copied verbatim from `drive_node_renders_mono_bounded` (node.rs:1399). `In`/`OutView` are already imported in the node.rs test module.

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph comp_node`
Expected: FAIL (`Kind::Comp` not found).

- [ ] **Step 3: Wire `Kind::Comp`**

In `crates/deluge-audio-graph/src/node.rs`:

1. **Import** (line 10, beside `drive::{Drive, Shape}`): add `dynamics::{Comp, Detector},`.
2. **Kind enum** (after `Drive,` at line 72): `Comp,`.
3. **State enum** (after `Drive(Drive),` at line 133): `Comp(Comp),`.
4. **Constructor** (after `Kind::Drive => State::Drive(Drive::new(Shape::Soft)),` at line 212):
```rust
    Kind::Comp => State::Comp(Comp::new(-20.0, 4.0, 0.01, 0.1, 6.0, 0.0, Detector::Rms)),
```
(Default = a gentle general compressor; the Wren factory overrides via set_param.)
5. **`out_width`:** `Kind::Comp` is mono → it falls into the default `_ => 1` arm. Do NOT add it to the width-2 list. (The test asserts `out_width(Kind::Comp) == 1`.)
6. **`set_param`** (after the `State::Drive(d) => match param { … }` arm at line 388):
```rust
    State::Comp(c) => match param {
        0 => c.set_threshold(value),
        1 => c.set_ratio(value),
        2 => c.set_attack(value),
        3 => c.set_release(value),
        4 => c.set_knee(value),
        5 => c.set_makeup(value),
        6 => c.set_detector(if value == 0.0 { Detector::Peak } else { Detector::Rms }),
        _ => {}
    },
```
7. **Process arm** (after the `Kind::Drive` arm at node.rs:734):
```rust
    Kind::Comp => {
        if let State::Comp(c) = &mut self.state {
            c.process(ins[0]…, dt, out);
        }
    }
```
Match the EXACT `ins[0]`/`dt`/`out` argument shape the `Kind::Drive` arm uses (Drive ignores dt; `Comp` USES `dt` — pass the real `dt` the arm receives). Copy Drive's arm structure and swap the kernel call.

- [ ] **Step 4: Run to verify pass + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph comp_node` (+ `--features deluge-dsp-kernels/simd`).
Then FULL graph crate both configs (existing nodes untouched). Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(graph): Kind::Comp — compressor node (out_width 1, set_param 0-6)"
```

---

### Task 3: Wren `Comp` factories + setters

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (factory + setters + `install_methods` registration, near `node_drive_impl` ~806 / registration ~1923)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (static `METHODS` table, near the Drive entries ~884/900)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (Node `foreign` decls near ~123/139; new `class Comp` near `class Drive` ~711)

**Interfaces:**
- Consumes: `Kind::Comp` + its `set_param(0..6)` (Task 2); the `node_drive_impl` factory template (bindings_audio.rs:806) and the `node_set_drive`/`node_set_tone` setter template.
- Produces: Wren `Comp.new(input, threshold, ratio, attack, release)`, `Comp.limit(input, threshold)`, and instance setters `threshold=`/`ratio=`/`attack=`/`release=`/`knee=`/`makeup=`/`detector=`.

- [ ] **Step 1: The factory foreign**

In `bindings_audio.rs`, model on `node_drive_impl` (line 806). Add `node_comp_impl` taking 8 args (input + threshold, ratio, attack, release, knee, makeup, detector):
```rust
/// `Kind::Comp` compressor node (no buffer); sets threshold/ratio/attack/
/// release/knee/makeup/detector params. (Modeled on `node_drive_impl`, :806.)
pub(crate) fn node_comp_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let threshold = vm.get_f(2) as f32;
    let ratio = vm.get_f(3) as f32;
    let attack = vm.get_f(4) as f32;
    let release = vm.get_f(5) as f32;
    let knee = vm.get_f(6) as f32;
    let makeup = vm.get_f(7) as f32;
    let detector = vm.get_f(8) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Comp, [input, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, threshold);
    audio::set_param(id, 1, ratio);
    audio::set_param(id, 2, attack);
    audio::set_param(id, 3, release);
    audio::set_param(id, 4, knee);
    audio::set_param(id, 5, makeup);
    audio::set_param(id, 6, detector);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_comp(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_comp_impl(&vm);
}
```
(Byte-for-byte the `node_drive_impl` idiom — `arg_input(vm, 1)`, `audio::alloc_node_id()`, `new_node(id, Kind, [input, Const, Const])`, `set_param`, `return_node`, and the `Vm(raw)` shim.)

- [ ] **Step 2: The setter foreigns**

Model on `node_set_tone_impl` (bindings_audio.rs:826) / `node_set_drive_impl` (:521) — each setter is:
```rust
pub(crate) fn comp_set_threshold_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32;
    audio::set_param(self_id(vm), 0, v); // param index 0 = threshold
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn comp_set_threshold(raw: *mut WrenVM) {
    let vm = Vm(raw);
    comp_set_threshold_impl(&vm);
}
```
Repeat for `comp_set_ratio`(index 1), `comp_set_attack`(2), `comp_set_release`(3), `comp_set_knee`(4), `comp_set_makeup`(5), `comp_set_detector`(6) — same body, different index. `detector=` passes the number straight to `set_param(6, v)` (the node arm maps 0→Peak / else Rms). `self_id(vm)` is the same accessor `node_set_tone_impl` uses.

- [ ] **Step 3: Register in BOTH tables**

`install_methods` (bindings_audio.rs ~1923, beside the `drive_` line):
```rust
method("main", "Node", true,  "comp_(_,_,_,_,_,_,_,_)", node_comp_impl::<S>);
method("main", "Node", false, "compThreshold=(_)", comp_set_threshold_impl::<S>);
method("main", "Node", false, "compRatio=(_)",     comp_set_ratio_impl::<S>);
method("main", "Node", false, "compAttack=(_)",    comp_set_attack_impl::<S>);
method("main", "Node", false, "compRelease=(_)",   comp_set_release_impl::<S>);
method("main", "Node", false, "compKnee=(_)",      comp_set_knee_impl::<S>);
method("main", "Node", false, "compMakeup=(_)",    comp_set_makeup_impl::<S>);
method("main", "Node", false, "compDetector=(_)",  comp_set_detector_impl::<S>);
```
`METHODS` table (bindings.rs, beside the Drive entries ~884/900):
```rust
static_method("Node", "comp_(_,_,_,_,_,_,_,_)", bindings_audio::node_comp),
method("Node", "compThreshold=(_)", bindings_audio::comp_set_threshold),
method("Node", "compRatio=(_)",     bindings_audio::comp_set_ratio),
method("Node", "compAttack=(_)",    bindings_audio::comp_set_attack),
method("Node", "compRelease=(_)",   bindings_audio::comp_set_release),
method("Node", "compKnee=(_)",      bindings_audio::comp_set_knee),
method("Node", "compMakeup=(_)",    bindings_audio::comp_set_makeup),
method("Node", "compDetector=(_)",  bindings_audio::comp_set_detector),
```
(Match the exact selector-name convention the file uses — the names above must be identical in both tables and the prelude `foreign` decls. Follow whatever casing/style `drive=`/`drive_` use.)

- [ ] **Step 4: Prelude `foreign` decls + `class Comp`**

In `wren/prelude.wren`, add to the `Node` class (near the drive foreigns ~123/139):
```
  foreign static comp_(input, threshold, ratio, attack, release, knee, makeup, detector)
  foreign compThreshold=(v)
  foreign compRatio=(v)
  foreign compAttack=(v)
  foreign compRelease=(v)
  foreign compKnee=(v)
  foreign compMakeup=(v)
  foreign compDetector=(v)
```
Add a `class Comp` near `class Drive` (line 711):
```
class Comp {
  // Full compressor: RMS detector, 6 dB soft knee, no makeup, by default.
  static new(input, threshold, ratio, attack, release) {
    return Node.comp_(input, threshold, ratio, attack, release, 6, 0, 1)
  }
  // Limiter preset: high ratio, fast attack, peak detector, hard knee.
  static limit(input, threshold) {
    return Node.comp_(input, threshold, 20, 0.001, 0.1, 0, 0, 0)
  }
}
```
(The trailing `1`/`0` in the factory calls are the detector: `1` = RMS for `new`, `0` = Peak for `limit`. If the file exposes enum-ish params via named statics (like `Drive.soft`), you MAY add `Comp.peak`/`Comp.rms` returning 0/1 — but the numeric literals are sufficient and match the `Drive` shape-int convention.)

The instance setters (`comp.threshold = x`, etc.) map to the `foreign compThreshold=` decls — a user calls `comp.compThreshold = -15` OR, if you add sugar setter names on a returned object, wire them; the minimal correct surface is the `foreign compX=` decls on `Node` (a `Comp.new(...)` returns a `Node`, so `node.compThreshold = …` works). Keep it consistent with how `Drive` exposes `drive=`/`tone=` on its returned node.

- [ ] **Step 5: Compile + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` (+ `--features deluge-dsp-kernels/simd`).
Gate: compiles clean + full existing suite green BOTH configs. If "metaclass does not implement" / "Node does not implement comp_" appears → a table registration is missing or a selector name mismatches between the tables and prelude. Nothing exercises `Comp` yet (Task 4). Expected: PASS both.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren
git commit -m "feat(wren): Comp.new / Comp.limit — compressor factories + setters (both tables + prelude)"
```

---

### Task 4: End-to-end

**Files:**
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Comp.new`/`Comp.limit` (Task 3); `run_and_render(src, &mut [StereoFrame; N])`; `run_script_ok`.

- [ ] **Step 1: Write the failing e2e tests**

Mirror the existing effect render tests (e.g. `svf_lp_renders_finite_nonsilent`). NEWLINE-separated statements.

```rust
#[test]
fn comp_renders_finite_nonsilent() {
    let mut out = [StereoFrame::default(); 64];
    run_and_render("Out.patch(Comp.new(Osc.saw(110), -20, 4, 0.005, 0.1))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "bounded/finite");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3), "compressed signal sounds");
}

#[test]
fn comp_limiter_reduces_peak_of_loud_source() {
    // A loud saw: dry peak vs Comp.limit peak. Limiter must lower the peak.
    let peak = |buf: &[StereoFrame]| buf.iter().map(|f| f.l.abs()).fold(0.0f32, f32::max);
    let mut dry = [StereoFrame::default(); 128];
    run_and_render("Out.patch(Osc.saw(110))", &mut dry);
    let mut lim = [StereoFrame::default(); 128];
    run_and_render("Out.patch(Comp.limit(Osc.saw(110), -12))", &mut lim);
    assert!(lim.iter().all(|f| f.l.is_finite()), "limiter finite");
    assert!(peak(&lim) < peak(&dry), "limiter lowers the peak (dry {} vs lim {})", peak(&dry), peak(&lim));
    assert!(lim.iter().any(|f| f.l.abs() > 1e-3), "limited signal still sounds");
}

#[test]
fn comp_unity_ratio_is_transparent_ish() {
    // ratio 1:1 → no compression; a quiet source passes essentially unchanged in level.
    assert!(run_script_ok("Out.patch(Comp.new(Osc.sine(220), -20, 1, 0.005, 0.1))"), "ratio 1 builds/runs");
}
```

> If `Osc.saw(110)` alone doesn't drive a loud enough signal to exceed −12 dB for the limiter test, scale it up within the script by whatever gain idiom the file uses, or lower the threshold — the discriminating property is `peak(lim) < peak(dry)`, which requires the source to actually exceed threshold. Verify the dry peak first; adjust threshold so the limiter genuinely engages (do NOT weaken to `<=`).

- [ ] **Step 2: Run (Comp already implemented in Task 3)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- comp_renders comp_limiter comp_unity` (+ `--features deluge-dsp-kernels/simd`).
If `comp_limiter_reduces_peak_of_loud_source` fails to show a peak reduction, the limiter isn't engaging (threshold too high vs source level) OR a real Task 1–3 bug — investigate; do NOT weaken the assertion. Expected: PASS both.

- [ ] **Step 3: Full regression**

Run the FULL wren-core suite both configs. Existing synths/effects byte-unchanged. Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "test(wren): compressor e2e — renders, limiter lowers peak, unity ratio transparent"
```

---

## Self-Review Notes (for the executor)

- **The kernel is the crux.** The static-curve steady-state test (`GR = (in−thr)(1−1/ratio)`) is the correctness oracle — if it fails, the gain computer or ballistics is wrong, NOT the test. The soft-knee test proves the knee softens the corner (partial GR at threshold). Do not loosen these.
- **`Comp` uses `dt`** (attack/release/RMS time constants) — unlike `Drive` which ignores it. The node process arm MUST pass the real `dt`.
- **`In` accessor:** the kernel `process` takes `In` (per-sample `.at(i)`), matching `Drive`/`Eq` — not `&[f32]`. Confirm the constant-`In` construction idiom from a sibling kernel test before writing `run_const`.
- **Both registration tables + prelude selector names must match exactly** (`comp_`, `compThreshold=`, … identical in `install_methods`, `METHODS`, and the prelude `foreign` decls) — the recurring "metaclass does not implement" gotcha.
- **Purely additive / non-breaking:** no existing kernel, node, effect, or test changes. Every step only ADDS. If an existing test needs editing, something is wrong.
- **Deferred (do NOT implement):** gate/expander, lookahead, stereo-linked/stereo-in detection, sidechain, GR metering, RMS-window as a param, and the rest of the effects backlog (bitcrush, phaser, tremolo, mid-side, stereo-in effects).
