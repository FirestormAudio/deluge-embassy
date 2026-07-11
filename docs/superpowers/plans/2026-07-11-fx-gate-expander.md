# Fx-1b: Dynamics — Gate / Expander Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A downward expander / noise gate (attenuate below threshold, with a hold stage), exposed as `Expander.new(...)` and `Gate.new(...)`, mono-in/out on the Fx-1 effect template.

**Architecture:** A new `Gate` struct in `dynamics.rs` reusing Fx-1's `Detector`/`lin_to_db`/`db_to_lin`/`one_pole_coeff`, with a downward-expansion gain computer, a range floor, a hold state machine, and INVERTED attack/release semantics. A `Kind::Gate` width-1 node; two Wren classes over one factory. Purely additive.

**Tech Stack:** Rust `no_std` (kernels → graph → wren-core), `libm`, Wren.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP paths. `libm` for transcendentals (NO std float methods — cross-compiled `no_std` for device). Bounded loops.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** purely additive — a new `Gate` struct/`Kind`/Wren classes/tests. `Comp` and all existing code untouched. Everything that doesn't use `Gate` is byte-unchanged.
- **Two registration tables:** `node_gate_impl` + 7 setters register in BOTH `install_methods` (bindings_audio.rs) AND `METHODS` (bindings.rs), identical selectors, + prelude `foreign` decls. Both `Expander` and `Gate` classes carry the `polyMode_` effect-guard.
- **Test invocation (per-crate, never `--workspace`; both configs; `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

---

### Task 1: `Gate` kernel

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/dynamics.rs` (ADD `Gate` after `Comp`; reuse the file's `Detector`, `LEVEL_EPS`, `RMS_WINDOW_S`, `lin_to_db`, `db_to_lin`, `one_pole_coeff`, and the test module's `run_const`/`db`/`lin` helpers)
- Test: the existing `dynamics.rs` `#[cfg(test)]` module

**Interfaces:**
- Consumes: `In`, `Detector`, `lin_to_db`/`db_to_lin`/`one_pole_coeff`, `RMS_WINDOW_S` (all already in `dynamics.rs`), `libm`.
- Produces: `pub struct Gate` with `pub fn new(threshold_db, ratio, attack_s, release_s, hold_s, range_db, detector) -> Gate`, per-param setters, and `pub fn process(&mut self, input: In, dt: f32, out: &mut [f32])`. Used by Task 2.

- [ ] **Step 1: Write the failing oracle tests**

Add to the existing `dynamics.rs` test module (reuse its `run_const`/`db`/`lin` — but `run_const` takes `&mut Comp`; add a parallel `run_const_gate` mirroring it, and a two-phase driver for the hold test):

```rust
// Mirror `run_const` for Gate (settle a constant input, return applied gain).
fn run_const_gate(g: &mut Gate, level_lin: f32, samples: usize, dt: f32) -> f32 {
    let arr = [level_lin; 64];
    let mut out = [0.0f32; 64];
    let mut last = 0.0;
    for _ in 0..(samples / 64) {
        g.process(In::A(&arr), dt, &mut out);
        last = out[out.len() - 1];
    }
    last / level_lin
}

const DT_G: f32 = 1.0 / 48_000.0;

#[test]
fn gate_above_threshold_is_unity() {
    // thr -20, input -6 dB (above) → open, ~unity.
    let mut g = Gate::new(-20.0, 2.0, 0.001, 0.05, 0.0, 40.0, Detector::Peak);
    let gain = run_const_gate(&mut g, lin(-6.0), 4096, DT_G);
    assert!(db(gain).abs() < 0.2, "above threshold ≈ unity, got {} dB", db(gain));
}

#[test]
fn gate_expansion_static_curve() {
    // thr -40, ratio 2:1, input -50 dB, range 40, hold 0.
    // GR = min((thr-level)(ratio-1), range) = min(10*1, 40) = 10 dB.
    let mut g = Gate::new(-40.0, 2.0, 0.001, 0.05, 0.0, 40.0, Detector::Peak);
    let gain = run_const_gate(&mut g, lin(-50.0), 8192, DT_G);
    assert!((db(gain) - (-10.0)).abs() < 0.5, "expansion GR ≈ 10 dB, got {} dB", -db(gain));
}

#[test]
fn gate_range_floor_caps_attenuation() {
    // thr -40, ratio 4:1, input -70 dB → raw GR = 30*3 = 90 dB, capped at range 20.
    let mut g = Gate::new(-40.0, 4.0, 0.001, 0.05, 0.0, 20.0, Detector::Peak);
    let gain = run_const_gate(&mut g, lin(-70.0), 8192, DT_G);
    assert!((db(gain) - (-20.0)).abs() < 0.6, "GR capped at range 20 dB, got {} dB", -db(gain));
}

#[test]
fn gate_hold_keeps_open_then_closes() {
    // Open on a loud input, then drop below threshold; within the hold window the
    // gain stays ~1 (open), after the hold it drops (closing).
    let mut g = Gate::new(-20.0, 8.0, 0.0005, 0.02, 0.002, 60.0, Detector::Peak); // hold 2 ms
    // settle open with a loud (above-threshold) constant
    run_const_gate(&mut g, lin(0.0), 4096, DT_G);
    // now one big block of below-threshold input; capture per-sample gain
    let quiet = lin(-60.0);
    let arr = [quiet; 512];
    let mut out = [0.0f32; 512];
    g.process(In::A(&arr), DT_G, &mut out);
    let hold_samples = (0.002 / DT_G) as usize; // ≈ 96
    let gain_at = |k: usize| out[k] / quiet;
    // early (well within hold) → still open (~unity)
    assert!(gain_at(hold_samples / 4).abs() > 0.9, "gain held open early, got {}", gain_at(hold_samples / 4));
    // late (well past hold) → closing/closed (attenuated)
    assert!(gain_at(500).abs() < 0.5, "gate closed after hold, got {}", gain_at(500));
}

#[test]
fn gate_ballistics_open_and_close() {
    // Open (attack) on loud, close (release) on quiet-past-hold.
    let mut g = Gate::new(-20.0, 8.0, 0.0005, 0.02, 0.0, 60.0, Detector::Peak);
    let closed = run_const_gate(&mut g, lin(-60.0), 4096, DT_G); // settle closed → strong GR
    let opened = run_const_gate(&mut g, lin(0.0), 4096, DT_G);   // loud → opens → ~unity
    assert!(db(opened) > db(closed) + 1.0, "loud input opens the gate vs quiet");
    assert!(db(opened).abs() < 0.5, "opened ≈ unity");
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels gate_`
Expected: FAIL (`Gate` not defined).

- [ ] **Step 3: Implement `Gate`**

Add to `dynamics.rs` after `Comp`'s `impl` block:

```rust
#[derive(Clone, Copy)]
pub struct Gate {
    threshold_db: f32,
    ratio: f32,
    attack_s: f32,
    release_s: f32,
    hold_s: f32,
    range_db: f32, // max attenuation depth (dB, >= 0)
    detector: Detector,
    rms_sq: f32,
    gr_db: f32,    // current gain reduction, dB, >= 0
    hold_ctr: i32, // samples remaining in the hold-open window
}

impl Gate {
    pub fn new(
        threshold_db: f32,
        ratio: f32,
        attack_s: f32,
        release_s: f32,
        hold_s: f32,
        range_db: f32,
        detector: Detector,
    ) -> Gate {
        Gate {
            threshold_db,
            ratio: ratio.max(1.0),
            attack_s: attack_s.max(0.0),
            release_s: release_s.max(0.0),
            hold_s: hold_s.max(0.0),
            range_db: range_db.max(0.0),
            detector,
            rms_sq: 0.0,
            gr_db: 0.0,
            hold_ctr: 0,
        }
    }

    pub fn set_threshold(&mut self, db: f32) { self.threshold_db = db; }
    pub fn set_ratio(&mut self, r: f32) { self.ratio = r.max(1.0); }
    pub fn set_attack(&mut self, s: f32) { self.attack_s = s.max(0.0); }
    pub fn set_release(&mut self, s: f32) { self.release_s = s.max(0.0); }
    pub fn set_hold(&mut self, s: f32) { self.hold_s = s.max(0.0); }
    pub fn set_range(&mut self, db: f32) { self.range_db = db.max(0.0); }
    pub fn set_detector(&mut self, d: Detector) { self.detector = d; }

    pub fn process(&mut self, input: In, dt: f32, out: &mut [f32]) {
        let atk_c = one_pole_coeff(self.attack_s, dt);
        let rel_c = one_pole_coeff(self.release_s, dt);
        let rms_c = one_pole_coeff(RMS_WINDOW_S, dt);
        let hold_samples = (self.hold_s / dt) as i32;
        for i in 0..out.len() {
            let x = input.at(i);
            let level = match self.detector {
                Detector::Peak => libm::fabsf(x),
                Detector::Rms => {
                    self.rms_sq += (x * x - self.rms_sq) * rms_c;
                    libm::sqrtf(self.rms_sq)
                }
            };
            let over = lin_to_db(level) - self.threshold_db;
            // gain computer + hold state machine
            let target_gr = if over >= 0.0 {
                self.hold_ctr = hold_samples; // above threshold: open, arm hold
                0.0
            } else if self.hold_ctr > 0 {
                self.hold_ctr -= 1; // holding open
                0.0
            } else {
                ((-over) * (self.ratio - 1.0)).min(self.range_db) // closing / expanding
            };
            // ballistics: attack when GR falling (opening), release when rising (closing)
            let c = if target_gr < self.gr_db { atk_c } else { rel_c };
            self.gr_db += (target_gr - self.gr_db) * c;
            out[i] = x * db_to_lin(-self.gr_db);
        }
    }
}
```

- [ ] **Step 4: Run to verify pass (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels gate_` (+ `--features simd`).
Then FULL kernels crate both configs (Comp + everything else untouched). Expected: PASS.

> If a static-curve test misses tolerance, increase the settle `samples` (ballistics need time) — do NOT loosen the physics. The hold test's `gain_at` offsets assume `hold_samples ≈ 96` at 48 kHz — if you change `DT_G`/hold, recompute the offsets, keep the assertion meaning (open within hold, closed after).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/dynamics.rs
git commit -m "feat(kernels): Gate — downward expander/gate with hold + range floor"
```

---

### Task 2: `Kind::Gate` graph node

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs` (import ~line 11; `Kind` enum; `State` enum ~136; constructor ~216; `set_param` ~399; process arm ~753 — all beside the `Comp` spots)
- Test: `node.rs` test module

**Interfaces:**
- Consumes: `Gate`/`Detector` (Task 1); the `Kind::Comp` wiring as the exact template.
- Produces: `Kind::Gate` (out_width 1, one signal input), `State::Gate(Gate)`, `set_param(0..=6)`. Used by Task 3.

- [ ] **Step 1: Write the failing test** (mirror `comp_node_wires_params_and_compresses`, node.rs:1437)

```rust
#[test]
fn gate_node_wires_params_and_gates() {
    assert_eq!(Node::out_width(Kind::Gate), 1);
    let mut n = Node::new(Kind::Gate, 0);
    n.set_param(0, 0.0);   // threshold 0 dB
    n.set_param(1, 4.0);   // ratio
    n.set_param(2, 0.001); // attack
    n.set_param(3, 0.01);  // release
    n.set_param(4, 0.0);   // hold
    n.set_param(5, 40.0);  // range
    n.set_param(6, 0.0);   // Peak
    // Constant 0.1 (-20 dB, below the 0 dB threshold) → gate closes → attenuated.
    let input = [0.1f32; 64];
    let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
    let mut buf = [0.0f32; 64];
    for _ in 0..20 {
        let mut outs = OutView::single(&mut buf);
        n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
    }
    assert!(buf.iter().all(|s| s.is_finite()), "finite");
    assert!(buf[buf.len() - 1].abs() < 0.05, "below-threshold input gated down, got {}", buf[buf.len() - 1]);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph gate_node`
Expected: FAIL (`Kind::Gate` not found).

- [ ] **Step 3: Wire `Kind::Gate`** (mirror the `Comp` wiring exactly)

1. **Import** (line 11, extend the dynamics import): `dynamics::{Comp, Detector, Gate},`.
2. **Kind enum** (after `Comp,`): `Gate,`.
3. **State enum** (after `Comp(Comp),` at line 136): `Gate(Gate),`.
4. **Constructor** (after the `Kind::Comp =>` arm at line 216):
```rust
    Kind::Gate => State::Gate(Gate::new(-40.0, 2.0, 0.001, 0.1, 0.0, 20.0, Detector::Peak)),
```
5. **`out_width`:** `Kind::Gate` falls into `_ => 1` (do NOT add to the width-2 list).
6. **`set_param`** (after the `State::Comp(c) => match param { … }` arm at line 399):
```rust
    State::Gate(g) => match param {
        0 => g.set_threshold(value),
        1 => g.set_ratio(value),
        2 => g.set_attack(value),
        3 => g.set_release(value),
        4 => g.set_hold(value),
        5 => g.set_range(value),
        6 => g.set_detector(if value == 0.0 { Detector::Peak } else { Detector::Rms }),
        _ => {}
    },
```
7. **Process arm** (after the `Kind::Comp` arm at line 753):
```rust
    Kind::Gate => {
        if let State::Gate(g) = &mut self.state {
            g.process(ins[0], dt, outs.port(0));
        }
    }
```

- [ ] **Step 4: Run to verify pass + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph gate_node` (+ `--features deluge-dsp-kernels/simd`).
Then FULL graph crate both configs (existing nodes untouched). Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(graph): Kind::Gate — gate/expander node (out_width 1, set_param 0-6)"
```

---

### Task 3: Wren `Gate` / `Expander` classes + setters

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (factory + 7 setters + install_methods, beside the `comp_` entries ~849/2024)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (METHODS table, beside the comp entries ~903)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (Node foreign decls ~142; new `class Expander` + `class Gate` near `class Comp`)

**Interfaces:**
- Consumes: `Kind::Gate` + `set_param(0..6)` (Task 2); the `node_comp_impl`/`comp_set_*` templates.
- Produces: Wren `Expander.new(input, threshold, ratio, attack, release)` and `Gate.new(input, threshold, attack, release, hold)`, + the `gateX=` setters.

- [ ] **Step 1: Factory** (mirror `node_comp_impl` at bindings_audio.rs:849 exactly)

```rust
pub(crate) fn node_gate_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let threshold = vm.get_f(2) as f32;
    let ratio = vm.get_f(3) as f32;
    let attack = vm.get_f(4) as f32;
    let release = vm.get_f(5) as f32;
    let hold = vm.get_f(6) as f32;
    let range = vm.get_f(7) as f32;
    let detector = vm.get_f(8) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Gate, [input, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, threshold);
    audio::set_param(id, 1, ratio);
    audio::set_param(id, 2, attack);
    audio::set_param(id, 3, release);
    audio::set_param(id, 4, hold);
    audio::set_param(id, 5, range);
    audio::set_param(id, 6, detector);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_gate(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_gate_impl(&vm);
}
```

- [ ] **Step 2: The 7 setters** (mirror `comp_set_threshold_impl` at bindings_audio.rs:875 — same body, different index)

`gate_set_threshold`(0), `gate_set_ratio`(1), `gate_set_attack`(2), `gate_set_release`(3), `gate_set_hold`(4), `gate_set_range`(5), `gate_set_detector`(6). Each:
```rust
pub(crate) fn gate_set_threshold_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32;
    audio::set_param(self_id(vm), 0, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn gate_set_threshold(raw: *mut WrenVM) {
    let vm = Vm(raw);
    gate_set_threshold_impl(&vm);
}
```

- [ ] **Step 3: Register in BOTH tables**

`install_methods` (bindings_audio.rs ~2024, beside `comp_`):
```rust
method("main", "Node", true,  "gate_(_,_,_,_,_,_,_,_)", node_gate_impl::<S>);
method("main", "Node", false, "gateThreshold=(_)", gate_set_threshold_impl::<S>);
method("main", "Node", false, "gateRatio=(_)",     gate_set_ratio_impl::<S>);
method("main", "Node", false, "gateAttack=(_)",    gate_set_attack_impl::<S>);
method("main", "Node", false, "gateRelease=(_)",   gate_set_release_impl::<S>);
method("main", "Node", false, "gateHold=(_)",      gate_set_hold_impl::<S>);
method("main", "Node", false, "gateRange=(_)",     gate_set_range_impl::<S>);
method("main", "Node", false, "gateDetector=(_)",  gate_set_detector_impl::<S>);
```
`METHODS` (bindings.rs ~903, beside the comp entries):
```rust
static_method("Node", "gate_(_,_,_,_,_,_,_,_)", bindings_audio::node_gate),
method("Node", "gateThreshold=(_)", bindings_audio::gate_set_threshold),
method("Node", "gateRatio=(_)",     bindings_audio::gate_set_ratio),
method("Node", "gateAttack=(_)",    bindings_audio::gate_set_attack),
method("Node", "gateRelease=(_)",   bindings_audio::gate_set_release),
method("Node", "gateHold=(_)",      bindings_audio::gate_set_hold),
method("Node", "gateRange=(_)",     bindings_audio::gate_set_range),
method("Node", "gateDetector=(_)",  bindings_audio::gate_set_detector),
```

- [ ] **Step 4: Prelude foreign decls + two classes**

`Node` class (prelude.wren ~142, beside `foreign static comp_`):
```
  foreign static gate_(input, threshold, ratio, attack, release, hold, range, detector)
  foreign gateThreshold=(v)
  foreign gateRatio=(v)
  foreign gateAttack=(v)
  foreign gateRelease=(v)
  foreign gateHold=(v)
  foreign gateRange=(v)
  foreign gateDetector=(v)
```
Two classes near `class Comp` (mirror its `polyMode_` guard shape verbatim):
```
class Expander {
  // Gentle downward expander: no hold, 20 dB range, peak detector.
  static new(input, threshold, ratio, attack, release) {
    if (Node.polyMode_ == 1) Fiber.abort("Expander is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.gate_(input, threshold, ratio, attack, release, 0, 20, 0)
  }
}

class Gate {
  // Hard noise gate: ratio 10, deep range (80 dB ≈ mute below threshold), peak, with hold.
  static new(input, threshold, attack, release, hold) {
    if (Node.polyMode_ == 1) Fiber.abort("Gate is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.gate_(input, threshold, 10, attack, release, hold, 80, 0)
  }
}
```

- [ ] **Step 5: Compile + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` (+ `--features deluge-dsp-kernels/simd`).
Gate: compiles clean + full existing suite green BOTH configs. If "Node does not implement gate_" / "metaclass does not implement" appears → a table registration is missing or a selector name mismatches. Nothing exercises `Gate` yet (Task 4). Expected: PASS both. (A `golden_sim` `metro_callback_fires`/`output_sets_cv` test is a KNOWN pre-existing timing flake — rerun if seen.)

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren
git commit -m "feat(wren): Gate.new / Expander.new — gate/expander factories + setters (both tables + prelude)"
```

---

### Task 4: End-to-end

**Files:**
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Expander.new`/`Gate.new` (Task 3); `run_and_render(src, &mut [StereoFrame; N])` (now renders all N frames, Fx-1 fix); `run_script_ok`.

- [ ] **Step 1: Write the failing e2e tests**

```rust
#[test]
fn expander_renders_finite_nonsilent() {
    let mut out = [StereoFrame::default(); 128];
    run_and_render("Out.patch(Expander.new(Osc.saw(110), -6, 2, 0.001, 0.1))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "bounded/finite");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3), "expander passes signal above threshold");
}

#[test]
fn gate_closes_below_threshold() {
    // High threshold (+6 dB, above the ~0 dB saw peak) → gate stays closed → attenuated.
    let peak = |buf: &[StereoFrame]| buf.iter().map(|f| f.l.abs()).fold(0.0f32, f32::max);
    let mut dry = [StereoFrame::default(); 128];
    run_and_render("Out.patch(Osc.saw(110))", &mut dry);
    let mut gated = [StereoFrame::default(); 128];
    run_and_render("Out.patch(Gate.new(Osc.saw(110), 6, 0.001, 0.05, 0.001))", &mut gated);
    assert!(gated.iter().all(|f| f.l.is_finite()), "gated finite");
    assert!(peak(&gated) < peak(&dry), "gate closed below threshold lowers peak (dry {} vs gated {})", peak(&dry), peak(&gated));
}

#[test]
fn gate_low_threshold_passes() {
    // Low threshold (-40, below the source) → gate stays open → non-silent.
    let mut out = [StereoFrame::default(); 128];
    run_and_render("Out.patch(Gate.new(Osc.saw(110), -40, 0.001, 0.05, 0.001))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "bounded");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3), "open gate passes loud source");
}
```

- [ ] **Step 2: Run (Gate live from Task 3)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- expander_renders gate_closes gate_low` (+ `--features deluge-dsp-kernels/simd`).
If `gate_closes_below_threshold` fails to show a peak reduction, the gate isn't closing (threshold below source? verify the dry peak — a saw peaks near 1.0 = 0 dB, so a +6 dB threshold keeps it closed) or a real Task 1–3 bug — investigate, do NOT weaken the `<`. Expected: PASS both.

- [ ] **Step 3: Full regression**

Run the FULL wren-core suite both configs. Existing synths/effects byte-unchanged. Expected: PASS. (Known pre-existing `golden_sim` flake — rerun if seen.)

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "test(wren): gate/expander e2e — expander passes, high-threshold gate closes, low-threshold passes"
```

---

## Self-Review Notes (for the executor)

- **INVERTED ballistics is the crux gotcha:** the gate uses `if target_gr < gr_db { attack } else { release }` — attack when GR is FALLING (opening), release when RISING (closing) — the OPPOSITE of `Comp`. Getting this backwards makes the gate open slowly / close instantly. The `gate_ballistics_open_and_close` + `gate_hold_keeps_open_then_closes` tests catch it.
- **Downward expansion:** GR applies BELOW threshold (`over < 0`), `GR = min((−over)(ratio−1), range)`, capped by `range_db`. At/above threshold GR = 0. Opposite of `Comp`.
- **Hold state machine:** `hold_ctr` reset to `hold_s/dt` when above threshold; counts down while below (staying open, GR target 0); expansion only after it hits 0. `hold_s == 0` ⇒ no hold.
- **Reuse, don't duplicate:** `Detector`, `lin_to_db`, `db_to_lin`, `one_pole_coeff`, `RMS_WINDOW_S`, `LEVEL_EPS` all already exist in `dynamics.rs` (Fx-1) — reuse them; do NOT re-declare.
- **`Comp` is untouched.** Purely additive everywhere. If an existing test needs editing, something is wrong.
- **Param-index mapping (0=threshold … 6=detector)** must match across kernel setters, node `set_param`, the Wren factory, and the 7 Wren setters — an off-by-one silently mis-maps a knob. Cross-check.
- **Both tables + prelude selectors identical** (`gate_`, `gateThreshold=`, …) — the "metaclass does not implement" gotcha. Both `Expander` AND `Gate` classes need the `polyMode_` guard.
- **Deferred (do NOT implement):** lookahead, sidechain/external key, GR metering, stereo-in/linked detection, hysteresis (dual threshold), duck/upward expansion.
