# Osc part 2 — expressive oscillators (PWM + PM/FM) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add pulse-width modulation, phase-modulation/FM, and feedback FM to the band-limited oscillators — via the three existing input ports (freq/pmod/width) plus `SetParam` for feedback — with defaults that reproduce today's output exactly.

**Architecture:** The kernel (`Osc::process`) grows `pmod`/`width` inputs and a feedback scalar; the graph passes ports 1/2 and wires the previously-stubbed `Cmd::SetParam` to `Osc::set_feedback`; Wren adds `.pm`/`.width`/`.feedback` setters. The kernel signature change ripples to the graph (Task 2) then Wren (Task 3), P1-style.

**Tech Stack:** Rust, `no_std` kernel (host tests use `std` + `deluge-dsp-test`), `deluge-audio-graph`, `deluge-wren-core`. Host target `x86_64-unknown-linux-gnu`.

**Reference spec:** [Osc part 2 design](../specs/2026-07-07-osc-expressive-pwm-fm-design.md). Depends on merged Osc-1 + QA.

## Global Constraints

- No `MAX_INPUTS`/`MAX_ARGS` change — fits the existing 3 ports (0=freq, 1=pmod, 2=width) + `SetParam` for feedback.
- **Transparency is the linchpin:** with `pmod=0`, `width<=0 (→0.5)`, `feedback=0`, output equals the current band-limited oscillator — the existing Osc-1 goldens (`golden_saw_lpf_env_first_block`, `golden_saw_lpf_renders_expected_block`) and kernel aliasing tests must still pass unchanged.
- `no_std` in the kernel: use `floorf` (never `f32::fract()`). `deluge-dsp-test` is dev-only.
- **FM/PM aliasing is inherent** — FM tests gate *bounded/stable + expected sideband structure*, NOT a `worst_alias_db` floor. PWM (a real edge) IS alias-gated (measured floor, like Osc-1).
- Feedback uses the averaged previous outputs `0.5*(last+last2)` and a clamp (`[-1,1]`) for stability.
- Kernel tests: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`. Graph: add `--features simd`. Wren: `-p deluge-wren-core --features test-support`.
- Test output pristine (no warnings). Commit after each task.

---

## Task 1: Kernel — PWM + PM + feedback

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/osc.rs`

**Interfaces:**
- Produces: `Osc { phase, last, last2, feedback }`; `Osc::set_feedback(&mut self, f: f32)`; `Osc::process(&mut self, wave: Wave, freq: In, pmod: In, width: In, dt: f32, out: &mut [f32])` (NEW signature — adds `pmod`, `width`).
- Consumes: existing `poly_blep`, `poly_blamp`, `fast_sin`, `floorf`, `In`.

This changes `Osc::process`'s signature, which breaks `deluge-audio-graph`'s caller — that is fixed in Task 2. Task 1's verification builds only `-p deluge-dsp-kernels`.

- [ ] **Step 1: Update existing osc test call-sites to the new signature (transparency)**

Every existing call `osc.process(Wave::X, In::K(f), dt, &mut buf)` in `osc.rs`'s `mod tests` becomes `osc.process(Wave::X, In::K(f), In::K(0.0), In::K(0.0), dt, &mut buf)` (add `pmod=0`, `width=0`). These tests (`saw_is_band_limited`, `square_is_band_limited`, `triangle_is_band_limited`, `band_limited_*_beats_naive`, `saw_ramps_upward_over_a_cycle`, `sine_is_bounded_and_starts_near_zero`, `triangle_low_freq_shape_intact`, and the `osc_output_is_finite_and_bounded` proptest) must still pass unchanged after Step 4 — that IS the transparency proof (`width=0 → 0.5`, `pmod=0`, `feedback=0` reproduce Osc-1). Do this first so the crate compiles once the signature changes.

- [ ] **Step 2: Write the failing new-feature tests**

Add to `osc.rs`'s `mod tests`:

```rust
    #[test]
    fn pwm_is_band_limited() {
        let sr = 48_000.0;
        for &w in &[0.1f32, 0.3, 0.5] {
            for &f0 in &[2_000.0f32, 5_000.0] {
                let mut osc = Osc::new();
                let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
                osc.process(Wave::Square, In::K(f0), In::K(0.0), In::K(w), 1.0 / sr, &mut buf);
                let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
                let wa = spec.worst_alias_db(f0, 3.0 * spec.bin_hz);
                // 4-point PolyBLEP on both edges; ~-28 dB like the 50% square.
                // Measure and set just below the measured floor if it differs.
                assert!(wa < -25.0, "pwm w={w} f0={f0}: worst_alias {wa} dB should be < -25");
            }
        }
    }

    #[test]
    fn pwm_duty_cycle_tracks_width() {
        // At low freq the +1 fraction should ≈ width.
        let mut osc = Osc::new();
        let mut buf = [0.0f32; 4800]; // 100 Hz at 48k → 48 cycles
        osc.process(Wave::Square, In::K(100.0), In::K(0.0), In::K(0.3), 1.0 / 48_000.0, &mut buf);
        let high = buf.iter().filter(|&&s| s > 0.0).count() as f32 / buf.len() as f32;
        assert!((high - 0.3).abs() < 0.03, "duty ≈ 0.3, got {high}");
    }

    #[test]
    fn pm_produces_sidebands() {
        // Carrier 4000 Hz sine, phase-modulated by a 500 Hz sine at index 1.0
        // → energy should appear at 4000 ± 500 (sidebands).
        let sr = 48_000.0;
        let (fc, fm, index) = (4_000.0f32, 500.0f32, 1.0f32);
        let mut modbuf = [0.0f32; deluge_dsp_test::FFT_N];
        let mut m = Osc::new();
        m.process(Wave::Sine, In::K(fm), In::K(0.0), In::K(0.0), 1.0 / sr, &mut modbuf);
        for s in modbuf.iter_mut() { *s *= index; } // pmod in cycles
        let mut carrier = [0.0f32; deluge_dsp_test::FFT_N];
        let mut c = Osc::new();
        c.process(Wave::Sine, In::K(fc), In::A(&modbuf), In::K(0.0), 1.0 / sr, &mut carrier);
        let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &carrier);
        let fund = spec.level_at(fc);
        // First sidebands present well above the noise floor.
        assert!(spec.level_at(fc + fm) > 0.05 * fund, "upper sideband present");
        assert!(spec.level_at(fc - fm) > 0.05 * fund, "lower sideband present");
    }

    #[test]
    fn pm_zero_is_clean_sine() {
        let sr = 48_000.0;
        let f0 = 64.0 * (sr / deluge_dsp_test::FFT_N as f32);
        let mut osc = Osc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(Wave::Sine, In::K(f0), In::K(0.0), In::K(0.0), 1.0 / sr, &mut buf);
        let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
        assert!(spec.thd(f0, 5) < 1e-2, "pmod=0 sine stays clean");
    }

    proptest! {
        #[test]
        fn fm_and_feedback_stay_bounded(
            wave_idx in 0u8..4,
            freq in 20.0f32..=8_000.0,
            pm in -2.0f32..=2.0,
            fb in -1.0f32..=1.0,
        ) {
            let wave = match wave_idx { 0 => Wave::Sine, 1 => Wave::Saw, 2 => Wave::Square, _ => Wave::Tri };
            let mut osc = Osc::new();
            osc.set_feedback(fb);
            let mut out = [0.0f32; 256];
            osc.process(wave, In::K(freq), In::K(pm), In::K(0.5), 1.0 / 48_000.0, &mut out);
            for s in out { prop_assert!(s.is_finite() && s.abs() <= 4.0); }
        }
    }
```

(Note the FM/feedback bound is `±4.0`, not `±1.1` — PM/feedback legitimately push peaks past the base range; the point is *finite and bounded*, not alias-free.)

- [ ] **Step 3: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: FAIL to COMPILE first (the new tests + updated call-sites reference the new `process` signature and `set_feedback`, which don't exist yet).

- [ ] **Step 4: Implement the kernel**

Replace the `Osc` struct + `impl` in `osc.rs` (keep `poly_blep`/`poly_blamp`/`fast_sin`/`floorf` and the `Wave` enum as-is):

```rust
#[derive(Clone, Copy)]
pub struct Osc {
    phase: f32,
    last: f32,
    last2: f32,
    feedback: f32,
}

impl Osc {
    pub fn new() -> Osc {
        Osc { phase: 0.0, last: 0.0, last2: 0.0, feedback: 0.0 }
    }

    /// Feedback-FM depth (control-rate scalar), clamped for stability.
    pub fn set_feedback(&mut self, f: f32) {
        self.feedback = f.clamp(-1.0, 1.0);
    }

    /// Fill `out` with one block. `freq` Hz; `pmod` phase modulation in cycles;
    /// `width` PWM duty for Square (`<= 0` → 0.5, else clamped to `[0.01,0.99]`);
    /// `dt = 1/sr`. `pmod=0`, `width<=0`, feedback=0 reproduce the band-limited
    /// waveform exactly. Band-limiting tracks the carrier wrap; deep PM/feedback
    /// alias inherently (accepted).
    pub fn process(&mut self, wave: Wave, freq: In, pmod: In, width: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            let dtp = freq.at(i) * dt;
            // Effective phase = carrier + external PM + averaged self-feedback.
            let fb = self.feedback * 0.5 * (self.last + self.last2);
            let mut ph = self.phase + pmod.at(i) + fb;
            ph -= floorf(ph);

            let y = match wave {
                Wave::Sine => fast_sin(ph),
                Wave::Saw => (2.0 * ph - 1.0) - poly_blep(ph, dtp),
                Wave::Square => {
                    let mut w = width.at(i);
                    if w <= 0.0 { w = 0.5; }
                    let w = w.clamp(0.01, 0.99);
                    let naive = if ph < w { 1.0 } else { -1.0 };
                    let mut pw = ph - w;
                    pw -= floorf(pw); // phase relative to the falling edge
                    naive + poly_blep(ph, dtp) - poly_blep(pw, dtp)
                }
                Wave::Tri => {
                    let naive = 1.0 - 4.0 * (ph - 0.5).abs();
                    let mut p2 = ph + 0.5;
                    p2 -= floorf(p2);
                    naive + 8.0 * dtp * (poly_blamp(ph, dtp) - poly_blamp(p2, dtp))
                }
            };
            *s = y;

            self.last2 = self.last;
            self.last = y;
            self.phase += dtp;
            self.phase -= floorf(self.phase);
        }
    }
}

impl Default for Osc {
    fn default() -> Self {
        Osc::new()
    }
}
```

(This preserves Osc-1's exact band-limiting: with `pmod=0`/`fb=0`, `ph == phase`; the square's `frac(ph-0.5) == frac(ph+0.5)` equals Osc-1's edge; the tri arm is unchanged.)

- [ ] **Step 5: Run, verify pass; record PWM measurements**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: PASS — the updated Osc-1 tests still hold (transparency) and the new PWM/PM/feedback tests pass. Record the measured PWM `worst_alias_db` per width; if a width misses the `< -25` gate, set the gate just below the measured floor (document) — do NOT widen the kernel. Zero warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-dsp-kernels/src/osc.rs
git commit -m "feat(dsp-kernels): oscillator PWM + phase-mod/FM + feedback"
```

---

## Task 2: Graph — pass pmod/width, wire SetParam → feedback

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs`, `crates/deluge-audio-graph/src/engine.rs`

**Interfaces:**
- Consumes: the new `Osc::process(wave, freq, pmod, width, dt, out)` + `Osc::set_feedback` (Task 1).
- Produces: `Node::set_param(&mut self, param: u8, value: f32)`; `Engine::apply(Cmd::SetParam{..})` now dispatches to it.

Task 1 left `deluge-audio-graph` uncompilable (the osc arm still calls the old `process`). This task restores it.

- [ ] **Step 1: Confirm the break**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: FAIL to compile — `Osc::process` arity changed.

- [ ] **Step 2: Pass pmod/width in the osc dispatch**

In `crates/deluge-audio-graph/src/node.rs`, the `Kind::Sine | Kind::Saw | Kind::Square | Kind::Tri` arm currently calls `o.process(wave, ins[0], dt, outs.port(0))`. Change it to pass ports 1 (pmod) and 2 (width):

```rust
                if let State::Osc(o) = &mut self.state {
                    o.process(wave, ins[0], ins[1], ins[2], dt, outs.port(0));
                }
```

- [ ] **Step 3: Add `Node::set_param` and wire `SetParam`**

In `node.rs`, add a method on `Node` (near `gate`/`trigger`):

```rust
    /// Set a non-signal scalar parameter. For oscillators, `param 0` = feedback.
    pub fn set_param(&mut self, param: u8, value: f32) {
        if let State::Osc(o) = &mut self.state {
            if param == 0 {
                o.set_feedback(value);
            }
        }
    }
```

In `crates/deluge-audio-graph/src/engine.rs`, replace the `SetParam` no-op:

```rust
            Cmd::SetParam { node, param, value } => {
                if let Some(n) = self.arena.node_mut(node) {
                    n.set_param(param, value);
                }
            }
```

- [ ] **Step 4: Write a failing SetParam/feedback graph test**

Add to `engine.rs`'s tests:

```rust
    #[test]
    fn setparam_feedback_renders_bounded() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Sine);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(2_000.0);
        e.apply(Cmd::SetParam { node: NodeId(0), param: 0, value: 0.8 });
        e.render_block();
        let out = e.node_output(NodeId(0), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 4.0));
        assert!(out.iter().any(|&s| s != 0.0)); // feedback sine still oscillates
    }
```

- [ ] **Step 5: Run, verify pass (incl. transparency golden)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features simd`
Expected: PASS both — including `golden_saw_lpf_env_first_block` unchanged (transparency: the saw node's ports 1/2 default to `Const(0.0)` → pmod 0, width 0→0.5) and the new `setparam_feedback_renders_bounded`. Zero warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-audio-graph/src
git commit -m "feat(audio-graph): osc pmod/width ports + wire SetParam→feedback"
```

---

## Task 3: Wren — `.pm` / `.width` / `.feedback` setters

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs`, `src/bindings_audio.rs`, `src/bindings.rs`, `wren/prelude.wren`, `tests/audio_bindings.rs`

**Interfaces:**
- Produces: `audio::set_param(id: u16, param: u8, value: f32)`; Node setters `pm=`/`width=`/`feedback=`.
- Consumes: `deluge_audio_graph::Cmd::SetParam`.

- [ ] **Step 1: Add the `set_param` emitter**

In `crates/deluge-wren-core/src/audio.rs`, next to `set_input`, add:

```rust
pub fn set_param(id: u16, param: u8, value: f32) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::SetParam { node: NodeId(id), param, value });
}
```

- [ ] **Step 2: Write the failing Cmd-sequence tests**

Add to `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn osc_width_emits_setinput_port2() {
    let cmds = run_and_capture_cmds("var s = Osc.square(110)\ns.width = 0.3");
    assert!(cmds.iter().any(|c| *c == Cmd::SetInput {
        node: NodeId(0), port: 2, src: Input::Const(0.3)
    }));
}

#[test]
fn osc_pm_emits_setinput_port1() {
    let cmds = run_and_capture_cmds("var s = Osc.sine(440)\ns.pm = 0.5");
    assert!(cmds.iter().any(|c| *c == Cmd::SetInput {
        node: NodeId(0), port: 1, src: Input::Const(0.5)
    }));
}

#[test]
fn osc_feedback_emits_setparam() {
    let cmds = run_and_capture_cmds("var s = Osc.sine(440)\ns.feedback = 0.8");
    assert!(cmds.iter().any(|c| *c == Cmd::SetParam {
        node: NodeId(0), param: 0, value: 0.8
    }));
}
```

- [ ] **Step 3: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: FAIL — `width=`/`pm=`/`feedback=` are unknown Wren methods.

- [ ] **Step 4: Implement the three setters**

In `crates/deluge-wren-core/src/bindings_audio.rs`, add (mirroring `node_set_cutoff_impl` + its `#[cfg(feature = "wren-sys-backend")] extern "C"` wrapper for each):

```rust
pub(crate) fn node_set_pm_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 1, v); // port 1 = phase-mod
}
pub(crate) fn node_set_width_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 2, v); // port 2 = PWM width
}
pub(crate) fn node_set_feedback_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // scalar param, not a Node/Input
    audio::set_param(self_id(vm), 0, v);
}
```

Add the `extern "C"` wrappers (each: `let vm = Vm(raw); node_set_*_impl(&vm);`) as the other setters have.

In `register_audio`, add (next to the `freq=`/`cutoff=` lines):

```rust
    method("main", "Node", false, "pm=(_)", node_set_pm_impl::<S>);
    method("main", "Node", false, "width=(_)", node_set_width_impl::<S>);
    method("main", "Node", false, "feedback=(_)", node_set_feedback_impl::<S>);
```

In the `#[cfg(feature = "wren-sys-backend")]` `METHODS` static table (`bindings.rs`), add the three matching entries binding the `extern "C"` wrappers with the same signatures.

- [ ] **Step 5: Add the prelude members**

In `crates/deluge-wren-core/wren/prelude.wren`, add to the `Node` foreign class (next to `foreign cutoff=(v)`):

```wren
  foreign pm=(v)
  foreign width=(v)
  foreign feedback=(v)
```

- [ ] **Step 6: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: PASS — the three new Cmd-sequence tests plus all existing tests (goldens unchanged). Zero warnings.

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-wren-core
git commit -m "feat(wren-core): osc .pm / .width / .feedback setters"
```

---

## Self-review notes

- **Spec coverage:** PWM (Task 1 kernel + alias/duty tests; Task 2 port 2; Task 3 `.width`); PM/FM (Task 1 pmod + sideband/clean tests; Task 2 port 1; Task 3 `.pm`); feedback (Task 1 scalar + stability proptest; Task 2 `SetParam`→`set_feedback`; Task 3 `.feedback`); transparency (defaults reproduce Osc-1 — updated kernel tests + unchanged goldens); no `MAX_INPUTS` bump; `SetParam` gets its first real use (Task 2).
- **Deliberate choices:** FM/feedback bound is `±4.0` (finite/stable), not `±1.1` — PM legitimately overshoots; the alias floor is only gated for PWM (a real edge). `width <= 0 → 0.5` keeps defaults transparent given `Const(0.0)` unset ports.
- **Atomicity:** Task 1 changes `Osc::process`'s signature, breaking `deluge-audio-graph` until Task 2 (verified via `-p deluge-dsp-kernels`). Task 2 restores the whole workspace.
- **Known follow-ups (not this cut):** hard sync (own sub-project), wavetable, noise variants, audio-rate feedback modulation, SIMD, through-zero FM refinements.
- **Risk note:** the `pwm_is_band_limited` threshold (`< -25`) and the `pm_produces_sidebands` sideband-level bar are the two measured/empirical gates — set from the first run, like Osc-1.
