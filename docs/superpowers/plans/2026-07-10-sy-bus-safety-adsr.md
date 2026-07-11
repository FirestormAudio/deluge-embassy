# Sy-2e: Bus Poly-Safety + ADSR Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the last silent-mono footgun (`Bus.write` of a poly node inside a `Synth`) and add a full ADSR envelope (`Env.adsr(a,d,s,r)`) alongside `Env.ar`.

**Architecture:** Two independent deliverables in one branch. (1) A surgical Wren-layer guard on `Bus.write`. (2) A new `Adsr`/`PolyAdsr` kernel extending the existing `Ar` stage machine (scalar-per-voice, no f32x8), a graph node, and the `Env.adsr` Wren surface — reusing the existing per-voice gate model with no VoiceAllocator change.

**Tech Stack:** Rust `no_std`, Wren scripting via `deluge-wren-core`.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP paths. ARM Cortex-A9; host x86 for tests. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Additive, no regression:** `Ar`/`PolyAr`/`Env.ar` stay behaviorally unchanged.
- **Test invocation (per-crate, never `--workspace`; both configs; cargo rejects multiple bare positional names — use `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` and again `--features simd`.
  - Graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph <name>` and again `--features deluge-dsp-kernels/simd`.
  - Wren: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core <name>` and again `--features deluge-dsp-kernels/simd`.
  - An LSP `armv7a … can't find crate for test` error is environmental noise — ignore it.

## File Structure

- `crates/deluge-wren-core/wren/prelude.wren` — `Bus.write` guard wrapper; `Env.adsr` route.
- `crates/deluge-wren-core/src/bindings_audio.rs` — `write`→`write_` registration; `node_adsr_impl`/`node_polyadsr_impl`.
- `crates/deluge-wren-core/src/bindings.rs` — (if METHODS table lives here) register `adsr_`/`polyadsr_`.
- `crates/deluge-dsp-kernels/src/env.rs` — `Stage::Decay` + new `Adsr`.
- `crates/deluge-dsp-kernels/src/poly.rs` — new `PolyAdsr`.
- `crates/deluge-audio-graph/src/node.rs` — `Kind::Adsr`/`PolyAdsr`, State/predicate/dispatch/set_param/gate arms.
- Tests: the respective crate test modules + `crates/deluge-wren-core/tests/audio_bindings.rs`.

---

## Task 1: Bus poly-safety guard

**Files:**
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`Bus` class)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (registration `write(_)` → `write_(_)`, ~:1748)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Node.polyMode_` (Wren), `Node.isPoly_` (Wren getter, Sy-2d), the existing `bus_write_impl` (registered foreign).
- Produces: a guarded `Bus.write(src)` that aborts in poly mode when `src` is a poly `Node`.

- [ ] **Step 1: Write the failing tests**

In `tests/audio_bindings.rs` (mirror the existing abort/render harness — find `synth_error_cases_abort` / `run_and_render` / the abort helper `run_script_ok` and reuse them):

```rust
#[test]
fn bus_write_poly_source_aborts_in_synth() {
    // A poly node written to a bus inside a Synth builder must abort.
    assert!(!run_script_ok(
        "Synth.new { |p| var b = Bus.new(); b.write(Osc.saw(p)); Osc.saw(p) * Env.ar(0.01, 0.3) }"
    ));
}

#[test]
fn bus_write_mono_and_const_ok() {
    // A numeric constant to a bus inside a Synth is a safe mono write (no abort).
    assert!(run_script_ok(
        "Synth.new { |p| var b = Bus.new(); b.write(0.5); Osc.saw(p) * Env.ar(0.01, 0.3) }"
    ));
    // A normal bus write OUTSIDE a Synth (mono graph) is unaffected.
    assert!(run_script_ok("var b = Bus.new(); b.write(Osc.saw(110)); Out.patch(b)"));
}
```

> Match the exact helper names/signatures the existing tests use. If `run_script_ok` returns `true` on `WREN_RESULT_SUCCESS`, `!run_script_ok(...)` asserts an abort. If the harness needs an `Out.patch`/render to exercise the script, follow the existing abort tests' shape (they compile+run the builder, which is enough to trigger `Fiber.abort`).

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core bus_write_poly_source_aborts bus_write_mono_and_const_ok`
Expected: `bus_write_poly_source_aborts_in_synth` FAILS (no guard yet — the poly write currently succeeds).

- [ ] **Step 3: Rename the foreign method and add the guard wrapper**

In `prelude.wren`, replace the `Bus` class:

```wren
foreign class Bus {
  foreign static new_()      // returns a fresh Bus foreign in slot 0
  foreign write_(src)        // native bus write (renamed; guarded by `write` below)
  static new() { new_() }    // the public Bus.new()
  // Guard the Sy-2d silent-mono footgun: a poly voice signal written to a
  // (mono) bus inside a Synth bypasses the VoiceSum that `.out` inserts and is
  // read as a single interleaved row. A poly source is always a `Node`
  // (`return_poly_node`); Ports come only from Split etc., which abort in poly
  // mode, so `src is Node` fully covers it. A Num/mono write is safe.
  write(src) {
    if (Node.polyMode_ == 1 && (src is Node) && src.isPoly_ == 1) {
      Fiber.abort("can't write a poly voice to a Bus inside a Synth — route the Synth's .out to a bus instead (Sy-2e)")
    }
    write_(src)
  }
}
```

In `bindings_audio.rs` (~:1748), change the registration selector:

```rust
method("main", "Bus", false, "write_(_)", bus_write_impl::<S>);
```

(from `"write(_)"` → `"write_(_)"`; the `bus_write_impl` body is unchanged.)

> `Node` is the Wren foreign class every `Osc.*`/filter/poly factory returns; `src is Node` is true for a poly oscillator and false for a `Num` or a `Port`. This avoids needing `isPoly_` on `Port` (a `Port` can never be poly inside a Synth — `Split`/multi-out nodes abort in poly mode). This is a deliberate refinement of the spec's illustrative `!(src is Num)` form.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core bus_write_poly_source_aborts bus_write_mono_and_const_ok` and again `--features deluge-dsp-kernels/simd`.
Expected: PASS both.

- [ ] **Step 5: Regression + commit**

Run the full wren-core suite both configs (`cargo test ... -p deluge-wren-core` and `--features deluge-dsp-kernels/simd`) — no existing bus test regressed (the rename is transparent; the guard only fires on a poly Node in poly mode).

```bash
git add crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "fix(wren): guard Bus.write of a poly node inside a Synth (close last footgun)"
```

---

## Task 2: `Adsr` kernel

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/env.rs` (add `Stage::Decay`, new `Adsr`)
- Test: `env.rs` test module

**Interfaces:**
- Consumes: `In` (`crate::In`), the existing `Stage` enum.
- Produces: `pub struct Adsr` with `new()`, `gate(&mut self, on: bool)`, `trigger(&mut self)`, `set_sustain(&mut self, s: f32)`, `tick(&mut self, attack: f32, decay: f32, release: f32, dt: f32) -> f32`, `process(&mut self, attack: In, decay: In, release: In, dt: f32, out: &mut [f32])`. Used by Tasks 3-4.

- [ ] **Step 1: Add `Stage::Decay` (keep `Ar` unchanged)**

In env.rs, extend the enum (adding a variant `Ar` never enters — `Ar`'s match arms are unaffected, but confirm `Ar::tick`'s `match self.stage` stays exhaustive; a `_` arm or all variants covered):

```rust
#[derive(Clone, Copy, PartialEq)]
pub enum Stage {
    Idle,
    Attack,
    Decay,     // added — used only by Adsr
    Sustain,
    Release,
}
```

> If `Ar::tick`'s `match self.stage { ... }` does not have a catch-all and now fails to compile (non-exhaustive is not the risk — adding a variant to a matched enum makes existing exhaustive matches non-exhaustive), add a `Stage::Decay => { /* unreachable for Ar */ }` arm to `Ar::tick` that does nothing (or `self.level = self.level`), so `Ar` is behavior-identical. Run the existing `env`/`ar` tests after this edit to confirm `Ar` is unchanged.

- [ ] **Step 2: Write the failing `Adsr` tests**

```rust
#[cfg(test)]
mod adsr_tests {
    use super::*;

    #[test]
    fn adsr_attack_decay_sustain_release() {
        let dt = 1.0 / 48_000.0;
        let mut e = Adsr::new();
        e.set_sustain(0.5);
        e.gate(true);
        // Attack 10ms → reaches ~1.0
        let mut peak = 0.0f32;
        for _ in 0..(0.02 / dt as f64) as usize { peak = e.tick(0.01, 0.05, 0.1, dt); }
        assert!(peak > 0.99, "attack should reach ~1.0, got {peak}");
        // Decay 50ms → settles to sustain 0.5
        let mut lvl = peak;
        for _ in 0..(0.1 / dt as f64) as usize { lvl = e.tick(0.01, 0.05, 0.1, dt); }
        assert!((lvl - 0.5).abs() < 1e-3, "should hold sustain 0.5, got {lvl}");
        // Sustain holds
        for _ in 0..100 { lvl = e.tick(0.01, 0.05, 0.1, dt); }
        assert!((lvl - 0.5).abs() < 1e-3, "sustain must hold, got {lvl}");
        // Release from sustain → 0
        e.gate(false);
        let mut last = lvl;
        for _ in 0..(0.2 / dt as f64) as usize { last = e.tick(0.01, 0.05, 0.1, dt); }
        assert!(last < 1e-3, "release should reach 0, got {last}");
    }

    #[test]
    fn adsr_release_from_current_level_mid_decay() {
        let dt = 1.0 / 48_000.0;
        let mut e = Adsr::new();
        e.set_sustain(0.2);
        e.gate(true);
        // Attack quickly to ~1.0
        for _ in 0..(0.005 / dt as f64) as usize { e.tick(0.001, 1.0, 0.1, dt); }
        // Mid-decay (long decay so we're still above sustain), then release
        let mid = e.tick(0.001, 1.0, 0.1, dt);
        e.gate(false);
        let after = e.tick(0.001, 1.0, 0.1, dt);
        assert!(after < mid, "release must fall from the current level, {after} !< {mid}");
    }

    #[test]
    fn adsr_bounded_0_1() {
        let dt = 1.0 / 48_000.0;
        let mut e = Adsr::new();
        e.set_sustain(0.7);
        e.gate(true);
        for i in 0..8192 {
            if i == 4000 { e.gate(false); }
            let v = e.tick(0.01, 0.03, 0.05, dt);
            assert!((0.0..=1.0).contains(&v), "env out of [0,1]: {v}");
        }
    }
}
```

- [ ] **Step 3: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels adsr_`
Expected: FAIL — `Adsr` not found.

- [ ] **Step 4: Implement `Adsr`**

```rust
/// Attack/Decay/Sustain/Release envelope, gate-driven. `sustain` is the held
/// level in [0,1]. Serial state machine → scalar (like `Ar`). `Ar` is unchanged;
/// this is an additive full-ADSR sibling.
#[derive(Clone, Copy)]
pub struct Adsr {
    level: f32,
    stage: Stage,
    sustain: f32,
}

impl Adsr {
    pub fn new() -> Adsr {
        Adsr { level: 0.0, stage: Stage::Idle, sustain: 1.0 }
    }

    /// Gate on → attack (then decay → sustain); gate off → release.
    pub fn gate(&mut self, on: bool) {
        self.stage = if on { Stage::Attack } else { Stage::Release };
    }

    /// Enter attack. A percussive AD (no held tail) is achieved with sustain=0.
    pub fn trigger(&mut self) {
        self.stage = Stage::Attack;
    }

    pub fn set_sustain(&mut self, s: f32) {
        self.sustain = s.clamp(0.0, 1.0);
    }

    /// Advance one sample; returns the new level. a/d/r in seconds.
    pub fn tick(&mut self, attack: f32, decay: f32, release: f32, dt: f32) -> f32 {
        match self.stage {
            Stage::Attack => {
                self.level += dt / attack.max(0.0001);
                if self.level >= 1.0 {
                    self.level = 1.0;
                    self.stage = Stage::Decay;
                }
            }
            Stage::Decay => {
                self.level -= dt / decay.max(0.0001);
                if self.level <= self.sustain {
                    self.level = self.sustain;
                    self.stage = Stage::Sustain;
                }
            }
            Stage::Sustain => self.level = self.sustain,
            Stage::Release => {
                self.level -= dt / release.max(0.0001);
                if self.level <= 0.0 {
                    self.level = 0.0;
                    self.stage = Stage::Idle;
                }
            }
            Stage::Idle => self.level = 0.0,
        }
        self.level
    }

    pub fn process(&mut self, attack: In, decay: In, release: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            *s = self.tick(attack.at(i), decay.at(i), release.at(i), dt);
        }
    }
}

impl Default for Adsr {
    fn default() -> Self { Adsr::new() }
}
```

> Edge case in `adsr_attack_decay_sustain_release`: if `sustain >= 1.0` the Decay stage exits immediately (`level <= sustain` at 1.0) — fine. If `decay` is tiny it snaps to sustain in one step — fine. The `.max(0.0001)` mirrors `Ar`.

- [ ] **Step 5: Run to verify they pass + `Ar` regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels adsr_ ar_ env` and again `--features simd ...`.
Expected: PASS — `Adsr` tests green AND the existing `Ar`/`env` tests unchanged.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-dsp-kernels/src/env.rs
git commit -m "feat(kernels): Adsr envelope (Attack/Decay/Sustain/Release), Ar unchanged"
```

---

## Task 3: `PolyAdsr` kernel

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (new `PolyAdsr`)
- Test: `poly.rs` test module

**Interfaces:**
- Consumes: `Adsr` (Task 2), `In`, `VOICES`.
- Produces: `pub struct PolyAdsr` with `new()`, `gate_voice(&mut self, v: usize, on: bool)`, `trigger_voice(&mut self, v: usize)`, `set_sustain(&mut self, s: f32)`, `process(&mut self, attack: In, decay: In, release: In, dt: f32, out: &mut [f32])`. Used by Task 4.

- [ ] **Step 1: Write the failing tests**

In poly.rs test module:

```rust
#[test]
fn polyadsr_matches_mono_per_voice() {
    let dt = 1.0 / 48_000.0;
    let n = 512usize;
    let mut poly = PolyAdsr::new();
    poly.set_sustain(0.4);
    let mut refs: [Adsr; VOICES] = core::array::from_fn(|_| { let mut a = Adsr::new(); a.set_sustain(0.4); a });
    for v in 0..VOICES { poly.gate_voice(v, true); refs[v].gate(true); }
    let mut out = std::vec![0.0f32; n * VOICES];
    // shared a/d/r
    poly.process(In::K(0.01), In::K(0.05), In::K(0.1), dt, &mut out);
    for v in 0..VOICES {
        for i in 0..n {
            let want = refs[v].tick(0.01, 0.05, 0.1, dt);
            assert_eq!(out[i * VOICES + v], want, "lane {v} sample {i}");
        }
    }
}

#[test]
fn polyadsr_gates_are_independent() {
    let dt = 1.0 / 48_000.0;
    let mut poly = PolyAdsr::new();
    poly.set_sustain(1.0);
    for v in 0..VOICES { poly.gate_voice(v, true); }
    let mut out = std::vec![0.0f32; VOICES];
    // let all attack to sustain
    for _ in 0..2000 { poly.process(In::K(0.001), In::K(0.001), In::K(0.5), dt, &mut out); }
    poly.gate_voice(3, false); // release only voice 3
    for _ in 0..2000 { poly.process(In::K(0.001), In::K(0.001), In::K(0.5), dt, &mut out); }
    assert!(out[3] < out[0], "voice 3 released, others held: {} !< {}", out[3], out[0]);
}
```

> Match the test module's exact conventions for `In::K`/`In::A` and `std::vec`/`alloc::vec` — read how the `PolyAr` / other poly tests import and construct. Adjust `In::K` to the codebase's constant-`In` variant name.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels polyadsr_`
Expected: FAIL — `PolyAdsr` not found.

- [ ] **Step 3: Implement `PolyAdsr`** (mirror `PolyAr`)

```rust
/// Poly ADSR: 8 independent `Adsr`s, scalar-per-voice (the state machine is
/// branchy/cheap — no f32x8, like `PolyAr`). Shared mono a/d/r; per-node sustain.
#[derive(Clone, Copy)]
pub struct PolyAdsr {
    voices: [Adsr; VOICES],
}
impl PolyAdsr {
    pub fn new() -> PolyAdsr {
        PolyAdsr { voices: [Adsr::new(); VOICES] }
    }
    pub fn gate_voice(&mut self, v: usize, on: bool) {
        if v < VOICES { self.voices[v].gate(on); }
    }
    pub fn trigger_voice(&mut self, v: usize) {
        if v < VOICES { self.voices[v].trigger(); }
    }
    pub fn set_sustain(&mut self, s: f32) {
        for a in &mut self.voices { a.set_sustain(s); }
    }
    /// attack/decay/release mono controls; writes a voice-interleaved env tile.
    pub fn process(&mut self, attack: In, decay: In, release: In, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            let (atk, dec, rel) = (attack.at(i), decay.at(i), release.at(i));
            for v in 0..VOICES {
                out[i * VOICES + v] = self.voices[v].tick(atk, dec, rel, dt);
            }
        }
    }
}
impl Default for PolyAdsr {
    fn default() -> Self { PolyAdsr::new() }
}
```

> Add `use crate::env::Adsr;` (or match the existing `use ... Ar` import path in poly.rs).

- [ ] **Step 4: Run to verify they pass (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels polyadsr_` and again `--features simd`.
Expected: PASS both (scalar kernel; the `--features simd` build just confirms it compiles/passes there too).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/poly.rs
git commit -m "feat(kernels): PolyAdsr (scalar-per-voice, mirrors PolyAr) — bit-identical per-voice oracle"
```

---

## Task 4: Graph node

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs`
- Test: `node.rs` test module

**Interfaces:**
- Consumes: `Adsr`, `PolyAdsr` (Tasks 2-3), `In`, `VOICES`.
- Produces: `Kind::Adsr`, `Kind::PolyAdsr`; `State::Adsr(Adsr)`, `State::PolyAdsr(PolyAdsr)`; `set_param(0)` = sustain; `gate_voice`/`trigger_voice` dispatch to `PolyAdsr`. Used by Task 5.

- [ ] **Step 1: Add the Kinds/State + constructors**

- `Kind::Adsr` (mono), `Kind::PolyAdsr` (alongside `Kind::Env`/`Kind::PolyAr`).
- `State::Adsr(Adsr)`, `State::PolyAdsr(PolyAdsr)`.
- Constructor arms (beside `Kind::Env => State::Ar(...)` @:182 and `Kind::PolyAr => State::PolyAr(...)` @:210):

```rust
            Kind::Adsr => State::Adsr(Adsr::new()),
            Kind::PolyAdsr => State::PolyAdsr(PolyAdsr::new()),
```

Import `Adsr`/`PolyAdsr` into node.rs (beside the `Ar`/`PolyAr` imports).

- [ ] **Step 2: Predicates + dispatch + set_param + gate**

- `out_width`: `Kind::Adsr => 1` (default arm already gives 1 — no change needed unless the match is explicit); add `Kind::PolyAdsr` to the `=> VOICES` arm (@:236).
- `is_poly` (@:248): add `Kind::PolyAdsr` to the `matches!` list.
- `poly_in_count` (@ the `=> 0` region): `PolyAdsr` is a pure source → falls in the `_ => 0` arm like `PolyAr` (verify PolyAr is `0`; add `PolyAdsr` explicitly if the arm enumerates kinds).
- `process_resolved` (mono, beside `Kind::Env` @:461):

```rust
            Kind::Adsr => {
                if let State::Adsr(a) = &mut self.state {
                    a.process(ins[0], ins[1], ins[2], dt, outs.port(0));
                }
            }
```

- `poly_process` (beside `Kind::PolyAr` @:783):

```rust
            Kind::PolyAdsr => {
                if let State::PolyAdsr(a) = &mut self.state {
                    a.process(ins[0], ins[1], ins[2], dt, out);
                }
            }
```

- Add `PolyAdsr` to the `process_resolved` no-op poly arm (@:745) so the exhaustive match compiles.
- `set_param` (@:301, beside `State::Moog4(m) if param == 0 => m.set_drive(value)`):

```rust
            State::Adsr(a) if param == 0 => a.set_sustain(value),
            State::PolyAdsr(a) if param == 0 => a.set_sustain(value),
```

- `gate_voice` (@:293) and `trigger_voice` (@:296) — extend to also match `PolyAdsr`:

```rust
    pub fn gate_voice(&mut self, v: usize, on: bool) {
        match &mut self.state {
            State::PolyAr(a) => a.gate_voice(v, on),
            State::PolyAdsr(a) => a.gate_voice(v, on),
            _ => {}
        }
    }
    pub fn trigger_voice(&mut self, v: usize) {
        match &mut self.state {
            State::PolyAr(a) => a.trigger_voice(v),
            State::PolyAdsr(a) => a.trigger_voice(v),
            _ => {}
        }
    }
```

- [ ] **Step 3: Write the graph render test**

Mirror the existing `PolyAr` voice-render graph test. A `PolyAdsr` node with `set_param(0, 0.5)` gated on renders an env tile that attacks then holds ~0.5 on all gated lanes:

```rust
#[test]
fn polyadsr_node_sustains_at_set_level() {
    let mut n = Node::new(Kind::PolyAdsr, 0);
    // inputs [attack, decay, release]
    *n.input_mut(0).unwrap() = Input::Const(0.001);
    *n.input_mut(1).unwrap() = Input::Const(0.001);
    *n.input_mut(2).unwrap() = Input::Const(0.5);
    n.set_param(0, 0.5); // sustain
    for v in 0..VOICES { n.gate_voice(v, true); }
    // render enough blocks that attack+decay complete, read the tail
    // (use the node/poly_process harness the PolyAr graph test uses; assert
    //  every gated lane's steady value ≈ 0.5)
}
```

> Follow the exact harness the existing `PolyAr` node/poly test uses (how it drives `poly_process` with an empty/`None` poly_in and reads the interleaved out tile). Assert the steady-state lane value is within a small tolerance of 0.5.

- [ ] **Step 4: Run to verify (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph polyadsr` and again `--features deluge-dsp-kernels/simd`.
Expected: PASS both. Also run the full graph crate both configs — no existing test regressed.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(graph): Adsr/PolyAdsr nodes + set_param sustain + PolyAdsr gate dispatch"
```

---

## Task 5: Wren `Env.adsr` surface + end-to-end

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_adsr_impl`, `node_polyadsr_impl`, registration)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`Env.adsr`, foreign decls)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Kind::Adsr`, `Kind::PolyAdsr` (Task 4); `arg_input`, `audio::{alloc_node_id, new_node, poly_record_gate}`, `Cmd::SetParam` emit path, `return_node` (existing bindings helpers).
- Produces: Wren `Node.adsr_(a,d,s,r)` / `Node.polyadsr_(a,d,s,r)`; `Env.adsr` route.

- [ ] **Step 1: Add the Rust factories**

In bindings_audio.rs (mirror `node_env_impl` / `node_polyar_impl`). Inputs are `[attack, decay, release]`; sustain is emitted as a `SetParam(node, 0, sustain)`:

```rust
pub(crate) fn node_adsr_impl<S: SlotApi>(vm: &S) {
    let attack = arg_input(vm, 1);
    let decay = arg_input(vm, 2);
    let sustain = arg_f32(vm, 3);      // use the codebase's numeric-arg reader
    let release = arg_input(vm, 4);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Adsr, [attack, decay, release]);
    audio::set_param(id, 0, sustain);  // use the existing param-emit helper
    unsafe { return_node(vm, id) };
}

pub(crate) fn node_polyadsr_impl<S: SlotApi>(vm: &S) {
    let attack = arg_input(vm, 1);
    let decay = arg_input(vm, 2);
    let sustain = arg_f32(vm, 3);
    let release = arg_input(vm, 4);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolyAdsr, [attack, decay, release]);
    audio::set_param(id, 0, sustain);
    audio::poly_record_gate(id);       // ADSR is the amp gate (like polyar_)
    unsafe { return_node(vm, id) };
}
```

> Confirm the exact helpers: (a) how a scalar f64 arg is read (`arg_f32`/`arg_f64`/`get_slot_double` — find what `node_set_width_impl` or similar uses); (b) the control-plane param emit — there must be an `audio::set_param(node, param, value)` or a `Cmd::SetParam` emit helper (find how any existing `.foo =` setter emits `Cmd::SetParam`; e.g. `node_set_width_impl`). Use those exact names. `new_node` takes `[Input; 3]` — `[attack, decay, release]` fills all three (no trailing `Const(0.0)` needed since there are exactly 3).

- [ ] **Step 2: Register + foreign decls**

- bindings_audio.rs registration (beside `polyar_`/`env_`): `method("main", "Node", true, "adsr_(_,_,_,_)", node_adsr_impl::<S>);` and `method("main", "Node", true, "polyadsr_(_,_,_,_)", node_polyadsr_impl::<S>);` (arity 4).
- prelude.wren `Node` class: add `foreign static adsr_(a, d, s, r)` and `foreign static polyadsr_(a, d, s, r)` decls (the Sy-2c gotcha — missing decl errors "metaclass does not implement").

- [ ] **Step 3: Flip `Env.adsr` in prelude**

```wren
class Env {
  static ar(attack, release) {                 // unchanged
    if (Node.polyMode_ == 1) return Node.polyar_(attack, release)
    return Node.env_(attack, release)
  }
  static adsr(attack, decay, sustain, release) {
    if (Node.polyMode_ == 1) return Node.polyadsr_(attack, decay, sustain, release)
    return Node.adsr_(attack, decay, sustain, release)
  }
}
```

- [ ] **Step 4: End-to-end tests**

In tests/audio_bindings.rs (mirror the existing Synth render harness):

```rust
#[test]
fn synth_adsr_renders_and_sustains() {
    // Builds (adsr is the single amp gate → polyGateCount_ == 1), renders a note,
    // note-off releases. Assert bounded, non-silent, and the held region tracks
    // the 0.6 sustain (e.g. the sustained RMS is well below the attack peak but > 0).
    run_and_render("Synth.new { |p| Osc.saw(p) * Env.adsr(0.01, 0.1, 0.6, 0.3) }", /* finite, |s|<=8.1, non-silent */);
}

#[test]
fn synth_adsr_counts_as_amp_gate() {
    // Exactly one amp envelope — an ADSR satisfies the gate requirement; a Synth
    // with NO envelope still aborts.
    assert!(run_script_ok("Synth.new { |p| Osc.saw(p) * Env.adsr(0.01,0.1,0.6,0.3) }"));
    assert!(!run_script_ok("Synth.new { |p| Osc.saw(p) }")); // no amp gate → abort (unchanged rule)
}
```

> Use the exact harness helper names (`run_and_render` / `run_script_ok`). If the harness exposes per-sample output, add a stronger assertion that the sustained level ≈ 0.6 × the oscillator amplitude; otherwise assert bounded + non-silent + that it builds with `polyGateCount_ == 1`.

- [ ] **Step 5: Regression + commit**

Run the FULL wren-core suite both configs — `Env.ar` (used inside and outside a Synth) must be unchanged; no existing test regressed.

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` and `--features deluge-dsp-kernels/simd`.
Expected: PASS both.

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): Env.adsr(a,d,s,r) — poly ADSR voice, Env.ar unchanged"
```

---

## Self-Review Notes (for the executor)

- **Additive discipline:** `Ar`/`PolyAr`/`Env.ar` must stay behaviorally identical. Adding `Stage::Decay` to the shared enum is the one place `Ar` could break — after Task 2 Step 1, run the `ar`/`env` tests and confirm they're unchanged (a `Decay` arm in `Ar::tick` is a no-op).
- **Oracle discipline:** `PolyAdsr` reuses `Adsr` per voice, so the per-voice null-test (`polyadsr_matches_mono_per_voice`, `assert_eq!`) is bit-identical by construction — same as `PolyAr`.
- **Param plumbing:** attack/decay/release are the 3 node Inputs (`ins[0..2]`); sustain is `set_param(0)`. Do NOT try to pass sustain as a 4th Input — `MAX_INPUTS == 3`.
- **Gate model unchanged:** the VoiceAllocator is NOT touched. `PolyAdsr` receives `GateVoice` exactly like `PolyAr` (Task 4 extends `Node::gate_voice`/`trigger_voice`). Release is best-effort (lane freed on note-off) — intentional, matches `Env.ar`.
- **Bus guard mechanism:** `src is Node && src.isPoly_ == 1` — refines the spec's `!(src is Num)` form so no `isPoly_` is needed on `Port` (a `Port` can't be poly in a Synth). Do not add `isPoly_` to `Port`.
- **Confirm helper names** (`arg_f32`/`audio::set_param`/`run_script_ok`/`run_and_render`, the `In::K` constant variant) against the actual codebase before transcribing — match what neighboring code uses.
- **Deferred (do NOT implement):** release-tail voice protection, multiple envelopes per voice (`polyGateCount_>1` still aborts), Tb303/Modal poly, per-voice wavetable morph, f32x8 ADSR.
