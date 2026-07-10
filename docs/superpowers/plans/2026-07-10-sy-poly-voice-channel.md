# Sy-2: Poly Voice Channel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the Sy-1 poly plumbing into a per-voice-gated synth voice — a poly envelope (`PolyAr`), poly SVF lowpass (`PolySvf`), poly multiply (`PolyMul`), and per-voice gate/trigger — rendering `PolyCtrl → PolyOsc → PolySvf → PolyMul(·, PolyAr) → VoiceSum`.

**Architecture:** Three new poly kernels reusing the audited scalar `Ar`/`Svf` (via an extracted `Ar::tick` and the existing `Svf::tick`), with `PolySvf` also carrying an `f32x8` fast path. The engine generalizes from one poly input to `poly_in_count(kind) ∈ {0,1,2}` (first `k` ports poly), resolving two voice-interleaved `poly_scratch` tiles and passing them plus the mono control inputs to a widened `poly_process`.

**Tech Stack:** Rust `no_std` (`deluge-dsp-kernels`), audio graph/engine (`deluge-audio-graph`), `core::simd`, `proptest`.

## Global Constraints

- `no_std`, no heap, pure `f32`, deterministic. Poly nodes mono-per-voice, voice-interleaved tiles `tile[i*VOICES+v]`.
- Host tests per-crate on x86: `cargo test --target x86_64-unknown-linux-gnu -p <crate>`; `PolySvf` also `--features deluge-dsp-kernels/simd` (nightly toolchain, default). Never `--workspace` on host.
- LSP armv7a "can't find crate for test/std" diagnostics are noise — ignore.
- The scalar path is the correctness oracle; the `f32x8` path is null-tested against it (the `math.rs`/`PolyOsc` convention). `#[cfg(feature="simd")] const _: () = assert!(VOICES == 8)` guards any f32x8 poly kernel.
- No Wren surface (Sy-4). No panics on the audio path; a dangling/short poly edge → silence.
- **`Ar::tick` extraction must be behavior-preserving** — the existing `env` tests pass verbatim.
- Poly port convention: the **first `poly_in_count(kind)` ports are poly edges**, the rest mono controls. `PolyCtrl`/`PolyAr`→0, `PolyOsc`/`PolySvf`/`VoiceSum`→1, `PolyMul`→2.

---

### Task 1: `Ar::tick` extraction + `PolyAr` (pure poly envelope)

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/env.rs` (extract `Ar::tick`)
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (add `PolyAr`)

**Interfaces:**
- Consumes: `crate::env::Ar` (with the new `tick`), `crate::In`, `poly::VOICES`.
- Produces: `Ar::tick(&mut self, attack: f32, release: f32, dt: f32) -> f32`;
  `PolyAr` with `new()`, `gate_voice(v: usize, on: bool)`, `trigger_voice(v: usize)`,
  `process(attack: In, release: In, dt: f32, out: &mut [f32])`.

- [ ] **Step 1: Extract `Ar::tick`, refactor `Ar::process`**

In `env.rs`, add a `tick` method and make `process` call it (per-sample body moved verbatim; the `.max(0.0001)` clamps move into `tick`):

```rust
impl Ar {
    /// Advance one sample; returns the new level. attack/release in seconds.
    pub fn tick(&mut self, attack: f32, release: f32, dt: f32) -> f32 {
        let atk = attack.max(0.0001);
        let rel = release.max(0.0001);
        match self.stage {
            Stage::Attack => {
                self.level += dt / atk;
                if self.level >= 1.0 {
                    self.level = 1.0;
                    self.stage = if self.oneshot { Stage::Release } else { Stage::Sustain };
                }
            }
            Stage::Sustain => self.level = 1.0,
            Stage::Release => {
                self.level -= dt / rel;
                if self.level <= 0.0 {
                    self.level = 0.0;
                    self.stage = Stage::Idle;
                }
            }
            Stage::Idle => self.level = 0.0,
        }
        self.level
    }

    pub fn process(&mut self, attack: In, release: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            *s = self.tick(attack.at(i), release.at(i), dt);
        }
    }
}
```

- [ ] **Step 2: Verify the refactor is behavior-preserving**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels env`
Expected: PASS — the existing `env` tests are unchanged and still pass.

- [ ] **Step 3: Add `PolyAr` to `poly.rs`**

At the top of `poly.rs`, add `use crate::env::Ar;` (next to the existing uses). Then add:

```rust
/// A pure poly envelope source: 8 independent attack/release envelopes with
/// per-voice gate state. attack/release are shared (mono) controls; output is a
/// voice-interleaved tile of levels ∈ [0,1]. No audio input — a modulation
/// source. Scalar (the AR state machine is branchy and cheap).
#[derive(Clone, Copy)]
pub struct PolyAr {
    voices: [Ar; VOICES],
}
impl PolyAr {
    pub fn new() -> PolyAr {
        PolyAr { voices: [Ar::new(); VOICES] }
    }
    pub fn gate_voice(&mut self, v: usize, on: bool) {
        if v < VOICES { self.voices[v].gate(on); }
    }
    pub fn trigger_voice(&mut self, v: usize) {
        if v < VOICES { self.voices[v].trigger(); }
    }
    /// attack/release mono controls; writes a voice-interleaved env tile.
    pub fn process(&mut self, attack: In, release: In, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            let atk = attack.at(i);
            let rel = release.at(i);
            for v in 0..VOICES {
                out[i * VOICES + v] = self.voices[v].tick(atk, rel, dt);
            }
        }
    }
}
impl Default for PolyAr {
    fn default() -> Self { Self::new() }
}
```

- [ ] **Step 4: Add `PolyAr` tests**

In `poly.rs`'s `#[cfg(test)] mod tests`, add:

```rust
    #[test]
    fn polyar_gates_per_voice_independently() {
        let dt = 1.0 / 48_000.0;
        let mut env = PolyAr::new();
        env.gate_voice(0, true); // voice 0 attacks
        env.gate_voice(3, true); // voice 3 attacks
        let n = 4800; // 100 ms
        // attack/release are mono broadcast blocks (same for all voices per sample).
        let atk_mono: std::vec::Vec<f32> = (0..n).map(|_| 0.01).collect(); // 10 ms attack
        let rel_mono: std::vec::Vec<f32> = (0..n).map(|_| 0.05).collect();
        let mut out = std::vec![0.0f32; VOICES * n];
        env.process(In::A(&atk_mono), In::A(&rel_mono), dt, &mut out);
        // Gated voices reach ~1.0 by the end of a 100 ms window (10 ms attack).
        assert!((out[(n - 1) * VOICES + 0] - 1.0).abs() < 1e-3, "voice 0 reached sustain");
        assert!((out[(n - 1) * VOICES + 3] - 1.0).abs() < 1e-3, "voice 3 reached sustain");
        // Ungated voices stay silent.
        for v in [1usize, 2, 4, 5, 6, 7] {
            assert!(out[(n - 1) * VOICES + v].abs() < 1e-9, "voice {v} silent");
        }
        // Monotonic rise for voice 0 over the attack.
        assert!(out[10 * VOICES + 0] > out[0 * VOICES + 0]);
    }

    #[test]
    fn polyar_release_returns_to_zero() {
        let dt = 1.0 / 48_000.0;
        let mut env = PolyAr::new();
        env.gate_voice(0, true);
        let n = 480;
        let atk: std::vec::Vec<f32> = (0..n).map(|_| 0.001).collect(); // 1 ms
        let rel: std::vec::Vec<f32> = (0..n).map(|_| 0.001).collect();
        let mut out = std::vec![0.0f32; VOICES * n];
        env.process(In::A(&atk), In::A(&rel), dt, &mut out);
        assert!((out[(n - 1) * VOICES] - 1.0).abs() < 1e-3, "reached sustain");
        env.gate_voice(0, false); // release
        let mut out2 = std::vec![0.0f32; VOICES * n];
        env.process(In::A(&atk), In::A(&rel), dt, &mut out2);
        assert!(out2[(n - 1) * VOICES].abs() < 1e-3, "released to 0");
    }
```

(The `atk`/`rel` interleaved vecs above are unused scaffolding kept minimal — the
mono broadcast blocks `atk_mono`/`rel_mono` are what `process` reads; delete the
unused `atk`/`rel`/`_ =` lines if your linter prefers.)

- [ ] **Step 5: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels poly`
Expected: PASS — `polyar_gates_per_voice_independently`, `polyar_release_returns_to_zero`, plus the existing Sy-1 poly tests. No warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-dsp-kernels/src/env.rs crates/deluge-dsp-kernels/src/poly.rs
git commit -m "feat(dsp-kernels): Ar::tick + PolyAr poly envelope source"
```

---

### Task 2: `PolySvf` (scalar oracle + f32x8) + `poly_mul`

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/poly.rs`

**Interfaces:**
- Consumes: `crate::filter::{Svf, svf_coeffs, svf_k_from_res, svf_tan_prewarp, SvfResp}` (all `pub(crate)`), `crate::In`, `poly::VOICES`.
- Produces: `PolySvf` with `new()` and `process(audio: &[f32], cutoff: In, res: In, dt: f32, out: &mut [f32])`; free fn `poly_mul(a: &[f32], b: &[f32], out: &mut [f32])`.

- [ ] **Step 1: Add imports + the shared coefficient helper**

At the top of `poly.rs` add:

```rust
use crate::filter::{svf_coeffs, svf_k_from_res, svf_tan_prewarp, Svf, SvfResp};
use core::f32::consts::PI;
```

Add a shared scalar coeff helper (both paths use it, so they agree exactly):

```rust
/// Scalar SVF coefficients for a shared (mono) cutoff/res. Returns (k, a1, a2, a3).
/// Uses the polynomial prewarp (matches scalar `Svf`'s audio-rate path).
#[inline]
fn poly_svf_coeffs(fc: f32, res: f32, dt: f32) -> (f32, f32, f32, f32) {
    let theta = (PI * fc.max(1.0) * dt).min(0.49 * PI);
    let g = svf_tan_prewarp(theta);
    let k = svf_k_from_res(res);
    let (a1, a2, a3) = svf_coeffs(g, k);
    (k, a1, a2, a3)
}
```

- [ ] **Step 2: Add `PolySvf` (cfg-selected state) + `poly_mul`**

```rust
/// Poly SVF lowpass. Poly audio in → 8 filtered lanes; shared mono cutoff/res.
/// Scalar path holds `[Svf; VOICES]` and reuses the audited `Svf::tick`; the
/// SIMD path keeps SoA `f32x8` state register-resident. Both use identical
/// scalar coeffs, so they agree.
#[cfg(feature = "simd")]
const _: () = assert!(VOICES == 8);

#[derive(Clone, Copy)]
pub struct PolySvf {
    #[cfg(not(feature = "simd"))]
    voices: [Svf; VOICES],
    #[cfg(feature = "simd")]
    ic1: [f32; VOICES],
    #[cfg(feature = "simd")]
    ic2: [f32; VOICES],
}
impl PolySvf {
    #[cfg(not(feature = "simd"))]
    pub fn new() -> PolySvf {
        PolySvf { voices: [Svf::new(); VOICES] }
    }
    #[cfg(feature = "simd")]
    pub fn new() -> PolySvf {
        PolySvf { ic1: [0.0; VOICES], ic2: [0.0; VOICES] }
    }

    /// `audio` = voice-interleaved poly input; cutoff/res mono; LP output tile.
    pub fn process(&mut self, audio: &[f32], cutoff: In, res: In, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            let two = f32x8::splat(2.0);
            let mut ic1 = f32x8::from_array(self.ic1);
            let mut ic2 = f32x8::from_array(self.ic2);
            for i in 0..n {
                let (_k, a1, a2, a3) = poly_svf_coeffs(cutoff.at(i), res.at(i), dt);
                let (a1v, a2v, a3v) = (f32x8::splat(a1), f32x8::splat(a2), f32x8::splat(a3));
                let v0 = f32x8::from_slice(&audio[i * VOICES..]);
                let v3 = v0 - ic2;
                let v1 = a1v * ic1 + a2v * v3;
                let v2 = ic2 + a2v * ic1 + a3v * v3;
                ic1 = two * v1 - ic1;
                ic2 = two * v2 - ic2;
                v2.copy_to_slice(&mut out[i * VOICES..]); // LP = v2
            }
            self.ic1 = ic1.to_array();
            self.ic2 = ic2.to_array();
        }
        #[cfg(not(feature = "simd"))]
        {
            for i in 0..n {
                let (k, a1, a2, a3) = poly_svf_coeffs(cutoff.at(i), res.at(i), dt);
                for v in 0..VOICES {
                    out[i * VOICES + v] =
                        self.voices[v].tick(audio[i * VOICES + v], k, a1, a2, a3, SvfResp::Lp);
                }
            }
        }
    }
}
impl Default for PolySvf {
    fn default() -> Self { Self::new() }
}

/// Poly × poly, lanewise: `out[j] = a[j] * b[j]`. The VCA. Auto-vectorizes.
pub fn poly_mul(a: &[f32], b: &[f32], out: &mut [f32]) {
    for j in 0..out.len() {
        out[j] = a[j] * b[j];
    }
}
```

- [ ] **Step 3: Add tests**

```rust
    #[test]
    fn polysvf_attenuates_above_cutoff() {
        // A high-frequency input (18 kHz) fed to a low cutoff (300 Hz) comes out
        // much smaller; a low tone (100 Hz) passes ~unchanged.
        let dt = 1.0 / 48_000.0;
        let n = 4800;
        let hi = make_sine_tile(18_000.0, n, dt);
        let lo = make_sine_tile(100.0, n, dt);
        let cutoff: std::vec::Vec<f32> = (0..n).map(|_| 300.0).collect();
        let res: std::vec::Vec<f32> = (0..n).map(|_| 0.0).collect();
        let mut out_hi = std::vec![0.0f32; VOICES * n];
        let mut out_lo = std::vec![0.0f32; VOICES * n];
        PolySvf::new().process(&hi, In::A(&cutoff), In::A(&res), dt, &mut out_hi);
        PolySvf::new().process(&lo, In::A(&cutoff), In::A(&res), dt, &mut out_lo);
        let peak = |t: &[f32]| t[VOICES * n / 2..].iter().fold(0.0f32, |m, &s| m.max(s.abs()));
        assert!(peak(&out_hi) < 0.3, "18 kHz attenuated: {}", peak(&out_hi));
        assert!(peak(&out_lo) > 0.7, "100 Hz passes: {}", peak(&out_lo));
    }

    #[test]
    fn polysvf_matches_scalar_svf_single_voice() {
        // Cross-check: PolySvf voice 0 ≈ a scalar Svf on the same signal.
        use crate::filter::{Svf, SvfResp};
        let dt = 1.0 / 48_000.0;
        let n = 2000;
        let sig = make_sine_tile(1000.0, n, dt); // all 8 voices identical here
        let cutoff: std::vec::Vec<f32> = (0..n).map(|i| 500.0 + i as f32).collect(); // audio-rate → both use prewarp
        let res: std::vec::Vec<f32> = (0..n).map(|_| 0.3).collect();
        let mut poly_out = std::vec![0.0f32; VOICES * n];
        PolySvf::new().process(&sig, In::A(&cutoff), In::A(&res), dt, &mut poly_out);
        let mono_in: std::vec::Vec<f32> = (0..n).map(|i| sig[i * VOICES]).collect();
        let mut mono_out = std::vec![0.0f32; n];
        Svf::new().process(In::A(&mono_in), In::A(&cutoff), In::A(&res), SvfResp::Lp, dt, &mut mono_out);
        for i in 0..n {
            assert!((poly_out[i * VOICES] - mono_out[i]).abs() < 2e-3, "sample {i}");
        }
    }

    #[test]
    fn poly_mul_lanewise() {
        let a: [f32; VOICES * 2] = core::array::from_fn(|j| j as f32);
        let b: [f32; VOICES * 2] = core::array::from_fn(|_| 2.0);
        let mut out = [0.0f32; VOICES * 2];
        poly_mul(&a, &b, &mut out);
        for j in 0..VOICES * 2 {
            assert_eq!(out[j], j as f32 * 2.0);
        }
    }
```

Add this test helper inside the `tests` module (interleaved sine tile, all voices
identical — used by the filter tests):

```rust
    fn make_sine_tile(freq: f32, n: usize, dt: f32) -> std::vec::Vec<f32> {
        let mut t = std::vec![0.0f32; VOICES * n];
        let mut ph = 0.0f32;
        for i in 0..n {
            ph += freq * dt;
            ph -= floorf(ph);
            let s = fast_sin(ph);
            for v in 0..VOICES {
                t[i * VOICES + v] = s;
            }
        }
        t
    }
```

- [ ] **Step 4: Run BOTH feature configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels poly`
Expected: PASS (scalar).
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd poly`
Expected: PASS — the same tests, now exercising the `f32x8` PolySvf (the cross-check + attenuation tests are the null test). No warnings in either config.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/poly.rs
git commit -m "feat(dsp-kernels): PolySvf (scalar oracle + f32x8) + poly_mul"
```

---

### Task 3: Graph + engine — 3 kinds, two poly inputs, per-voice gate

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs`
- Modify: `crates/deluge-audio-graph/src/engine.rs`
- Modify: `crates/deluge-audio-graph/src/cmd.rs`

**Interfaces:**
- Consumes: `deluge_dsp_kernels::poly::{PolyAr, PolySvf, poly_mul}` (Tasks 1–2).
- Produces: `Kind::{PolyAr, PolySvf, PolyMul}`; `Node::poly_in_count(kind) -> usize` (replaces `has_poly_in`); `Node::poly_process(&mut self, ins: &[In; MAX_INPUTS], poly_in: [Option<&[f32]>; 2], dt, out: &mut [f32])`; `Node::gate_voice(v, on)`/`trigger_voice(v)`; `Cmd::GateVoice`/`TriggerVoice`.

- [ ] **Step 1: Kernel imports + `Kind`/`State` variants (node.rs)**

Add to the `use deluge_dsp_kernels::{ ... }` block: `poly::{poly_mul, PolyAr, PolySvf, PolyCtrl, PolyOsc, VOICES, voice_sum},` (extend the existing `poly::{...}` line with `poly_mul, PolyAr, PolySvf`).

In `enum Kind`, after `VoiceSum,` add:

```rust
    PolyAr,
    PolySvf,
    PolyMul,
```

In `enum State`, after `PolyOsc(PolyOsc),` add:

```rust
    PolyAr(PolyAr),
    PolySvf(PolySvf),
```

(`PolyMul` is stateless.)

- [ ] **Step 2: Construction, width, predicates (node.rs)**

`Node::new`: add `Kind::PolyMul` to the `State::Stateless` arm (append `| Kind::PolyMul` to the `Kind::Mul | ... | Kind::VoiceSum => State::Stateless,` line), and add:

```rust
            Kind::PolyAr => State::PolyAr(PolyAr::new()),
            Kind::PolySvf => State::PolySvf(PolySvf::new()),
```

`out_width`: extend the poly arm to `Kind::PolyCtrl | Kind::PolyOsc | Kind::PolyAr | Kind::PolySvf | Kind::PolyMul => VOICES,`.

`is_poly`: extend to `matches!(kind, Kind::PolyCtrl | Kind::PolyOsc | Kind::VoiceSum | Kind::PolyAr | Kind::PolySvf | Kind::PolyMul)`.

Replace `has_poly_in` with `poly_in_count`:

```rust
    /// Number of leading input ports that are poly edges (the rest are mono
    /// controls). Generalizes the Sy-1 single-poly-input model.
    pub fn poly_in_count(kind: Kind) -> usize {
        match kind {
            Kind::PolyOsc | Kind::PolySvf | Kind::VoiceSum => 1,
            Kind::PolyMul => 2,
            _ => 0, // PolyCtrl, PolyAr, and all mono kinds
        }
    }
```

- [ ] **Step 3: Per-voice gate/trigger + the widened `poly_process` (node.rs)**

Add (next to `gate`/`trigger`):

```rust
    pub fn gate_voice(&mut self, v: usize, on: bool) {
        if let State::PolyAr(a) = &mut self.state { a.gate_voice(v, on); }
    }
    pub fn trigger_voice(&mut self, v: usize) {
        if let State::PolyAr(a) = &mut self.state { a.trigger_voice(v); }
    }
```

Replace `poly_process` entirely with the widened version (mono `ins` + two poly tiles):

```rust
    /// Dispatch a poly node. `ins` = the resolved mono control ports; `poly_in`
    /// = up to two voice-interleaved input tiles (`poly_in[j]` is `Some` for
    /// `j < poly_in_count`). `out` is the writable region (VOICES*BLOCK, or BLOCK
    /// for VoiceSum).
    pub fn poly_process(
        &mut self,
        ins: &[In; MAX_INPUTS],
        poly_in: [Option<&[f32]>; 2],
        dt: f32,
        out: &mut [f32],
    ) {
        match self.kind {
            Kind::PolyCtrl => {
                if let State::PolyCtrl(c) = &mut self.state { c.process(out); }
            }
            Kind::PolyOsc => {
                if let (State::PolyOsc(o), Some(pin)) = (&mut self.state, poly_in[0]) {
                    o.process(pin, dt, out);
                }
            }
            Kind::VoiceSum => {
                if let Some(pin) = poly_in[0] { voice_sum(pin, out); }
            }
            Kind::PolyAr => {
                if let State::PolyAr(a) = &mut self.state {
                    a.process(ins[0], ins[1], dt, out);
                }
            }
            Kind::PolySvf => {
                if let (State::PolySvf(s), Some(audio)) = (&mut self.state, poly_in[0]) {
                    s.process(audio, ins[1], ins[2], dt, out);
                }
            }
            Kind::PolyMul => {
                if let (Some(a), Some(b)) = (poly_in[0], poly_in[1]) {
                    poly_mul(a, b, out);
                }
            }
            _ => {}
        }
    }
```

Extend the `process_resolved` poly no-op arm to all six poly kinds:
`Kind::PolyCtrl | Kind::PolyOsc | Kind::VoiceSum | Kind::PolyAr | Kind::PolySvf | Kind::PolyMul => {}`.

- [ ] **Step 4: Update the Sy-1 node-layer tests to the new signatures (node.rs)**

In `poly_widths_and_predicates`, replace the `has_poly_in` asserts:

```rust
        assert_eq!(Node::poly_in_count(Kind::PolyOsc), 1);
        assert_eq!(Node::poly_in_count(Kind::VoiceSum), 1);
        assert_eq!(Node::poly_in_count(Kind::PolyCtrl), 0);
        assert_eq!(Node::poly_in_count(Kind::PolyMul), 2);
        assert_eq!(Node::poly_in_count(Kind::PolyAr), 0);
```

In `polyctrl_node_fills_and_voicesum_collapses`, update the two `poly_process`
calls to the new signature:

```rust
        let dummy: [In; MAX_INPUTS] = [In::K(0.0); MAX_INPUTS];
        ctrl.poly_process(&dummy, [None, None], 1.0 / 48_000.0, &mut tile);
        // ...
        sum.poly_process(&dummy, [Some(&tile), None], 1.0 / 48_000.0, &mut mono);
```

(Ensure `In` and `MAX_INPUTS` are in scope in the test module — they are via
`use super::*`.)

- [ ] **Step 5: `Cmd::GateVoice`/`TriggerVoice` (cmd.rs)**

In `enum Cmd`, after `Trigger { node: NodeId },` add:

```rust
    GateVoice { node: NodeId, voice: u8, on: bool },
    TriggerVoice { node: NodeId, voice: u8 },
```

- [ ] **Step 6: Engine — two-poly-input resolution + dispatch + apply routing (engine.rs)**

Change `poly_scratch` to two tiles:

```rust
            let mut poly_scratch = [[[0.0f32; BLOCK]; VOICES]; 2];
```

Replace the `if Node::has_poly_in(kind) { ... }` resolution block with a loop over the poly ports:

```rust
                for j in 0..Node::poly_in_count(kind) {
                    // Ports 0..k are poly edges; copy each source's VOICES rows
                    // verbatim (row-copy preserves the interleaved layout).
                    match inputs[j] {
                        Input::Node { node, .. } => match self.arena.out_base(node) {
                            Some(sbase) if sbase + VOICES <= OUTS => {
                                for v in 0..VOICES { poly_scratch[j][v] = arr[sbase + v]; }
                            }
                            _ => { for v in 0..VOICES { poly_scratch[j][v] = [0.0; BLOCK]; } }
                        },
                        _ => { for v in 0..VOICES { poly_scratch[j][v] = [0.0; BLOCK]; } }
                    }
                }
```

Replace the poly dispatch branch:

```rust
            if Node::is_poly(kind) {
                let count = Node::poly_in_count(kind);
                let poly_in: [Option<&[f32]>; 2] = [
                    if count > 0 { Some(poly_scratch[0].as_flattened()) } else { None },
                    if count > 1 { Some(poly_scratch[1].as_flattened()) } else { None },
                ];
                let out = arr[base..base + width].as_flattened_mut();
                if let Some(n) = self.arena.node_mut(id) {
                    n.poly_process(&ins, poly_in, self.dt, out);
                }
            } else {
                // ... existing mono/stereo process_resolved path unchanged ...
```

In `apply()`, after the `Cmd::Trigger` arm add:

```rust
            Cmd::GateVoice { node, voice, on } => {
                if let Some(n) = self.arena.node_mut(node) { n.gate_voice(voice as usize, on); }
            }
            Cmd::TriggerVoice { node, voice } => {
                if let Some(n) = self.arena.node_mut(node) { n.trigger_voice(voice as usize); }
            }
```

- [ ] **Step 7: Node-level engine tests (engine.rs tests module)**

Add:

```rust
    #[test]
    fn poly_mul_node_multiplies_two_poly_sources() {
        // Two PolyCtrl sources → PolyMul → VoiceSum. Sum == Σ_v (a_v * b_v).
        type PE = Engine<64, 8, 40, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(0), Kind::PolyCtrl);
        e.create(NodeId(1), Kind::PolyCtrl);
        for v in 0..VOICES {
            e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v + 1) as f32 }); // 1..=8
            e.apply(Cmd::SetParam { node: NodeId(1), param: v as u8, value: 2.0 });
        }
        e.create(NodeId(2), Kind::PolyMul);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        *e.node_input_mut(NodeId(2), 1).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        e.create(NodeId(3), Kind::VoiceSum);
        *e.node_input_mut(NodeId(3), 0).unwrap() = Input::Node { node: NodeId(2), port: 0 };
        e.render_block();
        let out = e.node_output(NodeId(3), 0);
        let want: f32 = (1..=VOICES).map(|x| x as f32 * 2.0).sum(); // Σ 2·(1..8) = 72
        assert!(out.iter().all(|&s| (s - want).abs() < 1e-3), "sum of a·b == {want}");
    }

    #[test]
    fn gate_voice_reaches_only_addressed_lane() {
        // A PolyAr gated on voice 3 only → after some samples, VoiceSum > 0 comes
        // solely from lane 3 (all others idle at 0).
        type PE = Engine<64, 8, 40, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(0), Kind::PolyAr);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.0005); // fast attack
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.05);
        e.create(NodeId(1), Kind::VoiceSum);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.apply(Cmd::GateVoice { node: NodeId(0), voice: 3, on: true });
        e.render_block();
        let out = e.node_output(NodeId(1), 0);
        // Exactly one voice ramping ⇒ sum rises toward ~1 (not 0, not ~8).
        let peak = out.iter().cloned().fold(0.0f32, |m, s| m.max(s));
        assert!(peak > 0.1 && peak < 1.5, "one gated voice: peak {peak}");
    }
```

- [ ] **Step 8: Run the tests (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: PASS — full suite incl. the two new tests + updated Sy-1 tests. No warnings.
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features deluge-dsp-kernels/simd`
Expected: PASS (exercises the `f32x8` PolySvf through the graph).

- [ ] **Step 9: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs crates/deluge-audio-graph/src/engine.rs crates/deluge-audio-graph/src/cmd.rs
git commit -m "feat(audio-graph): PolyAr/PolySvf/PolyMul nodes + two-poly-input + per-voice gate"
```

---

### Task 4: Full-voice integration test (both feature configs)

**Files:**
- Modify: `crates/deluge-audio-graph/src/engine.rs` (tests module)

**Interfaces:**
- Consumes: everything from Tasks 1–3 (`Kind::PolyCtrl/PolyOsc/PolySvf/PolyMul/PolyAr/VoiceSum`, `Cmd::GateVoice`, `poly_in_count`, engine two-poly resolution).

- [ ] **Step 1: Write the headline gated-voice test**

Graph: `PolyCtrl(pitch) → PolyOsc → PolySvf(cutoff) → PolyMul(·, PolyAr) → VoiceSum`.

```rust
    #[test]
    fn full_voice_gated_per_voice() {
        // A complete voice: gate voices 0 & 1 on, 2..7 off. The mix is non-silent
        // and comes only from the gated voices; with no gates it is silent; the
        // filter keeps the output bounded. Runs in both feature configs.
        type PE = Engine<64, 8, 48, 4, 45056, 2048>;
        let build = |gates: &[usize]| -> [f32; 64] {
            let mut e = PE::new(48_000.0);
            // pitch source
            e.create(NodeId(0), Kind::PolyCtrl);
            for v in 0..VOICES {
                e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v as f32 + 1.0) * 110.0 });
            }
            // osc → filter
            e.create(NodeId(1), Kind::PolyOsc);
            *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
            e.create(NodeId(2), Kind::PolySvf);
            *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
            *e.node_input_mut(NodeId(2), 1).unwrap() = Input::Const(1200.0); // cutoff
            *e.node_input_mut(NodeId(2), 2).unwrap() = Input::Const(0.2);    // res
            // envelope + VCA
            e.create(NodeId(3), Kind::PolyAr);
            *e.node_input_mut(NodeId(3), 0).unwrap() = Input::Const(0.0005); // attack
            *e.node_input_mut(NodeId(3), 1).unwrap() = Input::Const(0.05);   // release
            e.create(NodeId(4), Kind::PolyMul);
            *e.node_input_mut(NodeId(4), 0).unwrap() = Input::Node { node: NodeId(2), port: 0 };
            *e.node_input_mut(NodeId(4), 1).unwrap() = Input::Node { node: NodeId(3), port: 0 };
            // sum → out
            e.create(NodeId(5), Kind::VoiceSum);
            *e.node_input_mut(NodeId(5), 0).unwrap() = Input::Node { node: NodeId(4), port: 0 };
            for &g in gates {
                e.apply(Cmd::GateVoice { node: NodeId(3), voice: g as u8, on: true });
            }
            e.render_block();
            let mono = e.node_output(NodeId(5), 0);
            let mut out = [0.0f32; 64];
            out.copy_from_slice(&mono[..64]);
            out
        };

        // No gates → silence.
        let silent = build(&[]);
        assert!(silent.iter().all(|&s| s.abs() < 1e-6), "no gates → silent");

        // Gate voices 0 & 1 → non-silent, bounded/finite.
        let voiced = build(&[0, 1]);
        assert!(voiced.iter().all(|&s| s.is_finite() && s.abs() <= 8.0), "bounded");
        assert!(voiced.iter().any(|&s| s.abs() > 1e-4), "gated voices sound");
    }
```

- [ ] **Step 2: Run both feature configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph full_voice_gated_per_voice`
Expected: PASS (scalar).
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features deluge-dsp-kernels/simd full_voice_gated_per_voice`
Expected: PASS (f32x8 PolySvf). No warnings.

- [ ] **Step 3: Commit**

```bash
git add crates/deluge-audio-graph/src/engine.rs
git commit -m "test(audio-graph): full per-voice-gated voice end-to-end"
```

---

## Notes for the implementer

- `Ar` and `Svf` derive `Copy`, so `[Ar::new(); VOICES]` / `[Svf::new(); VOICES]` compile.
- The `f32x8` PolySvf uses `core::simd::prelude::*` (same as `fast_sin_x8`); coeffs are scalar (shared mono cutoff) then splat, so only the recurrence vectorizes — and the scalar/SIMD paths use the *same* `poly_svf_coeffs`, so they agree.
- The two-poly-input engine change is atomic with the `poly_process` signature and its caller — that's why Task 3 lands node.rs + engine.rs + cmd.rs together (splitting them would leave the build non-compiling between tasks).
- If an anchor moved, grep the named symbol (`has_poly_in`, the `poly_scratch` decl, the `Cmd::Trigger` arm) — the relationship matters, not line numbers.
- `poly_scratch[j].as_flattened()` for two `j` are two immutable borrows of distinct array elements — they coexist fine, and with the mutable `arr`/`out` (distinct object) per the Sy-1 borrow discipline.
