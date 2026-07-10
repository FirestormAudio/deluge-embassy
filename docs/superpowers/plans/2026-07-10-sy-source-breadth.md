# Sy-2d: Source Breadth Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make poly hard-sync, poly wavetable (single + 2D morph), per-voice PWM, and pink/brown poly noise work inside a Wren `Synth`, plus a mono→poly broadcast so mono signals can drive poly input ports.

**Architecture:** One foundational engine change (mono→poly broadcast), then four source kernels through kernel → graph → Wren, each following the locked recipe (scalar oracle; f32x8 only where it's a hot band-limited path; scalar-per-voice where per-voice divergence defeats SIMD).

**Tech Stack:** Rust `no_std`, `core::simd` f32x8 (behind `simd`), Wren scripting via `deluge-wren-core`.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP paths. ARM Cortex-A9 (VFPv3+NEON); host x86 for tests only. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **SIMD convention:** scalar path is the correctness oracle; `#[cfg(feature="simd")]` f32x8 null-tested lane-for-lane to `≤ 1e-4`. Doc comments attach to the struct, never a cfg'd const guard. The `#[cfg(feature="simd")] const _: () = assert!(VOICES==8)` guard already exists in poly.rs — do not add a duplicate.
- **Poly control convention:** leading ports are poly edges (`poly_in_count` = their count); trailing controls are shared-mono `In`, splatted per sample.
- **Test invocation (per-crate, never `--workspace`; cargo rejects multiple bare positional names — use `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` and again with `--features simd`.
  - Graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph <name>` and again `--features deluge-dsp-kernels/simd`.
  - Wren: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core <name>` and again `--features deluge-dsp-kernels/simd`.
  - An LSP `armv7a … can't find crate for test` error is environmental noise — ignore it.

## File Structure

- `crates/deluge-audio-graph/src/engine.rs` — mono→poly broadcast in poly-input resolution (~:203-215); thread a resolved table region into `poly_process` for poly wavetable.
- `crates/deluge-dsp-kernels/src/noise.rs` — extract `Noise::tick`.
- `crates/deluge-dsp-kernels/src/osc.rs` — `wave_sample_x8` gains `width`; add `naive_wave_x8`; extract `SyncOsc::tick`.
- `crates/deluge-dsp-kernels/src/poly.rs` — `PolyNoise` gains color; `PolyOsc` gains width port; new `PolySync`, `PolyWt`.
- `crates/deluge-audio-graph/src/node.rs` — new poly Kinds/State/predicate/dispatch arms; PolyOsc `poly_in_count 1→2`.
- `crates/deluge-wren-core/src/{bindings_audio.rs,bindings.rs}`, `wren/prelude.wren`, `tests/audio_bindings.rs` — factories, routes, tests.

---

## Task 1: Mono→poly broadcast (engine foundation)

**Files:**
- Modify: `crates/deluge-audio-graph/src/engine.rs` (poly-input resolution, ~:203-215)
- Test: `engine.rs` test module

**Interfaces:**
- Consumes: `Node::out_base`, `Node::out_width` (already used in the render loop), `VOICES`, `BLOCK`.
- Produces: broadcast semantics — a poly input port fed by a source whose `out_width == 1` sees that single row on all `VOICES` lanes. Relied on by Task 3 (PWM) and Task 6 (Wren e2e).

- [ ] **Step 1: Write the failing test**

Add to the engine.rs test module a test that builds a mono producer (e.g. `Kind::Ctrl` set to a constant, or a mono `Kind::Sine`) wired into one poly-input port of a poly consumer, renders a block, and asserts all `VOICES` lanes of the consumer's input are equal. Mirror the existing engine poly test harness (`full_voice_gated_per_voice` construction). Concretely, assert that a mono constant `c` feeding a `PolyMul(mono, polySource)` broadcasts `c` to every lane (all 8 lanes multiplied by the same `c`).

```rust
#[test]
fn mono_source_broadcasts_to_all_poly_lanes() {
    // Build: PolyOsc(pitch) as poly source A; a mono Ctrl=0.5 as source B;
    // PolyMul(A, B). Because B is width-1, the broadcast must splat 0.5 to all
    // lanes, so PolyMul output lane v == A_lane_v * 0.5 for every v.
    // (Use the existing engine test harness; gate all voices; render one block.)
    // Assert: for each voice v, out[v] ≈ polyosc_lane_v * 0.5 (not 0 for v>0).
}
```

> Fill in with the concrete harness the existing engine tests use (Engine generic params, `bind`/wire calls, `Cmd::GateVoice`, `render_block`). The assertion that distinguishes broadcast from the old behavior: lanes 1..8 are non-zero (old code would read garbage/zero rows for the mono source).

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph mono_source_broadcasts`
Expected: FAIL (lanes 1..7 wrong — mono source not broadcast).

- [ ] **Step 3: Implement broadcast**

In the poly-input resolution loop (engine.rs:206-214), split the `Input::Node` arm on the source's `out_width`:

```rust
                    match inputs[j] {
                        Input::Node { node, .. } => match self.arena.out_base(node) {
                            Some(sbase) if Node::out_width(self.arena.kind_of(node)) == 1
                                && sbase < OUTS => {
                                // Mono producer → broadcast its single row to all lanes.
                                for v in 0..VOICES { poly_scratch[j][v] = arr[sbase]; }
                            }
                            Some(sbase) if sbase + VOICES <= OUTS => {
                                for v in 0..VOICES { poly_scratch[j][v] = arr[sbase + v]; }
                            }
                            _ => { for v in 0..VOICES { poly_scratch[j][v] = [0.0; BLOCK]; } }
                        },
                        _ => { for v in 0..VOICES { poly_scratch[j][v] = [0.0; BLOCK]; } }
                    }
```

> Use whatever accessor the arena exposes for a node's kind (if `kind_of` doesn't exist, read it via the same path the snapshot uses at engine.rs:176 — `n.kind`; you may need to capture the source kind alongside `out_base`). The key semantic: `out_width==1` → replicate `arr[sbase]` to all `VOICES` lanes.

- [ ] **Step 4: Run to verify it passes**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph mono_source_broadcasts` and again `--features deluge-dsp-kernels/simd`.
Expected: PASS both.

- [ ] **Step 5: Regression + commit**

Run the full graph crate both configs (`cargo test ... -p deluge-audio-graph` and `--features deluge-dsp-kernels/simd`) — no existing poly test regressed (poly sources feeding poly inputs still copy `VOICES` rows; only width-1 sources changed behavior).

```bash
git add crates/deluge-audio-graph/src/engine.rs
git commit -m "feat(engine): mono→poly broadcast — splat width-1 producers to all voice lanes"
```

---

## Task 2: Pink/brown poly noise

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/noise.rs` (extract `Noise::tick`)
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (`PolyNoise` gains color)
- Modify: `crates/deluge-audio-graph/src/node.rs` (Kinds/State/arms)
- Test: `poly.rs` + `node.rs` test modules

**Interfaces:**
- Consumes: `Noise`, `NoiseColor` (noise.rs), `VOICES`.
- Produces: `PolyNoise::new_color(color: NoiseColor)`; `PolyNoise::process(&mut self, out: &mut [f32])` (unchanged signature, now colored); graph `Kind::PolyPink`, `Kind::PolyBrown` (alongside existing `Kind::PolyNoise`). Used by Task 6.

- [ ] **Step 1: Extract `Noise::tick` (pure refactor)**

In noise.rs, factor the per-sample body of `process` (noise.rs:67-95) into:

```rust
    /// One noise sample (advances rng + color state). Shared by `process` and `PolyNoise`.
    #[inline]
    pub(crate) fn tick(&mut self) -> f32 {
        let mut r = self.rng;
        r ^= r << 13;
        r ^= r >> 17;
        r ^= r << 5;
        self.rng = r;
        let white = (r as i32 as f32) / (i32::MAX as f32);
        match self.color {
            NoiseColor::White => white,
            NoiseColor::Pink => {
                let b = &mut self.pink;
                b[0] = 0.99886 * b[0] + white * 0.0555179;
                b[1] = 0.99332 * b[1] + white * 0.0750759;
                b[2] = 0.96900 * b[2] + white * 0.153852;
                b[3] = 0.86650 * b[3] + white * 0.3104856;
                b[4] = 0.55000 * b[4] + white * 0.5329522;
                b[5] = -0.7616 * b[5] - white * 0.016898;
                let pink = (b[0]+b[1]+b[2]+b[3]+b[4]+b[5]+b[6] + white*0.5362) * PINK_GAIN;
                b[6] = white * 0.115926;
                pink
            }
            NoiseColor::Brown => {
                self.brown = (self.brown + white * BROWN_RATE) * BROWN_LEAK;
                (self.brown * BROWN_GAIN).clamp(-1.0, 1.0)
            }
        }
    }
```

Refactor `Noise::process` to `for s in out.iter_mut() { *s = self.tick(); }`.

- [ ] **Step 2: Verify the refactor is behavior-preserving**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels noise` (both configs). Existing `noise` tests PASS unchanged.

- [ ] **Step 3: Write the failing PolyNoise-color null-test**

In poly.rs test module:

```rust
#[test]
fn polynoise_colors_match_mono_per_lane() {
    use deluge_dsp_kernels::noise::{Noise, NoiseColor}; // match poly.rs test import convention
    let n = 256usize;
    for color in [NoiseColor::White, NoiseColor::Pink, NoiseColor::Brown] {
        let mut poly = PolyNoise::new_color(color);
        let mut out = std::vec![0.0f32; n * VOICES];
        poly.process(&mut out);
        // Each lane must equal a mono Noise seeded with that lane's seed + color.
        for v in 0..VOICES {
            let seed = 0x2545_F491u32 ^ 0x9E37_79B9u32.wrapping_mul((v as u32) + 1);
            let mut refn = Noise::seeded_color(seed, color);
            for i in 0..n {
                let want = refn.tick();
                assert_eq!(out[i * VOICES + v], want, "color {color:?} lane {v} sample {i}");
            }
        }
    }
}
```

> Use the EXACT per-voice seed formula the current Sy-2b `PolyNoise` uses (check poly.rs — it is `0x2545_F491 ^ 0x9E37_79B9.wrapping_mul(v+1)`). This keeps white bit-identical to today. Match the test module's `std::vec`/import conventions.

- [ ] **Step 4: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels polynoise_colors_match`
Expected: FAIL — `PolyNoise::new_color` not found.

- [ ] **Step 5: Reimplement `PolyNoise` over `[Noise; VOICES]`**

Replace the `PolyNoise { rng: [u32; VOICES] }` struct with per-voice `Noise` instances (each voice IS a mono colored generator — the cleanest scalar oracle; noise is serial-recurrence, no f32x8):

```rust
/// Poly noise: 8 independent colored generators (white/pink/brown), each a mono
/// `Noise` with a decorrelated seed. Scalar (serial IIR recurrences). White lanes
/// remain bit-identical to the Sy-2b `PolyNoise`.
#[derive(Clone, Copy)]
pub struct PolyNoise {
    voices: [Noise; VOICES],
}
impl PolyNoise {
    pub fn new() -> PolyNoise { PolyNoise::new_color(NoiseColor::White) }
    pub fn new_color(color: NoiseColor) -> PolyNoise {
        let voices = core::array::from_fn(|v| {
            let seed = 0x2545_F491u32 ^ 0x9E37_79B9u32.wrapping_mul((v as u32) + 1);
            Noise::seeded_color(seed, color)
        });
        PolyNoise { voices }
    }
    /// Voice-interleaved output tile, length `VOICES * n_samples`.
    pub fn process(&mut self, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            for v in 0..VOICES {
                out[i * VOICES + v] = self.voices[v].tick();
            }
        }
    }
}
impl Default for PolyNoise { fn default() -> Self { Self::new() } }
```

> Add `use crate::noise::{Noise, NoiseColor};` (or `deluge_dsp_kernels::noise::…`) to poly.rs's imports as the file convention dictates. If the old `PolyNoise` had a different `process` loop shape, keep the voice-interleaved `out[i*VOICES+v]` layout (matches all other poly kernels).

- [ ] **Step 6: Run to verify it passes**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels polynoise` (both configs). Expected: PASS. Also re-run any existing white PolyNoise test — white output must be unchanged.

- [ ] **Step 7: Graph Kinds**

In node.rs: add `Kind::PolyPink`, `Kind::PolyBrown` (alongside `Kind::PolyNoise`). Constructor arms:

```rust
            Kind::PolyNoise => State::PolyNoise(PolyNoise::new()),
            Kind::PolyPink => State::PolyNoise(PolyNoise::new_color(NoiseColor::Pink)),
            Kind::PolyBrown => State::PolyNoise(PolyNoise::new_color(NoiseColor::Brown)),
```

Add all three to `out_width` (VOICES), `is_poly`, and `poly_in_count => 0` (pure sources); add to the `process_resolved` no-op poly arm; and a `poly_process` arm:

```rust
            Kind::PolyNoise | Kind::PolyPink | Kind::PolyBrown => {
                if let State::PolyNoise(nz) = &mut self.state { nz.process(out); }
            }
```

Import `NoiseColor` into node.rs. Add a graph render test asserting a `PolyPink`/`PolyBrown` node renders finite, bounded, non-silent (mirror the existing PolyNoise graph test if present, else the PolySvf voice-render harness).

- [ ] **Step 8: Run + commit**

Run graph tests both configs (`polynoise`/`polypink`/`polybrown` and the render test). Expected: PASS.

```bash
git add crates/deluge-dsp-kernels/src/noise.rs crates/deluge-dsp-kernels/src/poly.rs crates/deluge-audio-graph/src/node.rs
git commit -m "feat(kernels+graph): pink/brown poly noise ([Noise;VOICES] via Noise::tick), white bit-unchanged"
```

---

## Task 3: Per-voice PWM

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/osc.rs` (`wave_sample_x8` gains `width`)
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (`PolyOsc` width port)
- Modify: `crates/deluge-audio-graph/src/node.rs` (PolyOsc `poly_in_count 1→2`, dispatch)
- Test: `poly.rs` test module

**Interfaces:**
- Consumes: `wave_sample`/`wave_sample_x8`, `Wave`, `VOICES`.
- Produces: `wave_sample_x8(wave, ph, dtp, width: f32x8)` (arity +1); `PolyOsc::process(&mut self, pitch: &[f32], width: &[f32], dt: f32, out: &mut [f32])` (arity +1). Used by Task 6.

- [ ] **Step 1: Add `width` to `wave_sample_x8` (with the backward-compat null-test)**

First write a failing test asserting `wave_sample_x8(Square, ph, dtp, width=0)` equals the scalar `wave_sample(Square, ph, dtp, 0.0)` (→ 0.5 duty) lane-for-lane, AND that a swept width matches scalar:

```rust
#[cfg(feature = "simd")]
#[test]
fn wave_sample_x8_pwm_matches_scalar() {
    use core::simd::prelude::*;
    let dtp = 0.01f32;
    for &w in &[0.0f32, 0.1, 0.25, 0.5, 0.75, 0.99, 1.5] {
        for &ph in &[0.0f32, 0.1, 0.49, 0.5, 0.51, 0.9] {
            let got = wave_sample_x8(Wave::Square, f32x8::splat(ph), f32x8::splat(dtp), f32x8::splat(w)).to_array();
            let want = wave_sample(Wave::Square, ph, dtp, w);
            for lane in got { assert!((lane - want).abs() <= 1e-4, "w={w} ph={ph}: {lane} vs {want}"); }
        }
    }
}
```

Run it (fails: arity mismatch). Then update `wave_sample_x8` (osc.rs:157) to take `width: f32x8` and use it in the Square arm, replicating the scalar `<=0 ⇒ 0.5` + clamp `[0.01,0.99]`:

```rust
pub(crate) fn wave_sample_x8(wave: Wave, ph: f32x8, dtp: f32x8, width: f32x8) -> f32x8 {
    let one = f32x8::splat(1.0);
    match wave {
        Wave::Sine => crate::fast_sin_x8(ph),
        Wave::Saw => (f32x8::splat(2.0) * ph - one) - poly_blep_x8(ph, dtp),
        Wave::Square => {
            // width <= 0 ⇒ 0.5, else clamp [0.01, 0.99] (matches scalar wave_sample).
            let half = f32x8::splat(0.5);
            let w = width.simd_le(f32x8::splat(0.0)).select(half, width)
                .simd_max(f32x8::splat(0.01)).simd_min(f32x8::splat(0.99));
            let naive = ph.simd_lt(w).select(one, -one);
            let mut pw = ph - w;
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

Update the existing `wave_sample_x8_matches_scalar` test (osc.rs:343) to pass a width arg (e.g. `f32x8::splat(0.5)`) so it still compiles. Run — the PWM test + the existing test pass.

- [ ] **Step 2: Add the width tile to `PolyOsc::process`**

Write a failing test: (a) backward-compat — width tile all-zeros ⇒ output bit-identical to a reference computed with the scalar `wave_sample(shape, p, dtp, 0.0)` for all four shapes; (b) swept per-voice width on Square ⇒ each lane matches scalar `wave_sample(Square, p, dtp, width_v)`.

```rust
#[test]
fn polyosc_pwm_backcompat_and_per_voice() {
    let dt = 1.0/48_000.0; let n = 128;
    // (a) width all-zero ⇒ 0.5 duty, identical for every shape
    for code in 0u8..4 {
        let mut o = PolyOsc::new(); o.set_shape(code);
        let pitch = std::vec![220.0f32; n*VOICES];
        let width = std::vec![0.0f32; n*VOICES];
        let mut out = std::vec![0.0f32; n*VOICES];
        o.process(&pitch, &width, dt, &mut out);
        // reference: a scalar mono re-derivation with width 0.0 → 0.5
        // (assert lane 0 equals a hand-rolled scalar wave_sample loop with width 0.0)
        // ... see note; the key gate is that supplying width=0 reproduces pre-Sy-2d output.
    }
    // (b) per-voice width: lanes with distinct widths differ
    let mut o = PolyOsc::new(); o.set_shape(2); // Square
    let pitch = std::vec![220.0f32; n*VOICES];
    let width: std::vec::Vec<f32> = (0..n*VOICES).map(|j| 0.1 + 0.1*((j%VOICES) as f32)).collect();
    let mut out = std::vec![0.0f32; n*VOICES];
    o.process(&pitch, &width, dt, &mut out);
    // assert two lanes with different widths produce different output at some sample
    assert!((0..n).any(|i| out[i*VOICES+1] != out[i*VOICES+7]));
}
```

> For the strongest backward-compat gate, compute the reference by running the CURRENT PolyOsc math inline (scalar `wave_sample(shape, p, dtp, 0.5)`) and assert equality — i.e. the new width-tile path with width=0 must equal the old hardcoded-0.5 path. Adjust to the test module's conventions.

Run — fails (arity). Then update `PolyOsc::process` to take `width: &[f32]`:
- SIMD branch: `let w = f32x8::from_slice(&width[i*VOICES..]);` then `wave_sample_x8(self.shape, p, dtp, w)`.
- Scalar branch: replace the hardcoded `0.5` with `width[i*VOICES+v]` in `wave_sample(self.shape, p, dtp, width[i*VOICES+v])`.

Run — passes both configs. The all-zero-width gate proves existing voices are bit-unchanged (the `<=0 ⇒ 0.5` convention).

- [ ] **Step 3: Graph — PolyOsc gains the width port**

In node.rs: change `poly_in_count(Kind::PolyOsc)` from the `=> 1` arm to a `=> 2` arm (pitch = port 0, width = port 1). Update the `poly_process` PolyOsc arm:

```rust
            Kind::PolyOsc => {
                if let (State::PolyOsc(o), Some(pitch)) = (&mut self.state, poly_in[0]) {
                    // width port optional: unconnected ⇒ Const(0.0) broadcast ⇒ 0.5 duty.
                    let width = poly_in[1].unwrap_or(pitch); // placeholder; see note
                    o.process(pitch, width, dt, out);
                }
            }
```

> IMPORTANT width-default semantics: with `poly_in_count == 2`, port 1 is always resolved by the engine — an unconnected port 1 is `Input::Const(0.0)`, which the engine's poly-input resolution turns into an all-zero tile (or, via Task 1 broadcast, a width-1 zero source → all-zero lanes). So `poly_in[1]` will be `Some(all-zeros)` when unconnected, NOT `None`. Do NOT fall back to `pitch`. Use `poly_in[1]` directly; if the engine ever passes `None` (count mismatch), pass a zero tile. Verify against the engine: `poly_in[j]` is `Some` for `j < poly_in_count`, so with count 2 `poly_in[1]` is always `Some`. Write:
> ```rust
>             Kind::PolyOsc => {
>                 if let (State::PolyOsc(o), Some(pitch), Some(width)) =
>                     (&mut self.state, poly_in[0], poly_in[1]) {
>                     o.process(pitch, width, dt, out);
>                 }
>             }
> ```

Add a graph render test: a PolyOsc Square voice with an unconnected width port renders (0.5 duty), and with a mono width source (via Task 1 broadcast) renders a shifted duty — both finite/bounded/non-silent.

- [ ] **Step 4: Run + commit**

Run kernels + graph both configs (`wave_sample_x8_pwm`, `polyosc_pwm`, the graph render test). Expected: PASS. Confirm no existing PolyOsc graph/render test regressed (backward-compat).

```bash
git add crates/deluge-dsp-kernels/src/osc.rs crates/deluge-dsp-kernels/src/poly.rs crates/deluge-audio-graph/src/node.rs
git commit -m "feat(kernels+graph): per-voice PWM — PolyOsc width port + wave_sample_x8 width (0.5 default bit-unchanged)"
```

---

## Task 4: Poly hard sync

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/osc.rs` (extract `SyncOsc::tick`, add `naive_wave_x8`)
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (`PolySync`)
- Modify: `crates/deluge-audio-graph/src/node.rs` (Kinds/State/arms)
- Test: `poly.rs` + `node.rs` test modules

**Interfaces:**
- Consumes: `SyncOsc`, `wave_sample`/`wave_sample_x8`, `poly_blep`/`poly_blep_x8`, `naive_wave`, `floor_x8`, `Wave`, `VOICES`.
- Produces: `SyncOsc::tick(&mut self, wave: Wave, master_freq: f32, slave_freq: f32, dt: f32) -> f32`; `naive_wave_x8(wave, ph: f32x8) -> f32x8`; `PolySync::new()`, `PolySync::process(&mut self, master: &[f32], slave: &[f32], wave: Wave, dt: f32, out: &mut [f32])`. Used by Task 6.

- [ ] **Step 1: Extract `SyncOsc::tick` (pure refactor)**

Factor the per-sample loop body of `SyncOsc::process` (osc.rs:279-305) into a `tick`:

```rust
    /// One hard-sync sample: advance master, hard-reset slave on master wrap with
    /// a reset-BLEP. Shared by mono `process` and poly `PolySync`. (Same math as
    /// the current loop body — behavior-preserving.)
    #[inline]
    pub(crate) fn tick(&mut self, wave: Wave, master_freq: f32, slave_freq: f32, dt: f32) -> f32 {
        let dtp_m = master_freq * dt;
        let dtp_s = slave_freq * dt;
        let mut y = wave_sample(wave, self.slave_phase, dtp_s, 0.5);
        let mp_before = self.master_phase;
        let mp = self.master_phase + dtp_m;
        if mp >= 1.0 && dtp_m > 0.0 {
            let t_reset = (1.0 - mp_before) / dtp_m;
            let ph_at_reset = { let mut p = self.slave_phase + t_reset * dtp_s; p -= floorf(p); p };
            let step = naive_wave(wave, 0.0) - naive_wave(wave, ph_at_reset);
            y += 0.5 * step * poly_blep(mp_before, dtp_m);
            self.master_phase = mp - floorf(mp);
            self.slave_phase = (1.0 - t_reset) * dtp_s;
            self.slave_phase -= floorf(self.slave_phase);
        } else {
            self.master_phase = mp - floorf(mp);
            self.slave_phase += dtp_s;
            self.slave_phase -= floorf(self.slave_phase);
        }
        y
    }
```

Refactor `SyncOsc::process` to loop `*s = self.tick(wave, master_freq.at(i), slave_freq.at(i), dt);`.

Run `cargo test ... -p deluge-dsp-kernels sync` (both configs) — existing `sync_*` tests PASS unchanged.

- [ ] **Step 2: Add `naive_wave_x8`**

Beside `naive_wave` (osc.rs:181), add the f32x8 counterpart:

```rust
#[cfg(feature = "simd")]
#[inline]
fn naive_wave_x8(wave: Wave, ph: f32x8) -> f32x8 {
    let one = f32x8::splat(1.0);
    match wave {
        Wave::Sine => crate::fast_sin_x8(ph),
        Wave::Saw => f32x8::splat(2.0) * ph - one,
        Wave::Square => ph.simd_lt(f32x8::splat(0.5)).select(one, -one),
        Wave::Tri => one - f32x8::splat(4.0) * (ph - f32x8::splat(0.5)).abs(),
    }
}
```

- [ ] **Step 3: Write the failing PolySync null-test**

```rust
#[test]
fn polysync_matches_scalar_oracle_all_waves() {
    let dt = 1.0/48_000.0; let n = 300;
    for (code, wave) in [(0u8,Wave::Sine),(1,Wave::Saw),(2,Wave::Square),(3,Wave::Tri)] {
        let mut poly = PolySync::new();
        let mut refs: [SyncOsc; VOICES] = core::array::from_fn(|_| SyncOsc::new());
        // per-voice distinct master & slave freqs
        let master: std::vec::Vec<f32> = (0..n*VOICES).map(|j| 110.0 + 20.0*((j%VOICES) as f32)).collect();
        let slave:  std::vec::Vec<f32> = (0..n*VOICES).map(|j| 165.0 + 30.0*((j%VOICES) as f32)).collect();
        let mut out = std::vec![0.0f32; n*VOICES];
        poly.process(&master, &slave, wave, dt, &mut out);
        for v in 0..VOICES {
            for i in 0..n {
                let want = refs[v].tick(wave, master[i*VOICES+v], slave[i*VOICES+v], dt);
                assert!((out[i*VOICES+v]-want).abs() <= 1e-4, "{wave:?} lane {v} i {i}: {} vs {}", out[i*VOICES+v], want);
            }
        }
        let _ = code;
    }
}
```

Run — fails (`PolySync` not found).

- [ ] **Step 4: Implement `PolySync`**

```rust
/// Poly hard-sync oscillator. Two poly frequency inputs (master, slave); band-
/// limited (natural-wrap BLEP + reset-BLEP). Scalar path holds `[SyncOsc; VOICES]`
/// and reuses `SyncOsc::tick`; the SIMD path runs the sync sample across `f32x8`
/// lanes branchlessly (master-wrap mask + select).
#[derive(Clone, Copy)]
pub struct PolySync {
    #[cfg(not(feature = "simd"))]
    voices: [SyncOsc; VOICES],
    #[cfg(feature = "simd")]
    master_phase: core::simd::f32x8,
    #[cfg(feature = "simd")]
    slave_phase: core::simd::f32x8,
}
impl PolySync {
    #[cfg(not(feature = "simd"))]
    pub fn new() -> Self { PolySync { voices: [SyncOsc::new(); VOICES] } }
    #[cfg(feature = "simd")]
    pub fn new() -> Self { PolySync { master_phase: core::simd::f32x8::splat(0.0), slave_phase: core::simd::f32x8::splat(0.0) } }

    pub fn process(&mut self, master: &[f32], slave: &[f32], wave: Wave, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            use crate::osc::{wave_sample_x8, poly_blep_x8, floor_x8, naive_wave_x8};
            let dtv = f32x8::splat(dt);
            let zero = f32x8::splat(0.0);
            let one = f32x8::splat(1.0);
            let half = f32x8::splat(0.5);
            let width = half; // sync slave PWM fixed 0.5, matching scalar tick
            let mut mp = self.master_phase;
            let mut sp = self.slave_phase;
            for i in 0..n {
                let dtp_m = f32x8::from_slice(&master[i*VOICES..]) * dtv;
                let dtp_s = f32x8::from_slice(&slave[i*VOICES..]) * dtv;
                // slave natural-wrap value
                let mut y = wave_sample_x8(wave, sp, dtp_s, width);
                let mp_before = mp;
                let mp_adv = mp + dtp_m;
                // reset mask: wrapped AND advancing
                let reset = mp_adv.simd_ge(one) & dtp_m.simd_gt(zero);
                // reset-path quantities (safe to compute for all lanes)
                let t_reset = (one - mp_before) / dtp_m; // dtp_m>0 where reset true
                let mut ph_at_reset = sp + t_reset * dtp_s; ph_at_reset -= floor_x8(ph_at_reset);
                let step = naive_wave_x8(wave, zero) - naive_wave_x8(wave, ph_at_reset);
                let y_reset = y + half * step * poly_blep_x8(mp_before, dtp_m);
                let mut mp_wrapped = mp_adv - floor_x8(mp_adv);
                let mut sp_reset = (one - t_reset) * dtp_s; sp_reset -= floor_x8(sp_reset);
                // no-reset path
                let mp_cont = mp_adv - floor_x8(mp_adv);
                let mut sp_cont = sp + dtp_s; sp_cont -= floor_x8(sp_cont);
                // select
                y = reset.select(y_reset, y);
                mp = reset.select(mp_wrapped, mp_cont);
                sp = reset.select(sp_reset, sp_cont);
                let _ = &mut mp_wrapped; let _ = &mut sp_reset;
                y.copy_to_slice(&mut out[i*VOICES..]);
            }
            self.master_phase = mp; self.slave_phase = sp;
        }
        #[cfg(not(feature = "simd"))]
        {
            for i in 0..n {
                for v in 0..VOICES {
                    out[i*VOICES+v] = self.voices[v].tick(wave, master[i*VOICES+v], slave[i*VOICES+v], dt);
                }
            }
        }
    }
}
impl Default for PolySync { fn default() -> Self { Self::new() } }
```

> Notes: (1) `mp_cont` and `mp_wrapped` are the same expression (`mp_adv - floor(mp_adv)`) — the scalar `else` branch also does `mp - floorf(mp)`, so a single `mp_next = mp_adv - floor_x8(mp_adv)` suffices for both; simplify to avoid the dead bindings. (2) `t_reset = (1-mp_before)/dtp_m` divides by `dtp_m` which is 0 on non-reset lanes → produces inf/nan there, but those lanes are masked away by `select`; if the target's nan handling in the unused arithmetic is a concern, guard with `dtp_m.simd_max(f32x8::splat(f32::MIN_POSITIVE))` before dividing (the reset lanes have dtp_m>0 so the guard doesn't change them). Prefer the guard — it keeps all lanes finite. (3) `width` fixed 0.5 matches the scalar `tick`'s `wave_sample(..., 0.5)`. Import `SyncOsc` into poly.rs.

- [ ] **Step 5: Run to verify (both configs) + band-limit gate**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels polysync` and `--features simd polysync`. Expected: PASS.

Add a band-limiting gate mirroring the mono `sync_saw_is_band_limited` test but on a poly lane: render a poly-sync saw on one lane (others silent), FFT/alias-measure exactly as the mono test does, assert the same ~-30 dB alias floor. (Reuse the mono test's alias-measurement helper.)

- [ ] **Step 6: Graph Kinds + render test**

node.rs: add `Kind::PolySyncSine/Saw/Square/Tri` + `State::PolySync(PolySync)`. Constructor: all four → `State::PolySync(PolySync::new())`. Predicates: `out_width VOICES`, `is_poly`, `poly_in_count => 2` (master + slave). `process_resolved` no-op arm. `poly_process` arm:

```rust
            Kind::PolySyncSine | Kind::PolySyncSaw | Kind::PolySyncSquare | Kind::PolySyncTri => {
                let wave = match self.kind {
                    Kind::PolySyncSine => Wave::Sine,
                    Kind::PolySyncSaw => Wave::Saw,
                    Kind::PolySyncSquare => Wave::Square,
                    _ => Wave::Tri,
                };
                if let (State::PolySync(s), Some(master), Some(slave)) =
                    (&mut self.state, poly_in[0], poly_in[1]) {
                    s.process(master, slave, wave, dt, out);
                }
            }
```

> Read `self.kind` into a local before borrowing `self.state` if the borrow checker complains (mirror the mono `SyncSine|...` arm). Add a graph render test: a poly sync voice (master+slave from two poly pitch sources) renders finite/bounded/non-silent for each wave.

- [ ] **Step 7: Run + commit**

Run kernels + graph both configs. Expected: PASS.

```bash
git add crates/deluge-dsp-kernels/src/osc.rs crates/deluge-dsp-kernels/src/poly.rs crates/deluge-audio-graph/src/node.rs
git commit -m "feat(kernels+graph): PolySync hard-sync (scalar tick oracle + f32x8 branchless reset-BLEP)"
```

---

## Task 5: Poly wavetable (single-cycle + 2D morph)

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (`PolyWt`)
- Modify: `crates/deluge-audio-graph/src/node.rs` (Kinds/State/arms; thread table region into poly dispatch)
- Modify: `crates/deluge-audio-graph/src/engine.rs` (pass resolved pool_region into `poly_process` for poly wavetable)
- Test: `poly.rs` + `node.rs` test modules

**Interfaces:**
- Consumes: `WtOsc`, `WtOsc::process(mips: MipSet, freq, pmod, dt, out)`, `WtOsc::process_morph(region, frames, freq, pmod, position, dt, out)`, `MipSet`, `compact_levels`, `TableSrc`, `static_table_flat`, `COMPACT_LEN`, `In`, `VOICES`.
- Produces: `PolyWt::new()`; `PolyWt::process_voice(&mut self, v: usize, mips: MipSet, freq: In, pmod: In, dt: f32, out: &mut [f32])` and `PolyWt::process_voice_morph(&mut self, v: usize, region: &[f32], frames: usize, freq: In, pmod: In, position: In, dt: f32, out: &mut [f32])` — process ONE voice into a mono block; the interleave gather/scatter loop lives in the graph layer (which owns `BLOCK`-sized scratch). Used by Task 6.

- [ ] **Step 1: Write the failing PolyWt null-test (graph level)**

`PolyWt` reuses the mono `WtOsc` per voice, so a poly-node lane must be bit-identical to a mono `Kind::Wavetable` node fed that lane's pitch. Place the test in the GRAPH crate (`node.rs` tests), where `compact_levels`/`static_table_flat` and `Kind::Wavetable` exist:

```rust
#[test]
fn polywt_single_matches_mono_per_voice() {
    // Static table 0 (saw). Build a PolyWt-backed poly node with distinct per-voice
    // pitch; build 8 mono Kind::Wavetable nodes, one per pitch. Render both; assert
    // poly lane v == mono node v output, bit-identical (same WtOsc kernel).
}
```

Run — fails (`Kind::PolyWt` / `PolyWt` not found).

- [ ] **Step 2: Implement `PolyWt` (per-voice, scalar)**

`PolyWt` is a thin holder of `[WtOsc; VOICES]` that processes ONE voice at a time into a mono block. This keeps the block-sized scratch (and the `BLOCK` const) in the graph layer where they already exist, and keeps `pmod`/`position` audio-rate (passed straight through as `In`).

```rust
/// Poly wavetable oscillator: 8 voices share one borrowed mip pyramid; each keeps
/// its own phase. Scalar-per-voice (per-voice pitch ⇒ per-voice mip level, which
/// defeats f32x8 without a table-row gather the A9 lacks). Reuses the audited
/// `WtOsc` per voice, for both single-cycle and 2D morph. The graph layer owns the
/// voice-interleave gather/scatter and the block scratch; this kernel processes one
/// voice's mono block per call so `freq`/`pmod`/`position` stay audio-rate `In`.
#[derive(Clone, Copy)]
pub struct PolyWt {
    voices: [WtOsc; VOICES],
}
impl PolyWt {
    pub fn new() -> Self { PolyWt { voices: [WtOsc::new(); VOICES] } }

    /// Single-cycle: process voice `v` into a mono `out` block, sharing `mips`.
    pub fn process_voice(&mut self, v: usize, mips: MipSet, freq: In, pmod: In, dt: f32, out: &mut [f32]) {
        self.voices[v].process(mips, freq, pmod, dt, out);
    }

    /// 2D morph: process voice `v` into a mono `out` block.
    pub fn process_voice_morph(&mut self, v: usize, region: &[f32], frames: usize, freq: In, pmod: In, position: In, dt: f32, out: &mut [f32]) {
        self.voices[v].process_morph(region, frames, freq, pmod, position, dt, out);
    }
}
impl Default for PolyWt { fn default() -> Self { Self::new() } }
```

> Add `use crate::wavetable::{WtOsc, MipSet};` (or the `deluge_dsp_kernels::wavetable::…` path) to poly.rs's imports per the file convention. Confirm `In` is already imported in poly.rs (it is — other poly kernels use it).

- [ ] **Step 3: Graph — the per-voice gather/scatter loop**

The graph layer (node.rs `poly_process`, which has `BLOCK`) drives the voices: gather each voice's pitch column from the interleaved `poly_in[0]` tile into a `[f32; BLOCK]` scratch, call `process_voice`, scatter the mono result back into the interleaved `out`. `pmod`/`position` are trailing mono control ports (`ins[…]`), passed straight through as `In` (shared across voices). This arm needs the resolved table `region`/`frames`/`mips` from Step 4:

```rust
            Kind::PolyWt | Kind::PolyWtMorph => {
                if let (State::PolyWt(w), Some(pitch)) = (&mut self.state, poly_in[0]) {
                    let n = out.len() / VOICES;
                    // `region`, `frames`, `mips` resolved from self.table — see Step 4.
                    let mut col = [0.0f32; BLOCK];
                    let mut ocol = [0.0f32; BLOCK];
                    for v in 0..VOICES {
                        for i in 0..n { col[i] = pitch[i * VOICES + v]; }
                        if matches!(self.kind, Kind::PolyWtMorph) {
                            w.process_voice_morph(v, region, frames, In::A(&col[..n]), ins[1], ins[2], dt, &mut ocol[..n]);
                        } else {
                            w.process_voice(v, mips, In::A(&col[..n]), ins[1], dt, &mut ocol[..n]);
                        }
                        for i in 0..n { out[i * VOICES + v] = ocol[i]; }
                    }
                }
            }
```

> `BLOCK` is the graph crate's block const (the engine's `poly_scratch` uses it). `In::A(&[f32])` names a buffer `In` — confirm the variant name against node.rs's mono arms. `ins[1]` = pmod port, `ins[2]` = morph position port — match the port order the mono `Kind::Wavetable` factory binds. Read `self.kind` into a local before the `self.state` borrow if the borrow checker complains.

- [ ] **Step 4: Table-region resolution + engine plumbing**

node.rs: add `Kind::PolyWt` + `Kind::PolyWtMorph` (or a single `PolyWt` kind that branches on `frames`, mirroring how mono `Kind::Wavetable` branches on frames) + `State::PolyWt(PolyWt)`. `bind_table`/`table_src` already exist and are Kind-agnostic — reuse them so a poly wavetable node carries a `TableSrc` the same way. Predicates: `out_width VOICES`, `is_poly`, `poly_in_count => 1` (pitch; pmod/position are mono controls on trailing ports).

The table region must be resolved and handed to the kernel. Mirror the mono `Kind::Wavetable` arm (node.rs:504+): resolve `region` from `self.table` (`TableSrc::Static(id)` → `static_table_flat(id)`, `TableSrc::Pooled` → the engine-passed pool region), build the `MipSet` via `compact_levels`, decide single vs morph by `frames` (Kind `PolyWt` vs `PolyWtMorph`), and feed the Step 3 arm's per-voice `process_voice`/`process_voice_morph` calls. Because poly nodes dispatch through `poly_process` (which today lacks `pool_region`), extend the poly dispatch to receive it:

- In `engine.rs` (the poly branch, ~:234-244), resolve the pool region for a poly wavetable node exactly as the mono path does at its call site, and pass it into `poly_process`.
- Change `Node::poly_process` signature to accept `pool_region: Option<&mut [f32]>` (mirror `process_resolved`'s param). Thread `poly_in`/`ins`/`pool_region` into the `PolyWt` arm. All other poly arms ignore `pool_region`.

> This is the meatiest plumbing in Sy-2d. Keep the change minimal: add the one param, resolve it in the engine only for the poly-wavetable kind (or unconditionally, matching the mono path), and use it only in the `PolyWt`/`PolyWtMorph` arm. Read the mono `Kind::Wavetable` arm and its engine call site in full before writing, and mirror the static-vs-pooled resolution + the `frames`-based single/morph branch verbatim.

- [ ] **Step 5: Run to verify + graph render test**

Run `polywt` kernel/graph tests both configs. Add a graph render test: a `PolyWt` voice (static saw table) and a `PolyWtMorph` voice (a 2D morph bank with a `position` control) each render finite/bounded/non-silent; and a per-voice null test (poly lane == mono `Kind::Wavetable` fed that pitch).

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-dsp-kernels/src/poly.rs crates/deluge-audio-graph/src/node.rs crates/deluge-audio-graph/src/engine.rs
git commit -m "feat(kernels+graph): PolyWt wavetable (scalar-per-voice, shared MipSet, single + 2D morph)"
```

---

## Task 6: Wren surface + end-to-end

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs`, `bindings.rs`, `wren/prelude.wren`
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: the graph Kinds from Tasks 2–5 (`PolyPink`/`PolyBrown`, PolyOsc width port, `PolySync*`, `PolyWt`/`PolyWtMorph`), the poly-node construction path used by existing poly factories (`node_polyosc_impl` etc.), `Node.polyMode_`.
- Produces: Wren `polypink_`/`polybrown_`/`polysync_`/`polywt_`/`polywt_pooled_` factories; poly-aware `Noise.pink/brown`, `Osc.syncX`, `Osc.wavetable`, `.width =`.

- [ ] **Step 1: Rust factories**

In bindings_audio.rs, mirror the existing mono factories, building the poly Kinds:
- `node_polypink_impl` (arity 0) → poly node `Kind::PolyPink`; `node_polybrown_impl` (arity 0) → `Kind::PolyBrown`.
- `node_polysync_impl` (arity 3: wave, master, slave) → `Kind::PolySyncSine/Saw/Square/Tri` by wave code, inputs `[master, slave]` (both poly edges).
- `node_polywt_impl` (arity: table, freq, pmod?, position?) and `node_polywt_pooled_impl` — mirror `node_wavetable_impl`/`node_wavetable_pooled_impl` (bindings_audio.rs:411/508), binding the `TableSrc` via `bind_table`, input `[freq]` (poly pitch), pmod/position as trailing mono controls; select single vs morph the same way the mono factory does.
- For PWM there is no new factory — the `.width =` setter (`node_set_width_impl`, bindings_audio.rs:1345) must, in poly mode, write the PolyOsc **poly width port** (port 1) instead of the mono port 2. Add a poly branch or a `polyosc_set_width_` path.

- [ ] **Step 2: Registration + foreign decls**

bindings.rs: register `polypink_()`, `polybrown_()`, `polysync_(_,_,_)`, `polywt_(...)`, `polywt_pooled_(...)` in METHODS + `register_audio`. prelude.wren: add matching `foreign static` decls in the `Node` class (the Sy-2c gotcha — a missing decl errors "metaclass does not implement").

- [ ] **Step 3: Flip the prelude routes**

- `Noise.pink()`/`brown()` (prelude.wren:714-721): `if (Node.polyMode_ == 1) return Node.polypink_()` / `polybrown_()`; else the mono `Node.pink_()`/`brown_()`.
- `Osc.syncSine/Saw/Square/Tri(master, slave)` (prelude.wren:358-373): `if (Node.polyMode_ == 1) return Node.polysync_(<waveCode>, master, slave)`; else mono `Node.sync_(...)`.
- `Osc.wavetable(t, f)` (prelude.wren:374-378): `if (Node.polyMode_ == 1) return <polywt_ / polywt_pooled_ by table type>`; else the mono static-vs-pooled dispatch. Preserve the pooled-vs-static branch on both sides.
- `.width =` setter: in poly mode, route to the PolyOsc poly width port (fed via Task 1 broadcast from a mono LFO, or a poly source).

- [ ] **Step 4: End-to-end tests**

In tests/audio_bindings.rs (mirror the Sy-2c `synth_moog_and_ms20_render_sound` harness):

```rust
#[test]
fn synth_sources_render_sound() {
    run_and_render("Synth.new { |p| Osc.syncSaw(p, p*1.5).lpf(2000) * Env.ar(0.01,0.3) }", /* finite,|s|<=8.1,non-silent */);
    run_and_render("Synth.new { |p| Osc.wavetable(WT.saw, p) * Env.ar(0.01,0.3) }", ...);
    run_and_render("Synth.new { |p| Noise.pink() * Env.ar(0.01,0.3) }", ...);
    run_and_render("Synth.new { |p| Noise.brown() * Env.ar(0.01,0.3) }", ...);
    // PWM: square with a mono LFO on width (broadcast) — renders, non-silent.
    run_and_render("Synth.new { |p| var o = Osc.square(p); o.width = LFO.sine(4).to(0.2,0.8); o * Env.ar(0.01,0.3) }", ...);
}
```

> Use the exact harness helper names the existing tests use (`run_and_render` / the abort-expecting helper). If a wavetable morph or per-voice-position path is still deferred, keep it aborting and don't test it as working. Confirm `Osc.wavetable` with a 2D-morph table renders (morph supported per Task 5).

- [ ] **Step 5: Backward-compat regression**

Add/confirm a test that an existing PolyOsc voice with no `.width` set is bit-unchanged, and run the FULL wren-core suite both configs — no existing mono `Osc.sync`/`wavetable`/`Noise.pink` test (used OUTSIDE a Synth) regressed (the flips are `polyMode_`-gated).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` and `--features deluge-dsp-kernels/simd`. Expected: PASS both.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): poly sync/wavetable/PWM/pink-brown in Synth (flip source routes)"
```

---

## Self-Review Notes (for the executor)

- **Oracle discipline:** every scalar-per-voice kernel (noise, wavetable) holds `[MonoKernel; VOICES]` and reuses the mono per-sample/per-block entry point, so the lane == mono oracle is bit-identical by construction. The two f32x8 kernels (PWM osc, PolySync) share their sample math with the scalar path (`wave_sample`/`SyncOsc::tick`) and are null-tested ≤1e-4.
- **Backward-compat is a hard gate (Task 3):** PolyOsc gaining a width port must NOT change existing voices. The all-zero-width ⇒ 0.5-duty path is the proof; if any existing PolyOsc render/graph test changes output, stop and fix before proceeding.
- **`poly_in[1]` is always `Some` when `poly_in_count==2`** (engine contract) — do not fall back to `poly_in[0]`/pitch for width.
- **PolySync nan-safety:** guard the `t_reset` division so non-reset lanes stay finite (masked-away but keep them finite to avoid target-specific nan propagation surprises).
- **Wavetable plumbing** is the riskiest task: read the mono `Kind::Wavetable` arm + its engine pool-region call site IN FULL and mirror the static/pooled resolution + `frames` branch. The one signature change (poly_process gains `pool_region`) touches all poly arms — they just ignore it.
- **Deferred (still abort, do NOT implement):** Tb303/Modal poly filters, `Bus.write(src)` poly footgun, per-voice morph position.
- **Import/`In`-variant names:** confirm `In::A`/`In::K` (or `In::Const`/`In::Buf`) against the actual codebase before transcribing test/kernel code — match what poly.rs/node.rs already use.
