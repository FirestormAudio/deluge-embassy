# Poly Granular (Sa-4) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A poly granular voice — `Granular.new(pitch, sampleBuffer)` inside `Synth.new{}` plays a cloud of overlapping Hann-windowed grains from an in-RAM `SampleBuffer`, note pitch transposing the grains.

**Architecture:** New `PolyGranular` kernel (`granular.rs`, scalar, reuses `sampler.rs`'s `hermite_read` + a xorshift32 for spray) → `Kind::PolyGranular` graph node (mirrors `Kind::PolySamplePlayer`, reads the shared `SampleBuffer` pool region) → Wren `Granular.new` (mirrors `Sample.new`).

**Tech Stack:** `deluge-dsp-kernels` (scalar kernel, `libm::{cosf,exp2f}`), `deluge-audio-graph` (node), `deluge-wren-core` (binding + prelude). VOICES=8, MAX_GRAINS=8.

## Global Constraints

- `no_std`, no heap, **no panic on any input** (empty buffer, zero/huge/negative density/size, `spray` out of range, out-of-range voice). Grain `pos` clamped into `[0, len)` at spawn; `hermite_read` is already OOB-safe (32-bit-`usize` disciplined — see [[target-32bit-usize-overflow]]).
- VOICES == 8, MAX_GRAINS == 8. MIT/Apache-2.0. **Scalar** (per-grain gathers don't vectorize — [[prefer-neon-simd]] "don't force it"); tests pass BOTH crate configs.
- Purely additive — no existing node/kernel/binding behavior changes.
- Both Wren registration tables + prelude; new foreign reads use `checked_*` slotapi guards ([[wren-binding-safety]]).
- Test invocation: per-crate, NEVER `--workspace`; both configs. LSP `armv7a … can't find crate for test` noise.

## Interfaces (produced this sub-project)

- Kernel: `PolyGranular::{new, process_voice(v, pcm: &[f32], hz: In, dt: f32, out: &mut [f32]), trigger_voice(v), set_root/set_position/set_size/set_density/set_spray}`.
- Graph: `Kind::PolyGranular`; `set_param` `0=root, 1=position, 2=size, 3=density, 4=spray`.
- Wren: `Granular.new(pitch, buffer)` + `position=`/`size=`/`density=`/`spray=`/`root=`.

---

### Task 1: `PolyGranular` kernel

**Files:**
- Create: `crates/deluge-dsp-kernels/src/granular.rs`; Modify `lib.rs` (`pub mod granular;`)
- Modify: `crates/deluge-dsp-kernels/src/sampler.rs` (make `hermite_read` `pub(crate)`)
- Test: `granular.rs` test module

**Interfaces:**
- Consumes: `sampler::hermite_read` (make `pub(crate)`), `crate::poly::VOICES`, `crate::In`, `libm::{cosf, exp2f}`.
- Produces: `PolyGranular` (see Interfaces). Consumed by Task 2.

- [ ] **Step 1: Write the failing tests**

`granular.rs` test module (the crate's tests use `std::vec::Vec` via `extern crate std;`):
```rust
#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;

    fn ramp(n: usize) -> std::vec::Vec<f32> { (0..n).map(|i| i as f32).collect() }

    #[test]
    fn density_zero_is_silent() {
        let pcm = ramp(1000);
        let mut g = PolyGranular::new();
        g.set_density(0.0);
        g.trigger_voice(0);
        let mut out = [1.0f32; 64];
        g.process_voice(0, &pcm, In::K(261.6256), 1.0 / 48000.0, &mut out);
        assert!(out.iter().all(|&s| s == 0.0), "no grains → silence");
    }

    #[test]
    fn triggered_cloud_renders_bounded_nonsilent() {
        let pcm = ramp(4000);
        let mut g = PolyGranular::new();
        g.set_position(0.25);
        g.set_size(20.0);
        g.set_density(100.0);
        g.trigger_voice(0);
        let mut out = [0.0f32; 256];
        g.process_voice(0, &pcm, In::K(261.6256), 1.0 / 48000.0, &mut out);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 5000.0), "finite/bounded");
        assert!(out.iter().any(|&s| s != 0.0), "cloud sounds");
    }

    #[test]
    fn hann_window_shape() {
        assert!((hann(0.0)).abs() < 1e-6);           // fades in from 0
        assert!((hann(0.5) - 1.0).abs() < 1e-6);      // peak at center
        assert!((hann(1.0)).abs() < 1e-6);            // fades to 0
    }

    #[test]
    fn spray_zero_is_deterministic_at_position() {
        // spray=0 → every grain starts exactly at position*len (no jitter).
        // Compare two fresh clouds: identical output.
        let pcm = ramp(2000);
        let run = || {
            let mut g = PolyGranular::new();
            g.set_position(0.5); g.set_size(10.0); g.set_density(200.0); g.set_spray(0.0);
            g.trigger_voice(0);
            let mut out = [0.0f32; 128];
            g.process_voice(0, &pcm, In::K(261.6256), 1.0/48000.0, &mut out);
            out
        };
        assert_eq!(run(), run(), "spray=0 is deterministic");
    }

    #[test]
    fn rate_octave_up_reads_twice_as_fast() {
        // A voice one octave above root advances grain pos at ~2x. Assert the
        // cloud is non-silent at both pitches and differs (higher pitch shifts content).
        let pcm = ramp(4000);
        let render = |hz: f32| {
            let mut g = PolyGranular::new();
            g.set_position(0.1); g.set_size(30.0); g.set_density(100.0);
            g.trigger_voice(0);
            let mut out = [0.0f32; 128];
            g.process_voice(0, &pcm, In::K(hz), 1.0/48000.0, &mut out);
            out
        };
        let root = render(261.6256);   // note 60
        let octave = render(523.2512); // note 72 → rate 2
        assert!(octave.iter().any(|&s| s != 0.0));
        assert!(root.iter().zip(octave.iter()).any(|(a, b)| (a - b).abs() > 1e-4), "pitch changes the cloud");
    }

    #[test]
    fn no_panic_on_adversarial() {
        let mut g = PolyGranular::new();
        g.trigger_voice(0);
        let mut out = [0.0f32; 32];
        let empty: [f32; 0] = [];
        g.process_voice(0, &empty, In::K(440.0), 1.0/48000.0, &mut out); // empty buffer
        g.set_density(1e9); g.set_size(-5.0); g.set_spray(9.0);
        g.process_voice(0, &ramp(10), In::K(440.0), 1.0/48000.0, &mut out); // absurd params, tiny buffer
        g.process_voice(99, &ramp(10), In::K(440.0), 1.0/48000.0, &mut out); // out-of-range voice
        assert!(out.iter().all(|s| s.is_finite()));
    }
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- granular::`
Expected: FAIL to compile (`PolyGranular`/`hann` undefined).

- [ ] **Step 3: Make `hermite_read` `pub(crate)`**

`sampler.rs:26`: `fn hermite_read(...)` → `pub(crate) fn hermite_read(...)`.

- [ ] **Step 4: Implement `granular.rs`**

`lib.rs`: add `pub mod granular;` (alphabetical — after `pub mod filter;`, before `pub mod lfo;`, or wherever alphabetical). Then:
```rust
//! Poly granular synthesis (Sa-4): each voice plays a cloud of overlapping
//! Hann-windowed grains from an in-RAM sample, note pitch transposing the grains.
//! Scalar (per-grain gathers). `no_std`, no heap, no panic.

use crate::poly::VOICES;
use crate::sampler::hermite_read;
use crate::In;

const MAX_GRAINS: usize = 8;

/// Hann window over the grain's normalized phase `[0,1]`.
#[inline]
pub(crate) fn hann(phase: f32) -> f32 {
    0.5 - 0.5 * libm::cosf(core::f32::consts::TAU * phase)
}

#[inline]
fn xorshift32(s: &mut u32) -> u32 {
    let mut r = *s;
    r ^= r << 13;
    r ^= r >> 17;
    r ^= r << 5;
    *s = r;
    r
}

#[derive(Clone, Copy)]
struct Grain { active: bool, pos: f32, rate: f32, phase: f32, phase_inc: f32 }
impl Grain {
    const fn new() -> Grain { Grain { active: false, pos: 0.0, rate: 1.0, phase: 0.0, phase_inc: 1.0 } }
}

#[derive(Clone, Copy)]
struct GrainCloud {
    grains: [Grain; MAX_GRAINS],
    rng: u32,
    spawn_accum: f32,
    playing: bool,
    position: f32, // 0..1 fraction of the buffer
    size_ms: f32,
    density: f32,  // grains/sec
    spray: f32,    // 0..1 position-jitter fraction
    root: f32,     // MIDI note played at grain rate 1.0
}
impl GrainCloud {
    const fn new() -> GrainCloud {
        GrainCloud {
            grains: [Grain::new(); MAX_GRAINS], rng: 1, spawn_accum: 0.0, playing: false,
            position: 0.0, size_ms: 50.0, density: 20.0, spray: 0.0, root: 60.0,
        }
    }
}

#[derive(Clone, Copy)]
pub struct PolyGranular { voices: [GrainCloud; VOICES] }

impl PolyGranular {
    pub fn new() -> PolyGranular { PolyGranular { voices: [GrainCloud::new(); VOICES] } }

    pub fn set_root(&mut self, note: f32) { for c in &mut self.voices { c.root = note; } }
    pub fn set_position(&mut self, p: f32) { for c in &mut self.voices { c.position = p; } }
    pub fn set_size(&mut self, ms: f32) { for c in &mut self.voices { c.size_ms = ms; } }
    pub fn set_density(&mut self, d: f32) { for c in &mut self.voices { c.density = d; } }
    pub fn set_spray(&mut self, s: f32) { for c in &mut self.voices { c.spray = s; } }

    /// (Re)start voice `v`'s cloud: clear grains, reseed the (decorrelated) RNG.
    pub fn trigger_voice(&mut self, v: usize) {
        if v < VOICES {
            let c = &mut self.voices[v];
            c.grains = [Grain::new(); MAX_GRAINS];
            c.spawn_accum = 0.0;
            c.rng = 0x2545_F491 ^ (v as u32 + 1).wrapping_mul(0x9E37_79B9);
            c.playing = true;
        }
    }

    pub fn process_voice(&mut self, v: usize, pcm: &[f32], hz: In, dt: f32, out: &mut [f32]) {
        if v >= VOICES {
            for o in out.iter_mut() { *o = 0.0; }
            return;
        }
        let len = pcm.len();
        let c = &mut self.voices[v];
        if !c.playing || len == 0 {
            for o in out.iter_mut() { *o = 0.0; }
            return;
        }
        let flen = len as f32;
        let density = c.density.max(0.0);
        let spray = c.spray.clamp(0.0, 1.0);
        let size_s = (c.size_ms.max(0.1)) / 1000.0;
        let root_hz = 440.0 * libm::exp2f((c.root - 69.0) / 12.0);
        for (i, o) in out.iter_mut().enumerate() {
            let rate = hz.at(i).max(1e-6) / root_hz;
            // schedule new grains
            c.spawn_accum += density * dt;
            while c.spawn_accum >= 1.0 {
                c.spawn_accum -= 1.0;
                let jit = (xorshift32(&mut c.rng) as f32 / u32::MAX as f32) * 2.0 - 1.0; // [-1,1)
                let start = (c.position.clamp(0.0, 1.0) * flen + jit * spray * flen)
                    .clamp(0.0, (flen - 1.0).max(0.0));
                let size_samples = (size_s / dt).max(2.0);
                if let Some(g) = c.grains.iter_mut().find(|g| !g.active) {
                    *g = Grain { active: true, pos: start, rate, phase: 0.0, phase_inc: 1.0 / size_samples };
                }
            }
            // mix + advance active grains
            let mut sum = 0.0f32;
            for g in c.grains.iter_mut() {
                if !g.active { continue; }
                sum += hann(g.phase) * hermite_read(pcm, g.pos, 0, len as isize, false);
                g.pos += g.rate;
                g.phase += g.phase_inc;
                if g.phase >= 1.0 { g.active = false; }
            }
            *o = sum;
        }
    }
}
impl Default for PolyGranular { fn default() -> Self { PolyGranular::new() } }
```
> No-panic: `len==0`/`!playing`/`v>=VOICES` → silence; `density.max(0)`/`spray.clamp`/`size_ms.max(0.1)`/`size_samples.max(2.0)` bound the params; `start.clamp(0, len-1)` keeps grain reads in range; `hermite_read` clamps taps. The grain `find(!active)` + `xorshift32(&mut c.rng)` are disjoint field accesses (jit/start/size computed before the `grains` borrow). `spawn_accum` can't loop forever (each iteration subtracts 1.0; `density*dt` is finite).

- [ ] **Step 5: Run tests, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- granular::` (+ `--features simd`), then the FULL crate both configs. Expected: PASS (existing tests unaffected — additive; `hermite_read` visibility widened only).

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-dsp-kernels/src/granular.rs crates/deluge-dsp-kernels/src/lib.rs crates/deluge-dsp-kernels/src/sampler.rs
git commit -m "feat(kernels): PolyGranular — poly grain-cloud synthesis over a sample buffer (Hann, xorshift spray)"
```

---

### Task 2: `Kind::PolyGranular` graph node

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs`
- Test: `node.rs` test module

**Interfaces:**
- Consumes: `PolyGranular` (Task 1). Produces: `Kind::PolyGranular` + its `set_param` scheme. Consumed by Task 3.

- [ ] **Steps:** Mirror `Kind::PolySamplePlayer` EXACTLY (it's the sibling pooled poly source). Add: `Kind::PolyGranular`; `State::PolyGranular(PolyGranular)` + `Node::new` constructor arm; `PolyGranular` in the `out_width => VOICES`, `is_poly` (true), `poly_in_count => 1` arms (alongside `PolySamplePlayer`); `Node::trigger_voice` arm `State::PolyGranular(p) => p.trigger_voice(v)`; `set_param` arm:
```rust
State::PolyGranular(p) => match param {
    0 => p.set_root(value),
    1 => p.set_position(value),
    2 => p.set_size(value),
    3 => p.set_density(value),
    4 => p.set_spray(value),
    _ => {}
},
```
and the `poly_process` arm mirroring `Kind::PolySamplePlayer`'s (de-interleave `poly_in[0]` per lane into a `MAX_BLOCK` col, read `pool_region.as_deref()` as raw PCM, `p.process_voice(v, region, In::A(&col[..n]), dt, &mut ocol[..n])`, re-interleave). Node test (mirror the PolySamplePlayer node test): `poly_in_count(PolyGranular)==1`, `is_poly`, `out_width==VOICES`; a triggered voice over a bound pool region + a pitch tile renders finite (density>0). Commit.

---

### Task 3: Wren `Granular.new` + setters

**Files:**
- Modify: `crates/deluge-wren-core/src/{audio.rs, bindings_audio.rs, bindings.rs}`, `wren/prelude.wren`
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Kind::PolyGranular`, the `Sample.new` binding pattern (`node_polysampleplayer_impl` + `new_poly_sample_player` + tag-dispatch + `poly_record_trigger` + `return_poly_node`).
- Produces: `Granular.new(pitch, buffer)` + setters.

- [ ] **Steps:** Mirror `Sample.new` (a poly voice source over a `SampleBuffer`). `audio::new_poly_granular(id, handle, pitch)` = `Cmd::NewNode{Kind::PolyGranular,[pitch,Const,Const]}` + (if `Some(h)`) `Cmd::BindTable{Pooled(h)}` (NO SetParams needed — defaults are set in the kernel; `position=` etc. are set via later setters). `node_granular_impl<S>`: `pitch=arg_input(vm,1)`; read the `SampleObj` handle at slot 2 via **`checked_tagged_foreign::<SampleObj>(vm, 2, TAG_SAMPLE)`** (the wren-binding-safety guard, like `Sample.new`; non-`SampleObj` → `None` handle → silent); `id=alloc_node_id()`; `new_poly_granular(id, handle, pitch)`; `poly_record_trigger(id)`; `return_poly_node`. Setters `node_set_position/size/density/spray_impl` → `set_param(self_id, {1,2,3,4}, get_f(vm,1))`; reuse a `root=` setter → `set_param(0)` if free (else add `granRoot=`; grep for `root=` collision — `Sample.new`'s `root=` targets param 6, DIFFERENT node kind, so dispatch-by-State keeps them separate — a `root=` on a PolyGranular node hits the PolyGranular set_param param... wait, `root=` emits a fixed param index; confirm the index. If `Sample.new`'s `root=` emits param 6, then for granular `root` is param 0 — a collision in the SETTER's emitted index. RESOLVE: use distinct setter names OR a poly-mode/kind-aware emit. Simplest: NEW setters `position=`/`size=`/`density=`/`spray=` emit params 1..4; for granular root, emit param 0 via a granular-specific path OR default root=60 with no setter this slice. **DEFER a granular `root=` setter — default root 60**, matching the streaming slice's approach). Register `Node.granular_(_,_)` + the 4 setters in BOTH tables + prelude `class Granular { static new(pitch, buffer) { polyMode_ guard; Node.granular_(pitch, buffer) } }` + `foreign` decls. Binding test: `Granular.new(p, SampleBuffer.from([...]))` in a Synth builds; setters build. Commit.

---

### Task 4: e2e

**Files:** Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

- [ ] **Steps:** Mirror the `sample_new_*` render round-trips (real `EngineHost`). Tests: a `Synth.new { |p| Granular.new(p, SampleBuffer.from([… a few hundred samples …])).density = 100 ... * Env.adsr(...) }` renders finite/bounded/non-silent + polyphonic (2 notes); a `position=`/`spray=` change renders (still finite/non-silent); existing synths unchanged. Both configs. Commit.

---

## Self-Review

**Spec coverage:** Task 1 = the `PolyGranular` kernel (grain schedule + Hann mix + advance + xorshift spray + setters/trigger, no-panic). Task 2 = `Kind::PolyGranular` node (mirrors PolySamplePlayer + the 5-param set_param). Task 3 = Wren `Granular.new` + setters (mirrors `Sample.new`, `checked_tagged_foreign` guard, root default 60 / root= setter deferred). Task 4 = e2e. Matches the spec's 4-task decomposition + params (`0=root,1=position,2=size,3=density,4=spray`).

**Placeholder scan:** Task 1's kernel + tests are verbatim. Tasks 2–4 point at the NAMED sibling templates (`Kind::PolySamplePlayer`, `Sample.new`/`node_polysampleplayer_impl`, `sample_new_*` render tests) to mirror — transcription guidance, not gaps; the set_param scheme + the `root=` collision resolution (defer) are spelled out.

**Type consistency:** `PolyGranular::process_voice(v, pcm, hz: In, dt, out)` (Task 1) is consumed identically in Task 2's poly_process arm (matching `PolySamplePlayer::process_voice`'s shape). `set_param` `0=root,1=position,2=size,3=density,4=spray` is consistent between Task 2's arm and Task 3's setters. `Kind::PolyGranular` mirrors `Kind::PolySamplePlayer` at every predicate/dispatch site.
