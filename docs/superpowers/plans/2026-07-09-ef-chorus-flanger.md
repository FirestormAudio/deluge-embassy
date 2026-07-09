# Ef-2 Chorus / Flanger Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add chorus and flanger effects — a `ModDelay<VOICES>` modulated-delay kernel (phase-spread panned voices over a shared ring, internal LFO) exposed as two mono→stereo graph nodes and Wren `Chorus.new`/`Flanger.new`.

**Architecture:** `ModDelay<VOICES>` reuses `DelayLine` (Ef-1), `fast_sin` (LFO), and `math::pan_gains` (voice panning, from the stereo work); it borrows one short pooled ring and writes a stereo pair. `Kind::Chorus` = `ModDelay<3>`, `Kind::Flanger` = `ModDelay<1>` (+feedback), both `out_width` 2, routed to L/R by the merged width-aware `Out.patch`.

**Tech Stack:** Rust, `no_std`/no-heap, Wren via `wren-sys`, proptest.

## Global Constraints

- **`no_std`, no heap, pure `f32`, deterministic.** The LFO is a phase accumulator (no RNG/time). The kernel owns NO ring storage — `buf` is borrowed per call. Use the module-private `floorf` (already in `delay.rs`) for fractional wrapping — do NOT use `f32::fract()` (std-only). `fast_sin` and `math::pan_gains` come from the crate.
- **Modulated delay (multiplicative depth):** per voice `v`, `ph = frac(lfo_phase + v/VOICES)`, `d = base·(1 + depth·fast_sin(ph)) / dt` samples. Multiplicative keeps `d` positive at any base. `read_hermite` clamps as a floor.
- **Voice pan/normalize:** `pos_v = if VOICES==1 { 0.0 } else { -1.0 + 2.0·v/(VOICES-1) }`; `(gl,gr)=pan_gains(pos_v)`; `wet_l += tap·gl; wet_r += tap·gr`; then `wet_l/=VOICES; wet_r/=VOICES`.
- **Feedback (flanger):** `fb = feedback·(wet_l+wet_r)·0.5`; `line.write(buf, x+fb)`. Chorus feedback = 0.
- **Output:** `out_l = x·(1−mix) + wet_l·mix`, `out_r = x·(1−mix) + wet_r·mix` (`x` = input). Read-before-write per sample.
- **Clamps:** `rate ≥ 0`, `depth ∈ [0, 0.99]`, `mix ∈ [0,1]`, `feedback ∈ [0, 0.95]` (BIBO stable).
- **Node kinds:** `Kind::Chorus` = `ModDelay<3>` (base 0.020 s), `Kind::Flanger` = `ModDelay<1>` (base 0.002 s); `out_width` = 2. **Param map:** `set_param` 0=mix, 1=rate, 2=depth, 3=feedback. Dry passthrough (both ports = input) when no bound buffer / buffer < 4 samples; never panic.
- **Wren:** `Chorus.new(input, rate, depth, mix)`, `Flanger.new(input, rate, depth, feedback, mix)` → **width-2** nodes (`return_node_w(vm, id, 2)`). Setters: `mix=` (reuses the Ef-1 selector → `set_param(0)`), `rate=` → `set_param(1)`, `depth=` → `set_param(2)`, **`regen=`** → `set_param(3)` (NOT `feedback=` — that selector is the Osc's `set_param(0)`). Ring buffer `CHORUS_BUF_SAMPLES = 2400` (~50 ms).
- Host tests run with `--target x86_64-unknown-linux-gnu`.

---

### Task 1: `ModDelay<VOICES>` kernel

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/delay.rs` (add `ModDelay` above the `#[cfg(test)]` block; add tests in it)

**Interfaces:**
- Consumes: `DelayLine` (`read_hermite`/`write`), `crate::In`, `crate::fast_sin`, `crate::math::pan_gains`, the module-private `floorf`.
- Produces: `pub struct ModDelay<const VOICES: usize>`; `ModDelay::<V>::new(base_s: f32)`; `set_mix`/`set_rate`/`set_depth`/`set_feedback`; `process(&mut self, input: In, dt: f32, buf: &mut [f32], out_l: &mut [f32], out_r: &mut [f32])`. Task 2 (`State::Chorus`/`Flanger`) consumes these.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-dsp-kernels/src/delay.rs`'s `mod tests` (the `extern crate std;` and `use crate::In;` are already there from Ef-1):

```rust
    // ── ModDelay (chorus / flanger) ───────────────────────────────────────

    fn render_mod<const V: usize>(
        md: &mut ModDelay<V>,
        buf: &mut [f32],
        input: &[f32],
        dt: f32,
    ) -> (std::vec::Vec<f32>, std::vec::Vec<f32>) {
        let mut l = std::vec![0.0f32; input.len()];
        let mut r = std::vec![0.0f32; input.len()];
        md.process(In::A(input), dt, buf, &mut l, &mut r);
        (l, r)
    }

    #[test]
    fn chorus_is_stereo_and_modulates() {
        let dt = 1.0 / 48_000.0;
        let mut buf = [0.0f32; 4096];
        let mut md = ModDelay::<3>::new(0.020);
        md.set_mix(1.0);
        md.set_rate(2.0);
        md.set_depth(0.5);
        // A steady tone so only the effect creates L/R and time variation.
        let input: std::vec::Vec<f32> =
            (0..2000).map(|i| (i as f32 * 0.02).sin()).collect();
        let (l, r) = render_mod(&mut md, &mut buf, &input, dt);
        assert!(l.iter().all(|v| v.is_finite()) && r.iter().all(|v| v.is_finite()));
        // Voices panned → the two channels are not identical.
        let diff: f32 = l.iter().zip(&r).map(|(a, b)| (a - b).abs()).sum();
        assert!(diff > 0.5, "chorus channels should differ (stereo): {diff}");
        // The LFO moves the wet content: a late window differs from an early one.
        let early: f32 = l[200..400].iter().map(|v| v.abs()).sum();
        let late: f32 = l[1600..1800].iter().map(|v| v.abs()).sum();
        assert!((early - late).abs() > 1e-3, "no modulation motion");
    }

    #[test]
    fn flanger_feedback_increases_resonance() {
        let dt = 1.0 / 48_000.0;
        // White-ish input; higher feedback → more energy variance (comb resonance).
        let input: std::vec::Vec<f32> =
            (0..2000).map(|i| ((i * 7 % 13) as f32 / 13.0) - 0.5).collect();
        let energy = |fb: f32| -> f32 {
            let mut buf = [0.0f32; 4096];
            let mut md = ModDelay::<1>::new(0.002);
            md.set_mix(1.0);
            md.set_rate(0.3);
            md.set_depth(0.7);
            md.set_feedback(fb);
            let (l, _r) = render_mod(&mut md, &mut buf, &input, dt);
            l[1000..].iter().map(|v| v * v).sum()
        };
        let low = energy(0.0);
        let high = energy(0.9);
        assert!(high.is_finite() && low.is_finite());
        assert!(high > low * 1.2, "feedback should build resonance: {low} → {high}");
    }

    #[test]
    fn mod_mix_balance() {
        let dt = 1.0 / 48_000.0;
        let input = [0.5f32; 512];
        // mix = 0 → both channels ≈ dry input.
        let mut buf0 = [0.0f32; 4096];
        let mut m0 = ModDelay::<3>::new(0.020);
        m0.set_mix(0.0);
        let (l0, r0) = render_mod(&mut m0, &mut buf0, &input, dt);
        assert!((l0[10] - 0.5).abs() < 1e-4 && (r0[10] - 0.5).abs() < 1e-4);
    }

    #[test]
    fn mod_depth_zero_is_static_and_finite() {
        let dt = 1.0 / 48_000.0;
        let mut buf = [0.0f32; 4096];
        let mut md = ModDelay::<3>::new(0.020);
        md.set_depth(0.0); // no modulation
        md.set_mix(1.0);
        let input = [0.3f32; 512];
        let (l, r) = render_mod(&mut md, &mut buf, &input, dt);
        assert!(l.iter().all(|v| v.is_finite()) && r.iter().all(|v| v.is_finite()));
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels mod`
Expected: FAIL — `ModDelay` not found.

- [ ] **Step 3: Implement `ModDelay`**

In `crates/deluge-dsp-kernels/src/delay.rs`, add imports near the top (the file already has `use crate::In;`):

```rust
use crate::fast_sin;
use crate::math::pan_gains;
```

Add the struct + impl after the `Delay` impl (before `#[cfg(test)]`):

```rust
/// Modulated multi-voice delay: chorus (`VOICES > 1`, no feedback) / flanger
/// (`VOICES == 1` + feedback). `VOICES` phase-spread taps on a shared borrowed
/// ring buffer, an internal LFO, panned to a stereo pair. `no_std`, no heap.
#[derive(Clone, Copy)]
pub struct ModDelay<const VOICES: usize> {
    line: DelayLine,
    lfo_phase: f32, // normalized [0,1)
    base: f32,      // base delay, seconds
    rate: f32,      // LFO Hz
    depth: f32,     // [0, 0.99] fraction-of-base sweep
    mix: f32,       // dry/wet [0,1]
    feedback: f32,  // [0, 0.95] flanger regen
}

impl<const VOICES: usize> ModDelay<VOICES> {
    pub fn new(base_s: f32) -> ModDelay<VOICES> {
        ModDelay { line: DelayLine::new(), lfo_phase: 0.0, base: base_s,
                   rate: 0.5, depth: 0.25, mix: 0.5, feedback: 0.0 }
    }
    pub fn set_mix(&mut self, v: f32) { self.mix = v.clamp(0.0, 1.0); }
    pub fn set_rate(&mut self, v: f32) { self.rate = v.max(0.0); }
    pub fn set_depth(&mut self, v: f32) { self.depth = v.clamp(0.0, 0.99); }
    pub fn set_feedback(&mut self, v: f32) { self.feedback = v.clamp(0.0, 0.95); }

    /// One block. `input` = mono; `buf` = pooled ring; writes the stereo pair.
    pub fn process(&mut self, input: In, dt: f32, buf: &mut [f32],
                   out_l: &mut [f32], out_r: &mut [f32]) {
        let norm = 1.0 / VOICES as f32;
        for i in 0..out_l.len() {
            let x = input.at(i);
            // advance LFO phase, wrapped to [0,1) without std fract().
            let p = self.lfo_phase + self.rate * dt;
            self.lfo_phase = p - floorf(p);
            let mut wet_l = 0.0;
            let mut wet_r = 0.0;
            for v in 0..VOICES {
                let t = self.lfo_phase + v as f32 / VOICES as f32;
                let ph = t - floorf(t);
                let d = self.base * (1.0 + self.depth * fast_sin(ph)) / dt;
                let tap = self.line.read_hermite(buf, d);
                let pos = if VOICES == 1 {
                    0.0
                } else {
                    -1.0 + 2.0 * v as f32 / (VOICES as f32 - 1.0)
                };
                let (gl, gr) = pan_gains(pos);
                wet_l += tap * gl;
                wet_r += tap * gr;
            }
            wet_l *= norm;
            wet_r *= norm;
            let fb = self.feedback * (wet_l + wet_r) * 0.5;
            self.line.write(buf, x + fb);
            out_l[i] = x * (1.0 - self.mix) + wet_l * self.mix;
            out_r[i] = x * (1.0 - self.mix) + wet_r * self.mix;
        }
    }
}
```

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels mod`
Expected: PASS (4 tests).

- [ ] **Step 5: Add the boundedness proptest**

Append to `mod tests`:

```rust
    proptest! {
        #![proptest_config(ProptestConfig { cases: 64, ..ProptestConfig::default() })]
        #[test]
        fn moddelay_is_finite_and_bounded(
            rate in 0.0f32..8.0,
            depth in 0.0f32..1.0,   // clamped to 0.99 inside
            mix in 0.0f32..1.0,
            feedback in 0.0f32..1.0, // clamped to 0.95 inside
            amp in 0.0f32..1.0,
        ) {
            let dt = 1.0 / 48_000.0;
            let mut buf = [0.0f32; 4096];
            let mut md = ModDelay::<3>::new(0.020);
            md.set_rate(rate); md.set_depth(depth); md.set_mix(mix); md.set_feedback(feedback);
            let input = std::vec![amp; 2000];
            let (l, r) = render_mod(&mut md, &mut buf, &input, dt);
            for (a, b) in l.iter().zip(&r) {
                prop_assert!(a.is_finite() && b.is_finite());
                prop_assert!(a.abs() <= 8.0 && b.abs() <= 8.0, "unbounded: {a},{b}");
            }
        }
    }
```

- [ ] **Step 6: Run the proptest**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels mod`
Expected: PASS (including `moddelay_is_finite_and_bounded`).

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-dsp-kernels/src/delay.rs
git commit -m "feat(dsp-kernels): ModDelay<VOICES> chorus/flanger kernel"
```

---

### Task 2: Graph — `Kind::Chorus`/`Flanger` + `OutView::port_pair`

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs` (import, Kind, State, `Node::new`, `out_width`, `set_param`, render arm, `OutView::port_pair`, tests)

**Interfaces:**
- Consumes: `deluge_dsp_kernels::delay::ModDelay` (Task 1).
- Produces: `Kind::Chorus`/`Kind::Flanger`; `OutView::port_pair(&mut self) -> (&mut [f32], &mut [f32])`. Task 3's factories create these kinds.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-audio-graph/src/node.rs`'s `mod tests`:

```rust
    #[test]
    fn chorus_node_renders_stereo_with_buffer() {
        let mut n = Node::new(Kind::Chorus, 0);
        assert_eq!(Node::out_width(Kind::Chorus), 2);
        n.set_param(0, 1.0); // mix = wet
        n.set_param(1, 2.0); // rate
        n.set_param(2, 0.6); // depth
        let dt = 1.0 / 48_000.0;
        let input: [f32; 64] = core::array::from_fn(|i| (i as f32 * 0.1).sin());
        let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut ring = [0.0f32; 4096];
        let mut p0 = [0.0f32; 64];
        let mut p1 = [0.0f32; 64];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, dt, &mut outs, Some(&mut ring));
        }
        assert!(p0.iter().all(|s| s.is_finite() && s.abs() <= 8.0));
        assert!(p1.iter().all(|s| s.is_finite() && s.abs() <= 8.0));
    }

    #[test]
    fn chorus_node_without_buffer_is_dry_both_ports() {
        let mut n = Node::new(Kind::Flanger, 0);
        assert_eq!(Node::out_width(Kind::Flanger), 2);
        let input = [0.4f32; 16];
        let ins = [In::A(&input), In::A(&[0.0; 16]), In::A(&[0.0; 16])];
        let mut p0 = [0.0f32; 16];
        let mut p1 = [0.0f32; 16];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None); // no ring
        }
        assert!(p0.iter().all(|&s| (s - 0.4).abs() < 1e-6), "L dry: {p0:?}");
        assert!(p1.iter().all(|&s| (s - 0.4).abs() < 1e-6), "R dry: {p1:?}");
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph chorus`
Expected: FAIL — no variant `Chorus`.

- [ ] **Step 3: Add `OutView::port_pair`**

In `crates/deluge-audio-graph/src/node.rs`, add to `impl<'a> OutView<'a>` (next to `port`):

```rust
    /// Both output ports as disjoint mutable slices (for width-2 stereo nodes).
    /// Panics if this view has fewer than 2 ports.
    pub fn port_pair(&mut self) -> (&mut [f32], &mut [f32]) {
        let [a, b] = &mut self.ports;
        (
            a.as_deref_mut().expect("port 0 present"),
            b.as_deref_mut().expect("port 1 present"),
        )
    }
```

- [ ] **Step 4: Add the kinds, state, construction, params**

In `node.rs`:

Extend the `ModDelay` import (the file already imports `delay::Delay`):

```rust
use deluge_dsp_kernels::delay::{Delay, ModDelay};
```

Add to `enum Kind` (after `Delay`):

```rust
    Delay,
    Chorus,
    Flanger,
```

Add to `enum State` (after `Delay(Delay)`):

```rust
    Chorus(ModDelay<3>),
    Flanger(ModDelay<1>),
```

In `Node::new`, add arms (after `Kind::Delay => …`):

```rust
            Kind::Chorus => State::Chorus(ModDelay::<3>::new(0.020)),
            Kind::Flanger => State::Flanger(ModDelay::<1>::new(0.002)),
```

In `out_width`, extend the width-2 arm:

```rust
            Kind::Split2 | Kind::Pan | Kind::Chorus | Kind::Flanger => 2,
```

In `set_param`, add arms (alongside `State::Delay`):

```rust
            State::Chorus(md) => match param {
                0 => md.set_mix(value),
                1 => md.set_rate(value),
                2 => md.set_depth(value),
                3 => md.set_feedback(value),
                _ => {}
            },
            State::Flanger(md) => match param {
                0 => md.set_mix(value),
                1 => md.set_rate(value),
                2 => md.set_depth(value),
                3 => md.set_feedback(value),
                _ => {}
            },
```

- [ ] **Step 5: Add the render arm**

In `process_resolved`, add after the `Kind::Delay` arm:

```rust
            Kind::Chorus | Kind::Flanger => {
                // Mono→stereo modulated delay. Bound ring (≥4) → run the
                // effect writing both ports; otherwise dry passthrough to both.
                let (out_l, out_r) = outs.port_pair();
                let ran = if let Some(buf) = pool_region {
                    if buf.len() >= 4 {
                        match &mut self.state {
                            State::Chorus(md) => { md.process(ins[0], dt, buf, out_l, out_r); true }
                            State::Flanger(md) => { md.process(ins[0], dt, buf, out_l, out_r); true }
                            _ => false,
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

- [ ] **Step 6: Run the new tests + full suite**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph chorus`
Expected: PASS (2 new tests).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: PASS — all existing tests too (Pan/Delay/wavetable arms unchanged).

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(audio-graph): Kind::Chorus/Flanger stereo nodes + OutView::port_pair"
```

---

### Task 3: Wren `Chorus.new` / `Flanger.new` surface

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (`new_pooled_node` emitter)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`CHORUS_BUF_SAMPLES`, `node_chorus_impl`/`node_flanger_impl`, `node_set_rate_impl`/`node_set_depth_impl`/`node_set_regen_impl` + wrappers, register)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (wren-sys registration)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`chorus_`/`flanger_` + `rate=`/`depth=`/`regen=` decls + `class Chorus`/`class Flanger`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Kind::Chorus`/`Flanger` (Task 2), `audio::{alloc_buffer, alloc_node_id, set_param}`, `arg_input`, `return_node_w`.
- Produces: `audio::new_pooled_node(id, kind, handle, input)`; Wren `Chorus.new`/`Flanger.new`, `rate=`/`depth=`/`regen=`.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn chorus_new_emits_node_and_params_no_bind_on_capture_host() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    // CmdCaptureHost has no pool → alloc_buffer None → NewNode + SetParams but
    // NO BindTable (dry-passthrough contract, same as Ef-1 Delay). The real
    // bind + render is covered by `chorus_renders_stereo_bounded_on_engine_host`.
    let cmds = run_and_capture_cmds("var c = Chorus.new(Osc.saw(110), 0.5, 0.4, 0.5)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Chorus, .. })));
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::BindTable { .. })), "no pool → no BindTable: {cmds:?}");
    // rate(1)/depth(2)/mix(0) SetParams emitted from the constructor args (pool-independent).
    for p in [0u8, 1, 2] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, .. } if *param == p)), "missing SetParam {p}: {cmds:?}");
    }
}

#[test]
fn chorus_patch_routes_stereo() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    // A Chorus is width-2 → Out.patch emits the two side-writes.
    let cmds = run_and_capture_cmds("Out.patch(Chorus.new(Osc.saw(110), 0.5, 0.4, 0.5))");
    let gains: std::vec::Vec<(f32, f32)> = cmds.iter().filter_map(|c| match c {
        Cmd::BusWriteGains { gl, gr, .. } => Some((*gl, *gr)),
        _ => None,
    }).collect();
    assert!(gains.contains(&(1.0, 0.0)) && gains.contains(&(0.0, 1.0)), "not stereo-routed: {gains:?}");
}

#[test]
fn flanger_regen_sets_feedback_param() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds(
        "var f = Flanger.new(Osc.saw(110), 0.3, 0.7, 0.6, 0.5)\nf.regen = 0.8",
    );
    // Flanger.new sets feedback (param 3) from its arg; regen= sets it again.
    let p3 = cmds.iter().filter(|c| matches!(c, Cmd::SetParam { param: 3, .. })).count();
    assert!(p3 >= 2, "expected feedback param set by ctor and regen=: {cmds:?}");
}

#[test]
fn chorus_renders_stereo_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Chorus.new(Osc.saw(110), 1.0, 0.5, 0.6))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite() && f.l.abs() <= 1.0 && f.r.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0 || f.r != 0.0), "chorus should be non-silent");
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings chorus flanger`
Expected: FAIL — Wren `Chorus`/`Flanger` undefined.

- [ ] **Step 3: Add the `new_pooled_node` emitter**

In `crates/deluge-wren-core/src/audio.rs`, add after `new_delay`:

```rust
/// Create a pooled effect node of `kind` with `input` on port 0, binding a
/// pool ring if `handle` is `Some` (unbound → dry passthrough). Params are set
/// separately by the caller via `set_param`. Used by `Chorus`/`Flanger`.
pub fn new_pooled_node(id: u16, kind: Kind, handle: Option<deluge_audio_graph::PoolHandle>, input: Input) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind,
        args: [input, Input::Const(0.0), Input::Const(0.0)],
    });
    if let Some(h) = handle {
        host().audio_cmd(Cmd::BindTable {
            node: NodeId(id),
            src: deluge_audio_graph::node::TableSrc::Pooled(h),
        });
    }
}
```

- [ ] **Step 4: Add the factories, setters, and const**

In `crates/deluge-wren-core/src/bindings_audio.rs`, add near the `DELAY_MAX_SAMPLES` const:

```rust
/// Ring length for a chorus/flanger node (~50 ms @ 48 kHz). Covers `base·(1+depth)`
/// for both effects (chorus base 20 ms, flanger 2 ms) plus headroom.
pub(crate) const CHORUS_BUF_SAMPLES: usize = 2400;
```

Add the factories (near `node_delay_impl`):

```rust
/// `Node.chorus_(input, rate, depth, mix)` — allocate a ring, create a width-2
/// `Kind::Chorus` node, and set its rate/depth/mix params from the args. A
/// host with no pool leaves it unbound (dry passthrough).
pub(crate) fn node_chorus_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let rate = vm.get_f(2) as f32;
    let depth = vm.get_f(3) as f32;
    let mix = vm.get_f(4) as f32;
    let handle = audio::alloc_buffer(CHORUS_BUF_SAMPLES);
    let id = audio::alloc_node_id();
    audio::new_pooled_node(id, Kind::Chorus, handle, input);
    audio::set_param(id, 1, rate);
    audio::set_param(id, 2, depth);
    audio::set_param(id, 0, mix);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_chorus(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_chorus_impl(&vm);
}

/// `Node.flanger_(input, rate, depth, feedback, mix)` — like `node_chorus_impl`
/// but `Kind::Flanger` (single voice) with a feedback (param 3) arg.
pub(crate) fn node_flanger_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let rate = vm.get_f(2) as f32;
    let depth = vm.get_f(3) as f32;
    let feedback = vm.get_f(4) as f32;
    let mix = vm.get_f(5) as f32;
    let handle = audio::alloc_buffer(CHORUS_BUF_SAMPLES);
    let id = audio::alloc_node_id();
    audio::new_pooled_node(id, Kind::Flanger, handle, input);
    audio::set_param(id, 1, rate);
    audio::set_param(id, 2, depth);
    audio::set_param(id, 3, feedback);
    audio::set_param(id, 0, mix);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_flanger(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_flanger_impl(&vm);
}
```

Add the setters (near `node_set_mix_impl`):

```rust
pub(crate) fn node_set_rate_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 1 = rate (Kind::Chorus/Flanger)
    audio::set_param(self_id(vm), 1, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_rate(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_rate_impl(&vm);
}

pub(crate) fn node_set_depth_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 2 = depth
    audio::set_param(self_id(vm), 2, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_depth(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_depth_impl(&vm);
}

// `regen=` (NOT `feedback=`, which is the Osc's set_param(0)): flanger feedback.
pub(crate) fn node_set_regen_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 3 = feedback
    audio::set_param(self_id(vm), 3, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_regen(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_regen_impl(&vm);
}
```

Register in `register_audio` (bindings_audio.rs):

```rust
    method("main", "Node", true, "chorus_(_,_,_,_)", node_chorus_impl::<S>);
    method("main", "Node", true, "flanger_(_,_,_,_,_)", node_flanger_impl::<S>);
    method("main", "Node", false, "rate=(_)", node_set_rate_impl::<S>);
    method("main", "Node", false, "depth=(_)", node_set_depth_impl::<S>);
    method("main", "Node", false, "regen=(_)", node_set_regen_impl::<S>);
```

- [ ] **Step 5: Register in the wren-sys table + prelude**

In `crates/deluge-wren-core/src/bindings.rs`, add near the other audio registrations:

```rust
    static_method("Node", "chorus_(_,_,_,_)", bindings_audio::node_chorus),
    static_method("Node", "flanger_(_,_,_,_,_)", bindings_audio::node_flanger),
    method("Node", "rate=(_)", bindings_audio::node_set_rate),
    method("Node", "depth=(_)", bindings_audio::node_set_depth),
    method("Node", "regen=(_)", bindings_audio::node_set_regen),
```

In `crates/deluge-wren-core/wren/prelude.wren`, add to `foreign class Node`:

```wren
  foreign static chorus_(input, rate, depth, mix)
  foreign static flanger_(input, rate, depth, feedback, mix)
  foreign rate=(v)
  foreign depth=(v)
  foreign regen=(v)     // Flanger feedback (NOT feedback= — that's the Osc's)
```

And the sugar classes (after `class Delay` / `class Resonator`):

```wren
// Chorus — a lush multi-voice stereo modulated delay. `rate` LFO Hz, `depth`
// [0,1] sweep, `mix` dry/wet. Stereo (width-2); patch it straight out:
//   Out.patch(Chorus.new(Osc.saw(110), 0.5, 0.4, 0.5))
//   var c = Chorus.new(pad, 0.3, 0.6, 0.5); c.rate = 0.8; c.depth = 0.7
class Chorus {
  static new(input, rate, depth, mix) { Node.chorus_(input, rate, depth, mix) }
}

// Flanger — a swept single-voice comb with feedback (`regen`). Short delay,
// jet-sweep. `rate`/`depth` as chorus; `feedback` [0,0.95] is the resonance:
//   Out.patch(Flanger.new(Osc.saw(110), 0.3, 0.7, 0.6, 0.5))
//   var f = Flanger.new(pad, 0.2, 0.8, 0.7, 0.5); f.regen = 0.8
class Flanger {
  static new(input, rate, depth, feedback, mix) { Node.flanger_(input, rate, depth, feedback, mix) }
}
```

- [ ] **Step 6: Run the new tests + full suite**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --test audio_bindings chorus flanger`
Expected: PASS (all 4).

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core`
Expected: PASS — existing bindings (Delay `mix=`, Resonator, Pan, wavetable) unchanged, no warnings.

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): Chorus.new / Flanger.new stereo effect surface"
```

---

## Scope notes (deliberate deferrals)

- **Stereo flanger via quadrature L/R LFO** — `ModDelay<1>` pans its one voice to center (mono-in-both-channels); a true stereo flanger is deferred.
- **Ef-3 Reverb** — the `DelayLine::allpass` diffuser (the stub deferred from Ef-1) + an FDN/Dattorro network land there.
- **Tempo-sync, mid-side/width, per-voice detune, LFO waveform choice** — later refinements.

## Post-implementation

After all three tasks pass, use **superpowers:finishing-a-development-branch** to verify the suites and merge.
