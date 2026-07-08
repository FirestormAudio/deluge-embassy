# Noise variants — pink & brown Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Add pink (−3 dB/oct, Kellet filter) and brown (−6 dB/oct, leaky integrator) noise alongside white, with white output byte-identical and each color's spectral slope QA-gated.

**Architecture:** `Noise` gains a `NoiseColor` + per-color filter/integrator state; `process` branches on color (white path unchanged). Two new node `Kind`s map to the colored `Noise` (render arm unchanged). Wren gains `Osc.pink()`/`Osc.brown()`. A new `deluge-dsp-test` spectral-slope metric gates the dB/oct.

**Tech Stack:** Rust `no_std` (`deluge-dsp-kernels`), `deluge-audio-graph`, `deluge-wren-core`; QA via `deluge-dsp-test` (realfft). Host target `x86_64-unknown-linux-gnu`.

**Reference spec:** [noise variants](../specs/2026-07-08-noise-variants-design.md).

## Global Constraints

- **White byte-identical:** `Kind::Noise` / `Noise::seeded` output equals today's xorshift32 white exactly (existing noise proptest/determinism unchanged; a direct reference-block comparison).
- **Measured, not predicted:** `PINK_GAIN`, brown constants (`BROWN_RATE`/`BROWN_LEAK`/`BROWN_GAIN`), the slope tolerances, and the peak/RMS bounds are all set from measurement.
- `no_std`; deterministic (seeded, xorshift only); zero warnings.
- Slope metric averages over multiple blocks (noise is stochastic) — report estimate + spread.
- Test commands: `-p deluge-dsp-kernels`, `-p deluge-audio-graph` (+`--features simd`), `-p deluge-wren-core --features test-support`, `-p wren-firmware` compile check (device target, both audio modes).

## Reference values

- `noise.rs`: `Noise { rng: u32 }`, `Noise::seeded(seed) -> Noise` (`rng = if seed==0 {0x2545_F491} else {seed}`), `process(&mut self, out: &mut [f32])` (per sample: xorshift32 `r^=r<<13; r^=r>>17; r^=r<<5;`, `out = (r as i32 as f32)/(i32::MAX as f32)`). Noise proptest asserts finite + `[-1,1]`.
- `node.rs`: `Kind::Noise` → `State::Noise(Noise::seeded(0x2545_F491))`; render arm `if let State::Noise(nz) = ... { nz.process(outs.port(0)) }`.
- Wren: `node_noise_impl` → `Kind::Noise` (static `Node.noise_()`); `Osc.noise()` prelude sugar. Registration 4-place.
- `deluge-dsp-test`: `spectrum::analyze_buf(sr, &[f32; FFT_N]) -> Spectrum { bins, bin_hz, sample_rate }`, `FFT_N=8192`.

---

## Task 1: Kernel colors + slope metric + measure constants

**Files:** Modify `crates/deluge-dsp-kernels/src/noise.rs`; add `slope_db_per_octave` to `crates/deluge-dsp-test/src/spectrum.rs`.

**Interfaces:**
- Produces: `NoiseColor { White, Pink, Brown }`, `Noise::seeded_color(seed, color)`, colored `process`. `deluge_dsp_test::spectrum::slope_db_per_octave(&Spectrum, f_lo, f_hi) -> f32` (least-squares dB vs log2(freq) over octave bands in `[f_lo, f_hi]`).

- [ ] **Step 1: Add the slope metric to `deluge-dsp-test`**

In `crates/deluge-dsp-test/src/spectrum.rs`, add:
```rust
/// Least-squares slope (dB/octave) of the magnitude spectrum over [f_lo, f_hi],
/// averaging bins into octave bands to reduce stochastic variance. Positive = rising.
pub fn slope_db_per_octave(spec: &Spectrum, f_lo: f32, f_hi: f32) -> f32 {
    // For each octave band center f, mean magnitude of bins in [f/√2, f·√2];
    // x = log2(f), y = 20*log10(mean_mag); return least-squares slope of (x,y).
}
```
Unit-test it with a synthetic spectrum of known slope (e.g. a `1/f` magnitude → −6 dB/oct... actually 1/f magnitude = −6 dB/oct in power? clarify: magnitude 1/f → 20log10 drops 6 dB per octave; power 1/f → −3 dB/oct. Use MAGNITUDE convention consistently and document it; verify the metric returns the expected slope on a hand-built spectrum).

- [ ] **Step 2: Write failing noise-color tests**

In `noise.rs` tests (uses `deluge-dsp-test` dev-dep — confirm it's a dev-dep, add if not):
```rust
    fn capture(color: NoiseColor, blocks: usize) -> deluge_dsp_test::spectrum::Spectrum {
        // average |spectrum| over `blocks` independent FFT_N captures for a stable slope.
        // (Accumulate magnitude across blocks; build one averaged Spectrum.)
    }

    #[test]
    fn white_unchanged() {
        let mut w = Noise::seeded(12345);
        let mut a = [0.0f32; 64]; w.process(&mut a);
        let mut w2 = Noise::seeded_color(12345, NoiseColor::White);
        let mut b = [0.0f32; 64]; w2.process(&mut b);
        assert_eq!(a, b); // colored White == plain seeded white, bit-exact
        // and both equal the pre-change output (compare to a saved reference block).
    }

    #[test]
    fn slopes_match_colors() {
        let sr = 48_000.0f32;
        let wslope = deluge_dsp_test::spectrum::slope_db_per_octave(&capture(NoiseColor::White, 32), 200.0, 12_000.0);
        let pslope = deluge_dsp_test::spectrum::slope_db_per_octave(&capture(NoiseColor::Pink, 32), 200.0, 12_000.0);
        let bslope = deluge_dsp_test::spectrum::slope_db_per_octave(&capture(NoiseColor::Brown, 32), 200.0, 12_000.0);
        // gates set from measurement (Step 5) — start loose:
        assert!(wslope.abs() < 1.5, "white ~0, got {wslope}");
        assert!((pslope - (-3.0)).abs() < 1.5, "pink ~-3, got {pslope}");
        assert!((bslope - (-6.0)).abs() < 1.5, "brown ~-6, got {bslope}");
    }

    proptest! {
        #[test]
        fn all_colors_bounded(seed in 1u32..=u32::MAX, c in 0u8..3) {
            let color = match c { 0 => NoiseColor::White, 1 => NoiseColor::Pink, _ => NoiseColor::Brown };
            let mut n = Noise::seeded_color(seed, color);
            let mut out = [0.0f32; 256];
            n.process(&mut out);
            for s in out { prop_assert!(s.is_finite() && s.abs() <= 1.0); }
        }
    }
```

- [ ] **Step 3: Run, verify fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -p deluge-dsp-test`. Expected: FAIL (`NoiseColor`/`seeded_color`/`slope_db_per_octave` undefined).

- [ ] **Step 4: Implement colors (spec §2 code)**

Add `NoiseColor`, extend `Noise` struct (`color`, `pink: [f32;7]`, `brown: f32`), `seeded` (White, unchanged output), `seeded_color`. `process` draws `white` (current xorshift), branches: White → `white`; Pink → the Kellet refined filter (spec §2) `* PINK_GAIN`; Brown → leaky integrator (spec §2) with clamp. Add `const PINK_GAIN`, `BROWN_RATE`, `BROWN_LEAK`, `BROWN_GAIN` (placeholder values, tuned in Step 5).

- [ ] **Step 5: Run, MEASURE + tune constants + tighten gates**

Run the tests. MEASURE: the pink & brown slopes over `[200, 12000]` Hz (averaged over 32 blocks) and the peak/RMS of each color. Tune `PINK_GAIN` so pink peaks near `±1` without frequent clamping; tune `BROWN_RATE`/`BROWN_LEAK`/`BROWN_GAIN` so brown's slope is ≈−6 dB/oct AND it stays bounded (leak sets the low corner + bound). Re-measure slopes; TIGHTEN the `slopes_match_colors` gates to just around the measured values (e.g. `< 0.7` from target if the estimate is stable — document the measured slope + spread). Record all four constants + the measured slopes/peaks in comments. Do NOT loosen a gate to pass a wrong constant.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-dsp-kernels/src/noise.rs crates/deluge-dsp-test/src/spectrum.rs
git commit -m "feat(noise): pink (Kellet) + brown (leaky integrator) + spectral-slope QA metric"
```

---

## Task 2: Graph — `Kind::PinkNoise` / `Kind::BrownNoise`

**Files:** Modify `crates/deluge-audio-graph/src/node.rs`.

- [ ] **Step 1: Failing test** — create a `Kind::PinkNoise` node, render, assert bounded + non-silent (and, if easy, the slope; else just bounded/non-silent — the slope is gated in Task 1's kernel test).
```rust
    #[test]
    fn pink_brown_nodes_render_bounded() {
        for k in [Kind::PinkNoise, Kind::BrownNoise] {
            let mut e = E::new(48_000.0);
            e.create(NodeId(0), k);
            e.render_block();
            let out = e.node_output(NodeId(0), 0);
            assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.0));
            assert!(out.iter().any(|&s| s != 0.0));
        }
    }
```

- [ ] **Step 2: Add the kinds + mapping** — `Kind::PinkNoise`, `Kind::BrownNoise` in `enum Kind`; `Node::new` maps them to `State::Noise(Noise::seeded_color(0x2545_F491, NoiseColor::Pink|Brown))`. `Kind::Noise` stays White. Render arm unchanged (`nz.process` dispatches on color). `out_width` default 1.

- [ ] **Step 3: Verify (both modes) + transparency** — `cargo test -p deluge-audio-graph` and `--features simd`: new test + existing `Kind::Noise` (white) tests unchanged, both goldens unchanged. Zero warnings.

- [ ] **Step 4: Commit** — `feat(audio-graph): Kind::PinkNoise + Kind::BrownNoise`.

---

## Task 3: Wren — `Osc.pink()` / `Osc.brown()`

**Files:** Modify `crates/deluge-wren-core/src/{bindings_audio.rs,bindings.rs}`, `wren/prelude.wren`, `tests/audio_bindings.rs`.

- [ ] **Step 1: Failing Cmd-sequence tests** — `Osc.pink()` emits `NewNode{kind: Kind::PinkNoise}`; `Osc.brown()` → `Kind::BrownNoise`.

- [ ] **Step 2: Implement** — extend the existing white-noise binding: either `node_noise_impl` takes a color code (`Node.noise_(color)`) with `Osc.noise()/pink()/brown()` passing 0/1/2, OR add `node_pink_impl`/`node_brown_impl` static factories. Mirror `node_noise_impl`'s exact pattern (impl + `#[cfg(wren-sys-backend)]` extern wrapper, `register_audio`, `METHODS` table, prelude). Whichever, keep `Osc.noise()` (white) unchanged. Add `Osc.pink()`/`Osc.brown()` prelude sugar.

- [ ] **Step 3: Verify + sys-backend** — `cargo test -p deluge-wren-core --features test-support` (new Cmd tests + a render test asserting bounded/non-silent; goldens unchanged); `cargo build -p deluge-wren-core --no-default-features --features wren-sys-backend`; firmware compiles both modes. Zero warnings. Confirm 4-place registration for any new methods.

- [ ] **Step 4: Commit** — `feat(wren-core): Osc.pink() / Osc.brown() noise colors`.

---

## Self-review notes

- **Spec coverage:** colors + Kellet/brown + slope metric + measurement (Task 1); node kinds (Task 2); Wren surface (Task 3). QA: slope gates + bounds + white-byte-identical + determinism (Task 1), node render (Task 2), Cmd-sequence (Task 3).
- **White transparency:** `seeded` output unchanged (same xorshift); `white_unchanged` test pins it bit-exact; existing noise proptest/determinism unchanged.
- **Measured, not predicted:** `PINK_GAIN`, brown constants, and the slope-gate tolerances all set from the Step-5 measurement.
- **Slope metric convention:** magnitude (20log10) vs frequency-octave; documented + unit-tested on a synthetic spectrum so the noise gates are meaningful.
- **Known follow-ups:** blue/violet/grey; continuous color knob; hard sync (next).
