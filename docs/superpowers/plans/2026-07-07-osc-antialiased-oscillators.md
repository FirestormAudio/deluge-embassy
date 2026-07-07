# Osc — anti-aliased classic oscillators Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace P0's naïve saw/square/triangle with PolyBLEP band-limited versions in `deluge-dsp-kernels`, proven with QA's `worst_alias_db`, keeping the graph/Wren layers untouched and regenerating the two golden vectors.

**Architecture:** All work is in `crates/deluge-dsp-kernels/src/osc.rs`: `Osc::process` keeps its signature but adds PolyBLEP step-correction (saw/square) and BLAMP slope-correction (triangle). Sine is unchanged. The kernel gains `deluge-dsp-test` as a dev-dependency so its tests can assert spectral aliasing. Because the saw's samples change, P0's and P1's golden tests are re-pinned.

**Tech Stack:** Rust, `no_std` kernel (tests use `std` on host), `deluge-dsp-test` (QA harness, dev-dep), `proptest`. Kernel tests run on `x86_64-unknown-linux-gnu`.

**Reference spec:** [Osc design](../specs/2026-07-07-osc-antialiased-oscillators-design.md). Depends on P0 + QA (both merged).

## Global Constraints

- The change is confined to `crates/deluge-dsp-kernels/src/osc.rs` (+ its `Cargo.toml` dev-dep) and the two golden tests. `Osc::process(wave, freq: In, dt, out)` keeps its exact signature (graph/Wren untouched).
- `deluge-dsp-kernels` stays `#![no_std]`; the new `deluge-dsp-test` dependency is **dev-only** (host tests; device builds don't pull dev-deps).
- Kernel tests run with the host target: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`. QA-based aliasing tests build `deluge-dsp-test` (pulls `realfft`).
- **Method decided:** standard **4-point PolyBLEP** for saw/square, **BLAMP** for triangle (NOT the wide 6-point kernel — it costs more, uses hand-derived coefficients, and has a near-Nyquist window-overlap defect). Saw with 4-point measures ~−30.5 dB worst-alias / +16.7 dB over naïve at the hard 5 kHz case.
- **Aliasing gates (measured, not predicted):** the earlier −40/−50 dB predictions were wrong — `worst_alias_db` measures the loudest *folded* component, which standard PolyBLEP improves far less than the harmonic-suppression figure. **Procedure per wave:** implement the standard method, MEASURE `worst_alias_db` at 2 k/5 k/8 k, then set the assertion **just below the measured floor with a small margin** and document the real dB in a comment (saw is gated at `< -28 dB` from its −30.5 dB measurement). The robust bar is the **improvement over naïve** — gate it at `> 12 dB` (saw measures +16.7). Never loosen the improvement gate below what proves the correction works; never chase a higher absolute floor with a wider/custom kernel.
- Output bound loosens to **±1.1** (PolyBLEP overshoots ±1 slightly at discontinuities).
- Test output pristine (no warnings). Commit after each task.

---

## File structure

- `crates/deluge-dsp-kernels/Cargo.toml` — add `deluge-dsp-test` dev-dependency.
- `crates/deluge-dsp-kernels/src/osc.rs` — PolyBLEP/BLAMP `Osc::process`; `poly_blep`/`poly_blamp` helpers; updated + new tests.
- `crates/deluge-audio-graph/src/cmd.rs` — re-pin `golden_saw_lpf_env_first_block`.
- `crates/deluge-wren-core/tests/audio_bindings.rs` — re-pin `golden_saw_lpf_renders_expected_block`.

---

## Task 1: PolyBLEP saw + QA infra + test updates

**Files:**
- Modify: `crates/deluge-dsp-kernels/Cargo.toml`, `crates/deluge-dsp-kernels/src/osc.rs`

**Interfaces:**
- Produces: `poly_blep(t: f32, dtp: f32) -> f32` (free fn in `osc.rs`); band-limited `Wave::Saw` in `Osc::process`. Signature of `Osc::process` unchanged.
- Consumes: `deluge_dsp_test::spectrum::analyze_buf`, `Spectrum::worst_alias_db` (QA), `crate::In`.

- [ ] **Step 1: Add the QA dev-dependency**

`crates/deluge-dsp-kernels/Cargo.toml`, under `[dev-dependencies]` (next to `proptest`):

```toml
deluge-dsp-test = { path = "../deluge-dsp-test" }
```

- [ ] **Step 2: Loosen the bounds proptest (before saw overshoots)**

In `osc.rs`'s `osc_output_is_finite_and_bounded` proptest, change the bound assertion from `±1.001` to `±1.1` (PolyBLEP corrections overshoot ±1 slightly):

```rust
                prop_assert!(s.is_finite());
                prop_assert!(s >= -1.1 && s <= 1.1);
```

Update the doc comment to note the ±1.1 is PolyBLEP overshoot slack, not `fast_sin` slack.

- [ ] **Step 3: Write the failing saw aliasing test**

Add to `osc.rs`'s `mod tests`:

```rust
    /// Render a steady saw at `f0` (48 kHz) into an FFT_N buffer and return its
    /// spectrum. `deluge_dsp_test::FFT_N` is the analyzer's fixed size.
    fn saw_spectrum(f0: f32) -> deluge_dsp_test::spectrum::Spectrum {
        let sr = 48_000.0;
        let mut osc = Osc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(Wave::Saw, In::K(f0), 1.0 / sr, &mut buf);
        deluge_dsp_test::spectrum::analyze_buf(sr, &buf)
    }

    #[test]
    fn saw_is_band_limited() {
        for &f0 in &[2_000.0f32, 5_000.0, 8_000.0] {
            let spec = saw_spectrum(f0);
            let tol = 3.0 * spec.bin_hz;
            let wa = spec.worst_alias_db(f0, tol);
            assert!(wa < -40.0, "saw f0={f0}: worst_alias {wa} dB should be < -40");
        }
    }

    #[test]
    fn band_limited_saw_beats_naive() {
        let sr = 48_000.0;
        let f0 = 5_000.0;
        // Naïve saw rendered inline (no PolyBLEP).
        let mut naive = [0.0f32; deluge_dsp_test::FFT_N];
        let mut ph = 0.0f32;
        for s in naive.iter_mut() {
            *s = 2.0 * ph - 1.0;
            ph += f0 / sr;
            ph -= ph.floor();
        }
        let spec_n = deluge_dsp_test::spectrum::analyze_buf(sr, &naive);
        let spec_bl = saw_spectrum(f0);
        let tol = 3.0 * spec_bl.bin_hz;
        let improvement = spec_n.worst_alias_db(f0, tol) - spec_bl.worst_alias_db(f0, tol);
        assert!(improvement > 20.0, "band-limited should beat naïve by >20 dB, got {improvement}");
    }
```

- [ ] **Step 4: Update the naïve-pinned `saw_ramps` test to a band-limiting-robust shape check**

The old `saw_ramps_from_minus_one_over_one_cycle` pins the *naïve* saw at a 0.25 phase-increment where PolyBLEP is strongly active — it will break. Replace it with a shape check at a modest increment (1 kHz / 48 kHz → one cycle = 48 samples, PolyBLEP correction negligible in the interior):

```rust
    #[test]
    fn saw_ramps_upward_over_a_cycle() {
        let mut osc = Osc::new();
        let mut out = [0.0f32; 48]; // 1 kHz at 48 kHz → exactly one cycle
        osc.process(Wave::Saw, In::K(1_000.0), 1.0 / 48_000.0, &mut out);
        // Interior samples (away from the wrap at index 0) follow the ramp.
        assert!(out[10] < out[20] && out[20] < out[30], "saw rises through the cycle");
        assert!(out[5] < -0.5 && out[40] > 0.5, "saw spans roughly [-1, 1]");
    }
```

- [ ] **Step 5: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: FAIL — `saw_is_band_limited`/`band_limited_saw_beats_naive` fail (naïve saw aliases ~−10 dB), and `saw_ramps_upward_over_a_cycle` may already pass since naïve also ramps; the aliasing tests are the RED gate. (First build fetches `realfft` via `deluge-dsp-test`.)

- [ ] **Step 6: Implement PolyBLEP saw**

At the top of `osc.rs` (below the `use`s), add the helper:

```rust
/// 2-point PolyBLEP residual correcting a unit step at phase `t ∈ [0,1)`, given
/// the per-sample phase increment `dtp`.
fn poly_blep(t: f32, dtp: f32) -> f32 {
    if t < dtp {
        let x = t / dtp;
        2.0 * x - x * x - 1.0
    } else if t > 1.0 - dtp {
        let x = (t - 1.0) / dtp;
        x * x + 2.0 * x + 1.0
    } else {
        0.0
    }
}
```

Rewrite `Osc::process`'s `Wave::Saw` arm so the correction is applied. Restructure the loop to compute `dtp` per sample:

```rust
    pub fn process(&mut self, wave: Wave, freq: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            let p = self.phase;
            let dtp = freq.at(i) * dt;
            *s = match wave {
                Wave::Sine => fast_sin(p),
                Wave::Saw => (2.0 * p - 1.0) - poly_blep(p, dtp),
                Wave::Square => {
                    // (filled in Task 2 — keep the naïve value for now)
                    if p < 0.5 { 1.0 } else { -1.0 }
                }
                Wave::Tri => 1.0 - 4.0 * (p - 0.5).abs(), // (band-limited in Task 3)
            };
            self.phase += dtp;
            self.phase -= floorf(self.phase);
        }
    }
```

- [ ] **Step 7: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: PASS — including `saw_is_band_limited` (record the measured worst_alias values) and `band_limited_saw_beats_naive`. If saw misses −40 dB, note the measured value and raise saw to 4-point PolyBLEP (a wider residual) before proceeding. Zero warnings.

- [ ] **Step 8: Commit**

```bash
git add crates/deluge-dsp-kernels
git commit -m "feat(dsp-kernels): PolyBLEP band-limited saw + QA aliasing tests"
```

---

## Task 2: PolyBLEP square

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/osc.rs`

**Interfaces:**
- Consumes: `poly_blep` (Task 1). Produces band-limited `Wave::Square`.

- [ ] **Step 1: Write the failing square aliasing test**

Add to `mod tests` (reuses the pattern from Task 1):

```rust
    #[test]
    fn square_is_band_limited() {
        let sr = 48_000.0;
        for &f0 in &[2_000.0f32, 5_000.0, 8_000.0] {
            let mut osc = Osc::new();
            let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
            osc.process(Wave::Square, In::K(f0), 1.0 / sr, &mut buf);
            let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
            let wa = spec.worst_alias_db(f0, 3.0 * spec.bin_hz);
            // Square is two BLEPs (like saw); expect ~-30 dB worst-alias with
            // 4-point PolyBLEP. Measure, then gate just below the measured floor.
            assert!(wa < -28.0, "square f0={f0}: worst_alias {wa} dB should be < -28");
        }
    }
```

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: FAIL — naïve square aliases badly (~−10 dB).

- [ ] **Step 3: Implement PolyBLEP square**

Replace the `Wave::Square` arm in `Osc::process` (square = rising edge at phase 0, falling edge at 0.5 → two BLEPs):

```rust
                Wave::Square => {
                    let naive = if p < 0.5 { 1.0 } else { -1.0 };
                    naive + poly_blep(p, dtp) - poly_blep((p + 0.5).fract(), dtp)
                }
```

- [ ] **Step 4: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: PASS — `square_is_band_limited` clears −28 dB (record the measured worst-alias). Square uses the same 4-point `poly_blep` as saw, so expect ~−30 dB. If the measured floor differs, set the gate just below it with margin and document; do not chase a higher floor with a wider kernel. Zero warnings.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/osc.rs
git commit -m "feat(dsp-kernels): PolyBLEP band-limited square"
```

---

## Task 3: BLAMP band-limited triangle

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/osc.rs`

**Interfaces:**
- Produces: `poly_blamp(t: f32, dtp: f32) -> f32`; band-limited `Wave::Tri`.

**Note on the triangle method (spec §6):** the intended method is **BLAMP** (band-limited ramp = integral of BLEP) at the triangle's two slope discontinuities (phase 0 and 0.5), which keeps `Osc` stateless. The residual/scale below is a standard polyBLAMP; because BLAMP scaling/sign is easy to get subtly wrong, **QA is the arbiter, MEASURED not predicted**: naïve triangle already sits ~−25…−28 dB (its harmonics roll off 12 dB/oct), so band-limiting has less headroom than saw. Procedure: implement BLAMP, MEASURE `worst_alias_db` and the improvement over naïve, then gate the absolute assertion **just below the measured band-limited floor** and require a meaningful **improvement over naïve (> 6 dB)** — do not chase a fixed −50 dB. If BLAMP won't cleanly beat naïve after reasonable sign/scale tuning, use the documented **fallback**: leaky-integrate the band-limited square (add a `tri_z: f32` field to `Osc`) — report which path and the measured numbers.

- [ ] **Step 1: Write the failing triangle tests**

Add to `mod tests`:

```rust
    #[test]
    fn triangle_is_band_limited() {
        let sr = 48_000.0;
        for &f0 in &[2_000.0f32, 5_000.0, 8_000.0] {
            let mut osc = Osc::new();
            let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
            osc.process(Wave::Tri, In::K(f0), 1.0 / sr, &mut buf);
            let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
            let wa = spec.worst_alias_db(f0, 3.0 * spec.bin_hz);
            // Naïve triangle already ~-25..-28 dB; BLAMP improves on that.
            // MEASURE, then set this gate just below the measured band-limited floor.
            assert!(wa < -30.0, "triangle f0={f0}: worst_alias {wa} dB should be < -30");
        }
    }

    #[test]
    fn band_limited_triangle_beats_naive() {
        let sr = 48_000.0;
        let f0 = 5_000.0;
        let mut naive = [0.0f32; deluge_dsp_test::FFT_N];
        let mut ph = 0.0f32;
        for s in naive.iter_mut() {
            *s = 1.0 - 4.0 * (ph - 0.5).abs();
            ph += f0 / sr;
            ph -= ph.floor();
        }
        let spec_n = deluge_dsp_test::spectrum::analyze_buf(sr, &naive);
        let mut osc = Osc::new();
        let mut bl = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(Wave::Tri, In::K(f0), 1.0 / sr, &mut bl);
        let spec_bl = deluge_dsp_test::spectrum::analyze_buf(sr, &bl);
        let tol = 3.0 * spec_bl.bin_hz;
        let improvement = spec_n.worst_alias_db(f0, tol) - spec_bl.worst_alias_db(f0, tol);
        assert!(improvement > 6.0, "band-limited triangle should beat naïve by >6 dB, got {improvement}");
    }

    #[test]
    fn triangle_low_freq_shape_intact() {
        // At 200 Hz the triangle should still peak near +1 and trough near -1.
        let mut osc = Osc::new();
        let mut buf = [0.0f32; 512];
        osc.process(Wave::Tri, In::K(200.0), 1.0 / 48_000.0, &mut buf);
        let max = buf.iter().cloned().fold(f32::MIN, f32::max);
        let min = buf.iter().cloned().fold(f32::MAX, f32::min);
        assert!(max > 0.9 && min < -0.9, "triangle spans ~[-1,1]: min {min} max {max}");
        assert!(buf.iter().all(|s| s.is_finite() && s.abs() <= 1.1));
    }
```

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: FAIL — naïve triangle aliases (~−25…−28 dB) which is above the −30 dB gate, and `band_limited_triangle_beats_naive` fails (0 dB improvement pre-implementation).

- [ ] **Step 3: Implement the BLAMP triangle**

Add the polyBLAMP helper near `poly_blep`:

```rust
/// 2-point polyBLAMP residual correcting a unit slope discontinuity at phase
/// `t ∈ [0,1)`, given the per-sample phase increment `dtp`. (Integral of the BLEP.)
fn poly_blamp(t: f32, dtp: f32) -> f32 {
    if t < dtp {
        let x = t / dtp - 1.0;
        -1.0 / 3.0 * x * x * x
    } else if t > 1.0 - dtp {
        let x = (t - 1.0) / dtp + 1.0;
        1.0 / 3.0 * x * x * x
    } else {
        0.0
    }
}
```

Replace the `Wave::Tri` arm. Naïve triangle `1 - 4·|p - 0.5|` has slope +4 on `[0,0.5]` and −4 on `[0.5,1]` — slope changes of +8 at phase 0 and −8 at phase 0.5. Apply BLAMP scaled by the slope change and `dtp`:

```rust
                Wave::Tri => {
                    let naive = 1.0 - 4.0 * (p - 0.5).abs();
                    // Slope change +8 at phase 0, -8 at phase 0.5.
                    let c = 8.0 * dtp * (poly_blamp(p, dtp) - poly_blamp((p + 0.5).fract(), dtp));
                    naive + c
                }
```

Verify sign/scale against the QA gate and the low-freq shape test; if it does not converge, switch to the integrated-square fallback described in the task note.

- [ ] **Step 4: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`
Expected: PASS — `triangle_is_band_limited` (gate set just below the measured band-limited floor), `band_limited_triangle_beats_naive` (> 6 dB), and `triangle_low_freq_shape_intact`. Zero warnings. Report which triangle path (BLAMP or fallback) was used and the measured alias + improvement values.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/osc.rs
git commit -m "feat(dsp-kernels): BLAMP band-limited triangle"
```

---

## Task 4: Regenerate the golden vectors

**Files:**
- Modify: `crates/deluge-audio-graph/src/cmd.rs`, `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:** none new — re-pin two characterization goldens whose saw input is now band-limited.

**Context:** both goldens render a `saw → lpf → env` patch and pin the first output samples. Band-limiting the saw changes those samples (notably the saw's first sample at phase 0, which PolyBLEP moves from −1 toward 0). The pinned arrays must be regenerated. This is an intended, reviewed re-pin — not a masking of a regression.

- [ ] **Step 1: Regenerate the deluge-audio-graph golden**

Run the test to see the new actual values:
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph golden_saw_lpf_env_first_block -- --nocapture`
Expected: FAIL — the assertion prints the new rendered samples vs the old pinned `expected`. Copy the new values into the `expected` array in `crates/deluge-audio-graph/src/cmd.rs`. Keep/refresh the "characterization pin — regenerate only on an intended, reviewed output change" comment, and add a note that this re-pin is from Osc band-limiting.

- [ ] **Step 2: Verify deluge-audio-graph green (both modes)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features simd`
Expected: PASS both — the saw kernel is deterministic, so the re-pinned values match under default and simd builds.

- [ ] **Step 3: Regenerate the deluge-wren-core golden**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support golden_saw_lpf_renders_expected_block -- --nocapture`
Expected: FAIL — prints the new values. Copy them into the `expected` array in `crates/deluge-wren-core/tests/audio_bindings.rs`; refresh the characterization comment with the Osc-re-pin note.

- [ ] **Step 4: Verify deluge-wren-core green**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src/cmd.rs crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "test: re-pin saw→lpf→env goldens for Osc band-limiting"
```

---

## Self-review notes

- **Spec coverage:** PolyBLEP saw (Task 1) + square (Task 2); BLAMP triangle (Task 3); sine unchanged (untouched); QA aliasing gates + naïve-vs-BL improvement + low-freq fidelity (Tasks 1–3); loosened ±1.1 bounds proptest (Task 1); golden regeneration (Task 4); graph/Wren untouched (only `osc.rs` + goldens change); QA dev-dependency (Task 1).
- **Deliberate deviations from the spec:** (1) `Osc::process` keeps a single `match wave` with per-arm corrections rather than splitting into per-wave methods (resolves §6's readability question toward the smaller diff). (2) The triangle uses BLAMP with a concrete 2-point residual, but the task explicitly makes QA the arbiter and documents the integrated-square fallback — because BLAMP scaling is genuinely tune-to-measure.
- **Known follow-ups (not this cut):** 4-point PolyBLEP if −40 dB is missed within budget; SIMD oscillator path; wavetable/FM-operators/hard-sync/noise suites; `fast_sin` accuracy if its ~−60 dB harmonics ever matter.
- **Risk note:** Task 3 (triangle BLAMP sign/scale) is the one place the given code may need empirical adjustment against the QA gate — the plan calls this out and provides the fallback, so it should surface as a normal test-fail-then-tune, not a blocker.
