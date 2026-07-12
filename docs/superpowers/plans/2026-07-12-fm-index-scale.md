# FM Index via `.scale(k)` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the poly-FM feature a clean way to set a modulation INDEX — `carrier.pm = modulator.scale(3)` — by making the Wren `scale` helper audio-safe (it currently aborts on audio nodes inside a Synth).

**Architecture:** Pure Wren prelude change, zero Rust. The Synth-DSL `*`/`+` operators abort on `audio-node * Num` but NOT on `audio-node * <control node>`; `Node.ctrl_(k)` wraps a constant into a broadcastable control node. So `this * Node.ctrl_(k)` scales an audio signal safely and the engine broadcasts the constant across voices. Add a one-arg `scale(k)` and fix the dead two-arg `scale(m, a)` to route through `ctrl_`, on both `Node` and `Port`.

**Tech Stack:** Wren prelude (`crates/deluge-wren-core/wren/prelude.wren`); the `run_and_render`/`run_script_ok` test harness in `crates/deluge-wren-core/tests/audio_bindings.rs`.

## Global Constraints

- Prelude-only. NO Rust/kernel/graph/`bindings_audio.rs` change. No new foreign method.
- `no_std` unaffected (prelude is Wren text loaded at runtime). MIT/Apache-2.0.
- Tests pass in BOTH configs (default + `--features deluge-dsp-kernels/simd`).
- **The `*`/`+` Num-guard stays untouched:** bare `Osc.sine(p) * 0.5` (audio × Num) MUST still abort — `synth_error_cases_abort` (`audio_bindings.rs:1094`) and `synth_env_scaled_by_constant_renders` (`:1125`) stay green. `.scale(k)` is the sanctioned scaling path; do NOT modify those tests or the operator overloads.
- Change both the `Node` class AND the `Port` class (their `*`/`scale` are duplicated); both get both `scale` arities.
- Verified fact (throwaway render test passed): `carrier.pm = mod * Node.ctrl_(3)` renders inside a Synth and scales FM depth (index 3 ≠ index 1). `carrier.pm = mod * Env.adsr(...)` (node × control node) also already works.
- Test invocation: per-crate, NEVER `--workspace`; `-- name1 name2`. LSP `armv7a … can't find crate for test` is noise.

---

### Task 1: audio-safe `scale` on `Node` + `Port`

**Files:**
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`Node` class ~line 269, `Port` class ~line 383)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: the existing `Node.ctrl_(value)` foreign static (prelude:178 — wraps a constant into a control `Kind::Ctrl` node), the `*`/`+` operator overloads (unchanged).
- Produces: one-arg `scale(k)` + audio-safe two-arg `scale(m, a)` on `Node` and `Port`. No Rust surface.

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-wren-core/tests/audio_bindings.rs` (mirror the existing `poly_sine_fm_plays_in_synth` harness — `run_and_render(script, &mut out)` with `StereoFrame`; and `run_script_ok` for the no-abort check):

```rust
#[test]
fn fm_index_via_scale_renders_and_scales_depth() {
    // car.pm = m.scale(3) must render AND differ from car.pm = m (index 1),
    // proving .scale applies real, scaled FM depth.
    let mut out_scaled = [StereoFrame::default(); 64];
    run_and_render(
        "var s = Synth.new { |p|\n  var m = Osc.sine(p)\n  var c = Osc.sine(p)\n  c.pm = m.scale(3)\n  return c * Env.adsr(0.001, 0.5, 1, 0.2)\n}\nOut.patch(s.out)\ns.noteOn(60, 100)",
        &mut out_scaled,
    );
    assert!(out_scaled.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "scaled bounded/finite");
    assert!(out_scaled.iter().any(|f| f.l.abs() > 1e-3), "m.scale(3) sounds (no abort)");
    let mut out_unit = [StereoFrame::default(); 64];
    run_and_render(
        "var s = Synth.new { |p|\n  var m = Osc.sine(p)\n  var c = Osc.sine(p)\n  c.pm = m\n  return c * Env.adsr(0.001, 0.5, 1, 0.2)\n}\nOut.patch(s.out)\ns.noteOn(60, 100)",
        &mut out_unit,
    );
    let differs = out_scaled.iter().zip(out_unit.iter()).any(|(a, b)| (a.l - b.l).abs() > 1e-4);
    assert!(differs, "index 3 differs from index 1 -> FM depth scaled");
}

#[test]
fn fm_index_enveloped_renders() {
    // A modulated (enveloped) index: car.pm = m * Env.adsr(...) (node × control node).
    let mut out = [StereoFrame::default(); 64];
    run_and_render(
        "var s = Synth.new { |p|\n  var m = Osc.sine(p)\n  var c = Osc.sine(p)\n  c.pm = m * Env.adsr(0.001, 0.3, 0.5, 0.2)\n  return c * Env.adsr(0.001, 0.5, 1, 0.2)\n}\nOut.patch(s.out)\ns.noteOn(60, 100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite()) && out.iter().any(|f| f.l.abs() > 1e-3), "enveloped index sounds");
}

#[test]
fn scale_two_arg_audio_safe_renders() {
    // The fixed two-arg scale(m,a) must no longer abort on an audio node.
    assert!(
        run_script_ok("var s = Synth.new { |p|\n  var c = Osc.sine(p).scale(2, 0)\n  return c * Env.adsr(0.001, 0.5, 1, 0.2)\n}\nOut.patch(s.out)\ns.noteOn(60, 100)"),
        "audio.scale(2,0) renders without aborting"
    );
}
```
> Confirm `Env.adsr` is the right name (the poly-FM tests use it). If a render window longer than 64 frames is needed for the ADSR attack to become audible, widen it — but do NOT weaken the `>1e-3` / `differs` assertions.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- fm_index_via_scale fm_index_enveloped scale_two_arg`
Expected: `fm_index_via_scale_renders_and_scales_depth` FAILS (Wren has no `scale(_)` one-arg selector → runtime error → render silent/non-differing); `scale_two_arg_audio_safe_renders` FAILS (old `scale(2,0) = osc * 2 + 0` hits the audio×Num guard → aborts → `run_script_ok` false). `fm_index_enveloped_renders` may already PASS (node×control works today) — that's fine, it's a lock.

- [ ] **Step 3: Edit the `Node` class `scale` (prelude.wren ~269)**

```wren
// BEFORE
  scale(m, a)    { this * m + a }
// AFTER
  scale(k)       { this * Node.ctrl_(k) }               // audio-safe scale-by-constant (FM index, etc.)
  scale(m, a)    { this * Node.ctrl_(m) + Node.ctrl_(a) } // audio-safe affine (was `this * m + a`, aborted on audio)
```
> `Node.ctrl_(x)` is a control node, so `this * <control node>` / `+ <control node>` take the non-abort operator branch and the engine broadcasts the constant across voices. Single-expression bodies auto-return (matches the neighboring `to`/`atten`/`scale` style — no explicit `return`). For a CONTROL `this` this is behavior-identical to the old body (control `this * Num` already wrapped the Num in `ctrl_`); for an AUDIO `this` it now works instead of aborting.

- [ ] **Step 4: Edit the `Port` class `scale` (prelude.wren ~383)** — identical change

```wren
// BEFORE
  scale(m, a)    { this * m + a }
// AFTER
  scale(k)       { this * Node.ctrl_(k) }
  scale(m, a)    { this * Node.ctrl_(m) + Node.ctrl_(a) }
```
> `Node.ctrl_` is a static on the global `Node` class, reachable from `Port` methods.

- [ ] **Step 5: Run the new tests + full regression, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- fm_index_via_scale fm_index_enveloped scale_two_arg` (+ `--features deluge-dsp-kernels/simd`).
Expected: PASS.
Then the FULL wren-core suite BOTH configs. Expected: all green — CRUCIALLY `synth_error_cases_abort` (bare `Osc.sine(p) * 0.5` still aborts) and `synth_env_scaled_by_constant_renders` stay green (the operator guard is untouched). Existing poly-FM and control-node tests unchanged.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): audio-safe scale(k)/scale(m,a) via ctrl_ — sets FM index (car.pm = mod.scale(3))"
```

---

## Self-Review

**Spec coverage:** The spec's design — one-arg `scale(k)` + audio-safe two-arg `scale(m, a)` on both `Node` and `Port`, via `ctrl_` wrapping, guard untouched — is Task 1 in full. Tests cover the constant index (`m.scale(3)` renders + scales depth), the modulated index (`m * Env.adsr`), and the two-arg audio-safe regression; the guard-lock tests are explicitly kept green. Docs (flipping the poly-FM spec's limitation note + the `osc-fm-poly` memory) are controller-handled at merge, per the spec's note that Task 2 folds in when small.

**Placeholder scan:** All code is verbatim before/after. The one "confirm `Env.adsr` name / widen window if needed" note is a transcription check with a concrete fallback, not a design gap.

**Type consistency:** `scale(k)` (arity 1) and `scale(m, a)` (arity 2) are distinct Wren selectors — they coexist. Both delegate to the existing `*`/`+` overloads and `Node.ctrl_` (unchanged). Identical edit on `Node` and `Port`.
