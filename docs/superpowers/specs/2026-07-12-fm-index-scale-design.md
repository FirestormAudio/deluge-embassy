# FM Index via `.scale(k)` — Design Spec

**Date:** 2026-07-12
**Status:** Approved (design), pending implementation plan
**Sub-project:** Osc suite — the poly-FM depth-idiom follow-up (flagged by the poly-FM review)

## Problem

The just-merged poly FM feature ([[osc-fm-poly]]) can't set a static modulation
INDEX via its own pitched idiom: `carrier.pm = modulator * 3` ABORTS inside a
`Synth`. Root cause is a pre-existing Synth-DSL guard (`prelude.wren:238`/`:358`):
`node * Num` where the node is a per-voice AUDIO signal (`isPoly_==1`) aborts
("multiply by a constant inside a Synth isn't supported yet — the amp comes from
`Env.ar`"), a deliberate Sy-4 decision to steer amplitude through `Env.ar` and
away from `osc * k`.

## Key discovery (verified empirically)

The guard ONLY fires on `audio-node * Num`. It does NOT fire on `node * node`
(the `o is Num` check gates the whole abort branch). And `Node.ctrl_(k)` wraps a
constant into a control-rate node that the engine broadcasts across all VOICES
lanes. Therefore **`carrier.pm = modulator * Node.ctrl_(3)` already works today**
— it builds `PolyMul(modulator, ctrl(3))`, renders without aborting, and the FM
depth genuinely scales (a throwaway render test confirmed index 3 ≠ index 1).
And `carrier.pm = modulator * Env.ar(...)` (node × control node) gives a
time-varying / enveloped index — the expressive DX case — also already working.

So the FM-index CAPABILITY exists; only ergonomics + discoverability are missing.
No kernel/graph/Rust changes are needed — the osc kernel already adds `pm`
unscaled (`rp = p + pm + fb`), so a pre-scaled modulator signal is exactly right.
(A DX-style per-operator `index=` kernel scalar param was considered and REJECTED:
it would be constant-only, add a redundant depth path alongside signal-scaling,
and require kernel edits — whereas signal-scaling handles constant AND modulated
index uniformly.)

## The collision: `.scale` already exists (as a broken-for-audio affine)

`prelude.wren:269` (Node) and `:383` (Port) already define a TWO-arg affine map
`scale(m, a) { this * m + a }`. But that version is itself broken for audio nodes:
its `this * m` (Num operand) hits the same guard and aborts. It has **zero
callers** anywhere in the prelude or tests (grep-confirmed) — effectively dead.
So the clean design makes the whole `scale` family audio-safe, not just adds a
one-arg method.

## Design (prelude-only — zero Rust/kernel/graph changes)

On BOTH the `Node` class and the `Port` class in `crates/deluge-wren-core/wren/prelude.wren`:

1. **Add a one-arg `scale(k)`** — the FM-index / audio-signal-scale sugar:
   ```wren
   scale(k) { this * Node.ctrl_(k) }
   ```
   `Node.ctrl_(k)` is a control node (not a Num), so `this * <control node>` takes
   the non-abort operator branch → `PolyMul(this, ctrl(k))` (poly) or `binop_`
   (mono), and the engine broadcasts `k` across voices. Works for a per-voice
   AUDIO `this` (the modulator) AND for control nodes. Single-expression body
   auto-returns (matches the existing `scale(m, a)` style — no explicit `return`).

2. **Fix the existing two-arg `scale(m, a)`** to be audio-safe too, so the family
   is consistent (no "`scale(3)` works but `scale(3, 0)` aborts on audio" wart):
   ```wren
   scale(m, a) { this * Node.ctrl_(m) + Node.ctrl_(a) }
   ```
   Wrapping both operands in `Node.ctrl_` routes through the non-abort branch.
   For CONTROL nodes this is behavior-identical to the old body (a control `this
   * Num` already wrapped the Num in `ctrl_` internally); for AUDIO nodes it now
   works instead of aborting. Zero callers today → purely additive.

Usage after this change:
```wren
Synth.new { |p|
  var m = Osc.sine(p)
  var c = Osc.sine(p)
  c.pm = m.scale(3)        // constant FM index 3
  // c.pm = m * Env.adsr(...)   // OR a modulated/enveloped index (already works)
  return c * Env.adsr(0.001, 0.5, 1, 0.2)
}
```

## What is deliberately NOT changed

- The `*`/`+` **Num-guard stays**: a bare `Osc.sine(p) * 0.5` (audio × Num) STILL
  aborts. `synth_error_cases_abort` (`audio_bindings.rs:1094`) must stay green.
  `.scale(k)` is the sanctioned path for intentional audio-signal scaling; the
  guard still catches the accidental `osc * k`-as-amplitude footgun (`* k` reads
  as amplitude; `.scale(k)` reads as an explicit deliberate scale).
- No kernel/graph/`bindings_audio.rs` change. No new foreign method (`scale` is
  pure Wren over the existing `ctrl_`/`*`/`+`).
- Control-node `*`/`+`/`scale`/`.to` behavior is unchanged.

## Non-breaking guarantee

- `synth_error_cases_abort` (bare audio × Num aborts) stays green — the guard is
  untouched.
- `synth_env_scaled_by_constant_renders` (control × Num renders) stays green — no
  change to the control-node path.
- The two-arg `scale(m, a)` has no existing callers, and its control-node
  behavior is preserved (equivalent `ctrl_` wrapping), so nothing regresses.
- All existing poly-FM tests (`car.pm = mod`) stay green — untouched.

## Testing

- **`car.pm = m.scale(3)`** inside a Synth renders finite/bounded/non-silent AND
  differs measurably from `car.pm = m` (index-1) — proving `.scale` applies real,
  scaled FM depth (mirror the throwaway verification that passed).
- **`car.pm = m * Env.adsr(...)`** (modulated index) renders — locks the
  enveloped-index path.
- **`someAudioNode.scale(2, 0)`** (the fixed two-arg) inside a Synth renders
  without aborting (regression lock for the audio-safe affine).
- **Guard still holds:** `synth_error_cases_abort` and
  `synth_env_scaled_by_constant_renders` stay green (do not modify them).
- Full wren-core suite green BOTH configs (default + `--features
  deluge-dsp-kernels/simd`).

## Docs

- Flip the `2026-07-12-osc-fm-poly-design.md` "KNOWN LIMITATION" note into a
  "setting FM index" note pointing at `m.scale(k)` / `m * Env.ar(...)`.
- Update the `osc-fm-poly` memory (limitation → resolved via `.scale`).
- A short prelude comment beside `scale` explaining the audio-safe `ctrl_` wrap
  and the FM-index use.

## Constraints (global)

- `no_std` (unaffected — prelude is Wren text). MIT/Apache-2.0.
- Tests pass BOTH feature configs.
- Prelude changes only; both `Node` AND `Port` classes get both `scale` arities.
- Non-breaking: the `*`/`+` Num-guard and all control-node behavior unchanged.

## Out of scope / deferred

- Relaxing the `*`/`+` Num-guard so bare `mod * 3` works (rejected — reverses the
  test-locked Sy-4 amplitude-footgun decision).
- A DX-style per-operator `index=` kernel param (rejected — constant-only,
  redundant with signal-scaling, needs kernel edits).
- PolySync pm/feedback, multi-operator algorithm routing (from the poly-FM
  deferred list).

## Likely task decomposition (for writing-plans)

1. Prelude: add one-arg `scale(k)` + fix two-arg `scale(m, a)` on both `Node` and
   `Port`; add the audio-safe FM-index render tests + the two-arg regression test;
   confirm the guard-lock tests stay green; both configs.
2. Docs: flip the poly-FM spec limitation note + update the `osc-fm-poly` memory
   + prelude comment. (May fold into Task 1 if small.)
