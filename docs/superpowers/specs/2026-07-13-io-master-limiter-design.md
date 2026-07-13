# Master Limiter (IO-3a) — Design Spec

**Date:** 2026-07-13
**Status:** Approved (design), pending implementation plan
**Sub-project:** IO suite — IO-3 master chain, first slice (the limiter). The IO suite
decomposes into IO-1 audio input (DONE), IO-2 mixer/bus expansion, IO-3 master chain, IO-4
multiple outputs. IO-3 is itself sliced: **IO-3a master limiter** (this spec); DC-block and
EQ are deferred to later IO-3 slices.

## Context

Today the ONLY processing between the summed master (root) bus and the DAC is a hard
`[-1, 1]` clamp in `Engine::render` (`crates/deluge-audio-graph/src/engine.rs:389-401`) —
harsh clipping distortion on any overshoot. The vision's IO row names a "master chain"
(`docs/superpowers/specs/2026-07-06-dsp-library-vision-design.md:170`); the highest-value,
safety-relevant element is a **limiter** that catches peaks musically *before* that clamp.
IO-1 explicitly deferred "IO-3 master chain (master insert: limiter/EQ/DC-block)"
(`docs/superpowers/specs/2026-07-12-io-input-design.md`). See [[io-suite-routing]],
[[fx-suite-effects]], [[prefer-neon-simd]], [[wren-prelude-single-line-body]].

**Model (chosen):** an **engine-level, opt-in, stereo-LINKED, feedforward (no-lookahead)
peak limiter** applied to the root bus at the render seam (right after buses sum, before the
clamp). Opt-in via `Option<MasterLimiter>` (`None` default = existing render byte-unchanged).
The existing `[-1, 1]` clamp stays as the residual safety net for fast transients the
zero-latency limiter lets through.

## Design

### Kernel — `crates/deluge-dsp-kernels/src/limiter.rs` (new, pure `no_std`, scalar)

```rust
pub struct MasterLimiter {
    gain: f32,        // current gain reduction, 1.0 = no reduction (envelope state)
    ceiling: f32,     // linear output ceiling (0..1), the max |sample| target
    release_s: f32,   // release time in seconds
}
```

`process(&mut self, l: &mut [f32], r: &mut [f32], dt: f32)` — **stereo in-place** (no scratch;
operates directly on the two bus rows). Per sample `i`:
- `peak = max(|l[i]|, |r[i]|)`
- `target = if peak > ceiling { ceiling / peak } else { 1.0 }` (guarded: `peak > ceiling >
  0` ⇒ divisor is strictly positive, no div-by-zero)
- **attack** (near-instant): `if target < self.gain { self.gain = target }`
- **release** (else): `self.gain += (1.0 - self.gain) * release_coeff` where
  `release_coeff = 1 - exp(-dt / release_s)` (one-pole recovery toward unity;
  `release_s`-derived, floored so `dt`/`release_s` is finite)
- apply **linked**: `l[i] *= self.gain; r[i] *= self.gain` (SAME gain both channels → no
  stereo-image shift — the defining property of a stereo master limiter)

Setters `set_ceiling(v)` / `set_release(v)`; `new(ceiling, release)` (gain starts at 1.0).

**No panic on any input:** `ceiling` finite-guarded and floored to a small positive value
(`ceiling = if ceiling.is_finite() { ceiling.max(1e-4) } else { DEFAULT }`) — a zero/negative/
NaN ceiling must not divide-by-zero or over-attenuate to silence-forever; `release_s`
finite-guarded and floored (`.max(1e-4)`) so `release_coeff` is finite and in `(0, 1]`; a
non-finite input sample yields `peak` non-finite → `target` computes to `1.0` via the guard
path (finite-check the peak before the compare, treat non-finite as pass-through 1.0). 32-bit
`usize` safe (indexes `0..l.len()`).

**Scalar** — the gain envelope is a serial recursion (`gain[i]` depends on `gain[i-1]`), so
the loop does not vectorize across a fixed lane width. Per [[prefer-neon-simd]]'s "don't
force it" (same as the `Comp`/`Gate` envelope followers in `dynamics.rs`). No `f32x8` path;
tests pass identically in both crate configs because the crate builds both ways.

Ceiling is **linear** (0..1, matching the output/clamp scale; default `0.95`). Release in
seconds (default `0.05`).

### Engine — `crates/deluge-audio-graph/src/engine.rs` + `cmd.rs`

- **Field:** `master_limiter: Option<MasterLimiter>` on `Engine`, initialized `None` in
  `Engine::new`, cleared to `None` in the `Cmd::Reset` arm (mirroring how `root`/`stream_state`
  reset).
- **`Cmd::SetMasterLimit { ceiling: f32, release: f32 }`** (new variant in `cmd.rs`) + an
  `Engine::apply` arm: if `self.master_limiter` is `Some`, update its params in place via the
  setters (PRESERVE the running `gain` envelope — no click on a live param change); if `None`,
  set `Some(MasterLimiter::new(ceiling, release))`.
- **Render seam** (between `engine.rs:388` end-of-bus-writes and `:389` clamp-copy):
  ```rust
  if let Some(root) = self.root {
      if let Some(lim) = &mut self.master_limiter {
          let b = root.0 as usize;
          lim.process(&mut self.bus_l[b], &mut self.bus_r[b], self.dt);
      }
  }
  ```
  Three disjoint field borrows (`master_limiter`, `bus_l`, `bus_r`), in-place; the existing
  clamp-copy at `:389-401` then runs unchanged. When `master_limiter` is `None`, this block is
  a no-op and the render path is byte-identical to today.

### Wren — `Out.limit(...)` (`crates/deluge-wren-core/`)

- **`audio::set_master_limit(ceiling: f32, release: f32)`** facade → `Cmd::SetMasterLimit`
  (mirrors `audio::set_root`).
- **`node_master_limit_impl<S>(vm)`** (`bindings_audio.rs`): `ceiling = vm.get_f(1)`,
  `release = vm.get_f(2)` (both `get_f` — SAFE, no `checked_*` needed; nullary of foreign
  args), `audio::set_master_limit(ceiling, release)`. It is a void config call — returns
  nothing (leaves slot 0 as-is, exactly like `node_patch_impl` for `Out.patch`). Register
  `Node.masterLimit_(_,_)` in BOTH tables + the extern-C shim.
- **Prelude `class Out`** (extend the existing class) — respecting the
  [[wren-prelude-single-line-body]] gotcha (implicit-return one-liner is fine, explicit
  `return` on one line is NOT):
  ```wren
  static limit(ceiling) { Node.masterLimit_(ceiling, 0.05) }
  static limit(ceiling, release) { Node.masterLimit_(ceiling, release) }
  ```
  plus `foreign static masterLimit_(_,_)` in `foreign class Node`.
- Usage: `Out.patch(mySynth); Out.limit(0.9)`. It is a global/master control (NOT a poly
  voice source, NOT patched into a chain) — no `polyMode_` guard.

## NEON / SIMD

Scalar — serial gain-envelope recursion (see kernel note). No forced SIMD; no data-parallel
sub-loop is left un-vectorized (the per-sample `max(|l|,|r|)` is dominated by the serial
envelope dependency).

## Non-breaking

Purely additive: a new kernel module, one `Engine` field (`Option`, `None` default → the
render path is byte-identical when unused), one `Cmd` variant + `apply` arm + `Reset`
clearing, one `audio.rs` facade, one binding + prelude method. No existing node, render
behavior, `Cmd`, or binding changes. Existing suites stay green both configs (default +
`--features deluge-dsp-kernels/simd`); the device build cross-compiles.

## Constraints (global)

- `no_std`, no heap, **no panic on any input** (ceiling/release finite-guarded + floored;
  non-finite samples pass through at gain 1.0; the `peak > ceiling > 0` guard prevents
  div-by-zero). 32-bit-`usize` safe.
- Linked stereo (one shared gain for L+R). Ceiling linear (0..1). MIT/Apache-2.0.
- Wren reads only `get_f` (safe); any future foreign-arg read uses the [[wren-binding-safety]]
  `checked_*` guards.
- The final `[-1, 1]` clamp in `render` REMAINS (residual safety net for transients the
  zero-latency limiter overshoots).

## Out of scope / deferred

- **Lookahead / brickwall** (needs a delay buffer + latency — the feedforward limiter plus
  the residual clamp is the chosen zero-latency slice).
- **DC-block** master stage (promote `pub(crate) OnePoleHp`) and **master EQ** — later IO-3
  slices.
- A **bypass/disable `Cmd`** (once enabled it stays until `Reset`; opt-in-only this slice),
  attack-time control (attack is fixed near-instant), gain-reduction metering, dB-unit
  ceiling, per-channel (unlinked) mode, multiband.
- The other IO sub-projects (IO-2 mixer/bus expansion, IO-4 multiple outputs).

## Testing

- **Kernel oracle** (`limiter.rs`): a block above `ceiling` → every output `|sample| <=
  ceiling` (within a small attack-transient tolerance on the first sample); **linked** — a
  loud-L/quiet-R block scales BOTH by the same gain (assert `out_r/in_r == out_l/in_l` ratio,
  R's relative level preserved, no image shift); a below-ceiling block passes byte-unchanged
  (gain stays 1.0); release recovers gain toward 1.0 over successive blocks after the loud
  input stops; no-panic on NaN/inf/zero/negative/huge `ceiling` and `release`, and on
  non-finite input samples. Scalar oracle; passes in BOTH crate configs.
- **Engine**: enable via `Cmd::SetMasterLimit`, render a hot root bus → output peak `<=
  ceiling`; **disabled (no Cmd) → output byte-identical** to the pre-IO-3 raw→clamp path;
  `Cmd::Reset` clears the limiter (subsequent render is raw→clamp again); a live
  `SetMasterLimit` on an already-enabled limiter updates params without resetting the gain
  envelope.
- **Wren** (Cmd-capture): `Out.limit(0.5)` emits `Cmd::SetMasterLimit { ceiling: 0.5,
  release: 0.05 }`; `Out.limit(0.5, 0.1)` emits the 2-arg form.
- **e2e** (real-`EngineHost`): a loud synth through `Out.patch(...)` + `Out.limit(0.5)`
  renders bounded near the ceiling (peak `<= 0.5 + tol`); a quiet patch is unaffected;
  existing synths unchanged.

## Likely task decomposition (for writing-plans)

1. **`MasterLimiter` kernel** (`limiter.rs`): struct + `process` (peak → linked gain
   envelope: instant attack, one-pole release) + setters + `new`; no-panic guards
   (ceiling/release floor, non-finite peak pass-through); oracle tests (bounded, linked,
   passthrough-below-ceiling, release-recovery, no-panic). Add `pub mod limiter;` to
   `lib.rs`.
2. **Engine wiring**: `master_limiter: Option<MasterLimiter>` field + `new` init + `Reset`
   clear; `Cmd::SetMasterLimit { ceiling, release }` + `apply` arm (create-or-update-in-place);
   the render seam call. Engine tests (bounded-when-enabled, byte-identical-when-disabled,
   Reset-clears, live-param-update-preserves-envelope).
3. **Wren `Out.limit`**: `audio::set_master_limit` facade + `node_master_limit_impl` + both
   tables + extern-C shim + prelude `class Out` methods + `foreign static masterLimit_`.
   Cmd-capture test (both arities emit the right `SetMasterLimit`).
4. **e2e**: real-`EngineHost` loud-synth-through-`Out.limit` renders bounded near ceiling;
   quiet patch unaffected; non-breaking.
