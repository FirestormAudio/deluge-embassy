# Master DC-Block (IO-3b) — Design Spec

**Date:** 2026-07-13
**Status:** Approved (design), pending implementation plan
**Sub-project:** IO suite — IO-3 master chain, second slice. IO-3 slices: IO-3a master
limiter (DONE), **IO-3b master DC-block** (this spec), IO-3c master EQ (deferred). Follows
IO-1 audio input (DONE).

## Context

IO-3a added an opt-in master **limiter** on the render root bus at the render seam, before
the `[-1, 1]` clamp. IO-3b adds a stereo **DC-blocker** as the FIRST master-chain stage —
removing DC offset / subsonic content before the limiter, because DC eats headroom and
biases the limiter's peak detector. It reuses the IO-3a engine-`Option<>`-stage machinery
exactly. See [[io-suite-routing]], [[fx-suite-effects]], [[prefer-neon-simd]],
[[wren-prelude-single-line-body]].

The kernel building block already exists: `OnePoleHp` (`crates/deluge-dsp-kernels/src/
filter.rs:253`) — a bilinear first-order high-pass (`H(z) = (1+a)/2·(1−z⁻¹)/(1−a·z⁻¹)`, a
**true DC-blocker**: zero at DC `H(1)=0`, unity at Nyquist `H(−1)=1`). But it is `pub(crate)`
with a module-private `process`, so `deluge-audio-graph` cannot use it directly.

**Model (chosen):** an engine-level, **opt-in**, stereo DC-blocker applied to the root bus
at the render seam, ordered **DC-block → limiter → clamp**. Cutoff **configurable with a
20 Hz default** (`Out.dcBlock()` / `Out.dcBlock(hz)`). Per-channel (no linking — DC is
independent per channel, unlike the limiter's shared gain).

## Design

### Kernel — `MasterDcBlock` in `crates/deluge-dsp-kernels/src/filter.rs`

Placed in `filter.rs` (the SAME module as `OnePoleHp`) so it can call `OnePoleHp`'s
module-private `process` and `pub(crate) set_coeff` WITHOUT changing `OnePoleHp`'s
visibility. `MasterDcBlock` itself is `pub` (reachable as
`deluge_dsp_kernels::filter::MasterDcBlock`, since `pub mod filter;` in `lib.rs`).

```rust
pub struct MasterDcBlock {
    ch: [OnePoleHp; 2], // [L, R] — per-channel, no cross-channel linking
}
```

- `new(cutoff_hz: f32, dt: f32) -> MasterDcBlock` — `ch: [OnePoleHp::default(); 2]`, then
  `set_cutoff(cutoff_hz, dt)`.
- `set_cutoff(&mut self, cutoff_hz: f32, dt: f32)` — `wc = sanitize_cutoff(cutoff_hz) as f64
  * TAU * dt.max(FLOOR) as f64` (radians/sample), then `ch[0].set_coeff(wc); ch[1].set_coeff
  (wc)` (same coeff both channels).
- `process(&mut self, l: &mut [f32], r: &mut [f32])` — per sample `l[i] = ch[0].process
  (l[i]); r[i] = ch[1].process(r[i])` (each channel independent; `n = l.len().min(r.len())`).
  (No `dt` param — the coeff is already set; the seam call is `dc.process(&mut bus_l, &mut
  bus_r)`.)

**No panic on any input:** `sanitize_cutoff` finite-guards and clamps `cutoff_hz` to a sane
positive range (e.g. `[0.1, 20_000.0]`) so `wc`/`tan` stay finite; `dt` floored to a small
positive `FLOOR`. `OnePoleHp::process` is a stable IIR with no division/index. 32-bit-`usize`
safe (indexes `0..len`). Default cutoff **20 Hz** (`const DEFAULT_DC_HZ: f32 = 20.0`).

**Scalar** — the one-pole HP is a per-sample IIR recursion (`y[n]` depends on `y[n-1]`), not
data-parallel; per [[prefer-neon-simd]]'s "don't force it" (same as the limiter and the
existing filters). No `f32x8` path; tests pass identically in both crate configs.

### Engine — `crates/deluge-audio-graph/src/engine.rs` + `cmd.rs`

- **Field:** `master_dcblock: Option<MasterDcBlock>` on `Engine` (init `None` in `Engine::new`;
  cleared in the `Cmd::Reset` arm), mirroring `master_limiter`. Import
  `use deluge_dsp_kernels::filter::MasterDcBlock;`.
- **`Cmd::SetMasterDcBlock { cutoff_hz: f32 }`** (new variant in `cmd.rs`) + an `Engine::apply`
  arm (create-or-update-in-place, using `self.dt`): if `Some`, `dc.set_cutoff(cutoff_hz,
  self.dt)`; if `None`, `Some(MasterDcBlock::new(cutoff_hz, self.dt))`.
- **Render seam:** fold the DC-block into the existing master-chain seam, BEFORE the limiter:
  ```rust
  if let Some(root) = self.root {
      let b = root.0 as usize;
      if let Some(dc) = &mut self.master_dcblock { dc.process(&mut self.bus_l[b], &mut self.bus_r[b]); }
      if let Some(lim) = &mut self.master_limiter { lim.process(&mut self.bus_l[b], &mut self.bus_r[b], self.dt); }
  }
  ```
  (The current seam has the limiter in its own `if let Some(root)` block; IO-3b combines the
  DC-block and limiter under one `if let Some(root)` with the DC-block first. The `[-1, 1]`
  clamp-copy that follows is UNCHANGED.) Disjoint field borrows (`master_dcblock`,
  `master_limiter`, `bus_l`, `bus_r`, `dt`). When both stages are `None`, the seam is a no-op
  ⇒ render byte-identical.

### Wren — `Out.dcBlock(...)` (`crates/deluge-wren-core/`)

- **`audio::set_master_dcblock(cutoff_hz: f32)`** facade → `Cmd::SetMasterDcBlock` (mirrors
  `audio::set_master_limit`).
- **`node_master_dcblock_impl<S>(vm)`** (`bindings_audio.rs`): `cutoff = vm.get_f(1) as f32`
  (SAFE — reads no foreign args, no `checked_*` needed), `audio::set_master_dcblock(cutoff)`.
  A void config call, returns nothing (like `node_master_limit_impl`). Plus the
  `#[cfg(feature="wren-sys-backend")]` extern-C shim `node_master_dcblock`.
- Register `Node.masterDcBlock_(_)` (arity 1) in BOTH tables (`METHODS` in `bindings.rs` +
  `register_audio` in `bindings_audio.rs`) + the shim.
- **Prelude `class Out`** (extend the existing class) — implicit-return one-liners (a
  single-line explicit-`return` body does NOT compile — see [[wren-prelude-single-line-body]]):
  ```wren
  static dcBlock() { Node.masterDcBlock_(20.0) }
  static dcBlock(cutoff) { Node.masterDcBlock_(cutoff) }
  ```
  plus `foreign static masterDcBlock_(cutoff)` in `foreign class Node`. Two arities dispatch
  by count; the nullary form passes the `20.0` default explicitly to the one-arg foreign.
- Usage: `Out.patch(mySynth); Out.dcBlock(); Out.limit(0.9)`. Global/master control — no
  `polyMode_` guard.

## NEON / SIMD

Scalar — per-sample IIR recursion (see kernel note). No forced SIMD; no data-parallel
sub-loop left un-vectorized.

## Non-breaking

Purely additive: a new `pub` kernel struct (`OnePoleHp` UNCHANGED — no visibility change),
one `Engine` field (`Option`, `None` default → render byte-identical when unused), one `Cmd`
variant + `apply` arm + `Reset` clear, one `audio.rs` facade, one binding + prelude method.
The IO-3a limiter and the `[-1, 1]` clamp are untouched. Existing suites stay green both
configs (default + `--features deluge-dsp-kernels/simd`); the device build cross-compiles.

## Constraints (global)

- `no_std`, no heap, **no panic on any input** (cutoff finite-guarded + range-clamped, dt
  floored). 32-bit-`usize` safe.
- Per-channel (no linking). Cutoff configurable, default 20 Hz. MIT/Apache-2.0.
- Wren reads only `get_f` (safe); future foreign reads use the [[wren-binding-safety]]
  `checked_*` guards.
- The DC-block runs BEFORE the limiter; the final `[-1, 1]` clamp REMAINS.

## Out of scope / deferred

- **Master EQ** (IO-3c — the next master-chain slice), **lookahead limiter**, a **bypass/
  disable `Cmd`** (opt-in-only; `Reset` clears), a higher-order / steeper DC-block, per-channel
  independent cutoffs, metering.
- The other IO sub-projects (IO-2 mixer/bus expansion, IO-4 multiple outputs).

## Testing

- **Kernel oracle** (`filter.rs` tests): a constant-DC input (e.g. 0.5) → output decays
  toward ~0 over a short settle (DC removed); a DC+AC input (0.5 + a sine) → the DC component
  is removed (block mean → ~0) while the AC amplitude is preserved (within tolerance); L/R
  independent (different per-channel DC offsets each removed); no-panic on NaN/inf/0/negative/
  huge `cutoff_hz` and on a tiny/zero `dt`. Scalar; passes BOTH crate configs.
- **Engine**: enable via `Cmd::SetMasterDcBlock`, render a DC-offset root bus → output block
  mean → ~0; **disabled → output byte-identical**; `Cmd::Reset` clears it; DC-block + limiter
  compose (both applied, DC-block first — a DC-offset loud signal is both DC-removed and
  limited).
- **Wren** (Cmd-capture): `Out.dcBlock()` emits `Cmd::SetMasterDcBlock { cutoff_hz: 20.0 }`;
  `Out.dcBlock(10)` emits `{ cutoff_hz: 10.0 }`.
- **e2e** (real-`EngineHost`): a source with a DC offset through `Out.dcBlock()` renders with
  the offset removed (block mean near 0, AC content intact); existing synths unchanged.

## Likely task decomposition (for writing-plans)

1. **`MasterDcBlock` kernel** (`filter.rs`): the `pub struct` wrapping `[OnePoleHp; 2]` +
   `new`/`set_cutoff`/`process` + `sanitize_cutoff` + `DEFAULT_DC_HZ`; oracle tests
   (DC-removed, AC-preserved, L/R-independent, no-panic). `OnePoleHp` unchanged.
2. **Engine wiring**: `master_dcblock: Option<MasterDcBlock>` field + `new` init + `Reset`
   clear; `Cmd::SetMasterDcBlock { cutoff_hz }` + `apply` arm (create-or-update-in-place with
   `self.dt`); the render seam (DC-block before the limiter). Engine tests (DC-removed-when-
   enabled, byte-identical-when-disabled, Reset-clears, composes-with-limiter).
3. **Wren `Out.dcBlock`**: `audio::set_master_dcblock` facade + `node_master_dcblock_impl` +
   both tables + shim + prelude `class Out` methods + `foreign static masterDcBlock_`.
   Cmd-capture test (both arities emit the right `SetMasterDcBlock`, nullary → 20.0).
4. **e2e**: real-`EngineHost` DC-offset-source-through-`Out.dcBlock` renders with the offset
   removed; non-breaking.
