# Master EQ (IO-3c) — Design Spec

**Date:** 2026-07-13
**Status:** Approved (design), pending implementation plan
**Sub-project:** IO suite — IO-3 master chain, THIRD and final slice (completes IO-3). IO-3
slices: IO-3a master limiter (DONE), IO-3b master DC-block (DONE), **IO-3c master EQ** (this
spec). Follows IO-1 audio input (DONE).

## Context

IO-3a/3b built the opt-in master-chain seam: a DC-block and a limiter applied to the render
root bus (`DC-block → limiter → clamp`), each an engine `Option<>` stage configured via a
`Cmd`. IO-3c adds a single-band stereo **EQ** between them (`DC-block → EQ → limiter`) —
EQ before the limiter so boosts get caught. It reuses the existing `Eq` kernel and the
established `Option<>`-stage machinery. See [[io-suite-routing]], [[fx-suite-effects]],
[[prefer-neon-simd]], [[wren-prelude-single-line-body]].

The kernel already exists and is directly usable: `Eq` (`crates/deluge-dsp-kernels/src/
eq.rs`) — a `pub` RBJ biquad EQ (`EqType::{Peak, LowShelf, HighShelf}`), setters `set_freq`
(Hz, clamp 10–20k), `set_gain` (dB, clamp ±24), `set_q` (clamp 0.1–20), `set_type`
(0=Peak, 1=LowShelf, 2=HighShelf). It is **mono** (one `Biquad` with `z1/z2` state), so a
stereo master EQ needs two `Eq` instances (L/R), mirroring `MasterDcBlock`'s two `OnePoleHp`.

**Model (chosen):** an engine-level, **opt-in**, stereo **single-band** EQ on the root bus,
ordered `DC-block → EQ → limiter → clamp`. Configured via `Out.eq/eqLowShelf/eqHighShelf`
(freq/gain/q, type by named method). Multi-band deferred.

## Design

### Kernel — `MasterEq` in `crates/deluge-dsp-kernels/src/eq.rs`

Placed in `eq.rs` (same module as `Eq`/`EqType`/the private `rbj_coeffs`), `pub` (reachable
as `deluge_dsp_kernels::eq::MasterEq`; `pub mod eq;` in `lib.rs`).

```rust
pub struct MasterEq { ch: [Eq; 2] } // [L, R] — same params, independent biquad state
```

- `new(freq: f32, gain_db: f32, q: f32, eq_type: u8) -> MasterEq` — `[Eq::new(EqType::Peak);
  2]` then `set_params(...)`.
- `set_params(&mut self, freq: f32, gain_db: f32, q: f32, eq_type: u8)` — **finite-guard**
  each float (NaN/inf → a safe default, since `Eq`'s `clamp` propagates NaN), then forward to
  BOTH channels: `set_type(eq_type)`, `set_freq(freq)`, `set_gain(gain_db)`, `set_q(q)` (the
  `Eq` setters already range-clamp). Same params on both channels.
- `process(&mut self, l: &mut [f32], r: &mut [f32], dt: f32)` — per channel, IN PLACE.
  Because `Eq::process(In, dt, out)` reads an `In` and writes a *separate* `out` (so `l`
  cannot be borrowed as both input and output), add a small **additive** helper to `eq.rs`:
  `Eq::process_in_place(&mut self, buf: &mut [f32], dt: f32)` — recompute coeffs via
  `rbj_coeffs(self.ty, self.freq, self.gain, self.q, dt)`, `set_coeffs`, then `for i in
  0..buf.len() { buf[i] = self.biquad.tick(buf[i]); }` (reads index `i` before writing it →
  in-place safe). `MasterEq::process` calls `ch[0].process_in_place(l, dt); ch[1]
  .process_in_place(r, dt)`. The existing `Eq::process` stays UNCHANGED.

**No panic on any input:** the three float params finite-guarded in `set_params` before the
`Eq` clamps; `eq_type` is a `u8` mapped by `Eq::set_type` (any value → Peak default).
32-bit-`usize` safe (index `0..len`). Type codes: `0=Peak, 1=LowShelf, 2=HighShelf`.

**Scalar** — the biquad is a per-sample IIR recursion (`z1/z2` feedback), not data-parallel;
per [[prefer-neon-simd]]'s "don't force it" (same as the limiter/DC-block/existing filters).
No `f32x8` path; tests pass identically both configs.

### Engine — `crates/deluge-audio-graph/src/engine.rs` + `cmd.rs`

- **Field:** `master_eq: Option<MasterEq>` on `Engine` (init `None` in `Engine::new`; cleared
  in `Cmd::Reset`), mirroring `master_dcblock`/`master_limiter`. Import
  `use deluge_dsp_kernels::eq::MasterEq;`.
- **`Cmd::SetMasterEq { freq: f32, gain_db: f32, q: f32, eq_type: u8 }`** (new variant in
  `cmd.rs`; all-`Copy` fields satisfy `Cmd`'s `Clone,Copy,Debug,PartialEq` derives) + an
  `Engine::apply` arm (create-or-update-in-place): if `Some`, `eq.set_params(freq, gain_db,
  q, eq_type)`; if `None`, `Some(MasterEq::new(freq, gain_db, q, eq_type))`.
- **Render seam:** insert the EQ block BETWEEN the DC-block and limiter blocks (it needs
  `self.dt`, like the limiter):
  ```rust
  if let Some(root) = self.root {
      let b = root.0 as usize;
      if let Some(dc)  = &mut self.master_dcblock { dc.process(&mut self.bus_l[b], &mut self.bus_r[b]); }
      if let Some(eq)  = &mut self.master_eq      { eq.process(&mut self.bus_l[b], &mut self.bus_r[b], self.dt); }
      if let Some(lim) = &mut self.master_limiter { lim.process(&mut self.bus_l[b], &mut self.bus_r[b], self.dt); }
  }
  ```
  Disjoint field borrows. When `master_eq` is `None`, the seam is unchanged from IO-3b ⇒
  render byte-identical. The `[-1,1]` clamp that follows is UNCHANGED.

### Wren — `Out.eq(...)` (`crates/deluge-wren-core/`)

- **`audio::set_master_eq(freq: f32, gain_db: f32, q: f32, eq_type: u8)`** facade →
  `Cmd::SetMasterEq` (mirrors `audio::set_master_dcblock`).
- **`node_master_eq_impl<S>(vm)`** (`bindings_audio.rs`): `freq = get_f(1) as f32`, `gain =
  get_f(2) as f32`, `q = get_f(3) as f32`, `eq_type = get_f(4) as u8` (all `get_f` — SAFE, no
  foreign reads, no `checked_*` needed), `audio::set_master_eq(freq, gain, q, eq_type)`. A
  void config call. Plus the `#[cfg(feature="wren-sys-backend")]` extern-C shim
  `node_master_eq`.
- Register `Node.masterEq_(_,_,_,_)` (arity 4) in BOTH tables (`METHODS` in `bindings.rs` +
  `register_audio` in `bindings_audio.rs`) + the shim.
- **Prelude `class Out`** (extend the existing class) — implicit-return one-liners (a
  single-line explicit-`return` body does NOT compile — see
  [[wren-prelude-single-line-body]]), mirroring the existing `EQ` named-static convention
  (type by method name → 0/1/2):
  ```wren
  static eq(freq, gain, q) { Node.masterEq_(freq, gain, q, 0) }
  static eqLowShelf(freq, gain, q) { Node.masterEq_(freq, gain, q, 1) }
  static eqHighShelf(freq, gain, q) { Node.masterEq_(freq, gain, q, 2) }
  ```
  plus `foreign static masterEq_(freq, gain, q, type)` in `foreign class Node`.
- Usage: `Out.patch(mySynth); Out.eq(3000, 4, 1.0); Out.limit(0.9)`. Global/master control —
  no `polyMode_` guard.

## NEON / SIMD

Scalar — per-sample biquad IIR recursion (see kernel note). No forced SIMD; no data-parallel
sub-loop left un-vectorized.

## Non-breaking

Purely additive: a new `pub MasterEq` + a small additive `Eq::process_in_place` (the existing
`Eq::process` and `Kind::Eq` node UNCHANGED), one `Engine` field (`Option`, `None` default →
render byte-identical when unused), one `Cmd` variant + `apply` arm + `Reset` clear, one
`audio.rs` facade, one binding + 3 prelude methods. The IO-3a limiter, IO-3b DC-block, and the
`[-1,1]` clamp are untouched. Existing suites stay green both configs (default + `--features
deluge-dsp-kernels/simd`); the device build cross-compiles.

## Constraints (global)

- `no_std`, no heap, **no panic on any input** (params finite-guarded + `Eq`'s clamps).
  32-bit-`usize` safe.
- Stereo = two `Eq` (L/R), same params. Type codes `0=Peak, 1=LowShelf, 2=HighShelf`.
  MIT/Apache-2.0.
- Wren reads only `get_f` (safe); future foreign reads use the [[wren-binding-safety]]
  `checked_*` guards.
- The EQ runs BETWEEN the DC-block and the limiter; the final `[-1,1]` clamp REMAINS.

## Out of scope / deferred

- **Multi-band** master EQ (3-band stack, or N bands), a **dynamic/adaptive EQ**, a
  **bypass/disable `Cmd`** (opt-in-only; `Reset` clears), a per-band instance addressed by
  index, a linear-phase EQ.
- The rest of IO (IO-2 mixer/bus expansion, IO-4 multiple outputs).

## Testing

- **Kernel oracle** (`eq.rs` tests): a peak boost (+gain) at freq `f` amplifies a sine at
  `f` (settled output RMS > input RMS); a cut (−gain) attenuates it; a low-shelf boosts a low
  tone and a high-shelf boosts a high tone; flat (gain 0) passes ~unchanged; L/R get the same
  response (same params); no-panic on NaN/inf/huge/negative params. Scalar; passes BOTH crate
  configs.
- **Engine**: enable via `Cmd::SetMasterEq`, render a tone at the EQ freq → measurably
  boosted/cut; **disabled → output byte-identical**; `Cmd::Reset` clears it; the EQ composes
  with the DC-block + limiter (order `DC → EQ → limiter`).
- **Wren** (Cmd-capture): `Out.eq(1000, 6, 1.0)` emits `Cmd::SetMasterEq { freq: 1000.0,
  gain_db: 6.0, q: 1.0, eq_type: 0 }`; `Out.eqLowShelf(...)` → `eq_type: 1`;
  `Out.eqHighShelf(...)` → `eq_type: 2`.
- **e2e** (real-`EngineHost`): a synth (or a tone) through `Out.eq(...)` renders
  finite/non-silent with the boosted band amplified relative to the un-EQ'd render; existing
  synths unchanged.

## Likely task decomposition (for writing-plans)

1. **`MasterEq` kernel** (`eq.rs`): the `pub struct` wrapping `[Eq; 2]` + `new`/`set_params`
   (finite-guarded) + `process`; the additive `Eq::process_in_place` helper; oracle tests
   (peak-boost-amplifies, cut-attenuates, shelf-shape, flat-passes, L/R-same, no-panic).
   `Eq::process` and `Kind::Eq` unchanged.
2. **Engine wiring**: `master_eq: Option<MasterEq>` field + `new` init + `Reset` clear;
   `Cmd::SetMasterEq { freq, gain_db, q, eq_type }` + `apply` arm (create-or-update-in-place);
   the render seam (EQ between DC-block and limiter). Engine tests (boosted-when-enabled,
   byte-identical-when-disabled, Reset-clears, composes-with-chain).
3. **Wren `Out.eq`**: `audio::set_master_eq` facade + `node_master_eq_impl` + both tables +
   shim + prelude `class Out` methods (`eq`/`eqLowShelf`/`eqHighShelf`) + `foreign static
   masterEq_`. Cmd-capture test (all three emit the right `SetMasterEq` with type 0/1/2).
4. **e2e**: real-`EngineHost` tone-through-`Out.eq` renders with the boosted band amplified;
   non-breaking.
