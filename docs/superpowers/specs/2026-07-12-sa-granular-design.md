# Poly Granular (Sa-4) — Design Spec

**Date:** 2026-07-12
**Status:** Approved (design), pending implementation plan
**Sub-project:** Sa suite — Sa-4 granular synthesis over in-RAM sample buffers.

## Context

Granular synthesis plays a **cloud of overlapping short windowed grains** taken
from a sample, giving textures, time-stretch/scrub feels, and pitched pads. This
sub-project adds a **poly** granular voice over the existing in-RAM
`SampleBuffer` (Sa-1's pooled PCM) — reusing the `BindTable{Pooled}`/`pool_region`
path and `hermite_read` from `sampler.rs`, and the poly voice model (VOICES,
per-voice pitch tile). Fully host-testable (in-RAM, no SD). See
[[sa-suite-samples]], [[sy-suite-voice-model]], [[prefer-neon-simd]].

**Model (chosen):** the note's pitch transposes the grains
(`rate = hz/mtof(root)`); `position`/`size`/`density`/`spray` are shared knobs;
**8 grains/voice** (8×8 = 64 concurrent grains max) balances richness vs A9 CPU.

## Design

### Kernel — `crates/deluge-dsp-kernels/src/granular.rs` (new, pure `no_std`, scalar)

Reuses `hermite_read` from `sampler.rs` (make it `pub(crate)`). Scalar — per-grain
reads are gathers that don't vectorize (like `PolyWt`/`PolySamplePlayer`; the
[[prefer-neon-simd]] "don't force it" clause).

```rust
const MAX_GRAINS: usize = 8;

#[derive(Clone, Copy)]
struct Grain {
    active: bool,
    pos: f32,        // read position in the buffer (samples)
    rate: f32,       // grain playback rate (note transpose)
    phase: f32,      // 0..1 through the grain window (envelope)
    phase_inc: f32,  // 1 / grain_size_samples
}

#[derive(Clone, Copy)]
struct GrainCloud {
    grains: [Grain; MAX_GRAINS],
    rng: u32,          // xorshift32 for spray jitter (seeded per voice)
    spawn_accum: f32,  // fractional grains accumulated (density * dt)
    playing: bool,
    // params (shared across this voice's grains):
    position: f32,     // 0..1 (fraction of buffer)
    size_ms: f32,      // grain duration
    density: f32,      // grains/sec
    spray: f32,        // 0..1 position jitter fraction
    root: f32,         // MIDI note at which a grain plays at rate 1
}

pub struct PolyGranular { voices: [GrainCloud; VOICES] }
```

`process_voice(&mut self, v, pcm: &[f32], hz: In, dt, out: &mut [f32])`:
- `rate = hz.at(i) / mtof(root)` (per voice — reuse a shared `hz_to_rate(hz, root)`
  helper, factored from `PolyStreamPlayer::rate_for`/`PolySamplePlayer`).
- Per output sample:
  1. **Schedule:** `spawn_accum += density * dt`; while `spawn_accum >= 1.0`,
     `spawn_accum -= 1.0`, spawn a grain into a free slot (skip if none free —
     grain drop, no steal). Spawn: `start = position*len + (xorshift→[-1,1)) *
     spray * len` (clamped to `[0, len)`); `pos = start`; `grain.rate = rate`;
     `phase = 0`; `phase_inc = 1.0 / (size_ms/1000 * sr)` (≥ a small floor to
     avoid div-by-0 / infinite grains).
  2. **Mix:** `out[i] = Σ_active hann(g.phase) * hermite_read(pcm, g.pos, 0, len,
     false)` where `hann(p) = 0.5 - 0.5*cosf(2π*p)` (libm) — clamp reads at buffer
     edges (`loopable=false`).
  3. **Advance:** each active grain `g.pos += g.rate; g.phase += g.phase_inc; if
     g.phase >= 1.0 { g.active = false }`.
- **No panic on any input:** `len == 0`/`!playing` → silence; `density`/`size`/
  `spray` clamped to sane ranges (density ≥ 0, size ≥ ~1 sample, spray ∈ [0,1]);
  grain `pos` clamped into `[0, len)` at spawn; `hermite_read` already OOB-safe.
  32-bit-safe indexing (`hermite_read`'s existing discipline).

Setters: `set_root`/`set_position`/`set_size`/`set_density`/`set_spray`;
`trigger_voice(v)` = seed `rng` (per-voice, nonzero), reset `spawn_accum`, clear
grains, `playing = true`. (No explicit stop — the amp env, multiplied in the
graph, shapes the note; the cloud spawns continuously while `playing`, like
`PolySamplePlayer`.)

### Graph — `Kind::PolyGranular` (mirrors `Kind::PolySamplePlayer`)

`node.rs`: `Kind::PolyGranular`; `State::PolyGranular(PolyGranular)`; predicates
(`out_width == VOICES`, `is_poly`, `poly_in_count == 1` = pitch) alongside
`PolySamplePlayer`; `Node::trigger_voice` arm → `p.trigger_voice(v)`; `set_param`
scheme `0=root, 1=position, 2=size, 3=density, 4=spray`; the `poly_process` arm
mirrors `Kind::PolySamplePlayer` (de-interleave `poly_in[0]` per lane, read the
shared `pool_region` as raw PCM, call `process_voice`, re-interleave).

### Wren — `Granular.new(pitch, source)` (mirrors `Sample.new`)

A poly voice source used inside `Synth.new{}`: `class Granular { static
new(pitch, buffer) { polyMode_ guard; return Node.granular_(pitch, buffer) } }`;
`node_granular_impl` reads `pitch = arg_input`, the `SampleBuffer` handle via the
tag-dispatch (`checked_tagged_foreign::<SampleObj>` — the wren-binding-safety
guard, like `Sample.new`), `alloc`-less (reuses the buffer's existing pool
handle), `new_poly_granular(id, handle, pitch)` (`NewNode` + `BindTable{Pooled}`),
`poly_record_trigger(id)`, `return_poly_node`. Setters `position=`/`size=`/
`density=`/`spray=` (+ `root=` reuse) → `SetParam(node, slot, v)`. Register both
tables + prelude `class Granular`. See [[wren-binding-safety]] for the foreign
read guard.

## NEON / SIMD

Scalar — the per-grain windowed Hermite reads are gathers (like `PolyWt`
scalar-per-voice). The Hann `cosf` + the mix loop are per-sample scalar. Per
[[prefer-neon-simd]]'s "where possible / don't force it" — granular's inner loop
isn't data-parallel across a fixed lane width (variable active-grain count,
per-grain gather positions), so it stays scalar. (A future optimization could
precompute a Hann table to drop `cosf`.)

## Non-breaking

Purely additive — a new kernel module, a new `Kind`/`State` variant, a new
binding/prelude class. `SampleBuffer`/`SamplePlayer`/`PolySamplePlayer` and every
existing node unchanged. Existing suites stay green both configs.

## Testing

- **Kernel oracle** (`granular.rs`): a triggered cloud over a known ramp buffer
  renders finite/bounded/non-silent; `density=0` → no grains → silence;
  `hann(0)==0`, `hann(0.5)==1` window shape; a single grain (density tuned to one
  grain) reproduces a windowed slice of the buffer at the right position/rate;
  `spray` moves grain start positions (deterministic via the seeded xorshift);
  `rate` (note one octave up) doubles the grain read speed; no-panic on `len==0`,
  huge/zero density/size, `spray > 1`. Scalar path is the oracle (no simd path —
  gather; but tests pass in BOTH crate configs since the crate builds both ways).
- **Graph** (`node.rs`): `Kind::PolyGranular` predicates; `poly_process` reads the
  pool region + de-interleaves pitch; `set_param` slots reach the right setters;
  `trigger_voice` starts a voice's cloud.
- **e2e** (`tests/audio_bindings.rs`): `Synth.new { |p| Granular.new(p,
  SampleBuffer.from([...])) * Env.adsr(...) }` renders finite/bounded/non-silent +
  polyphonic (two notes); `density`/`position`/`spray` setters build + audibly
  change the render; existing synths unchanged.

## Constraints (global)

- `no_std`, no heap, **no panic on any input** (empty buffer, zero/huge
  density/size, spray out of range, out-of-range voice). 32-bit-`usize` safe (grain
  positions via `hermite_read`'s existing OOB-safe discipline — see
  [[target-32bit-usize-overflow]]).
- VOICES == 8, MAX_GRAINS == 8. MIT/Apache-2.0. Scalar (gather).
- Both Wren registration tables + prelude; new foreign reads use the `checked_*`
  slotapi guards.

## Out of scope / deferred

- Multiple window shapes (Hann fixed this sub-project; Tukey/Gaussian later);
  per-grain pitch spray; grain reverse; stereo grain pan; a precomputed Hann table
  (perf); granular over the STREAMING ring (Sa-3b) or keymaps (Sa-2) — this is
  over a single `SampleBuffer`; grain-steal (drop-on-full this slice).

## Likely task decomposition (for writing-plans)

1. `PolyGranular` kernel (`granular.rs`): `Grain`/`GrainCloud`/`PolyGranular`,
   `process_voice` (schedule + Hann mix + advance), setters, `trigger_voice`,
   xorshift spray, `hz_to_rate`; oracle tests (bounded/non-silent, density=0
   silence, window shape, single-grain slice, spray-moves-start, rate-octave,
   no-panic). Make `hermite_read` `pub(crate)`.
2. `Kind::PolyGranular` graph node (mirror `PolySamplePlayer`): Kind/State/
   predicates/trigger_voice/set_param(0..4)/poly_process arm; node tests.
3. Wren `Granular.new` + setters (mirror `Sample.new`): `node_granular_impl` +
   `new_poly_granular` facade + both tables + prelude `class Granular`; binding
   tests.
4. e2e: poly granular renders in a Synth (bounded/non-silent/polyphonic) + setters
   change the render; non-breaking.
