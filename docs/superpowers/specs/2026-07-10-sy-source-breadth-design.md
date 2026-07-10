# Sy-2d: Source Breadth — Design Spec

**Date:** 2026-07-10
**Suite:** Sy (synth/voice), sub-project Sy-2d
**Status:** Approved — ready for implementation plan

## Goal

Close every remaining voice-**source** `Fiber.abort` inside a Wren `Synth`
builder — poly hard-sync oscillator, poly wavetable (single-cycle + 2D morph),
per-voice PWM, and pink/brown poly noise — so a Synth voice can use any
oscillator or noise color the mono API offers.

## Background

After Sy-2c, inside a `Synth` these poly sources/filters work: `Osc.sine/saw/
square/tri`, `+`, `Noise.new` (white), `.lpf` (PolySvf), `Moog.lp/lp2`,
`Ms20.lp/hp`, `Env.ar`, `*`. Still guarded with `Fiber.abort` in poly mode:
`Osc.syncSine/Saw/Square/Tri`, `Osc.wavetable`, `Noise.pink/brown`, and per-voice
PWM (the `.width =` setter on the square oscillator).

Sy-2d poly-ifies these four source families. It also adds one cross-cutting
engine capability the PWM (and sync) plumbing needs: **mono→poly broadcast**.

## Scope

Four source families in one pass, plus the broadcast foundation:

1. **Mono→poly broadcast** (foundation) — engine poly-input resolution.
2. **Pink/brown poly noise** — cheap, scalar.
3. **Per-voice PWM** — width becomes a poly input port on PolyOsc.
4. **Poly hard sync** — band-limited, f32x8.
5. **Poly wavetable** (single-cycle + 2D morph) — scalar-per-voice.

### Explicitly out of scope (still abort in a Synth)

- **Tb303 / Modal poly filters** — deferred from Sy-2c (register-pressure /
  physical-model reasons); still abort.
- **`Bus.write(src)` poly footgun** — a signal-input routing method on a distinct
  Rust path (`bus_write_impl`), not the `Node.<x>_()` factory pattern; unassessed
  for poly. Deferred to a future pass; document as a known gap.
- **Per-voice morph position** — the wavetable 2D-morph `position` is a *shared
  mono* control in Sy-2d (the natural sweep). Per-voice morph is Sy-5-style
  expressiveness.
- **f32x8 poly wavetable** — per-voice mip-level divergence needs a table-row
  gather the A9 NEON lacks; Sy-2d ships scalar-per-voice (see §5).

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP paths. ARM Cortex-A9
  (VFPv3+NEON) target; host x86 for tests only. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **SIMD convention:** the scalar path is the correctness oracle; the
  `#[cfg(feature = "simd")]` f32x8 fast path is null-tested lane-for-lane to
  `≤ 1e-4`. Doc comments attach to the struct, never to a cfg'd const guard.
  `const _: () = assert!(VOICES == 8)` guards f32x8 (the existing guard covers
  new poly.rs code — do not add a duplicate).
- **Poly control convention:** leading ports are poly edges (`poly_in_count` =
  their count); trailing controls are shared-mono `In`, splatted per sample.
- **Test invocation (per-crate, never `--workspace`; both feature configs):**
  `cargo test --target x86_64-unknown-linux-gnu -p <crate>` and again with
  `--features simd` (kernels) / `--features deluge-dsp-kernels/simd` (graph, wren).
  cargo rejects multiple bare positional test names — use `-- name1 name2`.

## Architecture

### §0 — Mono→poly broadcast (foundation)

**File:** `crates/deluge-audio-graph/src/engine.rs` (poly-input resolution,
~:203-215).

Today the poly-input path copies `VOICES` consecutive rows verbatim from a
source node. A source whose `out_width == 1` (a mono producer) thus reads
`VOICES` rows starting at its single output row — dropping into adjacent nodes'
rows (garbage). Sy-2d adds a broadcast branch: **if the source node's
`out_width == 1`, splat its single output row to all `VOICES` lanes** of the
`poly_scratch[j]` tile; otherwise copy `VOICES` rows as today.

This makes any mono/constant signal usable as a poly input. It is required so a
shared mono LFO can drive the per-voice PWM width port (§3), and lets a
constant/mono master frequency drive poly sync (§4).

**Test:** a width-1 producer feeding a poly input port yields identical values on
all `VOICES` lanes of the consumer's input tile (direct broadcast test).

### §1 — Pink/brown poly noise

**Files:** `crates/deluge-dsp-kernels/src/poly.rs`;
`crates/deluge-audio-graph/src/node.rs`.

Extend `PolyNoise` (Sy-2b, `rng: [u32; VOICES]`, white) with color:

- Add per-voice colored state: `pink: [[f32; 7]; VOICES]` (Paul-Kellet refined
  bank `b0..b6`) and `brown: [f32; VOICES]` (leaky integrator), plus a `color:
  NoiseColor` selector. Each voice runs the exact mono `noise.rs` recurrence over
  its own xorshift32 lane.
- **Scalar-only** (serial IIR recurrences — the mono kernel is documented
  "serial recurrence → scalar"; no f32x8). `poly_in_count 0` (pure source).
- **Oracle:** the mono `Noise` per voice, bit-matched (the existing white lane
  already bit-matches `noise.rs`; pink/brown lanes match identically).
- **Graph:** color-carrying Kinds `PolyNoise` (white), `PolyPink`, `PolyBrown`
  for clean Wren routing (mirrors how the mono side selects color); each
  `out_width VOICES`, `is_poly`, `poly_in_count 0`.

### §2 — Per-voice PWM

**Files:** `crates/deluge-dsp-kernels/src/osc.rs` (`wave_sample_x8`);
`crates/deluge-dsp-kernels/src/poly.rs` (`PolyOsc`);
`crates/deluge-audio-graph/src/node.rs`.

- `wave_sample_x8` (currently hardcodes width 0.5) gains a `width: f32x8` arg.
  The scalar `wave_sample` already takes `width`. Square uses it
  (`<= 0 ⇒ 0.5`, else clamp `[0.01, 0.99]`); Sine/Saw/Tri ignore it.
- `PolyOsc::process` gains a `width: &[f32]` voice-interleaved tile (port 1).
  `PolyOsc` becomes `poly_in_count 2` (pitch = port 0, width = port 1). `wave` /
  `shape` stays node config.
- **Backward-compat:** an unconnected width port resolves to `Const(0.0)` →
  broadcast → `width 0` → the `<= 0 ⇒ 0.5` convention. Existing PolyOsc voices
  (all shapes) are **bit-unchanged**. This is a required regression gate.
- **Oracle/null-test:** (a) width unconnected ⇒ output bit-identical to the
  pre-Sy-2d PolyOsc for every shape; (b) swept per-voice width ⇒ each lane
  matches the scalar `wave_sample(Square, phase, dtp, width_v)` to `≤ 1e-4`.
- **Graph:** `poly_process` passes `poly_in[1]` as width. **Wren:** the `.width =`
  setter, in poly mode, writes the PolyOsc width poly-input port (fed via §0
  broadcast from a mono LFO, or directly from a poly source for true per-voice
  PWM).

### §3 — Poly hard sync

**Files:** `crates/deluge-dsp-kernels/src/osc.rs` (extract `SyncOsc::tick`, add
`naive_wave_x8`); `crates/deluge-dsp-kernels/src/poly.rs` (`PolySync`);
`crates/deluge-audio-graph/src/node.rs`.

- New `PolySync` kernel. State SoA: `master_phase: [f32; VOICES]`,
  `slave_phase: [f32; VOICES]` (2 f32/voice, register-light). Two poly frequency
  inputs (master + slave Hz) → `poly_in_count 2`. `wave` (Sine/Saw/Square/Tri) is
  node config → four Kinds (`PolySyncSine/Saw/Square/Tri`), mirroring the mono
  `Sync` wave-code kinds.
- **Scalar oracle:** extract `SyncOsc::tick(&mut self, wave, master_freq: f32,
  slave_freq: f32, dt: f32) -> f32` from the current per-sample loop body of
  `SyncOsc::process`. Mono `SyncOsc::process` calls it (behavior-preserving,
  proven by existing `sync_*` tests). Scalar poly path holds `[SyncOsc; VOICES]`
  and calls `tick` per voice.
- **f32x8:** vectorize the sync sample across voices with the branchless-select
  pattern (Sy-2b `wave_sample_x8` osc idiom). The master-wrap condition
  `mp >= 1.0 && dtp_m > 0.0` becomes a per-lane mask; both the reset path (new
  `slave_phase`, the reset-BLEP term `y += 0.5·step·poly_blep(mp_before,dtp_m)`)
  and the no-reset path are computed for all lanes and combined with `select`.
  Reuses `wave_sample_x8` (slave natural-wrap band-limiting), `poly_blep_x8`
  (reset-BLEP), `floor_x8` (Sy-2b); adds a small branchless `naive_wave_x8` (the
  un-band-limited wave value, used for the reset `step`).
- **Oracle/null-test:** f32x8 lane == scalar `SyncOsc::tick` per voice `≤ 1e-4`;
  plus the existing `sync_saw_is_band_limited` / `sync_all_waves_band_limited`
  alias-floor gates applied to a poly lane (band-limiting preserved).

### §4 — Poly wavetable (single-cycle + 2D morph)

**Files:** `crates/deluge-dsp-kernels/src/poly.rs` (`PolyWt`);
`crates/deluge-audio-graph/src/node.rs`;
`crates/deluge-wren-core/src/bindings_audio.rs` (table plumbing).

- New `PolyWt` kernel. State: `phase: [f32; VOICES]` (1 f32/voice). The mip
  pyramid is a **shared borrowed `MipSet`** passed at process time (per-voice
  phase, one shared table — the clean poly fit). pitch = poly-in (port 0);
  `pmod` and morph `position` are **shared-mono** `In` controls. `poly_in_count 1`.
- **Scalar-per-voice, deliberately not f32x8.** Each voice's pitch selects a
  different mip level (`floor(log2(dtp·N))`); an f32x8 lookup across voices would
  need a per-lane table-row gather the A9 NEON has no cheap instruction for. So
  `PolyWt` holds `[WtOsc; VOICES]` and reuses the audited `WtOsc::process`
  (single-cycle) / `WtOsc::process_morph` (2D morph) per voice — correct, no
  gather. The mono `simd8` path is untouched (still serves the mono const-freq
  case). f32x8 poly wavetable is a future revisit only if profiling demands it.
- **Table resource:** the poly wavetable node holds the same `TableId` / pooled
  handle the mono node does, resolving to a `MipSet` at process time; all voices
  share it. Static and pooled (user-uploaded) tables both supported, preserving
  the mono `Osc.wavetable` static-vs-pooled dispatch.
- **Oracle/null-test:** `PolyWt` lane `v` == mono `WtOsc` fed voice `v`'s pitch,
  bit-identical (same scalar kernel), for both single-cycle and morph.

### §5 — Wren surface

**Files:** `crates/deluge-wren-core/src/bindings_audio.rs`, `bindings.rs`,
`wren/prelude.wren`.

Flip each class route on `polyMode_ == 1` (the Sy-2c pattern), add the Rust
factories + registration + `foreign static` prelude decls (the Sy-2c gotcha —
a missing `foreign static` errors "metaclass does not implement"):

- **Noise:** `Noise.pink()`/`Noise.brown()` → `polypink_()`/`polybrown_()`.
- **Sync:** `Osc.syncSine/Saw/Square/Tri(master, slave)` → `polysync_(wave,
  master, slave)`.
- **Wavetable:** `Osc.wavetable(t, f)` → `polywt_(table, freq, ...)` /
  `polywt_pooled_(wt, freq, ...)`, preserving static-vs-pooled dispatch; a
  2D-morph table routes the morph path with its `position` control.
- **PWM:** the `.width =` setter, in poly mode, writes the PolyOsc width poly
  input port.

## Data Flow

A poly sync-lead voice:
```
PolyCtrl(pitch) → PolyMtof → { master: ×ratio, slave: pitch } → PolySync
    → PolyMoog(cutoff,res) → PolyMul(·, PolyAr) → VoiceSum → out
```
A PWM voice: a mono LFO → (§0 broadcast) → PolyOsc width port; pitch poly-in →
`PolyOsc(Square, width)` → filter/amp → VoiceSum.

`PolySync` consumes two poly Hz tiles (master, slave); `PolyOsc` consumes pitch +
width tiles; `PolyWt` consumes a pitch tile + shared table + mono pmod/position;
`PolyNoise`/`PolyPink`/`PolyBrown` are pure sources. All produce `VOICES`-wide
poly edges.

## Error Handling

New failure modes are misuse only, handled by the existing `Fiber.abort` guard
pattern (deferred features — Tb303/Modal, Bus, per-voice morph — stay aborting).
No new runtime error paths in the DSP: kernels are total, inputs clamped, outputs
bounded by construction.

## Testing

All tests in both feature configs (`default` and `--features …simd`).

1. **Engine broadcast** (`engine.rs`): a width-1 producer feeding a poly input
   port yields identical values across all `VOICES` lanes.
2. **Kernel null-tests** (`poly.rs`/`osc.rs`):
   - Poly pink/brown bit-match the mono `Noise` per lane (both colors).
   - `PolyOsc` with width unconnected ⇒ bit-identical to pre-Sy-2d output (all
     shapes); with swept per-voice width ⇒ each lane matches scalar `wave_sample`.
   - `PolySync` f32x8 == scalar `SyncOsc::tick` per lane `≤ 1e-4`, both configs;
     the `sync_*_band_limited` alias-floor gate holds on a poly lane.
   - `PolyWt` lane == mono `WtOsc` per voice, single-cycle and 2D morph.
3. **Graph render** (`node.rs`): a poly voice through each new source
   (`PolySync*`, `PolyWt`, `PolyPink`, PWM `PolyOsc`) renders finite, bounded,
   non-silent.
4. **Wren end-to-end** (`tests/audio_bindings.rs`): a `Synth` using `Osc.syncSaw`,
   `Osc.wavetable` (static + pooled + a morph table), `Noise.pink`, and a PWM
   square (`Osc.square(p)` with `.width =` a mono LFO via broadcast, and a poly
   source) each produce bounded, non-silent sound. Deferred cases
   (`Tb303`/`Resonator`, `Bus.write`) stay aborting.

## Success Criteria

- A playable poly sync-lead, wavetable, PWM, and pink/brown-noise voice from
  Wren, e.g. `Synth.new { |p| Osc.syncSaw(p, p*1.5).lpf(2000) * Env.ar(0.01,0.4) }`
  and `Synth.new { |p| Osc.wavetable(WT.saw, p) * Env.ar(0.01,0.3) }`.
- f32x8 paths (sync, PWM osc) bit-close (`≤ 1e-4`) to the scalar oracle,
  lane-for-lane; scalar-per-voice paths (noise, wavetable) bit-identical to the
  mono kernel per voice.
- Existing PolyOsc voices bit-unchanged (width-port backward-compat gate).
- Mono→poly broadcast works: a mono source drives a poly input port on all lanes.
- Both feature configs green, per-crate.
