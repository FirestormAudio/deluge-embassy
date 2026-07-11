# Fx-2: Lo-fi — Bitcrush / Decimate — Design Spec

**Date:** 2026-07-11
**Suite:** Fx (audio effects), sub-project Fx-2 — the first lo-fi/character
effects, after the Fx-1 dynamics pair (compressor/limiter, gate/expander).
See [[fx-suite-effects]].
**Status:** Approved — ready for implementation plan

## Goal

Two lo-fi degraders, as SEPARATE effects: `Bitcrush.new(input, bits)` reduces
amplitude bit-depth (quantization); `Decimate.new(input, rate)` reduces the
effective sample rate (sample-and-hold downsampling, `rate` in Hz). Gritty /
vintage digital character. Both mono-in/mono-out, `out_width == 1`, on the
Fx-1 `Comp`/`Drive` effect template. The simplest new effects — no
detector/ballistics.

## Background

The effect pipeline is well-established (kernel → `Kind` → Wren factory in both
tables → prelude class; `Drive`/`Comp` are the mono templates). Lo-fi is missing
entirely. Two distinct, memoryless-or-minimal-state degraders:
- **Bitcrush** — memoryless amplitude quantization: `out = round(x/step)·step`,
  `step = 1/2^(bits−1)`. Low `bits` → coarse/distorted; high `bits` (≈16) →
  transparent.
- **Decimate** — sample-rate reduction by sample-and-hold: a phase accumulator
  advances by `rate/sr` each sample; on wrap it latches a fresh input sample;
  between latches it outputs the held value. Aliasing/downsampled character.

**No anti-aliasing, no dither** — the aliasing and quantization artifacts ARE the
effect.

## Scope (Fx-2)

Two kernels (`Bitcrush`, `Decimate`) in a new `crates/deluge-dsp-kernels/src/lofi.rs`,
two graph `Kind`s, two Wren classes with a live setter each. Purely additive.

### Explicitly out of scope / deferred

- **Combined single "Crush" node** — the chosen shape is two separate effects
  (chain both to get bit+rate crushing); a combined node is not built.
- **Anti-aliasing filters, dithering, noise-shaping** — deliberately omitted
  (defeat the lo-fi character).
- **Downsample interpolation modes, stereo, sidechain.**

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP paths. `libm` for
  `roundf`/`floorf` etc. (NO std-only float methods — device build). Bounded loops.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** purely additive — a new kernel file + module line, two new
  `Kind`s, two new Wren classes. No existing kernel/node/effect/test changes.
- **Two registration tables:** the `bitcrush_`/`decimate_` factories + their
  setters register in BOTH `install_methods` (`bindings_audio.rs`) AND `METHODS`
  (`bindings.rs`), identical selectors, + prelude `foreign` decls. Both classes
  carry the `polyMode_` effect-guard.
- **Naming (collision-checked):** `Bitcrush`/`Decimate` class names are free
  (verified — the existing `.quantize` is an unrelated CV pitch-quant instance
  method; no `Crush`/`Bitcrush`/`Decimate`/`Lofi` class exists). Factory selectors
  `bitcrush_(_,_)` / `decimate_(_,_)`. **Setter names (collision-resolved):**
  `bits=` is FREE → use it for Bitcrush; `rate=` is TAKEN (an existing
  `foreign rate=(_)` instance setter `node_set_rate`, used by LFO/Chorus/Flanger,
  prelude:197) → Decimate uses **`decimateRate=`** (a dedicated
  `node_set_decimate_rate`). Do NOT reuse `node_set_rate` for Decimate (its param
  index is LFO/Chorus-specific).
- **Test invocation (per-crate, never `--workspace`; both configs; `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## Architecture

### §1 — The kernels (`crates/deluge-dsp-kernels/src/lofi.rs`, new file)

**`Bitcrush`** — memoryless amplitude quantizer:

```rust
pub struct Bitcrush { bits: f32 }
impl Bitcrush {
    pub fn new(bits: f32) -> Bitcrush { Bitcrush { bits: bits.clamp(1.0, 24.0) } }
    pub fn set_bits(&mut self, b: f32) { self.bits = b.clamp(1.0, 24.0); }
    pub fn process(&mut self, input: In, _dt: f32, out: &mut [f32]) {
        let step = 1.0 / libm::powf(2.0, self.bits - 1.0); // 1/2^(bits-1)
        for i in 0..out.len() {
            let x = input.at(i);
            out[i] = libm::roundf(x / step) * step;
        }
    }
}
```

`bits` clamped `[1, 24]`; `bits == 1` → step 1 → ~3 levels (−1/0/+1); high `bits`
→ near-transparent. No state.

**`Decimate`** — sample-and-hold rate reducer:

```rust
pub struct Decimate { rate_hz: f32, held: f32, phase: f32 }
impl Decimate {
    pub fn new(rate_hz: f32) -> Decimate { Decimate { rate_hz: rate_hz.max(1.0), held: 0.0, phase: 1.0 } }
    pub fn set_rate(&mut self, hz: f32) { self.rate_hz = hz.max(1.0); }
    pub fn process(&mut self, input: In, dt: f32, out: &mut [f32]) {
        let inc = (self.rate_hz * dt).min(1.0); // rate/sr, capped at 1 (>= sr ⇒ passthrough)
        for i in 0..out.len() {
            self.phase += inc;
            if self.phase >= 1.0 {
                self.phase -= 1.0;
                self.held = input.at(i); // latch a fresh sample
            }
            out[i] = self.held;
        }
    }
}
```

`rate_hz` clamped `≥ 1`; `phase` init 1.0 so the first sample latches; `inc`
capped at 1.0 so `rate ≥ sr` is a clean passthrough (latches every sample) and
phase never overshoots (single `-= 1.0` suffices). Between latches the held value
is repeated → downsampled/aliased.

### §2 — Graph nodes (`crates/deluge-audio-graph/src/node.rs`)

Two width-1 nodes, one signal input (port 0), on the `Comp`/`Drive` template:
- `Kind::Bitcrush`, `State::Bitcrush(Bitcrush)`, `set_param(0) = bits`, process arm
  `b.process(ins[0], dt, outs.port(0))` (dt ignored by Bitcrush).
- `Kind::Decimate`, `State::Decimate(Decimate)`, `set_param(0) = rate`, process arm
  `d.process(ins[0], dt, outs.port(0))` (Decimate USES dt).

Constructor defaults: `Bitcrush::new(8.0)`, `Decimate::new(8000.0)`. Both
`out_width == 1` (default arm). Both derive `Clone, Copy` (node `State` needs it).

### §3 — Wren surface (`bindings_audio.rs`, `bindings.rs`, `prelude.wren`)

- **`node_bitcrush_impl`** factory: `bitcrush_(input, bits)` → `new_node(Kind::Bitcrush)`,
  `set_param(0, bits)`, `return_node`. **`node_decimate_impl`**:
  `decimate_(input, rate)` → `Kind::Decimate`, `set_param(0, rate)`. Paired
  `extern "C"` shims. Register both + their setters in BOTH tables.
- **Setters:** `bits=` (index 0 on Bitcrush — free) and `decimateRate=` (index 0
  on Decimate — `rate=` is taken by LFO/Chorus), each a `set_param`-index setter
  with its own `extern "C"` shim.
- **prelude** (each with the `polyMode_` effect-guard):
  - `class Bitcrush`: `Bitcrush.new(input, bits)` → `Node.bitcrush_(input, bits)`.
  - `class Decimate`: `Decimate.new(input, rate)` → `Node.decimate_(input, rate)`.

## Data Flow

```
Bitcrush.new(src, bits) → bitcrush_(src, bits) → Kind::Bitcrush node
  per sample: out = round(x / step) * step,  step = 1/2^(bits-1)   (memoryless)

Decimate.new(src, rate) → decimate_(src, rate) → Kind::Decimate node
  per sample: phase += min(rate/sr, 1); on wrap latch held = x; out = held

Chain both for full lo-fi: Out.patch(Bitcrush.new(Decimate.new(Osc.saw(110), 6000), 6))
```

## Error Handling

`bits` clamped `[1, 24]`, `rate ≥ 1` at the setter boundary; `libm::powf`/`roundf`
bounded (bits in range); `inc` capped at 1.0; `phase` bounded `[0, 2)`. No heap,
no panics, `no_std`. `bits == 24` ≈ transparent; `rate ≥ sr` ≈ passthrough.

## Testing

Both feature configs, per-crate. Oracle-driven.

1. **Bitcrush quantizes** (kernel): `bits = 1` → every output is a multiple of
   `step = 1` (i.e. in {−1, 0, +1}); an input `0.7` → `round(0.7)·1 = 1.0`, input
   `0.3` → `0.0`. `bits = 16` (step 1/32768) → output ≈ input within one step
   (near-transparent). Assert outputs are integer multiples of `step`.
2. **Decimate holds** (kernel): pick `rate = sr/4` so `inc = 0.25` → latches every
   4 samples. Feed a distinct-per-sample ramp (`input[i] = i`); assert the output
   is piecewise-constant in groups of ~4 (`out[k] == out[k+1] == …` within a hold,
   changing at the latch boundary), and each held value equals the input at the
   latch sample. `rate ≥ sr` → passthrough (`out[i] == input[i]`).
3. **Graph nodes** (node.rs): `Kind::Bitcrush`/`Kind::Decimate` `out_width == 1`,
   `set_param(0)` drives bits/rate, process arm degrades the signal (bitcrush:
   low bits quantizes; decimate: low rate holds).
4. **Wren e2e** (`tests/audio_bindings.rs`): `Out.patch(Bitcrush.new(Osc.saw(110),
   3))` renders finite/bounded/non-silent AND the output takes only a few distinct
   quantized values (assert a small distinct-value count, or that all samples are
   multiples of the 3-bit step); `Out.patch(Decimate.new(Osc.saw(110), 4000))`
   renders finite/bounded/non-silent AND is piecewise-constant (fewer distinct
   values / repeated samples vs the dry saw); chaining both builds+renders.
   Existing synths/effects unchanged.

## Success Criteria

- `Bitcrush.new(src, bits)` quantizes amplitude to `2^bits`-ish levels (audible
  grit at low bits, transparent at high); `Decimate.new(src, rate)` reduces the
  effective rate by sample-and-hold (aliased/chunky at low rate, transparent at
  `rate ≥ sr`).
- Live setters update the running effect.
- Purely additive: everything not using them is byte-unchanged.
- Both feature configs green, per-crate.
