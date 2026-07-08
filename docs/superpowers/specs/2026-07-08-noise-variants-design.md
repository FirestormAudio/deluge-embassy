# Noise variants — pink & brown — design & spec

Completes the deferred [Osc](2026-07-07-osc-antialiased-oscillators-design.md) "noise
variants" item: add **pink** (−3 dB/oct) and **brown** (−6 dB/oct) noise alongside the
existing **white** (`Noise`, xorshift32). Pink via the **Paul Kellet refined filter**;
brown via a **leaky integrator**.

> **Status:** design proposal. Depends on merged P0 (`Noise` kernel + `Kind::Noise`
> node) and QA (`deluge-dsp-test` spectrum). Self-contained: `noise.rs` + two node
> kinds + Wren factories + a new spectral-slope QA metric.

---

## 1. Goals & non-goals

**Goals**
- **Pink noise** (−3 dB/oct, 1/f): white filtered by the Paul Kellet refined 7-section
  IIR (~±0.5 dB from ideal, 20 Hz–20 kHz), normalized to ~`[-1, 1]`.
- **Brown noise** (−6 dB/oct, 1/f², Brownian): white through a leaky integrator (bounded
  random walk), normalized to ~`[-1, 1]`.
- **White unchanged (transparency):** `Kind::Noise` keeps producing the exact current
  xorshift32 white output — the existing noise tests/determinism are byte-identical.
- **QA-proven slope:** a new spectral-slope metric gates each color's dB/oct (≈0 / −3 /
  −6 within a measured tolerance); all colors bounded + deterministic (seeded).

**Non-goals**
- Blue/violet/grey noise (out of scope; the framework extends later).
- Audio-rate parameterized "color" knob (fixed colors as node kinds this cut).
- Perfect −3 dB/oct at spectrum extremes (Kellet's small deviation is accepted).

---

## 2. Kernel (`deluge-dsp-kernels/src/noise.rs`)

`Noise` gains a color + the per-color state; `process` branches on color. The **white
path is byte-identical** to today (same xorshift32).

```rust
#[derive(Clone, Copy, PartialEq)]
pub enum NoiseColor { White, Pink, Brown }

pub struct Noise {
    rng: u32,          // xorshift32 (unchanged white source)
    color: NoiseColor,
    pink: [f32; 7],    // Kellet filter state (b0..b6)
    brown: f32,        // leaky integrator state
}

impl Noise {
    pub fn seeded(seed: u32) -> Noise;                       // White (unchanged API + output)
    pub fn seeded_color(seed: u32, color: NoiseColor) -> Noise;
    pub fn process(&mut self, out: &mut [f32]);              // branches on self.color
}
```

Per sample: draw `white` from xorshift32 (the current `(r as i32 as f32)/i32::MAX` in
`[-1,1)`), then:
- **White:** `out = white` (unchanged).
- **Pink (Kellet refined):**
  ```
  b0 = 0.99886*b0 + white*0.0555179;
  b1 = 0.99332*b1 + white*0.0750759;
  b2 = 0.96900*b2 + white*0.1538520;
  b3 = 0.86650*b3 + white*0.3104856;
  b4 = 0.55000*b4 + white*0.5329522;
  b5 = -0.7616*b5 - white*0.0168980;
  pink = (b0+b1+b2+b3+b4+b5+b6 + white*0.5362) * PINK_GAIN;
  b6 = white*0.115926;
  out = pink;
  ```
  `PINK_GAIN` normalizes to ~`[-1,1]` (Kellet's raw output is ~±several) — **set from
  measurement** (§4).
- **Brown (leaky integrator):**
  ```
  brown = (brown + white * BROWN_RATE) * BROWN_LEAK;  // leak < 1 bounds the walk
  out = (brown * BROWN_GAIN).clamp(-1.0, 1.0);
  ```
  `BROWN_RATE`/`BROWN_LEAK`/`BROWN_GAIN` set the −6 dB/oct slope + bound + normalization
  — **from measurement** (§4). The clamp is a safety net (rare walk excursions), not the
  primary bound (the leak is).

`no_std` (`libm` if needed; the filters are plain arithmetic). Deterministic (seeded, no
RNG beyond xorshift).

---

## 3. Graph & Wren surface

**`deluge-audio-graph`:**
- Add `Kind::PinkNoise`, `Kind::BrownNoise`. `Node::new` maps them to
  `State::Noise(Noise::seeded_color(seed, Pink|Brown))`; `Kind::Noise` stays White.
- The render arm is **unchanged** — it already calls `nz.process(out)`, which now
  dispatches on the node's color. No new dispatch logic.

**`deluge-wren-core`:**
- `Osc.pink()` / `Osc.brown()` (pure-Wren sugar, or `Noise.white()/pink()/brown()`) →
  `Node.noise_(color)` (extend the existing `node_noise_impl` to take a color code) OR
  new `Node.pink_()/brown_()` static factories. Existing `Osc.noise()` (white) unchanged.
- Mirror the existing noise binding's 4-place registration.

---

## 4. QA acceptance & testing

**New spectral-slope metric** (noise has no harmonics, so `worst_alias_db` doesn't
apply). Add a `deluge_dsp_test::spectrum` helper (or in-test): FFT the noise, average the
magnitude spectrum into log-spaced (octave) bands, and least-squares fit `dB` vs
`log2(freq)` → **slope in dB/octave**. Because noise is stochastic, average over multiple
blocks (or a long capture) to reduce variance; report the estimate + its spread.

- **Slope gates (measured tolerance):** White ≈ 0 dB/oct; Pink ≈ −3; Brown ≈ −6 — each
  within a tolerance SET FROM the measured estimate's variance (measure, don't predict).
  Record the measured slopes.
- **Normalization/bounds:** each color's output is finite and within `[-1, 1]` (brown
  via leak+clamp; pink via `PINK_GAIN`) over a long capture — set `PINK_GAIN`/brown
  constants so the RMS/peak sits in a sensible range (document the measured peak).
- **White byte-identical (transparency):** `Kind::Noise` / `Noise::seeded` output equals
  the pre-change white noise exactly — the existing noise proptest/determinism test
  passes unchanged, and a direct comparison against a saved white reference block.
- **Determinism:** seeded → reproducible for all three colors (host & device).
- **Wren:** `Osc.pink()`/`Osc.brown()` create the right kinds (Cmd-sequence test) and
  render bounded/non-silent with the expected slope (a render test).

**Determinism note:** all three are deterministic given the seed; slopes reproduce across
host/device (fixed constants, no platform trig in the filters).

---

## 5. Deferred / follow-ups

- Blue/violet/grey noise; a continuous color knob.
- Higher-order pink filters if a tighter slope is wanted.
- (Tracked elsewhere:) hard sync; async device upload of large dynamic wavetable banks.
