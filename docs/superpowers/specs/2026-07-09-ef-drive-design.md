# Ef-4: Drive (waveshaper distortion) — design & spec

The fourth Ef sub-project: a **multi-shape waveshaper/distortion** — soft / hard /
fold / tube characters with **4× oversampling** for anti-aliasing, a tone filter,
and dry/wet mix. A small per-sample kernel (no pooled buffer, mono in/out) like
the Fi filters; reuses `pade_tanh` (Fi) and the `Svf` LP primitive (as the
decimation filter).

> **Status:** design proposal. Depends on the merged `pade_tanh`
> (`filter.rs`, `pub(crate)`) and `Svf` (needs `Svf::tick` + `svf_coeffs` exposed
> `pub(crate)` for the decimation LP). MIT/Apache; standard waveshaping, no ports.

**Ef roadmap:** Ef-1 Delay ✓ → stereo-routing ✓ → Ef-2 Chorus/Flanger ✓ → Ef-3
Reverb ✓ → **Ef-4 Drive (this)** → Ef-5 EQ (RBJ biquad).

---

## 1. Goals & non-goals

**Goals**
- **`Drive` waveshaper kernel.** A small `Copy` per-sample kernel (no heap, no
  buffer). A **shape** enum + params (drive / tone / mix). Per sample:
  pre-gain (drive) → **4× oversample** [linear-interp upsample → shape → 4-pole
  Butterworth decimation LP → downsample] → tone one-pole → output makeup →
  dry/wet mix.
- **Four shapes:** `Soft` = `8·pade_tanh(x/8)`-style soft clip (reuse `pade_tanh`);
  `Hard` = `clamp(x, −1, 1)`; `Fold` = triangle wavefolder (reflect at ±1);
  `Tube` = asymmetric `pade_tanh` (`pade_tanh(x+b) − pade_tanh(b)`, even harmonics).
- **4× anti-aliasing.** The nonlinearity generates harmonics that alias; oversample
  ×4, shape at 4·fs, and decimate with a **4-pole (cascaded `Svf`) Butterworth LP**
  at ≈ fs/2 before downsampling. Alias suppression is a QA gate.
- **Tone + makeup.** A post-shape one-pole tone (tilt/LP), and an output makeup
  gain so heavy drive doesn't run away in level.
- **Graph node** `Kind::Drive` — mono (`out_width` 1), the simplest per-sample arm
  (like `Svf`): `d.process(ins[0], dt, outs.port(0))`; no pooled buffer, no stereo.
- **QA-proven:** each shape distorts (adds harmonics), drive increases distortion,
  hard/fold clip harder than soft, tone darkens, 4× suppresses aliasing vs base
  rate, output bounded, mix balances.

**Non-goals (deferred)**
- **Proper polyphase (FIR halfband) oversampling** — v1 uses linear-interp
  upsampling + IIR decimation (adequate; polyphase is a later quality bump).
- **Pre/post EQ, bias/asymmetry control, more shapes (diode, bitcrush), stereo** —
  later. Drive is mono (width-1).
- **Auto-gain-matched A/B** — makeup is a fixed per-shape curve, not level-matched.

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/drive.rs`, new module)

```rust
#[derive(Clone, Copy)]
pub enum Shape { Soft, Hard, Fold, Tube }

/// Multi-shape waveshaper with 4× oversampling. Per-sample; no heap, no buffer.
#[derive(Clone, Copy)]
pub struct Drive {
    shape: Shape,
    drive: f32,   // param 0 → pre-gain
    tone: f32,    // param 1 → post one-pole cutoff
    mix: f32,     // param 2 → dry/wet
    up_prev: f32, // last input, for linear-interp upsampling
    dec_a: Svf,   // decimation LP stage 1 (Butterworth Lp)
    dec_b: Svf,   // decimation LP stage 2 (4-pole total)
    tone_z: f32,  // post tone one-pole state
}

impl Drive {
    pub fn new(shape: Shape) -> Drive;
    pub fn set_drive(&mut self, v: f32);  // param 0, clamp/​map to gain
    pub fn set_tone(&mut self, v: f32);   // param 1, [0,1]
    pub fn set_mix(&mut self, v: f32);    // param 2, [0,1]
    pub fn set_shape(&mut self, code: u8);// param 3, 0=Soft 1=Hard 2=Fold 3=Tube
    pub fn process(&mut self, input: In, dt: f32, out: &mut [f32]);
}
```

**Per sample `i`:** `x = input.at(i)`; `g = drive_gain(self.drive)` (e.g. `1 +
drive·MAX_DRIVE`); for `k in 0..4`: `up = lerp(up_prev, x, (k+1)/4)`; `sh =
shape(up·g)`; `y = dec_b.tick_lp(dec_a.tick_lp(sh))` (4-pole decimation LP at
≈fs/2 in the 4·fs domain)`; keep the last `y` as the decimated output (take every
4th). `up_prev = x`. Then tone one-pole: `tone_z += tc·(y − tone_z); y = tone_z`
(`tc` from `tone`); makeup `y *= makeup(shape, drive)`; `out[i] = x·(1−mix) +
y·mix`.

**Shapes:** `Soft`: `8·pade_tanh(z/8)` (bounded ±8, transparent at low level);
`Hard`: `z.clamp(−1,1)`; `Fold`: reflect `z` into `[−1,1]` by triangle folding
(`z − 4·round(z/4)`-style, then reflect); `Tube`: `pade_tanh(z + BIAS) −
pade_tanh(BIAS)` (asymmetric → even harmonics).

**Decimation LP:** two `Svf` LP stages (Butterworth `k = √2` per stage; cascade ≈
4-pole) with cutoff ≈ fs/2, coefficients from `svf_coeffs`/`Svf::tick` at the 4×
rate (`dt/4`). Exact cutoff/`k` pinned in the plan and validated by the alias gate.

`no_std`, pure `f32`, deterministic. Reuses `crate::filter::pade_tanh` and the
`Svf` primitive (exposing `Svf::tick` + `svf_coeffs` as `pub(crate)`).

---

## 3. Graph & Wren surface

**`deluge-audio-graph`:**
- `enum Kind`: add `Drive`. `enum State`: add `Drive(Drive)`.
- `Node::new`: `Kind::Drive => State::Drive(Drive::new(Shape::Soft))` (shape then
  set via `set_param(3)`). `out_width` = 1 (mono — falls through the default arm).
- Render arm (simplest, like `Svf`): `if let State::Drive(d) = … { d.process(ins[0],
  dt, outs.port(0)) }`. No pooled region, no `port_pair`.
- `set_param`: `0 => set_drive, 1 => set_tone, 2 => set_mix, 3 => set_shape`.

**`deluge-wren-core`:**
- `node_drive_impl(input, drive, tone, mix, shape_code)`: create `Kind::Drive`,
  `set_param` 3=shape / 0=drive / 1=tone / 2=mix. (No buffer — a plain `new_node`,
  not `new_pooled_node`.) Returns a mono node (`return_node`, width 1).
- **Setters:** `drive=` (reused → `set_param(0)`; the Moog/Ms20 overdrive selector,
  same "drive amount" concept), `tone=` (new → 1), **`wet=`** (new → 2 for mix —
  NOT `mix=`, which is `set_param(0)` and would collide with `drive=`). Shape is
  construction-only (per factory).
- Prelude: `foreign static drive_(input, drive, tone, mix, shape)` + `foreign tone=`
  + `foreign wet=`; a `class Drive` with named factories:
  `Drive.soft/hard/fold/tube(input, drive, tone, mix)` → `Node.drive_(…, code)`
  (codes 0/1/2/3). Registered in both binding tables.

---

## 4. QA acceptance & testing

**Kernel (`deluge-dsp-kernels`, reuse the Fi `spectrum`/THD harness where useful):**
- **Each shape distorts.** A sine in at moderate drive: output THD / harmonic
  energy > the (near-zero) input THD, for all four shapes.
- **Drive increases distortion.** Higher `drive` → more harmonic energy (monotonic).
- **Shape ordering.** `Hard`/`Fold` produce more high-harmonic energy than `Soft`
  at the same drive (harder edges); `Tube` adds even harmonics (2nd) that `Soft`
  (odd-symmetric) does not.
- **4× suppresses aliasing.** Drive a high sine (near fs/4) hard; the aliased
  (inharmonic) energy is lower with the 4× path than a base-rate reference —
  `worst_alias_db` below a threshold (reuse the Fi alias harness).
- **Tone darkens.** Lower `tone` → less HF energy in the output.
- **Boundedness (P0 gate).** Proptest: `drive/tone/mix ∈ [0,1]`, each shape, bounded
  input → finite and within a measured bound.
- **Mix balance.** `mix=0` → output ≈ dry; `mix=1` → fully shaped.

**Graph/Wren:**
- A `Kind::Drive` node renders bounded, non-silent, mono; each shape differs.
- `Drive.soft/hard/fold/tube(...)` emit a `Kind::Drive` `NewNode` + the SetParams
  (incl. the right shape code); `drive=`/`tone=`/`wet=` update params 0/1/2.

**Determinism:** no RNG/time; reproducible; proptests seeded.

---

## 5. Deferred / follow-ups

- **Polyphase (FIR halfband) oversampling** for cleaner anti-aliasing than the
  linear-interp + IIR v1.
- **Bias/asymmetry, pre/post EQ, diode/bitcrush shapes, stereo drive, level-matched
  makeup** — later.
- **Ef-5 EQ** (RBJ biquad, from scratch — NOT the GPL DelugeEq) completes the Ef
  suite's planned set.
