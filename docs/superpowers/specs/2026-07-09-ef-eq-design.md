# Ef-5: EQ (RBJ biquad) — design & spec

The final planned Ef sub-project: an **RBJ cookbook parametric EQ** — peaking
(bell) and low/high shelving filters, the gain-shaping filters the Fi `Svf`
(LP/HP/BP/notch) doesn't provide. A small per-sample kernel (mono, no buffer)
like `Svf`/`Drive`, built from a **Direct-Form-II-Transposed biquad** in the
CMSIS-DSP style (Apache-2.0-inspired structure — NOT spark's GPL `deluge/fx/eq.rs`).

> **Status:** design proposal. Depends on `fast_sin` (sin/cos), `libm`
> (sqrtf/powf), and the merged graph. MIT/Apache. The RBJ "Audio EQ Cookbook"
> coefficient formulas are public/textbook; the DF2T biquad structure follows
> CMSIS-DSP `arm_biquad_cascade_df2T_f32` (Apache-2.0). Do NOT read/port the
> GPL DelugeEq.

**Ef roadmap:** Ef-1 Delay ✓ → stereo-routing ✓ → Ef-2 Chorus/Flanger ✓ → Ef-3
Reverb ✓ → Ef-4 Drive ✓ → **Ef-5 EQ (this)** — completes the planned Ef set.

---

## 1. Goals & non-goals

**Goals**
- **DF2T biquad primitive.** Direct-Form-II-Transposed (CMSIS-DSP style): state
  `z1, z2`; per sample `y = b0·x + z1; z1 = b1·x − a1·y + z2; z2 = b2·x − a2·y`.
  Numerically friendly for `f32`, two state variables, the canonical
  SIMD/cascade-friendly form (a single mono biquad's recurrence is serial, so —
  consistent with the codebase's "serial-recurrence kernels stay scalar" policy —
  cross-voice / cascade SIMD is deferred to the voice layer).
- **RBJ coefficients** for three types: **peaking**, **low-shelf**, **high-shelf**
  (from `freq` Hz, `gain` dB, `Q`). Standard "Audio EQ Cookbook" formulas,
  normalized by `a0`.
- **`Eq` kernel.** Stores `freq/gain/q/type` + a `Biquad`; `process()` recomputes
  the coefficients **once per block** (control-rate — RBJ needs `sin`/`cos`
  [`fast_sin`, `cos(ω)=fast_sin(p+0.25)`], `sqrtf`, `powf`; per-block keeps it
  cheap), then ticks the biquad per sample.
- **Graph node** `Kind::Eq` — mono (`out_width` 1), the simplest per-sample arm
  (like `Svf`/`Drive`): `e.process(ins[0], dt, outs.port(0))`; no buffer.
- **QA-proven:** peaking boosts/cuts at `freq` (gain > / < 0 dB there); low/high
  shelf lift/drop the low/high band; `Q` sets the bell width; unity at 0 dB gain;
  bounded/stable; the coefficient-recompute is per block.

**Non-goals (deferred)**
- **RBJ LP/HP/BP/notch/allpass** — `Svf` already covers those; the EQ adds only the
  gain filters.
- **Multi-band EQ node, audio-rate freq/gain modulation (ports), cascade SIMD,
  analog-matched / linear-phase EQ** — later. `freq/gain/q` are control-rate params.
- **Stereo** — mono (width-1); wrap two for stereo if needed.

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/eq.rs`, new module)

```rust
/// Direct-Form-II-Transposed biquad (CMSIS-DSP `df2T` style). `a0`-normalized
/// coefficients (`a0 == 1`). State `z1, z2`.
#[derive(Clone, Copy)]
pub struct Biquad { b0: f32, b1: f32, b2: f32, a1: f32, a2: f32, z1: f32, z2: f32 }
impl Biquad {
    pub fn new() -> Biquad;                 // identity (b0=1, rest 0)
    pub fn set_coeffs(&mut self, b0: f32, b1: f32, b2: f32, a1: f32, a2: f32);
    #[inline] pub fn tick(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }
}

#[derive(Clone, Copy)]
pub enum EqType { Peak, LowShelf, HighShelf }

/// RBJ parametric EQ band (peaking / low-shelf / high-shelf). Per-sample, mono,
/// no heap/buffer. Coefficients recomputed per block from freq/gain/Q.
#[derive(Clone, Copy)]
pub struct Eq {
    ty: EqType,
    freq: f32, // Hz     (param 0)
    gain: f32, // dB     (param 1)
    q: f32,    //        (param 2)
    biquad: Biquad,
}
impl Eq {
    pub fn new(ty: EqType) -> Eq;           // freq 1000, gain 0, q 0.707
    pub fn set_freq(&mut self, v: f32);     // param 0, clamp ≥ ~10 Hz
    pub fn set_gain(&mut self, v: f32);     // param 1, dB (clamp ±24)
    pub fn set_q(&mut self, v: f32);        // param 2, clamp ≥ ~0.1
    pub fn set_type(&mut self, code: u8);   // param 3, 0=Peak 1=LowShelf 2=HighShelf
    pub fn process(&mut self, input: In, dt: f32, out: &mut [f32]);
}
```

**Coefficient recompute (per block, RBJ):** `p = (freq·dt).clamp(…, 0.49)`; `sinw
= fast_sin(p)`, `cosw = fast_sin(p + 0.25)`; `A = powf(10, gain/40)`; `alpha =
sinw / (2·Q)`. Then the standard RBJ formulas per `ty`:
- **Peak:** `b0 = 1+alpha·A; b1 = −2·cosw; b2 = 1−alpha·A; a0 = 1+alpha/A; a1 =
  −2·cosw; a2 = 1−alpha/A`.
- **Low-shelf:** `b0 = A·((A+1)−(A−1)·cosw + 2·√A·alpha); b1 = 2·A·((A−1)−(A+1)·cosw);
  b2 = A·((A+1)−(A−1)·cosw − 2·√A·alpha); a0 = (A+1)+(A−1)·cosw + 2·√A·alpha; a1 =
  −2·((A−1)+(A+1)·cosw); a2 = (A+1)+(A−1)·cosw − 2·√A·alpha`.
- **High-shelf:** `b0 = A·((A+1)+(A−1)·cosw + 2·√A·alpha); b1 = −2·A·((A−1)+(A+1)·cosw);
  b2 = A·((A+1)+(A−1)·cosw − 2·√A·alpha); a0 = (A+1)−(A−1)·cosw + 2·√A·alpha; a1 =
  2·((A−1)−(A+1)·cosw); a2 = (A+1)−(A−1)·cosw − 2·√A·alpha`.

Normalize `b0..a2` by `a0` and `set_coeffs(b0/a0, b1/a0, b2/a0, a1/a0, a2/a0)`.
`no_std`, pure `f32`, deterministic. `gain = 0 dB → A = 1 → identity` (unity).

---

## 3. Graph & Wren surface

**`deluge-audio-graph`:**
- `enum Kind`: add `Eq`. `enum State`: add `Eq(Eq)`.
- `Node::new`: `Kind::Eq => State::Eq(Eq::new(EqType::Peak))`. `out_width` = 1
  (mono — falls through the default arm).
- Render arm (simplest, like `Svf`/`Drive`): `if let State::Eq(e) = … {
  e.process(ins[0], dt, outs.port(0)) }`.
- `set_param`: `0 => set_freq, 1 => set_gain, 2 => set_q, 3 => set_type`.

**`deluge-wren-core`:**
- `node_eq_impl(input, freq, gain, q, type)`: create `Kind::Eq` (plain `new_node`,
  no buffer), `set_param` 3=type / 0=freq / 1=gain / 2=q, `return_node` (mono).
- **Setters:** **`hz=`** (new → param 0, freq — NOT `freq=`, which is the Osc's
  port-0 setter), **`gain=`** (new → 1, dB), **`q=`** (new → 2). Type is
  construction-only.
- Prelude: `foreign static eq_(input, freq, gain, q, type)` + `foreign hz=` +
  `foreign gain=` + `foreign q=`; a `class EQ` with named factories:
  `EQ.peak/lowShelf/highShelf(input, freq, gain, q)` → `Node.eq_(…, code 0/1/2)`.
  Registered in both binding tables.

---

## 4. QA acceptance & testing

**Kernel (`deluge-dsp-kernels`, reuse the Fi `spectrum`/`magnitude_db` harness):**
- **Peak boosts/cuts at freq.** Peak `+12 dB` at 1 kHz: the magnitude at 1 kHz is
  ≈ +12 dB (within tol); away from `freq` it's ≈ 0 dB. `−12 dB` cuts.
- **Shelves.** Low-shelf `+9 dB`: low band (e.g. 100 Hz) lifted, high band (e.g.
  8 kHz) ≈ 0 dB; high-shelf mirrors.
- **Q sets bell width.** Higher `Q` → a narrower peak (steeper skirts / narrower
  −3 dB bandwidth) than low `Q`.
- **Unity at 0 dB.** `gain = 0` → output ≈ input (all types, all freqs) within a
  tight tolerance.
- **Boundedness/stability (P0 gate).** Proptest: `freq ∈ [20, 18k]`, `gain ∈
  [−24, 24]`, `q ∈ [0.2, 8]`, each type, bounded input → finite and bounded (a
  stable biquad; `a`-poles inside the unit circle for these RBJ params).
- **Per-block coeffs.** Coefficients recompute once per `process` call, not per
  sample (a design check — a const-cutoff render is deterministic).

**Graph/Wren:**
- A `Kind::Eq` node renders bounded, non-silent, mono; a peak at gain ≠ 0 changes
  the spectrum vs gain = 0.
- `EQ.peak/lowShelf/highShelf(...)` emit a `Kind::Eq` `NewNode` + the SetParams
  (incl. type code); `hz=`/`gain=`/`q=` update params 0/1/2.

**Determinism:** no RNG/time; reproducible; proptests seeded.

---

## 5. Deferred / follow-ups

- **RBJ LP/HP/BP/notch/allpass** (SVF covers them), **multi-band EQ node**,
  **audio-rate freq/gain ports**, **cascade/cross-voice SIMD** of the DF2T biquad,
  **analog-matched / linear-phase** EQ — later.
- **Stereo EQ** — wrap two mono bands (or a future stereo node model).
- Completes the planned **Ef suite**; next are the **Mod** (modulation) and **Sy**
  (synth/voice) suites (greenfield).
