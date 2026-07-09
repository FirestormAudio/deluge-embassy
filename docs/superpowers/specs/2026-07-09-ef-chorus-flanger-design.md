# Ef-2: Chorus / Flanger — design & spec

The second Ef-suite sub-project: a **`ModDelay<VOICES>`** modulated-delay kernel —
`VOICES` phase-spread, panned voices tapping a shared short delay buffer with an
internal LFO — exposed as two **mono→stereo (width-2)** graph nodes:
**`Kind::Chorus`** (`ModDelay<3>`) and **`Kind::Flanger`** (`ModDelay<1>` +
feedback). Built on the merged `DelayLine` (Ef-1), the pooled-buffer host seam
(`Host::alloc_buffer`), and the stereo routing + constant-power `pan_gains`
(stereo-routing sub-project).

> **Status:** design proposal. Depends on: `DelayLine`
> (`deluge-dsp-kernels/src/delay.rs`, `read_hermite`), `math::pan_gains`
> (constant-power), `fast_sin` (normalized-phase LFO), the pooled-region graph
> path (`TableSrc::Pooled` + `process_resolved`'s `&mut` region), `Host::alloc_buffer`,
> width-2 nodes + width-aware `Out.patch` (all merged). MIT/Apache — nothing ported.

**Ef roadmap:** Ef-1 Delay ✓ → **Ef-2 Chorus/Flanger** (this) → Ef-3 Reverb →
Ef-4 Drive → Ef-5 EQ.

---

## 1. Goals & non-goals

**Goals**
- **`ModDelay<VOICES>` kernel.** A tiny `Copy` struct: a `DelayLine` write cursor,
  one internal LFO phase accumulator, a `base` delay (seconds), and control
  params (rate, depth, feedback, mix). Borrows **one** short ring buffer per
  block (all voices are taps on the same written history). Produces a **stereo**
  (L/R) pair.
- **Two graph nodes** (like `Moog<4>`/`Moog<2>`): `Kind::Chorus` = `ModDelay<3>`
  (base ~20 ms, feedback 0), `Kind::Flanger` = `ModDelay<1>` (base ~2 ms, with
  feedback). `out_width` = 2.
- **Internal LFO, phase-spread voices, panned across the field.** Voice `v`'s LFO
  phase is offset by `v / VOICES` of a cycle; voice `v` is panned to
  `pos_v ∈ [-1, +1]` (evenly spread) via the constant-power `pan_gains` law.
- **Wren:** `Chorus.new(input, rate, depth, mix)` and
  `Flanger.new(input, rate, depth, feedback, mix)` → width-2 stereo nodes
  (route to L/R via the merged width-aware `Out.patch`/`Bus.write`).
- **Graceful degradation** (no bound buffer / pool exhausted): **dry passthrough**
  to both channels, never panic (mirrors `Kind::Delay`).
- **QA-proven:** modulated pitch/comb motion, stereo width (L ≠ R for chorus),
  flanger feedback resonance, mix balance, boundedness, graceful degradation.

**Non-goals (deferred)**
- **Quadrature stereo flanger** (L/R 90°-offset LFO) — `ModDelay<1>` pans its one
  voice to center, so the flanger is mono-in-both-channels for now; a true stereo
  flanger is a follow-up.
- **Tempo-sync, mid-side, stereo width control, per-voice detune spread** — later.
- **User-facing base-delay control** — base is a per-effect internal default.
- **Stereo (2-in) input** — input is mono (port 0); stereo→stereo chaining is out
  of scope (as elsewhere in the suite).

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/delay.rs`, alongside `DelayLine`/`Delay`)

```rust
/// Modulated multi-voice delay: chorus (VOICES>1, no feedback) / flanger
/// (VOICES=1 + feedback). `VOICES` phase-spread taps on a shared borrowed ring
/// buffer, an internal LFO, panned to a stereo pair. `no_std`, no heap.
#[derive(Clone, Copy)]
pub struct ModDelay<const VOICES: usize> {
    line: DelayLine,
    lfo_phase: f32,   // normalized [0,1)
    base: f32,        // base delay, seconds (per-effect default)
    rate: f32,        // LFO Hz          (param 1)
    depth: f32,       // [0,1) → fraction-of-base sweep (param 2)
    mix: f32,         // dry/wet [0,1]   (param 0)
    feedback: f32,    // [0,0.9] flanger regen (param 3)
}

impl<const VOICES: usize> ModDelay<VOICES> {
    pub fn new(base_s: f32) -> Self;   // sensible rate/depth/mix defaults, feedback 0
    pub fn set_mix(&mut self, v: f32);       // param 0, clamp [0,1]
    pub fn set_rate(&mut self, v: f32);      // param 1, clamp ≥0 (Hz)
    pub fn set_depth(&mut self, v: f32);     // param 2, clamp [0,1]
    pub fn set_feedback(&mut self, v: f32);  // param 3, clamp [0,0.9]
    /// One block. `input` = mono port 0; `buf` = pooled ring; writes the
    /// stereo pair `out_l`/`out_r`.
    pub fn process(&mut self, input: In, dt: f32, buf: &mut [f32],
                   out_l: &mut [f32], out_r: &mut [f32]);
}
```

**Constants:** Chorus base 20 ms, flanger base 2 ms (passed to `new`). Depth is
**multiplicative** (a fraction of `base`), so the modulated delay is always
positive for any base — the flanger's short base can't be driven negative, and
`depth → 1` sweeps the flanger toward ~0 delay (the classic through-zero-ish
flange).

**Per sample `i`:**
1. `x = input.at(i)`; advance LFO: `lfo_phase = (lfo_phase + rate·dt).fract()`.
2. `wet_l = 0; wet_r = 0;` and for `v in 0..VOICES`:
   - `ph_v = (lfo_phase + v as f32 / VOICES as f32).fract()`
   - `d = base·(1.0 + depth·fast_sin(ph_v)) / dt` samples — a **multiplicative**
     bipolar sweep (`fast_sin ∈ [-1,1]`, `depth ∈ [0,1)`), so
     `d ∈ base·(1−depth) .. base·(1+depth)`, always positive. `read_hermite`
     still clamps to its valid sample range as a floor.
   - `tap = line.read_hermite(buf, d)`
   - `pos_v = if VOICES == 1 { 0.0 } else { -1.0 + 2.0·v/(VOICES-1) }`
     (VOICES=3 → −1, 0, +1); `(gl, gr) = pan_gains(pos_v)`
   - `wet_l += tap·gl; wet_r += tap·gr`
3. Normalize: also accumulate a **pre-pan** `wet_mono += tap` in the voice loop;
   then `wet_l /= VOICES; wet_r /= VOICES; wet_mono /= VOICES` (bounded, keeps wet
   ≈ input level regardless of voice count).
4. **Feedback (flanger):** `fb = feedback.clamp(0, 0.9) · wet_mono`; `line.write(buf,
   x + fb)`. Feedback regenerates the **raw (pre-pan) delayed signal** so the
   `feedback` knob is the true loop gain (a proper resonant flanger, up to ~0.9).
   For `VOICES == 1`, `wet_mono == tap`. Chorus `feedback = 0` → a plain modulated
   tap. Loop gain `≤ 0.9 < 1` ⇒ BIBO stable; sustained-input steady state reaches
   `≈ 1/(1−0.9) = 10×`, so the boundedness gate allows `≤ 16`.
5. `out_l[i] = x·(1 − mix) + wet_l·mix;  out_r[i] = x·(1 − mix) + wet_r·mix;`

`no_std`, pure `f32`, deterministic (LFO is a phase accumulator; no RNG). The
kernel owns no ring storage — `buf` is borrowed. Feedback clamp `< 1` ⇒ BIBO
stable (same argument as `Delay`).

---

## 3. Graph & Wren surface

**`deluge-audio-graph`:**
- `enum Kind`: add `Chorus`, `Flanger`. `enum State`: add `Chorus(ModDelay<3>)`,
  `Flanger(ModDelay<1>)`.
- `Node::new`: `Kind::Chorus => State::Chorus(ModDelay::<3>::new(0.020))`,
  `Kind::Flanger => State::Flanger(ModDelay::<1>::new(0.002))`.
- `out_width`: `Kind::Chorus | Kind::Flanger => 2` (stereo).
- Render arm: resolve the pooled ring (the merged `&mut` `pool_region`); if
  present, `md.process(ins[0], dt, region, outs.port(0), outs.port(1))`; else
  **dry passthrough** (`out_l = out_r = ins[0]`). Port 0 = input (only port used).
- `set_param`: `0 => set_mix, 1 => set_rate, 2 => set_depth, 3 => set_feedback`
  (both kinds).
- Buffer: bound via the existing `TableSrc::Pooled` / `Cmd::BindTable`, allocated
  by the host `alloc_buffer` (~50 ms ≈ 2205 samples, covers base+depth+headroom;
  chosen as a `CHORUS_BUF_SAMPLES` const in the Wren layer).

**`deluge-wren-core`:**
- Factories `node_chorus_impl` / `node_flanger_impl`: allocate the ring
  (`audio::alloc_buffer(CHORUS_BUF_SAMPLES)`), create the node
  (`Kind::Chorus`/`Flanger`) bound to it, **emit `SetParam` for the constructor
  args** (rate/depth/mix, and feedback for flanger), and return a **width-2**
  node (`return_node_w(vm, id, 2)`) so `Out.patch` routes it stereo. Unbound
  (no pool) → node created, no bind → dry passthrough.
- Setters (post-construction tweaking): `rate=` → `set_param(1)`, `depth=` →
  `set_param(2)`; `mix=` (reuses the Ef-1 selector → `set_param(0)`); flanger
  feedback via **`regen=`** → `set_param(3)`. (NOT `feedback=`: that selector is
  already the Osc's `set_param(0)`; a Chorus/Flanger's param 0 is `mix`. `regen`
  — flanger regeneration — is the distinct, apt name, same reasoning as Ef-1's
  `damp=` vs the Resonator's `damping=`.)
- Prelude: `foreign static chorus_(...)` / `flanger_(...)`, `foreign rate=`,
  `foreign depth=`, `foreign regen=`; sugar `class Chorus { static new(input,
  rate, depth, mix) {...} }` and `class Flanger { static new(input, rate, depth,
  feedback, mix) {...} }`. Registered in both binding tables.

---

## 4. QA acceptance & testing

**Kernel (`deluge-dsp-kernels`, `process` takes `&mut [f32]` bufs):**
- **Chorus modulates + is stereo.** A steady input through `ModDelay<3>` with
  `depth>0`: the two output channels differ (`out_l != out_r` over a block —
  voices panned) and the wet content moves over time (the delayed copy's phase
  sweeps — measure that a late window differs from an early one). `mix=1`.
- **Flanger feedback resonates.** `ModDelay<1>`, short base, `feedback` high vs 0:
  higher feedback → a stronger resonant comb notch/peak (more spectral contrast),
  bounded.
- **Mix balance.** `mix=0` → both channels ≈ dry input; `mix=1` → wet only.
- **Boundedness (P0 gate).** Proptest: `rate ∈ [0,8] Hz`, `depth ∈ [0,1]`,
  `mix/feedback ∈ [0,1]` (feedback clamped 0.9), bounded input → finite and
  `≤ 16` (raw-tap feedback loop gain ≤0.9 → steady state ~10×); test BOTH
  `ModDelay<3>` (chorus) and `ModDelay<1>` (flanger); buffer covers `base·(1+depth)`.
- **Depth 0 is a static (unmodulated) short delay** — no NaN, no motion.

**Graph/Wren:**
- A `Kind::Chorus`/`Flanger` node with a bound buffer renders bounded, non-silent,
  stereo (l/r finite); no buffer → dry passthrough (`out_l == out_r == input`),
  never panics.
- `Chorus.new`/`Flanger.new` allocate+bind a buffer and emit the right `Cmd`s
  (NewNode + BindTable + the SetParams), return a width-2 node; `Out.patch(Chorus.new(...))`
  emits the two stereo side-writes (via the merged width-aware routing).
- `rate=`/`depth=`/`mix=`/`regen=` update the right params; pool-exhaustion → dry.

**Determinism:** no RNG; LFO is a deterministic phase accumulator; proptests seeded.

---

## 5. Deferred / follow-ups

- **Stereo flanger** (quadrature L/R LFO) — richer than the current center-panned
  single voice.
- **Ef-3 Reverb** — the `DelayLine::allpass` diffuser network (the allpass stub
  deferred from Ef-1 lands here) + pooled delay lines.
- **Tempo-sync, stereo-width/mid-side, per-voice detune, LFO waveform choice**
  (sine only for now).
- **Rename** `TableSrc`/`bind_table` → generic `PooledBuf`/`bind_buffer` (now
  three effects reuse it) — mechanical cleanup.
