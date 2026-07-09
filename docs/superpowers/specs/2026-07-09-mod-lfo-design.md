# Mod-1: LFO — design & spec

The first sub-project of the **Mod (modulation) suite**: dedicated **LFO** nodes —
raw (non-band-limited) low-frequency shapes with retrigger and a modulatable
rate — plus a **`.to(min, max)`** scaling helper. The graph already routes any
node to any port (arithmetic modulation), so Mod adds modulation *sources* and
ergonomic *scaling*, not routing.

> **Status:** design proposal. Depends on `fast_sin`, `crate::floorf`, the graph
> gate/trigger seam (`Node::gate/trigger` → `Cmd::Gate/Trigger`, currently
> dispatched to `State::Ar`), and the arithmetic-node prelude sugar. MIT/Apache.

**Mod roadmap:** **Mod-1 LFO (this)** → Mod-2 Sample&Hold / Random → Mod-3
modulation scaling / macros.

---

## 1. Goals & non-goals

**Goals**
- **`Lfo` kernel.** A tiny per-sample kernel (mono, no buffer) — a phase
  accumulator driving a **raw, non-band-limited** shape. (Deliberately unlike
  `Osc`, whose band-limiting is for *audio* — an LFO wants the clean geometric
  shape.) Bipolar **[−1, +1]** output.
- **Six shapes:** `Sine`, `Tri` (triangle), `Saw` (ramp), `Square` (50 % pulse),
  `SampleHold` (a new random level each cycle), `Random` (smooth random —
  interpolated between per-cycle random levels).
- **Modulatable rate.** Rate (Hz) is **port 0** (like `Osc` freq) → an LFO can
  modulate another LFO's rate.
- **Retrigger.** `trigger()`/`gate(on)` reset the phase to `phase_offset` (and
  re-roll the random level) — so LFOs sync to notes (reuses the `Ar` gate/trigger
  seam, extended to `State::Lfo`).
- **`.to(min, max)` sugar.** A Wren method on `Node`/`Port`: bipolar → the range
  `[min, max]` (`this·(hi−lo)/2 + (hi+lo)/2`), so `filter.cutoff = lfo.to(200,
  2000)` reads naturally. Unipolar is just `lfo.to(0, x)`.
- **QA-proven:** each shape's waveform (sine smooth, saw ramp with a per-cycle
  jump, square ±1, triangle, S&H held-then-stepped, random smooth); rate sets the
  period; retrigger resets phase; output bounded [−1,1]; `.to()` maps the range.

**Non-goals (deferred)**
- **Tempo/clock sync, one-shot (envelope-style) LFOs, per-node random seeding**
  (all `Random`/`SampleHold` LFOs share a fixed seed → correlated; a `seed`
  param is a later add), **fade-in/delay, unipolar as a kernel mode** — later.
- **Sample & Hold of an external input, stepped sequencer** — Mod-2.
- **Band-limited LFO shapes** — LFOs are sub-audio; aliasing is inaudible.

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/lfo.rs`, new module)

```rust
#[derive(Clone, Copy)]
pub enum LfoShape { Sine, Tri, Saw, Square, SampleHold, Random }

/// Low-frequency modulation source. Phase accumulator + raw shape; bipolar
/// [−1,1]. Per-sample, mono, no buffer. `no_std`, deterministic.
#[derive(Clone, Copy)]
pub struct Lfo {
    shape: LfoShape,
    phase: f32,        // [0,1)
    phase_offset: f32, // param 1 (retrig/start phase)
    rng: u32,          // xorshift32 state (random shapes)
    sh_cur: f32,       // current S&H / random-target level
    sh_prev: f32,      // previous level (Random interpolation)
}
impl Lfo {
    pub fn new() -> Lfo;                    // Sine, phase 0, seeded rng
    pub fn set_shape(&mut self, code: u8);  // param 0 → LfoShape
    pub fn set_phase(&mut self, v: f32);    // param 1 → phase_offset [0,1)
    pub fn retrigger(&mut self);            // reset phase to offset, re-roll rng
    /// `rate` = Hz (port 0). Writes the bipolar modulation signal.
    pub fn process(&mut self, rate: In, dt: f32, out: &mut [f32]);
}
```

**Per sample:** advance `phase = frac(phase + rate.at(i)·dt)` (wrap via
`floorf`); on a **cycle wrap** (phase decreased), `sh_prev = sh_cur; sh_cur =
next_rand()` (bipolar). Output by shape at `p = frac(phase + phase_offset)`:
- `Sine`: `fast_sin(p)`.
- `Tri`: `if p < 0.5 { p·4 − 1 } else { 3 − p·4 }` (−1 → +1 → −1).
- `Saw`: `2·p − 1` (rising ramp, raw jump at wrap).
- `Square`: `if p < 0.5 { 1 } else { −1 }`.
- `SampleHold`: `sh_cur` (held constant across the cycle).
- `Random`: `sh_prev + (sh_cur − sh_prev)·phase` (smooth interpolation).

`next_rand`: xorshift32 → `(state as f32 / u32::MAX)·2 − 1`. Seeded from a fixed
constant in `new()`. `retrigger()` sets `phase = phase_offset` and re-rolls
`sh_cur`. `no_std`, pure `f32`, deterministic.

---

## 3. Graph & Wren surface

**`deluge-audio-graph`:**
- `enum Kind`: add `Lfo`. `enum State`: add `Lfo(Lfo)`.
- `Node::new`: `Kind::Lfo => State::Lfo(Lfo::new())`. `out_width` = 1 (mono).
- Render arm (simplest, like `Osc`/`Svf`): `if let State::Lfo(l) = … {
  l.process(ins[0], dt, outs.port(0)) }` (port 0 = rate).
- `set_param`: `0 => set_shape, 1 => set_phase`.
- **`Node::gate/trigger`:** extend the existing `State::Ar` arms to also reset a
  `State::Lfo` (`l.retrigger()` on `trigger()`, and on `gate(true)`).

**`deluge-wren-core`:**
- `node_lfo_impl(rate, shape)`: create `Kind::Lfo` (plain `new_node`, `[rate,
  0, 0]`), `set_param(0, shape)`, `return_node` (mono).
- Named factories in a `class LFO`: `LFO.sine/tri/saw/square/sampleHold/random(rate)`
  → `Node.lfo_(rate, code 0…5)`.
- **Setters:** rate reuses **`freq=`** (→ `set_input(port 0)` — the LFO's rate IS
  port 0, so `lfo.freq = 5` works); **`phase=`** (new → `set_param(1)`). Shape is
  construction-only. `trigger()`/`gate(on)` already exist on `Node`.
- **`.to(min, max)`** method added to the `Node` AND `Port` prelude classes:
  `to(lo, hi) { this * ((hi - lo) / 2) + ((hi + lo) / 2) }` (uses the `*`/`+`
  binop sugar → an add(mul(this, halfrange), center) node).
- Registered in both binding tables.

---

## 4. QA acceptance & testing

**Kernel (`deluge-dsp-kernels`):**
- **Shapes.** Over one period at a known rate: `Saw` rises ~linearly −1→+1 then
  jumps; `Square` is ±1 with a mid-period flip; `Tri` is piecewise-linear peaking
  at mid; `Sine` matches `fast_sin`; `SampleHold` is piecewise-constant per cycle
  (holds, then steps); `Random` is continuous (no big jumps) but non-periodic.
- **Rate sets the period.** A 2 Hz LFO completes 2 cycles/sec (count zero-crossings
  or wraps).
- **Bipolar bounded.** All shapes ∈ [−1, 1] (± tiny epsilon), finite.
- **Retrigger resets phase.** After `retrigger()`, the next sample is the shape at
  `phase_offset` (e.g. Saw → `2·offset − 1`).
- **Modulatable rate.** An audio-rate `rate` block changes the instantaneous
  period (a rate ramp speeds the LFO up).
- **Boundedness proptest.** rate ∈ [0, 50] Hz, each shape, → finite, ∈ [−1,1].

**Graph/Wren:**
- A `Kind::Lfo` node renders bounded [−1,1], non-constant (for a non-zero rate);
  `trigger()` on it resets its phase (first post-trigger sample = shape at offset).
- `LFO.saw(2)` etc. emit a `Kind::Lfo` `NewNode` + `SetParam(0, code)`; `phase=`
  → `SetParam(1)`; `lfo.freq = r` → `SetInput(port 0)`.
- `lfo.to(200, 2000)` builds a `mul`+`add` graph mapping [−1,1] → [200, 2000]
  (verify via the emitted `Cmd`s or a render: a bipolar LFO through `.to` spans
  the range).

**Determinism:** xorshift32 seeded from a constant; reproducible; proptests seeded.

---

## 5. Deferred / follow-ups

- **Tempo/clock sync, one-shot LFOs, fade-in/delay, per-node `seed` param**
  (decorrelate multiple random LFOs), **unipolar kernel mode** — later Mod-1
  refinements.
- **Mod-2 Sample&Hold (of an external input) + stepped random/sequencer**, **Mod-3
  modulation scaling / attenuators / macro (mod-matrix) sugar** — the rest of Mod.
- The `.to(min,max)` helper is the seed of Mod-3's scaling layer.
