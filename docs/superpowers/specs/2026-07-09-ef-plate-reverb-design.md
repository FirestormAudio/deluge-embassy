# Ef-3c: Plate reverb (Dattorro) — design & spec

The final reverb sub-project (completes **Ef-3 Reverb** = Room ✓ / Hall ✓ /
Plate): a **Dattorro plate** reverb — input diffusion feeding a figure-8 tank of
modulated allpasses with a characteristic multi-tap output network. Reuses the
shared reverb foundation (`DelayLine`, the partitioned-pooled-buffer pattern,
the width-2 stereo node, the Room param surface).

> **Status:** design proposal. Depends on: `DelayLine` (`read_hermite`/`write`),
> `fast_sin`, `libm::floorf`, the pooled-region graph path, `OutView::port_pair`,
> width-aware `Out.patch`, `new_pooled_node`, and the **Room param surface**
> (`mix=`/`damp=`/`size=`/`spread=` — all merged). Implemented from the **public**
> Dattorro 1997 "Effect Design Part 1: Reverberator and Other Filters" structure;
> MIT/Apache, no GPL source consulted.

**Reverb roadmap:** Ef-3a Room ✓ → Ef-3b Hall ✓ → **Ef-3c Plate (this)** —
completes Ef-3.

---

## 1. Goals & non-goals

**Goals**
- **`Dattorro` plate engine.** The classic topology:
  `input → bandwidth one-pole LP → 4 series input-diffusion allpasses → figure-8
  tank`. The **tank** is two cross-fed halves; each half:
  *modulated allpass* (excursion LFO via `read_hermite`) → *delay* → *damping
  one-pole LP* → *× decay* → *allpass* → *delay*; each half's tail feeds the
  **other** half's input (the figure-8), scaled by `decay`.
- **Multi-tap output network.** L and R are each the sum of ~7 taps read at
  specific offsets *into* the tank delays/allpasses (the defining Dattorro
  feature — exact node/offset table pinned in the plan from the paper).
- **`Allpass` gains.** The Dattorro allpasses use per-stage gains (0.75/0.625
  input; 0.7/0.5 tank). The Dattorro allpass uses a **different recurrence** than
  the Ef-3a freeverb `Allpass` (`w = x − g·d; out = d + g·w`), so it's realized as
  **standalone integer-read helper fns** (`plate_ap`/`plate_ap_read`) — the
  existing `Allpass`/`Comb` are left untouched.
- **One partitioned pooled buffer.** All delay elements are `DelayLine`s over
  disjoint slices of one region (`PLATE_BUF_SAMPLES` = Σ slice lengths, each slice
  sized for its base length + modulation/tap headroom). Mono→stereo (width-2).
- **Reuses the Room param surface** — `set_param` 0=mix, 1=damp (→ damping +
  bandwidth), 2=size (→ tank `decay`), 3=width; Wren setters
  `mix=`/`damp=`/`size=`/`spread=` already exist. Ef-3c adds ONLY the `Plate.new`
  factory + `class Plate`.
- **QA-proven:** impulse → a smooth, dense, plate-like decaying tail (size
  lengthens it, damp darkens it); stereo L ≠ R (the tap network + cross-feed);
  mix balance; boundedness; graceful degradation.

**Non-goals (deferred)**
- **Pre-delay** (Dattorro's optional front delay) — a small fixed/omitted delay
  in v1; add as a param later.
- **Sample-rate-scaled tunings** — fixed Dattorro-nominal sample counts (character
  is SR-nominal, consistent with Room/Hall).
- **Freeze/infinite, tempo-sync, separate bandwidth/excursion params** — later
  (folded into `damp`/fixed for now).

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/reverb.rs`, alongside `Freeverb`/`Fdn8`)

```rust
impl Allpass {
    /// Schroeder allpass with an explicit feedback gain (Dattorro uses several).
    // Dattorro allpass = standalone helper fns (not the freeverb `Allpass`):
    //   plate_ap(slice, cursor, x, gain) / plate_ap_read(slice, cursor, x, gain, read_back)
    //   → d = slice[read]; w = x - gain*d; slice[cursor] = w; advance; return d + gain*w
}

/// Dattorro plate reverb: input diffusion + a figure-8 modulated-allpass tank +
/// a multi-tap output network, over one partitioned buffer. Mono in → stereo out.
#[derive(Clone, Copy)]
pub struct Dattorro {
    // input diffusers, tank allpasses (incl. the 2 modulated), tank delays —
    // all `DelayLine` cursors; a bandwidth-LP state, 2 damping-LP states,
    // an excursion LFO phase, and the figure-8 feedback carry.
    ...
    size: f32, damp: f32, width: f32, mix: f32, // params
}
impl Dattorro {
    pub fn new() -> Dattorro;                // size 0.5, damp 0.5, width 1, mix 0.5
    pub fn set_mix(&mut self, v: f32);       // param 0
    pub fn set_damp(&mut self, v: f32);      // param 1
    pub fn set_size(&mut self, v: f32);      // param 2 (→ decay)
    pub fn set_width(&mut self, v: f32);     // param 3
    pub fn process(&mut self, input: In, dt: f32, buf: &mut [f32],
                   out_l: &mut [f32], out_r: &mut [f32]);
}

pub const PLATE_BUF_SAMPLES: usize; // Σ slice lengths; exact value pinned in the plan
```

**Constants (pinned in the plan, from the paper, validated by a layout-sum test):**
input diffusers `142/107/379/277` (gains `0.75/0.75/0.625/0.625`); tank
decay-diffusion-1 modulated allpasses `672/908` (gain `0.7`, excursion ~±8);
tank delays `4453/4217` and `3720/3163`; decay-diffusion-2 allpasses `1800/2656`
(gain `0.5`); the L/R output tap offsets; the bandwidth LP coefficient. Slices are
sized `base + margin` so modulation and every tap offset stay in range.

**Per sample:** bandwidth-LP the input; run the 4 input diffusers in series;
feed the diffused signal + the figure-8 carry into the tank; per half, run
modulated-allpass → delay → damping-LP → `× decay` → allpass → delay; cross the
two halves' tails (`× decay`) as the next-sample carry; read the L/R output taps
at their fixed offsets from the tank elements; `wet1 = mix·(width·0.5+0.5); wet2 =
mix·((1−width)·0.5); dry = 1−mix`; `out_l = x·dry + tapL·wet1 + tapR·wet2; out_r =
x·dry + tapR·wet1 + tapL·wet2`.

**Mappings:** `decay = size·0.4 + 0.5` (∈ [0.5, 0.9] — plates decay faster than
halls); `damping = damp·0.4` (tank LP) with `bandwidth` derived from `damp` too;
excursion LFO ~1 Hz (phase wrapped via `floorf`).

**Buffer safety:** `process` dry-passes-through if `buf.len() < PLATE_BUF_SAMPLES`.
`no_std`, pure `f32`, deterministic; kernel owns no delay storage.

**Stability:** the input/tank allpasses are unity-gain (lossless), the damping LP
is passive (≤1), and every loop path is scaled by `decay < 1` ⇒ bounded (the
boundedness proptest is the gate).

---

## 3. Graph & Wren surface

**`deluge-audio-graph`** (mirror `Kind::Hall`):
- `enum Kind`: add `Plate`. `enum State`: add `Plate(Dattorro)`.
- `Node::new`: `Kind::Plate => State::Plate(Dattorro::new())`. `out_width`: add
  `Kind::Plate` to the width-2 arm.
- Render arm (like `Kind::Hall`): `port_pair`; bound region `≥ PLATE_BUF_SAMPLES`
  → `d.process(ins[0], dt, buf, out_l, out_r)`; else dry passthrough both ports.
- `set_param`: `0 => set_mix, 1 => set_damp, 2 => set_size, 3 => set_width`.

**`deluge-wren-core`** (minimal — reuses the Room setters):
- `node_plate_impl`: `alloc_buffer(PLATE_BUF_SAMPLES)`, `new_pooled_node(id,
  Kind::Plate, handle, input)`, set params 2=size / 1=damp / 0=mix, `return_node_w(vm,
  id, 2)`. Unbound → dry passthrough.
- Registered in both binding tables + `class Plate { static new(input, size, damp,
  mix) { Node.plate_(input, size, damp, mix) } }`. **No new setters.**

---

## 4. QA acceptance & testing

**Kernel (`deluge-dsp-kernels`):**
- **Layout sum.** `PLATE_BUF_SAMPLES == Σ` slice lengths (pinned test).
- **Layout sum.** `PLATE_BUF_SAMPLES == Σ` element lengths (pinned test).
- **Impulse → dense decaying tail.** Non-silent well after the impulse; late-window
  energy < early-window; bounded.
- **Size lengthens / damp darkens** the tail (monotonic).
- **Stereo decorrelation.** `out_l != out_r`.
- **Mix balance.** `mix=0` → output ≈ dry input.
- **Boundedness (P0 gate).** Proptest: `size/damp/width/mix ∈ [0,1]`, bounded input
  → finite and within a measured bound; `PLATE_BUF_SAMPLES`-long buffer.
- **Graceful:** too-short / absent buffer → dry passthrough, never panic.

**Graph/Wren:**
- `Kind::Plate` node renders bounded, non-silent, stereo (l ≠ r); no buffer → dry
  both ports; never panics.
- `Plate.new(...)` allocates+binds + emits NewNode + BindTable + SetParams, returns
  width-2; `Out.patch(Plate.new(...))` emits the two stereo side-writes; the reused
  `size=`/`damp=`/`spread=`/`mix=` update the right params.

**Determinism:** no RNG/time; reproducible; proptests seeded.

---

## 5. Deferred / follow-ups

- **Pre-delay, separate bandwidth/excursion/decay-diffusion params, freeze,
  tempo-sync, SR-scaled tunings** — later plate refinements.
- **Rename** `TableSrc`/`bind_table` → `PooledBuf`/`bind_buffer` (six effects now
  reuse it) — mechanical cleanup.
- **A unified `Reverb.new(type, …)`** selector over Room/Hall/Plate, if a single
  entry point is later wanted (the three named classes stay).
