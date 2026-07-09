# Mod-2: Sample&Hold / Slew / Steps — design & spec

The second Mod sub-project: three **modulation-utility** nodes —
**`SampleHold`** (latch a signal on a clock edge), **`Slew`** (one-pole
glide/lag), and **`Steps`** (a fixed-array step sequencer). All tiny per-sample
mono kernels; the sequencer stores its steps **inline** (`[f32; 16]`), so no pool
is needed. Together with Mod-1's LFOs they give clocked/random/smoothed
modulation on top of the graph's existing arithmetic routing.

> **Status:** design proposal. Depends only on the merged graph (`In`, mono node
> pattern, `set_param`, Wren list reading à la `Wavetable.from`). MIT/Apache.

**Mod roadmap:** Mod-1 LFO ✓ → **Mod-2 S&H / Slew / Steps (this)** → Mod-3
modulation scaling / attenuators / macros.

---

## 1. Goals & non-goals

**Goals**
- **`SampleHold`** — ports 0 = input, 1 = clock. On a **rising clock edge**
  (`prev ≤ 0 < cur`) latch `held = input`; output `held`. `SampleHold.new(input,
  clock)` (e.g. `SampleHold.new(Noise.pink(), clock)` = stepped random).
- **`Slew`** — ports 0 = input, 1 = time (s, modulatable). A one-pole lag:
  `z += (input − z)·c`, `c = (dt / max(time, dt)).min(1)`. Glide/portamento;
  smooths stepped modulation. `Slew.new(input, time)`.
- **`Steps`** — port 0 = clock; a step sequencer over an inline `[f32; 16]` value
  array with a length and an index. On each **rising clock edge** advance the
  index (wrapping over `len`) and output `values[idx]`; the **first** edge plays
  step 0. `Steps.new([v0, v1, …], clock)` (values set via `set_param`; > 16 steps
  truncate, logged).
- **Shared rising-edge clock detector** (`prev ≤ 0 < cur`) — a signal (a square
  LFO, a metro, any bipolar/unipolar-from-0 source) clocks `SampleHold`/`Steps`.
- **QA-proven:** S&H latches on the edge and holds between edges; Slew converges
  toward a step with the expected time constant (and passes DC unchanged);
  Steps walks its values one per clock, wrapping, first-edge = step 0; all
  bounded.

**Non-goals (deferred)**
- **Trigger/gate reset of `Steps`** (reset to step 0 on a note) — a later add
  (would extend the `Node::gate/trigger` seam again).
- **> 16 steps, per-step gates/ratchets, pooled value arrays, separate rise/fall
  slew, linear (constant-rate) slew** — later.
- **Internal clock** (S&H/Steps are externally clocked; use an LFO/metro) — later.

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/modutil.rs`, new module)

```rust
/// Rising-edge detector: true when the clock crosses 0 upward (`prev ≤ 0 < cur`).
#[inline]
fn rising(prev: f32, cur: f32) -> bool { prev <= 0.0 && cur > 0.0 }

/// Sample & hold: latch `input` on each rising clock edge. Ports 0=input, 1=clock.
#[derive(Clone, Copy)]
pub struct SampleHold { held: f32, prev_clock: f32 }
impl SampleHold {
    pub fn new() -> SampleHold;
    pub fn process(&mut self, input: In, clock: In, out: &mut [f32]);
}

/// One-pole slew/lag (glide). Ports 0=input, 1=time (seconds).
#[derive(Clone, Copy)]
pub struct Slew { z: f32 }
impl Slew {
    pub fn new() -> Slew;
    pub fn process(&mut self, input: In, time: In, dt: f32, out: &mut [f32]);
}

/// Step sequencer over an inline `[f32; MAX_STEPS]` array, clocked by port 0.
pub const MAX_STEPS: usize = 16;
#[derive(Clone, Copy)]
pub struct Steps { values: [f32; MAX_STEPS], len: usize, idx: usize, prev_clock: f32, first: bool }
impl Steps {
    pub fn new() -> Steps;              // len 1, idx 0, first=true, values 0
    pub fn set_len(&mut self, n: u8);   // param 0, clamp 1..=MAX_STEPS
    pub fn set_value(&mut self, i: usize, v: f32); // param k → values[k-1]
    pub fn process(&mut self, clock: In, out: &mut [f32]);
}
```

**Per sample:**
- `SampleHold`: `let c = clock.at(i); if rising(self.prev_clock, c) { self.held =
  input.at(i); } out[i] = self.held; self.prev_clock = c;`
- `Slew`: `let c = (dt / time.at(i).max(dt)).min(1.0); self.z += (input.at(i) −
  self.z)·c; out[i] = self.z;`
- `Steps`: `let c = clock.at(i); if rising(self.prev_clock, c) { if self.first {
  self.first = false; } else { self.idx = (self.idx + 1) % self.len.max(1); } }
  out[i] = self.values[self.idx.min(MAX_STEPS−1)]; self.prev_clock = c;`

`no_std`, pure `f32`, deterministic, no heap, no buffer.

---

## 3. Graph & Wren surface

**`deluge-audio-graph`:**
- `enum Kind`: add `SampleHold`, `Slew`, `Steps`. `enum State`: add
  `SampleHold(SampleHold)`, `Slew(Slew)`, `Steps(Steps)`.
- `Node::new`: construct each kernel. `out_width` = 1 (all mono).
- Render arms: `SampleHold` → `sh.process(ins[0], ins[1], outs.port(0))`;
  `Slew` → `s.process(ins[0], ins[1], dt, outs.port(0))`; `Steps` →
  `st.process(ins[0], outs.port(0))` (clock on port 0).
- `set_param`: `Steps` — `0 => set_len(value as u8)`, `k (1..=16) => set_value(k,
  value)` (`k` is the `param` index; `set_value(k, v)` writes `values[k−1]`).
  `SampleHold`/`Slew` have no params (all-ports).

**`deluge-wren-core`:**
- `node_sh_impl(input, clock)` → `Kind::SampleHold` (ports 0/1), mono.
- `node_slew_impl(input, time)` → `Kind::Slew` (ports 0/1), mono.
- `node_steps_impl(list, clock)` — reads the Wren list (à la `Wavetable.from`:
  `get_list_count`/`get_list_element`), creates `Kind::Steps` (clock on port 0),
  emits `set_param(0, len)` and `set_param(k+1, values[k])` for `k in 0..len`
  (`len = count.min(MAX_STEPS)`), returns a mono node.
- Sugar classes: `class SampleHold { static new(input, clock) {…} }`, `class Slew
  { static new(input, time) {…} }`, `class Steps { static new(values, clock) {…}
  }`. No new setters. Registered in both binding tables.

---

## 4. QA acceptance & testing

**Kernel (`deluge-dsp-kernels`):**
- **S&H latches on the edge, holds between.** Feed a ramp input and a clock that
  rises at known samples: the output steps to the input value at each edge and is
  flat between edges; a clock that never rises → the output stays at the initial
  held value.
- **Slew converges + passes DC.** A step input (0 → 1) with `time`: the output
  rises monotonically toward 1, reaching ~63 % within ~`time` seconds; a constant
  input is passed through (steady state == input); very small `time` ≈ instant.
- **Steps walks the values.** `[10, 20, 30]` clocked: first edge → 10, then 20,
  30, 10 (wrap); `len` and per-step `set_value` respected; unclocked → step 0.
- **Bounded.** Proptest: S&H/Steps output ∈ range of their inputs/values; Slew of
  a bounded input stays bounded; all finite.

**Graph/Wren:**
- `Kind::SampleHold`/`Slew`/`Steps` render bounded, mono; S&H holds, Slew smooths,
  Steps sequences (behavioral, small).
- `SampleHold.new(in, clk)` / `Slew.new(in, t)` emit the right `Kind` `NewNode`
  with the right ports; `Steps.new([…], clk)` emits `NewNode(Steps)` +
  `SetParam(0, len)` + one `SetParam(k+1, …)` per value.

**Determinism:** no RNG (S&H/Steps just latch/index); reproducible; proptests seeded.

---

## 5. Deferred / follow-ups

- **`Steps` trigger/gate reset** (to step 0), **per-step gates/ratchets, > 16
  steps (pooled), swing** — later sequencer refinements.
- **Slew: separate rise/fall times, linear (constant-rate) mode** — later.
- **Internal clock / tempo** for S&H and Steps — later (Mod-3 or a clock node).
- **Mod-3 modulation scaling / attenuators / macros** (extends Mod-1's `.to()`).
