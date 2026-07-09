# Ef-1: Delay — design & spec

The first sub-project of the **Ef (Effects) suite**: a **delay-line primitive** (over a
borrowed pooled ring buffer) plus a **feedback delay effect** (time / feedback / mix /
damping). It also stands up the **mutable-pool-region** graph infrastructure that the whole
Ef suite (chorus, flanger, reverb) reuses — the first effects that need large, writable
sample buffers rather than the tiny fixed-state filter/osc kernels.

> **Status:** design proposal. Depends on the merged graph `Pool`/`PoolHandle`
> (`deluge-audio-graph/src/pool.rs`), the wavetable's pooled-region binding
> (`TableSrc::Pooled`, `Cmd::BindTable`, `Node::process_resolved`'s `pool_region`), and P1
> Wren. Reuses that pooled-region mechanism (generic `[f32]` region, "table"-flavored name)
> for the delay's ring buffer. spark's delay/chorus/reverb are **reference-only** (their
> `FxEngine` is heap/`std`); we reimplement over a borrowed `&mut [f32]`. (Do NOT port the
> GPL-3.0 `deluge/fx/eq.rs` / `freeverb` — this crate is MIT/Apache.)

**Ef roadmap (context):** **Ef-1 Delay** (this spec) → Ef-2 Chorus/Flanger → Ef-3 Reverb
(Dattorro/FDN) → Ef-4 Drive (waveshaper, reuses `pade_tanh`) → Ef-5 EQ (RBJ biquad, from
scratch — not the GPL DelugeEq).

---

## 1. Goals & non-goals

**Goals**
- **`DelayLine` kernel primitive.** A tiny `Copy` struct holding only a **write cursor**; the
  ring buffer is a borrowed `&mut [f32]` passed in per call (never owned — 1 s @ 48 kHz =
  192 KB can't live in a kernel struct or a const array). Ops:
  - `write(buf, x)` — store + advance the cursor (mod `buf.len()`).
  - `read_hermite(buf, delay)` — **4-point Hermite fractional** read (smooth arbitrary /
    modulated delay times; the chorus/flanger primitive).
  - `allpass(buf, delay, gain)` — Schroeder allpass (stub/used by Ef-3 reverb diffusers).
- **Mutable-pool-region graph path (the reusable infrastructure).** Widen
  `Node::process_resolved`'s `pool_region: Option<&[f32]>` → **`Option<&mut [f32]>`** (the
  wavetable arm reborrows it immutably — no behavior change); the engine resolves a pooled
  node's region via `pool.slice_mut(h)` instead of `slice(h)`. The delay node binds a pooled
  ring buffer (via the existing `TableSrc::Pooled`/`Cmd::BindTable`) sized for its max delay.
- **`Delay` feedback effect.** Per sample: read the delayed sample at `time`, output
  `dry·(1−mix) + delayed·mix`, and write `input + feedback · damp(delayed)` back into the
  ring — a one-pole low-pass `damp` in the feedback for darker repeats. `feedback` clamped
  `< 1` for stability; a soft output bound.
- **Graceful degradation.** If no pool region is bound / the pool is exhausted (alloc
  returns `None`), the delay renders **dry passthrough**, never panics (mirroring the
  wavetable's silence-on-missing-region).
- **QA-proven:** impulse → a delayed copy at `time`; feedback → decaying repeats; mix →
  dry/wet balance; damping → darker repeats; fractional `time` is smooth (Hermite); bounded.

**Non-goals (deferred / out of scope)**
- **Modulated delay time / chorus / flanger** — Ef-2 (adds an LFO on the fractional read).
- **Reverb** (multi-tap FDN/Dattorro) — Ef-3.
- **Tempo sync, ping-pong/stereo, tape/BBD emulation** — later refinements.
- **Dynamic pool resize** — the buffer length is fixed at bind time (max delay).
- **Audio-rate feedback/mix** beyond what the `In` model already gives per block.

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/`) — new `delay.rs` (or in a shared effects module)

```rust
/// A delay line over a caller-owned ring buffer. Holds only the write cursor;
/// the buffer (`&mut [f32]`) is passed per call. `no_std`, no heap.
#[derive(Clone, Copy)]
pub struct DelayLine { w: usize }

impl DelayLine {
    pub fn new() -> DelayLine;
    /// Write `x` at the cursor and advance (wrapping on `buf.len()`).
    pub fn write(&mut self, buf: &mut [f32], x: f32);
    /// Read `delay` samples back (fractional), 4-point Hermite interpolation.
    /// `delay` clamped to `[1.0, buf.len()−2]`.
    pub fn read_hermite(&self, buf: &[f32], delay: f32) -> f32;
    /// Schroeder allpass tap (for Ef-3). `y = -gain·x + d + gain·? …` (form pinned in Ef-3).
    pub fn allpass(&mut self, buf: &mut [f32], delay: f32, gain: f32) -> f32;
}

/// Feedback delay effect. Borrows its ring buffer each block; holds the line + a
/// one-pole damping state + control params (mix, damping).
#[derive(Clone, Copy)]
pub struct Delay {
    line: DelayLine,
    damp_z: f32,      // one-pole LP state in the feedback path
    mix: f32,         // control param, default ~0.35 (dry/wet)
    damping: f32,     // control param, [0,1] → feedback LP cutoff
}

impl Delay {
    pub fn new() -> Delay;
    pub fn set_mix(&mut self, v: f32);       // clamp [0,1]
    pub fn set_damping(&mut self, v: f32);   // clamp [0,1]
    /// `time` in seconds (→ samples via dt), `feedback` [0, ~0.98]. `buf` = the pooled ring.
    pub fn process(&mut self, input: In, time: In, feedback: In, dt: f32,
                   buf: &mut [f32], out: &mut [f32]);
}
```

**Per sample:** `d = time.at(i)/dt` samples (clamped `[1, buf.len()−2]`); `y = line.read_hermite(buf, d)`; one-pole damp `damp_z += c·(y − damp_z)` (`c` from `damping`); `fb = feedback.at(i).clamp(0, 0.98) · damp_z`; `line.write(buf, input.at(i) + fb)`; `out[i] = input.at(i)·(1−mix) + y·mix`. Optional soft output bound.

`no_std`; pure `f32`; the kernel owns no storage — the buffer is borrowed. Deterministic.

---

## 3. Graph & Wren surface

**`deluge-audio-graph` — the infrastructure change:**
- **Widen `Node::process_resolved`** `pool_region: Option<&[f32]>` → `Option<&mut [f32]>`.
  The `Kind::Wavetable` arm reborrows it immutably (`pool_region.as_deref()` /
  `.map(|r| &*r)`) — identical behavior. The engine's resolution (`engine.rs`) changes
  `self.pool.slice(h)` → `self.pool.slice_mut(h)` (still a disjoint field borrow of
  `self.pool` vs `self.arena`, so the existing SAFETY reasoning holds).
- Add `Kind::Delay` → `State::Delay(Delay)`. Render arm: resolve the node's pooled region
  (reusing `TableSrc::Pooled`), and if present call `d.process(ins[0], ins[1], ins[2], dt,
  region, outs.port(0))`; if absent, **dry passthrough** (`out = ins[0]`). Ports 0=input,
  1=time, 2=feedback. `out_width` = 1.
- **Buffer binding:** the delay node binds a `TableSrc::Pooled(handle)` (the ring buffer),
  allocated by the host via `Engine::pool_alloc(max_delay_samples)` and bound with
  `Cmd::BindTable` — the same lifecycle as a pooled wavetable (freed on `Cmd::Free`). `time`
  is bounded by the buffer length. (The `TableSrc`/`bind_table` naming is historical; it is a
  generic pooled `[f32]` region. A later cleanup may rename it `PooledBuf`/`bind_buffer`.)
- **Control params** (mix, damping) via `Node::set_param` (index 0/1): extend the match with
  `State::Delay(d) => match param { 0 => d.set_mix(v), 1 => d.set_damping(v), _ => {} }`.

**`deluge-wren-core`:**
- A `Delay` Wren class: `Delay.new(input, time, feedback)` — a `node_delay_impl` that creates
  the node AND allocates+binds its pool buffer (mirroring the pooled-wavetable factory
  `node_wavetable_pooled_impl`: allocate `max_delay` samples, `Cmd::BindTable`). Default max
  delay ~1 s (48000 samples). Setters `mix=`→`set_param(0)`, `damping=`→`set_param(1)`;
  `time`/`feedback` via construction/patching (ports 1/2). Registered in both binding tables
  + `class Delay` in `prelude.wren`.

---

## 4. QA acceptance & testing

Reuse the Fi-1 harness where useful; new delay-specific checks over a caller-provided buffer.

**`deluge-dsp-kernels` tests (the kernel takes a `&mut [f32]` buffer directly):**
- **Impulse → delayed copy.** A unit impulse in, `feedback=0`, `mix=1`, `time=T`: the output
  has an impulse ≈ `T` samples later (within the Hermite interpolation error), and ~nothing
  before it.
- **Feedback → decaying repeats.** `feedback=0.5`: repeats at `T, 2T, 3T…` with geometrically
  decreasing amplitude; `feedback=0.9` decays slower; all bounded.
- **Mix balance.** `mix=0` → output ≈ dry input; `mix=1` → output ≈ pure delayed; monotonic
  in between.
- **Damping darkens repeats.** With `feedback` high, higher `damping` → the repeats lose
  high-frequency energy faster (measure spectral centroid of a late repeat).
- **Fractional time is smooth.** A non-integer `time` (Hermite) produces a clean delayed
  impulse (no large interpolation artifact / not just nearest-sample).
- **Boundedness (P0 gate).** Proptest: `time ∈ [1 sample, buf.len()−2]`, `feedback ∈ [0,1]`
  (clamped to 0.98 internally), `mix`/`damping ∈ [0,1]`, bounded input → finite, within a
  measured bound (feedback<1 + damping keep it stable).
- **Graceful degradation.** (Graph-level) a `Kind::Delay` node with NO bound buffer renders
  dry passthrough (out == in), never panics.

**Graph/Wren:** a `Kind::Delay` node with a bound pool buffer renders a delayed/fed-back
signal (bounded); the `Kind::Wavetable` tests still pass after the `pool_region` widening
(no behavior change); `Delay.new(in,t,fb)` allocates+binds a buffer and emits the right
`Cmd`s; `mix=`/`damping=` update the node; pool-exhaustion (no handle) → dry passthrough.

**Determinism:** no RNG; reproducible; proptests seed-fixed.

---

## 5. Deferred / follow-ups

- **Chorus/Flanger** (Ef-2): an LFO modulating the `read_hermite` delay + feedback (flanger).
- **Reverb** (Ef-3): the `allpass` primitive + a Dattorro/FDN network of pooled delay lines.
- **Rename** `TableSrc`/`bind_table`/`Cmd::BindTable` → generic `PooledBuf`/`bind_buffer`
  (now that pooled regions aren't wavetable-specific) — a mechanical cleanup.
- **Tempo sync, ping-pong/stereo, tape/BBD** delay refinements.
- **Cross-voice / multi-instance** pool budgeting as more effects allocate buffers.
