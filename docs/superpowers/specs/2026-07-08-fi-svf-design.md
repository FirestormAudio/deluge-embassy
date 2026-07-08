# Fi-1: State-Variable Filter (SVF) — design & spec

The first sub-project of the **Fi (Filter) suite**: a scalar, `no_std`, block-oriented
**Zavalishin TPT state-variable filter** with LP/HP/BP/notch responses, resonance up to
self-oscillation, and audio-rate-modulatable cutoff. It also stands up the **reusable
filter QA harness** the rest of the suite will reuse.

> **Status:** design proposal. Depends on the merged P0 graph foundation (`Kind`/`State`/
> `Node`, the `In` input type), the P1 Wren bindings, and the `deluge-dsp-test` QA harness.
> Adapts the topology of the first-party `spark` `StateVariableFilter` (owned code — no
> licensing constraint), rewritten from scratch to fit our block/`In` kernel model.

**Fi roadmap (context, not this spec):** Fi-1 SVF → Fi-2 TB-303 diode ladder → Fi-3
Huovilainen Moog → Fi-4 MS-20 Sallen-Key → Fi-5 modal resonator. Each is its own
spec → plan → build cycle. This spec covers **only Fi-1**.

---

## 1. Goals & non-goals

**Goals**
- **TPT SVF kernel.** A `Svf` struct (two trapezoidal-integrator states) producing the
  four standard responses — lowpass, highpass, bandpass, notch — from one shared state,
  via the Cytomic/Zavalishin two-integrator loop. Unconditionally stable (the TPT
  property), self-oscillating at maximum resonance, cutoff usable up to near Nyquist.
- **Const-cutoff fast path (the performance shape).** When `cutoff`/`res` are block-
  constant (`In::as_const()`), compute the coefficients (`g`, `k`, and the shared
  denominator) **once per block** using an exact `tan`, then run a tight scalar
  recurrence. This mirrors the oscillator/wavetable const-freq hoist and is the primary
  path (fixed or control-rate cutoff is the common case).
- **Audio-rate cutoff.** When `cutoff` (or `res`) is `In::A`, fall to a per-sample
  coefficient update using a **polynomial `tan` prewarp approximation** (the Cortex-A9 has
  no hardware transcendentals, so per-sample exact `tan` is too costly). Bounded and
  stable across audio-rate cutoff sweeps.
- **Musician-facing resonance.** `res ∈ [0, 1]` mapped internally to the damping `k = 1/Q`,
  so `res=0` is gently damped and `res=1` reaches the edge of self-oscillation.
- **Graph + Wren surface.** Four node kinds and a per-filter Wren class `Svf` with
  `lp/hp/bp/notch` factories and modulatable `cutoff=`/`res=`.
- **Reusable filter QA harness.** Spectral + stability test helpers in `deluge-dsp-test`
  that this and every later Fi sub-project reuse (magnitude response at a frequency,
  −3 dB point, peak-gain-vs-resonance, boundedness sweeps, self-oscillation detection).

**Non-goals (deferred / out of scope)**
- **SIMD.** A single filter is a serial IIR recurrence — output sample *n* depends on
  *n−1*'s state — so there is no across-sample vectorization to win (unlike the closed-form
  oscillator phase). SIMD returns only as **cross-voice batching** (N filter instances in
  lockstep) at the future voice/`Sy` layer, uniformly across all kernels. No
  `#[cfg(feature="simd")]` dual-path here; the scalar kernel is the whole implementation.
- **Multi-output ports.** Exposing all four responses from one node (`Svf.new(...)` with
  `.lp/.hp/.bp/.notch` ports) needs multi-output-port support the graph does not have yet.
  Deferred to a later graph-layer feature; Fi-1 ships one response per node.
- **Nonlinear / saturating SVF** (drive in the feedback path) — a later refinement.
- **The other Fi sub-projects** (TB-303, Moog, MS-20, resonator) — separate specs.
- Replacing the existing `OnePole`/`Kind::Lpf` — it stays as a cheap utility (and is used
  by existing golden tests).

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/filter.rs`, new `Svf` beside `OnePole`)

```rust
pub struct Svf { ic1eq: f32, ic2eq: f32 }   // the two TPT integrator states

#[derive(Clone, Copy)]
pub enum SvfResp { Lp, Hp, Bp, Notch }

impl Svf {
    pub fn new() -> Svf { Svf { ic1eq: 0.0, ic2eq: 0.0 } }
    // resp is fixed per node (chosen by the graph Kind).
    pub fn process(&mut self, input: In, cutoff: In, res: In, resp: SvfResp,
                   dt: f32, out: &mut [f32]);
}
```

**Coefficients (Cytomic/Andy Simper TPT form).** For cutoff `fc`, resonance `res`:

```
g  = tan(π · fc · dt)             // prewarped cutoff (dt = 1/fs)
k  = 2.0 * (1.0 - res)            // damping = 1/Q; res∈[0,1] → k∈[0,2]; clamp k ≥ K_MIN
a1 = 1.0 / (1.0 + g * (g + k))
a2 = g * a1
a3 = g * a2
```

`res` is clamped to `[0,1]`; `k` is floored at a small `K_MIN` (e.g. `1e-4`) so `res=1`
gives a very-high-Q, constant-amplitude self-oscillation that stays numerically bounded
(a pure `k=0` linear resonator is only marginally stable). The exact `res→k` curve and
`K_MIN` are **tuned against the QA gates** (§4), not guessed — a wrong map shows up as a
self-oscillation that decays or grows.

**Per-sample recurrence** (`v0` = input sample):

```
v3    = v0 - ic2eq
v1    = a1 * ic1eq + a2 * v3
v2    = ic2eq + a2 * ic1eq + a3 * v3
ic1eq = 2.0 * v1 - ic1eq
ic2eq = 2.0 * v2 - ic2eq
```

**Responses** (cheap linear combos of the same state — `resp` selects which is written):

```
Lp    = v2
Bp    = v1
Hp    = v0 - k * v1 - v2
Notch = v0 - k * v1          // = Lp + Hp
```

**Const-cutoff fast path.** When `cutoff.as_const()` and `res.as_const()` are both `Some`,
compute `g` (via exact `tan`), `k`, `a1`, `a2`, `a3` **once**, then loop the recurrence
with those constants. This is the primary path.

**Audio-rate path.** When either is `In::A`, recompute `g` (via the polynomial `tan`
prewarp), `k`, `a1..a3` **per sample** from `cutoff.at(i)`/`res.at(i)`. `fc` is clamped to
`(0, NYQUIST_FRAC · fs)` (e.g. `0.49·fs`) so `g = tan(π·fc·dt)` stays finite and the
approximation stays in its valid range.

**The `tan` prewarp approximation** (audio-rate path). `π·fc·dt ∈ [0, π/2)`; a rational
(Padé-style) approximation of `tan` on that interval, accurate enough that the audio-rate
path's cutoff matches the const path's within the QA tolerance across the cutoff range.
The const path uses exact `tan` (once per block, cost irrelevant). `no_std`: exact `tan`
comes from `libm` (already a dependency) or a shared crate helper; the approximation is
pure arithmetic.

`no_std`; deterministic; state is the two integrator values. No heap, no SIMD.

---

## 3. Graph & Wren surface

**`deluge-audio-graph`:**
- Add `Kind::SvfLp`, `Kind::SvfHp`, `Kind::SvfBp`, `Kind::SvfNotch`. `Node::new` maps all
  four to `State::Svf(Svf::new())`. The render arm maps the kind → `SvfResp` and calls
  `svf.process(ins[0], ins[1], ins[2], resp, dt, outs.port(0))` — port 0 = input signal,
  port 1 = cutoff, port 2 = resonance. `out_width` = 1.
- Inputs default: cutoff and res as `Input::Const(..)` when not patched.

**`deluge-wren-core`:**
- A new `Svf` Wren class with four static factories:
  `Svf.lp(input, cutoff, res)` / `Svf.hp(...)` / `Svf.bp(...)` / `Svf.notch(...)`, each
  emitting `NewNode { kind: Kind::Svf*, args: [input, cutoff, res] }` (mirroring
  `node_src_impl`/`node_lpf_impl`, with a `svf_resp_kind`-style code→kind mapping).
- Instance setters `cutoff=(_)` and `res=(_)` for post-creation modulation (mirroring the
  existing `cutoff=(_)` on the `Lpf` node — reuse `node_set_cutoff_impl`, add
  `node_set_res_impl`).
- Registration in the binding tables alongside the existing filter methods.

---

## 4. QA acceptance & testing

**New reusable filter helpers in `deluge-dsp-test`** (the harness the whole Fi suite reuses):
- `magnitude_response(kernel_render, freq, fs) -> f32` — drive a sine at `freq` through a
  render closure, measure steady-state output/input amplitude ratio (skip the transient).
- `minus_3db_point(...)` — sweep frequencies, find where response crosses −3 dB.
- helpers to detect **self-oscillation** (sustained tone with no input) and its frequency.

**`deluge-dsp-kernels` tests (via `deluge-dsp-test`):**
- **Boundedness / finiteness (P0 gate).** Proptest: for randomized `cutoff ∈ [20, 20k]`,
  `res ∈ [0, 1]`, and bounded input, every output sample over a block is finite and within
  a bound (e.g. `[-4, 4]` — resonance can boost, so not `±1`). Includes `res = 1`.
- **Frequency-response ordering per mode.** LP: response at `0.1·fc` ≫ response at `10·fc`.
  HP: the reverse. BP: peak near `fc`, attenuated on both sides. Notch: deep dip near `fc`,
  ~unity far above/below. (Measured thresholds, set from measurement.)
- **Cutoff accuracy.** For LP (and HP), the measured −3 dB point is within a tolerance of
  the set `cutoff` across several cutoffs (validates `g`/the `tan` prewarp).
- **Resonance boosts peak gain.** For BP (or LP near cutoff), peak gain at `res=0.9` is
  meaningfully greater than at `res=0.1` (monotonic in `res`).
- **Self-oscillation at `res=1`.** With `res=1` and a brief impulse (or tiny noise),
  the filter sustains a tone near `cutoff` that neither decays to silence nor grows
  unbounded over a long render — validates the `res→k` map and `K_MIN`.
- **Const-vs-audio-rate equivalence.** A block with constant-valued `In::A` cutoff/res
  (audio-rate path) matches the `In::K` const fast path within a documented f32 tolerance
  (the polynomial-`tan` vs exact-`tan` difference — tolerance measured, not guessed).
- **Audio-rate cutoff sweep is stable.** A block sweeping cutoff per sample (e.g. 100 Hz→
  10 kHz) at high `res` stays finite and bounded.

**Graph/Wren:** a `Kind::SvfLp` node renders bounded/non-silent; `Svf.lp(in,c,r)` emits the
right `Cmd` and renders; `cutoff=`/`res=` update the running node.

**Determinism:** no RNG in the kernel; reproducible host/device. Proptests seed-fixed.

---

## 5. Deferred / follow-ups

- **Cross-voice SIMD** (batch N SVF instances) at the voice/`Sy` layer — the only place
  filter SIMD pays off.
- **Multi-output SVF node** (all four responses from one node) — needs multi-output ports.
- **Nonlinear/saturating SVF** (feedback drive) for extra character.
- **Peak/allpass responses** (also derivable from the same state) if wanted later.
- Fi-2..Fi-5 (TB-303, Huovilainen Moog, MS-20, modal resonator) — separate specs; each
  reuses this spec's filter QA harness.
