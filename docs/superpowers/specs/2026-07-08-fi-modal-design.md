# Fi-5: Modal resonator — design & spec

The final sub-project of the **Fi (Filter) suite**: a **modal resonator** — a bank of N
high-Q bandpass modes (à la Mutable Instruments Rings) that ring when struck, for
plucked/struck/bowed physical-modeling timbres (strings, bells, plates). Built as
`Modal<const N>` = **N inlined SVF-bandpass modes** reusing the Fi-1 `Svf` math (no heap),
with Rings-style `freq / structure / brightness / damping / position` controls.

> **Status:** design proposal. Depends on merged Fi-1 (`Svf` bandpass tick, `svf_coeffs`,
> `svf_tan_prewarp`, `SVF_K_MIN`), the Fi-1 QA harness + `spectrum`, the P0 graph (incl.
> `Cmd::SetParam`), P1 Wren. Adapts the *behavioral* model of spark's MIT Rings
> `resonator.rs` (harmonic+stiffness ratios, Q-decay brightness, `cos(nπ·position)` comb) but
> from-scratch `no_std`/no-heap: closed-form ratios (not 256-entry LUTs), a fixed `[_; N]`
> state array (not `Vec`), coefficients once per block.

**Fi roadmap:** Fi-1 SVF ✅ → Fi-2 TB-303 ✅ → Fi-3 Moog ✅ → Fi-4 MS-20 ✅ → **Fi-5 modal
resonator** (this spec — completes the suite).

---

## 1. Goals & non-goals

**Goals**
- **N-mode bandpass bank.** `Modal<const N: usize>` (N=16) — N inlined copies of the `Svf`
  bandpass recurrence (each with its own `ic1eq`/`ic2eq`, per-mode `g_i`, per-mode `k_i`),
  summed. Reuses `svf_coeffs`/`svf_tan_prewarp`; **no heap** (`[_; N]` state array).
- **Externally excited.** The input signal (port 0) is the exciter — an impulse, noise burst,
  or audio feeds all modes and they ring. No self-excitation.
- **Rings-style controls:**
  - **freq** — the fundamental; mode `i` sits at `f_i`.
  - **structure** — inharmonicity via a **closed-form stiff-string** ratio
    `f_i = freq · i · √(1 + B·(i−1)²)`, `B = structure · B_MAX`. `structure=0` → harmonic
    (`f_i = freq·i`); rising → stretched/inharmonic (bell/plate).
  - **damping** — per-mode `k_i` (SVF resonance ⇒ Q ⇒ decay time), floored at `SVF_K_MIN`;
    the ring length.
  - **brightness** — per-mode spectral tilt (higher modes attenuated as brightness drops).
  - **position** — comb weight `gain_i ∝ |sin(i·π·position)|` (strike point; a mode at a
    node of the strike is not excited).
- **Coefficients once per block** (N modes × per-sample would be too costly on the A9): the
  per-mode `(g_i, k_i, gain_i)` are recomputed at block start from the block's control values.
- **Mono output**, normalized (the sum of N bandpasses is scaled to a sane level). Bandpasses
  have no DC → no DC blocker needed.
- **QA-proven:** struck-impulse rings-and-decays (decay ∝ damping); spectral peaks at the mode
  frequencies (harmonic vs stretched by `structure`); brightness shifts the centroid; position
  nulls the expected mode; bounded; not-ringing-forever at normal damping.

**Non-goals (deferred / out of scope)**
- **Audio-rate structure/brightness/position** — control params (per-block); audio-rate would
  force per-sample N-mode coefficient recompute. `freq`/`damping` may be audio-rate ports
  (resolved per block).
- **N > 16 / runtime-variable mode count** — `N` is a compile-time `const`; measure-up later
  (like Fi-2 oversampling) if the A9 has headroom.
- **Stereo odd/even "spread"** (Rings' dual output), **sympathetic-string / Karplus-Strong**
  models, internal exciter generator — later, separate.
- **Cross-voice SIMD** (voice/`Sy` layer). No edits to `Svf`/ladders/`Ms20` — Fi-5 reuses the
  SVF math read-only (inlined) + is additive.

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/filter.rs`, new `Modal<N>`)

```rust
pub const MODAL_MODES: usize = 16;

pub struct Modal<const N: usize> {
    modes: [(f32, f32); N],   // per-mode (ic1eq, ic2eq) — inlined SVF bandpass state
    // control params (block-rate), defaults chosen so a struck note rings musically:
    structure: f32,           // 0 = harmonic … 1 = inharmonic  (set_param 0)
    brightness: f32,          // spectral tilt                   (set_param 1)
    position: f32,            // strike position (comb)          (set_param 2)
}

impl<const N: usize> Modal<N> {
    pub fn new() -> Self;                  // sensible defaults
    pub fn set_structure(&mut self, v: f32);   // clamp [0,1]
    pub fn set_brightness(&mut self, v: f32);  // clamp [0,1]
    pub fn set_position(&mut self, v: f32);    // clamp [0,1]
    pub fn process(&mut self, input: In, freq: In, damping: In, dt: f32, out: &mut [f32]);
}
```

**Per block (`process`):** read `freq`/`damping` (block-const or first sample), and the
control params, then for each mode `i∈[1,N]`:
```
ratio_i = i · √(1 + B·(i−1)²)           // B = structure·B_MAX ; ratio_1 = 1
f_i     = (freq · ratio_i).min(0.49·fs) // clamp below Nyquist; skip/zero modes over Nyquist
g_i     = svf_tan_prewarp(π·f_i·dt)
k_i     = SVF_K_MIN + (1 − damping)·… → small k (high Q, long ring) when damping low
(a1,a2,a3)_i = svf_coeffs(g_i, k_i)
gain_i  = |sin(i·π·position)| · brightness_tilt(i) · norm   // comb × tilt × normalization
```
**Per sample:** `y = Σ_i gain_i · bandpass_tick_i(input.at(j))`, where `bandpass_tick_i` is the
`Svf` recurrence (`v3 = x − ic2eq; v1 = a1·ic1eq + a2·v3; …; return v1`) run on mode `i`'s
state. `out[j] = y`.

**The tuned constants (set by measurement, §4):** `B_MAX` (max inharmonicity), the
`damping → k` curve (decay-time mapping, exponential over a few decades), `brightness_tilt(i)`
(e.g. `brightness^(i−1)` or a dB tilt), and `norm` (output level so a struck note peaks near
unity). Modes whose `f_i` exceeds Nyquist are muted (`gain_i = 0`).

`no_std`; pure `f32` hot path (`libm`-free — `svf_tan_prewarp` is the prewarp; a per-mode
`sin` for `position` is once per block, `libm::sinf` acceptable, or a small recurrence).
Deterministic; fixed-size state. No SIMD.

---

## 3. Graph & Wren surface

**`deluge-audio-graph`:**
- Add `Kind::Modal` → `State::Modal(Modal::<MODAL_MODES>::new())`. Render arm:
  `m.process(ins[0], ins[1], ins[2], dt, outs.port(0))` — port 0 = input (exciter),
  1 = freq, 2 = damping. `out_width` = 1.
- **Control params** via `Node::set_param` (extend the existing match):
  `State::Modal(m) => match param { 0 => m.set_structure(v), 1 => m.set_brightness(v),
  2 => m.set_position(v), _ => {} }`.

**`deluge-wren-core`:**
- A `Resonator` Wren class: `Resonator.new(input, freq, damping)` (`node_modal_impl` →
  `NewNode{ kind: Modal, args:[input, freq, damping] }`). Setters: `structure=`→`set_param(0)`,
  `brightness=`→`set_param(1)`, `position=`→`set_param(2)` (each a `node_set_*` mirroring
  `node_set_drive`); `freq=`→port 1, `damping=`→port 2 (port setters like `cutoff=`).
  Registered in both binding tables + `class Resonator` in `prelude.wren`.

---

## 4. QA acceptance & testing

Reuse the Fi-1 harness (`filter_meas::self_osc_hz_and_rms` for ring detection) and
`spectrum::{analyze, level_at, peak_bin}`.

**`deluge-dsp-kernels` tests:**
- **Rings and decays (the core behavior).** Strike with a unit impulse (input `[1,0,0,…]`),
  no sustained input; the output rings (RMS of an early window ≫ noise) then **decays** (late
  window RMS ≪ early) at a normal `damping`. A wrong bank (unstable / silent) fails.
- **Decay time tracks damping.** Higher `damping` → faster decay (late/early RMS ratio smaller).
- **Modal peaks at the fundamental & partials.** Struck at `freq`, the spectrum has a peak at
  `freq` and at the low partials. At `structure=0` the 2nd peak is at ~`2·freq` (harmonic);
  at high `structure` it is **stretched** (`> 2·freq`) — assert the ratio moves.
- **Brightness shifts the centroid.** High `brightness` → more high-partial energy (spectral
  centroid / a high-partial level higher) than low `brightness`.
- **Position nulls a mode.** At `position=0.5`, mode 2 (`|sin(2π·0.5)|=0`) is suppressed —
  its partial level is far below its level at `position=0.25`.
- **Boundedness (P0 gate).** Proptest: `freq ∈ [20, 4000]`, `damping ∈ [0,1]`, structure/
  brightness/position ∈ [0,1], bounded input → every output finite and within a measured bound.
- **Does not self-oscillate at normal damping** — with `damping` mid-range, a struck note
  decays to silence (no runaway); only very low damping gives a long (bounded) ring.

**Graph/Wren:** a `Kind::Modal` node renders bounded (rings then decays); `Resonator.new(…)`
emits the right `Cmd` and renders; `structure=`/`brightness=`/`position=` (via `Cmd::SetParam`)
change the output; `freq=`/`damping=` update the ports.

**Determinism:** no RNG; reproducible; proptests seed-fixed.

---

## 5. Deferred / follow-ups

- **Measure-up `N`** (16 → 32/48) after on-device CPU measurement.
- **Stereo odd/even spread** (Rings' dual output).
- **Internal exciter** (built-in impulse/noise-burst/mallet generator) so it can be played as a
  voice without an external exciter node.
- **Sympathetic-string / Karplus-Strong** models (delay-line based) — a separate physical-model
  sub-project.
- **Cross-voice SIMD** at the `Sy` layer.
- With Fi-5 the **Fi (Filter) suite is complete** — next suites: **Mod**, **Ef** (effects),
  **Sy** (synth/voice).
