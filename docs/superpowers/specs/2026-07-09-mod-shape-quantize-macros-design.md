# Mod-3: Shaping / Quantize / Macros — design & spec

The third and final **Mod** sub-project: the modulation-*shaping* layer. Where
Mod-1 (LFO) and Mod-2 (S&H / Slew / Steps) add modulation **sources**, Mod-3
adds the tools to **shape, map, quantize, and distribute** a modulation signal:
a non-linear **`Curve`**, a generic **`QuantStep`** and a scale-aware
**`QuantPitch`** quantizer, a semitone→Hz **`Mtof`** converter, a settable
**`Ctrl`** source that powers **macros** (one control → many destinations), and
a layer of **scaling sugar** (`.atten`/`.offset`/`.invert`/`.unipolar`/…).

> **Status:** design proposal. Depends only on the merged graph (`In`, the mono
> node pattern, `set_param`, `set_input`, the arithmetic/`.to()` prelude sugar,
> Wren list-free scalar params). MIT/Apache. `no_std`, no-heap, device-first.

**Mod roadmap:** Mod-1 LFO ✓ → Mod-2 S&H / Slew / Steps ✓ → **Mod-3 Shaping /
Quantize / Macros (this)** — completes the Mod suite.

**Why one spec:** linear scale/offset/attenuvert are already expressible with
the shipped `*`/`+`/`.to()` sugar, so Mod-3's genuinely-new content is the five
kernels below plus two thin Wren layers. They share one theme (shape a
modulation signal), so they ship together. `Mtof`'s `2^x` uses `libm::exp2f`
(`libm` is already a kernel dependency — `log2f`/`sinf`/`sqrtf`/`floorf` are all
used today); no custom exp2 primitive is needed.

---

## 1. Goals & non-goals

**Goals**
- **`Curve`** — a non-linear, odd-symmetric transfer function on a bipolar
  signal, via Schlick's *bias*. Continuous knob `k ∈ [−1, 1]` (port 1,
  modulatable): `k=0` linear, `k>0` exponential (ease-in), `k<0` logarithmic
  (ease-out). Named presets `Curve.exp`/`Curve.log`. `sig.curve(k)`.
- **`QuantStep`** — snap a bipolar `[−1, 1]` signal to `N` evenly-spaced levels
  (endpoints inclusive). `param 0 = N` (clamp `≥ 2`). `sig.steps(n)`.
- **`QuantPitch`** — snap a semitone-valued signal to the nearest degree of a
  musical scale. `param 0 = 12-bit pitch-class mask`, `param 1 = root (0–11)`.
  Output in semitones. `sig.quantize(scale, root)`.
- **`Mtof`** — semitone→Hz: `ref · 2^(x/12)`. `param 0 = reference Hz`
  (default `440`). Input = semitones above the reference. `sig.hz(ref)`.
- **`Ctrl` + macros** — a source node holding a scalar (`param 0 = value`,
  settable at runtime), no input ports. Because it is a *node*, one `Ctrl` fans
  out to many destinations through the existing arithmetic/`.to()` sugar — that
  *is* a macro. `Macro.new(v)` → `Ctrl` with a `.value=` setter.
- **Scaling sugar** — Wren methods on `Node`/`Port` wrapping existing
  arithmetic, plus wiring methods to the new nodes.
- **QA-proven:** Curve monotonic/odd-symmetric/linear at
  `k=0`; QuantStep hits exact levels; QuantPitch snaps known inputs to known
  degrees (incl. octave wrap + root shift); Mtof octave doublings exact; Ctrl
  outputs and updates its value; all bounded, finite, deterministic.

**Non-goals (deferred)**
- **S-curve / smoothstep shape** (not in the power/bias family), **per-axis
  asymmetric curves** — later; the bias knob covers exp/log.
- **Explicit multi-tap macro helper** (`macro.tap(dest, lo, hi)` one-liner),
  **macro morph/snapshots, named macro banks** — fan-out via existing sugar is
  enough for v1.
- **Custom user interval lists for `QuantPitch`** (beyond the named masks),
  **microtonal / non-12-TET scales, glide-on-quantize (hysteresis)** — later.
- **`QuantStep` unipolar mode / arbitrary range** — bipolar `[−1,1]` only;
  compose with `.unipolar()`/`.scale()` for other ranges.

---

## 2. `no_std` numeric note (applies to every kernel below)

`core` does not provide `f32::floor`/`round`/`exp2` (they need libm). This crate
already depends on `libm` (`libm::log2f`/`sinf`/`sqrtf`/`floorf` are used today),
plus a crate-root `pub(crate) fn floorf` (used by Lfo/Delay). For Mod-3:

- **Floor / round:** use the crate-root `floorf`; express `round(x)` as
  `floorf(x + 0.5)` (half-rounds toward +∞ — consistent with `QuantPitch`'s
  tie-up policy).
- **`2^x` (Mtof):** `libm::exp2f` — exact, no custom primitive.
- **`clamp`** is `core` (fine); for **`abs`**, use a branch
  (`if x < 0.0 { -x } else { x }`) to stay portable.

---

## 3. Kernel DSP

### 3.1 `Curve` (`deluge-dsp-kernels/src/shape.rs`, new module)

`Curve` is **stateless**, so it is a free function (like `math::mul`/`add`),
not a struct — `State::Stateless`, no per-node state.

```rust
/// Odd-symmetric non-linear transfer (Schlick bias). in0 = input, in1 = k [-1,1].
pub fn curve(input: In, k: In, out: &mut [f32]);
```

**Per sample:**
```text
let x = input.at(i).clamp(-1.0, 1.0);
let b = ((k.at(i) + 1.0) * 0.5).clamp(0.05, 0.95);   // k∈[-1,1] → b∈[0.05,0.95]
let s = if x < 0.0 { -1.0 } else { 1.0 };
let u = if x < 0.0 { -x } else { x };                // |x| ∈ [0,1]
let y = u / ((1.0 / b - 2.0) * (1.0 - u) + 1.0);     // Schlick bias, (0,0)→(1,1)
out[i] = s * y;
```

`b=0.5` (`k=0`) ⇒ denominator `= 1` ⇒ `y=u` (linear). `k>0` ⇒ ease-in
(exponential-like), `k<0` ⇒ ease-out (logarithmic-like). Monotonic on `[-1,1]`,
fixed points at `0` and `±1`, bounded `[-1,1]`.

Named factories preset `k`: `Curve.exp` → `k=+0.6`, `Curve.log` → `k=−0.6`.

### 3.2 `QuantStep` (`deluge-dsp-kernels/src/quant.rs`, new module)

```rust
/// Snap a bipolar [-1,1] signal to N evenly-spaced levels. Port 0=input.
#[derive(Clone, Copy)]
pub struct QuantStep { levels: u16 }
impl QuantStep {
    pub fn new() -> QuantStep;              // levels = 2
    pub fn set_levels(&mut self, n: u16);   // param 0, clamp >= 2
    pub fn process(&mut self, input: In, out: &mut [f32]);
}
```

**Per sample:** map `x∈[-1,1]` to `[0, N−1]`, round, map back:
```text
let n = self.levels.max(2) as f32;
let u = (input.at(i).clamp(-1.0, 1.0) + 1.0) * 0.5;   // [0,1]
let q = crate::floorf(u * (n - 1.0) + 0.5);           // nearest of N levels
out[i] = (q / (n - 1.0)) * 2.0 - 1.0;                 // back to [-1,1]
```
Endpoints inclusive: `N=2` ⇒ `{−1, +1}`; `N=3` ⇒ `{−1, 0, +1}`. Bounded.

### 3.3 `QuantPitch` (`deluge-dsp-kernels/src/quant.rs`)

```rust
/// Snap a semitone-valued signal to the nearest degree of a scale.
/// Port 0 = input (semitones). param 0 = 12-bit pitch-class mask, param 1 = root.
#[derive(Clone, Copy)]
pub struct QuantPitch { mask: u16, root: u8 }
impl QuantPitch {
    pub fn new() -> QuantPitch;             // mask = 0xFFF (chromatic), root = 0
    pub fn set_mask(&mut self, m: u16);     // param 0 (low 12 bits used)
    pub fn set_root(&mut self, r: u8);      // param 1, mod 12
    pub fn process(&mut self, input: In, out: &mut [f32]);
}
```

**Per sample:** round input to the nearest integer semitone `n`, then find the
nearest semitone `m` such that `((m − root) mod 12)` is a set bit of `mask`,
searching outward (`0, +1, −1, +2, −2, …`) so octave wrap is handled and ties
resolve upward. Chromatic mask (`0xFFF`) ⇒ identity (nearest integer semitone).
Empty mask is treated as chromatic (guard). Output = `m` (float semitones).

```text
let n = crate::floorf(input.at(i) + 0.5) as i32;      // round, ties → +∞
let mut best = n;
for d in [0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6] {
    let cand = n + d;
    let pc = (cand - root as i32).rem_euclid(12) as u16;
    if mask == 0 || (mask & (1 << pc)) != 0 { best = cand; break; }
}
out[i] = best as f32;
```

### 3.4 `Mtof` (`deluge-dsp-kernels/src/quant.rs`)

```rust
/// Semitone → Hz. Port 0 = input (semitones above ref). param 0 = ref Hz.
#[derive(Clone, Copy)]
pub struct Mtof { ref_hz: f32 }
impl Mtof {
    pub fn new() -> Mtof;                   // ref_hz = 440.0
    pub fn set_ref(&mut self, hz: f32);     // param 0
    pub fn process(&mut self, input: In, out: &mut [f32]);
}
```

**Per sample:** `out[i] = self.ref_hz * libm::exp2f(input.at(i) / 12.0)`.
`0 → ref`, `+12 → 2·ref`, `−12 → ½·ref`. Bounded/finite for finite input.

### 3.5 `Ctrl` (`deluge-dsp-kernels/src/shape.rs`)

```rust
/// A held, runtime-settable scalar source. No input ports. param 0 = value.
#[derive(Clone, Copy)]
pub struct Ctrl { value: f32 }
impl Ctrl {
    pub fn new() -> Ctrl;                   // value = 0.0
    pub fn set_value(&mut self, v: f32);    // param 0
    pub fn process(&mut self, out: &mut [f32]);   // fills out with value
}
```

All kernels: `no_std`, pure `f32`, deterministic, no heap, no buffer, mono.

---

## 4. Graph & Wren surface

**`deluge-audio-graph` (`node.rs`):**
- `enum Kind`: add `Curve`, `QuantStep`, `QuantPitch`, `Mtof`, `Ctrl`.
- `enum State`: `Curve` is stateless (`State::Stateless`, like `Mul`/`Add`);
  `QuantStep`/`QuantPitch`/`Mtof`/`Ctrl` carry their kernel structs.
- `Node::new`: construct each. `out_width` = 1 (all mono, default `_ => 1` arm).
- Render arms:
  - `Curve` → `shape::curve(ins[0], ins[1], outs.port(0))` (stateless free
    function, exactly like `Kind::Mul => math::mul(...)`).
  - `QuantStep` → `q.process(ins[0], outs.port(0))`.
  - `QuantPitch` → `q.process(ins[0], outs.port(0))`.
  - `Mtof` → `m.process(ins[0], outs.port(0))`.
  - `Ctrl` → `c.process(outs.port(0))`.
- `set_param`:
  - `QuantStep` → `0 => set_levels(value as u16)`.
  - `QuantPitch` → `0 => set_mask(value as u16)`, `1 => set_root(value as u8)`.
  - `Mtof` → `0 => set_ref(value)`.
  - `Ctrl` → `0 => set_value(value)`.
  - `Curve` has no params (k is port 1, all-ports).

**`deluge-wren-core`:**
- Factories (mirroring `node_lfo_impl` / `node_sh_impl`, all mono):
  - `node_curve_impl(input, k)` → `Kind::Curve`, ports `[input, k]`.
  - `node_qstep_impl(input, n)` → `Kind::QuantStep`, port `[input]`,
    `set_param(0, n)`.
  - `node_qpitch_impl(input, mask, root)` → `Kind::QuantPitch`, port `[input]`,
    `set_param(0, mask)` + `set_param(1, root)`.
  - `node_mtof_impl(input, ref_hz)` → `Kind::Mtof`, port `[input]`,
    `set_param(0, ref_hz)`.
  - `node_ctrl_impl(value)` → `Kind::Ctrl`, no input, `set_param(0, value)`.
- **Setters (new selectors, flat namespace — avoid collisions):**
  `value=` → `set_param(0)` (Ctrl). No other new setters (`Curve` k, `Mtof`
  ref, `QuantStep` n, `QuantPitch` mask/root are construction-time or via
  existing generic setters if present; keep construction-time for v1).
- **Prelude sugar** (`wren/prelude.wren`):
  - `class Curve { static exp(s){…k=0.6} static log(s){…k=-0.6} static new(s,k){…} }`.
  - `class Macro { static new(v) { Node.ctrl_(v) } value=(x){…set_param(0)} }`.
  - `class Scale` — static getters returning 12-bit masks:
    `Chromatic`(0xFFF), `Major`(0b101010110101), `Minor`(0b010110101101),
    `HarmonicMinor`, `MajorPentatonic`(0b001010010101),
    `MinorPentatonic`(0b010010101001), `Dorian`, `Mixolydian`, `WholeTone`,
    `Blues`. (Bit `i` set ⇒ pitch-class `i` allowed, relative to root.)
  - Methods on `Node` **and** `Port`:
    ```wren
    atten(k)      { this * k }
    offset(c)     { this + c }
    invert()      { this * -1 }
    unipolar()    { this * 0.5 + 0.5 }   // [-1,1] → [0,1]
    bipolar()     { this * 2 - 1 }       // [0,1] → [-1,1]
    scale(m, a)   { this * m + a }
    curve(k)      { Node.curve_(this, k) }
    steps(n)      { Node.qstep_(this, n) }
    quantize(s, r){ Node.qpitch_(this, s, r) }
    hz(ref)       { Node.mtof_(this, ref) }
    ```
- Register all new foreign methods in **both** binding tables
  (`bindings.rs` wren-sys table AND `bindings_audio.rs::register_audio`),
  mirroring the Lfo/S&H registration.

**Scale mask reference** (bit `i` = pitch class `i`, root-relative; LSB = root):
Major `{0,2,4,5,7,9,11}`, natural Minor `{0,2,3,5,7,8,10}`, Major-pentatonic
`{0,2,4,7,9}`, Minor-pentatonic `{0,3,5,7,10}`. (Exact bit constants pinned in
the plan and cross-checked in tests.)

---

## 5. QA acceptance & testing

**Kernels (`deluge-dsp-kernels`):**
- **`Curve`** — `k=0` ⇒ output == input (linear, within eps); monotonic
  non-decreasing over a `−1→1` ramp for any `k`; odd-symmetric
  (`f(−x) = −f(x)`); fixed points `f(0)=0`, `f(±1)=±1`; `k>0` boosts mid
  (`f(0.5) > 0.5`), `k<0` attenuates (`f(0.5) < 0.5`); bounded `[−1,1]`.
- **`QuantStep`** — `N=2` snaps to `{−1,+1}`; `N=3` snaps to `{−1,0,+1}`; a ramp
  produces exactly the expected staircase; bounded.
- **`QuantPitch`** — Major/root 0: `1→0` or `2` (nearest, tie→up ⇒ `2`), `3→4`,
  `6→5` or `7`; Minor-pentatonic snaps known inputs; octave wrap (`11→12` for a
  scale whose next degree is the octave); root shift moves the whole grid;
  chromatic mask = identity; empty mask guarded to chromatic.
- **`Mtof`** — `0→ref`, `+12→2·ref`, `−12→0.5·ref`, `+7→ref·1.4983…` (±cents);
  `set_ref` changes it; bounded/finite.
- **`Ctrl`** — fills the block with its value; `set_value` changes subsequent
  output; finite.
- **Boundedness proptests** — Curve (input, k ∈ ranges) finite ∈ [−1,1];
  QuantStep finite ∈ [−1,1]; Mtof finite > 0 for finite input.

**Graph / Wren:**
- Each `Kind` renders bounded, mono, with the documented behavior (Curve shapes,
  QuantStep staircases, QuantPitch snaps, Mtof converts, Ctrl holds).
- Factories emit the right `NewNode` kind + ports + `SetParam`s
  (`QuantStep`→`SetParam(0,n)`; `QuantPitch`→`SetParam(0,mask)`+`SetParam(1,root)`;
  `Mtof`→`SetParam(0,ref)`; `Ctrl`→`SetParam(0,v)`).
- `Macro.new(v).value = x` emits `SetParam(0, x)` on the `Ctrl` node; the same
  `Ctrl` referenced by two destinations fans out (both see its value).
- Sugar: `sig.curve(k)` / `.steps(n)` / `.quantize(Scale.Minor, 0)` / `.hz(220)`
  build the right nodes; `.atten/.offset/.invert/.unipolar/.bipolar/.scale`
  build the right arithmetic graphs (verify via emitted `Cmd`s or a render).
- End-to-end: `seq.to(0,24).quantize(Scale.Minor, 0).hz(220)` drives `osc.freq`
  and renders a bounded, non-silent tone.

**Determinism:** no RNG; all kernels reproducible; proptests seeded.

---

## 6. Deferred / follow-ups

- **Hand-rolled libm-free `exp2`** for the pitch path — only if a future
  requirement bans `libm` on that path (it doesn't today; `Mtof` uses
  `libm::exp2f`).
- **S-curve/smoothstep, asymmetric curves** — a distinct shape family, later.
- **Explicit multi-tap macro helper, macro snapshots/morph, named banks** —
  fan-out via existing sugar suffices for v1.
- **Custom interval lists / microtonal scales / quantize hysteresis
  (anti-chatter glide)** for `QuantPitch` — later.
- **`QuantStep` unipolar/arbitrary-range mode** — compose with sugar for now.
- **Generic runtime setters** for Curve-k / Mtof-ref / QuantStep-n / mask / root
  (beyond construction-time) if live control is wanted — small later add.
- With Mod-3 merged, the **Mod suite is complete**; next greenfield area is
  **Sy** (synth/voice: allocation, polyphony, note handling).
