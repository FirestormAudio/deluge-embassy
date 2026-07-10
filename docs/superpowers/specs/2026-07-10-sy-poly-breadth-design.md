# Sy-2b: Poly breadth (waveshapes + PolyAdd + PolyNoise) — design & spec

The **poly-breadth** pass. The Sy suite (Sy-1–4) ships a playable Wren `Synth`,
but inside a `Synth` builder only `Osc.sine`, `.lpf`, `Env.ar`, and `*` work —
`Osc.saw/square/tri`, `+`, and `Noise` `Fiber.abort`. Sy-2b poly-ifies the
**core subtractive** kernels so those routes work: band-limited poly oscillator
waveshapes, a poly mixer (`PolyAdd`), and a poly white-noise source
(`PolyNoise`). Filters/Sync/Wavetable stay deferred (Sy-2c).

> **Status:** design proposal. Depends on merged Sy-1–4 (poly kinds,
> `poly_process`/`poly_in_count`, the Wren `Synth` surface + prelude aborts) and
> the mono `Osc` band-limiting (`wave_sample`/`poly_blep` in `osc.rs`).
> MIT/Apache. `no_std`, device-first (Cortex-A9, VFPv3 + NEON).

**Sy roadmap:** Sy-1..4 ✓ → **Sy-2b core subtractive breadth (this)** → Sy-2c
timbral breadth (poly filters / Sync / Wavetable) → Sy-5 expressiveness.

---

## 1. Goals & non-goals

**Goals**
- **`PolyOsc` waveshapes.** Add `shape ∈ {Sine, Saw, Square, Tri}` to `PolyOsc`
  (currently sine-only), settable via `set_param(0, code)`. Per voice, per
  sample: `dtp = freq·dt` (freq from the poly pitch input), advance phase, emit
  the **band-limited** sample. Reuses the scalar `wave_sample` (made
  `pub(crate)`) as the oracle; the `f32x8` fast path adds branchless SIMD
  counterparts of `poly_blep`/`blep_right`/`poly_blamp`, null-tested. Sine keeps
  `fast_sin_x8`. Square PWM fixed at 0.5.
- **`PolyAdd`.** `poly_add(a, b, out)` (mirrors `poly_mul`) + `Kind::PolyAdd`
  (`poly_in_count = 2`) — the mixer for `+`.
- **`PolyNoise`.** A poly *source* (`poly_in_count = 0`, `out_width = VOICES`):
  8 independent xorshift white-noise lanes with **decorrelated per-voice seeds**.
  Scalar (noise is cheap, non-band-limited).
- **Wren routing flips.** Extend the `polyosc_` factory to take a shape code
  (`polyosc_(pitch, shape)`); add `polyadd_`/`polynoise_`; flip the prelude
  aborts: `Osc.saw/square/tri` → `polyosc_(f, code)`, `+` → `polyadd_`,
  `Noise.new()` → `polynoise_`.
- **QA-proven:** `f32x8` saw/square/tri match the scalar `wave_sample` oracle and
  are band-limited (vs mono `Osc` cross-check); `PolyNoise` lanes are
  independent, bounded, decorrelated; `PolyAdd` is lanewise; a Wren `Synth` with
  `Osc.saw`, `+` mixing, and `Noise.new()` builds and sounds; both feature
  configs.

**Non-goals (deferred)**
- **Poly Moog/Tb303/Ms20/Modal filters, poly hard-Sync, poly Wavetable** — Sy-2c
  (each a real recurrent-kernel poly-ification); their prelude aborts stay.
- **Per-voice PWM (Square width), poly pink/brown noise** (filtered) — later;
  `Noise.pink()`/`.brown()` in a `Synth` keep aborting.
- **Velocity, portamento, unison, multiple envelopes** — Sy-5.

---

## 2. `PolyOsc` waveshapes (`deluge-dsp-kernels/src/poly.rs` + `osc.rs`)

### 2.1 Expose the scalar band-limiting (osc.rs)
Make `wave_sample(wave: Wave, ph: f32, dtp: f32, width: f32) -> f32` and the
helpers it calls (`poly_blep`, `blep_right`, `poly_blamp`) `pub(crate)` (they are
currently private `fn`s). No behavior change — the mono `Osc`/`SyncOsc` still use
them; `PolyOsc` reuses them as the scalar path + oracle.

### 2.2 `PolyOsc` gains a shape
```rust
pub struct PolyOsc {
    phase: [f32; VOICES],
    shape: Wave,           // NEW; default Sine (Sy-1 behavior preserved)
}
impl PolyOsc {
    pub fn new() -> PolyOsc;                 // shape = Sine
    pub fn set_shape(&mut self, code: u8);   // 0=Sine,1=Saw,2=Square,3=Tri
    pub fn process(&mut self, pitch: &[f32], dt: f32, out: &mut [f32]);
}
```

**Per sample per voice** (scalar path, `#[cfg(not(feature = "simd"))]`):
```text
let f = pitch[i*VOICES+v];
let dtp = f * dt;                 // per-voice phase increment
let mut p = phase[v] + dtp; p -= floorf(p);
phase[v] = p;
out[i*VOICES+v] = wave_sample(self.shape, p, dtp, 0.5);   // reuse scalar oracle
```
(Sine → `wave_sample` → `fast_sin`; identical to Sy-1's sine output.)

### 2.3 `f32x8` band-limited fast path (`osc.rs`, new `pub(crate)` helpers)
Add branchless `f32x8` counterparts, matching the scalar polynomials lane-for-lane
(the residual is small, so agreement is to f32 rounding):
- `blep_right_x8(x: f32x8) -> f32x8` — `x.simd_lt(1)` selects the two quartic
  pieces.
- `poly_blep_x8(t: f32x8, dtp: f32x8) -> f32x8` — `t.simd_lt(2·dtp)` /
  `t.simd_gt(1 − 2·dtp)` select `blep_right_x8(t/dtp)` / `−blep_right_x8(−(t−1)/dtp)`
  / `0`.
- `poly_blamp_x8(t: f32x8, dtp: f32x8) -> f32x8` — the two cubic pieces via select.
- `wave_sample_x8(shape, ph: f32x8, dtp: f32x8) -> f32x8` — the four shapes
  (Square PWM fixed 0.5), reusing the above; Sine → `fast_sin_x8`.

`PolyOsc::process` (`#[cfg(feature = "simd")]`) keeps `phase` as `f32x8` across
the block; per sample: `f = f32x8::from_slice(pitch[i*V..])`, `dtp = f * dt`,
advance/wrap phase (existing floor-trick), `wave_sample_x8(shape, p, dtp)` →
`copy_to_slice`. `const _: () = assert!(VOICES == 8)` already guards it.

Phase wrap for the SIMD path reuses the existing `p.cast::<i32>().cast::<f32>()`
trunc-floor from the Sy-1 `PolyOsc`.

---

## 3. `PolyAdd` + `PolyNoise` (`poly.rs`)

### 3.1 `PolyAdd`
```rust
/// out[j] = a[j] + b[j]. Two poly inputs. Stateless.
pub fn poly_add(a: &[f32], b: &[f32], out: &mut [f32]);
```
Element-wise; auto-vectorizes (no hand-SIMD). Graph `Kind::PolyAdd`
(`State::Stateless`, `poly_in_count = 2`).

### 3.2 `PolyNoise`
```rust
/// 8 independent xorshift white-noise lanes. A poly source (no input).
pub struct PolyNoise { rng: [u32; VOICES] }
impl PolyNoise {
    pub fn new() -> PolyNoise;               // decorrelated per-voice seeds
    pub fn process(&mut self, out: &mut [f32]);   // interleaved, [-1,1]
}
```
`new()` seeds each lane distinctly (e.g. `rng[v] = SEED0 ^ (0x9E3779B9 * (v+1))`,
all non-zero — xorshift must not be seeded 0). Per sample per voice: xorshift32
advance → bipolar `[-1, 1]` (reuse the crate's existing noise mapping from
`noise.rs`). Scalar; `no_std`, deterministic.

---

## 4. Graph & Wren surface

**`deluge-audio-graph` (`node.rs`):**
- `enum Kind`: add `PolyAdd`, `PolyNoise`. `enum State`: `PolyNoise(PolyNoise)`;
  `PolyAdd` is `State::Stateless`.
- `out_width` → `VOICES` for both. `is_poly` includes both.
  `poly_in_count`: `PolyAdd → 2`, `PolyNoise → 0`.
- `PolyOsc` gains a `set_param` arm: `0 => set_shape(value as u8)`.
- Render: `PolyAdd` → `poly_add(poly_in[0], poly_in[1], out)`; `PolyNoise` →
  `n.process(out)` (no input). `process_resolved` no-op arm extended.

**`deluge-wren-core`:**
- Extend `node_polyosc_impl` to read a shape arg: `polyosc_(pitch, shape)` →
  `Kind::PolyOsc` [pitch] + `set_param(0, shape)`. (Update its selector arity in
  both tables + the `Osc.sine` route to pass `0`.)
- Add `node_polyadd_impl(a, b)` → `Kind::PolyAdd`; `node_polynoise_impl()` →
  `Kind::PolyNoise`. Register in both tables + `foreign static` decls.
- **Prelude route flips** (`polyMode_ == 1` branch now routes instead of aborts):
  - `Osc.sine(f)` → `polyosc_(f, 0)`; `Osc.saw` → `polyosc_(f, 1)`;
    `Osc.square` → `polyosc_(f, 2)`; `Osc.tri` → `polyosc_(f, 3)`.
  - `+(o)` on Node/Port → poly `polyadd_(this, o)` (if `o is Num`, abort like
    `*` — poly + constant is a different op).
  - `Noise.new()` → poly `polynoise_()`. `Noise.pink()`/`.brown()` still abort
    ("poly pink/brown noise not supported yet").

---

## 5. QA acceptance & testing (both feature configs)

**Kernels (`deluge-dsp-kernels`):**
- **Waveshape null test:** for saw/square/tri, `f32x8` `wave_sample_x8` matches
  scalar `wave_sample` lane-for-lane (< 1e-4) over a phase/dtp sweep.
- **Band-limited cross-check:** a `PolyOsc` saw voice matches a mono `Osc` saw on
  the same frequency (both use `wave_sample`) within tolerance; the saw is
  band-limited (worst-case alias below the naïve saw, reusing the existing
  spectrum harness if convenient, or a direct `wave_sample` equality).
- **`PolyNoise`:** each lane finite, `∈ [-1, 1]`; lanes are **decorrelated**
  (lane 0 ≠ lane 1 sample-for-sample); reproducible (fixed seeds).
- **`poly_add`:** lanewise sum of two interleaved tiles.

**Graph:** `PolyOsc` `set_param(0, code)` selects the shape (a Saw node renders a
band-limited saw); `PolyAdd` sums two poly sources; `PolyNoise` renders bounded
non-silent 8 lanes; `poly_in_count(PolyAdd)==2`, `(PolyNoise)==0`.

**Wren (`deluge-wren-core`):**
- `Synth { |p| Osc.saw(p).lpf(1200) * Env.ar(0.01,0.3) }` builds (emits
  `PolyOsc` with `SetParam(0,1)`) and renders non-silent.
- `Synth { |p| (Osc.sine(p) + Osc.saw(p)) * Env.ar(0.01,0.3) }` builds a
  `PolyAdd` and renders (mixing works — the earlier `+` abort is gone).
- `Synth { |p| Noise.new() * Env.ar(0.01,0.3) }` builds a `PolyNoise` and renders
  non-silent.
- `Noise.pink()` inside a `Synth` still aborts (deferred).
- The former error tests that now WORK (`Osc.saw`, `+`, `Noise`) are removed from
  the abort list; the still-deferred ones (Sync/Wavetable/pink) stay.

**Determinism:** xorshift seeded from constants; `f32x8` null-tested against scalar.

---

## 6. Deferred / follow-ups

- **Sy-2c:** poly `Moog`/`Tb303`/`Ms20`/`Modal` filters (SoA/`f32x8` recurrent
  kernels), poly hard-`Sync`, poly `Wavetable` (per-voice table reads).
- **Per-voice PWM** (Square width as a control), **poly pink/brown noise**
  (filtered white) — later.
- **Sy-5:** velocity → amp, portamento, unison/detune, multiple envelopes,
  sustain.
