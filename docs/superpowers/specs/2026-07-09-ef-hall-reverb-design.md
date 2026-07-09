# Ef-3b: Hall reverb (FDN) — design & spec

The second reverb sub-project: an **8-line modulated Feedback Delay Network**
hall reverb, reusing the shared foundation established by Ef-3a Room (the
`DelayLine` primitive + the single-big-partitioned-pooled-buffer pattern + the
width-2 stereo node + the merged pooled/stereo-routing surface). Adds a fast
Walsh-Hadamard feedback mixer and light delay-line modulation for a dense,
smooth, lush hall tail.

> **Status:** design proposal. Depends on: `DelayLine` (`read_hermite`), `fast_sin`,
> the pooled-region graph path, `OutView::port_pair`, width-aware `Out.patch`,
> `new_pooled_node`, and the **Room param surface** (`mix=`/`damp=`/`size=`/`spread=`
> setters — all merged). MIT/Apache; FDN + Hadamard are public/textbook.

**Reverb roadmap:** Ef-3a Room (Freeverb) ✓ → **Ef-3b Hall (FDN, this)** → Ef-3c
Plate (Dattorro).

---

## 1. Goals & non-goals

**Goals**
- **`Fdn8` hall engine** — 8 delay lines (each a `DelayLine` over a partitioned
  sub-slice of one pooled buffer), mixed through a **normalized 8×8 Hadamard
  feedback matrix**, with **per-line one-pole damping** and a global **decay
  gain** from `size`. Mono input injected into all lines; **even lines → L, odd →
  R** for inherent stereo.
- **Light modulation.** Each line's read delay is modulated by a phase-spread LFO
  (via `read_hermite`) to smooth flutter — so each line's buffer slice carries a
  few samples of headroom above its base length.
- **Guaranteed stability.** The feedback is `g · H · damp(taps)` with `H`
  orthonormal (Hadamard/√8) and `g < 1`; the loop's spectral radius is `g < 1`,
  so it is **BIBO-stable by construction** for any line lengths.
- **`Kind::Hall` graph node** (width-2 stereo), mirroring `Kind::Room`; graceful
  dry passthrough when unbound / region too small.
- **Reuses the Room param surface exactly** — `set_param` 0=mix, 1=damp, 2=size,
  3=width; Wren setters `mix=`/`damp=`/`size=`/`spread=` already exist. Ef-3b adds
  ONLY the `Hall.new` factory + `class Hall`.
- **QA-proven:** impulse → a dense, decaying, smooth tail (size lengthens it, damp
  darkens it); stereo L ≠ R; mix balance; boundedness; graceful degradation.

**Non-goals (deferred)**
- **Ef-3c Plate (Dattorro)** — reuses this foundation.
- **Pre-delay, early reflections, tempo-sync, freeze, sample-rate-scaled tunings**
  (fixed 44.1 kHz sample counts), **per-line diffusion allpasses** — later.

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/reverb.rs`, alongside `Freeverb`)

```rust
/// Fast Walsh-Hadamard transform on 8 samples, normalized by 1/√8 (orthonormal
/// → lossless). 3 butterfly stages, adds/subtracts only.
fn fwht8(v: &mut [f32; 8]); // in place

/// 8-line modulated FDN hall reverb over a shared partitioned buffer.
#[derive(Clone, Copy)]
pub struct Fdn8 {
    lines: [DelayLine; 8],
    damp_z: [f32; 8],   // per-line one-pole feedback LP state
    lfo_phase: f32,     // shared LFO; per-line phase offset = k/8
    size: f32, damp: f32, width: f32, mix: f32, // params
}
impl Fdn8 {
    pub fn new() -> Fdn8;                    // size 0.5, damp 0.5, width 1, mix 0.5
    pub fn set_mix(&mut self, v: f32);       // param 0
    pub fn set_damp(&mut self, v: f32);      // param 1
    pub fn set_size(&mut self, v: f32);      // param 2 (→ decay gain)
    pub fn set_width(&mut self, v: f32);     // param 3
    pub fn process(&mut self, input: In, dt: f32, buf: &mut [f32],
                   out_l: &mut [f32], out_r: &mut [f32]);
}

/// Σ of the 8 line slice lengths (each `base_len + MOD_MARGIN`).
pub const HALL_BUF_SAMPLES: usize; // ≈ 24.6k; exact value pinned in the plan
```

**Constants (pinned in the plan, validated by a layout-sum test):** 8 **coprime
prime** base lengths spanning ~1500–4500 samples; `MOD_MARGIN` (per-line
headroom for modulation + Hermite, e.g. 16); `MOD_DEPTH` (~8 samples); LFO rate
(~0.7 Hz); input-injection and output gains.

**Per sample `i`:**
1. Advance `lfo_phase` (wrapped via `floorf`, no `fract()`).
2. For each line `k`: `d_k = base_len[k] − MOD_DEPTH·(0.5 + 0.5·fast_sin(lfo_phase +
   k/8))` (always `≤ base_len`, `> base_len − MOD_DEPTH`); `s[k] =
   lines[k].read_hermite(slice_k, d_k)`.
3. **Damping:** `damp_z[k] = s[k]·(1−dc) + damp_z[k]·dc` where `dc = damp·0.4`;
   `sd[k] = damp_z[k]`.
4. **Hadamard mix:** `h = sd; fwht8(&mut h);` (orthonormal).
5. **Feedback + inject + write:** `g = size·0.28 + 0.7`; for each `k`,
   `lines[k].write(slice_k, input.at(i)·INJ + g·h[k])`.
6. **Output:** `ol = (s[0]+s[2]+s[4]+s[6])·OUT; or = (s[1]+s[3]+s[5]+s[7])·OUT`;
   `wet1 = mix·(width·0.5+0.5); wet2 = mix·((1−width)·0.5); dry = 1−mix`;
   `out_l[i] = x·dry + ol·wet1 + or·wet2; out_r[i] = x·dry + or·wet1 + ol·wet2`
   (`x = input.at(i)`).

**Buffer safety:** `process` dry-passes-through (both = input) if `buf.len() <
HALL_BUF_SAMPLES` — never index OOB. `no_std`, pure `f32`, deterministic (LFO is a
phase accumulator; no RNG); kernel owns no delay storage.

**Stability:** `H` orthonormal ⇒ `‖H·sd‖ = ‖sd‖`; the damping one-pole is passive
(gain ≤ 1); the feedback scales by `g < 1` ⇒ the state map is a contraction
(spectral radius `g`) ⇒ BIBO. Bounded input → bounded output, independent of the
line lengths.

---

## 3. Graph & Wren surface

**`deluge-audio-graph`** (mirror `Kind::Room`):
- `enum Kind`: add `Hall`. `enum State`: add `Hall(Fdn8)`.
- `Node::new`: `Kind::Hall => State::Hall(Fdn8::new())`. `out_width`: add
  `Kind::Hall` to the width-2 arm.
- Render arm (like `Kind::Room`): `port_pair`; bound region `≥ HALL_BUF_SAMPLES` →
  `f.process(ins[0], dt, buf, out_l, out_r)`; else dry passthrough both ports.
- `set_param`: `0 => set_mix, 1 => set_damp, 2 => set_size, 3 => set_width`.

**`deluge-wren-core`** (minimal — reuses the Room setters):
- `node_hall_impl`: `alloc_buffer(HALL_BUF_SAMPLES)`, `new_pooled_node(id,
  Kind::Hall, handle, input)`, set params 2=size / 1=damp / 0=mix from args,
  `return_node_w(vm, id, 2)`. Unbound → dry passthrough.
- Registered in both binding tables + `class Hall { static new(input, size, damp,
  mix) { Node.hall_(input, size, damp, mix) } }` in `prelude.wren`.
- **No new setters** — `mix=`/`damp=`/`size=`/`spread=` (from Room/Delay) already
  cover params 0/1/2/3.

`HALL_BUF_SAMPLES` (one pool alloc) fits the wren test engine pool alongside the
other effects.

---

## 4. QA acceptance & testing

**Kernel (`deluge-dsp-kernels`):**
- **Layout sum.** A `hall_layout_sums_to_buf_samples` test pins `HALL_BUF_SAMPLES
  == Σ(base_len[k] + MOD_MARGIN)`.
- **Impulse → dense decaying tail.** Impulse in (`mix=1`): non-silent well after
  the impulse, energy in a late window < an early window; bounded.
- **Size lengthens the tail.** Higher size → slower decay (more late-window energy).
- **Damping darkens the tail.** Higher damp → less HF (successive-diff proxy) in a
  late window.
- **Stereo decorrelation.** `out_l != out_r` (even/odd tap split + per-line LFO).
- **Mix balance.** `mix=0` → output ≈ dry input.
- **Boundedness (P0 gate).** Proptest: `size/damp/width/mix ∈ [0,1]`, bounded
  input → finite and within a measured bound (orthonormal `H` × `g<1` ⇒ stable);
  `HALL_BUF_SAMPLES`-long buffer.
- **`fwht8` is orthonormal.** A unit-vector round-trip / energy-preservation check
  (`‖fwht8(v)‖ ≈ ‖v‖`), and a known small case (e.g. all-ones → first bin).
- **Graceful:** too-short / absent buffer → dry passthrough, never panic.

**Graph/Wren:**
- `Kind::Hall` node with a bound buffer renders bounded, non-silent, stereo (l ≠
  r); no buffer → dry passthrough both ports; never panics.
- `Hall.new(...)` allocates+binds + emits NewNode + BindTable + the SetParams,
  returns width-2; `Out.patch(Hall.new(...))` emits the two stereo side-writes;
  the reused `size=`/`damp=`/`spread=`/`mix=` setters update the right params.

**Determinism:** no RNG/time; reproducible; proptests seeded.

---

## 5. Deferred / follow-ups

- **Ef-3c Plate (Dattorro)** — the last reverb; reuses `Comb`/`Allpass`/`DelayLine`
  + the partitioned-buffer pattern.
- **Sample-rate-scaled tunings**, **pre-delay**, **early reflections**,
  **per-line diffusion allpasses**, **freeze/infinite**, **tempo-sync** — later.
- **Rename** `TableSrc`/`bind_table` → `PooledBuf`/`bind_buffer` (five effects now
  reuse it) — mechanical cleanup.
