# Ef-3a: Room reverb (Freeverb) — design & spec

The first of three reverb sub-projects (**Ef-3 Reverb** = Ef-3a Room / Ef-3b Hall
/ Ef-3c Plate, each a distinct engine). Ef-3a builds a **Schroeder-Moorer
(freeverb-topology) room reverb** — implemented from the *public algorithm
structure*, NOT ported from the GPL freeverb source — and stands up the shared
reverb foundation the other two reuse: the `Comb`/`Allpass` primitives and the
**single-big-pooled-buffer partitioning** (one region carved into all internal
delay lines), over the merged pooled-buffer seam + width-2 stereo routing.

> **Status:** design proposal. Depends on: the pooled-region graph path
> (`TableSrc::Pooled` + `process_resolved`'s `&mut` region + `Host::alloc_buffer`),
> `OutView::port_pair`, width-aware `Out.patch`, `new_pooled_node` (all merged).
> No new graph infra. MIT/Apache — the Schroeder-Moorer comb+allpass structure
> and its standard delay tunings are public; do NOT read/port the GPL `freeverb`
> C++.

**Reverb roadmap:** **Ef-3a Room (Freeverb, this)** → Ef-3b Hall (FDN) → Ef-3c
Plate (Dattorro). Ef-3a establishes `Comb`/`Allpass` + buffer partitioning; the
others reuse them.

---

## 1. Goals & non-goals

**Goals**
- **`Comb` primitive** — a damped-feedback comb (lowpass in the feedback for HF
  absorption): `out = buf[c]; damp_z = out·(1−d) + damp_z·d; buf[c] = x +
  damp_z·fb; advance c`. Fixed integer delay = the buffer length.
- **`Allpass` primitive** — a Schroeder allpass diffuser (fixed feedback 0.5):
  `bufout = buf[c]; out = -x + bufout; buf[c] = x + bufout·0.5; advance c`. (This
  is the diffuser the Ef-1 "allpass stub" was reserved for — realized as a
  dedicated struct.)
- **`Freeverb` room engine** — per channel: **8 parallel combs** summed → **4
  series allpasses**; right channel delays offset by a **stereospread** (+23
  samples) for L/R decorrelation. Mono input → stereo (L/R) output.
- **One partitioned pooled buffer.** The reverb node binds ONE pool region sized
  for all 24 internal lines; the kernel carves it via a const layout table
  `[(offset, len); 24]` and each `tick` indexes `buf[off + cursor]` (sequential,
  so no simultaneous disjoint borrows). `REVERB_BUF_SAMPLES` = Σ line lengths.
- **Graph node** `Kind::Room` (width-2, stereo), routed to L/R by the merged
  width-aware `Out.patch`. **Graceful dry passthrough** (both ports = input) when
  no buffer bound / region too small; never panic.
- **QA-proven:** impulse → a decaying diffuse tail (RT60 grows with roomsize);
  damping darkens the tail; stereo L ≠ R; mix balance; boundedness.

**Non-goals (deferred)**
- **Hall (FDN)** and **Plate (Dattorro)** — Ef-3b / Ef-3c (reuse this foundation).
- **Pre-delay, modulation, freeze, sample-rate-scaled tunings** — later (delay
  lengths are fixed sample counts, 44.1 kHz-nominal; character shifts slightly at
  48 kHz — acceptable, documented).
- **Fractional/modulated allpass on `DelayLine`** — not needed here (fixed
  integer delays); revisit if Ef-3c Plate wants it.

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/reverb.rs`, new module)

Standard freeverb tunings (samples @ 44.1 kHz): combs `[1116, 1188, 1277, 1356,
1422, 1491, 1557, 1617]`, allpasses `[556, 441, 341, 225]`, stereospread `23`,
input gain `0.015`, allpass feedback `0.5`, `roomsize→feedback = size·0.28 + 0.7`
(0.7–0.98), `damp→coeff = damp·0.4`.

```rust
/// Damped-feedback comb (lowpass in the feedback path). Fixed integer delay =
/// the buffer slice length; holds a cursor + one-pole damp state.
#[derive(Clone, Copy)]
pub struct Comb { c: usize, damp_z: f32 }
impl Comb {
    pub fn new() -> Comb;
    /// `off`/`len` locate this comb's slice in the shared buffer.
    pub fn tick(&mut self, buf: &mut [f32], off: usize, len: usize,
                x: f32, feedback: f32, damp: f32) -> f32;
}

/// Schroeder allpass diffuser (fixed feedback 0.5). Cursor only.
#[derive(Clone, Copy)]
pub struct Allpass { c: usize }
impl Allpass {
    pub fn new() -> Allpass;
    pub fn tick(&mut self, buf: &mut [f32], off: usize, len: usize, x: f32) -> f32;
}

/// Freeverb room reverb: 8 combs + 4 allpasses per channel over a shared
/// partitioned buffer; mono in → stereo out. `no_std`, no heap.
#[derive(Clone, Copy)]
pub struct Freeverb {
    combs_l: [Comb; 8], combs_r: [Comb; 8],
    aps_l: [Allpass; 4], aps_r: [Allpass; 4],
    roomsize: f32, damp: f32, width: f32, mix: f32, // params
}
impl Freeverb {
    pub fn new() -> Freeverb;                 // roomsize 0.5, damp 0.5, width 1, mix 0.5
    pub fn set_mix(&mut self, v: f32);        // param 0, clamp [0,1]
    pub fn set_damp(&mut self, v: f32);       // param 1, clamp [0,1]
    pub fn set_roomsize(&mut self, v: f32);   // param 2, clamp [0,1]
    pub fn set_width(&mut self, v: f32);      // param 3, clamp [0,1]
    /// `buf` = the partitioned pool region (`≥ REVERB_BUF_SAMPLES`).
    pub fn process(&mut self, input: In, dt: f32, buf: &mut [f32],
                   out_l: &mut [f32], out_r: &mut [f32]);
}

/// Total samples the shared buffer must hold (Σ of all 24 line lengths).
pub const REVERB_BUF_SAMPLES: usize = 25_450;
```

**Layout** (const `[(offset, len); 24]`, cumulative): 8 L combs
(1116…1617), 8 R combs (+23 each), 4 L allpasses (556/441/341/225), 4 R allpasses
(+23 each). `REVERB_BUF_SAMPLES` = the final `offset + len` = 25 450.

**Per sample:** `x = input.at(i)`; `in_g = x · 0.015`; `feedback =
roomsize·0.28 + 0.7`; `dc = damp·0.4`.
`ol = Σ combs_l[k].tick(buf, …, in_g, feedback, dc)`; then series
`ol = aps_l[k].tick(buf, …, ol)`. Same for `or` with the R lines.
`wet1 = mix·(width/2 + 0.5); wet2 = mix·((1−width)/2); dry = 1 − mix`.
`out_l[i] = x·dry + ol·wet1 + or·wet2; out_r[i] = x·dry + or·wet1 + ol·wet2`.

`no_std`, pure `f32`, deterministic, no heap; the kernel owns no delay storage
(buffer borrowed). Comb feedback ≤ 0.98 < 1 with the 0.015 input scaling ⇒ BIBO
stable and well-bounded.

---

## 3. Graph & Wren surface

**`deluge-audio-graph`** (mirrors `Kind::Chorus`, a width-2 pooled effect):
- `enum Kind`: add `Room`. `enum State`: add `Room(Freeverb)`.
- `Node::new`: `Kind::Room => State::Room(Freeverb::new())`.
- `out_width`: add `Kind::Room` to the width-2 arm.
- Render arm (like `Kind::Chorus`): `let (out_l, out_r) = outs.port_pair();` bound
  region (≥ `REVERB_BUF_SAMPLES`) → `fv.process(ins[0], dt, buf, out_l, out_r)`;
  else dry passthrough to both ports.
- `set_param`: `0 => set_mix, 1 => set_damp, 2 => set_roomsize, 3 => set_width`.

**`deluge-wren-core`:**
- `node_room_impl`: `alloc_buffer(REVERB_BUF_SAMPLES)`, `new_pooled_node(id,
  Kind::Room, handle, input)`, set params 2=roomsize / 1=damp / 0=mix from the
  args, `return_node_w(vm, id, 2)` (width-2 stereo). Unbound (no pool / too-small)
  → dry passthrough.
- Setters: `mix=` (reused → param 0), `damp=` (reused from Ef-1 Delay → param 1;
  same "damping" concept/index), **`size=`** (new → param 2), **`spread=`** (new
  → param 3 stereo width — NOT `width=`, which is the Osc's PWM *port* setter).
- Registered in both binding tables + `class Room` in `prelude.wren`:
  `Room.new(input, roomsize, damp, mix)`.

`REVERB_BUF_SAMPLES` (25 450) is one pool alloc; the wren test engine pool
(`PCAP 90112`, `CHUNK 2048`) holds it (13 chunks) with room to spare.

---

## 4. QA acceptance & testing

**Kernel (`deluge-dsp-kernels`, `process` takes `&mut [f32]` bufs):**
- **Impulse → decaying diffuse tail.** An impulse in (`mix=1`): output is a dense,
  decaying tail — energy in a late window < an early window, and non-silent well
  after the impulse (the combs recirculate). Bounded.
- **Roomsize lengthens the tail.** Higher roomsize → slower decay (more late-window
  energy / longer RT60) than low roomsize.
- **Damping darkens the tail.** Higher damp → less HF energy in a late window
  (successive-sample-difference proxy) than low damp.
- **Stereo decorrelation.** `out_l != out_r` (the +23-sample R offset).
- **Mix balance.** `mix=0` → output ≈ dry input; `mix=1` → wet only.
- **Boundedness (P0 gate).** Proptest: `roomsize/damp/width/mix ∈ [0,1]`, bounded
  input → finite and within a measured bound; `REVERB_BUF_SAMPLES`-long buffer.
- **Graceful:** a too-short / absent buffer → dry passthrough, never panic.

**Graph/Wren:**
- A `Kind::Room` node with a bound buffer renders bounded, non-silent, stereo
  (l/r finite, l ≠ r); no buffer → dry passthrough both ports; never panics.
- `Room.new(...)` allocates+binds the buffer and emits NewNode + BindTable + the
  SetParams; returns width-2; `Out.patch(Room.new(...))` emits the two stereo
  side-writes. `size=`/`damp=`/`spread=`/`mix=` update the right params;
  pool-exhaustion → dry passthrough.

**Determinism:** no RNG/time; reproducible; proptests seeded.

---

## 5. Deferred / follow-ups

- **Ef-3b Hall (FDN)** and **Ef-3c Plate (Dattorro)** — reuse `Comb`/`Allpass` +
  the partitioned-buffer pattern.
- **Sample-rate-scaled tunings** (currently fixed 44.1 kHz sample counts),
  **pre-delay**, **modulation/chorused tail**, **freeze/infinite**.
- **Rename** `TableSrc`/`bind_table` → generic `PooledBuf`/`bind_buffer` (now four
  effects reuse it) — mechanical cleanup.
