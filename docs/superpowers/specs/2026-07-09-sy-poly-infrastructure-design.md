# Sy-1: Polyphony infrastructure + VoiceSum — design & spec

The first sub-project of the **Sy (synth/voice) suite**: the foundational
**polyphony plumbing**. It teaches the audio graph to carry `VOICES` parallel
voice-lanes through a *poly region*, collapse them to mono at an explicit
**`VoiceSum`** boundary, and proves the machinery end-to-end with a poly
oscillator fed per-voice pitch. No note handling, allocation, or Wren surface
yet — this is pure infrastructure the rest of the suite builds on.

> **Status:** design proposal. Depends on the merged audio graph
> (`Engine`, the output arena, `out_width`/`out_base`, `Input`, `OutView`,
> `process_resolved`) and the `math.rs` `f32x8` SIMD pattern. MIT/Apache.
> `no_std`, no-heap, device-first (Cortex-A9, VFPv3 + NEON).

**Sy roadmap:** **Sy-1 poly infra + VoiceSum (this)** → Sy-2 per-voice gate +
poly envelope/filter → Sy-3 voice allocation & note handling → Sy-4 Wren
`Synth`/`Voices` surface + MIDI → Sy-5 (deferred) expressiveness.

---

## 1. Goals & non-goals

**Goals**
- **`VOICES = 8`** — a `deluge-audio-graph` crate const. Fixed at compile time
  (no heap, SIMD-friendly). Justified against the Cortex-A9 NEON
  microarchitecture (see §2.1), and left a `const` so it can be re-tuned by
  benchmark later.
- **Poly node** — a node whose output is `VOICES` parallel lanes instead of 1
  (mono) or 2 (stereo). It reserves `VOICES` contiguous arena row-slots (reusing
  the existing `out_width`/`out_base` budgeting) and interprets that region as a
  **voice-interleaved tile** (§2.2).
- **Poly-edge input resolution** — a node may declare it consumes a poly input
  (`VOICES` lanes from a poly source); the engine resolves that edge into a
  voice-interleaved `poly_scratch` tile, alongside the existing mono control
  inputs.
- **`PolyCtrl`** — a poly *source*: `VOICES` lanes, lane `v` a settable scalar
  (`set_param(v, x)`). No input. The per-voice pitch feed for Sy-1 and the
  **allocator's write-target in Sy-3**.
- **`PolyOsc`** — a poly oscillator: poly pitch input (`VOICES` Hz lanes) →
  `VOICES` audio lanes. SoA per-voice phase, voice loop as the vectorizable
  inner dimension (`f32x8`, `math.rs` pattern).
- **`VoiceSum`** — the poly→mono collapse: poly input (`VOICES` lanes) → one mono
  output = per-sample sum across voices.
- **QA-proven:** `PolyCtrl` emits its 8 set values; `PolyOsc` renders 8
  independent partials from its pitch lanes; `VoiceSum` equals the arithmetic
  per-sample sum; an engine render of `PolyCtrl → PolyOsc → VoiceSum → out`
  produces 8 distinct partials, bounded and finite; poly `out_width == VOICES`
  and slot budgeting is correct.

**Non-goals (deferred)**
- **Per-voice gate/trigger, poly envelope, poly filter** — Sy-2.
- **Voice allocation, note-on/off, voice stealing, note→pitch (`mtof`)** — Sy-3.
- **Wren `Synth`/`Voices` authoring surface + MIDI binding** — Sy-4 (Sy-1 is
  graph/engine + kernel, Rust-tested only).
- **Poly × stereo** (a poly node that is also stereo, `VOICES × 2` lanes) — a
  poly node here is mono-per-voice. Later, if needed.
- **Velocity / portamento / unison / MPE** — Sy-5.

---

## 2. Poly data model

### 2.1 `VOICES = 8` and the NEON rationale (why interleaved matters)

`VOICES = 8` is chosen for the Cortex-A9 (RZ/A1L, VFPv3 + NEON). The A9 NEON
datapath is 64-bit — a 128-bit (4×f32, Q) op has *throughput 2* — and NEON FP
result latency is high: **VADD/VMUL 5 cycles, VMLA 9 cycles** at 1 result/cycle
(ARM Cortex-A9 NEON MPE TRM ddi0409 Table 3.8; FPU TRM ddi0408 Table 2.1). The
kernels that benefit from voice-batching are the **recurrent** ones (oscillator
phase accumulators, filters), where each sample depends on the previous — the
only independent parallelism is *across voices*. 4 lanes cannot cover the 5–9
cycle latency (the pipeline stalls on its own recurrence); 16 lanes spill the
16 Q-registers on multi-state kernels; **8 is the sweet spot**. This only pays
off if the *voice* axis is the SIMD axis — hence §2.2.

### 2.2 Voice-interleaved tiles (sample-major)

A poly port is a **voice-interleaved** buffer of `VOICES * BLOCK` f32, laid out
**sample-major**: `tile[i * VOICES + v]` is voice `v` at sample `i`. So the 8
voices at sample `i` are contiguous — `f32x8::from_slice(&tile[i*VOICES..])`
loads all voices for one SIMD step. (A lane-major `[VOICES][BLOCK]` layout would
strand the across-voices SIMD and negate §2.1.)

Storage reuses the existing arena: a poly node reserves `VOICES` contiguous
row-slots (`out_base .. out_base + VOICES`), and that `VOICES * BLOCK`
contiguous region *is* the tile — `arr[base..base+VOICES].as_flattened()`.
`out_width(poly_kind) == VOICES`, so the engine's existing slot allocator and
`OUTS` budgeting handle it unchanged; a patch's `OUTS` const must be sized for
`Σ out_width` (each poly node costs 8 slots).

Kernels take flat tiles (`&[f32]` / `&mut [f32]` of length `VOICES * BLOCK`),
which are both SIMD-friendly and trivially unit-testable without the engine.

---

## 3. Kernel DSP (`deluge-dsp-kernels/src/poly.rs`, new module)

```rust
use crate::In; // (control-rate mono inputs, if any)

pub const VOICES: usize = 8; // re-exported by deluge-audio-graph as the graph const

/// A per-voice settable scalar source: lane `v` outputs `values[v]`. The macro
/// / allocator write-target. Output tile is voice-interleaved `VOICES*BLOCK`.
#[derive(Clone, Copy)]
pub struct PolyCtrl {
    values: [f32; VOICES],
}
impl PolyCtrl {
    pub fn new() -> PolyCtrl { PolyCtrl { values: [0.0; VOICES] } }
    pub fn set_voice(&mut self, v: usize, x: f32) { if v < VOICES { self.values[v] = x; } }
    /// `out` is voice-interleaved, length `VOICES * n_samples`.
    pub fn process(&mut self, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            for v in 0..VOICES { out[i * VOICES + v] = self.values[v]; }
        }
    }
}
// + Default

/// A poly oscillator. `pitch` is a voice-interleaved tile of per-voice Hz;
/// writes a voice-interleaved audio tile. SoA phase; the voice loop is the
/// inner (vectorizable) dimension.
#[derive(Clone, Copy)]
pub struct PolyOsc {
    phase: [f32; VOICES], // [0,1) per voice
}
impl PolyOsc {
    pub fn new() -> PolyOsc { PolyOsc { phase: [0.0; VOICES] } }
    /// `pitch` and `out` are voice-interleaved, length `VOICES * n_samples`.
    pub fn process(&mut self, pitch: &[f32], dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            for v in 0..VOICES {
                let f = pitch[i * VOICES + v];
                let mut p = self.phase[v] + f * dt;
                p -= crate::floorf(p); // wrap [0,1)
                self.phase[v] = p;
                out[i * VOICES + v] = crate::fast_sin(p); // raw shape; band-limiting is later
            }
        }
    }
}
// + Default
```

- The inner `for v in 0..VOICES` loops over the interleaved lane block; behind
  the `simd` feature it lowers to one `f32x8` step (scalar fallback otherwise) —
  identical structure to `math.rs`. (Sy-1 may ship the scalar form; the layout
  is what makes the `f32x8` rewrite a drop-in, and that rewrite/bench is a
  tracked follow-up.)
- `PolyOsc` uses `fast_sin` (a clean sine) for Sy-1; the poly counterparts of
  the band-limited `Osc` shapes are a later concern.

`no_std`, pure `f32`, deterministic, no heap.

**`VoiceSum`** is a stateless graph op (no kernel struct): for each sample `i`,
`out[i] = Σ_v tile[i*VOICES + v]`. Implemented as a free function in the graph
render arm (like `math::mul`), SIMD-reducible (`f32x8` horizontal sum).

---

## 4. Graph & engine integration (`deluge-audio-graph`)

### 4.1 Const + node kinds
- `pub const VOICES: usize = 8;` (re-exported; equals `deluge_dsp_kernels::poly::VOICES`).
- `enum Kind`: add `PolyCtrl`, `PolyOsc`, `VoiceSum`.
- `enum State`: `PolyCtrl(PolyCtrl)`, `PolyOsc(PolyOsc)`; `VoiceSum` is
  `State::Stateless`.

### 4.2 Widths and the poly-input flag
- `out_width(kind)`: `PolyCtrl`/`PolyOsc` → `VOICES`; `VoiceSum` → `1`; others
  unchanged. (Poly kinds join the width arm; they are NOT in the width-2 stereo
  arm.)
- New `Node::has_poly_in(kind) -> bool`: `true` for `PolyOsc` (pitch) and
  `VoiceSum` (the lanes it collapses); `false` otherwise. The poly input is
  always input **port 0**, referencing a poly source (`out_width == VOICES`).
- `set_param`: `State::PolyCtrl(c) => c.set_voice(param as usize, value)` (param
  index = voice `v`). `PolyOsc`/`VoiceSum` have no params.

### 4.3 Engine render changes (`engine.rs::render_block`)
- Add a per-iteration `poly_scratch: [f32; VOICES * BLOCK]` (voice-interleaved).
- When `has_poly_in(kind)`: resolve input 0 as a poly edge — look up the source's
  `out_base`; if `sbase + VOICES <= OUTS`, copy the source tile
  (`arr[sbase..sbase+VOICES]` flattened) into `poly_scratch`; else fill silence
  (never panic, matching the existing dangling-ref guard). Mono control inputs
  (ports 1‑2) still resolve into the normal 3-row `scratch`.
- Output: for a poly node (`width == VOICES`), the writable region is the
  flattened `arr[base..base+VOICES]` tile. `OutView` is extended so a poly node
  writes its tile (see 4.4).
- Dispatch: `process_resolved` gains the poly input tile and a poly-capable
  output. Render arms:
  - `PolyCtrl` → `c.process(out_tile)`.
  - `PolyOsc` → `o.process(poly_scratch, dt, out_tile)`.
  - `VoiceSum` → `voice_sum(poly_scratch, out.port(0))` (interleaved → mono).

### 4.4 `OutView`
`OutView.ports` is currently `[Option<&mut [f32]>; 2]` (mono/stereo). Extend the
output path to expose a poly node's flat tile — either widen the port array to
`VOICES` or add a `poly_tile(&mut self) -> &mut [f32]` accessor built from the
flattened `arr[base..base+VOICES]`. Mono/stereo `port(p)`/`port_pair` unchanged.
(Implementation detail; the plan pins the exact shape. Keep the existing
copy-before-mutate borrow discipline — the poly tile is written after all reads
are copied into scratch/poly_scratch.)

---

## 5. QA acceptance & testing (Rust; kernel + graph/engine)

**Kernel (`deluge-dsp-kernels`):**
- **`PolyCtrl`** — after `set_voice(v, x)` for a few voices, `process` fills the
  interleaved tile so `tile[i*VOICES+v] == values[v]` for all `i`.
- **`PolyOsc`** — feed a constant pitch tile with 8 distinct frequencies over a
  long block; each voice lane is a sine at its own frequency (check per-voice
  zero-crossing rate / period), lanes are independent (changing one pitch does
  not affect another), output ∈ [−1, 1], finite.
- **`voice_sum`** — for a known interleaved tile, output equals the per-sample
  arithmetic sum across the 8 lanes.
- **Boundedness proptest** — random pitch tiles (0–8 kHz) → `PolyOsc` finite,
  ∈ [−1,1]; `voice_sum` finite.

**Graph / engine:**
- `Node::out_width(Kind::PolyOsc) == VOICES` (and `PolyCtrl`); `VoiceSum` == 1.
- A poly node reserves `VOICES` contiguous slots; slot budgeting/`out_base`
  advances by `VOICES` (assert the next node's base).
- `has_poly_in` true for `PolyOsc`/`VoiceSum`, false for `PolyCtrl`.
- **End-to-end engine render:** build `PolyCtrl → PolyOsc → VoiceSum → out`
  with 8 distinct per-voice frequencies via `set_param`; render a block; the
  summed output is bounded/finite and its spectrum contains the 8 distinct
  partials (or, more simply, equals the sum of 8 independently-rendered
  reference sines). A poly edge with a dangling/short source resolves to silence
  (no panic).

**Determinism:** no RNG; reproducible; proptests seeded.

---

## 6. Deferred / follow-ups

- **`f32x8` vectorization of the poly kernels** behind the `simd` feature +
  a QEMU-Cortex-A9 Criterion bench (the `deluge-fft` harness) to A/B
  `VOICES ∈ {4, 8, 16}` on a recurrent poly kernel — confirm §2.1 empirically.
- **Sy-2:** per-voice `gate(v, on)`/`trigger(v)`, poly `PolyAr` envelope, poly
  filter (`PolySvf`) — the rest of a voice; reuses the poly-edge seam.
- **Sy-3:** voice allocation (note-on → free/steal a lane, write `PolyCtrl`
  pitch + gate; note-off → release), stealing policy, `mtof` per lane.
- **Sy-4:** Wren `Synth`/`Voices` authoring + `Midi.onNoteOn/off` binding.
- **Poly × stereo**, band-limited poly oscillator shapes, per-voice pan — later.
