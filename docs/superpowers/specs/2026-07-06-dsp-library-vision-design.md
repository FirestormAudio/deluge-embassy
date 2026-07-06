# DSP library for Wren — fully-fleshed-out vision & roadmap

Turn the current *basic* Rust DSP library (the prototype audio graph in
`deluge-wren-core`) into a **device-first, SuperCollider-like audio-scripting
toolkit** callable from Wren: a broad, composable vocabulary of oscillators,
filters, envelopes, effects and samples, driven either as a low-level patch graph
or as high-level `SynthDef`/voice instruments — all running on the real Deluge
(ARM Cortex-A9, `no_std`, sharing CPU with the rest of the firmware) and mirrored
bit-for-bit in the web simulator.

> **Status:** umbrella vision. This is the *cluster* spec. It fixes the shared
> architecture, the crate structure, the decomposition into sub-projects, and the
> cross-cutting rules. Each sub-project below gets its **own** design → plan →
> implementation cycle (the way sub-project A — the wasm brain — was built). No
> code is written from this doc directly except as it seeds the P0 spec.

This is a **new crate cluster**, not an extension of `deluge-wren-core`. The
prototype's audio graph is treated as a reference that proved the
control→audio seam works; the fleshed-out system is greenfield and eventually
supersedes it.

---

## 1. Goals & non-goals

**Goals**

- A **broad audio vocabulary** — anti-aliased oscillators, a real filter suite,
  rich modulation, buffer-based effects, and sample playback — usable from Wren.
- **Two authoring layers over one substrate:** a low-level imperative node graph,
  and a high-level `SynthDef`/voice layer (templates, polyphony, MIDI voice
  allocation) built as *recorded* low-layer operations.
- **Device-first.** Everything must run within a bounded per-block CPU budget on
  the Deluge, `no_std`, with no heap allocation or locks on the audio path.
- **One portable core**, so the device and the web simulator render identically,
  pinned by golden audio vectors.
- **Cashes in `armv7-dsp-intrinsics`** — DSP kernels are block-oriented and
  SIMD-able through the existing NEON crate.

**Non-goals (explicitly out, or deferred to their own future work)**

- FFT/spectral processing, physical modelling, and convolution reverb — the
  block architecture leaves room for them, but they are not in this vision's
  scope.
- A visual patching UI. Authoring is Wren code.
- Replacing the Deluge's own native synth engine. This is a *scripting* engine
  that runs alongside the firmware, reached through Wren.
- Dynamic, unbounded polyphony. Voice count is a fixed, compile-time arena.

---

## 2. Architecture

A **three-layer stack**, all sharing one portable core so device and sim stay
identical:

```
Wren authoring
 ├─ High layer  SynthDef (template) → Synth/Voice instances → MIDI voice allocator
 └─ Low layer   imperative node graph: Osc/Filter/Env/Math… patched to buses
        │ emits Cmds (control-rate)
        ▼
Rust portable core
 ├─ Block graph   node arena w/ free-list · audio-rate vs control-rate signals · stereo buses
 ├─ UGen library  oscillators · filters · envelopes · effects · samples · dynamics
 └─ Cmd transport (Host trait) → firmware ring · web direct-apply
        ▼
DSP kernels  (armv7-dsp-intrinsics / NEON, with portable fallbacks)
```

### 2.1 The block engine

Replaces today's per-sample `render_frame`:

- **Block size N** (~16–32 samples, start at 32). Each audio-rate node writes an
  `[f32; N]` output buffer; nodes evaluate in creation/topological order into
  their buffers — the same eval order as the prototype, just N samples at a time.
- **Two signal rates.** A node output is *audio-rate* (a full block) or
  *control-rate* (one value per block — LFOs, envelopes-as-modulation, knob
  values). Every param slot accepts a **constant**, a **control-rate source**
  (broadcast across the block), or an **audio-rate source** (audio-rate
  modulation). The engine handles rate-matching.
- **Stereo at buses, not nodes.** Node outputs stay mono; an explicit `Pan` node
  writes into a stereo **bus**, and a master bus sums to the output block. The
  current mono path becomes "one node → master." This keeps the node vocabulary
  simple while giving real stereo.
- **Node lifecycle via free-list.** The prototype's monotonic id allocator is
  replaced by a free-list over the fixed `MAX_NODES` arena, so subgraphs (voices,
  one-shot effects) can be spawned and **freed** — the precondition for
  polyphony. A full-graph reset still exists but is no longer the only way to
  reclaim nodes.
- **Buffer pool.** A fixed arena (sized from the Deluge's SDRAM budget) backs
  delay lines, reverb, and sample buffers — handed out to and returned with the
  nodes that own them.

Feedback loops carry one block of latency (acceptable), or are expressed via an
explicit feedback node when sample-tight feedback is needed.

### 2.2 Crate structure

New crates (existing `armv7-dsp-intrinsics` unchanged at the bottom):

```
armv7-dsp-intrinsics   (exists)  NEON intrinsics + portable fallbacks
        ▲
deluge-dsp-kernels     (new)     block-oriented DSP math: oscillator/filter/env/
                                  effect cores. no_std, no graph knowledge. Pure,
                                  property-testable functions over buffers.
        ▲
deluge-audio-graph     (new)     the block engine: node arena + free-list, buffer
                                  pool, audio/control rate model, stereo buses,
                                  Cmd vocabulary, Host transport trait.
        ▲
deluge-audio-wren      (new)     Wren bindings — low imperative layer + high
                                  SynthDef/voice layer. Supersedes the prototype
                                  audio classes in deluge-wren-core.
```

Hosts (the firmware audio task, the web/wasm build) wire `deluge-audio-graph` to
their transports exactly as the prototype does today — that control→audio seam is
the one thing worth carrying over verbatim.

---

## 3. Sub-projects & build order

Each row is its own future design → plan → implementation cycle.

| # | Sub-project | Delivers | Depends on |
|---|---|---|---|
| **P0** | **Kernels + graph foundation** | `deluge-dsp-kernels` + `deluge-audio-graph`: block render, free-list lifecycle, buffer pool, rate model, stereo buses, host transport. Ports the ~10 prototype primitives as its validation set. | intrinsics |
| **P1** | **Low-level Wren layer** | `deluge-audio-wren` imperative API: build/patch/spawn/free nodes, buses. Reaches parity with today's `Osc/Env/Out`, then exceeds it. | P0 |
| **QA** | **Validation & profiling harness** | DSP null-tests, frequency-response checks, NaN/denormal guards, per-block CPU-budget profiler (device + sim), golden audio vectors. Stood up early, grows with the library. | P0 |
| **Osc** | **Oscillator suite** | Anti-aliased osc (PolyBLEP/wavetable), wavetable, FM operators, hard sync, sub, noise variants. | P0 |
| **Fi** | **Filter suite** | SVF (LP/HP/BP/notch), ladder/Moog, one-pole, comb/allpass, drive, audio-rate-modulatable. | P0 |
| **Mod** | **Modulation** | LFOs, ADSR/DAHDSR, sample-&-hold, slew/lag, control-rate routing. | P0 |
| **Ef** | **Effects** | Delay, chorus/flanger, reverb, waveshaper/distortion, bitcrush, dynamics (comp/limiter), EQ. | P0 (buffer pool) |
| **Sa** | **Samples** | Sample buffers, one-shot/loop players, SD streaming, basic granular. | P0 + SD |
| **Sy** | **SynthDef & voice layer** | Templates with named params, Synth instances, MIDI voice allocation/stealing, per-voice params — the "high layer." | P0 lifecycle, P1; richer with Osc/Fi/Mod |
| **IO** | **I/O & routing** | Audio input processing, mixer/bus expansion, master chain, multiple outputs. | P0 |

**Build order.** `P0` is a hard gate — nothing else starts until it is solid.
Then `P1` + `QA` stand up together. The four vocabulary suites (`Osc`, `Fi`,
`Mod`, `Ef`) then proceed largely in parallel on the stable substrate. `Sy` (the
voice layer) follows once lifecycle is proven and enough vocabulary exists to make
instruments interesting. `Sa` and `IO` fold in as buffer/SD and routing needs
mature.

---

## 4. Cross-cutting decisions

These bind every sub-project; each later spec inherits them.

**Memory & real-time discipline.**
- Everything is a **fixed, compile-time-sized arena**: `MAX_NODES`, the buffer
  pool (delay/reverb/sample RAM from the SDRAM budget), and the voice count. No
  heap allocation and no locks on the audio path — control→audio stays the
  lock-free ring the prototype already uses.
- Bounded work per block: no unbounded loops in `render`, denormals flushed,
  outputs clamped. The **QA profiler enforces a per-block CPU budget as a merge
  gate** — a UGen that blows the budget does not land.

**Rate model.** Each param slot resolves to {constant, control-rate source (one
value/block, broadcast), audio-rate source (full block)}. Block size N is a single
tunable (start ~32), trading modulation latency against dispatch savings — fixed
in P0, revisited only with profiler data.

**SynthDef mechanics (the device-friendly bit).** A `SynthDef` is a **recorded,
parameterized Cmd list**, not a live graph. Instantiating a voice = replay that
list into freed arena slots with named params bound — cheap, allocation-free, and
identical on device and sim. The voice allocator owns note→voice mapping and a
configurable stealing policy (oldest / quietest). This is why "both layers" is
clean: the high layer is just *recorded* low-layer operations.

**Migration off the prototype.** `deluge-wren-core`'s `Osc/Env/Out` classes become
deprecated shims or a clean break — decided in **P1**, not here. The Wren *surface*
stays source-compatible where that is free, so a script like
`wren-firmware/examples/midi_synth.wren` either still runs or has a one-line port.
The prototype `Engine`/`Cmd`/`audio.rs` are removed once `deluge-audio-graph`
reaches parity.

**Determinism & parity.** One core, both targets; golden audio vectors pin
device ≡ sim (the established pattern); noise/random UGens take explicit seeds so
vectors reproduce.

**Testing strategy per layer.**
- *Kernels* → property tests + **null tests against a reference model** (naïve vs
  SIMD implementations must agree).
- *Graph* → golden audio vectors.
- *Voice layer* → allocation/stealing unit tests.
- *Device* → the per-block CPU-budget profiler gate.

---

## 5. Open questions (resolved as each sub-project is specced)

- Exact `MAX_NODES`, voice count, and buffer-pool size vs. measured SDRAM/CPU
  headroom (sized in P0 with the QA profiler).
- Anti-aliasing strategy for oscillators — PolyBLEP vs. wavetable vs. both
  (decided in `Osc`).
- Reverb topology within the CPU budget — Freeverb-style vs. Dattorro (decided in
  `Ef`).
- Whether `deluge-audio-wren` is a distinct crate or is folded into the firmware's
  binding layer (decided in P1).
- SD streaming cadence and granular design (decided in `Sa`).
