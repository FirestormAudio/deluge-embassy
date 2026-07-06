# P0 — DSP kernels + graph foundation — design & spec

The foundation sub-project of the [DSP library vision](2026-07-06-dsp-library-vision-design.md).
Delivers the two bottom crates of the new cluster — `deluge-dsp-kernels` and
`deluge-audio-graph` — as a portable, `no_std`, allocation-free block-rendering
audio engine with a first-class multi-output node model, buses, node lifecycle,
and a host transport seam. Everything above it (the Wren layer, the vocabulary
suites, the voice layer) builds on the contracts fixed here.

> **Status:** design proposal. P0 is the hard gate of the cluster — no other
> sub-project starts until the contracts below (node/port model, `Input`, `Cmd`,
> `Host`, `GraphConfig`) are implemented and pinned by golden vectors.

This is **greenfield**. The prototype audio graph in `deluge-wren-core`
(`engine.rs`, `audio.rs`) is a *reference* — its control→audio seam and its
topological-eval trick carry over conceptually — but P0 is new crates, not an
edit of it. The prototype is removed once `deluge-audio-graph` reaches parity
(handled in P1).

---

## 1. Goals & non-goals

**Goals**

- A portable `no_std` **block-rendering engine**: nodes process `N` samples at a
  time into pooled output buffers, evaluated in topological order.
- A **first-class multi-output node model** — one compute node publishes several
  named output ports (e.g. an SVF's LP/HP/BP), each independently addressable as
  `(node, port)`.
- **Node lifecycle** via a free-list (spawn *and* free), the precondition for
  polyphony — replacing the prototype's monotonic, never-freed allocator.
- **Stereo buses** carrying homogeneous channel bundles and fan-in (summing),
  with a master bus as the render root.
- A **rate model**: every input resolves to a constant, a control-rate source
  (one value/block, broadcast), or an audio-rate source (a full block).
- **Host-supplied sizing** (arena sizes, block size) via a `GraphConfig` trait
  and a runtime sample rate — no hardcoded budgets.
- A **`Host` transport seam** (the prototype's `audio_cmd`) and a **`Cmd`
  vocabulary** that expresses the whole model.
- `deluge-dsp-kernels`: the DSP math as **pure, self-contained `f32` structs**
  with a `process_block`, testable in isolation, with **no graph knowledge**.
  Data-parallel kernels (math, mixing, gain, constant-freq oscillators) use
  `core::simd` (→ NEON on ARM) behind a `simd` feature with a scalar fallback;
  serial-recurrence kernels (IIR filters, noise, envelopes) stay scalar (their
  SIMD path is cross-voice batching, out of P0). No dependency on
  `armv7-dsp-intrinsics` (that crate is fixed-point).
- A **validation set**: the ~10 prototype primitives re-implemented on this
  substrate, proving the model and seeding the golden vectors.

**Non-goals (deferred to their sub-projects)**

- Any new DSP vocabulary beyond the ported validation set (Osc/Fi/Mod/Ef).
- The Wren bindings and port-naming sugar (`svf.lp`) — P1.
- SynthDef/voice allocation — Sy.
- A production buffer-pool allocator for large delay/reverb/sample state — P0
  ships the pool *interface* + a simple allocator, exercised by one minimal test
  node; the real stress and tuning land in Ef/Sa.
- Multi-output *rate inference* across a graph — P0 uses static per-kind-per-port
  rates, not propagation.

---

## 2. Crate boundary

```
deluge-dsp-kernels     pure DSP: struct + process_block over &[f32]/&mut [f32].
                       No NodeId, no Input, no graph. f32; core::simd (→NEON)
                       for data-parallel kernels behind a `simd` feature (scalar
                       fallback); serial-recurrence kernels stay scalar.
        ▲
deluge-audio-graph     the engine: node arena + free-list, pooled output arena,
                       buses, rate model, Cmd vocabulary, Host trait, GraphConfig.
                       Wraps each kernel struct in an enum variant.
```

A UGen's *math* lives in `deluge-dsp-kernels` (e.g. `struct Svf { z1, z2 } fn
process_block(&mut self, x: &[f32], cutoff: Rate, res: Rate, lp: &mut [f32], hp:
&mut [f32], bp: &mut [f32])`). The *graph node* is the enum variant in
`deluge-audio-graph` that owns that struct plus its `Input` slots and output run,
and adapts the engine's port buffers to the kernel's arguments.

---

## 3. Node & signal model

### 3.1 Representation — hybrid enum

Nodes are a **uniform enum**, one variant per kind, each wrapping its kernel
struct's state and its typed `Input` slots:

```rust
enum Node {
    Sine(Osc), Saw(Osc), Square(Osc), Tri(Osc),
    Noise(Noise), Env(Ar), Lpf(OnePole),
    Mul(Bin), Add(Bin), Sub(Bin),
    Custom(&'static mut dyn Ugen),   // reserved escape hatch (see §3.6)
}
```

Dispatch is a generated `match` (an `enum_dispatch`-style macro turns `impl Ugen
for Svf` into the arm), giving static dispatch, an auditable closed set, and
alloc-free uniform storage — with each UGen still authored as its own testable
struct. Runtime cost is amortized over the block; see the vision doc's analysis.

### 3.2 Output ports — node owns a pooled slot-run

There is one output arena of per-slot block buffers; a node **owns a contiguous
run** of it:

```rust
struct Block([f32; N]);
outs: [Cell<Block>; MAX_OUT_SLOTS]      // interior mutability — see §3.5

struct NodeHdr { kind: Kind, out_base: u16, inputs: /* typed */, /* state in the enum */ }
// width is static:  kind.out_width()  →  Sine=1, Svf=3, …
```

- Each UGen declares `OUT_WIDTH` and the meaning of each port index (SVF: 0=LP,
  1=HP, 2=BP).
- `process` receives `&mut [Block]` of **exactly** this node's slots and fills
  each. A width-1 node writes only `outs[0]`.
- Freeing the node returns its run to the pool — one owner, one lifetime.
- Physical separation from the node struct (rather than inline output) keeps the
  uniform enum small: inline output would charge every node `max_width × [f32;
  N]` (~128 B/output at N=32), wasteful for the width-1 majority.

### 3.3 Wiring — typed inputs, `(node, port)` connections

```rust
enum Input { Const(f32), Node { node: NodeId, port: u8 }, Bus(BusId) }
```

- Inputs are **fields of the UGen struct** (e.g. `Svf { input, cutoff, res }`),
  exposed by index via `fn input_mut(&mut self, port: u8) -> Option<&mut Input>`.
  A node carries exactly as many inputs as it needs — no padded array.
- A connection names a source **port**: reading `Node { node, port }` resolves to
  `outs[nodes[node].out_base + port]`.
- **`Bus` handles fan-in.** Many-to-one summing (mixers, summing voices) is not a
  variadic node input — writers sum into a bus and a consumer reads the bus as one
  input. Node input sets stay small; buses absorb fan-in.

### 3.4 Rate model

- A **port** is audio-rate (a full `[f32; N]` block) or control-rate (one value
  per block). Rate is a **static property of kind+port** in P0 — no cross-graph
  inference. Most ports are audio-rate; modulation sources (LFO, envelope-as-mod)
  declare control-rate outputs.
- Reading an input yields a block (audio-rate source or `Bus`) or a scalar
  (`Const`, or a control-rate source — **broadcast** across the block on read).
- Block size `N` is fixed by `GraphConfig`; sample rate is a runtime value passed
  at engine construction. Kernels are written rate-agnostic (they take `1/sr`).

### 3.5 Eval order & the borrow invariant

Nodes evaluate in **topological order** (a node's inputs are produced by
already-evaluated nodes; a node writes only its own run and reads only others').
That guarantees no *true* read/write aliasing — only the borrow checker objects.

The prototype relied on monotonic ids so memory order == eval order and used
`split_at_mut`. The free-list (§3.6) breaks that equality — a reused slot can sit
anywhere — so P0 keeps an **explicit eval-order list** (topological) and backs the
output arena with **interior mutability** (`Cell<Block>`, or a single documented
`unsafe` justified by the topo invariant). This is the standard audio-graph
approach and stays sound under free/reuse.

### 3.6 Lifecycle — free-list

- `MAX_NODES` node slots and `MAX_OUT_SLOTS` output slots each have a **free-list**.
  The binding owns the `NodeId` free-list (client-side, no round-trip); the engine
  owns the output-slot free-list.
- On `NewNode` the engine allocates a contiguous output-slot run of
  `kind.out_width()` for the node; it and the node slot are freed together on
  `Free`. If the output pool is exhausted the node is inert (renders silence),
  mirroring the prototype's out-of-range no-op — never a panic.
- The **eval-order list** is maintained on create/free so topological order
  survives reuse.
- `Custom(&'static mut dyn Ugen)` is reserved but unused in P0: it is the future
  open-set escape hatch (a host places a UGen in its own static storage and hands
  a reference). Defining the variant now keeps it additive later.

---

## 4. Sizing — `GraphConfig`

One type parameter, associated consts, so every arena is fixed at compile time
with zero hardcoded budget:

```rust
trait GraphConfig {
    const BLOCK: usize;          // N — samples per block
    const MAX_NODES: usize;
    const MAX_OUT_SLOTS: usize;
    const MAX_BUSES: usize;
    const POOL_BYTES: usize;     // persistent buffer pool (§6)
}

struct Engine<C: GraphConfig> { /* arenas sized from C */ }
impl<C: GraphConfig> Engine<C> {
    fn new(sample_rate: f32) -> Self;
    fn apply(&mut self, cmd: Cmd);
    fn render(&mut self, out: &mut [StereoFrame]);   // one block
}
```

A host writes `struct DelugeCfg; impl GraphConfig for DelugeCfg { … }` with values
validated by the QA profiler; the web sim can pick larger ones. Sample rate is
runtime (it affects no layout).

---

## 5. Control → audio: `Cmd` + `Host`

The prototype's seam, generalized. The binding side (P1) owns the **`NodeId`**
free-list and allocates ids without a round-trip; the engine allocates each node's
output-slot run **internally** on `NewNode` (connections resolve through the
node's stored `out_base`, so `OutId` never appears on the wire). Commands:

```rust
enum Cmd {
    Nop,
    NewNode { node: NodeId, kind: Kind, args: [Input; K] },  // K primary inputs; engine allocates the output run
    SetInput { node: NodeId, port: u8, src: Input },
    SetParam { node: NodeId, param: u8, value: f32 },   // non-signal config (e.g. mode select)
    Gate    { node: NodeId, on: bool },
    Trigger { node: NodeId },
    BusWrite { src: Input, bus: BusId },                // sum src into a bus
    SetRoot { bus: BusId },                             // master (stereo) render root
    Free    { node: NodeId },
    Reset,
}

trait Host { fn audio_cmd(&self, cmd: Cmd); }
```

`NewNode` carries up to `K` primary constructor inputs (matching factory args like
`Osc.saw(freq)`); additional inputs are wired by follow-up `SetInput`. Transport
is unchanged from the prototype: firmware enqueues onto a critical-section ring
drained by the audio task; the web sim applies directly on the audio thread.

## 6. Buffer pool (scaffold)

Distinct from the per-block **output arena** (transient signal buffers, §3.2), the
**buffer pool** holds *persistent* state that outlives a block — delay lines,
reverb, sample data. P0 ships:

- a fixed `POOL_BYTES` arena with a **fixed-chunk free-list** allocator
  (allocations rounded to whole chunks; freed runs returned),
- a `PoolHandle` a node stores in its state and releases on `Free`,
- graceful failure: exhaustion returns `None`, the node degrades (e.g. a delay
  with no line passes dry) rather than panicking.

The real allocator tuning and large-buffer semantics are `Ef`/`Sa`; P0 exercises
the interface with one minimal test node (a short delay) to prove ergonomics.

## 7. Validation set

Re-implement the prototype's primitives on the new substrate — no new DSP, just
proof the model carries them and a seed for golden vectors:

- Oscillators: sine (parabolic `fast_sin`), saw, square, tri — width-1.
- Noise (seeded xorshift32), AR envelope (gate/trigger), one-pole LPF.
- Math: mul/add/sub as width-1 two-input nodes.
- Plus **one multi-output test node** (e.g. a trivial 2-port splitter or a stub
  SVF emitting LP/HP) to exercise ports end-to-end, and **one buffer-pool node**
  (short delay) to exercise §6.

Parity target: an equivalent patch renders sample-identical to the prototype
(mono → master bus) within f32 tolerance.

## 8. Testing

- **Kernels** (`deluge-dsp-kernels`): property tests (bounded output, no
  NaN/denormal, phase wrap) + **null tests** against a naïve scalar reference
  within tolerance. Where a kernel has a `core::simd` path, it null-tests against
  its own scalar fallback — and host CI runs that comparison on SSE while device
  runs it on NEON, so agreement is checked on both ISAs.
- **Graph** (`deluge-audio-graph`): golden audio vectors at a fixed
  `(sample_rate, N)`; seeded noise for reproducibility; lifecycle tests
  (create/free/reuse preserves eval order and produces no stale reads); a
  multi-output test (two consumers, two ports, single compute); a bus fan-in test.
- **Determinism:** device ≡ sim pinned by the same golden vectors (the cluster's
  established pattern).

## 9. Open questions (resolved during P0 implementation)

- Default `GraphConfig` values shipped by the library, and the Deluge's concrete
  values (validated with the QA profiler once it exists).
- `K` — how many primary inputs `NewNode` inlines before falling back to
  `SetInput` (chosen from the factory signatures in the validation set).
- Whether `enum_dispatch` (the crate) is used or a small in-tree macro generates
  the dispatch `match`.
- Chunk size for the buffer-pool allocator.
- The `simd` feature's exact gating (`#![feature(portable_simd)]` under it, scalar
  default) and which validation kernels ship a `core::simd` path in P0 vs. land
  scalar-only and get vectorized when their suite arrives.
- Exact `Cell` vs documented-`unsafe` choice for the output arena (benchmark both
  on-device).
