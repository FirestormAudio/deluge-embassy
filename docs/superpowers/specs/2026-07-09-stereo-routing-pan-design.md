# Stereo bus routing + Pan — design & spec

A **graph-infrastructure** sub-project: give the audio graph a real stereo signal
path and a constant-power **`Pan`** node, so a mono source can be placed in the
stereo field and a **mono→stereo node** (Pan now; Chorus/Flanger next) can send
distinct L/R to the output. This is the prerequisite that unblocks the stereo
**Ef-2 Chorus/Flanger** sub-project.

> **Status:** design proposal. Depends only on the merged graph (`engine.rs`
> bus model, `node.rs` width-2 nodes à la `Split2`, `cmd.rs`) and the P1 Wren
> bindings. No new kernels beyond a tiny stateless pan; MIT/Apache (nothing
> ported).

**Roadmap:** this sub-project → **Ef-2 Chorus/Flanger** (mono→stereo effect
nodes built on this) → Ef-3 Reverb (stereo) → Ef-4 Drive → Ef-5 EQ.

---

## 1. The stereo convention (core idea)

**A stereo signal *is* a width-2 node: port 0 = L, port 1 = R.** There is no new
"stereo value" type in the kernels or the `In`/`Input` model — width-2 is the
entire convention, and the graph already supports width-2 nodes (`Kind::Split2`,
`OutView::pair`, `out_width == 2`). A **mono→stereo node** takes a mono input and
writes both output ports. Mono signals stay width-1 and route to center (L = R),
exactly as today.

- **Stereo → stereo chaining** (e.g. a future stereo reverb after a stereo
  chorus) is expressible now via explicit ports (`node.out(0)` = L, `node.out(1)`
  = R) and is **not** special infrastructure here.
- Feeding a stereo (width-2) node where a **mono** value is expected uses **port
  0 (L)**, documented (see §4).

## 2. `Kind::Pan` — the constant-power pan node

A **stateless** mono→stereo node. `out_width` = 2.

- **Ports:** 0 = input (mono signal), 1 = **position** ∈ [−1, +1] — a *port*, so
  pan position is audio-rate modulatable (auto-pan via an LFO/env).
- **Constant-power law, no trig** (Cortex-A9 has hardware `VSQRT`):
  ```text
  p = (position.clamp(-1.0, 1.0) + 1.0) * 0.5   // → [0,1]
  L = input * sqrt(1.0 - p)
  R = input * sqrt(p)
  ```
  Hard-left (pos −1): `(L,R) = (1,0)`. Center (pos 0): `(0.707…, 0.707…)` = −3 dB
  each. Hard-right (pos +1): `(0,1)`. `L² + R²` is constant (constant power).
- Written into the graph render match as its own arm (mirrors `Kind::Split2`,
  which writes both ports): compute `L`/`R` per sample from `ins[0]` (input) and
  `ins[1]` (position), write `outs.port(0)`/`outs.port(1)`.
- No control params, no `State` (stateless like `Mul`/`Add`/`Split2`).

**Wren:** `Pan.new(input, position)` → a width-2 (stereo) node.

## 3. Engine routing — per-side write gains

Today a bus write is `(src, bus)` and the source is summed **equally** into
`bus_l` and `bus_r` (hard center). Generalize a write to carry **per-side
gains**:

- `engine.rs`: `writes: [Option<(Input, BusId, f32, f32)>; NODES]` — `(src, bus,
  gain_l, gain_r)`. In `render`, apply `bus_l[b][i] += v * gl; bus_r[b][i] += v *
  gr;`.
- `bus_write(&mut self, src: Input, bus: BusId, gl: f32, gr: f32)`. The internal
  mono/center write passes `(…, 1.0, 1.0)` → **byte-identical** to today's
  behavior (regression-guarded).
- A **stereo source** (width-2 node) routes as **two** writes:
  `(Input::Node{node, port:0}, bus, 1.0, 0.0)` (L port → L bus) and
  `(Input::Node{node, port:1}, bus, 0.0, 1.0)` (R port → R bus).
- `Cmd::BusWrite { src, bus }` → `Cmd::BusWrite { src, bus, gl, gr }`. Existing
  constructors (engine/cmd tests, `audio::bus_write`) pass `gl: 1.0, gr: 1.0`
  (mechanical update; enumerated in the plan).

Note: **pan itself is a node** (§2), so a *panned mono* source is already a
width-2 `Pan` node — its two ports carry the pan-weighted L/R and route via the
same two-write stereo path. The engine never needs a "pan value" in the write;
it only needs the L/R port routing (`1,0` / `0,1`). The `gl`/`gr` generality is
kept minimal but leaves room for a future direct pan-on-write if wanted.

## 4. Wren surface — width-aware `patch`/`write`

The routing layer must know whether a source is stereo (width-2) to emit the two
L/R writes. Track it on the foreign object:

- **`NodeObj`** gains a `width: u8` field: `{ tag: u8, width: u8, id: u16 }`
  (still 4 bytes — `arg_input` only ever reads the leading `tag` byte for
  unknown tags, so the layout change is safe). Mono factories set `width = 1`;
  stereo factories (`Pan.new`; later `Chorus`/`Flanger`) set `width = 2`.
- **`Out.patch(x)`** and **`Bus.write(x)`** read the argument's foreign directly
  (via its tag) to get its width **before** converting to an `Input`:
  - width-1 `Node` (or a `Port`, or a number/`Bus`): one mono-center write
    `(src, bus, 1.0, 1.0)` — unchanged behavior.
  - width-2 `Node`: two writes — `(Node{id,0}, bus, 1,0)` and `(Node{id,1}, bus,
    0,1)`.
  - a `Bus` argument to `Out.patch`: `set_root(bus)` (unchanged).
- **`Node.out(p)`** returns a `PortObj` (a single port) — inherently mono; a
  `Port` always routes mono-center. (Route a stereo node's sides explicitly by
  `n.out(0)`/`n.out(1)` when you need them separately.)
- **Mono context for a stereo node:** `arg_input` on a width-2 node yields
  `Input::Node{node, port:0}` (its **L** port). Documented so `Pan.new` /
  `Chorus.new` results used as a mono input degrade predictably (L only) rather
  than silently taking port 0 by accident.

**Prelude:** a `Pan` sugar class (`Pan.new(input, position)` →
`Node.pan_(input, position)`), and the width-aware `Out.patch` / `Bus.write`
logic in the foreign impls.

## 5. QA acceptance & testing

**Kernel/graph (`deluge-audio-graph`):**
- **Pan law.** Hard-left position → port0 == input, port1 == 0; hard-right →
  port0 == 0, port1 == input; center → both == `input * 0.707…` (±tol);
  `port0² + port1²` ≈ `input²` across positions (constant power).
- **Modulated position (auto-pan).** A time-varying position port sweeps the
  energy from L to R smoothly (no zipper/step artifact beyond per-sample).
- **Per-side write gains.** A `BusWrite` with `(gl,gr)=(1,0)` lands only in
  `bus_l`; `(0,1)` only in `bus_r`; `(1,1)` in both (regression: existing
  center writes unchanged — the merged golden/parity tests still pass).
- **Stereo source → distinct L/R.** A width-2 node routed via two writes puts
  different signals on L vs R at the output (`StereoFrame.l != .r`).
- **Graceful:** a stereo node written mono-center, or a mono node written to one
  side, never panics; out-of-range ports contribute silence (existing rule).

**Wren (`deluge-wren-core`):**
- `Pan.new(in, pos)` emits a `Kind::Pan` `NewNode`; `Out.patch(Pan.new(...))`
  emits **two** `BusWrite`s with `(1,0)` and `(0,1)` to the master bus + a
  `SetRoot` — verified on `CmdCaptureHost`.
- `Out.patch(monoNode)` still emits a single `(1,1)` center write (regression).
- End-to-end on `EngineHost`: a hard-panned source renders with `l != r` and is
  bounded; a centered mono source renders `l == r`.

**Determinism:** no RNG/time; reproducible.

## 6. Deferred / follow-ups

- **Ef-2 Chorus/Flanger** — the first *effect* mono→stereo nodes, built on this
  (`ModDelay` kernel: internal LFO modulating `read_hermite`, phase-spread voices
  for chorus, feedback for flanger; width-2 L/R output).
- **Stereo→stereo effect nodes** (2-in/2-out chaining) — a per-effect concern
  when a stereo reverb/effect needs a stereo input, not general infra.
- **Direct pan-on-write** (a `gl`/`gr` derived from a pan value at the write
  site, without a `Pan` node) — the `gl`/`gr` write generality already allows it;
  add a Wren `write(src, pan:)` sugar only if a use case appears.
- **Tempo-sync, balance/width controls, mid-side** — later stereo refinements.
