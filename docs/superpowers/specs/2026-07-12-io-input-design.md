# Audio Input (IO-1) — Design Spec

**Date:** 2026-07-12
**Status:** Approved (design), pending implementation plan
**Sub-project:** IO suite — IO-1 of 4 (audio input → graph). The IO suite decomposes
into IO-1 audio input, IO-2 mixer/bus expansion, IO-3 master chain, IO-4 multiple
outputs — each its own spec → plan → build. This spec covers IO-1 only.

## Context

The Deluge already **captures** codec RX audio end-to-end — SSI0 RX DMA + a ring-tap
(`deluge-bsp/src/audio.rs`, `audio_block.rs`), and the SDK hands the audio callback a
`BLOCK`-length slice **pre-loaded with codec input** (`deluge-sdk/src/audio.rs:59-60`).
But the graph engine has **no input concept**: `wren-firmware/src/audio.rs` overwrites
that pre-loaded block entirely with `eng.render()` output, discarding the input. There
is no `Kind`/`Input`/`Cmd`/binding to read external audio into the graph.

IO-1 closes that gap: expose the already-captured **stereo** line-in as a width-2 graph
source (`In.line()`), so it can be monitored (`Out.patch(In.line())`), gained, panned,
and run through the existing effect/synth graph. This is a **plumbing/API gap on top of a
complete hardware capture chain** — not a driver task. See [[sa-suite-samples]] (the
pool/`process_resolved` threading precedents), [[prefer-neon-simd]], [[wren-binding-safety]],
[[target-32bit-usize-overflow]].

**Model (chosen):** stereo width-2 source (port0=L, port1=R, matching the codec's native
I²S stereo and the vision's "stereo at buses, not nodes"); the host delivers the per-block
input via a new `render(out, input)` parameter (chunk-aligned with `out`, matching the
hardware in/out callback); **raw input only** — gain/pan/mute/monitor all compose from the
existing `Mul`/`Pan`/bus/`Out.patch` nodes.

## Design

### Engine — `crates/deluge-audio-graph/src/engine.rs`

- **Input home:** engine gains `in_l: [f32; BLOCK]` + `in_r: [f32; BLOCK]` rows, mirroring
  the existing `bus_l`/`bus_r` accumulator rows.
- **`render` signature:** `render(&mut self, out: &mut [StereoFrame], input: &[StereoFrame])`.
  At render top (before `render_block`), **deinterleave** `input` into `in_l`/`in_r` with a
  length guard: for `i in 0..BLOCK`, `in_l[i] = input.get(i).map_or(0.0, |f| f.l)` (and
  `.r`), so any frame beyond `input.len()` — including an empty slice — reads as `0.0`
  (silence). No panic on a short/empty/over-long input.
- **Threading to the node:** `render_block` passes the input rows down to
  `Node::process_resolved` via a new trailing parameter (see below), exactly as the Sa-3b
  stream cursors were threaded into `poly_process`. The engine reads `in_l`/`in_r` (disjoint
  from the per-node `arena.node_mut` borrow, like `bus_l`/`bus_r` and `stream_state`).
- **Existing call sites:** every current `render(out)` caller (inline engine tests, the
  firmware `audio_task`, the sim/host paths) is updated to pass an `input` slice — a
  silence slice where input is irrelevant. This is the one unavoidable signature ripple
  (the user accepted it over a backward-compat overload).

### Graph node — `crates/deluge-audio-graph/src/node.rs`

- **`Kind::Input`** — a new variant. **Stateless** (no `State::Input` — it holds no DSP
  state; it copies engine-owned input each block), so `Node::new`'s arm constructs it with
  no kernel.
- **Predicates:** `out_width => 2` (stereo, alongside `Pan`/`StereoVoiceSum`); NOT added to
  `is_poly` (mono-style dispatch → `process_resolved`); `poly_in_count` default 0 (pure
  source, no signal inputs).
- **`process_resolved` signature:** gains a trailing `input: Option<(&[f32], &[f32])>`
  parameter (the `(in_l, in_r)` rows, or `None`). Every existing arm ignores it — identical
  to how the `pool_region`/stream params thread through. The `Kind::Input` arm copies per
  sample `out.port(0)[i] = in_l[i]`, `out.port(1)[i] = in_r[i]` (from the `Some` tuple; if
  `None`, write silence — defensive, never hit in the real engine path).
- **No DSP kernel** — the node is a straight block copy. Per [[prefer-neon-simd]]'s "don't
  force it": a memcpy-shaped copy is already optimal; there is no data-parallel DSP loop to
  vectorize. No new `deluge-dsp-kernels` file.

### Wren — `In.line()` (`crates/deluge-wren-core/`)

- **`class In { static line() { return Node.line_() } }`** in `wren/prelude.wren` +
  `foreign static line_()` decl. A **global/top-level source** (NOT a poly voice source —
  no `polyMode_` guard; it is used like a directly-patched `Osc`, not inside `Synth.new{}`).
- **`node_line_impl<S>(vm)`** (`bindings_audio.rs`, the simplest factory in the codebase):
  `id = audio::alloc_node_id()`; `audio::new_input(id)`; `return_node(vm, id)` (the mono/
  non-poly node-return path, e.g. how `Osc.saw` returns its node — NOT `return_poly_node`).
  No foreign args to read (so no `checked_*` guard needed), no pool handle, no trigger.
- **`audio::new_input(id)`** facade: emits a single `Cmd::NewNode{Kind::Input,
  [Const, Const, Const]}` — no `BindTable`, no `SetParam`.
- Register `Node.line_()` in BOTH tables (`METHODS` + `register_audio`/`install_methods`)
  + the extern-C shim + the prelude `foreign` decl. No setters (raw input only).
- **Composition (all via existing nodes):** `Out.patch(In.line())` uses the existing
  stereo-aware `write_source_to_bus` (width-2 → port0 `(gl=1,gr=0)` + port1 `(gl=0,gr=1)`
  into master, then `set_root`); `In.line() * 0.5` (`Mul`); `In.line()` into `Room`/effects;
  `m.write(In.line())` into an explicit bus.

### Firmware/SDK wiring — `wren-firmware/src/audio.rs` (host + device)

`audio_task` currently renders `eng.render(chunk)` straight over the pre-loaded input
`block`, discarding input. Fix: before overwriting, **copy the chunk's pre-loaded input
into a stack scratch** `[StereoFrame; CHUNK]`, then `eng.render(chunk_out, &input_scratch)`
(input and output must not alias — `render` reads `input` while writing `out`). The
`Audio::process` callback already delivers input in-place; this change stops throwing it
away. Purely additive to the task loop.

## Testability (honest — as agreed)

- **Engine + node + binding: fully host-tested** (feed a synthetic input block, assert
  round-trip on both channels + the emitted Cmds).
- **Firmware chunk-copy + audible `cargo deluge sim` input: compile + cross-build + review
  ONLY here; user-verified** with real input on device / in the sim. (Same testability class
  as Sa-3b Slice 4's `stream_task` and Slice 5's device code — the automated coverage stops
  at "correct engine round-trip + correct Cmds"; live input capture is user-verified.)

## Non-breaking

Purely additive except two signature ripples, both threaded with a silence/`None` default
so **no existing node or render behavior changes**: (1) `Engine::render` gains `input:
&[StereoFrame]` (existing callers pass a silence slice); (2) `Node::process_resolved` gains
a trailing `input: Option<(&[f32], &[f32])>` (every existing arm ignores it, exactly like
the `pool_region`/stream params). No new `Cmd`, no `Kind` removed, no existing binding
touched. Existing suites stay green in both configs (default + `--features
deluge-dsp-kernels/simd`); the device build cross-compiles.

## Constraints (global)

- `no_std`, no heap, **no panic on any input** (the `render` input-length guard covers
  short/empty/over-long slices; `Kind::Input`'s `None` path writes silence). 32-bit-`usize`
  safe (the copy indexes `0..BLOCK` bounded by the guard — see
  [[target-32bit-usize-overflow]]).
- Stereo = port0 L / port1 R (the existing convention). `BLOCK` is the engine block const.
  MIT/Apache-2.0. No new foreign-arg reads (so no new `checked_*` sites), but any that arise
  use the [[wren-binding-safety]] guards.
- Raw input only — gain/pan/mute/mono-sum/monitor are OUT of scope (compose from
  `Mul`/`Pan`/bus/`Out.patch`).

## Out of scope / deferred

- **Input gain / mono-sum / monitor conveniences** (compose from existing nodes).
- **The other IO sub-projects:** IO-2 mixer/bus expansion (sends/returns/aux buses, per-bus
  master gain, bus→bus routing, the persistent routing table), IO-3 master chain (master
  insert: limiter/EQ/DC-block), IO-4 multiple named outputs (multi-destination render +
  headphone/line-out jack mapping — partly HW-gated).
- Per-input trim, input metering, latency compensation, a mono `In.mono()` variant,
  input as a poly voice source.

## Likely task decomposition (for writing-plans)

Task 1 couples the engine plumbing and the `Kind::Input` node deliberately: the engine
round-trip test needs a node to route the input rows to master, and the node needs the
engine input rows to read — neither is independently testable, so they are one task.

1. **`Kind::Input` node + engine `render(out, input)` plumbing** (`node.rs` + `engine.rs`):
   the `Kind::Input` variant (stateless) + `Node::new` arm + `out_width==2` predicate; the
   engine `in_l`/`in_r` rows + the `render(out, input)` signature (all engine-test call sites
   updated to pass a silence slice) + the deinterleave-with-guard; the `process_resolved`
   trailing `input: Option<(&[f32], &[f32])>` param threaded through `render_block` (every
   other arm ignores it) + the `Kind::Input` copy arm. Tests: node predicates (`out_width==2`,
   `!is_poly`, `poly_in_count==0`); engine round-trip — a fed input block through a
   `Kind::Input` node routed to master mirrors both channels to output; short/empty input →
   trailing silence, no panic.
2. **Wren `In.line()`** (`audio.rs`/`bindings_audio.rs`/`bindings.rs` + `prelude.wren`):
   `new_input` facade (`NewNode{Kind::Input, [Const,Const,Const]}`, no BindTable/SetParam) +
   `node_line_impl` (`alloc_node_id` → `new_input` → `return_node`, no foreign-arg reads) +
   both tables + extern-C shim + prelude `class In` + `foreign static line_`. Cmd-capture
   test: `Out.patch(In.line())` emits `NewNode{Input}` + the two stereo bus writes +
   `SetRoot`; `In.line() * 0.5` builds.
3. **e2e + firmware wiring:** real-`EngineHost` round-trip (Wren-built `Out.patch(In.line())`
   mirrors a fed input block to output; `In.line()` through `Room` renders finite/non-silent).
   Firmware `audio_task` chunk-copy into a stack scratch then `eng.render(chunk_out,
   &input_scratch)` (host build + `armv7a-none-eabihf` cross-build clean; review only — user
   verifies audible input in the sim / on device).
