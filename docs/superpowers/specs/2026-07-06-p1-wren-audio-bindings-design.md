# P1 — Low-level Wren audio bindings — design & spec

Sub-project P1 of the [DSP library cluster](2026-07-06-dsp-library-vision-design.md).
Rebind the Wren audio surface in `deluge-wren-core` onto the new
[P0 engine](2026-07-06-p0-dsp-graph-foundation-design.md) (`deluge-audio-graph`),
superseding the prototype's `Osc`/`Env`/`Out` classes and their old DSP engine.
Reaches parity with today's Wren API, then exceeds it with the engine's new
capabilities: **multi-output ports, buses, and explicit spawn/free lifecycle**.

> **Status:** design proposal. Depends on P0 (merged). This is the low-level
> imperative layer; the high-level `SynthDef`/voice layer is `Sy`, a later
> sub-project.

**Approach: refactor `deluge-wren-core` in place** (not a new crate). The audio
bindings are tightly coupled to that crate's VM infrastructure (`SlotApi`, the
`Host` trait, `prelude.wren`, `CLASSES`/`METHODS` registration); a separate
`deluge-audio-wren` crate would require extracting those traits first. That
extraction is deferred to whenever the audio surface grows enough to warrant it
(naturally at `Sy`). The vision's "a non-Wren host depends only on the two lower
crates" property already holds — `deluge-audio-graph` has no Wren dependency.

---

## 1. Goals & non-goals

**Goals**

- Retarget `deluge-wren-core`'s Wren audio classes onto `deluge-audio-graph`,
  emitting `deluge_audio_graph::Cmd` through the existing `Host::audio_cmd` seam.
- **Parity:** `Osc`/`Env`/`Noise`/`Out`, the `*`/`+`/`-` operators, `.lpf()`,
  `.freq=`/`.cutoff=`, `.gate()`/`.trigger()` keep working — existing scripts
  (e.g. `wren-firmware/examples/midi_synth.wren`) still run or take a one-line port.
- **New surface:** output ports (`node.out(p)`), buses (`Bus.new`, `.write`,
  bus-as-input, bus-as-root), and explicit `node.free()` lifecycle.
- **Client-side allocators:** the binding owns the `NodeId` free-list and `BusId`
  allocation (no round-trip), per the P0 control→audio contract.
- **Validated on the host** by real-VM tests (extended `test_support`): a
  capturing host asserts `Cmd` sequences, and an applying host renders audio end
  to end (script → `Cmd` → concrete `Engine` → rendered block). No hardware/wasm.
- Keep the workspace compiling: minimally re-wire `wren-firmware`'s audio task
  (the one in-workspace consumer) to the new engine.

**Non-goals (deferred, tracked)**

- The high-level `SynthDef`/voice layer and MIDI voice allocation — `Sy`.
- Migrating `tools/wren-web` (the wasm sim, excluded from the workspace) and any
  device performance validation/tuning — a tracked P1 follow-up.
- Extracting a standalone `deluge-audio-wren` crate — deferred (see Approach).
- New DSP kinds beyond what P0 shipped — the vocabulary suites (`Osc`/`Fi`/…).
- Per-bus pan/gain, multi-bus routing beyond master + explicit buses — later.

---

## 2. Structure & build integrity

### 2.1 Changes in `deluge-wren-core`

| File | Change |
|---|---|
| `Cargo.toml` | add `deluge-audio-graph` path dependency |
| `src/engine.rs` | **delete** — the prototype DSP engine; superseded by `deluge-audio-graph` |
| `src/audio.rs` | **rewrite** — client-side `NodeId` free-list + `BusId` allocator + `Cmd` emitters targeting `deluge_audio_graph::Cmd` |
| `src/bindings_audio.rs` | **new** — the `Node`/`Port`/`Bus`/`Out` foreign classes, split out of the 1125-line `bindings.rs` (a focused module; that file does too much) |
| `src/bindings.rs` | remove the audio (`Node`) section; keep CV/gate/MIDI/pads/buttons/enc/LED/OLED/metro untouched; register the new audio classes in `CLASSES`/`METHODS` |
| `src/host.rs` | `Host::audio_cmd(&mut self, cmd: Cmd)` now carries `deluge_audio_graph::Cmd`; rest of the trait unchanged |
| `src/lib.rs` | re-export `Cmd`/`Engine`/`Input`/`Kind` from `deluge-audio-graph` (replacing the old `engine` re-exports); drop `K_*`/`MAX_NODES` |
| `wren/prelude.wren` | rewrite only the "Audio: native DSP graph" section; everything else untouched |

The non-audio Wren surface (CV/gate/MIDI/surface/OLED) is not touched.

### 2.2 Keeping the workspace green

`wren-firmware` is the **only in-workspace consumer** of `deluge-wren-core`
(it builds on host — `cargo deluge sim` — and device). `tools/wren-web-debug`
and `tools/wren-web` are excluded from the workspace and built separately.

An in-place refactor changes `deluge-wren-core`'s audio API, so P1 includes the
**minimal mechanical re-wire of `wren-firmware/src/audio.rs`**: its command ring
carries the new `Cmd`; its audio task owns a concrete
`deluge_audio_graph::Engine<BLOCK, NODES, OUTS, BUSES>`, applies queued commands,
and renders the engine's **stereo block** into the SDK `Audio` callback (the old
task rendered mono per-frame via `render_frame`; the new engine renders a block
of `StereoFrame` via `render`). This is the smallest edit that avoids a broken
build — not a tuned migration. `wren-firmware`'s `Host` impl (`FwHost::audio_cmd`)
keeps the same enqueue mechanism; only the `Cmd` type changes.

**Deferred (tracked follow-up):** `tools/wren-web` (wasm) migration and device
performance validation.

### 2.3 The host-capacity contract

The engine's arena sizes are host const-generics the binding cannot see, but the
binding allocates ids client-side, so it declares its own caps:

```rust
pub const WREN_MAX_NODES: usize = 64;   // binding-side id space
pub const WREN_MAX_BUSES: usize = 8;
```

**Contract:** a host wiring an `Engine` must size it with `NODES >= WREN_MAX_NODES`
and `BUSES >= WREN_MAX_BUSES` (and `OUTS` large enough for the widest reachable
graph). Documented at the binding and asserted where a host constructs its engine.
Allocation past a cap yields an out-of-range id whose factory no-ops — inert,
never a panic — matching the engine's own out-of-range contract.

---

## 3. The Wren API surface

### 3.1 Parity (unchanged)

```wren
Osc.sine(f) / saw(f) / square(f) / tri(f)     // → Node
Env.ar(a, r)                                   // → Node; .trigger(), .gate(on)
Noise.new()                                    // → Node
a * b,  a + b,  a - b,  node.lpf(cutoff)        // → Node
node.freq = v,  node.cutoff = v
Out.patch(x),  Out.reset()
```

### 3.2 Output ports

A bare `Node` used as an input resolves to its **port 0** (so single-output usage
is identical to the prototype). Other ports are reached via `.out(p)`, which
returns a lightweight `Port` value usable anywhere an input is:

```wren
var s = Split.new(src)              // width-2 demonstrator node (P0's Split2)
Out.patch(s.out(0) + s.out(1))
```

`Split.new` is exposed so ports are reachable and testable from Wren before the
`Fi` suite brings real multi-output filters (SVF LP/HP/BP). Port access is
`.out(p)` (explicit) rather than a `[]` subscript.

### 3.3 Buses

```wren
var mix = Bus.new()
mix.write(voiceA)                   // sum a source into the bus (Cmd::BusWrite)
mix.write(voiceB)
Out.patch(mix)                      // set the bus as render root (Cmd::SetRoot)
var trem = Osc.sine(5) * mix        // a Bus as an operand reads its (mono) signal
```

### 3.4 Lifecycle

```wren
var n = Osc.saw(110)
Out.patch(n)
n.free()                            // free the id + emit Cmd::Free
```

Explicit only: a node lives until `node.free()` or `Out.reset()`. Predictable and
real-time-safe; no GC finalizer (a node must not stop at a nondeterministic GC
point). Un-freed nodes persist until the next reset — the `Sy` voice layer will
automate lifecycles later.

### 3.5 The `Out.patch(x)` unifier

The render root is now a **bus**, not a node; `Out.patch` absorbs the change while
keeping the prototype call working:

- `x` is a `Node`/`Port` → write it into the implicit **master bus** (`BusId(0)`)
  and set that bus as root. (This *is* the prototype's `Out.patch(node)`, now sugar.)
- `x` is a `Bus` → set that bus as root directly.
- `Out.reset()` → `Cmd::Reset`; also clears the binding's id/bus allocators and
  re-arms the master bus.

---

## 4. Binding internals

### 4.1 Allocators & emitters (`audio.rs`)

- **`NodeId` free-list** over `[_; WREN_MAX_NODES]`: `alloc_node_id() -> u16`
  (reuse a freed id, else bump), `free_node_id(id)`.
- **`BusId` allocator** over `WREN_MAX_BUSES` with `BusId(0)` reserved as master.
- **Cmd emitters** — `new_node`, `set_input`, `set_param`, `gate`, `trigger`,
  `bus_write`, `set_root`, `free`, `reset` — each builds a `deluge_audio_graph::Cmd`
  and calls `host().audio_cmd(cmd)`. Kinds map to `deluge_audio_graph::Kind`.
- **`reset()`** clears both allocators, re-arms the master bus, emits `Cmd::Reset`.

State is a VM-thread process-global (matching the rest of the binding state and
the existing `host()` accessor).

### 4.2 Foreign classes (`bindings_audio.rs`)

- `NodeObj { id: u16 }` — factories (`Osc.*`, `Env.ar`, `Noise.new`, binops,
  `.lpf`); instance `.freq=`/`.cutoff=`, `.gate`/`.trigger`, `.out(p)`, `.free()`.
- `PortObj { node: u16, port: u8 }` — produced by `.out(p)`; only ever an input.
- `BusObj { id: u16 }` — `Bus.new`, `.write(src)`; usable as an input.
- `arg_input(vm, slot) -> Input` — one resolver: `Num → Input::Const`,
  `Node → Input::Node { id, 0 }`, `Port → Input::Node { id, port }`,
  `Bus → Input::Bus(id)`. Mirrors the prototype's `arg_input`.

All bound through the existing `SlotApi`/`WrenForeign` abstraction, so binding
bodies stay backend-generic (`impl<S: SlotApi>`) and unit-testable.

---

## 5. Testing

There is **no mock `SlotApi`** in the codebase — the established pattern
(`test_support`, `tests/golden_sim.rs`) boots the *real* wren-sys C VM with a
`Host` impl and reads state back. P1 follows that pattern with a **capturing /
applying `Host`**, not a mock. Tests live in `deluge-wren-core/tests/` (in the
workspace, run under `cargo test`), driven by an extended `test_support`. The
`deluge_audio_graph::Cmd`/`Input`/`Kind` types gain `Debug`/`PartialEq` derives so
sequences can be asserted (Task 1).

### 5.1 Cmd-sequence tests (real VM + capturing host)

Extend `test_support` with a **`CmdCaptureHost`** whose `audio_cmd` records every
command, and a helper that boots the VM, runs a script, and returns the captured
`Vec<Cmd>`. Assert the exact sequence for:

- each factory (`Osc.saw` etc.) emits `NewNode` with the right `Kind` + args;
- `.freq=`/`.cutoff=` emit `SetInput` on the right port;
- `a * b`, `.lpf(c)` build the right binop/filter nodes with resolved inputs;
- `.out(p)` yields a `Port` that resolves to `Input::Node { id, p }`;
- `Bus.new`/`.write`/`Out.patch(bus)` emit `BusWrite`/`SetRoot`;
- `.free()` emits `Free` and returns the id to the free-list (next alloc reuses it);
- `Out.reset()` emits `Reset` and clears the allocators;
- allocation past `WREN_MAX_NODES` no-ops (inert id).

### 5.2 End-to-end golden audio (real VM + applying host)

Add an **`EngineHost`** to `test_support` that owns a concrete
`deluge_audio_graph::Engine<…>` and applies each `audio_cmd` to it, plus a helper
that runs a script then renders a block. Then:

- **golden script test:** run `Out.patch(Osc.saw(110).lpf(800))`, render one
  block, assert a pinned output block — proving script → `Cmd` → engine → audio
  end to end on the host;
- a small script exercising ports (`Split`) and buses renders finite, bounded audio.

*(Wiring `tools/wren-web-debug`'s existing `RecordingHost::audio_cmd` to an
`Engine` for interactive use is a nice-to-have, not required for P1's automated
proof — the authoritative tests are the in-workspace `deluge-wren-core` ones above.)*

### 5.3 Compat check

`wren-firmware/examples/midi_synth.wren` (and any other example scripts) still
parse and run against the new bindings — a parse/run smoke test through the same
VM harness.

---

## 6. Open questions (resolved during implementation)

- Concrete `WREN_MAX_NODES` / `WREN_MAX_BUSES` / `OUTS` defaults and the assertion
  form a host uses to prove `Engine` capacity meets them.
- Whether `Port` is a foreign object or a tagged small value (both work through
  `SlotApi`); pick the lighter one that the VM boundary allows.
- The block-size reconciliation in `wren-firmware/src/audio.rs` (SDK `Audio`
  block length vs the engine's const `BLOCK` — render in `BLOCK`-sized chunks).
- Exact prelude wording/ordering for the rewritten audio section.
