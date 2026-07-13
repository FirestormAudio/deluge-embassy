# Bus-Fed Effects — One-Block-Delayed Bus Reads (IO-2e) — Design Spec

**Date:** 2026-07-13
**Status:** Approved (design), pending implementation plan
**Sub-project:** IO suite — IO-2 mixer/bus expansion, a foundational engine slice that
unblocks IO-2d (sends/returns/aux sugar). IO-2 slices: IO-2a routing table, IO-2b per-bus
gain, IO-2c bus→bus routing (all DONE), **IO-2e bus-fed effects** (this spec), IO-2d
Aux/Mixer sugar (next, once IO-2e lands).

## Context

A node reading `Input::Bus(b)` currently sees `0.0`: `render` zeroes all buses BEFORE
`render_block`, and buses are only filled from node outputs AFTER (the write loop). So
feeding a bus INTO an effect node (an aux reverb/delay return — the canonical mixer
workflow) reads silence. IO-2e fixes this with **one-block-delayed bus reads**: reorder
`render` so nodes read the buses' current content (the PREVIOUS block's end-of-block state)
before the zero+refill. A node reading `Input::Bus` then sees last block — a BLOCK-sample
latency (~0.7 ms at 48 kHz / 32-sample blocks), inaudible and standard for effect sends.
See [[io-suite-routing]].

**Model (chosen):** one-block-delayed reads (the simple, low-risk approach) over the
same-block topological alternative — negligible latency for a fraction of the complexity.

## Design

Engine-internal only (`deluge-audio-graph/src/engine.rs`). No new `Cmd`/field/binding.

### 1. Render-order swap

Move the "zero buses" step from BEFORE `render_block` to AFTER it. Current order:

```
fill in_l/in_r → ZERO buses → render_block → write loop → sends → gain → master → copy root
```

New order:

```
fill in_l/in_r → render_block → ZERO buses → write loop → sends → gain → master → copy root
```

So `render_block` (the node-evaluation pass, which resolves `Input::Bus(b)` as
`bus_l[b]+bus_r[b]`) now reads the bus rows as left by the PREVIOUS `render` (its end-of-block
state after write+sends+gain+master), or `[0.0]` on the very first render. Then the buses are
zeroed and refilled from this block's node outputs.

### 2. Reset zeros the buses

Buses now carry state across `render` calls (they are no longer zeroed at the top). So the
`Cmd::Reset` arm must also zero them (`self.bus_l = [[0.0; BLOCK]; BUSES]; self.bus_r =
[[0.0; BLOCK]; BUSES];`) so a reset engine does not feed stale bus content into the next
block's `Input::Bus` reads. (Currently `Reset` clears arena/writes/root/stream/master/bus_gain/
bus_sends but not `bus_l`/`bus_r`, which were previously zeroed every render.)

### 3. Why byte-identical for existing graphs

`render_block`'s node outputs depend on bus contents ONLY through `Input::Bus` — and every
current graph reads `0.0` there (an unused path, since buses were zeroed just before). For any
graph with no `Input::Bus` node-read, `render_block` produces identical node outputs, and the
subsequent zero → write → sends → gain → master → copy steps are identical → **output
byte-identical**. The only behavior change is the intended one: `Input::Bus` node-reads see
last block instead of `0.0`.

### 4. Semantics note

The bus content a node reads is the PREVIOUS block's END-OF-BLOCK state (post write, sends,
gain, and — for the root bus — the master chain). For an aux bus (node-writes + sends-into-it
+ its gain), that is the fully-processed aux, one block late — the correct content for an
effect return. Reading the root bus via `Input::Bus` would see the master-processed root
(an unusual pattern; documented, not a concern for the aux use case).

## Non-breaking

Purely a render-order swap + a Reset bus-zero. No new `Cmd`, field, binding, or Wren change.
Output byte-identical for all graphs that don't read a bus as a node input (i.e. every
existing graph). `no_std`, no-heap, no-panic, 32-bit-`usize` safe. Both configs stay green;
device cross-build clean. MIT/Apache-2.0.

## Constraints (global)

- `no_std`, no heap, no panic. 32-bit-`usize` safe. MIT/Apache-2.0. Engine-internal only.
- **`Input::Bus` node-reads are one block (BLOCK samples) late.** Same-block (zero-latency,
  topological) is explicitly deferred.
- Existing non-bus-reading graphs MUST render byte-identically.

## Out of scope / deferred

- **Same-block (zero-latency) bus reads** (topological node+bus evaluation) — deferred; the
  one-block latency is acceptable for the aux-send use case.
- **IO-2d Aux/Mixer sugar** — the next slice, now unblocked by IO-2e.

## Testing

- **`bus_fed_node_reads_previous_block`** (the crux): node0 = a const source written to bus1;
  node1 reads `Input::Bus(1)` and is routed to master (bus0). Render TWO blocks → block 1's
  master output is `~0` (node1 read the initial zeroed bus1), block 2's reflects node0's
  block-1 write (the one-block-delayed read now works). Proves bus→node routing functions,
  one block late.
- **`existing_routing_byte_identical`** (or lean on the existing `two_nodes_sum_into_master_bus`
  / render tests): a no-`Input::Bus` graph renders the exact same output value as before the
  reorder — byte-identical.
- **`reset_zeros_buses`**: drive bus content, `Cmd::Reset`, rebuild a bus-reading graph →
  the first post-reset render reads `0` (not stale content).
- All existing `deluge-audio-graph` tests stay green both configs (the reorder is
  output-preserving for non-bus-reading graphs, which is all of them).

## Likely task decomposition (for writing-plans)

Single task (one file, `engine.rs`):
1. **Render-order swap + Reset bus-zero** (`engine.rs`): move the bus-zeroing to after
   `render_block`; add `bus_l`/`bus_r` zeroing to `Cmd::Reset`; the three tests
   (`bus_fed_node_reads_previous_block`, byte-identical, `reset_zeros_buses`). Full graph
   crate green both configs; dependent crates build.
