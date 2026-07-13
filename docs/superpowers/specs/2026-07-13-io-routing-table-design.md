# Persistent Routing Table — Write Invalidation (IO-2a) — Design Spec

**Date:** 2026-07-13
**Status:** Approved (design), pending implementation plan
**Sub-project:** IO suite — IO-2 mixer/bus expansion, FIRST sub-slice. IO-2 decomposes into
**IO-2a persistent routing table** (this spec), IO-2b per-bus gain/trim, IO-2c bus→bus
routing, IO-2d sends/returns/aux API. Follows IO-1 (audio input) and the complete IO-3
master chain.

## Context

Bus routing today (`crates/deluge-audio-graph/src/{engine.rs, bus.rs}`) is an **append-only
`writes[]` list** of `(Input src, BusId bus, f32 gl, f32 gr)`, re-applied every `render` and
**never invalidated** (the field comment: *"P0: no persistent routing table yet"*). `Cmd::Free
{node}` frees the node's arena slot / pool region / stream cursor but leaves its bus writes in
the list. Because writes resolve `Input::Node` by id at render time (with a dangling-guard →
`0.0`), a freed node's writes are harmlessly silent — UNTIL the id is **reused**: the stale
write silently re-binds to the recreated node (double-sum / wrong routing). The existing
`free_then_reuse_keeps_eval_order_sound` test documents exactly this hazard (bus0 sums the
recreated node's output twice → `0.5`).

IO-2a fixes the lifecycle: **a node's bus writes are invalidated when it is freed**, so a
reused id inherits no stale routing. Chosen model (per brainstorming): **keyed by source node**
— minimal, engine-internal, no new `Cmd`/binding. Removable individual sends (handle-based) are
deferred to IO-2d. See [[io-suite-routing]].

## Design

Purely engine-internal (`deluge-audio-graph`). No new `Cmd`, no `audio.rs`/binding/prelude
change, no Wren surface. Three small edits to `engine.rs`:

### 1. `bus_write_gains` — reuse freed slots

Today it always appends at `writes_len` (dropping the write if `writes_len == writes.len()`).
Change it to **reuse a `None` hole** first:
- scan `0..self.writes_len` for the first `None` entry; if found, place the write there and
  return.
- else if `self.writes_len < self.writes.len()`, append at `writes_len` and increment (the
  current behavior).
- else drop (array full — unchanged).

For a fresh engine with no frees there are no holes, so writes append in the same order as
today → **existing routing is byte-identical**. Hole-reuse only matters after an invalidation,
preventing the `NODES`-sized array from leaking slots across free/patch cycles.

### 2. `Cmd::Free { node }` — invalidate that node's writes

After the existing free steps (pool-region free if bound, `stream_state[idx] = None`,
`arena.free(node)`), null every write sourced from the freed node:

```rust
for w in self.writes.iter_mut() {
    if let Some((Input::Node { node: n, .. }, ..)) = w {
        if *n == node {
            *w = None;
        }
    }
}
```

`Input::Const`/`Input::Bus`-sourced writes are untouched (they don't reference the node —
correct). A node sourcing multiple writes (a stereo node's two per-side writes, or node→master
+ node→aux) gets **all** of them dropped — correct, since the node's output no longer exists.
This is the only behavior change: it affects the render **only** when a freed id is later
reused (otherwise the dangling-guard already yielded `0.0`).

### 3. `Cmd::Reset` — clear the whole array

Today `Reset` sets `writes_len = 0` (leaving stale `Some` entries above index 0, which the
hole-reuse scan could otherwise resurrect). Also set `self.writes = [None; NODES]` for a clean
slate. Defensive + keeps the hole-reuse invariant simple.

### Unchanged

The `render` write-accumulation loop is UNCHANGED — it already `if let Some((src, bus, gl,
gr)) = self.writes[w]`-skips `None` entries. `Input::Node`/`Input::Const`/`Input::Bus`
resolution (incl. the dangling-guards and the `Input::Bus(_) => 0.0` bus→bus stub), the master
chain, `set_root`, and every other `Cmd` are untouched.

## Non-breaking

Behavior-preserving except the intended fix. A no-free patch routes byte-identically (no
`None` holes appear without a `Free`, so `bus_write_gains` appends in the same order). No Wren/
binding change; no new `Cmd`. `no_std`, no-heap, no-panic (the invalidation scan is a bounded
`iter_mut` with no new indexing), 32-bit-`usize` safe. Both configs stay green (except the one
`free_then_reuse` test whose expectation is corrected — see Testing); device cross-build clean.

## Constraints (global)

- `no_std`, no heap, no panic on any input. 32-bit-`usize` safe. MIT/Apache-2.0.
- Engine-internal only — no binding/prelude/Wren surface this slice.
- Fresh-engine routing MUST be byte-identical (hole-reuse must not perturb append order when
  there are no holes).

## Out of scope / deferred (later IO-2 slices)

- **Removable individual sends** (handle-based `Cmd::RemoveWrite`) — IO-2d, when the sends API
  needs to drop one send without freeing the node.
- **Re-patch-replace semantics** (`Out.patch(y)` after `Out.patch(x)` replacing rather than
  adding) — a separate routing-semantics question; today (and after IO-2a) re-patch is
  additive.
- **Per-bus gain/trim** (IO-2b), **bus→bus routing** (IO-2c), **sends/returns/aux API** (IO-2d).

## Testing

- **`free_then_reuse_no_stale_rebind`** (the crux, NEW): build a patch routing `NodeId(0)` to
  a bus, `Cmd::Free { node: NodeId(0) }`, recreate `NodeId(0)` as a different node WITHOUT
  re-adding a write, `set_root`, render → the recreated node's bus is **silent** (its
  predecessor's write was invalidated, and no new write was added) — proving no stale routing
  is inherited.
- **Update `free_then_reuse_keeps_eval_order_sound`** (`cmd.rs`): today it asserts `out[0].l ≈
  0.5` (the stale write from `saw_patch` + the re-added write both name `NodeId(0)`, summing
  twice). After IO-2a, `Free` invalidates `saw_patch`'s write, so only the re-added write
  remains → assert `out[0].l ≈ 0.25`. Update the comment to explain IO-2a invalidates the
  pre-free write (it no longer re-binds to the recreated id). This test change IS the proof
  that the hazard is fixed.
- **`bus_write_reuses_freed_slot`** (NEW): record a write, `Free` its source (a `None` hole
  appears), record a new write → it occupies the hole (assert `writes_len` did not grow past
  the reused index) — proves no slot leak.
- **`fresh_engine_routing_byte_identical`** (NEW, or lean on existing render tests): a no-free
  patch (e.g. two nodes summed to master, mirroring `two_nodes_sum_into_master_bus`) renders
  exactly as before — hole-reuse doesn't perturb append order.
- All existing `deluge-audio-graph` tests stay green both configs (only the one `free_then_reuse`
  expectation changes).

## Likely task decomposition (for writing-plans)

Small enough for a **single task** (one file, `engine.rs`, plus the `cmd.rs` test update):
1. **Write invalidation + slot-reuse** (`engine.rs` + `cmd.rs` test): `bus_write_gains`
   hole-reuse; the `Cmd::Free` invalidation scan; the `Cmd::Reset` array-clear; the new tests
   (`free_then_reuse_no_stale_rebind`, `bus_write_reuses_freed_slot`) + the corrected
   `free_then_reuse_keeps_eval_order_sound` expectation. Full graph crate green both configs;
   dependent crates build.
