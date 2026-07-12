# `Kind::StreamPlayer` Graph Node + Engine Cursor Plumbing — Design Spec (Sa-3b Slice 3)

**Date:** 2026-07-12
**Status:** Approved (design), pending implementation plan
**Sub-project:** Sa-3b (SD sample streaming) — Slice 3 of 5. Follows Slice 1 (WAV decoder) + Slice 2 (`PolyStreamPlayer` kernel), both on main.

## Context

Slice 2 built the `PolyStreamPlayer` kernel — each voice reads a moving ring
window via `process_voice(v, ring, fill_lo, fill_hi, total, rate, out)` +
`read_cursor(v)`. THIS slice wires it into the graph/engine: a `Kind::StreamPlayer`
node that partitions one pool region into 8 per-voice rings, threads the fill
cursors (owned by the engine, updated by the — later-slice — prefetch task via a
new `Cmd`), computes per-voice `rate` from the pitch tile, and exposes each
voice's `read_cursor` back out. No SD / no Wren binding yet (slices 4–5); this
slice is testable via the engine + a **mock prefetch** (write the pool region +
submit fill Cmds). See [[sa-suite-samples]].

**Pitch model (chosen):** a single streamed file per `StreamPlayer`, one root
note; the 8 voices play that same file at `rate = hz/mtof(root)` (8 windows into
one file). Keymap-of-streamed-samples is deferred.

## Design (reuses the existing patterns; the audit confirmed each)

### Ring storage — one region, partitioned (audit option a)

The 8 per-voice rings are ONE pool region sized `VOICES * ring_cap`, bound via the
**existing** `Cmd::BindTable{Pooled(h)}` → `Node.table` → engine
`pool.slice_mut(h)` → `pool_region` path, **unchanged**. The node arm slices
`region[v*cap .. (v+1)*cap]` per voice (`cap = region.len() / VOICES`). No change
to `TableSrc`, `BindTable`, or the engine's pool resolution.

### Cursors — net-new, engine-owned (NOT in `Node`/`State`)

`Node`/`State` are `#[derive(Clone,Copy)]`, stored by value in a fixed array, and
hold *kernel* state; the fill cursors are *engine/prefetch* state (produced by the
prefetch task, consumed at render). So add a new `Engine` field, keyed by node
index, parallel to `pool`:

```rust
struct StreamCursors {
    total: u64,                    // full sample length (same file → one value)
    fill: [(u64, u64); VOICES],    // per-voice resident window [fill_lo, fill_hi)
}
// Engine: stream_state: [Option<StreamCursors>; NODES]  (or a small capped map)
```

Updated by a new command:
```rust
Cmd::StreamFill { node: NodeId, voice: u8, fill_lo: u64, fill_hi: u64, total: u64 }
```
`Engine::apply(Cmd::StreamFill{..})` → lazily create `stream_state[node]` if absent,
set `sc.total = total` and `sc.fill[voice] = (fill_lo, fill_hi)`. Mirrors the
existing per-voice `GateVoice`/`TriggerVoice` Cmds. (`total` is carried on each
fill — constant per stream, idempotent set — to avoid a second config Cmd.)

### Threading the cursors into render — extend `poly_process`

`poly_process` gains a `stream: Option<&StreamCursors>` param, **exactly how
`pool_region` was added** — every non-stream poly node ignores it. The engine
resolves it from `stream_state[id]` alongside `pool_region` from `table_src`
(disjoint `Engine` fields — `stream_state`, `pool`, `arena` — co-borrowable, same
as today's `pool`+`arena` split). The `Kind::StreamPlayer` arm:

```rust
Kind::StreamPlayer => {
    if let State::PolyStreamPlayer(p) = &mut self.state {
        let root = p.root(); // Copy f32, read before the &mut voice loop
        if let (Some(region), Some(sc)) = (pool_region.as_deref(), stream) {
            let cap = region.len() / VOICES;
            if cap > 0 {
                let n = out.len() / VOICES;
                let (mut col, mut ocol) = ([0.0f32; MAX_BLOCK], [0.0f32; MAX_BLOCK]);
                for v in 0..VOICES {
                    if let Some(pitch) = poly_in[0] {
                        for i in 0..n { col[i] = pitch[i * VOICES + v]; }
                    }
                    let hz = col[0].max(1e-6);
                    let rate = hz / mtof(root);              // hz→rate (see below)
                    let ring = &region[v * cap .. (v + 1) * cap];
                    let (fl, fh) = sc.fill[v];
                    p.process_voice(v, ring, fl, fh, sc.total, rate, &mut ocol[..n]);
                    for i in 0..n { out[i * VOICES + v] = ocol[i]; }
                }
            }
        }
        // no region/cursors bound → silence (out already zero-filled by the engine)
    }
}
```

### Read-cursor accessor — net-new

`Engine::stream_read_cursor(node: NodeId, voice: usize) -> Option<u64>` →
`match node.state { State::PolyStreamPlayer(p) => Some(p.read_cursor(voice)), _ => None }`.
Mirrors the existing `Engine::node_output`/`pool_slice` read accessors. The
(Slice-5) prefetch task polls this to trail playback (advance `fill_lo`, target
`fill_hi`).

### Node wiring + rate/root

- `Kind::StreamPlayer` in the `Kind` enum; `State::PolyStreamPlayer(PolyStreamPlayer)`;
  `is_poly == true`, `out_width == VOICES`, `poly_in_count == 1` (pitch), mirroring
  `Kind::PolySamplePlayer`. `Node::trigger_voice` arm → `p.trigger_voice(v)`.
- **`root`** lives in the kernel: add `root: f32` to `PolyStreamPlayer` + `set_root`
  (default e.g. 60), set via the node's `set_param(0)`. The node arm reads
  `p.root()` (Copy) before the voice loop.
- **`hz → rate`:** `rate = hz / mtof(root)` where `mtof(n) = 440 * exp2f((n-69)/12)`
  — copy the exact snippet `PolySamplePlayer::process_voice` already inlines (no
  shared `mtof` helper exists); optionally factor a tiny
  `fn hz_to_rate(hz, root) -> f32` in `sampler.rs` reused by both. (`rate` per
  voice from the Hz tile — de-interleave `poly_in[0]` like the sibling arms.)

## Non-breaking

Additive: new `Cmd` variant (unhandled by old code paths → the match is
exhaustive so all `apply`/serialization sites get the arm), new `Engine` field
(default `[None; NODES]`), new `poly_process` param (ignored by every existing
node — like `pool_region`), new `Kind`/`State` variant. No existing node's
behavior changes. `poly_process`'s new param means every call site + the trait
signature updates, but the value is `None` for non-stream nodes.

## Testing (engine-level, mock prefetch)

- **Streamed render:** create a `Kind::StreamPlayer`, bind a `VOICES*cap` pool
  region, write known PCM into voice 0's sub-ring (`region[0..cap]`), submit
  `Cmd::StreamFill{node, voice:0, fill_lo:0, fill_hi:cap, total}`, set a pitch so
  `rate≈1`, `TriggerVoice(0)`, render → voice-0 output reproduces the samples.
- **Underrun → silence → resume:** fill only a short window (`fill_hi` small) →
  output silence past it and `stream_read_cursor` holds; submit a wider
  `StreamFill` + refill the ring → next render resumes at the held cursor.
- **`stream_read_cursor`** returns `⌊pos⌋` and advances only as samples are read.
- **rate:** `set_param(0, root)` + a pitch one octave up → `rate ≈ 2` (reads every
  other sample).
- **No region/cursors bound → silence, no panic.** Per-voice independence.
- **`Cmd::StreamFill` apply:** lazily creates `stream_state`, sets total + the
  voice's window; out-of-range `voice`/`node` → ignored, no panic.
- Both configs (`-p deluge-audio-graph` ± `--features deluge-dsp-kernels/simd`).

## Constraints (global)

- `no_std`, no heap, no panic on any input (out-of-range voice/node in
  `StreamFill`/`stream_read_cursor`, `cap==0`, unbound region). 32-bit-`usize`
  safe (cursors are `u64`; ring offset `v*cap` and `% cap` bounded by `region.len()`
  — see [[target-32bit-usize-overflow]]).
- VOICES == 8. MIT/Apache-2.0. Scalar node arm (the kernel is scalar).
- `poly_process` param added like `pool_region` (ignored by non-stream nodes).

## Out of scope / deferred

- **Slice 4:** `Sample.stream("path")` Wren binding + host-sim `std::fs` prefetch
  (create the node, alloc the ring region, a sim prefetch loop that decodes into
  the ring + submits `StreamFill` Cmds + reads `stream_read_cursor`).
- **Slice 5:** firmware async-SD prefetch task (resolve extents via blocking FAT
  once, raw async `sd::read_sectors`, decode via the Slice-1 `wav`, 8-window SD
  bandwidth budget / voice-steal).
- Keymap-of-streamed-samples (8 different files); looped streaming; stereo;
  24/8-bit/float PCM.

## Likely task decomposition (for writing-plans)

1. **Engine cursor plumbing:** `Cmd::StreamFill` variant + `StreamCursors` +
   `Engine.stream_state` + the `apply` arm + `Engine::stream_read_cursor`
   accessor. Tests: `StreamFill` sets total+window, out-of-range ignored, cursor
   accessor.
2. **`PolyStreamPlayer` kernel `root` + `Kind::StreamPlayer` node:** add
   `root`/`set_root` to the kernel; `Kind`/`State`/predicates
   (`is_poly`/`out_width`/`poly_in_count`)/`trigger_voice` arm/`set_param(0)=root`;
   extend `poly_process`'s signature with the `stream` param + resolve it in the
   engine render loop; the `Kind::StreamPlayer` render arm (partition rings, rate,
   process_voice). Tests: streamed render, underrun+resume, rate, no-panic.
