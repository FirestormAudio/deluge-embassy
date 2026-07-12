# `Kind::StreamPlayer` Node + Engine Cursor Plumbing — Implementation Plan (Sa-3b Slice 3)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire the `PolyStreamPlayer` kernel (Slice 2) into the graph — a `Kind::StreamPlayer` node reading 8 per-voice rings from one pool region, with engine-owned fill cursors updated by a new `Cmd::StreamFill`, per-voice `rate` from the pitch tile, and a `read_cursor` accessor for the (later) prefetch.

**Architecture:** One cohesive integration across 4 crates' files — it only compiles as a unit, so it is one task. Rings = one `VOICES*cap` pool region (existing `BindTable`/`pool_region` path, unchanged). Cursors = a new engine-owned `stream_state` array (NOT in `Node`/`State`), updated by `Cmd::StreamFill`. `poly_process` gains a `stream` param (ignored by non-stream nodes, like `pool_region`). Pitch→rate math lives in the kernel (`rate_for`) so the graph crate needs no `libm`.

**Tech Stack:** `deluge-dsp-kernels` (kernel `root`/`rate_for`), `deluge-audio-graph` (`Cmd`, `Engine`, `Node`/`poly_process`, a new `stream` module). VOICES=8.

## Global Constraints

- `no_std`, no heap, **no panic on any input** — out-of-range `voice`/`node` in `Cmd::StreamFill`/`stream_read_cursor` (bounds-guard `node.0 as usize < NODES`, `voice < VOICES`), `cap == 0`, unbound region/cursors → silence. Cursors are `u64`; ring offsets `v*cap`/`% cap` are bounded by `region.len()` (reduce in-range before `as usize` — see [[target-32bit-usize-overflow]]).
- VOICES == 8. MIT/Apache-2.0. Scalar. Purely additive (a new `Cmd` variant, a new `Engine` field, a new `poly_process` param ignored by every existing node, a new `Kind`/`State` variant) — no existing node behavior changes.
- Test invocation: per-crate, NEVER `--workspace`; both configs (`-p deluge-dsp-kernels` ± `--features simd`; `-p deluge-audio-graph` ± `--features deluge-dsp-kernels/simd`). LSP `armv7a … can't find crate for test` is noise.

## Interfaces (produced this slice; consumed by Slices 4–5)

- Kernel: `PolyStreamPlayer::set_root(&mut self, note: f32)`, `PolyStreamPlayer::root(&self) -> f32`, `PolyStreamPlayer::rate_for(&self, hz: f32) -> f32`.
- Graph: `pub struct StreamCursors { pub total: u64, pub fill: [(u64, u64); VOICES] }` (in a new `crate::stream` module); `Cmd::StreamFill { node: NodeId, voice: u8, fill_lo: u64, fill_hi: u64, total: u64 }`; `Kind::StreamPlayer`; `Engine::stream_read_cursor(&self, node: NodeId, voice: usize) -> Option<u64>`.

---

### Task 1: `Kind::StreamPlayer` node + engine cursor plumbing (the whole slice)

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/sampler.rs` (`PolyStreamPlayer`: `root` + `set_root`/`root`/`rate_for`)
- Create: `crates/deluge-audio-graph/src/stream.rs` (`StreamCursors`); Modify `lib.rs` (`pub mod stream;`)
- Modify: `crates/deluge-audio-graph/src/cmd.rs` (`Cmd::StreamFill`)
- Modify: `crates/deluge-audio-graph/src/node.rs` (`Kind::StreamPlayer`, `State`, predicates, `trigger_voice`, `set_param`, `poly_process` signature + arm)
- Modify: `crates/deluge-audio-graph/src/engine.rs` (`stream_state` field + init, `apply` arm, `poly_process` call site, `stream_read_cursor`)
- Test: `engine.rs` test module (engine-level e2e with a mock prefetch)

**Interfaces:**
- Consumes: `PolyStreamPlayer` (Slice 2: `new`/`trigger_voice`/`read_cursor`/`is_playing`/`process_voice`), `VOICES`, the existing `Cmd`/`apply`/`poly_process`/`pool_region` machinery.
- Produces: the Interfaces listed above.

- [ ] **Step 1: Write the failing engine test**

Add to `crates/deluge-audio-graph/src/engine.rs`'s test module (mirror an existing `Kind::PolySamplePlayer`/pool test's Engine setup — `Engine::<32,64,128,8,PCAP,PCHUNK>::new(sr)`, `pool_alloc`, `apply(Cmd::NewNode…)`, `apply(Cmd::BindTable…)`, render):

```rust
#[test]
fn stream_player_renders_from_mock_prefetch() {
    // Small concrete engine (match the existing test alias in this module).
    let mut e = test_engine(); // helper the module already uses, or inline Engine::new
    let cap = 64usize;
    let node = NodeId(0);
    // ring region for VOICES sub-rings
    let h = e.pool_alloc(VOICES * cap).unwrap();
    e.apply(Cmd::NewNode { node, kind: Kind::StreamPlayer, args: [Input::Const(0.0); MAX_ARGS] });
    e.apply(Cmd::BindTable { node, src: crate::node::TableSrc::Pooled(h) });
    e.apply(Cmd::SetParam { node, param: 0, value: 60.0 }); // root = 60
    // mock prefetch: write known PCM into voice 0's sub-ring [0..cap), samples = index.
    {
        let region = e.pool_slice_mut(h);
        for a in 0..cap { region[a] = a as f32; } // voice 0 sub-ring = region[0..cap]
    }
    e.apply(Cmd::StreamFill { node, voice: 0, fill_lo: 0, fill_hi: cap as u64, total: cap as u64 });
    e.apply(Cmd::TriggerVoice { node, voice: 0 });
    // drive voice 0 at root pitch (hz == mtof(60)) so rate == 1.0.
    // feed the pitch tile: set the node's poly pitch input to a PolyCtrl/Const carrying mtof(60).
    // (mirror how the existing PolySamplePlayer engine test supplies poly_in[0].)
    /* … supply pitch = mtof(60) via the same mechanism the PolySamplePlayer test uses … */
    let mut out = [StereoFrame::default(); 32];
    e.render(&mut out); // or the module's render entry
    // voice 0 should be reading samples 0,1,2,... (rate 1). Assert non-silent + the
    // read_cursor advanced.
    assert!(e.stream_read_cursor(node, 0).unwrap() > 0, "read cursor advanced");
    // underrun: a fresh voice with a short window renders silence + holds.
    let node2 = NodeId(1);
    let h2 = e.pool_alloc(VOICES * cap).unwrap();
    e.apply(Cmd::NewNode { node: node2, kind: Kind::StreamPlayer, args: [Input::Const(0.0); MAX_ARGS] });
    e.apply(Cmd::BindTable { node: node2, src: crate::node::TableSrc::Pooled(h2) });
    e.apply(Cmd::SetParam { node: node2, param: 0, value: 60.0 });
    e.apply(Cmd::StreamFill { node: node2, voice: 0, fill_lo: 0, fill_hi: 2, total: 1000 }); // only 2 samples
    e.apply(Cmd::TriggerVoice { node: node2, voice: 0 });
    // (render node2 into a bus / assert its output is silence, cursor held small)
    assert!(e.stream_read_cursor(node2, 0).unwrap() <= 2, "held under underrun");
    // out-of-range StreamFill / accessor → no panic
    e.apply(Cmd::StreamFill { node: NodeId(999), voice: 99, fill_lo: 0, fill_hi: 0, total: 0 });
    assert!(e.stream_read_cursor(NodeId(999), 0).is_none());
    assert!(e.stream_read_cursor(node, 99).is_none());
}
```
> This is a SKETCH — adapt the pitch-supply + render-entry to match the EXACT idioms of the existing `Kind::PolySamplePlayer` engine test in this file (find it: it binds a pool region, feeds a poly pitch input at `mtof(60)`, triggers a voice, and renders). Copy its harness verbatim (the `test_engine()`/`Engine::new` alias, how it patches the pitch input, the render call, how it reads a node's output). Keep the three assertions (cursor advanced on a full window; cursor held under underrun; no-panic on out-of-range).

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- stream_player_renders`
Expected: FAIL to compile (`Kind::StreamPlayer`, `Cmd::StreamFill`, `stream_read_cursor` undefined).

- [ ] **Step 3: Kernel — `root` + `rate_for` (`sampler.rs`)**

Extend `PolyStreamPlayer` (Slice 2) with a root note + pitch→rate:
```rust
// add to the struct:
//   root: f32,   (default 60.0)
// in new():  root: 60.0,
    pub fn set_root(&mut self, note: f32) { self.root = note; }
    pub fn root(&self) -> f32 { self.root }
    /// samples-advanced per output sample for a voice at `hz` (root plays at rate 1).
    pub fn rate_for(&self, hz: f32) -> f32 {
        let root_hz = 440.0 * libm::exp2f((self.root - 69.0) / 12.0);
        (hz.max(1e-6)) / root_hz
    }
```

- [ ] **Step 4: Graph — `StreamCursors`, `Cmd::StreamFill`, engine state/apply/accessor**

`crates/deluge-audio-graph/src/stream.rs` (new; `lib.rs`: `pub mod stream;`):
```rust
//! Engine-owned per-`StreamPlayer`-node fill cursors (produced by the prefetch
//! task, consumed at render). Kept out of `Node`/`State` (which are `Copy` and
//! hold kernel state).
use deluge_dsp_kernels::poly::VOICES;

#[derive(Clone, Copy)]
pub struct StreamCursors {
    pub total: u64,
    pub fill: [(u64, u64); VOICES], // per-voice resident window [fill_lo, fill_hi)
}
impl StreamCursors {
    pub fn new() -> Self { StreamCursors { total: 0, fill: [(0, 0); VOICES] } }
}
impl Default for StreamCursors { fn default() -> Self { StreamCursors::new() } }
```
> Confirm the `VOICES` path (`deluge_dsp_kernels::poly::VOICES` vs a crate re-export — this crate may re-export it, e.g. `deluge_audio_graph::VOICES`; use whatever the existing poly code imports).

`crates/deluge-audio-graph/src/cmd.rs` — add to the `Cmd` enum (it is `Copy`; `u64` fields keep it `Copy`):
```rust
    StreamFill { node: NodeId, voice: u8, fill_lo: u64, fill_hi: u64, total: u64 },
```

`crates/deluge-audio-graph/src/engine.rs`:
- Field (beside `writes`): `stream_state: [Option<crate::stream::StreamCursors>; NODES],` and init `stream_state: [None; NODES],` in `new()`.
- `apply` arm (beside `Cmd::TriggerVoice`):
```rust
    Cmd::StreamFill { node, voice, fill_lo, fill_hi, total } => {
        let idx = node.0 as usize;
        if idx < NODES && (voice as usize) < deluge_dsp_kernels::poly::VOICES {
            let sc = self.stream_state[idx].get_or_insert_with(crate::stream::StreamCursors::new);
            sc.total = total;
            sc.fill[voice as usize] = (fill_lo, fill_hi);
        }
    }
```
- Also clear `stream_state[idx]` on `Cmd::Free { node }` and `Cmd::Reset` (find those arms; set the slot(s) to `None` so a freed/reset node doesn't keep stale cursors).
- Accessor (beside `pool_slice`/`node_output`):
```rust
    pub fn stream_read_cursor(&self, node: NodeId, voice: usize) -> Option<u64> {
        let n = self.arena.node(node)?;
        match n.stream_read_cursor(voice) { c => c } // see Node method below
    }
```
> The `Node` needs a small getter into its `State::PolyStreamPlayer`. Add `Node::stream_read_cursor(&self, voice: usize) -> Option<u64>` → `match &self.state { State::PolyStreamPlayer(p) => Some(p.read_cursor(voice)), _ => None }`. (Keeps `State` access inside `node.rs`.)

- [ ] **Step 5: Graph — `Kind::StreamPlayer` node + `poly_process` `stream` param + arm**

`crates/deluge-audio-graph/src/node.rs`:
- `Kind::StreamPlayer` enum variant; `State::PolyStreamPlayer(PolyStreamPlayer)` + `Node::new` constructor arm (`PolyStreamPlayer::new()`); add `StreamPlayer` to the `out_width => VOICES`, `is_poly` (true), `poly_in_count => 1` arms — ALONGSIDE `PolySamplePlayer` (mirror it).
- `Node::trigger_voice`: arm `State::PolyStreamPlayer(p) => p.trigger_voice(v),`.
- `Node::set_param`: arm `State::PolyStreamPlayer(p) if param == 0 => p.set_root(value),`.
- `Node::stream_read_cursor` getter (from Step 4).
- **`poly_process` signature** (node.rs:908) — add a LAST param `stream: Option<&crate::stream::StreamCursors>`. Every existing arm ignores it (like `pool_region`). Update the fn signature + doc.
- The `Kind::StreamPlayer` `poly_process` arm (mirror `Kind::PolySamplePlayer`, but slice the region into per-voice rings + use `stream`):
```rust
Kind::StreamPlayer => {
    if let State::PolyStreamPlayer(p) = &mut self.state {
        let region = pool_region.as_deref();
        if let (Some(region), Some(sc)) = (region, stream) {
            let cap = region.len() / VOICES;
            if cap > 0 {
                let n = out.len() / VOICES;
                let mut col = [0.0f32; MAX_BLOCK];
                let mut ocol = [0.0f32; MAX_BLOCK];
                for v in 0..VOICES {
                    if let Some(pitch) = poly_in[0] {
                        for i in 0..n { col[i] = pitch[i * VOICES + v]; }
                    }
                    let rate = p.rate_for(col[0]); // hz of this voice's tile
                    let ring = &region[v * cap..(v + 1) * cap];
                    let (fl, fh) = sc.fill[v];
                    p.process_voice(v, ring, fl, fh, sc.total, rate, &mut ocol[..n]);
                    for i in 0..n { out[i * VOICES + v] = ocol[i]; }
                }
            }
        }
        // unbound region/cursors → out stays silent (engine zero-filled it)
    }
}
```
> `p.rate_for(col[0])` reads the voice's Hz then `p.process_voice(...)` — `rate_for` is `&self`, `process_voice` is `&mut self`; sequential borrows in the loop are fine. `col[0]` is the block's first Hz sample for that voice (rate is per-block here — matches `PolySamplePlayer`'s latch-on-first-sample model closely enough for Slice 3; a per-sample rate is deferred).

`crates/deluge-audio-graph/src/engine.rs` — the `poly_process` call site (engine.rs:262): resolve `stream` from `stream_state` (a disjoint field from `arena`/`pool`, co-borrowable like `pool_region`) and pass it:
```rust
    let stream = if (id.0 as usize) < NODES { self.stream_state[id.0 as usize].as_ref() } else { None };
    // … existing pool_region resolution …
    if let Some(n) = self.arena.node_mut(id) {
        n.poly_process(&ins, poly_in, self.dt, out, pool_region, stream);
    }
```
> Read `stream` (borrows `self.stream_state`) BEFORE the `self.arena.node_mut` borrow, mirroring how `pool_region` (borrows `self.pool`) is read before `arena`. Three disjoint `Engine` field borrows — allowed.

- [ ] **Step 6: Run tests, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- stream_` (kernel unchanged behavior + new root/rate_for compile), then `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- stream_player_renders` (+ `--features deluge-dsp-kernels/simd`), then the FULL graph crate both configs. Expected: PASS — every existing poly node still renders (the new `poly_process` param is `None` for them; the new `Cmd`/`State`/`Kind` are additive).

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-dsp-kernels/src/sampler.rs crates/deluge-audio-graph/src/{stream.rs,lib.rs,cmd.rs,node.rs,engine.rs}
git commit -m "feat(graph): Kind::StreamPlayer node + Cmd::StreamFill + engine stream cursors + read-cursor accessor"
```

---

## Self-Review

**Spec coverage:** the spec's design maps to this one cohesive task — ring partitioning (option a, existing `pool_region` path), engine-owned `StreamCursors` + `Cmd::StreamFill` + `apply` arm (+ Free/Reset cleanup), `poly_process` `stream` param (ignored by non-stream nodes), `Kind::StreamPlayer` arm with per-voice ring slice + `rate_for` + `process_voice`, and `Engine::stream_read_cursor`. Pitch→rate in the kernel (`rate_for`) so the graph crate needs no `libm`. Deferred (binding, prefetch, keymap, loop) is out of scope.

**Placeholder scan:** The kernel/type/Cmd/apply/accessor/arm code is verbatim. The engine TEST is explicitly a SKETCH to adapt to the existing `Kind::PolySamplePlayer` engine test's exact harness (pitch supply + render entry + node-output read) — because inventing those idioms risks divergence from this file's real test setup; the three assertions to keep are named.

**Type consistency:** `StreamCursors { total: u64, fill: [(u64,u64); VOICES] }`, `Cmd::StreamFill { node, voice: u8, fill_lo/fill_hi/total: u64 }`, `poly_process(…, stream: Option<&StreamCursors>)`, `stream_read_cursor(node, voice) -> Option<u64>`, and kernel `set_root`/`root`/`rate_for` are consistent across the Interfaces block, the engine plumbing, and the node arm. `Kind::StreamPlayer` mirrors `Kind::PolySamplePlayer` in every predicate/dispatch arm.
