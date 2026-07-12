# `Sample.stream` Binding + Host-Sim Prefetch — Implementation Plan (Sa-3b Slice 4)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make SD streaming play end-to-end in `cargo deluge sim` — a Wren `Sample.stream(pitch, path)` poly voice source (Layer A) + a host-only `wren-firmware` prefetch task that decodes a WAV into the node's ring and advances the fill cursors (Layer B).

**Architecture:** Layer A (`deluge-wren-core`, `no_std`, unit-tested) = the binding + facade + a new `Host::stream_register` no-op trait method. Layer B (`wren-firmware`, host-only) = a pure unit-tested `plan_window` + the registry + `stream_task` (mirrors `flash_task`) that reuses Slices 1–3 (`wav`, `Cmd::StreamFill`, `Engine::stream_read_cursor`).

**Tech Stack:** `deluge-wren-core` bindings + prelude; `wren-firmware` host-only prefetch (`std::fs` + `Vec` behind `#[cfg(not(target_os="none"))]`). Reuses `deluge_dsp_kernels::wav`, `Cmd::StreamFill`, `Engine::stream_read_cursor`, `poly_record_trigger`.

## Global Constraints

- `deluge-wren-core` stays `no_std`/no-heap. The prefetch registry + `Vec`/`String` + `std::fs` are **`wren-firmware` HOST-ONLY** (`#[cfg(not(target_os="none"))]`; the device build gets no-op stubs). The device build MUST still cross-compile.
- **No panic on any input:** bad path → WAV load fails gracefully → node stays silent; `checked_str` for the path arg (see [[wren-binding-safety]]); `plan_window` is `u64`-only + 32-bit-safe; ring writes `ring[(a as u64 % cap as u64) as usize]` guard `a < pcm.len()` (see [[target-32bit-usize-overflow]]).
- MIT/Apache-2.0. VOICES == 8.
- Test invocation: per-crate, NEVER `--workspace`; both configs where relevant. LSP `armv7a … can't find crate for test` is noise.
- **Testability:** Layer A (Task 1) + `plan_window` (Task 2) are unit-tested; the `stream_task` wiring (Task 3) is compile/cross-build + review — audible playback is user-verified via `cargo deluge sim`.

## Interfaces (produced this slice)

- `Host::stream_register(&mut self, node: deluge_audio_graph::NodeId, handle: deluge_audio_graph::PoolHandle, path: &str)` (default no-op).
- `deluge_wren_core::audio::{new_stream_player(id, handle, pitch, root), stream_register(id, handle, path)}`; `node_stream_impl` + `Node.stream_(pitch, path)` + prelude `Sample.stream(pitch, path)`.
- `wren_firmware::stream::plan_window(read_cursor: u64, ring_cap: u64, total: u64) -> (u64, u64)`.
- `wren_firmware::audio::{stream_register(node, handle, path), stream_read_cursor(node, voice) -> Option<u64>}`.

---

### Task 1: Layer A — `Sample.stream` binding + `Host::stream_register`

**Files:**
- Modify: `crates/deluge-wren-core/src/host.rs` (`Host::stream_register` default no-op)
- Modify: `crates/deluge-wren-core/src/audio.rs` (`new_stream_player`, `stream_register` facades)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_stream_impl` + shim + register)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (METHODS)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`class Sample` gains `stream`, `Node` gains `stream_`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `audio::alloc_buffer`/`alloc_node_id`/`poly_record_trigger`, `arg_input`, `checked_str`, `return_poly_node`, `Kind::StreamPlayer`, `Cmd::{NewNode,BindTable,SetParam}`.
- Produces: `Host::stream_register`, `audio::{new_stream_player, stream_register}`, `Sample.stream`.

- [ ] **Step 1: Write the failing test**

In `crates/deluge-wren-core/tests/audio_bindings.rs` (mirror the `sample_new_*` Cmd-capture tests — find how they build a `Synth` body + assert emitted `Cmd`s via the capture host; and how a capture host can record a side-channel):
```rust
#[test]
fn sample_stream_emits_streamplayer_and_registers() {
    // Inside a Synth, Sample.stream(p, "cello.wav") builds a Kind::StreamPlayer
    // node (NewNode + SetParam{0}=root) and registers the path.
    // Use the same command-capture harness the sample_new_* tests use.
    /* run: Synth.new { |p| Sample.stream(p, "cello.wav") * Env.adsr(0.001,0.5,1,0.2) } … noteOn
       assert a Cmd::NewNode{kind: Kind::StreamPlayer, ..} was emitted,
       assert a Cmd::SetParam{param: 0, ..} (root) for that node,
       assert (via the capture host's stream_register record) the path "cello.wav" was registered. */
}

#[test]
fn sample_stream_outside_synth_aborts() {
    assert!(!run_script_ok("var s = Sample.stream(Osc.saw(110), \"x.wav\")"),
        "Sample.stream aborts outside a Synth (poly-only source)");
}
```
> Copy the exact harness from `sample_new_*`: how it captures Cmds (the `CmdCaptureHost`), how a Synth body is built and rendered/run, and the abort-check style. If the capture host can't yet record `stream_register`, add a minimal recording override to the TEST host (a thread-local/static the test reads) — do NOT change the production `CmdCaptureHost` semantics beyond recording.

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- sample_stream`
Expected: FAIL (`Sample.stream`/`Node.stream_`/`stream_register` undefined).

- [ ] **Step 3: `Host::stream_register` (host.rs)**

After `pool_set` (host.rs:109), add to the `Host` trait:
```rust
    /// Register that `node` (a `Kind::StreamPlayer`) should stream from `path`,
    /// with its ring at pool `handle`. Default no-op (hosts with no filesystem /
    /// prefetch, e.g. tests, ignore it). The firmware host wires it to a prefetch
    /// task; see the Sa-3b slice-4 design.
    fn stream_register(&mut self, node: deluge_audio_graph::NodeId, handle: deluge_audio_graph::PoolHandle, path: &str) {
        let _ = (node, handle, path);
    }
```

- [ ] **Step 4: `audio.rs` facades**

Add to `crates/deluge-wren-core/src/audio.rs` (mirror `new_poly_sample_player`):
```rust
/// Create a `Kind::StreamPlayer` node bound to a `VOICES*cap` ring `handle`,
/// wired to `pitch`, with `root` note (param 0). Streaming data is filled by the
/// host prefetch task (registered separately via `stream_register`).
pub fn new_stream_player(id: u16, handle: Option<deluge_audio_graph::PoolHandle>, pitch: Input, root: f32) {
    if id == NULL_ID { return; }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind: Kind::StreamPlayer,
        args: [pitch, Input::Const(0.0), Input::Const(0.0)],
    });
    if let Some(h) = handle {
        host().audio_cmd(Cmd::BindTable { node: NodeId(id), src: deluge_audio_graph::node::TableSrc::Pooled(h) });
    }
    host().audio_cmd(Cmd::SetParam { node: NodeId(id), param: 0, value: root });
}

/// Register a streamed node+ring with the host's prefetch (no-op host → ignored).
pub fn stream_register(id: u16, handle: Option<deluge_audio_graph::PoolHandle>, path: &str) {
    if let Some(h) = handle {
        host().stream_register(NodeId(id), h, path);
    }
}
```

- [ ] **Step 5: `node_stream_impl` (bindings_audio.rs) + register both tables + prelude**

```rust
/// `Node.stream_(pitch, path)` — a poly streaming sample voice source. Allocates a
/// VOICES*cap ring, creates a `Kind::StreamPlayer`, registers the file with the
/// host prefetch, and records the trigger so `note_on` fans `TriggerVoice`.
pub(crate) fn node_stream_impl<S: SlotApi>(vm: &S) {
    const STREAM_RING_CAP: usize = 8192; // samples per voice (~0.19 s @ 44.1 kHz lookahead)
    let pitch = arg_input(vm, 1);
    let path = checked_str(vm, 2);
    let handle = audio::alloc_buffer(deluge_audio_graph::VOICES * STREAM_RING_CAP);
    let id = audio::alloc_node_id();
    audio::new_stream_player(id, handle, pitch, 60.0); // root C4 (setter deferred)
    audio::stream_register(id, handle, path);
    audio::poly_record_trigger(id);
    unsafe { return_poly_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_stream(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_stream_impl(&vm);
}
```
> Confirm the `VOICES` path (`deluge_audio_graph::VOICES` re-export, as Sy-5e used). Register in BOTH tables: `install_methods` → `method("main","Node",true,"stream_(_,_)", node_stream_impl::<S>)`; `METHODS` (bindings.rs) → `static_method("Node","stream_(_,_)", bindings_audio::node_stream)`. Prelude: `Node` gains `foreign static stream_(pitch, path)`; `class Sample` gains
> ```wren
>   static stream(pitch, path) {
>     if (Node.polyMode_ != 1) Fiber.abort("Sample.stream is a poly voice source — use it inside Synth.new/Synth.mono")
>     return Node.stream_(pitch, path)
>   }
> ```

- [ ] **Step 6: Run tests, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- sample_stream` (+ `--features deluge-dsp-kernels/simd`), then the FULL wren-core suite both configs. Then confirm the crate still cross-builds for the firmware target (or at least `cargo build -p deluge-wren-core`). Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-wren-core/src/{host.rs,audio.rs,bindings_audio.rs,bindings.rs} crates/deluge-wren-core/wren/prelude.wren crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): Sample.stream(pitch, path) — poly stream voice source + Host::stream_register seam"
```

---

### Task 2: Layer B pure core — `plan_window` + registry types

**Files:**
- Create: `wren-firmware/src/stream.rs` (host-gated `plan_window` + registry types + tests)
- Modify: `wren-firmware/src/main.rs` (`mod stream;`)

**Interfaces:**
- Produces: `wren_firmware::stream::plan_window(read_cursor: u64, ring_cap: u64, total: u64) -> (u64, u64)`; `StreamReg` registry type.

- [ ] **Step 1: Write the failing tests**

`wren-firmware/src/stream.rs` (host-gated `#[cfg(all(test, not(target_os = "none")))]` test module):
```rust
#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    const CAP: u64 = 1000;
    #[test]
    fn window_trails_the_cursor_bounded_by_cap() {
        // Mid-stream: window is ≤ CAP wide and contains the cursor.
        let (lo, hi) = plan_window(5000, CAP, 1_000_000);
        assert!(hi - lo <= CAP);
        assert!(lo <= 5000 && 5000 < hi, "cursor is inside the resident window");
    }
    #[test]
    fn window_clamps_to_total_at_end() {
        let (lo, hi) = plan_window(999_990, CAP, 1_000_000);
        assert_eq!(hi, 1_000_000, "never resident past the file end");
        assert!(hi - lo <= CAP);
    }
    #[test]
    fn window_at_start() {
        let (lo, hi) = plan_window(0, CAP, 1_000_000);
        assert_eq!(lo, 0);
        assert!(hi <= CAP && hi > 0);
    }
    #[test]
    fn unbounded_total_uses_cap() {
        let (lo, hi) = plan_window(10_000, CAP, 0); // total 0 = unknown/unbounded
        assert!(hi - lo <= CAP && lo <= 10_000 && 10_000 < hi);
    }
    #[test]
    fn no_panic_on_huge_cursor() {
        let _ = plan_window(u64::MAX, CAP, u64::MAX);
        let _ = plan_window(u64::MAX, 0, 0); // cap 0
    }
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p wren-firmware --lib -- stream::tests` (host target — wren-firmware's host test config; use the project's standard host test invocation for this crate). Expected: FAIL (`plan_window` undefined).

- [ ] **Step 3: Implement `plan_window` + registry types (host-gated)**

`wren-firmware/src/stream.rs`:
```rust
//! Host-only SD-sample streaming prefetch (Sa-3b slice 4). Reads a WAV into RAM
//! and feeds each `StreamPlayer` node's ring window as playback advances. The
//! DEVICE prefetch (async `sd::read_sectors`, no-heap) is slice 5.
#![cfg(not(target_os = "none"))]

use deluge_audio_graph::{NodeId, PoolHandle};

extern crate std;
use std::{string::String, vec::Vec};

/// How far ahead of the read cursor to keep resident (≤ ring_cap).
const LOOKAHEAD: u64 = 6144;

/// Target resident window `[fill_lo, fill_hi)` for a voice whose playback has
/// consumed up to `read_cursor`, given the ring capacity and total sample count
/// (`0` = unknown/unbounded). The window trails the cursor and is ≤ `ring_cap`.
/// Pure, no I/O, `u64`-only (32-bit-`usize` safe).
pub fn plan_window(read_cursor: u64, ring_cap: u64, total: u64) -> (u64, u64) {
    if ring_cap == 0 {
        return (read_cursor, read_cursor);
    }
    let ahead = read_cursor.saturating_add(LOOKAHEAD.min(ring_cap));
    let fill_hi = if total > 0 { ahead.min(total) } else { ahead };
    let width = fill_hi.min(ring_cap);           // window ≤ ring_cap
    let fill_lo = fill_hi.saturating_sub(width);  // trail the cursor
    (fill_lo, fill_hi)
}

/// One registered streamed node: its ring pool handle, source path, and the
/// decoded PCM (loaded lazily on the first prefetch tick).
pub struct StreamReg {
    pub node: NodeId,
    pub handle: PoolHandle,
    pub path: String,
    pub pcm: Vec<f32>,
    pub total: u64,
    pub loaded: bool,
}
```
> `LOOKAHEAD.min(ring_cap)` keeps the read chunk ≤ the ring; `width = fill_hi.min(ring_cap)` keeps the window from exceeding the ring even near a small `total`. Confirm `plan_window` keeps `read_cursor` inside `[fill_lo, fill_hi)` whenever `read_cursor < total` (the tests assert it).

`main.rs`: add `mod stream;` beside `mod audio; mod host;` (line ~59) — host-only is handled by the module's own `#![cfg(not(target_os = "none"))]`, but guard the `mod` too: `#[cfg(not(target_os = "none"))] mod stream;`.

- [ ] **Step 4: Run tests + device cross-build**

Run: `cargo test -p wren-firmware --lib -- stream::tests` (host) — PASS. Then confirm the DEVICE build still compiles: `cargo build -p wren-firmware --target armv7a-none-eabihf` (or the repo's device-build command, e.g. `build-fw`) — the host-only `stream` module is `cfg`-excluded on device, so this must stay clean.

- [ ] **Step 5: Commit**

```bash
git add wren-firmware/src/stream.rs wren-firmware/src/main.rs
git commit -m "feat(firmware): stream::plan_window prefetch window planner (host-only) + StreamReg"
```

---

### Task 3: Layer B wiring — audio wrappers, `FwHost::stream_register`, `stream_task`

**Files:**
- Modify: `wren-firmware/src/audio.rs` (`stream_register`, `stream_read_cursor` wrappers)
- Modify: `wren-firmware/src/host.rs` (`FwHost::stream_register`)
- Modify: `wren-firmware/src/stream.rs` (the registry static + `stream_task`)
- Modify: `wren-firmware/src/main.rs` (spawn `stream_task`, host-only)

**Interfaces:**
- Consumes: `plan_window`/`StreamReg` (Task 2), `deluge_dsp_kernels::wav`, `crate::audio::submit`, `Engine::{pool_slice_mut, stream_read_cursor}`, `Cmd::StreamFill`.
- Produces: the running host prefetch (validated by the user in `cargo deluge sim`).

> **Review-only task** (no automated end-to-end audio test — the sim is interactive). Deliverable: compiles on host, cross-builds on device, logic reviewed against the design.

- [ ] **Step 1: `audio.rs` wrappers (mirror `pool_set`)**

`wren-firmware/src/audio.rs` — add (host-only where they touch `std`; `stream_read_cursor` is device-safe but only used by the host task, so gate the whole prefetch host-only):
```rust
/// Read a StreamPlayer voice's playback read-cursor from the engine. `&self` read
/// (even safer than `pool_set`'s mutable access) — same single-executor SAFETY as
/// `upload_table`.
pub fn stream_read_cursor(node: deluge_audio_graph::NodeId, voice: usize) -> Option<u64> {
    let eng: &Eng = unsafe { (*addr_of_mut!(ENGINE)).assume_init_ref() };
    eng.stream_read_cursor(node, voice)
}
/// Bulk-write a slice of decoded PCM into a pool region (for the prefetch fill).
pub fn pool_write(h: deluge_audio_graph::PoolHandle, index: usize, value: f32) {
    // (or reuse pool_set; a per-sample write is fine for the sim)
    pool_set(h, index, value);
}
```

- [ ] **Step 2: `FwHost::stream_register` + registry + `stream_task` (host-only)**

`wren-firmware/src/host.rs`:
```rust
    fn stream_register(&mut self, node: deluge_audio_graph::NodeId, handle: deluge_audio_graph::PoolHandle, path: &str) {
        #[cfg(not(target_os = "none"))]
        crate::stream::register(node, handle, path);
        #[cfg(target_os = "none")]
        let _ = (node, handle, path); // device prefetch is slice 5
    }
```
`wren-firmware/src/stream.rs` — the registry static (mirror `CMD_RING`'s `Mutex<RefCell<...>>`) + `register` + the task:
```rust
static REGISTRY: Mutex<CriticalSectionRawMutex, RefCell<Vec<StreamReg>>> = Mutex::new(RefCell::new(Vec::new()));

pub fn register(node: NodeId, handle: PoolHandle, path: &str) {
    REGISTRY.lock(|r| r.borrow_mut().push(StreamReg {
        node, handle, path: String::from(path), pcm: Vec::new(), total: 0, loaded: false,
    }));
}

#[embassy_executor::task]
pub async fn stream_task() {
    loop {
        Timer::after(Duration::from_millis(20)).await;
        REGISTRY.lock(|r| {
            for reg in r.borrow_mut().iter_mut() {
                // lazy load+decode the whole WAV on first tick
                if !reg.loaded {
                    if let Ok(bytes) = std::fs::read(/* sim-sd root */ join_sim_sd(&reg.path)) {
                        if let Ok(info) = deluge_dsp_kernels::wav::parse(&bytes) {
                            let n_samples = info.data_len / 2;
                            reg.pcm.resize(n_samples, 0.0);
                            deluge_dsp_kernels::wav::decode_i16_le(&bytes[info.data_offset..info.data_offset + info.data_len], &mut reg.pcm);
                            reg.total = n_samples as u64;
                        }
                    }
                    reg.loaded = true;
                }
                let cap = /* ring cap per voice */ region_cap(reg.handle);
                for v in 0..deluge_audio_graph::VOICES {
                    let rc = crate::audio::stream_read_cursor(reg.node, v).unwrap_or(0);
                    let (lo, hi) = plan_window(rc, cap, reg.total);
                    // write pcm[lo..hi) into voice v's sub-ring (region[v*cap..]) at (a % cap)
                    for a in lo..hi {
                        let val = reg.pcm.get(a as usize).copied().unwrap_or(0.0);
                        let idx = (v as u64 * cap + (a % cap)) as usize; // voice sub-ring + ring offset
                        crate::audio::pool_write(reg.handle, idx, val);
                    }
                    crate::audio::submit(deluge_audio_graph::Cmd::StreamFill {
                        node: reg.node, voice: v as u8, fill_lo: lo, fill_hi: hi, total: reg.total,
                    });
                }
            }
        });
    }
}
```
> This is the review-only wiring — adapt to the crate's actual imports (`Mutex`/`CriticalSectionRawMutex`/`Timer`/`Duration` as `audio.rs`/`main.rs` use them; a `join_sim_sd`/sim-sd-root helper mirroring `deluge-sdk`'s `sim_sd_root`; `region_cap(handle)` = the handle's `len / VOICES`, from `PoolHandle.len`). The per-voice full-window rewrite each tick is O(cap) — acceptable for the sim; the device (slice 5) will write only the newly-exposed `[old_fill_hi, hi)` delta. Confirm the `assume_init_ref`/`assume_init_mut` access doesn't overlap `audio_task`'s per-block borrow (it runs on the same cooperative executor; `stream_task`'s body is synchronous between `.await`s — same argument as `audio.rs`'s `## Concurrency`).

`main.rs`: `#[cfg(not(target_os = "none"))] spawner.spawn(crate::stream::stream_task().unwrap());` beside the other spawns.

- [ ] **Step 3: Build host + cross-build device**

Run: `cargo build -p wren-firmware` (host) — compiles. `cargo build -p wren-firmware --target armv7a-none-eabihf` (device) — compiles (host-only prefetch `cfg`-excluded; `FwHost::stream_register` device arm is a no-op). Run `cargo test -p wren-firmware --lib` (host) — `plan_window` tests still green. Fix any import/borrow issues.

- [ ] **Step 4: Commit**

```bash
git add wren-firmware/src/{audio.rs,host.rs,stream.rs,main.rs}
git commit -m "feat(firmware): host-only stream_task prefetch — WAV->ring fill + StreamFill (sim-playable)"
```

---

## Self-Review

**Spec coverage:** Task 1 = Layer A (binding + `new_stream_player`/`stream_register` facades + `Host::stream_register` no-op + prelude `Sample.stream`, Cmd-capture tested + abort-outside-Synth). Task 2 = Layer B pure `plan_window` + `StreamReg` (unit-tested). Task 3 = Layer B wiring (`audio` wrappers, `FwHost::stream_register`, `stream_task`, spawn) — review-only, cross-builds device. The spec's testability split (A + `plan_window` tested; task wiring compile+review, sim-audible by the user) is honored.

**Placeholder scan:** Task 1 + `plan_window` (Task 2) are verbatim. Task 3's `stream_task`/registry is explicitly review-level with the imports/helpers to adapt named (it's the compile+review deliverable, not an automated test) — this is the agreed testability boundary, not a hidden gap.

**Type consistency:** `Host::stream_register(node: NodeId, handle: PoolHandle, path: &str)`, `audio::new_stream_player(id, handle, pitch, root)`/`stream_register(id, handle, path)`, `plan_window(u64,u64,u64)->(u64,u64)`, `Cmd::StreamFill{node,voice,fill_lo,fill_hi,total}` (Slice 3), `Kind::StreamPlayer` (Slice 3) — consistent across layers. The binding mirrors `node_polysampleplayer_impl` + `new_poly_sample_player`; the task mirrors `flash_task` + `audio::pool_set`.
