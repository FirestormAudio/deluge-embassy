# `Sample.stream` Binding + Host-Sim Prefetch — Design Spec (Sa-3b Slice 4)

**Date:** 2026-07-12
**Status:** Approved (design), pending implementation plan
**Sub-project:** Sa-3b (SD sample streaming) — Slice 4 of 5. Follows slices 1–3 (WAV decoder, `PolyStreamPlayer` kernel, `Kind::StreamPlayer` node), all on main. **The first end-to-end-playable slice.**

## Context

Slices 1–3 built the decoder, the streaming kernel, and the graph node + engine
cursor plumbing (`Cmd::StreamFill`, `Engine::stream_read_cursor`). THIS slice ties
the chain together: a Wren `Sample.stream(pitch, path)` factory (a poly voice
source, used inside `Synth.new{}` like `Sample.new`) + a host-side **prefetch
task** that reads a WAV, decodes it (Slice-1 `wav`), fills the node's ring, and
advances the fill cursors — so streaming actually PLAYS in `cargo deluge sim`.
See [[sa-suite-samples]], [[wren-binding-safety]], [[prefer-neon-simd]].

**Key facts (from the audit):** `cargo deluge sim` runs **`wren-firmware` compiled
for host** (`#![cfg_attr(target_os="none", no_std)]` → full `std` on host). The
host `Sd::read` is whole-file `std::fs::read`. So the sim prefetch reads the whole
WAV into RAM once and feeds the ring in CHUNKS (exercising windowing/underrun
without real SD streaming). The prefetch task mirrors the existing `flash_task`
(a spawned async task doing blocking file I/O).

## Two layers

### Layer A — `deluge-wren-core` (platform-agnostic, `no_std`, unit-testable)

- **`Sample.stream(pitch, path)`** — prelude `class Sample` static, mirroring
  `Sample.new`'s poly-mode guard (abort if `Node.polyMode_ != 1`); calls a new
  `foreign static Node.stream_(pitch, path)`.
- **`node_stream_impl<S>(vm)`** (bindings_audio.rs, mirrors `node_polysampleplayer_impl`):
  `pitch = arg_input(vm, 1)`; `path = checked_str(vm, 2)` (the wren-binding-safety
  guard); `handle = audio::alloc_buffer(VOICES * STREAM_RING_CAP)`;
  `id = audio::alloc_node_id()`; `audio::new_stream_player(id, handle, pitch, root=60.0)`;
  `audio::stream_register(id, handle, path)`; `audio::poly_record_trigger(id)`
  (wires it into the `VoiceAllocator` so note-on fans `TriggerVoice`);
  `return_poly_node(vm, id)`.
- **`audio::new_stream_player(id, handle: Option<PoolHandle>, pitch: Input, root: f32)`**
  facade (mirrors `new_poly_sample_player`): `Cmd::NewNode{Kind::StreamPlayer,
  [pitch, Const, Const]}` + (if `Some(h)`) `Cmd::BindTable{Pooled(h)}` +
  `Cmd::SetParam{param:0, value:root}`.
- **`Host::stream_register(&mut self, node: NodeId, handle: PoolHandle, path: &str)`**
  — a NEW `Host` trait method, **default no-op** (like `alloc_buffer`) — the
  registration handoff (`Host` has no path notion today). `audio::stream_register`
  is the facade → `host().stream_register(...)`.
- Register `Node.stream_(_,_)` in BOTH tables (`install_methods` + `METHODS`) +
  the extern-C shim; prelude `class Sample { static stream(...) }` + `foreign
  static stream_`.
- **`STREAM_RING_CAP`** — a constant samples-per-voice ring size (e.g. 8192 ≈ 0.19 s
  at 44.1 kHz of lookahead per voice; `VOICES * 8192 * 4B = 256 KB` — sits in the
  pool budget; the plan picks/justifies the exact value against `PCAP`).
- **root:** fixed 60 (C4) this slice — the sample plays at rate 1 for note 60;
  pitching still works (`rate = hz/mtof(60)`). A `root=` setter is deferred.
- **Testable** (Cmd-capture host): `Sample.stream(p, "x.wav")` inside a `Synth`
  emits `NewNode{StreamPlayer}` + `SetParam{0,60}` (+ `BindTable` only when a real
  pool binds the handle) and calls `stream_register` (a capture host records the
  path). `poly_record_trigger` registers the id (so a subsequent note-on would
  `TriggerVoice` it).

### Layer B — `wren-firmware` (host-only prefetch)

New module `wren-firmware/src/stream.rs`, all host-gated (`#[cfg(not(target_os="none"))]`;
device build gets a no-op / Slice-5 stub):

- **Pure prefetch-planning core (unit-testable):**
  `fn plan_window(read_cursor: u64, ring_cap: u64, total: u64) -> (u64, u64)` —
  the target resident window `[fill_lo, fill_hi)`: `fill_hi = min(total, read_cursor
  + LOOKAHEAD)` (or `read_cursor + ring_cap` if `total==0`), `fill_lo = fill_hi -
  min(fill_hi, ring_cap)` (keep the window ≤ `ring_cap`, trailing the cursor).
  Pure, no I/O — unit-tested (windowing, clamp to `total`, cursor near 0 / near
  end, `ring_cap`-bounded, `total==0` unbounded, no-panic on huge cursors — 32-bit
  `u64` safe).
- **Registry:** a `Mutex<RefCell<Vec<StreamReg>>>` static (same style as
  `CMD_RING`), `StreamReg { node: NodeId, handle: PoolHandle, path: String,
  pcm: Vec<f32>, total: u64, loaded: bool }`. `crate::audio::stream_register(node,
  handle, path)` pushes an entry (unloaded).
- **`crate::audio::stream_read_cursor(node, voice) -> Option<u64>`** — the
  `assume_init_mut` wrapper mirroring `pool_set`/`upload_table`, calling
  `eng.stream_read_cursor(node, voice)` (a `&self` read — even safer than the
  existing mutable accessors).
- **`FwHost::stream_register`** forwards to `crate::audio::stream_register` (host);
  device build no-ops.
- **`stream_task` (host-only spawned, `#[cfg(not(target_os="none"))]`):** a
  `flash_task`-style loop (`Timer::after(N ms).await`): for each registry entry,
  lazily load+decode the WAV on first tick (`std::fs::read(path)` → `wav::parse` +
  `wav::decode_i16_le` → `pcm`, `total`); then per voice: `rc =
  audio::stream_read_cursor(node, voice)`; `(lo, hi) = plan_window(rc, cap, total)`;
  copy the newly-exposed `pcm[..]` samples into the ring region
  (`eng.pool_slice_mut(handle)[a % cap] = pcm[a]` for the new range, per voice's
  sub-ring `region[v*cap..]`); `crate::audio::submit(Cmd::StreamFill{node, voice,
  fill_lo:lo, fill_hi:hi, total})`. Spawned alongside the other tasks in
  `main.rs`, host-only.

## Testability (honest — as agreed)

- **Layer A:** fully unit-tested (Cmd-capture: the binding emits the right Cmds +
  registers).
- **Layer B pure core (`plan_window`):** fully unit-tested.
- **Layer B async `stream_task` + registry + `std::fs`/engine wiring:**
  compile/cross-compile + review ONLY. **The audible "stream a WAV and hear it"
  validation is done by the USER running `cargo deluge sim`** (put a WAV in
  `./sim-sd`, type `Out.patch(Synth.new{|p| Sample.stream(p, "cello.wav") * Env.ar(...)})`,
  `noteOn`). This slice's automated coverage stops at "correct Cmds + correct
  window planning"; end-to-end playback is user-verified. (Same class as Slice 5's
  device code.)

## Non-breaking

Additive: a new `Host` trait method (default no-op → every existing host
unaffected), a new binding/facade/prelude method, a new host-only firmware module
+ task. `Cmd::StreamFill`/`Engine::stream_read_cursor`/`Kind::StreamPlayer` (Slice
3) unchanged. No existing behavior changes; existing wren-core + firmware builds
stay green (both host + `armv7a-none-eabihf` cross-build).

## Constraints (global)

- `deluge-wren-core` stays `no_std`/no-heap; the prefetch registry + `Vec`/`String`
  + `std::fs` are `wren-firmware` HOST-ONLY (`#[cfg(not(target_os="none"))]`).
- No panic on any input (bad path → the WAV load fails gracefully → the node stays
  silent/unbound; out-of-range cursors; `plan_window` 32-bit-`u64` safe — see
  [[target-32bit-usize-overflow]]). `checked_str` for the path arg.
- The ring copy uses `pcm[a]` guarded by `a < pcm.len()` and writes `ring[(a as u64
  % cap as u64) as usize]` (u64 modulo before `as usize`).
- MIT/Apache-2.0. VOICES == 8. The device build must still cross-compile
  (`cargo build -p wren-firmware --target armv7a-none-eabihf --no-default-features`
  or equivalent) — the host-only code is behind `cfg`.

## Out of scope / deferred (Slice 5 + later)

- **Slice 5:** the DEVICE prefetch — resolve file extents via the blocking FAT
  once, then raw async `sd::read_sectors` (bypassing the FAT `block_on`), no-heap
  ring/registry, the 8-window SD-bandwidth budget / voice-steal. (Layer B's pure
  `plan_window` + the `StreamFill`/`stream_read_cursor` seam are reused; only the
  I/O source + storage change.)
- A `root=` setter for streamed samples; keymap-of-streamed-files; looped
  streaming; stereo; 24/8-bit/float PCM.

## Likely task decomposition (for writing-plans)

1. **Layer A (wren-core):** `Host::stream_register` trait method (default no-op) +
   `audio::{new_stream_player, stream_register}` facades + `node_stream_impl` +
   both registration tables + prelude `Sample.stream`. Cmd-capture test (emits
   NewNode/SetParam/register; poly_record_trigger id registered). Must cross-build.
2. **Layer B pure core:** `wren-firmware/src/stream.rs` `plan_window` (host-gated)
   + unit tests (windowing/clamp/underrun-trail/`total==0`/no-panic). (The registry
   types compile.)
3. **Layer B wiring:** `audio::{stream_register, stream_read_cursor}` wrappers,
   `FwHost::stream_register`, the `stream_task` (host-only spawn + lazy WAV
   load/decode + ring fill + `StreamFill` submit). Host build compiles + runs;
   device cross-build clean (host-only behind `cfg`). Review only (user runs the
   sim to hear it).
