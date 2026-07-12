# Sa-1: In-memory Sample Buffer + Player — Design Spec

**Date:** 2026-07-11
**Suite:** Sa (samples) — sub-project Sa-1, the FIRST of the Samples suite (per the
north-star roadmap `2026-07-06-dsp-library-vision-design.md` §3, the `Sa` row).
After the Sy voice suite and the Ef/Fx effects suite (both complete).
**Status:** Approved — ready for implementation plan

## Goal

Play back an in-memory PCM sample. `SampleBuffer.from([…])` uploads raw PCM into
the buffer pool; `Player.new(buffer)` plays it at a controllable pitch (speed ×
semitone transpose), one-shot or looped (with settable loop points), 4-point
Hermite-interpolated, restartable via `trigger`. Mono source, routes through
`Out.patch`.

## Background

The buffer-pool + data-load + read-through-a-buffer infrastructure already exists
(wavetables are the closest precedent, an end-to-end "Wren array → pool → kernel
read at a rate" pipeline):
- **Pool** (`crates/deluge-audio-graph/src/pool.rs`): `Pool<CAP, CHUNK>` fixed
  arena, `alloc(len) -> Option<PoolHandle>` / `free` / `slice` / `slice_mut`,
  first-fit, never panics. Engine owns it (`pool_alloc`/`pool_free`/`pool_slice`/
  `pool_slice_mut`); firmware/sim size `PCAP=90112`, `PCHUNK=2048`.
- **`pool_region` threading** (`engine.rs`): a node bound to `TableSrc::Pooled(h)`
  gets its region resolved to a `pool_region: Option<&mut [f32]>` passed into its
  process fn (how `Wavetable`/`Delay`/reverb read their rings).
- **Wren→Rust bulk-float path**: `SlotApi::get_list_count` / `get_list_element`
  (`slotapi.rs`), proven by `Wavetable.from([…])` (`wavetable_from_impl`,
  `bindings_audio.rs`).
- **Read-through-a-buffer kernels**: `DelayLine::read_hermite` (4-point Hermite,
  `delay.rs`) and the `WtOsc` phase accumulator (`wavetable.rs`).
- **Pooled-source create pattern**: `node_delay_impl` → `alloc_buffer` +
  `new_delay` (`Cmd::NewNode` + `Cmd::BindTable{Pooled}`).

What's **missing** and built here:
- A **raw-PCM upload** (every existing pool producer either zero-fills — effects —
  or runs `build_pyramid_into` — wavetables; nothing copies caller floats verbatim
  as playable PCM).
- A **player node**: a `Kind`/kernel/Wren class that reads a PCM region at a
  playback rate with one-shot/loop/loop-point/trigger semantics.

Device sample origin (SD/flash/`.wav`) is entirely absent and **deferred** (vision
§5 defers SD streaming); the host-memory pool path (Wren array → pool) is the Sa-1
supply route, and works on device and sim identically.

## Scope (Sa-1)

An in-memory `SampleBuffer` (raw-PCM upload) + a mono `Player` node: speed +
semitone pitch, 4-point Hermite read, one-shot / loop with loop points, trigger
restart. All on existing pool/routing infra.

### Explicitly out of scope / deferred

- **Poly sample voices / MIDI-note keymap** — integrating the player with the Sy
  voice layer (per-voice players, note→pitch relative to a root) is Sa-2.
- **SR-aware root pitch** — Sa-1 defines `rate 1.0 = one buffer-sample per
  output-sample` (no sample-rate metadata); root-note pitch is Sa-2.
- **SD streaming** (the device origin) — Sa-3. **Granular** — Sa-4.
- **Per-object buffer refcounting / per-node region free** — Sa-1's buffer region
  lives for the graph and frees on full reset (matches the pool's documented
  node-scoped ownership caveat). Gate-to-stop, reverse playback, crossfade loops.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/upload paths. `libm`
  (`exp2f` for semitones, plus the Hermite arithmetic) — NO std float methods
  (device build). Bounded loops. The pool never panics on exhaustion (returns
  `None` → the buffer/player degrades gracefully, as effects already do).
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** additive — a new kernel, a new `Kind`, a new `upload_pcm` host
  path, new Wren classes. No existing kernel/node/effect/wavetable behavior
  changes.
- **Two registration tables:** the `Player`/`SampleBuffer` foreigns register in
  BOTH `install_methods` (`bindings_audio.rs`) AND `METHODS` (`bindings.rs`), plus
  prelude `foreign` decls / classes. Selector names collision-checked against the
  existing surface during the plan (the Fx-1b/Fx-2 lesson: grep `foreign class`,
  `node_<x>`, `foreign <setter>=` before naming).
- **Test invocation (per-crate, never `--workspace`; both configs; `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## Architecture

### §1 — Raw-PCM upload + `SampleBuffer` (`host.rs` / `bindings_audio.rs` / `prelude.wren`)

- **Host path `upload_pcm`** — mirror the wavetable uploader but copy verbatim (no
  pyramid): the binding reads `n = get_list_count(1)`, `pool_alloc(n)` to reserve
  the region, then writes each `get_list_element(1, i, …)` into
  `pool_slice_mut(handle)[i]`. No large stack scratch (element-by-element, as the
  wavetable path already does). Exact facade signatures (whether via a `Host`
  method or `pool_alloc` + a per-element write accessor already surfaced) resolved
  in the plan after grepping the `audio::`/`Host` facade.
- **Wren `SampleBuffer.from([f32…])`** → a foreign `SampleObj { handle: PoolHandle,
  len: u32 }` (mirror `WtObj`). `pool_alloc` failure → a null/empty buffer
  (graceful, like a failed effect alloc).

### §2 — The `SamplePlayer` kernel (`crates/deluge-dsp-kernels/src/sampler.rs`, new)

```rust
#[derive(Clone, Copy)]
pub struct SamplePlayer {
    speed: f32,       // playback speed multiplier (1.0 = as-recorded)
    semitones: f32,   // transpose in semitones (added on top of speed)
    loop_start: f32,  // sample index
    loop_end: f32,    // sample index (exclusive-ish)
    loop_mode: bool,  // false = one-shot, true = loop
    pos: f32,         // fractional read position (samples)
    playing: bool,    // one-shot gate (false ⇒ output silence)
}
```

`process(&mut self, pcm: &[f32], dt: f32, out: &mut [f32])` — per output sample:
1. `rate = self.speed * libm::exp2f(self.semitones / 12.0)` (block-constant).
2. If `!playing` → `out[i] = 0.0`; continue.
3. **4-point Hermite** read at `pos` (indices `⌊pos⌋−1 … ⌊pos⌋+2`, fractional
   `pos − ⌊pos⌋`): in **loop** mode indices wrap within `[loop_start, loop_end)`;
   in **one-shot** they clamp to `[0, len)`. (Adapt the `DelayLine::read_hermite`
   4-point formula; the wrap/clamp is player-specific, not the ring `rem_euclid`.)
4. `pos += rate`.
   - **one-shot:** `if pos >= len { playing = false }` (len = `pcm.len()`).
   - **loop:** `if pos >= loop_end { pos -= (loop_end - loop_start) }` (guard
     `loop_end > loop_start`; if the loop region is degenerate, fall back to
     one-shot behavior).
5. Setters (`set_speed`/`set_semitones`/`set_loop_start`/`set_loop_end`/
   `set_loop_mode`) with clamps: loop points clamped `>= 0`, `speed`/rate finite.
   `trigger()` → `pos = loop_start` (or 0 for one-shot), `playing = true`.

No panics: all reads are bounds-checked/clamped against `pcm.len()`; an empty
`pcm` → silence.

### §3 — Graph node (`crates/deluge-audio-graph/src/node.rs`)

`Kind::SamplePlayer` — a **pooled source**: `out_width == 1`, `is_poly == false`,
`poly_in_count == 0`; bound to `TableSrc::Pooled(handle)` at create so the engine
passes its PCM region as `pool_region`. `State::SamplePlayer(SamplePlayer)`.
`set_param`: 0=speed, 1=semitones, 2=loop_start, 3=loop_end, 4=loop_mode (0=one-
shot, else loop). `trigger` dispatch: extend `Node::trigger` (the mono
`Cmd::TriggerVoice` path, as `PolySlew`/`Ar` use) to call `SamplePlayer::trigger`.
Process arm (in `process_resolved`, where `pool_region` is available):
`if let (State::SamplePlayer(p), Some(region)) = (&mut self.state, pool_region) {
p.process(region, dt, out.port(0)); }` — match the exact `Wavetable` arm's
`pool_region` handling.

### §4 — Wren surface (`bindings_audio.rs`, `bindings.rs`, `prelude.wren`)

- **`SampleBuffer.from([…])`** (§1) → `SampleObj`.
- **`Player.new(buffer)`** — reads the `SampleObj` foreign arg (its handle + len),
  creates a `Kind::SamplePlayer` node with `Cmd::BindTable{Pooled(handle)}`
  (mirror `node_wavetable_pooled_impl`), defaults `loop_end = len`, speed 1,
  semitones 0, one-shot. Returns the node.
- Instance methods on the returned node: `trigger` (emits `Cmd::TriggerVoice`),
  setters `speed=`, `semitones=`, `loopStart=`, `loopEnd=`, `loop=` (mode:
  truthy→loop). Register the factory + `trigger` + setters in BOTH tables + prelude
  `class Player` (and `class SampleBuffer`).
- **`polyMode_` guard**: `Player.new` (and `SampleBuffer.from`) abort inside a
  Synth block ("sample player not usable in a Synth yet — Sa-2"), mirroring the
  codebase's not-yet-poly guard convention.

## Data Flow

```
SampleBuffer.from([…]) → upload_pcm: pool_alloc(len) + copy list verbatim
    → SampleObj { handle, len }

Player.new(buf) → new_node(Kind::SamplePlayer) + BindTable{Pooled(buf.handle)}
    defaults: speed 1, semitones 0, loop_start 0, loop_end len, one-shot
player.trigger()         → Cmd::TriggerVoice → SamplePlayer.trigger (pos=start, playing)
player.speed = / .loop = → Cmd::SetParam(0..4)

per sample (engine passes pool_region = the PCM):
    rate = speed * 2^(semitones/12)
    out = hermite4(pcm, pos)   [wrap in loop region | clamp one-shot]
    pos += rate; one-shot: stop at len; loop: wrap loop_end→loop_start
→ Out.patch(player)  (mono source → master bus)
```

## Error Handling

`pool_alloc` returns `None` on exhaustion → empty buffer / silent player (no
panic). Kernel reads are clamped/wrapped to `[0, pcm.len())`; empty PCM →
silence. Setter clamps (loop points `>= 0`, `loop_end` effectively `<= len` via
kernel clamp). `libm::exp2f` bounded. `trigger` on an empty/absent region →
silence. No heap, no panics, `no_std`.

## Testing

Both feature configs, per-crate. Oracle-driven.

1. **Hermite exactness** (kernel): at integer `pos` the 4-point Hermite returns the
   exact control-point sample (Hermite interpolates its knots). Feed a known PCM,
   `speed = 1`, one-shot → output equals the buffer sample-for-sample (until end).
2. **Rate** (kernel): `speed = 2.0` reads every other sample (pos 0,2,4,…);
   `semitones = 12` doubles the effective rate (equivalent to speed 2); a
   fractional rate interpolates between samples (bounded, monotone within a ramp).
3. **One-shot stop** (kernel): after `pos` passes `len`, output is silence
   (`playing == false`); `trigger()` restarts from the start.
4. **Loop wrap** (kernel): `loop_mode`, `loop_start`/`loop_end` set → `pos` wraps
   at `loop_end` back to `loop_start`; a long render stays within the loop region
   and repeats it (assert periodicity / that late samples equal the loop content).
5. **Graph node** (node.rs): `Kind::SamplePlayer` `out_width == 1`; `set_param`
   drives speed/semitones/loop; `trigger` restarts; process reads the bound
   `pool_region` and outputs the PCM.
6. **Wren e2e** (`tests/audio_bindings.rs`): `var b = SampleBuffer.from([…known…])`
   + `var p = Player.new(b)` + `Out.patch(p)` + `p.trigger()` renders the buffer
   (finite/bounded/non-silent; matches the known PCM at speed 1); a looped player
   renders continuously non-silent past the buffer length; a one-shot player goes
   silent after its length; `p.speed = 2` / `p.semitones = 12` render (pitch up).
   Using a real pool via the test host. Existing synths/effects/wavetables
   unchanged.

## Success Criteria

- `SampleBuffer.from([…])` stores PCM in the pool; `Player.new(buf)` plays it back
  Hermite-interpolated at `speed × 2^(semitones/12)`, one-shot or looped with loop
  points, `trigger`-restartable.
- `rate 1.0` reproduces the buffer verbatim; one-shot stops at the end; loop
  repeats the loop region.
- Additive: existing wavetable/effect/synth behavior byte-unchanged.
- Both feature configs green, per-crate.
