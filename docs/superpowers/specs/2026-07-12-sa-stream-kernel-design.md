# Streaming Ring + `PolyStreamPlayer` Kernel — Design Spec (Sa-3b Slice 2)

**Date:** 2026-07-12
**Status:** Approved (design), pending implementation plan
**Sub-project:** Sa-3b (SD sample streaming) — Slice 2 of 5. Follows Slice 1 (WAV decoder, merged 645557e).

## Context

Sa-3b streams samples too large for RAM from the SD card ([[sa-suite-samples]]).
Slice 1 built the WAV decoder. THIS slice builds the **playback kernel**: a poly
(`VOICES`-wide) `PolyStreamPlayer` that reads each voice from a **ring window**
(only a moving slice of the sample is resident) rather than a fully-resident PCM
buffer (Sa-1/Sa-2's `SamplePlayer`/`PolySamplePlayer` assume the whole sample is
in the pool). It handles **underrun** (the prefetch hasn't filled far enough yet)
by outputting silence and holding position, and it exposes a **read-cursor** per
voice so the (later-slice) prefetch task knows how far playback has consumed.

Pure `no_std` kernel, fully unit-testable with a mock-filled ring — no SD, no
task, no graph yet (those are slices 3–5).

## Design

New kernel in `crates/deluge-dsp-kernels/src/sampler.rs` (alongside
`SamplePlayer`/`PolySamplePlayer`), reusing the existing `hermite` helper.

### The ring-window model

- The full sample is `total` samples (u64; `0` = unknown/unbounded). At any moment
  only a **resident window** `[fill_lo, fill_hi)` of absolute sample indices is in
  the ring (`fill_hi − fill_lo ≤ ring.len()`). Absolute sample `a` (when
  `fill_lo ≤ a < fill_hi`) lives at `ring[(a % ring.len())]`.
- The **prefetch task (Slice 5) OWNS** `fill_lo`/`fill_hi` and the ring contents;
  the kernel RECEIVES them as `process_voice` params and never writes the ring. So
  the kernel is a pure consumer — trivially testable by hand-filling a ring.

### Kernel types

```rust
#[derive(Clone, Copy)]
pub struct StreamVoice {
    pos: f64,      // absolute fractional playback position (f64: streamed files
                   // run minutes long; f32's 24-bit mantissa would lose sample
                   // precision past ~16M samples)
    playing: bool,
}
#[derive(Clone, Copy)]
pub struct PolyStreamPlayer { voices: [StreamVoice; VOICES] }
```

### Per-voice process (mirrors `PolySamplePlayer::process_voice`)

```rust
// ring     : this voice's resident window buffer (len = ring capacity)
// fill_lo  : absolute index of the first resident sample
// fill_hi  : absolute index one past the last resident sample
// total    : full sample length (u64; 0 = unbounded)
// rate     : samples-advanced per output sample (f32; graph computes from pitch)
pub fn process_voice(&mut self, v: usize, ring: &[f32], fill_lo: u64, fill_hi: u64,
                     total: u64, rate: f32, out: &mut [f32]);
```

Per output sample, for voice `v`:
1. If `!playing` (or one-shot ended) → `out[i] = 0.0`.
2. Compute the 4 Hermite taps' absolute integer indices `⌊pos⌋−1 ..= ⌊pos⌋+2`.
   **UNDERRUN** if ANY tap is outside `[fill_lo, fill_hi)` (prefetch hasn't caught
   up) → `out[i] = 0.0` and **HOLD `pos`** (do NOT advance). When the prefetch
   later extends `fill_hi`, playback resumes at the exact same `pos` — no dropped
   samples. (Also underrun if a tap is `< fill_lo`, i.e. the prefetch discarded
   data still needed — shouldn't happen if the prefetch trails the read-cursor.)
3. Otherwise read `hermite(ring[t-1 % cap], ring[t % cap], ring[t+1 % cap],
   ring[t+2 % cap], frac)` (each tap modulo-indexed into the ring), `out[i] =`
   that, then `pos += rate`.
4. **One-shot end:** when `pos ≥ total` (and `total > 0`) → `playing = false`,
   remaining `out` = silence. (Looped streaming is deferred — Slice 2 is
   one-shot; a loop would require keeping the loop region resident.)

Bounds: `ring[a % ring.len()]` is always in-bounds for a non-empty ring; an empty
ring (`ring.len() == 0`) → treat as permanent underrun (all silence, hold). No
panic on any `fill_lo`/`fill_hi`/`total`/`rate` (incl. `fill_hi < fill_lo`, huge
values) — every ring index goes through `% ring.len()` and every tap is
range-checked against the window before the read. Mind 32-bit `usize` when
converting `u64` cursors to ring offsets: reduce modulo `ring.len()` in `u64`
BEFORE `as usize` (see [[target-32bit-usize-overflow]]).

### Voice API

```rust
pub fn new() -> PolyStreamPlayer;              // all voices pos 0, not playing
pub fn trigger_voice(&mut self, v: usize);     // pos = 0.0, playing = true
pub fn read_cursor(&self, v: usize) -> u64;    // ⌊pos⌋ — how far playback has consumed
pub fn is_playing(&self, v: usize) -> bool;
```
`read_cursor` is what the Slice-5 prefetch task polls to advance `fill_lo`
(discard behind) and target `fill_hi` (prefetch ahead). Bounds-guard `v < VOICES`
on every method (silently ignore / return default on out-of-range, like
`PolySamplePlayer`).

## Poly (8-voice) — accepted scope + caveat

Per the design decision, this is `VOICES`-wide from the start: 8 independent
`StreamVoice`s, each with its OWN ring + cursors (the graph/prefetch thread 8
ring sub-regions). **Caveat (a Slice-5 concern, not this kernel's):** 8
simultaneous streams = up to 8× SD read bandwidth + 8× ring RAM; the SD may not
sustain 8 concurrent high-rate streams — the prefetch task will need a bandwidth
budget / voice-stealing policy. The KERNEL is agnostic (each voice just consumes
its given window), so this caveat does not affect Slice 2, but is recorded for
Slice 5.

## NEON / SIMD

The per-sample windowed Hermite read is a **4-tap modulo gather** — not
NEON-friendly (same reason `PolyWt` is scalar-per-voice, see
[[sy-suite-voice-model]]). So this kernel is **scalar**, per [[prefer-neon-simd]]'s
"where possible / don't force it" clause. The arc's NEON acceleration is the
i16→f32 **decode** (Slice 1, done) that the prefetch fill uses — not the read.

## Non-breaking / integration

Purely additive — a new kernel type in `sampler.rs`. No existing code touched
(`SamplePlayer`/`PolySamplePlayer` unaffected). No graph/binding this slice.

## Testing

- **Full-resident read:** fill a ring so `[fill_lo, fill_hi)` covers all taps;
  `process_voice` at `rate = 1.0` reproduces the source samples (Hermite at
  integer pos returns the tap verbatim); `rate = 2.0` reads every other sample.
- **Underrun + seamless resume:** set `fill_hi` short of `⌊pos⌋+2` → output is
  silence and `read_cursor`/`pos` does NOT advance; then extend `fill_hi` and
  process again → playback resumes at the SAME `pos` and produces the correct
  samples (no drop). This is the core new behavior — lock it hard.
- **One-shot end:** `pos` reaching `total` → `is_playing` false, silence after.
- **Wrap-around ring:** a `fill_lo`/`fill_hi` window straddling the ring's modulo
  boundary reads the right samples (taps at `cap-1` and `0` are adjacent).
- **Per-voice independence:** voice 1 underrunning doesn't stall voice 0.
- **No panic:** empty ring, `fill_hi < fill_lo`, huge `total`/cursors, `v ≥ VOICES`.
- Tests pass BOTH configs (`cargo test -p deluge-dsp-kernels` ± `--features simd`
  — the kernel is scalar, but the crate builds both ways).

## Constraints (global)

- `no_std`, no heap, **no panic on any input** (cursors, rate, ring size). Reduce
  cursors modulo `ring.len()` in `u64` before `as usize` (32-bit `usize` safety).
- VOICES == 8. MIT/Apache-2.0. `f64` `pos` for large-file precision.
- Scalar (gather not vectorizable); tests pass both configs.

## Out of scope / deferred

- **Slice 3:** `Kind::StreamPlayer` graph node + a `Host` fill/resolve seam
  (thread the 8 rings + cursors from the engine; compute per-voice `rate` from a
  pitch tile like `PolySamplePlayer`).
- **Slice 4:** `Sample.stream("path")` Wren binding + host-sim `std::fs` prefetch
  (end-to-end playable in `cargo deluge sim`).
- **Slice 5:** firmware async-SD prefetch task (resolve extents via blocking FAT
  once, then raw async `sd::read_sectors`), + the 8-stream SD-bandwidth budget /
  voice-stealing policy.
- Looped streaming (Slice 2 is one-shot); 24/8-bit/float PCM; stereo.

## Likely task decomposition (for writing-plans)

1. `StreamVoice` + `PolyStreamPlayer::{new, trigger_voice, read_cursor, is_playing}`
   + the single-voice ring-window read core (full-resident + one-shot end) + tests
   (resident read matches reference, rate, one-shot stop, wrap-around, no-panic).
2. Underrun handling (hold-pos on missing taps) + the seamless-resume test + the
   per-voice-independence test.
