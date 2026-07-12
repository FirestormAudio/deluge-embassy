# WAV Decoder — Design Spec (Sa-3b Slice 1)

**Date:** 2026-07-12
**Status:** Approved (design), pending implementation plan
**Sub-project:** Sa-3b (SD sample streaming) — Slice 1 of a 5-slice arc. The foundational, fully-testable piece every later slice consumes.

## Context

Sa-3b brings SD-card sample **streaming** to the deluge-sdk. The SD/FAT driver
already exists and is production-proven (`crates/deluge-bsp/src/{sd,fat}.rs`,
`deluge-sdk/src/sd.rs::Sd::read`, used by `wren-firmware` `flash_task`), but there
is **no WAV/PCM decoder anywhere** in the workspace. This slice builds it:
a pure `no_std` decoder that (1) parses a WAV header and (2) converts raw PCM
bytes to `f32` — the latter **NEON/SIMD-accelerated** (see [[prefer-neon-simd]]).

The decoder is built **streaming-friendly**: header parse is separate from PCM
conversion, and conversion operates on an arbitrary **byte chunk** of the data
section (not just a whole file), so later slices call it per ring-refill. See
[[sa-suite-samples]], [[sy-suite-voice-model]].

## Scope

- **In:** RIFF/WAVE parsing, **16-bit PCM**, mono + stereo, standard chunk order
  (skips unknown chunks); little-endian i16 → f32 conversion with a scalar oracle
  + `f32x8`/NEON fast path.
- **Deferred (later slices / follow-ons):** 24-bit / 8-bit / float PCM; stereo
  downmix / channel selection (the streaming/binding layer decides); extent
  resolution, ring buffer, prefetch task, `Sample.stream("path")` binding (Sa-3b
  slices 2–5).

## Design

New module `crates/deluge-dsp-kernels/src/wav.rs` (pure `no_std`, same crate as
`sampler.rs`). Two clearly-separated parts, split by hot-vs-cold:

### 1. Header parse (scalar — one-time, cold)

```rust
pub struct WavInfo {
    pub channels: u8,       // 1 or 2
    pub sample_rate: u32,   // e.g. 44100
    pub bits: u8,           // 16 (this slice)
    pub data_offset: usize, // byte offset of the `data` chunk payload
    pub data_len: usize,    // byte length of the PCM payload
}
pub enum WavErr { BadMagic, Truncated, UnsupportedFormat, NoData }
pub fn parse(bytes: &[u8]) -> Result<WavInfo, WavErr>;
```

- Walk the RIFF chunk list: verify `RIFF`…`WAVE`; find `fmt ` (read
  `audio_format` — require 1 = PCM; `channels` 1–2; `sample_rate`;
  `bits_per_sample` — require 16); find `data` (record payload offset + len);
  skip any other chunks (`LIST`, `fact`, …) by their size field.
- **No panic on any input:** every field read is bounds-checked against
  `bytes.len()`; malformed/truncated/unsupported → the matching `WavErr`. Uses
  `from_le_bytes` on checked sub-slices. Scalar (tiny, one-time — not vectorized).

### 2. PCM conversion (the hot path — NEON-accelerated)

```rust
/// Convert a chunk of little-endian 16-bit PCM bytes to f32 in [-1, 1).
/// Writes `min(bytes.len()/2, out.len())` samples; returns the count written.
/// Channel-agnostic: interleaved stereo stays interleaved (the caller
/// de-interleaves / down-mixes). A trailing odd byte is ignored.
pub fn decode_i16_le(bytes: &[u8], out: &mut [f32]) -> usize;
```

- **Scalar path = correctness ORACLE:** per sample `i16::from_le_bytes([b0, b1])
  as f32 * (1.0 / 32768.0)` → f32 in `[-1.0, 1.0)` (full-scale `-32768 → -1.0`,
  `32767 → ~0.99997`, `0 → 0.0`).
- **`#[cfg(feature="simd")]` `f32x8`:** process 8 samples/iteration — load 16
  bytes → `i16x8` (little-endian: read via `from_le_bytes` into an `[i16; 8]`
  then `Simd::from_array`, or an endianness-correct load) → `.cast::<f32>()` →
  multiply by `f32x8::splat(1.0 / 32768.0)` → `copy_to_slice`. Scalar-tail the
  remainder (< 8 samples) and any odd trailing byte. **Lowers to NEON on
  armv7a.** Must match the scalar oracle **bit-exactly** (the operations —
  integer widen, int→float cast, single multiply — are exact and
  reassociation-free, so bit-exact is achievable, unlike the tolerance-based osc
  paths).

## Non-breaking / integration

Purely additive — a brand-new module, no existing code touched. No graph/binding
changes this slice. `sampler.rs`'s existing pool-resident playback is unaffected;
later slices wire the decoder into the streaming path.

## Testing

- **Header parse:** a hand-built minimal 16-bit **mono** WAV (RIFF/WAVE/fmt/data)
  parses to the right `WavInfo`; a **stereo** one gives `channels == 2`; reject
  cases each return the right `WavErr` — bad magic, truncated header, non-PCM
  `audio_format`, 24-bit `bits_per_sample`, missing `data`; an unknown chunk
  before `data` is skipped correctly.
- **PCM conversion:** known bytes → known floats (`0x0000→0.0`, `0x0080` i.e.
  `-32768 → -1.0`, `0xFF7F` i.e. `32767 → ~0.99997`); count-written correctness;
  a trailing odd byte is ignored; `out` shorter than the sample count stops
  cleanly.
- **`scalar == simd`:** the same byte buffer through `decode_i16_le` yields
  **bit-identical** `f32` output in the default and `--features simd` builds.
- **No panic:** empty input, 1-byte input, truncated header, `out.len() == 0`.
- Tests pass in BOTH configs (`cargo test -p deluge-dsp-kernels` ± `--features
  simd`).

## Constraints (global)

- `no_std`, no heap, **no panic on any input** (malformed WAV, truncated bytes).
- Scalar path is the oracle; the `f32x8`/NEON fast path must match it
  **bit-exactly** for the i16→f32 conversion; tests pass both configs.
- MIT/Apache-2.0.
- Little-endian PCM (WAV is LE); the code reads via `from_le_bytes` so it is
  correct regardless of host endianness (x86 test host and armv7a target both LE
  anyway, but don't rely on `transmute`).

## Out of scope / deferred (the rest of the Sa-3b arc)

- **Slice 2:** streaming ring buffer + windowed `StreamPlayer` kernel (read by
  `pos`, read-cursor, underrun→silence).
- **Slice 3:** `Kind::StreamPlayer` graph node + a `Host` fill seam.
- **Slice 4:** `Sample.stream("path")` Wren binding + host-sim `std::fs` prefetch
  (end-to-end playable in `cargo deluge sim`).
- **Slice 5:** firmware async-SD prefetch — resolve file extents via the blocking
  FAT once, then a prefetch task reading raw sectors via async
  `sd::read_sectors` (bypassing the FAT `block_on` that stalls `audio_task`).
  Compile/cross-compile + review only here (HW-verified on device).
- Also deferred: 24/8-bit + float PCM, stereo downmix/channel-select.

## Likely task decomposition (for writing-plans)

1. `wav.rs` header parser `WavInfo`/`WavErr`/`parse` + parse tests (valid
   mono/stereo, all reject cases, chunk-skip). Register the module in `lib.rs`.
2. `decode_i16_le` scalar + `f32x8`/NEON path + conversion tests (known
   values, count/odd-byte handling, `scalar == simd` bit-exact, no-panic).
