# WAV Decoder Implementation Plan (Sa-3b Slice 1)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A pure `no_std` WAV decoder — parse a 16-bit-PCM WAV header, and convert little-endian i16 PCM bytes to `f32` with a NEON/`f32x8` fast path — the foundation the rest of the SD-streaming arc consumes.

**Architecture:** New module `crates/deluge-dsp-kernels/src/wav.rs`. Cold path (`parse`) is scalar and bounds-checked (no panic on malformed input). Hot path (`decode_i16_le`) has a scalar oracle + `#[cfg(feature="simd")]` `f32x8` path that lowers to NEON on armv7a and is **bit-exact** to the scalar (integer widen + int→float cast + one multiply are exact).

**Tech Stack:** Rust `no_std`, `core::simd` (behind the crate's `simd` feature = nightly `portable_simd`); the crate's scalar-oracle + `f32x8` convention (see [[prefer-neon-simd]]).

## Global Constraints

- `no_std`, no heap, **no panic on ANY input** (malformed/truncated WAV, odd byte counts, empty slices). Every byte read is bounds-checked (`get`/length guards, `from_le_bytes` on checked sub-slices) — no bare indexing that could panic.
- Scalar path is the oracle; the `f32x8`/NEON conversion must be **bit-identical** to scalar in both configs.
- MIT/Apache-2.0.
- Little-endian PCM via `from_le_bytes` (no `transmute`, correct on any host).
- Test invocation: per-crate, NEVER `--workspace`; `-- name1 name2`. Both configs: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels` ± `--features simd`. LSP `armv7a … can't find crate for test` is noise.

## Interfaces (public API this slice produces — later slices consume)

```rust
pub struct WavInfo { pub channels: u8, pub sample_rate: u32, pub bits: u8, pub data_offset: usize, pub data_len: usize }
pub enum WavErr { BadMagic, Truncated, UnsupportedFormat, NoData }
pub fn parse(bytes: &[u8]) -> Result<WavInfo, WavErr>;
pub fn decode_i16_le(bytes: &[u8], out: &mut [f32]) -> usize; // returns samples written
```

---

### Task 1: `wav.rs` header parser

**Files:**
- Create: `crates/deluge-dsp-kernels/src/wav.rs`
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (register `pub mod wav;`)
- Test: `crates/deluge-dsp-kernels/src/wav.rs` test module

**Interfaces:**
- Produces: `WavInfo`, `WavErr`, `parse`. Consumed by Task 2's tests (a valid header) and later slices.

- [ ] **Step 1: Write the failing tests**

Create `crates/deluge-dsp-kernels/src/wav.rs` with the types + a `parse` stub returning `Err(WavErr::BadMagic)` so the tests compile-and-fail, plus this test module. The `wav16` helper builds a minimal 16-bit WAV:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal 16-bit PCM WAV: `channels`, `rate`, and `data` bytes.
    fn wav16(channels: u16, rate: u32, data: &[u8]) -> std::vec::Vec<u8> {
        let mut v = std::vec::Vec::new();
        let block_align: u16 = channels * 2;
        let byte_rate: u32 = rate * block_align as u32;
        v.extend_from_slice(b"RIFF");
        v.extend_from_slice(&(36u32 + data.len() as u32).to_le_bytes());
        v.extend_from_slice(b"WAVE");
        v.extend_from_slice(b"fmt ");
        v.extend_from_slice(&16u32.to_le_bytes());
        v.extend_from_slice(&1u16.to_le_bytes());          // audio_format = PCM
        v.extend_from_slice(&channels.to_le_bytes());
        v.extend_from_slice(&rate.to_le_bytes());
        v.extend_from_slice(&byte_rate.to_le_bytes());
        v.extend_from_slice(&block_align.to_le_bytes());
        v.extend_from_slice(&16u16.to_le_bytes());          // bits = 16
        v.extend_from_slice(b"data");
        v.extend_from_slice(&(data.len() as u32).to_le_bytes());
        v.extend_from_slice(data);
        v
    }

    #[test]
    fn parses_mono_16bit() {
        let w = wav16(1, 44100, &[1, 0, 2, 0]); // 2 samples
        let info = parse(&w).unwrap();
        assert_eq!(info.channels, 1);
        assert_eq!(info.sample_rate, 44100);
        assert_eq!(info.bits, 16);
        assert_eq!(info.data_len, 4);
        assert_eq!(&w[info.data_offset..info.data_offset + info.data_len], &[1, 0, 2, 0]);
    }

    #[test]
    fn parses_stereo() {
        let w = wav16(2, 48000, &[0; 8]);
        let info = parse(&w).unwrap();
        assert_eq!(info.channels, 2);
        assert_eq!(info.sample_rate, 48000);
    }

    #[test]
    fn skips_unknown_chunk_before_data() {
        // Insert a LIST chunk between fmt and data.
        let mut w = wav16(1, 44100, &[9, 0]);
        // find "data" and splice a LIST chunk (id+size+4 payload) before it
        let dpos = w.windows(4).position(|c| c == b"data").unwrap();
        let mut chunk = std::vec::Vec::new();
        chunk.extend_from_slice(b"LIST");
        chunk.extend_from_slice(&4u32.to_le_bytes());
        chunk.extend_from_slice(&[b'I', b'N', b'F', b'O']);
        w.splice(dpos..dpos, chunk);
        let info = parse(&w).unwrap();
        assert_eq!(info.data_len, 2);
        assert_eq!(&w[info.data_offset..info.data_offset + 2], &[9, 0]);
    }

    #[test]
    fn rejects_bad_and_truncated() {
        assert!(matches!(parse(b"NOPEwxyzWAVE"), Err(WavErr::BadMagic)));
        assert!(matches!(parse(b"RI"), Err(WavErr::Truncated)));
        assert!(matches!(parse(&[]), Err(WavErr::Truncated)));
        // non-PCM audio_format (3 = float)
        let mut w = wav16(1, 44100, &[0, 0]);
        let fpos = w.windows(4).position(|c| c == b"fmt ").unwrap() + 8;
        w[fpos] = 3; // audio_format low byte = 3
        assert!(matches!(parse(&w), Err(WavErr::UnsupportedFormat)));
        // 24-bit
        let mut w2 = wav16(1, 44100, &[0, 0]);
        let fpos2 = w2.windows(4).position(|c| c == b"fmt ").unwrap() + 8;
        w2[fpos2 + 14] = 24; // bits low byte
        assert!(matches!(parse(&w2), Err(WavErr::UnsupportedFormat)));
    }

    #[test]
    fn rejects_missing_data() {
        // fmt-only file (no data chunk)
        let mut v = std::vec::Vec::new();
        v.extend_from_slice(b"RIFF");
        v.extend_from_slice(&28u32.to_le_bytes());
        v.extend_from_slice(b"WAVE");
        v.extend_from_slice(b"fmt ");
        v.extend_from_slice(&16u32.to_le_bytes());
        v.extend_from_slice(&1u16.to_le_bytes());
        v.extend_from_slice(&1u16.to_le_bytes());
        v.extend_from_slice(&44100u32.to_le_bytes());
        v.extend_from_slice(&88200u32.to_le_bytes());
        v.extend_from_slice(&2u16.to_le_bytes());
        v.extend_from_slice(&16u16.to_le_bytes());
        assert!(matches!(parse(&v), Err(WavErr::NoData)));
    }
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- wav::tests`
Expected: FAIL (stub returns `BadMagic` for everything → valid-parse tests fail).

- [ ] **Step 3: Implement `parse` + register the module**

`crates/deluge-dsp-kernels/src/lib.rs`: add `pub mod wav;` (alphabetical — immediately before `pub mod wavetable;` at line ~27).

`wav.rs` (types + parser):
```rust
//! Minimal `no_std` WAV decoder: header parse + LE i16 PCM → f32 (NEON fast path).
//! Scope: 16-bit PCM, mono/stereo. No panic on malformed input.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WavInfo {
    pub channels: u8,
    pub sample_rate: u32,
    pub bits: u8,
    pub data_offset: usize,
    pub data_len: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WavErr { BadMagic, Truncated, UnsupportedFormat, NoData }

/// Parse a WAV header. Returns the format + the `data` chunk's payload
/// offset/len. Never panics: every read is bounds-checked.
pub fn parse(bytes: &[u8]) -> Result<WavInfo, WavErr> {
    fn u16le(b: &[u8], o: usize) -> Option<u16> {
        b.get(o..o + 2).map(|s| u16::from_le_bytes([s[0], s[1]]))
    }
    fn u32le(b: &[u8], o: usize) -> Option<u32> {
        b.get(o..o + 4).map(|s| u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }

    if bytes.len() < 12 {
        return Err(WavErr::Truncated);
    }
    if &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return Err(WavErr::BadMagic);
    }

    let mut pos = 12usize;
    let mut fmt: Option<(u8, u32, u8)> = None; // (channels, rate, bits)
    let mut data: Option<(usize, usize)> = None; // (offset, len)

    while pos + 8 <= bytes.len() {
        let id = &bytes[pos..pos + 4];
        let size = u32le(bytes, pos + 4).ok_or(WavErr::Truncated)? as usize;
        let payload = pos + 8;
        if id == b"fmt " {
            let af = u16le(bytes, payload).ok_or(WavErr::Truncated)?;
            let ch = u16le(bytes, payload + 2).ok_or(WavErr::Truncated)?;
            let rate = u32le(bytes, payload + 4).ok_or(WavErr::Truncated)?;
            let bits = u16le(bytes, payload + 14).ok_or(WavErr::Truncated)?;
            if af != 1 {
                return Err(WavErr::UnsupportedFormat); // 1 = PCM
            }
            if bits != 16 || ch < 1 || ch > 2 {
                return Err(WavErr::UnsupportedFormat);
            }
            fmt = Some((ch as u8, rate, bits as u8));
        } else if id == b"data" {
            let len = size.min(bytes.len().saturating_sub(payload));
            data = Some((payload, len));
        }
        // advance past this chunk (payload padded to even length)
        pos = payload.saturating_add(size).saturating_add(size & 1);
    }

    let (channels, sample_rate, bits) = fmt.ok_or(WavErr::UnsupportedFormat)?;
    let (data_offset, data_len) = data.ok_or(WavErr::NoData)?;
    Ok(WavInfo { channels, sample_rate, bits, data_offset, data_len })
}
```
> `pos = payload.saturating_add(size).saturating_add(size & 1)` — a maliciously huge `size` saturates `pos` past `bytes.len()`, exiting the loop without OOB. The `data` len is clamped to the actual remaining bytes.

- [ ] **Step 4: Run tests, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- wav::tests` (+ `--features simd`). Then the full crate both configs. Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/wav.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(kernels): wav.rs header parser — 16-bit PCM mono/stereo, no-panic bounds-checked"
```

---

### Task 2: `decode_i16_le` — scalar oracle + `f32x8`/NEON

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/wav.rs` (add `decode_i16_le` + tests)

**Interfaces:**
- Consumes: `parse`/`WavInfo` (Task 1) in the round-trip test.
- Produces: `pub fn decode_i16_le(bytes: &[u8], out: &mut [f32]) -> usize`.

- [ ] **Step 1: Write the failing tests**

Add to `wav.rs`'s test module:
```rust
#[test]
fn decode_known_values() {
    // 0x0000=0.0, 0x0080(LE)=-32768=-1.0, 0xFF7F(LE)=32767≈0.99997
    let bytes = [0u8, 0, 0x00, 0x80, 0xFF, 0x7F];
    let mut out = [0.0f32; 3];
    let n = decode_i16_le(&bytes, &mut out);
    assert_eq!(n, 3);
    assert_eq!(out[0], 0.0);
    assert_eq!(out[1], -1.0);
    assert!((out[2] - 0.99996948).abs() < 1e-6);
}

#[test]
fn decode_counts_and_odd_byte() {
    // 5 whole samples + 1 trailing odd byte → 5 written, odd byte ignored.
    let bytes = [0u8; 11];
    let mut out = [1.0f32; 8];
    let n = decode_i16_le(&bytes, &mut out);
    assert_eq!(n, 5);
    assert!(out[..5].iter().all(|&s| s == 0.0));
    assert_eq!(out[5], 1.0); // untouched
}

#[test]
fn decode_out_shorter_than_samples() {
    let bytes = [0u8; 40]; // 20 samples
    let mut out = [9.0f32; 3];
    let n = decode_i16_le(&bytes, &mut out);
    assert_eq!(n, 3); // clamped to out.len()
}

#[test]
fn decode_no_panic_on_empty_and_tiny() {
    let mut out = [0.0f32; 4];
    assert_eq!(decode_i16_le(&[], &mut out), 0);
    assert_eq!(decode_i16_le(&[1], &mut out), 0); // 1 byte = 0 whole samples
    let mut empty: [f32; 0] = [];
    assert_eq!(decode_i16_le(&[1, 0, 2, 0], &mut empty), 0);
}

#[test]
fn decode_wide_buffer_covers_simd_and_tail() {
    // 19 samples: 2 f32x8 chunks + 3-sample scalar tail (exercises both paths).
    let mut bytes = std::vec::Vec::new();
    for i in 0..19i16 {
        bytes.extend_from_slice(&(i * 1000).to_le_bytes());
    }
    let mut out = [0.0f32; 19];
    let n = decode_i16_le(&bytes, &mut out);
    assert_eq!(n, 19);
    for i in 0..19 {
        let expected = (i as i16 * 1000) as f32 * (1.0 / 32768.0);
        assert_eq!(out[i], expected, "sample {} bit-exact", i);
    }
}
```
> The `assert_eq!` on the expected float in `decode_wide_buffer_covers_simd_and_tail` is what makes it a **bit-exact scalar==simd** lock: it runs in both configs and both must produce the identical `(s as f32) * (1.0/32768.0)`.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- wav::tests::decode`
Expected: FAIL (`decode_i16_le` not defined).

- [ ] **Step 3: Implement `decode_i16_le`**

Add to `wav.rs`:
```rust
/// Convert a chunk of little-endian 16-bit PCM `bytes` to `f32` in [-1, 1).
/// Writes `min(bytes.len()/2, out.len())` samples and returns that count.
/// Channel-agnostic (interleaved stays interleaved). A trailing odd byte is
/// ignored. Never panics.
pub fn decode_i16_le(bytes: &[u8], out: &mut [f32]) -> usize {
    const INV: f32 = 1.0 / 32768.0;
    let n = (bytes.len() / 2).min(out.len());

    #[cfg(feature = "simd")]
    {
        use core::simd::prelude::*;
        let inv = f32x8::splat(INV);
        let chunks = n / 8;
        for c in 0..chunks {
            let base = c * 8;
            let mut a = [0i16; 8];
            for k in 0..8 {
                let j = 2 * (base + k);
                a[k] = i16::from_le_bytes([bytes[j], bytes[j + 1]]);
            }
            let v: f32x8 = i16x8::from_array(a).cast::<f32>() * inv;
            v.copy_to_slice(&mut out[base..base + 8]);
        }
        for i in (chunks * 8)..n {
            let s = i16::from_le_bytes([bytes[2 * i], bytes[2 * i + 1]]);
            out[i] = s as f32 * INV;
        }
    }
    #[cfg(not(feature = "simd"))]
    {
        for i in 0..n {
            let s = i16::from_le_bytes([bytes[2 * i], bytes[2 * i + 1]]);
            out[i] = s as f32 * INV;
        }
    }
    n
}
```
> Bit-exact: `i16x8.cast::<f32>()` is an exact int→float widen (i16 fits the f32 mantissa), and `* inv` is the same single multiply as the scalar `s as f32 * INV` — no reassociation, so both configs produce identical bits. Indexing `bytes[2*i+1]`/`bytes[j+1]` is in-bounds because `n = bytes.len()/2` guarantees `2*n <= bytes.len()`.

- [ ] **Step 4: Run tests, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- wav::tests::decode` (+ `--features simd`). Then the FULL crate both configs. Expected: PASS — the `decode_wide_buffer` bit-exact assertions hold in both scalar and simd builds.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/wav.rs
git commit -m "feat(kernels): wav decode_i16_le — LE i16→f32, scalar oracle + f32x8/NEON (bit-exact)"
```

---

## Self-Review

**Spec coverage:** Task 1 = the header parser (`WavInfo`/`WavErr`/`parse`, 16-bit PCM mono/stereo, chunk-skip, all reject cases, no-panic); Task 2 = `decode_i16_le` (scalar oracle + `f32x8`/NEON, bit-exact, count/odd-byte/no-panic). Together they are the spec's two parts. Deferred items (24/8-bit/float, downmix, ring/prefetch/binding) are out of scope, per the spec.

**Placeholder scan:** All code (parser + decoder + tests + the `wav16` helper) is verbatim. No TBD/vague steps.

**Type consistency:** `parse(&[u8]) -> Result<WavInfo, WavErr>` and `decode_i16_le(&[u8], &mut [f32]) -> usize` match the Interfaces block and the spec exactly. `WavInfo` fields (`channels/sample_rate/bits/data_offset/data_len`) are consistent across the parser, the tests, and the round-trip. The `simd` feature gate and `pub mod wav;` registration match the crate's existing convention.
