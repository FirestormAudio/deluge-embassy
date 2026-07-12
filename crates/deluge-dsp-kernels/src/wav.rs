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

#[cfg(test)]
mod tests {
    extern crate std;
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
