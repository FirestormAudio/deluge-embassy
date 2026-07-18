//! USB host UAC **playback** primitives: `f32`->24-bit encode and an
//! output-driven (pull) resampler that turns engine-rate frames into
//! device-rate frames. Pure `core` math, host-tested. See the Phase-2 design.

use super::MAX_CHANNELS;
use super::ring::SampleRing;

/// 2^23 — full-scale for signed 24-bit PCM.
const FULL_SCALE_24: f32 = 8_388_608.0;

/// Encode one `f32` sample in `[-1.0, 1.0)` to signed 24-bit little-endian
/// (writes 3 bytes to `out`). Full-scale; clamps to the valid 24-bit range.
#[inline]
pub fn encode_s24le(x: f32, out: &mut [u8]) {
    let i = (x * FULL_SCALE_24).clamp(-FULL_SCALE_24, FULL_SCALE_24 - 1.0) as i32;
    out[0] = i as u8;
    out[1] = (i >> 8) as u8;
    out[2] = (i >> 16) as u8;
}

/// Pull one whole frame (`ch` samples) out of `ring` into `out`. Returns
/// `false` (leaving `out` untouched) if a full frame isn't available.
fn pull_frame(ring: &mut SampleRing, ch: usize, out: &mut [f32]) -> bool {
    if ring.fill_frames() >= 1 {
        ring.read(&mut out[..ch]);
        true
    } else {
        false
    }
}

/// Output-driven linear resampler: engine-rate frames (from a ring the app
/// fills) -> exactly `frames` device-rate frames. Emits silence for any frame
/// the ring can't supply (underrun never stalls). Holds `prev`/`cur` so
/// interpolation is continuous across `produce` calls.
pub struct PlaybackResampler {
    channels: usize,
    prev: [f32; MAX_CHANNELS],
    cur: [f32; MAX_CHANNELS],
    primed: bool,
    /// Fractional position within `[prev, cur)`, in engine-frame units.
    pos: f32,
}

impl PlaybackResampler {
    pub const fn new() -> Self {
        Self {
            channels: 1,
            prev: [0.0; MAX_CHANNELS],
            cur: [0.0; MAX_CHANNELS],
            primed: false,
            pos: 0.0,
        }
    }

    pub fn reset(&mut self, channels: usize) {
        self.channels = channels.clamp(1, MAX_CHANNELS);
        self.prev = [0.0; MAX_CHANNELS];
        self.cur = [0.0; MAX_CHANNELS];
        self.primed = false;
        self.pos = 0.0;
    }

    /// Produce `frames` output frames, advancing `step` engine-frames per output
    /// frame (`step = 1.0 / capture_ratio`). Calls `emit` once per output frame
    /// with an interleaved slice of length `channels`.
    pub fn produce(
        &mut self,
        ring: &mut SampleRing,
        step: f32,
        frames: usize,
        mut emit: impl FnMut(&[f32]),
    ) {
        let ch = self.channels;
        let mut out = [0.0f32; MAX_CHANNELS];
        for _ in 0..frames {
            if !self.primed {
                if !pull_frame(ring, ch, &mut self.prev) {
                    // Nothing buffered yet: emit silence, stay unprimed.
                    out[..ch].fill(0.0);
                    emit(&out[..ch]);
                    continue;
                }
                if !pull_frame(ring, ch, &mut self.cur) {
                    self.cur[..ch].copy_from_slice(&self.prev[..ch]);
                }
                self.primed = true;
                self.pos = 0.0;
            }
            while self.pos >= 1.0 {
                self.pos -= 1.0;
                self.prev[..ch].copy_from_slice(&self.cur[..ch]);
                if !pull_frame(ring, ch, &mut self.cur) {
                    self.cur[..ch].fill(0.0); // underrun -> silence
                }
            }
            let f = self.pos;
            for c in 0..ch {
                out[c] = self.prev[c] + (self.cur[c] - self.prev[c]) * f;
            }
            emit(&out[..ch]);
            self.pos += step;
        }
    }
}

impl Default for PlaybackResampler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::super::decode_s24le;
    use super::*;
    use heapless::Vec;

    #[test]
    fn encode_s24le_endpoints() {
        let mut b = [0u8; 3];
        encode_s24le(0.0, &mut b);
        assert_eq!(b, [0, 0, 0]);
        encode_s24le(0.5, &mut b);
        assert_eq!(b, [0x00, 0x00, 0x40]); // 0.5 * 2^23 = 0x400000
        encode_s24le(-1.0, &mut b);
        assert_eq!(b, [0x00, 0x00, 0x80]); // -2^23 = 0xFF800000 -> low 3 bytes
    }

    #[test]
    fn encode_decode_round_trips() {
        for &x in &[0.0f32, 0.25, -0.5, 0.9] {
            let mut b = [0u8; 3];
            encode_s24le(x, &mut b);
            let y = decode_s24le(&b);
            assert!((x - y).abs() < 1.0 / FULL_SCALE_24 * 2.0, "x={x} y={y}");
        }
    }

    #[test]
    fn unity_step_passes_engine_frames_through() {
        // step = 1.0: one engine frame consumed per output frame.
        let mut ring = SampleRing::new();
        ring.reset(1);
        for v in [1.0, 2.0, 3.0, 4.0] {
            ring.push_frame(&[v]);
        }
        let mut pr = PlaybackResampler::new();
        pr.reset(1);
        let mut got: Vec<f32, 8> = Vec::new();
        pr.produce(&mut ring, 1.0, 3, |o| got.push(o[0]).unwrap());
        // Primed with prev=1,cur=2; first output at pos 0 = prev = 1.0, then
        // pos steps by 1 each frame advancing prev<-cur and pulling the next.
        assert_eq!(got.len(), 3);
        assert_eq!(got[0], 1.0);
        assert_eq!(got[1], 2.0);
        assert_eq!(got[2], 3.0);
    }

    #[test]
    fn underrun_emits_silence() {
        let mut ring = SampleRing::new();
        ring.reset(2);
        // Only one frame available; ask for 4 stereo frames.
        ring.push_frame(&[0.5, -0.5]);
        let mut pr = PlaybackResampler::new();
        pr.reset(2);
        let mut frames: Vec<[f32; 2], 8> = Vec::new();
        pr.produce(&mut ring, 1.0, 4, |o| frames.push([o[0], o[1]]).unwrap());
        assert_eq!(frames.len(), 4);
        // Later frames must be silence, never a panic or stale garbage.
        assert_eq!(frames[3], [0.0, 0.0]);
    }

    #[test]
    fn produces_exact_frame_count_and_channel_width() {
        let mut ring = SampleRing::new();
        ring.reset(2);
        for _ in 0..16 {
            ring.push_frame(&[0.1, 0.2]);
        }
        let mut pr = PlaybackResampler::new();
        pr.reset(2);
        let mut n = 0usize;
        pr.produce(&mut ring, 1.0, 5, |o| {
            assert_eq!(o.len(), 2);
            n += 1;
        });
        assert_eq!(n, 5);
    }
}
