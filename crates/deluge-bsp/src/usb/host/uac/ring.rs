//! Engine-rate interleaved-`f32` capture ring.
//!
//! The resampler pushes whole frames at the engine rate (44.1 kHz); the app
//! drains with [`SampleRing::read`]. Overrun drops the oldest frame (never
//! blocks the USB task); underrun is a short `read` (never blocks the app).

use super::MAX_CHANNELS;

/// Ring depth in frames. ~11.6 ms at 44.1 kHz — enough slack for the PI
/// controller to correct drift without over/underrunning under normal jitter.
pub const RING_FRAMES: usize = 512;
const RING_SAMPLES: usize = RING_FRAMES * MAX_CHANNELS;

/// Fixed-capacity interleaved-`f32` ring. Alloc-free.
pub struct SampleRing {
    buf: [f32; RING_SAMPLES],
    head: usize,  // index of the oldest stored sample
    count: usize, // stored samples
    channels: usize,
}

impl SampleRing {
    pub const fn new() -> Self {
        Self {
            buf: [0.0; RING_SAMPLES],
            head: 0,
            count: 0,
            channels: 1,
        }
    }

    /// Reset to empty for a stream of `channels` channels (clamped to
    /// `1..=MAX_CHANNELS`).
    pub fn reset(&mut self, channels: usize) {
        self.head = 0;
        self.count = 0;
        self.channels = channels.clamp(1, MAX_CHANNELS);
    }

    pub fn channels(&self) -> usize {
        self.channels
    }

    /// Whole frames currently buffered.
    pub fn fill_frames(&self) -> usize {
        self.count / self.channels
    }

    /// Ring depth in frames for the current channel count.
    pub fn capacity_frames(&self) -> usize {
        RING_SAMPLES / self.channels
    }

    /// Push one interleaved frame (`frame.len()` must equal `channels()`).
    /// On overrun, drops the oldest frame first — the USB task never blocks.
    pub fn push_frame(&mut self, frame: &[f32]) {
        debug_assert_eq!(frame.len(), self.channels);
        if self.count + self.channels > RING_SAMPLES {
            self.head = (self.head + self.channels) % RING_SAMPLES;
            self.count -= self.channels;
        }
        for &s in frame {
            let tail = (self.head + self.count) % RING_SAMPLES;
            self.buf[tail] = s;
            self.count += 1;
        }
    }

    /// Drain up to `out.len()` samples. Returns how many were written; a short
    /// return is an underrun (the app must tolerate it, never block).
    pub fn read(&mut self, out: &mut [f32]) -> usize {
        let n = out.len().min(self.count);
        for o in out.iter_mut().take(n) {
            *o = self.buf[self.head];
            self.head = (self.head + 1) % RING_SAMPLES;
        }
        self.count -= n;
        n
    }
}

impl Default for SampleRing {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn push_then_read_roundtrips_frames() {
        let mut r = SampleRing::new();
        r.reset(2);
        r.push_frame(&[1.0, 2.0]);
        r.push_frame(&[3.0, 4.0]);
        assert_eq!(r.fill_frames(), 2);
        let mut out = [0.0; 4];
        assert_eq!(r.read(&mut out), 4);
        assert_eq!(out, [1.0, 2.0, 3.0, 4.0]);
        assert_eq!(r.fill_frames(), 0);
    }

    #[test]
    fn read_underrun_is_short_not_blocking() {
        let mut r = SampleRing::new();
        r.reset(1);
        r.push_frame(&[9.0]);
        let mut out = [0.0; 4];
        assert_eq!(r.read(&mut out), 1, "only one sample available");
        assert_eq!(out[0], 9.0);
    }

    #[test]
    fn overrun_drops_oldest_frame() {
        let mut r = SampleRing::new();
        r.reset(2);
        let cap = r.capacity_frames();
        for i in 0..cap {
            r.push_frame(&[i as f32, i as f32]);
        }
        assert_eq!(r.fill_frames(), cap);
        // One more frame evicts the oldest (frame 0).
        r.push_frame(&[-1.0, -1.0]);
        assert_eq!(r.fill_frames(), cap);
        let mut first = [0.0; 2];
        assert_eq!(r.read(&mut first), 2);
        assert_eq!(first, [1.0, 1.0], "frame 0 was evicted, frame 1 is oldest");
    }
}
