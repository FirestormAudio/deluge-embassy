//! Fractional resampler + PI drift controller for USB capture.
//!
//! `r` is **input frames per output frame**. A device running fast fills the
//! ring, the PI controller raises `r`, the resampler emits fewer frames per
//! input frame, and the ring drains back to its setpoint. No feedback endpoint,
//! no rate estimation — ring fill *is* the clock error (design of record §3.4).

use super::MAX_CHANNELS;

/// Linear-interpolating variable-rate resampler over interleaved `f32` frames.
///
/// Fed one input frame at a time; emits zero or more output frames per input.
/// Holds the previous input frame so interpolation is continuous across calls.
pub struct Resampler {
    channels: usize,
    prev: [f32; MAX_CHANNELS],
    have_prev: bool,
    /// Position of the next output within `[prev, cur)`, in input-frame units.
    pos: f32,
}

impl Resampler {
    pub const fn new() -> Self {
        Self { channels: 1, prev: [0.0; MAX_CHANNELS], have_prev: false, pos: 0.0 }
    }

    pub fn reset(&mut self, channels: usize) {
        self.channels = channels.clamp(1, MAX_CHANNELS);
        self.have_prev = false;
        self.pos = 0.0;
    }

    /// Feed one interleaved input frame (`cur.len()` must be `channels`) at
    /// ratio `r` (> 0). Calls `emit` once per emitted output frame.
    pub fn feed(&mut self, cur: &[f32], r: f32, mut emit: impl FnMut(&[f32])) {
        let ch = self.channels;
        if !self.have_prev {
            self.prev[..ch].copy_from_slice(&cur[..ch]);
            self.have_prev = true;
            self.pos = 0.0;
            return;
        }
        let mut out = [0.0f32; MAX_CHANNELS];
        // Emit every output whose position lies in [prev@0.0, cur@1.0).
        while self.pos < 1.0 {
            let f = self.pos;
            for c in 0..ch {
                out[c] = self.prev[c] + (cur[c] - self.prev[c]) * f;
            }
            emit(&out[..ch]);
            self.pos += r;
        }
        self.pos -= 1.0; // cur becomes the new prev (coordinate 0.0)
        self.prev[..ch].copy_from_slice(&cur[..ch]);
    }
}

impl Default for Resampler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    use heapless::Vec;

    fn run(channels: usize, r: f32, frames: &[&[f32]]) -> Vec<[f32; MAX_CHANNELS], 64> {
        let mut rs = Resampler::new();
        rs.reset(channels);
        let mut out: Vec<[f32; MAX_CHANNELS], 64> = Vec::new();
        for fr in frames {
            rs.feed(fr, r, |o| {
                let mut a = [0.0; MAX_CHANNELS];
                a[..o.len()].copy_from_slice(o);
                out.push(a).unwrap();
            });
        }
        out
    }

    #[test]
    fn unity_ratio_is_passthrough_delayed_one_frame() {
        // r = 1.0: first frame primes, then each feed emits the previous frame.
        let out = run(1, 1.0, &[&[1.0], &[2.0], &[3.0]]);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0][0], 1.0);
        assert_eq!(out[1][0], 2.0);
    }

    #[test]
    fn downsample_two_to_one() {
        let out = run(1, 2.0, &[&[1.0], &[2.0], &[3.0], &[4.0], &[5.0]]);
        // ~one output per two inputs after priming.
        assert_eq!(out.len(), 2);
    }

    #[test]
    fn upsample_one_to_two_interpolates_midpoint() {
        // prev = 0, cur = 10, r = 0.5 -> outputs at pos 0.0 (=0) and 0.5 (=5).
        let out = run(1, 0.5, &[&[0.0], &[10.0]]);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0][0], 0.0);
        assert_eq!(out[1][0], 5.0);
    }

    #[test]
    fn preserves_channel_interleave() {
        let out = run(2, 1.0, &[&[1.0, -1.0], &[2.0, -2.0], &[3.0, -3.0]]);
        assert_eq!(out.len(), 2);
        assert_eq!(&out[0][..2], &[1.0, -1.0]);
        assert_eq!(&out[1][..2], &[2.0, -2.0]);
    }
}
