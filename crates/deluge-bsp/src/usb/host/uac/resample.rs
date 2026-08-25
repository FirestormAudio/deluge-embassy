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
        Self {
            channels: 1,
            prev: [0.0; MAX_CHANNELS],
            have_prev: false,
            pos: 0.0,
        }
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

/// PI controller mapping ring **fill error** to the resampler ratio `r`.
///
/// Ring fill is the clock error: too full (device fast) raises `r` so the
/// resampler emits fewer frames per input frame and the ring drains back to
/// `setpoint`. Output `r` is clamped to a narrow band around 1.0 — with the
/// 44.1/24-only policy true drift stays within a few hundred ppm.
pub struct PiController {
    setpoint: f32,
    kp: f32,
    ki: f32,
    integ: f32,
    r_min: f32,
    r_max: f32,
}

impl PiController {
    /// `setpoint_frames` is the target ring fill (aim for half the ring).
    pub fn new(setpoint_frames: f32) -> Self {
        Self {
            setpoint: setpoint_frames,
            kp: 2.0e-6,
            ki: 2.0e-8,
            integ: 0.0,
            r_min: 0.98,
            r_max: 1.02,
        }
    }

    /// Update from the current ring fill (frames); returns the new ratio `r`.
    pub fn update(&mut self, fill_frames: f32) -> f32 {
        let err = fill_frames - self.setpoint;
        self.integ += err;
        // Anti-windup: clamp the integral term's contribution to the r-band.
        let i_limit = (self.r_max - 1.0) / self.ki;
        self.integ = self.integ.clamp(-i_limit, i_limit);
        (1.0 + self.kp * err + self.ki * self.integ).clamp(self.r_min, self.r_max)
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

    // --- PI controller ---

    #[test]
    fn pi_raises_r_when_ring_too_full() {
        let mut pi = PiController::new(256.0);
        let r = pi.update(300.0); // above setpoint
        assert!(r > 1.0, "device fast (ring full) must raise r, got {r}");
    }

    #[test]
    fn pi_lowers_r_when_ring_too_empty() {
        let mut pi = PiController::new(256.0);
        let r = pi.update(200.0); // below setpoint
        assert!(r < 1.0, "device slow (ring empty) must lower r, got {r}");
    }

    #[test]
    fn pi_output_is_clamped_to_band() {
        let mut pi = PiController::new(256.0);
        for _ in 0..10_000 {
            let r = pi.update(100_000.0); // absurd error, drive windup
            assert!((0.98..=1.02).contains(&r));
        }
    }

    /// The real hazard: a device clock 100 ppm fast must converge to a stable
    /// ring fill, not drift into over/underrun. Model one real-time slice per
    /// iteration: the device delivers `in_frames` at (1 + ppm)*nominal, the
    /// engine drains `nominal` output frames, and the PI steers `r`.
    #[test]
    fn converges_under_100ppm_fast_clock() {
        use super::super::ring::SampleRing;

        const PPM: f32 = 100.0e-6;
        const NOMINAL: f32 = 64.0; // engine frames drained per slice
        let setpoint = 256.0f32;

        let mut ring = SampleRing::new();
        ring.reset(1);
        // Pre-fill to the setpoint so we start balanced.
        for _ in 0..(setpoint as usize) {
            ring.push_frame(&[0.0]);
        }

        let mut rs = Resampler::new();
        rs.reset(1);
        let mut pi = PiController::new(setpoint);
        let mut r = 1.0f32;
        let mut drained_acc = 0.0f32;
        let mut in_acc = 0.0f32;
        let mut sample = 0.0f32;

        for _ in 0..20_000 {
            // Device delivers (1+ppm)*NOMINAL input frames this slice.
            in_acc += NOMINAL * (1.0 + PPM);
            while in_acc >= 1.0 {
                in_acc -= 1.0;
                sample += 1.0;
                rs.feed(&[sample], r, |o| ring.push_frame(o));
            }
            // Engine drains NOMINAL output frames.
            drained_acc += NOMINAL;
            let want = drained_acc as usize;
            drained_acc -= want as f32;
            let mut scratch = [0.0f32; 256];
            let mut left = want;
            while left > 0 {
                let n = ring.read(&mut scratch[..left.min(256)]);
                if n == 0 {
                    break; // underrun tolerated
                }
                left -= n;
            }
            r = pi.update(ring.fill_frames() as f32);
        }

        let fill = ring.fill_frames() as f32;
        assert!(
            (fill - setpoint).abs() < 64.0,
            "ring should stay near setpoint, ended at {fill}"
        );
        assert!((0.98..=1.02).contains(&r), "r must stay bounded, ended {r}");
        // Steady-state r tracks the clock offset (~1 + ppm), well inside band.
        assert!(
            r > 1.0,
            "sustained fast clock should hold r slightly above 1.0"
        );
    }
}
