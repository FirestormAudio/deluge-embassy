//! Master limiter: a stereo-linked feedforward peak limiter applied to the
//! render root bus before the output clamp. Scalar — the gain envelope is a
//! serial recursion (`gain[i]` depends on `gain[i-1]`), so it does not
//! vectorize (see prefer-neon-simd "don't force it").

const DEFAULT_CEILING: f32 = 0.95;
const DEFAULT_RELEASE: f32 = 0.05;
const FLOOR: f32 = 1e-4;

/// A finite, strictly-positive ceiling (0/neg/NaN → default). Prevents both a
/// div-by-zero and an over-attenuate-to-silence on a bad param.
fn sanitize_ceiling(v: f32) -> f32 {
    if v.is_finite() {
        v.max(FLOOR)
    } else {
        DEFAULT_CEILING
    }
}
/// A finite, strictly-positive release time (0/neg/NaN → default).
fn sanitize_release(v: f32) -> f32 {
    if v.is_finite() {
        v.max(FLOOR)
    } else {
        DEFAULT_RELEASE
    }
}

#[derive(Clone, Copy)]
pub struct MasterLimiter {
    gain: f32,      // current gain reduction, 1.0 = no reduction (envelope state)
    ceiling: f32,   // linear output ceiling (0..1)
    release_s: f32, // release time, seconds
}

impl MasterLimiter {
    pub fn new(ceiling: f32, release: f32) -> MasterLimiter {
        MasterLimiter {
            gain: 1.0,
            ceiling: sanitize_ceiling(ceiling),
            release_s: sanitize_release(release),
        }
    }
    pub fn set_ceiling(&mut self, v: f32) {
        self.ceiling = sanitize_ceiling(v);
    }
    pub fn set_release(&mut self, v: f32) {
        self.release_s = sanitize_release(v);
    }

    /// Limit `l`/`r` in place with a single linked gain. `dt` = 1/sample_rate.
    pub fn process(&mut self, l: &mut [f32], r: &mut [f32], dt: f32) {
        // Sanitize dt: a non-finite/≤0 dt would make `rel_c` NaN and PERMANENTLY
        // latch `self.gain` to NaN (once NaN, `target < gain` is always false so
        // the instant-attack branch can never recover it) — fatal on the master
        // output. `dt` is normally 1/sample_rate (a sane constant), but this
        // kernel is the last safety net before the clamp, so guard it.
        let dt = if dt.is_finite() && dt > 0.0 {
            dt
        } else {
            FLOOR
        };
        // One-pole release coefficient: 1 - e^(-dt/tau), tau floored at dt.
        let rel_c = 1.0 - libm::expf(-dt / self.release_s.max(dt));
        let n = l.len().min(r.len());
        for i in 0..n {
            let li = l[i];
            let ri = r[i];
            let peak = libm::fabsf(li).max(libm::fabsf(ri));
            // Non-finite peak (NaN/inf sample) → pass through at unity, no panic.
            let target = if peak.is_finite() && peak > self.ceiling {
                self.ceiling / peak // peak > ceiling > 0 ⇒ divisor strictly positive
            } else {
                1.0
            };
            // Instant attack (snap down), one-pole release toward target.
            let c = if target < self.gain { 1.0 } else { rel_c };
            self.gain += (target - self.gain) * c;
            l[i] = li * self.gain;
            r[i] = ri * self.gain;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    const DT: f32 = 1.0 / 48_000.0;

    #[test]
    fn limits_peak_to_ceiling() {
        // Constant 0.8 in both channels, ceiling 0.5 → every out |sample| <= 0.5.
        let mut lim = MasterLimiter::new(0.5, 0.05);
        let mut l = [0.8f32; 64];
        let mut r = [0.8f32; 64];
        lim.process(&mut l, &mut r, DT);
        for i in 0..64 {
            assert!(l[i].abs() <= 0.5 + 1e-6, "l[{}]={}", i, l[i]);
            assert!(r[i].abs() <= 0.5 + 1e-6, "r[{}]={}", i, r[i]);
        }
        // Instant attack ⇒ even sample 0 is clamped.
        assert!((l[0] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn linked_preserves_channel_ratio() {
        // Loud L, quiet R, same gain ⇒ R/L ratio preserved (no image shift).
        let mut lim = MasterLimiter::new(0.5, 0.05);
        let mut l = [0.8f32; 64];
        let mut r = [0.2f32; 64];
        lim.process(&mut l, &mut r, DT);
        // both scaled by the same gain: out_r/out_l == in_r/in_l == 0.25
        assert!((l[0] - 0.5).abs() < 1e-6); // L clamped to ceiling
        assert!((r[0] / l[0] - 0.25).abs() < 1e-6); // ratio preserved
    }

    #[test]
    fn passes_below_ceiling_unchanged() {
        let mut lim = MasterLimiter::new(0.5, 0.05);
        let mut l = [0.3f32; 64];
        let mut r = [0.3f32; 64];
        lim.process(&mut l, &mut r, DT);
        for i in 0..64 {
            assert!((l[i] - 0.3).abs() < 1e-7);
            assert!((r[i] - 0.3).abs() < 1e-7);
        }
    }

    #[test]
    fn gain_recovers_after_loud() {
        // Drive gain down with a loud block, then feed quiet blocks; the applied
        // gain (out/in) should climb back toward 1.0.
        let mut lim = MasterLimiter::new(0.5, 0.001); // fast release for the test
        let mut l = [1.0f32; 64];
        let mut r = [1.0f32; 64];
        lim.process(&mut l, &mut r, DT); // gain now ~0.5
        let mut gains = std::vec::Vec::new();
        for _ in 0..40 {
            let mut ql = [0.1f32; 64];
            let mut qr = [0.1f32; 64];
            lim.process(&mut ql, &mut qr, DT);
            gains.push(ql[63] / 0.1); // applied gain this block
        }
        assert!(gains[0] < gains[gains.len() - 1], "gain should recover");
        assert!(*gains.last().unwrap() > 0.9, "gain should approach 1.0");
    }

    #[test]
    fn no_panic_on_adversarial_input() {
        // NaN/inf/zero/negative/huge params + non-finite samples must not panic.
        let mut lim = MasterLimiter::new(f32::NAN, -1.0); // → sanitized defaults
        let mut l = [f32::NAN, f32::INFINITY, 0.0, 1e30, -1e30, 0.5];
        let mut r = [0.0, f32::NEG_INFINITY, f32::NAN, 1.0, -1.0, 0.5];
        lim.process(&mut l, &mut r, DT);
        lim.set_ceiling(0.0); // floored, not div-by-zero
        lim.set_release(f32::NAN); // → default
        let mut l2 = [2.0f32; 8];
        let mut r2 = [2.0f32; 8];
        lim.process(&mut l2, &mut r2, DT); // must not panic
        assert!(l2[7].is_finite());
    }

    #[test]
    fn nonfinite_dt_does_not_latch_gain_to_nan() {
        // A NaN/inf dt must not permanently poison the gain envelope: a bad-dt
        // call is sanitized, and a subsequent valid-dt call still limits cleanly.
        let mut lim = MasterLimiter::new(0.5, 0.05);
        let mut l = [0.8f32; 64];
        let mut r = [0.8f32; 64];
        lim.process(&mut l, &mut r, f32::NAN); // must not poison gain
        lim.process(&mut l, &mut r, f32::INFINITY); // nor this
        let mut l2 = [0.8f32; 64];
        let mut r2 = [0.8f32; 64];
        lim.process(&mut l2, &mut r2, DT); // valid dt → clean limiting
        assert!(
            l2[63].is_finite() && l2[63].abs() <= 0.5 + 1e-6,
            "l2[63]={}",
            l2[63]
        );
    }
}
