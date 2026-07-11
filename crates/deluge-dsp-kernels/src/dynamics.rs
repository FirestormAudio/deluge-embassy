//! Feed-forward dynamics (Fx-1): a peak/RMS compressor with a soft knee,
//! attack/release ballistics, and makeup gain. Mono, scalar, `no_std`.

use crate::In; // same import path drive.rs uses for the per-sample input accessor

const LEVEL_EPS: f32 = 1e-9; // floor for log10 (avoid -inf on silence)
const RMS_WINDOW_S: f32 = 0.01; // 10 ms RMS averaging window

#[derive(Clone, Copy, PartialEq)]
pub enum Detector {
    Peak,
    Rms,
}

pub struct Comp {
    threshold_db: f32,
    ratio: f32,
    attack_s: f32,
    release_s: f32,
    knee_db: f32,
    makeup_db: f32,
    detector: Detector,
    rms_sq: f32, // smoothed x^2 (RMS state)
    gr_db: f32,  // current smoothed gain reduction, dB, >= 0
}

#[inline]
fn lin_to_db(x: f32) -> f32 {
    20.0 * libm::log10f(x.max(LEVEL_EPS))
}

#[inline]
fn db_to_lin(db: f32) -> f32 {
    libm::powf(10.0, db / 20.0)
}

#[inline]
fn one_pole_coeff(time_s: f32, dt: f32) -> f32 {
    // 1 - e^(-dt/tau); tau floored at dt so a ~0 time is ~1 (near-instant).
    1.0 - libm::expf(-dt / time_s.max(dt))
}

impl Comp {
    pub fn new(
        threshold_db: f32,
        ratio: f32,
        attack_s: f32,
        release_s: f32,
        knee_db: f32,
        makeup_db: f32,
        detector: Detector,
    ) -> Comp {
        Comp {
            threshold_db,
            ratio: ratio.max(1.0),
            attack_s: attack_s.max(0.0),
            release_s: release_s.max(0.0),
            knee_db: knee_db.max(0.0),
            makeup_db,
            detector,
            rms_sq: 0.0,
            gr_db: 0.0,
        }
    }

    pub fn set_threshold(&mut self, db: f32) { self.threshold_db = db; }
    pub fn set_ratio(&mut self, r: f32) { self.ratio = r.max(1.0); }
    pub fn set_attack(&mut self, s: f32) { self.attack_s = s.max(0.0); }
    pub fn set_release(&mut self, s: f32) { self.release_s = s.max(0.0); }
    pub fn set_knee(&mut self, db: f32) { self.knee_db = db.max(0.0); }
    pub fn set_makeup(&mut self, db: f32) { self.makeup_db = db; }
    pub fn set_detector(&mut self, d: Detector) { self.detector = d; }

    /// Static gain-reduction curve (dB, >= 0) for a given over-threshold amount.
    #[inline]
    fn gain_reduction(&self, over_db: f32) -> f32 {
        let slope = 1.0 - 1.0 / self.ratio; // 0 at 1:1, →1 at ∞:1
        let half = self.knee_db * 0.5;
        if self.knee_db > 0.0 && over_db > -half && over_db < half {
            // soft knee: quadratic, C0-continuous with the two branches.
            let t = over_db + half;
            slope * t * t / (2.0 * self.knee_db)
        } else if over_db <= 0.0 {
            0.0
        } else {
            slope * over_db
        }
    }

    pub fn process(&mut self, input: In, dt: f32, out: &mut [f32]) {
        let rms_c = one_pole_coeff(RMS_WINDOW_S, dt);
        let atk_c = one_pole_coeff(self.attack_s, dt);
        let rel_c = one_pole_coeff(self.release_s, dt);
        let makeup_lin = db_to_lin(self.makeup_db);
        for i in 0..out.len() {
            let x = input.at(i);
            // 1. detector level (dB)
            let level = match self.detector {
                Detector::Peak => libm::fabsf(x),
                Detector::Rms => {
                    self.rms_sq += (x * x - self.rms_sq) * rms_c;
                    libm::sqrtf(self.rms_sq)
                }
            };
            let level_db = lin_to_db(level);
            // 2. target gain reduction (dB, >= 0)
            let target_gr = self.gain_reduction(level_db - self.threshold_db);
            // 3. ballistics: attack when GR rising, release when falling
            let c = if target_gr > self.gr_db { atk_c } else { rel_c };
            self.gr_db += (target_gr - self.gr_db) * c;
            // 4. apply
            out[i] = x * db_to_lin(-self.gr_db) * makeup_lin;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    // dB helpers for tests.
    fn db(x: f32) -> f32 { 20.0 * libm::log10f(x.abs().max(1e-9)) }
    fn lin(db: f32) -> f32 { libm::powf(10.0, db / 20.0) }

    // `In::A(&slice)` is the array input accessor (see drive.rs node/kernel
    // tests: `In::A(&input)`). A constant input is a slice of the constant.
    fn run_const(c: &mut Comp, level_lin: f32, samples: usize, dt: f32) -> f32 {
        let arr = [level_lin; 64];
        let mut out = [0.0f32; 64];
        let mut last = 0.0;
        for _ in 0..(samples / 64) {
            c.process(In::A(&arr), dt, &mut out);
            last = out[out.len() - 1];
        }
        last / level_lin // = applied gain (linear)
    }

    const DT: f32 = 1.0 / 48_000.0;

    #[test]
    fn comp_below_threshold_is_unity() {
        // threshold -20 dB, ratio 4, no makeup, no knee; input -40 dB (below).
        let mut c = Comp::new(-20.0, 4.0, 0.001, 0.05, 0.0, 0.0, Detector::Peak);
        let g = run_const(&mut c, lin(-40.0), 4096, DT);
        assert!((db(g)).abs() < 0.2, "below threshold ≈ unity gain, got {} dB", db(g));
    }

    #[test]
    fn comp_static_curve_steady_state() {
        // thr -20, ratio 4:1, input -8 dB → GR = (in-thr)(1-1/ratio) = 12*0.75 = 9 dB.
        let mut c = Comp::new(-20.0, 4.0, 0.001, 0.05, 0.0, 0.0, Detector::Peak);
        let g = run_const(&mut c, lin(-8.0), 8192, DT);
        // applied gain in dB should be ≈ -9 dB (attenuation).
        assert!((db(g) - (-9.0)).abs() < 0.5, "static curve GR ≈ 9 dB, got {} dB", -db(g));
    }

    #[test]
    fn comp_limiter_clamps_near_threshold() {
        // ratio 20, fast attack, peak; input 0 dB, threshold -10 → output ≈ -10 dB.
        let mut c = Comp::new(-10.0, 20.0, 0.001, 0.05, 0.0, 0.0, Detector::Peak);
        let g = run_const(&mut c, lin(0.0), 8192, DT);
        let out_db = db(lin(0.0) * g); // input 0 dB + gain
        assert!(out_db < -8.5 && out_db > -11.5, "limiter clamps near -10 dB, got {}", out_db);
    }

    #[test]
    fn comp_soft_knee_partial_at_threshold() {
        // input exactly at threshold. hard knee → ~0 GR; soft knee → partial GR.
        let mut hard = Comp::new(-20.0, 4.0, 0.001, 0.05, 0.0, 0.0, Detector::Peak);
        let gh = run_const(&mut hard, lin(-20.0), 8192, DT);
        let mut soft = Comp::new(-20.0, 4.0, 0.001, 0.05, 12.0, 0.0, Detector::Peak);
        let gs = run_const(&mut soft, lin(-20.0), 8192, DT);
        assert!(db(gh).abs() < 0.3, "hard knee at threshold ≈ 0 GR, got {} dB", -db(gh));
        assert!(-db(gs) > 0.3, "soft knee at threshold has partial GR, got {} dB", -db(gs));
    }

    #[test]
    fn comp_ballistics_move_and_settle() {
        // After a step up in input, GR increases (gain drops); after step down, recovers.
        let mut c = Comp::new(-20.0, 8.0, 0.01, 0.05, 0.0, 0.0, Detector::Peak);
        let quiet = run_const(&mut c, lin(-40.0), 2048, DT); // settle quiet → ~unity
        let loud = run_const(&mut c, lin(0.0), 4096, DT);    // settle loud → strong GR
        let recovered = run_const(&mut c, lin(-40.0), 8192, DT); // release back
        assert!(db(loud) < db(quiet) - 1.0, "loud input reduces gain vs quiet");
        assert!(db(recovered) > db(loud) + 1.0, "release recovers gain after loud");
    }
}
