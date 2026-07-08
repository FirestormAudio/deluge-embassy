//! One-pole low-pass. Serial IIR recurrence → scalar (cross-voice SIMD later).
//! Mirrors the prototype's K_LPF.

use crate::In;
use core::f32::consts::PI;

#[derive(Clone, Copy)]
pub struct OnePole {
    z: f32,
}

impl OnePole {
    pub fn new() -> OnePole {
        OnePole { z: 0.0 }
    }

    pub fn process(&mut self, input: In, cutoff: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            let x = input.at(i);
            let fc = cutoff.at(i).max(1.0);
            let c = (2.0 * PI * fc * dt).min(1.0);
            self.z += c * (x - self.z);
            *s = self.z;
        }
    }
}

impl Default for OnePole {
    fn default() -> Self {
        OnePole::new()
    }
}

/// Which SVF response a node writes. Fixed per node (chosen by the graph Kind).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SvfResp {
    Lp,
    Hp,
    Bp,
    Notch,
}

/// Minimum damping. `res=1` maps to `k=0` (a marginally-stable, lossless
/// resonator); flooring at `SVF_K_MIN` keeps self-oscillation numerically
/// bounded instead of drifting.
pub(crate) const SVF_K_MIN: f32 = 1e-4;

/// `res ∈ [0,1]` → damping `k = 1/Q`. `res=0` → k=2 (Q=0.5, gentle);
/// `res=1` → k=SVF_K_MIN (edge of self-oscillation).
#[inline]
pub(crate) fn svf_k_from_res(res: f32) -> f32 {
    let r = res.clamp(0.0, 1.0);
    (2.0 * (1.0 - r)).max(SVF_K_MIN)
}

/// Zavalishin/Cytomic TPT state-variable filter (two trapezoidal integrators).
/// Serial recurrence → scalar (cross-voice SIMD lives at the voice layer).
#[derive(Clone, Copy)]
pub struct Svf {
    ic1eq: f32,
    ic2eq: f32,
}

impl Svf {
    pub fn new() -> Svf {
        Svf { ic1eq: 0.0, ic2eq: 0.0 }
    }

    pub fn process(
        &mut self,
        input: In,
        cutoff: In,
        res: In,
        resp: SvfResp,
        dt: f32,
        out: &mut [f32],
    ) {
        // Const-cutoff fast path: coefficients once per block (exact tanf).
        if let (Some(fc), Some(r)) = (cutoff.as_const(), res.as_const()) {
            let g = libm::tanf(PI * fc * dt);
            let k = svf_k_from_res(r);
            let (a1, a2, a3) = svf_coeffs(g, k);
            for (i, s) in out.iter_mut().enumerate() {
                *s = self.tick(input.at(i), k, a1, a2, a3, resp);
            }
            return;
        }
        // Audio-rate path: per-sample coefficients (exact tanf for now;
        // Task 3 swaps in the polynomial prewarp).
        for (i, s) in out.iter_mut().enumerate() {
            let fc = cutoff.at(i);
            let g = libm::tanf(PI * fc * dt);
            let k = svf_k_from_res(res.at(i));
            let (a1, a2, a3) = svf_coeffs(g, k);
            *s = self.tick(input.at(i), k, a1, a2, a3, resp);
        }
    }

    /// One TPT step + response selection (spec §2).
    #[inline]
    fn tick(&mut self, v0: f32, k: f32, a1: f32, a2: f32, a3: f32, resp: SvfResp) -> f32 {
        let v3 = v0 - self.ic2eq;
        let v1 = a1 * self.ic1eq + a2 * v3;
        let v2 = self.ic2eq + a2 * self.ic1eq + a3 * v3;
        self.ic1eq = 2.0 * v1 - self.ic1eq;
        self.ic2eq = 2.0 * v2 - self.ic2eq;
        match resp {
            SvfResp::Lp => v2,
            SvfResp::Bp => v1,
            SvfResp::Hp => v0 - k * v1 - v2,
            SvfResp::Notch => v0 - k * v1,
        }
    }
}

impl Default for Svf {
    fn default() -> Self {
        Svf::new()
    }
}

/// TPT coefficients from prewarped `g` and damping `k` (spec §2).
#[inline]
fn svf_coeffs(g: f32, k: f32) -> (f32, f32, f32) {
    let a1 = 1.0 / (1.0 + g * (g + k));
    let a2 = g * a1;
    let a3 = g * a2;
    (a1, a2, a3)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::In;
    use proptest::prelude::*;

    proptest! {
        /// P0 gate (spec §8): a stable one-pole never overshoots a bounded,
        /// constant input's range. For any cutoff and constant input in
        /// [-1, 1], every output sample over a block is finite and stays
        /// within [-1.001, 1.001].
        #[test]
        fn onepole_output_is_finite_and_bounded(
            cutoff in 1.0f32..=20_000.0,
            x in -1.0f32..=1.0,
        ) {
            let mut f = OnePole::new();
            let mut out = [0.0f32; 128];
            let dt = 1.0 / 48_000.0;
            f.process(In::K(x), In::K(cutoff), dt, &mut out);
            for s in out {
                prop_assert!(s.is_finite());
                prop_assert!(s >= -1.001 && s <= 1.001);
            }
        }
    }

    #[test]
    fn onepole_lowpass_attenuates_a_step_toward_it() {
        let mut f = OnePole::new();
        let mut out = [0.0f32; 64];
        // Step input 1.0, cutoff 100 Hz at 44.1k: output rises toward 1 but lags.
        f.process(In::K(1.0), In::K(100.0), 1.0 / 44_100.0, &mut out);
        assert!(out[0] > 0.0 && out[0] < 0.1); // first sample only partway
        assert!(out[63] > out[0]); // monotonically approaching
        assert!(out[63] < 1.0);
    }
}

#[cfg(test)]
mod svf_tests {
    extern crate std;

    use super::*;
    use crate::In;
    use deluge_dsp_test::filter_meas::{magnitude_db, minus_3db_hz, self_osc_hz_and_rms};
    use proptest::prelude::*;

    const FS: f32 = 48_000.0;
    const DT: f32 = 1.0 / FS;

    // Render N samples of `resp` at constant cutoff/res into `buf`, from a
    // fresh filter, given an input closure.
    fn render(resp: SvfResp, cutoff: f32, res: f32, input: impl Fn(usize) -> f32, buf: &mut [f32]) {
        let mut f = Svf::new();
        // feed sample-by-sample as an audio-rate input buffer
        let x: std::vec::Vec<f32> = (0..buf.len()).map(&input).collect();
        f.process(In::A(&x), In::K(cutoff), In::K(res), resp, DT, buf);
    }

    proptest! {
        #![proptest_config(ProptestConfig { cases: 128, ..ProptestConfig::default() })]
        #[test]
        fn svf_is_finite_and_bounded(
            cutoff in 20.0f32..=20_000.0,
            res in 0.0f32..=1.0,
            amp in 0.0f32..=1.0,
        ) {
            for resp in [SvfResp::Lp, SvfResp::Hp, SvfResp::Bp, SvfResp::Notch] {
                let mut out = [0.0f32; 256];
                render(resp, cutoff, res, |i| amp * (0.03 * i as f32).sin(), &mut out);
                for s in out {
                    prop_assert!(s.is_finite());
                    prop_assert!(s.abs() <= 4.0, "resp={:?} s={s}", resp);
                }
            }
        }
    }

    fn mag_at(resp: SvfResp, cutoff: f32, res: f32, probe: f32) -> f32 {
        magnitude_db(FS, probe, |buf| {
            let mut f = Svf::new();
            let x: std::vec::Vec<f32> = (0..buf.len())
                .map(|i| (core::f32::consts::TAU * probe / FS * i as f32).sin())
                .collect();
            f.process(In::A(&x), In::K(cutoff), In::K(res), resp, DT, buf);
        })
    }

    #[test]
    fn lp_attenuates_highs_hp_attenuates_lows() {
        let fc = 1_000.0;
        // LP: passband well above stopband
        assert!(mag_at(SvfResp::Lp, fc, 0.2, 100.0) - mag_at(SvfResp::Lp, fc, 0.2, 10_000.0) > 30.0);
        // HP: mirror
        assert!(mag_at(SvfResp::Hp, fc, 0.2, 10_000.0) - mag_at(SvfResp::Hp, fc, 0.2, 100.0) > 30.0);
        // Notch: deep dip at fc, ~unity an octave away
        assert!(mag_at(SvfResp::Notch, fc, 0.2, fc) < -12.0);
        assert!(mag_at(SvfResp::Notch, fc, 0.2, fc / 4.0) > -2.0);
    }

    #[test]
    fn lp_minus_3db_point_tracks_cutoff() {
        // Probe at the Butterworth resonance (Q=1/sqrt(2)), where the 2-pole
        // TPT lowpass's -3dB point sits exactly at `fc` by construction —
        // unlike `res=0` (k=2, Q=0.5, overdamped), which is not at fc and
        // forces a weak/asymmetric tolerance to accommodate. `k = 2(1-res)`,
        // so `k = sqrt(2)` (Butterworth) gives `res = 1 - sqrt(2)/2`.
        // Measured: fc=300 -> got=299.94; fc=1000 -> got=1015.70;
        // fc=4000 -> got=3981.68. Tolerance kept tight (0.1) since the
        // theoretical deviation is zero; the small residual is purely the
        // harness's log-sweep quantization (~2.4% per step).
        let res = 1.0 - core::f32::consts::FRAC_1_SQRT_2;
        for &fc in &[300.0f32, 1_000.0, 4_000.0] {
            let got = minus_3db_hz(FS, |probe, buf| {
                let mut f = Svf::new();
                let x: std::vec::Vec<f32> = (0..buf.len())
                    .map(|i| (core::f32::consts::TAU * probe / FS * i as f32).sin())
                    .collect();
                f.process(In::A(&x), In::K(fc), In::K(res), SvfResp::Lp, DT, buf);
            });
            assert!((got - fc).abs() / fc < 0.1, "fc={fc} got={got}");
        }
    }

    #[test]
    fn resonance_boosts_peak_gain() {
        let fc = 1_000.0;
        let low = mag_at(SvfResp::Bp, fc, 0.1, fc);
        let high = mag_at(SvfResp::Bp, fc, 0.9, fc);
        assert!(high - low > 6.0, "res peak low={low} high={high}");
    }

    #[test]
    fn self_oscillates_at_max_res() {
        let fc = 1_000.0;
        let (hz, rms) = self_osc_hz_and_rms(FS, |buf| {
            let mut f = Svf::new();
            // tiny impulse to excite, no sustained input
            let mut x = std::vec![0.0f32; buf.len()];
            x[0] = 1.0;
            f.process(In::A(&x), In::K(fc), In::K(1.0), SvfResp::Bp, DT, buf);
        });
        assert!(rms > 1e-3, "self-osc rms={rms} (should sustain)");
        assert!((hz - fc).abs() / fc < 0.25, "self-osc hz={hz}");
    }
}
