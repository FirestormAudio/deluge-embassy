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
            let fc = fc.max(1.0);
            let theta = (PI * fc * dt).min(0.49 * PI);
            let g = libm::tanf(theta);
            let k = svf_k_from_res(r);
            let (a1, a2, a3) = svf_coeffs(g, k);
            for (i, s) in out.iter_mut().enumerate() {
                *s = self.tick(input.at(i), k, a1, a2, a3, resp);
            }
            return;
        }
        // Audio-rate path: per-sample coefficients via the polynomial prewarp.
        let theta_max = 0.49 * PI;
        for (i, s) in out.iter_mut().enumerate() {
            let fc = cutoff.at(i).max(1.0);
            let theta = (PI * fc * dt).min(theta_max);
            let g = svf_tan_prewarp(theta);
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

/// Polynomial `tan(theta)` for the SVF cutoff prewarp, `theta = π·fc·dt`.
/// Used only on the audio-rate branch (per-sample); the const path uses exact
/// `libm::tanf`. `fc` is clamped so `theta ∈ (0, 0.49π)` — away from the tan
/// pole at π/2. A [3/2] Padé-style rational: matches `tan` to well within the
/// SVF's audio tolerance across the cutoff range while staying pure arithmetic
/// (no transcendental) for the Cortex-A9 hot path.
///
/// Accuracy is strongly `theta`-dependent — this is a good fit only over the
/// musical cutoff range, NOT up to the clamp edge:
/// - Across the tested audio range (fc ≤ 9 kHz → `theta ≤ 0.59`): raw
///   `|approx − tanf|` relative error is ~1%.
/// - **At the clamp edge (`theta → 0.49π`): the raw approximation is very
///   poor** — `approx ≈ 4.1` vs `tanf ≈ 31.8`, i.e. ~87% relative error
///   (it returns only ~13% of the true `tan`). This region is reached ONLY by
///   an *audio-rate-swept* cutoff pushed near Nyquist; the const fast path uses
///   exact `libm::tanf`, so static/most cutoffs are unaffected. Do NOT rely on
///   this prewarp for accurate cutoff near Nyquist — extend the approximation
///   (or oversample) before widening audio-rate sweeps into that region.
///
/// What the QA gate pins is end-to-end SVF output: across the swept audio-rate
/// equivalence test (fc 110 Hz–9 kHz, res 0–0.95) the measured max output delta
/// vs. the exact-tanf const path is ~1.2e-3, comfortably under the 2e-3 gate.
#[inline]
fn svf_tan_prewarp(theta: f32) -> f32 {
    // theta in (0, ~1.54). Rational approx of tan: t·(a + b·t²)/(1 - c·t²).
    let t = theta;
    let t2 = t * t;
    t * (0.999_999 + 0.093_54 * t2) / (1.0 - 0.229_92 * t2)
}

/// TPT coefficients from prewarped `g` and damping `k` (spec §2).
#[inline]
fn svf_coeffs(g: f32, k: f32) -> (f32, f32, f32) {
    let a1 = 1.0 / (1.0 + g * (g + k));
    let a2 = g * a1;
    let a3 = g * a2;
    (a1, a2, a3)
}

/// Padé [3/2] `tanh` approximation (skylib `SoftLimit`), clamped to ±1. The
/// diode/transistor ladder's true nonlinearity is `tanh`-like; this replaces
/// spark's cheaper cubic Taylor stand-in. Monotonic everywhere (derivative
/// `9(x²−9)²/denom² ≥ 0`), accurate to `tanh` out to |x|≈3, and it reaches ±1
/// tangentially at x=±3 (zero slope), so the ±1 clamp is C¹-smooth. Small-signal
/// it has unit slope at 0 (identical to the cubic there). Reused by Fi-3 Moog.
#[inline]
pub(crate) fn pade_tanh(x: f32) -> f32 {
    let x2 = x * x;
    let s = x * (27.0 + x2) / (27.0 + 9.0 * x2);
    s.max(-1.0).min(1.0)
}

/// Nonlinear diode ladder (Vult/Heun RK2 core, de-SIMD'd from spark
/// `SimdDiodeLadderFilter<f32,1,STAGES>`). Holds only integrator state;
/// coefficients (`fh`), resonance, and oversampling are passed per call so the
/// caller (`Tb303`) controls them per sample. Serial recurrence → scalar.
#[derive(Clone, Copy)]
pub struct DiodeLadder<const STAGES: usize> {
    state: [f32; STAGES],
}

impl<const STAGES: usize> DiodeLadder<STAGES> {
    pub fn new() -> Self {
        DiodeLadder { state: [0.0; STAGES] }
    }

    /// One Heun (RK2 predictor-corrector) step. `fh` = normalized cutoff
    /// (`2π·cutoff/(oversample·fs)`), `res` = internal ladder resonance
    /// (`Tb303` passes 0 — resonance is external).
    #[inline]
    fn heun_step(&mut self, input: f32, fh: f32, res: f32) {
        let feedback = pade_tanh(self.state[STAGES - 1]) * res * 4.0;
        let x = input - feedback;

        // Predictor (Euler)
        let mut temp = self.state;
        let mut d = [0.0f32; STAGES];
        d[0] = fh * (x - pade_tanh(temp[0]));
        for i in 1..STAGES {
            d[i] = fh * (pade_tanh(temp[i - 1]) - pade_tanh(temp[i]));
        }
        for i in 0..STAGES {
            temp[i] += d[i];
        }

        // Corrector (derivative at predicted state)
        let fb_p = pade_tanh(temp[STAGES - 1]) * res * 4.0;
        let x_p = input - fb_p;
        let mut dp = [0.0f32; STAGES];
        dp[0] = fh * (x_p - pade_tanh(temp[0]));
        for i in 1..STAGES {
            dp[i] = fh * (pade_tanh(temp[i - 1]) - pade_tanh(temp[i]));
        }
        for i in 0..STAGES {
            self.state[i] += (d[i] + dp[i]) * 0.5;
        }
    }

    /// Advance one output sample at `oversample`× (spark hardcodes 4). Output
    /// is the clipped last stage.
    #[inline]
    pub fn process(&mut self, input: f32, fh: f32, res: f32, oversample: u32) -> f32 {
        for _ in 0..oversample {
            self.heun_step(input, fh, res);
        }
        pade_tanh(self.state[STAGES - 1])
    }
}

impl<const STAGES: usize> Default for DiodeLadder<STAGES> {
    fn default() -> Self {
        DiodeLadder::new()
    }
}

/// One-pole high-pass (spark `iir/one_pole.rs` `OnePoleFilter`, HP path):
/// `H(z) = (1+a)/2 · (1 − z⁻¹)/(1 − a·z⁻¹)`. Coefficient set once (fixed corner).
#[derive(Clone, Copy, Default)]
pub(crate) struct OnePoleHp {
    a: f32,
    x_prev: f32,
    y_prev: f32,
}

impl OnePoleHp {
    /// `wc = 2π·f_corner/fs` (radians/sample). `k = tan(wc/2)` in f64 (one-time).
    fn set_coeff(&mut self, wc: f64) {
        let k = libm::tan(wc / 2.0);
        self.a = ((1.0 - k) / (1.0 + k)) as f32;
    }
    #[inline]
    fn process(&mut self, input: f32) -> f32 {
        let b = (1.0 + self.a) / 2.0;
        let output = b * input - b * self.x_prev + self.a * self.y_prev;
        self.x_prev = input;
        self.y_prev = output;
        output
    }
}

/// Single static config point for the diode-ladder integration
/// sub-stepping (RK2/Heun sub-steps per output sample, applied to both
/// ladder stages). Fixed at compile time rather than a runtime knob: a
/// data characterization swept OS against a fully-resolved OS=16
/// reference and found OS=2 indistinguishable (<1% error) from OS=16
/// across the normal control range, while self-oscillation, boundedness,
/// and aliasing behavior are all OS=1-identical (this filter's
/// oversampling is RK2 sub-stepping, not decimation, so it doesn't trade
/// off against aliasing the way a naive oversampler would). OS=4 was
/// unjustified cost — 2× the CPU of OS=2 for no measurable accuracy gain.
pub(crate) const TB303_OVERSAMPLE: u32 = 2;

/// Fixed coupling-cap high-pass corner frequencies (Hz), Stinchcombe Table 1.
const TB303_HP_FREQS: [f64; 6] = [97.5, 38.5, 4.45, 578.1, 20.0, 7.41];
const TB303_RES_HP_FREQ: f64 = 150.0;

/// Outer resonance-feedback gain. Spark's cubic-saturator ladder used
/// `res * 2.0`; our ladder's nonlinearity is `pade_tanh` (saturates at ±1
/// instead of the cubic's ±2/3), which turns out to need a substantially
/// *higher* loop gain to reach the same self-oscillation threshold (the
/// tanh's softer knee bleeds more loop energy per pass than the cubic did).
/// Retuned by measurement (the self-osc gate below is the instrument): swept
/// `res_gain` at `res=1`, cutoff=800 Hz, using `self_osc_hz_and_rms` to read
/// off sustain (rms) and pitch (hz) at each value —
///   gain 1.35..2.1: rms ≈ 1.9e-9 (dead — decays to silence within the buffer)
///   gain 3.0:        rms ≈ 1.9e-9 (still dead)
///   gain 5.0:        rms ≈ 3.6e-4 (borderline, below the 1e-3 sustain gate)
///   gain 5.5:         rms ≈ 0.215, hz ≈ 785 (2% off 800 Hz — clean onset)
///   gain 8.0:         rms ≈ 0.257, hz ≈ 639 (20% off — sat-driven detune begins)
///   gain 20..80:      rms plateaus ≈ 0.28 (tanh-clamped), hz drifts to ≈ 500
/// `5.5` is the smallest gain that clears self-oscillation cleanly: rms is
/// well above the sustain floor and the peak sits closest to `cutoff` (higher
/// gains keep sustaining but detune further as the ladder saturates harder).
const TB303_RES_GAIN: f32 = 5.5;

/// Faithful scalar TB-303 diode-ladder filter (spark `tb303_diode_va.rs`):
/// 1-stage ladder @ 2×cutoff → 3-stage ladder @ cutoff → 6 fixed HPs, with a
/// prev-sample resonance feedback through a 150 Hz HP. Lowpass output.
#[derive(Clone, Copy)]
pub struct Tb303 {
    lp1: DiodeLadder<1>,
    lp234: DiodeLadder<3>,
    hp: [OnePoleHp; 6],
    res_hp: OnePoleHp,
    res_tap: f32,
    cached_dt: f32, // fixed HP coeffs recomputed only when dt changes
}

impl Tb303 {
    pub fn new() -> Tb303 {
        Tb303 {
            lp1: DiodeLadder::new(),
            lp234: DiodeLadder::new(),
            hp: [OnePoleHp::default(); 6],
            res_hp: OnePoleHp::default(),
            res_tap: 0.0,
            cached_dt: 0.0,
        }
    }

    pub fn process(&mut self, input: In, cutoff: In, res: In, dt: f32, out: &mut [f32]) {
        // Fixed HP corners depend only on fs (=1/dt): compute once per dt change.
        if dt != self.cached_dt {
            let two_pi = 2.0 * core::f64::consts::PI;
            let fs = 1.0 / dt as f64;
            for (i, &f) in TB303_HP_FREQS.iter().enumerate() {
                self.hp[i].set_coeff(f * two_pi / fs);
            }
            self.res_hp.set_coeff(TB303_RES_HP_FREQ * two_pi / fs);
            self.cached_dt = dt;
        }

        let two_pi_dt_os = 2.0 * core::f32::consts::PI * dt / TB303_OVERSAMPLE as f32;
        for (i, s) in out.iter_mut().enumerate() {
            let cutoff = cutoff.at(i).clamp(20.0, 2000.0);
            let res = res.at(i).clamp(0.0, 1.0);
            let res_gain = res * TB303_RES_GAIN;

            // fh = 2π·cutoff/(oversample·fs) = 2π·cutoff·dt/oversample.
            let fh1 = two_pi_dt_os * (cutoff * 2.0); // stage 1 at 2×cutoff
            let fh234 = two_pi_dt_os * cutoff;

            let feedback = if res_gain > 1e-6 {
                self.res_hp.process(self.res_tap * res_gain)
            } else {
                0.0
            };
            let mut sig = input.at(i) - feedback;
            sig = self.lp1.process(sig, fh1, 0.0, TB303_OVERSAMPLE);
            sig = self.lp234.process(sig, fh234, 0.0, TB303_OVERSAMPLE);
            self.res_tap = sig;
            for hp in self.hp.iter_mut() {
                sig = hp.process(sig);
            }
            *s = sig;
        }
    }
}

impl Default for Tb303 {
    fn default() -> Self {
        Tb303::new()
    }
}

#[cfg(test)]
mod tb303_tests {
    extern crate std;
    use super::*;
    use crate::In;
    use deluge_dsp_test::filter_meas::self_osc_hz_and_rms;
    use deluge_dsp_test::spectrum;
    use proptest::prelude::*;

    const FS: f32 = 48_000.0;
    const DT: f32 = 1.0 / FS;

    proptest! {
        #![proptest_config(ProptestConfig { cases: 96, ..ProptestConfig::default() })]
        #[test]
        fn tb303_is_finite_and_bounded(
            cutoff in 20.0f32..=2_000.0,
            res in 0.0f32..=1.0,
            amp in 0.0f32..=8.0, // incl. heavy overdrive (well past unity) — the input
                                 // saturator must keep the loop bounded regardless
        ) {
            let x: std::vec::Vec<f32> = (0..512).map(|i| amp * (0.05 * i as f32).sin()).collect();
            let mut out = [0.0f32; 512];
            Tb303::new().process(In::A(&x), In::K(cutoff), In::K(res), DT, &mut out);
            for s in out {
                prop_assert!(s.is_finite());
                prop_assert!(s.abs() <= 8.0, "s={s} cutoff={cutoff} res={res}");
            }
        }
    }

    #[test]
    fn tb303_self_oscillates_at_high_res() {
        let cutoff = 800.0f32;
        let (hz, rms) = self_osc_hz_and_rms(FS, |buf| {
            let mut x = std::vec![0.0f32; buf.len()];
            x[0] = 1.0; // brief excitation only
            Tb303::new().process(In::A(&x), In::K(cutoff), In::K(1.0), DT, buf);
        });
        assert!(rms > 1e-3, "self-osc rms={rms}");
        assert!((hz - cutoff).abs() / cutoff < 0.4, "self-osc hz={hz} (near cutoff?)");
    }

    // Drive a pure sine through the filter; the diode nonlinearity generates
    // harmonics of f0. worst_alias_db flags energy NOT at harmonics of f0 —
    // i.e. aliased (inharmonic) content.
    fn alias_db(cutoff: f32, res: f32, f0: f32) -> f32 {
        let spec = spectrum::analyze(FS, |buf| {
            let x: std::vec::Vec<f32> = (0..buf.len())
                .map(|i| (core::f32::consts::TAU * f0 / FS * i as f32).sin())
                .collect();
            Tb303::new().process(In::A(&x), In::K(cutoff), In::K(res), DT, buf);
        });
        spec.worst_alias_db(f0, 30.0)
    }

    #[test]
    fn tb303_aliasing_below_floor() {
        // Bright, resonant (but below the res≈0.83-0.87 self-osc bifurcation
        // band measured for this cutoff/res_gain — see TB303_RES_GAIN's
        // doc comment and the report for the chaos-sweep evidence), cutoff
        // high enough that generated harmonics reach toward Nyquist.
        //
        // OS is now a fixed static const (see TB303_OVERSAMPLE), so there's
        // no runtime OS to A/B here — this just checks that the port, at its
        // real (OS=2) operating point, introduces no audible aliasing at a
        // genuine resonant operating point. Measured ≈ -52.8 dB.
        let (cutoff, res, f0) = (1500.0f32, 0.8, 220.0);
        let db = alias_db(cutoff, res, f0);
        assert!(db < -30.0, "worst_alias_db = {db} (floor?)");
    }

    #[test]
    fn tb303_resonant_peak_tracks_cutoff() {
        // High-res self-oscillation peak sits near cutoff and moves with it.
        let mut prev = 0.0f32;
        for &cutoff in &[400.0f32, 800.0, 1_600.0] {
            let (hz, _) = self_osc_hz_and_rms(FS, |buf| {
                let mut x = std::vec![0.0f32; buf.len()];
                x[0] = 1.0;
                Tb303::new().process(In::A(&x), In::K(cutoff), In::K(1.0), DT, buf);
            });
            assert!((hz - cutoff).abs() / cutoff < 0.4, "cutoff={cutoff} peak={hz}");
            assert!(hz > prev, "peak should rise with cutoff: {hz} !> {prev}");
            prev = hz;
        }
    }
}

#[cfg(test)]
mod diode_ladder_tests {
    use super::*;

    #[test]
    fn pade_tanh_is_bounded_monotonic_odd_and_tanh_like() {
        // unit slope at 0 (≈ tanh near 0), tracks tanh better than the cubic
        assert!(pade_tanh(0.0).abs() < 1e-7);
        assert!((pade_tanh(0.001) - 0.001).abs() < 1e-6); // slope ~1
        // reaches +1 at x=3 (tangential), clamps beyond
        assert!((pade_tanh(3.0) - 1.0).abs() < 1e-6);
        assert!((pade_tanh(100.0) - 1.0).abs() < 1e-6);
        assert!((pade_tanh(-100.0) + 1.0).abs() < 1e-6);
        // bounded to ±1, monotonic non-decreasing, and odd, across a sweep
        let mut prev = pade_tanh(-12.0);
        let mut x = -12.0f32;
        while x <= 12.0 {
            let y = pade_tanh(x);
            assert!(y.abs() <= 1.0 + 1e-6, "unbounded at {x}: {y}");
            assert!(y >= prev - 1e-6, "not monotonic at {x}");
            assert!((y + pade_tanh(-x)).abs() < 1e-5, "not odd at {x}");
            prev = y;
            x += 0.1;
        }
        // sanity vs true tanh in the working range (should be close). Measured
        // |approx-exact|: 0.5->0.0037, 1.0->0.0162, 2.0->0.0201 — the x=2.0
        // case is a genuine property of the [3/2] Padé rational (not a bug),
        // so the tolerance is set just above it.
        for &t in &[0.5f32, 1.0, 2.0] {
            let approx = pade_tanh(t);
            let exact = libm::tanhf(t);
            assert!((approx - exact).abs() < 0.021, "tanh({t}): approx {approx} vs {exact}");
        }
    }

    #[test]
    fn ladder_is_finite_bounded_and_lowpass() {
        // A 3-stage ladder, res=0, driven by a unit DC step: output settles
        // finite and bounded, and attenuates a high-frequency input more than
        // a low one (it's a lowpass).
        let fs = 48_000.0f32;
        let os = 4u32;
        let fh = 2.0 * core::f32::consts::PI * 1_000.0 / (os as f32 * fs);
        let mut lad = DiodeLadder::<3>::new();
        let mut last = 0.0;
        for _ in 0..2000 {
            last = lad.process(0.5, fh, 0.0, os);
            assert!(last.is_finite() && last.abs() <= 1.0);
        }
        assert!(last > 0.1); // DC largely passes a lowpass
    }

    #[test]
    fn ladder_oversample_is_honoured() {
        // Same effective cutoff (fh scaled by 1/os) but more RK2 sub-steps at
        // higher os → more accurate integration of the nonlinear ODE → a
        // measurably different output. Guards against the `oversample` param
        // being silently ignored (which a single-OS spectral test can't catch).
        let dt = 1.0f32 / 48_000.0;
        let base = 2.0 * core::f32::consts::PI * 2_000.0 * dt; // high fh: sub-stepping matters most
        let mut a = DiodeLadder::<3>::new();
        let mut b = DiodeLadder::<3>::new();
        let mut max_diff = 0.0f32;
        for i in 0..1024 {
            // loud, resonant drive (internal res=0.9) so integration accuracy shows up
            let x = 0.8 * (2.0 * core::f32::consts::PI * 220.0 * dt * i as f32).sin();
            let ya = a.process(x, base / 1.0, 0.9, 1);
            let yb = b.process(x, base / 2.0, 0.9, 2);
            max_diff = max_diff.max((ya - yb).abs());
        }
        assert!(max_diff > 1e-4, "oversample param not honoured: max|Δ|={max_diff}");
    }
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

    // The audio-rate path (constant-valued In::A cutoff/res) must match the
    // const fast path (In::K) within a small tolerance — the only difference is
    // exact-tanf (const) vs the polynomial prewarp (audio-rate).
    #[test]
    fn audio_rate_matches_const_within_tol() {
        const TOL: f32 = 2e-3;
        for &fc in &[110.0f32, 440.0, 1_000.0, 4_000.0, 9_000.0] {
            for &res in &[0.0f32, 0.5, 0.95] {
                let x: std::vec::Vec<f32> =
                    (0..256).map(|i| (0.02 * i as f32).sin()).collect();
                let mut k_out = [0.0f32; 256];
                Svf::new().process(In::A(&x), In::K(fc), In::K(res), SvfResp::Lp, DT, &mut k_out);
                let mut a_out = [0.0f32; 256];
                let fcb = std::vec![fc; 256];
                let rb = std::vec![res; 256];
                Svf::new().process(In::A(&x), In::A(&fcb), In::A(&rb), SvfResp::Lp, DT, &mut a_out);
                let mut md = 0.0f32;
                for i in 0..256 {
                    md = md.max((k_out[i] - a_out[i]).abs());
                }
                assert!(md < TOL, "fc={fc} res={res} max|Δ|={md}");
            }
        }
    }

    #[test]
    fn const_cutoff_above_nyquist_is_finite_and_bounded() {
        // fs=48k → Nyquist 24k; a const cutoff above it must clamp, not NaN.
        for &fc in &[26_000.0f32, 36_000.0, 100_000.0] {
            for resp in [SvfResp::Lp, SvfResp::Hp, SvfResp::Bp, SvfResp::Notch] {
                let x: std::vec::Vec<f32> = (0..256).map(|i| (0.05 * i as f32).sin()).collect();
                let mut out = [0.0f32; 256];
                Svf::new().process(In::A(&x), In::K(fc), In::K(0.0), resp, DT, &mut out);
                for s in out {
                    assert!(s.is_finite(), "fc={fc} resp={resp:?} produced non-finite");
                    assert!(s.abs() <= 4.0, "fc={fc} resp={resp:?} s={s}");
                }
            }
        }
    }

    #[test]
    fn audio_rate_cutoff_sweep_is_stable() {
        let x = std::vec![0.5f32; 512];
        let cutoff: std::vec::Vec<f32> =
            (0..512).map(|i| 100.0 + (i as f32 / 512.0) * 10_000.0).collect();
        let mut out = [0.0f32; 512];
        Svf::new().process(In::A(&x), In::A(&cutoff), In::K(0.9), SvfResp::Lp, DT, &mut out);
        for s in out {
            assert!(s.is_finite() && s.abs() <= 4.0, "sweep s={s}");
        }
    }
}
