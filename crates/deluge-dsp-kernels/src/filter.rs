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
    pub(crate) fn tick(&mut self, v0: f32, k: f32, a1: f32, a2: f32, a3: f32, resp: SvfResp) -> f32 {
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
pub(crate) fn svf_coeffs(g: f32, k: f32) -> (f32, f32, f32) {
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

/// Moog ladder oversampling (Heun/RK2 integration sub-steps). Huovilainen-canonical 2×.
pub(crate) const MOOG_OVERSAMPLE: u32 = 2;

/// Per-slope feedback scale (loop gain = `res·MOOG_K·4`). The 4-pole reaches clean
/// self-oscillation at loop gain 4 (`MOOG_K4 = 1`, so `res=1` self-oscillates). The 2-pole
/// cannot self-oscillate (two cascaded one-poles reach 180° phase only at ∞) — it resonates
/// modestly; `MOOG_K2 = 1` gives it the same feedback scale. Kept per-slope for future tuning.
const MOOG_K4: f32 = 1.0;
const MOOG_K2: f32 = 1.0;

#[inline]
const fn moog_k(poles: usize) -> f32 {
    if poles <= 2 {
        MOOG_K2
    } else {
        MOOG_K4
    }
}

/// Authentic Moog transistor-ladder (Huovilainen) — a thin wrapper over the Fi-2
/// `DiodeLadder` (already the tanh one-pole cascade), adding resonance compensation and a
/// drive gain. `POLES` selects the slope (4 = 24 dB/oct, 2 = 12 dB/oct).
#[derive(Clone, Copy)]
pub struct Moog<const POLES: usize> {
    ladder: DiodeLadder<POLES>,
    drive: f32,
}

impl<const POLES: usize> Moog<POLES> {
    pub fn new() -> Self {
        Moog { ladder: DiodeLadder::new(), drive: 1.0 }
    }

    /// Pre-ladder input gain into the tanh (control param; 1.0 = clean).
    pub fn set_drive(&mut self, d: f32) {
        self.drive = d.max(0.0);
    }

    pub fn process(&mut self, input: In, cutoff: In, res: In, dt: f32, out: &mut [f32]) {
        let two_pi_dt_os = 2.0 * core::f32::consts::PI * dt / MOOG_OVERSAMPLE as f32;
        for (i, s) in out.iter_mut().enumerate() {
            let cutoff = cutoff.at(i).clamp(20.0, 18_000.0);
            let res = res.at(i).clamp(0.0, 1.0);
            let fh = two_pi_dt_os * cutoff;
            let ladder_res = res * moog_k(POLES);
            // Huovilainen resonance compensation: the ladder feedback (loop gain
            // k = ladder_res·4) drops the passband gain by ≈1/(1+k); pre-scale the
            // input by (1+k) to hold it flat. Plus the drive gain into the tanh.
            let k = ladder_res * 4.0;
            let x = input.at(i) * self.drive * (1.0 + k);
            *s = self.ladder.process(x, fh, ladder_res, MOOG_OVERSAMPLE);
        }
    }
}

impl<const POLES: usize> Default for Moog<POLES> {
    fn default() -> Self {
        Moog::new()
    }
}

/// Moog/MS-20-style oversampling (Heun-free here — direct ZDF; OS is for clip aliasing).
pub(crate) const MS20_OVERSAMPLE: u32 = 2;

/// Output DC-blocker corner (Hz). The spec's suggested ~5–20 Hz is too slow to settle
/// within the `ms20_dc_blocker_keeps_output_centered` gate's measurement window (a
/// 4096-sample buffer, mean taken from sample 512 on — ~74.6 ms of averaging starting
/// 10.7 ms in): at drive=6 the loop's DC operating point is ≈ drive·input (≈3.0 for that
/// gate's 0.5 constant input), and a 10 Hz corner (τ≈15.9 ms) only decays that offset to
/// an average of ≈0.32 over the window — measured, and it fails the <0.05 gate. 30 Hz
/// (τ≈5.3 ms) decays it to a measured mean of ≈0.03–0.04, comfortably inside the gate,
/// while staying low enough not to color audio-rate cutoffs (≥800 Hz, well above this
/// corner) — see the report for a probe sweep confirming the passband is unaffected
/// at fc≥800 Hz. Note: a `minus_3db_hz`-style sweep (which always starts probing at
/// 20 Hz) is NOT usable to validate this kernel's cutoff accuracy while this corner is
/// active — 20 Hz sits close enough to a 30 Hz one-pole HP that it reads as already
/// past -3 dB (measured -5.1 dB) regardless of the LP core's own cutoff; that's the
/// DC blocker, not the resonant core, and is why `ms20_tests` doesn't include a
/// `minus_3db_hz`-based cutoff-accuracy gate (band/slope + self-osc-near-cutoff cover
/// cutoff tracking instead).
const MS20_DC_HP_HZ: f64 = 30.0;

/// Asymmetric diode-pair clipper (the MS-20 scream): softer on one polarity, harder on the
/// other → even harmonics + a DC offset (removed downstream by the DC blocker). Pure f32,
/// monotonic, bounded. STARTING FORM — tune the asymmetry/hardness against the even-harmonic
/// and self-osc gates. (A biased tanh: shift, shape, de-bias.)
#[inline]
pub(crate) fn ms20_clip(x: f32) -> f32 {
    // pade_tanh is symmetric; bias it to make one side clip earlier (asymmetry), then
    // subtract the bias's DC so small signals stay ~centered (DC blocker mops up the rest).
    const B: f32 = 0.5;
    pade_tanh(x + B) - pade_tanh(B)
}

/// Which Sallen-Key response a node writes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ms20Resp {
    Lp,
    Hp,
}

/// MS-20 (Korg35) Sallen-Key: a resonant 2-pole (TPT two-integrator core — the Sallen-Key
/// LP transfer is `K/(1+(3−K)a+a²)`, i.e. an SVF lowpass with `k=3−K`) whose resonance
/// feedback is nonlinearly clipped by a diode-pair saturator, with drive + a DC blocker.
#[derive(Clone, Copy)]
pub struct Ms20 {
    ic1eq: f32,
    ic2eq: f32,
    dc: OnePoleHp,
    drive: f32,
    cached_dt: f32,
}

impl Ms20 {
    pub fn new() -> Ms20 {
        Ms20 { ic1eq: 0.0, ic2eq: 0.0, dc: OnePoleHp::default(), drive: 1.0, cached_dt: 0.0 }
    }

    pub fn set_drive(&mut self, d: f32) {
        self.drive = d.max(0.0);
    }

    pub fn process(&mut self, input: In, cutoff: In, res: In, resp: Ms20Resp, dt: f32, out: &mut [f32]) {
        if dt != self.cached_dt {
            // DC blocker (removes the asymmetric clip's offset) — see MS20_DC_HP_HZ.
            self.dc.set_coeff(MS20_DC_HP_HZ * 2.0 * core::f64::consts::PI / (1.0 / dt as f64));
            self.cached_dt = dt;
        }
        for (i, s) in out.iter_mut().enumerate() {
            let fc = cutoff.at(i).clamp(20.0, 18_000.0);
            let res = res.at(i).clamp(0.0, 1.0);
            // res→k: k=2 (gentle) at res=0, k→~0 (self-osc edge, Sallen-Key K→3) at res=1.
            let k = (2.0 * (1.0 - res)).max(1e-4);
            // Prewarp at dt/MS20_OVERSAMPLE, not dt: the loop below sub-steps
            // MS20_OVERSAMPLE times per output sample, so each sub-step advances
            // physical time by dt/MS20_OVERSAMPLE, not a full dt (an earlier version of
            // this kernel prewarped at the full dt here, which advanced physical time by
            // MS20_OVERSAMPLE·dt per output sample instead — self-oscillation came out
            // pitched ~2× sharp, e.g. 1998 Hz measured for a 1000 Hz target).
            let theta = (core::f32::consts::PI * fc * dt / MS20_OVERSAMPLE as f32)
                .min(0.49 * core::f32::consts::PI);
            let g = svf_tan_prewarp(theta);
            let a1 = 1.0 / (1.0 + g * (g + k));
            let a2 = g * a1;
            let a3 = g * a2;
            let mut y = 0.0;
            for _ in 0..MS20_OVERSAMPLE {
                // The resonance feedback (bandpass v1) is where the diode clips (MS-20).
                // Apply the asymmetric clip to the resonance signal, iteration-free using
                // the current bandpass state estimate (ic1eq).
                let v0 = input.at(i) * self.drive - k * ms20_clip(self.drive * self.ic1eq);
                let v3 = v0 - self.ic2eq;
                let v1 = a1 * self.ic1eq + a2 * v3; // bandpass (resonance)
                let v2 = self.ic2eq + a2 * self.ic1eq + a3 * v3; // lowpass
                self.ic1eq = 2.0 * v1 - self.ic1eq;
                self.ic2eq = 2.0 * v2 - self.ic2eq;
                y = match resp {
                    Ms20Resp::Lp => v2,
                    Ms20Resp::Hp => v0 - k * v1 - v2,
                };
            }
            // Output soft-saturation (MS-20 output stage): only the resonance
            // *feedback* (the `k·ms20_clip(...)` term) is nonlinearly bounded going in —
            // the forward path (`drive·input`, and `v0` directly in the Hp tap) is not,
            // so under drive+resonance the pre-saturation output can reach tens-to-hundreds
            // (e.g. Lp measured ≈92 at cutoff=18000, res=0.99, drive=8, well inside the
            // tested domain — this is NOT limited to a narrow Hp corner). A hard clamp
            // there would flatten that into a digital rail; instead run it through the same
            // Padé tanh used for the resonance clip, scaled to ±8 (the kernel's contractual
            // bound), giving an analog-style output limiter: ~transparent at normal levels
            // (|y| ≲ 4), soft-clips heavy driven output. `pade_tanh` ∈ [-1,1], so this
            // bounds the output to (-8, 8) by construction — DC-block first, then saturate.
            *s = 8.0 * pade_tanh(self.dc.process(y) / 8.0);
        }
    }
}

impl Default for Ms20 {
    fn default() -> Self {
        Ms20::new()
    }
}

/// Number of modes in the modal resonator bank.
pub const MODAL_MODES: usize = 16;

/// Max inharmonicity (structure=1): `f_i = freq·i·√(1 + B·(i−1)²)`, `B = structure·MODAL_B_MAX`.
const MODAL_B_MAX: f32 = 0.5;

/// Slope of the `damping → k` curve (`k = SVF_K_MIN + damping²·MODAL_K_SLOPE`). Tuned by
/// measurement (see `modal_tests`) so damping=0.5 rings then clearly decays within an
/// 8192-sample buffer at 48 kHz, and higher damping decays measurably faster.
const MODAL_K_SLOPE: f32 = 0.4;

/// Output normalization numerator (`norm = MODAL_NORM_SCALE / active`) so a struck note is
/// loud but stays within the ±8 kernel bound. Tuned by measurement (see `modal_tests`).
const MODAL_NORM_SCALE: f32 = 0.5;

/// Modal resonator: a bank of `N` SVF-bandpass modes (own state each), struck by the input.
/// Reuses the `Svf` bandpass recurrence + coefficients. Rings-style controls.
#[derive(Clone, Copy)]
pub struct Modal<const N: usize> {
    modes: [(f32, f32); N], // (ic1eq, ic2eq) per mode
    structure: f32,         // [0,1] inharmonicity
    brightness: f32,        // [0,1] spectral tilt
    position: f32,          // [0,1] strike comb
}

impl<const N: usize> Modal<N> {
    pub fn new() -> Self {
        Modal { modes: [(0.0, 0.0); N], structure: 0.0, brightness: 0.7, position: 0.3 }
    }
    pub fn set_structure(&mut self, v: f32) {
        self.structure = v.clamp(0.0, 1.0);
    }
    pub fn set_brightness(&mut self, v: f32) {
        self.brightness = v.clamp(0.0, 1.0);
    }
    pub fn set_position(&mut self, v: f32) {
        self.position = v.clamp(0.0, 1.0);
    }

    pub fn process(&mut self, input: In, freq: In, damping: In, dt: f32, out: &mut [f32]) {
        let fbase = freq.at(0).clamp(20.0, 8_000.0);
        let damp = damping.at(0).clamp(0.0, 1.0);
        let b = self.structure * MODAL_B_MAX;
        // damping → k (SVF resonance): low damping → tiny k (high Q, long ring),
        // high damping → larger k (quick decay). Tuned by measurement (see
        // MODAL_K_SLOPE's doc) so mid damping (0.5) rings then clearly decays
        // within an 8192-sample buffer at 48 kHz.
        let k = SVF_K_MIN + damp * damp * MODAL_K_SLOPE;
        let theta_max = 0.49 * core::f32::consts::PI;
        let nyq = 0.49 / dt; // ~0.49·fs

        // Per-block per-mode coefficients + gains.
        let mut a1 = [0.0f32; N];
        let mut a2 = [0.0f32; N];
        let mut a3 = [0.0f32; N];
        let mut gain = [0.0f32; N];
        let mut active = 0.0f32;
        for i in 0..N {
            let n = (i + 1) as f32; // mode index 1..N
            let ratio = n * libm::sqrtf(1.0 + b * (n - 1.0) * (n - 1.0));
            let fi = fbase * ratio;
            if fi >= nyq {
                continue; // muted (gain stays 0), state left frozen
            }
            let theta = (core::f32::consts::PI * fi * dt).min(theta_max);
            let g = svf_tan_prewarp(theta);
            let (c1, c2, c3) = svf_coeffs(g, k);
            a1[i] = c1;
            a2[i] = c2;
            a3[i] = c3;
            // comb (strike position) × spectral tilt (brightness).
            let comb = libm::fabsf(libm::sinf(n * core::f32::consts::PI * self.position));
            let tilt = libm::powf(self.brightness, n - 1.0);
            gain[i] = comb * tilt;
            active += gain[i];
        }
        // Normalize so a struck note peaks near unity (tuned; see MODAL_NORM_SCALE).
        let norm = if active > 1e-6 { MODAL_NORM_SCALE / active } else { 0.0 };

        for (j, s) in out.iter_mut().enumerate() {
            let x = input.at(j);
            let mut y = 0.0f32;
            for i in 0..N {
                // Nyquist-muted modes (fi >= nyq) were `continue`'d in the coeff loop
                // above, leaving a1[i]==0.0 — their recurrence is invalid, so skip
                // entirely. Comb-nulled modes (gain[i]==0.0 from position/brightness)
                // have valid coefficients and must still tick so their (ic1eq,ic2eq)
                // state decays; only their output contribution (gain[i]*v1 = 0) is
                // silent. Otherwise a later control-rate change that un-nulls the mode
                // would resume from stale frozen state (a pop).
                if a1[i] == 0.0 {
                    continue;
                }
                let (ic1, ic2) = self.modes[i];
                let v3 = x - ic2;
                let v1 = a1[i] * ic1 + a2[i] * v3; // bandpass
                let v2 = ic2 + a2[i] * ic1 + a3[i] * v3;
                self.modes[i] = (2.0 * v1 - ic1, 2.0 * v2 - ic2);
                y += gain[i] * v1;
            }
            // Output soft-saturator: transparent at normal levels (|y·norm| ≲ 4), soft-clips a
            // driven-into-resonance-catastrophe signal instead of running away, and bounds the
            // output to (−8, 8) by construction (pade_tanh ∈ [−1,1]). A driven resonator saturating
            // is physically realistic.
            *s = 8.0 * pade_tanh(y * norm / 8.0);
        }
    }
}

impl<const N: usize> Default for Modal<N> {
    fn default() -> Self {
        Modal::new()
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

#[cfg(test)]
mod moog_tests {
    extern crate std;
    use super::*;
    use crate::In;
    use deluge_dsp_test::filter_meas::{magnitude_db, self_osc_hz_and_rms};
    use deluge_dsp_test::spectrum;
    use proptest::prelude::*;

    const FS: f32 = 48_000.0;
    const DT: f32 = 1.0 / FS;

    proptest! {
        #![proptest_config(ProptestConfig { cases: 96, ..ProptestConfig::default() })]
        #[test]
        fn moog_is_finite_and_bounded(
            cutoff in 20.0f32..=18_000.0,
            res in 0.0f32..=1.0,
            drive in 1.0f32..=8.0,
            amp in 0.0f32..=1.0,
        ) {
            let x: std::vec::Vec<f32> = (0..512).map(|i| amp * (0.05 * i as f32).sin()).collect();
            let mut out = [0.0f32; 512];
            let mut m = Moog::<4>::new();
            m.set_drive(drive);
            m.process(In::A(&x), In::K(cutoff), In::K(res), DT, &mut out);
            for s in out {
                prop_assert!(s.is_finite());
                prop_assert!(s.abs() <= 8.0, "s={s} cutoff={cutoff} res={res} drive={drive}");
            }
        }
    }

    #[test]
    fn moog4_self_oscillates_at_max_res() {
        let cutoff = 1_000.0f32;
        let (hz, rms) = self_osc_hz_and_rms(FS, |buf| {
            let mut x = std::vec![0.0f32; buf.len()];
            x[0] = 1.0;
            Moog::<4>::new().process(In::A(&x), In::K(cutoff), In::K(1.0), DT, buf);
        });
        assert!(rms > 1e-3, "4-pole self-osc rms={rms}");
        assert!((hz - cutoff).abs() / cutoff < 0.4, "4-pole self-osc hz={hz}");
    }

    // Passband level (well below cutoff) in dB — what the resonance compensation
    // (`MOOG_COMP`) preserves: without it the ladder's feedback drops the passband
    // gain (≈1/(1+k), k=res·4) as resonance rises.
    fn moog4_passband_db(cutoff: f32, res: f32) -> f32 {
        magnitude_db(FS, 100.0, |buf| {
            let x: std::vec::Vec<f32> = (0..buf.len())
                .map(|i| (core::f32::consts::TAU * 100.0 / FS * i as f32).sin())
                .collect();
            Moog::<4>::new().process(In::A(&x), In::K(cutoff), In::K(res), DT, buf);
        })
    }

    #[test]
    fn moog4_passband_level_stable_across_resonance() {
        // MOOG_COMP holds the passband level ~constant as resonance rises.
        let cutoff = 2_000.0f32; // well above the 100 Hz probe
        let lo = moog4_passband_db(cutoff, 0.1);
        for &res in &[0.3f32, 0.6, 0.9] {
            let db = moog4_passband_db(cutoff, res);
            assert!((db - lo).abs() < 3.0, "passband drift at res={res}: {db} dB vs {lo} dB");
        }
    }

    fn moog_mag_db<const P: usize>(cutoff: f32, res: f32, probe: f32) -> f32 {
        magnitude_db(FS, probe, |buf| {
            let x: std::vec::Vec<f32> = (0..buf.len())
                .map(|i| (core::f32::consts::TAU * probe / FS * i as f32).sin())
                .collect();
            Moog::<P>::new().process(In::A(&x), In::K(cutoff), In::K(res), DT, buf);
        })
    }

    // Stopband rolloff over one octave (2×cutoff → 4×cutoff), low res, via clean
    // sine magnitude probes (an impulse train's lines miss these frequencies).
    fn slope_db_per_oct<const P: usize>(cutoff: f32) -> f32 {
        moog_mag_db::<P>(cutoff, 0.1, 4.0 * cutoff) - moog_mag_db::<P>(cutoff, 0.1, 2.0 * cutoff)
    }

    #[test]
    fn moog_slopes_are_24_and_12_db_per_oct() {
        let s4 = slope_db_per_oct::<4>(800.0);
        let s2 = slope_db_per_oct::<2>(800.0);
        assert!(s4 < -18.0, "4-pole slope {s4} dB/oct (expect ~ -24)");
        assert!(s2 < -8.0 && s2 > -18.0, "2-pole slope {s2} dB/oct (expect ~ -12)");
        assert!(s4 < s2 - 6.0, "4-pole must roll off steeper than 2-pole ({s4} vs {s2})");
    }

    #[test]
    fn moog2_resonates_strongly() {
        // A 2-pole ladder (two cascaded one-poles) reaches only 180° phase at ∞,
        // so — unlike the 4-pole — it resonates/rings strongly at high resonance
        // but does NOT sustain a self-oscillation. Assert the resonant peak near
        // cutoff grows substantially from low to high resonance.
        let cutoff = 1_000.0f32;
        let peak_db = |res: f32| {
            magnitude_db(FS, cutoff, |buf| {
                let x: std::vec::Vec<f32> = (0..buf.len())
                    .map(|i| (core::f32::consts::TAU * cutoff / FS * i as f32).sin())
                    .collect();
                Moog::<2>::new().process(In::A(&x), In::K(cutoff), In::K(res), DT, buf);
            })
        };
        // The 2-pole is the gentler mode — it resonates modestly (no self-osc);
        // the peak at cutoff rises measurably with resonance.
        assert!(peak_db(0.9) > peak_db(0.1) + 2.0, "2-pole resonant peak should grow with res");
    }

    #[test]
    fn drive_adds_harmonics() {
        let f0 = 220.0f32;
        let thd_at = |drive: f32| {
            let spec = spectrum::analyze(FS, |buf| {
                let x: std::vec::Vec<f32> = (0..buf.len())
                    .map(|i| 0.5 * (core::f32::consts::TAU * f0 / FS * i as f32).sin())
                    .collect();
                let mut m = Moog::<4>::new();
                m.set_drive(drive);
                m.process(In::A(&x), In::K(4_000.0), In::K(0.3), DT, buf);
            });
            spec.thd(f0, 8)
        };
        assert!(thd_at(4.0) > thd_at(1.0) + 0.02, "drive should raise THD");
    }
}

#[cfg(test)]
mod ms20_tests {
    extern crate std;
    use super::*;
    use crate::In;
    use deluge_dsp_test::filter_meas::{magnitude_db, self_osc_hz_and_rms};
    use deluge_dsp_test::spectrum;
    use proptest::prelude::*;

    const FS: f32 = 48_000.0;
    const DT: f32 = 1.0 / FS;

    proptest! {
        #![proptest_config(ProptestConfig { cases: 96, ..ProptestConfig::default() })]
        #[test]
        fn ms20_is_finite_and_bounded(
            cutoff in 20.0f32..=18_000.0,
            res in 0.0f32..=1.0,
            drive in 1.0f32..=8.0,
            amp in 0.0f32..=1.0,
        ) {
            for resp in [Ms20Resp::Lp, Ms20Resp::Hp] {
                let x: std::vec::Vec<f32> = (0..512).map(|i| amp * (0.05 * i as f32).sin()).collect();
                let mut out = [0.0f32; 512];
                let mut f = Ms20::new();
                f.set_drive(drive);
                f.process(In::A(&x), In::K(cutoff), In::K(res), resp, DT, &mut out);
                for s in out {
                    prop_assert!(s.is_finite());
                    prop_assert!(s.abs() <= 8.0, "resp={resp:?} s={s} cutoff={cutoff} res={res} drive={drive}");
                }
            }
        }
    }

    fn self_osc(resp: Ms20Resp, cutoff: f32) -> (f32, f32) {
        self_osc_hz_and_rms(FS, |buf| {
            let mut x = std::vec![0.0f32; buf.len()];
            x[0] = 1.0;
            Ms20::new().process(In::A(&x), In::K(cutoff), In::K(1.0), resp, DT, buf);
        })
    }

    #[test]
    fn ms20_self_oscillates_both_modes() {
        for resp in [Ms20Resp::Lp, Ms20Resp::Hp] {
            let (hz, rms) = self_osc(resp, 1_000.0);
            assert!(rms > 1e-3, "{resp:?} self-osc rms={rms}");
            assert!((hz - 1_000.0).abs() / 1_000.0 < 0.5, "{resp:?} self-osc hz={hz}");
        }
    }

    fn spec_at(resp: Ms20Resp, cutoff: f32, res: f32, drive: f32, f0: f32) -> spectrum::Spectrum {
        spectrum::analyze(FS, |buf| {
            let x: std::vec::Vec<f32> = (0..buf.len())
                .map(|i| 0.5 * (core::f32::consts::TAU * f0 / FS * i as f32).sin())
                .collect();
            let mut f = Ms20::new();
            f.set_drive(drive);
            f.process(In::A(&x), In::K(cutoff), In::K(res), resp, DT, buf);
        })
    }

    #[test]
    fn ms20_asymmetric_clip_makes_even_harmonics() {
        // Driven, the asymmetric clip produces a 2nd harmonic (symmetric ladders suppress it).
        let f0 = 300.0f32;
        let spec = spec_at(Ms20Resp::Lp, 3_000.0, 0.4, 6.0, f0);
        let h1 = spec.level_at(f0).max(1e-9);
        let h2 = spec.level_at(2.0 * f0);
        assert!(20.0 * (h2 / h1).log10() > -40.0, "2nd harmonic too weak: {}", 20.0 * (h2 / h1).log10());
    }

    #[test]
    fn ms20_dc_blocker_keeps_output_centered() {
        // Despite the asymmetric clip, the sustained output mean (DC) is ~0.
        let mut out = [0.0f32; 4096];
        let x = std::vec![0.5f32; 4096];
        let mut f = Ms20::new();
        f.set_drive(6.0);
        f.process(In::A(&x), In::K(2_000.0), In::K(0.6), Ms20Resp::Lp, DT, &mut out);
        let mean: f32 = out[512..].iter().sum::<f32>() / (out.len() - 512) as f32;
        assert!(mean.abs() < 0.05, "output DC not blocked: mean={mean}");
    }

    #[test]
    fn ms20_lp_hp_bands_and_slope() {
        let mag = |resp, probe| magnitude_db(FS, probe, |buf| {
            let x: std::vec::Vec<f32> = (0..buf.len())
                .map(|i| (core::f32::consts::TAU * probe / FS * i as f32).sin())
                .collect();
            Ms20::new().process(In::A(&x), In::K(1_000.0), In::K(0.2), resp, DT, buf);
        });
        // LP passes lows over highs; HP the reverse; ~2-pole (≈12 dB/oct) rolloff.
        assert!(mag(Ms20Resp::Lp, 200.0) - mag(Ms20Resp::Lp, 5_000.0) > 20.0);
        assert!(mag(Ms20Resp::Hp, 5_000.0) - mag(Ms20Resp::Hp, 200.0) > 20.0);
        let s = mag(Ms20Resp::Lp, 4_000.0) - mag(Ms20Resp::Lp, 2_000.0);
        assert!(s < -8.0 && s > -18.0, "LP stopband slope {s} dB/oct (~ -12)");
    }

    #[test]
    fn ms20_lp_passes_low_cutoff_passband() {
        // Regression coverage for a low cutoff: all other ms20_tests use cutoff=1000 Hz,
        // so the interaction between a low SVF cutoff and the 30 Hz output DC blocker
        // (MS20_DC_HP_HZ) was untested. Probe at 60 Hz — comfortably above the 30 Hz DC
        // corner, comfortably below a 300 Hz filter cutoff (a genuine passband point,
        // not the filter's own -3dB knee) — at low, non-resonant res (0.2, matching the
        // convention used by ms20_lp_hp_bands_and_slope).
        //
        // Measured at MS20_DC_HP_HZ=30: -1.97 dB — comfortably inside this gate, so the
        // corner was kept at 30 (see that const's doc comment for why it's not lower).
        // Note: closer to the filter's own cutoff (e.g. probing at 0.8x cutoff, as in an
        // fc=150/probe=120 Hz setup) reads far more attenuated (-8 to -4 dB depending on
        // res) — but that's the SVF core's own overdamped-region rolloff (confirmed by
        // measuring with the DC blocker corner pushed to ~0 Hz: attenuation barely moves,
        // e.g. -8.6 -> -8.3 dB), not the DC blocker, and isn't what this gate targets.
        let mag = magnitude_db(FS, 60.0, |buf| {
            let x: std::vec::Vec<f32> = (0..buf.len())
                .map(|i| (core::f32::consts::TAU * 60.0 / FS * i as f32).sin())
                .collect();
            Ms20::new().process(In::A(&x), In::K(300.0), In::K(0.2), Ms20Resp::Lp, DT, buf);
        });
        assert!(mag > -4.0, "60 Hz passband unexpectedly attenuated: {mag} dB");
    }

    #[test]
    fn ms20_drive_raises_thd() {
        // At res=0.3 (a non-resonant, "clean" operating point) the filter itself is nearly
        // linear, so THD stays small in absolute terms even at high drive — measured
        // thd(1.0)≈0.0004, thd(4.0)≈0.0199 (a ~50× relative rise). The Moog kernel's +0.02
        // absolute-delta threshold doesn't transfer (that filter's resonant nonlinearity is
        // stronger at the same operating point); +0.01 is set from this kernel's measured
        // delta with ~2× headroom, still comfortably above the near-zero baseline.
        let thd = |d| spec_at(Ms20Resp::Lp, 4_000.0, 0.3, d, 220.0).thd(220.0, 8);
        assert!(thd(4.0) > thd(1.0) + 0.01, "drive should raise THD");
    }
}

#[cfg(test)]
mod modal_tests {
    extern crate std;
    use super::*;
    use crate::In;
    use deluge_dsp_test::spectrum;
    use proptest::prelude::*;

    const FS: f32 = 48_000.0;
    const DT: f32 = 1.0 / FS;

    // Strike with a unit impulse; return (early_rms, late_rms) of the ring.
    fn strike(structure: f32, brightness: f32, position: f32, freq: f32, damping: f32) -> (f32, f32) {
        let mut m = Modal::<MODAL_MODES>::new();
        m.set_structure(structure);
        m.set_brightness(brightness);
        m.set_position(position);
        let mut x = std::vec![0.0f32; 8192];
        x[0] = 1.0;
        let mut out = [0.0f32; 8192];
        m.process(In::A(&x), In::K(freq), In::K(damping), DT, &mut out);
        let rms = |s: &[f32]| (s.iter().map(|v| v * v).sum::<f32>() / s.len() as f32).sqrt();
        (rms(&out[64..1088]), rms(&out[7168..8192]))
    }

    #[test]
    fn modal_rings_and_decays() {
        // A struck resonator rings (early energy) then decays (late ≪ early) at mid damping.
        // Measured at damping=0.5 (MODAL_K_SLOPE=0.4): early_rms≈2.628e-3, late_rms≈6.51e-8
        // (late/early ≈ 2.48e-5, well under the 0.5 gate).
        let (early, late) = strike(0.0, 0.7, 0.3, 220.0, 0.5);
        assert!(early > 1e-3, "no ring: early_rms={early}");
        assert!(late < early * 0.5, "did not decay: early={early} late={late}");
    }

    #[test]
    fn modal_decay_tracks_damping() {
        // Measured: ratio(0.2)≈0.0930 (long ring, still audible at buffer end),
        // ratio(0.8)≈3.4e-12 (fully decayed well before the late window) — clearly ordered.
        let ratio = |d| { let (e, l) = strike(0.0, 0.7, 0.3, 220.0, d); l / e.max(1e-9) };
        assert!(ratio(0.8) < ratio(0.2), "more damping should decay faster");
    }

    proptest! {
        #![proptest_config(ProptestConfig { cases: 96, ..ProptestConfig::default() })]
        #[test]
        fn modal_is_finite_and_bounded(
            freq in 20.0f32..=4_000.0,
            damping in 0.0f32..=1.0,
            structure in 0.0f32..=1.0,
            brightness in 0.0f32..=1.0,
            position in 0.0f32..=1.0,
            amp in 0.0f32..=1.0,
        ) {
            let mut m = Modal::<MODAL_MODES>::new();
            m.set_structure(structure); m.set_brightness(brightness); m.set_position(position);
            let x: std::vec::Vec<f32> = (0..512).map(|i| amp * (0.05 * i as f32).sin()).collect();
            let mut out = [0.0f32; 512];
            m.process(In::A(&x), In::K(freq), In::K(damping), DT, &mut out);
            for s in out {
                prop_assert!(s.is_finite());
                prop_assert!(s.abs() <= 8.0, "s={s} freq={freq} damping={damping}");
            }
        }
    }

    fn strike_spec(structure: f32, brightness: f32, position: f32, freq: f32, damping: f32) -> spectrum::Spectrum {
        spectrum::analyze(FS, |buf| {
            let mut m = Modal::<MODAL_MODES>::new();
            m.set_structure(structure); m.set_brightness(brightness); m.set_position(position);
            let mut x = std::vec![0.0f32; buf.len()];
            x[0] = 1.0;
            m.process(In::A(&x), In::K(freq), In::K(damping), DT, buf);
        })
    }

    #[test]
    fn modal_has_peak_at_fundamental() {
        let spec = strike_spec(0.0, 0.9, 0.25, 220.0, 0.1);
        // fundamental is a dominant spectral component
        assert!(spec.level_at(220.0) > spec.level_at(220.0 * 1.5) * 2.0, "no fundamental peak");
    }

    #[test]
    fn modal_structure_stretches_partials() {
        // structure=0: 2nd partial ≈ 2×freq (harmonic). structure=1: stretched (> 2×freq).
        // Measured: ratio_at(0.0)≈1.991, ratio_at(1.0)≈2.445 (matches theory closely: at
        // n=2, B=MODAL_B_MAX=0.5, ratio=2·√(1+0.5)=2.449) — clears the ">2.0+0.3" gate.
        let f0 = 220.0f32;
        let ratio_at = |structure| {
            let spec = strike_spec(structure, 0.9, 0.25, f0, 0.1);
            // find the strongest bin between 1.5×f0 and 3.5×f0 (the 2nd partial region)
            let (lo, hi) = ((1.5 * f0) as usize, (3.5 * f0) as usize);
            let mut best = lo; let mut best_lvl = 0.0f32;
            for hz in (lo..hi).step_by(2) {
                let l = spec.level_at(hz as f32);
                if l > best_lvl { best_lvl = l; best = hz; }
            }
            best as f32 / f0
        };
        assert!((ratio_at(0.0) - 2.0).abs() < 0.3, "structure=0 2nd partial not ~2× : {}", ratio_at(0.0));
        assert!(ratio_at(1.0) > ratio_at(0.0) + 0.3, "structure=1 should stretch: {} vs {}", ratio_at(1.0), ratio_at(0.0));
    }

    #[test]
    fn modal_position_nulls_a_mode() {
        // position=0.5 → |sin(2π·0.5)|=0 → mode 2 (~2×freq) suppressed vs position=0.25.
        // Measured: p25≈6.398, p50≈2.62e-5 (ratio ≈ 4.1e-6, far under the 0.3 gate) — the
        // exact structural null, as expected.
        let f0 = 220.0f32;
        let p25 = strike_spec(0.0, 0.9, 0.25, f0, 0.1).level_at(2.0 * f0);
        let p50 = strike_spec(0.0, 0.9, 0.5, f0, 0.1).level_at(2.0 * f0);
        assert!(p50 < p25 * 0.3, "position=0.5 should null mode 2: p25={p25} p50={p50}");
    }

    #[test]
    fn modal_brightness_shifts_spectrum() {
        // Higher brightness → more high-partial energy (4th partial louder).
        //
        // Strike position 0.3, NOT the 0.25 used by the other modal_tests: the comb weight
        // is `|sin(n·π·position)|` (the exact mechanism modal_position_nulls_a_mode checks
        // for n=2), and at position=0.25 that comb is an EXACT structural zero at n=4
        // (`|sin(4·π·0.25)| = |sin(π)| = 0`, confirmed to float precision — 1.2e-16) — every
        // multiple of 4 sits on a comb node. That's not a bank defect, it's the same
        // by-construction null the position gate validates; it just collides with probing
        // the 4th partial specifically. Measured with the bank unchanged: at position=0.25
        // dull=1.7e-5, bright=1.5e-5 — both pinned to FFT leakage/noise floor, no brightness
        // effect reachable (mode 4 is silenced regardless of tilt). At position=0.3
        // (comb_4=|sin(1.2π)|≈0.588, no coincidental null for n≤4) the same bank measures
        // dull≈0.2156, bright≈3.158 (~14.7×) — well past the 2× gate.
        let f0 = 220.0f32;
        let dull = strike_spec(0.0, 0.2, 0.3, f0, 0.1).level_at(4.0 * f0);
        let bright = strike_spec(0.0, 0.95, 0.3, f0, 0.1).level_at(4.0 * f0);
        assert!(bright > dull * 2.0, "brightness should raise high partials: dull={dull} bright={bright}");
    }

    #[test]
    fn modal_bounded_under_sustained_on_resonance_drive() {
        // Drive continuously at the fundamental with damping=0 (k floored) — the worst
        // case for resonant buildup. Must stay bounded (the output saturator handles it).
        let mut m = Modal::<MODAL_MODES>::new();
        m.set_brightness(0.9);
        let freq = 220.0f32;
        let x: std::vec::Vec<f32> = (0..48_000)
            .map(|i| (core::f32::consts::TAU * freq / FS * i as f32).sin())
            .collect();
        let mut out = [0.0f32; 48_000];
        m.process(In::A(&x), In::K(freq), In::K(0.0), DT, &mut out);
        for s in out {
            assert!(s.is_finite() && s.abs() <= 8.0, "sustained drive unbounded: s={s}");
        }
    }
}
