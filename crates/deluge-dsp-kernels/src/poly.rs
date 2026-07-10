//! Polyphony kernels: a per-voice settable source (`PolyCtrl`), a poly
//! oscillator (`PolyOsc`), and the voice→mono collapse (`voice_sum`). All poly
//! buffers are voice-interleaved (sample-major): `tile[i * VOICES + v]` is voice
//! `v` at sample `i`, so the voice loop vectorizes to `f32x8` on NEON.

#[cfg(not(feature = "simd"))]
use crate::floorf;
use crate::env::Ar;
#[cfg(not(feature = "simd"))]
use crate::filter::{DiodeLadder, Ms20, Svf, SvfResp};
use crate::noise::{Noise, NoiseColor};
use crate::filter::{
    moog_coeffs, ms20_coeffs, svf_coeffs, svf_k_from_res, svf_tan_prewarp, Ms20Resp, MOOG_OVERSAMPLE,
};
#[cfg(not(feature = "simd"))]
use crate::osc::wave_sample;
use crate::osc::Wave;
#[cfg(feature = "simd")]
use crate::osc::wave_sample_x8;
use crate::quant::semitones_to_hz;
use crate::In;
use core::f32::consts::PI;

/// Voices processed in parallel per poly node. Fixed at compile time.
pub const VOICES: usize = 8;

/// Scalar SVF coefficients for a shared (mono) cutoff/res. Returns (k, a1, a2, a3).
/// Uses the polynomial prewarp (matches scalar `Svf`'s audio-rate path).
#[inline]
fn poly_svf_coeffs(fc: f32, res: f32, dt: f32) -> (f32, f32, f32, f32) {
    let theta = (PI * fc.max(1.0) * dt).min(0.49 * PI);
    let g = svf_tan_prewarp(theta);
    let k = svf_k_from_res(res);
    let (a1, a2, a3) = svf_coeffs(g, k);
    (k, a1, a2, a3)
}

/// A per-voice settable scalar source: lane `v` outputs `values[v]`. No input.
/// The allocator's write-target (Sy-3). Output tile is voice-interleaved.
#[derive(Clone, Copy)]
pub struct PolyCtrl {
    values: [f32; VOICES],
}
impl PolyCtrl {
    pub fn new() -> PolyCtrl {
        PolyCtrl { values: [0.0; VOICES] }
    }
    pub fn set_voice(&mut self, v: usize, x: f32) {
        if v < VOICES {
            self.values[v] = x;
        }
    }
    /// `out` is voice-interleaved, length `VOICES * n_samples`.
    pub fn process(&mut self, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            for v in 0..VOICES {
                out[i * VOICES + v] = self.values[v];
            }
        }
    }
}
impl Default for PolyCtrl {
    fn default() -> Self {
        Self::new()
    }
}

// The f32x8 poly path assumes exactly 8 voices. Changing VOICES requires
// revisiting the SIMD width (e.g. f32x16 or 2× f32x8).
#[cfg(feature = "simd")]
const _: () = assert!(VOICES == 8);

/// A poly oscillator. `pitch` is a voice-interleaved tile of per-voice Hz;
/// writes a voice-interleaved audio tile. SoA phase; voice loop is the inner
/// (vectorizable) dimension. Shape-aware: routes through `wave_sample` /
/// `wave_sample_x8` (Sy-1's band-limited waveshapes), so Sine output is
/// bit-identical to the prior raw-`fast_sin` PolyOsc.
#[derive(Clone, Copy)]
pub struct PolyOsc {
    phase: [f32; VOICES], // [0,1) per voice
    shape: Wave,
}
impl PolyOsc {
    pub fn new() -> PolyOsc {
        PolyOsc { phase: [0.0; VOICES], shape: Wave::Sine }
    }
    /// `code`: 0=Sine, 1=Saw, 2=Square, 3=Tri (default Sine).
    pub fn set_shape(&mut self, code: u8) {
        self.shape = match code {
            1 => Wave::Saw,
            2 => Wave::Square,
            3 => Wave::Tri,
            _ => Wave::Sine,
        };
    }
    /// `pitch`, `width`, and `out` are voice-interleaved, length
    /// `VOICES * n_samples`. `width` is the per-voice Square PWM duty
    /// (`<= 0` → 0.5, else clamped to `[0.01, 0.99]`, matching scalar
    /// `wave_sample`); ignored by Sine/Saw/Tri. An all-zero `width` tile
    /// reproduces the pre-Sy-2d fixed-0.5-duty PolyOsc output bit-for-bit.
    pub fn process(&mut self, pitch: &[f32], width: &[f32], dt: f32, out: &mut [f32]) {
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            let n = out.len() / VOICES;
            let one = f32x8::splat(1.0);
            let dtv = f32x8::splat(dt);
            let mut ph = f32x8::from_array(self.phase);
            for i in 0..n {
                let f = f32x8::from_slice(&pitch[i * VOICES..]);
                let w = f32x8::from_slice(&width[i * VOICES..]);
                let dtp = f * dtv;
                let mut p = ph + dtp;
                // wrap: p -= trunc-floor(p), matching scalar `floorf`
                let t: f32x8 = p.cast::<i32>().cast::<f32>();
                let fl = t.simd_gt(p).select(t - one, t);
                p -= fl;
                ph = p;
                wave_sample_x8(self.shape, p, dtp, w).copy_to_slice(&mut out[i * VOICES..]);
            }
            self.phase = ph.to_array();
        }
        #[cfg(not(feature = "simd"))]
        {
            let n = out.len() / VOICES;
            for i in 0..n {
                for v in 0..VOICES {
                    let f = pitch[i * VOICES + v];
                    let dtp = f * dt;
                    let mut p = self.phase[v] + dtp;
                    p -= floorf(p); // wrap [0,1)
                    self.phase[v] = p;
                    out[i * VOICES + v] = wave_sample(self.shape, p, dtp, width[i * VOICES + v]);
                }
            }
        }
    }
}
impl Default for PolyOsc {
    fn default() -> Self {
        Self::new()
    }
}

/// A pure poly envelope source: 8 independent attack/release envelopes with
/// per-voice gate state. attack/release are shared (mono) controls; output is a
/// voice-interleaved tile of levels ∈ [0,1]. No audio input — a modulation
/// source. Scalar (the AR state machine is branchy and cheap).
#[derive(Clone, Copy)]
pub struct PolyAr {
    voices: [Ar; VOICES],
}
impl PolyAr {
    pub fn new() -> PolyAr {
        PolyAr { voices: [Ar::new(); VOICES] }
    }
    pub fn gate_voice(&mut self, v: usize, on: bool) {
        if v < VOICES { self.voices[v].gate(on); }
    }
    pub fn trigger_voice(&mut self, v: usize) {
        if v < VOICES { self.voices[v].trigger(); }
    }
    /// attack/release mono controls; writes a voice-interleaved env tile.
    pub fn process(&mut self, attack: In, release: In, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            let atk = attack.at(i);
            let rel = release.at(i);
            for v in 0..VOICES {
                out[i * VOICES + v] = self.voices[v].tick(atk, rel, dt);
            }
        }
    }
}
impl Default for PolyAr {
    fn default() -> Self { Self::new() }
}

/// Collapse a voice-interleaved tile to mono: `out[i] = Σ_v tile[i*VOICES + v]`.
/// `tile.len() == VOICES * out.len()`.
pub fn voice_sum(tile: &[f32], out: &mut [f32]) {
    for i in 0..out.len() {
        let mut s = 0.0;
        for v in 0..VOICES {
            s += tile[i * VOICES + v];
        }
        out[i] = s;
    }
}

#[cfg(feature = "simd")]
const _: () = assert!(VOICES == 8);

/// Poly SVF lowpass. Poly audio in → 8 filtered lanes; shared mono cutoff/res.
/// Scalar path holds `[Svf; VOICES]` and reuses the audited `Svf::tick`; the
/// SIMD path keeps SoA `f32x8` state register-resident. Both use identical
/// scalar coeffs, so they agree.
#[derive(Clone, Copy)]
pub struct PolySvf {
    #[cfg(not(feature = "simd"))]
    voices: [Svf; VOICES],
    #[cfg(feature = "simd")]
    ic1: [f32; VOICES],
    #[cfg(feature = "simd")]
    ic2: [f32; VOICES],
}
impl PolySvf {
    #[cfg(not(feature = "simd"))]
    pub fn new() -> PolySvf {
        PolySvf { voices: [Svf::new(); VOICES] }
    }
    #[cfg(feature = "simd")]
    pub fn new() -> PolySvf {
        PolySvf { ic1: [0.0; VOICES], ic2: [0.0; VOICES] }
    }

    /// `audio` = voice-interleaved poly input; cutoff/res mono; LP output tile.
    pub fn process(&mut self, audio: &[f32], cutoff: In, res: In, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            let two = f32x8::splat(2.0);
            let mut ic1 = f32x8::from_array(self.ic1);
            let mut ic2 = f32x8::from_array(self.ic2);
            for i in 0..n {
                let (_k, a1, a2, a3) = poly_svf_coeffs(cutoff.at(i), res.at(i), dt);
                let (a1v, a2v, a3v) = (f32x8::splat(a1), f32x8::splat(a2), f32x8::splat(a3));
                let v0 = f32x8::from_slice(&audio[i * VOICES..]);
                let v3 = v0 - ic2;
                let v1 = a1v * ic1 + a2v * v3;
                let v2 = ic2 + a2v * ic1 + a3v * v3;
                ic1 = two * v1 - ic1;
                ic2 = two * v2 - ic2;
                v2.copy_to_slice(&mut out[i * VOICES..]); // LP = v2
            }
            self.ic1 = ic1.to_array();
            self.ic2 = ic2.to_array();
        }
        #[cfg(not(feature = "simd"))]
        {
            for i in 0..n {
                let (k, a1, a2, a3) = poly_svf_coeffs(cutoff.at(i), res.at(i), dt);
                for v in 0..VOICES {
                    out[i * VOICES + v] =
                        self.voices[v].tick(audio[i * VOICES + v], k, a1, a2, a3, SvfResp::Lp);
                }
            }
        }
    }
}
impl Default for PolySvf {
    fn default() -> Self { Self::new() }
}

/// One Heun (RK2) diode-ladder sub-step across `VOICES` lanes. Mirrors the
/// scalar `DiodeLadder::heun_step` (filter.rs) exactly; `state` is SoA per-stage.
/// `fh`/`res` are shared (splatted); `input` is per-voice.
#[cfg(feature = "simd")]
#[inline]
fn heun_step_x8<const STAGES: usize>(
    state: &mut [core::simd::f32x8; STAGES],
    input: core::simd::f32x8,
    fh: core::simd::f32x8,
    res4: core::simd::f32x8, // res * 4.0, pre-multiplied
) {
    use core::simd::prelude::*;
    use crate::filter::pade_tanh_x8;
    let feedback = pade_tanh_x8(state[STAGES - 1]) * res4;
    let x = input - feedback;

    // Predictor (Euler)
    let mut temp = *state;
    let mut d = [f32x8::splat(0.0); STAGES];
    d[0] = fh * (x - pade_tanh_x8(temp[0]));
    for i in 1..STAGES {
        d[i] = fh * (pade_tanh_x8(temp[i - 1]) - pade_tanh_x8(temp[i]));
    }
    for i in 0..STAGES {
        temp[i] += d[i];
    }

    // Corrector (derivative at predicted state)
    let fb_p = pade_tanh_x8(temp[STAGES - 1]) * res4;
    let x_p = input - fb_p;
    let mut dp = [f32x8::splat(0.0); STAGES];
    dp[0] = fh * (x_p - pade_tanh_x8(temp[0]));
    for i in 1..STAGES {
        dp[i] = fh * (pade_tanh_x8(temp[i - 1]) - pade_tanh_x8(temp[i]));
    }
    for i in 0..STAGES {
        state[i] += (d[i] + dp[i]) * f32x8::splat(0.5);
    }
}

/// Poly Moog transistor-ladder. Poly audio in → 8 filtered lanes; shared mono
/// cutoff/res. Scalar path holds `[DiodeLadder<POLES>; VOICES]` and reuses the
/// audited `DiodeLadder::process`; the SIMD path runs the Heun ladder across
/// `f32x8` lanes via `heun_step_x8`. Both use `moog_coeffs`, so they agree.
#[derive(Clone, Copy)]
pub struct PolyMoog<const POLES: usize> {
    drive: f32,
    #[cfg(not(feature = "simd"))]
    ladders: [DiodeLadder<POLES>; VOICES],
    #[cfg(feature = "simd")]
    state: [core::simd::f32x8; POLES],
}

impl<const POLES: usize> PolyMoog<POLES> {
    #[cfg(not(feature = "simd"))]
    pub fn new() -> Self {
        PolyMoog { drive: 1.0, ladders: [DiodeLadder::new(); VOICES] }
    }
    #[cfg(feature = "simd")]
    pub fn new() -> Self {
        PolyMoog { drive: 1.0, state: [core::simd::f32x8::splat(0.0); POLES] }
    }

    pub fn set_drive(&mut self, d: f32) {
        self.drive = d.max(0.0);
    }

    /// `audio` = voice-interleaved poly input; cutoff/res shared mono; LP tile out.
    pub fn process(&mut self, audio: &[f32], cutoff: In, res: In, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            use crate::filter::pade_tanh_x8;
            let drive = f32x8::splat(self.drive);
            for i in 0..n {
                // res4 = ladder_res*4 = k, so moog_coeffs' k is exactly heun_step_x8's res4.
                let (fh, _ladder_res, k) = moog_coeffs(cutoff.at(i), res.at(i), dt, POLES);
                let (fhv, res4) = (f32x8::splat(fh), f32x8::splat(k));
                let gain = drive * f32x8::splat(1.0 + k);
                let x = f32x8::from_slice(&audio[i * VOICES..]) * gain;
                for _ in 0..MOOG_OVERSAMPLE {
                    heun_step_x8::<POLES>(&mut self.state, x, fhv, res4);
                }
                let y = pade_tanh_x8(self.state[POLES - 1]); // clipped last stage
                y.copy_to_slice(&mut out[i * VOICES..]);
            }
        }
        #[cfg(not(feature = "simd"))]
        {
            for i in 0..n {
                let (fh, ladder_res, k) = moog_coeffs(cutoff.at(i), res.at(i), dt, POLES);
                let gain = self.drive * (1.0 + k);
                for v in 0..VOICES {
                    let x = audio[i * VOICES + v] * gain;
                    out[i * VOICES + v] =
                        self.ladders[v].process(x, fh, ladder_res, MOOG_OVERSAMPLE);
                }
            }
        }
    }
}

impl<const POLES: usize> Default for PolyMoog<POLES> {
    fn default() -> Self { Self::new() }
}

/// Poly MS-20 (Korg35) Sallen-Key. Poly audio in → 8 filtered lanes; shared
/// mono cutoff/res. Scalar path holds `[Ms20; VOICES]` and reuses `Ms20::tick`;
/// the SIMD path runs the ZDF two-integrator + `ms20_clip_x8` feedback + DC
/// blocker + `pade_tanh_x8` limiter across `f32x8`. Both use `ms20_coeffs`.
#[derive(Clone, Copy)]
pub struct PolyMs20 {
    drive: f32,
    #[cfg(not(feature = "simd"))]
    voices: [Ms20; VOICES],
    #[cfg(feature = "simd")]
    cached_dt: f32,
    #[cfg(feature = "simd")]
    ic1: core::simd::f32x8,
    #[cfg(feature = "simd")]
    ic2: core::simd::f32x8,
    #[cfg(feature = "simd")]
    dc_x: core::simd::f32x8,
    #[cfg(feature = "simd")]
    dc_y: core::simd::f32x8,
    #[cfg(feature = "simd")]
    dc_a: f32, // DC-blocker coeff (shared; depends only on dt)
}

impl PolyMs20 {
    #[cfg(not(feature = "simd"))]
    pub fn new() -> Self {
        PolyMs20 { drive: 1.0, voices: [Ms20::new(); VOICES] }
    }
    #[cfg(feature = "simd")]
    pub fn new() -> Self {
        use core::simd::f32x8;
        PolyMs20 {
            drive: 1.0,
            cached_dt: 0.0,
            ic1: f32x8::splat(0.0),
            ic2: f32x8::splat(0.0),
            dc_x: f32x8::splat(0.0),
            dc_y: f32x8::splat(0.0),
            dc_a: 0.0,
        }
    }

    pub fn set_drive(&mut self, d: f32) {
        let d = d.max(0.0);
        self.drive = d;
        #[cfg(not(feature = "simd"))]
        for v in &mut self.voices {
            v.set_drive(d);
        }
    }

    /// `audio` = voice-interleaved poly input; cutoff/res/resp shared mono; out tile.
    pub fn process(&mut self, audio: &[f32], cutoff: In, res: In, resp: Ms20Resp, dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            use crate::filter::{ms20_clip_x8, pade_tanh_x8, OnePoleHp, MS20_DC_HP_HZ, MS20_OVERSAMPLE};
            if dt != self.cached_dt {
                // Reuse the scalar OnePoleHp coeff computation for exact agreement.
                let mut hp = OnePoleHp::default();
                hp.set_coeff(MS20_DC_HP_HZ * 2.0 * core::f64::consts::PI / (1.0 / dt as f64));
                self.dc_a = hp.coeff();
                self.cached_dt = dt;
            }
            let drive = f32x8::splat(self.drive);
            let a = f32x8::splat(self.dc_a);
            let b = (f32x8::splat(1.0) + a) * f32x8::splat(0.5);
            let two = f32x8::splat(2.0);
            let eight = f32x8::splat(8.0);
            for i in 0..n {
                let (k, a1, a2, a3) = ms20_coeffs(cutoff.at(i), res.at(i), dt);
                let (kv, a1v, a2v, a3v) =
                    (f32x8::splat(k), f32x8::splat(a1), f32x8::splat(a2), f32x8::splat(a3));
                let input = f32x8::from_slice(&audio[i * VOICES..]);
                let mut y = f32x8::splat(0.0);
                for _ in 0..MS20_OVERSAMPLE {
                    let v0 = input * drive - kv * ms20_clip_x8(drive * self.ic1);
                    let v3 = v0 - self.ic2;
                    let v1 = a1v * self.ic1 + a2v * v3;
                    let v2 = self.ic2 + a2v * self.ic1 + a3v * v3;
                    self.ic1 = two * v1 - self.ic1;
                    self.ic2 = two * v2 - self.ic2;
                    y = match resp {
                        Ms20Resp::Lp => v2,
                        Ms20Resp::Hp => v0 - kv * v1 - v2,
                    };
                }
                // DC blocker (OnePoleHp recurrence) then ±8 tanh limiter.
                let dc = b * y - b * self.dc_x + a * self.dc_y;
                self.dc_x = y;
                self.dc_y = dc;
                let s = eight * pade_tanh_x8(dc / eight);
                s.copy_to_slice(&mut out[i * VOICES..]);
            }
        }
        #[cfg(not(feature = "simd"))]
        {
            for i in 0..n {
                let (k, a1, a2, a3) = ms20_coeffs(cutoff.at(i), res.at(i), dt);
                for v in 0..VOICES {
                    out[i * VOICES + v] =
                        self.voices[v].tick_with_dt(audio[i * VOICES + v], k, a1, a2, a3, resp, dt);
                }
            }
        }
    }
}

impl Default for PolyMs20 {
    fn default() -> Self {
        Self::new()
    }
}

/// Poly × poly, lanewise: `out[j] = a[j] * b[j]`. The VCA. Auto-vectorizes.
pub fn poly_mul(a: &[f32], b: &[f32], out: &mut [f32]) {
    for j in 0..out.len() {
        out[j] = a[j] * b[j];
    }
}

/// Poly + poly, lanewise: `out[j] = a[j] + b[j]`. Two poly inputs. Stateless
/// (auto-vectorizes).
pub fn poly_add(a: &[f32], b: &[f32], out: &mut [f32]) {
    for j in 0..out.len() {
        out[j] = a[j] + b[j];
    }
}

/// Poly noise: 8 independent colored generators (white/pink/brown), each a mono
/// `Noise` with a decorrelated seed. Scalar (serial IIR recurrences). White lanes
/// remain bit-identical to the Sy-2b `PolyNoise`.
#[derive(Clone, Copy)]
pub struct PolyNoise {
    voices: [Noise; VOICES],
}
impl PolyNoise {
    pub fn new() -> PolyNoise {
        PolyNoise::new_color(NoiseColor::White)
    }
    pub fn new_color(color: NoiseColor) -> PolyNoise {
        let voices = core::array::from_fn(|v| {
            let seed = 0x2545_F491u32 ^ 0x9E37_79B9u32.wrapping_mul((v as u32) + 1);
            Noise::seeded_color(seed, color)
        });
        PolyNoise { voices }
    }
    /// Voice-interleaved output tile, length `VOICES * n_samples`.
    pub fn process(&mut self, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            for v in 0..VOICES {
                out[i * VOICES + v] = self.voices[v].tick();
            }
        }
    }
}
impl Default for PolyNoise {
    fn default() -> Self {
        Self::new()
    }
}

/// Poly semitone→Hz: `out[v] = ref_hz · 2^(semitone[v]/12)`. Poly-in note-offset
/// lanes → Hz lanes. Scalar (pitch is control-rate, not a recurrent kernel).
#[derive(Clone, Copy)]
pub struct PolyMtof {
    ref_hz: f32,
}
impl PolyMtof {
    pub fn new() -> PolyMtof {
        PolyMtof { ref_hz: 440.0 }
    }
    pub fn set_ref(&mut self, hz: f32) {
        self.ref_hz = hz;
    }
    /// `semitones` = voice-interleaved note-offset tile; writes an Hz tile
    /// (element-wise, so the interleave is preserved trivially).
    pub fn process(&mut self, semitones: &[f32], out: &mut [f32]) {
        for j in 0..out.len() {
            out[j] = semitones_to_hz(semitones[j], self.ref_hz);
        }
    }
}
impl Default for PolyMtof {
    fn default() -> Self { Self::new() }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::{fast_sin, floorf}; // explicit so tests build under `--features simd` too

    fn make_sine_tile(freq: f32, n: usize, dt: f32) -> std::vec::Vec<f32> {
        let mut t = std::vec![0.0f32; VOICES * n];
        let mut ph = 0.0f32;
        for i in 0..n {
            ph += freq * dt;
            ph -= floorf(ph);
            let s = fast_sin(ph);
            for v in 0..VOICES {
                t[i * VOICES + v] = s;
            }
        }
        t
    }

    #[test]
    fn polysvf_attenuates_above_cutoff() {
        // A high-frequency input (18 kHz) fed to a low cutoff (300 Hz) comes out
        // much smaller; a low tone (100 Hz) passes ~unchanged.
        let dt = 1.0 / 48_000.0;
        let n = 4800;
        let hi = make_sine_tile(18_000.0, n, dt);
        let lo = make_sine_tile(100.0, n, dt);
        let cutoff: std::vec::Vec<f32> = (0..n).map(|_| 300.0).collect();
        let res: std::vec::Vec<f32> = (0..n).map(|_| 0.0).collect();
        let mut out_hi = std::vec![0.0f32; VOICES * n];
        let mut out_lo = std::vec![0.0f32; VOICES * n];
        PolySvf::new().process(&hi, In::A(&cutoff), In::A(&res), dt, &mut out_hi);
        PolySvf::new().process(&lo, In::A(&cutoff), In::A(&res), dt, &mut out_lo);
        let peak = |t: &[f32]| t[VOICES * n / 2..].iter().fold(0.0f32, |m, &s| m.max(s.abs()));
        assert!(peak(&out_hi) < 0.3, "18 kHz attenuated: {}", peak(&out_hi));
        assert!(peak(&out_lo) > 0.7, "100 Hz passes: {}", peak(&out_lo));
    }

    #[test]
    fn polysvf_matches_scalar_svf_single_voice() {
        // Cross-check: PolySvf voice 0 ≈ a scalar Svf on the same signal.
        use crate::filter::{Svf, SvfResp};
        let dt = 1.0 / 48_000.0;
        let n = 2000;
        let sig = make_sine_tile(1000.0, n, dt); // all 8 voices identical here
        let cutoff: std::vec::Vec<f32> = (0..n).map(|i| 500.0 + i as f32).collect(); // audio-rate → both use prewarp
        let res: std::vec::Vec<f32> = (0..n).map(|_| 0.3).collect();
        let mut poly_out = std::vec![0.0f32; VOICES * n];
        PolySvf::new().process(&sig, In::A(&cutoff), In::A(&res), dt, &mut poly_out);
        let mono_in: std::vec::Vec<f32> = (0..n).map(|i| sig[i * VOICES]).collect();
        let mut mono_out = std::vec![0.0f32; n];
        Svf::new().process(In::A(&mono_in), In::A(&cutoff), In::A(&res), SvfResp::Lp, dt, &mut mono_out);
        for i in 0..n {
            assert!((poly_out[i * VOICES] - mono_out[i]).abs() < 1e-4, "sample {i}");
        }
    }

    #[test]
    fn poly_mul_lanewise() {
        let a: [f32; VOICES * 2] = core::array::from_fn(|j| j as f32);
        let b: [f32; VOICES * 2] = core::array::from_fn(|_| 2.0);
        let mut out = [0.0f32; VOICES * 2];
        poly_mul(&a, &b, &mut out);
        for j in 0..VOICES * 2 {
            assert_eq!(out[j], j as f32 * 2.0);
        }
    }

    #[test]
    fn poly_add_lanewise() {
        let a: [f32; VOICES * 2] = core::array::from_fn(|j| j as f32);
        let b: [f32; VOICES * 2] = core::array::from_fn(|_| 10.0);
        let mut out = [0.0f32; VOICES * 2];
        poly_add(&a, &b, &mut out);
        for j in 0..VOICES * 2 {
            assert_eq!(out[j], j as f32 + 10.0);
        }
    }

    #[test]
    fn polyosc_saw_matches_mono_osc() {
        // A PolyOsc saw voice equals the mono Osc saw on the same frequency
        // (both go through wave_sample) — proves band-limiting is reused.
        use crate::osc::{Osc, Wave};
        let dt = 1.0 / 48_000.0;
        let n = 400;
        let freq = 220.0;
        let mut po = PolyOsc::new();
        po.set_shape(1); // Saw
        let mut out = std::vec![0.0f32; VOICES * n];
        po.process(&std::vec![freq; VOICES * n], &std::vec![0.0f32; VOICES * n], dt, &mut out);
        // Mono reference: Osc::process(wave, freq, pmod, width, dt, out).
        // PolyOsc advances phase THEN outputs (Sy-1 convention); mono Osc outputs
        // THEN advances — PolyOsc is one sample ahead: poly[i] == mono[i+1].
        let mut mono = Osc::new();
        let mut mref = std::vec![0.0f32; n + 1];
        mono.process(Wave::Saw, In::K(freq), In::K(0.0), In::K(0.0), dt, &mut mref);
        for i in 0..n {
            assert!((out[i * VOICES] - mref[i + 1]).abs() < 1e-4, "sample {i}: {} vs {}", out[i * VOICES], mref[i + 1]);
        }
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
    }

    #[test]
    fn polynoise_colors_match_mono_per_lane() {
        use crate::noise::{Noise, NoiseColor}; // match poly.rs test import convention (crate::, per existing `use crate::{fast_sin, floorf};`)
        let n = 256usize;
        for color in [NoiseColor::White, NoiseColor::Pink, NoiseColor::Brown] {
            let mut poly = PolyNoise::new_color(color);
            let mut out = std::vec![0.0f32; n * VOICES];
            poly.process(&mut out);
            // Each lane must equal a mono Noise seeded with that lane's seed + color.
            for v in 0..VOICES {
                let seed = 0x2545_F491u32 ^ 0x9E37_79B9u32.wrapping_mul((v as u32) + 1);
                let mut refn = Noise::seeded_color(seed, color);
                for i in 0..n {
                    let want = refn.tick();
                    assert_eq!(out[i * VOICES + v], want, "color {color:?} lane {v} sample {i}");
                }
            }
        }
    }

    #[test]
    fn polynoise_lanes_independent_and_bounded() {
        let mut nz = PolyNoise::new();
        let n = 64;
        let mut out = std::vec![0.0f32; VOICES * n];
        nz.process(&mut out);
        assert!(out.iter().all(|&s| s.is_finite() && s.abs() <= 1.0), "bounded");
        assert!(out.iter().any(|&s| s != 0.0), "non-silent");
        // Decorrelated: lane 0 and lane 1 differ.
        assert!((0..n).any(|i| out[i * VOICES] != out[i * VOICES + 1]), "lanes decorrelated");
    }

    #[test]
    fn polyctrl_fills_interleaved_lanes() {
        let mut c = PolyCtrl::new();
        for v in 0..VOICES {
            c.set_voice(v, v as f32 + 1.0); // 1..=8
        }
        let n = 5;
        let mut out = std::vec![0.0f32; VOICES * n];
        c.process(&mut out);
        for i in 0..n {
            for v in 0..VOICES {
                assert_eq!(out[i * VOICES + v], v as f32 + 1.0, "lane {v} sample {i}");
            }
        }
    }

    #[test]
    fn polyctrl_ignores_out_of_range_voice() {
        let mut c = PolyCtrl::new();
        c.set_voice(VOICES, 99.0); // no-op, no panic
        let mut out = std::vec![0.0f32; VOICES];
        c.process(&mut out);
        assert!(out.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn polyosc_renders_independent_per_voice_partials() {
        // Voice v runs at (v+1)*100 Hz. Each lane must equal a reference sine at
        // its own frequency, and lanes must not cross-contaminate.
        let dt = 1.0 / 48_000.0;
        let n = 480;
        let mut pitch = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                pitch[i * VOICES + v] = (v as f32 + 1.0) * 100.0;
            }
        }
        let mut osc = PolyOsc::new();
        let mut out = std::vec![0.0f32; VOICES * n];
        let width = std::vec![0.0f32; VOICES * n];
        osc.process(&pitch, &width, dt, &mut out);
        // Reference: independent phase accumulators per voice.
        let mut ph = [0.0f32; VOICES];
        for i in 0..n {
            for v in 0..VOICES {
                let f = (v as f32 + 1.0) * 100.0;
                ph[v] += f * dt;
                ph[v] -= floorf(ph[v]);
                let want = fast_sin(ph[v]);
                assert!((out[i * VOICES + v] - want).abs() < 1e-5, "voice {v} sample {i}");
            }
        }
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.0001));
    }

    #[test]
    fn polyosc_pwm_backcompat_and_per_voice() {
        let dt = 1.0 / 48_000.0;
        let n = 128;
        // (a) width all-zero ⇒ bit-identical to the pre-Sy-2d hardcoded-0.5
        // PolyOsc, for all four shapes. Per the `<=0 ⇒ 0.5` convention (scalar
        // `wave_sample`'s `if w <= 0.0 { w = 0.5 }`, mirrored structurally by
        // `wave_sample_x8`'s `select`), a width=0 tile and a width=0.5 tile
        // must flow through identical downstream arithmetic — so comparing
        // against a *second* PolyOsc instance fed an all-0.5 width tile is
        // exactly the "old hardcoded-0.5 path" the brief calls for, without
        // re-deriving the phase recurrence (and without risking an
        // unrelated scalar-vs-SIMD tolerance mismatch — both runs go
        // through the identical code path for this build).
        let pitch: std::vec::Vec<f32> =
            (0..n * VOICES).map(|j| 110.0 + (j % VOICES) as f32 * 37.0).collect();
        for code in 0u8..4 {
            let mut o_zero = PolyOsc::new();
            o_zero.set_shape(code);
            let width_zero = std::vec![0.0f32; n * VOICES];
            let mut out_zero = std::vec![0.0f32; n * VOICES];
            o_zero.process(&pitch, &width_zero, dt, &mut out_zero);

            let mut o_half = PolyOsc::new();
            o_half.set_shape(code);
            let width_half = std::vec![0.5f32; n * VOICES];
            let mut out_half = std::vec![0.0f32; n * VOICES];
            o_half.process(&pitch, &width_half, dt, &mut out_half);

            assert_eq!(
                out_zero, out_half,
                "shape code {code}: width=0 tile must reproduce the hardcoded-0.5 PolyOsc bit-for-bit"
            );

            // Independent ground truth: voice 0's phase recurrence run through
            // the (unchanged) scalar `wave_sample(..., 0.5)` directly — the
            // exact expression the pre-Sy-2d PolyOsc hardcoded.
            let shape = match code {
                1 => Wave::Saw,
                2 => Wave::Square,
                3 => Wave::Tri,
                _ => Wave::Sine,
            };
            let mut p0 = 0.0f32;
            for i in 0..n {
                let dtp = pitch[i * VOICES] * dt;
                p0 += dtp;
                p0 -= floorf(p0);
                let want = crate::osc::wave_sample(shape, p0, dtp, 0.5);
                assert!(
                    (out_zero[i * VOICES] - want).abs() < 1e-4,
                    "shape code {code} sample {i} voice 0: {} vs scalar ground truth {want}",
                    out_zero[i * VOICES]
                );
            }
        }

        // (b) per-voice width: lanes with distinct widths differ (Square).
        let mut o = PolyOsc::new();
        o.set_shape(2); // Square
        let pitch = std::vec![220.0f32; n * VOICES];
        let width: std::vec::Vec<f32> =
            (0..n * VOICES).map(|j| 0.1 + 0.1 * ((j % VOICES) as f32)).collect();
        let mut out = std::vec![0.0f32; n * VOICES];
        o.process(&pitch, &width, dt, &mut out);
        assert!(
            (0..n).any(|i| out[i * VOICES + 1] != out[i * VOICES + 7]),
            "distinct per-voice widths must produce distinct output"
        );
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
    }

    #[cfg(feature = "simd")]
    #[test]
    fn fast_sin_x8_matches_scalar() {
        use core::simd::f32x8;
        // Sweep phases across two full cycles; every lane must match scalar fast_sin.
        for base in 0..250 {
            let ps: [f32; 8] = core::array::from_fn(|k| (base as f32 * 8.0 + k as f32) / 1000.0);
            let v = crate::fast_sin_x8(f32x8::from_array(ps)).to_array();
            for k in 0..8 {
                assert!((v[k] - crate::fast_sin(ps[k])).abs() < 1e-6, "phase {} lane {k}", ps[k]);
            }
        }
    }

    #[test]
    fn voice_sum_adds_lanes_per_sample() {
        let n = 4;
        let mut tile = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                tile[i * VOICES + v] = (i * VOICES + v) as f32; // distinct
            }
        }
        let mut out = std::vec![0.0f32; n];
        voice_sum(&tile, &mut out);
        for i in 0..n {
            let want: f32 = (0..VOICES).map(|v| (i * VOICES + v) as f32).sum();
            assert_eq!(out[i], want, "sample {i}");
        }
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn polyosc_bounded(f in 0.0f32..8000.0) {
            let dt = 1.0 / 48_000.0;
            let n = 64;
            let pitch = std::vec![f; VOICES * n];
            let width = std::vec![0.0f32; VOICES * n];
            let mut osc = PolyOsc::new();
            let mut out = std::vec![0.0f32; VOICES * n];
            osc.process(&pitch, &width, dt, &mut out);
            for &s in &out {
                prop_assert!(s.is_finite() && s.abs() <= 1.0001, "polyosc unbounded: {s}");
            }
        }
    }

    #[test]
    fn polyar_gates_per_voice_independently() {
        let dt = 1.0 / 48_000.0;
        let mut env = PolyAr::new();
        env.gate_voice(0, true); // voice 0 attacks
        env.gate_voice(3, true); // voice 3 attacks
        let n = 4800; // 100 ms
        // attack/release are mono broadcast blocks (same for all voices per sample).
        let atk_mono: std::vec::Vec<f32> = (0..n).map(|_| 0.01).collect(); // 10 ms attack
        let rel_mono: std::vec::Vec<f32> = (0..n).map(|_| 0.05).collect();
        let mut out = std::vec![0.0f32; VOICES * n];
        env.process(In::A(&atk_mono), In::A(&rel_mono), dt, &mut out);
        // Gated voices reach ~1.0 by the end of a 100 ms window (10 ms attack).
        assert!((out[(n - 1) * VOICES + 0] - 1.0).abs() < 1e-3, "voice 0 reached sustain");
        assert!((out[(n - 1) * VOICES + 3] - 1.0).abs() < 1e-3, "voice 3 reached sustain");
        // Ungated voices stay silent.
        for v in [1usize, 2, 4, 5, 6, 7] {
            assert!(out[(n - 1) * VOICES + v].abs() < 1e-9, "voice {v} silent");
        }
        // Monotonic rise for voice 0 over the attack.
        assert!(out[10 * VOICES + 0] > out[0 * VOICES + 0]);
    }

    #[test]
    fn polyar_release_returns_to_zero() {
        let dt = 1.0 / 48_000.0;
        let mut env = PolyAr::new();
        env.gate_voice(0, true);
        let n = 480;
        let atk: std::vec::Vec<f32> = (0..n).map(|_| 0.001).collect(); // 1 ms
        let rel: std::vec::Vec<f32> = (0..n).map(|_| 0.001).collect();
        let mut out = std::vec![0.0f32; VOICES * n];
        env.process(In::A(&atk), In::A(&rel), dt, &mut out);
        assert!((out[(n - 1) * VOICES] - 1.0).abs() < 1e-3, "reached sustain");
        env.gate_voice(0, false); // release
        let mut out2 = std::vec![0.0f32; VOICES * n];
        env.process(In::A(&atk), In::A(&rel), dt, &mut out2);
        assert!(out2[(n - 1) * VOICES].abs() < 1e-3, "released to 0");
    }

    #[test]
    fn polyar_trigger_voice_is_one_shot() {
        // trigger_voice → attack then release with no sustain, on the addressed
        // lane only. Voice 2 rises to ~1 then decays back to 0; others stay 0.
        let dt = 1.0 / 48_000.0;
        let mut env = PolyAr::new();
        env.trigger_voice(2);
        let n = 2000;
        let atk: std::vec::Vec<f32> = (0..n).map(|_| 0.002).collect(); // 2 ms
        let rel: std::vec::Vec<f32> = (0..n).map(|_| 0.005).collect(); // 5 ms
        let mut out = std::vec![0.0f32; VOICES * n];
        env.process(In::A(&atk), In::A(&rel), dt, &mut out);
        let peak = (0..n).map(|i| out[i * VOICES + 2]).fold(0.0f32, f32::max);
        assert!(peak > 0.99, "one-shot reaches peak: {peak}");
        assert!(out[(n - 1) * VOICES + 2] < 0.05, "one-shot decays after: {}", out[(n - 1) * VOICES + 2]);
        assert!(out[(n - 1) * VOICES].abs() < 1e-9, "untriggered voice 0 silent");
    }

    #[test]
    fn polymtof_maps_semitones_to_hz() {
        let mut m = PolyMtof::new(); // ref 440
        let offsets = [0.0f32, 12.0, -12.0, 7.0, -9.0, 24.0, 1.0, -1.0];
        let n = 3;
        let mut inp = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                inp[i * VOICES + v] = offsets[v];
            }
        }
        let mut out = std::vec![0.0f32; VOICES * n];
        m.process(&inp, &mut out);
        assert!((out[0] - 440.0).abs() < 1e-2, "0 → 440: {}", out[0]);
        assert!((out[1] - 880.0).abs() < 1e-2, "+12 → 880: {}", out[1]);
        assert!((out[2] - 220.0).abs() < 1e-2, "-12 → 220: {}", out[2]);
        assert!(out.iter().all(|&h| h.is_finite() && h > 0.0));
    }

    #[test]
    fn polymtof_set_ref_retunes() {
        let mut m = PolyMtof::new();
        m.set_ref(100.0);
        let mut out = std::vec![0.0f32; VOICES];
        m.process(&std::vec![0.0f32; VOICES], &mut out);
        assert!(out.iter().all(|&h| (h - 100.0).abs() < 1e-3));
    }

    #[test]
    fn polymoog_matches_scalar_oracle_both_slopes() {
        use crate::filter::Moog;
        let dt = 1.0 / 48_000.0;
        let n = 200usize;
        // Shared controls; per-voice distinct audio so lanes are genuinely independent.
        let cutoff = 1200.0f32;
        let res = 0.8f32;
        check_slope::<4>(dt, n, cutoff, res);
        check_slope::<2>(dt, n, cutoff, res);

        fn check_slope<const P: usize>(dt: f32, n: usize, cutoff: f32, res: f32) {
            let mut poly = PolyMoog::<P>::new();
            let mut refs: [Moog<P>; VOICES] = core::array::from_fn(|_| Moog::<P>::new());
            // Voice v gets a saw-ish ramp scaled per voice.
            let audio: std::vec::Vec<f32> = (0..n * VOICES)
                .map(|j| { let i = j / VOICES; let v = j % VOICES; ((i as f32 * 0.017 + v as f32 * 0.03) % 1.0) * 2.0 - 1.0 })
                .collect();
            let mut out = std::vec![0.0f32; n * VOICES];
            poly.process(&audio, In::K(cutoff), In::K(res), dt, &mut out);
            for v in 0..VOICES {
                let vin: std::vec::Vec<f32> = (0..n).map(|i| audio[i * VOICES + v]).collect();
                let mut vout = std::vec![0.0f32; n];
                refs[v].process(In::A(&vin), In::K(cutoff), In::K(res), dt, &mut vout);
                for i in 0..n {
                    assert!((out[i * VOICES + v] - vout[i]).abs() <= 1e-4,
                        "slope {P} voice {v} sample {i}: {} vs {}", out[i * VOICES + v], vout[i]);
                }
            }
        }
    }

    #[test]
    fn polymoog_stays_bounded_at_high_res() {
        let dt = 1.0 / 48_000.0;
        let n = 4096;
        let mut poly = PolyMoog::<4>::new();
        let audio = std::vec![0.5f32; n * VOICES]; // constant excitation
        let mut out = std::vec![0.0f32; n * VOICES];
        poly.process(&audio, In::K(1000.0), In::K(1.0), dt, &mut out);
        for &s in &out { assert!(s.abs() <= 1.0001, "moog diverged: {s}"); }
    }

    #[test]
    fn polyms20_matches_scalar_oracle_both_responses() {
        use crate::filter::{Ms20, Ms20Resp};
        let dt = 1.0 / 48_000.0;
        let n = 200usize;
        for resp in [Ms20Resp::Lp, Ms20Resp::Hp] {
            let mut poly = PolyMs20::new();
            let mut refs: [Ms20; VOICES] = core::array::from_fn(|_| Ms20::new());
            // Voice v gets a saw-ish ramp scaled per voice.
            let audio: std::vec::Vec<f32> = (0..n * VOICES)
                .map(|j| { let i = j / VOICES; let v = j % VOICES; ((i as f32 * 0.021 + v as f32 * 0.04) % 1.0) * 2.0 - 1.0 })
                .collect();
            let mut out = std::vec![0.0f32; n * VOICES];
            poly.process(&audio, In::K(1500.0), In::K(0.8), resp, dt, &mut out);
            for v in 0..VOICES {
                let vin: std::vec::Vec<f32> = (0..n).map(|i| audio[i * VOICES + v]).collect();
                let mut vout = std::vec![0.0f32; n];
                refs[v].process(In::A(&vin), In::K(1500.0), In::K(0.8), resp, dt, &mut vout);
                for i in 0..n {
                    assert!((out[i * VOICES + v] - vout[i]).abs() <= 1e-4,
                        "{resp:?} voice {v} sample {i}: {} vs {}", out[i * VOICES + v], vout[i]);
                }
            }
        }
    }

    #[test]
    fn polyms20_stays_bounded_at_high_res_and_drive() {
        use crate::filter::Ms20Resp;
        let dt = 1.0 / 48_000.0;
        let n = 4096;
        let mut poly = PolyMs20::new();
        poly.set_drive(6.0);
        let audio = std::vec![0.5f32; n * VOICES];
        let mut out = std::vec![0.0f32; n * VOICES];
        poly.process(&audio, In::K(8000.0), In::K(0.99), Ms20Resp::Lp, dt, &mut out);
        for &s in &out { assert!(s.abs() <= 8.0001, "ms20 diverged: {s}"); }
    }
}
