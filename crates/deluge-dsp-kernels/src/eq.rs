//! RBJ parametric EQ (peaking / low-shelf / high-shelf) on a Direct-Form-II-
//! Transposed biquad (CMSIS-DSP `df2T` style). Per-sample, mono, no heap/buffer.
//! Coefficients (RBJ "Audio EQ Cookbook") recomputed per block. Not ported from
//! any GPL source.

use crate::In;
use crate::fast_sin;

/// Direct-Form-II-Transposed biquad (CMSIS-DSP `df2T` style). `a0`-normalized.
#[derive(Clone, Copy)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}
impl Biquad {
    pub fn new() -> Biquad {
        Biquad {
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            z1: 0.0,
            z2: 0.0,
        }
    }
    pub fn set_coeffs(&mut self, b0: f32, b1: f32, b2: f32, a1: f32, a2: f32) {
        self.b0 = b0;
        self.b1 = b1;
        self.b2 = b2;
        self.a1 = a1;
        self.a2 = a2;
    }
    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }
}
impl Default for Biquad {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy)]
pub enum EqType {
    Peak,
    LowShelf,
    HighShelf,
}

/// RBJ cookbook coefficients (`a0`-normalized) for one band.
fn rbj_coeffs(ty: EqType, freq: f32, gain_db: f32, q: f32, dt: f32) -> (f32, f32, f32, f32, f32) {
    let p = (freq * dt).clamp(1e-4, 0.49); // normalized freq, away from Nyquist
    let sinw = fast_sin(p);
    let cosw = fast_sin(p + 0.25); // cos(2π·p)
    let a = libm::powf(10.0, gain_db / 40.0);
    let alpha = sinw / (2.0 * q.max(0.1));
    let (b0, b1, b2, a0, a1, a2) = match ty {
        EqType::Peak => (
            1.0 + alpha * a,
            -2.0 * cosw,
            1.0 - alpha * a,
            1.0 + alpha / a,
            -2.0 * cosw,
            1.0 - alpha / a,
        ),
        EqType::LowShelf => {
            let sa = 2.0 * libm::sqrtf(a) * alpha;
            (
                a * ((a + 1.0) - (a - 1.0) * cosw + sa),
                2.0 * a * ((a - 1.0) - (a + 1.0) * cosw),
                a * ((a + 1.0) - (a - 1.0) * cosw - sa),
                (a + 1.0) + (a - 1.0) * cosw + sa,
                -2.0 * ((a - 1.0) + (a + 1.0) * cosw),
                (a + 1.0) + (a - 1.0) * cosw - sa,
            )
        }
        EqType::HighShelf => {
            let sa = 2.0 * libm::sqrtf(a) * alpha;
            (
                a * ((a + 1.0) + (a - 1.0) * cosw + sa),
                -2.0 * a * ((a - 1.0) + (a + 1.0) * cosw),
                a * ((a + 1.0) + (a - 1.0) * cosw - sa),
                (a + 1.0) - (a - 1.0) * cosw + sa,
                2.0 * ((a - 1.0) - (a + 1.0) * cosw),
                (a + 1.0) - (a - 1.0) * cosw - sa,
            )
        }
    };
    (b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)
}

/// RBJ parametric EQ band. Per-sample, mono; coefficients recomputed per block.
#[derive(Clone, Copy)]
pub struct Eq {
    ty: EqType,
    freq: f32,
    gain: f32,
    q: f32,
    biquad: Biquad,
}
impl Eq {
    pub fn new(ty: EqType) -> Eq {
        Eq {
            ty,
            freq: 1000.0,
            gain: 0.0,
            q: 0.707,
            biquad: Biquad::new(),
        }
    }
    pub fn set_freq(&mut self, v: f32) {
        self.freq = v.clamp(10.0, 20_000.0);
    }
    pub fn set_gain(&mut self, v: f32) {
        self.gain = v.clamp(-24.0, 24.0);
    }
    pub fn set_q(&mut self, v: f32) {
        self.q = v.clamp(0.1, 20.0);
    }
    pub fn set_type(&mut self, code: u8) {
        self.ty = match code {
            1 => EqType::LowShelf,
            2 => EqType::HighShelf,
            _ => EqType::Peak,
        };
    }

    pub fn process(&mut self, input: In, dt: f32, out: &mut [f32]) {
        let (b0, b1, b2, a1, a2) = rbj_coeffs(self.ty, self.freq, self.gain, self.q, dt);
        self.biquad.set_coeffs(b0, b1, b2, a1, a2);
        for i in 0..out.len() {
            out[i] = self.biquad.tick(input.at(i));
        }
    }

    /// Like `process`, but filters `buf` in place (reads index `i` before writing
    /// it — the biquad state is internal, so same-index in-place is safe). Used by
    /// the stereo `MasterEq` on the root bus.
    pub fn process_in_place(&mut self, buf: &mut [f32], dt: f32) {
        let (b0, b1, b2, a1, a2) = rbj_coeffs(self.ty, self.freq, self.gain, self.q, dt);
        self.biquad.set_coeffs(b0, b1, b2, a1, a2);
        for i in 0..buf.len() {
            buf[i] = self.biquad.tick(buf[i]);
        }
    }
}
impl Default for Eq {
    fn default() -> Self {
        Self::new(EqType::Peak)
    }
}

/// Stereo master EQ: two independent one-band `Eq`s (L/R) sharing the same
/// params. Applied to the render root bus between the DC-block and the limiter.
pub struct MasterEq {
    ch: [Eq; 2], // [L, R]
}

impl MasterEq {
    pub fn new(freq: f32, gain_db: f32, q: f32, eq_type: u8) -> MasterEq {
        let mut m = MasterEq {
            ch: [Eq::new(EqType::Peak); 2],
        };
        m.set_params(freq, gain_db, q, eq_type);
        m
    }
    /// Set both channels' band params. Finite-guarded (NaN/inf → a safe default)
    /// before `Eq`'s own range-clamps, so a bad param never poisons the output.
    pub fn set_params(&mut self, freq: f32, gain_db: f32, q: f32, eq_type: u8) {
        let freq = if freq.is_finite() { freq } else { 1000.0 };
        let gain_db = if gain_db.is_finite() { gain_db } else { 0.0 };
        let q = if q.is_finite() { q } else { 0.707 };
        for e in self.ch.iter_mut() {
            e.set_type(eq_type);
            e.set_freq(freq);
            e.set_gain(gain_db);
            e.set_q(q);
        }
    }
    /// EQ `l`/`r` in place. `dt` = 1/sample_rate (guarded — a non-finite dt would
    /// otherwise poison the coeffs on the master output).
    pub fn process(&mut self, l: &mut [f32], r: &mut [f32], dt: f32) {
        let dt = if dt.is_finite() && dt > 0.0 {
            dt
        } else {
            1.0 / 48_000.0
        };
        self.ch[0].process_in_place(l, dt);
        self.ch[1].process_in_place(r, dt);
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;
    use deluge_dsp_test::filter_meas::magnitude_db;

    const FS: f32 = 48_000.0;

    fn tail_peak(b: &[f32]) -> f32 {
        b[b.len() / 2..].iter().fold(0.0f32, |m, &x| m.max(x.abs()))
    }

    #[test]
    fn mastereq_peak_boost_amplifies() {
        const DT: f32 = 1.0 / 48_000.0;
        let f = 1000.0f32;
        let mut l = [0.0f32; 8192];
        let mut r = [0.0f32; 8192];
        for i in 0..8192 {
            let s = 0.2 * libm::sinf(core::f32::consts::TAU * f * (i as f32) * DT);
            l[i] = s;
            r[i] = s;
        }
        let in_peak = tail_peak(&l);
        let mut eq = MasterEq::new(f, 12.0, 2.0, 0); // +12 dB peak at 1 kHz
        eq.process(&mut l, &mut r, DT);
        let out_peak = tail_peak(&l);
        assert!(
            out_peak > in_peak * 1.5,
            "peak boost should amplify: in={} out={}",
            in_peak,
            out_peak
        );
        assert!(
            (tail_peak(&l) - tail_peak(&r)).abs() < 1e-4,
            "L/R same response"
        );
    }

    #[test]
    fn mastereq_cut_attenuates() {
        const DT: f32 = 1.0 / 48_000.0;
        let f = 1000.0f32;
        let mut l = [0.0f32; 8192];
        let mut r = [0.0f32; 8192];
        for i in 0..8192 {
            let s = 0.2 * libm::sinf(core::f32::consts::TAU * f * (i as f32) * DT);
            l[i] = s;
            r[i] = s;
        }
        let in_peak = tail_peak(&l);
        let mut eq = MasterEq::new(f, -12.0, 2.0, 0); // -12 dB cut at 1 kHz
        eq.process(&mut l, &mut r, DT);
        assert!(tail_peak(&l) < in_peak * 0.7, "cut should attenuate");
    }

    #[test]
    fn mastereq_flat_passes_approximately() {
        const DT: f32 = 1.0 / 48_000.0;
        let f = 1000.0f32;
        let mut l = [0.0f32; 8192];
        let mut r = [0.0f32; 8192];
        for i in 0..8192 {
            let s = 0.2 * libm::sinf(core::f32::consts::TAU * f * (i as f32) * DT);
            l[i] = s;
            r[i] = s;
        }
        let in_peak = tail_peak(&l);
        let mut eq = MasterEq::new(f, 0.0, 2.0, 0); // flat (0 dB peak)
        eq.process(&mut l, &mut r, DT);
        assert!(
            (tail_peak(&l) - in_peak).abs() < in_peak * 0.1,
            "flat EQ ~unchanged"
        );
    }

    #[test]
    fn mastereq_no_panic_on_adversarial_params() {
        let mut eq = MasterEq::new(f32::NAN, f32::INFINITY, -1.0, 99);
        let mut l = [0.5f32; 16];
        let mut r = [0.5f32; 16];
        eq.process(&mut l, &mut r, 1.0 / 48_000.0);
        eq.set_params(f32::NAN, f32::NAN, f32::NAN, 1);
        eq.process(&mut l, &mut r, f32::NAN); // dt guarded → finite
        assert!(l[15].is_finite(), "must stay finite: {}", l[15]);
    }

    // Filter magnitude (dB) at `probe` Hz for an EQ band centered at `center`.
    fn mag(ty: EqType, center: f32, gain: f32, q: f32, probe: f32) -> f32 {
        magnitude_db(FS, probe, |buf| {
            let mut e = Eq::new(ty);
            e.set_freq(center);
            e.set_gain(gain);
            e.set_q(q);
            let sine: std::vec::Vec<f32> = (0..buf.len())
                .map(|i| (core::f32::consts::TAU * probe * i as f32 / FS).sin())
                .collect();
            e.process(In::A(&sine), 1.0 / FS, buf);
        })
    }

    #[test]
    fn peak_boosts_at_center() {
        assert!(
            (mag(EqType::Peak, 1000.0, 12.0, 1.0, 1000.0) - 12.0).abs() < 1.5,
            "peak +12dB at center"
        );
        assert!(
            mag(EqType::Peak, 1000.0, 12.0, 1.0, 150.0).abs() < 2.0,
            "≈0dB far below center"
        );
    }

    #[test]
    fn peak_cuts_at_center() {
        assert!(
            (mag(EqType::Peak, 1000.0, -12.0, 1.0, 1000.0) + 12.0).abs() < 1.5,
            "peak −12dB at center"
        );
    }

    #[test]
    fn low_shelf_lifts_lows() {
        let low = mag(EqType::LowShelf, 300.0, 9.0, 0.707, 60.0);
        let high = mag(EqType::LowShelf, 300.0, 9.0, 0.707, 8000.0);
        assert!((low - 9.0).abs() < 2.0, "low band ≈ +9dB: {low}");
        assert!(high.abs() < 2.0, "high band ≈ 0dB: {high}");
    }

    #[test]
    fn high_shelf_lifts_highs() {
        let high = mag(EqType::HighShelf, 3000.0, 9.0, 0.707, 12000.0);
        let low = mag(EqType::HighShelf, 3000.0, 9.0, 0.707, 200.0);
        assert!((high - 9.0).abs() < 2.5, "high band ≈ +9dB: {high}");
        assert!(low.abs() < 2.0, "low band ≈ 0dB: {low}");
    }

    #[test]
    fn higher_q_is_narrower() {
        // Off-center (half-octave below) the low-Q peak still has boost; the
        // high-Q peak has rolled off more there → less boost.
        let off_lowq = mag(EqType::Peak, 1000.0, 12.0, 0.7, 700.0);
        let off_highq = mag(EqType::Peak, 1000.0, 12.0, 5.0, 700.0);
        assert!(
            off_lowq > off_highq,
            "higher Q → narrower (less boost off-center): {off_lowq} vs {off_highq}"
        );
    }

    #[test]
    fn zero_gain_is_unity() {
        for ty in [EqType::Peak, EqType::LowShelf, EqType::HighShelf] {
            for probe in [100.0, 1000.0, 8000.0] {
                let db = mag(ty, 1000.0, 0.0, 1.0, probe);
                assert!(db.abs() < 0.2, "0dB gain should be unity: {db} at {probe}");
            }
        }
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn eq_is_finite_and_bounded(
            freq in 20.0f32..18_000.0,
            gain in -24.0f32..24.0,
            q in 0.2f32..8.0,
            type_code in 0u8..3,
            amp in 0.0f32..1.0,
        ) {
            let tys = [EqType::Peak, EqType::LowShelf, EqType::HighShelf];
            let mut e = Eq::new(tys[type_code as usize]);
            e.set_freq(freq); e.set_gain(gain); e.set_q(q);
            let input = std::vec![amp; 1024];
            let mut out = std::vec![0.0f32; 1024];
            e.process(In::A(&input), 1.0 / FS, &mut out);
            for &v in &out {
                prop_assert!(v.is_finite());
                prop_assert!(v.abs() <= 32.0, "unbounded: {v}");
            }
        }
    }
}
