//! Multi-shape waveshaper/distortion (soft/hard/fold/tube) with 4× oversampling.
//! A small per-sample kernel — no heap, no buffer. Reuses `pade_tanh` and the
//! `Svf` LP (as a constant-coefficient 4-pole decimation filter).

use crate::filter::{pade_tanh, svf_coeffs, Svf, SvfResp};
use crate::In;

#[derive(Clone, Copy)]
pub enum Shape {
    Soft,
    Hard,
    Fold,
    Tube,
}

const MAX_DRIVE: f32 = 24.0;
const TUBE_BIAS: f32 = 0.5;
const DEC_G: f32 = 0.414_213_56; // tan(π/8) = √2 − 1
const DEC_K: f32 = 1.414_213_56; // √2 (Butterworth)

#[inline]
fn apply_shape(shape: Shape, z: f32) -> f32 {
    match shape {
        Shape::Soft => pade_tanh(z),
        Shape::Hard => z.clamp(-1.0, 1.0),
        Shape::Fold => {
            // Triangle fold into [-1, 1]: reduce mod 4 to [-2, 2], then reflect.
            let z4 = z - 4.0 * libm::floorf(z * 0.25 + 0.5);
            if z4 > 1.0 {
                2.0 - z4
            } else if z4 < -1.0 {
                -2.0 - z4
            } else {
                z4
            }
        }
        Shape::Tube => pade_tanh(z + TUBE_BIAS) - pade_tanh(TUBE_BIAS),
    }
}

/// Multi-shape waveshaper with 4× oversampling. Per-sample; no heap, no buffer.
#[derive(Clone, Copy)]
pub struct Drive {
    shape: Shape,
    drive: f32,
    tone: f32,
    mix: f32,
    up_prev: f32,
    dec_a: Svf,
    dec_b: Svf,
    tone_z: f32,
}
impl Drive {
    pub fn new(shape: Shape) -> Drive {
        Drive {
            shape,
            drive: 0.5,
            tone: 1.0,
            mix: 1.0,
            up_prev: 0.0,
            dec_a: Svf::new(),
            dec_b: Svf::new(),
            tone_z: 0.0,
        }
    }
    pub fn set_drive(&mut self, v: f32) { self.drive = v.clamp(0.0, 1.0); }
    pub fn set_tone(&mut self, v: f32) { self.tone = v.clamp(0.0, 1.0); }
    pub fn set_mix(&mut self, v: f32) { self.mix = v.clamp(0.0, 1.0); }
    pub fn set_shape(&mut self, code: u8) {
        self.shape = match code {
            1 => Shape::Hard,
            2 => Shape::Fold,
            3 => Shape::Tube,
            _ => Shape::Soft,
        };
    }

    pub fn process(&mut self, input: In, _dt: f32, out: &mut [f32]) {
        let g = 1.0 + self.drive * MAX_DRIVE;
        let tc = 0.05 + self.tone * 0.95;
        // Decimation LP coefficients are constant (cutoff = fs/2 in the 4× domain).
        let (a1, a2, a3) = svf_coeffs(DEC_G, DEC_K);
        for i in 0..out.len() {
            let x = input.at(i);
            let mut y = 0.0;
            for k in 0..4 {
                let frac = (k + 1) as f32 * 0.25;
                let up = self.up_prev + (x - self.up_prev) * frac;
                let sh = apply_shape(self.shape, up * g);
                let s1 = self.dec_a.tick(sh, DEC_K, a1, a2, a3, SvfResp::Lp);
                y = self.dec_b.tick(s1, DEC_K, a1, a2, a3, SvfResp::Lp);
            }
            self.up_prev = x;
            self.tone_z += tc * (y - self.tone_z);
            out[i] = x * (1.0 - self.mix) + self.tone_z * self.mix;
        }
    }
}
impl Default for Drive {
    fn default() -> Self {
        Self::new(Shape::Soft)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;
    use deluge_dsp_test::spectrum;

    const FS: f32 = 48_000.0;

    fn drive_render(d: &mut Drive, buf: &mut [f32], f0: f32, amp: f32) {
        let sine: std::vec::Vec<f32> = (0..buf.len())
            .map(|i| amp * (core::f32::consts::TAU * f0 * i as f32 / FS).sin())
            .collect();
        d.process(In::A(&sine), 1.0 / FS, buf);
    }

    #[test]
    fn each_shape_distorts() {
        for shape in [Shape::Soft, Shape::Hard, Shape::Fold, Shape::Tube] {
            let spec = spectrum::analyze(FS, |buf| {
                let mut d = Drive::new(shape);
                d.set_drive(0.8);
                d.set_tone(1.0);
                d.set_mix(1.0);
                drive_render(&mut d, buf, 220.0, 0.8);
            });
            let thd = spec.thd(220.0, 8);
            assert!(thd > 0.05, "shape should add harmonics: thd={thd}");
        }
    }

    #[test]
    fn drive_increases_distortion() {
        let thd = |drv: f32| -> f32 {
            spectrum::analyze(FS, |buf| {
                let mut d = Drive::new(Shape::Soft);
                d.set_drive(drv);
                d.set_tone(1.0);
                d.set_mix(1.0);
                drive_render(&mut d, buf, 220.0, 0.7);
            })
            .thd(220.0, 8)
        };
        assert!(thd(0.9) > thd(0.2), "more drive → more distortion");
    }

    #[test]
    fn hard_clips_harder_than_soft() {
        let thd = |shape| {
            spectrum::analyze(FS, |buf| {
                let mut d = Drive::new(shape);
                d.set_drive(0.7);
                d.set_tone(1.0);
                d.set_mix(1.0);
                drive_render(&mut d, buf, 220.0, 0.8);
            })
            .thd(220.0, 10)
        };
        assert!(thd(Shape::Hard) > thd(Shape::Soft), "hard should distort more than soft");
    }

    #[test]
    fn tube_adds_even_harmonics() {
        // Asymmetric tube produces a strong 2nd harmonic; symmetric soft does not.
        let second = |shape| {
            let spec = spectrum::analyze(FS, |buf| {
                let mut d = Drive::new(shape);
                d.set_drive(0.6);
                d.set_tone(1.0);
                d.set_mix(1.0);
                drive_render(&mut d, buf, 220.0, 0.7);
            });
            spec.level_at(440.0) // 2nd harmonic
        };
        assert!(second(Shape::Tube) > second(Shape::Soft) * 3.0, "tube should have a much stronger 2nd harmonic");
    }

    #[test]
    fn oversampling_suppresses_aliasing() {
        // A high fundamental driven hard: its harmonics fold; 4× + decimation
        // keeps inharmonic (alias) energy low. (Mirrors the Fi alias gate.)
        let spec = spectrum::analyze(FS, |buf| {
            let mut d = Drive::new(Shape::Hard);
            d.set_drive(0.9);
            d.set_tone(1.0);
            d.set_mix(1.0);
            drive_render(&mut d, buf, 3000.0, 0.9);
        });
        let db = spec.worst_alias_db(3000.0, 30.0);
        assert!(db < -20.0, "aliasing too high: worst_alias_db={db}");
    }

    #[test]
    fn tone_darkens_output() {
        let hf = |tone: f32| -> f32 {
            let spec = spectrum::analyze(FS, |buf| {
                let mut d = Drive::new(Shape::Hard);
                d.set_drive(0.8);
                d.set_tone(tone);
                d.set_mix(1.0);
                drive_render(&mut d, buf, 220.0, 0.8);
            });
            spec.level_at(3300.0) // a high harmonic (15th)
        };
        assert!(hf(0.2) < hf(1.0), "lower tone → less HF");
    }

    #[test]
    fn mix_zero_is_dry() {
        let mut d = Drive::new(Shape::Hard);
        d.set_drive(1.0);
        d.set_mix(0.0);
        let input = std::vec![0.5f32; 128];
        let mut out = std::vec![0.0f32; 128];
        d.process(In::A(&input), 1.0 / FS, &mut out);
        assert!(out.iter().all(|&v| (v - 0.5).abs() < 1e-4), "mix=0 should pass dry");
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn drive_is_finite_and_bounded(
            drv in 0.0f32..1.0,
            tone in 0.0f32..1.0,
            mix in 0.0f32..1.0,
            shape_code in 0u8..4,
            amp in 0.0f32..1.0,
        ) {
            let shapes = [Shape::Soft, Shape::Hard, Shape::Fold, Shape::Tube];
            let mut d = Drive::new(shapes[shape_code as usize]);
            d.set_drive(drv); d.set_tone(tone); d.set_mix(mix);
            let input = std::vec![amp; 1024];
            let mut out = std::vec![0.0f32; 1024];
            d.process(In::A(&input), 1.0 / FS, &mut out);
            for &v in &out {
                prop_assert!(v.is_finite());
                prop_assert!(v.abs() <= 4.0, "unbounded: {v}");
            }
        }
    }
}
