//! Low-frequency modulation source: a phase accumulator driving raw (non-band-
//! limited) shapes, bipolar [−1,1]. Per-sample, mono, no buffer, deterministic.

use crate::{In, fast_sin, floorf};

#[derive(Clone, Copy)]
pub enum LfoShape {
    Sine,
    Tri,
    Saw,
    Square,
    SampleHold,
    Random,
}

#[derive(Clone, Copy)]
pub struct Lfo {
    shape: LfoShape,
    phase: f32,
    phase_offset: f32,
    rng: u32,
    sh_cur: f32,
    sh_prev: f32,
}

impl Lfo {
    pub fn new() -> Lfo {
        let mut l = Lfo {
            shape: LfoShape::Sine,
            phase: 0.0,
            phase_offset: 0.0,
            rng: 0x2545_F491,
            sh_cur: 0.0,
            sh_prev: 0.0,
        };
        l.sh_cur = l.next_rand();
        l
    }
    fn next_rand(&mut self) -> f32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        (x as f32 / u32::MAX as f32) * 2.0 - 1.0
    }
    pub fn set_shape(&mut self, code: u8) {
        self.shape = match code {
            1 => LfoShape::Tri,
            2 => LfoShape::Saw,
            3 => LfoShape::Square,
            4 => LfoShape::SampleHold,
            5 => LfoShape::Random,
            _ => LfoShape::Sine,
        };
    }
    pub fn set_phase(&mut self, v: f32) {
        self.phase_offset = v;
    }
    pub fn retrigger(&mut self) {
        self.phase = self.phase_offset - floorf(self.phase_offset);
        self.sh_prev = self.sh_cur;
        self.sh_cur = self.next_rand();
    }
    pub fn process(&mut self, rate: In, dt: f32, out: &mut [f32]) {
        for i in 0..out.len() {
            let inc = rate.at(i) * dt;
            let prev = self.phase;
            let np = self.phase + inc;
            self.phase = np - floorf(np);
            if self.phase < prev || inc >= 1.0 {
                self.sh_prev = self.sh_cur;
                self.sh_cur = self.next_rand();
            }
            let po = self.phase + self.phase_offset;
            let p = po - floorf(po);
            out[i] = match self.shape {
                LfoShape::Sine => fast_sin(p),
                LfoShape::Tri => {
                    if p < 0.5 {
                        p * 4.0 - 1.0
                    } else {
                        3.0 - p * 4.0
                    }
                }
                LfoShape::Saw => 2.0 * p - 1.0,
                LfoShape::Square => {
                    if p < 0.5 {
                        1.0
                    } else {
                        -1.0
                    }
                }
                LfoShape::SampleHold => self.sh_cur,
                LfoShape::Random => self.sh_prev + (self.sh_cur - self.sh_prev) * self.phase,
            };
        }
    }
}
impl Default for Lfo {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;

    const FS: f32 = 48_000.0;

    fn render(shape: u8, rate: f32, n: usize) -> std::vec::Vec<f32> {
        let mut l = Lfo::new();
        l.set_shape(shape);
        let mut out = std::vec![0.0f32; n];
        let r = std::vec![rate; n];
        l.process(In::A(&r), 1.0 / FS, &mut out);
        out
    }

    #[test]
    fn all_shapes_bounded_bipolar() {
        for s in 0u8..6 {
            let out = render(s, 3.0, 48_000);
            assert!(
                out.iter().all(|v| v.is_finite() && v.abs() <= 1.0001),
                "shape {s} out of range"
            );
        }
    }

    #[test]
    fn saw_ramps_up() {
        // One period of a 1 Hz saw: rises monotonically until the wrap.
        let out = render(2 /* Saw */, 1.0, 48_000);
        assert!(
            (out[0] - (-1.0)).abs() < 0.01,
            "saw starts near −1: {}",
            out[0]
        );
        assert!(out[24_000] > out[100], "saw rising"); // quarter → later
        assert!(
            out[47_000] > 0.9,
            "saw near +1 before wrap: {}",
            out[47_000]
        );
    }

    #[test]
    fn square_is_plus_minus_one() {
        let out = render(3 /* Square */, 1.0, 48_000);
        assert!((out[100] - 1.0).abs() < 1e-6, "first half = +1");
        assert!((out[30_000] + 1.0).abs() < 1e-6, "second half = −1");
    }

    #[test]
    fn sample_hold_is_piecewise_constant() {
        // Within a cycle S&H holds; across cycles it (almost surely) steps.
        let out = render(4 /* SampleHold */, 1.0, 96_000); // 2 cycles
        assert!((out[100] - out[40_000]).abs() < 1e-6, "held within a cycle");
        assert!((out[100] - out[60_000]).abs() > 1e-6, "steps across cycles");
    }

    #[test]
    fn rate_sets_period() {
        // 2 Hz saw: wraps at samples 24000 and 48000 — render 49000 so both fall
        // inside the buffer (a saw reset is a big negative jump).
        let out = render(2 /* Saw */, 2.0, 49_000);
        let wraps = out.windows(2).filter(|w| w[1] - w[0] < -1.0).count();
        assert_eq!(wraps, 2, "2 Hz → 2 wraps captured in 49000 samples");
    }

    #[test]
    fn retrigger_resets_phase() {
        let mut l = Lfo::new();
        l.set_shape(2); // Saw
        let mut out = std::vec![0.0f32; 1000];
        let r = std::vec![5.0f32; 1000];
        l.process(In::A(&r), 1.0 / FS, &mut out); // advance somewhere mid-cycle
        l.retrigger();
        let mut one = [0.0f32; 1];
        l.process(In::A(&[5.0f32][..]), 1.0 / FS, &mut one);
        // After retrigger (phase_offset=0) the saw is ≈ −1 + one increment.
        assert!(
            one[0] < -0.99,
            "retrigger resets phase to ~start: {}",
            one[0]
        );
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn lfo_is_finite_and_bounded(shape in 0u8..6, rate in 0.0f32..50.0) {
            let out = render(shape, rate, 4_000);
            for &v in &out {
                prop_assert!(v.is_finite() && v.abs() <= 1.0001, "out of range: {v}");
            }
        }
    }
}
