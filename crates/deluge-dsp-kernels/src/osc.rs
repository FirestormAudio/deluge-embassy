//! Phase-accumulating oscillator — the prototype's four naïve waveforms.
//! Serial phase recurrence, so scalar in P0 (const-freq SIMD is a later opt).

use crate::{fast_sin, floorf, In};

#[derive(Clone, Copy)]
pub enum Wave {
    Sine,
    Saw,
    Square,
    Tri,
}

#[derive(Clone, Copy)]
pub struct Osc {
    phase: f32,
}

impl Osc {
    pub fn new() -> Osc {
        Osc { phase: 0.0 }
    }

    /// Fill `out` with one block. `freq` in Hz (const or audio-rate); `dt = 1/sr`.
    pub fn process(&mut self, wave: Wave, freq: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            let p = self.phase;
            *s = match wave {
                Wave::Sine => fast_sin(p),
                Wave::Saw => 2.0 * p - 1.0,
                Wave::Square => {
                    if p < 0.5 {
                        1.0
                    } else {
                        -1.0
                    }
                }
                Wave::Tri => 1.0 - 4.0 * (p - 0.5).abs(),
            };
            self.phase += freq.at(i) * dt;
            self.phase -= floorf(self.phase);
        }
    }
}

impl Default for Osc {
    fn default() -> Self {
        Osc::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// P0 gate (spec §8): for any wave/freq in the audio range, every
        /// sample of a rendered block is finite and stays within the
        /// oscillator's nominal [-1, 1] range (with slack for fast_sin's
        /// small overshoot).
        #[test]
        fn osc_output_is_finite_and_bounded(
            wave_idx in 0u8..4,
            freq in 0.0f32..=20_000.0,
        ) {
            let wave = match wave_idx {
                0 => Wave::Sine,
                1 => Wave::Saw,
                2 => Wave::Square,
                _ => Wave::Tri,
            };
            let mut osc = Osc::new();
            let mut out = [0.0f32; 64];
            let dt = 1.0 / 48_000.0;
            osc.process(wave, In::K(freq), dt, &mut out);
            for s in out {
                prop_assert!(s.is_finite());
                prop_assert!(s >= -1.001 && s <= 1.001);
            }
        }
    }

    #[test]
    fn saw_ramps_from_minus_one_over_one_cycle() {
        // 4 Hz at 16 Hz sample rate → phase advances 0.25/sample; one cycle = 4 samples.
        let mut osc = Osc::new();
        let mut out = [0.0f32; 4];
        osc.process(Wave::Saw, In::K(4.0), 1.0 / 16.0, &mut out);
        // saw = 2*phase - 1, sampled at phase 0, .25, .5, .75
        assert!((out[0] - (-1.0)).abs() < 1e-6);
        assert!((out[1] - (-0.5)).abs() < 1e-6);
        assert!((out[2] - 0.0).abs() < 1e-6);
        assert!((out[3] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn sine_is_bounded_and_starts_near_zero() {
        let mut osc = Osc::new();
        let mut out = [0.0f32; 64];
        osc.process(Wave::Sine, In::K(100.0), 1.0 / 1000.0, &mut out);
        assert!(out[0].abs() < 1e-3);
        assert!(out.iter().all(|s| s.abs() <= 1.001));
    }
}
