//! White/pink/brown noise via xorshift32 → colored filters. Serial recurrence → scalar.

/// Which noise color a [`Noise`] generator produces.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum NoiseColor {
    /// Flat magnitude spectrum (xorshift32 direct).
    White,
    /// ~−3 dB/octave magnitude slope (1/f power) via the Paul Kellet refined filter.
    Pink,
    /// ~−6 dB/octave magnitude slope (1/f² power) via a leaky integrator.
    Brown,
}

// Pink (Kellet refined filter) output normalization. MEASURED (see task-1 report,
// `examples/measure_noise.rs`): raw Kellet sum (gain=1) peaks ≈7.39 over 32×8192
// samples; with PINK_GAIN below, the worst observed |sample| over a 20,000-seed ×
// 512-sample sweep (~10.2M samples, fresh generator state per seed, as in real use)
// is ≈0.910 — comfortably under 1.0 (pink has NO clamp, so this margin IS the bound).
// Measured slope over [200,12000] Hz @ 48 kHz (32-block average): ≈ −2.99 dB/oct,
// stable to < 0.07 dB across 5 independent measurement runs.
const PINK_GAIN: f32 = 0.115;

// Brown (leaky integrator): `brown = (brown + white*BROWN_RATE) * BROWN_LEAK`,
// `out = (brown*BROWN_GAIN).clamp(-1,1)`. BROWN_LEAK sets the low-frequency corner
// (and, since leak < 1, the walk's steady-state variance is bounded — this, not the
// clamp, is the primary bound); BROWN_RATE is the per-sample step size; BROWN_GAIN
// normalizes to use most of the ±1 range without clamping often. MEASURED (see
// task-1 report, `examples/measure_noise.rs`): with these constants, worst observed
// |sample| over a 20,000-seed × 512-sample sweep (~10.2M samples) is ≈0.730, ZERO
// clamp hits. Measured slope over [200,12000] Hz @ 48 kHz (32-block average): ≈
// −5.93 dB/oct, stable to < 0.08 dB across 5 independent measurement runs (an
// earlier candidate, RATE=0.02/LEAK=0.992/GAIN=8.8, clamped ~21% of samples — the
// GAIN was tuned against the wrong (unbounded worst-case) reference; this GAIN was
// picked from the actual measured raw-state worst-case instead).
const BROWN_RATE: f32 = 0.01;
const BROWN_LEAK: f32 = 0.995;
const BROWN_GAIN: f32 = 2.8;

/// xorshift32-seeded noise generator (white, pink, or brown). Seed with a nonzero
/// value (0 is remapped to a fixed nonzero constant).
#[derive(Clone, Copy)]
pub struct Noise {
    rng: u32,
    color: NoiseColor,
    /// Kellet pink-filter state (b0..b6).
    pink: [f32; 7],
    /// Leaky-integrator state for brown noise.
    brown: f32,
}

impl Noise {
    /// White noise (unchanged API + output vs. the pre-color-support kernel).
    pub fn seeded(seed: u32) -> Noise {
        Noise::seeded_color(seed, NoiseColor::White)
    }

    pub fn seeded_color(seed: u32, color: NoiseColor) -> Noise {
        Noise {
            rng: if seed == 0 { 0x2545_F491 } else { seed },
            color,
            pink: [0.0; 7],
            brown: 0.0,
        }
    }

    /// One noise sample (advances rng + color state). Shared by `process` and `PolyNoise`.
    #[inline]
    pub(crate) fn tick(&mut self) -> f32 {
        let mut r = self.rng;
        r ^= r << 13;
        r ^= r >> 17;
        r ^= r << 5;
        self.rng = r;
        let white = (r as i32 as f32) / (i32::MAX as f32);
        match self.color {
            NoiseColor::White => white,
            NoiseColor::Pink => {
                let b = &mut self.pink;
                b[0] = 0.99886 * b[0] + white * 0.0555179;
                b[1] = 0.99332 * b[1] + white * 0.0750759;
                b[2] = 0.96900 * b[2] + white * 0.153852;
                b[3] = 0.86650 * b[3] + white * 0.3104856;
                b[4] = 0.55000 * b[4] + white * 0.5329522;
                b[5] = -0.7616 * b[5] - white * 0.016898;
                let pink =
                    (b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6] + white * 0.5362) * PINK_GAIN;
                b[6] = white * 0.115926;
                pink
            }
            NoiseColor::Brown => {
                self.brown = (self.brown + white * BROWN_RATE) * BROWN_LEAK;
                (self.brown * BROWN_GAIN).clamp(-1.0, 1.0)
            }
        }
    }

    pub fn process(&mut self, out: &mut [f32]) {
        for s in out.iter_mut() {
            *s = self.tick();
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use deluge_dsp_test::FFT_N;
    use deluge_dsp_test::spectrum::{Spectrum, analyze_buf, slope_db_per_octave};
    use proptest::prelude::*;
    use std::vec;

    proptest! {
        /// P0 gate (spec §8): for any nonzero seed, every sample of a
        /// rendered block is finite and stays within [-1, 1].
        #[test]
        fn noise_output_is_finite_and_bounded(seed in 1u32..=u32::MAX) {
            let mut nz = Noise::seeded(seed);
            let mut out = [0.0f32; 64];
            nz.process(&mut out);
            for s in out {
                prop_assert!(s.is_finite());
                prop_assert!(s >= -1.0 && s <= 1.0);
            }
        }
    }

    #[test]
    fn noise_is_bounded_and_deterministic_per_seed() {
        let mut a = Noise::seeded(0x2545_F491);
        let mut b = Noise::seeded(0x2545_F491);
        let mut oa = [0.0f32; 32];
        let mut ob = [0.0f32; 32];
        a.process(&mut oa);
        b.process(&mut ob);
        assert_eq!(oa, ob); // same seed → same stream
        assert!(oa.iter().all(|s| s.abs() <= 1.0));
        assert!(oa.iter().any(|s| *s != oa[0])); // not a constant
    }

    /// Pre-change reference: the exact white output for seed 12345, first 8 samples,
    /// captured from the kernel before color support was added. Guards against any
    /// accidental change to the white xorshift path.
    const WHITE_REFERENCE_SEED_12345: [f32; 8] = [
        -0.4461226,
        0.7903454,
        -0.68845946,
        0.91059136,
        0.33473703,
        -0.47094506,
        -0.004321862,
        -0.28594217,
    ];

    #[test]
    fn white_unchanged() {
        let mut w = Noise::seeded(12345);
        let mut a = [0.0f32; 64];
        w.process(&mut a);
        let mut w2 = Noise::seeded_color(12345, NoiseColor::White);
        let mut b = [0.0f32; 64];
        w2.process(&mut b);
        assert_eq!(a, b); // colored White == plain seeded white, bit-exact
        assert_eq!(&a[..8], &WHITE_REFERENCE_SEED_12345[..]);
    }

    /// Average |spectrum| over `blocks` independent FFT_N captures for a stable slope
    /// estimate (noise is stochastic — a single block's slope is noisy).
    fn capture(color: NoiseColor, blocks: usize) -> Spectrum {
        let sr = 48_000.0f32;
        let mut acc = vec![0.0f32; FFT_N / 2 + 1];
        let mut bin_hz = 0.0f32;
        for block in 0..blocks {
            // Distinct, nonzero seed per block for independent captures.
            let seed = 0x9E37_79B9u32.wrapping_add((block as u32).wrapping_mul(0x0100_0193));
            let mut n = Noise::seeded_color(seed, color);
            let mut buf = [0.0f32; FFT_N];
            n.process(&mut buf);
            let spec = analyze_buf(sr, &buf);
            bin_hz = spec.bin_hz;
            for (a, b) in acc.iter_mut().zip(spec.bins.iter()) {
                *a += b;
            }
        }
        for a in acc.iter_mut() {
            *a /= blocks as f32;
        }
        Spectrum {
            bins: acc,
            bin_hz,
            sample_rate: sr,
        }
    }

    #[test]
    fn slopes_match_colors() {
        let wslope = slope_db_per_octave(&capture(NoiseColor::White, 32), 200.0, 12_000.0);
        let pslope = slope_db_per_octave(&capture(NoiseColor::Pink, 32), 200.0, 12_000.0);
        let bslope = slope_db_per_octave(&capture(NoiseColor::Brown, 32), 200.0, 12_000.0);
        // Gates tightened from measurement (task-1 report, examples/measure_noise.rs):
        // over [200, 12000] Hz at 48 kHz, 32-block magnitude average, 5 independent
        // measurement runs (different seed sets) gave: white in [-0.03, 0.04] dB/oct
        // (spread < 0.07), pink in [-3.05, -2.98] (spread < 0.07), brown in
        // [-6.00, -5.92] (spread < 0.08). Tolerances below are ~4x that spread.
        assert!(wslope.abs() < 0.3, "white ~0, got {wslope}");
        assert!((pslope - (-3.0)).abs() < 0.3, "pink ~-3, got {pslope}");
        assert!((bslope - (-6.0)).abs() < 0.3, "brown ~-6, got {bslope}");
    }

    proptest! {
        #[test]
        fn all_colors_bounded(seed in 1u32..=u32::MAX, c in 0u8..3) {
            let color = match c { 0 => NoiseColor::White, 1 => NoiseColor::Pink, _ => NoiseColor::Brown };
            let mut n = Noise::seeded_color(seed, color);
            let mut out = [0.0f32; 256];
            n.process(&mut out);
            for s in out { prop_assert!(s.is_finite() && s.abs() <= 1.0); }
        }
    }
}
