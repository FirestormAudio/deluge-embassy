//! Polyphony kernels: a per-voice settable source (`PolyCtrl`), a poly
//! oscillator (`PolyOsc`), and the voice→mono collapse (`voice_sum`). All poly
//! buffers are voice-interleaved (sample-major): `tile[i * VOICES + v]` is voice
//! `v` at sample `i`, so the voice loop vectorizes to `f32x8` on NEON.

use crate::{fast_sin, floorf};

/// Voices processed in parallel per poly node. Fixed at compile time.
pub const VOICES: usize = 8;

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

/// A poly oscillator. `pitch` is a voice-interleaved tile of per-voice Hz;
/// writes a voice-interleaved audio tile. SoA phase; voice loop is the inner
/// (vectorizable) dimension. Raw sine shape (band-limiting is a later concern).
#[derive(Clone, Copy)]
pub struct PolyOsc {
    phase: [f32; VOICES], // [0,1) per voice
}
impl PolyOsc {
    pub fn new() -> PolyOsc {
        PolyOsc { phase: [0.0; VOICES] }
    }
    /// `pitch` and `out` are voice-interleaved, length `VOICES * n_samples`.
    pub fn process(&mut self, pitch: &[f32], dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            for v in 0..VOICES {
                let f = pitch[i * VOICES + v];
                let mut p = self.phase[v] + f * dt;
                p -= floorf(p); // wrap [0,1)
                self.phase[v] = p;
                out[i * VOICES + v] = fast_sin(p);
            }
        }
    }
}
impl Default for PolyOsc {
    fn default() -> Self {
        Self::new()
    }
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

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::{fast_sin, floorf}; // explicit so tests build under `--features simd` too

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
        osc.process(&pitch, dt, &mut out);
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
            let mut osc = PolyOsc::new();
            let mut out = std::vec![0.0f32; VOICES * n];
            osc.process(&pitch, dt, &mut out);
            for &s in &out {
                prop_assert!(s.is_finite() && s.abs() <= 1.0001, "polyosc unbounded: {s}");
            }
        }
    }
}
