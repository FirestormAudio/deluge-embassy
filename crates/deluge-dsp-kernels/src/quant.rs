//! Quantizers and pitch conversion: snap a signal to N equal levels
//! (`QuantStep`), snap a semitone signal to a musical scale (`QuantPitch`), and
//! convert semitones to Hz (`Mtof`). Per-sample mono kernels — no heap.

use crate::{floorf, In};

/// Snap a bipolar `[-1,1]` signal to `N` evenly-spaced levels (endpoints
/// inclusive). Port 0 = input; `param 0 = N` (clamp ≥ 2).
#[derive(Clone, Copy)]
pub struct QuantStep {
    levels: u16,
}
impl QuantStep {
    pub fn new() -> QuantStep {
        QuantStep { levels: 2 }
    }
    pub fn set_levels(&mut self, n: u16) {
        self.levels = n.max(2);
    }
    pub fn process(&mut self, input: In, out: &mut [f32]) {
        let n = self.levels.max(2) as f32;
        for i in 0..out.len() {
            let u = (input.at(i).clamp(-1.0, 1.0) + 1.0) * 0.5; // [0,1]
            let q = floorf(u * (n - 1.0) + 0.5); // nearest of N levels
            out[i] = (q / (n - 1.0)) * 2.0 - 1.0; // back to [-1,1]
        }
    }
}
impl Default for QuantStep {
    fn default() -> Self {
        Self::new()
    }
}

/// Snap a semitone-valued signal to the nearest degree of a scale. Port 0 =
/// input (semitones). `param 0 = 12-bit pitch-class mask` (bit i ⇒ pc i allowed,
/// root-relative), `param 1 = root` (0–11). Output in semitones.
#[derive(Clone, Copy)]
pub struct QuantPitch {
    mask: u16,
    root: u8,
}
impl QuantPitch {
    pub fn new() -> QuantPitch {
        QuantPitch { mask: 0xFFF, root: 0 }
    }
    pub fn set_mask(&mut self, m: u16) {
        self.mask = m & 0xFFF;
    }
    pub fn set_root(&mut self, r: u8) {
        self.root = r % 12;
    }
    pub fn process(&mut self, input: In, out: &mut [f32]) {
        // Search order resolves ties upward and covers a full octave of pitch
        // classes, so a non-empty mask always matches within ±6 semitones.
        const SEARCH: [i32; 13] = [0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6];
        for i in 0..out.len() {
            let n = floorf(input.at(i) + 0.5) as i32; // round, ties → +∞
            let mut best = n;
            for d in SEARCH {
                let cand = n + d;
                let pc = (cand - self.root as i32).rem_euclid(12) as u16;
                if self.mask == 0 || (self.mask & (1 << pc)) != 0 {
                    best = cand;
                    break;
                }
            }
            out[i] = best as f32;
        }
    }
}
impl Default for QuantPitch {
    fn default() -> Self {
        Self::new()
    }
}

/// Semitone → Hz: `ref · 2^(x/12)`. Port 0 = input (semitones above the
/// reference), `param 0 = reference Hz` (default 440).
#[derive(Clone, Copy)]
pub struct Mtof {
    ref_hz: f32,
}
impl Mtof {
    pub fn new() -> Mtof {
        Mtof { ref_hz: 440.0 }
    }
    pub fn set_ref(&mut self, hz: f32) {
        self.ref_hz = hz;
    }
    pub fn process(&mut self, input: In, out: &mut [f32]) {
        for i in 0..out.len() {
            out[i] = self.ref_hz * libm::exp2f(input.at(i) / 12.0);
        }
    }
}
impl Default for Mtof {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;

    const MAJOR: u16 = 0b101010110101; // {0,2,4,5,7,9,11}
    const MINOR: u16 = 0b010110101101; // {0,2,3,5,7,8,10}

    #[test]
    fn qstep_two_levels() {
        let mut q = QuantStep::new(); // N = 2 → {-1, +1}
        let mut out = std::vec![0.0f32; 4];
        q.process(In::A(&[-0.3, 0.3, -0.9, 0.9]), &mut out);
        assert_eq!(out, std::vec![-1.0, 1.0, -1.0, 1.0]);
    }

    #[test]
    fn qstep_three_levels() {
        let mut q = QuantStep::new();
        q.set_levels(3); // {-1, 0, +1}
        let mut out = std::vec![0.0f32; 4];
        q.process(In::A(&[-0.6, -0.1, 0.1, 0.6]), &mut out);
        assert_eq!(out, std::vec![-1.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn qstep_clamps_levels_min_two() {
        let mut q = QuantStep::new();
        q.set_levels(0); // clamp to 2
        let mut out = std::vec![0.0f32; 2];
        q.process(In::A(&[-0.4, 0.4]), &mut out);
        assert_eq!(out, std::vec![-1.0, 1.0]);
    }

    #[test]
    fn qpitch_major_root0_snaps() {
        let mut q = QuantPitch::new();
        q.set_mask(MAJOR);
        q.set_root(0);
        let mut out = std::vec![0.0f32; 5];
        //  1.0→2 (tie-up),  3.0→4,  6.4→7,  0.0→0,  11.6→12 (octave wrap)
        q.process(In::A(&[1.0, 3.0, 6.4, 0.0, 11.6]), &mut out);
        assert_eq!(out, std::vec![2.0, 4.0, 7.0, 0.0, 12.0]);
    }

    #[test]
    fn qpitch_root_shift_moves_grid() {
        let mut q = QuantPitch::new();
        q.set_mask(MAJOR);
        q.set_root(2); // D major: allowed pcs {2,4,6,7,9,11,1}
        let mut out = std::vec![0.0f32; 2];
        q.process(In::A(&[3.0, 5.0]), &mut out);
        assert_eq!(out, std::vec![4.0, 6.0], "3→4, 5→6 under root 2");
    }

    #[test]
    fn qpitch_chromatic_is_identity() {
        let mut q = QuantPitch::new(); // mask 0xFFF
        let mut out = std::vec![0.0f32; 3];
        q.process(In::A(&[5.2, -3.4, 7.5]), &mut out);
        assert_eq!(out, std::vec![5.0, -3.0, 8.0], "nearest integer semitone");
    }

    #[test]
    fn qpitch_empty_mask_is_chromatic() {
        // An all-zero mask is guarded to chromatic (identity to nearest semitone),
        // not an infinite/failed search.
        let mut q = QuantPitch::new();
        q.set_mask(0);
        let mut out = std::vec![0.0f32; 3];
        q.process(In::A(&[5.2, -3.4, 7.5]), &mut out);
        assert_eq!(out, std::vec![5.0, -3.0, 8.0]);
    }

    #[test]
    fn qpitch_minor_snaps_known() {
        let mut q = QuantPitch::new();
        q.set_mask(MINOR); // {0,2,3,5,7,8,10}
        let mut out = std::vec![0.0f32; 3];
        q.process(In::A(&[1.0, 4.0, 6.0]), &mut out);
        //  1→2, 4→3 or 5 (tie→up 5), 6→5 or 7 (tie→up 7)
        assert_eq!(out, std::vec![2.0, 5.0, 7.0]);
    }

    #[test]
    fn mtof_octaves_exact() {
        let mut m = Mtof::new(); // ref 440
        let mut out = std::vec![0.0f32; 3];
        m.process(In::A(&[0.0, 12.0, -12.0]), &mut out);
        assert!((out[0] - 440.0).abs() < 1e-2, "0 → ref: {}", out[0]);
        assert!((out[1] - 880.0).abs() < 1e-2, "+12 → 2×ref: {}", out[1]);
        assert!((out[2] - 220.0).abs() < 1e-2, "-12 → ½×ref: {}", out[2]);
    }

    #[test]
    fn mtof_set_ref() {
        let mut m = Mtof::new();
        m.set_ref(100.0);
        let mut out = std::vec![0.0f32; 2];
        m.process(In::A(&[0.0, 12.0]), &mut out);
        assert!((out[0] - 100.0).abs() < 1e-3 && (out[1] - 200.0).abs() < 1e-3);
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn qstep_bounded(x in -2.0f32..2.0, n in 2u16..64) {
            let mut q = QuantStep::new();
            q.set_levels(n);
            let mut out = std::vec![0.0f32; 8];
            q.process(In::A(&std::vec![x; 8]), &mut out);
            for &v in &out {
                prop_assert!(v.is_finite() && v.abs() <= 1.0001, "qstep unbounded: {v}");
            }
        }
        #[test]
        fn mtof_positive_finite(x in -72.0f32..72.0) {
            let mut m = Mtof::new();
            let mut out = std::vec![0.0f32; 4];
            m.process(In::A(&std::vec![x; 4]), &mut out);
            for &v in &out {
                prop_assert!(v.is_finite() && v > 0.0, "mtof non-positive: {v}");
            }
        }
    }
}
