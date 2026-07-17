//! Modulation shaping: a non-linear transfer curve and a settable scalar source.
//! Tiny per-sample mono kernels — no heap, no buffer.

use crate::In;

/// Odd-symmetric non-linear transfer (Schlick's *bias*). Stateless, so this is
/// a free function (like `math::mul`). `in0` = input, `in1` = k ∈ [-1,1]:
/// `k=0` linear, `k>0` exponential (ease-in), `k<0` logarithmic (ease-out).
/// Bipolar-in → bipolar-out, monotonic, fixed points at 0 and ±1.
pub fn curve(input: In, k: In, out: &mut [f32]) {
    for i in 0..out.len() {
        let x = input.at(i).clamp(-1.0, 1.0);
        let b = ((k.at(i) + 1.0) * 0.5).clamp(0.05, 0.95); // k∈[-1,1] → b∈[0.05,0.95]
        let s = if x < 0.0 { -1.0 } else { 1.0 };
        let u = if x < 0.0 { -x } else { x }; // |x| ∈ [0,1]
        let y = u / ((1.0 / b - 2.0) * (1.0 - u) + 1.0); // (0,0)→(1,1)
        out[i] = s * y;
    }
}

/// A held, runtime-settable scalar source — the macro primitive. No input
/// ports; `param 0 = value`. Fills the output block with `value`.
#[derive(Clone, Copy)]
pub struct Ctrl {
    value: f32,
}
impl Ctrl {
    pub fn new() -> Ctrl {
        Ctrl { value: 0.0 }
    }
    pub fn set_value(&mut self, v: f32) {
        self.value = v;
    }
    pub fn process(&mut self, out: &mut [f32]) {
        for s in out.iter_mut() {
            *s = self.value;
        }
    }
}
impl Default for Ctrl {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;

    fn ramp(n: usize) -> std::vec::Vec<f32> {
        (0..n)
            .map(|i| i as f32 / (n - 1) as f32 * 2.0 - 1.0)
            .collect() // -1..=1
    }

    #[test]
    fn curve_linear_at_k0() {
        let x = ramp(21);
        let mut out = std::vec![0.0f32; 21];
        curve(In::A(&x), In::K(0.0), &mut out);
        for i in 0..21 {
            assert!(
                (out[i] - x[i]).abs() < 1e-6,
                "k=0 is identity at {i}: {} vs {}",
                out[i],
                x[i]
            );
        }
    }

    #[test]
    fn curve_fixed_points_and_midpoint_gain() {
        // Fixed points 0 and ±1 for any k; k>0 boosts the midpoint, k<0 attenuates.
        let mut out = std::vec![0.0f32; 3];
        curve(In::A(&[-1.0, 0.0, 1.0]), In::K(0.6), &mut out);
        assert!((out[0] + 1.0).abs() < 1e-6 && out[1].abs() < 1e-6 && (out[2] - 1.0).abs() < 1e-6);
        // b = (0.6+1)/2 = 0.8; bias(0.5,0.8) = 0.8.
        let mut m = std::vec![0.0f32; 1];
        curve(In::A(&[0.5]), In::K(0.6), &mut m);
        assert!((m[0] - 0.8).abs() < 1e-4, "k=0.6 boosts 0.5→0.8: {}", m[0]);
        // b = 0.2; bias(0.5,0.2) = 0.2.
        curve(In::A(&[0.5]), In::K(-0.6), &mut m);
        assert!(
            (m[0] - 0.2).abs() < 1e-4,
            "k=-0.6 attenuates 0.5→0.2: {}",
            m[0]
        );
    }

    #[test]
    fn curve_odd_symmetric_and_monotonic() {
        let x = ramp(41);
        let mut out = std::vec![0.0f32; 41];
        curve(In::A(&x), In::K(0.7), &mut out);
        // odd symmetry: out[mid-j] == -out[mid+j]
        for j in 0..=20 {
            assert!(
                (out[20 - j] + out[20 + j]).abs() < 1e-5,
                "odd symmetry at {j}"
            );
        }
        // monotonic non-decreasing, bounded [-1,1]
        for i in 1..41 {
            assert!(out[i] >= out[i - 1] - 1e-6, "monotonic at {i}");
            assert!(out[i].abs() <= 1.0001, "bounded at {i}: {}", out[i]);
        }
    }

    #[test]
    fn ctrl_holds_and_updates() {
        let mut c = Ctrl::new();
        let mut out = std::vec![9.0f32; 4];
        c.process(&mut out);
        assert_eq!(out, std::vec![0.0, 0.0, 0.0, 0.0], "new Ctrl outputs 0");
        c.set_value(3.5);
        c.process(&mut out);
        assert_eq!(
            out,
            std::vec![3.5, 3.5, 3.5, 3.5],
            "set_value changes output"
        );
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn curve_stays_bounded(x in -2.0f32..2.0, k in -1.0f32..1.0) {
            let mut out = std::vec![0.0f32; 8];
            curve(In::A(&std::vec![x; 8]), In::K(k), &mut out);
            for &v in &out {
                prop_assert!(v.is_finite() && v.abs() <= 1.0001, "curve unbounded: {v}");
            }
        }
    }
}
