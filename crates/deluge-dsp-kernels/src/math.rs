//! Element-wise block math (mul/add/sub). Fully data-parallel, so this is the
//! crate's `core::simd` showcase: when both operands are audio-rate blocks and
//! the `simd` feature is on, process in `f32` lanes; otherwise scalar. The two
//! paths are null-tested against each other.

use crate::In;

macro_rules! binop {
    ($name:ident, $op:tt, $simd_op:tt) => {
        pub fn $name(a: In, b: In, out: &mut [f32]) {
            // Fast path: both blocks, SIMD feature on.
            #[cfg(feature = "simd")]
            {
                if let (In::A(av), In::A(bv)) = (a, b) {
                    use core::simd::f32x8;
                    let n = out.len();
                    let chunks = n / 8;
                    for c in 0..chunks {
                        let i = c * 8;
                        let va = f32x8::from_slice(&av[i..i + 8]);
                        let vb = f32x8::from_slice(&bv[i..i + 8]);
                        (va $simd_op vb).copy_to_slice(&mut out[i..i + 8]);
                    }
                    for i in (chunks * 8)..n {
                        out[i] = av[i] $op bv[i];
                    }
                    return;
                }
            }
            // Scalar fallback (also the const-operand path).
            for (i, s) in out.iter_mut().enumerate() {
                *s = a.at(i) $op b.at(i);
            }
        }
    };
}

binop!(mul, *, *);
binop!(add, +, +);
binop!(sub, -, -);

/// Constant-power stereo pan gains for `position ∈ [-1, +1]` (clamped):
/// `-1` = hard left `(1, 0)`, `0` = center `(√½, √½)` (−3 dB), `+1` = hard
/// right `(0, 1)`. `gain_l² + gain_r² == 1`. Trig-free (`sqrtf`; the Cortex-A9
/// has hardware `VSQRT`).
#[inline]
pub fn pan_gains(position: f32) -> (f32, f32) {
    let p = (position.clamp(-1.0, 1.0) + 1.0) * 0.5; // → [0, 1]
    (libm::sqrtf(1.0 - p), libm::sqrtf(p))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::In;

    #[test]
    fn mul_block_by_block() {
        let a = [1.0, 2.0, 3.0, 4.0];
        let b = [10.0, 10.0, 10.0, 10.0];
        let mut out = [0.0f32; 4];
        mul(In::A(&a), In::A(&b), &mut out);
        assert_eq!(out, [10.0, 20.0, 30.0, 40.0]);
    }

    #[test]
    fn add_const_broadcasts() {
        let a = [1.0, 2.0, 3.0];
        let mut out = [0.0f32; 3];
        add(In::A(&a), In::K(0.5), &mut out);
        assert_eq!(out, [1.5, 2.5, 3.5]);
    }

    #[test]
    fn sub_matches_scalar_reference_on_a_long_block() {
        // Null test: whatever path compiles (SIMD or scalar), it must equal the
        // element-wise reference. On host CI this exercises SSE (with `simd`);
        // on device, NEON.
        let a: [f32; 37] = core::array::from_fn(|i| i as f32 * 0.3);
        let b: [f32; 37] = core::array::from_fn(|i| 100.0 - i as f32);
        let mut out = [0.0f32; 37];
        sub(In::A(&a), In::A(&b), &mut out);
        for i in 0..37 {
            assert!((out[i] - (a[i] - b[i])).abs() < 1e-6);
        }
    }
}

#[cfg(test)]
mod pan_tests {
    use super::*;

    #[test]
    fn pan_hard_left_and_right() {
        let (l, r) = pan_gains(-1.0);
        assert!((l - 1.0).abs() < 1e-6 && r.abs() < 1e-6, "hard left: {l},{r}");
        let (l, r) = pan_gains(1.0);
        assert!(l.abs() < 1e-6 && (r - 1.0).abs() < 1e-6, "hard right: {l},{r}");
    }

    #[test]
    fn pan_center_is_minus_3db() {
        let (l, r) = pan_gains(0.0);
        let c = (0.5f32).sqrt(); // 0.7071…
        assert!((l - c).abs() < 1e-6 && (r - c).abs() < 1e-6, "center: {l},{r}");
    }

    #[test]
    fn pan_is_constant_power_and_clamped() {
        // gl² + gr² == 1 across the range, and out-of-range clamps.
        for k in -12..=12 {
            let pos = k as f32 / 10.0; // -1.2 .. 1.2 (exercises the clamp)
            let (l, r) = pan_gains(pos);
            assert!((l * l + r * r - 1.0).abs() < 1e-5, "power@{pos}: {l},{r}");
            assert!(l >= 0.0 && r >= 0.0);
        }
        // Beyond the range stays pinned to the ends.
        assert_eq!(pan_gains(-5.0), pan_gains(-1.0));
        assert_eq!(pan_gains(5.0), pan_gains(1.0));
    }
}
