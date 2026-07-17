//! Property-based tests (proptest).

use super::*;
use proptest::prelude::*;

// Generate valid i32 values for raw fixed-point
fn raw_value() -> impl Strategy<Value = i32> {
    any::<i32>()
}

// Generate valid float values for Q31 (-1.0 to 1.0)
fn q31_float() -> impl Strategy<Value = f32> {
    -1.0f32..1.0f32
}

// Generate valid float values for Q16 (-32768.0 to 32767.0)
fn q16_float() -> impl Strategy<Value = f32> {
    -32768.0f32..32768.0f32
}

// Independent reference for Q31 saturating division (FRAC_BITS = 31, ROUNDED =
// false). Re-derives the result from the documented semantics in plain i64 math
// rather than reusing the crate's own expression.
fn ref_q31_div(a_raw: i32, b_raw: i32) -> i32 {
    if b_raw == 0 {
        return if a_raw >= 0 {
            Q31::MAX.raw()
        } else {
            Q31::MIN.raw()
        };
    }
    let dividend = (a_raw as i64) << 31;
    (dividend / b_raw as i64).clamp(i32::MIN as i64, i32::MAX as i64) as i32
}

proptest! {
    #[test]
    fn prop_addition_commutative(a in raw_value(), b in raw_value()) {
        let fp_a = Q31::from_raw(a);
        let fp_b = Q31::from_raw(b);

        assert_eq!((fp_a + fp_b).raw(), (fp_b + fp_a).raw());
    }

    #[test]
    fn prop_multiplication_commutative(a in raw_value(), b in raw_value()) {
        let fp_a = Q31::from_raw(a);
        let fp_b = Q31::from_raw(b);

        assert_eq!((fp_a * fp_b).raw(), (fp_b * fp_a).raw());
    }

    #[test]
    fn prop_zero_identity_addition(a in raw_value()) {
        let fp = Q31::from_raw(a);
        assert_eq!((fp + Q31::ZERO).raw(), fp.raw());
        assert_eq!((Q31::ZERO + fp).raw(), fp.raw());
    }

    #[test]
    fn prop_zero_identity_multiplication(a in raw_value()) {
        let fp = Q31::from_raw(a);
        assert_eq!((fp * Q31::ZERO).raw(), Q31::ZERO.raw());
        assert_eq!((Q31::ZERO * fp).raw(), Q31::ZERO.raw());
    }

    #[test]
    fn prop_negation_involution(a in raw_value()) {
        let fp = Q31::from_raw(a);
        let neg_neg = -(-fp);

        // Double negation should give original (except for MIN)
        if a != i32::MIN {
            assert_eq!(neg_neg.raw(), fp.raw());
        }
    }

    #[test]
    fn prop_addition_subtraction_inverse(a in raw_value(), b in raw_value()) {
        let fp_a = Q31::from_raw(a);
        let fp_b = Q31::from_raw(b);

        // (a + b) - b should equal a (when no saturation occurs)
        let sum = fp_a + fp_b;
        let diff = sum - fp_b;

        // Check if saturation occurred in either direction
        let saturated = (a > 0 && b > 0 && sum.raw() == i32::MAX) ||
                       (a < 0 && b < 0 && sum.raw() == i32::MIN) ||
                       (sum - fp_b).raw() == i32::MIN ||
                       (sum - fp_b).raw() == i32::MAX;

        if !saturated {
            // Allow for rounding error of 1 due to saturation
            let error = (diff.raw() - fp_a.raw()).abs();
            assert!(error <= 1, "Error too large: {} vs {}", diff.raw(), fp_a.raw());
        }
    }

    #[test]
    fn prop_comparison_transitivity(a in raw_value(), b in raw_value(), c in raw_value()) {
        let fp_a = Q31::from_raw(a);
        let fp_b = Q31::from_raw(b);
        let fp_c = Q31::from_raw(c);

        // If a <= b and b <= c, then a <= c
        if fp_a <= fp_b && fp_b <= fp_c {
            assert!(fp_a <= fp_c);
        }
    }

    #[test]
    fn prop_float_roundtrip_q31(f in q31_float()) {
        let fp = Q31::from_float(f);
        let back = fp.to_float();

        // Should be close (within quantization error)
        let error = (back - f).abs();
        assert!(error < 0.001);
    }

    #[test]
    fn prop_float_roundtrip_q16(f in q16_float()) {
        let fp = Q16::from_float(f);
        let back = fp.to_float();

        // Should be close (within quantization error)
        let error = (back - f).abs();
        assert!(error < 0.1);
    }

    #[test]
    fn prop_abs_non_negative(a in raw_value()) {
        let fp = Q31::from_raw(a);
        let abs_fp = fp.abs();

        assert!(abs_fp.raw() >= 0);
    }

    #[test]
    fn prop_abs_idempotent(a in raw_value()) {
        let fp = Q31::from_raw(a);
        let abs_once = fp.abs();
        let abs_twice = abs_once.abs();

        assert_eq!(abs_once.raw(), abs_twice.raw());
    }

    #[test]
    fn prop_multiplication_by_one_identity(a in raw_value()) {
        let fp = Q31::from_raw(a);
        let one = Q31::from_raw(Q31::ONE);
        let result = fp * one;

        // `one` is `Q31::ONE == i32::MAX` (≈ 1 − 2⁻³¹, since exact 1.0 isn't
        // representable). On ARM the Q31 multiply is `SMMUL` (a 64→hi-32 i.e.
        // `>>32`) then `*2`, which drops one more bit than the portable `>>31`,
        // so the hardware path can be up to 2 LSB off here (the portable path
        // stays within 1). This matches the Deluge firmware's `q31_mult`.
        let diff = (result.raw() - fp.raw()).abs();
        assert!(diff <= 2);
    }

    #[test]
    fn prop_division_by_one_identity(a in raw_value()) {
        let fp = Q31::from_raw(a);
        let one = Q31::from_raw(Q31::ONE);
        let result = fp / one;

        // Should be approximately equal
        let diff = (result.raw() - fp.raw()).abs();
        assert!(diff <= 1);
    }

    #[test]
    fn prop_int_conversion_preserves_value(i in -1000i32..1000i32) {
        let fp = Q16::from_int(i);
        let back = fp.to_int();

        assert_eq!(back, i);
    }

    #[test]
    fn prop_lshift_rshift_inverse(a in raw_value(), shift in 0u32..16u32) {
        let fp = Q16::from_raw(a);
        let shifted = fp.lshift_saturate(shift);
        let back = shifted.rshift(shift);

        // Check if saturation occurred
        let max_safe = i32::MAX >> shift;
        let min_safe = i32::MIN >> shift;

        if a >= min_safe && a <= max_safe {
            assert_eq!(back.raw(), fp.raw());
        }
    }

    #[test]
    fn prop_conversion_roundtrip_q31_q16(a in raw_value()) {
        let q31 = Q31::from_raw(a);
        let q16: Q16 = q31.convert();
        let back: Q31 = q16.convert();

        // Should be close (precision loss expected)
        let diff = (back.to_float() - q31.to_float()).abs();
        assert!(diff < 0.01);
    }

    #[test]
    fn prop_saturating_add_never_wraps(a in raw_value(), b in raw_value()) {
        let fp_a = Q31::from_raw(a);
        let fp_b = Q31::from_raw(b);
        let result = fp_a + fp_b;

        // If both positive, result should be >= both inputs (or MAX)
        if a > 0 && b > 0 {
            assert!(result.raw() >= a || result.raw() == i32::MAX);
            assert!(result.raw() >= b || result.raw() == i32::MAX);
        }

        // If both negative, result should be <= both inputs (or MIN)
        if a < 0 && b < 0 {
            assert!(result.raw() <= a || result.raw() == i32::MIN);
            assert!(result.raw() <= b || result.raw() == i32::MIN);
        }
    }

    #[test]
    fn prop_mul_add_equals_separate_ops(a in raw_value(), b in raw_value(), c in raw_value()) {
        let fp_a = Q31::from_raw(a);
        let fp_b = Q31::from_raw(b);
        let fp_c = Q31::from_raw(c);

        let fused = fp_a.mul_add(fp_b, fp_c);
        let separate = fp_a + (fp_b * fp_c);

        // Should be identical
        assert_eq!(fused.raw(), separate.raw());
    }

    // --- Division (the path the absent `fuzz_division` target was meant to
    // cover): exhaustively fuzz the operands, including the divide-by-zero edge,
    // against an independent reference. ---

    #[test]
    fn prop_div_matches_reference(a in raw_value(), b in raw_value()) {
        let q = Q31::from_raw(a).saturating_div(Q31::from_raw(b));
        prop_assert_eq!(q.raw(), ref_q31_div(a, b));
        // The `/` operator must agree with the inherent method.
        prop_assert_eq!((Q31::from_raw(a) / Q31::from_raw(b)).raw(), q.raw());
    }

    #[test]
    fn prop_div_by_zero_saturates_by_sign(a in raw_value()) {
        let q = Q31::from_raw(a) / Q31::ZERO;
        let expected = if a >= 0 { Q31::MAX } else { Q31::MIN };
        prop_assert_eq!(q.raw(), expected.raw());
    }

    #[test]
    fn prop_div_by_self_is_one(a in raw_value()) {
        // x / x == ONE for any non-zero x (exactly, since dividend and divisor
        // share the value and the << FRAC_BITS makes the quotient ONE).
        prop_assume!(a != 0);
        let q = Q31::from_raw(a) / Q31::from_raw(a);
        prop_assert_eq!(q.raw(), Q31::ONE);
    }

    #[test]
    fn prop_div_int_never_panics_and_matches(a in raw_value(), d in any::<i32>()) {
        let q = Q31::from_raw(a).div_int(d);
        let expected = if d == 0 {
            if a >= 0 { Q31::MAX.raw() } else { Q31::MIN.raw() }
        } else {
            a / d
        };
        prop_assert_eq!(q.raw(), expected);
    }
}
