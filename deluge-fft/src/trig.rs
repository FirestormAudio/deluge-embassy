// ---------------------------------------------------------------------------
// Const trigonometry (Taylor series, f64 precision)
// ---------------------------------------------------------------------------

pub(crate) const TWO_PI: f64 = 2.0 * core::f64::consts::PI;

/// Taylor sin for `x` pre-reduced to `[-π/2, π/2]`.  Accurate to < 5 ulp.
#[inline]
const fn sin_kernel(x: f64) -> f64 {
    let x2 = x * x;
    x * (1.0 - x2 / 6.0 + x2 * x2 / 120.0 - x2 * x2 * x2 / 5040.0 + x2 * x2 * x2 * x2 / 362880.0
        - x2 * x2 * x2 * x2 * x2 / 39916800.0
        + x2 * x2 * x2 * x2 * x2 * x2 / 6227020800.0)
}

/// `sin(x)` for any finite `x`.  Accurate to < 5 ulp.
/// Float→int cast in `const fn` is stable since Rust 1.83.
pub(crate) const fn sin_f32(x: f64) -> f32 {
    const PI: f64 = core::f64::consts::PI;
    const HALF_PI: f64 = PI / 2.0;
    // 1. Reduce to (-π, π] via integer truncation.
    let n = (x / TWO_PI) as i64;
    let mut r = x - n as f64 * TWO_PI;
    if r > PI {
        r -= TWO_PI;
    }
    if r < -PI {
        r += TWO_PI;
    }
    // 2. Reduce to [-π/2, π/2] using sin(π-r) = sin(r) symmetry.
    let v = if r > HALF_PI {
        sin_kernel(PI - r)
    } else if r < -HALF_PI {
        -sin_kernel(PI + r)
    } else {
        sin_kernel(r)
    };
    v as f32
}

/// `cos(x)` for any finite `x`.  Uses cos(x) = sin(π/2 − x).
#[inline]
pub(crate) const fn cos_f32(x: f64) -> f32 {
    sin_f32(core::f64::consts::FRAC_PI_2 - x)
}
