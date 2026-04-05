use core::ops::{Add, Mul, Sub};

// ---------------------------------------------------------------------------
// sqrt shim — works in both no_std (bare-metal) and std (host tests)
// ---------------------------------------------------------------------------

/// `vsqrt.f32` on VFP targets, `sqrtss` on x86 SSE.
/// In no_std builds we call the `sqrtf` C symbol provided by compiler_builtins.
#[cfg(not(test))]
#[inline(always)]
fn sqrt_f32(x: f32) -> f32 {
    unsafe extern "C" {
        fn sqrtf(x: f32) -> f32;
    }
    // SAFETY: sqrtf is a pure IEEE 754 sqrt with no side-effects.
    unsafe { sqrtf(x) }
}

#[cfg(test)]
#[inline(always)]
fn sqrt_f32(x: f32) -> f32 {
    x.sqrt()
}

/// Public re-export of the sqrt shim for use within this crate.
#[inline(always)]
pub(crate) fn sqrt_f32_pub(x: f32) -> f32 {
    sqrt_f32(x)
}

// ---------------------------------------------------------------------------
// Complex<f32>
// ---------------------------------------------------------------------------

/// Complex number `(re, im)` stored as two `f32` values.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Complex {
    pub re: f32,
    pub im: f32,
}

impl Complex {
    pub const ZERO: Self = Self { re: 0.0, im: 0.0 };

    #[inline(always)]
    pub const fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }

    /// Squared magnitude (avoids a `sqrt`).
    #[inline(always)]
    pub fn norm_sq(self) -> f32 {
        self.re * self.re + self.im * self.im
    }

    /// Magnitude.
    #[inline(always)]
    pub fn abs(self) -> f32 {
        sqrt_f32(self.norm_sq())
    }
}

impl Add for Complex {
    type Output = Self;
    #[inline(always)]
    fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl Sub for Complex {
    type Output = Self;
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self {
        Self {
            re: self.re - rhs.re,
            im: self.im - rhs.im,
        }
    }
}

impl Mul for Complex {
    type Output = Self;
    #[inline(always)]
    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

impl From<f32> for Complex {
    #[inline(always)]
    fn from(re: f32) -> Self {
        Self { re, im: 0.0 }
    }
}
