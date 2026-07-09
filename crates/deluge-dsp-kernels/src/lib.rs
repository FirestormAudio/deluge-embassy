//! Portable f32 DSP kernels for the Deluge audio graph.
//!
//! Each kernel is a small `struct` with a `process` method that fills an output
//! slice a block at a time. Kernels know nothing about the graph (no node ids,
//! no `Input`). Data-parallel kernels (see `math`) use `core::simd` behind the
//! `simd` feature; serial-recurrence kernels stay scalar.
#![no_std]
#![cfg_attr(feature = "simd", feature(portable_simd))]

pub mod delay;
pub mod env;
pub mod filter;
pub mod math;
pub mod noise;
pub mod osc;
pub mod reverb;
pub mod wavetable;
mod wavetables_generated;

/// A kernel input: a control-rate constant (`K`) or an audio-rate block (`A`).
/// The graph resolves each `Input` slot into one of these before calling a kernel.
#[derive(Clone, Copy)]
pub enum In<'a> {
    K(f32),
    A(&'a [f32]),
}

impl<'a> In<'a> {
    /// Value at sample `i` (constants broadcast; blocks index).
    #[inline]
    pub fn at(&self, i: usize) -> f32 {
        match self {
            In::K(v) => *v,
            In::A(b) => b[i],
        }
    }
    /// `Some(v)` if this is a constant (lets kernels pick a cheaper path).
    #[inline]
    pub fn as_const(&self) -> Option<f32> {
        match self {
            In::K(v) => Some(*v),
            In::A(_) => None,
        }
    }
}

/// Sine of a normalized phase `p∈[0,1)` (≈ `sin(2π p)`), parabola + correction —
/// ~0.1% error, no `libm`/table. Matches the prototype's `fast_sin`.
#[inline]
pub fn fast_sin(p: f32) -> f32 {
    use core::f32::consts::PI;
    let mut x = 2.0 * PI * p;
    if x > PI {
        x -= 2.0 * PI;
    }
    const B: f32 = 4.0 / PI;
    const C: f32 = -4.0 / (PI * PI);
    let y = B * x + C * x * x.abs();
    0.225 * (y * y.abs() - y) + y
}

/// `floorf` without libm (phase is small + finite here). Matches the prototype.
#[inline]
pub(crate) fn floorf(x: f32) -> f32 {
    let t = x as i32 as f32;
    if t > x {
        t - 1.0
    } else {
        t
    }
}
