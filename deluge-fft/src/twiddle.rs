use crate::complex::Complex;

// ---------------------------------------------------------------------------
// AoS twiddle table (used by test_utils::dft only)
// ---------------------------------------------------------------------------

/// Build the full natural-order twiddle table.  Entry `k` = `W_N^k`.
#[allow(dead_code)]
const fn make_twiddles_aos<const N: usize>() -> [Complex; N] {
    let mut table = [Complex::ZERO; N];
    let mut k = 0usize;
    while k < N {
        let angle = -crate::trig::TWO_PI * k as f64 / N as f64;
        table[k] = Complex {
            re: crate::trig::cos_f32(angle),
            im: crate::trig::sin_f32(angle),
        };
        k += 1;
    }
    table
}

/// Natural-order `W_N^k` table.  Only used by `test_utils::dft`.
#[allow(dead_code)]
pub(crate) struct TwiddleTable<const N: usize>;
impl<const N: usize> TwiddleTable<N> {
    #[allow(dead_code)]
    pub(crate) const TABLE: [Complex; N] = make_twiddles_aos::<N>();
}

// ---------------------------------------------------------------------------
// Stage-sequential SoA twiddle table (used by all FFT hot paths)
// ---------------------------------------------------------------------------
//
// Layout: the table for size N has N-1 entries, split into two flat f32 arrays
// (real parts and imaginary parts), both of size N (last entry unused).
//
//   stage s (half_m = 2^s):  W_{2*half_m}^j  for j = 0..half_m
//   offset into array:        half_m - 1
//
//   [W_2^0 | W_4^0, W_4^1 | W_8^0..W_8^3 | ...]
//    off=0     1       2      3   4  5  6
//
// Twiddle at flat index `off` = half_m - 1 + j:
//   re  →  RE[off]
//   im  →  IM[off]

const fn make_twiddles_re<const N: usize>() -> [f32; N] {
    let mut table = [0f32; N];
    let mut half_m = 1usize;
    while half_m < N {
        let m = half_m * 2;
        let mut j = 0usize;
        while j < half_m {
            let angle = -crate::trig::TWO_PI * j as f64 / m as f64;
            table[half_m - 1 + j] = crate::trig::cos_f32(angle);
            j += 1;
        }
        half_m *= 2;
    }
    table
}

const fn make_twiddles_im<const N: usize>() -> [f32; N] {
    let mut table = [0f32; N];
    let mut half_m = 1usize;
    while half_m < N {
        let m = half_m * 2;
        let mut j = 0usize;
        while j < half_m {
            let angle = -crate::trig::TWO_PI * j as f64 / m as f64;
            table[half_m - 1 + j] = crate::trig::sin_f32(angle);
            j += 1;
        }
        half_m *= 2;
    }
    table
}

impl<const N: usize> TwiddleTable<N> {
    /// Real parts, stage-sequential.  Index `off` = `half_m - 1 + j`.
    pub(crate) const RE: [f32; N] = make_twiddles_re::<N>();

    /// Imaginary parts, stage-sequential.  Index `off` = `half_m - 1 + j`.
    pub(crate) const IM: [f32; N] = make_twiddles_im::<N>();

    /// Real part of twiddle at flat index `off`.
    #[inline(always)]
    pub(crate) fn re(off: usize) -> f32 {
        Self::RE[off]
    }

    /// Imaginary part of twiddle at flat index `off`.
    #[inline(always)]
    pub(crate) fn im(off: usize) -> f32 {
        Self::IM[off]
    }
}

// ---------------------------------------------------------------------------
// Interleaved (re,im) twiddle table — used by wide NEON paths
// ---------------------------------------------------------------------------
//
// Stores the same stage-sequential twiddle values as TwiddleTable, but
// interleaved: RI[2*off] = re, RI[2*off+1] = im.
//
// A single `vld2q_f32(RI[2*off..].as_ptr())` loads 4 complex twiddles as
// two float32x4_t: .0 = {wr0..wr3}, .1 = {wi0..wi3}.  This halves the
// number of load instructions compared to two separate vld1q_f32 calls.

const fn make_twiddles_ri<const N: usize>() -> [f32; 2 * N]
where
    [(); 2 * N]:,
{
    let mut table = [0f32; 2 * N];
    let mut half_m = 1usize;
    while half_m < N {
        let m = half_m * 2;
        let mut j = 0usize;
        while j < half_m {
            let angle = -crate::trig::TWO_PI * j as f64 / m as f64;
            let off = half_m - 1 + j;
            table[2 * off] = crate::trig::cos_f32(angle);
            table[2 * off + 1] = crate::trig::sin_f32(angle);
            j += 1;
        }
        half_m *= 2;
    }
    table
}

/// Interleaved `(re, im)` twiddle table.
///
/// `RI[2*off]` = real part, `RI[2*off+1]` = imaginary part,
/// where `off` = `half_m - 1 + j` (same flat index as `TwiddleTable`).
///
/// On ARM+NEON, a single `vld2q_f32(&RI[2*off])` loads 4 complex twiddles
/// into two `float32x4_t` registers with one instruction.
pub(crate) struct TwiddleTableRI<const N: usize>
where
    [(); 2 * N]:;

impl<const N: usize> TwiddleTableRI<N>
where
    [(); 2 * N]:,
{
    pub(crate) const RI: [f32; 2 * N] = make_twiddles_ri::<N>();
}
