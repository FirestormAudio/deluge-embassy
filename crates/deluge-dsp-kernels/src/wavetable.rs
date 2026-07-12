//! Single-cycle band-limited wavetable oscillator. Reads a borrowed mip pyramid
//! (`MipSet`), selects the level safe for the current pitch, cubic-Hermite
//! interpolates within it, and linearly crossfades to the adjacent level across
//! octave boundaries. `no_std`, no alloc, no FFT (tables are built offline).

use crate::{floorf, In};

const N: usize = 2048; // must equal mipgen::N
/// Number of mip levels in a pyramid; must equal `mipgen::LEVELS`. Defined
/// locally (rather than re-exported from `mipgen`, a dev-only dependency) so
/// the layout below is self-contained for non-test builds.
pub const LEVELS: usize = 11;

pub const N_MIN: usize = 32;
/// **Re-measured** (QA fix to the original Step-6 measurement, which only
/// rendered at 5 kHz — a frequency that selects mip levels 7/8, whose
/// `level_len` clamps to `N_MIN=32` under both `OS_FACTOR` 1 and 2, so it was
/// mathematically incapable of discriminating between them; see
/// `compact_saw_is_band_limited`'s doc-comment for that still-valid alias
/// check). This measurement instead compares **compact-built vs
/// full-N-built** pyramids (same base, same freq — only storage layout
/// differs) at **mid fundamentals** (~110/440/1000/2000 Hz, saw + organ
/// bases), which select mip levels ~2-7 where compact storage actually
/// truncates relative to full-N. See `compact_vs_full_n_harmonic_fidelity_mid_freqs`.
///
/// For each such (base, f0), swept every harmonic `k*f0` below sr/4 (12 kHz —
/// perceptually relevant) and measured the compact-vs-full-N magnitude delta
/// in dB. A **fine sweep** of f0 (saw base, 60-1150 Hz in 3 Hz steps, all
/// harmonics below 12 kHz) found the worst-case delta per `OS_FACTOR`:
///
///   OS_FACTOR | worst delta (dB) | at (f0, harmonic k)
///   ----------|-------------------|---------------------
///   1         | **1.78**          | f0=114 Hz, k=102 (10.4 kHz)
///   1         | 1.69               | f0=126 Hz, k=94 (11.8 kHz)
///   1         | 1.67               | f0=63 Hz, k=188 (11.8 kHz)
///   2         | **0.18**           | f0=150 Hz, k=78 (11.7 kHz)
///
/// `OS_FACTOR=1` (critical sampling: a level's grid Nyquist sits exactly at
/// its own top harmonic) measurably dulls perceptually-relevant harmonics —
/// up to 1.78 dB, well past the 1 dB fidelity tolerance — because
/// cubic/Catmull-Rom interpolation starts rolling off well before the
/// harmonic that sits exactly at the grid Nyquist (this rolloff is worst
/// right at mip-level crossfade boundaries, where the incoming level's own
/// top harmonic lands near sr/4). `OS_FACTOR=2` (2x oversampling headroom)
/// keeps that top harmonic at half the grid Nyquist, where interpolation is
/// accurate: worst case 0.18 dB, ~10x tighter than F=1. Per the brief's
/// explicit rule ("if F=1 measurably dulls perceptually-relevant harmonics
/// where F=2 doesn't, set OS_FACTOR=2" — do not weaken the gate to keep F=1),
/// **F=2 is required**. This costs storage (`COMPACT_LEN`=6208 vs F=1's 4192,
/// still 3.6x below the flat `N*LEVELS`=22528) but is the correct trade: the
/// 5 kHz-only alias check (`compact_saw_is_band_limited`,
/// `all_static_tables_band_limited`) still passes identically under either F
/// (see that test's doc-comment) — it was simply never able to see this cost.
pub const OS_FACTOR: usize = 2;

/// Samples stored for mip level `L`. Level L carries `(N/2)>>L` harmonics, needing
/// `N>>L` critical samples; oversample by `OS_FACTOR`, clamp to `[N_MIN, N]`.
/// (`N>>L` and `OS_FACTOR∈{1,2}` are powers of two, so no rounding needed.)
pub const fn level_len(level: usize) -> usize {
    let crit = N >> level; // = 2*max_harmonic(level)
    let mut m = crit.saturating_mul(OS_FACTOR);
    if m > N {
        m = N;
    }
    if m < N_MIN {
        m = N_MIN;
    }
    m
}

/// f32 offset of level `L` in a flat compact pyramid (prefix sum of level_len).
pub const fn level_offset(level: usize) -> usize {
    let mut off = 0;
    let mut l = 0;
    while l < level {
        off += level_len(l);
        l += 1;
    }
    off
}

/// Total f32 in one compact pyramid.
pub const COMPACT_LEN: usize = level_offset(LEVELS);

/// Borrowed view of a mip pyramid: `levels[0]` = fullest band, each higher level
/// halves the harmonic count. Level `L` has length `level_len(L)` (per-level
/// compaction: `N` down to `N_MIN`), with `levels[0].len() == N`.
pub struct MipSet<'a> {
    pub levels: &'a [&'a [f32]],
}

#[derive(Clone, Copy)]
pub struct WtOsc {
    phase: f32,
    last: f32,   // previous output sample (feedback)
    last2: f32,  // output two samples ago (feedback)
    feedback: f32, // self-FM depth, [-1,1]
}

impl WtOsc {
    pub fn new() -> WtOsc {
        WtOsc { phase: 0.0, last: 0.0, last2: 0.0, feedback: 0.0 }
    }

    /// Self-FM depth, clamped to [-1, 1] (mirrors mono `Osc::set_feedback` /
    /// `PolyOsc::set_feedback`).
    pub fn set_feedback(&mut self, f: f32) {
        self.feedback = f.clamp(-1.0, 1.0);
    }

    pub fn process(&mut self, mips: MipSet, freq: In, pmod: In, dt: f32, out: &mut [f32]) {
        debug_assert!(!mips.levels.is_empty());
        debug_assert_eq!(mips.levels[0].len(), N);
        // Const-freq/pmod fast path: mip_select (which calls log2f) is a
        // pure function of dtp and the level count, so when both freq and
        // pmod are block-constant it's hoisted out of the per-sample loop
        // (called once per block instead of once per sample). Phase still
        // advances iteratively, so this is bit-exact vs the general path
        // fed the same values as a constant-valued audio-rate buffer (see
        // `const_freq_hoist_is_bit_exact`).
        //
        // The feedback term below changes the read phase every sample (it
        // depends on `self.last`/`self.last2`, which update per-sample), so
        // it breaks this fast path's per-block-constant-phase assumption.
        // Only take the fast path when `feedback == 0.0`, so every
        // feedback=0 render still takes this exact, unmodified path
        // (bit-identical to before feedback existed).
        if self.feedback == 0.0 {
        if let (Some(f), Some(pm)) = (freq.as_const(), pmod.as_const()) {
            let dtp = f * dt;
            let (lo, hi, frac) = mip_select(dtp, mips.levels.len());
            let (lo_s, hi_s) = (mips.levels[lo], mips.levels[hi]);

            // SIMD (f32x8) sub-branch: 8 output samples per chunk via a
            // closed-form phase (phase0 + pm + k*dtp for k in 0..8) and a
            // vectorized gather+Catmull-Rom read; scalar-tail the remainder.
            // This is tolerance-equivalent to the scalar path below (float
            // reassociation + closed-form phase), NOT bit-exact — see
            // `simd_matches_scalar_within_tol`.
            //
            // Each chunk's phase is derived from a single call-start anchor
            // (`phase0`) via one multiply-add (`phase0 + c*chunk_span`),
            // *not* by repeatedly `+=`-ing onto a running accumulator once
            // per chunk: with many chunks in one call (and `self.phase`
            // persisting across many calls over a note's lifetime),
            // compounding rounding error once per chunk measurably drifts
            // from the scalar path's once-per-*sample* accumulation, which
            // — right at a mip table's per-cycle wrap boundary (steepest
            // local slope in a band-limited edge) — can flip which table
            // cell the tail end of a boundary-adjacent chunk reads. Deriving
            // every chunk fresh from `phase0` bounds that error to a single
            // multiply-add's rounding regardless of block length. `self.phase`
            // is set once at the end (also a single multiply-add), then the
            // scalar tail continues from it exactly like the scalar path
            // would. The `#[cfg(not(feature = "simd"))]` twin below is the
            // unmodified, bit-exact Task-1 scalar path (default build).
            #[cfg(feature = "simd")]
            {
                use core::simd::{f32x8, Simd};
                let lane: f32x8 = Simd::from_array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);
                let step = lane * f32x8::splat(dtp);
                let pm_v = f32x8::splat(pm);
                let frac_v = f32x8::splat(frac.clamp(0.0, 1.0));
                let phase0 = self.phase;
                let chunk_span = 8.0 * dtp;
                let n_chunks = out.len() / 8;
                for c in 0..n_chunks {
                    let chunk_phase = phase0 + (c as f32) * chunk_span;
                    let raw = f32x8::splat(chunk_phase) + step + pm_v;
                    let ph = raw - simd8::floor(raw);
                    let out_v = simd8::sample_at_level(lo_s, hi_s, frac_v, ph);
                    out_v.copy_to_slice(&mut out[c * 8..c * 8 + 8]);
                }
                self.phase = phase0 + (n_chunks as f32) * chunk_span;
                self.phase -= floorf(self.phase);
                for s in out[(n_chunks * 8)..].iter_mut() {
                    let mut ph = self.phase + pm;
                    ph -= floorf(ph);
                    *s = sample_at_level(lo_s, hi_s, frac, ph);
                    self.phase += dtp;
                    self.phase -= floorf(self.phase);
                }
                return;
            }

            #[cfg(not(feature = "simd"))]
            {
                for s in out.iter_mut() {
                    let mut ph = self.phase + pm;
                    ph -= floorf(ph);
                    *s = sample_at_level(lo_s, hi_s, frac, ph);
                    self.phase += dtp;
                    self.phase -= floorf(self.phase);
                }
                return;
            }
        }
        }
        for (i, s) in out.iter_mut().enumerate() {
            let dtp = freq.at(i) * dt; // cycles/sample
            let fb = self.feedback * 0.5 * (self.last + self.last2);
            let mut ph = self.phase + pmod.at(i) + fb;
            ph -= floorf(ph);

            let y = sample_one(&mips, ph, dtp);
            self.last2 = self.last;
            self.last = y;
            *s = y;

            self.phase += dtp;
            self.phase -= floorf(self.phase);
        }
    }

    /// Multi-frame morphing read: Catmull-Rom cubic-blends across 4 clamped
    /// frames (`f-1, f, f+1, f+2`) of a flat, contiguous stack of compact
    /// pyramids (`region`, `frames * COMPACT_LEN` long, each frame laid out
    /// like `compact_levels` expects) selected by `position` (normalized
    /// `[0,1]` across `frames`), while each frame's own sample is produced by
    /// the same band-limited single-cycle read (`sample_one`) `process`
    /// uses. Out-of-range taps clamp to the first/last frame, so the blend
    /// degrades gracefully near the ends (and is exact — `catmull_rom`
    /// passes through its control points — at `position` `0`/`1` and every
    /// interior frame boundary). `region.len() < frames * COMPACT_LEN` (or
    /// `frames == 0`) silently produces silence (out left as-is by
    /// caller/zeroed) rather than panicking, since callers may pass a
    /// not-yet-fully-uploaded region.
    pub fn process_morph(
        &mut self, region: &[f32], frames: usize,
        freq: In, pmod: In, position: In, dt: f32, out: &mut [f32],
    ) {
        // Region must be `frames * COMPACT_LEN`; each frame is a compact pyramid.
        if frames == 0 || region.len() < frames * COMPACT_LEN { return; }
        let last = frames - 1;
        // Const-freq/pmod/position fast path: when all three are
        // block-constant, the frame bracket (fm1/f0c/f1/f2/ffrac) and the
        // mip-select (lo/hi/frac — same for every frame, since nlev==LEVELS
        // is shared) are pure functions of those constants, so both are
        // hoisted out of the per-sample loop (computed once per block).
        // Phase still advances iteratively, so this is bit-exact vs the
        // general path fed the same values as constant-valued audio-rate
        // buffers (see `const_freq_hoist_process_morph_is_bit_exact`).
        //
        // Same feedback-breaks-the-per-block-constant-phase-assumption
        // reasoning as `process`'s fast path — only take it when
        // `feedback == 0.0`.
        if self.feedback == 0.0 {
        if let (Some(f), Some(pm), Some(pos)) = (freq.as_const(), pmod.as_const(), position.as_const()) {
            let dtp = f * dt;
            let fpos = pos.clamp(0.0, 1.0) * last as f32;
            let f0 = fpos as usize;
            let f0 = if f0 > last { last } else { f0 };
            let ffrac = (fpos - f0 as f32).clamp(0.0, 1.0);
            let fm1 = if f0 == 0 { 0 } else { f0 - 1 };
            let f0c = if f0 > last { last } else { f0 };
            let f1 = if f0c + 1 > last { last } else { f0c + 1 };
            let f2 = if f0c + 2 > last { last } else { f0c + 2 };
            let mm1 = compact_levels(&region[fm1 * COMPACT_LEN..(fm1 + 1) * COMPACT_LEN]);
            let m0 = compact_levels(&region[f0c * COMPACT_LEN..(f0c + 1) * COMPACT_LEN]);
            let m1 = compact_levels(&region[f1 * COMPACT_LEN..(f1 + 1) * COMPACT_LEN]);
            let m2 = compact_levels(&region[f2 * COMPACT_LEN..(f2 + 1) * COMPACT_LEN]);
            let (lo, hi, frac) = mip_select(dtp, LEVELS);
            let (mm1_lo, mm1_hi) = (mm1[lo], mm1[hi]);
            let (m0_lo, m0_hi) = (m0[lo], m0[hi]);
            let (m1_lo, m1_hi) = (m1[lo], m1[hi]);
            let (m2_lo, m2_hi) = (m2[lo], m2[hi]);

            // SIMD (f32x8) sub-branch: same structure as `process`'s
            // (including the call-start-anchored, non-compounding chunk
            // phase — see that branch's comment), but 4 frames' worth of
            // gather+Catmull-Rom per lane, then the frame-axis Catmull-Rom
            // blend by the (already-clamped) constant `ffrac`.
            // Tolerance-equivalent to the scalar path below, not bit-exact —
            // see `simd_matches_scalar_within_tol_morph`.
            #[cfg(feature = "simd")]
            {
                use core::simd::{f32x8, Simd};
                let lane: f32x8 = Simd::from_array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);
                let step = lane * f32x8::splat(dtp);
                let pm_v = f32x8::splat(pm);
                let frac_v = f32x8::splat(frac.clamp(0.0, 1.0));
                let ffrac_v = f32x8::splat(ffrac);
                let phase0 = self.phase;
                let chunk_span = 8.0 * dtp;
                let n_chunks = out.len() / 8;
                for c in 0..n_chunks {
                    let chunk_phase = phase0 + (c as f32) * chunk_span;
                    let raw = f32x8::splat(chunk_phase) + step + pm_v;
                    let ph = raw - simd8::floor(raw);
                    let ym1 = simd8::sample_at_level(mm1_lo, mm1_hi, frac_v, ph);
                    let y0 = simd8::sample_at_level(m0_lo, m0_hi, frac_v, ph);
                    let y1 = simd8::sample_at_level(m1_lo, m1_hi, frac_v, ph);
                    let y2 = simd8::sample_at_level(m2_lo, m2_hi, frac_v, ph);
                    let out_v = simd8::catmull_rom(ym1, y0, y1, y2, ffrac_v);
                    out_v.copy_to_slice(&mut out[c * 8..c * 8 + 8]);
                }
                self.phase = phase0 + (n_chunks as f32) * chunk_span;
                self.phase -= floorf(self.phase);
                for s in out[(n_chunks * 8)..].iter_mut() {
                    let mut ph = self.phase + pm;
                    ph -= floorf(ph);
                    let ym1 = sample_at_level(mm1_lo, mm1_hi, frac, ph);
                    let y0 = sample_at_level(m0_lo, m0_hi, frac, ph);
                    let y1 = sample_at_level(m1_lo, m1_hi, frac, ph);
                    let y2 = sample_at_level(m2_lo, m2_hi, frac, ph);
                    *s = catmull_rom(ym1, y0, y1, y2, ffrac);
                    self.phase += dtp;
                    self.phase -= floorf(self.phase);
                }
                return;
            }

            #[cfg(not(feature = "simd"))]
            {
                for s in out.iter_mut() {
                    let mut ph = self.phase + pm;
                    ph -= floorf(ph);
                    let ym1 = sample_at_level(mm1_lo, mm1_hi, frac, ph);
                    let y0 = sample_at_level(m0_lo, m0_hi, frac, ph);
                    let y1 = sample_at_level(m1_lo, m1_hi, frac, ph);
                    let y2 = sample_at_level(m2_lo, m2_hi, frac, ph);
                    *s = catmull_rom(ym1, y0, y1, y2, ffrac);
                    self.phase += dtp;
                    self.phase -= floorf(self.phase);
                }
                return;
            }
        }
        }
        for (i, s) in out.iter_mut().enumerate() {
            let dtp = freq.at(i) * dt;
            let fb = self.feedback * 0.5 * (self.last + self.last2);
            let mut ph = self.phase + pmod.at(i) + fb;
            ph -= floorf(ph);
            // Frame bracket from position in [0,1].
            let fpos = position.at(i).clamp(0.0, 1.0) * last as f32;
            let f0 = fpos as usize;
            let f0 = if f0 > last { last } else { f0 };
            let ffrac = fpos - f0 as f32;
            let fm1 = if f0 == 0 { 0 } else { f0 - 1 };
            let f0c = if f0 > last { last } else { f0 };
            let f1 = if f0c + 1 > last { last } else { f0c + 1 };
            let f2 = if f0c + 2 > last { last } else { f0c + 2 };
            let mm1 = compact_levels(&region[fm1 * COMPACT_LEN..(fm1 + 1) * COMPACT_LEN]);
            let m0 = compact_levels(&region[f0c * COMPACT_LEN..(f0c + 1) * COMPACT_LEN]);
            let m1 = compact_levels(&region[f1 * COMPACT_LEN..(f1 + 1) * COMPACT_LEN]);
            let m2 = compact_levels(&region[f2 * COMPACT_LEN..(f2 + 1) * COMPACT_LEN]);
            let ym1 = sample_one(&MipSet { levels: &mm1 }, ph, dtp);
            let y0 = sample_one(&MipSet { levels: &m0 }, ph, dtp);
            let y1 = sample_one(&MipSet { levels: &m1 }, ph, dtp);
            let y2 = sample_one(&MipSet { levels: &m2 }, ph, dtp);
            let y = catmull_rom(ym1, y0, y1, y2, ffrac.clamp(0.0, 1.0));
            self.last2 = self.last;
            self.last = y;
            *s = y;
            self.phase += dtp; self.phase -= floorf(self.phase);
        }
    }
}

impl Default for WtOsc {
    fn default() -> Self {
        WtOsc::new()
    }
}

/// Identifies one of the named `&'static` mip pyramids baked into
/// `wavetables_generated::TABLES` (see that module's `TABLES` order for the
/// id assignment: 0=Saw 1=Square 2=Sine 3=Tri 4=Organ 5=Formant; 6=HarmonicSweep
/// and 7=FormantMorph are *2D* (multi-frame) morph banks — same lookup, but
/// `static_table_flat`'s returned region is `FRAMES*COMPACT_LEN` long instead
/// of `COMPACT_LEN`, so callers deriving `frames = region.len()/COMPACT_LEN`
/// (see `deluge_audio_graph::node`'s `Kind::Wavetable` arm) route them through
/// `process_morph` instead of `process`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TableId(pub u16);

/// Resolve a named static table to its flat, compact (`COMPACT_LEN`-long)
/// pyramid region. `None` if `id` is out of range.
///
/// Returns the flat region rather than a `MipSet` because `MipSet` borrows a
/// `&'a [&'a [f32]]` level-slice array: an array assembled here (via
/// `level_offset`/`level_len`) would live on this fn's stack frame and could
/// not be returned by reference. Callers assemble the `[&[f32]; LEVELS]` from
/// this flat slice via the same `level_offset`/`level_len` layout the pooled
/// (dynamically-uploaded) path uses (see `deluge_audio_graph::node`'s
/// `Kind::Wavetable` arm), unifying static + pooled assembly at the render
/// call site.
pub fn static_table_flat(id: TableId) -> Option<&'static [f32]> {
    let idx = id.0 as usize;
    let tables = &crate::wavetables_generated::TABLES;
    if idx >= tables.len() {
        return None;
    }
    Some(tables[idx])
}

/// Single-cycle band-limited read: mip-select by pitch (`dtp` = cycles/sample)
/// at phase `ph`, cubic-Hermite interpolate within the selected level, and
/// linearly crossfade to the adjacent level across octave boundaries. This is
/// the exact per-sample inner read `process` uses; `process_morph` calls it
/// once per morph frame and crossfades the results by `position`.
fn sample_one(mips: &MipSet, ph: f32, dtp: f32) -> f32 {
    let (lo, hi, frac) = mip_select(dtp, mips.levels.len());
    sample_at_level(mips.levels[lo], mips.levels[hi], frac, ph)
}

/// Mip-level selection: as `dtp` (cycles/sample) doubles (one octave up),
/// drop one level of harmonics. Level 0 is safe while its top harmonic
/// (N/2) stays below Nyquist: `dtp*(N/2) < 0.5` → `dtp < 1/N`. Each octave
/// above adds 1. Returns `(lo, hi, frac)`: the two levels to crossfade
/// between and the fractional crossfade weight. Pure function of `dtp` and
/// the pyramid's level count — safe to hoist out of a per-sample loop when
/// `dtp` is constant across a block.
fn mip_select(dtp: f32, nlev: usize) -> (usize, usize, f32) {
    let flevel = if dtp <= 0.0 {
        0.0
    } else {
        // log2(dtp * N) clamped ≥ 0
        let x = dtp * N as f32;
        let l = libm::log2f(x);
        if l < 0.0 { 0.0 } else { l }
    };
    let lo = flevel as usize;
    let lo = if lo >= nlev { nlev - 1 } else { lo };
    let hi = if lo + 1 >= nlev { nlev - 1 } else { lo + 1 };
    let frac = flevel - lo as f32;
    (lo, hi, frac)
}

/// Cubic-Hermite sample within the selected mip level (`lo`), crossfaded
/// toward the adjacent level (`hi`) by `frac` (the fractional mip level from
/// `mip_select`). Shared by `sample_one` and the hoisted const-freq paths in
/// `process`/`process_morph`, which call `mip_select` once per block instead
/// of once per sample.
fn sample_at_level(lo: &[f32], hi: &[f32], frac: f32, ph: f32) -> f32 {
    let a = interp_cubic(lo, ph);
    let b = interp_cubic(hi, ph);
    a + (b - a) * frac.clamp(0.0, 1.0)
}

/// Assembles a per-frame level view `[&[f32]; LEVELS]` from a flat compact
/// pyramid region (`COMPACT_LEN` long), via the `level_offset`/`level_len`
/// layout. Shared by the static-table, pooled, and morph (`process_morph`)
/// read paths.
pub(crate) fn compact_levels(region: &[f32]) -> [&[f32]; LEVELS] {
    core::array::from_fn(|l| &region[level_offset(l)..level_offset(l) + level_len(l)])
}

/// Catmull-Rom cubic through 4 control points `(y0,y1,y2,y3)`, interpolating
/// between `y1` (at `t=0`) and `y2` (at `t=1`) using `y0`/`y3` as the outer
/// tangents. Shared by `interp_cubic` (phase axis, within one frame) and
/// `process_morph` (frame axis, across morph frames).
fn catmull_rom(y0: f32, y1: f32, y2: f32, y3: f32, t: f32) -> f32 {
    let a = -0.5 * y0 + 1.5 * y1 - 1.5 * y2 + 0.5 * y3;
    let b = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
    let c = -0.5 * y0 + 0.5 * y2;
    ((a * t + b) * t + c) * t + y1
}

/// 4-point Catmull-Rom at fractional phase `ph` in [0,1) over a length-N table.
fn interp_cubic(table: &[f32], ph: f32) -> f32 {
    let n = table.len();
    let x = ph * n as f32;
    let i1 = x as usize % n;
    let frac = x - floorf(x);
    let i0 = (i1 + n - 1) % n;
    let i2 = (i1 + 1) % n;
    let i3 = (i1 + 2) % n;
    let (y0, y1, y2, y3) = (table[i0], table[i1], table[i2], table[i3]);
    catmull_rom(y0, y1, y2, y3, frac)
}

/// SIMD (`f32x8`) analogs of `floorf`/`catmull_rom`/`interp_cubic`/
/// `sample_at_level`, used by the `#[cfg(feature = "simd")]` const-freq
/// sub-branches in `process`/`process_morph`. Same coefficient arithmetic and
/// the same truncate-then-correct `floor` trick as the scalar versions —
/// differences vs scalar output are float-reassociation/closed-form-phase
/// noise, not algorithmic (see `simd_matches_scalar_within_tol` and
/// `simd_matches_scalar_within_tol_morph`).
#[cfg(feature = "simd")]
mod simd8 {
    use core::simd::{
        cmp::SimdPartialOrd,
        f32x8,
        num::{SimdFloat, SimdInt},
        Select, Simd,
    };

    /// `floorf`, vectorized: truncate toward zero (`as i32`), then subtract 1
    /// where truncation rounded up (`t > x`). Same form as `super::floorf`.
    #[inline]
    pub(super) fn floor(x: f32x8) -> f32x8 {
        let t: f32x8 = x.cast::<i32>().cast::<f32>();
        let gt = t.simd_gt(x);
        gt.select(t - f32x8::splat(1.0), t)
    }

    /// `catmull_rom`, vectorized (identical coefficient form).
    #[inline]
    pub(super) fn catmull_rom(y0: f32x8, y1: f32x8, y2: f32x8, y3: f32x8, t: f32x8) -> f32x8 {
        let a = f32x8::splat(-0.5) * y0 + f32x8::splat(1.5) * y1 - f32x8::splat(1.5) * y2
            + f32x8::splat(0.5) * y3;
        let b = y0 - f32x8::splat(2.5) * y1 + f32x8::splat(2.0) * y2 - f32x8::splat(0.5) * y3;
        let c = f32x8::splat(-0.5) * y0 + f32x8::splat(0.5) * y2;
        ((a * t + b) * t + c) * t + y1
    }

    /// `interp_cubic`, vectorized: 8 independent phase lanes each gather
    /// their own 4 (wrapped) Catmull-Rom taps out of `table`.
    #[inline]
    pub(super) fn interp_cubic(table: &[f32], ph: f32x8) -> f32x8 {
        let n = table.len();
        let x = ph * f32x8::splat(n as f32);
        let xf = floor(x); // x >= 0 (ph in [0,1)), so floor == trunc here.
        let frac = x - xf;
        let n_v: Simd<usize, 8> = Simd::splat(n);
        let one: Simd<usize, 8> = Simd::splat(1);
        let two: Simd<usize, 8> = Simd::splat(2);
        let i1: Simd<usize, 8> = xf.cast::<usize>() % n_v;
        let i0 = (i1 + n_v - one) % n_v;
        let i2 = (i1 + one) % n_v;
        let i3 = (i1 + two) % n_v;
        let zero = f32x8::splat(0.0);
        let y0 = f32x8::gather_or(table, i0, zero);
        let y1 = f32x8::gather_or(table, i1, zero);
        let y2 = f32x8::gather_or(table, i2, zero);
        let y3 = f32x8::gather_or(table, i3, zero);
        catmull_rom(y0, y1, y2, y3, frac)
    }

    /// `sample_at_level`, vectorized.
    #[inline]
    pub(super) fn sample_at_level(lo: &[f32], hi: &[f32], frac: f32x8, ph: f32x8) -> f32x8 {
        let a = interp_cubic(lo, ph);
        let b = interp_cubic(hi, ph);
        a + (b - a) * frac
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::In;

    fn saw_mips() -> [[f32; mipgen::N]; mipgen::LEVELS] {
        let mut base = [0.0f32; mipgen::N];
        for (i, s) in base.iter_mut().enumerate() {
            *s = 2.0 * (i as f32 / mipgen::N as f32) - 1.0;
        }
        let mut out = [[0.0f32; mipgen::N]; mipgen::LEVELS];
        mipgen::build_all(&base, &mut out);
        out
    }

    fn mipset(levels: &[[f32; mipgen::N]; mipgen::LEVELS]) -> [&[f32]; mipgen::LEVELS] {
        core::array::from_fn(|i| &levels[i][..])
    }

    #[test]
    fn static_saw_table_is_band_limited() {
        let sr = 48_000.0f32;
        let region = static_table_flat(TableId(0)).expect("saw table");
        let levels = compact_levels(region);
        let mut osc = WtOsc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(MipSet { levels: &levels }, In::K(5_000.0), In::K(0.0), 1.0 / sr, &mut buf);
        let wa = deluge_dsp_test::spectrum::analyze_buf(sr, &buf)
            .worst_alias_db(5_000.0, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));
        assert!(wa < -20.0, "static saw worst_alias {wa} dB");
    }

    #[test]
    fn wavetable_saw_is_band_limited_high() {
        let sr = 48_000.0f32;
        let m = saw_mips();
        let refs = mipset(&m);
        for &f0 in &[2_000.0f32, 5_000.0] {
            let mut osc = WtOsc::new();
            let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
            osc.process(MipSet { levels: &refs }, In::K(f0), In::K(0.0), 1.0 / sr, &mut buf);
            let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
            let wa = spec.worst_alias_db(f0, 3.0 * spec.bin_hz);
            // Measured: 2kHz -41.1 dB, 5kHz -25.6 dB (hardest case, near an
            // octave-boundary crossfade). Gate at -23 dB, ~2.6 dB below the
            // measured floor (Osc-1's margin convention).
            assert!(wa < -23.0, "wt saw f0={f0}: worst_alias {wa} dB");
        }
    }

    #[test]
    fn wavetable_beats_naive_single_table() {
        // A naive single-table (no mip-select) reference at high freq aliases;
        // the mip'd version must be meaningfully better.
        let sr = 48_000.0f32;
        let f0 = 5_000.0f32;
        let m = saw_mips();
        let refs = mipset(&m);

        let mut osc = WtOsc::new();
        let mut bl = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(MipSet { levels: &refs }, In::K(f0), In::K(0.0), 1.0 / sr, &mut bl);
        let bl_wa = deluge_dsp_test::spectrum::analyze_buf(sr, &bl).worst_alias_db(f0, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));

        // Naive: always read level 0 (full band) with nearest-neighbor
        // (zero-order hold) sampling, no mip-select.
        let mut naive = [0.0f32; deluge_dsp_test::FFT_N];
        let mut ph = 0.0f32;
        let dtp = f0 / sr;
        for s in naive.iter_mut() {
            let x = ph * mipgen::N as f32;
            let i0 = x as usize % mipgen::N;
            *s = m[0][i0];
            ph += dtp;
            ph -= floorf(ph);
        }
        let nv_wa = deluge_dsp_test::spectrum::analyze_buf(sr, &naive).worst_alias_db(f0, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));

        // Measured: mip'd -25.6 dB vs naive -13.8 dB at 5 kHz -> +11.75 dB
        // improvement. Gate at >9 dB, ~2.75 dB below the measured margin
        // (Osc-1's convention) rather than the originally-planned >10, which
        // left only ~1.75 dB of headroom above the measured value.
        assert!(bl_wa < nv_wa - 9.0, "mip'd {bl_wa} should beat naive {nv_wa} by >9 dB");
    }

    #[test]
    fn wavetable_low_freq_has_harmonics() {
        let sr = 48_000.0f32;
        let f0 = 110.0f32;
        let m = saw_mips();
        let refs = mipset(&m);
        let mut osc = WtOsc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(MipSet { levels: &refs }, In::K(f0), In::K(0.0), 1.0 / sr, &mut buf);
        let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
        // Saw has 1st..Nth harmonics; check 2nd & 3rd are present.
        assert!(spec.level_at(2.0 * f0) > 0.05 * spec.level_at(f0));
        assert!(spec.level_at(3.0 * f0) > 0.02 * spec.level_at(f0));
    }

    #[test]
    fn wavetable_mip_boundary_is_continuous() {
        // Mip-level boundaries sit at freq = (sr/N) * 2^L = 23.4375 * 2^L Hz,
        // e.g. L5 = 750 Hz. Sweep 700->800 Hz so the fractional mip level
        // (flevel) passes through the integer 5.0 at 750 Hz mid-sweep, which
        // is exactly where `lo`/`hi` swap (4->5 and 5->6). This is the case
        // an off-by-one in the mip-select math would break; a sweep that
        // stays inside one crossfade band (e.g. the old 1000-1100 Hz range,
        // flevel ~5.42-5.55) never exercises the level swap at all. Assert
        // no discontinuity as the block-rendered RMS varies across freq.
        let sr = 48_000.0f32;
        let m = saw_mips();
        let refs = mipset(&m);
        let mut prev_rms: Option<f32> = None;
        let mut f = 700.0f32;
        while f < 800.0 {
            let mut osc = WtOsc::new();
            let mut buf = [0.0f32; 512];
            osc.process(MipSet { levels: &refs }, In::K(f), In::K(0.0), 1.0 / sr, &mut buf);
            let rms = (buf.iter().map(|s| s * s).sum::<f32>() / buf.len() as f32).sqrt();
            if let Some(p) = prev_rms {
                // 0.05 abs-RMS-delta threshold, same as before: tight enough
                // to catch a hard level-swap jump but loose enough to pass
                // the smooth crossfade across this real boundary at 750 Hz.
                assert!((rms - p).abs() < 0.05, "rms jump at f={f}: {p}->{rms}");
            }
            prev_rms = Some(rms);
            f += 5.0;
        }
    }

    #[test]
    fn all_static_tables_band_limited() {
        let sr = 48_000.0f32;
        // Measured worst_alias_db at 5 kHz for each static table (id: name):
        //   0 Saw:     -25.58 dB
        //   1 Square:  -25.58 dB
        //   2 Sine:    -41.12 dB (near-alias-free, as expected of a pure tone)
        //   3 Tri:     -39.56 dB
        //   4 Organ:   -23.64 dB (worst of the six — additive mix pushes energy
        //              close to Nyquist at some harmonics)
        //   5 Formant: -41.12 dB
        // Gate set ~2.6 dB below the worst measured floor (Organ, -23.64 dB),
        // matching the margin convention used by `wavetable_saw_is_band_limited_high`.
        for id in 0u16..6 {
            let region = static_table_flat(TableId(id)).expect("table");
            let levels = compact_levels(region);
            let mut osc = WtOsc::new();
            let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
            osc.process(MipSet { levels: &levels }, In::K(5_000.0), In::K(0.0), 1.0 / sr, &mut buf);
            let wa = deluge_dsp_test::spectrum::analyze_buf(sr, &buf)
                .worst_alias_db(5_000.0, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));
            assert!(wa < -21.0, "table {id}: worst_alias {wa} dB");
        }
    }

    #[test]
    fn layout_invariants() {
        assert_eq!(level_len(0), N);
        for l in 0..LEVELS { assert!(level_len(l).is_power_of_two() && level_len(l) >= N_MIN); }
        for l in 1..LEVELS { assert!(level_len(l) <= level_len(l - 1)); }
        let mut sum = 0;
        for l in 0..LEVELS { assert_eq!(level_offset(l), sum); sum += level_len(l); }
        assert_eq!(COMPACT_LEN, sum);
        assert_eq!(COMPACT_LEN, level_offset(LEVELS));
    }

    fn full_levels(region: &[f32]) -> [&[f32]; LEVELS] {
        core::array::from_fn(|l| &region[l * N..(l + 1) * N])
    }

    #[test]
    fn compact_saw_is_band_limited() {
        let mut base = [0.0f32; N];
        for (i, s) in base.iter_mut().enumerate() { *s = 2.0 * (i as f32 / N as f32) - 1.0; }
        let mut region = [0.0f32; COMPACT_LEN];
        mipgen::build_pyramid_flat_compact(&base, &mut region);
        let levels = compact_levels(&region);
        let sr = 48_000.0f32;
        let mut osc = WtOsc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(MipSet { levels: &levels }, In::K(5_000.0), In::K(0.0), 1.0 / sr, &mut buf);
        let wa = deluge_dsp_test::spectrum::analyze_buf(sr, &buf)
            .worst_alias_db(5_000.0, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));
        // Measured floor -23.73 dB (organ, see `compact_vs_full_n_harmonic_fidelity_mid_freqs`'s
        // doc-comment and OS_FACTOR's doc-comment); gate set 2.73 dB below that.
        assert!(wa < -21.0, "compact saw worst_alias {wa} dB");
    }

    fn organ_base() -> [f32; N] {
        let mut b = [0.0f32; N];
        for (i, s) in b.iter_mut().enumerate() {
            let t = i as f32 / N as f32;
            *s = libm::sinf(core::f32::consts::TAU * t)
                + 0.5 * libm::sinf(core::f32::consts::TAU * 3.0 * t)
                + 0.25 * libm::sinf(core::f32::consts::TAU * 5.0 * t);
        }
        b
    }

    fn saw_base() -> [f32; N] {
        let mut b = [0.0f32; N];
        for (i, s) in b.iter_mut().enumerate() { *s = 2.0 * (i as f32 / N as f32) - 1.0; }
        b
    }

    /// Renders `base` through both a full-N pyramid (`build_pyramid_flat`,
    /// every level stored at N=2048 samples) and a compact pyramid
    /// (`build_pyramid_flat_compact`, per-level lengths from `level_len`,
    /// governed by `OS_FACTOR`) at `f0`, and returns (compact_spectrum,
    /// full_spectrum).
    fn render_compact_and_full(
        base: &[f32; N],
        f0: f32,
        sr: f32,
    ) -> (deluge_dsp_test::spectrum::Spectrum, deluge_dsp_test::spectrum::Spectrum) {
        let mut region_full = [0.0f32; N * LEVELS];
        mipgen::build_pyramid_flat(base, &mut region_full);
        let full = full_levels(&region_full);

        let mut region_compact = [0.0f32; COMPACT_LEN];
        mipgen::build_pyramid_flat_compact(base, &mut region_compact);
        let compact = compact_levels(&region_compact);

        let mut buf_full = [0.0f32; deluge_dsp_test::FFT_N];
        WtOsc::new().process(MipSet { levels: &full }, In::K(f0), In::K(0.0), 1.0 / sr, &mut buf_full);
        let spec_full = deluge_dsp_test::spectrum::analyze_buf(sr, &buf_full);

        let mut buf_compact = [0.0f32; deluge_dsp_test::FFT_N];
        WtOsc::new().process(MipSet { levels: &compact }, In::K(f0), In::K(0.0), 1.0 / sr, &mut buf_compact);
        let spec_compact = deluge_dsp_test::spectrum::analyze_buf(sr, &buf_compact);

        (spec_compact, spec_full)
    }

    /// Compact-vs-full-N harmonic-fidelity equivalence gate (the acceptance
    /// test required by the wavetable-mip-compaction spec, Task 1 QA fix).
    ///
    /// Renders saw and organ bases at mid fundamentals (~110/440/1000/2000 Hz)
    /// — chosen because they select mip levels ~2-7, where per-level storage
    /// length actually differs between compact (`level_len(L)`, governed by
    /// `OS_FACTOR`) and full-N (always N=2048) storage. (5 kHz, used by the
    /// original `compact_saw_is_band_limited`/`all_static_tables_band_limited`
    /// tests, selects levels 7/8, whose `level_len` is clamped to `N_MIN=32`
    /// under both OS_FACTOR=1 and 2 — mathematically incapable of
    /// discriminating compact-storage quality, hence this separate test.)
    ///
    /// For each harmonic `k*f0` below the fundamental's Nyquist, compares the
    /// compact-built render's magnitude against the full-N-built render's
    /// magnitude (same base, same freq — the only difference is storage
    /// layout). Harmonics below ~sr/4 (12 kHz @ 48 kHz sr) are perceptually
    /// relevant and must match within 1 dB — this is what "compaction doesn't
    /// change the audible sound" means operationally. Harmonics approaching
    /// Nyquist are allowed to attenuate under compact storage (the known,
    /// inaudible cost of storing a level's top harmonic at/near its grid
    /// Nyquist, where cubic/Catmull-Rom interpolation rolls off) and are not
    /// gated.
    ///
    /// This is the discriminator that decided `OS_FACTOR` (see its
    /// doc-comment for the full measurement): under `OS_FACTOR=1` a wider
    /// sweep than the four freqs below finds deltas up to 1.78 dB
    /// (exceeding this test's 1 dB tolerance) at some mid frequencies, which
    /// is why `OS_FACTOR=2` is set. At `OS_FACTOR=2` this test passes with
    /// comfortable margin (worst observed delta ~0.18 dB across a fine
    /// sweep). If this test ever fails again, do not loosen the 1 dB/12 kHz
    /// thresholds to make it pass — that would silently regress storage
    /// quality; re-run the `OS_FACTOR` measurement instead.
    #[test]
    fn compact_vs_full_n_harmonic_fidelity_mid_freqs() {
        let sr = 48_000.0f32;
        let bases: [(&str, [f32; N]); 2] = [("saw", saw_base()), ("organ", organ_base())];
        let freqs = [110.0f32, 440.0, 1_000.0, 2_000.0];

        for (name, base) in &bases {
            for &f0 in &freqs {
                let (spec_compact, spec_full) = render_compact_and_full(base, f0, sr);
                let fund_full = spec_full.level_at(f0).max(1e-12);
                let nyquist = sr / 2.0;
                let mut k = 1usize;
                loop {
                    let hz = k as f32 * f0;
                    if hz >= nyquist {
                        break;
                    }
                    let full_lin = spec_full.level_at(hz);
                    // Skip harmonics with negligible energy in the reference
                    // (full-N) render — their dB delta is noise-floor
                    // comparison, not a meaningful fidelity signal.
                    if full_lin >= 1e-3 * fund_full {
                        let compact_lin = spec_compact.level_at(hz).max(1e-12);
                        let delta_db = 20.0 * (compact_lin / full_lin.max(1e-12)).log10();
                        if hz < 12_000.0 {
                            assert!(
                                delta_db.abs() < 1.0,
                                "{name} f0={f0}: harmonic k={k} ({hz} Hz) compact vs full-N \
                                 delta {delta_db} dB exceeds 1 dB tolerance (perceptually relevant, <12kHz)"
                            );
                        }
                        // >=12kHz (near-Nyquist): not gated — attenuation here
                        // under compact storage is the expected, inaudible cost.
                    }
                    k += 1;
                }
            }
        }
    }

    proptest::proptest! {
        #[test]
        fn wavetable_output_bounded(freq in 20.0f32..=8_000.0, pm in -2.0f32..=2.0) {
            let m = saw_mips();
            let refs = mipset(&m);
            let mut osc = WtOsc::new();
            let mut out = [0.0f32; 256];
            osc.process(MipSet { levels: &refs }, In::K(freq), In::K(pm), 1.0 / 48_000.0, &mut out);
            for s in out { proptest::prop_assert!(s.is_finite() && s.abs() <= 1.2); }
        }
    }

    #[test]
    fn wtosc_feedback_zero_is_identity() {
        // A fresh WtOsc with feedback=0 must render bit-identically to before.
        // (Compare a feedback=0 instance against an explicit second instance
        // driven the same way — this locks the feedback==0 fast-path/general-
        // path identity.)
        let m = saw_mips();
        let refs = mipset(&m);
        let dt = 1.0 / 48_000.0;
        let n = 64;
        let mut a = WtOsc::new(); // feedback defaults 0
        let mut out_a = std::vec![0.0f32; n];
        a.process(MipSet { levels: &refs }, In::K(440.0), In::K(0.0), dt, &mut out_a);
        let mut b = WtOsc::new();
        b.set_feedback(0.0); // explicit 0 — must not change anything
        let mut out_b = std::vec![0.0f32; n];
        b.process(MipSet { levels: &refs }, In::K(440.0), In::K(0.0), dt, &mut out_b);
        assert_eq!(out_a, out_b, "feedback=0 is identity");
    }

    #[test]
    fn wtosc_feedback_stays_bounded() {
        let m = saw_mips();
        let refs = mipset(&m);
        let dt = 1.0 / 48_000.0;
        let n = 256;
        let mut a = WtOsc::new();
        a.set_feedback(1.0);
        let mut out = std::vec![0.0f32; n];
        a.process(MipSet { levels: &refs }, In::K(440.0), In::K(0.0), dt, &mut out);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 2.0), "feedback bounded");
    }

    use std::vec::Vec;
    use std::vec;

    fn frame_region(bases: &[[f32; N]]) -> Vec<f32> { // host test; std Vec ok in tests
        let mut r = vec![0.0f32; bases.len() * COMPACT_LEN];
        for (f, b) in bases.iter().enumerate() {
            mipgen::build_pyramid_flat_compact(b, &mut r[f * COMPACT_LEN..(f + 1) * COMPACT_LEN]);
        }
        r
    }

    /// Renders a single base waveform alone (compact-pyramid `process`, not
    /// `process_morph`) at `f0`, starting from a fresh (phase=0) `WtOsc`. Used
    /// as ground truth for individual morph-frame samples: since `process_morph`
    /// also starts each render from a fresh `WtOsc` and uses the same
    /// freq/dt, the per-sample phase trajectory is identical, so these values
    /// are exactly the `sample_one` outputs `process_morph` reads for that
    /// frame at each output index.
    fn render_frame_alone(base: &[f32; N], f0: f32, sr: f32) -> [f32; 256] {
        let mut region = vec![0.0f32; COMPACT_LEN];
        mipgen::build_pyramid_flat_compact(base, &mut region);
        let levels = compact_levels(&region);
        let mut o = [0.0f32; 256];
        let mut w = WtOsc::new();
        w.process(MipSet { levels: &levels }, In::K(f0), In::K(0.0), 1.0 / sr, &mut o);
        o
    }

    #[test]
    fn morph_frames1_matches_single_cycle() {
        // FRAMES==1 morph must equal the single-cycle process bit-for-bit.
        let mut saw = [0.0f32; N];
        for (i, s) in saw.iter_mut().enumerate() { *s = 2.0 * (i as f32 / N as f32) - 1.0; }
        let region = frame_region(&[saw]);
        let levels = compact_levels(&region);
        let (mut a, mut b) = (WtOsc::new(), WtOsc::new());
        let mut oa = [0.0f32; 256];
        let mut ob = [0.0f32; 256];
        a.process(MipSet { levels: &levels }, In::K(220.0), In::K(0.0), 1.0 / 48_000.0, &mut oa);
        b.process_morph(&region, 1, In::K(220.0), In::K(0.0), In::K(0.5), 1.0 / 48_000.0, &mut ob);
        assert_eq!(oa, ob); // bit-exact
    }

    #[test]
    fn morph_position_endpoints_and_midpoint() {
        // position=0 → frame 0 exactly; position=1 → frame 1 exactly
        // (Catmull-Rom passes through its control points: `catmull_rom`
        // returns its `y1` arg at t=0 and its `y2` arg at t=1, and the
        // frame→arg mapping puts f0 in the `y1` slot, f1 in the `y2` slot).
        let mut saw = [0.0f32; N];
        let mut sq = [0.0f32; N];
        for i in 0..N { saw[i] = 2.0 * (i as f32 / N as f32) - 1.0; sq[i] = if i < N/2 {1.0} else {-1.0}; }
        let region = frame_region(&[saw, sq]);
        let sr = 48_000.0f32;
        let f0hz = 220.0f32;
        let render = |pos: f32| { let mut o=[0.0f32;256]; let mut w=WtOsc::new();
            w.process_morph(&region, 2, In::K(f0hz), In::K(0.0), In::K(pos), 1.0/sr, &mut o); o };

        let ya = render_frame_alone(&saw, f0hz, sr); // frame 0 in isolation
        let yb = render_frame_alone(&sq, f0hz, sr);  // frame 1 in isolation
        let f0 = render(0.0);
        let f1 = render(1.0);
        assert_eq!(f0, ya, "position=0 must equal frame 0 exactly");
        assert_eq!(f1, yb, "position=1 must equal frame 1 exactly");

        // Midpoint: with only 2 frames, both outer taps clamp to duplicate
        // the adjacent real frame (fm1==f0, f1==f2), i.e. the read is
        // catmull_rom(A,A,B,B,0.5). That specific case's cubic polynomial
        // happens to numerically equal the 2-tap linear average AT t=0.5
        // (though not at other t — a property of the symmetric double-clamp,
        // not of Catmull-Rom in general). Assert against the explicit
        // 4-tap formula (not "the average") so this test documents *why*,
        // and see `morph_cubic_frame_read_uses_four_frames` for a >=4-frame
        // case where the cubic blend provably reads the outer taps and
        // differs from the 2-tap linear result.
        let mid = render(0.5);
        for i in 0..256 {
            let expected = catmull_rom(ya[i], ya[i], yb[i], yb[i], 0.5);
            assert!((mid[i] - expected).abs() < 1e-4, "midpoint cubic @ {i}: {} vs {}", mid[i], expected);
        }
    }

    #[test]
    fn morph_cubic_frame_read_uses_four_frames() {
        // With >=4 distinct (non-collinear) frames, the cubic frame blend
        // must depend on the outer taps (f-1, f+2), not just the two
        // bracketing frames (f, f+1) — proving it reads 4 frames, not 2.
        let mut saw = [0.0f32; N];
        let mut sq = [0.0f32; N];
        let mut tri = [0.0f32; N];
        let mut sine = [0.0f32; N];
        for i in 0..N {
            let t = i as f32 / N as f32;
            saw[i] = 2.0 * t - 1.0;
            sq[i] = if i < N / 2 { 1.0 } else { -1.0 };
            tri[i] = 4.0 * (t - 0.5).abs() - 1.0;
            sine[i] = libm::sinf(core::f32::consts::TAU * t);
        }
        let sr = 48_000.0f32;
        let f0hz = 220.0f32;
        let region = frame_region(&[saw, sq, tri, sine]); // frames 0..3, last=3

        let ya = render_frame_alone(&saw, f0hz, sr);
        let yb = render_frame_alone(&sq, f0hz, sr);
        let yc = render_frame_alone(&tri, f0hz, sr);
        let yd = render_frame_alone(&sine, f0hz, sr);

        // position=0.5 -> fpos = 0.5*3 = 1.5 exactly: f0=1(sq), ffrac=0.5,
        // fm1=0(saw), f0c=1(sq), f1=2(tri), f2=3(sine) — all 4 distinct frames.
        let mut out = [0.0f32; 256];
        let mut w = WtOsc::new();
        w.process_morph(&region, 4, In::K(f0hz), In::K(0.0), In::K(0.5), 1.0 / sr, &mut out);

        let mut max_abs_diff_from_linear = 0.0f32;
        for i in 0..256 {
            let expected_cubic = catmull_rom(ya[i], yb[i], yc[i], yd[i], 0.5);
            assert!((out[i] - expected_cubic).abs() < 1e-4,
                "sample {i}: {} vs expected cubic {expected_cubic} (reads all 4 frames)", out[i]);
            let linear_2tap = 0.5 * (yb[i] + yc[i]);
            max_abs_diff_from_linear = max_abs_diff_from_linear.max((out[i] - linear_2tap).abs());
        }
        assert!(max_abs_diff_from_linear > 1e-3,
            "cubic blend should differ measurably from the old 2-tap linear average \
             (max abs diff = {max_abs_diff_from_linear}); outer taps may not be in use");
    }

    #[test]
    fn named_2d_bank_morphs_and_is_well_formed() {
        // Task 4: HarmonicSweep (id 6) and FormantMorph (id 7) are named
        // *static* 2D morph banks baked into `wavetables_generated::TABLES`
        // at `gen_tables` time (no runtime build). `static_table_flat`
        // returns their flat region exactly like the single-cycle tables;
        // `frames = region.len()/COMPACT_LEN` must come out >1 (multi-frame)
        // and the render path (`process_morph`, same as a pooled 2D bank)
        // must produce finite, non-silent, continuous, and — crucially —
        // *position-dependent* output (position actually selects different
        // frames, not just replaying the same one).
        let sr = 48_000.0f32;
        let f0 = 220.0f32;
        for &(id, name) in &[(6u16, "HarmonicSweep"), (7u16, "FormantMorph")] {
            let region = static_table_flat(TableId(id)).expect("named 2d bank");
            assert_eq!(region.len() % COMPACT_LEN, 0, "{name}: not a whole number of frames");
            let frames = region.len() / COMPACT_LEN;
            assert!(frames > 1, "{name}: expected a multi-frame (2D) bank, got frames={frames}");

            let render = |pos: f32| {
                let mut o = [0.0f32; deluge_dsp_test::FFT_N];
                let mut w = WtOsc::new();
                w.process_morph(region, frames, In::K(f0), In::K(0.0), In::K(pos), 1.0 / sr, &mut o);
                o
            };

            // Endpoints: finite, non-silent.
            let buf0 = render(0.0);
            let buf1 = render(1.0);
            for (buf, tag) in [(&buf0, "pos=0"), (&buf1, "pos=1")] {
                for &s in buf.iter() {
                    assert!(s.is_finite(), "{name} {tag}: non-finite sample");
                }
                let rms = (buf.iter().map(|s| s * s).sum::<f32>() / buf.len() as f32).sqrt();
                assert!(rms > 0.01, "{name} {tag}: silent (rms={rms})");
            }

            // position=0 vs position=1 must select measurably different
            // frames: compare the harmonic-magnitude spectra (broadband
            // distance across the first 24 harmonics of f0), not just RMS
            // (which per-frame normalization keeps roughly constant).
            let spec0 = deluge_dsp_test::spectrum::analyze_buf(sr, &buf0);
            let spec1 = deluge_dsp_test::spectrum::analyze_buf(sr, &buf1);
            let h0 = spec0.harmonics_db(f0, 24);
            let h1 = spec1.harmonics_db(f0, 24);
            let dist: f32 = h0.iter().zip(h1.iter()).map(|(a, b)| (a - b).powi(2)).sum();
            assert!(dist > 4.0, "{name}: pos=0 vs pos=1 harmonic spectra too similar (dist^2={dist})");

            // Continuity across a position sweep: no hard jump (a broken
            // frame-bracket/crossfade would show up as a large RMS step).
            let mut prev_rms: Option<f32> = None;
            let mut p = 0.0f32;
            while p <= 1.0 {
                let buf = render(p);
                let rms = (buf.iter().map(|s| s * s).sum::<f32>() / buf.len() as f32).sqrt();
                if let Some(pr) = prev_rms {
                    assert!((rms - pr).abs() < 0.15, "{name}: rms jump at pos {p}: {pr}->{rms}");
                }
                prev_rms = Some(rms);
                p += 0.05;
            }
        }
    }

    #[test]
    fn morph_sweep_is_continuous() {
        // Note on step size: saw and square are antiphase over much of the
        // cycle, so a raw per-sample amplitude crossfade between them has a
        // legitimately steep (but smooth, no jump) RMS trend from
        // destructive interference — verified analytically (naive, un-band-
        // limited model) to have a worst-case slope of ~1.5 RMS-units per
        // unit position, peaking near the frame endpoints. The brief's
        // original 0.05 position step (matched to the *unrelated*
        // mip-crossfade-boundary test's convention) is too coarse for that
        // slope against the 0.05 RMS-delta gate: it fires on the legitimate
        // trend, not a real discontinuity. A 0.01 step keeps worst-case
        // per-step delta (~0.015, well under the 0.05 gate) while still
        // catching a real jump (e.g. a hard frame swap without
        // interpolation), which would show up as a delta far larger than
        // this smooth trend's.
        let mut saw = [0.0f32; N]; let mut sq = [0.0f32; N];
        for i in 0..N { saw[i] = 2.0*(i as f32/N as f32)-1.0; sq[i] = if i<N/2 {1.0} else {-1.0}; }
        let region = frame_region(&[saw, sq]);
        let sr = 48_000.0f32; let mut prev: Option<f32> = None; let mut p = 0.0f32;
        while p <= 1.0 {
            let mut o = [0.0f32; 256]; let mut w = WtOsc::new();
            w.process_morph(&region, 2, In::K(220.0), In::K(0.0), In::K(p), 1.0/sr, &mut o);
            let rms = (o.iter().map(|s| s*s).sum::<f32>()/o.len() as f32).sqrt();
            if let Some(pr) = prev { assert!((rms-pr).abs() < 0.05, "morph rms jump @ pos {p}"); }
            prev = Some(rms); p += 0.01;
        }
    }

    // These two "hoist is bit-exact" tests assert a scalar-only property: the
    // hoisted const-freq path (`In::K`, taken via `as_const()`) must equal
    // the always-scalar general per-sample path (`In::A`) bit-for-bit. Under
    // `--features simd`, the hoisted path takes the `f32x8` SIMD sub-branch
    // instead — tolerance-equivalent to scalar by design (float
    // reassociation + closed-form phase), not bit-exact — so this specific
    // assertion no longer applies (the SIMD feature's parity guarantee is
    // `simd_matches_scalar_within_tol`/`_morph` below instead). Gated to the
    // non-simd (default) build, where they remain the bit-exact reference.
    #[cfg(not(feature = "simd"))]
    #[test]
    fn const_freq_hoist_is_bit_exact() {
        // The hoisted const-freq scalar path in `process` must equal the
        // general per-sample path bit-for-bit: feed the same freq/pmod
        // values once as `In::K` (takes the new hoisted branch) and once as
        // a constant-valued `In::A` buffer (forces the unchanged general
        // per-sample loop, since `as_const()` is `None` for `In::A`).
        let m = saw_mips();
        let refs = mipset(&m);
        let mut a = WtOsc::new();
        let mut oa = [0.0f32; 512];
        a.process(MipSet { levels: &refs }, In::K(220.0), In::K(0.0), 1.0 / 48_000.0, &mut oa);

        let fbuf = [220.0f32; 512];
        let pbuf = [0.0f32; 512];
        let mut b = WtOsc::new();
        let mut ob = [0.0f32; 512];
        b.process(MipSet { levels: &refs }, In::A(&fbuf), In::A(&pbuf), 1.0 / 48_000.0, &mut ob);

        assert_eq!(oa, ob); // bit-for-bit
    }

    #[cfg(not(feature = "simd"))]
    #[test]
    fn const_freq_hoist_process_morph_is_bit_exact() {
        // Same bit-exact parity check as `const_freq_hoist_is_bit_exact`,
        // but for `process_morph`'s const-freq/pmod/position fast path
        // (which also hoists the frame bracket and shared mip_select).
        let mut saw = [0.0f32; N];
        let mut sq = [0.0f32; N];
        let mut tri = [0.0f32; N];
        let mut sine = [0.0f32; N];
        for i in 0..N {
            let t = i as f32 / N as f32;
            saw[i] = 2.0 * t - 1.0;
            sq[i] = if i < N / 2 { 1.0 } else { -1.0 };
            tri[i] = 4.0 * (t - 0.5).abs() - 1.0;
            sine[i] = libm::sinf(core::f32::consts::TAU * t);
        }
        let region = frame_region(&[saw, sq, tri, sine]);
        let sr = 48_000.0f32;

        let mut a = WtOsc::new();
        let mut oa = [0.0f32; 512];
        a.process_morph(&region, 4, In::K(220.0), In::K(0.0), In::K(0.5), 1.0 / sr, &mut oa);

        let fbuf = [220.0f32; 512];
        let pbuf = [0.0f32; 512];
        let posbuf = [0.5f32; 512];
        let mut b = WtOsc::new();
        let mut ob = [0.0f32; 512];
        b.process_morph(&region, 4, In::A(&fbuf), In::A(&pbuf), In::A(&posbuf), 1.0 / sr, &mut ob);

        assert_eq!(oa, ob); // bit-for-bit
    }

    // ---- SIMD (f32x8) const-freq null-tests + speedup (Task 2) ----
    //
    // These reproduce the Task-1 hoisted-SCALAR const-freq body verbatim
    // (same `mip_select`/`sample_at_level`/`catmull_rom` calls, iterative
    // phase) as a standalone reference. Under `--features simd`,
    // `WtOsc::process`/`process_morph`'s own const-freq branch always takes
    // the new `f32x8` sub-path (the scalar sub-branch is compiled out by
    // `#[cfg(not(feature = "simd"))]`), so this reference is the only way to
    // get the bit-exact-scalar comparison point in a `--features simd`
    // build; it is exactly the code the non-simd build runs (see the
    // `#[cfg(not(feature = "simd"))]` branches in `process`/`process_morph`).
    #[cfg(feature = "simd")]
    fn scalar_hoisted_process(mips: &MipSet, freq: f32, pm: f32, dt: f32, phase: &mut f32, out: &mut [f32]) {
        let dtp = freq * dt;
        let (lo, hi, frac) = mip_select(dtp, mips.levels.len());
        let (lo_s, hi_s) = (mips.levels[lo], mips.levels[hi]);
        for s in out.iter_mut() {
            let mut ph = *phase + pm;
            ph -= floorf(ph);
            *s = sample_at_level(lo_s, hi_s, frac, ph);
            *phase += dtp;
            *phase -= floorf(*phase);
        }
    }

    #[cfg(feature = "simd")]
    #[allow(clippy::too_many_arguments)]
    fn scalar_hoisted_process_morph(
        region: &[f32], frames: usize, freq: f32, pm: f32, pos: f32, dt: f32,
        phase: &mut f32, out: &mut [f32],
    ) {
        if frames == 0 || region.len() < frames * COMPACT_LEN { return; }
        let last = frames - 1;
        let dtp = freq * dt;
        let fpos = pos.clamp(0.0, 1.0) * last as f32;
        let f0 = fpos as usize;
        let f0 = if f0 > last { last } else { f0 };
        let ffrac = (fpos - f0 as f32).clamp(0.0, 1.0);
        let fm1 = if f0 == 0 { 0 } else { f0 - 1 };
        let f0c = if f0 > last { last } else { f0 };
        let f1 = if f0c + 1 > last { last } else { f0c + 1 };
        let f2 = if f0c + 2 > last { last } else { f0c + 2 };
        let mm1 = compact_levels(&region[fm1 * COMPACT_LEN..(fm1 + 1) * COMPACT_LEN]);
        let m0 = compact_levels(&region[f0c * COMPACT_LEN..(f0c + 1) * COMPACT_LEN]);
        let m1 = compact_levels(&region[f1 * COMPACT_LEN..(f1 + 1) * COMPACT_LEN]);
        let m2 = compact_levels(&region[f2 * COMPACT_LEN..(f2 + 1) * COMPACT_LEN]);
        let (lo, hi, frac) = mip_select(dtp, LEVELS);
        let (mm1_lo, mm1_hi) = (mm1[lo], mm1[hi]);
        let (m0_lo, m0_hi) = (m0[lo], m0[hi]);
        let (m1_lo, m1_hi) = (m1[lo], m1[hi]);
        let (m2_lo, m2_hi) = (m2[lo], m2[hi]);
        for s in out.iter_mut() {
            let mut ph = *phase + pm;
            ph -= floorf(ph);
            let ym1 = sample_at_level(mm1_lo, mm1_hi, frac, ph);
            let y0 = sample_at_level(m0_lo, m0_hi, frac, ph);
            let y1 = sample_at_level(m1_lo, m1_hi, frac, ph);
            let y2 = sample_at_level(m2_lo, m2_hi, frac, ph);
            *s = catmull_rom(ym1, y0, y1, y2, ffrac);
            *phase += dtp;
            *phase -= floorf(*phase);
        }
    }

    /// Max `|simd - scalar|` tolerance for a single realistic render (see
    /// `NS` below): measured worst case ~3.2e-4 (`process`) / ~1.3e-4
    /// (`process_morph`) across a sweep of ~20 log-spaced frequencies
    /// (20 Hz-8 kHz, covering every mip level) x 7 pmod values; set with a
    /// ~1.5x margin above the larger (`process`) figure. This is f32 FMA/reassociation +
    /// closed-form-vs-iterative-phase noise (see `simd8`'s doc-comment) —
    /// NOT a formula/index bug: verified by (a) the divergence tracking
    /// exactly with proximity to a mip-table wrap boundary (the steepest
    /// local slope in a band-limited saw, where a sub-ULP phase disagreement
    /// has the largest effect on which table cell gets read) and (b)
    /// restructuring the SIMD chunk-phase derivation to a single multiply
    /// from a call-start anchor (avoiding compounding per-chunk rounding)
    /// barely moving the number — the dominant term is the scalar
    /// reference's own per-*sample* accumulated rounding over many
    /// additions, which no SIMD-side restructuring can eliminate.
    ///
    /// IMPORTANT — this bound does **not** hold for arbitrarily long
    /// renders: `self.phase` is an iteratively-accumulated f32, and two
    /// differently-grouped summations of the same `dtp` (per-sample, scalar;
    /// per-chunk-then-once-per-call, SIMD) drift apart roughly with sample
    /// count. Two distinct measurements, don't conflate them: (1) the
    /// per-block worst that `SIMD_TOL` bounds is ~3e-4, taken across the whole
    /// frequency sweep at the real block length (`NS`=133; one `MAX_BLOCK` of
    /// 128 plus tail). (2) Growth *at a single low frequency* over
    /// artificially long renders (informational, see
    /// `simd_long_render_drift_is_bounded_informational`, freq=20 Hz landing
    /// on a table wrap): ~1e-7 at 128 samples, ~1.7e-2 at 2048 — climbing with
    /// sample count, but only at render lengths no real audio block reaches.
    /// This is inherent to
    /// comparing two independent f32 accumulators, not unique to SIMD (any
    /// two differently-ordered summations of the same series diverge like
    /// this) — and immaterial in practice, since a shipped build only ever
    /// runs *one* of the two paths (never both side-by-side), so this
    /// cross-path divergence is never actually observed in production audio.
    #[cfg(feature = "simd")]
    const SIMD_TOL: f32 = 5e-4;

    /// One real audio block (`deluge_audio_graph::node::MAX_BLOCK` = 128)
    /// plus a few extra samples so the scalar tail (remainder after the
    /// last full `f32x8` chunk) is exercised too.
    #[cfg(feature = "simd")]
    const NS: usize = 133;

    #[cfg(feature = "simd")]
    #[test]
    fn simd_matches_scalar_within_tol() {
        let m = saw_mips();
        let refs = mipset(&m);
        let dt = 1.0 / 48_000.0f32;
        let mut freqs = std::vec::Vec::new();
        let mut f = 20.0f32;
        while f < 8_000.0 { freqs.push(f); f *= 1.13; }
        for &freq in freqs.iter() {
            for &pm in &[0.0f32, 0.05, -0.3, 1.7, -1.99, 0.999, 0.4321] {
                let mut simd_osc = WtOsc::new();
                let mut simd_out = [0.0f32; NS];
                simd_osc.process(MipSet { levels: &refs }, In::K(freq), In::K(pm), dt, &mut simd_out);
                let mut phase = 0.0f32;
                let mut scalar_out = [0.0f32; NS];
                scalar_hoisted_process(&MipSet { levels: &refs }, freq, pm, dt, &mut phase, &mut scalar_out);
                let mut max_diff = 0.0f32;
                for i in 0..NS {
                    max_diff = max_diff.max((simd_out[i] - scalar_out[i]).abs());
                }
                assert!(
                    max_diff < SIMD_TOL,
                    "process: freq={freq} pm={pm}: max |simd-scalar| = {max_diff}, TOL = {SIMD_TOL}"
                );
                // `self.phase` consistency: after one block, the SIMD path's
                // running phase must track the scalar path's closely, so a
                // subsequent block on the same `WtOsc` doesn't itself start
                // from an already-diverged phase.
                assert!(
                    (simd_osc.phase - phase).abs() < SIMD_TOL,
                    "process: freq={freq} pm={pm}: phase drift simd={} scalar={}", simd_osc.phase, phase
                );
            }
        }
    }

    #[cfg(feature = "simd")]
    #[test]
    fn simd_matches_scalar_within_tol_morph() {
        let mut saw = [0.0f32; N];
        let mut sq = [0.0f32; N];
        let mut tri = [0.0f32; N];
        let mut sine = [0.0f32; N];
        for i in 0..N {
            let t = i as f32 / N as f32;
            saw[i] = 2.0 * t - 1.0;
            sq[i] = if i < N / 2 { 1.0 } else { -1.0 };
            tri[i] = 4.0 * (t - 0.5).abs() - 1.0;
            sine[i] = libm::sinf(core::f32::consts::TAU * t);
        }
        let region = frame_region(&[saw, sq, tri, sine]);
        let sr = 48_000.0f32;
        let dt = 1.0 / sr;

        for &freq in &[55.0f32, 110.0, 220.0, 337.7, 1234.5, 2500.0, 4999.0] {
            for &pm in &[0.0f32, 0.05, -0.3, 1.7] {
                for &pos in &[0.0f32, 0.37, 0.5, 0.82, 1.0] {
                    let mut simd_osc = WtOsc::new();
                    let mut simd_out = [0.0f32; NS];
                    simd_osc.process_morph(&region, 4, In::K(freq), In::K(pm), In::K(pos), dt, &mut simd_out);

                    let mut phase = 0.0f32;
                    let mut scalar_out = [0.0f32; NS];
                    scalar_hoisted_process_morph(&region, 4, freq, pm, pos, dt, &mut phase, &mut scalar_out);

                    let mut max_diff = 0.0f32;
                    for i in 0..NS {
                        max_diff = max_diff.max((simd_out[i] - scalar_out[i]).abs());
                    }
                    assert!(
                        max_diff < SIMD_TOL,
                        "process_morph: freq={freq} pm={pm} pos={pos}: max |simd-scalar| = {max_diff}, TOL = {SIMD_TOL}"
                    );
                    assert!(
                        (simd_osc.phase - phase).abs() < SIMD_TOL,
                        "process_morph: freq={freq} pm={pm} pos={pos}: phase drift simd={} scalar={}", simd_osc.phase, phase
                    );
                }
            }
        }
    }

    /// Informational (not a correctness gate — see `SIMD_TOL`'s doc-comment):
    /// confirms the long-render cross-path phase divergence documented there
    /// is real, monotonic-ish with sample count, and doesn't blow up
    /// catastrophically (e.g. runaway/NaN) — just prints the measured worst
    /// case at a few render lengths. A shipped build only ever runs one path,
    /// so this never surfaces as an actual audio artifact; it's recorded here
    /// so a future maintainer re-discovering the divergence doesn't mistake
    /// it for a regression.
    #[cfg(feature = "simd")]
    #[test]
    fn simd_long_render_drift_is_bounded_informational() {
        use std::println;
        let m = saw_mips();
        let refs = mipset(&m);
        let dt = 1.0 / 48_000.0f32;
        for &ns in &[128usize, 512, 2048] {
            let freq = 20.0f32; // worst-observed case: lowest freq -> full-N table -> finest phase quantization
            let pm = 0.4321f32;
            let mut simd_osc = WtOsc::new();
            let mut simd_out = std::vec![0.0f32; ns];
            simd_osc.process(MipSet { levels: &refs }, In::K(freq), In::K(pm), dt, &mut simd_out);
            let mut phase = 0.0f32;
            let mut scalar_out = std::vec![0.0f32; ns];
            scalar_hoisted_process(&MipSet { levels: &refs }, freq, pm, dt, &mut phase, &mut scalar_out);
            let mut max_diff = 0.0f32;
            for i in 0..ns {
                max_diff = max_diff.max((simd_out[i] - scalar_out[i]).abs());
            }
            println!("[wavetable process, informational] {ns}-sample render: max |simd-scalar| = {max_diff}, final phase drift = {}", (simd_osc.phase - phase).abs());
            assert!(max_diff.is_finite(), "non-finite output at ns={ns}");
        }
    }

    // ---- Speedup measurement (informational, host/x86 only) ----
    //
    // `deluge_dsp_test::cpu::measure` (median-of-N wall clock) times the
    // SIMD const-freq path against the `scalar_hoisted_process*` reference
    // above (the exact Task-1 hoisted-scalar body). This is HOST x86 timing
    // — noisy and NOT the target device: it shows the win from
    // arithmetic-vectorization on top of Task 1's log2f/mip-select hoist,
    // using whatever SIMD ISA the host compiles to (SSE/AVX here). The real
    // Cortex-A9 number is a maintainer on-device measurement — NEON has no
    // hardware gather instruction, so the device speedup will differ
    // (possibly a loss, if lane-by-lane tap extraction dominates) from this
    // host figure. No specific ratio is asserted here, only that both sides
    // measured a positive duration.
    #[cfg(feature = "simd")]
    #[test]
    fn simd_speedup_process_host_informational() {
        use std::hint::black_box;
        use std::println;
        let m = saw_mips();
        let refs = mipset(&m);
        let sr = 48_000.0f32;
        let dt = 1.0 / sr;
        const BLOCK: usize = 512;

        let simd_report = deluge_dsp_test::cpu::measure(sr, BLOCK, 300, || {
            let mut osc = WtOsc::new();
            let mut out = [0.0f32; BLOCK];
            osc.process(MipSet { levels: &refs }, In::K(220.0), In::K(0.0), dt, &mut out);
            black_box(&out);
        });
        let scalar_report = deluge_dsp_test::cpu::measure(sr, BLOCK, 300, || {
            let mut phase = 0.0f32;
            let mut out = [0.0f32; BLOCK];
            scalar_hoisted_process(&MipSet { levels: &refs }, 220.0, 0.0, dt, &mut phase, &mut out);
            black_box(&out);
        });
        let speedup = deluge_dsp_test::cpu::compare(&scalar_report, &simd_report);
        println!(
            "[wavetable process, host/x86, informational] scalar {:.1} ns/block, simd {:.1} ns/block, speedup {:.2}x (device NEON number TBD, no hardware gather)",
            scalar_report.ns_per_block, simd_report.ns_per_block, speedup
        );
        assert!(simd_report.ns_per_block > 0.0 && scalar_report.ns_per_block > 0.0);
    }

    #[cfg(feature = "simd")]
    #[test]
    fn simd_speedup_process_morph_host_informational() {
        use std::hint::black_box;
        use std::println;
        let mut saw = [0.0f32; N];
        let mut sq = [0.0f32; N];
        let mut tri = [0.0f32; N];
        let mut sine = [0.0f32; N];
        for i in 0..N {
            let t = i as f32 / N as f32;
            saw[i] = 2.0 * t - 1.0;
            sq[i] = if i < N / 2 { 1.0 } else { -1.0 };
            tri[i] = 4.0 * (t - 0.5).abs() - 1.0;
            sine[i] = libm::sinf(core::f32::consts::TAU * t);
        }
        let region = frame_region(&[saw, sq, tri, sine]);
        let sr = 48_000.0f32;
        let dt = 1.0 / sr;
        const BLOCK: usize = 512;

        let simd_report = deluge_dsp_test::cpu::measure(sr, BLOCK, 300, || {
            let mut osc = WtOsc::new();
            let mut out = [0.0f32; BLOCK];
            osc.process_morph(&region, 4, In::K(220.0), In::K(0.0), In::K(0.5), dt, &mut out);
            black_box(&out);
        });
        let scalar_report = deluge_dsp_test::cpu::measure(sr, BLOCK, 300, || {
            let mut phase = 0.0f32;
            let mut out = [0.0f32; BLOCK];
            scalar_hoisted_process_morph(&region, 4, 220.0, 0.0, 0.5, dt, &mut phase, &mut out);
            black_box(&out);
        });
        let speedup = deluge_dsp_test::cpu::compare(&scalar_report, &simd_report);
        println!(
            "[wavetable process_morph, host/x86, informational] scalar {:.1} ns/block, simd {:.1} ns/block, speedup {:.2}x (device NEON number TBD, no hardware gather)",
            scalar_report.ns_per_block, simd_report.ns_per_block, speedup
        );
        assert!(simd_report.ns_per_block > 0.0 && scalar_report.ns_per_block > 0.0);
    }
}
