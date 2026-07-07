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
/// Measured in Step 6 (compact-mipgen QA, see git log): six named bases
/// (saw/square/sine/tri/organ/formant, `2t-1` / `±1`@0.5 / `sin2πt` /
/// `1-4|t-0.5|` / `sin+0.5sin3+0.25sin5` / `sin·(1+cos)`) rendered at 5 kHz
/// through `WtOsc`, `worst_alias_db` vs the `< -21 dB` gate:
///
///   table    | F=1        | F=2
///   ---------|------------|------------
///   saw      | -25.67 dB  | -25.67 dB
///   square   | -25.67 dB  | -25.67 dB
///   sine     | -41.12 dB  | -41.12 dB
///   tri      | -39.65 dB  | -39.65 dB
///   organ    | -23.73 dB  | -23.73 dB  (worst of six, both F)
///   formant  | -41.12 dB  | -41.12 dB
///
/// F=1 and F=2 are numerically identical here: at 5 kHz the crossfaded
/// levels (7/8) already sit at `N_MIN` under both factors (their critical
/// length `N>>level` is ≤16, so `OS_FACTOR∈{1,2}` both clamp to 32), so
/// doubling `OS_FACTOR` changes zero bytes actually read at this frequency.
/// F=1 already clears the -21 dB gate by 2.3 dB (organ, the worst case) —
/// chosen over F=2 for the 5.5x storage win (`COMPACT_LEN`=4192 vs 6208,
/// both vs the flat `N*LEVELS`=22528), per the brief's "prefer F=1 if it
/// holds" rule. Do not weaken this gate to relax OS_FACTOR further.
pub const OS_FACTOR: usize = 1;

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
/// halves the harmonic count. Every level slice has length `N`.
pub struct MipSet<'a> {
    pub levels: &'a [&'a [f32]],
}

#[derive(Clone, Copy)]
pub struct WtOsc {
    phase: f32,
}

impl WtOsc {
    pub fn new() -> WtOsc {
        WtOsc { phase: 0.0 }
    }

    pub fn process(&mut self, mips: MipSet, freq: In, pmod: In, dt: f32, out: &mut [f32]) {
        debug_assert!(!mips.levels.is_empty());
        debug_assert_eq!(mips.levels[0].len(), N);
        let nlev = mips.levels.len();
        for (i, s) in out.iter_mut().enumerate() {
            let dtp = freq.at(i) * dt; // cycles/sample
            // Mip select: as dtp doubles (one octave up), drop one level of
            // harmonics. level0 is safe while its top harmonic (N/2) stays below
            // Nyquist: dtp*(N/2) < 0.5 → dtp < 1/N. Each octave above adds 1.
            // fractional level → crossfade weight.
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

            let mut ph = self.phase + pmod.at(i);
            ph -= floorf(ph);

            let a = interp_cubic(mips.levels[lo], ph);
            let b = interp_cubic(mips.levels[hi], ph);
            *s = a + (b - a) * frac.clamp(0.0, 1.0);

            self.phase += dtp;
            self.phase -= floorf(self.phase);
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
/// id assignment: 0=Saw 1=Square 2=Sine 3=Tri 4=Organ 5=Formant).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TableId(pub u16);

/// Resolve a named static table to a borrowed `MipSet`. `None` if out of range.
pub fn static_mipset(id: TableId) -> Option<MipSet<'static>> {
    let idx = id.0 as usize;
    let tables = &crate::wavetables_generated::TABLES;
    if idx >= tables.len() {
        return None;
    }
    Some(MipSet { levels: tables[idx] })
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
    let a = -0.5 * y0 + 1.5 * y1 - 1.5 * y2 + 0.5 * y3;
    let b = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
    let c = -0.5 * y0 + 0.5 * y2;
    ((a * frac + b) * frac + c) * frac + y1
}

#[cfg(test)]
mod tests {
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
        let m = static_mipset(TableId(0)).expect("saw table");
        let mut osc = WtOsc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(m, In::K(5_000.0), In::K(0.0), 1.0 / sr, &mut buf);
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
            let m = static_mipset(TableId(id)).expect("table");
            let mut osc = WtOsc::new();
            let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
            osc.process(m, In::K(5_000.0), In::K(0.0), 1.0 / sr, &mut buf);
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

    fn compact_levels(region: &[f32]) -> [&[f32]; LEVELS] {
        core::array::from_fn(|l| &region[level_offset(l)..level_offset(l) + level_len(l)])
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
        assert!(wa < -21.0, "compact saw worst_alias {wa} dB");
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
}
