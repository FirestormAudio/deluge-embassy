#![no_std]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)]

use deluge_fft::RealFft;
use libm::sinf;

pub const N: usize = 2048;
pub const LEVELS: usize = 11;

const CORE_TWO_PI: f32 = core::f32::consts::TAU;

pub struct Harmonics {
    pub amp: [f32; N / 2 + 1],
    pub phase: [f32; N / 2 + 1],
}

/// Forward real FFT of one single-cycle base → per-harmonic amplitude & phase.
/// Amplitude is normalized so harmonic k reconstructs as `amp[k]*sin(2π k t + phase[k])`.
pub fn analyze(base: &[f32; N]) -> Harmonics {
    let mut spec = [Complex { re: 0.0, im: 0.0 }; N / 2 + 1];
    RealFft::<N, 4>::process(base, &mut spec);
    let mut amp = [0.0f32; N / 2 + 1];
    let mut phase = [0.0f32; N / 2 + 1];
    for k in 0..(N / 2 + 1) {
        let re = spec[k].re;
        let im = spec[k].im;
        // Real-FFT bin magnitude → time-domain sine amplitude: 2/N for
        // interior bins 1..N/2-1. DC (k=0) and Nyquist (k=N/2) are
        // special-cased to 1/N: those two bins have no "mirror" bin folded
        // into them (a real N-point DFT only has N/2-1 independent complex
        // bins plus two purely-real bins at k=0 and k=N/2), so the usual
        // one-sided doubling does not apply there. Concretely, a pure
        // Nyquist tone x[n] = A*(-1)^n gives X[N/2] = A*N (real), so
        // A = |X[N/2]| / N, not the doubled 2*|X[N/2]|/N.
        let scale = if k == 0 || k == N / 2 {
            1.0 / N as f32
        } else {
            2.0 / N as f32
        };
        amp[k] = scale * libm::sqrtf(re * re + im * im);
        // deluge-fft's RealFft uses the standard DFT convention
        // X[k] = sum_n x[n] e^{-j2*pi*k*n/N}. For a real signal built purely
        // from sin(2*pi*k*n/N + phase), this yields X[k] = -j*(N/2)*amp*e^{j*phase},
        // i.e. re = amp*(N/2)*sin(phase), im = -amp*(N/2)*cos(phase).
        // So phase = atan2(re, -im). Empirically verified in Step 5 below by
        // reconstructing level 0 from a base with arbitrary per-harmonic
        // phases and confirming it matches to numerical precision, while two
        // plausible alternative conventions do not (see
        // `phase_convention_reconstructs_base`).
        phase[k] = libm::atan2f(re, -im);
    }
    Harmonics { amp, phase }
}

/// Highest harmonic retained at `level` (0 = fullest, halving per octave, floor 1).
pub fn max_harmonic(level: usize) -> usize {
    let full = N / 2;
    let k = full >> level;
    if k < 1 {
        1
    } else {
        k
    }
}

/// Additive resynthesis of harmonics `1..=max_harmonic(level)` into `out`.
pub fn synth_level(h: &Harmonics, level: usize, out: &mut [f32; N]) {
    let kmax = max_harmonic(level);
    for (i, s) in out.iter_mut().enumerate() {
        let t = i as f32 / N as f32; // cycle position [0,1)
        let mut acc = 0.0f32;
        for k in 1..=kmax {
            acc += h.amp[k] * sinf(CORE_TWO_PI * k as f32 * t + h.phase[k]);
        }
        *s = acc;
    }
}

/// Build all `LEVELS` mip levels from one base cycle (additive resynthesis).
///
/// Kept as the equivalence oracle for the faster IFFT path (`build_all_ifft`
/// / default `build_all` below) and used by `gen_tables` so the committed
/// static tables are not perturbed by the new path.
pub fn build_all_additive(base: &[f32; N], out: &mut [[f32; N]; LEVELS]) {
    let h = analyze(base);
    for (level, out_level) in out.iter_mut().enumerate() {
        synth_level(&h, level, out_level);
    }
}

pub use deluge_fft::Complex;

/// Band-limit one level from a full forward spectrum: keep harmonics `1..=kmax`
/// (drop DC and everything above the cutoff — matching additive's `k=1..=kmax`),
/// then inverse-FFT.
pub fn build_level_ifft(spectrum: &[Complex; N / 2 + 1], level: usize, out: &mut [f32; N]) {
    let kmax = max_harmonic(level);
    let mut s = *spectrum;
    s[0] = Complex::ZERO; // drop DC to match additive (which sums k>=1)
    for (k, bin) in s.iter_mut().enumerate() {
        if k > kmax {
            *bin = Complex::ZERO;
        }
    }
    deluge_fft::RealFft::<N, 4>::process_inverse(&s, out);
}

/// Build all levels from one base via forward FFT + per-level band-limit + inverse.
pub fn build_all_ifft(base: &[f32; N], out: &mut [[f32; N]; LEVELS]) {
    let mut spectrum = [Complex::ZERO; N / 2 + 1];
    deluge_fft::RealFft::<N, 4>::process(base, &mut spectrum);
    for (level, out_level) in out.iter_mut().enumerate() {
        build_level_ifft(&spectrum, level, out_level);
    }
}

/// Build a full pyramid into a flat `N*LEVELS` region (for the 3b runtime path).
/// `base` is padded/truncated to `N`.
pub fn build_pyramid_flat(base: &[f32], region: &mut [f32]) {
    if region.len() < N * LEVELS {
        return;
    }
    let mut b = [0.0f32; N];
    let n = N.min(base.len());
    b[..n].copy_from_slice(&base[..n]);
    let mut spectrum = [Complex::ZERO; N / 2 + 1];
    deluge_fft::RealFft::<N, 4>::process(&b, &mut spectrum);
    let mut lvl = [0.0f32; N];
    for level in 0..LEVELS {
        build_level_ifft(&spectrum, level, &mut lvl);
        region[level * N..(level + 1) * N].copy_from_slice(&lvl);
    }
}

/// Default build path is now the fast IFFT one.
pub fn build_all(base: &[f32; N], out: &mut [[f32; N]; LEVELS]) {
    build_all_ifft(base, out);
}

#[cfg(test)]
mod tests {
    use super::*;

    // A naive full-harmonic saw single cycle as the base.
    fn saw_base() -> [f32; N] {
        let mut b = [0.0f32; N];
        for (i, s) in b.iter_mut().enumerate() {
            *s = 2.0 * (i as f32 / N as f32) - 1.0;
        }
        b
    }

    #[test]
    fn max_harmonic_halves_per_level() {
        assert_eq!(max_harmonic(0), N / 2);
        assert_eq!(max_harmonic(1), N / 4);
        assert!(max_harmonic(LEVELS - 1) >= 1);
    }

    #[test]
    fn synth_level_is_band_limited() {
        // Build level 3, FFT it with the INDEPENDENT realfft, assert no
        // energy above its cutoff harmonic.
        let h = analyze(&saw_base());
        let mut lvl = [0.0f32; N];
        synth_level(&h, 3, &mut lvl);

        // Render the single cycle repeated to FFT_N and measure.
        let sr = 48_000.0f32;
        let f0 = sr / N as f32; // one cycle spans N samples → fundamental = sr/N
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        for (i, s) in buf.iter_mut().enumerate() {
            *s = lvl[i % N];
        }
        let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
        let cutoff_hz = max_harmonic(3) as f32 * f0;
        // Energy at 2 harmonics above the cutoff should be ~silent.
        let above = spec.level_at(cutoff_hz + 2.0 * f0);
        let fund = spec.level_at(f0);
        assert!(above < 1e-3 * fund, "band-limited: above={above} fund={fund}");
    }

    /// A 3-partial tone with deliberately arbitrary, non-0/π per-harmonic
    /// phases (0.7, 2.3, -1.1 rad). Unlike the saw (whose Fourier phases are
    /// all exactly 0 or π by symmetry, which makes phase-sign errors
    /// invisible — see below), this waveform's shape is phase-sensitive: any
    /// wrong sign/axis convention detectably misaligns the harmonics.
    fn arbitrary_phase_base() -> [f32; N] {
        let mut b = [0.0f32; N];
        for (i, s) in b.iter_mut().enumerate() {
            let t = i as f32 / N as f32;
            *s = 1.0 * sinf(CORE_TWO_PI * t + 0.7)
                + 0.5 * sinf(CORE_TWO_PI * 2.0 * t + 2.3)
                + 0.3 * sinf(CORE_TWO_PI * 3.0 * t - 1.1);
        }
        b
    }

    /// Empirical phase-convention calibration (brief §note, Step 5).
    ///
    /// `analyze`'s `phase[k] = atan2f(re, -im)` is a claim about deluge-fft's
    /// FFT sign convention; this test is the measurement that backs it up.
    /// It reconstructs level 0 (all N/2 harmonics, i.e. the full signal for
    /// this 3-partial base) from the analyzed amp/phase pairs and checks the
    /// result is a near-perfect, same-orientation copy of the base via
    /// normalized cross-correlation. It also synthesizes the SAME amplitudes
    /// with two wrong phase conventions (negated phase, and swapped atan2
    /// args) to confirm the check actually discriminates — a wrong
    /// convention should score far worse, not merely "also pass".
    ///
    /// (An earlier version of this test used the naive saw as the base and
    /// found negated-phase scored ~0.997 too — a false pass. The saw's
    /// harmonics all have phase exactly 0 or π, and sin(x-0)=sin(x) while
    /// sin(x-π)=sin(x+π), so negating phase is a no-op for it: it can't
    /// discriminate a sign error. This arbitrary-phase base fixes that.)
    #[test]
    fn phase_convention_reconstructs_base() {
        let base = arbitrary_phase_base();
        let h = analyze(&base);

        // Correct convention: reuse the crate's own synth_level.
        let mut lvl0 = [0.0f32; N];
        synth_level(&h, 0, &mut lvl0);
        let corr_correct = normalized_correlation(&base, &lvl0);

        // Wrong convention #1: negate the phase.
        let mut lvl0_neg = [0.0f32; N];
        synth_with_phase(&h, |p| -p, &mut lvl0_neg);
        let corr_neg = normalized_correlation(&base, &lvl0_neg);

        // Wrong convention #2: swap atan2 args (atan2f(-im, re) instead of
        // atan2f(re, -im) — equivalent to a constant +-pi/2 phase shift for a
        // real signal built from sines, which badly distorts a saw's shape).
        let mut lvl0_swapped = [0.0f32; N];
        {
            let mut spec = [Complex { re: 0.0, im: 0.0 }; N / 2 + 1];
            RealFft::<N, 4>::process(&base, &mut spec);
            let mut h2 = Harmonics { amp: h.amp, phase: h.phase };
            for (k, c) in spec.iter().enumerate() {
                h2.phase[k] = libm::atan2f(-c.im, c.re);
            }
            synth_level(&h2, 0, &mut lvl0_swapped);
        }
        let corr_swapped = normalized_correlation(&base, &lvl0_swapped);

        assert!(
            corr_correct > 0.99,
            "chosen phase convention atan2f(re,-im) should reconstruct the \
             base saw closely: corr={corr_correct}"
        );
        assert!(
            corr_correct > corr_neg + 0.5,
            "negated-phase convention should score far worse: correct={corr_correct} neg={corr_neg}"
        );
        assert!(
            corr_correct > corr_swapped + 0.5,
            "swapped-atan2 convention should score far worse: correct={corr_correct} swapped={corr_swapped}"
        );
    }

    /// Like `synth_level` at level 0 but applying `f` to each phase first —
    /// used only to probe alternative phase conventions in the calibration test.
    fn synth_with_phase(h: &Harmonics, f: impl Fn(f32) -> f32, out: &mut [f32; N]) {
        let kmax = max_harmonic(0);
        for (i, s) in out.iter_mut().enumerate() {
            let t = i as f32 / N as f32;
            let mut acc = 0.0f32;
            for k in 1..=kmax {
                acc += h.amp[k] * sinf(CORE_TWO_PI * k as f32 * t + f(h.phase[k]));
            }
            *s = acc;
        }
    }

    /// Normalized cross-correlation in [-1, 1]; 1.0 = identical shape & sign.
    fn normalized_correlation(a: &[f32; N], b: &[f32; N]) -> f32 {
        let mut dot = 0.0f32;
        let mut na = 0.0f32;
        let mut nb = 0.0f32;
        for i in 0..N {
            dot += a[i] * b[i];
            na += a[i] * a[i];
            nb += b[i] * b[i];
        }
        dot / libm::sqrtf(na * nb)
    }

    #[test]
    fn nyquist_amplitude_not_doubled() {
        // A pure Nyquist tone: x[n] = A * (-1)^n. Its entire energy sits in
        // bin k = N/2, so analyze() must report amp[N/2] ≈ A. The old
        // (buggy) 2/N scale at this bin would report ≈ 2A instead.
        let a = 0.5f32;
        let mut base = [0.0f32; N];
        for (i, s) in base.iter_mut().enumerate() {
            *s = a * if i % 2 == 0 { 1.0 } else { -1.0 };
        }
        let h = analyze(&base);
        assert!(
            (h.amp[N / 2] - a).abs() < 1e-3,
            "Nyquist amplitude should be ~A={a}, got {}",
            h.amp[N / 2]
        );

        // Guard against over-correction: an interior-bin tone must still
        // round-trip at its usual 2/N scale.
        let k0 = 5usize;
        let mut interior = [0.0f32; N];
        for (i, s) in interior.iter_mut().enumerate() {
            let t = i as f32 / N as f32;
            *s = a * sinf(CORE_TWO_PI * k0 as f32 * t);
        }
        let h_interior = analyze(&interior);
        assert!(
            (h_interior.amp[k0] - a).abs() < 1e-3,
            "interior bin amplitude should still be ~A={a}, got {}",
            h_interior.amp[k0]
        );
    }

    #[test]
    fn build_all_is_deterministic() {
        let base = saw_base();
        let mut a = [[0.0f32; N]; LEVELS];
        let mut b = [[0.0f32; N]; LEVELS];
        build_all_additive(&base, &mut a);
        build_all_additive(&base, &mut b);
        assert_eq!(a, b);
    }

    #[test]
    fn ifft_matches_additive() {
        // Use an arbitrary-phase base (saw's all-0/π phases would hide sign bugs).
        let mut base = [0.0f32; N];
        for (i, s) in base.iter_mut().enumerate() {
            let t = i as f32 / N as f32;
            *s = libm::sinf(core::f32::consts::TAU * t + 0.7)
               + 0.5 * libm::sinf(core::f32::consts::TAU * 3.0 * t + 2.3)
               + 0.25 * libm::sinf(core::f32::consts::TAU * 5.0 * t - 1.1);
        }
        let mut add = [[0.0f32; N]; LEVELS];
        let mut ift = [[0.0f32; N]; LEVELS];
        build_all_additive(&base, &mut add);
        build_all_ifft(&base, &mut ift);
        for level in 0..LEVELS {
            for i in 0..N {
                assert!((add[level][i] - ift[level][i]).abs() < 1e-3,
                    "level {level} i {i}: additive {} vs ifft {}", add[level][i], ift[level][i]);
            }
        }
    }

    #[test]
    fn ifft_level_is_band_limited() {
        let mut base = [0.0f32; N];
        for (i, s) in base.iter_mut().enumerate() { *s = 2.0 * (i as f32 / N as f32) - 1.0; }
        let mut ift = [[0.0f32; N]; LEVELS];
        build_all_ifft(&base, &mut ift);
        let sr = 48_000.0f32;
        let f0 = sr / N as f32;
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        for (i, s) in buf.iter_mut().enumerate() { *s = ift[3][i % N]; }
        let spec = deluge_dsp_test::spectrum::analyze_buf(sr, &buf);
        let above = spec.level_at(max_harmonic(3) as f32 * f0 + 2.0 * f0);
        let fund = spec.level_at(f0);
        assert!(above < 1e-3 * fund, "ifft band-limited: above={above} fund={fund}");
    }

    #[test]
    fn build_pyramid_flat_matches_build_all_ifft() {
        let mut base = [0.0f32; N];
        for (i, s) in base.iter_mut().enumerate() { *s = 2.0 * (i as f32 / N as f32) - 1.0; }
        let mut nested = [[0.0f32; N]; LEVELS];
        build_all_ifft(&base, &mut nested);
        let mut flat = [0.0f32; N * LEVELS];
        build_pyramid_flat(&base, &mut flat);
        for level in 0..LEVELS {
            for i in 0..N {
                assert_eq!(flat[level * N + i], nested[level][i]);
            }
        }
    }
}
