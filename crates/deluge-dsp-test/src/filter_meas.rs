//! Reusable filter measurement helpers (host-side, std). Drive a known signal
//! through a caller `render` closure and measure the steady-state response.
//! Shared by the whole Fi (filter) suite.

use crate::spectrum::analyze_buf;
use crate::FFT_N;

fn sine_buf(fs: f32, freq: f32) -> [f32; FFT_N] {
    let mut b = [0.0f32; FFT_N];
    let w = core::f32::consts::TAU * freq / fs;
    for (i, s) in b.iter_mut().enumerate() {
        *s = (w * i as f32).sin();
    }
    b
}

/// Steady-state magnitude of `render` at `freq`, in dB relative to unity
/// (0 dB = passthrough). `render` filters the buffer in place; it is called
/// once on a warm-up buffer (to settle state) then on the measured buffer.
pub fn magnitude_db(fs: f32, freq: f32, mut render: impl FnMut(&mut [f32])) -> f32 {
    let mut warm = sine_buf(fs, freq);
    render(&mut warm);
    let mut buf = sine_buf(fs, freq);
    render(&mut buf);
    let out = analyze_buf(fs, &buf).level_at(freq);
    let refl = analyze_buf(fs, &sine_buf(fs, freq)).level_at(freq);
    20.0 * (out / refl).max(1e-9).log10()
}

/// Lowest frequency (log-swept 20 Hz–20 kHz) where `render_at(freq, buf)`'s
/// magnitude first crosses −3 dB. `render_at` builds/render the filter at the
/// given probe `freq`, filling `buf` in place with the response to a unit sine
/// at that `freq` (fresh filter state per call). For a lowpass this is the
/// cutoff estimate.
pub fn minus_3db_hz(fs: f32, mut render_at: impl FnMut(f32, &mut [f32])) -> f32 {
    let mut prev = 20.0f32;
    let mut f = 20.0f32;
    while f < 20_000.0 {
        let db = magnitude_db(fs, f, |buf| render_at(f, buf));
        if db <= -3.0 {
            return (prev * f).sqrt(); // geometric midpoint of the crossing bracket
        }
        prev = f;
        f *= 1.05;
    }
    20_000.0
}

/// Drive `render` with no input (its closure must inject only its own state /
/// a tiny excitation), then report the dominant output frequency and the RMS
/// of the second half of the buffer (to check a sustained, non-decaying tone).
pub fn self_osc_hz_and_rms(fs: f32, mut render: impl FnMut(&mut [f32])) -> (f32, f32) {
    let mut buf = [0.0f32; FFT_N];
    render(&mut buf);
    let peak_hz = analyze_buf(fs, &buf).peak_bin() as f32 * fs / FFT_N as f32;
    // RMS of the second half — a sustained (non-decaying) oscillation keeps energy late.
    let half = FFT_N / 2;
    let mut acc = 0.0f32;
    for &s in &buf[half..] {
        acc += s * s;
    }
    let rms = (acc / (FFT_N - half) as f32).sqrt();
    (peak_hz, rms)
}

#[cfg(test)]
mod tests {
    use super::*;

    // A trivial reference: passing the signal through unchanged is ~0 dB at
    // any frequency; halving amplitude is ~ -6 dB.
    #[test]
    fn magnitude_db_measures_passthrough_and_gain() {
        let fs = 48_000.0;
        let passthrough = magnitude_db(fs, 1_000.0, |_buf| {});
        assert!(passthrough.abs() < 0.5, "passthrough {passthrough} dB");
        let halved = magnitude_db(fs, 1_000.0, |buf| {
            for s in buf.iter_mut() {
                *s *= 0.5;
            }
        });
        assert!((halved + 6.02).abs() < 0.5, "halved {halved} dB");
    }

    // A one-pole lowpass built inline (y += c*(x−y), c = 2π·fc·dt clamped) has
    // its −3 dB point at ≈ fc; the harness should recover it within ~20%.
    #[test]
    fn minus_3db_hz_recovers_one_pole_cutoff() {
        let fs = 48_000.0;
        let dt = 1.0 / fs;
        let fc = 1_000.0;
        let got = minus_3db_hz(fs, |_probe, buf| {
            let c = (core::f32::consts::TAU * fc * dt).min(1.0);
            let mut z = 0.0f32;
            for s in buf.iter_mut() {
                z += c * (*s - z);
                *s = z;
            }
        });
        assert!((got - fc).abs() / fc < 0.2, "one-pole cutoff est {got}");
    }
}
