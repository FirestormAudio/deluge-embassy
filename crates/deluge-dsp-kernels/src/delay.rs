//! Delay-line primitive and feedback delay effect. A `DelayLine` holds only a
//! write cursor; its ring buffer is a borrowed `&mut [f32]` passed per call
//! (1 s @ 48 kHz = 48000 f32 can't live in a `Copy` kernel struct). `no_std`,
//! no heap, deterministic. The `Delay` effect (Task 2) borrows the same buffer.

/// A delay line over a caller-owned ring buffer. Holds only the write cursor;
/// the ring buffer (`&mut [f32]`) is supplied on every `write`/`read`.
#[derive(Clone, Copy)]
pub struct DelayLine {
    w: usize,
}

impl DelayLine {
    pub fn new() -> DelayLine {
        DelayLine { w: 0 }
    }

    /// Write `x` at the cursor, then advance (wrapping on `buf.len()`).
    /// No-op on an empty buffer.
    pub fn write(&mut self, buf: &mut [f32], x: f32) {
        let len = buf.len();
        if len == 0 {
            return;
        }
        if self.w >= len {
            self.w = 0; // defensive: a shrunk buffer between calls
        }
        buf[self.w] = x;
        self.w += 1;
        if self.w >= len {
            self.w = 0;
        }
    }

    /// Read `delay` samples back from the write cursor, 4-point (3rd-order)
    /// Hermite interpolation. `delay` is clamped to `[1.0, len-2]`. Returns
    /// `0.0` for a buffer too short (< 4) to interpolate.
    /// Intended for delays of at least ~3 samples (the 4-point kernel needs a
    /// valid tap on each side of the read point); musical delay times are far
    /// larger, so this is not a practical limit.
    pub fn read_hermite(&self, buf: &[f32], delay: f32) -> f32 {
        let len = buf.len();
        if len < 4 {
            return 0.0;
        }
        let d = delay.clamp(1.0, (len - 2) as f32);
        // Read position measured back from the write cursor.
        let rp = self.w as f32 - d;
        let i1 = floorf(rp);
        let frac = rp - i1;
        let base = i1 as isize;
        let at = |off: isize| -> f32 {
            // Euclidean wrap into [0, len).
            let idx = (base + off).rem_euclid(len as isize) as usize;
            buf[idx]
        };
        let xm1 = at(-1);
        let x0 = at(0);
        let x1 = at(1);
        let x2 = at(2);
        // Laurent de Soras 4-point 3rd-order Hermite.
        let c0 = x0;
        let c1 = 0.5 * (x1 - xm1);
        let c2 = xm1 - 2.5 * x0 + 2.0 * x1 - 0.5 * x2;
        let c3 = 0.5 * (x2 - xm1) + 1.5 * (x0 - x1);
        ((c3 * frac + c2) * frac + c1) * frac + c0
    }
}

impl Default for DelayLine {
    fn default() -> Self {
        Self::new()
    }
}

/// `floorf` without libm dependency creep here — small, finite positions.
#[inline]
fn floorf(x: f32) -> f32 {
    let i = x as i64 as f32;
    if x < i {
        i - 1.0
    } else {
        i
    }
}

use crate::In;

/// Feedback delay effect over a borrowed ring buffer. Holds the delay line, a
/// one-pole damping state in the feedback path, and the mix/damping control
/// params. `process` borrows the ring buffer each block.
#[derive(Clone, Copy)]
pub struct Delay {
    line: DelayLine,
    damp_z: f32, // one-pole LP state in the feedback path
    mix: f32,    // dry/wet, [0,1]
    damping: f32, // feedback-path LP amount, [0,1] (0 = bright, 1 = frozen/dark)
}

impl Delay {
    pub fn new() -> Delay {
        Delay { line: DelayLine::new(), damp_z: 0.0, mix: 0.35, damping: 0.0 }
    }

    pub fn set_mix(&mut self, v: f32) {
        self.mix = v.clamp(0.0, 1.0);
    }
    pub fn set_damping(&mut self, v: f32) {
        self.damping = v.clamp(0.0, 1.0);
    }

    /// One block. `time` is in seconds (→ samples via `dt`), `feedback` is
    /// clamped to `[0, 0.98]` internally. `buf` is the pooled ring buffer;
    /// `out.len()` samples are produced.
    pub fn process(
        &mut self,
        input: In,
        time: In,
        feedback: In,
        dt: f32,
        buf: &mut [f32],
        out: &mut [f32],
    ) {
        let max_d = if buf.len() >= 2 { (buf.len() - 2) as f32 } else { 1.0 };
        // One-pole coefficient: damping 0 → g=1 (bright/no filtering),
        // damping→1 → g→0 (dark). g is applied as damp_z += g*(y - damp_z).
        let g = 1.0 - self.damping;
        for i in 0..out.len() {
            let x = input.at(i);
            let d = (time.at(i) / dt).clamp(1.0, max_d);
            let y = self.line.read_hermite(buf, d);
            self.damp_z += g * (y - self.damp_z);
            let fb = feedback.at(i).clamp(0.0, 0.98) * self.damp_z;
            self.line.write(buf, x + fb);
            out[i] = x * (1.0 - self.mix) + y * self.mix;
        }
    }
}

impl Default for Delay {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_then_read_integer_delay_returns_written_sample() {
        let mut buf = [0.0f32; 64];
        let mut line = DelayLine::new();
        // Write a ramp 1,2,3,... then read integer delays back.
        for k in 1..=32 {
            line.write(&mut buf, k as f32);
        }
        // Last written was 32 (delay 1), previous 31 (delay 2), etc. Integer
        // delays hit taps exactly (frac == 0) so Hermite returns the sample.
        assert!((line.read_hermite(&buf, 1.0) - 32.0).abs() < 1e-3);
        assert!((line.read_hermite(&buf, 2.0) - 31.0).abs() < 1e-3);
        assert!((line.read_hermite(&buf, 5.0) - 28.0).abs() < 1e-3);
    }

    #[test]
    fn fractional_read_interpolates_between_neighbors() {
        let mut buf = [0.0f32; 64];
        let mut line = DelayLine::new();
        for k in 1..=32 {
            line.write(&mut buf, k as f32);
        }
        // Between delay 6 (buf value 27) and delay 5 (28): delay 5.5 ≈ 27.5 on
        // a linear ramp (Hermite is exact for a straight line, and all four
        // interpolation taps lie in written history at this delay — a 4-point
        // kernel needs valid samples on both sides of the read point, so very
        // small fractional delays below ~3 samples are outside its range).
        let mid = line.read_hermite(&buf, 5.5);
        assert!((mid - 27.5).abs() < 1e-3, "got {mid}");
    }

    #[test]
    fn wraps_around_without_panic_or_stale_read() {
        let mut buf = [0.0f32; 8];
        let mut line = DelayLine::new();
        // Write 100 samples through an 8-slot ring: must wrap, never panic.
        for k in 0..100 {
            line.write(&mut buf, k as f32);
        }
        // Most recent (delay 1) is 99.
        assert!((line.read_hermite(&buf, 1.0) - 99.0).abs() < 1e-3);
        // A clamped-large delay stays finite and in-buffer.
        assert!(line.read_hermite(&buf, 1000.0).is_finite());
    }

    #[test]
    fn short_or_empty_buffer_is_safe() {
        let mut empty: [f32; 0] = [];
        let mut line = DelayLine::new();
        line.write(&mut empty, 1.0); // no-op, no panic
        assert_eq!(line.read_hermite(&empty, 1.0), 0.0);
        let three = [1.0f32, 2.0, 3.0];
        assert_eq!(line.read_hermite(&three, 1.0), 0.0); // len < 4 → 0.0
    }

    // ── Delay effect ──────────────────────────────────────────────────────

    extern crate std; // test-only: std::vec for dynamic render buffers (this crate is no_std)
    use crate::In;

    // Render a whole block through a Delay with constant control values.
    fn render_delay(
        buf: &mut [f32],
        d: &mut Delay,
        input: &[f32],
        time_s: f32,
        feedback: f32,
        dt: f32,
    ) -> std::vec::Vec<f32> {
        let mut out = std::vec![0.0f32; input.len()];
        let t = std::vec![time_s; input.len()];
        let fb = std::vec![feedback; input.len()];
        d.process(In::A(input), In::A(&t), In::A(&fb), dt, buf, &mut out);
        out
    }

    #[test]
    fn impulse_appears_delayed_by_time() {
        let dt = 1.0 / 48_000.0;
        let time_s = 100.0 * dt; // 100-sample delay
        let mut buf = [0.0f32; 4096];
        let mut d = Delay::new();
        d.set_mix(1.0); // fully wet so the delayed impulse is the whole output
        let mut input = [0.0f32; 512];
        input[0] = 1.0;
        let out = render_delay(&mut buf, &mut d, &input, time_s, 0.0, dt);
        // Peak is near sample 100; almost nothing in [1, 90].
        let (peak_i, peak_v) =
            out.iter().enumerate().fold((0usize, 0.0f32), |(bi, bv), (i, &v)| {
                if v.abs() > bv { (i, v.abs()) } else { (bi, bv) }
            });
        assert!((peak_i as i32 - 100).abs() <= 1, "peak at {peak_i}");
        assert!(peak_v > 0.5, "delayed impulse too small: {peak_v}");
        let early: f32 = out[1..90].iter().map(|v| v.abs()).sum();
        assert!(early < 0.05, "energy before the delay tap: {early}");
    }

    #[test]
    fn feedback_produces_decaying_repeats() {
        let dt = 1.0 / 48_000.0;
        let time_s = 50.0 * dt;
        let mut buf = [0.0f32; 4096];
        let mut d = Delay::new();
        d.set_mix(1.0);
        let mut input = [0.0f32; 400];
        input[0] = 1.0;
        let out = render_delay(&mut buf, &mut d, &input, time_s, 0.6, dt);
        // Repeats near 50, 100, 150 with geometric decay.
        let tap = |n: usize| out[n - 1..=n + 1].iter().fold(0.0f32, |m, v| m.max(v.abs()));
        let (a, b, c) = (tap(50), tap(100), tap(150));
        assert!(a > b && b > c, "repeats not decaying: {a} {b} {c}");
        assert!(c > 0.05, "third repeat vanished: {c}");
        assert!(out.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn mix_balances_dry_and_wet() {
        let dt = 1.0 / 48_000.0;
        let time_s = 30.0 * dt;
        let input = [0.5f32; 256]; // steady DC
        // mix = 0 → output ≈ dry input at sample 0 (no wet yet).
        let mut b0 = [0.0f32; 2048];
        let mut d0 = Delay::new();
        d0.set_mix(0.0);
        let dry = render_delay(&mut b0, &mut d0, &input, time_s, 0.0, dt);
        assert!((dry[0] - 0.5).abs() < 1e-4, "mix=0 should pass dry: {}", dry[0]);
        // mix = 1 → output at sample 0 is pure (not-yet-arrived) wet ≈ 0.
        let mut b1 = [0.0f32; 2048];
        let mut d1 = Delay::new();
        d1.set_mix(1.0);
        let wet = render_delay(&mut b1, &mut d1, &input, time_s, 0.0, dt);
        assert!(wet[0].abs() < 1e-4, "mix=1 sample0 should be silent wet: {}", wet[0]);
    }

    #[test]
    fn damping_darkens_repeats() {
        // A high-frequency-rich burst fed back: more damping → less HF energy
        // in a late repeat. Measure via successive-sample difference energy
        // (a crude HF proxy) over a late window.
        let dt = 1.0 / 48_000.0;
        let time_s = 64.0 * dt;
        let mut input = [0.0f32; 600];
        for (i, s) in input.iter_mut().enumerate().take(32) {
            *s = if i % 2 == 0 { 1.0 } else { -1.0 }; // Nyquist-ish burst
        }
        let hf_energy = |out: &[f32], lo: usize, hi: usize| -> f32 {
            out[lo..hi].windows(2).map(|w| (w[1] - w[0]).powi(2)).sum()
        };
        let mut b_bright = [0.0f32; 4096];
        let mut bright = Delay::new();
        bright.set_mix(1.0);
        bright.set_damping(0.0);
        let ob = render_delay(&mut b_bright, &mut bright, &input, time_s, 0.7, dt);
        let mut b_dark = [0.0f32; 4096];
        let mut dark = Delay::new();
        dark.set_mix(1.0);
        dark.set_damping(0.5);
        let od = render_delay(&mut b_dark, &mut dark, &input, time_s, 0.7, dt);
        // Late repeat window (~4th repeat).
        let (lo, hi) = (250, 300);
        assert!(
            hf_energy(&od, lo, hi) < hf_energy(&ob, lo, hi),
            "damping should reduce HF energy in the repeat"
        );
    }

    use proptest::prelude::*;

    proptest! {
        #![proptest_config(ProptestConfig { cases: 96, ..ProptestConfig::default() })]
        #[test]
        fn delay_is_finite_and_bounded(
            time_ms in 0.5f32..40.0,
            feedback in 0.0f32..1.0, // clamped to 0.98 inside
            mix in 0.0f32..1.0,
            damping in 0.0f32..1.0,
            amp in 0.0f32..1.0,
        ) {
            let dt = 1.0 / 48_000.0;
            let time_s = time_ms * 1e-3;
            let mut buf = [0.0f32; 4096];
            let mut d = Delay::new();
            d.set_mix(mix);
            d.set_damping(damping);
            // Sustained bounded input is the worst case for a feedback loop.
            let input = std::vec![amp; 2000];
            let out = render_delay(&mut buf, &mut d, &input, time_s, feedback, dt);
            for &v in &out {
                prop_assert!(v.is_finite());
                // Loop gain ≤ 0.98 → steady-state gain ≤ ~50; 64 is a safe cap.
                prop_assert!(v.abs() <= 64.0, "unbounded: {v}");
            }
        }
    }
}
