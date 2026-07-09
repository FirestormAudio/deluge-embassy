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
}
