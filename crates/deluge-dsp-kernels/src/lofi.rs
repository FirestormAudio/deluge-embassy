//! Lo-fi degraders (Fx-2): `Bitcrush` (amplitude quantize) and `Decimate`
//! (sample-and-hold rate reduction). Mono, `no_std`. No anti-aliasing/dither —
//! the aliasing and quantization artifacts are the effect.

use crate::In; // same import path drive.rs/dynamics.rs use

/// Memoryless amplitude bit-depth reduction.
#[derive(Clone, Copy)]
pub struct Bitcrush {
    bits: f32,
}

impl Bitcrush {
    pub fn new(bits: f32) -> Bitcrush {
        Bitcrush {
            bits: bits.clamp(1.0, 24.0),
        }
    }
    pub fn set_bits(&mut self, b: f32) {
        self.bits = b.clamp(1.0, 24.0);
    }

    pub fn process(&mut self, input: In, _dt: f32, out: &mut [f32]) {
        let step = 1.0 / libm::powf(2.0, self.bits - 1.0); // 1 / 2^(bits-1)
        for i in 0..out.len() {
            let x = input.at(i);
            out[i] = libm::roundf(x / step) * step;
        }
    }
}

/// Sample-rate reduction by sample-and-hold. `rate_hz` = target rate.
#[derive(Clone, Copy)]
pub struct Decimate {
    rate_hz: f32,
    held: f32,
    phase: f32,
}

impl Decimate {
    pub fn new(rate_hz: f32) -> Decimate {
        Decimate {
            rate_hz: rate_hz.max(1.0),
            held: 0.0,
            phase: 1.0,
        }
    }
    pub fn set_rate(&mut self, hz: f32) {
        self.rate_hz = hz.max(1.0);
    }

    pub fn process(&mut self, input: In, dt: f32, out: &mut [f32]) {
        let inc = (self.rate_hz * dt).min(1.0); // rate/sr, capped (rate>=sr ⇒ passthrough)
        for i in 0..out.len() {
            self.phase += inc;
            if self.phase >= 1.0 {
                self.phase -= 1.0;
                self.held = input.at(i); // latch a fresh sample
            }
            out[i] = self.held;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::In;

    #[test]
    fn bitcrush_quantizes_to_step() {
        // bits=1 → step = 1/2^0 = 1 → round(x): 0.7→1, 0.3→0, -0.7→-1
        let mut b = Bitcrush::new(1.0);
        let inp = [0.7f32, 0.3, -0.7, -0.3];
        let mut out = [0.0f32; 4];
        b.process(In::A(&inp), 1.0 / 48_000.0, &mut out);
        assert_eq!(out, [1.0, 0.0, -1.0, 0.0]);
    }

    #[test]
    fn bitcrush_high_bits_near_transparent_and_on_grid() {
        // bits=8 → step = 1/128. Output on the grid, close to input.
        let mut b = Bitcrush::new(8.0);
        let inp: [f32; 32] = core::array::from_fn(|i| (i as f32 / 31.0) * 2.0 - 1.0); // -1..1
        let mut out = [0.0f32; 32];
        b.process(In::A(&inp), 1.0 / 48_000.0, &mut out);
        let step = 1.0 / libm::powf(2.0, 8.0 - 1.0);
        for (o, x) in out.iter().zip(inp.iter()) {
            let q = o / step;
            assert!(
                (q - libm::roundf(q)).abs() < 1e-3,
                "output on the {}-step grid",
                step
            );
            assert!((o - x).abs() <= step, "within one step of input");
        }
    }

    #[test]
    fn decimate_holds_and_passthrough() {
        // rate = sr/4 → inc = 0.25 → holds ~4 samples. phase inits to 1.0 so
        // sample 0 latches immediately.
        let sr = 48_000.0;
        let mut d = Decimate::new(sr / 4.0);
        let inp: [f32; 16] = core::array::from_fn(|i| i as f32);
        let mut out = [0.0f32; 16];
        d.process(In::A(&inp), 1.0 / sr, &mut out);
        // first sample latched; next two are held repeats of it.
        assert_eq!(out[0], inp[0]);
        assert_eq!(out[1], out[0], "held");
        assert_eq!(out[2], out[0], "held");
        // it does change (a later latch grabs a new sample) …
        assert!(
            out.iter().any(|&v| v != out[0]),
            "decimator latches new samples"
        );
        // … and is heavily piecewise-constant (most consecutive pairs equal).
        let repeats = out.windows(2).filter(|w| w[0] == w[1]).count();
        assert!(
            repeats >= out.len() / 2,
            "piecewise-constant (held), got {} repeats",
            repeats
        );

        // rate >= sr → inc capped at 1 → latches every sample → passthrough.
        let mut d2 = Decimate::new(sr);
        let mut out2 = [0.0f32; 16];
        d2.process(In::A(&inp), 1.0 / sr, &mut out2);
        assert_eq!(out2, inp, "rate>=sr is passthrough");
    }
}
