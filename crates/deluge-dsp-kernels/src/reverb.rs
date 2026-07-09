//! Schroeder-Moorer (freeverb-topology) room reverb: damped-feedback `Comb` +
//! Schroeder `Allpass` primitives over a borrowed, partitioned ring buffer, and
//! a `Freeverb` engine (8 combs + 4 allpasses per channel, stereospread on the
//! right). Implemented from the public algorithm structure — not the GPL
//! freeverb source. `no_std`, no heap.

use crate::In;

const COMB: [usize; 8] = [1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617];
const AP: [usize; 4] = [556, 441, 341, 225];
const SPREAD: usize = 23;
const GAIN: f32 = 0.015;

/// Total samples the shared buffer must hold (Σ of all 24 line lengths).
pub const REVERB_BUF_SAMPLES: usize = 25_450;

/// Damped-feedback comb. Fixed integer delay = the slice length; cursor + one-pole
/// damp state. `off`/`len` locate this comb's slice in the shared buffer.
#[derive(Clone, Copy)]
pub struct Comb {
    c: usize,
    damp_z: f32,
}
impl Comb {
    pub fn new() -> Comb {
        Comb { c: 0, damp_z: 0.0 }
    }
    pub fn tick(&mut self, buf: &mut [f32], off: usize, len: usize, x: f32, feedback: f32, damp: f32) -> f32 {
        let out = buf[off + self.c];
        self.damp_z = out * (1.0 - damp) + self.damp_z * damp;
        buf[off + self.c] = x + self.damp_z * feedback;
        self.c += 1;
        if self.c >= len {
            self.c = 0;
        }
        out
    }
}

/// Schroeder allpass diffuser (fixed feedback 0.5). Cursor only.
#[derive(Clone, Copy)]
pub struct Allpass {
    c: usize,
}
impl Allpass {
    pub fn new() -> Allpass {
        Allpass { c: 0 }
    }
    pub fn tick(&mut self, buf: &mut [f32], off: usize, len: usize, x: f32) -> f32 {
        let bufout = buf[off + self.c];
        let out = -x + bufout;
        buf[off + self.c] = x + bufout * 0.5;
        self.c += 1;
        if self.c >= len {
            self.c = 0;
        }
        out
    }
}

/// Freeverb room reverb: 8 combs + 4 allpasses per channel over a shared
/// partitioned buffer; mono in → stereo out.
#[derive(Clone, Copy)]
pub struct Freeverb {
    combs_l: [Comb; 8],
    combs_r: [Comb; 8],
    aps_l: [Allpass; 4],
    aps_r: [Allpass; 4],
    roomsize: f32,
    damp: f32,
    width: f32,
    mix: f32,
}
impl Freeverb {
    pub fn new() -> Freeverb {
        Freeverb {
            combs_l: [Comb::new(); 8],
            combs_r: [Comb::new(); 8],
            aps_l: [Allpass::new(); 4],
            aps_r: [Allpass::new(); 4],
            roomsize: 0.5,
            damp: 0.5,
            width: 1.0,
            mix: 0.5,
        }
    }
    pub fn set_mix(&mut self, v: f32) { self.mix = v.clamp(0.0, 1.0); }
    pub fn set_damp(&mut self, v: f32) { self.damp = v.clamp(0.0, 1.0); }
    pub fn set_roomsize(&mut self, v: f32) { self.roomsize = v.clamp(0.0, 1.0); }
    pub fn set_width(&mut self, v: f32) { self.width = v.clamp(0.0, 1.0); }

    pub fn process(&mut self, input: In, _dt: f32, buf: &mut [f32], out_l: &mut [f32], out_r: &mut [f32]) {
        // Real-time safety: a too-small region → dry passthrough, never panic.
        if buf.len() < REVERB_BUF_SAMPLES {
            for i in 0..out_l.len() {
                let x = input.at(i);
                out_l[i] = x;
                out_r[i] = x;
            }
            return;
        }
        let feedback = self.roomsize * 0.28 + 0.7;
        let dc = self.damp * 0.4;
        let wet1 = self.mix * (self.width * 0.5 + 0.5);
        let wet2 = self.mix * ((1.0 - self.width) * 0.5);
        let dry = 1.0 - self.mix;
        for i in 0..out_l.len() {
            let x = input.at(i);
            let in_g = x * GAIN;
            let mut ol = 0.0;
            let mut or = 0.0;
            let mut off = 0usize;
            for k in 0..8 {
                let len = COMB[k];
                ol += self.combs_l[k].tick(buf, off, len, in_g, feedback, dc);
                off += len;
            }
            for k in 0..8 {
                let len = COMB[k] + SPREAD;
                or += self.combs_r[k].tick(buf, off, len, in_g, feedback, dc);
                off += len;
            }
            for k in 0..4 {
                let len = AP[k];
                ol = self.aps_l[k].tick(buf, off, len, ol);
                off += len;
            }
            for k in 0..4 {
                let len = AP[k] + SPREAD;
                or = self.aps_r[k].tick(buf, off, len, or);
                off += len;
            }
            out_l[i] = x * dry + ol * wet1 + or * wet2;
            out_r[i] = x * dry + or * wet1 + ol * wet2;
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;

    fn render(fv: &mut Freeverb, buf: &mut [f32], input: &[f32]) -> (std::vec::Vec<f32>, std::vec::Vec<f32>) {
        let mut l = std::vec![0.0f32; input.len()];
        let mut r = std::vec![0.0f32; input.len()];
        fv.process(In::A(input), 1.0 / 44_100.0, buf, &mut l, &mut r);
        (l, r)
    }

    #[test]
    fn layout_sums_to_reverb_buf_samples() {
        let combs: usize = COMB.iter().sum::<usize>() + COMB.iter().map(|c| c + SPREAD).sum::<usize>();
        let aps: usize = AP.iter().sum::<usize>() + AP.iter().map(|a| a + SPREAD).sum::<usize>();
        assert_eq!(combs + aps, REVERB_BUF_SAMPLES);
    }

    #[test]
    fn impulse_produces_decaying_diffuse_tail() {
        let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
        let mut fv = Freeverb::new();
        fv.set_mix(1.0);
        fv.set_roomsize(0.7);
        let mut input = std::vec![0.0f32; 20_000];
        input[0] = 1.0;
        let (l, r) = render(&mut fv, &mut buf, &input);
        assert!(l.iter().all(|v| v.is_finite()) && r.iter().all(|v| v.is_finite()));
        // Non-silent well after the impulse (recirculating tail).
        let late_energy: f32 = l[10_000..12_000].iter().map(|v| v * v).sum();
        assert!(late_energy > 1e-6, "tail died too fast: {late_energy}");
        // Decaying: an earlier window has more energy than a later one.
        let early: f32 = l[2_000..4_000].iter().map(|v| v * v).sum();
        let later: f32 = l[14_000..16_000].iter().map(|v| v * v).sum();
        assert!(early > later, "tail not decaying: {early} → {later}");
    }

    #[test]
    fn roomsize_lengthens_tail() {
        let tail = |size: f32| -> f32 {
            let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
            let mut fv = Freeverb::new();
            fv.set_mix(1.0);
            fv.set_roomsize(size);
            let mut input = std::vec![0.0f32; 30_000];
            input[0] = 1.0;
            let (l, _r) = render(&mut fv, &mut buf, &input);
            l[24_000..26_000].iter().map(|v| v * v).sum()
        };
        assert!(tail(0.9) > tail(0.3), "bigger room should decay slower");
    }

    #[test]
    fn damping_darkens_tail() {
        let hf = |damp: f32| -> f32 {
            let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
            let mut fv = Freeverb::new();
            fv.set_mix(1.0);
            fv.set_roomsize(0.8);
            fv.set_damp(damp);
            let mut input = std::vec![0.0f32; 20_000];
            input[0] = 1.0;
            let (l, _r) = render(&mut fv, &mut buf, &input);
            l[8_000..12_000].windows(2).map(|w| (w[1] - w[0]).powi(2)).sum()
        };
        assert!(hf(0.9) < hf(0.05), "more damping → less HF in the tail");
    }

    #[test]
    fn output_is_stereo() {
        let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
        let mut fv = Freeverb::new();
        fv.set_mix(1.0);
        let input: std::vec::Vec<f32> = (0..8_000).map(|i| (i as f32 * 0.03).sin()).collect();
        let (l, r) = render(&mut fv, &mut buf, &input);
        let diff: f32 = l.iter().zip(&r).map(|(a, b)| (a - b).abs()).sum();
        assert!(diff > 1.0, "reverb should be stereo (l != r): {diff}");
    }

    #[test]
    fn mix_zero_is_dry() {
        let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
        let mut fv = Freeverb::new();
        fv.set_mix(0.0);
        let input = std::vec![0.5f32; 256];
        let (l, r) = render(&mut fv, &mut buf, &input);
        assert!((l[100] - 0.5).abs() < 1e-4 && (r[100] - 0.5).abs() < 1e-4);
    }

    #[test]
    fn short_buffer_is_dry_passthrough() {
        let mut buf = std::vec![0.0f32; 64]; // < REVERB_BUF_SAMPLES
        let mut fv = Freeverb::new();
        fv.set_mix(1.0);
        let input = std::vec![0.3f32; 32];
        let (l, r) = render(&mut fv, &mut buf, &input);
        assert!(l.iter().all(|&v| (v - 0.3).abs() < 1e-6) && r.iter().all(|&v| (v - 0.3).abs() < 1e-6));
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 32, ..ProptestConfig::default() })]
        #[test]
        fn freeverb_is_finite_and_bounded(
            roomsize in 0.0f32..1.0,
            damp in 0.0f32..1.0,
            width in 0.0f32..1.0,
            mix in 0.0f32..1.0,
            amp in 0.0f32..1.0,
        ) {
            let mut buf = std::vec![0.0f32; REVERB_BUF_SAMPLES];
            let mut fv = Freeverb::new();
            fv.set_roomsize(roomsize); fv.set_damp(damp); fv.set_width(width); fv.set_mix(mix);
            let input = std::vec![amp; 4_000];
            let (l, r) = render(&mut fv, &mut buf, &input);
            for (a, b) in l.iter().zip(&r) {
                prop_assert!(a.is_finite() && b.is_finite());
                prop_assert!(a.abs() <= 16.0 && b.abs() <= 16.0, "unbounded: {a},{b}");
            }
        }
    }
}
