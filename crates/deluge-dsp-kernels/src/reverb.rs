//! Schroeder-Moorer (freeverb-topology) room reverb: damped-feedback `Comb` +
//! Schroeder `Allpass` primitives over a borrowed, partitioned ring buffer, and
//! a `Freeverb` engine (8 combs + 4 allpasses per channel, stereospread on the
//! right). Implemented from the public algorithm structure — not the GPL
//! freeverb source. `no_std`, no heap.

use crate::In;
use crate::delay::DelayLine;
use crate::fast_sin;

const COMB: [usize; 8] = [1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617];
const AP: [usize; 4] = [556, 441, 341, 225];
const SPREAD: usize = 23;
const GAIN: f32 = 0.015;

/// Total samples the shared buffer must hold (Σ of all 24 line lengths).
pub const REVERB_BUF_SAMPLES: usize = 25_450;

const BASE_LEN: [usize; 8] = [1499, 1889, 2311, 2749, 3187, 3571, 3931, 4483];
const MOD_MARGIN: usize = 16;
const MOD_DEPTH: f32 = 8.0;
const LFO_RATE: f32 = 0.7;
const INJ: f32 = 0.05;
const OUT: f32 = 1.0;
const HADAMARD_NORM: f32 = 0.353_553_39; // 1/√8

/// Σ of the 8 line slice lengths (`base + MOD_MARGIN`).
pub const HALL_BUF_SAMPLES: usize = 23_748;

/// In-place fast Walsh-Hadamard transform on 8 samples, normalized by 1/√8
/// (orthonormal → energy-preserving). Adds/subtracts only.
pub(crate) fn fwht8(v: &mut [f32; 8]) {
    let mut h = 1;
    while h < 8 {
        let mut i = 0;
        while i < 8 {
            for j in i..i + h {
                let a = v[j];
                let b = v[j + h];
                v[j] = a + b;
                v[j + h] = a - b;
            }
            i += 2 * h;
        }
        h *= 2;
    }
    for x in v.iter_mut() {
        *x *= HADAMARD_NORM;
    }
}

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

impl Default for Comb {
    fn default() -> Self {
        Self::new()
    }
}
impl Default for Allpass {
    fn default() -> Self {
        Self::new()
    }
}
impl Default for Freeverb {
    fn default() -> Self {
        Self::new()
    }
}

/// 8-line modulated FDN hall reverb over a shared partitioned buffer; mono in →
/// stereo out. Feedback = `g · Hadamard · damp(taps)` with `g < 1` and an
/// orthonormal matrix ⇒ BIBO-stable by construction.
#[derive(Clone, Copy)]
pub struct Fdn8 {
    lines: [DelayLine; 8],
    damp_z: [f32; 8],
    lfo_phase: f32,
    size: f32,
    damp: f32,
    width: f32,
    mix: f32,
}
impl Fdn8 {
    pub fn new() -> Fdn8 {
        Fdn8 {
            lines: [DelayLine::new(); 8],
            damp_z: [0.0; 8],
            lfo_phase: 0.0,
            size: 0.5,
            damp: 0.5,
            width: 1.0,
            mix: 0.5,
        }
    }
    pub fn set_mix(&mut self, v: f32) { self.mix = v.clamp(0.0, 1.0); }
    pub fn set_damp(&mut self, v: f32) { self.damp = v.clamp(0.0, 1.0); }
    pub fn set_size(&mut self, v: f32) { self.size = v.clamp(0.0, 1.0); }
    pub fn set_width(&mut self, v: f32) { self.width = v.clamp(0.0, 1.0); }

    pub fn process(&mut self, input: In, dt: f32, buf: &mut [f32], out_l: &mut [f32], out_r: &mut [f32]) {
        if buf.len() < HALL_BUF_SAMPLES {
            for i in 0..out_l.len() {
                let x = input.at(i);
                out_l[i] = x;
                out_r[i] = x;
            }
            return;
        }
        let dc = self.damp * 0.4;
        let g = self.size * 0.25 + 0.7;
        let wet1 = self.mix * (self.width * 0.5 + 0.5);
        let wet2 = self.mix * ((1.0 - self.width) * 0.5);
        let dry = 1.0 - self.mix;
        for i in 0..out_l.len() {
            let x = input.at(i);
            let p = self.lfo_phase + LFO_RATE * dt;
            self.lfo_phase = p - libm::floorf(p);
            // Read (modulated) all 8 lines.
            let mut s = [0.0f32; 8];
            let mut off = 0usize;
            for k in 0..8 {
                let len = BASE_LEN[k] + MOD_MARGIN;
                let t = self.lfo_phase + k as f32 / 8.0;
                let lfo = fast_sin(t - libm::floorf(t));
                let d = BASE_LEN[k] as f32 - MOD_DEPTH * (0.5 + 0.5 * lfo);
                s[k] = self.lines[k].read_hermite(&buf[off..off + len], d);
                off += len;
            }
            // Per-line damping.
            let mut h = [0.0f32; 8];
            for k in 0..8 {
                self.damp_z[k] = s[k] * (1.0 - dc) + self.damp_z[k] * dc;
                h[k] = self.damp_z[k];
            }
            // Orthonormal Hadamard mix.
            fwht8(&mut h);
            // Feedback + inject + write.
            let mut off = 0usize;
            for k in 0..8 {
                let len = BASE_LEN[k] + MOD_MARGIN;
                self.lines[k].write(&mut buf[off..off + len], x * INJ + g * h[k]);
                off += len;
            }
            // Output: even taps → L, odd → R.
            let ol = (s[0] + s[2] + s[4] + s[6]) * OUT;
            let or = (s[1] + s[3] + s[5] + s[7]) * OUT;
            out_l[i] = x * dry + ol * wet1 + or * wet2;
            out_r[i] = x * dry + or * wet1 + ol * wet2;
        }
    }
}
impl Default for Fdn8 {
    fn default() -> Self {
        Self::new()
    }
}

/// Dattorro element lengths, order [idf0,idf1,idf2,idf3, ad1L,ad1R, daL,daR, ad2L,ad2R, dbL,dbR].
const PLATE_LEN: [usize; 12] = [142, 107, 379, 277, 672, 908, 4453, 4217, 1800, 2656, 3720, 3163];
pub const PLATE_BUF_SAMPLES: usize = 22_494;
const PLATE_BANDWIDTH: f32 = 0.9995;
const PLATE_DDIFF1: f32 = 0.7;
const PLATE_DDIFF2: f32 = 0.5;
const PLATE_EXC_DEPTH: usize = 16;
const PLATE_LFO_RATE: f32 = 1.0;
const PLATE_OUT_SCALE: f32 = 0.6;

/// Dattorro allpass reading at the full slice length (`slice[cursor]`).
fn plate_ap(slice: &mut [f32], c: &mut usize, x: f32, gain: f32) -> f32 {
    let len = slice.len();
    let d = slice[*c];
    let w = x - gain * d;
    slice[*c] = w;
    *c += 1;
    if *c >= len {
        *c = 0;
    }
    d + gain * w
}

/// Dattorro allpass reading `read_back` samples behind the cursor (modulated ad1).
fn plate_ap_read(slice: &mut [f32], c: &mut usize, x: f32, gain: f32, read_back: usize) -> f32 {
    let len = slice.len();
    let d = slice[(*c + len - read_back) % len];
    let w = x - gain * d;
    slice[*c] = w;
    *c += 1;
    if *c >= len {
        *c = 0;
    }
    d + gain * w
}

/// Plain delay: return the len-samples-ago output, write `x`, advance.
fn plate_delay(slice: &mut [f32], c: &mut usize, x: f32) -> f32 {
    let len = slice.len();
    let out = slice[*c];
    slice[*c] = x;
    *c += 1;
    if *c >= len {
        *c = 0;
    }
    out
}

/// Read `offset` samples behind the cursor (output tap; no state change).
#[inline]
fn plate_tap(slice: &[f32], c: usize, offset: usize) -> f32 {
    let len = slice.len();
    slice[(c + len - offset) % len]
}

/// Dattorro plate reverb: input diffusion + a figure-8 modulated-allpass tank +
/// a multi-tap output network, over one partitioned buffer. Mono in → stereo out.
#[derive(Clone, Copy)]
pub struct Dattorro {
    c: [usize; 12],   // per-element cursors
    damp_z: [f32; 2], // per-half damping LP state
    bw: f32,          // input bandwidth LP state
    lfo_phase: f32,   // excursion LFO
    size: f32,
    damp: f32,
    width: f32,
    mix: f32,
}
impl Dattorro {
    pub fn new() -> Dattorro {
        Dattorro { c: [0; 12], damp_z: [0.0; 2], bw: 0.0, lfo_phase: 0.0, size: 0.5, damp: 0.5, width: 1.0, mix: 0.5 }
    }
    pub fn set_mix(&mut self, v: f32) { self.mix = v.clamp(0.0, 1.0); }
    pub fn set_damp(&mut self, v: f32) { self.damp = v.clamp(0.0, 1.0); }
    pub fn set_size(&mut self, v: f32) { self.size = v.clamp(0.0, 1.0); }
    pub fn set_width(&mut self, v: f32) { self.width = v.clamp(0.0, 1.0); }

    pub fn process(&mut self, input: In, dt: f32, buf: &mut [f32], out_l: &mut [f32], out_r: &mut [f32]) {
        if buf.len() < PLATE_BUF_SAMPLES {
            for i in 0..out_l.len() {
                let x = input.at(i);
                out_l[i] = x;
                out_r[i] = x;
            }
            return;
        }
        // Element offsets (cumulative walk).
        let mut off = [0usize; 12];
        {
            let mut o = 0;
            for k in 0..12 {
                off[k] = o;
                o += PLATE_LEN[k];
            }
        }
        let decay = self.size * 0.4 + 0.5;
        let damp_c = 1.0 - self.damp * 0.9;
        let wet1 = self.mix * (self.width * 0.5 + 0.5);
        let wet2 = self.mix * ((1.0 - self.width) * 0.5);
        let dry = 1.0 - self.mix;
        // Convenience: a mutable slice for element k.
        macro_rules! el {
            ($k:expr) => {
                &mut buf[off[$k]..off[$k] + PLATE_LEN[$k]]
            };
        }
        for i in 0..out_l.len() {
            let x = input.at(i);
            // Input bandwidth low-pass.
            self.bw += (x - self.bw) * PLATE_BANDWIDTH;
            let mut s = self.bw;
            // 4 series input diffusers.
            for k in 0..4 {
                let g = if k < 2 { 0.75 } else { 0.625 };
                s = plate_ap(el!(k), &mut self.c[k], s, g);
            }
            // Excursion LFO → integer read-offset for the two ad1 allpasses.
            let p = self.lfo_phase + PLATE_LFO_RATE * dt;
            self.lfo_phase = p - libm::floorf(p);
            let exc = (PLATE_EXC_DEPTH as f32 * (0.5 + 0.5 * fast_sin(self.lfo_phase))) as usize;
            // Cross-feed: read both post-damping-delay (db) outputs before writes.
            let fb_l = buf[off[11] + self.c[11]] * decay; // dbR output → left half
            let fb_r = buf[off[10] + self.c[10]] * decay; // dbL output → right half
            // Left half (elements 4=ad1L, 6=daL, 8=ad2L, 10=dbL).
            let mut t = s + fb_l;
            t = plate_ap_read(el!(4), &mut self.c[4], t, -PLATE_DDIFF1, PLATE_LEN[4] - exc);
            t = plate_delay(el!(6), &mut self.c[6], t);
            self.damp_z[0] += (t - self.damp_z[0]) * damp_c;
            t = self.damp_z[0] * decay;
            t = plate_ap(el!(8), &mut self.c[8], t, PLATE_DDIFF2);
            plate_delay(el!(10), &mut self.c[10], t);
            // Right half (elements 5=ad1R, 7=daR, 9=ad2R, 11=dbR).
            let mut u = s + fb_r;
            u = plate_ap_read(el!(5), &mut self.c[5], u, -PLATE_DDIFF1, PLATE_LEN[5] - exc);
            u = plate_delay(el!(7), &mut self.c[7], u);
            self.damp_z[1] += (u - self.damp_z[1]) * damp_c;
            u = self.damp_z[1] * decay;
            u = plate_ap(el!(9), &mut self.c[9], u, PLATE_DDIFF2);
            plate_delay(el!(11), &mut self.c[11], u);
            // Output taps (read-only, after all writes/advances).
            let ta = |k: usize, o: usize| plate_tap(&buf[off[k]..off[k] + PLATE_LEN[k]], self.c[k], o);
            let yl = PLATE_OUT_SCALE
                * (ta(7, 266) + ta(7, 2974) - ta(9, 1913) + ta(11, 1996) - ta(6, 1990) - ta(8, 187) - ta(10, 1066));
            let yr = PLATE_OUT_SCALE
                * (ta(6, 353) + ta(6, 3627) - ta(8, 1228) + ta(10, 2673) - ta(7, 2111) - ta(9, 335) - ta(11, 121));
            out_l[i] = x * dry + yl * wet1 + yr * wet2;
            out_r[i] = x * dry + yr * wet1 + yl * wet2;
        }
    }
}
impl Default for Dattorro {
    fn default() -> Self {
        Self::new()
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

    fn render_hall(fdn: &mut Fdn8, buf: &mut [f32], input: &[f32]) -> (std::vec::Vec<f32>, std::vec::Vec<f32>) {
        let mut l = std::vec![0.0f32; input.len()];
        let mut r = std::vec![0.0f32; input.len()];
        fdn.process(In::A(input), 1.0 / 44_100.0, buf, &mut l, &mut r);
        (l, r)
    }

    #[test]
    fn hall_layout_sums_to_buf_samples() {
        let sum: usize = BASE_LEN.iter().map(|b| b + MOD_MARGIN).sum();
        assert_eq!(sum, HALL_BUF_SAMPLES);
    }

    #[test]
    fn fwht8_is_orthonormal() {
        // Energy preserved: ‖fwht8(v)‖ ≈ ‖v‖.
        let mut v = [0.3, -0.7, 1.1, 0.2, -0.5, 0.9, -0.1, 0.4];
        let e_in: f32 = v.iter().map(|x| x * x).sum();
        fwht8(&mut v);
        let e_out: f32 = v.iter().map(|x| x * x).sum();
        assert!((e_in - e_out).abs() < 1e-4, "not energy-preserving: {e_in} vs {e_out}");
        // All-ones → only bin 0 = √8, rest 0.
        let mut ones = [1.0f32; 8];
        fwht8(&mut ones);
        assert!((ones[0] - (8.0f32).sqrt()).abs() < 1e-4);
        assert!(ones[1..].iter().all(|x| x.abs() < 1e-4));
    }

    #[test]
    fn hall_impulse_produces_decaying_tail() {
        let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
        let mut fdn = Fdn8::new();
        fdn.set_mix(1.0);
        fdn.set_size(0.8);
        let mut input = std::vec![0.0f32; 40_000];
        input[0] = 1.0;
        let (l, r) = render_hall(&mut fdn, &mut buf, &input);
        assert!(l.iter().all(|v| v.is_finite()) && r.iter().all(|v| v.is_finite()));
        let late: f32 = l[20_000..22_000].iter().map(|v| v * v).sum();
        assert!(late > 1e-8, "tail died too fast: {late}");
        let early: f32 = l[4_000..6_000].iter().map(|v| v * v).sum();
        let later: f32 = l[30_000..32_000].iter().map(|v| v * v).sum();
        assert!(early > later, "tail not decaying: {early} → {later}");
    }

    #[test]
    fn hall_size_lengthens_tail() {
        let tail = |size: f32| -> f32 {
            let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
            let mut fdn = Fdn8::new();
            fdn.set_mix(1.0);
            fdn.set_size(size);
            let mut input = std::vec![0.0f32; 60_000];
            input[0] = 1.0;
            let (l, _r) = render_hall(&mut fdn, &mut buf, &input);
            l[50_000..52_000].iter().map(|v| v * v).sum()
        };
        assert!(tail(0.95) > tail(0.3), "bigger size should decay slower");
    }

    #[test]
    fn hall_damping_darkens_tail() {
        let hf = |damp: f32| -> f32 {
            let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
            let mut fdn = Fdn8::new();
            fdn.set_mix(1.0);
            fdn.set_size(0.85);
            fdn.set_damp(damp);
            let mut input = std::vec![0.0f32; 30_000];
            input[0] = 1.0;
            let (l, _r) = render_hall(&mut fdn, &mut buf, &input);
            l[15_000..19_000].windows(2).map(|w| (w[1] - w[0]).powi(2)).sum()
        };
        assert!(hf(0.9) < hf(0.05), "more damping → less HF in the tail");
    }

    #[test]
    fn hall_output_is_stereo() {
        let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
        let mut fdn = Fdn8::new();
        fdn.set_mix(1.0);
        let input: std::vec::Vec<f32> = (0..12_000).map(|i| (i as f32 * 0.03).sin()).collect();
        let (l, r) = render_hall(&mut fdn, &mut buf, &input);
        let diff: f32 = l.iter().zip(&r).map(|(a, b)| (a - b).abs()).sum();
        assert!(diff > 1.0, "hall should be stereo (l != r): {diff}");
    }

    #[test]
    fn hall_mix_zero_is_dry() {
        let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
        let mut fdn = Fdn8::new();
        fdn.set_mix(0.0);
        let input = std::vec![0.5f32; 256];
        let (l, r) = render_hall(&mut fdn, &mut buf, &input);
        assert!((l[100] - 0.5).abs() < 1e-4 && (r[100] - 0.5).abs() < 1e-4);
    }

    #[test]
    fn hall_short_buffer_is_dry_passthrough() {
        let mut buf = std::vec![0.0f32; 64];
        let mut fdn = Fdn8::new();
        fdn.set_mix(1.0);
        let input = std::vec![0.3f32; 32];
        let (l, r) = render_hall(&mut fdn, &mut buf, &input);
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

    proptest! {
        #![proptest_config(ProptestConfig { cases: 24, ..ProptestConfig::default() })]
        #[test]
        fn hall_is_finite_and_bounded(
            size in 0.0f32..1.0,
            damp in 0.0f32..1.0,
            width in 0.0f32..1.0,
            mix in 0.0f32..1.0,
            amp in 0.0f32..1.0,
        ) {
            let mut buf = std::vec![0.0f32; HALL_BUF_SAMPLES];
            let mut fdn = Fdn8::new();
            fdn.set_size(size); fdn.set_damp(damp); fdn.set_width(width); fdn.set_mix(mix);
            let input = std::vec![amp; 6_000];
            let (l, r) = render_hall(&mut fdn, &mut buf, &input);
            for (a, b) in l.iter().zip(&r) {
                prop_assert!(a.is_finite() && b.is_finite());
                prop_assert!(a.abs() <= 16.0 && b.abs() <= 16.0, "unbounded: {a},{b}");
            }
        }
    }

    fn render_plate(d: &mut Dattorro, buf: &mut [f32], input: &[f32]) -> (std::vec::Vec<f32>, std::vec::Vec<f32>) {
        let mut l = std::vec![0.0f32; input.len()];
        let mut r = std::vec![0.0f32; input.len()];
        d.process(In::A(input), 1.0 / 44_100.0, buf, &mut l, &mut r);
        (l, r)
    }

    #[test]
    fn plate_layout_sums_to_buf_samples() {
        let sum: usize = PLATE_LEN.iter().sum();
        assert_eq!(sum, PLATE_BUF_SAMPLES);
    }

    #[test]
    fn plate_impulse_produces_decaying_tail() {
        let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
        let mut d = Dattorro::new();
        d.set_mix(1.0);
        d.set_size(0.8);
        let mut input = std::vec![0.0f32; 40_000];
        input[0] = 1.0;
        let (l, r) = render_plate(&mut d, &mut buf, &input);
        assert!(l.iter().all(|v| v.is_finite()) && r.iter().all(|v| v.is_finite()));
        let late: f32 = l[20_000..22_000].iter().map(|v| v * v).sum();
        assert!(late > 1e-9, "tail died too fast: {late}");
        let early: f32 = l[4_000..6_000].iter().map(|v| v * v).sum();
        let later: f32 = l[30_000..32_000].iter().map(|v| v * v).sum();
        assert!(early > later, "tail not decaying: {early} → {later}");
    }

    #[test]
    fn plate_size_lengthens_tail() {
        let tail = |size: f32| -> f32 {
            let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
            let mut d = Dattorro::new();
            d.set_mix(1.0);
            d.set_size(size);
            let mut input = std::vec![0.0f32; 50_000];
            input[0] = 1.0;
            let (l, _r) = render_plate(&mut d, &mut buf, &input);
            l[40_000..42_000].iter().map(|v| v * v).sum()
        };
        assert!(tail(0.95) > tail(0.2), "bigger size should decay slower");
    }

    #[test]
    fn plate_damping_darkens_tail() {
        let hf = |damp: f32| -> f32 {
            let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
            let mut d = Dattorro::new();
            d.set_mix(1.0);
            d.set_size(0.85);
            d.set_damp(damp);
            let mut input = std::vec![0.0f32; 25_000];
            input[0] = 1.0;
            let (l, _r) = render_plate(&mut d, &mut buf, &input);
            l[12_000..16_000].windows(2).map(|w| (w[1] - w[0]).powi(2)).sum()
        };
        assert!(hf(0.9) < hf(0.05), "more damping → less HF in the tail");
    }

    #[test]
    fn plate_output_is_stereo() {
        let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
        let mut d = Dattorro::new();
        d.set_mix(1.0);
        let input: std::vec::Vec<f32> = (0..12_000).map(|i| (i as f32 * 0.03).sin()).collect();
        let (l, r) = render_plate(&mut d, &mut buf, &input);
        let diff: f32 = l.iter().zip(&r).map(|(a, b)| (a - b).abs()).sum();
        assert!(diff > 1.0, "plate should be stereo (l != r): {diff}");
    }

    #[test]
    fn plate_mix_zero_is_dry() {
        let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
        let mut d = Dattorro::new();
        d.set_mix(0.0);
        let input = std::vec![0.5f32; 256];
        let (l, r) = render_plate(&mut d, &mut buf, &input);
        assert!((l[100] - 0.5).abs() < 1e-4 && (r[100] - 0.5).abs() < 1e-4);
    }

    #[test]
    fn plate_short_buffer_is_dry_passthrough() {
        let mut buf = std::vec![0.0f32; 64];
        let mut d = Dattorro::new();
        d.set_mix(1.0);
        let input = std::vec![0.3f32; 32];
        let (l, r) = render_plate(&mut d, &mut buf, &input);
        assert!(l.iter().all(|&v| (v - 0.3).abs() < 1e-6) && r.iter().all(|&v| (v - 0.3).abs() < 1e-6));
    }

    proptest! {
        #![proptest_config(ProptestConfig { cases: 24, ..ProptestConfig::default() })]
        #[test]
        fn plate_is_finite_and_bounded(
            size in 0.0f32..1.0,
            damp in 0.0f32..1.0,
            width in 0.0f32..1.0,
            mix in 0.0f32..1.0,
            amp in 0.0f32..1.0,
        ) {
            let mut buf = std::vec![0.0f32; PLATE_BUF_SAMPLES];
            let mut d = Dattorro::new();
            d.set_size(size); d.set_damp(damp); d.set_width(width); d.set_mix(mix);
            let input = std::vec![amp; 6_000];
            let (l, r) = render_plate(&mut d, &mut buf, &input);
            for (a, b) in l.iter().zip(&r) {
                prop_assert!(a.is_finite() && b.is_finite());
                prop_assert!(a.abs() <= 32.0 && b.abs() <= 32.0, "unbounded: {a},{b}");
            }
        }
    }
}
