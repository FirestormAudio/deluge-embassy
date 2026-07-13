//! Poly granular synthesis (Sa-4): each voice plays a cloud of overlapping
//! Hann-windowed grains from an in-RAM sample, note pitch transposing the grains.
//! Scalar (per-grain gathers). `no_std`, no heap, no panic.

use crate::poly::VOICES;
use crate::sampler::hermite_read;
use crate::In;

const MAX_GRAINS: usize = 8;

/// `trigger_voice` seeds `spawn_accum` just under 1.0 so the cloud's first
/// grain fires within a few samples of the trigger rather than waiting a
/// full `1/density` seconds of silence.
const TRIGGER_HEADSTART: f32 = 1.0 - 1e-4;

/// Hann window over the grain's normalized phase `[0,1]`.
#[inline]
pub(crate) fn hann(phase: f32) -> f32 {
    0.5 - 0.5 * libm::cosf(core::f32::consts::TAU * phase)
}

#[inline]
fn xorshift32(s: &mut u32) -> u32 {
    let mut r = *s;
    r ^= r << 13;
    r ^= r >> 17;
    r ^= r << 5;
    *s = r;
    r
}

#[derive(Clone, Copy)]
struct Grain { active: bool, pos: f32, rate: f32, phase: f32, phase_inc: f32 }
impl Grain {
    const fn new() -> Grain { Grain { active: false, pos: 0.0, rate: 1.0, phase: 0.0, phase_inc: 1.0 } }
}

#[derive(Clone, Copy)]
struct GrainCloud {
    grains: [Grain; MAX_GRAINS],
    rng: u32,
    spawn_accum: f32,
    playing: bool,
    position: f32, // 0..1 fraction of the buffer
    size_ms: f32,
    density: f32,  // grains/sec
    spray: f32,    // 0..1 position-jitter fraction
    root: f32,     // MIDI note played at grain rate 1.0
}
impl GrainCloud {
    const fn new() -> GrainCloud {
        GrainCloud {
            grains: [Grain::new(); MAX_GRAINS], rng: 1, spawn_accum: 0.0, playing: false,
            position: 0.0, size_ms: 50.0, density: 20.0, spray: 0.0, root: 60.0,
        }
    }
}

#[derive(Clone, Copy)]
pub struct PolyGranular { voices: [GrainCloud; VOICES] }

impl PolyGranular {
    pub fn new() -> PolyGranular { PolyGranular { voices: [GrainCloud::new(); VOICES] } }

    pub fn set_root(&mut self, note: f32) { for c in &mut self.voices { c.root = note; } }
    pub fn set_position(&mut self, p: f32) { for c in &mut self.voices { c.position = p; } }
    pub fn set_size(&mut self, ms: f32) { for c in &mut self.voices { c.size_ms = ms; } }
    pub fn set_density(&mut self, d: f32) { for c in &mut self.voices { c.density = d; } }
    pub fn set_spray(&mut self, s: f32) { for c in &mut self.voices { c.spray = s; } }

    /// (Re)start voice `v`'s cloud: clear grains, reseed the (decorrelated) RNG.
    ///
    /// `spawn_accum` seeds just under the 1.0 spawn threshold (not exactly 0.0)
    /// so the cloud's first grain fires on (or within a few samples of) the very
    /// first processed sample whenever `density > 0`, rather than waiting a full
    /// `1/density` seconds of silence after every trigger. `density == 0` still
    /// never crosses the threshold (the accumulator only advances by
    /// `density * dt`), so silence is preserved exactly.
    pub fn trigger_voice(&mut self, v: usize) {
        if v < VOICES {
            let c = &mut self.voices[v];
            c.grains = [Grain::new(); MAX_GRAINS];
            c.spawn_accum = TRIGGER_HEADSTART;
            c.rng = 0x2545_F491 ^ (v as u32 + 1).wrapping_mul(0x9E37_79B9);
            c.playing = true;
        }
    }

    pub fn process_voice(&mut self, v: usize, pcm: &[f32], hz: In, dt: f32, out: &mut [f32]) {
        if v >= VOICES {
            for o in out.iter_mut() { *o = 0.0; }
            return;
        }
        let len = pcm.len();
        let c = &mut self.voices[v];
        if !c.playing || len == 0 {
            for o in out.iter_mut() { *o = 0.0; }
            return;
        }
        let flen = len as f32;
        // Guard against NaN/Inf params (e.g. from unclamped automation/MIDI):
        // `.max(0.0)` already turns NaN density into 0.0 (f32::max returns the
        // non-NaN operand), but `.clamp()` alone lets a NaN operand pass through
        // unchanged, so spray/position/size/root need an explicit finite check.
        let density = c.density.max(0.0);
        let spray = if c.spray.is_finite() { c.spray.clamp(0.0, 1.0) } else { 0.0 };
        let position = if c.position.is_finite() { c.position } else { 0.0 };
        let size_ms = if c.size_ms.is_finite() { c.size_ms } else { 0.1 };
        let root = if c.root.is_finite() { c.root } else { 60.0 };
        let size_s = (size_ms.max(0.1)) / 1000.0;
        let root_hz = 440.0 * libm::exp2f((root - 69.0) / 12.0);
        for (i, o) in out.iter_mut().enumerate() {
            let rate = hz.at(i).max(1e-6) / root_hz;
            // schedule new grains
            c.spawn_accum += density * dt;
            while c.spawn_accum >= 1.0 {
                c.spawn_accum -= 1.0;
                let jit = (xorshift32(&mut c.rng) as f32 / u32::MAX as f32) * 2.0 - 1.0; // [-1,1)
                let start = (position.clamp(0.0, 1.0) * flen + jit * spray * flen)
                    .clamp(0.0, (flen - 1.0).max(0.0));
                let size_samples = (size_s / dt).max(2.0);
                match c.grains.iter_mut().find(|g| !g.active) {
                    Some(g) => *g = Grain { active: true, pos: start, rate, phase: 0.0, phase_inc: 1.0 / size_samples },
                    // All MAX_GRAINS busy — no free slot can appear later in this
                    // pass (grains only free up in the mix loop below, which runs
                    // after scheduling), so further iterations are guaranteed
                    // no-ops. Bail out rather than spinning up to `density * dt`
                    // times (unbounded on a bad/unclamped density value).
                    None => break,
                }
            }
            // mix + advance active grains
            let mut sum = 0.0f32;
            for g in c.grains.iter_mut() {
                if !g.active { continue; }
                sum += hann(g.phase) * hermite_read(pcm, g.pos, 0, len as isize, false);
                g.pos += g.rate;
                g.phase += g.phase_inc;
                if g.phase >= 1.0 { g.active = false; }
            }
            *o = sum;
        }
    }
}
impl Default for PolyGranular { fn default() -> Self { PolyGranular::new() } }

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;

    fn ramp(n: usize) -> std::vec::Vec<f32> { (0..n).map(|i| i as f32).collect() }

    #[test]
    fn density_zero_is_silent() {
        let pcm = ramp(1000);
        let mut g = PolyGranular::new();
        g.set_density(0.0);
        g.trigger_voice(0);
        let mut out = [1.0f32; 64];
        g.process_voice(0, &pcm, In::K(261.6256), 1.0 / 48000.0, &mut out);
        assert!(out.iter().all(|&s| s == 0.0), "no grains → silence");
    }

    #[test]
    fn triggered_cloud_renders_bounded_nonsilent() {
        let pcm = ramp(4000);
        let mut g = PolyGranular::new();
        g.set_position(0.25);
        g.set_size(20.0);
        g.set_density(100.0);
        g.trigger_voice(0);
        let mut out = [0.0f32; 256];
        g.process_voice(0, &pcm, In::K(261.6256), 1.0 / 48000.0, &mut out);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 5000.0), "finite/bounded");
        assert!(out.iter().any(|&s| s != 0.0), "cloud sounds");
    }

    #[test]
    fn hann_window_shape() {
        assert!((hann(0.0)).abs() < 1e-6);           // fades in from 0
        assert!((hann(0.5) - 1.0).abs() < 1e-6);      // peak at center
        assert!((hann(1.0)).abs() < 1e-6);            // fades to 0
    }

    #[test]
    fn spray_zero_is_deterministic_at_position() {
        // spray=0 → every grain starts exactly at position*len (no jitter).
        // Compare two fresh clouds: identical output.
        let pcm = ramp(2000);
        let run = || {
            let mut g = PolyGranular::new();
            g.set_position(0.5); g.set_size(10.0); g.set_density(200.0); g.set_spray(0.0);
            g.trigger_voice(0);
            let mut out = [0.0f32; 128];
            g.process_voice(0, &pcm, In::K(261.6256), 1.0/48000.0, &mut out);
            out
        };
        assert_eq!(run(), run(), "spray=0 is deterministic");
    }

    #[test]
    fn rate_octave_up_reads_twice_as_fast() {
        // A voice one octave above root advances grain pos at ~2x. Assert the
        // cloud is non-silent at both pitches and differs (higher pitch shifts content).
        let pcm = ramp(4000);
        let render = |hz: f32| {
            let mut g = PolyGranular::new();
            g.set_position(0.1); g.set_size(30.0); g.set_density(100.0);
            g.trigger_voice(0);
            let mut out = [0.0f32; 128];
            g.process_voice(0, &pcm, In::K(hz), 1.0/48000.0, &mut out);
            out
        };
        let root = render(261.6256);   // note 60
        let octave = render(523.2512); // note 72 → rate 2
        assert!(octave.iter().any(|&s| s != 0.0));
        assert!(root.iter().zip(octave.iter()).any(|(a, b)| (a - b).abs() > 1e-4), "pitch changes the cloud");
    }

    #[test]
    fn no_panic_on_adversarial() {
        let mut g = PolyGranular::new();
        g.trigger_voice(0);
        let mut out = [0.0f32; 32];
        let empty: [f32; 0] = [];
        g.process_voice(0, &empty, In::K(440.0), 1.0/48000.0, &mut out); // empty buffer
        g.set_density(1e9); g.set_size(-5.0); g.set_spray(9.0);
        g.process_voice(0, &ramp(10), In::K(440.0), 1.0/48000.0, &mut out); // absurd params, tiny buffer
        g.process_voice(99, &ramp(10), In::K(440.0), 1.0/48000.0, &mut out); // out-of-range voice
        assert!(out.iter().all(|s| s.is_finite()));
    }

    #[test]
    fn nan_params_produce_finite_output() {
        // A tainted (NaN/Inf) automation/MIDI value must not propagate into the
        // render: spray/position/size/root are finite-guarded before use.
        let pcm = ramp(1000);
        let mut g = PolyGranular::new();
        g.set_spray(f32::NAN);
        g.set_position(f32::NAN);
        g.set_size(f32::NAN);
        g.set_root(f32::INFINITY);
        g.trigger_voice(0);
        let mut out = [0.0f32; 64];
        g.process_voice(0, &pcm, In::K(261.6256), 1.0 / 48000.0, &mut out);
        assert!(out.iter().all(|s| s.is_finite()), "NaN/Inf params must not taint output");
    }

    #[test]
    fn extreme_density_bounds_spawn_loop_over_many_blocks() {
        // Finding 1 regression guard: an unclamped density (e.g. runaway
        // automation) must not blow up per-sample scheduling work, and
        // spawn_accum must not become non-finite even after sustained extreme
        // density across many blocks (the loop now bails as soon as all
        // MAX_GRAINS slots are busy instead of spinning ~density*dt times).
        let pcm = ramp(4000);
        let mut g = PolyGranular::new();
        g.set_density(1e9);
        g.set_position(0.25);
        g.set_size(20.0);
        g.trigger_voice(0);
        let mut out = [0.0f32; 128];
        // Bound is generous (MAX_GRAINS fully-overlapping grains at the loudest
        // sample in the buffer, Hann-windowed to <=1.0 each) — this test is
        // about finiteness/boundedness under sustained extreme density, not a
        // tight amplitude check (see `triggered_cloud_renders_bounded_nonsilent`
        // for that at a sane density).
        let bound = MAX_GRAINS as f32 * 4001.0;
        for _ in 0..200 {
            g.process_voice(0, &pcm, In::K(261.6256), 1.0 / 48000.0, &mut out);
            assert!(out.iter().all(|s| s.is_finite() && s.abs() <= bound));
        }
    }
}
