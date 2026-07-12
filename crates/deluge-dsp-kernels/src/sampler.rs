//! In-memory sample playback (Sa-1): a `SamplePlayer` reads a PCM buffer at a
//! playback rate (speed × semitone transpose) with 4-point Hermite interpolation,
//! one-shot or looped (with loop points), restartable via `trigger`. Mono,
//! `no_std`, no heap.

/// 4-point (Catmull-Rom) Hermite. At `frac == 0` returns `y1` exactly, so integer
/// read positions reproduce the source sample verbatim.
#[inline]
fn hermite(y0: f32, y1: f32, y2: f32, y3: f32, frac: f32) -> f32 {
    let c0 = y1;
    let c1 = 0.5 * (y2 - y0);
    let c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
    let c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
    ((c3 * frac + c2) * frac + c1) * frac + c0
}

#[derive(Clone, Copy)]
pub struct SamplePlayer {
    speed: f32,
    semitones: f32,
    loop_start: f32,
    loop_end: f32,
    loop_mode: bool,
    pos: f32,
    playing: bool,
}

impl SamplePlayer {
    pub fn new() -> SamplePlayer {
        SamplePlayer {
            speed: 1.0,
            semitones: 0.0,
            loop_start: 0.0,
            loop_end: 0.0, // set to buffer len by the graph/Wren on create
            loop_mode: false,
            pos: 0.0,
            playing: false,
        }
    }

    pub fn set_speed(&mut self, v: f32) { self.speed = v; }
    pub fn set_semitones(&mut self, v: f32) { self.semitones = v; }
    pub fn set_loop_start(&mut self, v: f32) { self.loop_start = v.max(0.0); }
    pub fn set_loop_end(&mut self, v: f32) { self.loop_end = v.max(0.0); }
    pub fn set_loop_mode(&mut self, on: bool) { self.loop_mode = on; }
    pub fn trigger(&mut self) {
        self.pos = if self.loop_mode { self.loop_start } else { 0.0 };
        self.playing = true;
    }

    pub fn process(&mut self, pcm: &[f32], dt: f32, out: &mut [f32]) {
        let _ = dt;
        let len = pcm.len();
        if len == 0 {
            for o in out.iter_mut() { *o = 0.0; }
            return;
        }
        let rate = self.speed * libm::exp2f(self.semitones / 12.0);
        // effective loop region, clamped to the buffer
        let ls = self.loop_start.max(0.0);
        let le = self.loop_end.min(len as f32);
        let loopable = self.loop_mode && le > ls + 1.0;
        for o in out.iter_mut() {
            if !self.playing {
                *o = 0.0;
                continue;
            }
            // index helper: clamp (one-shot) or wrap into [ls, le) (loop)
            let read = |idx: isize| -> f32 {
                let n = if loopable {
                    let span = (le - ls) as isize;
                    let base = ls as isize;
                    let mut k = (idx - base) % span;
                    if k < 0 { k += span; }
                    base + k
                } else {
                    idx.clamp(0, len as isize - 1)
                };
                pcm[n as usize]
            };
            let i = libm::floorf(self.pos) as isize;
            let frac = self.pos - (i as f32);
            *o = hermite(read(i - 1), read(i), read(i + 1), read(i + 2), frac);
            self.pos += rate;
            if loopable {
                if self.pos >= le { self.pos -= le - ls; }
                if self.pos < ls { self.pos += le - ls; } // guard reverse/underrun
            } else if self.pos >= len as f32 {
                self.playing = false;
            }
        }
    }
}

impl Default for SamplePlayer {
    fn default() -> Self { SamplePlayer::new() }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    fn player() -> SamplePlayer { SamplePlayer::new() }

    #[test]
    fn plays_buffer_verbatim_at_speed_1() {
        // Hermite interpolates its knots, so at integer positions (speed 1) the
        // output equals the PCM sample-for-sample.
        let pcm: [f32; 8] = core::array::from_fn(|i| (i as f32) * 0.1 - 0.3);
        let mut p = player(); // defaults: speed 1, one-shot, loop_end will be set below
        p.set_loop_end(pcm.len() as f32);
        p.trigger();
        let mut out = [0.0f32; 8];
        p.process(&pcm, 1.0 / 48_000.0, &mut out);
        for i in 0..8 { assert!((out[i] - pcm[i]).abs() < 1e-5, "sample {} verbatim", i); }
    }

    #[test]
    fn speed_2_reads_every_other() {
        let pcm: [f32; 8] = core::array::from_fn(|i| i as f32);
        let mut p = player();
        p.set_loop_end(8.0);
        p.set_speed(2.0);
        p.trigger();
        let mut out = [0.0f32; 4];
        p.process(&pcm, 1.0 / 48_000.0, &mut out);
        // pos 0,2,4,6 → pcm 0,2,4,6
        assert!((out[0] - 0.0).abs() < 1e-5 && (out[1] - 2.0).abs() < 1e-5
             && (out[2] - 4.0).abs() < 1e-5 && (out[3] - 6.0).abs() < 1e-5);
    }

    #[test]
    fn semitone_12_doubles_rate() {
        // +12 semitones == speed 2: same read pattern as speed_2.
        let pcm: [f32; 8] = core::array::from_fn(|i| i as f32);
        let mut p = player();
        p.set_loop_end(8.0);
        p.set_semitones(12.0);
        p.trigger();
        let mut out = [0.0f32; 4];
        p.process(&pcm, 1.0 / 48_000.0, &mut out);
        assert!((out[3] - 6.0).abs() < 1e-4, "+12 semis doubles rate, got {}", out[3]);
    }

    #[test]
    fn one_shot_stops_at_end_and_retriggers() {
        let pcm = [1.0f32; 4];
        let mut p = player();
        p.set_loop_end(4.0);
        p.trigger();
        let mut out = [0.0f32; 8];
        p.process(&pcm, 1.0 / 48_000.0, &mut out); // 8 out, 4-sample buffer
        assert!(out[0..4].iter().all(|&v| (v - 1.0).abs() < 1e-5), "plays the buffer");
        assert!(out[4..8].iter().all(|&v| v.abs() < 1e-6), "silence after end (one-shot)");
        // retrigger → plays again
        let mut out2 = [0.0f32; 4];
        p.trigger();
        p.process(&pcm, 1.0 / 48_000.0, &mut out2);
        assert!(out2.iter().all(|&v| (v - 1.0).abs() < 1e-5), "retrigger replays");
    }

    #[test]
    fn loop_wraps_within_region() {
        // loop the whole 4-sample buffer; render 12 samples → 3 repeats.
        let pcm: [f32; 4] = [10.0, 20.0, 30.0, 40.0];
        let mut p = player();
        p.set_loop_start(0.0);
        p.set_loop_end(4.0);
        p.set_loop_mode(true);
        p.trigger();
        let mut out = [0.0f32; 12];
        p.process(&pcm, 1.0 / 48_000.0, &mut out);
        for i in 0..12 { assert!((out[i] - pcm[i % 4]).abs() < 1e-4, "loop repeats at {}", i); }
    }
}
