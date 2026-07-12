//! In-memory sample playback (Sa-1): a `SamplePlayer` reads a PCM buffer at a
//! playback rate (speed × semitone transpose) with 4-point Hermite interpolation,
//! one-shot or looped (with loop points), restartable via `trigger`. Mono,
//! `no_std`, no heap.
//!
//! Sa-2 adds `PolySamplePlayer`: `VOICES` scalar voices sharing a note-zone
//! keymap (offset/len/low/high/root), each voice's playback rate = hz/root.

use crate::poly::VOICES;
use crate::In;

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

/// 4-point Hermite read of `pcm` at absolute fractional position `pos`, with
/// taps clamped (one-shot) or wrapped (loop) inside the window `[lo, hi)`.
#[inline]
fn hermite_read(pcm: &[f32], pos: f32, lo: isize, hi: isize, loopable: bool) -> f32 {
    let i = libm::floorf(pos) as isize;
    let frac = pos - (i as f32);
    let tap = |idx: isize| -> f32 {
        let span = hi - lo;
        let n = if loopable && span > 0 {
            let mut k = (idx - lo) % span;
            if k < 0 { k += span; }
            lo + k
        } else {
            idx.clamp(lo, hi - 1)
        };
        pcm[n as usize]
    };
    hermite(tap(i - 1), tap(i), tap(i + 1), tap(i + 2), frac)
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
        let (lo, hi) = if loopable { (ls as isize, le as isize) } else { (0, len as isize) };
        for o in out.iter_mut() {
            if !self.playing {
                *o = 0.0;
                continue;
            }
            *o = hermite_read(pcm, self.pos, lo, hi, loopable);
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

pub const MAX_ZONES: usize = 8;

#[derive(Clone, Copy)]
pub struct Zone { pub offset: u32, pub len: u32, pub low: u8, pub high: u8, pub root: u8 }
impl Zone { const fn empty() -> Zone { Zone { offset: 0, len: 0, low: 0, high: 0, root: 60 } } }

#[derive(Clone, Copy)]
struct SVoice { pos: f32, rate: f32, off: u32, len: u32, playing: bool, latch: bool }
impl SVoice { const fn new() -> SVoice { SVoice { pos: 0.0, rate: 1.0, off: 0, len: 0, playing: false, latch: false } } }

#[derive(Clone, Copy)]
pub struct PolySamplePlayer {
    voices: [SVoice; VOICES],
    zones: [Zone; MAX_ZONES],
    n_zones: usize,
    loop_mode: bool,
}

impl PolySamplePlayer {
    pub fn new() -> PolySamplePlayer {
        PolySamplePlayer { voices: [SVoice::new(); VOICES], zones: [Zone::empty(); MAX_ZONES], n_zones: 0, loop_mode: false }
    }
    pub fn set_n_zones(&mut self, n: f32) { self.n_zones = (n.max(0.0) as usize).min(MAX_ZONES); }
    pub fn set_loop_mode(&mut self, on: bool) { self.loop_mode = on; }
    /// field: 0=offset,1=len,2=low,3=high,4=root
    pub fn set_zone_field(&mut self, zone: usize, field: usize, value: f32) {
        if zone >= MAX_ZONES { return; }
        let z = &mut self.zones[zone];
        match field {
            0 => z.offset = value.max(0.0) as u32,
            1 => z.len = value.max(0.0) as u32,
            2 => z.low = value.clamp(0.0, 127.0) as u8,
            3 => z.high = value.clamp(0.0, 127.0) as u8,
            4 => z.root = value.clamp(0.0, 127.0) as u8,
            _ => {}
        }
    }
    pub fn trigger_voice(&mut self, v: usize) {
        if v < VOICES { self.voices[v] = SVoice { playing: true, latch: true, ..SVoice::new() }; }
    }

    pub fn process_voice(&mut self, v: usize, pcm: &[f32], hz: In, _dt: f32, out: &mut [f32]) {
        if v >= VOICES { return; }
        let loop_mode = self.loop_mode;
        // zone latch on first process after trigger
        if self.voices[v].latch {
            let h = hz.at(0).max(1e-6);
            let note = libm::roundf(69.0 + 12.0 * libm::log2f(h / 440.0));
            let mut found = false;
            for z in 0..self.n_zones {
                let zn = self.zones[z];
                if note >= zn.low as f32 && note <= zn.high as f32 && zn.len > 0 {
                    let root_hz = 440.0 * libm::exp2f((zn.root as f32 - 69.0) / 12.0);
                    self.voices[v].off = zn.offset;
                    self.voices[v].len = zn.len;
                    self.voices[v].rate = h / root_hz;
                    found = true;
                    break;
                }
            }
            if !found { self.voices[v].playing = false; }
            self.voices[v].latch = false;
        }
        let vc = &mut self.voices[v];
        let lo = vc.off as isize;
        let hi = (vc.off + vc.len) as isize;
        for o in out.iter_mut() {
            if !vc.playing || vc.len == 0 || (vc.off as usize + vc.len as usize) > pcm.len() {
                *o = 0.0;
                continue;
            }
            *o = hermite_read(pcm, vc.off as f32 + vc.pos, lo, hi, loop_mode);
            vc.pos += vc.rate;
            let l = vc.len as f32;
            if loop_mode {
                if vc.pos >= l { vc.pos -= l; }
            } else if vc.pos >= l {
                vc.playing = false;
            }
        }
    }
}

impl Default for PolySamplePlayer {
    fn default() -> Self { PolySamplePlayer::new() }
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

    #[test]
    fn poly_voice_pitch_is_hz_over_root() {
        // 1 zone, full range, root 60. mtof(60) → rate 1 (verbatim); mtof(72) → rate 2.
        let pcm: [f32; 8] = core::array::from_fn(|i| i as f32);
        let mtof = |n: f32| 440.0 * libm::exp2f((n - 69.0) / 12.0);
        let mut p = PolySamplePlayer::new();
        p.set_n_zones(1.0);
        p.set_zone_field(0, 0, 0.0);   // offset
        p.set_zone_field(0, 1, 8.0);   // len
        p.set_zone_field(0, 2, 0.0);   // low
        p.set_zone_field(0, 3, 127.0); // high
        p.set_zone_field(0, 4, 60.0);  // root
        // voice 0 at root pitch → rate 1 → verbatim
        p.trigger_voice(0);
        let hz0 = [mtof(60.0); 4];
        let mut o0 = [0.0f32; 4];
        p.process_voice(0, &pcm, In::A(&hz0), 1.0 / 48_000.0, &mut o0);
        assert!((o0[0] - 0.0).abs() < 1e-4 && (o0[1] - 1.0).abs() < 1e-4 && (o0[2] - 2.0).abs() < 1e-4);
        // voice 1 an octave up → rate 2 → reads pcm[0,2,4,6]
        p.trigger_voice(1);
        let hz1 = [mtof(72.0); 4];
        let mut o1 = [0.0f32; 4];
        p.process_voice(1, &pcm, In::A(&hz1), 1.0 / 48_000.0, &mut o1);
        assert!((o1[0] - 0.0).abs() < 1e-4 && (o1[1] - 2.0).abs() < 1e-4 && (o1[2] - 4.0).abs() < 1e-4);
    }

    #[test]
    fn poly_zone_selection_by_note() {
        // zone A: notes 0-59 → pcm region [0,4) content ~10; zone B: 60-127 → [4,8) ~20.
        let pcm: [f32; 8] = [10.0, 10.0, 10.0, 10.0, 20.0, 20.0, 20.0, 20.0];
        let mtof = |n: f32| 440.0 * libm::exp2f((n - 69.0) / 12.0);
        let mut p = PolySamplePlayer::new();
        p.set_n_zones(2.0);
        // zone 0
        p.set_zone_field(0, 0, 0.0); p.set_zone_field(0, 1, 4.0); p.set_zone_field(0, 2, 0.0); p.set_zone_field(0, 3, 59.0); p.set_zone_field(0, 4, 48.0);
        // zone 1
        p.set_zone_field(1, 0, 4.0); p.set_zone_field(1, 1, 4.0); p.set_zone_field(1, 2, 60.0); p.set_zone_field(1, 3, 127.0); p.set_zone_field(1, 4, 72.0);
        // note 48 (root of zone 0) → plays A content (~10) at rate 1
        p.trigger_voice(0);
        let ha = [mtof(48.0); 4]; let mut oa = [0.0f32; 4];
        p.process_voice(0, &pcm, In::A(&ha), 1.0 / 48_000.0, &mut oa);
        assert!(oa.iter().all(|&v| (v - 10.0).abs() < 1e-3), "note 48 plays zone A");
        // note 72 (root of zone 1) → plays B content (~20)
        p.trigger_voice(1);
        let hb = [mtof(72.0); 4]; let mut ob = [0.0f32; 4];
        p.process_voice(1, &pcm, In::A(&hb), 1.0 / 48_000.0, &mut ob);
        assert!(ob.iter().all(|&v| (v - 20.0).abs() < 1e-3), "note 72 plays zone B");
        // note 100 in no... (covered by zone 1). Test an unmapped note with a 1-zone map elsewhere:
    }

    #[test]
    fn poly_unmapped_note_is_silent() {
        let pcm = [1.0f32; 4];
        let mtof = |n: f32| 440.0 * libm::exp2f((n - 69.0) / 12.0);
        let mut p = PolySamplePlayer::new();
        p.set_n_zones(1.0);
        p.set_zone_field(0, 0, 0.0); p.set_zone_field(0, 1, 4.0); p.set_zone_field(0, 2, 60.0); p.set_zone_field(0, 3, 60.0); p.set_zone_field(0, 4, 60.0); // only note 60
        p.trigger_voice(0);
        let h = [mtof(48.0); 4]; let mut o = [0.0f32; 4]; // note 48, unmapped
        p.process_voice(0, &pcm, In::A(&h), 1.0 / 48_000.0, &mut o);
        assert!(o.iter().all(|&v| v.abs() < 1e-6), "unmapped note is silent");
    }
}
