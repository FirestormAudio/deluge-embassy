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
/// taps clamped (one-shot) or wrapped (loop) inside the window `[lo, lo+span)`.
#[inline]
fn hermite_read(pcm: &[f32], pos: f32, lo: isize, span: isize, loopable: bool) -> f32 {
    let i = libm::floorf(pos) as isize;
    let frac = pos - (i as f32);
    let tap = |idx: isize| -> f32 {
        let n = if loopable && span > 0 {
            let mut k = (idx - lo) % span;
            if k < 0 { k += span; }
            lo + k
        } else {
            idx.clamp(lo, lo + span - 1)
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
        let (lo, span) = if loopable { (ls as isize, (le - ls) as isize) } else { (0, len as isize) };
        for o in out.iter_mut() {
            if !self.playing {
                *o = 0.0;
                continue;
            }
            *o = hermite_read(pcm, self.pos, lo, span, loopable);
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
        let span = vc.len as isize;
        for o in out.iter_mut() {
            if !vc.playing || vc.len == 0 || (vc.off as usize + vc.len as usize) > pcm.len() {
                *o = 0.0;
                continue;
            }
            *o = hermite_read(pcm, vc.off as f32 + vc.pos, lo, span, loop_mode);
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

/// One streaming voice: an absolute `f64` playback position into a large sample
/// that is only partially resident (in a ring). `f64` because a streamed file can
/// run for minutes — past ~16M samples an `f32` position loses sample precision.
#[derive(Clone, Copy)]
struct StreamVoice {
    pos: f64,
    playing: bool,
}
impl StreamVoice {
    fn new() -> StreamVoice { StreamVoice { pos: 0.0, playing: false } }
}

/// Poly (VOICES-wide) streaming sample player: each voice reads from a moving
/// ring window `[fill_lo, fill_hi)` of a large sample (owned/filled by the
/// prefetch task — this kernel is a pure consumer). Underrun (a needed tap not
/// yet resident) → silence + hold position (seamless resume). One-shot; scalar
/// (the 4-tap modulo gather doesn't vectorize). `no_std`, no heap, no panic.
#[derive(Clone, Copy)]
pub struct PolyStreamPlayer {
    voices: [StreamVoice; VOICES],
}

impl PolyStreamPlayer {
    pub fn new() -> PolyStreamPlayer {
        PolyStreamPlayer { voices: [StreamVoice::new(); VOICES] }
    }

    /// (Re)start voice `v` from the beginning of its sample.
    pub fn trigger_voice(&mut self, v: usize) {
        if v < VOICES {
            self.voices[v].pos = 0.0;
            self.voices[v].playing = true;
        }
    }

    /// The voice's current integer read position — how far playback has consumed.
    /// The prefetch task trails this to advance `fill_lo` and target `fill_hi`.
    pub fn read_cursor(&self, v: usize) -> u64 {
        if v < VOICES {
            let p = self.voices[v].pos;
            if p > 0.0 { libm::floor(p) as u64 } else { 0 }
        } else {
            0
        }
    }

    pub fn is_playing(&self, v: usize) -> bool {
        v < VOICES && self.voices[v].playing
    }

    /// Render one block for voice `v` from its resident ring window.
    /// `ring`: this voice's window buffer (len = ring capacity, sample `a` at
    /// `a % len`). `fill_lo..fill_hi`: absolute indices currently resident.
    /// `total`: full sample length (`0` = unbounded). `rate`: samples/output-sample.
    pub fn process_voice(&mut self, v: usize, ring: &[f32], fill_lo: u64, fill_hi: u64,
                         total: u64, rate: f32, out: &mut [f32]) {
        if v >= VOICES {
            for o in out.iter_mut() { *o = 0.0; }
            return;
        }
        let cap = ring.len() as u64;
        // last valid absolute sample index (for edge-tap clamping, like one-shot Hermite)
        let file_hi: i64 = if total == 0 { i64::MAX } else { (total - 1).min(i64::MAX as u64) as i64 };
        let voice = &mut self.voices[v];
        for o in out.iter_mut() {
            if !voice.playing || cap == 0 {
                *o = 0.0;
                continue;
            }
            if total > 0 && voice.pos >= total as f64 {
                voice.playing = false;
                *o = 0.0;
                continue;
            }
            let ip = libm::floor(voice.pos);
            let base = ip as i64;
            let frac = (voice.pos - ip) as f32;
            // clamp each tap to the file bounds [0, file_hi] (so the first/last
            // samples read correctly), then require every clamped tap resident.
            let clamp = |a: i64| -> i64 {
                if a < 0 { 0 } else if a > file_hi { file_hi } else { a }
            };
            let (t0, t1, t2, t3) = (clamp(base - 1), clamp(base), clamp(base + 1), clamp(base + 2));
            let resident = |t: i64| -> bool { (t as u64) >= fill_lo && (t as u64) < fill_hi };
            if !(resident(t0) && resident(t1) && resident(t2) && resident(t3)) {
                // UNDERRUN: prefetch hasn't caught up — silence, HOLD pos.
                *o = 0.0;
                continue;
            }
            // reduce modulo in u64 BEFORE `as usize` (32-bit usize safety)
            let rd = |t: i64| -> f32 { ring[((t as u64) % cap) as usize] };
            *o = hermite(rd(t0), rd(t1), rd(t2), rd(t3), frac);
            voice.pos += rate as f64;
        }
    }
}
impl Default for PolyStreamPlayer {
    fn default() -> Self { PolyStreamPlayer::new() }
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

    // helper: fill ring so absolute sample `a` (lo<=a<hi) holds `a as f32`.
    fn fill_ring(ring: &mut [f32], lo: u64, hi: u64) {
        let cap = ring.len() as u64;
        for a in lo..hi {
            ring[(a % cap) as usize] = a as f32;
        }
    }

    #[test]
    fn stream_resident_read_reproduces_samples() {
        let mut ring = [0.0f32; 128];
        let total = 100u64;
        fill_ring(&mut ring, 0, total);
        let mut p = PolyStreamPlayer::new();
        p.trigger_voice(0);
        let mut out = [0.0f32; 16];
        p.process_voice(0, &ring, 0, total, total, 1.0, &mut out);
        // rate 1.0, integer pos → Hermite returns the tap verbatim: out[i] == i.
        for i in 0..16 { assert_eq!(out[i], i as f32, "sample {}", i); }
        assert_eq!(p.read_cursor(0), 16);
    }

    #[test]
    fn stream_rate_two_reads_every_other() {
        let mut ring = [0.0f32; 128];
        let total = 100u64;
        fill_ring(&mut ring, 0, total);
        let mut p = PolyStreamPlayer::new();
        p.trigger_voice(0);
        let mut out = [0.0f32; 8];
        p.process_voice(0, &ring, 0, total, total, 2.0, &mut out);
        for i in 0..8 { assert_eq!(out[i], (2 * i) as f32, "sample {}", i); }
    }

    #[test]
    fn stream_underrun_holds_then_resumes() {
        let mut ring = [0.0f32; 128];
        let total = 100u64;
        fill_ring(&mut ring, 0, 5); // only [0,5) resident
        let mut p = PolyStreamPlayer::new();
        p.trigger_voice(0);
        let mut out = [0.0f32; 8];
        p.process_voice(0, &ring, 0, 5, total, 1.0, &mut out);
        // reads pos 0,1,2 (taps within [0,5)); pos 3 needs tap 5 → underrun → silence, hold.
        assert_eq!(out[0], 0.0);
        assert_eq!(out[1], 1.0);
        assert_eq!(out[2], 2.0);
        assert!(out[3..].iter().all(|&s| s == 0.0), "underrun → silence");
        assert_eq!(p.read_cursor(0), 3, "pos held at 3 (not advanced through underrun)");
        // prefetch catches up: extend the window, resume seamlessly.
        fill_ring(&mut ring, 5, total);
        let mut out2 = [0.0f32; 4];
        p.process_voice(0, &ring, 0, total, total, 1.0, &mut out2);
        assert_eq!(out2[0], 3.0, "resumes at the exact held pos");
        assert_eq!(out2[1], 4.0);
    }

    #[test]
    fn stream_one_shot_stops_at_total() {
        let mut ring = [0.0f32; 128];
        let total = 10u64;
        fill_ring(&mut ring, 0, total);
        let mut p = PolyStreamPlayer::new();
        p.trigger_voice(0);
        let mut out = [0.0f32; 16];
        p.process_voice(0, &ring, 0, total, total, 1.0, &mut out);
        assert!(!p.is_playing(0), "stopped at end of file");
        assert!(out[10..].iter().all(|&s| s == 0.0), "silence past the end");
    }

    #[test]
    fn stream_wraparound_ring() {
        // Slide a small (cap=8) window forward so a read crosses the modulo boundary.
        let cap = 8usize;
        let mut ring = [0.0f32; 8];
        let total = 100u64;
        let mut p = PolyStreamPlayer::new();
        p.trigger_voice(0);
        // Window 1: [0,8) resident. Play pos 0..=5 (pos 6 would need tap 8 → underrun).
        fill_ring(&mut ring, 0, 8);
        let mut out1 = [0.0f32; 6];
        p.process_voice(0, &ring, 0, 8, total, 1.0, &mut out1);
        for i in 0..6 { assert_eq!(out1[i], i as f32); }
        assert_eq!(p.read_cursor(0), 6);
        // Slide window forward to [4,12): filling [8,12) overwrites ring[0..4] (8→ring[0],
        // 9→ring[1], 10→ring[2], 11→ring[3]) — so reads now WRAP across the cap boundary.
        fill_ring(&mut ring, 8, 12);
        let mut out2 = [0.0f32; 4];
        p.process_voice(0, &ring, 4, 12, total, 1.0, &mut out2);
        // pos 6: taps [5,6,7,8] — tap 8 reads ring[8 % 8 = 0] = 8.0 (the wrap). Hermite@0 → 6.
        assert_eq!(out2[0], 6.0);
        assert_eq!(out2[1], 7.0);
        assert_eq!(out2[2], 8.0, "wrapped read (sample 8 at ring[0]) correct");
        assert_eq!(out2[3], 9.0);
        let _ = cap;
    }

    #[test]
    fn stream_per_voice_independent() {
        let mut ring = [0.0f32; 128];
        let total = 100u64;
        fill_ring(&mut ring, 0, total);
        let mut p = PolyStreamPlayer::new();
        p.trigger_voice(0);
        // voice 1 NOT triggered → silent, independent of voice 0.
        let mut out0 = [0.0f32; 8];
        let mut out1 = [0.0f32; 8];
        p.process_voice(0, &ring, 0, total, total, 1.0, &mut out0);
        p.process_voice(1, &ring, 0, total, total, 1.0, &mut out1);
        assert_eq!(out0[4], 4.0);
        assert!(out1.iter().all(|&s| s == 0.0), "untriggered voice 1 is silent");
        assert!(!p.is_playing(1));
    }

    #[test]
    fn stream_no_panic_on_adversarial() {
        let ring = [0.0f32; 16];
        let empty: [f32; 0] = [];
        let mut p = PolyStreamPlayer::new();
        p.trigger_voice(0);
        let mut out = [0.0f32; 8];
        // empty ring → silence, no panic
        p.process_voice(0, &empty, 0, 100, 100, 1.0, &mut out);
        assert!(out.iter().all(|&s| s == 0.0));
        // fill_hi < fill_lo, huge total/cursors, v >= VOICES → no panic
        p.process_voice(0, &ring, 50, 10, u64::MAX, 1.0, &mut out);
        p.process_voice(0, &ring, u64::MAX, u64::MAX, u64::MAX, 3.5, &mut out);
        p.process_voice(99, &ring, 0, 16, 16, 1.0, &mut out);
        assert_eq!(p.read_cursor(99), 0);
        assert!(!p.is_playing(99));
    }

    #[test]
    fn stream_no_panic_huge_total() {
        // Regression: `total` near/at the u64 boundary used to compute
        // `file_hi` via `total as i64 - 1`, which for total == 2^63
        // reinterprets to i64::MIN and panics on `- 1` (overflow-checked
        // build), and for total in (2^63, u64::MAX] silently produced a
        // NEGATIVE file_hi. Use a RESIDENT window (fill_lo=0, fill_hi=16,
        // non-empty ring, v=0 triggered) so the clamp/resident checks in
        // process_voice actually reach the file_hi computation — unlike the
        // adversarial test above, which used fill_lo==fill_hi and so
        // short-circuited before file_hi could matter.
        let mut ring = [0.0f32; 128];
        fill_ring(&mut ring, 0, 16);
        let mut out = [0.0f32; 8];

        // total == 2^63: `total as i64` == i64::MIN; `- 1` used to overflow-panic.
        let mut p1 = PolyStreamPlayer::new();
        p1.trigger_voice(0);
        p1.process_voice(0, &ring, 0, 16, 1u64 << 63, 1.0, &mut out);
        assert_eq!(out[0], 0.0, "reads verbatim, no panic, for total = 2^63");

        // total == u64::MAX: `total as i64` is negative; `file_hi` used to go negative
        // and every clamped tap would fail residency (or worse). Now saturates to
        // i64::MAX, so normal resident reads still work.
        let mut p2 = PolyStreamPlayer::new();
        p2.trigger_voice(0);
        p2.process_voice(0, &ring, 0, 16, u64::MAX, 1.0, &mut out);
        assert_eq!(out[0], 0.0, "reads verbatim, no panic, for total = u64::MAX");
        assert_eq!(out[1], 1.0);
    }
}
