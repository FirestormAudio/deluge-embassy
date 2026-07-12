# PolyStreamPlayer Kernel Implementation Plan (Sa-3b Slice 2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A pure `no_std` `PolyStreamPlayer` (VOICES-wide) that plays each voice from a moving **ring window** of a large sample — reading only the resident slice, holding position (silence) on underrun, and exposing a per-voice read-cursor for the (later) prefetch task.

**Architecture:** New kernel type in `crates/deluge-dsp-kernels/src/sampler.rs`, reusing the existing `hermite` helper. Each voice tracks an `f64` absolute `pos`. `process_voice` clamps the 4 Hermite taps to the file bounds `[0, total)` (so the very first/last samples read correctly, like one-shot Hermite), checks all clamped taps are resident in `[fill_lo, fill_hi)`, and either reads (advance `pos`) or underruns (silence, hold `pos`). Scalar — the 4-tap modulo gather doesn't vectorize.

**Tech Stack:** Rust `no_std`, `libm::floor`; the crate's `sampler.rs` conventions (`hermite`, `VOICES`, per-voice methods mirroring `PolySamplePlayer`).

## Global Constraints

- `no_std`, no heap, **no panic on ANY input** — any `fill_lo`/`fill_hi` (incl. `fill_hi < fill_lo`), `total`, `rate`, empty ring, `v >= VOICES`. Every ring index is `(t as u64 % ring.len() as u64) as usize` — **reduce modulo in `u64` BEFORE `as usize`** (the deploy target is 32-bit `usize`; see [[target-32bit-usize-overflow]]). Clamped taps are always `>= 0`.
- VOICES == 8. MIT/Apache-2.0. `pos` is `f64` (large-file precision).
- Scalar (gather not NEON-friendly, per [[prefer-neon-simd]]'s "don't force it"); tests pass BOTH configs (`cargo test -p deluge-dsp-kernels` ± `--features simd`).
- Purely additive — no existing code touched.
- Test invocation: per-crate, NEVER `--workspace`; `-- name`. LSP `armv7a … can't find crate for test` is noise.

## Interfaces (public API this slice produces — later slices consume)

```rust
pub struct PolyStreamPlayer { /* [StreamVoice; VOICES] */ }
impl PolyStreamPlayer {
    pub fn new() -> PolyStreamPlayer;
    pub fn trigger_voice(&mut self, v: usize);                 // pos=0, playing=true
    pub fn read_cursor(&self, v: usize) -> u64;                // ⌊pos⌋
    pub fn is_playing(&self, v: usize) -> bool;
    pub fn process_voice(&mut self, v: usize, ring: &[f32], fill_lo: u64, fill_hi: u64,
                         total: u64, rate: f32, out: &mut [f32]);
}
```

---

### Task 1: `PolyStreamPlayer` — ring-window read, underrun-hold, read-cursor

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/sampler.rs` (add the new kernel + tests)

**Interfaces:**
- Consumes: the existing `hermite(y0,y1,y2,y3,frac)` fn + `VOICES` (both already in `sampler.rs`).
- Produces: `PolyStreamPlayer` (see Interfaces above). Consumed by Slice 3 (graph node).

- [ ] **Step 1: Write the failing tests**

Add to `crates/deluge-dsp-kernels/src/sampler.rs`'s test module (it already has `extern crate std;` from Sa-1/Sa-2). A `fill_ring` helper writes absolute samples `[lo, hi)` into a ring so `ring[a % cap] = value(a)`:

```rust
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
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- stream_`
Expected: FAIL to compile (`PolyStreamPlayer` undefined).

- [ ] **Step 3: Implement `PolyStreamPlayer`**

Add to `crates/deluge-dsp-kernels/src/sampler.rs` (after `PolySamplePlayer`, reusing the file's `hermite` + `VOICES`):

```rust
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
        let file_hi: i64 = if total > 0 { total as i64 - 1 } else { i64::MAX };
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
```
> Every ring access `((t as u64) % cap) as usize` reduces in `u64` first (no 32-bit overflow), and `t` is clamped `>= 0` so `t as u64` is well-defined. `cap == 0` and `v >= VOICES` short-circuit to silence. `fill_hi < fill_lo` → `resident` is false for every tap → underrun (silence). No indexing can panic.

- [ ] **Step 4: Run tests, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- stream_` (+ `--features simd`). Then the FULL crate both configs. Expected: PASS — existing `sampler.rs` tests (SamplePlayer/PolySamplePlayer) unaffected (purely additive).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/sampler.rs
git commit -m "feat(kernels): PolyStreamPlayer — ring-window streaming voice, underrun-hold + read-cursor"
```

---

## Self-Review

**Spec coverage:** One task delivers the whole `PolyStreamPlayer` kernel — the ring-window model (`ring[a % cap]`, resident `[fill_lo, fill_hi)`), `f64` `pos`, edge-tap clamping to `[0, total)`, underrun→silence+hold, one-shot stop at `total`, and `new`/`trigger_voice`/`read_cursor`/`is_playing`/`process_voice`. The kernel is one cohesive unit (splitting the read from the underrun check would be artificial — a reviewer judges them together), so it's one right-sized task. Tests cover resident read (rate 1/2), underrun+seamless-resume (the core new behavior), one-shot stop, modulo wrap, per-voice independence, and no-panic on adversarial cursors. Deferred items (graph node, binding, prefetch task, loop, stereo) are out of scope per the spec.

**Placeholder scan:** All code (kernel + tests + `fill_ring` helper) is verbatim. No TBD.

**Type consistency:** `process_voice(v, ring, fill_lo: u64, fill_hi: u64, total: u64, rate: f32, out)` and the `u64` read-cursor match the Interfaces block and the spec. Reuses `hermite`/`VOICES` from `sampler.rs` (not redefined). The 32-bit-`usize`-safe modulo (`% cap` in `u64` before `as usize`) is applied at the one ring-index site.
