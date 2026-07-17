# UAC Host — Capture (Phase 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the capture half of our own USB Audio Class **host** class driver — `deluge_bsp::usb::host::uac::UacIn` — so the Deluge can record multichannel audio from a class-compliant USB audio interface, with software clock-drift correction, mirroring the `host::midi` pattern.

**Architecture:** A new `crates/deluge-bsp/src/usb/host/uac/` module: `UacIn` (descriptor match → 44.1/24 negotiation via our own control requests → isochronous IN pump), a pure engine-rate ring buffer (`ring.rs`), and a pure fractional resampler + PI drift controller (`resample.rs`). Data path: iso IN packet → 24-bit→f32 → resampler(r) → ring → `read()`; ring fill drives the PI controller that sets `r` (implicit feedback — full-duplex leaves no pipe for an explicit feedback endpoint). Everything left of the ring runs in the USB task; `read()` is a plain drain with no DSP.

**Tech Stack:** Rust (edition 2024), `embassy-usb-host` 0.1.0 (descriptor parser + `SetupPacket` + `UsbPipe`), `embassy-usb-driver` host traits, our `rza1l-hal` RUSB1 host driver, `deluge-bsp` host infra, `MockAlloc` for unit tests.

## Global Constraints

- **Design of record:** `docs/superpowers/specs/2026-07-16-usb-uac-host-design.md`. This plan is **Phase 1 (capture)**; playback is Phase 2 (`out.rs`), a separate plan. The `2026-07-17` test-firmware artifacts are superseded and folded into the eventual validation firmware.
- **Policy: 44.1 kHz / 24-bit, UAC2 only.** Reject anything else explicitly and free the address. `SAMPLE_RATE_HZ = deluge_bsp::audio_block::SAMPLE_RATE_HZ = 44_100`. 24-bit = UAC Type-I `subslot_size == 3`. UAC2 = interface protocol `0x20` (what `AudioInterfaceCollection::try_from_configuration` requires); UAC1 devices are out of scope.
- **Alloc-free.** `deluge-bsp` has no heap. Fixed-size arrays / `heapless` only. `MAX_CHANNELS = 8`.
- **Audio path never blocks on USB; USB never blocks on audio.** `read()` returns a short count on underrun, never awaits. Iso transfers have **no retries**: a failed/short iso transfer logs and continues — never tears down the stream (the ring absorbs the gap).
- **Pipe budget:** iso lives only on RUSB1 pipes 1–2 (the HAL enforces this). This plan claims one iso IN pipe + one control pipe. The `feat/usb-midi-tx-multiplex` branch pins `TX_BULK_PIPE = 1` which will collide with iso — that work is on a **different branch**, so it is out of scope here; flag it at merge time.
- **Host tests** for `deluge-bsp` run under its Linux host target:
  `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf`
  (platform code is `cfg(target_os = "none")`-gated; tests are `cfg(all(test, not(target_os = "none")))`.)
- **Device build** check: `cargo build-fw -p deluge-bsp`.
- **32-bit `usize` on device**; review offset/index math for `u32` overflow by hand.

---

### Task 1: Scaffold the `uac` module + `SampleRing`

Registers the module and lands the engine-rate ring buffer (pure, fully host-tested).

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/mod.rs` (add `pub mod uac;`)
- Create: `crates/deluge-bsp/src/usb/host/uac/mod.rs`
- Create: `crates/deluge-bsp/src/usb/host/uac/ring.rs`

**Interfaces:**
- Produces: `uac::MAX_CHANNELS: usize = 8`; `uac::ring::SampleRing` with `const fn new()`, `reset(&mut self, channels: usize)`, `push_frame(&mut self, frame: &[f32])`, `read(&mut self, out: &mut [f32]) -> usize`, `fill_frames(&self) -> usize`, `capacity_frames(&self) -> usize`, `channels(&self) -> usize`.

- [ ] **Step 1: Register the module**

In `crates/deluge-bsp/src/usb/host/mod.rs`, next to the existing `pub mod midi;` (near line 15):

```rust
pub mod midi;
pub mod uac;
```

- [ ] **Step 2: Create `uac/mod.rs` with the module skeleton**

```rust
//! USB **host**-side USB Audio Class (UAC2) capture driver.
//!
//! Records multichannel audio from a class-compliant USB audio interface.
//! Mirrors [`super::midi`]: generic over the allocator so it unit-tests against
//! [`super::mock`]. See the design of record,
//! `docs/superpowers/specs/2026-07-16-usb-uac-host-design.md`.
//!
//! Playback (iso OUT) is Phase 2 (`out.rs`) and not present yet.

pub mod resample;
pub mod ring;

/// Widest capture channel count this driver supports. Bounds the fixed-size
/// per-frame scratch and the ring; a device declaring more is rejected.
pub const MAX_CHANNELS: usize = 8;
```

- [ ] **Step 3: Write `uac/ring.rs` with failing tests**

```rust
//! Engine-rate interleaved-`f32` capture ring.
//!
//! The resampler pushes whole frames at the engine rate (44.1 kHz); the app
//! drains with [`SampleRing::read`]. Overrun drops the oldest frame (never
//! blocks the USB task); underrun is a short `read` (never blocks the app).

use super::MAX_CHANNELS;

/// Ring depth in frames. ~11.6 ms at 44.1 kHz — enough slack for the PI
/// controller to correct drift without over/underrunning under normal jitter.
pub const RING_FRAMES: usize = 512;
const RING_SAMPLES: usize = RING_FRAMES * MAX_CHANNELS;

/// Fixed-capacity interleaved-`f32` ring. Alloc-free.
pub struct SampleRing {
    buf: [f32; RING_SAMPLES],
    head: usize,  // index of the oldest stored sample
    count: usize, // stored samples
    channels: usize,
}

impl SampleRing {
    pub const fn new() -> Self {
        Self { buf: [0.0; RING_SAMPLES], head: 0, count: 0, channels: 1 }
    }

    /// Reset to empty for a stream of `channels` channels (clamped to
    /// `1..=MAX_CHANNELS`).
    pub fn reset(&mut self, channels: usize) {
        self.head = 0;
        self.count = 0;
        self.channels = channels.clamp(1, MAX_CHANNELS);
    }

    pub fn channels(&self) -> usize {
        self.channels
    }

    /// Whole frames currently buffered.
    pub fn fill_frames(&self) -> usize {
        self.count / self.channels
    }

    /// Ring depth in frames for the current channel count.
    pub fn capacity_frames(&self) -> usize {
        RING_SAMPLES / self.channels
    }

    /// Push one interleaved frame (`frame.len()` must equal `channels()`).
    /// On overrun, drops the oldest frame first — the USB task never blocks.
    pub fn push_frame(&mut self, frame: &[f32]) {
        debug_assert_eq!(frame.len(), self.channels);
        if self.count + self.channels > RING_SAMPLES {
            self.head = (self.head + self.channels) % RING_SAMPLES;
            self.count -= self.channels;
        }
        for &s in frame {
            let tail = (self.head + self.count) % RING_SAMPLES;
            self.buf[tail] = s;
            self.count += 1;
        }
    }

    /// Drain up to `out.len()` samples. Returns how many were written; a short
    /// return is an underrun (the app must tolerate it, never block).
    pub fn read(&mut self, out: &mut [f32]) -> usize {
        let n = out.len().min(self.count);
        for o in out.iter_mut().take(n) {
            *o = self.buf[self.head];
            self.head = (self.head + 1) % RING_SAMPLES;
        }
        self.count -= n;
        n
    }
}

impl Default for SampleRing {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn push_then_read_roundtrips_frames() {
        let mut r = SampleRing::new();
        r.reset(2);
        r.push_frame(&[1.0, 2.0]);
        r.push_frame(&[3.0, 4.0]);
        assert_eq!(r.fill_frames(), 2);
        let mut out = [0.0; 4];
        assert_eq!(r.read(&mut out), 4);
        assert_eq!(out, [1.0, 2.0, 3.0, 4.0]);
        assert_eq!(r.fill_frames(), 0);
    }

    #[test]
    fn read_underrun_is_short_not_blocking() {
        let mut r = SampleRing::new();
        r.reset(1);
        r.push_frame(&[9.0]);
        let mut out = [0.0; 4];
        assert_eq!(r.read(&mut out), 1, "only one sample available");
        assert_eq!(out[0], 9.0);
    }

    #[test]
    fn overrun_drops_oldest_frame() {
        let mut r = SampleRing::new();
        r.reset(2);
        let cap = r.capacity_frames();
        for i in 0..cap {
            r.push_frame(&[i as f32, i as f32]);
        }
        assert_eq!(r.fill_frames(), cap);
        // One more frame evicts the oldest (frame 0).
        r.push_frame(&[-1.0, -1.0]);
        assert_eq!(r.fill_frames(), cap);
        let mut first = [0.0; 2];
        assert_eq!(r.read(&mut first), 2);
        assert_eq!(first, [1.0, 1.0], "frame 0 was evicted, frame 1 is oldest");
    }
}
```

- [ ] **Step 4: Run the ring tests**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf uac::ring`
Expected: PASS — 3 tests.

- [ ] **Step 5: Verify the device build**

Run: `cargo build-fw -p deluge-bsp`
Expected: compiles (module registered, no platform code yet).

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/mod.rs crates/deluge-bsp/src/usb/host/uac
git commit -m "feat(deluge-bsp): host::uac scaffold + engine-rate SampleRing"
```

---

### Task 2: Fractional resampler

Linear interpolating resampler, `r` = input frames per output frame. Pure, host-tested.

**Files:**
- Create: `crates/deluge-bsp/src/usb/host/uac/resample.rs`

**Interfaces:**
- Produces: `uac::resample::Resampler` with `const fn new()`, `reset(&mut self, channels: usize)`, `feed(&mut self, cur: &[f32], r: f32, emit: impl FnMut(&[f32]))`.

- [ ] **Step 1: Write `uac/resample.rs` with the resampler and failing tests**

```rust
//! Fractional resampler + PI drift controller for USB capture.
//!
//! `r` is **input frames per output frame**. A device running fast fills the
//! ring, the PI controller raises `r`, the resampler emits fewer frames per
//! input frame, and the ring drains back to its setpoint. No feedback endpoint,
//! no rate estimation — ring fill *is* the clock error (design of record §3.4).

use super::MAX_CHANNELS;

/// Linear-interpolating variable-rate resampler over interleaved `f32` frames.
///
/// Fed one input frame at a time; emits zero or more output frames per input.
/// Holds the previous input frame so interpolation is continuous across calls.
pub struct Resampler {
    channels: usize,
    prev: [f32; MAX_CHANNELS],
    have_prev: bool,
    /// Position of the next output within `[prev, cur)`, in input-frame units.
    pos: f32,
}

impl Resampler {
    pub const fn new() -> Self {
        Self { channels: 1, prev: [0.0; MAX_CHANNELS], have_prev: false, pos: 0.0 }
    }

    pub fn reset(&mut self, channels: usize) {
        self.channels = channels.clamp(1, MAX_CHANNELS);
        self.have_prev = false;
        self.pos = 0.0;
    }

    /// Feed one interleaved input frame (`cur.len()` must be `channels`) at
    /// ratio `r` (> 0). Calls `emit` once per emitted output frame.
    pub fn feed(&mut self, cur: &[f32], r: f32, mut emit: impl FnMut(&[f32])) {
        let ch = self.channels;
        if !self.have_prev {
            self.prev[..ch].copy_from_slice(&cur[..ch]);
            self.have_prev = true;
            self.pos = 0.0;
            return;
        }
        let mut out = [0.0f32; MAX_CHANNELS];
        // Emit every output whose position lies in [prev@0.0, cur@1.0).
        while self.pos < 1.0 {
            let f = self.pos;
            for c in 0..ch {
                out[c] = self.prev[c] + (cur[c] - self.prev[c]) * f;
            }
            emit(&out[..ch]);
            self.pos += r;
        }
        self.pos -= 1.0; // cur becomes the new prev (coordinate 0.0)
        self.prev[..ch].copy_from_slice(&cur[..ch]);
    }
}

impl Default for Resampler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    use heapless::Vec;

    fn run(channels: usize, r: f32, frames: &[&[f32]]) -> Vec<[f32; MAX_CHANNELS], 64> {
        let mut rs = Resampler::new();
        rs.reset(channels);
        let mut out: Vec<[f32; MAX_CHANNELS], 64> = Vec::new();
        for fr in frames {
            rs.feed(fr, r, |o| {
                let mut a = [0.0; MAX_CHANNELS];
                a[..o.len()].copy_from_slice(o);
                out.push(a).unwrap();
            });
        }
        out
    }

    #[test]
    fn unity_ratio_is_passthrough_delayed_one_frame() {
        // r = 1.0: first frame primes, then each feed emits the previous frame.
        let out = run(1, 1.0, &[&[1.0], &[2.0], &[3.0]]);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0][0], 1.0);
        assert_eq!(out[1][0], 2.0);
    }

    #[test]
    fn downsample_two_to_one() {
        let out = run(1, 2.0, &[&[1.0], &[2.0], &[3.0], &[4.0], &[5.0]]);
        // ~one output per two inputs after priming.
        assert_eq!(out.len(), 2);
    }

    #[test]
    fn upsample_one_to_two_interpolates_midpoint() {
        // prev = 0, cur = 10, r = 0.5 -> outputs at pos 0.0 (=0) and 0.5 (=5).
        let out = run(1, 0.5, &[&[0.0], &[10.0]]);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0][0], 0.0);
        assert_eq!(out[1][0], 5.0);
    }

    #[test]
    fn preserves_channel_interleave() {
        let out = run(2, 1.0, &[&[1.0, -1.0], &[2.0, -2.0], &[3.0, -3.0]]);
        assert_eq!(out.len(), 2);
        assert_eq!(&out[0][..2], &[1.0, -1.0]);
        assert_eq!(&out[1][..2], &[2.0, -2.0]);
    }
}
```

- [ ] **Step 2: Run the resampler tests**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf uac::resample`
Expected: PASS — 4 tests.

- [ ] **Step 3: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/resample.rs
git commit -m "feat(deluge-bsp): host::uac linear fractional resampler"
```

---

### Task 3: PI drift controller + closed-loop convergence test

Adds the controller that maps ring fill → `r`, and the CI test of the real hazard: a mismatched clock must converge, not drift.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/uac/resample.rs`

**Interfaces:**
- Produces: `uac::resample::PiController` with `new(setpoint_frames: f32) -> Self`, `update(&mut self, fill_frames: f32) -> f32`.

- [ ] **Step 1: Add `PiController` to `resample.rs`**

Add below `Resampler` (before the `#[cfg(test)]` module):

```rust
/// PI controller mapping ring **fill error** to the resampler ratio `r`.
///
/// Ring fill is the clock error: too full (device fast) raises `r` so the
/// resampler emits fewer frames per input frame and the ring drains back to
/// `setpoint`. Output `r` is clamped to a narrow band around 1.0 — with the
/// 44.1/24-only policy true drift stays within a few hundred ppm.
pub struct PiController {
    setpoint: f32,
    kp: f32,
    ki: f32,
    integ: f32,
    r_min: f32,
    r_max: f32,
}

impl PiController {
    /// `setpoint_frames` is the target ring fill (aim for half the ring).
    pub fn new(setpoint_frames: f32) -> Self {
        Self {
            setpoint: setpoint_frames,
            kp: 2.0e-6,
            ki: 2.0e-8,
            integ: 0.0,
            r_min: 0.98,
            r_max: 1.02,
        }
    }

    /// Update from the current ring fill (frames); returns the new ratio `r`.
    pub fn update(&mut self, fill_frames: f32) -> f32 {
        let err = fill_frames - self.setpoint;
        self.integ += err;
        // Anti-windup: clamp the integral term's contribution to the r-band.
        let i_limit = (self.r_max - 1.0) / self.ki;
        self.integ = self.integ.clamp(-i_limit, i_limit);
        (1.0 + self.kp * err + self.ki * self.integ).clamp(self.r_min, self.r_max)
    }
}
```

- [ ] **Step 2: Add PI unit tests + the closed-loop convergence test**

Add these tests inside the existing `mod tests` in `resample.rs`:

```rust
    // --- PI controller ---

    #[test]
    fn pi_raises_r_when_ring_too_full() {
        let mut pi = PiController::new(256.0);
        let r = pi.update(300.0); // above setpoint
        assert!(r > 1.0, "device fast (ring full) must raise r, got {r}");
    }

    #[test]
    fn pi_lowers_r_when_ring_too_empty() {
        let mut pi = PiController::new(256.0);
        let r = pi.update(200.0); // below setpoint
        assert!(r < 1.0, "device slow (ring empty) must lower r, got {r}");
    }

    #[test]
    fn pi_output_is_clamped_to_band() {
        let mut pi = PiController::new(256.0);
        for _ in 0..10_000 {
            let r = pi.update(100_000.0); // absurd error, drive windup
            assert!((0.98..=1.02).contains(&r));
        }
    }

    /// The real hazard: a device clock 100 ppm fast must converge to a stable
    /// ring fill, not drift into over/underrun. Model one real-time slice per
    /// iteration: the device delivers `in_frames` at (1 + ppm)*nominal, the
    /// engine drains `nominal` output frames, and the PI steers `r`.
    #[test]
    fn converges_under_100ppm_fast_clock() {
        use super::super::ring::SampleRing;

        const PPM: f32 = 100.0e-6;
        const NOMINAL: f32 = 64.0; // engine frames drained per slice
        let setpoint = 256.0f32;

        let mut ring = SampleRing::new();
        ring.reset(1);
        // Pre-fill to the setpoint so we start balanced.
        for _ in 0..(setpoint as usize) {
            ring.push_frame(&[0.0]);
        }

        let mut rs = Resampler::new();
        rs.reset(1);
        let mut pi = PiController::new(setpoint);
        let mut r = 1.0f32;
        let mut drained_acc = 0.0f32;
        let mut in_acc = 0.0f32;
        let mut sample = 0.0f32;

        for _ in 0..20_000 {
            // Device delivers (1+ppm)*NOMINAL input frames this slice.
            in_acc += NOMINAL * (1.0 + PPM);
            while in_acc >= 1.0 {
                in_acc -= 1.0;
                sample += 1.0;
                rs.feed(&[sample], r, |o| ring.push_frame(o));
            }
            // Engine drains NOMINAL output frames.
            drained_acc += NOMINAL;
            let want = drained_acc as usize;
            drained_acc -= want as f32;
            let mut scratch = [0.0f32; 256];
            let mut left = want;
            while left > 0 {
                let n = ring.read(&mut scratch[..left.min(256)]);
                if n == 0 {
                    break; // underrun tolerated
                }
                left -= n;
            }
            r = pi.update(ring.fill_frames() as f32);
        }

        let fill = ring.fill_frames() as f32;
        assert!(
            (fill - setpoint).abs() < 64.0,
            "ring should stay near setpoint, ended at {fill}"
        );
        assert!((0.98..=1.02).contains(&r), "r must stay bounded, ended {r}");
        // Steady-state r tracks the clock offset (~1 + ppm), well inside band.
        assert!(r > 1.0, "sustained fast clock should hold r slightly above 1.0");
    }
```

- [ ] **Step 3: Run the resampler+PI tests**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf uac::resample`
Expected: PASS — 8 tests. If `converges_under_100ppm_fast_clock` fails on the `abs() < 64.0` bound, retune `kp`/`ki` (raise `kp` toward `4.0e-6` for faster correction) and re-run; the test is the tuning oracle.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/resample.rs
git commit -m "feat(deluge-bsp): host::uac PI drift controller + convergence test"
```

---

### Task 4: Extend `MockAlloc` for control transfers

The mock's `control_in`/`control_out` are silent stubs, so the negotiation path (Tasks 6) can't be tested without this. Add a scripted control-read queue and a SETUP log.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/mock.rs`

**Interfaces:**
- Produces (on `MockState`): `pub control_reads: RefCell<Vec<Vec<u8, 64>, 8>>` (responses popped by successive `control_in`), `pub setups: RefCell<Vec<[u8; 8], 16>>` (every SETUP seen, IN or OUT, in order), `pub control_out_data: RefCell<Vec<u8, 256>>` (concatenated OUT data stage bytes).

- [ ] **Step 1: Add the new fields to `MockState`**

In `mock.rs`, extend the `MockState` struct and `MockState::new()`:

```rust
pub struct MockState {
    pub script: RefCell<Script>,
    pub log: RefCell<PipeLog>,
    pub allocs: RefCell<Vec<AllocRecord, 8>>,
    pub alloc_err: RefCell<Option<HostError>>,
    /// Responses returned by successive `control_in` calls (front popped).
    pub control_reads: RefCell<Vec<Vec<u8, 64>, 8>>,
    /// Every SETUP packet seen by `control_in` / `control_out`, in order.
    pub setups: RefCell<Vec<[u8; 8], 16>>,
    /// Concatenated data-stage bytes from `control_out`.
    pub control_out_data: RefCell<Vec<u8, 256>>,
}
```

In `MockState::new()`, initialise them:

```rust
            control_reads: RefCell::new(Vec::new()),
            setups: RefCell::new(Vec::new()),
            control_out_data: RefCell::new(Vec::new()),
```

- [ ] **Step 2: Make `control_in` / `control_out` record + script**

Replace the `control_in` and `control_out` impls (currently `Ok(0)`/`Ok(())` stubs) in the `impl UsbPipe for MockPipe` block:

```rust
    async fn control_in(&mut self, setup: &[u8; 8], buf: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        self.inner.setups.borrow_mut().push(*setup).ok();
        let mut reads = self.inner.control_reads.borrow_mut();
        if reads.is_empty() {
            return Ok(0);
        }
        let data = reads.remove(0);
        let n = data.len().min(buf.len());
        buf[..n].copy_from_slice(&data[..n]);
        Ok(n)
    }

    async fn control_out(&mut self, setup: &[u8; 8], buf: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        self.inner.setups.borrow_mut().push(*setup).ok();
        self.inner
            .control_out_data
            .borrow_mut()
            .extend_from_slice(buf)
            .map_err(|_| PipeError::BufferOverflow)?;
        Ok(())
    }
```

> If `MockPipe`'s field holding the `&'static MockState` is not named `inner`, use its actual name (check the struct near `mock.rs:105`).

- [ ] **Step 3: Add a mock self-test**

Add to the `#[cfg(test)] mod tests` in `mock.rs` (create one if absent, mirroring `midi.rs`'s test module style):

```rust
#[cfg(all(test, not(target_os = "none")))]
mod control_mock_tests {
    use super::*;
    use embassy_usb_driver::host::{pipe, UsbHostAllocator, UsbPipe};
    use embassy_usb_driver::{EndpointType};
    use embassy_usb_driver::EndpointInfo;

    #[test]
    fn control_in_pops_scripted_response_and_logs_setup() {
        let state = MockState::leak();
        state
            .control_reads
            .borrow_mut()
            .push(Vec::from_slice(&[0x44, 0xAC, 0x00, 0x00]).unwrap())
            .unwrap();
        let alloc = MockAlloc::new(state);
        let mut pipe = alloc
            .alloc_pipe::<pipe::Control, pipe::InOut>(
                1,
                &EndpointInfo { addr: 0u8.into(), ep_type: EndpointType::Control, max_packet_size: 64, interval_ms: 0 },
                None,
            )
            .unwrap();

        let setup = [0xA1, 0x01, 0x00, 0x01, 0x00, 0x00, 0x04, 0x00];
        let mut buf = [0u8; 4];
        let n = embassy_futures::block_on(pipe.control_in(&setup, &mut buf)).unwrap();
        assert_eq!(n, 4);
        assert_eq!(u32::from_le_bytes(buf), 44_100);
        assert_eq!(state.setups.borrow()[0], setup);
    }

    #[test]
    fn control_out_records_setup_and_data() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let mut pipe = alloc
            .alloc_pipe::<pipe::Control, pipe::InOut>(
                1,
                &EndpointInfo { addr: 0u8.into(), ep_type: EndpointType::Control, max_packet_size: 64, interval_ms: 0 },
                None,
            )
            .unwrap();
        let setup = [0x21, 0x01, 0x00, 0x01, 0x00, 0x01, 0x04, 0x00];
        embassy_futures::block_on(pipe.control_out(&setup, &44_100u32.to_le_bytes())).unwrap();
        assert_eq!(state.setups.borrow()[0], setup);
        assert_eq!(&state.control_out_data.borrow()[..], &44_100u32.to_le_bytes());
    }
}
```

- [ ] **Step 4: Run the mock tests and confirm MIDI is unaffected**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf usb::host`
Expected: PASS — the two new control-mock tests plus all existing `midi` tests.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/mock.rs
git commit -m "test(deluge-bsp): MockAlloc control-transfer scripting + SETUP log"
```

---

### Task 5: Descriptor matching — find the capture interface

Parse the configuration into `AudioInterfaceCollection`, find the input streaming interface, enforce Type-I/24-bit, and extract everything negotiation needs. Pure; host-tested with a hand-built UAC2 descriptor.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/uac/mod.rs`

**Interfaces:**
- Produces: `uac::UacError` (enum), `uac::UacCaptureMatch { ac_interface: u8, streaming_interface: u8, alternate_setting: u8, endpoint: embassy_usb_host::descriptor::EndpointDescriptor, num_channels: u8, clock_source_id: u8 }`, `uac::find_uac_capture(cfg: &ConfigurationDescriptor) -> Result<UacCaptureMatch, UacError>`.

- [ ] **Step 1: Add imports, the error type, the match struct, and `find_uac_capture` to `uac/mod.rs`**

Append to `uac/mod.rs`:

```rust
use embassy_usb_driver::Direction;
use embassy_usb_driver::host::{HostError, PipeError};
use embassy_usb_host::descriptor::{ConfigurationDescriptor, EndpointDescriptor};
use embassy_usb_host::class::uac::descriptors::{
    AudioInterfaceCollection, FormatTypeDescriptor, TerminalDescriptor,
};

/// Errors from the host UAC capture driver.
#[derive(Debug)]
pub enum UacError {
    /// No UAC2 audio-streaming interface with an input (capture) endpoint.
    NoInputInterface,
    /// Found capture, but not Type-I 24-bit PCM (our only supported format).
    UnsupportedFormat,
    /// The device cannot run 44.1 kHz.
    UnsupportedRate,
    /// A required pipe could not be allocated.
    NoPipe(HostError),
    /// A control or data transfer failed.
    Transfer(PipeError),
    /// The device's descriptors could not be parsed as UAC2.
    BadDescriptors,
}

impl From<PipeError> for UacError {
    fn from(e: PipeError) -> Self {
        UacError::Transfer(e)
    }
}

/// A matched UAC2 capture interface and the facts needed to negotiate + stream.
#[derive(Clone, Copy, Debug)]
pub struct UacCaptureMatch {
    /// AudioControl interface number (`wIndex` low byte for clock requests).
    pub ac_interface: u8,
    /// AudioStreaming interface number (target of SET_INTERFACE).
    pub streaming_interface: u8,
    /// Alternate setting carrying the stream (SET_INTERFACE `wValue`).
    pub alternate_setting: u8,
    /// The isochronous IN data endpoint.
    pub endpoint: EndpointDescriptor,
    /// Channels the device declares (app-visible; no stereo assumption).
    pub num_channels: u8,
    /// Clock-source entity ID (target of the sampling-frequency control).
    pub clock_source_id: u8,
}

/// Find the first usable UAC2 capture interface in `cfg`.
///
/// Requires Type-I PCM with `subslot_size == 3` (24-bit). Returns
/// [`UacError::NoInputInterface`] / [`UacError::UnsupportedFormat`] otherwise.
pub fn find_uac_capture(cfg: &ConfigurationDescriptor<'_>) -> Result<UacCaptureMatch, UacError> {
    let coll =
        AudioInterfaceCollection::try_from_configuration(cfg).map_err(|_| UacError::BadDescriptors)?;
    let ac_interface = coll.control_interface.interface_descriptors[0].interface_number;

    for asi in coll.audio_streaming_interfaces.iter() {
        let ep = match asi.endpoint_descriptor {
            Some(ep) if ep.ep_dir() == Direction::In => ep,
            _ => continue,
        };
        // 24-bit Type-I PCM only. Match by reference: `FormatTypeDescriptor` is
        // not `Copy`, so matching by value would move out of the borrowed `asi`.
        match &asi.format_type_descriptor {
            Some(FormatTypeDescriptor::I(f)) if f.subslot_size == 3 => {}
            _ => return Err(UacError::UnsupportedFormat),
        }
        // The streaming alt setting carrying the endpoints (max endpoint count).
        let alt = asi
            .interface_descriptors
            .iter()
            .max_by_key(|i| i.num_endpoints)
            .ok_or(UacError::BadDescriptors)?;
        // Clock source feeding the input terminal linked to this stream.
        let clock_source_id = coll
            .control_interface
            .terminal_descriptors
            .values()
            .find_map(|t| match t {
                TerminalDescriptor::Input(_) => Some(t.clock_source_id()),
                _ => None,
            })
            .ok_or(UacError::BadDescriptors)?;

        return Ok(UacCaptureMatch {
            ac_interface,
            streaming_interface: alt.interface_number,
            alternate_setting: alt.alternate_setting,
            endpoint: ep,
            num_channels: asi.class_descriptor.num_channels,
            clock_source_id,
        });
    }
    Err(UacError::NoInputInterface)
}
```

- [ ] **Step 2: Add the matching tests with a UAC2 capture descriptor fixture**

Add a `#[cfg(all(test, not(target_os = "none")))] mod tests` to `uac/mod.rs`. The fixture assembles a minimal UAC2 mic: IAD + AudioControl (clock source + input terminal) + AudioStreaming alt 0 (zero-bandwidth) + alt 1 (Type-I 24-bit, iso IN endpoint).

```rust
#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    use heapless::Vec;

    // --- UAC2 descriptor fixture bytes ---
    // Class/subclass/protocol: AUDIO=0x01, AUDIOCONTROL=0x01, AUDIOSTREAMING=0x02,
    // protocol IP_VERSION_02_00=0x20. CS_INTERFACE=0x24. subslot_size at the
    // FORMAT_TYPE descriptor; num_channels in the AS general descriptor.

    fn uac2_mic_cfg(subslot: u8, channels: u8) -> Vec<u8, 256> {
        let mut b: Vec<u8, 256> = Vec::new();
        let mut push = |s: &[u8], b: &mut Vec<u8, 256>| b.extend_from_slice(s).unwrap();

        // IAD: first iface 0, count 2, AUDIO/undefined/UAC2.
        push(&[8, 0x0B, 0, 2, 0x01, 0x00, 0x20, 0], &mut b);
        // Std AC interface 0, alt 0, 0 eps, AUDIO/AUDIOCONTROL/UAC2.
        push(&[9, 0x04, 0, 0, 0, 0x01, 0x01, 0x20, 0], &mut b);
        // CS AC header (UAC2 CLOCK etc. are inside; length total little-endian).
        push(&[9, 0x24, 0x01, 0x00, 0x02, 30, 0, 0x00, 0x00], &mut b);
        // CS Clock Source: subtype 0x0A, clock id 0x09, attrs, controls, assoc.
        push(&[8, 0x24, 0x0A, 0x09, 0x01, 0x07, 0x00, 0x00], &mut b);
        // CS Input Terminal (subtype 0x02): term id 0x01, type MIC 0x0201,
        // assoc 0, clock source id 0x09, nrchannels, ...
        push(&[17, 0x24, 0x02, 0x01, 0x01, 0x02, 0, 0x09, channels, 0, 0, 0, 0, 0, 0, 0, 0], &mut b);
        // Std AS interface 1, alt 0 (zero bandwidth): 0 eps.
        push(&[9, 0x04, 1, 0, 0, 0x01, 0x02, 0x20, 0], &mut b);
        // Std AS interface 1, alt 1: 1 ep, AUDIO/AUDIOSTREAMING/UAC2.
        push(&[9, 0x04, 1, 1, 1, 0x01, 0x02, 0x20, 0], &mut b);
        // CS AS general (subtype 0x01): terminal link 0x01, controls, FORMAT_TYPE_I=1,
        // formats bitmap(4), nrchannels, channel config(4), name.
        push(&[16, 0x24, 0x01, 0x01, 0x00, 0x01, 0x01, 0, 0, 0, channels, 0, 0, 0, 0, 0], &mut b);
        // CS AS FORMAT_TYPE (subtype 0x02): FORMAT_TYPE_I=1, subslot, bit_res.
        push(&[6, 0x24, 0x02, 0x01, subslot, subslot * 8], &mut b);
        // Std iso IN endpoint 0x81, iso(0x01) async, mps 294, interval 4.
        push(&[7, 0x05, 0x81, 0x01, 0x26, 0x01, 0x04], &mut b);
        // CS AS iso endpoint (0x25): general, attrs, controls, lock delay units/val.
        push(&[8, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00], &mut b);

        // Configuration header: len 9, CONFIGURATION=0x02, total_len LE, 2 ifaces.
        let total = (9 + b.len()) as u16;
        let mut cfg: Vec<u8, 256> = Vec::new();
        cfg.extend_from_slice(&[
            9, 0x02, total as u8, (total >> 8) as u8, 2, 1, 0, 0x80, 50,
        ])
        .unwrap();
        cfg.extend_from_slice(&b).unwrap();
        cfg
    }

    #[test]
    fn matches_24bit_capture_interface() {
        let raw = uac2_mic_cfg(3, 2);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let m = find_uac_capture(&cfg).expect("should match a 24-bit UAC2 mic");
        assert_eq!(m.streaming_interface, 1);
        assert_eq!(m.alternate_setting, 1);
        assert_eq!(m.num_channels, 2);
        assert_eq!(m.clock_source_id, 0x09);
        assert_eq!(m.ac_interface, 0);
        assert_eq!(m.endpoint.endpoint_address, 0x81);
        assert_eq!(m.endpoint.ep_dir(), Direction::In);
    }

    #[test]
    fn rejects_non_24bit_format() {
        let raw = uac2_mic_cfg(2, 2); // 16-bit
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        assert!(matches!(find_uac_capture(&cfg), Err(UacError::UnsupportedFormat)));
    }
}
```

> The exact CS-descriptor field layouts above are UAC2 §4; if `try_from_configuration` rejects the fixture, dump the parse error and adjust the CS AC header total-length byte (offset into the AudioControl block) and the input-terminal length until it parses. The two asserts (`match`, `reject`) are the contract — keep them.

- [ ] **Step 3: Run the matching tests**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf uac::tests`
Expected: PASS — `matches_24bit_capture_interface`, `rejects_non_24bit_format`.

- [ ] **Step 4: Verify the device build**

Run: `cargo build-fw -p deluge-bsp`
Expected: compiles.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/mod.rs
git commit -m "feat(deluge-bsp): host::uac capture-interface matching (24-bit UAC2)"
```

---

### Task 6: `UacIn::try_register` — pipes + 44.1 kHz negotiation

Allocate the control + iso IN pipes, confirm/force the clock to 44,100 Hz, and SET_INTERFACE to the streaming alt. Built with our own `SetupPacket`s; tested against the extended mock.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/uac/mod.rs`

**Interfaces:**
- Consumes: `UacCaptureMatch` (Task 5), the extended `MockAlloc` (Task 4).
- Produces: `uac::UacIn<'d, A: UsbHostAllocator<'d>>`, `UacIn::try_register(alloc: &A, addr: u8, split: Option<SplitInfo>, cfg: &ConfigurationDescriptor) -> Result<Self, UacError>`, `UacIn::channels(&self) -> u8`.

- [ ] **Step 1: Add pipe/control imports and the `UacIn` struct + `try_register`**

Extend the imports at the top of `uac/mod.rs`:

```rust
use embassy_usb_driver::host::{pipe, SplitInfo, UsbHostAllocator, UsbPipe};
use embassy_usb_driver::EndpointType;
use embassy_usb_host::control::{ControlType, Recipient, RequestType, SetupPacket};
use embassy_usb_host::class::uac::codes;
use crate::audio_block::SAMPLE_RATE_HZ;
```

Define the one standard request code we issue locally (avoids a direct
`embassy-usb` dependency on the host-test target — `embassy-usb-host` is enough):

```rust
/// USB standard SET_INTERFACE request code (USB 2.0 §9.4).
const REQ_SET_INTERFACE: u8 = 11;
```

Add the driver type and constructor:

```rust
/// Sampling-frequency SET_CUR/GET_CUR wValue: SAMPLING_FREQ_CONTROL in the high
/// byte (the `codes` constant is pre-shifted), channel number 0 (master).
const SAMPLING_FREQ_WVALUE: u16 = codes::control_selector::clock_source::SAMPLING_FREQ_CONTROL;

/// A hosted UAC2 capture device.
///
/// Owns the control pipe and the isochronous IN data pipe. Generic over the
/// allocator so it unit-tests against [`super::mock`].
pub struct UacIn<'d, A: UsbHostAllocator<'d>> {
    control: A::Pipe<pipe::Control, pipe::InOut>,
    iso_in: A::Pipe<pipe::Isochronous, pipe::In>,
    num_channels: u8,
    max_packet: u16,
    _p: core::marker::PhantomData<&'d ()>,
}

impl<'d, A: UsbHostAllocator<'d>> UacIn<'d, A> {
    /// Channels this device declared. `read`'s output length should be a
    /// multiple of this.
    pub fn channels(&self) -> u8 {
        self.num_channels
    }

    /// Match, claim pipes, negotiate 44.1 kHz, and select the streaming alt.
    pub async fn try_register(
        alloc: &A,
        addr: u8,
        split: Option<SplitInfo>,
        cfg: &ConfigurationDescriptor<'_>,
    ) -> Result<Self, UacError> {
        let m = find_uac_capture(cfg)?;

        // Control pipe (EP0) + the iso IN data pipe.
        let mut control = alloc
            .alloc_pipe::<pipe::Control, pipe::InOut>(
                addr,
                &embassy_usb_driver::EndpointInfo {
                    addr: 0u8.into(),
                    ep_type: EndpointType::Control,
                    max_packet_size: 64,
                    interval_ms: 0,
                },
                split,
            )
            .map_err(UacError::NoPipe)?;
        let iso_in = alloc
            .alloc_pipe::<pipe::Isochronous, pipe::In>(addr, &m.endpoint.into(), split)
            .map_err(UacError::NoPipe)?;

        // Negotiate 44.1 kHz on the clock-source entity.
        let cur = Self::get_sampling_freq(&mut control, m.ac_interface, m.clock_source_id).await?;
        if cur != SAMPLE_RATE_HZ {
            Self::set_sampling_freq(&mut control, m.ac_interface, m.clock_source_id, SAMPLE_RATE_HZ)
                .await?;
            let now =
                Self::get_sampling_freq(&mut control, m.ac_interface, m.clock_source_id).await?;
            if now != SAMPLE_RATE_HZ {
                return Err(UacError::UnsupportedRate);
            }
        }

        // Activate the stream (alt 0 is zero-bandwidth; the real stream is 1+).
        Self::set_interface(&mut control, m.streaming_interface, m.alternate_setting).await?;

        Ok(Self {
            control,
            iso_in,
            num_channels: m.num_channels,
            max_packet: m.endpoint.max_packet_size,
            _p: core::marker::PhantomData,
        })
    }

    async fn get_sampling_freq(
        control: &mut A::Pipe<pipe::Control, pipe::InOut>,
        ac_interface: u8,
        clock_id: u8,
    ) -> Result<u32, UacError> {
        let setup = SetupPacket {
            request_type: RequestType {
                direction: Direction::In,
                control_type: ControlType::Class,
                recipient: Recipient::Interface,
            },
            request: codes::request_code::CUR,
            value: SAMPLING_FREQ_WVALUE,
            index: (clock_id as u16) << 8 | ac_interface as u16,
            length: 4,
        };
        let mut buf = [0u8; 4];
        let n = control.control_in(&setup.to_bytes(), &mut buf).await?;
        if n < 4 {
            return Err(UacError::Transfer(PipeError::BadResponse));
        }
        Ok(u32::from_le_bytes(buf))
    }

    async fn set_sampling_freq(
        control: &mut A::Pipe<pipe::Control, pipe::InOut>,
        ac_interface: u8,
        clock_id: u8,
        freq: u32,
    ) -> Result<(), UacError> {
        let setup = SetupPacket {
            request_type: RequestType {
                direction: Direction::Out,
                control_type: ControlType::Class,
                recipient: Recipient::Interface,
            },
            request: codes::request_code::CUR,
            value: SAMPLING_FREQ_WVALUE,
            index: (clock_id as u16) << 8 | ac_interface as u16,
            length: 4,
        };
        control.control_out(&setup.to_bytes(), &freq.to_le_bytes()).await?;
        Ok(())
    }

    async fn set_interface(
        control: &mut A::Pipe<pipe::Control, pipe::InOut>,
        interface: u8,
        alt: u8,
    ) -> Result<(), UacError> {
        let setup = SetupPacket {
            request_type: RequestType {
                direction: Direction::Out,
                control_type: ControlType::Standard,
                recipient: Recipient::Interface,
            },
            request: REQ_SET_INTERFACE,
            value: alt as u16,
            index: interface as u16,
            length: 0,
        };
        control.control_out(&setup.to_bytes(), &[]).await?;
        Ok(())
    }
}
```

> The `super::super::super::audio_block::SAMPLE_RATE_HZ` path walks `uac → host → usb → (crate root) audio_block`. If the path is wrong, replace with a `use crate::audio_block::SAMPLE_RATE_HZ;` at the top of the file and reference `SAMPLE_RATE_HZ` directly.

- [ ] **Step 2: Add negotiation tests (extended mock)**

Add to `uac/mod.rs`'s `mod tests`:

```rust
    use crate::usb::host::mock::{MockAlloc, MockState};
    use embassy_futures::block_on;

    #[test]
    fn register_negotiates_when_rate_already_44100() {
        let state = MockState::leak();
        // GET_CUR returns 44100 -> no SET_CUR expected.
        state
            .control_reads
            .borrow_mut()
            .push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap())
            .unwrap();
        let alloc = MockAlloc::new(state);
        let raw = uac2_mic_cfg(3, 2);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let host = block_on(UacIn::try_register(&alloc, 1, None, &cfg)).expect("register");
        assert_eq!(host.channels(), 2);
        // Two pipes claimed: control (EP0) + iso IN (0x81).
        assert_eq!(state.allocs.borrow().len(), 2);
        // Exactly one SETUP: the GET_CUR (no SET_CUR), plus the SET_INTERFACE.
        let setups = state.setups.borrow();
        // Last setup is SET_INTERFACE (bRequest 11, standard/interface/out).
        let last = setups.last().unwrap();
        assert_eq!(last[1], 11, "bRequest = SET_INTERFACE");
        assert_eq!(last[2], 1, "wValue lo = alt setting 1");
        assert_eq!(last[4], 1, "wIndex lo = streaming interface 1");
    }

    #[test]
    fn register_forces_rate_when_not_44100() {
        let state = MockState::leak();
        // GET_CUR -> 48000, then after SET_CUR the confirming GET_CUR -> 44100.
        {
            let mut r = state.control_reads.borrow_mut();
            r.push(heapless::Vec::from_slice(&48_000u32.to_le_bytes()).unwrap()).unwrap();
            r.push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap()).unwrap();
        }
        let alloc = MockAlloc::new(state);
        let raw = uac2_mic_cfg(3, 1);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let host = block_on(UacIn::try_register(&alloc, 1, None, &cfg)).expect("register");
        assert_eq!(host.channels(), 1);
        // The SET_CUR data stage carried 44100 LE.
        assert_eq!(&state.control_out_data.borrow()[..4], &44_100u32.to_le_bytes());
    }
```

- [ ] **Step 3: Run the negotiation tests**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf uac::tests`
Expected: PASS — matching + `register_negotiates_when_rate_already_44100` + `register_forces_rate_when_not_44100`.

- [ ] **Step 4: Verify the device build**

Run: `cargo build-fw -p deluge-bsp`
Expected: compiles.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/mod.rs
git commit -m "feat(deluge-bsp): host::uac UacIn::try_register (pipes + 44.1kHz negotiation)"
```

---

### Task 7: Capture pump — iso IN → 24→f32 → resampler → ring → `read`

The per-transfer pump and the app-facing drain, wiring decode + resampler + PI + ring.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/uac/mod.rs`

**Interfaces:**
- Consumes: `SampleRing`, `Resampler`, `PiController` (Tasks 1–3).
- Produces: `uac::decode_s24le(b: &[u8]) -> f32`; on `UacIn`: fields `ring: SampleRing`, `resampler: Resampler`, `pi: PiController`, `r: f32`; methods `pump_once(&mut self) -> Result<(), UacError>`, `read(&mut self, out: &mut [f32]) -> usize`, `fill_frames(&self) -> usize`.

- [ ] **Step 1: Add the decode helper + pump/read to `UacIn`**

Add the free function to `uac/mod.rs`:

```rust
use self::resample::{PiController, Resampler};
use self::ring::SampleRing;

/// Decode one signed 24-bit little-endian sample (`b.len() == 3`) to `f32` in
/// `[-1.0, 1.0)`.
pub fn decode_s24le(b: &[u8]) -> f32 {
    let raw = (b[0] as i32) | ((b[1] as i32) << 8) | ((b[2] as i32) << 16);
    let signed = (raw << 8) >> 8; // sign-extend 24 -> 32
    signed as f32 / 8_388_608.0 // 2^23
}
```

Add these fields to the `UacIn` struct definition (Task 6):

```rust
    ring: SampleRing,
    resampler: Resampler,
    pi: PiController,
    r: f32,
```

Initialise them in `try_register`'s `Ok(Self { ... })` (setpoint = half the ring, sized for the channel count):

```rust
        let mut ring = SampleRing::new();
        ring.reset(m.num_channels as usize);
        let mut resampler = Resampler::new();
        resampler.reset(m.num_channels as usize);
        let setpoint = (ring.capacity_frames() / 2) as f32;
        Ok(Self {
            control,
            iso_in,
            num_channels: m.num_channels,
            max_packet: m.endpoint.max_packet_size,
            ring,
            resampler,
            pi: PiController::new(setpoint),
            r: 1.0,
            _p: core::marker::PhantomData,
        })
```

Add the pump + drain methods to `impl UacIn`:

```rust
    /// Do one isochronous IN transfer: decode → resample → ring, then update
    /// the drift ratio from the new ring fill.
    ///
    /// Iso has no retries: a failed or empty transfer logs and returns `Ok`
    /// (the ring absorbs the gap). Only a channel/format invariant break is an
    /// error.
    pub async fn pump_once(&mut self) -> Result<(), UacError> {
        let ch = self.num_channels as usize;
        let mut buf = [0u8; 1024]; // >= any HS iso mps
        let cap = self.max_packet as usize;
        let n = match self.iso_in.request_in(&mut buf[..cap]).await {
            Ok(n) => n,
            Err(_e) => {
                // Lost packet: ring absorbs it. Do not tear down the stream.
                self.r = self.pi.update(self.ring.fill_frames() as f32);
                return Ok(());
            }
        };
        let frame_bytes = ch * 3;
        let mut frame = [0.0f32; MAX_CHANNELS];
        for chunk in buf[..n].chunks_exact(frame_bytes) {
            for c in 0..ch {
                frame[c] = decode_s24le(&chunk[c * 3..c * 3 + 3]);
            }
            let ring = &mut self.ring;
            self.resampler.feed(&frame[..ch], self.r, |o| ring.push_frame(o));
        }
        self.r = self.pi.update(self.ring.fill_frames() as f32);
        Ok(())
    }

    /// Drain up to `out.len()` samples of captured interleaved `f32`. Short
    /// return = underrun; never blocks. `out.len()` should be a multiple of
    /// `channels()`.
    pub fn read(&mut self, out: &mut [f32]) -> usize {
        self.ring.read(out)
    }

    /// Frames currently buffered (for diagnostics / the validation firmware).
    pub fn fill_frames(&self) -> usize {
        self.ring.fill_frames()
    }
```

- [ ] **Step 2: Add decode + pump tests**

Add to `uac/mod.rs`'s `mod tests`:

```rust
    #[test]
    fn decode_s24le_endpoints() {
        assert_eq!(decode_s24le(&[0, 0, 0]), 0.0);
        assert_eq!(decode_s24le(&[0, 0, 0x40]), 0.5); // 0x400000 / 2^23
        assert_eq!(decode_s24le(&[0, 0, 0x80]), -1.0); // 0x800000 sign-extended
    }

    #[test]
    fn pump_decodes_iso_packet_into_ring() {
        let state = MockState::leak();
        state
            .control_reads
            .borrow_mut()
            .push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap())
            .unwrap();
        // One iso IN packet: two mono 24-bit frames, values +0.5 and -1.0.
        let mut pkt = heapless::Vec::<u8, 64>::new();
        pkt.extend_from_slice(&[0, 0, 0x40, 0, 0, 0x80]).unwrap();
        state.script.borrow_mut().reads.push(pkt).unwrap();

        let alloc = MockAlloc::new(state);
        let raw = uac2_mic_cfg(3, 1);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = block_on(UacIn::try_register(&alloc, 1, None, &cfg)).unwrap();

        block_on(host.pump_once()).expect("pump");
        // At r≈1.0 the resampler emits ~one frame per input frame after priming;
        // at least one decoded sample must be readable.
        let mut out = [0.0f32; 4];
        let got = host.read(&mut out);
        assert!(got >= 1, "expected decoded samples in the ring, got {got}");
        assert!(out[..got].iter().any(|&s| s < 0.0 || s > 0.0), "non-silent");
    }

    #[test]
    fn pump_tolerates_transfer_error() {
        let state = MockState::leak();
        state
            .control_reads
            .borrow_mut()
            .push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap())
            .unwrap();
        *state.script.borrow_mut() = crate::usb::host::mock::Script {
            reads: Default::default(),
            read_err: Some(embassy_usb_driver::host::PipeError::Timeout),
        };
        let alloc = MockAlloc::new(state);
        let raw = uac2_mic_cfg(3, 1);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = block_on(UacIn::try_register(&alloc, 1, None, &cfg)).unwrap();

        // A failed iso transfer must NOT error out — the stream survives.
        block_on(host.pump_once()).expect("iso error is absorbed, not fatal");
    }
```

> The negotiation `control_reads` are re-pushed in these tests because `try_register` consumes one GET_CUR response; keep the mono (`channels = 1`) fixture so a 3-byte frame is one sample.

- [ ] **Step 3: Run the pump tests**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf uac`
Expected: PASS — all `uac::ring`, `uac::resample`, and `uac::tests` (matching, negotiation, decode, pump).

- [ ] **Step 4: Verify the device build**

Run: `cargo build-fw -p deluge-bsp`
Expected: compiles.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/mod.rs
git commit -m "feat(deluge-bsp): host::uac capture pump + 24-bit decode + read()"
```

---

### Task 8: Supervisor integration — `bind_uac` + capture task + app read API

Wire `UacIn` into the host supervisor and expose a cross-task capture read. Device-only (`cfg(target_os = "none")`); gated by the device build, not host tests.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/uac/mod.rs` (app-facing static + accessors)
- Modify: `crates/deluge-bsp/src/usb/host/mod.rs` (`bind_uac`, capture task, supervisor arm)

**Interfaces:**
- Consumes: `UacIn` (Tasks 6–7); `HostAlloc = BusHandle<'static, Rusb1Allocator>`, the `usb_host_supervisor` / `bind_midi` patterns.
- Produces: `uac::capture_read(out: &mut [f32]) -> usize`, `uac::capture_channels() -> u8` (app-facing, cross-task).

- [ ] **Step 1: Add the cross-task capture bridge to `uac/mod.rs`**

Append (device-gated, since it uses a `CriticalSectionRawMutex` static that only runs on target):

```rust
#[cfg(target_os = "none")]
pub(crate) mod shared {
    use super::ring::SampleRing;
    use super::MAX_CHANNELS;
    use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
    use core::cell::RefCell;

    struct Capture {
        ring: SampleRing,
        channels: u8,
        active: bool,
    }

    static CAPTURE: Mutex<CriticalSectionRawMutex, RefCell<Capture>> =
        Mutex::new(RefCell::new(Capture {
            ring: SampleRing::new(),
            channels: 0,
            active: false,
        }));

    /// Called by the capture task when a device is registered.
    pub(crate) fn begin(channels: u8) {
        CAPTURE.lock(|c| {
            let mut c = c.borrow_mut();
            c.ring.reset(channels.clamp(1, MAX_CHANNELS as u8) as usize);
            c.channels = channels;
            c.active = true;
        });
    }

    /// Called by the capture task on detach.
    pub(crate) fn end() {
        CAPTURE.lock(|c| {
            let mut c = c.borrow_mut();
            c.active = false;
            c.channels = 0;
            c.ring.reset(1);
        });
    }

    /// Push captured interleaved frames from the task.
    pub(crate) fn push(samples: &[f32], channels: usize) {
        CAPTURE.lock(|c| {
            let mut c = c.borrow_mut();
            for frame in samples.chunks_exact(channels) {
                c.ring.push_frame(frame);
            }
        });
    }

    /// Frames currently buffered — the PI controller's single source of truth.
    pub(crate) fn fill_frames() -> usize {
        CAPTURE.lock(|c| c.borrow().ring.fill_frames())
    }

    /// App-facing drain. Short return = underrun; never blocks.
    pub fn capture_read(out: &mut [f32]) -> usize {
        CAPTURE.lock(|c| c.borrow_mut().ring.read(out))
    }

    /// Channels the hosted device declared, or 0 if none.
    pub fn capture_channels() -> u8 {
        CAPTURE.lock(|c| c.borrow().channels)
    }
}

#[cfg(target_os = "none")]
pub use shared::{capture_channels, capture_read};
```

To feed the static from the pump, add a device-only variant that emits into `shared::push` instead of the internal ring. Add to `impl UacIn` (device-gated):

```rust
    /// Like [`pump_once`], but publishes to the process-wide capture bridge so
    /// a different task (the engine) can `capture_read`. Used by the host task.
    #[cfg(target_os = "none")]
    pub async fn pump_once_shared(&mut self) -> Result<(), UacError> {
        let ch = self.num_channels as usize;
        let mut buf = [0u8; 1024];
        let cap = self.max_packet as usize;
        let n = match self.iso_in.request_in(&mut buf[..cap]).await {
            Ok(n) => n,
            Err(_e) => {
                // Lost packet: the shared ring absorbs it; keep streaming.
                self.r = self.pi.update(shared::fill_frames() as f32);
                return Ok(());
            }
        };
        let frame_bytes = ch * 3;
        let mut frame = [0.0f32; MAX_CHANNELS];
        let mut out = [0.0f32; MAX_CHANNELS];
        for chunk in buf[..n].chunks_exact(frame_bytes) {
            for c in 0..ch {
                frame[c] = decode_s24le(&chunk[c * 3..c * 3 + 3]);
            }
            let mut emit = |o: &[f32]| {
                out[..o.len()].copy_from_slice(o);
                shared::push(&out[..o.len()], ch);
            };
            self.resampler.feed(&frame[..ch], self.r, &mut emit);
        }
        // Single source of truth: the shared ring the app actually drains.
        self.r = self.pi.update(shared::fill_frames() as f32);
        Ok(())
    }
```

> The device path publishes into the shared ring and drives the PI off
> `shared::fill_frames()` — one source of truth. `self.ring` stays unused on the
> device path (it exists for the host-tested `pump_once` in Task 7, which is the
> correctness reference); leaving it costs one unused `SampleRing` and is fine
> for Phase 1. Do **not** dead-code-strip it — `pump_once` needs it.

- [ ] **Step 2: Add `bind_uac` + the capture task + a supervisor arm in `host/mod.rs`**

In the `#[cfg(target_os = "none")] mod runtime` block, mirror `bind_midi` / `midi_device_task`:

```rust
    fn bind_uac(
        handle: &HostAlloc,
        spawner: Spawner,
        dev_info: &EnumerationInfo,
        cfg: &ConfigurationDescriptor<'_>,
    ) -> bool {
        use crate::usb::host::uac::UacIn;
        match embassy_futures::block_on(UacIn::try_register(
            handle,
            dev_info.device_address,
            dev_info.split(),
            cfg,
        )) {
            Ok(host) => {
                super::uac::shared::begin(host.channels());
                if spawner.spawn(uac_capture_task(host)).is_ok() {
                    true
                } else {
                    super::uac::shared::end();
                    false
                }
            }
            Err(_) => false,
        }
    }

    #[embassy_executor::task]
    async fn uac_capture_task(mut host: crate::usb::host::uac::UacIn<'static, HostAlloc>) {
        loop {
            if host.pump_once_shared().await.is_err() {
                break; // only a fatal invariant break exits; iso errors are absorbed
            }
        }
        super::uac::shared::end();
        // `host` drops here → pipes freed → address reclaimed (MIDI precedent).
    }
```

In `usb_host_supervisor`, add a `bind_uac` attempt in the class-binding fallthrough (after the hub check, alongside `bind_midi`) — try MIDI first, then UAC, so a composite audio+MIDI device gets both considered:

```rust
                if !bind_midi(&handle, spawner, &dev_info, &cfg) {
                    bind_uac(&handle, spawner, &dev_info, &cfg);
                }
```

> Match the exact surrounding structure at `usb_host_supervisor` (the subagent noted it near `host/mod.rs:339`). `block_on` inside `bind_uac` is acceptable because `try_register`'s control transfers are short and the supervisor is already the enumeration owner (mirrors how `bind_midi` registers synchronously).

- [ ] **Step 3: Verify the device build links**

Run: `cargo build-fw -p deluge-bsp`
Expected: compiles and links — `bind_uac`, `uac_capture_task`, and the shared bridge are wired.

- [ ] **Step 4: Verify all host tests still pass**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf`
Expected: PASS — all `usb::host::midi`, `usb::host::mock`, and `usb::host::uac` tests. (The `shared` bridge and task are `cfg(target_os = "none")`, so they don't affect host tests.)

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/mod.rs crates/deluge-bsp/src/usb/host/mod.rs
git commit -m "feat(deluge-bsp): host::uac supervisor bind + capture task + app read API"
```

---

## Notes for the implementer

- **Order matters:** Tasks 1–3 (ring, resampler, PI) are pure and carry the design's real risk — do them first and let the convergence test tune the gains. Task 4 (mock) unblocks Tasks 6–7. Task 5 (matching) is pure descriptor work. Task 8 is device-only glue.
- **What's deliberately deferred to Phase 2:** playback (`out.rs`, iso OUT paced by capture arrivals), `GET_RANGE`-based rate validation (Phase 1 uses GET_CUR/SET_CUR/confirm — simpler and avoids the `Layout3ParameterBlock` wire-size caveat the reference has), NEON resampler inner loop (scalar is inaudible at r≈1.0 per the spec; the oracle tests already guard a future SIMD swap — see [[prefer-neon-simd]]), and cubic interpolation (linear is fine at r≈1.0).
- **Cross-branch hazard:** `feat/usb-midi-tx-multiplex` pins `TX_BULK_PIPE = 1`, which collides with the iso pipes (1–2). It is not on this branch; at merge time it must move to pipe 3 before multiplexed MIDI and audio can coexist (spec §3.1).
- **UAC2 only:** `AudioInterfaceCollection::try_from_configuration` requires interface protocol `0x20`. A UAC1 device is skipped (returns `BadDescriptors`); that's the intended Phase-1 boundary.
