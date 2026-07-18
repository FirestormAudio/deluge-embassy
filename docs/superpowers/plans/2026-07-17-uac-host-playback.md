# UAC Host — Playback (Phase 2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the host UAC driver to full-duplex — stream engine audio out to the same UAC2 interface Phase 1 captures from, at 44.1 kHz / 24-bit, paced by the capture clock (implicit feedback).

**Architecture:** A new `out.rs` (playback ring the app fills + an output-driven "pull" resampler engine→device + `encode_s24le` + the iso-OUT send) plus extensions to `mod.rs`: `find_uac_playback` matching, `UacIn` evolves into a full-duplex `Uac` owning control + iso IN + iso OUT, one pump driving both directions, and a `playback_write` app API. Playback reuses the capture PI's drift ratio `r` — one device clock, no second controller. A capture-only device gracefully degrades to Phase-1 behavior.

**Tech Stack:** Rust (edition 2024), `embassy-usb-host` 0.1.0, `embassy-usb-driver` host traits, `rza1l-hal` RUSB1 host driver, `deluge-bsp` host infra, `MockAlloc`.

## Global Constraints

- **Design of record:** `docs/superpowers/specs/2026-07-17-uac-host-playback-design.md` (Phase 2), which builds on `2026-07-16-usb-uac-host-design.md` §3.5 and the merged Phase 1 capture driver (`deluge_bsp::usb::host::uac`).
- **Full-duplex only.** Playback runs only alongside an active capture stream (implicit feedback). Playback-only USB DACs are out of scope. A device with a capture interface but no playback interface runs capture-only (Phase-1 behavior preserved).
- **44.1 kHz / 24-bit / UAC2.** Playback wire format = Type-I `subslot_size == 3`, signed 24-bit little-endian, **full-scale** (real engine audio, not the −6 dBFS test tone). Playback channel count is independent of capture.
- **One clock.** The capture PI's ratio `self.r` (input-device-frames per output-engine-frame ≈ device_rate/engine_rate) is the sole clock authority. Playback resamples engine→device at **step = 1.0 / self.r** (engine-frames per device-frame). No playback controller in Phase 2.
- **Pacing.** Per pump cycle, playback emits the SAME frame count the capture packet delivered this device microframe (one shared device clock). A missed capture packet (`Ok(0)`) emits 0 playback frames that cycle.
- **Never block; no retries.** `playback_write` short-returns on a full ring (never blocks the app). Iso OUT has no retries: a failed transfer logs + continues (a click, not a teardown); detach still keys on the shared sustained-`BadResponse` threshold. Playback **underrun → silence** (zeros) for missing frames, never a stall.
- **Pipe budget:** both iso pipes now permanently claimed (IN + OUT); control on pipe 0. The cross-branch `TX_BULK_PIPE = 1` collision (`feat/usb-midi-tx-multiplex`, not on this branch) is a hard merge gate — flag, do not fix here.
- **Alloc-free.** Fixed arrays / `heapless`. `MAX_CHANNELS = 8`.
- **Host tests:** `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf`. **Device build:** `cargo build-fw -p deluge-bsp`. 32-bit `usize` on device — review index math by hand.

---

### Task 1: `out.rs` — `encode_s24le` + output-driven playback resampler

Pure, host-tested playback primitives.

**Files:**
- Create: `crates/deluge-bsp/src/usb/host/uac/out.rs`
- Modify: `crates/deluge-bsp/src/usb/host/uac/mod.rs` (add `pub mod out;`)

**Interfaces:**
- Produces: `uac::out::encode_s24le(x: f32, out: &mut [u8])` (writes 3 bytes); `uac::out::PlaybackResampler` with `const fn new()`, `reset(&mut self, channels: usize)`, `produce(&mut self, ring: &mut SampleRing, step: f32, frames: usize, emit: impl FnMut(&[f32]))`.

- [ ] **Step 1: Register the module** — in `uac/mod.rs`, next to `pub mod ring;` / `pub mod resample;`:

```rust
pub mod out;
```

- [ ] **Step 2: Write `uac/out.rs` with the code and failing tests**

```rust
//! USB host UAC **playback** primitives: `f32`→24-bit encode and an
//! output-driven (pull) resampler that turns engine-rate frames into
//! device-rate frames. Pure `core` math, host-tested. See the Phase-2 design.

use super::ring::SampleRing;
use super::MAX_CHANNELS;

/// 2^23 — full-scale for signed 24-bit PCM.
const FULL_SCALE_24: f32 = 8_388_608.0;

/// Encode one `f32` sample in `[-1.0, 1.0)` to signed 24-bit little-endian
/// (writes 3 bytes to `out`). Full-scale; clamps to the valid 24-bit range.
#[inline]
pub fn encode_s24le(x: f32, out: &mut [u8]) {
    let i = (x * FULL_SCALE_24).clamp(-FULL_SCALE_24, FULL_SCALE_24 - 1.0) as i32;
    out[0] = i as u8;
    out[1] = (i >> 8) as u8;
    out[2] = (i >> 16) as u8;
}

/// Pull one whole frame (`ch` samples) out of `ring` into `out`. Returns
/// `false` (leaving `out` untouched) if a full frame isn't available.
fn pull_frame(ring: &mut SampleRing, ch: usize, out: &mut [f32]) -> bool {
    if ring.fill_frames() >= 1 {
        ring.read(&mut out[..ch]);
        true
    } else {
        false
    }
}

/// Output-driven linear resampler: engine-rate frames (from a ring the app
/// fills) → exactly `frames` device-rate frames. Emits silence for any frame
/// the ring can't supply (underrun never stalls). Holds `prev`/`cur` so
/// interpolation is continuous across `produce` calls.
pub struct PlaybackResampler {
    channels: usize,
    prev: [f32; MAX_CHANNELS],
    cur: [f32; MAX_CHANNELS],
    primed: bool,
    /// Fractional position within `[prev, cur)`, in engine-frame units.
    pos: f32,
}

impl PlaybackResampler {
    pub const fn new() -> Self {
        Self {
            channels: 1,
            prev: [0.0; MAX_CHANNELS],
            cur: [0.0; MAX_CHANNELS],
            primed: false,
            pos: 0.0,
        }
    }

    pub fn reset(&mut self, channels: usize) {
        self.channels = channels.clamp(1, MAX_CHANNELS);
        self.prev = [0.0; MAX_CHANNELS];
        self.cur = [0.0; MAX_CHANNELS];
        self.primed = false;
        self.pos = 0.0;
    }

    /// Produce `frames` output frames, advancing `step` engine-frames per output
    /// frame (`step = 1.0 / capture_ratio`). Calls `emit` once per output frame
    /// with an interleaved slice of length `channels`.
    pub fn produce(
        &mut self,
        ring: &mut SampleRing,
        step: f32,
        frames: usize,
        mut emit: impl FnMut(&[f32]),
    ) {
        let ch = self.channels;
        let mut out = [0.0f32; MAX_CHANNELS];
        for _ in 0..frames {
            if !self.primed {
                if !pull_frame(ring, ch, &mut self.prev) {
                    // Nothing buffered yet: emit silence, stay unprimed.
                    out[..ch].fill(0.0);
                    emit(&out[..ch]);
                    continue;
                }
                if !pull_frame(ring, ch, &mut self.cur) {
                    self.cur[..ch].copy_from_slice(&self.prev[..ch]);
                }
                self.primed = true;
                self.pos = 0.0;
            }
            while self.pos >= 1.0 {
                self.pos -= 1.0;
                self.prev[..ch].copy_from_slice(&self.cur[..ch]);
                if !pull_frame(ring, ch, &mut self.cur) {
                    self.cur[..ch].fill(0.0); // underrun → silence
                }
            }
            let f = self.pos;
            for c in 0..ch {
                out[c] = self.prev[c] + (self.cur[c] - self.prev[c]) * f;
            }
            emit(&out[..ch]);
            self.pos += step;
        }
    }
}

impl Default for PlaybackResampler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    use crate::usb::host::uac::decode_s24le;
    use heapless::Vec;

    #[test]
    fn encode_s24le_endpoints() {
        let mut b = [0u8; 3];
        encode_s24le(0.0, &mut b);
        assert_eq!(b, [0, 0, 0]);
        encode_s24le(0.5, &mut b);
        assert_eq!(b, [0x00, 0x00, 0x40]); // 0.5 * 2^23 = 0x400000
        encode_s24le(-1.0, &mut b);
        assert_eq!(b, [0x00, 0x00, 0x80]); // -2^23 = 0xFF800000 -> low 3 bytes
    }

    #[test]
    fn encode_decode_round_trips() {
        for &x in &[0.0f32, 0.25, -0.5, 0.9] {
            let mut b = [0u8; 3];
            encode_s24le(x, &mut b);
            let y = decode_s24le(&b);
            assert!((x - y).abs() < 1.0 / FULL_SCALE_24 * 2.0, "x={x} y={y}");
        }
    }

    #[test]
    fn unity_step_passes_engine_frames_through() {
        // step = 1.0: one engine frame consumed per output frame.
        let mut ring = SampleRing::new();
        ring.reset(1);
        for v in [1.0, 2.0, 3.0, 4.0] {
            ring.push_frame(&[v]);
        }
        let mut pr = PlaybackResampler::new();
        pr.reset(1);
        let mut got: Vec<f32, 8> = Vec::new();
        pr.produce(&mut ring, 1.0, 3, |o| got.push(o[0]).unwrap());
        // Primed with prev=1,cur=2; first output at pos 0 = prev = 1.0, then
        // pos steps by 1 each frame advancing prev<-cur and pulling the next.
        assert_eq!(got.len(), 3);
        assert_eq!(got[0], 1.0);
        assert_eq!(got[1], 2.0);
        assert_eq!(got[2], 3.0);
    }

    #[test]
    fn underrun_emits_silence() {
        let mut ring = SampleRing::new();
        ring.reset(2);
        // Only one frame available; ask for 4 stereo frames.
        ring.push_frame(&[0.5, -0.5]);
        let mut pr = PlaybackResampler::new();
        pr.reset(2);
        let mut frames: Vec<[f32; 2], 8> = Vec::new();
        pr.produce(&mut ring, 1.0, 4, |o| frames.push([o[0], o[1]]).unwrap());
        assert_eq!(frames.len(), 4);
        // Later frames must be silence, never a panic or stale garbage.
        assert_eq!(frames[3], [0.0, 0.0]);
    }

    #[test]
    fn produces_exact_frame_count_and_channel_width() {
        let mut ring = SampleRing::new();
        ring.reset(2);
        for _ in 0..16 {
            ring.push_frame(&[0.1, 0.2]);
        }
        let mut pr = PlaybackResampler::new();
        pr.reset(2);
        let mut n = 0usize;
        pr.produce(&mut ring, 1.0, 5, |o| {
            assert_eq!(o.len(), 2);
            n += 1;
        });
        assert_eq!(n, 5);
    }
}
```

> `decode_s24le` is `pub` in `uac/mod.rs` (Phase 1); the round-trip test imports it. If its path differs, adjust the `use`.

- [ ] **Step 3: Run the tests**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf uac::out`
Expected: PASS — 5 tests.

- [ ] **Step 4: Device build**

Run: `cargo build-fw -p deluge-bsp`
Expected: compiles.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/out.rs crates/deluge-bsp/src/usb/host/uac/mod.rs
git commit -m "feat(deluge-bsp): host::uac playback primitives (encode_s24le + pull resampler)"
```

---

### Task 2: `find_uac_playback` — match the OUT streaming interface

Descriptor matching for the playback direction, with a full-duplex test fixture.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/uac/mod.rs`

**Interfaces:**
- Consumes: the Phase-1 `UacCaptureMatch`/`UacError`/`MAX_CHANNELS`, and the `uac2_mic_cfg`/`uac2_mic_cfg_mps` test fixtures.
- Produces: `uac::UacPlaybackMatch { streaming_interface: u8, alternate_setting: u8, endpoint: EndpointDescriptor, num_channels: u8 }`; `uac::find_uac_playback(cfg: &ConfigurationDescriptor) -> Option<UacPlaybackMatch>`.

- [ ] **Step 1: Add `UacPlaybackMatch` + `find_uac_playback`**

Mirror `find_uac_capture`, but select `Direction::Out` and return `None` (not `Err`) when no playback interface exists — playback is optional (capture-only devices are valid). Reuse the existing `AudioInterfaceCollection` parse. Add to `uac/mod.rs`:

```rust
/// A matched UAC2 playback (host→device) streaming interface.
#[derive(Clone, Copy, Debug)]
pub struct UacPlaybackMatch {
    pub streaming_interface: u8,
    pub alternate_setting: u8,
    pub endpoint: EndpointDescriptor,
    pub num_channels: u8,
}

/// Find the first usable UAC2 playback interface (Direction::Out, Type-I
/// 24-bit) in `cfg`. Returns `None` if the device has no playback interface
/// (capture-only devices are valid) or it isn't 24-bit Type-I within bounds.
pub fn find_uac_playback(cfg: &ConfigurationDescriptor<'_>) -> Option<UacPlaybackMatch> {
    let coll = AudioInterfaceCollection::try_from_configuration(cfg).ok()?;
    for asi in coll.audio_streaming_interfaces.iter() {
        let ep = match asi.endpoint_descriptor {
            Some(ep) if ep.ep_dir() == Direction::Out => ep,
            _ => continue,
        };
        match &asi.format_type_descriptor {
            Some(FormatTypeDescriptor::I(f)) if f.subslot_size == 3 => {}
            _ => continue,
        }
        let channels = asi.class_descriptor.num_channels;
        if channels == 0
            || channels as usize > MAX_CHANNELS
            || ep.max_packet_size as usize > 1024
        {
            continue;
        }
        let alt = asi
            .interface_descriptors
            .iter()
            .max_by_key(|i| i.num_endpoints)?;
        return Some(UacPlaybackMatch {
            streaming_interface: alt.interface_number,
            alternate_setting: alt.alternate_setting,
            endpoint: ep,
            num_channels: channels,
        });
    }
    None
}
```

- [ ] **Step 2: Extend the test fixture to a full-duplex device + add tests**

The Phase-1 `uac2_mic_cfg_mps(subslot, channels, mps)` builds a capture-only device. Add a full-duplex variant that appends a second AudioStreaming interface (interface 2, alt 0 + alt 1) with a Direction::Out endpoint (address `0x02`), and a matching CS-AS-general + FORMAT_TYPE + std OUT endpoint + CS endpoint — mirroring the existing IN interface's descriptor block but with `bInterfaceNumber = 2`, endpoint address `0x02` (OUT), and the playback channel count. Bump the config header's `bNumInterfaces` accordingly and recompute `wTotalLength`. Then:

```rust
    #[test]
    fn matches_full_duplex_playback_interface() {
        // 2-ch capture + 2-ch playback, 24-bit.
        let raw = uac2_full_duplex_cfg(3, 2, 2);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let p = find_uac_playback(&cfg).expect("playback interface should match");
        assert_eq!(p.num_channels, 2);
        assert_eq!(p.endpoint.ep_dir(), Direction::Out);
        assert_eq!(p.endpoint.endpoint_address, 0x02);
        // Capture still matches independently.
        assert!(find_uac_capture(&cfg).is_ok());
    }

    #[test]
    fn capture_only_device_has_no_playback() {
        let raw = uac2_mic_cfg(3, 2); // Phase-1 capture-only fixture
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        assert!(find_uac_playback(&cfg).is_none());
    }
```

Build `uac2_full_duplex_cfg(subslot, cap_channels, play_channels)` by factoring the existing IN-interface descriptor emission so the OUT interface reuses it with interface number 2 / endpoint `0x02` / `play_channels`. If `AudioInterfaceCollection::try_from_configuration` rejects the two-streaming-interface fixture, dump the parse error and adjust descriptor lengths / the AC-header total until it parses — the contract is the two assertions (a full-duplex device yields both a capture and a playback match; a capture-only device yields `None` for playback). Do NOT weaken those.

- [ ] **Step 3: Run the tests + device build**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf uac::tests` then `cargo build-fw -p deluge-bsp`
Expected: PASS; compiles.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/mod.rs
git commit -m "feat(deluge-bsp): host::uac find_uac_playback (full-duplex OUT interface)"
```

---

### Task 3: `Uac` full-duplex driver — allocate iso OUT + negotiate playback

Evolve `UacIn` into `Uac`, allocating the iso OUT pipe and issuing the OUT `SET_INTERFACE` when a playback interface matched.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/uac/mod.rs`
- Modify: `crates/deluge-bsp/src/usb/host/mod.rs` (rename references in `bind_uac`/`uac_capture_task`)

**Interfaces:**
- Consumes: `find_uac_playback`/`UacPlaybackMatch` (Task 2), `PlaybackResampler` (Task 1), the Phase-1 `try_register` negotiation.
- Produces: `uac::Uac<'d, A>` (renamed from `UacIn`) with the Phase-1 capture members plus `iso_out: Option<A::Pipe<pipe::Isochronous, pipe::Out>>`, `playback: Option<PlaybackState>` (playback ring + `PlaybackResampler` + `play_channels` + `play_max_packet`); `Uac::playback_channels(&self) -> u8` (0 if none).

- [ ] **Step 1: Rename `UacIn` → `Uac`**

Rename the struct `UacIn` to `Uac` and update all references (its `impl` blocks, `bind_uac`/`uac_capture_task` in `host/mod.rs`, and the `#[embassy_executor::task]` param type). This is mechanical; the build is the gate. Keep `try_register`/`pump_once`/`pump_once_shared`/`read`/`channels`/`fill_frames` names.

- [ ] **Step 2: Add the playback state + fields**

Add near the driver:

```rust
use self::out::PlaybackResampler;

/// Playback (host→device) state, present only for a full-duplex device.
struct PlaybackState {
    ring: SampleRing,
    resampler: PlaybackResampler,
    channels: u8,
    max_packet: u16,
}
```

Add to the `Uac` struct: `iso_out: Option<A::Pipe<pipe::Isochronous, pipe::Out>>,` and `playback: Option<PlaybackState>,`. Add:

```rust
    /// Playback channels the device declared, or 0 if capture-only.
    pub fn playback_channels(&self) -> u8 {
        self.playback.as_ref().map(|p| p.channels).unwrap_or(0)
    }
```

- [ ] **Step 3: Extend `try_register` to open playback (optional)**

After the Phase-1 capture setup (control + iso IN pipes + 44.1 kHz negotiation + capture SET_INTERFACE), before `Ok(Self{...})`, add playback discovery. Reuse the already-negotiated clock (shared-clock assumption; the design notes a distinct playback clock would need its own SET_CUR — for Phase 2, if `find_uac_playback` matched, proceed on the shared clock):

```rust
        // Optional playback (full-duplex). Absent on capture-only devices.
        let (iso_out, playback) = match find_uac_playback(cfg) {
            Some(p) => {
                let pipe = alloc
                    .alloc_pipe::<pipe::Isochronous, pipe::Out>(addr, &p.endpoint.into(), split)
                    .map_err(UacError::NoPipe)?;
                Self::set_interface(&mut control, p.streaming_interface, p.alternate_setting).await?;
                let mut ring = SampleRing::new();
                ring.reset(p.num_channels as usize);
                let mut resampler = PlaybackResampler::new();
                resampler.reset(p.num_channels as usize);
                (
                    Some(pipe),
                    Some(PlaybackState { ring, resampler, channels: p.num_channels, max_packet: p.endpoint.max_packet_size }),
                )
            }
            None => (None, None),
        };
```

Add `iso_out,` and `playback,` to the `Ok(Self { ... })`. (`set_interface` is the Phase-1 helper.)

- [ ] **Step 4: Tests (extended mock)**

```rust
    #[test]
    fn register_opens_playback_on_full_duplex_device() {
        let state = MockState::leak();
        state.control_reads.borrow_mut()
            .push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap()).unwrap();
        let alloc = MockAlloc::new(state);
        let raw = uac2_full_duplex_cfg(3, 2, 2);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let host = block_on(Uac::try_register(&alloc, 1, None, &cfg)).expect("register");
        assert_eq!(host.channels(), 2);           // capture
        assert_eq!(host.playback_channels(), 2);  // playback opened
        // control + iso IN + iso OUT = 3 pipes.
        assert_eq!(state.allocs.borrow().len(), 3);
        // A SET_INTERFACE for the playback interface (2) was issued.
        let setups = state.setups.borrow();
        assert!(setups.iter().any(|s| s[1] == 11 && s[4] == 2),
            "expected SET_INTERFACE on playback interface 2");
    }

    #[test]
    fn register_capture_only_has_no_playback() {
        let state = MockState::leak();
        state.control_reads.borrow_mut()
            .push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap()).unwrap();
        let alloc = MockAlloc::new(state);
        let raw = uac2_mic_cfg(3, 2);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let host = block_on(Uac::try_register(&alloc, 1, None, &cfg)).expect("register");
        assert_eq!(host.playback_channels(), 0);
        assert_eq!(state.allocs.borrow().len(), 2); // control + iso IN only
    }
```

- [ ] **Step 5: Run tests + device build**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf uac` then `cargo build-fw -p deluge-bsp`
Expected: PASS (Phase-1 tests still green under the rename); compiles.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/mod.rs crates/deluge-bsp/src/usb/host/mod.rs
git commit -m "feat(deluge-bsp): host::uac full-duplex Uac (iso OUT pipe + playback negotiation)"
```

---

### Task 4: Full-duplex pump — send iso OUT paced by capture

Extend both pumps to emit a playback packet per capture tick.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/uac/mod.rs`

**Interfaces:**
- Consumes: `PlaybackState`, `encode_s24le` (Task 1), the capture pump (Phase 1).
- Produces: an internal `send_playback(&mut self, frames: usize)` helper used by both `pump_once` and `pump_once_shared`.

- [ ] **Step 1: Add the playback-send helper**

Add to `impl Uac`. It emits `frames` device frames (the capture packet's frame count this tick) from the playback ring via the pull resampler at `step = 1.0/self.r`, encodes to 24-bit LE, and sends one iso OUT packet. Underrun is handled inside `PlaybackResampler` (silence). A failed OUT transfer is logged + absorbed (no teardown; detach is still governed by the capture side's `BadResponse` propagation).

```rust
    async fn send_playback(&mut self, frames: usize) {
        let step = if self.r > 0.0 { 1.0 / self.r } else { 1.0 };
        let Some(pb) = self.playback.as_mut() else { return };
        let iso_out = match self.iso_out.as_mut() {
            Some(p) => p,
            None => return,
        };
        if frames == 0 {
            return;
        }
        let ch = pb.channels as usize;
        let mut buf = [0u8; 1024];
        let frame_bytes = ch * 3;
        let cap_frames = (buf.len() / frame_bytes).min(frames);
        let mut w = 0usize;
        pb.resampler.produce(&mut pb.ring, step, cap_frames, |frame| {
            for c in 0..ch {
                encode_s24le(frame[c], &mut buf[w..w + 3]);
                w += 3;
            }
        });
        let n = w.min(pb.max_packet as usize).min(buf.len());
        if let Err(e) = iso_out.request_out(&buf[..n], false).await {
            log::warn!("uac: iso OUT transfer failed ({:?}), dropping playback packet", e);
        }
    }
```

> Borrow note: `self.playback` and `self.iso_out` are separate fields, so taking `&mut` on both plus reading `self.r` (Copy) first is disjoint-field borrowing — bind `step` from `self.r` before the `self.playback`/`self.iso_out` borrows, as shown.

- [ ] **Step 2: Call it from both pumps**

In `pump_once` and `pump_once_shared`, after the capture processing succeeds (the `Ok(n)` path, once `n` bytes are decoded and the capture frame count is known), compute `let frames = n / (capture_channels * 3);` and `self.send_playback(frames).await;` before returning `Ok(())`. On the capture-error paths (BadResponse propagate / other-error absorb), do NOT send playback (there's no clock tick). Keep the capture data-flow and detach handling exactly as Phase 1.

- [ ] **Step 3: Tests — full-duplex pump asserts iso OUT bytes**

The mock's `state.log.borrow().sent` concatenates all `request_out` bytes. Pre-fill the playback ring (via the driver — expose a test-only way to push into `playback.ring`, e.g. a `#[cfg(test)] fn push_playback_frame(&mut self, f: &[f32])`, or drive `playback_write` once it exists in Task 5; for this task add the tiny test hook). Then pump one scripted capture packet and assert the OUT bytes:

```rust
    #[test]
    fn full_duplex_pump_sends_encoded_playback() {
        let state = MockState::leak();
        state.control_reads.borrow_mut()
            .push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap()).unwrap();
        // One capture packet: two mono 24-bit frames (so 2 playback frames sent).
        let mut pkt = heapless::Vec::<u8, 64>::new();
        pkt.extend_from_slice(&[0, 0, 0x40, 0, 0, 0x40]).unwrap(); // +0.5, +0.5
        state.script.borrow_mut().reads.push(pkt).unwrap();

        let alloc = MockAlloc::new(state);
        let raw = uac2_full_duplex_cfg(3, 1, 1); // mono cap + mono play
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = block_on(Uac::try_register(&alloc, 1, None, &cfg)).unwrap();

        // App-side playback data: a couple of known engine frames.
        host.push_playback_frame(&[0.5]);
        host.push_playback_frame(&[0.5]);

        block_on(host.pump_once()).expect("pump");
        // Exactly one OUT packet was sent, carrying encoded 24-bit frames.
        let sent = state.log.borrow().sent.clone();
        assert!(!sent.is_empty(), "playback packet must be sent");
        assert_eq!(sent.len() % 3, 0);
        // First sample encodes ~0.5 -> [0x00,0x00,0x40].
        assert_eq!(&sent[0..3], &[0x00, 0x00, 0x40]);
    }

    #[test]
    fn capture_only_pump_sends_no_playback() {
        let state = MockState::leak();
        state.control_reads.borrow_mut()
            .push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap()).unwrap();
        let mut pkt = heapless::Vec::<u8, 64>::new();
        pkt.extend_from_slice(&[0, 0, 0x40]).unwrap();
        state.script.borrow_mut().reads.push(pkt).unwrap();
        let alloc = MockAlloc::new(state);
        let raw = uac2_mic_cfg(3, 1);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = block_on(Uac::try_register(&alloc, 1, None, &cfg)).unwrap();

        block_on(host.pump_once()).expect("pump");
        assert!(state.log.borrow().sent.is_empty(), "capture-only sends no OUT");
    }
```

Add the test hook to `impl Uac`:

```rust
    #[cfg(all(test, not(target_os = "none")))]
    fn push_playback_frame(&mut self, f: &[f32]) {
        if let Some(pb) = self.playback.as_mut() {
            pb.ring.push_frame(f);
        }
    }
```

> The exact leading OUT bytes depend on the pull resampler's priming (first output = first buffered frame). If the priming makes the very first sample differ, assert instead that *some* emitted frame encodes `0.5` and that the total sent length is a whole number of frames — keep the "playback packet sent, correctly encoded, capture-only sends nothing" contract intact.

- [ ] **Step 4: Run tests + device build**

Run: `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf uac` then `cargo build-fw -p deluge-bsp`
Expected: PASS; compiles.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/mod.rs
git commit -m "feat(deluge-bsp): host::uac full-duplex pump (iso OUT paced by capture)"
```

---

### Task 5: `playback_write` app API + shared bridge

Expose playback to the app (engine task) via the device-only shared bridge, mirroring `capture_read`.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/uac/mod.rs`

**Interfaces:**
- Produces: `uac::playback_write(samples: &[f32]) -> usize` (device-gated, `pub`), and the `pump_once_shared` playback path drains the shared playback ring.

- [ ] **Step 1: Add a playback ring to the `shared` bridge**

In the `#[cfg(target_os = "none")] pub(crate) mod shared` block, add a second ring for playback alongside the capture ring:

```rust
    struct Playback {
        ring: SampleRing,
        channels: u8,
        active: bool,
    }
    static PLAYBACK: Mutex<CriticalSectionRawMutex, RefCell<Playback>> =
        Mutex::new(RefCell::new(Playback { ring: SampleRing::new(), channels: 0, active: false }));

    pub(crate) fn playback_begin(channels: u8) {
        PLAYBACK.lock(|c| {
            let mut c = c.borrow_mut();
            c.ring.reset(channels.clamp(1, MAX_CHANNELS as u8) as usize);
            c.channels = channels;
            c.active = true;
        });
    }
    pub(crate) fn playback_end() {
        PLAYBACK.lock(|c| { let mut c = c.borrow_mut(); c.active = false; c.channels = 0; c.ring.reset(1); });
    }
    /// Move all currently-available whole frames from the shared playback ring
    /// into `dst` (the pump's local staging ring), under ONE short critical
    /// section (bounded by ring capacity, no await). The resampler then pulls
    /// from `dst` outside the lock.
    pub(crate) fn playback_drain(dst: &mut SampleRing) {
        PLAYBACK.lock(|c| {
            let mut c = c.borrow_mut();
            let ch = c.channels.max(1) as usize;
            let mut frame = [0.0f32; MAX_CHANNELS];
            while c.ring.fill_frames() >= 1 && dst.fill_frames() < dst.capacity_frames() {
                c.ring.read(&mut frame[..ch]);
                dst.push_frame(&frame[..ch]);
            }
        });
    }
    /// App-facing: enqueue interleaved playback frames. Short return = ring
    /// full; never blocks.
    pub fn playback_write(samples: &[f32]) -> usize {
        PLAYBACK.lock(|c| {
            let mut c = c.borrow_mut();
            let ch = c.channels.max(1) as usize;
            let mut n = 0;
            for frame in samples.chunks_exact(ch) {
                if c.ring.fill_frames() >= c.ring.capacity_frames() { break; }
                c.ring.push_frame(frame);
                n += ch;
            }
            n
        })
    }
```

Re-export: extend the device-gated `pub use shared::{capture_channels, capture_read};` to also export `playback_write`.

- [ ] **Step 2: Drive the shared playback ring from `pump_once_shared` + lifecycle in `bind_uac`**

On device, the app fills the SHARED playback ring (`playback_write`); the pump's
resampler consumes from `self.playback.ring` (the local staging ring — the same
field the host-tested `pump_once` uses, so it is NOT dead on device). Each
`pump_once_shared` cycle first moves available frames shared→staging under one
short lock, then reuses Task 4's `send_playback` verbatim (which produces from
`self.playback.ring`):

In `pump_once_shared`, on the capture-success path, after computing `frames`:
```rust
        if let Some(pb) = self.playback.as_mut() {
            super::uac::shared::playback_drain(&mut pb.ring);
        }
        self.send_playback(frames).await;
```
(Order: drain into staging, then `send_playback` resamples staging → iso OUT.
`playback_drain`'s critical section is a bounded frame copy with no await.)

In `bind_uac` (`host/mod.rs`), after the successful spawn + `shared::begin(ch)`,
also call `super::uac::shared::playback_begin(host.playback_channels())` when
`host.playback_channels() > 0`. In `uac_capture_task`, after the loop exits, call
`super::uac::shared::playback_end()` alongside `shared::end()`.

> This task is device-only (`cfg(target_os = "none")`) glue plus the
> `playback_write` app API. It is verified by the device build linking and host
> tests staying green (the shared/task code is cfg'd out on host). The
> host-tested pump (`pump_once`, Task 4) remains the correctness reference for
> the playback data path.

- [ ] **Step 3: Device build + host tests**

Run: `cargo build-fw -p deluge-bsp` then `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf`
Expected: links; all host tests still pass.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/mod.rs crates/deluge-bsp/src/usb/host/mod.rs
git commit -m "feat(deluge-bsp): host::uac playback_write + shared bridge + full-duplex supervisor"
```

---

## Notes for the implementer

- **Order:** Task 1 (pure `out.rs`) and Task 2 (matching) are independent and fully host-tested. Task 3 renames `UacIn`→`Uac` and opens the pipe; Task 4 wires the pump; Task 5 is device-only app glue.
- **The pull resampler is output-driven** (Phase 1's `Resampler` is input-driven) — playback must produce an exact device-frame count, so it can't reuse the capture resampler directly. This is the one deliberate deviation from the spec's "reuse the resampler verbatim" wording; the ring is reused verbatim.
- **`step = 1.0 / self.r`:** `self.r` is the capture PI ratio (device/engine). Playback consuming `step` engine-frames per device-frame keeps the playback ring balanced against the same clock. No playback controller (design §2).
- **Deferred (do not build):** a dedicated playback drift trim, playback-only DAC support (explicit feedback), the on-hardware validation firmware. Carry forward the `TX_BULK_PIPE=1` cross-branch gate.
