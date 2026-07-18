# USB Audio Class host — design & spec

Full-duplex host-side USB audio for the Deluge: plug a class-compliant audio
interface into the Deluge's USB port and both record from it and play out to it.
Builds on `embassy-usb-host` 0.1.0 and the `usb::host` supervisor landed by the
[USB MIDI host work](2026-07-16-usb-midi-host-design.md).

> **Status:** design proposal. The second of the two host-class sub-projects.
> The pipe map fixed in the MIDI spec §3.1 is a hard input here — MIDI and audio
> contend for the same hardware pipes.

---

## 1. Goals & non-goals

**Goals**

- **Capture**: record multichannel audio from a class-compliant USB interface.
- **Playback**: stream engine audio out to the same interface.
- **Multichannel capture**, surfaced as interleaved `f32` frames plus a channel
  count — not `StereoFrame`.
- Absorb clock drift between the Deluge's crystal and the device's, inaudibly.
- Keep the class driver **generic and host-testable** via the existing mock
  allocator.
- Stay **alloc-free** (`deluge-bsp` has no heap dependency).

**Non-goals**

- **Sample-rate conversion.** Devices that cannot run 44.1 kHz / 24-bit are
  rejected. This is not the same as ignoring clock drift — see §2.
- **SCUX.** No hardware SRC route, no new DMA channels. See §3.
- Changing the existing audio path. Capture is a separate API the app reads;
  `Audio::process` and codec line-in are untouched.
- Vendor-specific (non-class-compliant) interfaces.
- UAC1-only devices, if they turn out to need a materially different path.

---

## 2. Background: two clocks, one rate

**Rejecting non-44.1 kHz devices does not remove the clocking problem.** The
Deluge's audio clock is its own crystal (SSI0, crystal-locked 44.1 kHz,
`audio_block::SAMPLE_RATE_HZ`). A USB interface running "44.1 kHz" is running
*its* crystal's idea of 44.1 kHz. Two independent crystals at ±50 ppm drift
relative to each other by roughly 2–4 samples per second. Left uncorrected, the
buffer between them walks in one direction until it overruns or underruns — an
audible glitch roughly once a minute.

The HAL already encodes this: `rza1l_hal::scux::INTIFS_44100_TO_44100` exists
precisely because same-nominal-rate conversion is still asynchronous.

So drift correction is **mandatory regardless** of the device matrix. Narrowing
to 44.1/24 is a separate, cheaper decision layered on top: it removes rate
*negotiation* work and pins the resampler ratio near 1.0, which makes drift
correction cheap and high-quality. It does not remove the need for it.

### 2.1 Why not SCUX

The SCUX has four SRC paths (`rza1l-hal/src/scux.rs`: "2SRC | 2 units × 2 ch =
4 paths"), route 3 is free, and `scux_usb_tx_path` is already written for the
host-mode engine→USB direction (though it has **zero callers** and has never
been exercised). It is nonetheless not used here:

- **8-channel ceiling.** SCUX supports 1/2/4/6/8 channels; multichannel capture
  is a goal and this is where that ceiling bites first.
- **Route and DMA contention.** Routes 0 (`scux_dvu_path` → SSIF0) and 1
  (`scux_src_path`, device-mode UAC2) are in use; adding routes couples this
  work to the audio path's hardware budget.
- **The ratio is ~1.0 anyway.** With the 44.1/24-only policy, a software
  interpolator is inaudible and costs little.

`scux_usb_tx_path` remains available if the playback stage later wants it.

---

## 3. Architecture

### 3.1 Pipe budget

Isochronous transfers exist **only on pipes 1–2** (TRM §28.1(4); pipes 3–5 are
bulk-only). Full-duplex audio consumes both:

| Endpoint | Purpose | Pipe |
|---|---|---|
| iso IN | capture | 1 or 2 |
| iso OUT | playback | the other |
| iso IN | explicit feedback | **impossible — none left** |

**Explicit feedback is therefore off the table in full-duplex**, which forces
implicit feedback (§3.4). That is not a compromise: on a full-duplex interface
both streams share one device clock, so the capture stream *is* the clock
reference. This is standard practice (`snd-usb-audio` does it for a large class
of interfaces).

Consequences to hold onto:

- With both iso pipes claimed, MIDI is confined to bulk 3–5.
- The MIDI TX multiplexing work pins `TX_BULK_PIPE = 1`
  (`feat/usb-midi-tx-multiplex`, not yet on `main`). **That constant collides
  with iso.** It must become pipe 3 before audio and multiplexed MIDI can
  coexist.

### 3.2 Module layout

`crates/deluge-bsp/src/usb/host/uac/`, alongside `midi.rs`:

| File | Responsibility |
|---|---|
| `mod.rs` | `UacIn` class driver: matching, negotiation, the iso IN pump |
| `resample.rs` | Fractional resampler + PI drift controller (pure, no I/O) |
| `ring.rs` | Engine-rate ring buffer and the `read()` API |
| `out.rs` *(stage 2)* | Playback + implicit-feedback pacing |

`UacIn<'d, A: UsbHostAllocator>` is generic over the allocator, so it is unit
tested against `usb::host::mock::MockAlloc` exactly as `MidiHost` is. The mock
needs no changes — it is already generic over `pipe::Type`, so
`MockPipe<pipe::Isochronous, pipe::In>` works as-is.

### 3.3 Matching and negotiation

Reuse `embassy_usb_host::class::uac::descriptors::AudioInterfaceCollection::try_from_configuration(cfg)`.
It parses **all** streaming interfaces regardless of direction — only
`UacHandler::try_register` filters to output-only — so the whole UAC descriptor
tree (~3,200 lines across `descriptors.rs` / `codes.rs`) is reused, and we write
only the input-side search.

1. Find the first `AudioStreamingInterface` whose `endpoint_descriptor`
   has `ep_dir() == Direction::In`.
2. Require `format_type_descriptor == FormatTypeI` with `subslot_size == 3`
   (24-bit). Otherwise reject.
3. Require the clock entity to support 44,100 Hz; `SET_CUR` it to 44100 if it is
   not already there. Otherwise reject.
4. Read `class_descriptor.num_channels` — this is the app-visible channel count.
   No stereo assumption anywhere in this path.
5. `SET_INTERFACE` to the alt setting carrying the stream. **Mandatory, unlike
   MIDI**: UAC puts a zero-bandwidth setting on alt 0 and the real stream on
   alt 1+.

Rejection is explicit and logged (`UacError::UnsupportedFormat`), and frees the
device address immediately so an unusable device does not hold a scarce slot.

**What is *not* reusable:** `UacHandler`'s control-request helpers
(`get_sampling_freq`, `get_range_entity1..3`, `get_curr_entity1..3`) are methods
on `UacHandler`, taking `&mut self.control_channel` — they cannot be called
without constructing a `UacHandler`, and `UacHandler::try_register` fails outright
on a device with no output endpoint. So steps 3 and 5 mean allocating our own
control pipe and building the `SET_CUR` / `GET_CUR` / `GET_RANGE` and
`SET_INTERFACE` requests directly from `embassy_usb_host::control::SetupPacket`
plus the `uac::codes` constants. That is a small amount of code, but it is code
we write, not code we get.

### 3.4 Capture data path

```
iso IN packet ──► 24-bit→f32 ──► resampler(r) ──► ring (engine rate) ──► read()
                                      ▲                  │
                                      └──── PI(fill) ◄───┘
```

Everything left of the ring runs in the USB task. `read()` is a plain drain —
**no DSP on the audio path**, which matters because the engine's block deadline
is 128 frames at 44.1 kHz ≈ 2.9 ms.

**The PI controller is the clocking design.** Ring fill *is* the clock error: a
device running fast grows the ring, which raises `r`, which drains it faster. No
rate estimation, no feedback endpoint. With the 44.1/24-only policy `r` stays
within ~±200 ppm of 1.0.

The interpolator is deliberately boring — at `r ≈ 1.0` a windowed-sinc or cubic
kernel is inaudible. `deluge-dsp-kernels` and `armv7-dsp-intrinsics` (NEON) are
the natural home for the inner loop.

**App-facing API:**

```rust
/// Frames actually read; short reads mean underrun, never a block.
pub fn read(&self, out: &mut [f32]) -> usize;
/// Channels this device declared. `out.len()` should be a multiple of this.
pub fn channels(&self) -> u8;
```

Interleaved `f32`, allocation-free, any channel count the device declares.

### 3.5 Playback (stage 2)

`embassy-usb-host`'s `UacOut` paces playback from `parse_feedback` on a
dedicated feedback pipe. **We will not have that pipe** (§3.1), so playback is
paced by the capture stream's arrival instead: each captured packet is one tick
of the device's clock, and playback emits the matching number of frames.

This is why the plan is staged capture-first: playback's clock is a *product* of
capture working.

---

## 4. Error handling

- **Isochronous has no retries and no error recovery, by design.** A corrupted
  or missed packet is simply lost. A failed transfer must therefore **log and
  continue**, never tear down the stream; the ring absorbs the gap. This is the
  opposite of the bulk/MIDI path's instinct.
- **Underrun** returns a short read. USB must never wait on the audio thread and
  the audio thread must never wait on USB.
- **Overrun** (app not draining) drops oldest frames and lets the PI controller
  correct, rather than stalling the USB task.
- **Detach**: task exits → pipes drop → address freed, per the MIDI precedent.
- **Pipe exhaustion**: iso lives only on pipes 1–2. If MIDI holds them (pre-
  multiplexing, on `main`), audio cannot attach — report it clearly rather than
  failing opaquely.

---

## 5. Testing

- **Mock allocator** drives descriptor matching, format/rate rejection paths,
  channel-count reporting, and 24-bit→f32 decode under `cargo test`
  (`cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib`).
- **The resampler and PI controller are pure functions** — and they are the
  actual risk in this design, so they get the most testing. Feed a synthetic
  clock 100 ppm fast and assert the ring converges to its setpoint instead of
  drifting; assert `r` stays bounded; assert underrun returns short.
  This is a genuine test of the real hazard, runnable in CI.
- **Hardware**: a class-compliant interface enumerating and negotiating to
  44.1/24; sustained multi-minute capture watching for drift glitches; a
  rejected (48k-only) device failing cleanly; detach mid-stream.

---

## 6. Risks

- **The PI controller is the design.** Badly tuned, it either oscillates
  (audible pitch wobble) or converges too slowly to prevent over/underrun. It is
  also the one piece fully testable in CI, which is the mitigation.
- **`scux_usb_tx_path` has never been exercised.** Not used here, but if
  playback later reaches for it, that is unproven ground.
- **`TX_BULK_PIPE = 1` collides with iso** (§3.1). Must be resolved before
  multiplexed MIDI and audio coexist.
- **`embassy-usb-host` is 0.1.0** and will churn; we are pinned with no upstream.
- **UAC1 vs UAC2** differences are not yet surveyed; a device may need paths this
  design does not anticipate.

---

## 7. Future

- **SCUX offload**: if CPU cost or channel count demands it, route 3 is free and
  `scux_usb_tx_path` exists for the playback direction.
- **Rate negotiation** beyond 44.1 kHz, should the 44.1-only policy prove too
  narrow in practice.
- **Capture as an engine input source**, rather than only a separate read API.
