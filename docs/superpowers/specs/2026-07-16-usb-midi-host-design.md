# USB MIDI host — design & spec

Host-side USB MIDI 1.0 for the Deluge: plug a class-compliant controller, synth,
or hub into the Deluge's USB port and have its MIDI reach the firmware. Builds on
`embassy-usb-host` 0.1.0 and the existing `rza1l-hal` host driver, which today
enumerates a device and does nothing with it.

> **Status:** design proposal. The first of two host-class sub-projects; USB Audio
> Class **capture** (`UacIn`) gets its own spec and is out of scope here, but the
> pipe map in §3.1 is fixed now because MIDI and audio contend for the same
> hardware pipes.

This is **greenfield in this repo**, but not a new design. The original Synthstrom
firmware (`~/GitHub/DelugeFirmware-clean`) solves this exact problem on this exact
silicon, and §2 documents its approach. We port it rather than reinvent it.

---

## 1. Goals & non-goals

**Goals**

- Host **USB MIDI 1.0** (4-byte USB-MIDI event packets, alt setting 0) from
  class-compliant devices.
- Support **multiple devices through a hub** — target parity with the original
  firmware's 6.
- Support **sources** (controllers), **sinks** (synths/sound modules that only
  receive), and **bidirectional** devices (keyboard synths).
- Deliver events via a **merged, source-tagged channel** *and* **per-device
  handles**.
- Keep the class driver **generic and host-testable** via a mock allocator.
- Stay **alloc-free** — `deluge-bsp` has no heap dependency today and keeps none.

**Non-goals**

- **MIDI 2.0 / UMP.** The device-side class (`usb/classes/midi.rs`) does both 1.0
  and 2.0; the host side ships 1.0 only. Structure for UMP later, build none of it.
- **UAC capture.** Separate spec.
- Vendor-specific (non-class-compliant) controllers.
- Multi-tier hub topologies beyond what the hardware's `DEVADDn` HUBPORT field
  (3 bits, ports 1–7) and one hub natively express.

---

## 2. Background: the pipe budget is the whole design

### 2.1 The hardware constraint

RZ/A1L host mode (TRM §28.1(4), `vendor/docs/rza1/r01uh0437ej0700_rz_a1l.txt`):

| Pipe | Capability |
|---|---|
| 0 | Control (DCP), 256-byte fixed |
| 1–2 | Bulk **or isochronous** — iso is available *only* here |
| 3–5 | Bulk |
| 6–8 | Interrupt, 64-byte fixed |
| 9 | Interrupt (host mode); bulk only in function mode |
| 10–15 | **Function-controller mode only — unusable in host mode** |

So host mode has **5 bulk pipes and 4 interrupt pipes**, full stop. `rza1l-hal`'s
existing `alloc_pipe` partition (`crates/rza1l-hal/src/usb/host.rs:191`) is already
correct and is not leaving anything on the table. Widening the range does not work.

Buffer RAM is *not* a constraint: `crates/rza1l-hal/src/usb/pipe.rs:221` manages
128 × 64-byte blocks (8 KB) with 0–6 reserved; a double-buffered 64-byte bulk pipe
costs 2. The pipe *register sets* are the scarce resource.

### 2.2 The insight: RX and TX are not symmetric

The naive reading — one pipe per endpoint — gives ~2 bidirectional MIDI devices,
and 1 once audio streams. The original firmware gets **6**
(`MAX_NUM_USB_MIDI_DEVICES`, `src/definitions.h:74`).

USB is host-polled. A device's bulk IN endpoint transmits only when the host issues
an IN token, and a receive must be **armed** to catch MIDI the instant it arrives —
so **RX wants a dedicated pipe per device**. TX is host-initiated and can therefore
be **serialized onto one shared pipe**, retargeted per transfer, round-robin.

**TX is nearly free. RX is the scarce resource.** This asymmetry is the design.

### 2.3 What the original does

Pipe map (`src/RZA1/usb/userdef/r_usb_hmidi_config.h`):

```c
#define USB_CFG_HMIDI_BULK_SEND     (USB_PIPE1)   // shared across all devices
#define USB_CFG_HMIDI_INT_SEND      (USB_PIPE6)
#define USB_CFG_HMIDI_BULK_RECV_MIN (USB_PIPE2)
#define USB_CFG_HMIDI_BULK_RECV_MAX (USB_PIPE5)   // dedicated, 4 devices
#define USB_CFG_HMIDI_INT_RECV_MIN  (USB_PIPE7)
#define USB_CFG_HMIDI_INT_RECV_MAX  (USB_PIPE8)   // dedicated, 2 devices
// Hub uses PIPE9
```

4 bulk-RX + 2 interrupt-RX = 6 devices, filling all ten host pipes exactly.

Retarget mechanism (`src/deluge/io/midi/midi_engine.cpp:279-285`):

```c
if (d != currentDeviceNumWithSendPipe[ip][isInterrupt]) {
    currentDeviceNumWithSendPipe[ip][isInterrupt] = d;
    change_destination_of_send_pipe(&utr, pipeNumber, tmp_ep_tbl[ip][d],
                                    connectedDevice->sq);
}
connectedDevice->sq = !connectedDevice->sq;
```

Data toggle is per-*pipe* hardware state, so retargeting clobbers it. The original
keeps a per-device toggle in software (`ConnectedUSBMIDIDevice::sq`,
`midi_device_manager.h:78`) and restores it on switch.

### 2.4 Hard-won lessons to inherit

- **Hub race** (`midi_engine.cpp:266-269`): *"there was an assumption that the pipe
  wouldn't have changed if we were resuming a transfer but that has turned out not
  to be true if hubs are involved. A hub transaction seems to be able to run before
  the usbSendCompleteAsHost interrupt is called and changes the pipe, and then the
  next write doesn't go anywhere useful."*
- **Send sizing behind hubs** (`midi_device_manager.h:37-39`):
  `MIDI_SEND_BUFFER_LEN_INNER_HOST 2` — *"Seems to be the max for a hydrasynth on a
  usb hub... Widi bud's can do 3, both do fine at 16 without a hub involved."*

---

## 3. Architecture

### 3.1 The pipe map (global, fixed now)

MIDI-only (default):

| Pipe | Role | Capacity |
|---|---|---|
| 0 | DCP — control/enumeration | shared |
| 1 | Bulk TX — shared, retargeted | all devices |
| 2–5 | Bulk RX — dedicated | 4 devices |
| 6 | Interrupt TX — shared | all devices |
| 7–8 | Interrupt RX — dedicated | 2 devices |
| 9 | Hub status-change (interrupt IN) | 1 hub |

With UAC streaming, iso claims 1–2, degrading MIDI to bulk TX on 3 and RX on 4–5:
**2 bulk + 2 interrupt = 4 devices**. This is a documented operating mode, and the
reason the map is fixed in this spec rather than deferred to the audio one.

### 3.2 HAL: transparent OUT-pipe multiplexing

**`crates/rza1l-hal/src/usb/host.rs`.** `alloc_pipe` for an **OUT** endpoint returns
a *virtual* pipe sharing the one TX pipe for its transfer type — PIPE1 for bulk,
PIPE6 for interrupt — instead of consuming a dedicated pipe. **IN** endpoints keep
1:1 dedicated allocation (bulk from 2–5, interrupt from 7–8); exhaustion returns
`HostError::OutOfPipes`, which becomes the device ceiling.

The two TX pipes are independent resources with independent locks; a bulk retarget
must not serialize against an interrupt one. The description below is written for
bulk/PIPE1 and applies unchanged to interrupt/PIPE6.

Per virtual pipe the allocator tracks `(dev_addr, ep_addr, saved_toggle)`. Before a
transfer, if PIPE1 targets a different (device, endpoint):

1. Quiesce: `PID=NAK`, spin until `PBUSY=0` (required before `PIPEBUF`/`PIPEMAXP`/
   `PIPEPERI` writes — `pipe.rs:297`).
2. Save the outgoing pipe's toggle from `PIPECTR.SQMON`.
3. Rewrite `PIPESEL`/`PIPECFG.EPNUM`/`PIPEMAXP.DEVSEL`, and `DEVADDn` if the device
   differs.
4. Restore the incoming toggle via `SQSET`/`SQCLR`.
5. Transfer.

This is legal under embassy's contract — `UsbPipe` states *"Implementations are
responsible for maintaining the data toggle sequence"* — so `embassy-usb-host`
needs no changes and the class driver never learns about it.

**Atomicity.** Steps 1–5 must be atomic against hub traffic; this is exactly the
§2.4 race. A lock is held from retarget through transfer completion, not merely
around the retarget. Embassy also requires drop-aborts-transfer, so cancellation
must release the lock and leave PIPE1 quiesced.

**Also in this file:** replace the `debug_assert!(dev_addr < HCD_MAX_DEV)` in
`device_open` (`host.rs:493`) with a real bounds check returning
`HostError::OutOfSlots`. Multi-device makes a 6th address reachable, where today it
would index `ctl_mps[6]` on a 6-element array and panic in release.

### 3.3 Class driver

**`crates/deluge-bsp/src/usb/host/midi.rs`**, mirroring the device-side
`usb/classes/` layout.

`MidiHost<'d, A: UsbHostAllocator>` — fully generic, so a mock allocator drives it
under `cargo test`. Matching uses `ConfigurationDescriptor::try_from_slice` +
`iter_interface()` (no descriptor parsing to write): an interface with
`interface_class == 0x01` (Audio) and `interface_subclass == 0x03` (MIDIStreaming).
Matching the *interface* rather than the device class makes composite
audio+MIDI devices fall out correctly. MIDI 1.0 uses alt 0, so no `SET_INTERFACE`
in the common path.

**Claim policy.** Claim the directions the device declares. A sink (bulk OUT only —
a synth) is fully supported; a source (bulk IN only) likewise. Reject only when the
device declares RX and no RX pipe remains. Since TX shares a pipe, TX claims
effectively never fail.

Because `#[embassy_executor::task]` cannot be generic, the spawned task is a thin
monomorphic wrapper over `MidiHost<'static, Rusb1Allocator>`. The BSP already
depends on `rza1l-hal`, so this composes cleanly and keeps the generic driver
host-testable.

### 3.4 Orchestration

**`crates/deluge-bsp/src/usb/host/mod.rs`.** Approach: supervisor + static task
pools, alloc-free.

- **Supervisor task** owns the `BusController` — the only thing touching root-port
  events. On attach: enumerate, run matchers, hand off to a pooled per-device task.
  Unmatched devices are logged and their address freed immediately so they don't
  burn a slot.
- **Per-device task**, `#[embassy_executor::task(pool_size = 6)]`, owns its pipes
  and pumps decoded packets. Pipe lifetime is task lifetime, so detach cleanup is
  "task exits, pipes drop, address freed" — which matters when pipes are the scarce
  resource and leaking one costs a device.
- **Hub task** runs `HubHandler` on PIPE9, translating port events back to the
  supervisor, which enumerates hub-attached devices with `BusRoute::Translated`
  carrying `SplitInfo`. The HAL already handles splits — the RUSB hardware generates
  SSPLIT/CSPLIT automatically from `DEVADDn` (`host.rs:483-530`).

---

## 4. Data model & consumer API

The driver yields **raw 4-byte USB-MIDI packets** (`[u8; 4]`, cable number and CIN
intact) — the thinnest layer that loses nothing. Cable routing and sysex boundaries
survive; sysex passes through naturally since CIN 0x4–0x7 need no special handling
and callers reassemble.

Two consumer paths, both provided:

- **Merged channel** — a static `Channel` of `(DeviceId, [u8; 4])`, matching the
  device-side idiom (static channels + non-async accessors like
  `try_recv_from_host()`). One place to poll; a groovebox generally wants merged
  MIDI it filters by source.
- **Per-device handles** — yielded on attach, for callers wanting explicit routing
  or TX to a specific device.

`DeviceId` is a **slot index plus a monotonic generation counter**, not the raw USB
address. Addresses are freed and reused on hot-plug, so a stale ID could otherwise
silently alias a different controller.

Note the SDK's DIN MIDI (`crates/deluge-sdk/src/midi.rs`) is a byte stream
(`send(&[u8])` / `recv() -> u8`) and there is no parsed `MidiMessage` type anywhere
in the repo. A DIN-compatible byte-stream adapter over these packets is a plausible
follow-up; it is not in this spec.

---

## 5. Error handling & hot-plug

- **Detach**: task exits → pipes drop → `free_address`. The generation counter
  invalidates outstanding `DeviceId`s.
- **Shared TX pipe**: cancellation (dropped future) must abort the transfer, release
  the retarget lock, and leave PIPE1 quiesced. A device disconnecting mid-send must
  not strand the lock — this is the failure mode that would take down *all* MIDI TX.
- **Transfer errors**: the original notes send errors occur normally when a device
  attaches/detaches from a hub during fast MIDI sending (`midi_engine.cpp:128-135`)
  and deliberately does *not* drop the device. Mirror that: log and continue.
- **Send sizing**: start conservative behind hubs, per §2.4.
- **Pipe exhaustion**: report clearly rather than failing silently — the user needs
  to know the 5th/7th device didn't attach and why.

---

## 6. Testing

- **Mock `UsbHostAllocator`** with scripted pipe responses drives descriptor
  matching, endpoint claiming, packet decode, and the sink/source/bidirectional
  claim paths under `cargo test` on the host. Matches the repo's inline
  `#[cfg(test)]` idiom (`deluge-bsp/src/jacks.rs`, `flash.rs`) and runs in the
  existing `test.yml` CI job.
- **Toggle save/restore** gets unit tests against a mock register block if the HAL's
  structure permits; otherwise hardware-validated.
- **Hardware integration**: a single controller; a sink-only synth; two devices
  behind a hub (the multiplexing + race path); hot-plug churn during sustained TX.

---

## 7. Build sequence

1. `HCD_MAX_DEV` bounds-check fix (small, independent, removes a release-mode panic).
2. Mock allocator + test scaffolding.
3. `MidiHost` class driver + matcher + packet decode, against the mock. No HAL
   changes yet — 1:1 pipes, ~2 devices. **Shippable and useful.**
4. Supervisor + per-device tasks + merged channel; single-device hardware validation.
5. Hub task + `BusRoute::Translated`; multi-device hardware validation.
6. **HAL OUT-pipe multiplexing** + retarget lock; lifts the ceiling to 6.

Steps 1–5 deliver working MIDI at a lower device count; step 6 — the risky one — is
last and isolated.

---

## 8. Risks

- **The multiplexing is in the HAL transfer path**, where a bug corrupts *all* USB
  traffic, not just MIDI. Mitigated by sequencing it last (§7) and by the original
  firmware proving the approach on this exact silicon.
- **The hub/TX race** (§2.4) is subtle and timing-dependent. It cost the original
  authors real debugging; treat the lock scope as load-bearing.
- **`embassy-usb-host` is 0.1.0** (released 2026-05-04, no releases since) and will
  churn. We're pinned to stable with no upstream to track.
- **Toggle handling on retarget is the likeliest source of silent corruption** —
  symptom would be dropped/duplicated MIDI under multi-device load, not a clean
  failure.

---

## 9. Future

- **MIDI 2.0 / UMP host**: GTB descriptor fetch (class-specific `GET_DESCRIPTOR`
  0x26), alt-1 negotiation, 32-bit UMP words. The device-side class already does
  this and is the reference. Packet-level API here should not obstruct it.
- **UAC capture** (`UacIn`): its own spec. `embassy-usb-host`'s `UacHandler` is
  playback-only (`try_register` requires an output endpoint; format gated to
  `Type1::PCM`), so capture means a new input-interface search and iso IN handling.
  Budget per §3.1.
- **DIN-compatible byte-stream adapter** over the packet API.
