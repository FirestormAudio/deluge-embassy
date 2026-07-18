# UAC host — Validation firmware (Phase 3) design & spec

A dedicated host-mode firmware that runs our `usb_host_supervisor` (which
auto-binds the `host::uac` driver on connect), loops captured USB audio straight
back to playback (capture → playback round-trip), and reports status + level on
OLED and RTT. This is the **first real-hardware exercise** of the Phase 1–2 host
UAC stack.

> **Status:** design proposal — scoped, awaiting sign-off before planning.
> **Builds on:** Phase 1 capture + Phase 2 playback (`deluge_bsp::usb::host::uac`,
> on branch `feat/usb-uac-host`) and the design of record
> [`2026-07-16-usb-uac-host-design.md`](2026-07-16-usb-uac-host-design.md).
> **Supersedes-in-spirit:** the embassy-`UacHandler`-based test firmware
> (`2026-07-17-uac-host-test-firmware-design.md`), whose scaffold is reused but
> re-pointed at our own supervisor + app API.

---

## 1. Goals & non-goals

**Goals**
- Prove the Phase 1–2 stack on hardware: enumerate a class-compliant full-duplex
  USB audio interface, negotiate 44.1/24, and run capture + playback end-to-end.
- **Loopback**: read captured frames and write them straight back to playback —
  an audible round-trip (input → device → Deluge → device → output) that proves
  both directions *and* the shared-clock coupling in one shot.
- **Observability**: OLED (connection, negotiated format, in/out counters, input
  level) + rate-limited RTT stats, so the run is diagnosable even with the audio
  path misbehaving.
- Exercise the deferred hardware-bring-up items: detach behavior (Phase-2 review
  Important #1), PI-gain behavior under a real device clock, actual 24-bit audio.

**Non-goals**
- Automated/CI hardware testing (manual bring-up).
- Device-mode USB, MIDI-host, or hub demonstration (host-UAC only; the supervisor
  still binds those if present, but this firmware drives only the UAC path).
- Fixing the Phase-2 DTCH/detach HAL concern pre-emptively — surface it here; only
  reconcile the HAL if the hardware actually hangs on unplug.
- DAC-only playback / capture-as-engine-input (later phases).

---

## 2. Architecture

### 2.1 New firmware crate `firmwares/uac-host-firmware`

Modeled on `msc-firmware` (plain `#![no_std]`/`#![no_main]`, RTT feature,
`build.rs` + `memory.x`/`memory_rtt.x`, a small task set). **Host-only USB**: no
device-mode `embassy_usb::Builder`, no descriptor statics. The supervisor does
enumeration + class binding, so the firmware is thin.

```
firmwares/uac-host-firmware/
  Cargo.toml, build.rs, memory.x, memory_rtt.x   (msc-firmware pattern)
  src/main.rs        host bring-up: RTT, heaps, clocks, PIC, RSPI0, GPIO LED,
                     USB0 ISR -> hcd_int_handler(0), init_host_mode(0),
                     spawn usb_host_supervisor(hd, spawner) + blink/pic/oled/loopback
  src/tasks/{mod,blink,pic,oled,loopback}.rs
```

`main` spawns `deluge_bsp::usb::host::usb_host_supervisor(hd, spawner)` — that
task waits for connect, enumerates, and calls `bind_uac` (Phase 1/2), which
spawns the capture/playback pump task and publishes to the `capture`/`playback`
shared rings. The firmware's own tasks are purely app-side.

### 2.2 One small `deluge-bsp` addition

Expose `deluge_bsp::usb::host::uac::playback_channels() -> u8` (mirror of the
existing `capture_channels`), so the loopback task can adapt when the device's
capture and playback channel counts differ. ~4 lines in the `shared` bridge +
the re-export.

### 2.3 The loopback task

```
loop {
    let cap_ch  = uac::capture_channels();   // 0 until a device binds
    let play_ch = uac::playback_channels();
    if cap_ch == 0 { STATUS = Waiting; Timer(100ms); continue }
    STATUS = Streaming { rate: 44100, cap_ch, play_ch };

    let n = uac::capture_read(&mut scratch);        // interleaved f32, cap_ch
    if n > 0 {
        let peak = peak_abs(&scratch[..n]);         // metering
        remap(cap_ch, play_ch, &scratch[..n], &mut out);
        uac::playback_write(&out);                  // short-return tolerated
        stats.tally(n, peak);
    }
    Timer(1ms);   // batch ~one engine block; capture_read is non-blocking
}
```

- **Channel remap** (`remap(cap_ch, play_ch, in, out)`): `cap_ch == play_ch` →
  copy through; `cap_ch == 1` → duplicate mono to all playback channels;
  otherwise → copy `min(cap_ch, play_ch)` channels per frame, zero the rest.
  Kept deliberately simple; the device counts are logged so a mismatch is visible.
- **Metering**: `peak_abs` = max |sample| over the batch → a 0–N bar / rough dBFS
  for the OLED, and the running peak for RTT.
- Non-blocking throughout: `capture_read`/`playback_write` short-return; the 1 ms
  `Timer` batches roughly an engine block and keeps the task off a busy-spin.

### 2.4 OLED + RTT

`oled_task` (msc pattern: `pic::wait_ready()` → `oled::init()` → ~250 ms redraw)
reads the shared status the loopback task publishes:

```
UAC HOST                UAC HOST
waiting for device      44100Hz 2ch 24bit
                        loop in>out
                        in ####----  -9dB
```

RTT carries startup + connect/disconnect transitions (from `capture_channels()`
crossing 0) and a ~1 Hz stats line: captured frames/s, played frames/s, input
peak, and any playback short-writes (backpressure indicator).

---

## 3. Error handling / edge cases

- **No device / capture-only device**: `capture_channels() == 0` (or
  `playback_channels() == 0`) → the loopback idles in "waiting", no writes.
  A capture-only device shows format but "no playback" (loopback inert).
- **Channel mismatch**: handled by `remap`; logged once with both counts.
- **Playback backpressure**: `playback_write` short-returns; the firmware drops
  the unwritten tail (a loopback glitch, never a stall) and counts it.
- **Detach mid-stream**: the supervisor + capture task tear down (Phase 1/2);
  `capture_channels()` returns to 0 → loopback reverts to "waiting". **This is
  the path that exercises the Phase-2 DTCH concern** — watch for a hang here.

---

## 4. Testing

- **Build**: `cargo build-uac` (debug, build-std=core) and the release
  `--no-default-features` variant both link. (New cargo aliases mirroring
  `build-msc`.)
- **RTT-only (no device)**: startup banner, "USB0 host mode", "waiting for
  device"; OLED "waiting"; heartbeat LED. Confirms host bring-up + supervisor +
  executor are alive.
- **On-hardware (full-duplex USB audio interface)**: enumeration + UAC bind in
  RTT; OLED shows `44100Hz Nch 24bit` + "loop in>out"; feeding audio into the
  interface's input is heard back out its output (round-trip), and the level
  meter tracks it. Unplug → "waiting" (and no hang). This is the acceptance test
  for the whole Phase 1–2 stack.
- The firmware logic is thin bring-up glue; `remap`/`peak_abs` are trivial pure
  helpers. If either grows non-trivial, move it into `deluge-bsp` where it can be
  host-tested; otherwise no host tests (the validation is the hardware run).

---

## 5. Risks

- **This is the first hardware run** — any latent Phase 1–2 defect (PI tuning,
  detach, byte-lane, pacing) surfaces here. That is the point; the RTT/OLED
  observability is the mitigation.
- **Loopback latency**: capture ring + playback ring + the 1 ms batch add audible
  delay (tens of ms). Expected; it validates correctness, not latency.
- **DTCH/detach** (Phase-2 review #1): if unplug hangs the pump, reconcile the HAL
  DTCH path — tracked, only actioned if observed.
- **Channel-mismatch remap** is simplistic; exotic layouts may need refinement,
  but the common stereo/stereo interface works directly.

---

## 6. Phasing

Phase 3 delivers the validation firmware + the `playback_channels` accessor.
Deferred: automated HW testing, DAC-only playback, capture-as-engine-input, and
any HAL detach reconciliation the hardware run turns out to require.
