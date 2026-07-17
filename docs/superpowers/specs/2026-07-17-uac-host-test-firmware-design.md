# UAC host test firmware — design & spec

A dedicated, minimal firmware image that brings USB0 up in **host mode**,
enumerates a class-compliant USB audio **output** device (DAC / audio interface
/ powered speaker), registers it through `embassy-usb-host` 0.1.0's `UacHandler`,
opens the isochronous OUT pipe, and streams a synthesized 440 Hz sine to it.
Every step is logged over RTT and mirrored to the OLED, so the host audio path
can be validated from the log/panel **even with nothing plugged in**.

> **⚠️ SUPERSEDED (2026-07-17).** This spec targeted embassy-usb-host's
> `UacHandler`, which is **output-only** and bypasses the project's own host
> class-driver layer (`deluge_bsp::usb::host`, cf. `host::midi`). The decision
> is to build the real custom `deluge_bsp::usb::host::uac` (capture + playback)
> per [`2026-07-16-usb-uac-host-design.md`](2026-07-16-usb-uac-host-design.md),
> staged capture-first. The validation **firmware** described here is retained
> as the final phase, but re-pointed to drive *our* `UacHost` (and to add a
> capture-record path) — not embassy's `UacHandler`. Read this only for the
> firmware scaffold (crate layout, RTT/OLED, `is_connected`); ignore the
> `UacHandler`/output-stream specifics.
>
> **Original status:** design proposal — approved for implementation.
> **Relationship to the production spec:** this was a *bring-up / validation*
> firmware, not the product feature. The full-duplex production design lives in
> [`2026-07-16-usb-uac-host-design.md`](2026-07-16-usb-uac-host-design.md)
> (capture + playback, drift correction, SCUX analysis, MIDI/audio pipe
> contention).

---

## 1. Goals & non-goals

**Goals**

- Exercise the whole host audio path end-to-end: root-port attach → bus reset →
  enumeration → UAC descriptor parse → SET_INTERFACE alt-select → ISO OUT pipe
  open → isochronous streaming (+ feedback tracking if the device exposes it).
- Be **observable without a device**: at every stage the firmware prints a clear
  RTT line and updates the OLED, so a maintainer reading `rtt.log` (or the panel)
  can tell exactly how far the host stack got.
- Produce **audible, correct** output on a real USB DAC — a clean 440 Hz tone at
  the device's negotiated format proves the ISO OUT lane byte-encoding is right.
- Stay a **single-purpose, minimal** image (modeled on `msc-firmware`): no
  product UI, no device-mode path.

**Non-goals**

- **Capture / input.** The embassy 0.1.0 UAC class implements output only; the
  production spec owns capture.
- **Concurrent MIDI-host or device-mode operation.** USB0 is host-only here, so
  the MIDI/audio pipe-contention constraint from the production spec §3.1 does
  not bite. One ISO OUT (+ optional ISO IN feedback) pipe is all we open.
- **Hub support, multiple devices, runtime host↔device switching, sample-rate
  conversion, drift correction into an engine.** None are needed to validate the
  host lane with a tone.
- **Rejecting devices by rate/format.** Unlike the production policy, this test
  firmware plays the tone at whatever Type-I PCM format the device negotiates
  (it just needs `bytes_per_sample` and `num_channels` to encode correctly).

---

## 2. Where it sits

New workspace member `firmwares/uac-host-firmware`, alongside `msc-firmware` /
`wp-probe` as a single-purpose bring-up image. It depends on `rza1l-hal`
(host driver), `embassy-usb-host` (UAC class + bus), `deluge-bsp` (OLED, PIC,
GPIO), and `deluge-alloc` (heaps, matching the other firmwares).

The **one shared-crate change** is a small accessor on the host driver:

- `rza1l_hal::usb::Rusb1HostDriver::is_connected(&self) -> bool`, backed by a new
  `static HCD_CONNECTED: AtomicBool` in `usb/host.rs` set `true` on the ATTCH ISR
  path and `false` on the DTCH ISR path (right beside the existing
  `HCD_EVENTS`/`need_reset` writes). This feeds `UacOut::output_stream`'s
  `is_connected: impl Fn() -> bool` guard so the stream ends promptly on unplug
  rather than only when the next `request_out` fails. Reusable by
  controller-firmware later.

---

## 3. Crate structure

```
firmwares/uac-host-firmware/
  Cargo.toml            # rza1l-hal, deluge-bsp, deluge-alloc, embassy-{executor,
                        #  time, sync, futures, usb-driver, usb-host}, log,
                        #  rtt-target(optional); [features] rtt (default on)
  src/main.rs           # manual main() (msc-firmware pattern): RTT init, SRAM+
                        #  SDRAM heaps, clocks, PIC UART, RSPI0/cv_gate, GPIO LED,
                        #  register USB0 ISR -> hcd_int_handler(0), init_host_mode(0),
                        #  build executor, spawn tasks
  src/tasks/mod.rs
  src/tasks/blink.rs    # P6_7 heartbeat LED — "firmware alive"
  src/tasks/pic.rs      # PIC baud handshake + OLED chip-select echo (msc copy)
  src/tasks/oled.rs     # renders host status + negotiated format from shared state
  src/tasks/uac_host.rs # the enumerate -> register -> stream loop + shared state
```

Workspace `Cargo.toml`: add `"firmwares/uac-host-firmware"` to `members`.
`.cargo/config.toml`: add `build-uac` / `build-uac-bin` aliases mirroring
`build-msc` / `build-msc-bin`.

`main()` wires the USB0 GIC handler unconditionally to `hcd_int_handler(0)` — no
`USB0_HOST_MODE` flag, no device-mode `embassy_usb::Builder`, no descriptor
statics. This is strictly simpler than the demo/controller dual-mode `main`.

---

## 4. The host task (`uac_host_task`)

Owns the `Rusb1HostDriver` (moved in at spawn). Pseudocode:

```rust
static BUS_STATE: BusState = BusState::new();
let (mut controller, handle) = bus(driver, &BUS_STATE);
let mut cfg_buf = [0u8; 512];

loop {
    STATUS.set(Waiting);                                   // OLED: "waiting for device"
    let speed = controller.wait_for_connection().await;    // ATTCH + bus reset
    log speed;

    let (enum_info, _n) = match handle.enumerate(BusRoute::Direct(speed), &mut cfg_buf).await {
        Ok(v) => v, Err(e) => { log; continue }
    };
    log VID/PID/device class/#configs;

    match UacHandler::try_register(&handle, enum_info).await {
        Ok(mut uac) => {
            let fmt = capture_format(&uac);   // {channels, bytes_per_sample, bit_res, sample_rate}
            log full UAC topology (see §6);
            STATUS.set(Connected(fmt));       // OLED shows format

            let mut out = match uac.output().await { Ok(o) => o, Err(e) => { log; continue } };
            let mut osc   = SineGen::new(440.0, fmt);
            let mut stats = StreamStats::new();

            STATUS.set(Streaming(fmt));
            let r = out.output_stream(
                || controller.controller().is_connected(),
                |buf| { osc.fill(buf); stats.tally(buf.len()); stats.maybe_log(); },
            ).await;
            log stream ended: {r:?};
        }
        Err(e) => { log "not a supported UAC output device: {e:?}"; STATUS.set(Error); }
    }

    handle.free_address(addr);   // addr from enum_info.device_address
}
```

`controller.controller().is_connected()` borrows `controller` immutably for the
duration of `output_stream`; `out` is a separate object, so there's no borrow
conflict. Because `enumerate` consumes fields of `enum_info` and
`try_register` takes it by value, the device address is captured **before**
`try_register` (`let addr = enum_info.device_address;`).

`&handle` is passed to `try_register` directly — `BusHandle` implements
`UsbHostAllocator` by forwarding to its inner `Rusb1Allocator`.

---

## 5. Sine generator — format-aware fill callback

`UacOut::output_stream` hands the callback a raw `&mut [u8]` packet each
microframe. Frames-per-packet = `len / (bytes_per_sample × num_channels)`.
`SineGen` holds an `f32` phase accumulator with `dφ = 2π · 440 / sample_rate`
and, per frame, writes the **same** sample to every channel encoded to the
negotiated subslot width (from `FormatTypeI.subslot_size`):

| `bytes_per_sample` | encoding                                             |
|--------------------|------------------------------------------------------|
| 2                  | `i16` little-endian                                  |
| 3                  | 24-bit signed, packed little-endian (3 bytes/sample) |
| 4                  | `i32` little-endian, sample top-justified (24-in-32) |

Amplitude ≈ −6 dBFS of the format's full scale — clearly audible, never clips.
If `subslot_size` is not 2/3/4 the callback fills silence and the task logs the
unsupported width once (the stream still runs so timing/feedback can be observed).

**Testability.** `SineGen::fill` is pure and scalar; it gets `#[cfg(test)]` host
unit tests (run under `--target x86_64-unknown-linux-gnu`) asserting: correct
frame count for a given packet length, identical bytes across channels within a
frame, and correct byte-lane encoding for each of the 2/3/4-byte widths (a
scalar oracle over one cycle). Per [[prefer-neon-simd]], SIMD is reserved for hot
DSP paths; a 440 Hz test oscillator stays scalar, but the 24-bit lane packing is
exactly the kind of byte math the oracle test guards — and note the deploy target
is 32-bit ([[target-32bit-usize-overflow]]), so any index/offset math in the
fill is reviewed for `u32` overflow by hand.

---

## 6. Shared state + OLED UI

The host task publishes a tiny lock-free snapshot the OLED task polls (all
`core::sync::atomic`, no heap, no `StereoFrame`):

```rust
// in uac_host.rs
static STAGE:        AtomicU8   = ...;  // Waiting|Enumerating|Connected|Streaming|Error
static VID:          AtomicU16;         // 0 until known
static PID:          AtomicU16;
static SAMPLE_RATE:  AtomicU32;         // Hz
static CHANNELS:     AtomicU8;
static BITS:         AtomicU8;          // bit_resolution (e.g. 16/24)
static PACKETS:      AtomicU32;         // running ISO OUT packet count (liveness)
```

`oled_task` (msc pattern: `pic::wait_ready().await` → `oled::init().await` →
render loop @ ~250 ms) draws:

```
UAC HOST                 UAC HOST                 UAC HOST
waiting for device       1234:5678                stream ended
                         44100Hz 2ch 24bit        (unplug/err)
                         streaming  1234pk
```

- **Waiting:** title + "waiting for device".
- **Connected/Streaming:** title, `VID:PID` (hex), a format line
  `"<rate>Hz <ch>ch <bits>bit"`, and a liveness line showing the packet count
  (proves ISO OUT is actually cycling).
- **Error / ended:** title + a short reason.

RTT logging carries the fuller detail the panel can't fit: on register, dump the
input terminal type/id + clock-source id, the output streaming interface's
channel count + Type-I format (subslot/bit-res) + `wMaxPacketSize` for the ISO
OUT endpoint + whether a feedback endpoint was found; during streaming, a
rate-limited (~1 Hz) line with packets/s, bytes/s, and current
samples/microframe (which moves as feedback is tracked); on teardown, the reason
and the freed address.

---

## 7. Error handling

- **Enumeration failure** (`EnumerationError`): log, drop back to `Waiting`,
  loop. No panic — a hot-plugged non-audio or misbehaving device must not brick
  the test.
- **`try_register` failure** (`RegisterError::NoSupportedInterface`, host error):
  log the variant, set `Error` on the panel briefly, loop. This is the expected
  outcome for a non-UAC device and is treated as normal.
- **`output()` / stream failure** (`RequestError`, incl. `DeviceDisconnected`):
  log, free the address, loop.
- **Disconnect during stream**: `is_connected()` flips false → `output_stream`
  returns `DeviceDisconnected`; belt-and-suspenders, the driver's DTCH path also
  fails in-flight pipes so `request_out` would error regardless.
- **Panic handler**: log `PANIC` and spin (msc-firmware pattern).

---

## 8. Testing & validation

1. **Host unit tests** — `SineGen::fill` byte-lane/frame-count/channel-replication
   (see §5), `cargo test -p uac-host-firmware --target x86_64-unknown-linux-gnu`.
   (Guard the `[[bin]]` with `test = false`; put `SineGen` in a module compiled
   for tests.)
2. **Builds** — `cargo build-uac` (debug, build-std=core) and the release
   `--no-default-features` variant both link.
3. **RTT-only validation (no device)** — flash, watch `rtt.log`: startup banner,
   "USB0 host mode", "waiting for device". Confirms init + host bring-up + the
   ISR/executor are alive with nothing plugged in. OLED shows "waiting".
4. **On-hardware with a USB DAC** — plug a class-compliant audio interface;
   expect the enumeration + UAC topology dump in RTT, the OLED format line, and
   an audible clean 440 Hz tone. The ~1 Hz stream stats should show a stable
   packets/s and the samples/microframe tracking the device's feedback.

---

## 9. Build & flash

`cargo build-uac` / `cargo build-uac-bin` (aliases added in §3), then the user's
existing dev-mode upload flow (`cargo deluge run --release`) and their own J-Link
RTT capture (writes `rtt.log`). No probe-rs.
