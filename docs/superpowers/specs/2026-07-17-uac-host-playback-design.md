# UAC host — Playback (Phase 2) design & spec

Full-duplex extension of the host UAC driver: stream engine audio **out** to the
same class-compliant UAC2 interface that Phase 1 captures from, at 44.1 kHz /
24-bit, paced by the capture clock (implicit feedback).

> **Status:** design proposal — scoped, awaiting sign-off before planning.
> **Builds on:** Phase 1 capture (`deluge_bsp::usb::host::uac`, merged to `main`
> at `c9d4a10`) and the design of record
> [`2026-07-16-usb-uac-host-design.md`](2026-07-16-usb-uac-host-design.md) §3.5.
> **Continues on** branch `feat/usb-uac-host`.

---

## 1. Goals & non-goals

**Goals**
- Play engine audio out to a full-duplex USB audio interface's isochronous OUT
  endpoint, interleaved multichannel `f32` in, 24-bit PCM on the wire.
- Pace playback by the **capture** stream (implicit feedback): one arriving iso
  IN packet = one device-clock tick → emit the matching number of OUT frames.
  No explicit feedback endpoint (full-duplex claims both iso pipes; §3.1).
- Reuse the capture drift estimate — there is **one device clock**, so no second
  controller. The capture PI's ratio drives both directions.
- Keep the class driver generic + host-testable against `MockAlloc`, exactly as
  Phase 1. Stay alloc-free; never block audio on USB or USB on audio.

**Non-goals (Phase 2)**
- **Playback-only devices (USB DACs with no capture).** With no capture stream
  there is no implicit clock; those need the explicit-feedback path (a separate
  pacing model — the second iso pipe would be free for a feedback IN endpoint).
  Deferred to a later phase.
- Rate/format beyond 44.1 kHz / 24-bit Type-I; UAC1; SCUX offload; the on-device
  validation firmware (still future — nothing has run on hardware yet).

---

## 2. Clocking — one device clock, reused

The Deluge's engine runs on its crystal (44.1 kHz). The USB device runs on *its*
crystal. Phase 1's capture PI already estimates the device/engine offset as the
resampler ratio `r` (device fast → ring fills → `r` rises). Both directions see
the **same** two crystals, so playback needs no new estimate:

- **Capture** resamples device→engine at `r` to keep the capture ring near its
  setpoint (Phase 1).
- **Playback** resamples engine→device using the same offset so the *playback*
  ring (app writes engine-rate, pump drains device-rate) stays balanced.

The exact algebra (whether playback consumes `r` or its reciprocal per emitted
device frame) is an implementation detail for the plan; the invariant is: **the
capture PI is the single clock authority, and playback reads it — it does not run
its own controller in Phase 2.** (A dedicated playback fill-trim is a possible
later refinement if app-side write jitter proves to walk the playback ring; noted
as a risk, not built.)

Because playback is paced off capture arrivals, **a capture stream must be
active** for playback to run — which is exactly why the work is staged
capture-first and why Phase 2 is full-duplex-only.

---

## 3. Architecture

### 3.1 Pipe budget (tightens Phase 1)

Full-duplex claims **both** iso pipes: pipe A = iso IN (capture), pipe B = iso
OUT (playback). Control stays on pipe 0. No iso pipe remains for explicit
feedback (forces the implicit model, §2) and none for MIDI iso. The cross-branch
**`TX_BULK_PIPE = 1`** collision (`feat/usb-midi-tx-multiplex`, not on this
branch) is *more* acute now that both iso pipes are always in use — carry it
forward as a hard merge gate.

### 3.2 Module layout

Add to `crates/deluge-bsp/src/usb/host/uac/`:

| File | Responsibility |
|---|---|
| `out.rs` *(new)* | Playback ring (app writes) + engine→device resampler + `f32`→24-bit LE encode + the iso OUT send |
| `mod.rs` *(extend)* | `find_uac_playback` match; `Uac` full-duplex driver (was `UacIn`); full-duplex pump; `shared` gains a playback ring + `playback_write` |

`decode_s24le` (Phase 1) gets a sibling `encode_s24le` (shared, pure, tested).
The `SampleRing` and `Resampler` from Phase 1 are reused verbatim for the
playback ring and the engine→device resampler.

### 3.3 Matching & negotiation

Extend the descriptor pass: alongside `find_uac_capture` (Direction::In), add
`find_uac_playback` — the first `AudioStreamingInterface` with a Direction::Out
endpoint, Type-I `subslot_size == 3`, its own `num_channels` (playback channel
count is independent of capture — stereo-out/mono-in is normal). A full-duplex
interface exposes both; a capture-only device yields no playback match and the
driver runs capture-only (graceful degrade — Phase 1 behavior preserved).

Negotiation adds one `SET_INTERFACE` on the OUT streaming interface's alt
setting. The sampling clock is typically **shared** across both directions, so
the 44.1 kHz `SET_CUR` from Phase 1 already covers it; the plan will confirm the
OUT interface links the same clock entity and only re-`SET_CUR` if it declares a
distinct one.

### 3.4 Full-duplex driver + pump

`UacIn` evolves into `Uac` owning: the control pipe, the iso IN pipe + capture
ring/resampler/PI (Phase 1), and — when a playback interface matched — the iso
OUT pipe + playback ring + engine→device resampler. One pump task drives both
(pacing couples them, so a single task is cleaner than two):

```
pump_once (full-duplex):
  ── capture ── request_in(iso IN) → decode s24→f32 → resample(r) → capture ring
                r = capture_PI.update(capture ring fill)         [Phase 1, unchanged]
  ── playback ─ if playback active:
                  n = frames this device tick (from the capture packet / r)
                  drain n frames from playback ring via engine→device resampler(r)
                  encode f32→s24 → request_out(iso OUT)
  detach: BadResponse propagates (Phase 1's threshold logic) → task exits →
          both pipes drop → address frees
```

App-facing (device-gated `shared` bridge): `capture_read` (exists) gains a mirror
`playback_write(&[f32]) -> usize` that fills the playback ring (short write =
ring full, never blocks). `bind_uac`/`uac_capture_task` extend to the full-duplex
`Uac`; a capture-only device simply never sends OUT.

---

## 4. Error handling

- Iso OUT, like iso IN, has **no retries**: a failed OUT transfer logs and
  continues (a dropped playback packet is a click, not a teardown). Detach still
  keys on the sustained-`BadResponse` threshold shared with capture.
- **Playback underrun** (app hasn't written): send silence (zeros) for the
  missing frames rather than stalling the pump — USB must not wait on the engine.
- **Playback overrun** (app writes faster than drain): `playback_write` short-
  returns; the app drops the excess. The pump never blocks on the app.
- A device that negotiates capture but whose OUT interface fails to open →
  capture-only, logged; the capture path is unaffected.

---

## 5. Testing

- **`out.rs` pure units** (host, `armv7-unknown-linux-gnueabihf`): playback ring
  (write/drain/overrun), engine→device resampler (reuse Phase 1's resampler
  tests' shape), `encode_s24le` endpoints (mirror of `decode_s24le`: `0.5 →
  [.., 0x3F]`, `-1.0 → [.., 0x80]`, round-trip `decode(encode(x)) ≈ x`).
- **Matching:** a full-duplex UAC2 fixture (IN + OUT streaming interfaces) →
  `find_uac_playback` returns the OUT interface/alt/channels; a capture-only
  fixture → `None`.
- **Negotiation:** the OUT `SET_INTERFACE` SETUP recorded via the extended mock.
- **Full-duplex pump:** script iso IN reads *and* assert iso OUT bytes via the
  mock's `sent` log — a known engine-rate playback ring input produces the
  expected 24-bit LE frames at the expected count; underrun sends silence.
- Device build (`cargo build-fw -p deluge-bsp`) links; full host suite stays
  green.

---

## 6. Risks

- **Single-clock reuse for playback.** Correct if the device shares one clock
  across directions (the common case at 44.1/24). A device with independent
  in/out clocks would walk the playback ring; the plan will confirm the shared-
  clock assumption from the descriptor and reject/degrade otherwise.
- **No playback fill-trim.** Reusing `r` assumes the app writes at true engine
  rate; sustained app-side jitter could drift the playback ring. Underrun→silence
  and overrun→short-write bound the audible damage; a playback PI is the refine-
  ment if measurement shows drift.
- **Both iso pipes now permanently claimed** — the `TX_BULK_PIPE` collision and
  any future iso-MIDI ambition are foreclosed until that constant moves.

---

## 7. Phasing

Phase 2 delivers full-duplex playback. Deferred: playback-only DAC support
(explicit feedback), a dedicated playback drift controller, and the on-hardware
validation firmware (which will exercise both capture and playback end-to-end).
