# Deluge simulator → web — design & spec

Port the native front-panel simulator (`tools/deluge-simulator`, currently an
`iced` desktop GUI) to a client-side browser app, dropping the `iced`
dependency. The near-term deliverable keeps the existing "brain" (DelugeFirmware's
C `deluge_host`) and reaches it over a WebSocket instead of raw TCP. The headline
future goal — a wasm firmware brain running fully in-browser — is explicitly
out of scope here but the transport seam is designed so it drops in later.

> **Status:** design proposal. Nothing built yet.

---

## 1. Goals & non-goals

**Goals (MVP)**

- Render a faithful Deluge faceplate in the browser: 128×48 OLED, 18×8 RGB pad
  grid, named buttons + LEDs, six encoders, knob indicators — driven entirely by
  the `deluge-protocol` wire.
- Forward pad / button / encoder input from the browser back over the wire.
- Talk to the **existing** C `deluge_host` brain, unmodified in behaviour, over a
  WebSocket (browsers cannot open raw TCP).
- Ship as static assets (Vite/TS), no server component of our own.
- No drift from the canonical wire contract: a golden-vector test pins the
  hand-written TS codec to the Rust `deluge-protocol` crate byte-for-byte.

**Non-goals (deferred past MVP)**

- **Audio.** Not carried by the `deluge-protocol` wire (the native sim's `rack`
  is compiled out over the protocol link). Belongs with the wasm-brain phase +
  Web Audio.
- **MIDI.** Not on the wire either; a future WebMIDI bridge + brain support.
- **CV/gate scopes.** The wire *does* carry `SET_CV`/`SET_GATE`, but visualizing
  them is deferred to keep the first milestone tight.
- **The wasm firmware brain** (in-browser brain replacing the WS link). The
  eventual goal; the WS transport is designed to be brain-agnostic so it slots in
  without touching the panel.
- Emulating the full Deluge / DelugeFirmware beyond what the wire exposes;
  cycle-accurate timing; USB/SD emulation.

**Success criteria**

The browser app connects to a WS-enabled `deluge_host`, renders live OLED / pad
RGB / LED / knob-indicator state as the firmware drives it, and pad/button/encoder
input reaches the firmware — a working faceplate, no `iced`, no native window.

---

## 2. Guiding insight — the panel seam already exists

The hard part of a web faceplate is already built. `tools/wren-web/app` renders a
faithful Deluge faceplate in the browser (`panel.ts` + `deluge-layout.ts`): pads,
named buttons+LEDs, encoders at real SVG coordinates, OLED on a canvas. That
`Panel` is fed through a narrow interface — input widgets call `sim.pad/button/enc(…)`,
output widgets poll `sim.oled_ptr()/led_ptr()/…` each frame. Today the object
behind that interface is the Wren VM wasm module.

**A web deluge-simulator is: keep the `Panel`, replace what sits behind it** — swap
the Wren brain for a `deluge-protocol` client that speaks to the firmware over a
WebSocket. This mirrors the native simulator's own split: `renderer.rs` (the
faceplate) reads a `DelugeHardware` state that `hardware_state.rs` maintains from
inbound `ToDeluge` frames delivered by `link.rs`. We are porting that same split
to the browser.

---

## 3. Architecture overview

```
┌─ browser (tools/deluge-web-sim/) ───────────────┐        ┌─ native (DelugeFirmware repo) ─┐
│                                                  │        │                                │
│  Panel (copied)  ⇄  ProtocolClient  ⇄  WebSocket ├────────┤ host_link (new WS listener)    │
│  faceplate DOM      state reducer +              │  ws://  │  ⇄ deluge_host "brain"         │
│  + OLED canvas      TS codec                     │ binary  │                                │
└──────────────────────────────────────────────────┘        └────────────────────────────────┘
   polls illumination each frame                                emits ToDeluge (illumination)
   calls pad/button/enc on input                                consumes FromDeluge (input)
```

Two new components and one copy:

| Component | Language | Role | Native counterpart |
| --- | --- | --- | --- |
| `protocol.ts` | TS | Encode/decode `deluge-protocol` message bodies | `deluge-protocol` crate |
| `protocol-client.ts` | TS | WebSocket owner + `ToDeluge`→state reducer + input encode | `link.rs` + `hardware_state.rs` |
| `panel.ts`, `deluge-layout.ts` | TS | Faceplate DOM + OLED canvas (copied, slimmed) | `renderer.rs` |
| WS listener in `host_link.c` | C | HTTP Upgrade handshake + WS binary framing | new transport branch |

### Wire mapping

One `deluge-protocol` frame = one WS **binary** message carrying `[type][data]`
(the u16 length prefix is redundant under WS and is dropped on this transport).
WS delivers message boundaries, so **the browser needs no `FrameDecoder`
reassembly** — it calls `decodeToDeluge(type, data)` per message directly.

The transport is **brain-agnostic**: anything that speaks `deluge-protocol` over
WS works behind the same `ProtocolClient` — the C firmware today, a Rust
SDK-app host or an in-browser wasm brain later.

### Handshake

`deluge-protocol` defines a small handshake the browser must honour:

- On WS open: browser sends `FromDeluge::Ready` (`0x12`).
- Brain may send `GET_VERSION` (`0x30`) → browser replies `VERSION` (`0x10`).
- Brain may send `PING` (`0x31`) → browser replies `PONG` (`0x11`).

---

## 4. Native side — WebSocket listener in `host_link.c`

**Repo:** `/home/kate/GitHub/DelugeFirmware` (`src/bsp/host/host_link.c`, ~293 LOC
today). This is the only change outside `deluge-sdk`, and the meatiest new piece.

`host_link.c` currently binds/listens TCP-loopback or AF_UNIX, `accept()`s one
client (the panel), then `host_link_send(type,data,n)` / `host_link_recv(...)`
shovel length-prefixed frames. The WS branch adds a third transport:

1. **Target form.** `host_link_init` recognises a `ws://:port` (or `ws://host:port`)
   target, sets a `link_is_ws` flag, and binds/listens TCP-loopback on that port
   (WS is HTTP-over-TCP — it can reuse the conventional port **9000**).
2. **Handshake.** After `accept()`, if `link_is_ws`: read the HTTP `GET` upgrade
   request, extract `Sec-WebSocket-Key`, compute `Sec-WebSocket-Accept`
   (`SHA1(key + magic)` → base64), and reply `101 Switching Protocols`.
3. **Framing.** `host_link_send` wraps each protocol frame `[type][data]` in a WS
   **binary** frame (server→client: unmasked; small-payload path is enough — the
   768-byte display blit uses the 16-bit length form). `host_link_recv` parses WS
   frames, **unmasks** client→server payloads (mandatory per RFC 6455), and
   handles control frames (respond to `ping`, honour `close`).
4. **No dependency.** Vendor a minimal single-file SHA1 + a compact WS frame
   codec (~200–300 LOC total). The existing TCP/Unix paths stay byte-identical —
   the WS logic is fully gated behind `link_is_ws`.

The brain's own `MessageToDeluge`/`FromDeluge` logic is untouched; only the
transport layer under `host_link_send/recv` learns WS.

---

## 5. Browser side — the standalone app

`tools/deluge-web-sim/` — a fresh Vite + TypeScript app on the same toolchain as
`tools/wren-web/app`, minus Monaco and the Wren VM. Standalone by choice: the
full-firmware faceplate emulator is a different product from the Wren playground
and is free to diverge. The `Panel` is **copied** (not shared) as the starting
point and slimmed to MVP.

### `protocol.ts` — the codec

Hand-written, byte-faithful to `crates/deluge-protocol/src/lib.rs`:

- `decodeToDeluge(type, data) → ToDeluge` for the host→device set:
  `UPDATE_DISPLAY` (0x20), `CLEAR_DISPLAY` (0x21), `SET_PAD_RGB` (0x22),
  `CLEAR_ALL_PADS` (0x23), `SET_LED` (0x24), `SET_CV` (0x25), `SET_GATE` (0x26),
  `SET_ALL_PADS` (0x27), `SET_KNOB_INDICATOR` (0x28), `SET_SYNCED_LED` (0x29),
  `CLEAR_ALL_LEDS` (0x2A), `SET_BRIGHTNESS` (0x2B), `GET_VERSION` (0x30),
  `PING` (0x31). (`SET_CV`/`SET_GATE` decoded but unused until the CV/gate
  milestone.)
- `encodeFromDeluge(ev) → Uint8Array` for the device→host set: `PAD_PRESSED`
  (0x01), `PAD_RELEASED` (0x02), `BUTTON_PRESSED` (0x03), `BUTTON_RELEASED`
  (0x04), `ENCODER_ROTATED` (0x05), plus handshake `VERSION` (0x10), `PONG`
  (0x11), `READY` (0x12).
- Constants mirrored from the crate: `DISPLAY_FRAME_BYTES = 768`,
  `ALL_PADS_BYTES = 432`, `BUTTON_ID_BASE = 144` / `cdc_button_id`.

### `protocol-client.ts` — WS owner + state reducer

The browser twin of `link.rs` + `hardware_state.rs`:

- Owns the `WebSocket` (binary type `arraybuffer`); connect / disconnect / status.
- **Reducer:** applies each decoded `ToDeluge` as a delta onto a full illumination
  state model — OLED framebuffer (`Uint8Array`, 768 bytes, page-major), pad RGB
  grid (18×8), per-LED state (raw PIC id → on/colour), knob indicators. The
  reducer is the source of what `Panel` renders each frame; incremental protocol
  ops (`SET_LED`, `SET_PAD_RGB`) and bulk ops (`SET_ALL_PADS`, `CLEAR_*`) both
  mutate this model.
- Exposes readers the copied `Panel` polls (`oledBytes()`, `padRGB()`,
  `ledState()`, `knobIndicators()`) and input methods `Panel` calls
  (`pad(x,y,down)`, `button(id,down)`, `enc(index,delta)`) → `encodeFromDeluge`
  → `ws.send`.
- Drives the handshake (§3).

### `panel.ts` + `deluge-layout.ts` — copied, retargeted, slimmed

Copied from `wren-web/app`, then: retarget the constructor from the `Sim` object
to `ProtocolClient` (same method/reader shape), and strip what MVP excludes —
audio (`out_*`), CV/gate scope wiring, MIDI monitor/keyboard, Wren error surface.
Faceplate geometry (`deluge-layout.ts`) is copied verbatim.

### `main.ts` — shell

- Connect UI: address field (default `ws://127.0.0.1:9000`), connect/disconnect,
  connection-status indicator.
- Render loop: `requestAnimationFrame` → poll `ProtocolClient` state → repaint
  OLED canvas + pad/LED DOM.
- Input wiring: Panel widget events → `ProtocolClient` input methods.

---

## 6. Testing & verification

- **Golden-vector conformance (drift guard).** A small Rust bin/test near
  `deluge-protocol` emits canonical encoded bytes for every message type (and
  representative payloads). A TS test asserts `decodeToDeluge`/`encodeFromDeluge`
  round-trip byte-equality against those vectors. This is what keeps the
  hand-written codec honest as the wire evolves — checked in CI.
- **Playwright e2e vs. a mock WS brain.** A scripted in-test WebSocket server
  replays a `ToDeluge` sequence → assert the OLED canvas and pad/LED DOM render
  correctly; simulate pad/button/encoder input → assert the exact `FromDeluge`
  bytes arrive on the mock server. No C brain needed in CI. (`wren-web/app`
  already uses Playwright — same harness.)
- **End-to-end smoke (M3).** The real WS-enabled `host_link` + the browser app,
  driven manually or by a scripted check, confirms the transport across repos.

---

## 7. Milestones

Browser-first, so the meatier cross-repo C change lands **last**, against an
already-proven browser side.

- **M0 — codec.** `protocol.ts` + golden-vector tests. No UI.
- **M1 — rendering.** App scaffold + copied/slimmed `Panel` rendering from a
  replayed `ToDeluge` fixture. Proves the faceplate with zero networking.
- **M2 — transport.** `protocol-client.ts` over WS + connect UI; Playwright e2e
  against a mock WS brain. **The full browser side is complete and tested here.**
- **M3 — native.** WS listener in `host_link.c`; end-to-end against the real C
  brain.

**Future (out of scope):** CV/gate scopes, WebMIDI bridge, Web Audio, and the
in-browser **wasm firmware brain** that replaces the WS link — making it a fully
client-side webapp.

---

## 8. Cross-repo note

This spec spans two repositories:

- **`deluge-sdk`** (this repo): the new `tools/deluge-web-sim/` app and the
  golden-vector fixtures (`crates/deluge-protocol`). M0–M2 live entirely here.
- **`DelugeFirmware`** (`/home/kate/GitHub/DelugeFirmware`): the WS listener in
  `src/bsp/host/host_link.c`. M3 only.

M0–M2 can be built and merged in `deluge-sdk` independently, validated against
the mock WS brain, before any DelugeFirmware change is needed.
