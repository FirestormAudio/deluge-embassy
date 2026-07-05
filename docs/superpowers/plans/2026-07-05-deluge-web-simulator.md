# Deluge Web Simulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the native front-panel simulator (`tools/deluge-simulator`, an `iced` desktop GUI) to a client-side browser app that renders the Deluge faceplate and drives the existing C `deluge_host` brain over a WebSocket.

**Architecture:** Reuse the already-built browser faceplate (`tools/wren-web/app`'s `Panel`, fed through a narrow input/illumination interface) by copying it into a new standalone app and swapping the brain behind it: instead of the Wren VM, a `deluge-protocol` client that decodes inbound illumination frames into a full state model and encodes input frames. Browsers cannot open raw TCP, so a WebSocket listener is added to DelugeFirmware's `host_link.c`; one `deluge-protocol` frame maps to one WS binary message.

**Tech Stack:** TypeScript + Vite + Vitest (unit) + Playwright (e2e); Rust (`deluge-protocol` crate, golden-vector generator); C (DelugeFirmware `host_link.c` WS listener).

**Spec:** `docs/superpowers/specs/2026-07-05-deluge-web-simulator-design.md`

## Global Constraints

These apply to every task; each task's requirements implicitly include them.

- **Byte-faithful wire.** The TS codec must produce/consume bytes identical to `crates/deluge-protocol/src/lib.rs`. Task 3's golden-vector test is the enforcement.
- **WS message = one frame body.** On the WS transport each binary message carries `[type_byte, ...data]` — the 2-byte little-endian length prefix used on TCP/Unix is dropped. The browser needs no frame reassembler.
- **Wire id conventions** (confirmed from `tools/deluge-simulator/src/link.rs`):
  - Button wire id = raw id + `144` (`BUTTON_ID_BASE`); the layout table's `rawId` is the raw id.
  - LED index (from `SetLed`) == raw button id == wire_id − 144 (all equal `x + 9*y`).
  - Encoder rotation id 0–5 is the wire id directly; the layout `EncoderSpec.index` already holds it.
- **OLED wire format** (confirmed from `tools/deluge-simulator/src/app.rs:463-491`): 768-byte page-major SSD1309, `buf[page*128 + col]`, bit `b` (0 = top) = panel row `page*8 + b`. Visible display is **43 rows starting at panel row 5** (`ui_row = panel_row − 5`, `0 ≤ ui_row < 43`).
- **Default connect target:** `ws://127.0.0.1:9000` (the conventional `DELUGE_HOST_LINK` port).
- **No new runtime npm dependencies.** The app is dependency-free at runtime (only `vite`/`vitest`/`@playwright/test`/`typescript` as devDependencies). Rendering uses DOM + Canvas 2D only.
- **TS strictness:** `tsconfig` uses `strict`, `noUnusedLocals`, `noUnusedParameters`, `verbatimModuleSyntax` (mirrors `tools/wren-web/app/tsconfig.json`).
- **MVP scope:** faceplate illumination (OLED, pad RGB, button LEDs, knob-indicator rings) + input (pads/buttons/encoders). CV/gate, MIDI, audio, and the in-browser wasm brain are out of scope; CV/gate/knob/synced-LED/brightness messages are still *decoded and stored* (never dropped) but only LEDs/pads/OLED/knob-rings are rendered.

## File Structure

New app under `tools/deluge-web-sim/` (excluded from the Rust workspace like the other `tools/*` web crates — but it is pure TS, so no `Cargo.toml`):

| File | Responsibility |
| --- | --- |
| `package.json`, `tsconfig.json`, `vite.config.ts`, `vitest.config.ts`, `playwright.config.ts` | Build/test config |
| `index.html` | Faceplate DOM shell (`#oled`, `#faceplate`, `#face-overlay`, connect UI) |
| `src/protocol.ts` | `deluge-protocol` codec: `decodeToDeluge`, `encodeFromDeluge`, constants |
| `src/protocol.test.ts` | Codec unit tests |
| `src/protocol-golden.json` | Rust-generated wire vectors (committed) |
| `src/protocol-conformance.test.ts` | Byte-equality vs the golden vectors |
| `src/deluge-layout.ts` | Faceplate coordinate tables (copied from wren-web) |
| `src/panel-client.ts` | `ToDeluge`→state reducer + input encode + handshake (the brain adapter) |
| `src/panel-client.test.ts` | Reducer + input-encode unit tests |
| `src/panel.ts` | Faceplate DOM + OLED canvas (copied from wren-web, slimmed) |
| `src/connection.ts` | WebSocket owner wiring `PanelClient` to a socket |
| `src/main.ts` | App shell: connect UI, render loop |
| `src/style.css` | Faceplate styling (copied subset from wren-web) |
| `tests/render.spec.ts` | M1 Playwright: renders a replayed frame fixture |
| `tests/e2e-ws.spec.ts` | M2 Playwright: end-to-end vs a mock WS brain |
| `crates/deluge-protocol/tests/golden.rs` | Vector generator + committed-file up-to-date test |
| DelugeFirmware `src/bsp/host/host_ws.{c,h}` | WS handshake + frame codec (testable helpers) |
| DelugeFirmware `src/bsp/host/host_link.c` | `ws://` target branch wiring `host_ws` in |

---

### Task 1: Scaffold the standalone app

**Files:**
- Create: `tools/deluge-web-sim/package.json`
- Create: `tools/deluge-web-sim/tsconfig.json`
- Create: `tools/deluge-web-sim/vite.config.ts`
- Create: `tools/deluge-web-sim/vitest.config.ts`
- Create: `tools/deluge-web-sim/index.html`
- Create: `tools/deluge-web-sim/src/main.ts`
- Create: `tools/deluge-web-sim/src/style.css`
- Create: `tools/deluge-web-sim/.gitignore`

**Interfaces:**
- Consumes: nothing.
- Produces: a buildable Vite app and a runnable Vitest harness that later tasks add files to.

- [ ] **Step 1: Create `package.json`**

```json
{
  "name": "deluge-web-sim",
  "private": true,
  "version": "0.1.0",
  "type": "module",
  "scripts": {
    "dev": "vite",
    "build": "tsc --noEmit && vite build",
    "preview": "vite preview",
    "test": "vitest run",
    "test:e2e": "playwright test"
  },
  "devDependencies": {
    "@playwright/test": "^1.49.1",
    "typescript": "^5.7.2",
    "vite": "^6.0.7",
    "vitest": "^2.1.8"
  }
}
```

- [ ] **Step 2: Create `tsconfig.json`**

```json
{
  "compilerOptions": {
    "target": "ES2022",
    "module": "ESNext",
    "moduleResolution": "bundler",
    "lib": ["ES2022", "DOM", "DOM.Iterable"],
    "strict": true,
    "noUnusedLocals": true,
    "noUnusedParameters": true,
    "skipLibCheck": true,
    "isolatedModules": true,
    "verbatimModuleSyntax": true,
    "resolveJsonModule": true,
    "types": ["vite/client", "vitest/globals"]
  },
  "include": ["src"]
}
```

- [ ] **Step 3: Create `vite.config.ts`**

```ts
import { defineConfig } from "vite";

// Standalone faceplate emulator: no SharedArrayBuffer (no audio in MVP), so no
// cross-origin-isolation headers are needed — unlike tools/wren-web/app.
export default defineConfig({
  base: "./",
  build: { target: "es2022", outDir: "dist" },
});
```

- [ ] **Step 4: Create `vitest.config.ts`**

```ts
import { defineConfig } from "vitest/config";

export default defineConfig({
  test: { globals: true, environment: "node", include: ["src/**/*.test.ts"] },
});
```

- [ ] **Step 5: Create `index.html`** (minimal shell; the faceplate DOM is filled in Task 5)

```html
<!doctype html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Deluge Web Simulator</title>
    <link rel="stylesheet" href="/src/style.css" />
  </head>
  <body>
    <main id="app"><p id="status">loading…</p></main>
    <script type="module" src="/src/main.ts"></script>
  </body>
</html>
```

- [ ] **Step 6: Create `src/style.css`** (placeholder; expanded in Task 5)

```css
:root { color-scheme: dark; }
body { margin: 0; background: #14171a; color: #e6edf0; font-family: system-ui, sans-serif; }
#app { padding: 1rem; }
```

- [ ] **Step 7: Create `src/main.ts`** (stub)

```ts
const status = document.querySelector<HTMLElement>("#status");
if (status) status.textContent = "scaffold ready";
```

- [ ] **Step 8: Create `.gitignore`**

```
node_modules/
dist/
test-results/
playwright-report/
```

- [ ] **Step 9: Install and verify build + empty test run**

Run: `cd tools/deluge-web-sim && npm install && npm run build && npm test`
Expected: `vite build` writes `dist/`; `vitest run` prints "No test files found" (exit 0) or passes with 0 tests. No TypeScript errors.

- [ ] **Step 10: Commit**

```bash
git add tools/deluge-web-sim/package.json tools/deluge-web-sim/package-lock.json \
  tools/deluge-web-sim/tsconfig.json tools/deluge-web-sim/vite.config.ts \
  tools/deluge-web-sim/vitest.config.ts tools/deluge-web-sim/index.html \
  tools/deluge-web-sim/src/main.ts tools/deluge-web-sim/src/style.css \
  tools/deluge-web-sim/.gitignore
git commit -m "feat(deluge-web-sim): scaffold standalone Vite/TS app"
```

---

### Task 2: Protocol codec (`protocol.ts`)

**Files:**
- Create: `tools/deluge-web-sim/src/protocol.ts`
- Test: `tools/deluge-web-sim/src/protocol.test.ts`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - Constants `OLED_W=128`, `OLED_H=43`, `OLED_PAGES=6`, `PANEL_TOP=5`, `DISPLAY_FRAME_BYTES=768`, `ALL_PADS_BYTES=432`, `PAD_COLS=18`, `PAD_ROWS=8`, `LED_COUNT=256`, `BUTTON_ID_BASE=144`.
  - `type ToDeluge` (discriminated union, `t` field) and `decodeToDeluge(type: number, data: Uint8Array): ToDeluge | null`.
  - `type FromDeluge` and `encodeFromDeluge(msg: FromDeluge): Uint8Array` (returns `[type_byte, ...data]`, no length prefix).
  - `cdcButtonId(raw: number): number` → `(raw + 144) & 0xff`.

- [ ] **Step 1: Write the failing tests**

Create `src/protocol.test.ts`:

```ts
import { describe, it, expect } from "vitest";
import { decodeToDeluge, encodeFromDeluge, cdcButtonId } from "./protocol";

describe("decodeToDeluge", () => {
  it("decodes SetPadRgb (0x22)", () => {
    expect(decodeToDeluge(0x22, new Uint8Array([3, 4, 10, 20, 30]))).toEqual({
      t: "SetPadRgb", col: 3, row: 4, rgb: [10, 20, 30],
    });
  });
  it("decodes SetLed (0x24)", () => {
    expect(decodeToDeluge(0x24, new Uint8Array([35, 1]))).toEqual({ t: "SetLed", index: 35, on: true });
  });
  it("decodes SetCv (0x25) big-endian u16", () => {
    expect(decodeToDeluge(0x25, new Uint8Array([1, 0x12, 0x34]))).toEqual({ t: "SetCv", channel: 1, value: 0x1234 });
  });
  it("decodes SetKnobIndicator (0x28)", () => {
    expect(decodeToDeluge(0x28, new Uint8Array([2, 1, 2, 3, 4]))).toEqual({
      t: "SetKnobIndicator", which: 2, levels: [1, 2, 3, 4],
    });
  });
  it("decodes UpdateDisplay (0x20) borrowing 768 bytes", () => {
    const data = new Uint8Array(768).map((_, i) => i & 0xff);
    const m = decodeToDeluge(0x20, data);
    expect(m?.t).toBe("UpdateDisplay");
    expect((m as { data: Uint8Array }).data.length).toBe(768);
  });
  it("decodes bare markers", () => {
    expect(decodeToDeluge(0x21, new Uint8Array())).toEqual({ t: "ClearDisplay" });
    expect(decodeToDeluge(0x2a, new Uint8Array())).toEqual({ t: "ClearAllLeds" });
    expect(decodeToDeluge(0x31, new Uint8Array())).toEqual({ t: "Ping" });
  });
  it("returns null on unknown type and short payloads", () => {
    expect(decodeToDeluge(0x99, new Uint8Array())).toBeNull();
    expect(decodeToDeluge(0x22, new Uint8Array([1, 2]))).toBeNull(); // needs 5
    expect(decodeToDeluge(0x20, new Uint8Array(10))).toBeNull(); // needs 768
  });
});

describe("encodeFromDeluge", () => {
  it("encodes ButtonPressed with wire id (raw+144)", () => {
    expect(Array.from(encodeFromDeluge({ t: "ButtonPressed", id: cdcButtonId(0) }))).toEqual([0x03, 144]);
  });
  it("encodes PadPressed", () => {
    expect(Array.from(encodeFromDeluge({ t: "PadPressed", col: 5, row: 2 }))).toEqual([0x01, 5, 2]);
  });
  it("encodes EncoderRotated with signed delta", () => {
    expect(Array.from(encodeFromDeluge({ t: "EncoderRotated", id: 4, delta: -1 }))).toEqual([0x05, 4, 0xff]);
  });
  it("encodes Ready/Pong/Version", () => {
    expect(Array.from(encodeFromDeluge({ t: "Ready" }))).toEqual([0x12]);
    expect(Array.from(encodeFromDeluge({ t: "Pong" }))).toEqual([0x11]);
    expect(Array.from(encodeFromDeluge({ t: "Version", major: 1, minor: 0, patch: 0 }))).toEqual([0x10, 1, 0, 0]);
  });
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd tools/deluge-web-sim && npx vitest run src/protocol.test.ts`
Expected: FAIL — `Cannot find module './protocol'`.

- [ ] **Step 3: Implement `src/protocol.ts`**

```ts
// deluge-protocol codec, byte-faithful to crates/deluge-protocol/src/lib.rs.
// On the WebSocket transport a binary message carries [type_byte, ...data] with
// no length prefix, so this module operates on message bodies, not framed bytes.

export const OLED_W = 128;
export const OLED_H = 43; // visible rows (panel rows 5..47)
export const OLED_PAGES = 6;
export const PANEL_TOP = 5; // first visible panel row
export const DISPLAY_FRAME_BYTES = 768;
export const ALL_PADS_BYTES = 432;
export const PAD_COLS = 18;
export const PAD_ROWS = 8;
export const LED_COUNT = 256;
export const BUTTON_ID_BASE = 144;

// host→device type bytes
const UPDATE_DISPLAY = 0x20, CLEAR_DISPLAY = 0x21, SET_PAD_RGB = 0x22,
  CLEAR_ALL_PADS = 0x23, SET_LED = 0x24, SET_CV = 0x25, SET_GATE = 0x26,
  SET_ALL_PADS = 0x27, SET_KNOB_INDICATOR = 0x28, SET_SYNCED_LED = 0x29,
  CLEAR_ALL_LEDS = 0x2a, SET_BRIGHTNESS = 0x2b, GET_VERSION = 0x30, PING = 0x31;

// device→host type bytes
const PAD_PRESSED = 0x01, PAD_RELEASED = 0x02, BUTTON_PRESSED = 0x03,
  BUTTON_RELEASED = 0x04, ENCODER_ROTATED = 0x05, VERSION = 0x10, PONG = 0x11,
  READY = 0x12;

export type ToDeluge =
  | { t: "UpdateDisplay"; data: Uint8Array }
  | { t: "ClearDisplay" }
  | { t: "SetPadRgb"; col: number; row: number; rgb: [number, number, number] }
  | { t: "ClearAllPads" }
  | { t: "SetLed"; index: number; on: boolean }
  | { t: "SetCv"; channel: number; value: number }
  | { t: "SetGate"; channel: number; on: boolean }
  | { t: "SetAllPads"; data: Uint8Array }
  | { t: "SetKnobIndicator"; which: number; levels: [number, number, number, number] }
  | { t: "SetSyncedLed"; on: boolean }
  | { t: "ClearAllLeds" }
  | { t: "SetBrightness"; level: number }
  | { t: "GetVersion" }
  | { t: "Ping" };

export function decodeToDeluge(type: number, d: Uint8Array): ToDeluge | null {
  switch (type) {
    case UPDATE_DISPLAY:
      return d.length >= DISPLAY_FRAME_BYTES ? { t: "UpdateDisplay", data: d.subarray(0, DISPLAY_FRAME_BYTES) } : null;
    case CLEAR_DISPLAY: return { t: "ClearDisplay" };
    case SET_PAD_RGB:
      return d.length >= 5 ? { t: "SetPadRgb", col: d[0], row: d[1], rgb: [d[2], d[3], d[4]] } : null;
    case CLEAR_ALL_PADS: return { t: "ClearAllPads" };
    case SET_LED: return d.length >= 2 ? { t: "SetLed", index: d[0], on: d[1] !== 0 } : null;
    case SET_CV: return d.length >= 3 ? { t: "SetCv", channel: d[0], value: (d[1] << 8) | d[2] } : null;
    case SET_GATE: return d.length >= 2 ? { t: "SetGate", channel: d[0], on: d[1] !== 0 } : null;
    case SET_ALL_PADS:
      return d.length >= ALL_PADS_BYTES ? { t: "SetAllPads", data: d.subarray(0, ALL_PADS_BYTES) } : null;
    case SET_KNOB_INDICATOR:
      return d.length >= 5 ? { t: "SetKnobIndicator", which: d[0], levels: [d[1], d[2], d[3], d[4]] } : null;
    case SET_SYNCED_LED: return d.length >= 1 ? { t: "SetSyncedLed", on: d[0] !== 0 } : null;
    case CLEAR_ALL_LEDS: return { t: "ClearAllLeds" };
    case SET_BRIGHTNESS: return d.length >= 1 ? { t: "SetBrightness", level: d[0] } : null;
    case GET_VERSION: return { t: "GetVersion" };
    case PING: return { t: "Ping" };
    default: return null;
  }
}

export type FromDeluge =
  | { t: "PadPressed"; col: number; row: number }
  | { t: "PadReleased"; col: number; row: number }
  | { t: "ButtonPressed"; id: number }
  | { t: "ButtonReleased"; id: number }
  | { t: "EncoderRotated"; id: number; delta: number }
  | { t: "Version"; major: number; minor: number; patch: number }
  | { t: "Pong" }
  | { t: "Ready" };

export function encodeFromDeluge(m: FromDeluge): Uint8Array {
  switch (m.t) {
    case "PadPressed": return Uint8Array.of(PAD_PRESSED, m.col, m.row);
    case "PadReleased": return Uint8Array.of(PAD_RELEASED, m.col, m.row);
    case "ButtonPressed": return Uint8Array.of(BUTTON_PRESSED, m.id & 0xff);
    case "ButtonReleased": return Uint8Array.of(BUTTON_RELEASED, m.id & 0xff);
    case "EncoderRotated": return Uint8Array.of(ENCODER_ROTATED, m.id, m.delta & 0xff);
    case "Version": return Uint8Array.of(VERSION, m.major, m.minor, m.patch);
    case "Pong": return Uint8Array.of(PONG);
    case "Ready": return Uint8Array.of(READY);
  }
}

export function cdcButtonId(raw: number): number {
  return (raw + BUTTON_ID_BASE) & 0xff;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools/deluge-web-sim && npx vitest run src/protocol.test.ts`
Expected: PASS (all cases).

- [ ] **Step 5: Commit**

```bash
git add tools/deluge-web-sim/src/protocol.ts tools/deluge-web-sim/src/protocol.test.ts
git commit -m "feat(deluge-web-sim): deluge-protocol TS codec"
```

---

### Task 3: Golden-vector conformance (Rust generator + TS check)

**Files:**
- Create: `crates/deluge-protocol/tests/golden.rs`
- Modify: `crates/deluge-protocol/Cargo.toml` (add `serde_json` dev-dependency; ensure `std`/`alloc` features usable in tests)
- Create: `tools/deluge-web-sim/src/protocol-golden.json` (generated, committed)
- Test: `tools/deluge-web-sim/src/protocol-conformance.test.ts`

**Interfaces:**
- Consumes: `decodeToDeluge`, `encodeFromDeluge` from Task 2; `ToDeluge`/`FromDeluge` `to_frame()` from the Rust crate.
- Produces: `protocol-golden.json` — an array of `{ "name": string, "dir": "to" | "from", "body": number[] }` where `body[0]` is the type byte. The two-sided pin: the Rust test regenerates and asserts the committed file is current; the TS test asserts the codec matches the file.

- [ ] **Step 1: Add the Rust dev-dependency**

In `crates/deluge-protocol/Cargo.toml`, under `[dev-dependencies]` (create the section if absent):

```toml
[dev-dependencies]
serde_json = "1"
```

Confirm the crate already exposes `std` and `alloc` features (it does — `#![cfg_attr(not(feature = "std"), no_std)]` and `to_frame` is `#[cfg(feature = "alloc")]`). Tests build with `std` implicitly enabled by the test harness only if a feature turns it on; add to `Cargo.toml` if not present:

```toml
[features]
# (keep existing entries; ensure these exist)
default = []
std = ["alloc"]
alloc = []
```

The golden test runs with `--features std`.

- [ ] **Step 2: Write the Rust generator + up-to-date test**

Create `crates/deluge-protocol/tests/golden.rs`:

```rust
//! Golden wire-vector generator + drift guard. The committed JSON at
//! `tools/deluge-web-sim/src/protocol-golden.json` is the single source the TS
//! codec is pinned against. Regenerate with:
//!   UPDATE_GOLDEN=1 cargo test -p deluge-protocol --features std --test golden
#![cfg(feature = "std")]

use deluge_protocol::{FromDeluge, ToDeluge, cdc_button_id};
use serde_json::json;
use std::path::PathBuf;

/// Body bytes = full frame minus the 2-byte length prefix (WS transport form).
fn body(frame: Vec<u8>) -> Vec<u8> {
    frame[2..].to_vec()
}

fn vectors() -> serde_json::Value {
    let mut display = vec![0u8; 768];
    for (i, b) in display.iter_mut().enumerate() {
        *b = (i % 251) as u8;
    }
    let mut pads = vec![0u8; 432];
    for (i, b) in pads.iter_mut().enumerate() {
        *b = (i % 253) as u8;
    }

    let to: Vec<(&str, Vec<u8>)> = vec![
        ("UpdateDisplay", body(ToDeluge::UpdateDisplay(&display).to_frame())),
        ("ClearDisplay", body(ToDeluge::ClearDisplay.to_frame())),
        ("SetPadRgb", body(ToDeluge::SetPadRgb { col: 3, row: 4, rgb: [10, 20, 30] }.to_frame())),
        ("ClearAllPads", body(ToDeluge::ClearAllPads.to_frame())),
        ("SetLed", body(ToDeluge::SetLed { index: 35, on: true }.to_frame())),
        ("SetCv", body(ToDeluge::SetCv { channel: 1, value: 0x1234 }.to_frame())),
        ("SetGate", body(ToDeluge::SetGate { channel: 2, on: true }.to_frame())),
        ("SetAllPads", body(ToDeluge::SetAllPads(&pads).to_frame())),
        ("SetKnobIndicator", body(ToDeluge::SetKnobIndicator { which: 2, levels: [1, 2, 3, 4] }.to_frame())),
        ("SetSyncedLed", body(ToDeluge::SetSyncedLed(true).to_frame())),
        ("ClearAllLeds", body(ToDeluge::ClearAllLeds.to_frame())),
        ("SetBrightness", body(ToDeluge::SetBrightness(200).to_frame())),
        ("GetVersion", body(ToDeluge::GetVersion.to_frame())),
        ("Ping", body(ToDeluge::Ping.to_frame())),
    ];
    let from: Vec<(&str, Vec<u8>)> = vec![
        ("PadPressed", body(FromDeluge::PadPressed { col: 5, row: 2 }.to_frame())),
        ("PadReleased", body(FromDeluge::PadReleased { col: 5, row: 2 }.to_frame())),
        ("ButtonPressed", body(FromDeluge::ButtonPressed { id: cdc_button_id(0) }.to_frame())),
        ("ButtonReleased", body(FromDeluge::ButtonReleased { id: cdc_button_id(35) }.to_frame())),
        ("EncoderRotated", body(FromDeluge::EncoderRotated { id: 4, delta: -1 }.to_frame())),
        ("Version", body(FromDeluge::Version { major: 1, minor: 0, patch: 0 }.to_frame())),
        ("Pong", body(FromDeluge::Pong.to_frame())),
        ("Ready", body(FromDeluge::Ready.to_frame())),
    ];

    let mut arr = Vec::new();
    for (name, b) in to {
        arr.push(json!({ "name": name, "dir": "to", "body": b }));
    }
    for (name, b) in from {
        arr.push(json!({ "name": name, "dir": "from", "body": b }));
    }
    serde_json::Value::Array(arr)
}

fn golden_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../tools/deluge-web-sim/src/protocol-golden.json")
}

#[test]
fn golden_vectors_up_to_date() {
    let generated = serde_json::to_string_pretty(&vectors()).unwrap() + "\n";
    let path = golden_path();
    if std::env::var("UPDATE_GOLDEN").is_ok() {
        std::fs::write(&path, &generated).unwrap();
        return;
    }
    let committed = std::fs::read_to_string(&path)
        .expect("protocol-golden.json missing — run with UPDATE_GOLDEN=1 to create it");
    assert_eq!(
        committed, generated,
        "protocol-golden.json is stale — regenerate: UPDATE_GOLDEN=1 cargo test -p deluge-protocol --features std --test golden"
    );
}
```

- [ ] **Step 3: Generate the committed fixture and verify the guard**

Run: `UPDATE_GOLDEN=1 cargo test -p deluge-protocol --features std --test golden`
Then: `cargo test -p deluge-protocol --features std --test golden`
Expected: first writes `tools/deluge-web-sim/src/protocol-golden.json`; second PASSES (file is current).

- [ ] **Step 4: Write the TS conformance test**

Create `src/protocol-conformance.test.ts`:

```ts
import { describe, it, expect } from "vitest";
import golden from "./protocol-golden.json";
import { decodeToDeluge, encodeFromDeluge, type FromDeluge, type ToDeluge } from "./protocol";

// The structured value each named vector is expected to decode to / encode from.
// Byte truth lives in protocol-golden.json (Rust-generated); this table is the
// TS side of the pin.
const DISPLAY = new Uint8Array(768).map((_, i) => i % 251);
const PADS = new Uint8Array(432).map((_, i) => i % 253);

const TO: Record<string, ToDeluge> = {
  UpdateDisplay: { t: "UpdateDisplay", data: DISPLAY },
  ClearDisplay: { t: "ClearDisplay" },
  SetPadRgb: { t: "SetPadRgb", col: 3, row: 4, rgb: [10, 20, 30] },
  ClearAllPads: { t: "ClearAllPads" },
  SetLed: { t: "SetLed", index: 35, on: true },
  SetCv: { t: "SetCv", channel: 1, value: 0x1234 },
  SetGate: { t: "SetGate", channel: 2, on: true },
  SetAllPads: { t: "SetAllPads", data: PADS },
  SetKnobIndicator: { t: "SetKnobIndicator", which: 2, levels: [1, 2, 3, 4] },
  SetSyncedLed: { t: "SetSyncedLed", on: true },
  ClearAllLeds: { t: "ClearAllLeds" },
  SetBrightness: { t: "SetBrightness", level: 200 },
  GetVersion: { t: "GetVersion" },
  Ping: { t: "Ping" },
};

const FROM: Record<string, FromDeluge> = {
  PadPressed: { t: "PadPressed", col: 5, row: 2 },
  PadReleased: { t: "PadReleased", col: 5, row: 2 },
  ButtonPressed: { t: "ButtonPressed", id: 144 },
  ButtonReleased: { t: "ButtonReleased", id: 179 },
  EncoderRotated: { t: "EncoderRotated", id: 4, delta: -1 },
  Version: { t: "Version", major: 1, minor: 0, patch: 0 },
  Pong: { t: "Pong" },
  Ready: { t: "Ready" },
};

describe("golden-vector conformance", () => {
  for (const v of golden as { name: string; dir: "to" | "from"; body: number[] }[]) {
    it(`${v.dir}:${v.name}`, () => {
      const body = Uint8Array.from(v.body);
      if (v.dir === "to") {
        const decoded = decodeToDeluge(body[0], body.subarray(1));
        expect(decoded).toEqual(TO[v.name]);
      } else {
        expect(Array.from(encodeFromDeluge(FROM[v.name]))).toEqual(v.body);
      }
    });
  }
});
```

- [ ] **Step 5: Run the TS conformance test**

Run: `cd tools/deluge-web-sim && npx vitest run src/protocol-conformance.test.ts`
Expected: PASS for every vector (both directions).

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-protocol/Cargo.toml crates/deluge-protocol/tests/golden.rs \
  tools/deluge-web-sim/src/protocol-golden.json tools/deluge-web-sim/src/protocol-conformance.test.ts
git commit -m "test(deluge-web-sim): Rust-generated golden vectors pin the TS codec"
```

---

### Task 4: Brain adapter — `PanelClient` state reducer + input encoding

**Files:**
- Create: `tools/deluge-web-sim/src/panel-client.ts`
- Test: `tools/deluge-web-sim/src/panel-client.test.ts`

**Interfaces:**
- Consumes: everything from `protocol.ts` (Task 2).
- Produces: `class PanelClient` — the object the copied `Panel` talks to (same method surface the wren `Sim` exposed for MVP):
  - `constructor(send: (bytes: Uint8Array) => void)` — `send` is called with `[type, ...data]` bodies to put on the wire.
  - Input (called by `Panel`): `pad(x: number, y: number, down: boolean)`, `button(rawId: number, down: boolean)`, `enc(index: number, delta: number)`.
  - Inbound: `onMessage(body: Uint8Array)` — decodes one `[type, ...data]` message, applies it to state, and auto-answers `Ping`→`Pong` and `GetVersion`→`Version{1,0,0}`.
  - Handshake: `sendReady()` — sends `Ready`.
  - Readers (called by `Panel.frame()`): `oled(): Uint8Array` (length `OLED_W*OLED_H` = 5504, unpacked, nonzero = lit), `leds(): Uint8Array` (length `LED_COUNT`), `padRgb(): Uint8Array` (length `PAD_COLS*PAD_ROWS*3`, col-major `(col*8+row)*3`), `knob(which: number): [number,number,number,number]`.

- [ ] **Step 1: Write the failing tests**

Create `src/panel-client.test.ts`:

```ts
import { describe, it, expect, vi } from "vitest";
import { PanelClient } from "./panel-client";
import { encodeFromDeluge, OLED_W } from "./protocol";

const to = (type: number, ...data: number[]) => Uint8Array.of(type, ...data);

describe("PanelClient input encoding", () => {
  it("encodes pad/button/encoder input onto the wire", () => {
    const send = vi.fn();
    const c = new PanelClient(send);
    c.pad(5, 2, true);
    c.button(0, true); // rawId 0 → wire 144
    c.enc(4, -1);
    expect(send.mock.calls[0][0]).toEqual(Uint8Array.of(0x01, 5, 2));
    expect(send.mock.calls[1][0]).toEqual(Uint8Array.of(0x03, 144));
    expect(send.mock.calls[2][0]).toEqual(Uint8Array.of(0x05, 4, 0xff));
  });
});

describe("PanelClient state reducer", () => {
  it("applies SetLed / ClearAllLeds", () => {
    const c = new PanelClient(() => {});
    c.onMessage(to(0x24, 35, 1)); // SetLed index 35 on
    expect(c.leds()[35]).toBe(1);
    c.onMessage(to(0x2a)); // ClearAllLeds
    expect(c.leds()[35]).toBe(0);
  });

  it("applies SetPadRgb (col-major offset) and ClearAllPads", () => {
    const c = new PanelClient(() => {});
    c.onMessage(to(0x22, 1, 2, 10, 20, 30)); // col 1, row 2
    const off = (1 * 8 + 2) * 3;
    expect(Array.from(c.padRgb().subarray(off, off + 3))).toEqual([10, 20, 30]);
    c.onMessage(to(0x23)); // ClearAllPads
    expect(Array.from(c.padRgb().subarray(off, off + 3))).toEqual([0, 0, 0]);
  });

  it("unpacks UpdateDisplay: page-major, bit0=top, 5-row bezel offset", () => {
    const c = new PanelClient(() => {});
    const fb = new Uint8Array(768);
    // panel row 5 (first visible → ui_row 0), col 0: page 0, bit 5.
    fb[0 * 128 + 0] = 1 << 5;
    // panel row 6 (ui_row 1), col 3: page 0, bit 6.
    fb[0 * 128 + 3] = 1 << 6;
    c.onMessage(Uint8Array.of(0x20, ...fb));
    const px = c.oled();
    expect(px[0 * OLED_W + 0]).toBe(1); // ui_row 0, col 0
    expect(px[1 * OLED_W + 3]).toBe(1); // ui_row 1, col 3
    expect(px[0 * OLED_W + 1]).toBe(0);
  });

  it("ignores bezel rows above the visible area", () => {
    const c = new PanelClient(() => {});
    const fb = new Uint8Array(768);
    fb[0 * 128 + 0] = 1 << 0; // panel row 0 → above visible, dropped
    c.onMessage(Uint8Array.of(0x20, ...fb));
    expect(c.oled().every((v) => v === 0)).toBe(true);
  });

  it("stores knob indicators", () => {
    const c = new PanelClient(() => {});
    c.onMessage(to(0x28, 2, 1, 2, 3, 4));
    expect(c.knob(2)).toEqual([1, 2, 3, 4]);
  });
});

describe("PanelClient handshake", () => {
  it("answers Ping with Pong and GetVersion with Version", () => {
    const send = vi.fn();
    const c = new PanelClient(send);
    c.onMessage(to(0x31)); // Ping
    c.onMessage(to(0x30)); // GetVersion
    expect(send.mock.calls[0][0]).toEqual(encodeFromDeluge({ t: "Pong" }));
    expect(send.mock.calls[1][0]).toEqual(encodeFromDeluge({ t: "Version", major: 1, minor: 0, patch: 0 }));
  });

  it("sendReady emits Ready", () => {
    const send = vi.fn();
    new PanelClient(send).sendReady();
    expect(send.mock.calls[0][0]).toEqual(encodeFromDeluge({ t: "Ready" }));
  });
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd tools/deluge-web-sim && npx vitest run src/panel-client.test.ts`
Expected: FAIL — `Cannot find module './panel-client'`.

- [ ] **Step 3: Implement `src/panel-client.ts`**

```ts
// The brain adapter: the object the faceplate Panel talks to. Decodes inbound
// ToDeluge illumination into a full state model the Panel polls each frame, and
// encodes Panel input as FromDeluge onto the wire. The browser twin of the
// native simulator's link.rs + hardware_state.rs.
import {
  decodeToDeluge, encodeFromDeluge, cdcButtonId,
  OLED_W, OLED_H, OLED_PAGES, PANEL_TOP, LED_COUNT, PAD_COLS, PAD_ROWS,
} from "./protocol";

const PROTOCOL_VERSION: [number, number, number] = [1, 0, 0];

export class PanelClient {
  private oledPx = new Uint8Array(OLED_W * OLED_H); // unpacked, nonzero = lit
  private ledState = new Uint8Array(LED_COUNT);
  private pads = new Uint8Array(PAD_COLS * PAD_ROWS * 3); // col-major (col*8+row)*3
  private knobs = new Map<number, [number, number, number, number]>();

  constructor(private send: (bytes: Uint8Array) => void) {}

  // ── input (called by Panel) ──────────────────────────────────────────────
  pad(x: number, y: number, down: boolean) {
    this.send(encodeFromDeluge(down ? { t: "PadPressed", col: x, row: y } : { t: "PadReleased", col: x, row: y }));
  }
  button(rawId: number, down: boolean) {
    const id = cdcButtonId(rawId);
    this.send(encodeFromDeluge(down ? { t: "ButtonPressed", id } : { t: "ButtonReleased", id }));
  }
  enc(index: number, delta: number) {
    this.send(encodeFromDeluge({ t: "EncoderRotated", id: index, delta }));
  }
  sendReady() {
    this.send(encodeFromDeluge({ t: "Ready" }));
  }

  // ── inbound (one [type, ...data] message) ────────────────────────────────
  onMessage(body: Uint8Array) {
    if (body.length === 0) return;
    const msg = decodeToDeluge(body[0], body.subarray(1));
    if (!msg) return;
    switch (msg.t) {
      case "UpdateDisplay": this.unpackDisplay(msg.data); break;
      case "ClearDisplay": this.oledPx.fill(0); break;
      case "SetPadRgb": {
        const off = (msg.col * PAD_ROWS + msg.row) * 3;
        this.pads.set(msg.rgb, off);
        break;
      }
      case "SetAllPads": this.pads.set(msg.data); break;
      case "ClearAllPads": this.pads.fill(0); break;
      case "SetLed": this.ledState[msg.index] = msg.on ? 1 : 0; break;
      case "ClearAllLeds": this.ledState.fill(0); break;
      case "SetKnobIndicator": this.knobs.set(msg.which, msg.levels); break;
      case "Ping": this.send(encodeFromDeluge({ t: "Pong" })); break;
      case "GetVersion":
        this.send(encodeFromDeluge({ t: "Version", major: PROTOCOL_VERSION[0], minor: PROTOCOL_VERSION[1], patch: PROTOCOL_VERSION[2] }));
        break;
      // SetCv / SetGate / SetSyncedLed / SetBrightness: decoded, not rendered in MVP.
      default: break;
    }
  }

  // buf[page*128 + col], bit b = panel row page*8+b (bit 0 = top); visible rows
  // start at PANEL_TOP (ui_row = panel_row - PANEL_TOP), 0..OLED_H.
  private unpackDisplay(fb: Uint8Array) {
    this.oledPx.fill(0);
    for (let page = 0; page < OLED_PAGES; page++) {
      for (let col = 0; col < OLED_W; col++) {
        const byte = fb[page * OLED_W + col];
        if (byte === 0) continue;
        for (let bit = 0; bit < 8; bit++) {
          if ((byte & (1 << bit)) === 0) continue;
          const uiRow = page * 8 + bit - PANEL_TOP;
          if (uiRow >= 0 && uiRow < OLED_H) this.oledPx[uiRow * OLED_W + col] = 1;
        }
      }
    }
  }

  // ── readers (called by Panel.frame()) ────────────────────────────────────
  oled(): Uint8Array { return this.oledPx; }
  leds(): Uint8Array { return this.ledState; }
  padRgb(): Uint8Array { return this.pads; }
  knob(which: number): [number, number, number, number] {
    return this.knobs.get(which) ?? [0, 0, 0, 0];
  }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools/deluge-web-sim && npx vitest run src/panel-client.test.ts`
Expected: PASS (input encoding, reducer, OLED unpack incl. bezel, handshake).

- [ ] **Step 5: Commit**

```bash
git add tools/deluge-web-sim/src/panel-client.ts tools/deluge-web-sim/src/panel-client.test.ts
git commit -m "feat(deluge-web-sim): PanelClient brain adapter (reducer + input + handshake)"
```

---

### Task 5: Faceplate rendering (copy Panel) + render loop from a replayed fixture (M1)

**Files:**
- Create: `tools/deluge-web-sim/src/deluge-layout.ts` (copy of `tools/wren-web/app/src/deluge-layout.ts`, verbatim)
- Create: `tools/deluge-web-sim/src/panel.ts` (copy of `tools/wren-web/app/src/panel.ts`, slimmed — see edits)
- Modify: `tools/deluge-web-sim/index.html` (faceplate DOM)
- Modify: `tools/deluge-web-sim/src/style.css` (faceplate styles copied from wren-web)
- Modify: `tools/deluge-web-sim/src/main.ts` (build `PanelClient` + `Panel`, render loop, replay hook)
- Create: `tools/deluge-web-sim/playwright.config.ts`
- Test: `tools/deluge-web-sim/tests/render.spec.ts`

**Interfaces:**
- Consumes: `PanelClient` (Task 4), `protocol.ts` constants.
- Produces: `class Panel` with `constructor(oled: HTMLCanvasElement, faceOverlay: HTMLElement, client: PanelClient)` and `frame(): void`. A `window.__replay(frames: number[][])` hook on the page that feeds `[type,...data]` bodies into the client for the e2e test.

- [ ] **Step 1: Copy `deluge-layout.ts` verbatim**

Run: `cp tools/wren-web/app/src/deluge-layout.ts tools/deluge-web-sim/src/deluge-layout.ts`
(No edits — the coordinate tables are the shared faceplate geometry.)

- [ ] **Step 2: Copy `panel.ts` and slim it**

Run: `cp tools/wren-web/app/src/panel.ts tools/deluge-web-sim/src/panel.ts`

Then apply these edits to `tools/deluge-web-sim/src/panel.ts`:

1. Replace the top import block:
```ts
import { PanelClient } from "./panel-client";
import { OLED_W, OLED_H, PAD_ROWS } from "./protocol";
import { BUTTONS, ENCODERS, PAD, FACE_W, FACE_H } from "./deluge-layout";

const OLED_SCALE = 3;
```
(Removes the `Sim`, `HIST`, `CV_MAX` symbols.)

2. Replace the class fields block (lines declaring `cvEls`/`gateEls`/`midiMon`/`cvScope`/`cvScopeCtx`/`cvHist`/`gateHist`/`histPos`/`keyBase`) with only:
```ts
  private oledCtx: CanvasRenderingContext2D;
  private ledEls = new Map<number, HTMLElement>(); // rawId → button element
  private knobEls = new Map<number, HTMLElement[]>(); // gold encoder index → 4 segment dots
```

3. Replace the constructor with:
```ts
  constructor(
    oled: HTMLCanvasElement,
    private faceOverlay: HTMLElement,
    private client: PanelClient,
  ) {
    oled.width = OLED_W * OLED_SCALE;
    oled.height = OLED_H * OLED_SCALE;
    this.oledCtx = oled.getContext("2d")!;
    this.buildPads();
    this.buildButtons();
    this.buildEncoders();
  }
```

4. In `buildPads`, `buildButtons`, `buildEncoders`: replace every `this.sim.` with `this.client.` (input calls `pad`/`button`/`enc` are identical on `PanelClient`). In `buildEncoders`, after creating the gold-encoder dial for an encoder with `e.gold`, add four indicator segments and register them:
```ts
      if (e.gold) {
        const segs: HTMLElement[] = [];
        for (let s = 0; s < 4; s++) {
          const dot = document.createElement("span");
          dot.className = "knob-seg";
          dot.style.transform = `rotate(${-60 + s * 40}deg) translateY(-140%)`;
          enc.querySelector(".enc-knob")!.appendChild(dot);
          segs.push(dot);
        }
        this.knobEls.set(e.index, segs);
      }
```

5. Delete these methods entirely: `buildCv`, `initOctaveControls`, `buildKeyboard`, `logMidi`, `logMidiTx`, `drawCvScope`.

6. Replace `frame()` with:
```ts
  frame() {
    // OLED phosphor render.
    const px = this.client.oled();
    const ctx = this.oledCtx;
    ctx.fillStyle = "#0a0e10";
    ctx.fillRect(0, 0, OLED_W * OLED_SCALE, OLED_H * OLED_SCALE);
    ctx.fillStyle = "#8fe9ff";
    ctx.shadowColor = "#8fe9ff";
    ctx.shadowBlur = OLED_SCALE * 1.5;
    for (let y = 0; y < OLED_H; y++) {
      for (let x = 0; x < OLED_W; x++) {
        if (px[y * OLED_W + x]) ctx.fillRect(x * OLED_SCALE, y * OLED_SCALE, OLED_SCALE - 0.5, OLED_SCALE - 0.5);
      }
    }
    ctx.shadowBlur = 0;

    // Indicator LEDs light their front-panel buttons (Led.on(id)).
    const leds = this.client.leds();
    for (const [rawId, el] of this.ledEls) el.classList.toggle("lit", leds[rawId] !== 0);

    // Gold-knob indicator rings (4 segments each).
    for (const [which, segs] of this.knobEls) {
      const levels = this.client.knob(which);
      segs.forEach((dot, s) => dot.classList.toggle("on", levels[s] > 0));
    }
  }
```

Note `PAD_ROWS` is still imported (used by `buildPads`).

- [ ] **Step 3: Fill the faceplate DOM in `index.html`**

Replace the `<main id="app">…</main>` block with:
```html
    <main id="app">
      <div class="topbar">
        <label>brain <input id="ws-url" value="ws://127.0.0.1:9000" size="24" /></label>
        <button id="connect">Connect</button>
        <span id="status">idle</span>
      </div>
      <canvas id="oled" role="img" aria-label="OLED display"></canvas>
      <div id="faceplate" class="faceplate" data-face-theme="hardware">
        <div id="face-overlay" class="face-overlay"></div>
      </div>
    </main>
```

- [ ] **Step 4: Copy the faceplate styles**

From `tools/wren-web/app/src/style.css`, copy the rules for these selectors into `tools/deluge-web-sim/src/style.css` (append after the existing base rules): `.faceplate`, `.face-overlay`, `.pad`, `.pad.lit`, `.fbtn`, `.fbtn.fn`, `.fbtn.pressed`, `.fbtn.lit`, `.enc`, `.enc.gold`, `.enc.inert`, `.enc-knob`, `.enc-dial`, `.enc-tick`, `#oled`. Add the new knob-segment rule:
```css
.knob-seg { position: absolute; left: 50%; top: 50%; width: 8%; height: 8%; margin: -4%; border-radius: 50%; background: #2a2f33; }
.knob-seg.on { background: #f2b549; box-shadow: 0 0 4px #f2b549; }
.topbar { display: flex; gap: .5rem; align-items: center; margin-bottom: .75rem; font-size: .85rem; }
```

Verify visually in Step 8 that pads/buttons/encoders are positioned; adjust copied rules if any selector was missed.

- [ ] **Step 5: Wire `main.ts` (render loop + replay hook)**

Replace `src/main.ts` with:
```ts
import { Panel } from "./panel";
import { PanelClient } from "./panel-client";

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;

// Until Task 6 adds the WebSocket, input is discarded and frames arrive only via
// the __replay test hook. The connect button is wired in Task 6.
const client = new PanelClient(() => {});
const panel = new Panel($<HTMLCanvasElement>("#oled"), $("#face-overlay"), client);

// e2e hook: feed [type, ...data] message bodies straight into the client.
(window as unknown as { __replay: (frames: number[][]) => void }).__replay = (frames) => {
  for (const f of frames) client.onMessage(Uint8Array.from(f));
};

function tick() {
  panel.frame();
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);
$("#status").textContent = "ready";
```

- [ ] **Step 6: Create `playwright.config.ts`**

```ts
import { defineConfig, devices } from "@playwright/test";

const PORT = 4273;

export default defineConfig({
  testDir: "./tests",
  fullyParallel: false,
  workers: 1,
  reporter: [["list"]],
  use: { baseURL: `http://localhost:${PORT}`, trace: "on-first-retry" },
  projects: [{ name: "chromium", use: { ...devices["Desktop Chrome"] } }],
  webServer: {
    command: `npm run build && npm run preview -- --port ${PORT} --strictPort`,
    url: `http://localhost:${PORT}`,
    reuseExistingServer: !process.env.CI,
    timeout: 180_000,
  },
});
```

- [ ] **Step 7: Write the M1 render e2e test**

Create `tests/render.spec.ts`:
```ts
import { test, expect } from "@playwright/test";

test("renders LEDs and OLED from replayed frames", async ({ page }) => {
  await page.goto("/");
  await expect(page.locator("#status")).toHaveText("ready");

  // Build an UpdateDisplay lighting ui_row 0, col 0 (panel row 5 → page 0, bit 5),
  // plus SetLed for PLAY (index 35).
  const fb = new Array(768).fill(0);
  fb[0] = 1 << 5;
  await page.evaluate(
    ([display]) => {
      (window as any).__replay([[0x20, ...display], [0x24, 35, 1]]);
    },
    [fb],
  );

  // PLAY button (aria-label PLAY) becomes lit.
  await expect(page.locator('[aria-label="PLAY"]')).toHaveClass(/lit/);

  // OLED canvas has a lit pixel near the top-left (non-background colour).
  const lit = await page.evaluate(() => {
    const c = document.querySelector<HTMLCanvasElement>("#oled")!;
    const px = c.getContext("2d")!.getImageData(1, 1, 1, 1).data;
    return px[0] + px[1] + px[2] > 60; // brighter than #0a0e10 background
  });
  expect(lit).toBe(true);
});
```

- [ ] **Step 8: Install browsers, run e2e, and eyeball the faceplate**

Run: `cd tools/deluge-web-sim && npx playwright install chromium && npm run test:e2e`
Expected: PASS. If a selector class is missing (pads/buttons unpositioned), fix the copied CSS in Step 4 and re-run.
Also run `npm run dev` once and confirm in a browser that pads, buttons, and encoders appear at faceplate positions.

- [ ] **Step 9: Commit**

```bash
git add tools/deluge-web-sim/src/deluge-layout.ts tools/deluge-web-sim/src/panel.ts \
  tools/deluge-web-sim/src/main.ts tools/deluge-web-sim/src/style.css \
  tools/deluge-web-sim/index.html tools/deluge-web-sim/playwright.config.ts \
  tools/deluge-web-sim/tests/render.spec.ts
git commit -m "feat(deluge-web-sim): faceplate rendering from replayed frames (M1)"
```

---

### Task 6: WebSocket transport + connect UI (M2)

**Files:**
- Create: `tools/deluge-web-sim/src/connection.ts`
- Modify: `tools/deluge-web-sim/src/main.ts` (wire connect/disconnect UI)
- Test: `tools/deluge-web-sim/tests/e2e-ws.spec.ts`

**Interfaces:**
- Consumes: `PanelClient` (Task 4).
- Produces: `class Connection` —
  - `constructor(client: PanelClient, onStatus: (s: "connecting" | "open" | "closed") => void)`
  - `connect(url: string): void` — opens a binary WebSocket, sends `Ready` on open, feeds each inbound binary message to `client.onMessage`, and rebinds `client`'s `send` to the socket.
  - `disconnect(): void`.

  Because `PanelClient.send` is set at construction, `Connection` needs to route the client's outbound bytes to the live socket. Add one method to `PanelClient` (Task 4 file): `setSend(fn: (bytes: Uint8Array) => void)` that reassigns the private `send`. Add it now.

- [ ] **Step 1: Add `setSend` to `PanelClient`**

In `src/panel-client.ts`, add inside the class:
```ts
  setSend(fn: (bytes: Uint8Array) => void) { this.send = fn; }
```
Change the constructor parameter from `private send:` to keep the field but allow reassignment — it already is a mutable field via `private send`, so no other change is needed.

- [ ] **Step 2: Write the failing e2e test**

Create `tests/e2e-ws.spec.ts`:
```ts
import { test, expect } from "@playwright/test";

// A mock WS brain runs inside the page via a stubbed WebSocket so no server is
// needed: we intercept construction, capture sent bytes, and push frames in.
test("connects, renders inbound frames, and sends input", async ({ page }) => {
  await page.addInitScript(() => {
    const sent: number[][] = [];
    (window as any).__sent = sent;
    class FakeWS {
      static OPEN = 1;
      readyState = 1;
      binaryType = "arraybuffer";
      onopen: (() => void) | null = null;
      onmessage: ((e: { data: ArrayBuffer }) => void) | null = null;
      onclose: (() => void) | null = null;
      constructor(public url: string) { setTimeout(() => this.onopen?.(), 0); }
      send(data: ArrayBuffer) { sent.push([...new Uint8Array(data)]); }
      close() { this.onclose?.(); }
      pushFrame(bytes: number[]) { this.onmessage?.({ data: Uint8Array.from(bytes).buffer }); }
    }
    (window as any).__lastWS = null;
    const orig = (window as any).WebSocket;
    (window as any).WebSocket = new Proxy(FakeWS, {
      construct(t, args) { const ws = new (t as any)(...args); (window as any).__lastWS = ws; return ws; },
    });
    void orig;
  });

  await page.goto("/");
  await page.fill("#ws-url", "ws://127.0.0.1:9000");
  await page.click("#connect");
  await expect(page.locator("#status")).toHaveText(/open/);

  // On open the client sends Ready (0x12).
  await expect.poll(() => page.evaluate(() => (window as any).__sent[0])).toEqual([0x12]);

  // Brain lights PLAY; the button becomes lit.
  await page.evaluate(() => (window as any).__lastWS.pushFrame([0x24, 35, 1]));
  await expect(page.locator('[aria-label="PLAY"]')).toHaveClass(/lit/);

  // Clicking a pad sends PadPressed (0x01) then PadReleased (0x02).
  await page.locator('[aria-label="pad 1, 1"]').click();
  const sent = await page.evaluate(() => (window as any).__sent);
  expect(sent.some((f: number[]) => f[0] === 0x01)).toBe(true);
  expect(sent.some((f: number[]) => f[0] === 0x02)).toBe(true);
});
```

- [ ] **Step 3: Run to verify it fails**

Run: `cd tools/deluge-web-sim && npm run test:e2e -- tests/e2e-ws.spec.ts`
Expected: FAIL — no `#connect` handler yet (status never becomes "open").

- [ ] **Step 4: Implement `src/connection.ts`**

```ts
// Owns the WebSocket and bridges it to a PanelClient: inbound binary messages →
// client.onMessage; client outbound bytes → socket. One binary message carries
// one [type, ...data] frame body.
import type { PanelClient } from "./panel-client";

export class Connection {
  private ws: WebSocket | null = null;

  constructor(
    private client: PanelClient,
    private onStatus: (s: "connecting" | "open" | "closed") => void,
  ) {}

  connect(url: string) {
    this.disconnect();
    this.onStatus("connecting");
    const ws = new WebSocket(url);
    ws.binaryType = "arraybuffer";
    this.ws = ws;
    this.client.setSend((bytes) => {
      if (ws.readyState === WebSocket.OPEN) ws.send(bytes);
    });
    ws.onopen = () => { this.onStatus("open"); this.client.sendReady(); };
    ws.onmessage = (e) => this.client.onMessage(new Uint8Array(e.data as ArrayBuffer));
    ws.onclose = () => { this.onStatus("closed"); this.client.setSend(() => {}); };
    ws.onerror = () => ws.close();
  }

  disconnect() {
    if (this.ws) { this.ws.onclose = null; this.ws.close(); this.ws = null; }
    this.client.setSend(() => {});
  }
}
```

- [ ] **Step 5: Wire the connect UI in `main.ts`**

Replace `src/main.ts` with:
```ts
import { Panel } from "./panel";
import { PanelClient } from "./panel-client";
import { Connection } from "./connection";

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;

const client = new PanelClient(() => {});
const panel = new Panel($<HTMLCanvasElement>("#oled"), $("#face-overlay"), client);
const status = $("#status");
const conn = new Connection(client, (s) => { status.textContent = s; });

$("#connect").addEventListener("click", () => {
  conn.connect($<HTMLInputElement>("#ws-url").value);
});

// e2e hook retained for the M1 render test.
(window as unknown as { __replay: (frames: number[][]) => void }).__replay = (frames) => {
  for (const f of frames) client.onMessage(Uint8Array.from(f));
};

function tick() { panel.frame(); requestAnimationFrame(tick); }
requestAnimationFrame(tick);
status.textContent = "ready";
```

- [ ] **Step 6: Run both e2e specs**

Run: `cd tools/deluge-web-sim && npm run test:e2e`
Expected: PASS — `render.spec.ts` (M1) and `e2e-ws.spec.ts` (M2) both green.

- [ ] **Step 7: Commit**

```bash
git add tools/deluge-web-sim/src/connection.ts tools/deluge-web-sim/src/panel-client.ts \
  tools/deluge-web-sim/src/main.ts tools/deluge-web-sim/tests/e2e-ws.spec.ts
git commit -m "feat(deluge-web-sim): WebSocket transport + connect UI (M2)"
```

The full browser side is now complete and tested against a mock brain. Task 7 makes a real brain reachable.

---

### Task 7: WebSocket listener in DelugeFirmware `host_link.c` (M3)

**Repo:** `/home/kate/GitHub/DelugeFirmware` (not `deluge-sdk`). All paths below are relative to that repo.

**Files:**
- Create: `src/bsp/host/host_ws.h`
- Create: `src/bsp/host/host_ws.c`
- Modify: `src/bsp/host/host_link.c` (recognise `ws://` target; call `host_ws` after `accept()` and in send/recv)
- Test: `src/bsp/host/host_ws_test.c` (standalone unit test for handshake + frame codec)

**Interfaces:**
- Consumes: the existing `host_link.c` accepted-socket fd and its `[len][type][data]` framing.
- Produces (in `host_ws.h`):
  - `int host_ws_accept(int fd);` — read the HTTP upgrade request from `fd`, write the `101` response; return 0 on success, −1 on failure.
  - `int host_ws_send(int fd, const uint8_t* payload, size_t n);` — write one unmasked binary WS frame. Return 0/−1.
  - `int host_ws_recv_frame(int fd, uint8_t* out, size_t cap, size_t* out_len);` — read one binary frame, unmask, store payload; handle/skip control frames (reply to ping, return −1 on close). Return 0 on a data frame, 1 if only a control frame was handled, −1 on error/close.
  - `void host_ws_accept_key(const char* client_key, char out_accept[29]);` — pure helper: `base64(SHA1(client_key + GUID))`. Exposed for the unit test.

- [ ] **Step 1: Write the failing unit test**

Create `src/bsp/host/host_ws_test.c`:
```c
// Standalone: cc -I. src/bsp/host/host_ws.c src/bsp/host/host_ws_test.c -o /tmp/ws_test && /tmp/ws_test
#include "host_ws.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

int main(void) {
    // RFC 6455 §1.3 canonical example.
    char accept[29];
    host_ws_accept_key("dGhlIHNhbXBsZSBub25jZQ==", accept);
    assert(strcmp(accept, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") == 0);
    printf("ok\n");
    return 0;
}
```

- [ ] **Step 2: Run it to confirm it fails (no implementation yet)**

Run: `cd /home/kate/GitHub/DelugeFirmware && cc -Isrc/bsp/host src/bsp/host/host_ws.c src/bsp/host/host_ws_test.c -o /tmp/ws_test`
Expected: FAIL to link/compile — `host_ws.c`/`host_ws.h` don't exist.

- [ ] **Step 3: Implement `host_ws.h` and `host_ws.c`**

Create `src/bsp/host/host_ws.h`:
```c
#pragma once
#include <stddef.h>
#include <stdint.h>

// Compute Sec-WebSocket-Accept: base64(SHA1(client_key + magic-GUID)).
// out_accept must hold 29 bytes (28 chars + NUL).
void host_ws_accept_key(const char* client_key, char out_accept[29]);

// Perform the server handshake on an accepted fd. 0 on success, -1 on failure.
int host_ws_accept(int fd);

// Send one unmasked binary frame. 0 on success, -1 on error.
int host_ws_send(int fd, const uint8_t* payload, size_t n);

// Read one frame. Returns 0 for a data frame (payload in out, length in out_len),
// 1 if only a control frame was handled (caller should retry), -1 on close/error.
int host_ws_recv_frame(int fd, uint8_t* out, size_t cap, size_t* out_len);
```

Create `src/bsp/host/host_ws.c` implementing:
- A small public-domain SHA1 (vendored inline) and a base64 encoder.
- `host_ws_accept_key`: concatenate `client_key` + `"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"`, SHA1, base64 into `out_accept`.
- `host_ws_accept`: `recv` the HTTP request into a buffer until `"\r\n\r\n"`, find `Sec-WebSocket-Key:`, compute the accept key, `send` the response:
  ```
  HTTP/1.1 101 Switching Protocols\r\n
  Upgrade: websocket\r\n
  Connection: Upgrade\r\n
  Sec-WebSocket-Accept: <key>\r\n\r\n
  ```
- `host_ws_send`: build a header — byte0 `0x82` (FIN + binary opcode); length: `n<126` → 1 byte; `n<65536` → `126` + 2-byte big-endian; server frames are unmasked (mask bit 0). `write` header then payload.
- `host_ws_recv_frame`: read 2 header bytes; opcode `0x8` (close) → return −1; `0x9` (ping) → read payload, send a pong (`0x8A`), return 1; `0x2`/`0x0` (binary/continuation) → decode length (7-bit / 16-bit / 64-bit), read the 4-byte mask (client frames MUST be masked), read and unmask the payload into `out`, set `*out_len`, return 0.

(Implement with blocking `read`/`write` helpers that loop until the full count is transferred, mirroring `host_link.c`'s existing `write` loop.)

- [ ] **Step 4: Run the unit test to verify it passes**

Run: `cd /home/kate/GitHub/DelugeFirmware && cc -Isrc/bsp/host src/bsp/host/host_ws.c src/bsp/host/host_ws_test.c -o /tmp/ws_test && /tmp/ws_test`
Expected: prints `ok` (the RFC 6455 accept-key vector matches).

- [ ] **Step 5: Wire WS into `host_link.c`**

In `src/bsp/host/host_link.c`:
1. Add `#include "host_ws.h"` and a `static bool link_is_ws = false;` flag.
2. In the target parser, recognise a `ws://[host:]port` form: set `link_is_ws = true` and `link_is_tcp = true`, and reuse `link_listen_tcp(port)`.
3. After `link_conn_fd = accept(...)`, if `link_is_ws`, call `host_ws_accept(link_conn_fd)`; on −1, close and bail.
4. In `host_link_send`: when `link_is_ws`, build the `[len][type][data]` frame as today but send its **body** (`type` + `data`, i.e. skip the 2 length bytes) via `host_ws_send`; otherwise the existing raw `write`.
5. In `host_link_recv`: when `link_is_ws`, source bytes via `host_ws_recv_frame` (looping while it returns 1) into the existing `rx_buf`/decoder path — each WS binary frame is exactly one `[type][data]` body, so hand `(type=body[0], data=body+1, len-1)` straight to the caller without the length-prefix reassembler.

- [ ] **Step 6: Build the host emulator with WS and smoke-test end-to-end**

Run (DelugeFirmware): `cmake -B build-sim -S sim -G Ninja && cmake --build build-sim --target deluge_host`
Then start the brain on a WS target:
`DELUGE_HOST_LINK=ws://:9000 ./build-sim/deluge_host`
In `tools/deluge-web-sim`: `npm run dev`, open the app, Connect to `ws://127.0.0.1:9000`.
Expected: status shows "open"; the OLED and LEDs render as the firmware drives them; clicking pads/buttons and turning encoders affects the firmware. (This is the manual M3 acceptance — the automated pin is the golden-vector + mock-brain e2e from Tasks 3/6.)

- [ ] **Step 7: Commit (in the DelugeFirmware repo)**

```bash
cd /home/kate/GitHub/DelugeFirmware
git add src/bsp/host/host_ws.h src/bsp/host/host_ws.c src/bsp/host/host_ws_test.c src/bsp/host/host_link.c
git commit -m "feat(host_link): WebSocket transport for the browser panel"
```

---

## Self-Review

**Spec coverage:**
- §1 goals (faceplate render, input, existing brain over WS, static assets, no drift) → Tasks 5/6 (render+input+WS), Task 7 (brain WS), Task 3 (drift guard). ✓
- §1 non-goals (audio/MIDI/CV-gate/wasm-brain out; still decoded/stored) → Task 4 decodes+stores CV/gate/synced/brightness without rendering; Global Constraints restate it. ✓
- §3 two new components + copy → `protocol.ts`+`panel-client.ts` (Tasks 2/4), `connection.ts` (Task 6), copied `panel.ts`/`deluge-layout.ts` (Task 5). ✓
- §3 wire mapping (one frame/WS message, no reassembler) → Global Constraints + Task 6/7. ✓
- §3 handshake (Ready on open; Ping→Pong; GetVersion→Version) → Task 4 + Task 6. ✓
- §4 native WS listener (handshake, framing, third transport) → Task 7. ✓
- §5 browser components + slimming → Tasks 2/4/5/6. ✓
- §6 testing (golden vectors, Playwright vs mock brain, e2e smoke) → Tasks 3/5/6/7. ✓
- §7 milestones M0–M3 → M0=Tasks 2/3, M1=Task 5, M2=Task 6, M3=Task 7. ✓
- §8 cross-repo (M0–M2 in deluge-sdk, M3 in DelugeFirmware) → Task 7 header flags the repo. ✓

**Placeholder scan:** No TBD/TODO; every code step shows complete code. The one prose-only implementation (Task 7 Step 3, `host_ws.c`) enumerates each function's exact behaviour and is pinned by the RFC 6455 accept-key unit test in Steps 1/4.

**Type consistency:** `PanelClient` method surface (`pad`/`button`/`enc`/`oled`/`leds`/`knob`/`onMessage`/`sendReady`/`setSend`) is identical across Tasks 4/5/6. `Panel` constructor `(oled, faceOverlay, client)` matches between Task 5 definition and Task 5/6 `main.ts` construction. Golden JSON shape `{name,dir,body}` identical in Task 3 Rust generator and TS consumer. OLED constants (`OLED_H=43`, `PANEL_TOP=5`) consistent across `protocol.ts`, `panel-client.ts`, and the render loop.
