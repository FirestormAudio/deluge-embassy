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
