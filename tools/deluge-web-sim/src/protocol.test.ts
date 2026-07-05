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
