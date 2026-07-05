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
