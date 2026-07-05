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
