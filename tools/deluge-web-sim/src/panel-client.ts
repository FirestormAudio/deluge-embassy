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

  // Rebind the outbound sink (e.g. to a live WebSocket, or back to a no-op on close).
  setSend(fn: (bytes: Uint8Array) => void) {
    this.send = fn;
  }

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
