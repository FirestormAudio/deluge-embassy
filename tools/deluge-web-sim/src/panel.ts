// The instrument: a faithful Deluge faceplate (pads, named buttons + LEDs, named
// encoders placed at their real positions, see deluge-layout.ts) plus a separate
// large OLED. Input widgets call the PanelClient; output widgets read it each
// frame. Slimmed from tools/wren-web/app's Panel: no CV/gate scope, MIDI monitor,
// or on-screen keyboard — this faceplate is driven purely by the deluge-protocol
// wire (illumination in, input out).
import { PanelClient } from "./panel-client";
import { OLED_W, OLED_H, PAD_ROWS } from "./protocol";
import { BUTTONS, ENCODERS, PAD, FACE_W, FACE_H } from "./deluge-layout";

const OLED_SCALE = 3;

/// Place an element centered at SVG (cx, cy) with SVG size (w, h), as a
/// percentage of the faceplate box (which has aspect-ratio FACE_W/FACE_H, so
/// equal-scaled width/height keep circles round).
function place(el: HTMLElement, cx: number, cy: number, w: number, h: number) {
  el.style.left = `${(cx / FACE_W) * 100}%`;
  el.style.top = `${(cy / FACE_H) * 100}%`;
  el.style.width = `${(w / FACE_W) * 100}%`;
  el.style.height = `${(h / FACE_H) * 100}%`;
}

export class Panel {
  private oledCtx: CanvasRenderingContext2D;
  private padEls = new Map<number, HTMLElement>(); // (col*PAD_ROWS + wy) → pad element
  private ledEls = new Map<number, HTMLElement>(); // rawId → button element
  private knobEls = new Map<number, HTMLElement[]>(); // gold encoder index → 4 segment dots

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

  // ── Faceplate (placed at real coordinates) ──────────────────────────────────

  private buildPads() {
    const cols = PAD.colOffsets.length;
    for (let svgRow = 0; svgRow < PAD.rowOffsets.length; svgRow++) {
      for (let col = 0; col < cols; col++) {
        const wy = PAD_ROWS - 1 - svgRow; // hardware: bottom-left = (0,0)
        const pad = document.createElement("button");
        pad.className = "pad";
        pad.setAttribute("aria-label", `pad ${col + 1}, ${wy + 1}`);
        const hue = (col / cols) * 320 + svgRow * 6;
        pad.style.setProperty("--hue", String(hue));
        place(
          pad,
          PAD.baseX + PAD.colOffsets[col] + PAD.size / 2,
          PAD.baseY + PAD.rowOffsets[svgRow] + PAD.size / 2,
          PAD.size,
          PAD.size,
        );
        const press = (down: boolean) => { pad.classList.toggle("lit", down); this.client.pad(col, wy, down); };
        pad.addEventListener("pointerdown", (e) => { e.preventDefault(); press(true); });
        pad.addEventListener("pointerup", () => press(false));
        pad.addEventListener("pointerleave", () => pad.classList.contains("lit") && press(false));
        this.faceOverlay.appendChild(pad);
        this.padEls.set(col * PAD_ROWS + wy, pad);
      }
    }
  }

  private buildButtons() {
    for (const b of BUTTONS) {
      const el = document.createElement("button");
      el.className = "fbtn" + (b.fn ? " fn" : "");
      el.setAttribute("aria-label", b.name);
      el.title = b.name;
      place(el, b.cx, b.cy, b.r * 2, b.r * 2);
      const press = (down: boolean) => { el.classList.toggle("pressed", down); this.client.button(b.rawId, down); };
      el.addEventListener("pointerdown", (e) => { e.preventDefault(); press(true); });
      el.addEventListener("pointerup", () => press(false));
      el.addEventListener("pointerleave", () => el.classList.contains("pressed") && press(false));
      this.faceOverlay.appendChild(el);
      this.ledEls.set(b.rawId, el);
    }
  }

  private buildEncoders() {
    for (const e of ENCODERS) {
      const enc = document.createElement("div");
      enc.className = "enc" + (e.gold ? " gold" : "") + (e.index < 0 ? " inert" : "");
      const interactive = e.index >= 0;
      if (interactive) {
        enc.tabIndex = 0;
        enc.setAttribute("role", "slider");
      }
      enc.setAttribute("aria-label", `${e.name} encoder`);
      enc.title = e.name;
      // The bezel (.enc-knob) stays static; only .enc-dial (holding the tick)
      // rotates, so the knob's shadow/highlight doesn't spin with it.
      enc.innerHTML = `<div class="enc-knob"><div class="enc-dial"><span class="enc-tick"></span></div></div>`;
      place(enc, e.cx, e.cy, e.r * 2, e.r * 2);
      const dial = enc.querySelector<HTMLElement>(".enc-dial")!;
      this.faceOverlay.appendChild(enc);

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
      if (!interactive) continue;

      let angle = 0;
      const turn = (d: number) => { angle += d * 20; dial.style.transform = `rotate(${angle}deg)`; this.client.enc(e.index, d); };
      enc.addEventListener("wheel", (ev) => { ev.preventDefault(); turn(ev.deltaY < 0 ? 1 : -1); }, { passive: false });
      enc.addEventListener("keydown", (ev) => {
        if (ev.key === "ArrowUp" || ev.key === "ArrowRight") { ev.preventDefault(); turn(1); }
        else if (ev.key === "ArrowDown" || ev.key === "ArrowLeft") { ev.preventDefault(); turn(-1); }
      });
      // Vertical drag = detents; a small drag (treated as a click) = shaft press.
      let dragging = false, lastY = 0, accum = 0, moved = 0;
      enc.addEventListener("pointerdown", (ev) => { dragging = true; lastY = ev.clientY; accum = 0; moved = 0; enc.setPointerCapture(ev.pointerId); });
      enc.addEventListener("pointermove", (ev) => {
        if (!dragging) return;
        const dy = lastY - ev.clientY; lastY = ev.clientY; accum += dy; moved += Math.abs(dy);
        while (accum >= 8) { accum -= 8; turn(1); }
        while (accum <= -8) { accum += 8; turn(-1); }
      });
      enc.addEventListener("pointerup", () => {
        dragging = false;
        if (moved < 4 && e.pushId >= 0) { // a tap on the shaft
          this.client.button(e.pushId, true);
          this.client.button(e.pushId, false);
        }
      });
    }
  }

  // ── Per-frame output ────────────────────────────────────────────────────────

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

    // Pad RGB from the wire (col-major (col*8+row)*3).
    const pads = this.client.padRgb();
    for (const [key, el] of this.padEls) {
      const o = key * 3;
      const r = pads[o], g = pads[o + 1], b = pads[o + 2];
      if (r || g || b) {
        el.style.background = `rgb(${r},${g},${b})`;
        el.style.boxShadow = `0 0 8px 1px rgb(${r},${g},${b})`;
      } else {
        el.style.background = "";
        el.style.boxShadow = "";
      }
    }

    // Indicator LEDs light their front-panel buttons (Led.on(id)).
    const leds = this.client.leds();
    for (const [rawId, el] of this.ledEls) el.classList.toggle("lit", leds[rawId] !== 0);

    // Gold-knob indicator rings (4 segments each).
    for (const [which, segs] of this.knobEls) {
      const levels = this.client.knob(which);
      segs.forEach((dot, s) => dot.classList.toggle("on", levels[s] > 0));
    }
  }
}
