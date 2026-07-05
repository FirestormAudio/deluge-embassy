// The instrument: a faithful Deluge faceplate (pads, named buttons + LEDs, named
// encoders placed at their real positions, see deluge-layout.ts), a separate
// large OLED, and a "rack" of sim-only instruments (CV/gate timeline, MIDI
// monitor, keyboard). Input widgets call the Sim; output widgets read it each
// frame. The visual surface is theme-switched in CSS via #faceplate's
// data-face-theme; this module is theme-agnostic.
import { Sim, OLED_W, OLED_H, PAD_ROWS } from "./sim";
import { BUTTONS, ENCODERS, PAD, FACE_W, FACE_H } from "./deluge-layout";

const OLED_SCALE = 3;
const HIST = 240; // ~4 s of CV/gate history at 60 fps
const CV_MAX = 10; // volts mapped to full scope height

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
  private cvEls: HTMLElement[] = [];
  private gateEls: HTMLElement[] = [];
  private ledEls = new Map<number, HTMLElement>(); // rawId → button element

  private midiMon = document.querySelector<HTMLElement>("#midi-monitor");
  private cvScope = document.querySelector<HTMLCanvasElement>("#cv-scope");
  private cvScopeCtx: CanvasRenderingContext2D | null = null;
  private cvHist = [new Float32Array(HIST), new Float32Array(HIST)];
  private gateHist = new Uint8Array(HIST);
  private histPos = 0;
  private keyBase = 48; // lowest MIDI note on the on-screen keyboard (octave-shiftable)

  constructor(
    oled: HTMLCanvasElement,
    private faceOverlay: HTMLElement,
    private cvRow: HTMLElement,
    private keyboard: HTMLElement,
    private sim: Sim,
  ) {
    oled.width = OLED_W * OLED_SCALE;
    oled.height = OLED_H * OLED_SCALE;
    this.oledCtx = oled.getContext("2d")!;
    this.buildPads();
    this.buildButtons();
    this.buildEncoders();
    this.buildCv();
    this.buildKeyboard();
    this.initOctaveControls();
    if (this.cvScope) {
      this.cvScope.width = 388;
      this.cvScope.height = 60;
      this.cvScopeCtx = this.cvScope.getContext("2d");
    }
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
        const press = (down: boolean) => { pad.classList.toggle("lit", down); this.sim.pad(col, wy, down); };
        pad.addEventListener("pointerdown", (e) => { e.preventDefault(); press(true); });
        pad.addEventListener("pointerup", () => press(false));
        pad.addEventListener("pointerleave", () => pad.classList.contains("lit") && press(false));
        this.faceOverlay.appendChild(pad);
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
      const press = (down: boolean) => { el.classList.toggle("pressed", down); this.sim.button(b.rawId, down); };
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
      if (!interactive) continue;

      let angle = 0;
      const turn = (d: number) => { angle += d * 20; dial.style.transform = `rotate(${angle}deg)`; this.sim.enc(e.index, d); };
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
          this.sim.button(e.pushId, true);
          this.sim.button(e.pushId, false);
        }
      });
    }
  }

  // ── Rack (sim-only instruments) ─────────────────────────────────────────────

  private buildCv() {
    const gates = document.createElement("div");
    gates.className = "gates";
    gates.innerHTML = `<span class="jack-legend">GATE</span>`;
    for (let g = 0; g < 4; g++) {
      const el = document.createElement("span");
      el.className = "gate-led";
      gates.appendChild(el);
      this.gateEls.push(el);
    }
    this.cvRow.appendChild(gates);

    const cvJacks = document.createElement("div");
    cvJacks.className = "cv-jacks";
    for (let ch = 0; ch < 2; ch++) {
      const jack = document.createElement("div");
      jack.className = "jack";
      jack.innerHTML = `<span class="jack-led"></span><span class="jack-legend">CV${ch + 1}</span><span class="jack-val">0.00<i>V</i></span>`;
      cvJacks.appendChild(jack);
      this.cvEls.push(jack.querySelector(".jack-val")!);
    }
    this.cvRow.appendChild(cvJacks);
  }

  /// Wire octave −/+ buttons (in the MIDI-in legend) to shift the keyboard range.
  private initOctaveControls() {
    const down = document.querySelector<HTMLButtonElement>("#oct-down");
    const up = document.querySelector<HTMLButtonElement>("#oct-up");
    const shift = (delta: number) => {
      this.keyBase = Math.min(108, Math.max(12, this.keyBase + delta));
      this.keyboard.replaceChildren();
      this.buildKeyboard();
    };
    down?.addEventListener("click", () => shift(-12));
    up?.addEventListener("click", () => shift(12));
  }

  private buildKeyboard() {
    const base = this.keyBase;
    const count = 17; // notes base..base+16
    const isBlack = (n: number) => [1, 3, 6, 8, 10].includes(n % 12);
    const names = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"];
    const noteName = (n: number) => `${names[n % 12]}${Math.floor(n / 12) - 1}`;
    const octLabel = document.querySelector("#oct-label");
    if (octLabel) octLabel.textContent = noteName(base);

    const makeKey = (note: number, black: boolean) => {
      const key = document.createElement("button");
      key.className = "key" + (black ? " black" : "");
      key.setAttribute("aria-label", `note ${noteName(note)}`);
      const down = (on: boolean) => {
        key.classList.toggle("down", on);
        const status = on ? 0x90 : 0x80;
        this.sim.midiIn(status, note, on ? 100 : 0);
        this.logMidi(status, note, on ? 100 : 0, "in");
      };
      key.addEventListener("pointerdown", (e) => { e.preventDefault(); down(true); });
      key.addEventListener("pointerup", () => down(false));
      key.addEventListener("pointerleave", () => key.classList.contains("down") && down(false));
      return key;
    };

    // Split into a flush row of white keys + black keys overlaid on the white
    // boundaries (`before` = number of white keys to the black key's left).
    const whites: number[] = [];
    const blacks: { note: number; before: number }[] = [];
    for (let i = 0; i < count; i++) {
      const note = base + i;
      if (isBlack(note)) blacks.push({ note, before: whites.length });
      else whites.push(note);
    }
    const whiteW = 100 / whites.length;
    const blackW = whiteW * 0.62;

    const inner = document.createElement("div");
    inner.className = "kb-inner";
    const row = document.createElement("div");
    row.className = "key-row";
    for (const note of whites) row.appendChild(makeKey(note, false));
    inner.appendChild(row);
    for (const b of blacks) {
      const key = makeKey(b.note, true);
      key.style.width = `${blackW}%`;
      key.style.left = `${b.before * whiteW - blackW / 2}%`;
      inner.appendChild(key);
    }
    this.keyboard.appendChild(inner);
  }

  logMidi(status: number, d1: number, d2: number, dir: "in" | "out") {
    if (!this.midiMon) return;
    const ch = (status & 0x0f) + 1;
    let s: string;
    switch (status & 0xf0) {
      case 0x90: s = d2 > 0 ? `note on  ${d1}  v${d2}` : `note off ${d1}`; break;
      case 0x80: s = `note off ${d1}`; break;
      case 0xb0: s = `cc ${d1} ${d2}`; break;
      case 0xc0: s = `prog ${d1}`; break;
      case 0xe0: s = `bend ${d1 | (d2 << 7)}`; break;
      default: s = `${(status & 0xf0).toString(16)} ${d1} ${d2}`;
    }
    const row = document.createElement("div");
    row.className = `midi-row ${dir}`;
    row.textContent = `${dir === "in" ? "▸" : "◂"} ch${ch}  ${s}`;
    this.midiMon.appendChild(row);
    while (this.midiMon.childElementCount > 100) this.midiMon.firstElementChild!.remove();
    this.midiMon.scrollTop = this.midiMon.scrollHeight;
  }

  private logMidiTx(bytes: Uint8Array) {
    let i = 0;
    while (i < bytes.length) {
      const status = bytes[i];
      if (status < 0x80) { i++; continue; }
      const len = (status & 0xf0) === 0xc0 || (status & 0xf0) === 0xd0 ? 1 : 2;
      this.logMidi(status, bytes[i + 1] ?? 0, len === 2 ? (bytes[i + 2] ?? 0) : 0, "out");
      i += 1 + len;
    }
  }

  private drawCvScope() {
    const ctx = this.cvScopeCtx;
    if (!ctx) return;
    const w = this.cvScope!.width, h = this.cvScope!.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#8fe9ff14";
    for (let x = 0; x < w; x++) {
      const idx = (this.histPos + Math.floor((x / w) * HIST)) % HIST;
      if (this.gateHist[idx] & 1) ctx.fillRect(x, 0, w / HIST + 1, h);
    }
    const colors = ["#8fe9ff", "#f2b549"];
    for (let ch = 0; ch < 2; ch++) {
      ctx.strokeStyle = colors[ch];
      ctx.lineWidth = 1.25;
      ctx.beginPath();
      for (let x = 0; x < w; x++) {
        const idx = (this.histPos + Math.floor((x / w) * HIST)) % HIST;
        const v = Math.max(0, Math.min(CV_MAX, this.cvHist[ch][idx]));
        const y = h - (v / CV_MAX) * (h - 2) - 1;
        x === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
  }

  // ── Per-frame output ────────────────────────────────────────────────────────

  frame() {
    // OLED phosphor render.
    const px = this.sim.oled();
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

    // CV voltages + gate LEDs.
    for (let ch = 0; ch < this.cvEls.length; ch++) {
      this.cvEls[ch].innerHTML = `${this.sim.cv(ch).toFixed(2)}<i>V</i>`;
    }
    const bits = this.sim.gateBits();
    this.gateEls.forEach((el, g) => el.classList.toggle("on", (bits & (1 << g)) !== 0));

    // Indicator LEDs light their front-panel buttons (Led.on(id)).
    const leds = this.sim.leds();
    for (const [rawId, el] of this.ledEls) el.classList.toggle("lit", leds[rawId] !== 0);

    // MIDI TX → monitor.
    const tx = this.sim.takeMidiTx();
    if (tx.length) this.logMidiTx(tx);

    // CV/gate timeline.
    this.cvHist[0][this.histPos] = this.sim.cv(0);
    this.cvHist[1][this.histPos] = this.sim.cv(1);
    this.gateHist[this.histPos] = bits;
    this.histPos = (this.histPos + 1) % HIST;
    this.drawCvScope();
  }
}
