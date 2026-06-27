// The instrument faceplate: OLED, 18x8 pad grid, CV/gate readouts, mini MIDI
// keyboard. Input widgets call into the Sim; output widgets read from it each
// frame. Pads and the keyboard are input surfaces (the binding set exposes pad
// presses and MIDI in); they light locally on press for feedback.
import { Sim, OLED_W, OLED_H, PAD_COLS, PAD_ROWS } from "./sim";

const OLED_SCALE = 3;
// 18x8 pad hues across the spectrum — the grid is the panel's signature.
const padHue = (x: number, y: number) => (x / PAD_COLS) * 320 + y * 6;

export class Panel {
  private oledCtx: CanvasRenderingContext2D;
  private padEls: HTMLButtonElement[] = [];
  private cvEls: HTMLElement[] = [];
  private gateEls: HTMLElement[] = [];
  private buttonEls: HTMLButtonElement[] = [];
  private encoders = document.querySelector<HTMLElement>("#encoders");
  private buttons = document.querySelector<HTMLElement>("#buttons");

  constructor(
    oled: HTMLCanvasElement,
    private padGrid: HTMLElement,
    private cvRow: HTMLElement,
    private keyboard: HTMLElement,
    private sim: Sim,
  ) {
    oled.width = OLED_W * OLED_SCALE;
    oled.height = OLED_H * OLED_SCALE;
    this.oledCtx = oled.getContext("2d")!;
    this.buildPads();
    this.buildEncoders();
    this.buildButtons();
    this.buildCv();
    this.buildKeyboard();
  }

  private buildEncoders() {
    if (!this.encoders) return;
    for (let i = 0; i < 6; i++) {
      const enc = document.createElement("div");
      enc.className = "enc";
      enc.innerHTML = `<div class="enc-knob"><span class="enc-tick"></span></div><span class="enc-legend">${i + 1}</span>`;
      const knob = enc.querySelector<HTMLElement>(".enc-knob")!;
      let angle = 0;
      const turn = (delta: number) => {
        angle += delta * 20;
        knob.style.transform = `rotate(${angle}deg)`;
        this.sim.enc(i, delta);
      };
      // Wheel = detents; vertical drag = detents (8 px each).
      enc.addEventListener("wheel", (e) => { e.preventDefault(); turn(e.deltaY < 0 ? 1 : -1); }, { passive: false });
      let dragging = false, lastY = 0, accum = 0;
      enc.addEventListener("pointerdown", (e) => { dragging = true; lastY = e.clientY; enc.setPointerCapture(e.pointerId); });
      enc.addEventListener("pointermove", (e) => {
        if (!dragging) return;
        accum += lastY - e.clientY;
        lastY = e.clientY;
        while (accum >= 8) { accum -= 8; turn(1); }
        while (accum <= -8) { accum += 8; turn(-1); }
      });
      enc.addEventListener("pointerup", () => { dragging = false; });
      this.encoders.appendChild(enc);
    }
  }

  private buildButtons() {
    if (!this.buttons) return;
    // Generic front-panel buttons by id (0..15); they light from Led.on(id).
    for (let id = 0; id < 16; id++) {
      const b = document.createElement("button");
      b.className = "fbtn";
      b.textContent = String(id);
      const press = (down: boolean) => { b.classList.toggle("pressed", down); this.sim.button(id, down); };
      b.addEventListener("pointerdown", (e) => { e.preventDefault(); press(true); });
      b.addEventListener("pointerup", () => press(false));
      b.addEventListener("pointerleave", () => b.classList.contains("pressed") && press(false));
      this.buttons.appendChild(b);
      this.buttonEls.push(b);
    }
  }

  private buildPads() {
    for (let y = 0; y < PAD_ROWS; y++) {
      for (let x = 0; x < PAD_COLS; x++) {
        const pad = document.createElement("button");
        pad.className = "pad";
        pad.style.setProperty("--hue", String(padHue(x, y)));
        const press = (down: boolean) => {
          pad.classList.toggle("lit", down);
          this.sim.pad(x, y, down);
        };
        pad.addEventListener("pointerdown", (e) => { e.preventDefault(); press(true); });
        pad.addEventListener("pointerup", () => press(false));
        pad.addEventListener("pointerleave", () => pad.classList.contains("lit") && press(false));
        this.padGrid.appendChild(pad);
        this.padEls.push(pad);
      }
    }
  }

  private buildCv() {
    for (let ch = 0; ch < 2; ch++) {
      const jack = document.createElement("div");
      jack.className = "jack";
      jack.innerHTML = `<span class="jack-led"></span><span class="jack-legend">CV${ch + 1}</span><span class="jack-val">0.00<i>V</i></span>`;
      this.cvRow.appendChild(jack);
      this.cvEls.push(jack.querySelector(".jack-val")!);
    }
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
  }

  private buildKeyboard() {
    // One octave from C3 (MIDI 48), enough to drive the synth examples.
    const base = 48;
    const isBlack = (n: number) => [1, 3, 6, 8, 10].includes(n % 12);
    for (let i = 0; i <= 16; i++) {
      const note = base + i;
      const key = document.createElement("button");
      key.className = "key" + (isBlack(note) ? " black" : "");
      const down = (on: boolean) => {
        key.classList.toggle("down", on);
        this.sim.midiIn(on ? 0x90 : 0x80, note, on ? 100 : 0);
      };
      key.addEventListener("pointerdown", (e) => { e.preventDefault(); down(true); });
      key.addEventListener("pointerup", () => down(false));
      key.addEventListener("pointerleave", () => key.classList.contains("down") && down(false));
      this.keyboard.appendChild(key);
    }
  }

  /// Redraw output-driven widgets from current sim state.
  frame() {
    // OLED: phosphor pixels with a soft additive bloom.
    const px = this.sim.oled();
    const ctx = this.oledCtx;
    ctx.fillStyle = "#0a0e10";
    ctx.fillRect(0, 0, OLED_W * OLED_SCALE, OLED_H * OLED_SCALE);
    ctx.fillStyle = "#8fe9ff";
    ctx.shadowColor = "#8fe9ff";
    ctx.shadowBlur = OLED_SCALE * 1.5;
    for (let y = 0; y < OLED_H; y++) {
      for (let x = 0; x < OLED_W; x++) {
        if (px[y * OLED_W + x]) {
          ctx.fillRect(x * OLED_SCALE, y * OLED_SCALE, OLED_SCALE - 0.5, OLED_SCALE - 0.5);
        }
      }
    }
    ctx.shadowBlur = 0;

    // CV voltages + gate LEDs.
    for (let ch = 0; ch < this.cvEls.length; ch++) {
      this.cvEls[ch].innerHTML = `${this.sim.cv(ch).toFixed(2)}<i>V</i>`;
    }
    const bits = this.sim.gateBits();
    this.gateEls.forEach((el, g) => el.classList.toggle("on", (bits & (1 << g)) !== 0));

    // Indicator LEDs (Led.on(id) / off(id)) light the front-panel buttons.
    const leds = this.sim.leds();
    this.buttonEls.forEach((b, id) => b.classList.toggle("lit", leds[id] !== 0));
  }
}
