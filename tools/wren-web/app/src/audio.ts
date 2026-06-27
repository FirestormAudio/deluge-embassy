// Web Audio output for the simulator.
//
// First cut: the DSP graph renders on the main thread (one wasm instance, shared
// with the VM) and blocks are scheduled into an AudioContext via lookahead
// scheduling. Simple and infra-free; the planned upgrade is to move the engine
// into an AudioWorklet with SharedArrayBuffer rings (glitch-free, off the main
// thread) — see docs/web-editor-plan.md §8.
import { Sim, SAMPLE_RATE } from "./sim";

const BLOCK = 2048; // samples per scheduled buffer (~46 ms at 44.1 kHz)
const LOOKAHEAD = 0.18; // seconds of audio to keep queued ahead of the playhead

export class Audio {
  private ctx: AudioContext | null = null;
  private gain!: GainNode;
  private nextTime = 0;
  private timer = 0;
  private scratch = new Float32Array(BLOCK);
  /// The most recently rendered block, for the scope.
  readonly latest = new Float32Array(BLOCK);
  running = false;

  constructor(private sim: Sim) {}

  /// Start (or resume) playback. Must be called from a user gesture.
  async start() {
    if (!this.ctx) {
      this.ctx = new AudioContext({ sampleRate: SAMPLE_RATE });
      this.gain = this.ctx.createGain();
      this.gain.gain.value = 0.6;
      this.gain.connect(this.ctx.destination);
      this.nextTime = this.ctx.currentTime;
    }
    await this.ctx.resume();
    this.running = true;
    if (!this.timer) this.timer = window.setInterval(() => this.pump(), 25);
  }

  async suspend() {
    this.running = false;
    if (this.timer) { clearInterval(this.timer); this.timer = 0; }
    await this.ctx?.suspend();
  }

  private pump() {
    const ctx = this.ctx;
    if (!ctx || !this.running) return;
    // Don't let a long stall schedule a flood of catch-up buffers.
    if (this.nextTime < ctx.currentTime) this.nextTime = ctx.currentTime;
    while (this.nextTime < ctx.currentTime + LOOKAHEAD) {
      const n = this.sim.render(this.scratch, BLOCK);
      this.latest.set(this.scratch.subarray(0, n));
      const buf = ctx.createBuffer(1, n, SAMPLE_RATE);
      buf.getChannelData(0).set(this.scratch.subarray(0, n));
      const src = ctx.createBufferSource();
      src.buffer = buf;
      src.connect(this.gain);
      src.start(this.nextTime);
      this.nextTime += n / SAMPLE_RATE;
    }
  }
}
