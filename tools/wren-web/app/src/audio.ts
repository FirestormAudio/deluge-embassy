// Web Audio output — rendered off the main thread in an AudioWorklet.
//
// The DSP engine runs inside the worklet (a second wasm instance); the main VM
// forwards control-rate graph commands here, which the worklet applies + renders
// per audio block. This keeps audio glitch-free under editor/GC load.
//
// Command transport: a lock-free SharedArrayBuffer SPSC ring when the page is
// cross-origin isolated (lowest jitter), else `postMessage` as a fallback. See
// docs/web-editor-plan.md §8.
import { Sim, SAMPLE_RATE } from "./sim";

// Ring layout (shared with public/wren-dsp.js): a fixed-size record ring of
// 16-byte serialized commands. Header = two Int32 (write count, read count);
// records are addressed by monotonic counts modulo RING_RECORDS.
const REC = 16;
const RING_RECORDS = 1024;
const HEADER_I32 = 2;
const SAB_BYTES = HEADER_I32 * 4 + RING_RECORDS * REC;

/// Single-producer writer over the command ring (main thread side).
class RingWriter {
  private ctrl: Int32Array;
  private data: Uint8Array;
  constructor(readonly sab: SharedArrayBuffer) {
    this.ctrl = new Int32Array(sab, 0, HEADER_I32);
    this.data = new Uint8Array(sab, HEADER_I32 * 4);
  }
  write(bytes: Uint8Array) {
    const recs = (bytes.length / REC) | 0;
    const w = Atomics.load(this.ctrl, 0);
    const r = Atomics.load(this.ctrl, 1);
    if (w - r + recs > RING_RECORDS) return; // full: drop (control-rate; shouldn't happen)
    for (let i = 0; i < recs; i++) {
      this.data.set(bytes.subarray(i * REC, i * REC + REC), ((w + i) % RING_RECORDS) * REC);
    }
    Atomics.store(this.ctrl, 0, w + recs); // publish
  }
}

export class Audio {
  private ctx: AudioContext | null = null;
  private node: AudioWorkletNode | null = null;
  private ring: RingWriter | null = null;
  /// Latest peak amplitude reported by the worklet (drives the "live" readout).
  peak = 0;
  running = false;
  /// Whether the SharedArrayBuffer transport is in use (vs postMessage fallback).
  usingSab = false;

  constructor(
    private sim: Sim,
    private wasmUrl: string,
    private workletUrl: string,
  ) {}

  /// Start (or resume) playback. Must be called from a user gesture.
  async start() {
    if (!this.ctx) {
      this.ctx = new AudioContext({ sampleRate: SAMPLE_RATE });
      await this.ctx.audioWorklet.addModule(this.workletUrl);
      const bytes = await (await fetch(this.wasmUrl)).arrayBuffer();
      const module = await WebAssembly.compile(bytes);

      // Use the SAB ring only when the page is cross-origin isolated.
      this.usingSab = self.crossOriginIsolated && typeof SharedArrayBuffer !== "undefined";
      let ringSab: SharedArrayBuffer | undefined;
      if (this.usingSab) {
        ringSab = new SharedArrayBuffer(SAB_BYTES);
        this.ring = new RingWriter(ringSab);
      }

      this.node = new AudioWorkletNode(this.ctx, "wren-dsp", {
        numberOfInputs: 0,
        outputChannelCount: [1],
        processorOptions: { module, ring: ringSab },
      });
      this.node.port.onmessage = (e) => { this.peak = e.data.peak; };
      this.node.connect(this.ctx.destination);
    }
    await this.ctx.resume();
    this.running = true;
  }

  async suspend() {
    this.running = false;
    await this.ctx?.suspend();
  }

  /// Forward the VM's queued audio-graph commands to the worklet engine. Call
  /// once per frame from the main loop.
  forward() {
    if (!this.node || !this.running) return;
    const cmds = this.sim.takeAudioCmds();
    if (!cmds.length) return;
    if (this.ring) this.ring.write(cmds);
    else this.node.port.postMessage(cmds, [cmds.buffer]);
  }
}
