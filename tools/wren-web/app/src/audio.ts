// Web Audio output — rendered off the main thread in an AudioWorklet.
//
// The DSP engine runs inside the worklet (a second wasm instance); the main VM
// forwards control-rate graph commands here, which the worklet applies + renders
// per audio block. This keeps audio glitch-free under editor/GC load, unlike
// rendering on the main thread. See docs/web-editor-plan.md §8.
import { Sim, SAMPLE_RATE } from "./sim";

export class Audio {
  private ctx: AudioContext | null = null;
  private node: AudioWorkletNode | null = null;
  /// Latest peak amplitude reported by the worklet (drives the "live" readout).
  peak = 0;
  running = false;

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
      // The worklet needs its own compiled copy of the wasm module.
      const bytes = await (await fetch(this.wasmUrl)).arrayBuffer();
      const module = await WebAssembly.compile(bytes);
      this.node = new AudioWorkletNode(this.ctx, "wren-dsp", {
        numberOfInputs: 0,
        outputChannelCount: [1],
        processorOptions: { module },
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

  /// Forward the VM's queued audio-graph commands to the worklet. Call once per
  /// frame from the main loop.
  forward() {
    if (!this.node || !this.running) return;
    const cmds = this.sim.takeAudioCmds();
    if (cmds.length) this.node.port.postMessage(cmds, [cmds.buffer]);
  }
}
