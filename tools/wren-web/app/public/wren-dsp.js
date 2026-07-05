// AudioWorklet processor: renders the Deluge DSP graph off the main thread.
//
// It owns a second instance of the wren-web wasm (the compiled module is handed
// in via processorOptions) and uses only its engine API: forwarded graph
// commands arrive on the port, are applied, then each 128-sample block is
// rendered in process(). No VM here — that runs on the main thread, which posts
// the (low-rate) command stream. Mirrors the firmware's vm_task → audio_task
// split, with the port standing in for the command ring.

const WASI_STUB = {
  fd_write: () => 0,
  fd_close: () => 0,
  fd_seek: () => 0,
  fd_fdstat_get: () => 0,
  clock_time_get: () => 0,
};

// Command ring layout (shared with src/audio.ts): 16-byte records, an 8-byte
// header of two Int32 (write count, read count).
const REC = 16;
const RING_RECORDS = 1024;

class WrenDsp extends AudioWorkletProcessor {
  constructor(options) {
    super();
    const { module, ring } = options.processorOptions;
    this.inst = new WebAssembly.Instance(module, { wasi_snapshot_preview1: WASI_STUB });
    this.x = this.inst.exports;
    if (this.x._initialize) this.x._initialize(); // apply data segments if it's a reactor
    this.mem = this.x.memory;
    this.frames = 0;
    // SharedArrayBuffer command ring (preferred), or a postMessage queue.
    if (ring) {
      this.ctrl = new Int32Array(ring, 0, 2);
      this.ringData = new Uint8Array(ring, 8);
    } else {
      this.pending = [];
      this.port.onmessage = (e) => this.pending.push(new Uint8Array(e.data));
    }
  }

  applyCmd(bytes, len) {
    const n = Math.min(len, this.x.sim_engine_cmd_cap());
    new Uint8Array(this.mem.buffer, this.x.sim_engine_cmd_ptr(), n).set(bytes.subarray(0, n));
    this.x.sim_engine_apply(n);
  }

  process(_inputs, outputs) {
    // Apply any forwarded graph commands before rendering this block.
    if (this.ctrl) {
      // Drain the SAB ring (single consumer).
      let r = Atomics.load(this.ctrl, 1);
      const w = Atomics.load(this.ctrl, 0);
      while (r < w) {
        const slot = (r % RING_RECORDS) * REC;
        this.applyCmd(this.ringData.subarray(slot, slot + REC), REC);
        r++;
      }
      Atomics.store(this.ctrl, 1, r);
    } else {
      while (this.pending.length) {
        const bytes = this.pending.shift();
        this.applyCmd(bytes, bytes.length);
      }
    }

    const out = outputs[0];
    if (!out || !out[0]) return true;
    const block = out[0].length;
    this.x.sim_engine_render(block);
    const audio = new Float32Array(this.mem.buffer, this.x.sim_audio_ptr(), block);
    for (let c = 0; c < out.length; c++) out[c].set(audio);

    // Heartbeat: report peak amplitude back to the main thread (drives the
    // "live" indicator and verification) ~5x/sec.
    if (++this.frames % 32 === 0) {
      let peak = 0;
      for (let i = 0; i < block; i++) { const a = Math.abs(audio[i]); if (a > peak) peak = a; }
      this.port.postMessage({ peak });
    }
    return true;
  }
}

registerProcessor("wren-dsp", WrenDsp);
