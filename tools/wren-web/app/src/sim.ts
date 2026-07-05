// Browser-side loader + wrapper for the wren-web wasm core.
//
// The module links wasi-libc, so it needs a WASI shim; in the browser we use
// @bjorn3/browser_wasi_shim. Everything else is the simulator's own C-ABI
// surface, wrapped here as an ergonomic `Sim`.
import { WASI, OpenFile, File, ConsoleStdout } from "@bjorn3/browser_wasi_shim";

export const OLED_W = 128;
export const OLED_H = 48;
export const PAD_COLS = 18;
export const PAD_ROWS = 8;

interface SimExports {
  memory: WebAssembly.Memory;
  sim_boot(): number;
  sim_reset(): number;
  sim_src_ptr(): number;
  sim_src_cap(): number;
  sim_load(len: number): number;
  sim_set_now_ms(ms: number): void;
  sim_tick(dt: number): void;
  sim_pad(x: number, y: number, down: number): void;
  sim_button(id: number, down: number): void;
  sim_enc(index: number, delta: number): void;
  sim_midi_in(status: number, d1: number, d2: number): void;
  sim_oled_ptr(): number;
  sim_oled_len(): number;
  sim_cv(ch: number): number;
  sim_gate_bits(): number;
  sim_led_ptr(): number;
  sim_led_len(): number;
  sim_out_ptr(): number;
  sim_out_len(): number;
  sim_out_clear(): void;
  sim_err_ptr(): number;
  sim_err_len(): number;
  sim_err_line(): number;
  sim_err_module_ptr(): number;
  sim_err_module_len(): number;
  sim_midi_tx_ptr(): number;
  sim_midi_tx_len(): number;
  sim_midi_tx_clear(): void;
  sim_audio_ptr(): number;
  sim_audio_cap(): number;
  sim_render(n: number): number;
  sim_audio_cmds_ptr(): number;
  sim_audio_cmds_len(): number;
  sim_audio_cmds_clear(): void;
  sim_clear_modules(): void;
  sim_mod_name_ptr(): number;
  sim_add_module(nameLen: number, srcLen: number): number;
}

export const SAMPLE_RATE = 44100;

/// The Deluge prelude's public names, auto-imported into every non-entry module
/// (see runProject). Mirrors crates/deluge-wren-core/wren/prelude.wren.
const PRELUDE_IMPORT =
  'import "main" for Output, Gate, Metro, Midi, Pads, Buttons, Enc, Led, Oled, Node, Osc, Env, Noise, Out, output, gate\n';

export interface LoadResult {
  ok: boolean;
  code: number; // 0 ok, 1 compile error, 2 runtime error
  output: string;
  error: string;
  errorLine: number;
  /// Module the error is in ("main" for the entry, else an imported module name).
  errorModule: string;
}

export async function loadSim(wasmUrl: string): Promise<Sim> {
  const fds = [
    new OpenFile(new File([])),
    ConsoleStdout.lineBuffered((m) => console.log("[wren]", m)),
    ConsoleStdout.lineBuffered((m) => console.warn("[wren]", m)),
  ];
  const wasi = new WASI([], [], fds);
  const { instance } = await WebAssembly.instantiateStreaming(fetch(wasmUrl), {
    wasi_snapshot_preview1: wasi.wasiImport,
  });
  wasi.initialize(instance as { exports: { memory: WebAssembly.Memory; _initialize?: () => void } });
  const sim = new Sim(instance.exports as unknown as SimExports);
  if (sim.x.sim_boot() !== 1) throw new Error("wren-web: VM failed to boot");
  return sim;
}

export class Sim {
  readonly x: SimExports;
  constructor(exports: SimExports) {
    this.x = exports;
  }

  private bytes(ptr: number, len: number): Uint8Array {
    return new Uint8Array(this.x.memory.buffer, ptr, len);
  }
  private str(ptr: number, len: number): string {
    return new TextDecoder().decode(this.bytes(ptr, len));
  }

  /// Rebuild the VM + clear all state, then run `source` fresh (no leftover
  /// module vars, metros, patches). This is "Run" semantics.
  run(source: string): LoadResult {
    this.x.sim_reset();
    return this.load(source);
  }

  /// Run a multi-file project: reset, register every non-entry file as an
  /// importable module (name = path without `.wren`), then run the entry.
  /// `import "lib/x"` resolves against the registry at compile time.
  ///
  /// The Deluge prelude (Osc/Env/Output/…) lives in the `main` module (the entry
  /// runs there), and Wren modules are isolated — so each imported module gets an
  /// auto-prepended `import "main" for <prelude names>` to make the API available
  /// (this shifts that module's runtime error lines by 1).
  runProject(files: Record<string, string>, entry: string): LoadResult {
    this.x.sim_reset();
    this.x.sim_clear_modules();
    for (const [path, content] of Object.entries(files)) {
      if (path === entry) continue; // the entry runs raw in `main`, with the prelude
      this.addModule(path.replace(/\.wren$/, ""), PRELUDE_IMPORT + content);
    }
    return this.load(files[entry] ?? "");
  }

  private addModule(name: string, content: string) {
    const nb = new TextEncoder().encode(name);
    new Uint8Array(this.x.memory.buffer, this.x.sim_mod_name_ptr(), nb.length).set(nb);
    const sb = new TextEncoder().encode(content);
    if (sb.length > this.x.sim_src_cap()) return;
    new Uint8Array(this.x.memory.buffer, this.x.sim_src_ptr(), sb.length).set(sb);
    this.x.sim_add_module(nb.length, sb.length);
  }

  load(source: string): LoadResult {
    const enc = new TextEncoder().encode(source);
    if (enc.length > this.x.sim_src_cap()) throw new Error("script too large");
    this.bytes(this.x.sim_src_ptr(), enc.length).set(enc);
    const code = this.x.sim_load(enc.length);
    return {
      ok: code === 0,
      code,
      output: this.output(),
      error: this.error(),
      errorLine: this.x.sim_err_line(),
      errorModule: this.str(this.x.sim_err_module_ptr(), this.x.sim_err_module_len()),
    };
  }

  pad(x: number, y: number, down: boolean) { this.x.sim_pad(x, y, down ? 1 : 0); }
  button(id: number, down: boolean) { this.x.sim_button(id, down ? 1 : 0); }
  enc(index: number, delta: number) { this.x.sim_enc(index, delta); }
  midiIn(status: number, d1: number, d2: number) { this.x.sim_midi_in(status, d1, d2); }

  tick(nowMs: number, dtS: number) { this.x.sim_set_now_ms(nowMs); this.x.sim_tick(dtS); }

  oled(): Uint8Array { return this.bytes(this.x.sim_oled_ptr(), this.x.sim_oled_len()); }
  cv(ch: number): number { return this.x.sim_cv(ch); }
  gateBits(): number { return this.x.sim_gate_bits(); }
  leds(): Uint8Array { return this.bytes(this.x.sim_led_ptr(), this.x.sim_led_len()); }

  output(): string { return this.str(this.x.sim_out_ptr(), this.x.sim_out_len()); }
  clearOutput() { this.x.sim_out_clear(); }
  error(): string { return this.str(this.x.sim_err_ptr(), this.x.sim_err_len()); }

  takeMidiTx(): Uint8Array {
    const out = this.bytes(this.x.sim_midi_tx_ptr(), this.x.sim_midi_tx_len()).slice();
    this.x.sim_midi_tx_clear();
    return out;
  }

  /// Drain the queued audio-graph commands (serialized) for forwarding to the
  /// AudioWorklet engine, and clear the queue.
  takeAudioCmds(): Uint8Array {
    const len = this.x.sim_audio_cmds_len();
    if (len === 0) return new Uint8Array(0);
    const out = this.bytes(this.x.sim_audio_cmds_ptr(), len).slice();
    this.x.sim_audio_cmds_clear();
    return out;
  }

  /// Render `n` mono samples from the *main* engine into `dst` (for the scope).
  render(dst: Float32Array, n: number): number {
    const got = this.x.sim_render(n);
    const view = new Float32Array(this.x.memory.buffer, this.x.sim_audio_ptr(), got);
    dst.set(view.subarray(0, got));
    return got;
  }
}
