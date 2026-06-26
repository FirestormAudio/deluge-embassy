// Loader + ergonomic wrapper for the wren-web wasm core.
//
// The module carries the VM's WASI imports (it links wasi-libc), so a WASI shim
// must be supplied. Pass one in (`wasiImport`): under Node use `node:wasi`; in the
// browser use a small shim such as @bjorn3/browser_wasi_shim. Everything else is
// the simulator's own C-ABI surface, wrapped here as a friendly `Sim` object.

const OLED_W = 128;
const OLED_H = 48;

/// Instantiate the wasm and return a booted `Sim`.
/// `wasmBytes`: the .wasm as a Uint8Array/ArrayBuffer.
/// `wasiImport`: the object to bind as `wasi_snapshot_preview1`.
export async function loadSim(wasmBytes, wasiImport, initWasi = () => {}) {
  const mod = await WebAssembly.compile(wasmBytes);
  const inst = await WebAssembly.instantiate(mod, { wasi_snapshot_preview1: wasiImport });
  initWasi(inst); // e.g. wasi.initialize(inst) for a reactor module
  const sim = new Sim(inst);
  if (sim.x.sim_boot() !== 1) throw new Error("wren-web: VM failed to boot");
  return sim;
}

export class Sim {
  constructor(inst) {
    this.x = inst.exports;
    this.mem = inst.exports.memory;
  }

  #bytes(ptr, len) {
    return new Uint8Array(this.mem.buffer, ptr, len);
  }
  #str(ptr, len) {
    return new TextDecoder().decode(this.#bytes(ptr, len));
  }

  /// Run a wren script in the persistent `main` module. Returns
  /// { ok, code, output, error, errorLine }. code: 0 ok, 1 compile, 2 runtime.
  load(source) {
    const enc = new TextEncoder().encode(source);
    const cap = this.x.sim_src_cap();
    if (enc.length > cap) throw new Error(`script too large (${enc.length} > ${cap})`);
    this.#bytes(this.x.sim_src_ptr(), enc.length).set(enc);
    const code = this.x.sim_load(enc.length);
    return {
      ok: code === 0,
      code,
      output: this.output(),
      error: this.error(),
      errorLine: this.x.sim_err_line(),
    };
  }

  // ── Input injection ──
  pad(x, y, down) { this.x.sim_pad(x, y, down ? 1 : 0); }
  button(id, down) { this.x.sim_button(id, down ? 1 : 0); }
  enc(index, delta) { this.x.sim_enc(index, delta); }
  midiIn(status, d1, d2) { this.x.sim_midi_in(status, d1, d2); }

  // ── Time / control-rate tick ──
  setNowMs(ms) { this.x.sim_set_now_ms(ms); }
  tick(nowMs, dtS) { this.x.sim_set_now_ms(nowMs); this.x.sim_tick(dtS); }

  // ── Output drains ──
  /// The OLED as a {width,height,px} where px is one byte/pixel (0 or 255).
  oled() {
    return { width: OLED_W, height: OLED_H, px: this.#bytes(this.x.sim_oled_ptr(), this.x.sim_oled_len()) };
  }
  cv(ch) { return this.x.sim_cv(ch); }
  /// Gate state as a bitmask (bit i = gate i+1).
  gateBits() { return this.x.sim_gate_bits(); }
  leds() { return this.#bytes(this.x.sim_led_ptr(), this.x.sim_led_len()); }

  output() { return this.#str(this.x.sim_out_ptr(), this.x.sim_out_len()); }
  clearOutput() { this.x.sim_out_clear(); }
  error() { return this.#str(this.x.sim_err_ptr(), this.x.sim_err_len()); }

  /// Drain queued MIDI TX bytes and clear the buffer.
  takeMidiTx() {
    const out = this.#bytes(this.x.sim_midi_tx_ptr(), this.x.sim_midi_tx_len()).slice();
    this.x.sim_midi_tx_clear();
    return out;
  }
}
