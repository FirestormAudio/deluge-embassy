// Web Worker: runs the wren-analyzer wasm off the editor thread, returning live
// diagnostics. The module is pure-compute (no imports), instantiated from bytes
// the main thread sends once. See tools/wren-analyzer-wasm for the wire format.

interface AnalyzerExports {
  memory: WebAssembly.Memory;
  src_reserve(len: number): number;
  analyze_run(): number;
  result_ptr(): number;
}

export interface Diag {
  startLine: number;
  startCol: number;
  endLine: number;
  endCol: number;
  severity: number; // 0=error 1=warning 2=info 3=hint
  message: string;
}

let x: AnalyzerExports | null = null;

function analyze(source: string): Diag[] {
  if (!x) return [];
  const enc = new TextEncoder().encode(source);
  const ptr = x.src_reserve(enc.length);
  new Uint8Array(x.memory.buffer, ptr, enc.length).set(enc);
  const len = x.analyze_run();
  const buf = new Uint8Array(x.memory.buffer, x.result_ptr(), len).slice();
  const dv = new DataView(buf.buffer);
  let o = 0;
  const count = dv.getUint32(o, true);
  o += 4;
  const out: Diag[] = [];
  for (let i = 0; i < count; i++) {
    const startLine = dv.getUint32(o, true);
    const startCol = dv.getUint32(o + 4, true);
    const endLine = dv.getUint32(o + 8, true);
    const endCol = dv.getUint32(o + 12, true);
    const severity = buf[o + 16];
    const mlen = dv.getUint32(o + 17, true);
    o += 21;
    const message = new TextDecoder().decode(buf.subarray(o, o + mlen));
    o += mlen;
    out.push({ startLine, startCol, endLine, endCol, severity, message });
  }
  return out;
}

self.onmessage = async (e: MessageEvent) => {
  const msg = e.data;
  if (msg.type === "init") {
    const { instance } = await WebAssembly.instantiate(msg.wasm as ArrayBuffer, {});
    x = instance.exports as unknown as AnalyzerExports;
    (self as unknown as Worker).postMessage({ type: "ready" });
  } else if (msg.type === "analyze") {
    (self as unknown as Worker).postMessage({ type: "diagnostics", version: msg.version, diags: analyze(msg.source) });
  }
};
