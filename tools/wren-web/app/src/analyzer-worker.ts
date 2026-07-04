// Web Worker: runs the wren-analyzer wasm off the editor thread. Handles
// fire-and-forget diagnostics plus request/response position queries (hover,
// definition, completions). The module is pure-compute (no imports). Wire format
// is defined in tools/wren-analyzer-wasm.

interface AnalyzerExports {
  memory: WebAssembly.Memory;
  src_reserve(len: number): number;
  analyze_run(): number;
  result_ptr(): number;
  hover_at(byteOffset: number): number;
  definition_at(byteOffset: number): number;
  completions(): number;
  // Project modules for cross-file import validation.
  mod_name_reserve(len: number): number;
  clear_modules(): void;
  add_module(nameLen: number): void;
}

export interface Diag {
  startLine: number;
  startCol: number;
  endLine: number;
  endCol: number;
  severity: number; // 0=error 1=warning 2=info 3=hint
  message: string;
}
export interface DefLoc {
  inPrelude: boolean;
  startLine: number;
  startCol: number;
  endLine: number;
  endCol: number;
}
export interface Completion {
  label: string;
  kind: number; // 0=class 1=method 2=field 3=variable 4=module
  detail: string;
}

let x: AnalyzerExports | null = null;
const td = new TextDecoder();

function setSource(source: string): number {
  const enc = new TextEncoder().encode(source);
  const ptr = x!.src_reserve(enc.length);
  new Uint8Array(x!.memory.buffer, ptr, enc.length).set(enc);
  return enc.length;
}

/// Register the other project files as modules so `analyze_run` can validate
/// cross-file imports. Each module's source is staged through the SRC buffer
/// (add_module copies it), so this must run BEFORE the active source is set.
function registerModules(modules: Record<string, string> | undefined): void {
  x!.clear_modules();
  if (!modules) return;
  const enc = new TextEncoder();
  for (const [name, src] of Object.entries(modules)) {
    const nb = enc.encode(name);
    // Reserve BEFORE reading memory.buffer — the reserve may grow wasm memory,
    // detaching the previous buffer.
    const ptr = x!.mod_name_reserve(nb.length);
    new Uint8Array(x!.memory.buffer, ptr, nb.length).set(nb);
    setSource(src); // stage the module source in SRC
    x!.add_module(nb.length);
  }
}
function out(len: number): { buf: Uint8Array; dv: DataView } {
  const buf = new Uint8Array(x!.memory.buffer, x!.result_ptr(), len).slice();
  return { buf, dv: new DataView(buf.buffer) };
}
/// Monaco gives UTF-16 offsets; the analyzer wants UTF-8 byte offsets.
function byteOffset(source: string, utf16: number): number {
  return new TextEncoder().encode(source.slice(0, utf16)).length;
}

function diagnostics(source: string, modules?: Record<string, string>): Diag[] {
  registerModules(modules); // stages module sources through SRC — must precede setSource(source)
  setSource(source);
  const len = x!.analyze_run();
  const { buf, dv } = out(len);
  let o = 0;
  const count = dv.getUint32(o, true);
  o += 4;
  const res: Diag[] = [];
  for (let i = 0; i < count; i++) {
    const startLine = dv.getUint32(o, true);
    const startCol = dv.getUint32(o + 4, true);
    const endLine = dv.getUint32(o + 8, true);
    const endCol = dv.getUint32(o + 12, true);
    const severity = buf[o + 16];
    const mlen = dv.getUint32(o + 17, true);
    o += 21;
    res.push({ startLine, startCol, endLine, endCol, severity, message: td.decode(buf.subarray(o, o + mlen)) });
    o += mlen;
  }
  return res;
}

function hover(source: string, utf16: number): string {
  setSource(source);
  const len = x!.hover_at(byteOffset(source, utf16));
  return len ? td.decode(out(len).buf) : "";
}

function definition(source: string, utf16: number): DefLoc | null {
  setSource(source);
  const len = x!.definition_at(byteOffset(source, utf16));
  if (len < 18) return null;
  const { buf, dv } = out(len);
  if (buf[0] !== 1) return null;
  return {
    inPrelude: buf[1] === 1,
    startLine: dv.getUint32(2, true),
    startCol: dv.getUint32(6, true),
    endLine: dv.getUint32(10, true),
    endCol: dv.getUint32(14, true),
  };
}

function completions(source: string): Completion[] {
  setSource(source);
  const len = x!.completions();
  const { buf, dv } = out(len);
  let o = 0;
  const count = dv.getUint32(o, true);
  o += 4;
  const res: Completion[] = [];
  for (let i = 0; i < count; i++) {
    const kind = buf[o];
    o += 1;
    const ll = dv.getUint32(o, true);
    o += 4;
    const label = td.decode(buf.subarray(o, o + ll));
    o += ll;
    const dl = dv.getUint32(o, true);
    o += 4;
    const detail = td.decode(buf.subarray(o, o + dl));
    o += dl;
    res.push({ label, kind, detail });
  }
  return res;
}

const post = (m: unknown) => (self as unknown as Worker).postMessage(m);

// Handle an analyze/query once the wasm is ready.
function handle(msg: { type: string; version?: number; source: string; id?: number; kind?: QueryKind; offset?: number; modules?: Record<string, string> }) {
  if (msg.type === "analyze") {
    post({ type: "diagnostics", version: msg.version, diags: diagnostics(msg.source, msg.modules) });
  } else if (msg.type === "query") {
    let result: unknown = null;
    if (msg.kind === "hover") result = hover(msg.source, msg.offset!);
    else if (msg.kind === "definition") result = definition(msg.source, msg.offset!);
    else if (msg.kind === "completions") result = completions(msg.source);
    // Always reply — even a null result — so the main thread's pending promise
    // resolves instead of hanging forever (which wedges Monaco's hover).
    post({ type: "queryResult", id: msg.id, result });
  }
}

type QueryKind = "hover" | "definition" | "completions";

// Messages that arrive before the wasm finishes initializing are queued and
// drained on ready — dropping them would hang the caller's promise.
const queued: Parameters<typeof handle>[0][] = [];

self.onmessage = async (e: MessageEvent) => {
  const msg = e.data;
  if (msg.type === "init") {
    try {
      const { instance } = await WebAssembly.instantiate(msg.wasm as ArrayBuffer, {});
      x = instance.exports as unknown as AnalyzerExports;
    } catch (err) {
      post({ type: "initError", error: String(err && (err as Error).stack ? (err as Error).stack : err) });
      return;
    }
    post({ type: "ready" });
    for (const m of queued.splice(0)) handle(m);
    return;
  }
  if (!x) {
    queued.push(msg);
    return;
  }
  handle(msg);
};
