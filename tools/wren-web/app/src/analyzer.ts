// Main-thread handle to the analyzer worker: ships source on edit (debounced),
// delivers diagnostics back. Pairs with src/analyzer-worker.ts.
import type { Diag } from "./analyzer-worker";

export type { Diag };

export class Analyzer {
  private worker: Worker;
  private latest = 0;
  private debounce = 0;
  onDiagnostics: (version: number, diags: Diag[]) => void = () => {};

  constructor(wasmUrl: string) {
    this.worker = new Worker(new URL("./analyzer-worker.ts", import.meta.url), { type: "module" });
    this.worker.onmessage = (e) => {
      if (e.data.type === "diagnostics") this.onDiagnostics(e.data.version, e.data.diags as Diag[]);
    };
    // Hand the worker its wasm bytes once.
    fetch(wasmUrl)
      .then((r) => r.arrayBuffer())
      .then((buf) => this.worker.postMessage({ type: "init", wasm: buf }, [buf]));
  }

  /// Schedule analysis of `source`, debounced. `version` lets the caller drop
  /// stale results.
  analyze(source: string, version: number, delayMs = 250) {
    this.latest = version;
    clearTimeout(this.debounce);
    this.debounce = window.setTimeout(() => {
      this.worker.postMessage({ type: "analyze", source, version });
    }, delayMs);
  }

  get currentVersion() {
    return this.latest;
  }
}
