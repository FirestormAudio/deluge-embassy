// Main-thread handle to the analyzer worker: fire-and-forget diagnostics on edit
// (debounced) plus request/response position queries (hover, definition,
// completions) that the Monaco providers await. Pairs with analyzer-worker.ts.
import type { Diag, DefLoc, Completion } from "./analyzer-worker";

export type { Diag, DefLoc, Completion };

type QueryKind = "hover" | "definition" | "completions";

export class Analyzer {
  private worker: Worker;
  private debounce = 0;
  private nextId = 1;
  private pending = new Map<number, (result: unknown) => void>();
  onDiagnostics: (version: number, diags: Diag[]) => void = () => {};

  constructor(wasmUrl: string) {
    this.worker = new Worker(new URL("./analyzer-worker.ts", import.meta.url), { type: "module" });
    this.worker.onmessage = (e) => {
      const m = e.data;
      if (m.type === "diagnostics") this.onDiagnostics(m.version, m.diags as Diag[]);
      else if (m.type === "queryResult") {
        const resolve = this.pending.get(m.id);
        if (resolve) {
          this.pending.delete(m.id);
          resolve(m.result);
        }
      }
    };
    fetch(wasmUrl)
      .then((r) => r.arrayBuffer())
      .then((buf) => this.worker.postMessage({ type: "init", wasm: buf }, [buf]));
  }

  /// Schedule analysis of `source` (debounced); `version` lets the caller drop
  /// stale diagnostics.
  analyze(source: string, version: number, delayMs = 250) {
    clearTimeout(this.debounce);
    this.debounce = window.setTimeout(() => {
      this.worker.postMessage({ type: "analyze", source, version });
    }, delayMs);
  }

  private query<T>(kind: QueryKind, source: string, offset: number): Promise<T> {
    const id = this.nextId++;
    return new Promise<T>((resolve) => {
      this.pending.set(id, resolve as (r: unknown) => void);
      this.worker.postMessage({ type: "query", id, kind, source, offset });
    });
  }

  hover(source: string, offset: number): Promise<string> {
    return this.query<string>("hover", source, offset);
  }
  definition(source: string, offset: number): Promise<DefLoc | null> {
    return this.query<DefLoc | null>("definition", source, offset);
  }
  completions(source: string): Promise<Completion[]> {
    return this.query<Completion[]>("completions", source, 0);
  }
}
