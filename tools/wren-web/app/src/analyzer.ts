// Main-thread handle to the analyzer worker: fire-and-forget diagnostics on edit
// (debounced) plus request/response position queries (hover, definition,
// completions) that the Monaco providers await. Pairs with analyzer-worker.ts.
import type { Diag, DefLoc, Completion } from "./analyzer-worker";

export type { Diag, DefLoc, Completion };

type QueryKind = "hover" | "definition" | "completions";

export class Analyzer {
  private worker: Worker;
  private nextId = 1;
  private pending = new Map<number, (result: unknown) => void>();
  /// Diagnostics for the file at `path` (so multi-file re-analysis can mark each
  /// file's own model). `version` lets the caller drop stale results per file.
  onDiagnostics: (version: number, diags: Diag[], path: string) => void = () => {};

  constructor(wasmUrl: string) {
    this.worker = new Worker(new URL("./analyzer-worker.ts", import.meta.url), { type: "module" });
    this.worker.onmessage = (e) => {
      const m = e.data;
      if (m.type === "diagnostics") this.onDiagnostics(m.version, m.diags as Diag[], m.path as string);
      else if (m.type === "queryResult") {
        const resolve = this.pending.get(m.id);
        if (resolve) {
          this.pending.delete(m.id);
          resolve(m.result);
        }
      } else if (m.type === "initError") {
        // The wasm never came up — fail loudly and drain any waiters (so the
        // language providers return null instead of hanging).
        console.error("[analyzer] wasm init failed:", m.error);
        this.failPending();
      }
    };
    // A worker load/runtime error (or a message that couldn't be cloned) also
    // leaves queries unanswered — surface it and drain the waiters.
    this.worker.onerror = (e) => {
      console.error("[analyzer] worker error:", e.message, `${e.filename}:${e.lineno}`);
      this.failPending();
    };
    this.worker.onmessageerror = () => {
      console.error("[analyzer] worker message deserialization error");
      this.failPending();
    };
    fetch(wasmUrl)
      .then((r) => {
        if (!r.ok) throw new Error(`${r.status} ${r.statusText}`);
        return r.arrayBuffer();
      })
      .then((buf) => this.worker.postMessage({ type: "init", wasm: buf }, [buf]))
      .catch((err) => {
        console.error("[analyzer] failed to load", wasmUrl, err);
        this.failPending();
      });
  }

  /// Resolve every outstanding query with null so awaiting providers don't hang.
  private failPending() {
    for (const [, resolve] of this.pending) resolve(null);
    this.pending.clear();
  }

  /// Analyze `source` for the file at `path` (the caller debounces). `version`
  /// lets the caller drop stale results per file; `modules` are the other
  /// project files (module name → source) so cross-file imports resolve.
  analyze(source: string, version: number, path: string, modules?: Record<string, string>) {
    this.worker.postMessage({ type: "analyze", source, version, path, modules });
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
