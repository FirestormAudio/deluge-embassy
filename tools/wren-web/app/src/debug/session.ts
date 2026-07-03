// Task 4.3: the shared `DebugSession` — one source of truth for a debug run.
//
// Extracted from the 4.2 `DebugToolbar`, which used to own the `DebugController`
// privately. Now BOTH the toolbar (transport buttons + paused-line decoration)
// and the left-sidebar debug view (call stack, and 4.4's variables) observe a
// single session: they call `start/continue/step/stop` and subscribe to events.
//
// Ownership relocated here (behaviour identical to 4.2):
//   - lazy `DebugController` construction on the first `start`;
//   - the `on("output") → log` routing;
//   - the launch/continue/step correlation (a reply is either a new `stopped`
//     or a `terminated` that ends the run) via the controller's promises;
//   - the cross-origin isolation gate + the analyzer pre-flight compile check.
//
// The debugger runs the ENTRY file only (the wasm `dbg_launch` takes an entry
// string + flat breakpoint lines; imported modules are a carry-forward).
import type { DebugController } from "./controller";
import type { DebugEvent, StackFrameDto, ScopeDto, VariableDto } from "./sab";

export type DebugState = "idle" | "starting" | "paused" | "running";

// A pre-flight compile error (line, message) — surfaced verbatim in the console
// and aborts the run before a worker is spawned (the debug core doesn't report
// compile errors yet — see the task report).
export interface PreflightError {
  line: number;
  message: string;
}

export interface DebugSessionDeps {
  /** Threaded debug wasm URL (fetched by the DebugController). */
  wasmUrl: string;
  /** Append a line to the console (kind "out" | "err"), reusing main.ts's log. */
  log: (text: string, kind?: string) => void;
  /** Compile pre-flight: any error-severity diagnostics on the entry, or null. */
  preflight?: () => PreflightError | null;
  /** Overridable for tests; defaults to the real `crossOriginIsolated`. */
  isIsolated?: () => boolean;
}

// Events views can subscribe to. `state` fires on every state transition;
// `stopped` carries the stop event (current line); `output` mirrors the
// controller's stdout/stderr; `terminated` fires when a run ends; `frameSelected`
// carries the frameId a view picked (drives 4.4's scopes).
export interface DebugSessionEvents {
  state: DebugState;
  stopped: Extract<DebugEvent, { event: "stopped" }>;
  output: Extract<DebugEvent, { event: "output" }>;
  terminated: void;
  frameSelected: number | null;
}

type Listener<K extends keyof DebugSessionEvents> = (payload: DebugSessionEvents[K]) => void;

type ResumeKind = "continue" | "stepOver" | "stepIn" | "stepOut";

export class DebugSession {
  private controller: DebugController | null = null;
  private _state: DebugState = "idle";
  private _stopped: Extract<DebugEvent, { event: "stopped" }> | null = null;
  private _selectedFrameId: number | null = null;
  private readonly listeners: { [K in keyof DebugSessionEvents]: Set<Listener<K>> } = {
    state: new Set(),
    stopped: new Set(),
    output: new Set(),
    terminated: new Set(),
    frameSelected: new Set(),
  };

  constructor(private deps: DebugSessionDeps) {}

  // ── observable state ────────────────────────────────────────────────────────

  get state(): DebugState {
    return this._state;
  }
  /** The last `stopped` event (current line/thread) while paused, else null. */
  get stopped(): Extract<DebugEvent, { event: "stopped" }> | null {
    return this._stopped;
  }
  /** The frameId a view selected at the current stop (top frame by default). */
  get selectedFrameId(): number | null {
    return this._selectedFrameId;
  }

  /** Cross-origin isolation is required for the SharedArrayBuffer transport. */
  isIsolated(): boolean {
    if (this.deps.isIsolated) return this.deps.isIsolated();
    // Test seam: `window.__wrenNoIsolation` forces the disabled state without
    // actually dropping COOP/COEP (which the page needs for everything else).
    const w = window as unknown as { __wrenNoIsolation?: boolean };
    return !w.__wrenNoIsolation && crossOriginIsolated;
  }

  // ── event emitter ───────────────────────────────────────────────────────────

  on<K extends keyof DebugSessionEvents>(evt: K, cb: Listener<K>): void {
    this.listeners[evt].add(cb);
  }
  off<K extends keyof DebugSessionEvents>(evt: K, cb: Listener<K>): void {
    this.listeners[evt].delete(cb);
  }
  private emit<K extends keyof DebugSessionEvents>(evt: K, payload: DebugSessionEvents[K]): void {
    for (const cb of this.listeners[evt]) cb(payload);
  }

  private setState(s: DebugState): void {
    this._state = s;
    this.emit("state", s);
  }

  // ── lifecycle ───────────────────────────────────────────────────────────────

  /** Start a debug session for the entry source with the given breakpoint lines. */
  async start(entry: string, breakpoints: number[]): Promise<void> {
    if (this._state !== "idle" || !this.isIsolated()) return;

    // Pre-flight: don't spin up a worker on code the editor already flags as
    // broken — show the compile error verbatim and abort (editor untouched).
    const err = this.deps.preflight?.();
    if (err) {
      this.deps.log(err.message, "err");
      return;
    }

    this.setState("starting");
    try {
      const { DebugController } = await import("./controller");
      this.controller = new DebugController(this.deps.wasmUrl);
      this.controller.on("output", (ev) => {
        if (ev.event !== "output") return;
        this.deps.log(ev.output, ev.category === "stderr" ? "err" : "out");
        this.emit("output", ev);
      });
      await this.controller.whenReady();
      const reply = await this.controller.launch(entry, breakpoints);
      this.onSettle(reply);
    } catch (e) {
      this.deps.log(`debug: ${e instanceof Error ? e.message : String(e)}`, "err");
      this.stop();
    }
  }

  continue(): Promise<void> {
    return this.resume("continue");
  }
  stepOver(): Promise<void> {
    return this.resume("stepOver");
  }
  stepIn(): Promise<void> {
    return this.resume("stepIn");
  }
  stepOut(): Promise<void> {
    return this.resume("stepOut");
  }

  private async resume(kind: ResumeKind): Promise<void> {
    if (!this.controller || this._state !== "paused") return;
    this._stopped = null;
    this._selectedFrameId = null;
    this.setState("running");
    try {
      const reply = await this.controller[kind]();
      this.onSettle(reply);
    } catch (e) {
      this.deps.log(`debug: ${e instanceof Error ? e.message : String(e)}`, "err");
      this.stop();
    }
  }

  /** A launch/continue/step settled: either a new stop, or the run ended. */
  private onSettle(reply: DebugEvent): void {
    if (reply.event === "stopped") {
      this._stopped = reply;
      this.setState("paused");
      this.emit("stopped", reply);
    } else {
      // terminated (ran to completion / no more breakpoints) → back to idle.
      this.emit("terminated", undefined);
      this.stop();
    }
  }

  /** Tear the session down and return to idle. */
  stop(): void {
    this.controller?.dispose();
    this.controller = null;
    this._stopped = null;
    this._selectedFrameId = null;
    this.setState("idle");
  }

  // ── selection (drives 4.4's scopes) ──────────────────────────────────────────

  /** Select a stack frame; emits `frameSelected` so views can fetch its scopes. */
  selectFrame(frameId: number | null): void {
    if (this._selectedFrameId === frameId) return;
    this._selectedFrameId = frameId;
    this.emit("frameSelected", frameId);
  }

  // ── inspection passthroughs (throw/no-op if idle) ────────────────────────────

  stackTrace(threadId = 1): Promise<{ stackFrames: StackFrameDto[]; totalFrames: number }> {
    if (!this.controller) return Promise.resolve({ stackFrames: [], totalFrames: 0 });
    return this.controller.stackTrace(threadId);
  }
  scopes(frameId: number): Promise<ScopeDto[]> {
    if (!this.controller) return Promise.resolve([]);
    return this.controller.scopes(frameId);
  }
  variables(variablesReference: number): Promise<VariableDto[]> {
    if (!this.controller) return Promise.resolve([]);
    return this.controller.variables(variablesReference);
  }
  evaluate(frameId: number, expression: string): Promise<DebugEvent> {
    if (!this.controller) return Promise.reject(new Error("no active debug session"));
    return this.controller.evaluate(frameId, expression);
  }
}
