// Task 3.3: the page-main-thread `DebugController`.
//
// Owns the shared `WebAssembly.Memory`, the module-main worker (worker.ts) that
// runs the wasm `serve()` loop, and the async SAB event pump. Mirrors the Node
// harness (`tools/wren-web-debug/threads-harness.mjs`) but on the browser main
// thread: events are read with `Atomics.waitAsync` (blocking `Atomics.wait` is
// forbidden on a browser main thread), while the worker side may block.
//
// Correlation follows `serve()` in `src/transport.rs` (deterministic, single
// message in flight):
//   - one `stopped` before any command (serve's pre-loop pump) → resolves launch,
//     also emitted to on("stopped");
//   - inspection commands (stackTrace/scopes/variables/evaluate/threads) → exactly
//     one reply event resolves the command;
//   - run-state commands (continue/next/stepIn/stepOut) → zero or more `output`
//     events (routed to on("output")), then either one `stopped` (resolves) or
//     `terminated` + `exited` (resolves on `terminated`; session then over).

import {
  SabChannel,
  type DebugCmd,
  type DebugEvent,
  type DebugEventName,
  type StackFrameDto,
  type ScopeDto,
  type VariableDto,
  type ThreadDto,
} from "./sab";

// Matches threads-harness.mjs:50 and the target's --max-memory. `initial` is
// generous so heap growth never has to move/realloc under the JS views; the
// shared memory's `maximum` MUST match the wasm module's memory import maximum.
const MEMORY_INITIAL = 512;
const MEMORY_MAXIMUM = 16384;

type EventHandler = (ev: DebugEvent) => void;

interface Pending {
  predicate: (ev: DebugEvent) => boolean;
  resolve: (ev: DebugEvent) => void;
  reject: (err: unknown) => void;
}

export class DebugController {
  private readonly ready: Promise<void>;
  private worker!: Worker;
  private channel!: SabChannel;
  private pending: Pending | null = null;
  private breakpoints: number[] = [];
  private disposed = false;
  private terminated = false;
  private commandChain: Promise<unknown> = Promise.resolve();
  private readonly handlers = new Map<DebugEventName, Set<EventHandler>>();

  constructor(private readonly wasmUrl: string) {
    this.ready = this.init();
  }

  /** Resolves once the worker is up and the SAB is mapped. */
  whenReady(): Promise<void> {
    return this.ready;
  }

  private async init(): Promise<void> {
    const module = await WebAssembly.compileStreaming(fetch(this.wasmUrl));
    const memory = new WebAssembly.Memory({
      initial: MEMORY_INITIAL,
      maximum: MEMORY_MAXIMUM,
      shared: true,
    });

    this.worker = new Worker(new URL("./worker.ts", import.meta.url), { type: "module" });

    const base = await new Promise<number>((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error("timed out waiting for SAB base")), 20000);
      this.worker.onmessage = (e: MessageEvent) => {
        const m = e.data;
        if (m && m.type === "base") {
          clearTimeout(timer);
          resolve(m.base as number);
        } else if (m && m.type === "error") {
          clearTimeout(timer);
          reject(new Error(`worker init error: ${m.error}`));
        }
      };
      this.worker.onerror = (ev: ErrorEvent) => {
        clearTimeout(timer);
        reject(new Error(`worker errored: ${ev.message}`));
      };
      this.worker.postMessage({ type: "init", module, memory });
    });

    this.channel = new SabChannel(memory.buffer, base);
    // Keep listening for post-init worker messages (done / error) without
    // interfering with the SAB event pump.
    this.worker.onmessage = (e: MessageEvent) => {
      const m = e.data;
      if (m && m.type === "error") this.failPending(new Error(`worker error: ${m.error}`));
    };
    void this.pump();
  }

  /** The async SAB event pump: read → dispatch, until disposed. */
  private async pump(): Promise<void> {
    while (!this.disposed) {
      let ev: DebugEvent;
      try {
        ev = await this.channel.readEventAsync();
      } catch (err) {
        if (!this.disposed) this.failPending(err);
        return;
      }
      this.dispatch(ev);
      // After `terminated`+`exited` the session is over; stop pumping so we don't
      // park forever on a seq that will never advance.
      if (ev.event === "exited") return;
    }
  }

  private dispatch(ev: DebugEvent): void {
    // Emit to registered listeners first (a `stopped` is both a command reply
    // AND an event; `output` only ever routes here).
    const set = this.handlers.get(ev.event);
    if (set) for (const cb of set) cb(ev);

    if (ev.event === "terminated") this.terminated = true;

    if (this.pending && this.pending.predicate(ev)) {
      const p = this.pending;
      this.pending = null;
      p.resolve(ev);
    }
  }

  private failPending(err: unknown): void {
    if (this.pending) {
      const p = this.pending;
      this.pending = null;
      p.reject(err);
    }
  }

  /** Install the resolver for the current in-flight command's terminal event. */
  private waitFor(predicate: (ev: DebugEvent) => boolean): Promise<DebugEvent> {
    return new Promise<DebugEvent>((resolve, reject) => {
      this.pending = { predicate, resolve, reject };
    });
  }

  /**
   * Serialize a unit of work so only one command is ever in flight (single SAB
   * slot). Never `writeCmd` while awaiting a reply.
   */
  private enqueue<T>(task: () => Promise<T>): Promise<T> {
    const run = this.commandChain.then(task, task);
    this.commandChain = run.then(
      () => undefined,
      () => undefined,
    );
    return run;
  }

  private inspect(cmd: DebugCmd, replyEvent: DebugEventName, altReplyEvent?: DebugEventName): Promise<DebugEvent> {
    return this.enqueue(async () => {
      await this.ready;
      if (this.terminated) throw new Error("debug session already terminated");
      const reply = this.waitFor((ev) => ev.event === replyEvent || ev.event === altReplyEvent);
      await this.channel.writeCmd(cmd);
      return reply;
    });
  }

  private runState(cmd: DebugCmd): Promise<DebugEvent> {
    return this.enqueue(async () => {
      await this.ready;
      if (this.terminated) throw new Error("debug session already terminated");
      const reply = this.waitFor((ev) => ev.event === "stopped" || ev.event === "terminated");
      await this.channel.writeCmd(cmd);
      return reply;
    });
  }

  // ── public API ─────────────────────────────────────────────────────────────

  /** Store the breakpoint lines applied on the next `launch`. */
  setBreakpoints(lines: number[]): void {
    // There is no live re-set command in the transport; breakpoints are fixed
    // at launch time (dbg_launch consumes them once). Document for Phase 4.
    this.breakpoints = [...lines];
  }

  /**
   * Launch a debug session: post the entry + breakpoints to the worker, resolve
   * when the first `stopped` arrives (the initial breakpoint stop). Also emitted
   * to on("stopped").
   */
  launch(entry: string, breakpoints?: number[]): Promise<DebugEvent> {
    if (breakpoints) this.setBreakpoints(breakpoints);
    return this.enqueue(async () => {
      await this.ready;
      const reply = this.waitFor((ev) => ev.event === "stopped");
      this.worker.postMessage({ type: "launch", entry, bpLines: this.breakpoints });
      return reply;
    });
  }

  continue(): Promise<DebugEvent> {
    return this.runState({ command: "continue" });
  }
  stepOver(): Promise<DebugEvent> {
    return this.runState({ command: "next" });
  }
  stepIn(): Promise<DebugEvent> {
    return this.runState({ command: "stepIn" });
  }
  stepOut(): Promise<DebugEvent> {
    return this.runState({ command: "stepOut" });
  }

  async threads(): Promise<ThreadDto[]> {
    const ev = await this.inspect({ command: "threads" }, "threads");
    return ev.event === "threads" ? ev.threads : [];
  }
  async stackTrace(threadId = 1): Promise<{ stackFrames: StackFrameDto[]; totalFrames: number }> {
    const ev = await this.inspect({ command: "stackTrace", threadId }, "stackTrace");
    return ev.event === "stackTrace" ? { stackFrames: ev.stackFrames, totalFrames: ev.totalFrames } : { stackFrames: [], totalFrames: 0 };
  }
  async scopes(frameId: number): Promise<ScopeDto[]> {
    const ev = await this.inspect({ command: "scopes", frameId }, "scopes");
    return ev.event === "scopes" ? ev.scopes : [];
  }
  async variables(variablesReference: number): Promise<VariableDto[]> {
    const ev = await this.inspect({ command: "variables", variablesReference }, "variables");
    return ev.event === "variables" ? ev.variables : [];
  }
  /** Resolves with the reply — either `evaluate` or `evaluateError`. */
  evaluate(frameId: number, expression: string): Promise<DebugEvent> {
    return this.inspect({ command: "evaluate", frameId, expression }, "evaluate", "evaluateError");
  }

  /**
   * Request an async pause (writes the PAUSE word). NOTE: reserved/unwired — the
   * current `debug_run` builds no `Pause` handle, so nothing polls it yet (see
   * src/sab.rs). Present for layout/API completeness; Phase 4 wiring pending.
   */
  pause(): void {
    this.channel?.pause();
  }

  on(event: DebugEventName, cb: EventHandler): void {
    let set = this.handlers.get(event);
    if (!set) {
      set = new Set();
      this.handlers.set(event, set);
    }
    set.add(cb);
  }
  off(event: DebugEventName, cb: EventHandler): void {
    this.handlers.get(event)?.delete(cb);
  }

  dispose(): void {
    this.disposed = true;
    this.failPending(new Error("controller disposed"));
    this.worker?.terminate();
  }
}
