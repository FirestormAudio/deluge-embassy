// Debug transport toolbar + paused-line highlight (Task 4.2).
//
// A single "Debug" trigger in the topbar (left of the amber Run button) starts a
// debug run of the entry file; while a session is live it expands into a compact
// transport cluster (Continue / Step Over / Step In / Step Out / Stop). On a
// `stopped` event the paused line gets a phosphor highlight + a ▶ marker (the
// "execution beam rests here"); resuming clears it. See the shared design doc
// (.superpowers/sdd/phase4-design-direction.md): breakpoints are amber, the
// paused line is phosphor (cyan), red stays errors-only.
//
// The debugger runs the ENTRY file only — the wasm `dbg_launch` takes an entry
// string + flat line breakpoints, no imported modules yet (a documented
// carry-forward). Breakpoints are seeded from the store for the entry file.
import type * as Monaco from "monaco-editor/esm/vs/editor/editor.api";
import type { ProjectStore } from "../project";
import type { Tabs } from "../tabs";
import type { DebugController } from "./controller";
import type { DebugEvent } from "./sab";

// A pre-flight compile error (entry, line, message) — used to abort a debug run
// before spawning the worker and surface the error verbatim in the console.
export interface PreflightError {
  line: number;
  message: string;
}

export interface DebugToolbarDeps {
  monaco: typeof Monaco;
  editor: Monaco.editor.IStandaloneCodeEditor;
  tabs: Tabs;
  store: ProjectStore;
  /** Threaded debug wasm URL (fetched by the DebugController). */
  wasmUrl: string;
  /** Append a line to the console (kind "out" | "err"), reusing main.ts's log. */
  log: (text: string, kind?: string) => void;
  /** Ensure the entry file is the shown model before decorating the stop. */
  showEntry: () => void;
  /** Compile pre-flight: any error-severity diagnostics on the entry, or null. */
  preflight: () => PreflightError | null;
  /** Overridable for tests; defaults to the real `crossOriginIsolated`. */
  isIsolated?: () => boolean;
}

type State = "idle" | "starting" | "paused" | "running";

export class DebugToolbar {
  private controller: DebugController | null = null;
  private decorations: Monaco.editor.IEditorDecorationsCollection | null = null;
  private state: State = "idle";

  private readonly debugBtn: HTMLButtonElement;
  private readonly cluster: HTMLElement;
  private readonly transport: Record<string, HTMLButtonElement> = {};
  readonly root: HTMLElement;

  constructor(private deps: DebugToolbarDeps) {
    this.root = document.createElement("div");
    this.root.className = "dbg-toolbar";

    this.debugBtn = this.button("debug", "Debug", "Debug (run with breakpoints)");
    this.debugBtn.classList.add("ghost-btn", "dbg-start");
    this.debugBtn.addEventListener("click", () => void this.start());

    this.cluster = document.createElement("span");
    this.cluster.className = "dbg-transport";
    this.cluster.hidden = true;
    const mk = (id: string, glyph: string, label: string, on: () => void) => {
      const b = this.button(`dbg-${id}`, glyph, label);
      b.addEventListener("click", on);
      this.transport[id] = b;
      this.cluster.appendChild(b);
      return b;
    };
    mk("continue", "▸", "Continue", () => void this.resume("continue"));
    mk("stepOver", "⤼", "Step over", () => void this.resume("stepOver"));
    mk("stepIn", "⤓", "Step into", () => void this.resume("stepIn"));
    mk("stepOut", "⤒", "Step out", () => void this.resume("stepOut"));
    mk("stop", "■", "Stop", () => this.stop());

    this.root.append(this.debugBtn, this.cluster);
    this.applyIsolation();
  }

  private button(id: string, text: string, label: string): HTMLButtonElement {
    const b = document.createElement("button");
    b.id = id;
    b.type = "button";
    b.textContent = text;
    b.setAttribute("aria-label", label);
    b.title = label;
    return b;
  }

  private isIsolated(): boolean {
    if (this.deps.isIsolated) return this.deps.isIsolated();
    // Test seam: `window.__wrenNoIsolation` forces the disabled state without
    // actually dropping COOP/COEP (which the page needs for everything else).
    const w = window as unknown as { __wrenNoIsolation?: boolean };
    return !w.__wrenNoIsolation && crossOriginIsolated;
  }

  /** Cross-origin isolation is required for the SharedArrayBuffer transport. */
  private applyIsolation() {
    if (!this.isIsolated()) {
      this.debugBtn.disabled = true;
      this.debugBtn.title = "Debugging needs cross-origin isolation (SharedArrayBuffer)";
      this.debugBtn.setAttribute("aria-label", this.debugBtn.title);
    }
  }

  private setState(s: State) {
    this.state = s;
    const live = s === "paused" || s === "running" || s === "starting";
    this.cluster.hidden = !live;
    this.debugBtn.hidden = live;
    // Transport is operable only while paused; disabled while a command runs.
    const paused = s === "paused";
    for (const [id, b] of Object.entries(this.transport)) {
      b.disabled = id === "stop" ? !live : !paused;
    }
  }

  /** Start a debug session for the entry file. */
  async start() {
    if (this.state !== "idle" || !this.isIsolated()) return;

    // Pre-flight: don't spin up a worker on code the editor already flags as
    // broken — show the compile error verbatim and abort (editor untouched).
    const err = this.deps.preflight();
    if (err) {
      this.deps.log(err.message, "err");
      return;
    }

    this.setState("starting");
    const entry = this.deps.store.project.entry;
    const source = this.deps.tabs.model(entry).getValue();
    const breakpoints = this.deps.store.breakpointsFor(entry);

    try {
      const { DebugController } = await import("./controller");
      this.controller = new DebugController(this.deps.wasmUrl);
      this.controller.on("output", (ev) => {
        if (ev.event === "output") this.deps.log(ev.output, ev.category === "stderr" ? "err" : "out");
      });
      await this.controller.whenReady();
      const reply = await this.controller.launch(source, breakpoints);
      this.onSettle(reply);
    } catch (e) {
      this.deps.log(`debug: ${e instanceof Error ? e.message : String(e)}`, "err");
      this.stop();
    }
  }

  private async resume(kind: "continue" | "stepOver" | "stepIn" | "stepOut") {
    if (!this.controller || this.state !== "paused") return;
    this.clearPausedLine();
    this.setState("running");
    try {
      const reply = await this.controller[kind]();
      this.onSettle(reply);
    } catch (e) {
      this.deps.log(`debug: ${e instanceof Error ? e.message : String(e)}`, "err");
      this.stop();
    }
  }

  /** A launch/continue/step settled: either a new stop, or the session ended. */
  private onSettle(reply: DebugEvent) {
    if (reply.event === "stopped") {
      this.deps.showEntry();
      this.showPausedLine(reply.line);
      this.setState("paused");
    } else {
      // terminated (ran to completion / no more breakpoints) → back to idle.
      this.stop();
    }
  }

  /** Tear the session down and return to idle. */
  stop() {
    this.clearPausedLine();
    this.controller?.dispose();
    this.controller = null;
    this.setState("idle");
  }

  private showPausedLine(line: number) {
    const model = this.deps.tabs.model(this.deps.store.project.entry);
    if (this.deps.editor.getModel() !== model) this.deps.editor.setModel(model);
    this.deps.editor.revealLineInCenterIfOutsideViewport(line);
    this.decorations?.clear();
    this.decorations = this.deps.editor.createDecorationsCollection([
      {
        range: new this.deps.monaco.Range(line, 1, line, 1),
        options: {
          isWholeLine: true,
          className: "dbg-current-line",
          glyphMarginClassName: "dbg-current-glyph",
          linesDecorationsClassName: "dbg-current-bar",
        },
      },
    ]);
  }

  private clearPausedLine() {
    this.decorations?.clear();
    this.decorations = null;
  }
}
