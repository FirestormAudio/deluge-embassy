// Debug transport toolbar + paused-line highlight (Task 4.2, refactored in 4.3).
//
// A single "Debug" trigger in the topbar (left of the amber Run button) starts a
// debug run of the entry file; while a session is live it expands into a compact
// transport cluster (Continue / Step Over / Step In / Step Out / Stop). On a
// `stopped` event the paused line gets a phosphor highlight + a ▶ marker (the
// "execution beam rests here"); resuming clears it. See the shared design doc
// (.superpowers/sdd/phase4-design-direction.md): breakpoints are amber, the
// paused line is phosphor (cyan), red stays errors-only.
//
// As of 4.3 this is a *view* over the shared `DebugSession` (src/debug/session.ts)
// — it no longer owns the DebugController or the state machine; it drives the
// session and reflects its `state`/`stopped` events. The DebugSession owns the
// controller lifecycle, output routing, isolation gate and compile pre-flight.
import type * as Monaco from "monaco-editor/esm/vs/editor/editor.api";
import type { ProjectStore } from "../project";
import type { Tabs } from "../tabs";
import type { DebugSession, DebugState } from "./session";

export interface DebugToolbarDeps {
  monaco: typeof Monaco;
  editor: Monaco.editor.IStandaloneCodeEditor;
  tabs: Tabs;
  store: ProjectStore;
  /** The shared session this toolbar drives + reflects. */
  session: DebugSession;
}

export class DebugToolbar {
  private decorations: Monaco.editor.IEditorDecorationsCollection | null = null;

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
    mk("continue", "▸", "Continue", () => void this.deps.session.continue());
    mk("stepOver", "⤼", "Step over", () => void this.deps.session.stepOver());
    mk("stepIn", "⤓", "Step into", () => void this.deps.session.stepIn());
    mk("stepOut", "⤒", "Step out", () => void this.deps.session.stepOut());
    mk("stop", "■", "Stop", () => this.deps.session.stop());

    this.root.append(this.debugBtn, this.cluster);

    // Reflect the shared session: paused-line on stop, clear on run/idle,
    // transport enable/disable on every state change.
    this.deps.session.on("stopped", (ev) => {
      this.showEntry();
      this.showPausedLine(ev.line);
    });
    this.deps.session.on("state", (s) => this.onState(s));

    this.applyIsolation();
    this.onState(this.deps.session.state);
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

  /** Cross-origin isolation is required for the SharedArrayBuffer transport. */
  private applyIsolation() {
    if (!this.deps.session.isIsolated()) {
      this.debugBtn.disabled = true;
      this.debugBtn.title = "Debugging needs cross-origin isolation (SharedArrayBuffer)";
      this.debugBtn.setAttribute("aria-label", this.debugBtn.title);
    }
  }

  private onState(s: DebugState) {
    const live = s === "paused" || s === "running" || s === "starting";
    this.cluster.hidden = !live;
    this.debugBtn.hidden = live;
    // Transport is operable only while paused; disabled while a command runs.
    const paused = s === "paused";
    for (const [id, b] of Object.entries(this.transport)) {
      b.disabled = id === "stop" ? !live : !paused;
    }
    // Clear the paused-line decoration whenever we leave `paused`.
    if (!paused) this.clearPausedLine();
  }

  /** Start a debug session for the entry file. */
  async start() {
    const { store, tabs, session } = this.deps;
    const entry = store.project.entry;
    const source = tabs.model(entry).getValue();
    const breakpoints = store.breakpointsFor(entry);
    await session.start(source, breakpoints);
  }

  /** Debug runs the entry file — make sure it's the shown model at a stop. */
  private showEntry() {
    const { store } = this.deps;
    if (store.project.active !== store.project.entry) store.activate(store.project.entry);
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
