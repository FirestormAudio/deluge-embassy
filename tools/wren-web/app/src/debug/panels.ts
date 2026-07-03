// Task 4.3: the left-sidebar debug view (VSCode "Run and Debug"-style).
//
// The left pane (`.file-browser`) gains a segmented header — [ files · debug ] —
// that swaps the pane body between the file tree (`#fb-body`/`#fb-tree`) and a
// stacked debug view (`#debug-view`). The debug view is a vertical accordion of
// collapsible sections: CALL STACK (this task) and VARIABLES (a 4.4 placeholder).
//
// The debug segment auto-activates while a session is live and returns the pane
// to whatever the user last chose when the run ends (a manual switch is
// respected). On each `stopped` the CALL STACK section renders one row per frame
// (from `session.stackTrace()`); the top frame is selected by default and
// clicking a frame stores the selection on the session (drives 4.4's scopes).
//
// Design language (.superpowers/sdd/phase4-design-direction.md): reuse the
// `.pane-legend`/`.fb-*` idioms + the `.fb-twisty` accordion glyph; the selected
// frame mirrors `.fb-file.is-active` (phosphor-tinted bg), the current ▶ marker
// is phosphor. No new palette/fonts.
import type { DebugSession, DebugState } from "./session";
import type { StackFrameDto } from "./sab";

type Pane = "files" | "debug";

const IDLE_HINT = "— start a debug session to inspect —";

export class DebugSidebar {
  private readonly paneFiles: HTMLButtonElement;
  private readonly paneDebug: HTMLButtonElement;
  private readonly fbBody: HTMLElement;
  private readonly view: HTMLElement;
  private readonly fbNew: HTMLElement | null;
  private callstackBody!: HTMLElement;

  /** What the user last chose manually — restored when a run ends. */
  private userChoice: Pane = "files";
  /** Currently shown pane (exposed for tests). */
  shown: Pane = "files";

  constructor(
    root: HTMLElement,
    private session: DebugSession,
  ) {
    this.paneFiles = root.querySelector<HTMLButtonElement>("#pane-files")!;
    this.paneDebug = root.querySelector<HTMLButtonElement>("#pane-debug")!;
    this.fbBody = root.querySelector<HTMLElement>("#fb-body")!;
    this.view = root.querySelector<HTMLElement>("#debug-view")!;
    this.fbNew = root.querySelector<HTMLElement>("#fb-new");

    this.buildView();
    this.paneFiles.addEventListener("click", () => this.activate("files", true));
    this.paneDebug.addEventListener("click", () => this.activate("debug", true));

    this.session.on("state", (s) => this.onState(s));
    this.session.on("stopped", () => void this.onStopped());

    this.clearStack();
    this.activate("files");
  }

  // ── pane switching ───────────────────────────────────────────────────────────

  private activate(which: Pane, user = false): void {
    if (user) this.userChoice = which;
    this.shown = which;
    const isFiles = which === "files";
    this.fbBody.hidden = !isFiles;
    this.view.hidden = isFiles;
    if (this.fbNew) this.fbNew.hidden = !isFiles;
    this.paneFiles.classList.toggle("is-active", isFiles);
    this.paneDebug.classList.toggle("is-active", !isFiles);
    this.paneFiles.setAttribute("aria-selected", String(isFiles));
    this.paneDebug.setAttribute("aria-selected", String(!isFiles));
  }

  private onState(s: DebugState): void {
    // Auto-show the debug view while a session is live; restore the user's last
    // choice when it ends. Clear the stack whenever we're not paused.
    if (s === "starting") this.activate("debug");
    if (s === "idle") this.activate(this.userChoice);
    if (s !== "paused") this.clearStack();
  }

  private async onStopped(): Promise<void> {
    this.activate("debug"); // ensure the stack is in view on every stop
    const { stackFrames } = await this.session.stackTrace();
    this.renderStack(stackFrames);
    if (stackFrames.length) this.session.selectFrame(stackFrames[0].id);
  }

  // ── view construction ────────────────────────────────────────────────────────

  private buildView(): void {
    this.view.replaceChildren();
    const call = this.section("call stack", "dbg-callstack");
    this.callstackBody = call.body;
    const vars = this.section("variables", "dbg-variables");
    vars.body.appendChild(this.hint("— locals appear here (4.4) —"));
    this.view.append(call.root, vars.root);
  }

  /** A collapsible accordion section (`.pane-legend` head + `.fb-twisty`). */
  private section(title: string, bodyId: string): { root: HTMLElement; body: HTMLElement } {
    const root = document.createElement("section");
    root.className = "dbg-section";

    const head = document.createElement("button");
    head.type = "button";
    head.className = "pane-legend dbg-section-head";
    head.setAttribute("aria-expanded", "true");
    const twisty = document.createElement("span");
    twisty.className = "fb-twisty";
    twisty.textContent = "▾";
    twisty.setAttribute("aria-hidden", "true");
    const label = document.createElement("span");
    label.textContent = title;
    head.append(twisty, label);

    const body = document.createElement("div");
    body.className = "dbg-section-body";
    body.id = bodyId;

    head.addEventListener("click", () => {
      const collapsed = root.classList.toggle("collapsed");
      twisty.textContent = collapsed ? "▸" : "▾";
      head.setAttribute("aria-expanded", String(!collapsed));
    });

    root.append(head, body);
    return { root, body };
  }

  private hint(text: string): HTMLElement {
    const el = document.createElement("div");
    el.className = "dbg-hint";
    el.textContent = text;
    return el;
  }

  // ── call stack ───────────────────────────────────────────────────────────────

  private renderStack(frames: StackFrameDto[]): void {
    this.callstackBody.replaceChildren();
    if (!frames.length) {
      this.callstackBody.appendChild(this.hint(IDLE_HINT));
      return;
    }
    frames.forEach((f, i) => this.callstackBody.appendChild(this.frameRow(f, i === 0)));
    // Top frame is selected by default.
    this.selectRow(frames[0].id);
  }

  private frameRow(f: StackFrameDto, isTop: boolean): HTMLElement {
    const row = document.createElement("div");
    row.className = "dbg-frame";
    row.dataset.frameId = String(f.id);
    row.setAttribute("role", "button");
    row.tabIndex = 0;
    row.setAttribute("aria-label", `${f.name} at ${f.module} line ${f.line}`);

    const mark = document.createElement("span");
    mark.className = isTop ? "dbg-frame-mark" : "dbg-frame-mark dim";
    mark.textContent = isTop ? "▶" : "·";
    mark.setAttribute("aria-hidden", "true");

    const name = document.createElement("span");
    name.className = "dbg-frame-name";
    name.textContent = f.name;

    const loc = document.createElement("span");
    loc.className = "dbg-frame-loc";
    loc.textContent = `${f.module}:${f.line}`;

    row.append(mark, name, loc);

    const select = () => {
      this.selectRow(f.id);
      this.session.selectFrame(f.id);
    };
    row.addEventListener("click", select);
    row.addEventListener("keydown", (e) => {
      if (e.key === "Enter" || e.key === " ") {
        e.preventDefault();
        select();
      }
    });
    return row;
  }

  private selectRow(frameId: number): void {
    for (const row of this.callstackBody.querySelectorAll<HTMLElement>(".dbg-frame")) {
      row.classList.toggle("selected", row.dataset.frameId === String(frameId));
    }
  }

  private clearStack(): void {
    if (!this.callstackBody) return;
    this.callstackBody.replaceChildren(this.hint(IDLE_HINT));
  }
}
