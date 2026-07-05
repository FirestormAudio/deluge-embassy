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
import type { StackFrameDto, ScopeDto, VariableDto } from "./sab";

type Pane = "files" | "debug";

const IDLE_HINT = "— start a debug session to inspect —";
const VARS_HINT = "— locals appear here at a stop —";

export class DebugSidebar {
  private readonly paneFiles: HTMLButtonElement;
  private readonly paneDebug: HTMLButtonElement;
  private readonly fbBody: HTMLElement;
  private readonly view: HTMLElement;
  private readonly fbNew: HTMLElement | null;
  private callstackBody!: HTMLElement;
  private variablesBody!: HTMLElement;
  /** Bumped on every VARIABLES render so stale async fetches self-abort. */
  private varRenderSeq = 0;

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
    // The variables tree follows the selected frame (the top frame is selected
    // by default on every stop — see `onStopped`).
    this.session.on("frameSelected", (id) => void this.onFrameSelected(id));

    this.clearStack();
    this.clearVariables();
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
    if (s !== "paused") {
      this.clearStack();
      this.clearVariables();
    }
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
    // DRIVE (Task 5.2): launch-time input config, usable even while idle so you
    // configure a note before hitting Debug. Sits above the run-state sections.
    const drive = this.section("drive", "dbg-drive");
    this.buildDriveForm(drive.body);
    const call = this.section("call stack", "dbg-callstack");
    this.callstackBody = call.body;
    const vars = this.section("variables", "dbg-variables");
    this.variablesBody = vars.body;
    this.view.append(drive.root, call.root, vars.root);
  }

  /**
   * The DRIVE form: a MIDI note + a control-rate block count fired at launch so a
   * breakpoint inside a driven callback (e.g. a `Midi.onNoteOn` body) stops. The
   * values write straight onto the shared `DebugSession` (`setDrive`); the
   * toolbar's Debug reads `session.drive` at start. A blank note = fire nothing
   * (a plain run) — the default, so existing runs are unchanged.
   */
  private buildDriveForm(body: HTMLElement): void {
    body.classList.add("dbg-drive-form");

    const noteRow = this.driveField(
      "note",
      "dbg-drive-note",
      "MIDI note to fire on Debug (0–127, blank = none)",
      { min: 0, max: 127, placeholder: "none" },
      (raw) => {
        const n = raw === "" ? -1 : Math.max(0, Math.min(127, Math.round(Number(raw))));
        this.session.setDrive({ note: Number.isFinite(n) ? n : -1 });
      },
    );

    const blocksRow = this.driveField(
      "blocks",
      "dbg-drive-blocks",
      "Control-rate blocks (ticks) to drive after the note",
      { min: 0, placeholder: "0", value: "0" },
      (raw) => {
        const b = raw === "" ? 0 : Math.max(0, Math.round(Number(raw)));
        this.session.setDrive({ blocks: Number.isFinite(b) ? b : 0 });
      },
    );

    const hint = document.createElement("div");
    hint.className = "dbg-hint dbg-drive-hint";
    hint.textContent = "fires this note (+ ticks) at launch so callback breakpoints hit.";

    body.append(noteRow, blocksRow, hint);
  }

  /** One labelled numeric drive input (`.oct-ctrl`-styled), wired to `onInput`. */
  private driveField(
    label: string,
    id: string,
    aria: string,
    attrs: { min?: number; max?: number; placeholder?: string; value?: string },
    onInput: (raw: string) => void,
  ): HTMLElement {
    const row = document.createElement("label");
    row.className = "dbg-drive-row";
    row.htmlFor = id;

    const name = document.createElement("span");
    name.className = "dbg-drive-label";
    name.textContent = label;

    const input = document.createElement("input");
    input.type = "number";
    input.id = id;
    input.className = "dbg-drive-input";
    input.inputMode = "numeric";
    input.setAttribute("aria-label", aria);
    input.title = aria;
    if (attrs.min !== undefined) input.min = String(attrs.min);
    if (attrs.max !== undefined) input.max = String(attrs.max);
    if (attrs.placeholder !== undefined) input.placeholder = attrs.placeholder;
    if (attrs.value !== undefined) input.value = attrs.value;
    input.addEventListener("input", () => onInput(input.value.trim()));

    row.append(name, input);
    return row;
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

  // ── variables (locals tree) ────────────────────────────────────────────────────

  /** A frame was selected (top frame by default on each stop) → render its scopes. */
  private async onFrameSelected(frameId: number | null): Promise<void> {
    const seq = ++this.varRenderSeq;
    if (frameId == null) {
      this.clearVariables();
      return;
    }
    const scopes = await this.session.scopes(frameId);
    if (seq !== this.varRenderSeq) return; // a newer selection superseded us
    this.variablesBody.replaceChildren();
    if (!scopes.length) {
      this.variablesBody.appendChild(this.hint(VARS_HINT));
      return;
    }
    // Render one sub-group per scope. The first scope ("Locals") is expanded by
    // default; the rest (e.g. the prelude-flooded "Module" scope) stay collapsed
    // and lazily fetch their variables only when the user opens them.
    scopes.forEach((sc, i) => this.variablesBody.appendChild(this.scopeGroup(sc, i === 0, seq)));
  }

  /** A collapsible scope sub-group: `.pane-legend` sub-header + a lazy body. */
  private scopeGroup(scope: ScopeDto, expand: boolean, seq: number): HTMLElement {
    const root = document.createElement("div");
    root.className = "dbg-scope";

    const head = document.createElement("button");
    head.type = "button";
    head.className = "pane-legend dbg-scope-head";
    head.setAttribute("aria-expanded", String(expand));
    const twisty = document.createElement("span");
    twisty.className = "fb-twisty";
    twisty.setAttribute("aria-hidden", "true");
    twisty.textContent = expand ? "▾" : "▸";
    const label = document.createElement("span");
    label.textContent = scope.name;
    head.append(twisty, label);

    const body = document.createElement("div");
    body.className = "dbg-scope-body";
    body.hidden = !expand;

    let loaded = false;
    const fill = async (): Promise<void> => {
      if (loaded) return;
      loaded = true;
      const vars = await this.session.variables(scope.variablesReference);
      if (!body.isConnected || seq !== this.varRenderSeq) {
        loaded = false; // superseded before we could paint — allow a later fill
        return;
      }
      for (const v of vars) body.appendChild(this.varNode(v, seq));
    };

    head.addEventListener("click", () => {
      const open = head.getAttribute("aria-expanded") !== "true";
      head.setAttribute("aria-expanded", String(open));
      twisty.textContent = open ? "▾" : "▸";
      body.hidden = !open;
      if (open) void fill();
    });

    root.append(head, body);
    if (expand) void fill();
    return root;
  }

  /** One variable row (`name = value`); expandable ones lazily fetch children. */
  private varNode(v: VariableDto, seq: number): HTMLElement {
    const node = document.createElement("div");
    node.className = "dbg-var";
    node.dataset.name = v.name;

    const row = document.createElement("div");
    row.className = "dbg-var-row";

    const twisty = document.createElement("span");
    twisty.className = "fb-twisty";
    twisty.setAttribute("aria-hidden", "true");

    const name = document.createElement("span");
    name.className = "dbg-var-name";
    name.textContent = v.name;
    const eq = document.createElement("span");
    eq.className = "dbg-var-eq";
    eq.textContent = "=";
    eq.setAttribute("aria-hidden", "true");
    const val = document.createElement("span");
    val.className = "dbg-var-val";
    val.textContent = v.value;

    row.append(twisty, name, eq, val);
    node.appendChild(row);

    if (v.variablesReference <= 0) {
      twisty.textContent = ""; // leaf: empty spacer keeps names aligned
      return node;
    }

    twisty.textContent = "▸";
    const children = document.createElement("div");
    children.className = "dbg-var-children";
    children.hidden = true;
    node.appendChild(children);

    row.setAttribute("role", "button");
    row.tabIndex = 0;
    row.setAttribute("aria-expanded", "false");
    row.setAttribute("aria-label", `${v.name} = ${v.value}`);

    let loaded = false;
    const fill = async (): Promise<void> => {
      if (loaded) return;
      loaded = true; // fetch children exactly once (collapse→expand won't refetch)
      const kids = await this.session.variables(v.variablesReference);
      if (!children.isConnected || seq !== this.varRenderSeq) {
        loaded = false;
        return;
      }
      for (const c of kids) children.appendChild(this.varNode(c, seq));
    };
    const toggle = (): void => {
      const open = row.getAttribute("aria-expanded") !== "true";
      row.setAttribute("aria-expanded", String(open));
      twisty.textContent = open ? "▾" : "▸";
      children.hidden = !open;
      if (open) void fill();
    };
    row.addEventListener("click", toggle);
    row.addEventListener("keydown", (e) => {
      if (e.key === "Enter" || e.key === " ") {
        e.preventDefault();
        toggle();
      }
    });
    return node;
  }

  private clearVariables(): void {
    if (!this.variablesBody) return;
    this.varRenderSeq++; // abort any in-flight fetch
    this.variablesBody.replaceChildren(this.hint(VARS_HINT));
  }
}
