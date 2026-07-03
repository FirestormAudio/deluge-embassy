// Breakpoint gutter: click Monaco's glyph margin to toggle a breakpoint on that
// line. The ProjectStore is the source of truth (persistence + reuse by the 4.2
// debug launch, which seeds `controller.launch` from `store.breakpointsFor`);
// this module only mirrors that state into per-model glyph decorations.
import type * as Monaco from "monaco-editor/esm/vs/editor/editor.api";
import type { ProjectStore } from "../project";
import type { Tabs } from "../tabs";

export class BreakpointGutter {
  // Decoration ids per file path — decorations are model-scoped in Monaco, so we
  // deltaDecorations on each file's own model (switching tabs must not clobber
  // another file's breakpoints).
  private ids = new Map<string, string[]>();
  // A single transient "ghost" decoration under the cursor (empty-gutter hover).
  private hoverId: string[] = [];
  private hoverModel: Monaco.editor.ITextModel | null = null;

  constructor(
    private monaco: typeof Monaco,
    private editor: Monaco.editor.IStandaloneCodeEditor,
    private tabs: Tabs,
    private store: ProjectStore,
  ) {
    const GLYPH = this.monaco.editor.MouseTargetType.GUTTER_GLYPH_MARGIN;

    // Toggle on a click in the glyph margin (the leftmost gutter strip).
    this.editor.onMouseDown((e) => {
      if (e.target.type !== GLYPH) return;
      const line = e.target.position?.lineNumber;
      const path = this.store.project.active;
      if (!line || !path) return;
      this.clearHover(); // the ghost becomes a real dot (or clears)
      this.store.toggleBreakpoint(path, line);
      this.renderActive();
    });

    // Ghost dot under the cursor while hovering an empty glyph-margin line.
    this.editor.onMouseMove((e) => {
      const line = e.target.position?.lineNumber;
      const path = this.store.project.active;
      if (e.target.type !== GLYPH || !line || !path || this.store.breakpointsFor(path).includes(line)) {
        this.clearHover();
        return;
      }
      this.setHover(this.tabs.model(path), line);
    });
    this.editor.onMouseLeave(() => this.clearHover());
  }

  private setHover(model: Monaco.editor.ITextModel, line: number) {
    if (this.hoverModel !== model) this.clearHover();
    this.hoverModel = model;
    this.hoverId = model.deltaDecorations(this.hoverId, [
      {
        range: new this.monaco.Range(line, 1, line, 1),
        options: { glyphMarginClassName: "dbg-breakpoint-ghost" },
      },
    ]);
  }

  private clearHover() {
    if (this.hoverModel && this.hoverId.length) {
      this.hoverModel.deltaDecorations(this.hoverId, []);
    }
    this.hoverId = [];
    this.hoverModel = null;
  }

  /// Re-render decorations for the currently active file. Call on tab switch and
  /// after the initial project load so restored breakpoints show immediately.
  renderActive() {
    const path = this.store.project.active;
    if (!path) return;
    const model = this.tabs.model(path);
    const lines = this.store.breakpointsFor(path);
    const decorations: Monaco.editor.IModelDeltaDecoration[] = lines.map((line) => ({
      range: new this.monaco.Range(line, 1, line, 1),
      options: {
        // Follow edits above the line so a breakpoint tracks its statement.
        stickiness: this.monaco.editor.TrackedRangeStickiness.NeverGrowsWhenTypingAtEdges,
        glyphMarginClassName: "dbg-breakpoint",
        glyphMarginHoverMessage: { value: "Breakpoint" },
      },
    }));
    const next = model.deltaDecorations(this.ids.get(path) ?? [], decorations);
    this.ids.set(path, next);
  }
}

/// Factory: wire the gutter to the editor/tabs/store and paint the initial set.
export function setupBreakpointGutter(
  m: typeof Monaco,
  editor: Monaco.editor.IStandaloneCodeEditor,
  tabs: Tabs,
  store: ProjectStore,
): BreakpointGutter {
  const gutter = new BreakpointGutter(m, editor, tabs, store);
  gutter.renderActive();
  return gutter;
}
