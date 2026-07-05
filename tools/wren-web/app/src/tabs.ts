// Tabbed editors: one Monaco model per open project file, a tab strip, and
// model-swapping on the shared editor. Structural changes come from the
// ProjectStore (open/close/activate); content is mirrored back per keystroke by
// main.ts.
import * as monaco from "monaco-editor/esm/vs/editor/editor.api";
import type { ProjectStore } from "./project";

const basename = (path: string) => path.split("/").pop() ?? path;

export class Tabs {
  private models = new Map<string, monaco.editor.ITextModel>();

  constructor(
    private editor: monaco.editor.IStandaloneCodeEditor,
    private tabBar: HTMLElement,
    private store: ProjectStore,
  ) {}

  /// The Monaco model for `path` (created from the stored content on first use).
  model(path: string): monaco.editor.ITextModel {
    let m = this.models.get(path);
    if (!m) {
      m = monaco.editor.createModel(this.store.read(path), "wren", monaco.Uri.parse(`inmemory://p/${path}`));
      this.models.set(path, m);
    }
    return m;
  }

  activeModel(): monaco.editor.ITextModel | null {
    const a = this.store.project.active;
    return a ? this.model(a) : null;
  }

  showActive() {
    const m = this.activeModel();
    if (m && this.editor.getModel() !== m) this.editor.setModel(m);
  }

  /// Discard every cached model — the whole project was replaced, so any
  /// surviving same-path model still holds the old file's text and must be
  /// rebuilt from the new store contents on the next render().
  reset() {
    this.editor.setModel(null); // detach before disposing the current model
    for (const m of this.models.values()) m.dispose();
    this.models.clear();
  }

  /// Drop models for files that no longer exist (deleted/renamed).
  private prune() {
    for (const [path, m] of this.models) {
      if (this.store.project.files[path] == null) {
        m.dispose();
        this.models.delete(path);
      }
    }
  }

  render() {
    this.prune();
    this.tabBar.replaceChildren();
    for (const path of this.store.project.open) {
      const tab = document.createElement("div");
      tab.className =
        "tab" +
        (path === this.store.project.active ? " active" : "") +
        (path === this.store.project.entry ? " entry" : "");
      tab.title = path;
      const name = document.createElement("span");
      name.className = "tab-name";
      name.textContent = basename(path);
      name.addEventListener("click", () => this.store.activate(path));
      const close = document.createElement("button");
      close.className = "tab-close";
      close.setAttribute("aria-label", `close ${basename(path)}`);
      close.textContent = "×";
      close.addEventListener("click", (e) => { e.stopPropagation(); this.store.close(path); });
      tab.append(name, close);
      this.tabBar.appendChild(tab);
    }
    this.showActive();
  }
}
