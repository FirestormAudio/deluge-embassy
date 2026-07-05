// File browser: a collapsible folder tree over the project. Click a file to open
// its tab; set the Run entry; create/rename/delete files (folders are implicit
// from `/`-separated paths). Uses prompt()/confirm() for names — minimal but
// functional.
import { ProjectStore, fileTree, type TreeNode } from "./project";

export class FileBrowser {
  private collapsed = new Set<string>();

  constructor(
    private root: HTMLElement,
    private store: ProjectStore,
  ) {
    this.root.querySelector("#fb-new")?.addEventListener("click", () => this.newFile());
  }

  private newFile(dir = "") {
    let path = prompt("New file path:", dir ? `${dir}/untitled.wren` : "untitled.wren");
    if (!path) return;
    path = path.trim();
    if (!/\.\w+$/.test(path)) path += ".wren"; // default extension
    if (this.store.project.files[path] != null) {
      alert("A file with that path already exists.");
      return;
    }
    this.store.create(path);
  }

  render() {
    const list = this.root.querySelector<HTMLElement>("#fb-tree")!;
    list.replaceChildren();
    for (const node of fileTree(this.store.project.files)) list.appendChild(this.renderNode(node, 0));
  }

  private renderNode(node: TreeNode, depth: number): HTMLElement {
    const row = document.createElement("div");
    row.className = node.dir ? "fb-folder" : "fb-file";
    row.style.paddingLeft = `${8 + depth * 12}px`;

    if (node.dir) {
      const collapsed = this.collapsed.has(node.path);
      row.innerHTML = `<span class="fb-twisty">${collapsed ? "▸" : "▾"}</span><span class="fb-label">${node.name}</span><button class="fb-add" title="new file in ${node.name}">＋</button>`;
      row.querySelector(".fb-twisty")!.addEventListener("click", () => {
        collapsed ? this.collapsed.delete(node.path) : this.collapsed.add(node.path);
        this.render();
      });
      row.querySelector(".fb-label")!.addEventListener("click", () => {
        collapsed ? this.collapsed.delete(node.path) : this.collapsed.add(node.path);
        this.render();
      });
      row.querySelector(".fb-add")!.addEventListener("click", (e) => { e.stopPropagation(); this.newFile(node.path); });

      const wrap = document.createElement("div");
      wrap.appendChild(row);
      if (!collapsed) for (const c of node.children ?? []) wrap.appendChild(this.renderNode(c, depth + 1));
      return wrap;
    }

    const isEntry = node.path === this.store.project.entry;
    row.classList.toggle("is-entry", isEntry);
    row.classList.toggle("is-active", node.path === this.store.project.active);
    row.innerHTML =
      `<button class="fb-entry" title="${isEntry ? "Run entry" : "set as Run entry"}">${isEntry ? "▶" : "○"}</button>` +
      `<span class="fb-label">${node.name}</span>` +
      `<button class="fb-act fb-rename" title="rename" aria-label="rename ${node.name}">✎</button>` +
      `<button class="fb-act fb-del" title="delete" aria-label="delete ${node.name}">×</button>`;
    row.querySelector(".fb-entry")!.addEventListener("click", (e) => { e.stopPropagation(); this.store.setEntry(node.path); });
    row.querySelector(".fb-label")!.addEventListener("click", () => this.store.activate(node.path));
    row.querySelector(".fb-rename")!.addEventListener("click", (e) => {
      e.stopPropagation();
      const np = prompt("Rename to:", node.path);
      if (np && np.trim() && np.trim() !== node.path) this.store.rename(node.path, np.trim());
    });
    row.querySelector(".fb-del")!.addEventListener("click", (e) => {
      e.stopPropagation();
      if (confirm(`Delete ${node.path}?`)) this.store.remove(node.path);
    });
    return row;
  }
}
