// Multi-file project model: a virtual filesystem (path → content) with an entry
// file, persisted to localStorage and shareable as a permalink. Folders are
// implicit from `/`-separated paths. The module name of a file is its path minus
// `.wren` (so `lib/synth.wren` imports as `import "lib/synth"`).
import { b64encode, b64decode, LEGACY_SCRIPT_KEY } from "./persistence";
import { EXAMPLES } from "./examples";

export interface Project {
  files: Record<string, string>;
  entry: string;
  open: string[];
  active: string;
  // Debug breakpoints, keyed by file path → sorted 1-based line numbers.
  // Rides along in the persisted JSON but is deliberately kept out of
  // permalink() (share the code, not someone else's probe points).
  breakpoints: Record<string, number[]>;
}

const KEY = "wren-deluge:project";

/// A file's Wren module name (path without the .wren extension).
export const moduleName = (path: string) => path.replace(/\.wren$/, "");

function cloneBreakpoints(bp: Record<string, number[]>): Record<string, number[]> {
  const out: Record<string, number[]> = {};
  for (const [path, lines] of Object.entries(bp)) out[path] = [...lines];
  return out;
}

function clone(p: Project): Project {
  return { files: { ...p.files }, entry: p.entry, open: [...p.open], active: p.active, breakpoints: cloneBreakpoints(p.breakpoints) };
}

export function defaultProject(): Project {
  const ex = EXAMPLES[0];
  return { files: { ...ex.files }, entry: ex.entry, open: [ex.entry], active: ex.entry, breakpoints: {} };
}

/// A Project from an example (used by the examples menu).
export function projectFromExample(ex: { files: Record<string, string>; entry: string }): Project {
  return { files: { ...ex.files }, entry: ex.entry, open: Object.keys(ex.files).includes(ex.entry) ? [ex.entry] : [], active: ex.entry, breakpoints: {} };
}

function fromPermalink(): Project | null {
  const h = location.hash.replace(/^#/, "");
  if (!h.startsWith("p=")) return null;
  try {
    return normalize(JSON.parse(b64decode(h.slice(2))));
  } catch {
    return null;
  }
}

function fromLocal(): Project | null {
  try {
    const raw = localStorage.getItem(KEY);
    if (raw) return normalize(JSON.parse(raw));
    // Migrate a legacy single script into a one-file project.
    const legacy = localStorage.getItem(LEGACY_SCRIPT_KEY);
    if (legacy != null) {
      return { files: { "main.wren": legacy }, entry: "main.wren", open: ["main.wren"], active: "main.wren", breakpoints: {} };
    }
  } catch {
    /* ignore */
  }
  return null;
}

/// Fill in/repair any missing fields so the rest of the app can trust a Project.
export function normalize(p: Partial<Project>): Project {
  const files = p.files && Object.keys(p.files).length ? p.files : { "main.wren": "" };
  const paths = Object.keys(files);
  const entry = p.entry && files[p.entry] != null ? p.entry : paths.find((x) => x.endsWith(".wren")) ?? paths[0];
  const open = (p.open ?? [entry]).filter((x) => files[x] != null);
  if (open.length === 0) open.push(entry);
  const active = p.active && open.includes(p.active) ? p.active : open[0];
  // Drop breakpoints for files that no longer exist; keep lines sorted + unique.
  const breakpoints: Record<string, number[]> = {};
  for (const [path, lines] of Object.entries(p.breakpoints ?? {})) {
    if (files[path] == null || !Array.isArray(lines)) continue;
    const clean = [...new Set(lines.filter((n) => Number.isInteger(n) && n > 0))].sort((a, b) => a - b);
    if (clean.length) breakpoints[path] = clean;
  }
  return { files, entry, open, active, breakpoints };
}

/// Initial project: permalink > localStorage (with legacy migration) > default.
export function loadInitialProject(): Project {
  return fromPermalink() ?? fromLocal() ?? defaultProject();
}

/// A node in the derived folder tree (for the file browser).
export interface TreeNode {
  name: string;
  path: string; // full path for files; folder path for folders
  dir: boolean;
  children?: TreeNode[];
}

/// Derive a sorted folder tree from the flat path set.
export function fileTree(files: Record<string, string>): TreeNode[] {
  const root: TreeNode = { name: "", path: "", dir: true, children: [] };
  for (const path of Object.keys(files).sort()) {
    const parts = path.split("/");
    let node = root;
    let acc = "";
    for (let i = 0; i < parts.length; i++) {
      acc = acc ? `${acc}/${parts[i]}` : parts[i];
      const isFile = i === parts.length - 1;
      let child = node.children!.find((c) => c.name === parts[i] && c.dir === !isFile);
      if (!child) {
        child = { name: parts[i], path: acc, dir: !isFile, children: isFile ? undefined : [] };
        node.children!.push(child);
      }
      node = child;
    }
  }
  // Folders before files, alphabetical within each.
  const sort = (n: TreeNode) => {
    n.children?.sort((a, b) => (a.dir === b.dir ? a.name.localeCompare(b.name) : a.dir ? -1 : 1));
    n.children?.forEach(sort);
  };
  sort(root);
  return root.children!;
}

/// Owns the live project + persistence; emits `onChange` after every mutation so
/// the browser/tabs re-render.
export class ProjectStore {
  project: Project;
  onChange: () => void = () => {};
  // Fired inside replace(), before onChange, so views holding per-file state
  // (e.g. cached editor models) can discard it before the re-render.
  onReplace: () => void = () => {};
  private saveTimer = 0;

  constructor(initial: Project) {
    this.project = initial;
  }

  private changed() {
    this.onChange();
    clearTimeout(this.saveTimer);
    this.saveTimer = window.setTimeout(() => this.save(), 400);
  }

  save() {
    try {
      localStorage.setItem(KEY, JSON.stringify(this.project));
    } catch {
      /* best-effort */
    }
  }

  paths(): string[] {
    return Object.keys(this.project.files).sort();
  }
  read(path: string): string {
    return this.project.files[path] ?? "";
  }
  /// Write content without re-rendering (called on every keystroke).
  writeQuiet(path: string, content: string) {
    this.project.files[path] = content;
    clearTimeout(this.saveTimer);
    this.saveTimer = window.setTimeout(() => this.save(), 400);
  }

  create(path: string, content = "") {
    if (this.project.files[path] != null) return;
    this.project.files[path] = content;
    this.open(path);
  }
  remove(path: string) {
    delete this.project.files[path];
    delete this.project.breakpoints[path];
    this.project.open = this.project.open.filter((p) => p !== path);
    if (this.project.entry === path) this.project.entry = this.paths().find((p) => p.endsWith(".wren")) ?? "";
    if (this.project.active === path) this.project.active = this.project.open[this.project.open.length - 1] ?? "";
    this.changed();
  }
  rename(oldPath: string, newPath: string) {
    if (this.project.files[newPath] != null || this.project.files[oldPath] == null) return;
    this.project.files[newPath] = this.project.files[oldPath];
    delete this.project.files[oldPath];
    if (this.project.breakpoints[oldPath]) {
      this.project.breakpoints[newPath] = this.project.breakpoints[oldPath];
      delete this.project.breakpoints[oldPath];
    }
    const swap = (p: string) => (p === oldPath ? newPath : p);
    this.project.open = this.project.open.map(swap);
    if (this.project.entry === oldPath) this.project.entry = newPath;
    if (this.project.active === oldPath) this.project.active = newPath;
    this.changed();
  }
  setEntry(path: string) {
    this.project.entry = path;
    this.changed();
  }

  /// Sorted breakpoint lines for a file (empty if none). 4.2 seeds
  /// `controller.launch` from this for the active file.
  breakpointsFor(path: string): number[] {
    return this.project.breakpoints[path] ?? [];
  }
  /// Toggle a breakpoint at `line` for `path`; returns the new state (true = set).
  /// Persists but doesn't re-render tabs/tree (breakpoints are gutter-local).
  toggleBreakpoint(path: string, line: number): boolean {
    const lines = this.project.breakpoints[path] ?? [];
    const set = lines.includes(line);
    this.setBreakpoints(path, set ? lines.filter((l) => l !== line) : [...lines, line]);
    return !set;
  }
  /// Replace a file's breakpoint set (sorted + deduped; empty clears the key).
  setBreakpoints(path: string, lines: number[]) {
    const clean = [...new Set(lines.filter((n) => Number.isInteger(n) && n > 0))].sort((a, b) => a - b);
    if (clean.length) this.project.breakpoints[path] = clean;
    else delete this.project.breakpoints[path];
    // Persist on the debounced timer without a full re-render (the gutter
    // updates its own decorations synchronously).
    clearTimeout(this.saveTimer);
    this.saveTimer = window.setTimeout(() => this.save(), 400);
  }

  open(path: string) {
    if (!this.project.open.includes(path)) this.project.open.push(path);
    this.project.active = path;
    this.changed();
  }
  close(path: string) {
    this.project.open = this.project.open.filter((p) => p !== path);
    if (this.project.active === path) this.project.active = this.project.open[this.project.open.length - 1] ?? "";
    this.changed();
  }
  activate(path: string) {
    if (this.project.files[path] == null) return;
    if (!this.project.open.includes(path)) this.project.open.push(path);
    this.project.active = path;
    this.changed();
  }

  /// Replace the whole project (loading an example, a slot, or an import).
  replace(p: Project) {
    this.project = clone(p);
    this.onReplace();
    this.changed();
  }

  permalink(): string {
    const { files, entry } = this.project;
    return `${location.origin}${location.pathname}#p=${b64encode(JSON.stringify({ files, entry }))}`;
  }
}
