// Build a Project from a loose tree of files picked off disk (an on-card
// project). Strips the selected top-level folder from each path, keeps only
// .wren files, and picks an entry preferring main.wren / MAIN.WREN.
import { type Project, normalize } from "./project";

export interface TreeEntry {
  /// Relative path as reported by the folder picker, e.g. "SDCARD/lib/voice.wren".
  path: string;
  content: string;
}

/// Drop the first path segment (the selected folder), which webkitdirectory
/// always prefixes.
function stripTop(p: string): string {
  const norm = p.replace(/\\/g, "/");
  const i = norm.indexOf("/");
  return i >= 0 ? norm.slice(i + 1) : norm;
}

/// Returns null if the tree contains no .wren files.
export function projectFromTree(entries: TreeEntry[]): Project | null {
  const files: Record<string, string> = {};
  for (const e of entries) {
    const rel = stripTop(e.path);
    if (!rel.toLowerCase().endsWith(".wren")) continue;
    files[rel] = e.content;
  }
  const paths = Object.keys(files);
  if (paths.length === 0) return null;
  const entry =
    paths.find((p) => p.toLowerCase() === "main.wren") ??
    paths.find((p) => /(^|\/)main\.wren$/i.test(p)) ??
    paths[0];
  return normalize({ files, entry });
}
