// Named project slots in localStorage — the multi-project store behind the
// Project menu's Save / Open. Separate from the live working copy
// (`wren-deluge:project`); this key holds every *named* save, each as a full
// workspace (files + entry + open tabs + breakpoints).
import { type Project, normalize } from "./project";

const KEY = "wren-deluge:saved";

function readAll(): Record<string, Project> {
  try {
    const raw = localStorage.getItem(KEY);
    if (!raw) return {};
    const obj = JSON.parse(raw);
    return obj && typeof obj === "object" ? (obj as Record<string, Project>) : {};
  } catch {
    return {};
  }
}

function writeAll(map: Record<string, Project>): boolean {
  try {
    localStorage.setItem(KEY, JSON.stringify(map));
    return true;
  } catch {
    // quota exceeded / private mode — best-effort; callers surface a message.
    return false;
  }
}

/// Names of all saved slots, alphabetically.
export function listSlots(): string[] {
  return Object.keys(readAll()).sort((a, b) => a.localeCompare(b));
}

/// Read a slot, repaired via normalize; null if absent.
export function readSlot(name: string): Project | null {
  const p = readAll()[name];
  return p ? normalize(p) : null;
}

/// Save (or overwrite) a slot. Returns false if storage rejected the write.
export function writeSlot(name: string, project: Project): boolean {
  const map = readAll();
  map[name] = project;
  return writeAll(map);
}

export function deleteSlot(name: string): void {
  const map = readAll();
  delete map[name];
  writeAll(map);
}
