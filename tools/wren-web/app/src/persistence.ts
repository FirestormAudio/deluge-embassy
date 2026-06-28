// Script persistence: localStorage autosave + shareable permalinks. On load, a
// `#s=` permalink wins over the autosaved script, which wins over the default
// example.

const KEY = "wren-deluge:script";

// Unicode-safe base64url (btoa only handles latin1). Shared with the project
// store (src/project.ts).
export function b64encode(s: string): string {
  return btoa(String.fromCharCode(...new TextEncoder().encode(s)))
    .replace(/\+/g, "-")
    .replace(/\//g, "_")
    .replace(/=+$/, "");
}
export function b64decode(s: string): string {
  const b = atob(s.replace(/-/g, "+").replace(/_/g, "/"));
  return new TextDecoder().decode(Uint8Array.from(b, (c) => c.charCodeAt(0)));
}

/// The legacy single-script localStorage key (for migration to projects).
export const LEGACY_SCRIPT_KEY = KEY;

export function saveLocal(content: string) {
  try {
    localStorage.setItem(KEY, content);
  } catch {
    /* storage may be unavailable (private mode); autosave is best-effort */
  }
}

export function loadLocal(): string | null {
  try {
    return localStorage.getItem(KEY);
  } catch {
    return null;
  }
}

/// A `#s=<base64>` permalink, if present and decodable.
export function loadPermalink(): string | null {
  const h = location.hash.replace(/^#/, "");
  if (!h.startsWith("s=")) return null;
  try {
    return b64decode(h.slice(2));
  } catch {
    return null;
  }
}

/// Build a shareable URL whose hash encodes `content`.
export function permalink(content: string): string {
  return `${location.origin}${location.pathname}#s=${b64encode(content)}`;
}
