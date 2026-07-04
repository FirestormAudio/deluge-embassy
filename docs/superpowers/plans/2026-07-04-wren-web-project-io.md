# Wren Web Project I/O Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the Wren web editor a project lifecycle — New, Open (named browser slots), Import (a loose `.wren` tree from disk), Save, and Download (a `.zip` for the SD card) — under one **Project ▾** menu, retiring the `example` dropdown.

**Architecture:** Four small, focused modules (`zip.ts`, `slots.ts`, `import-tree.ts`, `menu.ts`) provide dependency-free building blocks; `project-io.ts` wires them into the topbar menu with guards, prompts, and current-slot tracking. `main.ts` mounts the menu and exposes thin `window.wren` hooks for tests, following the existing inspection-hook pattern. Everything runs in-browser over `localStorage`.

**Tech Stack:** TypeScript, Vite, Monaco, Playwright (browser-only test harness — no unit runner). Storage: `localStorage`.

## Global Constraints

- **Zero new npm dependencies** (runtime or dev). The zip writer is hand-rolled; the folder picker uses the native `webkitdirectory` input.
- **Tests are Playwright specs** run from `tools/wren-web/app` (`npx playwright test`). There is no unit-test runner; pure functions are exercised through `window.wren.*` hooks via `page.evaluate`, matching the existing pattern (`markersOf`, `checkMarkers`, `createFile`, …).
- `npx tsc --noEmit` must stay clean; run it and Playwright **from `tools/wren-web/app`** (a stray global `tsc` is picked up elsewhere).
- After changes, force a fresh Playwright build by killing port 4173 first: `fuser -k 4173/tcp 2>/dev/null || true` (the webServer has `reuseExistingServer`).
- `store.project.files` is already kept in sync with the editor on every keystroke via `store.writeQuiet(active, editor.getValue())` (`main.ts:270`), so Save/Download read `store.project.files` directly — **no manual flush**.
- Follow the existing topbar/menu visual language (see `.field`, `.ghost-btn` in `src/style.css`); amber/graphite palette, no boxy borders (prior feedback).
- Storage keys: keep the live working copy under `wren-deluge:project` (existing, untouched); named slots go under a **new** key `wren-deluge:saved`.

---

## File Structure

**New**
- `src/zip.ts` — store-only ZIP writer + CRC-32. Export: `zipFiles(files: Record<string,string>): Uint8Array`.
- `src/slots.ts` — named-slot storage over `wren-deluge:saved`. Exports: `listSlots`, `readSlot`, `writeSlot`, `deleteSlot`.
- `src/import-tree.ts` — build a `Project` from a picked file tree. Export: `projectFromTree(entries: TreeEntry[]): Project | null`, `interface TreeEntry`.
- `src/menu.ts` — accessible popover menu. Exports: `createMenu(button, items)`, `interface MenuItem`, `interface Menu`.
- `src/project-io.ts` — wires actions into the menu; owns guards, current-slot, the import `<input>`, and the download anchor. Export: `setupProjectMenu(opts): void`.

**Modified**
- `src/project.ts` — `export` the existing `normalize`; add `blankProject()`.
- `index.html` — replace the `example` field with the `#project-menu` button.
- `src/main.ts` — drop the `#examples` wiring; call `setupProjectMenu`; add `window.wren` hooks.
- `src/style.css` — menu popover + submenu styling.

**New test**
- `tests/project-io.spec.ts` — all scenarios.

---

## Task 1: Store-only ZIP writer (`zip.ts`)

**Files:**
- Create: `tools/wren-web/app/src/zip.ts`
- Modify: `tools/wren-web/app/src/main.ts` (add one `window.wren` hook)
- Test: `tools/wren-web/app/tests/project-io.spec.ts` (create; first case)

**Interfaces:**
- Produces: `zipFiles(files: Record<string, string>): Uint8Array` — a valid store-method (uncompressed) ZIP; paths use forward slashes; folders implied.
- Produces (hook): `window.wren.zipBytes(files: Record<string,string>): number[]`.

- [ ] **Step 1: Write the failing test**

Create `tools/wren-web/app/tests/project-io.spec.ts`:

```ts
import { test, expect } from "@playwright/test";

// The editor gains a project lifecycle (New / Open / Import / Save / Download)
// under one Project ▾ menu. Pure pieces are exercised via window.wren hooks;
// the menu, folder import, and zip download are driven through the DOM.

type Wren = {
  zipBytes: (files: Record<string, string>) => number[];
};
const wren = <T,>(page: import("@playwright/test").Page, fn: (w: Wren) => T) =>
  page.evaluate(`(${fn})((window).wren)`) as Promise<T>;

// Parse a store-only zip's central directory → the set of entry filenames.
function zipEntryNames(bytes: Uint8Array): string[] {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  // End of central directory is the last 22 bytes (no comment).
  const eocd = bytes.length - 22;
  const count = dv.getUint16(eocd + 10, true);
  let off = dv.getUint32(eocd + 16, true); // central directory offset
  const names: string[] = [];
  for (let i = 0; i < count; i++) {
    const nameLen = dv.getUint16(off + 28, true);
    const extraLen = dv.getUint16(off + 30, true);
    const commentLen = dv.getUint16(off + 32, true);
    names.push(new TextDecoder().decode(bytes.subarray(off + 46, off + 46 + nameLen)));
    off += 46 + nameLen + extraLen + commentLen;
  }
  return names;
}

test("Download zips the project's file tree", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();
  const arr = await wren(page, (w) =>
    w.zipBytes({ "main.wren": "var a = 1\n", "lib/voice.wren": "class Voice {}\n" }),
  );
  const names = zipEntryNames(Uint8Array.from(arr));
  expect(names.sort()).toEqual(["lib/voice.wren", "main.wren"]);
});
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools/wren-web/app && fuser -k 4173/tcp 2>/dev/null; npx playwright test project-io -g "Download zips"`
Expected: FAIL — `w.zipBytes is not a function`.

- [ ] **Step 3: Write the zip writer**

Create `tools/wren-web/app/src/zip.ts`:

```ts
// Minimal store-only (uncompressed, method 0) ZIP writer — bundles a project's
// source tree for the SD card without a zip dependency. Each entry is its raw
// UTF-8 bytes behind a local file header; a central directory + end-of-central-
// directory record close the archive. Folders are implied by the paths.

const CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    t[n] = c >>> 0;
  }
  return t;
})();

function crc32(bytes: Uint8Array): number {
  let c = 0xffffffff;
  for (let i = 0; i < bytes.length; i++) c = CRC_TABLE[(c ^ bytes[i]) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}

interface Entry {
  nameBytes: Uint8Array;
  data: Uint8Array;
  crc: number;
  offset: number;
}

/// Build a store-only ZIP of `files` (path → text content).
export function zipFiles(files: Record<string, string>): Uint8Array {
  const enc = new TextEncoder();
  const chunks: Uint8Array[] = [];
  const entries: Entry[] = [];
  let offset = 0;
  const push = (b: Uint8Array) => {
    chunks.push(b);
    offset += b.length;
  };

  for (const [path, content] of Object.entries(files)) {
    const nameBytes = enc.encode(path);
    const data = enc.encode(content);
    const crc = crc32(data);
    const header = new Uint8Array(30 + nameBytes.length);
    const dv = new DataView(header.buffer);
    dv.setUint32(0, 0x04034b50, true); // local file header signature
    dv.setUint16(4, 20, true); // version needed
    dv.setUint16(6, 0x0800, true); // flags: UTF-8 filename
    dv.setUint16(8, 0, true); // method: store
    dv.setUint16(10, 0, true); // mod time
    dv.setUint16(12, 0, true); // mod date
    dv.setUint32(14, crc, true);
    dv.setUint32(18, data.length, true); // compressed size
    dv.setUint32(22, data.length, true); // uncompressed size
    dv.setUint16(26, nameBytes.length, true);
    dv.setUint16(28, 0, true); // extra length
    header.set(nameBytes, 30);
    entries.push({ nameBytes, data, crc, offset });
    push(header);
    push(data);
  }

  const cdStart = offset;
  for (const e of entries) {
    const rec = new Uint8Array(46 + e.nameBytes.length);
    const dv = new DataView(rec.buffer);
    dv.setUint32(0, 0x02014b50, true); // central directory header signature
    dv.setUint16(4, 20, true); // version made by
    dv.setUint16(6, 20, true); // version needed
    dv.setUint16(8, 0x0800, true); // flags: UTF-8
    dv.setUint16(10, 0, true); // method: store
    dv.setUint16(12, 0, true); // mod time
    dv.setUint16(14, 0, true); // mod date
    dv.setUint32(16, e.crc, true);
    dv.setUint32(20, e.data.length, true);
    dv.setUint32(24, e.data.length, true);
    dv.setUint16(28, e.nameBytes.length, true);
    dv.setUint16(30, 0, true); // extra length
    dv.setUint16(32, 0, true); // comment length
    dv.setUint16(34, 0, true); // disk number
    dv.setUint16(36, 0, true); // internal attrs
    dv.setUint32(38, 0, true); // external attrs
    dv.setUint32(42, e.offset, true); // local header offset
    rec.set(e.nameBytes, 46);
    push(rec);
  }
  const cdSize = offset - cdStart;

  const eocd = new Uint8Array(22);
  const dv = new DataView(eocd.buffer);
  dv.setUint32(0, 0x06054b50, true); // EOCD signature
  dv.setUint16(4, 0, true); // disk number
  dv.setUint16(6, 0, true); // cd start disk
  dv.setUint16(8, entries.length, true); // entries on this disk
  dv.setUint16(10, entries.length, true); // total entries
  dv.setUint32(12, cdSize, true);
  dv.setUint32(16, cdStart, true);
  dv.setUint16(20, 0, true); // comment length
  push(eocd);

  const out = new Uint8Array(offset);
  let p = 0;
  for (const c of chunks) {
    out.set(c, p);
    p += c.length;
  }
  return out;
}
```

- [ ] **Step 4: Add the `window.wren` hook**

In `tools/wren-web/app/src/main.ts`, add the import near the other `src` imports (top of file):

```ts
import { zipFiles } from "./zip";
```

Then inside the `(window ...).wren = { ... }` object (around `main.ts:402`), add one entry:

```ts
    zipBytes: (files: Record<string, string>) => Array.from(zipFiles(files)),
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cd tools/wren-web/app && fuser -k 4173/tcp 2>/dev/null; npx tsc --noEmit && npx playwright test project-io -g "Download zips"`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add tools/wren-web/app/src/zip.ts tools/wren-web/app/src/main.ts tools/wren-web/app/tests/project-io.spec.ts
git commit -m "feat(wren-web): store-only zip writer for project export"
```

---

## Task 2: Named slot storage (`slots.ts`)

**Files:**
- Modify: `tools/wren-web/app/src/project.ts` (export `normalize`)
- Create: `tools/wren-web/app/src/slots.ts`
- Modify: `tools/wren-web/app/src/main.ts` (add slot hooks)
- Test: `tools/wren-web/app/tests/project-io.spec.ts` (add cases)

**Interfaces:**
- Consumes: `normalize(p: Partial<Project>): Project` from `project.ts` (must become exported), `type Project`.
- Produces: `listSlots(): string[]`, `readSlot(name: string): Project | null`, `writeSlot(name: string, project: Project): boolean`, `deleteSlot(name: string): void`.
- Produces (hooks): `window.wren.saveSlot(name)`, `.openSlot(name)`, `.slots()`, `.deleteSlot(name)`.

- [ ] **Step 1: Write the failing test**

Extend `tools/wren-web/app/tests/project-io.spec.ts`. Update the `Wren` type and add cases:

```ts
type Wren = {
  zipBytes: (files: Record<string, string>) => number[];
  setSource: (s: string) => void;
  getSource: () => string;
  files: () => Record<string, string>;
  breakpoints: (p: string) => number[];
  saveSlot: (name: string) => boolean;
  openSlot: (name: string) => void;
  slots: () => string[];
  deleteSlot: (name: string) => void;
};

test("a saved slot round-trips through Open", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();
  await wren(page, (w) => w.setSource("var saved = 42\n"));
  await wren(page, (w) => w.saveSlot("mysong"));
  expect(await wren(page, (w) => w.slots())).toContain("mysong");

  // Change the working copy, then Open the slot → the saved content returns.
  await wren(page, (w) => w.setSource("var changed = 0\n"));
  await wren(page, (w) => w.openSlot("mysong"));
  expect(await wren(page, (w) => w.getSource())).toContain("var saved = 42");
});

test("deleting a slot removes it", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();
  await wren(page, (w) => w.saveSlot("scratch"));
  expect(await wren(page, (w) => w.slots())).toContain("scratch");
  await wren(page, (w) => w.deleteSlot("scratch"));
  expect(await wren(page, (w) => w.slots())).not.toContain("scratch");
});
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools/wren-web/app && fuser -k 4173/tcp 2>/dev/null; npx playwright test project-io -g "round-trips through Open"`
Expected: FAIL — `w.saveSlot is not a function`.

- [ ] **Step 3: Export `normalize`**

In `tools/wren-web/app/src/project.ts`, change the declaration at line 70 from:

```ts
function normalize(p: Partial<Project>): Project {
```

to:

```ts
export function normalize(p: Partial<Project>): Project {
```

- [ ] **Step 4: Write `slots.ts`**

Create `tools/wren-web/app/src/slots.ts`:

```ts
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
```

- [ ] **Step 5: Add the `window.wren` hooks**

In `tools/wren-web/app/src/main.ts`, add the import:

```ts
import { listSlots, readSlot, writeSlot, deleteSlot } from "./slots";
```

Add to the `window.wren` object:

```ts
    saveSlot: (name: string) => writeSlot(name, store.project),
    openSlot: (name: string) => {
      const p = readSlot(name);
      if (p) store.replace(p);
    },
    slots: () => listSlots(),
    deleteSlot: (name: string) => deleteSlot(name),
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `cd tools/wren-web/app && fuser -k 4173/tcp 2>/dev/null; npx tsc --noEmit && npx playwright test project-io -g "slot"`
Expected: PASS (both slot cases).

- [ ] **Step 7: Commit**

```bash
git add tools/wren-web/app/src/project.ts tools/wren-web/app/src/slots.ts tools/wren-web/app/src/main.ts tools/wren-web/app/tests/project-io.spec.ts
git commit -m "feat(wren-web): named project slots in localStorage"
```

---

## Task 3: Import a file tree (`import-tree.ts`)

**Files:**
- Create: `tools/wren-web/app/src/import-tree.ts`
- Modify: `tools/wren-web/app/src/main.ts` (add import hook)
- Test: `tools/wren-web/app/tests/project-io.spec.ts` (add case)

**Interfaces:**
- Consumes: `normalize`, `type Project` from `project.ts`.
- Produces: `interface TreeEntry { path: string; content: string }`, `projectFromTree(entries: TreeEntry[]): Project | null`.
- Produces (hook): `window.wren.importTree(entries: TreeEntry[]): boolean`.

- [ ] **Step 1: Write the failing test**

Add to `tests/project-io.spec.ts` (extend the `Wren` type with `importTree` and `entry`):

```ts
// add to Wren type:
//   importTree: (entries: { path: string; content: string }[]) => boolean;
//   entry: () => string;

test("Import builds a project from a picked tree, stripping the top folder", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();
  const ok = await wren(page, (w) =>
    w.importTree([
      { path: "SDCARD/main.wren", content: "var m = 1\n" },
      { path: "SDCARD/lib/voice.wren", content: "class Voice {}\n" },
      { path: "SDCARD/notes.txt", content: "ignore me" },
    ]),
  );
  expect(ok).toBe(true);
  const files = await wren(page, (w) => w.files());
  expect(Object.keys(files).sort()).toEqual(["lib/voice.wren", "main.wren"]);
  expect(await wren(page, (w) => w.entry())).toBe("main.wren");
});

test("Import returns false when there are no .wren files", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();
  const ok = await wren(page, (w) =>
    w.importTree([{ path: "SDCARD/readme.txt", content: "nothing here" }]),
  );
  expect(ok).toBe(false);
});
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools/wren-web/app && fuser -k 4173/tcp 2>/dev/null; npx playwright test project-io -g "Import builds"`
Expected: FAIL — `w.importTree is not a function`.

- [ ] **Step 3: Write `import-tree.ts`**

Create `tools/wren-web/app/src/import-tree.ts`:

```ts
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
```

- [ ] **Step 4: Add the `window.wren` hook**

In `tools/wren-web/app/src/main.ts`, add the import:

```ts
import { projectFromTree, type TreeEntry } from "./import-tree";
```

Add to the `window.wren` object:

```ts
    importTree: (entries: TreeEntry[]) => {
      const p = projectFromTree(entries);
      if (p) store.replace(p);
      return p != null;
    },
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd tools/wren-web/app && fuser -k 4173/tcp 2>/dev/null; npx tsc --noEmit && npx playwright test project-io -g "Import"`
Expected: PASS (both import cases).

- [ ] **Step 6: Commit**

```bash
git add tools/wren-web/app/src/import-tree.ts tools/wren-web/app/src/main.ts tools/wren-web/app/tests/project-io.spec.ts
git commit -m "feat(wren-web): build a project from an imported file tree"
```

---

## Task 4: Accessible popover menu + Project button with New (`menu.ts`)

This task replaces the `example` dropdown with a working **Project ▾** menu whose only branch is **New** (Blank + templates). Open/Import/Save/Download are added in Task 5.

**Files:**
- Create: `tools/wren-web/app/src/menu.ts`
- Modify: `tools/wren-web/app/src/project.ts` (add `blankProject`)
- Modify: `tools/wren-web/app/index.html` (replace the example field)
- Modify: `tools/wren-web/app/src/main.ts` (remove `#examples` wiring; mount menu)
- Modify: `tools/wren-web/app/src/style.css` (menu styling)
- Test: `tools/wren-web/app/tests/project-io.spec.ts` (add cases)

**Interfaces:**
- Consumes: `EXAMPLES` (`examples.ts`), `projectFromExample` (`project.ts`), `store.replace`.
- Produces: `interface MenuItem { label: string; action?: () => void; submenu?: () => MenuItem[]; danger?: boolean; onDelete?: () => void }`, `interface Menu { close(): void }`, `createMenu(button: HTMLButtonElement, items: () => MenuItem[]): Menu`.
- Produces: `blankProject(): Project` in `project.ts`.

- [ ] **Step 1: Write the failing test**

Add to `tests/project-io.spec.ts`:

```ts
test("the Project menu opens and New → Blank clears the project", async ({ page }) => {
  // New replaces the working copy → it prompts; accept the confirm.
  page.on("dialog", (d) => d.accept());
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  await page.locator("#project-menu").click();
  await expect(page.locator(".menu-popover")).toBeVisible();

  await page.getByRole("menuitem", { name: "New" }).click();
  await page.getByRole("menuitem", { name: "Blank" }).click();

  const files = await wren(page, (w) => w.files());
  expect(Object.keys(files)).toEqual(["main.wren"]);
  expect(files["main.wren"]).toBe("");
});

test("the Project menu closes on Escape", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();
  await page.locator("#project-menu").click();
  await expect(page.locator(".menu-popover")).toBeVisible();
  await page.keyboard.press("Escape");
  await expect(page.locator(".menu-popover")).toHaveCount(0);
});
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools/wren-web/app && fuser -k 4173/tcp 2>/dev/null; npx playwright test project-io -g "Project menu opens"`
Expected: FAIL — `#project-menu` not found.

- [ ] **Step 3: Write `menu.ts`**

Create `tools/wren-web/app/src/menu.ts`:

```ts
// A small accessible popover menu for the topbar (the Project ▾ button). No
// dependency: a button toggles a role="menu" list; a branch item expands its
// submenu inline (accordion). Arrow keys / Home / End move focus among visible
// items, Enter/Space activate, Escape or an outside click closes.

export interface MenuItem {
  label: string;
  action?: () => void; // leaf: run, then close the menu
  submenu?: () => MenuItem[]; // branch: built lazily on each open (live state)
  danger?: boolean; // destructive styling
  onDelete?: () => void; // optional inline ✕ affordance on a row
}

export interface Menu {
  close(): void;
}

/// Wire `button` to open a popover built from `items()` (re-invoked on every
/// open, so dynamic entries like saved slots stay current). The popover is
/// appended to the button's parent.
export function createMenu(button: HTMLButtonElement, items: () => MenuItem[]): Menu {
  let panel: HTMLElement | null = null;

  const focusables = (): HTMLElement[] =>
    panel
      ? Array.from(panel.querySelectorAll<HTMLElement>('[role="menuitem"]')).filter(
          (el) => el.offsetParent !== null,
        )
      : [];

  const close = () => {
    if (!panel) return;
    panel.remove();
    panel = null;
    button.setAttribute("aria-expanded", "false");
    document.removeEventListener("pointerdown", onDocDown, true);
    document.removeEventListener("keydown", onKey, true);
  };

  const onKey = (e: KeyboardEvent) => {
    if (!panel) return;
    const f = focusables();
    const i = f.indexOf(document.activeElement as HTMLElement);
    if (e.key === "Escape") {
      close();
      button.focus();
      e.preventDefault();
    } else if (e.key === "ArrowDown") {
      f[(i + 1) % f.length]?.focus();
      e.preventDefault();
    } else if (e.key === "ArrowUp") {
      f[(i - 1 + f.length) % f.length]?.focus();
      e.preventDefault();
    } else if (e.key === "Home") {
      f[0]?.focus();
      e.preventDefault();
    } else if (e.key === "End") {
      f[f.length - 1]?.focus();
      e.preventDefault();
    }
  };

  const onDocDown = (e: PointerEvent) => {
    if (panel && !panel.contains(e.target as Node) && e.target !== button) close();
  };

  const renderList = (list: MenuItem[], depth: number): HTMLUListElement => {
    const ul = document.createElement("ul");
    ul.className = "menu-list";
    ul.setAttribute("role", "menu");
    for (const item of list) {
      const li = document.createElement("li");
      li.className = "menu-row" + (depth ? " menu-sub" : "");
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "menu-item" + (item.danger ? " menu-danger" : "");
      btn.setAttribute("role", "menuitem");
      btn.textContent = item.label;
      if (item.submenu) {
        btn.setAttribute("aria-haspopup", "true");
        btn.setAttribute("aria-expanded", "false");
        const sub = renderList(item.submenu(), depth + 1);
        sub.hidden = true;
        btn.addEventListener("click", () => {
          const willOpen = sub.hidden;
          sub.hidden = !willOpen;
          btn.setAttribute("aria-expanded", String(willOpen));
        });
        li.append(btn, sub);
      } else {
        btn.addEventListener("click", () => {
          close();
          item.action?.();
        });
        li.append(btn);
        if (item.onDelete) {
          const del = document.createElement("button");
          del.type = "button";
          del.className = "menu-del";
          del.setAttribute("aria-label", `Delete ${item.label}`);
          del.textContent = "✕";
          del.addEventListener("click", (e) => {
            e.stopPropagation();
            item.onDelete!();
            close();
            open(); // rebuild from items() so the list reflects the deletion
          });
          li.append(del);
        }
      }
      ul.append(li);
    }
    return ul;
  };

  const open = () => {
    if (panel) {
      close();
      return;
    }
    panel = document.createElement("div");
    panel.className = "menu-popover";
    panel.append(renderList(items(), 0));
    (button.parentElement ?? document.body).append(panel);
    button.setAttribute("aria-expanded", "true");
    document.addEventListener("pointerdown", onDocDown, true);
    document.addEventListener("keydown", onKey, true);
    focusables()[0]?.focus();
  };

  button.setAttribute("aria-haspopup", "true");
  button.setAttribute("aria-expanded", "false");
  button.addEventListener("click", (e) => {
    e.stopPropagation();
    open();
  });

  return { close };
}
```

- [ ] **Step 4: Add `blankProject` to `project.ts`**

In `tools/wren-web/app/src/project.ts`, add after `defaultProject()` (around line 37):

```ts
/// A fresh, empty single-file project (New → Blank).
export function blankProject(): Project {
  return { files: { "main.wren": "" }, entry: "main.wren", open: ["main.wren"], active: "main.wren", breakpoints: {} };
}
```

- [ ] **Step 5: Replace the example field in `index.html`**

In `tools/wren-web/app/index.html`, replace lines 18-21:

```html
        <label class="field">
          <span class="legend">example</span>
          <select id="examples" aria-label="load example script"></select>
        </label>
```

with:

```html
        <div class="field">
          <button id="project-menu" class="menu-btn" type="button" aria-haspopup="true" aria-expanded="false">Project ▾</button>
        </div>
```

- [ ] **Step 6: Swap the `#examples` wiring for the menu in `main.ts`**

In `tools/wren-web/app/src/main.ts`, delete the examples block (`main.ts:52-70`, from the `// Examples menu loads a whole project.` comment through the `select.addEventListener(...)` call). Add the imports at the top:

```ts
import { createMenu, type MenuItem } from "./menu";
import { blankProject } from "./project";
```

(`projectFromExample` and `EXAMPLES` are already imported.) In its place, after `setupResizers();`, add:

```ts
  // Project ▾ menu. Task 4 wires New (Blank + templates); Task 5 adds
  // Open/Import/Save/Download via setupProjectMenu.
  const guardReplace = () => confirm("Replace the current project? Save or Download it first to keep it.");
  createMenu($<HTMLButtonElement>("#project-menu"), (): MenuItem[] => [
    {
      label: "New",
      submenu: () => [
        { label: "Blank", action: () => { if (guardReplace()) store.replace(blankProject()); } },
        ...EXAMPLES.map((ex) => ({
          label: ex.name,
          action: () => { if (guardReplace()) store.replace(projectFromExample(ex)); },
        })),
      ],
    },
  ]);
```

- [ ] **Step 7: Style the menu in `style.css`**

Append to `tools/wren-web/app/src/style.css` (adjust colors to match nearby topbar tokens — reuse the same variables the `.ghost-btn` / `.field` rules use):

```css
/* Project ▾ menu ---------------------------------------------------------- */
.field { position: relative; }
.menu-btn {
  font: inherit;
  color: inherit;
  background: transparent;
  border: 1px solid color-mix(in srgb, currentColor 22%, transparent);
  border-radius: 6px;
  padding: 4px 10px;
  cursor: pointer;
}
.menu-btn:hover { background: color-mix(in srgb, currentColor 8%, transparent); }
.menu-btn:focus-visible { outline: 2px solid var(--amber, #f0a500); outline-offset: 1px; }
.menu-popover {
  position: absolute;
  top: calc(100% + 6px);
  left: 0;
  z-index: 40;
  min-width: 200px;
  padding: 4px;
  background: var(--panel, #1b1b1f);
  border: 1px solid color-mix(in srgb, currentColor 18%, transparent);
  border-radius: 8px;
  box-shadow: 0 10px 30px rgba(0, 0, 0, 0.45);
}
.menu-list { list-style: none; margin: 0; padding: 0; }
.menu-sub { margin-left: 10px; }
.menu-row { display: flex; align-items: center; }
.menu-item {
  flex: 1;
  text-align: left;
  font: inherit;
  color: inherit;
  background: transparent;
  border: 0;
  border-radius: 5px;
  padding: 6px 10px;
  cursor: pointer;
}
.menu-item:hover, .menu-item:focus-visible {
  background: color-mix(in srgb, var(--amber, #f0a500) 16%, transparent);
  outline: none;
}
.menu-danger { color: #e0685f; }
.menu-del {
  font: inherit;
  color: inherit;
  opacity: 0.5;
  background: transparent;
  border: 0;
  padding: 4px 8px;
  cursor: pointer;
}
.menu-del:hover { opacity: 1; color: #e0685f; }
```

- [ ] **Step 8: Run tests to verify they pass**

Run: `cd tools/wren-web/app && fuser -k 4173/tcp 2>/dev/null; npx tsc --noEmit && npx playwright test project-io -g "Project menu"`
Expected: PASS (both menu cases). Then run the whole file to confirm no regressions: `npx playwright test project-io`.

- [ ] **Step 9: Commit**

```bash
git add tools/wren-web/app/src/menu.ts tools/wren-web/app/src/project.ts tools/wren-web/app/index.html tools/wren-web/app/src/main.ts tools/wren-web/app/src/style.css tools/wren-web/app/tests/project-io.spec.ts
git commit -m "feat(wren-web): Project menu replacing the example dropdown (New + templates)"
```

---

## Task 5: Wire Open / Import / Save / Download (`project-io.ts`)

Replace the Task-4 inline `createMenu` call with a full `setupProjectMenu` that owns all five actions, current-slot tracking, guards, prompts, the folder `<input>`, and the download anchor.

**Files:**
- Create: `tools/wren-web/app/src/project-io.ts`
- Modify: `tools/wren-web/app/src/main.ts` (call `setupProjectMenu`; remove the Task-4 inline menu)
- Test: `tools/wren-web/app/tests/project-io.spec.ts` (add cases)

**Interfaces:**
- Consumes: `createMenu`, `MenuItem` (`menu.ts`); `blankProject`, `projectFromExample`, `ProjectStore`, `type Project` (`project.ts`); `type Example` (`examples.ts`); `listSlots`, `readSlot`, `writeSlot`, `deleteSlot` (`slots.ts`); `projectFromTree`, `type TreeEntry` (`import-tree.ts`); `zipFiles` (`zip.ts`).
- Produces: `setupProjectMenu(opts: { button: HTMLButtonElement; store: ProjectStore; examples: Example[]; setStatus: (msg: string) => void }): void`.

- [ ] **Step 1: Write the failing tests**

Add to `tests/project-io.spec.ts`:

```ts
test("New → Blank is aborted when the confirm is dismissed (guard)", async ({ page }) => {
  page.on("dialog", (d) => d.dismiss()); // reject the guard
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();
  await wren(page, (w) => w.setSource("var keep = 1\n"));
  await page.locator("#project-menu").click();
  await page.getByRole("menuitem", { name: "New" }).click();
  await page.getByRole("menuitem", { name: "Blank" }).click();
  expect(await wren(page, (w) => w.getSource())).toContain("var keep = 1");
});

test("Save As names the current project and Download uses that name", async ({ page }) => {
  page.on("dialog", (d) =>
    d.type() === "prompt" ? d.accept("mytrack") : d.accept(),
  );
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();
  await wren(page, (w) => w.setSource("var t = 1\n"));

  await page.locator("#project-menu").click();
  await page.getByRole("menuitem", { name: "Save As" }).click();
  await expect(page.locator("#project-menu")).toHaveText(/mytrack/);
  expect(await wren(page, (w) => w.slots())).toContain("mytrack");

  const [download] = await Promise.all([
    page.waitForEvent("download"),
    (async () => {
      await page.locator("#project-menu").click();
      await page.getByRole("menuitem", { name: "Download" }).click();
    })(),
  ]);
  expect(download.suggestedFilename()).toBe("mytrack.zip");
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd tools/wren-web/app && fuser -k 4173/tcp 2>/dev/null; npx playwright test project-io -g "Save As names"`
Expected: FAIL — no `Save As` / `Download` menu items yet (Task 4 menu has only New).

- [ ] **Step 3: Write `project-io.ts`**

Create `tools/wren-web/app/src/project-io.ts`:

```ts
// Wires the Project ▾ menu to the project lifecycle: New (blank/template),
// Open (a named slot), Import (a folder off disk), Save / Save As (named
// slots), and Download (a .zip for the SD card). Owns the current-slot name
// (shown in the button label), the destructive-action guard, the hidden folder
// <input>, and the download anchor.
import { createMenu, type MenuItem } from "./menu";
import { ProjectStore, blankProject, projectFromExample } from "./project";
import { type Example } from "./examples";
import { listSlots, readSlot, writeSlot, deleteSlot } from "./slots";
import { projectFromTree, type TreeEntry } from "./import-tree";
import { zipFiles } from "./zip";

export interface ProjectMenuOpts {
  button: HTMLButtonElement;
  store: ProjectStore;
  examples: Example[];
  setStatus: (msg: string) => void;
}

export function setupProjectMenu(opts: ProjectMenuOpts): void {
  const { button, store, examples, setStatus } = opts;
  let currentSlot: string | null = null;

  const label = () => {
    button.textContent = currentSlot ? `Project · ${currentSlot} ▾` : "Project ▾";
  };
  label();

  const guard = () =>
    confirm("Replace the current project? Save or Download it first to keep it.");

  const newBlank = () => {
    if (!guard()) return;
    store.replace(blankProject());
    currentSlot = null;
    label();
  };
  const newExample = (ex: Example) => {
    if (!guard()) return;
    store.replace(projectFromExample(ex));
    currentSlot = null;
    label();
  };
  const openSlot = (name: string) => {
    const p = readSlot(name);
    if (!p || !guard()) return;
    store.replace(p);
    currentSlot = name;
    label();
  };
  const saveAs = () => {
    const name = prompt("Save project as:", currentSlot ?? "")?.trim();
    if (!name) return;
    if (listSlots().includes(name) && !confirm(`Overwrite "${name}"?`)) return;
    if (writeSlot(name, store.project)) {
      currentSlot = name;
      label();
      setStatus(`saved "${name}"`);
    } else {
      setStatus("save failed (storage full?)");
    }
  };
  const save = () => {
    if (!currentSlot) return saveAs();
    setStatus(writeSlot(currentSlot, store.project) ? `saved "${currentSlot}"` : "save failed (storage full?)");
  };
  const download = () => {
    const blob = new Blob([zipFiles(store.project.files)], { type: "application/zip" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${currentSlot ?? "wren-project"}.zip`;
    document.body.append(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  };

  // Hidden folder picker for Import.
  const picker = document.createElement("input");
  picker.type = "file";
  picker.id = "import-picker";
  picker.multiple = true;
  picker.hidden = true;
  // webkitdirectory is non-standard; set via attribute so TS doesn't complain.
  picker.setAttribute("webkitdirectory", "");
  picker.addEventListener("change", async () => {
    const list = picker.files;
    if (!list || list.length === 0) return;
    const entries: TreeEntry[] = [];
    for (const f of Array.from(list)) {
      const path = (f as File & { webkitRelativePath?: string }).webkitRelativePath || f.name;
      entries.push({ path, content: await f.text() });
    }
    picker.value = ""; // allow re-picking the same folder
    const project = projectFromTree(entries);
    if (!project) {
      setStatus("no .wren files found");
      return;
    }
    if (!guard()) return;
    store.replace(project);
    currentSlot = null;
    label();
    setStatus("imported from disk");
  });
  document.body.append(picker);

  const items = (): MenuItem[] => [
    {
      label: "New",
      submenu: () => [
        { label: "Blank", action: newBlank },
        ...examples.map((ex) => ({ label: ex.name, action: () => newExample(ex) })),
      ],
    },
    {
      label: "Open",
      submenu: () => {
        const names = listSlots();
        if (names.length === 0) return [{ label: "(no saved projects)" }];
        return names.map((n) => ({
          label: n,
          action: () => openSlot(n),
          onDelete: () => deleteSlot(n),
        }));
      },
    },
    { label: "Import from disk…", action: () => picker.click() },
    { label: "Save", action: save },
    { label: "Save As…", action: saveAs },
    { label: "Download .zip", action: download },
  ];

  createMenu(button, items);
}
```

- [ ] **Step 4: Replace the Task-4 inline menu in `main.ts`**

In `tools/wren-web/app/src/main.ts`, remove the Task-4 block (the `guardReplace` const and the `createMenu($<HTMLButtonElement>("#project-menu"), …)` call added in Task 4 Step 6) and the now-unused `createMenu` / `MenuItem` imports. Add the import:

```ts
import { setupProjectMenu } from "./project-io";
```

In place of the removed block (after `setupResizers();`), add:

```ts
  setupProjectMenu({
    button: $<HTMLButtonElement>("#project-menu"),
    store,
    examples: EXAMPLES,
    setStatus: (m) => { status.textContent = m; },
  });
```

Keep the `blankProject` import only if still referenced elsewhere; otherwise remove it (it's now used inside `project-io.ts`). Run `npx tsc --noEmit` to catch any unused-import error and delete accordingly.

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd tools/wren-web/app && fuser -k 4173/tcp 2>/dev/null; npx tsc --noEmit && npx playwright test project-io`
Expected: PASS (all project-io cases, including guard + Save As + Download).

- [ ] **Step 6: Run the full suite for regressions**

Run: `cd tools/wren-web/app && fuser -k 4173/tcp 2>/dev/null; npx playwright test`
Expected: All prior specs still pass (the examples dropdown is gone; confirm no spec references `#examples` — if one does, update it to drive the Project menu).

- [ ] **Step 7: Commit**

```bash
git add tools/wren-web/app/src/project-io.ts tools/wren-web/app/src/main.ts tools/wren-web/app/tests/project-io.spec.ts
git commit -m "feat(wren-web): wire Open/Import/Save/Download into the Project menu"
```

---

## Task 6: Verify examples-dropdown removal + manual smoke

**Files:**
- Possibly modify: any existing spec referencing `#examples`.
- Test: manual browser check.

- [ ] **Step 1: Grep for stale references**

Run: `cd tools/wren-web/app && grep -rn "#examples\|examples\b" tests/ src/main.ts`
Expected: `src/examples.ts` (`EXAMPLES` source, fine) and the Project menu usage only. Any test still selecting `#examples` must be repointed at the Project menu (New → template). Fix inline if found; otherwise no change.

- [ ] **Step 2: Manual smoke (dev server)**

Run `setsid npm run dev` from `tools/wren-web/app`, open the app, and confirm:
- Project ▾ opens; New → Blank clears (after the confirm); New → a template loads.
- Save As names it (button shows `Project · <name> ▾`); reload; Open → the slot restores.
- Import from disk opens the OS folder picker; picking a folder of `.wren` files loads them.
- Download saves `<name>.zip` (or `wren-project.zip`); unzip it and confirm the tree matches.

- [ ] **Step 3: Commit any fixes**

```bash
git add -A && git commit -m "test(wren-web): repoint example-dropdown references at the Project menu"
```

(Skip if Step 1 found nothing to change.)

---

## Self-Review

**Spec coverage:**
- Project menu (New/Open/Import/Save/Download) → Tasks 4 (New) + 5 (rest). ✓
- Fold examples into New → Task 4 Step 6. ✓
- Named browser slots under `wren-deluge:saved` → Task 2. ✓
- Folder-picker Import, strip top folder, prefer main.wren → Task 3 + Task 5 picker. ✓
- Dependency-free store-only zip → Task 1. ✓
- Guards on New/Open/Import → Task 4 (New), Task 5 (Open/Import) + guard test. ✓
- Download filename `wren-project.zip` / `<slot>.zip` → Task 5 `download()` + test. ✓
- Firmware imports + File System Access = non-goals → not implemented (correct). ✓
- Testing scenarios (New, template, round-trip, delete, download entries, import, guard) → Tasks 1-5 cover each. Note: Import is tested via the `importTree` hook (Task 3) rather than driving `webkitRelativePath` through Playwright, which `setInputFiles` cannot populate; the DOM picker glue is thin and smoke-checked in Task 6.

**Placeholder scan:** No TBD/TODO; every code step is complete. Menu color tokens in Task 4 Step 7 use `color-mix`/fallbacks and instruct matching nearby tokens — concrete, not a placeholder.

**Type consistency:** `zipFiles(files): Uint8Array`, `listSlots/readSlot/writeSlot/deleteSlot`, `projectFromTree(entries): Project | null`, `TreeEntry`, `createMenu(button, items)`, `MenuItem`, `blankProject()`, `setupProjectMenu(opts)` — names/signatures match across the tasks that define and consume them. `window.wren` hooks (`zipBytes`, `saveSlot`, `openSlot`, `slots`, `deleteSlot`, `importTree`) are consistent between their defining task and the tests.
