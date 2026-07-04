# Wren Web — Project I/O (New / Open / Import / Save / Download)

**Date:** 2026-07-04
**Component:** `tools/wren-web/app`
**Status:** Design approved, ready for implementation plan

## Summary

The Wren web editor can currently only hold one project at a time: a live
working copy autosaved to `localStorage`, seedable from a fixed `example`
dropdown, and shareable via a `#p=` permalink. There is no way to keep several
projects, to pull an existing project off an SD card for editing, or to export
one back to the card.

This adds a project lifecycle: **New**, **Open** (from named browser-storage
slots), **Import** (a loose `.wren` file tree from disk), **Save** (to a named
slot), and **Download** (a `.zip` of the source tree, for the SD card). The
existing `example` dropdown is retired — its templates move under **New**. All
five actions live in a single **Project ▾** menu in the topbar.

## Goals

- Keep multiple projects locally and switch between them, entirely in-browser.
- Import an on-card project (a folder of `.wren` files) for editing.
- Export the current project as a `.zip` mirroring its file tree, intended to be
  dropped onto the SD card.
- Fold the current examples into a "New from template" affordance.
- Add zero new npm dependencies.

## Non-goals (explicit)

- **Firmware cross-file imports.** The `.zip` mirrors the project's file tree
  (`main.wren`, `lib/voice.wren`, nested folders). The device firmware today
  runs a single `/MAIN.WREN` at the card root and returns NULL from
  `wren_host_load_module` (`wren-firmware/src/main.rs:338`) — it does **not**
  resolve cross-file imports, so a multi-file zip will not boot on current
  hardware. This design deliberately builds the export against the firmware *as
  it will be*. **Follow-up (not in this work):** implement
  `wren_host_load_module` against the SD card and boot nested entries, so
  downloaded projects run on-device. Recorded here so the gap is on the books.
- **Edit-in-place / write-back to the card.** Import reads *in* only. Writing
  edits straight back to the card via the File System Access API
  (`showDirectoryPicker`, live handles) is a plausible future enhancement but is
  Chromium-only, so it is out of scope. Round-trip for now is: Import → edit →
  Download → copy to card.
- **Zip re-import.** Import uses a folder picker (the card holds loose files, not
  a zip). Re-opening a downloaded `.zip` is not supported; easy to add later if
  wanted.
- **Cross-machine transport.** Already covered by the existing **Share** (`#p=`)
  permalink. Unchanged.

## Current state (what exists to build on)

- `Project` (`src/project.ts:8`) = `{ files, entry, open, active, breakpoints }`.
- `ProjectStore` (`src/project.ts:129`) autosaves the working copy to
  `localStorage` key `wren-deluge:project`; `store.replace(project)` swaps the
  whole project (already used by the examples menu).
- `normalize(partial)` (`src/project.ts:70`) repairs any partial/hand-authored
  Project — fills entry, prunes dangling breakpoints/tabs. Import and Open reuse
  it, so malformed input can't corrupt state.
- `EXAMPLES` (`src/examples.ts`) — the template list, currently wired to the
  `#examples` select in `src/main.ts` (~line 52).
- `b64encode/decode` + `permalink()` — Share path, untouched by this work.

## Design

### Storage model

Two `localStorage` keys, kept separate:

- `wren-deluge:project` (existing) — the single live working copy. Untouched;
  autosave continues exactly as today.
- `wren-deluge:saved` (**new**) — named slots: `{ [name: string]: Project }`.
  Each slot stores the **full** workspace (`files, entry, open, active,
  breakpoints`) so Open restores tabs and breakpoints faithfully.

A small `slots.ts` module owns this key: `listSlots(): string[]`,
`readSlot(name): Project | null` (via `normalize`), `writeSlot(name, project)`,
`deleteSlot(name)`. All wrapped in try/catch — storage may be unavailable
(private mode); failures degrade gracefully, matching existing `saveLocal`.

### Controls — the Project ▾ menu

Replace the `example` `<label class="field">` in `index.html` with a single
menu control. The `face` select, `status`, `Share`, and `Run` are unchanged.

```
Project ▾
├─ New            ▸  Blank · <EXAMPLES…>
├─ Open           ▸  <saved slots…>   (each row has an inline delete)
├─ Import from disk…      (folder picker)
├─ Save                    (re-saves the current slot; prompts if unnamed)
├─ Save As…                (prompts for a new name)
└─ Download .zip
```

A lightweight popover menu (button + `role="menu"`, arrow-key navigable,
Escape/outside-click to close). Submenus for New/Open expand inline (a nested
`role="menu"`), so the whole thing is keyboard- and screen-reader-navigable. No
menu library — a focused ~150-line `menu.ts` styled to match the topbar. The
"current slot name" (if any) shows in the button label, e.g. `Project · lead-synth ▾`.

### Actions

- **New → Blank** — `store.replace(blankProject())` where `blankProject()` is one
  empty `main.wren`. Guarded (see below).
- **New → <template>** — `store.replace(projectFromExample(ex))` (existing
  helper). Guarded.
- **Open → <slot>** — `store.replace(readSlot(name))`, set current-slot = name.
  Guarded. Inline delete on a row calls `deleteSlot` and refreshes the menu.
- **Import from disk…** — a hidden `<input type="file" webkitdirectory>`; on
  change: keep entries ending in `.wren`, strip the picked top-level directory
  from each `webkitRelativePath` (`SDCARD/lib/voice.wren` → `lib/voice.wren`),
  read each as text, build `{ files }`, choose entry preferring `main.wren` then
  `MAIN.WREN` (case-insensitive) else `normalize`'s heuristic, then
  `store.replace(normalize(...))`. Current-slot cleared (imported, not a slot).
  Guarded. Empty/no-`.wren` selection → status message, no state change.
- **Save** — if a current slot is set, `writeSlot(currentSlot, store.project)`.
  If not, behave as **Save As…**.
- **Save As…** — `prompt()` for a name; blank cancels; if the name exists,
  `confirm()` to overwrite; then `writeSlot`, set current-slot = name.
- **Download .zip** — `zipProject(store.project.files)` → Blob →
  object-URL `<a download>` → click → revoke. Filename `wren-project.zip`
  (or `<currentSlot>.zip` when a slot is active). Only `files` go in — no editor
  metadata. Folders are implied by the entry paths (a store-zip needs no explicit
  directory entries).

### Zip writer (`zip.ts`, dependency-free)

A minimal **store-only** (compression method 0, no deflate) ZIP writer, ~50
lines: per file emit a local-file header + raw bytes; accumulate a central
directory; end with the end-of-central-directory record. CRC-32 via a small
table. UTF-8 paths, forward slashes. This matches the project's self-hosted /
COEP ethos (fonts are vendored for the same reason) and avoids pulling in
jszip/fflate. Reader side is not needed (no zip import).

### Guards

New, Open, and Import each *replace* the working copy with no undo. Each calls
`confirm("Replace the current project? Save or Download it first to keep it.")`
before proceeding; cancel aborts with no change. Save/Save As/Download never
guard. (Note: the working copy is always in `wren-deluge:project` autosave, but
replacing it overwrites that too — hence the guard.)

## Files

**New**
- `src/slots.ts` — named-slot storage over `wren-deluge:saved`.
- `src/zip.ts` — store-only zip writer + CRC-32.
- `src/menu.ts` — accessible popover menu component.
- `src/project-io.ts` — wires actions to store/slots/zip/import; owns guards,
  current-slot state, and the `<input webkitdirectory>` import handler.
- `tests/project-io.spec.ts` — Playwright coverage (below).

**Modified**
- `index.html` — replace the `example` field with the Project menu mount point.
- `src/main.ts` — drop the `#examples` wiring; mount the Project menu; expose
  the store/slots to `project-io`.
- `src/style.css` — menu + submenu styling matched to the topbar.

**Untouched** — `persistence.ts`, `examples.ts` (still the template source),
Share, firmware.

## Testing (Playwright, `tests/project-io.spec.ts`)

1. **New → Blank** clears to a single empty `main.wren`.
2. **New → template** loads that example's files/entry.
3. **Save → Open round-trip** — edit, Save As "t1", New → Blank, Open "t1"
   restores files **and** breakpoints and open tabs.
4. **Delete slot** — a deleted slot disappears from the Open submenu.
5. **Download** — trigger Download, capture the `download` event, and assert the
   zip's entries (paths) match the project's file tree. (Decode the store-zip in
   the test, or assert on the central-directory filenames.)
6. **Import** — drive the folder `<input>` via Playwright `setInputFiles` with a
   crafted tree (`proj/main.wren`, `proj/lib/voice.wren`, plus a non-`.wren` file
   that must be ignored); assert the project loads with stripped paths and
   `main.wren` as entry.
7. **Guard** — with `confirm` stubbed to reject, New/Open/Import leave the
   project unchanged.

All run from `tools/wren-web/app`. `npx tsc --noEmit` clean.

## Risks / notes

- `webkitdirectory` is non-standard but supported across current Chromium,
  Firefox, and Safari; acceptable for a dev tool. No fallback planned.
- `localStorage` ~5 MB cap: text projects are tiny; many slots fit. If a write
  throws (quota/private mode), surface a status message and no-op — never throw
  into the UI.
- The store-zip is uncompressed, so a downloaded project is roughly the sum of
  its source sizes — negligible for `.wren` text.
