import { test, expect } from "@playwright/test";

// The editor gains a project lifecycle (New / Open / Import / Save / Download)
// under one Project ▾ menu. Pure pieces are exercised via window.wren hooks;
// the menu, folder import, and zip download are driven through the DOM.

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
  importTree: (entries: { path: string; content: string }[]) => boolean;
  entry: () => string;
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
