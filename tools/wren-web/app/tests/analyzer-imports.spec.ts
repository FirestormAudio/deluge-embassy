import { test, expect } from "@playwright/test";

// The live analyzer resolves cross-file imports against the other project files
// (module name → source), so `import "lib/x" for Y` errors appear and clear as
// you type — no Run needed. Previously import errors only surfaced on Run (a
// stale wren-vm marker), which is why they "didn't update."

type Wren = {
  createFile: (p: string, c?: string) => void;
  activate: (p: string) => void;
  setEntry: (p: string) => void;
  setSource: (s: string) => void;
  markers: () => { message: string; startLineNumber: number }[];
};
const wren = (page: import("@playwright/test").Page, fn: (w: Wren) => unknown) =>
  page.evaluate(`(${fn})((window).wren)`);

test("cross-file import errors appear live and clear when the module is fixed", async ({ page }) => {
  const pageErrors: string[] = [];
  page.on("pageerror", (e) => pageErrors.push(String(e)));

  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  // A lib that does NOT export `Synth`, and an entry that imports `Synth` from it.
  await wren(page, (w) => {
    w.createFile("lib/mod.wren", "class Other {}\n");
    w.activate("lib/mod.wren");
    w.setSource("class Other {}\n");
  });
  await page.waitForTimeout(300);
  await wren(page, (w) => {
    w.activate("main.wren");
    w.setEntry("main.wren");
    w.setSource('import "lib/mod" for Synth\nvar s = Synth\n');
  });

  // The import error shows up LIVE (no Run) as a wren-analyzer marker.
  await expect
    .poll(async () => (await wren(page, (w) => w.markers().map((m) => m.message))) as string[], { timeout: 8000 })
    .toContain("Could not find a variable named 'Synth' in module 'lib/mod'.");

  // Fix the module to export Synth; switching back re-validates and clears it.
  await wren(page, (w) => {
    w.activate("lib/mod.wren");
    w.setSource("class Synth {}\n");
  });
  await page.waitForTimeout(300);
  await wren(page, (w) => w.activate("main.wren"));

  await expect
    .poll(async () => (await wren(page, (w) => w.markers().map((m) => m.message))) as string[], { timeout: 8000 })
    .not.toContain("Could not find a variable named 'Synth' in module 'lib/mod'.");

  expect(pageErrors, `unexpected page errors: ${pageErrors.join("\n")}`).toEqual([]);
});
