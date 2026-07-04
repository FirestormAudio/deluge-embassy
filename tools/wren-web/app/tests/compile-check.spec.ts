import { test, expect } from "@playwright/test";

// A background worker compiles+runs the whole project through the REAL wren
// compiler on a debounce, surfacing errors the static analyzer can't (undefined
// module vars, arity, top-level runtime) as `wren-check` markers — live, no Run.
// It works in conjunction with the analyzer: when the analyzer already flags a
// file, the check defers so there's never a double squiggle.

type Wren = {
  createFile: (p: string, c?: string) => void;
  activate: (p: string) => void;
  setEntry: (p: string) => void;
  setSource: (s: string) => void;
  markersOf: (p: string) => string[];
  checkMarkers: (p?: string) => string[];
};
const wren = (page: import("@playwright/test").Page, fn: (w: Wren) => unknown) =>
  page.evaluate(`(${fn})((window).wren)`);
const poll = (page: import("@playwright/test").Page, fn: (w: Wren) => unknown) =>
  expect.poll(async () => (await wren(page, fn)) as string[], { timeout: 12000 });

test("the background check surfaces a compiler error the analyzer misses, then clears", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  // Undefined module var: analyzer is lenient (it could be a prelude/import), but
  // the real compiler errors — the check must flag it, no Run.
  await wren(page, (w) => {
    w.activate("main.wren");
    w.setEntry("main.wren");
    w.setSource("var x = Foo.bar\n");
  });
  await poll(page, (w) => w.checkMarkers()).toContain("Error at 'Foo': Variable is used but not defined.");

  // Fix it → the check marker clears.
  await wren(page, (w) => w.setSource("var x = 5\n"));
  await poll(page, (w) => w.checkMarkers()).toEqual([]);
});

test("the check defers to the analyzer (no double squiggle) when the analyzer already flags an error", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  await wren(page, (w) => {
    w.createFile("lib/voice.wren", "class Synth {}\n"); // does NOT export Voice
    w.activate("lib/voice.wren");
    w.setSource("class Synth {}\n");
  });
  await page.waitForTimeout(300);
  await wren(page, (w) => {
    w.activate("main.wren");
    w.setEntry("main.wren");
    w.setSource('import "lib/voice" for Voice\nvar v = Voice\n');
  });

  // The analyzer flags the import…
  await poll(page, (w) => w.markersOf("main.wren")).toContain(
    "Could not find a variable named 'Voice' in module 'lib/voice'.",
  );
  // …so the check stays quiet (defers), and there's a single squiggle on the line.
  await poll(page, (w) => w.checkMarkers()).toEqual([]);
  await expect(page.locator(".squiggly-error")).toHaveCount(1);
});

test("a compiler error inside an imported module lands on that module's file, not the entry", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  // lib/mod line 2 references an undefined var — a compile error INSIDE lib/mod.
  // main imports it (validly). The check must attribute the error to lib/mod's
  // own model (not the entry), despite the import cascade.
  await wren(page, (w) => {
    w.createFile("lib/mod.wren", "class Thing {}\n");
    w.activate("lib/mod.wren");
    w.setSource("class Thing {}\nvar y = Undefined\n");
  });
  await page.waitForTimeout(300);
  await wren(page, (w) => {
    w.activate("main.wren");
    w.setEntry("main.wren");
    w.setSource('import "lib/mod" for Thing\nvar t = Thing\n');
  });

  await poll(page, (w) => w.checkMarkers("lib/mod.wren")).toContain(
    "Error at 'Undefined': Variable is used but not defined.",
  );
  // …and NOT mis-attributed to the entry.
  await poll(page, (w) => w.checkMarkers("main.wren")).toEqual([]);
});
