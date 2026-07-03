import { test, expect } from "@playwright/test";

// Task 4.1: breakpoint gutter + persistence. Clicking Monaco's glyph margin
// toggles an amber breakpoint on that line; breakpoints live in the ProjectStore
// (source of truth for 4.2's launch) and persist to localStorage across reloads.
//
// The default project is example[0] "drone" (main.wren, 6 lines); line 3 is
// `var voice = Osc.saw(110)`.

// Click the glyph margin (leftmost gutter strip) at a 1-based line. Line height
// is 20px, top padding 14px, so line L's vertical center is 14 + (L-1)*20 + 10.
async function clickGutter(page: import("@playwright/test").Page, line: number) {
  const overlays = page.locator(".margin-view-overlays").first();
  // force: the rendered breakpoint glyph widget overlays the margin and would
  // otherwise fail the actionability (pointer-intercept) check; Monaco still
  // resolves the coordinate to the glyph margin and fires the toggle.
  await overlays.click({ position: { x: 8, y: 14 + (line - 1) * 20 + 10 }, force: true });
}

const bpFor = (page: import("@playwright/test").Page) =>
  page.evaluate(() => {
    const w = window as unknown as { wren: { active: () => string; breakpoints: (p: string) => number[] } };
    return w.wren.breakpoints(w.wren.active());
  });

test("glyph-margin click toggles an amber breakpoint, persists, and toggles off", async ({ page }) => {
  const consoleErrors: string[] = [];
  page.on("pageerror", (e) => consoleErrors.push(String(e)));

  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  // Set a breakpoint on line 3.
  await clickGutter(page, 3);
  await expect(page.locator(".dbg-breakpoint")).toHaveCount(1);
  expect(await bpFor(page)).toContain(3);

  // Wait for the debounced save to flush to localStorage, then reload.
  await page.waitForFunction(() => {
    const raw = localStorage.getItem("wren-deluge:project");
    return !!raw && (JSON.parse(raw).breakpoints?.["main.wren"] ?? []).includes(3);
  });
  await page.reload();
  await page.locator(".monaco-editor").first().waitFor();
  await expect(page.locator(".dbg-breakpoint")).toHaveCount(1);
  expect(await bpFor(page)).toContain(3);

  // Toggle it off.
  await clickGutter(page, 3);
  await expect(page.locator(".dbg-breakpoint")).toHaveCount(0);
  expect(await bpFor(page)).not.toContain(3);

  expect(consoleErrors, `unexpected page errors: ${consoleErrors.join("\n")}`).toEqual([]);
});
