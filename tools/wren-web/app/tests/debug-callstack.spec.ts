import { test, expect, type Page } from "@playwright/test";

// Task 4.3: call-stack panel in the left sidebar (VSCode-style). While debugging,
// the left pane switches from the file tree to a debug view whose CALL STACK
// section lists the current frames; the pane auto-switches to debug on a stop.
//
// Fixture: an entry that defines a Fn and calls it, with a breakpoint INSIDE the
// Fn body (line 2). The debug core then reports two frames — the top `<block>`
// frame at the breakpoint line and the `main` module frame at the call site
// (verified empirically against the real threaded debug wasm).
const PROGRAM = "var add = Fn.new { |a, b|\n  var sum = a + b\n  return sum\n}\nSystem.print(add.call(2, 3))\n";

// Click the glyph margin (leftmost gutter) at a 1-based line (line height 20,
// top padding 14 → center = 14 + (L-1)*20 + 10).
async function clickGutter(page: Page, line: number) {
  const overlays = page.locator(".margin-view-overlays").first();
  await overlays.click({ position: { x: 8, y: 14 + (line - 1) * 20 + 10 }, force: true });
}

test("left pane auto-switches to debug and lists the call stack at a breakpoint", async ({ page }) => {
  const pageErrors: string[] = [];
  page.on("pageerror", (e) => pageErrors.push(String(e)));

  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  // Load the fixture into the entry file and wait for the model to settle.
  await page.evaluate((src) => {
    (window as unknown as { wren: { setSource: (s: string) => void } }).wren.setSource(src);
  }, PROGRAM);
  await expect(page.locator(".view-lines")).toContainText("var sum = a + b");

  // Before a session: the files tree is shown, the debug view is hidden.
  await expect(page.locator("#fb-tree")).toBeVisible();
  await expect(page.locator("#debug-view")).toBeHidden();

  // Breakpoint inside the Fn body (line 2) and start a debug run.
  await clickGutter(page, 2);
  await expect(page.locator(".dbg-breakpoint")).toHaveCount(1);
  await page.locator("#debug").click();

  // Paused: the current-line highlight shows and the left pane auto-switched to
  // the debug view (files tree hidden).
  await expect(page.locator(".dbg-current-line")).toHaveCount(1, { timeout: 30_000 });
  await expect(page.locator("#debug-view")).toBeVisible();
  await expect(page.locator("#fb-tree")).toBeHidden();
  await expect(page.locator("#pane-debug")).toHaveClass(/is-active/);

  // CALL STACK lists ≥1 frame; the top frame is selected and sits at the bp line.
  const frames = page.locator("#dbg-callstack .dbg-frame");
  await expect(frames.first()).toBeVisible();
  expect(await frames.count()).toBeGreaterThanOrEqual(1);
  const top = frames.first();
  await expect(top).toHaveClass(/selected/);
  await expect(top).toContainText(":2");
  // The session recorded the selected frame (top frame, id 0).
  expect(await page.evaluate(() => (window as unknown as { wren: { selectedFrame: () => number | null } }).wren.selectedFrame())).toBe(0);

  // The files segment shows the tree again; the debug segment shows the stack.
  await page.locator("#pane-files").click();
  await expect(page.locator("#fb-tree")).toBeVisible();
  await expect(page.locator("#debug-view")).toBeHidden();
  await page.locator("#pane-debug").click();
  await expect(page.locator("#debug-view")).toBeVisible();
  await expect(frames.first()).toBeVisible();

  expect(pageErrors, `unexpected page errors: ${pageErrors.join("\n")}`).toEqual([]);
});
