import { test, expect, type Page } from "@playwright/test";

// Task 4.4: variables tree in the left-sidebar debug view (VSCode-style). At a
// stop the VARIABLES section shows the selected frame's named locals (name =
// value); expanding a structured value (a List) lazily fetches its indexed
// children; switching the selected frame re-renders VARIABLES for that frame.
//
// Fixture: a Fn with a param `x` and a body that binds a List `nums`, called
// from the module top level. A breakpoint inside the Fn body (line 3) stops with
// two frames — the top `<block>` frame (locals x=42, nums=[List]) and the `main`
// call-site frame (empty locals). Verified empirically against the real threaded
// debug wasm (tools/wren-web-debug/scopes-probe.mjs).
const PROGRAM = "var describe = Fn.new { |x|\n  var nums = [10, 20, 30]\n  return nums\n}\nSystem.print(describe.call(42))\n";

// Click the glyph margin (leftmost gutter) at a 1-based line (line height 20,
// top padding 14 → center = 14 + (L-1)*20 + 10).
async function clickGutter(page: Page, line: number) {
  const overlays = page.locator(".margin-view-overlays").first();
  await overlays.click({ position: { x: 8, y: 14 + (line - 1) * 20 + 10 }, force: true });
}

test("VARIABLES shows named locals, lazily expands a list, and re-renders on frame switch", async ({ page }) => {
  const pageErrors: string[] = [];
  page.on("pageerror", (e) => pageErrors.push(String(e)));

  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  await page.evaluate((src) => {
    (window as unknown as { wren: { setSource: (s: string) => void } }).wren.setSource(src);
  }, PROGRAM);
  await expect(page.locator(".view-lines")).toContainText("var nums = [10, 20, 30]");

  // Breakpoint inside the Fn body (line 3) and start a debug run.
  await clickGutter(page, 3);
  await expect(page.locator(".dbg-breakpoint")).toHaveCount(1);
  await page.locator("#debug").click();

  // Paused: the debug view is up and the top frame is selected.
  await expect(page.locator(".dbg-current-line")).toHaveCount(1, { timeout: 30_000 });
  await expect(page.locator("#debug-view")).toBeVisible();
  const frames = page.locator("#dbg-callstack .dbg-frame");
  await expect(frames.first()).toHaveClass(/selected/);

  const vars = page.locator("#dbg-variables");

  // The scalar local renders as `x = 42`.
  const xRow = vars.locator('.dbg-var[data-name="x"]');
  await expect(xRow).toBeVisible();
  await expect(xRow.locator(".dbg-var-name")).toHaveText("x");
  await expect(xRow.locator(".dbg-var-val")).toHaveText("42");

  // The list local renders with a twisty and is NOT yet expanded (lazy).
  const numsRow = vars.locator('.dbg-var[data-name="nums"]');
  await expect(numsRow).toBeVisible();
  await expect(numsRow.locator("> .dbg-var-row .fb-twisty")).toBeVisible();
  await expect(vars).not.toContainText("[0]");

  // Expand the list → its indexed children appear (fetched lazily on click).
  await numsRow.locator("> .dbg-var-row").click();
  await expect(numsRow.locator('.dbg-var[data-name="[0]"] .dbg-var-val')).toHaveText("10");
  await expect(numsRow.locator('.dbg-var[data-name="[1]"] .dbg-var-val')).toHaveText("20");
  await expect(numsRow.locator('.dbg-var[data-name="[2]"] .dbg-var-val')).toHaveText("30");
  await expect(numsRow.locator("> .dbg-var-row")).toHaveAttribute("aria-expanded", "true");

  // Switch to the caller frame (`main`, empty locals) → VARIABLES re-renders and
  // the Fn-body locals are gone.
  await frames.nth(1).click();
  await expect(page.evaluate(() => (window as unknown as { wren: { selectedFrame: () => number | null } }).wren.selectedFrame())).resolves.toBe(1);
  await expect(vars.locator('.dbg-var[data-name="nums"]')).toHaveCount(0);
  await expect(vars.locator('.dbg-var[data-name="x"]')).toHaveCount(0);

  // Switch back to the top frame → the locals return.
  await frames.first().click();
  await expect(vars.locator('.dbg-var[data-name="x"]')).toBeVisible();

  expect(pageErrors, `unexpected page errors: ${pageErrors.join("\n")}`).toEqual([]);
});
