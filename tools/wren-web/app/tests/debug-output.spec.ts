import { test, expect, type Page } from "@playwright/test";

// Deferred fix: the debug core used to swallow `System.print` (a no-op write_fn)
// so the console stayed empty during a debug session. Output now streams through
// the hook's OutputWriter → Output events → the DebugController's on("output") →
// the console. This proves a print inside a debug run reaches the console live.
const PROGRAM = 'var x = 21\nSystem.print("hi from debug %(x * 2)")\n';

async function clickGutter(page: Page, line: number) {
  const overlays = page.locator(".margin-view-overlays").first();
  await overlays.click({ position: { x: 8, y: 14 + (line - 1) * 20 + 10 }, force: true });
}

test("System.print streams to the console during a debug run", async ({ page }) => {
  const pageErrors: string[] = [];
  page.on("pageerror", (e) => pageErrors.push(String(e)));

  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  await page.evaluate((src) => {
    (window as unknown as { wren: { setSource: (s: string) => void } }).wren.setSource(src);
  }, PROGRAM);
  await expect(page.locator(".view-lines")).toContainText("System.print");

  // Break on line 2 (the print), start, pause, then continue so the print runs.
  await clickGutter(page, 2);
  await page.locator("#debug").click();
  await expect(page.locator(".dbg-current-line")).toHaveCount(1, { timeout: 30_000 });
  await page.locator("#dbg-continue").click();

  // The print output now appears in the console (as a stdout line).
  await expect(page.locator("#console")).toContainText("hi from debug 42", { timeout: 30_000 });
  // …and the run finishes.
  await expect(page.locator("#debug")).toBeVisible({ timeout: 30_000 });

  expect(pageErrors, `unexpected page errors: ${pageErrors.join("\n")}`).toEqual([]);
});
