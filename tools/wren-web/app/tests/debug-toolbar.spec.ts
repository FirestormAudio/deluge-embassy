import { test, expect, type Page } from "@playwright/test";

// Task 4.2: debug transport toolbar + paused-line highlight + isolation/compile
// gating. Drives the real threaded debugger via the DebugController; the default
// project entry is the "drone" example (main.wren), whose line 2 is
// `var lfo = Osc.sine(5)`.

// Click the glyph margin (leftmost gutter) at a 1-based line to toggle a
// breakpoint (line height 20, top padding 14 → center = 14 + (L-1)*20 + 10).
async function clickGutter(page: Page, line: number) {
  const overlays = page.locator(".margin-view-overlays").first();
  await overlays.click({ position: { x: 8, y: 14 + (line - 1) * 20 + 10 }, force: true });
}

test("Debug pauses at a breakpoint, enables transport, and Continue returns to idle", async ({ page }) => {
  const pageErrors: string[] = [];
  page.on("pageerror", (e) => pageErrors.push(String(e)));

  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  // Set a breakpoint on line 2 and start a debug run.
  await clickGutter(page, 2);
  await expect(page.locator(".dbg-breakpoint")).toHaveCount(1);
  await page.locator("#debug").click();

  // Paused: current-line highlight shows, the Debug trigger hides, and the
  // transport cluster is enabled (Continue operable).
  await expect(page.locator(".dbg-current-line")).toHaveCount(1, { timeout: 30_000 });
  await expect(page.locator("#debug")).toBeHidden();
  await expect(page.locator("#dbg-continue")).toBeEnabled();
  await expect(page.locator("#dbg-stop")).toBeEnabled();

  // Continue → runs to completion → highlight clears, Debug trigger returns.
  await page.locator("#dbg-continue").click();
  await expect(page.locator(".dbg-current-line")).toHaveCount(0, { timeout: 30_000 });
  await expect(page.locator("#debug")).toBeVisible();

  expect(pageErrors, `unexpected page errors: ${pageErrors.join("\n")}`).toEqual([]);
});

test("Debug is disabled without cross-origin isolation", async ({ page }) => {
  await page.addInitScript(() => {
    (window as unknown as { __wrenNoIsolation: boolean }).__wrenNoIsolation = true;
  });
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  const debugBtn = page.locator("#debug");
  await expect(debugBtn).toBeDisabled();
  await expect(debugBtn).toHaveAttribute("title", /cross-origin isolation/i);

  // Clicking a disabled button spawns nothing — no session starts.
  await debugBtn.click({ force: true });
  await expect(page.locator(".dbg-transport")).toBeHidden();
});

test("A compile error aborts the run and shows verbatim in the console", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  // Put a syntax error in the entry and wait for the analyzer to flag it.
  await page.evaluate(() => {
    const w = window as unknown as { wren: { setSource: (s: string) => void } };
    w.wren.setSource('System.print("unterminated\n');
  });
  await page.waitForFunction(() => {
    const w = window as unknown as { wren: { markers: () => { severity: number }[] } };
    // monaco MarkerSeverity.Error === 8
    return w.wren.markers().some((m) => m.severity === 8);
  }, { timeout: 15_000 });

  await page.locator("#debug").click();

  // The error shows in the console (red channel) and no paused state is entered.
  await expect(page.locator("#console .line.err")).toHaveCount(1);
  await expect(page.locator(".dbg-current-line")).toHaveCount(0);
  await expect(page.locator("#debug")).toBeVisible();
});
