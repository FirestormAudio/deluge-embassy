import { test, expect, type Page } from "@playwright/test";

// Task 5.2 (the last task of the Wren Web Debugger plan): the DRIVE form threads
// a launch-time host event through `dbg_launch -> debug_run_driven` so a
// breakpoint INSIDE a driven callback fires — in the browser, end-to-end.
//
// Fixture: a `Midi.onNoteOn` handler. Line 1 registers it (runs during
// `interpret`); line 2 is the body, reached ONLY when the handler is invoked —
// which happens solely via the DRIVE form's note-on fired after `interpret`.
// So a stop at line 2 proves the driven callback path (non-vacuous: without a
// note, line 2 never runs and the program terminates without stopping).
const PROGRAM = "Midi.onNoteOn = Fn.new { |ch, n, v|\n  output[1].volts = n / 12.0\n}\n";

// Click Monaco's glyph margin at a 1-based line (line height 20, top padding 14
// → center = 14 + (L-1)*20 + 10). Matches the other debug specs.
async function clickGutter(page: Page, line: number) {
  const overlays = page.locator(".margin-view-overlays").first();
  await overlays.click({ position: { x: 8, y: 14 + (line - 1) * 20 + 10 }, force: true });
}

test("DRIVE note fires a callback breakpoint; no note runs to termination", async ({ page }, testInfo) => {
  const pageErrors: string[] = [];
  page.on("pageerror", (e) => pageErrors.push(String(e)));

  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  // Cross-origin isolation is mandatory for the SharedArrayBuffer transport.
  expect(await page.evaluate(() => crossOriginIsolated === true)).toBe(true);

  // Load the handler fixture into the entry file; wait for the model to settle.
  await page.evaluate((src) => {
    (window as unknown as { wren: { setSource: (s: string) => void } }).wren.setSource(src);
  }, PROGRAM);
  await expect(page.locator(".view-lines")).toContainText("output[1].volts");

  // Open the DRIVE form (left pane → debug segment) and pick MIDI note 60.
  await page.locator("#pane-debug").click();
  const noteInput = page.locator("#dbg-drive-note");
  await expect(noteInput).toBeVisible();
  await noteInput.fill("60");

  // Breakpoint on line 2 (inside the handler body) and start the debug run.
  await clickGutter(page, 2);
  await expect(page.locator(".dbg-breakpoint")).toHaveCount(1);
  await page.locator("#debug").click();

  // The driven NoteOn fired the handler → we PAUSE at line 2 (phosphor current
  // line + the call-stack top frame at :2).
  await expect(page.locator(".dbg-current-line")).toHaveCount(1, { timeout: 30_000 });
  const frames = page.locator("#dbg-callstack .dbg-frame");
  await expect(frames.first()).toBeVisible();
  await expect(frames.first()).toHaveClass(/selected/);
  await expect(frames.first()).toContainText(":2");

  // Screenshot: the DRIVE form (note = 60) + the paused-in-callback state.
  const shot = testInfo.outputPath("debug-drive.png");
  await page.screenshot({ path: "tests/screenshots/debug-drive.png", fullPage: false });
  await page.screenshot({ path: shot, fullPage: false });

  // End the paused session.
  await page.locator("#dbg-stop").click();
  await expect(page.locator(".dbg-current-line")).toHaveCount(0);
  await expect(page.locator("#debug")).toBeVisible();

  // Non-vacuity: clear the note and Debug again — the handler never fires, so
  // the run terminates WITHOUT ever pausing (proves the line-2 stop above was
  // the driven callback, not an unconditional artifact).
  await noteInput.fill("");
  await page.locator("#debug").click();
  await expect(page.locator("#dbg-stop")).toBeVisible(); // session started…
  await expect(page.locator("#debug")).toBeVisible({ timeout: 30_000 }); // …ran to idle
  await expect(page.locator(".dbg-current-line")).toHaveCount(0); // never paused

  expect(pageErrors, `unexpected page errors: ${pageErrors.join("\n")}`).toEqual([]);
});
