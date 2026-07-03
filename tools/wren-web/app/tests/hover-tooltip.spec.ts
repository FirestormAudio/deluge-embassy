import { test, expect } from "@playwright/test";

// Regression: the editor imported the bare `monaco-editor/.../editor.api`, which
// ships NO editor contributions — so the registered hover/completion/definition
// providers were never invoked (no hover controller existed) and tooltips were
// dead. The contributions are now imported explicitly. This drives a real mouse
// hover over a prelude symbol and asserts the tooltip renders with its doc.

test("hovering a symbol shows the analyzer tooltip", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  // A single token on line 1 so hovering the line lands squarely on it.
  await page.evaluate(() => {
    (window as unknown as { wren: { setSource: (s: string) => void } }).wren.setSource("Osc\n");
  });
  await page.waitForTimeout(400); // let the model + analyzer settle

  // Hover the token span itself (a .view-line spans the full width, so its
  // centre is empty space, not the glyph).
  await page.getByText("Osc", { exact: true }).first().hover();

  // The analyzer-backed content hover renders asynchronously; assert the visible
  // (non-hidden) hover widget shows the prelude doc for the `Osc` class.
  await expect(page.locator(".monaco-hover:not(.hidden)")).toContainText("class", { timeout: 5000 });
});
