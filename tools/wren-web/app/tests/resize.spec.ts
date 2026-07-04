import { test, expect } from "@playwright/test";

// The left panel width and the console height are drag-resizable (src/resize.ts)
// via splitter handles, and the sizes persist across reloads.

test("dragging the splitters resizes the left panel + console and persists", async ({ page }) => {
  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  const fbWidth = () => page.locator(".file-browser").evaluate((el) => Math.round(el.getBoundingClientRect().width));
  const consoleHeight = () => page.locator(".console-pane").evaluate((el) => Math.round(el.getBoundingClientRect().height));

  const w0 = await fbWidth();
  const h0 = await consoleHeight();

  // Drag the left splitter ~120px to the right → wider file panel.
  const left = page.locator("#resize-left");
  const lb = (await left.boundingBox())!;
  await page.mouse.move(lb.x + lb.width / 2, lb.y + 200);
  await page.mouse.down();
  await page.mouse.move(lb.x + 120, lb.y + 200, { steps: 8 });
  await page.mouse.up();
  expect(await fbWidth()).toBeGreaterThan(w0 + 80);

  // Drag the instrument splitter ~140px left → wider instrument pane, and its
  // contents zoom-scale up (the OLED grows with the pane).
  const instW = () => page.locator(".instrument-pane").evaluate((el) => Math.round(el.getBoundingClientRect().width));
  const oledW = () => page.locator("#oled").evaluate((el) => Math.round(el.getBoundingClientRect().width));
  const iw0 = await instW();
  const ow0 = await oledW();
  const inst = page.locator("#resize-instrument");
  const ib = (await inst.boundingBox())!;
  await page.mouse.move(ib.x + ib.width / 2, ib.y + 200);
  await page.mouse.down();
  await page.mouse.move(ib.x - 140, ib.y + 200, { steps: 8 });
  await page.mouse.up();
  expect(await instW()).toBeGreaterThan(iw0 + 100);
  expect(await oledW()).toBeGreaterThan(ow0 + 40); // contents scaled with the pane

  // Drag the console splitter ~90px up → taller console.
  const con = page.locator("#resize-console");
  const cb = (await con.boundingBox())!;
  await page.mouse.move(cb.x + cb.width / 2, cb.y + cb.height / 2);
  await page.mouse.down();
  await page.mouse.move(cb.x + cb.width / 2, cb.y - 90, { steps: 8 });
  await page.mouse.up();
  expect(await consoleHeight()).toBeGreaterThan(h0 + 60);

  // Persisted: reload and confirm the new sizes stick.
  const wResized = await fbWidth();
  const hResized = await consoleHeight();
  await page.reload();
  await page.locator(".monaco-editor").first().waitFor();
  expect(Math.abs((await fbWidth()) - wResized)).toBeLessThan(4);
  expect(Math.abs((await consoleHeight()) - hResized)).toBeLessThan(4);
});
