import { test, expect } from "@playwright/test";

test("renders LEDs and OLED from replayed frames", async ({ page }) => {
  await page.goto("/");
  await expect(page.locator("#status")).toHaveText("ready");

  // Build an UpdateDisplay lighting ui_row 0, col 0 (panel row 5 → page 0, bit 5),
  // plus SetLed for PLAY (index 35).
  const fb = new Array(768).fill(0);
  fb[0] = 1 << 5;
  await page.evaluate(
    ([display]) => {
      (window as unknown as { __replay: (frames: number[][]) => void }).__replay([
        [0x20, ...display], // UpdateDisplay
        [0x24, 35, 1], // SetLed PLAY on
        [0x22, 0, 0, 10, 20, 200], // SetPadRgb col 0, row 0
      ]);
    },
    [fb],
  );

  // PLAY button (aria-label PLAY) becomes lit.
  await expect(page.locator('[aria-label="PLAY"]')).toHaveClass(/lit/);

  // Pad (col 0, row 0) shows its wire RGB as background.
  await expect(page.locator('[aria-label="pad 1, 1"]')).toHaveCSS("background-color", "rgb(10, 20, 200)");

  // OLED canvas has a lit pixel near the top-left (non-background colour).
  const lit = await page.evaluate(() => {
    const c = document.querySelector<HTMLCanvasElement>("#oled")!;
    const px = c.getContext("2d")!.getImageData(1, 1, 1, 1).data;
    return px[0] + px[1] + px[2] > 60; // brighter than #0a0e10 background
  });
  expect(lit).toBe(true);
});
