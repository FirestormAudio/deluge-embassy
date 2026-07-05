import { test, expect } from "@playwright/test";

// A mock WS brain runs inside the page via a stubbed WebSocket so no server is
// needed: we intercept construction, capture sent bytes, and push frames in.
test("connects, renders inbound frames, and sends input", async ({ page }) => {
  await page.addInitScript(() => {
    const sent: number[][] = [];
    (window as unknown as { __sent: number[][] }).__sent = sent;
    class FakeWS {
      static OPEN = 1;
      readyState = 1;
      binaryType = "arraybuffer";
      onopen: (() => void) | null = null;
      onmessage: ((e: { data: ArrayBuffer }) => void) | null = null;
      onclose: (() => void) | null = null;
      onerror: (() => void) | null = null;
      constructor(public url: string) { setTimeout(() => this.onopen?.(), 0); }
      send(data: ArrayBuffer) { sent.push([...new Uint8Array(data)]); }
      close() { this.onclose?.(); }
      pushFrame(bytes: number[]) { this.onmessage?.({ data: Uint8Array.from(bytes).buffer }); }
    }
    (window as unknown as { __lastWS: FakeWS | null }).__lastWS = null;
    (window as unknown as { WebSocket: unknown }).WebSocket = new Proxy(FakeWS, {
      construct(t, args) {
        const ws = new (t as typeof FakeWS)(args[0] as string);
        (window as unknown as { __lastWS: FakeWS }).__lastWS = ws;
        return ws;
      },
    });
  });

  await page.goto("/");
  await page.fill("#ws-url", "ws://127.0.0.1:9000");
  await page.click("#connect");
  await expect(page.locator("#status")).toHaveText(/open/);

  // On open the client sends Ready (0x12).
  await expect
    .poll(() => page.evaluate(() => (window as unknown as { __sent: number[][] }).__sent[0]))
    .toEqual([0x12]);

  // Brain lights PLAY; the button becomes lit.
  await page.evaluate(() =>
    (window as unknown as { __lastWS: { pushFrame(b: number[]): void } }).__lastWS.pushFrame([0x24, 35, 1]),
  );
  await expect(page.locator('[aria-label="PLAY"]')).toHaveClass(/lit/);

  // Clicking a pad sends PadPressed (0x01) then PadReleased (0x02).
  await page.locator('[aria-label="pad 1, 1"]').click();
  const sent = await page.evaluate(() => (window as unknown as { __sent: number[][] }).__sent);
  expect(sent.some((f: number[]) => f[0] === 0x01)).toBe(true);
  expect(sent.some((f: number[]) => f[0] === 0x02)).toBe(true);
});
