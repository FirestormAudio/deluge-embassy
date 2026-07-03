import { test, expect } from "@playwright/test";

// Regression: the analyzer worker used to silently drop any message that arrived
// before its wasm finished instantiating (and swallowed instantiate failures),
// so the main thread's hover/definition/completion promise hung forever and
// Monaco tooltips went dead. The worker now queues pre-init messages and always
// replies. This test proves a hover query resolves (with real content) rather
// than hanging.

test("analyzer hover resolves (never hangs)", async ({ page }) => {
  const pageErrors: string[] = [];
  page.on("pageerror", (e) => pageErrors.push(String(e)));

  await page.goto("/");
  await page.locator(".monaco-editor").first().waitFor();

  const result = await page.evaluate(async () => {
    const w = window as unknown as {
      wren: { setSource: (s: string) => void; getSource: () => string; hover: (o: number) => Promise<string> };
    };
    w.wren.setSource("var lfo = Osc.sine(5)\nOut.patch(Osc.saw(110))\n");
    // Let the debounced analyze + worker init settle.
    await new Promise((r) => setTimeout(r, 600));
    const off = w.wren.getSource().indexOf("Osc") + 1;
    // Fail fast (rather than hang the whole test) if the promise never settles.
    return Promise.race([
      w.wren.hover(off),
      new Promise<string>((res) => setTimeout(() => res("__HANG__"), 5000)),
    ]);
  });

  expect(result, "hover query hung — analyzer worker never replied").not.toBe("__HANG__");
  expect(result).toContain("Osc"); // the prelude hover doc for the `Osc` class
  expect(pageErrors, `unexpected page errors: ${pageErrors.join("\n")}`).toEqual([]);
});
