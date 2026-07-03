import { test, expect } from "@playwright/test";

// Task 3.3: prove the SAB-driven debug run works in a real browser (Chromium),
// driven from the page main thread — the browser analogue of the Node harness
// tools/wren-web-debug/threads-harness.mjs (Task 3.2 GREEN).
//
// Same program, same breakpoint (line 2), same command sequence
// (Stopped -> StackTrace -> Continue -> Terminated), same assertions.

const PROGRAM = "var a = 1\nvar b = 2\nSystem.print(b)\n";

test("SAB debug run: Stopped{line:2} -> StackTrace -> Terminated", async ({ page }) => {
  const consoleErrors: string[] = [];
  page.on("console", (m) => {
    if (m.type() === "error") consoleErrors.push(m.text());
  });
  page.on("pageerror", (e) => consoleErrors.push(String(e)));

  await page.goto("/");

  // Cross-origin isolation is mandatory for SharedArrayBuffer. Fail loudly.
  const isolated = await page.evaluate(() => crossOriginIsolated === true);
  expect(isolated, "page must be cross-origin isolated (COOP/COEP) for SharedArrayBuffer").toBe(true);

  // Drive the whole debug session inside the page via the __wrenDebug hook.
  const result = await page.evaluate(async (program) => {
    const hook = (window as unknown as {
      __wrenDebug: { createController: () => Promise<any> };
    }).__wrenDebug;
    if (!hook) throw new Error("window.__wrenDebug hook missing");

    const controller = await hook.createController();
    const events: any[] = [];
    controller.on("stopped", (e: any) => events.push(e));
    controller.on("output", (e: any) => events.push(e));
    let terminated = false;
    controller.on("terminated", () => {
      terminated = true;
    });

    controller.setBreakpoints([2]);
    const stopped = await controller.launch(program, [2]);
    const stack = await controller.stackTrace(1);
    await controller.continue();

    // continue() resolves on `terminated`; give the trailing exited event a beat.
    await new Promise((r) => setTimeout(r, 100));

    const out = { stopped, stack, terminated, events };
    controller.dispose();
    return out;
  }, PROGRAM);

  expect(consoleErrors, `unexpected console/page errors: ${consoleErrors.join("\n")}`).toEqual([]);

  // Assertions mirror threads-harness.mjs.
  expect(result.stopped.event).toBe("stopped");
  expect(result.stopped.line).toBe(2);
  expect(result.stopped.reason).toBe("breakpoint");

  expect(Array.isArray(result.stack.stackFrames)).toBe(true);
  expect(result.stack.stackFrames.length).toBeGreaterThanOrEqual(1);
  expect(result.stack.stackFrames[0].line).toBe(2);
  expect(result.stack.totalFrames).toBe(result.stack.stackFrames.length);

  expect(result.terminated).toBe(true);
});
