// Task 3.3: a spawned wasi-thread, browser port of
// `tools/wren-web-debug/thread-worker.mjs`.
//
// When the wasm calls `wasi.thread-spawn`, the controller (page main thread)
// creates one of these Workers with the SAME compiled module + SAME shared
// memory and posts `{module, memory, tid, startArg}` to it. This worker
// instantiates, then calls `wasi_thread_start(tid, startArg)` — the wasi-threads
// entry that sets up this thread's TLS from `startArg` and runs the Rust thread
// closure (here: the deluge VM under the debugger).
//
// Browser note: unlike Node's `workerData`, browser Workers receive their init
// payload via an initial `postMessage`. Also, all thread-workers are created by
// the MAIN thread (not nested inside another worker): a bundled module worker
// spawning another module worker does not execute reliably in a production
// build under cross-origin isolation, so `wasi.thread-spawn` requests are routed
// up to the controller, which does the `new Worker`. This worker's own
// `thread-spawn` therefore also posts a request up to the main thread.

import { makeWasiImports, ExitStatus } from "./wasi-shim";

interface WorkerScope {
  onmessage: ((e: MessageEvent) => void) | null;
  postMessage(msg: unknown): void;
}
const ctx = self as unknown as WorkerScope;

interface ThreadInit {
  module: WebAssembly.Module;
  memory: WebAssembly.Memory;
  tid: number;
  startArg: number;
}

interface ThreadExports extends WebAssembly.Exports {
  wasi_thread_start(tid: number, startArg: number): void;
}

ctx.onmessage = async (e: MessageEvent<ThreadInit>) => {
  const { module, memory, tid, startArg } = e.data;

  // Further thread-spawns (defensive; the debug session only spawns the one VM
  // thread today) are delegated up to the main thread. Seed tids away from the
  // parent's range to avoid collisions.
  let tidCounter = 1000 + tid * 100;
  const threadSpawn = (childStartArg: number): number => {
    const childTid = tidCounter++;
    ctx.postMessage({ type: "spawn-thread", tid: childTid, startArg: childStartArg });
    return childTid;
  };

  const imports: WebAssembly.Imports = {
    env: { memory },
    wasi_snapshot_preview1: makeWasiImports({ memory, label: `vm-t${tid}` }),
    wasi: { "thread-spawn": threadSpawn },
  };

  try {
    const instance = await WebAssembly.instantiate(module, imports);
    const ex = instance.exports as ThreadExports;
    ex.wasi_thread_start(tid, startArg);
  } catch (err) {
    if (err instanceof ExitStatus) return; // thread closure returned via proc_exit
    ctx.postMessage({
      type: "threaderror",
      tid,
      error: String(err && (err as Error).stack ? (err as Error).stack : err),
    });
  }
  // Thread closure returned; this Worker can exit.
};
