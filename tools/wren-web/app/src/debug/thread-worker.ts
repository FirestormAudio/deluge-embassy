// Task 3.3: a spawned wasi-thread, browser port of
// `tools/wren-web-debug/thread-worker.mjs`.
//
// When the wasm calls `wasi.thread-spawn`, the spawning worker (worker.ts, or
// another thread-worker) launches one of these Workers with the SAME compiled
// module + SAME shared memory and posts `{module, memory, tid, startArg}` to it.
// This worker instantiates, then calls `wasi_thread_start(tid, startArg)` — the
// wasi-threads entry that sets up this thread's TLS from `startArg` and runs the
// Rust thread closure (here: the deluge VM under the debugger).
//
// Browser note: unlike Node's `workerData`, browser Workers receive their init
// payload via an initial `postMessage`; we handle exactly one such message.

import { makeWasiImports, ExitStatus } from "./wasi-shim";

interface WorkerScope {
  onmessage: ((e: MessageEvent) => void) | null;
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

  // Allow this thread to spawn further threads (defensive; the debug session
  // only spawns the one VM thread today). Seed tids away from the parent's range.
  let tidCounter = 1000 + tid * 100;
  const threadSpawn = (childStartArg: number): number => {
    const childTid = tidCounter++;
    const w = new Worker(new URL("./thread-worker.ts", import.meta.url), { type: "module" });
    w.postMessage({ module, memory, tid: childTid, startArg: childStartArg } satisfies ThreadInit);
    return childTid;
  };

  const imports: WebAssembly.Imports = {
    env: { memory },
    wasi_snapshot_preview1: makeWasiImports({ memory, label: `vm-t${tid}` }),
    wasi: { "thread-spawn": threadSpawn },
  };

  const instance = await WebAssembly.instantiate(module, imports);
  const ex = instance.exports as ThreadExports;

  try {
    ex.wasi_thread_start(tid, startArg);
  } catch (err) {
    if (!(err instanceof ExitStatus)) throw err;
  }
  // Thread closure returned; this Worker can exit.
};
