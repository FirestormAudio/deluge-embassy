// Task 3.2: a spawned wasi-thread. When the wasm calls `wasi.thread-spawn`, the
// host (dbg-worker.mjs) launches one of these Workers with the SAME compiled
// module and the SAME shared memory, then this worker calls the module's
// `wasi_thread_start(tid, startArg)` — the wasi-threads entry that sets up this
// thread's TLS from `startArg` and runs the Rust thread closure (here: the
// deluge VM under the debugger). This is the sub-Worker model the spike proved.

import { workerData, Worker } from "node:worker_threads";
import { makeWasiImports, ExitStatus } from "./wasi-shim.mjs";

const { module, memory, tid, startArg } = workerData;

// Allow this thread to spawn further threads (defensive; the debug session
// only spawns the one VM thread today). Seed tids away from the parent's range.
let tidCounter = 1000 + tid * 100;
function threadSpawn(childStartArg) {
  const childTid = tidCounter++;
  new Worker(new URL("./thread-worker.mjs", import.meta.url), {
    workerData: { module, memory, tid: childTid, startArg: childStartArg },
  });
  return childTid;
}

const imports = {
  env: { memory },
  wasi_snapshot_preview1: makeWasiImports({ memory, label: `vm-t${tid}` }),
  wasi: { "thread-spawn": threadSpawn },
};

const instance = await WebAssembly.instantiate(module, imports);

try {
  instance.exports.wasi_thread_start(tid, startArg);
} catch (e) {
  if (!(e instanceof ExitStatus)) throw e;
}
// Thread closure returned; this Worker can exit.
