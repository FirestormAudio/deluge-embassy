// Task 3.2: the worker that hosts the wasm "module main" thread — the analogue
// of the wasm module running inside a browser Web Worker. It:
//   1. instantiates the threaded debug wasm over the shared memory,
//   2. calls `_start` once to bootstrap the main thread's TLS (the spike's
//      cdylib-init wrinkle; `_start` returns via a caught `proc_exit`),
//   3. publishes the SAB base back to the controller (the Node main thread),
//   4. writes the entry script + breakpoints into wasm memory,
//   5. calls `dbg_launch`, which spawns the VM thread (via `wasi.thread-spawn`,
//      handled by `threadSpawn` below) and runs `serve()` over the SAB —
//      BLOCKING here until the debug session terminates.

import { parentPort, workerData, Worker } from "node:worker_threads";
import { makeWasiImports, ExitStatus } from "./wasi-shim.mjs";

const { module, memory, entry, bpLines, note = -1, vel = 100, blocks = 0 } = workerData;

// `wasi.thread-spawn`: launch a sub-Worker that runs `wasi_thread_start` with
// the same module + shared memory. Returns a positive tid synchronously.
let tidCounter = 1;
function threadSpawn(startArg) {
  const tid = tidCounter++;
  new Worker(new URL("./thread-worker.mjs", import.meta.url), {
    workerData: { module, memory, tid, startArg },
  });
  return tid;
}

const imports = {
  env: { memory },
  wasi_snapshot_preview1: makeWasiImports({ memory, label: "dbg" }),
  wasi: { "thread-spawn": threadSpawn },
};

const instance = await WebAssembly.instantiate(module, imports);
const ex = instance.exports;

// Phase 1: initialize the main thread (TLS / thread pointer) via `_start`.
try {
  ex._start();
} catch (e) {
  if (!(e instanceof ExitStatus)) throw e;
}

// Phase 2: hand the controller the SAB base so it can map the header/regions.
const base = ex.dbg_sab_ptr();
parentPort.postMessage({ type: "base", base });

// Write the entry script and breakpoint-line array into wasm memory.
const entryBytes = new TextEncoder().encode(entry);
const entryPtr = ex.dbg_alloc(entryBytes.length);
new Uint8Array(memory.buffer, entryPtr, entryBytes.length).set(entryBytes);

const bpPtr = ex.dbg_alloc(bpLines.length * 4);
{
  const d = new DataView(memory.buffer);
  for (let i = 0; i < bpLines.length; i++) d.setInt32(bpPtr + i * 4, bpLines[i], true);
}

// Phase 3: launch + drive `serve()` over the SAB. BLOCKS until the session ends.
// Task 5.2: the trailing (note, vel, blocks) scalars fire a launch-time NoteOn
// (+ `blocks` control-rate ticks) so a breakpoint inside a driven callback stops.
const rc = ex.dbg_launch(entryPtr, entryBytes.length, bpPtr, bpLines.length, note, vel, blocks);
parentPort.postMessage({ type: "done", rc });
