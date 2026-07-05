// Task 3.3: the "module-main" worker — browser port of
// `tools/wren-web-debug/dbg-worker.mjs`. It hosts the wasm module's main thread:
//   1. instantiates the threaded debug wasm over the shared memory,
//   2. calls `_start` once to bootstrap the main thread's TLS (returns via a
//      caught `proc_exit`/`ExitStatus`),
//   3. publishes the SAB base back to the controller,
//   4. on a `launch` message: writes the entry script + breakpoint-line array
//      into wasm memory and calls `dbg_launch`, which spawns the VM thread (via
//      `wasi.thread-spawn`) and runs `serve()` over the SAB — BLOCKING this
//      worker until the debug session terminates. Workers may block, so this is
//      fine (the page main thread drives the SAB concurrently via
//      Atomics.waitAsync).
//
// `wasi.thread-spawn` allocates a tid synchronously and asks the MAIN thread to
// create the sub-Worker (a bundled worker spawning a nested worker does not
// execute reliably in a production build under cross-origin isolation). The tid
// is returned immediately; the VM thread runs once the main thread has created
// + inited the sub-Worker, synchronized purely via the shared memory's atomics.

import { makeWasiImports, ExitStatus } from "./wasi-shim";

interface WorkerScope {
  onmessage: ((e: MessageEvent) => void) | null;
  postMessage(msg: unknown): void;
}
const ctx = self as unknown as WorkerScope;

interface DbgExports extends WebAssembly.Exports {
  memory: WebAssembly.Memory;
  _start(): void;
  dbg_sab_ptr(): number;
  dbg_alloc(len: number): number;
  dbg_launch(
    entryPtr: number,
    entryLen: number,
    bpPtr: number,
    bpCount: number,
    note: number,
    vel: number,
    blocks: number,
  ): number;
}

type InMsg =
  | { type: "init"; module: WebAssembly.Module; memory: WebAssembly.Memory }
  | { type: "launch"; entry: string; bpLines: number[]; note: number; vel: number; blocks: number };

let ex: DbgExports | null = null;
let sharedMemory: WebAssembly.Memory | null = null;

// `wasi.thread-spawn`: allocate a tid and ask the main thread to create the
// thread-worker. Returns synchronously; the VM thread boots async (atomics sync).
let tidCounter = 1;
function threadSpawn(startArg: number): number {
  const tid = tidCounter++;
  ctx.postMessage({ type: "spawn-thread", tid, startArg });
  return tid;
}

async function onInit(module: WebAssembly.Module, memory: WebAssembly.Memory): Promise<void> {
  sharedMemory = memory;

  const imports: WebAssembly.Imports = {
    env: { memory },
    wasi_snapshot_preview1: makeWasiImports({ memory, label: "dbg" }),
    wasi: { "thread-spawn": threadSpawn },
  };

  const instance = await WebAssembly.instantiate(module, imports);
  ex = instance.exports as DbgExports;

  // Phase 1: init the main thread (TLS / thread pointer) via `_start`.
  try {
    ex._start();
  } catch (err) {
    if (!(err instanceof ExitStatus)) throw err;
  }

  // Phase 2: hand the controller the SAB base so it can map header/regions.
  ctx.postMessage({ type: "base", base: ex.dbg_sab_ptr() });
}

function onLaunch(entry: string, bpLines: number[], note: number, vel: number, blocks: number): void {
  if (!ex || !sharedMemory) throw new Error("worker: launch before init");
  const memory = sharedMemory;

  // Write the entry script and breakpoint-line array into wasm memory.
  const entryBytes = new TextEncoder().encode(entry);
  const entryPtr = ex.dbg_alloc(entryBytes.length);
  new Uint8Array(memory.buffer, entryPtr, entryBytes.length).set(entryBytes);

  const bpPtr = ex.dbg_alloc(bpLines.length * 4);
  {
    const d = new DataView(memory.buffer);
    for (let i = 0; i < bpLines.length; i++) d.setInt32(bpPtr + i * 4, bpLines[i], true);
  }

  // Phase 3: launch + drive `serve()` over the SAB. BLOCKS until session end.
  // Task 5.2: (note, vel, blocks) fire a launch-time NoteOn (+ control-rate
  // ticks) so a breakpoint inside a driven callback stops.
  const rc = ex.dbg_launch(entryPtr, entryBytes.length, bpPtr, bpLines.length, note, vel, blocks);
  ctx.postMessage({ type: "done", rc });
}

ctx.onmessage = (e: MessageEvent<InMsg>) => {
  const msg = e.data;
  if (msg.type === "init") {
    onInit(msg.module, msg.memory).catch((err) =>
      ctx.postMessage({ type: "error", error: String(err && err.stack ? err.stack : err) }),
    );
  } else if (msg.type === "launch") {
    try {
      onLaunch(msg.entry, msg.bpLines, msg.note, msg.vel, msg.blocks);
    } catch (err) {
      ctx.postMessage({ type: "error", error: String(err) });
    }
  }
};
