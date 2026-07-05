// Task 3.1 wasm boot smoke.
//
// Instantiates the wasm32-wasip1 build of the debug core, satisfies its
// `wasi_snapshot_preview1` imports with Node's built-in WASI, calls the
// exported `dbg_boot()`, and asserts it returns 0 and captured "booted".
//
// This is a REACTOR module (no `_start`; it exports `_initialize`/nothing and
// individual `dbg_*` functions), so we use `wasi.initialize(instance)` rather
// than `wasi.start()`.
//
// Run: node tools/wren-web-debug/smoke.mjs

import { readFile } from "node:fs/promises";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import { WASI } from "node:wasi";

const here = dirname(fileURLToPath(import.meta.url));
const wasmPath = join(
  here,
  "target",
  "wasm32-wasip1",
  "release",
  "wren_web_debug.wasm",
);

const wasi = new WASI({ version: "preview1", args: [], env: {} });

const bytes = await readFile(wasmPath);
const module = await WebAssembly.compile(bytes);
const instance = await WebAssembly.instantiate(module, {
  wasi_snapshot_preview1: wasi.wasiImport,
});

// Reactor init (runs libc ctors, wires memory); tolerate absence of _initialize.
wasi.initialize(instance);

const { dbg_boot, dbg_out_ptr, dbg_out_len, memory } = instance.exports;

const rc = dbg_boot();

const ptr = dbg_out_ptr();
const len = dbg_out_len();
const captured = new TextDecoder().decode(
  new Uint8Array(memory.buffer, ptr, len),
);

console.log(`dbg_boot() -> ${rc}`);
console.log(`captured output: ${JSON.stringify(captured)}`);

if (rc !== 0) {
  console.error(`FAIL: dbg_boot returned ${rc}, expected 0`);
  process.exit(1);
}
if (!captured.includes("booted")) {
  console.error(`FAIL: captured output did not contain "booted"`);
  process.exit(1);
}

console.log("SUCCESS: debug core booted under wasm (wren-core VM + deluge prelude ran)");
