// Task 3.2: prove a full `Stopped -> StackTrace -> Terminated` debug run over a
// SharedArrayBuffer, in real wasm threads, in Node.
//
// This is the SAB analogue of the native `tests/agent_transport.rs` test: same
// program, same breakpoint (line 2), same command sequence (StackTrace, then
// Continue), same expected events — but instead of an in-process `mpsc` pair,
// the commands/events cross a `SharedArrayBuffer` (a region of the threaded
// wasm's shared linear memory), driven from the Node MAIN thread while the
// wasm `serve()` loop runs on a Worker and the deluge VM runs on a third
// (wasi-thread-spawned) Worker.
//
// Run: node tools/wren-web-debug/threads-harness.mjs
//   (build first: tools/wren-web-debug/build-threads.sh)

import { readFile } from "node:fs/promises";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import { Worker } from "node:worker_threads";

const here = dirname(fileURLToPath(import.meta.url));
const wasmPath = join(here, "target", "wasm32-wasip1-threads", "release", "dbg_threads.wasm");

// ── SAB layout (MUST match src/sab.rs / the Task 3.3 TS controller) ──
const IDX_CMD_SEQ = 0;
const IDX_CMD_ACK = 1;
const IDX_CMD_LEN = 2;
const IDX_EVENT_SEQ = 3;
const IDX_EVENT_ACK = 4;
const IDX_EVENT_LEN = 5;
const HEADER_WORDS = 8;
const CMD_REGION_OFF = HEADER_WORDS * 4; // 32
const REGION_CAP = 8192;
const EVENT_REGION_OFF = CMD_REGION_OFF + REGION_CAP; // 8224

const enc = new TextEncoder();
const dec = new TextDecoder();

function fail(msg) {
  console.error(`FAIL: ${msg}`);
  process.exit(1);
}

async function main() {
  const bytes = await readFile(wasmPath);
  const module = await WebAssembly.compile(bytes);

  // One shared memory backs every thread. Generous initial size so heap growth
  // never has to move/reallocate under the JS views. (max matches the target
  // spec's --max-memory.)
  const memory = new WebAssembly.Memory({ initial: 512, maximum: 16384, shared: true });

  // Spawn the module-main worker; it will post back the SAB base, then block in
  // dbg_launch/serve.
  const worker = new Worker(new URL("./dbg-worker.mjs", import.meta.url), {
    workerData: {
      module,
      memory,
      entry: "var a = 1\nvar b = 2\nSystem.print(b)\n",
      bpLines: [2],
    },
  });
  worker.on("error", (e) => fail(`dbg-worker errored: ${e && e.stack ? e.stack : e}`));

  // Wait (async) for the base before we start blocking the main thread.
  const base = await new Promise((resolve, reject) => {
    const t = setTimeout(() => reject(new Error("timed out waiting for SAB base")), 20000);
    worker.on("message", (m) => {
      if (m && m.type === "base") {
        clearTimeout(t);
        resolve(m.base);
      }
    });
  });

  // Map the header + JSON regions over the shared memory. base is 8-aligned
  // (src/sab.rs' `#[repr(align(8))]` static), so the Int32Array is legal.
  const hdr = new Int32Array(memory.buffer, base, HEADER_WORDS);
  const cmdRegion = new Uint8Array(memory.buffer, base + CMD_REGION_OFF, REGION_CAP);
  const eventRegion = new Uint8Array(memory.buffer, base + EVENT_REGION_OFF, REGION_CAP);

  let lastEventSeq = 0;

  // ── controller primitives (the Node-main-thread side of the SAB protocol) ──
  function writeCmd(obj) {
    const json = enc.encode(JSON.stringify(obj));
    if (json.length > REGION_CAP) fail("command JSON exceeds region");
    // Wait for the worker to have consumed the previous command.
    while (Atomics.load(hdr, IDX_CMD_ACK) !== Atomics.load(hdr, IDX_CMD_SEQ)) {
      Atomics.wait(hdr, IDX_CMD_ACK, Atomics.load(hdr, IDX_CMD_ACK), 5000);
    }
    cmdRegion.set(json);
    Atomics.store(hdr, IDX_CMD_LEN, json.length);
    Atomics.store(hdr, IDX_CMD_SEQ, Atomics.load(hdr, IDX_CMD_SEQ) + 1);
    Atomics.notify(hdr, IDX_CMD_SEQ);
  }

  function readEvent(timeoutMs) {
    const deadline = Date.now() + timeoutMs;
    while (Atomics.load(hdr, IDX_EVENT_SEQ) === lastEventSeq) {
      const left = deadline - Date.now();
      if (left <= 0) return null;
      Atomics.wait(hdr, IDX_EVENT_SEQ, lastEventSeq, left);
    }
    const seq = Atomics.load(hdr, IDX_EVENT_SEQ);
    const len = Atomics.load(hdr, IDX_EVENT_LEN);
    const obj = JSON.parse(dec.decode(eventRegion.subarray(0, len)));
    lastEventSeq = seq;
    // Ack so the worker may reuse the event slot / send the next event.
    Atomics.store(hdr, IDX_EVENT_ACK, seq);
    Atomics.notify(hdr, IDX_EVENT_ACK);
    return obj;
  }

  // ── drive the same sequence as the native mpsc test ──
  const events = [];

  // 1. Initial breakpoint stop (serve's pre-loop run_until_stop pump). Generous
  //    timeout: the VM thread must boot the VM + compile the deluge prelude.
  const stopped = readEvent(30000);
  if (!stopped) fail("no Stopped event (VM never reached the breakpoint)");
  events.push(stopped);

  // 2. Ask for a stack trace at the stop.
  writeCmd({ command: "stackTrace", threadId: 1 });
  const stack = readEvent(15000);
  if (!stack) fail("no StackTrace response");
  events.push(stack);

  // 3. Continue -> runs to completion -> Terminated (+ Exited).
  writeCmd({ command: "continue" });
  let terminated = null;
  for (;;) {
    const e = readEvent(15000);
    if (!e) fail(`no more events before Terminated; got so far: ${JSON.stringify(events)}`);
    events.push(e);
    if (e.event === "terminated") {
      terminated = e;
      // Drain the trailing Exited so serve() returns cleanly (best effort).
      readEvent(3000);
      break;
    }
  }

  // ── assertions (mirror tests/agent_transport.rs) ──
  console.log("EVENTS:", JSON.stringify(events, null, 2));

  const s = events[0];
  if (s.event !== "stopped") fail(`expected Stopped first, got ${JSON.stringify(s)}`);
  if (s.line !== 2) fail(`expected Stopped at line 2, got line ${s.line}`);
  if (s.reason !== "breakpoint") fail(`expected reason "breakpoint", got ${s.reason}`);

  const st = events[1];
  if (st.event !== "stackTrace") fail(`expected StackTrace second, got ${JSON.stringify(st)}`);
  if (!Array.isArray(st.stackFrames) || st.stackFrames.length === 0)
    fail("expected at least one stack frame");
  if (st.stackFrames[0].line !== 2)
    fail(`expected top frame at line 2, got ${st.stackFrames[0].line}`);
  if (st.totalFrames !== st.stackFrames.length) fail("totalFrames mismatch");

  if (!terminated) fail("never saw Terminated");

  console.log(
    "SUCCESS: SAB-driven Stopped{line:2} -> StackTrace{stackFrames} -> Terminated over wasm threads",
  );
  await worker.terminate();
  process.exit(0);
}

main().catch((e) => fail(e && e.stack ? e.stack : String(e)));
