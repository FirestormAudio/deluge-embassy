// Task 3.2 / 5.2: prove a full `Stopped -> StackTrace -> Terminated` debug run
// over a SharedArrayBuffer, in real wasm threads, in Node.
//
// This is the SAB analogue of the native `tests/agent_transport.rs` test — the
// commands/events cross a `SharedArrayBuffer` (a region of the threaded wasm's
// shared linear memory), driven from the Node MAIN thread while the wasm
// `serve()` loop runs on a Worker and the deluge VM runs on a third
// (wasi-thread-spawned) Worker.
//
// Task 5.2 upgrade: the fixture is now a `Midi.onNoteOn` handler with a
// breakpoint INSIDE its body (line 2). That line never runs during `interpret`
// — it is reached ONLY when `dbg_launch`'s new (note, vel, blocks) scalars fire
// a driven NoteOn against the live VM. So a green run here proves the drive
// params thread through `dbg_launch -> debug_run_driven` and that a callback
// breakpoint parks the VM thread over the SAB (the non-browser check for 5.2).
//
// Run: node tools/wren-web-debug/threads-harness.mjs
//   (build first: npm run build:debug-wasm, or tools/wren-web-debug/build-threads.sh)

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

// Global watchdog: fail loudly instead of hanging forever if a stop/ack never
// arrives (e.g. an un-driven callback breakpoint terminates the run early and
// `writeCmd` would otherwise park on an ack that will never come).
const WATCHDOG_MS = 60000;
setTimeout(() => fail(`watchdog: no SUCCESS within ${WATCHDOG_MS} ms`), WATCHDOG_MS).unref();

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
      // A `Midi.onNoteOn` handler: line 1 registers, line 2 is the body (only
      // reached via a driven NoteOn). Matches the native 5.1 fixture.
      entry: "Midi.onNoteOn = Fn.new { |ch, n, v|\n  output[1].volts = n / 12.0\n}\n",
      bpLines: [2],
      // Task 5.2 drive scalars: fire NoteOn(60, 100), no ticks.
      note: 60,
      vel: 100,
      blocks: 0,
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
    "SUCCESS: driven NoteOn -> callback breakpoint Stopped{line:2} -> StackTrace{stackFrames} -> Terminated over wasm threads",
  );
  await worker.terminate();
  process.exit(0);
}

main().catch((e) => fail(e && e.stack ? e.stack : String(e)));
