// Task 3.3: the SharedArrayBuffer debug protocol, browser side.
//
// This is the TypeScript mirror of `tools/wren-web-debug/src/sab.rs` — the SAB
// layout MUST match that file byte-for-byte (the Rust worker and this module
// read/write the same header words + JSON regions over the *same* shared wasm
// linear memory). The layout table (from sab.rs' module docs):
//
//   | bytes       | i32 idx | name        | writer | meaning                       |
//   |-------------|---------|-------------|--------|-------------------------------|
//   | 0..4        | 0       | CMD_SEQ     | main   | bumped when a command is published |
//   | 4..8        | 1       | CMD_ACK     | worker | set to CMD_SEQ once command read   |
//   | 8..12       | 2       | CMD_LEN     | main   | byte length of the command JSON    |
//   | 12..16      | 3       | EVENT_SEQ   | worker | bumped when an event is published  |
//   | 16..20      | 4       | EVENT_ACK   | main   | set to EVENT_SEQ once event read   |
//   | 20..24      | 5       | EVENT_LEN   | worker | byte length of the event JSON      |
//   | 24..28      | 6       | PAUSE       | main   | pause request (reserved; unwired)  |
//   | 28..32      | 7       | reserved    | —      | padding / future use               |
//   | 32..8224    | —       | CMD region  | main   | command JSON bytes (main → worker) |
//   | 8224..16416 | —       | EVENT region| worker | event JSON bytes (worker → main)   |
//
// The SAB is a *region* of the wasm's shared linear memory, not a standalone
// SharedArrayBuffer; its base offset comes from the exported `dbg_sab_ptr()`.
// `memory.buffer` is a SharedArrayBuffer (memory created `{shared:true}`), so
// Atomics work on the Int32Array header.

export const HEADER_WORDS = 8;
export const IDX_CMD_SEQ = 0;
export const IDX_CMD_ACK = 1;
export const IDX_CMD_LEN = 2;
export const IDX_EVENT_SEQ = 3;
export const IDX_EVENT_ACK = 4;
export const IDX_EVENT_LEN = 5;
export const IDX_PAUSE = 6;

export const CMD_REGION_OFF = HEADER_WORDS * 4; // 32
export const REGION_CAP = 8192;
export const EVENT_REGION_OFF = CMD_REGION_OFF + REGION_CAP; // 8224
export const SAB_BYTES = CMD_REGION_OFF + REGION_CAP * 2; // 16416

// ── DTOs (mirror tools/wren-web-debug/src/transport.rs) ──────────────────────

export interface ThreadDto {
  id: number;
  name: string;
}
export interface StackFrameDto {
  id: number;
  name: string;
  line: number;
  column: number;
  module: string;
}
export interface ScopeDto {
  name: string;
  variablesReference: number;
  expensive: boolean;
}
export interface VariableDto {
  name: string;
  value: string;
  variablesReference: number;
}

/** Commands sent main → worker. Serialized as `{command, ...}` (camelCase). */
export type DebugCmd =
  | { command: "continue" }
  | { command: "threads" }
  | { command: "stackTrace"; threadId: number }
  | { command: "scopes"; frameId: number }
  | { command: "variables"; variablesReference: number }
  | { command: "evaluate"; frameId: number; expression: string }
  | { command: "next" }
  | { command: "stepIn" }
  | { command: "stepOut" };

/** Events emitted worker → main. Serialized as `{event, ...}` (camelCase). */
export type DebugEvent =
  | { event: "stopped"; threadId: number; line: number; reason: string; allThreadsStopped: boolean }
  | { event: "output"; category: string; output: string }
  | { event: "terminated" }
  | { event: "exited"; exitCode: number }
  | { event: "threads"; threads: ThreadDto[] }
  | { event: "stackTrace"; stackFrames: StackFrameDto[]; totalFrames: number }
  | { event: "scopes"; scopes: ScopeDto[] }
  | { event: "variables"; variables: VariableDto[] }
  | { event: "evaluate"; result: string; variablesReference: number }
  | { event: "evaluateError"; message: string };

export type DebugEventName = DebugEvent["event"];

const enc = new TextEncoder();
const dec = new TextDecoder();

/**
 * A single-slot, seq/ack, shared-memory channel over the wasm's linear memory —
 * the page-main-thread half of the protocol `src/sab.rs::SabTransport` runs on
 * the worker side. One instance per debug session.
 *
 * Reads (events) MUST use `Atomics.waitAsync` because blocking `Atomics.wait`
 * throws on a browser's main thread. Writes (commands) only ever short-block
 * waiting for the worker to ack the *previous* command, which — because we
 * serialize commands and never issue the next until the prior reply arrives —
 * is effectively always already true, so the async spin never actually parks.
 */
export class SabChannel {
  readonly hdr: Int32Array;
  private readonly cmdRegion: Uint8Array;
  private readonly eventRegion: Uint8Array;
  private lastEventSeq = 0;

  constructor(buffer: ArrayBufferLike, base: number) {
    this.hdr = new Int32Array(buffer, base, HEADER_WORDS);
    this.cmdRegion = new Uint8Array(buffer, base + CMD_REGION_OFF, REGION_CAP);
    this.eventRegion = new Uint8Array(buffer, base + EVENT_REGION_OFF, REGION_CAP);
  }

  /**
   * Publish a command JSON. Waits (async) until the worker has drained the
   * previous command (CMD_ACK === CMD_SEQ), then writes bytes + len and bumps
   * CMD_SEQ, notifying the worker parked in `recv_cmd`.
   */
  async writeCmd(cmd: DebugCmd): Promise<void> {
    const json = enc.encode(JSON.stringify(cmd));
    if (json.length > REGION_CAP) throw new Error("command JSON exceeds region cap");

    // Wait for the worker to have acked the previous command.
    while (Atomics.load(this.hdr, IDX_CMD_ACK) !== Atomics.load(this.hdr, IDX_CMD_SEQ)) {
      const ack = Atomics.load(this.hdr, IDX_CMD_ACK);
      const r = Atomics.waitAsync(this.hdr, IDX_CMD_ACK, ack);
      if (r.async) await r.value;
    }

    this.cmdRegion.set(json);
    Atomics.store(this.hdr, IDX_CMD_LEN, json.length);
    Atomics.store(this.hdr, IDX_CMD_SEQ, Atomics.load(this.hdr, IDX_CMD_SEQ) + 1);
    Atomics.notify(this.hdr, IDX_CMD_SEQ);
  }

  /**
   * Await the next event JSON. Parks on EVENT_SEQ via `Atomics.waitAsync` until
   * the worker bumps it, then decodes the bytes, advances `lastEventSeq`, and
   * acks (EVENT_ACK = EVENT_SEQ) so the worker's blocking `send_event` may reuse
   * the slot for the next event.
   */
  async readEventAsync(): Promise<DebugEvent> {
    for (;;) {
      const seq = Atomics.load(this.hdr, IDX_EVENT_SEQ);
      if (seq !== this.lastEventSeq) {
        const len = Atomics.load(this.hdr, IDX_EVENT_LEN);
        const json = dec.decode(this.eventRegion.subarray(0, len));
        this.lastEventSeq = seq;
        Atomics.store(this.hdr, IDX_EVENT_ACK, seq);
        Atomics.notify(this.hdr, IDX_EVENT_ACK);
        return JSON.parse(json) as DebugEvent;
      }
      const r = Atomics.waitAsync(this.hdr, IDX_EVENT_SEQ, this.lastEventSeq);
      if (r.async) await r.value; // "ok" (notified) or "timed-out" — loop re-checks
    }
  }

  /**
   * Set the PAUSE word (idx 6). NOTE: reserved/unwired — the current
   * `debug_run` builds the session without a `Pause` handle so nothing polls
   * this yet (see src/sab.rs' `# PAUSE` docs). Wired here for layout agreement.
   */
  pause(): void {
    Atomics.store(this.hdr, IDX_PAUSE, 1);
    Atomics.notify(this.hdr, IDX_PAUSE);
  }
}
