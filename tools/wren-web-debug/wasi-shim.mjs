// Task 3.2: a minimal `wasi_snapshot_preview1` host for the threaded debug
// wasm, over a shared `WebAssembly.Memory`. Node's built-in `node:wasi` does
// not implement wasi-threads' `wasi.thread-spawn`, so the harness rolls its
// own tiny host (this file) + a `worker_threads`-based thread spawner (see
// dbg-worker.mjs / thread-worker.mjs). Only the imports the module actually
// declares are implemented; everything else is a benign stub.
//
// `proc_exit` throws an `ExitStatus` instead of terminating the process: the
// module ships as a wasi *command* (its `_start` inits the main thread's TLS
// then returns, and `_start` finishes by calling `proc_exit`). The host calls
// `_start` purely for that init and catches the `ExitStatus` so it can go on to
// call the `dbg_*` exports on the now-initialized instance.

import { randomFillSync } from "node:crypto";

export class ExitStatus {
  constructor(code) {
    this.code = code;
  }
}

const ERRNO_SUCCESS = 0;

export function makeWasiImports({ memory, label = "wasm" }) {
  const dv = () => new DataView(memory.buffer);

  return {
    // Fill the destination with random bytes (wren/std may seed a hasher).
    random_get(ptr, len) {
      const tmp = Buffer.allocUnsafe(len);
      randomFillSync(tmp);
      new Uint8Array(memory.buffer, ptr, len).set(tmp);
      return ERRNO_SUCCESS;
    },
    // No environment.
    environ_sizes_get(countPtr, sizePtr) {
      dv().setUint32(countPtr, 0, true);
      dv().setUint32(sizePtr, 0, true);
      return ERRNO_SUCCESS;
    },
    environ_get() {
      return ERRNO_SUCCESS;
    },
    clock_time_get(_id, _precision, timePtr) {
      const nowNs = BigInt(Date.now()) * 1_000_000n;
      dv().setBigUint64(timePtr, nowNs, true);
      return ERRNO_SUCCESS;
    },
    fd_close() {
      return ERRNO_SUCCESS;
    },
    fd_fdstat_get(_fd, ptr) {
      // Zeroed fdstat (24 bytes) is accepted by wasi-libc for our use.
      const d = dv();
      for (let i = 0; i < 24; i++) d.setUint8(ptr + i, 0);
      return ERRNO_SUCCESS;
    },
    fd_prestat_get() {
      return 8; // EBADF: no preopens.
    },
    fd_prestat_dir_name() {
      return 8; // EBADF.
    },
    fd_seek(_fd, _offset, _whence, newOffsetPtr) {
      dv().setBigUint64(newOffsetPtr, 0n, true);
      return ERRNO_SUCCESS;
    },
    // Route stdout/stderr to the console (surfaces VM panics/prints); count all
    // bytes as written.
    fd_write(fd, iovs, iovsLen, nwrittenPtr) {
      const d = dv();
      const chunks = [];
      let total = 0;
      for (let i = 0; i < iovsLen; i++) {
        const p = iovs + i * 8;
        const buf = d.getUint32(p, true);
        const len = d.getUint32(p + 4, true);
        chunks.push(Buffer.from(new Uint8Array(memory.buffer, buf, len)));
        total += len;
      }
      const bytes = Buffer.concat(chunks);
      if (bytes.length) {
        if (fd === 2) process.stderr.write(`[${label} fd2] ${bytes}`);
        else process.stdout.write(`[${label} fd1] ${bytes}`);
      }
      d.setUint32(nwrittenPtr, total, true);
      return ERRNO_SUCCESS;
    },
    proc_exit(code) {
      throw new ExitStatus(code);
    },
    sched_yield() {
      return ERRNO_SUCCESS;
    },
  };
}
