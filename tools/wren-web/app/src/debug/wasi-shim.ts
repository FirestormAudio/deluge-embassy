// Task 3.3: a minimal `wasi_snapshot_preview1` host for the threaded debug wasm,
// over a shared `WebAssembly.Memory` — the browser port of
// `tools/wren-web-debug/wasi-shim.mjs`.
//
// The threaded module (wasm32-wasip1-threads) declares only a handful of wasi
// imports; we implement those and stub the rest benignly. Differences from the
// Node shim: `random_get` uses `crypto.getRandomValues` (no `node:crypto`), and
// `fd_write` routes to `console.log`/`console.warn` (a backstop for VM
// `System.print` / panics — DAP `output` events over the SAB are the primary
// path). `proc_exit` still throws `ExitStatus`: the module is a wasi *command*
// whose `_start` inits the main thread's TLS then returns via `proc_exit`, and
// the host catches that so it can call the `dbg_*` exports afterwards.

export class ExitStatus {
  constructor(public readonly code: number) {}
}

const ERRNO_SUCCESS = 0;
const ERRNO_BADF = 8;

// crypto.getRandomValues rejects views larger than 65536 bytes; chunk to obey.
const MAX_RANDOM_CHUNK = 65536;

export function makeWasiImports({
  memory,
  label = "wasm",
}: {
  memory: WebAssembly.Memory;
  label?: string;
}): WebAssembly.ModuleImports {
  const dv = () => new DataView(memory.buffer);
  const dec = new TextDecoder();

  return {
    random_get(ptr: number, len: number): number {
      const out = new Uint8Array(memory.buffer, ptr, len);
      for (let off = 0; off < len; off += MAX_RANDOM_CHUNK) {
        crypto.getRandomValues(out.subarray(off, Math.min(off + MAX_RANDOM_CHUNK, len)));
      }
      return ERRNO_SUCCESS;
    },
    environ_sizes_get(countPtr: number, sizePtr: number): number {
      dv().setUint32(countPtr, 0, true);
      dv().setUint32(sizePtr, 0, true);
      return ERRNO_SUCCESS;
    },
    environ_get(): number {
      return ERRNO_SUCCESS;
    },
    clock_time_get(_id: number, _precision: bigint, timePtr: number): number {
      const nowNs = BigInt(Date.now()) * 1_000_000n;
      dv().setBigUint64(timePtr, nowNs, true);
      return ERRNO_SUCCESS;
    },
    fd_close(): number {
      return ERRNO_SUCCESS;
    },
    fd_fdstat_get(_fd: number, ptr: number): number {
      const d = dv();
      for (let i = 0; i < 24; i++) d.setUint8(ptr + i, 0); // zeroed fdstat (24 bytes)
      return ERRNO_SUCCESS;
    },
    fd_prestat_get(): number {
      return ERRNO_BADF; // no preopens
    },
    fd_prestat_dir_name(): number {
      return ERRNO_BADF;
    },
    fd_seek(_fd: number, _offset: bigint, _whence: number, newOffsetPtr: number): number {
      dv().setBigUint64(newOffsetPtr, 0n, true);
      return ERRNO_SUCCESS;
    },
    fd_write(fd: number, iovs: number, iovsLen: number, nwrittenPtr: number): number {
      const d = dv();
      const parts: string[] = [];
      let total = 0;
      for (let i = 0; i < iovsLen; i++) {
        const p = iovs + i * 8;
        const buf = d.getUint32(p, true);
        const len = d.getUint32(p + 4, true);
        if (len) parts.push(dec.decode(new Uint8Array(memory.buffer, buf, len)));
        total += len;
      }
      const text = parts.join("");
      if (text.length) {
        if (fd === 2) console.warn(`[${label} fd2]`, text);
        else console.log(`[${label} fd1]`, text);
      }
      d.setUint32(nwrittenPtr, total, true);
      return ERRNO_SUCCESS;
    },
    proc_exit(code: number): never {
      throw new ExitStatus(code);
    },
    sched_yield(): number {
      return ERRNO_SUCCESS;
    },
  };
}
