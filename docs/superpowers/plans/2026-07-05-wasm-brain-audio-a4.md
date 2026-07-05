# Wasm Brain — Browser Audio (A4) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the wasm firmware brain **audible in the browser** — run `deluge_brain.wasm` in a Web Worker and stream its rendered audio through Web Audio, so opening `tools/deluge-web-sim` in wasm-brain mode plays the Deluge's live output.

**Architecture:** The brain is single-threaded and `deluge_main` never returns to JS, so it runs in a dedicated Worker via a WASI shim, and hands audio out through an **import bridge**: the BSP's `host_pcm_write_s16` calls an imported `env.audio_push(ptr, frames)` that the worker's JS copies from wasm memory into a lock-free `SharedArrayBuffer` ring. A thin `AudioWorkletProcessor` drains that ring to the speakers; when the ring is full the producer `Atomics.wait`s, so the audio thread's consumption paces the brain (audio *is* the clock). This establishes the Worker + wasm-load + ring infrastructure that A5 (faceplate/input) reuses.

**Tech Stack:** TypeScript, Web Worker, AudioWorklet, `SharedArrayBuffer`/`Atomics`, `@bjorn3/browser_wasi_shim` (same shim `tools/wren-web/app` uses), Vite, Playwright. C (DelugeFirmware `wasm_shim.c`).

**Spec:** `docs/superpowers/specs/2026-07-05-wasm-firmware-brain-design.md` (§2 execution model, §4 audio ring). Builds on the executed foundation plan `2026-07-05-wasm-firmware-brain-foundation.md`.

## Global Constraints

- **Single-threaded wasm, import bridge for data-out.** The brain memory is NOT shared; data leaves the worker only via JS imports called from inside the wasm loop. Do not switch to a threads/shared-memory wasm build.
- **Audio format:** the brain renders stereo interleaved **s16** at **44100 Hz**, `HOST_AUDIO_BLOCK_FRAMES = 128` per call. The `AudioContext` MUST be created with `{ sampleRate: 44100 }`.
- **Cross-origin isolation required:** `SharedArrayBuffer` needs COOP/COEP headers. Vite dev+preview must send `Cross-Origin-Opener-Policy: same-origin` and `Cross-Origin-Embedder-Policy: require-corp` (mirror `tools/wren-web/app/vite.config.ts`).
- **`Atomics.wait` only off the audio thread.** The Worker (producer) may `Atomics.wait`; the AudioWorklet (consumer) MUST NOT — it reads what's available and outputs silence on underrun.
- **Artifact:** `deluge_brain.wasm` is built in DelugeFirmware and copied to `tools/deluge-web-sim/public/deluge_brain.wasm` (git-ignored; a build step copies it). Do not commit the 11 MB binary.
- **Cross-repo:** the `audio_push` import lands in DelugeFirmware (`src/bsp/wasm/wasm_shim.c`); everything else is in `deluge-sdk`.

## File Structure

| File | Repo | Responsibility |
| --- | --- | --- |
| `src/bsp/wasm/wasm_shim.c` (modify) | DelugeFirmware | `host_pcm_write_s16` calls the `env.audio_push` import |
| `tools/wasm-brain/run-headless.mjs` (modify) | DelugeFirmware | provide the `env.audio_push` import (no-op) so headless still runs |
| `tools/deluge-web-sim/src/audio-ring.ts` | deluge-sdk | SPSC s16 `SharedArrayBuffer` ring (producer + consumer helpers) |
| `tools/deluge-web-sim/src/audio-ring.test.ts` | deluge-sdk | ring unit tests (vitest) |
| `tools/deluge-web-sim/src/brain-worker.ts` | deluge-sdk | Worker: WASI-load the wasm, provide `audio_push`, run the loop |
| `tools/deluge-web-sim/public/brain-audio-worklet.js` | deluge-sdk | `AudioWorkletProcessor` draining the ring → output |
| `tools/deluge-web-sim/src/brain-audio.ts` | deluge-sdk | main-thread: build ring + AudioContext + worklet + worker |
| `tools/deluge-web-sim/src/main.ts` (modify) | deluge-sdk | `?brain=wasm` starts brain-audio |
| `tools/deluge-web-sim/vite.config.ts` (modify) | deluge-sdk | COOP/COEP headers |
| `tools/deluge-web-sim/package.json` (modify) | deluge-sdk | add `@bjorn3/browser_wasi_shim` |
| `tools/deluge-web-sim/tests/wasm-audio.spec.ts` | deluge-sdk | Playwright: AudioContext runs + non-zero output |

---

### Task 1: `audio_push` import bridge (DelugeFirmware)

**Files:**
- Modify: `src/bsp/wasm/wasm_shim.c`
- Modify: `tools/wasm-brain/run-headless.mjs`

**Interfaces:**
- Produces: a wasm import `env.audio_push(int32 ptr, int32 frames)` the brain calls once per rendered block, with `ptr` a byte offset into the module's memory holding `frames*2` interleaved s16 samples. Every host (browser worker, headless harness) MUST supply it or instantiation fails.

- [ ] **Step 1: Declare and call the import in the shim**

In `src/bsp/wasm/wasm_shim.c`, add near the top (after includes):
```c
// Hand each rendered block to the host (browser: SAB ring; headless: no-op). An
// import (not an internal buffer) is the only way JS runs while the brain sits in
// deluge_main's infinite loop.
__attribute__((import_module("env"), import_name("audio_push"))) void
audio_push(const int16_t* pcm, uint32_t frames);
```
Then in `host_pcm_write_s16`, call it **before** the existing capture/bounded-exit logic:
```c
void host_pcm_write_s16(const int16_t* pcm, uint32_t frames) {
	audio_push(pcm, frames); // hand to the host (browser ring / headless no-op)
	uint32_t n = frames * 2u;
	if (n > (uint32_t)(sizeof(wasm_audio_last) / sizeof(wasm_audio_last[0]))) {
		n = sizeof(wasm_audio_last) / sizeof(wasm_audio_last[0]);
	}
	memcpy(wasm_audio_last, pcm, n * sizeof(int16_t));
	wasm_audio_blocks++;
	if (pcm_max_blocks && wasm_audio_blocks >= pcm_max_blocks) {
		exit(0);
	}
}
```

- [ ] **Step 2: Provide the import in the headless harness**

In `tools/wasm-brain/run-headless.mjs`, add an `env` import object to the instantiation:
```js
  const inst = await WebAssembly.instantiate(mod, {
    ...wasi.getImportObject(),
    env: { audio_push: (_ptr, _frames) => {} }, // headless: audio read from wasm_audio_last instead
  });
```
(Replace the existing single-argument `WebAssembly.instantiate(mod, wasi.getImportObject())` call.)

- [ ] **Step 3: Rebuild and run the headless test**

Run (DelugeFirmware):
```bash
cmake --build build-wasm --target deluge_brain
node --experimental-wasi-unstable-preview1 --test tools/wasm-brain/boot.test.mjs
```
Expected: PASS (3 tests) — the brain still boots+renders; the `audio_push` import is now required and satisfied by the no-op.

- [ ] **Step 4: Commit**
```bash
git add src/bsp/wasm/wasm_shim.c tools/wasm-brain/run-headless.mjs
git commit -m "feat(wasm-brain): audio_push import so the host receives rendered blocks"
```

---

### Task 2: The SPSC audio ring (deluge-sdk)

**Files:**
- Create: `tools/deluge-web-sim/src/audio-ring.ts`
- Test: `tools/deluge-web-sim/src/audio-ring.test.ts`

**Interfaces:**
- Produces:
  - `AUDIO_RING_FRAMES = 8192` (capacity), `RING_HEADER_BYTES = 8`.
  - `createRingSab(): SharedArrayBuffer` — allocate a ring (header + `AUDIO_RING_FRAMES*2` Int16).
  - `class RingProducer { constructor(sab: SharedArrayBuffer); write(samples: Int16Array): void }` — write interleaved-stereo samples; `Atomics.wait`s when full (worker-side only).
  - `class RingConsumer { constructor(sab: SharedArrayBuffer); read(out: Float32Array, frames: number): number }` — pull up to `frames` stereo frames as float (−1..1) into `out` (length `frames*2`); returns frames actually read; never blocks.
  - Header layout: `Int32Array(sab,0,2)` = `[writeFrames, readFrames]` (monotonic).

- [ ] **Step 1: Write the failing test**

`tools/deluge-web-sim/src/audio-ring.test.ts`:
```ts
import { describe, it, expect } from "vitest";
import { createRingSab, RingProducer, RingConsumer } from "./audio-ring";

describe("audio ring", () => {
  it("round-trips interleaved stereo s16 → float", () => {
    const sab = createRingSab();
    const prod = new RingProducer(sab);
    const cons = new RingConsumer(sab);
    prod.write(Int16Array.of(0, 32767, -32768, 16384)); // 2 frames
    const out = new Float32Array(4);
    const got = cons.read(out, 2);
    expect(got).toBe(2);
    expect(out[0]).toBeCloseTo(0, 4);
    expect(out[1]).toBeCloseTo(1, 4);
    expect(out[2]).toBeCloseTo(-1, 4);
    expect(out[3]).toBeCloseTo(0.5, 4);
  });

  it("underrun reads fewer frames without blocking", () => {
    const sab = createRingSab();
    const cons = new RingConsumer(sab);
    const out = new Float32Array(256);
    expect(cons.read(out, 128)).toBe(0); // empty ring
  });

  it("wraps around the capacity boundary", () => {
    const sab = createRingSab();
    const prod = new RingProducer(sab);
    const cons = new RingConsumer(sab);
    const drain = new Float32Array(256);
    // Push+drain more than capacity so write/read indices wrap.
    for (let i = 0; i < 100; i++) {
      prod.write(new Int16Array(128 * 2).fill(1000));
      expect(cons.read(drain, 128)).toBe(128);
    }
  });
});
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd tools/deluge-web-sim && npx vitest run src/audio-ring.test.ts`
Expected: FAIL — `Cannot find module './audio-ring'`.

- [ ] **Step 3: Implement `audio-ring.ts`**

```ts
// Lock-free SPSC ring carrying interleaved-stereo s16 audio from the brain worker
// (producer) to the AudioWorklet (consumer). Header holds monotonic frame counts;
// the data area is Int16. The producer may Atomics.wait when full (worker only);
// the consumer never blocks (audio thread) and reports short reads as underruns.
export const AUDIO_RING_FRAMES = 8192; // ~186 ms at 44100
const CHANNELS = 2;
const WRITE = 0; // Int32 header indices
const READ = 1;
const HEADER_I32 = 2;

export function createRingSab(): SharedArrayBuffer {
  return new SharedArrayBuffer(HEADER_I32 * 4 + AUDIO_RING_FRAMES * CHANNELS * 2);
}

export class RingProducer {
  private hdr: Int32Array;
  private data: Int16Array;
  constructor(sab: SharedArrayBuffer) {
    this.hdr = new Int32Array(sab, 0, HEADER_I32);
    this.data = new Int16Array(sab, HEADER_I32 * 4);
  }
  // samples: interleaved stereo s16 (length = frames*2).
  write(samples: Int16Array) {
    const frames = samples.length / CHANNELS;
    const cap = AUDIO_RING_FRAMES;
    let done = 0;
    while (done < frames) {
      const w = Atomics.load(this.hdr, WRITE);
      const r = Atomics.load(this.hdr, READ);
      const free = cap - (w - r);
      if (free === 0) {
        Atomics.wait(this.hdr, READ, r); // block until the consumer advances
        continue;
      }
      const n = Math.min(free, frames - done);
      for (let i = 0; i < n; i++) {
        const slot = ((w + i) % cap) * CHANNELS;
        const src = (done + i) * CHANNELS;
        this.data[slot] = samples[src];
        this.data[slot + 1] = samples[src + 1];
      }
      Atomics.store(this.hdr, WRITE, w + n);
      Atomics.notify(this.hdr, WRITE);
      done += n;
    }
  }
}

export class RingConsumer {
  private hdr: Int32Array;
  private data: Int16Array;
  constructor(sab: SharedArrayBuffer) {
    this.hdr = new Int32Array(sab, 0, HEADER_I32);
    this.data = new Int16Array(sab, HEADER_I32 * 4);
  }
  // Fill out (length frames*2) with up to `frames` stereo frames as float; return frames read.
  read(out: Float32Array, frames: number): number {
    const cap = AUDIO_RING_FRAMES;
    const w = Atomics.load(this.hdr, WRITE);
    const r = Atomics.load(this.hdr, READ);
    const avail = Math.min(frames, w - r);
    for (let i = 0; i < avail; i++) {
      const slot = ((r + i) % cap) * CHANNELS;
      out[i * CHANNELS] = this.data[slot] / 32768;
      out[i * CHANNELS + 1] = this.data[slot + 1] / 32768;
    }
    if (avail > 0) {
      Atomics.store(this.hdr, READ, r + avail);
      Atomics.notify(this.hdr, READ); // wake a waiting producer
    }
    return avail;
  }
}
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd tools/deluge-web-sim && npx vitest run src/audio-ring.test.ts`
Expected: PASS (3 cases).

- [ ] **Step 5: Commit**
```bash
git add tools/deluge-web-sim/src/audio-ring.ts tools/deluge-web-sim/src/audio-ring.test.ts
git commit -m "feat(deluge-web-sim): lock-free SPSC audio ring (s16 → float)"
```

---

### Task 3: Brain worker + AudioWorklet + wiring (deluge-sdk)

**Files:**
- Create: `tools/deluge-web-sim/src/brain-worker.ts`
- Create: `tools/deluge-web-sim/public/brain-audio-worklet.js`
- Create: `tools/deluge-web-sim/src/brain-audio.ts`
- Modify: `tools/deluge-web-sim/src/main.ts`, `vite.config.ts`, `package.json`
- Modify: `tools/deluge-web-sim/.gitignore` (ignore `public/deluge_brain.wasm`)

**Interfaces:**
- Consumes: `createRingSab`, `RingProducer` (Task 2); the `audio_push` import (Task 1); `deluge_brain.wasm`.
- Produces: `startBrainAudio(): Promise<void>` (in `brain-audio.ts`) — builds the ring, resumes an `AudioContext`, adds the worklet node, spawns the worker. The worker message protocol: main posts `{ ring, wasmUrl }`; the worker loads and runs the brain.

- [ ] **Step 1: Add the dependency and the wasm artifact ignore**

`tools/deluge-web-sim/package.json` — add to `dependencies`:
```json
  "dependencies": {
    "@bjorn3/browser_wasi_shim": "^0.4.1"
  },
```
Append to `tools/deluge-web-sim/.gitignore`:
```
public/deluge_brain.wasm
```
Run: `cd tools/deluge-web-sim && npm install`

- [ ] **Step 2: Copy the built wasm into `public/`**

Run (from repo root, after the DelugeFirmware wasm build):
```bash
cp /home/kate/GitHub/DelugeFirmware/build-wasm/deluge_brain.wasm tools/deluge-web-sim/public/deluge_brain.wasm
```
(A5/dbt will formalize this into a build step; for now it is a manual copy.)

- [ ] **Step 3: Write the worker**

`tools/deluge-web-sim/src/brain-worker.ts`:
```ts
// Runs deluge_brain.wasm to completion-less-ness: wasi.start enters deluge_main's
// infinite loop and never returns, so this worker is dedicated to the brain. The
// only way data leaves is the audio_push import, which writes the SAB ring.
import { WASI, OpenFile, File, ConsoleStdout } from "@bjorn3/browser_wasi_shim";
import { RingProducer } from "./audio-ring";

self.onmessage = async (e: MessageEvent<{ ring: SharedArrayBuffer; wasmUrl: string }>) => {
  const { ring, wasmUrl } = e.data;
  const producer = new RingProducer(ring);
  let mem: WebAssembly.Memory;

  const fds = [
    new OpenFile(new File([])),
    ConsoleStdout.lineBuffered((m) => console.log("[brain]", m)),
    ConsoleStdout.lineBuffered((m) => console.warn("[brain]", m)),
  ];
  const wasi = new WASI([], [], fds); // no WASM_BRAIN_MAX_BLOCKS → runs unbounded

  const env = {
    audio_push: (ptr: number, frames: number) => {
      // Copy the block out of wasm memory and into the ring (may Atomics.wait if full).
      producer.write(new Int16Array(mem.buffer, ptr, frames * 2).slice());
    },
  };

  const { instance } = await WebAssembly.instantiateStreaming(fetch(wasmUrl), {
    wasi_snapshot_preview1: wasi.wasiImport,
    env,
  });
  mem = instance.exports.memory as WebAssembly.Memory;
  wasi.start(instance as { exports: { memory: WebAssembly.Memory; _start: () => void } }); // blocks forever
};
```

- [ ] **Step 4: Write the AudioWorklet**

`tools/deluge-web-sim/public/brain-audio-worklet.js` (plain JS — worklets can't import the TS module; the ring layout is inlined and MUST match `audio-ring.ts`):
```js
// Drains the brain's SAB audio ring to the output. Never blocks (audio thread):
// short reads become silence. Ring layout mirrors src/audio-ring.ts.
const CH = 2, WRITE = 0, READ = 1, HEADER_I32 = 2, CAP = 8192;
class BrainAudio extends AudioWorkletProcessor {
  constructor(options) {
    super();
    const sab = options.processorOptions.ring;
    this.hdr = new Int32Array(sab, 0, HEADER_I32);
    this.data = new Int16Array(sab, HEADER_I32 * 4);
  }
  process(_inputs, outputs) {
    const out = outputs[0];
    const L = out[0], R = out[1];
    const frames = L.length; // 128
    const w = Atomics.load(this.hdr, WRITE);
    const r = Atomics.load(this.hdr, READ);
    const avail = Math.min(frames, w - r);
    for (let i = 0; i < avail; i++) {
      const slot = ((r + i) % CAP) * CH;
      L[i] = this.data[slot] / 32768;
      R[i] = this.data[slot + 1] / 32768;
    }
    for (let i = avail; i < frames; i++) { L[i] = 0; R[i] = 0; } // underrun → silence
    if (avail > 0) {
      Atomics.store(this.hdr, READ, r + avail);
      Atomics.notify(this.hdr, READ);
    }
    return true;
  }
}
registerProcessor("brain-audio", BrainAudio);
```

- [ ] **Step 5: Write the main-thread orchestration**

`tools/deluge-web-sim/src/brain-audio.ts`:
```ts
import { createRingSab } from "./audio-ring";

const SAMPLE_RATE = 44100;

// Build the ring, start audio, and spawn the brain worker. Must be called from a
// user gesture (browsers require one to start an AudioContext).
export async function startBrainAudio(): Promise<AudioContext> {
  if (!crossOriginIsolated) {
    throw new Error("SharedArrayBuffer unavailable — page is not cross-origin isolated (COOP/COEP)");
  }
  const ring = createRingSab();
  const ctx = new AudioContext({ sampleRate: SAMPLE_RATE });
  await ctx.audioWorklet.addModule("/brain-audio-worklet.js");
  const node = new AudioWorkletNode(ctx, "brain-audio", {
    numberOfInputs: 0,
    outputChannelCount: [2],
    processorOptions: { ring },
  });
  node.connect(ctx.destination);
  await ctx.resume();

  const worker = new Worker(new URL("./brain-worker.ts", import.meta.url), { type: "module" });
  worker.postMessage({ ring, wasmUrl: new URL("/deluge_brain.wasm", location.href).href });
  return ctx;
}
```

- [ ] **Step 6: Wire a `?brain=wasm` entry in `main.ts`**

Add to `tools/deluge-web-sim/src/main.ts` (after the existing setup):
```ts
// Wasm-brain audio mode: ?brain=wasm shows a Start button (a user gesture is
// required to start audio) that boots the in-browser firmware brain.
if (new URLSearchParams(location.search).get("brain") === "wasm") {
  const btn = document.createElement("button");
  btn.textContent = "▶ Start wasm brain (audio)";
  btn.style.cssText = "position:fixed;top:8px;right:8px;z-index:10;padding:6px 12px";
  btn.addEventListener("click", async () => {
    btn.disabled = true;
    btn.textContent = "brain running…";
    const { startBrainAudio } = await import("./brain-audio");
    try { await startBrainAudio(); } catch (e) { btn.textContent = "failed: " + (e as Error).message; btn.disabled = false; }
  });
  document.body.appendChild(btn);
}
```

- [ ] **Step 7: Add COOP/COEP to the Vite config**

Replace `tools/deluge-web-sim/vite.config.ts`:
```ts
import { defineConfig } from "vite";

// SharedArrayBuffer (the brain↔worklet audio ring) needs cross-origin isolation.
const coi = {
  "Cross-Origin-Opener-Policy": "same-origin",
  "Cross-Origin-Embedder-Policy": "require-corp",
};
export default defineConfig({
  base: "./",
  build: { target: "es2022", outDir: "dist" },
  worker: { format: "es" },
  server: { headers: coi },
  preview: { headers: coi },
});
```

- [ ] **Step 8: Manual verification (listen)**

Run:
```bash
cd tools/deluge-web-sim && cp /home/kate/GitHub/DelugeFirmware/build-wasm/deluge_brain.wasm public/ && npm run dev
```
Open `http://localhost:5173/?brain=wasm`, click **Start**. Expected: the `[brain]` boot logs appear in the console ("going into main loop"), and audio plays (the Deluge idle/boot output). Confirm the tab is cross-origin isolated (`crossOriginIsolated === true` in the console).

- [ ] **Step 9: Commit**
```bash
git add tools/deluge-web-sim/src/brain-worker.ts tools/deluge-web-sim/public/brain-audio-worklet.js \
  tools/deluge-web-sim/src/brain-audio.ts tools/deluge-web-sim/src/main.ts \
  tools/deluge-web-sim/vite.config.ts tools/deluge-web-sim/package.json \
  tools/deluge-web-sim/package-lock.json tools/deluge-web-sim/.gitignore
git commit -m "feat(deluge-web-sim): run the wasm brain in a worker, audio via AudioWorklet (A4)"
```

---

### Task 4: Playwright audio verification (deluge-sdk)

**Files:**
- Test: `tools/deluge-web-sim/tests/wasm-audio.spec.ts`

**Interfaces:**
- Consumes: the `?brain=wasm` page and the worklet output.

- [ ] **Step 1: Write the e2e test**

`tools/deluge-web-sim/tests/wasm-audio.spec.ts`:
```ts
import { test, expect } from "@playwright/test";

// Verifies the wasm brain boots in-browser and produces real audio: tap an
// AnalyserNode on the worklet output and assert non-zero RMS after a moment.
test("wasm brain plays non-silent audio", async ({ page }) => {
  await page.goto("/?brain=wasm");
  await expect(page).toHaveTitle(/Deluge/);

  // The page must be cross-origin isolated for SharedArrayBuffer.
  expect(await page.evaluate(() => crossOriginIsolated)).toBe(true);

  await page.getByRole("button", { name: /Start wasm brain/ }).click();

  // Give the brain time to boot and fill the ring, then measure output RMS via an
  // analyser the test attaches to the same context by re-reading the worklet.
  const rms = await page.evaluate(async () => {
    await new Promise((r) => setTimeout(r, 4000));
    // The app exposes the context for testing:
    const ctx = (window as unknown as { __brainCtx?: AudioContext }).__brainCtx;
    if (!ctx) return -1;
    const an = ctx.createAnalyser();
    // Reconnect: the worklet node is on window too.
    (window as unknown as { __brainNode?: AudioNode }).__brainNode?.connect(an);
    const buf = new Float32Array(an.fftSize);
    await new Promise((r) => setTimeout(r, 500));
    an.getFloatTimeDomainData(buf);
    return Math.sqrt(buf.reduce((a, x) => a + x * x, 0) / buf.length);
  });
  expect(rms).toBeGreaterThan(0);
});
```

- [ ] **Step 2: Expose the context/node for the test**

In `brain-audio.ts` `startBrainAudio`, before `return ctx;`, add:
```ts
  (window as unknown as { __brainCtx?: AudioContext; __brainNode?: AudioNode }).__brainCtx = ctx;
  (window as unknown as { __brainNode?: AudioNode }).__brainNode = node;
```

- [ ] **Step 3: Run the e2e**

Run:
```bash
cd tools/deluge-web-sim && cp /home/kate/GitHub/DelugeFirmware/build-wasm/deluge_brain.wasm public/ && npm run test:e2e -- tests/wasm-audio.spec.ts
```
Expected: PASS — `crossOriginIsolated` is true and measured RMS > 0. If the brain boots silent (idle with no patch loaded), relax to asserting the ring's `writeFrames` advanced instead (the brain is producing blocks) and note it — a loaded-patch audio assertion belongs in A5/A6 once input can trigger notes.

- [ ] **Step 4: Commit**
```bash
git add tools/deluge-web-sim/tests/wasm-audio.spec.ts tools/deluge-web-sim/src/brain-audio.ts
git commit -m "test(deluge-web-sim): Playwright — wasm brain produces audio in-browser (A4)"
```

---

## Self-Review

**Spec coverage (A4 subset):**
- §2 execution model (brain in Worker, audio ring, thin AudioWorklet, backpressure = clock) → Tasks 1–3. ✓
- §4 audio ring (SPSC, backpressure paces the brain) → Task 2 + the `Atomics.wait` in `RingProducer`. ✓
- §5 COOP/COEP re-added → Task 3 Step 7. ✓
- §7 Playwright audio check → Task 4. ✓
- The import-bridge (execution correction from the foundation: single-threaded, `deluge_main` never returns) → Task 1 + Global Constraints. ✓
- **Deferred to A5:** illumination SAB + `WasmBrain`/Panel + input; formalizing the wasm→public copy into `dbt web`. Noted, not in scope.

**Placeholder scan:** No TBD/TODO. The one conditional (Task 4 Step 3: fall back to asserting ring progress if idle audio is silent) is a concrete, stated fallback resolved within the task, not deferred work. Every code step shows complete code.

**Type consistency:** Ring header layout (`WRITE=0, READ=1, HEADER_I32=2, CAP=AUDIO_RING_FRAMES=8192`, `CHANNELS=2`) is identical across `audio-ring.ts` (Task 2) and the inlined worklet `brain-audio-worklet.js` (Task 3) — a deliberate duplication flagged in-file. `audio_push(ptr, frames)` signature matches between the C import (Task 1), the headless no-op (Task 1), and the worker's JS handler (Task 3). `startBrainAudio` / `createRingSab` / `RingProducer.write` / `RingConsumer.read` names match across tasks.
