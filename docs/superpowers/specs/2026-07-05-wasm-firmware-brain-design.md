# Wasm firmware brain + audio (Sub-project A) — design & spec

Run the DelugeFirmware host build as a WebAssembly module in the browser: a
single-threaded firmware "brain" hosted in a Web Worker, clocked by an audio ring
drained by an AudioWorklet, driving the existing `tools/deluge-web-sim` faceplate
in-process — no WebSocket, no native C process. This makes the web simulator
**audible and playable** entirely client-side and delivers the "audio" half of the
WebMIDI/audio goal (audio is intrinsic to running the brain — `deluge_app_render`
*is* the clock).

> **Status:** design proposal. De-risked by a compile/boot spike (§8): a real
> Deluge DSP source compiles cleanly to `wasm32` with SIMDe lowering NEON to wasm
> SIMD128. No language/target/SIMD incompatibility exists.

This is **Sub-project A** of a three-part cluster:
- **A (this spec)** — wasm brain + audio.
- **B** — WebMIDI (browser MIDI devices ↔ brain). Separate spec.
- **C** — Storage/SD (browser-backed FatFS via OPFS/IndexedDB). Separate spec.

---

## 1. Goals & non-goals

**Goals**

- Compile `deluge_app` + portable libs + a new **wasm BSP** to `wasm32-wasip1`,
  producing `deluge_brain.wasm`.
- Run the firmware's cooperative `deluge_main` loop in a Web Worker; produce
  glitch-free audio through Web Audio; render the existing faceplate (`Panel`)
  from the brain's illumination; deliver on-screen input to the brain.
- Reuse `tools/deluge-web-sim`'s `Panel` unchanged, behind a new `WasmBrain`
  adapter that exposes the same reader/input surface the WS `PanelClient` does.
- No changes to the firmware **application** — only a new platform/BSP layer under
  the existing `libdeluge` boundary.

**Non-goals (deferred)**

- **WebMIDI** — sub-project B. `midi_io` is stubbed here.
- **Full SD/storage** — sub-project C. A provides only enough (a stub or a tiny
  in-memory FAT image) to boot; `block_device`/`flash` are minimal.
- **CV/gate visualization**, and the existing WS/external-brain path (kept as a
  selectable alternative, not removed).
- Bit-exact audio parity as a hard requirement (SIMDe is bit-matched, so we
  *check* parity, but small divergence isn't a blocker).

**Success criteria**

In the browser, selecting the wasm brain boots the firmware, plays audio through
the speakers, renders live OLED/pads/LEDs on the faceplate, and responds to
on-screen pad/button/encoder input — with no external process and no audio
glitches under normal UI load.

---

## 2. Architecture

```
┌─ main thread ────────────┐   ┌─ Web Worker ───────────────┐   ┌─ AudioWorklet ─┐
│ Panel (existing)         │   │ deluge_brain.wasm          │   │ ring-drain     │
│  ⇄ WasmBrain adapter     │   │  deluge_main() coop loop   │   │  → speakers    │
│  reads illumination SAB ◄─┼───┤  wasm BSP (libdeluge)      │   │                │
│  writes input SAB ───────┼──►│  render → [SAB audio ring] ─┼──►│ drain ring     │
└──────────────────────────┘   └────────────────────────────┘   └────────────────┘
     rAF: poll + paint            ring backpressure = audio clock
```

- **Toolchain:** `clang --target=wasm32-wasip1 --sysroot=$WASI_SYSROOT`,
  single-threaded (the firmware is cooperative — no OS threads; see §8). Browser
  runs the module via a JS WASI shim (`@bjorn3/browser_wasi_shim`, the same one
  `tools/wren-web/app` uses). Node's built-in WASI runs it headless in CI.
- **Execution model:** one wasm instance in a **Web Worker** running the
  cooperative loop. The worker produces audio blocks into a SAB ring; a thin
  **AudioWorklet** drains the ring to the output. The ring's backpressure paces
  the worker at the audio rate, preserving the firmware's "audio is the clock"
  design while keeping heavy UI ticks off the audio thread (glitch-resistant).
- **No firmware app changes:** the wasm BSP implements the `libdeluge` boundary;
  `deluge_app` is compiled as-is.

---

## 3. Components

### 3.1 wasm BSP — `src/bsp/wasm/` (DelugeFirmware, new)

Implements the `libdeluge` outbound boundary against browser primitives, ported
from the POSIX `src/bsp/host/*` reference:

| Boundary header | A implementation |
| --- | --- |
| `board.h` | Board descriptor; `init_early`/`init_audio`/`init_storage` wired to wasm services; `probe_oled` → true. |
| `clock.h`, `system.h` | Monotonic time from a wasm import (JS `performance.now`-fed counter) or the audio frame count. |
| `control_surface.h` | Illumination setters write the **illumination SAB**; `deluge_control_poll_event` drains the **input SAB**; `deluge_control_read_boot_info`. |
| `display.h` | OLED framebuffer → illumination SAB (768B page-major, as on the wire). |
| `audio_io.h` | `deluge_audio_drive` renders a block via `deluge_app_render` into the **audio ring**; `sample_rate` = 44100; `max_block_frames` = 128. |
| `alloc.h`, `memory.h` | Heaps over a wasm linear-memory arena (port of the host allocator). |
| `scheduler.h`, `worker.h`, `signals.h` | Cooperative scheduling hooks (host port). |
| `midi_io.h` | **Stub** (0 ports / no-op) — sub-project B. |
| `cv_gate.h`, `encoder_io.h` | CV/gate stub; encoders arrive via `control_surface` input events. |
| `block_device.h`, `flash.h`, `storage_wait.h` | **Minimal** — "no card" stub or a tiny in-memory FAT image (§6). Full = sub-project C. |

### 3.2 Build — `sim-wasm/` (DelugeFirmware, new)

A CMake toolchain file (`wasm32-wasip1`) carrying the spike-proven flags (§8),
plus a target that links `deluge_app` + `deluge_scheduler` + the allocator + the
wasm BSP into `deluge_brain.wasm`. A build script copies the artifact to
`tools/deluge-web-sim/public/deluge_brain.wasm`. Reuses the existing SIMDe/ETL/NE10
dependency wiring from `sim/CMakeLists.txt`.

### 3.3 Browser glue — `tools/deluge-web-sim/src/` (new)

- `brain-worker.ts` — Web Worker: instantiates `deluge_brain.wasm` with the WASI
  shim, runs the cooperative loop, owns the audio-ring producer, illumination-SAB
  writer, input-SAB drainer.
- `audio-worklet.ts` — `AudioWorkletProcessor`: drains the audio ring into
  `process()`'s output. Registered on a 44100-Hz `AudioContext`.
- `wasm-brain.ts` — `class WasmBrain` implementing the **same surface** the
  `Panel` consumes from `PanelClient`: readers `oled()/leds()/padRgb()/knob()` (a
  seqlock read of the illumination SAB) and input `pad()/button()/enc()` (enqueue
  to the input SAB). So `Panel` renders it **unchanged**.
- `main.ts` — a brain-source toggle: **WS** (existing external C brain) or
  **wasm** (this). Wasm mode boots the worker + worklet and constructs
  `WasmBrain` instead of `Connection`/`PanelClient`.

---

## 4. Data flow / SAB layouts

- **Audio ring** — SPSC float32 interleaved-stereo ring (worker producer, worklet
  consumer). Header = write/read counts (`Atomics`); capacity sized for a few
  audio quanta of latency. Backpressure (producer waits when full) is what paces
  the brain at the audio rate.
- **Illumination snapshot** — a fixed SAB region: OLED (768B page-major) + pad RGB
  (432B col-major) + LED states (`LED_COUNT`) + knob indicators. Worker writes
  under a **seqlock** (bump a counter before/after); the main thread retries the
  read if the counter is odd or changed, for tear-free frames.
- **Input queue** — SAB ring, main producer → worker drains into
  `deluge_control_poll_event`. Events reuse the existing native ids (pad col/row,
  raw button id, encoder index/delta).

The illumination byte formats are identical to the `deluge-protocol` ones the
`Panel` already consumes (OLED page-major/5-row bezel, pad col-major, LED index ==
raw id), so the `WasmBrain` readers and the WS `PanelClient` readers agree.

---

## 5. Cross-origin isolation

`SharedArrayBuffer` requires the page to be cross-origin isolated. Sub-project A
re-adds the COOP/COEP headers to `tools/deluge-web-sim`'s Vite config (they were
intentionally omitted for the audio-less M0–M2). The app degrades gracefully: if
isolation is unavailable, wasm-brain mode is disabled and WS mode still works.

---

## 6. Boot & storage — the main remaining risk

`deluge_main()` may require a block device to boot. **A's first task (A1) is a
boot mini-spike:** stub `block_device`/`flash` to "no card" and run `deluge_main`
under Node WASI to see how far it gets. Two outcomes:

- **Boots to a usable default state without a card** → keep the stub for A;
  storage is entirely sub-project C.
- **Requires a card** → mount a tiny in-memory FAT image (a minimal default song /
  empty card) baked into the wasm or loaded from an asset — just enough to boot.
  Full read/write SD remains sub-project C.

A1 is a gating task: its result shapes 3.1's storage row and A2's BSP surface.

---

## 7. Testing & verification

- **Headless (Node WASI) — the CI gate.** Boot `deluge_brain.wasm`, pump N
  ticks/renders, assert: reaches the main loop, produces **non-silent** audio for
  a scripted note-on, and writes a **plausible OLED snapshot**. No browser needed.
- **Audio parity.** Render an identical input script through both the native host
  build (its WAV output) and the wasm brain; compare blocks. SIMDe is bit-matched,
  so expect near-identical; report divergence rather than hard-fail.
- **Playwright.** Load deluge-web-sim in wasm-brain mode: assert the
  `AudioContext` reaches `running` with non-zero ring output, the `Panel` renders
  live illumination, and on-screen pad/button/encoder input changes the OLED/LEDs.
  This **also closes the browser-vs-real-brain gap** left open in the WS work
  (M2 used a mock socket) by exercising a real brain end-to-end in the browser.
- **Glitch check.** Under induced UI load, assert no ring underruns over a window
  (the worker-in-front-of-a-ring model should hold).

---

## 8. Spike findings (de-risking, already run)

A compile/boot spike retired the fundamental risk. Confirmed:

- **Toolchain present:** system `clang` (v22) targets `wasm32`; `WASI_SYSROOT`
  (`/home/kate/.local/wasi-sysroot-25.0`) ships `libc++`, `libc++abi`, and a
  `wasm32-wasip1-threads` variant (unused — the firmware is single-threaded).
- **SIMDe → wasm SIMD:** `vaddq_s32` lowers to `v128` instructions.
- **Real firmware C++ compiles:** `src/deluge/modulation/envelope.cpp` (pulling in
  voice/filter/storage/memory/util headers) compiles to a clean `wasm32` object.
- **Execution model:** the host BSP has **no** `pthread_create`/`std::thread`
  (the lone `fork()` is the native audio-output player, replaced by Web Audio);
  `deluge_main` is one cooperative `mainLoop` `while(1)`. Hence single-threaded.

**Proven compile flags** (basis for the `sim-wasm` toolchain file):

```
clang --target=wasm32-wasip1 --sysroot=$WASI_SYSROOT \
  -std=gnu++23 -msimd128 -fexceptions \
  -DSIMDE_FLOAT16_API=1 -DSIMDE_ENABLE_NATIVE_ALIASES \
  -I<simde> -I<etl> <deluge-include-set>
```

Notes: `-fexceptions` is **required** (firmware uses `try`/`throw`);
`SIMDE_FLOAT16_API=1` selects portable half-float (`_Float16` is unsupported on
wasm32); `SIMDE_ENABLE_NATIVE_ALIASES` exposes `int32x4_t` etc.

---

## 9. Milestones

- **A1 — Boot mini-spike.** Stub storage, run `deluge_main` under Node WASI,
  determine boot-without-SD. Gates the storage surface. *(highest remaining risk)*
- **A2 — wasm BSP + build.** `src/bsp/wasm/` + `sim-wasm/` CMake toolchain →
  `deluge_brain.wasm` links against `deluge_app`.
- **A3 — Worker boot.** `brain-worker.ts` + WASI shim; headless Node WASI test:
  boots, ticks, renders a block without trapping.
- **A4 — Audio.** Audio-ring producer + AudioWorklet drain; audible in the browser.
- **A5 — Faceplate + input.** Illumination SAB + `WasmBrain`; `Panel` renders live;
  on-screen input reaches the brain.
- **A6 — Verify.** Playwright wasm-brain e2e + audio-parity check; COOP/COEP.

---

## 10. Cross-repo note

- **DelugeFirmware** (`/home/kate/GitHub/DelugeFirmware`): the wasm BSP
  (`src/bsp/wasm/`) and build (`sim-wasm/`). A1–A2 live here.
- **deluge-sdk** (this repo): the browser glue in `tools/deluge-web-sim/` and the
  `deluge_brain.wasm` artifact under `public/`. A3–A6 live here.
