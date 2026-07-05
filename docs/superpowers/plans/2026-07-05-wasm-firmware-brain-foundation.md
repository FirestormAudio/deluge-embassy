# Wasm Firmware Brain — Foundation (A1–A3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Compile the DelugeFirmware host application to `wasm32-wasip1` against a new browser-oriented BSP, and prove — headless, under Node's WASI — that it **links, boots to its render loop, and produces audio blocks** without trapping.

**Architecture:** A new `src/bsp/wasm/` implements the `libdeluge` boundary (ported from `src/bsp/host/*`, POSIX swapped for wasm-safe primitives). A `sim-wasm/` CMake toolchain links `deluge_app` + portable libs + the wasm BSP into `deluge_brain.wasm`. A Node WASI harness drives a **bounded** boot: the wasm BSP's audio driver renders N blocks into a buffer, then quiesces, so the firmware's non-returning `while(1)` mainLoop terminates for tests.

**Tech Stack:** clang 22 (`wasm32-wasip1`), `WASI_SYSROOT=/home/kate/.local/wasi-sysroot-25.0` (libc++), CMake, SIMDe, Node `node:wasi`.

**Scope:** This plan is the **foundation** of sub-project A. It delivers a headless booting+rendering wasm brain. The **browser** layers — audio ring + AudioWorklet (A4), illumination SAB + `WasmBrain` + `Panel` (A5), Playwright + audio parity (A6) — are a **follow-on plan** written once A3 reveals the real boot/BSP behavior. WebMIDI (sub-project B) and full storage (sub-project C) are out of scope; `midi_io`/`block_device` are stubbed here.

**Spec:** `docs/superpowers/specs/2026-07-05-wasm-firmware-brain-design.md`

**Repo:** All work is in `/home/kate/GitHub/DelugeFirmware` (the `next` branch). Nothing in this plan touches `deluge-sdk`.

## Global Constraints

- **Toolchain (verbatim from spec §8):** `clang --target=wasm32-wasip1 --sysroot=$WASI_SYSROOT -std=gnu++23 -msimd128 -fexceptions -DSIMDE_FLOAT16_API=1 -DSIMDE_ENABLE_NATIVE_ALIASES`. `-fexceptions` is **required** (firmware uses `try`/`throw`). Single-threaded — do **not** use the `wasm32-wasip1-threads` sysroot.
- **No firmware application changes.** Only `src/bsp/wasm/`, `sim-wasm/`, and the app's existing boundary are touched. If the app references a boundary symbol, the BSP defines it — never edit the app to remove the reference.
- **Boundary contract:** implement the functions declared in `include/libdeluge/*.h`. The `src/bsp/host/*` files are the reference implementation to port from.
- **Bounded boot:** `deluge_main()` never returns (`while(1)`). Tests bound it via the BSP audio driver (`WASM_BRAIN_MAX_BLOCKS` env → after N rendered blocks, call `deluge_system_quiesce()`/`__wasi_proc_exit`).
- **Audio format:** `deluge_app_render` writes Q31 stereo (`DelugeStereoSample{int32 l,r}`); sample rate 44100; block size `HOST_AUDIO_BLOCK_FRAMES` (128).
- **Isolation:** all new build output under `build-wasm/`; nothing overwrites `build-sim*/`.

## File Structure

| File | Responsibility |
| --- | --- |
| `sim-wasm/wasm32-wasip1.toolchain.cmake` | CMake toolchain: compiler, target, sysroot, the Global-Constraints flags |
| `sim-wasm/CMakeLists.txt` | `deluge_brain` wasm target: links `deluge_app` + libs + wasm BSP |
| `src/bsp/wasm/wasm_bsp.c` | Board descriptor + `deluge_board*` init entry points |
| `src/bsp/wasm/wasm_clock.c` | `clock.h`/`system.h`: monotonic time, `deluge_log`, critical sections, reset |
| `src/bsp/wasm/wasm_audio.c` | `audio_io.h`: `deluge_audio_drive` → `deluge_app_render` → block buffer; bounded-exit |
| `src/bsp/wasm/wasm_control.c` | `control_surface.h`: illumination setters → in-memory state; `poll_event` |
| `src/bsp/wasm/wasm_storage.c` | `block_device.h`/`flash.h`/`storage_wait.h`: "no card" stub |
| `src/bsp/wasm/wasm_stubs.c` | Remaining boundary symbols (`midi_io`, `cv_gate`, `encoder_io`, `scheduler`/`worker` glue) as safe stubs |
| `src/bsp/wasm/wasm_main.c` | `main()` → `deluge_platform_init()` + `deluge_main()` (mirrors `host_main.c`) |
| `src/bsp/wasm/wasm_import.h` | Declarations of the JS/WASI imports the BSP calls (`now_ms`, `console_log`) |
| `tools/wasm-brain/run-headless.mjs` | Node WASI harness: instantiate `deluge_brain.wasm`, run bounded boot, capture audio + OLED |
| `tools/wasm-brain/boot.test.mjs` | Node test asserting boot + render gates |

---

### Task 1: Wasm toolchain + a linking one-file probe

**Files:**
- Create: `sim-wasm/wasm32-wasip1.toolchain.cmake`
- Create: `sim-wasm/CMakeLists.txt`
- Create: `src/bsp/wasm/wasm_probe.c`

**Interfaces:**
- Consumes: `$WASI_SYSROOT` from the environment.
- Produces: a CMake toolchain that later tasks reuse; proof the exact spike flags work inside CMake (not just a bare `clang` line).

- [ ] **Step 1: Write the toolchain file**

`sim-wasm/wasm32-wasip1.toolchain.cmake`:
```cmake
# Wasm brain toolchain — the spec §8 spike-proven configuration.
set(CMAKE_SYSTEM_NAME WASI)
set(CMAKE_SYSTEM_PROCESSOR wasm32)

if(NOT DEFINED ENV{WASI_SYSROOT})
  message(FATAL_ERROR "WASI_SYSROOT env var must point at the wasi-sysroot")
endif()
set(WASI_SYSROOT "$ENV{WASI_SYSROOT}")

set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
set(_wasm_target "--target=wasm32-wasip1 --sysroot=${WASI_SYSROOT}")
set(_wasm_defs "-DSIMDE_FLOAT16_API=1 -DSIMDE_ENABLE_NATIVE_ALIASES -msimd128 -fexceptions")
set(CMAKE_C_FLAGS_INIT "${_wasm_target} ${_wasm_defs}")
set(CMAKE_CXX_FLAGS_INIT "${_wasm_target} ${_wasm_defs} -std=gnu++23")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${_wasm_target}")

# We only cross-compile; never try to run wasm during configure checks.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

- [ ] **Step 2: Write a probe source that exercises SIMDe + libc++**

`src/bsp/wasm/wasm_probe.c`:
```c
#include <arm/neon.h>
#include <stdint.h>
// Proves the toolchain compiles SIMDe NEON (→ wasm SIMD) in the CMake context.
int32x4_t wasm_probe_add(int32x4_t a, int32x4_t b) { return vaddq_s32(a, b); }
```

- [ ] **Step 3: Write a minimal CMakeLists building just the probe**

`sim-wasm/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.25)
project(deluge_brain C CXX)

# SIMDe lives in a sibling build's _deps; locate any checked-out copy.
file(GLOB _simde_dirs "${CMAKE_SOURCE_DIR}/../build-sim*/_deps/simde-src/simde")
list(GET _simde_dirs 0 SIMDE_INCLUDE)
if(NOT SIMDE_INCLUDE)
  message(FATAL_ERROR "SIMDe not found — configure a host sim build first to fetch _deps")
endif()

add_library(wasm_probe STATIC ${CMAKE_SOURCE_DIR}/../src/bsp/wasm/wasm_probe.c)
target_include_directories(wasm_probe PRIVATE ${SIMDE_INCLUDE})
```

- [ ] **Step 4: Configure + build the probe for wasm**

Run:
```bash
cd /home/kate/GitHub/DelugeFirmware
cmake -B build-wasm -S sim-wasm -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=sim-wasm/wasm32-wasip1.toolchain.cmake
cmake --build build-wasm --target wasm_probe
```
Expected: configures without trying to run wasm; `libwasm_probe.a` is produced. If SIMDe isn't found, first run any existing `cmake -B build-sim -S sim` once to populate `_deps`.

- [ ] **Step 5: Commit**
```bash
git add sim-wasm/ src/bsp/wasm/wasm_probe.c
git commit -m "build(wasm): wasm32-wasip1 toolchain + SIMDe probe"
```

---

### Task 2: Wasm BSP skeleton — link `deluge_brain.wasm`

This is the **link gate**: get every `libdeluge` boundary symbol the app references defined, so `deluge_app` + libs + the wasm BSP link into one module. Bring-up is iterative — the "test" is a successful link with zero undefined symbols. Port the core services from `src/bsp/host/*`; stub the rest.

**Files:**
- Create: `src/bsp/wasm/wasm_import.h`, `wasm_bsp.c`, `wasm_clock.c`, `wasm_audio.c`, `wasm_control.c`, `wasm_storage.c`, `wasm_stubs.c`, `wasm_main.c`
- Modify: `sim-wasm/CMakeLists.txt` (add the `deluge_brain` executable target)

**Interfaces:**
- Consumes: the `libdeluge` boundary headers (`include/libdeluge/*.h`); the app libraries `deluge_app`, `deluge_scheduler`, and the allocator (same targets `sim/CMakeLists.txt` links — reuse via `add_subdirectory(${REPO_ROOT}/src/deluge ...)`).
- Produces: `build-wasm/deluge_brain.wasm`. BSP entry `void deluge_platform_init(void)` and `int main(void)`. Board descriptor from `board.h` fields. `deluge_audio_drive` render path. Illumination state readable later by A5 (byte formats per spec §4).

- [ ] **Step 1: Declare the JS/WASI imports the BSP needs**

`src/bsp/wasm/wasm_import.h`:
```c
#ifndef WASM_IMPORT_H
#define WASM_IMPORT_H
#include <stdint.h>
// Imported from the host (Node WASI harness / browser worker). Under wasip1 these
// resolve via the module's import object; the harness supplies them.
__attribute__((import_module("env"), import_name("now_ms"))) double wasm_now_ms(void);
#endif
```
(`deluge_log` uses `fputs(stderr)` — wasi provides stdio — so no extra import is needed for logging.)

- [ ] **Step 2: Board descriptor + init entry points**

`src/bsp/wasm/wasm_bsp.c` — implement `board.h`, ported from `src/bsp/host/host_bsp.c`. Provide a `static const DelugeBoard` with the 144-pad OLED values (`pad_grid_width=16, pad_grid_height=8, encoder_count=6, cv_channels=2, gate_channels=4, display_width=128, display_height=48, audio_sample_rate_hz=44100, audio_in_channels=2, audio_out_channels=2`; copy exact values from `host_bsp.c`'s board). Implement `deluge_board()`, `deluge_board_probe_oled()`→`true`, and `deluge_board_init_early/_init_audio/_init_storage/_unlock_data_cache` as calls into the other wasm BSP units. Add `void deluge_platform_init(void)` that runs the same init sequence `host_platform.c` does (board → clock → control → storage → audio), reading `src/bsp/host/host_platform.c` for the order.

- [ ] **Step 3: Clock + system**

`src/bsp/wasm/wasm_clock.c` — implement `clock.h` + `system.h`, ported from `src/bsp/host/host_platform.c`/`host_debug.c`:
- `deluge_clock_now`/`deluge_clock_monotonic` from `wasm_now_ms()` (ms → the header's tick unit; mirror the host's `ticks_per_second`).
- `deluge_clock_delay_us/ms`: busy-spin on `wasm_now_ms()` (no `usleep` under wasi single-thread) — bounded, since the firmware only delays microseconds at boot.
- `deluge_log`: `fputs(text, stderr)`.
- `ENTER/EXIT_CRITICAL_SECTION`, `deluge_in_interrupt`→`false`, `deluge_system_reset`→`__builtin_trap()`, `deluge_system_quiesce`→set a flag the audio driver checks.

- [ ] **Step 4: Audio driver (render → buffer, bounded exit)**

`src/bsp/wasm/wasm_audio.c` — implement `audio_io.h`, ported from `src/bsp/host/host_audio.c` (live path), with the native player replaced by an in-memory latest-block buffer + a block counter:
```c
#include "libdeluge/audio_io.h"
#include "libdeluge/app.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define WASM_BLOCK_FRAMES 128u
static DelugeStereoSample in_block[WASM_BLOCK_FRAMES];
static DelugeStereoSample out_block[WASM_BLOCK_FRAMES];
int32_t  wasm_audio_last[WASM_BLOCK_FRAMES * 2]; // Q31 interleaved; read by the harness
uint32_t wasm_audio_blocks = 0;
static uint32_t max_blocks = 0; // 0 = unbounded (browser); >0 = bounded (headless test)

void deluge_audio_start(void) { const char* m = getenv("WASM_BRAIN_MAX_BLOCKS"); max_blocks = m ? (uint32_t)strtoul(m, 0, 10) : 0; }
void deluge_audio_stop(void) {}
uint32_t deluge_audio_max_block_frames(void) { return WASM_BLOCK_FRAMES; }
uint32_t deluge_audio_sample_rate(void) { return 44100u; }
uint32_t deluge_audio_output_latency_frames(void) { return WASM_BLOCK_FRAMES; }

uint32_t deluge_audio_drive(void) {
  memset(in_block, 0, sizeof(in_block));
  deluge_app_render(in_block, out_block, WASM_BLOCK_FRAMES);
  for (uint32_t i = 0; i < WASM_BLOCK_FRAMES; i++) { wasm_audio_last[2*i] = out_block[i].l; wasm_audio_last[2*i+1] = out_block[i].r; }
  wasm_audio_blocks++;
  if (max_blocks && wasm_audio_blocks >= max_blocks) { extern void deluge_system_quiesce(void); deluge_system_quiesce(); __builtin_trap(); }
  return WASM_BLOCK_FRAMES;
}
```
Implement the remaining `audio_io.h` timing helpers (`frames_until_block_offset`, `stamp_to_render_offset`, `input_resync`) as the host build does (copy from `host_audio.c`; they are frame-count arithmetic). In A4 the browser build replaces the buffer with the SAB ring; the render path is unchanged.

- [ ] **Step 5: Control surface (illumination → in-memory state)**

`src/bsp/wasm/wasm_control.c` — implement `control_surface.h`. Keep in-memory state in the spec §4 byte formats so A5 can expose them: OLED 768B page-major, pad RGB 432B col-major (`(x*8+y)*3`), LED array (`deluge_control_set_led`), knob indicators (`deluge_control_set_indicator`). Implement `deluge_control_set_pad`, `deluge_control_set_pad_columns`, the `scroll_*`/`flash_pad` helpers (write into the pad state), `deluge_control_flush`/`wait_for_flush`/`setup_for_pads`/`enable_oled` as no-ops or state commits, `deluge_control_read_boot_info` (`oled_present=true`, others 0/false), and `deluge_control_poll_event` returning `false` for now (input is A5). Export the state buffers (non-`static`) for the harness/A5:
```c
uint8_t wasm_oled[768];        // page-major
uint8_t wasm_pads[432];        // col-major RGB
uint8_t wasm_leds[256];
```
Reference `src/bsp/host/host_link.c`'s `ToDeluge` handling for how each setter maps to bytes.

- [ ] **Step 6: Storage stub (no card) + remaining stubs**

`src/bsp/wasm/wasm_storage.c` — implement `block_device.h`/`flash.h`/`storage_wait.h` as "no card present": `deluge_block_ready`→`false`, `deluge_block_sector_count/size`→0, `deluge_block_sd_unit`→0, card-event pollers→"no event". (Task 3 discovers whether boot tolerates this.)

`src/bsp/wasm/wasm_stubs.c` — define every remaining boundary symbol as a safe stub: `midi_io.h` (`deluge_midi_port_count`→0, reads→0, writes→consume), `cv_gate.h` (no-ops), `encoder_io.h` (no-ops), and any `scheduler.h`/`worker.h`/`signals.h` hooks the app needs that the host build provides — read `src/bsp/host/*` to enumerate. The link errors in Step 8 are the checklist of what's missing.

- [ ] **Step 7: BSP `main`**

`src/bsp/wasm/wasm_main.c`:
```c
#include "libdeluge/board.h"
#include "libdeluge/system.h"
extern void deluge_platform_init(void);
extern void deluge_main(void); // legacy cooperative boot; never returns (bounded by wasm_audio)
int main(void) {
  deluge_platform_init();
  deluge_log("wasm brain: boundary up — entering deluge_main()\n");
  deluge_main();
  return 0; // unreachable
}
```

- [ ] **Step 8: Add the `deluge_brain` target and iterate to a clean link**

In `sim-wasm/CMakeLists.txt`, `add_subdirectory` the app (mirror `sim/CMakeLists.txt` lines that add `src/deluge`, the scheduler, and the allocator), add the ETL include, then:
```cmake
add_executable(deluge_brain
  ${CMAKE_SOURCE_DIR}/../src/bsp/wasm/wasm_bsp.c
  ${CMAKE_SOURCE_DIR}/../src/bsp/wasm/wasm_clock.c
  ${CMAKE_SOURCE_DIR}/../src/bsp/wasm/wasm_audio.c
  ${CMAKE_SOURCE_DIR}/../src/bsp/wasm/wasm_control.c
  ${CMAKE_SOURCE_DIR}/../src/bsp/wasm/wasm_storage.c
  ${CMAKE_SOURCE_DIR}/../src/bsp/wasm/wasm_stubs.c
  ${CMAKE_SOURCE_DIR}/../src/bsp/wasm/wasm_main.c)
target_include_directories(deluge_brain PRIVATE
  ${REPO_ROOT}/include ${REPO_ROOT}/src ${SIMDE_INCLUDE} ${ETL_INCLUDE})
target_link_libraries(deluge_brain PRIVATE deluge_app deluge_scheduler ${SIM_ALLOC_LINK})
# Export the harness-visible symbols + main.
target_link_options(deluge_brain PRIVATE
  -Wl,--export=wasm_audio_last -Wl,--export=wasm_audio_blocks
  -Wl,--export=wasm_oled -Wl,--export=wasm_pads -Wl,--export=wasm_leds)
```

Run, then **iterate**: each `undefined symbol` at link time names a boundary function to add to `wasm_stubs.c` (or port properly). Repeat build until zero undefined.
```bash
cmake -B build-wasm -S sim-wasm -G Ninja -DCMAKE_TOOLCHAIN_FILE=sim-wasm/wasm32-wasip1.toolchain.cmake
cmake --build build-wasm --target deluge_brain 2>&1 | grep -iE "undefined|error" | sort -u
```
Expected end state: `build-wasm/deluge_brain.wasm` exists; the grep prints nothing. **Log any symbol you had to stub in the commit message** (visibility into what A-follow-on/B/C must make real).

- [ ] **Step 9: Commit**
```bash
git add sim-wasm/CMakeLists.txt src/bsp/wasm/
git commit -m "feat(wasm-bsp): link deluge_brain.wasm against a browser BSP skeleton

Ports clock/board/audio/control from src/bsp/host; stubs midi/cv/storage.
Stubbed-to-link symbols: <list>."
```

---

### Task 3: Headless boot + render (Node WASI) — A1 spike + A3 gate

The **boot gate**: prove the brain boots to its render loop and produces audio, and **answer A1** (does it boot without an SD card?). Uses Node's built-in WASI.

**Files:**
- Create: `tools/wasm-brain/run-headless.mjs`
- Test: `tools/wasm-brain/boot.test.mjs`

**Interfaces:**
- Consumes: `build-wasm/deluge_brain.wasm` and its exports (`wasm_audio_last`, `wasm_audio_blocks`, `wasm_oled`, `wasm_pads`, `wasm_leds`); the import `env.now_ms`.
- Produces: a reusable headless runner + a pass/fail boot test. Its findings resolve spec §6 (storage) and shape the follow-on A4–A6 plan.

- [ ] **Step 1: Write the headless runner**

`tools/wasm-brain/run-headless.mjs`:
```js
import { readFile } from "node:fs/promises";
import { WASI } from "node:wasi";

// Run deluge_brain.wasm bounded to N render blocks; return captured audio + OLED.
export async function runHeadless(wasmPath, maxBlocks = 8) {
  const wasi = new WASI({
    version: "preview1",
    env: { WASM_BRAIN_MAX_BLOCKS: String(maxBlocks) },
    returnOnExit: true,
  });
  const bytes = await readFile(wasmPath);
  const mod = await WebAssembly.compile(bytes);
  const t0 = performance.now();
  const imports = {
    ...wasi.getImportObject(),
    env: { now_ms: () => performance.now() - t0 },
  };
  const inst = await WebAssembly.instantiate(mod, imports);
  let exitCode = 0;
  try { wasi.start(inst); } catch (e) { exitCode = e?.code ?? -1; } // trap = bounded exit
  const ex = inst.exports;
  const mem = ex.memory.buffer;
  const view = (sym, len, Ctor = Uint8Array) => new Ctor(mem, ex[sym].value, len);
  const blocks = new Uint32Array(mem, ex.wasm_audio_blocks.value, 1)[0];
  const audio = new Int32Array(mem, ex.wasm_audio_last.value, 256).slice();
  const oled = view("wasm_oled", 768).slice();
  return { exitCode, blocks, audio, oled };
}
```
(`wasm_audio_blocks`/`wasm_audio_last` are exported globals/data; if the linker exports them as globals rather than addresses, read via `ex.wasm_audio_blocks.value` as the pointer — adjust to whichever the build produces, confirmed in Step 2.)

- [ ] **Step 2: First manual run — observe boot depth (this is the A1 spike)**

Run:
```bash
cd /home/kate/GitHub/DelugeFirmware
node --experimental-wasi-unstable-preview1 -e "import('./tools/wasm-brain/run-headless.mjs').then(m=>m.runHeadless('build-wasm/deluge_brain.wasm',8)).then(r=>console.log(JSON.stringify({exit:r.exitCode,blocks:r.blocks,audioNonZero:r.audio.some(x=>x!==0),oledNonZero:r.oled.some(x=>x!==0)})))"
```
Interpret:
- `blocks >= 8` → **boots and renders without a card. A1 resolved: storage entirely deferred to sub-project C.** Proceed.
- Boot hangs / traps before rendering (`blocks == 0`) → read stderr for where `deluge_main` stalls. If it's a storage wait, implement the **fallback**: bake a tiny in-memory FAT image into `wasm_storage.c` (a `static const uint8_t[]` card image; `deluge_block_ready`→true, sector reads served from it). Re-run. Record the outcome. This is the one place the plan branches on a discovered fact — resolve it here, in this task.

- [ ] **Step 3: Write the boot test**

`tools/wasm-brain/boot.test.mjs` (Node's built-in test runner):
```js
import { test } from "node:test";
import assert from "node:assert/strict";
import { runHeadless } from "./run-headless.mjs";

test("wasm brain boots and renders audio blocks", async () => {
  const r = await runHeadless(new URL("../../build-wasm/deluge_brain.wasm", import.meta.url).pathname, 8);
  assert.equal(r.blocks, 8, "should render exactly the bounded block count");
  // Boot reached the render loop; the OLED framebuffer was written (non-blank boot screen).
  assert.ok(r.oled.some((b) => b !== 0), "OLED should have content after boot");
});
```

- [ ] **Step 4: Run the test**

Run:
```bash
cd /home/kate/GitHub/DelugeFirmware
node --experimental-wasi-unstable-preview1 --test tools/wasm-brain/boot.test.mjs
```
Expected: PASS — 8 blocks rendered, OLED non-blank. (If the app boots silent/blank without a card but still reaches the loop, relax the OLED assertion to `r.blocks === 8` only and note it; the audio-content assertion belongs in A4/A6 with a note-on script.)

- [ ] **Step 5: Commit**
```bash
git add tools/wasm-brain/
git commit -m "test(wasm-brain): headless Node WASI boot+render harness (A1/A3)

A1 result: <boots without card | needs in-memory FAT image, added>."
```

---

## Self-Review

**Spec coverage (foundation subset):**
- §2 toolchain (wasm32-wasip1, single-threaded, flags) → Task 1 toolchain file + Global Constraints. ✓
- §3.1 wasm BSP (board/clock/control/display/audio/alloc; midi/cv/storage stubbed) → Task 2 Steps 2–6. ✓
- §3.2 build (`sim-wasm` CMake, links `deluge_app`) → Task 1 + Task 2 Step 8. ✓
- §4 illumination byte formats (OLED 768 page-major, pad 432 col-major, LED) → Task 2 Step 5. ✓
- §6 boot-without-SD gating spike → Task 3 Step 2 (with the in-memory-FAT fallback). ✓
- §7 headless Node WASI CI gate → Task 3. ✓
- §8 spike flags → Global Constraints + Task 1. ✓
- §9 milestones A1 (Task 3 Step 2), A2 (Tasks 1–2), A3 (Task 3). ✓
- **Deferred (not this plan, by design):** A4 audio ring/worklet, A5 illumination SAB + `WasmBrain` + `Panel`, A6 Playwright/parity, COOP/COEP — all in the browser and depend on Task 3's outcome; follow-on plan.

**Placeholder scan:** The one branch (Task 3 Step 2: boots-without-card vs. needs-FAT-image) is a genuine gating discovery resolved *within* the task, not a deferred TODO. Task 2's iterative link loop is inherent to build bring-up; its success gate (zero undefined symbols) is concrete. No "TBD"/"handle edge cases" left.

**Type consistency:** Export names (`wasm_audio_last`, `wasm_audio_blocks`, `wasm_oled`, `wasm_pads`, `wasm_leds`), `WASM_BLOCK_FRAMES=128`, sample rate 44100, and the `WASM_BRAIN_MAX_BLOCKS` env contract are identical across the audio driver (Task 2 Step 4), the linker exports (Task 2 Step 8), and the harness (Task 3 Steps 1–3).
