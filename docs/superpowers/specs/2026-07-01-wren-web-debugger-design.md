# Wren web editor — in-browser source-level debugger

## Context

The web editor (`tools/wren-web/app`) is an all-client-side Monaco editor +
Deluge "Wren" simulator (wasm). We want a **source-level debugger** in that
editor: line breakpoints, stepping, call stack, and variable inspection for the
running Wren script.

A near-complete DAP server already exists — **`wren-dap`** in the sibling
`wren-rs` repo (`/home/kate/GitHub/wren-rs/crates/wren-dap`). It provides line +
conditional breakpoints, logpoints, step over/into/out, call stack, scopes,
expandable variable inspection (lists/maps/instances), watch/hover evaluate,
exception breakpoints, pause, fibers-as-threads, and multi-file imports — over
DAP's stdio/TCP framing. Its debug **backend** lives in
`wren-core/src/vm/debug.rs` (~2.7k lines) and relies on a `debug`-feature source
patch to the C VM that inserts a `wrenDebugHook(vm, fiber, ip)` call at each
bytecode dispatch site (see `wren-core/build.rs`).

That backend assumes a native process with **two OS threads** sharing
`Arc<Mutex<…>>`/`mpsc`: the VM thread runs `interpret` and **parks** on a
breakpoint; a driver thread issues commands and reads back state. Neither the
threading model nor the stdio/TCP transport fits a pure browser build.

The deluge-sdk simulator uses a **different VM stack** than wren-dap: the sim
wasm (`tools/wren-web`, `wasm32-unknown-unknown`, `no_std`, C-ABI cdylib) is
built on `deluge-wren-core` (prelude, bindings, DSP `Engine`, `Host` trait) +
`wren-sys` (the **stock**, unpatched C VM compiled against a wasi-sdk sysroot).
A Deluge script is a **live synth**: it installs callbacks (audio render, note
on/off, pad/enc/button handlers) that the `Engine` drives — the interesting code
never runs in a plain top-to-bottom pass.

### Decisions locked in brainstorming

1. **In-browser, client-side.** The debugger runs inside the web editor with no
   server. (Native VS Code-via-DAP and a WebSocket→TCP bridge were rejected.)
2. **Web Worker + `SharedArrayBuffer`.** A breakpoint must return control to the
   event loop (keep the UI alive) while a synchronous wasm `interpret` call is on
   the stack. We run the debug VM in a Worker whose hook **blocks** on
   `Atomics.wait`; the main thread signals resume/step and reads state over a
   SAB, using `Atomics.waitAsync` (allowed on the main thread) for responses.
   (Asyncify single-thread suspend/resume was rejected: whole-VM instrumentation
   cost + a bigger departure from wren-dap's threaded backend.)
3. **Harness-driven debug run (load first, then callbacks).** The worker runs
   the script's top level *and* a debug harness that fires a scripted event
   sequence (note on/off, a few pad/enc/button events, N audio render blocks) so
   breakpoints inside callbacks fire. Load-time debugging works first; event
   driving layers on top. (Top-level-only MVP and "break the live main-thread
   sim in place" were rejected.)

### De-risking already in place

The app **already runs cross-origin-isolated with `SharedArrayBuffer`**:
`vite.config.ts` sets `Cross-Origin-Opener-Policy: same-origin` +
`Cross-Origin-Embedder-Policy: require-corp`, and `src/audio.ts` already uses a
SAB command ring to an AudioWorklet (with a postMessage fallback when isolation
is unavailable). The COOP/COEP headers, the SAB pattern, and the ES-module worker
setup (`worker: { format: "es" }`) are all present and proven in this codebase.

## Design

### 1. Architecture & data flow

Three actors. The two "threads" of wren-dap's design become two JS realms:

```
main thread                          debug worker
───────────                          ────────────
Monaco + debug UI                    debug-core.wasm
  │  set breakpoints, step, continue    = deluge-wren-core
  │  request stackTrace/scopes/vars/eval   + wren-sys[debug]  (VM w/ line hook)
  ▼                                        + debug-agent (ported debug.rs)
DebugController (TS)  ◄──── SAB ────►     + harness (drives events)
  await via Atomics.waitAsync          hook blocks on Atomics.wait
```

- The debug worker owns its **own** VM instance, separate from the main-thread
  audio sim, so debugging never perturbs live audio.
- The SAB carries (a) the resume/step signal, so the worker can unblock while
  parked deep inside a synchronous C frame, and (b) inspection request/response
  JSON payloads.
- The main thread never blocks: it uses `Atomics.waitAsync` for responses;
  only the worker uses blocking `Atomics.wait`.

### 2. The debug wasm core (build)

A **second wasm artifact** — the *debug core* — built from the **same**
`deluge-wren-core` + `wren-sys` stack as the sim, so prelude/bindings/`Engine`
behave identically. The main sim wasm is unchanged and stays unpatched (no
per-instruction hook cost); the debug core is fetched only when debugging starts.
Two additions:

- **`wren-sys` gains a `debug` cargo feature** that applies the `wrenDebugHook` +
  error-hook source patches to `wren_vm.c` inside `build.rs`. The patch text is
  ported from the equivalent patches in `wren-core/build.rs` (a global hook
  pointer + `shimSetDebugHook` setter; a `DEBUG_HOOK()` macro emitted after the
  no-trace branch of the trace macro and invoked at both the computed-goto and
  switch dispatch sites; the parallel error hook). Firmware and the plain sim
  build without the feature and are byte-for-byte unaffected.
- A new **`debug-agent`** module: a `no_std` + `alloc` port of the **VM-side** of
  `wren-core/src/vm/debug.rs`. We keep the hard logic — fiber-stack walk, locals
  + module-variable enumeration, value formatting, expandable lists/maps/
  instances, and read-only expression evaluation — and **drop** the
  `std::thread` / `mpsc` / `Arc<Mutex>` plumbing, replacing it with the SAB
  command loop (§3). `std`-only collections (`HashMap`/`HashSet`) become `alloc`
  equivalents (`hashbrown`/`BTreeMap`); `String`/`Vec`/`CStr` are already in
  `alloc`/`core`.

The debug core is a new `cdylib` (either a `wren-web-debug` crate or a feature/
second `[[bin]]`-equivalent target of `tools/wren-web`) exposing a C-ABI surface:
boot, install hook, set breakpoints, launch (compile+run entry under the
harness), and — callable **while parked** — `stack_trace`, `scopes`,
`variables(ref)`, `evaluate(frame, expr)`, plus resume/step controls.

### 3. Concurrency protocol

- **Running** (not parked): normal `postMessage`. Out: `output`, `stopped`,
  `terminated`. In: `setBreakpoints`, `launch`, harness `configure`.
- **On breakpoint**: the C hook calls the agent → the agent emits a `stopped`
  event, then enters a **synchronous SAB command loop** (`Atomics.wait` on a
  command slot). Because the worker is blocked in a C frame it cannot pump its
  message queue, so **while parked everything goes over the SAB**:
  1. Main writes a request (`stackTrace`/`scopes`/`variables`/`evaluate`, JSON in
     the SAB byte region) and `Atomics.notify`s the command slot.
  2. The worker wakes, calls the synchronous inspection export, writes the JSON
     response into the SAB, and `Atomics.notify`s the response slot.
  3. Main was `await`ing that slot via `Atomics.waitAsync`; it resolves and reads
     the response.
  4. A `continue`/`next`/`stepIn`/`stepOut` command **exits** the loop → the hook
     returns → `interpret` resumes.
- **Payload sizing:** DAP-style lazy expansion (`variablesReference` fetched on
  demand) keeps each request/response small, so a fixed SAB byte region (a few
  MB) never overflows. If a single string value would exceed the region it is
  truncated with an ellipsis marker.

### 4. The harness (load → driven callbacks)

The worker's run is not top-to-bottom only. A **debug harness** built on the
`Engine`:

1. Compile + run the entry module. Breakpoints in top-level code / `init` hit
   here. **(Layer 1 — implemented first; a complete, shippable increment.)**
2. Drive a **scripted event sequence** so callback breakpoints fire: note on/off,
   a few pad/enc/button events, and **N audio render blocks**. **(Layer 2.)**

The sequence is configurable from a small "debug drive" panel (note number,
number of render blocks, which events). Layer 1 ships and is verifiable before
Layer 2 lands.

### 5. Reuse map

| Source (wren-rs) | Target (deluge-sdk) | Transformation |
|---|---|---|
| `wren-core/src/vm/debug.rs` (VM-side) | `debug-agent` module | strip threads/channels; `std`→`alloc` collections |
| `wren-core/build.rs` debug patches | `wren-sys` `debug` feature | port patch text into `wren-sys/build.rs` |
| `wren-dap/src/session.rs` | worker message handler | transport becomes SAB/postMessage, not stdio; keep DAP-shaped request→backend→JSON mapping |
| `src/audio.ts` SAB pattern, `vite.config.ts` COOP/COEP, `worker: {format:"es"}` | reused directly | none |

DAP-**shaped** JSON messages (not a literal DAP socket) are kept so
`session.rs` lifts almost verbatim and a VS Code-attach bridge stays possible as
a later follow-up.

### 6. UI

Monaco standalone ships no debugger UI, so we build a small custom one, reusing
existing panel/scrollbar styling:

- Breakpoint gutter with click-to-toggle; breakpoints persist per file.
- A debug toolbar: Debug/Continue, Step Over, Step Into, Step Out, Stop.
- A call-stack panel (frames → click to select, drives scopes/vars).
- A variables tree: **Locals** and **Module** scopes, lazily expandable
  (lists indexed `[0]…`; instance fields by Wren name incl. leading `_`; maps in
  hash-table order).
- Current-line / current-frame highlight in the editor.

### 7. v1 scope

**In v1:** line breakpoints; continue; step over/into/out; call stack; variables
(Locals + Module, expandable); current-line highlight; program output; the
load+harness driven run.

**Deferred follow-ups (each an incremental port from wren-dap, not new
invention):** conditional breakpoints, logpoints, watch/hover evaluate, exception
breakpoints, fibers-as-threads, relative-import path fidelity, and a VS
Code-attach bridge.

## Components & boundaries

- **`wren-sys` `debug` feature** — build-time VM patch only. Interface: the
  extern `shimSetDebugHook`/`wrenDebugHook` symbols. Depends on: upstream
  `wren_vm.c` patch anchors (asserted present at build time, as wren-core does).
- **`debug-agent` (Rust, no_std+alloc)** — owns VM inspection. Interface: the
  C-ABI exports (set breakpoints, launch, resume/step, stack/scopes/vars/eval).
  Depends on: `wren-sys[debug]`, `deluge-wren-core`. Testable headless via a
  small native/`wasm32-wasi` harness that drives a canned script.
- **debug core wasm loader + `DebugController` (TS)** — owns the SAB protocol and
  worker lifecycle. Interface: a typed async API the UI calls
  (`setBreakpoints`, `launch`, `continue`, `step*`, `stackTrace`, `scopes`,
  `variables`, `evaluate`) + an event stream (`stopped`, `output`, `terminated`).
  Depends on: the debug worker, the SAB layout.
- **Debug UI (TS)** — owns gutter/toolbar/panels. Depends on: `DebugController`,
  Monaco, existing styling.
- **Harness (Rust, in the debug core)** — owns the driven event sequence.
  Depends on: `Engine`.

Each unit is understandable and testable in isolation: the agent via a canned
script + assertions on emitted state; the SAB protocol via a fake worker; the UI
via Playwright.

## Error handling

- **Missing SAB / no cross-origin isolation:** the debug feature is **disabled**
  with a clear in-UI message (mirrors `audio.ts`'s isolation fallback). No
  silent degradation — debugging strictly requires SAB.
- **Runtime error in the debugged script:** surfaced as an `output` event and
  program termination (v1); exception-*breakpoints* are a deferred follow-up.
- **Evaluate/inspection errors while parked:** returned as an error payload and
  shown inline (as wren-dap does), never crashing the worker.
- **Build-time patch-anchor drift:** `wren-sys`'s `debug` build asserts each
  patch target is found (as `wren-core/build.rs` does), failing loudly if an
  upstream VM update moves an anchor.
- **SAB payload overflow:** truncate oversized string values; keep structural
  payloads small via lazy expansion.

## Testing

- **debug-agent unit/headless:** a canned multi-file script with a known
  breakpoint; assert stopped line, frame names, a local's formatted value, an
  expandable list child, and step results — run natively or on `wasm32-wasi`
  without the browser.
- **Playwright end-to-end:** set a gutter breakpoint → Debug → assert `stopped`
  at the right line; step over/into/out; read a variable value; expand a
  list/instance; continue to termination. A breakpoint **inside a note handler**
  fires under the harness (Layer 2). A breakpoint in an **imported** module hits
  with the right `source.path`.
- **No regressions:** debug-core wasm + sim wasm build; `tsc --noEmit` +
  `vite build` green; the plain sim still runs single-file and multi-file
  scripts; device `cargo build-wren` + host build stay green (firmware provides
  the null debug hook, as it already does for `wren_host_load_module`).

## Open details to pin during implementation

- **SAB layout:** exact slot offsets (state word, command word, response word,
  byte-region length + bytes) and the JSON framing within the byte region.
- **debug core packaging:** a dedicated `wren-web-debug` crate vs. a feature of
  `tools/wren-web`. Lean: a sibling crate, so the plain sim's build graph never
  pulls the `debug` feature of `wren-sys`.
- **Breakpoint delivery while running:** breakpoints set before `launch` travel
  by postMessage; breakpoints toggled *while parked* travel over the SAB and take
  effect on resume.
- **`hashbrown` vs `BTreeMap`** for the agent's maps (determinism of map-entry
  order vs. matching wren-dap's hash-table order — pick and document).
- **Harness event vocabulary:** the exact set/order of simulated events and how
  the UI configures them.
