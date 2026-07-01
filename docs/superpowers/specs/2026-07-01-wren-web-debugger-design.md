# Wren web editor — in-browser source-level debugger

## Context

The web editor (`tools/wren-web/app`) is an all-client-side Monaco editor +
Deluge "Wren" simulator (wasm). We want a **source-level debugger** in that
editor: line breakpoints, stepping, call stack, and variable inspection for the
running Wren script.

A near-complete DAP server already exists — **`wren-dap`** in the sibling
`wren-rs` repo (`/home/kate/GitHub/wren-rs/crates/wren-dap`). It provides line +
conditional breakpoints, logpoints, step over/into/out, call stack, scopes,
expandable variable inspection (lists/maps/instances by name), watch/hover
evaluate, exception breakpoints, pause, fibers-as-threads, and multi-file
imports — over DAP's stdio/TCP framing. Its debug **backend** lives in
`wren-core/src/vm/debug.rs` (~2.7k lines): a `debug`-feature source patch to the
C VM inserts a `wrenDebugHook(vm, fiber, ip)` call at each bytecode dispatch site
(see `wren-core/build.rs`); the backend then walks the parked fiber to serve
stack/scopes/variables/evaluate.

`wren-dap`/`wren-core` get **named** locals and instance fields because
`wren-core` compiles source with its **own Rust compiler** (`compiler/emit.rs`
emits `DebugLocal`; `codegen_class.rs` emits `DebugClass`) and serializes the
resulting bytecode into the upstream **C VM** for execution. `wren-core` does
**not** compile `wren_compiler.c` at all — it is *Rust frontend + C VM runtime*.

### The two VM stacks (why this matters)

- **deluge-sdk (device + web sim)** compiles Wren with the **stock C compiler**.
  The firmware calls `wren_sys::interpret` (`wren-firmware/src/main.rs:812`) —
  i.e. C `wrenCompile` on-device — and the web sim (`tools/wren-web`,
  `wasm32-unknown-unknown`, `no_std`) uses the same `wren-sys` stock VM. The
  shared crate `deluge-wren-core` is only **bindings + DSP `Engine` + `Host`
  trait**, not a compiler. The stock C VM's `FnDebug` retains only the function
  name + per-instruction line numbers; **local and instance-field names are
  discarded after compilation**, so the deluge stack cannot show named locals
  without patching `wren_compiler.c`.
- **wren-core** is the Rust frontend on the same C VM runtime, and it keeps the
  names.

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
   SAB, using `Atomics.waitAsync` (allowed on the main thread; `Atomics.wait` is
   not) for responses. (Asyncify single-thread suspend/resume was rejected.)
3. **Harness-driven debug run (load first, then callbacks).** The worker runs the
   script's top level *and* a debug harness that fires a scripted event sequence
   (note on/off, a few pad/enc/button events, N audio render blocks) so
   breakpoints inside callbacks fire. Load-time debugging works first; event
   driving layers on top.
4. **Path B — the debug core uses `wren-core`'s Rust compiler.** The debugger's
   VM is `wren-core` (with the `debug` feature), **not** the stock C compiler.
   This buys **named locals + named instance fields for free** and lets us reuse
   `debug.rs`'s inspection logic and `wren-dap`'s DAP mapping nearly verbatim,
   instead of porting ~2.7k lines to `no_std`/`alloc` *and* patching
   `wren_compiler.c`. Deluge's existing foreign bindings register into
   `wren-core`'s foreign registry unchanged (it accepts raw
   `extern "C" fn(*mut WrenVM)` pointers — the exact signature deluge uses), and
   the DSP `Engine`/`Host` are reused for the harness.
   **Bindings are made backend-generic (Option 2).** deluge's foreign bindings
   cannot be reused *as-is* under wren-core: (a) `wren-sys` unconditionally
   compiles its own copy of the upstream C VM (`build.rs:127`), so a crate
   depending on both `wren-sys` and `wren-core` double-links every `wren*` C
   symbol; and (b) wren-core's `ForeignMethodFn` hands methods a typed
   `WrenSlotApi` with no raw `*mut WrenVM`, so deluge's
   `extern "C" fn(*mut WrenVM)` bodies can't be thinly wrapped. Instead we
   refactor `deluge-wren-core` so the ~41 binding bodies (and the `Engine`'s
   callback dispatch) are written **once** against a small `SlotApi` trait, with
   two impls: a `wren-sys`-backed one (sim + device, unchanged behavior) and a
   `wren-core`-backed one (debug core). One source of truth, no drift, and it is
   the groundwork the future Rust-compiler unification needs. Because the trait
   has generic methods (`foreign_mut<T>`, `alloc_foreign<T>`, `new_foreign_in<T>`)
   it is used via **static dispatch** (`fn body<S: SlotApi>(vm: &S)`), not `dyn`.
   (Rejected alternatives: reimplementing the bindings only in the debug core —
   duplication/drift; an "extern-only `wren-sys`" linking hack — fragile.)
   **Accepted cost — a fidelity risk:** the debugger compiles with the **Rust**
   compiler while the sim + device compile with the **C** compiler. Both emit
   bytecode for the *same* C VM runtime, so *valid* scripts execute identically;
   the risk is confined to edge cases where one compiler accepts/rejects source
   the other doesn't, or emits different error text. (Configurations weighed:
   "everything C" — perfect fidelity but large port + a compiler patch;
   "everything Rust on web" — pulls the *sim* away from the *device*, which stays
   on C, and adds a sim migration. Both rejected. See **Future direction**.)

### De-risking already in place

- The app **already runs cross-origin-isolated with `SharedArrayBuffer`**:
  `vite.config.ts` sets COOP `same-origin` + COEP `require-corp`, and
  `src/audio.ts` already uses a SAB command ring to an AudioWorklet (with a
  postMessage fallback when isolation is unavailable). The headers, the SAB
  pattern, and `worker: { format: "es" }` are all present and proven here.
- **The Rust frontend already ships on web.** `tools/wren-analyzer-wasm` already
  compiles `wren-analyzer` + `wren-syntax` (the Rust parser, which is already
  `no_std`) to wasm for the editor's diagnostics — so putting `wren-core`'s
  compiler in the web/debug path continues an existing direction rather than
  introducing the Rust frontend from scratch.

## Design

### 0. Backend-generic bindings (Option 2 — prerequisite)

Refactor `deluge-wren-core` so its foreign bindings and `Engine` callback
dispatch target a small `SlotApi` trait instead of `wren-sys`'s `Vm` directly.
The trait covers exactly the surface the bindings use (enumerated from
`bindings.rs`/`engine.rs`): `ensure_slots`, `slot_type`, `get_f`/`set_f`,
`get_bool`, `get_str`, `foreign_mut<T>`, `alloc_foreign<T>`,
`new_foreign_in<T: WrenForeign>`, and the callback-handle set
(`get_handle`/`set_handle`/`make_call_handle`/`call`/`release_handle`). The
shared value types (`WrenType`, the `WrenForeign` marker, an opaque handle
newtype) move into `deluge-wren-core` (re-exported by `wren-sys` for
compatibility).

- Binding bodies become `fn output_volts_get<S: SlotApi>(vm: &S) { … }` (static
  dispatch — the generic `foreign_mut<T>`/`alloc_foreign<T>` methods make the
  trait non-object-safe).
- `wren-sys` provides the `SlotApi` impl for its `Vm` **plus** the thin
  `extern "C" fn(*mut WrenVM)` wrappers that build a `Vm` and call the generic
  body — so `METHODS`/`CLASSES` and the sim/device paths behave **identically**
  (behavior-preserving refactor, guarded by the existing sim + firmware builds
  and a golden-script run).
- The debug core provides the `wren-core` `SlotApi` impl (over `WrenSlotApi`) and
  registers the same generic bodies into wren-core's foreign registry.

This is the only change to the firmware-shared crate; everything else is
additive (new debug-core crate + new TS).

### 1. Architecture & data flow

Three actors. wren-dap's two OS threads become two JS realms:

```
main thread                          debug worker (wasm32-wasi, std)
───────────                          ──────────────────────────────
Monaco + debug UI                    debug-core.wasm
  │  set breakpoints, step, continue    = wren-core[debug]  (Rust compiler + C VM
  │  request stackTrace/scopes/vars/eval    + wrenDebugHook)
  ▼                                      + deluge bindings (into wren-core registry)
DebugController (TS)  ◄──── SAB ────►     + deluge Engine/Host (harness)
  await via Atomics.waitAsync            + debug-agent (debug.rs inspection,
                                            SAB transport instead of mpsc)
                                       hook blocks on Atomics.wait
```

- The debug worker owns its **own** VM instance, separate from the main-thread
  audio sim, so debugging never perturbs live audio.
- The SAB carries (a) the resume/step signal, so the worker can unblock while
  parked deep inside a synchronous C frame, and (b) inspection request/response
  JSON payloads.
- The main thread never blocks: it uses `Atomics.waitAsync` for responses; only
  the worker uses blocking `Atomics.wait`.

### 2. The debug core (build)

A **new `wasm32-wasi` (std) cdylib** — the *debug core* — in its own crate
(`tools/wren-web-debug`), separate from the sim wasm (which is unchanged and
stays on the stock C compiler). It links:

- **`wren-core` with the `debug` feature** — the Rust compiler + the C VM with
  the `wrenDebugHook`/error-hook patches (both already implemented in
  `wren-core/build.rs`; nothing to port here).
- **A `wren-core`-backed `SlotApi` impl + a registry adapter** — the
  backend-generic binding bodies (now in `deluge-wren-core`, see §0) are
  registered into `wren-core`'s `ForeignMethodRegistry` / `ForeignClassRegistry`
  via `CWrenVm::with_foreign`, each wrapped as a `ForeignMethodFn::Plain` that
  builds the `wren-core` `SlotApi` impl over the call's slots and invokes the
  generic body. No `wren-sys` dependency in the debug core (avoids the
  double-C-VM link).
- **The deluge prelude + `Engine`/`Host`** — the prelude compiles as Wren source
  via `wren-core`; the `Engine` drives the harness's simulated events (`Cmd`,
  MIDI, encoders) exactly as in the sim.
- **The debug-agent** — reuses `wren-core::vm::debug`'s **inspection** (fiber
  stack walk, scopes, variables, evaluate, value formatting, expandable
  lists/maps/instances by name) but replaces `DebugSession`'s two-thread
  `mpsc`/park driver with the single-thread **SAB command loop** (§3). The
  DAP-shaped request→backend→JSON mapping in `wren-dap/src/session.rs` is reused
  as the shape of those requests/responses (see §5).

Exposed C-ABI surface (called from JS in the worker): boot + install hook, set
breakpoints, launch (compile + run entry under the harness), and — callable
**while parked** — `stack_trace`, `scopes`, `variables(ref)`,
`evaluate(frame, expr)`, plus resume/step/stop controls.

**Single C-VM link.** Only the debug core links `wren-core`'s C VM; the sim wasm
links `wren-sys`'s. They are separate wasm modules, so there is no cross-module
symbol clash. Within the debug core, `wren-core` owns the sole C-VM build.

### 3. Concurrency protocol

- **Running** (not parked): normal `postMessage`. Out: `output`, `stopped`,
  `terminated`. In: `setBreakpoints`, `launch`, harness `configure`.
- **On breakpoint**: the C hook calls the agent → the agent emits a `stopped`
  event, then enters a **synchronous SAB command loop** (`Atomics.wait` on a
  command slot). Because the worker is blocked in a C frame it cannot pump its
  message queue, so **while parked everything goes over the SAB**:
  1. Main writes a request (`stackTrace`/`scopes`/`variables`/`evaluate`, JSON in
     the SAB byte region) and `Atomics.notify`s the command slot.
  2. The worker wakes, computes on the live (parked) fiber via the reused
     inspection functions, writes the JSON response into the SAB, and
     `Atomics.notify`s the response slot.
  3. Main was `await`ing that slot via `Atomics.waitAsync`; it resolves and reads
     the response.
  4. A `continue`/`next`/`stepIn`/`stepOut` command **exits** the loop → the hook
     returns → `interpret` resumes.
- **Payload sizing:** DAP-style lazy expansion (`variablesReference` fetched on
  demand) keeps each request/response small, so a fixed SAB byte region (a few
  MB) never overflows. A single oversized string value is truncated with an
  ellipsis marker.

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
| `wren-core` + `debug` feature | debug-core dependency | used as-is (Rust compiler + patched C VM); no port |
| `wren-core/src/vm/debug.rs` inspection | debug-agent | reuse fiber walk / scopes / variables / evaluate; replace `DebugSession` mpsc/park driver with the SAB command loop |
| `wren-dap/src/session.rs` | request/response shapes | keep the DAP-shaped `stackTrace`/`scopes`/`variables`/`evaluate` JSON mapping; transport is SAB/postMessage, driver is main-thread TS |
| `deluge-wren-core::{bindings, Engine, Host, prelude}` | debug core | bindings refactored to a backend-generic `SlotApi` trait (Option 2, §0); wren-core `SlotApi` impl in the debug core registers them into wren-core's foreign registry |
| `src/audio.ts` SAB pattern, `vite.config.ts` COOP/COEP, `worker:{format:"es"}` | reused directly | none |

DAP-**shaped** JSON (not a literal DAP socket) keeps a future VS Code-attach
bridge possible and mirrors `session.rs`.

### 6. UI

Monaco standalone ships no debugger UI, so we build a small custom one, reusing
existing panel/scrollbar styling:

- Breakpoint gutter with click-to-toggle; breakpoints persist per file.
- A debug toolbar: Debug/Continue, Step Over, Step Into, Step Out, Stop.
- A call-stack panel (frames → click to select, drives scopes/vars).
- A variables tree: **Locals** and **Module** scopes, **by name** (locals and
  instance fields named, via `wren-core`'s debug info), lazily expandable (lists
  indexed `[0]…`; instance fields by Wren name incl. leading `_`; maps in
  hash-table order).
- Current-line / current-frame highlight in the editor.

### 7. v1 scope

**In v1:** line breakpoints; continue; step over/into/out; call stack; variables
(Locals + Module, **named**, expandable); current-line highlight; program output;
the load+harness driven run.

**Deferred follow-ups (each an incremental port from `wren-dap`, not new
invention):** conditional breakpoints, logpoints, watch/hover evaluate, exception
breakpoints, fibers-as-threads, relative-import path fidelity, and a VS
Code-attach bridge.

## Components & boundaries

- **debug-core crate (`tools/wren-web-debug`, Rust, `wasm32-wasi`)** — owns
  compile+run+inspect. Interface: the C-ABI exports (set breakpoints, launch,
  resume/step/stop, stack/scopes/vars/eval). Depends on: `wren-core[debug]`,
  `deluge-wren-core`. Testable headless natively (same crate compiled for the
  host) driving a canned script and asserting emitted state.
- **debug-agent module (inside the debug core)** — owns the SAB command loop and
  the reused inspection calls. Interface: a Rust command enum ↔ JSON matching
  `session.rs`'s DAP shapes. Depends on: `wren-core::vm::debug`, the SAB layout.
- **debug worker loader + `DebugController` (TS)** — owns the SAB protocol and
  worker lifecycle. Interface: a typed async API the UI calls (`setBreakpoints`,
  `launch`, `continue`, `step*`, `stackTrace`, `scopes`, `variables`,
  `evaluate`) + an event stream (`stopped`, `output`, `terminated`). Depends on:
  the debug worker, the SAB layout.
- **Debug UI (TS)** — owns gutter/toolbar/panels. Depends on: `DebugController`,
  Monaco, existing styling.
- **Harness (Rust, in the debug core)** — owns the driven event sequence. Depends
  on: `Engine`.

Each unit is understandable and testable in isolation: the agent via a canned
script + assertions; the SAB protocol via a fake worker; the UI via Playwright.

## Error handling

- **Missing SAB / no cross-origin isolation:** the debug feature is **disabled**
  with a clear in-UI message (mirrors `audio.ts`'s isolation fallback). No silent
  degradation — debugging strictly requires SAB.
- **Compiler divergence (the Path B risk):** if the debug core's Rust compiler
  rejects source the sim's C compiler accepts (or vice versa), surface the
  compile error verbatim in the debug console and abort the debug run — never
  fail silently or desync from the editor. A short "compiled by the Rust
  frontend" note in the debug panel sets expectations.
- **Runtime error in the debugged script:** surfaced as an `output` event and
  program termination (v1); exception-*breakpoints* are a deferred follow-up.
- **Evaluate/inspection errors while parked:** returned as an error payload and
  shown inline (as `wren-dap` does), never crashing the worker.
- **SAB payload overflow:** truncate oversized string values; keep structural
  payloads small via lazy expansion.

## Testing

- **debug-agent headless (native):** the debug core compiled for the host with a
  canned multi-file script and a known breakpoint; assert stopped line, frame
  names, a **named** local's formatted value, an expandable list/instance child,
  and step results — no browser.
- **Playwright end-to-end:** set a gutter breakpoint → Debug → assert `stopped`
  at the right line; step over/into/out; read a named variable value; expand a
  list/instance; continue to termination. A breakpoint **inside a note handler**
  fires under the harness (Layer 2). A breakpoint in an **imported** module hits
  with the right `source.path`.
- **No regressions:** debug-core wasm + sim wasm build; `tsc --noEmit` +
  `vite build` green; the plain sim still runs single-file and multi-file
  scripts on the C compiler; device `cargo build-wren` + host build stay green.

## Future direction (out of scope here)

Unifying **device + sim + debugger** on `wren-core`'s Rust compiler would erase
the Path B fidelity risk entirely (one compiler everywhere, named debug info even
on-device). Feasibility is partly proven — `wren-syntax` is already `no_std` and
already ships on web via `wren-analyzer-wasm`; the runtime never changes (still
the upstream C VM). The gap is a `no_std`/`alloc` port of `wren-core`'s backend
(~14 files touching `std::`, all with `no_std`-friendly deps) and, critically, an
**on-device code-size/RAM spike** — the stock C compiler was chosen partly for
compactness on the RAM-constrained Deluge. This is its own brainstorm/spec, gated
on that spike; **Path B is forward-compatible with it** (if the migration lands,
the debugger becomes fidelity-perfect with zero rework).

## Open details to pin during implementation

- **SAB layout:** exact slot offsets (state word, command word, response word,
  byte-region length + bytes) and the JSON framing within the byte region.
- **debug-agent reuse boundary:** whether to fork the relevant functions out of
  `debug.rs` behind a small transport trait (mpsc for native tests, SAB in the
  worker) vs. copy-adapt them — pick the smaller diff against `debug.rs`.
- **Breakpoint delivery while running:** breakpoints set before `launch` travel
  by postMessage; breakpoints toggled *while parked* travel over the SAB and take
  effect on resume.
- **wasi in a worker:** which wasi shim the debug worker needs (the sim already
  satisfies wasi stdio imports with a tiny JS shim; the debug core adds only what
  `wren-core`'s C VM imports).
- **Harness event vocabulary:** the exact set/order of simulated events and how
  the UI configures them.
