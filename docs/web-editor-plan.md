# Web editor + simulator for Wren scripting — design & plan

A browser-based environment for writing, checking, and running Deluge Wren
scripts: a Monaco editor with full language intelligence, plus a simulated front
panel (OLED, pads, CV/gate, MIDI, encoders/buttons) and a real audio engine that
makes sound through Web Audio — all client-side, no backend.

> **Status:** design proposal. Nothing here is built yet — but the two gating
> spikes (§9) have **both been run and passed**: the C VM (compiler included)
> compiles and links to wasm with a 4-function stdio-only host contract, and the
> Rust analyzer compiles to wasm unchanged. The plan is de-risked; what remains is
> the build-out.

> **Hardware context:** the target firmware is `wren-firmware` on the Renesas
> RZ/A1L (`armv7a-none-eabihf`). The web tool does **not** emulate the Deluge
> hardware or the stock DelugeFirmware — it runs only the Wren runtime and its
> Deluge bindings. See [`docs/wren-scripting.md`](wren-scripting.md) for the
> scripting surface and [`docs/simulator.md`](simulator.md) for the separate
> native faceplate emulator.

## Contents

1. [Goals & non-goals](#1-goals--non-goals)
2. [Guiding insight](#2-guiding-insight-its-the-host-build-retargeted)
3. [Architecture overview](#3-architecture-overview)
4. [The enabling refactor: `deluge-wren-core` + `Host`](#4-the-enabling-refactor-deluge-wren-core--host)
5. [Runtime half: VM + DSP to wasm](#5-runtime-half-vm--dsp-to-wasm)
6. [Editor half: grammar + LSP reuse](#6-editor-half-grammar--lsp-reuse-from-wren-rs)
7. [Frontend stack](#7-frontend-stack)
8. [Threading & audio model](#8-threading--audio-model)
9. [Risks & gating spikes](#9-risks--gating-spikes)
10. [Milestones](#10-milestones)
11. [Crate / repo layout](#11-crate--repo-layout)
12. [Open decisions](#12-open-decisions)

---

## 1. Goals & non-goals

**Goals**

- Edit `.wren` scripts in the browser with real language intelligence (highlight,
  diagnostics, completion, hover, go-to-def, rename, etc.).
- Run a script against a simulated Deluge: OLED render, 18×8 RGB pad grid, 6
  encoders, front-panel buttons/LEDs, 2 CV + 4 gate jacks, DIN MIDI in/out.
- Hear the native DSP audio graph (`Osc`/`Env`/`Noise`/`lpf`/`Out.patch`) through
  Web Audio.
- Be **bit-faithful** to the firmware: same prelude, same node-kind constants,
  same compiler — so "works in the web sim" closely predicts "works on device."
- Ship entirely as static assets (wasm + JS); no server round-trips to run code.

**Non-goals**

- Emulating the full Deluge / DelugeFirmware (that's the separate native
  `tools/deluge-simulator`, which drives a real firmware "brain" over
  `deluge-protocol`).
- Cycle-accurate timing or USB/SD emulation.
- An online compile/build service. Everything runs in the client.

## 2. Guiding insight: it's the host build, retargeted

We are not writing a new Wren runtime. We already have one that runs off-device,
and the web tool is that runtime retargeted to wasm with bindings that hit
in-memory buffers + JS instead of hardware. Three existing facts carry the plan:

1. **The Wren C VM already builds for host.** `wren-sys/build.rs` compiles the
   stock upstream VM *including* `wren_compiler.c` (the on-device compiler) for
   both `armv7a-none-eabihf` and the host triple. So compiling Wren *source* in
   the browser is a retargeting problem, not a porting problem.
2. **`wren-firmware` already has a host build path.** Everything hardware-specific
   (USB-CDC REPL, `rza1l_hal`, `embassy-usb`) is already `#[cfg(target_os =
   "none")]`-gated; the host build runs the REPL over stdin/stdout with the std
   Embassy executor.
3. **The binding surface is small and fully enumerated.** `wren-firmware/src/
   bindings.rs` + `wren/prelude.wren` define exactly: `Output`, `Gate`, `Metro`,
   `Midi`, `Pads`, `Buttons`, `Enc`, `Led`, `Oled`, and the `Node` audio DSP
   graph. The DSP engine (`wren-firmware/src/audio.rs`) is a pure per-sample node
   graph — ideal to feed Web Audio.

Likewise on the editor side, `~/GitHub/wren-rs` already has a full TextMate
grammar and a feature-complete Rust LSP whose analysis crates are I/O-free. We
reuse both rather than rebuild.

## 3. Architecture overview

```
┌────────────────────────────── Browser (main thread) ─────────────────────────────┐
│                                                                                   │
│  Monaco editor ──┬── TextMate grammar (vscode-textmate + oniguruma-wasm)          │
│                  └── monaco-languageclient ──► [LSP Worker]                        │
│                                                                                   │
│  Panel UI (React/Svelte/vanilla): OLED canvas · pad grid · CV/gate scope ·        │
│  MIDI monitor + keyboard · encoders/buttons/LEDs · audio scope                    │
│         ▲ render (rAF)                          │ inject input events             │
│         │  drain output rings                   ▼                                 │
└─────────┼───────────────────────────────────────┼────────────────────────────────┘
          │  SharedArrayBuffer rings                │  SharedArrayBuffer rings
┌─────────┴───────────────────────┐      ┌──────────┴───────────────────────────────┐
│  [LSP Worker]                   │      │  [AudioWorklet — audio thread]            │
│  wasm: wren-analyzer +          │      │  wasm: wren-sys VM + deluge-wren-core     │
│  wren-syntax + lsp feature fns  │      │        + WebHost (Host impl)              │
│  (seeded with prelude.wren)     │      │  · runs script, metros, MIDI/UI callbacks │
│                                 │      │  · renders DSP graph per sample → output  │
└─────────────────────────────────┘      └───────────────────────────────────────────┘
```

Two independent wasm modules, two workers, two concerns:

- **Editor intelligence** (LSP Worker): static analysis, never runs the code.
- **Execution** (AudioWorklet): the actual VM + DSP, sample-accurate, off the main
  thread.

They share only the source text. The main thread owns all rendering and input.

## 4. The enabling refactor: `deluge-wren-core` + `Host`

The single most important structural decision. Today `bindings.rs` and the audio
engine are entangled with firmware-specific plumbing (embassy tasks, hardware
hooks). To avoid maintaining two drifting binding implementations, extract a
portable crate **before** building anything web:

```
crates/deluge-wren-core   (new, no_std, target-agnostic)
  ├─ engine.rs     pure DSP node graph (moved from wren-firmware/src/audio.rs;
  │                the embassy `audio_task` / `deluge::Audio` plumbing stays in firmware)
  ├─ kinds.rs      node-kind constants (K_SINE…K_SUB) — single source of truth,
  │                shared by engine + bindings (today they're duplicated by convention)
  ├─ prelude.wren  the one true prelude (moved here; both targets include_str! it)
  ├─ bindings.rs   the Wren foreign-method glue, made target-agnostic
  └─ host.rs       trait Host {
                     fn cv_set(ch, volts); fn cv_slew(ch, s);
                     fn gate_set(ch, on);
                     fn midi_tx(&[u8]);
                     fn led(id, on); fn oled_clear(); fn oled_text(..); fn oled_pixel(..); fn oled_show();
                     fn now_ms() -> u64;
                   }
```

Then there are two `Host` implementations:

- **Firmware** (`wren-firmware`): `Host` over real CV/gate/MIDI/PIC/OLED + embassy
  command rings. This is essentially what `bindings.rs` does today, refactored to
  sit behind the trait.
- **Web** (`wren-web`): `Host` over in-memory buffers, draining to JS via
  `SharedArrayBuffer` rings.

> **Why this matters:** every new binding (a new oscillator type, a new UI event)
> is then written **once** in `deluge-wren-core` and picked up by both the device
> and the web sim. Skip this and the two will silently diverge.

The Metro/MIDI/UI callback dispatch (the `tick`, `midi_rx`, `input_dispatch`,
`enc_turn` functions in today's `bindings.rs`) moves into the core too; the host
just feeds it events and calls `tick(now_ms, dt)`.

## 5. Runtime half: VM + DSP to wasm

**Crate:** `wren-web` (wasm-bindgen). Depends on `wren-sys` + `deluge-wren-core`,
provides `WebHost`, and exports a thin JS API:

```rust
#[wasm_bindgen]
impl Sim {
    pub fn new(sample_rate: f32) -> Sim;
    pub fn compile_check(&self, src: &str) -> JsValue;   // VM compile phase only → diagnostics
    pub fn load(&mut self, src: &str) -> JsValue;        // run a script; returns runtime errors
    pub fn reset(&mut self);

    // input injection (called from the worklet after draining the input ring)
    pub fn pad(&mut self, x: u8, y: u8, down: bool);
    pub fn button(&mut self, id: u8, down: bool);
    pub fn enc(&mut self, index: u8, delta: i8);
    pub fn midi_in(&mut self, status: u8, d1: u8, d2: u8);

    // the per-block audio render: advances metros/tick + fills the output buffer
    pub fn render(&mut self, out_l: &mut [f32], out_r: &mut [f32], now_ms: f64);

    // output drains (OLED frame, LED states, CV/gate values, MIDI tx) → typed arrays
    pub fn drain_oled(&self) -> Vec<u8>;
    pub fn drain_state(&self) -> JsValue;   // cv[2], gate[4], leds[…]
    pub fn drain_midi_tx(&mut self) -> Vec<u8>;
}
```

`compile_check` is the key freebie: the real Wren compiler is in the module, so we
route its `errorFn` output back as `{line, col, message}` markers — a ground-truth
linter, no JS parser. This complements (does not replace) the static analyzer in
§6.

**The C-to-wasm hurdle — RESOLVED by spike S1 (§9).** The C VM needs an allocator
(already supplied via `deluge-alloc`/`wrenReallocate`) plus a little libc. The
spike settled the approach empirically:

- The wren C is **wasm-clean**: no inline asm, no syscall headers, no
  `mmap`/`dlopen`. All seven VM translation units + both optional modules compile
  to wasm with zero source changes.
- Targeting **`wasm32-wasi` against a wasi-sysroot** (no emscripten needed) links
  the VM + wasi-libc into a ~200 KB module whose **entire host import contract is
  four `wasi_snapshot_preview1` stdio calls** (`fd_write`, `fd_close`,
  `fd_fdstat_get`, `fd_seek`). Only `fd_write` does anything — and it carries the
  `System.print` output we want to capture. A ~20-line JS shim (or
  `@bjorn3/browser_wasi_shim`) satisfies it.
- The non-internal symbols the VM references are otherwise pure-compute libc
  (`sin/cos/pow/exp/log`, `memcmp/strlen/strtod`, `realloc/free`, `rand`) plus
  `clock`/`time` (RNG seed only) and `errno` — all provided by wasi-libc or
  trivially stubbed.

> **Build note:** `wren-sys/build.rs` currently has device vs. host branches; add
> a third that passes `--target=wasm32-wasi --sysroot=<wasi-sysroot>` to `cc`.
> `wasm32-unknown-unknown` + hand-rolled shims is also viable (the libc surface is
> nearly all pure-compute) but wasi-sysroot is the confirmed path of least
> resistance.

## 6. Editor half: grammar + LSP reuse from `wren-rs`

`~/GitHub/wren-rs` gives us two separable, reusable pieces.

### 6a. Grammar + language config — reuse verbatim

`editors/vscode/syntaxes/wren.tmLanguage.json` and `language-configuration.json`
are editor-agnostic data. Run them in Monaco via:

- `vscode-textmate` + `vscode-oniguruma` (regex engine; a wasm blob) for the
  grammar → **byte-identical highlighting to the VSCode extension**, ~zero
  maintenance.
- `language-configuration.json` → `monaco.languages.setLanguageConfiguration` via
  a small adapter (brackets, comments, auto-close pairs).

This supersedes any hand-written Monarch tokenizer.

### 6b. LSP — the real prize

`crates/wren-lsp` is feature-complete (its capabilities advertise diagnostics,
completion, hover, go-to-def, references, rename, signature help, semantic tokens,
formatting, code actions, code lens, call hierarchy, inlay hints). Architecture:

```
wren-lsp  (tower-lsp + tokio "full" stdio transport)   ← does NOT port to wasm
  ├─ feature modules: completion / definition / hover / references / rename / …
  │   (these are a [lib]; each fn takes/returns lsp_types — pure serde, wasm-fine)
  └─ depends on → wren-syntax + wren-analyzer
      ↳ verified: both gate on a `std` feature, touch no fs/thread/tokio/net
```

The analysis is **already I/O-free**, so it ports. The only thing that doesn't is
tower-lsp's tokio stdio server loop. Plan:

- Build the analysis (+ the `wren_lsp` feature fns, reused) into a wasm module run
  in a **Web Worker**.
- Drive an **in-worker JSON-RPC loop** (not tower-lsp/tokio) and connect Monaco
  via `monaco-languageclient`, which already maps `lsp_types` ↔ Monaco — so we get
  the full feature set without writing providers.
- Do **not** try to compile tokio-"full" + tower-lsp to wasm; depend on the
  analyzer/feature crates directly instead.

Spike S2 (§9) confirms `wren-analyzer` compiles clean to `wasm32-unknown-unknown`.

### 6c. Teaching the LSP the Deluge API

Upstream Wren analysis doesn't know `Osc.saw`, `output[1]`, `Midi.onNoteOn`, etc.
Because `wren-analyzer` works on source, seed the in-memory workspace with
`deluge-wren-core`'s `prelude.wren` so completion/hover/go-to-def resolve the
foreign classes. Verify the analyzer supports multi-file / injected globals; this
is a real task but a large UX payoff (autocomplete on the actual hardware API).

### 6d. Two diagnostic sources, on purpose

- **`wren-analyzer`** (LSP Worker) → live editor squiggles as you type.
- **VM `compile_check`** (runtime module) → ground-truth compile/runtime errors at
  run time.

Use different Monaco marker owners so they don't fight; analyzer is advisory, the
VM is the final word.

## 7. Frontend stack

- **Build:** Vite + TypeScript. `wasm-pack`/`wasm-bindgen` for both wasm modules.
- **Editor:** Monaco (marker/diagnostics API + `monaco-languageclient` ecosystem).
  CodeMirror 6 is the lighter fallback but loses the languageclient bridge.
- **UI framework:** thin is fine — Svelte or React, or vanilla + canvas. The panel
  is mostly canvas widgets, not heavy component trees.
- **Panel widgets** (driven directly by the binding surface):
  - OLED 128×48 monochrome canvas.
  - 18×8 RGB pad grid, clickable → inject `pad(x, y, down)`.
  - CV (2) + gate (4) timeline/scope.
  - MIDI monitor + on-screen keyboard; optional real **Web MIDI** input.
  - 6 encoders (drag/scroll) + buttons + LED indicators.
  - Audio waveform/spectrum scope on the worklet output.
- **Asset reuse:** `tools/deluge-simulator/assets/Deluge.svg` for faceplate
  layout; mine that simulator's pad-grid / OLED / scope rendering logic for ideas
  (it's `iced`, so not a direct port, but the geometry and palettes transfer).
- **Examples & sharing:** load `wren-firmware/examples/*.wren` as starter scripts;
  encode the buffer in the URL for shareable permalinks.

## 8. Threading & audio model

Mirror the firmware's real concurrency model (it already uses command/output rings
between `vm_task` and `audio_task`):

- **VM + DSP run inside a Web Audio `AudioWorklet`** (a dedicated audio thread) so
  audio is glitch-free and control→audio latency is one block. `render()` both
  advances `tick`/metros and fills the output buffer.
- **Main thread ↔ worklet via `SharedArrayBuffer` rings:** input events (pads,
  MIDI, encoders) main→worklet; output state (OLED frame, LEDs, CV/gate, MIDI tx)
  worklet→main, drained at `requestAnimationFrame`.
- **Constraint:** `SharedArrayBuffer` requires **cross-origin isolation
  (COOP/COEP headers)**. Bake this into the dev server and the deploy target from
  day one; retrofitting it is painful.

Rejected alternative: VM in a plain Worker with audio precomputed and shipped to
the worklet — adds control→audio latency and complicates metro timing.

## 9. Risks & gating spikes

Two contained experiments gated the plan. **Both have been run and passed.**

- **S1 — C VM to wasm. ✅ PASSED.** All seven wren VM `.c` files + both optional
  modules compile to wasm (`clang --target=wasm32-wasi --sysroot=<wasi-sysroot>`)
  with no source changes, and link with wasi-libc into a ~200 KB `.wasm`. The
  remaining host contract is **four WASI stdio imports** (`fd_write` + three
  trivial stubs). No inline asm, no syscalls beyond stdout. Target decided:
  `wasm32-wasi` + wasi-sysroot; emscripten not needed. *(See §5 build note.)*
- **S2 — analyzer to wasm. ✅ PASSED.** `wren-syntax` and `wren-analyzer` both
  build for `wasm32-unknown-unknown` with default `std` features, **unchanged**.
  Their only dependency is `log`; no `tokio`/`fs`/`thread`/`net` leakage. The
  editor-intelligence half ports cleanly.

Remaining risks, lower (none gating):

- **COOP/COEP** deployment requirement for `SharedArrayBuffer` (mitigate: configure
  early; there are non-SAB fallbacks with worse latency).
- **Analyzer multi-file seeding** for the prelude (§6c) — verify support.
- **Binding drift** if §4 is skipped — mitigate by doing the refactor first.
- **`deluge-wren-core` extraction** must not regress the device build — keep
  `wren-firmware` green at every step.

## 10. Milestones

1. **M0 — Spikes. ✅ DONE.** S1 + S2 both passed (§9); wasm targets decided
   (`wasm32-wasi` for the VM, `wasm32-unknown-unknown` for the analyzer).
2. **M1 — Core refactor. ✅ DONE.** Extracted `crates/deluge-wren-core` (engine,
   kinds, prelude, bindings, callback dispatch) + the `Host` trait. `wren-firmware`
   now implements `Host` (`src/host.rs`, `FwHost`) and keeps only the
   firmware-specific transport (`src/audio.rs`: command ring + render task). Both
   the device (`armv7a-none-eabihf`) and host (`x86_64`) builds are green;
   behavior-preserving. No web code yet.
3. **M2 — Headless runtime module. ✅ DONE.** `tools/wren-web`: a `Host` impl over
   in-memory buffers + a C-ABI surface (`sim_boot`/`sim_load` + input injection +
   output drains) + a JS loader (`web/loader.mjs`). Builds to a ~230 KB wasm; a
   Node smoke test (`web/test.mjs`) passes — load scripts, OLED text rasterises,
   MIDI→handler→CV, metro→gate, compile errors with line numbers. `wren-sys`
   gained a wasm build branch (C against wasi-sysroot + a `clock` shim).
4. **M3 — Editor. ✅ DONE.** `tools/wren-web/app`: Vite + TS + Monaco with Wren
   highlighting + a prelude-seeded completion provider, the wasm loaded via a
   browser WASI shim (`@bjorn3/browser_wasi_shim`), Run (⌘↵) + VM errors as editor
   markers. **Live static analysis** now runs off-thread: `tools/wren-analyzer-wasm`
   wraps `wren-analyzer` (from the sibling wren-rs checkout) into an import-free
   144 KB wasm; a Web Worker (`analyzer-worker.ts`) analyzes on each edit
   (debounced) and the diagnostics become Monaco markers (`owner: wren-analyzer`,
   separate from the VM's run-time errors). Verified in a real browser: a bad
   script yields a syntax error + unused-var warnings at correct lines; clean
   scripts and the Deluge prelude API produce no false positives. **Hover,
   go-to-definition, and symbol completion** also run through the worker against a
   `SymbolIndex` built over the prelude + source (so `Osc`/`output[]`/… resolve;
   prelude symbols hover as builtins and don't offer a jump). Verified in-browser.
   Highlighting uses the **reused VSCode TextMate grammar** (`vscode-textmate` +
   `vscode-oniguruma` over `wren.tmLanguage.json`), byte-identical to the wren-rs
   extension. *Optional:* member-aware (post-`.`) completion from the analyzer.
5. **M4 — Panel UI. ✅ DONE.** A live faceplate: emissive OLED canvas, 18×8 pad
   grid, CV/gate readouts, mini MIDI keyboard, `System.print`/error console, **6
   encoders, 16 buttons + indicator LEDs, a CV/gate timeline scope, and a MIDI
   monitor** — the full binding surface — driven by a `requestAnimationFrame` tick
   loop. Verified live: MIDI key → CV; metro → CV/gate/OLED; encoder → `Enc.onTurn`;
   `Led.on` lights a button; the timeline draws CV sweeps; the monitor decodes
   in/out MIDI.
6. **M5 — Audio. ✅ DONE.** `Out.patch(...)` makes sound, rendered **off the main
   thread in an AudioWorklet**. The worklet owns a second wasm instance (engine
   only); the main VM serializes audio-graph commands (16-byte records, `codec.rs`)
   and forwards them via `port.postMessage`, drained per audio block — the
   firmware's vm_task→audio_task split, with the port as the command ring (so no
   COOP/COEP/SAB needed). Run rebuilds the VM fresh (`sim_reset` +
   `deluge_wren_core::reset`, which also Resets the worklet graph). A phosphor
   waveform scope renders from the main engine. Verified live (playwright): the
   worklet reports non-zero peak for a drone (`live`), goes silent on a no-audio
   script (`idle`), and back — proving command forwarding + Reset reach the
   worklet. The command transport is a **lock-free SharedArrayBuffer SPSC ring**
   when the page is cross-origin isolated (COOP/COEP `require-corp`, fonts
   self-hosted via `@fontsource`), falling back to `postMessage` otherwise.
   Verified in-browser: `crossOriginIsolated` true, the ring path active, audio
   plays + Reset propagates through the ring.
7. **M6 — Polish (partial).** ✅ **Web MIDI input** (real keyboard/controller →
   `sim.midiIn` + monitor; device picker) and ✅ **localStorage autosave +
   shareable `#s=` permalinks** are done and verified. *Remaining (Phase 5,
   secondary):* build the committed wasms in CI instead of checking them in;
   TextMate grammar; SAB audio ring (COOP/COEP); Monaco bundle trim; responsive
   layout / error-UX / a11y polish. See `~/.claude/plans/zippy-marinating-balloon.md`.

## 11. Crate / repo layout

```
crates/deluge-wren-core/    (new) portable runtime: engine + kinds + prelude + bindings + Host trait
wren-firmware/              device Host impl (refactored onto deluge-wren-core); unchanged behavior
tools/wren-web/             (new) wasm-bindgen crate: WebHost + Sim exports (depends on wren-sys + deluge-wren-core)
tools/wren-web-lsp/         (new) wasm-bindgen crate: analyzer + lsp feature fns for the worker
web/                        (new) Vite + TS frontend: Monaco, panel widgets, workers, AudioWorklet glue
```

External source reused (not vendored): `~/GitHub/wren-rs` — `wren-analyzer`,
`wren-syntax`, `wren-lsp` (feature lib), and the VSCode extension's grammar +
language-configuration JSON.

> The two wasm crates live under `tools/` (host-side tooling, excluded from the
> no_std cross-compiled workspace, like `tools/deluge-simulator` already is).

## 12. Open decisions

- ~~**wasm target for the C VM**~~ — **settled by S1: `wasm32-wasi` + wasi-sysroot**,
  4-import stdio shim.
- **LSP transport** — in-worker JSON-RPC + `monaco-languageclient` (recommended)
  vs. direct wasm-bindgen provider calls. Default to the former for the full
  feature set.
- **UI framework** — Svelte vs React vs vanilla+canvas. Lean minimal; the work is
  canvas widgets, not components.
- **Hosting** — must serve COOP/COEP. GitHub Pages can't set those headers
  directly; needs Cloudflare Pages / Netlify / a `_headers` file equivalent, or a
  service-worker COEP shim.
- **Relationship to `tools/deluge-simulator`** — keep separate (this is Wren-only;
  that is full-firmware). Revisit only if a unified emulator is ever wanted.
