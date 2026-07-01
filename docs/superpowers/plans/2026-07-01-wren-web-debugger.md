# Wren Web Debugger Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** An in-browser, client-side source-level debugger for Deluge Wren scripts in the web editor — line breakpoints, stepping, call stack, and named variable inspection.

**Architecture:** A separate `wasm32-wasi` **debug core** built on `wren-core` (Rust compiler + patched C VM, `debug` feature) runs in a Web Worker. Its debug hook blocks on `Atomics.wait`; the main thread (Monaco + a custom debug UI) drives resume/step and reads state over a `SharedArrayBuffer` using `Atomics.waitAsync`. Deluge's foreign bindings are first refactored to a backend-generic `SlotApi` trait so the same binding bodies serve both the stock-C-compiler sim/device and the wren-core debug core.

**Tech Stack:** Rust (`wren-core`, `deluge-wren-core`), C wren VM, `wasm32-wasi` + `wasm32-unknown-unknown`, TypeScript, Vite, Monaco, Playwright.

**Spec:** `docs/superpowers/specs/2026-07-01-wren-web-debugger-design.md`

## Global Constraints

- **Branch:** `feat/wren-web` (continue on it; do not open a new branch).
- **The refactor in Phase 0 must be behavior-preserving** for the sim and firmware: `cargo build-wren` (device) and the host sim build stay green, and a golden-script run produces identical output before/after.
- **The debug core never depends on `wren-sys`** (avoids double-linking the upstream C VM). The sim wasm never depends on `wren-core`. **Because `deluge-wren-core` currently depends on `wren-sys` unconditionally and `wren-sys`'s `build.rs` always emits `cargo:rustc-link-lib=wrencore`, `wren-sys` must become an OPTIONAL, DEFAULT-ON feature of `deluge-wren-core`** (feature `wren-sys-backend`, added in Task 0.4). Sim + firmware get it by default (unchanged); `wren-web-debug` sets `default-features = false` so it links only `wren-core`'s C VM. The `SlotApi` trait, shared types, and generic binding bodies are always available (backend-agnostic); the `Vm` impl, `extern "C"` wrappers, `METHODS`/`CLASSES`, `boot*`, the `Engine`'s `Vm` entry points, and `test_support` live behind `wren-sys-backend`.
- **`WASI_SYSROOT`** must be set for any wasm build touching the C VM (already required by `wren-sys`; `wren-core` compiles its C against the host `cc` natively and against the wasi sysroot for wasm — see Task 3.1).
- **Cross-origin isolation is already configured** (`tools/wren-web/app/vite.config.ts` sets COOP/COEP; `worker: { format: "es" }`). Reuse it; the debug feature is disabled with a UI message when `crossOriginIsolated` is false.
- **Wren syntax gotchas:** no semicolons; a single-line block `{ return x }` is invalid (`{ x }` is fine); multi-line blocks with `return` are fine. Applies to every `.wren` test fixture.
- **wren-core lives at** `/home/kate/GitHub/wren-rs` (sibling repo). Paths like `wren-core = { path = "../../../wren-rs/crates/wren-core" }` mirror the existing `tools/wren-analyzer-wasm/Cargo.toml`.
- **This workspace's default target is `armv7a-none-eabihf`** (`.cargo/config.toml` `[build] target`). So every HOST `cargo test`/`cargo build`/`cargo clippy` for a host-testable crate MUST pass `--target x86_64-unknown-linux-gnu`, or it tries to compile tests for the no_std device and fails with `can't find crate for std`. Device builds use the `build-wren`/`build-fw*` aliases (which set `-Zbuild-std`). IDE/rust-analyzer `std`-missing diagnostics against `armv7a` for host-only crates are expected noise, not regressions — the authoritative no_std gate is `cargo build-wren`.

---

## File Structure

**Phase 0 — backend-generic bindings (modify shared crate):**
- `crates/deluge-wren-core/src/slotapi.rs` (create) — the `SlotApi` trait + shared `WrenType`/`WrenForeign`/`Handle` types.
- `crates/deluge-wren-core/src/bindings.rs` (modify) — binding bodies become generic over `S: SlotApi`.
- `crates/deluge-wren-core/src/engine.rs` (modify) — callback dispatch generic over `S: SlotApi`.
- `wren-sys/src/foreign.rs` (modify) — `impl SlotApi for Vm`; keep `extern "C"` wrappers.

**Phase 1–3 — debug core (new crate):**
- `tools/wren-web-debug/Cargo.toml`, `src/lib.rs` (create) — the `wasm32-wasi` cdylib + native tests.
- `tools/wren-web-debug/src/slotapi_wrencore.rs` (create) — `SlotApi` impl over `wren-core`'s `WrenSlotApi`.
- `tools/wren-web-debug/src/register.rs` (create) — register generic bindings into wren-core's registries.
- `tools/wren-web-debug/src/harness.rs` (create) — compile+run entry, then drive events.
- `tools/wren-web-debug/src/agent.rs` (create) — debug hook + inspection over a transport seam.
- `tools/wren-web-debug/src/sab.rs` (create) — SAB layout + the parked command loop.

**Phase 3–5 — web app (new TS + modify):**
- `tools/wren-web/app/src/debug/controller.ts`, `worker.ts`, `sab.ts` (create) — protocol.
- `tools/wren-web/app/src/debug/ui.ts`, `gutter.ts`, `panels.ts` (create) — UI.
- `tools/wren-web/app/index.html`, `src/style.css`, `src/main.ts` (modify) — wire UI.
- `tools/wren-web/app/public/wren-debug.wasm` (build output).

---

## Phase 0 — Backend-generic bindings (`SlotApi`)

### Task 0.1: Introduce the `SlotApi` trait + shared types

**Files:**
- Create: `crates/deluge-wren-core/src/slotapi.rs`
- Modify: `crates/deluge-wren-core/src/lib.rs` (add `pub mod slotapi;` + re-exports)
- Modify: `wren-sys/src/foreign.rs` (add `impl deluge_wren_core::SlotApi for Vm` — see note)
- Test: `crates/deluge-wren-core/tests/slotapi.rs`

**Interfaces:**
- Produces: `trait SlotApi` with methods (exact signatures below); `enum WrenType`; `trait WrenForeign`; `struct Handle(*mut c_void)`.
- Note on dependency direction: `wren-sys` already depends on nothing from `deluge-wren-core`, but `deluge-wren-core` depends on `wren-sys`. To avoid a cycle, the **trait lives in `deluge-wren-core`** and the **impl for `Vm` lives in `deluge-wren-core`** too (an `impl SlotApi for wren_sys::Vm` in a new `slotapi_wrensys.rs`), since `deluge-wren-core` can see both. `wren-sys` is not modified for the trait itself.

Corrected file list for 0.1:
- Create: `crates/deluge-wren-core/src/slotapi.rs` (trait + shared types)
- Create: `crates/deluge-wren-core/src/slotapi_wrensys.rs` (`impl SlotApi for wren_sys::Vm`)
- Modify: `crates/deluge-wren-core/src/lib.rs`

- [ ] **Step 1: Write the failing test**

`crates/deluge-wren-core/tests/slotapi.rs`:
```rust
// The trait must be object-safe-free (generic methods) but usable via a concrete
// type. This compile-level test asserts the wren-sys Vm implements SlotApi.
use deluge_wren_core::SlotApi;

fn assert_impl<S: SlotApi>() {}

#[test]
fn wren_sys_vm_is_slotapi() {
    assert_impl::<wren_sys::Vm>();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p deluge-wren-core --test slotapi`
Expected: FAIL — `SlotApi` not found / `Vm: SlotApi` not satisfied.

- [ ] **Step 3: Write the trait + shared types + wren-sys impl**

`crates/deluge-wren-core/src/slotapi.rs`:
```rust
//! Backend-generic slot access. Binding bodies target this trait so one
//! implementation serves the stock-C-VM sim/device (`wren-sys`) and the
//! `wren-core` debug core. Generic methods => static dispatch only.
use core::ffi::c_void;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WrenType { Bool, Num, Foreign, List, Map, Null, String, Unknown }

/// A persistent handle to a wren value (callback `Fn`, etc.). Opaque; the
/// backend owns the real pointer.
#[derive(Clone, Copy)]
pub struct Handle(pub *mut c_void);

/// Marks a Rust type that has a declared foreign wren class.
pub trait WrenForeign {
    fn module_name() -> &'static str;
    fn class_name() -> &'static str;
}

pub trait SlotApi {
    fn ensure_slots(&self, n: i32);
    fn slot_type(&self, slot: i32) -> WrenType;
    fn get_f(&self, slot: i32) -> f64;
    fn set_f(&self, slot: i32, v: f64);
    fn get_bool(&self, slot: i32) -> bool;
    fn get_str(&self, slot: i32) -> &str;
    /// # Safety: `slot` must hold a foreign of type `T`.
    unsafe fn foreign_mut<T>(&self, slot: i32) -> &mut T;
    /// # Safety: call only from a foreign-class allocator.
    unsafe fn alloc_foreign<T>(&self, value: T);
    /// # Safety: `T`'s foreign class must be declared in wren.
    unsafe fn new_foreign_in<T: WrenForeign>(&self, slot: i32, value: T);
    fn get_handle(&self, slot: i32) -> Handle;
    fn set_handle(&self, slot: i32, h: Handle);
    fn make_call_handle(&self, signature: &str) -> Handle;
    fn call(&self, method: Handle) -> i32;
    fn release_handle(&self, h: Handle);
}
```

`crates/deluge-wren-core/src/slotapi_wrensys.rs`:
```rust
//! `SlotApi` over the stock C VM (`wren_sys::Vm`). Delegates to the existing
//! ergonomic wrappers; behavior is identical to the pre-refactor direct calls.
use crate::slotapi::{Handle, SlotApi, WrenForeign as DwcForeign, WrenType};
use wren_sys::Vm;

fn conv_type(t: wren_sys::WrenType) -> WrenType {
    match t {
        wren_sys::WrenType::Bool => WrenType::Bool,
        wren_sys::WrenType::Num => WrenType::Num,
        wren_sys::WrenType::Foreign => WrenType::Foreign,
        wren_sys::WrenType::List => WrenType::List,
        wren_sys::WrenType::Map => WrenType::Map,
        wren_sys::WrenType::Null => WrenType::Null,
        wren_sys::WrenType::String => WrenType::String,
        wren_sys::WrenType::Unknown => WrenType::Unknown,
    }
}

impl SlotApi for Vm {
    fn ensure_slots(&self, n: i32) { Vm::ensure_slots(self, n) }
    fn slot_type(&self, slot: i32) -> WrenType { conv_type(Vm::slot_type(self, slot)) }
    fn get_f(&self, slot: i32) -> f64 { Vm::get_f(self, slot) }
    fn set_f(&self, slot: i32, v: f64) { Vm::set_f(self, slot, v) }
    fn get_bool(&self, slot: i32) -> bool { Vm::get_bool(self, slot) }
    fn get_str(&self, slot: i32) -> &str { Vm::get_str(self, slot) }
    unsafe fn foreign_mut<T>(&self, slot: i32) -> &mut T { unsafe { Vm::foreign_mut::<T>(self, slot) } }
    unsafe fn alloc_foreign<T>(&self, value: T) { unsafe { Vm::alloc_foreign::<T>(self, value) } }
    unsafe fn new_foreign_in<T: DwcForeign>(&self, slot: i32, value: T) {
        // Bridge the dwc marker to wren-sys's WrenForeign via a local shim type.
        // (Concrete binding types implement both; see Task 0.2 note.)
        unsafe { new_foreign_in_shim(self, slot, value) }
    }
    fn get_handle(&self, slot: i32) -> Handle { Handle(Vm::get_handle(self, slot) as *mut _) }
    fn set_handle(&self, slot: i32, h: Handle) { Vm::set_handle(self, slot, h.0 as *mut _) }
    fn make_call_handle(&self, sig: &str) -> Handle { Handle(Vm::make_call_handle(self, sig) as *mut _) }
    fn call(&self, method: Handle) -> i32 { Vm::call(self, method.0 as *mut _) }
    fn release_handle(&self, h: Handle) { Vm::release_handle(self, h.0 as *mut _) }
}
```
(Resolve the `new_foreign_in` marker bridge in 0.2 when the concrete foreign types are made generic; for 0.1, gate that one method behind a `todo!()`-free shim that forwards to `Vm::new_foreign_in` by having the dwc `WrenForeign` be the *only* marker — see Step 3 note below.)

**Simplification decision (adopt in 0.1):** delete `wren_sys::WrenForeign` usage from the binding types and make them implement the **dwc** `WrenForeign` only; `wren-sys`'s `new_foreign_in` is generic over `wren_sys::WrenForeign`, so add a second `Vm` method `new_foreign_in_dwc<T: deluge_wren_core::WrenForeign>` in `deluge-wren-core` (free function taking `&Vm`) that inlines the same body using the dwc marker. This keeps `wren-sys` unaware of the trait. Provide that free fn in `slotapi_wrensys.rs`:
```rust
unsafe fn new_foreign_in_shim<T: DwcForeign>(vm: &Vm, slot: i32, value: T) {
    vm.ensure_slots(slot + 2);
    let class_slot = slot + 1;
    vm.load_class(T::module_name(), T::class_name(), class_slot);
    let data = unsafe { wren_sys::wren_set_slot_new_foreign(vm.0, slot, class_slot, core::mem::size_of::<T>()) };
    if !data.is_null() { unsafe { core::ptr::write(data as *mut T, value) }; }
}
```
(If `wren_set_slot_new_foreign`/`Vm.0` aren't public, add `pub` in `wren-sys` — a mechanical visibility change, still not touching the trait.)

`crates/deluge-wren-core/src/lib.rs` — add near the other `pub use`s:
```rust
mod slotapi;
mod slotapi_wrensys;
pub use slotapi::{Handle, SlotApi, WrenForeign, WrenType};
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p deluge-wren-core --test slotapi`
Expected: PASS.

- [ ] **Step 5: Verify no regression in dependents build**

Run: `cargo build -p deluge-wren-core && cargo build -p wren-web`
Expected: both compile (the bindings still call `Vm` directly at this point; the trait is additive).

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/src/slotapi.rs crates/deluge-wren-core/src/slotapi_wrensys.rs crates/deluge-wren-core/src/lib.rs crates/deluge-wren-core/tests/slotapi.rs wren-sys/src/foreign.rs
git commit -m "wren-core: add backend-generic SlotApi trait + wren-sys impl"
```

### Task 0.2: Make binding bodies generic over `S: SlotApi`

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings.rs` (bodies: `*mut WrenVM` → `&S`)
- Test: `crates/deluge-wren-core/tests/golden_sim.rs`

**Interfaces:**
- Consumes: `SlotApi` (0.1).
- Produces: for each foreign fn, a generic body `fn <name><S: SlotApi>(vm: &S)` plus the retained `extern "C" fn(raw: *mut WrenVM)` wrapper `{ let vm = Vm::from(raw); <name>(&vm) }`. `METHODS`/`CLASSES` still point at the `extern "C"` wrappers, so the sim/device ABI is unchanged.

- [ ] **Step 1: Write the failing golden test**

`crates/deluge-wren-core/tests/golden_sim.rs` — boot the wren-sys VM, run a script exercising `Output`/`Gate`/`Osc`, and assert the CV output. Use the same boot path as `wren-web`:
```rust
// Golden: a known script drives CV1 to 1.0V; assert engine state is unchanged
// by the refactor. (Fill in the exact assert against deluge_wren_core::Engine
// readout used by tools/wren-web/src/lib.rs — mirror its cv accessor.)
#[test]
fn output_sets_cv() {
    let cv = deluge_wren_core::test_support::run_and_read_cv(
        "var o = Output.new(0)\no.volts = 1.0\n", 0);
    assert!((cv - 1.0).abs() < 1e-6, "cv1 = {cv}");
}
```
(If no `test_support` helper exists, add a minimal one in `lib.rs` behind `#[cfg(any(test, feature = \"test-support\"))]` that boots via `wren_sys::boot_with_foreign(METHODS, CLASSES)`, runs source, and reads a CV channel. This helper is reused by later phases.)

- [ ] **Step 2: Run to verify it passes BEFORE the refactor (baseline)**

Run: `cargo test -p deluge-wren-core --test golden_sim`
Expected: PASS (this is the golden baseline; capture the value).

- [ ] **Step 3: Refactor each binding body to generic form**

For every `unsafe extern "C" fn NAME(raw: *mut WrenVM) { let vm = Vm::from(raw); BODY }` in `bindings.rs`, split into:
```rust
fn NAME_impl<S: SlotApi>(vm: &S) { BODY_with_vm_methods_via_trait }
unsafe extern "C" fn NAME(raw: *mut WrenVM) { let vm = Vm::from(raw); NAME_impl(&vm); }
```
`BODY` changes are mechanical: `vm.get_f(..)`, `vm.set_f(..)`, `vm.foreign_mut::<T>(..)`, `vm.ensure_slots(..)`, handle calls — all already match `SlotApi` method names. Concrete foreign structs (`OutputObj`, `GateObj`, `MetroObj`, …) implement the dwc `WrenForeign` (replace `#[derive(WrenForeign)]`/manual impls of `wren_sys::WrenForeign` with the dwc one; keep `module_name`/`class_name`).

- [ ] **Step 4: Run the golden test + dependents**

Run: `cargo test -p deluge-wren-core --test golden_sim && cargo build -p wren-web`
Expected: PASS with the same CV value as Step 2; `wren-web` builds.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/tests/golden_sim.rs crates/deluge-wren-core/src/lib.rs
git commit -m "wren-core: make foreign binding bodies generic over SlotApi"
```

### Task 0.3: Make `Engine` callback dispatch generic

**Files:**
- Modify: `crates/deluge-wren-core/src/engine.rs` (handle/`call` paths → `&S`)
- Test: extend `crates/deluge-wren-core/tests/golden_sim.rs`

- [ ] **Step 1: Write the failing test** — a script registers a `Metro` callback that bumps a CV each tick; drive N ticks; assert CV advanced.
```rust
#[test]
fn metro_callback_fires() {
    let cv = deluge_wren_core::test_support::run_tick_read_cv(
        "var m = Metro.new()\nvar n = 0\nm.start {|| n = n + 1\nOutput.new(0).volts = n }\nm.time = 1",
        /*ms_per_tick*/ 1, /*ticks*/ 3, /*ch*/ 0);
    assert!(cv >= 3.0, "cv after 3 ticks = {cv}");
}
```

- [ ] **Step 2: Run to verify it passes (baseline)** — `cargo test -p deluge-wren-core --test golden_sim`.

- [ ] **Step 3: Refactor** `engine.rs`'s callback-invocation helpers (the `make_call_handle`/`call`/`get_handle`/`set_handle`/`release_handle` sites) to take `&S: SlotApi`; keep the `extern "C"`/`Vm`-based public entry points (`tick`, `midi_rx`, `enc_turn`, `input_dispatch`) building a `Vm` and delegating.

- [ ] **Step 4: Verify** — `cargo test -p deluge-wren-core --test golden_sim && cargo build -p wren-web`. Same result.

- [ ] **Step 5: Device build stays green** — `cargo build-wren` (the firmware wren build alias).
Expected: builds; the refactor is behavior-preserving.

- [ ] **Step 6: Commit**
```bash
git add crates/deluge-wren-core/src/engine.rs crates/deluge-wren-core/tests/golden_sim.rs
git commit -m "wren-core: make Engine callback dispatch generic over SlotApi"
```

### Task 0.4: Make the `wren-sys` backend an optional default feature

**Why:** `wren-web-debug` (Phase 1) links `wren-core`'s C VM; if it transitively pulls `wren-sys` (a hard dep of `deluge-wren-core`), the two upstream C VMs double-link and fail. So the `wren-sys`-specific surface must be feature-gated.

**Files:**
- Modify: `crates/deluge-wren-core/Cargo.toml` (make `wren-sys` optional; add `[features] default = ["wren-sys-backend"]`, `wren-sys-backend = ["dep:wren-sys"]`; `test-support = ["wren-sys-backend"]`)
- Modify: `crates/deluge-wren-core/src/lib.rs`, `src/slotapi_wrensys.rs`, `src/bindings.rs`, `src/engine.rs` (gate the `Vm`-specific items behind `#[cfg(feature = "wren-sys-backend")]`)
- Test: existing `slotapi`/`golden_sim` tests (run with default features on).

**Interfaces:**
- Always available (no feature): `SlotApi`, `WrenType`, `WrenForeign`, `Handle`, and the generic binding bodies `NAME_impl<S: SlotApi>` + generic Engine callback helpers.
- Behind `wren-sys-backend` (default): `impl SlotApi for wren_sys::Vm`, the `extern "C" fn(*mut WrenVM)` wrappers, `METHODS`/`CLASSES`, `prelude_ptr`/`boot*`, the `Vm`-based public entry points (`tick`/`midi_rx`/`enc_turn`/`input_dispatch`), and `test_support`.

- [ ] **Step 1: Write the failing test** — a build check that the crate compiles with the backend OFF, exposing only the generic surface:
```bash
# Expected to FAIL before gating: without wren-sys, unresolved wren_sys::* refs.
cargo build -p deluge-wren-core --no-default-features
```
Add a doc-test or `tests/no_backend.rs` gated `#![cfg(not(feature = "wren-sys-backend"))]` asserting `SlotApi`/`WrenType` are namable without the backend (compile-only).

- [ ] **Step 2: Run to verify it fails** — `cargo build -p deluge-wren-core --no-default-features` → FAIL (unresolved `wren_sys`).

- [ ] **Step 3: Implement the gating** — in `Cargo.toml`: `wren-sys = { path = "../../wren-sys", optional = true }` + the `[features]` block above. Add `#[cfg(feature = "wren-sys-backend")]` to: the `mod slotapi_wrensys;` line and the module; every `extern "C"` wrapper + `METHODS`/`CLASSES`/`boot`/`prelude_ptr` in `bindings.rs`; the `Vm` public entry points in `engine.rs`; and `test_support`. The generic bodies, `SlotApi`, and shared types stay ungated.

- [ ] **Step 4: Verify all configs**
Run: `cargo build -p deluge-wren-core --no-default-features` (PASS — generic-only), `cargo test -p deluge-wren-core` (PASS — defaults on, golden tests green), `cargo build -p wren-web` (PASS — gets defaults), `cargo build-wren` (PASS — device, defaults on).
Expected: all green.

- [ ] **Step 5: Commit**
```bash
git add crates/deluge-wren-core/Cargo.toml crates/deluge-wren-core/src
git commit -m "wren-core: gate wren-sys backend behind a default feature"
```

---

## Phase 1 — Debug core crate (wren-core stack, native)

### Task 1.1: New crate + wren-core `SlotApi` impl + bindings under wren-core

**Files:**
- Create: `tools/wren-web-debug/Cargo.toml`, `src/lib.rs`
- Create: `tools/wren-web-debug/src/slotapi_wrencore.rs`
- Create: `tools/wren-web-debug/src/register.rs`
- Test: `tools/wren-web-debug/tests/bindings_under_wrencore.rs`

**Interfaces:**
- Consumes: `deluge_wren_core::{SlotApi, WrenType, Handle, WrenForeign}` + the generic binding bodies (need `pub(crate)`→`pub` on the generic `*_impl` fns, or a `pub fn register_all<...>()`); `wren_core::vm::CWrenVm`, `wren_core::foreign::{ForeignMethodRegistry, ForeignClassRegistry, WrenSlotApi}`.
- Produces: `fn build_vm(write_fn) -> CWrenVm` with all deluge bindings registered.

- [ ] **Step 1: Cargo.toml + failing test**

`tools/wren-web-debug/Cargo.toml`:
```toml
[package]
name = "wren-web-debug"
version = "0.1.0"
edition = "2024"
publish = false

[lib]
crate-type = ["cdylib", "rlib"]

[dependencies]
wren-core = { path = "../../../wren-rs/crates/wren-core", features = ["debug"] }
# default-features = false is MANDATORY: it disables `wren-sys-backend` so this
# crate links ONLY wren-core's C VM (no double-VM). We use the backend-agnostic
# SlotApi trait + generic binding bodies, never deluge-wren-core's Vm surface or
# its wren-sys-based `test_support`.
deluge-wren-core = { path = "../../crates/deluge-wren-core", default-features = false }

[profile.release]
opt-level = "s"
lto = true
```
NOTE: `wren-web-debug` provides its OWN wren-core-based test helpers (`build_vm`, `run_project_capture`); it must NOT use `deluge_wren_core::test_support` (that is wren-sys-backed and would re-introduce the double-VM).

`tools/wren-web-debug/tests/bindings_under_wrencore.rs`:
```rust
use std::cell::RefCell; use std::rc::Rc;

#[test]
fn osc_runs_under_wrencore() {
    let out = Rc::new(RefCell::new(String::new()));
    let o2 = out.clone();
    let mut vm = wren_web_debug::build_vm(move |s: &str| o2.borrow_mut().push_str(s));
    // Osc is a deluge foreign class; if binding registration works, this prints.
    vm.interpret("main", "var s = Osc.new()\nSystem.print(\"ok\")\n").unwrap();
    assert!(out.borrow().contains("ok"));
}
```

- [ ] **Step 2: Run to verify it fails** — `cargo test -p wren-web-debug` → FAIL (no `build_vm`).

- [ ] **Step 3: Implement the wren-core SlotApi impl + registration**

`src/slotapi_wrencore.rs` — `struct CoreSlots<'a>(&'a dyn WrenSlotApi)` implementing `deluge_wren_core::SlotApi` by delegating to wren-core's `WrenSlotApi` (map `get_f`→`get_slot_double`, `set_f`→`set_slot_double`, `foreign_mut`→`get_slot_foreign as *mut T`, `alloc_foreign`→`set_slot_new_foreign(0,0,size)`+write, handles→wren-core's handle API). For methods wren-core's `WrenSlotApi` lacks (persistent call handles), use `CWrenVm`'s call API captured in the closure.

`src/register.rs` — `pub fn register_all(mreg: &mut ForeignMethodRegistry, creg: &mut ForeignClassRegistry)` that, for each deluge method/class, inserts a `ForeignMethodFn::Plain(|api| { let vm = CoreSlots(api); deluge_wren_core::bindings::NAME_impl(&vm); Ok(()) })`. (Requires exposing the generic bodies from `deluge-wren-core` via `pub mod bindings` or a `pub` re-export list; add that.)

`src/lib.rs` — `pub fn build_vm(write_fn: impl Fn(&str)+'static) -> CWrenVm` that builds the registries, calls `register_all`, and `CWrenVm::with_foreign(write_fn, None, None, mreg, creg)`; then compiles the deluge prelude (`deluge_wren_core::prelude_str()`).

- [ ] **Step 4: Run to verify it passes** — `cargo test -p wren-web-debug --test bindings_under_wrencore` → PASS. **This is the Path B crux de-risk: deluge bindings run under wren-core.**

- [ ] **Step 5: Commit**
```bash
git add tools/wren-web-debug crates/deluge-wren-core/src/lib.rs
git commit -m "wren-web-debug: run deluge bindings under wren-core"
```

### Task 1.2: Imports + prelude parity

**Files:** Modify `tools/wren-web-debug/src/lib.rs`; Test `tools/wren-web-debug/tests/imports.rs`.

- [ ] **Step 1: Failing test** — a two-module program (`import "lib/voice" for Voice`) where `Voice` uses `Osc`; assert it prints. Register a load fn over an in-memory `HashMap<String,String>`.
```rust
#[test]
fn multi_file_import_runs() {
    let src = "import \"lib/voice\" for Voice\nSystem.print(Voice.tag)\n";
    let lib = ("lib/voice".to_string(),
        "import \"main\" for Osc\nclass Voice {\n static tag { \"v-ok\" }\n}\n".to_string());
    let out = wren_web_debug::run_project_capture(src, vec![lib]);
    assert!(out.contains("v-ok"), "{out}");
}
```

- [ ] **Step 2: Run to verify it fails** — `cargo test -p wren-web-debug --test imports` → FAIL.

- [ ] **Step 3: Implement** `run_project_capture(entry, modules)` building the VM with a `load_fn` closure over the module map (mirrors `wren-dap/src/session.rs` load closure), pre-pending the prelude import as the sim's `runProject` does (`PRELUDE_IMPORT`).

- [ ] **Step 4: Verify** — PASS.

- [ ] **Step 5: Commit** — `git commit -m "wren-web-debug: multi-file imports + prelude"`.

### Task 1.3: Harness Layer 1 (compile + run entry via Engine)

**Files:** Create `tools/wren-web-debug/src/harness.rs`; Test `tools/wren-web-debug/tests/harness_load.rs`.

- [ ] **Step 1: Failing test** — running an entry that calls `Output.new(0).volts = 0.5` leaves CV0 at 0.5 (harness runs top-level, no events yet).
- [ ] **Step 2: Verify fail.**
- [ ] **Step 3: Implement** `Harness::run_entry(&mut self, entry, modules)` that builds the VM, installs the (empty) engine, runs the entry, and exposes `cv(ch)`/`gate(ch)` readouts (reuse `deluge_wren_core::Engine` accessors).
- [ ] **Step 4: Verify PASS.**
- [ ] **Step 5: Commit** — `"wren-web-debug: harness layer 1 (load-time run)"`.

---

## Phase 2 — Debug agent (reuse debug.rs inspection, native mpsc transport)

The agent reuses `wren_core::vm::debug` (`DebugSession`, `Breakpoints`, `install`, the `FrameInfo`/`Scope`/`Variable` types, and `stack_trace`/`scopes`/`variables`/`evaluate`/`step_*`). Native tests drive it exactly like `wren-dap/src/session.rs` does (the VM on a thread, the `DebugSession` as controller) — this proves inspection before introducing the SAB.

### Task 2.1: Hit a breakpoint

**Files:** Create `tools/wren-web-debug/src/agent.rs`; Test `tools/wren-web-debug/tests/agent_break.rs`.

**Interfaces:**
- Consumes: `wren_core::vm::{Breakpoints, DebugSession, DebugStop}` (see `wren-dap/src/session.rs:201-256` for the launch pattern).
- Produces: `fn debug_run(entry: &str, modules: Vec<(String,String)>, breakpoints: HashSet<i32>) -> DebugSession` (spawns the VM thread with the debug hook attached and the deluge bindings registered).

- [ ] **Step 1: Failing test**
```rust
use std::collections::HashSet;
#[test]
fn stops_at_line_2() {
    let session = wren_web_debug::agent::debug_run(
        "var a = 1\nvar b = 2\nSystem.print(b)\n", vec![], HashSet::from([2]));
    match session.wait_event() {
        wren_core::vm::DebugStop::Stopped { line, .. } => assert_eq!(line, 2),
        other => panic!("{other:?}"),
    }
}
```

- [ ] **Step 2: Verify fail.**
- [ ] **Step 3: Implement** `debug_run` mirroring `wren-dap`'s `launch` (session.rs:182-256) but using `wren_web_debug::build_vm` + `register_all` so the deluge bindings are present, and `vm.attach_debugger(hook)` from `DebugSession::new_with_breakpoints`.
- [ ] **Step 4: Verify PASS.**
- [ ] **Step 5: Commit** — `"wren-web-debug: agent hits line breakpoints"`.

### Task 2.2: Stack trace + named variables

**Files:** Test `tools/wren-web-debug/tests/agent_inspect.rs`.

- [ ] **Step 1: Failing test** — stop at a line inside a method with a local `x = 42`; assert `stack_trace()[0].name` is the method and `variables(scopes(frame)[0].var_ref)` contains a **named** entry `x` with value `"42"`.
```rust
#[test]
fn named_local_visible() {
    let src = "class C {\n static go() {\n var x = 42\n return x\n }\n}\nC.go()\n";
    let s = wren_web_debug::agent::debug_run(src, vec![], std::collections::HashSet::from([4]));
    let _ = s.wait_event();
    let frame = s.stack_trace(1)[0].id;
    let scope = s.scopes(frame).into_iter().find(|sc| sc.name == "Locals").unwrap();
    let vars = s.variables(scope.var_ref);
    assert!(vars.iter().any(|v| v.name == "x" && v.value == "42"), "{vars:?}");
}
```

- [ ] **Step 2–4:** verify fail → the reused `DebugSession` methods already implement this (wren-core's compiler emits `DebugLocal`) → PASS. If a scope-name mismatch, adjust the assertion to wren-core's actual scope naming (`"Locals"`/`"Module"`).
- [ ] **Step 5: Commit** — `"wren-web-debug: stack + named variable inspection"`.

### Task 2.3: Stepping + continue + evaluate

- [ ] **Step 1: Failing tests** — `step_over` advances one line; `evaluate(frame, "x + 1")` returns `"43"`; `resume` runs to termination (`DebugStop::Terminated`). One test per behavior.
- [ ] **Steps 2–4:** implement thin `agent` wrappers over `DebugSession::{step_over, step_in, step_out, resume, evaluate}`; verify PASS.
- [ ] **Step 5: Commit** — `"wren-web-debug: stepping, continue, evaluate"`.

### Task 2.4: Transport seam

**Files:** Modify `src/agent.rs`; Test `tools/wren-web-debug/tests/agent_transport.rs`.

**Interfaces:**
- Produces: `trait DebugTransport { fn recv_cmd(&self) -> DebugCmd; fn send_event(&self, ev: DebugEvent); }` where `DebugCmd`/`DebugEvent` are JSON-serializable (serde) mirrors of `wren-dap/src/session.rs`'s DAP request/response bodies; plus `fn serve(session: &DebugSession, transport: &impl DebugTransport)` — the request/response loop lifted from `session.rs`'s driver `match`.

- [ ] **Step 1: Failing test** — an in-memory `mpsc` transport feeds `{StackTrace}` then `{Continue}`; assert the emitted events include a `stackFrames` payload then `terminated`.
- [ ] **Step 2: Verify fail.**
- [ ] **Step 3: Implement** `serve` by porting `session.rs`'s `DriverCommand` match arms (lines 60-176) to `DebugCmd`/`DebugEvent` over the trait. Native test uses an mpsc impl; Phase 3 adds the SAB impl.
- [ ] **Step 4: Verify PASS.**
- [ ] **Step 5: Commit** — `"wren-web-debug: transport-agnostic DAP serve loop"`.

---

## Phase 3 — SAB protocol + wasm build

### Task 3.1: Build the debug core to `wasm32-wasi` + worker boot smoke

**Files:** Create `tools/wren-web-debug/build.sh` (or a cargo alias), `tools/wren-web/app/src/debug/worker.ts`; Test: a Node smoke that instantiates the wasm.

- [ ] **Step 1: Failing check** — a Node script (`tools/wren-web-debug/smoke.mjs`) that loads `wren-debug.wasm`, satisfies its WASI imports with the same tiny shim the sim uses (`tools/wren-web/app/src/sim.ts` WASI shim), calls an exported `dbg_boot()`, and prints "booted". Run: `node tools/wren-web-debug/smoke.mjs` → FAIL (no wasm yet).

- [ ] **Step 2: Add the C-ABI export surface** to `src/lib.rs` (`#[no_mangle] extern "C"`): `dbg_boot`, `dbg_set_breakpoints(ptr,len)`, `dbg_launch(entry_ptr,len)`, `dbg_add_module(...)` (mirror the sim's `sim_add_module`), and the parked inspection exports (added in 3.2). Guard `cdylib`-only exports behind `#[cfg(target_arch = "wasm32")]`.

- [ ] **Step 3: Build** — `WASI_SYSROOT=... cargo build -p wren-web-debug --release --target wasm32-wasi` (wren-core compiles its C against the wasi sysroot; pass it the same way `wren-sys` does). Copy the artifact to `tools/wren-web/app/public/wren-debug.wasm`.
Expected: builds; `node smoke.mjs` prints "booted".

- [ ] **Step 4: Commit** — `"wren-web-debug: wasm32-wasi build + boot smoke"`.

### Task 3.2: SAB layout + parked command loop + `DebugController`

**Files:** Create `tools/wren-web-debug/src/sab.rs`, `tools/wren-web/app/src/debug/sab.ts`, `controller.ts`; Test: `tools/wren-web/app/src/debug/sab.test.ts` (vitest) with a fake worker.

**Interfaces:**
- Produces (shared layout, both languages): a `SharedArrayBuffer` with an `Int32Array` control header — `[STATE, CMD_SEQ, RESP_SEQ, LEN]` at words 0–3 — followed by a byte region for JSON. `STATE`: 0=running, 1=parked. Main writes a request's JSON into bytes, bumps `CMD_SEQ`, `Atomics.notify(cmd)`. Worker (parked, in `Atomics.wait` on `CMD_SEQ`) computes, writes response JSON, bumps `RESP_SEQ`, `Atomics.notify(resp)`. Main `await`s via `Atomics.waitAsync(resp)`.
- `DebugController` (TS): `setBreakpoints`, `launch`, `continue`, `stepOver/In/Out`, `stackTrace`, `scopes`, `variables`, `evaluate`, `on(event, cb)`.

- [ ] **Step 1: Failing test** — `sab.test.ts`: instantiate the SAB, run a stub "worker" (a function on a real `Worker` or a mocked one) that echoes a `stackTrace` request; assert `DebugController.stackTrace(1)` resolves with the echoed payload via `Atomics.waitAsync`.
- [ ] **Step 2: Verify fail** — `npx vitest run src/debug/sab.test.ts` (add vitest if absent) → FAIL.
- [ ] **Step 3: Implement** `sab.ts` (encode/decode header + JSON), `controller.ts` (request → SAB → `Atomics.waitAsync` → response), and `src/sab.rs` (the parked loop `DebugTransport` impl: block on `Atomics.wait`-equivalent via `core::arch::wasm32::memory_atomic_wait32`, read CMD JSON, call `serve`'s handler, write RESP JSON, notify).
- [ ] **Step 4: Verify PASS.**
- [ ] **Step 5: Commit** — `"wren-web-debug: SAB protocol + DebugController"`.

### Task 3.3: End-to-end worker debug run

**Files:** Modify `worker.ts`, `controller.ts`; Test: Playwright `tools/wren-web/app/tests/debug-e2e.spec.ts`.

- [ ] **Step 1: Failing Playwright test** — load the app, `evaluate` a script into the project, call `controller.launch` with a breakpoint at line 2 via a test hook on `window`, assert a `stopped` event at line 2 and that `stackTrace()` returns ≥1 frame.
- [ ] **Step 2: Verify fail.**
- [ ] **Step 3: Wire** the worker: on `launch`, boot the debug wasm, register the SAB, run `dbg_launch` (which enters `serve` over the SAB transport); forward `output`/`stopped`/`terminated` as postMessage events while running, SAB while parked.
- [ ] **Step 4: Verify PASS** (`npx playwright test debug-e2e`).
- [ ] **Step 5: Commit** — `"wren-web-debug: end-to-end worker debug run"`.

---

## Phase 4 — Debug UI

### Task 4.1: Breakpoint gutter

**Files:** Create `tools/wren-web/app/src/debug/gutter.ts`; Modify `src/main.ts`, `src/style.css`; Test: Playwright `debug-gutter.spec.ts`.

- [ ] **Step 1: Failing test** — click the gutter at line 3 → a breakpoint decoration appears; the project's breakpoint set (persisted, per file) contains `main.wren:3`; reload preserves it.
- [ ] **Step 2: Verify fail.**
- [ ] **Step 3: Implement** Monaco `glyphMarginClassName` decorations toggled on `onMouseDown` in the glyph margin; store breakpoints in the `ProjectStore` (extend `project.ts` with a `breakpoints: Record<path, number[]>` field + persistence).
- [ ] **Step 4: Verify PASS.**
- [ ] **Step 5: Commit** — `"wren-web: breakpoint gutter + persistence"`.

### Task 4.2: Debug toolbar + current-line highlight

- [ ] **Step 1: Failing test** — a Debug button starts a run; on `stopped`, the current line gets a highlight decoration and the toolbar shows Continue/Step/Stop enabled; Continue resumes to termination and clears the highlight.
- [ ] **Steps 2–4:** implement toolbar (`src/debug/ui.ts`) wired to `DebugController`; current-line via a Monaco decoration; verify PASS.
- [ ] **Step 5 (error handling — spec §Error handling):** when `!crossOriginIsolated`, render the Debug button disabled with the tooltip "Debugging needs cross-origin isolation (SharedArrayBuffer)" and never spawn the worker (mirror `src/audio.ts`'s isolation fallback). When `dbg_launch` reports a compile error, show it verbatim in the debug console and abort the run (do not desync the editor). Add a Playwright assertion: with isolation faked off (test hook), the Debug button is disabled; a script with a syntax error shows the compile error in the console.
- [ ] **Step 6: Commit** — `"wren-web: debug toolbar, current-line highlight, isolation/compile-error handling"`.

### Task 4.3: Call-stack panel

- [ ] **Step 1: Failing test** — stop inside a nested call; the call-stack panel lists ≥2 frames; clicking a frame requests its scopes.
- [ ] **Steps 2–4:** `src/debug/panels.ts` renders `stackTrace()`; selecting a frame drives 4.4; verify PASS.
- [ ] **Step 5: Commit** — `"wren-web: call-stack panel"`.

### Task 4.4: Variables tree (lazy expand)

- [ ] **Step 1: Failing test** — at a stop, the variables panel shows a **named** local; expanding a list variable lazily fetches and shows `[0]`, `[1]`.
- [ ] **Steps 2–4:** render `scopes`→`variables`; expansion calls `variables(ref)` on demand; verify PASS.
- [ ] **Step 5: Commit** — `"wren-web: variables tree with lazy expansion"`.

---

## Phase 5 — Harness Layer 2 (driven callbacks) + drive panel

### Task 5.1: Drive events so callback breakpoints fire

**Files:** Modify `tools/wren-web-debug/src/harness.rs`; Test `tools/wren-web-debug/tests/harness_events.rs` + Playwright.

- [ ] **Step 1: Failing test** — a script sets a breakpoint inside a MIDI note-on handler; the harness fires a note-on; assert the agent stops at that handler's line.
```rust
#[test]
fn breakpoint_in_note_handler_fires() {
    let src = "Midi.noteOn {|n,v|\n Output.new(0).volts = n / 12.0\n}\n";
    // breakpoint at line 2, then drive a note-on(60,100) via the harness
    let stopped_line = wren_web_debug::harness::debug_drive_note(src, 2, 60, 100);
    assert_eq!(stopped_line, 2);
}
```

- [ ] **Step 2: Verify fail.**
- [ ] **Step 3: Implement** the harness event step: after the entry runs, call `deluge_wren_core::midi_rx`/`tick`/`enc_turn`/`input_dispatch` on the debug VM (with the hook attached) per a configurable sequence.
- [ ] **Step 4: Verify PASS.**
- [ ] **Step 5: Commit** — `"wren-web-debug: harness layer 2 (driven callbacks)"`.

### Task 5.2: Drive panel UI

- [ ] **Step 1: Failing Playwright test** — a "debug drive" panel lets the user pick a note + block count; running with a note-handler breakpoint stops there.
- [ ] **Steps 2–4:** small form in `src/debug/ui.ts` feeding `launch`'s harness config; verify PASS.
- [ ] **Step 5: Commit** — `"wren-web: debug drive panel"`.

---

## Final verification

- [ ] `cargo test -p deluge-wren-core` (golden refactor tests) green.
- [ ] `cargo test -p wren-web-debug` (agent/harness native tests) green.
- [ ] `cargo build-wren` (device) + host sim build green.
- [ ] `WASI_SYSROOT=... cargo build -p wren-web-debug --release --target wasm32-wasi` green; artifact copied to `public/`.
- [ ] `tsc --noEmit && vite build` green.
- [ ] `npx playwright test` (debug-e2e, gutter, toolbar, panels, drive) green.
- [ ] The plain sim still runs single-file and multi-file scripts (no regression).

## Notes on reuse boundaries (for the implementer)

- **Do not re-derive inspection logic.** `wren_core::vm::debug`'s `DebugSession` already implements stack/scopes/variables/evaluate against the parked fiber; Phase 2 calls it. Read `wren-dap/src/session.rs` (the whole file, 286 lines) — it is the reference for the launch pattern and the DAP request→response mapping you are porting to `DebugCmd`/`DebugEvent`.
- **Named locals come from wren-core's compiler**, not the C VM — this is why Path B was chosen. If a local shows no name, the bug is in registration/scope selection, not the compiler.
- **Keep the debug core off `wren-sys`.** If a build pulls `wren-sys` transitively (e.g. via a `deluge-wren-core` feature), you will double-link the C VM. `deluge-wren-core` must expose the generic binding bodies and shared types **without** forcing its `wren-sys`-backed VM boot into the debug core's build (use a default-off feature if needed).
