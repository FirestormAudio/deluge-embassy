//! Deluge Wren debugger core.
//!
//! Runs the *same* deluge foreign bindings (from `deluge-wren-core`, the single
//! source of truth for the scripting surface) under **wren-core** — a wren VM
//! whose compiler is written in Rust and emits bytecode for the same upstream C
//! VM. Using wren-core (instead of the stock `wren-sys` path) gives us named
//! locals and source spans for free, which the source-level debugger needs.
//!
//! This crate depends on `deluge-wren-core` with `default-features = false` so
//! it links ONLY wren-core's copy of the C VM — never `wren-sys`'s — since two
//! upstream C VMs in one binary would fail to link.

// The SAB transport (`sab.rs`) uses the `atomic.wait`/`atomic.notify` wasm
// intrinsics, still nightly-unstable. Enable them ONLY for the threads build
// (where `sab` is compiled at all) so native / single-threaded wasm — which
// don't touch these — need no unstable feature.
#![cfg_attr(
    all(target_arch = "wasm32", target_feature = "atomics"),
    feature(stdarch_wasm_atomic_wait)
)]

/// The debug agent (Task 2.1): spawns a deluge VM on its own thread with
/// wren-core's source debugger attached, and returns the [`DebugSession`]
/// controller so a caller can drive it to a breakpoint stop.
///
/// [`DebugSession`]: wren_core::vm::DebugSession
pub mod agent;
/// Task 5.1 (harness Layer 2): the [`drive::DriveEvent`] type + `dispatch`
/// for replaying host events (MIDI/tick/encoder) into a live debug VM so a
/// breakpoint inside a fired callback parks the VM thread. Threaded through
/// [`agent::debug_run_driven`] and [`harness::debug_drive_note`].
pub mod drive;
/// The harness (Task 1.3): runs a project through the same VM/prelude
/// machinery as [`build_vm`]/[`run_project_capture`], but with a *recording*
/// [`Host`] installed instead of [`NoopHost`], so it can expose CV/gate state
/// after the run.
pub mod harness;
/// Foreign-method registration onto wren-core registries. Public so an
/// out-of-process host (e.g. the Linux `wren-host`) can build a VM with the
/// deluge bindings, exactly as [`build_vm`]/[`harness`] do internally.
pub mod register;
/// Exposed (not just crate-private) so tests can fire deluge's generic event
/// entries (e.g. `deluge_wren_core::midi_rx_impl`) from host context, wrapping
/// a live [`wren_core::vm::CWrenVm`] in a [`slotapi_wrencore::CoreSlots`] —
/// the same adapter foreign methods get, reused outside a foreign call to
/// prove the callback round-trip.
pub mod slotapi_wrencore;
/// Task 3.2: a [`transport::DebugTransport`] backed by a `SharedArrayBuffer`
/// (a region of the wasm32-wasip1-threads shared linear memory), so the same
/// [`transport::serve`] loop runs unchanged in a wasm worker driven from the
/// JS main thread. Only compiled for the threads build (it uses the wasm
/// `atomic.wait`/`atomic.notify` intrinsics), so the native rlib and the
/// single-threaded Task 3.1 wasm are unaffected.
#[cfg(all(target_arch = "wasm32", target_feature = "atomics"))]
pub mod sab;
/// The transport seam (Task 2.4): a transport-agnostic driver loop
/// (`serve`) that pumps a [`agent::debug_run`]-created
/// [`wren_core::vm::DebugSession`] over any [`transport::DebugTransport`] —
/// an in-process `mpsc` pair for native tests today, a `SharedArrayBuffer`
/// protocol for the Phase 3 browser worker.
pub mod transport;

use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

use deluge_wren_core::{Cmd, Host, set_host};
use wren_core::foreign::{ForeignClassRegistry, ForeignMethodRegistry};
use wren_core::vm::CWrenVm;

/// The Deluge prelude's public names, auto-imported into every non-entry
/// module by [`run_project_capture`]. Wren modules are isolated, so a module
/// pulled in via `import` can't see `main`'s top-level names (the prelude
/// lives in `main`, see [`build_vm`]) unless it explicitly imports them.
/// Mirrors `PRELUDE_IMPORT` in `tools/wren-web/app/src/sim.ts`, which does
/// the same thing for the sim's `runProject`; keep the two lists in sync.
const PRELUDE_IMPORT: &str = "import \"main\" for Output, Gate, Metro, Midi, Pads, Buttons, Enc, Led, Oled, Node, Osc, Env, Noise, Out, output, gate\n";

/// A no-op [`Host`]: the deluge binding bodies reach the outside world (audio
/// graph, CV/gate jacks, MIDI, LEDs, OLED) through this trait, so one must be
/// registered before any script runs or the bodies panic. The debugger cares
/// about control flow and locals, not hardware effects, so every method is a
/// sink. Later phases can swap in a host that records effects for the UI.
struct NoopHost;

impl Host for NoopHost {
    fn now_ms(&mut self) -> u64 {
        0
    }
    fn cv_set(&mut self, _ch: u8, _volts: f32) {}
    fn gate_set(&mut self, _ch: u8, _on: bool) {}
    fn midi_tx(&mut self, _msg: &[u8]) {}
    fn led(&mut self, _id: u8, _on: bool) {}
    fn oled_clear(&mut self) {}
    fn oled_text(&mut self, _x: usize, _y: usize, _text: &[u8]) {}
    fn oled_pixel(&mut self, _x: usize, _y: usize, _on: bool) {}
    fn oled_show(&mut self) {}
    fn audio_cmd(&mut self, _cmd: Cmd) {}
}

/// Register a fresh no-op host, and clear any binding state from a previous VM.
fn install_noop_host() {
    // `set_host` needs a `&'static mut`; leaking one small ZST-ish host per VM
    // is acceptable for a debug tool. The global is single-threaded (VM thread).
    let host: &'static mut NoopHost = Box::leak(Box::new(NoopHost));
    set_host(host);
    // Drop any handles / CV / audio state a previous VM left behind.
    deluge_wren_core::reset();
}

/// Build the foreign registries every wren-core VM in this crate needs: every
/// deluge binding (`Output`, `Node`/`Osc`/…, `Led`, `Oled`, …), enumerated
/// once via [`register::register_all`] so `build_vm` and
/// [`run_project_capture`] never duplicate the registration.
fn foreign_registries() -> (ForeignMethodRegistry, ForeignClassRegistry) {
    let mut mreg = ForeignMethodRegistry::new();
    let mut creg = ForeignClassRegistry::new();
    register::register_all(&mut mreg, &mut creg);
    (mreg, creg)
}

/// A wren-core module loader: resolves an `import`ed module name to its
/// source, or `None` if unknown.
type LoadFn = Box<dyn Fn(&str) -> Option<String>>;

/// Build a wren-core VM with every deluge foreign binding registered and the
/// deluge prelude compiled into `main`, *without* touching the active
/// [`Host`] — callers must call [`set_host`]/[`deluge_wren_core::reset`]
/// first. Shared by [`build_vm`], [`run_project_capture`], and
/// [`harness::Harness`] (which installs a recording host instead of
/// [`NoopHost`]), so the registry/prelude setup lives in exactly one place.
fn boot_vm(write_fn: impl Fn(&str) + 'static, load_fn: Option<LoadFn>) -> CWrenVm {
    let (mreg, creg) = foreign_registries();

    let mut vm = CWrenVm::with_foreign(write_fn, None, load_fn, mreg, creg);
    vm.interpret("main", deluge_wren_core::prelude_str())
        .expect("deluge prelude failed to compile/run under wren-core");
    vm
}

/// Build a wren-core VM with every deluge foreign binding registered and the
/// deluge prelude compiled into the `main` module.
///
/// `write_fn` receives each chunk of `System.print` output.
pub fn build_vm(write_fn: impl Fn(&str) + 'static) -> CWrenVm {
    install_noop_host();
    boot_vm(write_fn, None)
}

/// Prepend [`PRELUDE_IMPORT`] to every project module's source, keyed by the
/// module name used in `import` statements. Shared by [`run_project_capture`]
/// and [`harness::Harness::run_entry`].
fn prelude_import_modules(modules: Vec<(String, String)>) -> HashMap<String, String> {
    modules
        .into_iter()
        .map(|(name, src)| (name, format!("{PRELUDE_IMPORT}{src}")))
        .collect()
}

/// Run a multi-file project under wren-core and capture its `System.print`
/// output as a `String`.
///
/// `entry` is the source of the `main` module (which gets the deluge prelude
/// compiled ahead of it, exactly like [`build_vm`]). `modules` are the
/// project's other files, keyed by the module name used in `import`
/// statements (e.g. `import "lib/voice" for Voice"` resolves against a
/// `("lib/voice", ...)` entry).
///
/// Wren modules are isolated from one another, and the deluge prelude
/// (`Osc`, `Output`, …) is compiled into `main` — so an imported module can't
/// see it unless it imports it explicitly. Mirroring the sim's `runProject`
/// (`tools/wren-web/app/src/sim.ts`), every module source is auto-prepended
/// with [`PRELUDE_IMPORT`] before being handed to the VM, so project files
/// can use the prelude API without writing that import by hand.
pub fn run_project_capture(entry: &str, modules: Vec<(String, String)>) -> String {
    install_noop_host();

    let out = Rc::new(RefCell::new(String::new()));
    let out_write = out.clone();
    let write_fn = move |s: &str| out_write.borrow_mut().push_str(s);

    let module_map = prelude_import_modules(modules);
    let load_fn = move |name: &str| module_map.get(name).cloned();

    let mut vm = boot_vm(write_fn, Some(Box::new(load_fn)));
    if let Err(e) = vm.interpret("main", entry) {
        out.borrow_mut().push_str(&format!("\n[error] {e}"));
    }
    // Drop the VM (and the `write_fn` closure it owns, which holds the other
    // `Rc` clone) before unwrapping, so this doesn't have to fall back to a
    // clone of the captured output.
    drop(vm);

    Rc::try_unwrap(out)
        .map(RefCell::into_inner)
        .unwrap_or_else(|out| out.borrow().clone())
}

// ── wasm boot smoke (Task 3.1) ──────────────────────────────────────────
//
// A minimal C-ABI export that exercises the WHOLE wasm pipeline
// single-threaded: wren-core's Rust compiler + the C VM + deluge foreign
// bindings + the deluge prelude. It builds a VM (which compiles/runs the
// prelude) and interprets a trivial `System.print("booted")` script, capturing
// the output so the Node smoke can confirm it ran. No threads, no `debug_run`,
// no transport — those are Task 3.2.
//
// The most-recent captured output is stashed in a thread-local so `dbg_out_*`
// can hand it back to the host (wasm is single-threaded; a `thread_local!` is
// effectively a plain global here).
#[cfg(target_arch = "wasm32")]
thread_local! {
    static LAST_OUTPUT: RefCell<String> = const { RefCell::new(String::new()) };
}

/// Boot the deluge VM under wasm and run a trivial script. Returns 0 on
/// success, non-zero on failure. Captured `System.print` output is retained
/// for [`dbg_out_ptr`]/[`dbg_out_len`].
///
/// # Safety
/// C-ABI export; takes no pointers, so it is trivially sound to call.
#[cfg(target_arch = "wasm32")]
#[unsafe(no_mangle)]
pub extern "C" fn dbg_boot() -> i32 {
    let out = Rc::new(RefCell::new(String::new()));
    let out_write = out.clone();
    let write_fn = move |s: &str| out_write.borrow_mut().push_str(s);

    // `build_vm` installs the no-op host, compiles + runs the deluge prelude.
    let mut vm = build_vm(write_fn);
    let result = vm.interpret("main", "System.print(\"booted\")");
    drop(vm);

    let captured = Rc::try_unwrap(out)
        .map(RefCell::into_inner)
        .unwrap_or_else(|out| out.borrow().clone());
    LAST_OUTPUT.with(|slot| *slot.borrow_mut() = captured);

    match result {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

/// Pointer to the UTF-8 bytes captured by the most recent [`dbg_boot`].
#[cfg(target_arch = "wasm32")]
#[unsafe(no_mangle)]
pub extern "C" fn dbg_out_ptr() -> *const u8 {
    LAST_OUTPUT.with(|slot| slot.borrow().as_ptr())
}

/// Byte length of the output captured by the most recent [`dbg_boot`].
#[cfg(target_arch = "wasm32")]
#[unsafe(no_mangle)]
pub extern "C" fn dbg_out_len() -> usize {
    LAST_OUTPUT.with(|slot| slot.borrow().len())
}
