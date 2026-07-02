//! Task 2.1: drive a deluge script under wren-core's source-level debugger.
//!
//! Mirrors `wren-dap`'s `launch` (`wren-dap/src/session.rs:182-256`): build a
//! [`DebugSession`]/`DebugHook` pair from the caller's breakpoints, then
//! spawn a thread that builds a fresh deluge VM (bindings + prelude, via
//! [`crate::boot_vm`]) with a no-op [`crate::install_noop_host`] host,
//! attaches the hook, and interprets `entry` as the `main` module — with
//! `modules` importable exactly like [`crate::run_project_capture`]. The
//! controller ([`DebugSession`]) is returned immediately; the VM runs (and,
//! at a breakpoint, parks) on its own thread.
//!
//! The C VM handle (`CWrenVm`) is not `Send`, so it must be *created* inside
//! the spawned thread — only the `DebugHook` and the entry/module source
//! strings cross the thread boundary.
//!
//! # Single VM per process
//! `deluge_wren_core`'s host/binding state lives in process globals (see
//! [`crate::install_noop_host`]), sound only with one live wren-core VM per
//! process at a time. Callers — and especially tests — must not run two
//! [`debug_run`] (or [`crate::build_vm`]/[`crate::harness::Harness`]) VMs
//! concurrently in the same process. The test suite serializes on a shared
//! lock (see `tests/common/mod.rs`'s `VM_TEST_LOCK`); production embedders
//! of this crate get the same guarantee for free as long as they only ever
//! drive one debug session at a time, which is the intended usage.

use std::collections::HashSet;

use wren_core::vm::{Breakpoints, DebugSession};

/// Spawn a deluge VM on its own thread with a debugger attached, breaking on
/// `breakpoints` (source lines in the `main` module — the entry's module
/// name, matching [`Breakpoints::new`]'s "main"-keyed default). Returns the
/// [`DebugSession`] controller immediately; the VM thread builds the VM
/// (deluge bindings + prelude), attaches the debug hook, then interprets
/// `entry` as `main` (with `modules` importable exactly like
/// [`crate::run_project_capture`]).
///
/// See the module docs for the single-VM-per-process caveat.
pub fn debug_run(entry: &str, modules: Vec<(String, String)>, breakpoints: HashSet<i32>) -> DebugSession {
    let (session, hook) = DebugSession::new_with_breakpoints(Breakpoints::new(breakpoints));

    let entry = entry.to_string();
    let module_map = crate::prelude_import_modules(modules);

    std::thread::spawn(move || {
        // The VM (and its `Host`) must be built on this thread: `CWrenVm`
        // wraps a raw C VM pointer and is not `Send`, and `install_noop_host`
        // touches process-global state that only one live VM may own.
        crate::install_noop_host();

        let load_fn = move |name: &str| module_map.get(name).cloned();
        let mut vm = crate::boot_vm(|_s: &str| {}, Some(Box::new(load_fn)));

        // Attach the debugger *after* the prelude has already run (inside
        // `boot_vm`) so prelude internals never trip a breakpoint, then
        // interpret the caller's entry — this is the call the hook parks.
        vm.attach_debugger(hook);
        let _ = vm.interpret("main", &entry);
        vm.detach_debugger();
    });

    session
}
