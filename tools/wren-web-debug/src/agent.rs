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

use wren_core::foreign::WrenSlotApi;
use wren_core::vm::{Breakpoints, DebugSession};

use crate::drive::DriveEvent;
use crate::slotapi_wrencore::CoreSlots;

/// Spawn a deluge VM on its own thread with a debugger attached, breaking on
/// `breakpoints` (source lines in the `main` module — the entry's module
/// name, matching [`Breakpoints::new`]'s "main"-keyed default). Returns the
/// [`DebugSession`] controller immediately; the VM thread builds the VM
/// (deluge bindings + prelude), attaches the debug hook, then interprets
/// `entry` as `main` (with `modules` importable exactly like
/// [`crate::run_project_capture`]).
///
/// Thin wrapper over [`debug_run_driven`] with no driven events — every
/// existing caller keeps the top-level-only behaviour. See the module docs
/// for the single-VM-per-process caveat.
pub fn debug_run(entry: &str, modules: Vec<(String, String)>, breakpoints: HashSet<i32>) -> DebugSession {
    debug_run_driven(entry, modules, breakpoints, Vec::new())
}

/// Like [`debug_run`], but after `interpret` returns (and BEFORE the debugger
/// is detached) the VM thread replays `drive` — a list of host events (MIDI
/// note/CC, ticks, encoder turns) — against the same VM via
/// [`crate::drive::dispatch`]. The hook stays attached across the drive, so a
/// breakpoint *inside* a fired callback (e.g. a `Midi.onNoteOn` handler)
/// suspends the fiber mid-`vm.call` and parks the VM thread exactly like a
/// top-level breakpoint. This is Task 5.1's payoff — the first breakpoint that
/// fires in host-driven callback code rather than in the interpreted entry.
///
/// The driven `*_impl` calls invoke `vm.call` from the VM thread's top level
/// (numFrames==0 after `interpret` returns), satisfying `wrenCall`'s
/// reentrancy constraint — the same context the sim/harness event loop drives
/// from. Passing an empty `drive` reproduces [`debug_run`] exactly.
pub fn debug_run_driven(
    entry: &str,
    modules: Vec<(String, String)>,
    breakpoints: HashSet<i32>,
    drive: Vec<DriveEvent>,
) -> DebugSession {
    let (session, hook) = DebugSession::new_with_breakpoints(Breakpoints::new(breakpoints));

    let entry = entry.to_string();
    let module_map = crate::prelude_import_modules(modules);

    std::thread::spawn(move || {
        // The VM (and its `Host`) must be built on this thread: `CWrenVm`
        // wraps a raw C VM pointer and is not `Send`, and `install_noop_host`
        // touches process-global state that only one live VM may own.
        crate::install_noop_host();

        let load_fn = move |name: &str| module_map.get(name).cloned();

        // Route the VM's `System.print` output into the debug session's event
        // stream as `Output` events, so stdout streams to the controller/console
        // live during a debug run (the hook itself only emits output for
        // logpoints/condition warnings). `err_writer` surfaces a failed
        // `interpret` (compile/runtime error) as stderr — otherwise it would be
        // swallowed and the console would show nothing for a broken script.
        let writer = hook.output_writer();
        let err_writer = writer.clone();
        let mut vm = crate::boot_vm(move |s: &str| writer.write(s), Some(Box::new(load_fn)));

        // Attach the debugger *after* the prelude has already run (inside
        // `boot_vm`) so prelude internals never trip a breakpoint, then
        // interpret the caller's entry — this is the call the hook parks.
        vm.attach_debugger(hook);
        if let Err(e) = vm.interpret("main", &entry) {
            err_writer.error(&format!("{e}\n"));
        }

        // Drive host events with the hook STILL attached (before detach), so a
        // breakpoint inside a fired handler parks this VM thread just like a
        // top-level one. `CoreSlots` is the same adapter foreign methods get,
        // reused here from host/top-level context (see `tests/callbacks.rs`).
        if !drive.is_empty() {
            let api: &dyn WrenSlotApi = &vm;
            let slots = CoreSlots::new(api);
            for ev in &drive {
                crate::drive::dispatch(&slots, ev);
            }
        }

        vm.detach_debugger();
    });

    session
}
