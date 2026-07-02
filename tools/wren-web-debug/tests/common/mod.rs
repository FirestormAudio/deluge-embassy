//! Shared test-only infrastructure for `wren-web-debug`'s integration tests.
//!
//! `deluge_wren_core` keeps its host/binding state in process globals (a
//! `static mut HOST` set via `set_host`, plus other VM-wide statics), which
//! is only sound with a single live VM per process. `cargo test` runs every
//! `#[test]` fn *within one test binary* on its own thread by default (each
//! `tests/*.rs` file is a separate binary/process, so different files can't
//! race each other this way — but two `#[test]` fns in the *same* file can).
//! Any test that boots a wren-core VM — directly (`build_vm`,
//! `run_project_capture`) or indirectly (`Harness`, `agent::debug_run`) —
//! MUST hold this lock for as long as its VM is live, or two VMs booting
//! concurrently on different threads is a real data race (UB), not just
//! flakiness.
//!
//! Usage: `let _g = common::VM_TEST_LOCK.lock().unwrap();` as the first line
//! of any VM-booting `#[test]` fn.
#[allow(dead_code)]
pub static VM_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Teardown for any test that parks a [`wren_core::vm::DebugSession`]'s VM
/// thread at a breakpoint (via `agent::debug_run`). `VM_TEST_LOCK` only
/// guards the *boot* window — it says nothing about a parked VM thread that's
/// still alive when a test function returns. If a test stops at a breakpoint
/// and returns without resuming, its VM thread is still live (parked in the
/// debug hook) when a *later* test in the same binary boots a second VM —
/// two live wren-core VMs touching the same process globals is a data race
/// (UB), not just flakiness.
///
/// Call this at the end of every debug test that reaches a breakpoint stop,
/// while still holding the `VM_TEST_LOCK` guard (drop the guard only after
/// this returns): it resumes execution and pumps `wait_event()` until the VM
/// thread actually exits (`DebugStop::Terminated`), so the next test's VM
/// boot is guaranteed not to race this one's teardown.
#[allow(dead_code)]
pub fn drive_to_end(session: &wren_core::vm::DebugSession) {
    session.resume();
    loop {
        match session.wait_event() {
            wren_core::vm::DebugStop::Terminated => break,
            // Another breakpoint/step stop (or async output) — keep resuming
            // until the VM thread actually exits.
            wren_core::vm::DebugStop::Stopped { .. } => session.resume(),
            wren_core::vm::DebugStop::Output { .. } => {}
        }
    }
}
