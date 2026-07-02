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
pub static VM_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
