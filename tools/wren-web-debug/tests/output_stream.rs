//! Deferred fix: a debug session's `System.print` output — and a failed
//! `interpret` (compile/runtime error) — reach the controller as `Output`
//! events, instead of vanishing into a no-op `write_fn`.
//!
//! `agent::debug_run` now wires the VM's write sink to the hook's
//! `OutputWriter`, so stdout streams to `wait_event()` as
//! `DebugStop::Output { category: "stdout", .. }`, and a broken script surfaces
//! as `"stderr"`.

mod common;

use std::collections::HashSet;

use wren_core::vm::DebugStop;

/// Drive a script to termination (no breakpoints), collecting all `Output`
/// events as (category, text).
fn collect_output(entry: &str) -> Vec<(String, String)> {
    let session = wren_web_debug::agent::debug_run(entry, vec![], HashSet::new());
    let mut out = Vec::new();
    loop {
        match session.wait_event() {
            DebugStop::Output { text, category } => out.push((category.to_string(), text)),
            DebugStop::Stopped { .. } => session.resume(), // no bps set, but be safe
            DebugStop::Terminated => break,
        }
    }
    out
}

#[test]
fn system_print_streams_as_stdout_output() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();
    let out = collect_output("System.print(\"hello from debug\")\n");
    let stdout: String = out.iter().filter(|(c, _)| c == "stdout").map(|(_, t)| t.as_str()).collect();
    assert!(
        stdout.contains("hello from debug"),
        "System.print should stream as stdout Output events; got {out:?}"
    );
}

#[test]
fn compile_error_streams_as_stderr_output() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();
    // `var x =` with no initializer is a parse error — `interpret` returns Err,
    // which must surface as a stderr Output rather than being swallowed.
    let out = collect_output("var x =\n");
    let stderr: String = out.iter().filter(|(c, _)| c == "stderr").map(|(_, t)| t.as_str()).collect();
    assert!(
        !stderr.is_empty(),
        "a compile error should surface as a stderr Output event; got {out:?}"
    );
}
