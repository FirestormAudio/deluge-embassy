//! Task 2.2: stack trace + NAMED variable inspection.
//!
//! This is the payoff of the whole Path B decision: wren-core's Rust
//! compiler emits local/field debug info (`DebugLocal`/`DebugClass`) that
//! the stock C compiler never produces, so `DebugSession::scopes`/
//! `variables` can report a local by its *source name* — not just a slot
//! index. Stop inside a method holding `var x = 42`, walk the stack via
//! `stack_trace`, find the "Locals" scope via `scopes`, and confirm
//! `variables` reports an entry literally named `x` with rendered value
//! `"42"`.

mod common;

use std::collections::HashSet;

use wren_core::vm::DebugStop;

#[test]
fn named_local_visible() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    // Line 4 is `return x` inside `C.go()` — multi-line method body, since a
    // single-line `{ return x }` block is invalid Wren syntax.
    let src = "class C {\n static go() {\n  var x = 42\n  return x\n }\n}\nC.go()\n";

    let session = wren_web_debug::agent::debug_run(src, vec![], HashSet::from([4]));

    match session.wait_event() {
        DebugStop::Stopped { line, .. } => assert_eq!(line, 4),
        other => panic!("expected a breakpoint stop, got {other:?}"),
    }

    // Thread 1 is always the current (only) fiber here.
    let frames = session.stack_trace(1);
    let frame = frames.first().expect("at least one frame at the breakpoint");
    assert!(
        frame.name.contains("go"),
        "expected the innermost frame to be C.go(), got name {:?} in {frames:?}",
        frame.name
    );

    let scopes = session.scopes(frame.id);
    let locals = scopes
        .iter()
        .find(|sc| sc.name == "Locals")
        .unwrap_or_else(|| panic!("no \"Locals\" scope among {scopes:?}"));

    let vars = session.variables(locals.var_ref);
    assert!(
        vars.iter().any(|v| v.name == "x" && v.value == "42"),
        "expected a named local `x` with value \"42\", got {vars:?}"
    );

    common::drive_to_end(&session);
}
