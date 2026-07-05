//! Task 2.3: stepping (`step_over`/`step_in`/`step_out`), `resume`, and
//! `evaluate` on a parked [`wren_core::vm::DebugSession`].
//!
//! These exercise `DebugSession`'s own stepping/evaluate machinery directly
//! (`wren-core/src/vm/debug.rs`) — `agent::debug_run` only builds the VM +
//! attaches the hook (Task 2.1); once parked, driving it is exactly what
//! wren-dap's `session.rs` does.

mod common;

use std::collections::HashSet;

use wren_core::vm::DebugStop;

/// Shared by `step_over_skips_call` and `step_in_enters_call`: line 7 is
/// `C.go()`, which calls into `C.go`'s body at line 3 (`return 1`); line 8
/// is `var b = 2`, right after the call returns.
const CALL_FIXTURE: &str =
    "class C {\n static go() {\n  return 1\n }\n}\nvar a = 1\nC.go()\nvar b = 2\n";

/// `step_over` on a top-level statement stops at the very next line, in the
/// same frame — no descent into any call.
#[test]
fn step_over_advances_one_line() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    let src = "var a = 1\nvar b = 2\nvar c = 3\n";
    let session = wren_web_debug::agent::debug_run(src, vec![], HashSet::from([1]));

    match session.wait_event() {
        DebugStop::Stopped { line, .. } => assert_eq!(line, 1),
        other => panic!("expected initial breakpoint stop, got {other:?}"),
    }

    session.step_over();
    match session.wait_event() {
        DebugStop::Stopped { line, reason } => {
            assert_eq!(line, 2, "step_over should land on the very next line");
            assert_eq!(reason, "step");
        }
        other => panic!("expected a step stop, got {other:?}"),
    }

    common::drive_to_end(&session);
}

/// `step_over` on a line that *calls* a method does not descend into it —
/// it stops at the next line in the calling frame, skipping the callee.
#[test]
fn step_over_skips_call() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    let session = wren_web_debug::agent::debug_run(CALL_FIXTURE, vec![], HashSet::from([7]));

    match session.wait_event() {
        DebugStop::Stopped { line, .. } => assert_eq!(line, 7),
        other => panic!("expected initial breakpoint stop, got {other:?}"),
    }

    session.step_over();
    match session.wait_event() {
        DebugStop::Stopped { line, .. } => {
            assert_eq!(
                line, 8,
                "step_over over a call should skip PAST the callee, not enter it"
            );
        }
        other => panic!("expected a step stop, got {other:?}"),
    }

    common::drive_to_end(&session);
}

/// `step_in` on a line that calls a method descends into it, stopping at the
/// callee's first line.
#[test]
fn step_in_enters_call() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    // Line 3 (`return 1`) is go()'s first line.
    let session = wren_web_debug::agent::debug_run(CALL_FIXTURE, vec![], HashSet::from([7]));

    match session.wait_event() {
        DebugStop::Stopped { line, .. } => assert_eq!(line, 7),
        other => panic!("expected initial breakpoint stop, got {other:?}"),
    }

    session.step_in();
    match session.wait_event() {
        DebugStop::Stopped { line, .. } => {
            assert_eq!(line, 3, "step_in over a call should stop inside the callee");
        }
        other => panic!("expected a step stop, got {other:?}"),
    }

    common::drive_to_end(&session);
}

/// `step_out` from inside a nested call stops back in the caller, at the
/// line right after the call returns — and NOT at the next line inside the
/// callee (which is where `step_over` would land instead).
///
/// `helper()` has two statements (`var y = 1` then `return y`) specifically
/// so the breakpoint line is *not* the callee's last statement: this makes
/// `step_over` and `step_out` diverge. If `step_out` were mis-wired to
/// `step_over`'s logic, it would stop at line 8 (`return y`, still inside
/// `helper()`) instead of line 4 (`return x`, back in `go()`), and this
/// test would fail.
#[test]
fn step_out_returns_to_caller() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    // Line map (1-indexed):
    //   1  class C {
    //   2   static go() {
    //   3    var x = C.helper()
    //   4    return x
    //   5   }
    //   6   static helper() {
    //   7    var y = 1        <- breakpoint
    //   8    return y
    //   9   }
    //   10 }
    //   11 C.go()
    let src = "class C {\n static go() {\n  var x = C.helper()\n  return x\n }\n static helper() {\n  var y = 1\n  return y\n }\n}\nC.go()\n";
    let session = wren_web_debug::agent::debug_run(src, vec![], HashSet::from([7]));

    match session.wait_event() {
        DebugStop::Stopped { line, .. } => assert_eq!(line, 7),
        other => panic!("expected initial breakpoint stop, got {other:?}"),
    }

    session.step_out();
    match session.wait_event() {
        DebugStop::Stopped { line, .. } => {
            assert_eq!(
                line, 4,
                "step_out should return to the caller's next line (go()'s `return x`), \
                 not helper()'s next line (`return y`)"
            );
        }
        other => panic!("expected a step stop, got {other:?}"),
    }

    common::drive_to_end(&session);
}

/// `evaluate` at a stop where a local `x = 42` is in scope: `x + 1` renders
/// as the primitive number `"43"` (leaf value, `var_ref == 0`).
#[test]
fn evaluate_local_expression() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    // Line 4 is `return x` inside `C.go()` — multi-line method body, since a
    // single-line `{ return x }` block is invalid Wren syntax.
    let src = "class C {\n static go() {\n  var x = 42\n  return x\n }\n}\nC.go()\n";
    let session = wren_web_debug::agent::debug_run(src, vec![], HashSet::from([4]));

    match session.wait_event() {
        DebugStop::Stopped { line, .. } => assert_eq!(line, 4),
        other => panic!("expected a breakpoint stop, got {other:?}"),
    }

    let frames = session.stack_trace(1);
    let frame = frames.first().expect("at least one frame at the breakpoint");

    let outcome = session
        .evaluate(frame.id, "x + 1")
        .unwrap_or_else(|e| panic!("evaluate(\"x + 1\") failed: {e}"));
    assert_eq!(outcome.result, "43");
    assert_eq!(outcome.var_ref, 0, "a bare number is a leaf, not expandable");

    common::drive_to_end(&session);
}

/// `resume` after a stop runs the program to completion: `wait_event` keeps
/// returning until the VM thread actually exits (`DebugStop::Terminated`).
#[test]
fn resume_runs_to_termination() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    let src = "var a = 1\nvar b = 2\nSystem.print(b)\n";
    let session = wren_web_debug::agent::debug_run(src, vec![], HashSet::from([1]));

    match session.wait_event() {
        DebugStop::Stopped { line, .. } => assert_eq!(line, 1),
        other => panic!("expected initial breakpoint stop, got {other:?}"),
    }

    session.resume();
    loop {
        match session.wait_event() {
            DebugStop::Terminated => break,
            DebugStop::Stopped { line, .. } => panic!("unexpected stop at line {line}, expected a clean run to termination"),
            DebugStop::Output { .. } => {} // async diagnostics are fine, keep pumping
        }
    }
}
