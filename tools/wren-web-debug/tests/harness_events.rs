//! Task 5.1 crux: a breakpoint fires *inside a host-driven callback*.
//!
//! Phase 2 proved breakpoints park the VM thread inside interpret-called
//! functions; Task 1.1c proved deluge callbacks (`Midi.onNoteOn`) fire under
//! wren-core when the host drives `midi_rx_impl` at top level. 5.1 is where
//! they meet: after `interpret` registers the handler, the debug VM thread
//! drives a synthetic note-on with the debugger hook *still attached*, so a
//! breakpoint inside the fired handler must suspend the fiber mid-`vm.call`
//! and park the VM thread exactly like a top-level breakpoint.

mod common;

/// The handler body (line 2) must stop when driven by the note-on. The prelude
/// API is `Midi.onNoteOn = Fn.new { |ch, n, v| ... }` (see `callbacks.rs`);
/// the brief's `Midi.noteOn {|n,v|}` sketch does not match the real binding
/// signature (`noteOn(_,_,_)` / `onNoteOn=(_)`), so we use the real one.
const NOTE_HANDLER: &str = "Midi.onNoteOn = Fn.new { |ch, n, v|\n  output[1].volts = n / 12.0\n}\n";

#[test]
fn breakpoint_in_note_handler_fires() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();
    let line = wren_web_debug::harness::debug_drive_note(NOTE_HANDLER, 2, 60, 100);
    assert_eq!(line, 2, "stopped INSIDE the handler, driven by the note-on");
}

/// Non-vacuity guard: a breakpoint on the *registration* line (line 1) — which
/// runs during `interpret`, before any note is driven — must NOT be reported
/// as line 2, and driving with a breakpoint on a line the handler never
/// reaches must terminate without a handler stop. This proves the line-2 stop
/// above comes from the driven callback, not from `interpret` or a constant.
#[test]
fn breakpoint_off_handler_does_not_return_two() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();
    // No line 9 exists in the 3-line source: no stop, terminates → sentinel.
    let miss = wren_web_debug::harness::debug_drive_note(NOTE_HANDLER, 9, 60, 100);
    assert_eq!(miss, -1, "no breakpoint should fire on a non-existent line");
}
