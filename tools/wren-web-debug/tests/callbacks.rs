//! Proves a deluge callback binding (`Midi.onNoteOn`) round-trips through
//! wren-core: a foreign method stores the `Fn` as a handle during `interpret`,
//! then the debug core's host loop fires it later via
//! [`deluge_wren_core::midi_rx_impl`] — the same generic entry point the
//! device firmware and web sim call through the `wren-sys-backend` wrapper
//! (`midi_rx`), just invoked directly here since this crate only links
//! wren-core's C VM.
//!
//! This is the empirical validation of [`CoreSlots`]'s de-stubbed handle
//! methods (`get_handle`/`set_handle`/`make_call_handle`/`call`/
//! `release_handle`): before they delegated to wren-core's new handle API,
//! this test failed (the callback silently never fired).

mod common;

use std::cell::RefCell;
use std::rc::Rc;

use deluge_wren_core::midi_rx_impl;
use wren_core::foreign::WrenSlotApi;
use wren_web_debug::slotapi_wrencore::CoreSlots;

#[test]
fn midi_note_on_callback_fires_under_wrencore() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    let out = Rc::new(RefCell::new(String::new()));
    let o2 = out.clone();
    let mut vm = wren_web_debug::build_vm(move |s: &str| o2.borrow_mut().push_str(s));

    // Register a 3-arg handler, matching `Midi.onNoteOn=(fn)`'s
    // `call(_,_,_)` invocation in `midi_rx_impl` (ch, note, vel).
    vm.interpret(
        "main",
        "Midi.onNoteOn = Fn.new { |ch, n, v|\n  System.print(\"fired n=%(n)\")\n}\n",
    )
    .expect("registering the handler should compile/run");

    // Registration alone must not fire the callback.
    assert!(
        out.borrow().is_empty(),
        "callback fired merely by registering it: {:?}",
        out.borrow()
    );

    // Fire a synthetic MIDI note-on from host (top-level) context — exactly
    // the context the device's VM loop calls `midi_rx` from (never from
    // inside a foreign method). `0x90` = note-on channel 1, note 60, vel 100.
    let api: &dyn WrenSlotApi = &vm;
    let slots = CoreSlots::new(api);
    midi_rx_impl(&slots, 0x90, 60, 100);

    assert!(
        out.borrow().contains("fired n=60"),
        "callback did not fire (handle round-trip failed): {:?}",
        out.borrow()
    );
}
