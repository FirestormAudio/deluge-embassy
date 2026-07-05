mod common;

use std::cell::RefCell;
use std::rc::Rc;

#[test]
fn osc_runs_under_wrencore() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    let out = Rc::new(RefCell::new(String::new()));
    let o2 = out.clone();
    let mut vm = wren_web_debug::build_vm(move |s: &str| o2.borrow_mut().push_str(s));
    // `Osc.sine` is a deluge prelude factory over the `Node` foreign class:
    // it dispatches a foreign method (`Node.src_`) that allocates a foreign
    // object. `output[1].volts=` dispatches an instance foreign method on the
    // `Output` foreign objects the prelude allocated at boot. If binding
    // registration + foreign allocation work under wren-core, this prints.
    //
    // NOTE: the task brief's original script used `Osc.new()`, but `Osc` has no
    // constructor (only the static factories `sine`/`saw`/`square`/`tri`), so
    // it is not a valid deluge API. `Osc.sine(440)` exercises the same surface
    // (and more: a real foreign-method dispatch + allocation). See the report.
    vm.interpret(
        "main",
        "var s = Osc.sine(440)\noutput[1].volts = 2.5\nSystem.print(\"ok\")\n",
    )
    .unwrap();
    assert!(out.borrow().contains("ok"));
}
