//! Task 1.3, Step 1: harness Layer 1 — running an entry that writes CV
//! (`output[1].volts = 0.5`, i.e. `Output.new(0)`, the first/0-based CV
//! channel per the prelude's `var output = [null, Output.new(0), ...]`)
//! leaves that channel observable at 0.5 after `Harness::run_entry`, with no
//! events/ticks driven beyond the harness's own post-run flush.

mod common;

use wren_web_debug::harness::Harness;

#[test]
fn run_entry_exposes_cv_written_at_load_time() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    let mut h = Harness::new();
    h.run_entry("output[1].volts = 0.5\n", Vec::new())
        .expect("entry should compile and run");

    assert_eq!(h.cv(0), 0.5, "CV channel 0 should reflect output[1].volts");
    // Untouched channel/gates stay at their power-on defaults.
    assert_eq!(h.cv(1), 0.0);
    assert!(!h.gate(0));
}
