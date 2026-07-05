// Golden test: pins CV-output behavior across the Task 0.2 refactor that makes
// binding bodies generic over `SlotApi` instead of hard-coding `wren_sys::Vm`.
// Boots the real wren-sys VM (same path as `tools/wren-web`'s `sim_boot`), runs
// a script that drives Output, and asserts the resulting CV voltage.
//
// Single test by design: the bindings' native state (`bindings.rs`'s `STATE`/
// `MIDI`/`UI`) is documented single-threaded-only, and `cargo test` runs
// `#[test]` fns in parallel threads by default — a second test in this file
// would race the first over that shared state.
use deluge_wren_core::test_support::{run_and_read_cv, run_tick_read_cv};

#[test]
fn output_sets_cv() {
    let cv = run_and_read_cv("var o = Output.new(0)\no.volts = 1.0\n", 0);
    assert!((cv - 1.0).abs() < 1e-6, "cv1 = {cv}");
}

// Task 0.3: pins the Engine callback-dispatch path (`tick` firing a stored
// `Metro` callback handle) across the refactor that makes it generic over
// `SlotApi`. A metro fires every tick; each fire bumps `n` and writes it to
// CV channel 0 — after 3 ticks the CV should have advanced to at least 3.0.
#[test]
fn metro_callback_fires() {
    let cv = run_tick_read_cv(
        "var m = Metro.new()\nvar n = 0\nm.start(Fn.new {|stage|\n  n = n + 1\n  Output.new(0).volts = n\n}, 0.001)\n",
        /*ms_per_tick*/ 1,
        /*ticks*/ 3,
        /*ch*/ 0,
    );
    assert!(cv >= 3.0, "cv after 3 ticks = {cv}");
}
