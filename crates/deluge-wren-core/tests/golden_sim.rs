// Golden test: pins CV-output behavior across the Task 0.2 refactor that makes
// binding bodies generic over `SlotApi` instead of hard-coding `wren_sys::Vm`.
// Boots the real wren-sys VM (same path as `tools/wren-web`'s `sim_boot`), runs
// a script that drives Output, and asserts the resulting CV voltage.
//
// Single test by design: the bindings' native state (`bindings.rs`'s `STATE`/
// `MIDI`/`UI`) is documented single-threaded-only, and `cargo test` runs
// `#[test]` fns in parallel threads by default — a second test in this file
// would race the first over that shared state.
use deluge_wren_core::test_support::run_and_read_cv;

#[test]
fn output_sets_cv() {
    let cv = run_and_read_cv("var o = Output.new(0)\no.volts = 1.0\n", 0);
    assert!((cv - 1.0).abs() < 1e-6, "cv1 = {cv}");
}
