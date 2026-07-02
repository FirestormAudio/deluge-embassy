/// A two-module program: `main` imports `lib/voice`, and `lib/voice` itself
/// references a prelude symbol (`Osc`) without importing it explicitly. This
/// only compiles if `run_project_capture` auto-prepends the prelude import to
/// imported modules (mirroring the sim's `runProject` / `PRELUDE_IMPORT`) —
/// Wren modules are isolated, so without that, `Osc` would be an undefined
/// variable in `lib/voice`.
mod common;

#[test]
fn multi_file_import_runs() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    let src = "import \"lib/voice\" for Voice\nSystem.print(Voice.tag)\n";
    let lib = (
        "lib/voice".to_string(),
        "class Voice {\n static tag { Osc.sine(1) is Node ? \"v-ok\" : \"v-bad\" }\n}\n"
            .to_string(),
    );
    let out = wren_web_debug::run_project_capture(src, vec![lib]);
    assert!(out.contains("v-ok"), "{out}");
}
