use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    for script in &["rza1l.x", "rza1l_rtt.x"] {
        fs::copy(manifest_dir.join(script), out_dir.join(script)).unwrap();
        println!("cargo:rerun-if-changed={script}");
    }

    // Make both scripts findable on the linker search path.  The binary crate's
    // build.rs selects which one to apply via -T and provides the matching
    // memory.x alongside it.
    println!("cargo:rustc-link-search={}", out_dir.display());
}
