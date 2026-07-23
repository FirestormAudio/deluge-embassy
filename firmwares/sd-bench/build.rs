use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    // Select the memory layout and linker script that match the rtt feature.
    // rza1l-hal's build.rs places rza1l.x / rza1l_rtt.x on the search path;
    // we just need to put the matching memory.x alongside them.
    let rtt = env::var("CARGO_FEATURE_RTT").is_ok();
    let (memory_src, linker_script) = if rtt {
        ("memory_rtt.x", "rza1l_rtt.x")
    } else {
        ("memory.x", "rza1l.x")
    };

    fs::copy(manifest_dir.join(memory_src), out_dir.join("memory.x")).unwrap();

    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rustc-link-arg=-T{linker_script}");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=memory_rtt.x");
}
