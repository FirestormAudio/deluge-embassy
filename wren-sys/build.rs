//! Compiles the upstream `wren-lang/wren` C VM into a static archive for the
//! Deluge firmware.
//!
//! Unlike `wren-rs`'s own build, we compile the **stock** VM *including* its
//! built-in C compiler (`wren_compiler.c`) and apply **no** source patches, so
//! Wren source compiles on-device — giving a real crow-style live-coding REPL.
//! This mirrors the proven `crow-sys` recipe (`arm-none-eabi-gcc`, cortex-a9
//! hard-float, function/data sections, no unwind tables).
//!
//! The wren checkout is located via `WREN_SRC` (default the `ext/wren`
//! submodule inside the sibling `wren-rs` repo).

use std::env;
use std::path::PathBuf;

/// Stock upstream VM translation units (`src/vm/`). Includes `wren_compiler.c`
/// (the on-device compiler) — the whole point of compiling source on the Deluge.
const VM_FILES: &[&str] = &[
    "wren_compiler.c",
    "wren_core.c",
    "wren_debug.c",
    "wren_primitive.c",
    "wren_utils.c",
    "wren_value.c",
    "wren_vm.c",
];

/// Optional modules (`src/optional/`), enabled via `WREN_OPT_*` defines.
const OPT_FILES: &[&str] = &["wren_opt_meta.c", "wren_opt_random.c"];

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    let wren = env::var("WREN_SRC")
        .map(PathBuf::from)
        .unwrap_or_else(|_| manifest.join("../../wren-rs/ext/wren"));
    let vm = wren.join("src/vm");
    let optional = wren.join("src/optional");
    let include = wren.join("src/include");
    let csrc = manifest.join("csrc");

    assert!(
        vm.join("wren_vm.c").exists(),
        "upstream wren sources not found at {} — set WREN_SRC or run \
         `git submodule update --init` in the wren-rs repo",
        vm.display()
    );

    // The device (`target_os = "none"`) cross-compiles the VM with the bare ARM
    // toolchain; the host build (the desktop simulator) lets `cc` pick the
    // default host compiler so the VM runs in-process. `CARGO_CFG_TARGET_OS` is
    // "none" only for the embedded triple.
    let embedded = env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("none");
    // The web simulator builds the VM to wasm. The Rust crate targets
    // `wasm32-unknown-unknown` (for wasm-bindgen), but the C VM needs a libc —
    // so the C is compiled against a wasi-sdk sysroot and wasi-libc/libm are
    // linked into the final module (its only host imports are four WASI stdio
    // calls; see docs/web-editor-plan.md §5). Point `WASI_SYSROOT` at an
    // extracted wasi-sysroot.
    let wasm = env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("wasm32");

    let mut build = cc::Build::new();

    for f in VM_FILES {
        build.file(vm.join(f));
    }
    for f in OPT_FILES {
        build.file(optional.join(f));
    }

    build
        .include(&include)
        .include(&vm)
        .include(&optional)
        // Enable the optional Meta + Random modules bundled with the VM.
        .define("WREN_OPT_META", "1")
        .define("WREN_OPT_RANDOM", "1")
        // NOTE: deliberately NOT `-fsingle-precision-constant` (which crow uses
        // for its LUA_32BITS single-precision Lua). Wren is a strictly f64/double
        // VM with NaN-tagging; truncating double constants miscompiles value
        // handling. `-fno-strict-aliasing` keeps the double<->uint64 type-punning
        // in NaN-tagging well-behaved under optimization.
        .flag("-fno-strict-aliasing")
        .opt_level(2)
        .warnings(false);

    if embedded {
        // newlib support: _sbrk arena (for snprintf's incidental malloc) + ARM
        // EH stubs. The host libc supplies all of this itself, and redefining
        // `_sbrk` there would clash with glibc — so this is device-only.
        build.file(csrc.join("csupport.c"));
        build
            // Cortex-A9 hard-float, matching the Rust target ABI (see crow-sys).
            .flag("-mcpu=cortex-a9")
            .flag("-mfloat-abi=hard")
            .flag("-mfpu=neon-vfpv3")
            .flag("-ffunction-sections")
            .flag("-fdata-sections")
            .flag("-fno-unwind-tables")
            .flag("-fno-asynchronous-unwind-tables")
            .flag("-fno-common")
            .compiler("arm-none-eabi-gcc")
            .archiver("arm-none-eabi-ar");
    }

    if wasm {
        let sysroot = env::var("WASI_SYSROOT").expect(
            "wasm build of wren-sys needs WASI_SYSROOT pointing at a wasi-sdk \
             sysroot (see docs/web-editor-plan.md §5)",
        );
        // Override cc's `--target=wasm32-unknown-unknown` (no libc) with the wasi
        // triple so the C resolves `<string.h>`/`<math.h>`/… against wasi-libc;
        // the emitted wasm objects still link into the unknown-unknown module.
        build
            .flag("--target=wasm32-wasi")
            .flag(format!("--sysroot={sysroot}"))
            .flag("-ffunction-sections")
            .flag("-fdata-sections");
        // Link wasi-libc + libm for the C runtime (realloc/free, memcpy, the math
        // intrinsics, and the stdio used by wren_debug / error formatting).
        println!("cargo:rustc-link-search=native={sysroot}/lib/wasm32-wasi");
        println!("cargo:rustc-link-lib=static=c");
        println!("cargo:rustc-link-lib=static=m");
        println!("cargo:rerun-if-env-changed=WASI_SYSROOT");
    }

    build.compile("wrencore");

    for f in VM_FILES {
        println!("cargo:rerun-if-changed={}", vm.join(f).display());
    }
    for f in OPT_FILES {
        println!("cargo:rerun-if-changed={}", optional.join(f).display());
    }
    println!("cargo:rerun-if-changed={}", csrc.display());
    println!("cargo:rerun-if-env-changed=WREN_SRC");
}
