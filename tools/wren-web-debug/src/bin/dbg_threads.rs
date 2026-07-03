//! Task 3.2: the threaded-wasm entry module for the debug core.
//!
//! # Why a `bin` (not the cdylib)
//! The spike (`.superpowers/sdd/task-3.2a-spike-report.md`) found that a
//! `wasm32-wasip1-threads` **cdylib** does not bootstrap the *main thread's*
//! TLS / thread pointer on its own (rustc emits no callable `_initialize` for
//! it, and `__wasm_call_ctors` alone is insufficient), so it hangs the first
//! time it touches `std::thread`. A **command** module has a real `_start`
//! (wasi-libc's `crt1-command`) that performs that bootstrap. So the threaded
//! build ships as this `bin`: the host calls `_start` once to initialize the
//! main thread (its empty `main` returns immediately and `_start` calls
//! `proc_exit`, which the host shim turns into a catchable sentinel — see
//! `dbg-worker.mjs`), then calls the exported `dbg_*` functions (which live in
//! the lib, `sab.rs`, and are force-exported by `build-threads.sh`) on the
//! now-initialized instance.
//!
//! The single-threaded Task 3.1 path is untouched: it keeps using the cdylib
//! (`lib.rs`'s `dbg_boot`).

/// `_start`'s body. Empty on purpose: its only job is to let wasi-libc's
/// `crt1-command` startup run (which initializes the main thread's TLS), after
/// which the host drives the exported `dbg_*` functions.
fn main() {
    // Reference the lib's threaded exports so the linker keeps them in the bin
    // (they are `--export`ed by build-threads.sh; this also documents the
    // dependency). No-op at runtime — `_start` returns immediately.
    #[cfg(all(target_arch = "wasm32", target_feature = "atomics"))]
    {
        let _ = wren_web_debug::sab::dbg_sab_ptr as usize;
        let _ = wren_web_debug::sab::dbg_alloc as usize;
        let _ = wren_web_debug::sab::dbg_launch as usize;
    }
}
