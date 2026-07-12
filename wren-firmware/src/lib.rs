//! `wren-firmware`'s lib target exists only to make the host-only [`stream`]
//! module unit-testable via `cargo test -p wren-firmware --lib`; all product
//! behavior lives in `main.rs` (the `[[bin]]` target). Nothing depends on this
//! crate as a library.
#![cfg_attr(target_os = "none", no_std)]

#[cfg(not(target_os = "none"))]
pub mod stream;
