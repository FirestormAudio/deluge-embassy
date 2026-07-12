//! `wren-firmware`'s lib target exists only to make the host-only [`stream`]
//! module unit-testable via `cargo test -p wren-firmware --lib`; all product
//! behavior lives in `main.rs` (the `[[bin]]` target). Nothing depends on this
//! crate as a library.
//!
//! [`stream`] is compiled TWICE — once here (so `plan_window` is
//! `--lib`-testable) and once into the `[[bin]]` via `main.rs`'s `mod
//! stream;`. The two compilations are entirely independent (distinct
//! `ENGINE`/`REGISTRY` statics, never linked together); `stream.rs`'s
//! `stream_task`/`register` reach a `crate::audio` that must therefore exist
//! in *this* crate root too — hence `pub mod audio;` below, the same file
//! `main.rs` also declares as `mod audio;` for the bin. Product behavior
//! (spawning the task, wiring `Host`) still lives only in `main.rs`/`host.rs`,
//! which are not part of this lib target.
#![cfg_attr(target_os = "none", no_std)]
// Only needed to expand `#[embassy_executor::task]` on `audio::audio_task`/
// `stream::stream_task`, both host-only — unused (and would warn) on device,
// where this lib target is otherwise empty.
#![cfg_attr(not(target_os = "none"), feature(impl_trait_in_assoc_type))]

#[cfg(not(target_os = "none"))]
pub mod audio;
#[cfg(not(target_os = "none"))]
pub mod stream;
