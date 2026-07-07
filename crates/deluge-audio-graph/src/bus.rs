//! Stereo buses (P0 model).
//!
//! A bus is a stereo accumulator: `bus_l`/`bus_r`, one `[f32; BLOCK]` row per
//! bus index, owned directly on [`crate::Engine`] (there is no separate `Bus`
//! struct in P0 — see the engine's `bus_l`/`bus_r` fields).
//!
//! [`crate::Engine::bus_write`] records a `(src, bus)` pair in a small
//! fixed-capacity list; the list is re-applied (not consumed) on every
//! [`crate::Engine::render`] call, so a write recorded once stays in effect
//! for subsequent blocks until a full routing/patch table replaces this list
//! representation in a later sub-project.
//!
//! Each `render`:
//! 1. Zeroes every bus's `bus_l`/`bus_r` rows.
//! 2. Evaluates all live nodes via `render_block`.
//! 3. Sums each recorded write's source into its bus, center-panned
//!    (`L += v; R += v`); `Pan` and per-write gain are deferred to the `IO`
//!    sub-project.
//! 4. Copies the bus selected by [`crate::Engine::set_root`] into the output
//!    `StereoFrame`s, clamped to `[-1.0, 1.0]`.
