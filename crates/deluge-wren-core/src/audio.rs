//! Control-rate audio-graph commands, emitted by the `Node` bindings.
//!
//! Node ids are handed out by [`NEXT_ID`] so a foreign constructor can return an
//! id immediately without touching the graph; the matching mutation is shipped to
//! the host as a [`Cmd`], which the host transports to its [`Engine`](crate::Engine)
//! (firmware: a critical-section ring drained by the audio task; web: applied
//! directly on the audio thread).

use core::sync::atomic::{AtomicU16, Ordering};

use crate::engine::{Cmd, Input, MAX_NODES};
use crate::host::host;

/// Monotonic node-id allocator (reset by [`reset`]).
static NEXT_ID: AtomicU16 = AtomicU16::new(0);

/// Allocate a node of `kind` with inputs `a`/`b`; returns its id. Returns an
/// out-of-range id (>= `MAX_NODES`) if the pool is full (callers no-op on it).
pub fn alloc_node(kind: u8, a: Input, b: Input) -> u16 {
    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
    if (id as usize) < MAX_NODES {
        host().audio_cmd(Cmd::NewNode { id, kind, a, b });
    }
    id
}
pub fn set_input(id: u16, port: u8, src: Input) {
    host().audio_cmd(Cmd::SetInput { id, port, src });
}
pub fn gate(id: u16, on: bool) {
    host().audio_cmd(Cmd::Gate { id, on });
}
pub fn trigger(id: u16) {
    host().audio_cmd(Cmd::Trigger { id });
}
pub fn set_root(id: u16) {
    host().audio_cmd(Cmd::SetRoot { id });
}
pub fn reset() {
    NEXT_ID.store(0, Ordering::Relaxed);
    host().audio_cmd(Cmd::Reset);
}

/// Reset only the node-id allocator, without touching any host. Used by
/// [`crate::reset`] when a host rebuilds the VM (the engine is rebuilt too).
pub fn reset_ids() {
    NEXT_ID.store(0, Ordering::Relaxed);
}
