//! Engine-owned per-`StreamPlayer`-node fill cursors (produced by the prefetch
//! task, consumed at render). Kept out of `Node`/`State` (which are `Copy` and
//! hold kernel state) — see the Sa-3b slice-3 design.

use deluge_dsp_kernels::poly::VOICES;

/// Per-voice resident-window cursors for one streaming node.
#[derive(Clone, Copy)]
pub struct StreamCursors {
    /// Full sample length in samples (`0` = unbounded). Same file → one value.
    pub total: u64,
    /// Per-voice resident window `[fill_lo, fill_hi)` of absolute sample indices.
    pub fill: [(u64, u64); VOICES],
}

impl StreamCursors {
    pub fn new() -> Self {
        StreamCursors {
            total: 0,
            fill: [(0, 0); VOICES],
        }
    }
}

impl Default for StreamCursors {
    fn default() -> Self {
        StreamCursors::new()
    }
}
