//! Host-only SD-sample streaming prefetch (Sa-3b slice 4). Reads a WAV into RAM
//! and feeds each `StreamPlayer` node's ring window as playback advances. The
//! DEVICE prefetch (async `sd::read_sectors`, no-heap) is slice 5.
#![cfg(not(target_os = "none"))]

use deluge_audio_graph::{NodeId, PoolHandle};

extern crate std;
use std::{string::String, vec::Vec};

/// How far ahead of the read cursor to keep resident (≤ ring_cap).
const LOOKAHEAD: u64 = 6144;

/// Target resident window `[fill_lo, fill_hi)` for a voice whose playback has
/// consumed up to `read_cursor`, given the ring capacity and total sample count
/// (`0` = unknown/unbounded). The window trails the cursor and is ≤ `ring_cap`.
/// Pure, no I/O, `u64`-only (32-bit-`usize` safe).
pub fn plan_window(read_cursor: u64, ring_cap: u64, total: u64) -> (u64, u64) {
    if ring_cap == 0 {
        return (read_cursor, read_cursor);
    }
    let ahead = read_cursor.saturating_add(LOOKAHEAD.min(ring_cap));
    let fill_hi = if total > 0 { ahead.min(total) } else { ahead };
    let width = fill_hi.min(ring_cap); // window ≤ ring_cap
    let fill_lo = fill_hi.saturating_sub(width); // trail the cursor
    (fill_lo, fill_hi)
}

/// One registered streamed node: its ring pool handle, source path, and the
/// decoded PCM (loaded lazily on the first prefetch tick).
pub struct StreamReg {
    pub node: NodeId,
    pub handle: PoolHandle,
    pub path: String,
    pub pcm: Vec<f32>,
    pub total: u64,
    pub loaded: bool,
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    const CAP: u64 = 1000;
    #[test]
    fn window_trails_the_cursor_bounded_by_cap() {
        // Mid-stream: window is ≤ CAP wide and contains the cursor.
        let (lo, hi) = plan_window(5000, CAP, 1_000_000);
        assert!(hi - lo <= CAP);
        assert!(lo <= 5000 && 5000 < hi, "cursor is inside the resident window");
    }
    #[test]
    fn window_clamps_to_total_at_end() {
        let (lo, hi) = plan_window(999_990, CAP, 1_000_000);
        assert_eq!(hi, 1_000_000, "never resident past the file end");
        assert!(hi - lo <= CAP);
    }
    #[test]
    fn window_at_start() {
        let (lo, hi) = plan_window(0, CAP, 1_000_000);
        assert_eq!(lo, 0);
        assert!(hi <= CAP && hi > 0);
    }
    #[test]
    fn unbounded_total_uses_cap() {
        let (lo, hi) = plan_window(10_000, CAP, 0); // total 0 = unknown/unbounded
        assert!(hi - lo <= CAP && lo <= 10_000 && 10_000 < hi);
    }
    #[test]
    fn no_panic_on_huge_cursor() {
        let _ = plan_window(u64::MAX, CAP, u64::MAX);
        let _ = plan_window(u64::MAX, 0, 0); // cap 0
    }
}
