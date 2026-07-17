//! Host-only SD-sample streaming prefetch (Sa-3b slice 4). Reads a WAV into RAM
//! and feeds each `StreamPlayer` node's ring window as playback advances. The
//! DEVICE prefetch (async `sd::read_sectors`, no-heap) is slice 5.
#![cfg(not(target_os = "none"))]

use deluge_audio_graph::{Cmd, NodeId, PoolHandle, VOICES};

extern crate std;
use std::{string::String, vec::Vec};

use core::cell::RefCell;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Timer};

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

// ── Registry + prefetch task ─────────────────────────────────────────────────

/// Streamed nodes registered via [`register`] (called from
/// `FwHost::stream_register`, i.e. `Sample.stream`'s `Node.stream_` binding).
/// Drained/serviced each tick by [`stream_task`]. Mirrors `audio.rs`'s
/// `CMD_RING` `Mutex<CriticalSectionRawMutex, RefCell<...>>` static style —
/// the one cooperative executor never contends this lock across a yield.
static REGISTRY: Mutex<CriticalSectionRawMutex, RefCell<Vec<StreamReg>>> =
    Mutex::new(RefCell::new(Vec::new()));

/// Register a `Kind::StreamPlayer` node's ring + source path with the host
/// prefetch. The WAV itself is loaded+decoded lazily, on `stream_task`'s first
/// tick after registration (never inline here — this runs synchronously from
/// the Wren foreign call).
pub fn register(node: NodeId, handle: PoolHandle, path: &str) {
    REGISTRY.lock(|r| {
        r.borrow_mut().push(StreamReg {
            node,
            handle,
            path: String::from(path),
            pcm: Vec::new(),
            total: 0,
            loaded: false,
        })
    });
}

/// The simulated SD-card root directory (`DELUGE_SIM_SD` env var, default
/// `./sim-sd`), joined with `path`. Mirrors `deluge_sdk::sd`'s private
/// `sim_sd_root` (duplicated rather than depended on — that crate's `Sd::read`
/// is a fixed-buffer root-file API, not what a `Vec`-growing WAV load wants,
/// and this prefetch is bin-local sim-only code).
fn sim_sd_join(path: &str) -> std::path::PathBuf {
    std::env::var_os("DELUGE_SIM_SD")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::path::PathBuf::from("sim-sd"))
        .join(path)
}

/// Host-only WAV prefetch: every 20 ms, for each registered streamed node,
/// lazily load+decode its WAV (once, on the first tick after registration),
/// then recompute and push each voice's resident ring window
/// (`plan_window`, Task 2) from its current playback read-cursor.
///
/// A bad path or unparsable WAV leaves `pcm` empty and `total` at `0` — the
/// node just never advances past silence (`Engine::stream_read_cursor` stays
/// `0`, `plan_window(0, cap, 0)` keeps re-planning the same empty-ish window,
/// and `pcm.get(a).unwrap_or(0.0)` always reads zero); no panic either way.
///
/// The per-voice full-window rewrite each tick is O(ring_cap) — acceptable
/// for the sim; the device prefetch (slice 5) will instead write only the
/// newly-exposed `[old_fill_hi, hi)` delta each tick.
///
/// ## Concurrency
/// Runs on the same one cooperative embassy executor as `audio_task`/`vm_task`
/// (see `audio.rs`'s `## Concurrency`). This task's body is synchronous
/// between `.await`s (the `REGISTRY.lock` closure and every
/// `crate::audio::*` call inside it never yield), so its `assume_init_ref`
/// engine borrows (via `stream_read_cursor`/`pool_len`/`pool_write`) never
/// overlap `audio_task`'s per-block `assume_init_mut` borrow — same argument
/// as `upload_table`'s.
#[embassy_executor::task]
pub async fn stream_task() {
    loop {
        Timer::after(Duration::from_millis(20)).await;
        REGISTRY.lock(|r| {
            for reg in r.borrow_mut().iter_mut() {
                if !reg.loaded {
                    if let Ok(bytes) = std::fs::read(sim_sd_join(&reg.path)) {
                        if let Ok(info) = deluge_dsp_kernels::wav::parse(&bytes) {
                            let n_samples = info.data_len / 2;
                            reg.pcm.resize(n_samples, 0.0);
                            let end = (info.data_offset + info.data_len).min(bytes.len());
                            if info.data_offset <= end {
                                deluge_dsp_kernels::wav::decode_i16_le(
                                    &bytes[info.data_offset..end],
                                    &mut reg.pcm,
                                );
                            }
                            reg.total = n_samples as u64;
                        }
                    }
                    reg.loaded = true; // bad path/WAV → pcm stays empty, total 0 (silence)
                }

                let cap = (crate::audio::pool_len(reg.handle) / VOICES) as u64;
                if cap == 0 {
                    continue; // unbound/empty ring — nothing to fill
                }
                for v in 0..VOICES {
                    let rc = crate::audio::stream_read_cursor(reg.node, v).unwrap_or(0);
                    let (lo, hi) = plan_window(rc, cap, reg.total);
                    // Write pcm[lo..hi) into voice v's sub-ring
                    // `region[v*cap..(v+1)*cap]` at offset `a % cap` — mirrors
                    // `Kind::StreamPlayer`'s render-side indexing exactly (see
                    // `deluge_audio_graph::node`'s `Kind::StreamPlayer` arm).
                    for a in lo..hi {
                        let val = reg.pcm.get(a as usize).copied().unwrap_or(0.0);
                        let idx = (v as u64 * cap + (a % cap)) as usize;
                        crate::audio::pool_write(reg.handle, idx, val);
                    }
                    crate::audio::submit(Cmd::StreamFill {
                        node: reg.node,
                        voice: v as u8,
                        fill_lo: lo,
                        fill_hi: hi,
                        total: reg.total,
                    });
                }
            }
        });
    }
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
        assert!(
            lo <= 5000 && 5000 < hi,
            "cursor is inside the resident window"
        );
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
