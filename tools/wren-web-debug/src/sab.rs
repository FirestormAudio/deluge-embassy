//! Task 3.2: a [`DebugTransport`] backed by a `SharedArrayBuffer` (a region of
//! the wasm32-wasip1-threads shared linear memory), so the *same* [`serve`]
//! driver loop that runs over an in-process `mpsc` pair in the native tests
//! (`tests/agent_transport.rs`) runs unchanged in a wasm worker, driven from
//! the JS main thread over shared memory.
//!
//! # Why this module only exists for the threads build
//! It uses `core::arch::wasm32::{memory_atomic_wait32, memory_atomic_notify}`
//! — the wasm `atomic.wait`/`atomic.notify` instructions — which are only
//! available when the module is compiled with the `atomics` target feature
//! (i.e. `wasm32-wasip1-threads`). So the whole module is gated on
//! `all(target_arch = "wasm32", target_feature = "atomics")` (see `lib.rs`):
//! the native rlib and the single-threaded `wasm32-wasip1` (Task 3.1) build
//! never see it and are unaffected.
//!
//! # SAB layout (SHARED CONTRACT with the JS controller — Task 3.3 must match
//! this byte-for-byte)
//! One contiguous region, base = [`sab_base`] (a zero-initialized `static`,
//! whose address the host reads via the exported `dbg_sab_ptr`):
//!
//! | bytes     | i32 idx | name        | writer  | meaning                                   |
//! |-----------|---------|-------------|---------|-------------------------------------------|
//! | 0..4      | 0       | `CMD_SEQ`   | main    | bumped when a command JSON is published   |
//! | 4..8      | 1       | `CMD_ACK`   | worker  | set to `CMD_SEQ` once the command is read |
//! | 8..12     | 2       | `CMD_LEN`   | main    | byte length of the command JSON           |
//! | 12..16    | 3       | `EVENT_SEQ` | worker  | bumped when an event JSON is published    |
//! | 16..20    | 4       | `EVENT_ACK` | main    | set to `EVENT_SEQ` once the event is read |
//! | 20..24    | 5       | `EVENT_LEN` | worker  | byte length of the event JSON             |
//! | 24..28    | 6       | `PAUSE`     | main    | pause request (reserved; see below)       |
//! | 28..32    | 7       | reserved    | —       | padding / future use                      |
//! | 32..8224  | —       | CMD region  | main    | command JSON bytes (main → worker)        |
//! | 8224..16416 | —     | EVENT region| worker  | event JSON bytes (worker → main)          |
//!
//! Both directions use a seq/ack handshake so a slow reader can never be
//! overwritten: the writer bumps `*_SEQ` (and `atomic.notify`s it) only after
//! the reader has acked the previous message via `*_ACK` (which the writer
//! `atomic.wait`s on). This makes [`SabTransport::send_event`] block until the
//! controller has drained the prior event — the SAB analogue of an unbuffered
//! channel, and exactly what lets `serve`'s back-to-back events
//! (`Terminated` + `Exited`; `Output…` + `Stopped`) survive a single JSON
//! slot without a queue.
//!
//! # `PAUSE`
//! Word 6 is reserved for an asynchronous pause request the VM's
//! `wren_core::vm::Pause` handle would poll (route pause *here*, not as a
//! `DebugCmd` through `serve`). This task wires the *layout* slot so the 3.3
//! TS controller and a future paused-VM build agree on the offset; the current
//! [`debug_run`] builds the session without a `Pause` handle, so nothing polls
//! it yet. See the task report.
//!
//! [`serve`]: crate::transport::serve
//! [`DebugTransport`]: crate::transport::DebugTransport
//! [`debug_run`]: crate::agent::debug_run

use core::arch::wasm32::{memory_atomic_notify, memory_atomic_wait32};
use core::cell::Cell;
use core::sync::atomic::{AtomicI32, Ordering};

use crate::transport::{DebugCmd, DebugEvent, DebugTransport};

// ── layout constants (keep in sync with the table above and 3.3's TS) ──
/// Number of i32 words in the control header.
pub const HEADER_WORDS: usize = 8;
const IDX_CMD_SEQ: usize = 0;
const IDX_CMD_ACK: usize = 1;
const IDX_CMD_LEN: usize = 2;
const IDX_EVENT_SEQ: usize = 3;
const IDX_EVENT_ACK: usize = 4;
const IDX_EVENT_LEN: usize = 5;
#[allow(dead_code)]
const IDX_PAUSE: usize = 6;

/// Byte offset of the command-JSON region (main → worker).
pub const CMD_REGION_OFF: usize = HEADER_WORDS * 4; // 32
/// Capacity in bytes of each JSON region.
pub const REGION_CAP: usize = 8192;
/// Byte offset of the event-JSON region (worker → main).
pub const EVENT_REGION_OFF: usize = CMD_REGION_OFF + REGION_CAP; // 8224
/// Total byte size of the SAB region.
pub const SAB_BYTES: usize = CMD_REGION_OFF + REGION_CAP * 2; // 16416

/// The backing store for the SAB region: a zero-initialized, 8-byte-aligned
/// static living in the shared linear memory. Zero-init means it needs no
/// data-segment/`memory.init` to be valid — wasm linear memory starts zeroed —
/// so the controller can read [`dbg_sab_ptr`]'s address before any thread runs
/// module init.
#[repr(align(8))]
// The payload is only ever touched through raw pointers at computed offsets
// (per-word atomics / byte copies), never via this field — so it reads as dead.
struct SabBuf(#[allow(dead_code)] [u8; SAB_BYTES]);

static mut SAB: SabBuf = SabBuf([0; SAB_BYTES]);

/// Base address (in the shared linear memory) of the SAB region. The JS host
/// reads this via the exported `dbg_sab_ptr` to locate the header/regions in
/// `memory.buffer`.
pub fn sab_base() -> usize {
    // Address of the static; taking a raw pointer to a `static mut` is sound
    // (we never form a reference to the whole buffer — all access is through
    // per-word atomics / byte copies at computed offsets).
    &raw const SAB as *const u8 as usize
}

#[inline]
fn word(base: usize, idx: usize) -> &'static AtomicI32 {
    // Each header word is 4-byte aligned within an 8-byte-aligned base.
    unsafe { &*((base + idx * 4) as *const AtomicI32) }
}

#[inline]
fn word_ptr(base: usize, idx: usize) -> *mut i32 {
    (base + idx * 4) as *mut i32
}

/// Block until `word(base, idx)` no longer equals `expected` (a notify or a
/// spurious wakeup returns; the caller re-reads and loops). `-1` timeout =
/// wait forever.
#[inline]
fn wait_while_eq(base: usize, idx: usize, expected: i32) {
    unsafe {
        // Returns 0 (woken), 1 (value already != expected), 2 (timed out —
        // impossible with -1). In all cases the caller re-checks the value.
        let _ = memory_atomic_wait32(word_ptr(base, idx), expected, -1);
    }
}

#[inline]
fn notify(base: usize, idx: usize) {
    unsafe {
        // Wake every waiter parked on this address (there is at most one: the
        // single controller for events, the single worker for acks).
        let _ = memory_atomic_notify(word_ptr(base, idx), u32::MAX);
    }
}

/// A [`DebugTransport`] over the shared-memory region at `base`.
///
/// Only ever touched from the one thread that runs [`serve`] (the worker's
/// module-main thread) — the VM runs on a *different* thread but never sees
/// this object — so the `Cell` interior mutability for the last-seen sequence
/// is sound without a lock.
pub struct SabTransport {
    base: usize,
    last_cmd_seq: Cell<i32>,
}

impl SabTransport {
    /// Wrap the SAB region based at `base` (typically [`sab_base`]).
    pub fn new(base: usize) -> Self {
        SabTransport { base, last_cmd_seq: Cell::new(0) }
    }
}

impl DebugTransport for SabTransport {
    fn recv_cmd(&self) -> Option<DebugCmd> {
        let base = self.base;
        loop {
            let seq = word(base, IDX_CMD_SEQ).load(Ordering::Acquire);
            if seq == self.last_cmd_seq.get() {
                // No new command yet — park on CMD_SEQ until the controller
                // bumps it. Re-loop on wakeup (handles spurious wakeups and
                // the value-already-changed race).
                wait_while_eq(base, IDX_CMD_SEQ, seq);
                continue;
            }
            self.last_cmd_seq.set(seq);

            let len = word(base, IDX_CMD_LEN).load(Ordering::Acquire) as usize;
            let cmd = read_region(base, CMD_REGION_OFF, len)
                .and_then(|bytes| serde_json::from_slice::<DebugCmd>(bytes).ok());

            // Ack: tell the controller the CMD slot is free to reuse.
            word(base, IDX_CMD_ACK).store(seq, Ordering::Release);
            notify(base, IDX_CMD_ACK);

            // A malformed/zero-length command means "no more" → stop serve's
            // loop (mirrors the mpsc transport's `recv().ok()` EOF).
            return cmd;
        }
    }

    fn send_event(&self, ev: DebugEvent) {
        let base = self.base;
        let json = serde_json::to_vec(&ev).unwrap_or_else(|_| b"{}".to_vec());
        let len = json.len().min(REGION_CAP);

        // Wait until the controller has acked the previous event (EVENT_ACK ==
        // EVENT_SEQ) so we never clobber an unread event JSON slot.
        loop {
            let seq = word(base, IDX_EVENT_SEQ).load(Ordering::Acquire);
            let ack = word(base, IDX_EVENT_ACK).load(Ordering::Acquire);
            if ack == seq {
                break;
            }
            wait_while_eq(base, IDX_EVENT_ACK, ack);
        }

        write_region(base, EVENT_REGION_OFF, &json[..len]);
        word(base, IDX_EVENT_LEN).store(len as i32, Ordering::Release);
        let next = word(base, IDX_EVENT_SEQ).load(Ordering::Relaxed).wrapping_add(1);
        word(base, IDX_EVENT_SEQ).store(next, Ordering::Release);
        notify(base, IDX_EVENT_SEQ);
    }
}

// ── C-ABI exports (the host drives these after `_start` init) ────────────
//
// These live in the LIB (not the `dbg_threads` bin) so the threaded build's
// `--export=<name>` link args resolve against a symbol that exists in every
// artifact being linked (both the co-built cdylib and the bin). The runtime
// host uses the BIN (`dbg_threads.wasm`), which additionally has the `_start`
// that bootstraps the main thread's TLS (see `src/bin/dbg_threads.rs`).

/// Allocate `len` bytes in the wasm heap and return a pointer the host can
/// write into (entry script, breakpoint-line array). Leaks — one-shot launch.
///
/// # Safety
/// C-ABI export; the returned pointer is valid for `len` bytes.
#[unsafe(no_mangle)]
pub extern "C" fn dbg_alloc(len: usize) -> *mut u8 {
    // Allocate as `u64` so the returned pointer is 8-byte aligned — the host
    // writes an `i32` breakpoint array here that `dbg_launch` reads back as a
    // `&[i32]`, which requires 4-byte alignment. Rounds up to whole words.
    let words = len.div_ceil(8).max(1);
    let mut v = Vec::<u64>::with_capacity(words);
    let p = v.as_mut_ptr() as *mut u8;
    core::mem::forget(v);
    p
}

/// Base address of the SAB control region so the host can map the header/JSON
/// regions in `memory.buffer`.
#[unsafe(no_mangle)]
pub extern "C" fn dbg_sab_ptr() -> *const u8 {
    sab_base() as *const u8
}

/// Launch a debug session and drive it over the SAB transport. Spawns the VM on
/// its own wasm thread (via [`crate::agent::debug_run`]) breaking on each line
/// in `bp_ptr[..bp_count]`, then runs the same `serve` loop the native tests use
/// — reading `DebugCmd`s from and writing `DebugEvent`s to shared memory.
/// **Blocks** until the session terminates (the host calls this on a worker
/// thread; the JS main thread drives the SAB concurrently).
///
/// # Safety
/// `entry_ptr[..entry_len]` must be valid UTF-8; `bp_ptr[..bp_count]` a valid
/// `i32` array. Both consumed synchronously here.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dbg_launch(
    entry_ptr: *const u8,
    entry_len: usize,
    bp_ptr: *const i32,
    bp_count: usize,
) -> i32 {
    let entry = {
        let bytes = unsafe { core::slice::from_raw_parts(entry_ptr, entry_len) };
        match core::str::from_utf8(bytes) {
            Ok(s) => s.to_string(),
            Err(_) => return 1,
        }
    };
    let breakpoints: std::collections::HashSet<i32> = if bp_count == 0 {
        std::collections::HashSet::new()
    } else {
        let lines = unsafe { core::slice::from_raw_parts(bp_ptr, bp_count) };
        lines.iter().copied().collect()
    };

    // Spawn the VM thread (parks at the breakpoint) — Task 2.1 code, unchanged.
    let session = crate::agent::debug_run(&entry, Vec::new(), breakpoints);
    // Drive it over shared memory with the exact same `serve` loop the native
    // `mpsc` test uses — only the transport differs.
    let transport = SabTransport::new(sab_base());
    crate::transport::serve(&session, &transport);
    0
}

fn read_region(base: usize, off: usize, len: usize) -> Option<&'static [u8]> {
    if len == 0 || len > REGION_CAP {
        return None;
    }
    Some(unsafe { core::slice::from_raw_parts((base + off) as *const u8, len) })
}

fn write_region(base: usize, off: usize, bytes: &[u8]) {
    unsafe {
        core::ptr::copy_nonoverlapping(bytes.as_ptr(), (base + off) as *mut u8, bytes.len());
    }
}
