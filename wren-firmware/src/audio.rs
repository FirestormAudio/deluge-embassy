//! Firmware audio transport: the control→audio command ring, the engine
//! instance, and the render task.
//!
//! The DSP itself (the node graph, the [`Engine`], the [`Cmd`] vocabulary) lives
//! in `deluge_wren_core` so the device and the web simulator share one signal
//! engine. This module is the firmware-specific plumbing around it:
//!
//! ## Concurrency
//! The [`Engine`] is rendered by [`audio_task`] every block. `vm_task` (via the
//! `Node` bindings → [`FwHost::audio_cmd`](crate::host::FwHost)) enqueues
//! [`Cmd`]s onto [`CMD_RING`]; `audio_task` drains them between render blocks.
//! Node ids are handed out by the core's allocator so a foreign ctor can return
//! an id immediately without touching the graph.
//!
//! [`upload_table`] is the one exception to "audio_task is the sole accessor":
//! it's called synchronously from `vm_task` (via
//! [`FwHost::upload_table`](crate::host::FwHost)) and needs an immediate
//! `PoolHandle` back, so it reaches into [`ENGINE`] directly rather than going
//! through the command ring. [`ENGINE`] is therefore a critical-section-guarded
//! [`Mutex`] (like [`CMD_RING`]) instead of a task-owned static. Every lock
//! scope is short — pool bookkeeping, or one block's worth of render — *never*
//! the mip-pyramid synthesis itself (FFT + additive resynthesis can run far
//! longer than a block's real-time budget), which [`upload_table`] does
//! into a scratch buffer with no lock held. This costs an occasional brief
//! IRQ-latency bump on table upload, not a real-time hazard.

use core::cell::RefCell;
use core::ptr::addr_of_mut;

use deluge::Audio;
use deluge_audio_graph::{Engine, PoolHandle};
use deluge_wren_core::Cmd;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};

// Sizes satisfy the binding contract: NODES >= WREN_MAX_NODES(64), BUSES >= 8.
type Eng = Engine<32, 64, 128, 8, 90112, 2048>; // BLOCK, NODES, OUTS, BUSES, PCAP, PCHUNK
const _: () = assert!(64 >= deluge_wren_core::WREN_MAX_NODES && 8 >= deluge_wren_core::WREN_MAX_BUSES);
const SAMPLE_RATE: f32 = 44_100.0;

/// The audio engine. `None` until `audio_task` creates it; see the module docs
/// for why this is a lock rather than a task-owned static.
static ENGINE: Mutex<CriticalSectionRawMutex, RefCell<Option<Eng>>> = Mutex::new(RefCell::new(None));

// ── Control → audio command queue ────────────────────────────────────────────

const CMD_CAP: usize = 256;

struct CmdRing {
    buf: [Cmd; CMD_CAP],
    head: usize,
    tail: usize,
    full: bool,
}

impl CmdRing {
    const fn new() -> Self {
        CmdRing { buf: [Cmd::Nop; CMD_CAP], head: 0, tail: 0, full: false }
    }
    fn push(&mut self, c: Cmd) {
        if self.full {
            return; // drop on overflow (control-rate; shouldn't happen)
        }
        self.buf[self.tail] = c;
        self.tail = (self.tail + 1) % CMD_CAP;
        if self.tail == self.head {
            self.full = true;
        }
    }
    fn pop(&mut self) -> Option<Cmd> {
        if self.head == self.tail && !self.full {
            return None;
        }
        let c = self.buf[self.head];
        self.head = (self.head + 1) % CMD_CAP;
        self.full = false;
        Some(c)
    }
}

static CMD_RING: Mutex<CriticalSectionRawMutex, RefCell<CmdRing>> =
    Mutex::new(RefCell::new(CmdRing::new()));

/// Enqueue a control-rate command (called from the `Node` bindings via the host).
pub fn submit(c: Cmd) {
    CMD_RING.lock(|r| r.borrow_mut().push(c));
}

// ── Render task ──────────────────────────────────────────────────────────────

/// Renders the DSP graph through the SDK [`Audio`] block callback. Each block:
/// drain the control-rate command queue into the engine, then render the SDK
/// block in engine-`BLOCK`-sized (32-frame) chunks, copying into the SDK's
/// stereo frames. Output is `[-1.0, 1.0]`; the SDK handles codec scaling and the
/// SSI TX/RX DMA cadence (poll loop, or the per-block RX interrupt under the
/// `audio-irq` feature).
#[embassy_executor::task]
pub async fn audio_task(audio: Audio) {
    ENGINE.lock(|c| *c.borrow_mut() = Some(Eng::new(SAMPLE_RATE)));

    audio
        .process(|block: &mut [deluge::StereoFrame]| {
            ENGINE.lock(|c| {
                let mut guard = c.borrow_mut();
                // The lock above serializes with `upload_table`'s pool-alloc /
                // pool-copy phases; this fn is the only place that renders, and
                // ENGINE was set to `Some` immediately above, before this
                // closure could ever run.
                let eng = guard.as_mut().expect("ENGINE initialized at audio_task start");
                // Apply all pending control-rate commands.
                while let Some(c) = CMD_RING.lock(|r| r.borrow_mut().pop()) {
                    eng.apply(c);
                }
                // Render the SDK block in engine-BLOCK-sized chunks. The SDK's
                // `StereoFrame` is a distinct (host-vs-device) type from the
                // engine's, so render into a local scratch buffer and copy.
                // Chunk size must equal the engine's BLOCK (32) so `render`
                // fills each chunk fully.
                let mut scratch = [deluge_audio_graph::StereoFrame::default(); 32];
                for chunk in block.chunks_mut(32) {
                    let out = &mut scratch[..chunk.len()];
                    eng.render(out);
                    // `Engine::render` already clamps its output to [-1, 1]; plain copy.
                    for (dst, src) in chunk.iter_mut().zip(out.iter()) {
                        dst.l = src.l;
                        dst.r = src.r;
                    }
                }
            });
        })
        .await
}

// ── Table upload (bulk-data seam, Task 4 of Osc 3b) ─────────────────────────

/// Scratch region for the (slow) mip-pyramid build in [`upload_table`], kept
/// outside the [`ENGINE`] lock so the FFT + additive resynthesis never runs
/// with interrupts disabled. Single-writer: `upload_table` is called
/// synchronously from a Wren foreign method (`vm_task`), and foreign methods
/// run to completion before the VM can be re-entered, so there is never more
/// than one call in flight.
static mut UPLOAD_SCRATCH: [f32; mipgen::N * mipgen::LEVELS] = [0.0; mipgen::N * mipgen::LEVELS];

/// Build a band-limited mip pyramid from `base` into a freshly-allocated pool
/// region of the audio engine, and return its handle. Called from
/// [`crate::host::FwHost::upload_table`]. `None` if `audio_task` hasn't
/// created the engine yet, or the pool is exhausted.
pub fn upload_table(base: &[f32]) -> Option<PoolHandle> {
    // Phase 1: allocate (fast, bitmap-only bookkeeping) under the engine lock.
    let h = ENGINE.lock(|c| c.borrow_mut().as_mut()?.pool_alloc(mipgen::N * mipgen::LEVELS))?;
    // Phase 2: the expensive synthesis, with interrupts enabled and no lock
    // held — see the module docs and `UPLOAD_SCRATCH`'s doc comment.
    // SAFETY: single writer (see `UPLOAD_SCRATCH`'s doc comment above).
    let scratch = unsafe { &mut *addr_of_mut!(UPLOAD_SCRATCH) };
    deluge_wren_core::build_pyramid_into(base, scratch);
    // Phase 3: copy the finished pyramid into the pool region under a short lock.
    ENGINE.lock(|c| {
        if let Some(eng) = c.borrow_mut().as_mut() {
            eng.pool_slice_mut(h).copy_from_slice(scratch);
        }
    });
    Some(h)
}
