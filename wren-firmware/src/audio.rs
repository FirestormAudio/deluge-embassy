//! Firmware audio transport: the control→audio command ring, the engine
//! instance, and the render task.
//!
//! The DSP itself (the node graph, the [`Engine`], the [`Cmd`] vocabulary) lives
//! in `deluge_wren_core` so the device and the web simulator share one signal
//! engine. This module is the firmware-specific plumbing around it:
//!
//! ## Concurrency
//! `vm_task` (via the `Node` bindings → [`FwHost::audio_cmd`](crate::host::FwHost))
//! only enqueues [`Cmd`]s onto [`CMD_RING`]; `audio_task` drains them between
//! render blocks. Node ids are handed out by the core's allocator so a foreign
//! ctor can return an id immediately without touching the graph.
//!
//! `ENGINE` is eager-initialized once by [`init_engine`], called from `main`
//! before any task is spawned — so no task can ever observe it uninitialized.
//! Every subsequent access is a freshly-derived, *scoped* `assume_init_mut`
//! borrow: `audio_task`'s per-block closure re-derives `&mut Eng` inside the
//! closure body (never held across the outer `.await`), and a future
//! `upload_table` will do the same. This is sound because: (1) there is exactly
//! one cooperative embassy executor with no preemption; (2) both accessors are
//! synchronous (no `.await` inside either), so they run to completion without
//! interleaving; (3) neither accessor ever holds a `&mut Eng` across a `.await`,
//! so two overlapping borrows can never coexist; (4) `ENGINE` is written exactly
//! once, before either accessor can run, so `assume_init_mut` never observes
//! uninitialized memory; (5) no ISR touches `ENGINE`.

use core::cell::RefCell;
use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;

use deluge::Audio;
use deluge_audio_graph::Engine;
use deluge_wren_core::Cmd;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};

// Sizes satisfy the binding contract: NODES >= WREN_MAX_NODES(64), BUSES >= 8.
type Eng = Engine<32, 64, 128, 8, 90112, 2048>; // BLOCK, NODES, OUTS, BUSES, PCAP, PCHUNK
const _: () = assert!(64 >= deluge_wren_core::WREN_MAX_NODES && 8 >= deluge_wren_core::WREN_MAX_BUSES);
const SAMPLE_RATE: f32 = 44_100.0;

// SAFETY: ENGINE is written exactly once by `init_engine` (from `main`, before any
// task is spawned/polled) and thereafter only *scoped* `assume_init_mut` borrows are
// taken — inside `audio_task`'s synchronous per-block closure and inside
// `upload_table`, which run on the one cooperative executor and never interleave, and
// never hold a borrow across an `.await`. No ISR touches ENGINE. See `## Concurrency`.
static mut ENGINE: MaybeUninit<Eng> = MaybeUninit::uninit();

/// Initialize the audio engine. MUST be called once from `main` before spawning
/// `audio_task`/`vm_task`, so no task ever observes an uninitialized `ENGINE`.
pub fn init_engine() {
    // SAFETY: called once from `main` before any task runs; no other accessor yet.
    unsafe {
        (*addr_of_mut!(ENGINE)).write(Eng::new(SAMPLE_RATE));
    }
}

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
    audio
        .process(|block: &mut [deluge::StereoFrame]| {
            // SAFETY: ENGINE was initialized by `init_engine` in `main` before this
            // task could be polled. This closure is synchronous (no `.await` inside),
            // so the borrow never crosses a yield and never overlaps `upload_table`'s
            // (cooperative executor). See `## Concurrency`.
            let eng: &mut Eng = unsafe { (*addr_of_mut!(ENGINE)).assume_init_mut() };
            // Apply all pending control-rate commands.
            while let Some(c) = CMD_RING.lock(|r| r.borrow_mut().pop()) {
                eng.apply(c);
            }
            // Render the SDK block in engine-BLOCK-sized chunks. The SDK's
            // `StereoFrame` is a distinct (host-vs-device) type from the
            // engine's, so render into a local scratch buffer and copy.
            // Chunk size must equal the engine's BLOCK (32) so `render` fills
            // each chunk fully.
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
        })
        .await
}
