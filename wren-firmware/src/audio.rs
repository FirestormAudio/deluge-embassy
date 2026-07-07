//! Firmware audio transport: the control→audio command ring, the engine
//! instance, and the render task.
//!
//! The DSP itself (the node graph, the [`Engine`], the [`Cmd`] vocabulary) lives
//! in `deluge_wren_core` so the device and the web simulator share one signal
//! engine. This module is the firmware-specific plumbing around it:
//!
//! ## Concurrency
//! The [`Engine`] is owned solely by [`audio_task`]. `vm_task` (via the `Node`
//! bindings → [`FwHost::audio_cmd`](crate::host::FwHost)) only enqueues [`Cmd`]s
//! onto [`CMD_RING`]; `audio_task` drains them between render blocks. Node ids are
//! handed out by the core's allocator so a foreign ctor can return an id
//! immediately without touching the graph.

use core::cell::RefCell;
use core::mem::MaybeUninit;

use deluge::Audio;
use deluge_audio_graph::Engine;
use deluge_wren_core::Cmd;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};

// Sizes satisfy the binding contract: NODES >= WREN_MAX_NODES(64), BUSES >= 8.
type Eng = Engine<32, 64, 128, 8>; // BLOCK, NODES, OUTS, BUSES
const _: () = assert!(64 >= deluge_wren_core::WREN_MAX_NODES && 8 >= deluge_wren_core::WREN_MAX_BUSES);
const SAMPLE_RATE: f32 = 44_100.0;

// SAFETY: ENGINE is initialized once at the top of audio_task and thereafter
// touched only by that task (single accessor).
static mut ENGINE: MaybeUninit<Eng> = MaybeUninit::uninit();

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
    // SAFETY: audio_task is the sole accessor; init happens once before use.
    let eng: &mut Eng = unsafe {
        let p = &mut *core::ptr::addr_of_mut!(ENGINE);
        p.write(Eng::new(SAMPLE_RATE));
        p.assume_init_mut()
    };
    audio
        .process(|block: &mut [deluge::StereoFrame]| {
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
