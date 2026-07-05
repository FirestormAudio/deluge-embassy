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

use deluge::{Audio, StereoFrame};
use deluge_wren_core::{Cmd, Engine};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};

// SAFETY: ENGINE is touched only by `audio_task` (single accessor).
static mut ENGINE: Engine = Engine::new();

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
/// drain the control-rate command queue into the engine, then render one mono
/// sample per frame (duplicated to L+R). Output is `[-1.0, 1.0]`; the SDK handles
/// codec scaling and the SSI TX/RX DMA cadence (poll loop, or the per-block RX
/// interrupt under the `audio-irq` feature).
#[embassy_executor::task]
pub async fn audio_task(audio: Audio) {
    // SAFETY: this task is the sole accessor of ENGINE.
    let eng = unsafe { &mut *core::ptr::addr_of_mut!(ENGINE) };
    audio
        .process(|block: &mut [StereoFrame]| {
            // Apply all pending control-rate commands.
            while let Some(c) = CMD_RING.lock(|r| r.borrow_mut().pop()) {
                eng.apply(c);
            }
            for f in block {
                let s = eng.render_frame().clamp(-1.0, 1.0);
                f.l = s;
                f.r = s;
            }
        })
        .await
}
