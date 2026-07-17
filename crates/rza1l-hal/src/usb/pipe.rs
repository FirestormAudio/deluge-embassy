//! RUSB1 pipe layer — per-pipe state, allocation, and transfer primitives.
//!
//! This module manages the 16 hardware pipes (DCP + pipes 1-15):
//!
//! - [`PipeConfig`]: describes how a pipe is configured (endpoint, type, MPS,
//!   packet buffer allocation).
//! - [`PipeState`]: runtime transfer state (active buffer pointer, remaining
//!   bytes, waker).
//! - [`PIPE_WAKERS`]: one [`AtomicWaker`] per pipe; the ISR wakes the
//!   appropriate waker when a transfer completes (BRDY/BEMP).
//! - [`BufAllocator`]: bump allocator over the 128 × 64-byte packet buffer.
//!
//! All mutable access to the per-pipe state in IRQ + task must go through
//! critical sections (`critical_section::with`).

use core::sync::atomic::AtomicU16;
use embassy_sync::waitqueue::AtomicWaker;

use super::fifo::{
    fifo_bclr, fifo_bval, fifo_dtln, fifo_for_pipe, fifo_is_ready, fifo_select_pipe,
    fifo_select_recv, hw_to_sw_fifo, sw_to_hw_fifo,
};
use super::regs::{
    FIFOSEL_CURPIPE_MASK, FIFOSEL_DREQE, PIPEBUF_BUFSIZE_MASK, PIPEBUF_BUFSIZE_SHIFT, PIPECFG_CNTMD,
    PIPECFG_DBLB,
    PIPECFG_DIR, PIPECFG_EPNUM_MASK, PIPECFG_SHTNAK, PIPECFG_TYPE_BULK, PIPECFG_TYPE_INTR,
    PIPECFG_TYPE_ISO, PIPECTR_ACLRM, PIPECTR_PBUSY, PIPECTR_PID_BUF, PIPECTR_PID_MASK,
    PIPECTR_PID_NAK, PIPECTR_PID_STALL10, PIPECTR_PID_STALL11, PIPECTR_SQCLR, PIPEMAXP_MXPS_MASK,
    PIPEPERI_IFIS,
    PKT_BUF_BLOCKS, Rusb1Regs, pipectr_ptr, rd, wr,
};

// ---------------------------------------------------------------------------
// Pipe count
// ---------------------------------------------------------------------------

/// Total number of hardware pipes (DCP + pipes 1-15).
pub const PIPE_COUNT: usize = 16;

// ---------------------------------------------------------------------------
// Per-pipe async wakers
// ---------------------------------------------------------------------------

/// One waker per pipe.  The ISR calls `wake()` when a transfer finishes.
///
/// These are `'static` because ISR context has no lifetime.
pub static PIPE_WAKERS: [AtomicWaker; PIPE_COUNT] = {
    #[allow(clippy::declare_interior_mutable_const)]
    const W: AtomicWaker = AtomicWaker::new();
    [W; PIPE_COUNT]
};

// ---------------------------------------------------------------------------
// Transfer-done / error flags set by the ISR
// ---------------------------------------------------------------------------

/// Bitmask: bit N = pipe N transfer has completed (BRDY/BEMP fired).
pub static PIPE_DONE: AtomicU16 = AtomicU16::new(0);

/// Bitmask: bit N = pipe N encountered a NRDY error.
pub static PIPE_NRDY: AtomicU16 = AtomicU16::new(0);

/// Bitmask: bit N = pipe N is an IN (device→host) pipe.  Set by
/// [`pipe_configure`].  The BEMP ISR uses this to distinguish non-zero IN pipes
/// from OUT pipes because BEMP is also reported for receive-side overflow/error
/// conditions.
pub static PIPE_IS_IN: AtomicU16 = AtomicU16::new(0);

// ---------------------------------------------------------------------------
// ISO OUT packet hook
// ---------------------------------------------------------------------------

/// Pipe number for which the ISO OUT hook is registered.  [`usize::MAX`] means
/// no hook is registered.
static mut ISO_OUT_HOOK_PIPE: usize = usize::MAX;

/// Optional callback invoked by the BRDY ISR immediately after an ISO OUT packet
/// is fully received, before the task waker is signalled.
///
/// `buf` points to the start of the received bytes; `len` is the byte count.
///
/// # Safety
/// Called from IRQ context on single-core ARMv7-A (IRQs already disabled).
static mut ISO_OUT_HOOK: Option<unsafe fn(*const u8, usize)> = None;

/// Scratch buffer used to drain an ISO OUT packet from the hardware FIFO when no
/// task-side `read()` is pending, before handing it to the registered hook.
/// Sized for the largest ISO packet this module configures (288 B), rounded up
/// to a block boundary.  Touched only by the BRDY ISR, which cannot reenter on
/// single-core ARMv7-A.
const ISO_DRAIN_LEN: usize = 512;
static mut ISO_DRAIN_BUF: [u8; ISO_DRAIN_LEN] = [0; ISO_DRAIN_LEN];

/// Register an ISO OUT packet callback for `pipe`.
///
/// The callback is invoked from the BRDY ISR with a pointer to the fully
/// received packet and its byte count, before the task waker is signalled.
/// This lets audio conversion run with zero scheduling latency.
///
/// # Safety
/// Must be called before the pipe is armed and before BRDY interrupts are
/// enabled for it.  Not interrupt-safe — call from task context only.
pub unsafe fn register_iso_out_hook(pipe: usize, cb: unsafe fn(*const u8, usize)) {
    unsafe {
        core::ptr::addr_of_mut!(ISO_OUT_HOOK_PIPE).write(pipe);
        core::ptr::addr_of_mut!(ISO_OUT_HOOK).write(Some(cb));
    }
}

// ---------------------------------------------------------------------------
// ISO IN packet hook (symmetric to the OUT hook — continuous BRDY-driven TX)
// ---------------------------------------------------------------------------

/// Pipe number for which the ISO IN hook is registered.  [`usize::MAX`] = none.
static mut ISO_IN_HOOK_PIPE: usize = usize::MAX;
/// Max bytes to request from the hook per packet (the pipe's max packet size).
static mut ISO_IN_HOOK_MPS: usize = 0;
/// Callback that FILLS the next ISO IN packet.  `buf` is a scratch buffer of at
/// least `max` bytes; the hook writes up to `max` bytes and returns the packet
/// length (may be 0 for an empty microframe).
///
/// # Safety
/// Called from IRQ context on single-core ARMv7-A (IRQs already disabled), and
/// once at enable time from task context.
static mut ISO_IN_HOOK: Option<unsafe fn(*mut u8, usize) -> usize> = None;
/// Scratch buffer the hook fills before it is copied into the hardware FIFO.
static mut ISO_IN_BUF: [u8; ISO_DRAIN_LEN] = [0; ISO_DRAIN_LEN];

/// Register an ISO IN packet callback for `pipe` carrying `mps`-byte packets.
///
/// Unlike the blocking `EndpointIn::write()` path, this drives the ISO IN pipe
/// in continuous BUF mode: the pipe stays armed and the BRDY ISR calls the hook
/// to stage each packet, matching the proven tinyusb model (and symmetric with
/// [`register_iso_out_hook`]).  Enable BRDY for the pipe (done by
/// `endpoint_set_enabled` for the hook pipe) after registering.
///
/// # Safety
/// Call from task context before the pipe is enabled.  Not interrupt-safe.
pub unsafe fn register_iso_in_hook(pipe: usize, mps: usize, cb: unsafe fn(*mut u8, usize) -> usize) {
    unsafe {
        core::ptr::addr_of_mut!(ISO_IN_HOOK_PIPE).write(pipe);
        core::ptr::addr_of_mut!(ISO_IN_HOOK_MPS).write(mps);
        core::ptr::addr_of_mut!(ISO_IN_HOOK).write(Some(cb));
    }
}

/// True if `pipe` is the registered ISO IN hook pipe.
#[inline]
pub unsafe fn is_iso_in_hook_pipe(pipe: usize) -> bool {
    unsafe { pipe == core::ptr::addr_of!(ISO_IN_HOOK_PIPE).read() }
}

/// Activate an ISO IN pipe for continuous BRDY-driven streaming.
///
/// Sequence: reset FIFO+toggle (leaves PID=NAK), prime both double-buffer
/// planes, clear stale BRDY, enable BRDY, then release to PID=BUF.  (IFIS —
/// isochronous IN buffer flush — is already set for the pipe by
/// [`pipe_configure`].)
///
/// # Safety
/// `regs` valid; task context (no transfer in flight on this pipe).
pub unsafe fn pipe_iso_in_activate(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        // pipe_reset quiesces (PID=NAK, deselect) and pulses ACLRM|SQCLR.
        pipe_reset(regs, n);
        // Prime BOTH double-buffer planes while PID is still NAK.  TRM §28.4
        // "BRDY interrupt (a), transmitting direction" condition 4: BRDY fires
        // when one plane empties AND the OTHER plane has been fully written — so
        // with only one plane primed, transmitting it leaves the other plane
        // empty and BRDY never fires, stalling the stream.  Filling both planes
        // bootstraps the double-buffer cadence (each subsequent BRDY refills the
        // plane the SIE just freed).  Also stages data so the first IN tokens
        // transmit real audio.
        pipe_xfer_in_brdy(regs, n); // plane A
        pipe_xfer_in_brdy(regs, n); // plane B
        // Clear any stale BRDY, arm BRDY *before* releasing to BUF.
        wr(core::ptr::addr_of_mut!((*regs).brdysts), !(1u16 << n));
        pipe_brdy_enable(regs, n);
        pipe_enable(regs, n); // PID=BUF
    }
}

/// Stage the next ISO IN packet for the hook pipe: ask the hook to fill a
/// packet, copy it into the hardware FIFO, and commit it, leaving the pipe
/// armed (PID stays BUF).  Called from the BRDY ISR (a buffer freed up) and once
/// at enable time to prime the first buffer.  No-op if no hook is registered or
/// the FIFO port is not ready.
///
/// # Safety
/// `regs` must be valid; single-core IRQ or task context (never both at once —
/// the enable-time prime runs with BRDY disabled).
pub unsafe fn pipe_xfer_in_brdy(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let Some(hook) = core::ptr::addr_of!(ISO_IN_HOOK).read() else {
            return;
        };
        let mps = core::ptr::addr_of!(ISO_IN_HOOK_MPS).read().min(ISO_DRAIN_LEN);
        let fifo = fifo_for_pipe(regs, n);
        fifo_select_pipe(&fifo, n, true); // ISEL=1 for IN (ignored on DnFIFO)
        if !fifo_is_ready(&fifo, n) {
            return;
        }
        let scratch = core::ptr::addr_of_mut!(ISO_IN_BUF) as *mut u8;
        let len = hook(scratch, mps).min(mps);
        if len > 0 {
            sw_to_hw_fifo(&fifo, scratch as *const u8, len);
        }
        // ISO packets are always < mps (a fraction of a full buffer), so commit
        // every packet with BVAL — including a 0-length one (empty microframe).
        // See fill_in_packet: a fill short of the buffer plane needs BVAL to send.
        if len < mps {
            fifo_bval(&fifo);
        }
    }
}

// ---------------------------------------------------------------------------
// Active transfer state (written by task, read by ISR)
// ---------------------------------------------------------------------------

/// Runtime state of one pipe's in-progress transfer.
///
/// Written by the async `write`/`read` futures before they suspend;
/// read and mutated by the ISR during BRDY/BEMP handling.
///
/// Guarded by `critical_section` whenever ISR and task must both access it.
pub struct PipeXferState {
    /// Current position in the user buffer.  `NonNull::dangling()` when idle.
    pub buf: core::ptr::NonNull<u8>,
    /// Total transfer length in bytes.
    pub length: u16,
    /// Bytes remaining (counts down as packets are transferred).
    pub remaining: u16,
    /// Bytes actually transferred so far.  Used to report the final byte count
    /// to the task, because for ISO OUT the `done` path forces `remaining = 0`
    /// early (to allow the ISO guard to fire on re-entry) before `remaining`
    /// naturally reaches zero.
    pub transferred: u16,
    /// Max packet size (cached from PIPEMAXP to avoid register reads in ISR).
    pub mps: u16,
    /// Maximum bytes to commit to the FIFO per fill.  Equals `mps` for ordinary
    /// pipes; for a continuous-mode (CNTMD) bulk IN pipe it is the whole
    /// multi-packet buffer, so one fill stages several packets the SIE then
    /// streams back-to-back.  Must be a multiple of `mps`.
    pub buf_bytes: u16,
    /// Transfer type — needed by the BRDY ISR to distinguish ISO vs bulk behaviour.
    pub xfer_type: XferType,
}

// Safety: single-core bare metal — no actual data races.
// Access must still be protected with critical_section to prevent ISR
// interrupting a multi-step mutation in task context.
unsafe impl Send for PipeXferState {}
unsafe impl Sync for PipeXferState {}

impl PipeXferState {
    const IDLE: Self = Self {
        buf: core::ptr::NonNull::dangling(),
        length: 0,
        remaining: 0,
        transferred: 0,
        mps: 0,
        buf_bytes: 0,
        xfer_type: XferType::Bulk,
    };
}

/// Per-port array of transfer states.  Index 0 = DCP (pipe 0).
pub static PIPE_XFER: [critical_section::Mutex<core::cell::UnsafeCell<PipeXferState>>; PIPE_COUNT] = {
    use core::cell::UnsafeCell;
    use critical_section::Mutex;
    #[allow(clippy::declare_interior_mutable_const)]
    const IDLE: Mutex<UnsafeCell<PipeXferState>> = Mutex::new(UnsafeCell::new(PipeXferState::IDLE));
    [IDLE; PIPE_COUNT]
};

// ---------------------------------------------------------------------------
// Pipe configuration (stored after allocate, used by configure)
// ---------------------------------------------------------------------------

/// Transfer type tag for a pipe.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum XferType {
    Control,
    Bulk,
    Interrupt,
    Isochronous,
}

impl XferType {
    /// PIPECFG TYPE field value (0b00 = control for DCP, not programmed here).
    pub fn pipecfg_type_bits(self) -> u16 {
        match self {
            XferType::Control => 0,
            XferType::Bulk => PIPECFG_TYPE_BULK,
            XferType::Interrupt => PIPECFG_TYPE_INTR,
            XferType::Isochronous => PIPECFG_TYPE_ISO,
        }
    }
}

/// Static configuration for one pipe.
#[derive(Clone, Copy)]
pub struct PipeConfig {
    /// Endpoint number (1-15; 0 = DCP is special).
    pub ep_num: u8,
    /// True if IN (device → host).
    pub is_in: bool,
    /// Transfer type.
    pub xfer_type: XferType,
    /// Max packet size in bytes.
    pub mps: u16,
    /// Start block index in the 8 KB packet buffer (64-byte granularity).
    pub buf_start: u8,
    /// Number of 64-byte blocks allocated for this pipe.  For a continuous-mode
    /// pipe this is the whole multi-packet buffer (not a per-bank size).
    pub buf_blocks: u8,
    /// Enable double-buffering (for bulk OUT and ISO pipes).
    pub double_buf: bool,
    /// Enable continuous transfer mode (CNTMD) — a single buffer larger than
    /// the max packet size over which the SIE streams multiple packets.  Only
    /// valid for bulk pipes 1-5; mutually exclusive with `double_buf`.
    pub continuous: bool,
}

// ---------------------------------------------------------------------------
// Packet buffer allocator
// ---------------------------------------------------------------------------

/// Simple bump allocator over the 128 × 64-byte packet buffer.
///
/// Uses a 128-bit bitmask: bit N = block N is allocated.
///
/// Blocks 0-6 are permanently reserved (TRM §28.3.33 BUFNMB / Table 28.19):
/// - Blocks 0-3: fixed DCP (pipe 0) buffer (256 bytes).
/// - Blocks 4, 5, 6: hardwired to PIPE6, PIPE7, PIPE8 respectively.  BUFNMB
///   writes for those pipes are *ignored* by the hardware, so if these blocks
///   were handed to another pipe while an interrupt pipe is in use the two
///   buffers would silently overlap.  Other pipes' valid BUFNMB range starts
///   at H'07.
pub struct BufAllocator {
    /// Bitmask: 1 = allocated.
    bits: u128,
}

impl BufAllocator {
    /// Create a new allocator with blocks 0-6 reserved (DCP + PIPE6-8 fixed areas).
    pub const fn new() -> Self {
        Self { bits: 0b111_1111 }
    }

    /// Allocate `n_blocks` contiguous blocks.  Returns the start block index,
    /// or `None` if the packet buffer is full.
    pub fn alloc(&mut self, n_blocks: usize) -> Option<u8> {
        // Linear scan for first run of `n_blocks` free bits.
        'outer: for start in 0..=(PKT_BUF_BLOCKS - n_blocks) {
            for i in 0..n_blocks {
                if (self.bits >> (start + i)) & 1 != 0 {
                    continue 'outer;
                }
            }
            // Found a free run — mark as allocated.
            for i in 0..n_blocks {
                self.bits |= 1u128 << (start + i);
            }
            return Some(start as u8);
        }
        None
    }

    /// Number of currently-free 64-byte blocks in the packet buffer.
    pub fn free_blocks(&self) -> usize {
        PKT_BUF_BLOCKS - (self.bits.count_ones() as usize)
    }

    /// Free `n_blocks` blocks starting at `start`.
    pub fn free(&mut self, start: u8, n_blocks: usize) {
        for i in 0..n_blocks {
            self.bits &= !(1u128 << (start as usize + i));
        }
    }
}

impl Default for BufAllocator {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Pipe configuration (hardware write)
// ---------------------------------------------------------------------------

/// Write PIPESEL, PIPECFG, PIPEBUF, PIPEMAXP for pipe `n` (1-15).
///
/// Pipe 0 (DCP) is configured separately via DCPCFG / DCPMAXP.
///
/// # Safety
/// `regs` must be a valid pointer to the USB register block with USBE=1.
/// Caller must ensure no transfer is active on this pipe.
pub unsafe fn pipe_configure(regs: *mut Rusb1Regs, n: usize, cfg: &PipeConfig) {
    unsafe {
        debug_assert!((1..=15).contains(&n), "pipe_configure: invalid pipe number");

        // Deactivate the pipe before reconfiguring.  TRM §28.3.32: PIPECFG /
        // PIPEBUF / PIPEMAXP / PIPEPERI may only be written while PID is NAK
        // *and* the pipe is not selected by any FIFO port's CURPIPE bits,
        // with PBUSY confirmed 0 after a software BUF→NAK transition.
        pipe_quiesce(regs, n);

        // Select the pipe and write configuration registers.
        wr(core::ptr::addr_of_mut!((*regs).pipesel), n as u16);

        // Record direction so the BRDY ISR can dispatch IN vs OUT (see PIPE_IS_IN).
        if cfg.is_in {
            PIPE_IS_IN.fetch_or(1u16 << n, core::sync::atomic::Ordering::Release);
        } else {
            PIPE_IS_IN.fetch_and(!(1u16 << n), core::sync::atomic::Ordering::Release);
        }

        let mut pipecfg = cfg.ep_num as u16 & PIPECFG_EPNUM_MASK;
        if cfg.is_in {
            pipecfg |= PIPECFG_DIR;
        }
        pipecfg |= cfg.xfer_type.pipecfg_type_bits();

        match cfg.xfer_type {
            XferType::Bulk => {
                if !cfg.is_in {
                    // SHTNAK: NAK on short packet (OUT), prevents spurious BRDYs.
                    pipecfg |= PIPECFG_SHTNAK;
                    // Double-buffer bulk OUT is explicitly disabled to avoid the
                    // BRDY race described in dcd_rusb1.c comments.
                } else if cfg.continuous {
                    // Continuous transfer mode: one buffer > MaxPacketSize over
                    // which the SIE streams several packets per micro-frame.
                    pipecfg |= PIPECFG_CNTMD;
                } else if cfg.double_buf {
                    pipecfg |= PIPECFG_DBLB;
                }
            }
            XferType::Isochronous => {
                pipecfg |= PIPECFG_DBLB; // ISO always double-buffered
            }
            _ => {}
        }

        wr(core::ptr::addr_of_mut!((*regs).pipecfg), pipecfg);

        // PIPEBUF: BUFNMB (start block) | BUFSIZE ((blocks - 1) in 64-byte units).
        // BUFSIZE is a 5-bit field [14:10]; mask after the shift so an oversized
        // buf_blocks can never spill into the reserved bit 15 (matches the C
        // reference `(0x1f & bufnmb_cnt) << 10`).
        let bufsize_field = (((cfg.buf_blocks as u16).saturating_sub(1)) << PIPEBUF_BUFSIZE_SHIFT)
            & PIPEBUF_BUFSIZE_MASK;
        let pipebuf = cfg.buf_start as u16 | bufsize_field;
        wr(core::ptr::addr_of_mut!((*regs).pipebuf), pipebuf);

        // Log the actual packet-RAM footprint so overlaps are visible: a
        // double-buffered pipe occupies TWO banks of `buf_blocks` each, i.e.
        // blocks [buf_start, buf_start + banks*buf_blocks).  Two pipes whose
        // ranges intersect will wedge on the hardware.
        let banks: u16 = if cfg.double_buf || cfg.xfer_type == XferType::Isochronous {
            2
        } else {
            1
        };
        let blocks = cfg.buf_blocks as u16 * banks;
        log::debug!(
            "pipe {} ep={:#04x} {} {}: buf blocks [{}, {}) ({} blk x{}buf, mps={})",
            n,
            cfg.ep_num as u16 | if cfg.is_in { 0x80 } else { 0 },
            if cfg.is_in { "IN " } else { "OUT" },
            match cfg.xfer_type {
                XferType::Isochronous => "ISO",
                XferType::Bulk => "BULK",
                XferType::Interrupt => "INT",
                XferType::Control => "CTRL",
            },
            cfg.buf_start,
            cfg.buf_start as u16 + blocks,
            cfg.buf_blocks,
            banks,
            cfg.mps,
        );

        // PIPEMAXP: max packet size.
        wr(
            core::ptr::addr_of_mut!((*regs).pipemaxp),
            cfg.mps & PIPEMAXP_MXPS_MASK,
        );

        // PIPEPERI: set IFIS for ISO IN pipes (function controller mode).
        // IFIS=1 causes the hardware to flush the TX buffer if no IN token arrives
        // within the interval frame, preventing stale audio data from accumulating
        // (TRM §28.4.9(5)).
        let pipeperi = if cfg.xfer_type == XferType::Isochronous && cfg.is_in {
            PIPEPERI_IFIS
        } else {
            0
        };
        wr(core::ptr::addr_of_mut!((*regs).pipeperi), pipeperi);

        // Deselect pipe.
        wr(core::ptr::addr_of_mut!((*regs).pipesel), 0);
    }
}

// ---------------------------------------------------------------------------
// Transfer initiation helpers (called from driver / ISR)
// ---------------------------------------------------------------------------

/// Arm the IN direction of pipe `n` for a transfer (write first packet
/// to FIFO, set PID=BUF).
///
/// Returns `true` if the entire transfer fit in one packet (done immediately).
///
/// # Safety
/// `regs` must be valid.
pub unsafe fn pipe_xfer_in_start(regs: *mut Rusb1Regs, n: usize, mps: u16) -> bool {
    unsafe {
        critical_section::with(|cs| {
            let state = &mut *PIPE_XFER[n].borrow(cs).get();
            if state.remaining == 0 {
                state.buf = core::ptr::NonNull::dangling();
                if state.length == 0 {
                    // Explicit zero-length-packet transmit (write(&[])).  TRM
                    // §28.4.5: "to send a zero-length packet, the BCLR bit must
                    // be used to clear the buffer and then the BVAL bit is set
                    // to end the writing."  Without this commit nothing is ever
                    // staged, the SIE NAKs every IN token, no BEMP fires, and
                    // write(&[]) hangs forever.
                    let fifo = fifo_for_pipe(regs, n);
                    fifo_select_pipe(&fifo, n, true);
                    if fifo_is_ready(&fifo, n) {
                        fifo_bclr(&fifo);
                        fifo_bval(&fifo);
                        return false; // ZLP staged — BEMP will signal completion
                    }
                }
                return true;
            }
            if !fill_in_packet(regs, n, mps as usize, state) {
                return false; // FIFO port not ready — nothing written
            }
            state.remaining == 0
        })
    }
}

/// Write up to one `mps`-sized packet from pipe `n`'s active descriptor into its
/// FIFO.  Returns `true` if a packet (including a short final one) was committed,
/// `false` if the FIFO port was not ready (nothing written).
///
/// Shared by [`pipe_xfer_in_start`] (task context, first packet) and
/// [`pipe_xfer_in_bemp`] (ISR context, continuation packets) so the two paths
/// fill identically.  The caller owns the `state` borrow and interprets
/// `state.remaining` afterward.
///
/// # Safety
/// `regs` must be valid.  The caller must already hold the `PIPE_XFER[n]`
/// critical section (task context) or be running in ISR context.
unsafe fn fill_in_packet(
    regs: *mut Rusb1Regs,
    n: usize,
    mps: usize,
    state: &mut PipeXferState,
) -> bool {
    unsafe {
        let fifo = fifo_for_pipe(regs, n);
        fifo_select_pipe(&fifo, n, true); // ISEL=1 for IN

        if !fifo_is_ready(&fifo, n) {
            return false;
        }

        // Commit up to one whole buffer.  For an ordinary pipe `buf_bytes == mps`
        // so this stages a single packet; for a continuous-mode (CNTMD) pipe it
        // stages several packets at once, which the SIE then streams back-to-back.
        let cap = (state.buf_bytes as usize).max(mps);
        let len = (state.remaining as usize).min(cap);
        sw_to_hw_fifo(&fifo, state.buf.as_ptr(), len);

        // Transmit trigger (RZ/A1 TRM Table 28.11, DIR=1):
        //   (1) a fill that reaches the full buffer-plane size auto-transmits;
        //   (2) any *partial* fill must be committed by writing BVAL.
        // So BVAL whenever we did not exactly fill the buffer.  For an ordinary
        // pipe `cap == mps`, this reduces to "BVAL on a short packet" — the
        // original behaviour.  (Filling a whole number of packets that is still
        // less than the buffer needs BVAL too, or it would never be sent.)
        if len < cap {
            fifo_bval(&fifo);
        }

        state.buf = core::ptr::NonNull::new_unchecked(state.buf.as_ptr().add(len));
        state.remaining = state.remaining.saturating_sub(len as u16);
        true
    }
}

/// Reads available data from an OUT pipe's FIFO into the active transfer
/// buffer.  Returns `true` when the transfer is complete (short packet or
/// remaining == 0).
///
/// For non-ISO pipes the hardware PID is set to NAK before the FIFO is read
/// (preventing the SIE from writing new data while CPU reads), then re-armed
/// to BUF if the transfer is not yet done — matching the C `process_pipe_brdy`
/// / `pipe_xfer_out` reference implementation.
///
/// `speculative` distinguishes the two callers:
/// - `false` — invoked from the BRDY ISR, where the SIE has set BRDYSTS, so a
///   packet is *guaranteed* present.  A zero-length read is then a genuine ZLP
///   (the terminator for an exact-multiple-of-mps bulk OUT transfer) and
///   completes the read by returning `true`.
/// - `true` — invoked eagerly from `read()` to drain a packet that may have
///   arrived while BRDY was disabled.  Here only the FIFO `FRDY` flag is
///   checked, which can be asserted with `DTLN == 0` (no real packet).  A
///   zero-byte non-ISO read in this mode is *not* treated as complete: the
///   function re-arms and returns `false` so `read()` waits for a real BRDY,
///   instead of returning `Ok(0)` and spinning a caller that loops on `read()`
///   (e.g. the MSC BOT loop awaiting a 31-byte CBW).
///
/// # Safety
/// `regs` must be valid.  When called from ISR context no critical section is
/// needed (single-core Cortex-A9 with IRQ disabled); the speculative call site
/// wraps it in one.
pub unsafe fn pipe_xfer_out_brdy(regs: *mut Rusb1Regs, n: usize, speculative: bool) -> bool {
    unsafe {
        let state = &mut *{
            // Access state without critical_section — we're already in IRQ context
            // where the task cannot run (single-core).
            PIPE_XFER[n]
                .borrow(critical_section::CriticalSection::new())
                .get()
        };

        let is_iso = state.xfer_type == XferType::Isochronous;

        if state.remaining == 0 {
            // No task-side read() is pending for this pipe.
            //
            // If an ISO OUT hook is registered for this pipe, drain the packet
            // from the FIFO into the scratch buffer and hand it to the hook here,
            // in ISR context.  This is the audio fast path: it decouples ISO OUT
            // servicing from executor scheduling latency entirely.  Without it, a
            // packet that arrives while no read() is armed sits in the
            // double-buffered FIFO; if the CPU does not drain it within ~2
            // microframes the SIE has nowhere to put the next packet and drops it
            // (TRM §28.4.9 / Table 28.26: OVRN set, NRDY raised, "no data packet
            // is received in response to OUT token") — an audible gap.
            //
            // A hook is only ever installed on the ISO OUT pipe, so matching the
            // pipe number is sufficient; `state.xfer_type` is NOT reliable here
            // because it is only set by `read()`, the very dependency this path
            // exists to avoid.
            if n == core::ptr::addr_of!(ISO_OUT_HOOK_PIPE).read()
                && let Some(hook) = core::ptr::addr_of!(ISO_OUT_HOOK).read()
            {
                let fifo = fifo_for_pipe(regs, n);
                fifo_select_pipe(&fifo, n, false);
                if fifo_is_ready(&fifo, n) {
                    let vld = fifo_dtln(&fifo) as usize;
                    let scratch = core::ptr::addr_of_mut!(ISO_DRAIN_BUF) as *mut u8;
                    let len = vld.min(ISO_DRAIN_LEN);
                    if len > 0 {
                        hw_to_sw_fifo(&fifo, scratch, len);
                    }
                    // Always BCLR to release the double-buffer bank so the SIE can
                    // accept the next packet.
                    fifo_bclr(&fifo);
                    if len > 0 {
                        hook(scratch as *const u8, len);
                    }
                }
                return false;
            }

            // No hook (bulk/interrupt-style OUT, or ISO before the hook is
            // installed): do NOT BCLR — leave the packet in the FIFO so the
            // task's read() re-arm path can drain it without missing it.
            return false;
        }

        let ctr_ptr = pipectr_ptr(regs, n);

        // For non-ISO: NAK first to prevent the SIE from filling the FIFO while
        // the CPU is reading it (matches C `process_pipe_brdy` NAK-before-read).
        if !is_iso {
            let cur = rd(ctr_ptr);
            wr(
                ctr_ptr,
                (cur & !super::regs::PIPECTR_PID_MASK) | PIPECTR_PID_NAK,
            );
        }

        let fifo = fifo_for_pipe(regs, n);
        // Receiving (OUT) pipe selection.  For bulk pipes on the shared D1FIFO
        // (e.g. CDC OUT pipe 3 sharing the port with IN pipe 4) use the TRM
        // deselect-first CURPIPE-change procedure so this read cannot surface
        // the previously-selected IN pipe's staged data (the IN/OUT
        // cross-corruption).  ISO pipes live on their own D0FIFO and are
        // timing-critical for audio, so keep their original single select.
        if is_iso {
            fifo_select_pipe(&fifo, n, false);
        } else {
            fifo_select_recv(&fifo, n);
        }

        if !fifo_is_ready(&fifo, n) {
            // FIFO port not ready; re-arm and retry next BRDY.
            if !is_iso {
                let cur = rd(ctr_ptr);
                wr(
                    ctr_ptr,
                    (cur & !super::regs::PIPECTR_PID_MASK) | PIPECTR_PID_BUF,
                );
            }
            return false;
        }

        let vld = fifo_dtln(&fifo) as usize;
        let mps = state.mps as usize;
        let rem = state.remaining as usize;
        let len = rem.min(mps).min(vld);
        if len > 0 {
            hw_to_sw_fifo(&fifo, state.buf.as_ptr(), len);
            state.buf = core::ptr::NonNull::new_unchecked(state.buf.as_ptr().add(len));
            state.remaining = state.remaining.saturating_sub(len as u16);
            state.transferred += len as u16;
        }

        // Always BCLR after reading (matches C reference — not just on short packet).
        fifo_bclr(&fifo);

        // A zero-byte non-ISO read from a speculative call is not a real packet
        // (FRDY without DTLN), so don't let it complete the transfer — re-arm and
        // wait for a genuine BRDY.  A real ZLP only reaches here via the ISR
        // (speculative == false), where len < mps correctly terminates the read.
        let real_packet = len > 0 || is_iso || !speculative;
        let done = real_packet && (len < mps || state.remaining == 0);
        if done {
            // Invoke the ISO OUT hook (if registered for this pipe) before
            // resetting state.  `state.buf` has advanced by `transferred` bytes
            // since the transfer started; rewind to recover the packet start.
            if n == core::ptr::addr_of!(ISO_OUT_HOOK_PIPE).read()
                && let Some(hook) = core::ptr::addr_of!(ISO_OUT_HOOK).read()
            {
                let start = state.buf.as_ptr().sub(state.transferred as usize);
                hook(start, state.transferred as usize);
            }
            state.buf = core::ptr::NonNull::dangling();
            // Reset remaining to 0 so the ISO guard at the top of this function
            // fires correctly if BRDY re-enters before the task calls read() again.
            // (For ISO, done can be true via len<mps while remaining is still non-zero.)
            state.remaining = 0;
            return true;
        }

        // Transfer not yet complete; re-arm the pipe so the host can send the
        // next packet (non-ISO only — ISO stays at BUF/auto-managed).
        if !is_iso {
            let cur = rd(ctr_ptr);
            wr(
                ctr_ptr,
                (cur & !super::regs::PIPECTR_PID_MASK) | PIPECTR_PID_BUF,
            );
        }

        false
    }
}

/// Called from the IN completion ISR path.  Writes the next packet to FIFO,
/// or signals completion if the transfer is done.
///
/// Returns `true` when the transfer is fully complete.
///
/// # Safety
/// `regs` must be valid.  Called from ISR context.
pub unsafe fn pipe_xfer_in_bemp(regs: *mut Rusb1Regs, n: usize) -> bool {
    unsafe {
        let state = &mut *{
            PIPE_XFER[n]
                .borrow(critical_section::CriticalSection::new())
                .get()
        };

        if state.remaining == 0 {
            state.buf = core::ptr::NonNull::dangling();
            return true;
        }

        let mps = state.mps as usize;
        // Fill the next packet (shared with the task-context first-fill path).
        // A `false` return means the FIFO port wasn't ready; we report "not yet
        // done" and the next BEMP will retry, matching the prior behaviour.
        fill_in_packet(regs, n, mps, state);
        false
    }
}

// ---------------------------------------------------------------------------
// Pipe enable / disable
// ---------------------------------------------------------------------------

/// Move a pipe out of a STALL condition back to NAK, stepping through the
/// intermediate STALL(10) if the controller autonomously drove it to STALL(11).
///
/// A direct STALL(11)→NAK write is ignored by the SIE, so a hardware-set STALL
/// (babble / control-sequence error) would otherwise never clear — leaving a
/// `CLEAR_FEATURE(ENDPOINT_HALT)` ineffective and the endpoint wedged.  Mirrors
/// Linux `__usbhsp_pid_try_nak_if_stall`.  No-op for NAK/BUF.
///
/// # Safety
/// `ctr` must be a valid PIPEnCTR/DCPCTR pointer.
unsafe fn pid_nak_if_stall(ctr: *mut u16) {
    unsafe {
        let pid = rd(ctr) & PIPECTR_PID_MASK;
        if pid == PIPECTR_PID_STALL11 {
            // 11 → 10 first; the SIE ignores a direct 11 → NAK transition.
            wr(ctr, (rd(ctr) & !PIPECTR_PID_MASK) | PIPECTR_PID_STALL10);
        }
        if pid == PIPECTR_PID_STALL11 || pid == PIPECTR_PID_STALL10 {
            wr(ctr, (rd(ctr) & !PIPECTR_PID_MASK) | PIPECTR_PID_NAK);
        }
    }
}

/// Set PID=BUF to allow data transfer on pipe `n`.
///
/// # Safety
/// `regs` must be valid.
pub unsafe fn pipe_enable(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let ctr = pipectr_ptr(regs, n);
        // Clear any residual STALL first (matches Linux usbhs_pipe_enable).
        pid_nak_if_stall(ctr);
        let cur = rd(ctr);
        wr(
            ctr,
            (cur & !super::regs::PIPECTR_PID_MASK) | PIPECTR_PID_BUF,
        );
    }
}

/// Set PID=NAK to halt data transfer on pipe `n`.
///
/// # Safety
/// `regs` must be valid.
pub unsafe fn pipe_disable(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let ctr = pipectr_ptr(regs, n);
        // Step down through STALL(10) so a hardware STALL(11) actually clears.
        pid_nak_if_stall(ctr);
        let cur = rd(ctr);
        wr(
            ctr,
            (cur & !super::regs::PIPECTR_PID_MASK) | PIPECTR_PID_NAK,
        );
    }
}

/// Set PID=STALL on pipe `n`, choosing the encoding from the current PID as the
/// hardware requires: NAK→STALL(10), BUF→STALL(11).  Leaves an existing STALL
/// unchanged.  Matches Linux `usbhs_pipe_stall`.
///
/// # Safety
/// `regs` must be valid.
pub unsafe fn pipe_stall(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let ctr = pipectr_ptr(regs, n);
        let stall = match rd(ctr) & PIPECTR_PID_MASK {
            PIPECTR_PID_NAK => PIPECTR_PID_STALL10,
            PIPECTR_PID_BUF => PIPECTR_PID_STALL11,
            _ => return, // already stalled — leave as-is
        };
        wr(ctr, (rd(ctr) & !PIPECTR_PID_MASK) | stall);
    }
}

/// Clear the data toggle and FIFO on pipe `n`, then set PID=NAK.
///
/// # Safety
/// `regs` must be valid.
pub unsafe fn pipe_reset(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        // ACLRM and SQCLR require PID=NAK with the pipe idle (PBUSY=0).
        pipe_quiesce(regs, n);
        let ctr = pipectr_ptr(regs, n);
        // Set ACLRM (auto clear FIFO and toggle) then clear it, per TRM.
        wr(ctr, PIPECTR_PID_NAK | PIPECTR_ACLRM);
        wr(ctr, PIPECTR_PID_NAK);
        wr(ctr, PIPECTR_PID_NAK | PIPECTR_SQCLR);
    }
}

/// Halt pipe `n` and make it safe to reconfigure (TRM §28.3.32 precondition).
///
/// Sets PID=NAK, waits for PBUSY to clear (required after a software
/// BUF→NAK transition before touching pipe configuration), and deselects
/// the pipe from any FIFO port whose CURPIPE bits point at it (clearing
/// DREQE first on the DnFIFO ports, as required before a CURPIPE change).
///
/// # Safety
/// `regs` must be valid.
unsafe fn pipe_quiesce(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let ctr = pipectr_ptr(regs, n);
        let entry_pid = rd(ctr) & PIPECTR_PID_MASK;
        // Step a hardware STALL(11) down to NAK before touching config (a direct
        // STALL→NAK write is ignored); then force NAK for the BUF case.
        pid_nak_if_stall(ctr);
        let cur = rd(ctr);
        if cur & PIPECTR_PID_MASK != PIPECTR_PID_NAK {
            wr(ctr, (cur & !PIPECTR_PID_MASK) | PIPECTR_PID_NAK);
        }
        if entry_pid != PIPECTR_PID_NAK {
            // PBUSY clears once the in-flight transaction (at most one
            // max-size packet) finishes; bound the wait so a wedged bus
            // cannot hang the driver.
            for _ in 0..100_000 {
                if rd(ctr) & PIPECTR_PBUSY == 0 {
                    break;
                }
            }
        }

        // CFIFOSEL has no DREQE bit (masking it there is harmless);
        // DnFIFOSEL requires DREQE=0 before CURPIPE may change.
        let sels = [
            core::ptr::addr_of_mut!((*regs).cfifosel),
            core::ptr::addr_of_mut!((*regs).d0fifosel),
            core::ptr::addr_of_mut!((*regs).d1fifosel),
        ];
        for sel in sels {
            let v = rd(sel);
            if (v & FIFOSEL_CURPIPE_MASK) as usize != n {
                continue;
            }
            if v & FIFOSEL_DREQE != 0 {
                wr(sel, v & !FIFOSEL_DREQE);
            }
            wr(sel, rd(sel) & !FIFOSEL_CURPIPE_MASK);
            // Read back until the CURPIPE change latches (TRM procedure).
            for _ in 0..1_000 {
                if rd(sel) & FIFOSEL_CURPIPE_MASK == 0 {
                    break;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// BRDY / BEMP interrupt enable helpers
// ---------------------------------------------------------------------------

/// Enable BRDY interrupt for pipe `n`.
///
/// # Safety
/// `regs` must be valid.
pub unsafe fn pipe_brdy_enable(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let brdysts = core::ptr::addr_of_mut!((*regs).brdysts);
        let brdyenb = core::ptr::addr_of_mut!((*regs).brdyenb);
        // Clear any pending BRDY for this pipe before enabling.
        wr(brdysts, !((1u16) << n));
        let cur = rd(brdyenb);
        wr(brdyenb, cur | (1u16 << n));
    }
}

/// Disable BRDY interrupt for pipe `n`.
///
/// # Safety
/// `regs` must be valid.
pub unsafe fn pipe_brdy_disable(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let brdyenb = core::ptr::addr_of_mut!((*regs).brdyenb);
        let cur = rd(brdyenb);
        wr(brdyenb, cur & !(1u16 << n));
    }
}

/// Enable BEMP interrupt for pipe `n`.
///
/// # Safety
/// `regs` must be valid.
pub unsafe fn pipe_bemp_enable(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let bempsts = core::ptr::addr_of_mut!((*regs).bempsts);
        let bempenb = core::ptr::addr_of_mut!((*regs).bempenb);
        wr(bempsts, !((1u16) << n));
        let cur = rd(bempenb);
        wr(bempenb, cur | (1u16 << n));
    }
}

/// Disable BEMP interrupt for pipe `n`.
///
/// # Safety
/// `regs` must be valid.
pub unsafe fn pipe_bemp_disable(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let bempenb = core::ptr::addr_of_mut!((*regs).bempenb);
        let cur = rd(bempenb);
        wr(bempenb, cur & !(1u16 << n));
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn buf_allocator_basic() {
        let mut alloc = BufAllocator::new();
        // Blocks 0-3 (DCP) and 4-6 (PIPE6-8 fixed areas) are pre-allocated.
        let a = alloc.alloc(4).unwrap();
        assert_eq!(a, 7); // first free block is 7
        let b = alloc.alloc(4).unwrap();
        assert_eq!(b, 11);
        alloc.free(a, 4);
        let c = alloc.alloc(2).unwrap();
        assert_eq!(c, 7); // reuse freed blocks
    }

    #[test]
    fn buf_allocator_full() {
        let mut alloc = BufAllocator::new();
        // Fill everything after the reserved area (blocks 7-127 = 121 blocks).
        let start = alloc.alloc(121).unwrap();
        assert_eq!(start, 7);
        // Now no room for even 1 more block.
        assert!(alloc.alloc(1).is_none());
    }

    #[test]
    fn xfer_type_pipecfg_bits() {
        assert_eq!(XferType::Bulk.pipecfg_type_bits(), PIPECFG_TYPE_BULK);
        assert_eq!(XferType::Interrupt.pipecfg_type_bits(), PIPECFG_TYPE_INTR);
        assert_eq!(XferType::Isochronous.pipecfg_type_bits(), PIPECFG_TYPE_ISO);
        assert_eq!(XferType::Control.pipecfg_type_bits(), 0);
    }
}
