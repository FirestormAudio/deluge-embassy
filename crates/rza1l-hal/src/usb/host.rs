//! RUSB1 USB host-mode driver.
//!
//! ## Architecture
//!
//! ```text
//! application
//!   │  Rusb1HostDriver (implements embassy_usb_driver::host::UsbHostDriver)
//!   │    ├─ wait_for_device_event() ← Connected(Speed) / Disconnected
//!   │    ├─ bus_reset()             ← async, drives USBRST for ~20 ms
//!   │    └─ alloc_channel()         ← returns Rusb1Channel<T,D>
//!   │
//!   │  Rusb1Channel<T, D> (implements UsbChannel<T, D>)
//!   │    ├─ control_in / control_out  ← setup + data + status ZLP
//!   │    ├─ request_in / request_out  ← non-control IN / OUT
//!   │    └─ retarget_channel()        ← after SET_ADDRESS
//!   │
//!   └─ hcd_int_handler(port)   ← GIC IRQ 73/74
//!         ├─ INTSTS1: ATTCH/DTCH → HCD_EVENTS + waker
//!         │           SACK/SIGN  → HCD_EVENTS + waker
//!         ├─ INTSTS0 BRDY → pipe0_in_brdy / pipe_brdy_in / pipe_brdy_out
//!         │         BEMP  → pipe0_out_bemp (control OUT continuation)
//!         └─         NRDY → HCD_PIPE_NRDY / HCD_PIPE_STALL + wakers
//! ```
//!
//! ## Pipe allocation (host mode)
//!
//! The reference `hcd_rusb1.c` driver uses only pipes 0-9.
//!
//! | Pipe | Purpose       |
//! |------|---------------|
//! | 0    | Control (DCP) |
//! | 1-2  | ISO or Bulk   |
//! | 3-5  | Bulk          |
//! | 6-9  | Interrupt     |
//!
//! ## FIFO routing (host mode)
//!
//! - Pipe 0 (control): CFIFO
//! - Pipes 1-9 (all others): **D0FIFO** (unlike device mode, host never uses D1FIFO)
//!
//! ## PIPECFG.DIR in host mode
//!
//! DIR is inverted relative to device mode: DIR=1 means the host is sending
//! (USB OUT), DIR=0 means the host is receiving (USB IN).  The C reference
//! computes this as `(1 ^ dir_in) << 4`.
//!
//! ## Reference
//!
//! `lib/tinyusb/src/portable/renesas/rusb1/hcd_rusb1.c`

use core::sync::atomic::{AtomicU8, AtomicU16, Ordering};
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Timer;
use embassy_usb_driver::host::{
    DeviceEvent, HostError, PipeError, SplitInfo, SplitSpeed, TimeoutConfig, UsbHostAllocator,
    UsbHostController, UsbPipe, pipe,
};
use embassy_usb_driver::{EndpointInfo, EndpointType, Speed};

use super::fifo::{FifoPort, fifo_bval, sw_to_hw_fifo};
use super::pipe::BufAllocator;
use super::regs::{
    BUSWAIT_VALUE, DCPCFG_DIR, DCPCFG_SHTNAK, DCPCTR_SUREQ, DCPCTR_SUREQCLR, DCPMAXP_DEVSEL_SHIFT,
    DCPMAXP_MXPS_MASK, DEVADD_HUBPORT_SHIFT, DEVADD_UPPHUB_SHIFT, DEVADD_USBSPD_FS,
    DEVADD_USBSPD_HS, DEVADD_USBSPD_LS, DVSTCTR0_RHST, DVSTCTR0_RHST_FS, DVSTCTR0_RHST_HS,
    DVSTCTR0_RHST_LS, DVSTCTR0_UACT, DVSTCTR0_USBRST, FIFOCTR_BCLR, FIFOCTR_BVAL,
    FIFOCTR_DTLN_MASK, FIFOCTR_FRDY, FIFOSEL_CURPIPE_MASK, FIFOSEL_ISEL, FIFOSEL_MBW_SHIFT,
    INTENB0_BEMPE, INTENB0_BRDYE, INTENB0_NRDYE, INTENB1_ATTCHE, INTENB1_DTCHE, INTENB1_SACKE,
    INTENB1_SIGNE, INTSTS0_BEMP, INTSTS0_BRDY, INTSTS0_NRDY, INTSTS1_ATTCH, INTSTS1_DTCH,
    INTSTS1_SACK, INTSTS1_SIGN, MBW_32, PIPEBUF_BUFNMB_MASK, PIPEBUF_BUFSIZE_MASK,
    PIPEBUF_BUFSIZE_SHIFT, PIPECFG_DBLB, PIPECFG_EPNUM_MASK, PIPECFG_SHTNAK, PIPECFG_TYPE_BULK,
    PIPECFG_TYPE_INTR, PIPECFG_TYPE_ISO, PIPECTR_ACLRM, PIPECTR_BSTS, PIPECTR_PBUSY,
    PIPECTR_PID_BUF, PIPECTR_PID_MASK, PIPECTR_PID_NAK, PIPECTR_SQCLR, PIPECTR_SQSET,
    PIPEMAXP_DEVSEL_SHIFT, PIPEMAXP_MXPS_MASK, PIPEPERI_IITV_MASK, PKT_BUF_BLOCK_SIZE, Rusb1Regs,
    SUSPMODE_SUSPM, SYSCFG_DCFM, SYSCFG_DPRPU, SYSCFG_DRPD, SYSCFG_HSE, SYSCFG_UPLLE, SYSCFG_USBE,
    SYSSTS0_LNST, devadd_ptr, pipectr_ptr, rd, rmw, wr,
};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Number of hardware pipes used in host mode (DCP=0, pipes 1-9).
const HCD_PIPE_COUNT: usize = 10;

/// Maximum device addresses supported (DEVADD0-5 → indices 0-5).
const HCD_MAX_DEV: usize = 6;

/// True iff `dev_addr` is a device address this driver can target.
///
/// Address 0 is the enumeration default address. The upper bound is
/// [`HCD_MAX_DEV`], which sizes `HcdAlloc::ctl_mps` and `HcdAlloc::ep_to_pipe` —
/// exceeding it would index out of bounds.
pub(crate) const fn dev_addr_supported(dev_addr: u8) -> bool {
    dev_addr > 0 && (dev_addr as usize) < HCD_MAX_DEV
}

// HCD_EVENTS bit layout (matches C: process_attach / process_detach).
const EVT_ATTACH: u8 = 1 << 0;
const EVT_DETACH: u8 = 1 << 1;
const EVT_SACK: u8 = 1 << 2; // setup ACK from device
const EVT_SIGN: u8 = 1 << 3; // setup ignored (NAK/error)

// FIFO SEL/CTR field layout is shared by CFIFO and DnFIFO and lives in
// `regs.rs` (CURPIPE [3:0], FRDY bit 13, DTLN [11:0] — TRM §28.3.8/28.3.9).
// Earlier revisions redefined FRDY as 0x0020 and DTLN as 0x01FF here, which
// polled a DTLN bit instead of FRDY and truncated a 512-byte HS bulk packet's
// length to 0.

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------
// (Re-exported from embassy_usb_driver — no custom type duplication needed.)

// ---------------------------------------------------------------------------
// Per-pipe ISR ↔ task state
// ---------------------------------------------------------------------------

struct HcdPipeXfer {
    /// User buffer pointer; null when no transfer is active.
    buf: *mut u8,
    /// Total requested byte count.
    length: u16,
    /// Bytes still to transfer.
    remaining: u16,
    /// Endpoint address (bit 7 = IN/OUT direction, bits 3:0 = EP number).
    ep_addr: u8,
    /// USB device address of the pipe's target.
    dev_addr: u8,
}

unsafe impl Send for HcdPipeXfer {}
unsafe impl Sync for HcdPipeXfer {}

impl HcdPipeXfer {
    const IDLE: Self = Self {
        buf: core::ptr::null_mut(),
        length: 0,
        remaining: 0,
        ep_addr: 0,
        dev_addr: 0,
    };
}

static HCD_XFER: [critical_section::Mutex<core::cell::UnsafeCell<HcdPipeXfer>>; HCD_PIPE_COUNT] = {
    use core::cell::UnsafeCell;
    use critical_section::Mutex;
    #[allow(clippy::declare_interior_mutable_const)]
    const IDLE: Mutex<UnsafeCell<HcdPipeXfer>> = Mutex::new(UnsafeCell::new(HcdPipeXfer::IDLE));
    [IDLE; HCD_PIPE_COUNT]
};

/// Bit N set → pipe N transfer completed.
static HCD_PIPE_DONE: AtomicU16 = AtomicU16::new(0);
/// Bit N set → pipe N transfer failed (NRDY/NAK).
static HCD_PIPE_NRDY: AtomicU16 = AtomicU16::new(0);
/// Bit N set → pipe N endpoint STALLed.
static HCD_PIPE_STALL: AtomicU16 = AtomicU16::new(0);

static HCD_PIPE_WAKERS: [AtomicWaker; HCD_PIPE_COUNT] = {
    #[allow(clippy::declare_interior_mutable_const)]
    const W: AtomicWaker = AtomicWaker::new();
    [W; HCD_PIPE_COUNT]
};

// ---------------------------------------------------------------------------
// Bus-level event state (ATTCH / DTCH / SACK / SIGN)
// ---------------------------------------------------------------------------

static HCD_EVENTS: AtomicU8 = AtomicU8::new(0);
static HCD_EVENT_WAKER: AtomicWaker = AtomicWaker::new();

// ---------------------------------------------------------------------------
// Pipe-allocation + EP→pipe mapping, per port
// ---------------------------------------------------------------------------

struct HcdAlloc {
    /// True between DeviceAttached and the next bus_reset().
    need_reset: bool,
    /// Bitmap of in-use pipes: bit N set → pipe N allocated.
    pipes_used: u16,
    /// ep_to_pipe[dev_addr-1][dir_in as usize][ep_num - 1] → pipe number.
    /// 0 = not open.  dev_addr must be 1-5.
    ep_to_pipe: [[[u8; 15]; 2]; 5],
    /// EP0 max-packet-size per device address.
    ctl_mps: [u16; HCD_MAX_DEV],
    /// Packet-buffer block allocator (128 × 64-byte blocks, blocks 0-6 reserved
    /// for DCP + the hardware-fixed PIPE6-8 areas).
    buf: BufAllocator,
    /// Per-pipe packet-buffer reservation, for freeing on close.
    /// `buf_start[n]` = first 64-byte block, `buf_reserved[n]` = block count.
    buf_start: [u8; HCD_PIPE_COUNT],
    buf_reserved: [u8; HCD_PIPE_COUNT],
}

impl HcdAlloc {
    const fn new() -> Self {
        Self {
            need_reset: false,
            pipes_used: 0x0001, // pipe 0 is permanently allocated (DCP)
            ep_to_pipe: [[[0u8; 15]; 2]; 5],
            ctl_mps: [64u16; HCD_MAX_DEV],
            buf: BufAllocator::new(),
            buf_start: [0u8; HCD_PIPE_COUNT],
            buf_reserved: [0u8; HCD_PIPE_COUNT],
        }
    }

    /// Allocate a free pipe for `ep_type`.  Searches backward (highest pipe
    /// first) within the type's range, same as the C `find_pipe` function.
    fn alloc_pipe(&mut self, ep_type: EndpointType) -> Option<usize> {
        let (first, last): (usize, usize) = match ep_type {
            EndpointType::Isochronous => (1, 2),
            EndpointType::Bulk => (1, 5),
            EndpointType::Interrupt => (6, 9),
            EndpointType::Control => return None, // control uses pipe 0 directly
        };
        for p in (first..=last).rev() {
            if self.pipes_used & (1 << p) == 0 {
                self.pipes_used |= 1 << p;
                return Some(p);
            }
        }
        None
    }

    fn free_pipe(&mut self, pipe: usize) {
        if pipe != 0 {
            self.pipes_used &= !(1u16 << pipe);
            // Release the pipe's packet-buffer blocks back to the pool.
            if self.buf_reserved[pipe] > 0 {
                self.buf
                    .free(self.buf_start[pipe], self.buf_reserved[pipe] as usize);
                self.buf_start[pipe] = 0;
                self.buf_reserved[pipe] = 0;
            }
        }
    }

    /// Reserve packet-buffer blocks for `pipe` carrying `mps`-byte packets.
    ///
    /// Returns `(buf_start, blocks_per_bank)` for the PIPEBUF write, or `None`
    /// if the packet buffer is full.  `double_buf` reserves two banks (matching
    /// PIPECFG.DBLB).  Pipes 6-8 have hardware-fixed buffers (blocks 4/5/6) and
    /// must NOT be passed here — their PIPEBUF is ignored by the controller.
    fn alloc_pipe_buf(&mut self, pipe: usize, mps: u16, double_buf: bool) -> Option<(u8, u8)> {
        let blocks_per_bank = (mps as usize).div_ceil(PKT_BUF_BLOCK_SIZE).max(1);
        let reserve = blocks_per_bank * if double_buf { 2 } else { 1 };
        let start = self.buf.alloc(reserve)?;
        self.buf_start[pipe] = start;
        self.buf_reserved[pipe] = reserve as u8;
        Some((start, blocks_per_bank as u8))
    }
}

static HCD_ALLOC: [critical_section::Mutex<core::cell::UnsafeCell<HcdAlloc>>; 2] = {
    use core::cell::UnsafeCell;
    use critical_section::Mutex;
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT: Mutex<UnsafeCell<HcdAlloc>> = Mutex::new(UnsafeCell::new(HcdAlloc::new()));
    [INIT; 2]
};

// ---------------------------------------------------------------------------
// Rusb1HostDriver
// ---------------------------------------------------------------------------

/// USB host-mode driver for one RUSB1 port.
///
/// Implements [`embassy_usb_driver::host::UsbHostDriver`] so it can be used
/// directly with the `embassy-usb-host` enumeration stack.
///
/// The raw async methods (`wait_for_event`, `setup_send`, `xfer_in`, …) are
/// also kept for direct use without the Embassy host stack.
pub struct Rusb1HostDriver {
    port: u8,
}

impl Rusb1HostDriver {
    /// Initialise the hardware in host mode and return a driver instance.
    ///
    /// # Safety
    ///
    /// - The RUSB1 module clock must be running.
    /// - No device-mode driver may be active on this port simultaneously.
    /// - `hcd_int_handler(port)` must be wired to GIC IRQ 73+port before
    ///   calling any async method.
    pub unsafe fn new(port: u8) -> Self {
        unsafe {
            debug_assert!(port <= 1);
            let regs = Rusb1Regs::ptr(port);

            // ── PLL / clock init (TRM §28.4.1 (5)) ─────────────────────────
            // Stop clock supply to this port first.
            rmw(core::ptr::addr_of_mut!((*regs).suspmode), SUSPMODE_SUSPM, 0);

            // UPLLE lives in USB0's SYSCFG0 (C reference always writes
            // rusb0->SYSCFG0) and may only be modified while SUSPM=0 on BOTH
            // channels (TRM §28.3.1).  Only enable it when off: a set UPLLE means
            // the PLL is already locked from the other port's bring-up, so we
            // must not repeat the lock wait or disturb that port's SUSPM.  When
            // off, the PLL is down and neither channel can be running, so
            // clearing the other channel's SUSPM is safe.
            let regs0 = Rusb1Regs::ptr(0);
            if rd(core::ptr::addr_of!((*regs0).syscfg0)) & SYSCFG_UPLLE == 0 {
                let other = Rusb1Regs::ptr(if port == 0 { 1 } else { 0 });
                rmw(
                    core::ptr::addr_of_mut!((*other).suspmode),
                    SUSPMODE_SUSPM,
                    0,
                );
                rmw(
                    core::ptr::addr_of_mut!((*regs0).syscfg0),
                    SYSCFG_UPLLE,
                    SYSCFG_UPLLE,
                );

                // Wait >= 1 ms for PLL lock (~400 K iterations at 400 MHz).
                for _ in 0u32..400_000 {
                    core::hint::spin_loop();
                }
            }

            // CPU-bus wait cycles and clock re-enable.
            wr(core::ptr::addr_of_mut!((*regs).buswait), BUSWAIT_VALUE);
            rmw(
                core::ptr::addr_of_mut!((*regs).suspmode),
                SUSPMODE_SUSPM,
                SUSPMODE_SUSPM,
            );

            // ── Mode / PHY config ────────────────────────────────────────────
            // Host mode: DCFM=1, DRPD=1 (pull-downs enabled), DPRPU=0 (no pull-up).
            // HSE is NOT set here; TRM requires it to be set after ATTCH and before
            // USBRST — it is written in bus_reset() based on the detected device speed.
            //
            // TRM §28.3.1: DCFM may only be modified while DPRPU and DRPD are
            // both 0.  DRPD's reset value is 1, so the first write must clear it
            // (along with DPRPU/HSE) *while* setting DCFM — otherwise the cold-boot
            // DCFM 0→1 transition violates the precondition.  The second write
            // then raises DRPD (host pull-downs).
            rmw(
                core::ptr::addr_of_mut!((*regs).syscfg0),
                SYSCFG_DCFM | SYSCFG_DRPD | SYSCFG_DPRPU | SYSCFG_HSE,
                SYSCFG_DCFM,
            );
            rmw(
                core::ptr::addr_of_mut!((*regs).syscfg0),
                SYSCFG_DRPD,
                SYSCFG_DRPD,
            );

            // NOTE: VBUS power is NOT module-controlled on the RZ/A1L —
            // DVSTCTR0 bits 11:9 are reserved (there is no VBUSEN bit, TRM
            // §28.3.4).  Host-port VBUS must be switched externally (GPIO).

            // TRM §28.3.1 USBE: in host mode, set USBE only "after setting DRPD
            // to 1, eliminating LNST bit chattering, and checking that the USB
            // bus has been settled."  Give the line state a brief settle window
            // after raising DRPD before enabling the SIE.
            for _ in 0u32..10_000 {
                core::hint::spin_loop();
            }

            // Enable the USB module.
            rmw(
                core::ptr::addr_of_mut!((*regs).syscfg0),
                SYSCFG_USBE,
                SYSCFG_USBE,
            );

            // Control pipe: always use SHTNAK (NAK on short packet) in host mode.
            wr(core::ptr::addr_of_mut!((*regs).dcpcfg), DCPCFG_SHTNAK);
            wr(core::ptr::addr_of_mut!((*regs).dcpmaxp), 64);

            // Global interrupt enables.
            wr(
                core::ptr::addr_of_mut!((*regs).intenb0),
                INTENB0_BRDYE | INTENB0_NRDYE | INTENB0_BEMPE,
            );
            // Start watching for device attach (ATTCH); DTCH is enabled by the ISR
            // after a device is connected.
            wr(
                core::ptr::addr_of_mut!((*regs).intenb1),
                INTENB1_SACKE | INTENB1_SIGNE | INTENB1_ATTCHE,
            );

            // Enable BEMP/NRDY/BRDY for pipe 0 (the DCP).
            wr(core::ptr::addr_of_mut!((*regs).bempenb), 1);
            wr(core::ptr::addr_of_mut!((*regs).nrdyenb), 1);
            wr(core::ptr::addr_of_mut!((*regs).brdyenb), 1);

            // Register the GIC interrupt for this port.
            crate::usb::int_enable(port);

            Self { port }
        }
    }

    // --- Bus-level API ---

    /// Stop the host controller and leave the hardware in a neutral state,
    /// ready for the device-mode driver to take over.
    ///
    /// TRM §28.3 (3): the DCFM bit may only be changed while DPRPU = 0 *and*
    /// DRPD = 0.  Host mode always has DPRPU = 0, so we clear DRPD first,
    /// then clear DCFM to return to function (device) mode.
    pub async fn stop(&mut self) {
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let e = &mut *regs;

            // Disable GIC interrupt so the ISR cannot fire mid-teardown.
            crate::usb::int_disable(self.port);

            // Disable all interrupt sources.
            wr(core::ptr::addr_of_mut!(e.intenb0), 0);
            wr(core::ptr::addr_of_mut!(e.intenb1), 0);
            wr(core::ptr::addr_of_mut!(e.brdyenb), 0);
            wr(core::ptr::addr_of_mut!(e.nrdyenb), 0);
            wr(core::ptr::addr_of_mut!(e.bempenb), 0);

            // Stop SOF generation.  (VBUS is switched externally on the
            // RZ/A1L — DVSTCTR0 has no VBUSEN bit.)
            rmw(core::ptr::addr_of_mut!(e.dvstctr0), DVSTCTR0_UACT, 0);
        }
        // Let the bus settle before changing resistor config.
        Timer::after_micros(1).await;
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Clear DRPD to satisfy the TRM DCFM-change pre-condition
            // (DPRPU = 0 already in host mode; now DRPD = 0 too).
            rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_DRPD, 0);
            // Switch to function (device) mode.
            rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_DCFM, 0);
        }
    }

    /// Wait for the next raw bus event: device attached or detached.
    ///
    /// For `Connected`, speed is not yet valid — call [`bus_reset`](Self::bus_reset)
    /// first, then read [`port_speed`](Self::port_speed).  Consider using
    /// `UsbHostDriver::wait_for_device_event` which handles this automatically.
    pub async fn wait_for_event(&self) -> DeviceEvent {
        core::future::poll_fn(|cx| {
            HCD_EVENT_WAKER.register(cx.waker());
            // Consume the event atomically.
            let ev = HCD_EVENTS.fetch_and(!(EVT_ATTACH | EVT_DETACH), Ordering::Acquire);
            if ev & EVT_ATTACH != 0 {
                return core::task::Poll::Ready(DeviceEvent::Connected(Speed::Full));
            }
            if ev & EVT_DETACH != 0 {
                return core::task::Poll::Ready(DeviceEvent::Disconnected);
            }
            core::task::Poll::Pending
        })
        .await
    }

    /// Drive a USB bus reset (~20 ms), then re-enable SOF generation.
    ///
    /// Must be called after every [`DeviceEvent::Connected`] before
    /// enumeration.  Drives USBRST for ~20 ms using the async timer.
    pub async fn bus_reset(&self) {
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);

            // Cancel any in-flight DCP transaction.
            wr(pipectr_ptr(regs, 0), PIPECTR_PID_NAK);
            while rd(pipectr_ptr(regs, 0)) & PIPECTR_PBUSY != 0 {}

            // Stop SOF and cancel any pending SETUP.
            crate::usb::int_disable(self.port);
            rmw(core::ptr::addr_of_mut!((*regs).dvstctr0), DVSTCTR0_UACT, 0);
            let dcpctr = rd(pipectr_ptr(regs, 0));
            if dcpctr & DCPCTR_SUREQ != 0 {
                wr(pipectr_ptr(regs, 0), dcpctr | DCPCTR_SUREQCLR);
            }
            crate::usb::int_enable(self.port);

            // Set HSE before asserting USBRST (TRM §28.3 SYSCFG.HSE: "after
            // detecting the ATTCH interrupt and before setting USBRST to 1").
            // LS devices cannot do HS negotiation — HSE must be 0 for them.
            //
            // LS detection: RHST only reports 001 (LS) after a *previous* bus
            // reset, so on the first reset after attach it still reads 000.
            // In that case use the idle line state instead (TRM Table 28.5
            // with DRPD=1): an attached FS/HS device pulls D+ → LNST = 01
            // (full-speed J); an LS device pulls D- → LNST = 10.
            let rhst_now = rd(core::ptr::addr_of!((*regs).dvstctr0)) & DVSTCTR0_RHST;
            // Debounce LNST: the TRM requires reading the line state repeatedly
            // until the same value is seen several times running (SYSSTS0
            // chattering).  A single mid-chatter read could flip the HSE choice,
            // running an LS device through the HS handshake (or vice-versa).
            let mut lnst = rd(core::ptr::addr_of!((*regs).syssts0)) & SYSSTS0_LNST;
            let mut stable = 0u8;
            for _ in 0..64 {
                let s = rd(core::ptr::addr_of!((*regs).syssts0)) & SYSSTS0_LNST;
                if s == lnst {
                    stable += 1;
                    if stable >= 3 {
                        break;
                    }
                } else {
                    lnst = s;
                    stable = 0;
                }
            }
            let is_ls = rhst_now == DVSTCTR0_RHST_LS || (rhst_now == 0 && lnst == 0b10);
            if is_ls {
                rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_HSE, 0);
            } else {
                rmw(
                    core::ptr::addr_of_mut!((*regs).syscfg0),
                    SYSCFG_HSE,
                    SYSCFG_HSE,
                );
            }
            rmw(
                core::ptr::addr_of_mut!((*regs).dvstctr0),
                DVSTCTR0_USBRST,
                DVSTCTR0_USBRST,
            );
        }

        // Wait for reset to propagate — async sleep instead of busy-spinning.
        Timer::after_millis(20).await;

        let p = self.port as usize;
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // End the reset and re-enable SOF generation in a single write:
            // TRM §28.3.4 USBRST: "Write 1 to the UACT bit simultaneously with
            // the end of the USB bus reset process (writing 0 to USBRST)."
            let cur = rd(core::ptr::addr_of!((*regs).dvstctr0));
            wr(
                core::ptr::addr_of_mut!((*regs).dvstctr0),
                (cur & !DVSTCTR0_USBRST) | DVSTCTR0_UACT,
            );
        }

        // Wait for the reset handshake to complete before the caller reads the
        // speed.  RHST only latches a determined value (001 LS / 010 FS / 011
        // HS) once SE0 driving finishes; for HS the chirp handshake can take a
        // few ms, during which RHST reads 0b1xx.  A prior version read RHST
        // immediately, so the 0b100 "in progress" value fell through
        // port_speed()'s `_ => Full` fallback and LS/HS devices were mis-detected
        // as Full.  Poll up to ~50 ms (mirrors the reference's bounded retry).
        for _ in 0..50 {
            let rhst = unsafe { rd(core::ptr::addr_of!((*Rusb1Regs::ptr(self.port)).dvstctr0)) }
                & DVSTCTR0_RHST;
            if matches!(rhst, DVSTCTR0_RHST_LS | DVSTCTR0_RHST_FS | DVSTCTR0_RHST_HS) {
                break;
            }
            Timer::after_millis(1).await;
        }

        unsafe {
            critical_section::with(|cs| {
                (&mut *HCD_ALLOC[p].borrow(cs).get()).need_reset = false;
            });
        }
    }

    /// Return the speed of the connected device.
    ///
    /// Only valid after [`bus_reset`](Self::bus_reset) completes.
    pub fn port_speed(&self) -> Speed {
        const RHST_LS: u16 = 0x0001;
        const RHST_FS: u16 = 0x0002;
        const RHST_HS: u16 = 0x0003;
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            match rd(core::ptr::addr_of!((*regs).dvstctr0)) & 0x0007 {
                RHST_LS => Speed::Low,
                RHST_FS => Speed::Full,
                RHST_HS => Speed::High,
                _ => Speed::Full, // safe fallback
            }
        }
    }

    // --- Device / endpoint management ---

    /// Configure DCPMAXP and DEVADD for a device's control endpoint.
    ///
    /// `hub_addr` and `hub_port` describe where in the topology the device
    /// lives:
    /// - Pass `hub_addr = 0, hub_port = 0` when the device is **directly**
    ///   attached to this USB port.
    /// - Pass the hub's USB address and the hub port number (1-based) when
    ///   the device is connected through a hub.  The hardware will
    ///   automatically generate SSPLIT/CSPLIT split transactions.
    ///
    /// Must be called (and DEVADDn fully configured) **before** starting
    /// any pipe that targets this device (TRM Note 1 for DEVADDn).
    ///
    /// Returns [`HostError::OutOfSlots`] if `dev_addr` exceeds what
    /// [`HCD_MAX_DEV`] allows. Address 0 is permitted: enumeration programmes
    /// the DCP at the default address before `SET_ADDRESS`.
    pub fn device_open(
        &self,
        dev_addr: u8,
        mps: u16,
        speed: Speed,
        hub_addr: u8,
        hub_port: u8,
    ) -> Result<(), HostError> {
        if dev_addr != 0 && !dev_addr_supported(dev_addr) {
            return Err(HostError::OutOfSlots);
        }
        // hub_addr 0 means direct; 1-10 are valid hub USB addresses.
        debug_assert!(hub_addr <= 10);
        // hub_port 0 means direct; 1-7 are valid hub port numbers.
        debug_assert!(hub_port <= 7);
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let p = self.port as usize;

            // Ensure the DCP is idle before modifying DEVADDn (TRM Note 2).
            wr(pipectr_ptr(regs, 0), PIPECTR_PID_NAK);
            while rd(pipectr_ptr(regs, 0)) & PIPECTR_PBUSY != 0 {}

            // Program DEVADDn FIRST: UPPHUB | HUBPORT | USBSPD.  TRM §28.3
            // DCPMAXP.DEVSEL: "these bits should be set after setting the DEVADDn
            // register corresponding to the value to be set."  Writing DEVSEL
            // before DEVADDn would latch stale speed/hub fields for this address.
            let usbspd = match speed {
                Speed::Low => DEVADD_USBSPD_LS,
                Speed::High => DEVADD_USBSPD_HS,
                Speed::Full => DEVADD_USBSPD_FS,
            };
            let devadd_val = ((hub_addr as u16) << DEVADD_UPPHUB_SHIFT)
                | ((hub_port as u16) << DEVADD_HUBPORT_SHIFT)
                | usbspd;
            wr(devadd_ptr(regs, dev_addr), devadd_val);

            // Then embed the device address (DEVSEL) in DCPMAXP bits [15:12].
            wr(
                core::ptr::addr_of_mut!((*regs).dcpmaxp),
                ((dev_addr as u16) << DCPMAXP_DEVSEL_SHIFT) | (mps & DCPMAXP_MXPS_MASK),
            );

            critical_section::with(|cs| {
                (&mut *HCD_ALLOC[p].borrow(cs).get()).ctl_mps[dev_addr as usize] = mps;
            });
        }
        Ok(())
    }

    /// Free all pipes that were allocated for `dev_addr`.
    ///
    /// Call after a device disconnects to reclaim hardware resources.
    pub fn device_close(&self, dev_addr: u8) {
        debug_assert!(dev_addr > 0 && (dev_addr as usize) < HCD_MAX_DEV);
        let p = self.port as usize;
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            critical_section::with(|cs| {
                let alloc = &mut *HCD_ALLOC[p].borrow(cs).get();
                alloc.ctl_mps[dev_addr as usize] = 64;

                for dir in 0..2usize {
                    for epn in 0..15usize {
                        let pipe = alloc.ep_to_pipe[dev_addr as usize - 1][dir][epn] as usize;
                        if pipe == 0 {
                            continue;
                        }

                        // Disable the pipe.
                        wr(pipectr_ptr(regs, pipe), 0);
                        let e = &mut (*regs);
                        rmw(core::ptr::addr_of_mut!(e.nrdyenb), 1 << pipe, 0);
                        rmw(core::ptr::addr_of_mut!(e.brdyenb), 1 << pipe, 0);
                        wr(core::ptr::addr_of_mut!(e.pipesel), pipe as u16);
                        wr(core::ptr::addr_of_mut!(e.pipecfg), 0);
                        wr(core::ptr::addr_of_mut!(e.pipemaxp), 0);
                        wr(core::ptr::addr_of_mut!(e.pipesel), 0);

                        // Clear the HCD_XFER entry.
                        let xfer = &mut *HCD_XFER[pipe].borrow(cs).get();
                        xfer.buf = core::ptr::null_mut();
                        xfer.dev_addr = 0;
                        xfer.ep_addr = 0;

                        alloc.free_pipe(pipe);
                        alloc.ep_to_pipe[dev_addr as usize - 1][dir][epn] = 0;
                    }
                }
            });
        }
    }

    // --- Control transfers ---

    /// Send a SETUP packet to `dev_addr`.
    ///
    /// Awaits SACK (returns `Ok(())`) or SIGN (returns
    /// `Err(PipeError::Canceled)`).  `setup` is the 8-byte packet in
    /// USB byte order (bmRequestType, bRequest, wValue, wIndex, wLength).
    pub async fn setup_send(&self, dev_addr: u8, setup: &[u8; 8]) -> Result<(), PipeError> {
        debug_assert!((dev_addr as usize) < HCD_MAX_DEV);
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let p = self.port as usize;

            let ctl_mps = critical_section::with(|cs| {
                (&*HCD_ALLOC[p].borrow(cs).get()).ctl_mps[dev_addr as usize]
            });

            // Ensure the DCP is idle.
            wr(pipectr_ptr(regs, 0), PIPECTR_PID_NAK);
            while rd(pipectr_ptr(regs, 0)) & PIPECTR_PBUSY != 0 {}

            // Update DCPMAXP with target device address.
            wr(
                core::ptr::addr_of_mut!((*regs).dcpmaxp),
                ((dev_addr as u16) << DCPMAXP_DEVSEL_SHIFT) | (ctl_mps & DCPMAXP_MXPS_MASK),
            );

            // Set DCPCFG.DIR: DIR=0 for IN data stage, DIR=1 for OUT data stage.
            // bmRequestType bit 7: 1 = device-to-host (IN).
            // SHTNAK (auto-NAK at end of a receiving transfer) must be 0 in
            // the transmitting direction (TRM §28.3.28), so set it only when
            // the DCP receives (IN data stage).
            let dir_in = setup[0] & 0x80 != 0;
            let dcpcfg = if dir_in { DCPCFG_SHTNAK } else { DCPCFG_DIR };
            wr(core::ptr::addr_of_mut!((*regs).dcpcfg), dcpcfg);

            // Write the 8 setup bytes to hardware registers.
            wr(
                core::ptr::addr_of_mut!((*regs).usbreq),
                u16::from_le_bytes([setup[0], setup[1]]),
            );
            wr(
                core::ptr::addr_of_mut!((*regs).usbval),
                u16::from_le_bytes([setup[2], setup[3]]),
            );
            wr(
                core::ptr::addr_of_mut!((*regs).usbindx),
                u16::from_le_bytes([setup[4], setup[5]]),
            );
            wr(
                core::ptr::addr_of_mut!((*regs).usbleng),
                u16::from_le_bytes([setup[6], setup[7]]),
            );

            // Clear pipe-0 completion flags.
            HCD_PIPE_DONE.fetch_and(!1, Ordering::Release);
            HCD_PIPE_NRDY.fetch_and(!1, Ordering::Release);
            HCD_PIPE_STALL.fetch_and(!1, Ordering::Release);
            // Clear SACK/SIGN event bits.
            HCD_EVENTS.fetch_and(!(EVT_SACK | EVT_SIGN), Ordering::Release);

            // Issue the SETUP token.
            let ctr = rd(pipectr_ptr(regs, 0));
            wr(pipectr_ptr(regs, 0), ctr | DCPCTR_SUREQ);
        }

        // Await SACK or SIGN from the ISR.
        core::future::poll_fn(|cx| {
            HCD_EVENT_WAKER.register(cx.waker());
            let ev = HCD_EVENTS.fetch_and(!(EVT_SACK | EVT_SIGN), Ordering::Acquire);
            if ev & EVT_SACK != 0 {
                return core::task::Poll::Ready(Ok(()));
            }
            if ev & EVT_SIGN != 0 {
                return core::task::Poll::Ready(Err(PipeError::Canceled));
            }
            core::task::Poll::Pending
        })
        .await
    }

    /// Control IN data stage: receive up to `buf.len()` bytes from EP0.
    ///
    /// Returns the number of bytes actually received (may be less than
    /// `buf.len()` if the device sends a short packet).
    pub async fn control_data_in(&self, dev_addr: u8, buf: &mut [u8]) -> Result<usize, PipeError> {
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);

            // Select CFIFO for reading (ISEL=0: host reads from device).
            // Wait for ISEL to deassert before accessing CFIFO.
            wr(core::ptr::addr_of_mut!((*regs).cfifosel), 0x0000); // pipe0, ISEL=0
            for _ in 0..1000u32 {
                if rd(core::ptr::addr_of!((*regs).cfifosel)) & FIFOSEL_ISEL == 0 {
                    break;
                }
            }

            // Set transfer state.
            critical_section::with(|cs| {
                let xfer = &mut *HCD_XFER[0].borrow(cs).get();
                xfer.buf = buf.as_mut_ptr();
                xfer.length = buf.len() as u16;
                xfer.remaining = buf.len() as u16;
                xfer.ep_addr = 0x80; // EP0 IN
                xfer.dev_addr = dev_addr;
            });

            HCD_PIPE_DONE.fetch_and(!1, Ordering::Release);
            HCD_PIPE_NRDY.fetch_and(!1, Ordering::Release);
            HCD_PIPE_STALL.fetch_and(!1, Ordering::Release);

            // Arm DCP: hardware will issue IN tokens and fire BRDY on receipt.
            wr(pipectr_ptr(regs, 0), PIPECTR_PID_BUF);
        }

        poll_pipe(0).await
    }

    /// Control OUT data stage: send `buf` to EP0.
    ///
    /// Pass `&[]` to issue a zero-length packet (status stage ZLP).
    pub async fn control_data_out(&self, dev_addr: u8, buf: &[u8]) -> Result<(), PipeError> {
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let p = self.port as usize;

            let ctl_mps = critical_section::with(|cs| {
                (&*HCD_ALLOC[p].borrow(cs).get()).ctl_mps[dev_addr as usize]
            }) as usize;

            // Select CFIFO for writing (ISEL=1: host writes to device).
            wr(
                core::ptr::addr_of_mut!((*regs).cfifosel),
                FIFOSEL_ISEL | (MBW_32 << FIFOSEL_MBW_SHIFT),
            );
            for _ in 0..1000u32 {
                if rd(core::ptr::addr_of!((*regs).cfifosel)) & FIFOSEL_ISEL != 0 {
                    break;
                }
            }

            // Initialise transfer state.
            critical_section::with(|cs| {
                let xfer = &mut *HCD_XFER[0].borrow(cs).get();
                xfer.buf = buf.as_ptr() as *mut u8;
                xfer.length = buf.len() as u16;
                xfer.remaining = buf.len() as u16;
                xfer.ep_addr = 0x00; // EP0 OUT
                xfer.dev_addr = dev_addr;
            });

            HCD_PIPE_DONE.fetch_and(!1, Ordering::Release);
            HCD_PIPE_NRDY.fetch_and(!1, Ordering::Release);
            HCD_PIPE_STALL.fetch_and(!1, Ordering::Release);

            if buf.is_empty() {
                // ZLP (TRM §28.4.5): clear the buffer with BCLR, then write
                // BVAL to end the (zero-byte) write and commit the packet.
                wr(core::ptr::addr_of_mut!((*regs).cfifoctr), FIFOCTR_BCLR);
                wr(core::ptr::addr_of_mut!((*regs).cfifoctr), FIFOCTR_BVAL);
            } else {
                // Wait (bounded) for BSTS=1 (FIFO writable) before the first
                // packet.  If the FIFO never became ready we must not arm the
                // pipe with nothing staged — with PID=BUF and an empty buffer
                // the SIE never issues OUT tokens, no BEMP arrives, and the
                // transfer would hang.
                let mut ready = false;
                for _ in 0..1000u32 {
                    if rd(pipectr_ptr(regs, 0)) & PIPECTR_BSTS != 0 {
                        ready = true;
                        break;
                    }
                }
                if !ready {
                    return Err(PipeError::Canceled);
                }
                let len = buf.len().min(ctl_mps);
                let fifo = FifoPort::cfifo(regs);
                sw_to_hw_fifo(&fifo, buf.as_ptr(), len);
                if len < ctl_mps {
                    wr(core::ptr::addr_of_mut!((*regs).cfifoctr), FIFOCTR_BVAL);
                }
                critical_section::with(|cs| {
                    let xfer = &mut *HCD_XFER[0].borrow(cs).get();
                    xfer.buf = xfer.buf.add(len);
                    xfer.remaining = xfer.remaining.saturating_sub(len as u16);
                });
            }

            // Arm DCP.
            wr(pipectr_ptr(regs, 0), PIPECTR_PID_BUF);
        }

        // Completion is signalled by pipe0_out_bemp() in the ISR.
        poll_pipe(0).await.map(|_| ())
    }

    // --- Non-control endpoints ---

    /// Allocate a hardware pipe and configure it for `ep_addr` on `dev_addr`.
    ///
    /// `ep_addr` format: bit 7 = direction (1=IN, 0=OUT), bits 3:0 = EP number.
    ///
    /// `binterval` is the USB descriptor bInterval field (in frames for FS interrupt
    /// endpoints; ignored for bulk endpoints).  Pass 0 or 1 for no specific interval.
    pub fn edpt_open(
        &self,
        dev_addr: u8,
        ep_addr: u8,
        ep_type: EndpointType,
        mps: u16,
        binterval: u8,
    ) -> Result<(), HostError> {
        debug_assert!(dev_addr > 0 && (dev_addr as usize) < HCD_MAX_DEV);
        let p = self.port as usize;
        let epn = (ep_addr & 0x0F) as usize;
        let dir_in = ep_addr & 0x80 != 0;

        critical_section::with(|cs| {
            let alloc = unsafe { &mut *HCD_ALLOC[p].borrow(cs).get() };
            let pipe = alloc.alloc_pipe(ep_type).ok_or(HostError::OutOfPipes)?;

            // Reserve packet-buffer blocks for programmable pipes.  Pipes 6-8
            // have hardware-fixed buffers (blocks 4/5/6, BUFNMB writes ignored),
            // so they are skipped; all other pipes (bulk/iso 1-5, interrupt 9)
            // MUST have a disjoint PIPEBUF region or their FIFOs overlap at
            // block 0 (the DCP area) and corrupt each other.  DBLB is set for
            // bulk and iso (see PIPECFG below), so those reserve two banks.
            let pipebuf: Option<(u8, u8)> = if (6..=8).contains(&pipe) {
                None
            } else {
                let double_buf = matches!(ep_type, EndpointType::Bulk | EndpointType::Isochronous);
                match alloc.alloc_pipe_buf(pipe, mps, double_buf) {
                    Some(v) => Some(v),
                    None => {
                        alloc.free_pipe(pipe);
                        return Err(HostError::OutOfPipes);
                    }
                }
            };

            // Record the EP→pipe mapping.
            // dev_addr is 1-based; array index is 0-based.
            alloc.ep_to_pipe[dev_addr as usize - 1][dir_in as usize][epn - 1] = pipe as u8;

            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                let e = &mut *regs;
                let ctr = pipectr_ptr(regs, pipe);

                // Clear FIFO and toggle sequencer.
                wr(ctr, PIPECTR_ACLRM | PIPECTR_SQCLR);
                wr(ctr, 0);

                // Select pipe for configuration.
                wr(core::ptr::addr_of_mut!(e.pipesel), pipe as u16);

                // PIPEMAXP: MPS | (dev_addr << 12).
                wr(
                    core::ptr::addr_of_mut!(e.pipemaxp),
                    ((dev_addr as u16) << PIPEMAXP_DEVSEL_SHIFT) | (mps & PIPEMAXP_MXPS_MASK),
                );

                // PIPECFG.DIR: 1 = host sends (OUT), 0 = host receives (IN).
                // C: cfg = ((1 ^ dir_in) << 4) | epn
                let dir_field: u16 = if dir_in { 0x0000 } else { 0x0010 };
                let type_field: u16 = match ep_type {
                    EndpointType::Bulk => PIPECFG_TYPE_BULK | PIPECFG_SHTNAK | PIPECFG_DBLB,
                    EndpointType::Interrupt => PIPECFG_TYPE_INTR,
                    EndpointType::Isochronous => PIPECFG_TYPE_ISO | PIPECFG_DBLB,
                    EndpointType::Control => unreachable!("control EP handled by DCP"),
                };
                wr(
                    core::ptr::addr_of_mut!(e.pipecfg),
                    type_field | dir_field | (epn as u16 & PIPECFG_EPNUM_MASK),
                );

                // PIPEBUF: assign a disjoint packet-buffer region.  BUFNMB =
                // start block, BUFSIZE = blocks-per-bank − 1 (the DBLB bit
                // supplies the second bank).  Skipped for hardware-fixed pipes.
                if let Some((buf_start, blocks_per_bank)) = pipebuf {
                    let bufsize = (((blocks_per_bank as u16).saturating_sub(1))
                        << PIPEBUF_BUFSIZE_SHIFT)
                        & PIPEBUF_BUFSIZE_MASK;
                    wr(
                        core::ptr::addr_of_mut!(e.pipebuf),
                        (buf_start as u16 & PIPEBUF_BUFNMB_MASK) | bufsize,
                    );
                }

                // PIPEPERI: set IITV for interrupt pipes (valid for pipes 6-9).
                // TRM §28.3.35: IITV = floor(log2(bInterval)).min(7).
                // Bulk pipes (3-5) and pipes 10-15 must have IITV=0.
                let iitv: u16 = if ep_type == EndpointType::Interrupt {
                    let b = binterval.max(1);
                    (u8::BITS - 1 - b.leading_zeros()) as u16
                } else {
                    0
                };
                wr(
                    core::ptr::addr_of_mut!(e.pipeperi),
                    iitv & PIPEPERI_IITV_MASK,
                );

                wr(core::ptr::addr_of_mut!(e.pipesel), 0);

                // Clear stale interrupt flags then enable per-pipe interrupts.
                wr(core::ptr::addr_of_mut!(e.brdysts), !(1u16 << pipe));
                wr(core::ptr::addr_of_mut!(e.nrdysts), !(1u16 << pipe));
                rmw(core::ptr::addr_of_mut!(e.brdyenb), 1 << pipe, 1 << pipe);
                rmw(core::ptr::addr_of_mut!(e.nrdyenb), 1 << pipe, 1 << pipe);

                // OUT pipes are armed immediately (host will send on demand);
                // IN pipes wait until xfer_in() is called.
                if !dir_in {
                    wr(ctr, PIPECTR_PID_BUF);
                }

                // Initialise the saved ep/dev info for the ISR.
                let xfer = &mut *HCD_XFER[pipe].borrow(cs).get();
                xfer.ep_addr = ep_addr;
                xfer.dev_addr = dev_addr;
                xfer.buf = core::ptr::null_mut();
            }

            Ok(())
        })
    }

    /// IN transfer: receive up to `buf.len()` bytes from `ep_addr` on `dev_addr`.
    pub async fn xfer_in(
        &self,
        dev_addr: u8,
        ep_addr: u8,
        buf: &mut [u8],
    ) -> Result<usize, PipeError> {
        let pipe = self
            .ep_to_pipe(dev_addr, ep_addr)
            .ok_or(PipeError::Canceled)?;

        critical_section::with(|cs| unsafe {
            let xfer = &mut *HCD_XFER[pipe].borrow(cs).get();
            xfer.buf = buf.as_mut_ptr();
            xfer.length = buf.len() as u16;
            xfer.remaining = buf.len() as u16;
            xfer.ep_addr = ep_addr;
            xfer.dev_addr = dev_addr;
        });

        let mask = 1u16 << pipe;
        HCD_PIPE_DONE.fetch_and(!mask, Ordering::Release);
        HCD_PIPE_NRDY.fetch_and(!mask, Ordering::Release);
        HCD_PIPE_STALL.fetch_and(!mask, Ordering::Release);

        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Arm pipe to issue IN tokens.
            let ctr = pipectr_ptr(regs, pipe);
            wr(ctr, (rd(ctr) & !PIPECTR_PID_MASK) | PIPECTR_PID_BUF);
        }

        poll_pipe(pipe).await
    }

    /// OUT transfer: send `buf` to `ep_addr` on `dev_addr`.
    pub async fn xfer_out(&self, dev_addr: u8, ep_addr: u8, buf: &[u8]) -> Result<(), PipeError> {
        let pipe = self
            .ep_to_pipe(dev_addr, ep_addr)
            .ok_or(PipeError::Canceled)?;

        critical_section::with(|cs| unsafe {
            let xfer = &mut *HCD_XFER[pipe].borrow(cs).get();
            xfer.buf = buf.as_ptr() as *mut u8;
            xfer.length = buf.len() as u16;
            xfer.remaining = buf.len() as u16;
            xfer.ep_addr = ep_addr;
            xfer.dev_addr = dev_addr;
        });

        let mask = 1u16 << pipe;
        HCD_PIPE_DONE.fetch_and(!mask, Ordering::Release);
        HCD_PIPE_NRDY.fetch_and(!mask, Ordering::Release);
        HCD_PIPE_STALL.fetch_and(!mask, Ordering::Release);

        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let e = &mut *regs;

            if buf.is_empty() {
                // ZLP: select D0FIFO, wait FRDY, then BCLR + BVAL (TRM
                // §28.4.5: "to send a zero-length packet, the BCLR bit must be
                // used to clear the buffer and then the BVAL bit is set").
                wr(
                    core::ptr::addr_of_mut!(e.d0fifosel),
                    pipe as u16 | (MBW_32 << FIFOSEL_MBW_SHIFT),
                );
                while rd(core::ptr::addr_of!(e.d0fifosel)) & FIFOSEL_CURPIPE_MASK != pipe as u16 {}
                while rd(core::ptr::addr_of!(e.d0fifoctr)) & FIFOCTR_FRDY == 0 {}
                wr(core::ptr::addr_of_mut!(e.d0fifoctr), FIFOCTR_BCLR);
                wr(core::ptr::addr_of_mut!(e.d0fifoctr), FIFOCTR_BVAL);
                wr(core::ptr::addr_of_mut!(e.d0fifosel), 0);
                while rd(core::ptr::addr_of!(e.d0fifosel)) & FIFOSEL_CURPIPE_MASK != 0 {}
            } else {
                // Write first packet to D0FIFO (16-bit access for speed).
                wr(
                    core::ptr::addr_of_mut!(e.d0fifosel),
                    pipe as u16 | (MBW_32 << FIFOSEL_MBW_SHIFT),
                );
                while rd(core::ptr::addr_of!(e.d0fifosel)) & FIFOSEL_CURPIPE_MASK != pipe as u16 {}
                while rd(core::ptr::addr_of!(e.d0fifoctr)) & FIFOCTR_FRDY == 0 {}

                // Read MPS from PIPEMAXP.
                wr(core::ptr::addr_of_mut!(e.pipesel), pipe as u16);
                let mps = (rd(core::ptr::addr_of!(e.pipemaxp)) & PIPEMAXP_MXPS_MASK) as usize;
                wr(core::ptr::addr_of_mut!(e.pipesel), 0);

                let len = buf.len().min(mps);
                let fifo = FifoPort::d0fifo(regs);
                sw_to_hw_fifo(&fifo, buf.as_ptr(), len);
                if len < mps {
                    fifo_bval(&fifo); // short packet — commit now
                }

                critical_section::with(|cs| {
                    let xfer = &mut *HCD_XFER[pipe].borrow(cs).get();
                    xfer.buf = xfer.buf.add(len);
                    xfer.remaining = xfer.remaining.saturating_sub(len as u16);
                });

                wr(core::ptr::addr_of_mut!(e.d0fifosel), 0);
                while rd(core::ptr::addr_of!(e.d0fifosel)) & FIFOSEL_CURPIPE_MASK != 0 {}
            }

            // Arm the pipe.
            let ctr = pipectr_ptr(regs, pipe);
            wr(ctr, (rd(ctr) & !PIPECTR_PID_MASK) | PIPECTR_PID_BUF);
        }

        poll_pipe(pipe).await.map(|_| ())
    }

    /// Clear the STALL condition on an endpoint and re-enable it.
    pub fn clear_stall(&self, dev_addr: u8, ep_addr: u8) {
        if let Some(pipe) = self.ep_to_pipe(dev_addr, ep_addr) {
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                let ctr = pipectr_ptr(regs, pipe);
                let pid = rd(ctr) & PIPECTR_PID_MASK;
                // Transition sequence: STALL → STALL10 → NAK per manual.
                if pid & 0b10 != 0 {
                    wr(ctr, pid & 0b10); // clear low bit → STALL10
                    wr(ctr, 0); // → NAK
                }
                // Reset data toggle.
                wr(ctr, PIPECTR_SQCLR);
                // Re-arm OUT endpoints immediately.
                if ep_addr & 0x80 == 0 {
                    wr(ctr, (rd(ctr) & !PIPECTR_PID_MASK) | PIPECTR_PID_BUF);
                }
            }
        }
    }

    // --- Internal helpers ---

    /// Prepare the DCP for a control-transfer **status stage**.
    ///
    /// TRM §28.4.6 (1)(c): the status-stage ZLP travels in the direction
    /// *opposite* to the data stage, so DCPCFG.DIR must be flipped, and its
    /// data PID is always DATA1, so SQSET must be written.  Both DCPCFG and
    /// SQSET may only be modified while PID = NAK (after the BUF→NAK
    /// transition has completed, PBUSY = 0).
    ///
    /// `dir_out` = true → host transmits the ZLP (status of a control read);
    /// false → host receives the ZLP (status of a control write / no-data).
    fn dcp_prepare_status(&self, dir_out: bool) {
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Halt the DCP and wait for the NAK transition to finish.
            let ctr = rd(pipectr_ptr(regs, 0));
            wr(
                pipectr_ptr(regs, 0),
                (ctr & !PIPECTR_PID_MASK) | PIPECTR_PID_NAK,
            );
            while rd(pipectr_ptr(regs, 0)) & PIPECTR_PBUSY != 0 {}
            // Status direction; SHTNAK only in the receiving direction
            // (must be 0 when transmitting, TRM §28.3.28).
            let dcpcfg = if dir_out { DCPCFG_DIR } else { DCPCFG_SHTNAK };
            wr(core::ptr::addr_of_mut!((*regs).dcpcfg), dcpcfg);
            // Status-stage ZLP is always DATA1.
            wr(pipectr_ptr(regs, 0), PIPECTR_SQSET);
        }
    }

    fn ep_to_pipe(&self, dev_addr: u8, ep_addr: u8) -> Option<usize> {
        if dev_addr == 0 || (dev_addr as usize) >= HCD_MAX_DEV {
            return None;
        }
        let epn = (ep_addr & 0x0F) as usize;
        let dir_in = ep_addr & 0x80 != 0;
        if epn == 0 {
            return Some(0);
        } // EP0 is always pipe 0
        let p = self.port as usize;
        critical_section::with(|cs| {
            let alloc = unsafe { &*HCD_ALLOC[p].borrow(cs).get() };
            let pipe = alloc.ep_to_pipe[dev_addr as usize - 1][dir_in as usize][epn - 1] as usize;
            if pipe == 0 { None } else { Some(pipe) }
        })
    }
}

// ---------------------------------------------------------------------------
// Async poll helper
// ---------------------------------------------------------------------------

/// Future that resolves when pipe `n`'s transfer completes or errors.
/// Returns `Ok(bytes_transferred)` or the appropriate `PipeError`.
async fn poll_pipe(n: usize) -> Result<usize, PipeError> {
    core::future::poll_fn(move |cx| {
        HCD_PIPE_WAKERS[n].register(cx.waker());
        let mask = 1u16 << n;
        if HCD_PIPE_DONE.load(Ordering::Acquire) & mask != 0 {
            HCD_PIPE_DONE.fetch_and(!mask, Ordering::Release);
            let (length, remaining) = critical_section::with(|cs| unsafe {
                let x = &*HCD_XFER[n].borrow(cs).get();
                (x.length, x.remaining)
            });
            return core::task::Poll::Ready(Ok((length - remaining) as usize));
        }
        if HCD_PIPE_STALL.load(Ordering::Acquire) & mask != 0 {
            HCD_PIPE_STALL.fetch_and(!mask, Ordering::Release);
            return core::task::Poll::Ready(Err(PipeError::Stall));
        }
        if HCD_PIPE_NRDY.load(Ordering::Acquire) & mask != 0 {
            HCD_PIPE_NRDY.fetch_and(!mask, Ordering::Release);
            return core::task::Poll::Ready(Err(PipeError::BadResponse));
        }
        core::task::Poll::Pending
    })
    .await
}

// ---------------------------------------------------------------------------
// Interrupt handler
// ---------------------------------------------------------------------------

/// USB host-mode interrupt handler.
///
/// Wire to GIC IRQ 73 (port 0) or 74 (port 1).  Must only be called from IRQ
/// context on a single-core bare-metal system.
///
/// # Safety
///
/// Must be called from the hardware IRQ handler for the corresponding port.
pub unsafe fn hcd_int_handler(port: u8) {
    unsafe {
        let regs = Rusb1Regs::ptr(port);
        let e = &mut *regs;

        let is0 = rd(core::ptr::addr_of!(e.intsts0));
        let is1 = rd(core::ptr::addr_of!(e.intsts1));

        // Clear sticky bits by writing 0 to each bit that was set (RC-W0 protocol).
        const STICKY1: u16 = INTSTS1_SACK | INTSTS1_SIGN | INTSTS1_ATTCH | INTSTS1_DTCH;
        wr(core::ptr::addr_of_mut!(e.intsts1), !(STICKY1 & is1));
        const STICKY0: u16 = INTSTS0_BRDY | INTSTS0_NRDY | INTSTS0_BEMP;
        wr(core::ptr::addr_of_mut!(e.intsts0), !(STICKY0 & is0));

        // Mask with enable registers so we only handle enabled sources.
        let is1 = is1 & rd(core::ptr::addr_of!(e.intenb1));
        let is0 = is0 & rd(core::ptr::addr_of!(e.intenb0));

        // ── SACK: setup packet was ACKed ──────────────────────────────────────
        if is1 & INTSTS1_SACK != 0 {
            // Advance DATA-stage toggle to DATA1.
            let ctr = rd(pipectr_ptr(regs, 0));
            wr(pipectr_ptr(regs, 0), ctr | PIPECTR_SQSET);
            HCD_EVENTS.fetch_or(EVT_SACK, Ordering::Release);
            HCD_EVENT_WAKER.wake();
        }

        // ── SIGN: setup packet was ignored (NAK / error) ──────────────────────
        if is1 & INTSTS1_SIGN != 0 {
            HCD_EVENTS.fetch_or(EVT_SIGN, Ordering::Release);
            HCD_EVENT_WAKER.wake();
        }

        // ── ATTCH: a device connected ─────────────────────────────────────────
        if is1 & INTSTS1_ATTCH != 0 {
            // Do NOT enable SOF (UACT) here: the reference leaves it off until
            // the bus reset completes.  Generating SOF to a freshly-attached,
            // un-reset device is non-standard; `bus_reset` owns UACT (it clears
            // it before asserting USBRST and re-enables it as the reset ends).
            critical_section::with(|cs| {
                (&mut *HCD_ALLOC[port as usize].borrow(cs).get()).need_reset = true;
            });
            // Switch to detecting detach.
            let enb = rd(core::ptr::addr_of!(e.intenb1));
            wr(
                core::ptr::addr_of_mut!(e.intenb1),
                (enb & !INTENB1_ATTCHE) | INTENB1_DTCHE,
            );
            HCD_EVENTS.fetch_or(EVT_ATTACH, Ordering::Release);
            HCD_EVENT_WAKER.wake();
        }

        // ── DTCH: the device was removed ─────────────────────────────────────
        if is1 & INTSTS1_DTCH != 0 {
            rmw(core::ptr::addr_of_mut!(e.dvstctr0), DVSTCTR0_UACT, 0);
            // Cancel any pending SETUP.
            let dcpctr = rd(pipectr_ptr(regs, 0));
            if dcpctr & DCPCTR_SUREQ != 0 {
                wr(pipectr_ptr(regs, 0), dcpctr | DCPCTR_SUREQCLR);
            }
            // Switch back to detecting attach.
            let enb = rd(core::ptr::addr_of!(e.intenb1));
            wr(
                core::ptr::addr_of_mut!(e.intenb1),
                (enb & !INTENB1_DTCHE) | INTENB1_ATTCHE,
            );
            // Fail every in-flight transfer.  The controller has idled the port
            // (UACT=0) and will emit no further BRDY/NRDY, so any future
            // awaiting a pipe or a SETUP completion would otherwise hang
            // forever.  Mark all pipes failed + wake them, and post EVT_SIGN so
            // an awaiting setup_send resolves to an error.  (Idle pipes clear
            // their NRDY bit when the next transfer arms them.)
            for (n, waker) in HCD_PIPE_WAKERS.iter().enumerate() {
                HCD_PIPE_NRDY.fetch_or(1u16 << n, Ordering::Release);
                waker.wake();
            }
            HCD_EVENTS.fetch_or(EVT_DETACH | EVT_SIGN, Ordering::Release);
            HCD_EVENT_WAKER.wake();
        }

        // ── BEMP: FIFO transmitted — continue control OUT ────────────────────
        if is0 & INTSTS0_BEMP != 0 {
            let bempsts = rd(core::ptr::addr_of!(e.bempsts));
            let bempenb = rd(core::ptr::addr_of!(e.bempenb));
            let active = bempsts & bempenb;
            wr(core::ptr::addr_of_mut!(e.bempsts), !active);
            if active & 1 != 0 {
                pipe0_out_bemp(regs);
            }
        }

        // ── NRDY: NAK or STALL received ──────────────────────────────────────
        if is0 & INTSTS0_NRDY != 0 {
            let nrdyenb = rd(core::ptr::addr_of!(e.nrdyenb));
            let mut active = rd(core::ptr::addr_of!(e.nrdysts)) & nrdyenb;
            wr(core::ptr::addr_of_mut!(e.nrdysts), !active);
            while active != 0 {
                let n = active.trailing_zeros() as usize;
                pipe_nrdy(regs, n);
                active &= !(1u16 << n);
            }
        }

        // ── BRDY: data received (IN) or slot ready (OUT) ─────────────────────
        if is0 & INTSTS0_BRDY != 0 {
            let brdyenb = rd(core::ptr::addr_of!(e.brdyenb));
            let mut active = rd(core::ptr::addr_of!(e.brdysts)) & brdyenb;
            wr(core::ptr::addr_of_mut!(e.brdysts), !active);
            while active != 0 {
                let n = active.trailing_zeros() as usize;
                pipe_brdy(regs, n);
                active &= !(1u16 << n);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// ISR helpers — all must be called only from IRQ context
// ---------------------------------------------------------------------------

/// BRDY on pipe 0 IN (control data stage): read data from CFIFO.
unsafe fn pipe0_in_brdy(regs: *mut Rusb1Regs) {
    unsafe {
        // SAFETY: called from IRQ only; no concurrent async access to CFIFO.
        let cs = critical_section::CriticalSection::new();
        let xfer = &mut *HCD_XFER[0].borrow(cs).get();

        let mps = (rd(core::ptr::addr_of!((*regs).dcpmaxp)) & DCPMAXP_MXPS_MASK) as usize;
        let vld = (rd(core::ptr::addr_of!((*regs).cfifoctr)) & FIFOCTR_DTLN_MASK) as usize;
        let rem = xfer.remaining as usize;
        let len = rem.min(mps).min(vld);

        // Byte-level read from CFIFO.  The FIFO port is selected at MBW=8; per
        // TRM Table 28.9 (8-bit access, BIGEND=0) the valid received byte sits
        // on bits[31:24] = CPU byte-address base+3, and each MBW=8 access pops
        // one byte and advances the FIFO.  (The unvalidated TinyUSB
        // `pipe_read_packet` this was modeled on read base+0 — the *prohibited*
        // lane — and would return garbage on silicon.)
        if len > 0 && !xfer.buf.is_null() {
            let cfifo_byte = (regs as usize
                + core::mem::offset_of!(super::regs::Rusb1Regs, cfifo)
                + 3) as *const u8;
            for i in 0..len {
                *xfer.buf.add(i) = cfifo_byte.read_volatile();
            }
            xfer.buf = xfer.buf.add(len);
            xfer.remaining = xfer.remaining.saturating_sub(len as u16);
        }

        // BCLR after a short packet to release the FIFO buffer.
        if len < mps {
            wr(core::ptr::addr_of_mut!((*regs).cfifoctr), FIFOCTR_BCLR);
        }

        let done = (len < mps) || (xfer.remaining == 0);
        if done {
            xfer.buf = core::ptr::null_mut();
            // NAK to stop receiving further IN tokens.
            wr(pipectr_ptr(regs, 0), PIPECTR_PID_NAK);
            HCD_PIPE_DONE.fetch_or(1, Ordering::Release);
            HCD_PIPE_WAKERS[0].wake();
        } else {
            // Re-arm for the next IN packet.
            wr(pipectr_ptr(regs, 0), PIPECTR_PID_BUF);
        }
    }
}

/// BEMP on pipe 0 OUT (control data stage): write next chunk to CFIFO.
unsafe fn pipe0_out_bemp(regs: *mut Rusb1Regs) {
    unsafe {
        let cs = critical_section::CriticalSection::new();
        let xfer = &mut *HCD_XFER[0].borrow(cs).get();

        let rem = xfer.remaining as usize;
        if rem == 0 || xfer.buf.is_null() {
            xfer.buf = core::ptr::null_mut();
            HCD_PIPE_DONE.fetch_or(1, Ordering::Release);
            HCD_PIPE_WAKERS[0].wake();
            return;
        }

        let mps = (rd(core::ptr::addr_of!((*regs).dcpmaxp)) & DCPMAXP_MXPS_MASK) as usize;
        let len = rem.min(mps);
        let fifo = FifoPort::cfifo(regs);
        sw_to_hw_fifo(&fifo, xfer.buf, len);
        if len < mps {
            wr(core::ptr::addr_of_mut!((*regs).cfifoctr), FIFOCTR_BVAL);
        }
        xfer.buf = xfer.buf.add(len);
        xfer.remaining = xfer.remaining.saturating_sub(len as u16);
        // BEMP will fire again after this chunk is transmitted.
    }
}

/// BRDY on a non-control IN pipe: read data from D0FIFO.
unsafe fn pipe_brdy_in(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let cs = critical_section::CriticalSection::new();
        let xfer = &mut *HCD_XFER[n].borrow(cs).get();

        if xfer.buf.is_null() {
            // No buffer — discard and BCLR.
            wr(core::ptr::addr_of_mut!((*regs).d0fifosel), n as u16);
            while rd(core::ptr::addr_of!((*regs).d0fifosel)) & FIFOSEL_CURPIPE_MASK != n as u16 {}
            wr(core::ptr::addr_of_mut!((*regs).d0fifoctr), FIFOCTR_BCLR);
            wr(core::ptr::addr_of_mut!((*regs).d0fifosel), 0);
            return;
        }

        // Select D0FIFO on this pipe for reading.
        wr(core::ptr::addr_of_mut!((*regs).d0fifosel), n as u16);
        while rd(core::ptr::addr_of!((*regs).d0fifosel)) & FIFOSEL_CURPIPE_MASK != n as u16 {}
        while rd(core::ptr::addr_of!((*regs).d0fifoctr)) & FIFOCTR_FRDY == 0 {}

        // PIPEMAXP is gated by PIPESEL, not by D0FIFOSEL.CURPIPE — select pipe explicitly.
        wr(core::ptr::addr_of_mut!((*regs).pipesel), n as u16);
        let mps = (rd(core::ptr::addr_of!((*regs).pipemaxp)) & PIPEMAXP_MXPS_MASK) as usize;
        wr(core::ptr::addr_of_mut!((*regs).pipesel), 0);
        let vld = (rd(core::ptr::addr_of!((*regs).d0fifoctr)) & FIFOCTR_DTLN_MASK) as usize;
        let rem = xfer.remaining as usize;
        let len = rem.min(mps).min(vld);

        // Byte-level read from D0FIFO.  Selected at MBW=8; per TRM Table 28.9
        // (8-bit access, BIGEND=0) the valid byte is on bits[31:24] = base+3,
        // and each access pops one byte and advances the FIFO.  (Reading base+0,
        // as the unvalidated TinyUSB reference did, samples the prohibited lane.)
        if len > 0 {
            let d0fifo_byte = (regs as usize
                + core::mem::offset_of!(super::regs::Rusb1Regs, d0fifo)
                + 3) as *const u8;
            for i in 0..len {
                *xfer.buf.add(i) = d0fifo_byte.read_volatile();
            }
            xfer.buf = xfer.buf.add(len);
            xfer.remaining = xfer.remaining.saturating_sub(len as u16);
        }

        if len < mps {
            wr(core::ptr::addr_of_mut!((*regs).d0fifoctr), FIFOCTR_BCLR);
        }

        // Deselect D0FIFO.
        wr(core::ptr::addr_of_mut!((*regs).d0fifosel), 0);
        while rd(core::ptr::addr_of!((*regs).d0fifosel)) & FIFOSEL_CURPIPE_MASK != 0 {}

        let done = (len < mps) || (xfer.remaining == 0);
        if done {
            xfer.buf = core::ptr::null_mut();
            HCD_PIPE_DONE.fetch_or(1u16 << n, Ordering::Release);
            HCD_PIPE_WAKERS[n].wake();
        }
        // Hardware continues issuing IN tokens until PID changes from BUF.
    }
}

/// BRDY on a non-control OUT pipe: write next chunk to D0FIFO.
unsafe fn pipe_brdy_out(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let cs = critical_section::CriticalSection::new();
        let xfer = &mut *HCD_XFER[n].borrow(cs).get();

        let rem = xfer.remaining as usize;
        if rem == 0 || xfer.buf.is_null() {
            xfer.buf = core::ptr::null_mut();
            HCD_PIPE_DONE.fetch_or(1u16 << n, Ordering::Release);
            HCD_PIPE_WAKERS[n].wake();
            return;
        }

        // Select at MBW=32 to match sw_to_hw_fifo's access width, so its internal
        // MBW assertion is a same-value write (no 8→32 port-switch re-trigger).
        wr(
            core::ptr::addr_of_mut!((*regs).d0fifosel),
            n as u16 | (MBW_32 << FIFOSEL_MBW_SHIFT),
        );
        while rd(core::ptr::addr_of!((*regs).d0fifosel)) & FIFOSEL_CURPIPE_MASK != n as u16 {}
        while rd(core::ptr::addr_of!((*regs).d0fifoctr)) & FIFOCTR_FRDY == 0 {}

        // PIPEMAXP is gated by PIPESEL, not by D0FIFOSEL.CURPIPE — select pipe explicitly.
        wr(core::ptr::addr_of_mut!((*regs).pipesel), n as u16);
        let mps = (rd(core::ptr::addr_of!((*regs).pipemaxp)) & PIPEMAXP_MXPS_MASK) as usize;
        wr(core::ptr::addr_of_mut!((*regs).pipesel), 0);
        let len = rem.min(mps);
        let fifo = FifoPort::d0fifo(regs);
        sw_to_hw_fifo(&fifo, xfer.buf, len);
        if len < mps {
            fifo_bval(&fifo);
        }
        xfer.buf = xfer.buf.add(len);
        xfer.remaining = xfer.remaining.saturating_sub(len as u16);

        wr(core::ptr::addr_of_mut!((*regs).d0fifosel), 0);
        while rd(core::ptr::addr_of!((*regs).d0fifosel)) & FIFOSEL_CURPIPE_MASK != 0 {}
        // BRDY fires again when the SIE has consumed this chunk.
    }
}

/// Dispatch a BRDY event for pipe `n`.
unsafe fn pipe_brdy(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let ep_addr = {
            let cs = critical_section::CriticalSection::new();
            (*HCD_XFER[n].borrow(cs).get()).ep_addr
        };
        let dir_in = ep_addr & 0x80 != 0;

        if n == 0 {
            if dir_in {
                pipe0_in_brdy(regs);
            }
            // Pipe-0 OUT is handled via BEMP, not BRDY.
        } else if dir_in {
            pipe_brdy_in(regs, n);
        } else {
            pipe_brdy_out(regs, n);
        }
    }
}

/// Handle NRDY (NAK or STALL) for pipe `n`.
unsafe fn pipe_nrdy(regs: *mut Rusb1Regs, n: usize) {
    unsafe {
        let pid = rd(pipectr_ptr(regs, n)) & PIPECTR_PID_MASK;
        // PID 0b10 or 0b11 = STALL; otherwise = NAK / general failure.
        if pid & 0b10 != 0 {
            HCD_PIPE_STALL.fetch_or(1u16 << n, Ordering::Release);
        } else {
            HCD_PIPE_NRDY.fetch_or(1u16 << n, Ordering::Release);
        }
        HCD_PIPE_WAKERS[n].wake();
    }
}

// ---------------------------------------------------------------------------
// embassy-usb-host: UsbHostController + Rusb1Allocator + Rusb1Pipe
// ---------------------------------------------------------------------------

/// An active USB pipe allocated by [`Rusb1Allocator::alloc_pipe`].
///
/// On drop the hardware pipe is freed and the PIPECFG/PIPEMAXP registers are
/// cleared (control pipes that use pipe 0 are exempt — the DCP is permanent).
pub struct Rusb1Pipe<T: pipe::Type, D: pipe::Direction> {
    port: u8,
    /// Hardware pipe number.  0 = control (DCP).
    pipe: u8,
    /// USB device address this pipe targets.
    dev_addr: u8,
    /// Endpoint info (for Drop).
    ep_info: EndpointInfo,
    _phantom: core::marker::PhantomData<(T, D)>,
}

impl<T: pipe::Type, D: pipe::Direction> Drop for Rusb1Pipe<T, D> {
    fn drop(&mut self) {
        // Pipe 0 (DCP / control) is shared and permanently allocated.
        // The hardware pipe slot 0 is never freed.
        if self.pipe == 0 {
            return;
        }
        let pipe = self.pipe as usize;
        let p = self.port as usize;
        let ep_addr = u8::from(self.ep_info.addr);
        let epn = (ep_addr & 0x0F) as usize;
        let dir_in = ep_addr & 0x80 != 0;

        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let e = &mut *regs;
            wr(pipectr_ptr(regs, pipe), 0);
            rmw(core::ptr::addr_of_mut!(e.nrdyenb), 1 << pipe, 0);
            rmw(core::ptr::addr_of_mut!(e.brdyenb), 1 << pipe, 0);
            wr(core::ptr::addr_of_mut!(e.pipesel), pipe as u16);
            wr(core::ptr::addr_of_mut!(e.pipecfg), 0);
            wr(core::ptr::addr_of_mut!(e.pipemaxp), 0);
            wr(core::ptr::addr_of_mut!(e.pipesel), 0);
        }
        critical_section::with(|cs| {
            let alloc = unsafe { &mut *HCD_ALLOC[p].borrow(cs).get() };
            alloc.free_pipe(pipe);
            if self.dev_addr > 0 && (self.dev_addr as usize) < HCD_MAX_DEV && epn > 0 && epn <= 15 {
                alloc.ep_to_pipe[self.dev_addr as usize - 1][dir_in as usize][epn - 1] = 0;
            }
        });
    }
}

// UsbHostController ───────────────────────────────────────────────────────────

impl<'d> UsbHostController<'d> for Rusb1HostDriver {
    type Allocator = Rusb1Allocator;

    fn allocator(&self) -> Self::Allocator {
        Rusb1Allocator { port: self.port }
    }

    async fn wait_for_device_event(&mut self) -> DeviceEvent {
        loop {
            // Wait for either attach or detach.
            let ev = core::future::poll_fn(|cx| {
                HCD_EVENT_WAKER.register(cx.waker());
                let ev = HCD_EVENTS.fetch_and(!(EVT_ATTACH | EVT_DETACH), Ordering::Acquire);
                if ev & (EVT_ATTACH | EVT_DETACH) != 0 {
                    return core::task::Poll::Ready(ev);
                }
                core::task::Poll::Pending
            })
            .await;

            if ev & EVT_DETACH != 0 {
                return DeviceEvent::Disconnected;
            }

            if ev & EVT_ATTACH != 0 {
                // Spec: issue bus reset before reporting speed.
                Rusb1HostDriver::bus_reset(&*self).await;
                let speed = self.port_speed();
                return DeviceEvent::Connected(speed);
            }
        }
    }

    async fn bus_reset(&mut self) {
        // Delegate to the existing inherent async method.
        Rusb1HostDriver::bus_reset(&*self).await;
    }
}

// UsbHostAllocator ────────────────────────────────────────────────────────────

/// Pipe allocator for one RUSB1 host port.
///
/// Cheaply clonable — all controller state lives in module statics, so the
/// allocator only carries the port index.
#[derive(Clone)]
pub struct Rusb1Allocator {
    port: u8,
}

impl<'d> UsbHostAllocator<'d> for Rusb1Allocator {
    type Pipe<T: pipe::Type, D: pipe::Direction> = Rusb1Pipe<T, D>;

    fn alloc_pipe<T: pipe::Type, D: pipe::Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        split: Option<SplitInfo>,
    ) -> Result<Self::Pipe<T, D>, HostError> {
        let drv = Rusb1HostDriver { port: self.port };

        // Decode the bus topology.  A directly attached device uses hub 0 /
        // port 0 at the root-port speed; a split (hub-attached) device carries
        // the location of its hub's Transaction Translator and its own
        // low/full speed.
        let (hub_addr, hub_port, speed) = match split {
            Some(info) => (
                info.hub_addr(),
                info.port(),
                match info.device_speed() {
                    SplitSpeed::Low => Speed::Low,
                    SplitSpeed::Full => Speed::Full,
                },
            ),
            None => (0, 0, drv.port_speed()),
        };

        match T::ep_type() {
            EndpointType::Control => {
                // Pipe 0 (DCP).  Programme DCPMAXP + DEVADD for this device.
                drv.device_open(addr, endpoint.max_packet_size, speed, hub_addr, hub_port)?;
                Ok(Rusb1Pipe {
                    port: self.port,
                    pipe: 0,
                    dev_addr: addr,
                    ep_info: *endpoint,
                    _phantom: core::marker::PhantomData,
                })
            }
            other => {
                // DEVADDn (hub topology + speed) was programmed when the
                // control pipe for this address was allocated; only the pipe's
                // PIPEMAXP/PIPECFG need configuring here.
                let ep_addr = u8::from(endpoint.addr);
                drv.edpt_open(
                    addr,
                    ep_addr,
                    other,
                    endpoint.max_packet_size,
                    endpoint.interval_ms,
                )?;
                let pipe = drv.ep_to_pipe(addr, ep_addr).ok_or(HostError::OutOfPipes)? as u8;
                Ok(Rusb1Pipe {
                    port: self.port,
                    pipe,
                    dev_addr: addr,
                    ep_info: *endpoint,
                    _phantom: core::marker::PhantomData,
                })
            }
        }
    }
}

// UsbPipe ─────────────────────────────────────────────────────────────────────

impl<T: pipe::Type, D: pipe::Direction> UsbPipe<T, D> for Rusb1Pipe<T, D> {
    /// Control IN: SETUP (IN direction) → DATA IN → STATUS (OUT ZLP).
    async fn control_in(&mut self, setup: &[u8; 8], buf: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        let drv = Rusb1HostDriver { port: self.port };
        drv.setup_send(self.dev_addr, setup).await?;
        let n = drv.control_data_in(self.dev_addr, buf).await?;
        // Status stage: OUT ZLP, opposite direction to the data stage, DATA1.
        drv.dcp_prepare_status(true);
        drv.control_data_out(self.dev_addr, &[]).await?;
        Ok(n)
    }

    /// Control OUT: SETUP (OUT direction) → [DATA OUT] → STATUS (IN ZLP).
    async fn control_out(&mut self, setup: &[u8; 8], buf: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        let drv = Rusb1HostDriver { port: self.port };
        drv.setup_send(self.dev_addr, setup).await?;
        if !buf.is_empty() {
            drv.control_data_out(self.dev_addr, buf).await?;
        }
        // Status stage: IN ZLP, opposite direction to the data stage, DATA1.
        drv.dcp_prepare_status(false);
        let mut dummy = [];
        drv.control_data_in(self.dev_addr, &mut dummy).await?;
        Ok(())
    }

    /// Non-control IN transfer on this pipe.
    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        let drv = Rusb1HostDriver { port: self.port };
        let ep_addr = u8::from(self.ep_info.addr);
        drv.xfer_in(self.dev_addr, ep_addr, buf).await
    }

    /// Non-control OUT transfer on this pipe.
    async fn request_out(
        &mut self,
        buf: &[u8],
        _ensure_transaction_end: bool,
    ) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        let drv = Rusb1HostDriver { port: self.port };
        let ep_addr = u8::from(self.ep_info.addr);
        drv.xfer_out(self.dev_addr, ep_addr, buf).await
    }

    /// Configure transfer timeouts for this pipe.
    ///
    /// The RUSB1 peripheral does not have a hardware timeout counter; this
    /// configuration is accepted but not acted upon.
    fn set_timeout(&mut self, _timeout: TimeoutConfig)
    where
        T: pipe::IsControl,
    {
    }

    /// Reset the host-side data toggle on this bulk/interrupt pipe to DATA0.
    fn reset_data_toggle(&mut self)
    where
        T: pipe::IsBulkOrInterrupt,
    {
        // The DCP (pipe 0) carries no bulk/interrupt toggle.
        if self.pipe == 0 {
            return;
        }
        let dir_in = u8::from(self.ep_info.addr) & 0x80 != 0;
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let ctr = pipectr_ptr(regs, self.pipe as usize);
            // SQCLR (reset toggle to DATA0) may only be written while the pipe
            // is halted: NAK it and wait for the SIE to go idle first.
            wr(ctr, (rd(ctr) & !PIPECTR_PID_MASK) | PIPECTR_PID_NAK);
            while rd(ctr) & PIPECTR_PBUSY != 0 {}
            wr(ctr, PIPECTR_SQCLR);
            // OUT pipes are re-armed immediately; IN pipes wait for request_in().
            if !dir_in {
                wr(ctr, (rd(ctr) & !PIPECTR_PID_MASK) | PIPECTR_PID_BUF);
            }
        }
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn dev_addr_zero_is_not_supported() {
        // Address 0 is the enumeration default address, never a target.
        assert!(!dev_addr_supported(0));
    }

    #[test]
    fn dev_addr_in_range_is_supported() {
        for addr in 1..HCD_MAX_DEV as u8 {
            assert!(dev_addr_supported(addr), "addr {addr} should be supported");
        }
    }

    #[test]
    fn dev_addr_at_or_above_max_is_not_supported() {
        // HCD_MAX_DEV is exclusive: ctl_mps and ep_to_pipe are sized from it.
        assert!(!dev_addr_supported(HCD_MAX_DEV as u8));
        assert!(!dev_addr_supported(127));
    }

    #[test]
    fn host_pipe_buffers_are_disjoint_and_clear_of_the_dcp() {
        // Regression: host-mode pipes previously never programmed PIPEBUF, so
        // every bulk/iso pipe defaulted to block 0 (the DCP region) and
        // overlapped.  Two 512-byte double-buffered bulk pipes must now get
        // disjoint regions, both clear of the reserved blocks 0-6.
        let mut a = HcdAlloc::new();
        let p1 = a.alloc_pipe(EndpointType::Bulk).unwrap();
        let (s1, n1) = a.alloc_pipe_buf(p1, 512, true).unwrap();
        let p2 = a.alloc_pipe(EndpointType::Bulk).unwrap();
        let (s2, n2) = a.alloc_pipe_buf(p2, 512, true).unwrap();

        assert_eq!(n1, 8, "512 bytes = 8 blocks per bank");
        assert_eq!(n2, 8);
        assert!(s1 >= 7, "must not touch reserved blocks 0-6, got {s1}");
        assert!(s2 >= 7, "must not touch reserved blocks 0-6, got {s2}");
        // Each reserves two banks (DBLB); ranges must not intersect.
        let (e1, e2) = (s1 as usize + 2 * n1 as usize, s2 as usize + 2 * n2 as usize);
        let disjoint = (e1 <= s2 as usize) || (e2 <= s1 as usize);
        assert!(disjoint, "pipe buffers overlap: [{s1},{e1}) vs [{s2},{e2})");

        // Freeing the pipe returns its blocks to the pool.
        a.free_pipe(p1);
        assert_eq!(a.buf_reserved[p1], 0);
    }
}
