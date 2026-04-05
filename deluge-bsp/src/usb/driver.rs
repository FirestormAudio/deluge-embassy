//! RUSB1 USB device-mode driver — implements `embassy_usb_driver::Driver`.
//!
//! ## Architecture
//!
//! ```text
//! embassy-usb ──┐
//!               │  Driver<'a> trait
//!               ▼
//!          Rusb1Driver
//!         /          \
//!   Rusb1Bus    Rusb1ControlPipe
//!         \          /
//!        dcd_int_handler   ← GIC IRQ 73/74
//!              |
//!         pipe::{pipe_xfer_in_start, pipe_xfer_out_brdy, pipe_xfer_in_bemp}
//!              |
//!         fifo::{hw_to_sw_fifo, sw_to_hw_fifo}
//! ```
//!
//! ## Interrupt handling
//!
//! The ISR `dcd_int_handler` reads INTSTS0 / BRDYSTS / BEMPSTS each time
//! hardware fires the **GIC** interrupt and:
//!
//! - **VBINT** (VBUS change): queues a `BUS_EVENT_VBUS` bit.
//! - **DVST** (device-state change): queues reset / suspend / resume events.
//! - **CTRT** (control-transfer ready): records CTSQ and queues event.
//! - **BRDY** (buffer-ready for OUT pipe): calls `pipe_xfer_out_brdy`; wakes
//!   the per-pipe `AtomicWaker` on completion.
//! - **BEMP** (buffer-empty for IN pipe): calls `pipe_xfer_in_bemp`; wakes on
//!   completion.
//!
//! ## Register-interrupt caveat (RZ/A1L §28.3.8)
//!
//! `INTENB0`, `BEMPENB`, and `BRDYENB` must not be written before VBUS rises.
//! This driver enables them inside `Rusb1Bus::enable()` which is called after
//! `Driver::start()` once embassy-usb has confirmed VBUS.

use core::sync::atomic::{AtomicU8, AtomicU16, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Timer;
use embassy_usb_driver::{
    Bus, ControlPipe, Direction, Driver, EndpointAddress, EndpointAllocError, EndpointError,
    EndpointIn, EndpointInfo, EndpointOut, EndpointType, Event, Unsupported,
};

use super::fifo::{
    FifoPort, fifo_bclr, fifo_bval, fifo_dtln, fifo_is_ready, fifo_select_pipe, hw_to_sw_fifo,
    sw_to_hw_fifo,
};
use super::pipe::{
    BufAllocator, PIPE_COUNT, PIPE_DONE, PIPE_NRDY, PIPE_WAKERS, PIPE_XFER, PipeConfig, XferType,
    pipe_bemp_disable, pipe_bemp_enable, pipe_brdy_disable, pipe_brdy_enable, pipe_configure,
    pipe_disable, pipe_enable, pipe_reset, pipe_stall, pipe_xfer_in_bemp, pipe_xfer_in_start,
    pipe_xfer_out_brdy,
};
use super::regs::{
    BUSWAIT_VALUE, DCPMAXP_MXPS_MASK, DVSQ_DEFAULT, DVSQ_SUSP0, DVSTCTR0_RHST, DVSTCTR0_RHST_FS,
    DVSTCTR0_RHST_HS, FIFOCTR_BCLR, INTENB0_BEMPE, INTENB0_BRDYE, INTENB0_CTRE, INTENB0_DVSE,
    INTENB0_VBSE, INTSTS0_BEMP, INTSTS0_BRDY, INTSTS0_CTRT, INTSTS0_CTSQ_MASK, INTSTS0_DVSQ_MASK,
    INTSTS0_DVSQ_SHIFT, INTSTS0_DVST, INTSTS0_VALID, INTSTS0_VBINT, INTSTS0_VBSTS, PIPECTR_ACLRM,
    PIPECTR_CCPL, PIPECTR_PID_BUF, PIPECTR_PID_MASK, PIPECTR_PID_STALL, PIPECTR_SQCLR,
    PKT_BUF_BLOCK_SIZE, Rusb1Regs, SUSPMODE_SUSPM, SYSCFG_DCFM, SYSCFG_DPRPU, SYSCFG_DRPD,
    SYSCFG_HSE, SYSCFG_UPLLE, SYSCFG_USBE, pipectr_ptr, rd, rmw, wr,
};

// ---------------------------------------------------------------------------
// Global ISR-facing state
// ---------------------------------------------------------------------------

/// Bus-level events queued by the ISR and consumed by [`Rusb1Bus::poll`].
/// Bit layout (per port, index 0 = port 0):
///
/// - bit 0: VBUS rose
/// - bit 1: VBUS fell
/// - bit 2: USB bus reset
/// - bit 3: suspend
/// - bit 4: resume
static BUS_EVENTS: [AtomicU16; 2] = [AtomicU16::new(0), AtomicU16::new(0)];

/// Waker for [`Rusb1Bus::poll`] per port.
static BUS_WAKERS: [AtomicWaker; 2] = [AtomicWaker::new(), AtomicWaker::new()];

const BUS_EVT_VBUS_ON: u16 = 1 << 0;
const BUS_EVT_VBUS_OFF: u16 = 1 << 1;
const BUS_EVT_RESET: u16 = 1 << 2;
const BUS_EVT_SUSPEND: u16 = 1 << 3;
const BUS_EVT_RESUME: u16 = 1 << 4;

/// Pending CTSQ value delivered by the last CTRT interrupt (per port).
static CTRL_STAGE: [AtomicU8; 2] = [AtomicU8::new(0xFF), AtomicU8::new(0xFF)];

/// Waker for [`Rusb1ControlPipe::setup`] and [`Rusb1ControlPipe::data_out`] per port.
static CTRL_WAKERS: [AtomicWaker; 2] = [AtomicWaker::new(), AtomicWaker::new()];

/// Set to 1 by the ISR when a BRDY fires on pipe 0 (control write data arrived).
/// Consumed by [`Rusb1ControlPipe::data_out`].
static PIPE0_BRDY: [AtomicU8; 2] = [AtomicU8::new(0), AtomicU8::new(0)];

/// Setup packet bytes captured from USBREQ/USBVAL/USBINDX/USBLENG per port.
static SETUP_PKT: [critical_section::Mutex<core::cell::UnsafeCell<[u8; 8]>>; 2] = {
    use core::cell::UnsafeCell;
    use critical_section::Mutex;
    #[allow(clippy::declare_interior_mutable_const)]
    const EMPTY: Mutex<UnsafeCell<[u8; 8]>> = Mutex::new(UnsafeCell::new([0u8; 8]));
    [EMPTY; 2]
};

// Sentinel meaning "no pending CTSQ".
const CTRL_STAGE_NONE: u8 = 0xFF;

// ---------------------------------------------------------------------------
// Per-port allocation bookkeeping (guarded by critical section)
// ---------------------------------------------------------------------------

struct PortAlloc {
    /// Which pipes are allocated: bit N = pipe N used (pipe 0 = DCP always used).
    pipes_used: u16,
    /// Endpoint → pipe mapping.  Index = ep_num, value = pipe number (0 = unused).
    ep_to_pipe: [[u8; 16]; 2], // [0]=OUT map, [1]=IN map
    /// Packet buffer block allocator.
    buf: BufAllocator,
    /// Pipe configuration (for re-configure on reset).
    pipe_cfg: [Option<PipeConfig>; PIPE_COUNT],
}

impl PortAlloc {
    const fn new() -> Self {
        Self {
            pipes_used: 0x0001, // pipe 0 pre-allocated
            ep_to_pipe: [[0u8; 16]; 2],
            buf: BufAllocator::new(),
            pipe_cfg: [None; PIPE_COUNT],
        }
    }

    fn alloc_pipe(&mut self, xfer_type: XferType, _is_in: bool) -> Option<usize> {
        let (start, end) = match xfer_type {
            XferType::Control => return Some(0),
            XferType::Isochronous => (1, 3), // pipes 1-2
            XferType::Bulk => (3, 10),       // pipes 3-9
            XferType::Interrupt => (6, 15),  // pipes 6-14 (avoid ISO range)
        };
        for n in start..end {
            if self.pipes_used & (1 << n) == 0 {
                self.pipes_used |= 1 << n;
                return Some(n);
            }
        }
        None
    }
}

static PORT_ALLOC: [critical_section::Mutex<core::cell::UnsafeCell<PortAlloc>>; 2] = {
    use core::cell::UnsafeCell;
    use critical_section::Mutex;
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT: Mutex<UnsafeCell<PortAlloc>> = Mutex::new(UnsafeCell::new(PortAlloc::new()));
    [INIT; 2]
};

// ---------------------------------------------------------------------------
// Rusb1Driver
// ---------------------------------------------------------------------------

/// The top-level USB device-mode driver for one RUSB1 port (0 or 1).
///
/// ## Safety invariant
/// Only one `Rusb1Driver` may exist per port at a time.  Enforced by the
/// `into_device_mode()` function in `mod.rs`.
pub struct Rusb1Driver {
    port: u8,
}

impl Rusb1Driver {
    /// Create a driver for `port` (0 or 1).
    ///
    /// # Safety
    /// Caller must ensure this is the only active driver for this port, and
    /// that `rza1l_hal::rusb1::module_clock_enable(port)` has already been called.
    pub unsafe fn new(port: u8) -> Self {
        unsafe {
            debug_assert!(port <= 1);
            // Set BUSWAIT early so register accesses after USBE=1 are safe.
            let regs = Rusb1Regs::ptr(port);
            wr(core::ptr::addr_of_mut!((*regs).buswait), BUSWAIT_VALUE);
            Self { port }
        }
    }
}

impl<'d> Driver<'d> for Rusb1Driver {
    type EndpointOut = Rusb1EndpointOut;
    type EndpointIn = Rusb1EndpointIn;
    type ControlPipe = Rusb1ControlPipe;
    type Bus = Rusb1Bus;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        _interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        let xfer_type = endpoint_type_to_xfer(ep_type);
        critical_section::with(|cs| {
            let alloc = unsafe { &mut *PORT_ALLOC[self.port as usize].borrow(cs).get() };

            // If a specific address is requested and it's already allocated to a pipe,
            // reuse that pipe for this alternate setting (same endpoint, different MPS).
            // This matches the UAC2 pattern of sharing an endpoint across alt settings
            // (e.g. alt1=16-bit, alt2=24-bit on the same ISO OUT endpoint).
            if let Some(addr) = ep_addr {
                let ep_num = addr.index() as u8;
                let existing_pipe = alloc.ep_to_pipe[0][ep_num as usize] as usize;
                if existing_pipe != 0 {
                    if let Some(ref mut cfg) = alloc.pipe_cfg[existing_pipe] {
                        // Keep the larger MPS so the hardware buffer covers both alt settings.
                        if max_packet_size > cfg.mps {
                            let new_blocks = mps_to_blocks(max_packet_size);
                            cfg.mps = max_packet_size;
                            cfg.buf_blocks = new_blocks as u8;
                        }
                    }
                    let info = EndpointInfo {
                        addr: EndpointAddress::from_parts(ep_num as usize, Direction::Out),
                        ep_type,
                        max_packet_size,
                        interval_ms: _interval_ms,
                    };
                    return Ok(Rusb1EndpointOut {
                        port: self.port,
                        pipe: existing_pipe as u8,
                        info,
                    });
                }
            }

            let pipe = alloc
                .alloc_pipe(xfer_type, false)
                .ok_or(EndpointAllocError)?;
            let buf_blocks = mps_to_blocks(max_packet_size);
            let buf_start = alloc.buf.alloc(buf_blocks).ok_or(EndpointAllocError)?;

            // Use requested ep address or pick a free one.
            let ep_num = if let Some(addr) = ep_addr {
                addr.index() as u8
            } else {
                (1u8..16)
                    .find(|&e| alloc.ep_to_pipe[0][e as usize] == 0)
                    .ok_or(EndpointAllocError)?
            };

            alloc.ep_to_pipe[0][ep_num as usize] = pipe as u8;
            alloc.pipe_cfg[pipe] = Some(PipeConfig {
                ep_num,
                is_in: false,
                xfer_type,
                mps: max_packet_size,
                buf_start,
                buf_blocks: buf_blocks as u8,
                double_buf: false,
            });

            let info = EndpointInfo {
                addr: EndpointAddress::from_parts(ep_num as usize, Direction::Out),
                ep_type,
                max_packet_size,
                interval_ms: _interval_ms,
            };
            Ok(Rusb1EndpointOut {
                port: self.port,
                pipe: pipe as u8,
                info,
            })
        })
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        _interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        let xfer_type = endpoint_type_to_xfer(ep_type);
        critical_section::with(|cs| {
            let alloc = unsafe { &mut *PORT_ALLOC[self.port as usize].borrow(cs).get() };

            // Reuse existing pipe if this ep_addr is already allocated (shared-endpoint alt setting).
            if let Some(addr) = ep_addr {
                let ep_num = addr.index() as u8;
                let existing_pipe = alloc.ep_to_pipe[1][ep_num as usize] as usize;
                if existing_pipe != 0 {
                    if let Some(ref mut cfg) = alloc.pipe_cfg[existing_pipe]
                        && max_packet_size > cfg.mps
                    {
                        let new_blocks = mps_to_blocks(max_packet_size);
                        cfg.mps = max_packet_size;
                        cfg.buf_blocks = new_blocks as u8;
                    }
                    let info = EndpointInfo {
                        addr: EndpointAddress::from_parts(ep_num as usize, Direction::In),
                        ep_type,
                        max_packet_size,
                        interval_ms: _interval_ms,
                    };
                    return Ok(Rusb1EndpointIn {
                        port: self.port,
                        pipe: existing_pipe as u8,
                        info,
                    });
                }
            }

            let pipe = alloc
                .alloc_pipe(xfer_type, true)
                .ok_or(EndpointAllocError)?;
            let buf_blocks = mps_to_blocks(max_packet_size);
            let buf_start = alloc.buf.alloc(buf_blocks).ok_or(EndpointAllocError)?;

            let ep_num = if let Some(addr) = ep_addr {
                addr.index() as u8
            } else {
                (1u8..16)
                    .find(|&e| alloc.ep_to_pipe[1][e as usize] == 0)
                    .ok_or(EndpointAllocError)?
            };

            alloc.ep_to_pipe[1][ep_num as usize] = pipe as u8;
            alloc.pipe_cfg[pipe] = Some(PipeConfig {
                ep_num,
                is_in: true,
                xfer_type,
                mps: max_packet_size,
                buf_start,
                buf_blocks: buf_blocks as u8,
                double_buf: false,
            });

            let info = EndpointInfo {
                addr: EndpointAddress::from_parts(ep_num as usize, Direction::In),
                ep_type,
                max_packet_size,
                interval_ms: _interval_ms,
            };
            Ok(Rusb1EndpointIn {
                port: self.port,
                pipe: pipe as u8,
                info,
            })
        })
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        log::debug!(
            "usb{}: start (ctrl mps={})",
            self.port,
            control_max_packet_size
        );
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);

            // ── PLL / clock init (TRM §28.4.1 (5)) ─────────────────────────
            // Stop clock supply to this port while touching the PLL.
            rmw(core::ptr::addr_of_mut!((*regs).suspmode), SUSPMODE_SUSPM, 0);
            log::trace!("usb{}: SUSPMODE cleared for PLL init", self.port);

            // UPLLE lives in USB0's SYSCFG0 regardless of which port we are
            // initialising (C reference: always writes to rusb0->SYSCFG0).
            let regs0 = Rusb1Regs::ptr(0);
            rmw(
                core::ptr::addr_of_mut!((*regs0).syscfg0),
                SYSCFG_UPLLE,
                SYSCFG_UPLLE,
            );

            // Wait >= 1 ms for PLL lock (at ~400 MHz ≈ 400 K spin iterations).
            for _ in 0u32..400_000 {
                core::hint::spin_loop();
            }

            // CPU-bus wait cycles and clock re-enable.
            wr(core::ptr::addr_of_mut!((*regs).buswait), BUSWAIT_VALUE);
            rmw(
                core::ptr::addr_of_mut!((*regs).suspmode),
                SUSPMODE_SUSPM,
                SUSPMODE_SUSPM,
            );

            // ── Mode / PHY config ────────────────────────────────────────────
            // HSE=1: module auto-negotiates HS or FS during the reset handshake.
            // Must be set while DPRPU=0 (TRM §28.3 SYSCFG.HSE).
            rmw(
                core::ptr::addr_of_mut!((*regs).syscfg0),
                SYSCFG_HSE,
                SYSCFG_HSE,
            );

            // DCFM=0 (function/device mode), DRPD=0 (no pull-downs in device mode).
            rmw(
                core::ptr::addr_of_mut!((*regs).syscfg0),
                SYSCFG_DCFM | SYSCFG_DRPD,
                0,
            );

            // Enable USB module.
            rmw(
                core::ptr::addr_of_mut!((*regs).syscfg0),
                SYSCFG_USBE,
                SYSCFG_USBE,
            );

            // D+ pull-up: notifies host of connection.
            rmw(
                core::ptr::addr_of_mut!((*regs).syscfg0),
                SYSCFG_DPRPU,
                SYSCFG_DPRPU,
            );
            log::debug!(
                "usb{}: SYSCFG0={:#06x} (DPRPU set, D+ pull-up active)",
                self.port,
                rd(core::ptr::addr_of!((*regs).syscfg0))
            );

            // DCPMAXP (control endpoint max packet size).
            wr(
                core::ptr::addr_of_mut!((*regs).dcpmaxp),
                control_max_packet_size & DCPMAXP_MXPS_MASK,
            );

            // Enable VBUS interrupt so Bus::poll() can detect connection.
            wr(core::ptr::addr_of_mut!((*regs).intenb0), INTENB0_VBSE);

            // Register the ISR with the GIC.
            rza1l_hal::rusb1::int_enable(self.port);

            // If VBUS is already asserted (cable was plugged in before init),
            // the rising-edge VBINT will never fire.  Synthesise the event now
            // so that Bus::poll() doesn't wait forever.
            let intsts = rd(core::ptr::addr_of!((*regs).intsts0));
            log::debug!(
                "usb{}: post-init INTSTS0={:#06x} VBSTS={}",
                self.port,
                intsts,
                intsts & INTSTS0_VBSTS != 0
            );
            if intsts & INTSTS0_VBSTS != 0 {
                log::debug!(
                    "usb{}: VBUS already present — synthesising PowerDetected",
                    self.port
                );
                BUS_EVENTS[self.port as usize].fetch_or(BUS_EVT_VBUS_ON, Ordering::Release);
                BUS_WAKERS[self.port as usize].wake();
            }
        }
        (
            Rusb1Bus {
                port: self.port,
                connected: false,
            },
            Rusb1ControlPipe {
                port: self.port,
                mps: control_max_packet_size,
            },
        )
    }
}

// ---------------------------------------------------------------------------
// Rusb1Bus
// ---------------------------------------------------------------------------

pub struct Rusb1Bus {
    port: u8,
    /// Whether we currently believe VBUS is present.
    connected: bool,
}

impl Bus for Rusb1Bus {
    async fn poll(&mut self) -> Event {
        core::future::poll_fn(|cx| {
            BUS_WAKERS[self.port as usize].register(cx.waker());
            let ev = BUS_EVENTS[self.port as usize].swap(0, Ordering::Acquire);
            if ev & BUS_EVT_RESET != 0 {
                log::debug!("usb{}: Bus::poll → Reset", self.port);
                return Poll::Ready(Event::Reset);
            }
            if ev & BUS_EVT_SUSPEND != 0 {
                log::debug!("usb{}: Bus::poll → Suspend", self.port);
                return Poll::Ready(Event::Suspend);
            }
            if ev & BUS_EVT_RESUME != 0 {
                log::debug!("usb{}: Bus::poll → Resume", self.port);
                return Poll::Ready(Event::Resume);
            }
            if ev & BUS_EVT_VBUS_ON != 0 {
                log::debug!("usb{}: Bus::poll → PowerDetected", self.port);
                self.connected = true;
                return Poll::Ready(Event::PowerDetected);
            }
            if ev & BUS_EVT_VBUS_OFF != 0 {
                log::debug!("usb{}: Bus::poll → PowerRemoved", self.port);
                self.connected = false;
                return Poll::Ready(Event::PowerRemoved);
            }
            Poll::Pending
        })
        .await
    }

    async fn enable(&mut self) {
        log::debug!(
            "usb{}: Bus::enable — waiting 30 ms for module settle",
            self.port
        );
        // RZA1L hardware quirk (confirmed in C firmware): INTENB0, BEMPENB, and
        // BRDYENB are NOT writable immediately after USBE=1 — the write is
        // silently dropped.  Wait for the module to settle before arming IRQs.
        // The C firmware waits ~25 ms via polling; we use 30 ms to be safe.
        embassy_time::Timer::after_millis(30).await;

        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let intenb = INTENB0_VBSE | INTENB0_DVSE | INTENB0_CTRE | INTENB0_BRDYE | INTENB0_BEMPE;

            // Write and readback-verify; retry up to 3× with 10 ms increments
            // if the register isn't ready yet (matches C firmware retry loop).
            for attempt in 0..3u32 {
                wr(core::ptr::addr_of_mut!((*regs).intenb0), intenb);
                // Pipe 0 BEMPENB bit must be set so data_in() receives BEMP.
                // Pipe 0 BRDYENB bit must be set for control OUT / setup data.
                wr(core::ptr::addr_of_mut!((*regs).bempenb), 0x0001);
                wr(core::ptr::addr_of_mut!((*regs).brdyenb), 0x0001);

                let rb = rd(core::ptr::addr_of!((*regs).intenb0));
                if rb == intenb {
                    log::debug!(
                        "usb{}: INTENB0={:#06x} verified (attempt {})",
                        self.port,
                        rb,
                        attempt
                    );
                    break;
                }
                log::warn!(
                    "usb{}: INTENB0 write failed attempt {} (wrote {:#06x}, read {:#06x}), retrying…",
                    self.port,
                    attempt + 1,
                    intenb,
                    rb
                );
                embassy_time::Timer::after_millis(10).await;
            }

            // Configure allocated pipes on the hardware.
            configure_all_pipes(self.port);
            log::debug!("usb{}: Bus::enable done", self.port);
        }
    }

    async fn disable(&mut self) {
        // TRM §28.3 (3): when disconnection from the USB host is recognised
        // (device mode), DPRPU and DCFM must be toggled in this exact sequence
        // before the bus can be reused (e.g. switching to host mode).
        //
        // Step 1: clear DPRPU — pull-up off; host sees SE0 (disconnect).
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_DPRPU, 0);
        }
        // Step 2: wait ≥ 1 µs.
        Timer::after_micros(1).await;
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Step 3: pulse DCFM to 1 (resets PHY data-line state machine).
            rmw(
                core::ptr::addr_of_mut!((*regs).syscfg0),
                SYSCFG_DCFM,
                SYSCFG_DCFM,
            );
        }
        // Step 4: wait ≥ 200 ns (timer minimum is 1 µs — safe to over-wait).
        Timer::after_micros(1).await;
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Step 5: clear DCFM back to function (device) mode.
            rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_DCFM, 0);
            // Disable all interrupts.
            wr(core::ptr::addr_of_mut!((*regs).intenb0), 0);
            wr(core::ptr::addr_of_mut!((*regs).brdyenb), 0);
            wr(core::ptr::addr_of_mut!((*regs).bempenb), 0);
        }
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let pipe = ep_addr_to_pipe(self.port, ep_addr);
        if let Some(p) = pipe {
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                if enabled {
                    pipe_enable(regs, p);
                    if ep_addr.direction() == Direction::In {
                        pipe_bemp_enable(regs, p);
                    } else {
                        pipe_brdy_enable(regs, p);
                    }
                } else {
                    pipe_disable(regs, p);
                    pipe_bemp_disable(regs, p);
                    pipe_brdy_disable(regs, p);
                    // Pulse ACLRM to clear the FIFO buffer contents and the
                    // double-buffer toggle state (TRM §28.4.4 Table 28.13).
                    // This prevents leftover data in the off-bank from causing
                    // a spurious BRDY when the endpoint is re-enabled.
                    // Requires PID=NAK (already set by pipe_disable above).
                    pipe_reset(regs, p);
                    // Signal any task blocked in read() that the endpoint is
                    // now disabled, so it returns Err(EndpointError::Disabled)
                    // rather than blocking forever.
                    PIPE_NRDY.fetch_or(1u16 << p, Ordering::Release);
                }
            }
            // Wake any task waiting in wait_enabled() or read() so it can observe
            // the new pipe state (PID changed to BUF/NAK, or PIPE_NRDY set).
            PIPE_WAKERS[p].wake();
        }
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let pipe = ep_addr_to_pipe(self.port, ep_addr);
        if let Some(p) = pipe {
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                if stalled {
                    pipe_stall(regs, p);
                } else {
                    pipe_reset(regs, p);
                    pipe_enable(regs, p);
                }
            }
        }
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let pipe = ep_addr_to_pipe(self.port, ep_addr);
        if let Some(p) = pipe {
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                let ctr = rd(pipectr_ptr(regs, p));
                (ctr & super::regs::PIPECTR_PID_MASK) == super::regs::PIPECTR_PID_STALL
            }
        } else {
            false
        }
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

// ---------------------------------------------------------------------------
// Rusb1ControlPipe
// ---------------------------------------------------------------------------

pub struct Rusb1ControlPipe {
    port: u8,
    mps: u16,
}

impl ControlPipe for Rusb1ControlPipe {
    fn max_packet_size(&self) -> usize {
        self.mps as usize
    }

    async fn setup(&mut self) -> [u8; 8] {
        log::trace!("usb{}: setup() waiting", self.port);
        let pkt = core::future::poll_fn(|cx| {
            CTRL_WAKERS[self.port as usize].register(cx.waker());
            let stage = CTRL_STAGE[self.port as usize].swap(CTRL_STAGE_NONE, Ordering::Acquire);
            // Accept any non-idle stage: CTSQ_READ_DATA (1), CTSQ_WRITE_DATA (3),
            // and CTSQ_WRITE_ZLP (5) all indicate a new SETUP packet was received.
            // (The ISR only sets CTRL_STAGE when INTSTS0.VALID=1.)
            if stage != CTRL_STAGE_NONE {
                let pkt = critical_section::with(|cs| unsafe {
                    *SETUP_PKT[self.port as usize].borrow(cs).get()
                });
                return Poll::Ready(pkt);
            }
            Poll::Pending
        })
        .await;
        log::debug!("usb{}: setup() → {:02x?}", self.port, pkt);
        pkt
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        _first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        let p = self.port as usize;
        log::trace!("usb{}: data_out buf_len={}", self.port, buf.len());

        // Set PID=BUF so hardware ACKs the host's OUT token and receives the
        // data-stage payload into the CFIFO.  PID was forced to NAK by hardware
        // when the preceding SETUP packet was received (TRM §28.3.30 PID[1:0]
        // function-controller note).  Without this, every host OUT token is
        // met with NAK and no data ever arrives (Bug #4).
        //
        // Mirror C `usb_pstd_ctrl_read()` preamble: re-enable BRDY0, clear any
        // stale BRDYSTS bit 0, and discard any previously latched PIPE0_BRDY
        // signal.  If these are not cleared, a zero-length OUT ACK from the host
        // to the preceding status stage (fired while BRDYENB was still set) will
        // cause the await below to return immediately with 0 bytes.
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Clear BRDYSTS bit 0 (write-0-to-clear, all other bits kept 1).
            wr(core::ptr::addr_of_mut!((*regs).brdysts), !(1u16));
            // Re-enable BRDY0 interrupt (may have been disabled by accept()).
            rmw(core::ptr::addr_of_mut!((*regs).brdyenb), 0x0001, 0x0001);
            // Discard any stale software flag set by a spurious BRDY0 above.
            PIPE0_BRDY[p].store(0, Ordering::Release);

            let ctr = rd(pipectr_ptr(regs, 0));
            wr(
                pipectr_ptr(regs, 0),
                (ctr & !PIPECTR_PID_MASK) | PIPECTR_PID_BUF,
            );
        }

        // Wait for the BRDY interrupt on pipe 0 signalling that the host's
        // OUT data packet has been received and is ready to read (Bug #5).
        core::future::poll_fn(|cx| {
            CTRL_WAKERS[p].register(cx.waker());
            if PIPE0_BRDY[p].swap(0, Ordering::Acquire) != 0 {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        // Read from CFIFO (pipe 0 OUT).
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let fifo = FifoPort::cfifo(regs);
            fifo_select_pipe(&fifo, 0, false);
            let vld = fifo_dtln(&fifo) as usize;
            let len = buf.len().min(vld);
            if len > 0 {
                hw_to_sw_fifo(&fifo, buf.as_mut_ptr(), len);
            }
            fifo_bclr(&fifo);
            log::trace!(
                "usb{}: data_out read {} bytes (fifo had {})",
                self.port,
                len,
                vld
            );
            Ok(len)
        }
    }

    async fn data_in(
        &mut self,
        data: &[u8],
        _first: bool,
        last: bool,
    ) -> Result<(), EndpointError> {
        log::trace!("usb{}: data_in len={} last={}", self.port, data.len(), last);
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);

            // Mirror C `usb_pstd_ctrl_read()` preamble: clear BEMPSTS bit 0,
            // re-enable BEMP0 interrupt, and discard any stale PIPE_DONE bit 0
            // that may have been latched by a previous control-write status-stage
            // BEMP (fired while BEMPENB was still set before accept() was called).
            // Without this, the poll_fn below resolves immediately on first call
            // and returns before the IN data has actually been sent.
            wr(core::ptr::addr_of_mut!((*regs).bempsts), !(1u16));
            rmw(core::ptr::addr_of_mut!((*regs).bempenb), 0x0001, 0x0001);
            PIPE_DONE.fetch_and(!(1u16), Ordering::Release);

            let fifo = FifoPort::cfifo(regs);
            fifo_select_pipe(&fifo, 0, true); // ISEL=1 selects IN direction

            // Wait for the ISEL bit to take effect (CFIFOSEL hardware latency).
            // Matches C `process_pipe0_xfer` which spins up to 1000 iterations.
            let mut ready = false;
            for _ in 0..1000u32 {
                let sel = rd(core::ptr::addr_of!((*regs).cfifosel));
                if sel & super::regs::FIFOSEL_ISEL != 0 {
                    ready = true;
                    break;
                }
            }
            log::trace!(
                "usb{}: CFIFO ISEL ready={} frdy={}",
                self.port,
                ready,
                fifo_is_ready(&fifo, 0)
            );
            if !ready || !fifo_is_ready(&fifo, 0) {
                log::warn!(
                    "usb{}: data_in CFIFO not ready — returning Disabled",
                    self.port
                );
                return Err(EndpointError::Disabled);
            }

            sw_to_hw_fifo(&fifo, data.as_ptr(), data.len());

            if last || data.len() < self.mps as usize {
                fifo_bval(&fifo);
            }

            // Arm DCP for IN transfers by setting PID=BUF (TRM §28.4.6.2(b)).
            // PID was left at NAK by hardware after the SETUP packet was received.
            // Without this the hardware NAKs every IN token from the host and
            // the data stage never completes (Bug #3).
            let ctr = rd(pipectr_ptr(regs, 0));
            wr(
                pipectr_ptr(regs, 0),
                (ctr & !PIPECTR_PID_MASK) | PIPECTR_PID_BUF,
            );

            // Wait for BEMP on pipe 0 (data transmitted to host).
            core::future::poll_fn(|cx| {
                PIPE_WAKERS[0].register(cx.waker());
                let done = PIPE_DONE.fetch_and(!(1u16), Ordering::Acquire);
                if done & 1 != 0 {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;

            // CCPL (status-stage ACK for control reads) is set by the ISR's
            // non-VALID CTRT ctsq=2 handler, matching the C driver's
            // usb_pstd_stand_req4() → usb_pstd_ctrl_end() path.
        }
        Ok(())
    }

    async fn accept(&mut self) {
        // Called by embassy-usb only for control OUT transfers (data from host).
        // The device sends a zero-length IN to acknowledge (status stage).
        // Mirrors C usb_pstd_ctrl_end(): disable BEMP0/BRDY0 BEFORE setting CCPL
        // so the status-ZLP BEMP/BRDY don't set stale flags for the next transfer.
        log::trace!("usb{}: accept (control-write status stage)", self.port);
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            rmw(core::ptr::addr_of_mut!((*regs).bempenb), 0x0001, 0x0000);
            rmw(core::ptr::addr_of_mut!((*regs).brdyenb), 0x0001, 0x0000);
            let ctr = rd(pipectr_ptr(regs, 0));
            wr(
                pipectr_ptr(regs, 0),
                (ctr & !PIPECTR_PID_MASK) | PIPECTR_PID_BUF | PIPECTR_CCPL,
            );
        }
    }

    async fn reject(&mut self) {
        log::trace!("usb{}: reject (STALL)", self.port);
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Set PID=STALL on DCP via RMW.  The previous code bare-wrote
            // PIPECTR_PID_STALL (0x0002) overwriting the entire DCPCTR register
            // and clobbering SQMON and other state bits (Bug #1).
            let ctr = rd(pipectr_ptr(regs, 0));
            wr(
                pipectr_ptr(regs, 0),
                (ctr & !PIPECTR_PID_MASK) | PIPECTR_PID_STALL,
            );
        }
    }

    async fn accept_set_address(&mut self, addr: u8) {
        log::debug!("usb{}: accept_set_address({})", self.port, addr);
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Hardware auto-updates USBADDR when CCPL is set.
            // Mirror C `usb_pstd_ctrl_end()`: disable pipe 0 BEMP and BRDY
            // interrupts BEFORE setting CCPL (see accept() for full rationale).
            rmw(core::ptr::addr_of_mut!((*regs).bempenb), 0x0001, 0x0000);
            rmw(core::ptr::addr_of_mut!((*regs).brdyenb), 0x0001, 0x0000);
            // Same as accept(): CCPL only completes the status stage when
            // PID=BUF (TRM §28.3.30 CCPL).  Must RMW to set both (Bug #2).
            let _ = addr;
            let ctr = rd(pipectr_ptr(regs, 0));
            wr(
                pipectr_ptr(regs, 0),
                (ctr & !PIPECTR_PID_MASK) | PIPECTR_PID_BUF | PIPECTR_CCPL,
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Rusb1EndpointOut
// ---------------------------------------------------------------------------

pub struct Rusb1EndpointOut {
    pub port: u8,
    pub pipe: u8,
    pub info: EndpointInfo,
}

impl embassy_usb_driver::Endpoint for Rusb1EndpointOut {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        core::future::poll_fn(|cx| {
            PIPE_WAKERS[self.pipe as usize].register(cx.waker());
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                let ctr = rd(pipectr_ptr(regs, self.pipe as usize));
                if (ctr & super::regs::PIPECTR_PID_MASK) == PIPECTR_PID_BUF {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            }
        })
        .await
    }
}

impl EndpointOut for Rusb1EndpointOut {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let pipe = self.pipe as usize;
        let mps = self.info.max_packet_size;

        // Store the transfer state so the ISR can fill the buffer.
        critical_section::with(|cs| {
            let state = unsafe { &mut *PIPE_XFER[pipe].borrow(cs).get() };
            state.buf = unsafe { core::ptr::NonNull::new_unchecked(buf.as_mut_ptr()) };
            state.length = buf.len() as u16;
            state.remaining = buf.len() as u16;
            state.transferred = 0;
            state.mps = mps;
            state.xfer_type = endpoint_type_to_xfer(self.info.ep_type);
        });

        let is_iso = self.info.ep_type == EndpointType::Isochronous;

        // Enable BRDY for this pipe → ISR will call pipe_xfer_out_brdy.
        //
        // For ISO OUT: BRDYENB must stay asserted continuously across packets.
        // Disabling and re-enabling it between packets wipes BRDYSTS, losing
        // any packet that arrived during the gap and permanently stalling the
        // double-buffer.  So for ISO we only arm BRDY the first time (when
        // BRDYENB is not yet set) and never disable it between reads.
        //
        // Additionally: for ISO, flush any stale packet that arrived before
        // this read() call armed the transfer state (remaining was 0 when the
        // BRDY ISR ran, so it wasn't consumed).  If we don't BCLR here the
        // double-buffer bank stays "owned" by the CPU and no further BRDYs fire.
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let already_enabled = is_iso && {
                let brdyenb = core::ptr::addr_of!((*regs).brdyenb);
                rd(brdyenb) & (1u16 << pipe) != 0
            };
            if !already_enabled {
                // First arm: flush any stale ISO packet before enabling BRDY.
                if is_iso {
                    use super::fifo::{fifo_bclr, fifo_for_pipe, fifo_select_pipe};
                    let fifo = fifo_for_pipe(regs, pipe);
                    fifo_select_pipe(&fifo, pipe, false);
                    fifo_bclr(&fifo);
                }
                pipe_brdy_enable(regs, pipe);
            } else if is_iso {
                // Subsequent re-arms: the ISR no longer BCLRs stale packets, so
                // if a packet arrived while remaining==0 it's still in the FIFO.
                // Trigger it directly by calling pipe_xfer_out_brdy now — this
                // fills the buf from the waiting FIFO data and signals PIPE_DONE,
                // so the poll_fn below completes immediately without waiting for
                // another BRDY interrupt.
                let fired = pipe_xfer_out_brdy(regs, pipe);
                if fired {
                    PIPE_DONE.fetch_or(1u16 << pipe, Ordering::Release);
                    PIPE_WAKERS[pipe].wake();
                }
            }
            pipe_enable(regs, pipe);
        }

        // Wait for ISR to complete the read.
        let nbytes = core::future::poll_fn(|cx| {
            PIPE_WAKERS[pipe].register(cx.waker());
            let done = PIPE_DONE.load(Ordering::Acquire);
            if done & (1u16 << pipe) != 0 {
                PIPE_DONE.fetch_and(!(1u16 << pipe), Ordering::Release);
                let state = critical_section::with(|cs| {
                    let s = unsafe { &*PIPE_XFER[pipe].borrow(cs).get() };
                    s.transferred
                });
                Poll::Ready(Ok(state as usize))
            } else if PIPE_NRDY.load(Ordering::Acquire) & (1u16 << pipe) != 0 {
                PIPE_NRDY.fetch_and(!(1u16 << pipe), Ordering::Release);
                Poll::Ready(Err(EndpointError::Disabled))
            } else {
                Poll::Pending
            }
        })
        .await;

        // For non-ISO: disable BRDY between reads so the ISR doesn't fire when
        // no transfer state is set up.  For ISO: leave BRDYENB set so we don't
        // lose packets that arrive before the next read() call re-arms the pipe.
        if !is_iso {
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                pipe_brdy_disable(regs, pipe);
            }
        }

        nbytes
    }
}

// ---------------------------------------------------------------------------
// Rusb1EndpointIn
// ---------------------------------------------------------------------------

pub struct Rusb1EndpointIn {
    pub port: u8,
    pub pipe: u8,
    pub info: EndpointInfo,
}

impl embassy_usb_driver::Endpoint for Rusb1EndpointIn {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        core::future::poll_fn(|cx| {
            PIPE_WAKERS[self.pipe as usize].register(cx.waker());
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                let ctr = rd(pipectr_ptr(regs, self.pipe as usize));
                if (ctr & super::regs::PIPECTR_PID_MASK) == PIPECTR_PID_BUF {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            }
        })
        .await
    }
}

impl EndpointIn for Rusb1EndpointIn {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        let pipe = self.pipe as usize;
        let mps = self.info.max_packet_size;

        critical_section::with(|cs| {
            let state = unsafe { &mut *PIPE_XFER[pipe].borrow(cs).get() };
            // Cast away const — we only read this pointer in the ISR.
            state.buf = unsafe { core::ptr::NonNull::new_unchecked(buf.as_ptr() as *mut u8) };
            state.length = buf.len() as u16;
            state.remaining = buf.len() as u16;
            state.mps = mps;
            state.xfer_type = endpoint_type_to_xfer(self.info.ep_type);
        });

        // Kick off the first packet; subsequent packets sent from BEMP ISR.
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            pipe_bemp_enable(regs, pipe);
            pipe_enable(regs, pipe);
            pipe_xfer_in_start(regs, pipe, mps);
        }

        // Wait for all packets to drain.
        core::future::poll_fn(|cx| {
            PIPE_WAKERS[pipe].register(cx.waker());
            let done = PIPE_DONE.load(Ordering::Acquire);
            if done & (1u16 << pipe) != 0 {
                PIPE_DONE.fetch_and(!(1u16 << pipe), Ordering::Release);
                Poll::Ready(Ok(()))
            } else if PIPE_NRDY.load(Ordering::Acquire) & (1u16 << pipe) != 0 {
                PIPE_NRDY.fetch_and(!(1u16 << pipe), Ordering::Release);
                Poll::Ready(Err(EndpointError::Disabled))
            } else {
                Poll::Pending
            }
        })
        .await?;

        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            pipe_bemp_disable(regs, pipe);
        }

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------

/// USB device-mode interrupt handler.
///
/// Must be registered with the GIC for IRQ 73 (port 0) or 74 (port 1) before
/// `Rusb1Bus::enable()` enables the per-pipe interrupt bits.
///
/// # Safety
/// Must only be called from an interrupt context matching the port's GIC IRQ.
/// Single-core bare-metal only.
pub unsafe fn dcd_int_handler(port: u8) {
    unsafe {
        let regs = Rusb1Regs::ptr(port);
        let p = port as usize;

        let sts = rd(core::ptr::addr_of!((*regs).intsts0));

        // Clear all active sticky flags atomically in a single write, preserving
        // the VALID bit (matches C ISR: don't write 0 to bits we didn't observe,
        // as hardware may have set new bits since the read).
        //
        // RC-W0 semantics: writing 0 to a bit clears it; writing 1 leaves it set.
        // So: ~(observed_flags) | VALID  →  clears observed bits, keeps VALID=1.
        const STICKY_FLAGS: u16 = INTSTS0_CTRT | INTSTS0_DVST | INTSTS0_VBINT;
        wr(
            core::ptr::addr_of_mut!((*regs).intsts0),
            !(STICKY_FLAGS & sts) | INTSTS0_VALID,
        );

        // ── VBINT (VBUS change) ──────────────────────────────────────────────
        if sts & INTSTS0_VBINT != 0 {
            if sts & INTSTS0_VBSTS != 0 {
                BUS_EVENTS[p].fetch_or(BUS_EVT_VBUS_ON, Ordering::Release);
            } else {
                BUS_EVENTS[p].fetch_or(BUS_EVT_VBUS_OFF, Ordering::Release);
            }
            BUS_WAKERS[p].wake();
        }

        // ── DVST (device-state change) ───────────────────────────────────────
        if sts & INTSTS0_DVST != 0 {
            let dvsq = (sts & INTSTS0_DVSQ_MASK) >> INTSTS0_DVSQ_SHIFT;
            // DVSQ=2 (Address) and DVSQ=3 (Configured): hardware auto-handles
            // SET_ADDRESS and SET_CONFIGURATION state transitions.  The C reference
            // driver does nothing for these states (USB_DS_ADDS / USB_DS_CNFG both
            // hit `break` in `usb_pstd_interrupt_handler`).  Emitting Resume here
            // disrupts embassy-usb's enumeration state machine.
            let evt = match dvsq {
                d if d == DVSQ_DEFAULT => Some(BUS_EVT_RESET),
                d if d >= DVSQ_SUSP0 => Some(BUS_EVT_SUSPEND),
                _ => None,
            };
            if let Some(evt) = evt {
                BUS_EVENTS[p].fetch_or(evt, Ordering::Release);
                BUS_WAKERS[p].wake();
            }

            // On reset, run the proper bus-reset sequence then reconfigure pipes.
            if dvsq == DVSQ_DEFAULT {
                process_bus_reset(port);
            }
        }

        // ── CTRT (control transfer stage) ────────────────────────────────────
        if sts & INTSTS0_CTRT != 0 {
            // Re-read INTSTS0 to get the live VALID bit (matches C
            // `process_setup_packet` which re-reads before checking VALID).
            let intsts_live = rd(core::ptr::addr_of!((*regs).intsts0));
            if intsts_live & INTSTS0_VALID != 0 {
                // Clear CFIFO before latching the setup packet — any data left
                // over from the previous transfer must be discarded (matches C
                // `process_setup_packet`: rusb->CFIFOCTR = USB_CFIFOCTR_BCLR).
                wr(core::ptr::addr_of_mut!((*regs).cfifoctr), FIFOCTR_BCLR);

                // Capture setup packet from USBREQ/USBVAL/USBINDX/USBLENG.
                let req = rd(core::ptr::addr_of!((*regs).usbreq));
                let val = rd(core::ptr::addr_of!((*regs).usbval));
                let idx = rd(core::ptr::addr_of!((*regs).usbindx));
                let len = rd(core::ptr::addr_of!((*regs).usbleng));
                let pkt = [
                    (req & 0xFF) as u8,
                    (req >> 8) as u8,
                    (val & 0xFF) as u8,
                    (val >> 8) as u8,
                    (idx & 0xFF) as u8,
                    (idx >> 8) as u8,
                    (len & 0xFF) as u8,
                    (len >> 8) as u8,
                ];
                let cs = critical_section::CriticalSection::new();
                *SETUP_PKT[p].borrow(cs).get() = pkt;

                // Clear VALID now that the setup packet has been latched (matches
                // C `process_setup_packet`: rusb->INTSTS0 = ~USB_INTSTS0_VALID).
                wr(core::ptr::addr_of_mut!((*regs).intsts0), !INTSTS0_VALID);

                // Signal setup() only when VALID=1 (i.e. a real SETUP packet).
                // Status-stage CTRT events (VALID=0) must NOT wake setup() or
                // it would return a stale packet.
                let ctsq = (sts & INTSTS0_CTSQ_MASK) as u8;
                CTRL_STAGE[p].store(ctsq, Ordering::Release);
                CTRL_WAKERS[p].wake();
            } else {
                // Non-VALID CTRT: a status-stage transition is completing.
                let ctsq = (sts & INTSTS0_CTSQ_MASK) as u8;
                if ctsq == 2 {
                    // Control read status stage (CS_RDSS, ctsq=2): host sends its
                    // zero-length OUT to ACK our IN data.  We must set CCPL so the
                    // hardware ACKs that ZLP.  Mirrors C usb_pstd_stand_req4() ->
                    // usb_pstd_ctrl_end(): disable BEMP0/BRDY0 then PID=BUF|CCPL.
                    // embassy-usb does NOT call accept() for control-read transfers.
                    rmw(core::ptr::addr_of_mut!((*regs).bempenb), 0x0001, 0x0000);
                    rmw(core::ptr::addr_of_mut!((*regs).brdyenb), 0x0001, 0x0000);
                    let ctr = rd(pipectr_ptr(regs, 0));
                    wr(
                        pipectr_ptr(regs, 0),
                        (ctr & !PIPECTR_PID_MASK) | PIPECTR_PID_BUF | PIPECTR_CCPL,
                    );
                }
                // ctsq=4 (WRSS) / ctsq=5 (WRND): accept() / accept_set_address()
                // are called by the embassy-usb task and set CCPL from there.
            }
        }

        // ── BRDY (buffer ready — data received on OUT pipe) ──────────────────
        if sts & INTSTS0_BRDY != 0 {
            let brdysts = rd(core::ptr::addr_of!((*regs).brdysts));
            let brdyenb = rd(core::ptr::addr_of!((*regs).brdyenb));
            let active = brdysts & brdyenb;

            #[allow(clippy::needless_range_loop)]
            for n in 0..16usize {
                if active & (1 << n) == 0 {
                    continue;
                }
                // Clear the BRDY flag for this pipe.
                wr(core::ptr::addr_of_mut!((*regs).brdysts), !((1u16) << n));

                if n == 0 {
                    // Pipe 0 BRDY: data arrived for the control write data stage.
                    // Signal data_out() which is waiting on PIPE0_BRDY.
                    PIPE0_BRDY[p].store(1, Ordering::Release);
                    CTRL_WAKERS[p].wake();
                } else {
                    let done = pipe_xfer_out_brdy(regs, n);
                    if done {
                        PIPE_DONE.fetch_or(1u16 << n, Ordering::Release);
                        PIPE_WAKERS[n].wake();
                    }
                }
            }
        }

        // ── BEMP (buffer empty — IN pipe sent its data) ───────────────────────
        if sts & INTSTS0_BEMP != 0 {
            let bempsts = rd(core::ptr::addr_of!((*regs).bempsts));
            let bempenb = rd(core::ptr::addr_of!((*regs).bempenb));
            let active = bempsts & bempenb;

            #[allow(clippy::needless_range_loop)]
            for n in 0..16usize {
                if active & (1 << n) == 0 {
                    continue;
                }
                wr(core::ptr::addr_of_mut!((*regs).bempsts), !((1u16) << n));

                if n == 0 {
                    // Pipe 0 BEMP: all data has been sent for the control-read
                    // data stage.  Disable BEMP0 interrupt now (mirrors C
                    // usb_pstd_bemp_pipe WRITESHRT: hw_usb_clear_bempenb(PIPE0)).
                    // CCPL will be set by the CTRT ctsq=2 ISR handler below.
                    rmw(core::ptr::addr_of_mut!((*regs).bempenb), 0x0001, 0x0000);
                    PIPE_DONE.fetch_or(1, Ordering::Release);
                    PIPE_WAKERS[0].wake();
                } else {
                    let done = pipe_xfer_in_bemp(regs, n);
                    if done {
                        PIPE_DONE.fetch_or(1u16 << n, Ordering::Release);
                        PIPE_WAKERS[n].wake();
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn endpoint_type_to_xfer(t: EndpointType) -> XferType {
    match t {
        EndpointType::Control => XferType::Control,
        EndpointType::Bulk => XferType::Bulk,
        EndpointType::Interrupt => XferType::Interrupt,
        EndpointType::Isochronous => XferType::Isochronous,
    }
}

/// Minimum 64-byte buffer blocks for a given MPS.
fn mps_to_blocks(mps: u16) -> usize {
    (mps as usize).div_ceil(PKT_BUF_BLOCK_SIZE)
}

/// Translate an `EndpointAddress` to a hardware pipe number for the given port.
fn ep_addr_to_pipe(port: u8, ep_addr: EndpointAddress) -> Option<usize> {
    let ep_num = ep_addr.index();
    let dir_idx = if ep_addr.direction() == Direction::In {
        1
    } else {
        0
    };
    critical_section::with(|cs| {
        let alloc = unsafe { &*PORT_ALLOC[port as usize].borrow(cs).get() };
        let p = alloc.ep_to_pipe[dir_idx][ep_num] as usize;
        if p == 0 && ep_num != 0 { None } else { Some(p) }
    })
}

/// Perform the hardware bus-reset sequence, matching C `process_bus_reset`.
///
/// - Disables all pipe BRDY/BEMP except DCP pipe 0 BRDY.
/// - Clears and deselects all FIFO ports.
/// - Issues ACLRM + SQCLR on every active pipe.
/// - Aborts any in-progress transfers (wakes waiters with NRDY error).
/// - Then reconfigures all pipes so they are ready for the re-enumeration.
unsafe fn process_bus_reset(port: u8) {
    unsafe {
        log::debug!("usb{}: bus reset — reconfiguring", port);
        let regs = Rusb1Regs::ptr(port);
        let p = port as usize;

        // Log the negotiated bus speed from DVSTCTR0.RHST.
        let rhst = rd(core::ptr::addr_of!((*regs).dvstctr0)) & DVSTCTR0_RHST;
        let speed_str = match rhst {
            r if r == DVSTCTR0_RHST_HS => "HS (480 Mbps)",
            r if r == DVSTCTR0_RHST_FS => "FS (12 Mbps)",
            _ => "LS / unknown",
        };
        log::info!(
            "usb{}: bus reset — negotiated speed: {} (RHST={:#03x})",
            port,
            speed_str,
            rhst
        );

        // Only keep pipe 0 BRDY/BEMP enabled — pipes 1-15 will be re-enabled when
        // dcd_edpt_open / iso_activate are called by the host stack.
        // BEMPENB bit 0 must be set so that data_in() receives BEMP on pipe 0
        // (matches C firmware: USB200.BRDYENB = 1; USB200.BEMPENB = 1).
        wr(core::ptr::addr_of_mut!((*regs).brdyenb), 0x0001);
        wr(core::ptr::addr_of_mut!((*regs).bempenb), 0x0001);

        // Clear the control FIFO.
        wr(core::ptr::addr_of_mut!((*regs).cfifoctr), FIFOCTR_BCLR);

        // Deselect D0FIFO and D1FIFO (CURPIPE = 0).
        wr(core::ptr::addr_of_mut!((*regs).d0fifosel), 0);
        wr(core::ptr::addr_of_mut!((*regs).d1fifosel), 0);

        // ACLRM + SQCLR every active pipe and abort any in-progress transfer.
        let mut abort_mask: u16 = 0;
        critical_section::with(|cs| {
            let alloc = &*PORT_ALLOC[p].borrow(cs).get();
            #[allow(clippy::needless_range_loop)]
            for n in 1..PIPE_COUNT {
                if alloc.pipe_cfg[n].is_some() {
                    let ctr = pipectr_ptr(regs, n);
                    // ACLRM clears the pipe FIFO and resets the toggle.
                    wr(ctr, PIPECTR_ACLRM | PIPECTR_SQCLR);
                    wr(ctr, 0);

                    // Signal transfer failure to any waiting task.
                    let state = &mut *PIPE_XFER[n].borrow(cs).get();
                    if state.remaining != 0 {
                        state.remaining = 0;
                        state.buf = core::ptr::NonNull::dangling();
                        abort_mask |= 1u16 << n;
                    }
                }
            }
        });

        // Wake aborted waiters (they will observe PIPE_NRDY and return an error).
        if abort_mask != 0 {
            PIPE_NRDY.fetch_or(abort_mask, Ordering::Release);
            #[allow(clippy::needless_range_loop)]
            for n in 1..16usize {
                if abort_mask & (1u16 << n) != 0 {
                    PIPE_WAKERS[n].wake();
                }
            }
        }

        // Re-apply all pipe hardware configs so the host can re-enumerate.
        configure_all_pipes(port);
    }
}

/// (Re-)configure all allocated pipes on the hardware.  Called on enable and
/// on bus reset.
unsafe fn configure_all_pipes(port: u8) {
    unsafe {
        let regs = Rusb1Regs::ptr(port);
        critical_section::with(|cs| {
            let alloc = &*PORT_ALLOC[port as usize].borrow(cs).get();
            for (n, cfg_opt) in alloc.pipe_cfg.iter().enumerate() {
                if n == 0 {
                    continue;
                } // DCP configured via DCPMAXP/DCPCFG
                if let Some(cfg) = cfg_opt {
                    pipe_configure(regs, n, cfg);
                }
            }
        });
    }
}
