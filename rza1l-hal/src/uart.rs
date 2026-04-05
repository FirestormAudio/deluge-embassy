//! Async SCIF UART driver for RZ/A1L.
//!
//! Supports all five SCIF channels available on RZ/A1L (SCIF0–SCIF4).
//! Base addresses (stride 0x800):
//!   SCIF0 @ 0xE800_7000 … SCIF4 @ 0xE800_9000
//!
//! GIC interrupt IDs (per RZ/A1L HW Manual §A.2, devdrv_intc.h):
//!   SCIF_n: BRI = 221+n*4, ERI = 222+n*4, RXI = 223+n*4, TXI = 224+n*4
//!
//! **This module is board-agnostic.** It does not know which ports are wired
//! to which SCIF channel. Configure GPIO pin-mux via [`crate::gpio::set_pin_mux`]
//! before (or after) calling [`init`].
//!
//! ## Usage
//! ```ignore
//! unsafe {
//!     // 1. Init the SCIF peripheral
//!     uart::init(0, 31250);
//!     // 2. Configure GPIO pins (board-specific)
//!     gpio::set_pin_mux(6, 15, 5); // TxD0 on Deluge
//!     gpio::set_pin_mux(6, 14, 5); // RxD0 on Deluge
//!     // 3. Register GIC IRQs for this channel
//!     uart::register_irqs_for(0);
//! }
//! // In an Embassy task:
//! uart::write_bytes(0, b"hello").await;
//! let b = uart::read_byte(0).await;
//! ```

use core::future::poll_fn;
use core::task::Poll;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::waitqueue::AtomicWaker;

use crate::gic;

// ---------------------------------------------------------------------------
// SCIF register offsets (see RZ/A1L HW Manual §24, struct st_scif layout)
// ---------------------------------------------------------------------------

const SCSMR: usize = 0x00; // u16  Serial Mode Register
const SCBRR: usize = 0x04; // u8   Bit Rate Register
const SCSCR: usize = 0x08; // u16  Serial Control Register
const FTDR: usize = 0x0C; // u8   FIFO Transmit Data Register (write-only)
const SCFSR: usize = 0x10; // u16  FIFO Status Register
const SCFRDR: usize = 0x14; // u8   FIFO Receive Data Register (read-only)
const SCFCR: usize = 0x18; // u16  FIFO Control Register
const SCFDR: usize = 0x1C; // u16  FIFO Data Count Register  [12:8]=TX  [4:0]=RX
const SCSPTR: usize = 0x20; // u16  Serial Port Register
const SCLSR: usize = 0x24; // u16  Line Status Register
const SCEMR: usize = 0x28; // u16  Serial Extended Mode Register

// SCSCR bit masks
const TE: u16 = 1 << 4; // Transmit Enable
const RE: u16 = 1 << 5; // Receive Enable
const RIE: u16 = 1 << 6; // Receive Interrupt Enable  (enables RXI + ERI)
const TIE: u16 = 1 << 7; // Transmit Interrupt Enable (enables TXI)

// SCFSR bit masks
const DR: u16 = 1 << 0; // Data Ready  (RX FIFO has data below trigger)
const RDF: u16 = 1 << 1; // Receive FIFO Data Full  (count ≥ trigger)
const TDFE: u16 = 1 << 5; // TX FIFO Data Empty  (count ≤ trigger)

// TX FIFO depth
const TX_FIFO_SIZE: usize = 16;

// Peripheral clock: XTAL(13.2256 MHz) × 5 = 66.128 MHz.
// The Deluge uses a 13.2256 MHz crystal, giving P0 = 66,128,000 Hz exactly.
// Used for SCBRR calculation with BGDM=1 (double-speed mode):
//   SCBRR = round(P_CLK / (16 × baud)) − 1
// BGDM=1 (SCEMR bit 7) doubles the effective clock to the BRG, so the
// oversampling ratio stays at 16× against the nominal SCBRR formula.
const P_CLK: u32 = 66_128_000;

/// Number of SCIF channels on RZ/A1L.
pub const NUM_CHANNELS: usize = 5;

// GIC base IDs for SCIF0; per RZ/A1L HW Manual §A.2:
//   SCIFn: BRI = 221+n*4, ERI = 222+n*4, RXI = 223+n*4, TXI = 224+n*4
const BRI_BASE: u16 = 221;
const ERI_BASE: u16 = 222;
const RXI_BASE: u16 = 223;
const TXI_BASE: u16 = 224;

// GIC interrupt priority for UART (lower number = higher priority).
const UART_IRQ_PRIORITY: u8 = 10;

// ---------------------------------------------------------------------------
// DMA RX constants and static storage
// ---------------------------------------------------------------------------

// UNCACHED_MIRROR_OFFSET is defined at crate root (crate::UNCACHED_MIRROR_OFFSET).
use crate::UNCACHED_MIRROR_OFFSET;

/// DMA trigger-mode bits for SCIF: DMA_AM_FOR_SCIF = 0b010 << 8.
const DMA_AM_FOR_SCIF: u32 = 0x0200;

/// Ring-buffer size for DMA RX.  Must be a power of two ≤ 256.
///
/// Matches `PIC_RX_BUFFER_SIZE` / `MIDI_RX_BUFFER_SIZE` from the original C
/// firmware (`cpu_specific.h`).
const DMA_RX_BUF_SIZE: usize = 64;

/// Cache-line-aligned ring buffer for one SCIF DMA RX channel.
#[repr(C, align(32))]
struct DmaRxBuf([u8; DMA_RX_BUF_SIZE]);

/// Cache-line-aligned 8-word DMA link descriptor.
#[repr(C, align(32))]
struct DmaRxDesc([u32; 8]);

// Two slots: index 0 = SCIF ch0 (MIDI), index 1 = SCIF ch1 (PIC).
// Only those two SCIF channels require DMA RX on the Deluge.
// SAFETY: accessed only in `init_dma_rx` (single-threaded init) and
//         `try_read_dma` (called from a single Embassy task per channel).
static mut DMA_RX_BUF: [DmaRxBuf; 2] = [
    DmaRxBuf([0; DMA_RX_BUF_SIZE]),
    DmaRxBuf([0; DMA_RX_BUF_SIZE]),
];
static mut DMA_RX_DESC: [DmaRxDesc; 2] = [DmaRxDesc([0; 8]), DmaRxDesc([0; 8])];

// Per-SCIF-channel DMA RX state.
static mut DMA_RX_DMACH: [u8; NUM_CHANNELS] = [0; NUM_CHANNELS];
static mut DMA_RX_READ: [u32; NUM_CHANNELS] = [0; NUM_CHANNELS]; // CPU read ptr (cached addr)
static mut DMA_RX_BASE: [u32; NUM_CHANNELS] = [0; NUM_CHANNELS]; // buf base (cached addr)
static mut DMA_RX_ACTIVE: [bool; NUM_CHANNELS] = [false; NUM_CHANNELS];

// ---------------------------------------------------------------------------
// DMA TX static storage
// ---------------------------------------------------------------------------

/// Staging buffer size for one DMA TX transfer.  Sized to fit the largest
/// single `write_bytes` call (PIC `set_column_pair_rgb` = 49 bytes), rounded
/// up to the next 32-byte cache-line boundary.
const DMA_TX_BUF_SIZE: usize = 64;

/// Cache-line-aligned staging buffer for one SCIF DMA TX channel.
#[repr(C, align(32))]
struct DmaTxBuf([u8; DMA_TX_BUF_SIZE]);

// One slot per SCIF channel.  SAFETY: each slot is written only from the
// single Embassy task that owns that channel.
static mut DMA_TX_BUF: [DmaTxBuf; NUM_CHANNELS] = [
    DmaTxBuf([0; DMA_TX_BUF_SIZE]),
    DmaTxBuf([0; DMA_TX_BUF_SIZE]),
    DmaTxBuf([0; DMA_TX_BUF_SIZE]),
    DmaTxBuf([0; DMA_TX_BUF_SIZE]),
    DmaTxBuf([0; DMA_TX_BUF_SIZE]),
];
static mut DMA_TX_DMACH: [u8; NUM_CHANNELS] = [0; NUM_CHANNELS];
static mut DMA_TX_ACTIVE: [bool; NUM_CHANNELS] = [false; NUM_CHANNELS];

/// Per-channel async mutex that serialises DMA TX transfers.
///
/// `write_bytes` holds this for the entire duration of the DMAC transfer so
/// that a second caller on the same channel cannot clobber `DMA_TX_BUF` or
/// re-arm the DMAC channel while a transfer is in progress.
static DMA_TX_LOCK: [Mutex<CriticalSectionRawMutex, ()>; NUM_CHANNELS] = [
    Mutex::new(()),
    Mutex::new(()),
    Mutex::new(()),
    Mutex::new(()),
    Mutex::new(()),
];

// ---------------------------------------------------------------------------
// Per-channel async state
// ---------------------------------------------------------------------------

struct UartState {
    rx_waker: AtomicWaker,
    tx_waker: AtomicWaker,
}

// Safety: AtomicWaker is Send + Sync.
unsafe impl Sync for UartState {}

static UART_STATE: [UartState; NUM_CHANNELS] = [
    UartState {
        rx_waker: AtomicWaker::new(),
        tx_waker: AtomicWaker::new(),
    },
    UartState {
        rx_waker: AtomicWaker::new(),
        tx_waker: AtomicWaker::new(),
    },
    UartState {
        rx_waker: AtomicWaker::new(),
        tx_waker: AtomicWaker::new(),
    },
    UartState {
        rx_waker: AtomicWaker::new(),
        tx_waker: AtomicWaker::new(),
    },
    UartState {
        rx_waker: AtomicWaker::new(),
        tx_waker: AtomicWaker::new(),
    },
];

// ---------------------------------------------------------------------------
// Register helpers
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn rr16(addr: usize) -> u16 {
    unsafe { core::ptr::read_volatile(addr as *const u16) }
}

#[inline(always)]
unsafe fn wr16(addr: usize, val: u16) {
    unsafe {
        core::ptr::write_volatile(addr as *mut u16, val);
    }
}

#[inline(always)]
unsafe fn rr8(addr: usize) -> u8 {
    unsafe { core::ptr::read_volatile(addr as *const u8) }
}

#[inline(always)]
unsafe fn wr8(addr: usize, val: u8) {
    unsafe {
        core::ptr::write_volatile(addr as *mut u8, val);
    }
}

/// Base address of SCIF channel `ch`.
#[inline(always)]
fn base(ch: usize) -> usize {
    0xE800_7000 + ch * 0x0800
}

// ---------------------------------------------------------------------------
// Baud rate calculation
// ---------------------------------------------------------------------------

/// Compute SCBRR value for the given baud rate.
///
/// With BGDM=1 (double-speed mode): SCBRR = round(P_CLK / (16 × baud)) − 1.
/// Integer rounding: (P_CLK + 8 × baud) / (16 × baud) − 1.
fn scbrr(baud: u32) -> u8 {
    (((P_CLK + 8 * baud) / (16 * baud)) - 1) as u8
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

/// Initialise SCIF channel `ch` (`0`–`4`) at `baud_rate` bps.
///
/// This function only programmes the SCIF peripheral registers. GPIO pin-mux
/// for the TX/RX pins must be configured separately via
/// [`crate::gpio::set_pin_mux`] using board-specific port/pin/mux values.
///
/// Must be called before [`register_irqs_for`] and with global IRQ disabled.
///
/// # Safety
/// Writes to memory-mapped SCIF registers. Caller must ensure `ch` < [`NUM_CHANNELS`].
pub unsafe fn init(ch: usize, baud_rate: u32) {
    unsafe {
        debug_assert!(ch < NUM_CHANNELS);
        log::debug!(
            "uart: ch{} init at {} bps (SCBRR={})",
            ch,
            baud_rate,
            scbrr(baud_rate)
        );
        let b = base(ch);

        // 1. Stop all SCIF operations.
        wr16(b + SCSCR, 0x0000);

        // 2. Reset TX and RX FIFOs (TFRST=bit2, RFRST=bit1).
        wr16(b + SCFCR, 0x0006);

        // 3. Clear status flags: mask out reserved/sticky bits, keep ER/BRK/DR clear.
        let scfsr = rr16(b + SCFSR);
        wr16(b + SCFSR, scfsr & 0xFF6E);

        // 4. Clear overrun error in SCLSR (ORER = bit 0).
        let sclsr = rr16(b + SCLSR);
        wr16(b + SCLSR, sclsr & !1u16);

        // 5. Internal clock source: CKE[1:0] = 00 (already 0 from step 1).

        // 6. Async 8N1, CKS=00 (P_CLK/1).
        wr16(b + SCSMR, 0x0000);

        // 7. Double-speed mode: BGDM = bit 7.
        wr16(b + SCEMR, 0x0080);

        // 8. Baud rate.
        wr8(b + SCBRR, scbrr(baud_rate));

        // 9. Release FIFO resets.  TX trigger=8 (TTRG=00), RX trigger=1 (RTRG=00).
        wr16(b + SCFCR, 0x0000);

        // 10. Break: keep TxD high.
        let scsptr = rr16(b + SCSPTR);
        wr16(b + SCSPTR, scsptr | 0x0003);

        // 11. Enable TX and RX; leave TIE/RIE off (enabled on demand by futures).
        wr16(b + SCSCR, TE | RE);
        log::debug!("uart: ch{} ready", ch);
    }
}

// ---------------------------------------------------------------------------
// GIC interrupt registration
// ---------------------------------------------------------------------------

/// Change the baud rate of an already-initialised SCIF channel without
/// resetting the FIFOs or GIC interrupt registration.
///
/// The SCIF must briefly stop TX/RX while SCBRR is updated.  Any bytes
/// currently being shifted out will be lost; the caller must ensure the TX
/// FIFO has drained (e.g. via `Timer::after_millis`) before calling this.
///
/// # Safety
/// Writes to memory-mapped SCIF registers. Caller must ensure `ch` < [`NUM_CHANNELS`].
pub unsafe fn set_baud(ch: usize, baud_rate: u32) {
    unsafe {
        debug_assert!(ch < NUM_CHANNELS);
        let b = base(ch);
        // Stop TE/RE so the baud rate change takes effect cleanly.
        wr16(b + SCSCR, 0x0000);
        wr8(b + SCBRR, scbrr(baud_rate));
        // Clear any accumulated error flags from the previous baud rate.  The TRM
        // states that ORER (overrun) is NOT cleared by toggling RE, so stale errors
        // from e.g. the baud-switch transition window must be cleared explicitly;
        // otherwise the SCIF silently discards all subsequent received bytes.
        let scfsr = rr16(b + SCFSR);
        wr16(b + SCFSR, scfsr & 0xFF6E); // clear ER/BRK/DR/RDF
        let sclsr = rr16(b + SCLSR);
        wr16(b + SCLSR, sclsr & !1u16); // clear ORER
        // Re-enable: DMA-RX channels need TIE|RIE set as DMA triggers; others just TE|RE.
        let scscr = if DMA_RX_ACTIVE[ch] {
            TIE | RIE | RE | TE
        } else {
            TE | RE
        };
        wr16(b + SCSCR, scscr);
    }
}

/// Register and enable the RXI, TXI, and ERI GIC interrupt sources for SCIF
/// channel `ch`.
///
/// ERI (Error Interrupt) must be registered to clear SCFSR error flags
/// (framing errors, overrun) that would otherwise permanently stall reception.
///
/// Must be called after [`init`] and before global IRQ is enabled.
///
/// # Safety
/// Writes to memory-mapped GIC registers via [`crate::gic`].
/// Caller must ensure `ch` < [`NUM_CHANNELS`].
pub unsafe fn register_irqs_for(ch: usize) {
    unsafe {
        debug_assert!(ch < NUM_CHANNELS);
        let eri = ERI_BASE + (ch as u16) * 4;
        let rxi = RXI_BASE + (ch as u16) * 4;
        let txi = TXI_BASE + (ch as u16) * 4;
        log::trace!(
            "uart: ch{} registering ERI={} RXI={} TXI={}",
            ch,
            eri,
            rxi,
            txi
        );
        gic::register(eri, ERI_HANDLERS[ch]);
        gic::register(rxi, RXI_HANDLERS[ch]);
        gic::register(txi, TXI_HANDLERS[ch]);
        gic::set_priority(eri, UART_IRQ_PRIORITY);
        gic::set_priority(rxi, UART_IRQ_PRIORITY);
        gic::set_priority(txi, UART_IRQ_PRIORITY);
        gic::enable(eri);
        gic::enable(rxi);
        gic::enable(txi);
    }
}

/// Register and enable **TXI** and **ERI** GIC interrupts for SCIF channel `ch`.
///
/// Used when the channel uses DMA for RX (so we must not enable RXI in the
/// GIC — RIE=1 in SCSCR sends the request to the DMAC, not the GIC).
/// ERI is still registered because error interrupts (framing errors, overrun)
/// fire via the GIC independently of the DMA path and must be cleared to
/// prevent the SCIF from silently discarding all subsequent received bytes.
///
/// # Safety
/// Writes to memory-mapped GIC registers. Caller must ensure `ch` < [`NUM_CHANNELS`].
pub unsafe fn register_txi_for(ch: usize) {
    unsafe {
        debug_assert!(ch < NUM_CHANNELS);
        let eri = ERI_BASE + (ch as u16) * 4;
        let txi = TXI_BASE + (ch as u16) * 4;
        log::trace!(
            "uart: ch{} registering ERI={} TXI={} (DMA RX, no RXI GIC)",
            ch,
            eri,
            txi
        );
        gic::register(eri, ERI_HANDLERS[ch]);
        gic::register(txi, TXI_HANDLERS[ch]);
        gic::set_priority(eri, UART_IRQ_PRIORITY);
        gic::set_priority(txi, UART_IRQ_PRIORITY);
        gic::enable(eri);
        gic::enable(txi);
    }
}

/// Set up circular DMA RX for SCIF channel `ch`.
///
/// - `dma_ch`: DMAC channel number (12 for PIC/SCIF1, 13 for MIDI/SCIF0).
/// - `dmars`:  DMARS resource-selector value (0x026 for SCIF1, 0x022 for SCIF0).
///
/// Builds a self-referential link descriptor so the DMAC loops through the
/// 64-byte ring buffer indefinitely.  Sets `SCSCR = TIE|RIE|TE|RE` to enable
/// DMA triggers; the GIC RXI interrupt is intentionally left disabled.
///
/// Call [`register_txi_for`] separately so TX still uses TXI interrupts.
///
/// Only supported for `ch` ∈ {0, 1} (MIDI and PIC on Deluge).
///
/// # Safety
/// Writes to SCIF and DMAC registers.  Must be called after [`init`] and
/// before global IRQ is enabled.
pub unsafe fn init_dma_rx(ch: usize, dma_ch: u8, dmars: u32) {
    unsafe {
        debug_assert!(ch < 2, "DMA RX only supported for SCIF ch0 and ch1");
        let b = base(ch);
        let slot = ch;

        // Cached base address of the ring buffer.
        let buf_base = core::ptr::addr_of!(DMA_RX_BUF[slot].0[0]) as u32;

        // CHCFG matches original C firmware link descriptor word[4]:
        //   0b10000001_00010000_00000000_01100000 | DMA_AM_FOR_SCIF | (dma_ch & 7)
        //                                          ^^ 0x0200        ^^ channel-group index
        let chcfg: u32 = 0x8110_0060 | DMA_AM_FOR_SCIF | ((dma_ch & 7) as u32);

        // The link descriptor must be readable by the DMAC without cache interference.
        // The DMAC accesses physical RAM directly (bypasses the Cortex-A9 L1/L2 cache).
        // We write through the uncached alias so the values land in physical SRAM
        // immediately — no cache flush needed.  The NXLA we hand to the DMAC is
        // also the uncached alias address, since that IS the physical bus address
        // the DMAC will fetch from.
        let desc_cached = core::ptr::addr_of_mut!(DMA_RX_DESC[slot]) as usize;
        let desc_uncached = (desc_cached + UNCACHED_MIRROR_OFFSET) as *mut u32;
        let desc_addr_uncached = desc_uncached as u32; // address the DMAC will use

        // Write each descriptor word via the uncached window.
        desc_uncached.add(0).write_volatile(0b1101); // Header
        desc_uncached.add(1).write_volatile((b + SCFRDR) as u32); // Source: SCIF SCFRDR
        desc_uncached.add(2).write_volatile(buf_base); // Destination: ring buffer
        desc_uncached.add(3).write_volatile(DMA_RX_BUF_SIZE as u32); // Transfer count
        desc_uncached.add(4).write_volatile(chcfg); // CHCFG
        desc_uncached.add(5).write_volatile(0); // Interval
        desc_uncached.add(6).write_volatile(0); // Extension
        desc_uncached.add(7).write_volatile(desc_addr_uncached); // NXLA: self (circular)

        // Save per-channel state.
        DMA_RX_DMACH[ch] = dma_ch;
        DMA_RX_READ[ch] = buf_base;
        DMA_RX_BASE[ch] = buf_base;
        DMA_RX_ACTIVE[ch] = true;

        // Start DMAC — pass the uncached descriptor address so the hardware can
        // always fetch it coherently.
        crate::dmac::init_with_link_descriptor(dma_ch, desc_addr_uncached as *const u32, dmars);
        crate::dmac::channel_start(dma_ch);

        // TIE|RIE|TE|RE = 0x00F0: enables DMA triggers on each TX/RX event.
        // Because GIC RXI is not enabled, the SCIF request goes only to the DMAC.
        wr16(b + SCSCR, TIE | RIE | RE | TE);

        log::debug!(
            "uart: ch{} DMA RX active (dma_ch={}, dmars={:#06x}, buf={:#010x})",
            ch,
            dma_ch,
            dmars,
            buf_base
        );
    }
}

/// Initialise DMA TX for SCIF channel `ch` using DMAC channel `dma_ch`.
///
/// After this call, [`write_bytes`] for this channel will use a one-shot
/// register-mode DMA transfer instead of CPU-driven FIFO fills with TXI
/// interrupts.  The DMAC paces itself to the UART baud rate via the SCIF
/// TDFE request signal (AM=2, cycle-steal level-sensitive), so the CPU
/// suspends exactly once per transfer and wakes via DMAINT.
///
/// # Parameters
/// - `ch`: SCIF channel index (0 for MIDI/SCIF0, 1 for PIC/SCIF1).
/// - `dma_ch`: DMAC channel number to use for TX.
///   - PIC/SCIF1 TX: channel 10 (`PIC_TX_DMA_CHANNEL` in C firmware).
///   - MIDI/SCIF0 TX: channel 11 (`MIDI_TX_DMA_CHANNEL` in C firmware).
/// - `dmars`: DMA resource selector for the SCIF TX request signal:
///   - SCIF0 TX: `0x61`  (`DMARS_FOR_SCIF0_TX`)
///   - SCIF1 TX: `0x65`  (`DMARS_FOR_SCIF0_TX + (1 << 2)`)
///
/// # Safety
/// Must be called once during hardware initialisation, after [`init`] and
/// before the Embassy executor starts.
pub unsafe fn init_dma_tx(ch: usize, dma_ch: u8, dmars: u32) {
    unsafe {
        let ftdr = base(ch) + FTDR;
        // CHCFG: mirrors C firmware's `DMA_SCIF_TX_CONFIG | DMA_AM_FOR_SCIF | (ch & 7)`.
        //   0x0020_0068  — 8-bit transfers, SAM=increment (src moves), DAM=fixed (dst=FTDR),
        //                  REQD=1 (destination-side request), LVL=1, AM=0 before OR
        //   | DMA_AM_FOR_SCIF (0x0000_0200)  — AM=2: cycle-steal level, SCIF pacing
        //   | (dma_ch & 7)                   — channel-group index in CHCFG[2:0]
        let chcfg = 0x0020_0068_u32 | DMA_AM_FOR_SCIF | (dma_ch as u32 & 7);
        crate::dmac::init_register_mode(dma_ch, chcfg, ftdr as u32, dmars);
        crate::dmac::register_completion_irq(dma_ch);
        DMA_TX_DMACH[ch] = dma_ch;
        DMA_TX_ACTIVE[ch] = true;
        // TIE in SCSCR controls both the TXI GIC interrupt AND the SCIF→DMAC
        // DREQ signal.  The on_txi ISR clears TIE to stop repeated TXI fire —
        // but that also kills the DREQ, stalling any active DMA mid-transfer.
        // Fix: disable TXI at the GIC distributor so on_txi never runs.
        // TIE remains set in SCSCR (written by init_dma_rx), so DREQ pulses
        // continue to reach the DMAC uninterrupted.
        let txi_id = TXI_BASE + (ch as u16) * 4;
        gic::disable(txi_id);
        log::info!(
            "uart: ch{} DMA TX active (dma_ch={}, dmars={:#06x}, ftdr={:#010x})",
            ch,
            dma_ch,
            dmars,
            ftdr,
        );
    }
}

// ---------------------------------------------------------------------------
// Interrupt handlers
// ---------------------------------------------------------------------------

/// Called from GIC dispatch when ERI fires for SCIF `ch`.
///
/// Clears error flags in SCFSR (ER, BRK) and SCLSR (ORER) so the SCIF can
/// resume forwarding received bytes.  Without this, a single framing error
/// permanently stalls RX because the SCIF holds RDF=0 until ER is cleared.
/// Wakes the RX task so it retries after recovery.
fn on_eri(ch: usize) {
    let b = base(ch);
    unsafe {
        // Clear sticky error bits.  The mask 0xFF6E preserves non-error status
        // bits while writing 0 to ER(bit3)/BRK(bit6)/DR(bit0) — matching the
        // pattern used in init() and set_baud().
        let scfsr = rr16(b + SCFSR);
        wr16(b + SCFSR, scfsr & 0xFF6E);
        // Clear overrun error in SCLSR (ORER = bit 0).
        let sclsr = rr16(b + SCLSR);
        wr16(b + SCLSR, sclsr & !1u16);
    }
    UART_STATE[ch].rx_waker.wake();
}

/// Called from GIC dispatch when RXI fires for SCIF `ch`.
/// Disables RIE (stops further RXI) and wakes any pending rx future.
fn on_rxi(ch: usize) {
    let b = base(ch);
    unsafe {
        let scscr = rr16(b + SCSCR);
        wr16(b + SCSCR, scscr & !RIE);
    }
    UART_STATE[ch].rx_waker.wake();
}

/// Called from GIC dispatch when TXI fires for SCIF `ch`.
/// Disables TIE (stops further TXI) and wakes any pending tx future.
fn on_txi(ch: usize) {
    let b = base(ch);
    unsafe {
        let scscr = rr16(b + SCSCR);
        wr16(b + SCSCR, scscr & !TIE);
    }
    UART_STATE[ch].tx_waker.wake();
}

fn eri0_handler() {
    on_eri(0);
}
fn eri1_handler() {
    on_eri(1);
}
fn eri2_handler() {
    on_eri(2);
}
fn eri3_handler() {
    on_eri(3);
}
fn eri4_handler() {
    on_eri(4);
}
fn rxi0_handler() {
    on_rxi(0);
}
fn txi0_handler() {
    on_txi(0);
}
fn rxi1_handler() {
    on_rxi(1);
}
fn txi1_handler() {
    on_txi(1);
}
fn rxi2_handler() {
    on_rxi(2);
}
fn txi2_handler() {
    on_txi(2);
}
fn rxi3_handler() {
    on_rxi(3);
}
fn txi3_handler() {
    on_txi(3);
}
fn rxi4_handler() {
    on_rxi(4);
}
fn txi4_handler() {
    on_txi(4);
}

type HandlerFn = fn();
static ERI_HANDLERS: [HandlerFn; NUM_CHANNELS] = [
    eri0_handler,
    eri1_handler,
    eri2_handler,
    eri3_handler,
    eri4_handler,
];
static RXI_HANDLERS: [HandlerFn; NUM_CHANNELS] = [
    rxi0_handler,
    rxi1_handler,
    rxi2_handler,
    rxi3_handler,
    rxi4_handler,
];
static TXI_HANDLERS: [HandlerFn; NUM_CHANNELS] = [
    txi0_handler,
    txi1_handler,
    txi2_handler,
    txi3_handler,
    txi4_handler,
];

// ---------------------------------------------------------------------------
// Async API
// ---------------------------------------------------------------------------

/// Read a single byte from SCIF channel `ch`, waiting asynchronously.
///
/// Dispatches to DMA-polled RX if the channel was set up with [`init_dma_rx`],
/// otherwise uses the interrupt-driven path.
pub async fn read_byte(ch: usize) -> u8 {
    if unsafe { DMA_RX_ACTIVE[ch] } {
        read_byte_dma(ch).await
    } else {
        log::warn!(
            "uart: ch{} DMA_RX_ACTIVE=false — falling back to IRQ RX (no RXI registered!)",
            ch
        );
        read_byte_irq(ch).await
    }
}

/// Interrupt-driven RX path (original implementation).
async fn read_byte_irq(ch: usize) -> u8 {
    let b = base(ch);
    poll_fn(|cx| {
        // Register waker BEFORE checking the FIFO so we cannot miss a wakeup
        // from an ISR that fires between the check and returning Pending.
        UART_STATE[ch].rx_waker.register(cx.waker());

        let scfsr = unsafe { rr16(b + SCFSR) };
        if scfsr & (RDF | DR) != 0 {
            // Data is available: read one byte and clear the status flags.
            let byte = unsafe { rr8(b + SCFRDR) };
            // Clear RDF/DR.  Hardware will re-assert RDF if the FIFO still
            // holds data at or above the trigger level after this write.
            unsafe { wr16(b + SCFSR, scfsr & !(RDF | DR)) };
            // Re-enable RIE now so the next byte (or any bytes already sitting
            // in the FIFO that cleared RDF) will wake the future on the next
            // poll.  Without this, RIE stays off after the ISR cleared it and
            // pick-up of subsequent bytes depends entirely on whether RDF
            // happened to be asserted at the moment poll_fn is called again —
            // which fails for the first byte of a new burst after a quiet period.
            unsafe {
                let scscr = rr16(b + SCSCR);
                wr16(b + SCSCR, scscr | RIE);
            }
            Poll::Ready(byte)
        } else {
            // No data yet: enable RIE so the ISR wakes us when data arrives.
            unsafe {
                let scscr = rr16(b + SCSCR);
                wr16(b + SCSCR, scscr | RIE);
            }
            // Re-check after enabling RIE to close the race where data arrived
            // between our initial FIFO check and the RIE write.
            let scfsr = unsafe { rr16(b + SCFSR) };
            if scfsr & (RDF | DR) != 0 {
                // Data arrived in the window — read it (ISR will fire but waker
                // is already registered; the next poll will see RIE enabled).
                unsafe {
                    let byte = rr8(b + SCFRDR);
                    wr16(b + SCFSR, scfsr & !(RDF | DR));
                    return Poll::Ready(byte);
                }
            }
            Poll::Pending
        }
    })
    .await
}

// ---------------------------------------------------------------------------
// DMA-polled RX path
// ---------------------------------------------------------------------------

/// DMA-polled read: polls `CRDA_n` vs the CPU read pointer; yields for 100 µs
/// when the ring buffer is empty.
async fn read_byte_dma(ch: usize) -> u8 {
    // Log the initial DMA state once per channel to aid diagnosis.
    static FIRST_CALL: [core::sync::atomic::AtomicBool; NUM_CHANNELS] = [
        core::sync::atomic::AtomicBool::new(true),
        core::sync::atomic::AtomicBool::new(true),
        core::sync::atomic::AtomicBool::new(true),
        core::sync::atomic::AtomicBool::new(true),
        core::sync::atomic::AtomicBool::new(true),
    ];
    if FIRST_CALL[ch].swap(false, core::sync::atomic::Ordering::Relaxed) {
        let dma_ch = unsafe { DMA_RX_DMACH[ch] };
        let crda = unsafe { crate::dmac::current_dst(dma_ch) };
        let read_ptr = unsafe { DMA_RX_READ[ch] };
        let buf_base = unsafe { DMA_RX_BASE[ch] };
        let b = base(ch);
        let scscr = unsafe { rr16(b + SCSCR) };
        log::info!(
            "uart: ch{} dma_rx first call: dma_ch={} crda={:#010x} read_ptr={:#010x} buf_base={:#010x} SCSCR={:#06x}",
            ch,
            dma_ch,
            crda,
            read_ptr,
            buf_base,
            scscr
        );
    }
    let mut spin: u32 = 0;
    loop {
        if let Some(b) = unsafe { try_read_dma(ch) } {
            return b;
        }
        spin = spin.wrapping_add(1);
        embassy_time::Timer::after_micros(100).await;
    }
}

/// Non-blocking attempt to read one byte from the DMA RX ring buffer.
///
/// Returns `None` if the buffer is currently empty (CRDA == read pointer).
/// Reads data via the **uncached mirror alias** so the CPU sees DMA-written
/// bytes without needing a cache invalidate.
///
/// # Safety
/// `DMA_RX_*` statics must have been initialised by `init_dma_rx` for this
/// channel, and this function must only be called from a single task per
/// channel.
unsafe fn try_read_dma(ch: usize) -> Option<u8> {
    unsafe {
        let dma_ch = DMA_RX_DMACH[ch];
        let buf_base = DMA_RX_BASE[ch];
        let read_ptr = DMA_RX_READ[ch];

        // CRDA_n: current DMA write position (cached address space).
        let write_ptr = crate::dmac::current_dst(dma_ch);

        if write_ptr == read_ptr {
            return None;
        }

        // Read via uncached alias so we see what the DMAC wrote to physical RAM
        // rather than a potentially stale cache line.
        let byte =
            core::ptr::read_volatile((read_ptr as usize + UNCACHED_MIRROR_OFFSET) as *const u8);

        // Advance read pointer with power-of-two wrap.
        let offset = (read_ptr - buf_base + 1) & (DMA_RX_BUF_SIZE as u32 - 1);
        DMA_RX_READ[ch] = buf_base + offset;

        Some(byte)
    }
}

/// Write `buf` to the transmit FIFO of SCIF channel `ch`.
///
/// If DMA TX was configured for this channel via [`init_dma_tx`], a one-shot
/// register-mode DMA transfer is used: the CPU copies `buf` into an uncached
/// staging buffer, kicks the DMAC, and suspends exactly once until the
/// DMAINT completion interrupt fires.  No TXI interrupts are generated.
///
/// Otherwise falls back to the CPU-driven FIFO path: suspends whenever the
/// TX FIFO fills, resuming on each TXI interrupt.
pub async fn write_bytes(ch: usize, buf: &[u8]) {
    // DMA TX fast path: one-shot DMAC transfer per chunk, zero TXI interrupts.
    // The DMAC paces itself to the UART baud rate via the SCIF TDFE request
    // (AM=2, level-sensitive), so the hardware never overflows the TX FIFO.
    if unsafe { DMA_TX_ACTIVE[ch] } {
        let dma_ch = unsafe { DMA_TX_DMACH[ch] };
        // Hold the per-channel lock for the entire multi-chunk transfer.
        // This prevents a second caller (e.g. oled_task vs rgb_task both using
        // PIC UART ch1) from clobbering DMA_TX_BUF or re-arming the DMAC
        // channel while we are yielded inside wait_transfer_complete.
        let _guard = DMA_TX_LOCK[ch].lock().await;
        let mut pos = 0;
        while pos < buf.len() {
            let chunk_len = (buf.len() - pos).min(DMA_TX_BUF_SIZE);
            unsafe {
                // Write through the uncached alias so the DMAC sees fresh bytes
                // without requiring an explicit cache flush.
                let cached_ptr = core::ptr::addr_of!(DMA_TX_BUF[ch].0[0]);
                let uncached_ptr = (cached_ptr as usize + UNCACHED_MIRROR_OFFSET) as *mut u8;
                core::ptr::copy_nonoverlapping(buf.as_ptr().add(pos), uncached_ptr, chunk_len);
                crate::dmac::start_transfer(dma_ch, uncached_ptr as u32, chunk_len as u32);
            }
            crate::dmac::wait_transfer_complete(dma_ch).await;
            pos += chunk_len;
        }
        return;
    }

    // Fallback: CPU-driven FIFO fill with TXI interrupts.
    let b = base(ch);
    let mut pos = 0;

    while pos < buf.len() {
        // How many bytes can we add right now?
        let in_fifo = unsafe { (rr16(b + SCFDR) >> 8) & 0x1F } as usize;
        let space = TX_FIFO_SIZE.saturating_sub(in_fifo);
        let to_write = (buf.len() - pos).min(space);

        for i in 0..to_write {
            unsafe { wr8(b + FTDR, buf[pos + i]) };
        }
        pos += to_write;

        if pos < buf.len() {
            // FIFO is full; wait for TDFE (count ≤ trigger) via TXI interrupt.
            wait_tdfe(ch).await;
        }
    }
}

/// Suspend until the TX FIFO has drained below the trigger level (TDFE=1).
async fn wait_tdfe(ch: usize) {
    let b = base(ch);
    poll_fn(|cx| {
        UART_STATE[ch].tx_waker.register(cx.waker());

        let scfsr = unsafe { rr16(b + SCFSR) };
        if scfsr & TDFE != 0 {
            Poll::Ready(())
        } else {
            // Enable TIE so the ISR fires when TDFE goes high.
            unsafe {
                let scscr = rr16(b + SCSCR);
                wr16(b + SCSCR, scscr | TIE);
            }
            // Double-check after enabling TIE to close the race.
            let scfsr = unsafe { rr16(b + SCFSR) };
            if scfsr & TDFE != 0 {
                unsafe {
                    let scscr = rr16(b + SCSCR);
                    wr16(b + SCSCR, scscr & !TIE);
                }
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    })
    .await
}

// ── embedded-io-async impls ───────────────────────────────────────────────────

use core::convert::Infallible;

/// A type-safe handle to a SCIF UART channel exposing `embedded-io-async`
/// `Read` and `Write` traits.
///
/// `CH` is the channel number (0–4).  The underlying free functions
/// (`read_byte`, `write_bytes`) must be available (i.e., [`init`] and
/// [`register_irqs_for`] already called for the channel).
pub struct ScifUart<const CH: usize>;

impl<const CH: usize> ScifUart<CH> {
    /// Create a handle.
    ///
    /// # Safety
    /// [`init`] and [`register_irqs_for`] must have been called for channel `CH`.
    #[inline]
    pub unsafe fn new() -> Self {
        ScifUart
    }
}

impl<const CH: usize> embedded_io_async::ErrorType for ScifUart<CH> {
    type Error = Infallible;
}

impl<const CH: usize> embedded_io_async::Read for ScifUart<CH> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Infallible> {
        if buf.is_empty() {
            return Ok(0);
        }
        // Read at least one byte, then drain whatever is already in the FIFO
        // without blocking, up to `buf.len()`.
        buf[0] = read_byte(CH).await;
        let mut n = 1usize;
        while n < buf.len() {
            let b = base(CH);
            let scfsr = unsafe { rr16(b + SCFSR) };
            if scfsr & (RDF | DR) == 0 {
                break; // No more data immediately available.
            }
            buf[n] = unsafe { rr8(b + SCFRDR) };
            // Clear RDF/DR after reading.
            let scfsr2 = unsafe { rr16(b + SCFSR) };
            unsafe { wr16(b + SCFSR, scfsr2 & !(RDF | DR)) };
            n += 1;
        }
        Ok(n)
    }
}

impl<const CH: usize> embedded_io_async::Write for ScifUart<CH> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Infallible> {
        write_bytes(CH, buf).await;
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Infallible> {
        wait_tdfe(CH).await;
        Ok(())
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::{NUM_CHANNELS, P_CLK, RXI_BASE, TXI_BASE, base, scbrr};

    // SCIF0 base = 0xE800_7000, stride 0x800 per channel.
    #[test]
    fn scif_base_addresses() {
        assert_eq!(base(0), 0xE800_7000);
        assert_eq!(base(1), 0xE800_7800);
        assert_eq!(base(2), 0xE800_8000);
        assert_eq!(base(3), 0xE800_8800);
        assert_eq!(base(4), 0xE800_9000);
    }

    #[test]
    fn num_channels_is_five() {
        assert_eq!(NUM_CHANNELS, 5);
    }

    /// MIDI baud rate (31250 bps).
    ///
    /// SCBRR = round(P_CLK / (16 × 31250)) − 1
    ///       = round(66128125 / 500000) − 1
    ///       = round(132.256) − 1 = 132 − 1 = 131
    #[test]
    fn scbrr_midi_31250() {
        assert_eq!(scbrr(31_250), 131);
    }

    /// 115200 bps (debug serial).
    ///
    /// SCBRR = round(66128125 / 1843200) − 1 = round(35.88) − 1 = 36 − 1 = 35
    #[test]
    fn scbrr_115200() {
        assert_eq!(scbrr(115_200), 35);
    }

    /// Spot-check that the resulting baud rate error is within ±2% for MIDI.
    #[test]
    fn scbrr_midi_accuracy() {
        let divisor = scbrr(31_250) as u32 + 1;
        let actual_baud = P_CLK / (16 * divisor);
        let error_ppm = (actual_baud as i64 - 31_250) * 1_000_000 / 31_250;
        assert!(
            error_ppm.abs() < 20_000,
            "MIDI baud error {error_ppm} ppm exceeds 2%"
        );
    }

    /// GIC interrupt IDs follow the pattern: RXI_n = RXI_BASE + n*4.
    /// RZ/A1L HW Manual §A.2: SCIF0 RXI=223, SCIF1 RXI=227, ...
    #[test]
    fn scif_irq_ids() {
        for ch in 0..NUM_CHANNELS as u16 {
            assert_eq!(RXI_BASE + ch * 4, 223 + ch * 4);
            assert_eq!(TXI_BASE + ch * 4, 224 + ch * 4);
        }
    }
}
