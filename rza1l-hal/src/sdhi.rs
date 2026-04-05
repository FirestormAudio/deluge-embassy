//! Renesas RZ/A1L SD Host Interface (SDHI) register-level driver.
//!
//! Two SDHI controllers are present on RZ/A1L:
//!   - SDHI0 at 0xE804_E000 (not used on Deluge)
//!   - SDHI1 at 0xE804_E800 (Deluge SD card — port 1)
//!
//! All registers are 16-bit with byte-granular offsets per TRM table 38.2.
//! The data FIFO (SD_BUF0) is at `base + 0x30`, accessed as 32-bit words.
//!
//! GIC interrupt IDs for SDHI1:
//!   305 = INTC_ID_SDHI1_3  (card-detect / status-change)
//!   306 = INTC_ID_SDHI1_0  (command / data complete)
//!   307 = INTC_ID_SDHI1_1  (SDIO interrupt — registered but masked)
//!
//! ## Async model
//!
//! Each port has one [`embassy_sync::waitqueue::AtomicWaker`].  The GIC
//! handler captures INFO1/INFO2 into atomics, then wakes the waker.
//! `send_cmd`, `read_blocks_sw`, and `write_blocks_sw` use `poll_fn` to
//! sleep until the expected interrupt bits arrive.
//!
//! ## Usage
//!
//! ```ignore
//! unsafe {
//!     sdhi::init(1);
//!     sdhi::register_irqs(1);
//! }
//! // In an async task — SD card protocol:
//! sdhi::send_cmd(1, 0 /*CMD0*/, 0).await?;   // Reset
//! let resp = sdhi::last_r1(1);
//! ```

use core::future::poll_fn;
use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::gic;

// ---------------------------------------------------------------------------
// Base addresses
// ---------------------------------------------------------------------------

const SDHI0_BASE: usize = 0xE804_E000;
const SDHI1_BASE: usize = 0xE804_E800;

// ---------------------------------------------------------------------------
// Register offsets — direct byte offsets from base address (TRM table 38.2).
// All registers are 16-bit except SD_BUF0 which is 32-bit.
// Note: the address map has a 2-byte gap at offset 0x002 (reserved within
// the CMD 32-bit slot), so offsets are NOT simply index*2 for all registers.
// ---------------------------------------------------------------------------

const OFF_CMD: usize = 0x00; // SD Command
const OFF_ARG0: usize = 0x04; // Argument low  [15:0]
const OFF_ARG1: usize = 0x06; // Argument high [31:16]
const OFF_STOP: usize = 0x08; // Data Stop / Automatic Stop
const OFF_SECCNT: usize = 0x0A; // Block Count
const OFF_RESP0: usize = 0x0C; // Response R[23:8]   (SD_RSP00)
const OFF_RESP1: usize = 0x0E; // Response R[39:24]  (SD_RSP01)
const OFF_RESP2: usize = 0x10; // Response R[55:40]  (SD_RSP02)
const OFF_RESP3: usize = 0x12; // Response R[71:56]  (SD_RSP03)
const OFF_RESP4: usize = 0x14; // Response R[87:72]  (SD_RSP04)
const OFF_RESP5: usize = 0x16; // Response R[103:88] (SD_RSP05)
const OFF_RESP6: usize = 0x18; // Response R[119:104](SD_RSP06)
const OFF_RESP7: usize = 0x1A; // Response R[127:120](SD_RSP07)
const OFF_INFO1: usize = 0x1C; // Status flag 1
const OFF_INFO2: usize = 0x1E; // Status flag 2
const OFF_INFO1_MASK: usize = 0x20; // Interrupt mask 1  (0 = enabled, 1 = masked)
const OFF_INFO2_MASK: usize = 0x22; // Interrupt mask 2  (0 = enabled, 1 = masked)
const OFF_CLK_CTRL: usize = 0x24; // Clock control
const OFF_SIZE: usize = 0x26; // Block size
const OFF_OPTION: usize = 0x28; // Access option (timing, bus width)
const OFF_ERR_STS1: usize = 0x2C; // CMD/CRC/END error status
const OFF_ERR_STS2: usize = 0x2E; // Timeout error status
const OFF_BUF0: usize = 0x30; // Data FIFO — 32-bit access
const OFF_SDIO_MODE: usize = 0x34; // SDIO mode
const OFF_SDIO_INFO1: usize = 0x36; // SDIO interrupt flag
const OFF_SDIO_INFO1_MASK: usize = 0x38; // SDIO interrupt mask
const OFF_CC_EXT_MODE: usize = 0xD8; // DMA mode enable
const OFF_SOFT_RST: usize = 0xE0; // Soft reset
const OFF_EXT_SWAP: usize = 0xF0; // Byte-swap control

// ---------------------------------------------------------------------------
// SD_INFO1 bit masks
// ---------------------------------------------------------------------------

/// Response end (CMD complete, response received).
pub const INFO1_RESP: u16 = 0x0001;
/// Data transfer end (all blocks transferred).
pub const INFO1_DATA_TRNS: u16 = 0x0004;
/// Card remove via DAT3 detect.
pub const INFO1_REM_DAT3: u16 = 0x0100;
/// Card insert  via DAT3 detect.
pub const INFO1_INS_DAT3: u16 = 0x0200;
/// Card remove  via CD pin.
pub const INFO1_REM_CD: u16 = 0x0008;
/// Card insert  via CD pin (latched event; cleared by writing 0).
pub const INFO1_INS_CD: u16 = 0x0010;
/// Card detect change (insert OR remove) — CD pin (latched event mask).
pub const INFO1_DET_CD: u16 = 0x0018;
/// Card currently present — SD_CD level (1 = SD_CD held low = card inserted).
/// Use this for steady-state presence queries; INFO1_INS_CD is only for edge events.
pub const INFO1_CD_LEVEL: u16 = 0x0020;

// ---------------------------------------------------------------------------
// SD_INFO2 bit masks
// ---------------------------------------------------------------------------

/// CMD error (command index mismatch in response).
pub const INFO2_ERR0: u16 = 0x0001;
/// CRC error (response CRC, read data CRC, or CRC status).
pub const INFO2_ERR1: u16 = 0x0002;
/// END error (end-bit missing in response, read data, or CRC status).
pub const INFO2_ERR2: u16 = 0x0004;
/// Data timeout (busy too long, R1b busy, write CRC, or read data not arriving).
pub const INFO2_ERR3: u16 = 0x0008;
/// SD_BUF illegal write access (write when not in data-write state or buffer full).
pub const INFO2_ERR4: u16 = 0x0010;
/// SD_BUF illegal read access (read when buffer empty or data has CRC/END error).
pub const INFO2_ERR5: u16 = 0x0020;
/// Response timeout (no response within 640 SD_CLK cycles).
pub const INFO2_ERR6: u16 = 0x0040;
/// Buffer Read Enable (FIFO has data to read).
pub const INFO2_BRE: u16 = 0x0100;
/// Buffer Write Enable (FIFO is ready to accept data).
pub const INFO2_BWE: u16 = 0x0200;
/// Command type register busy.
pub const INFO2_CBSY: u16 = 0x4000;
/// Illegal access (register out-of-sequence).
pub const INFO2_ILA: u16 = 0x8000;
/// Bus clock divider change complete.
pub const INFO2_SCLKDIVEN: u16 = 0x2000;

/// All error bits in INFO2.
pub const INFO2_ERR_ALL: u16 = 0x807F;

// ---------------------------------------------------------------------------
// CC_EXT_MODE bits
// ---------------------------------------------------------------------------
const CC_EXT_MODE_DMASDRW: u16 = 1 << 1; // Enable SDHI DMA read/write

// ---------------------------------------------------------------------------
// DMA resource selectors (DMARS) for SDHI (BSP: dma_if.h DMA_RS_SDHI_*)
// Encoding: (MID[6:0] << 2) | RID[1:0]
// ---------------------------------------------------------------------------

/// DMARS for SDHI0 transmit (memory → SD_BUF0). MID=0x30, RID=01.
const DMARS_SDHI0_TX: u32 = 0x00C1;
/// DMARS for SDHI0 receive  (SD_BUF0 → memory). MID=0x30, RID=02.
const DMARS_SDHI0_RX: u32 = 0x00C2;
/// DMARS for SDHI1 transmit (memory → SD_BUF0). MID=0x31, RID=01.
const DMARS_SDHI1_TX: u32 = 0x00C5;
/// DMARS for SDHI1 receive  (SD_BUF0 → memory). MID=0x31, RID=02.
const DMARS_SDHI1_RX: u32 = 0x00C6;

use crate::dmac::{
    CHCFG_AM_BURST, CHCFG_DAD, CHCFG_DDS_32BIT, CHCFG_HIEN, CHCFG_LVL, CHCFG_REQD, CHCFG_SAD,
    CHCFG_SDS_32BIT,
};

/// Compute CHCFG for an SDHI DMA TX channel (memory → SD_BUF0).
/// Source increments (SAD=0); destination is fixed SD_BUF0 (DAD=1, REQD=1).
/// Register mode (DMS=0), DEM=0 (completion interrupt enabled).
#[inline(always)]
const fn chcfg_sdhi_tx(dma_ch: u8) -> u32 {
    CHCFG_DAD
        | CHCFG_DDS_32BIT
        | CHCFG_SDS_32BIT
        | CHCFG_AM_BURST
        | CHCFG_HIEN
        | CHCFG_LVL
        | CHCFG_REQD
        | (dma_ch as u32 & 7)
}

/// Compute CHCFG for an SDHI DMA RX channel (SD_BUF0 → memory).
/// Source is fixed SD_BUF0 (SAD=1); destination increments (DAD=0, REQD=0).
/// Register mode (DMS=0), DEM=0 (completion interrupt enabled).
#[inline(always)]
const fn chcfg_sdhi_rx(dma_ch: u8) -> u32 {
    CHCFG_SAD
        | CHCFG_DDS_32BIT
        | CHCFG_SDS_32BIT
        | CHCFG_AM_BURST
        | CHCFG_HIEN
        | CHCFG_LVL
        | (dma_ch as u32 & 7)
}

// ---------------------------------------------------------------------------
// Clock divider constants (SD_CLK_CTRL bits[7:0], P1φ input ≈ 66.6 MHz)
//
// TRM table (one-hot encoding — only one bit may be set; 0x00 = P1/2):
//   0x80 = P1/512 ≈  130 kHz   (identification / card-detect)
//   0x40 = P1/256 ≈  260 kHz
//   0x20 = P1/128 ≈  521 kHz
//   0x10 = P1/64  ≈  1.0 MHz
//   0x08 = P1/32  ≈  2.1 MHz
//   0x04 = P1/16  ≈  4.2 MHz
//   0x02 = P1/8   ≈  8.3 MHz
//   0x01 = P1/4   ≈ 16.7 MHz   (fast mode after init)
//   0x00 = P1/2   ≈ 33.3 MHz
// ---------------------------------------------------------------------------
const CLK_DIV_512: u16 = 0x80; // P1/512 ≈ 130 kHz  (identification clock)
const CLK_DIV_4: u16 = 0x01; // P1/4   ≈ 16.7 MHz (fast mode)
const CLK_ENABLE: u16 = 1 << 8; // Bit 8 (SCLKEN) = clock output enable

// ---------------------------------------------------------------------------
// Module clock
// ---------------------------------------------------------------------------
const STBCR12: usize = 0xFCFE_0444;

// ---------------------------------------------------------------------------
// Per-port state
// ---------------------------------------------------------------------------

const NUM_PORTS: usize = 2;

struct SdhiState {
    waker: AtomicWaker,
    /// Accumulated INFO1 bits from ISR (ANDed with active mask at capture time).
    info1: AtomicU16,
    /// Accumulated INFO2 bits from ISR.
    info2: AtomicU16,
}

unsafe impl Sync for SdhiState {}

static STATE: [SdhiState; NUM_PORTS] = [
    SdhiState {
        waker: AtomicWaker::new(),
        info1: AtomicU16::new(0),
        info2: AtomicU16::new(0),
    },
    SdhiState {
        waker: AtomicWaker::new(),
        info1: AtomicU16::new(0),
        info2: AtomicU16::new(0),
    },
];

// ---------------------------------------------------------------------------
// GIC IRQ IDs
// ---------------------------------------------------------------------------

/// GIC IDs for SDHI0: status-change, operation, SDIO.
const SDHI0_IRQS: [u16; 3] = [302, 303, 304];
/// GIC IDs for SDHI1: status-change, operation, SDIO.
const SDHI1_IRQS: [u16; 3] = [305, 306, 307];

const SDHI_IRQ_PRIORITY: u8 = 10;

// ---------------------------------------------------------------------------
// STBCR12 clock-gate bit masks
// ---------------------------------------------------------------------------

/// STBCR12 bits 3+2: SDHI00+SDHI01 — clock-gate mask for SDHI port 0.
const STBCR12_SDHI0_MASK: u8 = 0x0C;
/// STBCR12 bits 1+0: SDHI10+SDHI11 — clock-gate mask for SDHI port 1.
const STBCR12_SDHI1_MASK: u8 = 0x03;

// ---------------------------------------------------------------------------
// Interrupt-mask initialisation values (sd_init.c SD_INFO*_MASK constants)
// ---------------------------------------------------------------------------

/// INFO1_MASK init value: mask all INFO1 interrupts except card-detect.
/// BSP: `SD_INFO1_MASK = 0x031D` (sd_init.c).
const INFO1_MASK_INIT: u16 = 0x031D;
/// INFO2_MASK init value: mask all INFO2 error/status interrupts.
/// BSP: `SD_INFO2_MASK = 0x8B7F` (sd_init.c).
const INFO2_MASK_INIT: u16 = 0x8B7F;
/// SDIO_INFO1_MASK init value: mask all SDIO status interrupts.
const SDIO_INFO1_MASK_INIT: u16 = 0xC007;

// ---------------------------------------------------------------------------
// Soft-reset sequence values (SOFT_RST register)
// ---------------------------------------------------------------------------

/// SOFT_RST value to assert reset (bit 1 set, bit 0 clear).
/// BSP: written first in the RZ/A1 two-step soft-reset sequence.
const SOFT_RST_ASSERT: u16 = 0x0006;
/// SOFT_RST value to deassert reset (bit 1 set, bit 0 set).
/// BSP: written second to release the core from reset while keeping CMD/DAT active.
const SOFT_RST_DEASSERT: u16 = 0x0007;

// ---------------------------------------------------------------------------
// Register helpers
// ---------------------------------------------------------------------------

#[inline]
fn port_base(port: u8) -> usize {
    match port {
        0 => SDHI0_BASE,
        _ => SDHI1_BASE,
    }
}

#[inline]
unsafe fn reg16(base: usize, off: usize) -> *mut u16 {
    (base + off) as *mut u16
}

#[inline]
unsafe fn reg32(base: usize, off: usize) -> *mut u32 {
    (base + off) as *mut u32
}

// ---------------------------------------------------------------------------
// Public API: hardware initialization
// ---------------------------------------------------------------------------

/// Initialize SDHI hardware for `port` (0 or 1).
///
/// Enables the module clock (STBCR12), releases soft-reset, configures
/// interrupt masks, and sets the SD timing option register.
///
/// Call once, with global IRQ disabled, before [`register_irqs`].
///
/// # Safety
/// Writes to memory-mapped peripheral registers.
pub unsafe fn init(port: u8, sd_option: u16) {
    unsafe {
        // ---- enable SDHI module clock (STBCR12) ----
        // STBCR12 layout (bit 7..0): [1][1][1][1][SDHI00][SDHI01][SDHI10][SDHI11]
        //   SDHI0 (port 0): bits 3 (SDHI00) + 2 (SDHI01) — mask 0x0C
        //   SDHI1 (port 1): bits 1 (SDHI10) + 0 (SDHI11) — mask 0x03
        // Use RMW to enable only the clocks for the requested port; preserve
        // the other port's stop-bits so unused clocks stay gated.
        let stbcr12 = STBCR12 as *mut u8;
        let bits: u8 = if port == 0 {
            STBCR12_SDHI0_MASK
        } else {
            STBCR12_SDHI1_MASK
        };
        stbcr12.write_volatile(stbcr12.read_volatile() & !bits);
        let _ = stbcr12.read_volatile(); // dummy read (required per RZ/A1 HW manual)

        let base = port_base(port);

        // ---- mask all meaningful interrupts (0 = enabled, 1 = masked) ----
        // Values match sd_init.c: SD_INFO1_MASK=0x031D, SD_INFO2_MASK=0x8B7F (RZA1)
        reg16(base, OFF_INFO1_MASK).write_volatile(INFO1_MASK_INIT);
        reg16(base, OFF_INFO2_MASK).write_volatile(INFO2_MASK_INIT);
        reg16(base, OFF_SDIO_INFO1_MASK).write_volatile(SDIO_INFO1_MASK_INIT);

        // ---- clear SDIO mode ----
        reg16(base, OFF_SDIO_MODE).write_volatile(0x0000);

        // ---- clear status registers ----
        // Clear RESP(bit0) and DATA_TRNS(bit2) only, per sd_init.c: info1 & ~0x0005
        let info1 = reg16(base, OFF_INFO1).read_volatile();
        reg16(base, OFF_INFO1).write_volatile(info1 & !(INFO1_RESP | INFO1_DATA_TRNS));
        reg16(base, OFF_INFO2).write_volatile(0x0000);
        reg16(base, OFF_SDIO_INFO1).write_volatile(0x0000);

        // ---- soft reset (RZ/A1 variant: SOFT_RST_ASSERT → SOFT_RST_DEASSERT) ----
        reg16(base, OFF_SOFT_RST).write_volatile(SOFT_RST_ASSERT);
        reg16(base, OFF_SOFT_RST).write_volatile(SOFT_RST_DEASSERT);

        // ---- SD_OPTION: NCycle = SDCLK×2^23, 4-bit bus ----
        // Pass a board-specific value via the sd_option parameter.
        reg16(base, OFF_OPTION).write_volatile(sd_option);

        // ---- EXT_SWAP: no byte-swap ----
        reg16(base, OFF_EXT_SWAP).write_volatile(0x0000);

        // ---- CC_EXT_MODE: software transfer mode (no DMA yet) ----
        reg16(base, OFF_CC_EXT_MODE).write_volatile(0x0000);

        // ---- block size = 512 bytes ----
        reg16(base, OFF_SIZE).write_volatile(512);

        // ---- start at identification clock (~130 kHz, P1/512) ----
        reg16(base, OFF_CLK_CTRL).write_volatile(CLK_DIV_512 | CLK_ENABLE);
    }
}

/// Switch the SD clock to high-speed mode (~16.7 MHz, P1/4).
///
/// Call after successful card initialization.
///
/// # Safety
/// Writes to memory-mapped SDHI register.
pub unsafe fn set_clock_fast(port: u8) {
    unsafe {
        let base = port_base(port);
        // Wait for clock divider change to settle (SCLKDIVEN = 1 means ready).
        let mut ok = false;
        for _ in 0..10_000u32 {
            if reg16(base, OFF_INFO2).read_volatile() & INFO2_SCLKDIVEN != 0 {
                ok = true;
                break;
            }
        }
        if !ok {
            log::warn!("sdhi{}: set_clock_fast: SCLKDIVEN pre-change timeout", port);
        }
        reg16(base, OFF_CLK_CTRL).write_volatile(CLK_DIV_4 | CLK_ENABLE);
        ok = false;
        for _ in 0..10_000u32 {
            if reg16(base, OFF_INFO2).read_volatile() & INFO2_SCLKDIVEN != 0 {
                ok = true;
                break;
            }
        }
        if !ok {
            log::warn!(
                "sdhi{}: set_clock_fast: SCLKDIVEN post-change timeout",
                port
            );
        }
    }
}

/// Register GIC interrupt handlers for `port`.
///
/// Must be called while global IRQ is disabled.
///
/// # Safety
/// Writes to GIC registers.
pub unsafe fn register_irqs(port: u8) {
    unsafe {
        let irqs = if port == 0 { SDHI0_IRQS } else { SDHI1_IRQS };
        let handlers: [gic::Handler; 3] = if port == 0 {
            [irq_sdhi0_cd, irq_sdhi0_op, irq_sdhi0_io]
        } else {
            [irq_sdhi1_cd, irq_sdhi1_op, irq_sdhi1_io]
        };
        for i in 0..3 {
            gic::register(irqs[i], handlers[i]);
            gic::set_priority(irqs[i], SDHI_IRQ_PRIORITY);
            gic::enable(irqs[i]);
        }
    }
}

// ---------------------------------------------------------------------------
// Interrupt handlers (called by GIC dispatch)
// ---------------------------------------------------------------------------

fn irq_sdhi0_cd() {
    unsafe {
        interrupt_handler(0);
    }
}
fn irq_sdhi0_op() {
    unsafe {
        interrupt_handler(0);
    }
}
fn irq_sdhi0_io() { /* SDIO not used; clear and ignore */
}

fn irq_sdhi1_cd() {
    unsafe {
        interrupt_handler(1);
    }
}
fn irq_sdhi1_op() {
    unsafe {
        interrupt_handler(1);
    }
}
fn irq_sdhi1_io() { /* SDIO not used; clear and ignore */
}

/// Core interrupt handler: capture INFO1/INFO2, accumulate, wake waiter.
///
/// # Safety
/// Reads and writes memory-mapped SDHI registers.
pub unsafe fn interrupt_handler(port: u8) {
    unsafe {
        let base = port_base(port);

        // Read status
        let info1 = reg16(base, OFF_INFO1).read_volatile();
        let info2 = reg16(base, OFF_INFO2).read_volatile();

        // Clear bits by writing 0 (write-0-to-clear per RZ/A1 HW manual).
        // Only clear the bits we just read to avoid losing a racing edge.
        reg16(base, OFF_INFO1).write_volatile(!info1);
        reg16(base, OFF_INFO2).write_volatile(!info2);

        // Also clear SDIO status to prevent spurious SDIO interrupts.
        let sdio = reg16(base, OFF_SDIO_INFO1).read_volatile();
        reg16(base, OFF_SDIO_INFO1).write_volatile(!sdio);

        let st = &STATE[port as usize];
        st.info1.fetch_or(info1, Ordering::Release);
        st.info2.fetch_or(info2, Ordering::Release);
        st.waker.wake();
    }
}

// ---------------------------------------------------------------------------
// Async waiting primitives
// ---------------------------------------------------------------------------

/// Take (atomically consume) accumulated INFO1/INFO2 bits.
#[allow(dead_code)]
fn take_info(port: u8) -> (u16, u16) {
    let st = &STATE[port as usize];
    let i1 = st.info1.swap(0, Ordering::Acquire);
    let i2 = st.info2.swap(0, Ordering::Acquire);
    (i1, i2)
}

/// Peek at INFO1/INFO2 without consuming.
#[allow(dead_code)]
fn peek_info(port: u8) -> (u16, u16) {
    let st = &STATE[port as usize];
    let i1 = st.info1.load(Ordering::Acquire);
    let i2 = st.info2.load(Ordering::Acquire);
    (i1, i2)
}

/// Clear accumulated state.
fn clear_info(port: u8) {
    let st = &STATE[port as usize];
    st.info1.store(0, Ordering::Release);
    st.info2.store(0, Ordering::Release);
}

// ---------------------------------------------------------------------------
// Command/argument helpers
// ---------------------------------------------------------------------------

/// Set the 32-bit command argument (split into two 16-bit registers).
///
/// # Safety
/// Writes to SDHI registers.
#[inline]
pub unsafe fn set_arg(port: u8, arg: u32) {
    unsafe {
        let base = port_base(port);
        reg16(base, OFF_ARG0).write_volatile((arg & 0xFFFF) as u16);
        reg16(base, OFF_ARG1).write_volatile((arg >> 16) as u16);
    }
}

/// Wait for the clock divider register to be ready (SCLKDIVEN bit).
///
/// # Safety
/// Reads SDHI INFO2 register.
pub unsafe fn wait_clk_stable(port: u8) {
    unsafe {
        let base = port_base(port);
        for _ in 0..100_000u32 {
            if reg16(base, OFF_INFO2).read_volatile() & INFO2_SCLKDIVEN != 0 {
                return;
            }
        }
        // Timeout — continue anyway; hardware may still work.
    }
}

// ---------------------------------------------------------------------------
// Async command issue
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SdhiError {
    /// Response timeout (no response received in time).
    ResponseTimeout,
    /// Response CRC error.
    ResponseCrc,
    /// Data CRC error.
    DataCrc,
    /// Data timeout.
    DataTimeout,
    /// SDHI command busy timeout.
    CmdBusy,
    /// Illegal register access.
    IllegalAccess,
    /// Generic hardware error.
    HardwareError,
}

impl From<SdhiError> for &'static str {
    fn from(e: SdhiError) -> &'static str {
        match e {
            SdhiError::ResponseTimeout => "SDHI: response timeout",
            SdhiError::ResponseCrc => "SDHI: response CRC error",
            SdhiError::DataCrc => "SDHI: data CRC error",
            SdhiError::DataTimeout => "SDHI: data timeout",
            SdhiError::CmdBusy => "SDHI: command busy timeout",
            SdhiError::IllegalAccess => "SDHI: illegal register access",
            SdhiError::HardwareError => "SDHI: hardware error",
        }
    }
}

fn check_info2_errors(info2: u16) -> Result<(), SdhiError> {
    if info2 & INFO2_ILA != 0 {
        return Err(SdhiError::IllegalAccess);
    }
    if info2 & INFO2_ERR6 != 0 {
        return Err(SdhiError::ResponseTimeout);
    }
    if info2 & INFO2_ERR5 != 0 {
        // SD_BUF illegal read — treat as hardware error
        return Err(SdhiError::HardwareError);
    }
    if info2 & INFO2_ERR4 != 0 {
        // SD_BUF illegal write — treat as hardware error
        return Err(SdhiError::HardwareError);
    }
    if info2 & INFO2_ERR0 != 0 {
        // Command index mismatch
        return Err(SdhiError::HardwareError);
    }
    if info2 & INFO2_ERR1 != 0 {
        return Err(SdhiError::ResponseCrc);
    }
    if info2 & INFO2_ERR2 != 0 {
        // END error (end-bit missing) — can occur in response or data
        return Err(SdhiError::HardwareError);
    }
    if info2 & INFO2_ERR3 != 0 {
        return Err(SdhiError::DataTimeout);
    }
    Ok(())
}

/// Issue an SD command and wait asynchronously for the response end.
///
/// `cmd_val` is written directly to the SD_CMD register.
/// The argument should have been set via [`set_arg`] immediately before.
///
/// Returns `Ok(())` when INFO1_RESP is set; `Err` on hardware error.
///
/// # Safety
/// Reads/writes SDHI peripheral registers. Must not be called concurrently
/// for the same port.
pub async unsafe fn send_cmd(port: u8, cmd_val: u16) -> Result<(), SdhiError> {
    unsafe {
        let base = port_base(port);

        // Wait for clock to be stable
        wait_clk_stable(port);

        // Clear any stale interrupt state
        clear_info(port);

        // Enable response-end and error interrupts.
        // Hardware polarity: 0 = enabled, 1 = masked → write bitwise complement.
        // INFO1: enable RESP only  → ~0x0001 = 0xFFFE
        // INFO2: enable ERR_ALL (includes ILA) → ~INFO2_ERR_ALL = 0x7F80
        reg16(base, OFF_INFO1_MASK).write_volatile(!INFO1_RESP);
        reg16(base, OFF_INFO2_MASK).write_volatile(!INFO2_ERR_ALL);

        // Issue command
        reg16(base, OFF_CMD).write_volatile(cmd_val);

        // Async wait
        let result = poll_fn(|cx| {
            let st = &STATE[port as usize];
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);

            // Check errors first
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            // Check success
            if i1 & INFO1_RESP != 0 {
                return Poll::Ready(Ok(()));
            }
            // Still waiting — register waker
            st.waker.register(cx.waker());
            // Re-check in case interrupt fired between the load and register
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_RESP != 0 {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await;

        // Disable all interrupts (0 = enabled, 1 = masked → 0xFFFF masks all)
        reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
        reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);

        // Clear accumulated state
        clear_info(port);

        result
    }
}

// ---------------------------------------------------------------------------
// Response reading
// ---------------------------------------------------------------------------

/// Read the 32-bit R1/R3/R6/R7 response (RESP0 + RESP1 combined).
///
/// # Safety
/// Reads SDHI response registers.
pub unsafe fn read_r1(port: u8) -> u32 {
    unsafe {
        let base = port_base(port);
        let lo = reg16(base, OFF_RESP0).read_volatile() as u32;
        let hi = reg16(base, OFF_RESP1).read_volatile() as u32;
        (hi << 16) | lo
    }
}

/// Read the full 128-bit R2 response (CID or CSD register).
///
/// Returns `[word0, word1, word2, word3]` where word0 contains bits [127:96].
///
/// # Safety
/// Reads SDHI response registers.
pub unsafe fn read_r2(port: u8) -> [u32; 4] {
    unsafe {
        let base = port_base(port);
        let r = |off: usize| reg16(base, off).read_volatile() as u32;
        [
            (r(OFF_RESP7) << 16) | r(OFF_RESP6),
            (r(OFF_RESP5) << 16) | r(OFF_RESP4),
            (r(OFF_RESP3) << 16) | r(OFF_RESP2),
            (r(OFF_RESP1) << 16) | r(OFF_RESP0),
        ]
    }
}

// ---------------------------------------------------------------------------
// Software (FIFO) data transfer
// ---------------------------------------------------------------------------

/// Read `count` 512-byte blocks from the card into `buf`.
///
/// Caller must have already:
///  1. Set block count: [`set_block_count`]
///  2. Issued CMD17 or CMD18 via [`send_cmd`]
///
/// Transfer is interrupt-driven: the hardware raises INFO2_BRE when the FIFO
/// has 512 bytes ready; we drain it, then repeat for each block.  After all
/// blocks arrive, we wait for INFO1_DATA_TRNS (access end).
///
/// # Safety
/// Reads/writes SDHI registers and the provided buffer.
pub async unsafe fn read_blocks_sw(port: u8, buf: *mut u8, count: u32) -> Result<(), SdhiError> {
    unsafe {
        let base = port_base(port);

        // Enable DATA_TRNS (access end), BRE, and all errors.
        // Hardware polarity: 0 = enabled, 1 = masked → write bitwise complement.
        // INFO1: enable DATA_TRNS only  → ~INFO1_DATA_TRNS = 0xFFFB
        // INFO2: enable ERR_ALL + BRE   → ~(INFO2_ERR_ALL | INFO2_BRE) = 0x7E80
        reg16(base, OFF_INFO1_MASK).write_volatile(!INFO1_DATA_TRNS);
        reg16(base, OFF_INFO2_MASK).write_volatile(!(INFO2_ERR_ALL | INFO2_BRE));

        let mut dst = buf;
        for _block in 0..count {
            // Wait for Buffer Read Enable
            poll_fn(|cx| {
                let st = &STATE[port as usize];
                let i2 = st.info2.load(Ordering::Acquire);
                if i2 & INFO2_BRE != 0 {
                    // Consume the BRE bit only
                    st.info2.fetch_and(!INFO2_BRE, Ordering::AcqRel);
                    return Poll::Ready(Ok::<(), SdhiError>(()));
                }
                if let Err(e) = check_info2_errors(i2) {
                    return Poll::Ready(Err(e));
                }
                st.waker.register(cx.waker());
                // Double-check
                let i2 = st.info2.load(Ordering::Acquire);
                if i2 & INFO2_BRE != 0 {
                    st.info2.fetch_and(!INFO2_BRE, Ordering::AcqRel);
                    return Poll::Ready(Ok(()));
                }
                if let Err(e) = check_info2_errors(i2) {
                    return Poll::Ready(Err(e));
                }
                Poll::Pending
            })
            .await?;

            // Drain 512 bytes from 32-bit FIFO (128 reads)
            let fifo = reg32(base, OFF_BUF0);
            for w in 0..128usize {
                let word = fifo.read_volatile();
                let p = dst.add(w * 4) as *mut u32;
                p.write_unaligned(word);
            }
            dst = dst.add(512);
        }

        // Wait for access end (DATA_TRNS)
        poll_fn(|cx| {
            let st = &STATE[port as usize];
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_DATA_TRNS != 0 {
                return Poll::Ready(Ok(()));
            }
            st.waker.register(cx.waker());
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_DATA_TRNS != 0 {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await?;

        // Disable all interrupts (0xFFFF = all masked)
        reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
        reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);
        clear_info(port);

        Ok(())
    }
}

/// Write `count` 512-byte blocks from `buf` to the card.
///
/// Caller must have already set block count and issued CMD24 or CMD25.
///
/// # Safety
/// Reads/writes SDHI registers and the provided buffer.
pub async unsafe fn write_blocks_sw(port: u8, buf: *const u8, count: u32) -> Result<(), SdhiError> {
    unsafe {
        let base = port_base(port);

        // Enable DATA_TRNS, BWE, and all errors.
        // Hardware polarity: 0 = enabled, 1 = masked → write bitwise complement.
        // INFO1: enable DATA_TRNS only  → ~INFO1_DATA_TRNS = 0xFFFB
        // INFO2: enable ERR_ALL + BWE   → ~(INFO2_ERR_ALL | INFO2_BWE) = 0x7D80
        reg16(base, OFF_INFO1_MASK).write_volatile(!INFO1_DATA_TRNS);
        reg16(base, OFF_INFO2_MASK).write_volatile(!(INFO2_ERR_ALL | INFO2_BWE));

        let mut src = buf;
        for _block in 0..count {
            // Wait for Buffer Write Enable
            poll_fn(|cx| {
                let st = &STATE[port as usize];
                let i2 = st.info2.load(Ordering::Acquire);
                if i2 & INFO2_BWE != 0 {
                    st.info2.fetch_and(!INFO2_BWE, Ordering::AcqRel);
                    return Poll::Ready(Ok::<(), SdhiError>(()));
                }
                if let Err(e) = check_info2_errors(i2) {
                    return Poll::Ready(Err(e));
                }
                st.waker.register(cx.waker());
                let i2 = st.info2.load(Ordering::Acquire);
                if i2 & INFO2_BWE != 0 {
                    st.info2.fetch_and(!INFO2_BWE, Ordering::AcqRel);
                    return Poll::Ready(Ok(()));
                }
                if let Err(e) = check_info2_errors(i2) {
                    return Poll::Ready(Err(e));
                }
                Poll::Pending
            })
            .await?;

            // Fill 512 bytes into 32-bit FIFO (128 writes)
            let fifo = reg32(base, OFF_BUF0);
            for w in 0..128usize {
                let p = src.add(w * 4) as *const u32;
                fifo.write_volatile(p.read_unaligned());
            }
            src = src.add(512);
        }

        // Wait for access end
        poll_fn(|cx| {
            let st = &STATE[port as usize];
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_DATA_TRNS != 0 {
                return Poll::Ready(Ok(()));
            }
            st.waker.register(cx.waker());
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_DATA_TRNS != 0 {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await?;

        reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
        reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);
        clear_info(port);

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// DMA data transfer
// ---------------------------------------------------------------------------

/// Read `count` 512-byte blocks from the card into `buf` using DMAC.
///
/// Handles the complete DMA transfer sequence per TRM §38.4.9:
/// 1. Enables SDHI DMA mode (`CC_EXT_MODE = 0x0002`) before issuing the command.
/// 2. Issues `cmd_val` (CMD17 or CMD18) and waits for response end.
/// 3. Configures and starts the DMAC (SD_BUF0 → `buf`).
/// 4. Waits for `INFO1_DATA_TRNS` via the SDHI interrupt.
///
/// Caller must have already called [`set_block_count`] and [`set_arg`].
///
/// # Safety
/// * `buf` must point to `count × 512` bytes of valid **uncached** memory
///   (virtual address in the uncached OCRAM mirror, i.e. physical +
///   `rza1::UNCACHED_MIRROR_OFFSET`).  Using a cached buffer results in
///   undefined cache coherence behaviour.
/// * Must not be called concurrently on the same port.
pub async unsafe fn read_blocks_dma(
    port: u8,
    cmd_val: u16,
    buf: *mut u8,
    count: u32,
    dma_ch: u8,
) -> Result<(), SdhiError> {
    unsafe {
        let base = port_base(port);
        let (dmars_rx, sd_buf0) = if port == 0 {
            (DMARS_SDHI0_RX, (SDHI0_BASE + OFF_BUF0) as u32)
        } else {
            (DMARS_SDHI1_RX, (SDHI1_BASE + OFF_BUF0) as u32)
        };

        // 1. Enable DMA mode BEFORE issuing the command (TRM §38.4.9).
        reg16(base, OFF_CC_EXT_MODE).write_volatile(CC_EXT_MODE_DMASDRW);

        // 2. Issue command and wait for response end (mirrors send_cmd).
        wait_clk_stable(port);
        clear_info(port);
        reg16(base, OFF_INFO1_MASK).write_volatile(!INFO1_RESP);
        reg16(base, OFF_INFO2_MASK).write_volatile(!INFO2_ERR_ALL);
        reg16(base, OFF_CMD).write_volatile(cmd_val);

        let cmd_result = poll_fn(|cx| {
            let st = &STATE[port as usize];
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_RESP != 0 {
                return Poll::Ready(Ok(()));
            }
            st.waker.register(cx.waker());
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_RESP != 0 {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await;

        reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
        reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);
        clear_info(port);

        if let Err(e) = cmd_result {
            reg16(base, OFF_CC_EXT_MODE).write_volatile(0x0000);
            return Err(e);
        }

        // 3. Configure DMAC: SD_BUF0 (fixed source) → buf (incrementing dest).
        crate::dmac::init_register_mode_rx(dma_ch, chcfg_sdhi_rx(dma_ch), sd_buf0, dmars_rx);

        // 4. Enable DATA_TRNS + error interrupts, then kick the DMAC.
        reg16(base, OFF_INFO1_MASK).write_volatile(!INFO1_DATA_TRNS);
        reg16(base, OFF_INFO2_MASK).write_volatile(!INFO2_ERR_ALL);
        crate::dmac::start_transfer_rx(dma_ch, buf as u32, count * 512);

        // 5. Wait for transfer complete (DATA_TRNS via SDHI interrupt).
        let data_result = poll_fn(|cx| {
            let st = &STATE[port as usize];
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_DATA_TRNS != 0 {
                return Poll::Ready(Ok(()));
            }
            st.waker.register(cx.waker());
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_DATA_TRNS != 0 {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await;

        // 6. Restore software transfer mode.
        reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
        reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);
        clear_info(port);
        reg16(base, OFF_CC_EXT_MODE).write_volatile(0x0000);

        data_result
    }
}

/// Write `count` 512-byte blocks from `buf` to the card using DMAC.
///
/// Handles the complete DMA transfer sequence per TRM §38.4.9:
/// 1. Enables SDHI DMA mode (`CC_EXT_MODE = 0x0002`) before issuing the command.
/// 2. Issues `cmd_val` (CMD24 or CMD25) and waits for response end.
/// 3. Configures and starts the DMAC (`buf` → SD_BUF0).
/// 4. Waits for `INFO1_DATA_TRNS` via the SDHI interrupt.
///
/// Caller must have already called [`set_block_count`] and [`set_arg`].
///
/// # Safety
/// * `buf` must point to `count × 512` bytes of valid **uncached** memory
///   so the DMAC reads the CPU-written bytes without stale cache lines.
/// * Must not be called concurrently on the same port.
pub async unsafe fn write_blocks_dma(
    port: u8,
    cmd_val: u16,
    buf: *const u8,
    count: u32,
    dma_ch: u8,
) -> Result<(), SdhiError> {
    unsafe {
        let base = port_base(port);
        let (dmars_tx, sd_buf0) = if port == 0 {
            (DMARS_SDHI0_TX, (SDHI0_BASE + OFF_BUF0) as u32)
        } else {
            (DMARS_SDHI1_TX, (SDHI1_BASE + OFF_BUF0) as u32)
        };

        // 1. Enable DMA mode BEFORE issuing the command (TRM §38.4.9).
        reg16(base, OFF_CC_EXT_MODE).write_volatile(CC_EXT_MODE_DMASDRW);

        // 2. Issue command and wait for response end.
        wait_clk_stable(port);
        clear_info(port);
        reg16(base, OFF_INFO1_MASK).write_volatile(!INFO1_RESP);
        reg16(base, OFF_INFO2_MASK).write_volatile(!INFO2_ERR_ALL);
        reg16(base, OFF_CMD).write_volatile(cmd_val);

        let cmd_result = poll_fn(|cx| {
            let st = &STATE[port as usize];
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_RESP != 0 {
                return Poll::Ready(Ok(()));
            }
            st.waker.register(cx.waker());
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_RESP != 0 {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await;

        reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
        reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);
        clear_info(port);

        if let Err(e) = cmd_result {
            reg16(base, OFF_CC_EXT_MODE).write_volatile(0x0000);
            return Err(e);
        }

        // 3. Configure DMAC: buf (incrementing source) → SD_BUF0 (fixed dest).
        crate::dmac::init_register_mode(dma_ch, chcfg_sdhi_tx(dma_ch), sd_buf0, dmars_tx);

        // 4. Enable DATA_TRNS + error interrupts, then kick the DMAC.
        reg16(base, OFF_INFO1_MASK).write_volatile(!INFO1_DATA_TRNS);
        reg16(base, OFF_INFO2_MASK).write_volatile(!INFO2_ERR_ALL);
        crate::dmac::start_transfer(dma_ch, buf as u32, count * 512);

        // 5. Wait for transfer complete (DATA_TRNS via SDHI interrupt).
        let data_result = poll_fn(|cx| {
            let st = &STATE[port as usize];
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_DATA_TRNS != 0 {
                return Poll::Ready(Ok(()));
            }
            st.waker.register(cx.waker());
            let i1 = st.info1.load(Ordering::Acquire);
            let i2 = st.info2.load(Ordering::Acquire);
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            if i1 & INFO1_DATA_TRNS != 0 {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await;

        // 6. Restore software transfer mode.
        reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
        reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);
        clear_info(port);
        reg16(base, OFF_CC_EXT_MODE).write_volatile(0x0000);

        data_result
    }
}

// ---------------------------------------------------------------------------
// Block-count / stop-transfer helpers
// ---------------------------------------------------------------------------

/// Set the number of blocks for a multi-block transfer.
///
/// Also writes SD_STOP to auto-stop after the given count (CMD12 auto-issue).
///
/// # Safety
/// Writes SDHI registers.
pub unsafe fn set_block_count(port: u8, count: u32) {
    unsafe {
        let base = port_base(port);
        reg16(base, OFF_SECCNT).write_volatile(count as u16);
        // Auto-stop enable (bit 8) when count > 1; otherwise no auto-stop.
        if count > 1 {
            reg16(base, OFF_STOP).write_volatile(0x0100); // SEC bit: auto-issue CMD12
        } else {
            reg16(base, OFF_STOP).write_volatile(0x0000);
        }
    }
}

/// Issue CMD12 (STOP_TRANSMISSION) to abort a multi-block transfer.
///
/// # Safety
/// Writes SDHI registers.
pub async unsafe fn stop_transfer(port: u8) -> Result<(), SdhiError> {
    unsafe {
        set_arg(port, 0);
        send_cmd(port, 12u16).await // CMD12
    }
}

// ---------------------------------------------------------------------------
// Card detect
// ---------------------------------------------------------------------------

/// Return `true` if a card is physically inserted (CD pin low = card present).
///
/// Reads INFO1 bit 5 (INFO5 = current SD_CD level) directly from hardware.
/// Bit 5 = 1 means SD_CD has been held low for at least Ncycle → card present.
///
/// Do NOT use INFO1_INS_CD (bit 4) here — that is an edge-triggered latch
/// (set on insertion, must be cleared) not the steady-state level.
///
/// # Safety
/// Reads SDHI INFO1 register.
pub unsafe fn card_inserted(port: u8) -> bool {
    unsafe {
        let base = port_base(port);
        let info1 = reg16(base, OFF_INFO1).read_volatile();
        // INFO1_CD_LEVEL (bit 5 = INFO5): 1 = SD_CD held low = card present.
        info1 & INFO1_CD_LEVEL != 0
    }
}

/// Enable card-detect interrupts (insert + remove) in INFO1_MASK.
///
/// # Safety
/// Writes SDHI INFO1_MASK register.
pub unsafe fn enable_card_detect_irq(port: u8) {
    unsafe {
        let base = port_base(port);
        // Hardware polarity: 0 = enabled, 1 = masked → clear bits to enable.
        let mask = reg16(base, OFF_INFO1_MASK).read_volatile();
        reg16(base, OFF_INFO1_MASK).write_volatile(mask & !INFO1_DET_CD);
    }
}

// ---------------------------------------------------------------------------
// Card capacity (set by deluge-bsp after CSD decode)
// ---------------------------------------------------------------------------

/// Total card capacity in 512-byte blocks, per port.
///
/// Populated by the BSP layer via [`set_card_blocks`] during card init so
/// that the polling `BlockDevice` impl can return a reliable block count.
static CARD_BLOCKS: [AtomicU32; NUM_PORTS] = [AtomicU32::new(0), AtomicU32::new(0)];

/// Store the card capacity (in 512-byte blocks) for `port`.
///
/// Called by the BSP once the CSD register has been decoded.
pub fn set_card_blocks(port: u8, blocks: u32) {
    CARD_BLOCKS[port as usize].store(blocks, Ordering::Release);
}

/// Return the stored card capacity (in 512-byte blocks) for `port`.
///
/// Returns 0 if [`set_card_blocks`] has not been called yet.
pub fn card_size_blocks(port: u8) -> u32 {
    CARD_BLOCKS[port as usize].load(Ordering::Acquire)
}

// ---------------------------------------------------------------------------
// Polling (synchronous, executor-free) data transfer
// ---------------------------------------------------------------------------
//
// These functions mirror the async read_blocks_sw / write_blocks_sw paths
// but spin on hardware register bits instead of using AtomicWaker.  They are
// safe to call outside of an Embassy executor (e.g. from embedded-sdmmc's
// synchronous BlockDevice trait impl).
//
// A timeout counter guards against hardware hang.  The limit is chosen to be
// comfortably above the maximum expected transfer time for 512-byte blocks at
// the fast SD clock (~33 MHz):  512 bytes / 33 MHz ≈ 120 µs → ≈ 4 000 ticks
// at P0 / 8.  We use 10 000 000 to allow for card back-off without being
// infinite.

const POLL_TIMEOUT: u32 = 10_000_000;

/// Issue an SD command using register polling (no IRQ / no executor).
///
/// Equivalent to [`send_cmd`] but spins on INFO1_RESP / INFO2_ERR_ALL.
///
/// # Safety
/// Must not be called concurrently on the same port.  Writes SDHI registers.
pub unsafe fn send_cmd_poll(port: u8, cmd_val: u16) -> Result<(), SdhiError> {
    unsafe {
        let base = port_base(port);

        wait_clk_stable(port);
        clear_info(port);

        // Mask all interrupts (we poll instead).
        reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
        reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);

        // Issue command.
        reg16(base, OFF_CMD).write_volatile(cmd_val);

        // Spin until RESP or error.
        for _ in 0..POLL_TIMEOUT {
            let info1 = reg16(base, OFF_INFO1).read_volatile();
            let info2 = reg16(base, OFF_INFO2).read_volatile();

            if info2 & INFO2_ERR_ALL != 0 {
                // Clear before returning.
                reg16(base, OFF_INFO1).write_volatile(!info1);
                reg16(base, OFF_INFO2).write_volatile(!info2);
                clear_info(port);
                return check_info2_errors(info2);
            }
            if info1 & INFO1_RESP != 0 {
                reg16(base, OFF_INFO1).write_volatile(!info1);
                reg16(base, OFF_INFO2).write_volatile(!info2);
                clear_info(port);
                return Ok(());
            }
        }

        clear_info(port);
        Err(SdhiError::ResponseTimeout)
    }
}

/// Read `count` 512-byte blocks from the card using register polling.
///
/// The caller must have already:
///   1. Set block count via [`set_block_count`].
///   2. Issued the read command via [`send_cmd_poll`].
///
/// # Safety
/// Must not be called concurrently on the same port.
pub unsafe fn read_blocks_poll(port: u8, buf: *mut u8, count: u32) -> Result<(), SdhiError> {
    unsafe {
        let base = port_base(port);

        let mut dst = buf;
        for _block in 0..count {
            // Wait for Buffer Read Enable (INFO2_BRE).
            let mut ready = false;
            for _ in 0..POLL_TIMEOUT {
                let info2 = reg16(base, OFF_INFO2).read_volatile();
                if info2 & INFO2_ERR_ALL != 0 {
                    reg16(base, OFF_INFO2).write_volatile(!info2);
                    return check_info2_errors(info2);
                }
                if info2 & INFO2_BRE != 0 {
                    // Clear only BRE.
                    reg16(base, OFF_INFO2).write_volatile(!(INFO2_BRE));
                    ready = true;
                    break;
                }
            }
            if !ready {
                return Err(SdhiError::DataTimeout);
            }

            // Drain 512 bytes from 32-bit FIFO (128 × 32-bit reads).
            let fifo = reg32(base, OFF_BUF0);
            for w in 0..128usize {
                let word = fifo.read_volatile();
                let p = dst.add(w * 4) as *mut u32;
                p.write_unaligned(word);
            }
            dst = dst.add(512);
        }

        // Wait for access end (INFO1_DATA_TRNS).
        for _ in 0..POLL_TIMEOUT {
            let info2 = reg16(base, OFF_INFO2).read_volatile();
            if info2 & INFO2_ERR_ALL != 0 {
                reg16(base, OFF_INFO2).write_volatile(!info2);
                return check_info2_errors(info2);
            }
            let info1 = reg16(base, OFF_INFO1).read_volatile();
            if info1 & INFO1_DATA_TRNS != 0 {
                reg16(base, OFF_INFO1).write_volatile(!info1);
                reg16(base, OFF_INFO2).write_volatile(!info2);
                return Ok(());
            }
        }

        Err(SdhiError::DataTimeout)
    }
}

/// Write `count` 512-byte blocks to the card using register polling.
///
/// The caller must have already set block count and issued the write command.
///
/// # Safety
/// Must not be called concurrently on the same port.
pub unsafe fn write_blocks_poll(port: u8, buf: *const u8, count: u32) -> Result<(), SdhiError> {
    unsafe {
        let base = port_base(port);

        let mut src = buf;
        for _block in 0..count {
            // Wait for Buffer Write Enable (INFO2_BWE).
            let mut ready = false;
            for _ in 0..POLL_TIMEOUT {
                let info2 = reg16(base, OFF_INFO2).read_volatile();
                if info2 & INFO2_ERR_ALL != 0 {
                    reg16(base, OFF_INFO2).write_volatile(!info2);
                    return check_info2_errors(info2);
                }
                if info2 & INFO2_BWE != 0 {
                    reg16(base, OFF_INFO2).write_volatile(!(INFO2_BWE));
                    ready = true;
                    break;
                }
            }
            if !ready {
                return Err(SdhiError::DataTimeout);
            }

            // Fill 512 bytes into 32-bit FIFO (128 × 32-bit writes).
            let fifo = reg32(base, OFF_BUF0);
            for w in 0..128usize {
                let p = src.add(w * 4) as *const u32;
                fifo.write_volatile(p.read_unaligned());
            }
            src = src.add(512);
        }

        // Wait for access end.
        for _ in 0..POLL_TIMEOUT {
            let info2 = reg16(base, OFF_INFO2).read_volatile();
            if info2 & INFO2_ERR_ALL != 0 {
                reg16(base, OFF_INFO2).write_volatile(!info2);
                return check_info2_errors(info2);
            }
            let info1 = reg16(base, OFF_INFO1).read_volatile();
            if info1 & INFO1_DATA_TRNS != 0 {
                reg16(base, OFF_INFO1).write_volatile(!info1);
                reg16(base, OFF_INFO2).write_volatile(!info2);
                return Ok(());
            }
        }

        Err(SdhiError::DataTimeout)
    }
}

// ---------------------------------------------------------------------------
// Owned typed wrapper
// ---------------------------------------------------------------------------

/// Owned handle for SDHI port `PORT` (0 or 1).
///
/// Wraps the free functions in an object-oriented API.  The type parameter
/// `PORT` encodes the port number at compile time.
pub struct Sdhi<const PORT: u8>;

impl<const PORT: u8> Sdhi<PORT> {
    /// Claim ownership of SDHI port `PORT`.
    ///
    /// # Safety
    /// The caller must ensure no other code uses port `PORT` concurrently, and
    /// that the SDHI clock has been enabled (via `stb::init` or equivalent).
    pub unsafe fn new() -> Self {
        Sdhi
    }

    /// Initialise the hardware for this port.
    ///
    /// # Safety
    /// Writes memory-mapped SDHI registers.
    pub unsafe fn init(&self, sd_option: u16) {
        unsafe { init(PORT, sd_option) }
    }

    /// Switch the SD clock to high speed (~16.7 MHz, P1/4).
    ///
    /// # Safety
    /// Writes memory-mapped SDHI registers.
    pub unsafe fn set_clock_fast(&self) {
        unsafe { set_clock_fast(PORT) }
    }

    /// Register GIC IRQs for this port's interrupt handler.
    ///
    /// # Safety
    /// Writes GIC registers.
    pub unsafe fn register_irqs(&self) {
        unsafe { register_irqs(PORT) }
    }

    /// Returns `true` if a card is detected in the slot.
    ///
    /// # Safety
    /// Reads memory-mapped SDHI registers.
    pub unsafe fn card_inserted(&self) -> bool {
        unsafe { card_inserted(PORT) }
    }

    /// Set the number of blocks for a multi-block transfer command.
    ///
    /// # Safety
    /// Writes memory-mapped SDHI registers.
    pub unsafe fn set_block_count(&self, count: u32) {
        unsafe { set_block_count(PORT, count) }
    }

    /// Issue an SD command and wait (polling) for a response.
    ///
    /// # Safety
    /// Must not be called concurrently.  Writes SDHI registers.
    pub unsafe fn send_cmd_poll(&self, cmd_val: u16) -> Result<(), SdhiError> {
        unsafe { send_cmd_poll(PORT, cmd_val) }
    }

    /// Read `count` 512-byte blocks into `buf` using register polling.
    ///
    /// # Safety
    /// Must not be called concurrently.  `buf` must be valid for `count * 512` bytes.
    pub unsafe fn read_blocks_poll(&self, buf: *mut u8, count: u32) -> Result<(), SdhiError> {
        unsafe { read_blocks_poll(PORT, buf, count) }
    }

    /// Write `count` 512-byte blocks from `buf` using register polling.
    ///
    /// # Safety
    /// Must not be called concurrently.  `buf` must be valid for `count * 512` bytes.
    pub unsafe fn write_blocks_poll(&self, buf: *const u8, count: u32) -> Result<(), SdhiError> {
        unsafe { write_blocks_poll(PORT, buf, count) }
    }
    /// Read `count` 512-byte blocks using DMAC (async, DMA-driven).
    ///
    /// Caller must have called [`set_block_count`] and [`set_arg`] first.
    /// `buf` must be uncached memory (`physical + rza1::UNCACHED_MIRROR_OFFSET`).
    ///
    /// # Safety
    /// Same requirements as [`read_blocks_dma`].
    pub async unsafe fn read_blocks_dma(
        &self,
        cmd_val: u16,
        buf: *mut u8,
        count: u32,
        dma_ch: u8,
    ) -> Result<(), SdhiError> {
        unsafe { read_blocks_dma(PORT, cmd_val, buf, count, dma_ch).await }
    }

    /// Write `count` 512-byte blocks using DMAC (async, DMA-driven).
    ///
    /// Caller must have called [`set_block_count`] and [`set_arg`] first.
    /// `buf` must be uncached memory.
    ///
    /// # Safety
    /// Same requirements as [`write_blocks_dma`].
    pub async unsafe fn write_blocks_dma(
        &self,
        cmd_val: u16,
        buf: *const u8,
        count: u32,
        dma_ch: u8,
    ) -> Result<(), SdhiError> {
        unsafe { write_blocks_dma(PORT, cmd_val, buf, count, dma_ch).await }
    }
}

// ---------------------------------------------------------------------------
// Unit tests (host-side, register address verification only)
// ---------------------------------------------------------------------------

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn sdhi1_base_address() {
        assert_eq!(SDHI1_BASE, 0xE804_E800);
    }

    #[test]
    fn register_offsets() {
        // Verify register byte offsets from TRM table 38.2
        // (SDHI0 base = H'E804_E000; offsets are absolute byte distances)
        assert_eq!(OFF_CMD, 0x00);
        assert_eq!(OFF_ARG0, 0x04);
        assert_eq!(OFF_ARG1, 0x06);
        assert_eq!(OFF_STOP, 0x08);
        assert_eq!(OFF_SECCNT, 0x0A);
        assert_eq!(OFF_RESP0, 0x0C);
        assert_eq!(OFF_RESP1, 0x0E);
        assert_eq!(OFF_INFO1, 0x1C);
        assert_eq!(OFF_INFO2, 0x1E);
        assert_eq!(OFF_INFO1_MASK, 0x20);
        assert_eq!(OFF_INFO2_MASK, 0x22);
        assert_eq!(OFF_CLK_CTRL, 0x24);
        assert_eq!(OFF_SIZE, 0x26);
        assert_eq!(OFF_OPTION, 0x28);
        assert_eq!(OFF_ERR_STS1, 0x2C);
        assert_eq!(OFF_ERR_STS2, 0x2E);
        assert_eq!(OFF_BUF0, 0x30);
        assert_eq!(OFF_SDIO_MODE, 0x34);
        assert_eq!(OFF_SDIO_INFO1, 0x36);
        assert_eq!(OFF_SDIO_INFO1_MASK, 0x38);
        assert_eq!(OFF_CC_EXT_MODE, 0xD8);
        assert_eq!(OFF_SOFT_RST, 0xE0);
        assert_eq!(OFF_EXT_SWAP, 0xF0);
    }

    #[test]
    fn info_bit_masks() {
        assert_eq!(INFO1_RESP, 0x0001);
        assert_eq!(INFO1_DATA_TRNS, 0x0004);
        assert_eq!(INFO2_BRE, 0x0100);
        assert_eq!(INFO2_BWE, 0x0200);
        assert_eq!(INFO2_CBSY, 0x4000);
    }

    #[test]
    fn gic_ids_for_sdhi1() {
        assert_eq!(SDHI1_IRQS, [305, 306, 307]);
    }
}
