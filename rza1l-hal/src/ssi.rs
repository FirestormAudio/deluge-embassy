//! Synchronous Serial Interface (SSIF) driver for the RZ/A1L — 44.1 kHz I²S.
//!
//! Implements stereo audio streaming on SSI channel 0 using DMA circular
//! link-descriptor mode.  Only SSI0 is wired to the audio codec on Deluge
//! hardware; the other five channels are unused.
//!
//! ## Clock derivation
//! ```text
//! AUDIO_X1 ≈ 22.5792 MHz  (external crystal on the Deluge board)
//! ÷ 4  (CKDV = 0b0010)  →  BCLK = 5.6448 MHz
//! ÷ 64 (2 ch × 32-bit SWL)  →  88 200 half-frames / s
//! ÷ 2  →  44 100 Hz sample rate
//! ```
//!
//! ## DMA layout
//! | Channel | Direction       | GIC IRQ |
//! |---------|-----------------|---------|
//! | DMA 6   | SRAM → SSIFTDR  | 47      |
//! | DMA 7   | SSIFRDR → SRAM  | 48      |
//!
//! Both DMA channels run in self-referential circular link-descriptor mode:
//! the `next` pointer in each descriptor points back to itself, so the DMA
//! engine re-arms automatically after each pass through the buffer.
//!
//! ## Buffer sizes
//! | Buffer | Frames  | Stereo samples | Bytes  | Duration |
//! |--------|---------|----------------|--------|----------|
//! | TX     | 1 024   | 2 048          | 8 192  | ~23.2 ms |
//! | RX     | 2 048   | 4 096          | 16 384 | ~46.4 ms |
//!
//! ## Uncached access
//! DMA transfers bypass the CPU cache.  Any CPU code that reads/writes the
//! sample buffers must use the **uncached mirror** alias
//! (`physical address + UNCACHED_MIRROR_OFFSET`) to avoid stale cache lines.
//! [`tx_buf_start`], [`tx_buf_end`], [`rx_buf_start`], [`rx_buf_end`],
//! [`tx_current_ptr`], and [`rx_current_ptr`] all return uncached pointers.

use crate::dmac;

// ── Channel / resource constants ────────────────────────────────────────────

/// SSI hardware channel used for audio (SSI0).
pub const SSI_CH: u8 = 0;

/// Stored DMA channel numbers (set by [`init`] / [`init_rx_only`]).
/// Used by [`tx_current_ptr`] and [`rx_current_ptr`] without needing the
/// original config reference.
static TX_DMA_CH_ACTIVE: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);
static RX_DMA_CH_ACTIVE: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

/// Board-specific SSI0 configuration.
///
/// Pass to [`init`] and [`init_rx_only`] so the HAL does not hard-code
/// Deluge-specific DMA channels or clock settings.
pub struct SsiConfig {
    /// SSICR initialisation value (master/slave mode, clock divider, word length).
    pub ssicr: u32,
    /// DMA channel number for SSI0 TX (SRAM → SSIFTDR).
    pub tx_dma_ch: u8,
    /// DMA channel number for SSI0 RX (SSIFRDR → SRAM).
    pub rx_dma_ch: u8,
}

/// DMARS resource-selector value for SSI0 transmit.
const DMARS_SSI0_TX: u32 = 0x00E1;
/// DMARS resource-selector value for SSI0 receive.
const DMARS_SSI0_RX: u32 = 0x00E2;

// ── CHCFG register field constants (shared with scux; defined in dmac) ───────
use crate::dmac::{
    CHCFG_AM_BURST, CHCFG_DAD, CHCFG_DDS_32BIT, CHCFG_DMS, CHCFG_HIEN, CHCFG_LVL,
    CHCFG_REQD, CHCFG_SAD, CHCFG_SDS_32BIT,
};

// ── SSI0 register map ────────────────────────────────────────────────────────

const SSI0_BASE: usize = 0xE820_B000;
const SSICR_OFF: usize = 0x00; // Control register
const SSIFCR_OFF: usize = 0x10; // FIFO control register
const SSIFTDR_OFF: usize = 0x18; // TX FIFO data register
const SSIFRDR_OFF: usize = 0x1C; // RX FIFO data register
const SSITDMR_OFF: usize = 0x20; // TDM mode register

// SSICR initialisation value — from `src/RZA1/ssi/drv_ssif_user.h`
// `SSI_SSICR0_USER_INIT_VALUE`:
//   - CKS=0  (bit 30)   : AUDIO_X1 clock source
//   - CHNL=00 (23:22)  : 1 channel pair (stereo)
//   - DWL=101 (21:19)  : 24-bit data word  [SSI_SSICR0_DWL_VALUE = 0x00280000]
//   - SWL=011 (18:16)  : 32-bit system word [SSI_SSICR0_SWL_VALUE = 0x00030000]
//   - SCKD=1  (bit 15) : RZ/A1L generates BCLK (master)
//   - SWSD=1  (bit 14) : RZ/A1L generates LRCK (master)
//   - SCKP=SWSP=SPDP=SDTA=PDTA=DEL=0 (all defaults)
//   - CKDV=0010 (7:4)  : AUDIO_X1 ÷ 4 = 5.6448 MHz BCLK
//
// NOTE: the previously-used value 0x003B_C020 had DWL=7 (prohibited). The
// correct Deluge SSICR value is 0x002B_C020 (DWL bits [21:19] = 0b101 = 24-bit),
// held in `deluge_bsp::system::SSI_CONFIG.ssicr`.

/// SSIFCR bit positions (verified against vendor/rbsp ssif.h SSIF_FCR_SHIFT_*).
const SSIFCR_RFRST: u32 = 1 << 0; // RX FIFO reset (active high)
const SSIFCR_TFRST: u32 = 1 << 1; // TX FIFO reset (active high)
const SSIFCR_RIE: u32 = 1 << 2; // RX interrupt enable (→ RXI DMA trigger)
const SSIFCR_TIE: u32 = 1 << 3; // TX interrupt enable (→ TXI DMA trigger)
// RTRG[1:0] = bits 5:4 — RX trigger level
// TTRG[1:0] = bits 7:6 — TX trigger level

/// SSICR bit positions (verified against vendor/rbsp ssif.h SSIF_CR_SHIFT_*).
const SSICR_REN: u32 = 1 << 0; // Receive enable
const SSICR_TEN: u32 = 1 << 1; // Transmit enable

/// SSITDMR bit 8: WS continue mode.
/// BSP (scux_dev.c): set for master mode — keeps WS clock running between frames.
/// Without this the WS line idles low between frames and the codec loses sync.
/// BSP: SCUX_SSITDMR_CONT_SET = (1U << 8)  (ssif.h: SSIF_TDMR_BIT_CONT)
const SSITDMR_CONT: u32 = 1 << 8;

/// SSIFCR bits [7:6]: TTRG — TX trigger level (0b10 = assert DMA request when ≥ 4 TX FIFO
/// stages are empty).  BSP: `SSI_SSIFCR_TTRG_INIT_VALUE = 0x000000A0` (Rohan).
const SSIFCR_TTRG_4: u32 = 0b10 << 6; // = 0x80
/// SSIFCR bits [5:4]: RTRG — RX trigger level (0b10 = assert DMA request when ≥ 4 RX FIFO
/// stages have been filled).
const SSIFCR_RTRG_4: u32 = 0b10 << 4; // = 0x20

/// SSIFCR initialisation value: both FIFOs held in reset, TX/RX triggers set to 4
/// stages, interrupts disabled.  Mirrors `SSI_SSIFCR_BASE_INIT_VALUE` from `drv_ssif.h`.
///   TFRST=1 | RFRST=1 | TTRG=0b10 (bits [7:6]) | RTRG=0b10 (bits [5:4]) | TIE=0 | RIE=0
///   = 0x02 | 0x01 | 0x80 | 0x20 = 0xA3
const SSIFCR_INIT: u32 = SSIFCR_TFRST | SSIFCR_RFRST | SSIFCR_TTRG_4 | SSIFCR_RTRG_4;

// ── CPG soft-reset ───────────────────────────────────────────────────────────

/// CPG SWRSTCR1 — byte register that controls peripheral soft-reset.
/// Bit `(6 - ssi_channel)` asserts / deasserts the SSI reset.
const SWRSTCR1: usize = 0xFCFE_0460;

// ── Uncached mirror ──────────────────────────────────────────────────────────

/// Add this offset to any cached internal-SRAM virtual address to obtain its
/// uncached alias.  DMA transfers go directly to the physical bus, bypassing
/// the CPU cache, so the CPU must access DMA buffers through this alias.
// UNCACHED_MIRROR_OFFSET is defined at crate root (crate::UNCACHED_MIRROR_OFFSET).
use crate::UNCACHED_MIRROR_OFFSET;

// ── TX / RX sample buffers ───────────────────────────────────────────────────

/// TX buffer depth: stereo frames.
pub const TX_FRAMES: usize = 1024;
/// RX buffer depth: stereo frames.
pub const RX_FRAMES: usize = 2048;

/// Total TX buffer length in `i32` samples (TX_FRAMES × 2 mono channels).
pub const TX_BUF_LEN: usize = TX_FRAMES * 2;
/// Total RX buffer length in `i32` samples (RX_FRAMES × 2 mono channels).
pub const RX_BUF_LEN: usize = RX_FRAMES * 2;

/// Cache-line-aligned wrapper so `repr(align(32))` can apply to a fixed-size
/// array.  The DMA hardware requires descriptors and buffers to start on a
/// 32-byte (cache-line) boundary.
#[repr(align(32))]
struct Aligned32<const N: usize>([i32; N]);

// SAFETY: these buffers are only accessed through the uncached alias during
// normal operation; the DMA engine accesses them on the bus side.
static mut TX_BUF: Aligned32<TX_BUF_LEN> = Aligned32([0i32; TX_BUF_LEN]);
static mut RX_BUF: Aligned32<RX_BUF_LEN> = Aligned32([0i32; RX_BUF_LEN]);

// ── DMA link descriptors ─────────────────────────────────────────────────────

/// An 8-word, cache-line-aligned DMA link descriptor.
///
/// Word layout (matching the Renesas RZ/A1L DMA hardware):
/// | Word | Field       | Description                                |
/// |------|-------------|--------------------------------------------|
/// | 0    | HEADER      | Link-descriptor control flags (0b1101)     |
/// | 1    | N0SA / src  | Source address                             |
/// | 2    | N0DA / dst  | Destination address                        |
/// | 3    | N0TB / size | Transfer byte count                        |
/// | 4    | CHCFG       | Channel config (loaded into CHCFG register)|
/// | 5    | CHITVL      | Interval register (0 = no delay)           |
/// | 6    | CHEXT       | Extension register (0)                     |
/// | 7    | NXLA        | Address of the *next* descriptor (patched  |
/// |      |             | at runtime to point back to self)          |
#[repr(C, align(32))]
struct LinkDesc([u32; 8]);

/// Link-descriptor HEADER word.
///
/// Bit layout (TRM §9.5 link-descriptor format):
///   bit 3 (LDEN): 1 — this channel uses link-descriptor mode
///   bit 2 (NXA):  1 — NXLA field (word\[7\]) holds the next descriptor address
///   bit 1:        0 — reserved
///   bit 0:        1 — descriptor valid
/// Combined: 0b1101 = LDEN | NXA | valid.
const DESC_HEADER: u32 = 0b1101;

/// Mask to align a DMA current-address register down to a stereo-frame boundary.
/// One stereo frame = 2 × i32 = 8 bytes, so the mask clears the low 3 bits.
const STEREO_FRAME_ALIGN_MASK: u32 = !7u32;

// CHCFG values for the two SSI DMA channels, composed from named field constants.
// Reference: C BSP `drivers/ssi/ssi.c` link-descriptor initialisers:
//
//   TX: DMS | DEM | DAD | DDS_32BIT | SDS_32BIT | AM_BURST | HIEN | REQD | LVL | sel
//       = 0x8000_0000 | 0x0100_0000 | 0x0020_0000 | 0x0002_0000 | 0x0000_2000
//         | 0x0000_0200 | 0x0000_0020 | 0x0000_0008 | 0x0000_0040 | ch
//       = 0x8122_2268 | ch  (ch=6 → 0x8122_226E)
//   RX: DMS | DEM | SAD | DDS_32BIT | SDS_32BIT | AM_BURST | HIEN | LVL | sel
//       = 0x8000_0000 | 0x0100_0000 | 0x0010_0000 | 0x0002_0000 | 0x0000_2000
//         | 0x0000_0200 | 0x0000_0020 | 0x0000_0040 | ch
//       = 0x8112_2260 | ch  (ch=7 → 0x8112_2267)

/// Compute CHCFG word for an SSI TX DMA channel (SRAM → SSIFTDR).
/// Source address increments (SAD=0); destination is fixed SSIFTDR (DAD=1).
/// Trigger: destination-select (REQD=1) — SSI TX FIFO drives the request.
/// DEM is cleared so DMAINT fires once per full buffer pass (enables interrupt).
#[inline(always)]
const fn tx_chcfg(dma_ch: u8) -> u32 {
    CHCFG_DMS
        | CHCFG_DAD
        | CHCFG_DDS_32BIT
        | CHCFG_SDS_32BIT
        | CHCFG_AM_BURST
        | CHCFG_HIEN
        | CHCFG_REQD
        | CHCFG_LVL
        | (dma_ch as u32 & 7)
}

/// Compute CHCFG word for an SSI RX DMA channel (SSIFRDR → SRAM).
/// Source is fixed SSIFRDR (SAD=1); destination address increments (DAD=0).
/// Trigger: source-select (REQD=0) — SSI RX FIFO drives the request.
/// DEM is cleared so DMAINT fires once per full buffer pass (enables interrupt).
#[inline(always)]
const fn rx_chcfg(dma_ch: u8) -> u32 {
    CHCFG_DMS
        | CHCFG_SAD
        | CHCFG_DDS_32BIT
        | CHCFG_SDS_32BIT
        | CHCFG_AM_BURST
        | CHCFG_HIEN
        | CHCFG_LVL
        | (dma_ch as u32 & 7)
}

/// TX link descriptor.  Word[1] (src), word[4] (CHCFG), and word[7] (NXLA)
/// are patched at runtime by [`init`].
static mut TX_DESC: LinkDesc = LinkDesc([
    DESC_HEADER,                                       // Header: LDEN + NXA + valid
    0,                                                 // src  (→ TX_BUF, set at init)
    (SSI0_BASE + SSIFTDR_OFF) as u32,                  // dst  = SSIFTDR (fixed)
    (TX_BUF_LEN * core::mem::size_of::<i32>()) as u32, // transfer bytes
    0,                                                 // CHCFG (patched at init)
    0,                                                 // CHITVL (no interval)
    0,                                                 // CHEXT
    0,                                                 // NXLA → &TX_DESC (set at init)
]);

/// RX link descriptor.  Word[2] (dst), word[4] (CHCFG), and word[7] (NXLA)
/// are patched at runtime by [`init`].
static mut RX_DESC: LinkDesc = LinkDesc([
    DESC_HEADER,                                       // Header: LDEN + NXA + valid
    (SSI0_BASE + SSIFRDR_OFF) as u32,                  // src  = SSIFRDR (fixed)
    0,                                                 // dst  (→ RX_BUF, set at init)
    (RX_BUF_LEN * core::mem::size_of::<i32>()) as u32, // transfer bytes
    0,                                                 // CHCFG (patched at init)
    0,                                                 // CHITVL
    0,                                                 // CHEXT
    0,                                                 // NXLA → &RX_DESC (set at init)
]);

// ── Register helpers ─────────────────────────────────────────────────────────

#[inline(always)]
fn ssi_reg(off: usize) -> *mut u32 {
    (SSI0_BASE + off) as *mut u32
}

// ── Public API ───────────────────────────────────────────────────────────────

/// Initialise SSI0 and start continuous DMA-driven I²S streaming.
///
/// After this call the DMA engine feeds the TX sample buffer into the codec
/// and captures the codec's output into the RX sample buffer, indefinitely.
///
/// **Call order requirements:**
/// 1. `rza1::stb::init()` — enables the SSIF0 module clock.
/// 2. Board audio pin-mux setup (done by `deluge_bsp::audio::init`).
/// 3. `ssi::init()` — this function.
/// 4. 5 ms hardware delay for codec power-on (done by `deluge_bsp::audio::init`).
/// 5. Assert the codec `CODEC_POWER` GPIO (done by `deluge_bsp::audio::init`).
///
/// # Safety
/// Must be called exactly once, from a single-threaded boot context, before
/// any audio task accesses the buffer pointers.
pub unsafe fn init(cfg: &SsiConfig) {
    unsafe {
        TX_DMA_CH_ACTIVE.store(cfg.tx_dma_ch, core::sync::atomic::Ordering::Relaxed);
        RX_DMA_CH_ACTIVE.store(cfg.rx_dma_ch, core::sync::atomic::Ordering::Relaxed);
        log::debug!(
            "ssi: init SSI0 (44.1 kHz I\u{00B2}S, DMA ch{}/{})",
            cfg.tx_dma_ch,
            cfg.rx_dma_ch
        );
        // 1. Patch self-referential fields in the link descriptors.
        //    Write through the uncached alias so the DMAC sees the correct values
        //    without needing a cache flush (DMAC reads physical SRAM directly).
        //    The NXLA pointer must also be the uncached alias address so every
        //    reload of the descriptor by the DMAC is coherent.
        let tx_desc_u =
            (core::ptr::addr_of!(TX_DESC) as usize + UNCACHED_MIRROR_OFFSET) as *mut u32;
        let rx_desc_u =
            (core::ptr::addr_of!(RX_DESC) as usize + UNCACHED_MIRROR_OFFSET) as *mut u32;

        tx_desc_u
            .add(1)
            .write_volatile(core::ptr::addr_of!(TX_BUF.0[0]) as u32);
        tx_desc_u.add(4).write_volatile(tx_chcfg(cfg.tx_dma_ch));
        tx_desc_u.add(7).write_volatile(tx_desc_u as u32);

        rx_desc_u
            .add(2)
            .write_volatile(core::ptr::addr_of!(RX_BUF.0[0]) as u32);
        rx_desc_u.add(4).write_volatile(rx_chcfg(cfg.rx_dma_ch));
        rx_desc_u.add(7).write_volatile(rx_desc_u as u32);

        // 2. SSI0 software reset via CPG SWRSTCR1 bit 6.
        let swrstcr = SWRSTCR1 as *mut u8;
        let reset_bit = 1u8 << (6 - SSI_CH);
        swrstcr.write_volatile(swrstcr.read_volatile() | reset_bit);
        let _ = swrstcr.read_volatile(); // mandatory dummy read
        swrstcr.write_volatile(swrstcr.read_volatile() & !reset_bit);
        let _ = swrstcr.read_volatile(); // mandatory dummy read

        // 3. Configure SSI registers.
        // SSITDMR: CONT=1 (WS continue mode, required for master mode);
        //   TDM=0 (normal stereo, not TDM).
        //   BSP sets SCUX_SSITDMR_CONT_SET when mode_master=true.
        ssi_reg(SSITDMR_OFF).write_volatile(SSITDMR_CONT);
        ssi_reg(SSICR_OFF).write_volatile(cfg.ssicr); // master, 24-bit/32-bit, ÷4
        ssi_reg(SSIFCR_OFF).write_volatile(SSIFCR_INIT); // FIFOs in reset

        // 4. Initialise and connect DMA channels to the link descriptors.
        //    Pass uncached alias addresses so the DMAC can re-fetch them coherently.
        let tx_desc_u =
            (core::ptr::addr_of!(TX_DESC) as usize + UNCACHED_MIRROR_OFFSET) as *const u32;
        let rx_desc_u =
            (core::ptr::addr_of!(RX_DESC) as usize + UNCACHED_MIRROR_OFFSET) as *const u32;
        dmac::init_with_link_descriptor(cfg.tx_dma_ch, tx_desc_u, DMARS_SSI0_TX);
        dmac::init_with_link_descriptor(cfg.rx_dma_ch, rx_desc_u, DMARS_SSI0_RX);

        // 5. Start both DMA channels (software-reset then enable).
        dmac::channel_start(cfg.tx_dma_ch);
        dmac::channel_start(cfg.rx_dma_ch);
        log::debug!(
            "ssi: DMA TX ch{} + RX ch{} started",
            cfg.tx_dma_ch,
            cfg.rx_dma_ch
        );

        // 6. Release the FIFOs from reset and enable TX/RX.
        //    This mirrors `ssiStart()` from the C firmware exactly.
        let fcr = ssi_reg(SSIFCR_OFF);

        // Release TX FIFO from reset, enable TX DMA trigger
        fcr.write_volatile(fcr.read_volatile() & !SSIFCR_TFRST);
        fcr.write_volatile(fcr.read_volatile() | SSIFCR_TIE);
        // Release RX FIFO from reset, enable RX DMA trigger
        fcr.write_volatile(fcr.read_volatile() & !SSIFCR_RFRST);
        fcr.write_volatile(fcr.read_volatile() | SSIFCR_RIE);

        // Enable transmit and receive
        let cr = ssi_reg(SSICR_OFF);
        cr.write_volatile(cr.read_volatile() | SSICR_TEN | SSICR_REN);
        log::debug!("ssi: TX+RX enabled, streaming");
    }
}

/// Initialise SSI0 **receive only** — used when the SCUX DVU path owns the
/// SSIF0 transmitter.
///
/// Performs the same SSI hardware reset and register setup as [`init`], then
/// starts **only** the RX DMA (ch 7) and enables only the RX FIFO and REN.
/// The TX DMA (ch 6), TX FIFO, and TEN bit are left untouched so that the
/// SCUX can drive SSIF0 TX via its own direct-drive path without conflict.
///
/// # Safety
/// Same requirements as [`init`].  Do not call both [`init`] and this
/// function — call exactly one of the two.
pub unsafe fn init_rx_only(cfg: &SsiConfig) {
    unsafe {
        RX_DMA_CH_ACTIVE.store(cfg.rx_dma_ch, core::sync::atomic::Ordering::Relaxed);
        log::debug!(
            "ssi: init SSI0 RX-only (44.1 kHz I\u{00B2}S, DMA ch{} RX, TX owned by SCUX)",
            cfg.rx_dma_ch
        );

        // 1. Patch RX link descriptor only.
        let rx_desc_u =
            (core::ptr::addr_of!(RX_DESC) as usize + UNCACHED_MIRROR_OFFSET) as *mut u32;
        rx_desc_u
            .add(2)
            .write_volatile(core::ptr::addr_of!(RX_BUF.0[0]) as u32);
        rx_desc_u.add(4).write_volatile(rx_chcfg(cfg.rx_dma_ch));
        rx_desc_u.add(7).write_volatile(rx_desc_u as u32);

        // 2. SSI0 software reset.
        let swrstcr = SWRSTCR1 as *mut u8;
        let reset_bit = 1u8 << (6 - SSI_CH);
        swrstcr.write_volatile(swrstcr.read_volatile() | reset_bit);
        let _ = swrstcr.read_volatile();
        swrstcr.write_volatile(swrstcr.read_volatile() & !reset_bit);
        let _ = swrstcr.read_volatile();

        // 3. Configure SSI registers (same as full init).
        ssi_reg(SSITDMR_OFF).write_volatile(SSITDMR_CONT); // WS continue mode (master)
        ssi_reg(SSICR_OFF).write_volatile(cfg.ssicr);
        ssi_reg(SSIFCR_OFF).write_volatile(SSIFCR_INIT); // both FIFOs held in reset

        // 4. Initialise and start RX DMA only.
        let rx_desc_u =
            (core::ptr::addr_of!(RX_DESC) as usize + UNCACHED_MIRROR_OFFSET) as *const u32;
        dmac::init_with_link_descriptor(cfg.rx_dma_ch, rx_desc_u, DMARS_SSI0_RX);
        dmac::channel_start(cfg.rx_dma_ch);
        log::debug!(
            "ssi: DMA RX ch{} started (TX skipped — SCUX owns TX)",
            cfg.rx_dma_ch
        );

        // 5. Release RX FIFO only; leave TX FIFO in reset (TFRST stays set).
        let fcr = ssi_reg(SSIFCR_OFF);
        fcr.write_volatile(fcr.read_volatile() & !SSIFCR_RFRST);
        fcr.write_volatile(fcr.read_volatile() | SSIFCR_RIE);

        // 6. Enable receive only (TEN stays 0 — SCUX owns TX).
        let cr = ssi_reg(SSICR_OFF);
        cr.write_volatile(cr.read_volatile() | SSICR_REN);
        log::debug!("ssi: RX-only enabled, streaming");
    }
}

/// Enable SSI0 transmitter (TEN = bit 1 of SSICR).
///
/// Must be called AFTER [`init_rx_only`] when the SCUX owns the SSI0 TX path.
/// The TRM requires `SSICRn.TEN = 1` when SCUX direct-drive (`SSICTRL.SSI0TX`)
/// is used (SSI012TEN at bit 1 of SSICTRL is for 6-channel mode only and must
/// remain 0 for direct drive).
///
/// Also releases the TX FIFO from reset (clears SSIFCR.TFRST) so that
/// SCUX writes to SSIFTDR can actually propagate into the TX serializer.
/// We clear only TFRST so the existing SSIFCR_RIE (RX DMA trigger) bit is preserved.
///
/// # Safety
/// Read-modify-writes SSI0 SSICR and SSIFCR.
pub unsafe fn enable_tx() {
    unsafe {
        let fcr = ssi_reg(SSIFCR_OFF);
        fcr.write_volatile(fcr.read_volatile() & !SSIFCR_TFRST);
        let cr = ssi_reg(SSICR_OFF);
        cr.write_volatile(cr.read_volatile() | SSICR_TEN);
        log::debug!("ssi: TX enabled (SCUX direct-drive mode), TX FIFO released");
    }
}

// ── Buffer pointer accessors ─────────────────────────────────────────────────
//
// All pointers are into the *uncached* mirror alias so the CPU never sees
// stale cache lines when reading or writing samples that the DMA has touched.

/// Current DMA read position in the TX buffer (uncached), aligned to one
/// stereo frame (8 bytes).
///
/// Write audio samples *ahead of* this pointer.  Wrap at [`tx_buf_end`].
pub fn tx_current_ptr() -> *mut i32 {
    let ch = TX_DMA_CH_ACTIVE.load(core::sync::atomic::Ordering::Relaxed);
    let crsa = unsafe { dmac::current_src(ch) };
    // Align down to a stereo frame (2 × i32 = 8 bytes).
    let aligned = crsa & STEREO_FRAME_ALIGN_MASK;
    (aligned as usize + UNCACHED_MIRROR_OFFSET) as *mut i32
}

/// Current DMA write position in the RX buffer (uncached), aligned to one
/// stereo frame.
///
/// Read received samples *behind* this pointer.  Wrap at [`rx_buf_start`].
pub fn rx_current_ptr() -> *const i32 {
    let ch = RX_DMA_CH_ACTIVE.load(core::sync::atomic::Ordering::Relaxed);
    let crda = unsafe { dmac::current_dst(ch) };
    let aligned = crda & STEREO_FRAME_ALIGN_MASK;
    (aligned as usize + UNCACHED_MIRROR_OFFSET) as *const i32
}

/// Pointer to the first sample in the TX buffer (uncached view).
pub fn tx_buf_start() -> *mut i32 {
    (unsafe { core::ptr::addr_of!(TX_BUF.0[0]) as usize } + UNCACHED_MIRROR_OFFSET) as *mut i32
}

/// One-past-the-end pointer for the TX buffer (uncached view).
pub fn tx_buf_end() -> *mut i32 {
    unsafe { tx_buf_start().add(TX_BUF_LEN) }
}

/// Pointer to the first sample in the RX buffer (uncached view).
pub fn rx_buf_start() -> *const i32 {
    (unsafe { core::ptr::addr_of!(RX_BUF.0[0]) as usize } + UNCACHED_MIRROR_OFFSET) as *const i32
}

/// One-past-the-end pointer for the RX buffer (uncached view).
pub fn rx_buf_end() -> *const i32 {
    unsafe { rx_buf_start().add(RX_BUF_LEN) }
}

/// Returns the DMA channel number assigned to the SSI RX path.
///
/// Valid after [`init`] or [`init_rx_only`] has been called.  Used by the
/// firmware to register a GIC completion handler (DMAINT = 41 + ch).
pub fn rx_dma_ch() -> u8 {
    RX_DMA_CH_ACTIVE.load(core::sync::atomic::Ordering::Relaxed)
}

/// Returns the DMA channel number assigned to the SSI TX path.
///
/// Valid after [`init`] has been called.  Used by the firmware to register a
/// GIC completion handler (DMAINT = 41 + ch).
pub fn tx_dma_ch() -> u8 {
    TX_DMA_CH_ACTIVE.load(core::sync::atomic::Ordering::Relaxed)
}
