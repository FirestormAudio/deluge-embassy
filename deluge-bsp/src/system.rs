//! Deluge board system initialisation.
//!
//! Centralises all board-specific peripheral configuration values and
//! provides [`init_clocks`] as the single boot-time entry point that
//! replaces the direct `rza1l_hal::stb`, `rza1l_hal::mmu`, `rza1l_hal::cache`,
//! `rza1l_hal::gic`, `rza1l_hal::ostm`, and `rza1l_hal::time_driver` calls that would
//! otherwise leak chip-level details into the top-level firmware crate.
//!
//! ## CPG clock-gate values (`StbConfig`)
//!
//! In CPG STBCRn, a **0 bit enables** the module clock and a **1 bit stops**
//! it.  The values below are taken from the original C BSP `STB_Init()`.
//!
//! | Register | Value      | Enabled modules                              |
//! |----------|------------|----------------------------------------------|
//! | STBCR2   | 0b01101010 | CoreSight                                    |
//! | STBCR3   | 0b11110101 | MTU2, PWM                                    |
//! | STBCR4   | 0b00000111 | SCIF0–4                                      |
//! | STBCR5   | 0b11111100 | OSTM0, OSTM1                                 |
//! | STBCR6   | 0b01111111 | RTClock                                      |
//! | STBCR7   | 0b00111101 | DVDEC0/1, USB0                               |
//! | STBCR8   | 0b11111101 | SCUX                                         |
//! | STBCR9   | 0b11110111 | VDC50                                        |
//! | STBCR10  | 0b00011111 | RSPI0–4                                      |
//! | STBCR11  | 0b11011111 | SSIF0                                        |
//! | STBCR12  | 0b11111011 | SDHI0 channel 0                              |

use rza1l_hal::ssi::SsiConfig;
use rza1l_hal::stb::StbConfig;

#[inline(always)]
unsafe fn cpg_basic_init() {
    let power_ctrl = 0x3FFF_FF80 as *mut u32;
    let frqcr = 0xFCFE_0010 as *mut u16;
    let frqcr2 = 0xFCFE_0014 as *mut u16;
    let syscr3 = 0xFCFE_0408 as *mut u8;

    // Matches the known-working Deluge firmware / bootloader CPG_Init path.
    unsafe {
        power_ctrl.write_volatile(0x0000_0001);
        let _ = power_ctrl.read_volatile();
    }

    // PLL(x30), I:G:B:P1:P0 = 30:20:10:5:5/2.
    unsafe {
        frqcr.write_volatile(0x1035);
        let _ = frqcr.read_volatile();
    }

    // CKIO = P1, low during standby.
    unsafe {
        frqcr2.write_volatile(0x0001);
        let _ = frqcr2.read_volatile();
    }

    // Enable writes to the on-chip data-retention RAM banks.
    unsafe {
        syscr3.write_volatile(0x0F);
        let _ = syscr3.read_volatile();
    }
}

// ---------------------------------------------------------------------------
// Board-specific configuration constants
// ---------------------------------------------------------------------------

/// CPG clock-gate configuration for the Deluge board.
///
/// Enables exactly the module clocks required by the Deluge firmware.
/// All other module clocks are left stopped (gated) to minimise power and
/// avoid spurious interrupt sources.
pub const STB_CONFIG: StbConfig = StbConfig {
    // CoreSight enabled; port-level standby for unused bits.
    // `[1][1][0][1][0][1][CoreSight][_]`
    stbcr2: 0b01101010,
    // MTU2 and PWM enabled; IEBus, IrDA, LIN0/1, RSCAN2 stopped.
    // `[IEBus][IrDA][LIN0][LIN1][MTU2][RSCAN2][0][PWM]`
    stbcr3: 0b11110101,
    // SCIF0–4 enabled; reserved bits [2:0] held at 1.
    // `[SCIF0][SCIF1][SCIF2][SCIF3][SCIF4][1][1][1]`
    stbcr4: 0b00000111,
    // OSTM0 and OSTM1 enabled; SCIM0/1 stopped.
    // `[SCIM0][SCIM1][1][1][1][1][OSTM1][OSTM0]`
    stbcr5: 0b11111100,
    // RTClock enabled; A/D, CEU, JCU stopped.
    // `[A/D][CEU][1][1][1][1][JCU][RTClock]`
    stbcr6: 0b01111111,
    // DVDEC0/1 and USB0 enabled; ETHER, FLCTL, USB1 stopped.
    // `[DVDEC0][DVDEC1][1][ETHER][FLCTL][1][USB0][USB1]`
    stbcr7: 0b00111101,
    // SCUX clock enabled; IMR-LS2x, MMCIF, MOST50 stopped.
    // `[IMR-LS20][IMR-LS21][IMR-LSD][MMCIF][MOST50][1][SCUX][1]`
    stbcr8: 0b11111101,
    // VDC50 enabled; I2Cx, SPIBSCx, VDC51 stopped.
    // `[I2C0][I2C1][I2C2][I2C3][SPIBSC0][SPIBSC1][VDC50][VDC51]`
    stbcr9: 0b11110111,
    // RSPI0–4 enabled; CD-ROMDEC, RSPDIF, RGPVG stopped.
    // `[RSPI0][RSPI1][RSPI2][RSPI3][RSPI4][CD-ROM][RSPDIF][RGPVG]`
    stbcr10: 0b00011111,
    // SSIF0 enabled; SSIF1–3 stopped.
    // `[1][1][SSIF0][SSIF1][SSIF2][SSIF3][reserved][reserved]`
    stbcr11: 0b11011111,
    // SDHI0 channel 0 enabled (port 0, pins 0+1); SDHI1, SDHI0 ch1 stopped.
    // `[1][1][1][1][SDHI00][SDHI01][SDHI10][SDHI11]`
    stbcr12: 0b11111011,
};

/// SSI0 configuration for the Deluge board.
///
/// - SSICR value 0x002B_C020: AUDIO_X1 clock source, stereo 24-bit/32-bit
///   system word, RZ/A1L as I²S master, BCLK = AUDIO_X1 ÷ 4 ≈ 5.6448 MHz.
/// - TX DMA channel 6 (SRAM → SSIFTDR).
/// - RX DMA channel 7 (SSIFRDR → SRAM).
pub const SSI_CONFIG: SsiConfig = SsiConfig {
    ssicr: 0x002B_C020,
    tx_dma_ch: 6,
    rx_dma_ch: 7,
};

/// SD_OPTION register value for SDHI port 1 on the Deluge board.
///
/// 0x00BD selects a 2^23 SDCLK timeout cycle count and 4-bit bus width.
/// This value was tuned by Rohan for reliable card detection on the Deluge
/// hardware; it differs from the SDHI reset default.
pub const SD_OPTION: u16 = 0x00BD;

// ---------------------------------------------------------------------------
// SCUX DMA channel assignments for the Deluge board
// ---------------------------------------------------------------------------
//
// These replace the previously-public rza1l_hal::scux::FFD*_DMA_CH constants.
// Pass them as the `dma_ch` argument to `rza1l_hal::scux::init_ffd_dma` and
// `rza1l_hal::scux::init_ffu_dma`.

/// DMA channel for FFD path 0 (CPU → SCUX, 8-ch, main synthesis / DVU path).
pub const SCUX_FFD0_DMA_CH: u8 = 0;
/// DMA channel for FFD path 1 (CPU → SCUX, 8-ch, sample playback / SRC path).
pub const SCUX_FFD1_DMA_CH: u8 = 3;
/// DMA channel for FFD path 2 (CPU → SCUX, 2-ch stereo aux path).
pub const SCUX_FFD2_DMA_CH: u8 = 2;
/// DMA channel for FFD path 3 (CPU → SCUX, 2-ch stereo aux path).
/// Channel 15 is used to avoid the collision with OLED DMA on channel 4.
pub const SCUX_FFD3_DMA_CH: u8 = 15;
/// DMA channel for FFU path 0 (SCUX → CPU, 8-ch, main capture / SRC path).
pub const SCUX_FFU0_DMA_CH: u8 = 1;
/// DMA channel for FFU path 1 (SCUX → CPU, 8-ch, capture path).
pub const SCUX_FFU1_DMA_CH: u8 = 5;
/// DMA channel for FFU path 2 (SCUX → CPU, 2-ch stereo aux path).
pub const SCUX_FFU2_DMA_CH: u8 = 8;
/// DMA channel for FFU path 3 (SCUX → CPU, 2-ch stereo aux path).
pub const SCUX_FFU3_DMA_CH: u8 = 9;

// ---------------------------------------------------------------------------
// UART (SCIF) DMA channel assignments
// ---------------------------------------------------------------------------
//
// SCIF0 = MIDI DIN; SCIF1 = PIC32 co-processor.
// DMARS values are fixed by the RZ/A1L hardware (SCIF0 RX=0x62, TX=0x61;
// each subsequent SCIF adds 4: SCIF1 RX=0x66, TX=0x65).

/// DMAC channel for SCIF0 (MIDI) receive ring buffer.
pub const MIDI_DMA_RX_CH: u8 = 13;
/// DMAC channel for SCIF1 (PIC32) receive ring buffer.
pub const PIC_DMA_RX_CH: u8 = 12;
/// DMAC channel for SCIF1 (PIC32) transmit.
pub const PIC_DMA_TX_CH: u8 = 10;

// ---------------------------------------------------------------------------
// SD-card DMA channel assignments (SDHI port 1)
// ---------------------------------------------------------------------------

/// DMAC channel for SDHI1 transmit (memory → SD_BUF0; card writes).
pub const SD_DMA_TX_CH: u8 = 11;
/// DMAC channel for SDHI1 receive (SD_BUF0 → memory; card reads).
pub const SD_DMA_RX_CH: u8 = 14;
/// Maximum sectors in a single DMA transfer; determines the bounce-buffer size.
pub const SD_DMA_MAX_SECTORS: usize = 128;

// ---------------------------------------------------------------------------
// Boot-time system initialisation
// ---------------------------------------------------------------------------

/// Initialise all chip-level and board-level peripherals required before
/// the async executor starts.
///
/// Performs, in order:
/// 1. Basic CPG init (matches DelugeFirmware `CPG_Init`).
/// 2. CPG module clock enable ([`rza1l_hal::stb::init`] with [`STB_CONFIG`]).
/// 3. GIC init ([`rza1l_hal::gic::init`]) to match working `resetprg()` order.
/// 4. MMU enable ([`rza1l_hal::mmu::init_and_enable`]).
/// 5. L1 cache enable ([`rza1l_hal::cache::l1_enable`]).
/// 6. L2 cache init ([`rza1l_hal::cache::l2_init`]).
/// 7. SDRAM init ([`crate::sdram::init`]).
/// 8. OSTM clock enable ([`rza1l_hal::ostm::enable_clock`]).
/// 9. Embassy time-driver init ([`rza1l_hal::time_driver::init`]).
///
/// After this call:
/// - All module clocks for peripherals used by the Deluge firmware are on.
/// - The MMU is active with the default page table.
/// - The L1/L2 caches are active.
/// - The 64 MB SDRAM at 0x0C00_0000–0x0FFF_FFFF is accessible.
/// - The GIC is initialised and ready to accept `gic::register` calls.
/// - OSTM0/1 clocks are running; the time-driver tick source is live.
///
/// The SRAM and SDRAM heap allocators must still be individually
/// initialised by the caller (they depend on linker-script symbol addresses
/// that cannot be embedded here).
///
/// # Safety
/// Writes to memory-mapped hardware registers.  Must be called exactly once,
/// from a single-threaded boot context, before interrupts are enabled and
/// before any async task starts.
#[cfg(target_os = "none")]
pub unsafe fn init_clocks() {
    unsafe {
        cpg_basic_init();
        rza1l_hal::stb::init(&STB_CONFIG);
        rza1l_hal::gic::init();
        rza1l_hal::mmu::init_and_enable();
        rza1l_hal::cache::l1_enable();
        rza1l_hal::cache::l2_init();
        crate::sdram::init();
        rza1l_hal::ostm::enable_clock();
        rza1l_hal::time_driver::init();
    }
}
