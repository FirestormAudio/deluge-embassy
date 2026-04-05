//! Sample Rate Conversion Unit (SCUX) driver for the RZ/A1L.
//!
//! The SCUX contains five functional blocks that can be combined into various
//! audio processing pipelines:
//!
//! ```text
//!                                                              ┌──► FFU0-3 ──► CPU/DMA
//! CPU/DMA ──► FFD0-3 ──► IPC0-3 ──► 2SRC0-3 ──► OPC0-3 ──────┤
//!                                                              └──► DVU0-3 ──► MIX ──► SSIF
//! ```
//!
//! ## Block summary
//! | Block | Instances | Purpose |
//! |-------|-----------|---------|
//! | FFD   | 4         | FIFO download — DMA from SRAM into SCUX input path |
//! | IPC   | 4         | Input path control — routing and format selection |
//! | 2SRC  | 2 units × 2 ch = 4 paths | Asynchronous / synchronous sample-rate conversion |
//! | DVU   | 4         | Digital volume unit — per-channel gain, ramp, zero-cross mute |
//! | MIX   | 1         | Mixer — combines up to 4 DVU outputs |
//! | OPC   | 4         | Output path control — routing to FFU or SSIF |
//! | FFU   | 4         | FIFO upload — DMA from SCUX output path to SRAM |
//! | CIM   | 1         | Common interface — DMA routing, SSIF clocking, reset |
//!
//! ## Base addresses
//! | Block | Base       |
//! |-------|------------|
//! | SCUX  | 0xE820_8000 |
//! | IPC   | 0xE820_8000 (stride 0x100, ×4) |
//! | OPC   | 0xE820_8400 (stride 0x100, ×4) |
//! | FFD   | 0xE820_8800 (stride 0x100, ×4) |
//! | FFU   | 0xE820_8C00 (stride 0x100, ×4) |
//! | 2SRC  | 0xE820_9000 (2 units, each 2×SRC at stride 0x34) |
//! | DVU   | 0xE820_9200 (stride 0x100, ×4) |
//! | MIX   | 0xE820_9600 |
//! | CIM   | 0xE820_9700 |
//!
//! ## DMA resource selectors (DMARS)
//! | Channel | DMARS value | Description |
//! |---------|-------------|-------------|
//! | FFD0_0  | 0x0101      | SCUTXI0 — CPU → SCUX input FIFO 0 |
//! | FFD0_1  | 0x0103      | SCUTXI1 — CPU → SCUX input FIFO 1 |
//! | FFU0_0  | 0x0102      | SCURXI0 — SCUX output FIFO 0 → CPU |
//! | FFU0_1  | 0x0104      | SCURXI1 — SCUX output FIFO 1 → CPU |
//!
//! ## DMA channel allocation (Deluge)
//! | DMA ch | Direction        | SCUX path    | Max ch |
//! |--------|-----------------|--------------|--------|
//! | 0      | SRAM → FFD0_0   | DMATD0_CIM   | 8      |
//! | 1      | FFU0_0 → SRAM   | DMATU0_CIM   | 8      |
//! | 2      | SRAM → FFD0_2   | DMATD2_CIM   | 2      |
//! | 3      | SRAM → FFD0_1   | DMATD1_CIM   | 8      |
//! | 4      | SRAM → FFD0_3   | DMATD3_CIM   | 2      |
//! | 5      | FFU0_1 → SRAM   | DMATU1_CIM   | 8      |
//! | 8      | FFU0_2 → SRAM   | DMATU2_CIM   | 2      |
//! | 9      | FFU0_3 → SRAM   | DMATU3_CIM   | 2      |

use crate::dmac;
use crate::dmac::{
    CHCFG_AM_BURST, CHCFG_DAD, CHCFG_DDS_32BIT, CHCFG_DEM, CHCFG_DMS, CHCFG_HIEN, CHCFG_LVL,
    CHCFG_REQD, CHCFG_SAD, CHCFG_SDS_32BIT,
};

// ── Uncached mirror (must match ssi.rs) ──────────────────────────────────────

/// Add to any internal-SRAM cached VA to get its uncached alias.
// UNCACHED_MIRROR_OFFSET is defined at crate root (crate::UNCACHED_MIRROR_OFFSET).
use crate::UNCACHED_MIRROR_OFFSET;

// ── SCUX top-level base ───────────────────────────────────────────────────────

const SCUX_BASE: usize = 0xE820_8000;

// ── IPC block (×4, stride 0x100) ─────────────────────────────────────────────

const IPC_BASE: usize = SCUX_BASE;
const IPC_STRIDE: usize = 0x100;

const IPCIR_OFF: usize = 0x00; // Init register (bit 0 = INIT)
const IPSLR_OFF: usize = 0x04; // Pass-select register

#[inline(always)]
fn ipc(ch: u8, off: usize) -> *mut u32 {
    (IPC_BASE + ch as usize * IPC_STRIDE + off) as *mut u32
}

// ── OPC block (×4, stride 0x100) ─────────────────────────────────────────────

const OPC_BASE: usize = SCUX_BASE + 0x0400;
const OPC_STRIDE: usize = 0x100;

const OPCIR_OFF: usize = 0x00; // Init register
const OPSLR_OFF: usize = 0x04; // Pass-select register

#[inline(always)]
fn opc(ch: u8, off: usize) -> *mut u32 {
    (OPC_BASE + ch as usize * OPC_STRIDE + off) as *mut u32
}

// ── FFD block (×4, stride 0x100) ─────────────────────────────────────────────

const FFD_BASE: usize = SCUX_BASE + 0x0800;
const FFD_STRIDE: usize = 0x100;

const FFDIR_OFF: usize = 0x00; // Init register (bit 0 = INIT)
const FDAIR_OFF: usize = 0x04; // Audio info register (channels, bit depth)
const DRQSR_OFF: usize = 0x08; // DMA request size register
const FFDPR_OFF: usize = 0x0C; // FIFO data pass register
const FFDBR_OFF: usize = 0x10; // Boot register (bit 0 = BOOT)
const DEVMR_OFF: usize = 0x14; // DMA event mode register (0 = DMA trigger, 1 = interrupt)
const DEVCR_OFF: usize = 0x1C; // DMA event clear register

#[inline(always)]
fn ffd(ch: u8, off: usize) -> *mut u32 {
    (FFD_BASE + ch as usize * FFD_STRIDE + off) as *mut u32
}

// ── FFU block (×4, stride 0x100) ─────────────────────────────────────────────

const FFU_BASE: usize = SCUX_BASE + 0x0C00;
const FFU_STRIDE: usize = 0x100;

const FFUIR_OFF: usize = 0x00; // Init register
const FUAIR_OFF: usize = 0x04; // Audio info register
const URQSR_OFF: usize = 0x08; // DMA request size register
const FFUPR_OFF: usize = 0x0C; // FIFO data pass register
const UEVMR_OFF: usize = 0x10; // DMA event mode register (0 = DMA trigger, 1 = interrupt)
const UEVCR_OFF: usize = 0x18; // DMA event clear register

#[inline(always)]
fn ffu(ch: u8, off: usize) -> *mut u32 {
    (FFU_BASE + ch as usize * FFU_STRIDE + off) as *mut u32
}

// ── 2SRC block ───────────────────────────────────────────────────────────────
//
// Two SRC units (unit 0 and unit 1), each containing two SRC paths (pair 0
// and pair 1).  The two paths within a unit share one BFSSR (buffer size
// register) and one SRCCR (common config), but have independent SADIR, IFSCR,
// IFSVR, MNFSR, SEVMR, and SRCIR / SRCIRR registers.
//
// Base of unit 0, pair 0 = 0xE820_9000.  Stride between pairs = 0x34.
// Unit 1 begins at 0xE820_9080.

const SRC_BASE: usize = 0xE820_9000;
const SRC_UNIT_STRIDE: usize = 0x80;
const SRC_PAIR_STRIDE: usize = 0x34;

const SRCIR_OFF: usize = 0x00; // Init register
const SADIR_OFF: usize = 0x04; // Audio input direction (channels, bit depth)
const SRCBR_OFF: usize = 0x08; // Bypass register (bit 0 = BYPASS)
const IFSCR_OFF: usize = 0x0C; // Input frequency select (0=sync, 1=async)
const IFSVR_OFF: usize = 0x10; // Input frequency value (INTIFS ratio Q22)
const SRCCR_OFF: usize = 0x14; // Common control (must-be-1 bits 16,8,4)
const MNFSR_OFF: usize = 0x18; // Minimum frequency select
const BFSSR_OFF: usize = 0x1C; // Buffer size select
// 0x20: SC2SR (status, read-only)
// 0x24: WATSR (wait time)
const SEVMR_OFF: usize = 0x28; // Sampling event mode register
// 0x2C: 4-byte reserved gap
const SEVCR_OFF: usize = 0x30; // Sampling event clear register
// SRCIRR lives at unit_base + 0x68 (after both pairs); see srcirr() below.
const SRCIRR_UNIT_OFF: usize = 0x68;

#[inline(always)]
fn src(unit: u8, pair: u8, off: usize) -> *mut u32 {
    (SRC_BASE + unit as usize * SRC_UNIT_STRIDE + pair as usize * SRC_PAIR_STRIDE + off) as *mut u32
}

/// Accessor for SRCIRR (Input-Rate Reload init register), which is per-unit
/// and sits after both pair register banks, at unit_base + 0x68.
#[inline(always)]
fn srcirr(unit: u8) -> *mut u32 {
    (SRC_BASE + unit as usize * SRC_UNIT_STRIDE + SRCIRR_UNIT_OFF) as *mut u32
}

// ── DVU block (×4, stride 0x100) ─────────────────────────────────────────────

const DVU_BASE: usize = 0xE820_9200;
const DVU_STRIDE: usize = 0x100;

const DVUIR_OFF: usize = 0x00; // Init register
const VADIR_OFF: usize = 0x04; // Audio direction (channels, bit depth)
const DVUBR_OFF: usize = 0x08; // Bypass register (bit 0 = BYPASS)
const DVUCR_OFF: usize = 0x0C; // Control (enable flags)
const ZCMCR_OFF: usize = 0x10; // Zero-cross mute control
const VRCTR_OFF: usize = 0x14; // Volume ramp control (bit 0 = enable ramp)
const VRPDR_OFF: usize = 0x18; // Volume ramp period
const VRDBR_OFF: usize = 0x1C; // Volume ramp dB step
const VRWTR_OFF: usize = 0x20; // Volume ramp wait time register
const VOL0R_OFF: usize = 0x24; // Per-channel volume registers (ch 0–7)
const DVUER_OFF: usize = 0x44; // DVU enable register (bit 0 = EN)
const VEVMR_OFF: usize = 0x4C; // Volume event mode
const VEVCR_OFF: usize = 0x54; // Volume event clear (0x50 is a 4-byte gap)

#[inline(always)]
fn dvu(ch: u8, off: usize) -> *mut u32 {
    (DVU_BASE + ch as usize * DVU_STRIDE + off) as *mut u32
}

/// Offset of VOL register for a specific audio sub-channel within a DVU instance.
#[inline(always)]
fn vol_off(audio_ch: u8) -> usize {
    VOL0R_OFF + audio_ch as usize * 4
}

// ── MIX block ────────────────────────────────────────────────────────────────

const MIX_BASE: usize = 0xE820_9600;

const MIXIR_OFF: usize = 0x00; // Init register
const MADIR_OFF: usize = 0x04; // Audio direction
const MIXBR_OFF: usize = 0x08; // Bypass register (bit 0 = BYPASS)
const MIXMR_OFF: usize = 0x0C; // Mix mode register
const MVPDR_OFF: usize = 0x10; // Master volume period
const MDB0R_OFF: usize = 0x14; // Mix data buffer 0 (per-source gain)
const MDB1R_OFF: usize = 0x18;
const MDB2R_OFF: usize = 0x1C;
const MDB3R_OFF: usize = 0x20;
const MDBER_OFF: usize = 0x24; // Mix data buffer enable

#[inline(always)]
fn mix(off: usize) -> *mut u32 {
    (MIX_BASE + off) as *mut u32
}

// ── CIM block ─────────────────────────────────────────────────────────────────

const CIM_BASE: usize = 0xE820_9700;

const SWRSR_CIM_OFF: usize = 0x00; // Software reset (bit 0; 0=reset, 1=run)
const DMACR_CIM_OFF: usize = 0x04; // DMA enable (bit 0=FFD0 TX, bit 1=FFD1, bit 16=FFU0 RX, …)
const DMATD0_CIM_OFF: usize = 0x08; // DMA transfer register: DMA channel number for FFD0
const DMATD1_CIM_OFF: usize = 0x0C; // DMA channel number for FFD1
const DMATD2_CIM_OFF: usize = 0x10;
const DMATD3_CIM_OFF: usize = 0x14;
const DMATU0_CIM_OFF: usize = 0x18; // DMA transfer register: DMA channel number for FFU0
const DMATU1_CIM_OFF: usize = 0x1C;
const DMATU2_CIM_OFF: usize = 0x20;
const DMATU3_CIM_OFF: usize = 0x24;
// 0x28–0x37: 16-byte reserved gap (TRM Table 37.2, between DMATUn and SSIRSEL)
const SSIRSEL_CIM_OFF: usize = 0x38; // SSI→SRC route selector
const FDTSEL0_CIM_OFF: usize = 0x3C; // FFD→SRC/SSI selector for FFD0
const FDTSEL1_CIM_OFF: usize = 0x40;
const FDTSEL2_CIM_OFF: usize = 0x44;
const FDTSEL3_CIM_OFF: usize = 0x48;
const FUTSEL0_CIM_OFF: usize = 0x4C; // FFU source selector for FFU0
const FUTSEL1_CIM_OFF: usize = 0x50;
const FUTSEL2_CIM_OFF: usize = 0x54;
const FUTSEL3_CIM_OFF: usize = 0x58;
const SSIPMD_CIM_OFF: usize = 0x5C; // SSI port mode
const SSICTRL_CIM_OFF: usize = 0x60; // SSI clock/gate control
const SRCRSEL0_CIM_OFF: usize = 0x64; // SRC route select 0 (reset = 0x76543210)
const SRCRSEL1_CIM_OFF: usize = 0x68;
const SRCRSEL2_CIM_OFF: usize = 0x6C;
const SRCRSEL3_CIM_OFF: usize = 0x70;
const MIXRSEL_CIM_OFF: usize = 0x74; // MIX input route selector (reset = 0x76543210)

#[inline(always)]
fn cim(off: usize) -> *mut u32 {
    (CIM_BASE + off) as *mut u32
}

// ── DMARS constants ───────────────────────────────────────────────────────────

/// DMARS resource-selector for SCUX input FIFO 0 TX (CPU → FFD0_0).
pub const DMARS_SCUTXI0: u32 = 0x0101;
/// DMARS resource-selector for SCUX input FIFO 1 TX (CPU → FFD0_1).
/// BSP: DMA_RS_SCUTXI1 = 0x105
pub const DMARS_SCUTXI1: u32 = 0x0105;
/// DMARS resource-selector for SCUX input FIFO 2 TX (CPU → FFD0_2, 2-ch path).
pub const DMARS_SCUTXI2: u32 = 0x0109;
/// DMARS resource-selector for SCUX input FIFO 3 TX (CPU → FFD0_3, 2-ch path).
pub const DMARS_SCUTXI3: u32 = 0x010D;
/// DMARS resource-selector for SCUX output FIFO 0 RX (FFU0_0 → CPU).
pub const DMARS_SCURXI0: u32 = 0x0102;
/// DMARS resource-selector for SCUX output FIFO 1 RX (FFU0_1 → CPU).
/// BSP: DMA_RS_SCURXI1 = 0x106
pub const DMARS_SCURXI1: u32 = 0x0106;
/// DMARS resource-selector for SCUX output FIFO 2 RX (FFU0_2 → CPU, 2-ch path).
pub const DMARS_SCURXI2: u32 = 0x010A;
/// DMARS resource-selector for SCUX output FIFO 3 RX (FFU0_3 → CPU, 2-ch path).
pub const DMARS_SCURXI3: u32 = 0x010E;

// ── DMA channel assignments (Deluge) — moved to deluge-bsp ───────────────────
//
// The DMA channels previously declared here as FFD*_DMA_CH / FFU*_DMA_CH
// public constants are board-specific and are now provided by the caller via
// the `dma_ch` parameter of [`init_ffd_dma`] and [`init_ffu_dma`].
// The Deluge values can be found in `deluge_bsp::system`.

/// Link-descriptor HEADER word (same field encoding as SSI — see ssi.rs `DESC_HEADER`).
/// bits: LDEN(3)=1, NXA(2)=1, reserved(1)=0, valid(0)=1 → 0b1101.
const DESC_HEADER: u32 = 0b1101;

// CHCFG for SCUX FFD DMA (SRAM → SCUX FIFO, source increments, destination fixed):
//   DMS | DEM | DAD | DDS_32BIT | SDS_32BIT | AM_BURST | HIEN | REQD | LVL | channel
//   = 0x8000_0000 | 0x0100_0000 | 0x0020_0000 | 0x0002_0000 | 0x0000_2000
//     | 0x0000_0200 | 0x0000_0020 | 0x0000_0008 | 0x0000_0040 | ch
//   = 0x8122_2268 | ch  (matches C BSP scux_dev.c)
const fn ffd_chcfg(dma_ch: u8) -> u32 {
    CHCFG_DMS
        | CHCFG_DEM
        | CHCFG_DAD
        | CHCFG_DDS_32BIT
        | CHCFG_SDS_32BIT
        | CHCFG_AM_BURST
        | CHCFG_HIEN
        | CHCFG_REQD
        | CHCFG_LVL
        | (dma_ch as u32 & 7)
}

// CHCFG for SCUX FFU DMA (SCUX FIFO → SRAM, source fixed, destination increments):
//   DMS | DEM | SAD | DDS_32BIT | SDS_32BIT | AM_BURST | HIEN | LVL | channel
//   = 0x8000_0000 | 0x0100_0000 | 0x0010_0000 | 0x0002_0000 | 0x0000_2000
//     | 0x0000_0200 | 0x0000_0020 | 0x0000_0040 | ch
//   = 0x8112_2260 | ch  (matches C BSP)
const fn ffu_chcfg(dma_ch: u8) -> u32 {
    CHCFG_DMS
        | CHCFG_DEM
        | CHCFG_SAD
        | CHCFG_DDS_32BIT
        | CHCFG_SDS_32BIT
        | CHCFG_AM_BURST
        | CHCFG_HIEN
        | CHCFG_LVL
        | (dma_ch as u32 & 7)
}

// ── INTIFS helper constants (Q22 ratio: fin/fout * 2^22) ─────────────────────

/// Interpolation ratio for 44.1 kHz → 44.1 kHz (unity, passthrough mode).
pub const INTIFS_44100_TO_44100: u32 = 0x0400_0000;
/// Interpolation ratio for 44.1 kHz → 48 kHz.
pub const INTIFS_44100_TO_48000: u32 = 0x03AC_CCCC;
/// Interpolation ratio for 48 kHz → 44.1 kHz.
pub const INTIFS_48000_TO_44100: u32 = 0x045A_8EC0;
/// Interpolation ratio for 48 kHz → 48 kHz (unity).
pub const INTIFS_48000_TO_48000: u32 = 0x0400_0000;
/// Interpolation ratio for 88.2 kHz → 44.1 kHz (exactly 2:1).
/// Use when CPU synthesises at 88.2 kHz for higher quality, output at 44.1 kHz.
pub const INTIFS_88200_TO_44100: u32 = intifs(88200, 44100);
/// Interpolation ratio for 96 kHz → 44.1 kHz.
/// Use when CPU synthesises at 96 kHz, output at 44.1 kHz.
pub const INTIFS_96000_TO_44100: u32 = intifs(96000, 44100);
/// Interpolation ratio for 96 kHz → 48 kHz (exactly 2:1).
pub const INTIFS_96000_TO_48000: u32 = intifs(96000, 48000);

/// Compute an arbitrary INTIFS ratio value for any integer input/output rate.
///
/// The hardware expects `(fin / fout) * 2^22` as a fixed-point Q22 value.
/// Returns 0 if `fout_hz` is zero.
pub const fn intifs(fin_hz: u32, fout_hz: u32) -> u32 {
    if fout_hz == 0 {
        return 0;
    }
    (((fin_hz as u64) << 22) / fout_hz as u64) as u32
}

// ── SRCCR must-be-1 bits ─────────────────────────────────────────────────────

/// SRCCR must-be-1 bits (bits 16, 8, and 4 per TRM §37 note 1).
/// BSP: SRCCR_2SRC0_BASE_VALUE = 0x00010110U
const SRCCR_MBZ1: u32 = (1 << 16) | (1 << 8) | (1 << 4);
/// SRCCR bit 0 = SRCMD: 0 = async mode, 1 = sync mode.
/// BSP: SRCCR_2SRC0_SRCMD_SET = (1U << 0)
const SRCCR_SYNC: u32 = 1 << 0;

// ── Sub-block register bit constants (verified vs vendor/rbsp scux.h) ─────────

/// SWRSR bit 0: 0 = reset asserted, 1 = normal operation.
const SWRSR_RESET: u32 = 0;
const SWRSR_RUN: u32 = 1;

/// Generic INIT bit (bit 0) for all xxxIR registers (FFD/FFU/SRC/DVU/MIX/IPC/OPC).
/// Writing 1 asserts init (stops block); writing 0 releases init (starts block).
const INIT_SET: u32 = 1; // BSP: FFDIR_FFD0_INIT_SET etc.
const INIT_CLR: u32 = 0;

/// FFDBR bit 0: set 1 after clearing FFDIR.INIT to boot the FIFO.
/// BSP: FFDBR_FFD0_BOOT_SET = (1U << 0)
const FFDBR_BOOT: u32 = 1 << 0;

/// FFDPR bit 0: 1 = enable async data path (CIM→FFD→IPC).
/// BSP: FFDPR_FFD0_PASS_SET_ASYNC = (1U << 0)
const FFDPR_PASS_ASYNC: u32 = 1 << 0;

/// FFUPR bit 0: 1 = enable async data path (OPC→FFU→CIM).
/// BSP: FFUPR_FFU0_PASS_SET_ASYNC = (1U << 0)
const FFUPR_PASS_ASYNC: u32 = 1 << 0;

/// SRCBR bit 0: 1 = bypass SRC (audio passes unchanged).
/// BSP: SRCBR_2SRC0_BYPASS_SET = (0x1U << 0)
const SRCBR_BYPASS: u32 = 1 << 0;

/// IFSCR bit 0: 1 = use INTIFS (async mode), 0 = sync mode.
/// BSP: IFSCR_2SRC0_INTIFSEN_SET = (0x1U << 0)
const IFSCR_INTIFSEN: u32 = 1 << 0;

/// DVUBR bit 0: 1 = bypass DVU.
/// BSP: DVUBR_DVU0_BYPASS_SET = (0x1U << 0)
const DVUBR_BYPASS: u32 = 1 << 0;

/// DVUCR bit 4: enable volume ramp mode (VRMD).
/// BSP: DVUCR_DVU0_VRMD_SET = (1U << 4)
/// The BSP ALWAYS sets this when DVU is enabled (not bypassed), even with
/// zero-rate "dummy" ramp parameters (VRPDR=0, VRDBR=0).  This appears to
/// be mandatory to open the volume-control data path through the DVU.
const DVUCR_VRMD: u32 = 1 << 4;

/// DVUCR bit 8: enable direct digital volume mode (VVMD).
/// BSP: DVUCR_DVU0_VVMD_SET = (1U << 8)
const DVUCR_VVMD: u32 = 1 << 8;

/// VRCTR bit 0: per-channel volume ramp enable (bit N = channel N).
/// BSP: VRCTR_DVU0_VREN_SET = (1U << 0)
const VRCTR_VREN: u32 = 1 << 0;

/// ZCMCR bit 0: per-channel zero-cross mute enable (bit N = channel N).
/// BSP: ZCMCR_DVU0_ZCEN_SET = (1U << 0)
const ZCMCR_ZCEN: u32 = 1 << 0;

/// DVUER bit 0: activate DVU volume register settings.
/// Must be written AFTER DVUIR.INIT is cleared.
/// BSP: DVUER_DVU0_DVUEN_SET = (1U << 0)
const DVUER_DVUEN: u32 = 1 << 0;

/// MIXBR bit 0: 1 = bypass MIX (first source passes through).
const MIXBR_BYPASS: u32 = 1 << 0;

/// MDBER bit 0: enable MIX data buffer (enables per-source gain coefficients).
/// BSP: MDBER_MIX0_MIXDBEN_SET = (1U << 0)
const MDBER_MIXDBEN: u32 = 1 << 0;

/// FDTSEL/FUTSEL bit 8: enable divided clock output (DIVEN).
/// BSP: FDTSEL_CIM_DIVEN_SET = (1U << 8)
pub const FDTSEL_DIVEN: u32 = 1 << 8;

/// FDTSEL/FUTSEL SCKSEL value for SSIF0 WS (bits [3:0] = 8).
/// BSP: FDTSEL_CIM_SCKSEL_SSIF0_WS_SET = (8U)
pub const FDTSEL_SCKSEL_SSIF0_WS: u32 = 8;

/// SSICTRL bit 14: SCUX drives SSIF0 TX directly.
/// BSP: SSICTRL_CIM_SSI0TX_SET = (1U << 14)
pub const SSICTRL_SSI0TX: u32 = 1 << 14;

// ── Configuration types ───────────────────────────────────────────────────────

/// Bit depth of audio samples.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BitDepth {
    /// 24-bit samples (OTBL = 0b00000).
    B24,
    /// 16-bit samples (OTBL = 0b00110).
    B16,
}

impl BitDepth {
    /// OTBL field value (written to bits [20:16] of SADIR/VADIR).
    /// BSP: SADIR_2SRC0_OTBL_SET_24BIT = (0x0U << 16), OTBL_SET_16BIT = (0x8U << 16)
    fn otbl(self) -> u32 {
        match self {
            BitDepth::B24 => 0x0, // = SADIR_2SRC0_OTBL_SET_24BIT value
            BitDepth::B16 => 0x8, // = SADIR_2SRC0_OTBL_SET_16BIT value
        }
    }
}

/// Audio channel/bit-depth descriptor written into FDAIR / FUAIR / SADIR / VADIR / MADIR.
///
/// Encodes as: `channels | (otbl << 16)` per the BSP convention.
/// CHNUM is the actual channel count (1=mono, 2=stereo), not count−1.
/// OTBL field occupies bits [20:16] in SADIR/VADIR (FDAIR/FUAIR/MADIR have no OTBL field).
/// For 24-bit audio, OTBL = 0 so the shift position is moot, but is kept correct.
#[derive(Copy, Clone, Debug)]
pub struct AudioInfo {
    /// Number of audio channels (1–8).
    pub channels: u8,
    /// Sample bit depth.
    pub depth: BitDepth,
}

impl AudioInfo {
    /// Stereo (2-channel) 24-bit audio — the most common configuration.
    pub const STEREO_24: AudioInfo = AudioInfo {
        channels: 2,
        depth: BitDepth::B24,
    };

    fn to_reg(self) -> u32 {
        (self.depth.otbl() << 16) | (self.channels as u32)
    }
}

/// 2SRC operating mode.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SrcMode {
    /// Asynchronous — SCUX generates an internal reference; input and output
    /// clocks are independent.  Use for CPU→SRAM pipelines.
    Async,
    /// Synchronous — SCUX uses an external (SSI) clock reference.  The
    /// interpolation ratio is computed from INTIFS.
    Sync,
}

/// Configuration for one 2SRC path.
#[derive(Copy, Clone, Debug)]
pub struct SrcConfig {
    /// Async or sync mode.
    pub mode: SrcMode,
    /// Audio input format fed into the SRC.
    pub audio: AudioInfo,
    /// If true, bypass the rate converter entirely — audio passes through
    /// unchanged with no clock-domain dependency.  When true, `mode`,
    /// `intifs`, `mnfsr`, and `buf_size` are ignored.
    pub bypass: bool,
    /// Interpolation ratio (INTIFS).  Use the [`intifs`] function or the
    /// `INTIFS_*` constants.  For sync passthrough use [`INTIFS_44100_TO_44100`].
    pub intifs: u32,
    /// Minimum frequency register (MNFSR).  Set to `intifs * 2 >> 1` for
    /// safety margin; 0 disables the minimum frequency check.
    pub mnfsr: u32,
    /// Buffer size select (BFSSR bits [1:0]).  0 = 256 samples (recommended
    /// for most use cases).
    pub buf_size: u32,
}

/// Volume ramp configuration for the DVU hardware fade engine.
#[derive(Copy, Clone, Debug)]
pub struct RampConfig {
    /// Ramp speed in VRPDR units (hardware step interval register value).
    /// Consult TRM §37.9.6.  A value of 0x0F corresponds to ~1 ms per step.
    pub vrpdr: u32,
    /// Ramp step size in VRDBR units (dB-per-step register value).
    /// 0 = minimum step (≈ 0.0078 dB), 0xFF = maximum step.
    pub vrdbr: u32,
}

/// Configuration for one DVU instance.
#[derive(Copy, Clone, Debug)]
pub struct DvuConfig {
    /// Audio format passing through this DVU instance.
    pub audio: AudioInfo,
    /// If true, all DVU processing is bypassed (audio passes unchanged).
    /// When bypassed, `volumes` and `ramp` are ignored.
    pub bypass: bool,
    /// Per-audio-channel volume in hardware units.
    ///
    /// 0 = mute, 0x0010_0000 = 0 dB (unity = 1.0 in 4.20 fixed-point), values above unity boost up to
    /// approximately +18 dB.  Applies only when `bypass == false`.
    pub volumes: [u32; 8],
    /// Hardware volume ramp configuration.  `None` = instant volume change.
    pub ramp: Option<RampConfig>,
    /// Enable zero-cross mute suppression (eliminates clicks on abrupt mutes).
    pub zero_cross_mute: bool,
}

impl DvuConfig {
    /// Bypass mode — audio passes through unchanged.
    pub const BYPASS: DvuConfig = DvuConfig {
        audio: AudioInfo::STEREO_24,
        bypass: true,
        volumes: [0x0010_0000; 8],
        ramp: None,
        zero_cross_mute: false,
    };
}

/// IPC input path selection (written to IPSLR).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum IpcSel {
    /// Input path disabled.
    None = 0b000,
    /// SSI → 2SRC (async clock domain).
    SsiToSrcAsync = 0b001,
    /// FFD → 2SRC (async, CPU supplies audio via DMA).
    FfdToSrcAsync = 0b011,
    /// FFD → 2SRC (sync, SCUX uses SSI clock reference).
    FfdToSrcSync = 0b111,
}

/// OPC output path selection (written to OPSLR).
///
/// TRM Table: 000=no-op, 001=DIRECT (→ async SRC/DVU/MIX → SSIF),
/// 011=ASYNC (→ FFU capture), 100=SYNC (→ FFU capture), 101–111=no-op.
/// The reference always uses 0b001 for any SSIF output route (SRC direct,
/// SRC+DVU, or SRC+DVU+MIX). DVU/MIX routing is configured separately.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum OpcSel {
    /// Output path disabled.
    None = 0b000,
    /// Async path → SSIF direct (works with or without DVU/MIX in chain).
    ToSsi = 0b001,
    /// Async path → FFU (CPU receives converted audio via DMA).
    ToFfu = 0b011,
}

/// MIX configuration.
#[derive(Copy, Clone, Debug)]
pub struct MixConfig {
    /// Audio format for MIX output.
    pub audio: AudioInfo,
    /// If true, the mixer is bypassed (first connected source passes through).
    pub bypass: bool,
}

// ── Link descriptors ──────────────────────────────────────────────────────────

/// Cache-line-aligned 8-word DMA link descriptor (same layout as SSI).
#[repr(C, align(32))]
struct LinkDesc([u32; 8]);

// Stored DMA channel numbers — set at init_ffd_dma / init_ffu_dma time so
// start() can channel_start() the correct channels from a bitmask alone.
static FFD_DMA_CH_STORED: [core::sync::atomic::AtomicU8; 4] = [
    core::sync::atomic::AtomicU8::new(0),
    core::sync::atomic::AtomicU8::new(0),
    core::sync::atomic::AtomicU8::new(0),
    core::sync::atomic::AtomicU8::new(0),
];
static FFU_DMA_CH_STORED: [core::sync::atomic::AtomicU8; 4] = [
    core::sync::atomic::AtomicU8::new(0),
    core::sync::atomic::AtomicU8::new(0),
    core::sync::atomic::AtomicU8::new(0),
    core::sync::atomic::AtomicU8::new(0),
];

static mut FFD0_DESC: LinkDesc = LinkDesc([
    DESC_HEADER,                        // LDEN + NXA + valid (see DESC_HEADER)
    0,                                  // src  → FFD0 buffer (set at init)
    (CIM_BASE + DMATD0_CIM_OFF) as u32, // dst = DMATD0_CIM data register
    0,                                  // byte count (set at init)
    0,                                  // CHCFG (patched at init from dma_ch param)
    0,                                  // CHITVL
    0,                                  // CHEXT
    0,                                  // NXLA → &FFD0_DESC (set at init)
]);

static mut FFD1_DESC: LinkDesc = LinkDesc([
    DESC_HEADER,
    0,
    (CIM_BASE + DMATD1_CIM_OFF) as u32,
    0,
    0, // CHCFG (patched at init)
    0,
    0,
    0,
]);

static mut FFU0_DESC: LinkDesc = LinkDesc([
    DESC_HEADER,
    (CIM_BASE + DMATU0_CIM_OFF) as u32, // src = DMATU0_CIM data register
    0,                                  // dst → FFU0 buffer (set at init)
    0,                                  // byte count (set at init)
    0,                                  // CHCFG (patched at init)
    0,
    0,
    0,
]);

static mut FFU1_DESC: LinkDesc = LinkDesc([
    DESC_HEADER,
    (CIM_BASE + DMATU1_CIM_OFF) as u32,
    0,
    0,
    0, // CHCFG (patched at init)
    0,
    0,
    0,
]);

static mut FFD2_DESC: LinkDesc = LinkDesc([
    DESC_HEADER,
    0,
    (CIM_BASE + DMATD2_CIM_OFF) as u32,
    0,
    0, // CHCFG (patched at init)
    0,
    0,
    0,
]);

static mut FFD3_DESC: LinkDesc = LinkDesc([
    DESC_HEADER,
    0,
    (CIM_BASE + DMATD3_CIM_OFF) as u32,
    0,
    0, // CHCFG (patched at init)
    0,
    0,
    0,
]);

static mut FFU2_DESC: LinkDesc = LinkDesc([
    DESC_HEADER,
    (CIM_BASE + DMATU2_CIM_OFF) as u32,
    0,
    0,
    0, // CHCFG (patched at init)
    0,
    0,
    0,
]);

static mut FFU3_DESC: LinkDesc = LinkDesc([
    DESC_HEADER,
    (CIM_BASE + DMATU3_CIM_OFF) as u32,
    0,
    0,
    0, // CHCFG (patched at init)
    0,
    0,
    0,
]);

// ── Software reset ─────────────────────────────────────────────────────────────

/// Assert and then deassert the SCUX block-level software reset.
///
/// After this call all INIT bits in FFD/FFU/SRC/DVU/MIX/IPC/OPC are 1 (held
/// in initialisation state), which is the required starting condition before
/// configuring any sub-block (TRM §37.4).
///
/// # Safety
/// Writes to CIM memory-mapped registers.  Call from a single-threaded init
/// context after `stb::init()` has enabled the SCUX module clock.
pub unsafe fn reset() {
    unsafe {
        cim(SWRSR_CIM_OFF).write_volatile(SWRSR_RESET);
        let _ = cim(SWRSR_CIM_OFF).read_volatile(); // mandatory dummy read
        cim(SWRSR_CIM_OFF).write_volatile(SWRSR_RUN);
        let _ = cim(SWRSR_CIM_OFF).read_volatile();
    }
}

// ── Sub-block configuration ────────────────────────────────────────────────────

/// Configure one IPC (input path control) channel.
///
/// Must be called while the IPC channel INIT bit is 1 (i.e. before [`start`]).
///
/// # Safety
/// Writes to IPC memory-mapped registers.
pub unsafe fn configure_ipc(ch: u8, sel: IpcSel) {
    unsafe {
        ipc(ch, IPSLR_OFF).write_volatile(sel as u32);
    }
}

/// Configure one OPC (output path control) channel.
///
/// Must be called while the OPC channel INIT bit is 1.
///
/// # Safety
/// Writes to OPC memory-mapped registers.
pub unsafe fn configure_opc(ch: u8, sel: OpcSel) {
    unsafe {
        opc(ch, OPSLR_OFF).write_volatile(sel as u32);
    }
}

/// Configure one FFD (FIFO download / CPU→SCUX) channel.
///
/// `dma_size` is the DRQSR DMA request threshold in stereo samples (typically
/// 8 for 16-byte bursts or 16 for 32-byte bursts).
///
/// # Safety
/// Writes to FFD memory-mapped registers.
pub unsafe fn configure_ffd(ch: u8, audio: AudioInfo, dma_size: u8) {
    unsafe {
        ffd(ch, FDAIR_OFF).write_volatile(audio.to_reg());
        ffd(ch, DRQSR_OFF).write_volatile(dma_size as u32);
        ffd(ch, DEVMR_OFF).write_volatile(0); // all event flags cleared = DMA mode
        // FFDPR.PASS_ASYNC: enable async data path CIM (DMATD) → FFD → IPC.
        // Without this the audio data written to DMATD0_CIM goes nowhere.
        ffd(ch, FFDPR_OFF).write_volatile(FFDPR_PASS_ASYNC);
    }
}

/// Configure one FFU (FIFO upload / SCUX→CPU) channel.
///
/// # Safety
/// Writes to FFU memory-mapped registers.
pub unsafe fn configure_ffu(ch: u8, audio: AudioInfo, dma_size: u8) {
    unsafe {
        ffu(ch, FUAIR_OFF).write_volatile(audio.to_reg());
        ffu(ch, URQSR_OFF).write_volatile(dma_size as u32);
        ffu(ch, UEVMR_OFF).write_volatile(0); // all event flags cleared = DMA mode
        // FFUPR.PASS_ASYNC: enable async data path OPC → FFU → CIM (DMATU).
        ffu(ch, FFUPR_OFF).write_volatile(FFUPR_PASS_ASYNC);
    }
}

/// Configure one 2SRC path (identified by `unit` 0–1 and `pair` 0–1).
///
/// # Safety
/// Writes to 2SRC memory-mapped registers.
pub unsafe fn configure_src(unit: u8, pair: u8, cfg: SrcConfig) {
    unsafe {
        // Audio direction — stereo 24-bit etc.
        src(unit, pair, SADIR_OFF).write_volatile(cfg.audio.to_reg());

        // BSP (SCUX_InitHw) always writes SRCCR = BASE_VALUE (0x00010110) regardless
        // of bypass mode.  After scux::reset() (SWRSR) these must-be-1 bits are 0;
        // skip them and the SRC may malfunction even in bypass mode.
        src(unit, pair, SRCCR_OFF).write_volatile(SRCCR_MBZ1);

        if cfg.bypass {
            src(unit, pair, SRCBR_OFF).write_volatile(SRCBR_BYPASS);
            return;
        }

        // Bypass disabled (0 = active conversion)
        src(unit, pair, SRCBR_OFF).write_volatile(0);

        // IFSCR_INTIFSEN=1: use INTIFS ratio (async mode); 0 = sync mode.
        let ifscr_val: u32 = match cfg.mode {
            SrcMode::Async => IFSCR_INTIFSEN,
            SrcMode::Sync => 0,
        };
        src(unit, pair, IFSCR_OFF).write_volatile(ifscr_val);

        // Interpolation ratio
        src(unit, pair, IFSVR_OFF).write_volatile(cfg.intifs);

        // Common control: must-be-1 bits | mode bit
        let srccr_val = SRCCR_MBZ1
            | match cfg.mode {
                SrcMode::Sync => SRCCR_SYNC,
                SrcMode::Async => 0,
            };
        src(unit, pair, SRCCR_OFF).write_volatile(srccr_val);

        // Minimum frequency
        src(unit, pair, MNFSR_OFF).write_volatile(cfg.mnfsr);

        // Buffer size
        src(unit, pair, BFSSR_OFF).write_volatile(cfg.buf_size & 0b11);
    }
}

/// Update the INTIFS ratio of a running 2SRC path.
///
/// Valid while the SRC is active (TRM §37.7.4).  Write the new ratio into
/// IFSVR and then reload it by briefly asserting SRCIRR.INIT.
///
/// # Safety
/// Writes to 2SRC live registers; the path must be running.
pub unsafe fn src_update_intifs(unit: u8, pair: u8, new_intifs: u32) {
    unsafe {
        src(unit, pair, IFSVR_OFF).write_volatile(new_intifs);
        // Reload the input-rate by toggling SRCIRR.INIT (unit-level register)
        srcirr(unit).write_volatile(1); // assert
        srcirr(unit).write_volatile(0); // release
    }
}

/// Configure one DVU instance (phase 1 — safe while DVUIR.INIT = 1).
///
/// TRM §37: while DVUIR.INIT=1 the DVU continuously forces all its registers
/// to their initial values, so writes to VOLxR, VRCTR, DVUCR, etc. are
/// **discarded**.  Only VADIR and DVUBR are written here (identical to BSP
/// `SCUX_SetupDvu`).  Call [`apply_dvu_after_init`] immediately after
/// [`start`] clears DVUIR.INIT to apply volume / ramp settings.
///
/// # Safety
/// Writes to DVU memory-mapped registers.
pub unsafe fn configure_dvu(ch: u8, cfg: DvuConfig) {
    unsafe {
        // VADIR and DVUBR can be written while INIT=1 (BSP SetupDvu does so).
        dvu(ch, VADIR_OFF).write_volatile(cfg.audio.to_reg());
        if cfg.bypass {
            dvu(ch, DVUBR_OFF).write_volatile(DVUBR_BYPASS);
        } else {
            dvu(ch, DVUBR_OFF).write_volatile(0); // active (not bypassed)
        }
        // All other DVU registers (VOLxR, VRCTR, DVUCR, DVUER, …) must be
        // written AFTER DVUIR.INIT is cleared — see apply_dvu_after_init().
    }
}

/// Apply DVU volume / ramp settings after DVUIR.INIT has been cleared.
///
/// **Must be called immediately after [`start`]** clears `DVUIR.INIT` for
/// this DVU channel.  Mirrors BSP `SCUX_SetupDvuVolume`: writes ZCMCR,
/// VOLxR, VRCTR, DVUCR, VRPDR, VRDBR, VRWTR, and finally enables the DVU
/// via DVUER.DVUEN.
///
/// Has no effect (returns immediately) when `cfg.bypass == true`.
///
/// # Safety
/// Writes to DVU memory-mapped registers.  DVUIR.INIT must be 0.
pub unsafe fn apply_dvu_after_init(ch: u8, cfg: DvuConfig) {
    unsafe {
        if cfg.bypass {
            return; // bypass: no volume processing, nothing to set
        }

        // ZCMCR: zero-cross mute per active channel.
        let zcmcr_val = if cfg.zero_cross_mute {
            (1u32 << cfg.audio.channels as u32) - 1
        } else {
            0
        };
        dvu(ch, ZCMCR_OFF).write_volatile(zcmcr_val * ZCMCR_ZCEN);

        // VOLxR: per-channel volume.
        // Format: 4.20 fixed-point. Unity (0 dB) = 0x0010_0000 (= 1.0).
        // WARNING: bit 23 is the sign bit — 0x00FF_FFFF has sign=1 → MUTE!
        for i in 0..cfg.audio.channels {
            dvu(ch, vol_off(i)).write_volatile(cfg.volumes[i as usize]);
        }

        // DVUCR: VRMD (bit 4) + VVMD (bit 8).
        // VRMD mandatory when DVU enabled; VVMD enables digital volume path.
        // BSP: DVUCR |= DVUCR_DVU0_VVMD_SET (SetDigiVolRegister)
        //      DVUCR |= DVUCR_DVU0_VRMD_SET (SetupDvuVolume)
        dvu(ch, DVUCR_OFF).write_volatile(DVUCR_VRMD | DVUCR_VVMD);

        // VRCTR: enable volume ramp per active channel (BSP "dummy ramp").
        // BSP ALWAYS sets VREN for each active channel even for instant changes.
        let vren_mask: u32 = (1u32 << cfg.audio.channels as u32) - 1;
        let (vrpdr, vrdbr) = match cfg.ramp {
            Some(r) => (r.vrpdr, r.vrdbr),
            None => (0, 0), // instant: zero-rate dummy ramp
        };
        dvu(ch, VRCTR_OFF).write_volatile(vren_mask * VRCTR_VREN);
        dvu(ch, VRPDR_OFF).write_volatile(vrpdr);
        dvu(ch, VRDBR_OFF).write_volatile(vrdbr);
        dvu(ch, VRWTR_OFF).write_volatile(0);

        // DVUER.DVUEN — enable DVU output.  Must be last (BSP SetupDvuVolume).
        dvu(ch, DVUER_OFF).write_volatile(DVUER_DVUEN);
    }
}

/// Set the volume for a single audio sub-channel within a DVU instance.
///
/// | Value        | Gain |
/// |--------------|------|
/// | 0x0000_0000  | Mute (−∞ dB) |
/// | 0x0010_0000  | Unity (0 dB) — 4.20 fixed-point value 1.0 |
/// | 0x7F_FFFF    | Maximum (+18 dB) |
///
/// The DVU must not be in bypass mode for this to take effect.
///
/// # Safety
/// Writes to a DVU VOLxR register.
pub unsafe fn set_volume(dvu_ch: u8, audio_ch: u8, vol: u32) {
    unsafe {
        dvu(dvu_ch, vol_off(audio_ch)).write_volatile(vol);
    }
}

/// Set the same volume level on all audio sub-channels of a DVU instance.
///
/// # Safety
/// Writes to DVU VOL0R–VOL7R registers.
pub unsafe fn set_volume_all(dvu_ch: u8, n_channels: u8, vol: u32) {
    unsafe {
        for i in 0..n_channels {
            dvu(dvu_ch, vol_off(i)).write_volatile(vol);
        }
    }
}

/// Arm the hardware volume ramp engine on a DVU instance.
///
/// Writes `vrpdr` (ramp period), `vrdbr` (dB step), and enables the ramp.
/// The ramp runs until the target volume set by [`set_volume`] is reached.
///
/// # Safety
/// Writes to DVU ramp registers.
pub unsafe fn start_ramp(dvu_ch: u8, ramp: RampConfig) {
    unsafe {
        dvu(dvu_ch, VRPDR_OFF).write_volatile(ramp.vrpdr);
        dvu(dvu_ch, VRDBR_OFF).write_volatile(ramp.vrdbr);
        dvu(dvu_ch, VRCTR_OFF).write_volatile(VRCTR_VREN); // caller sets channel mask if needed
    }
}

/// Configure the MIX block.
///
/// In bypass mode only the first input source passes through; the other
/// sources are ignored.
///
/// # Safety
/// Writes to MIX memory-mapped registers.
pub unsafe fn configure_mix(cfg: MixConfig) {
    unsafe {
        mix(MADIR_OFF).write_volatile(cfg.audio.to_reg());

        if cfg.bypass {
            mix(MIXBR_OFF).write_volatile(MIXBR_BYPASS);
        } else {
            mix(MIXBR_OFF).write_volatile(0);
            // Unity gain on all four mix inputs, all enabled
            // MDBxR: 10-bit dB gain, 0x000 = 0 dB (unity), 0x3FF = mute.
            mix(MDB0R_OFF).write_volatile(0x000);
            mix(MDB1R_OFF).write_volatile(0x000);
            mix(MDB2R_OFF).write_volatile(0x000);
            mix(MDB3R_OFF).write_volatile(0x000);
            mix(MDBER_OFF).write_volatile(MDBER_MIXDBEN); // BSP: MDBER_MIX0_MIXDBEN_SET
        }
    }
}

/// Set the per-source gain on one MIX input (MDB0R–MDB3R).
///
/// The MIX block has four input slots, each with a 10-bit dB gain coefficient:
/// - `0x000` = 0 dB (unity)
/// - `0x001`–`0x3FE` = attenuation (larger = more attenuation, ~−0.5 dB/step)
/// - `0x3FF` = mute
///
/// `source` is 0–3 corresponding to DVU0_0–DVU0_3.
/// This allows hardware crossfades between multiple FFD paths meeting at the MIX.
///
/// # Safety
/// Writes to MIX MDBxR registers.  MIX must not be in bypass mode.
pub unsafe fn set_mix_source_gain(source: u8, gain: u32) {
    unsafe {
        let off = match source {
            0 => MDB0R_OFF,
            1 => MDB1R_OFF,
            2 => MDB2R_OFF,
            _ => MDB3R_OFF,
        };
        mix(off).write_volatile(gain & 0x3FF);
    }
}

///
/// Called when async mode is used with an FFD input path.  Pass the encoded
/// value for the `SCKSEL` field; set `DIVEN = 1` to enable the signal output.
/// For example `(1 << 8) | 0b1000` selects SSIF0 WS with output enabled.
///
/// # Safety
/// Writes to CIM FDTSELn register.
pub unsafe fn set_fdtsel(src_unit: u8, val: u32) {
    unsafe {
        let off = match src_unit {
            0 => FDTSEL0_CIM_OFF,
            1 => FDTSEL1_CIM_OFF,
            2 => FDTSEL2_CIM_OFF,
            _ => FDTSEL3_CIM_OFF,
        };
        cim(off).write_volatile(val);
    }
}

/// Configure the FUTSELn_CIM output-timing-signal selector for one SRC unit.
///
/// Required when async mode is used and output goes through MIX or FFU.
/// Same encoding as [`set_fdtsel`].
///
/// # Safety
/// Writes to CIM FUTSELn register.
pub unsafe fn set_futsel(src_unit: u8, val: u32) {
    unsafe {
        let off = match src_unit {
            0 => FUTSEL0_CIM_OFF,
            1 => FUTSEL1_CIM_OFF,
            2 => FUTSEL2_CIM_OFF,
            _ => FUTSEL3_CIM_OFF,
        };
        cim(off).write_volatile(val);
    }
}

/// Write the CIM SSIRSEL register to route an SSI channel into a 2SRC unit.
///
/// `ssirsel` is written verbatim.  Consult TRM §37.3 Table 37-3 for encoding.
///
/// # Safety
/// Writes to CIM SSIRSEL register.
pub unsafe fn set_ssirsel(ssirsel: u32) {
    unsafe {
        cim(SSIRSEL_CIM_OFF).write_volatile(ssirsel);
    }
}

/// Write the CIM SSIPMD register (SSI port-mode / receive/transmit enable).
///
/// # Safety
/// Writes to CIM register.
pub unsafe fn set_ssipmd(val: u32) {
    unsafe {
        cim(SSIPMD_CIM_OFF).write_volatile(val);
    }
}

/// Write the CIM SSICTRL register (SSI clock/direct-drive control).
///
/// Setting bit `SSI012TEN` (bit 8) routes the SCUX output directly to the
/// SSIF0 transmitter, bypassing the SSI DMA path.
///
/// # Safety
/// Writes to CIM register.
pub unsafe fn set_ssictrl(val: u32) {
    unsafe {
        cim(SSICTRL_CIM_OFF).write_volatile(val);
    }
}

/// Set up DMA for one FFD (download) channel and connect it to CIM.
///
/// `src_buf` must point to a buffer accessible by the DMAC (physical SRAM).
/// `buf_bytes` is the total buffer size in bytes; the DMA will loop over this
/// entire region continuously.
///
/// # Safety
/// - Must be called after [`reset`] and sub-block configuration, before
///   [`start`].
/// - `src_buf` must remain valid and DMA-accessible for the lifetime of the
///   transfer.
/// - Writes to static link-descriptor memory and DMAC registers.
pub unsafe fn init_ffd_dma(ffd_ch: u8, dma_ch: u8, src_buf: *const u32, buf_bytes: usize) {
    unsafe {
        FFD_DMA_CH_STORED[ffd_ch as usize].store(dma_ch, core::sync::atomic::Ordering::Relaxed);
        let (desc_ptr, cim_dmatd_off, dmars) = match ffd_ch {
            0 => (
                core::ptr::addr_of_mut!(FFD0_DESC),
                DMATD0_CIM_OFF,
                DMARS_SCUTXI0,
            ),
            1 => (
                core::ptr::addr_of_mut!(FFD1_DESC),
                DMATD1_CIM_OFF,
                DMARS_SCUTXI1,
            ),
            2 => (
                core::ptr::addr_of_mut!(FFD2_DESC),
                DMATD2_CIM_OFF,
                DMARS_SCUTXI2,
            ),
            3 => (
                core::ptr::addr_of_mut!(FFD3_DESC),
                DMATD3_CIM_OFF,
                DMARS_SCUTXI3,
            ),
            _ => panic!("init_ffd_dma: invalid channel"),
        };

        let desc_u = (desc_ptr as usize + UNCACHED_MIRROR_OFFSET) as *mut u32;

        // Patch: source buffer address (SRAM audio data, increments each transfer)
        desc_u.add(1).write_volatile(src_buf as u32);
        // Patch: destination = DMATD_CIM data register (fixed address, SCUX ingests here)
        desc_u
            .add(2)
            .write_volatile((CIM_BASE + cim_dmatd_off) as u32);
        // Patch: transfer byte count
        desc_u.add(3).write_volatile(buf_bytes as u32);
        // Patch: CHCFG (computed from dma_ch at runtime)
        desc_u.add(4).write_volatile(ffd_chcfg(dma_ch));
        // Patch: self-referential NXLA (uncached alias)
        desc_u.add(7).write_volatile(desc_u as u32);

        dmac::init_with_link_descriptor(dma_ch, desc_u as *const u32, dmars);
    }
}

/// Set up DMA for one FFU (upload) channel.
///
/// # Safety
/// Same requirements as [`init_ffd_dma`].
pub unsafe fn init_ffu_dma(ffu_ch: u8, dma_ch: u8, dst_buf: *mut u32, buf_bytes: usize) {
    unsafe {
        FFU_DMA_CH_STORED[ffu_ch as usize].store(dma_ch, core::sync::atomic::Ordering::Relaxed);
        let (desc_ptr, cim_dmatu_off, dmars) = match ffu_ch {
            0 => (
                core::ptr::addr_of_mut!(FFU0_DESC),
                DMATU0_CIM_OFF,
                DMARS_SCURXI0,
            ),
            1 => (
                core::ptr::addr_of_mut!(FFU1_DESC),
                DMATU1_CIM_OFF,
                DMARS_SCURXI1,
            ),
            2 => (
                core::ptr::addr_of_mut!(FFU2_DESC),
                DMATU2_CIM_OFF,
                DMARS_SCURXI2,
            ),
            3 => (
                core::ptr::addr_of_mut!(FFU3_DESC),
                DMATU3_CIM_OFF,
                DMARS_SCURXI3,
            ),
            _ => panic!("init_ffu_dma: invalid channel"),
        };

        let desc_u = (desc_ptr as usize + UNCACHED_MIRROR_OFFSET) as *mut u32;

        // Patch: source = DMATU_CIM data register (fixed address, SCUX outputs here)
        desc_u
            .add(1)
            .write_volatile((CIM_BASE + cim_dmatu_off) as u32);
        // Patch: destination buffer (SRAM, increments each transfer)
        desc_u.add(2).write_volatile(dst_buf as u32);
        // Patch: byte count
        desc_u.add(3).write_volatile(buf_bytes as u32);
        // Patch: CHCFG (computed from dma_ch at runtime)
        desc_u.add(4).write_volatile(ffu_chcfg(dma_ch));
        // Patch: NXLA
        desc_u.add(7).write_volatile(desc_u as u32);

        dmac::init_with_link_descriptor(dma_ch, desc_u as *const u32, dmars);
    }
}

/// Enable DMA in CIM and clear INIT bits on all configured sub-blocks.
///
/// Call this after all sub-blocks have been configured and their DMA channels
/// set up.  The TRM mandates a specific INIT-clear order:
/// FFD → FFU → 2SRC (SRCIR + SRCIRR) → DVU → MIX → IPC → OPC (last).
///
/// `ffd_mask`: bitmask of FFD channels to start (bit 0 = FFD ch 0, etc.).
/// `ffu_mask`: bitmask of FFU channels to start.
/// `src_mask`: bitmask of 2SRC paths (bits 0–3, maps to unit/pair as
///   bit 0=unit0,pair0; bit 1=unit0,pair1; bit 2=unit1,pair0; bit 3=unit1,pair1).
/// `dvu_mask`: bitmask of DVU channels to start.
/// `mix_en`: true to start the MIX block.
/// `ipc_mask`, `opc_mask`: bitmasks of IPC/OPC channels to start.
///
/// # Safety
/// All specified sub-blocks must have been configured before calling this.
pub unsafe fn start(
    ffd_mask: u8,
    ffu_mask: u8,
    src_mask: u8,
    dvu_mask: u8,
    mix_en: bool,
    ipc_mask: u8,
    opc_mask: u8,
) {
    unsafe {
        // Enable DMA triggers in CIM.
        // DMACR: bits [3:0] = FFD0-3 TX enable, bits [19:16] = FFU0-3 RX enable
        let dmacr = (ffd_mask as u32) | ((ffu_mask as u32) << 16);
        cim(DMACR_CIM_OFF).write_volatile(dmacr);

        // Start DMA channels
        for ch in 0..4u8 {
            if ffd_mask & (1 << ch) != 0 {
                dmac::channel_start(
                    FFD_DMA_CH_STORED[ch as usize].load(core::sync::atomic::Ordering::Relaxed),
                );
            }
        }
        for ch in 0..4u8 {
            if ffu_mask & (1 << ch) != 0 {
                dmac::channel_start(
                    FFU_DMA_CH_STORED[ch as usize].load(core::sync::atomic::Ordering::Relaxed),
                );
            }
        }

        // 1. FFD: clear INIT (start), then set BOOT to arm the FIFO.
        for ch in 0..4u8 {
            if ffd_mask & (1 << ch) != 0 {
                ffd(ch, FFDIR_OFF).write_volatile(INIT_CLR);
                ffd(ch, FFDBR_OFF).write_volatile(FFDBR_BOOT);
            }
        }

        // 2. FFU: clear INIT (start).
        for ch in 0..4u8 {
            if ffu_mask & (1 << ch) != 0 {
                ffu(ch, FFUIR_OFF).write_volatile(INIT_CLR);
            }
        }

        // 3a. Routing registers — must be written after SWRST which may clear them.
        //     Hardware reset value 0x76543210 = pass-through identity mapping.
        cim(SRCRSEL0_CIM_OFF).write_volatile(0x7654_3210);
        cim(SRCRSEL1_CIM_OFF).write_volatile(0x7654_3210);
        cim(SRCRSEL2_CIM_OFF).write_volatile(0x7654_3210);
        cim(SRCRSEL3_CIM_OFF).write_volatile(0x7654_3210);
        cim(MIXRSEL_CIM_OFF).write_volatile(0x7654_3210);

        // 3b. 2SRC: clear SRCIR and SRCIRR for each active path.
        for idx in 0..4u8 {
            if src_mask & (1 << idx) != 0 {
                let unit = idx / 2;
                let pair = idx % 2;
                src(unit, pair, SRCIR_OFF).write_volatile(INIT_CLR);
                srcirr(unit).write_volatile(INIT_CLR);
            }
        }

        // 4. DVU: clear INIT only.
        //    VOLxR, VRCTR, DVUCR, DVUER must be written AFTER DVUIR.INIT=0;
        //    the caller must invoke apply_dvu_after_init() once start() returns.
        for ch in 0..4u8 {
            if dvu_mask & (1 << ch) != 0 {
                dvu(ch, DVUIR_OFF).write_volatile(INIT_CLR);
                // DVUER and volume regs: written by apply_dvu_after_init().
            }
        }

        // 5. MIX: clear INIT.
        if mix_en {
            mix(MIXIR_OFF).write_volatile(INIT_CLR);
        }

        // 6. IPC and OPC — MUST be last (TRM §37.4 startup order).
        for ch in 0..4u8 {
            if ipc_mask & (1 << ch) != 0 {
                ipc(ch, IPCIR_OFF).write_volatile(INIT_CLR);
            }
        }
        for ch in 0..4u8 {
            if opc_mask & (1 << ch) != 0 {
                opc(ch, OPCIR_OFF).write_volatile(INIT_CLR);
            }
        }
    }
}

/// Stop all active SCUX paths and re-assert INIT on all sub-blocks.
///
/// Asserts INIT in the reverse order from [`start`]: OPC/IPC first, MIX,
/// DVU, 2SRC, FFU, FFD last.  Disables DMA in CIM.
///
/// After this call, sub-blocks can be reconfigured and [`start`] called again.
///
/// # Safety
/// Writes to SCUX and DMAC registers.
pub unsafe fn stop() {
    unsafe {
        // OPC / IPC first (TRM §37.4 stop order: reverse of start)
        for ch in 0..4u8 {
            opc(ch, OPCIR_OFF).write_volatile(INIT_SET);
            ipc(ch, IPCIR_OFF).write_volatile(INIT_SET);
        }

        // MIX
        mix(MIXIR_OFF).write_volatile(INIT_SET);

        // DVU
        for ch in 0..4u8 {
            dvu(ch, DVUIR_OFF).write_volatile(INIT_SET);
        }

        // 2SRC
        for unit in 0..2u8 {
            for pair in 0..2u8 {
                src(unit, pair, SRCIR_OFF).write_volatile(INIT_SET);
            }
        }

        // FFU, FFD
        for ch in 0..4u8 {
            ffu(ch, FFUIR_OFF).write_volatile(1);
            ffd(ch, FFDIR_OFF).write_volatile(1);
        }

        // Disable DMA triggers in CIM
        cim(DMACR_CIM_OFF).write_volatile(0);
    }
}
