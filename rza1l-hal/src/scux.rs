//! Sample Rate Conversion Unit (SCUX) driver for the RZ/A1L.
//!
//! Hardware reference: RZ/A1L Group User's Manual: Hardware,
//! **Chapter 37 — SCUX** (R01UH0437EJ0700 Rev.7.00, Sep 2024).
//! Primary sections used:
//! - §37.3   Register Descriptions (§37.3.1 – §37.3.73)
//! - §37.4.1 Initial Setting Procedure (Fig. 37.6 – 37.8)
//! - §37.4.2 Transfer Start / Stop Procedure (Fig. 37.9 – 37.11)
//! - §37.4.5 Data Transfer Routes
//! - §37.4.6 Input/Output Timing Signals
//! - §37.4.7 2SRC Block (async/sync mode, INTIFS formula)
//! - §37.4.8 DVU Block (volume, ramp, zero-cross mute)
//! - §37.4.9 MIX Block
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
// TRM §37.3.1  IPCIR_IPC0_n — Initialization Register
// TRM §37.3.2  IPSLR_IPC0_n — Pass Select Register

const IPC_BASE: usize = SCUX_BASE;
const IPC_STRIDE: usize = 0x100;

const IPCIR_OFF: usize = 0x00; // IPCIR_IPC0_n (§37.3.1)  Init register (bit 0 = INIT)
const IPSLR_OFF: usize = 0x04; // IPSLR_IPC0_n (§37.3.2)  Pass-select register

#[inline(always)]
fn ipc(ch: u8, off: usize) -> *mut u32 {
    (IPC_BASE + ch as usize * IPC_STRIDE + off) as *mut u32
}

// ── OPC block (×4, stride 0x100) ─────────────────────────────────────────────
// TRM §37.3.3  OPCIR_OPC0_n — Initialization Register
// TRM §37.3.4  OPSLR_OPC0_n — Pass Select Register

const OPC_BASE: usize = SCUX_BASE + 0x0400;
const OPC_STRIDE: usize = 0x100;

const OPCIR_OFF: usize = 0x00; // OPCIR_OPC0_n (§37.3.3)  Init register
const OPSLR_OFF: usize = 0x04; // OPSLR_OPC0_n (§37.3.4)  Pass-select register

#[inline(always)]
fn opc(ch: u8, off: usize) -> *mut u32 {
    (OPC_BASE + ch as usize * OPC_STRIDE + off) as *mut u32
}

// ── FFD block (×4, stride 0x100) ─────────────────────────────────────────────
// TRM §37.3.5  FFDIR_FFD0_n  — FIFO Download Initialization Register
// TRM §37.3.6  FDAIR_FFD0_n  — FIFO Download Audio Information Register
// TRM §37.3.7  DRQSR_FFD0_n  — FIFO Download Request Size Register
// TRM §37.3.8  FFDPR_FFD0_n  — FIFO Download Pass Register
// TRM §37.3.9  FFDBR_FFD0_n  — FIFO Download Boot Register
// TRM §37.3.10 DEVMR_FFD0_n  — FIFO Download Event Mask Register
// TRM §37.3.11 DEVCR_FFD0_n  — FIFO Download Event Clear Register

const FFD_BASE: usize = SCUX_BASE + 0x0800;
const FFD_STRIDE: usize = 0x100;

const FFDIR_OFF: usize = 0x00; // FFDIR_FFD0_n (§37.3.5)  Init register (bit 0 = INIT)
const FDAIR_OFF: usize = 0x04; // FDAIR_FFD0_n (§37.3.6)  Audio info register (channels, bit depth)
const DRQSR_OFF: usize = 0x08; // DRQSR_FFD0_n (§37.3.7)  DMA request size register
const FFDPR_OFF: usize = 0x0C; // FFDPR_FFD0_n (§37.3.8)  FIFO data pass register
const FFDBR_OFF: usize = 0x10; // FFDBR_FFD0_n (§37.3.9)  Boot register (bit 0 = BOOT)
const DEVMR_OFF: usize = 0x14; // DEVMR_FFD0_n (§37.3.10) DMA event mode register (0 = DMA trigger, 1 = interrupt)
const DEVCR_OFF: usize = 0x1C; // DEVCR_FFD0_n (§37.3.11) DMA event clear register

#[inline(always)]
fn ffd(ch: u8, off: usize) -> *mut u32 {
    (FFD_BASE + ch as usize * FFD_STRIDE + off) as *mut u32
}

// ── FFU block (×4, stride 0x100) ─────────────────────────────────────────────
// TRM §37.3.12 FFUIR_FFU0_n  — FIFO Upload Initialization Register
// TRM §37.3.13 FUAIR_FFU0_n  — FIFO Upload Audio Information Register
// TRM §37.3.14 URQSR_FFU0_n  — FIFO Upload Request Size Register
// TRM §37.3.15 FFUPR_FFU0_n  — FIFO Upload Pass Register
// TRM §37.3.16 UEVMR_FFU0_n  — FIFO Upload Event Mask Register
// TRM §37.3.17 UEVCR_FFU0_n  — FIFO Upload Event Clear Register

const FFU_BASE: usize = SCUX_BASE + 0x0C00;
const FFU_STRIDE: usize = 0x100;

const FFUIR_OFF: usize = 0x00; // FFUIR_FFU0_n (§37.3.12) Init register
const FUAIR_OFF: usize = 0x04; // FUAIR_FFU0_n (§37.3.13) Audio info register
const URQSR_OFF: usize = 0x08; // URQSR_FFU0_n (§37.3.14) DMA request size register
const FFUPR_OFF: usize = 0x0C; // FFUPR_FFU0_n (§37.3.15) FIFO data pass register
const UEVMR_OFF: usize = 0x10; // UEVMR_FFU0_n (§37.3.16) DMA event mode register (0 = DMA trigger, 1 = interrupt)
const UEVCR_OFF: usize = 0x18; // UEVCR_FFU0_n (§37.3.17) DMA event clear register

#[inline(always)]
fn ffu(ch: u8, off: usize) -> *mut u32 {
    (FFU_BASE + ch as usize * FFU_STRIDE + off) as *mut u32
}

// ── 2SRC block ───────────────────────────────────────────────────────────────
// TRM §37.3.18 SRCIRp_2SRC0_m  — 2SRC Initialization Register p (m=0,1; p=0,1)
// TRM §37.3.19 SADIRp_2SRC0_m  — 2SRC Audio Information Register p
// TRM §37.3.20 SRCBRp_2SRC0_m  — 2SRC Bypass Register p
// TRM §37.3.21 IFSCRp_2SRC0_m  — 2SRC IFS Control Register p  (INTIFSEN bit 0)
// TRM §37.3.22 IFSVRp_2SRC0_m  — 2SRC IFS Value Setting Register p (INTIFS Q22)
// TRM §37.3.23 SRCCRp_2SRC0_m  — 2SRC Control Register p (SRCMD bit 0; must-be-1 bits 16,8,4)
// TRM §37.3.24 MNFSRp_2SRC0_m  — 2SRC Minimum FS Setting Register p
// TRM §37.3.25 BFSSRp_2SRC0_m  — 2SRC Buffer Size Setting Register p
// TRM §37.3.26 SC2SRp_2SRC0_m  — 2SRC Status Register p (read-only)
// TRM §37.3.27 WATSRp_2SRC0_m  — 2SRC Wait Time Setting Register p
// TRM §37.3.28 SEVMRp_2SRC0_m  — 2SRC Event Mask Register p
// TRM §37.3.29 SEVCRp_2SRC0_m  — 2SRC Event Clear Register p
// TRM §37.3.30 SRCIRR_2SRC0_m  — 2SRC Initialization Register RIF (unit-level)
//
// Two SRC units (unit 0 and unit 1), each containing two SRC paths (pair 0
// and pair 1).  The two paths within a unit share one BFSSR (buffer size
// register) and one SRCCR (common config), but have independent SADIR, IFSCR,
// IFSVR, MNFSR, SEVMR, and SRCIR / SRCIRR registers.
//
// Base of unit 0, pair 0 = 0xE820_9000.  Stride between pairs within a unit = 0x34.
// Unit 1 begins at 0xE820_9100 (stride 0x100 per TRM Table 37.2).

const SRC_BASE: usize = 0xE820_9000;
const SRC_UNIT_STRIDE: usize = 0x100;
const SRC_PAIR_STRIDE: usize = 0x34;

const SRCIR_OFF: usize = 0x00; // SRCIRp_2SRC0_m (§37.3.18) Init register
const SADIR_OFF: usize = 0x04; // SADIRp_2SRC0_m (§37.3.19) Audio input direction (channels, bit depth)
const SRCBR_OFF: usize = 0x08; // SRCBRp_2SRC0_m (§37.3.20) Bypass register (bit 0 = BYPASS)
const IFSCR_OFF: usize = 0x0C; // IFSCRp_2SRC0_m (§37.3.21) Input frequency select (0=sync, 1=async)
const IFSVR_OFF: usize = 0x10; // IFSVRp_2SRC0_m (§37.3.22) Input frequency value (INTIFS ratio Q22)
const SRCCR_OFF: usize = 0x14; // SRCCRp_2SRC0_m (§37.3.23) Common control (must-be-1 bits 16,8,4)
const MNFSR_OFF: usize = 0x18; // MNFSRp_2SRC0_m (§37.3.24) Minimum frequency select
const BFSSR_OFF: usize = 0x1C; // BFSSRp_2SRC0_m (§37.3.25) Buffer size select
// 0x20: SC2SRp_2SRC0_m (§37.3.26) status register (read-only)
// 0x24: WATSRp_2SRC0_m (§37.3.27) wait time
const SEVMR_OFF: usize = 0x28; // SEVMRp_2SRC0_m (§37.3.28) Sampling event mode register
// 0x2C: 4-byte reserved gap
const SEVCR_OFF: usize = 0x30; // SEVCRp_2SRC0_m (§37.3.29) Sampling event clear register
// SRCIRR_2SRC0_m (§37.3.30) lives at unit_base + 0x68 (after both pairs); see srcirr() below.
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
// TRM §37.3.31 DVUIR_DVU0_n  — DVU Initialization Register
// TRM §37.3.32 VADIR_DVU0_n  — DVU Audio Information Register
// TRM §37.3.33 DVUBR_DVU0_n  — DVU Bypass Register
// TRM §37.3.34 DVUCR_DVU0_n  — DVU Control Register (VRMD bit 4, VVMD bit 8)
// TRM §37.3.35 ZCMCR_DVU0_n  — DVU Zero Cross Mute Control Register
// TRM §37.3.36 VRCTR_DVU0_n  — DVU Volume Ramp Control Register
// TRM §37.3.37 VRPDR_DVU0_n  — DVU Volume Ramp Period Register
// TRM §37.3.38 VRDBR_DVU0_n  — DVU Volume Ramp Decibel Register
// TRM §37.3.39 VRWTR_DVU0_n  — DVU Volume Ramp Wait Time Register
// TRM §37.3.40–47 VOL0R–VOL7R_DVU0_n — Per-channel Volume Value Registers
// TRM §37.3.48 DVUER_DVU0_n  — DVU Enable Register (DVUEN bit 0)
// TRM §37.3.50 VEVMR_DVU0_n  — DVU Volume Event Mask Register
// TRM §37.3.51 VEVCR_DVU0_n  — DVU Volume Event Clear Register
// TRM §37.4.8  DVU Block operation notes

const DVU_BASE: usize = 0xE820_9200;
const DVU_STRIDE: usize = 0x100;

const DVUIR_OFF: usize = 0x00; // DVUIR_DVU0_n (§37.3.31) Init register
const VADIR_OFF: usize = 0x04; // VADIR_DVU0_n (§37.3.32) Audio direction (channels, bit depth)
const DVUBR_OFF: usize = 0x08; // DVUBR_DVU0_n (§37.3.33) Bypass register (bit 0 = BYPASS)
const DVUCR_OFF: usize = 0x0C; // DVUCR_DVU0_n (§37.3.34) Control (VRMD bit 4, VVMD bit 8)
const ZCMCR_OFF: usize = 0x10; // ZCMCR_DVU0_n (§37.3.35) Zero-cross mute control
const VRCTR_OFF: usize = 0x14; // VRCTR_DVU0_n (§37.3.36) Volume ramp control (bit 0 = enable ramp)
const VRPDR_OFF: usize = 0x18; // VRPDR_DVU0_n (§37.3.37) Volume ramp period
const VRDBR_OFF: usize = 0x1C; // VRDBR_DVU0_n (§37.3.38) Volume ramp dB step
const VRWTR_OFF: usize = 0x20; // VRWTR_DVU0_n (§37.3.39) Volume ramp wait time register
const VOL0R_OFF: usize = 0x24; // VOL0R–VOL7R_DVU0_n (§37.3.40–47) Per-channel volume registers
const DVUER_OFF: usize = 0x44; // DVUER_DVU0_n (§37.3.48) DVU enable register (DVUEN bit 0)
const VEVMR_OFF: usize = 0x4C; // VEVMR_DVU0_n (§37.3.50) Volume event mode
const VEVCR_OFF: usize = 0x54; // VEVCR_DVU0_n (§37.3.51) Volume event clear (0x50 is a 4-byte gap)

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
// TRM §37.3.52 MIXIR_MIX0_0  — MIX Initialization Register
// TRM §37.3.53 MADIR_MIX0_0  — MIX Audio Information Register
// TRM §37.3.54 MIXBR_MIX0_0  — MIX Bypass Register
// TRM §37.3.55 MIXMR_MIX0_0  — MIX Mode Register
// TRM §37.3.56 MVPDR_MIX0_0  — MIX Volume Period Register
// TRM §37.3.57 MDBAR_MIX0_0  — MIX Decibel A Register (source 0 gain)
// TRM §37.3.58 MDBBR_MIX0_0  — MIX Decibel B Register (source 1 gain)
// TRM §37.3.59 MDBCR_MIX0_0  — MIX Decibel C Register (source 2 gain)
// TRM §37.3.60 MDBDR_MIX0_0  — MIX Decibel D Register (source 3 gain)
// TRM §37.3.61 MDBER_MIX0_0  — MIX Decibel Enable Register (MIXDBEN bit 0)
// TRM §37.4.9  MIX Block operation notes

const MIX_BASE: usize = 0xE820_9600;

const MIXIR_OFF: usize = 0x00; // MIXIR_MIX0_0 (§37.3.52) Init register
const MADIR_OFF: usize = 0x04; // MADIR_MIX0_0 (§37.3.53) Audio direction
const MIXBR_OFF: usize = 0x08; // MIXBR_MIX0_0 (§37.3.54) Bypass register (bit 0 = BYPASS)
const MIXMR_OFF: usize = 0x0C; // MIXMR_MIX0_0 (§37.3.55) Mix mode register
const MVPDR_OFF: usize = 0x10; // MVPDR_MIX0_0 (§37.3.56) Master volume period
const MDB0R_OFF: usize = 0x14; // MDBAR_MIX0_0 (§37.3.57) Mix data buffer A — source 0 gain
const MDB1R_OFF: usize = 0x18; // MDBBR_MIX0_0 (§37.3.58) Mix data buffer B — source 1 gain
const MDB2R_OFF: usize = 0x1C; // MDBCR_MIX0_0 (§37.3.59) Mix data buffer C — source 2 gain
const MDB3R_OFF: usize = 0x20; // MDBDR_MIX0_0 (§37.3.60) Mix data buffer D — source 3 gain
const MDBER_OFF: usize = 0x24; // MDBER_MIX0_0 (§37.3.61) Mix data buffer enable

#[inline(always)]
fn mix(off: usize) -> *mut u32 {
    (MIX_BASE + off) as *mut u32
}

// ── CIM block ─────────────────────────────────────────────────────────────────
// TRM §37.3.63 SWRSR_CIM     — Software Reset Register
// TRM §37.3.64 DMACR_CIM     — DMA Control Register (bits 3:0=FFD0-3, bits 7:4=FFU0-3)
// TRM §37.3.65 DMATDn_CIM    — DMA Transfer Registers for FFD0_n (n=0–3)
// TRM §37.3.66 DMATUn_CIM    — DMA Transfer Registers for FFU0_n (n=0–3)
// TRM §37.3.67 SSIRSEL_CIM   — SSI Route Select Register
// TRM §37.3.68 FDTSELn_CIM   — FFD0_n Timing Select Register (n=0–3)
// TRM §37.3.69 FUTSELn_CIM   — FFU0_n Timing Select Register (n=0–3)
// TRM §37.3.70 SSIPMD_CIM    — SSI Pin Mode Register
// TRM §37.3.71 SSICTRL_CIM   — SSI Control Register (SSI012TEN bit 14, SSI012REN bit 8)
// TRM §37.3.72 SRCRSELn_CIM  — SRCn Route Select Register (n=0–3; reset = 0x76543210)
// TRM §37.3.73 MIXRSEL_CIM   — MIX Route Select Register (reset = 0x76543210)

const CIM_BASE: usize = 0xE820_9700;

const SWRSR_CIM_OFF: usize = 0x00; // SWRSR_CIM    (§37.3.63) Software reset (bit 0; 0=reset, 1=run)
const DMACR_CIM_OFF: usize = 0x04; // DMACR_CIM    (§37.3.64) DMA enable (bits 3:0=FFD0-3 TX, bits 7:4=FFU0-3 RX)
const DMATD0_CIM_OFF: usize = 0x08; // DMATD0_CIM   (§37.3.65) DMA channel number for FFD0
const DMATD1_CIM_OFF: usize = 0x0C; // DMATD1_CIM   (§37.3.65) DMA channel number for FFD1
const DMATD2_CIM_OFF: usize = 0x10; // DMATD2_CIM   (§37.3.65) DMA channel number for FFD2
const DMATD3_CIM_OFF: usize = 0x14; // DMATD3_CIM   (§37.3.65) DMA channel number for FFD3
const DMATU0_CIM_OFF: usize = 0x18; // DMATU0_CIM   (§37.3.66) DMA channel number for FFU0
const DMATU1_CIM_OFF: usize = 0x1C; // DMATU1_CIM   (§37.3.66) DMA channel number for FFU1
const DMATU2_CIM_OFF: usize = 0x20; // DMATU2_CIM   (§37.3.66) DMA channel number for FFU2
const DMATU3_CIM_OFF: usize = 0x24; // DMATU3_CIM   (§37.3.66) DMA channel number for FFU3
// 0x28–0x37: 16-byte reserved gap (TRM Table 37.2, between DMATUn and SSIRSEL)
const SSIRSEL_CIM_OFF: usize = 0x38; // SSIRSEL_CIM  (§37.3.67) SSI→SRC route selector
const FDTSEL0_CIM_OFF: usize = 0x3C; // FDTSEL0_CIM  (§37.3.68) FFD0 timing select
const FDTSEL1_CIM_OFF: usize = 0x40; // FDTSEL1_CIM  (§37.3.68) FFD1 timing select
const FDTSEL2_CIM_OFF: usize = 0x44; // FDTSEL2_CIM  (§37.3.68) FFD2 timing select
const FDTSEL3_CIM_OFF: usize = 0x48; // FDTSEL3_CIM  (§37.3.68) FFD3 timing select
const FUTSEL0_CIM_OFF: usize = 0x4C; // FUTSEL0_CIM  (§37.3.69) FFU0 timing select
const FUTSEL1_CIM_OFF: usize = 0x50; // FUTSEL1_CIM  (§37.3.69) FFU1 timing select
const FUTSEL2_CIM_OFF: usize = 0x54; // FUTSEL2_CIM  (§37.3.69) FFU2 timing select
const FUTSEL3_CIM_OFF: usize = 0x58; // FUTSEL3_CIM  (§37.3.69) FFU3 timing select
const SSIPMD_CIM_OFF: usize = 0x5C; // SSIPMD_CIM   (§37.3.70) SSI port mode
const SSICTRL_CIM_OFF: usize = 0x60; // SSICTRL_CIM  (§37.3.71) SSI clock/gate control
const SRCRSEL0_CIM_OFF: usize = 0x64; // SRCRSEL0_CIM (§37.3.72) SRC0 route select (reset = 0x76543210)
const SRCRSEL1_CIM_OFF: usize = 0x68; // SRCRSEL1_CIM (§37.3.72) SRC1 route select
const SRCRSEL2_CIM_OFF: usize = 0x6C; // SRCRSEL2_CIM (§37.3.72) SRC2 route select
const SRCRSEL3_CIM_OFF: usize = 0x70; // SRCRSEL3_CIM (§37.3.72) SRC3 route select
const MIXRSEL_CIM_OFF: usize = 0x74; // MIXRSEL_CIM  (§37.3.73) MIX input route selector (reset = 0x76543210)

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
/// The INTIFS field is written to IFSVRp_2SRC0_m (TRM §37.3.22) as a Q22
/// fixed-point value: `INTIFS = 2^22 × (Fin / Fout)`.  FSO is fixed at
/// `2^22 = 0x0040_0000`; FSI tracks the ratio continuously in async mode.
///
/// Example (TRM §37.3.22): Fin=32 kHz, Fout=44.1 kHz →
/// `INTIFS = 2^22 × 32000/44100 = 3043485 = 0x02E709D`.
///
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

/// IPC input path selection (written to IPSLR_IPC0_n, TRM §37.3.2).
///
/// Encodes the `IPC_PASS_SEL` field (bits 2:0):
/// - `000` = no operation
/// - `001` = SSI (external) → IPC → SRC (async)
/// - `011` = FFD → IPC → SRC (async) — CPU supplies audio via DMA ← usual
/// - `100` = FFD → IPC → SRC (sync) — clock from SSI
/// - `101–111` = no operation
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
    FfdToSrcSync = 0b100,
}

/// OPC output path selection (written to OPSLR_OPC0_n, TRM §37.3.4).
///
/// Encodes the `OPC_PASS_SEL` field (bits 2:0):
/// - `000` = no operation
/// - `001` = SRC (async) → OPC → DVU (or SSIF direct with DVU/MIX inline)
/// - `011` = SRC (async) → OPC → FFU (CPU receives converted audio via DMA)
/// - `100` = SRC (sync) → OPC → FFU
/// - `101–111` = no operation
///
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
/// Writes 0 then 1 to SWRSR_CIM.SWRST (TRM §37.3.63).  After this call all
/// INIT bits in FFD/FFU/SRC/DVU/MIX/IPC/OPC are 1 (held in initialisation
/// state), which is the required starting condition before configuring any
/// sub-block (TRM §37.4.1, Fig. 37.6 step <1>).
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
/// Writes `sel` to IPSLR_IPC0_n (TRM §37.3.2).  Must be called while the
/// IPC channel INIT bit is 1, i.e. before [`start`] or [`start_path`]
/// clears it (TRM §37.4.1, Fig. 37.6 step <2>).
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
/// Writes `sel` to OPSLR_OPC0_n (TRM §37.3.4).  Must be called while the
/// OPC channel INIT bit is 1 (TRM §37.4.1, Fig. 37.6 step <2>).
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
/// Writes to FDAIR, DRQSR, DEVMR, and FFDPR (TRM §§37.3.6–37.3.8, 37.3.10).
/// Must be called while FFDIR_FFD0_n.INIT = 1 (TRM §37.4.1, Fig. 37.7 step <5>).
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
/// Writes to FUAIR, URQSR, UEVMR, and FFUPR (TRM §§37.3.13–37.3.16).
/// Must be called while FFUIR_FFU0_n.INIT = 1 (TRM §37.4.1, Fig. 37.7 step <6>).
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
/// Writes to SADIRp, SRCBRp, IFSCRp, IFSVRp, SRCCRp, MNFSRp, BFSSRp
/// (TRM §§37.3.18–37.3.25).  Must be called while SRCIRp.INIT = 1
/// (TRM §37.4.1, Fig. 37.7 step <7>).
///
/// SRCCR must-be-1 bits (16, 8, 4) are always written per TRM §37.3.23 Note 1;
/// they are cleared by SWRSR and must be restored before the SRC will function
/// even in bypass mode.
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
/// Writes the new ratio to IFSVRp_2SRC0_m (TRM §37.3.22), then reloads it by
/// briefly asserting SRCIRR_2SRC0_m.INIT (TRM §37.3.30).  The SRC hardware
/// picks up the new rate within one output sample period without glitching.
///
/// Valid while the SRC is active (TRM §37.4.7).  The path must be running.
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
/// Writes VADIR_DVU0_n (TRM §37.3.32) and DVUBR_DVU0_n (TRM §37.3.33),
/// which are the only DVU registers that accept writes while DVUIR.INIT = 1
/// (TRM §37.4.8).  All other DVU registers (VOLxR, VRCTR, DVUCR, DVUER, …)
/// are continuously forced to their reset values while INIT = 1 and must be
/// written after INIT is cleared — see [`apply_dvu_after_init`].
///
/// Must be called during initial configuration before [`start`]
/// (TRM §37.4.1, Fig. 37.7 step <8>).
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
/// this DVU channel.  Writes ZCMCR (§37.3.35), VOL0R–VOL7R (§§37.3.40–47),
/// VRCTR (§37.3.36), VRPDR (§37.3.37), VRDBR (§37.3.38), VRWTR (§37.3.39),
/// DVUCR (§37.3.34), and finally DVUER.DVUEN (§37.3.48) — mirroring the BSP
/// `SCUX_SetupDvuVolume` sequence (TRM §37.4.8).
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
/// Writes MADIR_MIX0_0 (TRM §37.3.53), MIXBR_MIX0_0 (TRM §37.3.54), and in
/// non-bypass mode also MDBAR–MDBDR and MDBER (TRM §§37.3.57–37.3.61).
/// Must be called while MIXIR_MIX0_0.INIT = 1 (TRM §37.4.1, Fig. 37.8 step <9>).
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

/// Configure the FDTSELn_CIM input-timing-signal selector for one SRC unit.
///
/// Writes FDTSELn_CIM (TRM §37.3.68).  Called when async mode is used with
/// an FFD input path (TRM §37.4.6).  Pass the encoded value for the `SCKSEL`
/// field; set `DIVEN = 1` (bit 8, [`FDTSEL_DIVEN`]) to enable the signal output.
/// For example `FDTSEL_DIVEN | FDTSEL_SCKSEL_SSIF0_WS` selects SSIF0 WS with
/// output enabled.
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
/// Writes FUTSELn_CIM (TRM §37.3.69).  Required when async mode is used and
/// output goes through MIX or FFU (TRM §37.4.6).  Same encoding as
/// [`set_fdtsel`].
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
/// Writes SSIRSEL_CIM (TRM §37.3.67).  `ssirsel` is written verbatim; consult
/// TRM §37.3.67 Table 37-3 for encoding.  Required for SSIF→SRC paths.
///
/// # Safety
/// Writes to CIM SSIRSEL register.
pub unsafe fn set_ssirsel(ssirsel: u32) {
    unsafe {
        cim(SSIRSEL_CIM_OFF).write_volatile(ssirsel);
    }
}

/// Write the CIM SSIPMD register (SSI pin-mode / port direction).
///
/// Writes SSIPMD_CIM (TRM §37.3.70).  Controls whether each SSI pin is used
/// as transmit, receive, or master input.  Required during initial setup
/// (TRM §37.4.1, Fig. 37.6 step <4>).
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
/// Writes SSICTRL_CIM (TRM §37.3.71).  Setting [`SSICTRL_SSI0TX`] (bit 14)
/// routes the SCUX output directly to the SSIF0 transmitter, bypassing the
/// SSI DMA path.  Set this as the final step before enabling SSI TX
/// (TRM §37.4.2, Fig. 37.9 "Set transmission to start").
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
/// set up.  The TRM mandates a specific INIT-clear order for async SRC mode
/// (TRM §37.4.2, Fig. 37.9 "Clear initialization of FFD and start boot" and
/// "Clear initialization of FFU, SRC, DVU, MIX, IPC, and OPC"):
///
/// 1. Enable DMACR_CIM (§37.3.64) for FFD and FFU channels.
/// 2. Start corresponding DMAC channels.
/// 3. FFD: clear FFDIR.INIT, set FFDBR.BOOT (§§37.3.5, 37.3.9).
/// 4. FFU: clear FFUIR.INIT (§37.3.12).
/// 5. 2SRC: clear SRCIRp.INIT (§37.3.18) + SRCIRR.INIT (§37.3.30).
/// 6. DVU: clear DVUIR.INIT (§37.3.31) — then call [`apply_dvu_after_init`].
/// 7. MIX: clear MIXIR.INIT (§37.3.52).
/// 8. IPC: clear IPCIR.INIT (§37.3.1) — second-to-last.
/// 9. OPC: clear OPCIR.INIT (§37.3.3) — **must be last**.
///
/// Also writes SRCRSELn_CIM and MIXRSEL_CIM (§§37.3.72–37.3.73) to identity
/// mapping (0x7654_3210) after soft-reset clears them.
///
/// `ffd_mask`, `ffu_mask`: bitmask of channels to start (bit 0 = ch 0, etc.).
/// `src_mask`: bits 0–3 → unit0/pair0, unit0/pair1, unit1/pair0, unit1/pair1.
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
        // DMACR: bits [3:0] = FFD0-3 TX enable, bits [7:4] = FFU0-3 RX enable
        let dmacr = (ffd_mask as u32) | ((ffu_mask as u32) << 4);
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

/// Start additional SCUX sub-blocks without disturbing already-running paths.
///
/// Identical to [`start`] except:
/// - DMACR_CIM (§37.3.64) is updated with a read-modify-write (OR) so
///   FFD/FFU channels already enabled by a prior path are not cleared.
/// - Routing registers SRCRSELn_CIM and MIXRSEL_CIM (§§37.3.72–73) are
///   **not** re-written; they retain the identity mapping from [`start`].
///
/// INIT-clear order follows the same TRM §37.4.2 async sequence as [`start`]:
/// FFD → FFU → 2SRC → DVU → MIX → IPC → OPC (last).
///
/// # Safety
/// SCUX must have been started at least once via [`start`] before calling
/// this function.  Writes to SCUX and DMAC registers.
pub unsafe fn start_path(
    ffd_mask: u8,
    ffu_mask: u8,
    src_mask: u8,
    dvu_mask: u8,
    mix_en: bool,
    ipc_mask: u8,
    opc_mask: u8,
) {
    unsafe {
        // Read-modify-write DMACR so running channels are not cleared.
        let existing_dmacr = cim(DMACR_CIM_OFF).read_volatile();
        let new_dmacr = existing_dmacr | (ffd_mask as u32) | ((ffu_mask as u32) << 4);
        cim(DMACR_CIM_OFF).write_volatile(new_dmacr);

        // Start new DMA channels only.
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

        // FFD: clear INIT + set BOOT.
        for ch in 0..4u8 {
            if ffd_mask & (1 << ch) != 0 {
                ffd(ch, FFDIR_OFF).write_volatile(INIT_CLR);
                ffd(ch, FFDBR_OFF).write_volatile(FFDBR_BOOT);
            }
        }

        // FFU: clear INIT.
        for ch in 0..4u8 {
            if ffu_mask & (1 << ch) != 0 {
                ffu(ch, FFUIR_OFF).write_volatile(INIT_CLR);
            }
        }

        // 2SRC: clear SRCIR + SRCIRR.
        for idx in 0..4u8 {
            if src_mask & (1 << idx) != 0 {
                let unit = idx / 2;
                let pair = idx % 2;
                src(unit, pair, SRCIR_OFF).write_volatile(INIT_CLR);
                srcirr(unit).write_volatile(INIT_CLR);
            }
        }

        // DVU: clear INIT (caller must call apply_dvu_after_init after this).
        for ch in 0..4u8 {
            if dvu_mask & (1 << ch) != 0 {
                dvu(ch, DVUIR_OFF).write_volatile(INIT_CLR);
            }
        }

        // MIX: clear INIT.
        if mix_en {
            mix(MIXIR_OFF).write_volatile(INIT_CLR);
        }

        // IPC and OPC — MUST be last (TRM §37.4 startup order).
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

/// Stop the specified SCUX sub-blocks without disturbing other running paths.
///
/// Re-asserts INIT on each specified block in the reverse of the TRM startup
/// order (TRM §37.4.2, Fig. 37.11 "Transfer stop setting"):
/// OPC → IPC → MIX → DVU → 2SRC → FFU → FFD.
///
/// Clears the corresponding FFD/FFU bits in DMACR_CIM (§37.3.64) via
/// read-modify-write so other running channels are not disturbed.
///
/// After this call the stopped sub-blocks can be reconfigured and restarted
/// with [`start_path`].
///
/// # Safety
/// Writes to SCUX and DMAC registers.
pub unsafe fn stop_path(
    ffd_mask: u8,
    ffu_mask: u8,
    src_mask: u8,
    dvu_mask: u8,
    mix_en: bool,
    ipc_mask: u8,
    opc_mask: u8,
) {
    unsafe {
        // IPC / OPC first (TRM stop order: reverse of start).
        for ch in 0..4u8 {
            if opc_mask & (1 << ch) != 0 {
                opc(ch, OPCIR_OFF).write_volatile(INIT_SET);
            }
        }
        for ch in 0..4u8 {
            if ipc_mask & (1 << ch) != 0 {
                ipc(ch, IPCIR_OFF).write_volatile(INIT_SET);
            }
        }

        // MIX.
        if mix_en {
            mix(MIXIR_OFF).write_volatile(INIT_SET);
        }

        // DVU.
        for ch in 0..4u8 {
            if dvu_mask & (1 << ch) != 0 {
                dvu(ch, DVUIR_OFF).write_volatile(INIT_SET);
            }
        }

        // 2SRC.
        for idx in 0..4u8 {
            if src_mask & (1 << idx) != 0 {
                let unit = idx / 2;
                let pair = idx % 2;
                src(unit, pair, SRCIR_OFF).write_volatile(INIT_SET);
            }
        }

        // FFU, FFD.
        for ch in 0..4u8 {
            if ffu_mask & (1 << ch) != 0 {
                ffu(ch, FFUIR_OFF).write_volatile(INIT_SET);
            }
        }
        for ch in 0..4u8 {
            if ffd_mask & (1 << ch) != 0 {
                ffd(ch, FFDIR_OFF).write_volatile(INIT_SET);
            }
        }

        // Read-modify-write DMACR: clear only the stopped channels.
        let existing = cim(DMACR_CIM_OFF).read_volatile();
        let clear_mask = (ffd_mask as u32) | ((ffu_mask as u32) << 4);
        cim(DMACR_CIM_OFF).write_volatile(existing & !clear_mask);
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

/// Returns the DMA channel number assigned to the given FFD path.
///
/// Valid after [`init_ffd_dma`] has been called for `ffd_ch`.  Used by the
/// firmware to register a GIC completion handler (DMAINT = 41 + ch).
pub fn ffd_dma_ch(ffd_ch: u8) -> u8 {
    FFD_DMA_CH_STORED[ffd_ch as usize].load(core::sync::atomic::Ordering::Relaxed)
}
