//! SCUX DVU output path — stereo volume control on the SSIF0 output.
//!
//! Inserts the SCUX DVU block between the CPU audio buffer and the SSI0
//! transmitter, enabling hardware-accelerated volume control, zero-cross
//! mute, and smooth hardware-ramp fades without CPU involvement.
//!
//! ## Signal path
//! ```text
//! CPU SRAM ──DMA ch0──► FFD0_0 ──► IPC0 ──► 2SRC0/0 (sync passthrough)
//!                                                 │
//!                                              DVU0_0  ◄── set_volume / fade_to
//!                                                 │
//!                                              OPC0 ──► SSIF0 TX (direct drive)
//! ```
//!
//! ## Constraints
//! - **Disables SSI TX DMA (ch 6).**  Call [`init`] instead of (or instead
//!   of also calling) `rza1l_hal::ssi::init()`.  The SSI RX DMA path (ch 7) is
//!   not affected.
//! - Routing works only while SCUX is streaming (i.e. after [`init`]).
//! - `set_volume` and `fade_to` are safe to call from any context (they write
//!   only DVU register addresses, which are separate from DMA-accessed FIFOs).
//!
//! ## Volume encoding
//! | `vol` value    | Gain       |
//! |----------------|------------|
//! | 0x0000_0000    | Mute (−∞)  |
//! | 0x0010_0000    | 0 dB (unity — 4.20 fixed-point 1.0) |
//! | 0x7F_FFFF      | +18 dB (max) |
//!
//! Pass `0x0010_0000` for unity gain after init.

use rza1l_hal::{
    UNCACHED_MIRROR_OFFSET,
    scux::{
        self, AudioInfo, DvuConfig, INTIFS_44100_TO_44100, IpcSel, MixConfig, OpcSel, RampConfig,
        SrcConfig, SrcMode,
    },
    ssi,
};

/// Number of stereo frames in the DVU path DMA buffer.
/// At 44.1 kHz this is ~46.4 ms — enough headroom for the SCUX FFD FIFO
/// burst behaviour and USB SOF jitter without false underruns.
pub const DVU_PATH_FRAMES: usize = 2048;

/// Total buffer length (stereo = 2 samples per frame).
pub const DVU_PATH_BUF_LEN: usize = DVU_PATH_FRAMES * 2;

/// Cache-line-aligned audio sample buffer fed into FFD0_0.
#[repr(align(32))]
struct Aligned32<const N: usize>([i32; N]);

static mut DVU_TX_BUF: Aligned32<DVU_PATH_BUF_LEN> = Aligned32([0i32; DVU_PATH_BUF_LEN]);

// ── Internal DVU channel assignment ──────────────────────────────────────────

const DVU_CH: u8 = 0;
const IPC_CH: u8 = 0;
const OPC_CH: u8 = 0;
const FFD_CH: u8 = 0;
const SRC_UNIT: u8 = 0;
const SRC_PAIR: u8 = 0;

// ── SSICTRL / FDTSEL values (using pub constants from rza1l_hal::scux) ─────────────────
//
// SSICTRL_SSI0TX (bit 14): SCUX drives SSIF0 transmit directly.
// BSP: SSICTRL_CIM_SSI0TX_SET = (1U << 14)
// SSI012TEN (bit 1) is for 6-channel shared mode only — must stay 0 here.
// With SSI0TX set, the SSI0 TX DMA (ch 6) must NOT be running simultaneously.
use rza1l_hal::scux::{FDTSEL_DIVEN, FDTSEL_SCKSEL_SSIF0_WS, SSICTRL_SSI0TX};

// TSEL_SSIF0_WS: DIVEN=1 (bit 8) | SCKSEL=SSIF0_WS (value 8 = 0b1000).
// BSP: FDTSEL_CIM_DIVEN_SET | FDTSEL_CIM_SCKSEL_SSIF0_WS_SET
const TSEL_SSIF0_WS: u32 = FDTSEL_DIVEN | FDTSEL_SCKSEL_SSIF0_WS;

/// Initialise the DVU output path and start SCUX streaming.
///
/// After this call the SCUX block continuously reads samples from the internal
/// `DVU_TX_BUF`, passes them through the DVU volume stage, and drives the
/// SSIF0 transmitter directly.
///
/// Fill `DVU_TX_BUF` via [`tx_buf_start`] and [`tx_buf_end`] before calling
/// this function to avoid an initial burst of silence.
///
/// This function calls [`rza1l_hal::ssi::init_rx_only`] internally to start the
/// SSI0 receive path (codec → memory DMA).  **Do not** call `rza1l_hal::ssi::init()`
/// separately — doing so starts the SSI0 TX DMA on ch 6, which conflicts with
/// the SCUX direct-drive path on SSIF0.
///
/// # Safety
/// Must be called once from a single-threaded boot context after
/// `rza1l_hal::stb::init()` and SSI0 pin-mux setup.
pub unsafe fn init() {
    unsafe {
        log::debug!("scux_dvu_path: init DVU output path (FFD0→IPC0→2SRC0→DVU0→OPC0→SSIF0)");

        // Start SSI0 RX DMA (codec → memory) but intentionally skip SSI0 TX DMA —
        // the SCUX will drive SSIF0 TX directly via SSICTRL.SSI012TEN.
        ssi::init_rx_only(&crate::system::SSI_CONFIG);

        // 1. Software-reset the SCUX block.
        scux::reset();

        // 2. Patch the TX buffer pointer into the FFD0_0 link descriptor and
        //    set up DMA channel 0 → FFD0_0.
        let buf_ptr = core::ptr::addr_of!(DVU_TX_BUF.0[0]) as *const u32;
        let buf_bytes = DVU_PATH_BUF_LEN * core::mem::size_of::<i32>();
        scux::init_ffd_dma(FFD_CH, crate::system::SCUX_FFD0_DMA_CH, buf_ptr, buf_bytes);

        // 3. Configure sub-blocks (all INIT bits = 1 after reset — OK to write).

        // IPC0: FFD → 2SRC (async; SRC is bypassed so mode is irrelevant, but
        // async avoids any SSIRSEL / sync-clock-reference dependency)
        scux::configure_ipc(IPC_CH, IpcSel::FfdToSrcAsync);

        // OPC0: async path → SSIF0 direct drive.
        // OpcSel::ToSsi (0b001) is the only valid SSIF output value per TRM —
        // 0b101 is "no operation".  DVU/MIX routing is determined by block config,
        // not by this OPSLR field.
        scux::configure_opc(OPC_CH, OpcSel::ToSsi);

        // FFD0: stereo 24-bit, 8-sample DMA burst threshold
        scux::configure_ffd(FFD_CH, AudioInfo::STEREO_24, 8);

        // 2SRC0/0: bypass — audio passes straight through, no rate conversion
        // and no dependency on SSI clock reference via SSIRSEL.
        scux::configure_src(
            SRC_UNIT,
            SRC_PAIR,
            SrcConfig {
                mode: SrcMode::Async,
                audio: AudioInfo::STEREO_24,
                bypass: true,
                intifs: INTIFS_44100_TO_44100,
                mnfsr: 0,
                buf_size: 0,
            },
        );

        // DVU0 phase 1: write VADIR + DVUBR only (safe while DVUIR.INIT=1).
        // Phase 2 (VOLxR, VRCTR, DVUCR, DVUER) called after start() clears DVUIR.INIT.
        let dvu_cfg = DvuConfig {
            audio: AudioInfo::STEREO_24,
            bypass: false,
            volumes: [0x0010_0000; 8],
            ramp: None,
            zero_cross_mute: false,
        };
        scux::configure_dvu(DVU_CH, dvu_cfg);

        // MIX: not in the signal path.
        // BSP does NOT start MIX for SCUX_ROUTE_SRC0_SSIF0 (direct route).
        scux::configure_mix(MixConfig {
            audio: AudioInfo::STEREO_24,
            bypass: true,
        });

        // 4. Timing selectors — SCUX SRC async mode needs a timing reference.
        //    FDTSEL0: input  timing for 2SRC0 (used for FFD async path).
        //    FUTSEL0: output timing for 2SRC0 (required when output goes to MIX).
        //    SCKSEL = SSIF0_WS (value 8 = 44.1 kHz from codec), DIVEN = 1.
        //    Without this both registers default to DIVEN=0 = "output 0" and
        //    SCUX never produces samples → FFD FIFO never drains → DMA spins.
        scux::set_fdtsel(SRC_UNIT, TSEL_SSIF0_WS);
        scux::set_futsel(SRC_UNIT, TSEL_SSIF0_WS);

        // 5. Route SCUX output to SSIF0 TX.
        scux::set_ssictrl(SSICTRL_SSI0TX);

        // 6. Start all active sub-blocks.
        //    BSP (SCUX_ROUTE_SRC0_SSIF0): FFD→SRC→DVU→OPC→SSIF, no MIX.
        scux::start(
            0b0001, // FFD mask: FFD0
            0b0000, // FFU mask: none (output → SSIF via OPC, not captured by DMA)
            0b0001, // 2SRC mask: unit0, pair0
            0b0001, // DVU mask: DVU0
            false,  // MIX: not in path (BSP skips MIX for SCUX_ROUTE_SRC0_SSIF0)
            0b0001, // IPC mask: IPC0
            0b0001, // OPC mask: OPC0
        );

        // 6b. DVU phase 2: apply VOLxR, VRCTR, DVUCR, DVUER now that DVUIR.INIT=0.
        //     TRM §37: writes to these registers are discarded while DVUIR.INIT=1;
        //     they must be applied after start() clears the INIT bit.
        //     Mirrors BSP SCUX_SetupDvuVolume (called from SCUX_AsyncStartHw).
        scux::apply_dvu_after_init(DVU_CH, dvu_cfg);

        // 7. Enable SSI0 TX (TEN = SSICR bit 1).
        //    Required for SCUX direct-drive: SSICTRL.SSI0TX routes the SCUX output
        //    to the SSIF0 TX pin, but SSICR.TEN must also be 1 for the SSI0 TX
        //    serializer to run.  SSI012TEN (SSICTRL bit 1) must stay 0 in direct mode.
        ssi::enable_tx();

        log::debug!("scux_dvu_path: streaming started");
    }
}

/// Initialise the DVU output path with async sample-rate conversion.
///
/// Identical to [`init`] except that 2SRC0/0 is configured in async SRC mode
/// (input rate `engine_hz` → output 44.1 kHz) instead of bypass mode.
///
/// Use this when the engine is configured at a sample rate other than 44.1 kHz.
/// The engine writes samples at `engine_hz` into `DVU_TX_BUF`; the SCUX
/// 2SRC0/0 block converts them to 44.1 kHz before passing to the DVU and
/// ultimately to the SSI0 codec.
///
/// # Safety
/// Must be called once from a single-threaded boot context after
/// `rza1l_hal::stb::init()` and SSI0 pin-mux setup.
pub unsafe fn init_with_src(engine_hz: u32) {
    unsafe {
        log::debug!("scux_dvu_path: init with SRC {} Hz → 44100 Hz", engine_hz);

        ssi::init_rx_only(&crate::system::SSI_CONFIG);
        scux::reset();

        let buf_ptr = core::ptr::addr_of!(DVU_TX_BUF.0[0]) as *const u32;
        let buf_bytes = DVU_PATH_BUF_LEN * core::mem::size_of::<i32>();
        scux::init_ffd_dma(FFD_CH, crate::system::SCUX_FFD0_DMA_CH, buf_ptr, buf_bytes);

        scux::configure_ipc(IPC_CH, IpcSel::FfdToSrcAsync);
        scux::configure_opc(OPC_CH, OpcSel::ToSsi);
        scux::configure_ffd(FFD_CH, AudioInfo::STEREO_24, 8);

        // 2SRC0/0: async SRC — engine_hz → 44100 Hz.
        let intifs = scux::intifs(engine_hz, 44100);
        scux::configure_src(
            SRC_UNIT,
            SRC_PAIR,
            SrcConfig {
                mode: SrcMode::Async,
                audio: AudioInfo::STEREO_24,
                bypass: false,
                intifs,
                mnfsr: 0,
                buf_size: 0,
            },
        );

        let dvu_cfg = DvuConfig {
            audio: AudioInfo::STEREO_24,
            bypass: false,
            volumes: [0x0010_0000; 8],
            ramp: None,
            zero_cross_mute: false,
        };
        scux::configure_dvu(DVU_CH, dvu_cfg);
        scux::configure_mix(MixConfig {
            audio: AudioInfo::STEREO_24,
            bypass: true,
        });
        scux::set_fdtsel(SRC_UNIT, TSEL_SSIF0_WS);
        scux::set_futsel(SRC_UNIT, TSEL_SSIF0_WS);
        scux::set_ssictrl(SSICTRL_SSI0TX);

        scux::start(
            0b0001, // FFD mask: FFD0
            0b0000, // FFU mask: none
            0b0001, // 2SRC mask: unit0, pair0
            0b0001, // DVU mask: DVU0
            false,  // MIX: not in path
            0b0001, // IPC mask: IPC0
            0b0001, // OPC mask: OPC0
        );
        scux::apply_dvu_after_init(DVU_CH, dvu_cfg);
        ssi::enable_tx();

        log::debug!("scux_dvu_path: SRC streaming started");
    }
}

/// Set the volume for a single audio channel (0 = left, 1 = right).
///
/// Takes effect immediately (no ramp).  `vol = 0x0010_0000` is unity gain (0 dB).
///
/// # Safety
/// Writes to a DVU VOLxR memory-mapped register.
pub unsafe fn set_volume(audio_ch: u8, vol: u32) {
    unsafe {
        scux::set_volume(DVU_CH, audio_ch, vol);
    }
}

/// Set the same volume level on both left and right channels.
///
/// # Safety
/// Writes to DVU VOL0R and VOL1R registers.
pub unsafe fn set_volume_stereo(vol: u32) {
    unsafe {
        scux::set_volume_all(DVU_CH, 2, vol);
    }
}

/// Arm the hardware volume ramp engine targeting `target_vol` with the given
/// ramp parameters.
///
/// The ramp runs autonomously after this call; no further CPU intervention is
/// needed.  To monitor completion, poll the DVU status register (not yet
/// exposed) or calculate the expected duration from `ramp.vrpdr / vrdbr`.
///
/// # Safety
/// Writes to DVU ramp registers.
pub unsafe fn fade_to(target_vol: u32, ramp: RampConfig) {
    unsafe {
        // Write target volume first, then arm the ramp so it ramps *toward* target.
        scux::set_volume_all(DVU_CH, 2, target_vol);
        scux::start_ramp(DVU_CH, ramp);
    }
}

// ── Buffer pointer accessors ─────────────────────────────────────────────────

/// Pointer to the first sample in the SCUX DVU path TX buffer (uncached view).
///
/// Write audio samples here for the SCUX to pick up.  Wrap at [`tx_buf_end`].
pub fn tx_buf_start() -> *mut i32 {
    unsafe { (core::ptr::addr_of!(DVU_TX_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *mut i32 }
}

/// One-past-the-end pointer for the SCUX DVU path TX buffer (uncached view).
pub fn tx_buf_end() -> *mut i32 {
    unsafe { tx_buf_start().add(DVU_PATH_BUF_LEN) }
}

/// Current DMA read position within the TX buffer (uncached), aligned to one
/// stereo frame (8 bytes).
///
/// Mirrors `rza1l_hal::ssi::tx_current_ptr` in signature and semantics — write
/// audio samples *ahead of* this pointer.
pub fn tx_current_ptr() -> *mut i32 {
    let crsa = unsafe { rza1l_hal::dmac::current_src(crate::system::SCUX_FFD0_DMA_CH) };
    let aligned = crsa & !7u32;
    (aligned as usize + UNCACHED_MIRROR_OFFSET) as *mut i32
}

/// Update the input sample rate of 2SRC0/0 while the path is running.
///
/// Only valid when the path was initialised with [`init_with_src`].  Calling
/// this when the path is in bypass mode (initialised with [`init`]) has no
/// effect on audio quality but wastes a register write.
///
/// # Safety
/// Writes to SCUX SRC memory-mapped registers.  Must not be called from the
/// audio thread (register RMW is not atomic).
pub unsafe fn update_engine_rate(engine_hz: u32) {
    unsafe {
        let intifs = scux::intifs(engine_hz, 44_100);
        scux::src_update_intifs(SRC_UNIT, SRC_PAIR, intifs);
    }
}
