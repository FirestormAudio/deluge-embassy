//! SCUX async SRC path — sample-rate conversion between two independent clock
//! domains.
//!
//! Routes audio written by the CPU at an arbitrary input rate through the
//! SCUX asynchronous 2SRC block, producing output at a different (or the
//! same) rate for CPU consumption via DMA.  The primary use case is bridging
//! the SSI0 crystal-locked 44.1 kHz domain and the USB UAC2 host-locked
//! 48 kHz domain.
//!
//! ## Signal path
//! ```text
//! CPU SRAM ──DMA ch3──► FFD0_1 ──► IPC1 ──► 2SRC0/1 (async) ──► OPC1 ──► FFU0_1 ──DMA ch5──► CPU SRAM
//! ```
//!
//! ## Usage
//! 1. Call [`init`] with the input and output sample rates.
//! 2. Write input samples to the buffer returned by [`tx_buf_start`].
//! 3. Read converted output samples from the buffer returned by [`rx_buf_start`].
//! 4. Call [`update_input_rate`] at any time if the input clock drifts.
//!
//! ## Notes
//! - This path uses DMA channels 3 (FFD1) and 5 (FFU1), which are different
//!   from the DVU output path (channels 0 and 1) so both can run simultaneously.
//! - The 2SRC unit 0, pair 1 is used, independent of the DVU path's pair 0.

use rza1l_hal::UNCACHED_MIRROR_OFFSET;
use rza1l_hal::scux::{self, AudioInfo, IpcSel, MixConfig, OpcSel, SrcConfig, SrcMode};

/// Number of stereo frames in the SRC path input (TX) buffer.
pub const SRC_TX_FRAMES: usize = 1024;
/// Number of stereo frames in the SRC path output (RX) buffer.
pub const SRC_RX_FRAMES: usize = 2048;

/// Total TX buffer length in `i32` samples.
pub const SRC_TX_BUF_LEN: usize = SRC_TX_FRAMES * 2;
/// Total RX buffer length in `i32` samples.
pub const SRC_RX_BUF_LEN: usize = SRC_RX_FRAMES * 2;

/// Cache-line-aligned audio buffers.
#[repr(align(32))]
struct Aligned32<const N: usize>([i32; N]);

static mut SRC_TX_BUF: Aligned32<SRC_TX_BUF_LEN> = Aligned32([0i32; SRC_TX_BUF_LEN]);
static mut SRC_RX_BUF: Aligned32<SRC_RX_BUF_LEN> = Aligned32([0i32; SRC_RX_BUF_LEN]);

// ── Internal sub-block channel assignments ────────────────────────────────────

/// FFD channel used for this path (FFD0 channel 1).
const FFD_CH: u8 = 1;
/// FFU channel used for this path (FFU0 channel 1).
const FFU_CH: u8 = 1;
/// IPC channel.
const IPC_CH: u8 = 1;
/// OPC channel.
const OPC_CH: u8 = 1;
/// 2SRC unit.
const SRC_UNIT: u8 = 0;
/// 2SRC pair within the unit.
const SRC_PAIR: u8 = 1;

/// Initialise the async SRC path.
///
/// `fin_hz`: nominal input sample rate in Hz (e.g. 48000).
/// `fout_hz`: output sample rate in Hz (e.g. 44100).
///
/// After this call:
/// - DMA ch 3 continuously reads from the internal `SRC_TX_BUF` into FFD0_1.
/// - The SCUX 2SRC block asynchronously converts and writes to FFU0_1.
/// - DMA ch 5 continuously writes the converted audio into `SRC_RX_BUF`.
///
/// # Safety
/// Must be called once from a single-threaded boot context after `rza1l_hal::stb::init()`.
pub unsafe fn init(fin_hz: u32, fout_hz: u32) {
    unsafe {
        log::debug!(
            "scux_src_path: init async SRC {} Hz → {} Hz",
            fin_hz,
            fout_hz
        );

        // Note: scux::reset() is not called here because this path may run
        // alongside scux_dvu_path which owns the reset cycle.  If this path
        // is used standalone, call scux::reset() before this function.

        let intifs = rza1l_hal::scux::intifs(fin_hz, fout_hz);

        // DMA: TX buffer → FFD0_1 (DMA ch 3)
        let tx_ptr = core::ptr::addr_of!(SRC_TX_BUF.0[0]) as *const u32;
        let tx_bytes = SRC_TX_BUF_LEN * core::mem::size_of::<i32>();
        scux::init_ffd_dma(FFD_CH, crate::system::SCUX_FFD1_DMA_CH, tx_ptr, tx_bytes);

        // DMA: FFU0_1 → RX buffer (DMA ch 5)
        let rx_ptr = core::ptr::addr_of_mut!(SRC_RX_BUF.0[0]) as *mut u32;
        let rx_bytes = SRC_RX_BUF_LEN * core::mem::size_of::<i32>();
        scux::init_ffu_dma(FFU_CH, crate::system::SCUX_FFU1_DMA_CH, rx_ptr, rx_bytes);

        // Sub-block config
        scux::configure_ipc(IPC_CH, IpcSel::FfdToSrcAsync);
        scux::configure_opc(OPC_CH, OpcSel::ToFfu);
        scux::configure_ffd(FFD_CH, AudioInfo::STEREO_24, 8);
        scux::configure_ffu(FFU_CH, AudioInfo::STEREO_24, 8);

        // 2SRC: async mode
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

        // MIX: not in path
        scux::configure_mix(MixConfig {
            audio: AudioInfo::STEREO_24,
            bypass: true,
        });

        // Start: FFD1, FFU1, 2SRC unit0/pair1 (mask bit 1), no DVU, no MIX,
        // IPC1, OPC1.
        scux::start(
            0b0010, // FFD mask: bit 1 = FFD ch 1
            0b0010, // FFU mask: bit 1 = FFU ch 1
            0b0010, // 2SRC mask: bit 1 = unit0,pair1
            0b0000, // DVU mask: none
            false,  // MIX
            0b0010, // IPC mask: bit 1 = IPC ch 1
            0b0010, // OPC mask: bit 1 = OPC ch 1
        );

        log::debug!("scux_src_path: streaming started");
    }
}

/// Update the input sample rate while the SRC path is running.
///
/// Call this if the upstream clock drifts (e.g. USB host rate adjustment).
/// Internally updates the 2SRC IFSVR register and reloads the rate via SRCIRR.
///
/// # Safety
/// Writes to live 2SRC registers.  The SRC path must be running.
pub unsafe fn update_input_rate(fin_hz: u32, fout_hz: u32) {
    unsafe {
        let intifs = rza1l_hal::scux::intifs(fin_hz, fout_hz);
        scux::src_update_intifs(SRC_UNIT, SRC_PAIR, intifs);
    }
}

// ── Buffer pointer accessors ─────────────────────────────────────────────────

/// Pointer to the first sample in the SRC path input (TX) buffer (uncached view).
///
/// Write audio at the input rate here.
pub fn tx_buf_start() -> *mut i32 {
    unsafe { (core::ptr::addr_of!(SRC_TX_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *mut i32 }
}

/// One-past-the-end of the input buffer (uncached).
pub fn tx_buf_end() -> *mut i32 {
    unsafe { tx_buf_start().add(SRC_TX_BUF_LEN) }
}

/// Pointer to the first sample in the SRC path output (RX) buffer (uncached view).
///
/// Read rate-converted audio here.
pub fn rx_buf_start() -> *const i32 {
    unsafe {
        (core::ptr::addr_of!(SRC_RX_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *const i32
    }
}

/// One-past-the-end of the output buffer (uncached).
pub fn rx_buf_end() -> *const i32 {
    unsafe { rx_buf_start().add(SRC_RX_BUF_LEN) }
}
