//! SCUX async SRC path for USB host/device TX — sample-rate conversion from
//! engine rate to USB device rate, or vice versa.
//!
//! Routes audio written by the engine (or a USB source) through SCUX
//! asynchronous 2SRC1/0, producing output at the USB device's clock rate.
//! The primary use case is the engine → USB ISO OUT direction when a USB
//! audio device is connected in host mode, or USB ISO IN → engine in device
//! mode at a non-matching rate.
//!
//! ## Signal path
//! ```text
//! CPU SRAM ──DMA ch2──► FFD0_2 ──► IPC2 ──► 2SRC1/0 (async) ──► OPC2 ──► FFU0_2 ──DMA ch8──► CPU SRAM
//! ```
//!
//! ## Usage
//! 1. Call [`init`] with the input and output sample rates.
//! 2. Write input samples ahead of [`tx_current_ptr`].
//! 3. Read converted output samples ahead of [`rx_current_ptr`].
//! 4. Call [`stop`] before calling [`init`] again with new rates.
//!
//! ## Notes
//! - Uses DMA channels 2 (FFD2) and 8 (FFU2) — independent from the DVU
//!   output path (ch 0) and the USB RX SRC path (ch 3/5).
//! - 2SRC unit 1, pair 0 is used (mask bit 2 in SCUX sub-block registers).

use rza1l_hal::UNCACHED_MIRROR_OFFSET;
use rza1l_hal::scux::{self, AudioInfo, IpcSel, MixConfig, OpcSel, SrcConfig, SrcMode};

/// Number of stereo frames in the USB TX SRC input (TX) buffer.
pub const SRC_TX_FRAMES: usize = 1024;
/// Number of stereo frames in the USB TX SRC output (RX) buffer.
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

/// FFD channel used for this path (FFD0 channel 2).
const FFD_CH: u8 = 2;
/// FFU channel used for this path (FFU0 channel 2).
const FFU_CH: u8 = 2;
/// IPC channel.
const IPC_CH: u8 = 2;
/// OPC channel.
const OPC_CH: u8 = 2;
/// 2SRC unit.
const SRC_UNIT: u8 = 1;
/// 2SRC pair within the unit.
const SRC_PAIR: u8 = 0;

/// Initialise the USB TX async SRC path.
///
/// `fin_hz`: nominal input sample rate in Hz (e.g. 44100 from the engine).
/// `fout_hz`: output sample rate in Hz (e.g. 48000 for the USB device).
///
/// After this call:
/// - DMA ch 2 continuously reads from `SRC_TX_BUF` into FFD0_2.
/// - The SCUX 2SRC1/0 block asynchronously converts and writes to FFU0_2.
/// - DMA ch 8 continuously writes the converted audio into `SRC_RX_BUF`.
///
/// Uses `start_path` (read-modify-write on DMACR) so concurrent DVU and USB
/// RX SRC paths are not disturbed.
///
/// # Safety
/// Must be called from a single-threaded context after `rza1l_hal::stb::init()`.
pub unsafe fn init(fin_hz: u32, fout_hz: u32) {
    unsafe {
        log::debug!(
            "scux_usb_tx_path: init async SRC {} Hz → {} Hz",
            fin_hz,
            fout_hz
        );

        let intifs = rza1l_hal::scux::intifs(fin_hz, fout_hz);

        // DMA: TX buffer → FFD0_2 (DMA ch 2)
        let tx_ptr = core::ptr::addr_of!(SRC_TX_BUF.0[0]) as *const u32;
        let tx_bytes = SRC_TX_BUF_LEN * core::mem::size_of::<i32>();
        scux::init_ffd_dma(FFD_CH, crate::system::SCUX_FFD2_DMA_CH, tx_ptr, tx_bytes);

        // DMA: FFU0_2 → RX buffer (DMA ch 8)
        let rx_ptr = core::ptr::addr_of_mut!(SRC_RX_BUF.0[0]) as *mut u32;
        let rx_bytes = SRC_RX_BUF_LEN * core::mem::size_of::<i32>();
        scux::init_ffu_dma(FFU_CH, crate::system::SCUX_FFU2_DMA_CH, rx_ptr, rx_bytes);

        // Sub-block config
        scux::configure_ipc(IPC_CH, IpcSel::FfdToSrcAsync);
        scux::configure_opc(OPC_CH, OpcSel::ToFfu);
        scux::configure_ffd(FFD_CH, AudioInfo::STEREO_24, 8);
        scux::configure_ffu(FFU_CH, AudioInfo::STEREO_24, 8);

        // 2SRC1/0: async mode
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

        // Start: FFD2, FFU2, 2SRC unit1/pair0 (mask bit 2), no DVU, no MIX,
        // IPC2, OPC2.  Use start_path (read-modify-write on DMACR) so the DVU
        // output path and the USB RX SRC path are not disturbed.
        scux::start_path(
            0b0100, // FFD mask: bit 2 = FFD ch 2
            0b0100, // FFU mask: bit 2 = FFU ch 2
            0b0100, // 2SRC mask: bit 2 = unit1,pair0
            0b0000, // DVU mask: none
            false,  // MIX
            0b0100, // IPC mask: bit 2 = IPC ch 2
            0b0100, // OPC mask: bit 2 = OPC ch 2
        );

        log::debug!("scux_usb_tx_path: streaming started");
    }
}

/// Update the input sample rate while the SRC path is running.
///
/// Call this if the upstream clock drifts (e.g. engine rate change).
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

/// Pointer to the first sample in the input (TX) buffer (uncached view).
///
/// Write audio at `fin_hz` rate here, advancing ahead of [`tx_current_ptr`].
pub fn tx_buf_start() -> *mut i32 {
    unsafe { (core::ptr::addr_of!(SRC_TX_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *mut i32 }
}

/// One-past-the-end of the input buffer (uncached).
pub fn tx_buf_end() -> *mut i32 {
    unsafe { tx_buf_start().add(SRC_TX_BUF_LEN) }
}

/// Pointer to the first sample in the output (RX) buffer (uncached view).
///
/// Read rate-converted audio here, up to [`rx_current_ptr`].
pub fn rx_buf_start() -> *const i32 {
    unsafe {
        (core::ptr::addr_of!(SRC_RX_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *const i32
    }
}

/// One-past-the-end of the output buffer (uncached).
pub fn rx_buf_end() -> *const i32 {
    unsafe { rx_buf_start().add(SRC_RX_BUF_LEN) }
}

/// Current DMA write position in the RX buffer (uncached), aligned to one
/// stereo frame.  Read up to this point to drain converted audio.
pub fn rx_current_ptr() -> *const i32 {
    let ch = crate::system::SCUX_FFU2_DMA_CH;
    let crda = unsafe { rza1l_hal::dmac::current_dst(ch) };
    let aligned = crda & !7u32;
    (aligned as usize + rza1l_hal::UNCACHED_MIRROR_OFFSET) as *const i32
}

/// Current DMA read position in the TX buffer (uncached), aligned to one
/// stereo frame.  Write ahead of this pointer.
pub fn tx_current_ptr() -> *mut i32 {
    let ch = crate::system::SCUX_FFD2_DMA_CH;
    let crsa = unsafe { rza1l_hal::dmac::current_src(ch) };
    let aligned = crsa & !7u32;
    (aligned as usize + rza1l_hal::UNCACHED_MIRROR_OFFSET) as *mut i32
}

/// Stop the USB TX SRC path and release its SCUX sub-blocks.
///
/// After this call [`init`] can be called again with a new rate pair.
///
/// # Safety
/// Writes to live SCUX registers.
pub unsafe fn stop() {
    unsafe {
        rza1l_hal::scux::stop_path(
            0b0100, // FFD mask: FFD ch 2
            0b0100, // FFU mask: FFU ch 2
            0b0100, // 2SRC mask: unit1,pair0
            0b0000, false, 0b0100, // IPC mask: IPC ch 2
            0b0100, // OPC mask: OPC ch 2
        );
    }
}
