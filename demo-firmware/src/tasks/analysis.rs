//! Audio analysis task — FFT magnitude spectrum and waveform snapshot.
//!
//! Runs every 50 ms when the SSI RX DMA is active.  Publishes results to the
//! shared statics below for consumption by `oled_task` and `rgb_task`.

use core::arch::arm::{vcvtq_f32_s32, vhaddq_s32, vld2q_s32, vmulq_n_f32};
use core::sync::atomic::{AtomicU32, Ordering};

use deluge_bsp::scux_dvu_path;
use embassy_futures::yield_now;
use embassy_time::Timer;

use crate::tasks::audio::USB_AUDIO_STREAMING;
use deluge_fft::{Complex, RealFft, apply_hann_window_real};

// ---------------------------------------------------------------------------
// Shared output buffers
// ---------------------------------------------------------------------------

/// Incremented each time a fresh analysis frame is published.
///
/// Consumers can detect new data by comparing against a cached epoch value.
pub(crate) static ANALYSIS_EPOCH: AtomicU32 = AtomicU32::new(0);

/// 128-point waveform snapshot, normalised to `[-1, 1]`.
///
/// Written only by `analysis_task`; read only by `oled_task`.
/// Safe in a single-threaded cooperative executor with no concurrent writers.
pub(crate) static mut WAVEFORM: [f32; 128] = [0.0; 128];

/// 257-bin squared-magnitude spectrum (bins 0 … N/2 of a 512-point FFT),
/// normalised so that a full-scale sine wave produces a peak of ≈ 1.0.
///
/// Stored as squared magnitude to avoid 257 `sqrtf` calls per analysis frame;
/// `rgb_task` takes one `sqrt` per display column (18 total) after averaging.
///
/// Written only by `analysis_task`; read only by `rgb_task`.
pub(crate) static mut SPECTRUM: [f32; 257] = [0.0; 257];

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// FFT window length in left-channel samples (≈ 11.6 ms at 44.1 kHz).
const FFT_N: usize = 512;

/// SIMD lane width — 4 × f32 maps to a NEON `float32x4_t` on Cortex-A9.
const LANES: usize = 4;

/// Number of waveform points (equals `oled::WIDTH = 128`).
const WAVE_PTS: usize = 128;

// ---------------------------------------------------------------------------
// Deinterleaved sample extraction
// ---------------------------------------------------------------------------

/// Extract `FFT_N` mid-channel (L+R average) samples from the interleaved
/// stereo RX ring buffer into `out`, converting from MSB-aligned `i32` to
/// normalised `f32`.
///
/// Uses `vld2q_s32` (load + deinterleave in one instruction) followed by
/// `vhaddq_s32` (halving add — `(L+R)>>1`, overflow-free) before the
/// float conversion — 64 iterations instead of 512 scalar loads.
///
/// # Safety
/// - `base` must be the uncached alias of the SSI RX DMA buffer.
/// - All `FFT_N * 2` slots starting at `start` (wrapping at `buf_len`) must
///   have been committed by the DMA before this call.
/// - `buf_len` and `start` must both be even.
#[target_feature(enable = "neon,v7")]
unsafe fn extract_mid_channel(
    base: *const i32,
    buf_len: usize,
    start: usize,
    out: &mut [f32; FFT_N],
) {
    // Split the 1024-slot window at the ring-buffer wrap point (if any).
    // All accesses are standard (non-volatile) reads from uncached SRAM —
    // coherency with DMA is guaranteed by the uncached alias, not volatile.
    let total_slots = FFT_N * 2; // 1024
    let slots_to_end = buf_len - start;

    if total_slots <= slots_to_end {
        // Common case: no wrap.
        extract_segment(unsafe { base.add(start) }, total_slots, out);
    } else {
        // Window straddles the ring-buffer end.
        let seg1_slots = slots_to_end; // [start, buf_len)
        let seg2_slots = total_slots - seg1_slots; // [0, seg2_slots)
        let mid = seg1_slots / 2; // left samples in first segment
        extract_segment(unsafe { base.add(start) }, seg1_slots, &mut out[..mid]);
        extract_segment(base, seg2_slots, &mut out[mid..]);
    }
}

/// Process one contiguous stereo `i32` span into normalised `f32` mid-channel
/// samples (L+R average), 8 interleaved slots at a time via NEON.
///
/// `vld2q_s32` deinterleaves into left/right in one instruction;
/// `vhaddq_s32` computes `(L+R)>>1` with no overflow risk.
#[target_feature(enable = "neon,v7")]
#[inline]
fn extract_segment(ptr: *const i32, slots: usize, out: &mut [f32]) {
    const SCALE: f32 = 1.0 / 2_147_483_648.0_f32;
    let mut i = 0usize;

    // NEON path: vld2q_s32 → vhaddq_s32 → vcvtq_f32_s32 → vmulq_n_f32 → store.
    // 4 mid-channel f32 outputs per iteration, 64 iterations for FFT_N=512.
    while i + 8 <= slots {
        // SAFETY: caller guarantees ptr+i..ptr+i+8 is valid uncached SRAM.
        let deint = unsafe { vld2q_s32(ptr.add(i)) };
        // deint.0 = [L0,L1,L2,L3], deint.1 = [R0,R1,R2,R3]
        // vhaddq_s32: (L+R)>>1 per lane — halving add avoids i32 overflow.
        let mid = vhaddq_s32(deint.0, deint.1);
        let mid_f32 = vmulq_n_f32(vcvtq_f32_s32(mid), SCALE);
        // Store 4 floats directly into the output slice.
        unsafe {
            core::ptr::copy_nonoverlapping(
                &mid_f32 as *const _ as *const f32,
                out.as_mut_ptr().add(i / 2),
                4,
            );
        }
        i += 8;
    }

    // Scalar tail (< 4 stereo pairs remaining; at most 3 iterations).
    while i < slots {
        let l = unsafe { ptr.add(i).read() };
        let r = unsafe { ptr.add(i + 1).read() };
        out[i / 2] = ((l >> 1) + (r >> 1)) as f32 * SCALE;
        i += 2;
    }
}

// ---------------------------------------------------------------------------
// Task
// ---------------------------------------------------------------------------

/// Periodically analyses the SSI RX ring buffer.
///
/// Runs only while the host is actively sending USB speaker audio.
#[embassy_executor::task]
pub(crate) async fn analysis_task() {
    // Give hardware init and other tasks time to settle.
    Timer::after_millis(500).await;

    let base = scux_dvu_path::tx_buf_start() as *const i32;
    let buf_len = scux_dvu_path::DVU_PATH_BUF_LEN; // interleaved i32 slots (stereo)

    loop {
        Timer::after_millis(50).await;

        let cur_ptr = scux_dvu_path::tx_current_ptr() as *const i32;
        let streaming = USB_AUDIO_STREAMING.load(Ordering::Acquire);

        if !streaming {
            continue;
        }

        // ── Extract FFT_N left-channel samples ending at cur_ptr ────────────
        //
        // The TX ring buffer is interleaved stereo: slot 0=L, 1=R, 2=L, 3=R …
        // cur_ptr is the next slot the DMA will read (already committed data
        // is at cur_ptr-1 and earlier).
        //
        // We snap cur_ptr down to the nearest even (left-channel) index, then
        // walk back FFT_N × 2 stereo slots to read FFT_N mid-channel samples.

        let cur_off = (cur_ptr as usize - base as usize) / core::mem::size_of::<i32>();
        // Round down to the most recent left-channel sample (even index).
        let head = cur_off & !1usize;
        // Start index (wraps inside the ring).
        let start = head.wrapping_sub(FFT_N * 2) % buf_len;

        // Stack-allocate the working buffers (~2 kB, down from ~4 kB).
        let mut wave_raw = [0.0f32; FFT_N];

        // SAFETY: `base` is the uncached SRAM alias; all slots before cur_ptr
        // have been read by the DMA and were written by uac2_task.  Regular
        // (non-volatile) loads are correct and required for SIMD vectorisation.
        // buf_len and start are both even, so every segment boundary is
        // stereo-pair-aligned.
        unsafe { extract_mid_channel(base, buf_len, start, &mut wave_raw) };

        // Yield between each heavy stage so uac2_task can service USB packets.
        yield_now().await;

        // ── Waveform snapshot (downsample FFT_N → WAVE_PTS) ─────────────────
        //
        // Pick every (FFT_N / WAVE_PTS) = 4th sample for a time-domain view.
        const STEP: usize = FFT_N / WAVE_PTS;
        unsafe {
            for i in 0..WAVE_PTS {
                WAVEFORM[i] = wave_raw[i * STEP];
            }
        }

        // ── FFT → magnitude spectrum ─────────────────────────────────────────
        // Apply Hann window to the real samples, then run a real-input FFT
        // (internally N/2-point radix-4 complex FFT + post-processing).
        // ~2× faster than a full N-point complex FFT.  Stack usage also halved.
        apply_hann_window_real(&mut wave_raw);
        yield_now().await;
        let mut spec_cx = [Complex::ZERO; FFT_N / 2 + 1]; // 257 bins — halved stack
        RealFft::<FFT_N, LANES>::process(&wave_raw, &mut spec_cx);
        yield_now().await;

        // Store squared magnitude, normalised so a full-scale sine → peak ≈ 1.0.
        // Using norm_sq avoids 257 sqrtf calls here; rgb_task takes one sqrt per
        // column (18 total) after averaging, preserving identical display output.
        let norm = 4.0 / FFT_N as f32;
        let norm_sq = norm * norm;
        unsafe {
            for i in 0..257 {
                SPECTRUM[i] = spec_cx[i].norm_sq() * norm_sq;
            }
        }

        // Signal consumers that fresh data is available.
        ANALYSIS_EPOCH.fetch_add(1, Ordering::Release);
        deluge_bsp::oled::notify_redraw();
    }
}
