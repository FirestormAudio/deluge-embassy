//! Capture -> playback loopback + metering.
//!
//! Reads captured interleaved `f32` frames from the hosted UAC device and writes
//! them straight back to playback (round-trip), adapting channel counts and
//! publishing a status snapshot for the OLED task.

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use embassy_time::{Instant, Timer};
use log::info;

use deluge_bsp::usb::host::uac;

// ── Shared status (read by the OLED task) ────────────────────────────────
pub(crate) const STAGE_WAITING: u8 = 0;
pub(crate) const STAGE_STREAMING: u8 = 1;

pub(crate) static STAGE: AtomicU8 = AtomicU8::new(STAGE_WAITING);
pub(crate) static CAP_CH: AtomicU8 = AtomicU8::new(0);
pub(crate) static PLAY_CH: AtomicU8 = AtomicU8::new(0);
/// Input peak over the last window, in milli-units of full scale (0..=1000).
pub(crate) static PEAK_MILLI: AtomicU32 = AtomicU32::new(0);
pub(crate) static IN_FPS: AtomicU32 = AtomicU32::new(0);
pub(crate) static OUT_FPS: AtomicU32 = AtomicU32::new(0);

const MAX_CH: usize = 8;
/// Scratch frame budget per read (samples). 256 frames * 8ch.
const SCRATCH: usize = 256 * MAX_CH;

/// Peak absolute value over a sample slice, as milli-full-scale (0..=1000).
fn peak_milli(samples: &[f32]) -> u32 {
    let mut peak = 0.0f32;
    for &s in samples {
        let a = if s < 0.0 { -s } else { s };
        if a > peak {
            peak = a;
        }
    }
    (peak.min(1.0) * 1000.0) as u32
}

/// Remap `in_` (interleaved, `cap_ch`) into `out` (interleaved, `play_ch`),
/// returning samples written. `cap_ch == play_ch` copies through; mono capture
/// duplicates to all playback channels; otherwise copies `min(cap_ch, play_ch)`
/// channels per frame and zero-fills the rest.
fn remap(cap_ch: usize, play_ch: usize, in_: &[f32], out: &mut [f32]) -> usize {
    let frames = in_.len() / cap_ch;
    let mut w = 0;
    for f in 0..frames {
        let src = &in_[f * cap_ch..f * cap_ch + cap_ch];
        for c in 0..play_ch {
            let v = if cap_ch == 1 {
                src[0]
            } else if c < cap_ch {
                src[c]
            } else {
                0.0
            };
            out[w] = v;
            w += 1;
        }
    }
    w
}

#[embassy_executor::task]
pub(crate) async fn loopback_task() {
    info!("loopback: running");
    let mut scratch = [0.0f32; SCRATCH];
    let mut out = [0.0f32; SCRATCH];
    let mut in_frames: u32 = 0;
    let mut out_frames: u32 = 0;
    let mut window_peak: u32 = 0;
    let mut last_stats = Instant::now();
    let mut was_connected = false;

    loop {
        let cap_ch = uac::capture_channels() as usize;
        let play_ch = uac::playback_channels() as usize;

        if cap_ch == 0 {
            if was_connected {
                info!("loopback: device disconnected");
                was_connected = false;
            }
            STAGE.store(STAGE_WAITING, Ordering::Relaxed);
            CAP_CH.store(0, Ordering::Relaxed);
            PLAY_CH.store(0, Ordering::Relaxed);
            Timer::after_millis(100).await;
            continue;
        }
        if !was_connected {
            info!("loopback: device connected cap={}ch play={}ch", cap_ch, play_ch);
            was_connected = true;
        }
        STAGE.store(STAGE_STREAMING, Ordering::Relaxed);
        CAP_CH.store(cap_ch as u8, Ordering::Relaxed);
        PLAY_CH.store(play_ch as u8, Ordering::Relaxed);

        // Drain whole frames that fit the scratch budget.
        // Bound the read so both the input (`scratch`) and remap's output
        // (`frames * play_ch`, into `out`) stay within SCRATCH. Whole cap-frames
        // only, so the read stays frame-aligned. `play_ch == 0` (capture-only) skips
        // remap/playback below, so clamp it to 1 just for this sizing.
        let out_ch = play_ch.max(1);
        let max_frames = (SCRATCH / cap_ch).min(SCRATCH / out_ch);
        let budget = max_frames * cap_ch;
        let n = uac::capture_read(&mut scratch[..budget]);
        if n > 0 {
            let p = peak_milli(&scratch[..n]);
            if p > window_peak {
                window_peak = p;
            }
            in_frames += (n / cap_ch) as u32;
            if play_ch > 0 {
                let w = remap(cap_ch, play_ch, &scratch[..n], &mut out);
                let written = uac::playback_write(&out[..w]);
                out_frames += (written / play_ch) as u32;
            }
        }

        // ~1 Hz stats window.
        if last_stats.elapsed().as_millis() >= 1000 {
            PEAK_MILLI.store(window_peak, Ordering::Relaxed);
            IN_FPS.store(in_frames, Ordering::Relaxed);
            OUT_FPS.store(out_frames, Ordering::Relaxed);
            info!(
                "loopback: in={}f/s out={}f/s peak={}m/1000 ({}ch->{}ch)",
                in_frames, out_frames, window_peak, cap_ch, play_ch
            );
            in_frames = 0;
            out_frames = 0;
            window_peak = 0;
            last_stats = Instant::now();
        }

        // Batch roughly one engine block; capture_read is non-blocking.
        Timer::after_micros(1000).await;
    }
}
