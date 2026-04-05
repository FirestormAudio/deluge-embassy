use core::sync::atomic::{AtomicBool, Ordering};
use deluge_bsp::scux_dvu_path;
use embassy_usb::driver::{Endpoint as _, EndpointIn as _, EndpointOut as _};
use log::info;
use rza1l_hal::ssi;

use deluge_bsp::usb::classes::audio::{USB_BITS_PER_SAMPLE, USB_CAPTURE_BITS_PER_SAMPLE};

/// `true` while the host is actively sending UAC2 speaker data.
///
/// This is the UI-facing signal for whether the analyzer views should be
/// shown. It deliberately tracks actual USB audio traffic, not whether the
/// SCUX/SSI output path is running with dither.
pub(crate) static USB_AUDIO_STREAMING: AtomicBool = AtomicBool::new(false);

/// Small LFSR dither to prevent codec DC auto-mute.
///
/// The Akiyama codec auto-mutes after ~8192 consecutive identical samples
/// (≈0.19 s at 44.1 kHz).  We mix a ±16-LSB (of 24-bit) noise signal into
/// every sample written to the SSI TX buffer to keep the codec awake during
/// silence.  This matches the behaviour of the original C firmware.
#[inline]
fn dither_sample(lfsr: &mut u32) -> i32 {
    let bit = *lfsr & 1;
    *lfsr >>= 1;
    if bit != 0 {
        *lfsr ^= 0xB400;
    }
    // ±16 LSBs in the 24-bit range: noise in bits [12:8] of the 32-bit word.
    ((*lfsr & 0x1F) as i32 - 0x10) << 8
}

/// Fill the entire SSI TX buffer with low-level dither noise.
///
/// Called once at startup and after the USB stream ends to ensure the codec
/// never sees a burst of identical samples.
pub(crate) fn fill_tx_with_dither() {
    let buf_start = scux_dvu_path::tx_buf_start();
    let buf_end = scux_dvu_path::tx_buf_end();
    let mut lfsr: u32 = 0xACE1;
    let mut p = buf_start;
    while p < buf_end {
        unsafe {
            p.write_volatile(dither_sample(&mut lfsr));
            p = p.add(1);
        }
    }
}

/// UAC2 speaker task — reads isochronous OUT packets from the host and writes
/// them to the SSI TX buffer, keeping a fixed write-ahead lead ahead of the
/// DMA read head.
///
/// Packet format: stereo 24-bit LE PCM (3 bytes per mono sample).
/// SSI format: MSB-aligned 32-bit word (data in bits [31:8]).
///
/// Speaker-enable logic (matching the C firmware):
/// - `SPEAKER_ENABLE` (P4.1) is raised when the stream is active AND no
///   headphone or line-out jack is inserted.
/// - It is dropped when the stream stops (no data for >200 ms).
#[embassy_executor::task]
pub(crate) async fn uac2_task(mut ep_out: deluge_bsp::usb::Rusb1EndpointOut) {
    /// How far ahead of the DMA read head to keep the write pointer (mono i32 slots).
    /// 2048 slots = 1024 stereo frames ≈ 23.2 ms at 44.1 kHz.
    /// Sized to absorb SCUX FFD FIFO burst DMA and USB SOF jitter comfortably.
    const WRITE_AHEAD: usize = 2048;
    /// If no ISO packet arrives within this window the stream is considered stopped.
    /// ISO packets arrive once per SOF (every 1 ms at full-speed); 50 ms is well
    /// beyond any expected jitter but short enough that silence follows disconnect
    /// almost immediately.
    const READ_TIMEOUT_MS: u64 = 50;

    info!("uac2_task: waiting for USB enumeration");
    ep_out.wait_enabled().await;
    info!("uac2_task: ISO OUT endpoint enabled");

    let buf_start = scux_dvu_path::tx_buf_start();
    let buf_end = scux_dvu_path::tx_buf_end();
    let buf_len = scux_dvu_path::DVU_PATH_BUF_LEN;

    // Pre-fill with dither so the codec stays alive before streaming begins.
    fill_tx_with_dither();

    let mut write_ptr: *mut i32 = buf_start;
    let mut streaming = false;

    // 288 bytes: max ISO packet for stereo 24-bit at 48 kHz (48 frames × 6 B).
    let mut pkt: [u8; 288] = [0; 288];
    let mut lfsr: u32 = 0xACE1;

    // Diagnostics: track min/max packet size per second (8000 µSOFs).
    let mut diag_count: u32 = 0;
    let mut diag_min: usize = 288;
    let mut diag_max: usize = 0;

    loop {
        // Wrap the ISO read with a timeout so that a physical disconnect (where
        // BRDYENB stays asserted and NRDY may never fire) is still detected.
        match embassy_time::with_timeout(
            embassy_time::Duration::from_millis(READ_TIMEOUT_MS),
            ep_out.read(&mut pkt),
        )
        .await
        {
            Err(_timeout) => {
                // No packet for READ_TIMEOUT_MS — stream stalled or disconnected.
                if streaming {
                    info!("uac2_task: stream stopped (read timeout)");
                    streaming = false;
                    USB_AUDIO_STREAMING.store(false, Ordering::Release);
                    unsafe { rza1l_hal::gpio::write(4, 1, false) }; // SPEAKER_ENABLE off
                    fill_tx_with_dither();
                }
                continue;
            }
            Ok(Err(_)) => {
                // Endpoint disabled — host stopped streaming or disconnected.
                if streaming {
                    info!("uac2_task: stream stopped (endpoint disabled)");
                    streaming = false;
                    USB_AUDIO_STREAMING.store(false, Ordering::Release);
                    unsafe { rza1l_hal::gpio::write(4, 1, false) };
                    fill_tx_with_dither();
                }
                // Wait for the endpoint to become active again.
                ep_out.wait_enabled().await;
                info!("uac2_task: ISO OUT endpoint re-enabled");
                continue;
            }
            Ok(Ok(0)) => {
                // Zero-length packet — host is pacing but sending no audio.
                continue;
            }
            Ok(Ok(bytes_read)) => {
                // Packet size diagnostics — log once per ~8000 packets (≈1 s).
                if bytes_read > 0 {
                    if bytes_read < diag_min {
                        diag_min = bytes_read;
                    }
                    if bytes_read > diag_max {
                        diag_max = bytes_read;
                    }
                    diag_count += 1;
                    if diag_count >= 8_000 {
                        info!(
                            "uac2_task: pkt size min={} max={} (last 8k pkts)",
                            diag_min, diag_max
                        );
                        diag_count = 0;
                        diag_min = 288;
                        diag_max = 0;
                    }
                }

                let dma_ptr = scux_dvu_path::tx_current_ptr();
                // CRSA can briefly read one-past-the-end during the DMA link-descriptor
                // reload at the buffer wrap boundary.  Wrap into [0, buf_len) so the
                // ahead calculation below doesn't see a spuriously small value.
                let dma_off = (unsafe { dma_ptr.offset_from(buf_start) } as usize) % buf_len;

                if !streaming {
                    // First data after silence — re-anchor the write pointer
                    // WRITE_AHEAD slots ahead of the DMA, snapped to a stereo
                    // frame boundary (even offset) to keep L/R channels correct.
                    let mut off = (dma_off + WRITE_AHEAD) % buf_len;
                    off &= !1; // snap to even (stereo frame boundary)
                    write_ptr = unsafe { buf_start.add(off) };
                    streaming = true;
                    USB_AUDIO_STREAMING.store(true, Ordering::Release);

                    // Gate speaker: on if no headphone / line-out detected.
                    let hp = unsafe { rza1l_hal::gpio::read_pin(6, 5) };
                    let lol = unsafe { rza1l_hal::gpio::read_pin(6, 3) };
                    let lor = unsafe { rza1l_hal::gpio::read_pin(6, 4) };
                    unsafe { rza1l_hal::gpio::write(4, 1, !hp && !lol && !lor) };
                    info!("uac2_task: streaming started");
                }

                // ── Underrun guard ──────────────────────────────────────────────────
                // If the DMA has caught up to the write pointer, re-anchor.
                {
                    let wr_off = unsafe { write_ptr.offset_from(buf_start) } as usize;
                    let ahead = (wr_off + buf_len - dma_off) % buf_len;
                    if ahead < WRITE_AHEAD / 2 {
                        info!(
                            "uac2_task: underrun (ahead={} < {}), re-anchoring",
                            ahead,
                            WRITE_AHEAD / 2
                        );
                        let mut off = (dma_off + WRITE_AHEAD) % buf_len;
                        off &= !1;
                        write_ptr = unsafe { buf_start.add(off) };
                    }
                }

                // ── Convert USB samples → MSB-aligned 32-bit SSI ───────────
                // Supports 16-bit (2 B/sample) and 24-bit (3 B/sample) LE PCM.
                // SSI expects audio in bits [31:8]; 24-bit shifts left by 8,
                // 16-bit shifts left by 16.  Dither prevents codec auto-mute.
                let bytes_per_sample = (USB_BITS_PER_SAMPLE.load(Ordering::Relaxed) / 8) as usize;
                let num_samples = bytes_read / bytes_per_sample;
                let src = &pkt[..bytes_read];
                for i in 0..num_samples {
                    let sample = if bytes_per_sample == 3 {
                        let b0 = src[i * 3] as u32;
                        let b1 = src[i * 3 + 1] as u32;
                        let b2 = src[i * 3 + 2] as u32;
                        (b0 << 8 | b1 << 16 | b2 << 24) as i32
                    } else {
                        // 16-bit signed LE → MSB-align in 32 bits
                        let v = (src[i * 2] as u16 | (src[i * 2 + 1] as u16) << 8) as i16;
                        (v as i32) << 16
                    }
                    .wrapping_add(dither_sample(&mut lfsr));
                    unsafe {
                        write_ptr.write_volatile(sample);
                        write_ptr = write_ptr.add(1);
                        if write_ptr >= buf_end {
                            write_ptr = buf_start;
                        }
                    }
                }
            }
        }
    }
}

/// UAC2 microphone capture task — reads from SSI RX and sends over ISO IN.
///
/// The ISO IN packet cadence provides **implicit feedback** for the speaker
/// stream: the host observes the IN packet rate and adapts how much data it
/// sends per SOF, correcting long-term clock drift without a separate feedback
/// endpoint.
#[embassy_executor::task]
pub(crate) async fn uac2_mic_task(mut ep_in: deluge_bsp::usb::Rusb1EndpointIn) {
    info!("uac2_mic_task: waiting for capture enable");
    ep_in.wait_enabled().await;
    info!("uac2_mic_task: capture enabled");

    let rx_start = ssi::rx_buf_start();
    let rx_len = ssi::RX_BUF_LEN;
    let mut read_ptr = ssi::rx_current_ptr();

    let mut pkt = [0u8; 288];

    loop {
        // Compute how many stereo frames the SSI RX DMA has captured since we
        // last sent.  This ties the implicit feedback signal directly to the
        // hardware AUDIO_X1 crystal clock rather than an assumed call rate,
        // so the host adapts its OUT rate to exactly match the SSI regardless
        // of async executor scheduling jitter.
        //
        // Over any long interval: total IN frames sent = total SSI frames
        // captured = 44 100 Hz.  Host converges to sending 44 100 frames/sec
        // OUT, eliminating the systematic rate mismatch that caused underruns.
        let bytes_per_sample = (USB_CAPTURE_BITS_PER_SAMPLE.load(Ordering::Relaxed) / 8) as usize;
        let max_frames = pkt.len() / (2 * bytes_per_sample);
        let rx_hw_off = unsafe { (ssi::rx_current_ptr().offset_from(rx_start) as usize) % rx_len };
        let read_off = unsafe { read_ptr.offset_from(rx_start) as usize };
        let captured_mono = (rx_hw_off + rx_len - read_off) % rx_len;
        // Integer divide by 2 for stereo frames; capped so we never exceed the
        // packet buffer.  The fractional remainder carries naturally into the
        // next call via the unchanged read_ptr, giving correct Bresenham-style
        // alternating 5/6 frame packets averaging exactly 44 100 Hz.
        let frames = (captured_mono / 2).min(max_frames);

        // Read `frames` stereo pairs from SSI RX; convert MSB-aligned i32 → USB PCM.
        // Format matches the active capture alt setting (16-bit or 24-bit LE).
        let nbytes = frames * 2 * bytes_per_sample;
        for i in 0..frames * 2 {
            let sample = unsafe { read_ptr.read_volatile() };
            // SSI audio is in bits [31:8]; shift right to get the significant bits.
            let off = i * bytes_per_sample;
            if bytes_per_sample == 3 {
                let val = (sample >> 8) as u32;
                pkt[off] = (val & 0xFF) as u8;
                pkt[off + 1] = ((val >> 8) & 0xFF) as u8;
                pkt[off + 2] = ((val >> 16) & 0xFF) as u8;
            } else {
                // 16-bit: keep the top 16 bits of the MSB-aligned sample
                let val = (sample >> 16) as u16;
                pkt[off] = (val & 0xFF) as u8;
                pkt[off + 1] = ((val >> 8) & 0xFF) as u8;
            }
            unsafe {
                read_ptr = read_ptr.add(1);
                if read_ptr >= rx_start.add(rx_len) {
                    read_ptr = rx_start;
                }
            }
        }

        match ep_in.write(&pkt[..nbytes]).await {
            Ok(()) => {}
            Err(_) => {
                info!("uac2_mic_task: capture stopped");
                ep_in.wait_enabled().await;
                info!("uac2_mic_task: capture re-enabled");
                // Re-anchor behind current DMA write position.
                read_ptr = ssi::rx_current_ptr();
            }
        }
    }
}
