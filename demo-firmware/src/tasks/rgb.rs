use embassy_futures::yield_now;
use embassy_time::Timer;
use log::info;

use crate::tasks::analysis::SPECTRUM;
use crate::tasks::audio::USB_AUDIO_STREAMING;
use deluge_bsp::pads::{pad_get, pad_id_from_xy};
use deluge_bsp::pic;

#[inline(always)]
fn log10f(x: f32) -> f32 {
    unsafe extern "C" {
        fn log10f(x: f32) -> f32;
    }
    unsafe { log10f(x) }
}

/// Convert HSV to `[r, g, b]` (all 0–255).
///
/// `h`: hue 0–255 (mapped to 0–360°).  `s`: saturation 0–255.  `v`: value 0–255.
fn hsv_to_rgb(h: u8, s: u8, v: u8) -> [u8; 3] {
    if s == 0 {
        return [v, v, v];
    }
    let h6 = (h as u32 * 6) >> 8; // sector 0–5
    let f = ((h as u32 * 6) & 0xFF) as u8; // fractional part 0–255
    let p = ((v as u32 * (255 - s as u32)) >> 8) as u8;
    let q = ((v as u32 * (255 - ((s as u32 * f as u32) >> 8))) >> 8) as u8;
    let t = ((v as u32 * (255 - ((s as u32 * (255 - f as u32)) >> 8))) >> 8) as u8;
    match h6 {
        0 => [v, t, p],
        1 => [q, v, p],
        2 => [p, v, t],
        3 => [p, q, v],
        4 => [t, p, v],
        _ => [v, p, q],
    }
}

/// Build the 9-pair colour array for the spectrum visualisation.
///
/// Maps FFT magnitude bins onto 18 frequency-band columns using logarithmic
/// (musical) spacing.  Each column covers roughly a minor-third (~1.35×)
/// wider frequency range than the previous, matching human pitch perception.
///
/// Bin boundaries (each bin ≈ 86 Hz wide at 44.1 kHz / 512 pt FFT):
///   col  0:  bins   1–  1  (~  86 Hz — sub-bass)
///   col  4:  bins   5–  6  (~ 430 Hz — bass)
///   col  8:  bins  16– 21  (~1.4 kHz — mid)
///   col 12:  bins  54– 72  (~4.7 kHz — presence)
///   col 17:  bins 245–256  (~21 kHz  — brilliance)
fn spectrum_colours() -> [[[u8; 3]; 16]; 9] {
    const N_COLS: usize = 18;

    // 19 boundaries define 18 half-open bin ranges [BIN_LO[c], BIN_LO[c+1]).
    // Spaced ~1.35× per step (≈ minor third) from bin 1 to bin 257.
    const BIN_LO: [usize; 19] = [
        1, 2, 3, 4, 5, 7, 9, 12, 16, 22, 29, 40, 54, 73, 99, 134, 181, 245, 257,
    ];

    // SAFETY: written only by analysis_task; no concurrent writer in a
    //         single-threaded cooperative executor.
    let spectrum = unsafe { &*core::ptr::addr_of!(SPECTRUM) };

    let mut pairs: [[[u8; 3]; 16]; 9] = [[[0u8; 3]; 16]; 9];

    for col in 0..N_COLS {
        let bin_lo = BIN_LO[col];
        let bin_hi = BIN_LO[col + 1]; // exclusive; always ≥ bin_lo + 1

        // Average squared magnitude (power) across this column's bins.
        let mut avg_sq = 0.0f32;
        for &val in &spectrum[bin_lo..bin_hi] {
            avg_sq += val;
        }
        avg_sq /= (bin_hi - bin_lo) as f32;

        // Map power to rows using dB scaling.
        // SPECTRUM stores norm_sq so avg_sq is already power in [0, 1].
        // Range: -50 dBFS (dark) → -5 dBFS (full 8 rows).
        let filled = if avg_sq > 1e-10 {
            let db = 10.0 * log10f(avg_sq); // e.g. avg_sq=1.0 → 0 dBFS
            ((db + 50.0) * (8.0 / 45.0)).clamp(0.0, 8.0) as usize
        } else {
            0
        };

        // Hue: bass columns are red (hue=0), treble columns are violet (hue≈200).
        let hue = (col as u16 * 200 / (N_COLS as u16 - 1)) as u8;
        let rgb = hsv_to_rgb(hue, 220, 180);

        let pair = col / 2;
        let side = col % 2;

        for row in 0..8usize {
            let color = if row < filled { rgb } else { [0u8; 3] };
            if side == 0 {
                pairs[pair][row] = color; // left column of the pair
            } else {
                pairs[pair][8 + row] = color; // right column of the pair
            }
        }
    }

    pairs
}

/// RGB pad-grid demo task.
///
/// Renders the current pad state to the 18 × 8 RGB LED matrix every 50 ms.
///
/// - **Lit pads** glow with a per-column hue that slowly cycles over time,
///   making the whole grid rotate through the rainbow as pads are held.
/// - **Dark pads** are off (RGB 0 0 0).
///
/// The full grid is sent as 9 × `SET_COLOUR_FOR_TWO_COLUMNS` messages
/// followed by `DONE_SENDING_ROWS` (byte 240) to trigger the PIC's refresh.
#[embassy_executor::task]
pub(crate) async fn rgb_task() {
    pic::wait_ready().await;

    let mut hue_offset: u8 = 0;
    // C: per-pair cache of the colours sent in the last frame.
    // Initialised to all-zeros so the first frame detects any lit pads as
    // changed.  Pairs that remain all-black are never re-sent.
    let mut last_sent: [[[u8; 3]; 16]; 9] = [[[0u8; 3]; 16]; 9];

    info!("rgb_task: started");

    loop {
        if USB_AUDIO_STREAMING.load(core::sync::atomic::Ordering::Acquire) {
            // ── Spectrum view ─────────────────────────────────────────────
            let all_colours = spectrum_colours();
            for pair in 0u8..9 {
                let colours = all_colours[pair as usize];
                if colours != last_sent[pair as usize] {
                    pic::set_column_pair_rgb(pair, &colours).await;
                    last_sent[pair as usize] = colours;
                }
                yield_now().await;
            }
        } else {
            // ── Pad glow view ─────────────────────────────────────────────
            for pair in 0u8..9 {
                let col_l = pair * 2;
                let col_r = col_l + 1;
                let mut colours = [[0u8; 3]; 16];

                for row in 0..8u8 {
                    // Left column: pad (col_l, row) — row 0 = bottom of both grid and PIC protocol
                    if pad_get(pad_id_from_xy(col_l, row)) {
                        let hue = ((col_l as u16 * 256 / 18) as u8).wrapping_add(hue_offset);
                        colours[row as usize] = hsv_to_rgb(hue, 220, 180);
                    }
                    // Right column
                    if pad_get(pad_id_from_xy(col_r, row)) {
                        let hue = ((col_r as u16 * 256 / 18) as u8).wrapping_add(hue_offset);
                        colours[8 + row as usize] = hsv_to_rgb(hue, 220, 180);
                    }
                }

                // Skip pairs whose output colours are identical to the last
                // transmission.  Saves the 49-byte DMA burst for dark pairs and
                // any pair the animation hasn't changed since the previous frame.
                if colours != last_sent[pair as usize] {
                    pic::set_column_pair_rgb(pair, &colours).await;
                    last_sent[pair as usize] = colours;
                }

                // Explicit yield between pairs so uac2_task can service any
                // USB packets that arrived while we were busy.
                yield_now().await;
            }
        }
        pic::done_sending_rows().await;

        hue_offset = hue_offset.wrapping_add(2); // full rainbow cycle every ~6.4 s
        Timer::after_millis(50).await;
    }
}
