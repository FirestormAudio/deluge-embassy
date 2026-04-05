use core::sync::atomic::{AtomicBool, Ordering};
use log::info;

use crate::tasks::analysis::WAVEFORM;
use crate::tasks::audio::USB_AUDIO_STREAMING;
use deluge_bsp::oled;
use deluge_bsp::pads::{pad_get, pad_id_from_xy};
use deluge_bsp::pic;

// ---------------------------------------------------------------------------
// OLED layout constants
// ---------------------------------------------------------------------------
//
// 18 pad columns × 8 pad rows — each cell is CELL_W × CELL_H pixels.
// 18 × 7 = 126 px ≤ 128 (2 px right margin);  8 × 6 = 48 px exactly.

const CELL_W: usize = 7; // px per pad column  (18 × 7 = 126 ≤ 128)
const CELL_H: usize = 5; // px per pad row      ( 8 × 5 =  40 ≤ 43 visible)
const FILL_W: usize = 5; // CELL_W − 2px borders
const FILL_H: usize = 3; // CELL_H − 2px borders
/// First visible OLED row (rows 0–4 are off-panel, C constant OLED_MAIN_TOPMOST_PIXEL=5).
const TOPMOST: usize = 5;

// ---------------------------------------------------------------------------
// CDC-supplied framebuffer
// ---------------------------------------------------------------------------
//
// When a CDC host sends MSG_TO_UPDATE_DISPLAY, `set_cdc_display` stores the
// 768-byte frame here and sets CDC_FRAME_VALID.  The oled_task uses this
// framebuffer in preference to re-rendering from PAD_BITS.  On
// MSG_TO_CLEAR_DISPLAY or host disconnect, `clear_cdc_display` clears the
// flag and oled_task falls back to pad rendering.

static mut CDC_FRAME: [u8; oled::FRAME_BYTES] = [0u8; oled::FRAME_BYTES];
static CDC_FRAME_VALID: AtomicBool = AtomicBool::new(false);

/// Store a host-supplied OLED framebuffer and request a redraw.
///
/// `data` must be exactly [`oled::FRAME_BYTES`] bytes (768), laid out as
/// `data[page * oled::WIDTH + col]` (page-major, matching the SSD1309 wire
/// format).  Any extra bytes in `data` are ignored.
pub(crate) fn set_cdc_display(data: &[u8]) {
    let n = data.len().min(oled::FRAME_BYTES);
    // Safety: single-threaded cooperative executor; no concurrent writer.
    unsafe {
        CDC_FRAME[..n].copy_from_slice(&data[..n]);
    }
    CDC_FRAME_VALID.store(true, Ordering::Release);
    oled::notify_redraw();
}

/// Discard the host-supplied framebuffer and fall back to pad rendering.
pub(crate) fn clear_cdc_display() {
    CDC_FRAME_VALID.store(false, Ordering::Release);
    oled::notify_redraw();
}

/// Render the latest waveform snapshot from `analysis_task` as a dot-scope.
///
/// Render the latest waveform snapshot from `analysis_task` as an oscilloscope trace.
///
/// Consecutive columns are joined by a vertical line segment so the trace is
/// continuous even when the signal changes by more than one pixel per step.
fn render_waveform(fb: &mut oled::FrameBuffer) {
    fb.fill(0x00);

    // TOPMOST = 5 (first on-screen row); usable rows: [5, 47] = 43 rows.
    const CENTER: i32 = TOPMOST as i32 + (oled::HEIGHT as i32 - TOPMOST as i32) / 2; // ≈ 26
    const HALF_SCALE: f32 = 20.0; // ± pixels for a ±1 signal
    const Y_MIN: i32 = TOPMOST as i32;
    const Y_MAX: i32 = oled::HEIGHT as i32 - 1;

    // SAFETY: written only by analysis_task; no concurrent writer in a
    //         single-threaded cooperative executor.
    let waveform = unsafe { &*core::ptr::addr_of!(WAVEFORM) };

    let sample_to_y =
        |s: f32| -> usize { (CENTER - (s * HALF_SCALE) as i32).clamp(Y_MIN, Y_MAX) as usize };

    let mut prev_y = sample_to_y(waveform[0]);
    fb.set_pixel(0, prev_y, true);

    for (x, &sample) in waveform.iter().enumerate().skip(1).take(oled::WIDTH - 1) {
        let y = sample_to_y(sample);
        // Draw a vertical segment from prev_y to y so there are no gaps.
        let (y0, y1) = if prev_y <= y {
            (prev_y, y)
        } else {
            (y, prev_y)
        };
        for row in y0..=y1 {
            fb.set_pixel(x, row, true);
        }
        prev_y = y;
    }
}

/// Render the current `PAD_BITS` state into `fb`, clearing first.
///
/// OLED rows 0–4 are off-panel (`TOPMOST`=5).  Pad y=0 is the physical
/// bottom row (lower-left origin), so `py=0` (OLED top cell) maps to pad
/// row y=7, and `py=7` (OLED bottom cell) maps to pad row y=0.
fn render_pads(fb: &mut oled::FrameBuffer) {
    fb.fill(0x00);
    for py in 0..8u8 {
        let pad_y = 7 - py; // flip: OLED top row = highest pad row
        for px in 0..18u8 {
            if !pad_get(pad_id_from_xy(px, pad_y)) {
                continue;
            }
            // 1 px gap on all sides → inner rect starts at (+1, +1)
            let ox = px as usize * CELL_W + 1;
            let oy = TOPMOST + py as usize * CELL_H + 1;
            for dy in 0..FILL_H {
                for dx in 0..FILL_W {
                    fb.set_pixel(ox + dx, oy + dy, true);
                }
            }
        }
    }
}

/// OLED render task.
///
/// Initialises the SSD1309 then waits for [`oled::notify_redraw()`] on every
/// pad-state change before rendering and pushing a full 768-byte frame.
#[embassy_executor::task]
pub(crate) async fn oled_task() {
    // Wait for pic::init() to complete before issuing any PIC UART commands.
    // Both tasks start concurrently; without this barrier oled::init() would
    // race with the baud-rate handshake in pic::init().
    pic::wait_ready().await;

    info!("OLED: init");
    oled::init().await;
    info!("OLED: ready");

    let mut fb = oled::FrameBuffer::new();

    // Render the initial (empty) state immediately without waiting for a pad press.
    render_pads(&mut fb);
    oled::send_frame(&fb).await;

    loop {
        // Suspend until pic_task (or any other caller) marks the state dirty.
        oled::wait_redraw().await;

        if CDC_FRAME_VALID.load(Ordering::Acquire) {
            // Host supplied a framebuffer — copy directly and send.
            // Safety: CDC_FRAME is only mutated by set_cdc_display() which
            // runs from the CDC task; since the executor is single-threaded,
            // there is no concurrent access here.
            unsafe {
                fb.pages
                    .as_flattened_mut()
                    .copy_from_slice(&*core::ptr::addr_of!(CDC_FRAME));
            }
        } else if USB_AUDIO_STREAMING.load(Ordering::Acquire) {
            render_waveform(&mut fb);
        } else {
            render_pads(&mut fb);
        }
        oled::send_frame(&fb).await;
    }
}
