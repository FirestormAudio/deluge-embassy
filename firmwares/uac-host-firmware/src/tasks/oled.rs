//! OLED status for the UAC host validation firmware.
//!
//! ```text
//! UAC HOST                UAC HOST
//! waiting for device      44100Hz 2>2ch 24bit
//!                         loop  in ####----
//! ```

use core::sync::atomic::Ordering;

use embassy_time::Timer;

use deluge_bsp::oled::{self, text};
use deluge_bsp::pic;

use crate::tasks::loopback::{
    CAP_CH, PEAK_MILLI, PLAY_CH, STAGE, STAGE_STREAMING, STAGE_WAITING,
};

const INTERVAL_MS: u64 = 250;
const TOP: usize = 10;

#[embassy_executor::task]
pub(crate) async fn oled_task() {
    pic::wait_ready().await;
    oled::init().await;
    let mut fb = oled::FrameBuffer::new();
    let mut line = [0u8; 24];

    loop {
        fb.fill(0x00);
        text::draw_str(&mut fb, 0, TOP, b"UAC HOST");

        match STAGE.load(Ordering::Relaxed) {
            STAGE_WAITING => {
                text::draw_str(&mut fb, 0, TOP + 14, b"waiting for device");
            }
            STAGE_STREAMING => {
                let cap = CAP_CH.load(Ordering::Relaxed);
                let play = PLAY_CH.load(Ordering::Relaxed);
                let n = fmt_format(&mut line, cap, play);
                text::draw_str(&mut fb, 0, TOP + 12, &line[..n]);

                // "loop  in <bar>" — 8-cell peak bar from PEAK_MILLI (0..=1000).
                let peak = PEAK_MILLI.load(Ordering::Relaxed);
                let n = fmt_meter(&mut line, peak);
                text::draw_str(&mut fb, 0, TOP + 26, &line[..n]);
            }
            _ => {}
        }

        oled::send_frame(&fb).await;
        Timer::after_millis(INTERVAL_MS).await;
    }
}

/// `"44100Hz <cap>><play>ch 24bit"`.
fn fmt_format(out: &mut [u8], cap: u8, play: u8) -> usize {
    let mut p = 0;
    for &b in b"44100Hz " {
        push(out, &mut p, b);
    }
    push_u8(out, &mut p, cap);
    push(out, &mut p, b'>');
    push_u8(out, &mut p, play);
    for &b in b"ch 24bit" {
        push(out, &mut p, b);
    }
    p
}

/// `"loop  in ########"` — an 8-cell bar filled proportional to `peak` (0..=1000).
fn fmt_meter(out: &mut [u8], peak: u32) -> usize {
    let mut p = 0;
    for &b in b"loop  in " {
        push(out, &mut p, b);
    }
    let filled = ((peak.min(1000) * 8) / 1000) as usize;
    for i in 0..8 {
        push(out, &mut p, if i < filled { b'#' } else { b'-' });
    }
    p
}

#[inline]
fn push(out: &mut [u8], p: &mut usize, b: u8) {
    if *p < out.len() {
        out[*p] = b;
        *p += 1;
    }
}

fn push_u8(out: &mut [u8], p: &mut usize, mut v: u8) {
    if v == 0 {
        push(out, p, b'0');
        return;
    }
    let mut tmp = [0u8; 3];
    let mut i = 0;
    while v > 0 {
        tmp[i] = b'0' + (v % 10);
        v /= 10;
        i += 1;
    }
    while i > 0 {
        i -= 1;
        push(out, p, tmp[i]);
    }
}
