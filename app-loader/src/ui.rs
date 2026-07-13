//! OLED file-selector UI for the app loader (second-stage bootloader).
//!
//! Presents a scrollable list of application names on the 128×48 OLED display.
//! The user scrolls with the SELECT encoder and confirms by pressing the
//! SELECT encoder button.
//!
//! ## Layout (128 × 48 pixel panel, 6 pages of 8 rows)
//! ```text
//! Row  0– 7  : Title bar  "SELECT APP"
//! Row  8–15  : separator line
//! Row 16–23  : entry 0  (cursor ▶ if selected)
//! Row 24–31  : entry 1
//! Row 32–39  : entry 2
//! Row 40–47  : entry 3
//! ```
//! Up to 4 entries are visible at a time.  A solid triangle on the left edge
//! marks the highlighted entry.

use core::sync::atomic::Ordering;

use deluge_bsp::oled::text::draw_str;
use deluge_bsp::oled::{self, FrameBuffer, WIDTH};

const VISIBLE_ROWS: usize = 4;
/// Top padding in pixels.  The Deluge OLED panel's top 5 rows sit off the
/// visible area, so all content is shifted down to start at row 5 (matches the
/// demo/controller firmware's `TOPMOST = 5`).
const TOP_PAD: usize = 5;
/// Pixel row of the title bar.
const TITLE_ROW: usize = TOP_PAD;
/// Pixel row of the separator line.
const SEPARATOR_ROW: usize = TOP_PAD + 8;
/// Pixel row of the first entry line.
const ENTRY_START_ROW: usize = TOP_PAD + 11;
/// Height of one entry in pixels (one page = 8 rows).
const ENTRY_HEIGHT: usize = 8;

/// Draw a small filled right-pointing triangle at (`x`, `y`).
fn draw_solid_triangle(fb: &mut FrameBuffer, x: usize, y: usize) {
    for dy in 0..7usize {
        let span = 4usize.saturating_sub((3isize - dy as isize).unsigned_abs());
        for dx in 0..span {
            fb.set_pixel(x + dx, y + dy, true);
        }
    }
}

/// Format a "BOOT IN Ns" countdown title into `buf`, returning the used slice.
fn countdown_title(buf: &mut [u8; 12], secs: u8) -> &[u8] {
    const PREFIX: &[u8] = b"BOOT IN ";
    let mut n = 0;
    for &b in PREFIX {
        buf[n] = b;
        n += 1;
    }
    if secs >= 10 {
        buf[n] = b'0' + secs / 10;
        n += 1;
    }
    buf[n] = b'0' + secs % 10;
    n += 1;
    buf[n] = b'S';
    n += 1;
    &buf[..n]
}

/// Render a frame showing the selector list.
///
/// * `entries`  — full sorted list of entry names (full `BASE.EXT` filenames)
/// * `scroll`   — index of the first visible entry
/// * `cursor`   — index of the highlighted entry (absolute, not relative)
/// * `countdown`— `Some(secs_remaining)` shows a boot countdown in the title bar.
fn render(
    fb: &mut FrameBuffer,
    entries: &[&[u8]],
    scroll: usize,
    cursor: usize,
    countdown: Option<u8>,
) {
    fb.fill(0x00);

    // Title bar — show the countdown while it is running, otherwise the label.
    let mut cd_buf = [0u8; 12];
    let title: &[u8] = match countdown {
        Some(secs) => countdown_title(&mut cd_buf, secs),
        None => b"SELECT APP",
    };
    draw_str(fb, 4, TITLE_ROW, title);

    // Separator line.
    for x in 0..WIDTH {
        fb.set_pixel(x, SEPARATOR_ROW, true);
    }

    // Entry rows.
    for slot in 0..VISIBLE_ROWS {
        let idx = scroll + slot;
        if idx >= entries.len() {
            break;
        }
        let y = ENTRY_START_ROW + slot * ENTRY_HEIGHT;

        // Cursor marker.
        if idx == cursor {
            draw_solid_triangle(fb, 0, y);
        }

        // Filename starting at x=8.
        draw_str(fb, 8, y, entries[idx]);
    }

    // Proportional scrollbar on the right edge (Deluge-style).
    draw_scrollbar(fb, entries.len(), scroll);
}

// Scrollbar geometry — a 1px track with a 3px-wide hollow indicator on the
// right edge, spanning the visible entry-list area.
/// Centre column of the scrollbar track (the indicator straddles it).
const SCROLLBAR_X: usize = WIDTH - 2;
/// First pixel row of the track (top of the entry list).
const TRACK_TOP: usize = ENTRY_START_ROW;
/// Last pixel row of the track (bottom of the visible list).
const TRACK_BOTTOM: usize = ENTRY_START_ROW + VISIBLE_ROWS * ENTRY_HEIGHT - 1;
/// Total track span in pixels (both endpoints inclusive).
const TRACK_HEIGHT: usize = TRACK_BOTTOM - TRACK_TOP + 1;

/// Draw a proportional scrollbar on the right edge, mirroring the Deluge
/// firmware's list scrollbar (see `~/GitHub/spark` `list_menu_view::draw_scrollbar`).
///
/// The indicator's height is proportional to the visible fraction
/// (`VISIBLE_ROWS / total`, min 3 px) and its position is proportional to the
/// scroll offset.  Nothing is drawn when everything fits on screen.
fn draw_scrollbar(fb: &mut FrameBuffer, total: usize, scroll: usize) {
    if total <= VISIBLE_ROWS {
        return;
    }

    // Proportional indicator height (min 3 px) and travel.
    let indicator_h = ((VISIBLE_ROWS * TRACK_HEIGHT) / total).clamp(3, TRACK_HEIGHT);
    let travel = TRACK_HEIGHT - indicator_h;
    let denom = total - VISIBLE_ROWS; // > 0 (total > VISIBLE_ROWS above)
    let indicator_y = (TRACK_TOP + (travel * scroll) / denom).min(TRACK_TOP + travel);
    let indicator_y1 = indicator_y + indicator_h - 1;

    // Clear the scrollbar strip (4 px) so long filenames don't bleed into it.
    for y in TRACK_TOP..=TRACK_BOTTOM {
        for x in (SCROLLBAR_X - 2)..=(SCROLLBAR_X + 1) {
            fb.set_pixel(x, y, false);
        }
    }

    // Track line above and below the indicator.
    for y in TRACK_TOP..indicator_y {
        fb.set_pixel(SCROLLBAR_X, y, true);
    }
    for y in (indicator_y1 + 1)..=TRACK_BOTTOM {
        fb.set_pixel(SCROLLBAR_X, y, true);
    }

    // Hollow indicator rectangle (x = SCROLLBAR_X-1 ..= SCROLLBAR_X+1).
    let (x0, x1) = (SCROLLBAR_X - 1, SCROLLBAR_X + 1);
    for x in x0..=x1 {
        fb.set_pixel(x, indicator_y, true);
        fb.set_pixel(x, indicator_y1, true);
    }
    for y in indicator_y..=indicator_y1 {
        fb.set_pixel(x0, y, true);
        fb.set_pixel(x1, y, true);
    }
}

/// Hold time (ms) that distinguishes a long-press of SELECT from a short tap.
const LONG_PRESS_MS: u64 = 700;

/// Outcome of a confirmed selector entry.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Selection {
    /// Highlighted entry index.
    pub index: usize,
    /// `true` if the entry was confirmed with a long-press (hold ≥
    /// [`LONG_PRESS_MS`]) rather than a short tap.
    pub long_press: bool,
}

/// Run the interactive GRUB-style boot selector.
///
/// `entries` is a slice of byte-string labels.  The cursor starts on
/// `default_idx`.  When `countdown_secs > 0` a visible countdown runs and the
/// default entry auto-boots on expiry; turning the encoder cancels the countdown
/// and hands control to the user.
///
/// Pressing the SELECT encoder button confirms the highlighted entry: a short
/// tap returns `long_press = false`; holding it for ≥ [`LONG_PRESS_MS`] returns
/// `long_press = true` as soon as the threshold elapses (fire-on-hold), so the
/// user gets feedback without waiting for release.
///
/// Must be called from an Embassy task after `oled::init()` has completed.
pub async fn run_selector(entries: &[&[u8]], default_idx: usize, countdown_secs: u8) -> Selection {
    use embassy_time::{Duration, Instant, Timer};

    let mut cursor: usize = default_idx.min(entries.len().saturating_sub(1));
    // Keep the default entry visible at startup.
    let mut scroll: usize = cursor.saturating_sub(VISIBLE_ROWS - 1);
    let mut edge_acc: i8 = 0;

    let mut countdown_active = countdown_secs > 0;
    let start = Instant::now();
    let countdown = Duration::from_secs(countdown_secs as u64);

    // SELECT-button hold tracking.  `press_at` is `Some` while the button is
    // held; on the rising edge we record when, and once the hold crosses the
    // long-press threshold we fire immediately.
    let mut press_at: Option<Instant> = None;

    // A SELECT press held from before the selector opened — the recovery gesture,
    // or the press that confirmed a previous screen — is not a new press.  Wait
    // for the release before acting on the button at all; otherwise the hold
    // crosses the long-press threshold below and fires a write-to-flash prompt.
    let mut armed = !SELECT_DOWN.load(Ordering::Acquire);

    // Use the SELECT encoder for scrolling in the bootloader selector.
    const ENC: usize = deluge_bsp::controls::encoder::SELECT as usize;

    loop {
        // While a USB upload owns the OLED (dev mode), stand down: don't draw and
        // don't act on input, so the upload progress bar isn't fought over and a
        // stray encoder bump can't abort the transfer.  Resumes if the upload is
        // rejected; a successful one launches and never returns.
        if UPLOAD_ACTIVE.load(Ordering::Acquire) {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(16)).await;
            continue;
        }

        // Remaining seconds for the title bar (rounds up so it ends on "1S").
        let remaining = if countdown_active {
            let left = countdown.checked_sub(start.elapsed()).unwrap_or_default();
            Some((left.as_millis().div_ceil(1000)) as u8)
        } else {
            None
        };

        // Build and send frame.
        let mut fb = FrameBuffer::new();
        render(&mut fb, entries, scroll, cursor, remaining);
        oled::send_frame(&fb).await;

        // Poll for encoder input at ~60 Hz.
        Timer::after(Duration::from_millis(16)).await;

        let detents = deluge_bsp::encoder::take_detents(ENC, &mut edge_acc);

        if detents != 0 {
            // User took control — stop the auto-boot countdown.
            countdown_active = false;

            // Scroll cursor.
            if detents > 0 {
                if cursor + 1 < entries.len() {
                    cursor += 1;
                }
            } else {
                cursor = cursor.saturating_sub(1);
            }

            // Keep scroll window tracking cursor.
            if cursor < scroll {
                scroll = cursor;
            } else if cursor >= scroll + VISIBLE_ROWS {
                scroll = cursor + 1 - VISIBLE_ROWS;
            }
        }

        // SELECT button edge handling (state pumped by pic_rx_task in main.rs).
        let down = SELECT_DOWN.load(Ordering::Acquire);
        if !armed {
            // Still letting go of a press that predates this screen.
            armed = !down;
        } else {
            match (press_at, down) {
                (None, true) => {
                    // Rising edge: a press began. Any press cancels the countdown.
                    countdown_active = false;
                    press_at = Some(Instant::now());
                }
                (Some(at), true) => {
                    // Still held — fire as soon as it becomes a long-press.
                    if at.elapsed() >= Duration::from_millis(LONG_PRESS_MS) {
                        return Selection {
                            index: cursor,
                            long_press: true,
                        };
                    }
                }
                (Some(_), false) => {
                    // Falling edge before the threshold: a short tap = confirm.
                    return Selection {
                        index: cursor,
                        long_press: false,
                    };
                }
                (None, false) => {}
            }
        }

        // Auto-boot the default entry when the countdown expires.
        if countdown_active && start.elapsed() >= countdown {
            return Selection {
                index: default_idx.min(entries.len().saturating_sub(1)),
                long_press: false,
            };
        }
    }
}

/// Tracks whether the SELECT encoder button is currently held.
///
/// Pumped by `pic_rx_task` in `main.rs` (set on `ButtonPress`, cleared on
/// `ButtonRelease`); the selector and prompts derive press/hold edges from it.
pub static SELECT_DOWN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// Set by [`crate::devupload`] once an upload header has arrived, to claim the
/// OLED. While set, [`run_selector`] stops drawing *and* stops acting on input,
/// so the upload progress bar isn't fought over and an accidental encoder bump
/// can't abort an in-flight upload. Cleared again only if the upload is rejected
/// (a successful one launches and never returns).
pub static UPLOAD_ACTIVE: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Modal YES/NO prompt asking whether to write `label` to the flash slot.
///
/// Returns `true` only if the user selects YES.  The cursor defaults to NO (the
/// safe choice); the SELECT encoder scrolls between YES/NO and a fresh press
/// confirms.  The active option is marked with a solid triangle, matching the
/// boot selector.
///
/// The long-press that opened this prompt is normally still held on entry, so we
/// draw the prompt immediately (feedback the instant the hold fires) but wait for
/// that press to be released before accepting a confirmation — otherwise the
/// opening hold would be consumed as the YES/NO press.
pub async fn confirm_write_to_flash(label: &[u8]) -> bool {
    use embassy_time::{Duration, Timer};

    const ENC: usize = deluge_bsp::controls::encoder::SELECT as usize;
    let mut edge_acc: i8 = 0;
    let mut yes = false; // false = NO (default), true = YES

    // YES / NO label positions; the active one gets a triangle marker just to its
    // left (label_x - 6), like the boot selector's cursor.
    const YES_X: usize = 16;
    const NO_X: usize = 76;

    // The opening long-press is still held; only confirm after it clears.
    let mut released = false;

    loop {
        let mut fb = FrameBuffer::new();
        fb.fill(0x00);

        let title = b"WRITE TO FLASH?";
        let tx = (WIDTH.saturating_sub(title.len() * 6)) / 2;
        draw_str(&mut fb, tx, TOP_PAD, title);

        let lx = (WIDTH.saturating_sub(label.len() * 6)) / 2;
        draw_str(&mut fb, lx, TOP_PAD + 14, label);

        // YES / NO row with a triangle marker on the active option.
        let row = TOP_PAD + 28;
        draw_str(&mut fb, YES_X, row, b"YES");
        draw_str(&mut fb, NO_X, row, b"NO");
        draw_solid_triangle(&mut fb, if yes { YES_X } else { NO_X } - 6, row);
        oled::send_frame(&fb).await;

        Timer::after(Duration::from_millis(16)).await;

        // Direction-aware selection: scroll either way to land on a fixed option
        // (YES when scrolling back, NO when scrolling forward) rather than blindly
        // toggling, so holding a direction settles instead of flip-flopping.
        let detents = deluge_bsp::encoder::take_detents(ENC, &mut edge_acc);
        if detents < 0 {
            yes = true;
        } else if detents > 0 {
            yes = false;
        }

        let down = SELECT_DOWN.load(Ordering::Acquire);
        if !released {
            // Still letting go of the opening long-press.
            released = !down;
        } else if down {
            // Fresh press after release — confirm the highlighted option.
            return yes;
        }
    }
}

/// Display a static error or status message centred on the OLED.
pub async fn show_message(line1: &[u8], line2: &[u8]) {
    let mut fb = FrameBuffer::new();
    fb.fill(0x00);
    // Two lines, shifted down by the panel's top padding.
    let x1 = (WIDTH.saturating_sub(line1.len() * 6)) / 2;
    draw_str(&mut fb, x1, TOP_PAD + 16, line1);
    let x2 = (WIDTH.saturating_sub(line2.len() * 6)) / 2;
    draw_str(&mut fb, x2, TOP_PAD + 28, line2);
    oled::send_frame(&fb).await;
}

/// Display a simple progress bar with an app label.
pub async fn show_progress(label: &[u8], percent: u8) {
    let mut fb = FrameBuffer::new();
    fb.fill(0x00);

    let title = b"LOADING APP";
    let title_x = (WIDTH.saturating_sub(title.len() * 6)) / 2;
    draw_str(&mut fb, title_x, TOP_PAD + 8, title);

    let label_x = (WIDTH.saturating_sub(label.len() * 6)) / 2;
    draw_str(&mut fb, label_x, TOP_PAD + 18, label);

    let bar_x = 8usize;
    let bar_y = TOP_PAD + 30;
    let bar_w = WIDTH.saturating_sub(16);
    let bar_h = 10usize;

    for x in bar_x..(bar_x + bar_w) {
        fb.set_pixel(x, bar_y, true);
        fb.set_pixel(x, bar_y + bar_h - 1, true);
    }
    for y in bar_y..(bar_y + bar_h) {
        fb.set_pixel(bar_x, y, true);
        fb.set_pixel(bar_x + bar_w - 1, y, true);
    }

    let pct = core::cmp::min(percent, 100) as usize;
    let fill_w = (bar_w.saturating_sub(2) * pct) / 100;
    for y in (bar_y + 1)..(bar_y + bar_h - 1) {
        for x in (bar_x + 1)..(bar_x + 1 + fill_w) {
            fb.set_pixel(x, y, true);
        }
    }

    oled::send_frame(&fb).await;
}
