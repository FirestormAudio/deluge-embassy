//! Deluge PIC32 co-processor interface.
//!
//! The PIC32 handles the entire UI layer: 144 RGB pad matrix, 36 button
//! matrix, 6 rotary encoders, 36 indicator LEDs, two "gold knob" rings, a
//! 7-segment display, and OLED display handshaking.  It communicates with the
//! RZ/A1L main CPU over a UART link (SCIF1).
//!
//! ## Baud rate
//! The link starts at **31,250 bps** (shared with MIDI for robustness at boot)
//! and is switched to **200,000 bps** after the PIC has been configured.  Both
//! sides must switch in close sequence (PIC first, then host 50 ms later).
//!
//! ## Message framing
//! There is no framing layer — the protocol is a flat byte stream in both
//! directions.  Command bytes are sent from CPU → PIC; events are returned PIC
//! → CPU.
//!
//! ## Event byte ranges (PIC → CPU)
//! | Range   | Meaning                                                  |
//! |---------|----------------------------------------------------------|
//! | 0–143   | Pad event (press if normal, release if preceded by 252)  |
//! | 144–179 | Button event (same logic)                                |
//! | 180–244 | Undefined / ignored                                      |
//! | 245     | FIRMWARE_VERSION_NEXT (followed by version byte)         |
//! | 246     | OLED-related (ignore)                                    |
//! | 247     | ENABLE_OLED echo (ignore)                                |
//! | 248–249 | OLED chip-select handshake (SELECT / DESELECT)           |
//! | 252     | NEXT_PAD_OFF prefix                                      |
//! | 254     | NO_PRESSES_HAPPENING                                     |
//!
//! **Note:** Rotary encoders are wired directly to RZ/A1L GPIO pins and are
//! read by a dedicated encoder task via [`rza1l_hal::gpio`] — the PIC does not
//! transmit encoder data over UART.
//!
//! ## Commands (CPU → PIC) — selected subset
//! | Byte | Command                    | Payload                         |
//! |------|----------------------------|---------------------------------|
//! |  18  | SET_DEBOUNCE_TIME          | time  (value × 4 ms)            |
//! |  19  | SET_REFRESH_TIME           | time  (ms)                      |
//! |  20  | SET_GOLD_KNOB_0_INDICATORS | 4 × brightness bytes            |
//! |  21  | SET_GOLD_KNOB_1_INDICATORS | 4 × brightness bytes            |
//! |  22  | RESEND_BUTTON_STATES       | —                               |
//! |  23  | SET_FLASH_LENGTH           | time  (ms)                      |
//! | 225  | SET_UART_SPEED             | divider (baud = 4MOhm / (d+1))  |
//! | 244  | SET_MIN_INTERRUPT_INTERVAL | time  (ms)                      |
//! | 245  | REQUEST_FIRMWARE_VERSION   | —                               |
//! | 247  | ENABLE_OLED                | —                               |

use rza1l_hal::uart;

// ── Channel / baud constants ──────────────────────────────────────────────────

/// SCIF channel wired to the PIC32 (matches `deluge_bsp::uart::PIC_CH`).
pub const UART_CH: usize = 1;

/// Initial baud rate — PIC uses 31 250 at power-on.
#[allow(dead_code)]
const BAUD_INIT: u32 = 31_250;

/// High-speed baud rate after the handshake.
const BAUD_FAST: u32 = 200_000;

/// PIC's internal oscillator used for the UART speed divider formula:
/// `baud = PIC_CLK / (divider + 1)`.
const PIC_CLK_HZ: u32 = 4_000_000;

// ── Command byte constants ────────────────────────────────────────────────────

const CMD_SET_DEBOUNCE_TIME: u8 = 18;
const CMD_SET_REFRESH_TIME: u8 = 19;
const CMD_SET_GOLD_KNOB_0_INDICATORS: u8 = 20;
const CMD_SET_GOLD_KNOB_1_INDICATORS: u8 = 21;
const CMD_RESEND_BUTTON_STATES: u8 = 22;
const CMD_SET_FLASH_LENGTH: u8 = 23;
const CMD_SET_UART_SPEED: u8 = 225;
const CMD_SET_MIN_INTERRUPT_INTERVAL: u8 = 244;
const CMD_REQUEST_FIRMWARE_VERSION: u8 = 245;
const CMD_ENABLE_OLED: u8 = 247;
const CMD_SELECT_OLED: u8 = 248;
const CMD_DESELECT_OLED: u8 = 249;
const CMD_SET_DC_LOW: u8 = 250;
const CMD_SET_DC_HIGH: u8 = 251;

/// Base command byte for indicator LED off: `CMD_LED_OFF_BASE + id` (id 0–35).
const CMD_LED_OFF_BASE: u8 = 152;
/// Base command byte for indicator LED on: `CMD_LED_ON_BASE + id` (id 0–35).
const CMD_LED_ON_BASE: u8 = 188;
/// Base command byte for setting two-column RGB data: `CMD_SET_COLOUR_FOR_COLS_BASE + pair`
/// where pair is 0–8.
const CMD_SET_COLOUR_FOR_COLS_BASE: u8 = 1;
/// Sent after all column-pair data to trigger the PIC display refresh.
const CMD_DONE_SENDING_ROWS: u8 = 240;

// ── Response byte sentinels ───────────────────────────────────────────────────

const RESP_NEXT_PAD_OFF: u8 = 252;
const RESP_NO_PRESSES: u8 = 254;
const RESP_FIRMWARE_VERSION: u8 = 245;

// ── Public types ──────────────────────────────────────────────────────────────

/// An event decoded from the PIC byte stream.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// A pad was pressed.  `id` is 0–143.
    PadPress { id: u8 },
    /// A pad was released.  `id` is 0–143.
    PadRelease { id: u8 },
    /// A button was pressed.  `id` is 0–35 (raw PIC value − 144).
    ButtonPress { id: u8 },
    /// A button was released.  `id` is 0–35.
    ButtonRelease { id: u8 },
    /// PIC firmware version byte (one-off, received after [`request_firmware_version`]).
    FirmwareVersion(u8),
    /// OLED chip-select asserted (PIC is ready for SPI data).
    OledSelected,
    /// OLED chip-select de-asserted.
    OledDeselected,
    /// PIC reports no pads are currently active.
    NoPresses,
}

/// Pad ID → (column, row) coordinate.
///
/// The PIC encodes pads in a packed 9-column × 16-per-column layout.
/// This returns `(x, y)` where `x` ∈ 0..17 and `y` ∈ 0..7.
#[inline]
pub fn pad_coords(id: u8) -> (u8, u8) {
    let y_raw = id / 9;
    let mut x = (id - y_raw * 9) * 2;
    let mut y = y_raw;
    if y >= 8 {
        y -= 8;
        x += 1;
    }
    (x, y)
}

/// Stateful parser for the PIC → CPU byte stream.
///
/// Feed bytes from the UART one at a time via [`Parser::push`]; it returns
/// `Some(Event)` when a complete event has been decoded and `None` when more
/// bytes are needed.
///
/// Maintains two bits of state:
/// - Whether the next pad/button byte is a release (preceded by 0xFC).
/// - Whether we are waiting for the second byte of a firmware-version sequence.
pub struct Parser {
    next_is_off: bool,
    firmware_version_next: bool,
}

impl Parser {
    pub const fn new() -> Self {
        Self {
            next_is_off: false,
            firmware_version_next: false,
        }
    }
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    /// Push one byte received from the PIC.  Returns a decoded event, if any.
    pub fn push(&mut self, byte: u8) -> Option<Event> {
        // ---- Second byte of a firmware-version sequence -------------------
        if self.firmware_version_next {
            self.firmware_version_next = false;
            return Some(Event::FirmwareVersion(byte));
        }

        // ---- Dispatch on first byte ----------------------------------------
        match byte {
            0..=143 => {
                // Pad event
                let off = self.next_is_off;
                self.next_is_off = false;
                if off {
                    Some(Event::PadRelease { id: byte })
                } else {
                    Some(Event::PadPress { id: byte })
                }
            }
            144..=179 => {
                // Button event
                let off = self.next_is_off;
                self.next_is_off = false;
                let id = byte - 144;
                if off {
                    Some(Event::ButtonRelease { id })
                } else {
                    Some(Event::ButtonPress { id })
                }
            }
            180..=244 => {
                // Undefined bytes in the PIC protocol — silently ignored.
                // (Rotary encoders are wired to RZ/A1L GPIO, not the PIC.)
                None
            }
            RESP_FIRMWARE_VERSION => {
                // Firmware version prefix (byte 245) — next byte is the actual version value
                self.firmware_version_next = true;
                None
            }
            248 => Some(Event::OledSelected),
            249 => Some(Event::OledDeselected),
            RESP_NEXT_PAD_OFF => {
                self.next_is_off = true;
                None
            }
            RESP_NO_PRESSES => {
                self.next_is_off = false;
                Some(Event::NoPresses)
            }
            _ => None, // Ignore other bytes (246, 250, 251, 253, 255, etc.)
        }
    }
}

// ── PIC initialisation sequence ───────────────────────────────────────────────

/// Run the full PIC32 initialisation sequence asynchronously.
///
/// 1. Sends configuration commands at 31 250 bps (OLED enable, debounce,
///    refresh rate, interrupt interval, flash length, UART speed command).
/// 2. Waits 50 ms for the PIC to switch to 200 000 bps.
/// 3. Switches the host SCIF1 to 200 000 bps.
/// 4. Requests firmware version and button states; waits another 50 ms.
///
/// **Must be called after** `deluge_bsp::uart::init_pic(31_250)` has set up
/// SCIF1 and the Embassy executor is running (uses `Timer`).
#[cfg(target_os = "none")]
pub async fn init() {
    use embassy_time::Timer;
    log::debug!("pic: init at 31250 bps, will switch to {} bps", BAUD_FAST);

    // ---- Configure PIC while still at 31 250 bps --------------------------
    // Enable OLED
    uart::write_bytes(UART_CH, &[CMD_ENABLE_OLED]).await;
    // Debounce: 5  → 5 × 4 ms = 20 ms
    uart::write_bytes(UART_CH, &[CMD_SET_DEBOUNCE_TIME, 5]).await;
    // Refresh time: 23 ms
    uart::write_bytes(UART_CH, &[CMD_SET_REFRESH_TIME, 23]).await;
    // Min interrupt interval: 8 ms
    uart::write_bytes(UART_CH, &[CMD_SET_MIN_INTERRUPT_INTERVAL, 8]).await;
    // Flash length: 6 ms
    uart::write_bytes(UART_CH, &[CMD_SET_FLASH_LENGTH, 6]).await;
    // Tell PIC to switch to 200 000 bps: divider = PIC_CLK / BAUD_FAST − 1
    let speed_divider = (PIC_CLK_HZ / BAUD_FAST).saturating_sub(1) as u8;
    uart::write_bytes(UART_CH, &[CMD_SET_UART_SPEED, speed_divider]).await;

    // ---- Give PIC 50 ms to switch baud rate --------------------------------
    Timer::after_millis(50).await;

    // ---- Switch host SCIF1 to 200 000 bps ----------------------------------
    // Safety: write_bytes has completed, so the FIFO has drained.  SCIF1 was
    // initialised by `deluge_bsp::uart::init_pic`; we just update SCBRR here.
    unsafe { uart::set_baud(UART_CH, BAUD_FAST) };

    // ---- Request firmware version and initial button state -----------------
    uart::write_bytes(UART_CH, &[CMD_REQUEST_FIRMWARE_VERSION]).await;
    uart::write_bytes(UART_CH, &[CMD_RESEND_BUTTON_STATES]).await;

    // Give PIC time to respond
    Timer::after_millis(50).await;
    log::debug!("pic: ready at {} bps", BAUD_FAST);

    // Signal that init is complete — other tasks waiting on wait_ready() unblock.
    ready_signal::signal();
}

// ── Outbound helpers ──────────────────────────────────────────────────────────

/// Set indicator LED `id` (0–35) on.
#[inline]
pub async fn led_on(id: u8) {
    uart::write_bytes(UART_CH, &[CMD_LED_ON_BASE + id]).await;
}

/// Set indicator LED `id` (0–35) off.
#[inline]
pub async fn led_off(id: u8) {
    uart::write_bytes(UART_CH, &[CMD_LED_OFF_BASE + id]).await;
}

/// Set RGB colours for one column-pair of the main pad grid.
///
/// `pair` is 0–8:
/// - 0–7 → main pad column pairs 0–15 (pair n = physical columns 2n, 2n+1)
/// - 8   → sidebar columns 16–17
///
/// `colours[0..8]`  = rows 0–7 of the **left** column (col 2n).
/// `colours[8..16]` = rows 0–7 of the **right** column (col 2n+1).
///
/// Each entry is `[r, g, b]` with 0–255 per channel.
///
/// After sending all 9 pairs, call [`done_sending_rows`] to trigger the
/// PIC's display refresh.
pub async fn set_column_pair_rgb(pair: u8, colours: &[[u8; 3]; 16]) {
    // 1 command byte + 16 × 3 colour bytes = 49 bytes per pair
    let mut buf = [0u8; 49];
    buf[0] = CMD_SET_COLOUR_FOR_COLS_BASE + pair;
    for (i, [r, g, b]) in colours.iter().enumerate() {
        buf[1 + i * 3] = *r;
        buf[2 + i * 3] = *g;
        buf[3 + i * 3] = *b;
    }
    uart::write_bytes(UART_CH, &buf).await;
}

/// Signal the PIC that all column-pair colour data has been sent for this
/// frame, triggering a display refresh.  Call after the last
/// [`set_column_pair_rgb`] for a complete grid update.
#[inline]
pub async fn done_sending_rows() {
    uart::write_bytes(UART_CH, &[CMD_DONE_SENDING_ROWS]).await;
}

/// Request the PIC to re-send all button/pad pressed states.
#[inline]
pub async fn resend_button_states() {
    uart::write_bytes(UART_CH, &[CMD_RESEND_BUTTON_STATES]).await;
}

/// Request the PIC firmware version (arrives asynchronously as
/// [`Event::FirmwareVersion`]).
#[inline]
pub async fn request_firmware_version() {
    uart::write_bytes(UART_CH, &[CMD_REQUEST_FIRMWARE_VERSION]).await;
}

/// Set both gold-knob LED rings.
/// `knob`: 0 or 1.  `brightnesses`: four brightness values (0–255).
#[inline]
pub async fn set_gold_knob_indicators(knob: u8, brightnesses: [u8; 4]) {
    let cmd = if knob == 0 {
        CMD_SET_GOLD_KNOB_0_INDICATORS
    } else {
        CMD_SET_GOLD_KNOB_1_INDICATORS
    };
    uart::write_bytes(
        UART_CH,
        &[
            cmd,
            brightnesses[0],
            brightnesses[1],
            brightnesses[2],
            brightnesses[3],
        ],
    )
    .await;
}

// ── OLED SPI handshake helpers ────────────────────────────────────────────────

/// Enable OLED power via the PIC (send ENABLE_OLED = 247).
///
/// Must be called once during initialisation, before the first [`oled_select()`].
#[inline]
pub async fn oled_enable() {
    uart::write_bytes(UART_CH, &[CMD_ENABLE_OLED]).await;
}

/// Assert OLED chip-select via the PIC.
#[inline]
pub async fn oled_select() {
    uart::write_bytes(UART_CH, &[CMD_SELECT_OLED]).await;
}

/// De-assert OLED chip-select via the PIC.
#[inline]
pub async fn oled_deselect() {
    uart::write_bytes(UART_CH, &[CMD_DESELECT_OLED]).await;
}

/// Pull OLED Data/!Command line low (command mode).
#[inline]
pub async fn oled_dc_low() {
    uart::write_bytes(UART_CH, &[CMD_SET_DC_LOW]).await;
}

/// Pull OLED Data/!Command line high (data mode).
#[inline]
pub async fn oled_dc_high() {
    uart::write_bytes(UART_CH, &[CMD_SET_DC_HIGH]).await;
}

// ── PIC-ready signal ─────────────────────────────────────────────────────────
//
// Set by `pic::init()` once the full initialisation sequence has completed
// (baud switched, firmware version and button-state resend requested, 50 ms
// settling time elapsed).  Any task that depends on the PIC being fully
// configured (notably `oled_task`) should `pic::wait_ready().await` before
// issuing its own PIC commands.

#[cfg(target_os = "none")]
mod ready_signal {
    use core::sync::atomic::{AtomicBool, Ordering};

    static READY_FLAG: AtomicBool = AtomicBool::new(false);

    /// Called once by `pic::init()` when the sequence is complete.
    pub fn signal() {
        READY_FLAG.store(true, Ordering::Release);
    }

    /// Poll until `signal()` has been called.  Returns immediately if it
    /// was already called (safe to call multiple times, from any number of tasks).
    pub async fn wait() {
        while !READY_FLAG.load(Ordering::Acquire) {
            embassy_time::Timer::after_millis(1).await;
        }
    }
}

/// Suspend until [`init()`] has completed on this PIC channel.
///
/// Call this at the start of any task that issues PIC UART commands, to
/// ensure the baud-rate handshake and initial configuration have already
/// finished.  Returns immediately if `init()` has already run.
#[cfg(target_os = "none")]
#[inline]
pub async fn wait_ready() {
    ready_signal::wait().await;
}

// ── OLED chip-select handshake signals ───────────────────────────────────────
//
// The PIC echoes SELECT_OLED (248) and DESELECT_OLED (249) back when it has
// asserted/de-asserted the OLED chip-select line.  The OLED driver must wait
// for these echoes before starting / finishing a DMA transfer.
//
// The `pic_task` in firmware calls notify_oled_selected() / notify_oled_deselected()
// when it decodes the corresponding PIC events.

#[cfg(target_os = "none")]
mod oled_signal {
    use core::future::poll_fn;
    use core::sync::atomic::{AtomicBool, Ordering};
    use core::task::Poll;
    use embassy_sync::waitqueue::AtomicWaker;

    static SELECTED_FLAG: AtomicBool = AtomicBool::new(false);
    static SELECTED_WAKER: AtomicWaker = AtomicWaker::new();
    static DESELECTED_FLAG: AtomicBool = AtomicBool::new(false);
    static DESELECTED_WAKER: AtomicWaker = AtomicWaker::new();

    /// Called by `pic_task` when it receives an OledSelected event (byte 248).
    pub fn notify_selected() {
        SELECTED_FLAG.store(true, Ordering::Release);
        SELECTED_WAKER.wake();
    }

    /// Called by `pic_task` when it receives an OledDeselected event (byte 249).
    pub fn notify_deselected() {
        DESELECTED_FLAG.store(true, Ordering::Release);
        DESELECTED_WAKER.wake();
    }

    /// Suspend until the PIC confirms OLED CS is asserted (echo 248).
    pub async fn wait_selected() {
        poll_fn(|cx| {
            SELECTED_WAKER.register(cx.waker());
            if SELECTED_FLAG.swap(false, Ordering::AcqRel) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }

    /// Suspend until the PIC confirms OLED CS is de-asserted (echo 249).
    pub async fn wait_deselected() {
        poll_fn(|cx| {
            DESELECTED_WAKER.register(cx.waker());
            if DESELECTED_FLAG.swap(false, Ordering::AcqRel) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

/// Called by `pic_task` when [`Event::OledSelected`] is decoded.
#[cfg(target_os = "none")]
#[inline]
pub fn notify_oled_selected() {
    oled_signal::notify_selected();
}

/// Called by `pic_task` when [`Event::OledDeselected`] is decoded.
#[cfg(target_os = "none")]
#[inline]
pub fn notify_oled_deselected() {
    oled_signal::notify_deselected();
}

/// Suspend until the PIC confirms OLED CS is asserted.
///
/// Send [`oled_select()`] before calling this.
#[cfg(target_os = "none")]
#[inline]
pub async fn wait_oled_selected() {
    oled_signal::wait_selected().await;
}

/// Suspend until the PIC confirms OLED CS is de-asserted.
///
/// Send [`oled_deselect()`] before calling this.
#[cfg(target_os = "none")]
#[inline]
pub async fn wait_oled_deselected() {
    oled_signal::wait_deselected().await;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    // ---- pad_coords ---------------------------------------------------------

    #[test]
    fn pad_coords_first() {
        // id=0: y_raw=0, x=0, y=0 → (0, 0)
        assert_eq!(pad_coords(0), (0, 0));
    }

    #[test]
    fn pad_coords_second_column_first_row() {
        // id=1: y_raw=0, x=2, y=0 → (2, 0)
        assert_eq!(pad_coords(1), (2, 0));
    }

    #[test]
    fn pad_coords_ninth_column() {
        // id=8: y_raw=0, x=16, y=0 → (16, 0)
        assert_eq!(pad_coords(8), (16, 0));
    }

    #[test]
    fn pad_coords_second_half_adds_one_to_x() {
        // id=72: y_raw=8, x=0+1=1, y=8-8=0 → (1, 0)
        assert_eq!(pad_coords(72), (1, 0));
    }

    // ---- Parser -------------------------------------------------------------

    #[test]
    fn parser_pad_press() {
        let mut p = Parser::new();
        assert_eq!(p.push(5), Some(Event::PadPress { id: 5 }));
    }

    #[test]
    fn parser_pad_release() {
        let mut p = Parser::new();
        assert_eq!(p.push(RESP_NEXT_PAD_OFF), None);
        assert_eq!(p.push(5), Some(Event::PadRelease { id: 5 }));
    }

    #[test]
    fn parser_button_press() {
        let mut p = Parser::new();
        // Button ID 0 = raw byte 144
        assert_eq!(p.push(144), Some(Event::ButtonPress { id: 0 }));
    }

    #[test]
    fn parser_button_release_after_off_prefix() {
        let mut p = Parser::new();
        assert_eq!(p.push(RESP_NEXT_PAD_OFF), None);
        assert_eq!(p.push(150), Some(Event::ButtonRelease { id: 6 }));
    }

    #[test]
    fn parser_undefined_bytes_ignored() {
        // Bytes 180–244 are undefined in the PIC protocol and must be dropped
        // without consuming the following byte.
        let mut p = Parser::new();
        assert_eq!(p.push(180), None);
        // The byte after should still parse normally (not consumed as a delta).
        assert_eq!(p.push(5), Some(Event::PadPress { id: 5 }));
    }

    #[test]
    fn parser_no_presses() {
        let mut p = Parser::new();
        assert_eq!(p.push(RESP_NO_PRESSES), Some(Event::NoPresses));
    }

    #[test]
    fn parser_oled_select() {
        let mut p = Parser::new();
        assert_eq!(p.push(248), Some(Event::OledSelected));
        assert_eq!(p.push(249), Some(Event::OledDeselected));
    }

    #[test]
    fn parser_firmware_version() {
        let mut p = Parser::new();
        assert_eq!(p.push(245), None);
        assert_eq!(p.push(42), Some(Event::FirmwareVersion(42)));
    }

    #[test]
    fn parser_next_pad_off_clears_after_event() {
        let mut p = Parser::new();
        // OFF prefix affects only the NEXT 0–179 byte
        p.push(RESP_NEXT_PAD_OFF);
        p.push(10); // release
        // No prefix now — next should be a press
        assert_eq!(p.push(10), Some(Event::PadPress { id: 10 }));
    }

    #[test]
    fn uart_speed_divider_for_200k() {
        // 4_000_000 / 200_000 - 1 = 19
        let d = (PIC_CLK_HZ / BAUD_FAST).saturating_sub(1) as u8;
        assert_eq!(d, 19);
    }
}
