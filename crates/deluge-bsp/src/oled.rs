//! Deluge OLED display driver (SSD1309, 128 × 48 pixels).
//!
//! ## Hardware
//! | Panel         | SSD1309 OLED controller                      |
//! |---------------|---------------------------------------------|
//! | Resolution    | 128 × 48 pixels                             |
//! | SPI channel   | RSPI0 (shared with CV DAC)                  |
//! | SPI clock     | 10 MHz, mode 0 (CPHA=0, CPOL=0)            |
//! | CS / DC / RST | Controlled via PIC32 UART (bytes 248–251)   |
//! | Frame size    | 6 pages × 128 columns = 768 bytes           |
//!
//! ## CS/DC pin control
//! The chip-select and data/command pins are **not** GPIO on the RZ/A1L side —
//! they are driven by the PIC32 co-processor in response to short UART commands
//! sent over SCIF1.  The handshake is:
//! 1. CPU sends `250` (DC_LOW) or `251` (DC_HIGH) to set the data/command mode.
//! 2. CPU sends `248` (SELECT_OLED); PIC asserts CS and echoes `248` back.
//! 3. CPU waits for the echo via [`pic::wait_oled_selected()`].
//! 4. CPU streams pixel or command bytes over RSPI0 (8-bit mode).
//! 5. CPU sends `249` (DESELECT_OLED); PIC de-asserts CS and echoes `249` back.
//!
//! ## Frame buffer layout
//! ```text
//! oledMainImage[page][column]
//!
//! page   0 → rows  0– 7   (top, but skipped on 48px display)
//! page   1 → rows  8–15   (skipped)
//! page   2 → rows 16–23   (first visible row on 48px panel)
//! ...
//! page   7 → rows 56–63
//!
//! Within each byte, bit 0 = top pixel of the 8-row group,
//!                  bit 7 = bottom pixel.
//! ```
//! The SSD1309 is configured with page address 2–7 in horizontal addressing
//! mode, so the 768-byte flat dump of pages[2..=7][0..128] fills the panel.
//!
//! ## Usage
//! ```rust,ignore
//! // Once at startup (inside Embassy task, after pic_task has set up the bus):
//! oled::init().await;
//!
//! // Every frame:
//! let mut fb = oled::FrameBuffer::new();
//! oled::set_pixel(&mut fb, 64, 24, true);
//! oled::send_frame(&fb).await;
//! ```

#[cfg(target_os = "none")]
use rza1l_hal::UNCACHED_MIRROR_OFFSET;
#[cfg(target_os = "none")]
use rza1l_hal::dmac;

#[cfg(target_os = "none")]
use crate::pic;

// ── OLED SPI DMA constants ────────────────────────────────────────────────────

/// DMAC channel 4 = OLED SPI TX (matches original C firmware `OLED_SPI_DMA_CHANNEL`).
const OLED_DMA_CH: u8 = 4;

/// RSPI0 SPDR byte LL address (little-endian lowest byte of the 32-bit SPDR
/// at offset 0x04 from RSPI0 base 0xE800_C800).  The DMAC writes each pixel
/// byte here to clock it over SPI.
const RSPI0_SPDR_LL: u32 = 0xE800_C804;

/// DMARS for RSPI0 TX: DMARS_FOR_RSPI_TX = 0b100100001 = 0x121, channel 0, no shift.
const OLED_DMA_DMARS: u32 = 0x121;

/// CHCFG for the OLED SPI DMA TX channel.
///
/// From `oledDMAInit()`:
///   `0b00000000_00100000_00000010_01101000 | (OLED_SPI_DMA_CHANNEL & 7)`
///   = 0x00200268 | 4 = 0x0020026C
const OLED_CHCFG: u32 = 0x0020_026C;

/// Cache-line-aligned DMA staging buffer for OLED frame data.
///
/// The CPU copies the `FrameBuffer` into this buffer via the **uncached alias**
/// so that the DMAC reads coherent data without needing a cache flush.
#[repr(C, align(32))]
struct OledDmaBuf([u8; FRAME_BYTES]);

static mut OLED_DMA_BUF: OledDmaBuf = OledDmaBuf([0; FRAME_BYTES]);

// ── Dimensions ────────────────────────────────────────────────────────────────

/// Physical pixel columns.
pub const WIDTH: usize = 128;
/// Physical pixel rows.
pub const HEIGHT: usize = 48;
/// Number of SSD1309 pages used (HEIGHT / 8).
pub const PAGES: usize = HEIGHT / 8; // = 6
/// Total frame buffer size in bytes.
pub const FRAME_BYTES: usize = PAGES * WIDTH; // = 768

/// Pixel rows at the top of the panel hidden behind the faceplate.
///
/// This is the bezel, **distinct** from `FIRST_PAGE` below: that offset accounts
/// for the 48-row panel sitting in the SSD1309's 64-row RAM, whereas this is the
/// physical cover over the panel. Of the [`HEIGHT`] rows driven, the top
/// `VISIBLE_TOP` are covered, leaving [`VISIBLE_HEIGHT`] visible. Drawing code
/// should offset content down by this much (e.g. the menu's `top_inset`).
pub const VISIBLE_TOP: usize = 5;
/// Visible pixel rows: the [`HEIGHT`] panel minus the [`VISIBLE_TOP`] bezel rows.
pub const VISIBLE_HEIGHT: usize = HEIGHT - VISIBLE_TOP; // = 43

/// First SSD1309 page that maps to display row 0.
///
/// On a 128×64 controller driving a 48-line panel the top 16 lines (pages 0–1)
/// are off-screen.  Page addressing starts at `(64 - HEIGHT) / 8 = 2`.
const FIRST_PAGE: u8 = ((64 - HEIGHT) / 8) as u8; // = 2

// RSPI0 is owned by [`crate::bus`]; the OLED drives it only through a bus guard.

/// Set once [`init`] has run. The panic-path [`draw_blocking`] checks this so it
/// never pokes an uninitialised (and possibly unclocked) RSPI0/PIC — which could
/// hang the panic handler.
#[cfg(target_os = "none")]
static INITIALISED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// Host/QEMU capture surface: holds the frame most recently pushed via
/// [`send_frame`]/[`draw_blocking`]. There is no panel off-target, so a
/// consuming host (e.g. a future GUI/simulator, or a test) reads it back via
/// [`captured_frame`] instead of a real SSD1309.
#[cfg(not(target_os = "none"))]
static CAPTURED_FRAME: std::sync::Mutex<FrameBuffer> = std::sync::Mutex::new(FrameBuffer::new());

/// Host observability flag: flips true the first time [`capture_frame`] runs. A
/// fresh [`CAPTURED_FRAME`] is already all-zero (blank), so `captured_frame()`
/// alone cannot distinguish "no frame sent yet" from "a blank frame was sent" —
/// this flag makes that distinction available to the boot smoke.
#[cfg(not(target_os = "none"))]
static BOOT_FRAME_CAPTURED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

// ── Frame buffer ──────────────────────────────────────────────────────────────

/// 128 × 48 monochrome frame buffer, organized as 6 pages of 128 bytes.
///
/// Index as `buf.pages[page][col]`.  Each byte packs 8 vertical pixels
/// (bit 0 = topmost row of the page, bit 7 = bottommost).
#[derive(Clone)]
pub struct FrameBuffer {
    pub pages: [[u8; WIDTH]; PAGES],
}

impl FrameBuffer {
    /// Create a blank (all pixels off) frame buffer.
    pub const fn new() -> Self {
        Self {
            pages: [[0u8; WIDTH]; PAGES],
        }
    }
}

impl Default for FrameBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameBuffer {
    /// Set or clear the pixel at column `x` (0–127), row `y` (0–47).
    ///
    /// Silently ignores out-of-bounds coordinates.
    #[inline]
    pub fn set_pixel(&mut self, x: usize, y: usize, on: bool) {
        if x >= WIDTH || y >= HEIGHT {
            return;
        }
        let page = y / 8;
        let bit = y % 8;
        if on {
            self.pages[page][x] |= 1 << bit;
        } else {
            self.pages[page][x] &= !(1 << bit);
        }
    }

    /// Return `true` if the pixel at (x, y) is set.
    #[inline]
    pub fn get_pixel(&self, x: usize, y: usize) -> bool {
        if x >= WIDTH || y >= HEIGHT {
            return false;
        }
        (self.pages[y / 8][x] >> (y % 8)) & 1 != 0
    }

    /// Fill all pixels with `byte` (0x00 = all off, 0xFF = all on).
    #[inline]
    pub fn fill(&mut self, byte: u8) {
        for page in self.pages.iter_mut() {
            page.fill(byte);
        }
    }

    /// XOR-invert all pixels.
    #[inline]
    pub fn invert(&mut self) {
        for page in self.pages.iter_mut() {
            for b in page.iter_mut() {
                *b ^= 0xFF;
            }
        }
    }

    /// Return a flat byte slice over all 768 frame bytes (page-major order).
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        // Safety: [[u8; 128]; 6] has the same layout as [u8; 768].
        unsafe { core::slice::from_raw_parts(self.pages.as_ptr() as *const u8, FRAME_BYTES) }
    }
}

// ── embedded-graphics integration (optional) ──────────────────────────────────

/// `DrawTarget`/`OriginDimensions` over the page-major [`FrameBuffer`], so the whole
/// `embedded-graphics` ecosystem (text, fonts, shapes) draws straight onto it and the
/// result is read back as the 768-byte hardware frame via [`FrameBuffer::as_bytes`].
///
/// This is the runtime-agnostic counterpart to `deluge-sdk`'s async `Oled` wrapper:
/// it carries no panel I/O, so hosts that own their render loop (e.g. the spark
/// simulator/engine) can render here on the host target and stream `as_bytes()` to the
/// panel themselves. The surface is the full `128 × 48` panel; offset content down by
/// [`VISIBLE_TOP`] to clear the faceplate bezel.
#[cfg(feature = "embedded-graphics")]
mod eg {
    use super::{FrameBuffer, HEIGHT, WIDTH};
    use embedded_graphics_core::draw_target::DrawTarget;
    use embedded_graphics_core::geometry::{OriginDimensions, Point, Size};
    use embedded_graphics_core::pixelcolor::BinaryColor;
    use embedded_graphics_core::Pixel;

    impl OriginDimensions for FrameBuffer {
        #[inline]
        fn size(&self) -> Size {
            Size::new(WIDTH as u32, HEIGHT as u32)
        }
    }

    impl DrawTarget for FrameBuffer {
        type Color = BinaryColor;
        type Error = core::convert::Infallible;

        #[inline]
        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            for Pixel(Point { x, y }, color) in pixels {
                if x >= 0 && y >= 0 {
                    // `set_pixel` bounds-checks the upper edge.
                    self.set_pixel(x as usize, y as usize, color.is_on());
                }
            }
            Ok(())
        }
    }
}

// ── Text rendering (minimal 5×7 bitmap font) ──────────────────────────────────

/// Minimal 5×7 bitmap text rendering for the OLED frame buffer.
///
/// Supports space, A–Z (digits/letters are upper-cased), 0–9, and a handful of
/// punctuation (`. - _ ! / :`).  Any other byte renders as a solid box so
/// corrupt input is visible.  Each glyph is 5 px wide with a 1 px gap.
pub mod text {
    use super::{FrameBuffer, WIDTH};

    /// Return a 5-column glyph for `ch` (any printable ASCII, upper-cased).
    /// Each byte is one pixel column; bit 0 = top, bit 6 = bottom (bit 7 unused).
    pub fn glyph(ch: u8) -> [u8; 5] {
        match ch.to_ascii_uppercase() {
            b' ' => [0x00, 0x00, 0x00, 0x00, 0x00],
            b'!' => [0x00, 0x5F, 0x00, 0x00, 0x00],
            b'.' => [0x00, 0x60, 0x60, 0x00, 0x00],
            b'-' => [0x08, 0x08, 0x08, 0x08, 0x08],
            b'_' => [0x40, 0x40, 0x40, 0x40, 0x40],
            b'/' => [0x20, 0x10, 0x08, 0x04, 0x02],
            b':' => [0x00, 0x36, 0x36, 0x00, 0x00],
            b'0' => [0x3E, 0x51, 0x49, 0x45, 0x3E],
            b'1' => [0x00, 0x42, 0x7F, 0x40, 0x00],
            b'2' => [0x42, 0x61, 0x51, 0x49, 0x46],
            b'3' => [0x21, 0x41, 0x45, 0x4B, 0x31],
            b'4' => [0x18, 0x14, 0x12, 0x7F, 0x10],
            b'5' => [0x27, 0x45, 0x45, 0x45, 0x39],
            b'6' => [0x3C, 0x4A, 0x49, 0x49, 0x30],
            b'7' => [0x01, 0x71, 0x09, 0x05, 0x03],
            b'8' => [0x36, 0x49, 0x49, 0x49, 0x36],
            b'9' => [0x06, 0x49, 0x49, 0x29, 0x1E],
            b'A' => [0x7E, 0x11, 0x11, 0x11, 0x7E],
            b'B' => [0x7F, 0x49, 0x49, 0x49, 0x36],
            b'C' => [0x3E, 0x41, 0x41, 0x41, 0x22],
            b'D' => [0x7F, 0x41, 0x41, 0x22, 0x1C],
            b'E' => [0x7F, 0x49, 0x49, 0x49, 0x41],
            b'F' => [0x7F, 0x09, 0x09, 0x09, 0x01],
            b'G' => [0x3E, 0x41, 0x49, 0x49, 0x7A],
            b'H' => [0x7F, 0x08, 0x08, 0x08, 0x7F],
            b'I' => [0x00, 0x41, 0x7F, 0x41, 0x00],
            b'J' => [0x20, 0x40, 0x41, 0x3F, 0x01],
            b'K' => [0x7F, 0x08, 0x14, 0x22, 0x41],
            b'L' => [0x7F, 0x40, 0x40, 0x40, 0x40],
            b'M' => [0x7F, 0x02, 0x0C, 0x02, 0x7F],
            b'N' => [0x7F, 0x04, 0x08, 0x10, 0x7F],
            b'O' => [0x3E, 0x41, 0x41, 0x41, 0x3E],
            b'P' => [0x7F, 0x09, 0x09, 0x09, 0x06],
            b'Q' => [0x3E, 0x41, 0x51, 0x21, 0x5E],
            b'R' => [0x7F, 0x09, 0x19, 0x29, 0x46],
            b'S' => [0x46, 0x49, 0x49, 0x49, 0x31],
            b'T' => [0x01, 0x01, 0x7F, 0x01, 0x01],
            b'U' => [0x3F, 0x40, 0x40, 0x40, 0x3F],
            b'V' => [0x1F, 0x20, 0x40, 0x20, 0x1F],
            b'W' => [0x3F, 0x40, 0x38, 0x40, 0x3F],
            b'X' => [0x63, 0x14, 0x08, 0x14, 0x63],
            b'Y' => [0x07, 0x08, 0x70, 0x08, 0x07],
            b'Z' => [0x61, 0x51, 0x49, 0x45, 0x43],
            _ => [0x7F, 0x41, 0x41, 0x41, 0x7F], // solid box for unknown
        }
    }

    /// Draw an ASCII string at pixel (`x`, `y`) in the frame buffer.
    /// Characters are 5 px wide + 1 px gap; strings are clipped to display width.
    pub fn draw_str(fb: &mut FrameBuffer, mut x: usize, y: usize, s: &[u8]) {
        for &ch in s {
            if x + 6 > WIDTH {
                break;
            }
            let cols = glyph(ch);
            for (col_idx, &col_bits) in cols.iter().enumerate() {
                for bit in 0..7u8 {
                    if col_bits & (1 << bit) != 0 {
                        fb.set_pixel(x + col_idx, y + bit as usize, true);
                    }
                }
            }
            x += 6;
        }
    }
}

// ── SSD1309 init sequence helpers ─────────────────────────────────────────────

/// Complete SSD1309 initialisation command sequence, in order.
///
/// These are the commands used in `oledMainInit()` in the C firmware.
/// They configure the controller for 128×48 pixels with horizontal
/// page-addressing starting at page [`FIRST_PAGE`].
const INIT_CMDS: &[u8] = &[
    0xFD, 0x12, // SET COMMAND LOCK — unlock (0x12 = allow all commands)
    0xAE, // DISPLAY OFF
    0x81, 0xFF, // CONTRAST = maximum
    0xA4, // ENTIRE DISPLAY ON = off (normal RAM content)
    0xA6, // SET NORMAL DISPLAY (0xA7 = inverse)
    0x00, 0x10, // SET COLUMN START ADDRESS (low nibble = 0, high nibble = 0)
    0x20, 0x00, // SET MEMORY ADDRESSING MODE = horizontal
    0x22, FIRST_PAGE, 7,    // SET PAGE ADDRESS: start=FIRST_PAGE (2), end=7
    0x40, // SET DISPLAY START LINE = 0
    0xA0, // SET SEGMENT RE-MAP (A0 = normal, A1 = mirrored)
    0xA8, 0x3F, // SET MULTIPLEX RATIO = 63 (64 COM lines on controller)
    0xC0, // COM OUTPUT SCAN DIRECTION: COM0→COM63 (top→bottom)
    0xD3, 0x00, // SET DISPLAY OFFSET = 0
    0xDA, 0x12, // COM PIN HARDWARE CONFIGURATION = sequential
    0xD5, 0xF0, // SET DISPLAY CLOCK DIVIDE RATIO / OSCILLATOR FREQUENCY
    0xD9, 0xA2, // SET PRE-CHARGE PERIOD
    0xDB, 0x34, // SET VCOM DESELECT LEVEL
    0xAF, // DISPLAY ON
];

/// Send all INIT_CMDS bytes via RSPI0. The bus guard must already be in 8-bit
/// mode (see [`crate::bus::Rspi0::enter_8bit`]).
#[cfg(target_os = "none")]
fn send_init_commands(bus: &mut crate::bus::Rspi0) {
    for &b in INIT_CMDS {
        bus.send8(b);
    }
    bus.wait_end();
}

// ── Public async API (embedded only) ─────────────────────────────────────────

/// Perform full OLED initialisation.
///
/// Mirrors the `setup_oled()` sequence from `oled_init.c`:
/// 1. Configure RSPI0 for 8-bit mode.
/// 2. Send PIC commands: DC_LOW, ENABLE_OLED, SELECT_OLED.
/// 3. Wait 5 ms for the PIC to power up the display.
/// 4. Send the 21-command SSD1309 init sequence.
/// 5. Switch DC to high (data mode for subsequent frame writes).
/// 6. Deselect via PIC.
///
/// **Must be called inside an Embassy task**, after `pic::init().await`
/// has completed.
#[cfg(target_os = "none")]
pub async fn init() {
    use embassy_time::Timer;
    log::debug!("oled: init");

    // ---- 1. Take RSPI0 and configure it for 8-bit SPI mode ----------------
    // The guard is held across the whole init sequence (including the PIC and
    // timer awaits below) so a concurrent CV write cannot reconfigure or drive
    // RSPI0 mid-init.
    let mut bus = crate::bus::lock_rspi0().await;
    bus.enter_8bit();

    // ---- 1b. Set up DMAC channel 4 for OLED SPI TX ----------------------
    // Mirrors oledDMAInit() from the C firmware.  Must be done after RSPI0
    // is initialised (stb clock enabled) but before the first send_frame.
    unsafe {
        dmac::init_register_mode(OLED_DMA_CH, OLED_CHCFG, RSPI0_SPDR_LL, OLED_DMA_DMARS);
        dmac::register_completion_irq(OLED_DMA_CH);
    }

    // ---- 2. PIC: DC_LOW, ENABLE_OLED, SELECT_OLED -----------------------
    // Mirrors the original C firmware's setupOLED() exactly:
    //   PIC::setDCLow(); PIC::enableOLED(); PIC::selectOLED(); PIC::flush();
    // The PIC will echo these bytes back; pic_task will parse and discard
    // them (250→ignored, 247→ignored, 248→sets SELECTED_FLAG transiently).
    // We do NOT wait for the echo here — the C firmware never did either.
    pic::oled_dc_low().await;
    pic::oled_enable().await;
    pic::oled_select().await;

    // ---- 3. Wait for PIC to assert CS and echo SELECT_OLED (byte 248) ----
    pic::wait_oled_selected().await;

    // ---- 3b. Pre-clear full 128 × 64 controller RAM ----------------------
    //
    // The SSD1309 powers up with display OFF and undefined GDDRAM.  The panel
    // is only 48 rows tall, but the controller has 64 rows of RAM; the upper
    // 16 rows (pages 0–1) are physically off-screen but could show artefacts
    // if the page range is later changed.  Writing zeros now clears all 8
    // pages before INIT_CMDS locks the visible window to pages 2–7.
    //
    // DC is LOW (command mode) and CS is already asserted by the PIC.
    // Configure horizontal addressing, full column range 0–127, all pages 0–7.
    for &b in &[
        0x20u8, 0x00, // SET MEMORY ADDRESSING MODE = horizontal
        0x21, 0x00, 0x7F, // SET COLUMN ADDRESS: 0–127
        0x22, 0x00, 0x07, // SET PAGE ADDRESS:   0–7  (full 64 rows)
    ] {
        bus.send8(b);
    }
    bus.wait_end();
    // Switch to data mode, give PIC 1 ms to process the DC toggle.
    pic::oled_dc_high().await;
    Timer::after_millis(1).await;
    // 8 pages × 128 columns = 1 024 zero bytes → clears all GDDRAM.
    for _ in 0..(8 * WIDTH) {
        bus.send8(0x00);
    }
    bus.wait_end();
    // Back to command mode before INIT_CMDS.
    pic::oled_dc_low().await;
    Timer::after_millis(1).await;

    // ---- 4. Send SSD1309 init commands -----------------------------------
    send_init_commands(&mut bus);

    // ---- 5. Switch DC to high (data mode for subsequent frame writes) ----
    pic::oled_dc_high().await;
    log::debug!("oled: dc_high sent");

    // ---- 6. Deselect, and wait for the PIC to confirm (byte 249) ----------
    // Must wait for the echo before the bus guard drops at end of scope: the
    // anchor we just set (pages 2–7, col 0) is never re-sent — send_frame relies
    // on the 768-byte auto-wrap returning to it. If we released the bus while the
    // OLED CS were still asserted, the first CV write would clock a word into the
    // OLED and shift the column pointer *once* — and since it's never re-anchored,
    // that shift is baked in for every subsequent frame (a fixed horizontal
    // offset). Holding the bus until deselect is confirmed prevents it.
    pic::oled_deselect().await;
    pic::wait_oled_deselected().await;
    INITIALISED.store(true, core::sync::atomic::Ordering::Release);
    log::debug!("oled: ready");
}

/// Host/QEMU stand-in for [`init`]: no RSPI0/PIC/DMAC hardware to bring up, so
/// this just returns. Resolves without ever suspending.
#[cfg(not(target_os = "none"))]
pub async fn init() {
    log::debug!("oled(host): init (no-op)");
}

/// Copy `fb` into the host capture surface, standing in for actually driving
/// the panel. Shared by the host [`send_frame`] and [`draw_blocking`].
#[cfg(not(target_os = "none"))]
fn capture_frame(fb: &FrameBuffer) {
    let mut captured = CAPTURED_FRAME
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    *captured = fb.clone();
    BOOT_FRAME_CAPTURED.store(true, core::sync::atomic::Ordering::Release);
}

/// Read back the frame most recently pushed via [`send_frame`]/[`draw_blocking`]
/// on host/QEMU. There is no real panel off-target, so this is how a
/// consumer (or a test) observes what would have been drawn.
#[cfg(not(target_os = "none"))]
pub fn captured_frame() -> FrameBuffer {
    CAPTURED_FRAME
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .clone()
}

/// Host observability: true once the render task has pushed at least one frame
/// through the sim (i.e. `oled_render` completed `init()` + its first
/// `send_frame`). The host boot smoke waits on this to confirm the display path
/// booted, distinguishing a sent blank frame from the sim's blank initial state.
#[cfg(not(target_os = "none"))]
pub fn boot_frame_captured() -> bool {
    BOOT_FRAME_CAPTURED.load(core::sync::atomic::Ordering::Acquire)
}

/// Render a frame using a fully blocking, interrupt-free path — for a panic
/// handler, where the executor and IRQs are dead so the async [`send_frame`]
/// (DMA + PIC echo wait) cannot run.
///
/// Best-effort: no-ops if [`init`] never ran (so it can't poke an unclocked
/// RSPI0/PIC and hang), re-sends the addressing window in case a frame was
/// interrupted mid-stream, and streams bytes via polling `send8`. Uses
/// poll-based `ostm::delay_ms` for the PIC handshake timing.
///
/// # Safety
/// Call only from a single-threaded context with interrupts masked (a panic
/// handler): it steals the RSPI0 bus token without locking.
#[cfg(target_os = "none")]
pub unsafe fn draw_blocking(fb: &FrameBuffer) {
    use rza1l_hal::ostm;

    if !INITIALISED.load(core::sync::atomic::Ordering::Acquire) {
        return;
    }

    // SAFETY: panic context — we are the only RSPI0 user; no locking possible.
    // `ostm::delay_ms` is poll-based (no IRQ), safe here.
    unsafe {
        let mut bus = crate::bus::steal_rspi0();

        pic::oled_select_blocking();
        ostm::delay_ms(2);
        bus.enter_8bit();

        // Reset the GDDRAM window (columns 0–127, pages FIRST_PAGE–7) in command
        // mode, in case the panic interrupted a frame mid-stream.
        pic::oled_dc_low_blocking();
        ostm::delay_ms(1);
        for &b in &[0x21u8, 0x00, 0x7F, 0x22, FIRST_PAGE, 7] {
            bus.send8(b);
        }
        bus.wait_end();

        // Stream the frame in data mode.
        pic::oled_dc_high_blocking();
        ostm::delay_ms(1);
        for &b in fb.as_bytes() {
            bus.send8(b);
        }
        bus.wait_end();

        pic::oled_deselect_blocking();
    }
}

/// Host/QEMU stand-in for [`draw_blocking`]: no panel or panic-path timing
/// concerns off-target, so this just captures `fb` (see [`captured_frame`]).
///
/// # Safety
/// No unsafe operations; `unsafe` only to keep this a drop-in match for the
/// device signature so callers need no `#[cfg]` of their own.
#[cfg(not(target_os = "none"))]
pub unsafe fn draw_blocking(fb: &FrameBuffer) {
    capture_frame(fb);
}

/// Send a full 768-byte frame to the display using DMAC channel 4.
///
/// The SPI CS handshake goes through the PIC:
/// 1. Send SELECT_OLED to PIC; await the echo confirming CS is asserted.
/// 2. Copy the frame into the uncached DMA staging buffer.
/// 3. Configure RSPI0 for 8-bit mode.
/// 4. Arm and start DMAC ch4 to stream all 768 bytes → RSPI0 SPDR.
/// 5. Await DMAC completion interrupt (DMAINT4, GIC 45).
/// 6. Send DESELECT_OLED to PIC; wait 1 ms.
///
/// DC is assumed to already be high (data mode) from init or a previous frame.
///
/// **Must be called inside an Embassy task.**
#[cfg(target_os = "none")]
pub async fn send_frame(fb: &FrameBuffer) {
    // Take exclusive ownership of RSPI0 for the ENTIRE select→DMA→deselect
    // sequence — acquired BEFORE selecting the OLED. The OLED's chip-select is
    // driven by the PIC asynchronously, so if the bus were free at any moment
    // while the OLED CS is asserted, a concurrent CV write would clock its word
    // on the shared SCK/MOSI into the still-selected OLED and drift its GDDRAM
    // column pointer (horizontal shift). Selecting only after taking the bus
    // closes the select-side window; deselecting + settling before releasing it
    // (below) closes the deselect-side window. Matches oled::init's ordering.
    let mut bus = crate::bus::lock_rspi0().await;

    // Reconfigure 8-bit mode every frame (NOT the cached enter_8bit), and do it
    // BEFORE selecting the OLED — while its CS is still de-asserted. Two reasons:
    //  1. The OLED full-duplex DMA fills the RX FIFO, which must be reset per
    //     frame via SPBFCR.RXRST or the next frame mis-paces (horizontal stretch).
    //  2. When a CV write has left the bus in 32-bit mode, this is a real 32→8
    //     reconfigure that rewrites SPCMD0 (clock polarity/phase, bit-rate, data
    //     length). Doing that while the OLED CS is asserted clocks the switching
    //     transient on SCK into the OLED as a phantom bit → the image shifts. With
    //     CV idle the bus is already 8-bit so it's a no-op and the bug is hidden;
    //     reconfiguring before select makes it safe in both cases. Matches
    //     oled::init, which also reconfigures before it selects.
    bus.reconfigure_8bit();

    pic::oled_select().await;
    pic::wait_oled_selected().await;

    unsafe {
        // Copy frame into the uncached staging buffer so DMAC sees fresh data.
        let dst_uncached =
            (core::ptr::addr_of!(OLED_DMA_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *mut u8;
        core::ptr::copy_nonoverlapping(fb.as_bytes().as_ptr(), dst_uncached, FRAME_BYTES);

        // Arm DMA: source must be the uncached alias address because N0SA is the
        // bus address the DMAC will read from — same physical RAM as the cached
        // alias but coherent since we wrote through the uncached window above.
        let src_uncached = dst_uncached as u32;
        dmac::start_transfer(OLED_DMA_CH, src_uncached, FRAME_BYTES as u32);
    }

    // Await DMA completion interrupt (still holding the RSPI0 guard).
    dmac::wait_transfer_complete(OLED_DMA_CH).await;

    // Wait for RSPI0 to finish clocking out its FIFO/shift register (TEND=1).
    // DMA completion only means all bytes have been *enqueued*; deselecting
    // before TEND causes the SSD1309 to receive a truncated frame and its
    // GDDRAM address pointer drifts, corrupting every subsequent frame.
    bus.wait_end();

    // Deselect the OLED *before* releasing the bus. The OLED's chip-select is
    // driven by the PIC asynchronously (a UART round-trip), unlike the CV DAC's
    // CS which is a direct GPIO toggle. If we dropped the bus first, a waiting CV
    // write could acquire RSPI0 and clock its 32-bit word on the shared SCK/MOSI
    // while the OLED CS is still physically asserted — the OLED would ingest the
    // CV word and its GDDRAM column pointer would drift (horizontal offset /
    // first-page corruption). Holding the bus until the deselect has settled
    // guarantees nothing else clocks the bus while the OLED is still listening.
    // Symmetric with the select path above, which waits via wait_oled_selected
    // before touching the bus. (Sparse CV users rarely hit this window; a fast
    // OLED+CV interleave — e.g. an external app driving CV every audio block —
    // hits it constantly.)
    //
    // Wait for the PIC to *echo* the deselect (byte 249) rather than a blind
    // delay: the DESELECT command is queued behind any other PIC TX traffic
    // (e.g. pad-LED updates), so a fixed timeout can elapse while the CS is still
    // asserted. The echo means the PIC has processed it and the CS is physically
    // de-asserted — only then is it safe to release the bus.
    pic::oled_deselect().await;
    pic::wait_oled_deselected().await;

    // Release RSPI0 — only now, with the OLED deselected, may a CV write proceed.
    drop(bus);
}

/// Host/QEMU stand-in for [`send_frame`]: no RSPI0/PIC/DMAC hardware to drive,
/// so this just captures `fb` (see [`captured_frame`]). Resolves without ever
/// suspending.
#[cfg(not(target_os = "none"))]
pub async fn send_frame(fb: &FrameBuffer) {
    capture_frame(fb);
}

// ── Redraw signal ─────────────────────────────────────────────────────────────
//
// Allow the firmware's UI task to notify the OLED render task that the display
// contents have changed without any shared-memory lock. Pure `embassy-sync`
// logic (no MMIO), so this is unconditional — host and device share it.

mod redraw_signal {
    use core::future::poll_fn;
    use core::sync::atomic::{AtomicBool, Ordering};
    use core::task::Poll;
    use embassy_sync::waitqueue::AtomicWaker;

    static FLAG: AtomicBool = AtomicBool::new(true); // start dirty so first frame is sent
    static WAKER: AtomicWaker = AtomicWaker::new();

    pub fn notify() {
        FLAG.store(true, Ordering::Release);
        WAKER.wake();
    }

    pub async fn wait() {
        poll_fn(|cx| {
            WAKER.register(cx.waker());
            if FLAG.swap(false, Ordering::AcqRel) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

/// Mark the OLED display as needing a redraw.
///
/// Call this from any task whenever the contents to be displayed have changed.
/// [`wait_redraw()`] in the render task will unblock and push a new frame.
#[inline]
pub fn notify_redraw() {
    redraw_signal::notify();
}

/// Suspend until [`notify_redraw()`] has been called since the last resume.
///
/// Suitable as the loop condition of a render task:
/// ```rust,ignore
/// loop {
///     oled::wait_redraw().await;
///     render_frame(&mut fb);
///     oled::send_frame(&fb).await;
/// }
/// ```
#[inline]
pub async fn wait_redraw() {
    redraw_signal::wait().await;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn dimensions() {
        assert_eq!(WIDTH, 128);
        assert_eq!(HEIGHT, 48);
        assert_eq!(PAGES, 6);
        assert_eq!(FRAME_BYTES, 768);
        assert_eq!(FIRST_PAGE, 2);
    }

    #[test]
    fn framebuffer_zeroed_by_default() {
        let fb = FrameBuffer::new();
        assert!(fb.as_bytes().iter().all(|&b| b == 0));
    }

    #[test]
    fn framebuffer_fill() {
        let mut fb = FrameBuffer::new();
        fb.fill(0xFF);
        assert!(fb.as_bytes().iter().all(|&b| b == 0xFF));
    }

    #[test]
    fn set_and_get_pixel_top_left() {
        let mut fb = FrameBuffer::new();
        fb.set_pixel(0, 0, true);
        assert!(fb.get_pixel(0, 0));
        assert_eq!(fb.pages[0][0], 0b0000_0001); // bit 0 = row 0
    }

    #[test]
    fn set_and_get_pixel_bottom_right() {
        let mut fb = FrameBuffer::new();
        fb.set_pixel(127, 47, true);
        assert!(fb.get_pixel(127, 47));
        // row 47 → page 5 (47/8=5), bit 7 (47%8=7)
        assert_eq!(fb.pages[5][127], 0b1000_0000);
    }

    #[test]
    fn set_pixel_clears_correctly() {
        let mut fb = FrameBuffer::new();
        fb.fill(0xFF);
        fb.set_pixel(10, 10, false);
        // row 10 → page 1, bit 2
        assert_eq!(fb.pages[1][10], 0b1111_1011);
    }

    #[test]
    fn out_of_bounds_ignored() {
        let mut fb = FrameBuffer::new();
        fb.set_pixel(128, 0, true); // column OOB
        fb.set_pixel(0, 48, true); // row OOB
        assert!(!fb.get_pixel(128, 0));
        assert!(!fb.get_pixel(0, 48));
        assert!(fb.as_bytes().iter().all(|&b| b == 0));
    }

    #[test]
    fn invert_roundtrip() {
        let mut fb = FrameBuffer::new();
        fb.set_pixel(5, 5, true);
        fb.invert();
        assert!(!fb.get_pixel(5, 5));
        fb.invert();
        assert!(fb.get_pixel(5, 5));
    }

    #[test]
    fn as_bytes_length() {
        let fb = FrameBuffer::new();
        assert_eq!(fb.as_bytes().len(), FRAME_BYTES);
    }

    #[test]
    fn init_cmds_reasonable_length() {
        // SSD1309 init needs at least 10 command bytes; ours has 21.
        assert!(INIT_CMDS.len() >= 10);
    }

    #[test]
    fn page_addressing_for_48px() {
        // 128×48 display starts at page (64-48)/8 = 2
        assert_eq!(FIRST_PAGE, 2);
        // Verify INIT_CMDS contains the SET PAGE ADDRESS sequence
        let page_cmd_pos = INIT_CMDS
            .windows(3)
            .position(|w| w[0] == 0x22 && w[1] == FIRST_PAGE && w[2] == 7);
        assert!(
            page_cmd_pos.is_some(),
            "SET PAGE ADDRESS (0x22) not found in init commands"
        );
    }

    #[cfg(feature = "embedded-graphics")]
    #[test]
    fn draw_target_writes_page_major() {
        use embedded_graphics_core::draw_target::DrawTarget;
        use embedded_graphics_core::geometry::{OriginDimensions, Point, Size};
        use embedded_graphics_core::pixelcolor::BinaryColor;
        use embedded_graphics_core::Pixel;

        let mut fb = FrameBuffer::new();
        assert_eq!(fb.size(), Size::new(WIDTH as u32, HEIGHT as u32));

        // Drawing through the DrawTarget must land in the same page-major bytes
        // `as_bytes()` reinterprets, so a host render is the hardware frame for free.
        fb.draw_iter([
            Pixel(Point::new(0, 0), BinaryColor::On),
            Pixel(Point::new(127, 47), BinaryColor::On),
            // Off-canvas pixels are silently dropped (no panic, no wrap).
            Pixel(Point::new(-1, 5), BinaryColor::On),
            Pixel(Point::new(200, 5), BinaryColor::On),
        ])
        .unwrap();

        assert!(fb.get_pixel(0, 0));
        assert!(fb.get_pixel(127, 47));
        assert_eq!(fb.pages[0][0], 0b0000_0001);
        assert_eq!(fb.pages[5][127], 0b1000_0000);
    }

    /// `send_frame` on host captures rather than drives real hardware; reading
    /// it back must equal what was sent.
    #[test]
    fn send_frame_captures_readable_frame() {
        embassy_futures::block_on(init());

        let mut fb = FrameBuffer::new();
        fb.set_pixel(3, 3, true);
        fb.set_pixel(100, 40, true);

        embassy_futures::block_on(send_frame(&fb));

        let captured = captured_frame();
        assert_eq!(captured.as_bytes(), fb.as_bytes());
    }

    /// `draw_blocking` (the panic-path host stand-in) also captures.
    #[test]
    fn draw_blocking_captures_readable_frame() {
        let mut fb = FrameBuffer::new();
        fb.fill(0xAA);

        unsafe { draw_blocking(&fb) };

        let captured = captured_frame();
        assert_eq!(captured.as_bytes(), fb.as_bytes());
    }

    /// `capture_frame` (shared by `send_frame`/`draw_blocking`) must flip the
    /// boot-capture flag once it has stored a frame.
    #[test]
    fn host_boot_frame_captured_flips_on_capture() {
        // Process-global flag: it may already be true if another oled test captured
        // a frame first — that's fine, this asserts capture SETS it, never clears it.
        let mut fb = FrameBuffer::new();
        fb.fill(0x00);
        capture_frame(&fb); // the same host path send_frame/draw_blocking funnel through
        assert!(boot_frame_captured());
    }

    /// `wait_redraw` resolves immediately once `notify_redraw` has fired (the
    /// redraw signal starts dirty, so even a fresh call resolves).
    #[test]
    fn wait_redraw_resolves() {
        notify_redraw();
        embassy_futures::block_on(wait_redraw());
    }
}
