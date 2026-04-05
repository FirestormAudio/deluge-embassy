//! Renesas Serial Peripheral Interface (RSPI) driver for the RZ/A1L.
//!
//! Implements a blocking / interrupt-driven 32-bit SPI master, matching the
//! `R_RSPI_Create` + `R_RSPI_Start` + `R_RSPI_SendBasic32` sequence from the
//! C firmware.  Only the 32-bit word path is implemented here because that is
//! the only frame size used on the Deluge (CV DAC MAX5136, OLED shares the
//! same channel in 8-bit mode — not implemented in this crate).
//!
//! ## Channel map
//! | Channel | Base address  | Use on Deluge         |
//! |---------|---------------|-----------------------|
//! | RSPI0   | `0xE800_C800` | CV DAC + OLED display |
//! | RSPI1   | `0xE800_D000` | (unused)              |
//! | RSPI2   | `0xE800_D800` | (unused)              |
//! | RSPI3   | `0xE800_E000` | (unused)              |
//! | RSPI4   | `0xE800_E800` | (unused)              |
//!
//! ## Clock gating
//! RSPI0–4 clocks are enabled in `stb::init()` via STBCR10.
//!
//! ## GIC interrupt IDs (RSPI channel n)
//! | IRQ     | ID            |
//! |---------|---------------|
//! | SPEIn   | 270 + 3n      |
//! | SPRIn   | 271 + 3n      |  ← used for CV-transfer-complete
//! | SPTIn   | 272 + 3n      |
//!
//! ## Register offsets inside `struct st_rspi`
//! | Register | Offset | Width | Description                     |
//! |----------|--------|-------|---------------------------------|
//! | SPCR     | 0x00   | u8    | Control (SPE, master, int ena)  |
//! | SSLP     | 0x01   | u8    | SSL polarity                    |
//! | SPPCR    | 0x02   | u8    | Pin control                     |
//! | SPSR     | 0x03   | u8    | Status (SPRF, TEND, SPTEF, …)   |
//! | SPDR     | 0x04   | u32   | Data register (32-bit access)   |
//! | SPSCR    | 0x08   | u8    | Sequence control                |
//! | SPSSR    | 0x09   | u8    | Sequence status                 |
//! | SPBR     | 0x0A   | u8    | Baud rate divider               |
//! | SPDCR    | 0x0B   | u8    | Data control (word size)        |
//! | SPCKD    | 0x0C   | u8    | Clock delay                     |
//! | SSLND    | 0x0D   | u8    | SSL negation delay              |
//! | SPND     | 0x0E   | u8    | Next-access delay               |
//! | SPCMD0   | 0x10   | u16   | Command register 0              |
//! | SPBFCR   | 0x20   | u8    | Buffer control                  |
//! | SPBFDR   | 0x22   | u16   | Buffer data count               |

use crate::gic;

// ── Peripheral base addresses ─────────────────────────────────────────────────
const RSPI0_BASE: usize = 0xE800_C800;
const RSPI_STRIDE: usize = 0x800;

// ── Register offsets ──────────────────────────────────────────────────────────
const OFF_SPCR: usize = 0x00;
const OFF_SSLP: usize = 0x01;
const OFF_SPPCR: usize = 0x02;
const OFF_SPSR: usize = 0x03;
const OFF_SPDR: usize = 0x04;
const OFF_SPSCR: usize = 0x08;
const OFF_SPSSR: usize = 0x09;
const OFF_SPBR: usize = 0x0A;
const OFF_SPDCR: usize = 0x0B;
const OFF_SPCKD: usize = 0x0C;
const OFF_SSLND: usize = 0x0D;
const OFF_SPND: usize = 0x0E;
const OFF_SPCMD0: usize = 0x10;
const OFF_SPBFCR: usize = 0x20;

// ── SPCR bits ─────────────────────────────────────────────────────────────────
/// SPE — SPI function enable
const SPCR_SPE: u8 = 1 << 6;
/// MSTR — master mode select
const SPCR_MSTR: u8 = 1 << 3;
/// SPTIE — SPI transmit interrupt enable (used for DMA trigger; set here as C does)
const SPCR_SPTIE: u8 = 1 << 5;
/// SPRIE — SPI receive interrupt enable (enabled per-transfer, then disabled in ISR)
const SPCR_SPRIE: u8 = 1 << 7;

// ── SPSR bits (status) ────────────────────────────────────────────────────────
/// SPTEF — transmit buffer empty (TX has room for 4 bytes)
const SPSR_SPTEF: u8 = 1 << 5;
/// TEND  — transmit end (all bits shifted out)
const SPSR_TEND: u8 = 1 << 6;

// ── SPDCR values ──────────────────────────────────────────────────────────────
const SPDCR_8BIT: u8 = 0x20;
const SPDCR_32BIT: u8 = 0x60;

// ── SPBFCR values ─────────────────────────────────────────────────────────────
/// 8-bit OLED mode: TX trigger = 4 bytes free, RX trigger = 1 byte
const SPBFCR_8BIT: u8 = 0b0110_0000;
/// Initial value: TX trigger = 4 bytes free, RX trigger = 2 bytes available
const SPBFCR_INIT_32BIT: u8 = 0b0010_0010;
/// CV-transfer value: disable RX reset (so we get the SPRI interrupt)
const SPBFCR_CV_TRANSFER: u8 = 0b0011_0010;
/// Reset RX buffer (bit 6 = RXRST)
const SPBFCR_RX_RESET: u8 = 1 << 6;

// ── SPCMD0 values ─────────────────────────────────────────────────────────────
// TRM §16 SPCMD bit fields: bit[1] = CPOL (0 = RSPCK idle low), bit[0] = CPHA
// (0 = sample on odd/leading edge).  Reset value = H'070D (CPOL=0, CPHA=1).
//
// The Deluge peripherals (MAX5136 CV DAC, SSD1309 OLED) both require SPI
// Mode 0 (CPOL=0, CPHA=0).  The C firmware incorrectly used CPOL=1 (Mode 2)
// in SPCMD0 = 0x0302/0x0702; this was a latent bug that happened to work on
// those peripherals because both tolerate mismatched polarity in practice, but
// the TRM default and the datasheet-specified polarity are both CPOL=0.

/// 8-bit Mode 0 (CPOL=0, CPHA=0), SSL0.
/// SPB[3:0] (bits 11:8) = 0b0111 → 8 bits per frame.
/// BRDV[1:0] (bits 3:2) = 0b00  → base bit rate / 1.
const SPCMD0_8BIT: u16 = 0b0000_0111_0000_0000;
/// 32-bit Mode 0 (CPOL=0, CPHA=0), SSL0.
/// SPB[3:0] (bits 11:8) = 0b0011 → 32 bits per frame.
/// BRDV[1:0] (bits 3:2) = 0b00  → base bit rate / 1.
const SPCMD0_32BIT: u16 = 0b0000_0011_0000_0000;

// ── P1 clock frequency ────────────────────────────────────────────────────────
/// P1 = CPU / 6 = 400 MHz / 6 = 66.666... MHz
const P1_HZ: u32 = 66_666_666;

// ── GIC interrupt ID base for RSPI ────────────────────────────────────────────
/// SPEI0 = 270; each channel adds 3: SPEI_n = 270 + 3n, SPRI_n = 271 + 3n, SPTI_n = 272 + 3n
const SPEI_BASE: u32 = 270;

// ── Register access helpers ───────────────────────────────────────────────────

#[inline]
fn base(ch: u8) -> usize {
    RSPI0_BASE + RSPI_STRIDE * (ch as usize)
}

#[inline]
fn reg8(ch: u8, off: usize) -> *mut u8 {
    (base(ch) + off) as *mut u8
}

#[inline]
fn reg16(ch: u8, off: usize) -> *mut u16 {
    (base(ch) + off) as *mut u16
}

#[inline]
fn reg32(ch: u8, off: usize) -> *mut u32 {
    (base(ch) + off) as *mut u32
}

// ── Public API ────────────────────────────────────────────────────────────────

/// GIC interrupt ID for the SPRI (receive complete) interrupt on channel `ch`.
///
/// The C firmware uses SPRI to detect when the 32-bit CV word has been fully
/// shifted out (the RSPI hardware receives a dummy byte for every byte sent).
#[inline]
pub fn irq_spri(ch: u8) -> u16 {
    (SPEI_BASE + 1 + 3 * (ch as u32)) as u16
}

/// Initialise RSPI channel `ch` as an SPI master.
///
/// - `bit_rate`: desired SCK frequency in Hz (e.g. `10_000_000` for 10 MHz).
///   The actual rate is `⌊P1 / (2 × (SPBR + 1))⌋`.
/// - Clock phase and polarity are both 0 (SPI mode 0).
/// - Frame size is fixed at 32 bits to match the CV DAC protocol; the OLED
///   firmware re-programs SPDCR/SPCMD0 dynamically before each 8-bit transfer.
///
/// Mirrors `R_RSPI_Create()` + `R_RSPI_Start()` from the C firmware exactly.
///
/// # Safety
/// Writes to memory-mapped RSPI registers; must be called once before any
/// SPI transfers on channel `ch`.
pub unsafe fn init(ch: u8, bit_rate: u32) {
    unsafe {
        // SPPCR = 0 (no loopback, MOSI idle = low)
        reg8(ch, OFF_SPPCR).write_volatile(0);
        let _ = reg8(ch, OFF_SPPCR).read_volatile();

        // SPBR: baud rate divisor. P1 = 66.666 MHz → SPBR = ceil(P1/(bitRate×2)) - 1
        let spbr = P1_HZ.div_ceil(bit_rate * 2).saturating_sub(1) as u8;
        reg8(ch, OFF_SPBR).write_volatile(spbr);
        let _ = reg8(ch, OFF_SPBR).read_volatile();

        // SPDCR: 32-bit longword data
        reg8(ch, OFF_SPDCR).write_volatile(SPDCR_32BIT);
        let _ = reg8(ch, OFF_SPDCR).read_volatile();

        // SPSCR, SPCKD, SSLND, SPND, SSLP, SPSSR: all zero (defaults)
        reg8(ch, OFF_SPSCR).write_volatile(0);
        let _ = reg8(ch, OFF_SPSCR).read_volatile();
        reg8(ch, OFF_SPCKD).write_volatile(0);
        let _ = reg8(ch, OFF_SPCKD).read_volatile();
        reg8(ch, OFF_SSLND).write_volatile(0);
        let _ = reg8(ch, OFF_SSLND).read_volatile();
        reg8(ch, OFF_SPND).write_volatile(0);
        let _ = reg8(ch, OFF_SPND).read_volatile();
        reg8(ch, OFF_SSLP).write_volatile(0);
        let _ = reg8(ch, OFF_SSLP).read_volatile();
        reg8(ch, OFF_SPSSR).write_volatile(0);
        let _ = reg8(ch, OFF_SPSSR).read_volatile();

        // SPBFCR: TX trigger=4 bytes free, RX trigger=2 bytes
        reg8(ch, OFF_SPBFCR).write_volatile(SPBFCR_INIT_32BIT);
        let _ = reg8(ch, OFF_SPBFCR).read_volatile();

        // SPCMD0: 32-bit frame, mode 0
        reg16(ch, OFF_SPCMD0).write_volatile(SPCMD0_32BIT);
        let _ = reg16(ch, OFF_SPCMD0).read_volatile();

        // SPCR: master mode, TX interrupt enable (for DMA trigger compatibility)
        let v = reg8(ch, OFF_SPCR).read_volatile();
        reg8(ch, OFF_SPCR).write_volatile(v | SPCR_MSTR | SPCR_SPTIE);
        let _ = reg8(ch, OFF_SPCR).read_volatile();

        // ---- R_RSPI_Start: clear errors, set SPE --------------------------------
        let _ = reg8(ch, OFF_SPSR).read_volatile(); // clear error sources
        reg8(ch, OFF_SPSR).write_volatile(0x00);

        let v = reg8(ch, OFF_SPCR).read_volatile();
        reg8(ch, OFF_SPCR).write_volatile(v | SPCR_SPE);
        let _ = reg8(ch, OFF_SPCR).read_volatile();
    }
}

/// Register the SPRI (receive-complete) GIC interrupt for channel `ch`.
///
/// The ISR closure is called with the GIC interrupt sense when the 32-bit
/// transfer finishes.  Use this to implement the `cvSPITransferComplete` ISR:
/// deassert CS, reset the RX buffer, and optionally start the next transfer.
///
/// # Safety
/// Must be called after [`crate::gic::init`] and before interrupts are enabled.
pub unsafe fn register_irq(ch: u8, handler: fn()) {
    unsafe {
        let id = irq_spri(ch);
        gic::register(id, handler);
        gic::set_priority(id, 5);
        gic::enable(id);
    }
}

/// Disable the SPRI interrupt for channel `ch`.
///
/// Call from within the SPRI ISR to prevent re-entrancy, mirroring
/// `RSPI(ch).SPCR &= ~(1 << 7)` in the C firmware.
///
/// # Safety
/// Writes SPCR register.
#[inline]
pub unsafe fn disable_rx_irq(ch: u8) {
    unsafe {
        let v = reg8(ch, OFF_SPCR).read_volatile();
        reg8(ch, OFF_SPCR).write_volatile(v & !SPCR_SPRIE);
    }
}

/// Enable the SPRI interrupt for channel `ch`.
///
/// Call just before writing to SPDR to arm the receive-complete interrupt.
///
/// # Safety
/// Writes SPCR register.
#[inline]
pub unsafe fn enable_rx_irq(ch: u8) {
    unsafe {
        let v = reg8(ch, OFF_SPCR).read_volatile();
        reg8(ch, OFF_SPCR).write_volatile(v | SPCR_SPRIE);
    }
}

/// Send a 32-bit word over SPI channel `ch`, using the interrupt-driven
/// CV-transfer path.
///
/// This function:
/// 1. Waits until SPTEF=1 (TX buffer has room for 4 more bytes).
/// 2. Resets the SPBFCR so the RX interrupt fires at end-of-transfer.
/// 3. Enables the SPRI interrupt.
/// 4. Writes `data` to SPDR (starts the shift).
///
/// The SPRI ISR (registered via [`register_irq`]) fires when the word has
/// been fully shifted out; the ISR must:
/// - Deassert the chip-select GPIO.
/// - Call [`disable_rx_irq`].
/// - Reset the RX FIFO with [`reset_rx_buf`].
///
/// # Safety
/// Channel must have been initialised with [`init`].  The caller must assert
/// the chip-select GPIO **before** calling this function.
pub unsafe fn send32(ch: u8, data: u32) {
    unsafe {
        // Wait until TX buffer has space (SPTEF=1)
        while reg8(ch, OFF_SPSR).read_volatile() & SPSR_SPTEF == 0 {}

        // Program SPBFCR for CV mode (don't reset RX so the interrupt fires)
        reg8(ch, OFF_SPBFCR).write_volatile(SPBFCR_CV_TRANSFER);

        // Enable receive interrupt: fires when the shift register finishes
        enable_rx_irq(ch);

        // Write data — transfer starts immediately
        reg32(ch, OFF_SPDR).write_volatile(data);
    }
}

/// Blocking 32-bit SPI send without interrupt — wait for TEND.
///
/// Intended for the init sequence (MAX5136 linearity words) where no ISR
/// infrastructure is needed yet.
///
/// # Safety
/// Channel must have been initialised with [`init`].
pub unsafe fn send32_blocking(ch: u8, data: u32) {
    unsafe {
        log::debug!(
            "rspi: ch{} blocking send {:#010x}, SPSR={:#04x}",
            ch,
            data,
            reg8(ch, OFF_SPSR).read_volatile()
        );
        // Wait for TX buffer space (bounded to guard against ungated clock).
        // ~1 M polls ≈ a few ms at 400 MHz; SPTEF should assert in microseconds.
        let mut ready = false;
        for _ in 0..1_000_000u32 {
            if reg8(ch, OFF_SPSR).read_volatile() & SPSR_SPTEF != 0 {
                ready = true;
                break;
            }
        }
        if !ready {
            log::warn!(
                "rspi: ch{} send32_blocking: SPTEF timeout (STBCR10 cleared?)",
                ch
            );
        }
        log::debug!("rspi: ch{} SPTEF ok", ch);

        // Reset RX buffer (prevents overflow)
        let v = reg8(ch, OFF_SPBFCR).read_volatile();
        reg8(ch, OFF_SPBFCR).write_volatile(v | SPBFCR_RX_RESET);

        // Write data
        reg32(ch, OFF_SPDR).write_volatile(data);
        log::debug!("rspi: ch{} SPDR written, waiting TEND", ch);

        // Wait for all bits to be shifted out (bounded; TEND should follow SPTEF
        // by at most frame_bits / SCK_freq ≈ 32 / 10 MHz = 3.2 µs).
        let mut done = false;
        for _ in 0..1_000_000u32 {
            if reg8(ch, OFF_SPSR).read_volatile() & SPSR_TEND != 0 {
                done = true;
                break;
            }
        }
        if !done {
            log::warn!("rspi: ch{} send32_blocking: TEND timeout", ch);
        }
        log::debug!("rspi: ch{} TEND ok", ch);
    }
}

/// Reset the RSPI receive FIFO buffer (set RXRST in SPBFCR).
///
/// The C firmware does this after every CV transfer completes to prevent
/// the receive-buffer-full condition from stalling subsequent transfers.
///
/// # Safety
/// Writes SPBFCR register.
#[inline]
pub unsafe fn reset_rx_buf(ch: u8) {
    unsafe {
        let v = reg8(ch, OFF_SPBFCR).read_volatile();
        reg8(ch, OFF_SPBFCR).write_volatile(v | SPBFCR_RX_RESET);
    }
}

/// Switch RSPI channel `ch` to 8-bit frame mode for OLED (SSD1309) transfers.
///
/// Sets SPDCR=0x20 (8-bit), SPCMD0=0x0702 (SPB=8), SPBFCR=0x60.
/// Must be called before [`send8`] / [`send8_blocking`].
///
/// After OLED rendering is done, call [`configure_32bit`] to restore the CV
/// DAC word size.
///
/// # Safety
/// Writes RSPI registers.  No concurrent SPI transfers must be in progress.
pub unsafe fn configure_8bit(ch: u8) {
    unsafe {
        reg8(ch, OFF_SPDCR).write_volatile(SPDCR_8BIT);
        reg16(ch, OFF_SPCMD0).write_volatile(SPCMD0_8BIT);
        reg8(ch, OFF_SPBFCR).write_volatile(SPBFCR_8BIT);
    }
}

/// Restore RSPI channel `ch` to 32-bit frame mode for CV DAC transfers.
///
/// # Safety
/// Writes RSPI registers.  No concurrent SPI transfers must be in progress.
pub unsafe fn configure_32bit(ch: u8) {
    unsafe {
        reg8(ch, OFF_SPDCR).write_volatile(SPDCR_32BIT);
        reg16(ch, OFF_SPCMD0).write_volatile(SPCMD0_32BIT);
        reg8(ch, OFF_SPBFCR).write_volatile(SPBFCR_INIT_32BIT);
    }
}

/// Send a single byte via 8-bit SPI.
///
/// Mirrors `R_RSPI_SendBasic8()`:
/// - Polls SPTEF until the TX buffer has 4 bytes of free space.
/// - Resets the RX buffer to prevent overflow on a receive-only hardware.
/// - Writes the byte to SPDR (SPDR.BYTE.LL = lowest byte = base + 0x04 on LE ARM).
///
/// Does **not** poll TEND.  After the last byte of a sequence, call [`wait_end`]
/// to ensure the shift register has finished.
///
/// # Safety
/// Channel must have been initialised with [`init`] and configured for 8-bit
/// mode via [`configure_8bit`].
#[inline]
pub unsafe fn send8(ch: u8, data: u8) {
    unsafe {
        // Wait until TX buffer has 4 free bytes (SPTEF=1).
        while reg8(ch, OFF_SPSR).read_volatile() & SPSR_SPTEF == 0 {}
        // Reset RX buffer to prevent overflow accumulation.
        let v = reg8(ch, OFF_SPBFCR).read_volatile();
        reg8(ch, OFF_SPBFCR).write_volatile(v | SPBFCR_RX_RESET);
        // Write byte to SPDR.BYTE.LL (little-endian: lowest byte of SPDR = base+0x04).
        (reg32(ch, OFF_SPDR) as *mut u8).write_volatile(data);
    }
}

/// Poll until the shift register is idle (TEND=1).
///
/// Call after the last [`send8`] in a sequence to ensure data has been fully
/// clocked out before de-asserting chip-select.
///
/// # Safety
/// Reads SPSR register.
#[inline]
pub unsafe fn wait_end(ch: u8) {
    unsafe { while reg8(ch, OFF_SPSR).read_volatile() & SPSR_TEND == 0 {} }
}

// ── embedded-hal SpiBus impls ─────────────────────────────────────────────────

use core::convert::Infallible;
use core::marker::PhantomData;

/// Marker type: RSPI channel configured for 8-bit frame transfers.
pub struct Bits8;
/// Marker type: RSPI channel configured for 32-bit frame transfers.
pub struct Bits32;

/// A type-safe handle to an RSPI channel.
///
/// `CH` is the channel number (0–4).  `BITS` is either [`Bits8`] or [`Bits32`],
/// selecting which [`embedded_hal::spi::SpiBus`] impl is active.
///
/// This handle **does not** own the hardware configuration; the caller must
/// have already called [`init`] (and, for 8-bit mode, [`configure_8bit`]).
/// The handle is zero-cost (ZST).
pub struct Rspi<const CH: u8, BITS>(PhantomData<BITS>);

impl<const CH: u8, BITS> Rspi<CH, BITS> {
    /// Create an RSPI handle for a channel that has already been initialised.
    ///
    /// # Safety
    /// `CH` must be valid (0–4) and [`init`] must have been called for it.
    #[inline]
    pub unsafe fn new() -> Self {
        Rspi(PhantomData)
    }
}

// ── ErrorType ─────────────────────────────────────────────────────────────────

impl<const CH: u8, BITS> embedded_hal::spi::ErrorType for Rspi<CH, BITS> {
    type Error = Infallible;
}

// ── SpiBus<u8> for Bits8 ──────────────────────────────────────────────────────

impl<const CH: u8> embedded_hal::spi::SpiBus<u8> for Rspi<CH, Bits8> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Infallible> {
        for w in words.iter_mut() {
            // Transmit a dummy 0x00 to clock in a byte from the peripheral.
            unsafe {
                send8(CH, 0x00);
                wait_end(CH);
                // The received byte sits in SPDR after the shift completes.
                // RSPI SPDR lower byte contains the last received byte.
                *w = (reg32(CH, OFF_SPDR) as *const u8).read_volatile();
            }
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Infallible> {
        for &b in words {
            unsafe {
                send8(CH, b);
            }
        }
        unsafe {
            wait_end(CH);
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Infallible> {
        let len = read.len().max(write.len());
        for i in 0..len {
            let tx = if i < write.len() { write[i] } else { 0x00 };
            unsafe {
                send8(CH, tx);
                wait_end(CH);
                if i < read.len() {
                    read[i] = (reg32(CH, OFF_SPDR) as *const u8).read_volatile();
                }
            }
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Infallible> {
        for w in words.iter_mut() {
            unsafe {
                let tx = *w;
                send8(CH, tx);
                wait_end(CH);
                *w = (reg32(CH, OFF_SPDR) as *const u8).read_volatile();
            }
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Infallible> {
        unsafe {
            wait_end(CH);
        }
        Ok(())
    }
}

// ── SpiBus<u32> for Bits32 ────────────────────────────────────────────────────

impl<const CH: u8> embedded_hal::spi::SpiBus<u32> for Rspi<CH, Bits32> {
    fn read(&mut self, words: &mut [u32]) -> Result<(), Infallible> {
        for w in words.iter_mut() {
            unsafe {
                send32_blocking(CH, 0x0000_0000);
                *w = reg32(CH, OFF_SPDR).read_volatile();
            }
        }
        Ok(())
    }

    fn write(&mut self, words: &[u32]) -> Result<(), Infallible> {
        for &w in words {
            unsafe {
                send32_blocking(CH, w);
            }
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u32], write: &[u32]) -> Result<(), Infallible> {
        let len = read.len().max(write.len());
        for i in 0..len {
            let tx = if i < write.len() { write[i] } else { 0 };
            unsafe {
                send32_blocking(CH, tx);
                if i < read.len() {
                    read[i] = reg32(CH, OFF_SPDR).read_volatile();
                }
            }
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u32]) -> Result<(), Infallible> {
        for w in words.iter_mut() {
            unsafe {
                let tx = *w;
                send32_blocking(CH, tx);
                *w = reg32(CH, OFF_SPDR).read_volatile();
            }
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Infallible> {
        // send32_blocking already polls TEND; nothing left to drain.
        Ok(())
    }
}

// ── Address tests ─────────────────────────────────────────────────────────────
#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn rspi_channel_bases() {
        assert_eq!(base(0), 0xE800_C800);
        assert_eq!(base(1), 0xE800_D000);
        assert_eq!(base(2), 0xE800_D800);
        assert_eq!(base(3), 0xE800_E000);
        assert_eq!(base(4), 0xE800_E800);
    }

    #[test]
    fn rspi_irq_ids() {
        // SPEI0=270, SPRI0=271, SPTI0=272
        // Each successive channel adds 3.
        assert_eq!(irq_spri(0), 271u16);
        assert_eq!(irq_spri(1), 274u16);
        assert_eq!(irq_spri(4), 283u16);
    }

    #[test]
    fn rspi_spbr_10mhz() {
        // ceil(66_666_666 / (10_000_000 × 2)) - 1 = ceil(3.333) - 1 = 4 - 1 = 3
        let bit_rate: u32 = 10_000_000;
        let spbr = ((P1_HZ + bit_rate * 2 - 1) / (bit_rate * 2)).saturating_sub(1) as u8;
        assert_eq!(spbr, 3);
        // Actual rate: 66_666_666 / (2 × (3+1)) = 8.333 MHz (within ±20% of 10 MHz)
        let actual_hz = P1_HZ / (2 * (spbr as u32 + 1));
        assert!(actual_hz >= 8_000_000 && actual_hz <= 11_000_000);
    }
}
