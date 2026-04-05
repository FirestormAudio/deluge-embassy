//! Deluge CV/gate interface initialisation and control.
//!
//! ## Hardware
//! CV outputs use a **MAX5136** quad 16-bit SPI DAC connected to RSPI0.
//! Gate outputs use direct GPIO V-trig lines (high = gate OFF, low = gate ON).
//!
//! ## SPI wiring (RSPI0)
//! | Port.Pin | Mux | Signal     |
//! |----------|-----|------------|
//! | P6.0     | 3   | RSPCLK0    |  SCK
//! | P6.2     | 3   | MOSI0      |  MOSI
//! | P6.1     | GPIO| SPI_SSL    |  active-low CS (software-driven)
//!
//! ## MAX5136 32-bit write frame
//! ```text
//! Byte 3: 0x30 | (1 << channel)   — write-to-DAC + channel bitmask
//! Byte 2: value[15:8]
//! Byte 1: value[7:0]
//! Byte 0: 0x00
//! ```
//! Linearity init: send `0x0542_0000` (LIN=1), delay ≥10 ms, send `0x0540_0000` (LIN=0).
//!
//! ## Gate GPIO assignments (V-trig: 0 = ON, 1 = OFF)
//! | Channel | Port | Pin |
//! |---------|------|-----|
//! | 0       | 2    | 7   |
//! | 1       | 2    | 8   |
//! | 2       | 2    | 9   |
//! | 3       | 4    | 0   |

use rza1l_hal::{gpio, ostm, rspi};

// ── Constants ─────────────────────────────────────────────────────────────────

/// RSPI channel shared between the CV DAC and the OLED display.
const SPI_CH: u8 = 0;

/// Target SCK frequency for the MAX5136 (10 MHz, within its 20 MHz maximum).
const SPI_RATE_HZ: u32 = 10_000_000;

/// MAX5136 linearity-init delay in milliseconds (datasheet §8.3).
const LIN_DELAY_MS: u32 = 10;

/// Number of hardware CV channels supported.
pub const NUM_CV_CHANNELS: usize = 2;

/// Number of hardware gate channels supported.
pub const NUM_GATE_CHANNELS: usize = 4;

// ── Pin definitions ───────────────────────────────────────────────────────────

/// SPI clock pin: P6.0, mux 3 → RSPCLK0
const SCK_PORT: u8 = 6;
const SCK_PIN: u8 = 0;
const SCK_MUX: u8 = 3;

/// SPI MOSI pin: P6.2, mux 3 → MOSI0
const MOSI_PORT: u8 = 6;
const MOSI_PIN: u8 = 2;
const MOSI_MUX: u8 = 3;

/// SPI chip-select GPIO: P6.1 (software-controlled, active-low)
const CS_PORT: u8 = 6;
const CS_PIN: u8 = 1;

/// Gate output pins.  Index matches the gate channel number.
/// V-trig: write `false` (0) to assert gate ON; write `true` (1) for gate OFF.
const GATE_PINS: [(u8, u8); NUM_GATE_CHANNELS] = [(2, 7), (2, 8), (2, 9), (4, 0)];

// ── MAX5136 command helpers ───────────────────────────────────────────────────

/// Build a 32-bit write-to-DAC word for the MAX5136.
///
/// - `ch`: DAC channel (0–3).
/// - `value`: unsigned 16-bit voltage code (0 = 0 V, 65535 = full-scale).
///   Approximately 6552 counts per volt on the Deluge hardware.
#[inline]
fn dac_word(ch: u8, value: u16) -> u32 {
    ((0x30u32 | (1u32 << ch)) << 24) | ((value as u32) << 8)
}

// ── Initialise hardware ───────────────────────────────────────────────────────

/// Initialise RSPI0 and the CV DAC + gate GPIO lines.
///
/// **Must be called after** `rza1l_hal::stb::init()` (enables RSPI clocks) and
/// `rza1l_hal::ostm::start_free_running(0)` (required for `delay_ms`).
///
/// After this function returns:
/// - All gate outputs are de-asserted (gate OFF).
/// - The MAX5136 has been through its linearity initialisation sequence.
/// - CV outputs are at 0 V.
/// - RSPI0 is ready to accept transfers via [`cv_set`].
///
/// # Safety
/// Writes to memory-mapped GPIO and RSPI registers.
pub unsafe fn init() {
    unsafe {
        log::debug!("cv_gate: init");
        // ---- SPI pin-mux ---------------------------------------------------------
        gpio::set_pin_mux(SCK_PORT, SCK_PIN, SCK_MUX);
        gpio::set_pin_mux(MOSI_PORT, MOSI_PIN, MOSI_MUX);
        log::debug!("cv_gate: pin-mux ok");

        // ---- Chip-select: output, start deselected (high) -----------------------
        gpio::set_as_output(CS_PORT, CS_PIN);
        gpio::write(CS_PORT, CS_PIN, true);

        // ---- Gate outputs: all de-asserted (V-trig: high = gate OFF) ------------
        for &(port, pin) in GATE_PINS.iter() {
            gpio::set_as_output(port, pin);
            gpio::write(port, pin, true);
        }
        log::debug!("cv_gate: gate GPIOs ok");

        // ---- Initialise RSPI0 as 10 MHz SPI master ------------------------------
        rspi::init(SPI_CH, SPI_RATE_HZ);
        log::debug!("cv_gate: RSPI0 init at {} Hz ok", SPI_RATE_HZ);

        // ---- MAX5136 linearity initialisation -----------------------------------
        // Step 1: LIN=1 (linear ramp mode)
        log::debug!("cv_gate: MAX5136 LIN=1");
        gpio::write(CS_PORT, CS_PIN, false);
        rspi::send32_blocking(SPI_CH, 0x0542_0000);
        gpio::write(CS_PORT, CS_PIN, true);
        log::debug!("cv_gate: MAX5136 LIN=1 sent");

        // Step 2: wait ≥ 10 ms for internal capacitor to charge
        log::debug!("cv_gate: LIN delay start");
        ostm::delay_ms(LIN_DELAY_MS);
        log::debug!("cv_gate: LIN delay done");

        // Step 3: LIN=0 (return to normal mode)
        gpio::write(CS_PORT, CS_PIN, false);
        rspi::send32_blocking(SPI_CH, 0x0540_0000);
        gpio::write(CS_PORT, CS_PIN, true);
        log::debug!("cv_gate: MAX5136 linearity init done");

        // ---- Zero all CV outputs ------------------------------------------------
        for ch in 0..NUM_CV_CHANNELS as u8 {
            cv_set_blocking(ch, 0);
        }
        log::debug!("cv_gate: CV outputs zeroed");
    }
}

// ── CV output ────────────────────────────────────────────────────────────────

/// Write a voltage code to CV channel `ch` (blocking, polls TEND).
///
/// - `ch`: 0 or 1 (hardware supports 0–1 on the Deluge).
/// - `value`: 16-bit DAC code.  ~6552 counts ≈ 1 V.
///
/// # Safety
/// Writes RSPI0 and CS GPIO registers.  Must not be called concurrently with
/// any other RSPI0 transfer.  Spins briefly if an OLED DMA transfer is in
/// progress (typically < 1 ms).
pub unsafe fn cv_set_blocking(ch: u8, value: u16) {
    unsafe {
        // Wait for any in-progress OLED DMA transfer to complete before
        // reconfiguring RSPI0 for 32-bit mode.  The OLED task sets this flag
        // around `dmac::start_transfer` … `dmac::wait_transfer_complete`.
        while crate::RSPI0_DMA_ACTIVE.load(core::sync::atomic::Ordering::Acquire) {
            core::hint::spin_loop();
        }
        let word = dac_word(ch, value);
        gpio::write(CS_PORT, CS_PIN, false);
        rspi::send32_blocking(SPI_CH, word);
        gpio::write(CS_PORT, CS_PIN, true);
    }
}

// ── Gate output ───────────────────────────────────────────────────────────────

/// Assert or de-assert gate output `ch`.
///
/// - `ch`: 0–3.
/// - `on`: `true` = gate active (low GPIO), `false` = gate inactive (high GPIO).
///
/// # Safety
/// Writes GPIO registers.
#[inline]
pub unsafe fn gate_set(ch: u8, on: bool) {
    unsafe {
        let (port, pin) = GATE_PINS[ch as usize];
        // V-trig: 0 = active, 1 = inactive  → invert `on`
        gpio::write(port, pin, !on);
    }
}

// ── Address / logic tests (host only) ────────────────────────────────────────

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn dac_word_channel_0_zero_volts() {
        // ch=0, value=0 → (0x30 | 0x01) << 24 | 0 = 0x3100_0000
        assert_eq!(dac_word(0, 0), 0x3100_0000);
    }

    #[test]
    fn dac_word_channel_1_full_scale() {
        // ch=1, value=0xFFFF → (0x30 | 0x02) << 24 | (0xFFFF << 8)
        // = 0x32_00_00_00 | 0x00_FF_FF_00
        // = 0x32FF_FF00
        assert_eq!(dac_word(1, 0xFFFF), 0x32FF_FF00);
    }

    #[test]
    fn dac_word_channel_2_one_volt() {
        // ~1 V ≈ 6552 = 0x1998
        // ch=2 → 0x30 | 0x04 = 0x34
        // word = 0x34_19_98_00
        let word = dac_word(2, 6552);
        assert_eq!(word >> 24, 0x34);
        assert_eq!((word >> 8) & 0xFFFF, 6552);
        assert_eq!(word & 0xFF, 0);
    }

    #[test]
    fn gate_channel_count() {
        assert_eq!(GATE_PINS.len(), NUM_GATE_CHANNELS);
    }
}
