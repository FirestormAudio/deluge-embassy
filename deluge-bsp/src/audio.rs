//! Deluge board audio (SSI0 / SSIF0) initialisation.
//!
//! ## Pin assignments
//!
//! | Port.Pin | Mux | Signal       | Direction |
//! |----------|-----|--------------|-----------|
//! | P7.11    | 6   | AUDIO_XOUT   | Output    |
//! | P6.8     | 3   | SSITXD0      | Output    |
//! | P6.9     | 3   | SSISCK0      | Output    |
//! | P6.10    | 3   | SSIWS0       | Output    |
//! | P6.11    | 3   | SSIRXD0      | Input     |
//! | P6.12    | —   | CODEC_POWER  | GPIO out  |
//!
//! `CODEC_POWER` (P6.12) is driven low during SSI initialisation and raised
//! high after the codec's required 5 ms clock-stable delay, following the
//! sequence used by the original C firmware.
//!
//! The SCUX DVU block sits between the CPU audio buffer and the SSIF0
//! transmitter, providing hardware volume control and click-free fades.
//! SSI0 TX DMA (ch 6) is intentionally **not** started; SCUX drives SSIF0 TX
//! directly via `SSICTRL.SSI012TEN`.  SSI0 RX DMA (ch 7) is started as usual.

use crate::scux_dvu_path;
use rza1l_hal::{gpio, ostm};

/// Port and pin of the codec hardware-enable line.
const CODEC_PORT: u8 = 6;
const CODEC_PIN: u8 = 12;

/// Delay required between SSI clock start and codec enable, in milliseconds.
const CODEC_POWER_DELAY_MS: u32 = 5;

/// Configure SSI0 pin-mux, initialise the SSI peripheral with DMA, wait for
/// the codec power-on delay, then assert `CODEC_POWER`.
///
/// **Call after:** `rza1l_hal::stb::init()` and `rza1l_hal::ostm::start_free_running(0)`.
///
/// # Safety
/// Must be called exactly once from the single-threaded boot context, before
/// interrupts are enabled and before any audio task starts.
pub unsafe fn init() {
    unsafe {
        log::debug!("audio: pin-mux SSI0 (P7.11, P6.8-11)");
        // ── Pin-mux: SSI peripheral signals ──────────────────────────────────────
        //
        // set_pin_mux sets PMC=1 (peripheral mode), the three mux-select bits
        // (PFC/PFCE/PFCAE), and PIPC=1 so the peripheral controls the buffer
        // direction automatically.
        gpio::set_pin_mux(7, 11, 6); // AUDIO_XOUT — master clock to codec
        gpio::set_pin_mux(6, 8, 3); // SSITXD0    — I²S TX data
        gpio::set_pin_mux(6, 9, 3); // SSISCK0    — I²S BCLK
        gpio::set_pin_mux(6, 10, 3); // SSIWS0     — I²S LRCK
        gpio::set_pin_mux(6, 11, 3); // SSIRXD0    — I²S RX data (input)

        // ── CODEC_POWER GPIO: hold low while SSI clocks stabilise ────────────────
        gpio::set_as_output(CODEC_PORT, CODEC_PIN);
        gpio::write(CODEC_PORT, CODEC_PIN, false);

        // ── Initialise SCUX DVU path: SSI RX DMA + SCUX → SSIF0 TX ──────────────
        scux_dvu_path::init();

        // ── 5 ms clock-stable delay before enabling the codec ────────────────────
        // OSTM0 must already be running in free-running mode.
        ostm::delay_ms(CODEC_POWER_DELAY_MS);
        log::debug!("audio: codec power-on delay done");
        // ── Assert CODEC_POWER — codec begins normal operation ───────────────────
        gpio::write(CODEC_PORT, CODEC_PIN, true);
        log::debug!("audio: CODEC_POWER asserted, codec active");
    }
}
