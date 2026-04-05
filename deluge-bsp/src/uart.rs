//! Deluge board UART assignments.
//!
//! Maps the two SCIF channels used on the Deluge to their physical pins and
//! gives them meaningful names, keeping [`rza1l_hal::uart`] itself board-agnostic.
//!
//! | Channel | Signal  | TxD pin  | mux | RxD pin | mux |
//! |---------|---------|----------|-----|---------|-----|
//! | SCIF0   | MIDI DIN| P6_15    |  5  | P6_14   |  5  |
//! | SCIF1   | PIC32   | P3_15    |  5  | P1_9    |  3  |

/// SCIF channel used for MIDI DIN (31250 bps).
pub const MIDI_CH: usize = 0;

/// SCIF channel used for the PIC32 co-processor.
pub const PIC_CH: usize = 1;

pub use rza1l_hal::uart::{read_byte, write_bytes};

/// Write bytes to the MIDI DIN UART (SCIF0 @ 31 250 bps).
#[inline]
pub async fn write_midi(data: &[u8]) {
    rza1l_hal::uart::write_bytes(MIDI_CH, data).await;
}

/// Read one byte from the MIDI DIN UART (SCIF0 DMA RX ring).
#[inline]
pub async fn read_midi_byte() -> u8 {
    rza1l_hal::uart::read_byte(MIDI_CH).await
}

/// Initialise the MIDI DIN UART (SCIF0) with DMA-backed RX.
///
/// MIDI TX uses TXI GIC interrupts; SCIF0 RX uses DMAC channel
/// [`crate::system::MIDI_DMA_RX_CH`] so that incoming bytes are captured
/// without interrupt gaps.
///
/// DMARS = 0x62 (SCIF0 RX resource selector).
///
/// # Safety
/// Must be called with global IRQs disabled, before the Embassy executor starts.
pub unsafe fn init_midi(baud_rate: u32) {
    unsafe {
        log::debug!(
            "bsp uart: MIDI SCIF{} @ {} bps (DMA RX ch{})",
            MIDI_CH,
            baud_rate,
            crate::system::MIDI_DMA_RX_CH,
        );
        rza1l_hal::uart::init(MIDI_CH, baud_rate);
        rza1l_hal::gpio::set_pin_mux(6, 15, 5); // TxD0
        rza1l_hal::gpio::set_pin_mux(6, 14, 5); // RxD0
        // TX via TXI interrupt; RX via DMAC channel (crate::system::MIDI_DMA_RX_CH).
        rza1l_hal::uart::register_txi_for(MIDI_CH);
        rza1l_hal::uart::init_dma_rx(MIDI_CH, crate::system::MIDI_DMA_RX_CH, 0x62);
    }
}

/// Initialise the PIC32 UART (SCIF1) with DMA-backed RX and TX.
///
/// SCIF1 RX uses DMAC channel [`crate::system::PIC_DMA_RX_CH`] and TX uses
/// DMAC channel [`crate::system::PIC_DMA_TX_CH`].
///
/// DMARS values: RX = 0x66, TX = 0x65 (SCIF1 resource selectors).
///
/// # Safety
/// Must be called with global IRQs disabled, before the Embassy executor starts.
pub unsafe fn init_pic(baud_rate: u32) {
    unsafe {
        log::debug!(
            "bsp uart: PIC SCIF{} @ {} bps (DMA RX ch{}, DMA TX ch{})",
            PIC_CH,
            baud_rate,
            crate::system::PIC_DMA_RX_CH,
            crate::system::PIC_DMA_TX_CH,
        );
        rza1l_hal::uart::init(PIC_CH, baud_rate);
        rza1l_hal::gpio::set_pin_mux(3, 15, 5); // TxD1
        rza1l_hal::gpio::set_pin_mux(1, 9, 3); // RxD1
        // RX via DMAC channel (crate::system::PIC_DMA_RX_CH); TX via (crate::system::PIC_DMA_TX_CH).
        rza1l_hal::uart::register_txi_for(PIC_CH);
        rza1l_hal::uart::init_dma_rx(PIC_CH, crate::system::PIC_DMA_RX_CH, 0x66);
        rza1l_hal::uart::init_dma_tx(PIC_CH, crate::system::PIC_DMA_TX_CH, 0x65);
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::{MIDI_CH, PIC_CH};

    #[test]
    fn midi_channel_is_scif0() {
        assert_eq!(MIDI_CH, 0);
    }

    #[test]
    fn pic_channel_is_scif1() {
        assert_eq!(PIC_CH, 1);
    }

    /// Channels must be distinct so both UARTs can operate concurrently.
    #[test]
    fn channels_are_distinct() {
        assert_ne!(MIDI_CH, PIC_CH);
    }

    /// Both channels must be within the RZ/A1L SCIF range (0–4).
    #[test]
    fn channels_in_range() {
        assert!(MIDI_CH < rza1l_hal::uart::NUM_CHANNELS);
        assert!(PIC_CH < rza1l_hal::uart::NUM_CHANNELS);
    }
}
