//! Internal PIC co-processor service.
//!
//! The PIC32 is the Deluge's I/O co-processor: pads, buttons, indicator LEDs,
//! and the OLED chip-select handshake all flow over its UART. Several
//! capabilities ([`Oled`](crate::Oled), and later input/pads) depend on it, so
//! the SDK brings it up once, on demand, and runs a single RX pump that routes
//! incoming events.
//!
//! [`ensure_started`] is idempotent: the first capability that needs the PIC
//! initialises the UART and spawns the RX pump; later callers are no-ops.
//! Backend bodies live in [`crate::plat`].

use embassy_executor::Spawner;

/// Wait for the PIC to finish configuring. The host simulator has no PIC, so it
/// returns immediately.
pub(crate) async fn wait_ready() {
    crate::plat::pic_wait_ready().await;
}

/// Ensure the PIC UART is up and the RX pump is running. Idempotent.
///
/// Call from an async capability constructor before awaiting [`wait_ready`].
pub(crate) fn ensure_started(spawner: Spawner) {
    crate::plat::pic_ensure_started(spawner);
}
