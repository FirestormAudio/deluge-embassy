//! DIN MIDI input/output.

use core::sync::atomic::{AtomicBool, Ordering};

fn ensure_init() {
    static DONE: AtomicBool = AtomicBool::new(false);
    if DONE.swap(true, Ordering::Relaxed) {
        return;
    }
    crate::plat::midi_init();
}

/// The DIN MIDI port (SCIF0), taken once from [`Deluge::midi`](crate::Deluge::midi).
///
/// A raw byte stream in both directions — bring your own parser (a typed message
/// API may come later). RX is DMA-backed, so bytes are captured even while the
/// app is busy.
pub struct Midi {
    _private: (),
}

impl Midi {
    pub(crate) fn new() -> Self {
        ensure_init();
        Self { _private: () }
    }

    /// Send raw MIDI bytes. No-op on the host simulator (no DIN MIDI).
    #[inline]
    pub async fn send(&self, data: &[u8]) {
        crate::plat::midi_send(data).await;
    }

    /// Await the next received MIDI byte.
    #[inline]
    pub async fn recv(&self) -> u8 {
        crate::plat::midi_recv().await
    }

    /// Take the next received byte if one is buffered, without awaiting.
    #[inline]
    pub fn try_recv(&self) -> Option<u8> {
        crate::plat::midi_try_recv()
    }
}
