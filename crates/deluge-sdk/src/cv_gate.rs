//! CV and gate outputs.

use core::sync::atomic::{AtomicBool, Ordering};

/// Bring up the CV DAC + gate GPIOs once (RSPI0 wiring, gate lines, MAX5136
/// linearity init, CV zeroed). Shared by [`Cv`], [`Gate`], and
/// [`ClockOut`](crate::ClockOut) (which pulses a gate line).
pub(crate) fn ensure_init() {
    static DONE: AtomicBool = AtomicBool::new(false);
    if DONE.swap(true, Ordering::Relaxed) {
        return;
    }
    crate::plat::cv_gate_init();
}

/// The CV (control-voltage) outputs — a MAX5136 16-bit DAC.
///
/// Taken once from [`Deluge::cv`](crate::Deluge::cv). Values are 16-bit DAC
/// codes (~6552 counts per volt); [`set_volts`](Cv::set_volts) is the convenient
/// form. Writes go over the shared, arbitrated RSPI0 bus.
pub struct Cv {
    _private: (),
}

impl Cv {
    /// Number of CV channels (0–1 on the Deluge).
    pub const CHANNELS: usize = deluge_bsp::cv_gate::NUM_CV_CHANNELS;

    pub(crate) fn new() -> Self {
        ensure_init();
        Self { _private: () }
    }

    /// Write a raw 16-bit DAC code to channel `ch`. On the host simulator the
    /// value is recorded in the shared panel (not yet rendered).
    #[inline]
    pub async fn set(&mut self, ch: u8, code: u16) {
        crate::plat::cv_set(ch, code).await;
    }

    /// Write a voltage to channel `ch` (~6552 counts/V, clamped to 0–full scale).
    #[inline]
    pub async fn set_volts(&mut self, ch: u8, volts: f32) {
        let code = (volts * 6552.0).clamp(0.0, 65535.0) as u16;
        self.set(ch, code).await;
    }
}

/// The gate outputs — direct GPIO V-trig lines.
///
/// Taken once from [`Deluge::gate`](crate::Deluge::gate).
pub struct Gate {
    _private: (),
}

impl Gate {
    /// Number of gate channels (0–3 on the Deluge).
    pub const CHANNELS: usize = deluge_bsp::cv_gate::NUM_GATE_CHANNELS;

    pub(crate) fn new() -> Self {
        ensure_init();
        Self { _private: () }
    }

    /// Assert (`true`) or release (`false`) gate channel `ch`.
    #[inline]
    pub fn set(&mut self, ch: u8, on: bool) {
        crate::plat::gate_set(ch, on);
    }
}
