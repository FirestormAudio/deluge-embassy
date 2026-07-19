//! The Deluge SYNC LED.

use embedded_hal::digital::{OutputPin, StatefulOutputPin};
#[cfg(target_os = "none")]
use rza1l_hal::gpio::{Output, Pin};

/// The SYNC LED is wired to port 6, pin 7.
#[cfg(target_os = "none")]
type SyncLedPin = Pin<6, 7, Output>;

/// The Deluge SYNC LED (P6_7).
///
/// Obtained once from [`Deluge::sync_led`](crate::Deluge::sync_led). Owning the
/// `SyncLed` means the compiler keeps a single place in charge of it — the GPIO
/// analogue of the bus ownership the BSP enforces for shared peripherals.
///
/// Beyond the convenience methods, `SyncLed` is an `embedded-hal`
/// [`OutputPin`]/[`StatefulOutputPin`], and [`SyncLed::pin`] hands the underlying
/// pin to driver crates that expect one.
#[cfg(target_os = "none")]
pub struct SyncLed {
    pin: SyncLedPin,
}

#[cfg(target_os = "none")]
impl SyncLed {
    /// Configure P6_7 as an output and return the handle.
    ///
    /// Internal — apps obtain the LED via
    /// [`Deluge::sync_led`](crate::Deluge::sync_led), whose take-once guard
    /// guarantees single ownership.
    pub(crate) fn new() -> Self {
        SyncLed {
            pin: crate::plat::sync_led_init(),
        }
    }

    /// Turn the LED on.
    #[inline]
    pub fn on(&mut self) {
        let _ = crate::plat::sync_led_set_high(&mut self.pin);
    }

    /// Turn the LED off.
    #[inline]
    pub fn off(&mut self) {
        let _ = crate::plat::sync_led_set_low(&mut self.pin);
    }

    /// Set the LED on (`true`) or off (`false`).
    #[inline]
    pub fn set(&mut self, on: bool) {
        if on {
            self.on();
        } else {
            self.off();
        }
    }

    /// Toggle the LED.
    #[inline]
    pub fn toggle(&mut self) {
        let _ = crate::plat::sync_led_toggle(&mut self.pin);
    }

    /// Borrow the underlying `embedded-hal` output pin, for driver crates that
    /// take an [`OutputPin`].
    #[inline]
    pub fn pin(&mut self) -> &mut impl OutputPin {
        &mut self.pin
    }
}

#[cfg(target_os = "none")]
impl OutputPin for SyncLed {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        crate::plat::sync_led_set_high(&mut self.pin)
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        crate::plat::sync_led_set_low(&mut self.pin)
    }
}

#[cfg(target_os = "none")]
impl StatefulOutputPin for SyncLed {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        crate::plat::sync_led_is_set_high(&mut self.pin)
    }
    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        crate::plat::sync_led_is_set_low(&mut self.pin)
    }
}

// ── Host (desktop simulator) ─────────────────────────────────────────────────

/// The Deluge SYNC LED, backed by the shared panel on the host simulator.
#[cfg(not(target_os = "none"))]
pub struct SyncLed {
    state: bool,
}

#[cfg(not(target_os = "none"))]
impl SyncLed {
    pub(crate) fn new() -> Self {
        SyncLed {
            state: crate::plat::sync_led_init(),
        }
    }

    /// Turn the LED on.
    #[inline]
    pub fn on(&mut self) {
        self.set(true);
    }

    /// Turn the LED off.
    #[inline]
    pub fn off(&mut self) {
        self.set(false);
    }

    /// Set the LED on (`true`) or off (`false`).
    #[inline]
    pub fn set(&mut self, on: bool) {
        self.state = on;
        crate::plat::sync_led_set(on);
    }

    /// Toggle the LED.
    #[inline]
    pub fn toggle(&mut self) {
        self.set(!self.state);
    }

    /// Borrow the underlying `embedded-hal` output pin.
    #[inline]
    pub fn pin(&mut self) -> &mut impl OutputPin {
        self
    }
}

#[cfg(not(target_os = "none"))]
impl OutputPin for SyncLed {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set(true);
        Ok(())
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set(false);
        Ok(())
    }
}

#[cfg(not(target_os = "none"))]
impl StatefulOutputPin for SyncLed {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(crate::plat::sync_led_is_set_high(self.state))
    }
    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(crate::plat::sync_led_is_set_low(self.state))
    }
}

// `SyncLed` stands in wherever an `embedded-hal` output pin is expected.
impl embedded_hal::digital::ErrorType for SyncLed {
    type Error = core::convert::Infallible;
}
