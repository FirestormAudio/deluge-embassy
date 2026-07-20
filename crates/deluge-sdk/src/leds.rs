//! Button/indicator LEDs and the gold-knob LED columns (driven by the PIC).

/// The button/indicator LEDs and gold-knob columns, taken once from
/// [`Deluge::leds`](crate::Deluge::leds).
///
/// LED ids line up with button ids, so lighting the LED under a pressed button
/// is `leds.set(id, true)`. Names are in [`controls::button`](crate::controls::button).
pub struct Leds {
    _not_send: crate::NotSend,
}

impl Leds {
    /// Number of indicator LEDs (one per button; LED `id` matches the
    /// [`Event::Button`](crate::Event::Button) / [`controls::button`](crate::controls::button) id).
    pub const NUM_INDICATOR_LEDS: u8 = 36;
    /// Number of gold-knob LED columns (each a vertical strip of 4 LEDs).
    pub const NUM_GOLD_KNOBS: u8 = 2;

    pub(crate) fn new() -> Self {
        Self {
            _not_send: crate::NOT_SEND,
        }
    }

    /// Set indicator LED `id` (0–35) on or off.
    #[inline]
    pub async fn set(&mut self, id: u8, on: bool) {
        crate::plat::leds_set(id, on).await;
    }

    /// Turn indicator LED `id` on.
    #[inline]
    pub async fn on(&mut self, id: u8) {
        self.set(id, true).await;
    }

    /// Turn indicator LED `id` off.
    #[inline]
    pub async fn off(&mut self, id: u8) {
        self.set(id, false).await;
    }

    /// Turn all indicator LEDs off.
    pub async fn clear(&mut self) {
        crate::plat::leds_clear().await;
    }

    /// Set a gold-knob LED column. `knob` is 0 or 1; `brightness` is the four
    /// LEDs in the column, bottom→top (0–255 each).
    #[inline]
    pub async fn gold_knob(&mut self, knob: u8, brightness: [u8; 4]) {
        crate::plat::leds_gold_knob(knob, brightness).await;
    }
}
