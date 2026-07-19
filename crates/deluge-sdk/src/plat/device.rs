//! Device backend ops (deluge-bsp peripherals). Bodies moved verbatim from the
//! capability modules' `#[cfg(target_os = "none")]` arms.
use core::convert::Infallible;

use deluge_bsp::oled::{self, FrameBuffer};
use deluge_bsp::pic;
use deluge_bsp::rgb::PadLeds;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use rza1l_hal::gpio::{Output, Pin};

/// The SYNC LED is wired to port 6, pin 7.
type SyncLedPin = Pin<6, 7, Output>;

pub(crate) async fn oled_init_panel() {
    oled::init().await;
}
pub(crate) async fn oled_flush(fb: &FrameBuffer) {
    oled::send_frame(fb).await;
}

pub(crate) async fn pads_flush(leds: &mut PadLeds) {
    leds.flush().await;
}
pub(crate) async fn pads_set_brightness_interval(interval: u8) {
    deluge_bsp::pic::set_refresh_time(interval).await;
}

pub(crate) async fn leds_set(id: u8, on: bool) {
    if on {
        pic::led_on(id).await;
    } else {
        pic::led_off(id).await;
    }
}
pub(crate) async fn leds_clear() {
    for id in 0..crate::leds::Leds::NUM_INDICATOR_LEDS {
        pic::led_off(id).await;
    }
}
pub(crate) async fn leds_gold_knob(knob: u8, brightness: [u8; 4]) {
    pic::set_gold_knob_indicators(knob, brightness).await;
}

pub(crate) fn sync_led_init() -> SyncLedPin {
    // SAFETY: the take-once guard in `Deluge::sync_led` ensures this runs once
    // and nothing else owns P6_7; clocks are up by the time an app runs.
    unsafe { SyncLedPin::into_output() }
}
pub(crate) fn sync_led_set_high(pin: &mut SyncLedPin) -> Result<(), Infallible> {
    pin.set_high()
}
pub(crate) fn sync_led_set_low(pin: &mut SyncLedPin) -> Result<(), Infallible> {
    pin.set_low()
}
pub(crate) fn sync_led_toggle(pin: &mut SyncLedPin) -> Result<(), Infallible> {
    StatefulOutputPin::toggle(pin)
}
pub(crate) fn sync_led_is_set_high(pin: &mut SyncLedPin) -> Result<bool, Infallible> {
    pin.is_set_high()
}
pub(crate) fn sync_led_is_set_low(pin: &mut SyncLedPin) -> Result<bool, Infallible> {
    pin.is_set_low()
}

pub(crate) fn cv_gate_init() {
    // SAFETY: runs once. Configures GPIO + RSPI0 and runs the DAC's ~10 ms
    // linearity init (poll-based delays). Acquire CV/gate before entering a
    // loop that also drives the OLED, so this one-time RSPI0 setup can't race an
    // in-flight OLED transfer (see docs/advanced-guide.md §7).
    unsafe { deluge_bsp::cv_gate::init() };
}
pub(crate) async fn cv_set(ch: u8, code: u16) {
    deluge_bsp::cv_gate::cv_set(ch, code).await;
}
pub(crate) fn gate_set(ch: u8, on: bool) {
    // SAFETY: GPIO write to a gate line we own; pins configured by init.
    unsafe { deluge_bsp::cv_gate::gate_set(ch, on) };
}
