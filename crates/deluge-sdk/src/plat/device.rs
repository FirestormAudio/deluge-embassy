//! Device backend ops (deluge-bsp peripherals). Bodies moved verbatim from the
//! capability modules' `#[cfg(target_os = "none")]` arms.
use deluge_bsp::oled::{self, FrameBuffer};
use deluge_bsp::pic;
use deluge_bsp::rgb::PadLeds;

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
