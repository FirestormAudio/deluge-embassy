//! Device backend ops (deluge-bsp peripherals). Bodies moved verbatim from the
//! capability modules' `#[cfg(target_os = "none")]` arms.
use deluge_bsp::oled::{self, FrameBuffer};
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
