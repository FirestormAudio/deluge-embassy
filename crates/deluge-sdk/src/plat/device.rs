//! Device backend ops (deluge-bsp peripherals). Bodies moved verbatim from the
//! capability modules' `#[cfg(target_os = "none")]` arms.
use deluge_bsp::oled::{self, FrameBuffer};

pub(crate) async fn oled_init_panel() {
    oled::init().await;
}
pub(crate) async fn oled_flush(fb: &FrameBuffer) {
    oled::send_frame(fb).await;
}
