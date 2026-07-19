//! Host-simulator backend ops (the deluge-sim-link SharedPanel). Bodies moved
//! verbatim from the capability modules' `#[cfg(not(target_os = "none"))]` arms.
use deluge_bsp::oled::FrameBuffer;

pub(crate) async fn oled_init_panel() {}
pub(crate) async fn oled_flush(fb: &FrameBuffer) {
    crate::host::panel().set_display(fb.as_bytes());
}
