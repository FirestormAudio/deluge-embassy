use core::sync::atomic::{AtomicBool, Ordering};

use embassy_time::Timer;
use log::debug;

static HEARTBEAT_ENABLED: AtomicBool = AtomicBool::new(true);

pub(crate) fn toggle_heartbeat_enabled() -> bool {
    let enabled = !HEARTBEAT_ENABLED.load(Ordering::Relaxed);
    HEARTBEAT_ENABLED.store(enabled, Ordering::Relaxed);
    enabled
}

/// Heartbeat blink — P6_7 plays a double-pulse heartbeat pattern.
#[embassy_executor::task]
pub(crate) async fn blink_task() {
    debug!("blink_task: started");
    loop {
        if !HEARTBEAT_ENABLED.load(Ordering::Relaxed) {
            unsafe { rza1l_hal::gpio::write(6, 7, false) };
            Timer::after_millis(100).await;
            continue;
        }

        unsafe { rza1l_hal::gpio::write(6, 7, true) };
        Timer::after_millis(80).await;

        unsafe { rza1l_hal::gpio::write(6, 7, false) };
        Timer::after_millis(120).await;

        unsafe { rza1l_hal::gpio::write(6, 7, true) };
        Timer::after_millis(80).await;

        unsafe { rza1l_hal::gpio::write(6, 7, false) };
        Timer::after_millis(720).await;
    }
}
