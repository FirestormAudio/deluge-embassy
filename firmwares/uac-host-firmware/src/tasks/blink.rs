use embassy_time::Timer;
use log::debug;

/// Heartbeat blink — P6_7 plays a double-pulse heartbeat pattern so it is
/// visually obvious that the firmware is alive even when idle.
#[embassy_executor::task]
pub(crate) async fn blink_task() {
    debug!("blink_task: started");
    loop {
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
