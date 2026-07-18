use embassy_time::Timer;

/// Placeholder; the status display is implemented in Task 4.
#[embassy_executor::task]
pub(crate) async fn oled_task() {
    loop {
        Timer::after_millis(1000).await;
    }
}
