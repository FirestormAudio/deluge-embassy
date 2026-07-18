use embassy_time::Timer;

/// Placeholder; the capture->playback loop is implemented in Task 3.
#[embassy_executor::task]
pub(crate) async fn loopback_task() {
    loop {
        Timer::after_millis(1000).await;
    }
}
