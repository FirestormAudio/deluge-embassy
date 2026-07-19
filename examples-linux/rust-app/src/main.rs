//! The Rust analogue of `examples/app` (the C worked example): the smallest
//! thing that proves the pipeline. Open the device, blink, exit. `Deluge`'s
//! `Drop` impl calls `deluge_close` for us.

fn main() {
    let mut d = deluge_hal_linux::Deluge::open().expect("deluge_open failed");

    for i in 0..8 {
        d.leds_indicator(0, i % 2 == 0).expect("leds unavailable");
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}
