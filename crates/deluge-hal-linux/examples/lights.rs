fn main() {
    let mut d = deluge_hal_linux::Deluge::open().expect("open");
    let _ = d.leds_indicator(0, true);
    let _ = d.oled_write(&[0x55u8; 688]);
    let _ = d.pads_write(&[10u8; 432]);
    std::thread::sleep(std::time::Duration::from_secs(2));
}
