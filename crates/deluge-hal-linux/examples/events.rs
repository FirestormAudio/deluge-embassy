fn main() {
    let mut d = deluge_hal_linux::Deluge::open().expect("open");
    d.input_start(|e| println!("{:?}", e)).expect("input");
    std::thread::sleep(std::time::Duration::from_secs(30));
}
