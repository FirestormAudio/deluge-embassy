fn main() {
    let mut d = deluge_linux::Deluge::open().expect("open");
    match d.usb_role_get() {
        Ok(r) => println!("role: {:?}", r),
        Err(e) => println!("usb unavailable: {}", e),
    }
}
