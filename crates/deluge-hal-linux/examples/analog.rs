fn main() {
    let mut d = deluge_linux::Deluge::open().expect("open");
    let _ = d.cv_set_raw(0, 32768);
    let _ = d.gate_set(0, true);
    if let Some(buf) = d.sram() { buf[0] = 1; println!("sram {} bytes", buf.len()); }
}
