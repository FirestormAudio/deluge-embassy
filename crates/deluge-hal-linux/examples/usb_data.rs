fn main() {
    let mut d = deluge_linux::Deluge::open().expect("open");
    match d.usb_audio_card_id() {
        Some(id) => match d.usb_audio_probe() {
            Ok(i) => println!("usb audio: {id} {:?}", i),
            Err(e) => println!("usb audio: {id} (probe failed: {e})"),
        },
        None => println!("usb audio: absent"),
    }
    let mut buf = [0u8; 64];
    match d.usb_midi_read(&mut buf) {
        Ok(n) => println!("usb midi: read {n} bytes"),
        Err(e) => println!("usb midi: absent ({e})"),
    }
}
