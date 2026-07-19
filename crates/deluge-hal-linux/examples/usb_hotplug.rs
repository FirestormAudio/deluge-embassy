fn main() {
    let mut d = deluge_linux::Deluge::open().expect("open");
    println!("usb hotplug fd: {}", d.usb_hotplug_fd());
    // NOTE: the callback is moved into the SDK thread and can't borrow `d`, so
    // this example just logs; a real consumer routes AUDIO/MIDI events to
    // d.usb_audio_rebind()/d.usb_midi_rebind() from its own loop via usb_hotplug_fd().
    match d.usb_watch(|ev| println!("{:?} {:?} {}", ev.action, ev.class, ev.devnode)) {
        Ok(()) => {
            println!("watching 10s; on AUDIO/MIDI events a consumer calls usb_audio_rebind/usb_midi_rebind...");
            std::thread::sleep(std::time::Duration::from_secs(10));
            d.usb_unwatch();
            // demonstrate the rebind calls resolve (NODEV when nothing plugged):
            println!("audio rebind: {:?}", d.usb_audio_rebind());
            println!("midi rebind:  {:?}", d.usb_midi_rebind());
        }
        Err(e) => println!("usb_watch unavailable: {e}"),
    }
}
