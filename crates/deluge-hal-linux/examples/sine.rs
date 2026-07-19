fn main() {
    let mut d = deluge_linux::Deluge::open().expect("open");
    let mut phase = 0f32;
    d.audio_start(move |_in, out| {
        for f in out.iter_mut() {
            let s = 0.2 * phase.sin();
            phase += 2.0 * std::f32::consts::PI * 440.0 / 44100.0;
            if phase > 2.0 * std::f32::consts::PI { phase -= 2.0 * std::f32::consts::PI; }
            *f = [s, s];
        }
    }).expect("audio");
    std::thread::sleep(std::time::Duration::from_secs(3));
}
