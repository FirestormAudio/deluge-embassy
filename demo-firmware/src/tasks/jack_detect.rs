use embassy_time::Timer;
use log::info;

/// Jack-detect and speaker-enable task.
///
/// Polls the five audio jack-detect GPIO inputs at the caller-provided
/// interval, logs any changes, and drives the `SPEAKER_ENABLE` output using a
/// firmware-owned policy: the speaker amplifier is enabled only when neither
/// headphone nor line-out is inserted.
#[embassy_executor::task]
pub(crate) async fn jack_detect_task(poll_interval_ms: u64) {
    unsafe {
        rza1l_hal::gpio::set_as_input(6, 5);
        rza1l_hal::gpio::set_as_input(6, 6);
        rza1l_hal::gpio::set_as_input(7, 9);
        rza1l_hal::gpio::set_as_input(6, 3);
        rza1l_hal::gpio::set_as_input(6, 4);
        rza1l_hal::gpio::set_as_output(4, 1);
        rza1l_hal::gpio::write(4, 1, false);
    }

    let mut prev_hp = false;
    let mut prev_li = false;
    let mut prev_mic = false;
    let mut prev_lol = false;
    let mut prev_lor = false;

    info!("jack_detect_task: started");

    loop {
        let hp = unsafe { rza1l_hal::gpio::read_pin(6, 5) };
        let li = unsafe { rza1l_hal::gpio::read_pin(6, 6) };
        let mic = unsafe { rza1l_hal::gpio::read_pin(7, 9) };
        let lol = unsafe { rza1l_hal::gpio::read_pin(6, 3) };
        let lor = unsafe { rza1l_hal::gpio::read_pin(6, 4) };
        let speaker_enable = !(hp || lol || lor);

        unsafe { rza1l_hal::gpio::write(4, 1, speaker_enable) };

        if hp != prev_hp || li != prev_li || mic != prev_mic || lol != prev_lol || lor != prev_lor {
            prev_hp = hp;
            prev_li = li;
            prev_mic = mic;
            prev_lol = lol;
            prev_lor = lor;

            info!(
                "jack: hp={} line_in={} mic={} line_out_L={} line_out_R={} speaker_enable={}",
                hp, li, mic, lol, lor, speaker_enable
            );
        }

        Timer::after_millis(poll_interval_ms).await;
    }
}
