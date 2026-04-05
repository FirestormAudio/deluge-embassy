use log::{debug, info};

use crate::events::{EVENT_CHANNEL, HardwareEvent};
use deluge_bsp::controls;
use deluge_bsp::oled;
use deluge_bsp::pads::{pad_invert_all, pad_set_all, pad_toggle};
use deluge_bsp::pic;

/// PIC32 event dispatcher — input handling and LED feedback.
///
/// | Event                  | Action                                              |
/// |------------------------|-----------------------------------------------------|
/// | `PadPress { id }`         | Toggle pad; signal OLED redraw; log (x, y)          |
/// | `ButtonPress BACK`        | Clear all pads + LEDs; signal redraw                |
/// | `ButtonPress FILL`        | Fill all pads + LEDs; signal redraw                 |
/// | `ButtonPress SELECT`      | Invert all pads; signal redraw                      |
/// | `ButtonPress other`       | Light indicator LED while held                      |
/// | `ButtonRelease`           | Extinguish indicator LED                            |
/// | `OledSelected`            | Forward to [`pic::notify_oled_selected()`]          |
/// | `OledDeselected`          | Forward to [`pic::notify_oled_deselected()`]        |
#[embassy_executor::task]
pub(crate) async fn pic_task() {
    info!("PIC: init (31 250 → 200 000 baud)");
    pic::init().await;
    info!("PIC: ready");

    let mut parser = pic::Parser::new();
    debug!("pic_task: entering main loop");

    loop {
        let byte = rza1l_hal::uart::read_byte(pic::UART_CH).await;
        let Some(event) = parser.push(byte) else {
            continue;
        };

        match event {
            // ---- Pad toggle ------------------------------------------------
            pic::Event::PadPress { id } => {
                let lit = pad_toggle(id);
                let (col, row) = pic::pad_coords(id);
                info!(
                    "pad {} ({},{}) → {}",
                    id,
                    col,
                    row,
                    if lit { "on" } else { "off" }
                );
                oled::notify_redraw();
                let _ = EVENT_CHANNEL.try_send(HardwareEvent::PadPressed { col, row });
            }
            pic::Event::PadRelease { id } => {
                let (col, row) = pic::pad_coords(id);
                let _ = EVENT_CHANNEL.try_send(HardwareEvent::PadReleased { col, row });
            }

            // ---- Buttons ---------------------------------------------------
            pic::Event::ButtonPress { id } => {
                info!("button {} press", id);
                let _ = EVENT_CHANNEL.try_send(HardwareEvent::ButtonPressed { id });
                if id == controls::encoder_button::TEMPO {
                    let enabled = crate::tasks::blink::toggle_heartbeat_enabled();
                    info!(
                        "tempo knob press: heartbeat {}",
                        if enabled { "enabled" } else { "disabled" }
                    );
                }
                pic::led_on(id).await;
                match id {
                    controls::button::BACK => {
                        pad_set_all(false);
                        for led in 0..36u8 {
                            pic::led_off(led).await;
                        }
                        oled::notify_redraw();
                    }
                    controls::button::FILL => {
                        pad_set_all(true);
                        for led in 0..36u8 {
                            pic::led_on(led).await;
                        }
                        oled::notify_redraw();
                    }
                    controls::button::SELECT => {
                        pad_invert_all();
                        oled::notify_redraw();
                    }
                    _ => {}
                }
            }
            pic::Event::ButtonRelease { id } => {
                let _ = EVENT_CHANNEL.try_send(HardwareEvent::ButtonReleased { id });
                pic::led_off(id).await;
            }

            // ---- OLED CS handshake — must be forwarded --------------------
            pic::Event::OledSelected => {
                pic::notify_oled_selected();
            }
            pic::Event::OledDeselected => {
                pic::notify_oled_deselected();
            }

            // ---- Misc ------------------------------------------------------
            pic::Event::FirmwareVersion(v) => info!("PIC fw v{}", v),
            pic::Event::NoPresses => {}

            // Event is #[non_exhaustive]; future variants are silently ignored.
            _ => {}
        }
    }
}
