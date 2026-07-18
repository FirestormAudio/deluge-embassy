//! Minimal PIC32 transport task (copied from the MSC firmware).
//!
//! Performs the PIC baud-rate handshake and relays only the OLED chip-select
//! echo (`OledSelected` / `OledDeselected`) that [`deluge_bsp::oled`] waits on
//! during init and frame writes.  All other PIC events are discarded — the WP
//! probe needs nothing else from the front panel.

use log::info;

use deluge_bsp::pic;

#[embassy_executor::task]
pub(crate) async fn pic_task() {
    info!("PIC: init (31 250 → 200 000 baud)");
    pic::init().await;
    info!("PIC: ready");

    let mut parser = pic::Parser::new();
    loop {
        let byte = rza1l_hal::uart::read_byte(pic::UART_CH).await;
        let Some(event) = parser.push(byte) else {
            continue;
        };
        match event {
            pic::Event::OledSelected => pic::notify_oled_selected(),
            pic::Event::OledDeselected => pic::notify_oled_deselected(),
            _ => {}
        }
    }
}
