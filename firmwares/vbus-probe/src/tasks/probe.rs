//! VBUS-sense probe: read the RUSB1 VBUS monitor bit (`INTSTS0.VBSTS`) and show
//! PRESENT / ABSENT on the OLED and over RTT.
//!
//! In host mode the connected device must be powered by the port's external
//! VBUS switch (the RZ/A1L has no internal VBUS drive). Plug a device and watch
//! this: ABSENT means the device is unpowered — enumeration cannot proceed.

use embassy_time::Timer;
use log::info;

use deluge_bsp::oled::{self, text};
use deluge_bsp::pic;
use rza1l_hal::usb::regs::{INTSTS0_VBSTS, Rusb1Regs, rd};

/// First on-screen pixel row (the top rows sit off the visible panel area).
const TOP: usize = 10;

#[embassy_executor::task]
pub(crate) async fn probe_task() {
    // oled::init() drives the panel over RSPI0 and waits on the PIC chip-select
    // echo, so the PIC handshake must finish first.
    pic::wait_ready().await;
    oled::init().await;
    info!("VBUS-PROBE: reading USB0 INTSTS0.VBSTS (host mode)");

    let mut fb = oled::FrameBuffer::new();
    let mut last: i32 = -1;
    loop {
        // VBSTS (INTSTS0 bit 7): the RUSB1 VBUS-pin monitor. 1 = VBUS present.
        let present = unsafe {
            let regs = Rusb1Regs::ptr(0);
            rd(core::ptr::addr_of!((*regs).intsts0)) & INTSTS0_VBSTS != 0
        };

        if last != present as i32 {
            info!("VBUS-PROBE: VBUS {}", if present { "PRESENT" } else { "ABSENT" });
            last = present as i32;
        }

        fb.fill(0x00);
        text::draw_str(&mut fb, 0, TOP, b"VBUS PROBE (host)");
        text::draw_str(
            &mut fb,
            0,
            TOP + 16,
            if present { b"VBUS: PRESENT" } else { b"VBUS: ABSENT " },
        );
        oled::send_frame(&fb).await;
        Timer::after_millis(200).await;
    }
}
