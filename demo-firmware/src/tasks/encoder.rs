use core::sync::atomic::Ordering;
use log::info;

use crate::events::{EVENT_CHANNEL, HardwareEvent};
use deluge_bsp::{controls, encoder, pic, scux_dvu_path};

/// Map a 0–32 level to a 4-segment brightness bar.
///
/// Each 8-step increment fills one more segment.  Partial steps produce
/// intermediate brightness so movement is perceptible at any speed.
fn knob_brightnesses(level: i32) -> [u8; 4] {
    let mut b = [0u8; 4];
    for (i, slot) in b.iter_mut().enumerate() {
        let base = (i as i32) * 8;
        *slot = if level >= base + 8 {
            255
        } else if level > base {
            ((level - base) * 32) as u8 // 1‥7 → 32‥224
        } else {
            0
        };
    }
    b
}

/// Map a gold-knob level (0–32) to a DVU VOLxR value.
///
/// | Level | Volume |
/// |-------|--------|
/// | 0     | 0x0000_0000 (−∞ dB, mute) |
/// | 16    | 0x0010_0000 (0 dB, unity) |
/// | 32    | 0x007F_FFFF (+18 dB, max boost) |
fn dvu_vol_from_knob(level: i32) -> u32 {
    if level <= 0 {
        0
    } else if level <= 16 {
        // Attenuation zone: 0 → 0x0010_0000 linearly.
        (level as u32) * 0x0010_0000 / 16
    } else {
        // Boost zone: 0x0010_0000 → 0x007F_FFFF linearly.
        0x0010_0000 + (level as u32 - 16) * (0x007F_FFFF - 0x0010_0000) / 16
    }
}

/// Quadrature encoder task — interrupt-driven.
///
/// Sleeps via [`AtomicWaker`] until any encoder IRQ fires, then drains the BSP
/// encoder state and emits detent events.
///
/// Gold knobs (`controls::encoder::MOD_0` and `controls::encoder::MOD_1`) additionally
/// drive their indicator rings via [`pic::set_gold_knob_indicators`].
#[embassy_executor::task]
pub(crate) async fn encoder_task() {
    let mut enc_pos = [0i8; encoder::NUM_ENCODERS];
    // MOD knob levels: 0 = −∞ dB, 16 = 0 dB (unity), 32 = +18 dB.
    // Init at unity so audio is heard at full level on boot.
    let mut knob_level = [16i32; 2];

    info!("encoder_task: started (interrupt-driven)");

    loop {
        // Sleep until an ISR increments at least one delta.
        core::future::poll_fn(|cx| {
            encoder::ENCODER_WAKER.register(cx.waker());
            // Check AFTER registering to guarantee no lost wakeup.
            if encoder::ENCODER_DELTAS
                .iter()
                .any(|d| d.load(Ordering::Relaxed) != 0)
            {
                core::task::Poll::Ready(())
            } else {
                core::task::Poll::Pending
            }
        })
        .await;

        for (i, slot) in enc_pos.iter_mut().enumerate() {
            let detents = encoder::take_detents(i, slot);
            if detents != 0 {
                let encoder_id = i as u8;
                let _ = EVENT_CHANNEL.try_send(HardwareEvent::EncoderRotated {
                    id: encoder_id,
                    delta: detents,
                });
                if encoder_id == controls::encoder::MOD_0 || encoder_id == controls::encoder::MOD_1
                {
                    let k = if encoder_id == controls::encoder::MOD_0 {
                        controls::knob::MOD_0 as usize
                    } else {
                        controls::knob::MOD_1 as usize
                    };
                    knob_level[k] = (knob_level[k] + detents as i32).clamp(0, 32);
                    let brightness = knob_brightnesses(knob_level[k]);
                    pic::set_gold_knob_indicators(k as u8, brightness).await;
                    info!("knob {} level {} → {:?}", k, knob_level[k], brightness);
                    if k == controls::knob::MOD_0 as usize {
                        // Knob 0 (MOD_0) controls DVU master volume.
                        // 0–16: −∞ dB (0x000_0000) to 0 dB (0x0010_0000) linear
                        // 16–32: 0 dB (0x0010_0000) to +18 dB (0x007F_FFFF) linear
                        let vol = dvu_vol_from_knob(knob_level[0]);
                        unsafe { scux_dvu_path::set_volume_stereo(vol) };
                    }
                } else {
                    info!("encoder {} Δ{}", i, detents);
                }
            }
        }
    }
}
