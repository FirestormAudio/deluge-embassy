//! USB CDC-ACM serial task — implements the Deluge USB serial protocol.
//!
//! ## Protocol (version 1.0)
//! Every message on the wire:
//! ```text
//! [len: u16 LE] = 1 + N   (type byte + data bytes; does NOT include itself)
//! [type: u8]
//! [data: N bytes]
//! ```
//! Minimum: 3 bytes (`len=1`, type, no payload).
//!
//! See [`src/controller/usb_serial.h`] for the full message table.

use embassy_futures::select::{Either, select};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender};
use log::{debug, info, warn};

use deluge_bsp::pads::{PAD_BITS, pad_id_from_xy, pad_set_all};
use deluge_bsp::usb::Rusb1Driver;
use deluge_bsp::{cv_gate, pic, uart as bsp_uart};

use crate::events::{EVENT_CHANNEL, HardwareEvent};
use crate::tasks::oled as oled_task;

// Firmware version coordinates — must match the version advertised by the
// host-side Deluge library (examples/host-demo/src/deluge.rs).
const MAJOR: u8 = 1;
const MINOR: u8 = 0;
const PATCH: u8 = 0;

// ── Message type constants (matching usb_serial.h) ───────────────────────────

const MSG_FROM_PAD_PRESSED: u8 = 0x01;
const MSG_FROM_PAD_RELEASED: u8 = 0x02;
const MSG_FROM_BUTTON_PRESSED: u8 = 0x03;
const MSG_FROM_BUTTON_RELEASED: u8 = 0x04;
const MSG_FROM_ENCODER_ROTATED: u8 = 0x05;
const MSG_FROM_VERSION: u8 = 0x10;
const MSG_FROM_PONG: u8 = 0x11;
const MSG_FROM_READY: u8 = 0x12;

const MSG_TO_UPDATE_DISPLAY: u8 = 0x20;
const MSG_TO_CLEAR_DISPLAY: u8 = 0x21;
const MSG_TO_SET_PAD_RGB: u8 = 0x22;
const MSG_TO_CLEAR_ALL_PADS: u8 = 0x23;
const MSG_TO_SET_LED: u8 = 0x24;
const MSG_TO_SET_CV: u8 = 0x25;
const MSG_TO_SET_GATE: u8 = 0x26;
const MSG_TO_SET_ALL_PADS: u8 = 0x27;
const MSG_TO_SET_KNOB_INDICATOR: u8 = 0x28;
const MSG_TO_SET_SYNCED_LED: u8 = 0x29;
const MSG_TO_CLEAR_ALL_LEDS: u8 = 0x2A;
const MSG_TO_SET_BRIGHTNESS: u8 = 0x2B;
const MSG_TO_GET_VERSION: u8 = 0x30;
const MSG_TO_PING: u8 = 0x31;

#[inline]
const fn cdc_button_id(raw_button_id: u8) -> u8 {
    144 + raw_button_id
}

// ── RX accumulation buffer ───────────────────────────────────────────────────

const RX_BUF_SIZE: usize = 2048;

struct RxState {
    buf: [u8; RX_BUF_SIZE],
    pos: usize,
}

impl RxState {
    const fn new() -> Self {
        Self {
            buf: [0; RX_BUF_SIZE],
            pos: 0,
        }
    }

    fn reset(&mut self) {
        self.pos = 0;
    }

    fn append(&mut self, data: &[u8]) {
        let n = data.len().min(self.buf.len() - self.pos);
        self.buf[self.pos..self.pos + n].copy_from_slice(&data[..n]);
        self.pos += n;
    }

    /// Try to extract one complete message. Returns `Some((type, data_start,
    /// end))` if a full message is buffered; `None` if more data is needed.
    /// Sets `self.pos = 0` if the length field is invalid (resynchronise).
    fn try_frame(&mut self) -> Option<(u8, usize, usize)> {
        if self.pos < 3 {
            return None;
        }
        let len = u16::from_le_bytes([self.buf[0], self.buf[1]]) as usize;
        if len == 0 || len + 2 > self.buf.len() {
            // Bad length — discard and resync.
            self.pos = 0;
            return None;
        }
        let total = 2 + len; // length field (2 bytes) + payload (len bytes)
        if self.pos < total {
            return None;
        }
        Some((self.buf[2], 3, total))
    }

    /// Consume `end` bytes from the front of the buffer (shift remaining).
    fn consume(&mut self, end: usize) {
        let remaining = self.pos.saturating_sub(end);
        self.buf.copy_within(end..self.pos, 0);
        self.pos = remaining;
    }
}

// ── TX helpers ────────────────────────────────────────────────────────────────

/// Encode a CDC-from-Deluge message and write it as one USB packet.
///
/// `type_byte + data` must fit in one 64-byte USB FS bulk packet.
async fn send_msg(
    tx: &mut Sender<'static, Rusb1Driver>,
    type_byte: u8,
    data: &[u8],
) -> Result<(), embassy_usb::driver::EndpointError> {
    let total = 3 + data.len();
    debug_assert!(total <= 64, "CDC TX message too large for one FS packet");
    let mut buf = [0u8; 64];
    let len = 1u16 + data.len() as u16;
    buf[0] = (len & 0xFF) as u8;
    buf[1] = (len >> 8) as u8;
    buf[2] = type_byte;
    buf[3..3 + data.len()].copy_from_slice(data);
    tx.write_packet(&buf[..total]).await
}

// ── Incoming message dispatch ─────────────────────────────────────────────────

async fn handle_message(
    tx: &mut Sender<'static, Rusb1Driver>,
    type_byte: u8,
    data: &[u8],
) -> Result<(), embassy_usb::driver::EndpointError> {
    use core::sync::atomic::Ordering;
    match type_byte {
        MSG_TO_UPDATE_DISPLAY => {
            // 768-byte page-major framebuffer (6 pages × 128 columns).
            if data.len() >= deluge_bsp::oled::FRAME_BYTES {
                oled_task::set_cdc_display(&data[..deluge_bsp::oled::FRAME_BYTES]);
            }
        }
        MSG_TO_CLEAR_DISPLAY => {
            oled_task::clear_cdc_display();
        }
        MSG_TO_SET_PAD_RGB => {
            // [col:1][row:1][r:1][g:1][b:1]
            if data.len() >= 5 {
                let col = data[0];
                let row = data[1];
                let lit = (data[2] | data[3] | data[4]) != 0;
                if col < 18 && row < 8 {
                    let id = pad_id_from_xy(col, row) as usize;
                    let word_idx = id / 32;
                    let bit = 1u32 << (id % 32);
                    if lit {
                        PAD_BITS[word_idx].fetch_or(bit, Ordering::Relaxed);
                    } else {
                        PAD_BITS[word_idx].fetch_and(!bit, Ordering::Relaxed);
                    }
                    deluge_bsp::oled::notify_redraw();
                }
            }
        }
        MSG_TO_CLEAR_ALL_PADS => {
            pad_set_all(false);
            deluge_bsp::oled::notify_redraw();
        }
        MSG_TO_SET_LED => {
            // [led_index:1][on:1]
            if data.len() >= 2 {
                let idx = data[0];
                let on = data[1] != 0;
                if on {
                    pic::led_on(idx).await;
                } else {
                    pic::led_off(idx).await;
                }
            }
        }
        MSG_TO_SET_CV => {
            // [channel:1][value_hi:1][value_lo:1]
            if data.len() >= 3 {
                let ch = data[0];
                let value = u16::from_be_bytes([data[1], data[2]]);
                if (ch as usize) < deluge_bsp::cv_gate::NUM_CV_CHANNELS {
                    // Safety: single-threaded; RSPI0_DMA_ACTIVE guard is inside
                    // cv_set_blocking.
                    unsafe {
                        cv_gate::cv_set_blocking(ch, value);
                    }
                }
            }
        }
        MSG_TO_SET_GATE => {
            // [channel:1][on:1]
            if data.len() >= 2 {
                let ch = data[0];
                let on = data[1] != 0;
                if (ch as usize) < deluge_bsp::cv_gate::NUM_GATE_CHANNELS {
                    // Safety: GPIO write only.
                    unsafe {
                        cv_gate::gate_set(ch, on);
                    }
                }
            }
        }
        MSG_TO_SET_ALL_PADS => {
            // [18*8*3 = 432 bytes], col-major: offset = (col*8 + row)*3
            if data.len() >= 18 * 8 * 3 {
                use core::sync::atomic::Ordering;
                for col in 0..18u8 {
                    for row in 0..8u8 {
                        let off = (col as usize * 8 + row as usize) * 3;
                        let lit = (data[off] | data[off + 1] | data[off + 2]) != 0;
                        let id = pad_id_from_xy(col, row) as usize;
                        let word_idx = id / 32;
                        let bit = 1u32 << (id % 32);
                        if lit {
                            PAD_BITS[word_idx].fetch_or(bit, Ordering::Relaxed);
                        } else {
                            PAD_BITS[word_idx].fetch_and(!bit, Ordering::Relaxed);
                        }
                    }
                }
                deluge_bsp::oled::notify_redraw();
            }
        }
        MSG_TO_SET_KNOB_INDICATOR => {
            // [which:1][b0:1][b1:1][b2:1][b3:1]
            if data.len() >= 5 {
                let which = data[0] & 1;
                let levels = [data[1], data[2], data[3], data[4]];
                pic::set_gold_knob_indicators(which, levels).await;
            }
        }
        MSG_TO_SET_SYNCED_LED => {
            // Controls the GPIO "synced" LED (P6.7).  Note: this pin is also
            // toggled by blink_task; the host can override it by calling this.
            if !data.is_empty() {
                let on = data[0] != 0;
                unsafe {
                    rza1l_hal::gpio::write(6, 7, on);
                }
            }
        }
        MSG_TO_CLEAR_ALL_LEDS => {
            // Turn off all 36 indicator LEDs + gold knob indicators.
            for i in 0..36u8 {
                pic::led_off(i).await;
            }
            pic::set_gold_knob_indicators(0, [0; 4]).await;
            pic::set_gold_knob_indicators(1, [0; 4]).await;
            // Also clear the synced LED.
            unsafe {
                rza1l_hal::gpio::write(6, 7, false);
            }
        }
        MSG_TO_SET_BRIGHTNESS => {
            // [level: 0–25]  0=dimmest, 25=brightest.
            // Maps to PIC SET_REFRESH_TIME: interval = 25 - level.
            // TODO: expose set_dimmer_interval from deluge-bsp pic module.
            if !data.is_empty() {
                let level = data[0].min(25);
                warn!(
                    "SetBrightness level={} (PIC dimmer not yet exposed from BSP)",
                    level
                );
                let _ = bsp_uart::write_bytes(deluge_bsp::uart::PIC_CH, &[19u8, 25 - level]).await;
            }
        }
        MSG_TO_GET_VERSION => {
            send_msg(tx, MSG_FROM_VERSION, &[MAJOR, MINOR, PATCH]).await?;
        }
        MSG_TO_PING => {
            send_msg(tx, MSG_FROM_PONG, &[]).await?;
        }
        other => {
            debug!("CDC: unknown message type 0x{:02X}", other);
        }
    }
    Ok(())
}

// ── Session loop ─────────────────────────────────────────────────────────────

async fn run_session(
    tx: &mut Sender<'static, Rusb1Driver>,
    rx: &mut Receiver<'static, Rusb1Driver>,
) {
    let mut rx_state = RxState::new();
    let mut pkt_buf = [0u8; 64];

    loop {
        match select(EVENT_CHANNEL.receive(), rx.read_packet(&mut pkt_buf)).await {
            Either::First(event) => {
                let result = match event {
                    HardwareEvent::PadPressed { col, row } => {
                        send_msg(tx, MSG_FROM_PAD_PRESSED, &[col, row]).await
                    }
                    HardwareEvent::PadReleased { col, row } => {
                        send_msg(tx, MSG_FROM_PAD_RELEASED, &[col, row]).await
                    }
                    HardwareEvent::ButtonPressed { id } => {
                        send_msg(tx, MSG_FROM_BUTTON_PRESSED, &[cdc_button_id(id)]).await
                    }
                    HardwareEvent::ButtonReleased { id } => {
                        send_msg(tx, MSG_FROM_BUTTON_RELEASED, &[cdc_button_id(id)]).await
                    }
                    HardwareEvent::EncoderRotated { id, delta } => {
                        send_msg(tx, MSG_FROM_ENCODER_ROTATED, &[id, delta as u8]).await
                    }
                };
                if result.is_err() {
                    // Endpoint error — host disconnected.
                    rx_state.reset();
                    return;
                }
            }
            Either::Second(Ok(n)) => {
                rx_state.append(&pkt_buf[..n]);
                // Process all complete messages in the buffer.
                loop {
                    match rx_state.try_frame() {
                        None => break,
                        Some((type_byte, data_start, end)) => {
                            let data_end = end;
                            // We need a copy since we borrow rx_state mutably after.
                            let mut msg_buf = [0u8; RX_BUF_SIZE];
                            let data_len = data_end.saturating_sub(data_start);
                            msg_buf[..data_len]
                                .copy_from_slice(&rx_state.buf[data_start..data_end]);
                            rx_state.consume(end);
                            if handle_message(tx, type_byte, &msg_buf[..data_len])
                                .await
                                .is_err()
                            {
                                return;
                            }
                        }
                    }
                }
            }
            Either::Second(Err(_)) => {
                // Endpoint disabled — host disconnected.
                rx_state.reset();
                return;
            }
        }
    }
}

// ── Embassy task ─────────────────────────────────────────────────────────────

/// CDC-ACM serial task.
///
/// Loops forever: waits for a host connection (DTR set), sends the READY +
/// VERSION greeting, then drives the bidirectional event ↔ command session
/// until the host disconnects.  On disconnect, falls back to waiting for the
/// next connection.
#[embassy_executor::task]
pub(crate) async fn cdc_task(class: CdcAcmClass<'static, Rusb1Driver>) {
    let (mut tx, mut rx) = class.split();
    loop {
        // Wait for the host to open the port (DTR asserted / endpoint enabled).
        tx.wait_connection().await;
        info!("CDC: host connected");

        // Greet the host.
        let _ = send_msg(&mut tx, MSG_FROM_READY, &[]).await;
        let _ = send_msg(&mut tx, MSG_FROM_VERSION, &[MAJOR, MINOR, PATCH]).await;

        // Drive the session until the host disconnects.
        run_session(&mut tx, &mut rx).await;

        // When the host disconnects, revert the OLED to pad rendering.
        oled_task::clear_cdc_display();
        info!("CDC: host disconnected");
    }
}
