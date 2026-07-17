//! Deluge host↔device front-panel wire protocol (version 1.0).
//!
//! The Deluge can act as a *dumb control surface*: a **host** owns all illumination
//! (OLED, pad LEDs, button LEDs, knob indicators) and the **device** forwards raw
//! input (pads, buttons, encoders). This crate is the single source of truth for that
//! wire contract, shared by every implementation of either side:
//!
//! - the **device** side — deluge-sdk's `controller-firmware` over USB-CDC;
//! - the **host/panel** side — the desktop simulator (`tools/deluge-simulator`);
//! - the DelugeFirmware C build's `host_link` bridge (which speaks the same bytes
//!   over a Unix socket so the simulator can drive the real firmware natively).
//!
//! ## Framing
//!
//! Every message is length-prefixed:
//!
//! ```text
//! [len: u16 LE] = 1 + N     (type byte + N data bytes; excludes the length field itself)
//! [type: u8]
//! [data: N bytes]
//! ```
//!
//! Minimum frame is 3 bytes (`len = 1`, a type byte, no payload). Over USB-CDC a
//! device→host frame must fit one 64-byte FS bulk packet, so its payload is ≤ 61 bytes;
//! host→device frames (e.g. the 768-byte display blit) are reassembled from packets by
//! [`FrameDecoder`].
//!
//! ## Directions
//!
//! [`ToDeluge`] = host→device (display + LEDs + CV/gate); [`FromDeluge`] = device→host
//! (input + handshake). "To/From" are named from the *device's* point of view and never
//! flip — even though the emulator swaps *which process* plays each role (there the C
//! firmware is the brain emitting [`ToDeluge`], and the simulator is the panel emitting
//! [`FromDeluge`]).

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

#[cfg(feature = "alloc")]
extern crate alloc;
#[cfg(feature = "alloc")]
use alloc::vec::Vec;

// ── Message type bytes (the `usb_serial.h` table) ────────────────────────────

/// `FromDeluge` (device→host) type bytes.
pub mod from {
    pub const PAD_PRESSED: u8 = 0x01;
    pub const PAD_RELEASED: u8 = 0x02;
    pub const BUTTON_PRESSED: u8 = 0x03;
    pub const BUTTON_RELEASED: u8 = 0x04;
    pub const ENCODER_ROTATED: u8 = 0x05;
    pub const VERSION: u8 = 0x10;
    pub const PONG: u8 = 0x11;
    pub const READY: u8 = 0x12;
}

/// `ToDeluge` (host→device) type bytes.
pub mod to {
    pub const UPDATE_DISPLAY: u8 = 0x20;
    pub const CLEAR_DISPLAY: u8 = 0x21;
    pub const SET_PAD_RGB: u8 = 0x22;
    pub const CLEAR_ALL_PADS: u8 = 0x23;
    pub const SET_LED: u8 = 0x24;
    pub const SET_CV: u8 = 0x25;
    pub const SET_GATE: u8 = 0x26;
    pub const SET_ALL_PADS: u8 = 0x27;
    pub const SET_KNOB_INDICATOR: u8 = 0x28;
    pub const SET_SYNCED_LED: u8 = 0x29;
    pub const CLEAR_ALL_LEDS: u8 = 0x2A;
    pub const SET_BRIGHTNESS: u8 = 0x2B;
    pub const GET_VERSION: u8 = 0x30;
    pub const PING: u8 = 0x31;
}

/// The 128×48 OLED is sent as a page-major 1-bpp framebuffer: 6 pages × 128 columns,
/// bit `b` of `buf[page*128 + col]` = panel row `page*8 + b`. This is the byte count of
/// an [`ToDeluge::UpdateDisplay`] payload.
pub const DISPLAY_FRAME_BYTES: usize = 6 * 128; // 768

/// A full-grid [`ToDeluge::SetAllPads`] payload: col-major `[r,g,b]` per pad,
/// `offset = (col*8 + row)*3`, 18 columns × 8 rows.
pub const ALL_PADS_BYTES: usize = 18 * 8 * 3; // 432

/// Raw button ids (0-based, from the device's button matrix) are offset by 144 on the
/// wire so they never collide with pad/encoder small ids. See `hid/button.h`:
/// `id = 9*(y + 2*8) + x`, which is already ≥ 144.
pub const BUTTON_ID_BASE: u8 = 144;

/// Wire button id from a raw 0-based matrix id (`raw + 144`).
#[inline]
pub const fn cdc_button_id(raw: u8) -> u8 {
    BUTTON_ID_BASE.wrapping_add(raw)
}

// ── Framing ──────────────────────────────────────────────────────────────────

/// Write `[len_lo][len_hi][type][data…]` into `out`. Returns the total frame length, or
/// `None` if `out` is too small. `len = 1 + data.len()`.
pub fn encode_frame(type_byte: u8, data: &[u8], out: &mut [u8]) -> Option<usize> {
    let total = 3 + data.len();
    if out.len() < total {
        return None;
    }
    let len = 1u16 + data.len() as u16;
    out[0] = (len & 0xFF) as u8;
    out[1] = (len >> 8) as u8;
    out[2] = type_byte;
    out[3..total].copy_from_slice(data);
    Some(total)
}

/// `Vec`-returning [`encode_frame`].
#[cfg(feature = "alloc")]
pub fn frame(type_byte: u8, data: &[u8]) -> Vec<u8> {
    let len = 1u16 + data.len() as u16;
    let mut v = Vec::with_capacity(3 + data.len());
    v.extend_from_slice(&len.to_le_bytes());
    v.push(type_byte);
    v.extend_from_slice(data);
    v
}

/// Streaming frame reassembler: feed arbitrary byte chunks with [`push`](Self::push),
/// drain whole frames with [`pop_frame`](Self::pop_frame). A length field outside
/// `[1, N-2]` resyncs by discarding the buffer (matches the device's `RxState`).
///
/// `N` is the byte capacity — size it ≥ the largest frame (the 768-byte display blit is
/// `2 + 1 + 768 = 771`, so the default 2048 is comfortable).
pub struct FrameDecoder<const N: usize = 2048> {
    buf: [u8; N],
    pos: usize,
}

impl<const N: usize> Default for FrameDecoder<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> FrameDecoder<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            pos: 0,
        }
    }

    /// Append bytes. Returns the number actually stored (the rest, if any, is dropped
    /// when the buffer is full — drain frames between pushes to avoid that).
    pub fn push(&mut self, data: &[u8]) -> usize {
        let n = data.len().min(N - self.pos);
        self.buf[self.pos..self.pos + n].copy_from_slice(&data[..n]);
        self.pos += n;
        n
    }

    /// Pop the next complete frame: copies its payload into `out`, returns
    /// `(type_byte, payload_len)`, and consumes the frame. Returns `None` when no full
    /// frame is buffered. If the payload is longer than `out`, it is truncated (the
    /// frame is still consumed and the true length returned).
    pub fn pop_frame(&mut self, out: &mut [u8]) -> Option<(u8, usize)> {
        if self.pos < 3 {
            return None;
        }
        let len = u16::from_le_bytes([self.buf[0], self.buf[1]]) as usize;
        if len == 0 || len + 2 > N {
            self.pos = 0; // bad length — resync
            return None;
        }
        let total = 2 + len;
        if self.pos < total {
            return None;
        }
        let type_byte = self.buf[2];
        let payload_len = len - 1;
        let copy = payload_len.min(out.len());
        out[..copy].copy_from_slice(&self.buf[3..3 + copy]);
        // consume
        self.buf.copy_within(total..self.pos, 0);
        self.pos -= total;
        Some((type_byte, payload_len))
    }
}

// ── Typed messages ─────────────────────────────────────────────────────────────

/// Host→device messages (illumination + CV/gate + handshake queries). Borrows the bulk
/// payloads ([`UpdateDisplay`](Self::UpdateDisplay), [`SetAllPads`](Self::SetAllPads))
/// straight from the decode buffer — no allocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToDeluge<'a> {
    /// 768-byte page-major OLED framebuffer ([`DISPLAY_FRAME_BYTES`]).
    UpdateDisplay(&'a [u8]),
    ClearDisplay,
    SetPadRgb {
        col: u8,
        row: u8,
        rgb: [u8; 3],
    },
    ClearAllPads,
    SetLed {
        index: u8,
        on: bool,
    },
    SetCv {
        channel: u8,
        value: u16,
    },
    SetGate {
        channel: u8,
        on: bool,
    },
    /// 432-byte col-major RGB grid ([`ALL_PADS_BYTES`]).
    SetAllPads(&'a [u8]),
    SetKnobIndicator {
        which: u8,
        levels: [u8; 4],
    },
    SetSyncedLed(bool),
    ClearAllLeds,
    SetBrightness(u8),
    GetVersion,
    Ping,
}

impl<'a> ToDeluge<'a> {
    /// Decode one message from its `type` byte + payload (no length header). Returns
    /// `None` for unknown types or short payloads.
    pub fn decode(type_byte: u8, data: &'a [u8]) -> Option<Self> {
        Some(match type_byte {
            to::UPDATE_DISPLAY if data.len() >= DISPLAY_FRAME_BYTES => {
                Self::UpdateDisplay(&data[..DISPLAY_FRAME_BYTES])
            }
            to::CLEAR_DISPLAY => Self::ClearDisplay,
            to::SET_PAD_RGB if data.len() >= 5 => Self::SetPadRgb {
                col: data[0],
                row: data[1],
                rgb: [data[2], data[3], data[4]],
            },
            to::CLEAR_ALL_PADS => Self::ClearAllPads,
            to::SET_LED if data.len() >= 2 => Self::SetLed {
                index: data[0],
                on: data[1] != 0,
            },
            to::SET_CV if data.len() >= 3 => Self::SetCv {
                channel: data[0],
                value: u16::from_be_bytes([data[1], data[2]]),
            },
            to::SET_GATE if data.len() >= 2 => Self::SetGate {
                channel: data[0],
                on: data[1] != 0,
            },
            to::SET_ALL_PADS if data.len() >= ALL_PADS_BYTES => {
                Self::SetAllPads(&data[..ALL_PADS_BYTES])
            }
            to::SET_KNOB_INDICATOR if data.len() >= 5 => Self::SetKnobIndicator {
                which: data[0],
                levels: [data[1], data[2], data[3], data[4]],
            },
            to::SET_SYNCED_LED if !data.is_empty() => Self::SetSyncedLed(data[0] != 0),
            to::CLEAR_ALL_LEDS => Self::ClearAllLeds,
            to::SET_BRIGHTNESS if !data.is_empty() => Self::SetBrightness(data[0]),
            to::GET_VERSION => Self::GetVersion,
            to::PING => Self::Ping,
            _ => return None,
        })
    }

    /// Serialize to a complete framed wire message.
    #[cfg(feature = "alloc")]
    pub fn to_frame(&self) -> Vec<u8> {
        let (t, data) = self.type_and_data();
        match self {
            Self::UpdateDisplay(buf) => frame(t, buf),
            Self::SetAllPads(buf) => frame(t, buf),
            _ => frame(t, &data),
        }
    }

    /// Type byte + a small inline payload (bulk variants carry their payload separately;
    /// see [`to_frame`](Self::to_frame)).
    #[cfg(feature = "alloc")]
    fn type_and_data(&self) -> (u8, [u8; 5]) {
        let mut d = [0u8; 5];
        let t = match *self {
            Self::UpdateDisplay(_) => to::UPDATE_DISPLAY,
            Self::ClearDisplay => to::CLEAR_DISPLAY,
            Self::SetPadRgb { col, row, rgb } => {
                d = [col, row, rgb[0], rgb[1], rgb[2]];
                to::SET_PAD_RGB
            }
            Self::ClearAllPads => to::CLEAR_ALL_PADS,
            Self::SetLed { index, on } => {
                d[0] = index;
                d[1] = on as u8;
                to::SET_LED
            }
            Self::SetCv { channel, value } => {
                let [hi, lo] = value.to_be_bytes();
                d[0] = channel;
                d[1] = hi;
                d[2] = lo;
                to::SET_CV
            }
            Self::SetGate { channel, on } => {
                d[0] = channel;
                d[1] = on as u8;
                to::SET_GATE
            }
            Self::SetAllPads(_) => to::SET_ALL_PADS,
            Self::SetKnobIndicator { which, levels } => {
                d = [which, levels[0], levels[1], levels[2], levels[3]];
                to::SET_KNOB_INDICATOR
            }
            Self::SetSyncedLed(on) => {
                d[0] = on as u8;
                to::SET_SYNCED_LED
            }
            Self::ClearAllLeds => to::CLEAR_ALL_LEDS,
            Self::SetBrightness(level) => {
                d[0] = level;
                to::SET_BRIGHTNESS
            }
            Self::GetVersion => to::GET_VERSION,
            Self::Ping => to::PING,
        };
        (t, d)
    }
}

/// Device→host messages (raw input + handshake). [`ButtonPressed`](Self::ButtonPressed)
/// / [`ButtonReleased`](Self::ButtonReleased) carry the **wire** id (already +144; see
/// [`cdc_button_id`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FromDeluge {
    PadPressed { col: u8, row: u8 },
    PadReleased { col: u8, row: u8 },
    ButtonPressed { id: u8 },
    ButtonReleased { id: u8 },
    EncoderRotated { id: u8, delta: i8 },
    Version { major: u8, minor: u8, patch: u8 },
    Pong,
    Ready,
}

impl FromDeluge {
    /// Decode one message from its `type` byte + payload (no length header).
    pub fn decode(type_byte: u8, data: &[u8]) -> Option<Self> {
        Some(match type_byte {
            from::PAD_PRESSED if data.len() >= 2 => Self::PadPressed {
                col: data[0],
                row: data[1],
            },
            from::PAD_RELEASED if data.len() >= 2 => Self::PadReleased {
                col: data[0],
                row: data[1],
            },
            from::BUTTON_PRESSED if !data.is_empty() => Self::ButtonPressed { id: data[0] },
            from::BUTTON_RELEASED if !data.is_empty() => Self::ButtonReleased { id: data[0] },
            from::ENCODER_ROTATED if data.len() >= 2 => Self::EncoderRotated {
                id: data[0],
                delta: data[1] as i8,
            },
            from::VERSION if data.len() >= 3 => Self::Version {
                major: data[0],
                minor: data[1],
                patch: data[2],
            },
            from::PONG => Self::Pong,
            from::READY => Self::Ready,
            _ => return None,
        })
    }

    /// Serialize to a complete framed wire message.
    #[cfg(feature = "alloc")]
    pub fn to_frame(&self) -> Vec<u8> {
        let (t, data) = self.type_and_data();
        frame(t, &data[..data_len(self)])
    }

    #[cfg(feature = "alloc")]
    fn type_and_data(&self) -> (u8, [u8; 3]) {
        let mut d = [0u8; 3];
        let t = match *self {
            Self::PadPressed { col, row } => {
                d[0] = col;
                d[1] = row;
                from::PAD_PRESSED
            }
            Self::PadReleased { col, row } => {
                d[0] = col;
                d[1] = row;
                from::PAD_RELEASED
            }
            Self::ButtonPressed { id } => {
                d[0] = id;
                from::BUTTON_PRESSED
            }
            Self::ButtonReleased { id } => {
                d[0] = id;
                from::BUTTON_RELEASED
            }
            Self::EncoderRotated { id, delta } => {
                d[0] = id;
                d[1] = delta as u8;
                from::ENCODER_ROTATED
            }
            Self::Version {
                major,
                minor,
                patch,
            } => {
                d = [major, minor, patch];
                from::VERSION
            }
            Self::Pong => from::PONG,
            Self::Ready => from::READY,
        };
        (t, d)
    }
}

#[cfg(feature = "alloc")]
fn data_len(m: &FromDeluge) -> usize {
    match m {
        FromDeluge::PadPressed { .. } | FromDeluge::PadReleased { .. } => 2,
        FromDeluge::ButtonPressed { .. } | FromDeluge::ButtonReleased { .. } => 1,
        FromDeluge::EncoderRotated { .. } => 2,
        FromDeluge::Version { .. } => 3,
        FromDeluge::Pong | FromDeluge::Ready => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_roundtrips_through_decoder() {
        let mut dec = FrameDecoder::<2048>::new();
        let f = frame(to::SET_PAD_RGB, &[3, 4, 10, 20, 30]);
        // split across two pushes to exercise reassembly
        dec.push(&f[..2]);
        dec.push(&f[2..]);
        let mut out = [0u8; 64];
        let (t, n) = dec.pop_frame(&mut out).unwrap();
        assert_eq!(t, to::SET_PAD_RGB);
        assert_eq!(&out[..n], &[3, 4, 10, 20, 30]);
        assert!(dec.pop_frame(&mut out).is_none());
    }

    #[test]
    fn to_deluge_decode_matches_encode() {
        let m = ToDeluge::SetKnobIndicator {
            which: 1,
            levels: [1, 2, 3, 4],
        };
        let f = m.to_frame();
        let mut dec = FrameDecoder::<64>::new();
        dec.push(&f);
        let mut out = [0u8; 16];
        let (t, n) = dec.pop_frame(&mut out).unwrap();
        assert_eq!(ToDeluge::decode(t, &out[..n]), Some(m));
    }

    #[test]
    fn from_deluge_button_id_offset() {
        let f = FromDeluge::ButtonPressed {
            id: cdc_button_id(0),
        }
        .to_frame();
        assert_eq!(f, [2, 0, from::BUTTON_PRESSED, 144]);
    }

    #[test]
    fn bad_length_resyncs() {
        let mut dec = FrameDecoder::<64>::new();
        dec.push(&[0xFF, 0xFF, 0x01]); // len=65535 > capacity → resync
        let mut out = [0u8; 16];
        assert!(dec.pop_frame(&mut out).is_none());
        assert_eq!(dec.pos, 0);
    }
}
