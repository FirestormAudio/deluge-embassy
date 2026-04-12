//! USB MIDI class for the Deluge — MIDI 1.0 (alt 0) + MIDI 2.0 (alt 1).
//!
//! Exposes a MIDI Streaming interface with two alternate settings:
//!
//! | Alt | `bcdMSC` | Endpoint MPS | Wire format |
//! |---|---|---|---|
//! | 0 | `0x0100` | 64 bytes | USB MIDI 1.0 — 4-byte event packets (CIN + 3 bytes) |
//! | 1 | `0x0200` | 512 bytes | USB MIDI 2.0 — raw UMP 32-bit words |
//!
//! The host selects an alt setting via `SET_INTERFACE`.  A [`MidiClassHandler`]
//! implements [`embassy_usb::Handler`] to track the active mode (stored in the
//! static [`MIDI2_ACTIVE`] flag) and to respond to class-specific
//! `GET_DESCRIPTOR (0x26)` requests with the Group Terminal Block descriptors
//! required by the MIDI 2.0 spec.
//!
//! ## Quick start
//!
//! ```rust,no_run
//! // USB setup:
//! let (mut midi_handler, midi_eps) = midi::build(&mut builder);
//! builder.handler(&mut midi_handler);
//!
//! // Spawn four background tasks:
//! spawner.spawn(midi_rx_midi1_task(midi_eps.ep_out_midi1));
//! spawner.spawn(midi_tx_midi1_task(midi_eps.ep_in_midi1));
//! spawner.spawn(midi_rx_midi2_task(midi_eps.ep_out_midi2));
//! spawner.spawn(midi_tx_midi2_task(midi_eps.ep_in_midi2));
//!
//! // Non-async API (called from midi_task):
//! if midi::midi2_active() {
//!     if let Some(words) = midi::try_recv_ump_from_host() { /* … */ }
//!     midi::try_send_ump_to_host([0x1000_00F8, 0, 0, 0]);  // clock
//! } else {
//!     if let Some(msg) = midi::try_recv_from_host() { /* … */ }
//!     midi::try_send_to_host_byte(0xF8);
//! }
//! ```

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_usb::control::{InResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};

// ============================================================================
// USB class / descriptor constants
// ============================================================================

const USB_CLASS_AUDIO: u8 = 0x01;
const USB_SUBCLASS_AUDIO_CONTROL: u8 = 0x01;
const USB_SUBCLASS_MIDI_STREAMING: u8 = 0x03;

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;

// CS_INTERFACE subtypes
const MS_HEADER: u8 = 0x01;
const MIDI_IN_JACK: u8 = 0x02;
const MIDI_OUT_JACK: u8 = 0x03;

// Jack type codes
const JACK_EMBEDDED: u8 = 0x01;
const JACK_EXTERNAL: u8 = 0x02;

// Jack IDs (used in alt 0 / MIDI 1.0)
const JACK_EMB_IN_ID: u8 = 1;
const JACK_EXT_IN_ID: u8 = 2;
const JACK_EMB_OUT_ID: u8 = 3;
const JACK_EXT_OUT_ID: u8 = 4;

// CS_ENDPOINT subtypes
const MS_GENERAL: u8 = 0x01;     // MIDI 1.0
const MS_GENERAL_2_0: u8 = 0x02; // MIDI 2.0

// Group Terminal Block descriptor type and subtypes (USB MIDI 2.0 spec §5.5)
const CS_GR_TRM_BLOCK: u8 = 0x26;
const GR_TRM_BLOCK_HEADER: u8 = 0x01;
const GR_TRM_BLOCK: u8 = 0x02;
const GTB_ID: u8 = 0x01;             // single bidirectional GTB
const GTB_MIDI2_PROTOCOL: u8 = 0x02; // bMIDIProtocol: MIDI 2.0 UMP

// Bulk endpoint max packet sizes
const BULK_MPS_MIDI1: u16 = 64;  // USB FS-compatible, used for alt 0
const BULK_MPS_MIDI2: u16 = 512; // USB HS, used for alt 1

// bcdMSC values (little-endian bytes)
const BCD_MSC_1_0: [u8; 2] = [0x00, 0x01]; // 0x0100
const BCD_MSC_2_0: [u8; 2] = [0x00, 0x02]; // 0x0200

// MIDI 1.0 wTotalLength: header(7) + 2×IN jack(6) + 2×OUT jack(9) = 37
const MIDI1_MS_TOTAL: u16 = 37;
// MIDI 2.0 wTotalLength: header(7) only — GTBs served via GET_DESCRIPTOR
const MIDI2_MS_TOTAL: u16 = 7;
// AC header wTotalLength (includes the header descriptor itself)
const AC_TOTAL_LEN: u8 = 9;

// GTB descriptor: header (5 bytes) + 1 block (13 bytes) = 18 bytes.
// Returned by the Handler in response to class GET_DESCRIPTOR(0x26).
const GTB_TOTAL_LEN: u8 = 18;
static GTB_DESCRIPTOR: [u8; 18] = [
    // Group Terminal Block Header (USB MIDI 2.0 spec, Table 5-8)
    0x05, CS_GR_TRM_BLOCK, GR_TRM_BLOCK_HEADER,
    GTB_TOTAL_LEN, 0x00, // wTotalLength = 18 (LE)
    // Group Terminal Block 1 (Table 5-9)
    0x0D, CS_GR_TRM_BLOCK, GR_TRM_BLOCK,
    GTB_ID,              // bGrpTrmBlkID = 1
    0x00,                // bGrpTrmBlkType = 0 (Bi-Directional)
    0x00,                // nGroupTrm = 0 (first UMP group, 0-indexed)
    0x01,                // nNumGroupTrm = 1 (covers groups 0–0)
    0x00,                // iBlockItem = 0 (no string descriptor)
    GTB_MIDI2_PROTOCOL,  // bMIDIProtocol = 0x02 (MIDI 2.0 UMP)
    0x00, 0x00,          // wMaxInputBandwidth (0 = use default)
    0x00, 0x00,          // wMaxOutputBandwidth (0 = use default)
];

// ============================================================================
// Static channels and mode flag
// ============================================================================

/// MIDI 1.0 messages received from the host (alt 0, OUT endpoint).
static MIDI_RX: Channel<CriticalSectionRawMutex, [u8; 3], 16> = Channel::new();

/// Raw MIDI 1.0 bytes queued for transmission to the host (alt 0, IN endpoint).
static MIDI_TX: Channel<CriticalSectionRawMutex, u8, 64> = Channel::new();

/// UMP messages received from the host (alt 1, OUT endpoint).
///
/// Each entry is zero-padded to `[u32; 4]`; use [`ump_word_count`] to
/// determine the number of valid words.
static MIDI2_RX: Channel<CriticalSectionRawMutex, [u32; 4], 16> = Channel::new();

/// UMP messages queued for transmission to the host (alt 1, IN endpoint).
static MIDI2_TX: Channel<CriticalSectionRawMutex, [u32; 4], 16> = Channel::new();

/// `true` when the host has selected alt setting 1 (MIDI 2.0 UMP transport).
static MIDI2_ACTIVE: AtomicBool = AtomicBool::new(false);

// ============================================================================
// Public non-async API
// ============================================================================

/// Returns `true` if the host has negotiated MIDI 2.0 (alt setting 1).
#[inline]
pub fn midi2_active() -> bool {
    MIDI2_ACTIVE.load(Ordering::Acquire)
}

/// Try to receive the next 3-byte MIDI 1.0 message from the host.
/// Only meaningful when `!midi2_active()`.
#[inline]
pub fn try_recv_from_host() -> Option<[u8; 3]> {
    MIDI_RX.try_receive().ok()
}

/// Queue one raw MIDI 1.0 byte for transmission to the host.
/// Returns `true` if queued, `false` if the buffer is full.
/// Only meaningful when `!midi2_active()`.
#[inline]
pub fn try_send_to_host_byte(byte: u8) -> bool {
    MIDI_TX.try_send(byte).is_ok()
}

/// Try to receive the next UMP message from the host.
/// Only meaningful when `midi2_active()`.
#[inline]
pub fn try_recv_ump_from_host() -> Option<[u32; 4]> {
    MIDI2_RX.try_receive().ok()
}

/// Queue a UMP message for transmission to the host.
/// Returns `true` if queued, `false` if the buffer is full.
/// Only meaningful when `midi2_active()`.
#[inline]
pub fn try_send_ump_to_host(words: [u32; 4]) -> bool {
    MIDI2_TX.try_send(words).is_ok()
}

// ============================================================================
// Handler — SET_INTERFACE tracking and GTB GET_DESCRIPTOR response
// ============================================================================

/// USB event handler for the MIDI Streaming interface.
///
/// Must be registered with `builder.handler(&mut handler)` after calling
/// [`build`] and before `builder.build()`.
pub struct MidiClassHandler {
    ms_iface: InterfaceNumber,
}

impl Handler for MidiClassHandler {
    fn set_alternate_setting(&mut self, iface: InterfaceNumber, alternate_setting: u8) {
        if iface == self.ms_iface {
            MIDI2_ACTIVE.store(alternate_setting == 1, Ordering::Release);
            log::info!(
                "usb-midi: alt={} ({})",
                alternate_setting,
                if alternate_setting == 1 { "MIDI 2.0" } else { "MIDI 1.0" }
            );
        }
    }

    /// Respond to class-specific `GET_DESCRIPTOR(0x26)` — the Group Terminal
    /// Block descriptor required by the MIDI 2.0 spec.
    ///
    /// Request signature: `bmRequestType=0xA1`, `bRequest=0x01`,
    /// `wValue=0x2600`, `wIndex=<MS interface number>`.
    fn control_in<'a>(&'a mut self, req: Request, _buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.request_type == RequestType::Class
            && req.recipient == Recipient::Interface
            && req.request == 0x01
            && req.value == 0x2600
            && req.index == u8::from(self.ms_iface) as u16
        {
            return Some(InResponse::Accepted(&GTB_DESCRIPTOR));
        }
        None
    }
}

// ============================================================================
// Descriptor builder
// ============================================================================

/// Endpoint handles returned by [`build`].
pub struct MidiEndpoints<'d, D: Driver<'d>> {
    /// Alt 0 bulk OUT — 64-byte MIDI 1.0 event packets (host → device).
    pub ep_out_midi1: D::EndpointOut,
    /// Alt 0 bulk IN  — 64-byte MIDI 1.0 event packets (device → host).
    pub ep_in_midi1: D::EndpointIn,
    /// Alt 1 bulk OUT — 512-byte UMP words (host → device).
    pub ep_out_midi2: D::EndpointOut,
    /// Alt 1 bulk IN  — 512-byte UMP words (device → host).
    pub ep_in_midi2: D::EndpointIn,
}

/// Register the dual-alt MIDI function with the USB descriptor builder.
///
/// Returns `(handler, endpoints)`.  Register the handler with
/// `builder.handler(&mut handler)` before calling `builder.build()`.
pub fn build<'d, D: Driver<'d>>(
    builder: &mut Builder<'d, D>,
) -> (MidiClassHandler, MidiEndpoints<'d, D>) {
    let mut func = builder.function(USB_CLASS_AUDIO, 0x00, 0x00);

    // ── AudioControl interface (mandatory stub for USB Audio Class) ─────────
    let mut ac_ib = func.interface();
    let ac_if = ac_ib.interface_number();
    // The MS interface immediately follows the AC interface.
    let ms_if_num = u8::from(ac_if) + 1;
    let mut ac_alt =
        ac_ib.alt_setting(USB_CLASS_AUDIO, USB_SUBCLASS_AUDIO_CONTROL, 0x00, None);
    // CS AC Interface Header: bcdADC=1.00, wTotalLength=9, bInCollection=1, MS iface#
    ac_alt.descriptor(
        CS_INTERFACE,
        &[MS_HEADER, 0x00, 0x01, AC_TOTAL_LEN, 0x00, 0x01, ms_if_num],
    );

    // ── MIDI Streaming interface ────────────────────────────────────────────
    let mut ms_ib = func.interface();
    let ms_iface = ms_ib.interface_number();

    // Alt 0: MIDI 1.0 — 64-byte bulk, jack-based class descriptors ───────────
    let mut alt0 =
        ms_ib.alt_setting(USB_CLASS_AUDIO, USB_SUBCLASS_MIDI_STREAMING, 0x00, None);

    // CS MS Interface Header: bcdMSC=1.00, wTotalLength=37
    alt0.descriptor(
        CS_INTERFACE,
        &[
            MS_HEADER,
            BCD_MSC_1_0[0], BCD_MSC_1_0[1],
            (MIDI1_MS_TOTAL & 0xFF) as u8, (MIDI1_MS_TOTAL >> 8) as u8,
        ],
    );
    // MIDI IN Jacks
    alt0.descriptor(CS_INTERFACE, &[MIDI_IN_JACK, JACK_EMBEDDED, JACK_EMB_IN_ID, 0x00]);
    alt0.descriptor(CS_INTERFACE, &[MIDI_IN_JACK, JACK_EXTERNAL, JACK_EXT_IN_ID, 0x00]);
    // MIDI OUT Jacks (each references the opposite IN jack as its source)
    alt0.descriptor(
        CS_INTERFACE,
        &[MIDI_OUT_JACK, JACK_EMBEDDED, JACK_EMB_OUT_ID, 1, JACK_EXT_IN_ID, 1, 0x00],
    );
    alt0.descriptor(
        CS_INTERFACE,
        &[MIDI_OUT_JACK, JACK_EXTERNAL, JACK_EXT_OUT_ID, 1, JACK_EMB_IN_ID, 1, 0x00],
    );
    // Bulk OUT (host → device)
    let ep_out_midi1 = alt0.endpoint_bulk_out(None, BULK_MPS_MIDI1);
    alt0.descriptor(CS_ENDPOINT, &[MS_GENERAL, 1, JACK_EMB_OUT_ID]);
    // Bulk IN (device → host)
    let ep_in_midi1 = alt0.endpoint_bulk_in(None, BULK_MPS_MIDI1);
    alt0.descriptor(CS_ENDPOINT, &[MS_GENERAL, 1, JACK_EMB_IN_ID]);

    // Alt 1: MIDI 2.0 — 512-byte bulk, GTBs via class GET_DESCRIPTOR ─────────
    let mut alt1 =
        ms_ib.alt_setting(USB_CLASS_AUDIO, USB_SUBCLASS_MIDI_STREAMING, 0x00, None);

    // CS MS Interface Header: bcdMSC=2.00, wTotalLength=7 (no jack descriptors)
    alt1.descriptor(
        CS_INTERFACE,
        &[
            MS_HEADER,
            BCD_MSC_2_0[0], BCD_MSC_2_0[1],
            (MIDI2_MS_TOTAL & 0xFF) as u8, (MIDI2_MS_TOTAL >> 8) as u8,
        ],
    );
    // Bulk OUT (host → device)
    let ep_out_midi2 = alt1.endpoint_bulk_out(None, BULK_MPS_MIDI2);
    alt1.descriptor(CS_ENDPOINT, &[MS_GENERAL_2_0, 1, GTB_ID]);
    // Bulk IN (device → host)
    let ep_in_midi2 = alt1.endpoint_bulk_in(None, BULK_MPS_MIDI2);
    alt1.descriptor(CS_ENDPOINT, &[MS_GENERAL_2_0, 1, GTB_ID]);

    (
        MidiClassHandler { ms_iface },
        MidiEndpoints { ep_out_midi1, ep_in_midi1, ep_out_midi2, ep_in_midi2 },
    )
}

// ============================================================================
// UMP helpers
// ============================================================================

/// Return the number of 32-bit words in a UMP message.
///
/// Based on UMP spec (M2-104-UM) Table 2-1 — Message Type (bits 31–28).
#[inline]
pub fn ump_word_count(first_word: u32) -> usize {
    match (first_word >> 28) as u8 {
        // 32-bit (1 word): Utility, SysRT/SysCommon, MIDI 1.0 CV, reserved 6/7
        0x0..=0x2 | 0x6 | 0x7 => 1,
        // 64-bit (2 words): Sysex7, MIDI 2.0 CV, reserved 8–C
        0x3 | 0x4 | 0x8..=0xC => 2,
        // 128-bit (4 words): Sysex8/MixedData, FlexData, reserved E, UMP Stream
        _ => 4,
    }
}

// ============================================================================
// USB MIDI 1.0 packet helpers
// ============================================================================

/// Parse a 4-byte USB MIDI 1.0 event packet → 3-byte MIDI message.
fn parse_usb_midi_packet(packet: &[u8]) -> Option<[u8; 3]> {
    if packet.len() < 4 {
        return None;
    }
    let (b1, b2, b3) = (packet[1], packet[2], packet[3]);
    match packet[0] & 0x0F {
        0x03 | 0x08 | 0x09 | 0x0A | 0x0B | 0x0E => Some([b1, b2, b3]),
        0x02 | 0x06 | 0x0C | 0x0D => Some([b1, b2, 0]),
        0x05 | 0x0F => Some([b1, 0, 0]),
        _ => None,
    }
}

/// Build a 4-byte USB MIDI 1.0 event packet from a 3-byte MIDI message.
fn build_usb_midi_packet(msg: &[u8; 3]) -> [u8; 4] {
    let cin: u8 = match msg[0] {
        0x80..=0x8F => 0x08, 0x90..=0x9F => 0x09,
        0xA0..=0xAF => 0x0A, 0xB0..=0xBF => 0x0B,
        0xC0..=0xCF => 0x0C, 0xD0..=0xDF => 0x0D,
        0xE0..=0xEF => 0x0E, 0xF8..=0xFF => 0x0F,
        0xF2 => 0x03, 0xF3 => 0x02,
        _ => 0x0F,
    };
    [cin, msg[0], msg[1], msg[2]]
}

// ============================================================================
// Background tasks — Alt 0 (MIDI 1.0)
// ============================================================================

/// Drive the alt-0 bulk OUT endpoint (host → device, MIDI 1.0).
///
/// Reads 4-byte USB MIDI 1.0 event packets and pushes decoded 3-byte
/// messages into [`MIDI_RX`].
pub async fn run_rx_midi1<'d, D: Driver<'d>>(mut ep_out: D::EndpointOut)
where
    D::EndpointOut: EndpointOut,
{
    let mut buf = [0u8; 64];
    loop {
        ep_out.wait_enabled().await;
        loop {
            match ep_out.read(&mut buf).await {
                Ok(n) => {
                    let mut i = 0;
                    while i + 4 <= n {
                        if let Some(msg) = parse_usb_midi_packet(&buf[i..i + 4]) {
                            let _ = MIDI_RX.try_send(msg);
                        }
                        i += 4;
                    }
                }
                Err(EndpointError::Disabled) => break,
                Err(EndpointError::BufferOverflow) => {
                    log::warn!("usb-midi1: RX overflow");
                }
            }
        }
    }
}

/// Drive the alt-0 bulk IN endpoint (device → host, MIDI 1.0).
///
/// Reads raw MIDI 1.0 bytes from [`MIDI_TX`], assembles them into
/// 3-byte messages (with running-status and realtime handling), and writes
/// 4-byte USB MIDI event packets to the host.
pub async fn run_tx_midi1<'d, D: Driver<'d>>(mut ep_in: D::EndpointIn)
where
    D::EndpointIn: EndpointIn,
{
    let mut pending = [0u8; 3];
    let mut pos: usize = 0;
    let mut expected: usize = 0;

    loop {
        ep_in.wait_enabled().await;
        loop {
            let byte = MIDI_TX.receive().await;

            if byte >= 0xF8 {
                // Single-byte realtime — send immediately.
                let pkt = build_usb_midi_packet(&[byte, 0, 0]);
                if ep_in.write(&pkt).await.is_err() {
                    break;
                }
                continue;
            }

            if byte & 0x80 != 0 {
                pending[0] = byte;
                pos = 1;
                expected = match byte { 0xC0..=0xDF | 0xF3 => 2, _ => 3 };
            } else if pos > 0 {
                if pos < 3 { pending[pos] = byte; }
                pos += 1;
                if pos >= expected {
                    let msg = [
                        pending[0],
                        pending[1],
                        if expected >= 3 { pending[2] } else { 0 },
                    ];
                    let pkt = build_usb_midi_packet(&msg);
                    if ep_in.write(&pkt).await.is_err() {
                        break;
                    }
                    pos = 1; // retain status for running-status
                }
            }
        }
        pos = 0;
    }
}

// ============================================================================
// Background tasks — Alt 1 (MIDI 2.0 / UMP)
// ============================================================================

/// Drive the alt-1 bulk OUT endpoint (host → device, MIDI 2.0 UMP).
///
/// Reads raw 32-bit UMP words from the host and pushes complete messages
/// (zero-padded to `[u32; 4]`) into [`MIDI2_RX`].
pub async fn run_rx_midi2<'d, D: Driver<'d>>(mut ep_out: D::EndpointOut)
where
    D::EndpointOut: EndpointOut,
{
    let mut buf = [0u8; 512];
    loop {
        ep_out.wait_enabled().await;
        loop {
            match ep_out.read(&mut buf).await {
                Ok(n) => {
                    let mut i = 0;
                    while i + 4 <= n {
                        let w0 = u32::from_le_bytes(
                            buf[i..i + 4].try_into().unwrap_or([0; 4]),
                        );
                        let wc = ump_word_count(w0);
                        let end = i + wc * 4;
                        if end > n {
                            break; // incomplete UMP message, discard remainder
                        }
                        let mut words = [0u32; 4];
                        for k in 0..wc {
                            let off = i + k * 4;
                            words[k] = u32::from_le_bytes(
                                buf[off..off + 4].try_into().unwrap_or([0; 4]),
                            );
                        }
                        let _ = MIDI2_RX.try_send(words);
                        i = end;
                    }
                }
                Err(EndpointError::Disabled) => break,
                Err(EndpointError::BufferOverflow) => {
                    log::warn!("usb-midi2: RX overflow");
                }
            }
        }
    }
}

/// Drive the alt-1 bulk IN endpoint (device → host, MIDI 2.0 UMP).
///
/// Pops UMP messages from [`MIDI2_TX`] and writes the appropriate number
/// of 32-bit words (1, 2, or 4) as little-endian bytes.
pub async fn run_tx_midi2<'d, D: Driver<'d>>(mut ep_in: D::EndpointIn)
where
    D::EndpointIn: EndpointIn,
{
    let mut buf = [0u8; 16]; // max 4 words × 4 bytes
    loop {
        ep_in.wait_enabled().await;
        loop {
            let words = MIDI2_TX.receive().await;
            let wc = ump_word_count(words[0]);
            for k in 0..wc {
                buf[k * 4..k * 4 + 4].copy_from_slice(&words[k].to_le_bytes());
            }
            if ep_in.write(&buf[..wc * 4]).await.is_err() {
                break;
            }
        }
    }
}

// ============================================================================
// Backward-compatible aliases
// ============================================================================

/// Alias for [`run_rx_midi1`] — for callers that only use MIDI 1.0.
pub use run_rx_midi1 as run_rx;
/// Alias for [`run_tx_midi1`] — for callers that only use MIDI 1.0.
pub use run_tx_midi1 as run_tx;

