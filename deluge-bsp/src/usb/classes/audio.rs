//! USB Audio Class 2.0 (UAC2) device-side driver for the Deluge.
//!
//! ## Quick start
//!
//! ```rust,no_run
//! let (mut audio, ep_out) = AudioClass::new(&mut builder, 288);
//! builder.handler(&mut audio);
//! let usb = builder.build();
//! ```

use core::sync::atomic::{AtomicU8, Ordering};
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::driver::{Driver, Endpoint};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};

/// Current active bit depth for the speaker stream (16 or 24).
/// Set by [`AudioClass`] when the host selects an alternate setting.
/// Read by `uac2_task` to determine the sample conversion width.
pub static USB_BITS_PER_SAMPLE: AtomicU8 = AtomicU8::new(24);

/// Current active bit depth for the capture (mic) stream (16 or 24).
/// Set by [`AudioClass`] when the host selects a capture alternate setting.
/// Read by `uac2_mic_task` to determine the sample pack width.
pub static USB_CAPTURE_BITS_PER_SAMPLE: AtomicU8 = AtomicU8::new(24);

const USB_CLASS_AUDIO: u8 = 0x01;
const USB_SUBCLASS_AUDIO_CONTROL: u8 = 0x01;
const USB_SUBCLASS_AUDIO_STREAMING: u8 = 0x02;
const USB_PROTOCOL_IP_02_00: u8 = 0x20;

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;
const AC_HEADER: u8 = 0x01;
const AC_INPUT_TERM: u8 = 0x02;
const AC_OUTPUT_TERM: u8 = 0x03;
const AC_CLOCK_SOURCE: u8 = 0x0A;
const AS_GENERAL: u8 = 0x01;
const AS_FORMAT_TYPE: u8 = 0x02;

const CLOCK_SOURCE_ID: u8 = 0x10;
const INPUT_TERM_ID: u8 = 0x11; // USB streaming → speaker
const OUTPUT_TERM_ID: u8 = 0x12; // speaker out
const MIC_INPUT_TERM_ID: u8 = 0x13; // physical microphone
const MIC_OUTPUT_TERM_ID: u8 = 0x14; // USB streaming ← mic

const TERM_MICROPHONE: u16 = 0x0201;

// UAC2 §A.17: bRequest values. Direction is in bmRequestType bit 7, not here.
const UAC2_SET_CUR: u8 = 0x01;
const UAC2_GET_CUR: u8 = 0x01;
const UAC2_GET_RANGE: u8 = 0x02;
const SAM_FREQ_CS: u8 = 0x01;

const TERM_USB_STREAMING: u16 = 0x0101;
const TERM_SPEAKER: u16 = 0x0301;

pub const SAMPLE_RATE_44100: u32 = 44_100;
pub const SAMPLE_RATE_48000: u32 = 48_000;

/// Handles UAC2 SET_CUR / GET_CUR for sample-rate negotiation.
///
/// Create via [`AudioClass::new`], register with `builder.handler()`.
pub struct AudioClass {
    ac_iface: InterfaceNumber,
    as_iface: InterfaceNumber,         // speaker OUT
    as_capture_iface: InterfaceNumber, // mic IN
    /// Current sample rate in Hz (44100 or 48000).
    pub sample_rate: u32,
    /// True when the host selected alternate setting 1 for the speaker interface.
    pub streaming_active: bool,
    /// True when the host selected alternate setting 1 for the capture interface.
    pub capture_active: bool,
}

impl AudioClass {
    fn request_targets_ac_interface(&self, req: &Request) -> bool {
        (req.index as u8) == u8::from(self.ac_iface)
    }

    /// Allocate USB interfaces and ISO OUT (speaker) + ISO IN (mic) endpoints
    /// for the UAC2 device.
    ///
    /// The speaker endpoint uses **asynchronous** synchronisation with the mic
    /// ISO IN endpoint as the implicit feedback source: the host observes the
    /// IN packet rate to determine how many samples the device consumes per
    /// SOF interval and adjusts the OUT data rate accordingly.
    ///
    /// `iso_packet_size` — max bytes per isochronous packet.  288 covers
    /// both 44.1 kHz (264 B) and 48 kHz (288 B) at stereo 24-bit.
    ///
    /// Returns `(handler, ep_out, ep_in)`.
    pub fn new<'d, D: Driver<'d>>(
        builder: &mut Builder<'d, D>,
        iso_packet_size: u16,
    ) -> (Self, D::EndpointOut, D::EndpointIn)
    where
        D::EndpointIn: Endpoint,
    {
        let mut func = builder.function(USB_CLASS_AUDIO, 0x00, USB_PROTOCOL_IP_02_00);

        // ── AudioControl interface ──────────────────────────────────────────
        // wTotalLength: AC Header(9) + Clock Source(8) +
        //   Speaker Input Term(17) + Speaker Output Term(12) +
        //   Mic Input Term(17) + Mic Output Term(12) = 75
        let mut ac_ib = func.interface();
        let ac_iface = ac_ib.interface_number();
        let mut ac_alt = ac_ib.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_CONTROL,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        ac_alt.descriptor(CS_INTERFACE, &[AC_HEADER, 0x00, 0x02, 0x08, 75, 0x00, 0x00]);
        // Clock Source (entity 0x10, internal fixed, SAM_FREQ read-only)
        ac_alt.descriptor(
            CS_INTERFACE,
            &[AC_CLOCK_SOURCE, CLOCK_SOURCE_ID, 0b01, 0b01, 0x00, 0x00],
        );
        // Input Terminal: USB streaming → speaker (entity 0x11)
        ac_alt.descriptor(
            CS_INTERFACE,
            &[
                AC_INPUT_TERM,
                INPUT_TERM_ID,
                (TERM_USB_STREAMING & 0xFF) as u8,
                (TERM_USB_STREAMING >> 8) as u8,
                0x00,
                CLOCK_SOURCE_ID,
                0x02,
                0x03,
                0x00,
                0x00,
                0x00, // 2ch FL+FR
                0x00,
                0x00,
                0x00,
                0x00,
            ],
        );
        // Output Terminal: speaker (entity 0x12)
        ac_alt.descriptor(
            CS_INTERFACE,
            &[
                AC_OUTPUT_TERM,
                OUTPUT_TERM_ID,
                (TERM_SPEAKER & 0xFF) as u8,
                (TERM_SPEAKER >> 8) as u8,
                0x00,
                INPUT_TERM_ID,
                CLOCK_SOURCE_ID,
                0x00,
                0x00,
                0x00,
            ],
        );
        // Input Terminal: physical microphone (entity 0x13)
        ac_alt.descriptor(
            CS_INTERFACE,
            &[
                AC_INPUT_TERM,
                MIC_INPUT_TERM_ID,
                (TERM_MICROPHONE & 0xFF) as u8,
                (TERM_MICROPHONE >> 8) as u8,
                0x00,
                CLOCK_SOURCE_ID,
                0x02,
                0x03,
                0x00,
                0x00,
                0x00, // 2ch FL+FR
                0x00,
                0x00,
                0x00,
                0x00,
            ],
        );
        // Output Terminal: USB streaming ← mic (entity 0x14)
        ac_alt.descriptor(
            CS_INTERFACE,
            &[
                AC_OUTPUT_TERM,
                MIC_OUTPUT_TERM_ID,
                (TERM_USB_STREAMING & 0xFF) as u8,
                (TERM_USB_STREAMING >> 8) as u8,
                0x00,
                MIC_INPUT_TERM_ID,
                CLOCK_SOURCE_ID,
                0x00,
                0x00,
                0x00,
            ],
        );

        // ── AudioStreaming Capture interface (mic ISO IN) ────────────────────
        // Allocated BEFORE the speaker interface so we know ep_in.addr to use
        // as bSynchAddress in the speaker ISO OUT endpoint.
        let mut as_cap_ib = func.interface();
        let as_capture_iface = as_cap_ib.interface_number();
        // Alt 0: zero-bandwidth
        let _ = as_cap_ib.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_STREAMING,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        // Alt 1: 16-bit ISO IN (mic → host)
        let mut as_cap_alt1 = as_cap_ib.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_STREAMING,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        as_cap_alt1.descriptor(
            CS_INTERFACE,
            &[
                AS_GENERAL,
                MIC_OUTPUT_TERM_ID,
                0x00,
                0x01,
                0x01,
                0x00,
                0x00,
                0x00, // PCM format
                0x02,
                0x03,
                0x00,
                0x00,
                0x00,
                0x00, // 2ch FL+FR
            ],
        );
        as_cap_alt1.descriptor(CS_INTERFACE, &[AS_FORMAT_TYPE, 0x01, 0x02, 0x10]); // subslot=2, 16-bit
        let ep_in = as_cap_alt1.endpoint_isochronous_in(
            None,
            iso_packet_size, // MPS=288 so ep_in.info.max_packet_size covers alt2 too
            1,
            SynchronizationType::Asynchronous,
            UsageType::ImplicitFeedbackDataEndpoint,
            &[],
        );
        as_cap_alt1.descriptor(CS_ENDPOINT, &[0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);
        // Alt 2: 24-bit ISO IN — same endpoint address, larger MPS
        let mut as_cap_alt2 = as_cap_ib.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_STREAMING,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        as_cap_alt2.descriptor(
            CS_INTERFACE,
            &[
                AS_GENERAL,
                MIC_OUTPUT_TERM_ID,
                0x00,
                0x01,
                0x01,
                0x00,
                0x00,
                0x00, // PCM format
                0x02,
                0x03,
                0x00,
                0x00,
                0x00,
                0x00, // 2ch FL+FR
            ],
        );
        as_cap_alt2.descriptor(CS_INTERFACE, &[AS_FORMAT_TYPE, 0x01, 0x03, 0x18]); // subslot=3, 24-bit
        let _ep_in_24 = as_cap_alt2.endpoint_isochronous_in(
            Some(ep_in.info().addr), // reuses the same pipe as alt 1
            iso_packet_size,
            1,
            SynchronizationType::Asynchronous,
            UsageType::ImplicitFeedbackDataEndpoint,
            &[],
        );
        as_cap_alt2.descriptor(CS_ENDPOINT, &[0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);

        // ── AudioStreaming Speaker interface (ISO OUT) ───────────────────────
        let mut as_ib = func.interface();
        let as_iface = as_ib.interface_number();
        // Alt 0: zero-bandwidth
        let _ = as_ib.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_STREAMING,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        // Alt 1: 16-bit ISO OUT (host → speaker)
        let mut as_alt1 = as_ib.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_STREAMING,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        as_alt1.descriptor(
            CS_INTERFACE,
            &[
                AS_GENERAL,
                INPUT_TERM_ID,
                0x00,
                0x01,
                0x01,
                0x00,
                0x00,
                0x00, // PCM format
                0x02,
                0x03,
                0x00,
                0x00,
                0x00,
                0x00, // 2ch FL+FR
            ],
        );
        as_alt1.descriptor(CS_INTERFACE, &[AS_FORMAT_TYPE, 0x01, 0x02, 0x10]); // subslot=2, 16-bit
        let ep_out = as_alt1.endpoint_isochronous_out(
            Some(ep_in.info().addr), // bSynchAddress = mic IN (implicit feedback)
            iso_packet_size,         // MPS=288 so ep_out.info.max_packet_size covers alt2 too
            1,
            SynchronizationType::Asynchronous,
            UsageType::DataEndpoint,
            &[],
        );
        as_alt1.descriptor(CS_ENDPOINT, &[0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);
        // Alt 2: 24-bit ISO OUT — same endpoint address, larger MPS
        let mut as_alt2 = as_ib.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_STREAMING,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        as_alt2.descriptor(
            CS_INTERFACE,
            &[
                AS_GENERAL,
                INPUT_TERM_ID,
                0x00,
                0x01,
                0x01,
                0x00,
                0x00,
                0x00, // PCM format
                0x02,
                0x03,
                0x00,
                0x00,
                0x00,
                0x00, // 2ch FL+FR
            ],
        );
        as_alt2.descriptor(CS_INTERFACE, &[AS_FORMAT_TYPE, 0x01, 0x03, 0x18]); // subslot=3, 24-bit
        let _ep_out_24 = as_alt2.endpoint_isochronous_out(
            Some(ep_out.info().addr), // reuse same OUT pipe as alt 1
            iso_packet_size,
            1,
            SynchronizationType::Asynchronous,
            UsageType::DataEndpoint,
            &[],
        );
        as_alt2.descriptor(CS_ENDPOINT, &[0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);

        (
            Self {
                ac_iface,
                as_iface,
                as_capture_iface,
                sample_rate: SAMPLE_RATE_44100,
                streaming_active: false,
                capture_active: false,
            },
            ep_out,
            ep_in,
        )
    }

    /// Current sample rate in Hz.
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// True when the host has selected the active streaming alternate setting.
    pub fn is_streaming(&self) -> bool {
        self.streaming_active
    }
}

impl Handler for AudioClass {
    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        // UAC2 SET_CUR — clock source SAM_FREQ_CONTROL
        if req.request_type == RequestType::Class
            && req.recipient == Recipient::Interface
            && req.request == UAC2_SET_CUR
            && self.request_targets_ac_interface(&req)
            && (req.index >> 8) as u8 == CLOCK_SOURCE_ID
            && (req.value >> 8) as u8 == SAM_FREQ_CS
        {
            if data.len() >= 4 {
                let hz = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                if hz == SAMPLE_RATE_44100 || hz == SAMPLE_RATE_48000 {
                    self.sample_rate = hz;
                }
            }
            return Some(OutResponse::Accepted);
        }
        None
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.request_type != RequestType::Class || req.recipient != Recipient::Interface {
            return None;
        }
        if !self.request_targets_ac_interface(&req) {
            return None;
        }
        if (req.index >> 8) as u8 != CLOCK_SOURCE_ID || (req.value >> 8) as u8 != SAM_FREQ_CS {
            return None;
        }
        match req.request {
            UAC2_GET_CUR => {
                // Return current sample rate (4-byte LE).
                let bytes = self.sample_rate.to_le_bytes();
                let len = buf.len().min(4);
                buf[..len].copy_from_slice(&bytes[..len]);
                Some(InResponse::Accepted(&buf[..len]))
            }
            UAC2_GET_RANGE => {
                // UAC2 §5.2.3.1: GET_RANGE response = wNumSubRanges followed by
                // (dMIN, dMAX, dRES) tuples for each supported frequency.
                // Only 44100 Hz is advertised — the SSI clock is derived from the
                // AUDIO_X1 crystal (22.5792 MHz) and cannot produce 48000 Hz.
                // Advertising both rates causes Linux snd-usb-audio to pick 48000 Hz
                // which overflows the 44100 Hz DMA buffer with a 3900 frame/s surplus.
                let resp: [u8; 14] = {
                    let mut b = [0u8; 14];
                    b[0..2].copy_from_slice(&1u16.to_le_bytes()); // wNumSubRanges = 1
                    b[2..6].copy_from_slice(&44_100u32.to_le_bytes()); // dMIN
                    b[6..10].copy_from_slice(&44_100u32.to_le_bytes()); // dMAX
                    b[10..14].copy_from_slice(&0u32.to_le_bytes()); // dRES
                    b
                };
                let len = buf.len().min(resp.len());
                buf[..len].copy_from_slice(&resp[..len]);
                Some(InResponse::Accepted(&buf[..len]))
            }
            _ => None,
        }
    }

    fn set_alternate_setting(&mut self, iface: InterfaceNumber, alternate: u8) {
        if iface == self.as_iface {
            log::info!("audio: speaker SET_INTERFACE alt={}", alternate);
            self.streaming_active = alternate >= 1;
            // alt 1 = 16-bit PCM, alt 2 = 24-bit PCM
            let bits: u8 = if alternate == 1 { 16 } else { 24 };
            USB_BITS_PER_SAMPLE.store(bits, Ordering::Relaxed);
        } else if iface == self.as_capture_iface {
            log::info!("audio: capture SET_INTERFACE alt={}", alternate);
            self.capture_active = alternate >= 1;
            let bits: u8 = if alternate == 1 { 16 } else { 24 };
            USB_CAPTURE_BITS_PER_SAMPLE.store(bits, Ordering::Relaxed);
        }
    }

    fn reset(&mut self) {
        self.streaming_active = false;
        self.capture_active = false;
    }
}
