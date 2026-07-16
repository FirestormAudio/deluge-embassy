//! USB **host**-side MIDI 1.0 class driver.
//!
//! Matches class-compliant USB-MIDI 1.0 devices (Audio class, MIDIStreaming
//! subclass, alt setting 0) and exchanges 4-byte USB-MIDI event packets over
//! bulk endpoints.
//!
//! MIDI 2.0 / UMP is **not** supported here — see the device-side
//! [`crate::usb::classes::midi`] for that.

use embassy_usb_driver::{Direction, EndpointType};
use embassy_usb_host::descriptor::{ConfigurationDescriptor, EndpointDescriptor};

/// USB Audio class code (USB-MIDI lives under the Audio class).
const USB_CLASS_AUDIO: u8 = 0x01;
/// MIDIStreaming subclass code.
const USB_SUBCLASS_MIDI_STREAMING: u8 = 0x03;

/// A matched MIDIStreaming interface and its bulk endpoints.
///
/// At least one of `ep_in` / `ep_out` is always `Some` — a MIDIStreaming
/// interface with no bulk endpoints is unusable and does not match.
#[derive(Clone, Copy, Debug)]
pub struct MidiInterface {
    /// `bInterfaceNumber` of the MIDIStreaming interface.
    pub interface_number: u8,
    /// Bulk IN endpoint (device → host). `None` for a sink-only device.
    pub ep_in: Option<EndpointDescriptor>,
    /// Bulk OUT endpoint (host → device). `None` for a source-only device.
    pub ep_out: Option<EndpointDescriptor>,
}

/// Find the first usable MIDIStreaming interface in `cfg`.
///
/// Matches on the *interface* class/subclass rather than the device class, so
/// composite devices (e.g. an audio interface with a MIDI port) are handled.
/// Returns `None` if there is no MIDIStreaming interface, or if it declares no
/// bulk endpoints.
pub fn find_midi_interface(cfg: &ConfigurationDescriptor<'_>) -> Option<MidiInterface> {
    for iface in cfg.iter_interface() {
        if iface.interface_class != USB_CLASS_AUDIO
            || iface.interface_subclass != USB_SUBCLASS_MIDI_STREAMING
        {
            continue;
        }

        let mut ep_in = None;
        let mut ep_out = None;
        for ep in iface.iter_endpoints() {
            if ep.ep_type() != EndpointType::Bulk {
                continue;
            }
            match ep.ep_dir() {
                Direction::In if ep_in.is_none() => ep_in = Some(ep),
                Direction::Out if ep_out.is_none() => ep_out = Some(ep),
                _ => {}
            }
        }

        if ep_in.is_some() || ep_out.is_some() {
            return Some(MidiInterface {
                interface_number: iface.interface_number,
                ep_in,
                ep_out,
            });
        }
    }
    None
}

/// Split a bulk IN buffer into 4-byte USB-MIDI event packets.
///
/// USB-MIDI 1.0 (§4) frames every event as 4 bytes: byte 0 is
/// `(cable_number << 4) | code_index_number`, bytes 1-3 are the MIDI data.
/// Devices zero-pad the remainder of the buffer; CIN 0 is reserved, so a zero
/// header byte marks padding rather than an event.
///
/// Packets are yielded verbatim — cable number, CIN, and SysEx framing
/// (CIN 0x4-0x7) are all preserved for the caller to interpret.
pub fn decode_packets(buf: &[u8]) -> impl Iterator<Item = [u8; 4]> + '_ {
    buf.chunks_exact(4)
        .filter(|c| c[0] != 0)
        .map(|c| [c[0], c[1], c[2], c[3]])
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    /// Config descriptor: 9-byte config header + supplied body bytes.
    fn cfg_bytes(body: &[u8]) -> heapless::Vec<u8, 128> {
        let total = 9 + body.len();
        let mut v = heapless::Vec::new();
        v.extend_from_slice(&[9, 0x02, total as u8, 0, 1, 1, 0, 0x80, 50])
            .unwrap();
        v.extend_from_slice(body).unwrap();
        v
    }

    /// Audio/MIDIStreaming interface descriptor with `n` endpoints.
    fn ms_iface(n: u8) -> [u8; 9] {
        [9, 0x04, 0, 0, n, 0x01, 0x03, 0, 0]
    }

    /// Bulk endpoint descriptor. `addr` carries the 0x80 IN bit.
    fn bulk_ep(addr: u8) -> [u8; 7] {
        [7, 0x05, addr, 0x02, 64, 0, 0]
    }

    #[test]
    fn matches_bidirectional_device() {
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(2)).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        body.extend_from_slice(&bulk_ep(0x01)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let m = find_midi_interface(&cfg).expect("should match");
        assert_eq!(m.interface_number, 0);
        assert_eq!(m.ep_in.unwrap().endpoint_address, 0x81);
        assert_eq!(m.ep_out.unwrap().endpoint_address, 0x01);
    }

    #[test]
    fn matches_source_only_controller() {
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(1)).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let m = find_midi_interface(&cfg).expect("should match");
        assert_eq!(m.ep_in.unwrap().endpoint_address, 0x81);
        assert!(m.ep_out.is_none(), "controller has no OUT endpoint");
    }

    #[test]
    fn matches_sink_only_synth() {
        // A USB synth/sound module only receives MIDI: bulk OUT, no bulk IN.
        // This must match — the Deluge sequences external synths.
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(1)).unwrap();
        body.extend_from_slice(&bulk_ep(0x02)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let m = find_midi_interface(&cfg).expect("sink-only synth must match");
        assert!(m.ep_in.is_none());
        assert_eq!(m.ep_out.unwrap().endpoint_address, 0x02);
    }

    #[test]
    fn skips_audio_control_interface_and_finds_midi() {
        // Composite device: AudioControl (subclass 0x01) then MIDIStreaming.
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&[9, 0x04, 0, 0, 0, 0x01, 0x01, 0, 0])
            .unwrap();
        let mut ms = ms_iface(1);
        ms[2] = 1; // interface_number = 1
        body.extend_from_slice(&ms).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let m = find_midi_interface(&cfg).expect("should find the MS interface");
        assert_eq!(m.interface_number, 1);
    }

    #[test]
    fn rejects_non_midi_device() {
        // HID keyboard: class 0x03, nothing to do with MIDI.
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&[9, 0x04, 0, 0, 1, 0x03, 0x01, 1, 0])
            .unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        assert!(find_midi_interface(&cfg).is_none());
    }

    #[test]
    fn rejects_midi_interface_with_no_endpoints() {
        let raw = cfg_bytes(&ms_iface(0));
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        assert!(
            find_midi_interface(&cfg).is_none(),
            "no endpoints = unusable"
        );
    }

    // --- decode_packets ---

    #[test]
    fn decodes_a_single_note_on() {
        // Cable 0, CIN 0x9 (Note On), channel 0, note 60, velocity 100.
        let buf = [0x09, 0x90, 60, 100];
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0], [0x09, 0x90, 60, 100]);
    }

    #[test]
    fn decodes_multiple_packets() {
        let buf = [0x09, 0x90, 60, 100, 0x08, 0x80, 60, 0];
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out.len(), 2);
        assert_eq!(out[1], [0x08, 0x80, 60, 0]);
    }

    #[test]
    fn skips_zero_padding() {
        // Devices zero-pad the rest of the 64-byte buffer. CIN 0 is reserved,
        // so a zero header byte is never a real packet.
        let mut buf = [0u8; 64];
        buf[..4].copy_from_slice(&[0x09, 0x90, 60, 100]);
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out.len(), 1, "padding must not become packets");
    }

    #[test]
    fn preserves_cable_number() {
        // Cable 2, CIN 0xB (Control Change).
        let buf = [0x2B, 0xB0, 7, 127];
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out[0][0] >> 4, 2, "cable number must survive");
    }

    #[test]
    fn passes_sysex_packets_through_untouched() {
        // CIN 0x4 = SysEx start/continue, CIN 0x5 = SysEx end with 1 byte.
        let buf = [0x04, 0xF0, 0x7E, 0x00, 0x05, 0xF7, 0, 0];
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out.len(), 2);
        assert_eq!(out[0], [0x04, 0xF0, 0x7E, 0x00]);
        assert_eq!(out[1], [0x05, 0xF7, 0, 0]);
    }

    #[test]
    fn ignores_a_trailing_partial_packet() {
        let buf = [0x09, 0x90, 60, 100, 0x08, 0x80];
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out.len(), 1, "a 2-byte tail is not a packet");
    }

    #[test]
    fn empty_buffer_yields_nothing() {
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&[]).collect();
        assert!(out.is_empty());
    }
}
