//! USB **host**-side MIDI 1.0 class driver.
//!
//! Matches class-compliant USB-MIDI 1.0 devices (Audio class, MIDIStreaming
//! subclass, alt setting 0) and exchanges 4-byte USB-MIDI event packets over
//! bulk endpoints.
//!
//! MIDI 2.0 / UMP is **not** supported here — see the device-side
//! [`crate::usb::classes::midi`] for that.

use embassy_usb_driver::host::{HostError, PipeError, SplitInfo, UsbHostAllocator, UsbPipe, pipe};
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

/// Maximum bytes read from a bulk IN endpoint in one transfer.
///
/// 64 bytes = 16 USB-MIDI event packets, and matches the full-speed bulk MPS.
const RX_BUF_LEN: usize = 64;

/// Errors from the host MIDI class driver.
#[derive(Debug)]
pub enum MidiError {
    /// The device has no usable MIDIStreaming interface.
    NoMidiInterface,
    /// A required pipe could not be allocated (the device ceiling).
    NoPipe(HostError),
    /// A transfer failed.
    Transfer(PipeError),
    /// The operation is not supported by this device (e.g. TX on a
    /// source-only controller).
    NotSupported,
}

impl From<PipeError> for MidiError {
    fn from(e: PipeError) -> Self {
        Self::Transfer(e)
    }
}

/// A hosted USB MIDI 1.0 device.
///
/// Generic over the allocator so it can be unit-tested against a mock — see
/// [`super::mock`].
pub struct MidiHost<'d, A: UsbHostAllocator<'d>> {
    ep_in: Option<A::Pipe<pipe::Bulk, pipe::In>>,
    ep_out: Option<A::Pipe<pipe::Bulk, pipe::Out>>,
    _p: core::marker::PhantomData<&'d ()>,
}

impl<'d, A: UsbHostAllocator<'d>> MidiHost<'d, A> {
    /// Claim pipes for an enumerated device's MIDIStreaming interface.
    ///
    /// Claims whichever directions the device declares. A sink-only synth
    /// (bulk OUT only) and a source-only controller (bulk IN only) are both
    /// supported. Fails only if there is no MIDIStreaming interface, or if a
    /// declared endpoint's pipe cannot be allocated.
    ///
    /// MIDI 1.0 devices use alt setting 0, so no `SET_INTERFACE` is issued.
    pub fn try_register(
        alloc: &A,
        addr: u8,
        split: Option<SplitInfo>,
        cfg: &ConfigurationDescriptor<'_>,
    ) -> Result<Self, MidiError> {
        let iface = find_midi_interface(cfg).ok_or(MidiError::NoMidiInterface)?;

        let ep_in = match iface.ep_in {
            Some(ep) => Some(
                alloc
                    .alloc_pipe::<pipe::Bulk, pipe::In>(addr, &ep.into(), split)
                    .map_err(MidiError::NoPipe)?,
            ),
            None => None,
        };
        let ep_out = match iface.ep_out {
            Some(ep) => Some(
                alloc
                    .alloc_pipe::<pipe::Bulk, pipe::Out>(addr, &ep.into(), split)
                    .map_err(MidiError::NoPipe)?,
            ),
            None => None,
        };

        Ok(Self {
            ep_in,
            ep_out,
            _p: core::marker::PhantomData,
        })
    }

    /// True if this device accepts MIDI from the host.
    pub fn can_send(&self) -> bool {
        self.ep_out.is_some()
    }

    /// True if this device sends MIDI to the host.
    pub fn can_receive(&self) -> bool {
        self.ep_in.is_some()
    }

    /// Read up to `out.len()` event packets. Returns how many were decoded.
    ///
    /// Awaits the next bulk IN transfer, so this pends until the device sends
    /// something. Returns [`MidiError::NotSupported`] for a sink-only device.
    pub async fn read(&mut self, out: &mut [[u8; 4]]) -> Result<usize, MidiError> {
        let ep = self.ep_in.as_mut().ok_or(MidiError::NotSupported)?;
        let mut buf = [0u8; RX_BUF_LEN];
        let n = ep.request_in(&mut buf).await?;
        let mut count = 0;
        for pkt in decode_packets(&buf[..n]) {
            if count == out.len() {
                break;
            }
            out[count] = pkt;
            count += 1;
        }
        Ok(count)
    }

    /// Send event packets to the device.
    ///
    /// Returns [`MidiError::NotSupported`] for a source-only device.
    pub async fn write(&mut self, packets: &[[u8; 4]]) -> Result<(), MidiError> {
        let ep = self.ep_out.as_mut().ok_or(MidiError::NotSupported)?;
        let mut buf = [0u8; RX_BUF_LEN];
        let n = packets.len().min(RX_BUF_LEN / 4);
        for (i, p) in packets.iter().take(n).enumerate() {
            buf[i * 4..i * 4 + 4].copy_from_slice(p);
        }
        ep.request_out(&buf[..n * 4], false).await?;
        Ok(())
    }
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

    // --- MidiHost ---

    use crate::usb::host::mock::{MockAlloc, MockState};
    use embassy_usb_driver::host::HostError;

    fn bidir_cfg() -> heapless::Vec<u8, 128> {
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(2)).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        body.extend_from_slice(&bulk_ep(0x01)).unwrap();
        cfg_bytes(&body)
    }

    #[test]
    fn registers_bidirectional_device_and_claims_both_pipes() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let raw = bidir_cfg();
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let host = MidiHost::try_register(&alloc, 1, None, &cfg).expect("register");
        assert!(host.can_send());
        assert_eq!(state.allocs.borrow().len(), 2, "one IN pipe + one OUT pipe");
    }

    #[test]
    fn sink_only_device_registers_without_an_in_pipe() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(1)).unwrap();
        body.extend_from_slice(&bulk_ep(0x02)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let host = MidiHost::try_register(&alloc, 1, None, &cfg).expect("synth must register");
        assert!(host.can_send());
        assert!(!host.can_receive());
        assert_eq!(state.allocs.borrow().len(), 1, "OUT pipe only");
    }

    #[test]
    fn rejects_when_rx_pipe_is_unavailable() {
        let state = MockState::leak();
        *state.alloc_err.borrow_mut() = Some(HostError::OutOfPipes);
        let alloc = MockAlloc::new(state);
        let raw = bidir_cfg();
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let err = MidiHost::try_register(&alloc, 1, None, &cfg);
        assert!(matches!(err, Err(MidiError::NoPipe(_))));
    }

    #[test]
    fn rejects_non_midi_device_at_register() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&[9, 0x04, 0, 0, 1, 0x03, 0x01, 1, 0])
            .unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        assert!(matches!(
            MidiHost::try_register(&alloc, 1, None, &cfg),
            Err(MidiError::NoMidiInterface)
        ));
    }

    #[test]
    fn read_yields_scripted_packets() {
        let state = MockState::leak();
        let mut data = heapless::Vec::<u8, 64>::new();
        data.extend_from_slice(&[0x09, 0x90, 60, 100, 0x08, 0x80, 60, 0])
            .unwrap();
        state.script.borrow_mut().reads.push(data).unwrap();

        let alloc = MockAlloc::new(state);
        let raw = bidir_cfg();
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = MidiHost::try_register(&alloc, 1, None, &cfg).unwrap();

        let mut out = [[0u8; 4]; 16];
        let n = embassy_futures::block_on(host.read(&mut out)).expect("read");
        assert_eq!(n, 2);
        assert_eq!(out[0], [0x09, 0x90, 60, 100]);
        assert_eq!(out[1], [0x08, 0x80, 60, 0]);
    }

    #[test]
    fn write_sends_packet_bytes() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let raw = bidir_cfg();
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = MidiHost::try_register(&alloc, 1, None, &cfg).unwrap();

        embassy_futures::block_on(host.write(&[[0x09, 0x90, 64, 127]])).expect("write");
        assert_eq!(&state.log.borrow().sent[..], &[0x09, 0x90, 64, 127]);
    }

    #[test]
    fn write_to_source_only_device_is_not_supported() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(1)).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = MidiHost::try_register(&alloc, 1, None, &cfg).unwrap();

        assert!(!host.can_send());
        assert!(matches!(
            embassy_futures::block_on(host.write(&[[0x09, 0x90, 64, 127]])),
            Err(MidiError::NotSupported)
        ));
    }
}
