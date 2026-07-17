//! USB **host**-side USB Audio Class (UAC2) capture driver.
//!
//! Records multichannel audio from a class-compliant USB audio interface.
//! Mirrors [`super::midi`]: generic over the allocator so it unit-tests against
//! [`super::mock`]. See the design of record,
//! `docs/superpowers/specs/2026-07-16-usb-uac-host-design.md`.
//!
//! Playback (iso OUT) is Phase 2 (`out.rs`) and not present yet.

pub mod resample;
pub mod ring;

/// Widest capture channel count this driver supports. Bounds the fixed-size
/// per-frame scratch and the ring; a device declaring more is rejected.
pub const MAX_CHANNELS: usize = 8;

use embassy_usb_driver::Direction;
use embassy_usb_driver::host::{HostError, PipeError};
use embassy_usb_host::class::uac::descriptors::{
    AudioInterfaceCollection, FormatTypeDescriptor, TerminalDescriptor,
};
use embassy_usb_host::descriptor::{ConfigurationDescriptor, EndpointDescriptor};

/// Errors from the host UAC capture driver.
#[derive(Debug)]
pub enum UacError {
    /// No UAC2 audio-streaming interface with an input (capture) endpoint.
    NoInputInterface,
    /// Found capture, but not Type-I 24-bit PCM (our only supported format).
    UnsupportedFormat,
    /// The device cannot run 44.1 kHz.
    UnsupportedRate,
    /// A required pipe could not be allocated.
    NoPipe(HostError),
    /// A control or data transfer failed.
    Transfer(PipeError),
    /// The device's descriptors could not be parsed as UAC2.
    BadDescriptors,
}

impl From<PipeError> for UacError {
    fn from(e: PipeError) -> Self {
        UacError::Transfer(e)
    }
}

/// A matched UAC2 capture interface and the facts needed to negotiate + stream.
#[derive(Clone, Copy, Debug)]
pub struct UacCaptureMatch {
    /// AudioControl interface number (`wIndex` low byte for clock requests).
    pub ac_interface: u8,
    /// AudioStreaming interface number (target of SET_INTERFACE).
    pub streaming_interface: u8,
    /// Alternate setting carrying the stream (SET_INTERFACE `wValue`).
    pub alternate_setting: u8,
    /// The isochronous IN data endpoint.
    pub endpoint: EndpointDescriptor,
    /// Channels the device declares (app-visible; no stereo assumption).
    pub num_channels: u8,
    /// Clock-source entity ID (target of the sampling-frequency control).
    pub clock_source_id: u8,
}

/// Find the first usable UAC2 capture interface in `cfg`.
///
/// Requires Type-I PCM with `subslot_size == 3` (24-bit). Returns
/// [`UacError::NoInputInterface`] / [`UacError::UnsupportedFormat`] otherwise.
pub fn find_uac_capture(cfg: &ConfigurationDescriptor<'_>) -> Result<UacCaptureMatch, UacError> {
    let coll =
        AudioInterfaceCollection::try_from_configuration(cfg).map_err(|_| UacError::BadDescriptors)?;
    let ac_interface = coll.control_interface.interface_descriptors[0].interface_number;

    for asi in coll.audio_streaming_interfaces.iter() {
        let ep = match asi.endpoint_descriptor {
            Some(ep) if ep.ep_dir() == Direction::In => ep,
            _ => continue,
        };
        // 24-bit Type-I PCM only. Match by reference: `FormatTypeDescriptor` is
        // not `Copy`, so matching by value would move out of the borrowed `asi`.
        match &asi.format_type_descriptor {
            Some(FormatTypeDescriptor::I(f)) if f.subslot_size == 3 => {}
            _ => return Err(UacError::UnsupportedFormat),
        }
        // The streaming alt setting carrying the endpoints (max endpoint count).
        let alt = asi
            .interface_descriptors
            .iter()
            .max_by_key(|i| i.num_endpoints)
            .ok_or(UacError::BadDescriptors)?;
        // Clock source feeding the input terminal linked to this stream.
        let clock_source_id = coll
            .control_interface
            .terminal_descriptors
            .values()
            .find_map(|t| match t {
                TerminalDescriptor::Input(_) => Some(t.clock_source_id()),
                _ => None,
            })
            .ok_or(UacError::BadDescriptors)?;

        return Ok(UacCaptureMatch {
            ac_interface,
            streaming_interface: alt.interface_number,
            alternate_setting: alt.alternate_setting,
            endpoint: ep,
            num_channels: asi.class_descriptor.num_channels,
            clock_source_id,
        });
    }
    Err(UacError::NoInputInterface)
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    use heapless::Vec;

    // --- UAC2 descriptor fixture bytes ---
    // Class/subclass/protocol: AUDIO=0x01, AUDIOCONTROL=0x01, AUDIOSTREAMING=0x02,
    // protocol IP_VERSION_02_00=0x20. CS_INTERFACE=0x24. subslot_size at the
    // FORMAT_TYPE descriptor; num_channels in the AS general descriptor.

    fn uac2_mic_cfg(subslot: u8, channels: u8) -> Vec<u8, 256> {
        let mut b: Vec<u8, 256> = Vec::new();
        let push = |s: &[u8], b: &mut Vec<u8, 256>| b.extend_from_slice(s).unwrap();

        // IAD: first iface 0, count 2, AUDIO/undefined/UAC2.
        push(&[8, 0x0B, 0, 2, 0x01, 0x00, 0x20, 0], &mut b);
        // Std AC interface 0, alt 0, 0 eps, AUDIO/AUDIOCONTROL/UAC2.
        push(&[9, 0x04, 0, 0, 0, 0x01, 0x01, 0x20, 0], &mut b);
        // CS AC header (UAC2 CLOCK etc. are inside; length total little-endian).
        push(&[9, 0x24, 0x01, 0x00, 0x02, 30, 0, 0x00, 0x00], &mut b);
        // CS Clock Source: subtype 0x0A, clock id 0x09, attrs, controls, assoc.
        push(&[8, 0x24, 0x0A, 0x09, 0x01, 0x07, 0x00, 0x00], &mut b);
        // CS Input Terminal (subtype 0x02): term id 0x01, type MIC 0x0201,
        // assoc 0, clock source id 0x09, nrchannels, ...
        push(&[17, 0x24, 0x02, 0x01, 0x01, 0x02, 0, 0x09, channels, 0, 0, 0, 0, 0, 0, 0, 0], &mut b);
        // Std AS interface 1, alt 0 (zero bandwidth): 0 eps.
        push(&[9, 0x04, 1, 0, 0, 0x01, 0x02, 0x20, 0], &mut b);
        // Std AS interface 1, alt 1: 1 ep, AUDIO/AUDIOSTREAMING/UAC2.
        push(&[9, 0x04, 1, 1, 1, 0x01, 0x02, 0x20, 0], &mut b);
        // CS AS general (subtype 0x01): terminal link 0x01, controls, FORMAT_TYPE_I=1,
        // formats bitmap(4), nrchannels, channel config(4), name.
        push(&[16, 0x24, 0x01, 0x01, 0x00, 0x01, 0x01, 0, 0, 0, channels, 0, 0, 0, 0, 0], &mut b);
        // CS AS FORMAT_TYPE (subtype 0x02): FORMAT_TYPE_I=1, subslot, bit_res.
        push(&[6, 0x24, 0x02, 0x01, subslot, subslot * 8], &mut b);
        // Std iso IN endpoint 0x81, iso(0x01) async, mps 294, interval 4.
        push(&[7, 0x05, 0x81, 0x01, 0x26, 0x01, 0x04], &mut b);
        // CS AS iso endpoint (0x25): general, attrs, controls, lock delay units/val.
        push(&[8, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00], &mut b);

        // Configuration header: len 9, CONFIGURATION=0x02, total_len LE, 2 ifaces.
        let total = (9 + b.len()) as u16;
        let mut cfg: Vec<u8, 256> = Vec::new();
        cfg.extend_from_slice(&[
            9, 0x02, total as u8, (total >> 8) as u8, 2, 1, 0, 0x80, 50,
        ])
        .unwrap();
        cfg.extend_from_slice(&b).unwrap();
        cfg
    }

    #[test]
    fn matches_24bit_capture_interface() {
        let raw = uac2_mic_cfg(3, 2);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let m = find_uac_capture(&cfg).expect("should match a 24-bit UAC2 mic");
        assert_eq!(m.streaming_interface, 1);
        assert_eq!(m.alternate_setting, 1);
        assert_eq!(m.num_channels, 2);
        assert_eq!(m.clock_source_id, 0x09);
        assert_eq!(m.ac_interface, 0);
        assert_eq!(m.endpoint.endpoint_address, 0x81);
        assert_eq!(m.endpoint.ep_dir(), Direction::In);
    }

    #[test]
    fn rejects_non_24bit_format() {
        let raw = uac2_mic_cfg(2, 2); // 16-bit
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        assert!(matches!(find_uac_capture(&cfg), Err(UacError::UnsupportedFormat)));
    }
}
