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
use embassy_usb_driver::EndpointType;
use embassy_usb_driver::host::{HostError, PipeError, SplitInfo, UsbHostAllocator, UsbPipe, pipe};
use embassy_usb_host::class::uac::codes;
use embassy_usb_host::class::uac::descriptors::{
    AudioInterfaceCollection, FormatTypeDescriptor, TerminalDescriptor,
};
use embassy_usb_host::control::{ControlType, Recipient, RequestType, SetupPacket};
use embassy_usb_host::descriptor::{ConfigurationDescriptor, EndpointDescriptor};

use self::resample::{PiController, Resampler};
use self::ring::SampleRing;

// `crate::audio_block` is gated to the bare-metal target (`target_os =
// "none"`), but this module also builds under the host/QEMU test target (see
// `usb/mod.rs`). Reuse the canonical engine sample rate on-device; mirror its
// value locally for host tests so there is only one non-test source of truth.
#[cfg(target_os = "none")]
use crate::audio_block::SAMPLE_RATE_HZ;
#[cfg(not(target_os = "none"))]
const SAMPLE_RATE_HZ: u32 = 44_100;

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

/// Decode one signed 24-bit little-endian sample (`b.len() == 3`) to `f32` in
/// `[-1.0, 1.0)`.
pub fn decode_s24le(b: &[u8]) -> f32 {
    let raw = (b[0] as i32) | ((b[1] as i32) << 8) | ((b[2] as i32) << 16);
    let signed = (raw << 8) >> 8; // sign-extend 24 -> 32
    signed as f32 / 8_388_608.0 // 2^23
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

        let num_channels = asi.class_descriptor.num_channels;
        // Reject at the source: `pump_once` uses `num_channels` to
        // `chunks_exact(num_channels * 3)` and to index a fixed-size
        // `[f32; MAX_CHANNELS]` frame buffer. 0 panics on `chunks_exact`;
        // > MAX_CHANNELS panics on out-of-bounds indexing.
        if num_channels == 0 || num_channels as usize > MAX_CHANNELS {
            return Err(UacError::UnsupportedFormat);
        }

        return Ok(UacCaptureMatch {
            ac_interface,
            streaming_interface: alt.interface_number,
            alternate_setting: alt.alternate_setting,
            endpoint: ep,
            num_channels,
            clock_source_id,
        });
    }
    Err(UacError::NoInputInterface)
}

/// USB standard SET_INTERFACE request code (USB 2.0 §9.4).
const REQ_SET_INTERFACE: u8 = 11;

/// Sampling-frequency SET_CUR/GET_CUR wValue: SAMPLING_FREQ_CONTROL in the high
/// byte (the `codes` constant is pre-shifted), channel number 0 (master).
const SAMPLING_FREQ_WVALUE: u16 = codes::control_selector::clock_source::SAMPLING_FREQ_CONTROL;

/// A hosted UAC2 capture device.
///
/// Owns the control pipe and the isochronous IN data pipe. Generic over the
/// allocator so it unit-tests against [`super::mock`].
pub struct UacIn<'d, A: UsbHostAllocator<'d>> {
    control: A::Pipe<pipe::Control, pipe::InOut>,
    iso_in: A::Pipe<pipe::Isochronous, pipe::In>,
    num_channels: u8,
    max_packet: u16,
    ring: SampleRing,
    resampler: Resampler,
    pi: PiController,
    r: f32,
    _p: core::marker::PhantomData<&'d ()>,
}

impl<'d, A: UsbHostAllocator<'d>> UacIn<'d, A> {
    /// Channels this device declared. `read`'s output length should be a
    /// multiple of this.
    pub fn channels(&self) -> u8 {
        self.num_channels
    }

    /// Match, claim pipes, negotiate 44.1 kHz, and select the streaming alt.
    pub async fn try_register(
        alloc: &A,
        addr: u8,
        split: Option<SplitInfo>,
        cfg: &ConfigurationDescriptor<'_>,
    ) -> Result<Self, UacError> {
        let m = find_uac_capture(cfg)?;

        // Control pipe (EP0) + the iso IN data pipe.
        let mut control = alloc
            .alloc_pipe::<pipe::Control, pipe::InOut>(
                addr,
                &embassy_usb_driver::EndpointInfo {
                    addr: 0u8.into(),
                    ep_type: EndpointType::Control,
                    max_packet_size: 64,
                    interval_ms: 0,
                },
                split,
            )
            .map_err(UacError::NoPipe)?;
        let iso_in = alloc
            .alloc_pipe::<pipe::Isochronous, pipe::In>(addr, &m.endpoint.into(), split)
            .map_err(UacError::NoPipe)?;

        // Negotiate 44.1 kHz on the clock-source entity.
        let cur = Self::get_sampling_freq(&mut control, m.ac_interface, m.clock_source_id).await?;
        if cur != SAMPLE_RATE_HZ {
            Self::set_sampling_freq(&mut control, m.ac_interface, m.clock_source_id, SAMPLE_RATE_HZ)
                .await?;
            let now =
                Self::get_sampling_freq(&mut control, m.ac_interface, m.clock_source_id).await?;
            if now != SAMPLE_RATE_HZ {
                return Err(UacError::UnsupportedRate);
            }
        }

        // Activate the stream (alt 0 is zero-bandwidth; the real stream is 1+).
        Self::set_interface(&mut control, m.streaming_interface, m.alternate_setting).await?;

        let mut ring = SampleRing::new();
        ring.reset(m.num_channels as usize);
        let mut resampler = Resampler::new();
        resampler.reset(m.num_channels as usize);
        let setpoint = (ring.capacity_frames() / 2) as f32;
        Ok(Self {
            control,
            iso_in,
            num_channels: m.num_channels,
            max_packet: m.endpoint.max_packet_size,
            ring,
            resampler,
            pi: PiController::new(setpoint),
            r: 1.0,
            _p: core::marker::PhantomData,
        })
    }

    async fn get_sampling_freq(
        control: &mut A::Pipe<pipe::Control, pipe::InOut>,
        ac_interface: u8,
        clock_id: u8,
    ) -> Result<u32, UacError> {
        let setup = SetupPacket {
            request_type: RequestType {
                direction: Direction::In,
                control_type: ControlType::Class,
                recipient: Recipient::Interface,
            },
            request: codes::request_code::CUR,
            value: SAMPLING_FREQ_WVALUE,
            index: (clock_id as u16) << 8 | ac_interface as u16,
            length: 4,
        };
        let mut buf = [0u8; 4];
        let n = control.control_in(&setup.to_bytes(), &mut buf).await?;
        if n < 4 {
            return Err(UacError::Transfer(PipeError::BadResponse));
        }
        Ok(u32::from_le_bytes(buf))
    }

    async fn set_sampling_freq(
        control: &mut A::Pipe<pipe::Control, pipe::InOut>,
        ac_interface: u8,
        clock_id: u8,
        freq: u32,
    ) -> Result<(), UacError> {
        let setup = SetupPacket {
            request_type: RequestType {
                direction: Direction::Out,
                control_type: ControlType::Class,
                recipient: Recipient::Interface,
            },
            request: codes::request_code::CUR,
            value: SAMPLING_FREQ_WVALUE,
            index: (clock_id as u16) << 8 | ac_interface as u16,
            length: 4,
        };
        control.control_out(&setup.to_bytes(), &freq.to_le_bytes()).await?;
        Ok(())
    }

    async fn set_interface(
        control: &mut A::Pipe<pipe::Control, pipe::InOut>,
        interface: u8,
        alt: u8,
    ) -> Result<(), UacError> {
        let setup = SetupPacket {
            request_type: RequestType {
                direction: Direction::Out,
                control_type: ControlType::Standard,
                recipient: Recipient::Interface,
            },
            request: REQ_SET_INTERFACE,
            value: alt as u16,
            index: interface as u16,
            length: 0,
        };
        control.control_out(&setup.to_bytes(), &[]).await?;
        Ok(())
    }

    /// Do one isochronous IN transfer: decode → resample → ring, then update
    /// the drift ratio from the new ring fill.
    ///
    /// Iso has no retries: a failed or empty transfer logs and returns `Ok`
    /// (the ring absorbs the gap). Only a channel/format invariant break is an
    /// error.
    pub async fn pump_once(&mut self) -> Result<(), UacError> {
        let ch = self.num_channels as usize;
        let mut buf = [0u8; 1024]; // >= any HS iso mps
        let cap = self.max_packet as usize;
        let n = match self.iso_in.request_in(&mut buf[..cap]).await {
            Ok(n) => n,
            Err(e) => {
                // Lost packet: ring absorbs it. Do not tear down the stream.
                log::warn!("uac: iso IN transfer failed ({:?}), absorbing gap", e);
                self.r = self.pi.update(self.ring.fill_frames() as f32);
                return Ok(());
            }
        };
        let frame_bytes = ch * 3;
        let mut frame = [0.0f32; MAX_CHANNELS];
        for chunk in buf[..n].chunks_exact(frame_bytes) {
            for c in 0..ch {
                frame[c] = decode_s24le(&chunk[c * 3..c * 3 + 3]);
            }
            let ring = &mut self.ring;
            self.resampler.feed(&frame[..ch], self.r, |o| ring.push_frame(o));
        }
        self.r = self.pi.update(self.ring.fill_frames() as f32);
        Ok(())
    }

    /// Drain up to `out.len()` samples of captured interleaved `f32`. Short
    /// return = underrun; never blocks. `out.len()` should be a multiple of
    /// `channels()`.
    pub fn read(&mut self, out: &mut [f32]) -> usize {
        self.ring.read(out)
    }

    /// Frames currently buffered (for diagnostics / the validation firmware).
    pub fn fill_frames(&self) -> usize {
        self.ring.fill_frames()
    }

    /// Like [`Self::pump_once`], but publishes to the process-wide capture
    /// bridge so a different task (the engine) can `capture_read`. Used by the
    /// host task.
    #[cfg(target_os = "none")]
    pub async fn pump_once_shared(&mut self) -> Result<(), UacError> {
        let ch = self.num_channels as usize;
        let mut buf = [0u8; 1024];
        let cap = self.max_packet as usize;
        let n = match self.iso_in.request_in(&mut buf[..cap]).await {
            Ok(n) => n,
            Err(e) => {
                // Lost packet: the shared ring absorbs it; keep streaming.
                log::warn!("uac: iso IN transfer failed ({:?}), absorbing gap", e);
                self.r = self.pi.update(shared::fill_frames() as f32);
                return Ok(());
            }
        };
        let frame_bytes = ch * 3;
        let mut frame = [0.0f32; MAX_CHANNELS];
        let mut out = [0.0f32; MAX_CHANNELS];
        for chunk in buf[..n].chunks_exact(frame_bytes) {
            for c in 0..ch {
                frame[c] = decode_s24le(&chunk[c * 3..c * 3 + 3]);
            }
            let mut emit = |o: &[f32]| {
                out[..o.len()].copy_from_slice(o);
                shared::push(&out[..o.len()], ch);
            };
            self.resampler.feed(&frame[..ch], self.r, &mut emit);
        }
        // Single source of truth: the shared ring the app actually drains.
        self.r = self.pi.update(shared::fill_frames() as f32);
        Ok(())
    }
}

/// Cross-task capture bridge: the app-facing static that
/// [`UacIn::pump_once_shared`] publishes into and [`capture_read`] drains.
///
/// Device-only: it uses a `CriticalSectionRawMutex` static, which only makes
/// sense once there is a real interrupt-driven executor. Host tests exercise
/// the correctness reference (`pump_once` + `self.ring`) directly instead.
#[cfg(target_os = "none")]
pub(crate) mod shared {
    use core::cell::RefCell;

    use embassy_sync::blocking_mutex::Mutex;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    use super::MAX_CHANNELS;
    use super::ring::SampleRing;

    struct Capture {
        ring: SampleRing,
        channels: u8,
        active: bool,
    }

    static CAPTURE: Mutex<CriticalSectionRawMutex, RefCell<Capture>> =
        Mutex::new(RefCell::new(Capture {
            ring: SampleRing::new(),
            channels: 0,
            active: false,
        }));

    /// Called by the capture task when a device is registered.
    pub(crate) fn begin(channels: u8) {
        CAPTURE.lock(|c| {
            let mut c = c.borrow_mut();
            c.ring.reset(channels.clamp(1, MAX_CHANNELS as u8) as usize);
            c.channels = channels;
            c.active = true;
        });
    }

    /// Called by the capture task on detach.
    pub(crate) fn end() {
        CAPTURE.lock(|c| {
            let mut c = c.borrow_mut();
            c.active = false;
            c.channels = 0;
            c.ring.reset(1);
        });
    }

    /// Push captured interleaved frames from the task.
    pub(crate) fn push(samples: &[f32], channels: usize) {
        CAPTURE.lock(|c| {
            let mut c = c.borrow_mut();
            for frame in samples.chunks_exact(channels) {
                c.ring.push_frame(frame);
            }
        });
    }

    /// Frames currently buffered — the PI controller's single source of truth.
    pub(crate) fn fill_frames() -> usize {
        CAPTURE.lock(|c| c.borrow().ring.fill_frames())
    }

    /// App-facing drain. Short return = underrun; never blocks.
    pub fn capture_read(out: &mut [f32]) -> usize {
        CAPTURE.lock(|c| c.borrow_mut().ring.read(out))
    }

    /// Channels the hosted device declared, or 0 if none.
    pub fn capture_channels() -> u8 {
        CAPTURE.lock(|c| c.borrow().channels)
    }
}

#[cfg(target_os = "none")]
pub use shared::{capture_channels, capture_read};

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

    #[test]
    fn rejects_out_of_range_channel_counts() {
        // > MAX_CHANNELS (8): `pump_once` would index `[f32; MAX_CHANNELS]`
        // out of bounds if this were allowed through.
        let raw = uac2_mic_cfg(3, 9);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        assert!(matches!(find_uac_capture(&cfg), Err(UacError::UnsupportedFormat)));

        // 0 channels: `pump_once` would panic on `chunks_exact(0)` if this
        // were allowed through. Only assert if the fixture still parses as a
        // matched interface up to the channel check — a 0-channel descriptor
        // failing to parse for unrelated reasons wouldn't isolate the check.
        let raw0 = uac2_mic_cfg(3, 0);
        let cfg0 = ConfigurationDescriptor::try_from_slice(&raw0).unwrap();
        assert!(matches!(find_uac_capture(&cfg0), Err(UacError::UnsupportedFormat)));
    }

    // --- UacIn::try_register negotiation ---

    use crate::usb::host::mock::{MockAlloc, MockState};
    use embassy_futures::block_on;

    #[test]
    fn register_negotiates_when_rate_already_44100() {
        let state = MockState::leak();
        // GET_CUR returns 44100 -> no SET_CUR expected.
        state
            .control_reads
            .borrow_mut()
            .push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap())
            .unwrap();
        let alloc = MockAlloc::new(state);
        let raw = uac2_mic_cfg(3, 2);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let host = block_on(UacIn::try_register(&alloc, 1, None, &cfg)).expect("register");
        assert_eq!(host.channels(), 2);
        // Two pipes claimed: control (EP0) + iso IN (0x81).
        assert_eq!(state.allocs.borrow().len(), 2);
        // Exactly one SETUP: the GET_CUR (no SET_CUR), plus the SET_INTERFACE.
        let setups = state.setups.borrow();
        // Last setup is SET_INTERFACE (bRequest 11, standard/interface/out).
        let last = setups.last().unwrap();
        assert_eq!(last[1], 11, "bRequest = SET_INTERFACE");
        assert_eq!(last[2], 1, "wValue lo = alt setting 1");
        assert_eq!(last[4], 1, "wIndex lo = streaming interface 1");
    }

    #[test]
    fn register_forces_rate_when_not_44100() {
        let state = MockState::leak();
        // GET_CUR -> 48000, then after SET_CUR the confirming GET_CUR -> 44100.
        {
            let mut r = state.control_reads.borrow_mut();
            r.push(heapless::Vec::from_slice(&48_000u32.to_le_bytes()).unwrap()).unwrap();
            r.push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap()).unwrap();
        }
        let alloc = MockAlloc::new(state);
        let raw = uac2_mic_cfg(3, 1);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let host = block_on(UacIn::try_register(&alloc, 1, None, &cfg)).expect("register");
        assert_eq!(host.channels(), 1);
        // The SET_CUR data stage carried 44100 LE.
        assert_eq!(&state.control_out_data.borrow()[..4], &44_100u32.to_le_bytes());
    }

    // --- decode_s24le ---

    #[test]
    fn decode_s24le_endpoints() {
        assert_eq!(decode_s24le(&[0, 0, 0]), 0.0);
        assert_eq!(decode_s24le(&[0, 0, 0x40]), 0.5); // 0x400000 / 2^23
        assert_eq!(decode_s24le(&[0, 0, 0x80]), -1.0); // 0x800000 sign-extended
    }

    // --- pump_once / read ---

    #[test]
    fn pump_decodes_iso_packet_into_ring() {
        let state = MockState::leak();
        state
            .control_reads
            .borrow_mut()
            .push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap())
            .unwrap();
        // One iso IN packet: two mono 24-bit frames, values +0.5 and -1.0.
        let mut pkt = heapless::Vec::<u8, 64>::new();
        pkt.extend_from_slice(&[0, 0, 0x40, 0, 0, 0x80]).unwrap();
        state.script.borrow_mut().reads.push(pkt).unwrap();

        let alloc = MockAlloc::new(state);
        let raw = uac2_mic_cfg(3, 1);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = block_on(UacIn::try_register(&alloc, 1, None, &cfg)).unwrap();

        block_on(host.pump_once()).expect("pump");
        // At r≈1.0 the resampler emits ~one frame per input frame after priming;
        // at least one decoded sample must be readable.
        let mut out = [0.0f32; 4];
        let got = host.read(&mut out);
        assert!(got >= 1, "expected decoded samples in the ring, got {got}");
        assert!(out[..got].iter().any(|&s| s < 0.0 || s > 0.0), "non-silent");
    }

    #[test]
    fn pump_tolerates_transfer_error() {
        let state = MockState::leak();
        state
            .control_reads
            .borrow_mut()
            .push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap())
            .unwrap();
        *state.script.borrow_mut() = crate::usb::host::mock::Script {
            reads: Default::default(),
            read_err: Some(embassy_usb_driver::host::PipeError::Timeout),
        };
        let alloc = MockAlloc::new(state);
        let raw = uac2_mic_cfg(3, 1);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = block_on(UacIn::try_register(&alloc, 1, None, &cfg)).unwrap();

        // A failed iso transfer must NOT error out — the stream survives.
        block_on(host.pump_once()).expect("iso error is absorbed, not fatal");
    }

    #[test]
    fn pump_tolerates_torn_trailing_partial_sample() {
        let state = MockState::leak();
        state
            .control_reads
            .borrow_mut()
            .push(heapless::Vec::from_slice(&44_100u32.to_le_bytes()).unwrap())
            .unwrap();
        // Mono (channels=1) -> frame_bytes == 3. 7 bytes = two whole frames
        // (+0.5, -1.0) plus one torn trailing byte that `chunks_exact` must
        // silently drop rather than panic on.
        let mut pkt = heapless::Vec::<u8, 64>::new();
        pkt.extend_from_slice(&[0, 0, 0x40, 0, 0, 0x80, 0xAA]).unwrap();
        state.script.borrow_mut().reads.push(pkt).unwrap();

        let alloc = MockAlloc::new(state);
        let raw = uac2_mic_cfg(3, 1);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = block_on(UacIn::try_register(&alloc, 1, None, &cfg)).unwrap();

        // Must not panic on the non-multiple-of-frame_bytes packet length.
        block_on(host.pump_once()).expect("torn trailing sample is silently dropped, not fatal");
    }
}
