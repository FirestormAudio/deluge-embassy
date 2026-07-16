//! USB **host**-side class drivers.
//!
//! The device-side classes live in [`super::classes`]. This module is the
//! mirror image: drivers for devices the Deluge *hosts*, built on
//! `embassy-usb-host` and the RUSB1 host driver in [`rza1l_hal::usb`].
//!
//! Class drivers here are generic over [`UsbHostAllocator`], so they are unit
//! tested against [`mock::MockAlloc`] under QEMU — see `tools/test.sh`.
//!
//! [`UsbHostAllocator`]: embassy_usb_driver::host::UsbHostAllocator

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

pub mod midi;

#[cfg(all(test, not(target_os = "none")))]
pub(crate) mod mock;

/// Maximum concurrently hosted MIDI devices.
///
/// Bounded by the RUSB1 host pipe budget. RX pipes are dedicated per device
/// (USB is host-polled, so a receive must be armed to catch unsolicited MIDI),
/// while TX shares one pipe once OUT-pipe multiplexing lands in the HAL. With
/// bulk pipe 1 reserved for shared TX, bulk RX has pipes 2-5 — **4 devices**.
///
/// The original firmware reaches 6 by also servicing MIDI devices that use
/// *interrupt* endpoints on RX pipes 7-8 (`USB_CFG_HMIDI_INT_RECV_MIN/MAX`).
/// [`midi::find_midi_interface`] only claims bulk endpoints, so that last 2 is
/// not available here; interrupt-endpoint MIDI is a documented follow-up.
///
/// Until OUT-pipe multiplexing lands, TX also consumes a dedicated pipe, so the
/// practical ceiling is lower still (~2 bidirectional devices).
pub const MAX_MIDI_DEVICES: usize = 4;

/// Depth of the merged RX channel, in event packets.
const MIDI_RX_DEPTH: usize = 64;

/// Depth of each per-device TX channel, in event packets.
const MIDI_TX_DEPTH: usize = 16;

/// Number of device slots. Addresses run 1..=5 (`HCD_MAX_DEV` is 6), so index
/// by address directly and size for the widest address the HAL allows.
const MIDI_SLOTS: usize = 6;

/// Identifies a hosted device.
///
/// Not the raw USB address: addresses are freed and reused on hot-plug, so a
/// bare address could alias a different device. The generation counter makes
/// stale ids detectable.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DeviceId {
    slot: u8,
    generation: u16,
}

impl DeviceId {
    pub(crate) const fn new(slot: u8, generation: u16) -> Self {
        Self { slot, generation }
    }

    /// Device slot index (the USB address, `1..MIDI_SLOTS`).
    pub const fn slot(&self) -> u8 {
        self.slot
    }

    /// Generation counter — bumped each time a slot is reused.
    pub const fn generation(&self) -> u16 {
        self.generation
    }
}

/// Merged RX stream from every hosted MIDI device, tagged by source.
static MIDI_RX: Channel<CriticalSectionRawMutex, (DeviceId, [u8; 4]), MIDI_RX_DEPTH> =
    Channel::new();

/// Per-device TX queues, indexed by [`DeviceId::slot`].
///
/// `MidiHost` is owned by its device task and cannot be shared, so this channel
/// is the only route to a device's OUT endpoint.
static MIDI_TX: [Channel<CriticalSectionRawMutex, [u8; 4], MIDI_TX_DEPTH>; MIDI_SLOTS] =
    [const { Channel::new() }; MIDI_SLOTS];

/// Non-blocking read of the next hosted-MIDI packet, if any.
///
/// Mirrors the device-side idiom (`usb::classes::midi::try_recv_from_host`).
pub fn try_recv_midi() -> Option<(DeviceId, [u8; 4])> {
    MIDI_RX.try_receive().ok()
}

/// Await the next hosted-MIDI packet.
pub async fn recv_midi() -> (DeviceId, [u8; 4]) {
    MIDI_RX.receive().await
}

/// A send handle for one hosted device.
///
/// Obtained from [`midi_handle`]. Cheap and `Copy` — it is just an id.
#[derive(Clone, Copy, Debug)]
pub struct MidiHandle {
    id: DeviceId,
}

impl MidiHandle {
    /// The device this handle addresses.
    pub const fn id(&self) -> DeviceId {
        self.id
    }

    /// Queue a packet for the device, awaiting space if the queue is full.
    pub async fn send(&self, packet: [u8; 4]) {
        MIDI_TX[self.id.slot() as usize].send(packet).await;
    }

    /// Queue a packet, returning it if the queue is full.
    ///
    /// Prefer this on the audio path — never block audio on USB.
    pub fn try_send(&self, packet: [u8; 4]) -> Result<(), [u8; 4]> {
        MIDI_TX[self.id.slot() as usize]
            .try_send(packet)
            .map_err(|e| match e {
                embassy_sync::channel::TrySendError::Full(p) => p,
            })
    }
}

/// A send handle for `id`.
///
/// The handle does not verify the device is still attached: if it has detached,
/// packets queue and are drained when the slot is reused. Compare
/// [`DeviceId::generation`] against a freshly observed id to detect staleness.
pub const fn midi_handle(id: DeviceId) -> MidiHandle {
    MidiHandle { id }
}

#[cfg(target_os = "none")]
mod runtime {
    use embassy_executor::Spawner;
    use embassy_futures::select::{select, Either};
    use embassy_usb_driver::host::DeviceEvent;
    use embassy_usb_host::descriptor::ConfigurationDescriptor;
    use embassy_usb_host::handler::BusRoute;
    use embassy_usb_host::{bus, BusHandle, BusState};
    use log::{error, info, warn};
    use rza1l_hal::usb::{Rusb1Allocator, Rusb1HostDriver};

    use super::{DeviceId, MAX_MIDI_DEVICES, MIDI_RX, MIDI_TX};
    use crate::usb::host::midi::MidiHost;

    /// The allocator the class drivers are actually parameterised by.
    ///
    /// `BusHandle` implements `UsbHostAllocator` by forwarding to the inner
    /// `Rusb1Allocator`, and it is what `bus()` hands back — so this, not
    /// `Rusb1Allocator`, is the type that flows into the class drivers.
    type HostAlloc = BusHandle<'static, Rusb1Allocator>;

    /// Services one hosted MIDI device until it errors or detaches.
    ///
    /// Owns the pipes, so they drop — and the hardware pipes free — the moment
    /// this exits. That is the whole detach cleanup path.
    ///
    /// Reads and writes are serviced together via `select`: the device's TX
    /// channel is the only route to its OUT endpoint, since `MidiHost` lives
    /// here and cannot be shared.
    #[embassy_executor::task(pool_size = MAX_MIDI_DEVICES)]
    async fn midi_device_task(mut host: MidiHost<'static, HostAlloc>, id: DeviceId) {
        let tx = &MIDI_TX[id.slot() as usize];
        let mut packets = [[0u8; 4]; 16];

        loop {
            if !host.can_receive() {
                // Sink-only device (a synth): only ever send.
                let p = tx.receive().await;
                if let Err(e) = host.write(&[p]).await {
                    warn!("usb_host: device {:?} write error: {:?}", id, e);
                }
                continue;
            }

            match select(host.read(&mut packets), tx.receive()).await {
                Either::First(Ok(n)) => {
                    for p in &packets[..n] {
                        // Drop rather than block: a stalled consumer must not
                        // stall USB.
                        let _ = MIDI_RX.try_send((id, *p));
                    }
                }
                Either::First(Err(e)) => {
                    info!("usb_host: device {:?} read ended: {:?}", id, e);
                    return;
                }
                Either::Second(p) => {
                    if let Err(e) = host.write(&[p]).await {
                        // A send error is normal when a device attaches or
                        // detaches from a hub mid-traffic — the original
                        // firmware logs and continues rather than dropping the
                        // device (midi_engine.cpp:128-135). Do the same.
                        warn!("usb_host: device {:?} write error: {:?}", id, e);
                    }
                }
            }
        }
    }

    /// Owns the bus: waits for attach, enumerates, binds a class driver.
    ///
    /// Spawn this instead of a bare enumerate-and-log task when the port is in
    /// host mode. The `driver` must have been initialised via
    /// [`rza1l_hal::usb::init_host_mode`] and the ISR wired to
    /// [`rza1l_hal::usb::hcd_int_handler`].
    #[embassy_executor::task]
    pub async fn usb_host_supervisor(driver: Rusb1HostDriver, spawner: Spawner) {
        static BUS_STATE: BusState = BusState::new();
        let (mut controller, handle) = bus(driver, &BUS_STATE);
        let mut config_buf = [0u8; 512];
        let mut generation: u16 = 0;

        loop {
            let speed = controller.wait_for_connection().await;
            info!("usb_host: device connected ({:?})", speed);

            let (dev_info, _) = match handle
                .enumerate(BusRoute::Direct(speed), &mut config_buf)
                .await
            {
                Ok(v) => v,
                Err(e) => {
                    error!("usb_host: enumeration failed: {:?}", e);
                    continue;
                }
            };
            let addr = dev_info.device_address;

            match ConfigurationDescriptor::try_from_slice(&config_buf) {
                Ok(cfg) => {
                    generation = generation.wrapping_add(1);
                    let id = DeviceId::new(addr, generation);

                    match MidiHost::try_register(&handle, addr, dev_info.split(), &cfg) {
                        Ok(host) => {
                            info!(
                                "usb_host: MIDI device VID={:04x} PID={:04x} addr={}",
                                dev_info.device_desc.vendor_id,
                                dev_info.device_desc.product_id,
                                addr
                            );
                            // In embassy-executor 0.10 the task fn returns the
                            // token as a Result (pool exhaustion), while
                            // `Spawner::spawn` itself returns (). Reaching the
                            // Err arm means more devices than MAX_MIDI_DEVICES.
                            match midi_device_task(host, id) {
                                Ok(token) => spawner.spawn(token),
                                Err(_) => {
                                    error!("usb_host: no free device task slot");
                                    handle.free_address(addr);
                                    continue;
                                }
                            }
                        }
                        Err(e) => {
                            // Free the address immediately so an unsupported
                            // device does not consume one of the scarce slots.
                            warn!("usb_host: unsupported device: {:?}", e);
                            handle.free_address(addr);
                            continue;
                        }
                    }
                }
                Err(e) => {
                    error!("usb_host: bad config descriptor: {:?}", e);
                    handle.free_address(addr);
                    continue;
                }
            }

            // Wait for detach before accepting another device.
            loop {
                if controller.wait_for_device_event().await == DeviceEvent::Disconnected {
                    info!("usb_host: device disconnected");
                    handle.free_address(addr);
                    break;
                }
            }
        }
    }
}

#[cfg(target_os = "none")]
pub use runtime::usb_host_supervisor;

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn device_ids_in_the_same_slot_differ_across_generations() {
        // A USB address is freed and reused on hot-plug. If DeviceId were just
        // the address, a stale id would silently alias a different controller.
        let a = DeviceId::new(2, 7);
        let b = DeviceId::new(2, 8);
        assert_ne!(a, b, "same slot, later generation must not compare equal");
        assert_eq!(a.slot(), b.slot());
    }

    #[test]
    fn device_ids_are_equal_within_a_generation() {
        assert_eq!(DeviceId::new(3, 1), DeviceId::new(3, 1));
    }

    #[test]
    fn handle_try_send_routes_to_the_devices_own_queue() {
        let h = midi_handle(DeviceId::new(2, 1));
        h.try_send([0x09, 0x90, 60, 100]).expect("queue has space");

        assert_eq!(
            MIDI_TX[2].try_receive().ok(),
            Some([0x09, 0x90, 60, 100]),
            "packet must land in slot 2's queue"
        );
        assert!(
            MIDI_TX[3].try_receive().is_err(),
            "no other device's queue may be touched"
        );
    }

    #[test]
    fn handle_try_send_reports_a_full_queue_instead_of_blocking() {
        let h = midi_handle(DeviceId::new(4, 1));
        for _ in 0..MIDI_TX_DEPTH {
            h.try_send([0x0F, 0xF8, 0, 0]).expect("fills to capacity");
        }
        // The audio path must never block on USB backpressure.
        assert_eq!(h.try_send([0x0F, 0xF8, 0, 0]), Err([0x0F, 0xF8, 0, 0]));
    }
}
