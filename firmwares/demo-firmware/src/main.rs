//! Deluge demo firmware image — built on the `deluge` SDK.
//!
//! This crate assembles the board support package into a runnable Embassy-based
//! firmware for the Deluge. Platform bring-up (RTT logging, heaps, clocks, the
//! executor, the panic handler) is provided by [`#[deluge::app]`](deluge::app);
//! this crate owns product-level behavior layered on top.
//!
//! ## Initialisation split
//! - [`setup`] runs first, with interrupts masked: peripheral and GIC bring-up
//!   that must happen before IRQs are enabled (UART, audio, CV/RSPI0, the USB
//!   ISR handler, encoder/trigger GPIO interrupts).
//! - `main` then runs on the executor with interrupts enabled: it builds the USB
//!   stack and spawns the product tasks.
//!
//! ## UI model
//! - Shared pad state lives in [`deluge_bsp::pads`].
//! - The PIC task consumes raw PIC events and updates that shared state.
//! - The encoder task consumes decoded detents from [`deluge_bsp::encoder`].
//! - Rendering tasks read shared state and stream the resulting frames to the
//!   OLED and RGB surfaces.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![feature(stdarch_arm_neon_intrinsics)]
#![feature(arm_target_feature)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

mod tasks;

use core::sync::atomic::{AtomicBool, Ordering};

use deluge::prelude::*;
use deluge_bsp::cv_gate;
use deluge_bsp::uart as bsp_uart;
use rza1l_hal::gic;
use rza1l_hal::usb::{
    Rusb1Driver, dcd_int_handler, hcd_int_handler, init_device_mode, init_host_mode,
};

// ---------------------------------------------------------------------------
// USB mode selection
// ---------------------------------------------------------------------------

/// Set to `true` before `setup()` registers the ISR to start USB0 in host mode
/// instead of device mode. Switching at runtime requires quiescing the port,
/// calling `UsbPort::into_device_mode` / `UsbPort::into_host_mode`, and updating
/// this flag under an IRQ-disabled critical section. The ISR dispatcher reads
/// this on every interrupt.
static USB0_HOST_MODE: AtomicBool = AtomicBool::new(false);

// ---------------------------------------------------------------------------
// USB descriptor static buffers
// ---------------------------------------------------------------------------
//
// `'static` lifetimes required by `embassy_usb::Builder` and `builder.handler()`.

static mut USB_CONFIG_DESC: [u8; 768] = [0; 768];
static mut USB_BOS_DESC: [u8; 64] = [0; 64];
static mut USB_MSOS_DESC: [u8; 0] = [];
static mut USB_CONTROL_BUF: [u8; 64] = [0; 64];
/// Backing storage for the UAC2 `AudioClass` handler (must be `'static`).
static mut AUDIO_CLASS_BUF: core::mem::MaybeUninit<deluge_bsp::usb::classes::audio::AudioClass<2>> =
    core::mem::MaybeUninit::uninit();

// ---------------------------------------------------------------------------
// Pre-interrupt setup (runs with IRQs masked)
// ---------------------------------------------------------------------------

/// Synchronous board bring-up that must complete before interrupts are enabled.
///
/// Runs after the SDK has initialised heaps and clocks (see
/// [`#[deluge::app]`](deluge::app)), but before `cortex_ar::interrupt::enable()`.
/// It configures peripherals and registers GIC interrupt handlers; each driver
/// registers its handler before enabling its interrupt source, so the USB build
/// in `main` (after IRQs are on) is safe.
fn setup() {
    info!("Deluge demo firmware starting");
    info!("Pad paint demo: press pads to toggle, buttons 0/1/2 to clear/fill/invert");

    info!("GPIO: configuring heartbeat LED...");
    unsafe { rza1l_hal::gpio::set_as_output(6, 7) };

    info!("UART: initialising MIDI...");
    unsafe { bsp_uart::init_midi(31_250) };
    info!("UART: initialising PIC...");
    unsafe { bsp_uart::init_pic(31_250) };
    info!("UART: SCIF0/1 @ 31 250 baud");

    info!("audio: initialising SSI0...");
    unsafe { deluge_bsp::audio::init_with_scux() };
    info!("audio: SSI0 + DMA + codec running");
    // Pre-fill SSI TX buffer with dither so the codec does not auto-mute before
    // the first USB stream arrives.
    tasks::audio::fill_tx_with_dither();
    info!("audio: TX buffer pre-filled with dither");

    // RSPI0 init — shared between OLED (8-bit) and CV DAC (32-bit), arbitrated by
    // deluge_bsp::bus. cv_gate::init() leaves RSPI0 ready; oled::init() (in its
    // task) switches frame mode through the bus guard.
    unsafe { cv_gate::init() };
    info!("RSPI0: initialised via cv_gate::init");

    // Register the USB0 ISR handler. The dispatcher checks USB0_HOST_MODE on
    // every interrupt to direct the call to the device or host handler without
    // re-registering. The USB interrupt source is only enabled later when the
    // device/host task starts, so registering here (before global enable) is
    // sufficient.
    unsafe {
        gic::register(rza1l_hal::usb::USB0_IRQ, || {
            if USB0_HOST_MODE.load(Ordering::Relaxed) {
                hcd_int_handler(0);
            } else {
                dcd_int_handler(0);
            }
        });
    }

    info!("encoder: configuring interrupt-driven inputs...");
    unsafe { deluge_bsp::encoder::irq_init() };
    info!("trigger_clock: configuring interrupt-driven input...");
    unsafe { deluge_bsp::trigger_clock::irq_init() };
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[deluge::app(setup = setup)]
async fn main(dlg: Deluge) {
    let spawner = dlg.spawner();

    // ── Build the USB stack ──────────────────────────────────────────────────
    // The ISR handler was registered in `setup()`; `init_device_mode` /
    // `init_host_mode` enable the module clock, and the USB interrupt source is
    // only raised once the device/host task starts running. Building the
    // descriptors here (with interrupts enabled) therefore does not race the ISR.
    info!(
        "USB: initialising USB0 (host_mode={})...",
        USB0_HOST_MODE.load(Ordering::Relaxed)
    );

    let mut usb_host_driver: Option<rza1l_hal::usb::Rusb1HostDriver> = None;
    let mut usb_device_opt: Option<embassy_usb::UsbDevice<'static, Rusb1Driver>> = None;
    let mut ep_out_opt: Option<rza1l_hal::usb::Rusb1EndpointOut> = None;
    let mut ep_in_opt: Option<rza1l_hal::usb::Rusb1EndpointIn> = None;
    let mut midi_sender_opt: Option<embassy_usb::class::midi::Sender<'static, Rusb1Driver>> = None;
    let mut midi_receiver_opt: Option<embassy_usb::class::midi::Receiver<'static, Rusb1Driver>> =
        None;

    if USB0_HOST_MODE.load(Ordering::Relaxed) {
        let (_port, hd) = unsafe { init_host_mode(0) };
        usb_host_driver = Some(hd);
        info!("USB: host driver ready");
    } else {
        let (usb_device, ep_out, ep_in, midi_sender, midi_receiver) = unsafe {
            let (_port, driver) = init_device_mode(0);
            // Example/development USB identity — self-contained, NOT a product
            // ID. See `deluge_bsp::usb::ids` for the product-identity rules and
            // why distinct PIDs matter on macOS. TODO: use IDs you own (e.g. a
            // pid.codes prototype ID) before distributing this firmware.
            const USB_VID: u16 = 0x16D0;
            const USB_PID: u16 = 0x0EDA;
            let mut config = embassy_usb::Config::new(USB_VID, USB_PID);
            config.manufacturer = Some("Synthstrom Audible");
            config.product = Some("Deluge");
            config.self_powered = false;
            config.max_power = 250; // 500 mA

            let mut builder = embassy_usb::Builder::new(
                driver,
                config,
                &mut *core::ptr::addr_of_mut!(USB_CONFIG_DESC),
                &mut *core::ptr::addr_of_mut!(USB_BOS_DESC),
                &mut *core::ptr::addr_of_mut!(USB_MSOS_DESC),
                &mut *core::ptr::addr_of_mut!(USB_CONTROL_BUF),
            );

            // Allocate the UAC2 speaker + mic interfaces.
            // CAPTURE_CH=2: stereo ISO IN, matching the stereo SSI codec and the
            // stereo packing in uac2_mic_task.  (Was 8, which both mis-declared
            // the channel count and made the ISO IN maxpacket 49*8*3=1176 B —
            // over the 1024 B high-speed ISO limit, so the host rejected/clamped
            // it and capture failed.  Stereo → 49*2*3=294 B, valid.)  ISO OUT is
            // always stereo.
            let (audio_instance, ep_out, ep_in) =
                deluge_bsp::usb::classes::audio::AudioClass::<2>::new(&mut builder, 288);
            // Store in a `'static` slot so the `&'static mut` reference satisfies
            // `builder.handler`'s `'d` lifetime (= `'static` here).
            let audio_ref = (&mut *core::ptr::addr_of_mut!(AUDIO_CLASS_BUF)).write(audio_instance);
            builder.handler(audio_ref);

            // USB MIDI 1.0 class — 1 in-jack (DIN→USB), 1 out-jack (USB→DIN).
            // 512-byte bulk endpoints: the RUSB1 PHY always negotiates high
            // speed (SYSCFG.HSE=1), and USB 2.0 requires HS bulk endpoints to use
            // wMaxPacketSize 512. Advertising 64 (the FS value) makes the host
            // reject the endpoints ("invalid maxpacket 64") and silently breaks
            // the bulk OUT direction.
            let midi = embassy_usb::class::midi::MidiClass::new(&mut builder, 1, 1, 512);
            let (midi_sender, midi_receiver) = midi.split();

            (builder.build(), ep_out, ep_in, midi_sender, midi_receiver)
        };
        usb_device_opt = Some(usb_device);
        ep_out_opt = Some(ep_out);
        ep_in_opt = Some(ep_in);
        midi_sender_opt = Some(midi_sender);
        midi_receiver_opt = Some(midi_receiver);
        info!("USB: UsbDevice built");
    }

    // ── Spawn tasks ──────────────────────────────────────────────────────────
    info!("starting Embassy tasks");
    spawner.spawn(tasks::blink::blink_task().unwrap());

    // USB-mode-specific tasks ─────────────────────────────────────────────────
    if let Some(hd) = usb_host_driver {
        spawner.spawn(tasks::usb_host::usb_host_task(hd).unwrap());
    } else if let (
        Some(usb_device),
        Some(ep_out),
        Some(ep_in),
        Some(midi_sender),
        Some(midi_receiver),
    ) = (
        usb_device_opt,
        ep_out_opt,
        ep_in_opt,
        midi_sender_opt,
        midi_receiver_opt,
    ) {
        spawner.spawn(tasks::usb::usb_task(usb_device).unwrap());
        spawner.spawn(tasks::audio::uac2_task(ep_out).unwrap());
        spawner.spawn(tasks::audio::uac2_mic_task(ep_in).unwrap());
        spawner.spawn(tasks::midi::midi_usb_rx_task(midi_receiver).unwrap());
        spawner.spawn(tasks::midi::midi_din_tx_task(midi_sender).unwrap());
    }

    // Tasks common to both modes ───────────────────────────────────────────────
    spawner.spawn(tasks::pic::pic_task().unwrap());
    spawner.spawn(tasks::encoder::encoder_task().unwrap());
    spawner.spawn(tasks::jack_detect::jack_detect_task(200).unwrap());
    spawner.spawn(tasks::analysis::analysis_task().unwrap());
    spawner.spawn(tasks::rgb::rgb_task().unwrap());
    spawner.spawn(tasks::oled::oled_task().unwrap());
    spawner.spawn(tasks::sd::sd_task().unwrap());
}
