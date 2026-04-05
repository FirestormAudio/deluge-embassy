//! Deluge firmware image.
//!
//! This crate assembles the board support package into a runnable Embassy-based
//! firmware for the Deluge. The BSP owns board-level hardware concerns such as
//! clocking, PIC transport, shared pad state, encoder IRQ capture/decoding, and
//! long-running board tasks like jack detection. This crate owns product-level
//! behavior layered on top of those primitives.
//!
//! ## Runtime role
//! - Initialises the platform, heaps, USB, audio, and task executor.
//! - Runs the Deluge USB device or host stack depending on `USB0_HOST_MODE`.
//! - Translates BSP input streams into firmware events and UI behavior.
//! - Applies product policies such as OLED rendering, RGB rendering, analyzer
//!   display, heartbeat behavior, and MOD knob volume control.
//!
//! ## Task split
//! - **BSP services** — reusable hardware capture and shared state, such as
//!   PIC transport, shared pad state, and encoder IRQ capture/decoding.
//! - **Firmware tasks** — product behavior and policy, including PIC event
//!   handling, encoder actions, jack detection policy, CDC protocol handling,
//!   USB audio tasks, OLED/RGB rendering, and analysis.
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

mod events;
mod tasks;

use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use log::{debug, error, info};

use embassy_executor::{Executor, Spawner};

use core::sync::atomic::{AtomicBool, Ordering};
use deluge_bsp::cv_gate;
use deluge_bsp::uart as bsp_uart;
use deluge_bsp::usb::{
    Rusb1Driver, dcd_int_handler, hcd_int_handler, init_device_mode, init_host_mode,
};
use rza1l_hal::{allocator, gic};

unsafe extern "C" {
    /// Start of the free SRAM heap region (set by the linker script).
    static __sram_heap_start: u8;
    /// End of the free SRAM heap region (start of RTT/stack reservation).
    static __sram_heap_end: u8;
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("PANIC: {}", info);
    loop {
        core::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// USB mode selection
// ---------------------------------------------------------------------------

/// Set to `true` before calling `main()` (or before the ISR is registered)
/// to start USB0 in host mode instead of device mode.  Switching at runtime
/// requires quiescing the port, calling `UsbPort::into_device_mode` /
/// `UsbPort::into_host_mode`, and updating this flag under an IRQ-disabled
/// critical section.  The ISR dispatcher reads this on every interrupt.
static USB0_HOST_MODE: AtomicBool = AtomicBool::new(false);

// ---------------------------------------------------------------------------
// USB descriptor static buffers
// ---------------------------------------------------------------------------
//
// Must live here (in the root crate) so the `'static` lifetimes required by
// `embassy_usb::Builder` and `builder.handler()` are satisfied.

static mut USB_CONFIG_DESC: [u8; 768] = [0; 768];
static mut USB_BOS_DESC: [u8; 64] = [0; 64];
static mut USB_MSOS_DESC: [u8; 0] = [];
static mut USB_CONTROL_BUF: [u8; 64] = [0; 64];
/// Backing storage for the UAC2 `AudioClass` handler (must be `'static`).
static mut AUDIO_CLASS_BUF: core::mem::MaybeUninit<deluge_bsp::usb::classes::audio::AudioClass> =
    core::mem::MaybeUninit::uninit();
/// Backing storage for the CDC-ACM class state (line-coding, control signals).
static mut CDC_ACM_STATE: embassy_usb::class::cdc_acm::State<'static> =
    embassy_usb::class::cdc_acm::State::new();

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

static mut EXECUTOR: MaybeUninit<Executor> = MaybeUninit::uninit();

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    // rtt_init! must always run to define the _SEGGER_RTT control-block symbol
    // that rtt-target references at link time (also used by rza1 and deluge-bsp).
    // The 16 KB ring buffer lives in .rtt_buffer (uncached RAM).
    // In release builds the logger is never registered and all log!() call sites
    // compile to nothing (release_max_level_off feature on the log crate).
    #[cfg(feature = "rtt")]
    {
        let channels = rtt_target::rtt_init! {
            up: {
                0: {
                    size: 16384,
                    name: "Terminal",
                    section: ".rtt_buffer"   // ring buffer bytes in uncached RAM
                }
            }
            section_cb: ".rtt_buffer"        // _SEGGER_RTT control block in uncached RAM
        };
        rtt_target::set_print_channel(channels.up.0);
        rtt_target::init_logger_with_level(log::LevelFilter::Debug);
    }
    info!("Deluge demo firmware starting");
    info!("Pad paint demo: press pads to toggle, buttons 0/1/2 to clear/fill/invert");

    // Initialise the SRAM heap before any allocation from internal RAM.
    unsafe {
        let start = core::ptr::addr_of!(__sram_heap_start) as *mut u8;
        let size = core::ptr::addr_of!(__sram_heap_end) as usize - start as usize;
        allocator::SRAM.init(start, size);
    }
    info!("SRAM heap: initialised ({} KB)", {
        let s = core::ptr::addr_of!(__sram_heap_end) as usize
            - core::ptr::addr_of!(__sram_heap_start) as usize;
        s / 1024
    });

    unsafe { deluge_bsp::system::init_clocks() };
    info!("system: module clocks, MMU, cache, SDRAM, GIC, OSTM, time driver ready");

    // Initialise the SDRAM heap now that the SDRAM window is accessible.
    unsafe { allocator::SDRAM.init(0x0C00_0000 as *mut u8, 64 * 1024 * 1024) };
    info!("SDRAM heap: initialised (64 MB)");

    info!("GPIO: configuring heartbeat LED...");
    unsafe { rza1l_hal::gpio::set_as_output(6, 7) };
    info!("GPIO: heartbeat LED P6_7 configured");

    info!("UART: initialising MIDI...");
    unsafe { bsp_uart::init_midi(31_250) };
    info!("UART: MIDI ready");

    info!("UART: initialising PIC...");
    unsafe { bsp_uart::init_pic(31_250) };
    info!("UART: SCIF0/1 @ 31 250 baud");

    info!("audio: initialising SSI0...");
    unsafe { deluge_bsp::audio::init() };
    info!("audio: SSI0 + DMA + codec running");

    // Pre-fill SSI TX buffer with dither so the codec does not auto-mute before
    // the first USB stream arrives.
    tasks::audio::fill_tx_with_dither();
    info!("audio: TX buffer pre-filled with dither");

    // RSPI0 init — shared between OLED (8-bit) and CV DAC (32-bit).
    // cv_gate::init() puts it in 32-bit mode; oled::init() will switch to 8-bit.
    unsafe { cv_gate::init() };
    info!("RSPI0: initialised via cv_gate::init");

    // ── USB0 mode selection (host vs device) ──────────────────────────────
    // Set USB0_HOST_MODE to true *before* this point to start in host mode.
    // The ISR registered above dispatches to hcd_int_handler or dcd_int_handler
    // based on the flag, so no ISR re-registration is needed when switching.
    info!(
        "USB: initialising USB0 (host_mode={})...",
        USB0_HOST_MODE.load(Ordering::Relaxed)
    );

    // Build UsbDevice (device mode) or a Rusb1HostDriver (host mode).  Both
    // paths call module_clock_enable internally via init_device_mode /
    // init_host_mode, so the manual call below is no longer needed.

    // Register the USB0 ISR *before* IRQ is globally enabled.  The dispatcher
    // checks USB0_HOST_MODE on every interrupt to direct the call to either the
    // device or host interrupt handler without needing to re-register the ISR.
    unsafe {
        gic::register(rza1l_hal::rusb1::USB0_IRQ, || {
            if USB0_HOST_MODE.load(Ordering::Relaxed) {
                hcd_int_handler(0);
            } else {
                dcd_int_handler(0);
            }
        });
    }

    // ── Host mode: just allocate the host driver.  ──────────────────────────
    // ── Device mode: build the full UsbDevice stack.  ───────────────────────
    //
    // `init_device_mode` / `init_host_mode` both call `module_clock_enable`.
    // UsbDevice buffers must be `'static`; they live in the statics above.
    let mut usb_host_driver: Option<deluge_bsp::usb::Rusb1HostDriver> = None;
    let mut usb_device_opt: Option<embassy_usb::UsbDevice<'static, Rusb1Driver>> = None;
    let mut ep_out_opt: Option<deluge_bsp::usb::Rusb1EndpointOut> = None;
    let mut ep_in_opt: Option<deluge_bsp::usb::Rusb1EndpointIn> = None;
    let mut cdc_opt: Option<embassy_usb::class::cdc_acm::CdcAcmClass<'static, Rusb1Driver>> = None;
    let mut midi_sender_opt: Option<embassy_usb::class::midi::Sender<'static, Rusb1Driver>> = None;
    let mut midi_receiver_opt: Option<embassy_usb::class::midi::Receiver<'static, Rusb1Driver>> =
        None;

    if USB0_HOST_MODE.load(Ordering::Relaxed) {
        let (_port, hd) = unsafe { init_host_mode(0) };
        usb_host_driver = Some(hd);
        info!("USB: host driver ready");
    } else {
        let (usb_device, ep_out, ep_in, cdc, midi_sender, midi_receiver) = unsafe {
            let (_port, driver) = init_device_mode(0);
            let mut config = embassy_usb::Config::new(0x16D0, 0x0EDA);
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

            // Allocate the UAC2 speaker + mic interfaces (288 B covers stereo 24-bit @ 48 kHz).
            let (audio_instance, ep_out, ep_in) =
                deluge_bsp::usb::classes::audio::AudioClass::new(&mut builder, 288);
            // Store in a `'static` slot so the `&'static mut` reference satisfies
            // `builder.handler`'s `'d` lifetime (= `'static` here).
            let audio_ref = (&mut *core::ptr::addr_of_mut!(AUDIO_CLASS_BUF)).write(audio_instance);
            builder.handler(audio_ref);

            // CDC-ACM serial class (Deluge host protocol).
            let cdc = embassy_usb::class::cdc_acm::CdcAcmClass::new(
                &mut builder,
                &mut *core::ptr::addr_of_mut!(CDC_ACM_STATE),
                64,
            );

            // USB MIDI 1.0 class — 1 in-jack (DIN→USB), 1 out-jack (USB→DIN).
            let midi = embassy_usb::class::midi::MidiClass::new(&mut builder, 1, 1, 64);
            let (midi_sender, midi_receiver) = midi.split();

            (
                builder.build(),
                ep_out,
                ep_in,
                cdc,
                midi_sender,
                midi_receiver,
            )
        };
        usb_device_opt = Some(usb_device);
        ep_out_opt = Some(ep_out);
        ep_in_opt = Some(ep_in);
        cdc_opt = Some(cdc);
        midi_sender_opt = Some(midi_sender);
        midi_receiver_opt = Some(midi_receiver);
        info!("USB: UsbDevice built");
    }

    info!("encoder: configuring interrupt-driven inputs...");
    unsafe { deluge_bsp::encoder::irq_init() };

    debug!("enabling IRQ...");
    unsafe { cortex_ar::interrupt::enable() };
    debug!("IRQ enabled OK");
    info!("IRQ: enabled — starting Embassy tasks");

    debug!("Executor::new()...");
    #[allow(static_mut_refs)]
    let executor = unsafe {
        EXECUTOR.write(Executor::new());
        EXECUTOR.assume_init_mut()
    };
    debug!("executor.run()...");
    executor.run(|spawner: Spawner| {
        spawner.spawn(tasks::blink::blink_task().unwrap());

        // USB-mode-specific tasks ─────────────────────────────────────────
        if let Some(hd) = usb_host_driver {
            spawner.spawn(tasks::usb_host::usb_host_task(hd).unwrap());
        } else if let (
            Some(usb_device),
            Some(ep_out),
            Some(ep_in),
            Some(cdc),
            Some(midi_sender),
            Some(midi_receiver),
        ) = (
            usb_device_opt,
            ep_out_opt,
            ep_in_opt,
            cdc_opt,
            midi_sender_opt,
            midi_receiver_opt,
        ) {
            spawner.spawn(tasks::usb::usb_task(usb_device).unwrap());
            spawner.spawn(tasks::audio::uac2_task(ep_out).unwrap());
            spawner.spawn(tasks::audio::uac2_mic_task(ep_in).unwrap());
            spawner.spawn(tasks::cdc::cdc_task(cdc).unwrap());
            spawner.spawn(tasks::midi::midi_usb_rx_task(midi_receiver).unwrap());
            spawner.spawn(tasks::midi::midi_din_tx_task(midi_sender).unwrap());
        }

        // Tasks common to both modes ───────────────────────────────────────
        spawner.spawn(tasks::pic::pic_task().unwrap());
        spawner.spawn(tasks::encoder::encoder_task().unwrap());
        spawner.spawn(tasks::jack_detect::jack_detect_task(200).unwrap());
        spawner.spawn(tasks::analysis::analysis_task().unwrap());
        spawner.spawn(tasks::rgb::rgb_task().unwrap());
        spawner.spawn(tasks::oled::oled_task().unwrap());
        spawner.spawn(tasks::sd::sd_task().unwrap());
    });
}
