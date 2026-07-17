//! Deluge USB Mass Storage firmware image.
//!
//! Turns the Deluge into a USB SD-card reader/writer.  When connected to a host
//! the device enumerates as a USB Mass Storage (Bulk-Only Transport, SCSI
//! transparent) device exposing the inserted SD card as a raw block device, so
//! the host OS mounts the card's FAT volume directly.
//!
//! ## Runtime role
//! - Initialises the platform, heaps, PIC/OLED transport, and the task executor.
//! - Brings USB0 up in **device mode** with a single MSC interface.
//! - Runs the Bulk-Only Transport / SCSI command loop, bridging USB bulk
//!   endpoints to [`deluge_bsp::sd`] block read/write.
//! - Renders a live throughput display (TX/RX MB/s and cumulative MB) to the OLED.
//!
//! ## Concurrency note
//! While acting as USB mass storage the **host owns the filesystem**.  This app
//! performs raw block passthrough only and never mounts or writes FAT itself,
//! avoiding cache-coherency corruption on the RZ/A1L.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod tasks;

use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use log::{debug, error, info};

use embassy_executor::{Executor, Spawner};

use deluge_alloc as allocator;
use deluge_bsp::cv_gate;
use deluge_bsp::uart as bsp_uart;
use rza1l_hal::gic;
use rza1l_hal::usb::{dcd_int_handler, init_device_mode};

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
// USB descriptor static buffers
// ---------------------------------------------------------------------------
//
// Must live here (in the root crate) so the `'static` lifetimes required by
// `embassy_usb::Builder` and `builder.handler()` are satisfied.

static mut USB_CONFIG_DESC: [u8; 256] = [0; 256];
static mut USB_BOS_DESC: [u8; 64] = [0; 64];
static mut USB_MSOS_DESC: [u8; 0] = [];
static mut USB_CONTROL_BUF: [u8; 64] = [0; 64];
/// Backing storage for the MSC class handler (must be `'static`).
static mut MSC_CLASS_BUF: MaybeUninit<deluge_bsp::usb::classes::msc::MscClass> =
    MaybeUninit::uninit();

static mut EXECUTOR: MaybeUninit<Executor> = MaybeUninit::uninit();

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    // rtt_init! must always run to define the _SEGGER_RTT control-block symbol
    // that rtt-target references at link time (also used by rza1 and deluge-bsp).
    #[cfg(feature = "rtt")]
    {
        let channels = rtt_target::rtt_init! {
            up: {
                0: {
                    size: 16384,
                    name: "Terminal",
                    section: ".rtt_buffer"
                }
            }
            section_cb: ".rtt_buffer"
        };
        rtt_target::set_print_channel(channels.up.0);
        rtt_target::init_logger_with_level(log::LevelFilter::Debug);
    }
    info!("Deluge USB Mass Storage firmware starting");

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

    info!("UART: initialising PIC...");
    unsafe { bsp_uart::init_pic(31_250) };
    info!("UART: SCIF1 @ 31 250 baud");

    // RSPI0 init — shared between OLED (8-bit) and CV DAC (32-bit).
    // cv_gate::init() enables the RSPI0 module clock; oled::init() then switches
    // it to 8-bit mode for the panel.
    unsafe { cv_gate::init() };
    info!("RSPI0: initialised via cv_gate::init");

    // ── USB0 device mode ──────────────────────────────────────────────────
    // Register the USB0 ISR *before* IRQ is globally enabled.
    unsafe {
        gic::register(rza1l_hal::usb::USB0_IRQ, || {
            dcd_int_handler(0);
        });
    }

    let (usb_device, ep_out, ep_in) = unsafe {
        let (_port, driver) = init_device_mode(0);
        // Example/development USB identity — self-contained, NOT a product ID.
        // See `deluge_bsp::usb::ids` for the product-identity rules and why
        // distinct PIDs matter on macOS. TODO: use IDs you own (e.g. a pid.codes
        // prototype ID) before distributing this firmware.
        const USB_VID: u16 = 0x16D0;
        const USB_PID: u16 = 0x0EDA;
        let mut config = embassy_usb::Config::new(USB_VID, USB_PID);
        config.manufacturer = Some("Synthstrom Audible");
        config.product = Some("Deluge SD Card");
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

        // Single MSC interface with one bulk IN + bulk OUT endpoint pair.
        // 512-byte bulk endpoints: the RUSB1 PHY always negotiates high speed,
        // and USB 2.0 requires HS bulk endpoints to use wMaxPacketSize 512.
        let (msc, ep_out, ep_in) = deluge_bsp::usb::classes::msc::MscClass::new(&mut builder, 512);
        let msc_ref = (&mut *core::ptr::addr_of_mut!(MSC_CLASS_BUF)).write(msc);
        builder.handler(msc_ref);

        (builder.build(), ep_out, ep_in)
    };
    info!("USB: UsbDevice built (MSC)");

    debug!("enabling IRQ...");
    unsafe { cortex_ar::interrupt::enable() };
    info!("IRQ: enabled — starting Embassy tasks");

    #[allow(static_mut_refs)]
    let executor = unsafe {
        EXECUTOR.write(Executor::new());
        EXECUTOR.assume_init_mut()
    };
    executor.run(|spawner: Spawner| {
        spawner.spawn(tasks::blink::blink_task().unwrap());
        spawner.spawn(tasks::usb::usb_task(usb_device).unwrap());
        spawner.spawn(tasks::msc::msc_task(ep_in, ep_out).unwrap());
        spawner.spawn(tasks::pic::pic_task().unwrap());
        spawner.spawn(tasks::oled::oled_task().unwrap());
    });
}
