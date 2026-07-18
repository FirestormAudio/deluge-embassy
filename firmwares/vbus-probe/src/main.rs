//! USB VBUS-sense probe firmware.
//!
//! A throwaway diagnostic image: it brings up the platform + OLED, puts USB0
//! into **host mode** (the same state the UAC host firmware runs in), then
//! continuously reads the RUSB1 VBUS monitor (`INTSTS0.VBSTS`) and reports
//! PRESENT / ABSENT on the OLED and over RTT.
//!
//! Purpose: the RZ/A1L RUSB1 has no VBUS-drive bit (`DVSTCTR0` bits 11:9 are
//! reserved) — host-mode VBUS must be switched on externally. Use this to check
//! whether the USB port is actually powered while in host mode: plug a device
//! and watch whether VBUS reads PRESENT. If it stays ABSENT, the connected
//! device is unpowered (which stalls enumeration).
//!
//! Build: `cargo build-fw -p vbus-probe --features rtt`.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod tasks;

use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use log::{error, info};

use embassy_executor::{Executor, Spawner};

use deluge_alloc as allocator;
use deluge_bsp::cv_gate;
use deluge_bsp::uart as bsp_uart;

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

static mut EXECUTOR: MaybeUninit<Executor> = MaybeUninit::uninit();

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
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
    info!("Deluge VBUS-probe firmware starting");

    // Initialise the SRAM heap before any allocation from internal RAM.
    unsafe {
        let start = core::ptr::addr_of!(__sram_heap_start) as *mut u8;
        let size = core::ptr::addr_of!(__sram_heap_end) as usize - start as usize;
        allocator::SRAM.init(start, size);
    }

    unsafe { deluge_bsp::system::init_clocks() };
    info!("system: clocks, MMU, cache, SDRAM, GIC, OSTM, time driver ready");

    unsafe { allocator::SDRAM.init(0x0C00_0000 as *mut u8, 64 * 1024 * 1024) };

    // Heartbeat LED pin (unused beyond init; keeps parity with other firmwares).
    unsafe { rza1l_hal::gpio::set_as_output(6, 7) };

    info!("UART: initialising PIC...");
    unsafe { bsp_uart::init_pic(31_250) };

    // RSPI0 init — shared between OLED (8-bit) and CV DAC (32-bit).
    unsafe { cv_gate::init() };
    info!("RSPI0: initialised via cv_gate::init");

    // Bring USB0 up in host mode so VBSTS reflects the real host-mode VBUS
    // state. The returned driver is unused — this firmware only reads the VBUS
    // monitor; no ISR is registered, so USB interrupts stay masked at the GIC.
    let (_port, _driver) = unsafe { rza1l_hal::usb::init_host_mode(0) };
    info!("USB: USB0 in host mode (VBUS sense only)");

    unsafe { cortex_ar::interrupt::enable() };
    info!("IRQ: enabled — starting Embassy tasks");

    #[allow(static_mut_refs)]
    let executor = unsafe {
        EXECUTOR.write(Executor::new());
        EXECUTOR.assume_init_mut()
    };
    executor.run(|spawner: Spawner| {
        spawner.spawn(tasks::pic::pic_task().unwrap());
        spawner.spawn(tasks::probe::probe_task().unwrap());
    });
}
