//! SD High-Speed bench firmware.
//!
//! A throwaway diagnostic image: brings up the platform and SD card, logs the
//! CMD6 High-Speed query/switch outcome, measures sequential read throughput
//! at 16.7 MHz (P1/4) vs 33.3 MHz (P1/2), then runs a CRC soak.  Optional
//! write soak via [`tasks::bench::WRITE_SOAK`] (dev cards only).
//!
//! Build: `cargo build-fw -p sd-bench --features rtt` — output over RTT.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod tasks;

use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use log::{error, info};

use embassy_executor::{Executor, Spawner};

use deluge_alloc as allocator;
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
    info!("Deluge sd-bench firmware starting");

    // Initialise the SRAM heap before any allocation from internal RAM.
    unsafe {
        let start = core::ptr::addr_of!(__sram_heap_start) as *mut u8;
        let size = core::ptr::addr_of!(__sram_heap_end) as usize - start as usize;
        allocator::SRAM.init(start, size);
    }

    unsafe { deluge_bsp::system::init_clocks() };
    info!("system: clocks, MMU, cache, SDRAM, GIC, OSTM, time driver ready");

    // SDRAM heap (some BSP paths allocate from it; bring it up to be safe).
    unsafe { allocator::SDRAM.init(0x0C00_0000 as *mut u8, 64 * 1024 * 1024) };

    // Heartbeat LED pin (unused beyond init; keeps parity with other firmwares).
    unsafe { rza1l_hal::gpio::set_as_output(6, 7) };

    info!("UART: initialising PIC...");
    unsafe { bsp_uart::init_pic(31_250) };

    unsafe { cortex_ar::interrupt::enable() };
    info!("IRQ: enabled — starting Embassy tasks");

    #[allow(static_mut_refs)]
    let executor = unsafe {
        EXECUTOR.write(Executor::new());
        EXECUTOR.assume_init_mut()
    };
    executor.run(|spawner: Spawner| {
        spawner.spawn(tasks::pic::pic_task().unwrap());
        spawner.spawn(tasks::bench::bench_task().unwrap());
    });
}
