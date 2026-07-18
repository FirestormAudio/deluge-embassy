//! Deluge UAC host **validation** firmware.
//!
//! Brings USB0 up in host mode, runs `usb_host_supervisor` (which auto-binds the
//! `deluge_bsp::usb::host::uac` driver on connect), and loops captured audio back
//! to playback. First real-hardware exercise of the host UAC capture+playback
//! stack. See `docs/superpowers/specs/2026-07-17-uac-host-validation-firmware-design.md`.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod tasks;

use core::mem::MaybeUninit;
use core::panic::PanicInfo;

use embassy_executor::{Executor, Spawner};
use log::{error, info};

use deluge_alloc as allocator;
use deluge_bsp::{cv_gate, uart as bsp_uart};
use rza1l_hal::gic;
use rza1l_hal::usb::{hcd_int_handler, init_host_mode};

unsafe extern "C" {
    static __sram_heap_start: u8;
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
            up: { 0: { size: 16384, name: "Terminal", section: ".rtt_buffer" } }
            section_cb: ".rtt_buffer"
        };
        rtt_target::set_print_channel(channels.up.0);
        rtt_target::init_logger_with_level(log::LevelFilter::Debug);
    }
    info!("UAC host validation firmware starting");

    unsafe {
        let start = core::ptr::addr_of!(__sram_heap_start) as *mut u8;
        let size = core::ptr::addr_of!(__sram_heap_end) as usize - start as usize;
        allocator::SRAM.init(start, size);
    }
    unsafe { deluge_bsp::system::init_clocks() };
    unsafe { allocator::SDRAM.init(0x0C00_0000 as *mut u8, 64 * 1024 * 1024) };

    unsafe { rza1l_hal::gpio::set_as_output(6, 7) };
    unsafe { bsp_uart::init_pic(31_250) };
    unsafe { cv_gate::init() };

    // USB0 host-mode ISR.
    unsafe {
        gic::register(rza1l_hal::usb::USB0_IRQ, || {
            hcd_int_handler(0);
        });
    }

    let (_port, host_driver) = unsafe { init_host_mode(0) };
    info!("USB: host driver ready (USB0 host mode)");

    unsafe { cortex_ar::interrupt::enable() };
    info!("IRQ enabled — starting tasks");

    #[allow(static_mut_refs)]
    let executor = unsafe {
        EXECUTOR.write(Executor::new());
        EXECUTOR.assume_init_mut()
    };
    executor.run(|spawner: Spawner| {
        spawner.spawn(tasks::blink::blink_task().unwrap());
        spawner.spawn(tasks::pic::pic_task().unwrap());
        spawner.spawn(tasks::oled::oled_task().unwrap());
        spawner.spawn(tasks::loopback::loopback_task().unwrap());
        // The supervisor enumerates + binds the UAC driver, which spawns the
        // capture/playback pump. It needs the spawner to spawn that child task.
        spawner.spawn(
            deluge_bsp::usb::host::usb_host_supervisor(host_driver, spawner).expect("supervisor task pool full"),
        );
    })
}
