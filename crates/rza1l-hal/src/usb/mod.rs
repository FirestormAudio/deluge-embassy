//! RUSB1 USB modules for the RZ/A1L (HW manual §28).
//!
//! Provides the chip-level USB plumbing (base addresses, GIC interrupt IDs,
//! STBCR7 clock gates) plus full `embassy-usb-driver` implementations for
//! both directions of the dual-role RUSB1 controller:
//!
//! - [`driver`] — device mode (`embassy_usb_driver::Driver`):
//!   [`Rusb1Driver`], [`Rusb1Bus`], [`Rusb1ControlPipe`], endpoints.
//! - [`host`] — host mode (`embassy_usb_driver::host::UsbHostController`):
//!   [`Rusb1HostDriver`], [`Rusb1Allocator`], [`Rusb1Pipe`].
//! - [`pipe`], [`fifo`], [`regs`] — shared pipe/FIFO/register layers.
//!
//! USB *class* implementations (UAC2 audio, MIDI, MSC) and the Bulk-Only
//! Transport engine live in the board-support crate (`deluge_bsp::usb`),
//! which re-exports this module.
//!
//! ## Quick start — device mode
//!
//! ```rust,no_run
//! use rza1l_hal::usb::{init_device_mode, UsbMode};
//!
//! let (port, driver) = unsafe { init_device_mode(0) };
//! // pass `driver` to `embassy_usb::UsbDevice::new(driver, config, ...)`
//! ```
//!
//! ## Quick start — host mode
//!
//! ```rust,no_run
//! use rza1l_hal::usb::init_host_mode;
//! use embassy_usb_host::{bus, BusState, handler::BusRoute};
//!
//! static BUS_STATE: BusState = BusState::new();
//! let (port, host) = unsafe { init_host_mode(0) };
//! let (mut controller, handle) = bus(host, &BUS_STATE);
//! let speed = controller.wait_for_connection().await;
//! let (info, _) = handle.enumerate(BusRoute::Direct(speed), &mut [0u8; 256]).await.unwrap();
//! ```
//!
//! ## ISR wiring
//!
//! You must call `dcd_int_handler` (device mode) or `hcd_int_handler` (host
//! mode) from your GIC interrupt dispatcher:
//!
//! ```rust,no_run
//! use rza1l_hal::usb::{dcd_int_handler, hcd_int_handler};
//!
//! #[no_mangle]
//! extern "C" fn irq73_handler() {  // USB0 — device mode
//!     unsafe { dcd_int_handler(0); }
//! }
//! #[no_mangle]
//! extern "C" fn irq73_handler() {  // USB0 — host mode
//!     unsafe { hcd_int_handler(0); }
//! }
//! ```

pub mod driver;
pub mod fifo;
pub mod host;
pub mod pipe;
pub mod regs;

pub use driver::{
    Rusb1Bus, Rusb1ControlPipe, Rusb1Driver, Rusb1EndpointIn, Rusb1EndpointOut, dcd_int_handler,
};
pub use host::{Rusb1Allocator, Rusb1HostDriver, Rusb1Pipe, hcd_int_handler};

use core::marker::PhantomData;

// ---------------------------------------------------------------------------
// Base addresses, interrupt IDs, module clock (chip-level plumbing)
// ---------------------------------------------------------------------------

/// USB channel 0 peripheral register base address (TRM §28: SYSCFG0_0 at
/// H'E8010000).  Both channels are identical USB 2.0 high-speed host/function
/// modules.
pub const USB0_BASE: usize = 0xE801_0000;
/// USB channel 1 peripheral register base address (TRM §28: SYSCFG0_1 at
/// H'E8207000).  Same USB 2.0 HS host/function module as channel 0; note that
/// the UCKSEL/UPLLE clock bits exist only in channel 0's SYSCFG0.
pub const USB1_BASE: usize = 0xE820_7000;

/// GIC interrupt ID for USB0 (USB200).
pub const USB0_IRQ: u16 = 73;
/// GIC interrupt ID for USB1 (USB201).
pub const USB1_IRQ: u16 = 74;

/// Return the peripheral base address for the given port (0 or 1).
#[inline]
pub const fn base(port: u8) -> usize {
    if port == 0 { USB0_BASE } else { USB1_BASE }
}

/// Return the GIC interrupt ID for the given port.
#[inline]
pub const fn irq(port: u8) -> u16 {
    USB0_IRQ + port as u16
}

// CPG Standby Control Register 7 — USB clock gates.
// Bit 1: MSTP71 — channel 0 clock (0 = running, 1 = stopped)
// Bit 0: MSTP70 — channel 1 clock (0 = running, 1 = stopped)
const STBCR7: usize = 0xFCFE_0430;
const STBCR7_MSTP71: u8 = 1 << 1; // channel 0
const STBCR7_MSTP70: u8 = 1 << 0; // channel 1

/// Enable the clock for USB module `port` (0 or 1) in CPG STBCR7.
///
/// Channel 0 hosts the USB PLL (UPLLE/UCKSEL) shared by both channels, so
/// bringing up channel 1 (port 1) requires clocking channel 0 as well.  Per HW
/// manual §42.2.7 procedure (2): "Set MSTP71 to 0 … Note: When channel 1 is to
/// be used, also set the MSTP70 bit to 0."  Clearing only MSTP70 for port 1
/// would leave channel 0 gated, so the driver's UPLLE write to channel 0's
/// SYSCFG0 lands on an unclocked register block and the PLL never enables.
///
/// # Safety
/// Writes to memory-mapped CPG registers. Must not be called concurrently
/// with other STBCR7 writers.
pub unsafe fn module_clock_enable(port: u8) {
    unsafe {
        let bits: u8 = if port == 0 {
            STBCR7_MSTP71 // ch0 only
        } else {
            STBCR7_MSTP71 | STBCR7_MSTP70 // ch1 also needs ch0 (shared PLL)
        };
        let cur = core::ptr::read_volatile(STBCR7 as *const u8);
        core::ptr::write_volatile(STBCR7 as *mut u8, cur & !bits);
        // Dummy read to flush write buffer (required by HW manual §10).
        let _ = core::ptr::read_volatile(STBCR7 as *const u8);
    }
}

/// Stop the clock for USB module `port` in CPG STBCR7.
///
/// # Safety
/// Writes to memory-mapped CPG registers.
pub unsafe fn module_clock_disable(port: u8) {
    unsafe {
        // Stop only this port's own channel.  Port 1 must NOT gate channel 0
        // (MSTP71): channel 0 hosts the shared PLL and may still serve port 0.
        let bit: u8 = if port == 0 { STBCR7_MSTP71 } else { STBCR7_MSTP70 };
        let cur = core::ptr::read_volatile(STBCR7 as *const u8);
        core::ptr::write_volatile(STBCR7 as *mut u8, cur | bit);
        let _ = core::ptr::read_volatile(STBCR7 as *const u8);
    }
}

/// GIC interrupt priority for USB0/USB1.
///
/// Must be numerically less than 31 (the GICC_PMR threshold) to pass the
/// CPU-interface priority filter.  Matches the priority used by UART/SDHI.
const USB_IRQ_PRIORITY: u8 = 10;

/// Enable the GIC interrupt for USB module `port`.
///
/// Sets the interrupt priority before enabling, so the CPU-interface
/// priority filter (GICC_PMR = 31) does not block delivery.
///
/// # Safety
/// Writes to GIC distributor registers.
pub unsafe fn int_enable(port: u8) {
    unsafe {
        crate::gic::set_priority(irq(port), USB_IRQ_PRIORITY);
        crate::gic::enable(irq(port));
    }
}

/// Disable the GIC interrupt for USB module `port`.
///
/// # Safety
/// Writes to GIC distributor registers.
pub unsafe fn int_disable(port: u8) {
    unsafe {
        crate::gic::disable(irq(port));
    }
}

// ---------------------------------------------------------------------------
// Mode markers
// ---------------------------------------------------------------------------

/// Mode marker for device (peripheral) operation.
pub struct Device;

/// Mode marker for host operation.
pub struct Host;

// ---------------------------------------------------------------------------
// UsbPort handle
// ---------------------------------------------------------------------------

/// A handle representing ownership of one RUSB1 port in a given `MODE`.
///
/// Created by [`init_device_mode`] or [`init_host_mode`]; not constructible
/// directly.  Dropping this type does not disable the hardware — call
/// [`UsbPort::into_device_mode`] or [`UsbPort::into_host_mode`] explicitly to
/// switch modes at runtime.
pub struct UsbPort<MODE> {
    port: u8,
    _mode: PhantomData<MODE>,
}

impl<M> UsbPort<M> {
    /// The hardware port index (0 or 1).
    pub fn port_index(&self) -> u8 {
        self.port
    }
}

impl UsbPort<Device> {
    /// Re-enter host mode.  Disables pull-up, reconfigures DCFM, re-enables.
    ///
    /// # Safety
    /// No active USB traffic must be in progress.
    pub unsafe fn into_host_mode(self) -> (UsbPort<Host>, Rusb1HostDriver) {
        unsafe {
            quiesce_port(self.port);
            let hd = Rusb1HostDriver::new(self.port);
            (
                UsbPort {
                    port: self.port,
                    _mode: PhantomData,
                },
                hd,
            )
        }
    }
}

impl UsbPort<Host> {
    /// Re-enter device mode.
    ///
    /// # Safety
    /// No active USB traffic must be in progress.
    pub unsafe fn into_device_mode(self) -> (UsbPort<Device>, Rusb1Driver) {
        unsafe {
            quiesce_port(self.port);
            let drv = Rusb1Driver::new(self.port);
            (
                UsbPort {
                    port: self.port,
                    _mode: PhantomData,
                },
                drv,
            )
        }
    }
}

// ---------------------------------------------------------------------------
// Convenience initialisation functions
// ---------------------------------------------------------------------------

/// Enable the USB clock, initialise the RUSB1 hardware in **device** mode,
/// and return an ownership handle plus the `embassy-usb-driver` driver.
///
/// Call once per port; typically at startup before `embassy_usb::UsbDevice`
/// is created.
///
/// # Safety
/// Must only be called once per port.  Caller must ensure the PLL (UPLLE) and
/// the USB clock select (UCKSEL) have been configured in SYSCFG0 before this
/// call if an external clock is used (the Deluge board uses the internal PLL).
pub unsafe fn init_device_mode(port: u8) -> (UsbPort<Device>, Rusb1Driver) {
    unsafe {
        module_clock_enable(port);
        let drv = Rusb1Driver::new(port);
        (
            UsbPort {
                port,
                _mode: PhantomData,
            },
            drv,
        )
    }
}

/// Enable the USB clock, initialise the RUSB1 hardware in **host** mode, and
/// return an ownership handle plus the [`Rusb1HostDriver`].
///
/// The driver implements [`embassy_usb_driver::host::UsbHostDriver`] and is
/// ready to be wrapped in `embassy_usb_host::UsbHost` for enumeration.
///
/// # Safety
/// Must only be called once per port.
pub unsafe fn init_host_mode(port: u8) -> (UsbPort<Host>, Rusb1HostDriver) {
    unsafe {
        module_clock_enable(port);
        let hd = Rusb1HostDriver::new(port);
        (
            UsbPort {
                port,
                _mode: PhantomData,
            },
            hd,
        )
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Disconnect a device-mode port from the host and disable the SIE.
///
/// Clears the D+ pull-up (host sees SE0 / unplug), masks all USB interrupts, and
/// clears `USBE`.  Use this to leave a USB mode cleanly; a later
/// [`init_device_mode`] re-attaches and re-enumerates.
///
/// # Safety
/// Writes USB registers; no transfer may be in flight.
pub unsafe fn disconnect(port: u8) {
    unsafe {
        use regs::{Rusb1Regs, SYSCFG_DPRPU, SYSCFG_USBE, rmw, wr};
        int_disable(port);
        let regs = Rusb1Regs::ptr(port);
        // Pull-up off: host sees a disconnect.
        rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_DPRPU, 0);
        wr(core::ptr::addr_of_mut!((*regs).intenb0), 0);
        wr(core::ptr::addr_of_mut!((*regs).brdyenb), 0);
        wr(core::ptr::addr_of_mut!((*regs).bempenb), 0);
        // Clear USBE to reset the SIE.
        let cur = regs::rd(core::ptr::addr_of!((*regs).syscfg0));
        wr(core::ptr::addr_of_mut!((*regs).syscfg0), cur & !SYSCFG_USBE);
    }
}

/// Bring a port to a quiescent state (USBE=0, interrupts off) before a mode
/// switch.
unsafe fn quiesce_port(port: u8) {
    unsafe {
        use regs::{Rusb1Regs, SYSCFG_USBE, wr};
        int_disable(port);
        let regs = Rusb1Regs::ptr(port);
        wr(core::ptr::addr_of_mut!((*regs).intenb0), 0);
        wr(core::ptr::addr_of_mut!((*regs).brdyenb), 0);
        wr(core::ptr::addr_of_mut!((*regs).bempenb), 0);
        // Clear USBE to reset the SIE.
        let cur = regs::rd(core::ptr::addr_of!((*regs).syscfg0));
        wr(core::ptr::addr_of_mut!((*regs).syscfg0), cur & !SYSCFG_USBE);
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn base_addresses() {
        assert_eq!(USB0_BASE, 0xE801_0000);
        assert_eq!(USB1_BASE, 0xE820_7000);
    }

    #[test]
    fn irq_ids() {
        assert_eq!(irq(0), 73);
        assert_eq!(irq(1), 74);
    }

    #[test]
    fn base_fn() {
        assert_eq!(base(0), USB0_BASE);
        assert_eq!(base(1), USB1_BASE);
    }
}
