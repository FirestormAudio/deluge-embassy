//! RUSB1 USB driver — public module façade.
//!
//! ## Quick start — device mode
//!
//! ```rust,no_run
//! use deluge_bsp::usb::{init_device_mode, UsbMode};
//!
//! let (port, driver) = unsafe { init_device_mode(0) };
//! // pass `driver` to `embassy_usb::UsbDevice::new(driver, config, ...)`
//! ```
//!
//! ## Quick start — host mode
//!
//! ```rust,no_run
//! use deluge_bsp::usb::init_host_mode;
//! use embassy_usb_host::UsbHost;
//!
//! let (port, host) = unsafe { init_host_mode(0) };
//! let mut usb_host = UsbHost::new(host);
//! let speed = usb_host.wait_for_connection().await;
//! let (dev_desc, addr, _) = usb_host.enumerate(speed, &mut [0u8; 256]).await.unwrap();
//! ```
//!
//! ## ISR wiring
//!
//! You must call `dcd_int_handler` (device mode) or `hcd_int_handler` (host
//! mode) from your GIC interrupt dispatcher:
//!
//! ```rust,no_run
//! use deluge_bsp::usb::{dcd_int_handler, hcd_int_handler};
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

pub mod classes;
pub mod driver;
pub mod fifo;
pub mod host;
pub mod pipe;
pub mod regs;

pub use driver::{
    Rusb1Bus, Rusb1ControlPipe, Rusb1Driver, Rusb1EndpointIn, Rusb1EndpointOut, dcd_int_handler,
};
pub use host::{Rusb1Channel, Rusb1HostDriver, hcd_int_handler};

use core::marker::PhantomData;
use rza1l_hal::rusb1;

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
        rusb1::module_clock_enable(port);
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
/// ready to be wrapped in [`embassy_usb_host::UsbHost`] for enumeration.
///
/// # Safety
/// Must only be called once per port.
pub unsafe fn init_host_mode(port: u8) -> (UsbPort<Host>, Rusb1HostDriver) {
    unsafe {
        rusb1::module_clock_enable(port);
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

/// Bring a port to a quiescent state (USBE=0, interrupts off) before a mode
/// switch.
unsafe fn quiesce_port(port: u8) {
    unsafe {
        use regs::{Rusb1Regs, SYSCFG_USBE, wr};
        rusb1::int_disable(port);
        let regs = Rusb1Regs::ptr(port);
        wr(core::ptr::addr_of_mut!((*regs).intenb0), 0);
        wr(core::ptr::addr_of_mut!((*regs).brdyenb), 0);
        wr(core::ptr::addr_of_mut!((*regs).bempenb), 0);
        // Clear USBE to reset the SIE.
        let cur = regs::rd(core::ptr::addr_of!((*regs).syscfg0));
        wr(core::ptr::addr_of_mut!((*regs).syscfg0), cur & !SYSCFG_USBE);
    }
}
