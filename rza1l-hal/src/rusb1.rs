//! RUSB1 USB module primitives for RZ/A1L.
//!
//! This module handles only the chip-level concerns that live outside the USB
//! peripheral register space itself:
//!
//! - Module base addresses (USB200 / USB201)
//! - GIC interrupt IDs and enable/disable wrappers
//! - STBCR7 clock gate (USB0 = bit 1, USB1 = bit 0; 0 = clock running)
//!
//! Higher-level register access lives in `deluge_bsp::usb::regs`.

/// USB200 peripheral register base address.
/// USB200 is the High-Speed USB (USBHS) host/function controller on the RZ/A1L.
pub const USB0_BASE: usize = 0xE801_0000;
/// USB201 peripheral register base address.
/// USB201 is the Full-Speed USB (USBFS) host/function controller — distinct from
/// USB200 (USBHS).  The name `USB1` refers to the second port index, not a second
/// USBHS instance.  See RZ/A1L HW Manual §19 (USB Function Module, USB201).
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
// Bit 1: USB0 clock (0 = running, 1 = stopped)
// Bit 0: USB1 clock (0 = running, 1 = stopped)
const STBCR7: usize = 0xFCFE_0430;

/// Enable the clock for USB module `port` (0 or 1) in CPG STBCR7.
///
/// # Safety
/// Writes to memory-mapped CPG registers. Must not be called concurrently
/// with other STBCR7 writers.
pub unsafe fn module_clock_enable(port: u8) {
    unsafe {
        let bit: u8 = if port == 0 { 1 << 1 } else { 1 << 0 };
        let cur = core::ptr::read_volatile(STBCR7 as *const u8);
        core::ptr::write_volatile(STBCR7 as *mut u8, cur & !bit);
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
        let bit: u8 = if port == 0 { 1 << 1 } else { 1 << 0 };
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
