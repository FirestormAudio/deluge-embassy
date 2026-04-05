//! GPIO port registers for the RZ/A1L (see HW Manual §21).
//!
//! Port numbering mirrors the hardware: ports 1..=11 for P1..P11.
//! PMC is the exception — it includes PMC0, so [`pmc(0)`] is valid.
//!
//! ## Register map summary (all 16-bit, stride 4)
//! - Pn    (output data)           : `0xFCFE_3004 + (n-1)*4`
//! - PMn   (mode: 0=out, 1=in)     : `0xFCFE_3304 + (n-1)*4`
//! - PMCn  (mux: 0=GPIO, 1=periph) : `0xFCFE_3400 + n*4`
//! - PIPCn (ctrl: 0=sw, 1=periph)  : `0xFCFE_7204 + (n-1)*4`

use core::convert::Infallible;
use core::marker::PhantomData;

const GPIO_BASE: usize = 0xFCFE_3000;

// ── Address accessors ──────────────────────────────────────────────────────────

/// Port Mode Register n (PMn): bit=0 → output, bit=1 → input.
/// `port` is 1-based (1..=11).
pub fn pm(port: u8) -> *mut u16 {
    (GPIO_BASE + 0x300 + (port as usize) * 4) as *mut u16
}

/// Port Mode Control Register n (PMCn): bit=0 → GPIO, bit=1 → peripheral.
/// `port` is 0-based (0..=11, PMC0 exists).
pub fn pmc(port: u8) -> *mut u16 {
    (GPIO_BASE + 0x400 + (port as usize) * 4) as *mut u16
}

/// Port Data Register n (Pn) — output data.
/// `port` is 1-based (1..=11).
pub fn p(port: u8) -> *mut u16 {
    (GPIO_BASE + (port as usize) * 4) as *mut u16
}

/// Port Input Peripheral Control Register n (PIPCn): bit=0 → software ctrl.
/// Must be 0 for GPIO output. `port` is 1-based (1..=11).
/// Address: PIPC1 = 0xFCFE_7204, stride 4.
pub fn pipc(port: u8) -> *mut u16 {
    (0xFCFE_7204usize + (port as usize - 1) * 4) as *mut u16
}

/// Port Pin Read register n (PPRn) — reflects actual pin voltage regardless of direction.
/// Read-only. `port` is 1-based (1..=11). Address: PPR1 = 0xFCFE_3204.
fn ppr(port: u8) -> *const u16 {
    (GPIO_BASE + 0x200 + (port as usize) * 4) as *const u16
}

/// Port Input Buffer Control register n (PIBCn): bit=1 enables the input buffer.
/// Must be 1 to read external pin state via PPR. `port` is 0-based (PIBC0 exists).
/// Address: PIBC0 = 0xFCFE_7000, PIBC1 = 0xFCFE_7004, stride 4.
fn pibc(port: u8) -> *mut u16 {
    (0xFCFE_7000usize + (port as usize) * 4) as *mut u16
}

// ── Low-level free functions ───────────────────────────────────────────────────

/// Configure a pin as a software-controlled GPIO output.
///
/// # Safety
/// Writes to memory-mapped peripheral registers; must run with the pin not
/// already owned by another driver.
pub unsafe fn set_as_output(port: u8, pin: u8) {
    unsafe {
        // PMC = 0: GPIO (not peripheral multiplexed)
        let v = core::ptr::read_volatile(pmc(port));
        core::ptr::write_volatile(pmc(port), v & !(1u16 << pin));

        // PM = 0: output direction
        let v = core::ptr::read_volatile(pm(port));
        core::ptr::write_volatile(pm(port), v & !(1u16 << pin));

        // PIPC = 0: software (not hardware-peripheral) control
        let v = core::ptr::read_volatile(pipc(port));
        core::ptr::write_volatile(pipc(port), v & !(1u16 << pin));
    }
}

/// Configure a pin as a software-controlled GPIO input.
///
/// Sets PMC=0 (GPIO mode), PM=1 (input direction), PIBC=1 (enable input buffer).
/// After this call, the pin voltage can be read via [`read_pin`].
///
/// # Safety
/// Writes to memory-mapped peripheral registers; must run with the pin not
/// already owned by another driver.
pub unsafe fn set_as_input(port: u8, pin: u8) {
    unsafe {
        // PMC = 0: GPIO (not peripheral multiplexed)
        let v = core::ptr::read_volatile(pmc(port));
        core::ptr::write_volatile(pmc(port), v & !(1u16 << pin));

        // PM = 1: input direction
        let v = core::ptr::read_volatile(pm(port));
        core::ptr::write_volatile(pm(port), v | (1u16 << pin));

        // PIBC = 1: enable input buffer so PPR reflects the live pin state
        let v = core::ptr::read_volatile(pibc(port));
        core::ptr::write_volatile(pibc(port), v | (1u16 << pin));
    }
}

/// Enable the input buffer for a pin already muxed to a peripheral function.
///
/// [`set_pin_mux`] puts the pin in peripheral mode (PMC=1, PIPC=1) but does
/// **not** set PIBC, so the live logic level is not reflected in PPR.  Call
/// this after [`set_pin_mux`] when the ISR or any other code needs to read the
/// pin state via [`read_pin`] (e.g. for quadrature encoder direction decode).
///
/// # Safety
/// Writes to a memory-mapped peripheral register; `port` must be 1-based (1..=11).
pub unsafe fn enable_input_buffer(port: u8, pin: u8) {
    unsafe {
        let v = core::ptr::read_volatile(pibc(port));
        core::ptr::write_volatile(pibc(port), v | (1u16 << pin));
    }
}

/// Read the current logic level of a GPIO pin via the Port Pin Read (PPR) register.
///
/// Works after [`set_as_input`] has been called for the pin.
///
/// # Safety
/// Reads a memory-mapped peripheral register; `port` must be 1-based (1..=11).
pub unsafe fn read_pin(port: u8, pin: u8) -> bool {
    unsafe { (core::ptr::read_volatile(ppr(port)) >> pin) & 1 != 0 }
}

/// Read all 16 pins of a port in a single atomic PPR register read.
///
/// Returns a bitmask; bit `n` corresponds to pin `n` of `port`.
/// Both PIBC bits for the pins of interest must be set before calling this.
///
/// # Safety
/// Reads a memory-mapped peripheral register; `port` must be 1-based (1..=11).
pub unsafe fn read_port(port: u8) -> u16 {
    unsafe { core::ptr::read_volatile(ppr(port)) }
}

/// Drive a GPIO output pin high (`true`) or low (`false`).
///
/// # Safety
/// The pin must have been configured as an output via [`set_as_output`].
pub unsafe fn write(port: u8, pin: u8, high: bool) {
    unsafe {
        let v = core::ptr::read_volatile(p(port));
        if high {
            core::ptr::write_volatile(p(port), v | (1u16 << pin));
        } else {
            core::ptr::write_volatile(p(port), v & !(1u16 << pin));
        }
    }
}

/// Configure a pin as a peripheral function (GPIO pin-mux).
///
/// Mirrors `setPinMux(port, pin, mux)` from `gpio.c`.
///
/// `port`: 1-based (1–11).  `pin`: bit position (0–15).  `mux`: function (1–7).
///
/// # Safety
/// Writes to memory-mapped GPIO registers.
pub unsafe fn set_pin_mux(port: u8, pin: u8, mux: u8) {
    // Base address for port 1 of each mux register group; stride 4 per port.
    const PFC_BASE: usize = 0xFCFE_3504;
    const PFCE_BASE: usize = 0xFCFE_3604;
    const PFCAE_BASE: usize = 0xFCFE_3A04;
    const PMC_BASE: usize = 0xFCFE_3404; // PMC1 = 0x3404, PMCn = +4*(n-1)
    const PIPC_BASE: usize = 0xFCFE_7204;

    fn modify(base: usize, port: u8, pin: u8, set: bool) {
        let addr = (base + (port as usize - 1) * 4) as *mut u16;
        let v = unsafe { core::ptr::read_volatile(addr) };
        let new = if set {
            v | (1u16 << pin)
        } else {
            v & !(1u16 << pin)
        };
        unsafe { core::ptr::write_volatile(addr, new) };
    }

    log::trace!("gpio: set_pin_mux port={} pin={} mux={}", port, pin, mux);
    modify(PFCAE_BASE, port, pin, mux >= 5);
    modify(PFCE_BASE, port, pin, ((mux - 1) >> 1) & 1 != 0);
    modify(PFC_BASE, port, pin, (mux - 1) & 1 != 0);
    modify(PMC_BASE, port, pin, true);
    modify(PIPC_BASE, port, pin, true);
}

// ── Type-state GPIO pins ───────────────────────────────────────────────────────

/// Marker for a pin configured as a software-controlled output.
pub struct Output;
/// Marker for a pin configured as a software-controlled input.
pub struct Input;

/// A GPIO pin with a const-generic port and bit number, typed by direction.
///
/// `PORT` is 1-based (1–11); `BIT` is 0–15.
///
/// Owns no resources; the hardware configuration is managed via the
/// underlying [`set_as_output`] / [`write`] free functions.
pub struct Pin<const PORT: u8, const BIT: u8, MODE>(PhantomData<MODE>);

impl<const PORT: u8, const BIT: u8, MODE> Pin<PORT, BIT, MODE> {
    /// Create a new typed pin handle.
    ///
    /// # Safety
    /// The caller must ensure the pin is not concurrently owned by another
    /// driver and that GPIO clocks are enabled.
    #[inline]
    pub unsafe fn new() -> Self {
        Pin(PhantomData)
    }
}

impl<const PORT: u8, const BIT: u8> Pin<PORT, BIT, Output> {
    /// Configure the underlying hardware as an output and return a typed handle.
    ///
    /// # Safety
    /// Same requirements as [`set_as_output`].
    pub unsafe fn into_output() -> Self {
        unsafe {
            set_as_output(PORT, BIT);
            Pin(PhantomData)
        }
    }
}

impl<const PORT: u8, const BIT: u8> embedded_hal::digital::ErrorType for Pin<PORT, BIT, Output> {
    type Error = Infallible;
}

impl<const PORT: u8, const BIT: u8> embedded_hal::digital::OutputPin for Pin<PORT, BIT, Output> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Infallible> {
        unsafe { write(PORT, BIT, true) };
        Ok(())
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Infallible> {
        unsafe { write(PORT, BIT, false) };
        Ok(())
    }
}

impl<const PORT: u8, const BIT: u8> embedded_hal::digital::StatefulOutputPin
    for Pin<PORT, BIT, Output>
{
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Infallible> {
        let val = unsafe { core::ptr::read_volatile(p(PORT)) };
        Ok(val & (1u16 << BIT) != 0)
    }
    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Infallible> {
        let val = unsafe { core::ptr::read_volatile(p(PORT)) };
        Ok(val & (1u16 << BIT) == 0)
    }
}

impl<const PORT: u8, const BIT: u8> embedded_hal::digital::ErrorType for Pin<PORT, BIT, Input> {
    type Error = Infallible;
}

impl<const PORT: u8, const BIT: u8> embedded_hal::digital::InputPin for Pin<PORT, BIT, Input> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Infallible> {
        // Read PPR (Port Pin Read) — reflects actual pin voltage.
        // p() is the output-latch register and reads back 0 for input-configured pins.
        let val = unsafe { core::ptr::read_volatile(ppr(PORT)) };
        Ok(val & (1u16 << BIT) != 0)
    }
    #[inline]
    fn is_low(&mut self) -> Result<bool, Infallible> {
        let val = unsafe { core::ptr::read_volatile(ppr(PORT)) };
        Ok(val & (1u16 << BIT) == 0)
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    // GPIO_BASE = 0xFCFE_3000
    // Pn   (output data)    = GPIO_BASE + 0x000 + port*4   (1-based port)
    // PMn  (mode)           = GPIO_BASE + 0x300 + port*4   (1-based port)
    // PMCn (mux enable)     = GPIO_BASE + 0x400 + port*4   (0-based port)
    // PIPCn (peripheral)    = 0xFCFE_7204 + (port-1)*4    (1-based port)

    #[test]
    fn gpio_p_addr() {
        assert_eq!(p(1) as usize, 0xFCFE_3004);
        assert_eq!(p(2) as usize, 0xFCFE_3008);
        assert_eq!(p(6) as usize, 0xFCFE_3018);
        assert_eq!(p(11) as usize, 0xFCFE_302C);
    }

    #[test]
    fn gpio_pm_addr() {
        assert_eq!(pm(1) as usize, 0xFCFE_3304);
        assert_eq!(pm(2) as usize, 0xFCFE_3308);
        assert_eq!(pm(6) as usize, 0xFCFE_3318);
        assert_eq!(pm(11) as usize, 0xFCFE_332C);
    }

    #[test]
    fn gpio_pmc_addr() {
        assert_eq!(pmc(0) as usize, 0xFCFE_3400);
        assert_eq!(pmc(1) as usize, 0xFCFE_3404);
        assert_eq!(pmc(6) as usize, 0xFCFE_3418);
        assert_eq!(pmc(11) as usize, 0xFCFE_342C);
    }

    #[test]
    fn gpio_pipc_addr() {
        assert_eq!(pipc(1) as usize, 0xFCFE_7204);
        assert_eq!(pipc(2) as usize, 0xFCFE_7208);
        assert_eq!(pipc(6) as usize, 0xFCFE_7218);
        assert_eq!(pipc(11) as usize, 0xFCFE_722C);
    }

    /// The {PFCAE, PFCE, PFC} 3-bit field encodes `mux - 1` for every valid
    /// mux function (1–7).  This matches the RZ/A1L HW Manual Table 21-x.
    #[test]
    fn gpio_pin_mux_encoding() {
        for mux in 1u8..=7 {
            let pfcae = mux >= 5;
            let pfce = ((mux - 1) >> 1) & 1 != 0;
            let pfc = (mux - 1) & 1 != 0;
            let bits = ((pfcae as u8) << 2) | ((pfce as u8) << 1) | (pfc as u8);
            assert_eq!(bits, mux - 1, "mux={mux}");
        }
    }
}
