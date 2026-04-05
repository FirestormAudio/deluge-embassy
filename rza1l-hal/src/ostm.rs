//! OS Timer (OSTM) driver for the RZ/A1L.
//!
//! The RZ/A1L has two OSTM channels, each a 32-bit timer clocked by P0
//! (33.33 MHz = CPU / 12).
//!
//! Two operating modes (OSTMnCTL bits):
//!   - Interval timer (CTL = 0x00): counts down from CMP, fires interrupt at 0
//!   - Free-running  (CTL = 0x02): counts up from 0, optionally fires at CMP
//!
//! ## Usage
//! ```rust,no_run
//! # use rza1::ostm;
//! # use rza1::ostm::OSTM_HZ;
//! unsafe {
//!     ostm::enable_clock();          // ungate OSTM0+OSTM1 in CPG
//!     ostm::start_free_running(0);   // start OSTM0 in free-running mode
//!     let t0 = ostm::count(0);
//!     // ... wait ~1 s ...
//!     while ostm::count(0).wrapping_sub(t0) < OSTM_HZ {}
//! }
//! ```

/// OSTM P0 clock frequency (Hz). P0 = CPU / 12 = 400 MHz / 12 = 33.333... MHz.
pub const OSTM_HZ: u32 = 33_333_333;

// ------- Peripheral base addresses -------------------------------------------
const OSTM0_BASE: usize = 0xFCFE_C000;
const OSTM1_BASE: usize = 0xFCFE_C400;

// ------- Register offsets (from OSTMn base) ----------------------------------
const CMP: usize = 0x00; // OSTMnCMP — compare register (32-bit)
const CNT: usize = 0x04; // OSTMnCNT — counter, read-only (32-bit)
const TE: usize = 0x10; // OSTMnTE  — timer enable status (8-bit, read-only)
const TS: usize = 0x14; // OSTMnTS  — timer start: write 1 to start (8-bit)
const TT: usize = 0x18; // OSTMnTT  — timer stop:  write 1 to stop  (8-bit)
const CTL: usize = 0x20; // OSTMnCTL — control: bit1=mode, bit0=int_enable (8-bit)

// CTL mode values
const CTL_INTERVAL: u8 = 0x00; // count-down interval timer, optional interrupt
const CTL_FREE_RUN: u8 = 0x02; // free-running counter up, optional interrupt
const CTL_FREE_RUN_INT: u8 = 0x03; // free-running with compare interrupt

// ------- CPG standby control (clock gating) ----------------------------------
//
// CPG base = 0xFCFE_0010. STBCR5 is at offset 0x418 from the base:
//   STBCR5 address = 0xFCFE_0010 + 0x418 = 0xFCFE_0428
//
// STBCR5 layout (from stb.c comment):
//   bit 7 : SCIM0  stop (1 = stopped)
//   bit 6 : SCIM1  stop
//   bit 5 : [reserved, keep 1]
//   bit 4 : [reserved, keep 1]
//   bit 3 : [reserved, keep 1]
//   bit 2 : [reserved, keep 1]
//   bit 1 : OSTM1  stop  ← MSTP51
//   bit 0 : OSTM0  stop  ← MSTP50
//
// stb.c initializes STBCR5 = 0b11111100 — bits 0 and 1 clear → OSTM0+OSTM1 clocked.
// A dummy read after write is required by the hardware spec.
const STBCR5: usize = 0xFCFE_0428;
/// STBCR5 bit 0: MSTP50 — stop OSTM0 clock (1 = stopped, 0 = running).
const STBCR5_MSTP50: u8 = 1 << 0;
/// STBCR5 bit 1: MSTP51 — stop OSTM1 clock (1 = stopped, 0 = running).
const STBCR5_MSTP51: u8 = 1 << 1;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Ungate OSTM0 and OSTM1 clocks in the CPG module stop register (STBCR5).
///
/// The Renesas `STB_Init()` normally sets `STBCR5 = 0b11111100` which already
/// has bits 0+1 clear (= OSTM0+OSTM1 enabled). This function clears only
/// those two bits so it is safe to call even if the rest of STBCR5 was set
/// by an earlier boot stage.
///
/// In production firmware, prefer [`rza1::stb::init`] which enables all
/// peripheral clocks at once. Call this function directly only in isolated
/// tests or examples that do not call `stb::init()`.
///
/// # Safety
/// Writes to memory-mapped CPG registers. Safe to call once before starting
/// any OSTM channel.
pub unsafe fn enable_clock() {
    unsafe {
        let reg = STBCR5 as *mut u8;
        let val = reg.read_volatile();
        reg.write_volatile(val & !(STBCR5_MSTP50 | STBCR5_MSTP51)); // clear MSTP50 (OSTM0) and MSTP51 (OSTM1)
        let _ = reg.read_volatile(); // mandatory dummy read per HW spec
    }
}

#[inline(always)]
fn base(n: u8) -> usize {
    if n == 0 { OSTM0_BASE } else { OSTM1_BASE }
}

/// Configure OSTM channel `n` (0 or 1) for free-running mode and start it.
///
/// The counter begins at 0 and increments at P0 (~33.33 MHz). No interrupt
/// is generated. Safe to call when the channel is already stopped; if it is
/// running it is stopped first to allow CTL to be written.
///
/// # Safety
/// Writes to memory-mapped OSTM registers. Must only be called after
/// [`enable_clock`].
pub unsafe fn start_free_running(n: u8) {
    unsafe {
        let b = base(n);
        // Stop the timer in case it is already running (CTL can only be written
        // when the timer is stopped — TE must read 0 first).
        let tt = (b + TT) as *mut u8;
        let te = (b + TE) as *const u8;
        tt.write_volatile(1);
        // Wait for the hardware to confirm the timer has stopped.
        while te.read_volatile() != 0 {}

        // Free-running mode, compare interrupt disabled.
        let ctl = (b + CTL) as *mut u8;
        ctl.write_volatile(CTL_FREE_RUN);

        // Start.
        let ts = (b + TS) as *mut u8;
        ts.write_volatile(1);
    }
}

/// Read the current counter value of OSTM channel `n` (0 or 1).
///
/// Increments at ~33.33 MHz. Wraps at `u32::MAX`.
///
/// # Safety
/// Reads from memory-mapped OSTM registers.
#[inline]
pub unsafe fn count(n: u8) -> u32 {
    unsafe {
        let cnt = (base(n) + CNT) as *const u32;
        cnt.read_volatile()
    }
}

/// Busy-wait for approximately `ticks` OSTM counts (~30 ns each at P0).
///
/// Uses OSTM channel 0. [`start_free_running`]`(0)` must have been called first.
///
/// # Safety
/// Reads from memory-mapped OSTM registers.
pub unsafe fn delay_ticks(ticks: u32) {
    unsafe {
        let t0 = count(0);
        while count(0).wrapping_sub(t0) < ticks {}
    }
}

/// Busy-wait for approximately `ms` milliseconds using OSTM channel 0.
///
/// # Safety
/// See [`delay_ticks`].
pub unsafe fn delay_ms(ms: u32) {
    unsafe {
        // OSTM_HZ / 1000 = 33_333 ticks per millisecond
        delay_ticks(ms * (OSTM_HZ / 1000));
    }
}

/// A `DelayNs` implementation backed by the OSTM0 free-running counter.
///
/// [`start_free_running(0)`](start_free_running) must have been called before
/// constructing or using a `Delay`.
pub struct Delay;

impl Delay {
    /// Create a new `Delay`.  Does not start OSTM0; caller must have done
    /// that already via [`start_free_running`]`(0)`.
    #[inline]
    pub fn new() -> Self {
        Delay
    }
}

impl Default for Delay {
    fn default() -> Self {
        Self::new()
    }
}

impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // OSTM_HZ = 33_333_333 ticks/s → 1 tick ≈ 30 ns.
        // ticks = ceil(ns * OSTM_HZ / 1_000_000_000)
        // Use u64 to avoid overflow: max ns = u32::MAX ≈ 4.3 s → 143 M ticks, fits u32.
        let ticks = (ns as u64 * OSTM_HZ as u64).div_ceil(1_000_000_000) as u32;
        unsafe { delay_ticks(ticks) };
    }
}

/// Stop OSTM channel `n`.
///
/// # Safety
/// Writes to memory-mapped OSTM registers.
#[inline]
pub unsafe fn stop(n: u8) {
    unsafe {
        let tt = (base(n) + TT) as *mut u8;
        tt.write_volatile(1);
    }
}

/// Start OSTM channel `n` in free-running mode with a compare-match interrupt.
///
/// The interrupt fires when CNT == `cmp`. After firing, CNT continues counting
/// (one-shot style; the channel stays running). Re-arm by calling this again.
///
/// Use `cmp = 0` to get an overflow notification (fires when CNT wraps to 0).
///
/// # Safety
/// Writes to memory-mapped OSTM registers. Must only be called after
/// [`enable_clock`].
pub unsafe fn start_free_running_cmp(n: u8, cmp: u32) {
    unsafe {
        let b = base(n);
        let tt = (b + TT) as *mut u8;
        tt.write_volatile(1); // stop first so CTL is writeable

        let cmp_reg = (b + CMP) as *mut u32;
        cmp_reg.write_volatile(cmp);

        let ctl = (b + CTL) as *mut u8;
        ctl.write_volatile(CTL_FREE_RUN_INT); // free-running with CMP interrupt

        let ts = (b + TS) as *mut u8;
        ts.write_volatile(1);
    }
}

/// Start OSTM channel `n` in interval mode with a single-shot countdown.
///
/// The channel counts down from `delta_ticks` to 0, fires one interrupt
/// (at the end of counting — NOT at the start), then auto-reloads.
/// Call [`stop`] inside the ISR to prevent the auto-reload.
///
/// # Safety
/// Writes to memory-mapped OSTM registers. Must only be called after
/// [`enable_clock`].
pub unsafe fn start_alarm(n: u8, delta_ticks: u32) {
    unsafe {
        let b = base(n);
        let tt = (b + TT) as *mut u8;
        tt.write_volatile(1); // stop in case already running

        let cmp_reg = (b + CMP) as *mut u32;
        cmp_reg.write_volatile(delta_ticks);

        let ctl = (b + CTL) as *mut u8;
        ctl.write_volatile(CTL_INTERVAL); // interval mode; interrupt fires at end of count (CNT=0), not at start

        let ts = (b + TS) as *mut u8;
        ts.write_volatile(1);
    }
}

// ---------------------------------------------------------------------------
// Owned typed wrapper
// ---------------------------------------------------------------------------

/// Owned handle for OSTM channel `N` (0 or 1).
///
/// Wraps the free functions in an object-oriented API.  The type parameter `N`
/// encodes the channel at compile time, preventing accidental mix-ups.
pub struct Ostm<const N: u8>;

impl<const N: u8> Ostm<N> {
    /// Claim ownership of OSTM channel `N`.
    ///
    /// # Safety
    /// The caller must ensure no other code uses channel `N` concurrently.
    /// [`enable_clock`] must have been called before any method that accesses
    /// hardware registers.
    pub unsafe fn new() -> Self {
        Ostm
    }

    /// Read the current counter value (increments at ~33.33 MHz).
    ///
    /// # Safety
    /// Reads a memory-mapped register.
    pub unsafe fn count(&self) -> u32 {
        unsafe { count(N) }
    }

    /// Configure channel `N` for free-running mode and start it.
    ///
    /// # Safety
    /// Writes memory-mapped registers.
    pub unsafe fn start_free_running(&self) {
        unsafe { start_free_running(N) }
    }

    /// Configure channel `N` for free-running mode with a compare interrupt.
    ///
    /// # Safety
    /// Writes memory-mapped registers.
    pub unsafe fn start_free_running_cmp(&self, cmp: u32) {
        unsafe { start_free_running_cmp(N, cmp) }
    }

    /// Configure channel `N` for single-shot interval mode.
    ///
    /// # Safety
    /// Writes memory-mapped registers.
    pub unsafe fn start_alarm(&self, delta_ticks: u32) {
        unsafe { start_alarm(N, delta_ticks) }
    }

    /// Stop channel `N`.
    ///
    /// # Safety
    /// Writes memory-mapped registers.
    pub unsafe fn stop(&self) {
        unsafe { stop(N) }
    }

    /// Return a [`Delay`] backed by this channel.
    ///
    /// Channel `N` must already be running in free-running mode.
    pub fn delay(&self) -> Delay {
        Delay
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::{OSTM_HZ, OSTM0_BASE, OSTM1_BASE, base};

    #[test]
    fn ostm_hz_is_p0_frequency() {
        // P0 = CPU / 12 = 400 MHz / 12 = 33.333... MHz
        assert_eq!(OSTM_HZ, 33_333_333);
    }

    #[test]
    fn ostm_base_addresses() {
        // RZ/A1L HW Manual: OSTM0 @ 0xFCFEC000, OSTM1 @ 0xFCFEC400
        assert_eq!(OSTM0_BASE, 0xFCFE_C000);
        assert_eq!(OSTM1_BASE, 0xFCFE_C400);
    }

    #[test]
    fn base_selects_correct_channel() {
        assert_eq!(base(0), OSTM0_BASE);
        assert_eq!(base(1), OSTM1_BASE);
    }

    /// 1 ms in OSTM ticks: OSTM_HZ / 1000 = 33_333.
    #[test]
    fn one_ms_ticks() {
        assert_eq!(OSTM_HZ / 1000, 33_333);
    }
}
