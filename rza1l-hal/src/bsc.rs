//! Bus State Controller (BSC) chip-select area timing for RZ/A1L.
//!
//! The BSC divides the external address space into six areas (CS0–CS5).
//! CS2 and CS3 are configured for SDRAM in [`crate::sdram`].  This module
//! handles CS0 and CS1, which the Renesas BSP configures for a 16-bit
//! external-bus device (NOR flash timing from `bsc_userdef.c`).
//!
//! ## BSC register addresses (RZ/A1L HW Manual §8)
//! BSC base: `0x3FFF_C000` (struct `st_bsc`, field `CMNCR` first).
//!
//! | Register | Offset | Absolute         |
//! |----------|--------|-----------------|
//! | CS0BCR   | 0x04   | `0x3FFF_C004`   |
//! | CS1BCR   | 0x08   | `0x3FFF_C008`   |
//! | CS0WCR   | 0x28   | `0x3FFF_C028`   |
//! | CS1WCR   | 0x2C   | `0x3FFF_C02C`   |
//!
//! (BSC struct layout: `CMNCR` at 0x00, six `CSnBCR` at 0x04–0x18,
//!  twelve padding bytes, six `CSnWCR` at 0x28–0x3C, then SDRAM regs.)
//!
//! ## Timing values
//! Taken verbatim from `userdef_bsc_cs0_init()` and `userdef_bsc_cs1_init()`
//! in the Renesas `bsc_userdef.c` sample BSP:
//!
//! | Field | Value | Meaning |
//! |-------|-------|---------|
//! | `CSnBCR` | `0x1000_0C00` | 16-bit bus, 1 idle cycle between W/R and W/W |
//! | `CSnWCR` | `0x0000_0B40` | 1.5-cycle setup, 6 wait cycles, 0.5-cycle teardown |
//!
//! These must be called after the MMU is enabled (so the BSC register window
//! at `0x3FFF_Cxxx` is in the strongly-ordered I/O region) and before any
//! accesses to the CS0/CS1 address spaces.

// BSC register addresses.
const CS0BCR: usize = 0x3FFF_C004;
const CS1BCR: usize = 0x3FFF_C008;
const CS0WCR: usize = 0x3FFF_C028;
const CS1WCR: usize = 0x3FFF_C02C;

// Timing values from Renesas bsc_userdef.c.
// CSxBCR: bus size = 16-bit (IW[31:28]=0x1), 1 idle cycle (TYPE[11:9]=0x6, ICY[3:0]=0x0).
const BCR_16BIT_1IDLE: u32 = 0x1000_0C00;
// CSxWCR (non-SDRAM): setup 1.5 cyc, access 6 wait, teardown 0.5 cyc.
const WCR_6WAIT: u32 = 0x0000_0B40;

#[inline(always)]
unsafe fn wr32(addr: usize, val: u32) {
    unsafe {
        core::ptr::write_volatile(addr as *mut u32, val);
    }
}

/// Configure CS0 for 16-bit external bus access.
///
/// Sets bus timing to match the Renesas RSK+ NOR-flash configuration:
/// 16-bit bus width, 1 idle cycle between accesses, 6 wait-state access
/// cycles.  Safe to call even if no device is connected to CS0 — the timing
/// registers have no side-effects when the area is not accessed.
///
/// Must be called after [`crate::mmu::init_and_enable`] so the BSC register
/// window is in the strongly-ordered memory region.
///
/// # Safety
/// Writes to memory-mapped BSC registers.
pub unsafe fn init_cs0() {
    unsafe {
        wr32(CS0BCR, BCR_16BIT_1IDLE);
        wr32(CS0WCR, WCR_6WAIT);
    }
}

/// Configure CS1 for 16-bit external bus access.
///
/// Identical timing to [`init_cs0`].  CS1 is configured as a second 16-bit
/// external-bus area; on the Deluge this area is not populated.
///
/// # Safety
/// Writes to memory-mapped BSC registers.
pub unsafe fn init_cs1() {
    unsafe {
        wr32(CS1BCR, BCR_16BIT_1IDLE);
        wr32(CS1WCR, WCR_6WAIT);
    }
}
