//! SDRAM initialisation for RZ/A1L (Deluge board).
//!
//! Configures the Bus State Controller (BSC) for the 64 MB SDRAM part
//! (Micron MT48LC16M16A2P-75 equivalent) on CS3, and sets the GPIO pin-mux
//! for all SDRAM address/data/control pins.
//!
//! This is a direct port of `resetprg.c` (pin-mux section) and
//! `bsc_userdef.c::userdef_bsc_cs2_init(0)` (64 MB chip, 10-bit columns).
//!
//! ## Pin-mux assignments
//! All SDRAM pins use mux function 1.
//!
//! | Port | Pins       | Function        |
//! |------|------------|-----------------|
//! | P3   | 0–14       | Address A0–A14  |
//! | P5   | 0–15       | Data D0–D15     |
//! | P2   | 0          | CS3#            |
//! | P2   | 1          | RAS#            |
//! | P2   | 2          | CAS#            |
//! | P2   | 3          | CKE             |
//! | P2   | 4          | WE0#            |
//! | P2   | 5          | WE1#            |
//! | P2   | 6          | RD/!WR          |
//!
//! ## BSC register addresses (RZ/A1L HW Manual §8)
//! BSC base: 0x3FFF_C000  (struct `st_bsc`)
//!
//! | Register | Offset | Absolute        |
//! |----------|--------|-----------------|
//! | CS2BCR   | 0x0C   | 0x3FFF_C00C     |
//! | CS3BCR   | 0x10   | 0x3FFF_C010     |
//! | CS2WCR   | 0x30   | 0x3FFF_C030     |
//! | CS3WCR   | 0x34   | 0x3FFF_C034     |
//! | SDCR     | 0x4C   | 0x3FFF_C04C     |
//! | RTCSR    | 0x50   | 0x3FFF_C050     |
//! | RTCOR    | 0x58   | 0x3FFF_C058     |
//!
//! SDRAM mode registers (CS2 @ 0x3FFF_D040, CS3 @ 0x3FFF_E040) are written
//! as 16-bit accesses to latch the JEDEC mode register into the DRAM.

use rza1l_hal::gpio::set_pin_mux;

// BSC register addresses
const CS2BCR: usize = 0x3FFF_C00C;
const CS3BCR: usize = 0x3FFF_C010;
const CS3WCR: usize = 0x3FFF_C034;
const SDCR: usize = 0x3FFF_C04C;
const RTCSR: usize = 0x3FFF_C050;
const RTCOR: usize = 0x3FFF_C058;

// SDRAM mode register access addresses (write-only, triggers MRS command)
const SDRAM_MODE_CS2: usize = 0x3FFF_D040;
const SDRAM_MODE_CS3: usize = 0x3FFF_E040;

/// Initialise SDRAM: configure GPIO pin-mux then set up the BSC.
///
/// After this call, the 64 MB SDRAM window at 0x0C00_0000–0x0FFF_FFFF is
/// accessible. Call [`crate::mmu::init_and_enable`] first so the BSC
/// registers at 0x3FFF_Cxxx are in the strongly-ordered I/O region.
///
/// # Safety
/// Writes to memory-mapped GPIO, BSC, and SDRAM mode registers.
/// Must not be called while the SDRAM is actively in use.
pub unsafe fn init() {
    unsafe {
        log::debug!("sdram: init");
        // ---- SDRAM address pins: P3[0..14] mux1 ----
        for pin in 0..=14u8 {
            set_pin_mux(3, pin, 1);
        }

        // ---- SDRAM data pins: P5[0..15] mux1 ----
        for pin in 0..=15u8 {
            set_pin_mux(5, pin, 1);
        }

        // ---- SDRAM control pins: P2[0..6] mux1 ----
        // CS3#, RAS#, CAS#, CKE, WE0#, WE1#, RD/!WR
        for pin in 0..=6u8 {
            set_pin_mux(2, pin, 1);
        }

        // ---- BSC configuration (from userdef_bsc_cs2_init, ramSize=0 = 64 MB) ----
        log::debug!("sdram: pin-mux done, configuring BSC");

        // CS2BCR / CS3BCR: SDRAM, 16-bit bus, 0 idle cycles between W/R and W/W.
        wr32(CS2BCR, 0x0000_4C00);
        wr32(CS3BCR, 0x0000_4C00);

        // CS3WCR: precharge 1 cycle, tRCD 1 cycle, CAS latency 2,
        //         auto-precharge startup 2 cycles, tRC 5 cycles.
        wr32(CS3WCR, 0x0000_0088);

        // SDCR: CS2 & CS3 both 13 rows, CS2 = 9 col, CS3 = 10 col (64 MB chip),
        //       auto-refresh on, auto-precharge mode.
        // 0x00110912: CS2 col=9-bit, CS3 col=10-bit.
        // NOTE: this value is hardcoded for the Deluge's Micron MT48LC16M16A2P-75
        // (64 MB, 13-row, 10-col).  Adjust SDCR for any other SDRAM density.
        wr32(SDCR, 0x0011_0912);

        // RTCOR: refresh timer constant — 7.64 µs / 240 ns ≈ 128 cycles.
        // Written with key 0xA55A in the upper half-word.
        wr32(RTCOR, 0xA55A_0080);

        // RTCSR: initialisation sequence start, clock = B-phy/4, refresh once.
        // The BSC SDRAM controller automatically issues the required PRECHARGE ALL
        // and ≥2 AUTO REFRESH cycles before accepting the Mode Register Set (MRS)
        // command below.  No explicit CPU delay is needed — the BSC hardware manages
        // the full JEDEC power-up sequence via its state machine.
        wr32(RTCSR, 0xA55A_0008);

        // SDRAM mode register: burst-length 1, sequential, CAS-latency 2.
        // A write to the mode address (offset 0x040 from CS base, BA0=1)
        // generates a Mode Register Set (MRS) command.
        log::debug!("sdram: BSC configured, sending MRS");
        wr16(SDRAM_MODE_CS2, 0);
        wr16(SDRAM_MODE_CS3, 0);
        log::debug!("sdram: ready");
    }
}

#[inline(always)]
unsafe fn wr32(addr: usize, val: u32) {
    unsafe {
        core::ptr::write_volatile(addr as *mut u32, val);
    }
}

#[inline(always)]
unsafe fn wr16(addr: usize, val: u16) {
    unsafe {
        core::ptr::write_volatile(addr as *mut u16, val);
    }
}
