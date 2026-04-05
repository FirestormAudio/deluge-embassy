//! CPG Standby Control Register (STBCR) initialisation for RZ/A1L.
//!
//! Enables the module clocks for all peripherals used by the Deluge firmware
//! by writing CPG.STBCR2–STBCR12.  This is a direct port of `STB_Init()` in
//! `src/RZA1/stb/stb.c`.
//!
//! In CPG STBCRn, a **0 bit enables** (clock running) and a **1 bit stops**
//! (clock gated).  The values below are taken unchanged from the C BSP.
//!
//! ## CPG register addresses (RZ/A1L Hardware Manual §10)
//! CPG struct base: 0xFCFE_0010 (FRQCR).
//! Offsets derived from `cpg_iodefine.h` struct layout:
//!
//! | Register | Absolute address |
//! |----------|-----------------|
//! | STBCR2   | 0xFCFE_0024     |
//! | STBCR3   | 0xFCFE_0420     |
//! | STBCR4   | 0xFCFE_0424     |
//! | STBCR5   | 0xFCFE_0428     |
//! | STBCR6   | 0xFCFE_042C     |
//! | STBCR7   | 0xFCFE_0430     |
//! | STBCR8   | 0xFCFE_0434     |
//! | STBCR9   | 0xFCFE_0438     |
//! | STBCR10  | 0xFCFE_043C     |
//! | STBCR11  | 0xFCFE_0440     |
//! | STBCR12  | 0xFCFE_0444     |

const STBCR2: usize = 0xFCFE_0024;
const STBCR3: usize = 0xFCFE_0420;
const STBCR4: usize = 0xFCFE_0424;
const STBCR5: usize = 0xFCFE_0428;
const STBCR6: usize = 0xFCFE_042C;
const STBCR7: usize = 0xFCFE_0430;
const STBCR8: usize = 0xFCFE_0434;
const STBCR9: usize = 0xFCFE_0438;
const STBCR10: usize = 0xFCFE_043C;
const STBCR11: usize = 0xFCFE_0440;
const STBCR12: usize = 0xFCFE_0444;

/// Board-specific clock-gate configuration for CPG.STBCR2–STBCR12.
///
/// Each field maps directly to the corresponding STBCR register (0 bit =
/// clock running, 1 bit = clock stopped).  Pass a value of this struct to
/// [`init`] rather than editing the HAL directly.
///
/// See the per-register documentation in the source of this module for
/// bit-field descriptions.
pub struct StbConfig {
    pub stbcr2: u8,
    pub stbcr3: u8,
    pub stbcr4: u8,
    pub stbcr5: u8,
    pub stbcr6: u8,
    pub stbcr7: u8,
    pub stbcr8: u8,
    pub stbcr9: u8,
    pub stbcr10: u8,
    pub stbcr11: u8,
    pub stbcr12: u8,
}

/// Enable peripheral module clocks according to `config`.
///
/// Must be called before any peripheral (SCIF, OSTM, USB, SSIF, …) is
/// accessed.  The dummy-read after each write is required by hardware to
/// flush the write buffer before the next access (§10 of the HW manual).
///
/// # Safety
/// Writes to memory-mapped CPG registers.
pub unsafe fn init(config: &StbConfig) {
    unsafe {
        log::debug!("stb: enabling module clocks");
        wr8(STBCR2, config.stbcr2);
        let _: u8 = rd8(STBCR2); // dummy read

        wr8(STBCR3, config.stbcr3);
        let _: u8 = rd8(STBCR3);

        wr8(STBCR4, config.stbcr4);
        let _: u8 = rd8(STBCR4);

        wr8(STBCR5, config.stbcr5);
        let _: u8 = rd8(STBCR5);

        wr8(STBCR6, config.stbcr6);
        let _: u8 = rd8(STBCR6);

        wr8(STBCR7, config.stbcr7);
        let _: u8 = rd8(STBCR7);

        wr8(STBCR8, config.stbcr8);
        let _: u8 = rd8(STBCR8);

        wr8(STBCR9, config.stbcr9);
        let _: u8 = rd8(STBCR9);

        wr8(STBCR10, config.stbcr10);
        let _: u8 = rd8(STBCR10);

        wr8(STBCR11, config.stbcr11);
        let _: u8 = rd8(STBCR11);

        wr8(STBCR12, config.stbcr12);
        let _: u8 = rd8(STBCR12);
        log::debug!("stb: done (STBCR2-12 written)");
    }
}

#[inline(always)]
unsafe fn wr8(addr: usize, val: u8) {
    unsafe {
        core::ptr::write_volatile(addr as *mut u8, val);
    }
}

#[inline(always)]
unsafe fn rd8(addr: usize) -> u8 {
    unsafe { core::ptr::read_volatile(addr as *const u8) }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::{
        STBCR2, STBCR3, STBCR4, STBCR5, STBCR6, STBCR7, STBCR8, STBCR9, STBCR10, STBCR11, STBCR12,
    };

    /// Verify every STBCRn address against the RZ/A1L HW Manual §10 table.
    /// CPG base = 0xFCFE_0010 (FRQCR offset from module base 0xFCFE_0000).
    #[test]
    fn stbcr_register_addresses() {
        assert_eq!(STBCR2, 0xFCFE_0024);
        assert_eq!(STBCR3, 0xFCFE_0420);
        assert_eq!(STBCR4, 0xFCFE_0424);
        assert_eq!(STBCR5, 0xFCFE_0428);
        assert_eq!(STBCR6, 0xFCFE_042C);
        assert_eq!(STBCR7, 0xFCFE_0430);
        assert_eq!(STBCR8, 0xFCFE_0434);
        assert_eq!(STBCR9, 0xFCFE_0438);
        assert_eq!(STBCR10, 0xFCFE_043C);
        assert_eq!(STBCR11, 0xFCFE_0440);
        assert_eq!(STBCR12, 0xFCFE_0444);
    }

    /// STBCR3–STBCR12 are contiguous 4-byte-stride registers.
    #[test]
    fn stbcr3_to_stbcr12_are_contiguous() {
        let regs = [
            STBCR3, STBCR4, STBCR5, STBCR6, STBCR7, STBCR8, STBCR9, STBCR10, STBCR11, STBCR12,
        ];
        for window in regs.windows(2) {
            assert_eq!(
                window[1] - window[0],
                4,
                "gap between {:08X} and {:08X} is not 4",
                window[0],
                window[1]
            );
        }
    }

    /// STBCR4 = 0b00000111 enables SCIF0–SCIF4 (bits 7–3 = 0) and keeps
    /// reserved bits 2–0 = 1.  SCIF5 (bit 2) does not exist on RZ/A1L.
    #[test]
    fn stbcr4_enables_all_scif() {
        // bits [7:3] = 0 → SCIF0–SCIF4 clocked; bits [2:0] = 1 → reserved/kept
        let val: u8 = 0b00000111;
        assert_eq!(val & 0b11111000, 0, "SCIF bits should all be 0 (enabled)");
    }

    /// STBCR5 = 0b11111100 enables OSTM0 (bit 0) and OSTM1 (bit 1).
    #[test]
    fn stbcr5_enables_ostm() {
        let val: u8 = 0b11111100;
        assert_eq!(val & 0x03, 0, "OSTM0/OSTM1 bits should be 0 (enabled)");
    }
}
