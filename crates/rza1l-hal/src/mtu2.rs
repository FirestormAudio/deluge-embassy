//! Multi-Function Timer Pulse Unit 2 (MTU2) driver for the RZ/A1L.
//!
//! The RZ/A1L MTU2 has five 16-bit timer channels (0–4), all clocked by P0
//! (33.333 MHz).  Channels can run as free-running counters or configured for
//! one-shot compare-match use.
//!
//! ## Register layout peculiarity
//! The MTU2 register block at `0xFCFF_0000` is non-contiguous: channels 2, 3,
//! and 4 share one sub-block (starting at +0x000), while channels 0 and 1
//! occupy separate sub-blocks starting at +0x300 and +0x380.  All addresses
//! are derived from the `st_mtu2` struct in `mtu2_iodefine.h`.
//!
//! ## TSTR (Timer Start Register)
//! One shared TSTR register at `0xFCFF_0280` controls which channels run.
//! **Write access to all channel registers is locked at reset** — call
//! [`enable_write`] once (sets `TRWER = 1`) before touching any channel.
//!
//! ## Usage (free-running counters)
//! ```rust,no_run
//! # use rza1l_hal::mtu2;
//! unsafe {
//!     mtu2::enable_write();                   // unlock MTU2 registers
//!     mtu2::start_free_running(1, 1);         // ch1 @ 33.3 MHz (superfast)
//!     mtu2::start_free_running(0, 64);        // ch0 @ 520 kHz  (fast)
//!     mtu2::start_free_running(4, 1024);      // ch4 @ ~32.6 kHz (slow)
//!
//!     let t0 = mtu2::count(0);
//!     // do work...
//!     let elapsed = mtu2::count(0).wrapping_sub(t0);
//! }
//! ```
//!
//! ## Usage (one-shot)
//! ```rust,no_run
//! # use rza1l_hal::mtu2;
//! // `gic::Handler` is a safe `fn()`, so the unsafe work goes inside.
//! fn gate_handler() { unsafe { mtu2::clear_flag(2) } /* ... */ }
//! unsafe {
//!     mtu2::enable_write();
//!     mtu2::setup_one_shot(2, 64, gate_handler, 5); // configure ch2
//!     // later, when you want to schedule an event:
//!     mtu2::arm(2, 520);    // fires after 520 ticks ≈ 1 ms at /64
//!     // handler is called, then:
//!     mtu2::disarm(2);      // in handler or after timeout
//! }
//! ```

// ---------------------------------------------------------------------------
// Register addresses (absolute)
// Derived from struct st_mtu2 at base 0xFCFF_0000 (mtu2_iodefine.h).
// ---------------------------------------------------------------------------

// Per-channel TCR (Timer Control Register) — 8-bit
//   bits [2:0] = TPSC (prescaler select)
//   bits [7:5] = CCLR (counter-clear source; 0b001 = clear on TGRA match)
const TCR: [usize; 5] = [
    0xFCFF_0300, // ch0
    0xFCFF_0380, // ch1
    0xFCFF_0000, // ch2
    0xFCFF_0200, // ch3
    0xFCFF_0201, // ch4
];

// Per-channel TIER (Timer Interrupt Enable Register) — 8-bit
//   bit 0 = TGIEA: enable compare-match-A interrupt
const TIER: [usize; 5] = [
    0xFCFF_0304, // ch0
    0xFCFF_0384, // ch1
    0xFCFF_0004, // ch2
    0xFCFF_0208, // ch3
    0xFCFF_0209, // ch4
];

// Per-channel TSR (Timer Status Register) — 8-bit
//   bit 0 = TGFA: compare-match-A flag (write 0 to clear)
const TSR: [usize; 5] = [
    0xFCFF_0305, // ch0
    0xFCFF_0385, // ch1
    0xFCFF_0005, // ch2
    0xFCFF_022C, // ch3
    0xFCFF_022D, // ch4
];

// Per-channel TCNT (Timer Counter) — 16-bit, read/write
const TCNT: [usize; 5] = [
    0xFCFF_0306, // ch0
    0xFCFF_0386, // ch1
    0xFCFF_0006, // ch2
    0xFCFF_0210, // ch3
    0xFCFF_0212, // ch4
];

// Per-channel TGRA (Timer General Register A) — 16-bit compare value
const TGRA: [usize; 5] = [
    0xFCFF_0308, // ch0
    0xFCFF_0388, // ch1
    0xFCFF_0008, // ch2
    0xFCFF_0218, // ch3
    0xFCFF_021C, // ch4
];

// Shared registers
const TSTR: usize = 0xFCFF_0280; // Timer Start Register (8-bit)
const TRWER: usize = 0xFCFF_0284; // Timer Read/Write Enable Register (8-bit)

// TSTR bit for each channel (CST0..CST4).
// Channels 0–2 use bits 0–2; channels 3–4 use bits 6–7.
const CST: [u8; 5] = [0x01, 0x02, 0x04, 0x40, 0x80];

// ---------------------------------------------------------------------------
// Public constants
// ---------------------------------------------------------------------------

/// P0 clock frequency driving MTU2 (Hz).
///
/// Deluge 13.2256 MHz EXTAL: P0φ = 13,225,625 × 30 / 12 = 33,064,062.5 Hz.
pub const MTU2_P0_HZ: u32 = 33_064_062;

/// GIC interrupt IDs for the compare-match-A (TGInA) interrupt on channels 0–4.
/// Source: `devdrv_intc.h`.
pub const TGIA_IRQ: [u16; 5] = [139, 146, 150, 154, 159];

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Encode the TPSC field in TCR for the given channel and prescaler.
///
/// Valid prescalers:
/// - All channels: 1, 4, 16, 64
/// - Ch1, 3, 4 only: 256
/// - Ch2, 3, 4 only: 1024
///
/// Returns 0 (prescaler /1) for invalid combinations.
fn tpsc(ch: u8, prescaler: u16) -> u8 {
    match prescaler {
        1 => 0b000,
        4 => 0b001,
        16 => 0b010,
        64 => 0b011,
        256 => match ch {
            1 => 0b110,
            3 | 4 => 0b100,
            _ => 0b000,
        },
        1024 => match ch {
            2 => 0b111,
            3 | 4 => 0b101,
            _ => 0b000,
        },
        _ => 0b000,
    }
}

// Register access goes through the MMIO seam so init/arm sequences are testable
// (real volatile on firmware, a shadow + log under test).
#[inline(always)]
unsafe fn rd8(addr: usize) -> u8 {
    unsafe { crate::mmio::read8(addr) }
}
#[inline(always)]
unsafe fn wr8(addr: usize, val: u8) {
    unsafe { crate::mmio::write8(addr, val) }
}
#[inline(always)]
unsafe fn rd16(addr: usize) -> u16 {
    unsafe { crate::mmio::read16(addr) }
}
#[inline(always)]
unsafe fn wr16(addr: usize, val: u16) {
    unsafe { crate::mmio::write16(addr, val) }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Unlock MTU2 register write access.
///
/// The MTU2 write gate (`TRWER`) is cleared at reset, blocking writes to all
/// channel registers.  Call this once during startup before any other MTU2
/// function.  The MTU2 clock must already be running (enabled by
/// [`rza1::stb::init`], which sets STBCR3 bit 2 = 0).
///
/// # Safety
/// Writes to the memory-mapped TRWER register.
pub unsafe fn enable_write() {
    unsafe {
        wr8(TRWER, 1);
    }
}

/// Configure channel `ch` as a free-running up-counter and start it.
///
/// The counter increments from 0, wrapping at 0xFFFF (65535).  No interrupt
/// is generated.  The counter is stopped first so TCR can be written, then
/// restarted.
///
/// `prescaler` must be one of: 1, 4, 16, 64 (all channels), 256 (ch1/3/4),
/// 1024 (ch2/3/4).  Invalid combinations default to /1.
///
/// # Safety
/// Writes to memory-mapped MTU2 registers. [`enable_write`] must have been
/// called first.
pub unsafe fn start_free_running(ch: u8, prescaler: u16) {
    unsafe {
        if ch > 4 {
            return;
        }

        // Stop the channel while we configure TCR.
        wr8(TSTR, rd8(TSTR) & !CST[ch as usize]);

        // TCR: TPSC only, CCLR = 0 (counter not cleared on any match).
        wr8(TCR[ch as usize], tpsc(ch, prescaler));

        // Disable compare-match interrupt.
        wr8(TIER[ch as usize], 0);

        // Start.
        wr8(TSTR, rd8(TSTR) | CST[ch as usize]);
    }
}

/// Stop channel `ch`.
///
/// Clears the corresponding CST bit in TSTR.  The counter value is preserved.
///
/// # Safety
/// Writes to memory-mapped MTU2 registers.
pub unsafe fn stop(ch: u8) {
    unsafe {
        if ch > 4 {
            return;
        }
        wr8(TSTR, rd8(TSTR) & !CST[ch as usize]);
    }
}

/// Read the current 16-bit counter value of channel `ch`.
///
/// For a free-running channel this wraps every 65536 / effective_frequency.
/// Use [`u16::wrapping_sub`] for elapsed-time calculations.
///
/// # Safety
/// Reads from memory-mapped MTU2 registers.
pub unsafe fn count(ch: u8) -> u16 {
    unsafe {
        if ch > 4 {
            return 0;
        }
        rd16(TCNT[ch as usize])
    }
}

/// Configure channel `ch` for one-shot compare-match use.
///
/// This function:
/// 1. Stops the channel.
/// 2. Writes TCR with the prescaler and CCLR = TGRA (counter reset on compare
///    match), making it naturally one-shot when paired with [`arm`]/[`disarm`].
/// 3. Disables the compare-match interrupt locally (TIER = 0).
/// 4. Registers `handler` with the GIC at `TGIA_IRQ[ch]` with the given
///    `priority` (0 = highest, 31 = lowest), but does **not** enable it in
///    the GIC — [`arm`] does that.
///
/// Call once during startup.  [`arm`] / [`disarm`] are the run-time controls.
///
/// # Safety
/// Writes to memory-mapped MTU2 and GIC registers.  [`enable_write`] and
/// [`crate::gic::init`] must have been called first.
pub unsafe fn setup_one_shot(ch: u8, prescaler: u16, handler: crate::gic::Handler, priority: u8) {
    unsafe {
        if ch > 4 {
            return;
        }

        // Stop channel.
        wr8(TSTR, rd8(TSTR) & !CST[ch as usize]);

        // TCR: prescaler + CCLR = 0b001 (clear TCNT on TGRA match).
        wr8(TCR[ch as usize], tpsc(ch, prescaler) | (1 << 5));

        // Start with interrupts off.
        wr8(TIER[ch as usize], 0);

        // Register handler with GIC (priority, target CPU, but NOT enabled yet).
        let irq = TGIA_IRQ[ch as usize];
        crate::gic::register(irq, handler);
        crate::gic::set_priority(irq, priority);
    }
}

/// Arm channel `ch` for a one-shot event `tgra` ticks from now.
///
/// Resets the counter to 0, writes `tgra` as the compare value, clears any
/// stale flag, enables the GIC interrupt, and starts the counter.  The
/// registered handler is called when `TCNT` reaches `tgra`; after that the
/// counter resets to 0 (because TCR.CCLR = TGRA) and would fire again unless
/// [`disarm`] is called from the handler.
///
/// Typically called from audio-render context to schedule a MIDI gate edge.
///
/// # Safety
/// Writes to memory-mapped MTU2 registers and GIC ISENABLER.
pub unsafe fn arm(ch: u8, tgra: u16) {
    unsafe {
        if ch > 4 {
            return;
        }

        // Stop while reconfiguring.
        wr8(TSTR, rd8(TSTR) & !CST[ch as usize]);

        // Write compare value then reset counter.
        wr16(TGRA[ch as usize], tgra);
        wr16(TCNT[ch as usize], 0);

        // Clear any pending compare-match flag (write 0 to TGFA bit).
        clear_flag(ch);

        // Enable the GIC source so the handler fires.
        crate::gic::enable(TGIA_IRQ[ch as usize]);

        // Enable the channel's own interrupt output.
        wr8(TIER[ch as usize], 1); // TGIEA = 1

        // Start.
        wr8(TSTR, rd8(TSTR) | CST[ch as usize]);
    }
}

/// Clear the compare-match-A flag (TGFA) in channel `ch`'s TSR.
///
/// Must be called from the interrupt handler before returning, or the GIC
/// will immediately re-trigger.  Matches the loop logic in the C BSP
/// (`timerClearCompareMatchTGRA`): reads TSR until TGFA is clear.
///
/// # Safety
/// Reads and writes to memory-mapped MTU2 registers.
pub unsafe fn clear_flag(ch: u8) {
    unsafe {
        if ch > 4 {
            return;
        }
        let addr = TSR[ch as usize];
        // Loop until the flag is clear (normally 1–2 reads; bounded to 64 iterations
        // so a continuously re-firing timer cannot trap the CPU here indefinitely).
        for _ in 0..64u8 {
            let val = rd8(addr);
            if val & 0x01 == 0 {
                break;
            }
            wr8(addr, val & !0x01);
        }
    }
}

/// Disarm channel `ch`: stop the counter and disable the GIC interrupt.
///
/// Safe to call from within the interrupt handler (after [`clear_flag`]) or
/// from task context after a cancelled shot.
///
/// # Safety
/// Writes to memory-mapped MTU2 registers and GIC ICENABLER.
pub unsafe fn disarm(ch: u8) {
    unsafe {
        if ch > 4 {
            return;
        }

        // Disable the channel's interrupt output first.
        wr8(TIER[ch as usize], 0);

        // Stop the counter.
        wr8(TSTR, rd8(TSTR) & !CST[ch as usize]);

        // Mask the GIC source.
        crate::gic::disable(TGIA_IRQ[ch as usize]);
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn tpsc_common_prescalers() {
        // 1/4/16/64 are valid on every channel and encode identically.
        for ch in 0..5u8 {
            assert_eq!(tpsc(ch, 1), 0b000);
            assert_eq!(tpsc(ch, 4), 0b001);
            assert_eq!(tpsc(ch, 16), 0b010);
            assert_eq!(tpsc(ch, 64), 0b011);
        }
    }

    #[test]
    fn tpsc_channel_specific_prescalers() {
        // /256: ch1 -> 0b110; ch3,4 -> 0b100; others invalid.
        assert_eq!(tpsc(1, 256), 0b110);
        assert_eq!(tpsc(3, 256), 0b100);
        assert_eq!(tpsc(4, 256), 0b100);
        // /1024: ch2 -> 0b111; ch3,4 -> 0b101.
        assert_eq!(tpsc(2, 1024), 0b111);
        assert_eq!(tpsc(3, 1024), 0b101);
        assert_eq!(tpsc(4, 1024), 0b101);
    }

    #[test]
    fn tpsc_invalid_combinations_fall_back_to_div1() {
        // /256 not available on ch0/ch2; /1024 not on ch0/ch1; bogus values too.
        assert_eq!(tpsc(0, 256), 0b000);
        assert_eq!(tpsc(2, 256), 0b000);
        assert_eq!(tpsc(0, 1024), 0b000);
        assert_eq!(tpsc(1, 1024), 0b000);
        assert_eq!(tpsc(0, 3), 0b000);
    }

    #[test]
    fn channel_start_bits_are_unique() {
        // TSTR CST bits must be one-hot and distinct per channel.
        let mut seen = 0u8;
        for &b in CST.iter() {
            assert_eq!(b.count_ones(), 1, "each CST bit is one-hot");
            assert_eq!(seen & b, 0, "CST bits must not overlap");
            seen |= b;
        }
    }

    #[test]
    fn p0_clock_matches_deluge_extal() {
        // 13.2256 MHz EXTAL -> P0phi = 13_225_625 * 30 / 12 = 33_064_062.5 Hz.
        // (See the deluge-clocks notes: P0phi ~= 33.064 MHz, not 33.33.)
        assert_eq!(MTU2_P0_HZ, 33_064_062);
        assert_eq!(TGIA_IRQ.len(), 5);
    }

    // ── register-effect tests via the MMIO seam ─────────────────────────────
    use crate::mmio;

    #[test]
    fn start_free_running_configures_tcr_and_sets_cst() {
        mmio::test::reset();
        let ch = 2u8;
        unsafe { start_free_running(ch, 16) };
        // TCR holds the prescaler code for /16 (= 0b010).
        assert_eq!(mmio::test::peek8(TCR[ch as usize]), tpsc(ch, 16));
        assert_eq!(
            mmio::test::peek8(TIER[ch as usize]),
            0,
            "compare-match IRQ off"
        );
        // The channel's CST start bit is set in TSTR.
        assert_eq!(mmio::test::peek8(TSTR) & CST[ch as usize], CST[ch as usize]);
    }

    #[test]
    fn stop_clears_only_this_channels_cst_bit() {
        mmio::test::reset();
        // Pretend channels 1 and 3 are both running.
        mmio::test::poke8(TSTR, CST[1] | CST[3]);
        unsafe { stop(3) };
        assert_eq!(
            mmio::test::peek8(TSTR),
            CST[1],
            "ch1 still running, ch3 stopped"
        );
    }

    #[test]
    fn out_of_range_channel_is_a_noop() {
        mmio::test::reset();
        unsafe { start_free_running(9, 1) };
        unsafe { stop(9) };
        assert!(mmio::test::log().is_empty(), "ch>4 must touch no registers");
    }
}
