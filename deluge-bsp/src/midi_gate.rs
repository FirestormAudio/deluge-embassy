//! MIDI/CV gate one-shot timer using MTU2 channel 2.
//!
//! The Deluge uses a hardware timer to schedule precise gate-off (or gate-on)
//! events that must occur at a sub-millisecond offset within an audio render
//! window — too fine-grained for the Embassy `Timer` abstraction, which has
//! ~30 µs resolution and requires executor context.
//!
//! ## Mechanism
//! MTU2 channel 2, clocked at P0/64 ≈ **520.833 kHz** (~1.92 µs per tick), is
//! configured in one-shot compare-match mode.  When the counter reaches TGRA
//! it automatically resets (CCLR = TGRA), fires the `TGI2A` GIC interrupt
//! (ID 150, priority 5), and calls `handler()`, which:
//!
//! 1. Clears the MTU2 compare-match flag (TSR.TGFA).
//! 2. Reads the pending action from `STATE`.
//! 3. Executes the action (e.g. de-assert a gate GPIO).
//! 4. Disarms the timer (stops ch2, masks GIC ID 150).
//!
//! ## Tick frequency
//! `MTU2_P0_HZ` (33.333 MHz) / 64 = **520 833 Hz** → 1 tick ≈ **1.92 µs**.
//! Maximum one-shot delay: 65535 ticks ≈ **125.9 ms**.
//!
//! ## Usage
//! ```rust,no_run
//! use deluge_bsp::midi_gate;
//!
//! // Once at startup (after gic::init() and mtu2::enable_write()):
//! unsafe { midi_gate::setup(); }
//!
//! // At render time, to de-assert gate 0 in 520 ticks (~1 ms):
//! unsafe { midi_gate::schedule_gate_off(0, 520); }
//!
//! // To cancel a pending shot (e.g. note extended):
//! unsafe { midi_gate::cancel(); }
//! ```

use core::sync::atomic::{AtomicU8, Ordering};
use rza1l_hal::mtu2;

/// MTU2 channel used for one-shot gate timing.
const CH: u8 = 2;

/// Prescaler: P0 / 64 ≈ 520 kHz.
const PRESCALER: u16 = 64;

/// GIC priority for TGI2A (must be lower-number / higher-priority than USB
/// at 9, to ensure gate events are not delayed behind USB servicing).
const PRIORITY: u8 = 5;

/// Sentinel value meaning "no pending action".
const NO_ACTION: u8 = 0xFF;

// ---------------------------------------------------------------------------
// Shared state — written before arm(), read inside the ISR.
// ---------------------------------------------------------------------------

/// Gate channel to de-assert when the timer fires (0–3), or `NO_ACTION`.
/// Written by the scheduling side with `Release`; read by the ISR with `Acquire`.
static GATE_CH: AtomicU8 = AtomicU8::new(NO_ACTION);

// ---------------------------------------------------------------------------
// Interrupt handler — called by the GIC dispatch from the IRQ exception handler.
// Must be `fn()` (no arguments, no return value) to match `gic::Handler`.
// ---------------------------------------------------------------------------

fn handler() {
    // Safety: all register accesses are within the MTU2 driver's domain; no
    // other code touches ch2 registers after setup() completes.
    unsafe {
        // 1. Clear the compare-match flag immediately so the GIC does not
        //    re-trigger.  mtu2::clear_flag loops until TGFA reads 0.
        mtu2::clear_flag(CH);

        // 2. Read the pending action with Acquire, pairing with the Release
        //    store in schedule_gate_off().  Swap back to NO_ACTION atomically
        //    so a concurrent cancel() or re-schedule is race-free.
        let gate_ch = GATE_CH.swap(NO_ACTION, Ordering::Acquire);

        // 3. Execute the action.
        if gate_ch != NO_ACTION {
            crate::cv_gate::gate_set(gate_ch, false); // de-assert: gate OFF
        }

        // 4. Disarm: stop ch2, mask GIC 150.  Safe to call from within the
        //    ISR — disarm() does not touch TSR (flag already cleared) and the
        //    GIC mask takes effect after ICCEOIR is written by the assembly
        //    IRQ stub (the handler returns before that write occurs, so there
        //    is no self-deadlock on the GIC line).
        mtu2::disarm(CH);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Configure MTU2 channel 2 for one-shot gate timing.
///
/// Registers the ISR with the GIC and writes the channel's TCR.  The timer
/// is left stopped; it is started by [`schedule_gate_off`].
///
/// **Must be called:**
/// - After [`rza1l_hal::gic::init`] (so the GIC is ready for handler registration).
/// - After [`rza1l_hal::mtu2::enable_write`] (so channel registers are writable).
/// - Before [`cortex_ar::interrupt::enable`] (so no IRQ fires before the
///   handler is in place).
///
/// # Safety
/// Writes to memory-mapped MTU2 and GIC registers.
pub unsafe fn setup() {
    unsafe {
        mtu2::setup_one_shot(CH, PRESCALER, handler, PRIORITY);
    }
}

/// Schedule a gate-off event `ticks` from now on gate channel `gate_ch`.
///
/// Arms MTU2 ch2 to fire after `ticks` P0/64 ticks (~1.92 µs each).  The
/// ISR will de-assert `gate_ch` when the timer expires.
///
/// - `gate_ch`: hardware gate output to turn off (0–3).
/// - `ticks`: number of ticks (1–65535).  Minimum useful value is a few ticks
///   to allow for ISR latency.
///
/// Calling this again before the timer fires cancels the previous target and
/// re-arms with the new parameters (the ISR will see the updated gate channel).
///
/// # Safety
/// Writes to memory-mapped MTU2 and GIC registers.  `gate_ch` must be in
/// range 0–3.
pub unsafe fn schedule_gate_off(gate_ch: u8, ticks: u16) {
    unsafe {
        // Store the action before arming; pairs with the Acquire in handler().
        GATE_CH.store(gate_ch, Ordering::Release);
        mtu2::arm(CH, ticks);
    }
}

/// Cancel a pending gate-off shot without firing.
///
/// If the timer has already fired this is a no-op (the handler already cleared
/// `GATE_CH`).  If the timer is still running it is stopped and disarmed before
/// the ISR can execute the gate action.
///
/// # Safety
/// Writes to memory-mapped MTU2 and GIC registers.
pub unsafe fn cancel() {
    unsafe {
        // Disarm first so the ISR cannot begin executing between the store and the
        // disarm.  On single-core bare-metal this is sequentially consistent, but
        // the ordering makes the intent explicit.
        mtu2::disarm(CH);
        GATE_CH.store(NO_ACTION, Ordering::Release);
    }
}

/// Tick frequency of the one-shot timer (Hz): P0 / 64.
///
/// Use this to convert a duration to ticks:
/// `ticks = duration_us * TICK_HZ / 1_000_000`
pub const TICK_HZ: u32 = rza1l_hal::mtu2::MTU2_P0_HZ / PRESCALER as u32;
