//! Embassy time driver for the RZ/A1L, backed by OSTM0 and OSTM1.
//!
//! ## Hardware layout
//!
//! | Channel | Mode                         | Role                         |
//! |---------|------------------------------|------------------------------|
//! | OSTM0   | Free-running + CMP interrupt | 32-bit monotonic clock base  |
//! | OSTM1   | Interval + interrupt         | One-shot alarm               |
//!
//! ## Clock and tick rate
//!
//! OSTM0 runs at P0 = ~33.333 MHz. The Embassy tick rate is 1 MHz (1 µs per
//! tick), so `now()` returns `ostm0_count / OSTM_PER_US`.
//!
//! ## Overflow handling for `now()`
//!
//! OSTM0 is 32-bit and wraps every ~128 s. An overflow interrupt fires when
//! CNT == 0 (i.e. the cycle just wrapped). The ISR bumps `EPOCH_HI`.
//!
//! `now()` uses a double-read of `EPOCH_HI` around the CNT read to detect the
//! wrap race and retry if necessary.
//!
//! ## Alarm
//!
//! `schedule_wake(at, waker)` adds the waker to a software queue and, if it
//! is the earliest pending wake-up, programs OSTM1 in interval mode for
//! `at - now()` ticks. The OSTM1 ISR calls `Queue::handle_alarm`, which
//! wakes all expired tasks and returns the next expiration time; we re-arm
//! for that if needed.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use crate::gic;
use crate::ostm;

// GIC interrupt IDs for OSTM0 and OSTM1 (RZ/A1L HW Manual §12)
const OSTM0_IRQ: u16 = 134;
const OSTM1_IRQ: u16 = 135;
const OSTM_IRQ_PRIORITY: u8 = 14;

/// OSTM0 ticks per Embassy tick (1 µs at 33.333 MHz).
///
/// Two OSTM ticks ≈ 60 ns; dividing is a lossy integer rounding.
/// Effective resolution: 1/33.333 µs ≈ 30 ns, rounded to 1 µs at output.
const OSTM_PER_US: u64 = crate::ostm::OSTM_HZ as u64 / 1_000_000; // = 33

// ---------------------------------------------------------------------------
// Overflow counter
// ---------------------------------------------------------------------------

/// Upper epoch: each increment represents 2^32 OSTM0 ticks (~128 s).
///
/// The OSTM0 ISR fires when CNT == 0 (immediately after the wrap to 0) and
/// increments this counter.
static EPOCH_HI: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// Driver struct + timer queue
// ---------------------------------------------------------------------------

struct OstmDriver {
    queue: Mutex<RefCell<Queue>>,
}

impl OstmDriver {
    const fn new() -> Self {
        Self {
            queue: Mutex::new(RefCell::new(Queue::new())),
        }
    }

    /// Raw OSTM0 tick count as a 64-bit value by splicing `EPOCH_HI` and CNT.
    ///
    /// Uses a double-read of EPOCH_HI to survive the wrap race: if the high
    /// word changed between the two reads, a wrap occurred around our CNT
    /// sample so we retry.
    #[inline]
    fn raw_ostm_ticks() -> u64 {
        loop {
            let hi1 = EPOCH_HI.load(Ordering::Acquire);
            // Safety: OSTM0 is initialised before interrupts are enabled.
            let cnt = unsafe { ostm::count(0) };
            let hi2 = EPOCH_HI.load(Ordering::Acquire);
            if hi1 == hi2 {
                return ((hi1 as u64) << 32) | (cnt as u64);
            }
            // Wrap raced with our reads — retry.
        }
    }

    /// Try to set OSTM1 to fire `at` embassy-ticks from epoch.
    ///
    /// Returns `false` (alarm already expired) so the caller can call
    /// `Queue::next_expiration` again and retry with a fresh `now`.
    fn try_set_alarm(&self, _cs: &CriticalSection, at: u64) -> bool {
        let now = self.now();
        if at <= now {
            return false;
        }
        // Convert delta from Embassy µs ticks to OSTM0 ticks.
        let delta_us = at - now;
        // Clamp to u32 range (~128 s); if the alarm is farther out it will
        // be re-armed after the intermediate ISR fires.
        let delta_ostm = (delta_us.saturating_mul(OSTM_PER_US)).min(u32::MAX as u64) as u32;

        if delta_ostm == 0 {
            return false;
        }

        // Safety: both channels are clocked and registers accessible.
        unsafe {
            ostm::start_alarm(1, delta_ostm);
            gic::enable(OSTM1_IRQ);
        }
        true
    }
}

impl Driver for OstmDriver {
    fn now(&self) -> u64 {
        Self::raw_ostm_ticks() / OSTM_PER_US
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow_ref_mut(cs);
            if queue.schedule_wake(at, waker) {
                // We now own the alarm slot. Keep trying until we either
                // arm a future alarm or exhaust the queue (all tasks already due).
                let mut next = queue.next_expiration(self.now());
                while !self.try_set_alarm(&cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        });
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: OstmDriver = OstmDriver::new());

// ---------------------------------------------------------------------------
// ISR handlers registered with the GIC
// ---------------------------------------------------------------------------

/// OSTM0 compare-match ISR — fires when CNT == 0 (after 32-bit wrap).
///
/// Increments the epoch counter so `now()` stays monotonic across wraps.
fn ostm0_overflow_isr() {
    EPOCH_HI.fetch_add(1, Ordering::Release);
}

/// OSTM1 interval ISR — fires when the one-shot alarm expires.
fn ostm1_alarm_isr() {
    // Stop OSTM1 immediately to prevent auto-reload in interval mode.
    unsafe { ostm::stop(1) };
    // Disarm the GIC source until the next alarm is programmed.
    unsafe { gic::disable(OSTM1_IRQ) };

    let now = DRIVER.now();
    critical_section::with(|cs| {
        let mut queue = DRIVER.queue.borrow_ref_mut(cs);
        let mut next = queue.next_expiration(now);
        while !DRIVER.try_set_alarm(&cs, next) {
            next = queue.next_expiration(DRIVER.now());
        }
    });
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

/// Initialise the Embassy time driver.
///
/// Configures OSTM0 for overflow-tracked free-running mode (CMP=0 so the
/// interrupt fires immediately after each 32-bit wrap) and OSTM1 as a
/// dormant alarm channel. Registers both ISRs with the GIC.
///
/// # Safety
/// Must be called once, after [`gic::init`] and [`ostm::enable_clock`],
/// but before `cpsie i`. OSTM0 must NOT already be started (this function
/// re-starts it in the required mode).
pub unsafe fn init() {
    unsafe {
        // OSTM0: free-running with CMP=0 interrupt (fires at wrap to 0).
        //
        // `start_free_running_cmp(0, 0)` stops the channel, sets CMP=0, sets
        // CTL to free-running-with-interrupt, then starts.  After this, the
        // channel counts 0 → 1 → … → 0xFFFF_FFFF → [interrupt fires] → 0 → …
        ostm::start_free_running_cmp(0, 0);

        // Register ISRs and set priorities below PMR (default 31 = PMR, never fires).
        gic::register(OSTM0_IRQ, ostm0_overflow_isr);
        gic::register(OSTM1_IRQ, ostm1_alarm_isr);
        gic::set_priority(OSTM0_IRQ, OSTM_IRQ_PRIORITY);
        gic::set_priority(OSTM1_IRQ, OSTM_IRQ_PRIORITY);
        gic::enable(OSTM0_IRQ);
        // OSTM1 will be enabled by try_set_alarm when needed.
    }
}
