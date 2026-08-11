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
//! OSTM0 is 32-bit and wraps every ~129.9 s. The 64-bit clock is extended **in software**:
//! `raw_ostm_ticks` compares each CNT sample against the last published one and treats CNT going
//! backwards as a wrap, so the extension is monotonic by construction and no interrupt is on the
//! correctness path. [`LAST_RAW`] explains why that matters — the previous ISR-maintained epoch
//! could read an epoch low whenever its interrupt ran late, which stalled every timer-driven task
//! for 129.9 s. The wrap arithmetic lives in [`crate::time_math`], where it is host-tested.
//!
//! ## Alarm
//!
//! `schedule_wake(at, waker)` adds the waker to a software queue and, if it
//! is the earliest pending wake-up, programs OSTM1 in interval mode for
//! `at - now()` ticks. The OSTM1 ISR calls `Queue::handle_alarm`, which
//! wakes all expired tasks and returns the next expiration time; we re-arm
//! for that if needed.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU64, Ordering};
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::Driver;

use crate::time_math::{MAX_ARM_OSTM_TICKS, extend, ostm_ticks_to_us, us_to_ostm_ticks};
use embassy_time_queue_utils::Queue;

use crate::gic;
use crate::ostm;

// GIC interrupt IDs for OSTM0 and OSTM1 (RZ/A1L HW Manual §12)
const OSTM0_IRQ: u16 = 134;
const OSTM1_IRQ: u16 = 135;
const OSTM_IRQ_PRIORITY: u8 = 14;

/// Last value [`OstmDriver::raw_ostm_ticks`] published, as `(epoch << 32) | CNT`.
///
/// The 64-bit clock is extended from the 32-bit OSTM0 counter **in software**, by comparing each
/// CNT sample against the previous one: CNT going backwards means the counter wrapped. The wrap is
/// therefore detected by whoever reads the clock, not by an interrupt.
///
/// # Why not an ISR-maintained epoch counter
///
/// It used to be one (`EPOCH_HI`, incremented by the OSTM0 CMP==0 ISR), read as
/// `(EPOCH_HI << 32) | CNT` under a double-read of EPOCH_HI to catch a wrap racing the two reads.
/// That guard covers the wrong window. It does nothing about the OSTM0 ISR being **late** — pending
/// behind a same-or-higher-priority handler, or a critical section — and while it is late, CNT has
/// already wrapped to a small value whereas the epoch has not advanced. Both reads then agree on a
/// stale epoch, the guard passes, and `now()` silently returns a value **one full epoch (2^32 ticks,
/// 129.9 s) in the past**.
///
/// That is not a cosmetic error. `try_set_alarm` computes `at - now`, so a rewound `now` yields a
/// delta of ~129.9 s, which saturates the u32 tick clamp and arms OSTM1 for its maximum. Every
/// timer-driven task in the system then waits that long. It was observed on device as a total
/// freeze — no audio, no UI, no fault — whose measured duration was 129.900029 s against a
/// theoretical maximum arm of 129.898353 s: a match to five significant figures.
///
/// Software extension removes the ISR from the correctness path: monotonicity is guaranteed by
/// construction, so `now()` can never jump backwards and a spurious epoch-sized delta cannot arise
/// however late any interrupt is.
///
/// # Requirement
///
/// The clock must be read at least once per wrap period (~129.9 s), or a wrap goes unobserved and
/// time loses an epoch. Satisfied structurally — the audio render path reads it continuously — and
/// guaranteed regardless by [`ostm0_overflow_isr`], which exists now only to force one read per
/// wrap.
static LAST_RAW: AtomicU64 = AtomicU64::new(0);

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

    /// Raw OSTM0 tick count as a monotonic 64-bit value, extending the 32-bit counter in software.
    ///
    /// Never returns less than a previously returned value, for any interleaving of callers and any
    /// interrupt latency — see [`LAST_RAW`] for why that property is load-bearing rather than
    /// merely tidy.
    #[inline]
    fn raw_ostm_ticks() -> u64 {
        loop {
            let last = LAST_RAW.load(Ordering::Acquire);
            // Safety: OSTM0 is initialised before interrupts are enabled.
            let cnt = unsafe { ostm::count(0) };
            let val = extend(last, cnt);
            if val <= last {
                // A concurrent reader (or an ISR that preempted us mid-read) already published a
                // value at or ahead of ours. Hand back theirs: it is at least as current as ours,
                // and going backwards is the one thing this function must never do.
                return last;
            }
            match LAST_RAW.compare_exchange_weak(
                last,
                val,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return val,
                // Lost the race; re-read and retry. Bounded in practice: every retry means another
                // caller published, so the loop cannot spin without the clock advancing.
                Err(_) => continue,
            }
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
        // Clamp to MAX_ARM_OSTM_TICKS; a farther alarm is re-armed when that intermediate ISR
        // fires, so capping only costs wakeups, never accuracy.
        let delta_ostm = us_to_ostm_ticks(delta_us).min(MAX_ARM_OSTM_TICKS as u64) as u32;

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
        // ticks × 640 / 21161; the multiply overflows u64 only after ~28
        // years of uptime at 33.064 MHz.
        ostm_ticks_to_us(Self::raw_ostm_ticks())
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

/// OSTM0 compare-match ISR — fires when CNT == 0 (after each 32-bit wrap).
///
/// Reads the clock, and nothing else. The wrap itself is detected in software by
/// [`OstmDriver::raw_ostm_ticks`]; this ISR exists only to guarantee the "read at least once per
/// wrap period" requirement [`LAST_RAW`] documents, so the epoch cannot be skipped on a system that
/// somehow stopped reading the clock for 129.9 s.
///
/// Its latency is therefore no longer a correctness concern — which is the entire point of the
/// change. Previously this ISR owned the epoch, and being late made `now()` jump backwards an epoch.
fn ostm0_overflow_isr() {
    let _ = OstmDriver::raw_ostm_ticks();
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

