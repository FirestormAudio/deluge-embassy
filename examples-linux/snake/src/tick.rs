//! Absolute-deadline tick scheduler for the game loop.
//!
//! The snake advances one cell per tick. Deriving that tick from a fresh
//! `recv_timeout(period)` each loop iteration is wrong: every input event
//! returns early and re-arms the wait from zero, so a stream of encoder events
//! (turning any knob) keeps starving the timeout and the snake stalls. `Ticker`
//! instead tracks the next tick as an absolute `Instant`. Events shorten the
//! [`wait`](Ticker::wait) but never move the deadline, so they cannot delay a
//! tick.

use std::time::{Duration, Instant};

pub struct Ticker {
    next_tick: Instant,
}

impl Ticker {
    /// Arm the first tick one `period` after `now`.
    pub fn new(now: Instant, period: Duration) -> Self {
        Self {
            next_tick: now + period,
        }
    }

    /// How long to block for the next event before the tick is due. Zero once
    /// `now` has reached the deadline, so a due tick never waits.
    pub fn wait(&self, now: Instant) -> Duration {
        self.next_tick.saturating_duration_since(now)
    }

    /// If the deadline has arrived, consume exactly one tick — advancing the
    /// deadline by `period` — and return `true`; otherwise return `false`
    /// without changing state. Early wake-ups (input events) hit the `now <
    /// next_tick` guard and leave the deadline untouched, so they can't delay
    /// the tick. After a long stall (the loop was blocked past several
    /// deadlines) the deadline is resynced to `now + period` rather than firing
    /// a catch-up burst of back-to-back ticks.
    pub fn poll(&mut self, now: Instant, period: Duration) -> bool {
        if now < self.next_tick {
            return false;
        }
        self.next_tick += period;
        if self.next_tick <= now {
            self.next_tick = now + period;
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const P: Duration = Duration::from_millis(180);

    #[test]
    fn no_tick_before_the_deadline() {
        let t0 = Instant::now();
        let mut t = Ticker::new(t0, P);
        assert!(!t.poll(t0, P));
        assert!(!t.poll(t0 + P - Duration::from_millis(1), P));
    }

    #[test]
    fn one_tick_at_the_deadline_not_two() {
        let t0 = Instant::now();
        let mut t = Ticker::new(t0, P);
        assert!(t.poll(t0 + P, P));
        // Same instant: the deadline already advanced, so no double-fire.
        assert!(!t.poll(t0 + P, P));
    }

    #[test]
    fn a_burst_of_early_events_does_not_delay_the_tick() {
        // The regression guard for the knob-slows-the-game bug: a flood of early
        // wake-ups (each an input event) between t0 and the deadline must not
        // move when the tick fires.
        let t0 = Instant::now();
        let mut t = Ticker::new(t0, P);
        for i in 0..1000u32 {
            let now = t0 + Duration::from_micros(50) * i; // all within the period
            assert!(now < t0 + P, "test setup: events must stay within one period");
            assert!(!t.poll(now, P), "early event {i} wrongly triggered a tick");
        }
        // The tick still fires exactly at the original deadline.
        assert!(t.poll(t0 + P, P));
    }

    #[test]
    fn ticks_track_elapsed_time_regardless_of_events() {
        // Over 10 periods, exactly 10 ticks fire even though an extra "event"
        // poll happens at every step.
        let t0 = Instant::now();
        let mut t = Ticker::new(t0, P);
        let mut ticks = 0;
        let mut elapsed = Duration::ZERO;
        while elapsed <= 10 * P {
            let now = t0 + elapsed;
            if t.poll(now, P) {
                ticks += 1;
            }
            // A second poll at the same instant models an event wake-up: inert.
            assert!(!t.poll(now, P));
            elapsed += Duration::from_millis(1);
        }
        assert_eq!(ticks, 10);
    }

    #[test]
    fn resyncs_after_a_long_stall_instead_of_bursting() {
        let t0 = Instant::now();
        let mut t = Ticker::new(t0, P);
        // The loop was blocked past five deadlines, then wakes once.
        let late = t0 + 5 * P;
        assert!(t.poll(late, P)); // fires one tick...
        assert!(!t.poll(late, P)); // ...not four more in a burst.
        // The next tick is one period after the late wake, not a backlog drain.
        assert!(!t.poll(late + P - Duration::from_millis(1), P));
        assert!(t.poll(late + P, P));
    }

    #[test]
    fn a_shorter_period_takes_effect_from_the_next_deadline() {
        // Eating food shrinks tick_period. The already-armed deadline is fixed,
        // so the speed-up applies to the interval *after* it — not retroactively.
        let t0 = Instant::now();
        let mut t = Ticker::new(t0, P);
        assert!(t.poll(t0 + P, P)); // tick 1; deadline -> t0 + 2P
        let fast = Duration::from_millis(90);
        // The pending deadline (t0 + 2P) is unchanged by the shorter period.
        assert!(!t.poll(t0 + P + fast, fast));
        assert!(t.poll(t0 + 2 * P, fast)); // tick 2 at the original deadline
        // The interval after it now uses the shorter period.
        assert!(!t.poll(t0 + 2 * P + Duration::from_millis(50), fast));
        assert!(t.poll(t0 + 2 * P + fast, fast)); // tick 3 comes 90ms later
    }
}
