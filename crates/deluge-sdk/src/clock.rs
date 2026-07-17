//! Analog clock I/O: the trigger-clock input jack and a software clock output.
//!
//! [`ClockIn`] wraps the existing edge-counting driver
//! ([`deluge_bsp::trigger_clock`]) behind the SDK's take-once handle style.
//! [`ClockOut`] has no dedicated jack — it pulses one of the V-trig gate outputs
//! (see [`Gate`](crate::Gate)), so the channel it claims should not also be
//! driven through `Gate`.

#[cfg(target_os = "none")]
use core::future::poll_fn;
use core::sync::atomic::{AtomicBool, Ordering};
#[cfg(target_os = "none")]
use core::task::Poll;

use embassy_time::{Duration, Instant, Timer};

#[cfg(target_os = "none")]
use deluge_bsp::trigger_clock;

// ── Clock input ─────────────────────────────────────────────────────────────

/// The analog trigger-clock **input** jack.
///
/// Taken once from [`Deluge::clock_in`](crate::Deluge::clock_in). Each external
/// pulse advances an edge counter; [`tick`](ClockIn::tick) awaits the next one
/// and reports the interval since the previous tick (handy for tempo).
pub struct ClockIn {
    /// Embassy-time tick of the previously observed edge, for interval math.
    prev_ticks: Option<u64>,
}

#[cfg(target_os = "none")]
impl ClockIn {
    pub(crate) fn new() -> Self {
        static DONE: AtomicBool = AtomicBool::new(false);
        if !DONE.swap(true, Ordering::Relaxed) {
            // SAFETY: runs once. Registers the P1_14/IRQ6 handler and enables the
            // GIC line. Registering lazily here (after global IRQ enable) matches
            // the proven `input()`/encoder precedent.
            unsafe { trigger_clock::irq_init() };
        }
        Self { prev_ticks: None }
    }

    /// Await the next external clock pulse, returning the interval since the
    /// previous tick (or `None` on the first tick, when there's no prior edge).
    pub async fn tick(&mut self) -> Option<Duration> {
        let start = trigger_clock::EDGE_COUNT.load(Ordering::Relaxed);
        poll_fn(|cx| {
            trigger_clock::EDGE_WAKER.register(cx.waker());
            if trigger_clock::EDGE_COUNT.load(Ordering::Relaxed) != start {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        let now = trigger_clock::LAST_EDGE_TICKS.load(Ordering::Relaxed);
        let interval = self
            .prev_ticks
            .map(|p| Duration::from_ticks(now.saturating_sub(p)));
        self.prev_ticks = Some(now);
        interval
    }

    /// Total number of pulses seen since boot.
    #[inline]
    pub fn count(&self) -> u32 {
        trigger_clock::EDGE_COUNT.load(Ordering::Relaxed)
    }

    /// The time of the most recent pulse, or `None` if none has arrived yet.
    #[inline]
    pub fn last_edge(&self) -> Option<Instant> {
        match trigger_clock::LAST_EDGE_TICKS.load(Ordering::Relaxed) {
            0 => None,
            t => Some(Instant::from_ticks(t)),
        }
    }
}

/// Host: there is no trigger-clock jack in the simulator, so the input never
/// pulses. `tick` waits forever, `count`/`last_edge` report nothing.
#[cfg(not(target_os = "none"))]
impl ClockIn {
    pub(crate) fn new() -> Self {
        Self { prev_ticks: None }
    }

    /// Await the next external clock pulse — never arrives in the simulator.
    pub async fn tick(&mut self) -> Option<Duration> {
        core::future::pending::<()>().await;
        None
    }

    /// Total number of pulses seen since boot (always 0 in the simulator).
    #[inline]
    pub fn count(&self) -> u32 {
        0
    }

    /// The time of the most recent pulse (never any in the simulator).
    #[inline]
    pub fn last_edge(&self) -> Option<Instant> {
        None
    }
}

// ── Clock output ────────────────────────────────────────────────────────────

/// Default high time of an emitted clock pulse.
const DEFAULT_PULSE_WIDTH: Duration = Duration::from_millis(5);

/// A software clock **output** driven over one V-trig gate channel.
///
/// Taken once from [`Deluge::clock_out`](crate::Deluge::clock_out), which binds
/// it to a gate channel. Emit ticks manually with [`pulse`](ClockOut::pulse) or
/// free-run with [`run`](ClockOut::run).
pub struct ClockOut {
    channel: u8,
    pulse_width: Duration,
}

impl ClockOut {
    pub(crate) fn new(channel: u8) -> Self {
        // Configure the gate GPIOs (shared one-time bring-up with Cv/Gate).
        crate::cv_gate::ensure_init();
        Self {
            channel,
            pulse_width: DEFAULT_PULSE_WIDTH,
        }
    }

    /// Set the high time of each emitted pulse (default 5 ms). Keep it shorter
    /// than the clock period.
    #[inline]
    pub fn set_pulse_width(&mut self, width: Duration) {
        self.pulse_width = width;
    }

    /// Emit a single clock pulse: assert the gate for the pulse width, then
    /// release it.
    pub async fn pulse(&mut self) {
        // SAFETY: GPIO writes to the gate line this handle owns.
        #[cfg(target_os = "none")]
        unsafe {
            deluge_bsp::cv_gate::gate_set(self.channel, true)
        };
        #[cfg(not(target_os = "none"))]
        crate::host::panel().set_gate(self.channel as usize, true);
        Timer::after(self.pulse_width).await;
        #[cfg(target_os = "none")]
        unsafe {
            deluge_bsp::cv_gate::gate_set(self.channel, false)
        };
        #[cfg(not(target_os = "none"))]
        crate::host::panel().set_gate(self.channel as usize, false);
    }

    /// Free-run: emit a pulse every `period`, forever.
    ///
    /// Pair with [`period_from_bpm`](ClockOut::period_from_bpm) for musical
    /// rates. Consumes the task (like `Audio::process`); run it in its own task
    /// or a `select`/`join` if the app does other work concurrently.
    pub async fn run(&mut self, period: Duration) -> ! {
        loop {
            self.pulse().await;
            // We already waited `pulse_width` inside `pulse()`; wait the rest
            // (zero if the pulse is as long as the period).
            let rest = period
                .checked_sub(self.pulse_width)
                .unwrap_or(Duration::from_ticks(0));
            Timer::after(rest).await;
        }
    }

    /// The period between pulses for a given tempo and pulses-per-beat
    /// resolution (e.g. `period_from_bpm(120.0, 24)` for 24 ppqn at 120 BPM).
    pub fn period_from_bpm(bpm: f32, pulses_per_beat: u32) -> Duration {
        let beats_per_us = bpm / 60.0 / 1_000_000.0;
        let pulses_per_us = beats_per_us * pulses_per_beat as f32;
        let us = if pulses_per_us > 0.0 {
            (1.0 / pulses_per_us) as u64
        } else {
            0
        };
        Duration::from_micros(us)
    }
}
