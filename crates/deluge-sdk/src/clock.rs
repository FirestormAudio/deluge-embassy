//! Analog clock I/O: the trigger-clock input jack and a software clock output.
//!
//! [`ClockIn`] wraps the existing edge-counting driver
//! ([`deluge_bsp::trigger_clock`]) behind the SDK's take-once handle style.
//! [`ClockOut`] has no dedicated jack — it pulses one of the V-trig gate outputs
//! (see [`Gate`](crate::Gate)), so the channel it claims should not also be
//! driven through `Gate`.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_time::{Duration, Instant, Timer};

// ── Clock input ─────────────────────────────────────────────────────────────

/// Bring up the trigger-clock input's edge-counting IRQ once. No-op on the
/// host simulator (there is no trigger-clock jack).
fn ensure_clock_in_init() {
    static DONE: AtomicBool = AtomicBool::new(false);
    if DONE.swap(true, Ordering::Relaxed) {
        return;
    }
    crate::plat::clock_in_init();
}

/// The analog trigger-clock **input** jack.
///
/// Taken once from [`Deluge::clock_in`](crate::Deluge::clock_in). Each external
/// pulse advances an edge counter; [`tick`](ClockIn::tick) awaits the next one
/// and reports the interval since the previous tick (handy for tempo).
///
/// Host: there is no trigger-clock jack in the simulator, so the input never
/// pulses. `tick` waits forever, `count`/`last_edge` report nothing.
pub struct ClockIn {
    /// Embassy-time tick of the previously observed edge, for interval math.
    prev_ticks: Option<u64>,
}

impl ClockIn {
    pub(crate) fn new() -> Self {
        ensure_clock_in_init();
        Self { prev_ticks: None }
    }

    /// Await the next external clock pulse, returning the interval since the
    /// previous tick (or `None` on the first tick, when there's no prior edge).
    /// Never arrives on the host simulator.
    pub async fn tick(&mut self) -> Option<Duration> {
        let now = crate::plat::clock_in_wait_edge().await;
        let interval = self
            .prev_ticks
            .map(|p| Duration::from_ticks(now.saturating_sub(p)));
        self.prev_ticks = Some(now);
        interval
    }

    /// Total number of pulses seen since boot (always 0 on the host simulator).
    #[inline]
    pub fn count(&self) -> u32 {
        crate::plat::clock_in_count()
    }

    /// The time of the most recent pulse, or `None` if none has arrived yet
    /// (always `None` on the host simulator).
    #[inline]
    pub fn last_edge(&self) -> Option<Instant> {
        crate::plat::clock_in_last_edge()
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
        crate::plat::gate_set(self.channel, true);
        Timer::after(self.pulse_width).await;
        crate::plat::gate_set(self.channel, false);
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
