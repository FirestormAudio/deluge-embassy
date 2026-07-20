//! Audio jack detection + speaker-amplifier control.

use core::sync::atomic::{AtomicBool, Ordering};

fn ensure_init() {
    static DONE: AtomicBool = AtomicBool::new(false);
    if DONE.swap(true, Ordering::Relaxed) {
        return;
    }
    crate::plat::jacks_init();
}

/// Audio jack-detect inputs and the speaker-amplifier enable.
///
/// Taken once from [`Deluge::jacks`](crate::Deluge::jacks). Reads tell you which
/// jacks are inserted; [`set_speaker`](Jacks::set_speaker) drives the amp, and
/// [`apply_speaker_mute`](Jacks::apply_speaker_mute) applies the stock policy.
///
/// On the host simulator there is no physical panel: all jacks read as not
/// inserted and the speaker control is a no-op.
pub struct Jacks {
    _not_send: crate::NotSend,
}

impl Jacks {
    pub(crate) fn new() -> Self {
        ensure_init();
        Self {
            _not_send: crate::NOT_SEND,
        }
    }

    /// `true` if the headphone jack is inserted.
    #[inline]
    pub fn headphone(&self) -> bool {
        crate::plat::jacks_headphone()
    }

    /// `true` if the line-input jack is inserted.
    #[inline]
    pub fn line_in(&self) -> bool {
        crate::plat::jacks_line_in()
    }

    /// `true` if the microphone jack is inserted.
    #[inline]
    pub fn mic(&self) -> bool {
        crate::plat::jacks_mic()
    }

    /// `true` if the left line-output jack is inserted.
    #[inline]
    pub fn line_out_left(&self) -> bool {
        crate::plat::jacks_line_out_left()
    }

    /// `true` if the right line-output jack is inserted.
    #[inline]
    pub fn line_out_right(&self) -> bool {
        crate::plat::jacks_line_out_right()
    }

    /// Drive the speaker amplifier on (`true`) or off (`false`) directly. No-op
    /// on the host simulator.
    #[inline]
    pub fn set_speaker(&mut self, on: bool) {
        crate::plat::jacks_set_speaker(on);
    }

    /// Apply the stock speaker-mute policy once: enable the amplifier only when
    /// neither the headphone nor either line-output jack is inserted.
    ///
    /// Call this whenever jack state may have changed (e.g. on a poll). Returns
    /// the value written, so callers can log/observe it.
    pub fn apply_speaker_mute(&mut self) -> bool {
        let enable = !(self.headphone() || self.line_out_left() || self.line_out_right());
        self.set_speaker(enable);
        enable
    }
}
