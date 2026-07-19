//! The always-available escape chord: SHIFT + TRIPLETS + LEARN held ~1s.
//!
//! Pure state machine driven by an external monotonic clock (`now_ms`) so it is
//! fully deterministic under test. Fires exactly once per hold.

use deluge_linux_ui::controls::button::{LEARN, SHIFT, TRIPLETS};

pub struct KillWatch {
    hold_ms: u64,
    shift: bool,
    triplets: bool,
    learn: bool,
    armed_at: Option<u64>,
    fired: bool,
}

impl KillWatch {
    pub fn new(hold_ms: u64) -> Self {
        Self {
            hold_ms,
            shift: false,
            triplets: false,
            learn: false,
            armed_at: None,
            fired: false,
        }
    }

    fn all_held(&self) -> bool {
        self.shift && self.triplets && self.learn
    }

    fn recompute(&mut self, now_ms: u64) {
        if self.all_held() {
            if self.armed_at.is_none() {
                self.armed_at = Some(now_ms);
            }
        } else {
            self.armed_at = None;
            self.fired = false;
        }
    }

    fn maybe_fire(&mut self, now_ms: u64) -> bool {
        if self.fired {
            return false;
        }
        if let Some(start) = self.armed_at {
            if now_ms.saturating_sub(start) >= self.hold_ms {
                self.fired = true;
                return true;
            }
        }
        false
    }

    pub fn on_button(&mut self, id: u8, pressed: bool, now_ms: u64) -> bool {
        match id {
            SHIFT => self.shift = pressed,
            TRIPLETS => self.triplets = pressed,
            LEARN => self.learn = pressed,
            _ => return false,
        }
        self.recompute(now_ms);
        self.maybe_fire(now_ms)
    }

    pub fn poll(&mut self, now_ms: u64) -> bool {
        self.maybe_fire(now_ms)
    }

    pub fn reset(&mut self) {
        *self = KillWatch::new(self.hold_ms);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fires_after_hold_when_all_three_held() {
        let mut kw = KillWatch::new(1000);
        assert!(!kw.on_button(SHIFT, true, 0));
        assert!(!kw.on_button(TRIPLETS, true, 10));
        assert!(!kw.on_button(LEARN, true, 20)); // armed at 20, not yet elapsed
        assert!(!kw.poll(1000)); // 1000-20 = 980 < 1000
        assert!(kw.poll(1020)); // 1020-20 = 1000 >= 1000 -> fire
    }

    #[test]
    fn fires_only_once_per_hold() {
        let mut kw = KillWatch::new(1000);
        kw.on_button(SHIFT, true, 0);
        kw.on_button(TRIPLETS, true, 0);
        kw.on_button(LEARN, true, 0);
        assert!(kw.poll(1000));
        assert!(!kw.poll(1001));
        assert!(!kw.poll(5000));
    }

    #[test]
    fn releasing_before_hold_cancels() {
        let mut kw = KillWatch::new(1000);
        kw.on_button(SHIFT, true, 0);
        kw.on_button(TRIPLETS, true, 0);
        kw.on_button(LEARN, true, 0);
        assert!(!kw.on_button(LEARN, false, 500)); // released early
        assert!(!kw.poll(2000)); // never fires
    }

    #[test]
    fn re_holding_after_release_can_fire_again() {
        let mut kw = KillWatch::new(1000);
        kw.on_button(SHIFT, true, 0);
        kw.on_button(TRIPLETS, true, 0);
        kw.on_button(LEARN, true, 0);
        assert!(kw.poll(1000)); // first fire
        kw.on_button(LEARN, false, 1100); // release
        assert!(!kw.on_button(LEARN, true, 1200)); // re-arm at 1200
        assert!(kw.poll(2200)); // fires again
    }

    #[test]
    fn unrelated_buttons_do_not_arm() {
        let mut kw = KillWatch::new(1000);
        assert!(!kw.on_button(31, true, 0)); // select click
        assert!(!kw.on_button(SHIFT, true, 0));
        assert!(!kw.poll(5000));
    }
}
