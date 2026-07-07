//! Attack/Release envelope, gate- or trigger-driven. Serial → scalar. Mirrors
//! the prototype's K_ENV state machine.

use crate::In;

#[derive(Clone, Copy, PartialEq)]
pub enum Stage {
    Idle,
    Attack,
    Sustain,
    Release,
}

#[derive(Clone, Copy)]
pub struct Ar {
    level: f32,
    stage: Stage,
    oneshot: bool,
}

impl Ar {
    pub fn new() -> Ar {
        Ar {
            level: 0.0,
            stage: Stage::Idle,
            oneshot: false,
        }
    }

    /// Gate on → attack (then sustain); gate off → release.
    pub fn gate(&mut self, on: bool) {
        self.oneshot = false;
        self.stage = if on { Stage::Attack } else { Stage::Release };
    }

    /// One-shot: attack then immediately release, no sustain.
    pub fn trigger(&mut self) {
        self.oneshot = true;
        self.stage = Stage::Attack;
    }

    pub fn process(&mut self, attack: In, release: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            let atk = attack.at(i).max(0.0001);
            let rel = release.at(i).max(0.0001);
            match self.stage {
                Stage::Attack => {
                    self.level += dt / atk;
                    if self.level >= 1.0 {
                        self.level = 1.0;
                        self.stage = if self.oneshot {
                            Stage::Release
                        } else {
                            Stage::Sustain
                        };
                    }
                }
                Stage::Sustain => self.level = 1.0,
                Stage::Release => {
                    self.level -= dt / rel;
                    if self.level <= 0.0 {
                        self.level = 0.0;
                        self.stage = Stage::Idle;
                    }
                }
                Stage::Idle => self.level = 0.0,
            }
            *s = self.level;
        }
    }
}

impl Default for Ar {
    fn default() -> Self {
        Ar::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::In;
    use proptest::prelude::*;

    proptest! {
        /// P0 gate (spec §8): after `gate(true)`, for any attack/release
        /// time constants, every sample of a rendered block is finite and
        /// stays within [0, 1.0001] (the envelope's level is clamped to
        /// [0, 1] each sample; the tiny slack covers the sustain branch's
        /// exact-1.0 assignment and float rounding).
        #[test]
        fn ar_output_is_finite_and_bounded(
            attack in 0.0001f32..=2.0,
            release in 0.0001f32..=2.0,
        ) {
            let mut env = Ar::new();
            env.gate(true);
            let mut out = [0.0f32; 64];
            let dt = 1.0 / 48_000.0;
            env.process(In::K(attack), In::K(release), dt, &mut out);
            for s in out {
                prop_assert!(s.is_finite());
                prop_assert!(s >= 0.0 && s <= 1.0001);
            }
        }
    }

    #[test]
    fn ar_rises_on_gate_and_falls_on_release() {
        let mut env = Ar::new();
        let dt = 1.0 / 1000.0;
        let mut out = [0.0f32; 100];
        env.gate(true);
        env.process(In::K(0.05), In::K(0.05), dt, &mut out); // 50ms attack
        assert!(out[0] < out[99]); // rising
        assert!(out[99] > 0.9); // reached near top over 100ms
        let mut out2 = [0.0f32; 100];
        env.gate(false);
        env.process(In::K(0.05), In::K(0.05), dt, &mut out2);
        assert!(out2[0] > out2[99]); // falling
    }

    #[test]
    fn ar_trigger_is_one_shot() {
        let mut env = Ar::new();
        let dt = 1.0 / 1000.0;
        let mut out = [0.0f32; 300];
        env.trigger();
        env.process(In::K(0.01), In::K(0.01), dt, &mut out); // 10ms a, 10ms r
        assert!(out[5] > 0.0); // attacked
        assert!(out[299] < 1e-3); // released back to zero without a gate-off
    }
}
