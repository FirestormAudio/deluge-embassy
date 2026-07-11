//! Attack/Release envelope, gate- or trigger-driven. Serial → scalar. Mirrors
//! the prototype's K_ENV state machine.

use crate::In;

#[derive(Clone, Copy, PartialEq)]
pub enum Stage {
    Idle,
    Attack,
    Decay,
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

    /// Advance one sample; returns the new level. attack/release in seconds.
    pub fn tick(&mut self, attack: f32, release: f32, dt: f32) -> f32 {
        let atk = attack.max(0.0001);
        let rel = release.max(0.0001);
        match self.stage {
            Stage::Attack => {
                self.level += dt / atk;
                if self.level >= 1.0 {
                    self.level = 1.0;
                    self.stage = if self.oneshot { Stage::Release } else { Stage::Sustain };
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
            Stage::Decay => { /* unreachable for Ar: never entered, no-op */ }
        }
        self.level
    }

    pub fn process(&mut self, attack: In, release: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            *s = self.tick(attack.at(i), release.at(i), dt);
        }
    }
}

impl Default for Ar {
    fn default() -> Self {
        Ar::new()
    }
}

/// Attack/Decay/Sustain/Release envelope, gate-driven. `sustain` is the held
/// level in [0,1]. Serial state machine → scalar (like `Ar`). `Ar` is unchanged;
/// this is an additive full-ADSR sibling.
#[derive(Clone, Copy)]
pub struct Adsr {
    level: f32,
    stage: Stage,
    sustain: f32,
}

impl Adsr {
    pub fn new() -> Adsr {
        Adsr { level: 0.0, stage: Stage::Idle, sustain: 1.0 }
    }

    /// Gate on → attack (then decay → sustain); gate off → release.
    pub fn gate(&mut self, on: bool) {
        self.stage = if on { Stage::Attack } else { Stage::Release };
    }

    /// Enter attack. A percussive AD (no held tail) is achieved with sustain=0.
    pub fn trigger(&mut self) {
        self.stage = Stage::Attack;
    }

    pub fn set_sustain(&mut self, s: f32) {
        self.sustain = s.clamp(0.0, 1.0);
    }

    /// Advance one sample; returns the new level. a/d/r in seconds.
    pub fn tick(&mut self, attack: f32, decay: f32, release: f32, dt: f32) -> f32 {
        match self.stage {
            Stage::Attack => {
                self.level += dt / attack.max(0.0001);
                if self.level >= 1.0 {
                    self.level = 1.0;
                    self.stage = Stage::Decay;
                }
            }
            Stage::Decay => {
                self.level -= dt / decay.max(0.0001);
                if self.level <= self.sustain {
                    self.level = self.sustain;
                    self.stage = Stage::Sustain;
                }
            }
            Stage::Sustain => self.level = self.sustain,
            Stage::Release => {
                self.level -= dt / release.max(0.0001);
                if self.level <= 0.0 {
                    self.level = 0.0;
                    self.stage = Stage::Idle;
                }
            }
            Stage::Idle => self.level = 0.0,
        }
        self.level
    }

    pub fn process(&mut self, attack: In, decay: In, release: In, dt: f32, out: &mut [f32]) {
        for (i, s) in out.iter_mut().enumerate() {
            *s = self.tick(attack.at(i), decay.at(i), release.at(i), dt);
        }
    }
}

impl Default for Adsr {
    fn default() -> Self {
        Adsr::new()
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

    #[test]
    fn ar_tick_matches_expected_steps() {
        // Direct per-sample tick: attack adds dt/atk, release subtracts dt/rel.
        let dt = 0.001;
        let mut a = Ar::new();
        a.gate(true); // Attack
        assert!((a.tick(0.01, 0.05, dt) - 0.1).abs() < 1e-6); // +dt/atk = 0.1
        assert!((a.tick(0.01, 0.05, dt) - 0.2).abs() < 1e-6); // → 0.2
        a.gate(false); // Release
        assert!((a.tick(0.01, 0.05, dt) - 0.18).abs() < 1e-6); // -dt/rel = -0.02 → 0.18
    }
}

#[cfg(test)]
mod adsr_tests {
    use super::*;

    #[test]
    fn adsr_attack_decay_sustain_release() {
        let dt = 1.0 / 48_000.0;
        let mut e = Adsr::new();
        e.set_sustain(0.5);
        e.gate(true);
        // Attack 10ms → reaches ~1.0
        let mut peak = 0.0f32;
        for _ in 0..(0.02 / dt as f64) as usize { peak = peak.max(e.tick(0.01, 0.05, 0.1, dt)); }
        assert!(peak > 0.99, "attack should reach ~1.0, got {peak}");
        // Decay 50ms → settles to sustain 0.5
        let mut lvl = peak;
        for _ in 0..(0.1 / dt as f64) as usize { lvl = e.tick(0.01, 0.05, 0.1, dt); }
        assert!((lvl - 0.5).abs() < 1e-3, "should hold sustain 0.5, got {lvl}");
        // Sustain holds
        for _ in 0..100 { lvl = e.tick(0.01, 0.05, 0.1, dt); }
        assert!((lvl - 0.5).abs() < 1e-3, "sustain must hold, got {lvl}");
        // Release from sustain → 0
        e.gate(false);
        let mut last = lvl;
        for _ in 0..(0.2 / dt as f64) as usize { last = e.tick(0.01, 0.05, 0.1, dt); }
        assert!(last < 1e-3, "release should reach 0, got {last}");
    }

    #[test]
    fn adsr_release_from_current_level_mid_decay() {
        let dt = 1.0 / 48_000.0;
        let mut e = Adsr::new();
        e.set_sustain(0.2);
        e.gate(true);
        // Attack quickly to ~1.0
        for _ in 0..(0.005 / dt as f64) as usize { e.tick(0.001, 1.0, 0.1, dt); }
        // Mid-decay (long decay so we're still above sustain), then release
        let mid = e.tick(0.001, 1.0, 0.1, dt);
        e.gate(false);
        let after = e.tick(0.001, 1.0, 0.1, dt);
        assert!(after < mid, "release must fall from the current level, {after} !< {mid}");
    }

    #[test]
    fn adsr_bounded_0_1() {
        let dt = 1.0 / 48_000.0;
        let mut e = Adsr::new();
        e.set_sustain(0.7);
        e.gate(true);
        for i in 0..8192 {
            if i == 4000 { e.gate(false); }
            let v = e.tick(0.01, 0.03, 0.05, dt);
            assert!((0.0..=1.0).contains(&v), "env out of [0,1]: {v}");
        }
    }
}
