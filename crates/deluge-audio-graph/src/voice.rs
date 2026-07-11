//! Note→voice allocation: maps note-on/off events onto the `VOICES` poly lanes,
//! emitting the low-level `Cmd`s (pitch `SetParam` on a `PolyCtrl` + per-voice
//! `GateVoice` on a `PolyAr`). Oldest-LRU voice stealing. Control-plane only —
//! no audio, no heap, no panics.

use crate::cmd::Cmd;
use crate::NodeId;
use deluge_dsp_kernels::poly::VOICES;

/// MIDI note of the `PolyMtof` reference (A4 = 440 Hz). The allocator writes
/// `note - A440_NOTE` (semitones above the reference) to the pitch `PolyCtrl`.
const A440_NOTE: f32 = 69.0;

/// Max envelopes a single allocator can fan a gate out to (e.g. amp + filter
/// envelopes sharing one voice-allocation lane).
pub const MAX_GATES: usize = 4;

/// Semitone offset for unison voice `u` of `count`, spread symmetrically and
/// evenly over ±`detune_cents`. `count <= 1` ⇒ 0.0 (no detune).
fn unison_offset(u: usize, count: usize, detune_cents: f32) -> f32 {
    if count <= 1 { return 0.0; }
    let t = -1.0 + 2.0 * (u as f32) / ((count - 1) as f32); // -1..+1
    t * detune_cents / 100.0
}

/// Pan position for unison voice `u` of `count`, spread symmetrically and evenly
/// over ±`amount` of the stereo field. `count <= 1` ⇒ 0.0 (center). Result ∈
/// [-1.0, +1.0]: -1 = hard left, 0 = center, +1 = hard right.
fn width_offset(u: usize, count: usize, amount: f32) -> f32 {
    if count <= 1 { return 0.0; }
    let t = -1.0 + 2.0 * (u as f32) / ((count - 1) as f32); // -1..+1
    t * amount
}

/// Per-lane lifecycle for release-tail-aware allocation.
#[derive(Clone, Copy, PartialEq)]
enum LaneState {
    Free,      // never used (never reclaimed — the allocator has no time source)
    Held(u8),  // sounding a held note (the MIDI note)
    Releasing, // note-off fired, gate off, tail still ringing — lane stays occupied
}

pub struct VoiceAllocator {
    pitch_node: NodeId,           // a PolyCtrl: SetParam(pitch_node, lane, note - 69)
    gates: [NodeId; MAX_GATES],   // PolyAr/PolyAdsr nodes: GateVoice(gates[i], lane, on/off)
    n_gates: usize,                // number of valid entries in `gates`
    vel_node: Option<NodeId>, // a PolyCtrl carrying per-voice velocity, or None
    sum_node: NodeId, // a StereoVoiceSum: SetParam(sum_node, lane+1, pan) when width_amount != 0.0
    lane_state: [LaneState; VOICES],
    lane_age: [u32; VOICES],
    clock: u32,
    unison: usize,
    detune_cents: f32,
    width_amount: f32,
}

impl VoiceAllocator {
    pub fn new(
        pitch_node: NodeId,
        gates: [NodeId; MAX_GATES],
        n_gates: usize,
        vel_node: Option<NodeId>,
        sum_node: NodeId,
    ) -> VoiceAllocator {
        VoiceAllocator {
            pitch_node,
            gates,
            n_gates,
            vel_node,
            sum_node,
            lane_state: [LaneState::Free; VOICES],
            lane_age: [0; VOICES],
            clock: 0,
            unison: 1,
            detune_cents: 0.0,
            width_amount: 0.0,
        }
    }

    pub fn set_unison(&mut self, n: usize) { self.unison = n.clamp(1, VOICES); }
    pub fn set_detune(&mut self, cents: f32) { self.detune_cents = cents; }
    pub fn set_width(&mut self, amount: f32) { self.width_amount = amount; }

    /// Fan a gate transition out to every configured envelope, preserving order.
    fn gate_all(&self, lane: usize, on: bool, emit: &mut impl FnMut(Cmd)) {
        for g in &self.gates[..self.n_gates] {
            emit(Cmd::GateVoice { node: *g, voice: lane as u8, on });
        }
    }

    /// Choose a lane for a new note: a never-used lane first, else the lane released
    /// longest ago (most decayed), else steal the oldest-allocated held lane.
    fn pick_lane(&self) -> usize {
        // 1. a Free lane
        if let Some(v) = (0..VOICES).find(|&v| self.lane_state[v] == LaneState::Free) {
            return v;
        }
        // 2. the oldest Releasing lane (min lane_age among Releasing)
        let mut best: Option<usize> = None;
        for v in 0..VOICES {
            if self.lane_state[v] == LaneState::Releasing
                && best.map_or(true, |b| self.lane_age[v] < self.lane_age[b])
            {
                best = Some(v);
            }
        }
        if let Some(v) = best {
            return v;
        }
        // 3. all Held → steal the oldest-allocated (min lane_age overall)
        let mut best = 0;
        for v in 1..VOICES {
            if self.lane_age[v] < self.lane_age[best] {
                best = v;
            }
        }
        best
    }

    pub fn note_on(&mut self, note: u8, vel: u8, emit: &mut impl FnMut(Cmd)) {
        if vel == 0 {
            self.note_off(note, emit); // MIDI: note-on vel 0 == note-off
            return;
        }
        let u_count = self.unison.min(VOICES);
        for u in 0..u_count {
            let lane = self.pick_lane();
            self.lane_state[lane] = LaneState::Held(note);
            self.lane_age[lane] = self.clock;
            self.clock = self.clock.wrapping_add(1);
            let value = note as f32 - A440_NOTE + unison_offset(u, u_count, self.detune_cents);
            emit(Cmd::SetParam { node: self.pitch_node, param: lane as u8, value });
            if let Some(vn) = self.vel_node {
                emit(Cmd::SetParam { node: vn, param: lane as u8, value: vel as f32 / 127.0 });
            }
            if self.width_amount != 0.0 {
                emit(Cmd::SetParam {
                    node: self.sum_node,
                    param: (lane + 1) as u8, // param 0 is the gain slot on StereoVoiceSum
                    value: width_offset(u, u_count, self.width_amount),
                });
            }
            self.gate_all(lane, true, emit);
        }
    }

    pub fn note_off(&mut self, note: u8, emit: &mut impl FnMut(Cmd)) {
        // Release EVERY lane holding `note` (unison note-on may have grabbed several).
        for v in 0..VOICES {
            if self.lane_state[v] == LaneState::Held(note) {
                self.gate_all(v, false, emit);
                self.lane_state[v] = LaneState::Releasing;
                self.lane_age[v] = self.clock;
                self.clock = self.clock.wrapping_add(1);
            }
        }
        // Unheld note → no-op.
    }

    pub fn all_notes_off(&mut self, emit: &mut impl FnMut(Cmd)) {
        for v in 0..VOICES {
            if matches!(self.lane_state[v], LaneState::Held(_)) {
                self.gate_all(v, false, emit);
                self.lane_state[v] = LaneState::Releasing;
                self.lane_age[v] = self.clock;
                self.clock = self.clock.wrapping_add(1);
            }
        }
    }
}

/// Max simultaneously-held notes tracked for last-note priority.
pub const MONO_STACK: usize = 16;

/// Monophonic note allocator with last-note priority and legato glide. Drives
/// lane 0 of the poly graph: pitch (a PolyCtrl), a PolySlew (snapped from silence,
/// glides on legato), and one gate (PolyAr/PolyAdsr). Control-plane only.
pub struct MonoAllocator {
    pitch_node: NodeId,
    slew_node: NodeId,
    gates: [NodeId; MAX_GATES],
    n_gates: usize,
    vel_node: Option<NodeId>,
    sum_node: NodeId, // a StereoVoiceSum: SetParam(sum_node, lane+1, pan) when width_amount != 0.0
    notes: [u8; MONO_STACK], // press order; top (notes[len-1]) = sounding
    len: usize,
    unison: usize,
    detune_cents: f32,
    width_amount: f32,
}

impl MonoAllocator {
    pub fn new(
        pitch_node: NodeId,
        slew_node: NodeId,
        gates: [NodeId; MAX_GATES],
        n_gates: usize,
        vel_node: Option<NodeId>,
        sum_node: NodeId,
    ) -> MonoAllocator {
        MonoAllocator {
            pitch_node,
            slew_node,
            gates,
            n_gates,
            vel_node,
            sum_node,
            notes: [0; MONO_STACK],
            len: 0,
            unison: 1,
            detune_cents: 0.0,
            width_amount: 0.0,
        }
    }

    pub fn slew_node(&self) -> NodeId { self.slew_node }

    pub fn set_unison(&mut self, n: usize) { self.unison = n.clamp(1, VOICES); }
    pub fn set_detune(&mut self, cents: f32) { self.detune_cents = cents; }
    pub fn set_width(&mut self, amount: f32) { self.width_amount = amount; }

    /// Fan a gate transition out to every configured envelope (mono lane is always 0).
    fn gate_all(&self, lane: usize, on: bool, emit: &mut impl FnMut(Cmd)) {
        for g in &self.gates[..self.n_gates] {
            emit(Cmd::GateVoice { node: *g, voice: lane as u8, on });
        }
    }

    pub fn note_on(&mut self, note: u8, vel: u8, emit: &mut impl FnMut(Cmd)) {
        if vel == 0 { self.note_off(note, emit); return; }
        let from_silence = self.len == 0;
        // push (drop oldest if full)
        if self.len == MONO_STACK {
            for i in 1..MONO_STACK { self.notes[i - 1] = self.notes[i]; }
            self.len -= 1;
        }
        self.notes[self.len] = note;
        self.len += 1;
        let u_count = self.unison.min(VOICES);
        for u in 0..u_count {
            // pitch (always) → PolySlew glides toward it (or snaps, below)
            let value = note as f32 - A440_NOTE + unison_offset(u, u_count, self.detune_cents);
            emit(Cmd::SetParam { node: self.pitch_node, param: u as u8, value });
            if let Some(vn) = self.vel_node {
                emit(Cmd::SetParam { node: vn, param: u as u8, value: vel as f32 / 127.0 });
            }
            if self.width_amount != 0.0 {
                emit(Cmd::SetParam {
                    node: self.sum_node,
                    param: (u + 1) as u8, // param 0 is the gain slot on StereoVoiceSum
                    value: width_offset(u, u_count, self.width_amount),
                });
            }
            if from_silence {
                emit(Cmd::TriggerVoice { node: self.slew_node, voice: u as u8 }); // snap the glide
                self.gate_all(u, true, emit);                                    // attack
            }
            // legato (else): no snap, no re-gate — true legato
        }
    }

    pub fn note_off(&mut self, note: u8, emit: &mut impl FnMut(Cmd)) {
        // find first match
        let mut idx = None;
        for i in 0..self.len { if self.notes[i] == note { idx = Some(i); break; } }
        let Some(i) = idx else { return; }; // unheld → no-op
        let was_top = i == self.len - 1;
        // remove (shift down)
        for j in (i + 1)..self.len { self.notes[j - 1] = self.notes[j]; }
        self.len -= 1;
        let u_count = self.unison.min(VOICES);
        if self.len == 0 {
            for u in 0..u_count { self.gate_all(u, false, emit); } // release
        } else if was_top {
            let top = self.notes[self.len - 1];
            for u in 0..u_count {
                let value = top as f32 - A440_NOTE + unison_offset(u, u_count, self.detune_cents);
                emit(Cmd::SetParam { node: self.pitch_node, param: u as u8, value }); // glide back
            }
        }
        // removing a non-top held note → stack-only, no sound change
    }

    pub fn all_notes_off(&mut self, emit: &mut impl FnMut(Cmd)) {
        if self.len > 0 {
            let u_count = self.unison.min(VOICES);
            for u in 0..u_count { self.gate_all(u, false, emit); }
        }
        self.len = 0;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    fn one_gate(g: u16) -> ([NodeId; MAX_GATES], usize) {
        let mut arr = [NodeId(0); MAX_GATES];
        arr[0] = NodeId(g);
        (arr, 1)
    }

    // 1-gate VoiceAllocator: pitch=NodeId(10), gate=NodeId(20), sum=NodeId(30), no vel node.
    fn mk() -> VoiceAllocator {
        let mut g = [NodeId(0); MAX_GATES];
        g[0] = NodeId(20);
        VoiceAllocator::new(NodeId(10), g, 1, None, NodeId(30))
    }

    // 1-gate VoiceAllocator for unison tests: pitch=NodeId(10), gate=NodeId(20), no vel node.
    fn mk_poly() -> VoiceAllocator { mk() }

    // 1-gate MonoAllocator for unison tests: pitch=NodeId(10), slew=NodeId(20), gate=NodeId(30),
    // sum=NodeId(40), no vel node.
    fn mk_mono() -> MonoAllocator {
        let (g, n) = one_gate(30);
        MonoAllocator::new(NodeId(10), NodeId(20), g, n, None, NodeId(40))
    }

    // Capture the Cmds emitted by one note event.
    fn on(a: &mut VoiceAllocator, note: u8, vel: u8) -> Vec<Cmd> {
        let mut c: Vec<Cmd> = Vec::new();
        {
            let mut e = |x: Cmd| c.push(x);
            a.note_on(note, vel, &mut e);
        }
        c
    }
    fn off(a: &mut VoiceAllocator, note: u8) -> Vec<Cmd> {
        let mut c: Vec<Cmd> = Vec::new();
        {
            let mut e = |x: Cmd| c.push(x);
            a.note_off(note, &mut e);
        }
        c
    }

    // Capture the Cmds emitted by one mono note event.
    fn mon(m: &mut MonoAllocator, note: u8, vel: u8) -> Vec<Cmd> {
        let mut c: Vec<Cmd> = Vec::new();
        {
            let mut e = |x: Cmd| c.push(x);
            m.note_on(note, vel, &mut e);
        }
        c
    }
    fn moff(m: &mut MonoAllocator, note: u8) -> Vec<Cmd> {
        let mut c: Vec<Cmd> = Vec::new();
        {
            let mut e = |x: Cmd| c.push(x);
            m.note_off(note, &mut e);
        }
        c
    }

    #[test]
    fn note_on_emits_pitch_and_gate_on_lane_0() {
        let mut a = { let (g, n) = one_gate(20); VoiceAllocator::new(NodeId(10), g, n, None, NodeId(99)) };
        let c = on(&mut a, 69, 100); // A4 → semitone 0
        assert_eq!(c.len(), 2);
        match c[0] {
            Cmd::SetParam { node, param, value } => {
                assert_eq!(node.0, 10);
                assert_eq!(param, 0);
                assert!((value - 0.0).abs() < 1e-6);
            }
            _ => panic!("expected SetParam"),
        }
        match c[1] {
            Cmd::GateVoice { node, voice, on } => {
                assert_eq!(node.0, 20);
                assert_eq!(voice, 0);
                assert!(on);
            }
            _ => panic!("expected GateVoice"),
        }
    }

    #[test]
    fn eight_notes_fill_lanes_then_ninth_steals_oldest() {
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, None, NodeId(99)) };
        for k in 0..VOICES {
            let c = on(&mut a, 60 + k as u8, 100);
            match c[0] {
                Cmd::SetParam { param, .. } => assert_eq!(param as usize, k, "note {k} → lane {k}"),
                _ => panic!(),
            }
        }
        // All 8 lanes held; lane 0 is oldest. A 9th note steals lane 0.
        let c = on(&mut a, 72, 100);
        match c[0] {
            Cmd::SetParam { param, value, .. } => {
                assert_eq!(param, 0, "steals oldest lane 0");
                assert!((value - (72.0 - 69.0)).abs() < 1e-6);
            }
            _ => panic!(),
        }
        assert!(matches!(c[1], Cmd::GateVoice { voice: 0, on: true, .. }));
    }

    #[test]
    fn note_off_releases_the_right_lane_and_frees_it() {
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, None, NodeId(99)) };
        on(&mut a, 60, 100); // lane 0
        on(&mut a, 64, 100); // lane 1
        let c = off(&mut a, 60);
        assert_eq!(c.len(), 1);
        assert!(matches!(c[0], Cmd::GateVoice { voice: 0, on: false, .. }));
        // Lane 0 is Releasing (tail protected), not free → next note-on takes
        // a FREE lane (2; lanes 0 and 1 were used) instead of reusing lane 0.
        let c2 = on(&mut a, 67, 100);
        assert!(matches!(c2[0], Cmd::SetParam { param: 2, .. }));
    }

    #[test]
    fn released_lane_is_protected_new_note_takes_a_free_lane() {
        let mut a = mk();
        on(&mut a, 60, 100); // A → lane 0
        off(&mut a, 60); // lane 0 → Releasing (tail ringing)
        let c = on(&mut a, 64, 100); // B → should take a FREE lane (1), NOT reuse lane 0
        match c[0] {
            Cmd::SetParam { param, .. } => assert_eq!(param, 1, "new note protects lane 0's tail → lane 1"),
            _ => panic!("expected pitch SetParam"),
        }
    }

    #[test]
    fn reuse_oldest_releasing_before_stealing_held() {
        let mut a = mk();
        for k in 0..VOICES {
            on(&mut a, 60 + k as u8, 100);
        } // lanes 0..7 all Held
        off(&mut a, 62); // lane 2 → Releasing (released first)
        off(&mut a, 65); // lane 5 → Releasing (released second)
        // new note reuses the OLDEST releasing lane (2), not a held lane, not lane 5
        let c1 = on(&mut a, 80, 100);
        match c1[0] {
            Cmd::SetParam { param, .. } => assert_eq!(param, 2, "oldest releasing"),
            _ => panic!(),
        }
        // next new note reuses the remaining releasing lane (5)
        let c2 = on(&mut a, 81, 100);
        match c2[0] {
            Cmd::SetParam { param, .. } => assert_eq!(param, 5, "next releasing"),
            _ => panic!(),
        }
    }

    #[test]
    fn all_held_steals_oldest_held_unchanged() {
        let mut a = mk();
        for k in 0..VOICES {
            on(&mut a, 60 + k as u8, 100);
        } // lanes 0..7 Held, lane 0 oldest
        let c = on(&mut a, 72, 100); // no free/releasing → steal oldest held (lane 0)
        match c[0] {
            Cmd::SetParam { param, .. } => assert_eq!(param, 0, "steal oldest held"),
            _ => panic!(),
        }
        assert!(c.iter().any(|cmd| matches!(cmd, Cmd::GateVoice { voice: 0, on: true, .. })));
    }

    #[test]
    fn reusing_a_releasing_lane_re_gates_it() {
        let mut a = mk();
        on(&mut a, 60, 100); // lane 0
        off(&mut a, 60); // lane 0 Releasing
        // fill lanes 1..7 so the next note has no Free lane → must reuse releasing lane 0
        for k in 1..VOICES {
            on(&mut a, 61 + k as u8, 100);
        }
        let c = on(&mut a, 90, 100); // only lane 0 is Releasing (rest Held) → reuse lane 0, re-attack
        assert!(matches!(c[0], Cmd::SetParam { param: 0, .. }));
        assert!(
            c.iter().any(|cmd| matches!(cmd, Cmd::GateVoice { voice: 0, on: true, .. })),
            "re-gate on reuse"
        );
    }

    #[test]
    fn all_notes_off_marks_releasing_and_gates_off() {
        let mut a = mk();
        on(&mut a, 60, 100);
        on(&mut a, 64, 100);
        let mut c: Vec<Cmd> = Vec::new();
        {
            let mut e = |x: Cmd| c.push(x);
            a.all_notes_off(&mut e);
        }
        assert_eq!(c.len(), 2); // gate-off for the two held lanes
        assert!(c.iter().all(|cmd| matches!(cmd, Cmd::GateVoice { on: false, .. })));
        // after all-notes-off, the two lanes are Releasing (occupied), so a new note
        // still prefers the remaining Free lanes:
        let c2 = on(&mut a, 67, 100);
        match c2[0] {
            Cmd::SetParam { param, .. } => assert!(param >= 2, "new note avoids the two releasing lanes"),
            _ => panic!(),
        }
    }

    #[test]
    fn note_on_velocity_zero_is_note_off() {
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, None, NodeId(99)) };
        on(&mut a, 60, 100); // lane 0 held
        let c = on(&mut a, 60, 0); // vel 0 → note-off
        assert_eq!(c.len(), 1);
        assert!(matches!(c[0], Cmd::GateVoice { voice: 0, on: false, .. }));
    }

    #[test]
    fn note_off_for_unheld_note_is_noop() {
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, None, NodeId(99)) };
        let c = off(&mut a, 60);
        assert!(c.is_empty());
    }

    #[test]
    fn all_notes_off_releases_every_held_lane() {
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, None, NodeId(99)) };
        on(&mut a, 60, 100);
        on(&mut a, 64, 100);
        let mut c: Vec<Cmd> = Vec::new();
        {
            let mut e = |x: Cmd| c.push(x);
            a.all_notes_off(&mut e);
        }
        assert_eq!(c.len(), 2);
        assert!(c.iter().all(|cmd| matches!(cmd, Cmd::GateVoice { on: false, .. })));
    }

    #[test]
    fn note_on_emits_velocity_when_vel_node_present() {
        let mut a = { let (g, n) = one_gate(20); VoiceAllocator::new(NodeId(10), g, n, Some(NodeId(30)), NodeId(99)) };
        let c = on(&mut a, 69, 100); // A4, vel 100
        assert_eq!(c.len(), 3, "pitch + velocity + gate");
        // c[0] = pitch SetParam(node 10), c[1] = velocity SetParam(node 30), c[2] = GateVoice(node 20)
        match c[1] {
            Cmd::SetParam { node, param, value } => {
                assert_eq!(node.0, 30, "velocity node");
                assert_eq!(param, 0, "lane 0");
                assert!((value - 100.0 / 127.0).abs() < 1e-6, "vel/127, got {value}");
            }
            _ => panic!("expected velocity SetParam at index 1"),
        }
        assert!(matches!(c[2], Cmd::GateVoice { node: NodeId(20), voice: 0, on: true }));
    }

    #[test]
    fn note_on_no_velocity_node_emits_two_cmds() {
        let mut a = { let (g, n) = one_gate(20); VoiceAllocator::new(NodeId(10), g, n, None, NodeId(99)) };
        let c = on(&mut a, 69, 100);
        assert_eq!(c.len(), 2, "pitch + gate only");
        assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), .. }));
        assert!(matches!(c[1], Cmd::GateVoice { node: NodeId(20), .. }));
    }

    #[test]
    fn velocity_value_is_proportional() {
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, Some(NodeId(2)), NodeId(99)) };
        let hi = on(&mut a, 60, 127);
        let lo = on(&mut a, 62, 20);
        let vhi = match hi[1] { Cmd::SetParam { value, .. } => value, _ => panic!() };
        let vlo = match lo[1] { Cmd::SetParam { value, .. } => value, _ => panic!() };
        assert!(vhi > vlo, "higher velocity → larger value: {vhi} vs {vlo}");
        assert!((vhi - 1.0).abs() < 1e-6, "127 → 1.0");
    }

    #[test]
    fn mono_first_note_snaps_and_gates() {
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, None, NodeId(99)) };
        let c = mon(&mut m, 69, 100); // A4 from silence
        // SetParam(pitch=10, lane0, 0.0) + TriggerVoice(slew=20, 0) + GateVoice(gate=30, 0, on)
        assert_eq!(c.len(), 3);
        assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), param: 0, .. }));
        assert!(matches!(c[1], Cmd::TriggerVoice { node: NodeId(20), voice: 0 }));
        assert!(matches!(c[2], Cmd::GateVoice { node: NodeId(30), voice: 0, on: true }));
    }

    #[test]
    fn mono_legato_note_glides_without_regate() {
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, None, NodeId(99)) };
        mon(&mut m, 60, 100);       // first note (from silence)
        let c = mon(&mut m, 64, 100); // legato (60 still held)
        // Only a pitch SetParam (glide) — NO TriggerVoice, NO GateVoice
        assert_eq!(c.len(), 1);
        assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), param: 0, .. }));
    }

    #[test]
    fn mono_note_off_falls_back_to_held_note() {
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, None, NodeId(99)) };
        mon(&mut m, 60, 100);
        mon(&mut m, 64, 100); // 64 sounding, 60 held
        let c = moff(&mut m, 64); // release 64 → glide back to 60
        assert_eq!(c.len(), 1);
        match c[0] {
            Cmd::SetParam { node: NodeId(10), param: 0, value } => assert!((value - (60.0 - 69.0)).abs() < 1e-6),
            _ => panic!("expected glide-back SetParam to 60"),
        }
    }

    #[test]
    fn mono_last_note_off_releases() {
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, None, NodeId(99)) };
        mon(&mut m, 60, 100);
        let c = moff(&mut m, 60); // stack empty → release
        assert_eq!(c.len(), 1);
        assert!(matches!(c[0], Cmd::GateVoice { node: NodeId(30), voice: 0, on: false }));
    }

    #[test]
    fn mono_velocity_written_when_present() {
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, Some(NodeId(40)), NodeId(99)) };
        let c = mon(&mut m, 69, 100);
        // pitch SetParam, velocity SetParam(node 40, 100/127), TriggerVoice, GateVoice
        assert!(c.iter().any(|cmd| matches!(cmd,
            Cmd::SetParam { node: NodeId(40), param: 0, value } if (value - 100.0/127.0).abs() < 1e-6)));
    }

    #[test]
    fn mono_note_off_unheld_is_noop() {
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, None, NodeId(99)) };
        let c = moff(&mut m, 60);
        assert!(c.is_empty());
    }

    #[test]
    fn poly_note_on_gates_all_envelopes_in_order() {
        let mut gates = [NodeId(0); MAX_GATES];
        gates[0] = NodeId(20); gates[1] = NodeId(21);
        let mut a = VoiceAllocator::new(NodeId(10), gates, 2, None, NodeId(99));
        let c = on(&mut a, 69, 100); // pitch SetParam + GateVoice(20) + GateVoice(21)
        assert_eq!(c.len(), 3);
        assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), .. }));
        assert!(matches!(c[1], Cmd::GateVoice { node: NodeId(20), voice: 0, on: true }));
        assert!(matches!(c[2], Cmd::GateVoice { node: NodeId(21), voice: 0, on: true }));
    }

    #[test]
    fn poly_note_off_releases_all_envelopes() {
        let mut gates = [NodeId(0); MAX_GATES];
        gates[0] = NodeId(20); gates[1] = NodeId(21);
        let mut a = VoiceAllocator::new(NodeId(10), gates, 2, None, NodeId(99));
        on(&mut a, 69, 100);
        let c = off(&mut a, 69);
        assert_eq!(c.len(), 2);
        assert!(matches!(c[0], Cmd::GateVoice { node: NodeId(20), on: false, .. }));
        assert!(matches!(c[1], Cmd::GateVoice { node: NodeId(21), on: false, .. }));
    }

    #[test]
    fn mono_from_silence_gates_all_envelopes_legato_gates_none() {
        let mut gates = [NodeId(0); MAX_GATES];
        gates[0] = NodeId(30); gates[1] = NodeId(31);
        let mut m = MonoAllocator::new(NodeId(10), NodeId(20), gates, 2, None, NodeId(99));
        let c = mon(&mut m, 60, 100); // pitch SetParam + TriggerVoice(slew 20) + GateVoice(30) + GateVoice(31)
        assert_eq!(c.len(), 4);
        assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), .. }));
        assert!(matches!(c[1], Cmd::TriggerVoice { node: NodeId(20), voice: 0 }));
        assert!(matches!(c[2], Cmd::GateVoice { node: NodeId(30), voice: 0, on: true }));
        assert!(matches!(c[3], Cmd::GateVoice { node: NodeId(31), voice: 0, on: true }));
        // legato note: pitch SetParam ONLY, no gate to any envelope
        let c2 = mon(&mut m, 64, 100);
        assert_eq!(c2.len(), 1);
        assert!(matches!(c2[0], Cmd::SetParam { node: NodeId(10), .. }));
    }

    #[test]
    fn unison_offset_spreads_symmetric_and_even() {
        assert_eq!(unison_offset(0, 1, 10.0), 0.0); // single voice = no detune
        // U=2 @ 10 cents → ±0.1 semitone
        assert!((unison_offset(0, 2, 10.0) - (-0.1)).abs() < 1e-6);
        assert!((unison_offset(1, 2, 10.0) - 0.1).abs() < 1e-6);
        // U=3 @ 10 cents → -0.1, 0, +0.1
        assert!((unison_offset(0, 3, 10.0) - (-0.1)).abs() < 1e-6);
        assert!(unison_offset(1, 3, 10.0).abs() < 1e-6);
        assert!((unison_offset(2, 3, 10.0) - 0.1).abs() < 1e-6);
    }

    #[test]
    fn poly_unison_allocates_u_distinct_detuned_lanes() {
        let mut a = mk_poly(); // helper: VoiceAllocator, pitch=NodeId(10), 1 gate=NodeId(20), no vel
        a.set_unison(3);
        a.set_detune(10.0);
        let c = on(&mut a, 69, 100); // A4, 3 unison voices
        // 3 lanes × (SetParam pitch + GateVoice) = 6 Cmds; the 3 SetParam values are the 3 offsets around 0
        let pitches: Vec<(u8, f32)> = c.iter().filter_map(|cmd| match cmd {
            Cmd::SetParam { node: NodeId(10), param, value } => Some((*param, *value)),
            _ => None,
        }).collect();
        assert_eq!(pitches.len(), 3, "3 distinct pitch SetParams");
        // lanes distinct
        let mut lanes: Vec<u8> = pitches.iter().map(|(p, _)| *p).collect();
        lanes.sort(); lanes.dedup();
        assert_eq!(lanes.len(), 3, "3 distinct lanes");
        // values are 0 (=A4 semitone) ± 0.1
        let mut vals: Vec<f32> = pitches.iter().map(|(_, v)| *v).collect();
        vals.sort_by(|x, y| x.partial_cmp(y).unwrap());
        assert!((vals[0] - (-0.1)).abs() < 1e-6 && vals[1].abs() < 1e-6 && (vals[2] - 0.1).abs() < 1e-6);
        // and a GateVoice(on) for each of the 3 lanes
        assert_eq!(c.iter().filter(|cmd| matches!(cmd, Cmd::GateVoice { on: true, .. })).count(), 3);
    }

    #[test]
    fn poly_unison_note_off_releases_all_voices() {
        let mut a = mk_poly();
        a.set_unison(3);
        on(&mut a, 69, 100);
        let c = off(&mut a, 69);
        assert_eq!(c.iter().filter(|cmd| matches!(cmd, Cmd::GateVoice { on: false, .. })).count(), 3, "all 3 released");
    }

    #[test]
    fn poly_unison_second_note_grabs_more_lanes() {
        let pitch_lanes = |c: &[Cmd]| -> Vec<u8> {
            c.iter().filter_map(|cmd| match cmd {
                Cmd::SetParam { node: NodeId(10), param, .. } => Some(*param), _ => None }).collect()
        };
        let mut a = mk_poly();
        a.set_unison(3);
        let first = pitch_lanes(&on(&mut a, 60, 100)); // 3 lanes
        let c = on(&mut a, 64, 100); // 3 MORE lanes (distinct from the first 3)
        let lanes = pitch_lanes(&c);
        let mut u = lanes.clone(); u.sort(); u.dedup();
        assert_eq!(u.len(), 3, "second note grabs 3 distinct lanes");
        // and none of the second note's lanes collide with the first note's (both notes held)
        assert!(lanes.iter().all(|l| !first.contains(l)), "second note's lanes disjoint from first note's");
    }

    #[test]
    fn poly_unison_one_is_byte_identical() {
        let mut a = mk_poly();
        a.set_unison(1);
        let c = on(&mut a, 69, 100); // exactly 2 Cmds: pitch SetParam(param 0, value 0.0) + GateVoice(0, on)
        assert_eq!(c.len(), 2);
        assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), param: 0, value } if value.abs() < 1e-6));
        assert!(matches!(c[1], Cmd::GateVoice { voice: 0, on: true, .. }));
    }

    #[test]
    fn mono_unison_from_silence_drives_u_lanes() {
        let mut m = mk_mono(); // pitch=NodeId(10), slew=NodeId(20), 1 gate=NodeId(30), no vel
        m.set_unison(2);
        m.set_detune(10.0);
        let c = mon(&mut m, 69, 100); // from silence, 2 unison voices
        // lanes 0 and 1: each SetParam(pitch) + TriggerVoice(slew) + GateVoice(on)
        let pitch_lanes: Vec<u8> = c.iter().filter_map(|cmd| match cmd {
            Cmd::SetParam { node: NodeId(10), param, .. } => Some(*param), _ => None }).collect();
        assert_eq!(pitch_lanes, std::vec![0, 1], "lanes 0 and 1");
        assert_eq!(c.iter().filter(|cmd| matches!(cmd, Cmd::TriggerVoice { node: NodeId(20), .. })).count(), 2, "snap both");
        assert_eq!(c.iter().filter(|cmd| matches!(cmd, Cmd::GateVoice { on: true, .. })).count(), 2, "gate both");
    }

    #[test]
    fn mono_unison_legato_glides_all_lanes_no_regate() {
        let mut m = mk_mono();
        m.set_unison(2);
        mon(&mut m, 60, 100);         // from silence
        let c = mon(&mut m, 64, 100); // legato
        // both lanes get a pitch SetParam (glide), NO gate/trigger
        assert_eq!(c.iter().filter(|cmd| matches!(cmd, Cmd::SetParam { node: NodeId(10), .. })).count(), 2);
        assert!(!c.iter().any(|cmd| matches!(cmd, Cmd::GateVoice { .. } | Cmd::TriggerVoice { .. })), "true legato: no re-gate/snap");
    }

    #[test]
    fn mono_unison_note_off_gates_all_lanes() {
        let mut m = mk_mono();
        m.set_unison(2);
        mon(&mut m, 60, 100);
        let c = moff(&mut m, 60); // to silence
        assert_eq!(c.iter().filter(|cmd| matches!(cmd, Cmd::GateVoice { on: false, .. })).count(), 2);
    }

    #[test]
    fn mono_unison_one_is_byte_identical() {
        let mut m = mk_mono();
        m.set_unison(1);
        let c = mon(&mut m, 69, 100); // lane 0 only: SetParam(0) + TriggerVoice(0) + GateVoice(0)
        assert_eq!(c.len(), 3);
        assert!(matches!(c[0], Cmd::SetParam { param: 0, .. }));
        assert!(matches!(c[1], Cmd::TriggerVoice { voice: 0, .. }));
        assert!(matches!(c[2], Cmd::GateVoice { voice: 0, on: true, .. }));
    }

    #[test]
    fn width_offset_spreads_symmetric_and_scaled() {
        assert_eq!(width_offset(0, 1, 1.0), 0.0); // single voice = center
        // U=2 @ amount=1 → -1, +1
        assert!((width_offset(0, 2, 1.0) - (-1.0)).abs() < 1e-6);
        assert!((width_offset(1, 2, 1.0) - 1.0).abs() < 1e-6);
        // U=3 @ amount=0.5 → -0.5, 0, +0.5
        assert!((width_offset(0, 3, 0.5) - (-0.5)).abs() < 1e-6);
        assert!(width_offset(1, 3, 0.5).abs() < 1e-6);
        assert!((width_offset(2, 3, 0.5) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn poly_width_emits_per_lane_pan_on_sum_node() {
        let mut a = mk_poly(); // pitch=NodeId(10), gate=NodeId(20), sum=NodeId(30), no vel
        a.set_unison(3);
        a.set_width(1.0);
        let c = on(&mut a, 69, 100);
        // 3 pan SetParams on the sum node (NodeId(30)), params lane+1, 3 symmetric values.
        let pans: std::vec::Vec<(u8, f32)> = c.iter().filter_map(|cmd| match cmd {
            Cmd::SetParam { node: NodeId(30), param, value } => Some((*param, *value)),
            _ => None,
        }).collect();
        assert_eq!(pans.len(), 3, "3 pan SetParams on sum node");
        // params are lane+1 (>=1), distinct
        assert!(pans.iter().all(|(p, _)| *p >= 1));
        let mut params: std::vec::Vec<u8> = pans.iter().map(|(p, _)| *p).collect();
        params.sort(); params.dedup();
        assert_eq!(params.len(), 3, "3 distinct lane params");
        // values symmetric around 0: sorted ≈ -1, 0, +1
        let mut vals: std::vec::Vec<f32> = pans.iter().map(|(_, v)| *v).collect();
        vals.sort_by(|x, y| x.partial_cmp(y).unwrap());
        assert!((vals[0] - (-1.0)).abs() < 1e-6 && vals[1].abs() < 1e-6 && (vals[2] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn poly_width_zero_emits_no_pan() {
        let mut a = mk_poly();
        a.set_unison(3);
        // width defaults to 0.0 — no set_width call.
        let c = on(&mut a, 69, 100);
        assert_eq!(
            c.iter().filter(|cmd| matches!(cmd, Cmd::SetParam { node: NodeId(30), .. })).count(),
            0,
            "width=0 emits no pan SetParam (byte-identical Cmd stream)"
        );
    }

    #[test]
    fn mono_width_emits_per_lane_pan_on_sum_node() {
        let mut m = mk_mono(); // pitch=NodeId(10), slew=NodeId(20), gate=NodeId(30), sum=NodeId(40), no vel
        m.set_unison(2);
        m.set_width(1.0);
        let c = mon(&mut m, 69, 100); // from silence, 2 unison voices
        // pan on lanes 0 and 1 (params 1 and 2) on the sum node (NodeId(40)).
        let pans: std::vec::Vec<u8> = c.iter().filter_map(|cmd| match cmd {
            Cmd::SetParam { node: NodeId(40), param, .. } => Some(*param), _ => None }).collect();
        assert_eq!(pans, std::vec![1, 2], "pan params lane+1 for lanes 0 and 1");
    }

    #[test]
    fn mono_width_zero_emits_no_pan() {
        let mut m = mk_mono();
        m.set_unison(2);
        let c = mon(&mut m, 69, 100);
        assert_eq!(
            c.iter().filter(|cmd| matches!(cmd, Cmd::SetParam { node: NodeId(40), .. })).count(),
            0, "width=0 emits no pan"
        );
    }
}
