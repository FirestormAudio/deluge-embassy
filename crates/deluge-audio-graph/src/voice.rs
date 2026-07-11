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

pub struct VoiceAllocator {
    pitch_node: NodeId,           // a PolyCtrl: SetParam(pitch_node, lane, note - 69)
    gates: [NodeId; MAX_GATES],   // PolyAr/PolyAdsr nodes: GateVoice(gates[i], lane, on/off)
    n_gates: usize,                // number of valid entries in `gates`
    vel_node: Option<NodeId>, // a PolyCtrl carrying per-voice velocity, or None
    lane_note: [Option<u8>; VOICES],
    lane_age: [u32; VOICES],
    clock: u32,
}

impl VoiceAllocator {
    pub fn new(
        pitch_node: NodeId,
        gates: [NodeId; MAX_GATES],
        n_gates: usize,
        vel_node: Option<NodeId>,
    ) -> VoiceAllocator {
        VoiceAllocator {
            pitch_node,
            gates,
            n_gates,
            vel_node,
            lane_note: [None; VOICES],
            lane_age: [0; VOICES],
            clock: 0,
        }
    }

    /// Fan a gate transition out to every configured envelope, preserving order.
    fn gate_all(&self, lane: usize, on: bool, emit: &mut impl FnMut(Cmd)) {
        for g in &self.gates[..self.n_gates] {
            emit(Cmd::GateVoice { node: *g, voice: lane as u8, on });
        }
    }

    pub fn note_on(&mut self, note: u8, vel: u8, emit: &mut impl FnMut(Cmd)) {
        if vel == 0 {
            self.note_off(note, emit); // MIDI: note-on vel 0 == note-off
            return;
        }
        // A free lane, else steal the oldest (lowest age). Reassigning +
        // GateVoice(on) re-attacks the stolen lane — no separate gate-off needed.
        let lane = self.free_lane().unwrap_or_else(|| self.oldest_lane());
        self.lane_note[lane] = Some(note);
        self.lane_age[lane] = self.clock;
        self.clock = self.clock.wrapping_add(1);
        emit(Cmd::SetParam {
            node: self.pitch_node,
            param: lane as u8,
            value: note as f32 - A440_NOTE,
        });
        if let Some(vn) = self.vel_node {
            emit(Cmd::SetParam { node: vn, param: lane as u8, value: vel as f32 / 127.0 });
        }
        self.gate_all(lane, true, emit);
    }

    pub fn note_off(&mut self, note: u8, emit: &mut impl FnMut(Cmd)) {
        // Release the most-recently-allocated lane playing `note`.
        let mut best: Option<usize> = None;
        for v in 0..VOICES {
            if self.lane_note[v] == Some(note)
                && best.map_or(true, |b| self.lane_age[v] > self.lane_age[b])
            {
                best = Some(v);
            }
        }
        if let Some(lane) = best {
            self.gate_all(lane, false, emit);
            self.lane_note[lane] = None;
        }
        // Unheld note → no-op.
    }

    pub fn all_notes_off(&mut self, emit: &mut impl FnMut(Cmd)) {
        for v in 0..VOICES {
            if self.lane_note[v].is_some() {
                self.gate_all(v, false, emit);
                self.lane_note[v] = None;
            }
        }
    }

    fn free_lane(&self) -> Option<usize> {
        (0..VOICES).find(|&v| self.lane_note[v].is_none())
    }

    fn oldest_lane(&self) -> usize {
        let mut best = 0;
        for v in 1..VOICES {
            if self.lane_age[v] < self.lane_age[best] {
                best = v;
            }
        }
        best
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
    notes: [u8; MONO_STACK], // press order; top (notes[len-1]) = sounding
    len: usize,
}

impl MonoAllocator {
    pub fn new(
        pitch_node: NodeId,
        slew_node: NodeId,
        gates: [NodeId; MAX_GATES],
        n_gates: usize,
        vel_node: Option<NodeId>,
    ) -> MonoAllocator {
        MonoAllocator { pitch_node, slew_node, gates, n_gates, vel_node, notes: [0; MONO_STACK], len: 0 }
    }

    pub fn slew_node(&self) -> NodeId { self.slew_node }

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
        // pitch (always) → PolySlew glides toward it (or snaps, below)
        emit(Cmd::SetParam { node: self.pitch_node, param: 0, value: note as f32 - A440_NOTE });
        if let Some(vn) = self.vel_node {
            emit(Cmd::SetParam { node: vn, param: 0, value: vel as f32 / 127.0 });
        }
        if from_silence {
            emit(Cmd::TriggerVoice { node: self.slew_node, voice: 0 }); // snap the glide
            self.gate_all(0, true, emit);                               // attack
        }
        // legato (else): no snap, no re-gate — true legato
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
        if self.len == 0 {
            self.gate_all(0, false, emit); // release
        } else if was_top {
            let top = self.notes[self.len - 1];
            emit(Cmd::SetParam { node: self.pitch_node, param: 0, value: top as f32 - A440_NOTE }); // glide back
        }
        // removing a non-top held note → stack-only, no sound change
    }

    pub fn all_notes_off(&mut self, emit: &mut impl FnMut(Cmd)) {
        if self.len > 0 {
            self.gate_all(0, false, emit);
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
        let mut a = { let (g, n) = one_gate(20); VoiceAllocator::new(NodeId(10), g, n, None) };
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
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, None) };
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
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, None) };
        on(&mut a, 60, 100); // lane 0
        on(&mut a, 64, 100); // lane 1
        let c = off(&mut a, 60);
        assert_eq!(c.len(), 1);
        assert!(matches!(c[0], Cmd::GateVoice { voice: 0, on: false, .. }));
        // Lane 0 is free again → next note-on reuses it.
        let c2 = on(&mut a, 67, 100);
        assert!(matches!(c2[0], Cmd::SetParam { param: 0, .. }));
    }

    #[test]
    fn note_on_velocity_zero_is_note_off() {
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, None) };
        on(&mut a, 60, 100); // lane 0 held
        let c = on(&mut a, 60, 0); // vel 0 → note-off
        assert_eq!(c.len(), 1);
        assert!(matches!(c[0], Cmd::GateVoice { voice: 0, on: false, .. }));
    }

    #[test]
    fn note_off_for_unheld_note_is_noop() {
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, None) };
        let c = off(&mut a, 60);
        assert!(c.is_empty());
    }

    #[test]
    fn all_notes_off_releases_every_held_lane() {
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, None) };
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
        let mut a = { let (g, n) = one_gate(20); VoiceAllocator::new(NodeId(10), g, n, Some(NodeId(30))) };
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
        let mut a = { let (g, n) = one_gate(20); VoiceAllocator::new(NodeId(10), g, n, None) };
        let c = on(&mut a, 69, 100);
        assert_eq!(c.len(), 2, "pitch + gate only");
        assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), .. }));
        assert!(matches!(c[1], Cmd::GateVoice { node: NodeId(20), .. }));
    }

    #[test]
    fn velocity_value_is_proportional() {
        let mut a = { let (g, n) = one_gate(1); VoiceAllocator::new(NodeId(0), g, n, Some(NodeId(2))) };
        let hi = on(&mut a, 60, 127);
        let lo = on(&mut a, 62, 20);
        let vhi = match hi[1] { Cmd::SetParam { value, .. } => value, _ => panic!() };
        let vlo = match lo[1] { Cmd::SetParam { value, .. } => value, _ => panic!() };
        assert!(vhi > vlo, "higher velocity → larger value: {vhi} vs {vlo}");
        assert!((vhi - 1.0).abs() < 1e-6, "127 → 1.0");
    }

    #[test]
    fn mono_first_note_snaps_and_gates() {
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, None) };
        let c = mon(&mut m, 69, 100); // A4 from silence
        // SetParam(pitch=10, lane0, 0.0) + TriggerVoice(slew=20, 0) + GateVoice(gate=30, 0, on)
        assert_eq!(c.len(), 3);
        assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), param: 0, .. }));
        assert!(matches!(c[1], Cmd::TriggerVoice { node: NodeId(20), voice: 0 }));
        assert!(matches!(c[2], Cmd::GateVoice { node: NodeId(30), voice: 0, on: true }));
    }

    #[test]
    fn mono_legato_note_glides_without_regate() {
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, None) };
        mon(&mut m, 60, 100);       // first note (from silence)
        let c = mon(&mut m, 64, 100); // legato (60 still held)
        // Only a pitch SetParam (glide) — NO TriggerVoice, NO GateVoice
        assert_eq!(c.len(), 1);
        assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), param: 0, .. }));
    }

    #[test]
    fn mono_note_off_falls_back_to_held_note() {
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, None) };
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
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, None) };
        mon(&mut m, 60, 100);
        let c = moff(&mut m, 60); // stack empty → release
        assert_eq!(c.len(), 1);
        assert!(matches!(c[0], Cmd::GateVoice { node: NodeId(30), voice: 0, on: false }));
    }

    #[test]
    fn mono_velocity_written_when_present() {
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, Some(NodeId(40))) };
        let c = mon(&mut m, 69, 100);
        // pitch SetParam, velocity SetParam(node 40, 100/127), TriggerVoice, GateVoice
        assert!(c.iter().any(|cmd| matches!(cmd,
            Cmd::SetParam { node: NodeId(40), param: 0, value } if (value - 100.0/127.0).abs() < 1e-6)));
    }

    #[test]
    fn mono_note_off_unheld_is_noop() {
        let mut m = { let (g, n) = one_gate(30); MonoAllocator::new(NodeId(10), NodeId(20), g, n, None) };
        let c = moff(&mut m, 60);
        assert!(c.is_empty());
    }

    #[test]
    fn poly_note_on_gates_all_envelopes_in_order() {
        let mut gates = [NodeId(0); MAX_GATES];
        gates[0] = NodeId(20); gates[1] = NodeId(21);
        let mut a = VoiceAllocator::new(NodeId(10), gates, 2, None);
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
        let mut a = VoiceAllocator::new(NodeId(10), gates, 2, None);
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
        let mut m = MonoAllocator::new(NodeId(10), NodeId(20), gates, 2, None);
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
}
