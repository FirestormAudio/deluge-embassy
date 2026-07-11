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

pub struct VoiceAllocator {
    pitch_node: NodeId, // a PolyCtrl: SetParam(pitch_node, lane, note - 69)
    gate_node: NodeId,  // a PolyAr:   GateVoice(gate_node, lane, on/off)
    vel_node: Option<NodeId>, // a PolyCtrl carrying per-voice velocity, or None
    lane_note: [Option<u8>; VOICES],
    lane_age: [u32; VOICES],
    clock: u32,
}

impl VoiceAllocator {
    pub fn new(pitch_node: NodeId, gate_node: NodeId, vel_node: Option<NodeId>) -> VoiceAllocator {
        VoiceAllocator {
            pitch_node,
            gate_node,
            vel_node,
            lane_note: [None; VOICES],
            lane_age: [0; VOICES],
            clock: 0,
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
        emit(Cmd::GateVoice { node: self.gate_node, voice: lane as u8, on: true });
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
            emit(Cmd::GateVoice { node: self.gate_node, voice: lane as u8, on: false });
            self.lane_note[lane] = None;
        }
        // Unheld note → no-op.
    }

    pub fn all_notes_off(&mut self, emit: &mut impl FnMut(Cmd)) {
        for v in 0..VOICES {
            if self.lane_note[v].is_some() {
                emit(Cmd::GateVoice { node: self.gate_node, voice: v as u8, on: false });
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

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

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

    #[test]
    fn note_on_emits_pitch_and_gate_on_lane_0() {
        let mut a = VoiceAllocator::new(NodeId(10), NodeId(20), None);
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
        let mut a = VoiceAllocator::new(NodeId(0), NodeId(1), None);
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
        let mut a = VoiceAllocator::new(NodeId(0), NodeId(1), None);
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
        let mut a = VoiceAllocator::new(NodeId(0), NodeId(1), None);
        on(&mut a, 60, 100); // lane 0 held
        let c = on(&mut a, 60, 0); // vel 0 → note-off
        assert_eq!(c.len(), 1);
        assert!(matches!(c[0], Cmd::GateVoice { voice: 0, on: false, .. }));
    }

    #[test]
    fn note_off_for_unheld_note_is_noop() {
        let mut a = VoiceAllocator::new(NodeId(0), NodeId(1), None);
        let c = off(&mut a, 60);
        assert!(c.is_empty());
    }

    #[test]
    fn all_notes_off_releases_every_held_lane() {
        let mut a = VoiceAllocator::new(NodeId(0), NodeId(1), None);
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
        let mut a = VoiceAllocator::new(NodeId(10), NodeId(20), Some(NodeId(30)));
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
        let mut a = VoiceAllocator::new(NodeId(10), NodeId(20), None);
        let c = on(&mut a, 69, 100);
        assert_eq!(c.len(), 2, "pitch + gate only");
        assert!(matches!(c[0], Cmd::SetParam { node: NodeId(10), .. }));
        assert!(matches!(c[1], Cmd::GateVoice { node: NodeId(20), .. }));
    }

    #[test]
    fn velocity_value_is_proportional() {
        let mut a = VoiceAllocator::new(NodeId(0), NodeId(1), Some(NodeId(2)));
        let hi = on(&mut a, 60, 127);
        let lo = on(&mut a, 62, 20);
        let vhi = match hi[1] { Cmd::SetParam { value, .. } => value, _ => panic!() };
        let vlo = match lo[1] { Cmd::SetParam { value, .. } => value, _ => panic!() };
        assert!(vhi > vlo, "higher velocity → larger value: {vhi} vs {vlo}");
        assert!((vhi - 1.0).abs() < 1e-6, "127 → 1.0");
    }
}
