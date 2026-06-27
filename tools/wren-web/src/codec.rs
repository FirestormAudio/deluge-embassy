//! Fixed-width wire encoding for audio-graph [`Cmd`]s, so the main VM thread can
//! ship control-rate graph mutations to the AudioWorklet's render engine (a
//! second wasm instance). Both ends are this same crate compiled once, so the
//! 16-byte record layout is shared by construction.

use deluge_wren_core::{Cmd, Input};

/// Bytes per command record.
pub const REC: usize = 16;

// Tags.
const NOP: u8 = 0;
const NEW_NODE: u8 = 1;
const SET_INPUT: u8 = 2;
const GATE: u8 = 3;
const TRIGGER: u8 = 4;
const SET_ROOT: u8 = 5;
const RESET: u8 = 6;

fn enc_input(i: Input) -> (u8, u32) {
    match i {
        Input::Const(c) => (0, c.to_bits()),
        Input::Node(n) => (1, n as u32),
    }
}
fn dec_input(tag: u8, val: u32) -> Input {
    if tag == 1 { Input::Node(val as u16) } else { Input::Const(f32::from_bits(val)) }
}

fn put_id(r: &mut [u8; REC], id: u16) {
    r[2..4].copy_from_slice(&id.to_le_bytes());
}
fn put_a(r: &mut [u8; REC], i: Input) {
    let (t, v) = enc_input(i);
    r[4] = t;
    r[6..10].copy_from_slice(&v.to_le_bytes());
}
fn put_b(r: &mut [u8; REC], i: Input) {
    let (t, v) = enc_input(i);
    r[10] = t;
    r[12..16].copy_from_slice(&v.to_le_bytes());
}

/// Encode one command into a 16-byte record.
pub fn encode(c: Cmd) -> [u8; REC] {
    let mut r = [0u8; REC];
    match c {
        Cmd::Nop => r[0] = NOP,
        Cmd::NewNode { id, kind, a, b } => {
            r[0] = NEW_NODE;
            r[1] = kind;
            put_id(&mut r, id);
            put_a(&mut r, a);
            put_b(&mut r, b);
        }
        Cmd::SetInput { id, port, src } => {
            r[0] = SET_INPUT;
            r[1] = port;
            put_id(&mut r, id);
            put_a(&mut r, src);
        }
        Cmd::Gate { id, on } => {
            r[0] = GATE;
            r[1] = on as u8;
            put_id(&mut r, id);
        }
        Cmd::Trigger { id } => {
            r[0] = TRIGGER;
            put_id(&mut r, id);
        }
        Cmd::SetRoot { id } => {
            r[0] = SET_ROOT;
            put_id(&mut r, id);
        }
        Cmd::Reset => r[0] = RESET,
    }
    r
}

/// Decode one 16-byte record back into a command.
pub fn decode(r: &[u8]) -> Cmd {
    let id = u16::from_le_bytes([r[2], r[3]]);
    let a = dec_input(r[4], u32::from_le_bytes([r[6], r[7], r[8], r[9]]));
    let b = dec_input(r[10], u32::from_le_bytes([r[12], r[13], r[14], r[15]]));
    match r[0] {
        NEW_NODE => Cmd::NewNode { id, kind: r[1], a, b },
        SET_INPUT => Cmd::SetInput { id, port: r[1], src: a },
        GATE => Cmd::Gate { id, on: r[1] != 0 },
        TRIGGER => Cmd::Trigger { id },
        SET_ROOT => Cmd::SetRoot { id },
        RESET => Cmd::Reset,
        _ => Cmd::Nop,
    }
}
