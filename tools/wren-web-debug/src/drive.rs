//! Task 5.1 (harness Layer 2): drive host events into a live debug VM.
//!
//! The debug agent ([`crate::agent::debug_run_driven`]) interprets the entry
//! (which registers callbacks like `Midi.onNoteOn`), then — with the debugger
//! hook still attached — replays a list of [`DriveEvent`]s against the same VM
//! by calling deluge's generic event entries (`midi_rx_impl`/`tick_impl`/…)
//! through a [`CoreSlots`] wrapper. This is the same host-context dispatch the
//! device firmware and web sim do (see `tests/callbacks.rs`), reused here so a
//! breakpoint inside a fired handler parks the VM thread mid-`vm.call`.
//!
//! Each variant maps 1:1 onto a `deluge_wren_core::*_impl` entry; see
//! [`dispatch`]. The type is shared by `agent` and `harness` today and will be
//! threaded through 5.2's `dbg_launch` + controller + drive-panel UI.

use deluge_wren_core::{enc_turn_impl, midi_rx_impl, tick_impl};

use crate::slotapi_wrencore::CoreSlots;

/// A synthetic input event replayed against a debug VM after its entry has
/// registered handlers. MIDI events use channel 1 (status low nibble 0).
#[derive(Debug, Clone, PartialEq)]
pub enum DriveEvent {
    /// MIDI note-on (status `0x90`): fires `Midi.onNoteOn` with (ch, note, vel).
    NoteOn { note: u8, vel: u8 },
    /// MIDI note-off (status `0x80`): fires `Midi.onNoteOff`.
    NoteOff { note: u8, vel: u8 },
    /// MIDI control-change (status `0xB0`): fires `Midi.onCC` with (ch, num, val).
    Cc { num: u8, val: u8 },
    /// A render/metro tick: fires `Metro` callbacks + flushes CV slew.
    Tick { now_ms: u64, dt_s: f32 },
    /// An encoder detent turn: fires `Enc.onTurn` with (index, delta).
    Enc { index: u8, delta: i8 },
}

/// Map a [`DriveEvent`] onto the matching `deluge_wren_core::*_impl` entry,
/// invoked from host/top-level context via `slots`. Must be called only when
/// the VM has no live foreign frame (i.e. after `interpret` returns) — the
/// reentrancy constraint on `WrenSlotApi::call`.
pub(crate) fn dispatch(slots: &CoreSlots, ev: &DriveEvent) {
    match *ev {
        DriveEvent::NoteOn { note, vel } => midi_rx_impl(slots, 0x90, note, vel),
        DriveEvent::NoteOff { note, vel } => midi_rx_impl(slots, 0x80, note, vel),
        DriveEvent::Cc { num, val } => midi_rx_impl(slots, 0xB0, num, val),
        DriveEvent::Tick { now_ms, dt_s } => tick_impl(slots, now_ms, dt_s),
        DriveEvent::Enc { index, delta } => enc_turn_impl(slots, index, delta),
    }
}
