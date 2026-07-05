//! Harness Layer 1 (Task 1.3): run a deluge project's entry through the same
//! VM/prelude machinery as [`crate::build_vm`]/[`crate::run_project_capture`]
//! and expose the resulting engine state (CV/gate) so later phases (and the
//! debugger UI) can inspect what a script did.
//!
//! [`crate::build_vm`]/[`crate::run_project_capture`] install [`crate::NoopHost`]
//! — fine for compile/print-capture tests, but Output/Gate writes vanish into
//! it. This module installs its own recording [`Host`] instead (mirroring
//! `tools/wren-web/src/lib.rs`'s `WebHost` and `deluge-wren-core`'s test-only
//! `TestHost`, which the ungated harness can't reuse directly — see the
//! module docs on `deluge_wren_core::test_support`), so a run's CV/gate
//! writes are observable afterward via [`Harness::cv`]/[`Harness::gate`].

use std::collections::HashSet;

use deluge_wren_core::{CV_CHANNELS, Cmd, GATE_CHANNELS, Host, set_host};
use wren_core::foreign::WrenSlotApi;
use wren_core::vm::{CWrenVm, DebugStop};

use crate::drive::DriveEvent;
use crate::slotapi_wrencore::CoreSlots;

/// Harness Layer 2 (Task 5.1): run `src` under the debug agent breaking at
/// `bp_line`, drive a single MIDI note-on (`note`/`vel`) into the VM with the
/// debugger hook still attached, and return the source line the VM stopped at
/// — or `-1` if it terminated without stopping.
///
/// This is the payoff test-driver for driven callbacks: `src` registers a
/// `Midi.onNoteOn` handler at top level, and the note-on fires that handler
/// via [`deluge_wren_core::midi_rx_impl`] *after* `interpret` returns, so a
/// breakpoint inside the handler parks the VM thread. After capturing the stop
/// line, this drains the session to termination (resume until
/// [`DebugStop::Terminated`]) so the parked VM thread exits cleanly before the
/// next VM boots — the same discipline `tests/common::drive_to_end` uses.
pub fn debug_drive_note(src: &str, bp_line: i32, note: u8, vel: u8) -> i32 {
    let session = crate::agent::debug_run_driven(
        src,
        Vec::new(),
        HashSet::from([bp_line]),
        vec![DriveEvent::NoteOn { note, vel }],
    );

    let line = loop {
        match session.wait_event() {
            DebugStop::Stopped { line, .. } => break line,
            DebugStop::Terminated => break -1,
            // Async diagnostic output — keep waiting for the real stop.
            DebugStop::Output { .. } => continue,
        }
    };

    if line != -1 {
        // Resume and pump until the VM thread actually exits, so the parked
        // thread doesn't outlive this call and race a later VM boot.
        session.resume();
        loop {
            match session.wait_event() {
                DebugStop::Terminated => break,
                DebugStop::Stopped { .. } => session.resume(),
                DebugStop::Output { .. } => {}
            }
        }
    }

    line
}

/// Records every CV/gate write in memory; every other [`Host`] effect (MIDI,
/// LEDs, OLED, audio-graph commands) is a sink. Layer 1 only needs to observe
/// CV/gate state — later phases can extend this (or add a sibling host) if
/// the debugger UI needs OLED/LED/audio readouts too.
struct RecordingHost {
    now_ms: u64,
    cv: [f32; CV_CHANNELS],
    gate: [bool; GATE_CHANNELS],
}

impl RecordingHost {
    fn new() -> Self {
        RecordingHost { now_ms: 0, cv: [0.0; CV_CHANNELS], gate: [false; GATE_CHANNELS] }
    }
}

impl Host for RecordingHost {
    fn now_ms(&mut self) -> u64 {
        self.now_ms
    }
    fn cv_set(&mut self, ch: u8, volts: f32) {
        if (ch as usize) < CV_CHANNELS {
            self.cv[ch as usize] = volts;
        }
    }
    fn gate_set(&mut self, ch: u8, on: bool) {
        if (ch as usize) < GATE_CHANNELS {
            self.gate[ch as usize] = on;
        }
    }
    fn midi_tx(&mut self, _msg: &[u8]) {}
    fn led(&mut self, _id: u8, _on: bool) {}
    fn oled_clear(&mut self) {}
    fn oled_text(&mut self, _x: usize, _y: usize, _text: &[u8]) {}
    fn oled_pixel(&mut self, _x: usize, _y: usize, _on: bool) {}
    fn oled_show(&mut self) {}
    fn audio_cmd(&mut self, _cmd: Cmd) {}
}

/// Runs a deluge project's entry module and exposes engine state (CV/gate)
/// after the run.
///
/// Each [`Harness::run_entry`] call builds a fresh VM (registries + prelude,
/// via [`crate::boot_vm`]) with a fresh [`RecordingHost`] — mirroring
/// [`crate::run_project_capture`]'s one-VM-per-run design — so successive
/// runs never see stale CV/gate/metro state from a previous script.
pub struct Harness {
    /// The most recently run VM, kept alive so its `WrenSlotApi` can still be
    /// used (e.g. by a future Layer 2 debug session) after `run_entry`
    /// returns. `None` before the first run.
    vm: Option<CWrenVm>,
    /// Leaked (`Box::leak`, like `crate::install_noop_host`) alongside
    /// `set_host`; the harness keeps its own pointer so it can read back the
    /// recorded CV/gate state directly, without needing an accessor into
    /// `deluge_wren_core`'s private binding state (`bindings.rs`'s `state()`
    /// stays crate-private there — see the Task 1.3 report for why this was
    /// preferred over exposing one).
    host: *mut RecordingHost,
}

impl Default for Harness {
    fn default() -> Self {
        Self::new()
    }
}

impl Harness {
    /// A harness with no run yet: `cv`/`gate` read power-on defaults (0.0 /
    /// off) until [`Harness::run_entry`] is called.
    pub fn new() -> Self {
        let host: &'static mut RecordingHost = Box::leak(Box::new(RecordingHost::new()));
        Harness { vm: None, host }
    }

    /// Build a fresh VM, install a fresh recording host, run `entry` as the
    /// `main` module (with `modules` importable exactly like
    /// [`crate::run_project_capture`]), then flush CV/gate slew into the host
    /// with one render tick at `dt=0` — the harness runs top-level code only
    /// (no events/metros yet), and `Output.volts=` sets `target` (and
    /// `current` immediately, for the common zero-slew case) in
    /// `deluge-wren-core`'s private state, which only reaches the `Host`
    /// (and so `cv`/`gate`) via a render tick. This mirrors how
    /// `deluge-wren-core`'s own golden-test harness
    /// (`test_support::run_and_read_cv`) and `tick_impl`'s doc describe
    /// observing CV: drive a tick, then read the host.
    ///
    /// Returns the entry's compile/runtime error (if any) rather than
    /// panicking, since a debug harness must survive a broken script.
    pub fn run_entry(&mut self, entry: &str, modules: Vec<(String, String)>) -> Result<(), String> {
        let host: &'static mut RecordingHost = Box::leak(Box::new(RecordingHost::new()));
        self.host = host;
        set_host(host);
        deluge_wren_core::reset();

        let module_map = crate::prelude_import_modules(modules);
        let load_fn = move |name: &str| module_map.get(name).cloned();

        let mut vm = crate::boot_vm(|_s: &str| {}, Some(Box::new(load_fn)));
        let result = vm.interpret("main", entry).map_err(|e| e.to_string());

        let api: &dyn WrenSlotApi = &vm;
        let slots = CoreSlots::new(api);
        deluge_wren_core::tick_impl(&slots, 0, 0.0);

        self.vm = Some(vm);
        result
    }

    /// CV jack `ch` (0-based: `output[1]` in Wren is channel 0) in volts,
    /// post-slew, as of the last [`Harness::run_entry`]. Out-of-range `ch`
    /// reads 0.0.
    pub fn cv(&self, ch: usize) -> f32 {
        // SAFETY: `self.host` is a live `Box::leak`ed pointer (set in `new`
        // and refreshed by every `run_entry`), read only from the single
        // thread that owns this `Harness` — matches `deluge_wren_core`'s
        // single-VM-thread state discipline.
        let host = unsafe { &*self.host };
        host.cv.get(ch).copied().unwrap_or(0.0)
    }

    /// Gate jack `ch` (0-based: `gate[1]` in Wren is channel 0) as of the
    /// last [`Harness::run_entry`]. Out-of-range `ch` reads `false`.
    pub fn gate(&self, ch: usize) -> bool {
        // SAFETY: see `cv`.
        let host = unsafe { &*self.host };
        host.gate.get(ch).copied().unwrap_or(false)
    }
}
