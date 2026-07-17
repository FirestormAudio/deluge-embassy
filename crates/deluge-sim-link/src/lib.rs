//! In-process link between an SDK app ("brain") and the desktop simulator GUI
//! ("panel"), used by `cargo deluge sim`.
//!
//! When an SDK app is built for the host triple, the app logic and the simulator
//! window run in **one process**: the app's embassy executor runs on a worker
//! thread (the brain), and `iced` owns the main thread (the panel). They share
//! state through this crate instead of a wire protocol — no sockets, no
//! serialization.
//!
//! - [`SharedPanel`] carries *illumination* (the app writes OLED / pad RGB / LED
//!   state; the GUI reads it each frame) and *input* (the GUI pushes pad /
//!   button / encoder events; the app drains them).
//! - [`audio`] carries the real-time audio block stream over lock-free SPSC rings
//!   ([`audio::brain_ends`] / [`audio::gui_ends`]); the GUI's audio callback is the
//!   clock that paces the app's DSP loop.
//!
//! All ids here are **SDK-native** (raw PIC button id 0–35, encoder index 0–5,
//! raw PIC LED index `x + 9*y`); the GUI is responsible for translating its own
//! control enums to/from these so app code never sees wire ids.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

pub mod audio;

/// OLED frame size in bytes (128×48, 1bpp, page-major — `buf[page*128 + col]`).
pub const DISPLAY_BYTES: usize = 768;
/// Pad grid columns (16 main + 2 sidebar).
pub const PAD_COLS: usize = 18;
/// Pad grid rows.
pub const PAD_ROWS: usize = 8;
/// `SetAllPads` payload size: col-major `[r,g,b]`, offset `(col*ROWS+row)*3`.
pub const ALL_PADS_BYTES: usize = PAD_COLS * PAD_ROWS * 3;
/// Upper bound on raw PIC LED indices (`x + 9*y`) we track.
pub const LED_COUNT: usize = 256;

/// A panel input event, in SDK-native coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputEvent {
    /// A grid pad changed state. `x` 0–17, `y` 0–7.
    Pad { x: u8, y: u8, pressed: bool },
    /// A button changed state. `id` is the raw PIC button id (0–35).
    Button { id: u8, pressed: bool },
    /// An encoder turned. `index` 0–5; `delta` signed detents (positive = CW).
    Encoder { index: u8, delta: i8 },
}

/// Shared illumination + input state. Cheap to [`clone`](Clone) (an `Arc`); one
/// clone lives in the app's host backend, another in the GUI.
#[derive(Clone)]
pub struct SharedPanel {
    inner: Arc<Inner>,
}

struct Inner {
    display: Mutex<[u8; DISPLAY_BYTES]>,
    /// Col-major `[r,g,b]` per pad: `pads[col][row]`.
    pads: Mutex<[[[u8; 3]; PAD_ROWS]; PAD_COLS]>,
    /// Indexed by raw PIC LED index.
    leds: Mutex<[bool; LED_COUNT]>,
    /// Gold-knob ring segment levels: `[lower, upper][segment]`.
    knobs: Mutex<[[u8; 4]; 2]>,
    synced_led: Mutex<bool>,
    /// CV/gate have no panel rendering yet; stored so reads/logging work.
    cv: Mutex<[u16; 4]>,
    gate: Mutex<[bool; 4]>,

    /// Bumped on any OLED change so the GUI re-renders only when needed.
    display_gen: AtomicU64,
    /// Bumped on any pad change.
    pads_gen: AtomicU64,
    /// Bumped on any LED / knob / synced-LED change.
    controls_gen: AtomicU64,
    /// Bumped on any CV change.
    cv_gen: AtomicU64,
    /// Bumped on any gate change.
    gate_gen: AtomicU64,

    /// GUI → app input. Bounded; oldest dropped if the app stalls.
    input: Mutex<VecDeque<InputEvent>>,

    /// App → GUI MIDI output bytes (e.g. forwarded to a host MIDI port).
    midi_out: Mutex<VecDeque<u8>>,
    /// Bumped on each MIDI-out write (drives the OUT activity indicator).
    midi_out_gen: AtomicU64,
    /// GUI → app MIDI input bytes (from a host MIDI port / virtual keyboard).
    midi_in: Mutex<VecDeque<u8>>,
    /// Bumped on each MIDI-in push (drives the IN activity indicator).
    midi_in_gen: AtomicU64,
}

/// Cap on queued input events before the oldest are dropped (mirrors the SDK's
/// on-device 32-deep event channel).
const INPUT_CAP: usize = 64;

impl Default for SharedPanel {
    fn default() -> Self {
        Self::new()
    }
}

impl SharedPanel {
    /// Create an empty panel (display clear, pads/LEDs off, no input queued).
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Inner {
                display: Mutex::new([0u8; DISPLAY_BYTES]),
                pads: Mutex::new([[[0u8; 3]; PAD_ROWS]; PAD_COLS]),
                leds: Mutex::new([false; LED_COUNT]),
                knobs: Mutex::new([[0u8; 4]; 2]),
                synced_led: Mutex::new(false),
                cv: Mutex::new([0u16; 4]),
                gate: Mutex::new([false; 4]),
                display_gen: AtomicU64::new(0),
                pads_gen: AtomicU64::new(0),
                controls_gen: AtomicU64::new(0),
                cv_gen: AtomicU64::new(0),
                gate_gen: AtomicU64::new(0),
                input: Mutex::new(VecDeque::with_capacity(INPUT_CAP)),
                midi_out: Mutex::new(VecDeque::new()),
                midi_out_gen: AtomicU64::new(0),
                midi_in: Mutex::new(VecDeque::new()),
                midi_in_gen: AtomicU64::new(0),
            }),
        }
    }

    // ── illumination: written by the app (brain) ─────────────────────────────

    /// Replace the OLED framebuffer (768 page-major bytes).
    pub fn set_display(&self, buf: &[u8]) {
        if buf.len() != DISPLAY_BYTES {
            return;
        }
        self.inner.display.lock().unwrap().copy_from_slice(buf);
        self.inner.display_gen.fetch_add(1, Ordering::Release);
    }

    /// Clear the OLED to all-off.
    pub fn clear_display(&self) {
        *self.inner.display.lock().unwrap() = [0u8; DISPLAY_BYTES];
        self.inner.display_gen.fetch_add(1, Ordering::Release);
    }

    /// Set a single pad's RGB. `col` 0–17, `row` 0–7.
    pub fn set_pad(&self, col: usize, row: usize, rgb: [u8; 3]) {
        if col >= PAD_COLS || row >= PAD_ROWS {
            return;
        }
        self.inner.pads.lock().unwrap()[col][row] = rgb;
        self.inner.pads_gen.fetch_add(1, Ordering::Release);
    }

    /// Replace the whole pad grid from a `SetAllPads`-style col-major buffer.
    pub fn set_all_pads(&self, buf: &[u8]) {
        if buf.len() != ALL_PADS_BYTES {
            return;
        }
        let mut pads = self.inner.pads.lock().unwrap();
        for col in 0..PAD_COLS {
            for row in 0..PAD_ROWS {
                let o = (col * PAD_ROWS + row) * 3;
                pads[col][row] = [buf[o], buf[o + 1], buf[o + 2]];
            }
        }
        drop(pads);
        self.inner.pads_gen.fetch_add(1, Ordering::Release);
    }

    /// Turn all pads off.
    pub fn clear_all_pads(&self) {
        *self.inner.pads.lock().unwrap() = [[[0u8; 3]; PAD_ROWS]; PAD_COLS];
        self.inner.pads_gen.fetch_add(1, Ordering::Release);
    }

    /// Set an LED by raw PIC index.
    pub fn set_led(&self, index: usize, on: bool) {
        if index >= LED_COUNT {
            return;
        }
        self.inner.leds.lock().unwrap()[index] = on;
        self.inner.controls_gen.fetch_add(1, Ordering::Release);
    }

    /// Turn all LEDs off.
    pub fn clear_all_leds(&self) {
        *self.inner.leds.lock().unwrap() = [false; LED_COUNT];
        self.inner.controls_gen.fetch_add(1, Ordering::Release);
    }

    /// Set a gold-knob ring indicator. `which` 0 = lower, 1 = upper.
    pub fn set_knob_indicator(&self, which: usize, levels: [u8; 4]) {
        if which >= 2 {
            return;
        }
        self.inner.knobs.lock().unwrap()[which] = levels;
        self.inner.controls_gen.fetch_add(1, Ordering::Release);
    }

    /// Set the SYNC LED.
    pub fn set_synced_led(&self, on: bool) {
        *self.inner.synced_led.lock().unwrap() = on;
        self.inner.controls_gen.fetch_add(1, Ordering::Release);
    }

    /// Record a CV channel value (16-bit DAC code; ~6552 codes/V on the device).
    pub fn set_cv(&self, channel: usize, value: u16) {
        if let Some(v) = self.inner.cv.lock().unwrap().get_mut(channel) {
            *v = value;
            self.inner.cv_gen.fetch_add(1, Ordering::Release);
        }
    }

    /// Record a gate channel state.
    pub fn set_gate(&self, channel: usize, on: bool) {
        if let Some(g) = self.inner.gate.lock().unwrap().get_mut(channel) {
            *g = on;
            self.inner.gate_gen.fetch_add(1, Ordering::Release);
        }
    }

    // ── MIDI: app writes out, GUI writes in ──────────────────────────────────

    /// Queue MIDI bytes sent by the app (app → GUI). Bumps the out activity gen.
    pub fn push_midi_out(&self, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        self.inner
            .midi_out
            .lock()
            .unwrap()
            .extend(data.iter().copied());
        self.inner.midi_out_gen.fetch_add(1, Ordering::Release);
    }

    /// Drain all queued MIDI-out bytes (GUI forwards them to a host port).
    pub fn drain_midi_out(&self) -> Vec<u8> {
        self.inner.midi_out.lock().unwrap().drain(..).collect()
    }

    /// Push MIDI bytes from the GUI (GUI → app). Bumps the in activity gen.
    pub fn push_midi_in(&self, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        self.inner
            .midi_in
            .lock()
            .unwrap()
            .extend(data.iter().copied());
        self.inner.midi_in_gen.fetch_add(1, Ordering::Release);
    }

    /// Pop the next MIDI-in byte for the app, if any.
    pub fn pop_midi_in(&self) -> Option<u8> {
        self.inner.midi_in.lock().unwrap().pop_front()
    }

    /// Current MIDI-out activity generation (compare to detect new traffic).
    pub fn midi_out_gen(&self) -> u64 {
        self.inner.midi_out_gen.load(Ordering::Acquire)
    }
    /// Current MIDI-in activity generation.
    pub fn midi_in_gen(&self) -> u64 {
        self.inner.midi_in_gen.load(Ordering::Acquire)
    }

    // ── illumination: read by the GUI (panel) ────────────────────────────────

    /// Current OLED-change generation (compare to detect a new frame).
    pub fn display_gen(&self) -> u64 {
        self.inner.display_gen.load(Ordering::Acquire)
    }
    /// Current pad-change generation.
    pub fn pads_gen(&self) -> u64 {
        self.inner.pads_gen.load(Ordering::Acquire)
    }
    /// Current LED/knob/synced-change generation.
    pub fn controls_gen(&self) -> u64 {
        self.inner.controls_gen.load(Ordering::Acquire)
    }
    /// Current CV-change generation.
    pub fn cv_gen(&self) -> u64 {
        self.inner.cv_gen.load(Ordering::Acquire)
    }
    /// Current gate-change generation.
    pub fn gate_gen(&self) -> u64 {
        self.inner.gate_gen.load(Ordering::Acquire)
    }

    /// Copy the OLED framebuffer out.
    pub fn display_snapshot(&self) -> [u8; DISPLAY_BYTES] {
        *self.inner.display.lock().unwrap()
    }
    /// Copy the pad grid out (`[col][row] = [r,g,b]`).
    pub fn pads_snapshot(&self) -> [[[u8; 3]; PAD_ROWS]; PAD_COLS] {
        *self.inner.pads.lock().unwrap()
    }
    /// Copy the LED-by-index array out.
    pub fn leds_snapshot(&self) -> [bool; LED_COUNT] {
        *self.inner.leds.lock().unwrap()
    }
    /// Copy the knob indicator levels out (`[lower, upper][segment]`).
    pub fn knobs_snapshot(&self) -> [[u8; 4]; 2] {
        *self.inner.knobs.lock().unwrap()
    }
    /// Current SYNC LED state.
    pub fn synced_led(&self) -> bool {
        *self.inner.synced_led.lock().unwrap()
    }
    /// Copy the CV channel codes out (16-bit DAC code per channel).
    pub fn cv_snapshot(&self) -> [u16; 4] {
        *self.inner.cv.lock().unwrap()
    }
    /// Copy the gate channel states out.
    pub fn gate_snapshot(&self) -> [bool; 4] {
        *self.inner.gate.lock().unwrap()
    }

    // ── input: GUI pushes, app drains ────────────────────────────────────────

    /// Push an input event (from the GUI). Drops the oldest if the queue is full.
    pub fn push_event(&self, ev: InputEvent) {
        let mut q = self.inner.input.lock().unwrap();
        if q.len() >= INPUT_CAP {
            q.pop_front();
        }
        q.push_back(ev);
    }

    /// Pop the next queued input event (from the app), if any.
    pub fn pop_event(&self) -> Option<InputEvent> {
        self.inner.input.lock().unwrap().pop_front()
    }
}
