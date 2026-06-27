//! The Deluge Wren foreign bindings: `Output`/`Gate`/`Metro`/`Midi`/`Pads`/
//! `Buttons`/`Enc`/`Led`/`Oled` plus the `Node` audio-graph class, and the
//! prelude that declares them. Target-agnostic — every hardware effect goes
//! through the [`Host`](crate::Host) trait, so these same bindings drive a real
//! Deluge and the web simulator.
//!
//! ## Concurrency model
//! All native state here is touched **only from the VM thread**: foreign methods
//! run inside `wrenInterpret`, and [`tick`] runs between interprets on the same
//! thread. So plain `static mut` is sound — there is no other task or IRQ touching
//! it. The one rule: never hold a `&mut STATE` borrow across a `wrenCall` (a metro
//! callback can re-enter a foreign method, which would take a second `&mut STATE`).
//! [`tick`] captures what it needs, drops the borrow, *then* fires callbacks.

use core::ffi::c_char;
use core::ptr::addr_of_mut;

use wren_sys::{ClassEntry, MethodEntry, Vm, WrenForeign, WrenHandle, WrenType, WrenVM};

use crate::audio;
use crate::engine::{Input, K_ENV, K_LPF, K_MUL, K_NOISE};
use crate::host::{CV_CHANNELS, GATE_CHANNELS, host};

const N_CV: usize = CV_CHANNELS; // 2
const N_GATE: usize = GATE_CHANNELS; // 4
const N_METRO: usize = 8;

/// One CV channel: linear slew from `current` toward `target` at `rate` V/s.
#[derive(Clone, Copy)]
struct CvCh {
    current: f32,
    target: f32,
    rate: f32,
    slew_s: f32,
}
impl CvCh {
    const EMPTY: CvCh = CvCh { current: 0.0, target: 0.0, rate: 0.0, slew_s: 0.0 };
}

/// One metro pool slot.
#[derive(Clone, Copy)]
struct Metro {
    used: bool,
    active: bool,
    interval_s: f32,
    next_ms: u64,
    stage: i64,
    cb: *mut WrenHandle,
}
impl Metro {
    const EMPTY: Metro = Metro {
        used: false,
        active: false,
        interval_s: 0.0,
        next_ms: 0,
        stage: 0,
        cb: core::ptr::null_mut(),
    };
}

struct State {
    cv: [CvCh; N_CV],
    gate: [bool; N_GATE],
    metro: [Metro; N_METRO],
}
impl State {
    const EMPTY: State =
        State { cv: [CvCh::EMPTY; N_CV], gate: [false; N_GATE], metro: [Metro::EMPTY; N_METRO] };
}

static mut STATE: State = State::EMPTY;

/// Reusable `Fn.call(_)` handle, made lazily on the first metro fire. Kept out
/// of `State` so firing a metro doesn't need a `&mut STATE` borrow.
static mut CALL_HANDLE: *mut WrenHandle = core::ptr::null_mut();

/// Borrow the native state. Single-threaded (VM thread only); callers must hold at
/// most one borrow at a time and never across a `wrenCall`.
#[allow(clippy::mut_from_ref)]
fn state() -> &'static mut State {
    // SAFETY: the VM thread is the sole accessor; see module docs.
    unsafe { &mut *addr_of_mut!(STATE) }
}

// ── Per-iteration tick (called by the host's VM loop) ─────────────────────────

/// Advance CV slew + write the DAC/gates, then fire any due metro callbacks.
/// `now_ms` is the current millisecond tick; `dt_s` is seconds since the last
/// tick.
pub fn tick(vm: Vm, now_ms: u64, dt_s: f32) {
    render_cv_gate(dt_s);

    // Fire due metros *without* holding the state borrow across the call.
    for i in 0..N_METRO {
        if let Some((cb, stage)) = metro_take_due(i, now_ms) {
            fire_metro(vm, cb, stage);
        }
    }
}

/// Advance slew and push every CV/gate channel to the host. Brief state borrow,
/// no wren calls.
fn render_cv_gate(dt_s: f32) {
    let st = state();
    for ch in 0..N_CV {
        let c = &mut st.cv[ch];
        if c.current != c.target {
            if c.slew_s <= 0.0 || c.rate == 0.0 {
                c.current = c.target;
            } else {
                c.current += c.rate * dt_s;
                // Clamp once we reach/overshoot the target.
                if (c.rate > 0.0 && c.current >= c.target)
                    || (c.rate < 0.0 && c.current <= c.target)
                {
                    c.current = c.target;
                }
            }
        }
        // Hand the post-slew voltage to the host (it maps volts → hardware).
        host().cv_set(ch as u8, c.current);
    }
    for ch in 0..N_GATE {
        host().gate_set(ch as u8, st.gate[ch]);
    }
}

/// If metro `i` is active and due at `now_ms`, advance its schedule and return
/// its `(callback, stage)`; else `None`. Brief state borrow only.
fn metro_take_due(i: usize, now_ms: u64) -> Option<(*mut WrenHandle, i64)> {
    let st = state();
    let m = &mut st.metro[i];
    if !m.active || m.cb.is_null() || now_ms < m.next_ms {
        return None;
    }
    m.stage += 1;
    let interval_ms = (m.interval_s * 1000.0) as u64;
    m.next_ms = now_ms + interval_ms.max(1);
    Some((m.cb, m.stage))
}

/// Invoke a metro callback `cb.call(stage)`. No state borrow held.
fn fire_metro(vm: Vm, cb: *mut WrenHandle, stage: i64) {
    // SAFETY: the VM thread is the sole accessor of CALL_HANDLE.
    let call = unsafe {
        if CALL_HANDLE.is_null() {
            CALL_HANDLE = vm.make_call_handle("call(_)");
        }
        CALL_HANDLE
    };
    if call.is_null() {
        return;
    }
    vm.ensure_slots(2);
    vm.set_handle(0, cb); // receiver = the Fn
    vm.set_f64(1, stage as f64);
    vm.call(call); // ignore result; a throwing callback is reported by errorFn
}

// ── Foreign object structs ───────────────────────────────────────────────────

#[derive(Clone, Copy)]
struct OutputObj {
    ch: u32,
}
impl WrenForeign for OutputObj {
    fn class_name() -> &'static str {
        "Output"
    }
}

#[derive(Clone, Copy)]
struct GateObj {
    ch: u32,
}
impl WrenForeign for GateObj {
    fn class_name() -> &'static str {
        "Gate"
    }
}

#[derive(Clone, Copy)]
struct MetroObj {
    idx: u32,
}
impl WrenForeign for MetroObj {
    fn class_name() -> &'static str {
        "Metro"
    }
}

// ── Output (CV) methods ──────────────────────────────────────────────────────

unsafe extern "C" fn output_alloc(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let ch = vm.get_f64(1) as u32;
    unsafe { vm.alloc_foreign(OutputObj { ch }) };
}

unsafe extern "C" fn output_volts_get(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let ch = unsafe { vm.foreign_mut::<OutputObj>(0) }.ch as usize;
    let v = if ch < N_CV { state().cv[ch].current } else { 0.0 };
    vm.set_f64(0, v as f64);
}

unsafe extern "C" fn output_volts_set(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let v = vm.get_f64(1) as f32;
    let ch = unsafe { vm.foreign_mut::<OutputObj>(0) }.ch as usize;
    if ch < N_CV {
        let c = &mut state().cv[ch];
        c.target = v;
        c.rate = if c.slew_s <= 0.0 { 0.0 } else { (v - c.current) / c.slew_s };
        if c.slew_s <= 0.0 {
            c.current = v;
        }
    }
}

unsafe extern "C" fn output_slew_set(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let s = vm.get_f64(1) as f32;
    let ch = unsafe { vm.foreign_mut::<OutputObj>(0) }.ch as usize;
    if ch < N_CV {
        state().cv[ch].slew_s = s.max(0.0);
    }
}

// ── Gate methods ─────────────────────────────────────────────────────────────

unsafe extern "C" fn gate_alloc(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let ch = vm.get_f64(1) as u32;
    unsafe { vm.alloc_foreign(GateObj { ch }) };
}

unsafe extern "C" fn gate_on_set(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let on = vm.get_bool(1);
    let ch = unsafe { vm.foreign_mut::<GateObj>(0) }.ch as usize;
    if ch < N_GATE {
        state().gate[ch] = on;
    }
}

// ── Metro methods ────────────────────────────────────────────────────────────

unsafe extern "C" fn metro_alloc(raw: *mut WrenVM) {
    let vm = Vm(raw);
    // Claim a free pool slot.
    let st = state();
    let mut idx = N_METRO as u32;
    for (i, m) in st.metro.iter_mut().enumerate() {
        if !m.used {
            m.used = true;
            m.active = false;
            m.cb = core::ptr::null_mut();
            idx = i as u32;
            break;
        }
    }
    unsafe { vm.alloc_foreign(MetroObj { idx }) };
}

/// `start(fn, seconds)` — store the callback and begin firing.
unsafe extern "C" fn metro_start(raw: *mut WrenVM) {
    let vm = Vm(raw);
    // Take a persistent handle to the Fn in slot 1 *before* borrowing state.
    let cb = vm.get_handle(1);
    let seconds = vm.get_f64(2) as f32;
    let idx = unsafe { vm.foreign_mut::<MetroObj>(0) }.idx as usize;
    let now_ms = host().now_ms();
    if idx < N_METRO {
        let m = &mut state().metro[idx];
        if !m.cb.is_null() {
            // Replacing an existing callback: release the old handle.
            vm.release_handle(m.cb);
        }
        m.cb = cb;
        m.interval_s = seconds.max(0.0);
        m.stage = 0;
        m.active = true;
        m.next_ms = now_ms + ((seconds.max(0.0) * 1000.0) as u64).max(1);
    } else {
        // No slot: drop the handle we took.
        vm.release_handle(cb);
    }
}

unsafe extern "C" fn metro_stop(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let idx = unsafe { vm.foreign_mut::<MetroObj>(0) }.idx as usize;
    if idx < N_METRO {
        let m = &mut state().metro[idx];
        m.active = false;
        if !m.cb.is_null() {
            vm.release_handle(m.cb);
            m.cb = core::ptr::null_mut();
        }
    }
}

unsafe extern "C" fn metro_time_set(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let s = vm.get_f64(1) as f32;
    let idx = unsafe { vm.foreign_mut::<MetroObj>(0) }.idx as usize;
    if idx < N_METRO {
        state().metro[idx].interval_s = s.max(0.0);
    }
}

// ── MIDI (DIN in/out) ────────────────────────────────────────────────────────
//
// `Midi` is a static-only foreign class. RX messages are parsed by the host and
// dispatched here to the registered callbacks; TX messages go to the host's MIDI
// sink (`Host::midi_tx`).

struct MidiState {
    on_note_on: *mut WrenHandle,
    on_note_off: *mut WrenHandle,
    on_cc: *mut WrenHandle,
    /// Reusable `call(_,_,_)` handle for the 3-arg callbacks.
    call3: *mut WrenHandle,
}

static mut MIDI: MidiState = MidiState {
    on_note_on: core::ptr::null_mut(),
    on_note_off: core::ptr::null_mut(),
    on_cc: core::ptr::null_mut(),
    call3: core::ptr::null_mut(),
};

#[allow(clippy::mut_from_ref)]
fn midi() -> &'static mut MidiState {
    // SAFETY: the VM thread is the sole accessor (see module docs).
    unsafe { &mut *addr_of_mut!(MIDI) }
}

/// Number of bytes a channel-voice message carries, by status byte.
fn midi_len(status: u8) -> usize {
    match status & 0xF0 {
        0xC0 | 0xD0 => 2,
        0x80 | 0x90 | 0xA0 | 0xB0 | 0xE0 => 3,
        _ => 1,
    }
}

/// Push a TX message to the host MIDI sink (length implied by the status).
fn tx(b1: u8, b2: u8, b3: u8) {
    let buf = [b1, b2, b3];
    host().midi_tx(&buf[..midi_len(b1)]);
}

/// Channel arg (wren uses 1..16) → wire status nibble.
fn status_for(kind: u8, ch_arg: f64) -> u8 {
    let ch = (ch_arg as i32 - 1).clamp(0, 15) as u8;
    kind | ch
}

unsafe extern "C" fn midi_note_on(raw: *mut WrenVM) {
    let vm = Vm(raw);
    tx(status_for(0x90, vm.get_f64(1)), vm.get_f64(2) as u8, vm.get_f64(3) as u8);
}
unsafe extern "C" fn midi_note_off(raw: *mut WrenVM) {
    let vm = Vm(raw);
    tx(status_for(0x80, vm.get_f64(1)), vm.get_f64(2) as u8, vm.get_f64(3) as u8);
}
unsafe extern "C" fn midi_cc(raw: *mut WrenVM) {
    let vm = Vm(raw);
    tx(status_for(0xB0, vm.get_f64(1)), vm.get_f64(2) as u8, vm.get_f64(3) as u8);
}
unsafe extern "C" fn midi_send(raw: *mut WrenVM) {
    let vm = Vm(raw);
    tx(vm.get_f64(1) as u8, vm.get_f64(2) as u8, vm.get_f64(3) as u8);
}

/// Replace a stored callback handle with the Fn in slot 1 (releasing the old).
fn set_cb(vm: Vm, which: fn(&mut MidiState) -> &mut *mut WrenHandle) {
    let h = vm.get_handle(1);
    let slot = which(midi());
    if !slot.is_null() {
        vm.release_handle(*slot);
    }
    *slot = h;
}

unsafe extern "C" fn midi_set_on_note_on(raw: *mut WrenVM) {
    set_cb(Vm(raw), |m| &mut m.on_note_on);
}
unsafe extern "C" fn midi_set_on_note_off(raw: *mut WrenVM) {
    set_cb(Vm(raw), |m| &mut m.on_note_off);
}
unsafe extern "C" fn midi_set_on_cc(raw: *mut WrenVM) {
    set_cb(Vm(raw), |m| &mut m.on_cc);
}

/// Dispatch a parsed channel-voice MIDI message to the wren callbacks. Called by
/// the host's VM loop (not inside a foreign method), so `wrenCall` is legal.
/// Note-on with velocity 0 is treated as note-off (MIDI convention).
pub fn midi_rx(vm: Vm, status: u8, d1: u8, d2: u8) {
    let ch = (status & 0x0F) as f64 + 1.0;
    let (cb, a, b, c) = match status & 0xF0 {
        0x90 if d2 > 0 => (midi().on_note_on, ch, d1 as f64, d2 as f64),
        0x90 | 0x80 => (midi().on_note_off, ch, d1 as f64, d2 as f64),
        0xB0 => (midi().on_cc, ch, d1 as f64, d2 as f64),
        _ => return,
    };
    if cb.is_null() {
        return;
    }
    // Lazily make the 3-arg call handle.
    let call = {
        let m = midi();
        if m.call3.is_null() {
            m.call3 = vm.make_call_handle("call(_,_,_)");
        }
        m.call3
    };
    if call.is_null() {
        return;
    }
    vm.ensure_slots(4);
    vm.set_handle(0, cb);
    vm.set_f64(1, a);
    vm.set_f64(2, b);
    vm.set_f64(3, c);
    vm.call(call);
}

// ── UI: pads / buttons / encoders (in) + LEDs / OLED (out) ───────────────────
//
// Input events are produced by the host (pads/buttons/encoders) and dispatched
// here to the registered callbacks. Output (LEDs, OLED) goes to the host sinks.

struct UiState {
    on_pad_press: *mut WrenHandle,
    on_pad_release: *mut WrenHandle,
    on_button_press: *mut WrenHandle,
    on_button_release: *mut WrenHandle,
    on_enc: *mut WrenHandle,
    /// `call(_)` / `call(_,_)` reusable handles.
    call1: *mut WrenHandle,
    call2: *mut WrenHandle,
}

static mut UI: UiState = UiState {
    on_pad_press: core::ptr::null_mut(),
    on_pad_release: core::ptr::null_mut(),
    on_button_press: core::ptr::null_mut(),
    on_button_release: core::ptr::null_mut(),
    on_enc: core::ptr::null_mut(),
    call1: core::ptr::null_mut(),
    call2: core::ptr::null_mut(),
};

#[allow(clippy::mut_from_ref)]
fn ui() -> &'static mut UiState {
    // SAFETY: the VM thread is the sole accessor (see module docs).
    unsafe { &mut *addr_of_mut!(UI) }
}

fn ui_call1(vm: Vm, cb: *mut WrenHandle, a: f64) {
    if cb.is_null() {
        return;
    }
    let call = {
        let u = ui();
        if u.call1.is_null() {
            u.call1 = vm.make_call_handle("call(_)");
        }
        u.call1
    };
    if call.is_null() {
        return;
    }
    vm.ensure_slots(2);
    vm.set_handle(0, cb);
    vm.set_f64(1, a);
    vm.call(call);
}

fn ui_call2(vm: Vm, cb: *mut WrenHandle, a: f64, b: f64) {
    if cb.is_null() {
        return;
    }
    let call = {
        let u = ui();
        if u.call2.is_null() {
            u.call2 = vm.make_call_handle("call(_,_)");
        }
        u.call2
    };
    if call.is_null() {
        return;
    }
    vm.ensure_slots(3);
    vm.set_handle(0, cb);
    vm.set_f64(1, a);
    vm.set_f64(2, b);
    vm.call(call);
}

/// Dispatch an input event to the wren callbacks. Called by the host. `kind`:
/// 0=pad press, 1=pad release (`a`=x, `b`=y); 2=button press, 3=button release
/// (`a`=id). Pad coordinates arrive pre-decoded from the host's input stream.
pub fn input_dispatch(vm: Vm, kind: u8, a: u8, b: u8) {
    match kind {
        0 | 1 => {
            let cb = if kind == 0 { ui().on_pad_press } else { ui().on_pad_release };
            ui_call2(vm, cb, a as f64, b as f64);
        }
        2 | 3 => {
            let cb = if kind == 2 { ui().on_button_press } else { ui().on_button_release };
            ui_call1(vm, cb, a as f64);
        }
        _ => {}
    }
}

/// Dispatch an encoder detent change. Called by the host.
pub fn enc_turn(vm: Vm, index: u8, delta: i8) {
    ui_call2(vm, ui().on_enc, index as f64, delta as f64);
}

/// Replace a UI callback handle with the Fn in slot 1 (releasing the old).
fn set_ui_cb(vm: Vm, which: fn(&mut UiState) -> &mut *mut WrenHandle) {
    let h = vm.get_handle(1);
    let slot = which(ui());
    if !slot.is_null() {
        vm.release_handle(*slot);
    }
    *slot = h;
}

unsafe extern "C" fn pads_on_press(raw: *mut WrenVM) {
    set_ui_cb(Vm(raw), |u| &mut u.on_pad_press);
}
unsafe extern "C" fn pads_on_release(raw: *mut WrenVM) {
    set_ui_cb(Vm(raw), |u| &mut u.on_pad_release);
}
unsafe extern "C" fn buttons_on_press(raw: *mut WrenVM) {
    set_ui_cb(Vm(raw), |u| &mut u.on_button_press);
}
unsafe extern "C" fn buttons_on_release(raw: *mut WrenVM) {
    set_ui_cb(Vm(raw), |u| &mut u.on_button_release);
}
unsafe extern "C" fn enc_on_turn(raw: *mut WrenVM) {
    set_ui_cb(Vm(raw), |u| &mut u.on_enc);
}

// LEDs
unsafe extern "C" fn led_on(raw: *mut WrenVM) {
    let vm = Vm(raw);
    host().led(vm.get_f64(1) as u8, true);
}
unsafe extern "C" fn led_off(raw: *mut WrenVM) {
    let vm = Vm(raw);
    host().led(vm.get_f64(1) as u8, false);
}

// OLED
unsafe extern "C" fn oled_clear(_raw: *mut WrenVM) {
    host().oled_clear();
}
unsafe extern "C" fn oled_text(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let x = vm.get_f64(1) as usize;
    let y = vm.get_f64(2) as usize;
    let s = vm.get_str(3);
    host().oled_text(x, y, s.as_bytes());
}
unsafe extern "C" fn oled_pixel(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let x = vm.get_f64(1) as usize;
    let y = vm.get_f64(2) as usize;
    let on = vm.get_bool(3);
    host().oled_pixel(x, y, on);
}
unsafe extern "C" fn oled_show(_raw: *mut WrenVM) {
    host().oled_show();
}

// ── Audio: DSP node graph (`Node` foreign class) ─────────────────────────────
//
// The Wren `Node` foreign object just holds a node id into the engine
// (`crate::audio`/`crate::engine`). Factory statics (`src_`/`env_`/…) allocate a
// node and return a fresh `Node`; instance methods/operators mutate it via the
// command queue. The prelude's `Osc`/`Env`/`Noise`/`Out` classes are thin
// wrappers over these.

#[derive(Clone, Copy)]
struct NodeObj {
    id: u32,
}
impl WrenForeign for NodeObj {
    fn class_name() -> &'static str {
        "Node"
    }
}

/// Read a "number or Node" argument at `slot` into an engine [`Input`].
fn arg_input(vm: Vm, slot: i32) -> Input {
    match vm.slot_type(slot) {
        WrenType::Num => Input::Const(vm.get_f64(slot) as f32),
        WrenType::Foreign => Input::Node(unsafe { vm.foreign_mut::<NodeObj>(slot) }.id as u16),
        _ => Input::Const(0.0),
    }
}

/// `id` of the `Node` receiver in slot 0.
fn self_id(vm: Vm) -> u16 {
    unsafe { vm.foreign_mut::<NodeObj>(0) }.id as u16
}

/// Return a fresh `Node` wrapping engine node `id` in slot 0.
unsafe fn return_node(vm: Vm, id: u16) {
    unsafe { vm.new_foreign_in(0, NodeObj { id: id as u32 }) };
}

// Factory statics (return a Node).
unsafe extern "C" fn node_src(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let kind = vm.get_f64(1) as u8;
    let freq = arg_input(vm, 2);
    let id = audio::alloc_node(kind, freq, Input::Const(0.0));
    unsafe { return_node(vm, id) };
}
unsafe extern "C" fn node_env(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let a = arg_input(vm, 1);
    let b = arg_input(vm, 2);
    let id = audio::alloc_node(K_ENV, a, b);
    unsafe { return_node(vm, id) };
}
unsafe extern "C" fn node_noise(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let id = audio::alloc_node(K_NOISE, Input::Const(0.0), Input::Const(0.0));
    unsafe { return_node(vm, id) };
}
unsafe extern "C" fn node_binop(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let op = vm.get_f64(1) as u8; // 0=mul, 1=add, 2=sub
    let a = arg_input(vm, 2);
    let b = arg_input(vm, 3);
    let id = audio::alloc_node(K_MUL + op, a, b);
    unsafe { return_node(vm, id) };
}
unsafe extern "C" fn node_lpf(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let input = arg_input(vm, 1);
    let cutoff = arg_input(vm, 2);
    let id = audio::alloc_node(K_LPF, input, cutoff);
    unsafe { return_node(vm, id) };
}
unsafe extern "C" fn node_patch(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let id = unsafe { vm.foreign_mut::<NodeObj>(1) }.id as u16;
    audio::set_root(id);
}
unsafe extern "C" fn node_reset(_raw: *mut WrenVM) {
    audio::reset();
}

// Instance methods (self = slot 0).
unsafe extern "C" fn node_set_freq(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 0, v);
}
unsafe extern "C" fn node_set_cutoff(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 1, v);
}
unsafe extern "C" fn node_gate(raw: *mut WrenVM) {
    let vm = Vm(raw);
    let on = vm.get_bool(1);
    audio::gate(self_id(vm), on);
}
unsafe extern "C" fn node_trigger(raw: *mut WrenVM) {
    let vm = Vm(raw);
    audio::trigger(self_id(vm));
}

// ── Registry tables ──────────────────────────────────────────────────────────

pub static CLASSES: &[ClassEntry] = &[
    ClassEntry { module: "main", class: "Output", allocate: output_alloc, finalize: None },
    ClassEntry { module: "main", class: "Gate", allocate: gate_alloc, finalize: None },
    ClassEntry { module: "main", class: "Metro", allocate: metro_alloc, finalize: None },
];

pub static METHODS: &[MethodEntry] = &[
    // Output
    method("Output", "volts", output_volts_get),
    method("Output", "volts=(_)", output_volts_set),
    method("Output", "slew=(_)", output_slew_set),
    // Gate
    method("Gate", "on=(_)", gate_on_set),
    // Metro
    method("Metro", "start(_,_)", metro_start),
    method("Metro", "stop()", metro_stop),
    method("Metro", "time=(_)", metro_time_set),
    // Midi (static)
    static_method("Midi", "noteOn(_,_,_)", midi_note_on),
    static_method("Midi", "noteOff(_,_,_)", midi_note_off),
    static_method("Midi", "cc(_,_,_)", midi_cc),
    static_method("Midi", "send(_,_,_)", midi_send),
    static_method("Midi", "onNoteOn=(_)", midi_set_on_note_on),
    static_method("Midi", "onNoteOff=(_)", midi_set_on_note_off),
    static_method("Midi", "onCC=(_)", midi_set_on_cc),
    // Pads / Buttons / Enc (static input callbacks)
    static_method("Pads", "onPress=(_)", pads_on_press),
    static_method("Pads", "onRelease=(_)", pads_on_release),
    static_method("Buttons", "onPress=(_)", buttons_on_press),
    static_method("Buttons", "onRelease=(_)", buttons_on_release),
    static_method("Enc", "onTurn=(_)", enc_on_turn),
    // Led / Oled (static output)
    static_method("Led", "on(_)", led_on),
    static_method("Led", "off(_)", led_off),
    static_method("Oled", "clear()", oled_clear),
    static_method("Oled", "text(_,_,_)", oled_text),
    static_method("Oled", "pixel(_,_,_)", oled_pixel),
    static_method("Oled", "show()", oled_show),
    // Audio: Node factories (static) + instance methods
    static_method("Node", "src_(_,_)", node_src),
    static_method("Node", "env_(_,_)", node_env),
    static_method("Node", "noise_()", node_noise),
    static_method("Node", "binop_(_,_,_)", node_binop),
    static_method("Node", "lpf_(_,_)", node_lpf),
    static_method("Node", "patch_(_)", node_patch),
    static_method("Node", "reset_()", node_reset),
    method("Node", "freq=(_)", node_set_freq),
    method("Node", "cutoff=(_)", node_set_cutoff),
    method("Node", "gate(_)", node_gate),
    method("Node", "trigger()", node_trigger),
];

/// Terse instance-`MethodEntry` constructor for the `main` module.
const fn method(
    class: &'static str,
    signature: &'static str,
    func: unsafe extern "C" fn(*mut WrenVM),
) -> MethodEntry {
    MethodEntry { module: "main", class, is_static: false, signature, func }
}

/// Terse static-`MethodEntry` constructor for the `main` module.
const fn static_method(
    class: &'static str,
    signature: &'static str,
    func: unsafe extern "C" fn(*mut WrenVM),
) -> MethodEntry {
    MethodEntry { module: "main", class, is_static: true, signature, func }
}

// ── Wren prelude (compiled at boot, before user scripts) ─────────────────────

/// The prelude (declares the foreign classes + `output[]`/`gate[]`), embedded
/// from `wren/prelude.wren` with a trailing NUL appended at compile time so it
/// can be handed to the C VM as a C string.
const PRELUDE: &str = concat!(include_str!("../wren/prelude.wren"), "\0");

/// The prelude source as a `*const c_char` for `wren_sys::interpret`.
pub fn prelude_ptr() -> *const c_char {
    PRELUDE.as_ptr() as *const c_char
}

/// Tear down all VM-referencing binding state. A host calls this when it frees
/// the VM to run a script fresh (e.g. the web "Run" button): handles are nulled
/// rather than released — the VM that owned them is gone — and CV/gate/metro
/// state plus the audio-graph node-id allocator are reset. Not used on the device
/// (its VM lives for the session).
pub fn reset() {
    *state() = State::EMPTY;

    let m = midi();
    m.on_note_on = core::ptr::null_mut();
    m.on_note_off = core::ptr::null_mut();
    m.on_cc = core::ptr::null_mut();
    m.call3 = core::ptr::null_mut();

    let u = ui();
    u.on_pad_press = core::ptr::null_mut();
    u.on_pad_release = core::ptr::null_mut();
    u.on_button_press = core::ptr::null_mut();
    u.on_button_release = core::ptr::null_mut();
    u.on_enc = core::ptr::null_mut();
    u.call1 = core::ptr::null_mut();
    u.call2 = core::ptr::null_mut();

    // SAFETY: VM thread is the sole accessor.
    unsafe { CALL_HANDLE = core::ptr::null_mut() };

    crate::audio::reset_ids();
}
