//! The Deluge Wren foreign bindings: `Output`/`Gate`/`Metro`/`Midi`/`Pads`/
//! `Buttons`/`Enc`/`Led`/`Oled` plus the `Node` audio-graph class, and the
//! prelude that declares them. Target-agnostic — every hardware effect goes
//! through the [`Host`](crate::Host) trait, so these same bindings drive a real
//! Deluge and the web simulator.
//!
//! ## Backend-generic bodies
//! Each foreign method's logic lives in a `*_impl<S: SlotApi>(vm: &S)` function,
//! so the same body can run against any [`SlotApi`] implementation (today:
//! `wren_sys::Vm`; later: a second debug-core backend). The registered
//! `unsafe extern "C" fn` is a thin wrapper — `METHODS`/`CLASSES` still point at
//! those wrappers, so the sim/device wren ABI is byte-identical to before.
//! The wrappers (and everything else that names `wren_sys` types) live behind
//! the default-on `wren-sys-backend` feature; the generic bodies, the foreign
//! state structs, and their stored [`Handle`]s are backend-agnostic and stay
//! available with the feature off (see `Cargo.toml`).
//!
//! ## Concurrency model
//! All native state here is touched **only from the VM thread**: foreign methods
//! run inside `wrenInterpret`, and [`tick`] runs between interprets on the same
//! thread. So plain `static mut` is sound — there is no other task or IRQ touching
//! it. The one rule: never hold a `&mut STATE` borrow across a `wrenCall` (a metro
//! callback can re-enter a foreign method, which would take a second `&mut STATE`).
//! [`tick`] captures what it needs, drops the borrow, *then* fires callbacks.

use core::ptr::addr_of_mut;

#[cfg(feature = "wren-sys-backend")]
use core::ffi::c_char;

#[cfg(feature = "wren-sys-backend")]
use wren_sys::{ClassEntry, MethodEntry, Vm, WrenVM};

#[cfg(feature = "wren-sys-backend")]
use crate::bindings_audio;
use crate::host::{CV_CHANNELS, GATE_CHANNELS, host};
use crate::slotapi::{Handle, SlotApi, WrenForeign};

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
    cb: Handle,
}
impl Metro {
    const EMPTY: Metro = Metro {
        used: false,
        active: false,
        interval_s: 0.0,
        next_ms: 0,
        stage: 0,
        cb: Handle(core::ptr::null_mut()),
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
static mut CALL_HANDLE: Handle = Handle(core::ptr::null_mut());

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
#[cfg(feature = "wren-sys-backend")]
pub fn tick(vm: Vm, now_ms: u64, dt_s: f32) {
    tick_impl(&vm, now_ms, dt_s);
}

/// Generic body of [`tick`]: backend-agnostic over any [`SlotApi`], so a debug
/// core (e.g. `wren-web-debug`'s wren-core backend) can drive the same
/// per-iteration tick outside the `wren-sys-backend` feature.
pub fn tick_impl<S: SlotApi>(vm: &S, now_ms: u64, dt_s: f32) {
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
fn metro_take_due(i: usize, now_ms: u64) -> Option<(Handle, i64)> {
    let st = state();
    let m = &mut st.metro[i];
    if !m.active || m.cb.0.is_null() || now_ms < m.next_ms {
        return None;
    }
    m.stage += 1;
    let interval_ms = (m.interval_s * 1000.0) as u64;
    m.next_ms = now_ms + interval_ms.max(1);
    Some((m.cb, m.stage))
}

/// Invoke a metro callback `cb.call(stage)`. No state borrow held.
fn fire_metro<S: SlotApi>(vm: &S, cb: Handle, stage: i64) {
    // SAFETY: the VM thread is the sole accessor of CALL_HANDLE.
    let call = unsafe {
        if CALL_HANDLE.0.is_null() {
            CALL_HANDLE = vm.make_call_handle("call(_)");
        }
        CALL_HANDLE
    };
    if call.0.is_null() {
        return;
    }
    vm.ensure_slots(2);
    vm.set_handle(0, cb); // receiver = the Fn
    vm.set_f(1, stage as f64);
    vm.call(call); // ignore result; a throwing callback is reported by errorFn
}

// ── Foreign object structs ───────────────────────────────────────────────────

#[derive(Clone, Copy)]
struct OutputObj {
    ch: u32,
}
impl WrenForeign for OutputObj {
    fn module_name() -> &'static str {
        "main"
    }
    fn class_name() -> &'static str {
        "Output"
    }
}

#[derive(Clone, Copy)]
struct GateObj {
    ch: u32,
}
impl WrenForeign for GateObj {
    fn module_name() -> &'static str {
        "main"
    }
    fn class_name() -> &'static str {
        "Gate"
    }
}

#[derive(Clone, Copy)]
struct MetroObj {
    idx: u32,
}
impl WrenForeign for MetroObj {
    fn module_name() -> &'static str {
        "main"
    }
    fn class_name() -> &'static str {
        "Metro"
    }
}

// ── Output (CV) methods ──────────────────────────────────────────────────────

pub(crate) fn output_alloc_impl<S: SlotApi>(vm: &S) {
    let ch = vm.get_f(1) as u32;
    unsafe { vm.alloc_foreign(OutputObj { ch }) };
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn output_alloc(raw: *mut WrenVM) {
    let vm = Vm(raw);
    output_alloc_impl(&vm);
}

pub(crate) fn output_volts_get_impl<S: SlotApi>(vm: &S) {
    let ch = unsafe { vm.foreign_mut::<OutputObj>(0) }.ch as usize;
    let v = if ch < N_CV { state().cv[ch].current } else { 0.0 };
    vm.set_f(0, v as f64);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn output_volts_get(raw: *mut WrenVM) {
    let vm = Vm(raw);
    output_volts_get_impl(&vm);
}

pub(crate) fn output_volts_set_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32;
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
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn output_volts_set(raw: *mut WrenVM) {
    let vm = Vm(raw);
    output_volts_set_impl(&vm);
}

pub(crate) fn output_slew_set_impl<S: SlotApi>(vm: &S) {
    let s = vm.get_f(1) as f32;
    let ch = unsafe { vm.foreign_mut::<OutputObj>(0) }.ch as usize;
    if ch < N_CV {
        state().cv[ch].slew_s = s.max(0.0);
    }
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn output_slew_set(raw: *mut WrenVM) {
    let vm = Vm(raw);
    output_slew_set_impl(&vm);
}

// ── Gate methods ─────────────────────────────────────────────────────────────

pub(crate) fn gate_alloc_impl<S: SlotApi>(vm: &S) {
    let ch = vm.get_f(1) as u32;
    unsafe { vm.alloc_foreign(GateObj { ch }) };
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn gate_alloc(raw: *mut WrenVM) {
    let vm = Vm(raw);
    gate_alloc_impl(&vm);
}

pub(crate) fn gate_on_set_impl<S: SlotApi>(vm: &S) {
    let on = vm.get_bool(1);
    let ch = unsafe { vm.foreign_mut::<GateObj>(0) }.ch as usize;
    if ch < N_GATE {
        state().gate[ch] = on;
    }
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn gate_on_set(raw: *mut WrenVM) {
    let vm = Vm(raw);
    gate_on_set_impl(&vm);
}

// ── Metro methods ────────────────────────────────────────────────────────────

pub(crate) fn metro_alloc_impl<S: SlotApi>(vm: &S) {
    // Claim a free pool slot.
    let st = state();
    let mut idx = N_METRO as u32;
    for (i, m) in st.metro.iter_mut().enumerate() {
        if !m.used {
            m.used = true;
            m.active = false;
            m.cb = Handle(core::ptr::null_mut());
            idx = i as u32;
            break;
        }
    }
    unsafe { vm.alloc_foreign(MetroObj { idx }) };
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn metro_alloc(raw: *mut WrenVM) {
    let vm = Vm(raw);
    metro_alloc_impl(&vm);
}

/// `start(fn, seconds)` — store the callback and begin firing.
pub(crate) fn metro_start_impl<S: SlotApi>(vm: &S) {
    // Take a persistent handle to the Fn in slot 1 *before* borrowing state.
    let cb = vm.get_handle(1);
    let seconds = vm.get_f(2) as f32;
    let idx = unsafe { vm.foreign_mut::<MetroObj>(0) }.idx as usize;
    let now_ms = host().now_ms();
    if idx < N_METRO {
        let m = &mut state().metro[idx];
        if !m.cb.0.is_null() {
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
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn metro_start(raw: *mut WrenVM) {
    let vm = Vm(raw);
    metro_start_impl(&vm);
}

pub(crate) fn metro_stop_impl<S: SlotApi>(vm: &S) {
    let idx = unsafe { vm.foreign_mut::<MetroObj>(0) }.idx as usize;
    if idx < N_METRO {
        let m = &mut state().metro[idx];
        m.active = false;
        if !m.cb.0.is_null() {
            vm.release_handle(m.cb);
            m.cb = Handle(core::ptr::null_mut());
        }
    }
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn metro_stop(raw: *mut WrenVM) {
    let vm = Vm(raw);
    metro_stop_impl(&vm);
}

pub(crate) fn metro_time_set_impl<S: SlotApi>(vm: &S) {
    let s = vm.get_f(1) as f32;
    let idx = unsafe { vm.foreign_mut::<MetroObj>(0) }.idx as usize;
    if idx < N_METRO {
        state().metro[idx].interval_s = s.max(0.0);
    }
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn metro_time_set(raw: *mut WrenVM) {
    let vm = Vm(raw);
    metro_time_set_impl(&vm);
}

// ── MIDI (DIN in/out) ────────────────────────────────────────────────────────
//
// `Midi` is a static-only foreign class. RX messages are parsed by the host and
// dispatched here to the registered callbacks; TX messages go to the host's MIDI
// sink (`Host::midi_tx`).

struct MidiState {
    on_note_on: Handle,
    on_note_off: Handle,
    on_cc: Handle,
    /// Reusable `call(_,_,_)` handle for the 3-arg callbacks.
    call3: Handle,
}

static mut MIDI: MidiState = MidiState {
    on_note_on: Handle(core::ptr::null_mut()),
    on_note_off: Handle(core::ptr::null_mut()),
    on_cc: Handle(core::ptr::null_mut()),
    call3: Handle(core::ptr::null_mut()),
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

pub(crate) fn midi_note_on_impl<S: SlotApi>(vm: &S) {
    tx(status_for(0x90, vm.get_f(1)), vm.get_f(2) as u8, vm.get_f(3) as u8);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn midi_note_on(raw: *mut WrenVM) {
    let vm = Vm(raw);
    midi_note_on_impl(&vm);
}

pub(crate) fn midi_note_off_impl<S: SlotApi>(vm: &S) {
    tx(status_for(0x80, vm.get_f(1)), vm.get_f(2) as u8, vm.get_f(3) as u8);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn midi_note_off(raw: *mut WrenVM) {
    let vm = Vm(raw);
    midi_note_off_impl(&vm);
}

pub(crate) fn midi_cc_impl<S: SlotApi>(vm: &S) {
    tx(status_for(0xB0, vm.get_f(1)), vm.get_f(2) as u8, vm.get_f(3) as u8);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn midi_cc(raw: *mut WrenVM) {
    let vm = Vm(raw);
    midi_cc_impl(&vm);
}

pub(crate) fn midi_send_impl<S: SlotApi>(vm: &S) {
    tx(vm.get_f(1) as u8, vm.get_f(2) as u8, vm.get_f(3) as u8);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn midi_send(raw: *mut WrenVM) {
    let vm = Vm(raw);
    midi_send_impl(&vm);
}

/// Replace a stored callback handle with the Fn in slot 1 (releasing the old).
fn set_cb<S: SlotApi>(vm: &S, which: fn(&mut MidiState) -> &mut Handle) {
    let h = vm.get_handle(1);
    let slot = which(midi());
    if !slot.0.is_null() {
        vm.release_handle(*slot);
    }
    *slot = h;
}

pub(crate) fn midi_set_on_note_on_impl<S: SlotApi>(vm: &S) {
    set_cb(vm, |m| &mut m.on_note_on);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn midi_set_on_note_on(raw: *mut WrenVM) {
    let vm = Vm(raw);
    midi_set_on_note_on_impl(&vm);
}

pub(crate) fn midi_set_on_note_off_impl<S: SlotApi>(vm: &S) {
    set_cb(vm, |m| &mut m.on_note_off);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn midi_set_on_note_off(raw: *mut WrenVM) {
    let vm = Vm(raw);
    midi_set_on_note_off_impl(&vm);
}

pub(crate) fn midi_set_on_cc_impl<S: SlotApi>(vm: &S) {
    set_cb(vm, |m| &mut m.on_cc);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn midi_set_on_cc(raw: *mut WrenVM) {
    let vm = Vm(raw);
    midi_set_on_cc_impl(&vm);
}

/// Dispatch a parsed channel-voice MIDI message to the wren callbacks. Called by
/// the host's VM loop (not inside a foreign method), so `wrenCall` is legal.
/// Note-on with velocity 0 is treated as note-off (MIDI convention).
#[cfg(feature = "wren-sys-backend")]
pub fn midi_rx(vm: Vm, status: u8, d1: u8, d2: u8) {
    midi_rx_impl(&vm, status, d1, d2);
}

/// Generic body of [`midi_rx`]: backend-agnostic over any [`SlotApi`], so a
/// debug core can dispatch synthetic MIDI events to the registered wren
/// callbacks from host (top-level) context, outside the `wren-sys-backend`
/// feature.
pub fn midi_rx_impl<S: SlotApi>(vm: &S, status: u8, d1: u8, d2: u8) {
    let ch = (status & 0x0F) as f64 + 1.0;
    let (cb, a, b, c) = match status & 0xF0 {
        0x90 if d2 > 0 => (midi().on_note_on, ch, d1 as f64, d2 as f64),
        0x90 | 0x80 => (midi().on_note_off, ch, d1 as f64, d2 as f64),
        0xB0 => (midi().on_cc, ch, d1 as f64, d2 as f64),
        _ => return,
    };
    if cb.0.is_null() {
        return;
    }
    // Lazily make the 3-arg call handle.
    let call = {
        let m = midi();
        if m.call3.0.is_null() {
            m.call3 = vm.make_call_handle("call(_,_,_)");
        }
        m.call3
    };
    if call.0.is_null() {
        return;
    }
    vm.ensure_slots(4);
    vm.set_handle(0, cb);
    vm.set_f(1, a);
    vm.set_f(2, b);
    vm.set_f(3, c);
    vm.call(call);
}

// ── UI: pads / buttons / encoders (in) + LEDs / OLED (out) ───────────────────
//
// Input events are produced by the host (pads/buttons/encoders) and dispatched
// here to the registered callbacks. Output (LEDs, OLED) goes to the host sinks.

struct UiState {
    on_pad_press: Handle,
    on_pad_release: Handle,
    on_button_press: Handle,
    on_button_release: Handle,
    on_enc: Handle,
    /// `call(_)` / `call(_,_)` reusable handles.
    call1: Handle,
    call2: Handle,
}

static mut UI: UiState = UiState {
    on_pad_press: Handle(core::ptr::null_mut()),
    on_pad_release: Handle(core::ptr::null_mut()),
    on_button_press: Handle(core::ptr::null_mut()),
    on_button_release: Handle(core::ptr::null_mut()),
    on_enc: Handle(core::ptr::null_mut()),
    call1: Handle(core::ptr::null_mut()),
    call2: Handle(core::ptr::null_mut()),
};

#[allow(clippy::mut_from_ref)]
fn ui() -> &'static mut UiState {
    // SAFETY: the VM thread is the sole accessor (see module docs).
    unsafe { &mut *addr_of_mut!(UI) }
}

fn ui_call1<S: SlotApi>(vm: &S, cb: Handle, a: f64) {
    if cb.0.is_null() {
        return;
    }
    let call = {
        let u = ui();
        if u.call1.0.is_null() {
            u.call1 = vm.make_call_handle("call(_)");
        }
        u.call1
    };
    if call.0.is_null() {
        return;
    }
    vm.ensure_slots(2);
    vm.set_handle(0, cb);
    vm.set_f(1, a);
    vm.call(call);
}

fn ui_call2<S: SlotApi>(vm: &S, cb: Handle, a: f64, b: f64) {
    if cb.0.is_null() {
        return;
    }
    let call = {
        let u = ui();
        if u.call2.0.is_null() {
            u.call2 = vm.make_call_handle("call(_,_)");
        }
        u.call2
    };
    if call.0.is_null() {
        return;
    }
    vm.ensure_slots(3);
    vm.set_handle(0, cb);
    vm.set_f(1, a);
    vm.set_f(2, b);
    vm.call(call);
}

/// Dispatch an input event to the wren callbacks. Called by the host. `kind`:
/// 0=pad press, 1=pad release (`a`=x, `b`=y); 2=button press, 3=button release
/// (`a`=id). Pad coordinates arrive pre-decoded from the host's input stream.
#[cfg(feature = "wren-sys-backend")]
pub fn input_dispatch(vm: Vm, kind: u8, a: u8, b: u8) {
    input_dispatch_impl(&vm, kind, a, b);
}

/// Generic body of [`input_dispatch`]: see [`tick_impl`]/[`midi_rx_impl`] docs.
pub fn input_dispatch_impl<S: SlotApi>(vm: &S, kind: u8, a: u8, b: u8) {
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
#[cfg(feature = "wren-sys-backend")]
pub fn enc_turn(vm: Vm, index: u8, delta: i8) {
    enc_turn_impl(&vm, index, delta);
}

/// Generic body of [`enc_turn`]: see [`tick_impl`]/[`midi_rx_impl`] docs.
pub fn enc_turn_impl<S: SlotApi>(vm: &S, index: u8, delta: i8) {
    ui_call2(vm, ui().on_enc, index as f64, delta as f64);
}

/// Replace a UI callback handle with the Fn in slot 1 (releasing the old).
fn set_ui_cb<S: SlotApi>(vm: &S, which: fn(&mut UiState) -> &mut Handle) {
    let h = vm.get_handle(1);
    let slot = which(ui());
    if !slot.0.is_null() {
        vm.release_handle(*slot);
    }
    *slot = h;
}

pub(crate) fn pads_on_press_impl<S: SlotApi>(vm: &S) {
    set_ui_cb(vm, |u| &mut u.on_pad_press);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn pads_on_press(raw: *mut WrenVM) {
    let vm = Vm(raw);
    pads_on_press_impl(&vm);
}

pub(crate) fn pads_on_release_impl<S: SlotApi>(vm: &S) {
    set_ui_cb(vm, |u| &mut u.on_pad_release);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn pads_on_release(raw: *mut WrenVM) {
    let vm = Vm(raw);
    pads_on_release_impl(&vm);
}

pub(crate) fn buttons_on_press_impl<S: SlotApi>(vm: &S) {
    set_ui_cb(vm, |u| &mut u.on_button_press);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn buttons_on_press(raw: *mut WrenVM) {
    let vm = Vm(raw);
    buttons_on_press_impl(&vm);
}

pub(crate) fn buttons_on_release_impl<S: SlotApi>(vm: &S) {
    set_ui_cb(vm, |u| &mut u.on_button_release);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn buttons_on_release(raw: *mut WrenVM) {
    let vm = Vm(raw);
    buttons_on_release_impl(&vm);
}

pub(crate) fn enc_on_turn_impl<S: SlotApi>(vm: &S) {
    set_ui_cb(vm, |u| &mut u.on_enc);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn enc_on_turn(raw: *mut WrenVM) {
    let vm = Vm(raw);
    enc_on_turn_impl(&vm);
}

// LEDs
pub(crate) fn led_on_impl<S: SlotApi>(vm: &S) {
    host().led(vm.get_f(1) as u8, true);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn led_on(raw: *mut WrenVM) {
    let vm = Vm(raw);
    led_on_impl(&vm);
}

pub(crate) fn led_off_impl<S: SlotApi>(vm: &S) {
    host().led(vm.get_f(1) as u8, false);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn led_off(raw: *mut WrenVM) {
    let vm = Vm(raw);
    led_off_impl(&vm);
}

// OLED
pub(crate) fn oled_clear_impl<S: SlotApi>(_vm: &S) {
    host().oled_clear();
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn oled_clear(raw: *mut WrenVM) {
    let vm = Vm(raw);
    oled_clear_impl(&vm);
}

pub(crate) fn oled_text_impl<S: SlotApi>(vm: &S) {
    let x = vm.get_f(1) as usize;
    let y = vm.get_f(2) as usize;
    let s = vm.get_str(3);
    host().oled_text(x, y, s.as_bytes());
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn oled_text(raw: *mut WrenVM) {
    let vm = Vm(raw);
    oled_text_impl(&vm);
}

pub(crate) fn oled_pixel_impl<S: SlotApi>(vm: &S) {
    let x = vm.get_f(1) as usize;
    let y = vm.get_f(2) as usize;
    let on = vm.get_bool(3);
    host().oled_pixel(x, y, on);
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn oled_pixel(raw: *mut WrenVM) {
    let vm = Vm(raw);
    oled_pixel_impl(&vm);
}

pub(crate) fn oled_show_impl<S: SlotApi>(_vm: &S) {
    host().oled_show();
}
#[cfg(feature = "wren-sys-backend")]
unsafe extern "C" fn oled_show(raw: *mut WrenVM) {
    let vm = Vm(raw);
    oled_show_impl(&vm);
}

// ── Backend-agnostic binding enumeration ─────────────────────────────────────

/// The single source of truth for *which* generic binding bodies exist and how
/// they map to wren `(module, class, is_static, signature)`. Backend-agnostic:
/// it hands each entry to caller-provided registrars, so any [`SlotApi`] backend
/// can wire the same surface into its own foreign registry without re-listing
/// the ~37 signatures. (The `wren-sys` path keeps its own `METHODS`/`CLASSES`
/// tables below because those bind C-ABI `extern "C"` wrappers — a structurally
/// different function type — but they enumerate the same classes/signatures.)
///
/// - `method(module, class, is_static, signature, body)` registers one method.
/// - `class(module, class, allocate)` registers one foreign-class allocator.
pub fn register_foreign<S: SlotApi>(
    mut method: impl FnMut(&'static str, &'static str, bool, &'static str, fn(&S)),
    mut class: impl FnMut(&'static str, &'static str, fn(&S)),
) {
    // Foreign classes (allocators).
    class("main", "Output", output_alloc_impl::<S>);
    class("main", "Gate", gate_alloc_impl::<S>);
    class("main", "Metro", metro_alloc_impl::<S>);

    // Output (CV).
    method("main", "Output", false, "volts", output_volts_get_impl::<S>);
    method("main", "Output", false, "volts=(_)", output_volts_set_impl::<S>);
    method("main", "Output", false, "slew=(_)", output_slew_set_impl::<S>);
    // Gate.
    method("main", "Gate", false, "on=(_)", gate_on_set_impl::<S>);
    // Metro.
    method("main", "Metro", false, "start(_,_)", metro_start_impl::<S>);
    method("main", "Metro", false, "stop()", metro_stop_impl::<S>);
    method("main", "Metro", false, "time=(_)", metro_time_set_impl::<S>);
    // Midi (static).
    method("main", "Midi", true, "noteOn(_,_,_)", midi_note_on_impl::<S>);
    method("main", "Midi", true, "noteOff(_,_,_)", midi_note_off_impl::<S>);
    method("main", "Midi", true, "cc(_,_,_)", midi_cc_impl::<S>);
    method("main", "Midi", true, "send(_,_,_)", midi_send_impl::<S>);
    method("main", "Midi", true, "onNoteOn=(_)", midi_set_on_note_on_impl::<S>);
    method("main", "Midi", true, "onNoteOff=(_)", midi_set_on_note_off_impl::<S>);
    method("main", "Midi", true, "onCC=(_)", midi_set_on_cc_impl::<S>);
    // Pads / Buttons / Enc (static input callbacks).
    method("main", "Pads", true, "onPress=(_)", pads_on_press_impl::<S>);
    method("main", "Pads", true, "onRelease=(_)", pads_on_release_impl::<S>);
    method("main", "Buttons", true, "onPress=(_)", buttons_on_press_impl::<S>);
    method("main", "Buttons", true, "onRelease=(_)", buttons_on_release_impl::<S>);
    method("main", "Enc", true, "onTurn=(_)", enc_on_turn_impl::<S>);
    // Led / Oled (static output).
    method("main", "Led", true, "on(_)", led_on_impl::<S>);
    method("main", "Led", true, "off(_)", led_off_impl::<S>);
    method("main", "Oled", true, "clear()", oled_clear_impl::<S>);
    method("main", "Oled", true, "text(_,_,_)", oled_text_impl::<S>);
    method("main", "Oled", true, "pixel(_,_,_)", oled_pixel_impl::<S>);
    method("main", "Oled", true, "show()", oled_show_impl::<S>);
    // Audio: Node factories (static) + instance methods.
    crate::bindings_audio::register_audio(&mut method);
}

// ── Registry tables ──────────────────────────────────────────────────────────

#[cfg(feature = "wren-sys-backend")]
pub static CLASSES: &[ClassEntry] = &[
    ClassEntry { module: "main", class: "Output", allocate: output_alloc, finalize: None },
    ClassEntry { module: "main", class: "Gate", allocate: gate_alloc, finalize: None },
    ClassEntry { module: "main", class: "Metro", allocate: metro_alloc, finalize: None },
];

#[cfg(feature = "wren-sys-backend")]
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
    static_method("Node", "src_(_,_)", bindings_audio::node_src),
    static_method("Node", "sync_(_,_,_)", bindings_audio::node_sync),
    static_method("Node", "env_(_,_)", bindings_audio::node_env),
    static_method("Node", "noise_()", bindings_audio::node_noise),
    static_method("Node", "pink_()", bindings_audio::node_pink),
    static_method("Node", "brown_()", bindings_audio::node_brown),
    static_method("Node", "binop_(_,_,_)", bindings_audio::node_binop),
    static_method("Node", "lpf_(_,_)", bindings_audio::node_lpf),
    static_method("Node", "svf_(_,_,_,_)", bindings_audio::node_svf),
    static_method("Node", "moog_(_,_,_,_)", bindings_audio::node_moog),
    static_method("Node", "ms20_(_,_,_,_)", bindings_audio::node_ms20),
    static_method("Node", "modal_(_,_,_)", bindings_audio::node_modal),
    method("Node", "drive=(_)", bindings_audio::node_set_drive),
    static_method("Node", "tb303_(_,_,_)", bindings_audio::node_tb303),
    static_method("Node", "patch_(_)", bindings_audio::node_patch),
    static_method("Node", "reset_()", bindings_audio::node_reset),
    static_method("Node", "split_(_)", bindings_audio::node_split),
    static_method("Node", "pan_(_,_)", bindings_audio::node_pan),
    static_method("Node", "wavetable_(_,_)", bindings_audio::node_wavetable),
    static_method("Node", "wavetable_pooled_(_,_)", bindings_audio::node_wavetable_pooled),
    static_method("Node", "delay_(_,_,_)", bindings_audio::node_delay),
    method("Node", "mix=(_)", bindings_audio::node_set_mix),
    method("Node", "damp=(_)", bindings_audio::node_set_damp),
    static_method("Node", "chorus_(_,_,_,_)", bindings_audio::node_chorus),
    static_method("Node", "flanger_(_,_,_,_,_)", bindings_audio::node_flanger),
    static_method("Node", "room_(_,_,_,_)", bindings_audio::node_room),
    static_method("Node", "hall_(_,_,_,_)", bindings_audio::node_hall),
    static_method("Node", "plate_(_,_,_,_)", bindings_audio::node_plate),
    static_method("Node", "drive_(_,_,_,_,_)", bindings_audio::node_drive),
    method("Node", "tone=(_)", bindings_audio::node_set_tone),
    method("Node", "wet=(_)", bindings_audio::node_set_wet),
    static_method("Node", "eq_(_,_,_,_,_)", bindings_audio::node_eq),
    method("Node", "hz=(_)", bindings_audio::node_set_hz),
    method("Node", "gain=(_)", bindings_audio::node_set_gain),
    method("Node", "q=(_)", bindings_audio::node_set_q),
    static_method("Node", "lfo_(_,_)", bindings_audio::node_lfo),
    method("Node", "phase=(_)", bindings_audio::node_set_phase),
    static_method("Node", "sh_(_,_)", bindings_audio::node_sh),
    static_method("Node", "slew_(_,_)", bindings_audio::node_slew),
    static_method("Node", "steps_(_,_)", bindings_audio::node_steps),
    method("Node", "size=(_)", bindings_audio::node_set_size),
    method("Node", "spread=(_)", bindings_audio::node_set_spread),
    method("Node", "rate=(_)", bindings_audio::node_set_rate),
    method("Node", "depth=(_)", bindings_audio::node_set_depth),
    method("Node", "regen=(_)", bindings_audio::node_set_regen),
    method("Node", "freq=(_)", bindings_audio::node_set_freq),
    method("Node", "cutoff=(_)", bindings_audio::node_set_cutoff),
    method("Node", "res=(_)", bindings_audio::node_set_res),
    // Resonator (Kind::Modal) freq/damping = ports 1/2 — well-named aliases of the
    // port-1/port-2 setters (its port 0 is the exciter input, so the inherited `freq=`,
    // which targets port 0, must NOT be used to retune a Resonator).
    method("Node", "pitch=(_)", bindings_audio::node_set_cutoff),
    method("Node", "damping=(_)", bindings_audio::node_set_res),
    method("Node", "pm=(_)", bindings_audio::node_set_pm),
    method("Node", "width=(_)", bindings_audio::node_set_width),
    method("Node", "position=(_)", bindings_audio::node_set_position),
    method("Node", "feedback=(_)", bindings_audio::node_set_feedback),
    method("Node", "structure=(_)", bindings_audio::node_set_structure),
    method("Node", "brightness=(_)", bindings_audio::node_set_brightness),
    method("Node", "strike=(_)", bindings_audio::node_set_strike),
    method("Node", "gate(_)", bindings_audio::node_gate),
    method("Node", "trigger()", bindings_audio::node_trigger),
    method("Node", "out(_)", bindings_audio::node_out),
    method("Node", "free()", bindings_audio::node_free),
    static_method("Bus", "new_()", bindings_audio::bus_new),
    method("Bus", "write(_)", bindings_audio::bus_write),
    static_method("Wavetable", "from(_)", bindings_audio::wavetable_from),
    static_method("Wavetable", "from2d(_)", bindings_audio::wavetable_from2d),
];

/// Terse instance-`MethodEntry` constructor for the `main` module.
#[cfg(feature = "wren-sys-backend")]
const fn method(
    class: &'static str,
    signature: &'static str,
    func: unsafe extern "C" fn(*mut WrenVM),
) -> MethodEntry {
    MethodEntry { module: "main", class, is_static: false, signature, func }
}

/// Terse static-`MethodEntry` constructor for the `main` module.
#[cfg(feature = "wren-sys-backend")]
const fn static_method(
    class: &'static str,
    signature: &'static str,
    func: unsafe extern "C" fn(*mut WrenVM),
) -> MethodEntry {
    MethodEntry { module: "main", class, is_static: true, signature, func }
}

// ── Wren prelude (compiled at boot, before user scripts) ─────────────────────

/// The prelude source (declares the foreign classes + `output[]`/`gate[]`),
/// embedded from `wren/prelude.wren`. Backend-agnostic single source: the
/// `wren-sys` path appends a NUL for the C string API ([`prelude_ptr`]); the
/// `wren-core` path consumes it as a Rust `&str` ([`prelude_str`]).
const PRELUDE_SRC: &str = include_str!("../wren/prelude.wren");

/// The prelude source as a Rust `&str` (no trailing NUL), for backends that
/// compile from a Rust string (e.g. `wren-core`).
pub fn prelude_str() -> &'static str {
    PRELUDE_SRC
}

/// The prelude with a trailing NUL appended at compile time so it can be handed
/// to the C VM as a C string.
#[cfg(feature = "wren-sys-backend")]
const PRELUDE: &str = concat!(include_str!("../wren/prelude.wren"), "\0");

/// The prelude source as a `*const c_char` for `wren_sys::interpret`.
#[cfg(feature = "wren-sys-backend")]
pub fn prelude_ptr() -> *const c_char {
    PRELUDE.as_ptr() as *const c_char
}

/// Tear down all VM-referencing binding state. A host calls this when it frees
/// the VM to run a script fresh (e.g. the web "Run" button): handles are nulled
/// rather than released — the VM that owned them is gone — and CV/gate/metro
/// state plus the audio-graph node-id allocator are reset. Not used on the device
/// (its VM lives for the session).
///
/// This calls [`crate::audio::reset`], which emits `Cmd::Reset` via the registered
/// [`Host`](crate::Host) and therefore panics if no host has been registered
/// yet via [`crate::set_host`] — every current caller registers a host before
/// calling `reset()`.
pub fn reset() {
    *state() = State::EMPTY;

    let m = midi();
    m.on_note_on = Handle(core::ptr::null_mut());
    m.on_note_off = Handle(core::ptr::null_mut());
    m.on_cc = Handle(core::ptr::null_mut());
    m.call3 = Handle(core::ptr::null_mut());

    let u = ui();
    u.on_pad_press = Handle(core::ptr::null_mut());
    u.on_pad_release = Handle(core::ptr::null_mut());
    u.on_button_press = Handle(core::ptr::null_mut());
    u.on_button_release = Handle(core::ptr::null_mut());
    u.on_enc = Handle(core::ptr::null_mut());
    u.call1 = Handle(core::ptr::null_mut());
    u.call2 = Handle(core::ptr::null_mut());

    // SAFETY: VM thread is the sole accessor.
    unsafe { CALL_HANDLE = Handle(core::ptr::null_mut()) };

    crate::audio::reset();
}
