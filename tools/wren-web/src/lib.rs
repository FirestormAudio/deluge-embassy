//! Wasm core for the Deluge Wren web editor/simulator.
//!
//! Implements [`deluge_wren_core::Host`] against in-memory buffers (the browser
//! counterpart of the firmware's `FwHost`) and exposes a small C-ABI surface for
//! JS: boot the VM, run a script, inject input (pads/buttons/encoders/MIDI),
//! advance time, and drain output (print/errors, OLED pixels, CV/gate/LED state,
//! MIDI TX).
//!
//! The surface is raw `extern "C"` rather than wasm-bindgen because the module
//! also carries the VM's WASI imports (it links wasi-libc), so it needs a custom
//! JS loader regardless; a hand-written TS wrapper over these exports is small and
//! keeps the crate `no_std`. Audio is engine-applied but not yet rendered to Web
//! Audio (that's a later milestone).

#![no_std]

use core::ffi::{c_char, c_int};
use core::ptr::addr_of_mut;

use deluge_wren_core::{CV_CHANNELS, Cmd, Engine, GATE_CHANNELS, Host};
use wren_sys::{Vm, WrenVM};

mod oled;
use oled::Oled;

const N_LED: usize = 64;
const SRC_CAP: usize = 64 * 1024;
const OUT_CAP: usize = 8 * 1024;
const ERR_CAP: usize = 1024;
const MIDI_TX_CAP: usize = 512;

// ── Host state ───────────────────────────────────────────────────────────────

struct WebHost {
    now_ms: u64,
    cv: [f32; CV_CHANNELS],
    gate: [bool; GATE_CHANNELS],
    leds: [u8; N_LED],
    oled: Oled,
    midi_tx: [u8; MIDI_TX_CAP],
    midi_tx_len: usize,
    engine: Engine,
}

impl WebHost {
    const fn new() -> Self {
        WebHost {
            now_ms: 0,
            cv: [0.0; CV_CHANNELS],
            gate: [false; GATE_CHANNELS],
            leds: [0; N_LED],
            oled: Oled::new(),
            midi_tx: [0; MIDI_TX_CAP],
            midi_tx_len: 0,
            engine: Engine::new(),
        }
    }
}

impl Host for WebHost {
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
    fn midi_tx(&mut self, msg: &[u8]) {
        for &b in msg {
            if self.midi_tx_len < MIDI_TX_CAP {
                self.midi_tx[self.midi_tx_len] = b;
                self.midi_tx_len += 1;
            }
        }
    }
    fn led(&mut self, id: u8, on: bool) {
        if (id as usize) < N_LED {
            self.leds[id as usize] = on as u8;
        }
    }
    fn oled_clear(&mut self) {
        self.oled.clear();
    }
    fn oled_text(&mut self, x: usize, y: usize, text: &[u8]) {
        self.oled.draw_str(x, y, text);
    }
    fn oled_pixel(&mut self, x: usize, y: usize, on: bool) {
        self.oled.set_pixel(x, y, on);
    }
    fn oled_show(&mut self) {
        // The pixel buffer is always current; JS reads it each frame.
    }
    fn audio_cmd(&mut self, cmd: Cmd) {
        // Single-threaded: apply straight to the engine (rendering comes later).
        self.engine.apply(cmd);
    }
}

// ── Globals (single-threaded wasm; sole accessor is the JS-driven VM thread) ──

static mut HOST: WebHost = WebHost::new();
static mut VM: *mut WrenVM = core::ptr::null_mut();

static mut SRC: [u8; SRC_CAP + 1] = [0; SRC_CAP + 1];
static mut OUT: [u8; OUT_CAP] = [0; OUT_CAP];
static mut OUT_LEN: usize = 0;
static mut ERR: [u8; ERR_CAP] = [0; ERR_CAP];
static mut ERR_LEN: usize = 0;
static mut ERR_LINE: i32 = -1;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[inline]
fn vm() -> Vm {
    // SAFETY: set once by sim_boot before any input/tick call.
    Vm(unsafe { VM })
}

/// Append a NUL-terminated C string to a fixed byte buffer (best effort).
unsafe fn append_cstr(buf: &mut [u8], len: &mut usize, text: *const c_char) {
    if text.is_null() {
        return;
    }
    let mut i = 0;
    unsafe {
        while *text.add(i) != 0 && *len < buf.len() {
            buf[*len] = *text.add(i) as u8;
            *len += 1;
            i += 1;
        }
    }
}

// ── Host hooks the VM calls back into (provided by every wren-sys consumer) ───

#[unsafe(no_mangle)]
extern "C" fn wren_host_write(text: *const c_char) {
    // SAFETY: VM thread is the sole accessor.
    unsafe { append_cstr(&mut *addr_of_mut!(OUT), &mut *addr_of_mut!(OUT_LEN), text) };
}

#[unsafe(no_mangle)]
extern "C" fn wren_host_error(line: c_int, message: *const c_char) {
    unsafe {
        ERR_LINE = line;
        append_cstr(&mut *addr_of_mut!(ERR), &mut *addr_of_mut!(ERR_LEN), message);
        // Newline so multiple error lines stay readable.
        let e = &mut *addr_of_mut!(ERR);
        let el = &mut *addr_of_mut!(ERR_LEN);
        if *el < e.len() {
            e[*el] = b'\n';
            *el += 1;
        }
    }
}

// ── Exports ──────────────────────────────────────────────────────────────────

/// Boot the VM, register the host, and run the prelude. Returns 1 on success.
#[unsafe(no_mangle)]
pub extern "C" fn sim_boot() -> i32 {
    // SAFETY: single-threaded; called once at startup.
    let host = unsafe { &mut *addr_of_mut!(HOST) };
    deluge_wren_core::set_host(host);

    let vm = unsafe { wren_sys::boot_with_foreign(deluge_wren_core::METHODS, deluge_wren_core::CLASSES) };
    if vm.is_null() {
        return 0;
    }
    unsafe { VM = vm };
    let r = unsafe { wren_sys::interpret(vm, c"main".as_ptr(), deluge_wren_core::prelude_ptr()) };
    (r == wren_sys::WREN_RESULT_SUCCESS) as i32
}

/// Pointer to the source-input buffer: JS writes up to `sim_src_cap()` bytes here,
/// then calls [`sim_load`].
#[unsafe(no_mangle)]
pub extern "C" fn sim_src_ptr() -> *mut u8 {
    addr_of_mut!(SRC) as *mut u8
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_src_cap() -> usize {
    SRC_CAP
}

/// Interpret `SRC[..len]` in the `main` module. Clears print/error capture first.
/// Returns the wren result code (0 = success, 1 = compile error, 2 = runtime).
#[unsafe(no_mangle)]
pub extern "C" fn sim_load(len: usize) -> i32 {
    unsafe {
        OUT_LEN = 0;
        ERR_LEN = 0;
        ERR_LINE = -1;
        let src = &mut *addr_of_mut!(SRC);
        let n = len.min(SRC_CAP);
        src[n] = 0;
        wren_sys::interpret(VM, c"main".as_ptr(), src.as_ptr() as *const c_char) as i32
    }
}

/// Set the simulated clock (milliseconds), read by metro scheduling.
#[unsafe(no_mangle)]
pub extern "C" fn sim_set_now_ms(ms: f64) {
    unsafe { (*addr_of_mut!(HOST)).now_ms = ms as u64 };
}

/// Advance control-rate state: CV slew + fire due metro callbacks. `dt_s` is the
/// elapsed seconds since the previous tick. Call [`sim_set_now_ms`] first.
#[unsafe(no_mangle)]
pub extern "C" fn sim_tick(dt_s: f32) {
    let now = unsafe { (*addr_of_mut!(HOST)).now_ms };
    deluge_wren_core::tick(vm(), now, dt_s);
}

// Input injection — dispatched immediately (we're on the VM thread, not inside a
// foreign method, so re-entering the VM is legal).
#[unsafe(no_mangle)]
pub extern "C" fn sim_pad(x: u8, y: u8, down: i32) {
    deluge_wren_core::input_dispatch(vm(), if down != 0 { 0 } else { 1 }, x, y);
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_button(id: u8, down: i32) {
    deluge_wren_core::input_dispatch(vm(), if down != 0 { 2 } else { 3 }, id, 0);
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_enc(index: u8, delta: i32) {
    deluge_wren_core::enc_turn(vm(), index, delta as i8);
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_midi_in(status: u8, d1: u8, d2: u8) {
    deluge_wren_core::midi_rx(vm(), status, d1, d2);
}

// Output drains.
#[unsafe(no_mangle)]
pub extern "C" fn sim_oled_ptr() -> *const u8 {
    unsafe { (*addr_of_mut!(HOST)).oled.px.as_ptr() }
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_oled_len() -> usize {
    oled::WIDTH * oled::HEIGHT
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_cv(ch: u8) -> f32 {
    let h = unsafe { &*addr_of_mut!(HOST) };
    if (ch as usize) < CV_CHANNELS { h.cv[ch as usize] } else { 0.0 }
}
/// Gate state as a bitmask (bit `i` = gate `i+1`).
#[unsafe(no_mangle)]
pub extern "C" fn sim_gate_bits() -> u32 {
    let h = unsafe { &*addr_of_mut!(HOST) };
    let mut bits = 0u32;
    for (i, &g) in h.gate.iter().enumerate() {
        if g {
            bits |= 1 << i;
        }
    }
    bits
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_led_ptr() -> *const u8 {
    unsafe { (*addr_of_mut!(HOST)).leds.as_ptr() }
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_led_len() -> usize {
    N_LED
}

#[unsafe(no_mangle)]
pub extern "C" fn sim_out_ptr() -> *const u8 {
    addr_of_mut!(OUT) as *const u8
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_out_len() -> usize {
    unsafe { OUT_LEN }
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_out_clear() {
    unsafe { OUT_LEN = 0 };
}

#[unsafe(no_mangle)]
pub extern "C" fn sim_err_ptr() -> *const u8 {
    addr_of_mut!(ERR) as *const u8
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_err_len() -> usize {
    unsafe { ERR_LEN }
}
/// Line number of the last reported error (-1 if none / not applicable).
#[unsafe(no_mangle)]
pub extern "C" fn sim_err_line() -> i32 {
    unsafe { ERR_LINE }
}

#[unsafe(no_mangle)]
pub extern "C" fn sim_midi_tx_ptr() -> *const u8 {
    unsafe { (*addr_of_mut!(HOST)).midi_tx.as_ptr() }
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_midi_tx_len() -> usize {
    unsafe { (*addr_of_mut!(HOST)).midi_tx_len }
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_midi_tx_clear() {
    unsafe { (*addr_of_mut!(HOST)).midi_tx_len = 0 };
}
