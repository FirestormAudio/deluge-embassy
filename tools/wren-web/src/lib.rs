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

mod codec;
mod oled;
use oled::Oled;

const N_LED: usize = 64;
const AUDIO_CAP: usize = 8192;
const SRC_CAP: usize = 64 * 1024;
const OUT_CAP: usize = 8 * 1024;
const ERR_CAP: usize = 1024;
const MIDI_TX_CAP: usize = 512;
// Module registry for multi-file projects: packed names + sources (each
// NUL-terminated) addressed by a small table. Populated before a run; the wren
// `import` host hook resolves names against it (see wren_host_load_module).
const MAX_MODULES: usize = 64;
const MOD_NAME_CAP: usize = 8 * 1024;
const MOD_SRC_CAP: usize = 512 * 1024;
const MOD_NAME_BUF_CAP: usize = 512;
/// Serialized audio-graph command FIFO: main VM → AudioWorklet engine.
const CMD_CAP: usize = 512 * codec::REC;

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
        // Apply to the local engine (used to render the on-screen scope) and also
        // queue the serialized command for the AudioWorklet's render engine.
        self.engine.apply(cmd);
        push_cmd_out(cmd);
    }
}

/// Append a serialized command to the main→worklet FIFO (best effort).
fn push_cmd_out(cmd: Cmd) {
    // SAFETY: VM thread is the sole accessor.
    unsafe {
        let buf = &mut *addr_of_mut!(CMD_OUT);
        let len = &mut *addr_of_mut!(CMD_OUT_LEN);
        if *len + codec::REC <= CMD_CAP {
            buf[*len..*len + codec::REC].copy_from_slice(&codec::encode(cmd));
            *len += codec::REC;
        }
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
/// Module the last error occurred in (`main` for the entry, else an imported
/// module name), so the editor can attribute a compile error to the right file.
const ERR_MODULE_CAP: usize = 256;
static mut ERR_MODULE: [u8; ERR_MODULE_CAP] = [0; ERR_MODULE_CAP];
static mut ERR_MODULE_LEN: usize = 0;
static mut AUDIO: [f32; AUDIO_CAP] = [0.0; AUDIO_CAP];

// main→worklet audio-graph command FIFO (serialized).
static mut CMD_OUT: [u8; CMD_CAP] = [0; CMD_CAP];
static mut CMD_OUT_LEN: usize = 0;

// The AudioWorklet instance's render engine + its incoming-command buffer. These
// statics are exercised only by the worklet copy of this module (a second wasm
// instance with its own memory); the main copy never touches them.
static mut WORKLET_ENGINE: Engine = Engine::new();
static mut ECMD: [u8; CMD_CAP] = [0; CMD_CAP];

// Module registry (see MAX_MODULES). MOD_NAME_BUF is a scratch input for one
// module name; sources arrive via the shared SRC buffer.
static mut MOD_NAME_BUF: [u8; MOD_NAME_BUF_CAP] = [0; MOD_NAME_BUF_CAP];
static mut MOD_NAMES: [u8; MOD_NAME_CAP] = [0; MOD_NAME_CAP];
static mut MOD_SRC: [u8; MOD_SRC_CAP] = [0; MOD_SRC_CAP];
static mut MOD_TABLE: [(u32, u32); MAX_MODULES] = [(0, 0); MAX_MODULES]; // (name_off, src_off)
static mut MOD_COUNT: usize = 0;
static mut MOD_NAMES_POS: usize = 0;
static mut MOD_SRC_POS: usize = 0;

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
extern "C" fn wren_host_error(module: *const c_char, line: c_int, message: *const c_char) {
    unsafe {
        // Capture the FIRST error's location + module (the root cause). A failed
        // import cascades a second error in the *importing* module, and runtime
        // stack traces list outer frames after the inner one — neither should
        // overwrite where the real error is, so the editor attributes it right.
        if ERR_LINE == -1 && line >= 1 {
            ERR_LINE = line;
            if !module.is_null() {
                let dst = &mut *addr_of_mut!(ERR_MODULE);
                let mut i = 0usize;
                while i < ERR_MODULE_CAP {
                    let b = *module.add(i);
                    if b == 0 {
                        break;
                    }
                    dst[i] = b as u8;
                    i += 1;
                }
                ERR_MODULE_LEN = i;
            }
        }
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

/// Resolve an `import "name"` against the module registry (multi-file projects).
/// Returns the imported file's NUL-terminated source, or NULL if not registered.
#[unsafe(no_mangle)]
extern "C" fn wren_host_load_module(name: *const c_char) -> *const c_char {
    if name.is_null() {
        return core::ptr::null();
    }
    // SAFETY: VM thread is the sole accessor; registry buffers outlive the run.
    unsafe {
        let names = &*addr_of_mut!(MOD_NAMES);
        let srcs = &*addr_of_mut!(MOD_SRC);
        let table = &*addr_of_mut!(MOD_TABLE);
        for &(noff, soff) in &table[..MOD_COUNT] {
            if cstr_eq(name, names.as_ptr().add(noff as usize) as *const c_char) {
                return srcs.as_ptr().add(soff as usize) as *const c_char;
            }
        }
    }
    core::ptr::null()
}

/// Compare two NUL-terminated C strings for equality.
unsafe fn cstr_eq(a: *const c_char, b: *const c_char) -> bool {
    let mut i = 0;
    loop {
        let (ca, cb) = unsafe { (*a.add(i), *b.add(i)) };
        if ca != cb {
            return false;
        }
        if ca == 0 {
            return true;
        }
        i += 1;
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

/// Rebuild the VM and clear all simulated state, so the next [`sim_load`] runs a
/// script fresh (no leftover module vars, metros, patches, OLED, or CV). Returns
/// 1 on success. The web "Run" calls this so each run starts from a clean
/// instrument, unlike the device's persistent REPL.
#[unsafe(no_mangle)]
pub extern "C" fn sim_reset() -> i32 {
    unsafe {
        if !VM.is_null() {
            wren_sys::wrenFreeVM(VM);
            VM = core::ptr::null_mut();
        }
        // Drop all VM-referencing binding handles before the fresh boot.
        deluge_wren_core::reset();
        *addr_of_mut!(HOST) = WebHost::new();
        // Tell the worklet engine to clear its graph too (it's a separate
        // instance that won't see the host reset otherwise).
        CMD_OUT_LEN = 0;
    }
    push_cmd_out(Cmd::Reset);
    sim_boot()
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

// ── Module registry (multi-file projects) ────────────────────────────────────
// Before running, JS clears then registers each project file: write the module
// name into the name buffer + the source into the SRC buffer, then sim_add_module.

/// Empty the module registry (call before re-registering a project's files).
#[unsafe(no_mangle)]
pub extern "C" fn sim_clear_modules() {
    unsafe {
        MOD_COUNT = 0;
        MOD_NAMES_POS = 0;
        MOD_SRC_POS = 0;
    }
}
/// Name-input buffer for [`sim_add_module`] (one module name at a time).
#[unsafe(no_mangle)]
pub extern "C" fn sim_mod_name_ptr() -> *mut u8 {
    addr_of_mut!(MOD_NAME_BUF) as *mut u8
}
/// Register a module: name = `MOD_NAME_BUF[..name_len]`, source = `SRC[..src_len]`.
/// Both are stored NUL-terminated. Returns 1 on success, 0 if the registry is full.
#[unsafe(no_mangle)]
pub extern "C" fn sim_add_module(name_len: usize, src_len: usize) -> i32 {
    unsafe {
        let name_len = name_len.min(MOD_NAME_BUF_CAP);
        let src_len = src_len.min(SRC_CAP);
        if MOD_COUNT >= MAX_MODULES
            || MOD_NAMES_POS + name_len + 1 > MOD_NAME_CAP
            || MOD_SRC_POS + src_len + 1 > MOD_SRC_CAP
        {
            return 0;
        }
        let nbuf = &*addr_of_mut!(MOD_NAME_BUF);
        let sbuf = &*addr_of_mut!(SRC);
        let names = &mut *addr_of_mut!(MOD_NAMES);
        let srcs = &mut *addr_of_mut!(MOD_SRC);

        let noff = MOD_NAMES_POS;
        names[noff..noff + name_len].copy_from_slice(&nbuf[..name_len]);
        names[noff + name_len] = 0;
        MOD_NAMES_POS += name_len + 1;

        let soff = MOD_SRC_POS;
        srcs[soff..soff + src_len].copy_from_slice(&sbuf[..src_len]);
        srcs[soff + src_len] = 0;
        MOD_SRC_POS += src_len + 1;

        (*addr_of_mut!(MOD_TABLE))[MOD_COUNT] = (noff as u32, soff as u32);
        MOD_COUNT += 1;
    }
    1
}

/// Interpret `SRC[..len]` in the `main` module. Clears print/error capture first.
/// Returns the wren result code (0 = success, 1 = compile error, 2 = runtime).
#[unsafe(no_mangle)]
pub extern "C" fn sim_load(len: usize) -> i32 {
    unsafe {
        OUT_LEN = 0;
        ERR_LEN = 0;
        ERR_LINE = -1;
        ERR_MODULE_LEN = 0;
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
/// Name of the module the last error occurred in (`main` for the entry, else an
/// imported module). Read `[sim_err_module_ptr()..+sim_err_module_len()]`.
#[unsafe(no_mangle)]
pub extern "C" fn sim_err_module_ptr() -> *const u8 {
    addr_of_mut!(ERR_MODULE) as *const u8
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_err_module_len() -> usize {
    unsafe { ERR_MODULE_LEN }
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

// ── Audio render ─────────────────────────────────────────────────────────────
// The DSP graph renders on demand: `sim_render(n)` advances the engine `n` mono
// samples (at 44.1 kHz) into the audio buffer, which JS copies into a Web Audio
// block. The host's AudioContext must run at 44.1 kHz to match the engine.

#[unsafe(no_mangle)]
pub extern "C" fn sim_audio_ptr() -> *const f32 {
    addr_of_mut!(AUDIO) as *const f32
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_audio_cap() -> usize {
    AUDIO_CAP
}
/// Render `n` (clamped to `sim_audio_cap`) mono samples from the *main* engine
/// into the audio buffer; returns the count. Used to draw the on-screen scope —
/// actual audio output is rendered off the main thread (see the worklet engine
/// API below).
#[unsafe(no_mangle)]
pub extern "C" fn sim_render(n: usize) -> usize {
    let host = unsafe { &mut *addr_of_mut!(HOST) };
    let buf = unsafe { &mut *addr_of_mut!(AUDIO) };
    let n = n.min(AUDIO_CAP);
    for s in buf.iter_mut().take(n) {
        *s = host.engine.render_frame().clamp(-1.0, 1.0);
    }
    n
}

// ── main → worklet: audio-graph command drain ────────────────────────────────
// Each animation frame the main thread drains these serialized commands and
// posts them to the AudioWorklet, whose engine applies + renders them.

#[unsafe(no_mangle)]
pub extern "C" fn sim_audio_cmds_ptr() -> *const u8 {
    addr_of_mut!(CMD_OUT) as *const u8
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_audio_cmds_len() -> usize {
    unsafe { CMD_OUT_LEN }
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_audio_cmds_clear() {
    unsafe { CMD_OUT_LEN = 0 };
}

// ── worklet engine: render off the main thread ───────────────────────────────
// These run in the AudioWorklet's *own* wasm instance. It writes forwarded
// command bytes at `sim_engine_cmd_ptr`, calls `sim_engine_apply`, then
// `sim_engine_render` each audio block and reads the result from `sim_audio_ptr`.

#[unsafe(no_mangle)]
pub extern "C" fn sim_engine_cmd_ptr() -> *mut u8 {
    addr_of_mut!(ECMD) as *mut u8
}
#[unsafe(no_mangle)]
pub extern "C" fn sim_engine_cmd_cap() -> usize {
    CMD_CAP
}
/// Apply the `len` bytes of serialized commands now in the engine command buffer.
#[unsafe(no_mangle)]
pub extern "C" fn sim_engine_apply(len: usize) {
    let buf = unsafe { &*addr_of_mut!(ECMD) };
    let eng = unsafe { &mut *addr_of_mut!(WORKLET_ENGINE) };
    let len = len.min(CMD_CAP);
    let mut off = 0;
    while off + codec::REC <= len {
        eng.apply(codec::decode(&buf[off..off + codec::REC]));
        off += codec::REC;
    }
}
/// Render `n` mono samples from the worklet engine into the audio buffer.
#[unsafe(no_mangle)]
pub extern "C" fn sim_engine_render(n: usize) -> usize {
    let eng = unsafe { &mut *addr_of_mut!(WORKLET_ENGINE) };
    let buf = unsafe { &mut *addr_of_mut!(AUDIO) };
    let n = n.min(AUDIO_CAP);
    for s in buf.iter_mut().take(n) {
        *s = eng.render_frame().clamp(-1.0, 1.0);
    }
    n
}
