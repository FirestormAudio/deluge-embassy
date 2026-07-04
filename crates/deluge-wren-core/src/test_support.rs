//! Golden-test scaffolding: boot the wren-sys VM the same way a real host does
//! (mirrors `tools/wren-web/src/lib.rs`'s `sim_boot`/`sim_load`), run a script,
//! and read back CV state. Used by `tests/golden_sim.rs` to pin behavior across
//! the [`crate::SlotApi`] refactor; reused by later phases for the same purpose
//! against the second VM backend.
//!
//! Single-threaded, like the rest of the binding state (see `bindings.rs`'s
//! module docs) — do not call concurrently from multiple test threads.

use core::ffi::{c_char, c_int};
use core::sync::atomic::{AtomicI32, Ordering};

use crate::{CV_CHANNELS, Cmd, GATE_CHANNELS, Host};

// `wren-sys` links against these `wren_host_*` C hooks unconditionally (they're
// referenced by its write/error/load-module trampolines); every target that
// links `wren-sys` must define them (the firmware and `wren-web` each provide
// their own). This test helper is its own tiny "host" for that purpose.

/// Last VM error line (-1 = none / not applicable) and message text, for a
/// nicer panic message than a bare `WREN_RESULT_RUNTIME_ERROR` code.
static LAST_ERR_LINE: AtomicI32 = AtomicI32::new(-1);
static mut LAST_ERR_MSG: [u8; 256] = [0; 256];

#[unsafe(no_mangle)]
extern "C" fn wren_host_write(_text: *const c_char) {
    // Discarded: this helper isn't a REPL, just a golden-test fixture.
}

#[unsafe(no_mangle)]
extern "C" fn wren_host_error(_module: *const c_char, line: c_int, message: *const c_char) {
    LAST_ERR_LINE.store(line, Ordering::Relaxed);
    // SAFETY: single-threaded test helper (see module docs); `message` is a
    // valid NUL-terminated C string for the duration of this call.
    unsafe {
        let bytes = core::ffi::CStr::from_ptr(message).to_bytes();
        let buf = &mut *core::ptr::addr_of_mut!(LAST_ERR_MSG);
        let n = bytes.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&bytes[..n]);
        buf[n] = 0;
    }
}

/// The message from the most recent `wren_host_error` call, for panic text.
fn last_err_msg() -> &'static str {
    unsafe {
        core::ffi::CStr::from_ptr(core::ptr::addr_of!(LAST_ERR_MSG) as *const c_char)
            .to_str()
            .unwrap_or("<non-utf8>")
    }
}

#[unsafe(no_mangle)]
extern "C" fn wren_host_load_module(_name: *const c_char) -> *const c_char {
    // No multi-file `import` support needed for golden scripts.
    core::ptr::null()
}

struct TestHost {
    now_ms: u64,
    cv: [f32; CV_CHANNELS],
    gate: [bool; GATE_CHANNELS],
}
impl TestHost {
    const fn new() -> Self {
        TestHost { now_ms: 0, cv: [0.0; CV_CHANNELS], gate: [false; GATE_CHANNELS] }
    }
}
impl Host for TestHost {
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

static mut HOST: TestHost = TestHost::new();

/// Boot a fresh VM, run the prelude then `src` in the `main` module, tick once
/// (to flush CV slew into the host), read CV channel `ch` (0-based), then tear
/// the VM down. Panics on boot/compile/runtime failure — this is a test helper,
/// not production code.
pub fn run_and_read_cv(src: &str, ch: u8) -> f32 {
    // SAFETY: single-threaded test helper; see module docs. The host is
    // (re)registered before every run, and the VM is freed before returning.
    unsafe {
        let host = &mut *core::ptr::addr_of_mut!(HOST);
        host.now_ms = 0;
        host.cv = [0.0; CV_CHANNELS];
        host.gate = [false; GATE_CHANNELS];
        crate::set_host(&mut *core::ptr::addr_of_mut!(HOST));

        let vm = wren_sys::boot_with_foreign(crate::METHODS, crate::CLASSES);
        assert!(!vm.is_null(), "test_support: VM boot failed");

        let r = wren_sys::interpret(vm, c"main".as_ptr(), crate::prelude_ptr());
        assert_eq!(
            r,
            wren_sys::WREN_RESULT_SUCCESS,
            "test_support: prelude failed to compile (line {}): {}",
            LAST_ERR_LINE.load(Ordering::Relaxed),
            last_err_msg()
        );

        // NUL-terminate the script source for the C VM.
        let mut buf = [0u8; 8192];
        let n = src.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&src.as_bytes()[..n]);
        buf[n] = 0;
        let r = wren_sys::interpret(vm, c"main".as_ptr(), buf.as_ptr() as *const c_char);
        assert_eq!(
            r,
            wren_sys::WREN_RESULT_SUCCESS,
            "test_support: script failed (line {}): {}",
            LAST_ERR_LINE.load(Ordering::Relaxed),
            last_err_msg()
        );

        // Flush slew into the host (matches a real host's tick loop).
        crate::tick(wren_sys::Vm(vm), 0, 1.0);

        let cv = (*core::ptr::addr_of_mut!(HOST)).cv[ch as usize];

        wren_sys::wrenFreeVM(vm);
        crate::reset();
        cv
    }
}

/// Boot a fresh VM, run the prelude then `src` in the `main` module, call
/// [`crate::tick`] `ticks` times (each `ms_per_tick` milliseconds apart,
/// so registered `Metro` callbacks fire on schedule), read CV channel `ch`
/// (0-based), then tear the VM down. Panics on boot/compile/runtime failure —
/// this is a test helper, not production code.
pub fn run_tick_read_cv(src: &str, ms_per_tick: u64, ticks: u32, ch: u8) -> f32 {
    // SAFETY: single-threaded test helper; see module docs. The host is
    // (re)registered before every run, and the VM is freed before returning.
    unsafe {
        let host = &mut *core::ptr::addr_of_mut!(HOST);
        host.now_ms = 0;
        host.cv = [0.0; CV_CHANNELS];
        host.gate = [false; GATE_CHANNELS];
        crate::set_host(&mut *core::ptr::addr_of_mut!(HOST));

        let vm = wren_sys::boot_with_foreign(crate::METHODS, crate::CLASSES);
        assert!(!vm.is_null(), "test_support: VM boot failed");

        let r = wren_sys::interpret(vm, c"main".as_ptr(), crate::prelude_ptr());
        assert_eq!(
            r,
            wren_sys::WREN_RESULT_SUCCESS,
            "test_support: prelude failed to compile (line {}): {}",
            LAST_ERR_LINE.load(Ordering::Relaxed),
            last_err_msg()
        );

        // NUL-terminate the script source for the C VM.
        let mut buf = [0u8; 8192];
        let n = src.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&src.as_bytes()[..n]);
        buf[n] = 0;
        let r = wren_sys::interpret(vm, c"main".as_ptr(), buf.as_ptr() as *const c_char);
        assert_eq!(
            r,
            wren_sys::WREN_RESULT_SUCCESS,
            "test_support: script failed (line {}): {}",
            LAST_ERR_LINE.load(Ordering::Relaxed),
            last_err_msg()
        );

        let mut now_ms: u64 = 0;
        for _ in 0..ticks {
            now_ms += ms_per_tick;
            crate::tick(wren_sys::Vm(vm), now_ms, ms_per_tick as f32 / 1000.0);
        }
        // Flush: `tick` renders CV *then* fires due callbacks, so the last
        // tick's callback-driven write only reaches the host on the next
        // render pass. One more tick at the same timestamp (dt=0, so no
        // further callback becomes due) pushes it through before we read it
        // back — same flush idea as `run_and_read_cv`'s post-script tick.
        crate::tick(wren_sys::Vm(vm), now_ms, 0.0);

        let cv = (*core::ptr::addr_of_mut!(HOST)).cv[ch as usize];

        wren_sys::wrenFreeVM(vm);
        crate::reset();
        cv
    }
}
