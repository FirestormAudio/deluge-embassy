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

use deluge_audio_graph::{Engine, StereoFrame};

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

extern crate std;
use std::sync::Mutex;
use std::vec::Vec;

/// Serializes [`run_and_capture_cmds`] (and, below, [`run_and_render`]) calls:
/// `cargo test` runs the `#[test]` fns in this file's binary on separate
/// threads by default, and they all share the single [`CAP_HOST`] /
/// [`ENGINE_HOST`] process-globals (see `bindings.rs`'s concurrency-model
/// docs — this state is documented single-threaded-only). One lock for both
/// since they also both touch the shared `crate::set_host`/VM-boot/`reset`
/// process-globals, not just their own host static.
static CAP_LOCK: Mutex<()> = Mutex::new(());

/// A host that records every audio command for assertions.
pub struct CmdCaptureHost {
    pub cmds: Vec<crate::Cmd>,
}
impl CmdCaptureHost {
    pub const fn new() -> Self {
        CmdCaptureHost { cmds: Vec::new() }
    }
}
impl Host for CmdCaptureHost {
    fn now_ms(&mut self) -> u64 {
        0
    }
    fn cv_set(&mut self, _ch: u8, _v: f32) {}
    fn gate_set(&mut self, _ch: u8, _on: bool) {}
    fn midi_tx(&mut self, _m: &[u8]) {}
    fn led(&mut self, _id: u8, _on: bool) {}
    fn oled_clear(&mut self) {}
    fn oled_text(&mut self, _x: usize, _y: usize, _t: &[u8]) {}
    fn oled_pixel(&mut self, _x: usize, _y: usize, _on: bool) {}
    fn oled_show(&mut self) {}
    fn audio_cmd(&mut self, cmd: crate::Cmd) {
        self.cmds.push(cmd);
    }
}

static mut CAP_HOST: CmdCaptureHost = CmdCaptureHost::new();

/// Boot a VM, run `src`, and return the audio commands it emitted.
pub fn run_and_capture_cmds(src: &str) -> Vec<crate::Cmd> {
    // Serialize: see `CAP_LOCK` docs. Held for the whole call (not just the
    // `CAP_HOST` touches) since `set_host`/the VM/`crate::reset()` all touch
    // other shared process-globals too.
    let _guard = CAP_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // SAFETY: single-threaded test helper (serialized by `_guard` above); VM
    // freed before return.
    unsafe {
        let h = &mut *core::ptr::addr_of_mut!(CAP_HOST);
        h.cmds.clear();
        crate::set_host(&mut *core::ptr::addr_of_mut!(CAP_HOST));
        let vm = wren_sys::boot_with_foreign(crate::METHODS, crate::CLASSES);
        assert!(!vm.is_null(), "VM boot failed");
        let r = wren_sys::interpret(vm, c"main".as_ptr(), crate::prelude_ptr());
        assert_eq!(r, wren_sys::WREN_RESULT_SUCCESS, "prelude failed");
        let mut buf = [0u8; 8192];
        let n = src.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&src.as_bytes()[..n]);
        buf[n] = 0;
        let r = wren_sys::interpret(vm, c"main".as_ptr(), buf.as_ptr() as *const core::ffi::c_char);
        assert_eq!(
            r,
            wren_sys::WREN_RESULT_SUCCESS,
            "script failed (line {})",
            LAST_ERR_LINE.load(Ordering::Relaxed)
        );
        let out = (*core::ptr::addr_of_mut!(CAP_HOST)).cmds.clone();
        wren_sys::wrenFreeVM(vm);
        crate::reset();
        out
    }
}

/// Interpret `setup`, feed one DIN-MIDI message, capture the `Cmd`s it emits.
///
/// Mirrors [`run_and_capture_cmds`], but after `setup` interprets it clears
/// the captured (build-time) `Cmd`s, fires one MIDI event via
/// [`crate::midi_rx_impl`] (the same backend-agnostic body the `wren-sys`
/// extern wrapper calls — see `bindings.rs`'s `midi_rx`), and returns only the
/// `Cmd`s that event emits.
pub fn run_midi_capture_cmds(setup: &str, status: u8, d1: u8, d2: u8) -> Vec<crate::Cmd> {
    let _guard = CAP_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // SAFETY: serialized single-threaded test helper; VM freed before return.
    unsafe {
        let h = &mut *core::ptr::addr_of_mut!(CAP_HOST);
        h.cmds.clear();
        crate::set_host(&mut *core::ptr::addr_of_mut!(CAP_HOST));
        let vm = wren_sys::boot_with_foreign(crate::METHODS, crate::CLASSES);
        assert!(!vm.is_null(), "VM boot failed");
        assert_eq!(
            wren_sys::interpret(vm, c"main".as_ptr(), crate::prelude_ptr()),
            wren_sys::WREN_RESULT_SUCCESS,
            "prelude failed"
        );
        let mut buf = [0u8; 8192];
        let n = setup.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&setup.as_bytes()[..n]);
        buf[n] = 0;
        assert_eq!(
            wren_sys::interpret(vm, c"main".as_ptr(), buf.as_ptr() as *const core::ffi::c_char),
            wren_sys::WREN_RESULT_SUCCESS,
            "setup failed"
        );
        (*core::ptr::addr_of_mut!(CAP_HOST)).cmds.clear(); // ignore build-time cmds; capture only the MIDI event's
        // `midi_rx_impl` fires the bound `Midi.onNoteOn`/`onNoteOff` closure
        // with (ch, note, vel). Construct the `Vm` wrapper the same way the
        // `wren-sys-backend` extern wrappers (and `run_and_read_cv`'s `tick`
        // call above) do.
        crate::midi_rx_impl(&wren_sys::Vm(vm), status, d1, d2);
        let out = (*core::ptr::addr_of_mut!(CAP_HOST)).cmds.clone();
        wren_sys::wrenFreeVM(vm);
        crate::reset();
        out
    }
}

/// Interpret `src`; return true iff it ran without a compile/runtime error.
pub fn run_script_ok(src: &str) -> bool {
    let _guard = CAP_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // SAFETY: serialized single-threaded test helper; VM freed before return.
    unsafe {
        crate::set_host(&mut *core::ptr::addr_of_mut!(CAP_HOST));
        let vm = wren_sys::boot_with_foreign(crate::METHODS, crate::CLASSES);
        assert!(!vm.is_null(), "VM boot failed");
        let r0 = wren_sys::interpret(vm, c"main".as_ptr(), crate::prelude_ptr());
        assert_eq!(r0, wren_sys::WREN_RESULT_SUCCESS, "prelude failed");
        let mut buf = [0u8; 8192];
        let n = src.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&src.as_bytes()[..n]);
        buf[n] = 0;
        let r = wren_sys::interpret(vm, c"main".as_ptr(), buf.as_ptr() as *const core::ffi::c_char);
        wren_sys::wrenFreeVM(vm);
        crate::reset();
        r == wren_sys::WREN_RESULT_SUCCESS
    }
}

/// Block size / node / output-port / bus capacities for [`run_and_render`]'s
/// engine — generous enough for the small golden scripts this helper runs.
type TestEng = Engine<32, 64, 128, 8, 90112, 2048>;

/// A host that applies every audio command to a real [`deluge_audio_graph::Engine`],
/// so a script's rendered audio (not just its emitted `Cmd`s) can be asserted on.
pub struct EngineHost {
    pub eng: TestEng,
}
impl EngineHost {
    /// Construct a fresh host with its own engine, sampling at `sample_rate`.
    pub fn new(sample_rate: f32) -> Self {
        EngineHost { eng: TestEng::new(sample_rate) }
    }

    /// Borrow the underlying engine (e.g. to read back pooled memory a test
    /// asserts on — see `engine_host_upload_table_builds_band_limited`).
    pub fn engine(&self) -> &TestEng {
        &self.eng
    }
}
impl Host for EngineHost {
    fn now_ms(&mut self) -> u64 {
        0
    }
    fn cv_set(&mut self, _ch: u8, _v: f32) {}
    fn gate_set(&mut self, _ch: u8, _on: bool) {}
    fn midi_tx(&mut self, _m: &[u8]) {}
    fn led(&mut self, _id: u8, _on: bool) {}
    fn oled_clear(&mut self) {}
    fn oled_text(&mut self, _x: usize, _y: usize, _t: &[u8]) {}
    fn oled_pixel(&mut self, _x: usize, _y: usize, _on: bool) {}
    fn oled_show(&mut self) {}
    fn audio_cmd(&mut self, cmd: crate::Cmd) {
        self.eng.apply(cmd);
    }
    fn upload_table(&mut self, base: &[f32]) -> Option<deluge_audio_graph::PoolHandle> {
        let h = self.eng.pool_alloc(crate::PYRAMID_LEN)?;
        crate::host::build_pyramid_into(base, self.eng.pool_slice_mut(h));
        Some(h)
    }
    fn upload_table_2d(
        &mut self,
        nframes: usize,
        fill_frame: &mut dyn FnMut(usize, &mut [f32]),
    ) -> Option<deluge_audio_graph::PoolHandle> {
        if nframes == 0 {
            return None;
        }
        let h = self.eng.pool_alloc(nframes.checked_mul(crate::PYRAMID_LEN)?)?;
        for f in 0..nframes {
            let mut base = [0.0f32; crate::BASE_LEN];
            fill_frame(f, &mut base);
            let start = f * crate::PYRAMID_LEN;
            let region = &mut self.eng.pool_slice_mut(h)[start..start + crate::PYRAMID_LEN];
            crate::host::build_pyramid_into(&base, region);
        }
        Some(h)
    }
    fn alloc_buffer(&mut self, len: usize) -> Option<deluge_audio_graph::PoolHandle> {
        let h = self.eng.pool_alloc(len)?;
        self.eng.pool_slice_mut(h).fill(0.0); // ring buffers must start clean
        Some(h)
    }
}

static mut ENGINE_HOST: Option<EngineHost> = None;

/// Boot a VM, run `src`, then render one 32-frame block into `out`.
///
/// Serialized by [`CAP_LOCK`] (shared with [`run_and_capture_cmds`]): both
/// touch the same `crate::set_host`/VM-boot/`reset` process-globals, plus
/// this fn's own [`ENGINE_HOST`] static.
pub fn run_and_render(src: &str, out: &mut [StereoFrame; 32]) {
    let _guard = CAP_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // SAFETY: single-threaded test helper, serialized by `_guard` above; VM
    // freed before return.
    unsafe {
        // Each `addr_of_mut!(ENGINE_HOST)` reborrow below is kept to a single
        // self-contained expression (not stashed in a `let` that spans the
        // whole function) so the borrow checker can infer the `'static`
        // lifetime `crate::set_host` requires without the borrows
        // overlapping — same shape as `run_and_capture_cmds` above.
        *core::ptr::addr_of_mut!(ENGINE_HOST) = Some(EngineHost { eng: TestEng::new(44_100.0) });
        crate::set_host((*core::ptr::addr_of_mut!(ENGINE_HOST)).as_mut().unwrap());
        let vm = wren_sys::boot_with_foreign(crate::METHODS, crate::CLASSES);
        assert!(!vm.is_null(), "VM boot failed");
        assert_eq!(
            wren_sys::interpret(vm, c"main".as_ptr(), crate::prelude_ptr()),
            wren_sys::WREN_RESULT_SUCCESS,
            "prelude failed"
        );
        let mut buf = [0u8; 8192];
        let n = src.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&src.as_bytes()[..n]);
        buf[n] = 0;
        assert_eq!(
            wren_sys::interpret(vm, c"main".as_ptr(), buf.as_ptr() as *const core::ffi::c_char),
            wren_sys::WREN_RESULT_SUCCESS,
            "script failed (line {})",
            LAST_ERR_LINE.load(Ordering::Relaxed)
        );
        (*core::ptr::addr_of_mut!(ENGINE_HOST)).as_mut().unwrap().eng.render(out);
        wren_sys::wrenFreeVM(vm);
        crate::reset();
    }
}
