//! Deluge Wren debugger core.
//!
//! Runs the *same* deluge foreign bindings (from `deluge-wren-core`, the single
//! source of truth for the scripting surface) under **wren-core** — a wren VM
//! whose compiler is written in Rust and emits bytecode for the same upstream C
//! VM. Using wren-core (instead of the stock `wren-sys` path) gives us named
//! locals and source spans for free, which the source-level debugger needs.
//!
//! This crate depends on `deluge-wren-core` with `default-features = false` so
//! it links ONLY wren-core's copy of the C VM — never `wren-sys`'s — since two
//! upstream C VMs in one binary would fail to link.

mod register;
mod slotapi_wrencore;

use deluge_wren_core::{Cmd, Host, set_host};
use wren_core::vm::CWrenVm;

/// A no-op [`Host`]: the deluge binding bodies reach the outside world (audio
/// graph, CV/gate jacks, MIDI, LEDs, OLED) through this trait, so one must be
/// registered before any script runs or the bodies panic. The debugger cares
/// about control flow and locals, not hardware effects, so every method is a
/// sink. Later phases can swap in a host that records effects for the UI.
struct NoopHost;

impl Host for NoopHost {
    fn now_ms(&mut self) -> u64 {
        0
    }
    fn cv_set(&mut self, _ch: u8, _volts: f32) {}
    fn gate_set(&mut self, _ch: u8, _on: bool) {}
    fn midi_tx(&mut self, _msg: &[u8]) {}
    fn led(&mut self, _id: u8, _on: bool) {}
    fn oled_clear(&mut self) {}
    fn oled_text(&mut self, _x: usize, _y: usize, _text: &[u8]) {}
    fn oled_pixel(&mut self, _x: usize, _y: usize, _on: bool) {}
    fn oled_show(&mut self) {}
    fn audio_cmd(&mut self, _cmd: Cmd) {}
}

/// Register a fresh no-op host, and clear any binding state from a previous VM.
fn install_noop_host() {
    // `set_host` needs a `&'static mut`; leaking one small ZST-ish host per VM
    // is acceptable for a debug tool. The global is single-threaded (VM thread).
    let host: &'static mut NoopHost = Box::leak(Box::new(NoopHost));
    set_host(host);
    // Drop any handles / CV / audio state a previous VM left behind.
    deluge_wren_core::reset();
}

/// Build a wren-core VM with every deluge foreign binding registered and the
/// deluge prelude compiled into the `main` module.
///
/// `write_fn` receives each chunk of `System.print` output.
pub fn build_vm(write_fn: impl Fn(&str) + 'static) -> CWrenVm {
    install_noop_host();

    let mut mreg = wren_core::foreign::ForeignMethodRegistry::new();
    let mut creg = wren_core::foreign::ForeignClassRegistry::new();
    register::register_all(&mut mreg, &mut creg);

    let mut vm = CWrenVm::with_foreign(write_fn, None, None, mreg, creg);
    vm.interpret("main", deluge_wren_core::prelude_str())
        .expect("deluge prelude failed to compile/run under wren-core");
    vm
}
