//! The Wren audio foreign classes (`Node`/`Out`), retargeted onto
//! `deluge-audio-graph`. Factory statics allocate an id and emit `NewNode`;
//! instance methods mutate via the `crate::audio` emitters. `Port` and `Bus`
//! are added in later P1 tasks.

use deluge_audio_graph::{Input, Kind, NodeId};

#[cfg(feature = "wren-sys-backend")]
use wren_sys::{Vm, WrenVM};

use crate::audio;
use crate::slotapi::{SlotApi, WrenForeign, WrenType};

// All audio foreign objects lead with a `tag: u8` (offset 0 under `repr(C)`) so
// `arg_input` can discriminate a Node/Port/Bus argument by reading that byte —
// no VM class query, no `SlotApi` change. `Port`/`Bus` land in later tasks.
pub(crate) const TAG_NODE: u8 = 0;
pub(crate) const TAG_PORT: u8 = 1;
pub(crate) const TAG_BUS: u8 = 2;

#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct NodeObj {
    pub tag: u8,
    pub id: u16,
}
impl WrenForeign for NodeObj {
    fn module_name() -> &'static str {
        "main"
    }
    fn class_name() -> &'static str {
        "Node"
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct PortObj {
    pub tag: u8,
    pub port: u8,
    pub node: u16,
}
impl WrenForeign for PortObj {
    fn module_name() -> &'static str {
        "main"
    }
    fn class_name() -> &'static str {
        "Port"
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct BusObj {
    pub tag: u8,
    pub id: u16,
}
impl WrenForeign for BusObj {
    fn module_name() -> &'static str {
        "main"
    }
    fn class_name() -> &'static str {
        "Bus"
    }
}

/// Map a prelude waveform/op code to a `Kind`. Codes match the prelude
/// (`Osc.sine=0 saw=1 square=2 tri=3`; binop `mul=0 add=1 sub=2`).
fn src_kind(code: u8) -> Kind {
    match code {
        0 => Kind::Sine,
        1 => Kind::Saw,
        2 => Kind::Square,
        _ => Kind::Tri,
    }
}
fn binop_kind(code: u8) -> Kind {
    match code {
        0 => Kind::Mul,
        1 => Kind::Add,
        _ => Kind::Sub,
    }
}

/// Resolve a number / Node / Port / Bus argument at `slot` into an engine
/// `Input`, discriminating foreign objects by their leading `tag` byte. Task 2
/// only has `Node`; Tasks 4/5 add the `Port`/`Bus` arms.
pub(crate) fn arg_input<S: SlotApi>(vm: &S, slot: i32) -> Input {
    match vm.slot_type(slot) {
        WrenType::Num => Input::Const(vm.get_f(slot) as f32),
        WrenType::Foreign => {
            // SAFETY: reads only the 1-byte tag at offset 0, then a 4-byte audio foreign
            // (NodeObj/PortObj/BusObj are all 4 bytes == the minimum foreign size), so no
            // over-read. Passing a non-audio foreign is a script error, not UB: it is
            // misread as an audio object with an out-of-range id, which the engine treats
            // as inert. `arg_input` trusts only that the arg is *some* >=4-byte foreign.
            let tag = unsafe { *vm.foreign_mut::<u8>(slot) };
            match tag {
                TAG_NODE => {
                    let n = unsafe { vm.foreign_mut::<NodeObj>(slot) };
                    Input::Node { node: NodeId(n.id), port: 0 }
                }
                TAG_PORT => {
                    let p = unsafe { vm.foreign_mut::<PortObj>(slot) };
                    Input::Node { node: NodeId(p.node), port: p.port }
                }
                TAG_BUS => {
                    let b = unsafe { vm.foreign_mut::<BusObj>(slot) };
                    Input::Bus(deluge_audio_graph::BusId(b.id))
                }
                _ => Input::Const(0.0),
            }
        }
        _ => Input::Const(0.0),
    }
}

fn self_id<S: SlotApi>(vm: &S) -> u16 {
    unsafe { vm.foreign_mut::<NodeObj>(0) }.id
}
unsafe fn return_node<S: SlotApi>(vm: &S, id: u16) {
    unsafe { vm.new_foreign_in(0, NodeObj { tag: TAG_NODE, id }) };
}

// ── Factory statics (return a Node) ──────────────────────────────────────────

pub(crate) fn node_src_impl<S: SlotApi>(vm: &S) {
    let kind = src_kind(vm.get_f(1) as u8);
    let freq = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, kind, [freq, Input::Const(0.0), Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_src(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_src_impl(&vm);
}

pub(crate) fn node_env_impl<S: SlotApi>(vm: &S) {
    let a = arg_input(vm, 1);
    let b = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Env, [a, b, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_env(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_env_impl(&vm);
}

pub(crate) fn node_noise_impl<S: SlotApi>(vm: &S) {
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Noise, [Input::Const(0.0); 3]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_noise(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_noise_impl(&vm);
}

pub(crate) fn node_binop_impl<S: SlotApi>(vm: &S) {
    let kind = binop_kind(vm.get_f(1) as u8);
    let a = arg_input(vm, 2);
    let b = arg_input(vm, 3);
    let id = audio::alloc_node_id();
    audio::new_node(id, kind, [a, b, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_binop(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_binop_impl(&vm);
}

pub(crate) fn node_lpf_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let cutoff = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Lpf, [input, cutoff, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_lpf(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_lpf_impl(&vm);
}

pub(crate) fn node_split_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Split2, [input, Input::Const(0.0), Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_split(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_split_impl(&vm);
}

// ── Bus (factory + instance write) ───────────────────────────────────────────

pub(crate) fn bus_new_impl<S: SlotApi>(vm: &S) {
    let id = audio::alloc_bus_id();
    unsafe { vm.new_foreign_in(0, BusObj { tag: TAG_BUS, id }) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn bus_new(raw: *mut WrenVM) {
    let vm = Vm(raw);
    bus_new_impl(&vm);
}

pub(crate) fn bus_write_impl<S: SlotApi>(vm: &S) {
    let id = unsafe { vm.foreign_mut::<BusObj>(0) }.id;
    let src = arg_input(vm, 1);
    audio::bus_write(src, id);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn bus_write(raw: *mut WrenVM) {
    let vm = Vm(raw);
    bus_write_impl(&vm);
}

// ── Out (patch / reset) — master-bus sugar, or set a Bus as root directly ───

pub(crate) fn node_patch_impl<S: SlotApi>(vm: &S) {
    // `Out.patch(arg)` → if `arg` is a Bus, set it as root directly; otherwise
    // write the Node/Port into the master bus and set the master as root.
    // SAFETY: the patch argument is one of our tagged foreign objects.
    let tag = unsafe { *vm.foreign_mut::<u8>(1) };
    if tag == TAG_BUS {
        let bus = unsafe { vm.foreign_mut::<BusObj>(1) }.id;
        audio::set_root(bus);
    } else {
        let src = arg_input(vm, 1);
        audio::bus_write(src, audio::MASTER_BUS);
        audio::set_root(audio::MASTER_BUS);
    }
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_patch(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_patch_impl(&vm);
}

pub(crate) fn node_reset_impl<S: SlotApi>(_vm: &S) {
    audio::reset();
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_reset(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_reset_impl(&vm);
}

// ── Instance methods (self = slot 0) ─────────────────────────────────────────

pub(crate) fn node_set_freq_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 0, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_freq(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_freq_impl(&vm);
}

pub(crate) fn node_set_cutoff_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 1, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_cutoff(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_cutoff_impl(&vm);
}

pub(crate) fn node_set_pm_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 1, v); // port 1 = phase-mod
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_pm(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_pm_impl(&vm);
}

pub(crate) fn node_set_width_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 2, v); // port 2 = PWM width
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_width(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_width_impl(&vm);
}

pub(crate) fn node_set_feedback_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // scalar param, not a Node/Input
    audio::set_param(self_id(vm), 0, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_feedback(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_feedback_impl(&vm);
}

pub(crate) fn node_gate_impl<S: SlotApi>(vm: &S) {
    let on = vm.get_bool(1);
    audio::gate(self_id(vm), on);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_gate(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_gate_impl(&vm);
}

pub(crate) fn node_trigger_impl<S: SlotApi>(vm: &S) {
    audio::trigger(self_id(vm));
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_trigger(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_trigger_impl(&vm);
}

pub(crate) fn node_out_impl<S: SlotApi>(vm: &S) {
    let node = self_id(vm);
    let port = vm.get_f(1) as u8;
    unsafe { vm.new_foreign_in(0, PortObj { tag: TAG_PORT, node, port }) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_out(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_out_impl(&vm);
}

pub(crate) fn node_free_impl<S: SlotApi>(vm: &S) {
    audio::free(self_id(vm));
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_free(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_free_impl(&vm);
}

/// Register the audio surface into a caller-provided method/class registrar
/// (called from `bindings::register_foreign`).
pub(crate) fn register_audio<S: SlotApi>(
    method: &mut impl FnMut(&'static str, &'static str, bool, &'static str, fn(&S)),
) {
    method("main", "Node", true, "src_(_,_)", node_src_impl::<S>);
    method("main", "Node", true, "env_(_,_)", node_env_impl::<S>);
    method("main", "Node", true, "noise_()", node_noise_impl::<S>);
    method("main", "Node", true, "binop_(_,_,_)", node_binop_impl::<S>);
    method("main", "Node", true, "lpf_(_,_)", node_lpf_impl::<S>);
    method("main", "Node", true, "patch_(_)", node_patch_impl::<S>);
    method("main", "Node", true, "reset_()", node_reset_impl::<S>);
    method("main", "Node", true, "split_(_)", node_split_impl::<S>);
    method("main", "Node", false, "freq=(_)", node_set_freq_impl::<S>);
    method("main", "Node", false, "cutoff=(_)", node_set_cutoff_impl::<S>);
    method("main", "Node", false, "pm=(_)", node_set_pm_impl::<S>);
    method("main", "Node", false, "width=(_)", node_set_width_impl::<S>);
    method("main", "Node", false, "feedback=(_)", node_set_feedback_impl::<S>);
    method("main", "Node", false, "gate(_)", node_gate_impl::<S>);
    method("main", "Node", false, "trigger()", node_trigger_impl::<S>);
    method("main", "Node", false, "out(_)", node_out_impl::<S>);
    method("main", "Node", false, "free()", node_free_impl::<S>);
    method("main", "Bus", true, "new_()", bus_new_impl::<S>);
    method("main", "Bus", false, "write(_)", bus_write_impl::<S>);
}
