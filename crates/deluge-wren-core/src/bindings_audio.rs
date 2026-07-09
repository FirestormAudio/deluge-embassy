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
pub(crate) const TAG_WT: u8 = 3;

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

/// A handle to a dynamically-uploaded (pooled) wavetable, produced by
/// `Wavetable.from([samples])` (Task 5). `handle` is `None` when the upload
/// was rejected (bad host / pool exhaustion) — `PoolHandle` has no public
/// constructor, so `None` is the only representable "unbound" state; nodes
/// built from an unbound `Wavetable` skip `BindTable` and render silent
/// rather than panicking. Larger than the other (4-byte) audio foreigns —
/// safe, because `arg_input` only ever reads the leading tag *byte* for
/// unknown tags, never the whole struct.
#[repr(C)]
pub(crate) struct WtObj {
    pub tag: u8,
    pub handle: Option<deluge_audio_graph::PoolHandle>,
}
impl WrenForeign for WtObj {
    fn module_name() -> &'static str {
        "main"
    }
    fn class_name() -> &'static str {
        "Wavetable"
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
/// Map a prelude sync-waveform code to a hard-sync `Kind`. Codes match the
/// prelude (`Osc.syncSine=0 syncSaw=1 syncSquare=2 syncTri=3`), mirroring
/// `src_kind`'s ordering.
fn sync_kind(code: u8) -> Kind {
    match code {
        0 => Kind::SyncSine,
        1 => Kind::SyncSaw,
        2 => Kind::SyncSquare,
        _ => Kind::SyncTri,
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

/// `Node.sync_(wave, master, slave)` — a hard-sync oscillator: port 0 is the
/// master frequency (resets the slave phase each cycle), port 1 the slave
/// frequency. Mirrors `node_src_impl` but takes two frequency args instead
/// of one.
pub(crate) fn node_sync_impl<S: SlotApi>(vm: &S) {
    let kind = sync_kind(vm.get_f(1) as u8);
    let master = arg_input(vm, 2);
    let slave = arg_input(vm, 3);
    let id = audio::alloc_node_id();
    audio::new_node(id, kind, [master, slave, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_sync(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_sync_impl(&vm);
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

pub(crate) fn node_pink_impl<S: SlotApi>(vm: &S) {
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PinkNoise, [Input::Const(0.0); 3]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_pink(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_pink_impl(&vm);
}

pub(crate) fn node_brown_impl<S: SlotApi>(vm: &S) {
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::BrownNoise, [Input::Const(0.0); 3]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_brown(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_brown_impl(&vm);
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

/// Wren response code → Kind (0=lp 1=hp 2=bp 3=notch), mirroring `src_kind`.
fn svf_kind(code: u32) -> Kind {
    match code {
        0 => Kind::SvfLp,
        1 => Kind::SvfHp,
        2 => Kind::SvfBp,
        _ => Kind::SvfNotch,
    }
}

pub(crate) fn node_svf_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let cutoff = arg_input(vm, 2);
    let res = arg_input(vm, 3);
    let resp = vm.get_f(4) as u32;
    let id = audio::alloc_node_id();
    audio::new_node(id, svf_kind(resp), [input, cutoff, res]);
    unsafe { return_node(vm, id) };
}

fn moog_kind(code: u32) -> Kind {
    if code == 2 {
        Kind::MoogLp2
    } else {
        Kind::MoogLp4
    }
}

pub(crate) fn node_moog_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let cutoff = arg_input(vm, 2);
    let res = arg_input(vm, 3);
    let poles = vm.get_f(4) as u32; // 4 (24 dB) or 2 (12 dB)
    let id = audio::alloc_node_id();
    audio::new_node(id, moog_kind(poles), [input, cutoff, res]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_moog(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_moog_impl(&vm);
}

/// Wren response code → Kind (0=lp 1=hp), mirroring `svf_kind`.
fn ms20_kind(code: u32) -> Kind {
    if code == 1 { Kind::Ms20Hp } else { Kind::Ms20Lp }
}

pub(crate) fn node_ms20_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let cutoff = arg_input(vm, 2);
    let res = arg_input(vm, 3);
    let resp = vm.get_f(4) as u32; // 0=lp, 1=hp
    let id = audio::alloc_node_id();
    audio::new_node(id, ms20_kind(resp), [input, cutoff, res]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_ms20(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_ms20_impl(&vm);
}

pub(crate) fn node_modal_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let freq = arg_input(vm, 2);
    let damping = arg_input(vm, 3);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Modal, [input, freq, damping]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_modal(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_modal_impl(&vm);
}

pub(crate) fn node_set_drive_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // scalar control param (mirrors node_set_feedback_impl)
    audio::set_param(self_id(vm), 0, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_drive(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_drive_impl(&vm);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_svf(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_svf_impl(&vm);
}

pub(crate) fn node_tb303_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let cutoff = arg_input(vm, 2);
    let res = arg_input(vm, 3);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Tb303, [input, cutoff, res]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_tb303(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_tb303_impl(&vm);
}

pub(crate) fn node_wavetable_impl<S: SlotApi>(vm: &S) {
    let table_id = vm.get_f(1) as u16;
    let freq = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_wavetable(id, table_id, freq);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_wavetable(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_wavetable_impl(&vm);
}

/// `Wavetable.from([samples])` — read the Wren list arg (slot 1), copy up to
/// `mipgen::N` elements into a fixed base-cycle buffer, upload it to the
/// host's table pool, and return a `Wavetable` handle wrapping the resulting
/// `Option<PoolHandle>` (`None` on a host with no pool, e.g. the Cmd-capture
/// test host — the handle stays unbound rather than panicking).
///
/// Lifetime note: the pool region backing the returned handle is
/// node-scoped, not object-scoped — it is freed by `Cmd::Free` on whichever
/// node gets bound to this `Wavetable` (e.g. via `Node.wavetable_pooled_`),
/// *not* by the `Wavetable` Wren object's GC. Bind the returned `Wavetable`
/// to a node and free that node to release the table's memory; don't rely
/// on GC to free it. Don't free a node while another node still shares the
/// same `Wavetable` — that reclaims the region out from under the survivor.
/// Consequences of misuse are always graceful (silence or a finite leak
/// until the pool exhausts), never UB or a panic, but this is an accepted
/// limitation pending a proper object-scoped ownership model
/// (finalizer/refcount) as a follow-on.
pub(crate) fn wavetable_from_impl<S: SlotApi>(vm: &S) {
    let count = vm.get_list_count(1);
    let mut base = [0.0f32; mipgen::N];
    let n = (count.max(0) as usize).min(base.len());
    vm.ensure_slots(3); // guarantee slot 2 (scratch, for list-element reads) is valid
    for i in 0..n {
        vm.get_list_element(1, i as i32, 2); // element -> slot 2
        base[i] = vm.get_f(2) as f32;
    }
    let handle = audio::upload_table(&base[..n.max(1)]);
    unsafe { vm.new_foreign_in::<WtObj>(0, WtObj { tag: TAG_WT, handle }) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn wavetable_from(raw: *mut WrenVM) {
    let vm = Vm(raw);
    wavetable_from_impl(&vm);
}

/// `Wavetable.from2d([[frame0...], [frame1...], ...])` — read a nested Wren
/// list (outer = frames, inner = one frame's samples), and upload it as a
/// multi-frame table: one compact pyramid per frame, built into a single
/// `nframes * PYRAMID_LEN` pool region. Bound to a `Kind::Wavetable` node
/// (via `Osc.wavetable`), it morphs across frames by `.position` (port 2,
/// see `node_set_position_impl`) instead of rendering a single bit-exact
/// cycle. Same graceful-degrade contract as `wavetable_from_impl`: `None` on
/// a host with no pool leaves the returned handle unbound rather than
/// panicking, and the same node-scoped-lifetime caveat applies (see that
/// function's doc comment).
///
/// Nested-list read: the VM only exposes one "current" list per slot pair
/// (`get_list_element(list_slot, i, elem_slot)`), so reading frame `f`'s
/// samples needs its own slot (2) distinct from the outer list's slot (1)
/// and the per-sample scratch slot (3) — `ensure_slots(4)` guarantees all
/// three (plus slot 0, `self`/return) are valid. The inner read happens
/// inside the closure handed to `audio::upload_table_2d`: the host (pool
/// access) and the binding (VM slot access) never touch each other's state,
/// which is what keeps this sound through the `Host` trait object.
pub(crate) fn wavetable_from2d_impl<S: SlotApi>(vm: &S) {
    let nframes = (vm.get_list_count(1).max(0)) as usize;
    vm.ensure_slots(4); // 1=outer(frames) list, 2=inner(frame) list, 3=sample scratch
    let handle = audio::upload_table_2d(nframes, &mut |f, base| {
        // Zero first: `get_list_count(2)` may be shorter than `base.len()`
        // (mipgen::N) for this frame, and `base` is a fresh stack buffer per
        // frame with no other defined initial content to fall back on.
        for s in base.iter_mut() {
            *s = 0.0;
        }
        vm.get_list_element(1, f as i32, 2); // frame f -> slot 2
        let n = (vm.get_list_count(2).max(0) as usize).min(base.len());
        for i in 0..n {
            vm.get_list_element(2, i as i32, 3); // sample -> slot 3
            base[i] = vm.get_f(3) as f32;
        }
    });
    unsafe { vm.new_foreign_in::<WtObj>(0, WtObj { tag: TAG_WT, handle }) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn wavetable_from2d(raw: *mut WrenVM) {
    let vm = Vm(raw);
    wavetable_from2d_impl(&vm);
}

/// `Node.wavetable_pooled_(wt, freq)` — the pooled-table counterpart of
/// `node_wavetable_impl` (which binds a static/named table). Reads the
/// `Wavetable` handle from slot 1: a bound handle emits `NewNode` + a
/// `BindTable{Pooled}`; an unbound one (upload failed) still creates the
/// node but skips the bind, so it renders silent instead of panicking.
pub(crate) fn node_wavetable_pooled_impl<S: SlotApi>(vm: &S) {
    let handle = unsafe { vm.foreign_mut::<WtObj>(1) }.handle;
    let freq = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    match handle {
        Some(h) => audio::new_wavetable_pooled(id, h, freq),
        None => audio::new_node(id, Kind::Wavetable, [freq, Input::Const(0.0), Input::Const(0.0)]),
    }
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_wavetable_pooled(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_wavetable_pooled_impl(&vm);
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

pub(crate) fn node_set_res_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 2, v); // port 2 = resonance
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_res(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_res_impl(&vm);
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

pub(crate) fn node_set_position_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    audio::set_input(self_id(vm), 2, v); // port 2 = morph position (Kind::Wavetable, FRAMES>1)
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_position(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_position_impl(&vm);
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

pub(crate) fn node_set_structure_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // scalar control param — port 0 = structure (Kind::Modal)
    audio::set_param(self_id(vm), 0, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_structure(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_structure_impl(&vm);
}

pub(crate) fn node_set_brightness_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // scalar control param — port 1 = brightness (Kind::Modal)
    audio::set_param(self_id(vm), 1, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_brightness(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_brightness_impl(&vm);
}

// NOTE: `position=`/`node_set_position` already exist (wavetable morph → set_input). Do NOT
// reuse or shadow them. The resonator's strike position uses `strike=`/`node_set_strike`.
pub(crate) fn node_set_strike_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // scalar control param — port 2 = strike position (Kind::Modal)
    audio::set_param(self_id(vm), 2, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_strike(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_strike_impl(&vm);
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
    method("main", "Node", true, "sync_(_,_,_)", node_sync_impl::<S>);
    method("main", "Node", true, "env_(_,_)", node_env_impl::<S>);
    method("main", "Node", true, "noise_()", node_noise_impl::<S>);
    method("main", "Node", true, "pink_()", node_pink_impl::<S>);
    method("main", "Node", true, "brown_()", node_brown_impl::<S>);
    method("main", "Node", true, "binop_(_,_,_)", node_binop_impl::<S>);
    method("main", "Node", true, "lpf_(_,_)", node_lpf_impl::<S>);
    method("main", "Node", true, "svf_(_,_,_,_)", node_svf_impl::<S>);
    method("main", "Node", true, "moog_(_,_,_,_)", node_moog_impl::<S>);
    method("main", "Node", true, "ms20_(_,_,_,_)", node_ms20_impl::<S>);
    method("main", "Node", true, "modal_(_,_,_)", node_modal_impl::<S>);
    method("main", "Node", false, "drive=(_)", node_set_drive_impl::<S>);
    method("main", "Node", true, "tb303_(_,_,_)", node_tb303_impl::<S>);
    method("main", "Node", true, "patch_(_)", node_patch_impl::<S>);
    method("main", "Node", true, "reset_()", node_reset_impl::<S>);
    method("main", "Node", true, "split_(_)", node_split_impl::<S>);
    method("main", "Node", true, "wavetable_(_,_)", node_wavetable_impl::<S>);
    method("main", "Node", true, "wavetable_pooled_(_,_)", node_wavetable_pooled_impl::<S>);
    method("main", "Node", false, "freq=(_)", node_set_freq_impl::<S>);
    method("main", "Node", false, "cutoff=(_)", node_set_cutoff_impl::<S>);
    method("main", "Node", false, "res=(_)", node_set_res_impl::<S>);
    method("main", "Node", false, "pm=(_)", node_set_pm_impl::<S>);
    method("main", "Node", false, "width=(_)", node_set_width_impl::<S>);
    method("main", "Node", false, "position=(_)", node_set_position_impl::<S>);
    method("main", "Node", false, "feedback=(_)", node_set_feedback_impl::<S>);
    method("main", "Node", false, "structure=(_)", node_set_structure_impl::<S>);
    method("main", "Node", false, "brightness=(_)", node_set_brightness_impl::<S>);
    method("main", "Node", false, "strike=(_)", node_set_strike_impl::<S>);
    method("main", "Node", false, "gate(_)", node_gate_impl::<S>);
    method("main", "Node", false, "trigger()", node_trigger_impl::<S>);
    method("main", "Node", false, "out(_)", node_out_impl::<S>);
    method("main", "Node", false, "free()", node_free_impl::<S>);
    method("main", "Bus", true, "new_()", bus_new_impl::<S>);
    method("main", "Bus", false, "write(_)", bus_write_impl::<S>);
    method("main", "Wavetable", true, "from(_)", wavetable_from_impl::<S>);
    method("main", "Wavetable", true, "from2d(_)", wavetable_from2d_impl::<S>);
}
