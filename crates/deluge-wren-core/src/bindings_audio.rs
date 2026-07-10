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

/// Ring-buffer length for a `Delay` node: ≈1.09 s @ 44.1 kHz, 1.0 s @ 48 kHz.
/// The delay `time` (port 1) is clamped to this length inside the kernel.
pub(crate) const DELAY_MAX_SAMPLES: usize = 48_000;

/// Ring length for a chorus/flanger node (~50 ms @ 48 kHz). Covers `base·(1+depth)`
/// for both effects (chorus base 20 ms, flanger 2 ms) plus headroom.
pub(crate) const CHORUS_BUF_SAMPLES: usize = 2400;

/// Ring length for a Room reverb node. Keep in sync with
/// `deluge_dsp_kernels::reverb::REVERB_BUF_SAMPLES` (Σ of the 24 line lengths).
pub(crate) const REVERB_BUF_SAMPLES: usize = 25_450;

/// Ring length for a Hall (FDN) reverb node. Keep in sync with
/// `deluge_dsp_kernels::reverb::HALL_BUF_SAMPLES`.
pub(crate) const HALL_BUF_SAMPLES: usize = 23_748;

/// Ring length for a Plate (Dattorro) reverb node. Keep in sync with
/// `deluge_dsp_kernels::reverb::PLATE_BUF_SAMPLES`.
pub(crate) const PLATE_BUF_SAMPLES: usize = 22_494;

#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct NodeObj {
    pub tag: u8,
    pub width: u8, // 1 = mono, 2 = stereo (port0=L, port1=R). Read by width-aware routing.
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
unsafe fn return_node_w<S: SlotApi>(vm: &S, id: u16, width: u8) {
    unsafe { vm.new_foreign_in(0, NodeObj { tag: TAG_NODE, width, id }) };
}
unsafe fn return_node<S: SlotApi>(vm: &S, id: u16) {
    unsafe { return_node_w(vm, id, 1) };
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

/// `Node.delay_(input, time, feedback)` — allocate a zeroed ring buffer and
/// create a `Kind::Delay` node bound to it (ports 0/1/2 = input/time/feedback).
/// On a host with no pool the alloc returns `None`; the node is still created
/// but unbound, rendering dry passthrough rather than panicking.
pub(crate) fn node_delay_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let time = arg_input(vm, 2);
    let feedback = arg_input(vm, 3);
    let handle = audio::alloc_buffer(DELAY_MAX_SAMPLES);
    let id = audio::alloc_node_id();
    audio::new_delay(id, handle, input, time, feedback);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_delay(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_delay_impl(&vm);
}

pub(crate) fn node_set_mix_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // scalar control param — index 0 = mix (Kind::Delay)
    audio::set_param(self_id(vm), 0, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_mix(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_mix_impl(&vm);
}

/// `Node.chorus_(input, rate, depth, mix)` — allocate a ring, create a width-2
/// `Kind::Chorus` node, and set its rate/depth/mix params from the args. A
/// host with no pool leaves it unbound (dry passthrough).
pub(crate) fn node_chorus_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let rate = vm.get_f(2) as f32;
    let depth = vm.get_f(3) as f32;
    let mix = vm.get_f(4) as f32;
    let handle = audio::alloc_buffer(CHORUS_BUF_SAMPLES);
    let id = audio::alloc_node_id();
    audio::new_pooled_node(id, Kind::Chorus, handle, input);
    audio::set_param(id, 1, rate);
    audio::set_param(id, 2, depth);
    audio::set_param(id, 0, mix);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_chorus(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_chorus_impl(&vm);
}

/// `Node.flanger_(input, rate, depth, feedback, mix)` — like `node_chorus_impl`
/// but `Kind::Flanger` (single voice) with a feedback (param 3) arg.
pub(crate) fn node_flanger_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let rate = vm.get_f(2) as f32;
    let depth = vm.get_f(3) as f32;
    let feedback = vm.get_f(4) as f32;
    let mix = vm.get_f(5) as f32;
    let handle = audio::alloc_buffer(CHORUS_BUF_SAMPLES);
    let id = audio::alloc_node_id();
    audio::new_pooled_node(id, Kind::Flanger, handle, input);
    audio::set_param(id, 1, rate);
    audio::set_param(id, 2, depth);
    audio::set_param(id, 3, feedback);
    audio::set_param(id, 0, mix);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_flanger(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_flanger_impl(&vm);
}

/// `Node.room_(input, roomsize, damp, mix)` — allocate the partitioned reverb
/// buffer, create a width-2 `Kind::Room` node, and set roomsize/damp/mix. A host
/// with no pool leaves it unbound (dry passthrough).
pub(crate) fn node_room_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let roomsize = vm.get_f(2) as f32;
    let damp = vm.get_f(3) as f32;
    let mix = vm.get_f(4) as f32;
    let handle = audio::alloc_buffer(REVERB_BUF_SAMPLES);
    let id = audio::alloc_node_id();
    audio::new_pooled_node(id, Kind::Room, handle, input);
    audio::set_param(id, 2, roomsize);
    audio::set_param(id, 1, damp);
    audio::set_param(id, 0, mix);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_room(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_room_impl(&vm);
}

/// `Node.hall_(input, size, damp, mix)` — allocate the FDN buffer, create a
/// width-2 `Kind::Hall` node, and set size/damp/mix. Reuses the Room setters
/// (`mix=`/`damp=`/`size=`/`spread=`). Unbound → dry passthrough.
pub(crate) fn node_hall_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let size = vm.get_f(2) as f32;
    let damp = vm.get_f(3) as f32;
    let mix = vm.get_f(4) as f32;
    let handle = audio::alloc_buffer(HALL_BUF_SAMPLES);
    let id = audio::alloc_node_id();
    audio::new_pooled_node(id, Kind::Hall, handle, input);
    audio::set_param(id, 2, size);
    audio::set_param(id, 1, damp);
    audio::set_param(id, 0, mix);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_hall(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_hall_impl(&vm);
}

/// `Node.plate_(input, size, damp, mix)` — allocate the Dattorro buffer, create a
/// width-2 `Kind::Plate` node, and set size/damp/mix. Reuses the Room setters.
/// Unbound → dry passthrough.
pub(crate) fn node_plate_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let size = vm.get_f(2) as f32;
    let damp = vm.get_f(3) as f32;
    let mix = vm.get_f(4) as f32;
    let handle = audio::alloc_buffer(PLATE_BUF_SAMPLES);
    let id = audio::alloc_node_id();
    audio::new_pooled_node(id, Kind::Plate, handle, input);
    audio::set_param(id, 2, size);
    audio::set_param(id, 1, damp);
    audio::set_param(id, 0, mix);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_plate(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_plate_impl(&vm);
}

/// `Node.drive_(input, drive, tone, mix, shape)` — a mono waveshaper. Creates a
/// `Kind::Drive` node (no buffer) and sets shape/drive/tone/mix params.
pub(crate) fn node_drive_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let drive = vm.get_f(2) as f32;
    let tone = vm.get_f(3) as f32;
    let mix = vm.get_f(4) as f32;
    let shape = vm.get_f(5) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Drive, [input, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 3, shape);
    audio::set_param(id, 0, drive);
    audio::set_param(id, 1, tone);
    audio::set_param(id, 2, mix);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_drive(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_drive_impl(&vm);
}

pub(crate) fn node_set_tone_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 1 = tone (Kind::Drive)
    audio::set_param(self_id(vm), 1, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_tone(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_tone_impl(&vm);
}

// `wet=` (Drive dry/wet, param 2) — NOT `mix=` (set_param 0, collides with `drive=`).
pub(crate) fn node_set_wet_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32;
    audio::set_param(self_id(vm), 2, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_wet(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_wet_impl(&vm);
}

pub(crate) fn node_set_rate_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 1 = rate (Kind::Chorus/Flanger)
    audio::set_param(self_id(vm), 1, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_rate(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_rate_impl(&vm);
}

pub(crate) fn node_set_size_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 2 = roomsize (Kind::Room)
    audio::set_param(self_id(vm), 2, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_size(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_size_impl(&vm);
}

// `spread=` (stereo width, param 3) — NOT `width=` (the Osc's PWM port setter).
pub(crate) fn node_set_spread_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 3 = width (Kind::Room)
    audio::set_param(self_id(vm), 3, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_spread(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_spread_impl(&vm);
}

pub(crate) fn node_set_depth_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 2 = depth
    audio::set_param(self_id(vm), 2, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_depth(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_depth_impl(&vm);
}

// `regen=` (NOT `feedback=`, which is the Osc's set_param(0)): flanger feedback.
pub(crate) fn node_set_regen_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 3 = feedback
    audio::set_param(self_id(vm), 3, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_regen(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_regen_impl(&vm);
}

// NOTE: `damp=` (NOT `damping=`). `damping=` is already bound to
// `node_set_res_impl` (Resonator, port-2 set_input); a Delay's port 2 is
// feedback, so it needs a distinct selector → index 1 = damping (Kind::Delay).
pub(crate) fn node_set_damp_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32;
    audio::set_param(self_id(vm), 1, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_damp(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_damp_impl(&vm);
}

/// `Node.eq_(input, freq, gain, q, type)` — a mono RBJ EQ band. Creates a
/// `Kind::Eq` node (no buffer) and sets type/freq/gain/q params.
pub(crate) fn node_eq_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let freq = vm.get_f(2) as f32;
    let gain = vm.get_f(3) as f32;
    let q = vm.get_f(4) as f32;
    let ty = vm.get_f(5) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Eq, [input, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 3, ty);
    audio::set_param(id, 0, freq);
    audio::set_param(id, 1, gain);
    audio::set_param(id, 2, q);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_eq(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_eq_impl(&vm);
}

pub(crate) fn node_set_hz_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 0 = freq (Kind::Eq)
    audio::set_param(self_id(vm), 0, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_hz(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_hz_impl(&vm);
}

pub(crate) fn node_set_gain_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 1 = gain dB (Kind::Eq)
    audio::set_param(self_id(vm), 1, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_gain(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_gain_impl(&vm);
}

pub(crate) fn node_set_q_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 2 = Q (Kind::Eq)
    audio::set_param(self_id(vm), 2, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_q(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_q_impl(&vm);
}

/// `Node.lfo_(rate, shape)` — an LFO modulation source. `Kind::Lfo`, rate on
/// port 0, shape set as param 0. Mono, no buffer.
pub(crate) fn node_lfo_impl<S: SlotApi>(vm: &S) {
    let rate = arg_input(vm, 1);
    let shape = vm.get_f(2) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Lfo, [rate, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, shape);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_lfo(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_lfo_impl(&vm);
}

pub(crate) fn node_set_phase_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32; // param 1 = phase offset (Kind::Lfo)
    audio::set_param(self_id(vm), 1, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_phase(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_phase_impl(&vm);
}

/// `Node.sh_(input, clock)` — sample & hold. Ports 0=input, 1=clock.
pub(crate) fn node_sh_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let clock = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::SampleHold, [input, clock, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_sh(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_sh_impl(&vm);
}

/// `Node.slew_(input, time)` — one-pole glide. Ports 0=input, 1=time.
pub(crate) fn node_slew_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let time = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Slew, [input, time, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_slew(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_slew_impl(&vm);
}

/// `Node.steps_(values, clock)` — step sequencer. Reads the Wren list (slot 1)
/// into `set_param` calls: param 0 = length, param k+1 = values[k] (up to
/// MAX_STEPS; longer lists are truncated). Clock on port 0.
pub(crate) fn node_steps_impl<S: SlotApi>(vm: &S) {
    let count = vm.get_list_count(1).max(0) as usize;
    let len = count.min(deluge_dsp_kernels::modutil::MAX_STEPS);
    let clock = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Steps, [clock, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, len as f32);
    vm.ensure_slots(3); // slot 2 = per-element scratch
    for k in 0..len {
        vm.get_list_element(1, k as i32, 2);
        let v = vm.get_f(2) as f32;
        audio::set_param(id, (k + 1) as u8, v);
    }
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_steps(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_steps_impl(&vm);
}

/// `Node.curve_(input, k)` — odd-symmetric Schlick-bias transfer. Ports 0=input,
/// 1=k (∈[-1,1], modulatable). Stateless (`Kind::Curve`).
pub(crate) fn node_curve_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let k = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Curve, [input, k, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_curve(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_curve_impl(&vm);
}

/// `Node.ctrl_(value)` — a settable scalar macro source. No inputs; `param 0 = value`.
pub(crate) fn node_ctrl_impl<S: SlotApi>(vm: &S) {
    let value = vm.get_f(1) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Ctrl, [Input::Const(0.0); 3]);
    audio::set_param(id, 0, value);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_ctrl(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_ctrl_impl(&vm);
}

/// `Node.qstep_(input, n)` — snap to N equal levels. Port 0=input, `param 0 = N`.
pub(crate) fn node_qstep_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let n = vm.get_f(2) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::QuantStep, [input, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, n);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_qstep(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_qstep_impl(&vm);
}

/// `Node.qpitch_(input, mask, root)` — snap semitones to a scale. Port 0=input,
/// `param 0 = 12-bit mask`, `param 1 = root`.
pub(crate) fn node_qpitch_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let mask = vm.get_f(2) as f32;
    let root = vm.get_f(3) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::QuantPitch, [input, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, mask);
    audio::set_param(id, 1, root);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_qpitch(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_qpitch_impl(&vm);
}

/// `Node.mtof_(input, ref)` — semitone → Hz. Port 0=input, `param 0 = ref Hz`.
pub(crate) fn node_mtof_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let reference = vm.get_f(2) as f32;
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Mtof, [input, Input::Const(0.0), Input::Const(0.0)]);
    audio::set_param(id, 0, reference);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_mtof(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_mtof_impl(&vm);
}

/// `Node.polyMode_` — 1.0 while inside a Synth build, else 0.0 (number, since
/// Wren treats 0 as truthy — the prelude compares `== 1`).
pub(crate) fn node_poly_mode_impl<S: SlotApi>(vm: &S) {
    vm.set_f(0, if audio::poly_mode() { 1.0 } else { 0.0 });
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_poly_mode(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_poly_mode_impl(&vm);
}

/// `Node.polyGateCount_` — number of Env.ar created this build.
pub(crate) fn node_poly_gate_count_impl<S: SlotApi>(vm: &S) {
    vm.set_f(0, audio::poly_gate_count() as f64);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_poly_gate_count(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_poly_gate_count_impl(&vm);
}

/// `Node.polyBegin_()` — start a voice build; returns the `pitch` node (PolyMtof).
pub(crate) fn node_poly_begin_impl<S: SlotApi>(vm: &S) {
    let mtof = audio::poly_begin();
    unsafe { return_node(vm, mtof) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_poly_begin(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_poly_begin_impl(&vm);
}

/// `Node.polyosc_(pitch)` — poly sine oscillator. Ports 0 = pitch (poly Hz).
pub(crate) fn node_polyosc_impl<S: SlotApi>(vm: &S) {
    let pitch = arg_input(vm, 1);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolyOsc, [pitch, Input::Const(0.0), Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_polyosc(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_polyosc_impl(&vm);
}

/// `Node.polysvf_(audio, cutoff, res)` — poly SVF lowpass. Port 0 = audio (poly),
/// ports 1/2 = cutoff/res (mono).
pub(crate) fn node_polysvf_impl<S: SlotApi>(vm: &S) {
    let audio_in = arg_input(vm, 1);
    let cutoff = arg_input(vm, 2);
    let res = arg_input(vm, 3);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolySvf, [audio_in, cutoff, res]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_polysvf(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_polysvf_impl(&vm);
}

/// `Node.polyar_(attack, release)` — poly AR envelope (the amp gate). Ports
/// 0/1 = attack/release (mono). Records itself as the voice's gate.
pub(crate) fn node_polyar_impl<S: SlotApi>(vm: &S) {
    let attack = arg_input(vm, 1);
    let release = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolyAr, [attack, release, Input::Const(0.0)]);
    audio::poly_record_gate(id);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_polyar(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_polyar_impl(&vm);
}

/// `Node.polymul_(a, b)` — poly × poly (the VCA). Ports 0/1 = both poly.
pub(crate) fn node_polymul_impl<S: SlotApi>(vm: &S) {
    let a = arg_input(vm, 1);
    let b = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::PolyMul, [a, b, Input::Const(0.0)]);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_polymul(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_polymul_impl(&vm);
}

/// `macro.value = v` — set a `Ctrl` node's held value (`param 0`).
pub(crate) fn node_set_value_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32;
    audio::set_param(self_id(vm), 0, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_value(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_set_value_impl(&vm);
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

/// `Node.pan_(input, position)` — a constant-power mono→stereo pan node
/// (`Kind::Pan`, width-2: port0=L, port1=R). `position` is port 1 (∈[-1,1],
/// modulatable). Returns a WIDTH-2 node so width-aware routing sends its two
/// ports to L/R (see `write_source_to_bus`, Task 5).
pub(crate) fn node_pan_impl<S: SlotApi>(vm: &S) {
    let input = arg_input(vm, 1);
    let position = arg_input(vm, 2);
    let id = audio::alloc_node_id();
    audio::new_node(id, Kind::Pan, [input, position, Input::Const(0.0)]);
    unsafe { return_node_w(vm, id, 2) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_pan(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_pan_impl(&vm);
}

/// Route a source argument (at `slot`) to `bus`, honoring stereo width. A
/// width-2 `Node` (e.g. `Pan`, `Chorus`) emits TWO per-side writes — port0→L
/// `(1,0)`, port1→R `(0,1)`. Everything else (mono `Node`, `Port`, number,
/// `Bus`) emits one center write `(1,1)` — unchanged behavior.
fn write_source_to_bus<S: SlotApi>(vm: &S, slot: i32, bus: u16) {
    if vm.slot_type(slot) == WrenType::Foreign {
        // SAFETY: reads the leading tag byte, then (for TAG_NODE) the 4-byte
        // NodeObj — same access discipline as `arg_input`.
        let tag = unsafe { *vm.foreign_mut::<u8>(slot) };
        if tag == TAG_NODE {
            let n = unsafe { vm.foreign_mut::<NodeObj>(slot) };
            if n.width == 2 {
                let id = n.id;
                audio::bus_write_gains(Input::Node { node: NodeId(id), port: 0 }, bus, 1.0, 0.0);
                audio::bus_write_gains(Input::Node { node: NodeId(id), port: 1 }, bus, 0.0, 1.0);
                return;
            }
        }
    }
    audio::bus_write(arg_input(vm, slot), bus); // mono center (1,1)
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
    write_source_to_bus(vm, 1, id);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn bus_write(raw: *mut WrenVM) {
    let vm = Vm(raw);
    bus_write_impl(&vm);
}

// ── Out (patch / reset) — master-bus sugar, or set a Bus as root directly ───

pub(crate) fn node_patch_impl<S: SlotApi>(vm: &S) {
    // `Out.patch(arg)` → if `arg` is a Bus, set it as root directly; otherwise
    // route the source (stereo-aware) into the master bus and set it as root.
    let tag = unsafe { *vm.foreign_mut::<u8>(1) };
    if tag == TAG_BUS {
        let bus = unsafe { vm.foreign_mut::<BusObj>(1) }.id;
        audio::set_root(bus);
    } else {
        write_source_to_bus(vm, 1, audio::MASTER_BUS);
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
    method("main", "Node", true, "pan_(_,_)", node_pan_impl::<S>);
    method("main", "Node", true, "wavetable_(_,_)", node_wavetable_impl::<S>);
    method("main", "Node", true, "wavetable_pooled_(_,_)", node_wavetable_pooled_impl::<S>);
    method("main", "Node", true, "delay_(_,_,_)", node_delay_impl::<S>);
    method("main", "Node", false, "mix=(_)", node_set_mix_impl::<S>);
    method("main", "Node", false, "damp=(_)", node_set_damp_impl::<S>);
    method("main", "Node", true, "chorus_(_,_,_,_)", node_chorus_impl::<S>);
    method("main", "Node", true, "flanger_(_,_,_,_,_)", node_flanger_impl::<S>);
    method("main", "Node", true, "room_(_,_,_,_)", node_room_impl::<S>);
    method("main", "Node", true, "hall_(_,_,_,_)", node_hall_impl::<S>);
    method("main", "Node", true, "plate_(_,_,_,_)", node_plate_impl::<S>);
    method("main", "Node", true, "drive_(_,_,_,_,_)", node_drive_impl::<S>);
    method("main", "Node", false, "tone=(_)", node_set_tone_impl::<S>);
    method("main", "Node", false, "wet=(_)", node_set_wet_impl::<S>);
    method("main", "Node", true, "eq_(_,_,_,_,_)", node_eq_impl::<S>);
    method("main", "Node", false, "hz=(_)", node_set_hz_impl::<S>);
    method("main", "Node", false, "gain=(_)", node_set_gain_impl::<S>);
    method("main", "Node", false, "q=(_)", node_set_q_impl::<S>);
    method("main", "Node", true, "lfo_(_,_)", node_lfo_impl::<S>);
    method("main", "Node", false, "phase=(_)", node_set_phase_impl::<S>);
    method("main", "Node", true, "sh_(_,_)", node_sh_impl::<S>);
    method("main", "Node", true, "slew_(_,_)", node_slew_impl::<S>);
    method("main", "Node", true, "steps_(_,_)", node_steps_impl::<S>);
    method("main", "Node", true, "curve_(_,_)", node_curve_impl::<S>);
    method("main", "Node", true, "ctrl_(_)", node_ctrl_impl::<S>);
    method("main", "Node", true, "qstep_(_,_)", node_qstep_impl::<S>);
    method("main", "Node", true, "qpitch_(_,_,_)", node_qpitch_impl::<S>);
    method("main", "Node", true, "mtof_(_,_)", node_mtof_impl::<S>);
    method("main", "Node", true, "polyMode_", node_poly_mode_impl::<S>);
    method("main", "Node", true, "polyGateCount_", node_poly_gate_count_impl::<S>);
    method("main", "Node", true, "polyBegin_()", node_poly_begin_impl::<S>);
    method("main", "Node", true, "polyosc_(_)", node_polyosc_impl::<S>);
    method("main", "Node", true, "polysvf_(_,_,_)", node_polysvf_impl::<S>);
    method("main", "Node", true, "polyar_(_,_)", node_polyar_impl::<S>);
    method("main", "Node", true, "polymul_(_,_)", node_polymul_impl::<S>);
    method("main", "Node", false, "value=(_)", node_set_value_impl::<S>);
    method("main", "Node", false, "size=(_)", node_set_size_impl::<S>);
    method("main", "Node", false, "spread=(_)", node_set_spread_impl::<S>);
    method("main", "Node", false, "rate=(_)", node_set_rate_impl::<S>);
    method("main", "Node", false, "depth=(_)", node_set_depth_impl::<S>);
    method("main", "Node", false, "regen=(_)", node_set_regen_impl::<S>);
    method("main", "Node", false, "freq=(_)", node_set_freq_impl::<S>);
    method("main", "Node", false, "cutoff=(_)", node_set_cutoff_impl::<S>);
    method("main", "Node", false, "res=(_)", node_set_res_impl::<S>);
    // Resonator freq/damping aliases (ports 1/2) — see bindings.rs note.
    method("main", "Node", false, "pitch=(_)", node_set_cutoff_impl::<S>);
    method("main", "Node", false, "damping=(_)", node_set_res_impl::<S>);
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
