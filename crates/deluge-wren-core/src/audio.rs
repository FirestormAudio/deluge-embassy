//! Client-side id allocators + control-rate command emitters for the Wren audio
//! bindings. The binding owns the `NodeId` free-list and `BusId` allocation (no
//! round-trip); each emitter builds a `deluge_audio_graph::Cmd` and ships it
//! through the registered [`Host`](crate::Host). Single-threaded VM context.

use deluge_audio_graph::{BusId, Cmd, Input, Kind, NodeId, MAX_GATES};

use crate::host::host;

/// Binding-side node-id capacity. A host `Engine` must have `NODES >= this`.
pub const WREN_MAX_NODES: usize = 64;
/// Binding-side bus capacity. A host `Engine` must have `BUSES >= this`.
pub const WREN_MAX_BUSES: usize = 8;
/// The implicit master bus (render root for `Out.patch(node)`).
pub const MASTER_BUS: u16 = 0;
/// Returned when the pool is exhausted; factories no-op on it (inert node).
pub const NULL_ID: u16 = u16::MAX;

struct Alloc {
    next: u16,                     // bump pointer for node ids
    free: [u16; WREN_MAX_NODES],   // stack of freed node ids
    free_len: usize,
    next_bus: u16,                 // bump pointer for bus ids (1.. ; 0 = master)
}

impl Alloc {
    const fn new() -> Self {
        Alloc { next: 0, free: [0; WREN_MAX_NODES], free_len: 0, next_bus: 1 }
    }
    fn alloc_node(&mut self) -> u16 {
        if self.free_len > 0 {
            self.free_len -= 1;
            return self.free[self.free_len];
        }
        if (self.next as usize) < WREN_MAX_NODES {
            let id = self.next;
            self.next += 1;
            id
        } else {
            NULL_ID
        }
    }
    // Used by `free_node_id` below, which is called from `free()` (in turn
    // called by the `Node.free()` binding) to return an id to the free-list.
    fn free_node(&mut self, id: u16) {
        if (id as usize) < WREN_MAX_NODES && self.free_len < WREN_MAX_NODES {
            self.free[self.free_len] = id;
            self.free_len += 1;
        }
    }
    fn alloc_bus(&mut self) -> u16 {
        if (self.next_bus as usize) < WREN_MAX_BUSES {
            let id = self.next_bus;
            self.next_bus += 1;
            id
        } else {
            NULL_ID
        }
    }
    fn reset(&mut self) {
        self.next = 0;
        self.free_len = 0;
        self.next_bus = 1;
    }
}

// SAFETY: single-threaded VM context, like the rest of the binding state.
static mut ALLOC: Alloc = Alloc::new();

#[inline]
fn alloc() -> &'static mut Alloc {
    // SAFETY: sole accessor is the VM thread.
    unsafe { &mut *core::ptr::addr_of_mut!(ALLOC) }
}

/// Voice-build state for `Synth.new` (set between polyBegin_/polyEnd_).
struct PolyCtx {
    mode: bool,
    pitch_ctrl: u16,          // the PolyCtrl created by polyBegin_
    gates: [u16; MAX_GATES],  // envelope node ids (first `gate_count`)
    gate_count: u8,  // total envelopes created this build (may exceed MAX_GATES → guard aborts)
    vel_node: u16,   // velocity PolyCtrl id, or NULL_ID if the builder didn't take velocity
    slew_node: u16,  // the PolySlew created by mono_begin, or NULL_ID for a poly build
}
impl PolyCtx {
    const fn new() -> Self {
        PolyCtx {
            mode: false,
            pitch_ctrl: NULL_ID,
            gates: [NULL_ID; MAX_GATES],
            gate_count: 0,
            vel_node: NULL_ID,
            slew_node: NULL_ID,
        }
    }
}
// SAFETY: single-threaded VM context, like `ALLOC`.
static mut POLY: PolyCtx = PolyCtx::new();

#[inline]
fn poly() -> &'static mut PolyCtx {
    // SAFETY: sole accessor is the VM thread.
    unsafe { &mut *core::ptr::addr_of_mut!(POLY) }
}

pub fn poly_mode() -> bool {
    poly().mode
}
pub fn poly_gate_count() -> u8 {
    poly().gate_count
}
/// Begin a voice build: reset state fresh (clears any stale flag from an
/// aborted prior build), create the PolyCtrl pitch source + PolyMtof, return
/// the PolyMtof id (the `pitch` node).
pub fn poly_begin() -> u16 {
    let ctrl = alloc_node_id();
    new_node(ctrl, Kind::PolyCtrl, [Input::Const(0.0); 3]);
    let mtof = alloc_node_id();
    new_node(
        mtof,
        Kind::PolyMtof,
        [Input::Node { node: NodeId(ctrl), port: 0 }, Input::Const(0.0), Input::Const(0.0)],
    );
    let p = poly();
    p.mode = true;
    p.pitch_ctrl = ctrl;
    p.gates = [NULL_ID; MAX_GATES];
    p.gate_count = 0;
    p.vel_node = NULL_ID;
    p.slew_node = NULL_ID;
    mtof
}
/// Mono voice build: PolyCtrl → PolySlew → PolyMtof (the slew is the only
/// difference from poly_begin). Records pitch_ctrl + slew_node. Returns the
/// PolyMtof output (the `pitch` handed to the builder).
pub fn mono_begin() -> u16 {
    let ctrl = alloc_node_id();
    new_node(ctrl, Kind::PolyCtrl, [Input::Const(0.0); 3]);
    let slew = alloc_node_id();
    new_node(slew, Kind::PolySlew, [Input::Node { node: NodeId(ctrl), port: 0 }, Input::Const(0.0), Input::Const(0.0)]);
    let mtof = alloc_node_id();
    new_node(mtof, Kind::PolyMtof, [Input::Node { node: NodeId(slew), port: 0 }, Input::Const(0.0), Input::Const(0.0)]);
    let p = poly();
    p.mode = true;
    p.pitch_ctrl = ctrl;
    p.slew_node = slew;
    p.gates = [NULL_ID; MAX_GATES];
    p.gate_count = 0;
    p.vel_node = NULL_ID;
    mtof
}
/// Record a PolyAr/PolyAdsr as one of the voice's envelope gates (bounded push
/// into `gates`; `gate_count` still tracks the true total so the Wren guard
/// can reject builds that exceed `MAX_GATES`).
pub fn poly_record_gate(id: u16) {
    let p = poly();
    if (p.gate_count as usize) < MAX_GATES {
        p.gates[p.gate_count as usize] = id;
    }
    p.gate_count = p.gate_count.saturating_add(1);
}
/// Create the per-voice velocity carrier (a second PolyCtrl) and record it.
/// Returns the PolyCtrl node id (the `vel` signal handed to the builder).
pub fn poly_vel_begin() -> u16 {
    let ctrl = alloc_node_id();
    new_node(ctrl, Kind::PolyCtrl, [Input::Const(0.0); 3]);
    poly().vel_node = ctrl;
    ctrl
}
/// End a voice build: clear the flag; returns (pitch_ctrl, gates, gate_count,
/// vel_node) for the allocator.
pub fn poly_end() -> (u16, [u16; MAX_GATES], u8, u16) {
    let p = poly();
    p.mode = false;
    (p.pitch_ctrl, p.gates, p.gate_count, p.vel_node)
}
/// Returns (pitch_ctrl, slew_node, gates, gate_count, vel_node) for the mono SynthObj.
pub fn mono_end() -> (u16, u16, [u16; MAX_GATES], u8, u16) {
    let p = poly();
    p.mode = false;
    (p.pitch_ctrl, p.slew_node, p.gates, p.gate_count, p.vel_node)
}

pub fn alloc_node_id() -> u16 {
    alloc().alloc_node()
}
/// Return a node id to the free-list; used by [`free`] (the `Node.free()` binding).
pub fn free_node_id(id: u16) {
    alloc().free_node(id);
}
pub fn alloc_bus_id() -> u16 {
    alloc().alloc_bus()
}

pub fn new_node(id: u16, kind: Kind, args: [Input; 3]) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode { node: NodeId(id), kind, args });
}
/// Create a `Kind::Wavetable` node and bind it to a named static table (the
/// generated `TABLES` registry, Task 3). Used by `Node.wavetable_(table, freq)`.
pub fn new_wavetable(id: u16, table_id: u16, freq: Input) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind: Kind::Wavetable,
        args: [freq, Input::Const(0.0), Input::Const(0.0)],
    });
    host().audio_cmd(Cmd::BindTable {
        node: NodeId(id),
        src: deluge_audio_graph::node::TableSrc::Static(deluge_dsp_kernels::wavetable::TableId(
            table_id,
        )),
    });
}
/// Upload a base-cycle table (`mipgen::N` samples) to the host's pool, building
/// its band-limited mip pyramid. `None` on a host with no pool (e.g. the
/// Cmd-capture test host) or on pool exhaustion. Used by `Wavetable.from`.
pub fn upload_table(base: &[f32]) -> Option<deluge_audio_graph::PoolHandle> {
    host().upload_table(base)
}
/// Upload a multi-frame table (`nframes` base-cycles, each filled by
/// `fill_frame`) to the host's pool, building one pyramid per frame into a
/// single contiguous region. `None` on a host with no pool or on pool
/// exhaustion. Used by `Wavetable.from2d`.
pub fn upload_table_2d(
    nframes: usize,
    fill_frame: &mut dyn FnMut(usize, &mut [f32]),
) -> Option<deluge_audio_graph::PoolHandle> {
    host().upload_table_2d(nframes, fill_frame)
}
/// Create a `Kind::Wavetable` node and bind it to a pooled (dynamically
/// uploaded) table. The pooled counterpart of [`new_wavetable`]. Used by
/// `Node.wavetable_pooled_(wt, freq)`.
pub fn new_wavetable_pooled(id: u16, handle: deluge_audio_graph::PoolHandle, freq: Input) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind: Kind::Wavetable,
        args: [freq, Input::Const(0.0), Input::Const(0.0)],
    });
    host().audio_cmd(Cmd::BindTable {
        node: NodeId(id),
        src: deluge_audio_graph::node::TableSrc::Pooled(handle),
    });
}

/// Create a poly wavetable node (`Kind::PolyWt`/`PolyWtMorph`, selected by the
/// caller from the table's frame count — see `bindings_audio::poly_wt_kind`)
/// and bind it to a named static table. Mirrors `new_wavetable`, parameterized
/// by `kind` since (unlike the mono path) a poly node's Kind is fixed at
/// creation. Used by `Node.polywt_(table, freq)`.
pub fn new_polywt(id: u16, kind: Kind, table_id: u16, freq: Input) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind,
        args: [freq, Input::Const(0.0), Input::Const(0.0)],
    });
    host().audio_cmd(Cmd::BindTable {
        node: NodeId(id),
        src: deluge_audio_graph::node::TableSrc::Static(deluge_dsp_kernels::wavetable::TableId(
            table_id,
        )),
    });
}
/// Poly counterpart of `new_wavetable_pooled`: creates a `PolyWt`/`PolyWtMorph`
/// node bound to a pooled (dynamically-uploaded) table. Used by
/// `Node.polywt_pooled_(wt, freq)`.
pub fn new_polywt_pooled(id: u16, kind: Kind, handle: deluge_audio_graph::PoolHandle, freq: Input) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind,
        args: [freq, Input::Const(0.0), Input::Const(0.0)],
    });
    host().audio_cmd(Cmd::BindTable {
        node: NodeId(id),
        src: deluge_audio_graph::node::TableSrc::Pooled(handle),
    });
}

/// Allocate a zeroed effect ring buffer in the host pool (`None` on a host with
/// no pool, e.g. the Cmd-capture test host, or on exhaustion). Used by
/// `Node.delay_`.
pub fn alloc_buffer(len: usize) -> Option<deluge_audio_graph::PoolHandle> {
    host().alloc_buffer(len)
}

/// Write a single f32 at `index` into the pool region backing `h` (no-op on a
/// host with no pool, or out-of-range `index`). Used by `SampleBuffer.from`
/// to upload raw PCM verbatim, one element at a time.
pub fn pool_set(h: deluge_audio_graph::PoolHandle, index: usize, value: f32) {
    host().pool_set(h, index, value);
}

/// Create a `Kind::Delay` node (ports 0/1/2 = input/time/feedback). A bound
/// `handle` emits `NewNode` + `BindTable{Pooled}`; an unbound one (alloc
/// failed) still creates the node but skips the bind, so it renders as dry
/// passthrough instead of panicking — the same contract as
/// [`new_wavetable_pooled`].
pub fn new_delay(id: u16, handle: Option<deluge_audio_graph::PoolHandle>, input: Input, time: Input, feedback: Input) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind: Kind::Delay,
        args: [input, time, feedback],
    });
    if let Some(h) = handle {
        host().audio_cmd(Cmd::BindTable {
            node: NodeId(id),
            src: deluge_audio_graph::node::TableSrc::Pooled(h),
        });
    }
}

/// Create a pooled effect node of `kind` with `input` on port 0, binding a
/// pool ring if `handle` is `Some` (unbound → dry passthrough). Params are set
/// separately by the caller via `set_param`. Used by `Chorus`/`Flanger`.
pub fn new_pooled_node(id: u16, kind: Kind, handle: Option<deluge_audio_graph::PoolHandle>, input: Input) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind,
        args: [input, Input::Const(0.0), Input::Const(0.0)],
    });
    if let Some(h) = handle {
        host().audio_cmd(Cmd::BindTable {
            node: NodeId(id),
            src: deluge_audio_graph::node::TableSrc::Pooled(h),
        });
    }
}

pub fn set_input(id: u16, port: u8, src: Input) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::SetInput { node: NodeId(id), port, src });
}
pub fn set_param(id: u16, param: u8, value: f32) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::SetParam { node: NodeId(id), param, value });
}
pub fn gate(id: u16, on: bool) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::Gate { node: NodeId(id), on });
}
pub fn trigger(id: u16) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::Trigger { node: NodeId(id) });
}
pub fn bus_write(src: Input, bus: u16) {
    if bus == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::BusWrite { src, bus: BusId(bus) });
}
/// Emit a per-side gained bus write (`gl` → L, `gr` → R). Used by width-aware
/// routing (`write_source_to_bus` in `bindings_audio.rs`) to send a stereo
/// (width-2) source's two ports to L and R.
pub fn bus_write_gains(src: Input, bus: u16, gl: f32, gr: f32) {
    if bus == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::BusWriteGains { src, bus: BusId(bus), gl, gr });
}
pub fn set_root(bus: u16) {
    if bus == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::SetRoot { bus: BusId(bus) });
}
/// Free a node: return its id to the free-list and emit `Cmd::Free`.
/// Used by `Node.free()`.
pub fn free(id: u16) {
    if id == NULL_ID {
        return;
    }
    free_node_id(id);
    host().audio_cmd(Cmd::Free { node: NodeId(id) });
}
pub fn reset() {
    alloc().reset();
    *poly() = PolyCtx::new();
    host().audio_cmd(Cmd::Reset);
}
