//! Client-side id allocators + control-rate command emitters for the Wren audio
//! bindings. The binding owns the `NodeId` free-list and `BusId` allocation (no
//! round-trip); each emitter builds a `deluge_audio_graph::Cmd` and ships it
//! through the registered [`Host`](crate::Host). Single-threaded VM context.

use deluge_audio_graph::{BusId, Cmd, Input, Kind, NodeId, MAX_GATES, MAX_TRIGGERS};

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
    triggers: [u16; MAX_TRIGGERS], // registered sample-source node ids (first `trigger_count`)
    trigger_count: u8, // total sources registered this build (may exceed MAX_TRIGGERS → guard aborts)
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
            triggers: [NULL_ID; MAX_TRIGGERS],
            trigger_count: 0,
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
    p.triggers = [NULL_ID; MAX_TRIGGERS];
    p.trigger_count = 0;
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
    p.triggers = [NULL_ID; MAX_TRIGGERS];
    p.trigger_count = 0;
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
/// Record a registered sample-source node as one of the voice's per-lane
/// trigger targets (bounded push into `triggers`; `trigger_count` still
/// tracks the true total so a guard can reject builds that exceed
/// `MAX_TRIGGERS`). Used by Task 6's sample-source registration.
pub fn poly_record_trigger(id: u16) {
    let p = poly();
    if (p.trigger_count as usize) < MAX_TRIGGERS {
        p.triggers[p.trigger_count as usize] = id;
    }
    p.trigger_count = p.trigger_count.saturating_add(1);
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
/// vel_node, triggers, trigger_count) for the allocator.
pub fn poly_end() -> (u16, [u16; MAX_GATES], u8, u16, [u16; MAX_TRIGGERS], u8) {
    let p = poly();
    p.mode = false;
    (p.pitch_ctrl, p.gates, p.gate_count, p.vel_node, p.triggers, p.trigger_count)
}
/// Returns (pitch_ctrl, slew_node, gates, gate_count, vel_node, triggers,
/// trigger_count) for the mono SynthObj.
pub fn mono_end() -> (u16, u16, [u16; MAX_GATES], u8, u16, [u16; MAX_TRIGGERS], u8) {
    let p = poly();
    p.mode = false;
    (p.pitch_ctrl, p.slew_node, p.gates, p.gate_count, p.vel_node, p.triggers, p.trigger_count)
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

/// Create a `Kind::SamplePlayer` node (pure source — no signal inputs, all
/// three `NewNode` args are unused `Const(0.0)`). A bound `handle` emits
/// `NewNode` + `BindTable{Pooled}` + a `SetParam{param: 3}` seeding loop_end
/// to the buffer's sample count (`len`) so the default loop region covers the
/// whole sample; an unbound one (upload failed) still creates the node but
/// skips both the bind and the loop_end seed — same graceful-degrade contract
/// as [`new_delay`]/[`new_wavetable_pooled`].
pub fn new_sample_player(id: u16, handle: Option<deluge_audio_graph::PoolHandle>, len: u32) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind: Kind::SamplePlayer,
        args: [Input::Const(0.0), Input::Const(0.0), Input::Const(0.0)],
    });
    if let Some(h) = handle {
        host().audio_cmd(Cmd::BindTable {
            node: NodeId(id),
            src: deluge_audio_graph::node::TableSrc::Pooled(h),
        });
        host().audio_cmd(Cmd::SetParam { node: NodeId(id), param: 3, value: len as f32 }); // loop_end = buffer len
    }
}

/// Create a `Kind::PolySamplePlayer` node, wired to the per-voice `pitch`
/// (PolyMtof Hz) tile on port 0. Mirrors `new_polywt_pooled`'s
/// bound/unbound-`handle` shape: a bound `handle` emits `NewNode` +
/// `BindTable{Pooled}` + the zone table (`offset, len, low, high, root` per
/// zone) as `SetParam`s, per the scheme
/// `deluge_dsp_kernels::sampler::PolySamplePlayer::set_zone_field` numbers
/// (0=offset,1=len,2=low,3=high,4=root): param 0 = n_zones, param 1 =
/// loop_mode, then per zone `z` a 5-block at base `2 + z*5`. An unbound
/// `handle` (upload failed / no pool) still creates the node but skips both
/// the bind and every zone `SetParam` — same graceful-degrade contract as
/// [`new_sample_player`]/[`new_polywt_pooled`]; the node keeps
/// `PolySamplePlayer::new()`'s zero-zone default and renders silence, never
/// panics. Used by `Node.polysampleplayer_(pitch, source)` (Sa-2 Task 6).
pub fn new_poly_sample_player(
    id: u16,
    handle: Option<deluge_audio_graph::PoolHandle>,
    pitch: Input,
    zones: &[(u32, u32, u8, u8, u8)],
    loop_mode: bool,
) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind: Kind::PolySamplePlayer,
        args: [pitch, Input::Const(0.0), Input::Const(0.0)],
    });
    if let Some(h) = handle {
        host().audio_cmd(Cmd::BindTable {
            node: NodeId(id),
            src: deluge_audio_graph::node::TableSrc::Pooled(h),
        });
        let sp = |param: u8, value: f32| host().audio_cmd(Cmd::SetParam { node: NodeId(id), param, value });
        sp(0, zones.len() as f32); // n_zones
        sp(1, if loop_mode { 1.0 } else { 0.0 });
        for (z, &(off, len, lo, hi, root)) in zones.iter().enumerate() {
            let base = 2 + (z as u8) * 5;
            sp(base, off as f32);
            sp(base + 1, len as f32);
            sp(base + 2, lo as f32);
            sp(base + 3, hi as f32);
            sp(base + 4, root as f32);
        }
    }
}

/// Create a `Kind::PolyGranular` node, wired to the per-voice `pitch`
/// (PolyMtof Hz) tile on port 0 — mirrors [`new_poly_sample_player`]'s
/// bound/unbound-`handle` shape, but emits NO `SetParam`s: the kernel's
/// `PolyGranular::new()` defaults (root=60, position=0, size=50ms,
/// density=20/s, spray=0 — see `deluge_dsp_kernels::granular::PolyGranular`)
/// are used as-is, and `position=`/`size=`/`density=`/`spray=` retarget them
/// afterward via the four Wren setters (a granular `root=` setter is
/// deferred this slice, same as `new_stream_player`'s `root` param above —
/// no setter, default 60 stands). A bound `handle` emits `NewNode` +
/// `BindTable{Pooled}`; an unbound one (upload failed / no pool) still
/// creates the node but skips the bind — same graceful-degrade contract as
/// [`new_poly_sample_player`]/[`new_polywt_pooled`], never panics. Used by
/// `Node.granular_(pitch, source)` (Sa-4 Task 3).
pub fn new_poly_granular(id: u16, handle: Option<deluge_audio_graph::PoolHandle>, pitch: Input) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind: Kind::PolyGranular,
        args: [pitch, Input::Const(0.0), Input::Const(0.0)],
    });
    if let Some(h) = handle {
        host().audio_cmd(Cmd::BindTable {
            node: NodeId(id),
            src: deluge_audio_graph::node::TableSrc::Pooled(h),
        });
    }
}

/// Create a `Kind::StreamPlayer` node bound to a `VOICES*cap` ring `handle`,
/// wired to `pitch`, with `root` note (param 0). Streaming data is filled by the
/// host prefetch task (registered separately via `stream_register`).
pub fn new_stream_player(id: u16, handle: Option<deluge_audio_graph::PoolHandle>, pitch: Input, root: f32) {
    if id == NULL_ID {
        return;
    }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind: Kind::StreamPlayer,
        args: [pitch, Input::Const(0.0), Input::Const(0.0)],
    });
    if let Some(h) = handle {
        host().audio_cmd(Cmd::BindTable {
            node: NodeId(id),
            src: deluge_audio_graph::node::TableSrc::Pooled(h),
        });
    }
    host().audio_cmd(Cmd::SetParam { node: NodeId(id), param: 0, value: root });
}

/// Register a streamed node+ring with the host's prefetch (no-op host → ignored).
pub fn stream_register(id: u16, handle: Option<deluge_audio_graph::PoolHandle>, path: &str) {
    if let Some(h) = handle {
        host().stream_register(NodeId(id), h, path);
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
pub fn set_master_limit(ceiling: f32, release: f32) {
    host().audio_cmd(Cmd::SetMasterLimit { ceiling, release });
}
pub fn set_master_dcblock(cutoff_hz: f32) {
    host().audio_cmd(Cmd::SetMasterDcBlock { cutoff_hz });
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
