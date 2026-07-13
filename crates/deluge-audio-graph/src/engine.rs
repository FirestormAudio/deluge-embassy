//! The block-rendering engine. Owns the arena and the per-slot output arena and
//! evaluates nodes in topological (eval-order) order, a block at a time.
//!
//! ## Borrow model (spec §3.5)
//! The output arena is one `UnsafeCell<[[f32; BLOCK]; OUTS]>`. Each `render_block`
//! iteration first *resolves* the node's inputs — copying every source (a
//! constant, another slot's block, or a bus) into local `scratch` — and only then
//! writes the node's own slot-run. Memory-safety comes from this copy-out
//! discipline: every read is copied into `scratch` before the mutable-write
//! `unsafe` deref is created, so the write borrow never overlaps a read. This
//! holds regardless of eval order — topological order is what makes the
//! *values* correct (so a node sees its inputs' current-block outputs), not
//! what makes the borrow sound.

use core::cell::UnsafeCell;

use deluge_dsp_kernels::poly::VOICES;
use deluge_dsp_kernels::In;
use deluge_dsp_kernels::limiter::MasterLimiter;
use deluge_dsp_kernels::filter::MasterDcBlock;
use deluge_dsp_kernels::eq::MasterEq;

use crate::arena::Arena;
use crate::node::{Kind, OutView, MAX_BLOCK, MAX_INPUTS};
use crate::{BusId, Input, Node, NodeId, StereoFrame};

pub struct Engine<
    const BLOCK: usize,
    const NODES: usize,
    const OUTS: usize,
    const BUSES: usize,
    const PCAP: usize,
    const PCHUNK: usize,
> {
    arena: Arena<NODES, OUTS>,
    outs: UnsafeCell<[[f32; BLOCK]; OUTS]>,
    dt: f32,
    pub(crate) bus_l: [[f32; BLOCK]; BUSES],
    pub(crate) bus_r: [[f32; BLOCK]; BUSES],
    // Per-bus mono gain (channel fader), default 1.0. Applied to each bus's rows
    // after the write loop, before the master chain.
    pub(crate) bus_gain: [f32; BUSES],
    // Bus→bus sends: (from, to, gain), applied after the node-write loop,
    // descending `from` id (rule: from > to). Stereo-preserving, pre-fader.
    bus_sends: [Option<(BusId, BusId, f32)>; NODES],
    bus_sends_len: usize,
    // Stereo line-in for the current block, filled by `render` from its `input`
    // arg and read by `render_block` for `Kind::Input` nodes.
    pub(crate) in_l: [f32; BLOCK],
    pub(crate) in_r: [f32; BLOCK],
    // Bus rendered to the audio output; set via `set_root`, read in `render`.
    pub(crate) root: Option<BusId>,
    // Pending bus writes, re-applied every `render` (P0: no persistent routing
    // table yet — see spec §3.5 / bus.rs).
    writes: [Option<(Input, BusId, f32, f32)>; NODES],
    writes_len: usize,
    pool: crate::pool::Pool<PCAP, PCHUNK>,
    // Per-`StreamPlayer`-node fill cursors (produced by the prefetch task via
    // `Cmd::StreamFill`, consumed at render). Keyed by node index.
    stream_state: [Option<crate::stream::StreamCursors>; NODES],
    // Opt-in master limiter on the root bus, applied at the render seam before
    // the output clamp. `None` = disabled (render path byte-unchanged).
    master_limiter: Option<MasterLimiter>,
    // Opt-in master DC-blocker on the root bus, applied at the render seam BEFORE
    // the limiter. `None` = disabled (render path byte-unchanged).
    master_dcblock: Option<MasterDcBlock>,
    // Opt-in master EQ on the root bus, applied at the render seam between the
    // DC-block and the limiter. `None` = disabled (render path byte-unchanged).
    master_eq: Option<MasterEq>,
}

impl<
    const BLOCK: usize,
    const NODES: usize,
    const OUTS: usize,
    const BUSES: usize,
    const PCAP: usize,
    const PCHUNK: usize,
> Engine<BLOCK, NODES, OUTS, BUSES, PCAP, PCHUNK>
{
    pub fn new(sample_rate: f32) -> Self {
        assert!(BLOCK <= MAX_BLOCK, "BLOCK exceeds MAX_BLOCK");
        Engine {
            arena: Arena::new(),
            outs: UnsafeCell::new([[0.0; BLOCK]; OUTS]),
            dt: 1.0 / sample_rate,
            bus_l: [[0.0; BLOCK]; BUSES],
            bus_r: [[0.0; BLOCK]; BUSES],
            bus_gain: [1.0; BUSES],
            bus_sends: [None; NODES],
            bus_sends_len: 0,
            in_l: [0.0; BLOCK],
            in_r: [0.0; BLOCK],
            root: None,
            writes: [None; NODES],
            writes_len: 0,
            pool: crate::pool::Pool::new(),
            stream_state: [None; NODES],
            master_limiter: None,
            master_dcblock: None,
            master_eq: None,
        }
    }

    pub fn pool_alloc(&mut self, len: usize) -> Option<crate::pool::PoolHandle> {
        self.pool.alloc(len)
    }
    pub fn pool_free(&mut self, h: crate::pool::PoolHandle) {
        self.pool.free(h)
    }
    pub fn pool_slice(&self, h: crate::pool::PoolHandle) -> &[f32] {
        self.pool.slice(h)
    }
    pub fn pool_slice_mut(&mut self, h: crate::pool::PoolHandle) -> &mut [f32] {
        self.pool.slice_mut(h)
    }

    pub fn create(&mut self, id: NodeId, kind: crate::node::Kind) -> bool {
        self.arena.create(id, kind)
    }

    pub fn node_input_mut(&mut self, id: NodeId, port: u8) -> Option<&mut Input> {
        self.arena.node_mut(id)?.input_mut(port)
    }

    /// Apply one control-rate `Cmd`, mutating the arena/engine state it names.
    pub fn apply(&mut self, cmd: crate::cmd::Cmd) {
        use crate::cmd::Cmd;
        match cmd {
            Cmd::Nop => {}
            Cmd::NewNode { node, kind, args } => {
                if self.arena.create(node, kind) {
                    if let Some(n) = self.arena.node_mut(node) {
                        for p in 0..crate::cmd::MAX_ARGS {
                            if let Some(slot) = n.input_mut(p as u8) {
                                *slot = args[p];
                            }
                        }
                    }
                }
            }
            Cmd::SetInput { node, port, src } => {
                if let Some(n) = self.arena.node_mut(node) {
                    if let Some(slot) = n.input_mut(port) {
                        *slot = src;
                    }
                }
            }
            Cmd::SetParam { node, param, value } => {
                if let Some(n) = self.arena.node_mut(node) {
                    n.set_param(param, value);
                }
            }
            Cmd::BindTable { node, src } => {
                if let Some(n) = self.arena.node_mut(node) {
                    n.bind_table(src);
                }
            }
            Cmd::Gate { node, on } => {
                if let Some(n) = self.arena.node_mut(node) {
                    n.gate(on);
                }
            }
            Cmd::Trigger { node } => {
                if let Some(n) = self.arena.node_mut(node) {
                    n.trigger();
                }
            }
            Cmd::GateVoice { node, voice, on } => {
                if let Some(n) = self.arena.node_mut(node) { n.gate_voice(voice as usize, on); }
            }
            Cmd::TriggerVoice { node, voice } => {
                if let Some(n) = self.arena.node_mut(node) { n.trigger_voice(voice as usize); }
            }
            Cmd::StreamFill { node, voice, fill_lo, fill_hi, total } => {
                let idx = node.0 as usize;
                if idx < NODES && (voice as usize) < VOICES {
                    let sc = self.stream_state[idx]
                        .get_or_insert_with(crate::stream::StreamCursors::new);
                    sc.total = total;
                    sc.fill[voice as usize] = (fill_lo, fill_hi);
                }
            }
            Cmd::BusWrite { src, bus } => self.bus_write(src, bus),
            Cmd::BusWriteGains { src, bus, gl, gr } => self.bus_write_gains(src, bus, gl, gr),
            Cmd::SetRoot { bus } => self.set_root(bus),
            Cmd::BusGain { bus, gain } => self.set_bus_gain(bus, gain),
            Cmd::BusSend { from, to, gain } => self.bus_send(from, to, gain),
            Cmd::SetMasterLimit { ceiling, release } => match &mut self.master_limiter {
                Some(lim) => { lim.set_ceiling(ceiling); lim.set_release(release); }
                None => self.master_limiter = Some(MasterLimiter::new(ceiling, release)),
            },
            Cmd::SetMasterDcBlock { cutoff_hz } => match &mut self.master_dcblock {
                Some(dc) => dc.set_cutoff(cutoff_hz, self.dt),
                None => self.master_dcblock = Some(MasterDcBlock::new(cutoff_hz, self.dt)),
            },
            Cmd::SetMasterEq { freq, gain_db, q, eq_type } => match &mut self.master_eq {
                Some(eq) => eq.set_params(freq, gain_db, q, eq_type),
                None => self.master_eq = Some(MasterEq::new(freq, gain_db, q, eq_type)),
            },
            Cmd::Free { node } => {
                // Free a pooled table region (if bound) BEFORE reclaiming the
                // node's arena slot: `table_src()` reads through the node,
                // which must still be live.
                if let Some(n) = self.arena.node_mut(node) {
                    if let Some(crate::node::TableSrc::Pooled(h)) = n.table_src() {
                        self.pool.free(h);
                    }
                }
                let idx = node.0 as usize;
                if idx < NODES { self.stream_state[idx] = None; }
                self.arena.free(node);
                // Invalidate this node's bus writes so a reused id inherits no
                // stale routing (IO-2a). Const/Bus-sourced writes are untouched.
                for w in self.writes.iter_mut() {
                    if let Some((Input::Node { node: n, .. }, ..)) = w {
                        if *n == node {
                            *w = None;
                        }
                    }
                }
            }
            Cmd::Reset => {
                self.arena.reset();
                self.writes = [None; NODES];
                self.writes_len = 0;
                self.root = None;
                self.stream_state = [None; NODES];
                self.master_limiter = None;
                self.master_dcblock = None;
                self.master_eq = None;
                self.bus_gain = [1.0; BUSES];
                self.bus_sends = [None; NODES];
                self.bus_sends_len = 0;
                self.bus_l = [[0.0; BLOCK]; BUSES];
                self.bus_r = [[0.0; BLOCK]; BUSES];
            }
        }
    }

    /// Evaluate every live node in eval order into the output arena.
    pub fn render_block(&mut self) {
        // Snapshot eval order so we don't borrow the arena across the loop.
        let mut order = [0u16; NODES];
        let live = {
            let eo = self.arena.eval_order();
            order[..eo.len()].copy_from_slice(eo);
            eo.len()
        };

        for k in 0..live {
            let id = NodeId(order[k]);
            let (base, kind, width, inputs, table_src) = {
                let n = self.arena.node(id).expect("eval-order node exists");
                (n.out_base as usize, n.kind, Node::out_width(n.kind), n.inputs_snapshot(), n.table_src())
            };

            // ── Resolve inputs into scratch (all reads copied out first) ──
            let mut scratch = [[0.0f32; BLOCK]; MAX_INPUTS];
            let mut poly_scratch = [[[0.0f32; BLOCK]; VOICES]; 3];
            {
                // SAFETY: read-only view of the output arena; no writer is live.
                let arr = unsafe { &*self.outs.get() };
                for (p, row) in scratch.iter_mut().enumerate() {
                    match inputs[p] {
                        Input::Const(v) => row.fill(v),
                        Input::Node { node, port } => match self.arena.out_base(node) {
                            Some(sbase) if sbase + (port as usize) < OUTS => {
                                *row = arr[sbase + port as usize]
                            }
                            _ => *row = [0.0; BLOCK], // dangling ref or out-of-range port → silence (never panic, never slot-0 crosstalk)
                        },
                        Input::Bus(bus) => {
                            let b = bus.0 as usize;
                            for i in 0..BLOCK {
                                row[i] = self.bus_l[b][i] + self.bus_r[b][i];
                            }
                        }
                    }
                }

                for j in 0..Node::poly_in_count(kind) {
                    // Ports 0..k are poly edges. A poly (VOICES-wide) source's
                    // rows are copied verbatim (row-copy preserves the
                    // interleaved layout). A mono (width-1) source instead has
                    // its single output row splatted to every voice lane —
                    // copying VOICES rows from a width-1 producer would read
                    // adjacent-node garbage/silence past its one live slot.
                    match inputs[j] {
                        Input::Const(v) => {
                            for lane in &mut poly_scratch[j] { lane.fill(v); }
                        }
                        Input::Node { node, .. } => match self.arena.out_base_and_kind(node) {
                            Some((sbase, src_kind)) if Node::out_width(src_kind) == 1 && sbase < OUTS => {
                                for v in 0..VOICES { poly_scratch[j][v] = arr[sbase]; }
                            }
                            Some((sbase, _)) if sbase + VOICES <= OUTS => {
                                for v in 0..VOICES { poly_scratch[j][v] = arr[sbase + v]; }
                            }
                            _ => { for v in 0..VOICES { poly_scratch[j][v] = [0.0; BLOCK]; } }
                        },
                        _ => { for v in 0..VOICES { poly_scratch[j][v] = [0.0; BLOCK]; } }
                    }
                }
            }
            let ins = [
                In::A(&scratch[0][..]),
                In::A(&scratch[1][..]),
                In::A(&scratch[2][..]),
            ];

            // ── Write this node's ports ──
            // SAFETY (deref soundness): every read for this iteration was already
            // copied into `scratch` above, before this mutable deref exists, so
            // the write region `[base, base+width)` cannot alias a live read —
            // regardless of eval order (wrong order would be a correctness bug,
            // not UB).
            // SAFETY (disjoint borrow): the raw deref of `self.outs` is not
            // tracked against `self`, so the borrow checker still allows the
            // following `self.arena.node_mut(id)` call (a borrow of the
            // disjoint `arena` field) to coexist with `arr`.
            let arr = unsafe { &mut *self.outs.get() };
            if Node::is_poly(kind) {
                // Poly path: voice-interleaved tile in/out, isolated dispatch.
                let count = Node::poly_in_count(kind);
                let poly_in: [Option<&[f32]>; 3] = [
                    if count > 0 { Some(poly_scratch[0].as_flattened()) } else { None },
                    if count > 1 { Some(poly_scratch[1].as_flattened()) } else { None },
                    if count > 2 { Some(poly_scratch[2].as_flattened()) } else { None },
                ];
                let out = arr[base..base + width].as_flattened_mut(); // width*BLOCK
                // Resolve a pooled node's region MUTABLY before the node's
                // `&mut` borrow below, exactly as the mono path does at its
                // call site below — only `PolyWt`/`PolyWtMorph` read this;
                // every other poly kind ignores it.
                let pool_region: Option<&mut [f32]> = match table_src {
                    Some(crate::node::TableSrc::Pooled(h)) => Some(self.pool.slice_mut(h)),
                    _ => None,
                };
                // Stream fill cursors (a disjoint `Engine` field from `pool`/`arena`).
                let stream = {
                    let sidx = id.0 as usize;
                    if sidx < NODES { self.stream_state[sidx].as_ref() } else { None }
                };
                if let Some(n) = self.arena.node_mut(id) {
                    n.poly_process(&ins, poly_in, self.dt, out, pool_region, stream);
                }
            } else if kind == Kind::Input {
                // Stereo line-in: copy engine input rows into port0 (L) / port1 (R).
                // `self.in_l`/`in_r` are disjoint fields from `self.outs` (raw-ptr
                // borrow), same disjoint-field discipline as bus access.
                for i in 0..BLOCK {
                    arr[base][i] = self.in_l[i];
                    arr[base + 1][i] = self.in_r[i];
                }
            } else {
                let mut view = OutView::from_arena::<OUTS, BLOCK>(arr, base, width);
                // Resolve a pooled node's region MUTABLY (delay lines write it;
                // wavetables reborrow it immutably in `process_resolved`) BEFORE
                // the node's `&mut` borrow below — `self.pool` and `self.arena` are
                // separate fields of `Engine`, so the borrow checker tracks them
                // independently as long as each is accessed as a direct field
                // projection (not through a whole-`&mut self` helper method).
                let pool_region: Option<&mut [f32]> = match table_src {
                    Some(crate::node::TableSrc::Pooled(h)) => Some(self.pool.slice_mut(h)),
                    _ => None,
                };
                if let Some(n) = self.arena.node_mut(id) {
                    n.process_resolved(&ins, self.dt, &mut view, pool_region);
                }
            }
        }
    }

    /// A `StreamPlayer` voice's playback read-cursor (⌊pos⌋). `None` if `node`
    /// doesn't exist / isn't a `StreamPlayer` / `voice` is out of range. The
    /// prefetch task polls this to trail playback.
    pub fn stream_read_cursor(&self, node: NodeId, voice: usize) -> Option<u64> {
        self.arena.node(node)?.stream_read_cursor(voice)
    }

    /// Test/inspection accessor: a node's rendered output port.
    pub fn node_output(&self, id: NodeId, port: u8) -> &[f32] {
        let base = self.arena.out_base(id).expect("node exists");
        // SAFETY: shared read; no writer is live outside `render_block`.
        let arr = unsafe { &*self.outs.get() };
        &arr[base + port as usize][..]
    }

    /// Record a center (L = R) bus write — unchanged mono behavior.
    pub fn bus_write(&mut self, src: Input, bus: BusId) {
        self.bus_write_gains(src, bus, 1.0, 1.0);
    }

    /// Record a bus write with per-side gains (`gl` → L, `gr` → R). A stereo
    /// source routes as two of these: `(port0, 1, 0)` and `(port1, 0, 1)`.
    pub fn bus_write_gains(&mut self, src: Input, bus: BusId, gl: f32, gr: f32) {
        // Reuse a freed (`None`) slot first so free/patch cycles don't leak
        // slots; otherwise append. No holes exist without a prior `Free`, so a
        // fresh engine appends in the same order as before (byte-identical).
        for w in 0..self.writes_len {
            if self.writes[w].is_none() {
                self.writes[w] = Some((src, bus, gl, gr));
                return;
            }
        }
        if self.writes_len < self.writes.len() {
            self.writes[self.writes_len] = Some((src, bus, gl, gr));
            self.writes_len += 1;
        }
    }

    /// Select which bus is copied to the audio output by `render`.
    pub fn set_root(&mut self, bus: BusId) {
        self.root = Some(bus);
    }

    /// Set a bus's mono gain (bounds-guarded; out-of-range is a no-op).
    pub fn set_bus_gain(&mut self, bus: BusId, gain: f32) {
        let b = bus.0 as usize;
        if b < BUSES {
            self.bus_gain[b] = gain;
        }
    }

    /// Record a bus→bus send (reuses a freed slot first, else appends).
    pub fn bus_send(&mut self, from: BusId, to: BusId, gain: f32) {
        for s in 0..self.bus_sends_len {
            if self.bus_sends[s].is_none() {
                self.bus_sends[s] = Some((from, to, gain));
                return;
            }
        }
        if self.bus_sends_len < self.bus_sends.len() {
            self.bus_sends[self.bus_sends_len] = Some((from, to, gain));
            self.bus_sends_len += 1;
        }
    }

    /// Render one block: clear buses, evaluate nodes, apply pending bus
    /// writes, then copy the root bus into `out`, clamped to `[-1, 1]`.
    pub fn render(&mut self, out: &mut [StereoFrame], input: &[StereoFrame]) {
        // Fill the stereo line-in rows from `input`; frames beyond `input.len()`
        // (including an empty slice) read as silence — no panic on any length.
        for i in 0..BLOCK {
            match input.get(i) {
                Some(f) => { self.in_l[i] = f.l; self.in_r[i] = f.r; }
                None => { self.in_l[i] = 0.0; self.in_r[i] = 0.0; }
            }
        }
        // Evaluate nodes FIRST, so an `Input::Bus` node-read sees the PREVIOUS
        // block's bus content (one-block-delayed) rather than a freshly-zeroed
        // bus — this is what lets a bus feed an effect node (aux returns).
        self.render_block();
        // NOW zero the buses and refill them from this block's node outputs.
        for b in 0..BUSES {
            self.bus_l[b] = [0.0; BLOCK];
            self.bus_r[b] = [0.0; BLOCK];
        }
        // Apply bus writes (per-side gains; mono center = (1,1)).
        let arr = unsafe { &*self.outs.get() };
        for w in 0..self.writes_len {
            if let Some((src, bus, gl, gr)) = self.writes[w] {
                let b = bus.0 as usize;
                for i in 0..BLOCK {
                    let v = match src {
                        Input::Const(c) => c,
                        Input::Node { node, port } => match self.arena.out_base(node) {
                            Some(base) if base + (port as usize) < OUTS => arr[base + port as usize][i],
                            _ => 0.0, // dangling ref or out-of-range port → contributes silence
                        },
                        Input::Bus(_) => 0.0, // bus→bus not in P0
                    };
                    self.bus_l[b][i] += v * gl;
                    self.bus_r[b][i] += v * gr;
                }
            }
        }
        // Bus→bus sends (stereo-preserving, pre-fader): fold source buses into
        // targets in descending `from` id (rule: from > to, master=0 = sink) so a
        // source is fully filled before it feeds a lower bus. No-op when none.
        for from in (0..BUSES).rev() {
            for s in 0..self.bus_sends_len {
                if let Some((f, t, g)) = self.bus_sends[s] {
                    let (fi, ti) = (f.0 as usize, t.0 as usize);
                    if fi == from && fi < BUSES && ti < BUSES && fi != ti {
                        for i in 0..BLOCK {
                            self.bus_l[ti][i] += self.bus_l[fi][i] * g;
                            self.bus_r[ti][i] += self.bus_r[fi][i] * g;
                        }
                    }
                }
            }
        }
        // Per-bus gain (default 1.0 = no-op, byte-identical): fader before the master chain.
        for b in 0..BUSES {
            let g = self.bus_gain[b];
            if g != 1.0 {
                for i in 0..BLOCK {
                    self.bus_l[b][i] *= g;
                    self.bus_r[b][i] *= g;
                }
            }
        }
        // Master chain (opt-in): DC-block → limiter, on the root bus before the clamp.
        if let Some(root) = self.root {
            let b = root.0 as usize;
            if let Some(dc) = &mut self.master_dcblock {
                dc.process(&mut self.bus_l[b], &mut self.bus_r[b]);
            }
            if let Some(eq) = &mut self.master_eq {
                eq.process(&mut self.bus_l[b], &mut self.bus_r[b], self.dt);
            }
            if let Some(lim) = &mut self.master_limiter {
                lim.process(&mut self.bus_l[b], &mut self.bus_r[b], self.dt);
            }
        }
        // Copy root bus to output, clamped.
        let n = out.len().min(BLOCK);
        if let Some(root) = self.root {
            let b = root.0 as usize;
            for i in 0..n {
                out[i].l = self.bus_l[b][i].clamp(-1.0, 1.0);
                out[i].r = self.bus_r[b][i].clamp(-1.0, 1.0);
            }
        } else {
            for i in 0..n {
                out[i] = StereoFrame::default();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node::{Kind, TableSrc};
    use crate::{Cmd, Input, NodeId};

    type E = Engine<16, 8, 8, 4, 45056, 2048>;

    #[test]
    fn stream_player_renders_and_cursors() {
        // Two VOICES-wide (8-row) nodes need OUTS room for 16 output rows.
        type SE = Engine<16, 8, 128, 4, 45056, 2048>;
        let mut e = SE::new(16.0);
        let cap = 64usize;
        let node = NodeId(0);
        let h = e.pool_alloc(VOICES * cap).expect("ring pool");
        e.create(node, Kind::StreamPlayer);
        e.apply(Cmd::BindTable { node, src: TableSrc::Pooled(h) });
        e.apply(Cmd::SetParam { node, param: 0, value: 60.0 }); // root note
        // Mock prefetch: fill voice 0's sub-ring [0..cap) so sample `a` == a.
        {
            let region = e.pool_slice_mut(h);
            for a in 0..cap { region[a] = a as f32; }
        }
        e.apply(Cmd::StreamFill { node, voice: 0, fill_lo: 0, fill_hi: cap as u64, total: cap as u64 });
        e.apply(Cmd::TriggerVoice { node, voice: 0 });
        // pitch == root Hz (mtof(60) ≈ 261.63) → rate ≈ 1.0.
        *e.node_input_mut(node, 0).unwrap() = Input::Const(261.625_58);
        e.render_block();
        assert!(e.stream_read_cursor(node, 0).unwrap() > 0, "read cursor advanced on a full window");

        // Underrun: a short window (fill_hi = 4) reads a couple samples then holds.
        let node2 = NodeId(1);
        let h2 = e.pool_alloc(VOICES * cap).expect("ring pool 2");
        e.create(node2, Kind::StreamPlayer);
        e.apply(Cmd::BindTable { node: node2, src: TableSrc::Pooled(h2) });
        e.apply(Cmd::SetParam { node: node2, param: 0, value: 60.0 });
        { let r = e.pool_slice_mut(h2); for a in 0..cap { r[a] = a as f32; } }
        e.apply(Cmd::StreamFill { node: node2, voice: 0, fill_lo: 0, fill_hi: 4, total: 1000 });
        e.apply(Cmd::TriggerVoice { node: node2, voice: 0 });
        *e.node_input_mut(node2, 0).unwrap() = Input::Const(261.625_58);
        e.render_block();
        assert!(e.stream_read_cursor(node2, 0).unwrap() <= 2, "held under underrun");

        // No panic on out-of-range node / voice.
        e.apply(Cmd::StreamFill { node: NodeId(999), voice: 99, fill_lo: 0, fill_hi: 0, total: 0 });
        e.apply(Cmd::StreamFill { node, voice: 99, fill_lo: 0, fill_hi: 0, total: 0 });
        assert!(e.stream_read_cursor(NodeId(999), 0).is_none());
        assert!(e.stream_read_cursor(node, 99).is_none());
    }

    #[test]
    fn single_saw_node_renders() {
        let mut e = E::new(16.0); // sr so 4 Hz → 0.25/sample
        e.create(NodeId(0), Kind::Saw);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(4.0);
        e.render_block();
        let out = e.node_output(NodeId(0), 0);
        // Kernel-agnostic on purpose: this test verifies graph wiring (a
        // Kind::Saw node renders to its output port), not the oscillator's
        // exact samples. The naive vs. band-limited kernel shape is covered
        // by deluge-dsp-kernels; here we only check the output is a finite,
        // saw-like signal that swings through both polarities within the
        // expected [-1, 1] range (band-limiting can reduce peak amplitude
        // relative to the naive ramp, so thresholds are intentionally loose).
        assert!(out.iter().all(|s| s.is_finite() && *s >= -1.1 && *s <= 1.1));
        assert!(out.iter().cloned().fold(f32::MAX, f32::min) < -0.3);
        assert!(out.iter().cloned().fold(f32::MIN, f32::max) > 0.3);
    }

    #[test]
    fn pink_brown_nodes_render_bounded() {
        for k in [Kind::PinkNoise, Kind::BrownNoise] {
            let mut e = E::new(48_000.0);
            e.create(NodeId(0), k);
            e.render_block();
            let out = e.node_output(NodeId(0), 0);
            assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.0));
            assert!(out.iter().any(|&s| s != 0.0));
        }
    }

    #[test]
    fn sync_saw_node_renders_bounded() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::SyncSaw);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(220.0); // master
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(660.0); // slave
        e.render_block();
        let out = e.node_output(NodeId(0), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
        assert!(out.iter().any(|&s| s != 0.0));
    }

    #[test]
    fn chain_saw_times_const_scales() {
        // node0 = saw(4Hz); node1 = mul(node0, 0.5)
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Saw);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(4.0);
        e.create(NodeId(1), Kind::Mul);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        *e.node_input_mut(NodeId(1), 1).unwrap() = Input::Const(0.5);
        e.render_block();
        let saw = e.node_output(NodeId(0), 0)[1]; // -0.5
        let scaled = e.node_output(NodeId(1), 0)[1];
        assert!((scaled - saw * 0.5).abs() < 1e-6);
    }

    #[test]
    fn two_nodes_sum_into_master_bus() {
        // node0 = const 0.3 (via Add of const+const), node1 = const 0.4; both → bus0.
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.3);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.create(NodeId(1), Kind::Add);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Const(0.4);
        *e.node_input_mut(NodeId(1), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.bus_write(Input::Node { node: NodeId(1), port: 0 }, BusId(0));
        e.set_root(BusId(0));

        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.7).abs() < 1e-6);
        assert!((out[0].r - 0.7).abs() < 1e-6);
    }

    #[test]
    fn render_clamps_to_unit_range() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(5.0);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 1.0).abs() < 1e-6); // clamped
    }

    #[test]
    fn split2_feeds_two_consumers_from_two_ports_single_compute() {
        // src const 0.6 → split2 (ports 0,1); consumerA reads port0, consumerB port1.
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add); // produce a constant 0.6 source
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.6);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.create(NodeId(1), Kind::Split2);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        // consumerA = mul(port0, 2)
        e.create(NodeId(2), Kind::Mul);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        *e.node_input_mut(NodeId(2), 1).unwrap() = Input::Const(2.0);
        // consumerB = mul(port1, 3)
        e.create(NodeId(3), Kind::Mul);
        *e.node_input_mut(NodeId(3), 0).unwrap() = Input::Node { node: NodeId(1), port: 1 };
        *e.node_input_mut(NodeId(3), 1).unwrap() = Input::Const(3.0);

        e.render_block();
        assert!((e.node_output(NodeId(2), 0)[0] - 1.2).abs() < 1e-6); // 0.6*2
        assert!((e.node_output(NodeId(3), 0)[0] - 1.8).abs() < 1e-6); // 0.6*3
    }

    #[test]
    fn dangling_and_oob_refs_render_silence_not_panic() {
        // node0 = a real 1-output saw (out_base 0, width 1).
        // node1 sums two bad references into bus0:
        //   - Input::Node { node: NodeId(50), port: 0 } — node never created (dangling).
        //   - Input::Node { node: NodeId(0), port: 7 }  — real node, out-of-range port.
        // Both must resolve to silence (0.0), never panic, never read an
        // unrelated slot.
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Saw);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(4.0);

        e.bus_write(Input::Node { node: NodeId(50), port: 0 }, BusId(0));
        e.bus_write(Input::Node { node: NodeId(0), port: 7 }, BusId(0));
        e.set_root(BusId(0));

        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil); // must not panic

        for f in out.iter() {
            assert!(f.l.is_finite() && f.l.abs() <= 1.0);
            assert!(f.r.is_finite() && f.r.abs() <= 1.0);
        }
        // Both bus-write sources are bad refs, so the bus sum is exactly 0.
        assert_eq!(out[0].l, 0.0);
        assert_eq!(out[0].r, 0.0);

        // Also exercise the render_block (Input::Node input-resolve) path
        // directly: a consumer node reading a dangling node ref must get a
        // zero row, not a panic or slot-0 crosstalk.
        let mut e2 = E::new(16.0);
        e2.create(NodeId(0), Kind::Saw);
        *e2.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(4.0);
        e2.create(NodeId(1), Kind::Add);
        *e2.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(50), port: 0 };
        *e2.node_input_mut(NodeId(1), 1).unwrap() = Input::Node { node: NodeId(0), port: 7 };
        e2.render_block(); // must not panic
        let consumer_out = e2.node_output(NodeId(1), 0);
        assert!(consumer_out.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn setparam_feedback_renders_bounded() {
        // Render the same sine oscillator config TWICE: once with feedback=0
        // (no SetParam call), once with feedback=0.8 (via Cmd::SetParam).
        // Assert the two outputs DIFFER, proving feedback modulation actually
        // changed the signal. Keep existing finite/bounded checks on feedback=0.8.

        // ── Render with feedback=0 (no SetParam) ──
        let mut e0 = E::new(48_000.0);
        e0.create(NodeId(0), Kind::Sine);
        *e0.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(2_000.0);
        // No SetParam call → default feedback=0
        e0.render_block();
        let out_no_feedback = e0.node_output(NodeId(0), 0).to_vec();

        // ── Render with feedback=0.8 (via Cmd::SetParam) ──
        let mut e1 = E::new(48_000.0);
        e1.create(NodeId(0), Kind::Sine);
        *e1.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(2_000.0);
        e1.apply(Cmd::SetParam { node: NodeId(0), param: 0, value: 0.8 });
        e1.render_block();
        let out_with_feedback = e1.node_output(NodeId(0), 0);

        // ── Assert feedback=0.8 output is finite and bounded ──
        assert!(out_with_feedback.iter().all(|s| s.is_finite() && s.abs() <= 4.0));
        assert!(out_with_feedback.iter().any(|&s| s != 0.0)); // feedback sine still oscillates

        // ── Assert the two outputs DIFFER (feedback changed the waveform) ──
        // A sine with feedback=0.8 must visibly differ from feedback=0.
        // We check that at least one sample differs beyond a small epsilon.
        let epsilon = 1e-5;
        assert!(
            out_no_feedback
                .iter()
                .zip(out_with_feedback.iter())
                .any(|(a, b)| (a - b).abs() > epsilon),
            "feedback=0 and feedback=0.8 outputs must differ"
        );
    }

    #[test]
    fn engine_pool_alloc_fill_read_free() {
        let mut e = E::new(48_000.0);
        let h = e.pool_alloc(16).expect("alloc");
        e.pool_slice_mut(h).fill(0.25);
        assert!(e.pool_slice(h).iter().all(|&x| x == 0.25));
        e.pool_free(h);
        // After free, a full-capacity alloc succeeds (region reclaimed).
        assert!(e.pool_alloc(16).is_some());
    }

    #[test]
    fn wavetable_node_renders_bounded_nonsilent() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Wavetable);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(220.0);
        e.apply(Cmd::BindTable { node: NodeId(0), src: TableSrc::Static(deluge_dsp_kernels::wavetable::TableId(0)) });
        e.render_block();
        let out = e.node_output(NodeId(0), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
        assert!(out.iter().any(|&s| s != 0.0));
    }

    #[test]
    fn pooled_wavetable_renders_and_frees() {
        let mut e = E::new(48_000.0);
        // Build a saw pyramid directly into the pool (mimics upload_table),
        // via the same flat-compact builder the runtime upload path uses.
        let n = mipgen::N;
        let compact_len = deluge_dsp_kernels::wavetable::COMPACT_LEN;
        let h = e.pool_alloc(compact_len).expect("pool room");
        let mut base = [0.0f32; mipgen::N];
        for (i, s) in base.iter_mut().enumerate() { *s = 2.0 * (i as f32 / n as f32) - 1.0; }
        mipgen::build_pyramid_flat_compact(&base, e.pool_slice_mut(h));
        e.create(NodeId(0), Kind::Wavetable);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(220.0);
        e.apply(Cmd::BindTable { node: NodeId(0), src: TableSrc::Pooled(h) });
        e.render_block();
        let out = e.node_output(NodeId(0), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
        assert!(out.iter().any(|&s| s != 0.0));
        // Free the node → pool region reclaimed.
        e.apply(Cmd::Free { node: NodeId(0) });
        // First-fit: a genuine free lets the next same-size alloc reclaim the exact
        // region → same handle. This fails if Cmd::Free didn't actually pool.free(h).
        assert_eq!(e.pool_alloc(compact_len), Some(h));
    }

    #[test]
    fn pool_exhaustion_returns_none_not_panic() {
        // `E`'s pool is PCAP=45056, PCHUNK=2048 → 22 chunks total. One
        // wavetable pyramid is the flat-compact `COMPACT_LEN`=6208 f32,
        // which rounds up to 4 chunks (2048*3=6144 < 6208 <= 2048*4=8192), so
        // exactly 5 pyramids fit (20 chunks) and a 6th (needing 4 more, only
        // 2 free) must degrade to `None`, never panic — the caller (Wren
        // `Wavetable.from`) is expected to leave the table unbound in that case.
        let mut e = E::new(48_000.0);
        let want = deluge_dsp_kernels::wavetable::COMPACT_LEN;
        let mut handles = [None; 5];
        for (i, slot) in handles.iter_mut().enumerate() {
            *slot = Some(e.pool_alloc(want).unwrap_or_else(|| panic!("pyramid {i} should fit")));
        }
        assert!(e.pool_alloc(want).is_none(), "6th pyramid must not fit a 5-pyramid pool");
        // Pool is not corrupted by the failed alloc: existing handles still work.
        let h1 = handles[0].unwrap();
        let h2 = handles[1].unwrap();
        e.pool_slice_mut(h1).fill(0.5);
        e.pool_slice_mut(h2).fill(0.75);
        assert!(e.pool_slice(h1).iter().all(|&x| x == 0.5));
        assert!(e.pool_slice(h2).iter().all(|&x| x == 0.75));
    }

    #[test]
    fn pooled_wavetable_wrong_sized_region_renders_silence_not_panic() {
        // A `TableSrc::Pooled` handle whose region isn't exactly `COMPACT_LEN`
        // long (e.g. the upload path allocated the wrong size, or a stale
        // handle from a different table) must never be sliced by the
        // compact-layout helpers — `process_resolved`'s exact-size guard
        // (`region.len() == COMPACT_LEN`) should just leave the output
        // untouched (silence in a freshly-zeroed arena slot). This is the
        // reachable degrade path for a "bad pool region": a
        // legitimately-obtained `PoolHandle` (via the public `pool_alloc`)
        // whose length happens to be wrong, since `PoolHandle`'s fields are
        // private to `pool.rs` and can't be hand-forged from `engine::tests`.
        let mut e = E::new(48_000.0);
        let bad = e.pool_alloc(64).expect("small alloc fits"); // 64 != COMPACT_LEN (6208)
        e.create(NodeId(0), Kind::Wavetable);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(220.0);
        e.apply(Cmd::BindTable { node: NodeId(0), src: TableSrc::Pooled(bad) });
        e.render_block(); // must not panic
        let out = e.node_output(NodeId(0), 0);
        assert!(out.iter().all(|&s| s == 0.0), "wrong-sized pool region must render silence: {out:?}");
    }

    #[test]
    fn pooled_morph_wavetable_renders() {
        let mut e = E::new(48_000.0);
        let cl = deluge_dsp_kernels::wavetable::COMPACT_LEN;
        let h = e.pool_alloc(2 * cl).expect("pool");
        // build 2 frames (saw, square) into the region
        let mut saw = [0.0f32; mipgen::N]; let mut sq = [0.0f32; mipgen::N];
        for i in 0..mipgen::N { saw[i]=2.0*(i as f32/mipgen::N as f32)-1.0; sq[i]=if i<mipgen::N/2 {1.0} else {-1.0}; }
        { let r = e.pool_slice_mut(h);
          mipgen::build_pyramid_flat_compact(&saw, &mut r[..cl]);
          mipgen::build_pyramid_flat_compact(&sq, &mut r[cl..]); }
        e.create(NodeId(0), Kind::Wavetable);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(220.0); // freq
        *e.node_input_mut(NodeId(0), 2).unwrap() = Input::Const(0.5);   // position (port 2)
        e.apply(Cmd::BindTable { node: NodeId(0), src: TableSrc::Pooled(h) });
        e.render_block();
        let out = e.node_output(NodeId(0), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
        assert!(out.iter().any(|&s| s != 0.0));
    }

    #[test]
    fn per_side_write_gains_route_l_and_r_separately() {
        // node0 = const 0.5 (Add of const+0). Two gained writes: (1,0)→L only,
        // (0,1)→R only. Plus a plain center write from a second source to
        // prove (1,1) is unchanged.
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.apply(Cmd::BusWriteGains { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(0), gl: 1.0, gr: 0.0 });
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6, "L got the (1,0) write");
        assert!(out[0].r.abs() < 1e-6, "R silent for a (1,0) write");

        // A (0,1) write lands only on R.
        let mut e2 = E::new(16.0);
        e2.create(NodeId(0), Kind::Add);
        *e2.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e2.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e2.apply(Cmd::BusWriteGains { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(0), gl: 0.0, gr: 1.0 });
        e2.set_root(BusId(0));
        let mut out2 = [StereoFrame::default(); 16];
        let sil2 = [StereoFrame::default(); 16];
        e2.render(&mut out2, &sil2);
        assert!(out2[0].l.abs() < 1e-6 && (out2[0].r - 0.5).abs() < 1e-6);
    }

    #[test]
    fn input_node_routes_line_in_to_output() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Input);
        // route port0→L and port1→R of the input node into master, like Out.patch.
        e.bus_write_gains(Input::Node { node: NodeId(0), port: 0 }, BusId(0), 1.0, 0.0);
        e.bus_write_gains(Input::Node { node: NodeId(0), port: 1 }, BusId(0), 0.0, 1.0);
        e.set_root(BusId(0));

        let mut input = [StereoFrame::default(); 16];
        for i in 0..16 { input[i] = StereoFrame { l: 0.25, r: -0.5 }; }
        let mut out = [StereoFrame::default(); 16];
        e.render(&mut out, &input);
        assert!((out[0].l - 0.25).abs() < 1e-6);
        assert!((out[0].r - (-0.5)).abs() < 1e-6);
        assert!((out[15].l - 0.25).abs() < 1e-6);
    }

    #[test]
    fn input_shorter_than_block_is_silent_tail_no_panic() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Input);
        e.bus_write_gains(Input::Node { node: NodeId(0), port: 0 }, BusId(0), 1.0, 0.0);
        e.set_root(BusId(0));
        let input = [StereoFrame { l: 1.0, r: 1.0 }; 4]; // shorter than BLOCK=16
        let mut out = [StereoFrame::default(); 16];
        e.render(&mut out, &input); // must not panic
        assert!((out[0].l - 1.0).abs() < 1e-6);
        assert_eq!(out[15].l, 0.0); // beyond input → silence
        // empty input also fine
        e.render(&mut out, &[]);
        assert_eq!(out[0].l, 0.0);
    }

    #[test]
    fn delay_node_renders_delayed_impulse_via_pool() {
        let mut e = E::new(48_000.0);
        let ring = e.pool_alloc(4096).expect("pool room");
        e.pool_slice_mut(ring).fill(0.0);
        e.create(NodeId(0), Kind::Delay);
        // impulse input via a const won't give a single spike; drive port 0
        // with a one-shot is awkward at graph level, so assert boundedness +
        // that a bound-buffer Delay runs (differs from dry) instead.
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5); // steady input
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.005); // 5 ms
        *e.node_input_mut(NodeId(0), 2).unwrap() = Input::Const(0.5); // feedback
        e.apply(Cmd::SetParam { node: NodeId(0), param: 0, value: 0.5 }); // mix
        e.apply(Cmd::BindTable { node: NodeId(0), src: TableSrc::Pooled(ring) });
        e.render_block();
        let out = e.node_output(NodeId(0), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 8.0));
        assert!(out.iter().any(|&s| s != 0.0));
        // Free → pool region reclaimed (same handle reallocates).
        e.apply(Cmd::Free { node: NodeId(0) });
        assert_eq!(e.pool_alloc(4096), Some(ring));
    }

    #[test]
    fn poly_chain_sums_eight_voices_in_phase() {
        // PolyCtrl(all voices = same freq) → PolyOsc → VoiceSum.
        // All 8 lanes are identical (phase starts 0), so the sum peaks near 8×.
        type PE = Engine<64, 8, 32, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(0), Kind::PolyCtrl);
        for v in 0..VOICES {
            e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: 440.0 });
        }
        e.create(NodeId(1), Kind::PolyOsc);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.create(NodeId(2), Kind::VoiceSum);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        e.render_block();
        let out = e.node_output(NodeId(2), 0);
        let peak = out.iter().cloned().fold(0.0f32, |m, s| m.max(s.abs()));
        assert!(out.iter().all(|s| s.is_finite()), "finite");
        assert!(peak > 7.0 && peak <= 8.001, "8 in-phase voices sum near 8×: peak {peak}");
    }

    #[test]
    fn poly_distinct_voices_partially_cancel() {
        // Distinct per-voice frequencies → lanes drift out of phase → the sum's
        // peak stays well below 8× (proving voices are independent, not cloned).
        type PE = Engine<64, 8, 32, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(0), Kind::PolyCtrl);
        for v in 0..VOICES {
            e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v as f32 + 1.0) * 300.0 });
        }
        e.create(NodeId(1), Kind::PolyOsc);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.create(NodeId(2), Kind::VoiceSum);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        e.render_block();
        let out = e.node_output(NodeId(2), 0);
        let peak = out.iter().cloned().fold(0.0f32, |m, s| m.max(s.abs()));
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 8.001));
        assert!(peak < 7.0, "distinct voices don't all align: peak {peak}");
    }

    #[test]
    fn poly_dangling_input_is_silent() {
        // A VoiceSum whose poly input references a non-existent node → silence.
        type PE = Engine<64, 8, 32, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(2), Kind::VoiceSum);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(5), port: 0 };
        e.render_block();
        let out = e.node_output(NodeId(2), 0);
        assert!(out.iter().all(|&s| s == 0.0), "dangling poly edge → silence, no panic");
    }

    #[test]
    fn poly_chain_matches_independent_reference_sum() {
        // Distinct per-voice frequencies: the VoiceSum output must EXACTLY equal
        // an independently-computed sum of 8 per-voice reference sines. This pins
        // the full arena↔poly_scratch interleave round-trip end-to-end — a
        // transpose would deliver the wrong frequency to each lane and fail here
        // (unlike the bulk-property tests above, which survive a permutation).
        // Runs in both feature configs, so it also null-tests the f32x8 PolyOsc.
        type PE = Engine<64, 8, 32, 4, 45056, 2048>;
        let sr = 48_000.0f32;
        let dt = 1.0 / sr;
        let freqs: [f32; VOICES] = core::array::from_fn(|v| (v as f32 + 1.0) * 137.0);
        let mut e = PE::new(sr);
        e.create(NodeId(0), Kind::PolyCtrl);
        for v in 0..VOICES {
            e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: freqs[v] });
        }
        e.create(NodeId(1), Kind::PolyOsc);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.create(NodeId(2), Kind::VoiceSum);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        e.render_block();
        let out = e.node_output(NodeId(2), 0);
        // Reference: per-voice phase accumulator (phase stays positive, so
        // `fract()` matches the kernel's floorf-based wrap). Advance-then-output,
        // matching the kernel's order.
        let mut ph = [0.0f32; VOICES];
        for (i, &got) in out.iter().enumerate() {
            let mut want = 0.0f32;
            for v in 0..VOICES {
                ph[v] += freqs[v] * dt;
                ph[v] -= ph[v].floor();
                want += deluge_dsp_kernels::fast_sin(ph[v]);
            }
            assert!((got - want).abs() < 1e-3, "sample {i}: got {got}, want {want}");
        }
    }

    #[test]
    fn poly_mul_node_multiplies_two_poly_sources() {
        // Two PolyCtrl sources → PolyMul → VoiceSum. Sum == Σ_v (a_v * b_v).
        type PE = Engine<64, 8, 40, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(0), Kind::PolyCtrl);
        e.create(NodeId(1), Kind::PolyCtrl);
        for v in 0..VOICES {
            e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v + 1) as f32 }); // 1..=8
            e.apply(Cmd::SetParam { node: NodeId(1), param: v as u8, value: 2.0 });
        }
        e.create(NodeId(2), Kind::PolyMul);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        *e.node_input_mut(NodeId(2), 1).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        e.create(NodeId(3), Kind::VoiceSum);
        *e.node_input_mut(NodeId(3), 0).unwrap() = Input::Node { node: NodeId(2), port: 0 };
        e.render_block();
        let out = e.node_output(NodeId(3), 0);
        let want: f32 = (1..=VOICES).map(|x| x as f32 * 2.0).sum(); // Σ 2·(1..8) = 72
        assert!(out.iter().all(|&s| (s - want).abs() < 1e-3), "sum of a·b == {want}");
    }

    #[test]
    fn mono_source_broadcasts_to_all_poly_lanes() {
        // PolyOsc(pitch) as poly source A; a mono Ctrl=0.5 as source B;
        // PolyMul(A, B). Because B is width-1, the broadcast must splat 0.5 to
        // every lane, so PolyMul lane v == PolyOsc lane v * 0.5 for every v —
        // including v > 0, which the old VOICES-row-copy left as zero/garbage
        // (Ctrl only ever writes its own single output row).
        type PE = Engine<64, 8, 40, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(0), Kind::PolyCtrl); // pitch per voice
        for v in 0..VOICES {
            e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v as f32 + 1.0) * 110.0 });
        }
        e.create(NodeId(1), Kind::PolyOsc); // poly source A
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.create(NodeId(2), Kind::Ctrl); // mono source B
        e.apply(Cmd::SetParam { node: NodeId(2), param: 0, value: 0.5 });
        e.create(NodeId(3), Kind::PolyMul);
        *e.node_input_mut(NodeId(3), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        *e.node_input_mut(NodeId(3), 1).unwrap() = Input::Node { node: NodeId(2), port: 0 };
        e.render_block();

        for v in 0..VOICES {
            let osc = e.node_output(NodeId(1), v as u8);
            let out = e.node_output(NodeId(3), v as u8);
            let osc: [f32; 64] = osc.try_into().unwrap();
            let out: [f32; 64] = out.try_into().unwrap();
            for i in 0..64 {
                let want = osc[i] * 0.5;
                assert!(
                    (out[i] - want).abs() < 1e-5,
                    "voice {v} sample {i}: got {}, want {} (broadcast of mono B)",
                    out[i], want
                );
            }
            if v > 0 {
                assert!(
                    out.iter().any(|&s| s.abs() > 1e-6),
                    "voice {v} must carry the broadcast mono source, not zero/garbage"
                );
            }
        }
    }

    #[test]
    fn const_source_broadcasts_to_all_poly_lanes() {
        // I-1 regression guard: a poly input port driven directly by a
        // numeric literal (Input::Const(v), v != 0) — e.g. `Osc.syncSaw(60, p)`
        // or `o.width = 0.3` in Wren — must broadcast v to every VOICES lane,
        // exactly like a mono (width-1) Node source does. Before the fix,
        // Input::Const fell into the poly-resolution catch-all `_ => zero`
        // arm and silently produced an all-zero tile on every lane.
        type PE = Engine<64, 8, 40, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(0), Kind::PolyCtrl); // pitch per voice
        for v in 0..VOICES {
            e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v as f32 + 1.0) * 110.0 });
        }
        e.create(NodeId(1), Kind::PolyOsc); // poly source A
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.create(NodeId(2), Kind::PolyMul);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        *e.node_input_mut(NodeId(2), 1).unwrap() = Input::Const(0.5);
        e.render_block();

        for v in 0..VOICES {
            let osc = e.node_output(NodeId(1), v as u8);
            let out = e.node_output(NodeId(2), v as u8);
            let osc: [f32; 64] = osc.try_into().unwrap();
            let out: [f32; 64] = out.try_into().unwrap();
            for i in 0..64 {
                let want = osc[i] * 0.5;
                assert!(
                    (out[i] - want).abs() < 1e-5,
                    "voice {v} sample {i}: got {}, want {} (broadcast of Input::Const(0.5))",
                    out[i], want
                );
            }
            assert!(
                out.iter().any(|&s| s.abs() > 1e-6),
                "voice {v} must carry the broadcast Const source, not zero (I-1 regression)"
            );
        }
    }

    #[test]
    fn polyosc_width_port_default_and_mono_broadcast_pwm() {
        // PolyOsc's new width port (port 1): unconnected ⇒ Const(0.0) ⇒ the
        // engine's mono→poly broadcast (Task 1) delivers an all-zero tile ⇒
        // 0.5 duty (backward-compat default). Wiring a mono Ctrl source ⇒ that
        // value's duty, broadcast to every voice. Low freq (100 Hz, exactly 480
        // samples/cycle @ 48 kHz) over 10 cycles so the +1 fraction approximates
        // the duty cycle (mirrors `osc::tests::pwm_duty_cycle_tracks_width`).
        type PE = Engine<64, 8, 40, 4, 45056, 2048>;
        const BLOCK: usize = 64;
        const N_BLOCKS: usize = 75; // 4800 samples = 10 cycles @ 100 Hz
        let sr = 48_000.0f32;
        let freq = 100.0f32;

        let render = |width_const: Option<f32>| -> [f32; BLOCK * N_BLOCKS] {
            let mut e = PE::new(sr);
            e.create(NodeId(0), Kind::PolyCtrl); // pitch per voice
            for v in 0..VOICES {
                e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: freq });
            }
            e.create(NodeId(1), Kind::PolyOsc);
            e.apply(Cmd::SetParam { node: NodeId(1), param: 0, value: 2.0 }); // Square
            *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
            if let Some(w) = width_const {
                e.create(NodeId(2), Kind::Ctrl); // mono width source
                e.apply(Cmd::SetParam { node: NodeId(2), param: 0, value: w });
                *e.node_input_mut(NodeId(1), 1).unwrap() = Input::Node { node: NodeId(2), port: 0 };
            }
            // else: width port (1) stays unconnected, i.e. Input::Const(0.0).
            let mut out = [0.0f32; BLOCK * N_BLOCKS];
            for b in 0..N_BLOCKS {
                e.render_block();
                out[b * BLOCK..(b + 1) * BLOCK].copy_from_slice(e.node_output(NodeId(1), 0));
            }
            out
        };

        let default_out = render(None);
        let shifted_out = render(Some(0.2));

        for out in [&default_out, &shifted_out] {
            assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2), "finite/bounded");
            assert!(out.iter().any(|&s| s != 0.0), "non-silent");
        }

        let duty = |out: &[f32]| out.iter().filter(|&&s| s > 0.0).count() as f32 / out.len() as f32;
        let d_default = duty(&default_out);
        let d_shifted = duty(&shifted_out);
        assert!((d_default - 0.5).abs() < 0.05, "unconnected width port ⇒ ~0.5 duty, got {d_default}");
        assert!((d_shifted - 0.2).abs() < 0.05, "width=0.2 broadcast ⇒ ~0.2 duty, got {d_shifted}");
    }

    #[test]
    fn gate_voice_reaches_only_addressed_lane() {
        // A PolyAr gated on voice 3 only → after some samples, VoiceSum > 0 comes
        // solely from lane 3 (all others idle at 0).
        type PE = Engine<64, 8, 40, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(0), Kind::PolyAr);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.0005); // fast attack
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.05);
        e.create(NodeId(1), Kind::VoiceSum);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.apply(Cmd::GateVoice { node: NodeId(0), voice: 3, on: true });
        e.render_block();
        let out = e.node_output(NodeId(1), 0);
        // Exactly one voice ramping ⇒ sum rises toward ~1 (not 0, not ~8).
        let peak = out.iter().cloned().fold(0.0f32, |m, s| m.max(s));
        assert!(peak > 0.1 && peak < 1.5, "one gated voice: peak {peak}");
    }

    #[test]
    fn full_voice_gated_per_voice() {
        // A complete voice: gate voices 0 & 1 on, 2..7 off. The mix is non-silent
        // and comes only from the gated voices; with no gates it is silent; the
        // filter keeps the output bounded. Runs in both feature configs.
        type PE = Engine<64, 8, 48, 4, 45056, 2048>;
        let build = |gates: &[usize]| -> [f32; 64] {
            let mut e = PE::new(48_000.0);
            // pitch source
            e.create(NodeId(0), Kind::PolyCtrl);
            for v in 0..VOICES {
                e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v as f32 + 1.0) * 110.0 });
            }
            // osc → filter
            e.create(NodeId(1), Kind::PolyOsc);
            *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
            e.create(NodeId(2), Kind::PolySvf);
            *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
            *e.node_input_mut(NodeId(2), 1).unwrap() = Input::Const(1200.0); // cutoff
            *e.node_input_mut(NodeId(2), 2).unwrap() = Input::Const(0.2);    // res
            // envelope + VCA
            e.create(NodeId(3), Kind::PolyAr);
            *e.node_input_mut(NodeId(3), 0).unwrap() = Input::Const(0.0005); // attack
            *e.node_input_mut(NodeId(3), 1).unwrap() = Input::Const(0.05);   // release
            e.create(NodeId(4), Kind::PolyMul);
            *e.node_input_mut(NodeId(4), 0).unwrap() = Input::Node { node: NodeId(2), port: 0 };
            *e.node_input_mut(NodeId(4), 1).unwrap() = Input::Node { node: NodeId(3), port: 0 };
            // sum → out
            e.create(NodeId(5), Kind::VoiceSum);
            *e.node_input_mut(NodeId(5), 0).unwrap() = Input::Node { node: NodeId(4), port: 0 };
            for &g in gates {
                e.apply(Cmd::GateVoice { node: NodeId(3), voice: g as u8, on: true });
            }
            e.render_block();
            let mono = e.node_output(NodeId(5), 0);
            let mut out = [0.0f32; 64];
            out.copy_from_slice(&mono[..64]);
            out
        };

        // No gates → silence.
        let silent = build(&[]);
        assert!(silent.iter().all(|&s| s.abs() < 1e-6), "no gates → silent");

        // Gate voices 0 & 1 → non-silent, bounded/finite.
        let voiced = build(&[0, 1]);
        assert!(voiced.iter().all(|&s| s.is_finite() && s.abs() <= 8.0), "bounded");
        assert!(voiced.iter().any(|&s| s.abs() > 1e-4), "gated voices sound");
    }

    fn build_voice(e: &mut Engine<64, 8, 56, 4, 45056, 2048>) {
        // PolyCtrl(0) → PolyMtof(1) → PolyOsc(2) → PolySvf(3) →
        //     PolyMul(4, PolyAr(5)) → VoiceSum(6)
        e.create(NodeId(0), Kind::PolyCtrl);
        e.create(NodeId(1), Kind::PolyMtof);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.create(NodeId(2), Kind::PolyOsc);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        e.create(NodeId(3), Kind::PolySvf);
        *e.node_input_mut(NodeId(3), 0).unwrap() = Input::Node { node: NodeId(2), port: 0 };
        *e.node_input_mut(NodeId(3), 1).unwrap() = Input::Const(2000.0);
        *e.node_input_mut(NodeId(3), 2).unwrap() = Input::Const(0.2);
        e.create(NodeId(5), Kind::PolyAr);
        *e.node_input_mut(NodeId(5), 0).unwrap() = Input::Const(0.001); // attack
        *e.node_input_mut(NodeId(5), 1).unwrap() = Input::Const(0.005); // release
        e.create(NodeId(4), Kind::PolyMul);
        *e.node_input_mut(NodeId(4), 0).unwrap() = Input::Node { node: NodeId(3), port: 0 };
        *e.node_input_mut(NodeId(4), 1).unwrap() = Input::Node { node: NodeId(5), port: 0 };
        e.create(NodeId(6), Kind::VoiceSum);
        *e.node_input_mut(NodeId(6), 0).unwrap() = Input::Node { node: NodeId(4), port: 0 };
    }

    #[test]
    fn note_on_sounds_then_note_off_silences() {
        type PE = Engine<64, 8, 56, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        build_voice(&mut e);
        let mut gates = [NodeId(0); crate::voice::MAX_GATES];
        gates[0] = NodeId(5);
        let mut alloc = crate::voice::VoiceAllocator::new(
            NodeId(0), gates, 1, None, NodeId(99),
            [NodeId(0); crate::voice::MAX_TRIGGERS], 0,
        );

        { let mut emit = |c: Cmd| e.apply(c); alloc.note_on(69, 100, &mut emit); }
        for _ in 0..8 { e.render_block(); } // let the fast envelope attack
        let out = e.node_output(NodeId(6), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 8.5), "bounded");
        assert!(out.iter().any(|&s| s.abs() > 1e-3), "note-on sounds");

        { let mut emit = |c: Cmd| e.apply(c); alloc.note_off(69, &mut emit); }
        for _ in 0..300 { e.render_block(); } // past the 5 ms release
        let out2 = e.node_output(NodeId(6), 0);
        assert!(out2.iter().all(|&s| s.abs() < 1e-4), "note-off silences");
    }

    #[test]
    fn ninth_note_steals_and_still_sounds() {
        type PE = Engine<64, 8, 56, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        build_voice(&mut e);
        let mut gates = [NodeId(0); crate::voice::MAX_GATES];
        gates[0] = NodeId(5);
        let mut alloc = crate::voice::VoiceAllocator::new(
            NodeId(0), gates, 1, None, NodeId(99),
            [NodeId(0); crate::voice::MAX_TRIGGERS], 0,
        );
        {
            let mut emit = |c: Cmd| e.apply(c);
            for k in 0..9u8 {
                alloc.note_on(60 + k, 100, &mut emit); // 9 notes → one steal
            }
        }
        for _ in 0..8 { e.render_block(); }
        let out = e.node_output(NodeId(6), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 8.5), "bounded after steal");
        assert!(out.iter().any(|&s| s.abs() > 1e-3), "still sounds after steal");
    }

    #[test]
    fn master_limiter_bounds_root_bus_when_enabled() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::SetMasterLimit { ceiling: 0.5, release: 0.05 });
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        for i in 0..16 {
            assert!(out[i].l.abs() <= 0.5 + 1e-6, "out[{}].l={}", i, out[i].l);
        }
    }

    #[test]
    fn master_limiter_disabled_is_raw_then_clamp() {
        // Without SetMasterLimit, a 0.98 root bus passes at 0.98 (below the clamp).
        // 0.98 is deliberately ABOVE the limiter's 0.95 default ceiling, so this
        // also catches an accidental `Some(default-limiter)` init: a default
        // limiter would attenuate 0.98 toward 0.95 and fail this assertion.
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.98);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.98).abs() < 1e-6); // unlimited, only the [-1,1] clamp would act
    }

    #[test]
    fn master_limiter_reset_clears() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::SetMasterLimit { ceiling: 0.5, release: 0.05 });
        e.apply(Cmd::Reset);
        // After Reset the graph is torn down; rebuild the same patch, no limiter.
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.8).abs() < 1e-6); // limiter cleared → unlimited
    }

    #[test]
    fn master_dcblock_removes_dc_when_enabled() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5); // DC
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::SetMasterDcBlock { cutoff_hz: 20.0 });
        // Render several blocks so the HP settles.
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        for _ in 0..512 {
            e.render(&mut out, &sil);
        }
        assert!(out[15].l.abs() < 1e-2, "DC not removed: {}", out[15].l);
    }

    #[test]
    fn master_dcblock_disabled_is_byte_identical() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6); // no DC-block → 0.5 passes
    }

    #[test]
    fn master_dcblock_reset_clears() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::SetMasterDcBlock { cutoff_hz: 20.0 });
        e.apply(Cmd::Reset);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6); // DC-block cleared → 0.5 passes
    }

    #[test]
    fn master_eq_changes_render_when_enabled() {
        let build = |eq: bool| -> f32 {
            let mut e = E::new(48_000.0);
            e.create(NodeId(0), Kind::Add);
            *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.2);
            *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
            e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
            e.set_root(BusId(0));
            if eq {
                // Low-shelf (type 1) boost: a shelf below `freq` raises DC content,
                // so a constant source measurably changes (a peak EQ would not
                // touch DC). +12 dB → the 0.2 DC level climbs toward ~0.8.
                e.apply(Cmd::SetMasterEq { freq: 1000.0, gain_db: 12.0, q: 0.707, eq_type: 1 });
            }
            let mut out = [StereoFrame::default(); 16];
            let sil = [StereoFrame::default(); 16];
            for _ in 0..64 {
                e.render(&mut out, &sil);
            }
            out[15].l
        };
        let off = build(false);
        let on = build(true);
        assert!(on > off + 0.1, "low-shelf boost should raise the DC level (off={}, on={})", off, on);
    }

    #[test]
    fn master_eq_disabled_is_byte_identical() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6); // no EQ → 0.5 passes
    }

    #[test]
    fn master_eq_reset_clears() {
        let mut e = E::new(48_000.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::SetMasterEq { freq: 2000.0, gain_db: 12.0, q: 2.0, eq_type: 0 });
        e.apply(Cmd::Reset);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.5);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6); // EQ cleared → 0.5 passes
    }

    #[test]
    fn bus_write_reuses_freed_slot() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.25);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0)); // writes_len -> 1
        e.apply(Cmd::Free { node: NodeId(0) }); // nulls that write (hole at 0)
        // A new write should reuse the hole, not grow writes_len.
        e.create(NodeId(1), Kind::Add);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Const(0.4);
        *e.node_input_mut(NodeId(1), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(1), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.4).abs() < 1e-6, "only node1's write is live: {}", out[0].l);
    }

    #[test]
    fn bus_gain_scales_output() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::BusGain { bus: BusId(0), gain: 0.5 });
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.4).abs() < 1e-6, "0.8 * 0.5 = 0.4, got {}", out[0].l);
    }

    #[test]
    fn bus_gain_default_is_byte_identical() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.7);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.7).abs() < 1e-6); // no BusGain → unity, unchanged
    }

    #[test]
    fn bus_gain_reset_restores_unity() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::BusGain { bus: BusId(0), gain: 0.5 });
        e.apply(Cmd::Reset);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.8).abs() < 1e-6, "Reset → unity, got {}", out[0].l);
    }

    #[test]
    fn bus_gain_out_of_range_is_noop() {
        let mut e = E::new(16.0);
        e.apply(Cmd::BusGain { bus: BusId(99), gain: 0.5 }); // BUSES=4 → no-op, no panic
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.6);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.6).abs() < 1e-6); // unaffected
    }

    #[test]
    fn bus_send_folds_stereo_into_target() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.8);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        // node0 → busA(1) with L=0.8, R=0.2 (asymmetric, proves L→L/R→R).
        e.bus_write_gains(Input::Node { node: NodeId(0), port: 0 }, BusId(1), 1.0, 0.25);
        e.apply(Cmd::BusSend { from: BusId(1), to: BusId(0), gain: 0.5 });
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        // busA L=0.8, R=0.2 → sent ×0.5 into master → L=0.4, R=0.1.
        assert!((out[0].l - 0.4).abs() < 1e-6, "L: {}", out[0].l);
        assert!((out[0].r - 0.1).abs() < 1e-6, "R: {}", out[0].r);
    }

    #[test]
    fn bus_send_chain_routes_same_block() {
        // bus2 → bus1 → bus0 (master), each from > to, descending-from ordering.
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.4);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(2)); // node → bus2
        e.apply(Cmd::BusSend { from: BusId(2), to: BusId(1), gain: 1.0 });
        e.apply(Cmd::BusSend { from: BusId(1), to: BusId(0), gain: 1.0 });
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.4).abs() < 1e-6, "chain to master: {}", out[0].l);
    }

    #[test]
    fn bus_send_none_is_byte_identical() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.7);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.7).abs() < 1e-6); // no sends → unchanged
    }

    #[test]
    fn bus_send_self_and_oob_are_noop() {
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.6);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.apply(Cmd::BusSend { from: BusId(0), to: BusId(0), gain: 2.0 }); // self → no-op
        e.apply(Cmd::BusSend { from: BusId(9), to: BusId(0), gain: 2.0 }); // oob → no-op
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil); // must not panic
        assert!((out[0].l - 0.6).abs() < 1e-6, "self/oob send no-op: {}", out[0].l);
    }

    #[test]
    fn bus_fed_node_reads_previous_block() {
        let mut e = E::new(16.0);
        // node0 = const 0.25, written (center) into bus1 → bus_l[1]=bus_r[1]=0.25.
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.25);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(1));
        // node1 = reads bus1 (Add(bus1, 0)), routed to master bus0. Note
        // `Input::Bus` sums L+R, so a 0.25 center write reads back as 0.5.
        e.create(NodeId(1), Kind::Add);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Bus(BusId(1));
        *e.node_input_mut(NodeId(1), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(1), port: 0 }, BusId(0));
        e.set_root(BusId(0));

        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        // Block 1: node1 reads bus1 = 0 (initial), so master ~0.
        e.render(&mut out, &sil);
        assert!(out[0].l.abs() < 1e-6, "block1 bus read should be 0: {}", out[0].l);
        // Block 2: node1 reads bus1 = block1's write (0.25 center → L+R sum = 0.5)
        // → node1 = 0.5 → master = 0.5. Proves the one-block-late bus→node read.
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6, "block2 one-block-late read: {}", out[0].l);
    }

    #[test]
    fn existing_routing_byte_identical_after_reorder() {
        // Mirrors two_nodes_sum_into_master_bus: no Input::Bus reads → unchanged.
        let mut e = E::new(16.0);
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.3);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.create(NodeId(1), Kind::Add);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Const(0.4);
        *e.node_input_mut(NodeId(1), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(0));
        e.bus_write(Input::Node { node: NodeId(1), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.7).abs() < 1e-6); // 0.3 + 0.4, same as before the reorder
    }

    #[test]
    fn reset_zeros_buses() {
        let mut e = E::new(16.0);
        // Drive bus1 with content over a block.
        e.create(NodeId(0), Kind::Add);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(0.9);
        *e.node_input_mut(NodeId(0), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(0), port: 0 }, BusId(1));
        e.set_root(BusId(1));
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil); // bus1 now holds 0.9
        e.apply(Cmd::Reset);
        // Rebuild a bus1-reading graph; first render must read a zeroed bus1.
        e.create(NodeId(1), Kind::Add);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Bus(BusId(1));
        *e.node_input_mut(NodeId(1), 1).unwrap() = Input::Const(0.0);
        e.bus_write(Input::Node { node: NodeId(1), port: 0 }, BusId(0));
        e.set_root(BusId(0));
        e.render(&mut out, &sil);
        assert!(out[0].l.abs() < 1e-6, "Reset must zero buses; got stale {}", out[0].l);
    }
}
