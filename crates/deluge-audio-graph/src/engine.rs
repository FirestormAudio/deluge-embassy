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

use deluge_dsp_kernels::In;

use crate::arena::Arena;
use crate::node::{OutView, MAX_BLOCK, MAX_INPUTS};
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
    // Bus rendered to the audio output; set via `set_root`, read in `render`.
    pub(crate) root: Option<BusId>,
    // Pending bus writes, re-applied every `render` (P0: no persistent routing
    // table yet — see spec §3.5 / bus.rs).
    writes: [Option<(Input, BusId)>; NODES],
    writes_len: usize,
    pool: crate::pool::Pool<PCAP, PCHUNK>,
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
            root: None,
            writes: [None; NODES],
            writes_len: 0,
            pool: crate::pool::Pool::new(),
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
            Cmd::BusWrite { src, bus } => self.bus_write(src, bus),
            Cmd::SetRoot { bus } => self.set_root(bus),
            Cmd::Free { node } => {
                // Free a pooled table region (if bound) BEFORE reclaiming the
                // node's arena slot: `table_src()` reads through the node,
                // which must still be live.
                if let Some(n) = self.arena.node_mut(node) {
                    if let Some(crate::node::TableSrc::Pooled(h)) = n.table_src() {
                        self.pool.free(h);
                    }
                }
                self.arena.free(node);
            }
            Cmd::Reset => {
                self.arena.reset();
                self.writes_len = 0;
                self.root = None;
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
            let (base, width, inputs, table_src) = {
                let n = self.arena.node(id).expect("eval-order node exists");
                (n.out_base as usize, Node::out_width(n.kind), n.inputs_snapshot(), n.table_src())
            };

            // ── Resolve inputs into scratch (all reads copied out first) ──
            let mut scratch = [[0.0f32; BLOCK]; MAX_INPUTS];
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
            let mut view = OutView::from_arena::<OUTS, BLOCK>(arr, base, width);
            // Resolve a pooled table's flat region (immutable borrow of the
            // disjoint `self.pool` field) BEFORE the node's `&mut` borrow
            // below — `self.pool` and `self.arena` are separate fields of
            // `Engine`, so the borrow checker tracks them independently as
            // long as each is accessed as a direct field projection (not
            // through a whole-`&mut self` helper method).
            let pool_region: Option<&[f32]> = match table_src {
                Some(crate::node::TableSrc::Pooled(h)) => Some(self.pool.slice(h)),
                _ => None,
            };
            if let Some(n) = self.arena.node_mut(id) {
                n.process_resolved(&ins, self.dt, &mut view, pool_region);
            }
        }
    }

    /// Test/inspection accessor: a node's rendered output port.
    pub fn node_output(&self, id: NodeId, port: u8) -> &[f32] {
        let base = self.arena.out_base(id).expect("node exists");
        // SAFETY: shared read; no writer is live outside `render_block`.
        let arr = unsafe { &*self.outs.get() };
        &arr[base + port as usize][..]
    }

    /// Record that `src` should be summed into `bus` on every subsequent
    /// `render` (P0: center pan, L=R; re-applied each block, see bus.rs).
    pub fn bus_write(&mut self, src: Input, bus: BusId) {
        if self.writes_len < self.writes.len() {
            self.writes[self.writes_len] = Some((src, bus));
            self.writes_len += 1;
        }
    }

    /// Select which bus is copied to the audio output by `render`.
    pub fn set_root(&mut self, bus: BusId) {
        self.root = Some(bus);
    }

    /// Render one block: clear buses, evaluate nodes, apply pending bus
    /// writes, then copy the root bus into `out`, clamped to `[-1, 1]`.
    pub fn render(&mut self, out: &mut [StereoFrame]) {
        // Zero buses.
        for b in 0..BUSES {
            self.bus_l[b] = [0.0; BLOCK];
            self.bus_r[b] = [0.0; BLOCK];
        }
        // Evaluate nodes.
        self.render_block();
        // Apply bus writes (center pan → L=R).
        let arr = unsafe { &*self.outs.get() };
        for w in 0..self.writes_len {
            if let Some((src, bus)) = self.writes[w] {
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
                    self.bus_l[b][i] += v;
                    self.bus_r[b][i] += v;
                }
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
        e.render(&mut out);
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
        e.render(&mut out);
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
        e.render(&mut out); // must not panic

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
}
