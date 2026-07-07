//! The block-rendering engine. Owns the arena and the per-slot output arena and
//! evaluates nodes in topological (eval-order) order, a block at a time.
//!
//! ## Borrow model (spec §3.5)
//! The output arena is one `UnsafeCell<[[f32; BLOCK]; OUTS]>`. Each `render_block`
//! iteration first *resolves* the node's inputs — copying every source (a
//! constant, another slot's block, or a bus) into local `scratch` — and only then
//! writes the node's own slot-run. Because reads are copied out before the write,
//! the write borrow never overlaps a read, so the single `unsafe` deref is sound.

use core::cell::UnsafeCell;

use deluge_dsp_kernels::In;

use crate::arena::Arena;
use crate::node::{OutView, MAX_BLOCK, MAX_INPUTS};
use crate::{BusId, Input, Node, NodeId};

pub struct Engine<
    const BLOCK: usize,
    const NODES: usize,
    const OUTS: usize,
    const BUSES: usize,
> {
    arena: Arena<NODES, OUTS>,
    outs: UnsafeCell<[[f32; BLOCK]; OUTS]>,
    dt: f32,
    pub(crate) bus_l: [[f32; BLOCK]; BUSES],
    pub(crate) bus_r: [[f32; BLOCK]; BUSES],
    // Consumed by the engine (Task 9) to pick the bus rendered to the audio
    // output; unread within this crate until then.
    #[allow(dead_code)]
    pub(crate) root: Option<BusId>,
}

impl<const BLOCK: usize, const NODES: usize, const OUTS: usize, const BUSES: usize>
    Engine<BLOCK, NODES, OUTS, BUSES>
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
        }
    }

    pub fn create(&mut self, id: NodeId, kind: crate::node::Kind) -> bool {
        self.arena.create(id, kind)
    }

    pub fn node_input_mut(&mut self, id: NodeId, port: u8) -> Option<&mut Input> {
        self.arena.node_mut(id)?.input_mut(port)
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
            let (base, width, inputs) = {
                let n = self.arena.node(id).expect("eval-order node exists");
                (n.out_base as usize, Node::out_width(n.kind), n.inputs_snapshot())
            };

            // ── Resolve inputs into scratch (all reads copied out first) ──
            let mut scratch = [[0.0f32; BLOCK]; MAX_INPUTS];
            {
                // SAFETY: read-only view of the output arena; no writer is live.
                let arr = unsafe { &*self.outs.get() };
                for (p, row) in scratch.iter_mut().enumerate() {
                    match inputs[p] {
                        Input::Const(v) => row.fill(v),
                        Input::Node { node, port } => {
                            let sbase = self.arena.out_base(node).unwrap_or(0);
                            *row = arr[sbase + port as usize];
                        }
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
            // SAFETY: the writer touches only [base, base+width); every read was
            // copied into `scratch`, so no read aliases the write region. The raw
            // deref is not tracked against `self`, so the following `node_mut`
            // (a borrow of the disjoint `arena` field) is also permitted.
            let arr = unsafe { &mut *self.outs.get() };
            let mut view = OutView::from_arena::<OUTS, BLOCK>(arr, base, width);
            if let Some(n) = self.arena.node_mut(id) {
                n.process_resolved(&ins, self.dt, &mut view);
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node::Kind;
    use crate::{Input, NodeId};

    type E = Engine<16, 8, 8, 4>;

    #[test]
    fn single_saw_node_renders_expected_ramp() {
        let mut e = E::new(16.0); // sr so 4 Hz → 0.25/sample
        e.create(NodeId(0), Kind::Saw);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(4.0);
        e.render_block();
        let out = e.node_output(NodeId(0), 0);
        assert!((out[0] - (-1.0)).abs() < 1e-6);
        assert!((out[1] - (-0.5)).abs() < 1e-6);
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
}
