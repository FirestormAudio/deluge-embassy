//! Node + output-slot lifecycle. Node slots are indexed by `NodeId`. Output
//! slots are a separate index space (engine-internal) allocated first-fit and
//! reclaimed on free. An explicit eval-order list keeps topological order valid
//! across free/reuse (memory order no longer equals eval order once slots are
//! recycled — see spec §3.5).

use crate::node::Kind;
use crate::{Node, NodeId};

pub struct Arena<const NODES: usize, const OUTS: usize> {
    nodes: [Option<Node>; NODES],
    out_used: [bool; OUTS],
    order: [u16; NODES],
    order_len: usize,
}

impl<const NODES: usize, const OUTS: usize> Arena<NODES, OUTS> {
    pub fn new() -> Self {
        Arena {
            nodes: [None; NODES],
            out_used: [false; OUTS],
            order: [0; NODES],
            order_len: 0,
        }
    }

    fn alloc_out_run(&mut self, width: usize) -> Option<usize> {
        let mut start = 0;
        while start + width <= OUTS {
            if (start..start + width).all(|s| !self.out_used[s]) {
                for s in start..start + width {
                    self.out_used[s] = true;
                }
                return Some(start);
            }
            start += 1;
        }
        None
    }

    /// Create a node at `id`. Allocates its output run and appends to eval order.
    /// Returns false (node inert) if no output run fits.
    pub fn create(&mut self, id: NodeId, kind: Kind) -> bool {
        let idx = id.0 as usize;
        if idx >= NODES {
            return false;
        }
        if self.nodes[idx].is_some() {
            return false; // refuse to double-allocate a live id
        }
        let width = Node::out_width(kind);
        let base = match self.alloc_out_run(width) {
            Some(b) => b,
            None => return false,
        };
        self.nodes[idx] = Some(Node::new(kind, base as u16));
        self.order[self.order_len] = id.0;
        self.order_len += 1;
        true
    }

    pub fn free(&mut self, id: NodeId) {
        let idx = id.0 as usize;
        let taken = match self.nodes.get_mut(idx) {
            Some(slot) => slot.take(),
            None => return,
        };
        if let Some(n) = taken {
            let width = Node::out_width(n.kind);
            let base = n.out_base as usize;
            for s in base..base + width {
                self.out_used[s] = false;
            }
            // Remove from eval order (stable compaction).
            let mut w = 0;
            for r in 0..self.order_len {
                if self.order[r] != id.0 {
                    self.order[w] = self.order[r];
                    w += 1;
                }
            }
            self.order_len = w;
        }
    }

    pub fn reset(&mut self) {
        self.nodes = [None; NODES];
        self.out_used = [false; OUTS];
        self.order_len = 0;
    }

    pub fn node_mut(&mut self, id: NodeId) -> Option<&mut Node> {
        self.nodes.get_mut(id.0 as usize)?.as_mut()
    }

    pub fn node(&self, id: NodeId) -> Option<&Node> {
        self.nodes.get(id.0 as usize)?.as_ref()
    }

    pub fn out_base(&self, id: NodeId) -> Option<usize> {
        self.node(id).map(|n| n.out_base as usize)
    }

    /// Read-only accessor for a node's `Kind` (e.g. so a consumer can check a
    /// source's `out_width` before deciding how to resolve a poly input edge).
    pub fn kind_of(&self, id: NodeId) -> Option<Kind> {
        self.node(id).map(|n| n.kind)
    }

    /// Read-only accessor for a node's `(out_base, kind)` in one lookup — lets a
    /// consumer branch on a source's `out_width` (via `kind`) and its base row
    /// without a second table lookup or an infallible-relookup `expect`.
    pub fn out_base_and_kind(&self, id: NodeId) -> Option<(usize, Kind)> {
        self.node(id).map(|n| (n.out_base as usize, n.kind))
    }

    pub fn eval_order(&self) -> &[u16] {
        &self.order[..self.order_len]
    }
}

impl<const NODES: usize, const OUTS: usize> Default for Arena<NODES, OUTS> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::NodeId;
    use crate::node::Kind;

    type A = Arena<8, 8>;

    #[test]
    fn create_assigns_contiguous_output_runs() {
        let mut a = A::new();
        assert!(a.create(NodeId(0), Kind::Saw)); // width 1 → slot 0
        assert!(a.create(NodeId(1), Kind::Split2)); // width 2 → slots 1,2
        assert!(a.create(NodeId(2), Kind::Saw)); // width 1 → slot 3
        assert_eq!(a.out_base(NodeId(0)), Some(0));
        assert_eq!(a.out_base(NodeId(1)), Some(1));
        assert_eq!(a.out_base(NodeId(2)), Some(3));
        assert_eq!(a.eval_order(), &[0, 1, 2]);
    }

    #[test]
    fn free_reclaims_slots_and_eval_order() {
        let mut a = A::new();
        a.create(NodeId(0), Kind::Split2); // slots 0,1
        a.create(NodeId(1), Kind::Saw); // slot 2
        a.free(NodeId(0)); // slots 0,1 free again
        assert_eq!(a.eval_order(), &[1]);
        a.create(NodeId(2), Kind::Split2); // reuses slots 0,1
        assert_eq!(a.out_base(NodeId(2)), Some(0));
        assert_eq!(a.eval_order(), &[1, 2]);
    }

    #[test]
    fn output_exhaustion_reports_false() {
        let mut a = A::new(); // 8 output slots
        assert!(a.create(NodeId(0), Kind::Split2)); // 2
        assert!(a.create(NodeId(1), Kind::Split2)); // 4
        assert!(a.create(NodeId(2), Kind::Split2)); // 6
        assert!(a.create(NodeId(3), Kind::Split2)); // 8 (full)
        assert!(!a.create(NodeId(4), Kind::Saw)); // no slot → false
    }

    #[test]
    fn free_out_of_range_id_is_noop() {
        let mut a = A::new();
        assert!(a.create(NodeId(0), Kind::Saw));
        a.free(NodeId(99)); // out of range: must not panic, must be a no-op
        assert_eq!(a.eval_order(), &[0]);
        assert_eq!(a.out_base(NodeId(0)), Some(0));
    }

    #[test]
    fn create_on_live_id_is_refused() {
        let mut a = A::new();
        assert!(a.create(NodeId(0), Kind::Saw));
        assert!(!a.create(NodeId(0), Kind::Split2)); // refused: id already live
        assert_eq!(a.eval_order(), &[0]);
        assert_eq!(a.out_base(NodeId(0)), Some(0)); // unchanged: still Saw's slot, width 1
    }
}
