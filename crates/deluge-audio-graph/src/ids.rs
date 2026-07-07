//! Public identifiers and the connection type. `NodeId` is the only id an author
//! or the wire ever sees; output-slot indices are engine-internal.

/// A compute node in the arena.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NodeId(pub u16);

/// A stereo bus.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct BusId(pub u16);

/// A node input: a constant, a source node's output port, or a bus.
#[derive(Clone, Copy)]
pub enum Input {
    Const(f32),
    Node { node: NodeId, port: u8 },
    Bus(BusId),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn input_variants_are_copy_and_constructible() {
        let a = Input::Const(0.5);
        let b = Input::Node { node: NodeId(3), port: 2 };
        let c = Input::Bus(BusId(1));
        // Copy check
        let _copies = (a, b, c);
        match b {
            Input::Node { node, port } => {
                assert_eq!(node, NodeId(3));
                assert_eq!(port, 2);
            }
            _ => panic!("wrong variant"),
        }
    }
}
