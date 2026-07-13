//! Control-rate mutations and the host transport seam. A host (firmware ring /
//! web direct) ships `Cmd`s to the engine's `apply`. Mirrors the prototype's
//! `Cmd`/`audio_cmd`, generalized to the P0 model.

use crate::{BusId, Input, NodeId};
use crate::node::Kind;

pub const MAX_ARGS: usize = 3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Cmd {
    Nop,
    NewNode { node: NodeId, kind: Kind, args: [Input; MAX_ARGS] },
    SetInput { node: NodeId, port: u8, src: Input },
    SetParam { node: NodeId, param: u8, value: f32 },
    BindTable { node: NodeId, src: crate::node::TableSrc },
    Gate { node: NodeId, on: bool },
    Trigger { node: NodeId },
    GateVoice { node: NodeId, voice: u8, on: bool },
    TriggerVoice { node: NodeId, voice: u8 },
    /// Prefetch → engine: update a `StreamPlayer` voice's resident ring window
    /// `[fill_lo, fill_hi)` (and the stream's `total`, carried idempotently).
    StreamFill { node: NodeId, voice: u8, fill_lo: u64, fill_hi: u64, total: u64 },
    BusWrite { src: Input, bus: BusId },
    BusWriteGains { src: Input, bus: BusId, gl: f32, gr: f32 },
    SetRoot { bus: BusId },
    /// Enable/configure the master limiter on the root bus. Creates it if absent,
    /// else updates params in place (preserving the running gain envelope).
    SetMasterLimit { ceiling: f32, release: f32 },
    Free { node: NodeId },
    Reset,
}

/// Transport seam: the firmware enqueues onto a critical-section ring; the web
/// sim applies directly. Same shape as the prototype's `Host`.
pub trait Host {
    fn audio_cmd(&self, cmd: Cmd);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::Engine;
    use crate::node::Kind;
    use crate::{BusId, Input, NodeId, StereoFrame};

    type E = Engine<16, 8, 8, 4, 45056, 2048>;

    fn saw_patch(e: &mut E) {
        e.apply(Cmd::NewNode {
            node: NodeId(0),
            kind: Kind::Saw,
            args: [Input::Const(4.0), Input::Const(0.0), Input::Const(0.0)],
        });
        e.apply(Cmd::BusWrite { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(0) });
        e.apply(Cmd::SetRoot { bus: BusId(0) });
    }

    #[test]
    fn apply_builds_and_renders_a_patch() {
        let mut e = E::new(16.0);
        saw_patch(&mut e);
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[1].l - (-0.41666675)).abs() < 1e-6); // saw at phase .25 (band-limited)
    }

    #[test]
    fn free_then_reuse_keeps_eval_order_sound() {
        let mut e = E::new(16.0);
        saw_patch(&mut e);
        e.apply(Cmd::Free { node: NodeId(0) });
        // Reuse id 0 as a different node; must render cleanly (no stale slot read).
        e.apply(Cmd::NewNode {
            node: NodeId(0),
            kind: Kind::Add,
            args: [Input::Const(0.25), Input::Const(0.0), Input::Const(0.0)],
        });
        e.apply(Cmd::BusWrite { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(0) });
        e.apply(Cmd::SetRoot { bus: BusId(0) });
        // Note: the bus_write list still holds the write recorded before the
        // free/recreate (P0 has no persistent routing table to invalidate; see
        // engine.rs). Both recorded `BusWrite`s name `NodeId(0)`, which now
        // resolves at render time to the recreated Add node, so bus0 sums the
        // SAME node's output twice: 0.25 + 0.25 = 0.5 (not two distinct
        // sources).
        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        assert!((out[0].l - 0.5).abs() < 1e-6);
        assert!(out[0].l.is_finite() && out[0].l.abs() <= 1.0);
    }

    #[test]
    fn saw_lpf_env_parity_first_samples() {
        // Osc.saw(4) .lpf(800) * Env.ar(0.01,0.1), gated on. Mirrors a prototype
        // patch; we assert the first sample is near-silent (env starts at 0 and
        // ramps from there) and the block is bounded. A realistic audio sample
        // rate is used here (unlike the other tests' 16 Hz convenience rate) so
        // the 10ms attack and 800Hz cutoff time constants are actually resolved
        // across samples instead of both saturating within a single sample.
        let mut e = E::new(48_000.0);
        e.apply(Cmd::NewNode { node: NodeId(0), kind: Kind::Saw,
            args: [Input::Const(4.0), Input::Const(0.0), Input::Const(0.0)] });
        e.apply(Cmd::NewNode { node: NodeId(1), kind: Kind::Lpf,
            args: [Input::Node { node: NodeId(0), port: 0 }, Input::Const(800.0), Input::Const(0.0)] });
        e.apply(Cmd::NewNode { node: NodeId(2), kind: Kind::Env,
            args: [Input::Const(0.01), Input::Const(0.1), Input::Const(0.0)] });
        e.apply(Cmd::Gate { node: NodeId(2), on: true });
        e.apply(Cmd::NewNode { node: NodeId(3), kind: Kind::Mul,
            args: [Input::Node { node: NodeId(1), port: 0 }, Input::Node { node: NodeId(2), port: 0 }, Input::Const(0.0)] });
        e.apply(Cmd::BusWrite { src: Input::Node { node: NodeId(3), port: 0 }, bus: BusId(0) });
        e.apply(Cmd::SetRoot { bus: BusId(0) });

        let mut out = [StereoFrame::default(); 16];
        let sil = [StereoFrame::default(); 16];
        e.render(&mut out, &sil);
        // Env level after one sample's attack increment (dt/atk) is small but not
        // exactly 0 (the kernel writes post-increment); still far quieter than a
        // fully-open envelope would produce.
        assert!(out[0].l.abs() < 1e-3);
        assert!(out.iter().all(|f| f.l.abs() <= 1.0 && f.l.is_finite()));
    }

    /// CHARACTERIZATION golden (spec §8 P0 testing gate): pins the first 8
    /// rendered samples of the deterministic saw→lpf→env patch (same topology
    /// as `saw_lpf_env_parity_first_samples`) as literal constants. This test
    /// exists to catch *unintended* drift: if a future kernel refactor
    /// silently changes the numeric output of the oscillator, filter, or
    /// envelope, this test fails. Regenerating the pinned constants below is
    /// the correct response ONLY when the output change is intended and has
    /// been reviewed — do not "fix" a failure here by blindly re-pinning.
    #[test]
    fn golden_saw_lpf_env_first_block() {
        let mut e = E::new(48_000.0);
        e.apply(Cmd::NewNode { node: NodeId(0), kind: Kind::Saw,
            args: [Input::Const(4.0), Input::Const(0.0), Input::Const(0.0)] });
        e.apply(Cmd::NewNode { node: NodeId(1), kind: Kind::Lpf,
            args: [Input::Node { node: NodeId(0), port: 0 }, Input::Const(800.0), Input::Const(0.0)] });
        e.apply(Cmd::NewNode { node: NodeId(2), kind: Kind::Env,
            args: [Input::Const(0.01), Input::Const(0.1), Input::Const(0.0)] });
        e.apply(Cmd::Gate { node: NodeId(2), on: true });
        e.apply(Cmd::NewNode { node: NodeId(3), kind: Kind::Mul,
            args: [Input::Node { node: NodeId(1), port: 0 }, Input::Node { node: NodeId(2), port: 0 }, Input::Const(0.0)] });
        e.apply(Cmd::BusWrite { src: Input::Node { node: NodeId(3), port: 0 }, bus: BusId(0) });
        e.apply(Cmd::SetRoot { bus: BusId(0) });

        let mut out = [StereoFrame::default(); 8];
        let sil = [StereoFrame::default(); 8];
        e.render(&mut out, &sil);

        // CHARACTERIZATION golden re-pinned 2026-07-07 after Osc band-limiting (Tasks 1-3).
        // Regenerate only on an intended, reviewed output change.
        const EXPECTED: [f32; 8] = [
            0.0,
            -0.000_399_898_62,
            -0.001_191_312_4,
            -0.002_294_306_4,
            -0.003_657_662_3,
            -0.005_237_465_3,
            -0.006_996_135,
            -0.008_901_581,
        ];
        let actual: [f32; 8] = core::array::from_fn(|i| out[i].l);
        for (i, (a, e)) in actual.iter().zip(EXPECTED.iter()).enumerate() {
            assert!(
                (a - e).abs() < 1e-6,
                "sample {i}: actual {a} vs pinned {e} (diff {})",
                (a - e).abs()
            );
        }
    }

    #[test]
    fn cmds_are_comparable_and_debuggable() {
        use crate::{Input, NodeId};
        use crate::node::Kind;
        let a = Cmd::NewNode { node: NodeId(1), kind: Kind::Saw, args: [Input::Const(110.0), Input::Const(0.0), Input::Const(0.0)] };
        let b = Cmd::NewNode { node: NodeId(1), kind: Kind::Saw, args: [Input::Const(110.0), Input::Const(0.0), Input::Const(0.0)] };
        assert_eq!(a, b);
        assert_ne!(a, Cmd::Reset);
        // Debug is verified by assert_eq! error messages requiring it
    }
}
