#![cfg(feature = "test-support")]
use deluge_audio_graph::StereoFrame;
use deluge_audio_graph::node::TableSrc;
use deluge_wren_core::Host as _;
use deluge_wren_core::test_support::{EngineHost, run_and_capture_cmds, run_and_render};
use deluge_wren_core::{BusId, Cmd, Input, Kind, NodeId};

fn saw(freq: f32) -> Cmd {
    Cmd::NewNode {
        node: NodeId(0),
        kind: Kind::Saw,
        args: [Input::Const(freq), Input::Const(0.0), Input::Const(0.0)],
    }
}

#[test]
fn osc_saw_emits_newnode() {
    let cmds = run_and_capture_cmds("Osc.saw(110)");
    assert_eq!(cmds, vec![saw(110.0)]);
}

#[test]
fn patch_writes_master_bus_and_sets_root() {
    let cmds = run_and_capture_cmds("Out.patch(Osc.saw(110))");
    assert_eq!(
        cmds,
        vec![
            saw(110.0),
            Cmd::BusWrite { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(0) },
            Cmd::SetRoot { bus: BusId(0) },
        ]
    );
}

#[test]
fn lpf_and_binop_resolve_node_inputs() {
    // Osc.saw(110).lpf(800) → node0 saw, node1 lpf(node0, 800)
    let cmds = run_and_capture_cmds("var x = Osc.saw(110).lpf(800)");
    assert_eq!(
        cmds,
        vec![
            saw(110.0),
            Cmd::NewNode {
                node: NodeId(1),
                kind: Kind::Lpf,
                args: [
                    Input::Node { node: NodeId(0), port: 0 },
                    Input::Const(800.0),
                    Input::Const(0.0)
                ],
            },
        ]
    );
}

#[test]
fn nonaudio_foreign_arg_does_not_crash() {
    // `output[2]` is `Output.new(1)`, a 4-byte non-audio foreign whose leading
    // byte (ch=1) happens to equal `TAG_PORT`. Passing it where an audio Input
    // is expected must not over-read / panic; the arg degrades to a defined
    // (inert) value and the factory still emits its `NewNode`.
    let cmds = run_and_capture_cmds("Osc.sine(output[2])");
    assert_eq!(cmds.len(), 1);
    assert!(matches!(cmds[0], Cmd::NewNode { kind: Kind::Sine, .. }));
}

#[test]
fn reset_emits_reset() {
    let cmds = run_and_capture_cmds("Out.reset()");
    assert_eq!(cmds, vec![Cmd::Reset]);
}

#[test]
fn bus_write_and_patch() {
    let cmds = run_and_capture_cmds(
        "var m = Bus.new()\n\
         m.write(Osc.saw(110))\n\
         Out.patch(m)",
    );
    assert_eq!(cmds, vec![
        saw(110.0),
        Cmd::BusWrite { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(1) },
        Cmd::SetRoot { bus: BusId(1) },
    ]);
}

#[test]
fn out_port_resolves_to_that_port() {
    // Split.new(Osc.saw(110)); consumer reads .out(1)
    let cmds = run_and_capture_cmds(
        "var s = Split.new(Osc.saw(110))\n\
         var c = s.out(1) * 2",
    );
    // node0 saw, node1 split2(node0), node2 mul(node1.port1, 2)
    assert!(cmds.iter().any(|c| *c == Cmd::NewNode {
        node: NodeId(2), kind: Kind::Mul,
        args: [Input::Node { node: NodeId(1), port: 1 }, Input::Const(2.0), Input::Const(0.0)],
    }));
}

#[test]
fn free_emits_free_and_reuses_id() {
    // Allocate n0, free it, allocate again → id 0 reused.
    let cmds = run_and_capture_cmds(
        "var a = Osc.saw(110)\n\
         a.free()\n\
         var b = Osc.saw(220)",
    );
    assert_eq!(cmds, vec![
        Cmd::NewNode { node: NodeId(0), kind: Kind::Saw, args: [Input::Const(110.0), Input::Const(0.0), Input::Const(0.0)] },
        Cmd::Free { node: NodeId(0) },
        Cmd::NewNode { node: NodeId(0), kind: Kind::Saw, args: [Input::Const(220.0), Input::Const(0.0), Input::Const(0.0)] },
    ]);
}

#[test]
fn golden_saw_lpf_renders_expected_block() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Osc.saw(110).lpf(800))", &mut out);
    // CHARACTERIZATION golden re-pinned 2026-07-07 after Osc band-limiting (Tasks 1-3).
    // Regenerate only on an intended, reviewed output change.
    let expected = [0.0, -0.103913695, -0.204913, -0.29383174];
    for i in 0..4 {
        assert!((out[i].l - expected[i]).abs() < 1e-6, "sample {i}: {} vs {}", out[i].l, expected[i]);
    }
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
}

#[test]
fn ports_and_buses_render_finite() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var m = Bus.new()\n\
         var s = Split.new(Osc.saw(110))\n\
         m.write(s.out(0))\n\
         m.write(s.out(1))\n\
         Out.patch(m)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 2.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn midi_synth_example_parses_and_runs() {
    let src = include_str!("../../../wren-firmware/examples/midi_synth.wren");
    // Runs without a VM error (may reference Midi/output — that's fine, those
    // bindings exist); we only assert it boots + executes top-level code.
    let _ = run_and_capture_cmds(src);
}

#[test]
fn osc_width_emits_setinput_port2() {
    let cmds = run_and_capture_cmds("var s = Osc.square(110)\ns.width = 0.3");
    assert!(cmds.iter().any(|c| *c == Cmd::SetInput {
        node: NodeId(0), port: 2, src: Input::Const(0.3)
    }));
}

#[test]
fn osc_pm_emits_setinput_port1() {
    let cmds = run_and_capture_cmds("var s = Osc.sine(440)\ns.pm = 0.5");
    assert!(cmds.iter().any(|c| *c == Cmd::SetInput {
        node: NodeId(0), port: 1, src: Input::Const(0.5)
    }));
}

#[test]
fn osc_feedback_emits_setparam() {
    let cmds = run_and_capture_cmds("var s = Osc.sine(440)\ns.feedback = 0.8");
    assert!(cmds.iter().any(|c| *c == Cmd::SetParam {
        node: NodeId(0), param: 0, value: 0.8
    }));
}

#[test]
fn osc_wavetable_emits_newnode_and_bindtable() {
    let cmds = run_and_capture_cmds("var s = Osc.wavetable(WT.Saw, 220)");
    assert!(cmds.iter().any(|c| matches!(c,
        Cmd::NewNode { node: NodeId(0), kind: Kind::Wavetable, .. })));
    assert!(cmds.iter().any(|c| matches!(c,
        Cmd::BindTable { node: NodeId(0), src: TableSrc::Static(id) } if id.0 == 0)));
    // WT.Saw == id 0 — Saw is index 0 in the generated `TABLES` (Task 3).
}

#[test]
fn osc_wavetable_renders_finite_nonsilent() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Osc.wavetable(WT.Saw, 220))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn engine_host_upload_table_builds_band_limited() {
    let mut host = EngineHost::new(48_000.0);
    let mut base = [0.0f32; mipgen::N];
    for (i, s) in base.iter_mut().enumerate() {
        *s = 2.0 * (i as f32 / mipgen::N as f32) - 1.0;
    }
    let h = host.upload_table(&base).expect("upload");
    // level 0 region round-trips to a saw-ish shape; deeper levels are band-limited.
    let region = host.engine().pool_slice(h);
    assert_eq!(region.len(), mipgen::N * mipgen::LEVELS);
    assert!(region[..mipgen::N].iter().any(|&x| x != 0.0));
}
